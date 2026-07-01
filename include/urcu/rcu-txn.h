// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

#ifndef _URCU_RCU_TXN_H
#define _URCU_RCU_TXN_H

/*
 * Transaction front-end for the RCU MCAS engine (<urcu/rcu-mcas.h>).
 *
 * <urcu/rcu-mcas.h> is the multi-word CAS engine
 * (urcu_mcas_*): a set of {slot, old, new} records committed atomically.
 * This header wraps it in a begin / store / commit / end transaction whose state
 * is a small on-stack handle, so a mutator reads and buffers writes imperatively
 * while the engine handles the retry bookkeeping.  Loads inside the bracket go
 * through urcu_txn_load(): it forwards to urcu_mcas_read() (there is
 * no read-set) but keeps in-bracket reads routed through the handle.  begin/end
 * mark the scope, and only writes are buffered:
 *
 *     struct urcu_txn_domain domain;   // once, shared per structure
 *     urcu_txn_domain_init(&domain);
 *     ...
 *     struct urcu_mcas_txn txn;
 *     enum urcu_txn_status st;
 *
 *     urcu_txn_init(&txn, &domain);    // or NULL: no fallback
 *     do {
 *         urcu_txn_begin(&txn);
 *         // last arg TAG: the slot's proxy-tag bits (e.g. URCU_MCAS_TAG for a
 *         // bit-0 embedder); a parked record's slot value is (record | TAG).
 *         succ = urcu_txn_load(&txn, (void **) &pos->next, TAG);
 *         if (is_marked(succ)) { urcu_txn_end(&txn); return -ENOENT; }
 *         urcu_txn_store(&txn, (void **) &pos->next, succ, newp, TAG);
 *         urcu_txn_store(&txn, (void **) &succ->prev, pos,  newp, TAG);
 *         st = urcu_txn_commit(&txn);
 *         urcu_txn_end(&txn);
 *     } while (st == URCU_TXN_STATUS_ABORT);  // ABORT (>0) = retry;
 *                                                  // OK (0) = committed;
 *                                                  // MEMORY_ERROR (<0) = error
 *
 * Consistency model
 * -----------------
 * A committed transaction is linearizable, and its linearization point is the
 * single status-word commit (UNDECIDED -> SUCCEEDED): every record it parked
 * resolves to its OLD value before that CAS and to its NEW value after it, so one
 * store switches the whole frozen record set at once.  Install (proxies parked,
 * still OLD-visible) and settle (proxies rewritten to plain NEW, already
 * NEW-visible) change no observable value, so neither is a linearization point.
 *
 * Because the record set may span data structures, that one commit is a single
 * linearization point ACROSS ALL of them: everything folded into one transaction
 * (via the *_prepare forms -- e.g. publish a node into a trie AND splice it into
 * a list) becomes visible together.  This is strictly stronger than composing
 * independent RCU structures, where a node can be reachable in A before it is
 * reachable in B.  The unit of cross-structure atomicity is exactly "one
 * transaction": two separate commits to A and B are two linearization points,
 * the same as two unrelated RCU structures.
 *
 * This linearizes the WRITE.  A reader is NOT a transaction -- it is a sequence
 * of single-slot reads, each linearizing at its own access -- so a long traversal
 * may straddle a commit: it can observe the transaction in a slot it reads after
 * the commit and not in one it read before.  No slot and no instant is ever torn
 * (that is the guarantee); a reader's several reads simply are not a mutual
 * snapshot.  This is deliberately weaker than STM opacity: readers are plain RCU
 * readers that pay no per-read barrier.  An embedder needing a multi-read
 * snapshot layers its own versioning on top, as with any value-based MCAS.
 *
 * Preconditions for all of the above:
 *   - every write to a transacted slot goes through this layer (the engine owns
 *     tag bit 0; a side-channel store to such a slot breaks atomicity);
 *   - every read of a transacted slot resolves through the engine accessor
 *     (proxy -> status), never the raw word;
 *   - a node's payload is initialized before the commit that links it -- commit
 *     is the release edge, so build-then-commit publishes it safely.
 *
 * The handle holds only what the engine does not: the cross-attempt retry count
 * (aging priority) and a pointer to this attempt's descriptor -- the write-set
 * itself lives once, in the engine descriptor, not in a second buffer.  The
 * descriptor is allocated lazily on the first store and grown as needed, so the
 * write-set is dynamically sized (no fixed cap) and an attempt that reads and
 * bails before storing allocates nothing.
 *
 * Retry / aging.  The handle carries a retry count, advanced on each contention
 * abort and threaded into the descriptor as the aging priority, so a starved
 * transaction climbs in priority without the caller threading anything.
 *
 * Writes.  urcu_txn_store() buffers a write whose old the caller
 * supplies.  Only writes are committed and validated: the MCAS install checks
 * each slot == old, which covers every read that became a write-old (read
 * subset of write for structural mutations).  There is no read-set.  A store can
 * fail to allocate; rather than make the caller check every store, the failure
 * is sticky -- the pending commit then reports -ENOMEM -- so a mutator only has
 * to test commit's result (which it already does).
 *
 * Reserve.  A mutator that knows its edge count up front may call
 * urcu_txn_reserve() right after begin: it allocates the descriptor to
 * that floor, so an OOM is reported before the mutator builds any nodes, and
 * later attempts start pre-sized rather than growing into it.  Optional -- store()
 * allocates lazily and grows on its own without it.
 *
 * Escalation fallback.  The optimistic retry above is bounded-blocking but not
 * starvation-free: a large or repeatedly-bypassed transaction can be
 * defeated by a stream of smaller ones (the single-edge fast path and the
 * read->install window let a committer change a footprint slot between this
 * op's read and its install).  When a handle crosses a threshold it
 * escalates into a per-domain fair mutex (urcu/fair-mutex.h) -- an
 * MCS-style lock -- and publishes domain->active so every *future*
 * transaction funnels through the same lane.  That closes the
 * optimistic-writer set: the escalated op then contends only with the
 * finite in-flight set (bounded by thread count) and commits within a
 * bounded number of retries while holding its turn -- progress is
 * guaranteed with no quiescence (no synchronize_rcu).  The lane only
 * serializes *who pushes with top priority*; commits still go through the
 * concurrency-safe MCAS path, so the residual in-flight optimistic writers
 * stay correct.  Two triggers escalate a handle (both gated on a non-NULL
 * domain -- NULL never escalates):
 *   - retry >= URCU_TXN_FALLBACK : a starved op, reactively;
 *   - size  >= URCU_TXN_BIG      : a large op, proactively -- a
 *     reserve(n >= BIG) escalates immediately, before building any nodes,
 *     and a handle whose realized write-set reached BIG escalates on its
 *     next attempt.
 * A handle keeps its turn across aborts (retry in place -- releasing would
 * forfeit the guaranteed turn) and releases it only on a terminal outcome
 * (commit, error, or a bail that ends the bracket); the last holder out
 * clears domain->active and the domain reverts to the optimistic regime.
 *
 * RCU.  The bracket opens an RCU read-side section per attempt, and commit
 * uses the flavor's call_rcu, so include this header AFTER an RCU flavor
 * header (e.g. <urcu-qsbr.h>); register threads and pass through quiescent
 * states as usual.
 */

#include <errno.h>

#include <urcu/compiler.h>
#include <urcu/fair-mutex.h>
#include <urcu/flavor.h>		/* struct rcu_flavor_struct */
#include <urcu/rcu-mcas.h>
#include <urcu/rcu-txn-status.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Default capacity the descriptor is created with on the first store of an
 * attempt, when the handle carries no floor (txn->min_alloc == 0); it grows
 * (doubling) from there, so this only sets the no-realloc fast path for small
 * transactions.
 */
#ifndef URCU_TXN_INIT
#define URCU_TXN_INIT	4
#endif

/*
 * Escalation thresholds (override before include).  A handle escalates into the
 * domain's lock when its retry count reaches URCU_TXN_FALLBACK
 * (reactive: a starved op) or its write-set size reaches URCU_TXN_BIG
 * (proactive: a large op, e.g. a wide merge).  FALLBACK sits well above the
 * engine's single-edge URCU_MCAS_ESCALATE (16) so ordinary contention rides the
 * optimistic path: escalating too early funnels every contending writer into the
 * one serial lane, which under a shared hot domain collapses both throughput and
 * latency far worse than leaving the optimistic priority protocol to resolve it.
 * BIG should sit above typical small-mutation edge counts so only genuinely large
 * transactions take the lane up front.
 */
#ifndef URCU_TXN_FALLBACK
#define URCU_TXN_FALLBACK	256
#endif
#ifndef URCU_TXN_BIG
#define URCU_TXN_BIG		128
#endif

/*
 * Compile-time fallback for the RCU read-side bracket, used by
 * urcu_txn_read_lock()/urcu_txn_read_unlock() (hence begin()/end()) ONLY when a
 * handle binds no flavor (urcu_txn_init's NULL).  Defaults to the
 * compile-time-selected flavor's rcu_read_lock / rcu_read_unlock -- hence the
 * "include after an RCU flavor header" rule -- and may be overridden before
 * include.  The PREFERRED path for a FLAVOR-AGNOSTIC embedder (one selecting its
 * RCU flavor at runtime) is to bind that flavor with urcu_txn_init_flavor() so
 * the bracket opens in it directly; this macro then never fires.
 */
#ifndef URCU_TXN_RCU_READ_LOCK
#define URCU_TXN_RCU_READ_LOCK()	rcu_read_lock()
#endif
#ifndef URCU_TXN_RCU_READ_UNLOCK
#define URCU_TXN_RCU_READ_UNLOCK()	rcu_read_unlock()
#endif

/*
 * Sticky out-of-memory marker parked in txn->mcas by a failed store: distinct
 * from NULL (no write buffered yet) and from any real descriptor, so commit can
 * tell "nothing to do" from "a store could not allocate".
 */
#define URCU_TXN_ENOMEM	((struct urcu_mcas *) -1L)

/*
 * Per-contention-domain escalation state, shared by every handle that transacts
 * the same structure.  Pass &domain to urcu_txn_init(), or NULL to
 * disable the fallback (pure optimistic retry).
 */
struct urcu_txn_domain {
	struct cds_fair_mutex lock;	/* the fair escalation lane */
	unsigned long active;		/* a fallback episode is in progress */
};

static inline
void urcu_txn_domain_init(struct urcu_txn_domain *d)
{
	cds_fair_mutex_init(&d->lock);
	d->active = 0;
}

struct urcu_mcas_txn {
	struct urcu_txn_domain *domain;	/* escalation domain, or NULL */
	const struct rcu_flavor_struct *flavor;	/* RCU flavor for the read-side
						 * bracket, or NULL to use the
						 * compile-time-selected flavor
						 * (URCU_TXN_RCU_READ_LOCK).  Set at
						 * create by urcu_txn_init_flavor();
						 * lets a FLAVOR-AGNOSTIC embedder
						 * (one selecting its flavor at
						 * runtime, e.g. the fractal trie)
						 * bracket the txn in its own flavor's
						 * read-side section. */
	unsigned long retry;		/* attempts so far; aging priority for the MCAS */
	unsigned int min_alloc;		/* floor for the attempt's initial descriptor
					 * capacity, grown past if exceeded (0 -> INIT
					 * default).  Set via reserve(); refreshed to the
					 * realized size at commit so retries don't re-grow. */
	struct urcu_mcas *mcas;	/* this attempt's descriptor: NULL (none yet),
					 * a live descriptor, or the ENOMEM marker */
	struct cds_fair_mutex_node waiter;	/* our node while awaiting the turn */
	int in_fallback;		/* we currently hold the lock */
	int retrying;			/* commit asked retry: keep the turn */
};

/*
 * Initialize a handle before its retry loop (retry := 0, no reservation),
 * bracketing the txn's RCU read-side section in @flavor's read_lock/read_unlock.
 * Pass @flavor NULL to use the compile-time-selected flavor (the
 * URCU_TXN_RCU_READ_LOCK default) -- the plain urcu_txn_init() does exactly
 * this.  A flavor-agnostic embedder (one selecting its RCU flavor at runtime
 * through a rcu_flavor_struct vtable, e.g. the fractal trie) passes that flavor
 * here so begin()/end() -- and the standalone urcu_txn_read_lock() /
 * urcu_txn_read_unlock() bracket -- open the read-side section in it, the same
 * way commit_flavor() defers reclaim through flavor->update_call_rcu.
 */
static inline
void urcu_txn_init_flavor(struct urcu_mcas_txn *txn,
		struct urcu_txn_domain *domain,
		const struct rcu_flavor_struct *flavor)
{
	txn->domain = domain;
	txn->flavor = flavor;
	txn->retry = 0;
	txn->min_alloc = 0;
	txn->mcas = NULL;
	txn->in_fallback = 0;
	txn->retrying = 0;
}

/* Initialize a handle bracketed in the compile-time-selected RCU flavor. */
static inline
void urcu_txn_init(struct urcu_mcas_txn *txn,
		struct urcu_txn_domain *domain)
{
	urcu_txn_init_flavor(txn, domain, NULL);
}

/*
 * Open / close the txn's RCU read-side section through its bound flavor (set by
 * urcu_txn_init_flavor), falling back to the compile-time-selected flavor when
 * none is bound.  begin()/end() bracket with these; an embedder that drives the
 * engine WITHOUT begin()/end() (only init/reserve/store/commit_flavor) calls
 * them directly to bracket the whole mutation -- the read-side section is what
 * keeps a parked record (or a helped foreign descriptor) alive across the
 * commit.
 */
static inline
void urcu_txn_read_lock(struct urcu_mcas_txn *txn)
{
	if (txn->flavor)
		txn->flavor->read_lock();
	else
		URCU_TXN_RCU_READ_LOCK();
}

static inline
void urcu_txn_read_unlock(struct urcu_mcas_txn *txn)
{
	if (txn->flavor)
		txn->flavor->read_unlock();
	else
		URCU_TXN_RCU_READ_UNLOCK();
}

/*
 * Take the domain's lock (blocks until we are the head) and publish that a
 * fallback episode is in progress, so future transactions funnel into the lane.
 * Caller must NOT hold the RCU read-side section: cds_fair_mutex_lock may block.
 */
static inline
void urcu_txn__enter_fallback(struct urcu_mcas_txn *txn)
{
	/*
	 * cds_fair_mutex_lock may park on a futex until our turn.  We hold the
	 * caller's RCU read-side section across that wait rather than going
	 * RCU-offline: that section is what keeps the caller's input pointers
	 * (e.g. a list anchor reached by key) alive for the whole transaction,
	 * and dropping it would let a concurrent grace period free them out from
	 * under us.  (Going offline is a QSBR-only move anyway -- the bracketing
	 * flavors hold a nested read-side lock the callee cannot release.)  This
	 * is safe because the wait is bounded: the FIFO turn is short and the
	 * lane owner's commit is a bounded MCAS that defers reclaim through
	 * call_rcu and never itself waits on a grace period, so holding the
	 * section across it cannot extend a grace period unboundedly.  That "the
	 * lane owner never blocks on a GP while holding the mutex" is the one
	 * invariant this relies on.
	 */
	cds_fair_mutex_lock(&txn->domain->lock, &txn->waiter);
	uatomic_store(&txn->domain->active, 1, CMM_RELAXED);
	txn->in_fallback = 1;
}

/*
 * Release the lock; if we were the last holder, end the episode by
 * clearing domain->active so new transactions resume the optimistic path.
 */
static inline
void urcu_txn__exit_fallback(struct urcu_mcas_txn *txn)
{
	if (cds_fair_mutex_unlock(&txn->domain->lock, &txn->waiter))
		uatomic_store(&txn->domain->active, 0, CMM_RELAXED);
	txn->in_fallback = 0;
}

/*
 * Whether this attempt should escalate into the lock before opening: a
 * starved (retry) or already-known large (min_alloc) handle initiates an
 * episode, and domain->active funnels every other handle into the same lane
 * for the episode's duration -- that funnelling is what closes the optimistic-
 * writer set and bounds the escalated op's progress.  domain->active is
 * advisory: a stale read only mis-routes one bounded attempt (the MCAS commit
 * is correct under the resulting concurrency), so it needs no acquire/release,
 * only atomicity.
 */
static inline
int urcu_txn__want_fallback(struct urcu_mcas_txn *txn)
{
	return txn->domain && !txn->in_fallback &&
		(uatomic_load(&txn->domain->active, CMM_RELAXED) ||
		 txn->retry >= URCU_TXN_FALLBACK ||
		 txn->min_alloc >= URCU_TXN_BIG);
}

/* Begin one attempt: clear the write-set and open the RCU read-side section. */
static inline
void urcu_txn_begin(struct urcu_mcas_txn *txn)
{
	txn->retrying = 0;
	/*
	 * Escalate before opening the attempt: a starved (retry) or
	 * already-known large (min_alloc) handle takes its FIFO turn here.
	 * cds_fair_mutex_lock may block, so it must run outside the RCU read-side
	 * section.
	 */
	if (urcu_txn__want_fallback(txn))
		urcu_txn__enter_fallback(txn);
	txn->mcas = NULL;		/* prior attempt's descriptor already consumed/freed */
	urcu_txn_read_lock(txn);
}

/*
 * Pre-reserve room for @n writes this attempt and record @n as the min_alloc
 * floor.  Allocates the descriptor up front, so an algorithm that knows its edge
 * count can fail early with -ENOMEM -- before building the nodes it meant to
 * link -- rather than discovering the failure partway through its stores.  The
 * floor also sizes every later attempt's initial descriptor, so retries start at
 * @n instead of growing into it.  Returns 0, or -ENOMEM (sticky: the pending
 * commit then also returns -ENOMEM).  Optional; call after begin, before the
 * first store.  store() still grows the descriptor should the write-set exceed @n.
 */
static inline
int urcu_txn_reserve(struct urcu_mcas_txn *txn, unsigned int n)
{
	struct urcu_mcas *m;

	txn->min_alloc = n;
	/*
	 * A large op declares its size here: escalate immediately, before
	 * building any nodes, so it never runs a disruptive optimistic attempt.
	 * We hold the read-side section across the (possibly blocking) FIFO enter
	 * (see __enter_fallback): the bounded wait keeps the caller's pinned
	 * pointers alive and cannot extend a grace period unboundedly.
	 */
	if (txn->domain && !txn->in_fallback && n >= URCU_TXN_BIG)
		urcu_txn__enter_fallback(txn);
	if (caa_unlikely(txn->mcas == URCU_TXN_ENOMEM))
		return -ENOMEM;		/* sticky: an earlier alloc already failed */
	if (!n)
		return 0;		/* no reservation; the INIT default applies */
	if (!txn->mcas) {
		m = urcu_mcas_create(n, txn->retry);
		if (caa_unlikely(!m)) {
			txn->mcas = URCU_TXN_ENOMEM;
			return -ENOMEM;
		}
		txn->mcas = m;
		return 0;
	}
	while (txn->mcas->cap < n) {	/* already buffering: grow to fit @n */
		m = urcu_mcas_grow(txn->mcas);
		if (caa_unlikely(!m)) {
			urcu_mcas_destroy(txn->mcas);	/* unpublished: sync free */
			txn->mcas = URCU_TXN_ENOMEM;
			return -ENOMEM;
		}
		txn->mcas = m;
	}
	return 0;
}

/*
 * Buffer or reconcile one record (see urcu_mcas_record): lazily create
 * the descriptor, grow it if full, and keep one record per slot.  @upgrade is 1
 * for a store (advances new_ptr), or 0 for a load-validate guard (keeps any
 * pending write).  Returns 0, or -ENOMEM (sticky -- the pending commit returns
 * it).
 */
static inline
int urcu_txn__record(struct urcu_mcas_txn *txn, void **slot,
		void *old_ptr, void *new_ptr, int upgrade, uintptr_t tag)
{
	struct urcu_mcas *m = txn->mcas;

	if (caa_unlikely(m == URCU_TXN_ENOMEM))
		return -ENOMEM;		/* sticky: an earlier record already failed */
	if (!m) {
		m = urcu_mcas_create(txn->min_alloc ? txn->min_alloc :
				URCU_TXN_INIT, txn->retry);
		if (caa_unlikely(!m)) {
			txn->mcas = URCU_TXN_ENOMEM;
			return -ENOMEM;
		}
		txn->mcas = m;
	}
	if (caa_unlikely(!urcu_mcas_record(m, slot, old_ptr, new_ptr,
			upgrade, tag))) {
		/* Descriptor full: grow (may move it) and retry. */
		m = urcu_mcas_grow(m);
		if (caa_unlikely(!m)) {
			urcu_mcas_destroy(txn->mcas);	/* unpublished: sync free */
			txn->mcas = URCU_TXN_ENOMEM;
			return -ENOMEM;
		}
		txn->mcas = m;
		urcu_mcas_record(m, slot, old_ptr, new_ptr, upgrade, tag);
	}
	return 0;
}

/*
 * Read @slot within the bracket and return its current logical value -- the old
 * for a word this attempt intends to transact.  Forwards to urcu_mcas_read():
 * there is no read-set, so the read is not recorded; commit reconciles it
 * through the slot == old check on whatever store consumes it (read subset of
 * write).  @txn is taken regardless -- it binds the read to the bracket's RCU
 * read-side section structurally (a live handle exists only between begin and
 * end), and it is the seam where the wait-free escalation lane would add read
 * validation: route in-bracket reads here, not through urcu_mcas_read(), so
 * that day is a one-line change.  A plain observer outside any transaction reads
 * with urcu_mcas_read() directly.
 */
static inline
void *urcu_txn_load(struct urcu_mcas_txn *txn, void **slot, uintptr_t tag)
{
	(void) txn;			/* no read-set today -- this is the seam */
	return urcu_mcas_read(slot, tag);
}

/*
 * Read @slot like urcu_txn_load AND pin it: besides returning its
 * current logical value, record a load-only guard so the commit succeeds only
 * if @slot still resolves to that value at the install point (records {v -> v})
 * -- a TM read-set entry folding a read into the commit's conflict set, for a
 * word the op depends on but does not rewrite (e.g. a tombstone an insert must
 * find clear).  This is value-CAS semantics: the guard checks that @slot resolves
 * to that value AT the linearization point, not that it stayed unchanged
 * throughout.  The engine is A-B-A-safe (the install latch tolerates any
 * slot-value recurrence -- no use-after-free), so a value that recurs benignly is
 * fine; but if the op's correctness needs to DETECT an intervening change (a true
 * A-B-A where the "B" matters -- the slot toggled away and back), the guard alone
 * will not see it, and the embedder must carry its own version/generation in the
 * word.  A later store to @slot upgrades the guard to a write in place;
 * validate/read a given slot once per attempt.  Sticky on OOM like store: the
 * value is returned regardless and the pending commit reports -ENOMEM.
 */
static inline
void *urcu_txn_load_validate(struct urcu_mcas_txn *txn, void **slot,
		uintptr_t tag)
{
	void *v = urcu_mcas_read(slot, tag);

	(void) urcu_txn__record(txn, slot, v, v, 0, tag);
	return v;
}

/*
 * Buffer a write {*slot: old -> new}.  @old_ptr is the value the caller saw.
 * If @slot already carries a record (a prior store, or a load-validate guard),
 * the write upgrades it in place rather than adding a second -- one record per
 * slot.  Returns 0, or -ENOMEM if the descriptor could not be allocated or
 * grown -- sticky, so the pending commit also returns -ENOMEM and the caller
 * may test only commit.
 */
static inline
int urcu_txn_store(struct urcu_mcas_txn *txn, void **slot,
		void *old_ptr, void *new_ptr, uintptr_t tag)
{
	return urcu_txn__record(txn, slot, old_ptr, new_ptr, 1, tag);
}

/*
 * Commit the buffered write-set through the MCAS, deferring reclaim through
 * @call_rcu_fn.  Returns enum urcu_txn_status: OK on commit, ABORT on a
 * contention abort (the caller re-runs begin..commit; the retry count is
 * advanced internally), or MEMORY_ERROR on allocation failure (including a store
 * that could not allocate).  @call_rcu_fn has the flavor call_rcu signature, so
 * a flavor-agnostic embedder passes its RCU flavor's call_rcu directly (e.g.
 * flavor->update_call_rcu) -- the same parametric-reclaim contract as
 * <urcu/rcu-txn-sw.h>, so an embedder migrating from the single-updater
 * front-end keeps its reclaim wiring.  The convenience wrapper urcu_txn_commit()
 * passes the compile-time-selected call_rcu.  Call between begin and end.
 */
static inline
enum urcu_txn_status urcu_txn_commit_flavor(struct urcu_mcas_txn *txn,
		void (*call_rcu_fn)(struct rcu_head *,
			void (*)(struct rcu_head *)))
{
	struct urcu_mcas *m = txn->mcas;

	if (caa_unlikely(m == URCU_TXN_ENOMEM)) {
		txn->mcas = NULL;
		return URCU_TXN_STATUS_MEMORY_ERROR;
	}
	if (!m)
		return URCU_TXN_STATUS_OK;	/* empty write-set: trivially committed */
	txn->min_alloc = m->nr;		/* learn the realized size: a retry won't re-grow */
	txn->mcas = NULL;		/* mcas_commit consumes the descriptor */
	if (urcu_mcas_commit(m, call_rcu_fn))
		return URCU_TXN_STATUS_OK;
	txn->retry++;			/* aged for the next attempt */
	txn->retrying = 1;		/* keep the turn across the retry */
	return URCU_TXN_STATUS_ABORT;
}

/*
 * Commit deferring reclaim through the compile-time-selected RCU flavor's
 * call_rcu (so this header must be included after an RCU flavor header).  A thin
 * wrapper over urcu_txn_commit_flavor(); see it for the full contract.
 */
static inline
enum urcu_txn_status urcu_txn_commit(struct urcu_mcas_txn *txn)
{
	return urcu_txn_commit_flavor(txn, call_rcu);
}

/*
 * Note a contention retry that abandons the attempt BEFORE commit -- e.g. a
 * load-validate guard observed a neighbour mid-deletion and the mutator must
 * re-read rather than commit.  Advances aging and keeps the FIFO turn exactly as
 * a commit ABORT does, so such a guard-driven retry escalates into the fallback
 * lane instead of spinning: without this, an op that keeps hitting the guard on a
 * hot slot never reaches commit, so txn->retry never advances and it can livelock.
 * Call after the guard fires and before end(), then end()+begin() and re-attempt.
 */
static inline
void urcu_txn_conflict(struct urcu_mcas_txn *txn)
{
	txn->retry++;			/* aged: a guard storm now escalates */
	txn->retrying = 1;		/* keep the FIFO turn across the retry */
}

/* End the attempt: close the RCU read-side section.  Always pair with begin. */
static inline
void urcu_txn_end(struct urcu_mcas_txn *txn)
{
	struct urcu_mcas *m = txn->mcas;

	/*
	 * A live descriptor survives to here only when the attempt buffered
	 * stores but bailed before commit; it was never parked, so free it
	 * synchronously.  After commit, mcas is NULL (consumed); after a store
	 * OOM it is the marker -- neither needs freeing.
	 */
	if (m && m != URCU_TXN_ENOMEM)
		urcu_mcas_destroy(m);
	txn->mcas = NULL;
	urcu_txn_read_unlock(txn);
	/*
	 * Release the FIFO turn on a terminal outcome (commit, error, or a
	 * bail that ends the bracket).  On a retry (commit returned 0 ->
	 * retrying) keep the turn and re-attempt as the same head.
	 */
	if (txn->in_fallback && !txn->retrying)
		urcu_txn__exit_fallback(txn);
}

#ifdef __cplusplus
}
#endif

#endif	/* _URCU_RCU_TXN_H */
