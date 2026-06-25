// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

#ifndef _URCU_FLIP_LATCH_TXN_LOCKFREE_H
#define _URCU_FLIP_LATCH_TXN_LOCKFREE_H

/*
 * Transaction front-end for the lock-free flip-latch.
 *
 * <urcu/flip-latch-lockfree.h> is the multi-word CAS engine
 * (urcu_flip_lf_mcas_*): a set of {slot, old, new} records committed atomically.
 * This header wraps it in a begin / store / commit / end transaction whose state
 * is a small on-stack handle, so a mutator reads and buffers writes imperatively
 * while the engine handles the retry bookkeeping.  Loads inside the bracket go
 * through urcu_flip_lf_txn_load(): it forwards to urcu_flip_lf_read() (there is
 * no read-set) but keeps in-bracket reads routed through the handle.  begin/end
 * mark the scope, and only writes are buffered:
 *
 *     struct urcu_flip_lf_txn_domain domain;   // once, shared per structure
 *     urcu_flip_lf_txn_domain_init(&domain);
 *     ...
 *     struct urcu_flip_lf_txn txn;
 *     int ret;
 *
 *     urcu_flip_lf_txn_init(&txn, &domain);    // or NULL: no fallback
 *     do {
 *         urcu_flip_lf_txn_begin(&txn);
 *         succ = urcu_flip_lf_txn_load(&txn, (void **) &pos->next);
 *         if (is_marked(succ)) { urcu_flip_lf_txn_end(&txn); return -ENOENT; }
 *         urcu_flip_lf_txn_store(&txn, (void **) &pos->next, succ, newp);
 *         urcu_flip_lf_txn_store(&txn, (void **) &succ->prev, pos,  newp);
 *         ret = urcu_flip_lf_txn_commit(&txn);
 *         urcu_flip_lf_txn_end(&txn);
 *     } while (ret == 0);    // 0 = retry, 1 = committed, <0 = error
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
 * Writes.  urcu_flip_lf_txn_store() buffers a write whose old the caller
 * supplies.  Only writes are committed and validated: the MCAS install checks
 * each slot == old, which covers every read that became a write-old (read
 * subset of write for structural mutations).  There is no read-set.  A store can
 * fail to allocate; rather than make the caller check every store, the failure
 * is sticky -- the pending commit then reports -ENOMEM -- so a mutator only has
 * to test commit's result (which it already does).
 *
 * Reserve.  A mutator that knows its edge count up front may call
 * urcu_flip_lf_txn_reserve() right after begin: it allocates the descriptor to
 * that floor, so an OOM is reported before the mutator builds any nodes, and
 * later attempts start pre-sized rather than growing into it.  Optional -- store()
 * allocates lazily and grows on its own without it.
 *
 * Escalation fallback.  The optimistic retry above is lock-free but not
 * starvation-free: a large or repeatedly-bypassed transaction can be
 * defeated by a stream of smaller ones (the single-edge fast path and the
 * read->install window let a committer change a footprint slot between this
 * op's read and its install).  When a handle crosses a threshold it
 * escalates into a per-domain FIFO turnstile (urcu/fifo.h) -- a fair,
 * lock-free handoff -- and publishes domain->active so every *future*
 * transaction funnels through the same lane.  That closes the
 * optimistic-writer set: the escalated op then contends only with the
 * finite in-flight set (bounded by thread count) and commits within a
 * bounded number of retries while holding its turn -- progress is
 * guaranteed with no quiescence (no synchronize_rcu).  The lane only
 * serializes *who pushes with top priority*; commits still go through the
 * concurrency-safe MCAS path, so the residual in-flight optimistic writers
 * stay correct.  Two triggers escalate a handle (both gated on a non-NULL
 * domain -- NULL never escalates):
 *   - retry >= URCU_FLIP_LF_TXN_FALLBACK : a starved op, reactively;
 *   - size  >= URCU_FLIP_LF_TXN_BIG      : a large op, proactively -- a
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
#include <urcu/fifo.h>
#include <urcu/flip-latch-lockfree.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Default capacity the descriptor is created with on the first store of an
 * attempt, when the handle carries no floor (txn->min_alloc == 0); it grows
 * (doubling) from there, so this only sets the no-realloc fast path for small
 * transactions.
 */
#ifndef URCU_FLIP_LF_TXN_INIT
#define URCU_FLIP_LF_TXN_INIT	4
#endif

/*
 * Escalation thresholds (override before include).  A handle escalates into the
 * domain's FIFO lane when its retry count reaches URCU_FLIP_LF_TXN_FALLBACK
 * (reactive: a starved op) or its write-set size reaches URCU_FLIP_LF_TXN_BIG
 * (proactive: a large op, e.g. a wide merge).  FALLBACK sits well above the
 * engine's single-edge URCU_FLIP_LF_ESCALATE so ordinary contention rides the
 * optimistic path; BIG should sit above typical small-mutation edge counts so
 * only genuinely large transactions take the lane up front.
 */
#ifndef URCU_FLIP_LF_TXN_FALLBACK
#define URCU_FLIP_LF_TXN_FALLBACK	64
#endif
#ifndef URCU_FLIP_LF_TXN_BIG
#define URCU_FLIP_LF_TXN_BIG		128
#endif

/*
 * Sticky out-of-memory marker parked in txn->mcas by a failed store: distinct
 * from NULL (no write buffered yet) and from any real descriptor, so commit can
 * tell "nothing to do" from "a store could not allocate".
 */
#define URCU_FLIP_LF_TXN_ENOMEM	((struct urcu_flip_lf_mcas *) -1L)

/*
 * Per-contention-domain escalation state, shared by every handle that transacts
 * the same structure.  Pass &domain to urcu_flip_lf_txn_init(), or NULL to
 * disable the fallback (pure optimistic retry).
 */
struct urcu_flip_lf_txn_domain {
	struct cds_fifo_turnstile fifo;	/* the fair escalation lane */
	unsigned long active;		/* a fallback episode is in progress */
};

static inline
void urcu_flip_lf_txn_domain_init(struct urcu_flip_lf_txn_domain *d)
{
	cds_fifo_turnstile_init(&d->fifo);
	d->active = 0;
}

struct urcu_flip_lf_txn {
	struct urcu_flip_lf_txn_domain *domain;	/* escalation domain, or NULL */
	unsigned long retry;		/* attempts so far; aging priority for the MCAS */
	unsigned int min_alloc;		/* floor for the attempt's initial descriptor
					 * capacity, grown past if exceeded (0 -> INIT
					 * default).  Set via reserve(); refreshed to the
					 * realized size at commit so retries don't re-grow. */
	struct urcu_flip_lf_mcas *mcas;	/* this attempt's descriptor: NULL (none yet),
					 * a live descriptor, or the ENOMEM marker */
	struct cds_fifo_waiter waiter;	/* our node while awaiting the turn */
	int in_fallback;		/* we currently hold the FIFO turn */
	int retrying;			/* commit asked retry: keep the turn */
};

/* Initialize a handle before its retry loop (retry := 0, no reservation). */
static inline
void urcu_flip_lf_txn_init(struct urcu_flip_lf_txn *txn,
		struct urcu_flip_lf_txn_domain *domain)
{
	txn->domain = domain;
	txn->retry = 0;
	txn->min_alloc = 0;
	txn->mcas = NULL;
	txn->in_fallback = 0;
	txn->retrying = 0;
}

/*
 * Take the domain's FIFO turn (blocks until we are the head) and publish that a
 * fallback episode is in progress, so future transactions funnel into the lane.
 * Caller must NOT hold the RCU read-side section: cds_fifo_enter may block.
 */
static inline
void urcu_flip_lf_txn__enter_fallback(struct urcu_flip_lf_txn *txn)
{
	/*
	 * cds_fifo_enter may park on a futex until our turn.  A QSBR reader
	 * that blocks while online stalls grace periods -- and thus the
	 * engine's call_rcu reclaim -- for the whole wait, so go RCU-offline
	 * around it.  We are outside the bracket's read-side section here
	 * (begin escalates before rcu_read_lock; reserve unlocks first), so
	 * this is safe; other flavors implement the pair too (flavor API).
	 */
	rcu_thread_offline();
	cds_fifo_enter(&txn->domain->fifo, &txn->waiter);
	rcu_thread_online();
	uatomic_store(&txn->domain->active, 1, CMM_RELEASE);
	txn->in_fallback = 1;
}

/*
 * Release the FIFO turn; if we were the last holder, end the episode by
 * clearing domain->active so new transactions resume the optimistic path.
 */
static inline
void urcu_flip_lf_txn__exit_fallback(struct urcu_flip_lf_txn *txn)
{
	if (cds_fifo_exit(&txn->domain->fifo, &txn->waiter))
		uatomic_store(&txn->domain->active, 0, CMM_RELEASE);
	txn->in_fallback = 0;
}

/*
 * Whether this attempt should escalate into the FIFO lane before opening: a
 * starved (retry) or already-known large (min_alloc) handle initiates an
 * episode, and domain->active funnels every other handle into the same lane
 * for the episode's duration -- that funnelling is what closes the optimistic-
 * writer set and bounds the escalated op's progress.
 */
static inline
int urcu_flip_lf_txn__want_fallback(struct urcu_flip_lf_txn *txn)
{
	return txn->domain && !txn->in_fallback &&
		(uatomic_load(&txn->domain->active, CMM_ACQUIRE) ||
		 txn->retry >= URCU_FLIP_LF_TXN_FALLBACK ||
		 txn->min_alloc >= URCU_FLIP_LF_TXN_BIG);
}

/* Begin one attempt: clear the write-set and open the RCU read-side section. */
static inline
void urcu_flip_lf_txn_begin(struct urcu_flip_lf_txn *txn)
{
	txn->retrying = 0;
	/*
	 * Escalate before opening the attempt: a starved (retry) or
	 * already-known large (min_alloc) handle takes its FIFO turn here.
	 * cds_fifo_enter may block, so it must run outside the RCU read-side
	 * section.
	 */
	if (urcu_flip_lf_txn__want_fallback(txn))
		urcu_flip_lf_txn__enter_fallback(txn);
	txn->mcas = NULL;		/* prior attempt's descriptor already consumed/freed */
	rcu_read_lock();
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
int urcu_flip_lf_txn_reserve(struct urcu_flip_lf_txn *txn, unsigned int n)
{
	struct urcu_flip_lf_mcas *m;

	txn->min_alloc = n;
	/*
	 * A large op declares its size here: escalate immediately, before
	 * building any nodes, so it never runs a disruptive optimistic
	 * attempt.  We are inside the bracket's RCU read-side section but have
	 * read nothing yet, so we can step out around the (possibly blocking)
	 * FIFO enter and back in.
	 */
	if (txn->domain && !txn->in_fallback && n >= URCU_FLIP_LF_TXN_BIG) {
		rcu_read_unlock();
		urcu_flip_lf_txn__enter_fallback(txn);
		rcu_read_lock();
	}
	if (caa_unlikely(txn->mcas == URCU_FLIP_LF_TXN_ENOMEM))
		return -ENOMEM;		/* sticky: an earlier alloc already failed */
	if (!n)
		return 0;		/* no reservation; the INIT default applies */
	if (!txn->mcas) {
		m = urcu_flip_lf_mcas_create(n, txn->retry);
		if (caa_unlikely(!m)) {
			txn->mcas = URCU_FLIP_LF_TXN_ENOMEM;
			return -ENOMEM;
		}
		txn->mcas = m;
		return 0;
	}
	while (txn->mcas->cap < n) {	/* already buffering: grow to fit @n */
		m = urcu_flip_lf_mcas_grow(txn->mcas);
		if (caa_unlikely(!m)) {
			urcu_flip_lf_mcas_destroy(txn->mcas);	/* unpublished: sync free */
			txn->mcas = URCU_FLIP_LF_TXN_ENOMEM;
			return -ENOMEM;
		}
		txn->mcas = m;
	}
	return 0;
}

/*
 * Read @slot within the bracket and return its current logical value -- the old
 * for a word this attempt intends to transact.  Forwards to urcu_flip_lf_read():
 * there is no read-set, so the read is not recorded; commit reconciles it
 * through the slot == old check on whatever store consumes it (read subset of
 * write).  @txn is taken regardless -- it binds the read to the bracket's RCU
 * read-side section structurally (a live handle exists only between begin and
 * end), and it is the seam where the wait-free escalation lane would add read
 * validation: route in-bracket reads here, not through urcu_flip_lf_read(), so
 * that day is a one-line change.  A plain observer outside any transaction reads
 * with urcu_flip_lf_read() directly.
 */
static inline
void *urcu_flip_lf_txn_load(struct urcu_flip_lf_txn *txn, void **slot)
{
	(void) txn;			/* no read-set today -- this is the seam */
	return urcu_flip_lf_read(slot);
}

/*
 * Buffer a write {*slot: old -> new}.  @old_ptr is the value the caller saw.
 * Returns 0, or -ENOMEM if the descriptor could not be allocated or grown -- in
 * which case the failure is recorded so the pending commit also returns -ENOMEM;
 * the caller may therefore ignore this return and test only commit.
 */
static inline
int urcu_flip_lf_txn_store(struct urcu_flip_lf_txn *txn, void **slot,
		void *old_ptr, void *new_ptr)
{
	struct urcu_flip_lf_mcas *m = txn->mcas;

	if (caa_unlikely(m == URCU_FLIP_LF_TXN_ENOMEM))
		return -ENOMEM;		/* sticky: an earlier store already failed */
	if (!m) {
		m = urcu_flip_lf_mcas_create(txn->min_alloc ? txn->min_alloc :
				URCU_FLIP_LF_TXN_INIT, txn->retry);
		if (caa_unlikely(!m)) {
			txn->mcas = URCU_FLIP_LF_TXN_ENOMEM;
			return -ENOMEM;
		}
		txn->mcas = m;
	}
	if (caa_unlikely(!urcu_flip_lf_mcas_add(m, slot, old_ptr, new_ptr))) {
		/* Descriptor full: grow (may move it) and retry the add. */
		m = urcu_flip_lf_mcas_grow(m);
		if (caa_unlikely(!m)) {
			urcu_flip_lf_mcas_destroy(txn->mcas);	/* unpublished: sync free */
			txn->mcas = URCU_FLIP_LF_TXN_ENOMEM;
			return -ENOMEM;
		}
		txn->mcas = m;
		urcu_flip_lf_mcas_add(m, slot, old_ptr, new_ptr);	/* room now */
	}
	return 0;
}

/*
 * Commit the buffered write-set through the MCAS.  Returns 1 on commit, 0 on a
 * contention abort (the caller re-runs begin..commit; the retry count is
 * advanced internally), or a negative errno on error (-ENOMEM, including a store
 * that could not allocate).  Reclaim is deferred through the flavor's call_rcu.
 * Call between begin and end.
 */
static inline
int urcu_flip_lf_txn_commit(struct urcu_flip_lf_txn *txn)
{
	struct urcu_flip_lf_mcas *m = txn->mcas;

	if (caa_unlikely(m == URCU_FLIP_LF_TXN_ENOMEM)) {
		txn->mcas = NULL;
		return -ENOMEM;
	}
	if (!m)
		return 1;		/* empty write-set: trivially committed */
	txn->min_alloc = m->nr;		/* learn the realized size: a retry won't re-grow */
	txn->mcas = NULL;		/* mcas_commit consumes the descriptor */
	if (urcu_flip_lf_mcas_commit(m, call_rcu))
		return 1;
	txn->retry++;			/* aged for the next attempt */
	txn->retrying = 1;		/* keep the turn across the retry */
	return 0;
}

/* End the attempt: close the RCU read-side section.  Always pair with begin. */
static inline
void urcu_flip_lf_txn_end(struct urcu_flip_lf_txn *txn)
{
	struct urcu_flip_lf_mcas *m = txn->mcas;

	/*
	 * A live descriptor survives to here only when the attempt buffered
	 * stores but bailed before commit; it was never parked, so free it
	 * synchronously.  After commit, mcas is NULL (consumed); after a store
	 * OOM it is the marker -- neither needs freeing.
	 */
	if (m && m != URCU_FLIP_LF_TXN_ENOMEM)
		urcu_flip_lf_mcas_destroy(m);
	txn->mcas = NULL;
	rcu_read_unlock();
	/*
	 * Release the FIFO turn on a terminal outcome (commit, error, or a
	 * bail that ends the bracket).  On a retry (commit returned 0 ->
	 * retrying) keep the turn and re-attempt as the same head.
	 */
	if (txn->in_fallback && !txn->retrying)
		urcu_flip_lf_txn__exit_fallback(txn);
}

#ifdef __cplusplus
}
#endif

#endif	/* _URCU_FLIP_LATCH_TXN_LOCKFREE_H */
