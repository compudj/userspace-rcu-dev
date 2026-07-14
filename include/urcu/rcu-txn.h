// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

#ifndef _URCU_RCU_TXN_H
#define _URCU_RCU_TXN_H

/*
 * Transaction front-end for the RCU MCAS engine (<urcu/rcu-mcas.h>).
 *
 * <urcu/rcu-mcas.h> is the multi-word CAS engine (urcu_mcas_*): a set of {slot,
 * old, new} records committed atomically.  This header wraps it in a begin /
 * store / commit / end transaction whose state is a small on-stack handle, so a
 * mutator reads and buffers writes imperatively while the engine handles the
 * retry bookkeeping.  Loads inside the bracket go through urcu_txn_load(): it
 * forwards to urcu_mcas_read() (there is no read-set) but keeps in-bracket
 * reads routed through the handle.  begin/end mark the scope, and only writes
 * are buffered:
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
 * resolves to its OLD value before that CAS and to its NEW value after it, so
 * one store switches the whole frozen record set at once.  Install (proxies
 * parked, still OLD-visible) and settle (proxies rewritten to plain NEW,
 * already NEW-visible) change no observable value, so neither is a
 * linearization point.
 *
 * Because the record set may span data structures, that one commit is a single
 * linearization point ACROSS ALL of them: everything folded into one
 * transaction (via the *_prepare forms -- e.g. publish a node into a trie AND
 * splice it into a list) becomes visible together.  This is strictly stronger
 * than composing independent RCU structures, where a node can be reachable in A
 * before it is reachable in B.  The unit of cross-structure atomicity is
 * exactly "one transaction": two separate commits to A and B are two
 * linearization points, the same as two unrelated RCU structures.
 *
 * This linearizes the WRITE.  A reader is NOT a transaction -- it is a sequence
 * of single-slot reads, each linearizing at its own access -- so a long
 * traversal may straddle a commit: it can observe the transaction in a slot it
 * reads after the commit and not in one it read before.  No slot and no instant
 * is ever torn (that is the guarantee); a reader's several reads simply are not
 * a mutual snapshot.  This is deliberately weaker than STM opacity: readers are
 * plain RCU readers that pay no per-read barrier.  An embedder needing a
 * multi-read snapshot layers its own versioning on top, as with any value-based
 * MCAS.
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
 * Writes.  urcu_txn_store() buffers a write whose old the caller supplies.
 * Only writes are committed and validated: the MCAS install checks each slot ==
 * old, which covers every read that became a write-old (read subset of write
 * for structural mutations).  There is no read-set.  A store can fail to
 * allocate; rather than make the caller check every store, the failure is
 * sticky -- the pending commit then reports -ENOMEM -- so a mutator only has to
 * test commit's result (which it already does).
 *
 * Read-your-own-writes is the DEFAULT: urcu_txn_load() returns a slot's pending
 * new_ptr once this attempt has recorded a store to it, and a store to an
 * already-recorded slot CHAINS onto that record -- keeping the committed old,
 * advancing new_ptr -- rather than requiring the committed old.  A transaction
 * keeps AT MOST ONE record per slot per attempt; chaining fuses same-slot
 * stores functionally (a disagreeing old poisons the attempt).
 *
 * This is what makes traversal-composed edits sound.  A commit is a state
 * transition, not a program: its edits are simultaneous, so a *_prepare that
 * searches the structure can pick a predecessor an earlier edit of the SAME
 * transaction already displaced -- both edits then land on one slot.  Reading
 * the transaction's own pending state, the traversal instead sees the structure
 * as this commit will leave it and computes the right write site; a residual
 * collision is fused by chaining.  Applies to any write site reached by a
 * traversal: an ordered skiplist's pred->next[L], an hlist's *elem->pprev.
 *
 * Two opt-outs.  urcu_txn_declare_disjoint() asserts the write set touches
 * DISTINCT SLOTS (a keyed structure whose write site is a pure function of the
 * key, over distinct keys -- a hash add/remove across distinct buckets); it
 * then blind-appends and maintains no filter -- the fast path -- since
 * read-your-own-writes is vacuous when no slot is re-touched.  Distinctness is
 * a property of the SLOT ADDRESS, not of the key: a bitmap packs 63 logical
 * bits into one physical word, so distinct bit indexes are NOT distinct slots.
 * urcu_txn_load_committed() is the per-read escape hatch, returning a slot's
 * committed value even for one this attempt has written.
 *
 * Reserve.  A mutator that knows its edge count up front may call
 * urcu_txn_reserve() right after begin: it allocates the descriptor to that
 * floor, so an OOM is reported before the mutator builds any nodes, and later
 * attempts start pre-sized rather than growing into it.  Optional -- store()
 * allocates lazily and grows on its own without it.
 *
 * Guards.  urcu_txn_load_validate() folds a READ into the commit's conflict set
 * -- the commit succeeds only if the slot still holds the value it returned.
 * urcu_txn_validate() is the same guard with a CALLER-SUPPLIED expected value,
 * for a word whose expected image the mutator computes rather than reads.  Both
 * park a proxy at plant, so a guard also serializes against a concurrent writer
 * of the guarded word.
 *
 * Escalation fallback.  The optimistic retry above is bounded-blocking but not
 * starvation-free: a large or repeatedly-bypassed transaction can be defeated
 * by a stream of smaller ones (the single-edge fast path and the read->install
 * window let a committer change a footprint slot between this op's read and
 * its install).  When a handle crosses a threshold it escalates into a
 * per-domain fair mutex (urcu/fair-mutex.h) -- an MCS-style lock -- and
 * publishes domain->active so every *future* transaction funnels through the
 * same lane. That closes the optimistic-writer set: the escalated op then
 * contends only with the finite in-flight set (bounded by thread count) and
 * commits within a bounded number of retries while holding its turn --
 * progress is guaranteed with no quiescence (no synchronize_rcu).  The lane
 * only serializes *who pushes with top priority*; commits still go through the
 * concurrency-safe MCAS path, so the residual in-flight optimistic writers
 * stay correct.  One trigger escalates a handle (gated on a non-NULL domain --
 * NULL never escalates): it has RETRIED past the budget its own cost earns it
 * (see URCU_TXN_FALLBACK_PER_COST_NUM).  Escalation is purely reactive: a
 * handle takes the lane because it is losing, never merely because it is
 * large.
 *
 * Only a handle that met a trigger ITSELF -- an INITIATOR -- publishes
 * domain->active.  A handle that escalates merely because it read the flag is a
 * JOINER: it takes a turn but advertises nothing, so it cannot outlive the
 * episode that captured it.  Were every holder to re-assert the flag, the
 * episode would sustain itself -- the flag is up whenever anyone holds the
 * lane, each arrival that samples it queues, and queuing guarantees a next
 * holder to raise it again -- so leaving the regime would require the lane to
 * drain with no arrival sampling it, i.e. write-side quiescence, which never
 * arrives under load.  A joiner that starves inside the lane is promoted to
 * initiator (urcu_txn__maybe_publish), so the funnel persists exactly as long
 * as some transaction still needs it.
 *
 * A handle keeps its turn across aborts (retry in place -- releasing would
 * forfeit the guaranteed turn) and releases it only on a terminal outcome: a
 * commit, an error, or a bail that ends the bracket with no retry pending.  An
 * ABORT is NOT terminal -- end() deliberately keeps the turn so the next
 * attempt re-enters as the same head -- so a caller that ABANDONS the operation
 * after an ABORT (end() with no further begin..commit) holds the domain's lane
 * forever and stalls every writer in the domain.  Running the bracket to a
 * terminal outcome is the required discipline; a caller that must give up after
 * an ABORT calls urcu_txn_abandon() before end().  A departing initiator clears
 * domain->active just before its unlock, ending the episode, and the domain
 * reverts to the optimistic regime once the remaining joiners drain.
 *
 * RCU.  The bracket opens an RCU read-side section per attempt, and commit
 * uses the flavor's call_rcu, so include this header AFTER an RCU flavor
 * header (e.g. <urcu-qsbr.h>); register threads and pass through quiescent
 * states as usual.
 */

#include <errno.h>
#include <stdint.h>
#include <string.h>

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
 * Reactive escalation threshold.  A handle escalates into the domain's lane
 * when it has RETRIED past the budget its own cost earns it:
 *
 *     budget = PER_COST * cost,   capped at URCU_TXN_FALLBACK_MAX
 *
 * where cost is the transaction's LOADS PLUS ITS WRITE-SET RECORDS -- the
 * whole bracket, not just the edges it commits.
 *
 * Why the budget scales with cost.  Escalating funnels EVERY writer in the
 * domain through one serial lane, so what an escalation costs is the length of
 * the critical section it serializes.  A single-edge op commits with a bare
 * CAS: serializing it is nearly free, the lane is just queueing, and escalating
 * early turns wasted CAS collisions into orderly turns.  A traversal mutator is
 * the opposite: funnelling those destroys N-way parallelism, and the optimistic
 * path (which aborts far MORE but retries in PARALLEL) beats the serialized
 * lane by a factor of 2.6 on a 3-skiplist move.  Retries are cheap and
 * parallel; the lane is not.  So the more a transaction costs, the longer it
 * should stay optimistic.
 *
 * (This is the opposite of a wasted-work rule, which would escalate an
 * expensive op SOONER because each failed attempt throws away more.
 * Measurement says wasted work is not the binding cost -- the serialization
 * is.)
 *
 * Why cost counts the LOADS.  Sizing the budget on the write set alone prices
 * the DESCENT at zero, and the descent is most of what the lane would
 * serialize: a 3-skiplist move commits ~25 records but reads ~145 slots to
 * find them.  By write set that move is indistinguishable from a 32-edge blind
 * update (24.5 vs 32 records) even though the two want budgets an order of
 * magnitude apart -- so no function of the write-set size can serve both.  By
 * cost they are 168 vs 64, and one constant does.
 *
 * 11/4 (2.75) is where the 3-skiplist peaks -- it falls off on BOTH sides
 * (2.25 -> 18.5, 2.75 -> 19.5, 4.0 -> 18.8 Mmoves/s at 192 cores).  Measured
 * against a flat 256 budget, WITH the floor below in place: 3-skiplist +11%,
 * 3-hash unchanged, bidir list unchanged, and a starved transaction's p99 ~7%
 * better.  (Without the floor the starved transaction does far better still --
 * p99 -40%, ~2x throughput -- but that costs the hot list domain 17%, which is
 * exactly what the floor buys back.)
 */
/*
 * The scale factor is a RATIONAL, kept in integer arithmetic:
 *
 *     budget = (PER_COST_NUM * cost) / PER_COST_DEN
 *
 * because the useful range turned out to be finer than one retry per unit of
 * cost.  Set PER_COST_NUM to 0 to select a flat URCU_TXN_FALLBACK budget
 * instead (the cost accounting then compiles out entirely).
 */
#ifndef URCU_TXN_FALLBACK_PER_COST_NUM
#define URCU_TXN_FALLBACK_PER_COST_NUM	11	/* 11/4 = 2.75 */
#endif
#ifndef URCU_TXN_FALLBACK_PER_COST_DEN
#define URCU_TXN_FALLBACK_PER_COST_DEN	4
#endif
#ifndef URCU_TXN_FALLBACK		/* only consulted when PER_COST_NUM == 0 */
#define URCU_TXN_FALLBACK		256
#endif
/*
 * Floor under the scaled budget.  It is what keeps a CHEAP transaction on a HOT
 * domain off the lane.  Cost measures what the ESCALATING op costs to
 * serialize; it says NOTHING about what the lane's other traffic costs, and
 * those are not the same thing -- escalating publishes domain->active, which
 * funnels every writer in the domain, however cheap the escalating op was.
 *
 * A bidir-list delete measures cost 6 (max 7), so scaling alone gives it a
 * budget of ~16 retries.  Its natural retry tail runs past that: over 562M
 * commits at 192 writers, 98.6% commit with no retry at all and 99.9999% within
 * 3 retries, but a thin tail reaches into 16..63.  Those few are enough --
 * every one of them funnels a domain running at 225 Mops/s into the serial
 * lane -- and
 * unfloored the churn panel loses 17%, all of it policy rather than accounting
 * overhead (the load counter itself measures 0.6%).  A floor of 64 puts the
 * budget above that tail: only 6 commits in 562M ever reach it.  The other
 * cheap-and-hot workload agrees -- a transacted hlist under the same contention
 * commits 99.9999% within 7 retries and never exceeds 15.
 *
 * Raising the floor further only erodes the starvation rescue (it is the CHEAP
 * ops escalating that pull a starved neighbour into the lane behind them), so
 * it wants to sit just above the natural tail of the domain's healthy traffic
 * and no higher.
 */
#ifndef URCU_TXN_FALLBACK_MIN
#define URCU_TXN_FALLBACK_MIN		64
#endif
#ifndef URCU_TXN_FALLBACK_MAX
#define URCU_TXN_FALLBACK_MAX		4096
#endif

/*
 * Read-your-own-writes lookup filter (a Bloom word), maintained for every
 * transaction except one that declared its write set disjoint.  urcu_txn__load
 * must, for each in-bracket read, decide whether the slot is already in this
 * attempt's write set.  The authoritative test is urcu_mcas_find() -- a linear
 * scan of the descriptor's records at the record stride (48 bytes).  A
 * traversal reads many slots and transacts few, so the hot case is the MISS,
 * and that scan is pure overhead on it.
 *
 * The filter is one word in the ON-STACK handle (never in struct urcu_mcas,
 * whose size is baked into the library's descriptor slab -- enlarging it there
 * overflows a slab block).  A store ORs the slot's bit; a load tests it.  A
 * clear bit (the common miss) skips find entirely; a set bit falls through to
 * find, which resolves the rare false positive -- so the filter can only ever
 * save the scan, never change a returned value.  It is reset per attempt.
 * Measured ~+4% at 192 writers on bench_txn_3skiplist (+3.6% at n=960, +4.4% at
 * n=3840, size-stable, ~12x the run-to-run spread), and non-negative for narrow
 * write-sets (a clear bit skips even the short hash scan); a dense {slot,val}
 * array was tried instead and LOST (-2.3% .. -4.6%, worsening with size)
 * because it stays O(nr) on the dominant miss.  Build -DURCU_TXN_RYW_NO_BLOOM
 * to fall back to the bare find (A/B / falsification).
 *
 * URCU_TXN_BLOOM_WORDS sets the filter width (64 bits each; default 16 = 1024
 * bits).  Widening it lowers the false-positive rate ~linearly (FP ~= k*records
 * / (64*WORDS)); the age-0/age-1 escalation study used it to separate genuine
 * RYW from filter FP.
 *
 * Both width and k are compile-time tunables that only ever trade filter cost
 * against the false-positive rate: correctness never depends on either, since a
 * false positive only ever costs a find (baseline) or an extra attempt (age 0).
 * The 16-word / k=3 default sits at the false-positive knee and wins across
 * every measured workload; escalation itself is tuned per-transaction at
 * runtime via urcu_txn_declare_disjoint() / urcu_txn_expect_conflict().
 */
#ifndef URCU_TXN_BLOOM_WORDS
# define URCU_TXN_BLOOM_WORDS	16
#endif
/*
 * URCU_TXN_BLOOM_K sets the number of hash BITS a slot maps to (default 3).
 * With k bits over m = 64*WORDS bits and n recorded slots the false-positive
 * rate is ~(1 - e^{-kn/m})^k, which for a sparse filter falls off as (kn/m)^k
 * -- so raising k cuts false positives super-linearly where widening WORDS only
 * helps linearly.  The filter is a double-hashed k-bit filter built from two
 * INDEPENDENT avalanche hashes h1,h2 (position i = h1 + i*h2); the age-0/age-1
 * study used k as the lever to drive the filter-FP escalation component toward
 * zero and isolate the genuine-RYW rate.  A degenerate k=1 is valid too (one
 * position).
 */
#ifndef URCU_TXN_BLOOM_K
# define URCU_TXN_BLOOM_K	3
#endif

#define URCU_TXN_BLOOM_BITS	(64ULL * URCU_TXN_BLOOM_WORDS)
/*
 * Two INDEPENDENT hashes of the slot.  A single multiply leaves the k derived
 * positions correlated (slot addresses are aligned and clustered).
 * Minimal-cost Kirsch-Mitzenmacher: run ONE SplitMix64 avalanche (two
 * multiplies) and split its fully-mixed 64 bits into two independent 32-bit
 * lanes -- one hash yields both h1,h2, half the cost of two separate hashes and
 * far cheaper than a multiply-free chain (Thomas Wang) whose long dependency
 * chain is slower in practice.  h2 is forced odd so the progression h1 + i*h2
 * visits k distinct bits.
 */
static inline
void urcu_txn__ryw_bloom_h1h2(void **slot, uint64_t *h1, uint64_t *h2)
{
	uint64_t x = (uint64_t) (uintptr_t) slot >> 3;	/* slots are pointer-aligned */

	x ^= x >> 30; x *= 0xbf58476d1ce4e5b9ULL;
	x ^= x >> 27; x *= 0x94d049bb133111ebULL;
	x ^= x >> 31;
	*h1 = x & 0xffffffffULL;		/* low lane */
	*h2 = (x >> 32) | 1;		/* high lane, odd stride */
}
static inline
int urcu_txn__ryw_bloom_test(const uint64_t *bloom, void **slot)
{
	uint64_t h1, h2;
	unsigned int i;

	urcu_txn__ryw_bloom_h1h2(slot, &h1, &h2);
	for (i = 0; i < URCU_TXN_BLOOM_K; i++) {
		uint64_t idx = (h1 + (uint64_t) i * h2) % URCU_TXN_BLOOM_BITS;

		if (!(bloom[idx >> 6] & ((uint64_t) 1 << (idx & 63))))
			return 0;	/* a clear bit: the slot is definitely absent */
	}
	return 1;			/* all k bits set: present (or a false positive) */
}
static inline
void urcu_txn__ryw_bloom_set(uint64_t *bloom, void **slot)
{
	uint64_t h1, h2;
	unsigned int i;

	urcu_txn__ryw_bloom_h1h2(slot, &h1, &h2);
	for (i = 0; i < URCU_TXN_BLOOM_K; i++) {
		uint64_t idx = (h1 + (uint64_t) i * h2) % URCU_TXN_BLOOM_BITS;

		bloom[idx >> 6] |= (uint64_t) 1 << (idx & 63);
	}
}
static inline
int urcu_txn__ryw_bloom_test_and_set(uint64_t *bloom, void **slot)
{
	uint64_t h1, h2;
	unsigned int i;
	int was_set = 1;

	urcu_txn__ryw_bloom_h1h2(slot, &h1, &h2);
	for (i = 0; i < URCU_TXN_BLOOM_K; i++) {
		uint64_t idx = (h1 + (uint64_t) i * h2) % URCU_TXN_BLOOM_BITS;
		unsigned int w = (unsigned int) (idx >> 6);
		uint64_t bit = (uint64_t) 1 << (idx & 63);

		if (!(bloom[w] & bit))
			was_set = 0;
		bloom[w] |= bit;
	}
	return was_set;
}

/*
 * Age-0/age-1 optimistic RYW escalation.
 *
 * The premise: read-your-own-writes only bites when an attempt reads a slot it
 * has already written, which for a sparse or low-batch write-set is rare -- yet
 * a naive RYW path pays the Bloom test (and, on a hit, the find scan) on every
 * in-bracket load regardless.
 *
 * So the FIRST attempt of an operation (retry == 0, "age 0") runs a stripped
 * RYW path: it maintains the Bloom filter as usual but NEVER calls find.  A
 * load or store whose slot is already in the filter -- a possible
 * read-after-write or write-after-write, true or a filter false positive --
 * sets esc_pending instead of resolving it.  The optimistic value a colliding
 * load returns may be stale, but esc_pending forces commit to ABORT, so an
 * age-0 attempt that saw ANY coincidence never installs: it is discarded
 * unpublished and re-run at retry >= 1 ("age 1+"), where the full path (Bloom +
 * find, chaining) resolves RYW correctly.  An age-0 attempt that saw NO
 * coincidence has a write-set with no same-slot records and no stale reads, so
 * it is exactly a baseline commit and installs directly.
 *
 * The trade: age 0 never runs find, at the price of a whole extra attempt
 * whenever it guesses wrong.  So the win is bounded by the ESCALATION RATE --
 * how often an operation hits a coincidence (genuine RYW) or a Bloom false
 * positive -- and is therefore WORKLOAD-DEPENDENT: it pays off for
 * disjoint/low-RYW write-sets (e.g. the hash key-move) and loses for dense-RYW
 * batched descents (e.g. the skiplist), which opt out per-transaction with
 * urcu_txn_expect_conflict() so their first attempt goes straight to the find
 * path.  Widening URCU_TXN_BLOOM_WORDS shrinks the false-positive component;
 * correctness never depends on the filter width, since a false positive only
 * ever spends an extra attempt.
 */

/*
 * Compile-time fallback for the RCU read-side bracket, used by
 * urcu_txn_read_lock()/urcu_txn_read_unlock() (hence begin()/end()) ONLY when a
 * handle binds no flavor (urcu_txn_init's NULL).  Defaults to the
 * compile-time-selected flavor's rcu_read_lock / rcu_read_unlock -- hence the
 * "include after an RCU flavor header" rule -- and may be overridden before
 * include.  The PREFERRED path for a FLAVOR-AGNOSTIC embedder (one selecting
 * its RCU flavor at runtime) is to bind that flavor with urcu_txn_init_flavor()
 * so the bracket opens in it directly; this macro then never fires.
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
 * THE READ POLICY.  Help iff the loaded slot ends up in this transaction's
 * read/write set; read optimistically ONLY to navigate.
 *
 * urcu_txn_load_optimistic() resolves an undecided parker to its logical old --
 * the value the slot takes if the parker aborts -- without driving the parker
 * to a decision.  For a slot the transaction then stores (or load-validates),
 * that value is stale by construction whenever the parker goes on to commit, so
 * the record's old_ptr cannot match at install and the plant CAS is DOOMED: the
 * attempt is guaranteed to abort.  A helping load instead pays the parker's
 * install once and returns the decided value, which commits either way.  The
 * saving is real only for a load whose value nothing later depends on: a
 * traversal hop that merely points at the next node.
 *
 * Measured both directions.  Making rcu-txn-hlist.h's *_prepare loads
 * optimistic cost 30% at 64 buckets / 192 writers (abort:commit 0.61 -> 0.86)
 * and was invisible at the 4096 buckets the published benchmark used, where
 * nothing aborts at all.  The converse -- rcu-txn-skiplist.h helping its five
 * _prepare loads while its descent stays optimistic -- gained 8.2% at 192
 * writers.
 *
 * The policy is a property of the CALL SEQUENCE, not of contention, so it is
 * checkable single-threaded: for every slot entering the read/write set, the
 * most recent load of that slot in the same attempt must have helped.  Build
 * with -DURCU_TXN_DEBUG_READ_POLICY to enforce it -- every urcu_txn__record()
 * (store or load-validate) is checked against the kind of the last load, and a
 * violation aborts with a diagnostic.  Add -DURCU_TXN_DEBUG_READ_POLICY_SOFT to
 * count violations in the handle instead of aborting, which is what
 * tests/unit/test_rcu_txn_read_policy.c does so it can assert on the count.
 *
 * The check costs a hash-table probe per load and per record, and grows the
 * on-stack handle by URCU_TXN_RP_SLOTS entries.  Debug builds only.
 *
 * A slot never loaded in this attempt is not a violation: a blind store (a
 * fresh node's own field, a head whose old value the caller already holds) has
 * no read to speak of.  Note that urcu_txn_load_validate_optimistic() is a
 * read-set read that does not help, so it violates the policy BY CONSTRUCTION
 * -- it has no callers, and this is why.
 */
#ifdef URCU_TXN_DEBUG_READ_POLICY
# include <stdio.h>
# include <stdlib.h>
# include <string.h>
# ifndef URCU_TXN_RP_SLOTS
#  define URCU_TXN_RP_SLOTS	256	/* Power of two. */
# endif
# define URCU_TXN_RP_PROBE	8	/* Linear probe before evicting. */
struct urcu_txn__rp_entry {
	void **slot;		/* NULL: empty */
	int optimistic;		/* Kind of the most recent load of @slot. */
};
#endif

/*
 * Per-contention-domain escalation state, shared by every handle that transacts
 * the same structure.  Pass &domain to urcu_txn_init(), or NULL to
 * disable the fallback (pure optimistic retry).
 */
struct urcu_txn_domain {
	struct cds_fair_mutex lock;	/* The fair escalation lane. */
	unsigned long active;		/* A fallback episode is in progress. */
};

static inline
void urcu_txn_domain_init(struct urcu_txn_domain *d)
{
	cds_fair_mutex_init(&d->lock);
	d->active = 0;
}

struct urcu_mcas_txn {
	struct urcu_txn_domain *domain;	/* Escalation domain, or NULL */
	const struct rcu_flavor_struct *flavor;	/*
						 * RCU flavor for the read-side
						 * bracket, or NULL to use the
						 * compile-time-selected flavor
						 * (URCU_TXN_RCU_READ_LOCK).
						 * Set at create by
						 * urcu_txn_init_flavor(); lets
						 * a FLAVOR-AGNOSTIC embedder
						 * (one selecting its flavor at
						 * runtime, e.g. the fractal
						 * trie) bracket the txn in its
						 * own flavor's read-side
						 * section.
						 */
	unsigned long retry;		/*
					 * Attempts so far; aging priority for
					 * the MCAS.
					 */
	unsigned int min_alloc;		/*
					 * floor for the attempt's initial
					 * descriptor capacity, grown past if
					 * exceeded (0 -> INIT default).  Set
					 * via reserve(); refreshed to the
					 * realized size at commit so retries
					 * don't re-grow.
					 */
	struct urcu_mcas *mcas;		/*
					 * this attempt's descriptor: NULL (none
					 * yet), a live descriptor, or the
					 * ENOMEM marker.
					 */
	struct cds_fair_mutex_node waiter;	/* our node while awaiting the turn */
	int in_fallback;		/*
					 * We currently hold the lock
					 * (thread-private, but relaxed-atomic:
					 * see __exit_fallback).
					 */
	int fb_published;		/*
					 * We raised domain->active and owe the
					 * clear: set only for an initiator,
					 * never a joiner.
					 */
	int retrying;			/* Commit asked retry: keep the turn. */
	uint64_t ryw_bloom[URCU_TXN_BLOOM_WORDS];
					/*
					 * Read-your-own-writes load filter: OR
					 * of recorded slots' bits; cleared per
					 * attempt by begin().  Not maintained
					 * for a disjoint handle, which then
					 * just carries the dead word(s).
					 */
	int disjoint;			/*
					 * Caller asserts this txn's write set
					 * touches DISTINCT slots (no
					 * write-after- write): e.g. a hash
					 * add/remove over distinct buckets.
					 * Lets age 0 blind-append WITHOUT the
					 * RYW Bloom -- no filter maintenance,
					 * no load-side test -- since there is
					 * no coincidence to catch.  Set with
					 * urcu_txn_declare_disjoint() before
					 * begin(); a violation corrupts (adds a
					 * duplicate record) unless
					 * URCU_TXN_DEBUG_DISJOINT traps it.
					 */
	unsigned int nload;		/*
					 * Loads issued by the CURRENT attempt
					 * (reset by begin()).  The read side
					 * of the attempt's cost: a
					 * traversal-driven mutator reads far
					 * more slots than it writes, and that
					 * descent sits INSIDE the bracket, so
					 * the fallback lane serializes it too.
					 * min_alloc alone (the write set)
					 * badly understates what escalating
					 * such a handle costs.
					 */
	unsigned int last_cost;		/*
					 * Cost of the last completed attempt:
					 * nload + write-set size, learned at
					 * commit exactly as min_alloc is.  0
					 * before the first commit attempt.
					 */
	int expect_conflict;		/*
					 * Caller expects this txn to conflict
					 * -- the negative dual of disjoint.
					 * Two shapes: dense
					 * read-your-own-writes by construction
					 * (batched adjacent edits on an ordered
					 * structure, whose consecutive keys
					 * share a predecessor slot so the
					 * composed edits always alias -- a
					 * skiplist range move), or contention
					 * heavy enough that the non- blocking
					 * age-0 install fail-fasts more than it
					 * commits.  Makes age 0 skip the
					 * optimistic path
					 * (skip-find/blind-append AND
					 * skip-sort/try-latch install) so a
					 * doomed attempt is never spent; runs
					 * the sorted, blocking path from the
					 * first attempt.  Set with
					 * urcu_txn_expect_conflict() before
					 * begin().
					 */
	int esc_pending;		/*
					 * The age-0 optimistic attempt saw a
					 * same-slot coincidence (RAW/WAW, or a
					 * Bloom FP) and must abort to re-run at
					 * age 1+; reset per attempt.
					 */
#ifdef URCU_TXN_ESCALATION_STATS
	/*
	 * Would-this-attempt-escalate instrumentation for the age-0/age-1 study
	 * (measurement only; not part of the engine).  Counts the same-slot
	 * coincidences that would bounce an optimistic age-0 attempt to age 1.
	 * Reset per attempt by begin().
	 */
	unsigned int esc_raw;		/*
					 * RYW loads that hit a recorded slot
					 * (true read-after-write).
					 */
	unsigned int esc_waw;		/*
					 * Stores onto an already-recorded slot
					 * (true write-after-write).
					 */
	unsigned int esc_bloom;		/*
					 * Accesses whose 64-bit bloom bit was
					 * already set (RAW/WAW + false
					 * positives). */
#endif
#ifdef URCU_TXN_DEBUG_READ_POLICY
	struct urcu_txn__rp_entry rp[URCU_TXN_RP_SLOTS];
					/*
					 * Kind of the most recent load of each
					 * slot THIS attempt; cleared by
					 * begin().
					 */
	unsigned long rp_violations;	/*
					 * Records made on a slot whose last
					 * load was optimistic; sums over the
					 * attempts of one transaction.
					 */
	unsigned long rp_evicted;	/*
					 * Table overflowed: a mark was dropped,
					 * so a zero violation count is NOT a
					 * proof.
					 */
#endif
};

#ifdef URCU_TXN_DEBUG_READ_POLICY

static inline
unsigned int urcu_txn__rp_hash(void **slot)
{
	uintptr_t h = (uintptr_t) slot >> 3;	/* slots are pointer-aligned */

	h *= 0x9e3779b97f4a7c15ULL;
	return (unsigned int) ((h >> 40) & (URCU_TXN_RP_SLOTS - 1));
}

/* A new attempt reloads everything: forget every load. */
static inline
void urcu_txn__rp_reset(struct urcu_mcas_txn *txn)
{
	memset(txn->rp, 0, sizeof(txn->rp));
}

/* Remember the kind of the most recent load of @slot. */
static inline
void urcu_txn__rp_note(struct urcu_mcas_txn *txn, void **slot, int optimistic)
{
	unsigned int h = urcu_txn__rp_hash(slot), i;

	for (i = 0; i < URCU_TXN_RP_PROBE; i++) {
		struct urcu_txn__rp_entry *e =
			&txn->rp[(h + i) & (URCU_TXN_RP_SLOTS - 1)];

		if (e->slot == NULL || e->slot == slot) {
			e->slot = slot;
			e->optimistic = optimistic;
			return;
		}
	}
	txn->rp[h].slot = slot;		/* evict: the dropped mark is a MISSED check */
	txn->rp[h].optimistic = optimistic;
	txn->rp_evicted++;
}

/*
 * @slot is entering the read/write set.  Its last load must have helped.  A
 * slot with no mark was never loaded here (a blind store): nothing to check.
 * Once checked, the slot is settled -- a later store chains onto the record
 * rather than off a fresh physical read -- so clear the mark rather than report
 * twice.
 */
static inline
void urcu_txn__rp_check(struct urcu_mcas_txn *txn, void **slot)
{
	unsigned int h = urcu_txn__rp_hash(slot), i;

	for (i = 0; i < URCU_TXN_RP_PROBE; i++) {
		struct urcu_txn__rp_entry *e =
			&txn->rp[(h + i) & (URCU_TXN_RP_SLOTS - 1)];

		if (e->slot == NULL)
			return;
		if (e->slot != slot)
			continue;
		if (!e->optimistic)
			return;
		e->optimistic = 0;
		txn->rp_violations++;
#ifndef URCU_TXN_DEBUG_READ_POLICY_SOFT
		fprintf(stderr, "urcu-txn: read-policy violation: slot %p enters "
			"the read/write set, but its last load in this attempt "
			"was optimistic.  The plant CAS is doomed whenever the "
			"parker commits.  Use urcu_txn_load()/urcu_txn_load_validate() "
			"for a slot this transaction stores or guards; keep "
			"urcu_txn_load_optimistic() for navigation.\n", (void *) slot);
		abort();
#endif
		return;
	}
}

/*
 * Records made this transaction on a slot whose last load was optimistic, and
 * marks the probe table dropped.  Both are 0 unless built with
 * -DURCU_TXN_DEBUG_READ_POLICY; a violation count is only meaningful when the
 * eviction count is 0.
 */
static inline
unsigned long urcu_txn_read_policy_violations(const struct urcu_mcas_txn *txn)
{
	return txn->rp_violations;
}

static inline
unsigned long urcu_txn_read_policy_evicted(const struct urcu_mcas_txn *txn)
{
	return txn->rp_evicted;
}

#else	/* !URCU_TXN_DEBUG_READ_POLICY */

# define urcu_txn__rp_reset(txn)		do { } while (0)
# define urcu_txn__rp_note(txn, slot, opt)	do { } while (0)
# define urcu_txn__rp_check(txn, slot)		do { } while (0)

static inline
unsigned long urcu_txn_read_policy_violations(const struct urcu_mcas_txn *txn)
{
	(void) txn;
	return 0;
}

static inline
unsigned long urcu_txn_read_policy_evicted(const struct urcu_mcas_txn *txn)
{
	(void) txn;
	return 0;
}

#endif	/* URCU_TXN_DEBUG_READ_POLICY */

/*
 * Empty this attempt's read-your-own-writes filter.  Called at the ONE point an
 * attempt's descriptor goes from "none" to "live" (urcu_txn_reserve,
 * urcu_txn__record) -- which is strictly before the filter can be consulted,
 * since urcu_txn__load and urcu_txn__record both test it only once txn->mcas is
 * a live descriptor, and every fresh attempt starts from txn->mcas == NULL
 * (begin(), and the commit paths, which consume the descriptor).
 *
 * Zeroing HERE rather than in begin() is what keeps the filter off the DISJOINT
 * fast path: such a handle consults no filter, so it must not pay to clear one
 * (a measurable ~1% on the disjoint hash key-move benchmark).  It also spares
 * an attempt that reads and bails without ever storing.  And it is what makes
 * the documented begin-less driving mode (init -> read_lock -> store ->
 * commit_flavor; see urcu_txn_read_lock) sound: that mode has no begin() to
 * clear the filter, and would otherwise test indeterminate stack bytes, whose
 * set bits flag a phantom coincidence on the very first load.
 */
static inline
void urcu_txn__bloom_reset(struct urcu_mcas_txn *txn)
{
	if (!txn->disjoint)
		memset(txn->ryw_bloom, 0, sizeof(txn->ryw_bloom));
}

/*
 * Initialize a handle before its retry loop (retry := 0, no reservation),
 * bracketing the txn's RCU read-side section in @flavor's
 * read_lock/read_unlock.  Pass @flavor NULL to use the compile-time-selected
 * flavor (the URCU_TXN_RCU_READ_LOCK default) -- the plain urcu_txn_init() does
 * exactly this.  A flavor-agnostic embedder (one selecting its RCU flavor at
 * runtime through a rcu_flavor_struct vtable, e.g. the fractal trie) passes
 * that flavor here so begin()/end() -- and the standalone urcu_txn_read_lock()
 * / urcu_txn_read_unlock() bracket -- open the read-side section in it, the
 * same way commit_flavor() defers reclaim through flavor->update_call_rcu.
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
	txn->nload = 0;
	txn->last_cost = 0;
	txn->mcas = NULL;
	uatomic_store(&txn->in_fallback, 0, CMM_RELAXED);
	txn->fb_published = 0;
	txn->retrying = 0;
	txn->disjoint = 0;
	txn->expect_conflict = 0;
	/*
	 * ryw_bloom is deliberately NOT zeroed here.  It is zeroed by
	 * urcu_txn__bloom_reset() at the point the attempt's descriptor is
	 * first allocated -- which is strictly before the filter can be
	 * consulted, and which a disjoint handle skips entirely.  Zeroing it at
	 * init would tax every disjoint handle for a filter it never reads.
	 */
	txn->esc_pending = 0;
#ifdef URCU_TXN_ESCALATION_STATS
	txn->esc_raw = txn->esc_waw = txn->esc_bloom = 0;
#endif
#ifdef URCU_TXN_DEBUG_READ_POLICY
	txn->rp_violations = 0;
	txn->rp_evicted = 0;
	urcu_txn__rp_reset(txn);
#endif
}

#ifdef URCU_TXN_ESCALATION_STATS
static inline
unsigned int urcu_txn_esc_raw(const struct urcu_mcas_txn *txn) { return txn->esc_raw; }
static inline
unsigned int urcu_txn_esc_waw(const struct urcu_mcas_txn *txn) { return txn->esc_waw; }
static inline
unsigned int urcu_txn_esc_bloom(const struct urcu_mcas_txn *txn) { return txn->esc_bloom; }
#endif

/*
 * Assert that this handle's transactions touch DISTINCT SLOTS -- no store ever
 * lands on a slot already recorded in the same commit (no write-after-write and
 * no read-of-own-write).  This is the common shape of a keyed structure whose
 * write site is a pure function of the key, over distinct keys: a hash table
 * add/remove across distinct buckets.
 *
 * Distinctness is a property of the SLOT ADDRESS, not of the key.  Two shapes
 * that look keyed and distinct but are NOT:
 *
 *   - a structure that PACKS several logical keys into one word.  A bitmap word
 *     holds 63 logical bits (<urcu/rcu-txn-bitmap.h>), so two composed flips of
 *     DISTINCT bit indexes can record the SAME slot.
 *   - a write site reached by a TRAVERSAL, whose slot is data-dependent rather
 *     than a function of the key.  Composed edits on ONE ordered structure (a
 *     skiplist range move or rotation, adjacent list/hlist deletes) share a
 *     predecessor slot as soon as the keys land adjacent -- which the keys
 *     alone do not tell you.
 *
 * Both want the DEFAULT (read-your-own-writes) handle, which catches the
 * coincidence and chains the two edits into one record; and because they alias
 * by construction rather than by luck, both want urcu_txn_expect_conflict() so
 * the doomed age-0 attempt is skipped.  Reserve declare_disjoint() for a write
 * set whose slots are provably pairwise distinct.
 *
 * Given the guarantee, age 0 blind-appends each store -- skipping the O(nr)
 * reconcile find -- AND maintains no read-your-own-writes Bloom filter (no
 * per-store hash-and-set, no per-load test), since there is no coincidence to
 * catch.  This is the fast path for a disjoint mutator; the default RYW
 * behaviour is for mutators whose composed edits CAN alias a slot (a
 * traversal-reached write site), which need the filter to catch and reconcile
 * the coincidence.
 *
 * Contract: if a store DOES hit a recorded slot, the blind append inserts a
 * duplicate record and the commit corrupts.  The sorted install plants the
 * first record's proxy and then takes the "own proxy" short-circuit for the
 * second (the distinct-slot invariant it rests on is exactly what was
 * violated), decides SUCCEEDED, and settle stores both new values to the one
 * slot in record order: the EARLIER edit is silently lost in a commit that
 * reports OK.  Build with -DURCU_TXN_DEBUG_DISJOINT to trap a violation at the
 * offending store instead -- but note it traps only the STORE side: a
 * read-of-own-write on a disjoint handle skips the write-set consult entirely
 * and silently returns the slot's COMMITTED value, which no debug build sees.
 * Call after init and before the first begin(); do not flip mid-transaction.
 */
static inline
void urcu_txn_declare_disjoint(struct urcu_mcas_txn *txn)
{
	txn->disjoint = 1;
}

/*
 * The negative dual of declare_disjoint(): assert that this handle's
 * transactions are EXPECTED to conflict, so the optimistic age-0 attempt is not
 * worth trying.  Two shapes want this.  (1) A txn whose own
 * read-your-own-writes is dense by construction -- batched adjacent edits on an
 * ordered structure, where the rotation maps one edit's destination onto the
 * next's source so consecutive keys always share a predecessor slot (a skiplist
 * range move): age 0 then reads its own pending write, sets esc_pending, and
 * aborts in the descent every time.  (2) A txn run under contention heavy
 * enough that the non-blocking age-0 install fail-fasts more often than it
 * commits.  In both, the age-0 attempt is spent only to abort; expect_conflict
 * skips it and runs the sorted, find-resolved, blocking path from the first
 * attempt.
 *
 * Call after init and before the first begin(); do not flip mid-transaction.
 * Belongs on a default (RYW) handle -- the aliasing is resolved at commit by
 * chaining -- and is mutually exclusive in intent with declare_disjoint().
 */
static inline
void urcu_txn_expect_conflict(struct urcu_mcas_txn *txn)
{
	txn->expect_conflict = 1;
}

/*
 * Effective attempt age.  Normally the retry count; but a handle that declared
 * expect_conflict() reports >= 1 even on its first attempt.  Both the txn layer
 * (skip-find/blind-append/esc_pending, keyed on this) and the mcas layer (skip-
 * sort/try-latch install, keyed on the descriptor's retry, which is created
 * from this) then bypass the age-0 optimistic path and run the sorted, blocking
 * path from the start.
 */
static inline
unsigned long urcu_txn__eff_retry(const struct urcu_mcas_txn *txn)
{
	return (txn->expect_conflict && txn->retry == 0) ? 1UL : txn->retry;
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
 * Callable with or without the RCU read-side section held: cds_fair_mutex_lock
 * may block, but the wait is bounded (a FIFO turn behind holders whose commits
 * are bounded MCAS runs) and the lane owner never blocks on a grace period
 * while holding the mutex, so holding the section across the wait cannot
 * extend a grace period unboundedly.  The body comment details why reserve()
 * deliberately enters while inside the bracket; begin() enters before opening
 * it (nothing is pinned yet).
 */

/*
 * Does this handle earn the lane on its own merits -- starved, or known large?
 * Such a handle is an INITIATOR: it advertises the episode.  A handle that
 * escalates only because domain->active was up is a joiner and advertises
 * nothing.  The distinction is what makes an episode terminate.
 */
/*
 * What the CURRENT attempt has cost so far: the slots it read plus the edges
 * it has buffered.  Sampled wherever an attempt ends -- commit AND the
 * guard-driven urcu_txn_conflict() path, which never reaches commit at all.
 * Missing the latter would leave a traversal mutator that keeps hitting a
 * guard with a cost of 0 forever, and a cost-scaled threshold would then
 * escalate it immediately -- exactly the handle that can least afford to be
 * serialized.
 */
static inline
unsigned int urcu_txn__attempt_cost(const struct urcu_mcas_txn *txn)
{
	const struct urcu_mcas *m = txn->mcas;
	unsigned int w = (m && m != URCU_TXN_ENOMEM) ? m->nr : 0;

	return txn->nload + w;
}

/*
 * Learn this operation's cost as a HIGH-WATER MARK, not as the last sample.
 * An attempt that aborts early in its descent is CHEAP -- it never got to the
 * far end of the traversal -- so taking the last sample would let a handle
 * that keeps failing mid-descent shrink its own threshold, escalating
 * precisely the mutator whose full bracket is most expensive to serialize.
 * The high-water mark is what this operation costs when it RUNS, which is what
 * the lane would serialize.
 */
static inline
void urcu_txn__learn_cost(struct urcu_mcas_txn *txn)
{
#if URCU_TXN_FALLBACK_PER_COST_NUM
	unsigned int c = urcu_txn__attempt_cost(txn);

	if (c > txn->last_cost)
		txn->last_cost = c;
#else
	(void) txn;			/* flat threshold: cost is never consulted */
#endif
}

/*
 * Retries this handle may spend on the optimistic path before it earns the
 * lane. With URCU_TXN_FALLBACK_PER_COST_NUM == 0 this is the flat
 * URCU_TXN_FALLBACK. Otherwise it scales with the realized write-set size
 * (min_alloc, learned at the first commit -- 0 on the very first attempt,
 * which cannot have retried anyway), so a cheap-to-serialize op escalates
 * early and an expensive one does not.  See the URCU_TXN_FALLBACK_PER_COST_NUM
 * comment above.
 */
static inline
unsigned long urcu_txn__fallback_at(const struct urcu_mcas_txn *txn)
{
	unsigned long n, t;

	if (!URCU_TXN_FALLBACK_PER_COST_NUM)
		return URCU_TXN_FALLBACK;
	/*
	 * 1 until the first attempt has been costed: a handle that has not run
	 * yet has not retried either, so the budget it gets cannot matter.
	 */
	n = txn->last_cost ? txn->last_cost : 1;
	t = ((unsigned long) URCU_TXN_FALLBACK_PER_COST_NUM * n) /
		URCU_TXN_FALLBACK_PER_COST_DEN;
	if (t < URCU_TXN_FALLBACK_MIN)
		t = URCU_TXN_FALLBACK_MIN;	/* also rules out a zero budget */
	return t > URCU_TXN_FALLBACK_MAX ? URCU_TXN_FALLBACK_MAX : t;
}

static inline
int urcu_txn__self_qualifies(const struct urcu_mcas_txn *txn)
{
	return txn->retry >= urcu_txn__fallback_at(txn);
}

static inline
void urcu_txn__enter_fallback(struct urcu_mcas_txn *txn)
{
	/*
	 * cds_fair_mutex_lock may park on a futex until our turn.  We hold the
	 * caller's RCU read-side section across that wait rather than going
	 * RCU-offline: that section is what keeps the caller's input pointers
	 * (e.g. a list anchor reached by key) alive for the whole transaction,
	 * and dropping it would let a concurrent grace period free them out
	 * from under us.  (Going offline is a QSBR-only move anyway -- the
	 * bracketing flavors hold a nested read-side lock the callee cannot
	 * release.)  This is safe because the wait is bounded: the FIFO turn is
	 * short and the lane owner's commit is a bounded MCAS that defers
	 * reclaim through call_rcu and never itself waits on a grace period, so
	 * holding the section across it cannot extend a grace period
	 * unboundedly.  That "the lane owner never blocks on a GP while holding
	 * the mutex" is the one invariant this relies on.
	 */
	cds_fair_mutex_lock(&txn->domain->lock, &txn->waiter);
	/*
	 * Advertise the episode only if we met a trigger ourselves; a joiner
	 * publishes nothing, so it cannot outlive the initiator that captured
	 * it (see the escalation-fallback paragraph at the top of this header).
	 */
	if (urcu_txn__self_qualifies(txn)) {
		uatomic_store(&txn->domain->active, 1, CMM_RELAXED);
		txn->fb_published = 1;
	}
	uatomic_store(&txn->in_fallback, 1, CMM_RELAXED);
}

/*
 * Release the lock.  An initiator clears domain->active BEFORE the unlock, so
 * the episode ends when its initiator leaves rather than when the lane happens
 * to empty; any joiners still queued drain with the flag already down and the
 * domain reverts to the optimistic path.
 */
static inline
void urcu_txn__exit_fallback(struct urcu_mcas_txn *txn)
{
	/*
	 * Clear the episode flag BEFORE releasing the lock.  Clearing after --
	 * even gated on the unlock's "last holder" return -- races the next
	 * INITIATOR: the actual lock release is the dequeue's tail-reset
	 * cmpxchg INSIDE cds_fair_mutex_unlock(), so by the time it returns
	 * "last", a new thread may have acquired the freed lock and stored
	 * active = 1; our late 0 would then overwrite the new episode's
	 * advertisement, and that whole episode would run unfunnelled -- no
	 * future transaction takes the lane, so "closes the optimistic-writer
	 * set" silently fails in exactly the starved case the lane exists for.
	 *
	 * Only a publisher clears, and only its own advertisement: at most one
	 * handle has fb_published set at a time, because a handle publishes
	 * only while it is the lock holder (on acquiring, or on being promoted
	 * mid- episode by begin()).  So the store below cannot erase a peer's
	 * flag.
	 */
	if (txn->fb_published) {
		uatomic_store(&txn->domain->active, 0, CMM_RELAXED);
		txn->fb_published = 0;
	}
	/*
	 * Drop in_fallback INSIDE the critical section, not after the unlock.
	 * Callers read it as "we already own the lane": want_fallback()
	 * suppresses escalation on it and urcu_txn_end() unlocks on it.
	 * Clearing after the unlock would leave a window claiming ownership of
	 * a lane already released -- the direction that misleads; clearing
	 * first can at worst deny ownership we still hold, which nothing
	 * between here and the unlock consults.
	 *
	 * The field is thread-private, so no barrier is owed and CMM_RELAXED is
	 * enough -- but it must be an atomic access, not a plain store: the
	 * compiler can see it is unaliased and would happily sink it past the
	 * (inlinable) unlock, or hoist the store in __enter_fallback above the
	 * lock, undoing exactly the ordering above.
	 */
	uatomic_store(&txn->in_fallback, 0, CMM_RELAXED);
	(void) cds_fair_mutex_unlock(&txn->domain->lock, &txn->waiter);
}

/*
 * Whether this attempt should escalate into the lock before opening: a starved
 * (retry) or already-known large (min_alloc) handle initiates an episode, and
 * domain->active funnels every other handle into the same lane for the
 * episode's duration -- that funnelling is what closes the optimistic- writer
 * set and bounds the escalated op's progress.  domain->active is advisory: a
 * stale read only mis-routes one bounded attempt (the MCAS commit is correct
 * under the resulting concurrency), so it needs no acquire/release, only
 * atomicity.
 */
static inline
int urcu_txn__want_fallback(struct urcu_mcas_txn *txn)
{
	return txn->domain && !uatomic_load(&txn->in_fallback, CMM_RELAXED) &&
		(uatomic_load(&txn->domain->active, CMM_RELAXED) ||
		 urcu_txn__self_qualifies(txn));
}

/*
 * A handle that starves (or grows large) while ALREADY holding its turn must be
 * promoted to initiator: it now needs the funnel that the departed initiator's
 * episode had been giving it.  want_fallback() cannot do this -- it is gated on
 * !in_fallback -- so without this a joiner would retry forever inside the lane
 * with the optimistic-writer set wide open, which is precisely the starvation
 * the lane exists to end.
 */
static inline
void urcu_txn__maybe_publish(struct urcu_mcas_txn *txn)
{
	if (txn->domain && uatomic_load(&txn->in_fallback, CMM_RELAXED) &&
			!txn->fb_published && urcu_txn__self_qualifies(txn)) {
		uatomic_store(&txn->domain->active, 1, CMM_RELAXED);
		txn->fb_published = 1;
	}
}

/* Begin one attempt: clear the write-set and open the RCU read-side section. */
static inline
void urcu_txn_begin(struct urcu_mcas_txn *txn)
{
	txn->retrying = 0;
	txn->nload = 0;		/* attempt cost is per-attempt */
	/*
	 * Escalate before opening the attempt: a starved (retry) or
	 * already-known large (min_alloc) handle takes its FIFO turn here.
	 * cds_fair_mutex_lock may block; nothing is pinned yet, so take the
	 * turn before opening the read-side section (holding one across the
	 * bounded wait would also be sound -- reserve() does; see
	 * urcu_txn__enter_fallback).
	 */
	if (urcu_txn__want_fallback(txn))
		urcu_txn__enter_fallback(txn);
	else
		urcu_txn__maybe_publish(txn);	/* joiner that starved: promote */
	txn->mcas = NULL;		/* prior attempt's descriptor already consumed/freed */
	/*
	 * The RYW Bloom is NOT cleared here.  Setting mcas = NULL above is what
	 * opens the fresh attempt, and the filter is emptied by
	 * urcu_txn__bloom_reset() when the next store (or reserve) allocates
	 * the descriptor -- strictly before anything can consult it.  See it
	 * for why that placement, and not this one, is what keeps a disjoint
	 * handle free of the filter entirely.
	 */
	txn->esc_pending = 0;		/* fresh attempt: no coincidence seen yet */
#ifdef URCU_TXN_ESCALATION_STATS
	txn->esc_raw = txn->esc_waw = txn->esc_bloom = 0;
#endif
	urcu_txn__rp_reset(txn);	/* debug: a new attempt reloads every slot */
	urcu_txn_read_lock(txn);
}

/*
 * Pre-reserve room for @n writes this attempt and record @n as the min_alloc
 * floor.  Allocates the descriptor up front, so an algorithm that knows its
 * edge count can fail early with -ENOMEM -- before building the nodes it meant
 * to link -- rather than discovering the failure partway through its stores.
 * The floor also sizes every later attempt's initial descriptor, so retries
 * start at @n instead of growing into it.  Returns 0, or -ENOMEM (sticky: the
 * pending commit then also returns -ENOMEM).  Optional; call after begin,
 * before the first store.  store() still grows the descriptor should the
 * write-set exceed @n.
 */
static inline
int urcu_txn_reserve(struct urcu_mcas_txn *txn, unsigned int n)
{
	struct urcu_mcas *m;

	txn->min_alloc = n;
	if (caa_unlikely(txn->mcas == URCU_TXN_ENOMEM))
		return -ENOMEM;		/* sticky: an earlier alloc already failed */
	if (!n)
		return 0;		/* no reservation; the INIT default applies */
	if (!txn->mcas) {
		/*
		 * The descriptor's retry is the age the MCAS install dispatches
		 * on, so create it at the EFFECTIVE age: an expect_conflict()
		 * handle that reserves before its first store must still get
		 * the sorted, blocking install on attempt 0 -- txn->retry alone
		 * would hand it the age-0 flat install it asked to skip.
		 */
		m = urcu_mcas_create(n, urcu_txn__eff_retry(txn));
		if (caa_unlikely(!m)) {
			txn->mcas = URCU_TXN_ENOMEM;
			return -ENOMEM;
		}
		urcu_txn__bloom_reset(txn);	/* the attempt's write set opens empty */
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
 * Reconcile one record against the write-set under the chaining rule (match the
 * pending new_ptr, keep the committed old_ptr, advance new_ptr).  See
 * urcu_mcas_record_chain().
 */
static inline
bool urcu_txn__reconcile(struct urcu_mcas_txn *txn, struct urcu_mcas *m,
		void **slot, void *old_ptr, void *new_ptr, int upgrade,
		uintptr_t tag)
{
	(void) txn;
	return urcu_mcas_record_chain(m, slot, old_ptr, new_ptr, upgrade, tag);
}

/*
 * Buffer or reconcile one record (see urcu_mcas_record_chain): lazily create
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

	urcu_txn__rp_check(txn, slot);	/* debug: @slot enters the read/write set */
	if (caa_unlikely(m == URCU_TXN_ENOMEM))
		return -ENOMEM;		/* sticky: an earlier record already failed */
	if (!m) {
		m = urcu_mcas_create(txn->min_alloc ? txn->min_alloc :
				URCU_TXN_INIT, urcu_txn__eff_retry(txn));
		if (caa_unlikely(!m)) {
			txn->mcas = URCU_TXN_ENOMEM;
			return -ENOMEM;
		}
		urcu_txn__bloom_reset(txn);	/* the attempt's write set opens empty */
		txn->mcas = m;
	}
	/*
	 * Age 0: a store onto an already-recorded slot (write-after-write, or a
	 * Bloom false positive) escalates to age 1+ rather than chaining here.
	 * One hash: fuse the pre-store test with setting the slot's bit (the
	 * filter add the baseline does at the tail below), so the store path
	 * hashes the slot exactly once.  The test reads the state BEFORE the
	 * OR, so it still sees "already present".
	 *
	 * The bit's only consumer is a later same-txn RYW load, so gate the
	 * whole hash-and-set on !txn->disjoint: a txn whose write set is
	 * disjoint by construction (a hash table add/remove touching distinct
	 * buckets) maintains no filter at all.  txn->disjoint is fixed before
	 * begin(), so the guard is a per-handle-constant, well-predicted
	 * branch.
	 */
	if (!txn->disjoint) {
		/*
		 * The return is the filter's state BEFORE this store's bits
		 * were OR'd in -- the only chance to see it, since the set has
		 * already happened.  Re-probing here would report "present" for
		 * every store.
		 */
		int coincide = urcu_txn__ryw_bloom_test_and_set(txn->ryw_bloom,
				slot);

		if (coincide && urcu_txn__eff_retry(txn) == 0)
			txn->esc_pending = 1;
#ifdef URCU_TXN_ESCALATION_STATS
		if (coincide)
			txn->esc_bloom++;	/* age-0 would escalate this store */
#endif
	}
#ifdef URCU_TXN_ESCALATION_STATS
	if (urcu_mcas_find(m, slot) != NULL)
		txn->esc_waw++;			/* true write-after-write */
#endif
	{
		bool recorded;

		/*
		 * Age 0: append blind, skipping the reconcile find (an O(nr)
		 * scan of the write set, run on every store) -- the store-path
		 * twin of the skip-find the age-0 load path above already does.
		 * Two callers reach it:
		 *  - RYW: the Bloom test_and_set above already flagged any
		 *    same-slot coincidence (real WAW or a false positive) with
		 *    esc_pending, and commit then DISCARDS this descriptor
		 *    before install -- so a transient duplicate record never
		 *    publishes.  On a Bloom miss (the disjoint case age 0
		 *    targets) find would miss anyway, so add is the identical
		 *    result minus the scan.  Age 1+ reconciles below.
		 *  - DISJOINT: the caller asserts distinct slots (no WAW), so
		 *    there is no coincidence to catch and no Bloom is
		 *    maintained -- the find is unconditionally redundant.  A
		 *    violated assertion corrupts (adds a duplicate record);
		 *    URCU_TXN_DEBUG_DISJOINT traps it here.
		 */
		if (urcu_txn__eff_retry(txn) == 0) {
#ifdef URCU_TXN_DEBUG_DISJOINT
			if (txn->disjoint && urcu_mcas_find(m, slot) != NULL) {
				fprintf(stderr, "urcu-txn: disjoint-contract violation: "
					"slot %p is already recorded in this commit, but the "
					"handle declared its write set disjoint via "
					"urcu_txn_declare_disjoint().  The blind append would "
					"insert a duplicate record and corrupt the commit.  Use "
					"the default (do not declare disjoint) for a mutator "
					"whose composed edits can alias a slot.\n", (void *) slot);
				abort();
			}
#endif
			recorded = urcu_mcas_add(m, slot, old_ptr, new_ptr, tag);
		} else
			recorded = urcu_txn__reconcile(txn, m, slot, old_ptr,
					new_ptr, upgrade, tag);
		if (caa_unlikely(!recorded)) {
			/* Descriptor full: grow (may move it) and retry. */
			m = urcu_mcas_grow(m);
			if (caa_unlikely(!m)) {
				urcu_mcas_destroy(txn->mcas);	/* unpublished: sync free */
				txn->mcas = URCU_TXN_ENOMEM;
				return -ENOMEM;
			}
			txn->mcas = m;
			if (urcu_txn__eff_retry(txn) == 0)
				urcu_mcas_add(m, slot, old_ptr, new_ptr, tag);
			else
				urcu_txn__reconcile(txn, m, slot, old_ptr,
						new_ptr, upgrade, tag);
		}
	}
	return 0;
}

/*
 * Read @slot within the bracket and return its logical value.  @txn is taken
 * regardless -- it binds the read to the bracket's RCU read-side section
 * structurally (a live handle exists only between begin and end), and it is the
 * seam where the wait-free escalation lane would add read validation: route
 * in-bracket reads here, not through urcu_mcas_read(), so that day is a
 * one-line change.  A plain observer outside any transaction reads with
 * urcu_mcas_read() directly.
 *
 * Read-your-own-writes is the default: the returned value reflects this
 * attempt's own buffered stores -- when the slot has already been recorded, its
 * pending new_ptr is returned -- so a traversal inside the bracket observes the
 * transaction's own edits and computes write sites against the structure as it
 * will be, not as it was.  @committed forces the committed logical value
 * instead (ignoring the write set), and a handle that declared its write set
 * disjoint never reads its own writes; both skip the write-set consult.  The
 * NULL-descriptor test keeps the miss free and makes an attempt's first loads
 * (before any store) free.
 *
 * One more state skips the consult: once ANY store has failed to allocate, the
 * descriptor is the sticky ENOMEM marker and no write set is left to read, so a
 * later in-bracket load silently returns @slot's COMMITTED value even though
 * this attempt "wrote" it.  A caller that commits is safe -- the commit reports
 * MEMORY_ERROR, so the stale reads are never acted on.  A caller that BAILS OUT
 * of the bracket without committing must not derive its return value (say
 * -EEXIST from a tombstone read) from a load issued after a store whose return
 * it did not check: that load may have missed its own pending write.
 */
static inline
void *urcu_txn__load(struct urcu_mcas_txn *txn, void **slot, uintptr_t tag,
		int optimistic, int committed)
{
#if URCU_TXN_FALLBACK_PER_COST_NUM
	txn->nload++;			/* attempt cost: the bracket's READ side */
#endif
	if (!committed && !txn->disjoint && txn->mcas != NULL
			&& txn->mcas != URCU_TXN_ENOMEM) {
		if (urcu_txn__eff_retry(txn) == 0) {
			/*
			 * Age 0 (optimistic): a Bloom hit is a possible
			 * read-after-write.  Flag it to escalate (commit aborts
			 * -> age 1+) and skip find entirely.  The value
			 * returned below is optimistic and may be stale, but
			 * esc_pending guarantees this attempt aborts, so age 1
			 * re-reads it with the resolved path.
			 */
			if (urcu_txn__ryw_bloom_test(txn->ryw_bloom, slot))
				txn->esc_pending = 1;
		} else {
#ifdef URCU_TXN_ESCALATION_STATS
		if (urcu_txn__ryw_bloom_test(txn->ryw_bloom, slot))
			txn->esc_bloom++;	/* age-0 would escalate this read */
#endif
#ifndef URCU_TXN_RYW_NO_BLOOM
		/*
		 * The Bloom filter gates the scan: a clear bit proves the slot
		 * is not in the write set (the common traversal miss) and skips
		 * find; a set bit is confirmed by find, which resolves a false
		 * positive.
		 */
		if (urcu_txn__ryw_bloom_test(txn->ryw_bloom, slot))
#endif
		{
			struct urcu_mcas_record *r = urcu_mcas_find(txn->mcas, slot);

			if (r != NULL) {
#ifdef URCU_TXN_ESCALATION_STATS
				txn->esc_raw++;		/* true read-after-write */
#endif
				return r->new_ptr;	/* this attempt's pending value:
							 * already in the write set, no
							 * physical read to classify */
			}
		}
		}
	}
	urcu_txn__rp_note(txn, slot, optimistic);	/* debug: read-policy */
	return optimistic ? urcu_mcas_read_optimistic(slot, tag)
			: urcu_mcas_read(slot, tag);
}

static inline
void *urcu_txn_load(struct urcu_mcas_txn *txn, void **slot, uintptr_t tag)
{
	return urcu_txn__load(txn, slot, tag, 0, 0);
}

/*
 * urcu_txn_load that ignores this attempt's own buffered stores and returns
 * @slot's COMMITTED logical value -- the scoped, per-read counterpart to the
 * default read-your-own-writes.  Use for a read that must see the world as
 * committed, ignoring a pending write this transaction made to the same slot.
 * Read-ONLY: its result must not be fed back as the old_ptr of a store to that
 * slot (a same-slot store chains onto the pending record and would mismatch the
 * committed value it was handed).
 */
static inline
void *urcu_txn_load_committed(struct urcu_mcas_txn *txn, void **slot,
		uintptr_t tag)
{
	return urcu_txn__load(txn, slot, tag, 0, 1);
}

/*
 * urcu_txn_load without helping an undecided transaction decide: forwards to
 * urcu_mcas_read_optimistic() (see it for why this is safe for a read set).
 * The value is the slot's logical value at the moment of the read, and commit()
 * reconciles it against the install-time physical value exactly as for
 * urcu_txn_load -- a value that moved aborts.  A stale read costs an abort, not
 * correctness, so this trades a rare extra retry for not dragging every reader
 * of a contended slot through the parking transaction's whole install.
 *
 * Route TRAVERSAL through here: reads that locate a write site and are checked
 * at commit anyway.  Keep urcu_txn_load() for a value the op must see settled
 * at the point it reads it.  RYW is honoured identically.
 */
static inline
void *urcu_txn_load_optimistic(struct urcu_mcas_txn *txn, void **slot,
		uintptr_t tag)
{
	return urcu_txn__load(txn, slot, tag, 1, 0);
}

/*
 * urcu_txn_load_committed that reads optimistically (see
 * urcu_txn_load_optimistic).  Same committed-view, read-only contract.
 */
static inline
void *urcu_txn_load_committed_optimistic(struct urcu_mcas_txn *txn, void **slot,
		uintptr_t tag)
{
	return urcu_txn__load(txn, slot, tag, 1, 1);
}

/*
 * Read @slot like urcu_txn_load AND pin it: besides returning its current
 * logical value, record a load-only guard so the commit succeeds only if @slot
 * still resolves to that value at the install point (records {v -> v}) -- a TM
 * read-set entry folding a read into the commit's conflict set, for a word the
 * op depends on but does not rewrite (e.g. a tombstone an insert must find
 * clear).  This is value-CAS semantics: the guard checks that @slot resolves to
 * that value AT the linearization point, not that it stayed unchanged
 * throughout.  The engine is A-B-A-safe (the install latch tolerates any
 * slot-value recurrence -- no use-after-free), so a value that recurs benignly
 * is fine; but if the op's correctness needs to DETECT an intervening change (a
 * true A-B-A where the "B" matters -- the slot toggled away and back), the
 * guard alone will not see it, and the embedder must carry its own
 * version/generation in the word.  A later store to @slot upgrades the guard to
 * a write in place; validate/read a given slot once per attempt.  Sticky on OOM
 * like store: the value is returned regardless and the pending commit reports
 * -ENOMEM.
 *
 * The value read is this attempt's pending one (read-your-own-writes), so
 * guarding a slot the transaction has already written re-affirms its own
 * pending value rather than the committed one (the chaining reconcile keeps the
 * record's committed old, which is what commit verifies).  In particular a
 * guard on a slot this transaction has tombstoned observes the tombstone -- the
 * self-conflict is visible instead of silently passing.
 */
static inline
void *urcu_txn_load_validate(struct urcu_mcas_txn *txn, void **slot,
		uintptr_t tag)
{
	void *v = urcu_txn_load(txn, slot, tag);

	(void) urcu_txn__record(txn, slot, v, v, 0, tag);
	return v;
}

/*
 * urcu_txn_load_validate that reads optimistically (urcu_txn_load_optimistic).
 * The guard is unchanged -- the commit still requires @slot to resolve to the
 * value returned here -- so an undecided parker observed as its logical old is
 * simply a guard on that old: it holds if the parker aborts, and aborts us if
 * it commits.  Exactly the outcome the helping read would have reached, one
 * attempt later.
 */
static inline
void *urcu_txn_load_validate_optimistic(struct urcu_mcas_txn *txn, void **slot,
		uintptr_t tag)
{
	void *v = urcu_txn_load_optimistic(txn, slot, tag);

	(void) urcu_txn__record(txn, slot, v, v, 0, tag);
	return v;
}

/*
 * Record a pure guard {@expected -> @expected} on @slot: the commit succeeds
 * only if @slot still holds @expected at the install point, and aborts
 * otherwise.  Like every record it PARKS A PROXY on @slot at plant time, so it
 * serializes against a concurrent writer of that word: whoever plants first
 * wins and the loser's plant CAS fails.
 *
 * The difference from urcu_txn_load_validate() is where the expected value
 * comes from: that one guards the value it READ, this one guards a value the
 * CALLER supplies.  So the guard can name a COMPUTED image of the word -- a
 * control word the consumer expects to find in a particular state, an image it
 * derived from other state, or a value it read outside the bracket -- which no
 * read-derived guard can express.  Everything else follows the record rules:
 *
 *   - Exact match, not a mask.  The install CAS compares the whole word, so
 *     @expected is the word the consumer predicts, subfields included; folding a
 *     status or marker bit into it is how one guards "the stable bits, with the
 *     marker in the state I expect".  A word that differs anywhere aborts.
 *   - Value-CAS semantics (as everywhere here): @slot is checked to HOLD
 *     @expected at the linearization point, not to have been stable throughout.
 *     An op whose correctness needs to DETECT an away-and-back change must carry
 *     its own version in the word.
 *   - Read-your-own-writes: on a slot this attempt has already recorded, the
 *     guard reconciles by CHAINING, so @expected must equal the record's PENDING
 *     new value, not the committed one.  A mismatch is a torn read-set: the
 *     attempt is poisoned and the commit aborts (a legal race, not a bug).
 *
 * Sticky on OOM like store: the pending commit reports -ENOMEM.
 */
static inline
void urcu_txn_validate(struct urcu_mcas_txn *txn, void **slot,
		void *expected, uintptr_t tag)
{
	(void) urcu_txn__record(txn, slot, expected, expected, 0, tag);
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
 * advanced internally), or MEMORY_ERROR on allocation failure (including a
 * store that could not allocate).  @call_rcu_fn has the flavor call_rcu
 * signature, so a flavor-agnostic embedder passes its RCU flavor's call_rcu
 * directly (e.g.  flavor->update_call_rcu) -- the same parametric-reclaim
 * contract as <urcu/rcu-txn-sw.h>, so an embedder migrating from the
 * single-updater front-end keeps its reclaim wiring.  The convenience wrapper
 * urcu_txn_commit() passes the compile-time-selected call_rcu.  Call between
 * begin and end.
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
	if (caa_unlikely(txn->esc_pending)) {
		/*
		 * Age-0 optimistic attempt saw a same-slot coincidence: its
		 * write-set was built against unresolved reads, so discard it
		 * (never published) and re-run at age 1+, where Bloom+find
		 * resolves RYW.
		 *
		 * Consume the flag here rather than leaving it to the next
		 * begin(): an embedder driving the engine WITHOUT begin()/end()
		 * (see urcu_txn_read_lock) has no begin() to clear it, and a
		 * flag that survives its own abort turns every later attempt --
		 * at any age -- into an unconditional abort.
		 */
		txn->esc_pending = 0;
		txn->min_alloc = m->nr;
		urcu_txn__learn_cost(txn);
		urcu_mcas_destroy(m);		/* unpublished: synchronous free */
		txn->mcas = NULL;
		txn->retry++;			/* -> age 1 */
		txn->retrying = 1;		/* keep the turn across the retry */
		return URCU_TXN_STATUS_ABORT;
	}
	txn->min_alloc = m->nr;		/* learn the realized size: a retry won't re-grow */
	/*
	 * Learn what this attempt COST: its reads plus its writes.  The write
	 * set alone understates a traversal-driven mutator badly -- a skiplist
	 * move commits ~25 records but descends over an order of magnitude
	 * more slots, and that descent is inside the bracket, so escalating
	 * serializes it too.
	 */
	urcu_txn__learn_cost(txn);
	txn->mcas = NULL;		/* mcas_commit consumes the descriptor */
	if (urcu_mcas_commit(m, call_rcu_fn))
		return URCU_TXN_STATUS_OK;
	txn->retry++;			/* aged for the next attempt */
	txn->retrying = 1;		/* keep the turn across the retry */
	return URCU_TXN_STATUS_ABORT;
}

/*
 * Commit deferring reclaim through the compile-time-selected RCU flavor's
 * call_rcu (so this header must be included after an RCU flavor header).  A
 * thin wrapper over urcu_txn_commit_flavor(); see it for the full contract.
 */
static inline
enum urcu_txn_status urcu_txn_commit(struct urcu_mcas_txn *txn)
{
	return urcu_txn_commit_flavor(txn, call_rcu);
}

/*
 * Note a contention retry that abandons the attempt BEFORE commit -- e.g. a
 * load-validate guard observed a neighbour mid-deletion and the mutator must
 * re-read rather than commit.  Advances aging and keeps the FIFO turn exactly
 * as a commit ABORT does, so such a guard-driven retry escalates into the
 * fallback lane instead of spinning: without this, an op that keeps hitting the
 * guard on a hot slot never reaches commit, so txn->retry never advances and it
 * can livelock.  Call after the guard fires and before end(), then
 * end()+begin() and re-attempt.  Like a commit ABORT this keeps the FIFO turn,
 * so a caller that gives up rather than re-attempting must call
 * urcu_txn_abandon() before end() -- see it.
 */
static inline
void urcu_txn_conflict(struct urcu_mcas_txn *txn)
{
	urcu_txn__learn_cost(txn);	/* the descent counts */
	txn->retry++;			/* aged: a guard storm now escalates */
	txn->retrying = 1;		/* keep the FIFO turn across the retry */
}

/*
 * Give up on the transaction instead of re-attempting it: the caller has taken
 * an ABORT (from commit, or from urcu_txn_conflict()) and will NOT run another
 * begin..commit -- it is walking away from the operation entirely.
 *
 * An ABORT is not a terminal outcome: it leaves retrying = 1 so that end()
 * KEEPS the handle's turn in the escalation lane and the next attempt re-enters
 * as the same head, which is what makes a starved transaction's progress
 * bounded.  A caller that just end()s and walks away after an ABORT therefore
 * never releases the domain's fair mutex -- the lane is held forever and every
 * writer in the domain stalls.  Call this before end() to forfeit the turn and
 * make that end() terminal.  Harmless on a handle that never escalated, or
 * never aborted.
 */
/* Cost of the last completed attempt: loads + write-set records. */
static inline
unsigned int urcu_txn_last_cost(const struct urcu_mcas_txn *txn)
{
	return txn->last_cost;
}

static inline
void urcu_txn_abandon(struct urcu_mcas_txn *txn)
{
	txn->retrying = 0;		/* forfeit the turn: end() releases it */
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
	if (uatomic_load(&txn->in_fallback, CMM_RELAXED) && !txn->retrying)
		urcu_txn__exit_fallback(txn);
}

#ifdef __cplusplus
}
#endif

#endif	/* _URCU_RCU_TXN_H */
