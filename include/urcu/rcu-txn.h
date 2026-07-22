// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

#ifndef _URCU_RCU_TXN_H
#define _URCU_RCU_TXN_H

/*
 * RCU transactions -- the general single-writer / multi-writer FRONT-END.
 *
 * A begin/load/store/commit bracket over the mixed MCAS
 * <urcu/rcu-txn-mcas.h>, with cost-scaled aging escalation and a per-domain
 * fair-mutex fallback lane.  This is the canonical transaction front-end:
 * store() is split into store_mw() and store_sw() so one commit can carry both
 * single-writer-owned and multi-writer slots, atomic against one linearization
 * point.  See <urcu/rcu-txn-mcas.h> for the MCAS mechanism (one control
 * word, two record kinds, MW-only abort).  <urcu/rcu-txn-sw.h> is the
 * single-writer-only specialization (urcu_txn_sw_*).
 *
 * Include AFTER an RCU flavor header (e.g. <urcu-qsbr.h>): the read-side bracket
 * and urcu_txn_commit() default to the compile-time-selected flavor's
 * rcu_read_lock / call_rcu, unless a flavor is bound with
 * urcu_txn_init_flavor().
 *
 * Two commit entry points:
 *   - urcu_txn_commit() -- the sw-mw-aware path; use it when the write-set
 *     may contain MW records (store_mw / a load-validate guard).
 *   - urcu_txn_commit_sw() -- for a write-set the embedder knows is
 *     store_sw-only; it skips the multi-writer machinery (partition, sort,
 *     CAS-install, abort) entirely and cannot contention-abort.
 */

#include <errno.h>
#include <limits.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <urcu/assert.h>
#include <urcu/compiler.h>
#include <urcu/uatomic.h>
#include <urcu/call-rcu.h>		/* struct rcu_head */
#include <urcu/fair-mutex.h>
#include <urcu/flavor.h>		/* struct rcu_flavor_struct */
#include <urcu/rcu-txn-mcas.h>	/* the mixed MCAS primitive */
#include <urcu/rcu-txn-bloom.h>		/* shared RYW lookup filter */
#include <urcu/rcu-txn-status.h>

#ifdef __cplusplus
extern "C" {
#endif

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

urcu_static_assert(URCU_TXN_FALLBACK_PER_COST_NUM >= 0,
		"URCU_TXN_FALLBACK_PER_COST_NUM must not be negative",
		urcu_txn_fallback_per_cost_num_nonnegative);
urcu_static_assert(URCU_TXN_FALLBACK_PER_COST_DEN > 0,
		"URCU_TXN_FALLBACK_PER_COST_DEN must be greater than zero",
		urcu_txn_fallback_per_cost_den_positive);
urcu_static_assert(URCU_TXN_FALLBACK_MIN <= URCU_TXN_FALLBACK_MAX,
		"URCU_TXN_FALLBACK_MIN must not exceed URCU_TXN_FALLBACK_MAX",
		urcu_txn_fallback_bounds_ordered);

/*
 * Read-your-own-writes lookup filter (a Bloom word), maintained for every
 * transaction except one that declared its write set disjoint.  urcu_txn__load
 * must, for each in-bracket read, decide whether the slot is already in this
 * attempt's write set.  The authoritative test is urcu_txn_find() -- a linear
 * scan of the descriptor's records at the record stride (48 bytes).  A
 * traversal reads many slots and transacts few, so the hot case is the MISS,
 * and that scan is pure overhead on it.
 *
 * The filter is one word in the ON-STACK handle (never in struct urcu_txn_desc,
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
 * The filter mechanism itself (width, k, hashing, test/set) is shared with
 * <urcu/rcu-txn-sw.h> and lives in <urcu/rcu-txn-bloom.h>; what is stated here
 * is this engine's POLICY for it.  Width and k are compile-time tunables that
 * only ever trade filter cost against the false-positive rate: correctness
 * never depends on either, since a false positive only ever costs a find
 * (baseline) or an extra attempt (age 0).  Escalation itself is tuned
 * per-transaction at runtime via urcu_txn_declare_disjoint() /
 * urcu_txn_expect_conflict().
 */

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
 * Compile-time fallback for the RCU read-side bracket, used only when a handle
 * binds no flavor.  Defaults to the compile-time-selected flavor.
 */
#ifndef URCU_TXN_RCU_READ_LOCK
#define URCU_TXN_RCU_READ_LOCK()	rcu_read_lock()
#endif
#ifndef URCU_TXN_RCU_READ_UNLOCK
#define URCU_TXN_RCU_READ_UNLOCK()	rcu_read_unlock()
#endif

/* Sticky out-of-memory marker parked in txn->desc by a failed store. */
#define URCU_TXN_ENOMEM	((struct urcu_txn_desc *) -1L)

/*
 * THE READ POLICY.  Wait iff the loaded slot enters this transaction's
 * read/write set; read optimistically ONLY to navigate.  The policy is a
 * property of the CALL SEQUENCE, not of contention, so it is checkable single-
 * threaded: for every slot entering the read/write set, the most recent load of
 * that slot in the same attempt must have been a waiting load
 * (urcu_txn_load / urcu_txn_load_validate), never an optimistic one.  Build with
 * -DURCU_TXN_DEBUG_READ_POLICY to enforce it -- every urcu_txn__record() is
 * checked against the kind of the last load, and a violation aborts.  Add
 * -DURCU_TXN_DEBUG_READ_POLICY_SOFT to count violations in the handle instead of
 * aborting.  The check costs a hash-table probe per load and per record and
 * grows the on-stack handle by URCU_TXN_RP_SLOTS entries.  Debug builds only.
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
 * Per-contention-domain escalation state, shared by every handle transacting the
 * same structure.  Pass &domain to init(), or NULL to disable the fallback.
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

struct urcu_txn {
	struct urcu_txn_domain *domain;	/* escalation domain, or NULL */
	const struct rcu_flavor_struct *flavor;	/* read-side bracket flavor, or NULL */
	unsigned long retry;		/* attempts so far; aging priority */
	unsigned int min_alloc;		/* floor for the initial descriptor capacity */
	struct urcu_txn_desc *desc;	/* this attempt's descriptor / ENOMEM marker */
	struct cds_fair_mutex_node waiter;	/* our node while awaiting the turn */
	int in_fallback;		/* we currently hold the lane */
	int fb_published;		/* we raised domain->active and owe the clear */
	int retrying;			/* commit asked retry: keep the turn */
	uint64_t ryw_bloom[URCU_TXN_BLOOM_WORDS];	/* read-your-own-writes filter */
	int disjoint;			/* write set touches DISTINCT slots */
	unsigned int nload;		/* loads issued by the CURRENT attempt */
	unsigned int last_cost;		/* high-water mark of completed attempts' costs */
	int expect_conflict;		/* caller expects this txn to conflict */
	int esc_pending;		/* age-0 optimistic attempt saw a coincidence */
#ifdef URCU_TXN_ESCALATION_STATS
	unsigned int esc_raw;
	unsigned int esc_waw;
	unsigned int esc_bloom;
#endif
#ifdef URCU_TXN_DEBUG_READ_POLICY
	struct urcu_txn__rp_entry rp[URCU_TXN_RP_SLOTS];
					/* Kind of the most recent load of each
					 * slot THIS attempt; cleared by begin(). */
	unsigned long rp_violations;	/* records made on a slot whose last load
					 * was optimistic; sums over the attempts. */
	unsigned long rp_evicted;	/* table overflowed: a mark was dropped, so
					 * a zero violation count is NOT a proof. */
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
void urcu_txn__rp_reset(struct urcu_txn *txn)
{
	memset(txn->rp, 0, sizeof(txn->rp));
}

/* Remember the kind of the most recent load of @slot. */
static inline
void urcu_txn__rp_note(struct urcu_txn *txn, void **slot, int optimistic)
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
 * @slot is entering the read/write set.  Its last load must have been a waiting
 * load (urcu_txn_load / urcu_txn_load_validate), not an optimistic one.  A slot
 * with no mark was never loaded here (a blind store): nothing to check.  Once
 * checked, the slot is settled -- a later store chains onto the record rather
 * than off a fresh physical read -- so clear the mark rather than report twice.
 */
static inline
void urcu_txn__rp_check(struct urcu_txn *txn, void **slot)
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

static inline
unsigned long urcu_txn_read_policy_violations(const struct urcu_txn *txn)
{
	return txn->rp_violations;
}

static inline
unsigned long urcu_txn_read_policy_evicted(const struct urcu_txn *txn)
{
	return txn->rp_evicted;
}

#else	/* !URCU_TXN_DEBUG_READ_POLICY */

# define urcu_txn__rp_reset(txn)		do { } while (0)
# define urcu_txn__rp_note(txn, slot, opt)	do { } while (0)
# define urcu_txn__rp_check(txn, slot)		do { } while (0)

static inline
unsigned long urcu_txn_read_policy_violations(const struct urcu_txn *txn)
{
	(void) txn;
	return 0;
}

static inline
unsigned long urcu_txn_read_policy_evicted(const struct urcu_txn *txn)
{
	(void) txn;
	return 0;
}

#endif	/* URCU_TXN_DEBUG_READ_POLICY */

static inline
void urcu_txn__bloom_reset(struct urcu_txn *txn)
{
	if (!txn->disjoint)
		memset(txn->ryw_bloom, 0, sizeof(txn->ryw_bloom));
}

/*
 * Initialize a handle before its retry loop, bracketing the txn's RCU read-side
 * section in @flavor (or the compile-time-selected flavor when NULL).
 */
static inline
void urcu_txn_init_flavor(struct urcu_txn *txn,
		struct urcu_txn_domain *domain,
		const struct rcu_flavor_struct *flavor)
{
	txn->domain = domain;
	txn->flavor = flavor;
	txn->retry = 0;
	txn->min_alloc = 0;
	txn->nload = 0;
	txn->last_cost = 0;
	txn->desc = NULL;
	uatomic_store(&txn->in_fallback, 0, CMM_RELAXED);
	txn->fb_published = 0;
	txn->retrying = 0;
	txn->disjoint = 0;
	txn->expect_conflict = 0;
	/* ryw_bloom is zeroed by __bloom_reset() at first descriptor alloc. */
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

/* Initialize a handle bracketed in the compile-time-selected RCU flavor. */
static inline
void urcu_txn_init(struct urcu_txn *txn,
		struct urcu_txn_domain *domain)
{
	urcu_txn_init_flavor(txn, domain, NULL);
}

#ifdef URCU_TXN_ESCALATION_STATS
static inline
unsigned int urcu_txn_esc_raw(const struct urcu_txn *txn) { return txn->esc_raw; }
static inline
unsigned int urcu_txn_esc_waw(const struct urcu_txn *txn) { return txn->esc_waw; }
static inline
unsigned int urcu_txn_esc_bloom(const struct urcu_txn *txn) { return txn->esc_bloom; }
#endif

/*
 * Assert this handle's transactions touch DISTINCT slots (no write-after-write,
 * no read-of-own-write).  Age 0 then blind-appends and maintains no RYW filter.
 * Call after init and before the first begin().
 */
static inline
void urcu_txn_declare_disjoint(struct urcu_txn *txn)
{
	txn->disjoint = 1;
}

/*
 * Assert this handle's transactions are EXPECTED to conflict, so the optimistic
 * age-0 attempt is skipped and the sorted, blocking path runs from the first
 * attempt.  Call after init and before the first begin().
 */
static inline
void urcu_txn_expect_conflict(struct urcu_txn *txn)
{
	txn->expect_conflict = 1;
}

/*
 * Effective attempt age: normally the retry count; an expect_conflict() handle
 * reports >= 1 even on its first attempt.
 */
static inline
unsigned long urcu_txn__eff_retry(const struct urcu_txn *txn)
{
	return (txn->expect_conflict && txn->retry == 0) ? 1UL : txn->retry;
}

static inline
void urcu_txn_read_lock(struct urcu_txn *txn)
{
	if (txn->flavor)
		txn->flavor->read_lock();
	else
		URCU_TXN_RCU_READ_LOCK();
}

static inline
void urcu_txn_read_unlock(struct urcu_txn *txn)
{
	if (txn->flavor)
		txn->flavor->read_unlock();
	else
		URCU_TXN_RCU_READ_UNLOCK();
}

/* What the current attempt has cost so far: slots read plus edges buffered. */
static inline
unsigned int urcu_txn__attempt_cost(const struct urcu_txn *txn)
{
	const struct urcu_txn_desc *m = txn->desc;
	unsigned int w = (m && m != URCU_TXN_ENOMEM) ? m->nr : 0;

	return txn->nload + w;
}

/* Learn this operation's cost as a HIGH-WATER MARK. */
static inline
void urcu_txn__learn_cost(struct urcu_txn *txn)
{
#if URCU_TXN_FALLBACK_PER_COST_NUM
	unsigned int c = urcu_txn__attempt_cost(txn);

	if (c > txn->last_cost)
		txn->last_cost = c;
#else
	(void) txn;
#endif
}

/* Retries this handle may spend on the optimistic path before it earns the lane. */
static inline
unsigned long urcu_txn__fallback_at(const struct urcu_txn *txn)
{
	const uint64_t num = URCU_TXN_FALLBACK_PER_COST_NUM;
	const uint64_t den = URCU_TXN_FALLBACK_PER_COST_DEN > 0 ?
		URCU_TXN_FALLBACK_PER_COST_DEN : 1;
	uint64_t n, t;

	if (!num)
		return URCU_TXN_FALLBACK;
	n = txn->last_cost ? txn->last_cost : 1;
	t = (num * n) / den;
	if (t < URCU_TXN_FALLBACK_MIN)
		t = URCU_TXN_FALLBACK_MIN;
	return t > URCU_TXN_FALLBACK_MAX ? URCU_TXN_FALLBACK_MAX : t;
}

static inline
int urcu_txn__self_qualifies(const struct urcu_txn *txn)
{
	return txn->retry >= urcu_txn__fallback_at(txn);
}

static inline
void urcu_txn__enter_fallback(struct urcu_txn *txn)
{
	cds_fair_mutex_lock(&txn->domain->lock, &txn->waiter);
	if (urcu_txn__self_qualifies(txn)) {
		uatomic_store(&txn->domain->active, 1, CMM_RELAXED);
		txn->fb_published = 1;
	}
	uatomic_store(&txn->in_fallback, 1, CMM_RELAXED);
}

static inline
void urcu_txn__exit_fallback(struct urcu_txn *txn)
{
	if (txn->fb_published) {
		uatomic_store(&txn->domain->active, 0, CMM_RELAXED);
		txn->fb_published = 0;
	}
	uatomic_store(&txn->in_fallback, 0, CMM_RELAXED);
	(void) cds_fair_mutex_unlock(&txn->domain->lock, &txn->waiter);
}

static inline
int urcu_txn__want_fallback(struct urcu_txn *txn)
{
	return txn->domain && !uatomic_load(&txn->in_fallback, CMM_RELAXED) &&
		(uatomic_load(&txn->domain->active, CMM_RELAXED) ||
		 urcu_txn__self_qualifies(txn));
}

static inline
void urcu_txn__maybe_publish(struct urcu_txn *txn)
{
	if (txn->domain && uatomic_load(&txn->in_fallback, CMM_RELAXED) &&
			!txn->fb_published && urcu_txn__self_qualifies(txn)) {
		uatomic_store(&txn->domain->active, 1, CMM_RELAXED);
		txn->fb_published = 1;
	}
}

/* Begin one attempt: clear the write-set and open the RCU read-side section. */
static inline
void urcu_txn_begin(struct urcu_txn *txn)
{
	txn->retrying = 0;
	txn->nload = 0;
	if (urcu_txn__want_fallback(txn))
		urcu_txn__enter_fallback(txn);
	else
		urcu_txn__maybe_publish(txn);
	txn->desc = NULL;
	txn->esc_pending = 0;
#ifdef URCU_TXN_ESCALATION_STATS
	txn->esc_raw = txn->esc_waw = txn->esc_bloom = 0;
#endif
	urcu_txn__rp_reset(txn);	/* debug: a new attempt reloads every slot */
	urcu_txn_read_lock(txn);
}

/*
 * Pre-reserve room for @n writes this attempt and record @n as the min_alloc
 * floor.  Returns 0, or -ENOMEM (sticky).  Optional; call after begin, before
 * the first store.
 */
static inline
int urcu_txn_reserve(struct urcu_txn *txn, unsigned int n)
{
	struct urcu_txn_desc *m;

	txn->min_alloc = n;
	if (caa_unlikely(txn->desc == URCU_TXN_ENOMEM))
		return -ENOMEM;
	if (!n)
		return 0;
	if (!txn->desc) {
		m = urcu_txn_create(n, urcu_txn__eff_retry(txn));
		if (caa_unlikely(!m)) {
			txn->desc = URCU_TXN_ENOMEM;
			return -ENOMEM;
		}
		urcu_txn__bloom_reset(txn);
		txn->desc = m;
		return 0;
	}
	while (txn->desc->cap < n) {
		m = urcu_txn_grow(txn->desc);
		if (caa_unlikely(!m)) {
			urcu_txn_destroy(txn->desc);
			txn->desc = URCU_TXN_ENOMEM;
			return -ENOMEM;
		}
		txn->desc = m;
	}
	return 0;
}

static inline
bool urcu_txn__reconcile(struct urcu_txn *txn,
		struct urcu_txn_desc *m, void **slot, void *old_ptr,
		void *new_ptr, int upgrade, uintptr_t tag, unsigned int kind)
{
	(void) txn;
	return urcu_txn_record_chain(m, slot, old_ptr, new_ptr, upgrade,
			tag, kind);
}

/*
 * Buffer or reconcile one record of kind @kind: lazily create the descriptor,
 * grow it if full, keep one record per slot.  @upgrade is 1 for a store, 0 for a
 * load-validate guard.  Returns 0, or -ENOMEM (sticky).
 */
static inline
int urcu_txn__record(struct urcu_txn *txn, void **slot,
		void *old_ptr, void *new_ptr, int upgrade, uintptr_t tag,
		unsigned int kind)
{
	struct urcu_txn_desc *m = txn->desc;

	urcu_txn__rp_check(txn, slot);	/* debug: @slot enters the read/write set */
	if (caa_unlikely(m == URCU_TXN_ENOMEM))
		return -ENOMEM;
	if (!m) {
		m = urcu_txn_create(txn->min_alloc ? txn->min_alloc :
				URCU_TXN_INIT, urcu_txn__eff_retry(txn));
		if (caa_unlikely(!m)) {
			txn->desc = URCU_TXN_ENOMEM;
			return -ENOMEM;
		}
		urcu_txn__bloom_reset(txn);
		txn->desc = m;
	}
	if (!txn->disjoint) {
		int coincide = urcu_txn__ryw_bloom_test_and_set(txn->ryw_bloom,
				slot);

		if (coincide && urcu_txn__eff_retry(txn) == 0)
			txn->esc_pending = 1;
#ifdef URCU_TXN_ESCALATION_STATS
		if (coincide)
			txn->esc_bloom++;
#endif
	}
#ifdef URCU_TXN_ESCALATION_STATS
	if (urcu_txn_find(m, slot) != NULL)
		txn->esc_waw++;
#endif
	{
		bool recorded;

		if (urcu_txn__eff_retry(txn) == 0) {
#ifdef URCU_TXN_DEBUG_DISJOINT
			if (txn->disjoint && urcu_txn_find(m, slot) != NULL) {
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
			recorded = urcu_txn_add(m, slot, old_ptr, new_ptr,
					tag, kind);
		} else
			recorded = urcu_txn__reconcile(txn, m, slot,
					old_ptr, new_ptr, upgrade, tag, kind);
		if (caa_unlikely(!recorded)) {
			m = urcu_txn_grow(m);
			if (caa_unlikely(!m)) {
				urcu_txn_destroy(txn->desc);
				txn->desc = URCU_TXN_ENOMEM;
				return -ENOMEM;
			}
			txn->desc = m;
			if (urcu_txn__eff_retry(txn) == 0)
				urcu_txn_add(m, slot, old_ptr, new_ptr,
						tag, kind);
			else
				urcu_txn__reconcile(txn, m, slot, old_ptr,
						new_ptr, upgrade, tag, kind);
		}
	}
	return 0;
}

static inline
void *urcu_txn__load(struct urcu_txn *txn, void **slot,
		uintptr_t tag, int optimistic, int committed)
{
#if URCU_TXN_FALLBACK_PER_COST_NUM
	txn->nload++;
#endif
	if (!committed && !txn->disjoint && txn->desc != NULL
			&& txn->desc != URCU_TXN_ENOMEM) {
		if (urcu_txn__eff_retry(txn) == 0) {
			if (urcu_txn__ryw_bloom_test(txn->ryw_bloom, slot))
				txn->esc_pending = 1;
		} else {
#ifdef URCU_TXN_ESCALATION_STATS
			if (urcu_txn__ryw_bloom_test(txn->ryw_bloom, slot))
				txn->esc_bloom++;
#endif
#ifndef URCU_TXN_RYW_NO_BLOOM
			if (urcu_txn__ryw_bloom_test(txn->ryw_bloom, slot))
#endif
			{
				struct urcu_txn_record *r =
					urcu_txn_find(txn->desc, slot);

				if (r != NULL) {
#ifdef URCU_TXN_ESCALATION_STATS
					txn->esc_raw++;
#endif
					return r->new_ptr;
				}
			}
		}
	}
	urcu_txn__rp_note(txn, slot, optimistic);	/* debug: read-policy */
	return optimistic ? urcu_txn_read_optimistic(slot, tag)
			: urcu_txn_read(slot, tag);
}

static inline
void *urcu_txn_load(struct urcu_txn *txn, void **slot,
		uintptr_t tag)
{
	return urcu_txn__load(txn, slot, tag, 0, 0);
}

static inline
void *urcu_txn_load_committed(struct urcu_txn *txn, void **slot,
		uintptr_t tag)
{
	return urcu_txn__load(txn, slot, tag, 0, 1);
}

static inline
void *urcu_txn_load_optimistic(struct urcu_txn *txn, void **slot,
		uintptr_t tag)
{
	return urcu_txn__load(txn, slot, tag, 1, 0);
}

static inline
void *urcu_txn_load_committed_optimistic(struct urcu_txn *txn,
		void **slot, uintptr_t tag)
{
	return urcu_txn__load(txn, slot, tag, 1, 1);
}

/*
 * Read @slot and pin it as an MW load-only guard: the commit succeeds only if
 * @slot still resolves to the returned value at the install point.  A guard is a
 * conflict-set entry, so it is MW-kind (it participates in CAS-old validation);
 * SW-owned slots are caller-exclusive and need no guard.
 */
static inline
void *urcu_txn_load_validate(struct urcu_txn *txn, void **slot,
		uintptr_t tag)
{
	void *v = urcu_txn_load(txn, slot, tag);

	(void) urcu_txn__record(txn, slot, v, v, 0, tag,
			URCU_TXN_KIND_MW);
	return v;
}

static inline
void *urcu_txn_load_validate_optimistic(struct urcu_txn *txn,
		void **slot, uintptr_t tag)
{
	void *v = urcu_txn_load_optimistic(txn, slot, tag);

	(void) urcu_txn__record(txn, slot, v, v, 0, tag,
			URCU_TXN_KIND_MW);
	return v;
}

/*
 * Record a pure MW guard {@expected -> @expected} on @slot: the commit succeeds
 * only if @slot still holds @expected at the install point.
 */
static inline
void urcu_txn_validate(struct urcu_txn *txn, void **slot,
		void *expected, uintptr_t tag)
{
	(void) urcu_txn__record(txn, slot, expected, expected, 0, tag,
			URCU_TXN_KIND_MW);
}

/*
 * Buffer a MULTI-WRITER write {*slot: old -> new}: the slot has concurrent
 * writers, so the commit installs it with a CAS-old and aborts on a mismatch.
 * Returns 0, or -ENOMEM (sticky).
 */
static inline
int urcu_txn_store_mw(struct urcu_txn *txn, void **slot,
		void *old_ptr, void *new_ptr, uintptr_t tag)
{
	return urcu_txn__record(txn, slot, old_ptr, new_ptr, 1, tag,
			URCU_TXN_KIND_MW);
}

/*
 * Buffer a SINGLE-WRITER write {*slot: old -> new}: the caller holds a lock over
 * @slot (no concurrent writer), so the commit parks it with a plain store that
 * never fails.  A transaction with only store_sw() records never contention-
 * aborts.  Returns 0, or -ENOMEM (sticky).
 *
 * SW is a PROMISE of exclusion that must hold for @slot across EVERY writer, not
 * just this one: a slot is SW xor MW, globally.  If any other transaction may
 * store_mw() the same slot, this park races that CAS -- store_mw() it here too.
 * See enum urcu_txn_kind in <urcu/rcu-txn-mcas.h>.
 */
static inline
int urcu_txn_store_sw(struct urcu_txn *txn, void **slot,
		void *old_ptr, void *new_ptr, uintptr_t tag)
{
	return urcu_txn__record(txn, slot, old_ptr, new_ptr, 1, tag,
			URCU_TXN_KIND_SW);
}

/*
 * Commit the buffered write-set (sw-mw-aware), deferring reclaim through
 * @call_rcu_fn.  Returns OK on commit, ABORT on a contention abort (MW-only; the
 * caller re-runs begin..commit), or MEMORY_ERROR on allocation failure.  Call
 * between begin and end.  See urcu_txn_commit_sw_flavor() for a write-set
 * known to be store_sw-only.
 */
static inline
enum urcu_txn_status urcu_txn_commit_flavor(struct urcu_txn *txn,
		void (*call_rcu_fn)(struct rcu_head *,
			void (*)(struct rcu_head *)))
{
	struct urcu_txn_desc *m = txn->desc;

	if (caa_unlikely(m == URCU_TXN_ENOMEM)) {
		txn->desc = NULL;
		return URCU_TXN_STATUS_MEMORY_ERROR;
	}
	if (!m)
		return URCU_TXN_STATUS_OK;
	if (caa_unlikely(txn->esc_pending)) {
		txn->esc_pending = 0;
		txn->min_alloc = m->nr;
		urcu_txn__learn_cost(txn);
		urcu_txn_destroy(m);
		txn->desc = NULL;
		txn->retry++;
		txn->retrying = 1;
		return URCU_TXN_STATUS_ABORT;
	}
	txn->min_alloc = m->nr;
	urcu_txn__learn_cost(txn);
	txn->desc = NULL;
	if (urcu_txn_desc_commit(m, call_rcu_fn))
		return URCU_TXN_STATUS_OK;
	txn->retry++;
	txn->retrying = 1;
	return URCU_TXN_STATUS_ABORT;
}

static inline
enum urcu_txn_status urcu_txn_commit(struct urcu_txn *txn)
{
	return urcu_txn_commit_flavor(txn, call_rcu);
}

/*
 * Commit assuming the write-set carries NO MW records (every store was
 * store_sw()): the branch-lean counterpart of urcu_txn_commit_flavor().
 * It publishes through urcu_txn_desc_commit_sw() -- park, flip, settle,
 * with no partition/sort/CAS/abort -- so a genuinely single-writer commit never
 * pays the multi-writer machinery.
 *
 * It still returns ABORT for the two non-contention retries the front-end owns:
 * an age-0 same-slot coincidence (esc_pending -> re-run at age 1+, where find
 * resolves read-your-own-writes) and a torn same-slot read-set (poisoned).  A
 * handle that declared disjoint or expect_conflict never hits the age-0 case, so
 * such a genuinely single-writer commit is abort-free.  Debug builds assert
 * every record is SW-kind.
 */
static inline
enum urcu_txn_status urcu_txn_commit_sw_flavor(struct urcu_txn *txn,
		void (*call_rcu_fn)(struct rcu_head *,
			void (*)(struct rcu_head *)))
{
	struct urcu_txn_desc *m = txn->desc;

	if (caa_unlikely(m == URCU_TXN_ENOMEM)) {
		txn->desc = NULL;
		return URCU_TXN_STATUS_MEMORY_ERROR;
	}
	if (!m)
		return URCU_TXN_STATUS_OK;
	if (caa_unlikely(txn->esc_pending)) {
		txn->esc_pending = 0;
		txn->min_alloc = m->nr;
		urcu_txn__learn_cost(txn);
		urcu_txn_destroy(m);
		txn->desc = NULL;
		txn->retry++;
		txn->retrying = 1;
		return URCU_TXN_STATUS_ABORT;
	}
	txn->min_alloc = m->nr;
	urcu_txn__learn_cost(txn);
	txn->desc = NULL;
	if (urcu_txn_desc_commit_sw(m, call_rcu_fn))
		return URCU_TXN_STATUS_OK;
	txn->retry++;			/* poisoned: torn read-set, re-run */
	txn->retrying = 1;
	return URCU_TXN_STATUS_ABORT;
}

static inline
enum urcu_txn_status urcu_txn_commit_sw(struct urcu_txn *txn)
{
	return urcu_txn_commit_sw_flavor(txn, call_rcu);
}

/*
 * Note a contention retry that abandons the attempt BEFORE commit.  Advances
 * aging and keeps the FIFO turn exactly as a commit ABORT does.  Call after the
 * guard fires and before end(), then end()+begin() and re-attempt.
 */
static inline
void urcu_txn_conflict(struct urcu_txn *txn)
{
	urcu_txn__learn_cost(txn);
	txn->retry++;
	txn->retrying = 1;
}

/* High-water cost across completed attempts: loads + write-set records. */
static inline
unsigned int urcu_txn_last_cost(const struct urcu_txn *txn)
{
	return txn->last_cost;
}

/*
 * Give up on the transaction instead of re-attempting after an ABORT: forfeits
 * the FIFO turn so end() releases the lane.  Call before end().
 */
static inline
void urcu_txn_abandon(struct urcu_txn *txn)
{
	txn->retrying = 0;
}

/* End the attempt: close the RCU read-side section.  Always pair with begin. */
static inline
void urcu_txn_end(struct urcu_txn *txn)
{
	struct urcu_txn_desc *m = txn->desc;

	if (m && m != URCU_TXN_ENOMEM)
		urcu_txn_destroy(m);
	txn->desc = NULL;
	urcu_txn_read_unlock(txn);
	if (uatomic_load(&txn->in_fallback, CMM_RELAXED) && !txn->retrying)
		urcu_txn__exit_fallback(txn);
}

#ifdef __cplusplus
}
#endif

#endif	/* _URCU_RCU_TXN_H */
