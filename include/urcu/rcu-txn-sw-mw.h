// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

#ifndef _URCU_RCU_TXN_SW_MW_H
#define _URCU_RCU_TXN_SW_MW_H

/*
 * RCU mixed single-writer / multi-writer transactions -- FRONT-END.
 *
 * A begin/load/store/commit bracket over the mixed engine
 * <urcu/rcu-mcas-sw-mw.h>, with cost-scaled aging escalation and a per-domain
 * fair-mutex fallback lane.  This is the mixed sibling of <urcu/rcu-txn.h>: same
 * front-end shape, but store() is split into store_mw() and store_sw() so one
 * commit can carry both single-writer-owned and multi-writer slots, atomic
 * against one linearization point.  See <urcu/rcu-mcas-sw-mw.h> for the engine
 * mechanism (one control word, two record kinds, MW-only abort).
 *
 * Include AFTER an RCU flavor header (e.g. <urcu-qsbr.h>): the read-side bracket
 * and urcu_txn_sw_mw_commit() default to the compile-time-selected flavor's
 * rcu_read_lock / call_rcu, unless a flavor is bound with
 * urcu_txn_sw_mw_init_flavor().
 *
 * Two commit entry points:
 *   - urcu_txn_sw_mw_commit() -- the sw-mw-aware path; use it when the write-set
 *     may contain MW records (store_mw / a load-validate guard).
 *   - urcu_txn_sw_mw_commit_sw() -- for a write-set the embedder knows is
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
#include <urcu/rcu-mcas-sw-mw.h>	/* the mixed engine */
#include <urcu/rcu-txn-bloom.h>		/* shared RYW lookup filter */
#include <urcu/rcu-txn-status.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef URCU_TXN_SW_MW_INIT
#define URCU_TXN_SW_MW_INIT	4
#endif

/* Reactive escalation threshold; see <urcu/rcu-txn.h> for the rationale. */
#ifndef URCU_TXN_SW_MW_FALLBACK_PER_COST_NUM
#define URCU_TXN_SW_MW_FALLBACK_PER_COST_NUM	11	/* 11/4 = 2.75 */
#endif
#ifndef URCU_TXN_SW_MW_FALLBACK_PER_COST_DEN
#define URCU_TXN_SW_MW_FALLBACK_PER_COST_DEN	4
#endif
#ifndef URCU_TXN_SW_MW_FALLBACK		/* only consulted when PER_COST_NUM == 0 */
#define URCU_TXN_SW_MW_FALLBACK		256
#endif
#ifndef URCU_TXN_SW_MW_FALLBACK_MIN
#define URCU_TXN_SW_MW_FALLBACK_MIN		64
#endif
#ifndef URCU_TXN_SW_MW_FALLBACK_MAX
#define URCU_TXN_SW_MW_FALLBACK_MAX		4096
#endif

urcu_static_assert(URCU_TXN_SW_MW_FALLBACK_PER_COST_NUM >= 0,
		"URCU_TXN_SW_MW_FALLBACK_PER_COST_NUM must not be negative",
		urcu_txn_sw_mw_fallback_per_cost_num_nonnegative);
urcu_static_assert(URCU_TXN_SW_MW_FALLBACK_PER_COST_DEN > 0,
		"URCU_TXN_SW_MW_FALLBACK_PER_COST_DEN must be greater than zero",
		urcu_txn_sw_mw_fallback_per_cost_den_positive);
urcu_static_assert(URCU_TXN_SW_MW_FALLBACK_MIN <= URCU_TXN_SW_MW_FALLBACK_MAX,
		"URCU_TXN_SW_MW_FALLBACK_MIN must not exceed URCU_TXN_SW_MW_FALLBACK_MAX",
		urcu_txn_sw_mw_fallback_bounds_ordered);

/*
 * Compile-time fallback for the RCU read-side bracket, used only when a handle
 * binds no flavor.  Defaults to the compile-time-selected flavor.
 */
#ifndef URCU_TXN_SW_MW_RCU_READ_LOCK
#define URCU_TXN_SW_MW_RCU_READ_LOCK()	rcu_read_lock()
#endif
#ifndef URCU_TXN_SW_MW_RCU_READ_UNLOCK
#define URCU_TXN_SW_MW_RCU_READ_UNLOCK()	rcu_read_unlock()
#endif

/* Sticky out-of-memory marker parked in txn->desc by a failed store. */
#define URCU_TXN_SW_MW_ENOMEM	((struct urcu_txn_sw_mw_desc *) -1L)

/*
 * Per-contention-domain escalation state, shared by every handle transacting the
 * same structure.  Pass &domain to init(), or NULL to disable the fallback.
 */
struct urcu_txn_sw_mw_domain {
	struct cds_fair_mutex lock;	/* the fair escalation lane */
	unsigned long active;		/* a fallback episode is in progress */
};

static inline
void urcu_txn_sw_mw_domain_init(struct urcu_txn_sw_mw_domain *d)
{
	cds_fair_mutex_init(&d->lock);
	d->active = 0;
}

struct urcu_txn_sw_mw_txn {
	struct urcu_txn_sw_mw_domain *domain;	/* escalation domain, or NULL */
	const struct rcu_flavor_struct *flavor;	/* read-side bracket flavor, or NULL */
	unsigned long retry;		/* attempts so far; aging priority */
	unsigned int min_alloc;		/* floor for the initial descriptor capacity */
	struct urcu_txn_sw_mw_desc *desc;	/* this attempt's descriptor / ENOMEM marker */
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
#ifdef URCU_TXN_SW_MW_ESCALATION_STATS
	unsigned int esc_raw;
	unsigned int esc_waw;
	unsigned int esc_bloom;
#endif
};

static inline
void urcu_txn_sw_mw__bloom_reset(struct urcu_txn_sw_mw_txn *txn)
{
	if (!txn->disjoint)
		memset(txn->ryw_bloom, 0, sizeof(txn->ryw_bloom));
}

/*
 * Initialize a handle before its retry loop, bracketing the txn's RCU read-side
 * section in @flavor (or the compile-time-selected flavor when NULL).
 */
static inline
void urcu_txn_sw_mw_init_flavor(struct urcu_txn_sw_mw_txn *txn,
		struct urcu_txn_sw_mw_domain *domain,
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
#ifdef URCU_TXN_SW_MW_ESCALATION_STATS
	txn->esc_raw = txn->esc_waw = txn->esc_bloom = 0;
#endif
}

/* Initialize a handle bracketed in the compile-time-selected RCU flavor. */
static inline
void urcu_txn_sw_mw_init(struct urcu_txn_sw_mw_txn *txn,
		struct urcu_txn_sw_mw_domain *domain)
{
	urcu_txn_sw_mw_init_flavor(txn, domain, NULL);
}

#ifdef URCU_TXN_SW_MW_ESCALATION_STATS
static inline
unsigned int urcu_txn_sw_mw_esc_raw(const struct urcu_txn_sw_mw_txn *txn) { return txn->esc_raw; }
static inline
unsigned int urcu_txn_sw_mw_esc_waw(const struct urcu_txn_sw_mw_txn *txn) { return txn->esc_waw; }
static inline
unsigned int urcu_txn_sw_mw_esc_bloom(const struct urcu_txn_sw_mw_txn *txn) { return txn->esc_bloom; }
#endif

/*
 * Assert this handle's transactions touch DISTINCT slots (no write-after-write,
 * no read-of-own-write).  Age 0 then blind-appends and maintains no RYW filter.
 * Call after init and before the first begin().
 */
static inline
void urcu_txn_sw_mw_declare_disjoint(struct urcu_txn_sw_mw_txn *txn)
{
	txn->disjoint = 1;
}

/*
 * Assert this handle's transactions are EXPECTED to conflict, so the optimistic
 * age-0 attempt is skipped and the sorted, blocking path runs from the first
 * attempt.  Call after init and before the first begin().
 */
static inline
void urcu_txn_sw_mw_expect_conflict(struct urcu_txn_sw_mw_txn *txn)
{
	txn->expect_conflict = 1;
}

/*
 * Effective attempt age: normally the retry count; an expect_conflict() handle
 * reports >= 1 even on its first attempt.
 */
static inline
unsigned long urcu_txn_sw_mw__eff_retry(const struct urcu_txn_sw_mw_txn *txn)
{
	return (txn->expect_conflict && txn->retry == 0) ? 1UL : txn->retry;
}

static inline
void urcu_txn_sw_mw_read_lock(struct urcu_txn_sw_mw_txn *txn)
{
	if (txn->flavor)
		txn->flavor->read_lock();
	else
		URCU_TXN_SW_MW_RCU_READ_LOCK();
}

static inline
void urcu_txn_sw_mw_read_unlock(struct urcu_txn_sw_mw_txn *txn)
{
	if (txn->flavor)
		txn->flavor->read_unlock();
	else
		URCU_TXN_SW_MW_RCU_READ_UNLOCK();
}

/* What the current attempt has cost so far: slots read plus edges buffered. */
static inline
unsigned int urcu_txn_sw_mw__attempt_cost(const struct urcu_txn_sw_mw_txn *txn)
{
	const struct urcu_txn_sw_mw_desc *m = txn->desc;
	unsigned int w = (m && m != URCU_TXN_SW_MW_ENOMEM) ? m->nr : 0;

	return txn->nload + w;
}

/* Learn this operation's cost as a HIGH-WATER MARK. */
static inline
void urcu_txn_sw_mw__learn_cost(struct urcu_txn_sw_mw_txn *txn)
{
#if URCU_TXN_SW_MW_FALLBACK_PER_COST_NUM
	unsigned int c = urcu_txn_sw_mw__attempt_cost(txn);

	if (c > txn->last_cost)
		txn->last_cost = c;
#else
	(void) txn;
#endif
}

/* Retries this handle may spend on the optimistic path before it earns the lane. */
static inline
unsigned long urcu_txn_sw_mw__fallback_at(const struct urcu_txn_sw_mw_txn *txn)
{
	const uint64_t num = URCU_TXN_SW_MW_FALLBACK_PER_COST_NUM;
	const uint64_t den = URCU_TXN_SW_MW_FALLBACK_PER_COST_DEN > 0 ?
		URCU_TXN_SW_MW_FALLBACK_PER_COST_DEN : 1;
	uint64_t n, t;

	if (!num)
		return URCU_TXN_SW_MW_FALLBACK;
	n = txn->last_cost ? txn->last_cost : 1;
	t = (num * n) / den;
	if (t < URCU_TXN_SW_MW_FALLBACK_MIN)
		t = URCU_TXN_SW_MW_FALLBACK_MIN;
	return t > URCU_TXN_SW_MW_FALLBACK_MAX ? URCU_TXN_SW_MW_FALLBACK_MAX : t;
}

static inline
int urcu_txn_sw_mw__self_qualifies(const struct urcu_txn_sw_mw_txn *txn)
{
	return txn->retry >= urcu_txn_sw_mw__fallback_at(txn);
}

static inline
void urcu_txn_sw_mw__enter_fallback(struct urcu_txn_sw_mw_txn *txn)
{
	cds_fair_mutex_lock(&txn->domain->lock, &txn->waiter);
	if (urcu_txn_sw_mw__self_qualifies(txn)) {
		uatomic_store(&txn->domain->active, 1, CMM_RELAXED);
		txn->fb_published = 1;
	}
	uatomic_store(&txn->in_fallback, 1, CMM_RELAXED);
}

static inline
void urcu_txn_sw_mw__exit_fallback(struct urcu_txn_sw_mw_txn *txn)
{
	if (txn->fb_published) {
		uatomic_store(&txn->domain->active, 0, CMM_RELAXED);
		txn->fb_published = 0;
	}
	uatomic_store(&txn->in_fallback, 0, CMM_RELAXED);
	(void) cds_fair_mutex_unlock(&txn->domain->lock, &txn->waiter);
}

static inline
int urcu_txn_sw_mw__want_fallback(struct urcu_txn_sw_mw_txn *txn)
{
	return txn->domain && !uatomic_load(&txn->in_fallback, CMM_RELAXED) &&
		(uatomic_load(&txn->domain->active, CMM_RELAXED) ||
		 urcu_txn_sw_mw__self_qualifies(txn));
}

static inline
void urcu_txn_sw_mw__maybe_publish(struct urcu_txn_sw_mw_txn *txn)
{
	if (txn->domain && uatomic_load(&txn->in_fallback, CMM_RELAXED) &&
			!txn->fb_published && urcu_txn_sw_mw__self_qualifies(txn)) {
		uatomic_store(&txn->domain->active, 1, CMM_RELAXED);
		txn->fb_published = 1;
	}
}

/* Begin one attempt: clear the write-set and open the RCU read-side section. */
static inline
void urcu_txn_sw_mw_begin(struct urcu_txn_sw_mw_txn *txn)
{
	txn->retrying = 0;
	txn->nload = 0;
	if (urcu_txn_sw_mw__want_fallback(txn))
		urcu_txn_sw_mw__enter_fallback(txn);
	else
		urcu_txn_sw_mw__maybe_publish(txn);
	txn->desc = NULL;
	txn->esc_pending = 0;
#ifdef URCU_TXN_SW_MW_ESCALATION_STATS
	txn->esc_raw = txn->esc_waw = txn->esc_bloom = 0;
#endif
	urcu_txn_sw_mw_read_lock(txn);
}

/*
 * Pre-reserve room for @n writes this attempt and record @n as the min_alloc
 * floor.  Returns 0, or -ENOMEM (sticky).  Optional; call after begin, before
 * the first store.
 */
static inline
int urcu_txn_sw_mw_reserve(struct urcu_txn_sw_mw_txn *txn, unsigned int n)
{
	struct urcu_txn_sw_mw_desc *m;

	txn->min_alloc = n;
	if (caa_unlikely(txn->desc == URCU_TXN_SW_MW_ENOMEM))
		return -ENOMEM;
	if (!n)
		return 0;
	if (!txn->desc) {
		m = urcu_txn_sw_mw_create(n, urcu_txn_sw_mw__eff_retry(txn));
		if (caa_unlikely(!m)) {
			txn->desc = URCU_TXN_SW_MW_ENOMEM;
			return -ENOMEM;
		}
		urcu_txn_sw_mw__bloom_reset(txn);
		txn->desc = m;
		return 0;
	}
	while (txn->desc->cap < n) {
		m = urcu_txn_sw_mw_grow(txn->desc);
		if (caa_unlikely(!m)) {
			urcu_txn_sw_mw_destroy(txn->desc);
			txn->desc = URCU_TXN_SW_MW_ENOMEM;
			return -ENOMEM;
		}
		txn->desc = m;
	}
	return 0;
}

static inline
bool urcu_txn_sw_mw__reconcile(struct urcu_txn_sw_mw_txn *txn,
		struct urcu_txn_sw_mw_desc *m, void **slot, void *old_ptr,
		void *new_ptr, int upgrade, uintptr_t tag, unsigned int kind)
{
	(void) txn;
	return urcu_txn_sw_mw_record_chain(m, slot, old_ptr, new_ptr, upgrade,
			tag, kind);
}

/*
 * Buffer or reconcile one record of kind @kind: lazily create the descriptor,
 * grow it if full, keep one record per slot.  @upgrade is 1 for a store, 0 for a
 * load-validate guard.  Returns 0, or -ENOMEM (sticky).
 */
static inline
int urcu_txn_sw_mw__record(struct urcu_txn_sw_mw_txn *txn, void **slot,
		void *old_ptr, void *new_ptr, int upgrade, uintptr_t tag,
		unsigned int kind)
{
	struct urcu_txn_sw_mw_desc *m = txn->desc;

	if (caa_unlikely(m == URCU_TXN_SW_MW_ENOMEM))
		return -ENOMEM;
	if (!m) {
		m = urcu_txn_sw_mw_create(txn->min_alloc ? txn->min_alloc :
				URCU_TXN_SW_MW_INIT, urcu_txn_sw_mw__eff_retry(txn));
		if (caa_unlikely(!m)) {
			txn->desc = URCU_TXN_SW_MW_ENOMEM;
			return -ENOMEM;
		}
		urcu_txn_sw_mw__bloom_reset(txn);
		txn->desc = m;
	}
	if (!txn->disjoint) {
		int coincide = urcu_txn__ryw_bloom_test_and_set(txn->ryw_bloom,
				slot);

		if (coincide && urcu_txn_sw_mw__eff_retry(txn) == 0)
			txn->esc_pending = 1;
#ifdef URCU_TXN_SW_MW_ESCALATION_STATS
		if (coincide)
			txn->esc_bloom++;
#endif
	}
#ifdef URCU_TXN_SW_MW_ESCALATION_STATS
	if (urcu_txn_sw_mw_find(m, slot) != NULL)
		txn->esc_waw++;
#endif
	{
		bool recorded;

		if (urcu_txn_sw_mw__eff_retry(txn) == 0)
			recorded = urcu_txn_sw_mw_add(m, slot, old_ptr, new_ptr,
					tag, kind);
		else
			recorded = urcu_txn_sw_mw__reconcile(txn, m, slot,
					old_ptr, new_ptr, upgrade, tag, kind);
		if (caa_unlikely(!recorded)) {
			m = urcu_txn_sw_mw_grow(m);
			if (caa_unlikely(!m)) {
				urcu_txn_sw_mw_destroy(txn->desc);
				txn->desc = URCU_TXN_SW_MW_ENOMEM;
				return -ENOMEM;
			}
			txn->desc = m;
			if (urcu_txn_sw_mw__eff_retry(txn) == 0)
				urcu_txn_sw_mw_add(m, slot, old_ptr, new_ptr,
						tag, kind);
			else
				urcu_txn_sw_mw__reconcile(txn, m, slot, old_ptr,
						new_ptr, upgrade, tag, kind);
		}
	}
	return 0;
}

static inline
void *urcu_txn_sw_mw__load(struct urcu_txn_sw_mw_txn *txn, void **slot,
		uintptr_t tag, int optimistic, int committed)
{
#if URCU_TXN_SW_MW_FALLBACK_PER_COST_NUM
	txn->nload++;
#endif
	if (!committed && !txn->disjoint && txn->desc != NULL
			&& txn->desc != URCU_TXN_SW_MW_ENOMEM) {
		if (urcu_txn_sw_mw__eff_retry(txn) == 0) {
			if (urcu_txn__ryw_bloom_test(txn->ryw_bloom, slot))
				txn->esc_pending = 1;
		} else {
#ifdef URCU_TXN_SW_MW_ESCALATION_STATS
			if (urcu_txn__ryw_bloom_test(txn->ryw_bloom, slot))
				txn->esc_bloom++;
#endif
#ifndef URCU_TXN_SW_MW_RYW_NO_BLOOM
			if (urcu_txn__ryw_bloom_test(txn->ryw_bloom, slot))
#endif
			{
				struct urcu_txn_sw_mw_record *r =
					urcu_txn_sw_mw_find(txn->desc, slot);

				if (r != NULL) {
#ifdef URCU_TXN_SW_MW_ESCALATION_STATS
					txn->esc_raw++;
#endif
					return r->new_ptr;
				}
			}
		}
	}
	return optimistic ? urcu_txn_sw_mw_read_optimistic(slot, tag)
			: urcu_txn_sw_mw_read(slot, tag);
}

static inline
void *urcu_txn_sw_mw_load(struct urcu_txn_sw_mw_txn *txn, void **slot,
		uintptr_t tag)
{
	return urcu_txn_sw_mw__load(txn, slot, tag, 0, 0);
}

static inline
void *urcu_txn_sw_mw_load_committed(struct urcu_txn_sw_mw_txn *txn, void **slot,
		uintptr_t tag)
{
	return urcu_txn_sw_mw__load(txn, slot, tag, 0, 1);
}

static inline
void *urcu_txn_sw_mw_load_optimistic(struct urcu_txn_sw_mw_txn *txn, void **slot,
		uintptr_t tag)
{
	return urcu_txn_sw_mw__load(txn, slot, tag, 1, 0);
}

static inline
void *urcu_txn_sw_mw_load_committed_optimistic(struct urcu_txn_sw_mw_txn *txn,
		void **slot, uintptr_t tag)
{
	return urcu_txn_sw_mw__load(txn, slot, tag, 1, 1);
}

/*
 * Read @slot and pin it as an MW load-only guard: the commit succeeds only if
 * @slot still resolves to the returned value at the install point.  A guard is a
 * conflict-set entry, so it is MW-kind (it participates in CAS-old validation);
 * SW-owned slots are caller-exclusive and need no guard.
 */
static inline
void *urcu_txn_sw_mw_load_validate(struct urcu_txn_sw_mw_txn *txn, void **slot,
		uintptr_t tag)
{
	void *v = urcu_txn_sw_mw_load(txn, slot, tag);

	(void) urcu_txn_sw_mw__record(txn, slot, v, v, 0, tag,
			URCU_TXN_SW_MW_KIND_MW);
	return v;
}

static inline
void *urcu_txn_sw_mw_load_validate_optimistic(struct urcu_txn_sw_mw_txn *txn,
		void **slot, uintptr_t tag)
{
	void *v = urcu_txn_sw_mw_load_optimistic(txn, slot, tag);

	(void) urcu_txn_sw_mw__record(txn, slot, v, v, 0, tag,
			URCU_TXN_SW_MW_KIND_MW);
	return v;
}

/*
 * Record a pure MW guard {@expected -> @expected} on @slot: the commit succeeds
 * only if @slot still holds @expected at the install point.
 */
static inline
void urcu_txn_sw_mw_validate(struct urcu_txn_sw_mw_txn *txn, void **slot,
		void *expected, uintptr_t tag)
{
	(void) urcu_txn_sw_mw__record(txn, slot, expected, expected, 0, tag,
			URCU_TXN_SW_MW_KIND_MW);
}

/*
 * Buffer a MULTI-WRITER write {*slot: old -> new}: the slot has concurrent
 * writers, so the commit installs it with a CAS-old and aborts on a mismatch.
 * Returns 0, or -ENOMEM (sticky).
 */
static inline
int urcu_txn_sw_mw_store_mw(struct urcu_txn_sw_mw_txn *txn, void **slot,
		void *old_ptr, void *new_ptr, uintptr_t tag)
{
	return urcu_txn_sw_mw__record(txn, slot, old_ptr, new_ptr, 1, tag,
			URCU_TXN_SW_MW_KIND_MW);
}

/*
 * Buffer a SINGLE-WRITER write {*slot: old -> new}: the caller holds a lock over
 * @slot (no concurrent writer), so the commit parks it with a plain store that
 * never fails.  A transaction with only store_sw() records never contention-
 * aborts.  Returns 0, or -ENOMEM (sticky).
 */
static inline
int urcu_txn_sw_mw_store_sw(struct urcu_txn_sw_mw_txn *txn, void **slot,
		void *old_ptr, void *new_ptr, uintptr_t tag)
{
	return urcu_txn_sw_mw__record(txn, slot, old_ptr, new_ptr, 1, tag,
			URCU_TXN_SW_MW_KIND_SW);
}

/*
 * Commit the buffered write-set (sw-mw-aware), deferring reclaim through
 * @call_rcu_fn.  Returns OK on commit, ABORT on a contention abort (MW-only; the
 * caller re-runs begin..commit), or MEMORY_ERROR on allocation failure.  Call
 * between begin and end.  See urcu_txn_sw_mw_commit_sw_flavor() for a write-set
 * known to be store_sw-only.
 */
static inline
enum urcu_txn_status urcu_txn_sw_mw_commit_flavor(struct urcu_txn_sw_mw_txn *txn,
		void (*call_rcu_fn)(struct rcu_head *,
			void (*)(struct rcu_head *)))
{
	struct urcu_txn_sw_mw_desc *m = txn->desc;

	if (caa_unlikely(m == URCU_TXN_SW_MW_ENOMEM)) {
		txn->desc = NULL;
		return URCU_TXN_STATUS_MEMORY_ERROR;
	}
	if (!m)
		return URCU_TXN_STATUS_OK;
	if (caa_unlikely(txn->esc_pending)) {
		txn->esc_pending = 0;
		txn->min_alloc = m->nr;
		urcu_txn_sw_mw__learn_cost(txn);
		urcu_txn_sw_mw_destroy(m);
		txn->desc = NULL;
		txn->retry++;
		txn->retrying = 1;
		return URCU_TXN_STATUS_ABORT;
	}
	txn->min_alloc = m->nr;
	urcu_txn_sw_mw__learn_cost(txn);
	txn->desc = NULL;
	if (urcu_txn_sw_mw_desc_commit(m, call_rcu_fn))
		return URCU_TXN_STATUS_OK;
	txn->retry++;
	txn->retrying = 1;
	return URCU_TXN_STATUS_ABORT;
}

static inline
enum urcu_txn_status urcu_txn_sw_mw_commit(struct urcu_txn_sw_mw_txn *txn)
{
	return urcu_txn_sw_mw_commit_flavor(txn, call_rcu);
}

/*
 * Commit assuming the write-set carries NO MW records (every store was
 * store_sw()): the branch-lean counterpart of urcu_txn_sw_mw_commit_flavor().
 * It publishes through urcu_txn_sw_mw_desc_commit_sw() -- park, flip, settle,
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
enum urcu_txn_status urcu_txn_sw_mw_commit_sw_flavor(struct urcu_txn_sw_mw_txn *txn,
		void (*call_rcu_fn)(struct rcu_head *,
			void (*)(struct rcu_head *)))
{
	struct urcu_txn_sw_mw_desc *m = txn->desc;

	if (caa_unlikely(m == URCU_TXN_SW_MW_ENOMEM)) {
		txn->desc = NULL;
		return URCU_TXN_STATUS_MEMORY_ERROR;
	}
	if (!m)
		return URCU_TXN_STATUS_OK;
	if (caa_unlikely(txn->esc_pending)) {
		txn->esc_pending = 0;
		txn->min_alloc = m->nr;
		urcu_txn_sw_mw__learn_cost(txn);
		urcu_txn_sw_mw_destroy(m);
		txn->desc = NULL;
		txn->retry++;
		txn->retrying = 1;
		return URCU_TXN_STATUS_ABORT;
	}
	txn->min_alloc = m->nr;
	urcu_txn_sw_mw__learn_cost(txn);
	txn->desc = NULL;
	if (urcu_txn_sw_mw_desc_commit_sw(m, call_rcu_fn))
		return URCU_TXN_STATUS_OK;
	txn->retry++;			/* poisoned: torn read-set, re-run */
	txn->retrying = 1;
	return URCU_TXN_STATUS_ABORT;
}

static inline
enum urcu_txn_status urcu_txn_sw_mw_commit_sw(struct urcu_txn_sw_mw_txn *txn)
{
	return urcu_txn_sw_mw_commit_sw_flavor(txn, call_rcu);
}

/*
 * Note a contention retry that abandons the attempt BEFORE commit.  Advances
 * aging and keeps the FIFO turn exactly as a commit ABORT does.  Call after the
 * guard fires and before end(), then end()+begin() and re-attempt.
 */
static inline
void urcu_txn_sw_mw_conflict(struct urcu_txn_sw_mw_txn *txn)
{
	urcu_txn_sw_mw__learn_cost(txn);
	txn->retry++;
	txn->retrying = 1;
}

/* High-water cost across completed attempts: loads + write-set records. */
static inline
unsigned int urcu_txn_sw_mw_last_cost(const struct urcu_txn_sw_mw_txn *txn)
{
	return txn->last_cost;
}

/*
 * Give up on the transaction instead of re-attempting after an ABORT: forfeits
 * the FIFO turn so end() releases the lane.  Call before end().
 */
static inline
void urcu_txn_sw_mw_abandon(struct urcu_txn_sw_mw_txn *txn)
{
	txn->retrying = 0;
}

/* End the attempt: close the RCU read-side section.  Always pair with begin. */
static inline
void urcu_txn_sw_mw_end(struct urcu_txn_sw_mw_txn *txn)
{
	struct urcu_txn_sw_mw_desc *m = txn->desc;

	if (m && m != URCU_TXN_SW_MW_ENOMEM)
		urcu_txn_sw_mw_destroy(m);
	txn->desc = NULL;
	urcu_txn_sw_mw_read_unlock(txn);
	if (uatomic_load(&txn->in_fallback, CMM_RELAXED) && !txn->retrying)
		urcu_txn_sw_mw__exit_fallback(txn);
}

#ifdef __cplusplus
}
#endif

#endif	/* _URCU_RCU_TXN_SW_MW_H */
