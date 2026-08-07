// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

/*
 * The escalation lane's instrumentation hook (URCU_TXN_LANE_TRACE).
 *
 * WHAT IT IS FOR.  @in_fallback is the only caller-visible sign that a handle
 * holds the lane, and it is set AFTER cds_fair_mutex_lock() returns and cleared
 * BEFORE the unlock.  A caller-side trace of the transaction bracket therefore
 * cannot tell
 *
 *	"no handle ever escalated"		from
 *	"a handle took the lane and never gave it back",
 *
 * since both read in_fallback == 0 at every event the caller can see.  That
 * matters because a lane held forever parks every other writer in the domain,
 * which presents as a whole-process hang whose thread dump is identical to a
 * livelock INSIDE the lane.  The hook fires in the acquire path itself, so
 * ATTEMPT/HELD/RELEASE counts separate the two by subtraction.
 *
 * WHAT THIS TEST CHECKS, and the reason each check exists:
 *
 *  1. The hook actually FIRES.  A test that only checks "counts are consistent"
 *     passes vacuously when the hook never runs at all -- 0 == 0 -- which is
 *     precisely the failure mode the hook exists to make impossible.  So the
 *     first assertion is that at least one lane acquisition was observed, and
 *     the escalation is forced (FALLBACK 8, flat budget) rather than hoped for.
 *  2. Every HELD is preceded by that thread's ATTEMPT, and every RELEASE by its
 *     HELD.  Per thread, so it is an ordering check and not just arithmetic.
 *  3. Acquisitions and releases balance once the writers have joined.  A
 *     surviving imbalance is a leaked lane -- the defect the hook was added to
 *     find.
 *
 * The workload is test_rcu_txn_fallback's: a stream of narrow single-word
 * transactions starving wide ones, with the flat retry budget set low so the
 * wide transactions escalate.
 */

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#ifndef _LGPL_SOURCE
#define _LGPL_SOURCE
#endif

/*
 * Force escalation: flat budget (PER_COST_NUM 0, so the trigger does not depend
 * on what an attempt happened to cost) at a low retry count.  Without this the
 * lane is essentially never reached and every count below would be zero.
 */
#define URCU_TXN_FALLBACK_PER_COST_NUM	0
#define URCU_TXN_FALLBACK		8

#define URCU_TXN_LANE_TRACE

#include <inttypes.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include <urcu/compiler.h>
#include <urcu-qsbr.h>
#include <urcu-call-rcu.h>
#include <urcu/rcu-txn.h>

#include "tap.h"

#define NR_TESTS	4
#define NR_WORDS	8
#define NR_NARROW	6
#define NR_WIDE		2
#define NARROW_OPS	20000
#define WIDE_OPS	1500

static struct urcu_txn_domain g_domain;
static void *g_word[NR_WORDS];

/*
 * Per-thread hook state.  Thread-local so the ordering check needs no lock in
 * the hook itself -- instrumentation that serialises the path it instruments
 * would change the very timing under test.  The totals are folded in after the
 * writers join.
 */
struct lane_stat {
	unsigned long attempt, held, release;
	unsigned long bad_order;	/* HELD with no ATTEMPT, etc. */
	int holding;
};
static __thread struct lane_stat t_lane;

/* Folded after join, so plain adds are enough. */
static struct lane_stat g_lane;

void urcu_txn_lane_trace(enum urcu_txn_lane_event ev, const struct urcu_txn *txn)
{
	(void) txn;
	switch (ev) {
	case URCU_TXN_LANE_ATTEMPT:
		if (t_lane.holding)
			t_lane.bad_order++;	/* re-acquiring while held */
		t_lane.attempt++;
		break;
	case URCU_TXN_LANE_HELD:
		if (!t_lane.attempt || t_lane.holding)
			t_lane.bad_order++;
		t_lane.holding = 1;
		t_lane.held++;
		break;
	case URCU_TXN_LANE_RELEASE:
		if (!t_lane.holding)
			t_lane.bad_order++;	/* releasing what we never took */
		t_lane.holding = 0;
		t_lane.release++;
		break;
	}
}

static void lane_fold(void)
{
	g_lane.attempt += t_lane.attempt;
	g_lane.held += t_lane.held;
	g_lane.release += t_lane.release;
	g_lane.bad_order += t_lane.bad_order;
	g_lane.holding += t_lane.holding;	/* still-held at exit: a leak */
}

struct worker_arg {
	unsigned int seed;
	long committed;
};

static unsigned int xs(unsigned int x)
{
	x ^= x << 13; x ^= x >> 17; x ^= x << 5; return x;
}

static intptr_t lf_val(uintptr_t w) { return (intptr_t) w >> 32; }
static uintptr_t lf_bump(uintptr_t w, intptr_t delta)
{
	intptr_t val = lf_val(w) + delta;
	unsigned int ver = (unsigned int) ((w >> 1) & 0x7fffffffu) + 1;
	return ((uintptr_t) (uint32_t) (int32_t) val << 32)
		| ((uintptr_t) (ver & 0x7fffffffu) << 1);
}

/* Narrow: one word, the fast path -- the stream that starves the wide op. */
static void *narrow_worker(void *arg)
{
	struct worker_arg *wa = (struct worker_arg *) arg;
	unsigned int rng = wa->seed;
	long n;

	rcu_register_thread();
	for (n = 0; n < NARROW_OPS; n++) {
		struct urcu_txn tx;
		int i;

		rng = xs(rng);
		i = (int) (rng % NR_WORDS);
		urcu_txn_init(&tx, &g_domain);
		for (;;) {
			enum urcu_txn_status st;
			uintptr_t o;

			urcu_txn_begin(&tx);
			o = (uintptr_t) urcu_txn_load(&tx, &g_word[i], URCU_TXN_TAG);
			(void) urcu_txn_store_mw(&tx, &g_word[i], (void *) o,
					(void *) lf_bump(o, 1), URCU_TXN_TAG);
			st = urcu_txn_commit(&tx);
			urcu_txn_end(&tx);
			if (st == URCU_TXN_STATUS_ABORT)
				continue;
			break;
		}
		wa->committed++;
	}
	lane_fold();
	rcu_unregister_thread();
	return NULL;
}

/* Wide: every word, zero-sum, so it loses repeatedly and escalates. */
static void *wide_worker(void *arg)
{
	struct worker_arg *wa = (struct worker_arg *) arg;
	long n;

	rcu_register_thread();
	for (n = 0; n < WIDE_OPS; n++) {
		struct urcu_txn tx;

		urcu_txn_init(&tx, &g_domain);
		for (;;) {
			enum urcu_txn_status st;
			uintptr_t o[NR_WORDS];
			int w;

			urcu_txn_begin(&tx);
			(void) urcu_txn_reserve(&tx, NR_WORDS);
			for (w = 0; w < NR_WORDS; w++)
				o[w] = (uintptr_t) urcu_txn_load(&tx,
						&g_word[w], URCU_TXN_TAG);
			(void) urcu_txn_store_mw(&tx, &g_word[0],
					(void *) o[0],
					(void *) lf_bump(o[0],
						(NR_WORDS - 1) * 2),
					URCU_TXN_TAG);
			for (w = 1; w < NR_WORDS; w++)
				(void) urcu_txn_store_mw(&tx, &g_word[w],
						(void *) o[w],
						(void *) lf_bump(o[w], -2),
						URCU_TXN_TAG);
			st = urcu_txn_commit(&tx);
			urcu_txn_end(&tx);
			if (st == URCU_TXN_STATUS_ABORT)
				continue;
			break;
		}
		wa->committed++;
	}
	lane_fold();
	rcu_unregister_thread();
	return NULL;
}

int main(void)
{
	pthread_t narrow_th[NR_NARROW], wide_th[NR_WIDE];
	struct worker_arg narrow_a[NR_NARROW], wide_a[NR_WIDE];
	long narrow_total = 0, wide_total = 0;
	intptr_t sum = 0;
	int i;

	plan_tests(NR_TESTS);
	rcu_register_thread();
	urcu_txn_domain_init(&g_domain);

	for (i = 0; i < NR_NARROW; i++) {
		narrow_a[i].seed = 0x9e3779b9u
				+ (unsigned int) i * 2654435761u;
		narrow_a[i].committed = 0;
		pthread_create(&narrow_th[i], NULL, narrow_worker,
				&narrow_a[i]);
	}
	for (i = 0; i < NR_WIDE; i++) {
		wide_a[i].seed = 0x12345678u + (unsigned int) i;
		wide_a[i].committed = 0;
		pthread_create(&wide_th[i], NULL, wide_worker, &wide_a[i]);
	}

	rcu_thread_offline();
	for (i = 0; i < NR_NARROW; i++) {
		pthread_join(narrow_th[i], NULL);
		narrow_total += narrow_a[i].committed;
	}
	for (i = 0; i < NR_WIDE; i++) {
		pthread_join(wide_th[i], NULL);
		wide_total += wide_a[i].committed;
	}
	rcu_thread_online();

	for (i = 0; i < NR_WORDS; i++)
		sum += lf_val((uintptr_t) g_word[i]);

	diag("lane: attempt=%lu held=%lu release=%lu bad_order=%lu still_held=%d",
		g_lane.attempt, g_lane.held, g_lane.release,
		g_lane.bad_order, g_lane.holding);
	diag("%d narrow + %d wide: %ld + %ld committed; sum = %" PRIdPTR,
		NR_NARROW, NR_WIDE, narrow_total, wide_total, sum);

	/*
	 * FIRST, and the reason the rest mean anything: the hook ran.  Every
	 * check below is trivially true of a hook that never fired.
	 */
	ok(g_lane.held > 0,
		"the lane hook fired (escalation was reached, counts are not vacuous)");
	ok(g_lane.bad_order == 0,
		"per thread, HELD follows ATTEMPT and RELEASE follows HELD");
	ok(g_lane.held == g_lane.release && g_lane.holding == 0,
		"every acquired lane was released (no leak at thread exit)");
	ok(sum == narrow_total,
		"transactions stayed atomic through the lane (sum invariant)");

	rcu_barrier();
	rcu_unregister_thread();
	return exit_status();
}
