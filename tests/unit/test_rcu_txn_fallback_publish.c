// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

/*
 * Who publishes domain->active, and therefore when a fallback episode ENDS.
 *
 * The escalation lane is entered by two kinds of handle.  An INITIATOR met a
 * trigger itself (it retried past its escalation budget); a JOINER escalated
 * only because it sampled domain->active.  Only an initiator advertises, so
 * the episode outlives neither its initiator nor a joiner it promoted.
 *
 * If every holder re-asserted the flag, the episode would sustain itself: the
 * flag is up whenever anyone holds the lane, each arrival that samples it
 * queues, and queuing guarantees a next holder to raise it again.  Leaving the
 * regime would then need the lane to drain with no arrival sampling it -- i.e.
 * write-side quiescence.  Test 10 is the regression that pins this down: a real
 * joiner, having taken the lane behind the initiator, must observe active == 0
 * once the initiator has gone.
 *
 * Tests 1-8 are deterministic and single-threaded, driving the internal helpers
 * directly (white-box) so no timing is involved.  Tests 9-11 run the real
 * two-thread hand-off.  QSBR flavor.
 */

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#ifndef _LGPL_SOURCE
#define _LGPL_SOURCE
#endif

/*
 * Low threshold so the lane fires on purpose, not by accident.  PER_COST_NUM 0
 * selects the FLAT budget: this test is about WHO PUBLISHES, so it wants a
 * trigger that does not depend on what an attempt happened to cost.
 */
#define URCU_TXN_FALLBACK_PER_COST_NUM	0
#define URCU_TXN_FALLBACK		8

#include <poll.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>

#include <urcu/compiler.h>
#include <urcu-qsbr.h>
#include <urcu-call-rcu.h>
#include <urcu/rcu-txn.h>

#include "tap.h"

#define NR_TESTS	11

/* Opaque, bit-0-clear slot values (the engine owns bit 0). */
#define V0	((void *) 0x10)
#define V1	((void *) 0x20)

static struct urcu_txn_domain g_dom;
static void *g_wa, *g_wb;

/* Thread B's progress: 0 = not started, 1 = about to begin(), 2 = in the lane. */
static int b_state;
static int b_gate;		/* main lets B finish */
/* What B observed the instant it got the lane. */
static int b_published, b_active_seen;

static void spin_until(int *p, int v)
{
	while (uatomic_read(p) != v)
		(void) poll(NULL, 0, 1);
}

/* ------------------------------------------------------------------ */
/* 1-4: an initiator publishes, and its exit ends the episode.        */

static void test_initiator_commit_ends_episode(void)
{
	struct urcu_mcas_txn tx;

	g_wa = V0;
	urcu_txn_init(&tx, &g_dom);
	tx.retry = URCU_TXN_FALLBACK;		/* white-box: a starved handle */
	urcu_txn_begin(&tx);
	ok(tx.in_fallback && tx.fb_published &&
			uatomic_read(&g_dom.active) == 1,
		"a starved handle escalates, and the initiator publishes the episode");

	urcu_txn_store(&tx, &g_wa, V0, V1, URCU_MCAS_TAG);
	if (urcu_txn_commit(&tx) != URCU_TXN_STATUS_OK)
		abort();
	urcu_txn_end(&tx);
	ok(!tx.in_fallback && !tx.fb_published &&
			uatomic_read(&g_dom.active) == 0 && g_wa == V1,
		"the initiator's exit clears domain->active: the episode ends");
}

static void test_initiator_by_retry(void)
{
	struct urcu_mcas_txn tx;

	urcu_txn_init(&tx, &g_dom);
	tx.retry = URCU_TXN_FALLBACK;		/* white-box: a starved handle */
	urcu_txn_begin(&tx);
	ok(tx.in_fallback && tx.fb_published &&
			uatomic_read(&g_dom.active) == 1,
		"retry >= FALLBACK escalates, and that initiator publishes too");
	urcu_txn_end(&tx);
	ok(uatomic_read(&g_dom.active) == 0,
		"a starved initiator's exit also ends the episode");
}

/* ------------------------------------------------------------------ */
/* 5-8: a joiner publishes nothing, and never clears a peer's flag --  */
/*      unless it starves inside the lane and is promoted.             */

static void test_joiner_does_not_publish(void)
{
	struct urcu_mcas_txn tx;

	/* Simulate an episode owned by some other handle. */
	uatomic_set(&g_dom.active, 1);

	urcu_txn_init(&tx, &g_dom);
	urcu_txn_begin(&tx);			/* joins: active is up */
	ok(tx.in_fallback && !tx.fb_published,
		"a joiner takes the lane but advertises nothing");

	urcu_txn_end(&tx);
	ok(uatomic_read(&g_dom.active) == 1,
		"a joiner's exit does NOT clear the flag it never raised");

	uatomic_set(&g_dom.active, 0);		/* the real owner would do this */
}

static void test_joiner_promoted_on_starvation(void)
{
	struct urcu_mcas_txn tx;

	uatomic_set(&g_dom.active, 1);
	urcu_txn_init(&tx, &g_dom);
	urcu_txn_begin(&tx);			/* joins, does not publish */
	if (tx.fb_published)
		abort();

	/* It starves while holding its turn: nothing protects it any more. */
	tx.retry = URCU_TXN_FALLBACK;
	urcu_txn__maybe_publish(&tx);
	ok(tx.fb_published && uatomic_read(&g_dom.active) == 1,
		"a joiner that starves inside the lane is promoted to initiator");

	urcu_txn_end(&tx);
	ok(uatomic_read(&g_dom.active) == 0,
		"the promoted handle now owns the episode, and ends it on exit");
}

/* ------------------------------------------------------------------ */
/* 9-11: the regression -- a real joiner must not sustain the episode. */

static void *thread_b(void *arg __attribute__((unused)))
{
	struct urcu_mcas_txn tx;

	rcu_register_thread();
	urcu_txn_init(&tx, &g_dom);

	uatomic_set(&b_state, 1);
	/*
	 * A holds the lane and active is up, so this blocks inside
	 * cds_fair_mutex_lock() until A departs.  b_state stays 1 meanwhile --
	 * that is how main knows B really joined rather than sailing past.
	 */
	urcu_txn_begin(&tx);

	b_published = tx.fb_published;
	b_active_seen = (int) uatomic_read(&g_dom.active);
	uatomic_set(&b_state, 2);

	spin_until(&b_gate, 1);
	urcu_txn_store(&tx, &g_wb, V0, V1, URCU_MCAS_TAG);
	if (urcu_txn_commit(&tx) != URCU_TXN_STATUS_OK)
		abort();
	urcu_txn_end(&tx);

	rcu_thread_offline();
	rcu_barrier();
	rcu_thread_online();
	rcu_unregister_thread();
	return NULL;
}

static void test_joiner_does_not_sustain_episode(void)
{
	struct urcu_mcas_txn ta, tc;
	pthread_t b;

	g_wa = V0;
	g_wb = V0;
	uatomic_set(&b_state, 0);
	uatomic_set(&b_gate, 0);

	/* A becomes the initiator and holds the lane. */
	urcu_txn_init(&ta, &g_dom);
	ta.retry = URCU_TXN_FALLBACK;		/* white-box: a starved handle */
	urcu_txn_begin(&ta);
	if (!ta.fb_published)
		abort();

	if (pthread_create(&b, NULL, thread_b, NULL))
		abort();
	spin_until(&b_state, 1);

	/* Give B time to reach the lock and park behind A. */
	(void) poll(NULL, 0, 200);
	ok(uatomic_read(&b_state) == 1,
		"the joiner is parked in the lane behind the initiator");

	/* A departs: it published, so it ends the episode. */
	urcu_txn_store(&ta, &g_wa, V0, V1, URCU_MCAS_TAG);
	if (urcu_txn_commit(&ta) != URCU_TXN_STATUS_OK)
		abort();
	urcu_txn_end(&ta);

	spin_until(&b_state, 2);
	ok(!b_published && b_active_seen == 0,
		"the joiner inherits the lane with the episode already OVER "
		"(active==0, published==0)");

	/* And a fresh handle must now take the optimistic path. */
	urcu_txn_init(&tc, &g_dom);
	ok(!urcu_txn__want_fallback(&tc),
		"a new transaction no longer funnels: the domain reverted");

	uatomic_set(&b_gate, 1);
	if (pthread_join(b, NULL))
		abort();
}

int main(void)
{
	plan_tests(NR_TESTS);
	rcu_register_thread();
	urcu_txn_domain_init(&g_dom);

	test_initiator_commit_ends_episode();	/* 1, 2 */
	test_initiator_by_retry();		/* 3, 4 */
	test_joiner_does_not_publish();		/* 5, 6 */
	test_joiner_promoted_on_starvation();	/* 7, 8 */
	test_joiner_does_not_sustain_episode();	/* 9, 10, 11 */

	rcu_thread_offline();
	rcu_barrier();
	rcu_thread_online();
	rcu_unregister_thread();
	return exit_status();
}
