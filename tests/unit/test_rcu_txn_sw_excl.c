// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

/*
 * Test URCU_TXN_SW_EXCL_VALIDATE, the single-writer validator of
 * <urcu/rcu-txn-sw.h>.
 *
 * The sw engine has no install-time CAS, no conflict detection and no abort: it
 * REQUIRES writer mutual exclusion, and two writers racing on one slot simply
 * corrupt it.  Nothing in a default build says so.  The validator makes such a
 * violation abort the process with a report naming the slot and the threads; it
 * is opt-in and this test is compiled with it (see Makefile.am).
 *
 * There is no per-structure object to claim an owner on -- the sw mutators take
 * a node, never a head -- so the validator watches the two things that do
 * exist: the HANDLE (driven end to end by the thread that init'd it) and the
 * SLOT, which is what two racing writers actually share.  In a correct
 * single-writer program a slot being recorded or parked cannot already hold an
 * installed proxy, and at settle a slot must still hold the proxy WE installed.
 *
 * Each violation is provoked in a forked child and the parent asserts the child
 * died by SIGABRT; the child drops its core limit so a passing run leaves no
 * cores.  The writer/writer races are simulated DETERMINISTICALLY: the other
 * writer's proxy is parked in the slot by hand, with the flip-group primitives,
 * as a real race would leave it -- so the test is reproducible rather than
 * timing-dependent.
 * A control case checks a well-formed single-writer transaction is untouched.
 *
 * QSBR flavor (the commit defers reclaim through call_rcu).
 */

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#ifndef _LGPL_SOURCE
#define _LGPL_SOURCE
#endif

#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/resource.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <urcu/compiler.h>
#include <urcu-qsbr.h>
#include <urcu-call-rcu.h>
#include <urcu/rcu-txn-sw.h>

#include "tap.h"

#define NR_TESTS	7

#define TAG	1UL		/* bit 0: no live value below carries it */

/* Opaque, tag-clear slot values. */
#define V0	((void *) 0x100)
#define V1	((void *) 0x200)
#define V2	((void *) 0x300)

static void *g_a, *g_b;

/* Run @body in a child; 1 iff it died by SIGABRT (the validator fired). */
static int aborts_in_child(void (*body)(void))
{
	struct rlimit nocore = { .rlim_cur = 0, .rlim_max = 0 };
	pid_t pid;
	int status;

	pid = fork();
	if (pid < 0) {
		diag("fork: failed");
		return 0;
	}
	if (pid == 0) {
		(void) setrlimit(RLIMIT_CORE, &nocore);
		body();
		_exit(0);			/* the validator did NOT fire */
	}
	if (waitpid(pid, &status, 0) < 0) {
		diag("waitpid: failed");
		return 0;
	}
	return WIFSIGNALED(status) && WTERMSIG(status) == SIGABRT;
}

/* 1. The handle is init'd here but recorded into from another thread. */
static struct urcu_txn_sw_txn g_handoff;

static void *handoff_thread(void *arg)
{
	(void) arg;
	rcu_register_thread();
	/* Different thread than the one that init'd it: a handle is driven end
	 * to end by one thread. */
	(void) urcu_txn_sw_record(&g_handoff, &g_a, V0, V1, TAG);
	rcu_unregister_thread();
	return NULL;
}

static void body_cross_thread(void)
{
	pthread_t th;

	g_a = V0;
	urcu_txn_sw_init(&g_handoff);		/* owner := this thread */
	if (pthread_create(&th, NULL, handoff_thread, NULL))
		return;				/* cannot provoke: child exits 0 */
	(void) pthread_join(th, NULL);
}

/*
 * Park @slot with a proxy by hand, as a writer stopped between its install and
 * its flip leaves it.  commit() runs install, flip and settle back to back, so
 * one thread cannot stop a transaction there.
 */
static struct urcu_txn_sw_group g_parked_group;
static struct urcu_txn_sw_record g_parked[2];

static void park_by_hand(unsigned int i, void **slot, void *old_ptr,
		void *new_ptr)
{
	urcu_txn_sw_group_init(&g_parked_group);
	urcu_txn_sw_record_set(&g_parked[i], slot, old_ptr, new_ptr, TAG);
	urcu_txn_sw_record_install(&g_parked_group, &g_parked[i]);
}

/*
 * 2. A second writer RECORDS a slot the first has already parked.  This is the
 *    interleaving where the violation is visible: writer 1 is between install
 *    and flip, so its proxy is sitting in the slot.
 */
static void body_record_over_parked(void)
{
	struct urcu_txn_sw_txn t;

	g_a = V0;
	park_by_hand(0, &g_a, V0, V1);		/* g_a holds writer 1's proxy */

	urcu_txn_sw_init(&t);			/* "the other writer" */
	(void) urcu_txn_sw_record(&t, &g_a, V1, V2, TAG);	/* g_a is parked! */
	(void) urcu_txn_sw_commit(&t);
}

/*
 * 3. A second writer INSTALLS onto a slot the first has already parked.  It
 *    recorded the slot while it was still plain, so record() saw nothing wrong;
 *    the overlap only becomes visible at its own install.
 */
static void body_park_over_parked(void)
{
	struct urcu_txn_sw_txn t;

	g_a = V0;
	g_b = V0;
	urcu_txn_sw_init(&t);			/* records first, installs last */
	(void) urcu_txn_sw_record(&t, &g_a, V0, V2, TAG);
	(void) urcu_txn_sw_record(&t, &g_b, V0, V2, TAG);

	park_by_hand(0, &g_a, V0, V1);		/* writer 1 parks both slots */
	park_by_hand(1, &g_b, V0, V1);

	(void) urcu_txn_sw_commit(&t);		/* would clobber its proxies */
}

/*
 * 4. Our proxy is gone by the time we settle: someone overwrote it.  Nothing
 *    can be staged between commit()'s own install and settle from one thread,
 *    so this runs the settle-time check itself on a proxy parked by hand.
 */
static void body_settle_clobbered(void)
{
	g_a = V0;
	park_by_hand(0, &g_a, V0, V1);		/* stands for our install */
	uatomic_store(&g_a, V2, CMM_RELEASE);	/* a peer overwrites our proxy */
	urcu_txn_sw__excl_slot_ours(&g_parked[0]);	/* settle finds it gone */
}

/*
 * 5. The single-edge commit stores BLIND (no proxy, no install), so its only
 *    witness is the slot's value: it must still hold the recorded old.
 */
static void body_single_edge_changed(void)
{
	struct urcu_txn_sw_txn t;

	g_a = V0;
	urcu_txn_sw_init(&t);
	(void) urcu_txn_sw_record(&t, &g_a, V0, V1, TAG);
	uatomic_store(&g_a, V2, CMM_RELEASE);	/* a peer changed it under us */
	(void) urcu_txn_sw_commit(&t);
}

/*
 * 6. The MULTI-edge install has the same witness.  A peer that ran a COMPLETE
 *    transaction on the slot -- install, flip, settle -- between our record()
 *    and our commit() leaves a PLAIN value behind, so a presence-only "is
 *    there a proxy here?" check sees a free slot and waves us through.  We
 *    would then install a proxy whose old is stale (readers in the install
 *    window watch the committed value go backwards) and settle our new over the
 *    peer's, silently.  The value check catches it; the peer's committed write
 *    is simulated here by a plain store, which is what it leaves behind.
 */
static void body_park_over_settled(void)
{
	struct urcu_txn_sw_txn t;

	g_a = V0;
	g_b = V0;
	urcu_txn_sw_init(&t);
	(void) urcu_txn_sw_record(&t, &g_a, V0, V1, TAG);
	(void) urcu_txn_sw_record(&t, &g_b, V0, V1, TAG);
	uatomic_store(&g_b, V2, CMM_RELEASE);	/* a peer committed V0 -> V2 */
	(void) urcu_txn_sw_commit(&t);		/* our install would lose it */
}

/* Control: a well-formed single-writer transaction trips nothing. */
static void body_control(void)
{
	struct urcu_txn_sw_txn t;
	unsigned int i;

	g_a = V0;
	g_b = V0;
	for (i = 0; i < 100; i++) {		/* sequential txns on the same slots */
		urcu_txn_sw_init(&t);
		(void) urcu_txn_sw_record(&t, &g_a, g_a, V1, TAG);
		(void) urcu_txn_sw_record(&t, &g_b, g_b, V1, TAG);
		if (urcu_txn_sw_commit(&t) != URCU_TXN_STATUS_OK)
			abort();	/* fail the control: an honest txn commits */
		urcu_txn_sw_init(&t);
		(void) urcu_txn_sw_record(&t, &g_a, g_a, V0, TAG);
		(void) urcu_txn_sw_record(&t, &g_b, g_b, V0, TAG);
		if (urcu_txn_sw_commit(&t) != URCU_TXN_STATUS_OK)
			abort();
	}
	if (g_a != V0 || g_b != V0)
		abort();		/* fail the control: the commits did not land */
}

int main(void)
{
	plan_tests(NR_TESTS);

#ifndef URCU_TXN_SW_EXCL_VALIDATE
	skip(NR_TESTS, "URCU_TXN_SW_EXCL_VALIDATE is off: the validator is compiled out");
#else
	rcu_register_thread();

	ok(aborts_in_child(body_cross_thread),
		"a handle driven from a thread other than the one that init'd it aborts");
	ok(aborts_in_child(body_record_over_parked),
		"recording a slot another writer has already parked aborts");
	ok(aborts_in_child(body_park_over_parked),
		"installing onto a slot another writer has already parked aborts");
	ok(aborts_in_child(body_park_over_settled),
		"installing onto a slot a peer already COMMITTED to aborts (the install "
		"witness is the value, not just proxy-presence)");
	ok(aborts_in_child(body_settle_clobbered),
		"settling a slot whose installed proxy was overwritten aborts");
	ok(aborts_in_child(body_single_edge_changed),
		"a single-edge commit whose slot no longer holds the recorded old aborts");
	ok(!aborts_in_child(body_control),
		"control: a well-formed single-writer transaction trips nothing");

	rcu_barrier();
	rcu_unregister_thread();
#endif
	return exit_status();
}
