// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

/*
 * VALIDATE THE RESERVATION-OVERFLOW DETECTOR (-DURCU_TXN_DEBUG_RESERVE).
 *
 * urcu_txn_reserve() is how a caller promises that its commit cannot fail for
 * want of memory: it sizes the descriptor BEFORE taking a side-effect it cannot
 * undo, so every store afterwards appends without allocating.  Nothing checks
 * the promise.  An under-reservation just grows the descriptor, so a wrong bound
 * is invisible in every green run and only bites under memory pressure -- and
 * then as a sticky -ENOMEM raised at the COMMIT, arbitrarily far from the store
 * that overflowed.  -DURCU_TXN_DEBUG_RESERVE makes the overflowing store itself
 * the failure.
 *
 * ★ THIS TEST TESTS THE DETECTOR, NOT THE ENGINE.  A detector is evidence only
 * once it has been shown RED, and a green suite is no proof at all here: the
 * detector's whole point is that it fires where nothing else does.  So the
 * violation is CONSTRUCTED -- reserve N, record N+1 -- which makes one
 * iteration proof and needs no shape, no contention and no rate.
 *
 * It is driven in a CHILD PROCESS because the armed detector aborts, which is
 * the reaction the gate would run with; testing only the counting (_SOFT) arm
 * would leave the arm that actually stops a build unproven.
 *
 * The two negative controls are the other half.  Without them a detector that
 * fired on EVERY grow would look identical here, and would condemn the many
 * transactions that legitimately grow: one reserves exactly what it records,
 * the other reserves nothing at all and records far past the default capacity.
 */

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#ifndef _LGPL_SOURCE
#define _LGPL_SOURCE
#endif

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <urcu/compiler.h>
#include <urcu-qsbr.h>
#include <urcu-call-rcu.h>
#include <urcu/rcu-txn.h>

#include "tap.h"

#define TAG		0x8UL		/* proxy tag for the transacted slots */
#define NR_SLOTS	64
static void *g_slots[NR_SLOTS];
static void *const V_OLD = (void *) 0x100UL;
static void *const V_NEW = (void *) 0x200UL;

/*
 * Record @nr stores into a handle that reserved @reserve (0 = never reserved).
 * Returns the handle's overflow count; under the armed, non-SOFT detector an
 * overflow aborts instead of returning.
 */
static unsigned long run_txn(unsigned int reserve, unsigned int nr)
{
	struct urcu_txn txn;
	unsigned long overflows;
	unsigned int k;

	for (k = 0; k < NR_SLOTS; k++)
		uatomic_store(&g_slots[k], V_OLD, CMM_RELEASE);
	urcu_txn_init(&txn, NULL);		/* no domain: no escalation */
	urcu_txn_begin(&txn);
	if (reserve && urcu_txn_reserve(&txn, reserve) < 0)
		abort();			/* not the class under test */
	for (k = 0; k < nr; k++)
		urcu_txn_store_mw(&txn, &g_slots[k], V_OLD, V_NEW, TAG);
	overflows = urcu_txn_reserve_overflows(&txn);
	urcu_txn_end(&txn);			/* discard: the records are the point */
	return overflows;
}

/*
 * Run @nr stores against a @reserve-record reservation in a child, and report
 * whether the child died on the detector's abort.
 */
static int child_aborts(unsigned int reserve, unsigned int nr)
{
	pid_t pid = fork();
	int status;

	if (pid < 0)
		abort();
	if (pid == 0) {
		/*
		 * The parent's tap output is already buffered in this copy of
		 * the process; an abort would flush it a second time and the
		 * harness would see the plan twice.
		 */
		fflush(NULL);
		(void) run_txn(reserve, nr);
		_exit(0);			/* detector stayed silent */
	}
	if (waitpid(pid, &status, 0) != pid)
		abort();
	return WIFSIGNALED(status) && WTERMSIG(status) == SIGABRT;
}

int main(void)
{
	plan_tests(3);

#ifndef URCU_TXN_DEBUG_RESERVE
	skip(3, "built without -DURCU_TXN_DEBUG_RESERVE: the detector is not compiled in");
	return exit_status();
#else
	rcu_register_thread();

	/*
	 * POSITIVE: reserve 8, record 9.  The ninth store is past the promise,
	 * and it is the store -- not the commit that would have inherited its
	 * -ENOMEM -- that has to fail.
	 */
	{
		int red = child_aborts(8, 9);

		ok(red, "the detector fires on a store past the reservation");
		if (!red)
			diag("DETECTOR IS BLIND: a 9th store into an 8-record "
				"reservation grew the descriptor and said nothing");
	}

	/*
	 * NEGATIVE 1: reserve 8, record exactly 8.  An off-by-one here would
	 * turn every correctly-sized commit in the tree red.
	 */
	{
		unsigned long n = run_txn(8, 8);

		ok(n == 0, "a reservation filled exactly is not an overflow");
		if (n)
			diag("FALSE POSITIVE: %lu overflow(s) reported for 8 "
				"stores into an 8-record reservation", n);
	}

	/*
	 * NEGATIVE 2: never reserve, record 40 -- eight doublings past
	 * URCU_TXN_INIT.  Growth is the documented behaviour of an unreserved
	 * handle, so the detector must be scoped to handles that PROMISED.
	 */
	{
		unsigned long n = run_txn(0, 40);

		ok(n == 0, "an unreserved handle grows without being reported");
		if (n)
			diag("FALSE POSITIVE: %lu overflow(s) reported for a "
				"handle that never called urcu_txn_reserve", n);
	}

	rcu_unregister_thread();
	return exit_status();
#endif
}
