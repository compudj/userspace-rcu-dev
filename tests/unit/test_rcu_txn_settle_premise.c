// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

/*
 * VALIDATE THE SETTLE-PREMISE DETECTOR (-DURCU_TXN_DEBUG_SETTLE).
 *
 * urcu_txn_settle() converts each parked slot with a PLAIN release store,
 * sound only while nothing else writes that word between the park and the
 * settle -- "an SW slot is caller-exclusive, an MW slot still holds OUR
 * proxy".  Both halves are EMBEDDER obligations the engine never checks, so
 * -DURCU_TXN_DEBUG_SETTLE adds the check and counts the violations.
 *
 * ★ THIS TEST TESTS THE DETECTOR, NOT THE ENGINE.  A detector is only evidence
 * once it has been shown RED, and the obvious way to show this one red -- run
 * the big contended FT arm with a historical defect re-injected -- DOES NOT
 * WORK on this tree: 126.9M settle records with both @8efc46cd and @04469a17
 * reverted produced FOREIGN=0 where the historical rate predicted ~4.7, and a
 * 2000x settle-window amplifier did not change it.  Later fixes prevent the
 * peer interaction those violations needed, so the DEFECT is reachable while
 * its consequence is not.  A validation that depends on an FT shape being
 * reachable is a validation that silently expires.
 *
 * So drive the premise directly: park a word, have a second thread write it
 * before the settle, and require the counter to move.  No FT, no shape, no
 * rate -- the violation is CONSTRUCTED, so one iteration is already proof and
 * the loop only buys margin against scheduling.
 *
 * The peer's write is deliberately impolite (a plain store over a parked
 * proxy).  That is the point: the detector's job is to NOTICE a foreign write,
 * and a peer that played by the rules would not produce one to notice.
 */

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#ifndef _LGPL_SOURCE
#define _LGPL_SOURCE
#endif

#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include <urcu/compiler.h>
#include <urcu-qsbr.h>
#include <urcu-call-rcu.h>
#include <urcu/rcu-txn.h>

#include "tap.h"

#define TAG		0x8UL		/* proxy tag for the transacted slot */
/*
 * A FLAKE BUDGET, not a throughput knob.  The assertion is "at least one", so
 * what matters is the chance of seeing NONE: with the write set shaped as below
 * the peer lands on 46-86% of iterations, which makes that chance nil long
 * before 2000 -- the count is kept here only so a future change that quietly
 * narrows the window shows up as a falling hit count in the diag rather than as
 * an occasional red.
 */
#define NR_ITER		2000

/*
 * The write set.  Two things about its SHAPE are load-bearing:
 *
 *  - MORE THAN ONE RECORD.  A single-record commit takes the lone-store fast
 *    path -- the store IS the commit, no proxy, no settle -- so nothing is ever
 *    parked and there is nothing to violate.  (Measured: with one record the
 *    census reported checked=0 and the test read as "detector blind".)
 *  - THE PEER TARGETS THE LAST SLOT.  The settle walks the records in order, so
 *    a peer aimed at the last one has the whole walk to land in, instead of
 *    racing the very first check.  That turns a coincidence into a structural
 *    window: measured 2-6 hits per 2000 iterations with the peer on record 0,
 *    and 928-1722 per 2000 with it on record NR_SLOTS-1.
 */
#define NR_SLOTS	16
static void *g_slots[NR_SLOTS];
#define g_slot		(g_slots[NR_SLOTS - 1])	/* the one the peer overwrites */
static void *const V_OLD  = (void *) 0x100UL;
static void *const V_NEW  = (void *) 0x200UL;
static void *const V_PEER = (void *) 0x300UL;

/* Handshake: the writer publishes an iteration, the peer acknowledges it. */
static unsigned long g_iter, g_ack;
static int g_stop;

/*
 * THE FOREIGN WRITER.  Spin until the slot no longer reads V_OLD -- that is
 * the instant the commit has PARKED it (the word then holds the descriptor's
 * proxy, never a value we wrote) -- and store over it.  Bounded, so a missed
 * park ends the iteration instead of hanging the suite.
 */
static void *peer_thread(void *unused __attribute__((unused)))
{
	unsigned long seen = 0;

	rcu_register_thread();
	for (;;) {
		unsigned long spins = 0;

		while (uatomic_load(&g_iter, CMM_ACQUIRE) == seen) {
			if (uatomic_load(&g_stop, CMM_RELAXED))
				goto out;
			caa_cpu_relax();
		}
		seen = uatomic_load(&g_iter, CMM_ACQUIRE);
		/* Wait for the park, then overwrite it. */
		while (uatomic_load(&g_slot, CMM_ACQUIRE) == V_OLD &&
				spins++ < 100000000UL)
			caa_cpu_relax();
		uatomic_store(&g_slot, V_PEER, CMM_RELEASE);
		uatomic_store(&g_ack, seen, CMM_RELEASE);
	}
out:
	rcu_unregister_thread();
	return NULL;
}

int main(void)
{
	pthread_t peer;
	unsigned long i;
	unsigned long before, after;
	int ret = 0;

	plan_tests(1);

#ifndef URCU_TXN_DEBUG_SETTLE
	skip(1, "built without -DURCU_TXN_DEBUG_SETTLE: the detector is not compiled in");
	return exit_status();
#else
	rcu_register_thread();
	if (pthread_create(&peer, NULL, peer_thread, NULL))
		abort();

	before = uatomic_load(&urcu_txn_dbg_settle_foreign, CMM_RELAXED);

	for (i = 1; i <= NR_ITER; i++) {
		struct urcu_txn txn;
		unsigned int k;

		urcu_txn_init(&txn, NULL);	/* no domain: no escalation needed */
		for (k = 0; k < NR_SLOTS; k++)
			uatomic_store(&g_slots[k], V_OLD, CMM_RELEASE);
		uatomic_store(&g_iter, i, CMM_RELEASE);	/* release the peer */

		urcu_txn_begin(&txn);
		for (k = 0; k < NR_SLOTS; k++)
			urcu_txn_store_mw(&txn, &g_slots[k], V_OLD, V_NEW, TAG);
		(void) urcu_txn_commit(&txn);	/* parks, decides, SETTLES */
		urcu_txn_end(&txn);

		while (uatomic_load(&g_ack, CMM_ACQUIRE) != i)
			caa_cpu_relax();
		rcu_quiescent_state();
	}

	after = uatomic_load(&urcu_txn_dbg_settle_foreign, CMM_RELAXED);
	uatomic_store(&g_stop, 1, CMM_RELEASE);
	uatomic_inc(&g_iter);			/* wake the peer to observe it */
	pthread_join(peer, NULL);

	diag("settle-premise: FOREIGN %lu -> %lu over %d constructed violations",
		before, after, NR_ITER);
	/*
	 * ONE is enough to prove the detector sees the class; requiring all
	 * NR_ITER would make this a timing test of the peer's spin instead.
	 */
	ret = (after > before);
	ok(ret, "the settle-premise detector fires on a foreign write to a parked slot");
	if (!ret)
		diag("DETECTOR IS BLIND: %d parked slots were overwritten before "
			"their settle and none was reported", NR_ITER);

	rcu_unregister_thread();
	return exit_status();
#endif
}
