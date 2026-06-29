// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

/*
 * Test for the load-validate guard of <urcu/rcu-txn.h>:
 * urcu_txn_load_validate() reads a slot AND folds a load-only {v -> v}
 * record into the commit, so the transaction commits only if that slot still
 * holds the read value at the install point -- a TM read-set entry.  The model
 * scenario is a tombstone an insert must find clear: validate the tombstone,
 * write the structural edges, and the commit is atomic with "tombstone still
 * clear".
 *
 * These are deterministic single-threaded checks (a "peer" change is simulated
 * with a plain store between buffering and commit, valid because no proxy is
 * parked until commit):
 *   1. guard holds            -> commit OK, write applied, guarded word intact;
 *   2. guard fails mid-flight -> commit ABORT, write NOT applied;
 *   3. validate-then-store same slot -> one record, commits as the write;
 *   4. store-then-validate same slot -> write preserved (validate never
 *      downgrades a pending write), one record;
 *   5. pure guard, unchanged  -> commit OK, slot untouched.
 *
 * QSBR flavor (the commit defers descriptor reclaim through call_rcu).
 */

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#ifndef _LGPL_SOURCE
#define _LGPL_SOURCE
#endif

#include <stdio.h>
#include <stdlib.h>

#include <urcu/compiler.h>
#include <urcu-qsbr.h>
#include <urcu-call-rcu.h>
#include <urcu/rcu-txn.h>

#include "tap.h"

#define NR_TESTS	5

/* Opaque, bit-0-clear slot values (the engine owns bit 0). */
#define CLEAR	((void *) 0x100)
#define SET	((void *) 0x200)
#define P0	((void *) 0x10)
#define P1	((void *) 0x20)
#define VX	((void *) 0x30)
#define VZ	((void *) 0x40)

static void *g_gate;		/* the "tombstone" */
static void *g_payload;		/* a word the op writes */
static void *g_w;		/* same-slot validate/store cases */

int main(void)
{
	struct urcu_mcas_txn tx;
	void *gv, *vv;
	unsigned int nr1, nr2;
	enum urcu_txn_status st;

	plan_tests(NR_TESTS);
	rcu_register_thread();

	/* 1. Guard holds: commit OK, payload written, gate left untouched. */
	g_gate = CLEAR;
	g_payload = P0;
	urcu_txn_init(&tx, NULL);
	urcu_txn_begin(&tx);
	gv = urcu_txn_load_validate(&tx, &g_gate);
	urcu_txn_store(&tx, &g_payload, P0, P1);
	st = urcu_txn_commit(&tx);
	urcu_txn_end(&tx);
	ok(gv == CLEAR && st == URCU_TXN_STATUS_OK &&
			g_gate == CLEAR && g_payload == P1,
		"guard holds -> commit applies the write, leaves the guarded word");

	/* 2. Guard fails: a peer sets the gate before our install -> ABORT,
	 * and the buffered write must NOT take effect. */
	g_gate = CLEAR;
	g_payload = P0;
	urcu_txn_init(&tx, NULL);
	urcu_txn_begin(&tx);
	gv = urcu_txn_load_validate(&tx, &g_gate);
	urcu_txn_store(&tx, &g_payload, P0, P1);
	g_gate = SET;			/* simulated racing tombstone */
	st = urcu_txn_commit(&tx);
	urcu_txn_end(&tx);
	ok(gv == CLEAR && st == URCU_TXN_STATUS_ABORT && g_payload == P0,
		"guard fails -> commit aborts and the write is not applied");

	/* 3. validate-then-store same slot: the store upgrades the guard in
	 * place (one record), and commits as the write. */
	g_w = VX;
	urcu_txn_init(&tx, NULL);
	urcu_txn_begin(&tx);
	vv = urcu_txn_load_validate(&tx, &g_w);
	urcu_txn_store(&tx, &g_w, VX, VZ);
	nr1 = tx.mcas->nr;
	st = urcu_txn_commit(&tx);
	urcu_txn_end(&tx);
	ok(vv == VX && nr1 == 1 && st == URCU_TXN_STATUS_OK && g_w == VZ,
		"validate-then-store on one slot -> one record, commits the write");

	/* 4. store-then-validate same slot: the validate must NOT downgrade the
	 * pending write; still one record; the write stands. */
	g_w = VX;
	urcu_txn_init(&tx, NULL);
	urcu_txn_begin(&tx);
	urcu_txn_store(&tx, &g_w, VX, VZ);
	vv = urcu_txn_load_validate(&tx, &g_w);
	nr2 = tx.mcas->nr;
	st = urcu_txn_commit(&tx);
	urcu_txn_end(&tx);
	ok(vv == VX && nr2 == 1 && st == URCU_TXN_STATUS_OK && g_w == VZ,
		"store-then-validate on one slot -> write preserved, one record");

	/* 5. Pure guard over an unchanged word: commit OK, no modification. */
	g_w = VX;
	urcu_txn_init(&tx, NULL);
	urcu_txn_begin(&tx);
	vv = urcu_txn_load_validate(&tx, &g_w);
	st = urcu_txn_commit(&tx);
	urcu_txn_end(&tx);
	ok(vv == VX && st == URCU_TXN_STATUS_OK && g_w == VX,
		"pure guard over an unchanged word commits without modifying it");

	rcu_barrier();			/* drain deferred descriptor frees */
	rcu_unregister_thread();
	return exit_status();
}
