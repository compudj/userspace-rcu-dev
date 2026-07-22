// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

/*
 * Test for the guards of <urcu/rcu-txn-mw.h>.
 *
 * urcu_txn_mw_load_validate() reads a slot AND folds a load-only {v -> v}
 * record into the commit, so the transaction commits only if that slot still
 * holds the read value at the install point -- a TM read-set entry.  The model
 * scenario is a tombstone an insert must find clear: validate the tombstone,
 * write the structural edges, and the commit is atomic with "tombstone still
 * clear".
 *
 * urcu_txn_mw_validate() is the same guard with a CALLER-SUPPLIED expected value:
 * it guards a word against an image the mutator COMPUTES rather than one it
 * read through the bracket, which is what the load-derived guard cannot express.
 *
 * These are deterministic single-threaded checks (a "peer" change is simulated
 * with a plain store between buffering and commit, valid because no proxy is
 * parked until commit):
 *   1. guard holds            -> commit OK, write applied, guarded word intact;
 *   2. guard fails mid-flight -> commit ABORT, write NOT applied;
 *   3. validate-then-store same slot -> one record, commits as the write;
 *   4. urcu_txn_mw_load_committed() past a pending store -> reads the slot's
 *      COMMITTED value, ignoring this attempt's pending write; records nothing,
 *      and the store still stands at commit;
 *   5. pure guard, unchanged  -> commit OK, slot untouched;
 *   6. store-then-validate same slot -> the validate reads this transaction's
 *      own PENDING value (read-your-own-writes, the default), its record chains
 *      rather than poisoning, one record, write stands;
 *   7. caller-expected guard over a COMPUTED image that holds -> commit OK,
 *      guarded word untouched;
 *   8. caller-expected guard, peer moved the word -> commit ABORT, co-recorded
 *      write NOT applied;
 *   9. the caller-expected guard is an EXACT word match, not a mask.
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
#include <urcu/rcu-txn-mw.h>

#include "tap.h"

#define NR_TESTS	9

/* Opaque, bit-0-clear slot values (the engine owns bit 0). */
#define CLEAR	((void *) 0x100)
#define SET	((void *) 0x200)
#define P0	((void *) 0x10)
#define P1	((void *) 0x20)
#define VX	((void *) 0x30)
#define VZ	((void *) 0x40)

/* A "control word": stable bits plus a marker subfield (never bit 0). */
#define CTRL_BASE	0x1000UL
#define CTRL_MARK	0x0040UL
#define CTRL(mark)	((void *) (CTRL_BASE | ((mark) ? CTRL_MARK : 0UL)))

static void *g_gate;		/* the "tombstone" */
static void *g_payload;		/* a word the op writes */
static void *g_w;		/* same-slot validate/store cases */
static void *g_ctrl;		/* the caller-expected guard's control word */

int main(void)
{
	struct urcu_txn_mw tx;
	void *gv, *vv;
	unsigned int nr1, nr2;
	enum urcu_txn_status st;

	plan_tests(NR_TESTS);
	rcu_register_thread();

	/* 1. Guard holds: commit OK, payload written, gate left untouched. */
	g_gate = CLEAR;
	g_payload = P0;
	urcu_txn_mw_init(&tx, NULL);
	urcu_txn_mw_begin(&tx);
	gv = urcu_txn_mw_load_validate(&tx, &g_gate, URCU_MCAS_TAG);
	urcu_txn_mw_store(&tx, &g_payload, P0, P1, URCU_MCAS_TAG);
	st = urcu_txn_mw_commit(&tx);
	urcu_txn_mw_end(&tx);
	ok(gv == CLEAR && st == URCU_TXN_STATUS_OK &&
			g_gate == CLEAR && g_payload == P1,
		"guard holds -> commit applies the write, leaves the guarded word");

	/* 2. Guard fails: a peer sets the gate before our install -> ABORT,
	 * and the buffered write must NOT take effect. */
	g_gate = CLEAR;
	g_payload = P0;
	urcu_txn_mw_init(&tx, NULL);
	urcu_txn_mw_begin(&tx);
	gv = urcu_txn_mw_load_validate(&tx, &g_gate, URCU_MCAS_TAG);
	urcu_txn_mw_store(&tx, &g_payload, P0, P1, URCU_MCAS_TAG);
	g_gate = SET;			/* simulated racing tombstone */
	st = urcu_txn_mw_commit(&tx);
	urcu_txn_mw_end(&tx);
	ok(gv == CLEAR && st == URCU_TXN_STATUS_ABORT && g_payload == P0,
		"guard fails -> commit aborts and the write is not applied");

	/* 3. validate-then-store same slot: the store upgrades the guard in
	 * place (one record), and commits as the write.  Like case 6 this is a
	 * deliberate same-slot self-conflict, so declare it -- otherwise an
	 * AGE_ESCALATE build would escalate on the age-0 coincidence rather than
	 * chain into the one record this asserts.  Inert in a stock build. */
	g_w = VX;
	urcu_txn_mw_init(&tx, NULL);
	urcu_txn_mw_expect_conflict(&tx);
	urcu_txn_mw_begin(&tx);
	vv = urcu_txn_mw_load_validate(&tx, &g_w, URCU_MCAS_TAG);
	urcu_txn_mw_store(&tx, &g_w, VX, VZ, URCU_MCAS_TAG);
	nr1 = tx.mcas->nr;
	st = urcu_txn_mw_commit(&tx);
	urcu_txn_mw_end(&tx);
	ok(vv == VX && nr1 == 1 && st == URCU_TXN_STATUS_OK && g_w == VZ,
		"validate-then-store on one slot -> one record, commits the write");

	/* 4. urcu_txn_mw_load_committed() is the scoped escape hatch from RYW: after
	 * buffering a store, it reads past the transaction's own pending write and
	 * returns the slot's COMMITTED value.  It is a read-only decision helper --
	 * it records nothing (nr unchanged from the lone store) and does not disturb
	 * the write, which still commits. */
	g_w = VX;
	urcu_txn_mw_init(&tx, NULL);
	urcu_txn_mw_begin(&tx);
	urcu_txn_mw_store(&tx, &g_w, VX, VZ, URCU_MCAS_TAG);
	vv = urcu_txn_mw_load_committed(&tx, &g_w, URCU_MCAS_TAG);
	nr2 = tx.mcas->nr;
	st = urcu_txn_mw_commit(&tx);
	urcu_txn_mw_end(&tx);
	ok(vv == VX && nr2 == 1 && st == URCU_TXN_STATUS_OK && g_w == VZ,
		"load_committed past a pending store -> reads committed value, write preserved");

	/* 5. Pure guard over an unchanged word: commit OK, no modification. */
	g_w = VX;
	urcu_txn_mw_init(&tx, NULL);
	urcu_txn_mw_begin(&tx);
	vv = urcu_txn_mw_load_validate(&tx, &g_w, URCU_MCAS_TAG);
	st = urcu_txn_mw_commit(&tx);
	urcu_txn_mw_end(&tx);
	ok(vv == VX && st == URCU_TXN_STATUS_OK && g_w == VX,
		"pure guard over an unchanged word commits without modifying it");

	/* 6. store-then-validate on one slot: under the default read-your-own-writes
	 * the validate observes this transaction's PENDING value (not the committed
	 * one), and its record chains rather than poisoning (the old it presents is
	 * the pending new).  One record; the write stands. */
	g_w = VX;
	urcu_txn_mw_init(&tx, NULL);
	/*
	 * Same-slot store+validate is a deliberate self-conflict: under an
	 * AGE_ESCALATE build age 0 would escalate on the coincidence rather than
	 * chain into one record, so declare it to exercise the age-1 chaining path
	 * this asserts.  Inert (a no-op) in a stock build.
	 */
	urcu_txn_mw_expect_conflict(&tx);
	urcu_txn_mw_begin(&tx);
	urcu_txn_mw_store(&tx, &g_w, VX, VZ, URCU_MCAS_TAG);
	vv = urcu_txn_mw_load_validate(&tx, &g_w, URCU_MCAS_TAG);
	nr2 = tx.mcas->nr;
	st = urcu_txn_mw_commit(&tx);
	urcu_txn_mw_end(&tx);
	ok(vv == VZ && nr2 == 1 && st == URCU_TXN_STATUS_OK && g_w == VZ,
		"store-then-validate, RYW -> reads its own pending write, one record, write stands");

	/* 7. urcu_txn_mw_validate(): the expected image is COMPUTED, never read
	 * through the bracket (which is what urcu_txn_mw_load_validate() cannot
	 * express): the mutator knows the control word's stable bits and predicts
	 * its marker.  It matches, so the commit stands. */
	g_ctrl = CTRL(1);
	g_payload = P0;
	urcu_txn_mw_init(&tx, NULL);
	urcu_txn_mw_begin(&tx);
	urcu_txn_mw_validate(&tx, &g_ctrl, CTRL(1), URCU_MCAS_TAG);
	urcu_txn_mw_store(&tx, &g_payload, P0, P1, URCU_MCAS_TAG);
	st = urcu_txn_mw_commit(&tx);
	urcu_txn_mw_end(&tx);
	ok(st == URCU_TXN_STATUS_OK && g_ctrl == CTRL(1) && g_payload == P1,
		"validate against a computed expected: it holds -> commit, guarded word untouched");

	/* 8. A peer changes the word before the install: the guard's record fails
	 * its plant CAS, so the commit aborts and the co-recorded write does not
	 * take effect. */
	g_ctrl = CTRL(1);
	g_payload = P0;
	urcu_txn_mw_init(&tx, NULL);
	urcu_txn_mw_begin(&tx);
	urcu_txn_mw_validate(&tx, &g_ctrl, CTRL(1), URCU_MCAS_TAG);
	urcu_txn_mw_store(&tx, &g_payload, P0, P1, URCU_MCAS_TAG);
	g_ctrl = VX;			/* simulated racing peer */
	st = urcu_txn_mw_commit(&tx);
	urcu_txn_mw_end(&tx);
	ok(st == URCU_TXN_STATUS_ABORT && g_payload == P0,
		"validate against a computed expected: a peer moved the word -> commit aborts, the write is not applied");

	/* 9. The guard is an EXACT word match, not a mask.  Guarding the same word
	 * against its CLEAN image (marker cleared) while the marker is in fact set
	 * aborts: to express "the stable bits, with the marker in the state I
	 * expect", the caller folds the marker it predicts INTO the expected word --
	 * which is exactly what case 7 did.  Pinned here so nobody reads "masked
	 * expected" as "the engine ignores the subfield". */
	g_ctrl = CTRL(1);
	g_payload = P0;
	urcu_txn_mw_init(&tx, NULL);
	urcu_txn_mw_begin(&tx);
	urcu_txn_mw_validate(&tx, &g_ctrl, CTRL(0), URCU_MCAS_TAG);
	urcu_txn_mw_store(&tx, &g_payload, P0, P1, URCU_MCAS_TAG);
	st = urcu_txn_mw_commit(&tx);
	urcu_txn_mw_end(&tx);
	ok(st == URCU_TXN_STATUS_ABORT && g_ctrl == CTRL(1) && g_payload == P0,
		"the guard is an exact word match, not a mask: an expected that clears a set marker aborts");

	rcu_barrier();			/* drain deferred descriptor frees */
	rcu_unregister_thread();
	return exit_status();
}
