// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

/*
 * Deterministic regression test for the lock-free MCAS engine's behaviour when a
 * transacted slot A-B-A's -- i.e. a slot's value recurs.  The engine no longer
 * assumes slots never repeat a bit pattern: a doubly-linked list's next-pointer
 * does so intrinsically (insert X between A and B records "A->next: B->X"; delete
 * X records "A->next: X->B", so &A->next cycles B -> X -> B with B a *live*
 * successor that RCU has no reason to keep from recurring).
 *
 * The hazard: an INSERT transaction V commits (SUCCEEDED), a higher-priority
 * DELETE T steals V's slot, commits, and settles it back to V's record's old
 * value (delete.new == insert.old == B).  A stale driver of V is still inside
 * drive_install for that record -- it passed the status check while V was
 * UNDECIDED, then stalled.  When it resumes, the slot reads B == old_ptr, so a
 * value-CAS alone would succeed and RE-PLANT V's proxy *after* V committed and
 * was stolen; V's settle then writes V's new value (X), resurrecting the
 * just-deleted node onto a live edge -> use-after-free.
 *
 * The fix is the per-record install latch: every plant runs through
 * urcu_mcas_plant() under the record's latch, gated by an install-once FLAG
 * (not by the slot value), so a stale second install is skipped no matter how the
 * slot value recurred.  This test drives that exact interleaving deterministically
 * with a single thread and the URCU_MCAS_PREINSTALL hook: when the stale driver
 * is about to plant the shared slot S, the hook runs everything that races with it
 * -- a helper *properly installs* V's record on S (through plant, so the flag is
 * set, exactly as a real helper would), V commits, and T steals+commits+settles S
 * back to B.  The stale driver then proceeds into plant(), which must see the
 * install-once flag and skip, leaving S == B.
 *
 * Expected, linearized result: V (insert) before T (delete) -> S holds B (the
 * node was inserted then deleted).  The bug leaves S == X (resurrected).
 *
 * Build -DURCU_MCAS_NO_ABA_FIX to compile the engine with the install-once
 * gate removed and watch this test fail -- that is the proof it catches the
 * regression.
 */

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#ifndef _LGPL_SOURCE
#define _LGPL_SOURCE
#endif

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include <urcu/compiler.h>
#include <urcu/uatomic.h>
#include <urcu-qsbr.h>
#include <urcu-call-rcu.h>

/*
 * Install the deterministic interleaving hook before pulling in the engine: the
 * header invokes URCU_MCAS_PREINSTALL(t, r) in the install path, at the point
 * a driver has decided to plant r but BEFORE it takes r's install latch.
 */
struct urcu_mcas;
struct urcu_mcas_record;
static void preinstall_hook(struct urcu_mcas *t,
		struct urcu_mcas_record *r);
#define URCU_MCAS_PREINSTALL(t, r)	preinstall_hook((t), (r))
#include <urcu/rcu-mcas.h>

#include "tap.h"

#define NR_TESTS	2

/*
 * Three transacted words in one array so their addresses are ordered: the shared
 * edge S is the highest, hence slot-sorted last in each transaction (the owner
 * installs the auxiliary edge first, then reaches S -- where the hook fires).
 */
static void *g_word[3];
#define S	(&g_word[2])		/* the shared edge (== A->next) */
#define VAUX	(&g_word[0])		/* V's second edge */
#define TAUX	(&g_word[1])		/* T's second edge */

/* Distinct, pointer-aligned sentinel values (bit 0 free for the engine tag). */
static long o_B, o_X, o_vaux0, o_vaux1, o_taux0, o_taux1;
#define B	((void *) &o_B)		/* the live successor */
#define X	((void *) &o_X)		/* the node inserted then deleted */

/* Hook state: the racing transactions. */
static struct urcu_mcas *g_V, *g_T;
static int g_fired;

/*
 * Pre-install hook: fires inside the stale driver of V, after it decided to plant
 * S (slot reads B == old_ptr) and BEFORE it takes S's install latch.  Runs the
 * whole race that the stale driver would otherwise be paused through, exactly once
 * for V's S record:
 *   (a) a helper installs V's record on S THROUGH plant -- so the install-once
 *       flag is set, exactly as a real concurrent helper would leave it;
 *   (b) V commits (its only/last record is now installed);
 *   (c) the higher-priority T steals S from the SUCCEEDED V, commits, and settles
 *       S back to B (== V's record old_ptr) -- the A-B-A.
 * On return, the stale driver proceeds into plant(S): with the fix it finds the
 * install-once flag and skips; without it, it re-plants V's proxy over B.
 */
static void preinstall_hook(struct urcu_mcas *t,
		struct urcu_mcas_record *r)
{
	if (g_fired || t != g_V || r->slot != S)
		return;
	g_fired = 1;

	/* (a) a helper properly installs V's record on S (sets install-once) */
	(void) urcu_mcas_plant(g_V, r, r->old_ptr);
	/* (b) V commits: every record installed */
	(void) uatomic_cmpxchg(&g_V->status, URCU_MCAS_UNDECIDED,
			URCU_MCAS_SUCCEEDED);
	/* (c) T steals S from the terminal V, commits, settles S = B */
	urcu_mcas_drive_install(g_T);
	urcu_mcas_settle(g_T);
}

static struct urcu_mcas *make_txn(void **slot_a, void *a_old, void *a_new,
		void **slot_s, void *s_old, void *s_new)
{
	struct urcu_mcas *t = urcu_mcas_create(2, 0);
	unsigned int i;

	if (!t)
		abort();
	urcu_mcas_add(t, slot_a, a_old, a_new);
	urcu_mcas_add(t, slot_s, s_old, s_new);
	urcu_mcas_sort(t);		/* slot-address order, as commit() does */
	for (i = 0; i < t->nr; i++) {
		t->recs[i].mcas = t;		/* back-pointers, as commit() does */
		t->recs[i].installed = 0;	/* arm the install latch, as commit() does */
		cds_fair_mutex_init(&t->recs[i].latch);
	}
	return t;
}

int main(void)
{
	void *final_s;

	plan_tests(NR_TESTS);
	rcu_register_thread();

	/* Initial list state: ... A -> B ..., with A->next == B (the slot S). */
	g_word[0] = (void *) &o_vaux0;
	g_word[1] = (void *) &o_taux0;
	g_word[2] = B;

	/* V = insert X between A and B:  S: B -> X  (+ one auxiliary edge). */
	g_V = make_txn(VAUX, &o_vaux0, &o_vaux1, S, B, X);
	/* T = delete X:  S: X -> B  (+ one auxiliary edge). */
	g_T = make_txn(TAUX, &o_taux0, &o_taux1, S, X, B);

	g_fired = 0;
	rcu_read_lock();
	/*
	 * Drive V as a stale driver.  The hook fires when V reaches S and runs a
	 * helper's proper install of S, V's commit, and T's full steal+commit+
	 * settle, so V's subsequent plant of S is the stale post-commit re-plant
	 * attempt.  The install-once flag must make it a no-op, so settle(V) cannot
	 * resurrect X.
	 */
	urcu_mcas_drive_install(g_V);
	urcu_mcas_settle(g_V);
	rcu_read_unlock();

	ok(g_fired, "the A-B-A interleaving hook actually fired");

	final_s = uatomic_load(S, CMM_ACQUIRE);
	diag("final S = %s (B=%p X=%p, raw=%p)",
		final_s == B ? "B (correct: insert then delete)" :
		final_s == X ? "X (BUG: deleted node resurrected)" : "other",
		B, X, final_s);
	ok(final_s == B,
		"deleted node not resurrected onto the live edge by a stale re-install");

	urcu_mcas_destroy(g_V);
	urcu_mcas_destroy(g_T);
	rcu_unregister_thread();
	return exit_status();
}
