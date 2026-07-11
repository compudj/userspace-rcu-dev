// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

/*
 * Deterministic regression test for the settle-time install-word claim: a stale
 * FIRST install must not REPUBLISH a retired descriptor, even transiently.
 *
 * The companion tests cover the neighbouring hazards: test_rcu_mcas_aba.c the
 * stale SECOND install of an already-planted record (closed by the install-once
 * DONE gate), test_rcu_mcas_linger.c the late first install LINGERING in a slot
 * (closed by the installer-self-settle).  This test covers what those two
 * mechanisms alone do not: the self-settle removes a late first install's proxy
 * only AFTER the plant CAS published it, so for a few instructions a slot names
 * a descriptor whose owner has already settled and called call_rcu().  A reader
 * whose read-side section began after that call_rcu() is not waited for by the
 * grace period; if it resolves the proxy inside the plant->self-settle window it
 * dereferences the descriptor after the GP ends -> use-after-free.  "No proxy
 * lingers" is not enough; reclaim needs "no proxy can appear at all".
 *
 * The fix is in urcu_mcas_settle(): before converting each slot it CLAIMS the
 * record's install word (FREE -> DONE), so once settle returns every record's
 * word is DONE and a stale plant fails its FREE->BUSY CAS -- refused without
 * touching the slot, no republication window at all.
 *
 * The hazard interleaving (eviction variant -- needs NO slot-value A-B-A): a
 * HELPER drives transaction V, installs V's lower-slot record, then -- having
 * just read V UNDECIDED -- stalls right before planting V's last record on slot
 * S (never planted by anyone: its install word is FREE).  V is EVICTED by a
 * higher-priority contender (V -> FAILED), and V's owner settles and retires the
 * descriptor (call_rcu).  The helper then resumes into urcu_mcas_plant(): S
 * still holds old_ptr (nothing ever changed it), so only the install word can
 * stop the plant.
 *
 * This drives that interleaving deterministically with one thread.  The
 * URCU_MCAS_PREINSTALL hook, firing at the helper's exact stall point, runs the
 * eviction and the owner's settle (the retire point), then itself performs the
 * stale resumed plant and records what the engine did:
 *   - S's install word after settle returns: must be DONE (claimed);
 *   - the stale plant's return: must be 2 (refused, "already installed");
 *   - the raw slot right after the refused plant: must be plain (no proxy was
 *     ever published).
 *
 * Build with the settle-time claim removed (or -DURCU_MCAS_NO_ABA_FIX) to watch
 * the word stay FREE and the stale plant return 1 (republished) -- the
 * regression this catches.
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

struct urcu_mcas;
struct urcu_mcas_record;
static void preinstall_hook(struct urcu_mcas *t,
		struct urcu_mcas_record *r);
#define URCU_MCAS_PREINSTALL(t, r)	preinstall_hook((t), (r))
#include <urcu/rcu-mcas.h>

#include "tap.h"

#define NR_TESTS	6

/* Two transacted words: AUX (lower, installed first) and S (higher, the stale one). */
static void *g_word[2];
#define AUX	(&g_word[0])		/* V's lower-slot record */
#define S	(&g_word[1])		/* V's last record -- never planted before the retire */

/* Distinct, pointer-aligned sentinel values (bit 0 free for the engine tag). */
static long o_s_old, o_s_new, o_aux_old, o_aux_new;
#define S_OLD	((void *) &o_s_old)
#define S_NEW	((void *) &o_s_new)

static struct urcu_mcas *g_V;
static int g_fired;
static int g_state_after_settle;	/* S record's install word once settle returned */
static int g_stale_plant_ret;		/* the stale resumed plant's return value */
static void *g_slot_after_plant;	/* raw S right after the stale plant returned */

/*
 * Pre-install hook: fires inside the helper driving V, after it read V UNDECIDED
 * and decided to plant S, and BEFORE it takes S's install latch -- the exact
 * stall point.  Runs the whole race the helper would otherwise be paused
 * through, then the helper's own stale resumption:
 *   (a) a higher-priority contender evicts V (V -> FAILED);
 *   (b) V's owner settles.  This is the retire point: the owner's next action is
 *       call_rcu(V), and the grace period covers no reader that starts later.
 *       Settle must therefore leave S's install word DONE (claimed) -- S's slot
 *       CAS is a no-op (nothing is parked there), but the claim is what gates
 *       the future;
 *   (c) the stale resumed plant of S: the slot still holds S_OLD (no A-B-A
 *       needed), so only the claimed install word can refuse it.  Must return 2
 *       ("already installed") and leave the raw slot plain -- a return of 1
 *       means a proxy of the retired V was published to any reader the GP does
 *       not cover.
 */
static void preinstall_hook(struct urcu_mcas *t,
		struct urcu_mcas_record *r)
{
	if (g_fired || t != g_V || r->slot != S)
		return;
	g_fired = 1;

	/* (a) a higher-priority contender evicts V */
	(void) uatomic_cmpxchg(&g_V->status, URCU_MCAS_UNDECIDED,
			URCU_MCAS_FAILED);
	/* (b) V's owner settles (AUX -> old; S never parked) and retires V.
	 * Help/steal build: settle load-tests each slot, so the planted count is
	 * unused here (S is skipped because it holds no proxy of V's). */
	urcu_mcas_settle(g_V, g_V->nr);
	g_state_after_settle = uatomic_load(&r->state, CMM_ACQUIRE);
	/* (c) the helper's stale resumed plant -- past the retire point */
	g_stale_plant_ret = urcu_mcas_plant(g_V, r, r->old_ptr);
	g_slot_after_plant = uatomic_load(S, CMM_ACQUIRE);
}

static struct urcu_mcas *make_txn(void)
{
	struct urcu_mcas *t = urcu_mcas_create(2, 0);
	unsigned int i;

	if (!t)
		abort();
	/* V: AUX: aux_old -> aux_new, and S: S_OLD -> S_NEW. */
	urcu_mcas_add(t, AUX, &o_aux_old, &o_aux_new, URCU_MCAS_TAG);
	urcu_mcas_add(t, S, S_OLD, S_NEW, URCU_MCAS_TAG);
	urcu_mcas_sort(t);		/* slot-address order, as commit() does */
	for (i = 0; i < t->nr; i++) {
		t->recs[i].mcas = t;
		urcu_mcas_latch_init(&t->recs[i]);
	}
	return t;
}

int main(void)
{
	void *final_s;

	plan_tests(NR_TESTS);
	rcu_register_thread();

	/* Initial slot state. */
	g_word[0] = (void *) &o_aux_old;
	g_word[1] = S_OLD;

	g_V = make_txn();

	g_fired = 0;
	rcu_read_lock();
	/*
	 * Drive V as a HELPER: install forward, never settle.  The hook fires at
	 * the stall point before planting S and runs V's eviction, the owner's
	 * settle (the retire point) and the stale resumed plant; the drive's own
	 * plant call right after the hook returns is refused the same way (word
	 * already DONE) and the drive then observes V terminal and returns.
	 */
	urcu_mcas_drive_install(g_V);
	rcu_read_unlock();

	ok(g_fired, "the stale-first-install interleaving hook actually fired");

	diag("S record install word after settle: %d (FREE=%d BUSY=%d DONE=%d)",
		g_state_after_settle, URCU_MCAS_INSTALL_FREE,
		URCU_MCAS_INSTALL_BUSY, URCU_MCAS_INSTALL_DONE);
	ok(g_state_after_settle == URCU_MCAS_INSTALL_DONE,
		"settle claimed the never-planted record's install word (FREE -> DONE)");

	diag("stale plant returned %d (1 = planted/REPUBLISHED, 2 = refused)",
		g_stale_plant_ret);
	ok(g_stale_plant_ret == 2,
		"a stale first install after the retire point is refused, not planted");

	ok(!urcu_mcas_is_proxy(g_slot_after_plant, URCU_MCAS_TAG) &&
			g_slot_after_plant == S_OLD,
		"the refused plant never touched the slot (no republication window)");

	final_s = uatomic_load(S, CMM_ACQUIRE);
	ok(!urcu_mcas_is_proxy(final_s, URCU_MCAS_TAG) && final_s == S_OLD,
		"final S is the plain old value (FAILED txn)");
	ok(g_word[0] == (void *) &o_aux_old,
		"final AUX settled back to its old value (FAILED txn)");

	urcu_mcas_destroy(g_V);
	rcu_unregister_thread();
	return exit_status();
}
