// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

/*
 * Deterministic regression test for the RCU MCAS engine's OTHER A-B-A
 * hazard: a late FIRST install that lingers past reclaim.  The companion test
 * test_rcu_mcas_aba.c covers the re-plant of an already-installed record
 * (closed by the install-once flag); this one covers the install-vs-settle
 * ordering hole (closed by the installer-self-settle).
 *
 * The hazard: a transaction V is being driven by a HELPER (a thread other than
 * the owner).  The helper installs V's lower-slot record, then -- having just
 * read V UNDECIDED -- is about to install V's last record on slot S.  Before it
 * does, V is EVICTED by a higher-priority contender (V -> FAILED), and V's OWNER
 * runs its settle and reclaims the descriptor.  The owner's settle cannot convert
 * the S record: it is not installed yet.  The helper then plants S anyway (a late
 * FIRST install, after the owner already settled and freed V).  That proxy now
 * names a reclaimed descriptor: any reader resolving S dereferences freed memory
 * -> use-after-free, and the slot never decays to a plain value.
 *
 * The fix is the installer-self-settle in urcu_mcas_plant(): a driver that
 * plants a record reads V's status AFTER the plant (under the install latch); if
 * V is already terminal it converts the just-planted proxy itself, so a late
 * first install cannot linger.  (Correctness is an SB exclusion with the regular
 * owner settle for the still-UNDECIDED case -- see the engine header.)
 *
 * This drives that interleaving deterministically with one thread.  The helper is
 * urcu_mcas_drive_install(V) with NO settle (helpers never settle); the
 * URCU_MCAS_PREINSTALL hook, firing just before the helper plants S, runs the
 * eviction and the owner's settle.  After the helper returns, slot S must hold a
 * PLAIN value (no lingering proxy).
 *
 * Build -DURCU_MCAS_NO_ABA_FIX to drop the self-settle (and the install-once
 * gate) and watch S retain a dangling proxy -- the regression this catches.
 *
 * Note: since urcu_mcas_settle() learned to CLAIM each record's install word,
 * this scenario's late plant is refused outright (word already DONE), which
 * satisfies the "no lingering proxy" assertion a fortiori; the self-settle
 * still covers a plant that completes before settle reaches its record.  The
 * companion test_rcu_mcas_republish.c asserts the claim itself.
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

#define NR_TESTS	2

/* Two transacted words: AUX (lower, installed first) and S (higher, the late one). */
static void *g_word[2];
#define AUX	(&g_word[0])		/* V's lower-slot record */
#define S	(&g_word[1])		/* V's last record -- the late first install */

/* Distinct, pointer-aligned sentinel values (bit 0 free for the engine tag). */
static long o_s_old, o_s_new, o_aux_old, o_aux_new;
#define S_OLD	((void *) &o_s_old)
#define S_NEW	((void *) &o_s_new)

static struct urcu_mcas *g_V;
static int g_fired;

/*
 * Pre-install hook: fires inside the helper driving V, after it read V UNDECIDED
 * and decided to plant S, and BEFORE it takes S's install latch.  Runs what the
 * helper would otherwise be paused through:
 *   (a) a higher-priority contender evicts V (V -> FAILED);
 *   (b) V's owner settles and would then reclaim V.  Settle converts the already-
 *       installed AUX record; it CANNOT convert S (not installed yet), so S keeps
 *       its plain old value here.
 * On return, the helper plants S as a late FIRST install of a now-FAILED, already-
 * settled transaction.  With the self-settle it converts that proxy immediately;
 * without it the proxy lingers on S past V's (modelled) reclaim.
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
	/* (b) V's owner settles (AUX -> old; S not installed, so no-op) and reclaims */
	urcu_mcas_settle(g_V);
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
	 * Drive V as a HELPER: install forward, never settle.  The hook fires when
	 * the helper reaches S and runs V's eviction + the owner's settle, so the
	 * helper's plant of S is a late first install of an already-settled FAILED
	 * transaction.  The self-settle must convert it so no proxy lingers.
	 */
	urcu_mcas_drive_install(g_V);
	rcu_read_unlock();

	ok(g_fired, "the late-first-install interleaving hook actually fired");

	final_s = uatomic_load(S, CMM_ACQUIRE);
	diag("final S: %s (raw=%p, S_OLD=%p, tag(S-rec) is a proxy)",
		urcu_mcas_is_proxy(final_s, URCU_MCAS_TAG) ? "PROXY (BUG: lingering past reclaim)" :
		final_s == S_OLD ? "S_OLD (correct: FAILED, settled to old)" : "other plain",
		final_s, S_OLD);
	ok(!urcu_mcas_is_proxy(final_s, URCU_MCAS_TAG),
		"no proxy lingers on the slot after a late first install of a settled txn");

	urcu_mcas_destroy(g_V);
	rcu_unregister_thread();
	return exit_status();
}
