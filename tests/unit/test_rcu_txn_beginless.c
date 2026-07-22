// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

/*
 * Test the BEGIN-LESS driving mode of <urcu/rcu-txn.h>: an embedder may drive
 * the engine without urcu_txn_begin()/urcu_txn_end(), bracketing the mutation
 * itself with urcu_txn_read_lock()/urcu_txn_read_unlock() and finishing with
 * urcu_txn_commit_flavor():
 *
 *     urcu_txn_init(&txn, domain);
 *     urcu_txn_read_lock(&txn);
 *     ... urcu_txn_load() / urcu_txn_store_mw() ...
 *     st = urcu_txn_commit_flavor(&txn, call_rcu);
 *     urcu_txn_read_unlock(&txn);
 *
 * The mode is documented (see urcu_txn_read_lock) and used by an embedder that
 * owns its own retry/bracket structure.  It has two dependencies that begin()
 * would otherwise have satisfied, and both were broken:
 *
 *   - the read-your-own-writes Bloom filter must be EMPTY before it is first
 *     consulted.  begin() used to be the only thing that zeroed it, so a
 *     begin-less handle tested indeterminate stack bytes; set bits there read as
 *     a same-slot coincidence and forced an abort.  It is now emptied by
 *     urcu_txn__bloom_reset() when the attempt's descriptor is first allocated,
 *     which happens in this mode too (and which a disjoint handle skips, so the
 *     fix costs the disjoint fast path nothing).
 *   - esc_pending must not survive the abort it caused.  begin() used to be the
 *     only thing that cleared it, so once a begin-less attempt set it, EVERY
 *     later attempt aborted -- at any age -- forever.  commit_flavor now
 *     consumes it on its own abort path.
 *
 * Against the pre-fix engine test 1 below fails outright: the transaction
 * aborts on every one of its 1000 attempts and NEVER commits.  So each attempt
 * here deliberately runs on a DIRTIED stack frame (memset 0xff) -- otherwise a
 * zeroed stack would hide the filter bug by accident.
 *
 * Deterministic and single-threaded (no contention => no genuine abort).
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
#include <string.h>

#include <urcu/compiler.h>
#include <urcu-qsbr.h>
#include <urcu-call-rcu.h>
#include <urcu/rcu-txn.h>

#include "tap.h"

#define NR_TESTS	6

#define NR_ITER		1000	/* enough that a sticky esc_pending cannot hide */

/* Opaque, bit-0-clear slot values (the engine owns bit 0 as its proxy tag). */
#define V0	((void *) 0x100)
#define V1	((void *) 0x200)
#define V2	((void *) 0x300)

static void *g_a, *g_b;		/* two transacted slots */
static void *g_c;		/* read-your-own-writes subject */

/*
 * One begin-less attempt on a deliberately dirty handle: advance both slots by
 * 2 (keeping bit 0 clear).  Returns the commit status.
 */
static enum urcu_txn_status beginless_bump(struct urcu_txn_domain *domain)
{
	struct urcu_txn txn;
	void *a, *b;
	enum urcu_txn_status st;

	memset(&txn, 0xff, sizeof(txn));	/* dirty frame: no accidental zeros */
	urcu_txn_init(&txn, domain);
	urcu_txn_read_lock(&txn);		/* no begin() */
	a = urcu_txn_load(&txn, &g_a, URCU_TXN_TAG);
	urcu_txn_store_mw(&txn, &g_a, a, (void *) ((uintptr_t) a + 2), URCU_TXN_TAG);
	b = urcu_txn_load(&txn, &g_b, URCU_TXN_TAG);
	urcu_txn_store_mw(&txn, &g_b, b, (void *) ((uintptr_t) b + 2), URCU_TXN_TAG);
	st = urcu_txn_commit_flavor(&txn, call_rcu);
	urcu_txn_read_unlock(&txn);		/* no end() */
	return st;
}

int main(void)
{
	struct urcu_txn_domain domain;
	struct urcu_txn txn;
	enum urcu_txn_status st, first;
	unsigned int oks = 0, aborts = 0, i;
	void *seen;
	int ret;

	plan_tests(NR_TESTS);
	rcu_register_thread();
	urcu_txn_domain_init(&domain);

	/*
	 * 1. The first begin-less attempt commits.  Pre-fix this ABORTs: the
	 *    unzeroed filter flags a phantom coincidence on the very first load.
	 */
	st = beginless_bump(&domain);
	ok(st == URCU_TXN_STATUS_OK,
		"begin-less: the FIRST attempt commits (no phantom coincidence from an unzeroed filter)");

	/*
	 * 2. And it keeps committing.  Pre-fix, even if attempt 1 had slipped
	 *    through, the first one to set esc_pending would wedge every later
	 *    attempt: nothing but begin() cleared it.
	 */
	for (i = 1; i < NR_ITER; i++) {
		st = beginless_bump(&domain);
		if (st == URCU_TXN_STATUS_OK)
			oks++;
		else if (st == URCU_TXN_STATUS_ABORT)
			aborts++;
	}
	ok(oks == NR_ITER - 1 && aborts == 0,
		"begin-less: all %u attempts commit, 0 aborts (esc_pending does not survive its own abort)",
		NR_ITER);

	/* 3. The commits actually landed: uncontended, so every one applied. */
	ok((uintptr_t) g_a == 2 * NR_ITER && (uintptr_t) g_b == 2 * NR_ITER,
		"begin-less: both slots carry every commit (a=%lu b=%lu, want %u)",
		(unsigned long) (uintptr_t) g_a, (unsigned long) (uintptr_t) g_b,
		2 * NR_ITER);

	/*
	 * 4. Read-your-own-writes works in this mode: the filter is live (it was
	 *    emptied at descriptor allocation, not merely left dirty), so a load
	 *    of an already-stored slot returns this attempt's PENDING value.
	 *
	 *    This needs age >= 1: at age 0 the RYW path is the stripped one --
	 *    it maintains the filter but never runs find, so a coincidence sets
	 *    esc_pending and returns the committed value instead (test 5 covers
	 *    exactly that).  urcu_txn_expect_conflict() is the documented knob
	 *    for a handle whose RYW is dense by construction: it puts the FIRST
	 *    attempt on the sorted, find-resolved path.
	 */
	memset(&txn, 0xff, sizeof(txn));
	urcu_txn_init(&txn, &domain);
	urcu_txn_expect_conflict(&txn);		/* age >= 1 from attempt 0: the find path */
	urcu_txn_read_lock(&txn);
	g_c = V0;
	urcu_txn_store_mw(&txn, &g_c, V0, V1, URCU_TXN_TAG);
	seen = urcu_txn_load(&txn, &g_c, URCU_TXN_TAG);
	st = urcu_txn_commit_flavor(&txn, call_rcu);
	urcu_txn_read_unlock(&txn);
	ok(seen == V1 && st == URCU_TXN_STATUS_OK && g_c == V1,
		"begin-less + expect_conflict: read-your-own-writes returns the pending value, commit stands");

	/*
	 * 5. The sharpest test of the esc_pending fix.  A default handle's age-0
	 *    attempt that hits a same-slot coincidence (here a write-after-write)
	 *    MUST abort -- that is the design -- and the RETRY, still begin-less,
	 *    must then commit at age 1 where find chains the two stores into one
	 *    record.  Pre-fix, esc_pending survived the abort it caused and only
	 *    begin() cleared it, so the retry aborted too, and so did every
	 *    attempt after it: the begin-less handle wedged forever.
	 */
	memset(&txn, 0xff, sizeof(txn));
	urcu_txn_init(&txn, &domain);
	g_c = V0;
	urcu_txn_read_lock(&txn);
	urcu_txn_store_mw(&txn, &g_c, V0, V1, URCU_TXN_TAG);
	urcu_txn_store_mw(&txn, &g_c, V1, V2, URCU_TXN_TAG);	/* same slot: age-0 coincidence */
	first = urcu_txn_commit_flavor(&txn, call_rcu);
	urcu_txn_read_unlock(&txn);

	urcu_txn_read_lock(&txn);		/* retry: same handle, now age 1 */
	urcu_txn_store_mw(&txn, &g_c, V0, V1, URCU_TXN_TAG);
	urcu_txn_store_mw(&txn, &g_c, V1, V2, URCU_TXN_TAG);	/* chains onto the record */
	st = urcu_txn_commit_flavor(&txn, call_rcu);
	urcu_txn_read_unlock(&txn);
	ok(first == URCU_TXN_STATUS_ABORT && st == URCU_TXN_STATUS_OK && g_c == V2,
		"begin-less: an age-0 coincidence aborts ONCE, then the retry commits (esc_pending is consumed)");

	/*
	 * 6. reserve() is the other site that first allocates the descriptor, so
	 *    it must empty the filter too -- otherwise a begin-less handle that
	 *    reserves before storing consults a dirty one.
	 */
	memset(&txn, 0xff, sizeof(txn));
	urcu_txn_init(&txn, &domain);
	urcu_txn_read_lock(&txn);
	ret = urcu_txn_reserve(&txn, 2);
	g_c = V0;
	seen = urcu_txn_load(&txn, &g_c, URCU_TXN_TAG);	/* consults the filter */
	urcu_txn_store_mw(&txn, &g_c, seen, V1, URCU_TXN_TAG);
	st = urcu_txn_commit_flavor(&txn, call_rcu);
	urcu_txn_read_unlock(&txn);
	ok(ret == 0 && seen == V0 && st == URCU_TXN_STATUS_OK && g_c == V1,
		"begin-less: reserve() before the first store also empties the filter");

	rcu_barrier();			/* drain deferred descriptor frees */
	rcu_unregister_thread();
	return exit_status();
}
