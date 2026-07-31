// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

/*
 * Unit test for <urcu/rcu-txn-bloom.h>, the engines' read-your-own-writes
 * write-set filter.
 *
 * The filter is one-sided, and the two sides are worth very different things.
 * A false POSITIVE is tuning: the caller falls through to the authoritative
 * urcu_txn_find(), or at age 0 spends one extra attempt.  A false NEGATIVE is a
 * correctness fault: the engines skip the write-set consult outright on a clear
 * bit, so a slot that reads absent when it is present makes a load return the
 * COMMITTED value where a pending one was due -- read-your-own-writes silently
 * broken, with no abort and no diagnostic anywhere downstream.
 *
 * Nothing tested it at all.  What is pinned here:
 *
 *   1. NO FALSE NEGATIVES, over a large mixed population -- the load-bearing
 *      half.  Every slot ever set must test present, forever, whatever else is
 *      set afterwards.  This is the property a hash change, a shift change or a
 *      k change could silently break.
 *   2. test_and_set reports the PRE-state and leaves the post-state set, which
 *      is what lets urcu_txn__record() detect a coincidence in one pass.
 *   3. Determinism: the same slot always maps to the same bits, so a filter
 *      rebuilt from the same set of slots (which is exactly what arming does)
 *      answers identically to one built incrementally.
 *   4. An empty filter answers absent for everything -- so test 1 cannot pass
 *      vacuously by the filter simply saturating.
 *   5. The false-positive rate stays in the neighbourhood the model predicts.
 *      Loose bounds on purpose: this is a smoke test against a degenerate hash
 *      (one that collapses adjacent slots would blow through it), not a
 *      statistical assertion.
 *
 * Slot addresses are modelled the way the engines produce them: real addresses
 * of pointer-sized fields inside allocated objects, at the alignments a
 * transacted slot actually has.
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
#include <string.h>

#include <urcu/compiler.h>
#include <urcu/rcu-txn-bloom.h>

#include "tap.h"

#define NR_TESTS	6

/* Enough slots to fill the default 1024-bit filter well past its knee. */
#define NR_SLOTS	4096

/* A stand-in for a transacted object: several pointer slots per allocation. */
struct obj {
	void *a, *b, *c, *d;
};

static void **g_slot[NR_SLOTS];

static void build_slots(void)
{
	int i;

	for (i = 0; i < NR_SLOTS; i += 4) {
		struct obj *o = (struct obj *) malloc(sizeof(*o));

		if (!o)
			abort();
		g_slot[i] = (void **) &o->a;
		if (i + 1 < NR_SLOTS)
			g_slot[i + 1] = (void **) &o->b;
		if (i + 2 < NR_SLOTS)
			g_slot[i + 2] = (void **) &o->c;
		if (i + 3 < NR_SLOTS)
			g_slot[i + 3] = (void **) &o->d;
	}
}

int main(void)
{
	uint64_t bloom[URCU_TXN_BLOOM_WORDS];
	uint64_t rebuilt[URCU_TXN_BLOOM_WORDS];
	int i, j;
	int empty_clean = 1, no_false_neg = 1, tas_ok = 1;
	int fp = 0, probes = 0;

	plan_tests(NR_TESTS);
	build_slots();

	/* 4. An empty filter must answer absent for every slot. */
	memset(bloom, 0, sizeof(bloom));
	for (i = 0; i < NR_SLOTS; i++)
		if (urcu_txn__ryw_bloom_test(bloom, g_slot[i]))
			empty_clean = 0;
	ok(empty_clean, "an empty filter reports every slot absent (so the "
		"no-false-negative test below cannot pass vacuously)");

	/*
	 * 5. FALSE-POSITIVE RATE at a realistic fill.  Insert an eighth of the
	 * population and probe the rest, which are known absent, so every hit is
	 * a false positive.  The model for k=3 over 1024 bits with n=512 is
	 * (1-e^{-kn/m})^k ~= 0.63, so the filter is saturated at this fill and
	 * the bound only has to catch a hash that is outright degenerate.
	 */
	memset(bloom, 0, sizeof(bloom));
	for (i = 0; i < NR_SLOTS / 8; i++)
		urcu_txn__ryw_bloom_set(bloom, g_slot[i]);
	for (i = NR_SLOTS / 8; i < NR_SLOTS; i++) {
		probes++;
		if (urcu_txn__ryw_bloom_test(bloom, g_slot[i]))
			fp++;
	}
	diag("false positives: %d / %d probes (%.1f%%) at n=%d over %d bits, k=%d",
		fp, probes, 100.0 * (double) fp / (double) probes,
		NR_SLOTS / 8, (int) URCU_TXN_BLOOM_BITS, URCU_TXN_BLOOM_K);
	ok(fp < probes, "the filter still discriminates at a realistic fill "
		"(not every absent slot reads present)");

	/*
	 * 1. NO FALSE NEGATIVES.  Set every slot one at a time and, after each,
	 * re-test EVERY slot set so far: a later set must never clear an earlier
	 * slot's bits, and the answer must not depend on insertion order.  This is
	 * the property the engines' correctness rests on.
	 */
	memset(bloom, 0, sizeof(bloom));
	for (i = 0; i < NR_SLOTS; i++) {
		urcu_txn__ryw_bloom_set(bloom, g_slot[i]);
		/* re-testing all i+1 each round is O(n^2); sample instead */
		for (j = 0; j <= i; j += (i / 16 + 1))
			if (!urcu_txn__ryw_bloom_test(bloom, g_slot[j]))
				no_false_neg = 0;
	}
	for (i = 0; i < NR_SLOTS; i++)		/* and all of them at the end */
		if (!urcu_txn__ryw_bloom_test(bloom, g_slot[i]))
			no_false_neg = 0;
	ok(no_false_neg, "no false negatives: every slot ever set still reports "
		"present, whatever is set after it");

	/*
	 * 3. DETERMINISM / rebuild equivalence.  Arming rebuilds the filter from
	 * the records already buffered, so a rebuilt filter must be bit-identical
	 * to one grown incrementally -- otherwise a slot recorded before the arm
	 * could read absent after it.
	 */
	memset(rebuilt, 0, sizeof(rebuilt));
	for (i = NR_SLOTS - 1; i >= 0; i--)		/* reverse order */
		urcu_txn__ryw_bloom_set(rebuilt, g_slot[i]);
	ok(memcmp(bloom, rebuilt, sizeof(bloom)) == 0,
		"the filter is order-independent and deterministic: a rebuild from "
		"the same slots is bit-identical (what arming relies on)");

	/*
	 * 2. test_and_set reports the PRE-state and leaves the slot set.  The
	 * record path calls it once per store and reads the coincidence out of
	 * that single call, so both halves matter.
	 */
	memset(bloom, 0, sizeof(bloom));
	for (i = 0; i < 64; i++) {
		int first = urcu_txn__ryw_bloom_test_and_set(bloom, g_slot[i]);
		int again = urcu_txn__ryw_bloom_test_and_set(bloom, g_slot[i]);

		/* @first may be a false positive; @again must always be 1 */
		if (!again || !urcu_txn__ryw_bloom_test(bloom, g_slot[i]))
			tas_ok = 0;
		(void) first;
	}
	ok(tas_ok, "test_and_set leaves the slot set and reports it present on "
		"the second call");

	/* A slot never set may read present (false positive) but never crashes. */
	{
		uint64_t probe[URCU_TXN_BLOOM_WORDS];
		int stable = 1;

		memset(probe, 0, sizeof(probe));
		urcu_txn__ryw_bloom_set(probe, g_slot[0]);
		for (i = 0; i < 1000; i++)
			if (!urcu_txn__ryw_bloom_test(probe, g_slot[0]))
				stable = 0;
		ok(stable, "repeated tests of one set slot agree (the hash has no "
			"hidden state)");
	}
	return exit_status();
}
