// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

/*
 * Regression test for THE READ POLICY of the RCU MCAS transaction front-end
 * (<urcu/rcu-txn.h>): help iff the loaded slot ends up in the transaction's
 * read/write set; read optimistically ONLY to navigate.
 *
 * Why this test exists.  An optimistic read of a slot the transaction then
 * stores (or load-validates) resolves an undecided parker to its logical OLD --
 * the value the slot takes if the parker aborts -- so whenever the parker
 * instead commits, the record's old_ptr is stale by construction and the plant
 * CAS is doomed: the attempt is forced to abort.  The damage is a throughput
 * loss under contention, NOT a correctness fault, so it is INVISIBLE to the
 * functional suites: making rcu-txn-hlist.h's _prepare loads optimistic passed
 * test_rcu_txn_hlist and reported "aborts: 0" at the 4096-bucket benchmark
 * config, while costing 30% (abort:commit 0.61 -> 0.86) at 64 buckets / 192
 * writers.  Nothing in the tree caught it.  This does.
 *
 * The policy is a property of the CALL SEQUENCE, not of contention, so it is
 * checkable single-threaded: for every slot entering the read/write set, the
 * most recent load of that slot in the same attempt must have HELPED.  Built
 * with -DURCU_TXN_DEBUG_READ_POLICY the engine enforces it (a violation aborts
 * the process); adding -DURCU_TXN_DEBUG_READ_POLICY_SOFT counts violations in
 * the handle instead.  This file compiles itself in SOFT mode (see its
 * *_CPPFLAGS in Makefile.am) and asserts on urcu_txn_read_policy_violations(),
 * so the counter is meaningful whatever the top-level build selects.
 *
 * Two parts:
 *   A. THE CHECKER (the spec, with a NEGATIVE CONTROL).  Drive the raw load /
 *      store / load-validate primitives on a plain slot and assert the exact
 *      violation count for each combination -- including the case that MUST fire
 *      (optimistic-then-store), without which a checker that never fires would
 *      pass vacuously.  Also proves the per-attempt table resets at begin().
 *   B. THE STRUCTURES OBEY IT.  Drive the real rcu-txn-hlist and rcu-txn-skiplist
 *      *_prepare paths -- the composable forms that hold the guarded loads -- on
 *      one handle, and assert zero cumulative violations (and zero probe-table
 *      evictions, so the zero is a proof and not a dropped mark).  Flip any
 *      _prepare load to urcu_txn_load_optimistic and part B goes red.
 *
 * QSBR flavor; the writer registers as an RCU thread so committed descriptors
 * stay alive across the (single-threaded here, but real) reclaim path.
 */

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#ifndef _LGPL_SOURCE
#define _LGPL_SOURCE
#endif

#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include <urcu/compiler.h>
#include <urcu-qsbr.h>
#include <urcu-call-rcu.h>

#include <urcu/rcu-txn.h>
#include <urcu/rcu-txn-hlist.h>
#include <urcu/rcu-txn-skiplist.h>

#include "tap.h"

#define NR_TESTS	9

#ifndef URCU_TXN_DEBUG_READ_POLICY
# error "test_rcu_txn_read_policy must be built with -DURCU_TXN_DEBUG_READ_POLICY"
#endif
#ifndef URCU_TXN_DEBUG_READ_POLICY_SOFT
# error "build with -DURCU_TXN_DEBUG_READ_POLICY_SOFT so a violation is counted, not fatal"
#endif

#define TAG	URCU_TXN_TAG

/*
 * Two tag-clear (bit 0 == 0), non-proxy pointer values a plain transacted slot
 * can legally hold.  long is 8-aligned, so bit 0 is clear -- a char would NOT be
 * (byte-aligned), and the engine would read its address as a proxy record.
 */
static long obj_a, obj_b;
#define A	((void *) &obj_a)
#define B	((void *) &obj_b)

/* Commit @txn and end the bracket; single-threaded, so it must succeed at once. */
static void commit_ok(struct urcu_txn *txn)
{
	enum urcu_txn_status st = urcu_txn_commit(txn);

	urcu_txn_end(txn);
	assert(st == URCU_TXN_STATUS_OK);
}

/* ---- Part A: the checker, with a negative control ------------------------- */

static void test_checker(void)
{
	struct urcu_txn txn;
	void *slot;
	void *v;

	/* 1. CORRECT: a helping load then a store to that slot -- 0 violations. */
	slot = A;
	urcu_txn_init(&txn, NULL);
	urcu_txn_begin(&txn);
	v = urcu_txn_load(&txn, &slot, TAG);
	urcu_txn_store_mw(&txn, &slot, v, B, TAG);
	commit_ok(&txn);
	ok(urcu_txn_read_policy_violations(&txn) == 0 &&
	   urcu_txn_read_policy_evicted(&txn) == 0,
	   "helping load then store to that slot: no violation");

	/*
	 * 2. NEGATIVE CONTROL: an OPTIMISTIC load then a store to that slot.  This
	 * is the doomed-plant pattern; the checker MUST flag exactly one.  (It
	 * commits here only because nothing else parks the slot single-threaded --
	 * which is precisely the point: the bug hides until there IS a parker.)
	 */
	slot = A;
	urcu_txn_init(&txn, NULL);
	urcu_txn_begin(&txn);
	v = urcu_txn_load_optimistic(&txn, &slot, TAG);
	urcu_txn_store_mw(&txn, &slot, v, B, TAG);
	commit_ok(&txn);
	ok(urcu_txn_read_policy_violations(&txn) == 1 &&
	   urcu_txn_read_policy_evicted(&txn) == 0,
	   "optimistic load then store to that slot: exactly one violation (the checker fires)");

	/* 3. NAVIGATION: an optimistic load with NO store to it -- allowed, 0. */
	slot = A;
	urcu_txn_init(&txn, NULL);
	urcu_txn_begin(&txn);
	(void) urcu_txn_load_optimistic(&txn, &slot, TAG);	/* just to navigate */
	commit_ok(&txn);					/* empty write set */
	ok(urcu_txn_read_policy_violations(&txn) == 0,
	   "optimistic load used only to navigate (never stored): no violation");

	/* 4. GUARD: a helping load-validate (read-set entry) -- 0. */
	slot = A;
	urcu_txn_init(&txn, NULL);
	urcu_txn_begin(&txn);
	(void) urcu_txn_load_validate(&txn, &slot, TAG);
	commit_ok(&txn);
	ok(urcu_txn_read_policy_violations(&txn) == 0,
	   "helping load-validate (a pinned read): no violation");

	/*
	 * 5. GUARD, WRONG: load_validate_optimistic reads a read-set slot without
	 * helping, so it violates BY CONSTRUCTION -- which is why it has no callers.
	 */
	slot = A;
	urcu_txn_init(&txn, NULL);
	urcu_txn_begin(&txn);
	(void) urcu_txn_load_validate_optimistic(&txn, &slot, TAG);
	commit_ok(&txn);
	ok(urcu_txn_read_policy_violations(&txn) == 1,
	   "load_validate_optimistic pins a slot it did not help: one violation");

	/* 6. BLIND STORE: a slot never loaded this attempt -- nothing to check, 0. */
	slot = A;
	urcu_txn_init(&txn, NULL);
	urcu_txn_begin(&txn);
	urcu_txn_store_mw(&txn, &slot, A, B, TAG);		/* caller already holds the old */
	commit_ok(&txn);
	ok(urcu_txn_read_policy_violations(&txn) == 0,
	   "blind store of a slot never loaded this attempt: no violation");

	/*
	 * 7. PER-ATTEMPT RESET.  Attempt 1 loads @slot optimistically for navigation
	 * (no store).  Attempt 2, on the SAME handle, does a blind store of the same
	 * slot.  If the probe table did not reset at begin(), attempt 2 would see
	 * attempt 1's stale optimistic mark and mis-fire; asserting 0 proves it did
	 * reset.  (Violations accumulate per transaction, so a fresh init resets the
	 * count; the reset under test is the TABLE, cleared each begin.)
	 */
	slot = A;
	urcu_txn_init(&txn, NULL);
	urcu_txn_begin(&txn);					/* attempt 1 */
	(void) urcu_txn_load_optimistic(&txn, &slot, TAG);
	commit_ok(&txn);
	urcu_txn_begin(&txn);					/* attempt 2 */
	urcu_txn_store_mw(&txn, &slot, A, B, TAG);
	commit_ok(&txn);
	ok(urcu_txn_read_policy_violations(&txn) == 0,
	   "the per-attempt probe table resets at begin(): no stale-mark misfire");
}

/* ---- Part B: the real structures obey the policy -------------------------- */

struct hnode {
	int key;
	struct urcu_txn_hlist_node node;
};

static struct hnode *hnode_alloc(int key)
{
	struct hnode *n = (struct hnode *) calloc(1, sizeof(*n));

	if (!n)
		abort();
	n->key = key;
	return n;
}

/*
 * Exercise every hlist _prepare form (head / after / before / del / replace)
 * on ONE handle, committing each, and return the cumulative violation and
 * eviction counts.  These are exactly the composable paths whose guarded loads
 * the policy protects; single-threaded so every commit lands first try.
 */
static void test_hlist_obeys(void)
{
	struct urcu_txn_hlist_head bkt;
	struct urcu_txn txn;
	struct hnode *n3, *n4, *n5, *n6, *n6b, *n7;
	int prep;

	urcu_txn_hlist_init(&bkt);
	urcu_txn_init(&txn, NULL);

	/* insert into empty bucket (head). */
	n5 = hnode_alloc(5);
	urcu_txn_begin(&txn);
	prep = urcu_txn_hlist_insert_head_prepare(&txn, &n5->node, &bkt);
	assert(prep == 0);
	commit_ok(&txn);

	/* insert-head with the old first node present (2-edge: head slot + pprev). */
	n3 = hnode_alloc(3);
	urcu_txn_begin(&txn);
	prep = urcu_txn_hlist_insert_head_prepare(&txn, &n3->node, &bkt);
	assert(prep == 0);
	commit_ok(&txn);

	/* insert-after an interior node. */
	n4 = hnode_alloc(4);
	urcu_txn_begin(&txn);
	prep = urcu_txn_hlist_insert_after_prepare(&txn, &n4->node, &n3->node);
	assert(prep == 0);
	commit_ok(&txn);

	/* insert-after the last node (its succ is NULL). */
	n7 = hnode_alloc(7);
	urcu_txn_begin(&txn);
	prep = urcu_txn_hlist_insert_after_prepare(&txn, &n7->node, &n5->node);
	assert(prep == 0);
	commit_ok(&txn);

	/* insert-before a node (re-points *pos->pprev). */
	n6 = hnode_alloc(6);
	urcu_txn_begin(&txn);
	prep = urcu_txn_hlist_insert_before_prepare(&txn, &n6->node, &n7->node);
	assert(prep == 0);
	commit_ok(&txn);

	/* replace an interior node in place. */
	n6b = hnode_alloc(6);
	urcu_txn_begin(&txn);
	prep = urcu_txn_hlist_replace_prepare(&txn, &n6->node, &n6b->node);
	assert(prep == 0);
	commit_ok(&txn);

	/* delete from interior, head, and tail positions. */
	urcu_txn_begin(&txn);
	prep = urcu_txn_hlist_del_prepare(&txn, &n4->node);
	assert(prep == 0);
	commit_ok(&txn);

	urcu_txn_begin(&txn);
	prep = urcu_txn_hlist_del_prepare(&txn, &n3->node);	/* head */
	assert(prep == 0);
	commit_ok(&txn);

	urcu_txn_begin(&txn);
	prep = urcu_txn_hlist_del_prepare(&txn, &n7->node);	/* tail */
	assert(prep == 0);
	commit_ok(&txn);

	ok(urcu_txn_read_policy_violations(&txn) == 0 &&
	   urcu_txn_read_policy_evicted(&txn) == 0,
	   "rcu-txn-hlist: every _prepare form helps on its read/write-set loads");

	free(n3); free(n4); free(n5); free(n6); free(n6b); free(n7);
}

struct snode {
	unsigned long key;
	struct urcu_txn_skiplist_node sl;	/* last: flexible next[] */
};

static int snode_cmp(struct urcu_txn_skiplist_node *n, void *key)
{
	unsigned long a = caa_container_of(n, struct snode, sl)->key;
	unsigned long b = *(unsigned long *) key;

	return (a > b) - (a < b);
}

static struct snode *snode_alloc(unsigned long key, unsigned int toplevel)
{
	struct snode *e = (struct snode *) malloc(sizeof(*e)
			+ (toplevel + 1) * sizeof(struct urcu_txn_skiplist_node *));

	if (!e)
		abort();
	e->key = key;
	urcu_txn_skiplist_node_init(&e->sl, toplevel);
	return e;
}

/*
 * Exercise skiplist insert_prepare / del_prepare (the five guarded _prepare
 * loads, plus a composed del+insert MOVE) on ONE handle and assert zero
 * cumulative violations.  A tower spans several levels, so a wrong (optimistic)
 * predecessor read on any level would register here.  Deterministic per-key
 * heights so the run is reproducible without a PRNG.
 */
static void test_skiplist_obeys(void)
{
	struct urcu_txn_skiplist a, b;
	struct urcu_txn txn;
	struct urcu_txn_skiplist_node *rem;
	unsigned long key;
	unsigned int i;
	int prep;
	enum urcu_txn_status st;
	/* a spread of tower heights so several levels get predecessor loads */
	static const unsigned int heights[] = { 0, 3, 1, 5, 2, 7, 0, 4, 1, 6 };

	assert(!urcu_txn_skiplist_init(&a, snode_cmp));
	assert(!urcu_txn_skiplist_init(&b, snode_cmp));
	urcu_txn_init(&txn, NULL);

	/* populate skiplist A with ten keys of assorted heights. */
	for (i = 0; i < 10; i++) {
		struct snode *n = snode_alloc(i, heights[i]);

		key = i;
		urcu_txn_begin(&txn);
		prep = urcu_txn_skiplist_insert_prepare(&txn, &a, &n->sl, &key);
		assert(prep == 0);
		commit_ok(&txn);
	}

	/* delete a couple outright. */
	for (i = 0; i < 2; i++) {
		key = i * 4;			/* keys 0 and 4 */
		urcu_txn_begin(&txn);
		prep = urcu_txn_skiplist_del_prepare(&txn, &a, &key, &rem);
		assert(prep == 0 && rem != NULL);
		commit_ok(&txn);
	}

	/* MOVE the rest from A to B: one composed del+insert txn each. */
	for (i = 1; i < 10; i++) {
		struct snode *nn;

		if (i == 4)
			continue;		/* already deleted */
		key = i;
		nn = snode_alloc(i, heights[i]);
		urcu_txn_begin(&txn);
		prep = urcu_txn_skiplist_del_prepare(&txn, &a, &key, &rem);
		assert(prep == 0 && rem != NULL);
		prep = urcu_txn_skiplist_insert_prepare(&txn, &b, &nn->sl, &key);
		assert(prep == 0);
		st = urcu_txn_commit(&txn);
		urcu_txn_end(&txn);
		assert(st == URCU_TXN_STATUS_OK);
	}

	ok(urcu_txn_read_policy_violations(&txn) == 0 &&
	   urcu_txn_read_policy_evicted(&txn) == 0,
	   "rcu-txn-skiplist: insert/del/move _prepare help on every predecessor load");

	urcu_txn_skiplist_destroy(&a);
	urcu_txn_skiplist_destroy(&b);
}

int main(void)
{
	rcu_register_thread();

	plan_tests(NR_TESTS);

	test_checker();			/* 7 */
	test_hlist_obeys();		/* 1 */
	test_skiplist_obeys();		/* 1 */

	rcu_barrier();
	rcu_unregister_thread();
	return exit_status();
}
