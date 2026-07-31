// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

/*
 * Test for <urcu/rcu-txn-hlist.h>: the single-pointer-head (8 B), pprev-encoded
 * concurrent-writer RCU hlist.
 *
 * Part 1 -- deterministic single-threaded edge cases.  Builds a bucket with
 * every insert shape (1-edge insert into an empty bucket, insert-at-head,
 * insert-after, insert-before, replace) and then tears it down by NODE from
 * every position (head, interior, last=2-edge, sole=1-edge), walking and
 * comparing the whole chain after each op.  Delete-by-node needs no bucket
 * argument -- it follows pprev -- so a correct pprev from any position is
 * exactly what this exercises: a botched pprev would unlink the wrong slot and
 * the next walk would diverge from the expected chain.
 *
 * Part 2 -- concurrent stress.  A small hash table of buckets sharing ONE
 * escalation domain; many writers do sorted insert / delete-by-key spread across
 * buckets while readers walk each bucket forward.  Invariant
 * (timing-independent): every writer keeps each bucket non-decreasing and every
 * mutation is one atomic MCAS, so at every instant each chain is sorted
 * (duplicates allowed).  A forward walk must see NON-DECREASING keys; a torn
 * multi-edge commit, an incoherent pprev, a botched mark, or a use-after-free
 * surfaces as a non-monotone step, an out-of-range key, a runaway walk, or a
 * crash (run under ASan to make the last two loud).  Progress: every writer
 * completes its op count (a livelock would hang the test).
 *
 * QSBR flavor: updaters and readers run in RCU read-side sections; the main
 * thread goes offline before joining so it does not stall grace periods.
 */

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#ifndef _LGPL_SOURCE
#define _LGPL_SOURCE	/* inline rcu_dereference: uatomic_load(CMM_CONSUME) */
#endif

#include <inttypes.h>
#include <limits.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include <urcu/compiler.h>
#include <urcu-qsbr.h>
#include <urcu-call-rcu.h>
#include <urcu/rcu-txn.h>
#include <urcu/rcu-txn-hlist.h>

#include "tap.h"

#define NR_TESTS	4
#ifndef KEY_MAX
#define KEY_MAX		100
#endif
#ifndef NR_BUCKETS
#define NR_BUCKETS	16
#endif
#ifndef NR_WRITERS
#define NR_WRITERS	6
#endif
#ifndef NR_READERS
#define NR_READERS	2
#endif
#ifndef WRITER_OPS
#define WRITER_OPS	50000
#endif
#define STEP_LIMIT	16384		/* runaway-walk guard (>> equilibrium size) */

struct hnode {
	struct urcu_txn_hlist_node node;
	struct rcu_head rh;
	int key;
};

static struct urcu_txn_hlist_head g_bkt[NR_BUCKETS];
static struct urcu_txn_domain g_dom;	/* one domain shared by the whole table */
static int g_stop;

static int key_of(struct urcu_txn_hlist_node *n)
{
	return caa_container_of(n, struct hnode, node)->key;
}

static void hnode_free(struct rcu_head *h)
{
	free(caa_container_of(h, struct hnode, rh));
}

static struct hnode *hnode_alloc(int key)
{
	struct hnode *n = (struct hnode *) malloc(sizeof(*n));

	if (!n)
		abort();
	n->key = key;
	n->node.next = NULL;
	n->node.pprev = NULL;
	return n;
}

static unsigned int xs(unsigned int x)
{
	x ^= x << 13;
	x ^= x >> 17;
	x ^= x << 5;
	return x;
}

static inline unsigned int hash_key(int key)
{
	return (unsigned int) key % NR_BUCKETS;
}

/* Collect a bucket's keys into @out (caller holds rcu_read_lock); returns count. */
static int collect(struct urcu_txn_hlist_head *bkt, int *out, int max)
{
	struct urcu_txn_hlist_node *p;
	int n = 0;

	urcu_txn_hlist_for_each_rcu(p, bkt) {
		if (n >= max)
			break;
		out[n++] = key_of(p);
	}
	return n;
}

/* Assert the bucket's chain equals @exp[0..len).  Returns 1 on match. */
static int expect_chain(struct urcu_txn_hlist_head *bkt, const int *exp, int len)
{
	int got[STEP_LIMIT], n, i;

	rcu_read_lock();
	n = collect(bkt, got, STEP_LIMIT);
	rcu_read_unlock();
	if (n != len)
		return 0;
	for (i = 0; i < n; i++)
		if (got[i] != exp[i])
			return 0;
	return 1;
}

/*
 * Part 1: deterministic single-threaded edge-case coverage.  Returns 1 if every
 * intermediate chain matched its expected contents.
 */
static int deterministic_edge_cases(void)
{
	struct urcu_txn_hlist_head bkt;
	struct hnode *n3, *n4, *n5, *n6, *n6b, *n7;
	int ok_all = 1;

	urcu_txn_hlist_init(&bkt);

	/*
	 * Every mutator return is checked.  Discarding them makes the test itself
	 * unsound: under descriptor OOM a replace_rcu() fails, the old node stays
	 * linked, and the call_rcu free below it is a use-after-free -- which
	 * would then surface as a baffling chain mismatch rather than as the OOM
	 * that caused it.
	 */
#define MUST_OK(call)	do { if ((call) < 0) abort(); } while (0)

	/* 1-edge insert into an empty bucket. */
	n5 = hnode_alloc(5);
	rcu_read_lock();
	MUST_OK(urcu_txn_hlist_add_rcu(&n5->node, &bkt, &g_dom));
	rcu_read_unlock();
	{ int e[] = { 5 }; ok_all &= expect_chain(&bkt, e, 1); }

	/* insert-at-head with a smaller key (2-edge: head slot + old-first pprev). */
	n3 = hnode_alloc(3);
	rcu_read_lock();
	MUST_OK(urcu_txn_hlist_add_rcu(&n3->node, &bkt, &g_dom));
	rcu_read_unlock();
	{ int e[] = { 3, 5 }; ok_all &= expect_chain(&bkt, e, 2); }

	/* insert-after an interior node. */
	n4 = hnode_alloc(4);
	rcu_read_lock();
	MUST_OK(urcu_txn_hlist_insert_after_rcu(&n4->node, &n3->node, &g_dom));
	rcu_read_unlock();
	{ int e[] = { 3, 4, 5 }; ok_all &= expect_chain(&bkt, e, 3); }

	/* insert-after the last node (its succ is NULL: newp->next becomes NULL). */
	n7 = hnode_alloc(7);
	rcu_read_lock();
	MUST_OK(urcu_txn_hlist_insert_after_rcu(&n7->node, &n5->node, &g_dom));
	rcu_read_unlock();
	{ int e[] = { 3, 4, 5, 7 }; ok_all &= expect_chain(&bkt, e, 4); }

	/* insert-before a node (re-points *pos->pprev). */
	n6 = hnode_alloc(6);
	rcu_read_lock();
	MUST_OK(urcu_txn_hlist_insert_before_rcu(&n6->node, &n7->node, &g_dom));
	rcu_read_unlock();
	{ int e[] = { 3, 4, 5, 6, 7 }; ok_all &= expect_chain(&bkt, e, 5); }

	/* replace an interior node in place. */
	n6b = hnode_alloc(6);
	rcu_read_lock();
	MUST_OK(urcu_txn_hlist_replace_rcu(&n6->node, &n6b->node, &g_dom));
	rcu_read_unlock();
	call_rcu(&n6->rh, hnode_free);	/* safe: the replace above succeeded */
	{ int e[] = { 3, 4, 5, 6, 7 }; ok_all &= expect_chain(&bkt, e, 5); }

	/*
	 * Tear down BY NODE from every position; each del follows pprev with no
	 * bucket argument.  A wrong pprev would unlink the wrong slot -> mismatch.
	 */
	rcu_read_lock();				/* interior */
	(void) urcu_txn_hlist_del_rcu(&n4->node, &g_dom);
	rcu_read_unlock();
	call_rcu(&n4->rh, hnode_free);
	{ int e[] = { 3, 5, 6, 7 }; ok_all &= expect_chain(&bkt, e, 4); }

	rcu_read_lock();				/* head (pprev == &bkt.first) */
	(void) urcu_txn_hlist_del_rcu(&n3->node, &g_dom);
	rcu_read_unlock();
	call_rcu(&n3->rh, hnode_free);
	{ int e[] = { 5, 6, 7 }; ok_all &= expect_chain(&bkt, e, 3); }

	rcu_read_lock();				/* last (2-edge: next == NULL) */
	(void) urcu_txn_hlist_del_rcu(&n7->node, &g_dom);
	rcu_read_unlock();
	call_rcu(&n7->rh, hnode_free);
	{ int e[] = { 5, 6 }; ok_all &= expect_chain(&bkt, e, 2); }

	rcu_read_lock();
	(void) urcu_txn_hlist_del_rcu(&n6b->node, &g_dom);
	rcu_read_unlock();
	call_rcu(&n6b->rh, hnode_free);
	{ int e[] = { 5 }; ok_all &= expect_chain(&bkt, e, 1); }

	rcu_read_lock();				/* sole element (1-edge delete) */
	(void) urcu_txn_hlist_del_rcu(&n5->node, &g_dom);
	rcu_read_unlock();
	call_rcu(&n5->rh, hnode_free);
	ok_all &= urcu_txn_hlist_empty(&bkt);
	ok_all &= expect_chain(&bkt, NULL, 0);

	return ok_all;
}

/*
 * Insert @key into its bucket keeping the chain non-decreasing (duplicates ok).
 *
 * This walks to the insertion point and pins the EXACT successor it validated
 * via insert_at_slot_prepare (slot -> succ old-value): a sorted chain must anchor
 * the specific successor whose key it checked, not "wherever @slot points now".
 * insert_after_prepare/insert_head_prepare re-read the slot fresh -- correct for
 * a positional insert, but for a sorted one a racing insert of a smaller key
 * between the walk and the commit would move the slot to a successor whose key is
 * <= @key, splicing @key out of order.  The *slot: succ -> newp old-value check
 * aborts exactly that race (mirrors the bidir-list test's succ pin).
 */
static void sorted_insert(int key)
{
	struct urcu_txn_hlist_head *bkt = &g_bkt[hash_key(key)];
	struct hnode *n = hnode_alloc(key);
	struct urcu_txn txn;
	enum urcu_txn_status st;

	urcu_txn_init(&txn, &g_dom);
	for (;;) {
		/* @slot names the insertion point; @succ is the first node with
		 * key > @key (or NULL), captured and then pinned at commit. */
		struct urcu_txn_hlist_node **slot = &bkt->first;
		struct urcu_txn_hlist_node *succ, *pred = NULL;
		void *raw;
		int prep, restart = 0;

		urcu_txn_begin(&txn);
		raw = urcu_txn_load(&txn, (void **) slot, URCU_TXN_HLIST_TAG);
		succ = urcu_txn_hlist_unmark(raw);	/* head->first: never marked */
		while (succ != NULL && key_of(succ) <= key) {
			pred = succ;
			slot = &pred->next;
			raw = urcu_txn_load(&txn, (void **) slot, URCU_TXN_HLIST_TAG);
			if (urcu_txn_hlist_is_marked(raw)) {
				restart = 1;		/* pred being deleted: re-walk */
				break;
			}
			succ = (struct urcu_txn_hlist_node *) raw;
		}
		if (restart) {
			urcu_txn_conflict(&txn);
			urcu_txn_end(&txn);
			continue;
		}
		prep = urcu_txn_hlist_insert_at_slot_prepare(&txn, &n->node,
				slot, succ);
		if (prep) {				/* -EAGAIN: succ mid-delete */
			urcu_txn_conflict(&txn);
			urcu_txn_end(&txn);
			continue;
		}
		st = urcu_txn_commit(&txn);
		urcu_txn_end(&txn);
		if (st == URCU_TXN_STATUS_OK)
			return;
		if (st == URCU_TXN_STATUS_ABORT)
			continue;			/* slot moved: re-find and retry */
		abort();				/* MEMORY_ERROR */
	}
}

/* Delete the first node with key == @key in its bucket, if present. */
static void delete_key(int key)
{
	struct urcu_txn_hlist_head *bkt = &g_bkt[hash_key(key)];
	struct urcu_txn_hlist_node *p, *target = NULL;

	rcu_read_lock();
	urcu_txn_hlist_for_each_rcu(p, bkt) {
		int k = key_of(p);

		if (k == key) {
			target = p;
			break;
		}
		if (k > key)
			break;				/* sorted: past @key, absent */
	}
	if (target) {
		int r = urcu_txn_hlist_del_rcu(target, &g_dom);

		rcu_read_unlock();
		if (r == 1)				/* this call removed it -> reclaim */
			call_rcu(&caa_container_of(target, struct hnode, node)->rh,
					hnode_free);
	} else {
		rcu_read_unlock();
	}
}

struct writer_stats {
	unsigned int seed;
	long ops;
};

static void *writer(void *arg)
{
	struct writer_stats *st = (struct writer_stats *) arg;
	unsigned int rng = st->seed;
	long n;

	rcu_register_thread();
	for (n = 0; n < WRITER_OPS; n++) {
		int key;

		rng = xs(rng);
		key = 1 + (int) ((rng >> 1) % KEY_MAX);
		if (rng & 1)
			sorted_insert(key);
		else
			delete_key(key);
		st->ops++;
		rcu_quiescent_state();
	}
	rcu_unregister_thread();
	return NULL;
}

struct reader_stats {
	long walks;
	long violations;
};

static void *reader(void *arg)
{
	struct reader_stats *st = (struct reader_stats *) arg;

	rcu_register_thread();
	while (!uatomic_load(&g_stop, CMM_RELAXED)) {
		struct urcu_txn_hlist_node *p;
		int b;

		for (b = 0; b < NR_BUCKETS; b++) {
			int prev_key = INT_MIN, steps = 0;

			rcu_read_lock();
			urcu_txn_hlist_for_each_rcu(p, &g_bkt[b]) {
				int k = key_of(p);

				if (k < prev_key || k < 1 || k > KEY_MAX ||
						++steps > STEP_LIMIT) {
					st->violations++;
					break;
				}
				prev_key = k;
			}
			rcu_read_unlock();
			st->walks++;
		}
		rcu_quiescent_state();
	}
	rcu_unregister_thread();
	return NULL;
}

/* Quiescent check: every bucket is non-decreasing and within range. */
static int check_all_sorted(void)
{
	int b;

	for (b = 0; b < NR_BUCKETS; b++) {
		struct urcu_txn_hlist_node *p;
		int prev_key = INT_MIN, steps = 0;

		for (p = urcu_txn_hlist_first_rcu(&g_bkt[b]); p != NULL;
				p = urcu_txn_hlist_next_rcu(p)) {
			int k = key_of(p);

			if (k < prev_key || k < 1 || k > KEY_MAX ||
					++steps > STEP_LIMIT)
				return 0;
			prev_key = k;
		}
	}
	return 1;
}

int main(void)
{
	pthread_t wt[NR_WRITERS], rt[NR_READERS];
	struct writer_stats ws[NR_WRITERS];
	struct reader_stats rs[NR_READERS];
	long total_ops = 0, total_walks = 0, total_viol = 0;
	int det_ok, i, b;

	plan_tests(NR_TESTS);
	rcu_register_thread();
	urcu_txn_domain_init(&g_dom);

	/* Part 1: deterministic edge cases (needs the domain; runs single-threaded). */
	det_ok = deterministic_edge_cases();
	ok(det_ok,
		"deterministic edge cases: all insert/replace/delete-by-node shapes "
		"(incl. head/last/sole) keep the chain coherent");
	rcu_barrier();				/* drain Part 1 reclaim */

	/* Part 2: concurrent stress over a shared-domain hash table. */
	for (b = 0; b < NR_BUCKETS; b++)
		urcu_txn_hlist_init(&g_bkt[b]);

	for (i = 0; i < NR_READERS; i++) {
		rs[i].walks = 0;
		rs[i].violations = 0;
		pthread_create(&rt[i], NULL, reader, &rs[i]);
	}
	for (i = 0; i < NR_WRITERS; i++) {
		ws[i].seed = 0x1234567u + (unsigned int) i * 2654435761u;
		ws[i].ops = 0;
		pthread_create(&wt[i], NULL, writer, &ws[i]);
	}

	rcu_thread_offline();			/* don't stall grace periods while joined */
	for (i = 0; i < NR_WRITERS; i++) {
		pthread_join(wt[i], NULL);
		total_ops += ws[i].ops;
	}
	uatomic_store(&g_stop, 1, CMM_RELAXED);
	for (i = 0; i < NR_READERS; i++) {
		pthread_join(rt[i], NULL);
		total_walks += rs[i].walks;
		total_viol += rs[i].violations;
	}
	rcu_thread_online();

	diag("%d writers did %ld ops; %d readers did %ld bucket-walks, %ld violations",
		NR_WRITERS, total_ops, NR_READERS, total_walks, total_viol);

	ok(total_ops == (long) NR_WRITERS * WRITER_OPS,
		"every writer completed (bounded-blocking progress)");
	ok(total_viol == 0,
		"forward walks stayed monotone (coherent chains, valid pprev/mark)");
	ok(check_all_sorted(),
		"at quiescence every bucket is non-decreasing and in range");

	/* Drain remaining nodes. */
	for (b = 0; b < NR_BUCKETS; b++) {
		struct urcu_txn_hlist_node *p;

		rcu_read_lock();
		p = urcu_txn_hlist_first_rcu(&g_bkt[b]);
		while (p != NULL) {
			struct hnode *n = caa_container_of(p, struct hnode, node);
			struct urcu_txn_hlist_node *nextp = urcu_txn_hlist_next_rcu(p);

			if (urcu_txn_hlist_del_rcu(&n->node, &g_dom) == 1)
				call_rcu(&n->rh, hnode_free);
			p = nextp;
		}
		rcu_read_unlock();
	}

	rcu_barrier();
	rcu_unregister_thread();
	return exit_status();
}
