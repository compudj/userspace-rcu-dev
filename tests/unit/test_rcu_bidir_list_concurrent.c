// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

/*
 * Concurrent stress test for <urcu/rcu-bidir-list.h>: one writer mutating a
 * key-sorted list while several readers iterate it forward AND backward.
 *
 * Invariant exploited (timing-independent): the writer keeps the list sorted
 * by key at all times, and every mutation is a single atomic two-edge flip,
 * so at every instant the whole ring is sorted.  Therefore any edge a reader
 * crosses is monotone:
 *   - a forward walk must see strictly INCREASING keys;
 *   - a reverse walk must see strictly DECREASING keys.
 * A coherence defect -- an unresolved/torn proxy, next and prev disagreeing,
 * a half-published insert, a use-after-free of a removed node -- would surface
 * as a non-monotone step, an out-of-range key, a runaway walk, or a crash
 * (run under ASan/TSan to make the last two loud).  This is exactly the
 * guarantee classic rculist's reverse walk cannot provide.
 *
 * QSBR flavor: read-side critical sections bracket each walk, and every
 * thread announces a quiescent state between critical sections so grace
 * periods (hence the deferred frees) can advance.
 */

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <inttypes.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <urcu/compiler.h>
#include <urcu-qsbr.h>
#include <urcu-call-rcu.h>
#include <urcu/rcu-bidir-list.h>

#include "tap.h"

#define NR_TESTS	2
#define KEY_MAX		100		/* key space 1..KEY_MAX */
#define NR_READERS	4
#define WRITER_OPS	300000
#define STEP_LIMIT	(KEY_MAX + 8)	/* runaway-walk guard */

struct bl_node {
	struct cds_bidir_list_head node;
	struct rcu_head rcu_head;
	int key;
};

static struct cds_bidir_list_head g_head = CDS_BIDIR_LIST_HEAD_INIT(g_head);
static int g_stop;

static void bl_node_free(struct rcu_head *head)
{
	free(caa_container_of(head, struct bl_node, rcu_head));
}

static struct bl_node *bl_node_new(int key)
{
	struct bl_node *n = (struct bl_node *) malloc(sizeof(*n));

	if (!n)
		abort();
	n->key = key;
	return n;
}

struct reader_stats {
	long walks;
	long violations;
};

static void *reader_fn(void *arg)
{
	struct reader_stats *st = (struct reader_stats *) arg;

	rcu_register_thread();
	while (!uatomic_load(&g_stop, CMM_RELAXED)) {
		struct cds_bidir_list_head *p;
		int prev_key, steps;

		/* Forward: strictly increasing keys, bounded, ends at head. */
		rcu_read_lock();
		prev_key = 0;
		steps = 0;
		for (p = cds_bidir_list_next_rcu(&g_head); p != &g_head;
				p = cds_bidir_list_next_rcu(p)) {
			int k = caa_container_of(p, struct bl_node, node)->key;

			if (k < 1 || k > KEY_MAX || k <= prev_key ||
					++steps > STEP_LIMIT) {
				st->violations++;
				break;
			}
			prev_key = k;
		}
		rcu_read_unlock();

		/* Reverse: strictly decreasing keys, bounded, ends at head. */
		rcu_read_lock();
		prev_key = KEY_MAX + 1;
		steps = 0;
		for (p = cds_bidir_list_prev_rcu(&g_head); p != &g_head;
				p = cds_bidir_list_prev_rcu(p)) {
			int k = caa_container_of(p, struct bl_node, node)->key;

			if (k < 1 || k > KEY_MAX || k >= prev_key ||
					++steps > STEP_LIMIT) {
				st->violations++;
				break;
			}
			prev_key = k;
		}
		rcu_read_unlock();

		st->walks += 2;
		rcu_quiescent_state();		/* let grace periods advance */
	}
	rcu_unregister_thread();
	return NULL;
}

/* Insert @n keeping the list sorted; @slot tracks the live node per key. */
static void sorted_insert(struct bl_node *n, struct bl_node **slot)
{
	struct cds_bidir_list_head *succ = &g_head;	/* default: tail */
	int j;

	for (j = n->key + 1; j <= KEY_MAX; j++) {
		if (slot[j]) {
			succ = &slot[j]->node;
			break;
		}
	}
	if (cds_bidir_list_add_before_rcu(&n->node, succ))
		abort();
	slot[n->key] = n;
}

int main(void)
{
	pthread_t readers[NR_READERS];
	struct reader_stats stats[NR_READERS];
	struct bl_node *slot[KEY_MAX + 1];
	long total_walks = 0, total_violations = 0;
	unsigned int rng = 2463534242u;
	int i, k;

	plan_tests(NR_TESTS);
	rcu_register_thread();

	memset(slot, 0, sizeof(slot));
	memset(stats, 0, sizeof(stats));

	for (i = 0; i < NR_READERS; i++)
		pthread_create(&readers[i], NULL, reader_fn, &stats[i]);

	for (i = 0; i < WRITER_OPS; i++) {
		/* xorshift32 for reproducible pseudo-randomness */
		rng ^= rng << 13;
		rng ^= rng >> 17;
		rng ^= rng << 5;
		k = 1 + (int) (rng % KEY_MAX);

		if (!slot[k]) {
			sorted_insert(bl_node_new(k), slot);
		} else if ((rng & 0x3) == 0) {
			struct bl_node *old = slot[k];
			struct bl_node *fresh = bl_node_new(k);

			/* replace keeps the same key, so order is preserved */
			if (cds_bidir_list_replace_rcu(&old->node, &fresh->node))
				abort();
			call_rcu(&old->rcu_head, bl_node_free);
			slot[k] = fresh;
		} else {
			struct bl_node *old = slot[k];

			if (cds_bidir_list_del_rcu(&old->node))
				abort();
			call_rcu(&old->rcu_head, bl_node_free);
			slot[k] = NULL;
		}
		if ((i & 0xff) == 0)
			rcu_quiescent_state();	/* drain deferred frees */
	}

	uatomic_store(&g_stop, 1, CMM_RELAXED);
	for (i = 0; i < NR_READERS; i++)
		pthread_join(readers[i], NULL);

	/* Drain the remaining live nodes. */
	for (k = 1; k <= KEY_MAX; k++) {
		if (slot[k]) {
			if (cds_bidir_list_del_rcu(&slot[k]->node))
				abort();
			call_rcu(&slot[k]->rcu_head, bl_node_free);
			slot[k] = NULL;
		}
	}

	for (i = 0; i < NR_READERS; i++) {
		total_walks += stats[i].walks;
		total_violations += stats[i].violations;
	}
	diag("%d readers performed %ld directional walks, %ld violations",
		NR_READERS, total_walks, total_violations);

	ok(total_walks > 0, "readers iterated concurrently with the writer");
	ok(total_violations == 0,
		"forward/reverse walks stayed monotone (coherent both directions)");

	rcu_barrier();
	rcu_unregister_thread();
	return exit_status();
}
