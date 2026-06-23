// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

/*
 * Concurrent stress test for <urcu/rcu-bidir-list-lockfree.h>: many LOCK-FREE
 * writers doing arbitrary-position sorted insert and delete-by-key, while reader
 * threads walk the list forward AND backward.
 *
 * Invariant (timing-independent): every writer keeps the list sorted by key, and
 * every mutation is one atomic MCAS, so at every instant the whole ring is
 * sorted (duplicates allowed).  Hence a forward walk must see NON-DECREASING keys
 * and a reverse walk NON-INCREASING keys.  A torn multi-edge commit, an
 * incoherent next/prev, a botched logical-deletion mark, or a use-after-free
 * would surface as a non-monotone step, an out-of-range key, a runaway walk, or a
 * crash (run under ASan to make the last two loud).  Progress: every writer
 * completes its op count (a livelock would hang the test).
 *
 * QSBR flavor: updaters and readers run in RCU read-side sections (descriptor
 * existence); the main thread goes offline before joining so it does not stall
 * grace periods.
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
#include <urcu/flip-latch-lockfree.h>
#include <urcu/rcu-bidir-list-lockfree.h>

#include "tap.h"

#define NR_TESTS	3
#ifndef KEY_MAX
#define KEY_MAX		100
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

struct lnode {
	struct cds_bidir_list_lf_node node;
	struct rcu_head rh;
	int key;
};

static struct cds_bidir_list_lf_node g_head = CDS_BIDIR_LIST_LF_HEAD_INIT(g_head);
static int g_stop;

static int key_of(struct cds_bidir_list_lf_node *n)
{
	return caa_container_of(n, struct lnode, node)->key;
}

static void lnode_free(struct rcu_head *h)
{
	free(caa_container_of(h, struct lnode, rh));
}

static unsigned int xs(unsigned int x)
{
	x ^= x << 13;
	x ^= x >> 17;
	x ^= x << 5;
	return x;
}

/* Insert @key keeping the list non-decreasing.  Single-attempt MCAS + re-find. */
static void sorted_insert(int key)
{
	struct lnode *n = (struct lnode *) malloc(sizeof(*n));

	if (!n)
		abort();
	n->key = key;
	for (;;) {
		struct cds_bidir_list_lf_node *prev = &g_head, *succ;
		struct urcu_flip_lf_txn *t;
		bool ok;

		rcu_read_lock();
		/* prev = last node with key <= @key; succ = first with key > @key */
		for (;;) {
			succ = cds_bidir_list_lf_next_rcu(prev);
			if (succ == &g_head || key_of(succ) > key)
				break;
			prev = succ;
		}
		n->node.next = succ;
		n->node.prev = prev;
		t = urcu_flip_lf_txn_create(2);
		if (!t)
			abort();
		/* validates prev->next == succ and succ->prev == prev */
		urcu_flip_lf_txn_add(t, (void **) &prev->next, succ, &n->node);
		urcu_flip_lf_txn_add(t, (void **) &succ->prev, prev, &n->node);
		ok = urcu_flip_lf_txn_commit(t, call_rcu);
		rcu_read_unlock();
		if (ok)
			return;
		/* aborted (position shifted / anchor gone): re-find and retry */
	}
}

/* Delete the first node with key == @key, if present. */
static void delete_key(int key)
{
	struct cds_bidir_list_lf_node *p;
	struct lnode *target = NULL;

	rcu_read_lock();
	for (p = cds_bidir_list_lf_next_rcu(&g_head); p != &g_head;
			p = cds_bidir_list_lf_next_rcu(p)) {
		int k = key_of(p);

		if (k == key) {
			target = caa_container_of(p, struct lnode, node);
			break;
		}
		if (k > key)
			break;			/* sorted: past @key, absent */
	}
	if (target) {
		int r = cds_bidir_list_lf_del_rcu(&target->node, call_rcu);

		rcu_read_unlock();
		if (r == 1)			/* this call removed it -> reclaim */
			call_rcu(&target->rh, lnode_free);
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
		struct cds_bidir_list_lf_node *p;
		int prev_key, steps;

		/* forward: non-decreasing keys, bounded, ends at head */
		rcu_read_lock();
		prev_key = INT_MIN;
		steps = 0;
		for (p = cds_bidir_list_lf_next_rcu(&g_head); p != &g_head;
				p = cds_bidir_list_lf_next_rcu(p)) {
			int k = key_of(p);

			if (k < prev_key || k < 1 || k > KEY_MAX ||
					++steps > STEP_LIMIT) {
				st->violations++;
				break;
			}
			prev_key = k;
		}
		rcu_read_unlock();

		/* reverse: non-increasing keys */
		rcu_read_lock();
		prev_key = INT_MAX;
		steps = 0;
		for (p = cds_bidir_list_lf_prev_rcu(&g_head); p != &g_head;
				p = cds_bidir_list_lf_prev_rcu(p)) {
			int k = key_of(p);

			if (k > prev_key || k < 1 || k > KEY_MAX ||
					++steps > STEP_LIMIT) {
				st->violations++;
				break;
			}
			prev_key = k;
		}
		rcu_read_unlock();

		st->walks += 2;
		rcu_quiescent_state();
	}
	rcu_unregister_thread();
	return NULL;
}

/* Quiescent coherence check: reverse is the exact mirror of forward. */
static int check_mirror_coherent(void)
{
	static int fwd[STEP_LIMIT], rev[STEP_LIMIT];
	struct cds_bidir_list_lf_node *p;
	int nf = 0, nr = 0, i;

	for (p = cds_bidir_list_lf_next_rcu(&g_head);
			p != &g_head && nf < STEP_LIMIT;
			p = cds_bidir_list_lf_next_rcu(p))
		fwd[nf++] = key_of(p);
	for (p = cds_bidir_list_lf_prev_rcu(&g_head);
			p != &g_head && nr < STEP_LIMIT;
			p = cds_bidir_list_lf_prev_rcu(p))
		rev[nr++] = key_of(p);
	if (nf != nr)
		return 0;
	for (i = 0; i < nf; i++)
		if (fwd[i] != rev[nf - 1 - i])
			return 0;
	return 1;
}

int main(void)
{
	pthread_t wt[NR_WRITERS], rt[NR_READERS];
	struct writer_stats ws[NR_WRITERS];
	struct reader_stats rs[NR_READERS];
	long total_ops = 0, total_walks = 0, total_viol = 0;
	struct cds_bidir_list_lf_node *p;
	int i;

	plan_tests(NR_TESTS);
	rcu_register_thread();

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

	rcu_thread_offline();		/* don't stall grace periods while joined */
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

	diag("%d writers did %ld ops; %d readers did %ld walks, %ld violations",
		NR_WRITERS, total_ops, NR_READERS, total_walks, total_viol);

	ok(total_ops == (long) NR_WRITERS * WRITER_OPS,
		"every writer completed (lock-free progress)");
	ok(total_viol == 0,
		"forward/reverse walks stayed monotone (coherent both directions)");
	ok(check_mirror_coherent(),
		"at quiescence the reverse list is the exact mirror of the forward list");

	/* Drain the remaining nodes. */
	rcu_read_lock();
	for (p = cds_bidir_list_lf_next_rcu(&g_head); p != &g_head; ) {
		struct lnode *n = caa_container_of(p, struct lnode, node);
		struct cds_bidir_list_lf_node *nextp = cds_bidir_list_lf_next_rcu(p);

		if (cds_bidir_list_lf_del_rcu(&n->node, call_rcu) == 1)
			call_rcu(&n->rh, lnode_free);
		p = nextp;
	}
	rcu_read_unlock();

	rcu_barrier();
	rcu_unregister_thread();
	return exit_status();
}
