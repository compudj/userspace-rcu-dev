// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

/*
 * Concurrent stress test for <urcu/rcu-txn-sw-list.h>: one writer mutating a
 * key-sorted list while several readers iterate it forward AND backward.
 *
 * TWO invariants, checking different things, because the first one alone
 * cannot see the property this test exists for.
 *
 * (1) MONOTONICITY (per direction).  The writer keeps the list sorted by key at
 * all times, so a forward walk must see strictly INCREASING keys and a reverse
 * walk strictly DECREASING ones.  Out-of-range keys, runaway walks and crashes
 * are caught here too (run under ASan/TSan to make the last loud).
 *
 * What (1) CANNOT catch is a torn two-edge publish, which is the headline
 * guarantee.  Take an insert of X between P and Q publishing only its forward
 * edge: the forward ring is P -> X -> Q, still strictly increasing, and the
 * reverse ring is Q -> P, still strictly decreasing.  Both walks are monotone,
 * so both pass.  Every torn state of a two-edge op is sorted in each direction
 * SEPARATELY -- that is what makes the invariant provably blind to it.
 *
 * (2) CROSS-DIRECTION AGREEMENT, which is where a torn publish shows.  After
 * each forward step p -> q the reader also resolves prev_rcu(q) and requires
 * key(prev_rcu(q)) >= key(p).  A forward-only torn insert yields the
 * not-yet-back-linked predecessor, whose key is SMALLER than p's: caught.  A
 * reverse-only torn insert is caught symmetrically -- the forward walk does not
 * yet see X while prev_rcu already names it.
 *
 * (2) RUNS ONLY DURING AN INSERT-ONLY PHASE, and that restriction is not
 * incidental.  Under concurrent deletes the invariant is simply false: delete
 * p between the reader's two loads and prev_rcu(q) legitimately becomes the
 * node BEFORE p, whose key is smaller.  With inserts alone the legal outcomes
 * are exactly p itself or a node inserted between p and q, both with a key at
 * least p's, so any smaller one is a defect.
 *
 * The phase change is handshaken rather than assumed: the writer publishes
 * g_phase = 1 and then waits for every reader to acknowledge.  A reader samples
 * the phase at the TOP of an iteration and acknowledges only once it has seen
 * 1, so the acknowledgement is strictly after the last cross-checking walk
 * finished -- when the writer proceeds, none is in flight.
 *
 * Both invariants are then timing-independent within their phase: a violation
 * is a defect, never a race with the writer.  This is also the guarantee
 * classic rculist's reverse walk cannot provide.
 *
 * QSBR flavor: read-side critical sections bracket each walk, and every
 * thread announces a quiescent state between critical sections so grace
 * periods (hence the deferred frees) can advance.
 */

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#ifndef _LGPL_SOURCE
#define _LGPL_SOURCE
#endif

#include <inttypes.h>
#include <poll.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <urcu/compiler.h>
#include <urcu-qsbr.h>
#include <urcu-call-rcu.h>
#include <urcu/rcu-txn-sw-list.h>

#include "tap.h"

#define NR_TESTS	3
#define KEY_MAX		100		/* key space 1..KEY_MAX */
#define NR_READERS	4
#define WRITER_OPS	300000
#define STEP_LIMIT	(KEY_MAX + 8)	/* runaway-walk guard */
#define INSERT_ROUNDS	40		/* insert-only rounds for invariant (2) */

struct bl_node {
	struct urcu_txn_sw_list_node node;
	struct rcu_head rcu_head;
	int key;
};

static struct urcu_txn_sw_list_head g_head = URCU_TXN_SW_LIST_HEAD_INIT(g_head);
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
	long violations;		/* invariant (1): monotone per direction */
	long cross_violations;		/* invariant (2): the two directions agree */
	long cross_checks;
	int ack;			/* saw g_phase == 1; no (2) walk in flight */
};

/* 0 = insert-only (invariant 2 applies), 1 = mixed insert/delete/replace. */
static int g_phase;

static void *reader_fn(void *arg)
{
	struct reader_stats *st = (struct reader_stats *) arg;

	rcu_register_thread();
	while (!uatomic_load(&g_stop, CMM_RELAXED)) {
		struct urcu_txn_sw_list_node *p;
		int prev_key, steps;
		int insert_only = !uatomic_load(&g_phase, CMM_RELAXED);

		if (!insert_only)
			uatomic_store(&st->ack, 1, CMM_RELAXED);

		/* Forward: strictly increasing keys, bounded, ends at head. */
		rcu_read_lock();
		prev_key = 0;
		steps = 0;
		for (p = urcu_txn_sw_list_next_rcu(&g_head.node); p != &g_head.node;
				p = urcu_txn_sw_list_next_rcu(p)) {
			int k = caa_container_of(p, struct bl_node, node)->key;

			if (k < 1 || k > KEY_MAX || k <= prev_key ||
					++steps > STEP_LIMIT) {
				st->violations++;
				break;
			}
			/*
			 * Invariant (2).  Having just stepped prev_key -> k,
			 * back-resolve: the node k names as its predecessor must
			 * have a key at least prev_key's.  It may be larger --
			 * an insert can have landed between the two since -- but
			 * a SMALLER one means k's prev still names a node the
			 * forward ring has already moved past, which is a
			 * forward-only publish of a two-edge op.  Skipped for
			 * the first step, whose predecessor is the head.
			 */
			if (insert_only && prev_key != 0) {
				struct urcu_txn_sw_list_node *bp =
					urcu_txn_sw_list_prev_rcu(p);

				st->cross_checks++;
				if (bp != &g_head.node &&
						caa_container_of(bp,
							struct bl_node,
							node)->key < prev_key)
					st->cross_violations++;
			}
			prev_key = k;
		}
		rcu_read_unlock();

		/* Reverse: strictly decreasing keys, bounded, ends at head. */
		rcu_read_lock();
		prev_key = KEY_MAX + 1;
		steps = 0;
		for (p = urcu_txn_sw_list_prev_rcu(&g_head.node); p != &g_head.node;
				p = urcu_txn_sw_list_prev_rcu(p)) {
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
	struct urcu_txn_sw_list_node *succ = &g_head.node;	/* default: tail */
	int j;

	for (j = n->key + 1; j <= KEY_MAX; j++) {
		if (slot[j]) {
			succ = &slot[j]->node;
			break;
		}
	}
	if (urcu_txn_sw_list_add_before_rcu(&n->node, succ))
		abort();
	slot[n->key] = n;
}

int main(void)
{
	pthread_t readers[NR_READERS];
	struct reader_stats stats[NR_READERS];
	struct bl_node *slot[KEY_MAX + 1];
	long total_walks = 0, total_violations = 0;
	long total_cross = 0, total_cross_checks = 0;
	unsigned int rng = 2463534242u;
	int i, k, round;

	plan_tests(NR_TESTS);
	rcu_register_thread();

	memset(slot, 0, sizeof(slot));
	memset(stats, 0, sizeof(stats));

	for (i = 0; i < NR_READERS; i++)
		pthread_create(&readers[i], NULL, reader_fn, &stats[i]);

	/*
	 * PHASE 0, repeated: fill the list one key at a time -- INSERTS ONLY, so
	 * invariant (2) holds and the readers are cross-checking against live
	 * concurrent inserts, which is the whole point.  Then hand over to phase
	 * 1, empty it with deletes (invariant 2 off), and go round again.
	 */
	for (round = 0; round < INSERT_ROUNDS; round++) {
		int order[KEY_MAX], m;

		for (k = 0; k < KEY_MAX; k++)
			order[k] = k + 1;
		for (k = KEY_MAX - 1; k > 0; k--) {	/* shuffle */
			rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5;
			m = (int) (rng % (unsigned int) (k + 1));
			i = order[k]; order[k] = order[m]; order[m] = i;
		}

		/* back to insert-only, and let every reader observe it */
		uatomic_store(&g_phase, 0, CMM_RELAXED);
		for (i = 0; i < NR_READERS; i++)
			uatomic_store(&stats[i].ack, 0, CMM_RELAXED);

		for (k = 0; k < KEY_MAX; k++) {
			sorted_insert(bl_node_new(order[k]), slot);
			rcu_quiescent_state();	/* let the readers walk between them */
		}

		/* hand over: wait until no phase-0 walk can still be running */
		uatomic_store(&g_phase, 1, CMM_RELAXED);
		for (i = 0; i < NR_READERS; i++) {
			while (!uatomic_load(&stats[i].ack, CMM_RELAXED)) {
				rcu_quiescent_state();
				(void) poll(NULL, 0, 1);
			}
		}

		for (k = 1; k <= KEY_MAX; k++) {	/* empty it again */
			if (!slot[k])
				continue;
			if (urcu_txn_sw_list_del_rcu(&slot[k]->node))
				abort();
			call_rcu(&slot[k]->rcu_head, bl_node_free);
			slot[k] = NULL;
			rcu_quiescent_state();
		}
	}

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
			if (urcu_txn_sw_list_replace_rcu(&old->node, &fresh->node))
				abort();
			call_rcu(&old->rcu_head, bl_node_free);
			slot[k] = fresh;
		} else {
			struct bl_node *old = slot[k];

			if (urcu_txn_sw_list_del_rcu(&old->node))
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
			if (urcu_txn_sw_list_del_rcu(&slot[k]->node))
				abort();
			call_rcu(&slot[k]->rcu_head, bl_node_free);
			slot[k] = NULL;
		}
	}

	for (i = 0; i < NR_READERS; i++) {
		total_walks += stats[i].walks;
		total_violations += stats[i].violations;
		total_cross += stats[i].cross_violations;
		total_cross_checks += stats[i].cross_checks;
	}
	diag("%d readers performed %ld directional walks, %ld violations; "
		"%ld cross-direction checks, %ld violations",
		NR_READERS, total_walks, total_violations,
		total_cross_checks, total_cross);

	ok(total_walks > 0, "readers iterated concurrently with the writer");
	ok(total_violations == 0,
		"forward/reverse walks stayed monotone (coherent both directions)");
	ok(total_cross_checks > 0 && total_cross == 0,
		"every forward step agreed with the successor's prev: no two-edge "
		"publish was ever seen torn (%ld checks)", total_cross_checks);

	rcu_barrier();
	rcu_unregister_thread();
	return exit_status();
}
