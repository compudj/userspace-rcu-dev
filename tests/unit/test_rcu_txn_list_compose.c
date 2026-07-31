// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

/*
 * Composability stress for the concurrent bidir list: a single node is a member
 * of TWO lists at once (multi-membership through two intrusive hooks in one
 * leaf), and every mutation updates BOTH lists in ONE MCAS via the composable
 * *_prepare() forms -- the headline use case for folding a list splice together
 * with another data structure that shares the node.
 *
 * Because one transaction now spans both lists, its install window is wider than
 * a single-list op's: the slot-sorted MCAS may touch a node in list Y while a
 * concurrent del() in list X is freeing a neighbour, and a contended anchor's
 * next-pointer can A-B-A back to a recurred value.  That window is exactly what
 * the per-record install latch (install-once + self-settle) makes safe; this
 * test exercises it under concurrency with real node reclaim.
 *
 * Each writer owns a distinct anchor in both lists and churns / keeps its leaves
 * after that anchor, so contention is spread across anchors rather than piled on
 * a single hot slot (a hot-slot churn pattern stresses the txn escalation lane,
 * an orthogonal concern -- here we validate composition, not lane throughput).
 * Writers churn dynamic leaves (insert-both -> del-both -> rcu-free) for memory
 * safety (run under ASan), then each writer inserts KEEPERS leaves into both
 * lists and stops.  Readers walk both lists, both directions, throughout.  At
 * quiescence we assert cross-list atomicity (every keeper is in BOTH lists, same
 * count) and that each list is internally coherent (forward == reverse).
 *
 * QSBR flavor.
 */

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#ifndef _LGPL_SOURCE
#define _LGPL_SOURCE
#endif

#include <errno.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>

#include <urcu/assert.h>
#include <urcu/compiler.h>
#include <urcu/uatomic.h>
#include <urcu-qsbr.h>
#include <urcu-call-rcu.h>
#include <urcu/rcu-txn.h>
#include <urcu/rcu-txn-list.h>

#include "tap.h"

#ifndef NR_WRITERS
#define NR_WRITERS		4
#endif
#ifndef NR_READERS
#define NR_READERS		2
#endif
#ifndef CHURN_ROUNDS
#define CHURN_ROUNDS		300
#endif
#ifndef KEEPERS_PER_WRITER
#define KEEPERS_PER_WRITER	4
#endif
#define NR_KEEPERS		(NR_WRITERS * KEEPERS_PER_WRITER)
#define NR_NODES		(NR_WRITERS + NR_KEEPERS)	/* anchors + keepers */

#define NR_TESTS		7

struct leaf {
	struct urcu_txn_list_node hx;	/* hook in list X */
	struct urcu_txn_list_node hy;	/* hook in list Y */
	struct rcu_head rcu;
	int is_anchor;
	int seen_x, seen_y;			/* set by the final verify walk */
};

/* Two lists plus one shared escalation lane: a composed txn spans both lists. */
static struct urcu_txn_list_head g_X, g_Y;
static struct urcu_txn_domain g_domain;
static int g_stop;				/* signals the readers to finish */

static struct leaf g_anchor[NR_WRITERS];
static struct leaf *g_keepers[NR_WRITERS][KEEPERS_PER_WRITER];

static void free_leaf_cb(struct rcu_head *h)
{
	free(caa_container_of(h, struct leaf, rcu));
}

/* Atomically link @nx after @px in X and @ny after @py in Y, in one MCAS. */
static void compose_insert(struct urcu_txn_list_node *nx,
		struct urcu_txn_list_node *px,
		struct urcu_txn_list_node *ny,
		struct urcu_txn_list_node *py)
{
	struct urcu_txn tx;
	enum urcu_txn_status st;

	urcu_txn_init(&tx, &g_domain);
	for (;;) {
		int a, b;

		urcu_txn_begin(&tx);
		a = urcu_txn_list_insert_after_prepare(&tx, nx, px);
		b = urcu_txn_list_insert_after_prepare(&tx, ny, py);
		if (a || b) {			/* -EAGAIN (neighbour moved): retry */
			urcu_txn_conflict(&tx);	/* age so a hot slot escalates */
			urcu_txn_end(&tx);
			continue;
		}
		st = urcu_txn_commit(&tx);
		urcu_txn_end(&tx);
		if (st != URCU_TXN_STATUS_ABORT)
			break;
	}
}

/* Atomically unlink @lf from BOTH lists in one MCAS.  Returns 1 if removed. */
static int compose_del(struct leaf *lf)
{
	struct urcu_txn tx;
	enum urcu_txn_status st;

	urcu_txn_init(&tx, &g_domain);
	for (;;) {
		int a, b;

		urcu_txn_begin(&tx);
		a = urcu_txn_list_del_prepare(&tx, &lf->hx);
		b = urcu_txn_list_del_prepare(&tx, &lf->hy);
		if (a == -EAGAIN || b == -EAGAIN) {	/* a neighbour moved: retry */
			urcu_txn_conflict(&tx);	/* age so a hot slot escalates */
			urcu_txn_end(&tx);
			continue;
		}
		if (a == -ENOENT && b == -ENOENT) {	/* both already gone */
			urcu_txn_end(&tx);
			return 0;
		}
		/*
		 * A MIXED outcome (one -ENOENT, the other 0) would commit a
		 * half-edit: the live hook is changed while the dead one is
		 * not, and the caller then reclaims a node still reachable from
		 * the other list.  It cannot happen here -- a leaf enters and
		 * leaves both lists in ONE commit, so the two hooks always die
		 * together -- but that is a test-local invariant, not a
		 * property of the composition.  An embedder composing edits of
		 * INDEPENDENTLY deletable nodes must abandon the bracket on a
		 * mixed outcome, not commit it.
		 */
		urcu_posix_assert((a == -ENOENT) == (b == -ENOENT));
		st = urcu_txn_commit(&tx);
		urcu_txn_end(&tx);
		if (st != URCU_TXN_STATUS_ABORT)
			break;
	}
	return 1;
}

/* Atomically replace @old by @nw in BOTH lists in one MCAS.  Returns 1 if done. */
static int compose_replace(struct leaf *nw, struct leaf *old)
{
	struct urcu_txn tx;
	enum urcu_txn_status st;

	urcu_txn_init(&tx, &g_domain);
	for (;;) {
		int a, b;

		urcu_txn_begin(&tx);
		a = urcu_txn_list_replace_prepare(&tx, &old->hx, &nw->hx);
		b = urcu_txn_list_replace_prepare(&tx, &old->hy, &nw->hy);
		if (a == -EAGAIN || b == -EAGAIN) {	/* a neighbour moved: retry */
			urcu_txn_conflict(&tx);	/* age so a hot slot escalates */
			urcu_txn_end(&tx);
			continue;
		}
		if (a == -ENOENT && b == -ENOENT) {	/* both already gone */
			urcu_txn_end(&tx);
			return 0;
		}
		/*
		 * A MIXED outcome (one -ENOENT, the other 0) would commit a
		 * half-edit: the live hook is changed while the dead one is
		 * not, and the caller then reclaims a node still reachable from
		 * the other list.  It cannot happen here -- a leaf enters and
		 * leaves both lists in ONE commit, so the two hooks always die
		 * together -- but that is a test-local invariant, not a
		 * property of the composition.  An embedder composing edits of
		 * INDEPENDENTLY deletable nodes must abandon the bracket on a
		 * mixed outcome, not commit it.
		 */
		urcu_posix_assert((a == -ENOENT) == (b == -ENOENT));
		st = urcu_txn_commit(&tx);
		urcu_txn_end(&tx);
		if (st != URCU_TXN_STATUS_ABORT)
			break;
	}
	return 1;
}

/* Walk one list both directions just to drive proxy resolution under mutation. */
static void walk_once(struct urcu_txn_list_head *h)
{
	struct urcu_txn_list_node *p;
	long n = 0, lim = 4L * NR_NODES + 8;	/* loose bound vs a transient cycle */

	rcu_read_lock();
	for (p = urcu_txn_list_next_rcu(&h->node);
			p != &h->node && n < lim;
			p = urcu_txn_list_next_rcu(p))
		n++;
	for (p = urcu_txn_list_prev_rcu(&h->node);
			p != &h->node && n < 2 * lim;
			p = urcu_txn_list_prev_rcu(p))
		n++;
	rcu_read_unlock();
}

static void *reader_fn(void *arg)
{
	(void) arg;
	rcu_register_thread();
	while (!uatomic_load(&g_stop, CMM_RELAXED)) {
		walk_once(&g_X);
		walk_once(&g_Y);
		rcu_quiescent_state();
	}
	rcu_unregister_thread();
	return NULL;
}

static void *writer_fn(void *arg)
{
	long w = (long) arg;
	struct urcu_txn_list_node *ax = &g_anchor[w].hx, *ay = &g_anchor[w].hy;
	int r, k;

	rcu_register_thread();

	/* Churn: insert-both -> replace-both -> del-both, all composed, with reclaim. */
	for (r = 0; r < CHURN_ROUNDS; r++) {
		struct leaf *lf = calloc(1, sizeof(*lf));
		struct leaf *lf2 = calloc(1, sizeof(*lf2));

		if (!lf || !lf2)
			abort();
		compose_insert(&lf->hx, ax, &lf->hy, ay);	/* lf into both lists */
		(void) compose_replace(lf2, lf);		/* swap lf -> lf2 in both */
		call_rcu(&lf->rcu, free_leaf_cb);		/* lf is now a ghost */
		(void) compose_del(lf2);			/* lf2 out of both lists */
		call_rcu(&lf2->rcu, free_leaf_cb);
		if ((r & 31) == 0)
			rcu_quiescent_state();	/* QSBR: let GPs (and reclaim) advance */
	}

	/* Keepers: insert into both lists and leave them in for the verify. */
	for (k = 0; k < KEEPERS_PER_WRITER; k++) {
		struct leaf *lf = calloc(1, sizeof(*lf));

		if (!lf)
			abort();
		compose_insert(&lf->hx, ax, &lf->hy, ay);
		g_keepers[w][k] = lf;
	}

	rcu_unregister_thread();
	return NULL;
}

/* Internally coherent: forward order is the exact reverse of backward order. */
static int list_coherent(struct urcu_txn_list_head *h, int expect)
{
	struct urcu_txn_list_node *fwd[NR_NODES + 2], *bwd[NR_NODES + 2], *p;
	int nf = 0, nb = 0, i;

	for (p = urcu_txn_list_next_rcu(&h->node);
			p != &h->node && nf <= NR_NODES + 1;
			p = urcu_txn_list_next_rcu(p))
		fwd[nf++] = p;
	for (p = urcu_txn_list_prev_rcu(&h->node);
			p != &h->node && nb <= NR_NODES + 1;
			p = urcu_txn_list_prev_rcu(p))
		bwd[nb++] = p;
	if (nf != expect || nb != expect)
		return 0;
	for (i = 0; i < nf; i++)
		if (fwd[i] != bwd[nf - 1 - i])
			return 0;
	return 1;
}

/*
 * SAME-LIST composition, the case the two-list fan-out above cannot reach.
 *
 * Every composed pair over X and Y touches structurally disjoint slots, so
 * read-your-own-writes and same-slot chaining -- the whole reason the header
 * says "compose on a DEFAULT handle" -- never fire.  These do: adjacent
 * deletes fuse three same-slot pairs (the second del must see the first's
 * redirected prev THROUGH RYW, then chain onto its record), and an insert next
 * to a delete collides on the shared anchor edge.  Both also exercise the
 * age-0 esc_pending abort and the age-1 chaining round-trip, since the
 * prepares alias by construction.
 */
static void same_list_compose(void)
{
	struct urcu_txn_list_head h;
	static struct urcu_txn_list_node n[4], ins;
	struct urcu_txn tx;
	enum urcu_txn_status st;
	struct urcu_txn_list_node *p;
	int i, len, ok_adj, ok_mix;

	/* ring: h -> n0 -> n1 -> n2 -> n3 -> h */
	urcu_txn_list_init(&h);
	p = &h.node;
	for (i = 0; i < 4; i++) {
		urcu_txn_init(&tx, NULL);
		urcu_txn_begin(&tx);
		if (urcu_txn_list_insert_after_prepare(&tx, &n[i], p))
			abort();
		if (urcu_txn_commit(&tx) != URCU_TXN_STATUS_OK)
			abort();
		urcu_txn_end(&tx);
		p = &n[i];
	}

	/* ADJACENT DELETES in one commit: n1 and n2, which share edges. */
	urcu_txn_init(&tx, NULL);
	for (;;) {
		int a, b;

		urcu_txn_begin(&tx);
		a = urcu_txn_list_del_prepare(&tx, &n[1]);
		b = urcu_txn_list_del_prepare(&tx, &n[2]);
		if (a == -EAGAIN || b == -EAGAIN) {
			urcu_txn_conflict(&tx);
			urcu_txn_end(&tx);
			continue;
		}
		if (a || b)
			abort();
		st = urcu_txn_commit(&tx);
		urcu_txn_end(&tx);
		if (st != URCU_TXN_STATUS_ABORT)
			break;
	}
	len = 0;
	ok_adj = (st == URCU_TXN_STATUS_OK);
	for (p = urcu_txn_list_next_rcu(&h.node); p != &h.node;
			p = urcu_txn_list_next_rcu(p)) {
		if (p == &n[1] || p == &n[2])
			ok_adj = 0;		/* a victim is still linked */
		if (++len > 8)
			break;			/* the ring is broken */
	}
	/* n0 <-> n3 must now be consecutive in BOTH directions */
	ok_adj = ok_adj && len == 2 &&
		urcu_txn_list_next_rcu(&n[0]) == &n[3] &&
		urcu_txn_list_prev_rcu(&n[3]) == &n[0];
	ok(ok_adj, "same-list compose: adjacent deletes fuse into one commit and "
		"leave a coherent ring");

	/* INSERT NEXT TO A DELETE: delete n3 and insert after n0, same commit. */
	urcu_txn_init(&tx, NULL);
	for (;;) {
		int a, b;

		urcu_txn_begin(&tx);
		a = urcu_txn_list_del_prepare(&tx, &n[3]);
		b = urcu_txn_list_insert_after_prepare(&tx, &ins, &n[0]);
		if (a == -EAGAIN || b == -EAGAIN) {
			urcu_txn_conflict(&tx);
			urcu_txn_end(&tx);
			continue;
		}
		if (a || b)
			abort();
		st = urcu_txn_commit(&tx);
		urcu_txn_end(&tx);
		if (st != URCU_TXN_STATUS_ABORT)
			break;
	}
	ok_mix = (st == URCU_TXN_STATUS_OK) &&
		urcu_txn_list_next_rcu(&h.node) == &n[0] &&
		urcu_txn_list_next_rcu(&n[0]) == &ins &&
		urcu_txn_list_next_rcu(&ins) == &h.node &&
		urcu_txn_list_prev_rcu(&ins) == &n[0] &&
		urcu_txn_list_prev_rcu(&h.node) == &ins;
	ok(ok_mix, "same-list compose: an insert adjacent to a delete commits as "
		"one edit, in both directions");
}

int main(void)
{
	pthread_t wt[NR_WRITERS], rt[NR_READERS];
	struct urcu_txn_list_node *p, *px, *py;
	long i;
	int cx = 0, cy = 0, all_both = 1, coh_x, coh_y;

	plan_tests(NR_TESTS);
	rcu_register_thread();

	same_list_compose();

	urcu_txn_list_init(&g_X);
	urcu_txn_list_init(&g_Y);
	urcu_txn_domain_init(&g_domain);

	/* Lay down one permanent anchor per writer, chained in both lists. */
	px = &g_X.node;
	py = &g_Y.node;
	for (i = 0; i < NR_WRITERS; i++) {
		g_anchor[i].is_anchor = 1;
		compose_insert(&g_anchor[i].hx, px, &g_anchor[i].hy, py);
		px = &g_anchor[i].hx;
		py = &g_anchor[i].hy;
	}

	for (i = 0; i < NR_READERS; i++)
		pthread_create(&rt[i], NULL, reader_fn, NULL);
	for (i = 0; i < NR_WRITERS; i++)
		pthread_create(&wt[i], NULL, writer_fn, (void *) i);
	for (i = 0; i < NR_WRITERS; i++)
		pthread_join(wt[i], NULL);
	uatomic_store(&g_stop, 1, CMM_RELAXED);
	for (i = 0; i < NR_READERS; i++)
		pthread_join(rt[i], NULL);

	rcu_quiescent_state();		/* drain reclaim so the lists are stable */

	/* Cross-list atomicity: every keeper landed in BOTH lists, equal counts. */
	rcu_read_lock();
	for (p = urcu_txn_list_next_rcu(&g_X.node); p != &g_X.node;
			p = urcu_txn_list_next_rcu(p)) {
		struct leaf *lf = caa_container_of(p, struct leaf, hx);

		if (!lf->is_anchor && ++cx <= NR_KEEPERS)
			lf->seen_x = 1;
	}
	for (p = urcu_txn_list_next_rcu(&g_Y.node); p != &g_Y.node;
			p = urcu_txn_list_next_rcu(p)) {
		struct leaf *lf = caa_container_of(p, struct leaf, hy);

		if (!lf->is_anchor && ++cy <= NR_KEEPERS)
			lf->seen_y = 1;
	}
	rcu_read_unlock();

	for (i = 0; i < NR_WRITERS; i++) {
		int k;

		for (k = 0; k < KEEPERS_PER_WRITER; k++) {
			struct leaf *lf = g_keepers[i][k];

			if (!lf || !lf->seen_x || !lf->seen_y)
				all_both = 0;
		}
	}

	ok(cx == NR_KEEPERS, "list X holds exactly the %d keepers (got %d)",
		NR_KEEPERS, cx);
	ok(cy == NR_KEEPERS, "list Y holds exactly the %d keepers (got %d)",
		NR_KEEPERS, cy);
	ok(all_both,
		"every keeper is in BOTH lists -- composed insert was atomic across them");

	coh_x = list_coherent(&g_X, NR_NODES);
	coh_y = list_coherent(&g_Y, NR_NODES);
	ok(coh_x, "list X coherent: forward order is the exact reverse of backward");
	ok(coh_y, "list Y coherent: forward order is the exact reverse of backward");

	/* Cleanup: unlink and free the keepers, drain all deferred frees. */
	for (i = 0; i < NR_WRITERS; i++) {
		int k;

		for (k = 0; k < KEEPERS_PER_WRITER; k++)
			if (g_keepers[i][k]) {
				(void) compose_del(g_keepers[i][k]);
				call_rcu(&g_keepers[i][k]->rcu, free_leaf_cb);
			}
	}
	rcu_barrier();
	rcu_unregister_thread();
	return exit_status();
}
