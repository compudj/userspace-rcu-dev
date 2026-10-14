// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

/*
 * Deterministic unit tests for <urcu/rcu-txn-sw-list.h>, the bidirectional
 * RCU list.  Validates that:
 *   - forward and reverse iteration are exact mirrors;
 *   - next/prev are mutual inverses around the whole ring (coherence);
 *   - add / add_tail / del / replace / move keep both directions coherent;
 *   - the proxy-flip resolves correctly across the
 *     install -> commit -> settle phases of a two-edge flip (the
 *     mechanism a concurrent reader relies on).
 *
 * QSBR flavor, single threaded, so the outcome is fully deterministic.
 */

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#ifndef _LGPL_SOURCE
#define _LGPL_SOURCE
#endif

#include <stdlib.h>
#include <string.h>

#include <urcu/compiler.h>
#include <urcu-qsbr.h>
#include <urcu-call-rcu.h>
#include <urcu/rcu-txn-sw-list.h>

#include "rcu-txn-sw-internal.h"	/* the steps of a commit */

#include "tap.h"

#define NR_TESTS	62

struct bl_node {
	struct urcu_txn_sw_list_node node;
	struct rcu_head rcu_head;
	int key;
};

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

/* Collect forward keys.  Returns count, fills buf (up to max). */
static int collect_forward(struct urcu_txn_sw_list_head *head, int *buf, int max)
{
	struct urcu_txn_sw_list_node *p;
	int n = 0;

	rcu_read_lock();
	urcu_txn_sw_list_for_each_rcu(p, head) {
		if (n < max)
			buf[n] = caa_container_of(p, struct bl_node, node)->key;
		n++;
	}
	rcu_read_unlock();
	return n;
}

static int collect_reverse(struct urcu_txn_sw_list_head *head, int *buf, int max)
{
	struct urcu_txn_sw_list_node *p;
	int n = 0;

	rcu_read_lock();
	urcu_txn_sw_list_for_each_reverse_rcu(p, head) {
		if (n < max)
			buf[n] = caa_container_of(p, struct bl_node, node)->key;
		n++;
	}
	rcu_read_unlock();
	return n;
}

static int arr_eq(const int *a, const int *b, int n)
{
	return memcmp(a, b, (size_t) n * sizeof(int)) == 0;
}

/*
 * Walk the whole ring (sentinel included) and verify next/prev are mutual
 * inverses at every edge: next_rcu(p) == q  implies  prev_rcu(q) == p.
 */
static int check_inverses(struct urcu_txn_sw_list_head *head)
{
	struct urcu_txn_sw_list_node *p, *q;
	int ok = 1;

	rcu_read_lock();
	for (p = &head->node; ; p = q) {
		q = urcu_txn_sw_list_next_rcu(p);
		if (urcu_txn_sw_list_prev_rcu(q) != p) {
			ok = 0;
			break;
		}
		if (q == &head->node)
			break;
	}
	rcu_read_unlock();
	return ok;
}

/* Find the node holding @key (single-threaded helper). */
static struct urcu_txn_sw_list_node *find_key(struct urcu_txn_sw_list_head *head,
		int key)
{
	struct urcu_txn_sw_list_node *p;

	urcu_txn_sw_list_for_each_rcu(p, head) {
		if (caa_container_of(p, struct bl_node, node)->key == key)
			return p;
	}
	return NULL;
}

/* Delete every node and reclaim it. */
static void destroy_list(struct urcu_txn_sw_list_head *head)
{
	while (!urcu_txn_sw_list_empty(head)) {
		struct urcu_txn_sw_list_node *p = urcu_txn_sw_list_next_rcu(&head->node);
		struct bl_node *n = caa_container_of(p, struct bl_node, node);

		urcu_txn_sw_list_del_rcu(p);
		call_rcu(&n->rcu_head, bl_node_free);
	}
}

static void test_empty(void)
{
	URCU_TXN_SW_LIST_HEAD(head);
	int buf[4];

	ok(urcu_txn_sw_list_empty(&head), "fresh list is empty");
	ok(collect_forward(&head, buf, 4) == 0, "empty: no forward elements");
	ok(collect_reverse(&head, buf, 4) == 0, "empty: no reverse elements");
}

static void test_add_head(void)
{
	URCU_TXN_SW_LIST_HEAD(head);
	int i, fwd[3], rev[3], n;
	const int want_fwd[3] = { 3, 2, 1 };	/* head-add reverses input */
	const int want_rev[3] = { 1, 2, 3 };

	for (i = 1; i <= 3; i++)
		urcu_txn_sw_list_add_rcu(&bl_node_new(i)->node, &head);

	n = collect_forward(&head, fwd, 3);
	ok(n == 3 && arr_eq(fwd, want_fwd, 3),
		"add at head: forward order is 3,2,1");
	n = collect_reverse(&head, rev, 3);
	ok(n == 3 && arr_eq(rev, want_rev, 3),
		"add at head: reverse order is the mirror 1,2,3");
	destroy_list(&head);
}

static void test_add_tail(void)
{
	URCU_TXN_SW_LIST_HEAD(head);
	int i, fwd[3], rev[3], n;
	const int want_fwd[3] = { 1, 2, 3 };
	const int want_rev[3] = { 3, 2, 1 };

	for (i = 1; i <= 3; i++)
		urcu_txn_sw_list_add_tail_rcu(&bl_node_new(i)->node, &head);

	n = collect_forward(&head, fwd, 3);
	ok(n == 3 && arr_eq(fwd, want_fwd, 3),
		"add at tail: forward order is 1,2,3");
	n = collect_reverse(&head, rev, 3);
	ok(n == 3 && arr_eq(rev, want_rev, 3),
		"add at tail: reverse order is the mirror 3,2,1");
	destroy_list(&head);
}

static void test_coherence(void)
{
	URCU_TXN_SW_LIST_HEAD(head);
	const int keys[4] = { 10, 20, 30, 40 };
	int i, fwd[4], rev[4], revmir[4], n;

	for (i = 0; i < 4; i++)
		urcu_txn_sw_list_add_tail_rcu(&bl_node_new(keys[i])->node, &head);

	ok(check_inverses(&head), "next/prev are mutual inverses around the ring");

	n = collect_forward(&head, fwd, 4);
	collect_reverse(&head, rev, 4);
	for (i = 0; i < n; i++)
		revmir[i] = rev[n - 1 - i];
	ok(n == 4 && arr_eq(fwd, revmir, n),
		"reverse iteration is the exact mirror of forward");
	destroy_list(&head);
}

static void test_del_middle(void)
{
	URCU_TXN_SW_LIST_HEAD(head);
	const int keys[5] = { 1, 2, 3, 4, 5 };
	const int want_fwd[4] = { 1, 2, 4, 5 };
	const int want_rev[4] = { 5, 4, 2, 1 };
	struct urcu_txn_sw_list_node *mid;
	struct bl_node *midn;
	int i, fwd[4], rev[4], n;

	for (i = 0; i < 5; i++)
		urcu_txn_sw_list_add_tail_rcu(&bl_node_new(keys[i])->node, &head);
	mid = find_key(&head, 3);
	midn = caa_container_of(mid, struct bl_node, node);
	urcu_txn_sw_list_del_rcu(mid);
	call_rcu(&midn->rcu_head, bl_node_free);

	n = collect_forward(&head, fwd, 4);
	ok(n == 4 && arr_eq(fwd, want_fwd, 4),
		"del middle: forward is 1,2,4,5");
	n = collect_reverse(&head, rev, 4);
	ok(n == 4 && arr_eq(rev, want_rev, 4),
		"del middle: reverse is 5,4,2,1");
	ok(check_inverses(&head), "del middle: ring stays coherent");
	destroy_list(&head);
}

static void test_del_ends(void)
{
	URCU_TXN_SW_LIST_HEAD(head);
	const int keys[4] = { 1, 2, 3, 4 };
	const int want_fwd[2] = { 2, 3 };
	struct urcu_txn_sw_list_node *p;
	struct bl_node *n;
	int i, fwd[2], cnt;

	for (i = 0; i < 4; i++)
		urcu_txn_sw_list_add_tail_rcu(&bl_node_new(keys[i])->node, &head);
	p = find_key(&head, 1);				/* delete head element */
	n = caa_container_of(p, struct bl_node, node);
	urcu_txn_sw_list_del_rcu(p);
	call_rcu(&n->rcu_head, bl_node_free);
	p = find_key(&head, 4);				/* delete tail element */
	n = caa_container_of(p, struct bl_node, node);
	urcu_txn_sw_list_del_rcu(p);
	call_rcu(&n->rcu_head, bl_node_free);

	cnt = collect_forward(&head, fwd, 2);
	ok(cnt == 2 && arr_eq(fwd, want_fwd, 2),
		"del both ends: forward is 2,3");
	ok(check_inverses(&head), "del both ends: ring stays coherent");
	destroy_list(&head);
}

static void test_replace(void)
{
	URCU_TXN_SW_LIST_HEAD(head);
	const int keys[3] = { 1, 2, 3 };
	const int want_fwd[3] = { 1, 22, 3 };
	const int want_rev[3] = { 3, 22, 1 };
	struct urcu_txn_sw_list_node *old;
	struct bl_node *oldn;
	int i, fwd[3], rev[3], n;

	for (i = 0; i < 3; i++)
		urcu_txn_sw_list_add_tail_rcu(&bl_node_new(keys[i])->node, &head);
	old = find_key(&head, 2);
	oldn = caa_container_of(old, struct bl_node, node);
	urcu_txn_sw_list_replace_rcu(old, &bl_node_new(22)->node);
	call_rcu(&oldn->rcu_head, bl_node_free);

	n = collect_forward(&head, fwd, 3);
	ok(n == 3 && arr_eq(fwd, want_fwd, 3),
		"replace: forward reflects new node");
	n = collect_reverse(&head, rev, 3);
	ok(n == 3 && arr_eq(rev, want_rev, 3),
		"replace: reverse reflects new node");
	ok(check_inverses(&head), "replace: ring stays coherent");
	destroy_list(&head);
}

/*
 * Drive the two-edge flip of a delete by hand and observe the reader's
 * resolution across the install -> commit -> settle phases.  This is the
 * exact view a concurrent reader has of urcu_txn_sw_list_del_rcu(): old in both
 * directions until the single selector flip, new in both directions after.
 *
 * Ring: head <-> A <-> B <-> head.  Delete A:
 *   edge 0: head->next : A -> B   (forward)
 *   edge 1: B->prev    : A -> head (backward)
 */
static void test_proxy_phases(void)
{
	URCU_TXN_SW_LIST_HEAD(head);
	struct bl_node *a = bl_node_new(1);
	struct bl_node *b = bl_node_new(2);
	struct urcu_txn_sw_list_node *A = &a->node, *B = &b->node;
	struct urcu_txn_sw_group group;
	struct urcu_txn_sw_record rec[2];

	urcu_txn_sw_list_add_tail_rcu(A, &head);
	urcu_txn_sw_list_add_tail_rcu(B, &head);

	/*
	 * Delete A by hand on the flip-group primitives, pausing between the
	 * steps to observe the reader's view of each -- the same install ->
	 * commit -> settle that urcu_txn_sw_commit() runs for the mutators.
	 */
	urcu_txn_sw__group_init(&group);
	urcu_txn_sw__record_set(&rec[0], (void **) &head.node.next, A, B, URCU_TXN_SW_LIST_PROXY_TAG);	/* head->next */
	urcu_txn_sw__record_set(&rec[1], (void **) &B->prev, A, &head.node, URCU_TXN_SW_LIST_PROXY_TAG);	/* B->prev */
	urcu_txn_sw__record_install(&group, &rec[0]);	/* selector 0 => old */
	urcu_txn_sw__record_install(&group, &rec[1]);

	rcu_read_lock();
	ok(urcu_txn_sw_list_next_rcu(&head.node) == A,
		"install: forward resolves to old (A still present)");
	ok(urcu_txn_sw_list_prev_rcu(B) == A,
		"install: backward resolves to old (A still present)");
	rcu_read_unlock();

	urcu_txn_sw__group_commit(&group);	/* one flip switches both edges */

	rcu_read_lock();
	ok(urcu_txn_sw_list_next_rcu(&head.node) == B,
		"commit: forward resolves to new (A removed)");
	ok(urcu_txn_sw_list_prev_rcu(B) == &head.node,
		"commit: backward resolves to new (A removed)");
	rcu_read_unlock();

	/* No reader holds a proxy here: the records need no grace period. */
	uatomic_store(rec[0].slot, rec[0].ptr[1], CMM_RELEASE);
	uatomic_store(rec[1].slot, rec[1].ptr[1], CMM_RELEASE);
	ok(head.node.next == B && B->prev == &head.node,
		"settle: slots hold the direct new targets");

	call_rcu(&a->rcu_head, bl_node_free);		/* A is now a ghost */
	destroy_list(&head);				/* frees B */
}

/*
 * Composition on ONE list, with neighbourhoods that TOUCH -- the case the
 * _prepare forms' read-your-own-writes loads exist for.  Recording these edges
 * from raw neighbour reads (as this header did before) publishes deleted nodes:
 * see the trap worked out in urcu_txn_sw_list_add_after_prepare().
 */

/* Delete @a and @b -- adjacent -- in ONE flip. */
static void test_compose_adjacent_del(void)
{
	URCU_TXN_SW_LIST_HEAD(head);
	const int keys[5] = { 1, 2, 3, 4, 5 };
	const int want_fwd[3] = { 1, 4, 5 };
	const int want_rev[3] = { 5, 4, 1 };
	struct urcu_txn_sw_list_node *e2, *e3;
	struct bl_node *n2, *n3;
	struct urcu_txn_sw_txn t;
	int i, fwd[3], rev[3], n, st;

	for (i = 0; i < 5; i++)
		urcu_txn_sw_list_add_tail_rcu(&bl_node_new(keys[i])->node, &head);
	e2 = find_key(&head, 2);
	e3 = find_key(&head, 3);
	n2 = caa_container_of(e2, struct bl_node, node);
	n3 = caa_container_of(e3, struct bl_node, node);

	urcu_txn_sw_init(&t);
	(void) urcu_txn_sw_list_del_prepare(&t, e2);
	(void) urcu_txn_sw_list_del_prepare(&t, e3);	/* neighbour of e2 */
	st = urcu_txn_sw_commit(&t) == URCU_TXN_STATUS_OK;
	call_rcu(&n2->rcu_head, bl_node_free);
	call_rcu(&n3->rcu_head, bl_node_free);

	n = collect_forward(&head, fwd, 3);
	ok(st && n == 3 && arr_eq(fwd, want_fwd, 3),
		"compose adjacent del: forward is 1,4,5");
	n = collect_reverse(&head, rev, 3);
	ok(n == 3 && arr_eq(rev, want_rev, 3),
		"compose adjacent del: reverse is 5,4,1");
	ok(check_inverses(&head), "compose adjacent del: ring stays coherent");
	destroy_list(&head);
}

/* Delete @b and insert a fresh node into the SAME gap, in ONE flip: both edges
 * of the insert chain onto records the delete already made. */
static void test_compose_del_and_add(void)
{
	URCU_TXN_SW_LIST_HEAD(head);
	const int keys[3] = { 1, 2, 3 };
	const int want_fwd[3] = { 1, 9, 3 };
	const int want_rev[3] = { 3, 9, 1 };
	struct urcu_txn_sw_list_node *e1, *e2;
	struct bl_node *n2;
	struct urcu_txn_sw_txn t;
	int i, fwd[3], rev[3], n, st;

	for (i = 0; i < 3; i++)
		urcu_txn_sw_list_add_tail_rcu(&bl_node_new(keys[i])->node, &head);
	e1 = find_key(&head, 1);
	e2 = find_key(&head, 2);
	n2 = caa_container_of(e2, struct bl_node, node);

	urcu_txn_sw_init(&t);
	(void) urcu_txn_sw_list_del_prepare(&t, e2);
	(void) urcu_txn_sw_list_add_after_prepare(&t, &bl_node_new(9)->node, e1);
	st = urcu_txn_sw_commit(&t) == URCU_TXN_STATUS_OK;
	call_rcu(&n2->rcu_head, bl_node_free);

	n = collect_forward(&head, fwd, 3);
	ok(st && n == 3 && arr_eq(fwd, want_fwd, 3),
		"compose del+add in one gap: forward is 1,9,3");
	n = collect_reverse(&head, rev, 3);
	ok(n == 3 && arr_eq(rev, want_rev, 3),
		"compose del+add in one gap: reverse is 3,9,1");
	ok(check_inverses(&head), "compose del+add in one gap: ring stays coherent");
	destroy_list(&head);
}

/* Three consecutive deletes in one flip: &prev->next is chained twice. */
static void test_compose_triple_del(void)
{
	URCU_TXN_SW_LIST_HEAD(head);
	const int keys[5] = { 1, 2, 3, 4, 5 };
	const int want_fwd[2] = { 1, 5 };
	struct urcu_txn_sw_list_node *e[3];
	struct bl_node *nn[3];
	struct urcu_txn_sw_txn t;
	int i, fwd[2], n, st;

	for (i = 0; i < 5; i++)
		urcu_txn_sw_list_add_tail_rcu(&bl_node_new(keys[i])->node, &head);
	for (i = 0; i < 3; i++) {
		e[i] = find_key(&head, i + 2);		/* 2, 3, 4 */
		nn[i] = caa_container_of(e[i], struct bl_node, node);
	}

	urcu_txn_sw_init(&t);
	for (i = 0; i < 3; i++)
		(void) urcu_txn_sw_list_del_prepare(&t, e[i]);
	st = urcu_txn_sw_commit(&t) == URCU_TXN_STATUS_OK;
	for (i = 0; i < 3; i++)
		call_rcu(&nn[i]->rcu_head, bl_node_free);

	n = collect_forward(&head, fwd, 2);
	ok(st && n == 2 && arr_eq(fwd, want_fwd, 2),
		"compose triple del: forward is 1,5");
	ok(check_inverses(&head), "compose triple del: ring stays coherent");
	destroy_list(&head);
}

/* Check both directions of @head against @want, and the ring's coherence. */
static void expect_order(struct urcu_txn_sw_list_head *head, const int *want,
		int nr, const char *what)
{
	int fwd[8], rev[8], i, n;

	n = collect_forward(head, fwd, 8);
	ok(n == nr && arr_eq(fwd, want, nr), "%s: forward order", what);
	n = collect_reverse(head, rev, 8);
	for (i = 0; i < nr && n == nr; i++)
		if (rev[i] != want[nr - 1 - i])
			n = -1;
	ok(n == nr, "%s: reverse order", what);
	ok(check_inverses(head), "%s: ring stays coherent", what);
}

/* The moves of a node, in a transaction of their own. */
static void test_move(void)
{
	URCU_TXN_SW_LIST_HEAD(head);
	const int want_front[5] = { 4, 1, 2, 3, 5 };
	const int want_tail[5] = { 1, 2, 3, 5, 4 };
	const int want_next[5] = { 1, 3, 2, 5, 4 };
	const int want_back[5] = { 1, 2, 3, 5, 4 };
	struct urcu_txn_sw_list_node *e1, *e2, *e3, *e4;
	struct urcu_txn_sw_txn t;
	int i;

	for (i = 1; i <= 5; i++)
		urcu_txn_sw_list_add_tail_rcu(&bl_node_new(i)->node, &head);
	e1 = find_key(&head, 1);
	e2 = find_key(&head, 2);
	e3 = find_key(&head, 3);
	e4 = find_key(&head, 4);

	ok(urcu_txn_sw_list_move_rcu(e4, &head) == 0, "move to the front");
	expect_order(&head, want_front, 5, "move to the front");
	ok(urcu_txn_sw_list_move_tail_rcu(e4, &head) == 0, "move to the tail");
	expect_order(&head, want_tail, 5, "move to the tail");
	/* Past its next, then back before it: the neighbourhoods overlap. */
	ok(urcu_txn_sw_list_move_after_rcu(e2, e3) == 0, "move after the next");
	expect_order(&head, want_next, 5, "move after the next");
	ok(urcu_txn_sw_list_move_before_rcu(e2, e3) == 0, "move before the prev");
	expect_order(&head, want_back, 5, "move before the prev");

	/* Where it already is: nothing is recorded. */
	urcu_txn_sw_init(&t);
	(void) urcu_txn_sw_list_move_after_prepare(&t, e2, e1);
	(void) urcu_txn_sw_list_move_before_prepare(&t, e1, e2);
	ok(t.nr == 0, "a move to where a node already is records nothing");
	urcu_txn_sw_cancel(&t);
	destroy_list(&head);
}

/*
 * Drive the six-edge flip of a move by hand: a reader standing on the moved
 * node, or next to it, sees the old list until the flip and the new one after,
 * the node's own links included.
 *
 * Ring: head <-> A <-> B <-> C <-> D <-> head.  Move D after A.
 */
static void test_move_phases(void)
{
	URCU_TXN_SW_LIST_HEAD(head);
	struct urcu_txn_sw_list_node *n[4], *H = &head.node;
	struct urcu_txn_sw_list_node *A, *B, *C, *D;
	struct urcu_txn_sw_txn t;
	unsigned int i;

	for (i = 0; i < 4; i++) {
		n[i] = &bl_node_new(i + 1)->node;
		urcu_txn_sw_list_add_tail_rcu(n[i], &head);
	}
	A = n[0];
	B = n[1];
	C = n[2];
	D = n[3];

	urcu_txn_sw_init(&t);
	ok(urcu_txn_sw_reserve(&t, 6), "move phases: reserve");
	(void) urcu_txn_sw_list_move_after_prepare(&t, D, A);
	ok(t.nr == 6, "move phases: six edges recorded");
	for (i = 0; i < t.nr; i++)
		urcu_txn_sw__record_install(&t.desc->group, &t.records[i]);

	rcu_read_lock();
	ok(urcu_txn_sw_list_next_rcu(A) == B && urcu_txn_sw_list_prev_rcu(B) == A,
		"install: A and B still linked");
	ok(urcu_txn_sw_list_next_rcu(C) == D && urcu_txn_sw_list_prev_rcu(H) == D,
		"install: D still between C and head");
	ok(urcu_txn_sw_list_next_rcu(D) == H && urcu_txn_sw_list_prev_rcu(D) == C,
		"install: D's own links are the old ones");
	rcu_read_unlock();

	urcu_txn_sw__group_commit(&t.desc->group);

	rcu_read_lock();
	ok(urcu_txn_sw_list_next_rcu(A) == D && urcu_txn_sw_list_prev_rcu(B) == D,
		"commit: D between A and B");
	ok(urcu_txn_sw_list_next_rcu(C) == H && urcu_txn_sw_list_prev_rcu(H) == C,
		"commit: C and head linked");
	ok(urcu_txn_sw_list_next_rcu(D) == B && urcu_txn_sw_list_prev_rcu(D) == A,
		"commit: D's own links are the new ones");
	rcu_read_unlock();

	/* No reader holds a proxy here: the records need no grace period. */
	for (i = 0; i < t.nr; i++)
		uatomic_store(t.records[i].slot, t.records[i].ptr[1], CMM_RELEASE);
	urcu_txn_sw__desc_free(t.desc);
	urcu_txn_sw__done(&t);
	{
		const int want[4] = { 1, 4, 2, 3 };

		expect_order(&head, want, 4, "move phases: settled");
	}
	destroy_list(&head);
}

/* Two moves to the front in one transaction: their edges chain. */
static void test_compose_moves(void)
{
	URCU_TXN_SW_LIST_HEAD(head);
	const int want[4] = { 4, 3, 1, 2 };
	struct urcu_txn_sw_txn t;
	int i, st;

	for (i = 1; i <= 4; i++)
		urcu_txn_sw_list_add_tail_rcu(&bl_node_new(i)->node, &head);
	urcu_txn_sw_init(&t);
	(void) urcu_txn_sw_list_move_after_prepare(&t, find_key(&head, 3),
			&head.node);
	(void) urcu_txn_sw_list_move_after_prepare(&t, find_key(&head, 4),
			&head.node);
	st = urcu_txn_sw_commit(&t) == URCU_TXN_STATUS_OK;
	ok(st, "compose two moves: commit");
	expect_order(&head, want, 4, "compose two moves");
	destroy_list(&head);
}

int main(void)
{
	int err;

	err = create_all_cpu_call_rcu_data(0);
	if (err)
		diag("Per-CPU call_rcu() workers unavailable, using default.");

	rcu_register_thread();
	plan_tests(NR_TESTS);

	test_empty();
	test_add_head();
	test_add_tail();
	test_coherence();
	test_del_middle();
	test_del_ends();
	test_replace();
	test_proxy_phases();
	test_compose_adjacent_del();
	test_compose_del_and_add();
	test_compose_triple_del();
	test_move();
	test_move_phases();
	test_compose_moves();

	rcu_barrier();			/* drain proxy + node reclaim callbacks */
	rcu_unregister_thread();
	free_all_cpu_call_rcu_data();
	return exit_status();
}
