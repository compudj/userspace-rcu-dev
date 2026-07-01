// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

/*
 * Deterministic unit tests for <urcu/rcu-txn-sw-list.h>, the bidirectional
 * RCU list.  Validates that:
 *   - forward and reverse iteration are exact mirrors;
 *   - next/prev are mutual inverses around the whole ring (coherence);
 *   - add / add_tail / del / replace keep both directions coherent;
 *   - the proxy-flip resolves correctly across the
 *     install -> commit -> settle phases of a two-edge flip (the
 *     mechanism a concurrent reader relies on).
 *
 * QSBR flavor, single threaded, so the outcome is fully deterministic.
 */

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <stdlib.h>
#include <string.h>

#include <urcu/compiler.h>
#include <urcu-qsbr.h>
#include <urcu-call-rcu.h>
#include <urcu/rcu-txn-sw-list.h>

#include "tap.h"

#define NR_TESTS	22

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
	struct urcu_txn_sw_txn _txn, *txn = &_txn;

	urcu_txn_sw_list_add_tail_rcu(A, &head);
	urcu_txn_sw_list_add_tail_rcu(B, &head);

	/*
	 * Delete A through the transaction by hand, pausing between the
	 * explicit install and the commit to observe the reader's view of
	 * each phase -- the same install -> commit -> settle the mutators run.
	 */
	urcu_txn_sw_init(txn);
	if (!urcu_txn_sw_reserve(txn, 2))
		abort();
	urcu_txn_sw_record(txn, (void **) &head.node.next, A, B, URCU_TXN_SW_LIST_PROXY_TAG);	/* head->next */
	urcu_txn_sw_record(txn, (void **) &B->prev, A, &head.node, URCU_TXN_SW_LIST_PROXY_TAG);	/* B->prev */
	urcu_txn_sw_install(txn);		/* park proxies; selector 0 => old */

	rcu_read_lock();
	ok(urcu_txn_sw_list_next_rcu(&head.node) == A,
		"install: forward resolves to old (A still present)");
	ok(urcu_txn_sw_list_prev_rcu(B) == A,
		"install: backward resolves to old (A still present)");
	rcu_read_unlock();

	/* Explicit install parked proxies, so commit owns reclaim (call_rcu). */
	(void) urcu_txn_sw_commit(txn);	/* one flip switches both edges */

	rcu_read_lock();
	ok(urcu_txn_sw_list_next_rcu(&head.node) == B,
		"commit: forward resolves to new (A removed)");
	ok(urcu_txn_sw_list_prev_rcu(B) == &head.node,
		"commit: backward resolves to new (A removed)");
	rcu_read_unlock();

	ok(head.node.next == B && B->prev == &head.node,
		"settle: slots hold the direct new targets");

	call_rcu(&a->rcu_head, bl_node_free);		/* A is now a ghost */
	destroy_list(&head);				/* frees B */
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

	rcu_barrier();			/* drain proxy + node reclaim callbacks */
	rcu_unregister_thread();
	free_all_cpu_call_rcu_data();
	return exit_status();
}
