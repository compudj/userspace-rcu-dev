// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

/*
 * Deterministic unit tests for <urcu/rcu-txn-sw-hlist.h>, the single-updater
 * kernel-hlist-shaped RCU list.  Validates that:
 *   - forward iteration matches the model after add_head / add_after /
 *     add_before / del / replace;
 *   - the writer-only pprev chain stays coherent (every node's pprev names the
 *     slot that holds it) -- the backward bookkeeping a reader never sees but
 *     every structural op depends on;
 *   - edits COMPOSE on one bucket: adjacent deletes, a delete folded with an
 *     insert into the same gap, and a run of deletes all commit correctly in a
 *     single flip.  This is the read-your-own-writes path the _prepare forms
 *     gained; recording from raw neighbour reads instead publishes deleted
 *     nodes (see urcu_txn_sw_hlist_del_prepare()).
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
#include <urcu/rcu-txn-sw-hlist.h>

#include "tap.h"

#define NR_TESTS	18

struct hn {
	struct urcu_txn_sw_hlist_node node;
	struct rcu_head rcu_head;
	int key;
};

static void hn_free(struct rcu_head *head)
{
	free(caa_container_of(head, struct hn, rcu_head));
}

static struct hn *hn_new(int key)
{
	struct hn *n = (struct hn *) malloc(sizeof(*n));

	if (!n)
		abort();
	n->key = key;
	return n;
}

/* Collect forward keys.  Returns count, fills buf (up to max). */
static int collect_forward(struct urcu_txn_sw_hlist_head *head, int *buf, int max)
{
	struct urcu_txn_sw_hlist_node *p;
	int n = 0;

	rcu_read_lock();
	urcu_txn_sw_hlist_for_each_rcu(p, head) {
		if (n < max)
			buf[n] = caa_container_of(p, struct hn, node)->key;
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
 * Walk the bucket and verify each node's pprev names the slot that holds it:
 * &head->first for the first node, &prev->next thereafter.  This is the
 * backward invariant every del/add_before/replace relies on; a torn pprev is
 * invisible to forward iteration but breaks the next structural op.
 */
static int check_pprev(struct urcu_txn_sw_hlist_head *head)
{
	struct urcu_txn_sw_hlist_node **slot = &head->first;
	struct urcu_txn_sw_hlist_node *p;
	int ok = 1;

	rcu_read_lock();
	for (p = urcu_txn_sw_hlist_first_rcu(head); p != NULL; ) {
		if (p->pprev != slot) {
			ok = 0;
			break;
		}
		slot = &p->next;
		p = urcu_txn_sw_hlist_next_rcu(p);
	}
	rcu_read_unlock();
	return ok;
}

static struct urcu_txn_sw_hlist_node *find_key(
		struct urcu_txn_sw_hlist_head *head, int key)
{
	struct urcu_txn_sw_hlist_node *p;

	urcu_txn_sw_hlist_for_each_rcu(p, head) {
		if (caa_container_of(p, struct hn, node)->key == key)
			return p;
	}
	return NULL;
}

static void destroy(struct urcu_txn_sw_hlist_head *head)
{
	while (!urcu_txn_sw_hlist_empty(head)) {
		struct urcu_txn_sw_hlist_node *p =
				urcu_txn_sw_hlist_first_rcu(head);
		struct hn *n = caa_container_of(p, struct hn, node);

		urcu_txn_sw_hlist_del_rcu(p);
		call_rcu(&n->rcu_head, hn_free);
	}
}

/* Build forward 1..n by prepending in reverse. */
static void build_seq(struct urcu_txn_sw_hlist_head *head, const int *keys, int n)
{
	int i;

	for (i = n - 1; i >= 0; i--)
		urcu_txn_sw_hlist_add_head_rcu(&hn_new(keys[i])->node, head);
}

static void test_add_head(void)
{
	struct urcu_txn_sw_hlist_head head = URCU_TXN_SW_HLIST_HEAD_INIT;
	const int keys[4] = { 1, 2, 3, 4 };
	int fwd[4], n;

	ok(urcu_txn_sw_hlist_empty(&head), "fresh hlist is empty");
	build_seq(&head, keys, 4);
	n = collect_forward(&head, fwd, 4);
	ok(n == 4 && arr_eq(fwd, keys, 4) && check_pprev(&head),
		"add_head: forward is 1,2,3,4 and pprev coherent");
	destroy(&head);
}

static void test_add_after(void)
{
	struct urcu_txn_sw_hlist_head head = URCU_TXN_SW_HLIST_HEAD_INIT;
	const int keys[3] = { 1, 2, 3 };
	const int want[4] = { 1, 2, 9, 3 };
	int fwd[4], n;

	build_seq(&head, keys, 3);
	urcu_txn_sw_hlist_add_after_rcu(&hn_new(9)->node, find_key(&head, 2));
	n = collect_forward(&head, fwd, 4);
	ok(n == 4 && arr_eq(fwd, want, 4), "add_after 2: forward is 1,2,9,3");
	ok(check_pprev(&head), "add_after: pprev coherent");
	destroy(&head);
}

static void test_add_before(void)
{
	struct urcu_txn_sw_hlist_head head = URCU_TXN_SW_HLIST_HEAD_INIT;
	const int keys[3] = { 1, 2, 3 };
	const int want[4] = { 1, 9, 2, 3 };
	int fwd[4], n;

	build_seq(&head, keys, 3);
	urcu_txn_sw_hlist_add_before_rcu(&hn_new(9)->node, find_key(&head, 2));
	n = collect_forward(&head, fwd, 4);
	ok(n == 4 && arr_eq(fwd, want, 4), "add_before 2: forward is 1,9,2,3");
	ok(check_pprev(&head), "add_before: pprev coherent");
	destroy(&head);
}

static void test_del_middle(void)
{
	struct urcu_txn_sw_hlist_head head = URCU_TXN_SW_HLIST_HEAD_INIT;
	const int keys[5] = { 1, 2, 3, 4, 5 };
	const int want[4] = { 1, 2, 4, 5 };
	struct urcu_txn_sw_hlist_node *mid;
	struct hn *midn;
	int fwd[4], n;

	build_seq(&head, keys, 5);
	mid = find_key(&head, 3);
	midn = caa_container_of(mid, struct hn, node);
	urcu_txn_sw_hlist_del_rcu(mid);
	call_rcu(&midn->rcu_head, hn_free);
	n = collect_forward(&head, fwd, 4);
	ok(n == 4 && arr_eq(fwd, want, 4), "del middle: forward is 1,2,4,5");
	ok(check_pprev(&head), "del middle: pprev coherent");
	destroy(&head);
}

static void test_del_ends(void)
{
	struct urcu_txn_sw_hlist_head head = URCU_TXN_SW_HLIST_HEAD_INIT;
	const int keys[4] = { 1, 2, 3, 4 };
	const int want[2] = { 2, 3 };
	struct urcu_txn_sw_hlist_node *p;
	struct hn *n;
	int fwd[2], cnt;

	build_seq(&head, keys, 4);
	p = find_key(&head, 1);				/* head */
	n = caa_container_of(p, struct hn, node);
	urcu_txn_sw_hlist_del_rcu(p);
	call_rcu(&n->rcu_head, hn_free);
	p = find_key(&head, 4);				/* tail */
	n = caa_container_of(p, struct hn, node);
	urcu_txn_sw_hlist_del_rcu(p);
	call_rcu(&n->rcu_head, hn_free);
	cnt = collect_forward(&head, fwd, 2);
	ok(cnt == 2 && arr_eq(fwd, want, 2), "del both ends: forward is 2,3");
	ok(check_pprev(&head), "del both ends: pprev coherent");
	destroy(&head);
}

static void test_replace(void)
{
	struct urcu_txn_sw_hlist_head head = URCU_TXN_SW_HLIST_HEAD_INIT;
	const int keys[3] = { 1, 2, 3 };
	const int want[3] = { 1, 22, 3 };
	struct urcu_txn_sw_hlist_node *old;
	struct hn *oldn;
	int fwd[3], n;

	build_seq(&head, keys, 3);
	old = find_key(&head, 2);
	oldn = caa_container_of(old, struct hn, node);
	urcu_txn_sw_hlist_replace_rcu(old, &hn_new(22)->node);
	call_rcu(&oldn->rcu_head, hn_free);
	n = collect_forward(&head, fwd, 3);
	ok(n == 3 && arr_eq(fwd, want, 3), "replace 2 with 22: forward is 1,22,3");
	ok(check_pprev(&head), "replace: pprev coherent");
	destroy(&head);
}

/*
 * Composition on ONE bucket, with neighbourhoods that TOUCH -- the case the
 * _prepare forms' read-your-own-writes loads exist for.  Recording these from
 * raw neighbour reads publishes deleted nodes: see the worked trap in
 * urcu_txn_sw_hlist_del_prepare().
 */

/* Delete adjacent 2 and 3 from 1..5 in ONE flip. */
static void test_compose_adjacent_del(void)
{
	struct urcu_txn_sw_hlist_head head = URCU_TXN_SW_HLIST_HEAD_INIT;
	const int keys[5] = { 1, 2, 3, 4, 5 };
	const int want[3] = { 1, 4, 5 };
	struct urcu_txn_sw_hlist_node *e2, *e3;
	struct hn *n2, *n3;
	struct urcu_txn_sw_txn t;
	int fwd[3], n, st;

	build_seq(&head, keys, 5);
	e2 = find_key(&head, 2);
	e3 = find_key(&head, 3);
	n2 = caa_container_of(e2, struct hn, node);
	n3 = caa_container_of(e3, struct hn, node);

	urcu_txn_sw_init(&t);
	(void) urcu_txn_sw_hlist_del_prepare(&t, e2);
	(void) urcu_txn_sw_hlist_del_prepare(&t, e3);	/* neighbour of e2 */
	st = urcu_txn_sw_commit(&t) == URCU_TXN_STATUS_OK;
	call_rcu(&n2->rcu_head, hn_free);
	call_rcu(&n3->rcu_head, hn_free);

	n = collect_forward(&head, fwd, 3);
	ok(st && n == 3 && arr_eq(fwd, want, 3),
		"compose adjacent del: forward is 1,4,5");
	ok(check_pprev(&head), "compose adjacent del: pprev coherent");
	destroy(&head);
}

/* Delete 2 and insert 9 into the SAME gap (after 1), in ONE flip. */
static void test_compose_del_and_add(void)
{
	struct urcu_txn_sw_hlist_head head = URCU_TXN_SW_HLIST_HEAD_INIT;
	const int keys[3] = { 1, 2, 3 };
	const int want[3] = { 1, 9, 3 };
	struct urcu_txn_sw_hlist_node *e1, *e2;
	struct hn *n2;
	struct urcu_txn_sw_txn t;
	int fwd[3], n, st;

	build_seq(&head, keys, 3);
	e1 = find_key(&head, 1);
	e2 = find_key(&head, 2);
	n2 = caa_container_of(e2, struct hn, node);

	urcu_txn_sw_init(&t);
	(void) urcu_txn_sw_hlist_del_prepare(&t, e2);
	(void) urcu_txn_sw_hlist_add_after_prepare(&t, &hn_new(9)->node, e1);
	st = urcu_txn_sw_commit(&t) == URCU_TXN_STATUS_OK;
	call_rcu(&n2->rcu_head, hn_free);

	n = collect_forward(&head, fwd, 3);
	ok(st && n == 3 && arr_eq(fwd, want, 3),
		"compose del+add in one gap: forward is 1,9,3");
	ok(check_pprev(&head), "compose del+add in one gap: pprev coherent");
	destroy(&head);
}

/* Three consecutive deletes (2,3,4) in one flip: &1->next chained twice. */
static void test_compose_triple_del(void)
{
	struct urcu_txn_sw_hlist_head head = URCU_TXN_SW_HLIST_HEAD_INIT;
	const int keys[5] = { 1, 2, 3, 4, 5 };
	const int want[2] = { 1, 5 };
	struct urcu_txn_sw_hlist_node *e[3];
	struct hn *nn[3];
	struct urcu_txn_sw_txn t;
	int i, fwd[2], n, st;

	build_seq(&head, keys, 5);
	for (i = 0; i < 3; i++) {
		e[i] = find_key(&head, i + 2);		/* 2, 3, 4 */
		nn[i] = caa_container_of(e[i], struct hn, node);
	}

	urcu_txn_sw_init(&t);
	for (i = 0; i < 3; i++)
		(void) urcu_txn_sw_hlist_del_prepare(&t, e[i]);
	st = urcu_txn_sw_commit(&t) == URCU_TXN_STATUS_OK;
	for (i = 0; i < 3; i++)
		call_rcu(&nn[i]->rcu_head, hn_free);

	n = collect_forward(&head, fwd, 2);
	ok(st && n == 2 && arr_eq(fwd, want, 2),
		"compose triple del: forward is 1,5");
	ok(check_pprev(&head), "compose triple del: pprev coherent");
	destroy(&head);
}

int main(void)
{
	int err;

	err = create_all_cpu_call_rcu_data(0);
	if (err)
		diag("Per-CPU call_rcu() workers unavailable, using default.");

	rcu_register_thread();
	plan_tests(NR_TESTS);

	test_add_head();
	test_add_after();
	test_add_before();
	test_del_middle();
	test_del_ends();
	test_replace();
	test_compose_adjacent_del();
	test_compose_del_and_add();
	test_compose_triple_del();

	rcu_barrier();			/* drain proxy + node reclaim callbacks */
	rcu_unregister_thread();
	free_all_cpu_call_rcu_data();
	return exit_status();
}
