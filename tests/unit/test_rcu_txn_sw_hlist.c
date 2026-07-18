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
 *     single flip.  Two mechanisms carry this, and check_pprev() is what would
 *     catch either regressing: the forward slots are read through the engine's
 *     read-your-own-writes load and chained, while pprev is plain-stored EAGERLY
 *     so a later _prepare's raw read of it already sees the earlier edit.  Defer
 *     that store and a composed delete re-points a slot inside an
 *     already-unlinked node (see urcu_txn_sw_hlist_del_prepare()).
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

#define NR_TESTS	22

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
	st = urcu_txn_sw_reserve(&t, 2);	/* up front: pprev stores do not roll back */
	(void) urcu_txn_sw_hlist_del_prepare(&t, e2);
	(void) urcu_txn_sw_hlist_del_prepare(&t, e3);	/* neighbour of e2 */
	st = st && urcu_txn_sw_commit(&t) == URCU_TXN_STATUS_OK;
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
	st = urcu_txn_sw_reserve(&t, 2);	/* up front: pprev stores do not roll back */
	(void) urcu_txn_sw_hlist_del_prepare(&t, e2);
	(void) urcu_txn_sw_hlist_add_after_prepare(&t, &hn_new(9)->node, e1);
	st = st && urcu_txn_sw_commit(&t) == URCU_TXN_STATUS_OK;
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
	st = urcu_txn_sw_reserve(&t, 3);	/* up front: pprev stores do not roll back */
	for (i = 0; i < 3; i++)
		(void) urcu_txn_sw_hlist_del_prepare(&t, e[i]);
	st = st && urcu_txn_sw_commit(&t) == URCU_TXN_STATUS_OK;
	for (i = 0; i < 3; i++)
		call_rcu(&nn[i]->rcu_head, hn_free);

	n = collect_forward(&head, fwd, 2);
	ok(st && n == 2 && arr_eq(fwd, want, 2),
		"compose triple del: forward is 1,5");
	ok(check_pprev(&head), "compose triple del: pprev coherent");
	destroy(&head);
}

/*
 * Insert 9 after 1, then delete 1 -- in that ORDER, so the fresh node inherits
 * &1->next as its naming slot and the very next op ghosts the node that slot
 * lives in.  add_after must have plain-stored 2->pprev to &9->next (NOT left it
 * at the doomed &1->next), and del(1) must then re-point head->first to the
 * PENDING 9 rather than the committed 2.
 *
 * Note the division of labour between the two assertions: mis-storing 2->pprev
 * leaves this bracket's FORWARD order intact (verified by injecting exactly that
 * bug), so it is check_pprev() alone that catches it here.  The damage would
 * otherwise stay latent until a later structural op followed the stale pprev
 * into the ghosted node -- which is the whole reason the backward chain is
 * asserted separately rather than inferred from a correct-looking walk.
 */
static void test_compose_add_then_del_host(void)
{
	struct urcu_txn_sw_hlist_head head = URCU_TXN_SW_HLIST_HEAD_INIT;
	const int keys[3] = { 1, 2, 3 };
	const int want[3] = { 9, 2, 3 };
	struct urcu_txn_sw_hlist_node *e1;
	struct hn *n1;
	struct urcu_txn_sw_txn t;
	int fwd[3], n, st;

	build_seq(&head, keys, 3);
	e1 = find_key(&head, 1);
	n1 = caa_container_of(e1, struct hn, node);

	urcu_txn_sw_init(&t);
	st = urcu_txn_sw_reserve(&t, 2);	/* up front: pprev stores do not roll back */
	(void) urcu_txn_sw_hlist_add_after_prepare(&t, &hn_new(9)->node, e1);
	(void) urcu_txn_sw_hlist_del_prepare(&t, e1);	/* ghosts 9's naming slot's host */
	st = st && urcu_txn_sw_commit(&t) == URCU_TXN_STATUS_OK;
	call_rcu(&n1->rcu_head, hn_free);

	n = collect_forward(&head, fwd, 3);
	ok(st && n == 3 && arr_eq(fwd, want, 3),
		"compose add_after then del its host: forward is 9,2,3");
	ok(check_pprev(&head), "compose add_after then del its host: pprev coherent");
	destroy(&head);
}

/*
 * Delete the first node, then insert before its successor -- add_before reads
 * 2->pprev RAW, so it sees &head->first only because del(1) eagerly stored it
 * there.  A deferred store would leave &1->next and publish head->first == 1.
 */
static void test_compose_del_then_add_before(void)
{
	struct urcu_txn_sw_hlist_head head = URCU_TXN_SW_HLIST_HEAD_INIT;
	const int keys[3] = { 1, 2, 3 };
	const int want[3] = { 9, 2, 3 };
	struct urcu_txn_sw_hlist_node *e1, *e2;
	struct hn *n1;
	struct urcu_txn_sw_txn t;
	int fwd[3], n, st;

	build_seq(&head, keys, 3);
	e1 = find_key(&head, 1);
	e2 = find_key(&head, 2);
	n1 = caa_container_of(e1, struct hn, node);

	urcu_txn_sw_init(&t);
	st = urcu_txn_sw_reserve(&t, 2);	/* up front: pprev stores do not roll back */
	(void) urcu_txn_sw_hlist_del_prepare(&t, e1);
	(void) urcu_txn_sw_hlist_add_before_prepare(&t, &hn_new(9)->node, e2);
	st = st && urcu_txn_sw_commit(&t) == URCU_TXN_STATUS_OK;
	call_rcu(&n1->rcu_head, hn_free);

	n = collect_forward(&head, fwd, 3);
	ok(st && n == 3 && arr_eq(fwd, want, 3),
		"compose del then add_before its successor: forward is 9,2,3");
	ok(check_pprev(&head), "compose del then add_before its successor: pprev coherent");
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
	test_compose_add_then_del_host();
	test_compose_del_then_add_before();

	rcu_barrier();			/* drain proxy + node reclaim callbacks */
	rcu_unregister_thread();
	free_all_cpu_call_rcu_data();
	return exit_status();
}
