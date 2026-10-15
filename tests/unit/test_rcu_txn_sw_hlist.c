// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

/*
 * Deterministic unit tests for <urcu/rcu-txn-sw-hlist.h>, the single-writer
 * kernel-hlist-shaped RCU list.  Validates that:
 *   - forward iteration matches the model after add_head / add_after /
 *     add_before / del / replace;
 *   - the writer-only pprev chain stays coherent (every node's pprev points to
 *     the slot that points to it) -- the backward bookkeeping a reader never
 *     sees but every structural op depends on;
 *   - edits COMPOSE on one bucket: adjacent deletes, a delete folded with an
 *     insert into the same gap, and a run of deletes all commit correctly in a
 *     single flip.  The _prepare forms read both the forward slots and pprev
 *     through the engine's read-your-own-writes load and chain onto earlier
 *     records; a read of a committed pprev instead re-points a slot inside an
 *     already-unlinked node (see urcu_txn_sw_hlist_del_prepare());
 *   - a composed transaction that is cancelled, or that runs out of memory
 *     midway, leaves the forward chain and every pprev as they were.
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

#define NR_TESTS	29

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
 * Walk the bucket and verify each node's pprev points to the slot that points
 * to it: &head->first for the first node, &prev->next thereafter.  This is the
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

/*
 * Insert 9 after 1, then delete 1 -- in that ORDER, so the fresh node inherits
 * &1->next as the slot that points to it, and the very next op ghosts the node
 * that slot lives in.  add_after must have recorded 2's pprev as &9->next (NOT
 * left it at the doomed &1->next), and del(1) must then re-point head->first to
 * the PENDING 9 rather than the committed 2, and record 9's pprev as
 * &head->first.
 *
 * Note the division of labour between the two assertions: a wrong pprev for 2
 * leaves this bracket's FORWARD order intact, so it is check_pprev() alone that
 * catches it here.  The damage would otherwise stay latent until a later
 * structural op followed the stale pprev into the ghosted node -- which is the
 * whole reason the backward chain is asserted separately rather than inferred
 * from a correct-looking walk.
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
	(void) urcu_txn_sw_hlist_add_after_prepare(&t, &hn_new(9)->node, e1);
	(void) urcu_txn_sw_hlist_del_prepare(&t, e1);	/* 9's slot is in 1 */
	st = urcu_txn_sw_commit(&t) == URCU_TXN_STATUS_OK;
	call_rcu(&n1->rcu_head, hn_free);

	n = collect_forward(&head, fwd, 3);
	ok(st && n == 3 && arr_eq(fwd, want, 3),
		"compose add_after then del its host: forward is 9,2,3");
	ok(check_pprev(&head), "compose add_after then del its host: pprev coherent");
	destroy(&head);
}

/*
 * Delete the first node, then insert before its successor -- add_before loads
 * 2's pprev as the transaction leaves it, &head->first, which del(1) recorded.
 * The committed &1->next would publish head->first == 1.
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
	(void) urcu_txn_sw_hlist_del_prepare(&t, e1);
	(void) urcu_txn_sw_hlist_add_before_prepare(&t, &hn_new(9)->node, e2);
	st = urcu_txn_sw_commit(&t) == URCU_TXN_STATUS_OK;
	call_rcu(&n1->rcu_head, hn_free);

	n = collect_forward(&head, fwd, 3);
	ok(st && n == 3 && arr_eq(fwd, want, 3),
		"compose del then add_before its successor: forward is 9,2,3");
	ok(check_pprev(&head), "compose del then add_before its successor: pprev coherent");
	destroy(&head);
}

/*
 * One _prepare alone, on a handle of its own: an update with a successor
 * records its forward edge and the successor's pprev, and commits through the
 * descriptor.
 */
static void test_prepare_alone(void)
{
	struct urcu_txn_sw_hlist_head head = URCU_TXN_SW_HLIST_HEAD_INIT;
	const int keys[3] = { 1, 2, 3 };
	const int want[2] = { 1, 3 };
	struct urcu_txn_sw_hlist_node *e2;
	struct urcu_txn_sw_txn t;
	struct hn *n2;
	int fwd[2], n, st;

	build_seq(&head, keys, 3);
	e2 = find_key(&head, 2);
	n2 = caa_container_of(e2, struct hn, node);

	urcu_txn_sw_init(&t);
	(void) urcu_txn_sw_hlist_del_prepare(&t, e2);
	st = t.nr == 2;
	st = st && urcu_txn_sw_commit(&t) == URCU_TXN_STATUS_OK;
	call_rcu(&n2->rcu_head, hn_free);

	n = collect_forward(&head, fwd, 2);
	ok(st && n == 2 && arr_eq(fwd, want, 2) && check_pprev(&head),
		"del_prepare alone: two edges, forward is 1,3, pprev coherent");
	destroy(&head);
}

/*
 * Every kind of edit in one transaction over 1..5: del(2), add_after(9, 1),
 * replace(4, 44), add_before(8, 5).  The last re-points the slot that the
 * replace made 5's, inside the fresh node 44, and so loads 5's pending pprev
 * to find it.
 */
static void prepare_mixed(struct urcu_txn_sw_txn *t,
		struct urcu_txn_sw_hlist_head *head, struct hn **fresh)
{
	struct urcu_txn_sw_hlist_node *e1 = find_key(head, 1);
	struct urcu_txn_sw_hlist_node *e2 = find_key(head, 2);
	struct urcu_txn_sw_hlist_node *e4 = find_key(head, 4);
	struct urcu_txn_sw_hlist_node *e5 = find_key(head, 5);

	fresh[0] = hn_new(9);
	fresh[1] = hn_new(44);
	fresh[2] = hn_new(8);
	(void) urcu_txn_sw_hlist_del_prepare(t, e2);
	(void) urcu_txn_sw_hlist_add_after_prepare(t, &fresh[0]->node, e1);
	(void) urcu_txn_sw_hlist_replace_prepare(t, e4, &fresh[1]->node);
	(void) urcu_txn_sw_hlist_add_before_prepare(t, &fresh[2]->node, e5);
}

static void test_compose_mixed(void)
{
	struct urcu_txn_sw_hlist_head head = URCU_TXN_SW_HLIST_HEAD_INIT;
	const int keys[5] = { 1, 2, 3, 4, 5 };
	const int want[6] = { 1, 9, 3, 44, 8, 5 };
	struct urcu_txn_sw_hlist_node *e2, *e4;
	struct urcu_txn_sw_txn t;
	struct hn *fresh[3];
	int fwd[6], n, st;

	build_seq(&head, keys, 5);
	e2 = find_key(&head, 2);
	e4 = find_key(&head, 4);

	urcu_txn_sw_init(&t);
	prepare_mixed(&t, &head, fresh);
	st = urcu_txn_sw_commit(&t) == URCU_TXN_STATUS_OK;
	call_rcu(&caa_container_of(e2, struct hn, node)->rcu_head, hn_free);
	call_rcu(&caa_container_of(e4, struct hn, node)->rcu_head, hn_free);

	n = collect_forward(&head, fwd, 6);
	ok(st && n == 6 && arr_eq(fwd, want, 6),
		"compose del, add_after, replace, add_before: forward is 1,9,3,44,8,5");
	ok(check_pprev(&head), "compose del, add_after, replace, add_before: pprev coherent");
	destroy(&head);
}

/*
 * The same transaction, cancelled: nothing was written outside it, so the chain
 * and every pprev are as they were, and the fresh nodes, never published, are
 * freed at once.  A later single update then finds the pprev it relies on.
 */
static void test_cancel_composed(void)
{
	struct urcu_txn_sw_hlist_head head = URCU_TXN_SW_HLIST_HEAD_INIT;
	const int keys[5] = { 1, 2, 3, 4, 5 };
	const int want[4] = { 1, 2, 4, 5 };
	struct urcu_txn_sw_hlist_node *e3;
	struct urcu_txn_sw_txn t;
	struct hn *fresh[3];
	int fwd[5], n, i;

	build_seq(&head, keys, 5);

	urcu_txn_sw_init(&t);
	prepare_mixed(&t, &head, fresh);
	urcu_txn_sw_cancel(&t);
	for (i = 0; i < 3; i++)
		free(fresh[i]);

	n = collect_forward(&head, fwd, 5);
	ok(n == 5 && arr_eq(fwd, keys, 5) && check_pprev(&head),
		"cancel of a composed transaction: forward is 1..5, pprev coherent");

	e3 = find_key(&head, 3);
	urcu_txn_sw_hlist_del_rcu(e3);
	call_rcu(&caa_container_of(e3, struct hn, node)->rcu_head, hn_free);
	n = collect_forward(&head, fwd, 5);
	ok(n == 4 && arr_eq(fwd, want, 4) && check_pprev(&head),
		"del_rcu after the cancel: forward is 1,2,4,5, pprev coherent");
	destroy(&head);
}

/*
 * The same transaction, out of memory after its first two edits: the sticky
 * state is set by hand, as a failed record() leaves it.  commit() reports it
 * and publishes nothing, and the edits recorded before the failure leave no
 * pprev behind either.
 */
static void test_oom_composed(void)
{
	struct urcu_txn_sw_hlist_head head = URCU_TXN_SW_HLIST_HEAD_INIT;
	const int keys[5] = { 1, 2, 3, 4, 5 };
	struct urcu_txn_sw_hlist_node *e1, *e2, *e4;
	struct urcu_txn_sw_txn t;
	struct hn *n9, *n44;
	int fwd[5], n, st;

	build_seq(&head, keys, 5);
	e1 = find_key(&head, 1);
	e2 = find_key(&head, 2);
	e4 = find_key(&head, 4);
	n9 = hn_new(9);
	n44 = hn_new(44);

	urcu_txn_sw_init(&t);
	(void) urcu_txn_sw_hlist_del_prepare(&t, e2);
	(void) urcu_txn_sw_hlist_add_after_prepare(&t, &n9->node, e1);
	t.state = URCU_TXN_SW_OOM;
	(void) urcu_txn_sw_hlist_replace_prepare(&t, e4, &n44->node);
	st = urcu_txn_sw_commit(&t);
	free(n9);
	free(n44);

	ok(st == URCU_TXN_STATUS_MEMORY_ERROR,
		"out of memory midway: commit reports MEMORY_ERROR");
	n = collect_forward(&head, fwd, 5);
	ok(n == 5 && arr_eq(fwd, keys, 5) && check_pprev(&head),
		"out of memory midway: forward is 1..5, pprev coherent");
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
	test_prepare_alone();
	test_compose_mixed();
	test_cancel_composed();
	test_oom_composed();

	rcu_barrier();			/* drain proxy + node reclaim callbacks */
	rcu_unregister_thread();
	free_all_cpu_call_rcu_data();
	return exit_status();
}
