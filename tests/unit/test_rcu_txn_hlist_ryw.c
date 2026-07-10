// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

/*
 * Composing several hlist deletes that touch NEIGHBOURING nodes within one
 * transaction, under read-your-own-writes (urcu_txn_enable_ryw).
 *
 * The hlist looks immune to the ordered structures' co-batching hazard -- a head
 * insert writes a fixed, key-independent slot, and a delete writes node-local
 * state.  It is not.  urcu_txn_hlist_del_prepare() reads its WRITE SITE out of
 * the structure:
 *
 *     ppv = urcu_txn_load(txn, &elem->pprev);   // the slot that names elem
 *     ...
 *     urcu_txn_store(txn, ppv, elem, next);     // store THROUGH it
 *
 * That is a one-hop traversal, and a one-hop traversal is still a traversal.
 * Delete two ADJACENT nodes A and B (B == A->next) in one transaction and, with
 * buffered writes invisible, del(B) loads the stale &A->next as its write site --
 * even though del(A) has already recorded that B is renamed to &P->next.  It then
 * stores through &A->next presenting old == B, which MATCHES the record del(A)
 * left there ({B -> MARK(B)}), so the engine's one-record-per-slot upgrade
 * overwrites new_ptr: A's TOMBSTONE IS DESTROYED.  The commit leaves A unlinked
 * but unmarked, B marked yet still linked (so its delete silently did not happen,
 * while the caller reclaims it), and C->pprev pointing INTO the removed A.
 *
 * The interesting part is that the engine fix is enough on its own: this file
 * makes NO change to <urcu/rcu-txn-hlist.h>.  Because del_prepare already routes
 * that load through urcu_txn_load(), enabling RYW retargets it onto &P->next,
 * and the subsequent store CHAINS onto del(A)'s record ({A -> B} becomes
 * {A -> C}).  Both nodes end marked, both unlinked, C correctly renamed.
 *
 * An INSERT composed with a delete aliases too, and shows that the trigger is
 * slot coincidence rather than the insertion end.  insert_head always stores
 * &head->first; del(X) stores *X->pprev, which IS &head->first exactly when X is
 * the bucket's first node.  Compose them and the insert's successor load reads
 * memory, never seeing this txn's pending unlink of X, so both stores present
 * the same old and the upgrade leaves X MARK-ed yet still linked -- its delete
 * lost, and the caller reclaims it.  The victim-position tests below pin that
 * down: FIRST corrupts, MIDDLE and LAST are correct even without RYW.  Inserting
 * at the tail would not cure it; the trigger would move to "victim is last".
 *
 * This is the evidence for the claim that RYW + chaining is a GENERIC engine
 * feature rather than a skiplist repair.  See test_rcu_txn_skiplist_ryw.c.
 */

#define _GNU_SOURCE
#define _LGPL_SOURCE
#include <assert.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>

#include <urcu/compiler.h>
#include <urcu-qsbr.h>
#include <urcu-call-rcu.h>
#include <urcu/rcu-mcas.h>
#include <urcu/rcu-txn-hlist.h>
#include <urcu/rcu-txn.h>

#include "tap.h"

#define NR_TESTS	9
#define SPIN_LIMIT	100000

struct node {
	unsigned long key;
	struct urcu_txn_hlist_node h;
};

static struct urcu_txn_domain g_dom;

static struct node *node_alloc(unsigned long key)
{
	struct node *e = (struct node *) malloc(sizeof(*e));

	if (!e)
		abort();
	e->key = key;
	return e;
}

/* Collect the bucket's keys in list order. Returns the count. */
static int collect(struct urcu_txn_hlist_head *head, unsigned long *out, int max)
{
	struct urcu_txn_hlist_node *n;
	int count = 0;

	for (n = urcu_txn_hlist_first_rcu(head); n != NULL;
			n = urcu_txn_hlist_next_rcu(n)) {
		assert(count < max);
		out[count++] = caa_container_of(n, struct node, h)->key;
	}
	return count;
}

/*
 * Every node reachable in the bucket must be unmarked, and must be named by the
 * slot it is actually reached through.  The second half is what catches
 * C->pprev pointing into a removed node.
 */
static void assert_chain_sane(struct urcu_txn_hlist_head *head)
{
	struct urcu_txn_hlist_node **expect_pprev = &head->first;
	struct urcu_txn_hlist_node *n;

	for (n = urcu_txn_hlist_first_rcu(head); n != NULL;
			n = urcu_txn_hlist_next_rcu(n)) {
		void *v = urcu_mcas_read((void **) &n->next, URCU_TXN_HLIST_TAG);

		assert(!urcu_txn_hlist_is_marked(v));	/* live => untombstoned */
		assert(n->pprev == expect_pprev);	/* named by its real slot */
		expect_pprev = &n->next;
	}
}

/* Is @key still reachable in the bucket? */
static int present(struct urcu_txn_hlist_head *head, unsigned long key)
{
	unsigned long got[16];
	int n = collect(head, got, 16), i;

	for (i = 0; i < n; i++) {
		if (got[i] == key)
			return 1;
	}
	return 0;
}

/*
 * Delete @a and @b (adjacent, a before b) in ONE transaction.  Returns 0,
 * a negative errno from a _prepare, or -ETIMEDOUT on a livelock.
 */
static int del_two(struct urcu_txn_hlist_node *a, struct urcu_txn_hlist_node *b,
		int ryw)
{
	struct urcu_mcas_txn txn;
	long spins = 0;
	enum urcu_txn_status st;

	urcu_txn_init(&txn, &g_dom);
	urcu_txn_set_ryw(&txn, ryw);	/* explicit: ignore URCU_TXN_RYW_DEFAULT */
	for (;;) {
		int prep, retry = 0, err = 0;

		if (++spins > SPIN_LIMIT)
			return -ETIMEDOUT;
		urcu_txn_begin(&txn);
		prep = urcu_txn_hlist_del_prepare(&txn, a);
		if (prep == -EAGAIN)
			retry = 1;
		else if (prep < 0)
			err = prep;
		if (!retry && !err) {
			prep = urcu_txn_hlist_del_prepare(&txn, b);
			if (prep == -EAGAIN)
				retry = 1;
			else if (prep < 0)
				err = prep;
		}
		if (err) {
			urcu_txn_end(&txn);
			return err;
		}
		if (retry) {
			urcu_txn_conflict(&txn);
			urcu_txn_end(&txn);
			continue;
		}
		st = urcu_txn_commit(&txn);
		urcu_txn_end(&txn);
		if (st == URCU_TXN_STATUS_OK)
			return 0;
		if (st == URCU_TXN_STATUS_ABORT)
			continue;
		return -ENOMEM;
	}
}

/* Build bucket {4,3,2,1} (head insert reverses), return the nodes by key. */
static void build(struct urcu_txn_hlist_head *head, struct node **out4)
{
	int i;

	urcu_txn_hlist_init(head);
	for (i = 1; i <= 4; i++) {
		out4[i - 1] = node_alloc((unsigned long) i);
		if (urcu_txn_hlist_add_rcu(&out4[i - 1]->h, head, &g_dom))
			abort();
	}
	/* list order is now 4,3,2,1 */
}

/* --------------------------------------------------------------------- */

static void test_ryw_adjacent_deletes(void)
{
	struct urcu_txn_hlist_head head;
	struct node *n[4];
	unsigned long got[16];
	int cnt, rc;

	build(&head, n);
	/* list: 4,3,2,1 -- delete the adjacent pair (3,2). */
	rc = del_two(&n[2]->h, &n[1]->h, 1);		/* keys 3 then 2 */
	ok(rc == 0, "ryw hlist: co-batched adjacent deletes commit (rc=%d)", rc);

	cnt = collect(&head, got, 16);
	ok(cnt == 2 && got[0] == 4 && got[1] == 1,
			"ryw hlist: both neighbours removed, chain is {4,1}");
	assert_chain_sane(&head);
	ok(1, "ryw hlist: survivors unmarked and named by their real slot");

	free(n[0]); free(n[1]); free(n[2]); free(n[3]);
}

static void test_ryw_whitebox_chain(void)
{
	struct urcu_txn_hlist_head head;
	struct node *n[4];
	struct urcu_mcas_txn txn;
	struct urcu_mcas_record *r;
	int pass;

	build(&head, n);		/* 4,3,2,1 */

	urcu_txn_init(&txn, &g_dom);
	urcu_txn_enable_ryw(&txn);
	urcu_txn_begin(&txn);
	if (urcu_txn_hlist_del_prepare(&txn, &n[2]->h))	/* key 3 */
		abort();
	if (urcu_txn_hlist_del_prepare(&txn, &n[1]->h))	/* key 2, adjacent */
		abort();

	/*
	 * del(3) recorded {&n4->next: n3 -> n2}.  del(2)'s RYW load of n2->pprev
	 * returns the pending &n4->next, so its unlink stores through THAT slot
	 * and chains: the single record must now read {n3 -> n1}, with the
	 * committed old preserved for the commit's check.
	 */
	r = urcu_mcas_find(txn.mcas, (void **) &n[3]->h.next);
	pass = r != NULL && r->old_ptr == (void *) &n[2]->h &&
			r->new_ptr == (void *) &n[0]->h;
	ok(pass, "ryw hlist whitebox: the two unlinks chain to {n(3) -> n(1)} on one slot");

	if (urcu_txn_commit(&txn) != URCU_TXN_STATUS_OK)
		abort();
	urcu_txn_end(&txn);
	assert_chain_sane(&head);

	free(n[0]); free(n[1]); free(n[2]); free(n[3]);
}

static void test_negative_no_ryw(void)
{
	struct urcu_txn_hlist_head head;
	struct node *n[4];
	int rc, corrupted;

	build(&head, n);		/* 4,3,2,1 */
	rc = del_two(&n[2]->h, &n[1]->h, 0);		/* RYW OFF */

	/*
	 * The commit "succeeds" and the caller would now reclaim both nodes, but
	 * key 2 is still reachable: del(2)'s store through the stale &n3->next
	 * destroyed del(3)'s tombstone, so the two unlinks did not compose.  We
	 * only INSPECT here -- reclaiming would be the use-after-free this bug
	 * causes in production.
	 */
	corrupted = (rc == 0) && present(&head, 2);
	ok(corrupted,
			"no-ryw hlist control: batch commits OK yet key 2 survives its own delete");

	free(n[0]); free(n[1]); free(n[2]); free(n[3]);
}

/* --------------------------------------------------------------------- */
/* 6-9. An insert COMPOSED WITH a delete: the trigger is slot coincidence, */
/*      not the insertion end.                                             */
/* --------------------------------------------------------------------- */

/*
 * insert_head always stores &head->first.  del(X) stores *X->pprev, which IS
 * &head->first exactly when X is the bucket's first node.  So composing the two
 * collides only for that victim -- and it collides destructively, because the
 * insert's successor load reads memory and never sees this txn's pending unlink
 * of X, so both stores present the same old.  Inserting at the tail would not
 * cure it; the trigger would move to "victim is last".
 *
 * Returns 1 if @victim survived its own delete (still reachable) -- the
 * corruption.  build() head-inserts 1,2,3,4, so the chain is 4,3,2,1 and
 * @victim_idx into n[] means: 3 = FIRST (key 4), 1 = MIDDLE (key 2),
 * 0 = LAST (key 1).
 */
static int del_plus_insert_head(int victim_idx, int ryw, unsigned long *chain,
		int *chain_len)
{
	struct urcu_txn_hlist_head head;
	struct urcu_mcas_txn txn;
	struct node *n[4], *ins;
	unsigned long victim_key;
	int survived;

	build(&head, n);			/* chain: 4,3,2,1 */
	victim_key = n[victim_idx]->key;
	ins = node_alloc(9);

	urcu_txn_init(&txn, &g_dom);
	urcu_txn_set_ryw(&txn, ryw);
	for (;;) {
		int p1, p2;
		enum urcu_txn_status st;

		urcu_txn_begin(&txn);
		p1 = urcu_txn_hlist_del_prepare(&txn, &n[victim_idx]->h);
		p2 = p1 ? 0 : urcu_txn_hlist_insert_head_prepare(&txn, &ins->h, &head);
		if (p1 == -EAGAIN || p2 == -EAGAIN) {
			urcu_txn_conflict(&txn);
			urcu_txn_end(&txn);
			continue;
		}
		if (p1 || p2)
			abort();
		st = urcu_txn_commit(&txn);
		urcu_txn_end(&txn);
		if (st == URCU_TXN_STATUS_OK)
			break;
		if (st == URCU_TXN_STATUS_ABORT)
			continue;
		abort();
	}

	*chain_len = collect(&head, chain, 16);
	survived = present(&head, victim_key);

	free(n[0]); free(n[1]); free(n[2]); free(n[3]); free(ins);
	return survived;
}

static void test_victim_position(void)
{
	unsigned long chain[16];
	int len, first_bad, middle_ok, last_ok, ryw_ok;

	/* Chain is 4,3,2,1.  Victim FIRST = key 4 = n[3]: aliases &head->first. */
	first_bad = del_plus_insert_head(3, 0, chain, &len) && len == 5;
	ok(first_bad,
			"no-ryw hlist: del(FIRST) + insert_head -- victim survives its own delete");

	/* Victim MIDDLE (key 2 = n[1]) and LAST (key 1 = n[0]): disjoint slots. */
	middle_ok = !del_plus_insert_head(1, 0, chain, &len) && len == 4;
	ok(middle_ok,
			"no-ryw hlist: del(MIDDLE) + insert_head is CORRECT -- head/tail is not the cause");

	last_ok = !del_plus_insert_head(0, 0, chain, &len) && len == 4;
	ok(last_ok,
			"no-ryw hlist: del(LAST) + insert_head is CORRECT -- only the head slot aliases");

	/* Same composition, RYW on: the insert's succ load sees the pending unlink. */
	ryw_ok = !del_plus_insert_head(3, 1, chain, &len) && len == 4 &&
			chain[0] == 9 && chain[1] == 3 && chain[2] == 2 && chain[3] == 1;
	ok(ryw_ok,
			"ryw hlist: del(FIRST) + insert_head yields the correct chain {9,3,2,1}");
}

int main(void)
{
	plan_tests(NR_TESTS);
	rcu_register_thread();
	urcu_txn_domain_init(&g_dom);

	test_ryw_adjacent_deletes();	/* 1, 2, 3 */
	test_ryw_whitebox_chain();	/* 4 */
	test_negative_no_ryw();		/* 5 */
	test_victim_position();		/* 6, 7, 8, 9 */

	rcu_thread_offline();
	rcu_barrier();
	rcu_thread_online();
	rcu_unregister_thread();
	return exit_status();
}
