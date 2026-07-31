// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

/*
 * Composing several hlist deletes that touch NEIGHBOURING nodes within one
 * transaction, under read-your-own-writes -- the engine's default.
 *
 * The hlist looks immune to the ordered structures' co-batching hazard -- a head
 * insert writes a fixed, key-independent slot, and a delete writes node-local
 * state.  It is not.  urcu_txn_hlist_del_prepare() reads its WRITE SITE out of
 * the structure:
 *
 *     ppv = urcu_txn_load(txn, &elem->pprev);   // the slot that names elem
 *     ...
 *     urcu_txn_store_mw(txn, ppv, elem, next);     // store THROUGH it
 *
 * That is a one-hop traversal, and a one-hop traversal is still a traversal.
 * Delete two ADJACENT nodes A and B (B == A->next) in one transaction and, were
 * buffered writes invisible (the retired pre-RYW mode), del(B) would load the
 * stale &A->next as its write site -- even though del(A) has already recorded
 * that B is renamed to &P->next.  It would then store through &A->next presenting
 * old == B, which MATCHES the record del(A) left there ({B -> MARK(B)}), so the
 * engine's one-record-per-slot upgrade would overwrite new_ptr: A's TOMBSTONE
 * DESTROYED.  The commit would leave A unlinked but unmarked, B marked yet still
 * linked (so its delete silently did not happen, while the caller reclaims it),
 * and C->pprev pointing INTO the removed A.
 *
 * The interesting part is that the engine handles this on its own: this file
 * makes NO change to <urcu/rcu-txn-hlist.h>.  Because del_prepare routes that
 * load through urcu_txn_load(), read-your-own-writes retargets it onto &P->next,
 * and the subsequent store CHAINS onto del(A)'s record ({A -> B} becomes
 * {A -> C}).  Both nodes end marked, both unlinked, C correctly renamed.
 *
 * An INSERT composed with a delete aliases the same way, showing the trigger is
 * slot coincidence rather than the insertion end.  insert_head always stores
 * &head->first; del(X) stores *X->pprev, which IS &head->first exactly when X is
 * the bucket's first node.  Were writes invisible the insert's successor load
 * would read memory, never seeing this txn's pending unlink of X, so both stores
 * would present the same old and the upgrade would leave X MARK-ed yet still
 * linked.  Read-your-own-writes retargets the load, so the two edges chain.  The
 * victim-position tests below confirm it: FIRST -- the aliasing case -- composes
 * correctly, and MIDDLE and LAST touch disjoint slots so they never aliased.
 * Inserting at the tail would only move the aliasing to "victim is last".
 *
 * This is the evidence that read-your-own-writes + chaining is a GENERIC engine
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
#include <urcu/rcu-txn-mcas.h>
#include <urcu/rcu-txn-hlist.h>
#include <urcu/rcu-txn.h>

#include "tap.h"

#define NR_TESTS	7
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
		void *v = urcu_txn_read((void **) &n->next, URCU_TXN_HLIST_TAG);

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
 *
 * EVERY GIVE-UP PATH CALLS urcu_txn_abandon() BEFORE end().  An ABORT (and
 * urcu_txn_conflict()) keeps the handle's FIFO turn so the re-attempt is not
 * sent to the back of the queue, and end() honours that -- so a bounded-retry
 * loop that simply returns leaves the domain's fair mutex held forever, and
 * every later writer in the domain parks behind an owner that is gone.  Note in
 * particular that the spin check has to happen while the bracket is still open:
 * testing it at the top of the loop, after the previous iteration's end() has
 * already kept the turn, is exactly the leak.  abandon() is idempotent and
 * costs nothing on a handle that never escalated, so it goes on every exit.
 */
static int del_two(struct urcu_txn_hlist_node *a, struct urcu_txn_hlist_node *b)
{
	struct urcu_txn txn;
	long spins = 0;
	enum urcu_txn_status st;

	urcu_txn_init(&txn, &g_dom);
	for (;;) {
		int prep, retry = 0, err = 0;

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
			urcu_txn_abandon(&txn);
			urcu_txn_end(&txn);
			return err;
		}
		if (retry) {
			urcu_txn_conflict(&txn);
			if (++spins > SPIN_LIMIT) {
				urcu_txn_abandon(&txn);
				urcu_txn_end(&txn);
				return -ETIMEDOUT;
			}
			urcu_txn_end(&txn);
			continue;
		}
		st = urcu_txn_commit(&txn);
		if (st == URCU_TXN_STATUS_OK) {
			urcu_txn_end(&txn);
			return 0;
		}
		if (st == URCU_TXN_STATUS_ABORT && ++spins <= SPIN_LIMIT) {
			urcu_txn_end(&txn);
			continue;
		}
		urcu_txn_abandon(&txn);
		urcu_txn_end(&txn);
		return st == URCU_TXN_STATUS_ABORT ? -ETIMEDOUT : -ENOMEM;
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

/*
 * NOTE ON THE free()s BELOW.  They are immediate, with no grace period, which
 * is NOT the contract an embedder must follow: an unlinked node is reclaimable
 * only after a grace period, because a concurrent reader may still be walking
 * through it.  It is safe HERE for two reasons that do not generalise -- these
 * cases are single-threaded, so there is no concurrent reader at all, and the
 * commit settles every slot before returning, so no proxy naming the node
 * survives the call.  Copy the discipline from test_rcu_txn_hlist.c's
 * call_rcu() teardown, not from these lines.
 */

static void test_ryw_adjacent_deletes(void)
{
	struct urcu_txn_hlist_head head;
	struct node *n[4];
	unsigned long got[16];
	int cnt, rc;

	build(&head, n);
	/* list: 4,3,2,1 -- delete the adjacent pair (3,2). */
	rc = del_two(&n[2]->h, &n[1]->h);		/* keys 3 then 2 */
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
	struct urcu_txn txn;
	struct urcu_txn_record *r;
	int pass;

	build(&head, n);		/* 4,3,2,1 */

	urcu_txn_init(&txn, &g_dom);
	/*
	 * A deliberate same-slot chaining txn is an expect-conflict txn: under an
	 * AGE_ESCALATE build age 0 would optimistically escalate instead of chaining,
	 * so declare the conflict to exercise the age-1 chaining path this whitebox
	 * asserts.  Inert (a no-op) in a stock build.
	 */
	urcu_txn_expect_conflict(&txn);
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
	r = urcu_txn_find(txn.desc, (void **) &n[3]->h.next);
	pass = r != NULL && r->old_ptr == (void *) &n[2]->h &&
			r->new_ptr == (void *) &n[0]->h;
	ok(pass, "ryw hlist whitebox: the two unlinks chain to {n(3) -> n(1)} on one slot");

	if (urcu_txn_commit(&txn) != URCU_TXN_STATUS_OK)
		abort();
	urcu_txn_end(&txn);
	assert_chain_sane(&head);

	free(n[0]); free(n[1]); free(n[2]); free(n[3]);
}

/* --------------------------------------------------------------------- */
/* An insert COMPOSED WITH a delete: the trigger is slot coincidence, not    */
/* the insertion end.  All three victim positions compose correctly under    */
/* the default read-your-own-writes.                                         */
/* --------------------------------------------------------------------- */

/*
 * insert_head always stores &head->first.  del(X) stores *X->pprev, which IS
 * &head->first exactly when X is the bucket's first node.  So composing the two
 * aliases only for that victim -- and read-your-own-writes retargets the
 * insert's successor load onto the pending unlink, so the two stores chain
 * instead of clobbering.  A victim in the MIDDLE or at the LAST position touches
 * a disjoint slot and never aliased.
 *
 * Returns 1 if @victim survived its own delete (still reachable).  build()
 * head-inserts 1,2,3,4, so the chain is 4,3,2,1 and @victim_idx into n[] means:
 * 3 = FIRST (key 4), 1 = MIDDLE (key 2), 0 = LAST (key 1).
 */
static int del_plus_insert_head(int victim_idx, unsigned long *chain,
		int *chain_len)
{
	struct urcu_txn_hlist_head head;
	struct urcu_txn txn;
	struct node *n[4], *ins;
	unsigned long victim_key;
	int survived;

	build(&head, n);			/* chain: 4,3,2,1 */
	victim_key = n[victim_idx]->key;
	ins = node_alloc(9);

	urcu_txn_init(&txn, &g_dom);
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
	int len, first_ok, middle_ok, last_ok;

	/* Chain is 4,3,2,1.  Victim FIRST = key 4 = n[3] aliases &head->first --
	 * the position that would corrupt without read-your-own-writes.  Under the
	 * default the insert's successor load sees the pending unlink and the two
	 * stores chain: insert 9 at head, unlink 4 -> {9,3,2,1}. */
	first_ok = !del_plus_insert_head(3, chain, &len) && len == 4 &&
			chain[0] == 9 && chain[1] == 3 && chain[2] == 2 && chain[3] == 1;
	ok(first_ok,
			"ryw hlist: del(FIRST) + insert_head composes to {9,3,2,1}");

	/* Victim MIDDLE (key 2 = n[1]) and LAST (key 1 = n[0]) touch disjoint
	 * slots -- never aliased, correct regardless. */
	middle_ok = !del_plus_insert_head(1, chain, &len) && len == 4 &&
			chain[0] == 9 && chain[1] == 4 && chain[2] == 3 && chain[3] == 1;
	ok(middle_ok,
			"ryw hlist: del(MIDDLE) + insert_head composes to {9,4,3,1}");

	last_ok = !del_plus_insert_head(0, chain, &len) && len == 4 &&
			chain[0] == 9 && chain[1] == 4 && chain[2] == 3 && chain[3] == 2;
	ok(last_ok,
			"ryw hlist: del(LAST) + insert_head composes to {9,4,3,2}");
}

int main(void)
{
	plan_tests(NR_TESTS);
	rcu_register_thread();
	urcu_txn_domain_init(&g_dom);

	test_ryw_adjacent_deletes();	/* 1, 2, 3 */
	test_ryw_whitebox_chain();	/* 4 */
	test_victim_position();		/* 5, 6, 7 */

	rcu_thread_offline();
	rcu_barrier();
	rcu_thread_online();
	rcu_unregister_thread();
	return exit_status();
}
