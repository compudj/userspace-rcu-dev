// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

/*
 * Worked example + correctness checks for urcu_txn_list_replace_rcu(): an
 * in-place atomic replacement of @old by @newp in the concurrent bidir list.
 * Replace touches the same slots as del (prev->next, next->prev, old->next-mark)
 * but swings the neighbours to @newp instead of skipping, so @newp inherits
 * @old's position while @old becomes a forward-escapable ghost.
 *
 * Deterministic single-threaded checks (the concurrent serialization is the same
 * MCAS as del, stressed by the compose test):
 *   1. replace returns 0 and @newp occupies @old's slot (both directions);
 *   2. forward and reverse stay coherent mirrors;
 *   3. @old is unreachable but a reader standing on it escapes forward;
 *   4. replacing an already-deleted node returns -ENOENT and links nothing.
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
#include <stdio.h>
#include <stdlib.h>

#include <urcu/compiler.h>
#include <urcu-qsbr.h>
#include <urcu-call-rcu.h>
#include <urcu/rcu-txn.h>
#include <urcu/rcu-txn-list.h>

#include "tap.h"

#define NR_TESTS	8

struct lnode {
	struct urcu_txn_list_node node;
	long id;
};

static struct urcu_txn_list_head g_head;
static struct urcu_txn_domain g_dom;	/* shared escalation domain */
static struct lnode A, B, C, Bp, Ap;

static int in_list(struct urcu_txn_list_node *n)
{
	struct urcu_txn_list_node *p;
	int found = 0, guard = 0;

	rcu_read_lock();
	for (p = urcu_txn_list_next_rcu(&g_head.node);
			p != &g_head.node && guard++ < 16;
			p = urcu_txn_list_next_rcu(p))
		if (p == n) {
			found = 1;
			break;
		}
	rcu_read_unlock();
	return found;
}

/* Collect forward (dir>0) or reverse (dir<0) node order into @out; return count. */
static int collect(struct urcu_txn_list_node **out, int max, int dir)
{
	struct urcu_txn_list_node *p;
	int n = 0;

	rcu_read_lock();
	if (dir > 0)
		for (p = urcu_txn_list_next_rcu(&g_head.node);
				p != &g_head.node && n < max;
				p = urcu_txn_list_next_rcu(p))
			out[n++] = p;
	else
		for (p = urcu_txn_list_prev_rcu(&g_head.node);
				p != &g_head.node && n < max;
				p = urcu_txn_list_prev_rcu(p))
			out[n++] = p;
	rcu_read_unlock();
	return n;
}

int main(void)
{
	struct urcu_txn_list_node *fwd[8], *rev[8];
	int nf, nr, r;

	plan_tests(NR_TESTS);
	rcu_register_thread();
	urcu_txn_list_init(&g_head);
	urcu_txn_domain_init(&g_dom);

	A.id = 1; B.id = 2; C.id = 3; Bp.id = 20; Ap.id = 10;

	/* Build head -> A -> B -> C. */
	urcu_txn_list_add_rcu(&C.node, &g_head, &g_dom);
	urcu_txn_list_add_rcu(&B.node, &g_head, &g_dom);
	urcu_txn_list_add_rcu(&A.node, &g_head, &g_dom);

	/* 1. Replace B with Bp. */
	r = urcu_txn_list_replace_rcu(&B.node, &Bp.node, &g_dom);
	ok(r == 0, "replace returns 0 (Bp took B's slot)");
	ok(in_list(&Bp.node) && !in_list(&B.node),
		"Bp is now in the list and B is not");

	/* 2. Forward order is A -> Bp -> C. */
	nf = collect(fwd, 8, +1);
	ok(nf == 3 && fwd[0] == &A.node && fwd[1] == &Bp.node && fwd[2] == &C.node,
		"forward order is A -> Bp -> C");

	/* 3. Reverse order is C -> Bp -> A (exact mirror). */
	nr = collect(rev, 8, -1);
	ok(nr == 3 && rev[0] == &C.node && rev[1] == &Bp.node && rev[2] == &A.node,
		"reverse order is C -> Bp -> A (coherent mirror)");

	/* 4. Neighbour edges point at Bp both ways. */
	rcu_read_lock();
	ok(urcu_txn_list_next_rcu(&A.node) == &Bp.node &&
			urcu_txn_list_prev_rcu(&C.node) == &Bp.node,
		"A->next == Bp and C->prev == Bp");
	ok(urcu_txn_list_next_rcu(&Bp.node) == &C.node &&
			urcu_txn_list_prev_rcu(&Bp.node) == &A.node,
		"Bp->next == C and Bp->prev == A");

	/* 5. A reader standing on the ghost B escapes forward to C. */
	ok(urcu_txn_list_next_rcu(&B.node) == &C.node,
		"a reader on the replaced ghost B escapes forward to C");
	rcu_read_unlock();

	/* 6. Replacing an already-deleted node returns -ENOENT, links nothing. */
	(void) urcu_txn_list_del_rcu(&A.node, &g_dom);
	r = urcu_txn_list_replace_rcu(&A.node, &Ap.node, &g_dom);
	ok(r == -ENOENT && !in_list(&Ap.node),
		"replacing a deleted node returns -ENOENT and links nothing");

	rcu_barrier();
	rcu_unregister_thread();
	return exit_status();
}
