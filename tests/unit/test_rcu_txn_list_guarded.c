// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

/*
 * Worked example for urcu_txn_list_insert_after_guarded_rcu(): an insert
 * that folds a load-validate guard on an embedder word into the same atomic
 * MCAS.  Here each node keeps a "live" marker beside it (killed monotonically
 * LIVE -> DEAD through the engine, so it is non-ABA and engine-transacted), and
 * the guarded insert links a node after @pos only if @pos is still live.
 *
 * Deterministic checks (the mid-flight race is covered atomically by
 * test_rcu_txn_validate.c; here we verify the mutator is wired right):
 *   1. guard holds (anchor live)   -> insert links the node;
 *   2. guard fails (anchor killed) -> insert refuses, node not linked;
 *   3. anchor deleted              -> insert returns -ENOENT (mark path).
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

#define NR_TESTS	3

/* Monotone liveness marker values (bit-0-clear: the engine owns bit 0). */
#define LIVE	((void *) 0x10)
#define DEAD	((void *) 0x20)

struct lnode {
	struct urcu_txn_list_node node;
	void *live;
};

static struct urcu_txn_list_head g_head;
static struct lnode A, B, C, E;

/* Kill a live marker LIVE -> DEAD through the engine (single-edge MCAS). */
static void kill_live(void **live_slot)
{
	struct urcu_mcas_txn tx;
	enum urcu_txn_status st;

	urcu_txn_init(&tx, &g_head.domain);
	do {
		urcu_txn_begin(&tx);
		urcu_txn_store(&tx, live_slot, LIVE, DEAD);
		st = urcu_txn_commit(&tx);
		urcu_txn_end(&tx);
	} while (st == URCU_TXN_STATUS_ABORT);
}

static int in_list(struct urcu_txn_list_node *n)
{
	struct urcu_txn_list_node *p;
	int found = 0;

	rcu_read_lock();
	for (p = urcu_txn_list_next_rcu(&g_head.node); p != &g_head.node;
			p = urcu_txn_list_next_rcu(p)) {
		if (p == n) {
			found = 1;
			break;
		}
	}
	rcu_read_unlock();
	return found;
}

int main(void)
{
	int r;

	plan_tests(NR_TESTS);
	rcu_register_thread();
	urcu_txn_list_init(&g_head);

	A.live = LIVE;
	B.live = LIVE;
	C.live = LIVE;
	E.live = LIVE;
	urcu_txn_list_add_rcu(&A.node, &g_head);		/* head <-> A */

	/* 1. Anchor live: the guarded insert links B after A. */
	r = urcu_txn_list_insert_after_guarded_rcu(&B.node, &A.node,
			&g_head, &A.live, LIVE);
	ok(r == 0 && in_list(&B.node),
		"guard holds -> guarded insert links the node after a live anchor");

	/* 2. Anchor killed: the guard no longer holds -> refuse, C not linked. */
	kill_live(&A.live);
	r = urcu_txn_list_insert_after_guarded_rcu(&C.node, &A.node,
			&g_head, &A.live, LIVE);
	ok(r == -ENOENT && !in_list(&C.node),
		"guard fails -> guarded insert refuses, node not linked");

	/* 3. Anchor deleted: caught on the mark before the guard -> -ENOENT. */
	(void) urcu_txn_list_del_rcu(&A.node, &g_head);
	r = urcu_txn_list_insert_after_guarded_rcu(&E.node, &A.node,
			&g_head, &B.live, LIVE);
	ok(r == -ENOENT && !in_list(&E.node),
		"deleted anchor -> guarded insert returns -ENOENT");

	rcu_barrier();			/* drain deferred descriptor frees */
	rcu_unregister_thread();
	return exit_status();
}
