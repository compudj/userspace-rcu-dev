// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

/*
 * Test for prep-time priority resolution (urcu_txn_resolve_prio /
 * urcu_mcas_resolve_prio): resolving a slot that bears a foreign proxy to a
 * DEFINITE value by AGING PRIORITY -- help a higher-priority owner, evict a
 * lower-priority one -- so a mutator can pick the value it will transact before
 * it has parked anything, with the install loop's bounded-blocking progress.
 * This is what recompaction prep needs to resolve a proxied child slot to the
 * one child it will reparent, without livelocking under a stream of peer latches.
 *
 * Writers run transfer transactions over a pair of source words (sum invariant
 * 0), parking proxies as they contend.  Preppers RESOLVE each source by priority
 * (urcu_txn_resolve_prio), then FREEZE the two resolved values via COPY_SLOT into
 * a fresh node and commit -- so the COPY_SLOT read-set check (src == V)
 * re-validates the resolution at the linearization point.  Two properties:
 *
 *  - Correctness: every committed snapshot has pair-sum 0 (the resolution picked
 *    consistent values, or the commit aborted and retried).
 *  - Progress: a prepper that meets a higher-priority writer and spends its help
 *    budget gets CAP (resolve_prio -> 0); it aborts and retries at a higher
 *    aging priority until it can evict, so it always eventually commits with
 *    bounded retry.
 *
 * The fresh node is allocated per attempt and reclaimed via call_rcu (the
 * COPY_SLOT publish runs on any driver, incl. a lagging helper).  Slots carry a
 * per-word monotonic version (lf_bump) so a stored word never repeats a bit
 * pattern (the engine's non-ABA precondition).  QSBR flavor.
 */

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#ifndef _LGPL_SOURCE
#define _LGPL_SOURCE
#endif

#include <inttypes.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include <urcu/compiler.h>
#include <urcu/uatomic.h>
#include <urcu-qsbr.h>
#include <urcu-call-rcu.h>
#include <urcu/rcu-txn.h>

#include "tap.h"

#define NR_TESTS	3
#define NR_WRITERS	4
#define NR_PREPPERS	4
#define NR_WORKERS	(NR_WRITERS + NR_PREPPERS)
#define OPS_PER_WORKER	40000
#define RETRY_BOUND	512

static void *g_src[2];

struct fresh_node {
	void *slot[2];
	struct rcu_head rcu;
};

static void free_fresh_rcu(struct rcu_head *h)
{
	free(caa_container_of(h, struct fresh_node, rcu));
}

struct worker_arg {
	long committed;
	unsigned long max_retry;
	long violations;
};

static intptr_t lf_val(uintptr_t w) { return (intptr_t) w >> 32; }
static uintptr_t lf_bump(uintptr_t w, intptr_t delta)
{
	intptr_t val = lf_val(w) + delta;
	unsigned int ver = (unsigned int) ((w >> 1) & 0x7fffffffu) + 1;
	return ((uintptr_t) (uint32_t) (int32_t) val << 32)
		| ((uintptr_t) (ver & 0x7fffffffu) << 1);
}

/* Writer: {g_src[0] += 2, g_src[1] -= 2}, parking proxies under contention. */
static void *writer(void *arg)
{
	struct worker_arg *wa = (struct worker_arg *) arg;
	long n;

	rcu_register_thread();
	for (n = 0; n < OPS_PER_WORKER; n++) {
		struct urcu_mcas_txn tx;
		int ret;

		urcu_txn_init(&tx, NULL);
		do {
			uintptr_t o0, o1;

			urcu_txn_begin(&tx);
			o0 = (uintptr_t) urcu_txn_load(&tx, &g_src[0], URCU_MCAS_TAG);
			o1 = (uintptr_t) urcu_txn_load(&tx, &g_src[1], URCU_MCAS_TAG);
			urcu_txn_store(&tx, &g_src[0], (void *) o0,
					(void *) lf_bump(o0, 2), URCU_MCAS_TAG);
			urcu_txn_store(&tx, &g_src[1], (void *) o1,
					(void *) lf_bump(o1, -2), URCU_MCAS_TAG);
			ret = urcu_txn_commit(&tx);
			urcu_txn_end(&tx);
			if (ret < 0)
				abort();
		} while (ret == URCU_TXN_STATUS_ABORT);

		if (tx.retry > wa->max_retry)
			wa->max_retry = tx.retry;
		wa->committed++;
		rcu_quiescent_state();
	}
	rcu_unregister_thread();
	return NULL;
}

/*
 * Prepper: resolve the pair by priority, then freeze via COPY_SLOT.  On CAP
 * (a higher-priority writer holds a source and the help budget is spent) abort
 * and retry at a higher aging priority.
 */
static void *prepper(void *arg)
{
	struct worker_arg *wa = (struct worker_arg *) arg;
	long n;

	rcu_register_thread();
	for (n = 0; n < OPS_PER_WORKER; n++) {
		struct urcu_mcas_txn tx;
		struct fresh_node *fn = NULL;
		void *d0, *d1;
		int committed = 0;

		urcu_txn_init(&tx, NULL);
		while (!committed) {
			void *v0, *v1;
			int ret;

			fn = malloc(sizeof(*fn));
			if (!fn)
				abort();
			uatomic_store(&fn->slot[0], NULL, CMM_RELAXED);
			uatomic_store(&fn->slot[1], NULL, CMM_RELAXED);

			urcu_txn_begin(&tx);
			if (urcu_txn_reserve(&tx, 2) < 0)	/* descriptor = self */
				abort();
			if (!urcu_txn_resolve_prio(&tx, &g_src[0], URCU_MCAS_TAG, &v0) ||
			    !urcu_txn_resolve_prio(&tx, &g_src[1], URCU_MCAS_TAG, &v1)) {
				/* CAP: abort prep, retry at a higher aging priority. */
				urcu_txn_abort(&tx);
				urcu_txn_end(&tx);
				call_rcu(&fn->rcu, free_fresh_rcu);
				fn = NULL;
				tx.retry++;	/* age up so the retry can evict */
				continue;
			}
			(void) urcu_txn_copy_slot(&tx, &g_src[0], v0, &fn->slot[0],
					URCU_MCAS_TAG);
			(void) urcu_txn_copy_slot(&tx, &g_src[1], v1, &fn->slot[1],
					URCU_MCAS_TAG);
			ret = urcu_txn_commit(&tx);
			urcu_txn_end(&tx);
			if (ret < 0)
				abort();
			if (ret == URCU_TXN_STATUS_OK) {
				committed = 1;
			} else {
				call_rcu(&fn->rcu, free_fresh_rcu);
				fn = NULL;
			}
		}

		d0 = uatomic_load(&fn->slot[0], CMM_ACQUIRE);
		d1 = uatomic_load(&fn->slot[1], CMM_ACQUIRE);
		if (lf_val((uintptr_t) d0) + lf_val((uintptr_t) d1) != 0)
			wa->violations++;
		call_rcu(&fn->rcu, free_fresh_rcu);

		if (tx.retry > wa->max_retry)
			wa->max_retry = tx.retry;
		wa->committed++;
		rcu_quiescent_state();
	}
	rcu_unregister_thread();
	return NULL;
}

int main(void)
{
	pthread_t th[NR_WORKERS];
	struct worker_arg args[NR_WORKERS];
	long total = 0, violations = 0;
	unsigned long max_retry = 0;
	intptr_t sum;
	int i;

	plan_tests(NR_TESTS);
	rcu_register_thread();

	g_src[0] = NULL;
	g_src[1] = NULL;
	for (i = 0; i < NR_WORKERS; i++) {
		args[i].committed = 0;
		args[i].max_retry = 0;
		args[i].violations = 0;
		pthread_create(&th[i], NULL,
				i < NR_WRITERS ? writer : prepper, &args[i]);
	}
	rcu_thread_offline();
	for (i = 0; i < NR_WORKERS; i++) {
		pthread_join(th[i], NULL);
		total += args[i].committed;
		violations += args[i].violations;
		if (args[i].max_retry > max_retry)
			max_retry = args[i].max_retry;
	}
	rcu_thread_online();

	sum = lf_val((uintptr_t) g_src[0]) + lf_val((uintptr_t) g_src[1]);

	diag("%d writers + %d preppers x %d ops = %ld committed; snapshot violations "
		"= %ld; final source sum = %" PRIdPTR "; max retry = %lu",
		NR_WRITERS, NR_PREPPERS, OPS_PER_WORKER, total, violations, sum,
		max_retry);

	ok(violations == 0,
		"every priority-resolved COPY_SLOT snapshot was atomic (pair sum 0)");
	ok(sum == 0,
		"priority resolution never disturbed a source (writers' sum preserved)");
	ok(total == (long) NR_WORKERS * OPS_PER_WORKER && max_retry < RETRY_BOUND,
		"every prepper eventually committed with bounded retry (evict progress)");

	rcu_barrier();
	rcu_unregister_thread();
	return exit_status();
}
