// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

/*
 * Test for the transaction front-end <urcu/rcu-txn-mw.h>: the
 * begin/store/commit/end bracket with the internally-managed retry count.
 *
 * Same atomicity + progress invariants as test_rcu_mcas.c, but
 * driven through the bracket instead of the raw MCAS: many threads run
 * "transfer" transactions (add and subtract equal amounts across 2-3 distinct
 * words) on a small hot array, so the total sum is invariantly 0 and a torn
 * k-CAS would show.
 *
 * The caller never threads a retry count -- the handle does it -- so this also
 * checks that the aging priority still bounds the worst single-op bypass.
 *
 * Slots carry a per-word monotonic version (lf_bump) so a stored word never
 * repeats a bit pattern (the engine's non-ABA precondition).  QSBR flavor.
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
#include <urcu-qsbr.h>
#include <urcu-call-rcu.h>
#include <urcu/rcu-txn-mw.h>

#include "tap.h"

#define NR_TESTS	3
#define NR_WORKERS	8
#define NR_WORDS	4		/* small => heavy contention */
#define OPS_PER_WORKER	40000
#define RETRY_BOUND	512		/* generous worst single-op bypass bound */

static void *g_word[NR_WORDS];

struct worker_arg {
	unsigned int seed;
	long committed;
	unsigned long max_retry;
};

static unsigned int xs(unsigned int x)
{
	x ^= x << 13; x ^= x >> 17; x ^= x << 5; return x;
}

static intptr_t lf_val(uintptr_t w) { return (intptr_t) w >> 32; }
static uintptr_t lf_bump(uintptr_t w, intptr_t delta)
{
	intptr_t val = lf_val(w) + delta;
	unsigned int ver = (unsigned int) ((w >> 1) & 0x7fffffffu) + 1;
	return ((uintptr_t) (uint32_t) (int32_t) val << 32)
		| ((uintptr_t) (ver & 0x7fffffffu) << 1);
}

static void *worker(void *arg)
{
	struct worker_arg *wa = (struct worker_arg *) arg;
	unsigned int rng = wa->seed;
	long n;

	rcu_register_thread();
	for (n = 0; n < OPS_PER_WORKER; n++) {
		struct urcu_txn_mw tx;
		int i, j, k, three, ret;

		rng = xs(rng);
		three = (int) (rng & 1);
		i = (int) ((rng >> 1) % NR_WORDS);
		rng = xs(rng);
		j = (int) (rng % NR_WORDS); if (j == i) j = (j + 1) % NR_WORDS;
		rng = xs(rng);
		k = (int) (rng % NR_WORDS); while (k == i || k == j) k = (k + 1) % NR_WORDS;

		urcu_txn_mw_init(&tx, NULL);
		do {
			uintptr_t oi, oj, ok2;

			urcu_txn_mw_begin(&tx);
			oi = (uintptr_t) urcu_txn_mw_load(&tx, &g_word[i], URCU_MCAS_TAG);
			oj = (uintptr_t) urcu_txn_mw_load(&tx, &g_word[j], URCU_MCAS_TAG);
			urcu_txn_mw_store(&tx, &g_word[i], (void *) oi, (void *) lf_bump(oi, 2), URCU_MCAS_TAG);
			if (three) {
				ok2 = (uintptr_t) urcu_txn_mw_load(&tx, &g_word[k], URCU_MCAS_TAG);
				urcu_txn_mw_store(&tx, &g_word[j], (void *) oj, (void *) lf_bump(oj, 2), URCU_MCAS_TAG);
				urcu_txn_mw_store(&tx, &g_word[k], (void *) ok2, (void *) lf_bump(ok2, -4), URCU_MCAS_TAG);
			} else {
				urcu_txn_mw_store(&tx, &g_word[j], (void *) oj, (void *) lf_bump(oj, -2), URCU_MCAS_TAG);
			}
			ret = urcu_txn_mw_commit(&tx);
			urcu_txn_mw_end(&tx);
			if (ret < 0)
				abort();		/* MEMORY_ERROR */
		} while (ret == URCU_TXN_STATUS_ABORT);

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
	long total = 0;
	unsigned long max_retry = 0;
	intptr_t sum = 0;
	int i;

	plan_tests(NR_TESTS);
	rcu_register_thread();

	for (i = 0; i < NR_WORKERS; i++) {
		args[i].seed = 0x9e3779b9u + (unsigned int) i * 2654435761u;
		args[i].committed = 0;
		args[i].max_retry = 0;
		pthread_create(&th[i], NULL, worker, &args[i]);
	}
	rcu_thread_offline();
	for (i = 0; i < NR_WORKERS; i++) {
		pthread_join(th[i], NULL);
		total += args[i].committed;
		if (args[i].max_retry > max_retry)
			max_retry = args[i].max_retry;
	}
	rcu_thread_online();

	for (i = 0; i < NR_WORDS; i++)
		sum += lf_val((uintptr_t) g_word[i]);

	diag("%d workers x %d ops over %d words = %ld committed; sum = %" PRIdPTR
		"; max single-op retry = %lu",
		NR_WORKERS, OPS_PER_WORKER, NR_WORDS, total, sum, max_retry);

	ok(sum == 0,
		"transactions stayed atomic across concurrent transfers (sum invariant)");
	ok(total == (long) NR_WORKERS * OPS_PER_WORKER,
		"every transaction eventually committed (bounded-blocking progress)");
	ok(max_retry < RETRY_BOUND,
		"worst single-op bypass stayed bounded (internal aging retry)");

	rcu_barrier();
	rcu_unregister_thread();
	return exit_status();
}
