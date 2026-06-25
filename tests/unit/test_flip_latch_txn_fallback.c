// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

/*
 * Fallback stress for <urcu/flip-latch-txn-lockfree.h>: the per-domain
 * FIFO escalation lane under a starvation pattern the engine's aging
 * cannot bound.
 *
 * The narrow workers do single-edge increments: a one-record
 * transaction takes the bare-CAS fast path, invisible to the
 * priority/steal protocol, so a steady stream of them keeps
 * invalidating a wide transaction's olds in its read->install window
 * without ever being out-ranked.  The wide workers run a transaction
 * touching every word (the merge_at analog); on the bare optimistic
 * path they would be starved by that stream.  With a non-NULL domain
 * the wide transaction escalates by size (reserve(NR_WORDS) >= BIG) on
 * its first attempt, sets domain->active, and funnels the single-edge
 * ops into the same lane, so it makes bounded progress.
 *
 * Invariant: each narrow op adds 1 and each wide op's deltas sum to 0,
 * so the total equals the number of narrow commits (a torn k-CAS would
 * break it).  Every transaction must commit (lane progress, no
 * deadlock).  QSBR flavor.
 */

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#ifndef _LGPL_SOURCE
#define _LGPL_SOURCE
#endif

/* Lower the thresholds before the include so the lane actually fires. */
#define URCU_FLIP_LF_TXN_FALLBACK	8
#define URCU_FLIP_LF_TXN_BIG		6

#include <inttypes.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include <urcu/compiler.h>
#include <urcu-qsbr.h>
#include <urcu-call-rcu.h>
#include <urcu/flip-latch-txn-lockfree.h>

#include "tap.h"

#define NR_TESTS	3
#define NR_WORDS	8		/* wide txn touches all of them */
#define NR_NARROW	6
#define NR_WIDE		2
#define NARROW_OPS	20000
#define WIDE_OPS	1500
/* Generous: progress is the point, not a tight bound. */
#define RETRY_BOUND	100000

static struct urcu_flip_lf_txn_domain g_domain;
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

/* High 32 bits: signed value; low bits: per-word monotonic version. */
static intptr_t lf_val(uintptr_t w) { return (intptr_t) w >> 32; }
static uintptr_t lf_bump(uintptr_t w, intptr_t delta)
{
	intptr_t val = lf_val(w) + delta;
	unsigned int ver = (unsigned int) ((w >> 1) & 0x7fffffffu) + 1;
	return ((uintptr_t) (uint32_t) (int32_t) val << 32)
		| ((uintptr_t) (ver & 0x7fffffffu) << 1);
}

/*
 * Narrow worker: single-edge "+1" on a random word.  A one-record
 * transaction is the fast path (bare CAS, no proxy, priority-invisible)
 * -- the stream that starves the wide op on the optimistic path.
 */
static void *narrow_worker(void *arg)
{
	struct worker_arg *wa = (struct worker_arg *) arg;
	unsigned int rng = wa->seed;
	long n;

	rcu_register_thread();
	for (n = 0; n < NARROW_OPS; n++) {
		struct urcu_flip_lf_txn tx;
		int i, ret;

		rng = xs(rng);
		i = (int) (rng % NR_WORDS);

		urcu_flip_lf_txn_init(&tx, &g_domain);
		do {
			uintptr_t oi;
			void *ni;

			urcu_flip_lf_txn_begin(&tx);
			oi = (uintptr_t) urcu_flip_lf_txn_load(&tx,
					&g_word[i]);
			ni = (void *) lf_bump(oi, 1);
			urcu_flip_lf_txn_store(&tx, &g_word[i],
					(void *) oi, ni);
			ret = urcu_flip_lf_txn_commit(&tx);
			urcu_flip_lf_txn_end(&tx);
			if (ret < 0)
				abort();		/* -ENOMEM */
		} while (ret == 0);

		if (tx.retry > wa->max_retry)
			wa->max_retry = tx.retry;
		wa->committed++;
		rcu_quiescent_state();
	}
	rcu_unregister_thread();
	return NULL;
}

/*
 * Wide worker: touch every word (word[0] += (N-1)*2, the rest -= 2
 * each, so the deltas sum to 0).  reserve(NR_WORDS) escalates it into
 * the lane up front.
 */
static void *wide_worker(void *arg)
{
	struct worker_arg *wa = (struct worker_arg *) arg;
	long n;

	rcu_register_thread();
	for (n = 0; n < WIDE_OPS; n++) {
		struct urcu_flip_lf_txn tx;
		uintptr_t o[NR_WORDS];
		int w, ret;

		urcu_flip_lf_txn_init(&tx, &g_domain);
		do {
			void *nw;

			urcu_flip_lf_txn_begin(&tx);
			(void) urcu_flip_lf_txn_reserve(&tx, NR_WORDS);
			for (w = 0; w < NR_WORDS; w++)
				o[w] = (uintptr_t) urcu_flip_lf_txn_load(
						&tx, &g_word[w]);
			nw = (void *) lf_bump(o[0], (NR_WORDS - 1) * 2);
			urcu_flip_lf_txn_store(&tx, &g_word[0],
					(void *) o[0], nw);
			for (w = 1; w < NR_WORDS; w++) {
				nw = (void *) lf_bump(o[w], -2);
				urcu_flip_lf_txn_store(&tx, &g_word[w],
						(void *) o[w], nw);
			}
			ret = urcu_flip_lf_txn_commit(&tx);
			urcu_flip_lf_txn_end(&tx);
			if (ret < 0)
				abort();		/* -ENOMEM */
		} while (ret == 0);

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
	pthread_t narrow_th[NR_NARROW], wide_th[NR_WIDE];
	struct worker_arg narrow_a[NR_NARROW], wide_a[NR_WIDE];
	long narrow_total = 0, wide_total = 0;
	unsigned long narrow_max = 0, wide_max = 0;
	intptr_t sum = 0;
	int i;

	plan_tests(NR_TESTS);
	rcu_register_thread();
	urcu_flip_lf_txn_domain_init(&g_domain);

	for (i = 0; i < NR_NARROW; i++) {
		narrow_a[i].seed = 0x9e3779b9u
				+ (unsigned int) i * 2654435761u;
		narrow_a[i].committed = 0;
		narrow_a[i].max_retry = 0;
		pthread_create(&narrow_th[i], NULL, narrow_worker,
				&narrow_a[i]);
	}
	for (i = 0; i < NR_WIDE; i++) {
		wide_a[i].seed = 0x12345678u + (unsigned int) i;
		wide_a[i].committed = 0;
		wide_a[i].max_retry = 0;
		pthread_create(&wide_th[i], NULL, wide_worker, &wide_a[i]);
	}

	rcu_thread_offline();
	for (i = 0; i < NR_NARROW; i++) {
		pthread_join(narrow_th[i], NULL);
		narrow_total += narrow_a[i].committed;
		if (narrow_a[i].max_retry > narrow_max)
			narrow_max = narrow_a[i].max_retry;
	}
	for (i = 0; i < NR_WIDE; i++) {
		pthread_join(wide_th[i], NULL);
		wide_total += wide_a[i].committed;
		if (wide_a[i].max_retry > wide_max)
			wide_max = wide_a[i].max_retry;
	}
	rcu_thread_online();

	for (i = 0; i < NR_WORDS; i++)
		sum += lf_val((uintptr_t) g_word[i]);

	diag("%d narrow + %d wide over %d words: %ld + %ld committed; "
		"sum = %" PRIdPTR " (want %ld); max retry narrow=%lu wide=%lu",
		NR_NARROW, NR_WIDE, NR_WORDS, narrow_total, wide_total,
		sum, narrow_total, narrow_max, wide_max);

	ok(sum == narrow_total,
		"transactions stayed atomic through the lane (sum invariant)");
	ok(narrow_total == (long) NR_NARROW * NARROW_OPS &&
			wide_total == (long) NR_WIDE * WIDE_OPS,
		"every narrow and wide transaction committed (lane progress)");
	ok(wide_max < RETRY_BOUND,
		"wide transaction progress stayed bounded under the lane");

	rcu_barrier();
	rcu_unregister_thread();
	return exit_status();
}
