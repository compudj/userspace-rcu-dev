// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

/*
 * Growth test for the transaction front-end
 * <urcu/rcu-txn.h>: the write-set is a heap
 * descriptor the handle allocates lazily and grows
 * (urcu_mcas_grow, realloc) as stores arrive, rather than a
 * fixed array.  Two things need exercising beyond
 * test_rcu_txn's 2-3 word transactions:
 *
 *  Phase 1 -- growth under writer/writer contention.  Each
 *  transaction spans PH1_TXN distinct words (> URCU_TXN_INIT,
 *  so the descriptor grows), many writers on a small array, half the
 *  ops pre-sizing with reserve() and half growing lazily.  Transfers
 *  are zero-sum, so a torn k-CAS shows as a non-zero total;
 *  multi-record descriptors also drive the steal/evict path.
 *
 *  Phase 2 -- descriptor RELOCATION.  A small grow is extended in
 *  place, so the descriptor never moves and the "back-pointer
 *  survives a move" path is never taken.  Here one writer builds a
 *  PH2_N-record transaction (~PH2_N*32 bytes, past glibc's mmap
 *  threshold), so realloc relocates the descriptor as it grows from
 *  INIT to PH2_N.  The record back-pointers (r->mcas) are filled only
 *  at commit, after the last move; concurrent readers then resolve
 *  the in-flight proxies through r->mcas.  A back-pointer set at add
 *  time (before a move) would point into the freed old descriptor --
 *  caught here as an out-of-range value, and under a sanitizer (whose
 *  realloc always relocates and poisons the old block) as a
 *  use-after-free in urcu_mcas_status().
 *
 * Slots never repeat a value (the engine's non-ABA precondition):
 * phase 1 packs a monotonic version per word (lf_bump); phase 2 only
 * ever increases a word.  QSBR flavor.
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
#include <urcu/rcu-txn.h>

#include "tap.h"

#define NR_TESTS	3

/* ---- Phase 1: growth under contention ---- */

#define PH1_WORKERS	8
#define PH1_WORDS	16
/* distinct words/txn: even (zero-sum), > INIT 4 (forces a grow) */
#define PH1_TXN		10
#define PH1_OPS		10000

static void *ph1_word[PH1_WORDS];

struct ph1_arg {
	unsigned int seed;
	long committed;
};

static unsigned int xs(unsigned int x)
{
	x ^= x << 13; x ^= x >> 17; x ^= x << 5; return x;
}

/*
 * Pack a signed value in the high 32 bits and a monotonic version in
 * the low bits (bit 0 stays clear for the engine's tag), so a word
 * never repeats a bit pattern.
 */
static intptr_t lf_val(uintptr_t w) { return (intptr_t) w >> 32; }
static uintptr_t lf_bump(uintptr_t w, intptr_t delta)
{
	intptr_t val = lf_val(w) + delta;
	unsigned int ver = (unsigned int) ((w >> 1) & 0x7fffffffu) + 1;
	return ((uintptr_t) (uint32_t) (int32_t) val << 32)
		| ((uintptr_t) (ver & 0x7fffffffu) << 1);
}

static void *ph1_worker(void *arg)
{
	struct ph1_arg *wa = (struct ph1_arg *) arg;
	unsigned int rng = wa->seed;
	long n;

	rcu_register_thread();
	for (n = 0; n < PH1_OPS; n++) {
		struct urcu_mcas_txn tx;
		int idx[PH1_WORDS], i, ret;

		/* Fisher-Yates prefix: PH1_TXN distinct indices. */
		for (i = 0; i < PH1_WORDS; i++)
			idx[i] = i;
		for (i = 0; i < PH1_TXN; i++) {
			int j, t;

			rng = xs(rng);
			j = i + (int) (rng % (unsigned int) (PH1_WORDS - i));
			t = idx[i]; idx[i] = idx[j]; idx[j] = t;
		}

		urcu_txn_init(&tx, NULL);
		do {
			urcu_txn_begin(&tx);
			/* odd ops reserve(); even grow lazily */
			if (n & 1)
				(void) urcu_txn_reserve(&tx, PH1_TXN);
			for (i = 0; i < PH1_TXN; i++) {
				uintptr_t old = (uintptr_t) urcu_txn_load(
						&tx, &ph1_word[idx[i]], URCU_MCAS_TAG);
				intptr_t delta = (i < PH1_TXN / 2) ? +3 : -3;

				urcu_txn_store(&tx, &ph1_word[idx[i]],
						(void *) old,
						(void *) lf_bump(old, delta), URCU_MCAS_TAG);
			}
			ret = urcu_txn_commit(&tx);
			urcu_txn_end(&tx);
			if (ret < 0)
				abort();		/* MEMORY_ERROR */
		} while (ret == URCU_TXN_STATUS_ABORT);
		wa->committed++;
		rcu_quiescent_state();
	}
	rcu_unregister_thread();
	return NULL;
}

/* ---- Phase 2: descriptor relocation ---- */

/* *32B ~= 640KB, past the mmap threshold, so realloc relocates */
#define PH2_N		20000
#define PH2_ROUNDS	2
#define PH2_READERS	2

static void **ph2_word;			/* PH2_N slots */
static volatile int ph2_reader_bad;	/* resolved a bad value */
static volatile int ph2_apply_bad;	/* a round was torn */
static volatile int ph2_done;

static inline void *enc(long v) { return (void *) (uintptr_t) (v << 1); }
static inline long  dec(void *p) { return (long) ((intptr_t) (uintptr_t) p >> 1); }

/*
 * Every slot is always enc(i + k*PH2_N) for some round k in
 * [0, PH2_ROUNDS].
 */
static int ph2_value_ok(long i, long v)
{
	long rem = v - i;

	return rem >= 0 && (rem % PH2_N) == 0 && (rem / PH2_N) <= PH2_ROUNDS;
}

static void *ph2_reader(void *arg)
{
	uint64_t rng = (uint64_t) (uintptr_t) arg * 0x9e3779b97f4a7c15ull + 1;

	rcu_register_thread();
	while (!CMM_LOAD_SHARED(ph2_done)) {
		int b;

		rcu_read_lock();
		for (b = 0; b < 256; b++) {
			long i, v;

			rng = rng * 6364136223846793005ull
					+ 1442695040888963407ull;
			i = (long) ((rng >> 33) % PH2_N);
			v = dec(urcu_mcas_resolve(
					urcu_mcas_read(&ph2_word[i], URCU_MCAS_TAG), URCU_MCAS_TAG));
			if (!ph2_value_ok(i, v))
				CMM_STORE_SHARED(ph2_reader_bad, 1);
		}
		rcu_read_unlock();
		rcu_quiescent_state();
	}
	rcu_unregister_thread();
	return NULL;
}

static void *ph2_writer(void *arg)
{
	int round;

	(void) arg;
	rcu_register_thread();
	for (round = 0; round < PH2_ROUNDS; round++) {
		struct urcu_mcas_txn tx;
		long i;
		int ret;

		/*
		 * Monotonic: word[i] grows enc(i+round*N) ->
		 * enc(i+(round+1)*N).  No reserve -- the descriptor
		 * must grow from INIT to PH2_N (so relocate) for this
		 * phase to mean anything.
		 */
		urcu_txn_init(&tx, NULL);
		do {
			urcu_txn_begin(&tx);
			for (i = 0; i < PH2_N; i++)
				urcu_txn_store(&tx, &ph2_word[i],
						enc(i + (long) round * PH2_N),
						enc(i + (long) (round + 1) * PH2_N), URCU_MCAS_TAG);
			ret = urcu_txn_commit(&tx);
			urcu_txn_end(&tx);
			if (ret < 0)
				abort();		/* MEMORY_ERROR */
		} while (ret == URCU_TXN_STATUS_ABORT);

		/* The whole k-CAS applied atomically: all flipped. */
		for (i = 0; i < PH2_N; i++)
			if (dec(ph2_word[i]) != i + (long) (round + 1) * PH2_N)
				ph2_apply_bad = 1;
		rcu_quiescent_state();
	}
	rcu_unregister_thread();
	return NULL;
}

int main(void)
{
	pthread_t ph1[PH1_WORKERS], wr, rd[PH2_READERS];
	struct ph1_arg a[PH1_WORKERS];
	long total = 0, i;
	intptr_t sum = 0;
	int final_ok = 1;

	plan_tests(NR_TESTS);
	rcu_register_thread();

	/* ---- Phase 1 ---- */
	for (i = 0; i < PH1_WORDS; i++)
		ph1_word[i] = (void *) lf_bump(0, 0);
	for (i = 0; i < PH1_WORKERS; i++) {
		a[i].seed = 0x9e3779b9u + (unsigned int) i * 2654435761u;
		a[i].committed = 0;
		pthread_create(&ph1[i], NULL, ph1_worker, &a[i]);
	}
	rcu_thread_offline();
	for (i = 0; i < PH1_WORKERS; i++) {
		pthread_join(ph1[i], NULL);
		total += a[i].committed;
	}
	rcu_thread_online();
	for (i = 0; i < PH1_WORDS; i++)
		sum += lf_val((uintptr_t) ph1_word[i]);

	diag("phase 1: %d workers x %d ops, %d words/txn over %d words "
		"(grows past INIT %d); committed=%ld sum=%" PRIdPTR,
		PH1_WORKERS, PH1_OPS, PH1_TXN, PH1_WORDS,
		URCU_TXN_INIT, total, sum);
	ok(sum == 0,
		"grow under contention stayed atomic (zero-sum invariant)");
	ok(total == (long) PH1_WORKERS * PH1_OPS,
		"every grown transaction committed (bounded-blocking progress)");

	/* ---- Phase 2 ---- */
	ph2_word = malloc((size_t) PH2_N * sizeof(*ph2_word));
	if (!ph2_word)
		ok(0, "phase 2 allocation");
	else {
		for (i = 0; i < PH2_N; i++)
			ph2_word[i] = enc(i);
		for (i = 0; i < PH2_READERS; i++)
			pthread_create(&rd[i], NULL, ph2_reader,
					(void *) (uintptr_t) (i + 1));
		pthread_create(&wr, NULL, ph2_writer, NULL);
		rcu_thread_offline();
		pthread_join(wr, NULL);
		CMM_STORE_SHARED(ph2_done, 1);
		for (i = 0; i < PH2_READERS; i++)
			pthread_join(rd[i], NULL);
		rcu_thread_online();

		for (i = 0; i < PH2_N; i++)
			if (dec(ph2_word[i]) != i + (long) PH2_ROUNDS * PH2_N)
				final_ok = 0;

		diag("phase 2: %d-record txn (~%.1f MB descriptor) x %d "
			"rounds, %d readers resolving proxies; "
			"reader_bad=%d apply_bad=%d",
			PH2_N, (double) PH2_N * 32 / (1024 * 1024),
			PH2_ROUNDS, PH2_READERS, ph2_reader_bad, ph2_apply_bad);
		ok(final_ok && !ph2_apply_bad && !ph2_reader_bad,
			"k-CAS stayed atomic across a relocated descriptor "
			"(no torn apply, no stale back-pointer)");
		free(ph2_word);
	}

	rcu_barrier();
	rcu_unregister_thread();
	return exit_status();
}
