// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

/*
 * Test for the transacted bitmap <urcu/rcu-txn-bitmap.h>: a fixed-size bitmap
 * over transacted words (63 data bits/word, bit 0 = engine proxy tag) whose
 * set/clear composes into an MCAS commit.
 *
 * Coverage:
 *  - single-thread encoding: test / rank (popcount-below) / weight / select /
 *    ffs, across every 63-bit word boundary, against a plain model; the bit-0
 *    tag invariant on every settled physical word; atomic multi-word range set.
 *  - single-thread COMPOSITION: one commit that flips a bitmap bit AND stores a
 *    pointer lands both.
 *  - concurrent NO-LOST-UPDATE: many threads set disjoint bits that share words
 *    (word-granular contention); no neighbour bit is clobbered (final weight
 *    exact), then all clear back to empty.
 *  - concurrent COMPOSITION ATOMICITY: writers toggle {bit k, slot[k]} together;
 *    a guarded-snapshot reader that certifies the pair at one linearization
 *    point must NEVER see bit != (ptr != NULL).  This is the property that lets
 *    the fractal trie replace COW recompaction with one in-place transaction.
 *
 * QSBR flavor; every worker is an RCU reader so descriptors stay alive while
 * helpers drive them.
 */

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#ifndef _LGPL_SOURCE
#define _LGPL_SOURCE
#endif

#include <inttypes.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <urcu/compiler.h>
#include <urcu-qsbr.h>
#include <urcu-call-rcu.h>

#include <urcu/rcu-txn-bitmap.h>

#include "tap.h"

#define NR_TESTS	12

#define BPW		URCU_TXN_BITMAP_BITS_PER_WORD	/* 63 on LP64 */

static uint64_t xs(uint64_t x)			/* xorshift64 */
{
	x ^= x << 13;
	x ^= x >> 7;
	x ^= x << 17;
	return x;
}

/* Every SETTLED physical word must keep bit 0 clear (the engine tag). */
static int tag_invariant_ok(const uintptr_t *words, size_t nwords)
{
	size_t w;

	for (w = 0; w < nwords; w++)
		if (words[w] & URCU_TXN_TAG)
			return 0;
	return 1;
}

/* ------------------------------------------------------------------ */
/* T1: single-thread encoding correctness                             */
/* ------------------------------------------------------------------ */

#define T1_NBITS	256
static void test_encoding(struct urcu_txn_domain *dom)
{
	size_t nw = URCU_TXN_BITMAP_NR_WORDS(T1_NBITS);		/* 5 */
	uintptr_t *bm = calloc(nw, sizeof *bm);
	/* a set touching every 63-bit boundary (62/63, 125/126, 188/189, 251/252) */
	size_t S[] = { 0, 1, 62, 63, 64, 125, 126, 127,
		       188, 189, 251, 252, 255 };
	size_t ns = sizeof S / sizeof S[0], j, b;
	char present[T1_NBITS];
	int empty_ok = 1, enc_ok = 1, clear_ok = 1, range_ok = 1;
	size_t model_rank;
	long prev, idx;

	memset(present, 0, sizeof present);

	/* empty bitmap */
	rcu_read_lock();
	empty_ok &= urcu_txn_bitmap_weight_rcu(bm, T1_NBITS) == 0;
	empty_ok &= urcu_txn_bitmap_ffs_from_rcu(bm, T1_NBITS, 0) == -1;
	empty_ok &= urcu_txn_bitmap_select_rcu(bm, T1_NBITS, 0) == -1;
	for (b = 0; b < T1_NBITS; b++) {
		empty_ok &= !urcu_txn_bitmap_test_rcu(bm, b);
		empty_ok &= urcu_txn_bitmap_rank_rcu(bm, b) == 0;
	}
	rcu_read_unlock();
	ok(empty_ok, "empty: weight/test/rank/ffs/select all zero");

	/* populate */
	for (j = 0; j < ns; j++) {
		urcu_txn_bitmap_set_rcu(dom, bm, S[j]);
		present[S[j]] = 1;
	}

	rcu_read_lock();
	model_rank = 0;
	for (b = 0; b < T1_NBITS; b++) {
		enc_ok &= urcu_txn_bitmap_test_rcu(bm, b) == present[b];
		enc_ok &= urcu_txn_bitmap_rank_rcu(bm, b) == model_rank;
		if (present[b])
			model_rank++;
	}
	enc_ok &= urcu_txn_bitmap_weight_rcu(bm, T1_NBITS) == ns;
	/* ffs walk in order + select as its inverse */
	prev = -1;
	idx = 0;
	for (long bit = urcu_txn_bitmap_ffs_from_rcu(bm, T1_NBITS, 0);
			bit != -1;
			bit = urcu_txn_bitmap_ffs_from_rcu(bm, T1_NBITS, (size_t) bit + 1)) {
		enc_ok &= present[bit] && bit > prev;
		enc_ok &= urcu_txn_bitmap_select_rcu(bm, T1_NBITS, (size_t) idx) == bit;
		prev = bit;
		idx++;
	}
	enc_ok &= (size_t) idx == ns;
	enc_ok &= urcu_txn_bitmap_select_rcu(bm, T1_NBITS, ns) == -1;
	rcu_read_unlock();
	ok(enc_ok, "encoding: test/rank/weight/ffs/select match model across word boundaries");

	ok(tag_invariant_ok(bm, nw), "tag: bit 0 clear on every physical word after sets");

	/* clear across a boundary, recheck */
	urcu_txn_bitmap_clear_rcu(dom, bm, 63);
	urcu_txn_bitmap_clear_rcu(dom, bm, 255);
	present[63] = present[255] = 0;
	rcu_read_lock();
	clear_ok &= !urcu_txn_bitmap_test_rcu(bm, 63);
	clear_ok &= !urcu_txn_bitmap_test_rcu(bm, 255);
	clear_ok &= urcu_txn_bitmap_weight_rcu(bm, T1_NBITS) == ns - 2;
	rcu_read_unlock();
	ok(clear_ok, "clear: bits drop, weight exact, no neighbour disturbed");

	/* atomic multi-word range set [100,150) */
	{
		struct urcu_txn t;
		enum urcu_txn_status st;

		urcu_txn_init(&t, dom);
		do {
			urcu_txn_begin(&t);
			(void) urcu_txn_bitmap_set_range_prepare(&t, bm, 100, 150);
			st = urcu_txn_commit(&t);
			urcu_txn_end(&t);
		} while (st == URCU_TXN_STATUS_ABORT);
	}
	rcu_read_lock();
	for (b = 100; b < 150; b++)
		range_ok &= urcu_txn_bitmap_test_rcu(bm, b);
	range_ok &= !urcu_txn_bitmap_test_rcu(bm, 99);
	range_ok &= !urcu_txn_bitmap_test_rcu(bm, 150);
	rcu_read_unlock();
	ok(range_ok, "range: [100,150) set atomically across words, bounds untouched");

	free(bm);
}

/* ------------------------------------------------------------------ */
/* T2: single-thread composition (bit + pointer in one commit)        */
/* ------------------------------------------------------------------ */

static void test_compose_single(struct urcu_txn_domain *dom)
{
	uintptr_t bm[URCU_TXN_BITMAP_NR_WORDS(64)];
	void *slot = NULL;
	void *sent = (void *) (uintptr_t) 0x40;		/* bit 0 clear */
	struct urcu_txn t;
	enum urcu_txn_status st;
	int set_ok, clr_ok;

	memset(bm, 0, sizeof bm);

	urcu_txn_init(&t, dom);
	do {						/* set bit 10 AND publish slot */
		urcu_txn_begin(&t);
		(void) urcu_txn_bitmap_set_prepare(&t, bm, 10);
		(void) urcu_txn_store_mw(&t, (void **) &slot, NULL, sent, URCU_TXN_TAG);
		st = urcu_txn_commit(&t);
		urcu_txn_end(&t);
	} while (st == URCU_TXN_STATUS_ABORT);
	rcu_read_lock();
	set_ok = urcu_txn_bitmap_test_rcu(bm, 10)
		&& urcu_txn_read((void **) &slot, URCU_TXN_TAG) == sent;
	rcu_read_unlock();
	ok(set_ok, "compose: one commit sets the bit AND stores the pointer");

	do {						/* clear bit 10 AND clear slot */
		urcu_txn_begin(&t);
		(void) urcu_txn_bitmap_clear_prepare(&t, bm, 10);
		(void) urcu_txn_store_mw(&t, (void **) &slot, sent, NULL, URCU_TXN_TAG);
		st = urcu_txn_commit(&t);
		urcu_txn_end(&t);
	} while (st == URCU_TXN_STATUS_ABORT);
	rcu_read_lock();
	clr_ok = !urcu_txn_bitmap_test_rcu(bm, 10)
		&& urcu_txn_read((void **) &slot, URCU_TXN_TAG) == NULL;
	rcu_read_unlock();
	ok(clr_ok, "compose: one commit clears the bit AND nulls the pointer");
}

/* ------------------------------------------------------------------ */
/* T3: concurrent no-lost-update under word-granular contention       */
/* ------------------------------------------------------------------ */

#define T3_NBITS	256
#define T3_THREADS	8
#define T3_ITERS	2000
static uintptr_t t3_bm[URCU_TXN_BITMAP_NR_WORDS(T3_NBITS)];
static struct urcu_txn_domain t3_dom;

struct t3_arg { int id; int clear; };

static void *t3_worker(void *a)
{
	struct t3_arg *arg = a;
	int it;
	size_t b;

	rcu_register_thread();
	for (it = 0; it < T3_ITERS; it++) {
		/* thread owns bits {id, id+T3_THREADS, ...} -- disjoint bits,
		 * heavily SHARED words (63 bits/word span many owners). */
		for (b = (size_t) arg->id; b < T3_NBITS; b += T3_THREADS) {
			if (arg->clear)
				urcu_txn_bitmap_clear_rcu(&t3_dom, t3_bm, b);
			else
				urcu_txn_bitmap_set_rcu(&t3_dom, t3_bm, b);
		}
		rcu_quiescent_state();
	}
	rcu_unregister_thread();
	return NULL;
}

static void t3_run(int clear)
{
	pthread_t th[T3_THREADS];
	struct t3_arg ar[T3_THREADS];
	int i;

	for (i = 0; i < T3_THREADS; i++) {
		ar[i].id = i;
		ar[i].clear = clear;
		(void) pthread_create(&th[i], NULL, t3_worker, &ar[i]);
	}
	rcu_thread_offline();
	for (i = 0; i < T3_THREADS; i++)
		(void) pthread_join(th[i], NULL);
	rcu_thread_online();
}

static void test_no_lost_update(void)
{
	size_t nw = URCU_TXN_BITMAP_NR_WORDS(T3_NBITS), b, wt;
	int all_set = 1, invariant;

	memset(t3_bm, 0, sizeof t3_bm);
	urcu_txn_domain_init(&t3_dom);

	t3_run(0);					/* everyone sets its bits */
	rcu_read_lock();
	wt = urcu_txn_bitmap_weight_rcu(t3_bm, T3_NBITS);
	for (b = 0; b < T3_NBITS; b++)
		all_set &= urcu_txn_bitmap_test_rcu(t3_bm, b);
	rcu_read_unlock();
	invariant = tag_invariant_ok(t3_bm, nw);
	ok(wt == T3_NBITS && all_set,
		"concurrent: %d threads set disjoint bits in shared words, none lost (weight=%zu/%d)",
		T3_THREADS, wt, T3_NBITS);
	ok(invariant, "concurrent: bit-0 tag invariant held under contention");

	t3_run(1);					/* everyone clears its bits */
	rcu_read_lock();
	wt = urcu_txn_bitmap_weight_rcu(t3_bm, T3_NBITS);
	rcu_read_unlock();
	ok(wt == 0, "concurrent: all bits cleared back to empty (weight=%zu)", wt);
}

/* ------------------------------------------------------------------ */
/* T4: concurrent composition atomicity (guarded snapshot)            */
/* ------------------------------------------------------------------ */

#define T4_K		128
#define T4_WRITERS	6
#define T4_READERS	3
#define T4_RUN_NS	400000000L			/* 400 ms */
static uintptr_t t4_bm[URCU_TXN_BITMAP_NR_WORDS(T4_K)];
static void *t4_slot[T4_K];
static struct urcu_txn_domain t4_dom;
static volatile int t4_stop;
static _Atomic long t4_violations;
static _Atomic long t4_saw_set, t4_saw_clear;

static void *t4_sent(size_t k)				/* non-NULL, bit 0 clear */
{
	return (void *) (((uintptr_t) (k + 1)) << 1);
}

static void *t4_writer(void *a)
{
	int id = (int) (intptr_t) a;

	rcu_register_thread();
	while (!CMM_LOAD_SHARED(t4_stop)) {
		size_t k;

		/* each writer owns a disjoint residue class of k (shared bitmap
		 * words), toggling {bit k, slot[k]} together in one commit. */
		for (k = (size_t) id; k < T4_K; k += T4_WRITERS) {
			struct urcu_txn t;
			enum urcu_txn_status st;

			urcu_txn_init(&t, &t4_dom);
			do {
				void *cur;

				urcu_txn_begin(&t);
				cur = urcu_txn_load(&t, (void **) &t4_slot[k],
						URCU_TXN_TAG);
				if (cur == NULL) {
					(void) urcu_txn_bitmap_set_prepare(&t, t4_bm, k);
					(void) urcu_txn_store_mw(&t, (void **) &t4_slot[k],
							NULL, t4_sent(k), URCU_TXN_TAG);
				} else {
					(void) urcu_txn_bitmap_clear_prepare(&t, t4_bm, k);
					(void) urcu_txn_store_mw(&t, (void **) &t4_slot[k],
							t4_sent(k), NULL, URCU_TXN_TAG);
				}
				st = urcu_txn_commit(&t);
				urcu_txn_end(&t);
			} while (st == URCU_TXN_STATUS_ABORT);
			rcu_quiescent_state();
		}
	}
	rcu_unregister_thread();
	return NULL;
}

static void *t4_reader(void *a)
{
	uint64_t rng = 0x9e3779b97f4a7c15ULL ^ (uint64_t) (intptr_t) a;

	rcu_register_thread();
	while (!CMM_LOAD_SHARED(t4_stop)) {
		size_t k, w;
		unsigned phys;
		struct urcu_txn t;
		enum urcu_txn_status st;
		uintptr_t bword = 0;
		void *p = NULL;
		int bit;

		rng = xs(rng);
		k = rng % T4_K;
		w = k / BPW;
		phys = (unsigned) (1 + k % BPW);
		/* guarded snapshot: certify the (bit, ptr) pair at one
		 * linearization point.  If the composition were two commits, a
		 * successful snapshot could straddle them and catch a mismatch. */
		urcu_txn_init(&t, &t4_dom);
		do {
			urcu_txn_begin(&t);
			bword = (uintptr_t) urcu_txn_load_validate(&t,
					(void **) &t4_bm[w], URCU_TXN_TAG);
			p = urcu_txn_load_validate(&t,
					(void **) &t4_slot[k], URCU_TXN_TAG);
			st = urcu_txn_commit(&t);
			urcu_txn_end(&t);
		} while (st == URCU_TXN_STATUS_ABORT);
		bit = (int) ((bword >> phys) & 1);
		if (bit != (p != NULL))
			atomic_fetch_add(&t4_violations, 1);
		else if (bit)
			atomic_fetch_add(&t4_saw_set, 1);
		else
			atomic_fetch_add(&t4_saw_clear, 1);
		rcu_quiescent_state();
	}
	rcu_unregister_thread();
	return NULL;
}

static void test_compose_atomic(void)
{
	pthread_t w[T4_WRITERS], r[T4_READERS];
	struct timespec ts = { 0, T4_RUN_NS };
	int i;
	long v, seen_set, seen_clear;

	memset(t4_bm, 0, sizeof t4_bm);
	memset(t4_slot, 0, sizeof t4_slot);
	urcu_txn_domain_init(&t4_dom);
	t4_stop = 0;
	atomic_store(&t4_violations, 0);
	atomic_store(&t4_saw_set, 0);
	atomic_store(&t4_saw_clear, 0);

	for (i = 0; i < T4_WRITERS; i++)
		(void) pthread_create(&w[i], NULL, t4_writer, (void *) (intptr_t) i);
	for (i = 0; i < T4_READERS; i++)
		(void) pthread_create(&r[i], NULL, t4_reader, (void *) (intptr_t) i);

	rcu_thread_offline();
	(void) nanosleep(&ts, NULL);
	CMM_STORE_SHARED(t4_stop, 1);
	for (i = 0; i < T4_WRITERS; i++)
		(void) pthread_join(w[i], NULL);
	for (i = 0; i < T4_READERS; i++)
		(void) pthread_join(r[i], NULL);
	rcu_thread_online();

	v = atomic_load(&t4_violations);
	seen_set = atomic_load(&t4_saw_set);
	seen_clear = atomic_load(&t4_saw_clear);
	diag("compose-atomic: %ld consistent (set) + %ld consistent (clear) snapshots, %ld violations",
		seen_set, seen_clear, v);
	ok(v == 0,
		"compose-atomic: guarded snapshot never saw bit != (ptr!=NULL) (%ld violations)",
		v);
	ok(seen_set > 0 && seen_clear > 0,
		"compose-atomic: readers actually observed both states (not vacuous)");
}

int main(void)
{
	struct urcu_txn_domain dom;

	plan_tests(NR_TESTS);
	rcu_register_thread();
	urcu_txn_domain_init(&dom);

	test_encoding(&dom);		/* 5 */
	test_compose_single(&dom);	/* 2 */
	test_no_lost_update();		/* 3 */
	test_compose_atomic();		/* 2 */

	rcu_barrier();
	rcu_unregister_thread();
	return exit_status();
}
