// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

/*
 * Test for the single-updater transacted bitmap <urcu/rcu-txn-sw-bitmap.h>: a
 * fixed-size bitmap over transacted words (63 data bits/word, bit 0 = engine
 * proxy tag) whose set/clear composes into a flip transaction.
 *
 * Coverage:
 *  - single-thread encoding: test / rank (popcount-below) / weight / select /
 *    ffs, across every 63-bit word boundary, against a plain model; the bit-0
 *    tag invariant on every settled physical word; multi-word range set.
 *    Mirrors test_rcu_txn_bitmap.c's T1 so the two front-ends are checked to
 *    share one encoding.
 *  - single-thread COMPOSITION: one commit that flips a bitmap bit AND stores a
 *    pointer lands both.
 *  - SAME-WORD FUSION -- the reason this header exists in this shape.  63 bits
 *    share a word, and the sw engine has no transactional load and no same-slot
 *    reconcile, so a naive port would record two latches on one word and settle
 *    them in record order, silently losing the earlier flip (install()'s debug
 *    scan aborts; an NDEBUG build corrupts quietly).  The _prepare forms fuse
 *    instead; these tests pin both the VALUES and the record count, since a
 *    correct-looking value could still hide a duplicate record.
 *  - concurrent PER-WORD ATOMICITY: one writer toggles a two-word range while
 *    readers resolve; each word must read all-clear or all-set, never torn.
 *    This exercises the proxy path (a 2-edge commit parks proxies and flips a
 *    group, where a 1-edge commit is just a release store).
 *
 * NOT covered, because the engine does not provide it: an atomic MULTI-WORD
 * snapshot.  The concurrent twin gets one from a read-only transaction that
 * urcu_txn_load_validate()s each word (that test's T4); the sw engine has no
 * read set, no validation and no abort, so a reader's scan may straddle a range
 * commit and there is nothing here to assert.  See the PARITY note in
 * <urcu/rcu-txn-sw-bitmap.h>.
 *
 * QSBR flavor; the writer is registered too (commit defers reclaim through
 * call_rcu), so it reports quiescent states between transactions.
 */

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#ifndef _LGPL_SOURCE
#define _LGPL_SOURCE
#endif

#include <pthread.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <urcu/compiler.h>
#include <urcu-qsbr.h>
#include <urcu-call-rcu.h>

#include <urcu/rcu-txn-sw-bitmap.h>

#include "tap.h"

#define NR_TESTS	15

#define BPW		URCU_TXN_SW_BITMAP_BITS_PER_WORD	/* 63 on LP64 */

/* Every SETTLED physical word must keep bit 0 clear (the engine tag). */
static int tag_invariant_ok(const uintptr_t *words, size_t nwords)
{
	size_t w;

	for (w = 0; w < nwords; w++)
		if (words[w] & URCU_MCAS_TAG)
			return 0;
	return 1;
}

/*
 * Resolve a non-bitmap slot transacted under URCU_MCAS_TAG (the pointer half of
 * the composition test).  The bitmap's own words go through
 * urcu_txn_sw_bitmap_word_rcu(); this is the same two lines for a plain slot.
 */
static void *sw_resolve(void *v)
{
	if ((uintptr_t) v & URCU_MCAS_TAG)
		return urcu_txn_sw_proxy_get((struct urcu_txn_sw_proxy *)
				((uintptr_t) v & ~(uintptr_t) URCU_MCAS_TAG));
	return v;
}

/* ------------------------------------------------------------------ */
/* T1: single-thread encoding correctness                             */
/* ------------------------------------------------------------------ */

#define T1_NBITS	256
static void test_encoding(void)
{
	size_t nw = URCU_TXN_SW_BITMAP_NR_WORDS(T1_NBITS);	/* 5 */
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

	/* empty bitmap: calloc'd storage needs no init */
	rcu_read_lock();
	empty_ok &= urcu_txn_sw_bitmap_weight_rcu(bm, T1_NBITS) == 0;
	empty_ok &= urcu_txn_sw_bitmap_ffs_from_rcu(bm, T1_NBITS, 0) == -1;
	empty_ok &= urcu_txn_sw_bitmap_select_rcu(bm, T1_NBITS, 0) == -1;
	for (b = 0; b < T1_NBITS; b++) {
		empty_ok &= !urcu_txn_sw_bitmap_test_rcu(bm, b);
		empty_ok &= urcu_txn_sw_bitmap_rank_rcu(bm, b) == 0;
	}
	rcu_read_unlock();
	ok(empty_ok, "empty: weight/test/rank/ffs/select all zero");

	/* populate */
	for (j = 0; j < ns; j++) {
		urcu_txn_sw_bitmap_set_rcu(bm, S[j]);
		present[S[j]] = 1;
	}

	rcu_read_lock();
	model_rank = 0;
	for (b = 0; b < T1_NBITS; b++) {
		enc_ok &= urcu_txn_sw_bitmap_test_rcu(bm, b) == present[b];
		enc_ok &= urcu_txn_sw_bitmap_rank_rcu(bm, b) == model_rank;
		if (present[b])
			model_rank++;
	}
	enc_ok &= urcu_txn_sw_bitmap_weight_rcu(bm, T1_NBITS) == ns;
	/* ffs walk in order + select as its inverse */
	prev = -1;
	idx = 0;
	for (long bit = urcu_txn_sw_bitmap_ffs_from_rcu(bm, T1_NBITS, 0);
			bit != -1;
			bit = urcu_txn_sw_bitmap_ffs_from_rcu(bm, T1_NBITS, (size_t) bit + 1)) {
		enc_ok &= present[bit] && bit > prev;
		enc_ok &= urcu_txn_sw_bitmap_select_rcu(bm, T1_NBITS, (size_t) idx) == bit;
		prev = bit;
		idx++;
	}
	enc_ok &= (size_t) idx == ns;
	enc_ok &= urcu_txn_sw_bitmap_select_rcu(bm, T1_NBITS, ns) == -1;
	rcu_read_unlock();
	ok(enc_ok, "encoding: test/rank/weight/ffs/select match model across word boundaries");

	ok(tag_invariant_ok(bm, nw), "tag: bit 0 clear on every physical word after sets");

	/* clear across a boundary, recheck */
	urcu_txn_sw_bitmap_clear_rcu(bm, 63);
	urcu_txn_sw_bitmap_clear_rcu(bm, 255);
	present[63] = present[255] = 0;
	rcu_read_lock();
	clear_ok &= !urcu_txn_sw_bitmap_test_rcu(bm, 63);
	clear_ok &= !urcu_txn_sw_bitmap_test_rcu(bm, 255);
	clear_ok &= urcu_txn_sw_bitmap_weight_rcu(bm, T1_NBITS) == ns - 2;
	rcu_read_unlock();
	ok(clear_ok, "clear: bits drop, weight exact, no neighbour disturbed");

	/* multi-word range set [100,150) as one transaction */
	{
		struct urcu_txn_sw_txn t;

		urcu_txn_sw_init(&t);
		(void) urcu_txn_sw_bitmap_set_range_prepare(&t, bm, 100, 150);
		range_ok &= urcu_txn_sw_commit(&t) == URCU_TXN_STATUS_OK;
	}
	rcu_read_lock();
	for (b = 100; b < 150; b++)
		range_ok &= urcu_txn_sw_bitmap_test_rcu(bm, b);
	range_ok &= !urcu_txn_sw_bitmap_test_rcu(bm, 99);
	range_ok &= !urcu_txn_sw_bitmap_test_rcu(bm, 150);
	rcu_read_unlock();
	ok(range_ok, "range: [100,150) set across words, bounds untouched");

	free(bm);
}

/* ------------------------------------------------------------------ */
/* T2: single-thread composition (bit + pointer in one commit)        */
/* ------------------------------------------------------------------ */

static void test_compose_single(void)
{
	uintptr_t bm[URCU_TXN_SW_BITMAP_NR_WORDS(64)];
	void *slot = NULL;
	void *sent = (void *) (uintptr_t) 0x40;		/* bit 0 clear */
	struct urcu_txn_sw_txn t;
	int set_ok, clr_ok;

	memset(bm, 0, sizeof bm);

	urcu_txn_sw_init(&t);				/* set bit 10 AND publish slot */
	(void) urcu_txn_sw_bitmap_set_prepare(&t, bm, 10);
	(void) urcu_txn_sw_record(&t, (void **) &slot, NULL, sent, URCU_MCAS_TAG);
	set_ok = urcu_txn_sw_commit(&t) == URCU_TXN_STATUS_OK;
	rcu_read_lock();
	set_ok &= urcu_txn_sw_bitmap_test_rcu(bm, 10)
		&& sw_resolve(uatomic_load(&slot, CMM_ACQUIRE)) == sent;
	rcu_read_unlock();
	ok(set_ok, "compose: one commit sets the bit AND stores the pointer");

	urcu_txn_sw_init(&t);				/* clear bit 10 AND clear slot */
	(void) urcu_txn_sw_bitmap_clear_prepare(&t, bm, 10);
	(void) urcu_txn_sw_record(&t, (void **) &slot, sent, NULL, URCU_MCAS_TAG);
	clr_ok = urcu_txn_sw_commit(&t) == URCU_TXN_STATUS_OK;
	rcu_read_lock();
	clr_ok &= !urcu_txn_sw_bitmap_test_rcu(bm, 10)
		&& sw_resolve(uatomic_load(&slot, CMM_ACQUIRE)) == NULL;
	rcu_read_unlock();
	ok(clr_ok, "compose: one commit clears the bit AND nulls the pointer");
}

/* ------------------------------------------------------------------ */
/* T3: same-word fusion                                               */
/* ------------------------------------------------------------------ */

/*
 * Each case pins the resulting VALUES *and* txn.nr (captured before commit,
 * which consumes the handle).  The record count is the point: without fusion
 * these cases record two latches on one word, which is the silent-corruption
 * bug this header exists to prevent -- and a value check alone can miss it,
 * since last-wins happens to give the right answer for some orderings.
 */
static void test_fuse(void)
{
	uintptr_t bm[URCU_TXN_SW_BITMAP_NR_WORDS(128)];
	struct urcu_txn_sw_txn t;
	unsigned int nr;
	int r;

	/* two sets of distinct bits sharing word 0 */
	memset(bm, 0, sizeof bm);
	urcu_txn_sw_init(&t);
	(void) urcu_txn_sw_bitmap_set_prepare(&t, bm, 3);
	(void) urcu_txn_sw_bitmap_set_prepare(&t, bm, 5);
	nr = t.nr;
	r = urcu_txn_sw_commit(&t) == URCU_TXN_STATUS_OK;
	rcu_read_lock();
	r &= urcu_txn_sw_bitmap_test_rcu(bm, 3) && urcu_txn_sw_bitmap_test_rcu(bm, 5);
	r &= urcu_txn_sw_bitmap_weight_rcu(bm, 128) == 2;
	rcu_read_unlock();
	ok(r, "fuse: two same-word sets in one txn both land");
	ok(nr == 1, "fuse: same-word sets collapse to one record (nr=%u, want 1)", nr);

	/* read-your-own-writes: set then clear the SAME bit -> net clear */
	memset(bm, 0, sizeof bm);
	urcu_txn_sw_init(&t);
	(void) urcu_txn_sw_bitmap_set_prepare(&t, bm, 7);
	(void) urcu_txn_sw_bitmap_clear_prepare(&t, bm, 7);
	nr = t.nr;
	r = urcu_txn_sw_commit(&t) == URCU_TXN_STATUS_OK;
	rcu_read_lock();
	r &= !urcu_txn_sw_bitmap_test_rcu(bm, 7);
	r &= urcu_txn_sw_bitmap_weight_rcu(bm, 128) == 0;
	rcu_read_unlock();
	ok(r && nr == 1, "fuse: set-then-clear of one bit nets clear in one record");

	/* a range, then a single-bit clear punching a hole in it */
	memset(bm, 0, sizeof bm);
	urcu_txn_sw_init(&t);
	(void) urcu_txn_sw_bitmap_set_range_prepare(&t, bm, 0, 10);
	(void) urcu_txn_sw_bitmap_clear_prepare(&t, bm, 5);
	nr = t.nr;
	r = urcu_txn_sw_commit(&t) == URCU_TXN_STATUS_OK;
	rcu_read_lock();
	r &= urcu_txn_sw_bitmap_weight_rcu(bm, 128) == 9;
	r &= !urcu_txn_sw_bitmap_test_rcu(bm, 5);
	r &= urcu_txn_sw_bitmap_test_rcu(bm, 4) && urcu_txn_sw_bitmap_test_rcu(bm, 6);
	rcu_read_unlock();
	ok(r && nr == 1, "fuse: single-bit clear punches a hole in a same-word range");

	/* distinct words must NOT fuse: two records, both land */
	memset(bm, 0, sizeof bm);
	urcu_txn_sw_init(&t);
	(void) urcu_txn_sw_bitmap_set_prepare(&t, bm, 3);	/* word 0 */
	(void) urcu_txn_sw_bitmap_set_prepare(&t, bm, 70);	/* word 1 */
	nr = t.nr;
	r = urcu_txn_sw_commit(&t) == URCU_TXN_STATUS_OK;
	rcu_read_lock();
	r &= urcu_txn_sw_bitmap_test_rcu(bm, 3) && urcu_txn_sw_bitmap_test_rcu(bm, 70);
	r &= urcu_txn_sw_bitmap_weight_rcu(bm, 128) == 2;
	rcu_read_unlock();
	ok(r, "fuse: flips on distinct words both land");
	ok(nr == 2, "fuse: distinct words stay distinct records (nr=%u, want 2)", nr);
}

/* ------------------------------------------------------------------ */
/* T4: concurrent per-word atomicity (proxy path)                     */
/* ------------------------------------------------------------------ */

#define T4_NBITS	(2 * 63)		/* exactly two words */
#define T4_READERS	4
#define T4_ITERS	20000
static uintptr_t t4_bm[URCU_TXN_SW_BITMAP_NR_WORDS(T4_NBITS)];
static atomic_int t4_stop;
static atomic_long t4_torn, t4_saw_full, t4_saw_empty, t4_saw_proxy;

/* The one updater: toggle both words between all-clear and all-set.  A
 * two-word range is two edges, so commit parks proxies and flips a group --
 * the path a single-edge commit (a lone release store) would never exercise. */
static void *t4_writer(void *unused __attribute__((unused)))
{
	int it;

	rcu_register_thread();
	for (it = 0; it < T4_ITERS; it++) {
		struct urcu_txn_sw_txn t;

		urcu_txn_sw_init(&t);
		if (it & 1)
			(void) urcu_txn_sw_bitmap_clear_range_prepare(&t, t4_bm,
					0, T4_NBITS);
		else
			(void) urcu_txn_sw_bitmap_set_range_prepare(&t, t4_bm,
					0, T4_NBITS);
		(void) urcu_txn_sw_commit(&t);
		rcu_quiescent_state();		/* registered: keep GPs moving */
	}
	atomic_store(&t4_stop, 1);
	rcu_unregister_thread();
	return NULL;
}

static void *t4_reader(void *unused __attribute__((unused)))
{
	long torn = 0, full = 0, empty = 0, proxy = 0;

	rcu_register_thread();
	while (!atomic_load(&t4_stop)) {
		size_t w;

		rcu_read_lock();
		for (w = 0; w < URCU_TXN_SW_BITMAP_NR_WORDS(T4_NBITS); w++) {
			uintptr_t raw = uatomic_load(&t4_bm[w], CMM_ACQUIRE);
			int pop;

			if ((raw & URCU_MCAS_TAG) == URCU_MCAS_TAG)
				proxy++;	/* caught the install..settle window */
			pop = __builtin_popcountl(
					urcu_txn_sw_bitmap_word_rcu(t4_bm, w));
			/*
			 * Per-word atomicity: this word is one transacted slot,
			 * so a reader resolves it to the pre- or post-flip
			 * value, never a mixture.  (Cross-word consistency is
			 * NOT asserted: a scan may straddle the flip -- see the
			 * PARITY note in the header.)
			 */
			if (pop == (int) BPW)
				full++;
			else if (pop == 0)
				empty++;
			else
				torn++;
		}
		rcu_read_unlock();
		rcu_quiescent_state();
	}
	rcu_unregister_thread();
	atomic_fetch_add(&t4_torn, torn);
	atomic_fetch_add(&t4_saw_full, full);
	atomic_fetch_add(&t4_saw_empty, empty);
	atomic_fetch_add(&t4_saw_proxy, proxy);
	return NULL;
}

static void test_word_atomicity(void)
{
	pthread_t wr, rd[T4_READERS];
	int i;
	long torn, full, empty, proxy;

	memset(t4_bm, 0, sizeof t4_bm);
	atomic_store(&t4_stop, 0);

	for (i = 0; i < T4_READERS; i++)
		if (pthread_create(&rd[i], NULL, t4_reader, NULL))
			abort();
	if (pthread_create(&wr, NULL, t4_writer, NULL))
		abort();

	pthread_join(wr, NULL);
	for (i = 0; i < T4_READERS; i++)
		pthread_join(rd[i], NULL);

	torn = atomic_load(&t4_torn);
	full = atomic_load(&t4_saw_full);
	empty = atomic_load(&t4_saw_empty);
	proxy = atomic_load(&t4_saw_proxy);

	diag("word-atomicity: %ld all-set + %ld all-clear reads, %ld torn, %ld caught mid-flip (proxy)",
			full, empty, torn, proxy);
	ok(torn == 0, "word-atomicity: no reader ever saw a torn word (%ld torn)", torn);
	ok(full > 0 && empty > 0,
		"word-atomicity: readers observed both states (not vacuous)");
}

int main(void)
{
	plan_tests(NR_TESTS);
	rcu_register_thread();

	test_encoding();		/* 5 */
	test_compose_single();		/* 2 */
	test_fuse();			/* 6 */

	rcu_thread_offline();		/* T4 drives its own registered threads */
	test_word_atomicity();		/* 2 */
	rcu_thread_online();

	rcu_barrier();
	rcu_unregister_thread();
	return exit_status();
}
