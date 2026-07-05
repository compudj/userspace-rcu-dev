// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

#ifndef _URCU_RCU_TXN_BITMAP_H
#define _URCU_RCU_TXN_BITMAP_H

/*
 * rcu-txn-bitmap.h
 *
 * A fixed-size bitmap laid over an array of transacted words, so that bit
 * set/clear composes into the SAME urcu-txn (MCAS) commit as the mutation it
 * accompanies -- e.g. a fractal-trie node's occupancy bit flips atomically with
 * the child-pointer store, turning a copy-on-write node recompaction into one
 * in-place transaction.  Reads are plain RCU (resolve the proxy, mask the bit).
 *
 * ENCODING.  The engine owns tag bit 0 of every transacted slot: a settled
 * literal must have (value & URCU_MCAS_TAG) != URCU_MCAS_TAG, i.e. bit 0 clear,
 * or it is mistaken for an in-flight descriptor (rcu-mcas.h tag contract).  A
 * bitmap word is all data, so we spend bit 0 as the tag and keep 63 data bits
 * per word (CAA_BITS_PER_LONG - 1).  This is exactly the engine's documented
 * "store small integers shifted left by 1" discipline: logical bit i lives at
 * PHYSICAL bit i+1, and a settled word always has bit 0 == 0.  No engine change
 * and no sentinel carve-out are needed, and the tag stays the narrow bit-0
 * URCU_MCAS_TAG so a bitmap word can share one commit with pointer slots that
 * carry a wider per-record tag (the fractal trie's typed pointers).
 *
 *   word  w  = bit / BITS_PER_WORD
 *   phys  p  = 1 + bit % BITS_PER_WORD      (in 1..BITS_PER_WORD)
 *   mask     = (uintptr_t) 1 << p
 *
 * A zero-filled region is a valid empty bitmap (all bits clear, bit 0 clear),
 * so demand-zero / calloc'd storage needs no init.
 *
 * LAYOUT.  The API operates on a caller-provided `uintptr_t *words` (the bitmap
 * is embedded wherever the caller wants -- inside a node, a metadata block, ...)
 * plus the logical bit index.  Size the array with URCU_TXN_BITMAP_NR_WORDS().
 *
 * READS must run inside an RCU read-side critical section of the flavor the
 * transactions use (they resolve descriptors, whose lifetime is the RCU grace
 * period) -- exactly like urcu_txn_list_*_rcu().  The _prepare writers append
 * edges to a caller-owned, already-begun transaction; the _rcu writers are
 * self-contained (their own begin/commit/end + retry loop).
 *
 * COMPOSITION / rank consistency.  rank()/select() read across several words but
 * a set/clear transacts only the ONE word it changes.  When a compressed-array
 * index (= rank) is folded into a commit, the caller must make any bitmap change
 * that would move that rank conflict with the commit -- typically the array
 * shift/tail edges already do (a lower bit's insert shifts the very slot the
 * higher bit's insert claims).  rank() is otherwise an optimistic read: the
 * word's value-CAS at commit re-validates the bits within that word.
 */

#include <stddef.h>			/* size_t */
#include <stdint.h>			/* uintptr_t */

#include <urcu/compiler.h>		/* CAA_BITS_PER_LONG, caa_likely */
#include <urcu/rcu-mcas.h>		/* urcu_mcas_read, URCU_MCAS_TAG */
#include <urcu/rcu-txn.h>		/* urcu_txn_load/store/begin/commit/... */

#ifdef __cplusplus
extern "C" {
#endif

/* Data bits per transacted word (bit 0 is the engine proxy tag). */
#define URCU_TXN_BITMAP_BITS_PER_WORD	((size_t) (CAA_BITS_PER_LONG - 1))

/* Number of transacted words needed to hold @nbits logical bits. */
#define URCU_TXN_BITMAP_NR_WORDS(nbits)					\
	(((size_t) (nbits) + URCU_TXN_BITMAP_BITS_PER_WORD - 1)		\
		/ URCU_TXN_BITMAP_BITS_PER_WORD)

/* bit -> (word index, single-bit mask at the physical position). */
static inline
void urcu_txn_bitmap__locate(size_t bit, size_t *word, uintptr_t *mask)
{
	*word = bit / URCU_TXN_BITMAP_BITS_PER_WORD;
	*mask = (uintptr_t) 1 << (1 + bit % URCU_TXN_BITMAP_BITS_PER_WORD);
}

/*
 * Resolved logical value of word @w (physical: bit 0 clear, data in 1..63).
 * Call within an RCU read-side section.
 */
static inline
uintptr_t urcu_txn_bitmap_word_rcu(const uintptr_t *words, size_t w)
{
	return (uintptr_t) urcu_mcas_read((void **) &((uintptr_t *) words)[w],
			URCU_MCAS_TAG);
}

/* True iff logical @bit is set.  Call within an RCU read-side section. */
static inline
int urcu_txn_bitmap_test_rcu(const uintptr_t *words, size_t bit)
{
	size_t w;
	uintptr_t mask;

	urcu_txn_bitmap__locate(bit, &w, &mask);
	return (urcu_txn_bitmap_word_rcu(words, w) & mask) != 0;
}

/*
 * rank(bit) = number of set bits strictly below @bit = the index of @bit in a
 * popcount-compressed array.  Call within an RCU read-side section.  (Bit 0 of
 * every physical word is always clear, so full-word popcounts need no masking.)
 */
static inline
size_t urcu_txn_bitmap_rank_rcu(const uintptr_t *words, size_t bit)
{
	size_t w = bit / URCU_TXN_BITMAP_BITS_PER_WORD;
	unsigned phys = (unsigned) (1 + bit % URCU_TXN_BITMAP_BITS_PER_WORD);
	uintptr_t below = ((uintptr_t) 1 << phys) - 1;	/* physical bits 0..phys-1 */
	size_t r = 0, i;

	for (i = 0; i < w; i++)
		r += (size_t) __builtin_popcountl(urcu_txn_bitmap_word_rcu(words, i));
	r += (size_t) __builtin_popcountl(urcu_txn_bitmap_word_rcu(words, w) & below);
	return r;
}

/* Total number of set bits over @nbits.  Call within an RCU read-side section. */
static inline
size_t urcu_txn_bitmap_weight_rcu(const uintptr_t *words, size_t nbits)
{
	size_t nwords = URCU_TXN_BITMAP_NR_WORDS(nbits), i, wt = 0;

	for (i = 0; i < nwords; i++)
		wt += (size_t) __builtin_popcountl(urcu_txn_bitmap_word_rcu(words, i));
	return wt;
}

/*
 * Smallest set bit >= @from, or -1 if none in [from, nbits).  The iteration
 * primitive (e.g. walk a pigeon node's populated slots).  A hint: it is
 * per-word linearizable, not a snapshot.  Call within an RCU read-side section.
 */
static inline
long urcu_txn_bitmap_ffs_from_rcu(const uintptr_t *words, size_t nbits,
		size_t from)
{
	size_t nwords = URCU_TXN_BITMAP_NR_WORDS(nbits), w;
	unsigned start;
	uintptr_t word;

	if (from >= nbits)
		return -1;
	w = from / URCU_TXN_BITMAP_BITS_PER_WORD;
	start = (unsigned) (1 + from % URCU_TXN_BITMAP_BITS_PER_WORD);
	/* mask off physical bits below @from in the first word */
	word = urcu_txn_bitmap_word_rcu(words, w) & ~(((uintptr_t) 1 << start) - 1);
	for (;;) {
		if (word) {
			unsigned p = (unsigned) __builtin_ctzl(word);	/* physical, >= 1 */
			long bit = (long) (w * URCU_TXN_BITMAP_BITS_PER_WORD)
					+ (long) (p - 1);
			return bit < (long) nbits ? bit : -1;
		}
		if (++w >= nwords)
			return -1;
		word = urcu_txn_bitmap_word_rcu(words, w);
	}
}

/*
 * The @i-th set bit (0-based), or -1 if fewer than @i+1 bits are set.  Inverse
 * of rank(); e.g. map a compressed-array index back to its key byte.  Call
 * within an RCU read-side section.
 */
static inline
long urcu_txn_bitmap_select_rcu(const uintptr_t *words, size_t nbits, size_t i)
{
	size_t nwords = URCU_TXN_BITMAP_NR_WORDS(nbits), w;

	for (w = 0; w < nwords; w++) {
		uintptr_t word = urcu_txn_bitmap_word_rcu(words, w);
		unsigned pop = (unsigned) __builtin_popcountl(word);

		if (i >= pop) {
			i -= pop;
			continue;
		}
		/* the (i+1)-th set bit is in this word */
		while (i--)
			word &= word - 1;		/* drop lowest set bit */
		{
			unsigned p = (unsigned) __builtin_ctzl(word);	/* physical, >= 1 */
			long bit = (long) (w * URCU_TXN_BITMAP_BITS_PER_WORD)
					+ (long) (p - 1);
			return bit < (long) nbits ? bit : -1;
		}
	}
	return -1;
}

/*
 * Composable writers: append the edge to caller-owned @txn (already begun),
 * WITHOUT committing.  Fold together with the accompanying mutation's edges and
 * commit once.  Returns 0, or -ENOMEM (sticky; also surfaces at commit).
 */
static inline
int urcu_txn_bitmap_set_prepare(struct urcu_mcas_txn *txn, uintptr_t *words,
		size_t bit)
{
	size_t w;
	uintptr_t mask, old;

	urcu_txn_bitmap__locate(bit, &w, &mask);
	old = (uintptr_t) urcu_txn_load(txn, (void **) &words[w], URCU_MCAS_TAG);
	return urcu_txn_store(txn, (void **) &words[w],
			(void *) old, (void *) (old | mask), URCU_MCAS_TAG);
}

static inline
int urcu_txn_bitmap_clear_prepare(struct urcu_mcas_txn *txn, uintptr_t *words,
		size_t bit)
{
	size_t w;
	uintptr_t mask, old;

	urcu_txn_bitmap__locate(bit, &w, &mask);
	old = (uintptr_t) urcu_txn_load(txn, (void **) &words[w], URCU_MCAS_TAG);
	return urcu_txn_store(txn, (void **) &words[w],
			(void *) old, (void *) (old & ~mask), URCU_MCAS_TAG);
}

/*
 * Composable range writers: set/clear logical bits [lo, hi) as one atomic
 * multi-word update (each spanned word one edge).  Demonstrates -- and delivers
 * -- the property a per-bit atomic cannot: a reader never sees a torn range.
 */
static inline
int urcu_txn_bitmap__range_prepare(struct urcu_mcas_txn *txn, uintptr_t *words,
		size_t lo, size_t hi, int set)
{
	size_t bpw = URCU_TXN_BITMAP_BITS_PER_WORD;

	while (lo < hi) {
		size_t w = lo / bpw;
		unsigned p_lo = (unsigned) (1 + lo % bpw);
		size_t word_end = (w + 1) * bpw;		/* first bit of next word */
		size_t seg_hi = hi < word_end ? hi : word_end;
		unsigned p_hi = (unsigned) (1 + (seg_hi - 1) % bpw);	/* inclusive */
		uintptr_t mask, old;
		int ret;

		/* physical bits p_lo..p_hi inclusive */
		mask = (p_hi == CAA_BITS_PER_LONG - 1)
			? ~(uintptr_t) 0 : (((uintptr_t) 1 << (p_hi + 1)) - 1);
		mask &= ~(((uintptr_t) 1 << p_lo) - 1);
		old = (uintptr_t) urcu_txn_load(txn, (void **) &words[w], URCU_MCAS_TAG);
		ret = urcu_txn_store(txn, (void **) &words[w], (void *) old,
				(void *) (set ? (old | mask) : (old & ~mask)),
				URCU_MCAS_TAG);
		if (ret)
			return ret;
		lo = seg_hi;
	}
	return 0;
}

static inline
int urcu_txn_bitmap_set_range_prepare(struct urcu_mcas_txn *txn,
		uintptr_t *words, size_t lo, size_t hi)
{
	return urcu_txn_bitmap__range_prepare(txn, words, lo, hi, 1);
}

static inline
int urcu_txn_bitmap_clear_range_prepare(struct urcu_mcas_txn *txn,
		uintptr_t *words, size_t lo, size_t hi)
{
	return urcu_txn_bitmap__range_prepare(txn, words, lo, hi, 0);
}

/*
 * Self-contained set/clear: own begin/commit/end + contention retry loop, over
 * escalation @domain (or NULL for no fallback).  For standalone use and tests;
 * the fractal trie composes via the _prepare forms instead.  Returns
 * URCU_TXN_STATUS_OK, or URCU_TXN_STATUS_MEMORY_ERROR.  (Commit uses the
 * compile-time-selected RCU flavor's call_rcu -- include a flavor header first.)
 */
static inline
enum urcu_txn_status urcu_txn_bitmap_set_rcu(struct urcu_txn_domain *domain,
		uintptr_t *words, size_t bit)
{
	struct urcu_mcas_txn txn;
	enum urcu_txn_status st;

	urcu_txn_init(&txn, domain);
	do {
		urcu_txn_begin(&txn);
		(void) urcu_txn_bitmap_set_prepare(&txn, words, bit);
		st = urcu_txn_commit(&txn);
		urcu_txn_end(&txn);
	} while (st == URCU_TXN_STATUS_ABORT);
	return st;
}

static inline
enum urcu_txn_status urcu_txn_bitmap_clear_rcu(struct urcu_txn_domain *domain,
		uintptr_t *words, size_t bit)
{
	struct urcu_mcas_txn txn;
	enum urcu_txn_status st;

	urcu_txn_init(&txn, domain);
	do {
		urcu_txn_begin(&txn);
		(void) urcu_txn_bitmap_clear_prepare(&txn, words, bit);
		st = urcu_txn_commit(&txn);
		urcu_txn_end(&txn);
	} while (st == URCU_TXN_STATUS_ABORT);
	return st;
}

#ifdef __cplusplus
}
#endif

#endif /* _URCU_RCU_TXN_BITMAP_H */
