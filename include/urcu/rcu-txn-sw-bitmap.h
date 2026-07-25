// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

#ifndef _URCU_RCU_TXN_SW_BITMAP_H
#define _URCU_RCU_TXN_SW_BITMAP_H

/*
 * rcu-txn-sw-bitmap.h
 *
 * The single-updater sibling of <urcu/rcu-txn-bitmap.h>: a fixed-size bitmap
 * laid over an array of transacted words, so that bit set/clear composes into
 * the SAME flip transaction (<urcu/rcu-txn-sw.h>) as the mutation it
 * accompanies -- e.g. a fractal-trie node's occupancy bit flips atomically with
 * the child-pointer store.  Writers must be mutually excluded, as with
 * cds_hlist_*_rcu(); reads are plain RCU (resolve the proxy, mask the bit).
 *
 * ENCODING -- identical to the concurrent twin, deliberately.  The engine owns
 * tag bit 0 of every transacted slot: a settled literal must have
 * (value & URCU_TXN_SW_BITMAP_TAG) != URCU_TXN_SW_BITMAP_TAG, i.e. bit 0 clear, or it is mistaken
 * for a parked proxy.  A bitmap word is all data, so we spend bit 0 as the tag
 * and keep 63 data bits per word (CAA_BITS_PER_LONG - 1): logical bit i lives
 * at PHYSICAL bit i+1, and a settled word always has bit 0 == 0.
 *
 *   word  w  = bit / BITS_PER_WORD
 *   phys  p  = 1 + bit % BITS_PER_WORD      (in 1..BITS_PER_WORD)
 *   mask     = (uintptr_t) 1 << p
 *
 * A zero-filled region is a valid empty bitmap (all bits clear, bit 0 clear),
 * so demand-zero / calloc'd storage needs no init.
 *
 * The tag is FIXED at bit 0 here, where <urcu/rcu-txn-sw-hlist.h> lets the
 * embedder pick one: an hlist slot is a POINTER, whose spare alignment bits
 * cost nothing and which an embedder may already tag under its own scheme.  A
 * bitmap word has no spare bits -- every bit is data, and the tag is SPENT from
 * the data space -- so widening it would shrink BITS_PER_WORD and move every
 * logical bit's physical position.  Holding it at bit 0 keeps this header's
 * layout bit-for-bit identical to <urcu/rcu-txn-bitmap.h>, so one words[] array
 * can be handed to either front-end (not to both at once -- see PARITY below).
 *
 * LAYOUT.  The API operates on a caller-provided `uintptr_t *words` (the bitmap
 * is embedded wherever the caller wants -- inside a node, a metadata block, ...)
 * plus the logical bit index.  Size the array with URCU_TXN_SW_BITMAP_NR_WORDS().
 *
 * READS must run inside an RCU read-side critical section of the flavor the
 * transactions use (they resolve proxies, whose lifetime is the RCU grace
 * period) -- exactly like urcu_txn_sw_list_*_rcu().  A reader resolves through
 * urcu_txn_sw_proxy_get(): one acquire load of the flip selector, never
 * blocking and never waiting on anyone.  The concurrent twin's read-policy
 * question ("a pure reader must not wait") simply does not arise -- this engine
 * has no UNDECIDED window at all, so a resolved word is always the flip's
 * linearized value, old before the selector store and new after.
 *
 * COMPOSITION -- and why this header composes where its sw siblings refuse to.
 * --------------------------------------------------------------------------
 * The transacted slot is the WORD, not the bit: 63 logical bits share one
 * physical word, so flips of DISTINCT bit indexes routinely land on the SAME
 * slot.  urcu_txn_sw_record() appends blindly and requires pairwise-distinct
 * slots, so recording each flip with it would park two proxies on one word and
 * settle them in record order, silently losing the earlier flip (install()'s
 * debug scan catches it; an NDEBUG build does not).  For a bitmap that
 * collision is the COMMON case, not a corner, so "compose only slot-disjoint
 * edits" -- the stance of <urcu/rcu-txn-sw-list.h> and
 * <urcu/rcu-txn-sw-hlist.h> -- would be close to unusable here.
 *
 * So the _prepare forms below record through the engine's read-your-own-writes
 * pair instead (urcu_txn_sw_load / urcu_txn_sw_record_chain, <urcu/rcu-txn-sw.h>):
 * a flip reads the word as this transaction will leave it and chains onto the
 * pending record rather than duplicating the slot.  Same-word flips fuse into
 * one record, so the write set stays slot-disjoint and composition on ONE
 * bitmap is sound -- the same guarantee the concurrent twin gets from
 * urcu_txn_load/urcu_txn_store, reached the same way.  The recorded old stays
 * the PRE-transaction word value: ptr[0] is the word as readers see it until
 * the selector store, ptr[1] the same word with every flip of this transaction
 * applied.
 *
 * Fusing is SUFFICIENT here and would not be for the sw list, an asymmetry
 * worth being explicit about.  A bitmap word's new value is a pure function of
 * THAT WORD's own old value, so reading the word's own pending state is the
 * whole job.  A list edit's new value is a NEIGHBOUR's pointer, so two edits
 * whose neighbourhoods touch build the write set from stale reads on
 * PAIRWISE-DISTINCT slots (the adjacent-delete trap in
 * urcu_txn_sw_list_add_after_prepare()) -- which same-slot fusion alone does
 * not repair; that traversal would have to read through urcu_txn_sw_load() too.
 * Hence: full composition here, disjoint-only there.
 *
 * COMPOSITION / rank consistency.  rank()/select() read across several words but
 * a set/clear transacts only the ONE word it changes.  When a compressed-array
 * index (= rank) is folded into a commit, the caller must ensure no bitmap
 * change that would move that rank races it -- under a single updater that is
 * free, since the updater making the commit is the only one that could.
 *
 * PARITY with <urcu/rcu-txn-bitmap.h>, and where it stops
 * ------------------------------------------------------
 * Same encoding, same accessor set, same _prepare shape, so a caller migrates
 * between the two mechanically.  Two differences are real:
 *
 *   - No ABORT and no escalation domain.  A single updater has no contention,
 *     so the _rcu forms take no domain, run no retry loop, and return only OK
 *     or MEMORY_ERROR.  A single-bit _rcu commit is one recorded edge, which
 *     takes the engine's nr == 1 fast path: a lone release store, with no proxy
 *     alloc, no install, no settle and no grace period -- strictly cheaper than
 *     the twin's install/CAS/settle cycle.
 *
 *   - No guarded multi-word snapshot.  The twin offers a strong one (a
 *     read-only transaction that urcu_txn_load_validate()s each word, retrying
 *     on ABORT, so the read set linearizes on one status-word CAS).  This
 *     engine has no read set, no validation and no abort, so that escape hatch
 *     has NO sw equivalent.  As with the twin, the _rcu scans below (rank,
 *     weight, find_*) are per-word hints, not snapshots: they hold no proxy and
 *     resolve each word against its own moment, so a scan can straddle a range
 *     commit.  If you need an atomic multi-word snapshot, use
 *     <urcu/rcu-txn-bitmap.h> even under a single updater.
 *
 * Include this header AFTER an RCU flavor header (e.g. <urcu-qsbr.h>): commit
 * reclaims parked proxies through that flavor's call_rcu().
 */

#include <errno.h>			/* ENOMEM */
#include <stddef.h>			/* size_t */
#include <stdint.h>			/* uintptr_t */

#include <urcu/compiler.h>		/* CAA_BITS_PER_LONG, caa_unlikely */
#include <urcu/uatomic.h>
#include <urcu/rcu-txn-status.h>	/* enum urcu_txn_status */
#include <urcu/rcu-txn-sw.h>		/* urcu_txn_sw_load/record_chain/commit/... */

/*
 * Proxy tag for every bitmap slot.  Override before include to drive the words
 * under an embedder's own tag; must satisfy (value & TAG) != TAG for every live
 * value a word holds (bit 0 clear -- every packed count is stored << 1).
 */
#ifndef URCU_TXN_SW_BITMAP_TAG
#define URCU_TXN_SW_BITMAP_TAG	1UL
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Data bits per transacted word (bit 0 is the engine proxy tag). */
#define URCU_TXN_SW_BITMAP_BITS_PER_WORD	((size_t) (CAA_BITS_PER_LONG - 1))

/* Number of transacted words needed to hold @nbits logical bits. */
#define URCU_TXN_SW_BITMAP_NR_WORDS(nbits)					\
	(((size_t) (nbits) + URCU_TXN_SW_BITMAP_BITS_PER_WORD - 1)		\
		/ URCU_TXN_SW_BITMAP_BITS_PER_WORD)

/* bit -> (word index, single-bit mask at the physical position). */
static inline
void urcu_txn_sw_bitmap__locate(size_t bit, size_t *word, uintptr_t *mask)
{
	*word = bit / URCU_TXN_SW_BITMAP_BITS_PER_WORD;
	*mask = (uintptr_t) 1 << (1 + bit % URCU_TXN_SW_BITMAP_BITS_PER_WORD);
}

/*
 * Resolved logical value of word @w (physical: bit 0 clear, data in 1..63).
 * Call within an RCU read-side section.
 *
 * A tagged proxy resolves through the flip selector -- one acquire load,
 * returning this word's old value until the owning transaction's selector store
 * and its new value after.  A settled literal passes through unchanged.
 */
static inline
uintptr_t urcu_txn_sw_bitmap_word_rcu(const uintptr_t *words, size_t w)
{
	uintptr_t v = (uintptr_t) uatomic_load(&((uintptr_t *) words)[w],
			CMM_ACQUIRE);

	return (uintptr_t) urcu_txn_sw_resolve((void *) v,
			URCU_TXN_SW_BITMAP_TAG);
}

/* True iff logical @bit is set.  Call within an RCU read-side section. */
static inline
int urcu_txn_sw_bitmap_test_rcu(const uintptr_t *words, size_t bit)
{
	size_t w;
	uintptr_t mask;

	urcu_txn_sw_bitmap__locate(bit, &w, &mask);
	return (urcu_txn_sw_bitmap_word_rcu(words, w) & mask) != 0;
}

/*
 * rank(bit) = number of set bits strictly below @bit = the index of @bit in a
 * popcount-compressed array.  Call within an RCU read-side section.  (Bit 0 of
 * every physical word is always clear, so full-word popcounts need no masking.)
 */
static inline
size_t urcu_txn_sw_bitmap_rank_rcu(const uintptr_t *words, size_t bit)
{
	size_t w = bit / URCU_TXN_SW_BITMAP_BITS_PER_WORD;
	unsigned phys = (unsigned) (1 + bit % URCU_TXN_SW_BITMAP_BITS_PER_WORD);
	uintptr_t below = ((uintptr_t) 1 << phys) - 1;	/* physical bits 0..phys-1 */
	size_t r = 0, i;

	for (i = 0; i < w; i++)
		r += (size_t) __builtin_popcountl(urcu_txn_sw_bitmap_word_rcu(words, i));
	r += (size_t) __builtin_popcountl(urcu_txn_sw_bitmap_word_rcu(words, w) & below);
	return r;
}

/* Total number of set bits over @nbits.  Call within an RCU read-side section. */
static inline
size_t urcu_txn_sw_bitmap_weight_rcu(const uintptr_t *words, size_t nbits)
{
	size_t nwords = URCU_TXN_SW_BITMAP_NR_WORDS(nbits), i, wt = 0;

	for (i = 0; i < nwords; i++)
		wt += (size_t) __builtin_popcountl(urcu_txn_sw_bitmap_word_rcu(words, i));
	return wt;
}

/*
 * Smallest set bit >= @from, or -1 if none in [from, nbits).  The iteration
 * primitive (e.g. walk a pigeon node's populated slots).  A hint: it is
 * per-word linearizable, not a snapshot.  Call within an RCU read-side section.
 */
static inline
long urcu_txn_sw_bitmap_ffs_from_rcu(const uintptr_t *words, size_t nbits,
		size_t from)
{
	size_t nwords = URCU_TXN_SW_BITMAP_NR_WORDS(nbits), w;
	unsigned start;
	uintptr_t word;

	if (from >= nbits)
		return -1;
	w = from / URCU_TXN_SW_BITMAP_BITS_PER_WORD;
	start = (unsigned) (1 + from % URCU_TXN_SW_BITMAP_BITS_PER_WORD);
	/* mask off physical bits below @from in the first word */
	word = urcu_txn_sw_bitmap_word_rcu(words, w) & ~(((uintptr_t) 1 << start) - 1);
	for (;;) {
		if (word) {
			unsigned p = (unsigned) __builtin_ctzl(word);	/* physical, >= 1 */
			long bit = (long) (w * URCU_TXN_SW_BITMAP_BITS_PER_WORD)
					+ (long) (p - 1);
			return bit < (long) nbits ? bit : -1;
		}
		if (++w >= nwords)
			return -1;
		word = urcu_txn_sw_bitmap_word_rcu(words, w);
	}
}

/*
 * The @i-th set bit (0-based), or -1 if fewer than @i+1 bits are set.  Inverse
 * of rank(); e.g. map a compressed-array index back to its key byte.  Call
 * within an RCU read-side section.
 */
static inline
long urcu_txn_sw_bitmap_select_rcu(const uintptr_t *words, size_t nbits, size_t i)
{
	size_t nwords = URCU_TXN_SW_BITMAP_NR_WORDS(nbits), w;

	for (w = 0; w < nwords; w++) {
		uintptr_t word = urcu_txn_sw_bitmap_word_rcu(words, w);
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
			long bit = (long) (w * URCU_TXN_SW_BITMAP_BITS_PER_WORD)
					+ (long) (p - 1);
			return bit < (long) nbits ? bit : -1;
		}
	}
	return -1;
}

/*
 * Fold @mask into word @w of @words (set it, or clear it), recording the edge
 * into @txn.  Reads the word through the engine's read-your-own-writes load, so
 * a second flip on a word this transaction already recorded sees the PENDING
 * value and chains onto that record rather than duplicating the slot.  Returns
 * 0, or -ENOMEM (sticky; also surfaces at commit).
 *
 * Line for line the concurrent twin's urcu_txn_bitmap_set_prepare() body, which
 * is the point: the fusing is the engine's, not this header's.
 */
static inline
int urcu_txn_sw_bitmap__word_edit(struct urcu_txn_sw_txn *txn, uintptr_t *words,
		size_t w, uintptr_t mask, int set)
{
	void **slot = (void **) &words[w];
	uintptr_t old;

	old = (uintptr_t) urcu_txn_sw_load(txn, slot, URCU_TXN_SW_BITMAP_TAG);
	return urcu_txn_sw_record_chain(txn, slot, (void *) old,
			(void *) (set ? (old | mask) : (old & ~mask)),
			URCU_TXN_SW_BITMAP_TAG) ? 0 : -ENOMEM;
}

/*
 * Composable writers: append the edge to caller-owned @txn (already init'd),
 * WITHOUT committing.  Fold together with the accompanying mutation's edges and
 * commit once.  Returns 0, or -ENOMEM (sticky; also surfaces at commit).
 *
 * Unlike this header's sw siblings, these compose freely on ONE bitmap: flips
 * of distinct bits sharing a word fuse into a single record.  See the header
 * intro.
 */
static inline
int urcu_txn_sw_bitmap_set_prepare(struct urcu_txn_sw_txn *txn, uintptr_t *words,
		size_t bit)
{
	size_t w;
	uintptr_t mask;

	urcu_txn_sw_bitmap__locate(bit, &w, &mask);
	return urcu_txn_sw_bitmap__word_edit(txn, words, w, mask, 1);
}

static inline
int urcu_txn_sw_bitmap_clear_prepare(struct urcu_txn_sw_txn *txn, uintptr_t *words,
		size_t bit)
{
	size_t w;
	uintptr_t mask;

	urcu_txn_sw_bitmap__locate(bit, &w, &mask);
	return urcu_txn_sw_bitmap__word_edit(txn, words, w, mask, 0);
}

/*
 * Composable range writers: set/clear logical bits [lo, hi) as one atomic
 * multi-word update (each spanned word one edge).  Delivers the property a
 * per-bit atomic cannot: a reader never sees a torn range.  Overlapping an
 * earlier flip -- another range, or a single bit -- is fine: the shared words
 * fuse.
 */
static inline
int urcu_txn_sw_bitmap__range_prepare(struct urcu_txn_sw_txn *txn, uintptr_t *words,
		size_t lo, size_t hi, int set)
{
	size_t bpw = URCU_TXN_SW_BITMAP_BITS_PER_WORD;

	while (lo < hi) {
		size_t w = lo / bpw;
		unsigned p_lo = (unsigned) (1 + lo % bpw);
		size_t word_end = (w + 1) * bpw;		/* first bit of next word */
		size_t seg_hi = hi < word_end ? hi : word_end;
		unsigned p_hi = (unsigned) (1 + (seg_hi - 1) % bpw);	/* inclusive */
		uintptr_t mask;
		int ret;

		/* physical bits p_lo..p_hi inclusive */
		mask = (p_hi == CAA_BITS_PER_LONG - 1)
			? ~(uintptr_t) 0 : (((uintptr_t) 1 << (p_hi + 1)) - 1);
		mask &= ~(((uintptr_t) 1 << p_lo) - 1);
		ret = urcu_txn_sw_bitmap__word_edit(txn, words, w, mask, set);
		if (ret)
			return ret;
		lo = seg_hi;
	}
	return 0;
}

static inline
int urcu_txn_sw_bitmap_set_range_prepare(struct urcu_txn_sw_txn *txn,
		uintptr_t *words, size_t lo, size_t hi)
{
	return urcu_txn_sw_bitmap__range_prepare(txn, words, lo, hi, 1);
}

static inline
int urcu_txn_sw_bitmap_clear_range_prepare(struct urcu_txn_sw_txn *txn,
		uintptr_t *words, size_t lo, size_t hi)
{
	return urcu_txn_sw_bitmap__range_prepare(txn, words, lo, hi, 0);
}

/*
 * Self-contained set/clear: own init/commit, no retry loop (a single updater
 * has no contention, so commit never returns ABORT) and no escalation domain.
 * For standalone use and tests; the fractal trie composes via the _prepare
 * forms instead.  Returns URCU_TXN_STATUS_OK, or URCU_TXN_STATUS_MEMORY_ERROR.
 *
 * One bit is one edge, so commit takes the engine's nr == 1 fast path: a lone
 * release store to the word, with no proxy, no group block and no grace period.
 * (Commit uses the compile-time-selected RCU flavor's call_rcu -- include a
 * flavor header first.)
 */
static inline
enum urcu_txn_status urcu_txn_sw_bitmap_set_rcu(uintptr_t *words, size_t bit)
{
	struct urcu_txn_sw_txn txn;

	urcu_txn_sw_init(&txn);
	urcu_txn_sw_declare_disjoint(&txn);	/* single-op commit: one word, no same-slot WAW */
	(void) urcu_txn_sw_bitmap_set_prepare(&txn, words, bit);
	return urcu_txn_sw_commit(&txn);
}

static inline
enum urcu_txn_status urcu_txn_sw_bitmap_clear_rcu(uintptr_t *words, size_t bit)
{
	struct urcu_txn_sw_txn txn;

	urcu_txn_sw_init(&txn);
	urcu_txn_sw_declare_disjoint(&txn);	/* single-op commit: one word, no same-slot WAW */
	(void) urcu_txn_sw_bitmap_clear_prepare(&txn, words, bit);
	return urcu_txn_sw_commit(&txn);
}

#ifdef __cplusplus
}
#endif

#endif /* _URCU_RCU_TXN_SW_BITMAP_H */
