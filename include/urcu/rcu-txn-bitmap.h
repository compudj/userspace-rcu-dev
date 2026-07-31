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
 * literal must have (value & URCU_TXN_TAG) != URCU_TXN_TAG, i.e. bit 0 clear,
 * or it is mistaken for an in-flight descriptor (rcu-txn-mcas.h tag contract).  A
 * bitmap word is all data, so we spend bit 0 as the tag and keep 63 data bits
 * per word (CAA_BITS_PER_LONG - 1).  This is exactly the engine's documented
 * "store small integers shifted left by 1" discipline: logical bit i lives at
 * PHYSICAL bit i+1, and a settled word always has bit 0 == 0.  No engine change
 * and no sentinel carve-out are needed, and the tag stays the narrow bit-0
 * URCU_TXN_TAG so a bitmap word can share one commit with pointer slots that
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
 *
 * That covers a CONCURRENT writer moving the rank.  It says nothing about this
 * transaction's OWN pending flips, which are in the commit rather than in
 * conflict with it -- and the _rcu accessors cannot see them.  A bracket that
 * computes a rank after recording a flip must read through the _txn accessors;
 * see the block above urcu_txn_bitmap_test_rcu().
 *
 * COMPOSITION / the transacted slot is the WORD, not the bit.  Composing
 * several _prepare flips in one transaction REQUIRES the default
 * (read-your-own-writes) handle.  63 logical bits share one physical word, so
 * flips of DISTINCT bit indexes routinely land on the SAME slot -- which means
 * a handle that declared its write set disjoint (urcu_txn_declare_disjoint(),
 * <urcu/rcu-txn.h>) is WRONG here even though every bit index is distinct: it
 * blind-appends a second record on that word, and the commit installs both and
 * settles them in record order, silently losing the earlier flip while
 * reporting OK.  On the default handle the second flip instead reads the word's
 * PENDING value and chains onto the existing record, fusing same-word flips
 * into one.  Use the range forms for a contiguous run.
 */

#include <stddef.h>			/* size_t */
#include <stdint.h>			/* uintptr_t */

#include <urcu/compiler.h>		/* CAA_BITS_PER_LONG, caa_likely */
#include <urcu/rcu-txn-mcas.h>		/* urcu_txn_read_optimistic, URCU_TXN_TAG */
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
	/*
	 * Optimistic: a pure reader never waits.  An UNDECIDED transaction has not
	 * linearized, so this word's logical value IS its old_ptr -- exactly what
	 * urcu_txn_read_optimistic() returns -- and waiting would make every bitmap
	 * probe spin on a stranger's install.
	 *
	 * The rule (measured): wait iff the loaded slot belongs to the caller's own
	 * read/write set, because there a stale value dooms the install-time CAS and
	 * costs an abort.  Read optimistically otherwise.  urcu_txn_bitmap_*_prepare
	 * below load the very word they store, so they keep the waiting urcu_txn_load;
	 * this accessor stores nothing, so it must not wait.
	 *
	 * ⚠ This accessor -- and the _rcu scans built on it (rank, weight, find_*) --
	 * never was a multi-word snapshot: it holds no descriptor, records nothing,
	 * and resolves each word against its own moment, so a scan can straddle a
	 * range commit under EITHER policy.  Not waiting does widen that window (a
	 * waiting scan, once it touches one word of a writer's range, tends to block
	 * until that writer's owner settles it, and so to see one decision for the
	 * rest of the range -- tends to, because the wait is bounded and a scan that
	 * caps out straddles exactly as this one does).  Nothing guaranteed is lost,
	 * because nothing was guaranteed.
	 *
	 * The multi-word guarantee lives elsewhere, and is STRONG: take a read-only
	 * transaction and urcu_txn_load_validate() each word, retrying on ABORT (the
	 * guarded-snapshot pattern in tests/unit/test_rcu_txn_bitmap.c, T4).  Each
	 * such load emits a full old == new MCAS record -- it PLANTS a proxy in the
	 * word, so a concurrent writer's plant CAS off that word conflicts, and the
	 * whole read set linearizes on the transaction's single status-word CAS.
	 * That is a genuine atomic multi-word snapshot, not a re-check.  It must
	 * be the WAITING load: a guard's expected value read without helping can
	 * be an undecided parker's logical old, which dooms the install the moment
	 * that parker commits -- which is why there is no
	 * urcu_txn_load_validate_optimistic() to reach for here.
	 */
	return (uintptr_t) urcu_txn_read_optimistic(
			(void **) &((uintptr_t *) words)[w], URCU_TXN_TAG);
}

/*
 * THE _rcu ACCESSORS BELOW RETURN COMMITTED STATE.  They are handle-less, so
 * inside a bracket they cannot see this transaction's own pending flips: a
 * store is BUFFERED until commit, and the RYW consult lives in urcu_txn_load(),
 * which needs the handle.
 *
 * That matters exactly where this header advertises composition.  Fold two
 * occupancy inserts into one commit, and computing the second one's compressed
 * index with urcu_txn_bitmap_rank_rcu() counts the pre-transaction bits: the
 * first insert's flip is not there yet, so the index is off by one.  Nothing
 * catches it -- the word record chains both flips correctly, the array edits
 * validate against the committed values the caller genuinely loaded, and the
 * commit installs and returns OK, leaving the compressed array inconsistent
 * with the ranks the committed bitmap implies.
 *
 * So inside a bracket, read through the _txn forms further down, which take the
 * handle and therefore see the transaction's own flips.  Age 0 answers such a
 * read with the committed value and arms the private abort, so the bracket
 * re-runs at age 1+ where the write set is consulted exactly -- the engine's
 * intended escalation, and the reason a composed batch wants
 * urcu_txn_expect_conflict().
 */

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

/*
 * IN-BRACKET counterparts of word_rcu / test_rcu / rank_rcu: they read through
 * the transaction, so they see its own pending flips.  Use these for any value
 * the bracket COMPUTES A WRITE SITE FROM -- a compressed-array index above all.
 * The word enters the read/write set, which is what makes the read policy hold
 * (see <urcu/rcu-txn.h>): these are waiting loads, and a word this transaction
 * has already flipped resolves to the pending value at age 1+.
 */
static inline
uintptr_t urcu_txn_bitmap_word_txn(struct urcu_txn *txn, const uintptr_t *words,
		size_t w)
{
	return (uintptr_t) urcu_txn_load(txn,
			(void **) &((uintptr_t *) words)[w], URCU_TXN_TAG);
}

static inline
int urcu_txn_bitmap_test_txn(struct urcu_txn *txn, const uintptr_t *words,
		size_t bit)
{
	size_t w;
	uintptr_t mask;

	urcu_txn_bitmap__locate(bit, &w, &mask);
	return (urcu_txn_bitmap_word_txn(txn, words, w) & mask) != 0;
}

static inline
size_t urcu_txn_bitmap_rank_txn(struct urcu_txn *txn, const uintptr_t *words,
		size_t bit)
{
	size_t w = bit / URCU_TXN_BITMAP_BITS_PER_WORD;
	unsigned phys = (unsigned) (1 + bit % URCU_TXN_BITMAP_BITS_PER_WORD);
	uintptr_t below = ((uintptr_t) 1 << phys) - 1;	/* physical bits 0..phys-1 */
	size_t r = 0, i;

	for (i = 0; i < w; i++)
		r += (size_t) __builtin_popcountl(
				urcu_txn_bitmap_word_txn(txn, words, i));
	r += (size_t) __builtin_popcountl(
			urcu_txn_bitmap_word_txn(txn, words, w) & below);
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
 *
 * @txn must be a DEFAULT handle, never one that declared its write set
 * disjoint:  the slot is the 63-bit WORD, so two flips of distinct bit indexes
 * can record the same slot.  See the composition note in the header intro.
 */
static inline
int urcu_txn_bitmap_set_prepare(struct urcu_txn *txn, uintptr_t *words,
		size_t bit)
{
	size_t w;
	uintptr_t mask, old;

	urcu_txn_bitmap__locate(bit, &w, &mask);
	old = (uintptr_t) urcu_txn_load(txn, (void **) &words[w], URCU_TXN_TAG);
	return urcu_txn_store_mw(txn, (void **) &words[w],
			(void *) old, (void *) (old | mask), URCU_TXN_TAG);
}

static inline
int urcu_txn_bitmap_clear_prepare(struct urcu_txn *txn, uintptr_t *words,
		size_t bit)
{
	size_t w;
	uintptr_t mask, old;

	urcu_txn_bitmap__locate(bit, &w, &mask);
	old = (uintptr_t) urcu_txn_load(txn, (void **) &words[w], URCU_TXN_TAG);
	return urcu_txn_store_mw(txn, (void **) &words[w],
			(void *) old, (void *) (old & ~mask), URCU_TXN_TAG);
}

/*
 * Composable range writers: set/clear logical bits [lo, hi) as one atomic
 * multi-word update (each spanned word one edge).  Demonstrates -- and delivers
 * -- the property a per-bit atomic cannot: a reader never sees a torn range.
 */
static inline
int urcu_txn_bitmap__range_prepare(struct urcu_txn *txn, uintptr_t *words,
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
		old = (uintptr_t) urcu_txn_load(txn, (void **) &words[w], URCU_TXN_TAG);
		ret = urcu_txn_store_mw(txn, (void **) &words[w], (void *) old,
				(void *) (set ? (old | mask) : (old & ~mask)),
				URCU_TXN_TAG);
		if (ret)
			return ret;
		lo = seg_hi;
	}
	return 0;
}

static inline
int urcu_txn_bitmap_set_range_prepare(struct urcu_txn *txn,
		uintptr_t *words, size_t lo, size_t hi)
{
	return urcu_txn_bitmap__range_prepare(txn, words, lo, hi, 1);
}

static inline
int urcu_txn_bitmap_clear_range_prepare(struct urcu_txn *txn,
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
	struct urcu_txn txn;
	enum urcu_txn_status st;

	urcu_txn_init(&txn, domain);
	urcu_txn_declare_disjoint(&txn);	/* single-op commit: one word, no same-slot WAW */
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
	struct urcu_txn txn;
	enum urcu_txn_status st;

	urcu_txn_init(&txn, domain);
	urcu_txn_declare_disjoint(&txn);	/* single-op commit: one word, no same-slot WAW */
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
