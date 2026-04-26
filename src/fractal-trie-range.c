// SPDX-FileCopyrightText: 2026 EfficiOS Inc.
//
// SPDX-License-Identifier: LGPL-2.1-or-later

/*
 * src/fractal-trie-range.c
 *
 * Range index layered on top of cds_ft. See urcu/fractal-trie-range.h
 * for the user-facing semantics. This file is the implementation only.
 */

#include <urcu/fractal-trie-range.h>

#include <limits.h>
#include <stdlib.h>
#include <string.h>

#include <urcu/compiler.h>
#include <urcu/fractal-trie.h>
#include <urcu/pointer.h>

/*
 * Internal handle. The per-level cds_ft instances are created lazily
 * on first insert at each level: most realistic workloads cluster at
 * a few orders of magnitude of length and the rest of the slots stay
 * NULL.
 *
 * The level pointers are written by cds_ft_range_insert() under the
 * caller's writer mutex and read by both readers (under RCU) and
 * subsequent writers. Readers observe either the new pointer or NULL,
 * never a torn value, because the publication uses rcu_assign_pointer.
 *
 * The group is owned by the caller (passed to cds_ft_range_create);
 * the range index does not destroy it.
 */
struct cds_ft_range {
	struct cds_ft_group *group;
	struct cds_ft *level[CDS_FT_RANGE_NR_LEVELS];
};

struct cds_ft_range_attr {
	int placeholder;	/* reserved for future flags */
};

/*
 * Per-iterator state.
 *
 * The iterator walks per-level cds_ft tries in increasing-k order
 * over [k_min, k_max].  Within each level it scans trie keys in
 * [scan_lo, scan_hi), determined by one of two strategies:
 *
 *   HALO: per-level [max(0, halo_q_a - 2^k), halo_q_b).  Used by
 *         overlap-style scans that must cover ranges starting up to
 *         one cell-width before the query window so they can reach
 *         into it.  This is the natural shape for lookup_overlap and
 *         the leading/trailing strip variants used by entering /
 *         leaving when filtering on the end position.
 *
 *   FIXED: same [scan_start_lo, scan_start_hi) at every level, with
 *          no halo.  Used when the start position itself defines
 *          membership (entering pan-right: start in
 *          [old_q_b, new_q_b); leaving pan-left: start in
 *          [new_q_b, old_q_b)).
 *
 * Within each trie key position, the duplicate chain is walked and
 * each entry is admitted iff:
 *   end_floor < end <= end_ceil  &&  length_lo <= L < length_hi
 * (with end_ceil = UINT64_MAX and length_hi = UINT64_MAX denoting
 * no upper bound).
 */
enum cds_ft_range_scan {
	CDS_FT_RANGE_SCAN_HALO,
	CDS_FT_RANGE_SCAN_FIXED,
};

struct cds_ft_range_iter {
	struct cds_ft_range *ftr;

	/* Per-range filter. */
	uint64_t end_floor;		/* require: end > end_floor */
	uint64_t end_ceil;		/* require: end <= end_ceil; UINT64_MAX = unrestricted */
	uint64_t length_lo;		/* require: L >= length_lo (granularity floor) */
	uint64_t length_hi;		/* require: L < length_hi; UINT64_MAX = unrestricted */

	/* Scan window. */
	enum cds_ft_range_scan scan_strategy;
	uint64_t halo_q_a, halo_q_b;	/* HALO mode: per-level [halo_q_a - 2^k, halo_q_b) */
	uint64_t scan_start_lo;		/* FIXED mode: per-level [scan_start_lo, scan_start_hi) */
	uint64_t scan_start_hi;

	/* Level range to visit (inclusive). */
	unsigned int k_min, k_max;
	unsigned int level;

	/* Active scan state. */
	uint64_t scan_lo, scan_hi;
	struct cds_ft_iter *ft_iter;
	struct cds_ft_node *chain_pos;
	struct cds_ft_range_node *current;
	bool exhausted;
};

/* ------------------------------------------------------------------ */
/* Routing                                                            */
/* ------------------------------------------------------------------ */

unsigned int cds_ft_range_route_level(uint64_t start, uint64_t end)
{
	uint64_t L;

	if (end <= start)
		return CDS_FT_RANGE_NR_LEVELS;
	L = end - start;
	if (L == 1)
		return 0;
	/* ceil(log2(L)) for L >= 2 */
	return 64u - (unsigned int) __builtin_clzll(L - 1);
}

/* Length-class threshold at level k: max L stored is 2^k. */
static inline
uint64_t level_max_len(unsigned int k)
{
	if (k >= CDS_FT_RANGE_NR_LEVELS - 1)
		return UINT64_MAX;	/* 2^63 saturates; treat top level as unbounded */
	return 1ULL << k;
}

/* Smallest level whose ranges might satisfy length >= granularity. */
static inline
unsigned int min_level_for_granularity(uint64_t granularity)
{
	if (granularity <= 1)
		return 0;
	/* Smallest k with 2^k >= granularity. */
	return 64u - (unsigned int) __builtin_clzll(granularity - 1);
}

/* ------------------------------------------------------------------ */
/* Attr                                                               */
/* ------------------------------------------------------------------ */

enum cds_ft_status cds_ft_range_attr_create(struct cds_ft_range_attr **result)
{
	struct cds_ft_range_attr *attr;

	attr = (struct cds_ft_range_attr *) calloc(1, sizeof(*attr));
	if (!attr) {
		*result = NULL;
		return CDS_FT_STATUS_MEMORY_ERROR;
	}
	*result = attr;
	return CDS_FT_STATUS_OK;
}

void cds_ft_range_attr_destroy(struct cds_ft_range_attr *attr)
{
	free(attr);
}

/* ------------------------------------------------------------------ */
/* Lifecycle                                                          */
/* ------------------------------------------------------------------ */

enum cds_ft_status cds_ft_range_create(
		struct cds_ft_group *group,
		const struct cds_ft_range_attr *attr __attribute__((unused)),
		struct cds_ft_range **result)
{
	struct cds_ft_range *ftr;

	*result = NULL;
	if (!group)
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;

	ftr = (struct cds_ft_range *) calloc(1, sizeof(*ftr));
	if (!ftr)
		return CDS_FT_STATUS_MEMORY_ERROR;
	ftr->group = group;
	*result = ftr;
	return CDS_FT_STATUS_OK;
}

void cds_ft_range_destroy(struct cds_ft_range *ftr)
{
	unsigned int k;

	if (!ftr)
		return;
	for (k = 0; k < CDS_FT_RANGE_NR_LEVELS; k++) {
		if (ftr->level[k])
			cds_ft_destroy(ftr->level[k]);
	}
	free(ftr);
}

/* ------------------------------------------------------------------ */
/* Per-level trie creation (lazy, under writer mutex)                 */
/* ------------------------------------------------------------------ */

static
enum cds_ft_status ensure_level(struct cds_ft_range *ftr, unsigned int k,
		struct cds_ft **out)
{
	struct cds_ft *trie;
	enum cds_ft_status s;

	trie = rcu_dereference(ftr->level[k]);
	if (trie) {
		*out = trie;
		return CDS_FT_STATUS_OK;
	}
	s = cds_ft_create(ftr->group, NULL, &trie);
	if (s < 0) {
		*out = NULL;
		return s;
	}
	rcu_assign_pointer(ftr->level[k], trie);
	*out = trie;
	return CDS_FT_STATUS_OK;
}

/* ------------------------------------------------------------------ */
/* Insert / Remove                                                    */
/* ------------------------------------------------------------------ */

static inline
void encode_u64_be(uint8_t *dst, uint64_t v)
{
	dst[0] = (uint8_t) (v >> 56);
	dst[1] = (uint8_t) (v >> 48);
	dst[2] = (uint8_t) (v >> 40);
	dst[3] = (uint8_t) (v >> 32);
	dst[4] = (uint8_t) (v >> 24);
	dst[5] = (uint8_t) (v >> 16);
	dst[6] = (uint8_t) (v >> 8);
	dst[7] = (uint8_t) v;
}

static inline
uint64_t decode_u64_be(const uint8_t *src)
{
	return ((uint64_t) src[0] << 56) | ((uint64_t) src[1] << 48)
		| ((uint64_t) src[2] << 40) | ((uint64_t) src[3] << 32)
		| ((uint64_t) src[4] << 24) | ((uint64_t) src[5] << 16)
		| ((uint64_t) src[6] << 8)  | (uint64_t) src[7];
}

enum cds_ft_status cds_ft_range_insert(struct cds_ft_range *ftr,
		uint64_t start, uint64_t end,
		struct cds_ft_range_node *node)
{
	struct cds_ft *trie;
	enum cds_ft_status s;
	unsigned int k;
	uint8_t key[8];

	if (end <= start)
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;

	k = cds_ft_range_route_level(start, end);
	if (k >= CDS_FT_RANGE_NR_LEVELS)
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;

	s = ensure_level(ftr, k, &trie);
	if (s < 0)
		return s;

	encode_u64_be(key, start);
	node->level = k;
	node->start = start;
	node->end = end;
	return cds_ft_insert(trie, key, 8, &node->ft_node);
}

enum cds_ft_status cds_ft_range_remove(struct cds_ft_range *ftr,
		uint64_t start,
		struct cds_ft_range_node *node)
{
	struct cds_ft *trie;
	struct cds_ft_iter *iter = NULL;
	enum cds_ft_status s;
	unsigned int k;
	uint8_t key[8];

	if (!node)
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	k = node->level;
	if (k >= CDS_FT_RANGE_NR_LEVELS)
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	trie = rcu_dereference(ftr->level[k]);
	if (!trie)
		return CDS_FT_STATUS_NOT_FOUND;

	encode_u64_be(key, start);

	s = cds_ft_iter_create(trie, &iter);
	if (s < 0)
		return s;
	s = cds_ft_iter_set_key(iter, key, 8);
	if (s < 0)
		goto out;
	/*
	 * Position the iterator at the key. cds_ft_remove will then
	 * unlink @node from the duplicate chain at that key. The iter
	 * path is fresh and the writer mutex is held by the caller, so
	 * no rcu_read_lock dance is needed for the cached path.
	 */
	s = cds_ft_lookup(trie, iter);
	if (s != CDS_FT_STATUS_OK)
		goto out;
	s = cds_ft_remove(trie, iter, &node->ft_node);
out:
	cds_ft_iter_destroy(iter);
	return s;
}

/* ------------------------------------------------------------------ */
/* Iterator                                                           */
/* ------------------------------------------------------------------ */

enum cds_ft_status cds_ft_range_iter_create(struct cds_ft_range *ftr,
		struct cds_ft_range_iter **result_iter)
{
	struct cds_ft_range_iter *iter;

	iter = (struct cds_ft_range_iter *) calloc(1, sizeof(*iter));
	if (!iter) {
		*result_iter = NULL;
		return CDS_FT_STATUS_MEMORY_ERROR;
	}
	iter->ftr = ftr;
	*result_iter = iter;
	return CDS_FT_STATUS_OK;
}

void cds_ft_range_iter_destroy(struct cds_ft_range_iter *iter)
{
	if (!iter)
		return;
	if (iter->ft_iter)
		cds_ft_iter_destroy(iter->ft_iter);
	free(iter);
}

struct cds_ft_range_node *cds_ft_range_iter_node(
		const struct cds_ft_range_iter *iter)
{
	if (!iter)
		return NULL;
	return iter->current;
}

/*
 * Compute scan_lo, scan_hi for the iterator's current level based on
 * its scan strategy.
 */
static inline
void compute_scan_window(struct cds_ft_range_iter *iter)
{
	if (iter->scan_strategy == CDS_FT_RANGE_SCAN_HALO) {
		uint64_t halo = level_max_len(iter->level);
		iter->scan_lo = (iter->halo_q_a > halo)
			? iter->halo_q_a - halo : 0;
		iter->scan_hi = iter->halo_q_b;
	} else {
		iter->scan_lo = iter->scan_start_lo;
		iter->scan_hi = iter->scan_start_hi;
	}
}

/*
 * Filter predicate applied per range during the chain walk.
 * Returns true when the range satisfies all of the iterator's
 * end / length bounds and should be yielded to the caller.
 */
static inline
bool range_passes_filter(struct cds_ft_range_iter *iter,
		const struct cds_ft_range_node *rn)
{
	uint64_t L;

	if (rn->end <= iter->end_floor)
		return false;
	if (rn->end > iter->end_ceil)
		return false;
	L = rn->end - rn->start;
	if (L < iter->length_lo)
		return false;
	if (L >= iter->length_hi)
		return false;
	return true;
}

/*
 * Discard the current per-level cds_ft_iter (if any) and switch the
 * range iter to a fresh state for level @new_level. The caller will
 * then attempt to position into level[@new_level].
 */
static
void release_level_iter(struct cds_ft_range_iter *iter)
{
	if (iter->ft_iter) {
		cds_ft_iter_destroy(iter->ft_iter);
		iter->ft_iter = NULL;
	}
	iter->chain_pos = NULL;
}

/*
 * Position @iter at the first trie key >= scan_lo at the current
 * level. Returns CDS_FT_STATUS_OK with iter->ft_iter / chain_pos set
 * to a valid head when there is a candidate within [scan_lo, scan_hi);
 * returns CDS_FT_STATUS_NOT_FOUND when no candidate exists at this
 * level (caller should advance to the next level).
 */
static
enum cds_ft_status seek_level(struct cds_ft_range_iter *iter)
{
	struct cds_ft_range *ftr = iter->ftr;
	struct cds_ft *trie;
	enum cds_ft_status s;
	uint8_t key[8];

	trie = rcu_dereference(ftr->level[iter->level]);
	if (!trie)
		return CDS_FT_STATUS_NOT_FOUND;

	if (!iter->ft_iter) {
		s = cds_ft_iter_create(trie, &iter->ft_iter);
		if (s < 0)
			return s;
	}
	encode_u64_be(key, iter->scan_lo);
	s = cds_ft_iter_set_key(iter->ft_iter, key, 8);
	if (s < 0)
		return s;
	s = cds_ft_lookup_ge(trie, iter->ft_iter);
	if (s != CDS_FT_STATUS_OK)
		return CDS_FT_STATUS_NOT_FOUND;

	iter->chain_pos = cds_ft_iter_node(iter->ft_iter);
	return CDS_FT_STATUS_OK;
}

/*
 * Read the iterator's current key as a uint64_t. Caller must have
 * the iter positioned at a valid key (cds_ft_iter_node != NULL).
 */
static
uint64_t iter_key_u64(struct cds_ft_iter *ft_iter)
{
	uint8_t key[8];
	size_t len = 0;

	(void) cds_ft_iter_get_key(ft_iter, key, sizeof(key), &len);
	if (len != 8)
		return 0;
	return decode_u64_be(key);
}

/*
 * Advance to the next match, starting from the iter's current state.
 * Sets iter->current to the matching node, or NULL when exhausted.
 * Returns CDS_FT_STATUS_OK on match, CDS_FT_STATUS_NOT_FOUND when
 * exhausted.
 */
static
enum cds_ft_status advance_to_match(struct cds_ft_range_iter *iter)
{
	struct cds_ft_range *ftr = iter->ftr;

	for (;;) {
		/* Phase A: walk the current duplicate chain. */
		while (iter->chain_pos) {
			struct cds_ft_range_node *rn = cds_ft_range_entry(
					iter->chain_pos,
					struct cds_ft_range_node, ft_node);

			if (range_passes_filter(iter, rn)) {
				iter->current = rn;
				return CDS_FT_STATUS_OK;
			}
			iter->chain_pos = rcu_dereference(iter->chain_pos->next);
		}

		/*
		 * Phase B: chain at current trie key exhausted. Try
		 * advancing the trie iter to the next key at this level,
		 * if we already have one positioned.
		 */
		if (iter->ft_iter) {
			struct cds_ft *trie;
			enum cds_ft_status s;

			trie = rcu_dereference(ftr->level[iter->level]);
			s = cds_ft_next(trie, iter->ft_iter);
			if (s == CDS_FT_STATUS_OK
					&& iter_key_u64(iter->ft_iter) < iter->scan_hi) {
				iter->chain_pos = cds_ft_iter_node(iter->ft_iter);
				continue;
			}
			/* Trie at this level exhausted or past scan_hi. */
			release_level_iter(iter);
		}

		/* Phase C: advance to next non-empty level (within k_min..k_max). */
		for (;;) {
			iter->level++;
			if (iter->level > iter->k_max) {
				iter->current = NULL;
				iter->exhausted = true;
				return CDS_FT_STATUS_NOT_FOUND;
			}
			if (rcu_dereference(ftr->level[iter->level]))
				break;
		}
		compute_scan_window(iter);

		/* Phase D: seek into the new level. */
		{
			enum cds_ft_status s = seek_level(iter);

			if (s == CDS_FT_STATUS_NOT_FOUND) {
				/* No key >= scan_lo at this level. */
				continue;
			}
			if (s < 0) {
				iter->current = NULL;
				return s;
			}
			/* Got a candidate trie key; check key < scan_hi. */
			if (iter_key_u64(iter->ft_iter) >= iter->scan_hi) {
				release_level_iter(iter);
				continue;
			}
			/* chain_pos set by seek_level; fall through to walk it. */
		}
	}
}

/*
 * Largest level whose ranges might satisfy length < length_hi.
 * Level k holds L in (2^(k-1), 2^k]: skip k iff 2^(k-1) >= length_hi.
 * Returns NR_LEVELS - 1 when length_hi == UINT64_MAX (no upper bound).
 */
static inline
unsigned int max_level_for_length_hi(uint64_t length_hi)
{
	if (length_hi == UINT64_MAX || length_hi > (1ULL << (CDS_FT_RANGE_NR_LEVELS - 1)))
		return CDS_FT_RANGE_NR_LEVELS - 1;
	if (length_hi <= 1)
		return 0;
	/* Smallest k with 2^k >= length_hi. */
	return 64u - (unsigned int) __builtin_clzll(length_hi - 1);
}

/*
 * Set iter->level so that the next advance_to_match() Phase C land
 * on @k_min. Phase C does iter->level++ before checking, so we set
 * level = k_min - 1 (with unsigned underflow when k_min == 0).
 */
static inline
void prime_level(struct cds_ft_range_iter *iter, unsigned int k_min)
{
	iter->level = (k_min == 0) ? UINT_MAX : (k_min - 1);
}

/*
 * Common iterator-bound setup shared by all entry points.  Resets
 * scan state, clears current/exhausted, primes the level walker.
 * Returns CDS_FT_STATUS_OK if the configured level range is
 * non-empty, CDS_FT_STATUS_NOT_FOUND otherwise (caller short-
 * circuits without scanning).
 */
static
enum cds_ft_status iter_begin(struct cds_ft_range_iter *iter)
{
	release_level_iter(iter);
	iter->current = NULL;
	iter->exhausted = false;
	if (iter->k_min > iter->k_max) {
		iter->exhausted = true;
		return CDS_FT_STATUS_NOT_FOUND;
	}
	prime_level(iter, iter->k_min);
	return CDS_FT_STATUS_OK;
}

enum cds_ft_status cds_ft_range_lookup_overlap(
		struct cds_ft_range_iter *iter,
		uint64_t q_a, uint64_t q_b,
		uint64_t granularity)
{
	enum cds_ft_status s;

	if (!iter)
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	if (q_b <= q_a) {
		iter->current = NULL;
		iter->exhausted = true;
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}

	iter->scan_strategy = CDS_FT_RANGE_SCAN_HALO;
	iter->halo_q_a = q_a;
	iter->halo_q_b = q_b;
	iter->end_floor = q_a;
	iter->end_ceil = UINT64_MAX;
	iter->length_lo = granularity;
	iter->length_hi = UINT64_MAX;
	iter->k_min = min_level_for_granularity(granularity);
	iter->k_max = CDS_FT_RANGE_NR_LEVELS - 1;
	s = iter_begin(iter);
	if (s != CDS_FT_STATUS_OK)
		return s;
	return advance_to_match(iter);
}

enum cds_ft_status cds_ft_range_lookup_stab(
		struct cds_ft_range_iter *iter,
		uint64_t x, uint64_t granularity)
{
	if (!iter)
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	if (x == UINT64_MAX) {
		/* No range can have end > UINT64_MAX. */
		iter->current = NULL;
		iter->exhausted = true;
		return CDS_FT_STATUS_NOT_FOUND;
	}
	return cds_ft_range_lookup_overlap(iter, x, x + 1, granularity);
}

enum cds_ft_status cds_ft_range_lookup_overlap_band(
		struct cds_ft_range_iter *iter,
		uint64_t q_a, uint64_t q_b,
		uint64_t length_lo, uint64_t length_hi)
{
	enum cds_ft_status s;

	if (!iter)
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	if (q_b <= q_a || length_hi <= length_lo) {
		iter->current = NULL;
		iter->exhausted = true;
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}

	iter->scan_strategy = CDS_FT_RANGE_SCAN_HALO;
	iter->halo_q_a = q_a;
	iter->halo_q_b = q_b;
	iter->end_floor = q_a;
	iter->end_ceil = UINT64_MAX;
	iter->length_lo = length_lo;
	iter->length_hi = length_hi;
	iter->k_min = min_level_for_granularity(length_lo);
	iter->k_max = max_level_for_length_hi(length_hi);
	s = iter_begin(iter);
	if (s != CDS_FT_STATUS_OK)
		return s;
	return advance_to_match(iter);
}

/*
 * Pan-classifier helpers.  Both viewports are assumed valid
 * (q_b > q_a).
 */
static inline
bool viewports_disjoint(uint64_t old_a, uint64_t old_b,
		uint64_t new_a, uint64_t new_b)
{
	return new_a >= old_b || new_b <= old_a;
}

static inline
bool is_pure_pan_right(uint64_t old_a, uint64_t old_b,
		uint64_t new_a, uint64_t new_b)
{
	return new_a > old_a && new_b > old_b;
}

static inline
bool is_pure_pan_left(uint64_t old_a, uint64_t old_b,
		uint64_t new_a, uint64_t new_b)
{
	return new_a < old_a && new_b < old_b;
}

static inline
bool viewport_contains(uint64_t outer_a, uint64_t outer_b,
		uint64_t inner_a, uint64_t inner_b)
{
	return outer_a <= inner_a && outer_b >= inner_b;
}

enum cds_ft_status cds_ft_range_lookup_entering(
		struct cds_ft_range_iter *iter,
		uint64_t old_q_a, uint64_t old_q_b,
		uint64_t new_q_a, uint64_t new_q_b,
		uint64_t granularity)
{
	enum cds_ft_status s;

	if (!iter)
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	if (old_q_b <= old_q_a || new_q_b <= new_q_a) {
		iter->current = NULL;
		iter->exhausted = true;
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}
	/*
	 * NEW subset of OLD (including identity): nothing entering.
	 * "subset" here means new_q_a >= old_q_a AND new_q_b <= old_q_b.
	 */
	if (viewport_contains(old_q_a, old_q_b, new_q_a, new_q_b)) {
		iter->current = NULL;
		iter->exhausted = true;
		return CDS_FT_STATUS_NOT_FOUND;
	}
	/* No overlap: entering = full new viewport. */
	if (viewports_disjoint(old_q_a, old_q_b, new_q_a, new_q_b))
		return cds_ft_range_lookup_overlap(iter,
			new_q_a, new_q_b, granularity);

	if (is_pure_pan_right(old_q_a, old_q_b, new_q_a, new_q_b)) {
		/*
		 * Entering: start in [old_q_b, new_q_b),
		 * end > new_q_a, length >= g.
		 */
		iter->scan_strategy = CDS_FT_RANGE_SCAN_FIXED;
		iter->scan_start_lo = old_q_b;
		iter->scan_start_hi = new_q_b;
		iter->end_floor = new_q_a;
		iter->end_ceil = UINT64_MAX;
		iter->length_lo = granularity;
		iter->length_hi = UINT64_MAX;
		iter->k_min = min_level_for_granularity(granularity);
		iter->k_max = CDS_FT_RANGE_NR_LEVELS - 1;
		s = iter_begin(iter);
		if (s != CDS_FT_STATUS_OK)
			return s;
		return advance_to_match(iter);
	}
	if (is_pure_pan_left(old_q_a, old_q_b, new_q_a, new_q_b)) {
		/*
		 * Entering: end in (new_q_a, old_q_a], start < new_q_b,
		 * length >= g.  Pan-left with overlap implies new_q_b >
		 * old_q_a, so any range with start < old_q_a satisfies
		 * start < new_q_b automatically.  We use a halo-style
		 * scan over the leading strip [new_q_a, old_q_a).
		 */
		iter->scan_strategy = CDS_FT_RANGE_SCAN_HALO;
		iter->halo_q_a = new_q_a;
		iter->halo_q_b = old_q_a;
		iter->end_floor = new_q_a;
		iter->end_ceil = old_q_a;
		iter->length_lo = granularity;
		iter->length_hi = UINT64_MAX;
		iter->k_min = min_level_for_granularity(granularity);
		iter->k_max = CDS_FT_RANGE_NR_LEVELS - 1;
		s = iter_begin(iter);
		if (s != CDS_FT_STATUS_OK)
			return s;
		return advance_to_match(iter);
	}
	/*
	 * Mixed transition (e.g., widening or shifted-and-resized):
	 * fall back to a full re-query of NEW.  Caller can compose
	 * two delta calls explicitly if it needs the cheap path.
	 */
	return cds_ft_range_lookup_overlap(iter,
		new_q_a, new_q_b, granularity);
}

enum cds_ft_status cds_ft_range_lookup_leaving(
		struct cds_ft_range_iter *iter,
		uint64_t old_q_a, uint64_t old_q_b,
		uint64_t new_q_a, uint64_t new_q_b,
		uint64_t granularity)
{
	enum cds_ft_status s;

	if (!iter)
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	if (old_q_b <= old_q_a || new_q_b <= new_q_a) {
		iter->current = NULL;
		iter->exhausted = true;
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}
	/* OLD subset of NEW (including identity): nothing leaving. */
	if (viewport_contains(new_q_a, new_q_b, old_q_a, old_q_b)) {
		iter->current = NULL;
		iter->exhausted = true;
		return CDS_FT_STATUS_NOT_FOUND;
	}
	/* No overlap: leaving = full old viewport. */
	if (viewports_disjoint(old_q_a, old_q_b, new_q_a, new_q_b))
		return cds_ft_range_lookup_overlap(iter,
			old_q_a, old_q_b, granularity);

	if (is_pure_pan_right(old_q_a, old_q_b, new_q_a, new_q_b)) {
		/*
		 * Leaving: end in (old_q_a, new_q_a], start < old_q_b,
		 * length >= g.  Pan-right with overlap implies new_q_a <
		 * old_q_b, so any range whose end <= new_q_a and was in
		 * OLD has start < old_q_b automatically (start <= end-1
		 * < new_q_a < old_q_b).  Halo-scan the leading strip
		 * [old_q_a, new_q_a) and bound end <= new_q_a.
		 */
		iter->scan_strategy = CDS_FT_RANGE_SCAN_HALO;
		iter->halo_q_a = old_q_a;
		iter->halo_q_b = new_q_a;
		iter->end_floor = old_q_a;
		iter->end_ceil = new_q_a;
		iter->length_lo = granularity;
		iter->length_hi = UINT64_MAX;
		iter->k_min = min_level_for_granularity(granularity);
		iter->k_max = CDS_FT_RANGE_NR_LEVELS - 1;
		s = iter_begin(iter);
		if (s != CDS_FT_STATUS_OK)
			return s;
		return advance_to_match(iter);
	}
	if (is_pure_pan_left(old_q_a, old_q_b, new_q_a, new_q_b)) {
		/*
		 * Leaving: start in [new_q_b, old_q_b), end > old_q_a,
		 * length >= g.
		 */
		iter->scan_strategy = CDS_FT_RANGE_SCAN_FIXED;
		iter->scan_start_lo = new_q_b;
		iter->scan_start_hi = old_q_b;
		iter->end_floor = old_q_a;
		iter->end_ceil = UINT64_MAX;
		iter->length_lo = granularity;
		iter->length_hi = UINT64_MAX;
		iter->k_min = min_level_for_granularity(granularity);
		iter->k_max = CDS_FT_RANGE_NR_LEVELS - 1;
		s = iter_begin(iter);
		if (s != CDS_FT_STATUS_OK)
			return s;
		return advance_to_match(iter);
	}
	/*
	 * Mixed transition (e.g., shrinking): fall back to a full
	 * re-query of OLD.  This may include ranges that ARE in NEW;
	 * callers that need a precise leaving-only set should filter
	 * the result against NEW client-side or compose two delta
	 * calls.
	 */
	return cds_ft_range_lookup_overlap(iter,
		old_q_a, old_q_b, granularity);
}

enum cds_ft_status cds_ft_range_iter_next(struct cds_ft_range_iter *iter)
{
	if (!iter)
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	if (iter->exhausted) {
		iter->current = NULL;
		return CDS_FT_STATUS_NOT_FOUND;
	}
	/* Step past the current match. */
	if (iter->chain_pos) {
		iter->chain_pos = rcu_dereference(iter->chain_pos->next);
	}
	iter->current = NULL;
	return advance_to_match(iter);
}
