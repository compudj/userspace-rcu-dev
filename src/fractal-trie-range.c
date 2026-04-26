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
 * The iterator walks per-level cds_ft tries in increasing-k order,
 * starting at the granularity-derived k_min. Within each level it
 * scans trie keys in [scan_lo, scan_hi), where scan_hi is q_b
 * (exclusive) and scan_lo accounts for the maximum range length at
 * that level (a range whose start is in [q_a - 2^k, q_a) may still
 * extend into the query window).
 *
 * Within each trie key position, we walk the duplicate chain (ranges
 * sharing the same start) and filter by end > q_a.
 */
struct cds_ft_range_iter {
	struct cds_ft_range *ftr;
	uint64_t q_a, q_b;
	uint64_t granularity;
	unsigned int level;
	uint64_t scan_lo, scan_hi;	/* current level's window */
	struct cds_ft_iter *ft_iter;	/* iter into ftr->level[level], or NULL */
	struct cds_ft_node *chain_pos;	/* current dup-chain node, NULL when between trie keys */
	struct cds_ft_range_node *current;	/* match yielded to the caller, or NULL */
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

static inline
uint64_t scan_lo_for(uint64_t q_a, unsigned int k)
{
	uint64_t halo = level_max_len(k);

	if (q_a > halo)
		return q_a - halo;
	return 0;
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

			/* Overlap: rn->end > q_a (rn->start < q_b is enforced
			 * by the per-level scan window). Granularity:
			 * (rn->end - rn->start) >= granularity. */
			if (rn->end > iter->q_a
					&& (rn->end - rn->start) >= iter->granularity) {
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

		/* Phase C: advance to next non-empty level. */
		for (;;) {
			iter->level++;
			if (iter->level >= CDS_FT_RANGE_NR_LEVELS) {
				iter->current = NULL;
				iter->exhausted = true;
				return CDS_FT_STATUS_NOT_FOUND;
			}
			if (rcu_dereference(ftr->level[iter->level]))
				break;
		}
		iter->scan_lo = scan_lo_for(iter->q_a, iter->level);
		iter->scan_hi = iter->q_b;

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

enum cds_ft_status cds_ft_range_lookup_overlap(
		struct cds_ft_range_iter *iter,
		uint64_t q_a, uint64_t q_b,
		uint64_t granularity)
{
	unsigned int k_min;

	if (!iter)
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	if (q_b <= q_a) {
		iter->current = NULL;
		iter->exhausted = true;
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}

	release_level_iter(iter);
	iter->q_a = q_a;
	iter->q_b = q_b;
	iter->granularity = granularity;
	iter->current = NULL;
	iter->exhausted = false;

	k_min = min_level_for_granularity(granularity);
	if (k_min >= CDS_FT_RANGE_NR_LEVELS) {
		iter->exhausted = true;
		return CDS_FT_STATUS_NOT_FOUND;
	}

	/*
	 * Position at the level-(k_min - 1) -> k_min boundary. The
	 * advance_to_match() loop starts by walking chain_pos (NULL,
	 * so skips), then tries advancing the trie iter (NULL, so
	 * goes to next_level) which increments iter->level. Set up so
	 * the first level visited is k_min.
	 */
	iter->level = (k_min == 0) ? UINT_MAX : (k_min - 1);
	/*
	 * If k_min is 0, we want next_level to land on 0. The do/while
	 * in advance_to_match increments first, so set level to "below
	 * 0" using underflow on unsigned: UINT_MAX -> 0 after ++.
	 */

	return advance_to_match(iter);
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
