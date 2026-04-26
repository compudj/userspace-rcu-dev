// SPDX-FileCopyrightText: 2026 EfficiOS Inc.
//
// SPDX-License-Identifier: LGPL-2.1-only

#ifndef _URCU_FRACTAL_TRIE_RANGE_H
#define _URCU_FRACTAL_TRIE_RANGE_H

/*
 * urcu/fractal-trie-range.h
 *
 * Userspace RCU library - Range index layered on top of Fractal Trie.
 *
 * Indexes half-open ranges [start, end) over uint64_t timelines and
 * answers range-overlap queries with two structural properties suited
 * to interactive timeline / trace viewers:
 *
 *   - Granularity culling: a query supplies a "granularity" floor and
 *     the index skips any range with length below it (sub-pixel state
 *     ranges are not visited at all).
 *
 *   - Pan locality: when the viewport shifts by Delta, the per-level
 *     scan window shifts by Delta too. The bulk of work at coarse
 *     levels is reusable; only fine levels need to re-scan more of
 *     their key window.
 *
 * Internally, ranges are partitioned into up to 64 per-level
 * cds_ft instances by length class:
 *
 *     k = ceil(log2(L))    where L = end - start
 *
 * Level k holds ranges with L in (2^(k-1), 2^k]. Each per-level trie
 * is keyed on the range's start position (big-endian uint64_t). One
 * trie entry per inserted range. Multiple ranges sharing the same
 * start ride the trie's native duplicate chain.
 *
 * Scope (v1): 8-byte unsigned integer keys only. Generalisation to
 * other widths, signed keys, and variable-length keys is future
 * work; the design does not preclude it.
 *
 * Concurrency: the range index inherits cds_ft's reader/writer
 * contract. Readers must hold the RCU read-side lock for the entire
 * iteration sequence (the iterator caches per-level trie state).
 * Writers (insert / remove) must serialise themselves the same way
 * as cds_ft writers do (typically a single writer mutex).
 *
 * Include this header _after_ the URCU flavor header, just like
 * urcu/fractal-trie.h.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <urcu/fractal-trie.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Number of per-length-class levels. One trie per bit position of L. */
#define CDS_FT_RANGE_NR_LEVELS	64

/* Opaque types. */
struct cds_ft_range;
struct cds_ft_range_iter;
struct cds_ft_range_attr;

/*
 * Intrusive node embedded in user data.
 *
 * The user provides the storage (typically by embedding this struct
 * in their own range record) and is responsible for its lifetime
 * relative to RCU grace periods, just like with cds_ft_node.
 *
 * Fields:
 *   ft_node - the trie node used by the underlying cds_ft. Must be
 *             initialised via cds_ft_node_init(&n->ft_node) (or
 *             zeroed) before insertion, the same way cds_ft_node is
 *             initialised. cds_ft_range_insert() does NOT call
 *             cds_ft_node_init for the user; doing so here would
 *             clobber re-inserted nodes that are on the dup chain of
 *             a parallel chain. Match cds_ft conventions.
 *   end     - end position of the half-open range [start, end).
 *             Stored on the node so the query path can filter
 *             overlap (T+L > q_a) without indirecting into user
 *             data.
 *   level   - routed level. Set by cds_ft_range_insert(); read by
 *             cds_ft_range_remove() to find the right per-level
 *             trie. Do not modify between insert and remove.
 *
 * To recover the user record from a cds_ft_range_node *, use
 * cds_ft_range_entry() (or directly caa_container_of).
 */
struct cds_ft_range_node {
	struct cds_ft_node ft_node;
	uint64_t start;
	uint64_t end;
	unsigned int level;
};

#define cds_ft_range_entry(ptr, type, member) \
	caa_container_of(ptr, type, member)

/*
 * cds_ft_range_node_init - Initialise a range node.
 * @node: The node.
 *
 * Wraps cds_ft_node_init() and zeros end/level. Equivalent to
 * memset(node, 0, sizeof(*node)).
 */
static inline
void cds_ft_range_node_init(struct cds_ft_range_node *node)
{
	cds_ft_node_init(&node->ft_node);
	node->start = 0;
	node->end = 0;
	node->level = 0;
}

/*
 * Range-attr API (currently empty; reserved for future flags such as
 * exclusive mode). Use NULL where attr is accepted to take defaults.
 */

enum cds_ft_status cds_ft_range_attr_create(struct cds_ft_range_attr **result);
void cds_ft_range_attr_destroy(struct cds_ft_range_attr *attr);

/*
 * cds_ft_range_create - Create a range index.
 * @group: A cds_ft_group (must be configured with key_len = 8).
 *         The group is owned by the caller; the range index simply
 *         creates per-level cds_ft instances inside it.
 * @attr: Attributes (may be NULL for defaults).
 * @result: Output range index handle.
 *
 * Per-level cds_ft instances are NOT created up front; they are
 * allocated lazily on first insert at each level.
 *
 * The group must outlive the range index. Multiple range indices
 * can share the same group; they will not interfere because each
 * uses its own per-level cds_ft instances.
 *
 * Returns CDS_FT_STATUS_OK on success, or a negative cds_ft_status
 * on error.
 */
enum cds_ft_status cds_ft_range_create(
		struct cds_ft_group *group,
		const struct cds_ft_range_attr *attr,
		struct cds_ft_range **result);

/*
 * cds_ft_range_destroy - Destroy a range index.
 * @ftr: The range index.
 *
 * Destroys all per-level cds_ft instances. The caller's group is
 * NOT destroyed (caller owns it). The caller must ensure no
 * concurrent readers or writers can access @ftr after this call
 * returns. All inserted nodes must have been removed and
 * reclaimed beforehand (the index does not own user nodes and
 * does not call_rcu them on destruction).
 */
void cds_ft_range_destroy(struct cds_ft_range *ftr);

/*
 * cds_ft_range_insert - Insert a half-open range [start, end).
 * @ftr: The range index.
 * @start: Range start (inclusive).
 * @end: Range end (exclusive). Must satisfy end > start.
 * @node: User-provided node. Must be initialised via
 *        cds_ft_range_node_init() (or zeroed) before this call.
 *
 * Routes the range to its length-class level (ceil(log2(end-start)))
 * and inserts at that level under key @start. Sets node->level and
 * node->end. Multiple ranges sharing the same @start ride the
 * trie's native duplicate chain.
 *
 * Returns CDS_FT_STATUS_OK on success, or a negative cds_ft_status
 * on error (CDS_FT_STATUS_INVALID_ARGUMENT_ERROR if end <= start;
 * CDS_FT_STATUS_MEMORY_ERROR on allocation failure of the per-level
 * trie).
 *
 * Mutual exclusion between writers (insert / remove) is the
 * caller's responsibility, like cds_ft.
 */
enum cds_ft_status cds_ft_range_insert(
		struct cds_ft_range *ftr,
		uint64_t start, uint64_t end,
		struct cds_ft_range_node *node);

/*
 * cds_ft_range_remove - Remove a previously inserted range.
 * @ftr: The range index.
 * @start: Range start, same as the value passed to insert.
 * @node: The node to remove (matched by identity within the
 *        duplicate chain at that key).
 *
 * Reads node->level to locate the per-level trie. The caller is
 * responsible for waiting an RCU grace period before reusing or
 * freeing @node, just like cds_ft_remove.
 *
 * Returns CDS_FT_STATUS_OK on success, CDS_FT_STATUS_NOT_FOUND if
 * the node is not present, or a negative cds_ft_status on error.
 */
enum cds_ft_status cds_ft_range_remove(
		struct cds_ft_range *ftr,
		uint64_t start,
		struct cds_ft_range_node *node);

/*
 * cds_ft_range_iter_create - Create an iterator for an overlap query.
 * @ftr: The range index.
 * @result_iter: Output iterator handle.
 *
 * The returned iterator is bound to @ftr until destroyed. It
 * does not start a query yet; call cds_ft_range_lookup_overlap()
 * to position it.
 *
 * Returns CDS_FT_STATUS_OK on success.
 */
enum cds_ft_status cds_ft_range_iter_create(
		struct cds_ft_range *ftr,
		struct cds_ft_range_iter **result_iter);

/*
 * cds_ft_range_iter_destroy - Destroy an iterator.
 */
void cds_ft_range_iter_destroy(struct cds_ft_range_iter *iter);

/*
 * cds_ft_range_lookup_overlap - Position the iterator at the first
 *                               range overlapping [q_a, q_b) with
 *                               length >= granularity.
 * @iter: The iterator (must be already created with
 *        cds_ft_range_iter_create).
 * @q_a: Query window start (inclusive).
 * @q_b: Query window end (exclusive). Must satisfy q_b > q_a.
 * @granularity: Minimum range length to return. Pass 0 to disable
 *               culling.
 *
 * After this call, cds_ft_range_iter_node() returns either the
 * first matching range's node (if any), or NULL when none.
 * Subsequent matches are reached with cds_ft_range_iter_next().
 *
 * The RCU read-side lock must be held by the caller throughout
 * the iteration sequence (lookup_overlap + next + ... + access of
 * returned nodes), the same way cds_ft requires for ordered
 * iteration with a path-cached iterator.
 *
 * Returns CDS_FT_STATUS_OK on success (iterator is positioned
 * at first match or exhausted), or a negative cds_ft_status on
 * error.
 */
enum cds_ft_status cds_ft_range_lookup_overlap(
		struct cds_ft_range_iter *iter,
		uint64_t q_a, uint64_t q_b,
		uint64_t granularity);

/*
 * cds_ft_range_lookup_stab - Position the iterator at the first
 *     range covering point @x with length >= @granularity.
 *     Predicate: start <= x AND end > x AND L >= g.
 * @iter: The iterator.
 * @x: Stabbing point.
 * @granularity: Minimum range length to return. Pass 0 to disable
 *     culling.
 *
 * Convenience for hit-test / cursor-at-point queries; equivalent to
 * cds_ft_range_lookup_overlap(iter, x, x + 1, granularity), with a
 * NOT_FOUND short-circuit when x = UINT64_MAX (no range can have
 * end > UINT64_MAX).
 *
 * Same RCU locking discipline as cds_ft_range_lookup_overlap().
 */
enum cds_ft_status cds_ft_range_lookup_stab(
		struct cds_ft_range_iter *iter,
		uint64_t x, uint64_t granularity);

/*
 * cds_ft_range_lookup_overlap_band - Position the iterator at the
 *     first range overlapping [q_a, q_b) with length in
 *     [length_lo, length_hi).
 * @iter: The iterator.
 * @q_a, @q_b: Query window (half-open).
 * @length_lo, @length_hi: Half-open length band. Pass 0 for
 *     length_lo to disable the lower bound; pass UINT64_MAX for
 *     length_hi to disable the upper bound.
 *
 * Useful for zoom-in transitions (g_new < g_old): query the band
 * [g_new, g_old) over the current viewport to fetch the
 * newly-visible shorter ranges. Cost is naturally proportional to
 * how many length-class levels overlap the band, not to the total
 * population.
 *
 * Same RCU locking discipline as cds_ft_range_lookup_overlap().
 */
enum cds_ft_status cds_ft_range_lookup_overlap_band(
		struct cds_ft_range_iter *iter,
		uint64_t q_a, uint64_t q_b,
		uint64_t length_lo, uint64_t length_hi);

/*
 * cds_ft_range_lookup_contained_in - Position the iterator at the
 *     first range fully inside [q_a, q_b) with length >= granularity.
 *     Predicate: start >= q_a AND end <= q_b AND L >= g.
 * @iter: The iterator.
 * @q_a, @q_b: Window bounds (half-open). q_b > q_a.
 * @granularity: Minimum range length to return. Pass 0 to disable
 *     culling.
 *
 * Useful for "list all ranges entirely within this selection" /
 * sub-trace export / layout passes that need self-contained ranges.
 * Cost is bounded by the per-level scan over [q_a, q_b) and skips
 * length-class levels whose minimum length exceeds the window
 * width.
 *
 * Same RCU locking discipline as cds_ft_range_lookup_overlap().
 */
enum cds_ft_status cds_ft_range_lookup_contained_in(
		struct cds_ft_range_iter *iter,
		uint64_t q_a, uint64_t q_b,
		uint64_t granularity);

/*
 * cds_ft_range_lookup_entering - Position the iterator at the first
 *     range entering the viewport on a transition from
 *     [old_q_a, old_q_b) to [new_q_a, new_q_b) at granularity g.
 *     "Entering" = ranges satisfying the new predicate but not the
 *     old.
 *
 * Pure pan-right (new_q_a > old_q_a, new_q_b > old_q_b, with
 * overlap): scans only ranges starting in [old_q_b, new_q_b),
 * cost proportional to the pan distance Δ = new_q_b - old_q_b.
 *
 * Pure pan-left (new_q_a < old_q_a, new_q_b < old_q_b, with
 * overlap): scans the leading strip [new_q_a, old_q_a), cost
 * proportional to old_q_a - new_q_a.
 *
 * No overlap: equivalent to cds_ft_range_lookup_overlap() over the
 * new viewport.
 *
 * NEW subset of OLD (including identity): empty result.
 *
 * Other mixed cases (e.g., widening): falls back to a full re-query
 * of the new viewport via cds_ft_range_lookup_overlap(). The result
 * is correct (a superset is impossible because the old ranges in
 * NEW \ OLD are all in NEW), but does not exploit the delta-only
 * fast path. Callers needing optimal widening can issue two
 * sequential entering queries: one for the right strip
 * [old_q_b, new_q_b) and one for the left strip
 * [new_q_a, old_q_a).
 *
 * Same RCU locking discipline as cds_ft_range_lookup_overlap().
 */
enum cds_ft_status cds_ft_range_lookup_entering(
		struct cds_ft_range_iter *iter,
		uint64_t old_q_a, uint64_t old_q_b,
		uint64_t new_q_a, uint64_t new_q_b,
		uint64_t granularity);

/*
 * cds_ft_range_lookup_leaving - Position the iterator at the first
 *     range leaving the viewport on a transition from
 *     [old_q_a, old_q_b) to [new_q_a, new_q_b) at granularity g.
 *     "Leaving" = ranges satisfying the old predicate but not the
 *     new.
 *
 * Symmetric to cds_ft_range_lookup_entering(): pure pan-right scans
 * the trailing strip via end <= new_q_a; pure pan-left scans
 * starts in [new_q_b, old_q_b); no-overlap returns the full old
 * viewport; OLD subset of NEW returns empty; other mixed cases
 * fall back to a full re-query of the old viewport.
 *
 * Same RCU locking discipline as cds_ft_range_lookup_overlap().
 */
enum cds_ft_status cds_ft_range_lookup_leaving(
		struct cds_ft_range_iter *iter,
		uint64_t old_q_a, uint64_t old_q_b,
		uint64_t new_q_a, uint64_t new_q_b,
		uint64_t granularity);

/*
 * cds_ft_range_iter_next - Advance the iterator to the next match.
 * @iter: Iterator already positioned by cds_ft_range_lookup_overlap().
 *
 * On return, cds_ft_range_iter_node() either returns the next
 * matching range's node, or NULL when iteration is exhausted.
 *
 * Returns CDS_FT_STATUS_OK on success, CDS_FT_STATUS_NOT_FOUND when
 * exhausted, or a negative cds_ft_status on error.
 */
enum cds_ft_status cds_ft_range_iter_next(struct cds_ft_range_iter *iter);

/*
 * cds_ft_range_iter_node - Return the current matching range node.
 * @iter: The iterator.
 *
 * Returns the cds_ft_range_node * for the current match, or NULL
 * when iteration is exhausted or before the first lookup_overlap()
 * call.
 */
struct cds_ft_range_node *cds_ft_range_iter_node(
		const struct cds_ft_range_iter *iter);

/*
 * cds_ft_range_for_each_overlap_rcu - Iterate over all matches.
 * @iter: Iterator (struct cds_ft_range_iter *), used as loop cursor.
 * @q_a, @q_b, @granularity: Query window and granularity (see
 *                           cds_ft_range_lookup_overlap()).
 *
 * The RCU read-side lock must be held continuously by the caller
 * during the entire loop, including access to nodes returned by
 * cds_ft_range_iter_node().
 */
#define cds_ft_range_for_each_overlap_rcu(iter, q_a, q_b, granularity)	\
	for (cds_ft_range_lookup_overlap((iter), (q_a), (q_b),		\
					(granularity));			\
			cds_ft_range_iter_node(iter);			\
			cds_ft_range_iter_next(iter))

/*
 * cds_ft_range_route_level - Compute the level a range would route to.
 * @start, @end: The half-open range.
 *
 * Pure function of L = end - start. Returns 0 for L = 1, otherwise
 * 64 - __builtin_clzll(L - 1) (i.e. ceil(log2(L))). Returns
 * CDS_FT_RANGE_NR_LEVELS if @end <= @start (caller error).
 *
 * Useful for callers that want to pre-classify ranges (e.g. when
 * loading from a file format that records level-class statistics).
 */
unsigned int cds_ft_range_route_level(uint64_t start, uint64_t end);

#ifdef __cplusplus
}
#endif

#endif /* _URCU_FRACTAL_TRIE_RANGE_H */
