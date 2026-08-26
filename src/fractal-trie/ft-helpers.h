// SPDX-FileCopyrightText: 2012-2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-only

/*
 * src/fractal-trie/ft-helpers.h
 *
 * Userspace RCU library - Fractal Trie: general helpers: tag/node/metadata accessors, key conversion + compare, publish, flip-latch, ordinal-cell and skip-compressed primitives, allocation glue.
 *
 * Implementation unit: #included once by fractal-trie.c, in dependency
 * order, into a single translation unit (preserves cross-module inlining).
 * Not a standalone header.
 */
#ifndef FRACTAL_TRIE_IMPL
#error "ft-helpers.h is an implementation unit; #include it from fractal-trie.c only"
#endif

static inline __attribute__((unused))
void static_array_size_check(void)
{
	CAA_BUILD_BUG_ON(CAA_ARRAY_SIZE(ft_types) < FT_TYPE_MAX_NR);
#ifdef FEATURE_FT_SKIP_COMPRESSED
	/*
	 * parent_slot_offset is 8 bits and stores byte_offset / sizeof(void *).
	 * Ensure the largest node (pigeon, 2^11 = 2048 bytes) fits:
	 * 2048 / sizeof(void *) = 256 slots, max index 255.  Only enabled
	 * on 64-bit architectures, where sizeof(void *) == 8 and the
	 * quotient is exactly 256.
	 */
	CAA_BUILD_BUG_ON((1U << 11) / sizeof(void *) > 256);
#endif
	/*
	 * Metadata packed bitfield must fit in a uint32_t.
	 * Layout: nr_child(9) + [parent_slot_offset(8)] + alloc_index
	 *         (near: FT_ALLOC_INDEX_BITS + 3; far: a separate uint32_t).
	 */
	CAA_BUILD_BUG_ON(9
#ifndef FT_FAR_METADATA
		/* far-metadata stores alloc_index as its own uint32_t. */
		+ (FT_ALLOC_INDEX_BITS + 3)
#endif
#ifdef FEATURE_FT_SKIP_COMPRESSED
		+ 8
#endif
		> 32);
}

/*
 * Reader-side helpers for the cds_ft_node.next removal tombstone (bit 1,
 * see CDS_FT_NODE_REMOVED_FLAG).  These run under the writer mutex (or RCU
 * read lock on the chain-walk side), so a plain masked load is sufficient;
 * readers use cds_ft_node_next_rcu() instead.
 *
 *   ft_node_next        masked successor (the actual chain link)
 *   ft_node_is_removed  has @node been removed from the trie?
 *
 * SETTING the tombstone is a COMMITTED flip edge -- the freed node's own next
 * is a word a concurrent duplicate-append CASes, so the mark rides the
 * descriptor protocol like every other reader-visible store; see
 * ft_node_mark_removed_flip / ft_chain_mark_removed_flip in
 * ft-mutation-helpers.h (doc/design/mcas-multiwriter-readiness.md §4
 * refinement-1 site 2).
 */
/*
 * May a mutation write a LIVE node's {child pointer, occupancy bitmap,
 * nr_child} in place, instead of routing through a whole-node recompact?
 *
 * Only on an EXCLUSIVE trie.  Those three words are separate stores, so a
 * concurrent READER can sample them torn, and a concurrent WRITER can rebuild
 * the node from its occupied slots while the mutation is mid-flight -- which
 * drops a reserved (bit set, NULL child) hole from under the writer that
 * reserved it.  cds_ft_attr_set_exclusive declares BOTH away ("single-writer,
 * no concurrent readers"), which is what makes the in-place tier sound rather
 * than merely faster: doc/design/mcas-multiwriter-readiness.md §5.2, "an
 * exclusive trie keeps the in-place store".
 *
 * The build flag is the OPT-IN, this is the SAFETY CONDITION.  Without
 * FEATURE_FT_INSERT_IN_PLACE the answer is always no and every caller behaves
 * exactly as before; with it, a shared trie still recompacts and only an
 * exclusive one takes the O(1) path.  The flag alone used to decide, so an
 * opt-in build applied it to shared tries too -- where it asserts
 * (ft_attach_node's slot_ptr, 303 failures in 480 saturated runs) or, worse,
 * tears a publish quietly.
 */
static inline
bool ft_in_place_ok(const struct cds_ft *ft)
{
#ifdef FEATURE_FT_INSERT_IN_PLACE
	return ft && ft->exclusive;
#else
	(void) ft;
	return false;
#endif
}

static inline
struct cds_ft_node *ft_node_next(const struct cds_ft_node *node)
{
	return (struct cds_ft_node *) ((uintptr_t) node->next &
			~CDS_FT_NODE_REMOVED_FLAG);
}

static inline
bool ft_node_is_removed(const struct cds_ft_node *node)
{
	return ((uintptr_t) node->next & CDS_FT_NODE_REMOVED_FLAG) != 0;
}

/*
 * Iterate through duplicates returned by cds_ft_lookup*()
 * Receives a struct cds_ft_node * as parameter, which is used as start
 * of duplicate list and loop cursor.  Masks the removal tombstone.
 */
#define cds_ft_for_each_duplicate(pos)				\
       for (; (pos) != NULL; (pos) = ft_node_next(pos))

enum ft_recompact {
	FT_RECOMPACT_ADD_SAME,
	FT_RECOMPACT_ADD_NEXT,
	FT_RECOMPACT_DEL,
	/*
	 * Pure relocation: same type and child set, copied verbatim into a
	 * fresh allocation (new address), children reparented, republished
	 * into the parent slot (and skip slot, via the shared publish path).
	 * Used by cds_ft_compact() to defragment the node arenas.
	 */
	FT_RECOMPACT_RELOCATE,
};

enum ft_lookup_inequality {
	FT_LOOKUP_GE,
	FT_LOOKUP_LE,
	FT_LOOKUP_GT,
	FT_LOOKUP_LT,
};

enum ft_lookup_limit {
	FT_LOOKUP_LIMIT_NONE,
	FT_LOOKUP_LIMIT_FIRST,
	FT_LOOKUP_LIMIT_LAST,
};

enum ft_direction {
	FT_LEFT,
	FT_RIGHT,
	FT_LEFTMOST,
	FT_RIGHTMOST,
};

/*
 * Fractal Trie iterator object. Can be used to keep backtracking state
 * across API calls. Path use for backtracking requires to keep RCU
 * read-side lock held across calls.
 *
 * The iterator lifetime is bound to the Trie. The Trie must not be
 * destroyed while iterators to that trie exist.
 *
 * The @prefix_len is the length of the key prefix within the key for
 * traversal under a given key prefix. Iterate over the entire Trie when
 * @prefix_len=0.
 */
struct cds_ft_iter {
	struct cds_ft *ft;		/* Point to the associated Fractal Trie. */
	struct cds_ft_node *node;	/* Current external node. */
	size_t path_len;		/* Key-path length of the cached position. */
	size_t key_len;			/* Key length of the current node. */
	size_t prefix_len;		/* Key prefix length. */
	enum cds_ft_status status;	/* Iteration status. */
	enum cds_ft_iter_cache_mode cache_mode;	/* Position-reuse mode (CACHED/UNCACHED). */
	bool cache_valid;		/* Whether the cached position is valid. */
	/*
	 * Byte offset within the @data buffer at which the current-position key
	 * begins.  0 for a descent / set_key key (filled at the front); the
	 * structural up-walk fills the key at the TAIL and sets this to
	 * (max_key_len - key_len) so ft_iter_read_key returns the right pointer
	 * even after the cached position is invalidated (bind / UNCACHED), with
	 * no copy to normalize the key to the front.
	 */
	size_t key_off;

	/*
	 * Ordinal-cell walk cursor.  @ord_cell caches the cell of the current
	 * head so cds_ft_next / cds_ft_prev advance via cell->ord_next/prev
	 * without re-loading the head's leaf each step; @ord_cell_node records
	 * the node it was cached for, so the cache is honoured only while
	 * @ord_cell_node == iter->node (a point lookup or descent that re-seeded
	 * iter->node leaves a mismatch, and the first step re-enters the walk via
	 * iter->node->prev -- the one leaf touch per walk entry).  No stale cell is
	 * dereferenced: validity is a node-pointer compare, not a cell read.
	 */
	struct ft_ord_cell *ord_cell;
	struct cds_ft_node *ord_cell_node;

	/*
	 * CARRIED POSITION KEY (in-trie move coherence).  When @pos_key_node ==
	 * @node, iter_key() holds -- at offset 0, length @key_len -- the key this
	 * position had when the last coherent step CONFIRMED it, as that step's
	 * own traversal spelled it.
	 *
	 * A continuation step taken while a move is in flight is defined against
	 * that KEY and re-descends from it, instead of hopping the ordered-list
	 * cells: a move rewrites the moved run's outer cell links IN PLACE, so a
	 * walker parked on one hops into the run's new neighbourhood and skips
	 * every key in between -- and no reader can detect that, because nothing
	 * it can observe changed address.  The tree path CAN be detected (the move
	 * COWs the moved subtree's top), which is what the two-pass rests on.
	 *
	 * The node pointer makes the carried key SELF-VALIDATING (usable only
	 * while it still describes @node), so no other path has to clear it.
	 */
	struct cds_ft_node *pos_key_node;

#ifdef URCU_FRACTAL_TRIE_DEBUG_PATH
	struct urcu_gp_poll_state gp_state;	/* GP snapshot when path was populated. */
	bool gp_state_valid;			/* Whether gp_state holds a meaningful value. */
#endif

	/*
	 * Trailing buffer holding the ordinal key bytes of the current
	 * iterator position.  The going-up backtrack recovers per-level
	 * nodes from the live parent chain, so no path-node array is kept.
	 *
	 * Flexible array member, pointer-aligned.
	 */
	char data[] __attribute__((__aligned__(sizeof(struct cds_ft_inode_flag *))));
};

/* Start of the uint8_t key array. */
#define iter_key(iter) \
	((uint8_t *)((iter)->data))

/*
 * Validate the iterator-based lookup contract: @ft must be the trie the
 * iterator was created for (cds_ft_iter_create).  The descent uses @ft while
 * key handling uses iter->ft, so passing a different trie mixes their key
 * mappings and produces undefined results.  Debug-only; compiled out under
 * NDEBUG.
 */
static inline
void ft_iter_assert_bound(const struct cds_ft *ft __attribute__((unused)),
		const struct cds_ft_iter *iter __attribute__((unused)))
{
	assert(ft == iter->ft);
}

/*
 * Debug helpers for detecting stale cached iterator paths.
 *
 * Three entry-point roles mirror the rculfhash pattern:
 *
 *  iter_debug_path_snapshot() -- unconditionally captures a fresh
 *      grace-period poll state.  Called at the entry of every
 *      fresh-population operation (lookup, longest-match lookup, and
 *      the slow-path / early-exit branches of inequality lookup).
 *      Because it always overwrites the snapshot, an iterator that is
 *      reused across RCU read-side critical sections gets a current
 *      baseline, preventing false positives on the next check.
 *
 *  iter_debug_path_check() -- polls the existing snapshot.  Called at
 *      continuation entry points that consume a previously populated
 *      cached path (inequality fast-path, replace, remove).  If a full
 *      grace period has elapsed since the snapshot was taken, the RCU
 *      read-side lock must have been dropped and the cached pointers
 *      may reference freed memory -- the check aborts.
 *
 *  iter_debug_path_update() -- invalidates the snapshot when the path
 *      becomes invalid (node not found / end of traversal).  It never
 *      captures a new snapshot; the one taken at the operation's entry
 *      point persists as long as the path remains valid, giving a
 *      tighter detection window.
 *
 *  iter_debug_path_clear() -- unconditionally resets the snapshot
 *      validity.  Used by iter_auto_invalidate_cache() and by
 *      operations that structurally modify the trie (replace, remove),
 *      after which the cached path is stale regardless of RCU state.
 */
#ifdef URCU_FRACTAL_TRIE_DEBUG_PATH

/*
 * Unconditionally capture a fresh grace-period snapshot.  Called at
 * the entry of fresh-population operations so that any prior stale
 * state left by iterator reuse is replaced.
 */
static inline
void iter_debug_path_snapshot(struct cds_ft_iter *iter)
{
	const struct rcu_flavor_struct *flavor = iter->ft->group->flavor;

	iter->gp_state = flavor->update_start_poll_synchronize_rcu();
	iter->gp_state_valid = true;
}

/*
 * Validate that the RCU read-side lock has been held continuously
 * since the snapshot was captured.  Called at continuation entry
 * points before reusing a cached path.
 */
static inline
void iter_debug_path_check(const struct cds_ft_iter *iter)
{
	const struct rcu_flavor_struct *flavor = iter->ft->group->flavor;

	if (iter->cache_mode != CDS_FT_ITER_CACHED)
		return;
	if (!iter->cache_valid)
		return;
	if (!iter->gp_state_valid)
		return;
	if (caa_unlikely(flavor->update_poll_state_synchronize_rcu(
				iter->gp_state))) {
		fprintf(stderr,
			"[Fatal] Fractal Trie: cached iterator path "
			"used after a grace period elapsed (RCU "
			"read-side lock was likely dropped). "
			"%s:%d\n", __FILE__, __LINE__);
		abort();
	}
}

/*
 * Update the snapshot validity after populating the iterator.  When
 * the path is no longer valid (node not found or end of traversal),
 * clear the snapshot so that any subsequent misuse is detected by
 * iter_debug_path_check.  When the path is valid, the grace-period
 * snapshot captured by iter_debug_path_snapshot at the operation's
 * entry point remains current because the RCU read-side lock must be
 * held continuously.
 */
static inline
void iter_debug_path_update(struct cds_ft_iter *iter)
{
	if (!iter->cache_valid)
		iter->gp_state_valid = false;
}

static inline
void iter_debug_path_clear(struct cds_ft_iter *iter)
{
	iter->gp_state_valid = false;
}
#else
static inline
void iter_debug_path_snapshot(struct cds_ft_iter *iter __attribute__((unused)))
{
}

static inline
void iter_debug_path_check(const struct cds_ft_iter *iter __attribute__((unused)))
{
}

static inline
void iter_debug_path_update(struct cds_ft_iter *iter __attribute__((unused)))
{
}

static inline
void iter_debug_path_clear(struct cds_ft_iter *iter __attribute__((unused)))
{
}
#endif

/*
 * Snapshot the iterator's current result key into its own buffer when that key
 * is a live reference into the matched leaf (a lazy-ref ordinal-cell group), so
 * a later re-descent reads a stable key rather than the soon-to-be-reclaimed
 * leaf.  A no-op for groups whose key is already a value in iter_key(iter)
 * (the descent filled it / a non-ordered-list group): the next re-descent uses
 * that buffer directly.  Defined after ft_speculative_keycopy_unconditional.
 */
static inline void ft_iter_materialize_key(struct cds_ft_iter *iter);

/*
 * Discard the cached position if the iterator is in uncached mode.
 * Called at the end of each public iterator-based operation.
 * Preserves iter->node so the caller can read the result.
 */
static inline
void iter_auto_invalidate_cache(struct cds_ft_iter *iter)
{
	if (iter->cache_mode == CDS_FT_ITER_UNCACHED) {
		/*
		 * Materialize a live leaf-referenced key BEFORE clearing, so the
		 * next uncached re-descent reads the saved key, not a stale leaf.
		 */
		ft_iter_materialize_key(iter);
		iter->cache_valid = false;
		iter->path_len = 0;
		iter_debug_path_clear(iter);
	}
}

static
size_t ft_key_len(const struct cds_ft *ft, size_t key_len)
{
	struct cds_ft_group *ft_group = ft->group;

	if (key_len == CDS_FT_LEN_DEFAULT) {
		if (ft_group->key_len == CDS_FT_LEN_VARIABLE)
			return CDS_FT_LEN_ERROR;
		return ft_group->key_len;
	}
	/* Validate that explicit and implicit key lengths match for fixed length Fractal Trie. */
	if (ft_group->key_len != CDS_FT_LEN_VARIABLE && key_len != ft_group->key_len)
		return CDS_FT_LEN_ERROR;
	return key_len;
}

uint64_t cds_ft_key_to_u64(const struct cds_ft *ft, const uint8_t *key,
		size_t _key_len)
{
	size_t key_len = ft_key_len(ft, _key_len);
	union {
		uint64_t v64;
		uint8_t array[8];
	} u;

	if (key_len == CDS_FT_LEN_ERROR || key_len > 8)
		return 0;
	u.v64 = 0;
	/* Copy len LSB. */
	memcpy(u.array + sizeof(u.array) - key_len , key, key_len);
	/* Big endian to host endianness. */
	return be64toh(u.v64);
}

void cds_ft_u64_to_key(const struct cds_ft *ft, uint64_t v, uint8_t *key,
		size_t _key_len)
{
	size_t key_len = ft_key_len(ft, _key_len);
	union {
		uint64_t v64;
		uint8_t array[8];
	} u;

	if (key_len == CDS_FT_LEN_ERROR || key_len > 8)
		return;
	/* Host endianness to big endian. */
	u.v64 = htobe64(v);
	/* Copy len LSB. */
	memcpy(key, u.array + sizeof(u.array) - key_len , key_len);
}

uint32_t cds_ft_key_to_u32(const struct cds_ft *ft, const uint8_t *key,
		size_t _key_len)
{
	size_t key_len = ft_key_len(ft, _key_len);
	union {
		uint32_t v32;
		uint8_t array[4];
	} u;

	if (key_len == CDS_FT_LEN_ERROR || key_len > 4)
		return 0;
	u.v32 = 0;
	/* Copy len LSB. */
	memcpy(u.array + sizeof(u.array) - key_len , key, key_len);
	/* Big endian to host endianness. */
	return be32toh(u.v32);
}

void cds_ft_u32_to_key(const struct cds_ft *ft, uint32_t v, uint8_t *key,
		size_t _key_len)
{
	size_t key_len = ft_key_len(ft, _key_len);
	union {
		uint32_t v32;
		uint8_t array[4];
	} u;

	if (key_len == CDS_FT_LEN_ERROR || key_len > 4)
		return;
	/* Host endianness to big endian. */
	u.v32 = htobe32(v);
	/* Copy len LSB. */
	memcpy(key, u.array + sizeof(u.array) - key_len , key_len);
}

/*
 * Signed integer key helpers.
 *
 * Signed integers need a sign-bit flip (XOR with the MSB of the
 * key-width value) so that the big-endian byte ordering used by the
 * Fractal Trie preserves the natural signed ordering.
 *
 * When the key is the full width of the integer type (e.g. 8 bytes
 * for int64_t), the mapping is:
 *
 *   INT64_MIN  -> 0x0000000000000000   (sorts first)
 *   -1         -> 0x7FFFFFFFFFFFFFFF
 *    0         -> 0x8000000000000000
 *   INT64_MAX  -> 0xFFFFFFFFFFFFFFFF   (sorts last)
 *
 * The same principle applies to 32-bit signed integers.
 *
 * When the key is narrower than the integer type (e.g. a 2-byte key
 * representing a signed 16-bit range within a 64-bit integer), the
 * sign bit is at position (key_len * 8 - 1), not at the MSB of the
 * full integer.  The key-to-integer direction therefore sign-extends
 * from the key's MSB to fill the integer.
 */

int64_t cds_ft_key_to_s64(const struct cds_ft *ft, const uint8_t *key,
		size_t _key_len)
{
	size_t key_len = ft_key_len(ft, _key_len);
	unsigned int shift;
	uint64_t u;

	if (key_len == 0 || key_len > 8)
		return 0;
	u = cds_ft_key_to_u64(ft, key, _key_len);
	shift = key_len * 8;
	/* Flip sign bit (MSB of key-width value) to recover signed encoding. */
	u ^= 1ULL << (shift - 1);
	/* Sign-extend from key width to 64 bits. */
	if (shift < 64) {
		uint64_t sign_bit = 1ULL << (shift - 1);

		if (u & sign_bit)
			u |= ~((1ULL << shift) - 1);
	}
	return (int64_t) u;
}

void cds_ft_s64_to_key(const struct cds_ft *ft, int64_t v, uint8_t *key,
		size_t _key_len)
{
	size_t key_len = ft_key_len(ft, _key_len);
	unsigned int shift;

	if (key_len == 0 || key_len > 8)
		return;
	shift = key_len * 8;
	/* Flip sign bit so that negative values sort before positive. */
	cds_ft_u64_to_key(ft, (uint64_t) v ^ ( 1ULL << (shift - 1)), key, _key_len);
}

int32_t cds_ft_key_to_s32(const struct cds_ft *ft, const uint8_t *key,
		size_t _key_len)
{
	size_t key_len = ft_key_len(ft, _key_len);
	unsigned int shift;
	uint32_t u;

	if (key_len == 0 || key_len > 4)
		return 0;
	u = cds_ft_key_to_u32(ft, key, _key_len);
	shift = key_len * 8;
	/* Flip sign bit (MSB of key-width value) to recover signed encoding. */
	u ^= 1U << (shift - 1);
	/* Sign-extend from key width to 32 bits. */
	if (shift < 32) {
		uint32_t sign_bit = 1U << (shift - 1);

		if (u & sign_bit)
			u |= ~((1U << shift) - 1);
	}
	return (int32_t) u;
}

void cds_ft_s32_to_key(const struct cds_ft *ft, int32_t v, uint8_t *key,
		size_t _key_len)
{
	size_t key_len = ft_key_len(ft, _key_len);
	unsigned int shift;

	if (key_len == 0 || key_len > 4)
		return;
	shift = key_len * 8;
	/* Flip sign bit so that negative values sort before positive. */
	cds_ft_u32_to_key(ft, (uint32_t) v ^ (1U << (shift - 1)), key, _key_len);
}

static inline_lookup
uint8_t key_to_ordinal(uint8_t key,
		const struct cds_ft_key_map *km)
{
	if (caa_likely(km->identity))
		return key;
	return km->key_to_ordinal[key];
}

static inline_lookup
uint8_t ordinal_to_key(const struct cds_ft *ft, uint8_t ordinal)
{
	if (caa_likely(ft->group->key_map.identity))
		return ordinal;
	return ft->group->key_map.ordinal_to_key[ordinal];
}

/*
 * Bulk key-to-ordinal conversion.  Converts @len external key bytes
 * into ordinals in @dst.  Identity maps short-circuit to memcpy.
 */
#include "ft-key.h"

static
struct cds_ft_inode_flag *ft_node_flag(struct cds_ft_inode *node,
		unsigned long type)
{
	assert(type < (1UL << FT_TYPE_BITS));
	return (struct cds_ft_inode_flag *) (((unsigned long) node) |
		(type << FT_INTERNAL_BITS) |
		FT_INTERNAL_MASK);
}

/*
 * Test whether @node has the external tag (bits 0-1 == 0b00).
 * This matches both non-NULL external leaf pointers AND NULL,
 * since NULL has tag bits 0b00.  Callers that need to distinguish
 * NULL from a valid external node should also check ft_node_ptr().
 */
static inline_lookup
bool ft_node_external(struct cds_ft_inode_flag *node)
{
	return ((unsigned long) node & FT_TAG_MASK) == 0;
}

#ifdef FEATURE_FT_COMPRESS
static inline_lookup
bool ft_node_compressed(struct cds_ft_inode_flag *node)
{
	return ((unsigned long) node & FT_TAG_MASK) == FT_COMPRESSED_MASK;
}
#else
static
bool ft_node_compressed(struct cds_ft_inode_flag *node __attribute__((unused)))
{
	return false;
}
#endif

/*
 * ft_metadata_set_external_nodes: Phase 1 -- set the cluster-internal
 * forward pointer (metadata->external_nodes) on a freshly-built node.
 * Asserts that the node is not a compressed node (compressed nodes
 * must not carry metadata->external_nodes).
 *
 * This is the cluster-init step.  The matching back-channel publish
 * (external_nodes->prev = node_flag) is intentionally NOT done here:
 * setting prev makes the cluster reachable to up-walkers via the live
 * external's back-pointer, so it must follow node_flag's own parent
 * being wired.  Use ft_publish_external_nodes_prev for that, ordered
 * after the cluster top's parent is set and immediately before (or as
 * part of) the forward publish.
 *
 * @node_flag: tagged pointer to the node (used for type check).
 * @metadata: the node's metadata.
 * @external_nodes: the external node list to set (may be NULL).
 */
static inline
void ft_metadata_set_external_nodes(struct cds_ft_inode_flag *node_flag,
		struct cds_ft_metadata *metadata,
		struct cds_ft_node *external_nodes)
{
	if (ft_node_compressed(node_flag)) {
		fprintf(stderr, "BUG: ft_metadata_set_external_nodes called on compressed node %p\n", node_flag);
		abort();
	}
	metadata->external_nodes = external_nodes;
	FT_TP(metadata_set_external_nodes, (const void *) node_flag,
		(const void *) external_nodes);
}

/*
 * Pointer unmasking via speculative mask + conditional select.
 *
 * Exploit the fact that each internal node type's allocation order
 * equals 4 + type_idx (type 0 is 16B-aligned, type 1 is 32B, etc.)
 * to compute the internal-node mask speculatively, in parallel with
 * the bit-0 test:
 *
 *   mask_internal = (~15UL) << ((v >> 1) & 7)
 *                 = ~0UL << (4 + type_idx)
 *
 * This clears all tag bits that sit below the type's alignment
 * boundary.  The shift amount is derived purely from bits 1-3 with
 * no dependency on bit 0.
 *
 * For non-internal nodes (bit 0 clear): external nodes are >= 8-byte
 * aligned (bits 0-2 zero), compressed nodes are >= 16-byte aligned
 * with tag in bit 1.  A fixed ~7UL mask suffices.
 *
 * The conditional select lets the two mask computations run in
 * parallel; the compiler emits a CMOV, keeping the critical path
 * to 4 cycles.
 */
/*
 * nr_keys is an MCAS-transacted scalar: on the rank-stats-ON path the
 * order-statistics count is folded into the op's flip-txn (so it is exact under
 * concurrent writers instead of a drifting approximate aggregate).  Like the
 * per-node state word, it therefore reserves its LOW bit for the engine's
 * in-band proxy marker -- the logical count is stored as (count << 1) and bit 0
 * = FT_NR_KEYS_PROXY_TAG carries a parked proxy for the duration of a commit.
 * Bit 0 rather than a high bit so the reservation is valid on 32-bit too.
 * Access nr_keys ONLY through these helpers; never read/write the field direct.
 */
#define FT_NR_KEYS_PROXY_TAG	1UL

/* Writer-side read of an owned / quiescent node (never a mid-commit proxy). */
static inline
unsigned long ft_nr_keys_get(const struct cds_ft_metadata *m)
{
	return m->nr_keys >> 1;
}

/*
 * Reader-side count read: acquire-load and resolve a parked proxy to its
 * committed logical value.  urcu_txn_read short-circuits to a plain acquire
 * load whenever nr_keys holds no proxy (the common case, and always so under
 * writer exclusion), so the resolve costs nothing off the commit window.
 */
static inline
unsigned long ft_nr_keys_load(const struct cds_ft_metadata *m)
{
	return (unsigned long) urcu_txn_read(
			(void **) (uintptr_t) &m->nr_keys,
			FT_NR_KEYS_PROXY_TAG) >> 1;
}

/*
 * Store a node's order-statistics key count -- a no-op unless the trie
 * maintains order statistics (cds_ft_group_attr_set_rank_stats).  Gating the
 * single write chokepoint on @ft->rank_stats means a default (rank-stats-off)
 * trie touches the nr_keys field nowhere: no per-node init, no propagation, no
 * root-ward count contention.  @ft is read-only; on cross-trie ops src/dst
 * share a group (enforced) and thus the same flag, so any in-scope trie works.
 */
static inline
void ft_nr_keys_store(const struct cds_ft *ft, struct cds_ft_metadata *m,
		unsigned long val, int mo)
{
	if (ft->rank_stats)
		uatomic_store(&m->nr_keys, val << 1, mo);
}

/*
 * Reader-side read of a node's live-child count that resolves a mid-commit proxy
 * on the state word.  Once atomic detach is wired the LIVE->DEAD tombstone rides
 * an MCAS edge on state, transiently parking a proxy (a full pointer with bit 0
 * = FT_STATE_PROXY set) that would otherwise corrupt the nr_child bits for a
 * concurrent reader.  urcu_txn_read short-circuits to a plain acquire load when
 * state holds no proxy (always so under writer exclusion), so it costs nothing
 * off the commit window.  The writer-owned ft_meta_nr_child (direct read) stays
 * for reads of a node the caller owns or that is quiescent.
 */
static inline
unsigned int ft_meta_nr_child_load(const struct cds_ft_metadata *meta)
{
	return (unsigned int) (((uintptr_t) urcu_txn_read(
			(void **) (uintptr_t) &meta->state,
			FT_STATE_PROXY) >> FT_STATE_NR_CHILD_SHIFT)
			& FT_STATE_NR_CHILD_VALMASK);
}

/*
 * Proxy-resolving read of a node's parent-slot offset -- the Phase 4.3 mirror of
 * ft_meta_nr_child_load anticipated at the ft_meta_parent_slot_offset declaration.
 * parent_slot_offset shares the state word with the flip proxy, so a RAW read of
 * a live peer-owned node mid-commit returns the proxy pointer's bits as the
 * offset (arbitrary, up to FT_STATE_PSO_VALMASK): ft_get_parent_slot would then
 * compute ptr(parent) + garbage*8 = a WILD address and fault on deref, before any
 * commit guard runs.  urcu_txn_read resolves the proxy to the real state word
 * first (and short-circuits to a plain load when no proxy is parked -- always so
 * under writer exclusion, so it is free off the commit window).  The direct
 * ft_meta_parent_slot_offset stays for a node the caller owns / that is quiescent.
 */
static inline
unsigned int ft_meta_parent_slot_offset_load(const struct cds_ft_metadata *meta)
{
	return FT_PSO_DECODE(urcu_txn_read(
			(void **) (uintptr_t) &meta->parent_slot_offset,
			FT_STATE_PROXY));
}

/*
 * Flip-proxy tag (see the full encoding note above ft_node_flip_proxy's original
 * home, further down).  Hoisted here because the tag-stripping helpers below
 * must be able to assert against it.
 */
#define FT_FLIP_PROXY_TYPE	7U
#define FT_FLIP_PROXY_TAG	(FT_INTERNAL_MASK | (FT_FLIP_PROXY_TYPE << FT_INTERNAL_BITS))

/*
 * The parent slot's tag width must cover the proxy that parks in it: a trie
 * pointer is distinguished from every other parent value by clearing exactly
 * this mask (see ft_parent_is_trie).
 */
urcu_static_assert(FT_FLIP_PROXY_TAG <= FT_PARENT_TAG_MASK,
		"the flip-proxy tag must fit within FT_PARENT_TAG_MASK",
		ft_parent_tag_covers_proxy);

static inline_lookup
bool ft_node_flip_proxy(struct cds_ft_inode_flag *node)
{
	return ((unsigned long) node & (FT_INTERNAL_MASK | FT_TYPE_MASK))
		== FT_FLIP_PROXY_TAG;
}

/*
 * RESOLVED-POINTER CONTRACT.
 *
 * A slot under an in-flight commit does not hold a node: it holds a pointer to
 * the transaction's MCAS record, tagged FT_FLIP_PROXY_TAG (low nibble 0xF).
 * Every structural predicate MISREADS such a flag rather than rejecting it --
 *
 *	ft_node_external(proxy)   -> (0xF & 0b11) == 0b00  -> false
 *	ft_node_compressed(proxy) -> (0xF & 0b11) == 0b10  -> false
 *
 * -- so a proxy silently dispatches as "an internal node of type 7", and the
 * tag-stripping helpers below then mint a wild node pointer out of the record's
 * address.  The fault surfaces frames later, in cds_ft_item_to_metadata(), as a
 * segfault on a garbage range -- with no trace of who failed to resolve.
 *
 * The contract is therefore: RESOLVE FIRST (ft_resolve_flip_proxy), THEN
 * dispatch on kind.  ft_assert_resolved() traps a violation at its first use,
 * where the culprit is still on the stack.  Build with -DFT_DEBUG_PROXY_ASSERT;
 * it compiles to nothing otherwise.
 *
 * Deliberately NOT asserted: _ft_node_mask_ptr() and ft_flip_proxy_ptr(), which
 * exist precisely to strip a proxy's tag, and ft_node_flip_proxy() itself.
 */
#ifdef FT_DEBUG_PROXY_ASSERT
#include <stdio.h>
#include <stdlib.h>

__attribute__((noinline, cold))
static void ft_proxy_assert_fail(const char *fn, const void *p)
{
	fprintf(stderr, "FT_PROXY_ASSERT: %s() got a parked flip proxy %p\n",
		fn, p);
	fflush(stderr);
	abort();
}

# define ft_assert_resolved(node)					\
	do {								\
		if (caa_unlikely(ft_node_flip_proxy(			\
				(struct cds_ft_inode_flag *) (node))))	\
			ft_proxy_assert_fail(__func__,			\
				(const void *) (node));			\
	} while (0)
#else
# define ft_assert_resolved(node)	((void) 0)
#endif

static inline_lookup
struct cds_ft_inode *ft_node_ptr(struct cds_ft_inode_flag *node)
{
	unsigned long v = (unsigned long) node;

	ft_assert_resolved(node);

	/*
	 * Compute mask from the original pointer: the skip-compressed
	 * length bits (57-63) don't affect bits 0-3 used for type
	 * dispatch, so this runs in parallel with the ADDR_MASK AND
	 * below (full ILP).
	 */
	unsigned long mask_internal = (~15UL) << ((v >> 1) & 7);
	unsigned long mask = (v & 1) ? mask_internal : ~7UL;

#ifdef FEATURE_FT_SKIP_COMPRESSED
	/*
	 * Clear the top FT_SKIP_LEN_BITS (7 bits).  In a hot loop
	 * the compiler hoists FT_ADDR_MASK into a register, making
	 * this a single 1-cycle AND that runs in parallel with the
	 * mask chain above.
	 */
	v &= FT_ADDR_MASK;
#endif

	return (struct cds_ft_inode *) (v & mask);
}

/*
 * Identity-only variant: mask a RAW slot value to a comparable address WITHOUT
 * asserting it is resolved.  A slot under an in-flight commit legitimately holds
 * a parked flip proxy, and a conflict check that only compares the masked value
 * against a descent-captured node (proxy != captured -> retry) never
 * dereferences it.  Use this at those sites, and ft_node_ptr() -- which asserts
 * -- everywhere the result is dereferenced.  Never dereference this result.
 */
static inline_lookup
struct cds_ft_inode *ft_node_ptr_raw(struct cds_ft_inode_flag *node)
{
	unsigned long v = (unsigned long) node;
	unsigned long mask_internal = (~15UL) << ((v >> 1) & 7);
	unsigned long mask = (v & 1) ? mask_internal : ~7UL;

#ifdef FEATURE_FT_SKIP_COMPRESSED
	v &= FT_ADDR_MASK;
#endif

	return (struct cds_ft_inode *) (v & mask);
}

/*
 * Lookup-hot variant: caller has already established that the
 * internal-flag bit is set (e.g. ft_node_get_nth_skip checks
 * !(tag & FT_INTERNAL_MASK) and returns NULL before this call).
 * Skips the (v & 1) ? ... : ~7UL branch in ft_node_ptr() above,
 * shaving the cmov/branch from the per-visit dependency chain on
 * the lookup hot path.
 */
static inline_lookup
struct cds_ft_inode *ft_node_ptr_internal(struct cds_ft_inode_flag *node)
{
	unsigned long v = (unsigned long) node;
	unsigned long mask = (~15UL) << ((v >> 1) & 7);

#ifdef FEATURE_FT_SKIP_COMPRESSED
	v &= FT_ADDR_MASK;
#endif

	assert((v & FT_INTERNAL_MASK) || node == NULL);
	return (struct cds_ft_inode *) (v & mask);
}

static
struct cds_ft_inode *_ft_node_mask_ptr(struct cds_ft_inode_flag *node)
{
	unsigned long v = (unsigned long) node;

#ifdef FEATURE_FT_SKIP_COMPRESSED
	v = (v << FT_SKIP_LEN_BITS) >> FT_SKIP_LEN_BITS;
#endif
	return (struct cds_ft_inode *) (v & FT_PTR_MASK);
}

static inline_lookup
bool ft_node_internal(struct cds_ft_inode_flag *node)
{
	return (unsigned long) node & FT_INTERNAL_MASK;
}

static inline_lookup
unsigned long ft_node_type(struct cds_ft_inode_flag *node)
{
	unsigned long type;

	ft_assert_resolved(node);

	if (_ft_node_mask_ptr(node) == NULL) {
		return NODE_INDEX_NULL;
	}
	/* Compressed nodes don't have a type index. */
	assert(!ft_node_compressed(node));
	type = (unsigned int) (((unsigned long) node & FT_TYPE_MASK) >> FT_INTERNAL_BITS);
	assert(type < (1UL << FT_TYPE_BITS));
	return type;
}

/*
 * Flip-proxy encoding (see <urcu/rcu-txn-sw.h>).  cds_ft_merge_at needs
 * to switch a whole set of back-pointers (and the merge-point forward
 * slot) from their old to their new target with no mixed-regime window.
 * Each such slot transiently holds a tagged pointer to a MCAS proxy
 * latch; a single MCAS flip commit store flips them all atomically.
 *
 * A proxy is tagged as a synthetic INTERNAL node of type-index 7, the
 * maximal tag value (low nibble (FT_INTERNAL_MASK | FT_TYPE_MASK) == 0xF).
 * ft_types[7] is FT_NULL on every arch -- the canonical NULL slot on
 * 64-bit (NODE_INDEX_NULL == 7), and reserved padding on 32-bit (where
 * NODE_INDEX_NULL == 6 and real types stop at 5) -- so no real node ever
 * carries type 7 in either tier.  A flag whose low nibble is 0xF is thus a
 * proxy and nothing else (external, compressed, NULL and skip pointers
 * never set all of bits 0..3).  The
 * proxy is 16-byte aligned (low 4 bits free for the tag) and lives at a
 * userspace address with the skip-len high bits clear, so
 * _ft_node_mask_ptr recovers it exactly.  The same encoding is valid in
 * both forward child slots and parent slots, since both resolve a flag
 * through this dispatch.
 */
/* FT_FLIP_PROXY_TYPE / FT_FLIP_PROXY_TAG / ft_node_flip_proxy(): hoisted above
 * ft_node_ptr(), so the tag-stripping helpers can assert against the tag. */

static inline_lookup
struct urcu_txn_record *ft_flip_proxy_ptr(struct cds_ft_inode_flag *node)
{
	return (struct urcu_txn_record *) _ft_node_mask_ptr(node);
}

/*
 * Resolve a possibly-proxied flag to its current target.  Sits right
 * after a parent / root pointer load on the read side; the common case
 * (no mutation in flight) is a single predicted-not-taken mask-compare
 * (ft_node_flip_proxy), and the parked-record deref is reached only during a
 * commit's brief install-to-settle window.  A parked record carries FT's own
 * 0xF tag (see URCU_TXN_PROXY_* in fractal-trie-internal.h), so this masks it
 * off and resolves the record through its MCAS status word.
 */
static inline_lookup
struct cds_ft_inode_flag *ft_resolve_flip_proxy(struct cds_ft_inode_flag *node)
{
	if (caa_unlikely(ft_node_flip_proxy(node))) {
		struct urcu_txn_record *r = ft_flip_proxy_ptr(node);

		return (struct cds_ft_inode_flag *) urcu_txn_resolve_record(r);
	}
	return node;
}

/*
 * Explicit acquire-load for child pointer dereference.
 *
 * Count-based readers (lookup_nth, skip, count_keys) need acquire
 * ordering on child pointer loads to pair with the writer's
 * rcu_assign_pointer (release) during removal.  This ensures that
 * if a reader sees a detached pointer, it also sees the preceding
 * nr_keys decrement (undercount guarantee on weakly-ordered
 * architectures).
 *
 * Current toolchains already compile rcu_dereference (CMM_CONSUME)
 * as CMM_ACQUIRE; this macro makes the acquire unconditional,
 * removing the dependency on the URCU_DEREFERENCE_USE_VOLATILE
 * escape hatch.
 */
static inline_lookup
void ft_maybe_prefetch(const void *ptr)
{
	/*
	 * Prefetch the RAW pointer without clearing the skip-compressed
	 * length high bits.  __builtin_prefetch doesn't fault on
	 * non-canonical addresses (it's a hint that silently drops invalid
	 * loads), so:
	 *   - clean child (~97% on dns): canonical -> prefetch fires with
	 *     zero added latency on the common path;
	 *   - skip-encoded child (~3%): non-canonical -> prefetch dropped.
	 *
	 * Clearing the bits first is a NET LOSS (measured on dns ft_specv,
	 * 2026-05-23): an unconditional mask (& 57-bit imm) and an
	 * unconditional double-shift were BOTH ~2% slower because the clear
	 * sits ahead of the prefetch in the dep chain and delays the
	 * common-case prefetch issue.  A raw-prefetch-then-conditional-clear
	 * shape keeps the common case fast but only TIES no-clear --
	 * prefetching the rare 3% skip children buys nothing measurable.
	 * So: prefetch raw, accept the dropped 3%.  Do NOT re-add the clear
	 * without a skip-heavy workload that shows a real win.
	 *
	 * Prefetch INTERNAL (tagged) children only.  External (leaf) children
	 * (tag bits clear) are random and use-once, and prefetching them is at
	 * best useless and at worst harmful (measured, EPYC 9654 / dns
	 * load-names):
	 *   - under 4 KiB leaf pages it is a no-op -- the leaf-arena TLB miss
	 *     drops the prefetch before its translation resolves;
	 *   - under 2 MiB leaf pages the translation resolves, so the prefetches
	 *     flood the memory controller (~5x more software-prefetch fills
	 *     reach DRAM) and inflate demand-load latency: -27% throughput.
	 * Removing it is neutral at 4 KiB (it was dropped anyway) and removes
	 * that 2 MiB foot-gun.  Internal nodes have descent locality + reuse and
	 * do not flood, so they keep a temporal prefetch.
	 */
	if (((unsigned long) ptr & FT_TAG_MASK) != 0)
		__builtin_prefetch(ptr);
}

/*
 * Non-temporal variant for stream-once spatial prefetch (the inequality
 * adjacent-sibling: a near-future iteration target read once, not reused like a
 * descent node).  prefetchnta fills with minimal cache-level allocation so the
 * streamed siblings do not evict the hot working set -- aimed at the extra LLC
 * traffic the temporal adjacent prefetch adds.  Same internal-only FT_TAG_MASK
 * guard (leaves are the random/use-once 2 MiB-page foot-gun -- see above).
 */
static inline_lookup
void ft_maybe_prefetch_nta(const void *ptr)
{
	if (((unsigned long) ptr & FT_TAG_MASK) != 0)
		__builtin_prefetch(ptr, 0, 0);
}

/*
 * ft_dereference_prefetch: for tagged FT node pointers.  Prefetches the
 * raw pointer via ft_maybe_prefetch, which does NOT clear the
 * skip-compressed length bits (see there: a skip-encoded pointer is
 * non-canonical and its prefetch is silently dropped -- keeping the
 * common-case prefetch un-delayed beats prefetching the rare skip child).
 *
 * ft_dereference_external: for external (leaf) pointers like
 * external_nodes.  external children are deliberately NOT prefetched (see
 * ft_maybe_prefetch: random/use-once leaves drop the prefetch on a 4 KiB TLB
 * miss or flood the memory controller under 2 MiB).  It also RESOLVES a parked
 * flip proxy: a "new key at an existing internal node" publish parks a proxy
 * in external_nodes so it commits atomically with the ordinal-cell splice (one
 * MCAS flip commit), so a reader loading external_nodes must resolve it to the
 * old/new head exactly as it does for a child slot.  The common case (a real
 * head, proxy tag clear) is a single predicted-not-taken tag test.
 */
#define ft_dereference_prefetch(p)		\
	({							\
		__typeof__(p) __ft_tmp = rcu_dereference(p);	\
		ft_maybe_prefetch(__ft_tmp);			\
		__ft_tmp;					\
	})

#define ft_dereference_external(p)					\
	((__typeof__(p)) ft_resolve_flip_proxy(				\
		(struct cds_ft_inode_flag *) rcu_dereference(p)))

#define ft_dereference_acquire_prefetch(p)	\
	({							\
		__typeof__(p) __ft_tmp =			\
			(__typeof__(p)) uatomic_load(&(p),	\
						     CMM_ACQUIRE); \
		ft_maybe_prefetch(__ft_tmp);			\
		__ft_tmp;					\
	})

#define ft_dereference_acquire(p)	\
	(__typeof__(p)) uatomic_load(&(p), CMM_ACQUIRE)

/*
 * Acquire-ordered external-head dereference that ALSO resolves a flip proxy.
 * The ordered-query / rank-select readers (cds_ft_lookup_nth and friends) load
 * metadata->external_nodes with acquire ordering -- to order it against the
 * metadata key counts they consume -- but a concurrent one-commit splice
 * (ft_insert_park_external_nodes) parks a flip proxy in that slot.  Resolve it
 * exactly as ft_dereference_external does for the consume-ordered descent
 * readers, else a tagged proxy would be mistaken for the external head.  The
 * common case (no splice in flight, tag clear) is one predicted-not-taken test.
 */
#define ft_dereference_external_acquire(p)				\
	((__typeof__(p)) ft_resolve_flip_proxy(				\
		(struct cds_ft_inode_flag *) ft_dereference_acquire(p)))

/*
 * Root-slot dereference for read-side descents: like
 * ft_dereference_*(ft->root) but additionally resolves a flip-proxy that
 * a cds_ft_merge_at commit may transiently install at the merge-point
 * forward slot (here, the root).  The resolved flag then flows into the
 * cached path, the key_len==0 / longest-match metadata access, and the
 * child dispatch.  The common case (no merge in flight) is a single
 * predicted-not-taken mask-compare in ft_resolve_flip_proxy.
 */
/*
 * Root-is-internal invariant enforcement.  No mutator ever publishes a
 * compressed / skip-compressed node at the trie root: a would-be compressed
 * root is re-internalized build-invisibly BEFORE publish (ft_make_root_internal_glue
 * on the re-root/graft-swap side; the compressed -> fresh-internal replace on the
 * detach side), so a reader must never observe one.  ft_root_assert_not_compressed()
 * (defined after the skip-compressed helpers below) asserts it on EVERY reader
 * root load -- after flip-proxy resolution, so a merge/graft proxy (which resolves
 * to an internal node in both commit phases) does not trip it -- catching a stray
 * compressed root at first observation rather than as downstream descent
 * corruption.  A NULL root (untagged) passes; compiled out under NDEBUG.
 */
#define ft_root_dereference_prefetch(ft)				\
	ft_root_assert_not_compressed(					\
		ft_resolve_flip_proxy(ft_dereference_prefetch((ft)->root)))
#define ft_root_dereference_acquire_prefetch(ft)			\
	ft_root_assert_not_compressed(					\
		ft_resolve_flip_proxy(ft_dereference_acquire_prefetch((ft)->root)))
#define ft_root_dereference(ft)						\
	ft_root_assert_not_compressed(					\
		ft_resolve_flip_proxy(rcu_dereference((ft)->root)))

/*
 * cn->child dereference for read-side descents: like
 * ft_dereference_acquire_prefetch(cn->child) but additionally resolves a
 * flip-proxy that a key-disappearing remove's recompaction (or external
 * promote) commit transiently installs at cn->child -- the forward edge it
 * flips together with the ordered-cell unsplice.  EVERY reader that descends
 * through a compressed node's child must resolve it; the common case (no such
 * remove in flight) is a single predicted-not-taken mask-compare.
 */
#define ft_cn_child_dereference_acquire_prefetch(cn)			\
	ft_resolve_flip_proxy(ft_dereference_acquire_prefetch((cn)->child))

/*
 * Per-caller prefetch hint for ft_node_get_nth_skip / ft_node_get_nth
 * and the underlying scanners.  Compile-time constant at each call
 * site -- the branches inside ft_maybe_prefetch_hint fold away, leaving
 * at most a single prefetch per caller.
 *
 *   FT_PF_NONE:        no prefetch.
 *   FT_PF_DATA:        prefetch child's data (node body).  Right for
 *                      candidate lookup and non-skip exact lookup
 *                      that traverse the returned child's data next.
 */
enum ft_pf_target {
	FT_PF_NONE,
	FT_PF_DATA,
};

static inline_lookup
void ft_maybe_prefetch_hint(const void *ptr, enum ft_pf_target hint)
{
	switch (hint) {
	case FT_PF_NONE:
		break;
	case FT_PF_DATA:
		ft_maybe_prefetch(ptr);
		break;
	}
}

#define ft_dereference_acquire_prefetch_hint(p, hint)			\
	({								\
		__typeof__(p) __ft_tmp =				\
			(__typeof__(p)) uatomic_load(&(p),		\
						     CMM_ACQUIRE);	\
		ft_maybe_prefetch_hint(__ft_tmp, (hint));		\
		__ft_tmp;						\
	})

#define ft_dereference_prefetch_hint(p, hint)				\
	({								\
		__typeof__(p) __ft_tmp = rcu_dereference(p);		\
		ft_maybe_prefetch_hint(__ft_tmp, (hint));		\
		__ft_tmp;						\
	})

/*
 * Return @node's external_nodes (the dup-chain head hanging off an
 * internal or compressed node).  @node must be internal or compressed.
 * Used to recover a cached iterator position's deepest trie node without
 * re-descending: a prefix key sits at an internal/compressed node
 * whose external_nodes == iter->node.
 */
static inline_lookup
struct cds_ft_node *ft_node_external_nodes(struct cds_ft_inode_flag *node)
{
	struct cds_ft_metadata *metadata;

	assert(!ft_node_external(node));
	if (ft_node_compressed(node))
		metadata = cds_ft_item_to_metadata(ft_node_ptr(node));
	else {
		const struct cds_ft_type *type = &ft_types[ft_node_type(node)];

		metadata = cds_ft_item_to_metadata_fast(ft_node_ptr(node),
				type->order);
	}
	return ft_dereference_external(metadata->external_nodes);
}

/*
 * Speculative inequality result-key capture: when the group is configured for
 * speculative skip-compressed lookup with a leaf-key offset, the matched leaf
 * @leaf stores the full result key -- in the byte order the application passed
 * to cds_ft_insert() -- at that offset.  Transform @level bytes of it to the
 * iterator's ordinal (trie) order into @dst and return true; the caller then
 * need not rebuild the key from the descent's compressed-node bytes.
 * ft_key_to_ordinals applies the group's key map, which is a plain copy for an
 * identity map and a per-byte remap otherwise, so the fast path covers
 * non-identity maps too (no identity restriction).  Returns false (copying
 * nothing) on groups without the offset / skip-compressed encoding, so the
 * caller falls back to the descent-built ordinal_key accumulation.
 *
 * This is the result-key source on a configured speculative group: the
 * min-descent intentionally leaves dispatch-irrelevant holes in ordinal_key
 * (it follows skip pointers without filling the spanned bytes), so the leaf
 * copy -- not ordinal_key -- carries the full result key.  Correctness is
 * validated end-to-end by the ordered-iteration / relational invariant tests.
 */
static inline_lookup
bool ft_speculative_keycopy(const struct cds_ft *ft,
		const struct cds_ft_node *leaf,
		uint8_t *dst, ssize_t level)
{
	const struct cds_ft_group *group = ft->group;
	const uint8_t *leaf_key;

	if (!leaf || level < 0)
		return false;
	/*
	 * Per-trie gate: a trie that opted out of speculative keys
	 * (cds_ft_attr_set_speculative_keys false) must NOT copy from the leaf's
	 * stored key here -- its leaves may hold a key that does not match their
	 * position -- so the caller falls back to the descent-built ordinal_key.
	 */
	if (!ft->speculative_key_offset_active || !group->speculative ||
			!(group->flags & CDS_FT_FLAG_SKIP_COMPRESSED))
		return false;
	leaf_key = (const uint8_t *) leaf + group->speculative_key_offset;
	ft_key_to_ordinals(dst, leaf_key, (size_t) level, &group->key_map);
	return true;
}

/*
 * Unconditional variant of ft_speculative_keycopy for callers that have already
 * resolved use_keycopy at compile time: the leaf-copy config gate
 * (speculative_key_offset_set && speculative && SKIP_COMPRESSED) is then known
 * to hold, so the runtime re-check folds away.  Copies @level ordinal bytes of
 * @leaf's stored key into @dst; a NULL @leaf (NOT_FOUND) or @level < 0 copies
 * nothing (the result key is then unused).
 */
static inline_lookup
void ft_speculative_keycopy_unconditional(const struct cds_ft *ft,
		const struct cds_ft_node *leaf, uint8_t *dst, ssize_t level)
{
	const struct cds_ft_group *group = ft->group;
	const uint8_t *leaf_key;

	if (!leaf || level < 0)
		return;
	leaf_key = (const uint8_t *) leaf + group->speculative_key_offset;
	ft_key_to_ordinals(dst, leaf_key, (size_t) level, &group->key_map);
}

/*
 * Ordinal-cell tag + accessors (cell-always model).
 *
 * Every duplicate-chain HEAD has a library-owned ordinal cell (struct
 * ft_ord_cell), and the head's cds_ft_node.prev points to it.  The head's
 * flagged parent is relocated into ft_ord_cell.parent; the cell is the
 * external head's metadata record, peer to internal/compressed metadata.
 *
 * The cell pointer is tagged with FT_INTERNAL_MASK (bit 0) so the head-vs-dup
 * test ft_node_external(prev)==false is preserved (a non-head dup's prev is an
 * untagged external cds_ft_node, bits 0-1 == 0).  Bit 0 is the ONLY tag: in a
 * cell build a head's prev is ALWAYS a cell, so there is nothing to
 * distinguish and no per-pointer marker is needed (32-bit safe).  Cells are
 * >= 2-byte aligned, so bit 0 is free.
 *
 * The DOWNWARD child slots still point straight at the external node; the cell
 * is interposed only on the UPWARD walk (parent recovery) and ordered
 * traversal.  Every reader of a head's prev-as-parent resolves through
 * ft_resolve_head_prev (identity outside the feature).
 */
#define FT_ORD_CELL_TAG		FT_INTERNAL_MASK

#include "ft-lookup-helpers.h"

/*
 * ft_node_holder: write-side resolution of a node's holder (the slot owner
 * "above" it), independent of the cell relocation.
 *
 *   - non-head duplicate: prev is the predecessor cds_ft_node (external).
 *   - head: prev is the flagged parent directly (non-cell build) or the
 *     cell whose ->parent holds the flagged parent (cell build).
 *   - never-inserted (prev NULL): returns NULL.
 *
 * Mutex-held callers (remove / replace / locate-chain-head) that previously
 * read node->prev as the holder route through this so the cell indirection
 * is transparent.  Identity in non-cell builds.
 */
static inline
struct cds_ft_inode_flag *ft_node_holder(struct cds_ft *ft,
		const struct cds_ft_node *node)
{
	/*
	 * Resolve a parked flip proxy at the load: once a head-promote folds
	 * its prev-inherit store onto the commit flip-txn (Phase 4.3), this
	 * word transiently carries FT's type-7 proxy, and a raw external/cell
	 * classification of the proxy value would mis-derive the holder.
	 */
	void *prev = (void *) ft_resolve_flip_proxy((struct cds_ft_inode_flag *)
			rcu_dereference(((struct cds_ft_node *) node)->prev));

	if (ft_node_external((struct cds_ft_inode_flag *) prev))
		return (struct cds_ft_inode_flag *) prev;
	return ft_resolve_head_prev(ft, prev);
}

/*
 * ft_chain_head_holder: resolve the trie HOLDER (the head's IMMEDIATE PARENT)
 * of @node's duplicate chain -- the single lockable state-word node every op on
 * the chain serialises on (MW LOCK_FINE holder lock).  @node may be the head
 * (its prev is the cell / flagged parent -> resolve directly, one iteration) or
 * an interior duplicate (its prev is a predecessor external -> walk prev up to
 * the head, whose prev is NOT external).  A flip proxy parked on prev mid-commit
 * resolves at each load, as in ft_node_holder.  HOLDER-LOCK / mutex-held callers
 * only: the prev walk is not stable under a concurrent chain relink (an interior
 * op can only find its head's holder to lock while some coarser exclusion --
 * today the FT-wide writer_lock -- still holds; that chicken-and-egg is resolved
 * with the FT-wide-lock drop).  Returns NULL for a never-inserted node (prev
 * NULL).
 */
static inline
struct cds_ft_inode_flag *ft_chain_head_holder(struct cds_ft *ft,
		struct cds_ft_node *node)
{
	struct cds_ft_node *cur = node;

	for (;;) {
		void *prev = (void *) ft_resolve_flip_proxy(
			(struct cds_ft_inode_flag *)
			rcu_dereference(cur->prev));

		if (!prev)
			return NULL;
		if (!ft_node_external((struct cds_ft_inode_flag *) prev))
			return ft_resolve_head_prev(ft, prev);
		cur = (struct cds_ft_node *) prev;
	}
}

/*
 * Record a fresh head's flagged parent.  Non-cell builds store it directly
 * into the (pre-publish) head's prev; cell builds store it into the head's
 * pre-wired cell (node->prev already carries the cell), leaving prev intact.
 * For the fresh-head wiring sites only (the subsequent forward publish
 * orders this store); existing-head re-parents use ft_set_parent / the
 * external-nodes choke point, which resolve the cell themselves.
 */
#define ft_external_head_set_parent(ft, node, parent)			\
	do {								\
		if ((ft)->ordered_list)					\
			ft_ord_cell_set_parent((node),			\
				(struct cds_ft_inode_flag *) (parent));	\
		else							\
			(node)->prev = (parent);			\
	} while (0)

/*
 * ft_publish_external_nodes_prev: Phase 2 -- publish the back-channel pointer
 * up from the displaced/transferred external head @external_nodes to its
 * (re-)parent @node_flag via rcu_assign_pointer.
 *
 * Call AFTER node_flag's own parent is wired (so an up-walker arriving via
 * the new back-channel lands on a parent-wired cluster top, not a NULL
 * parent), and at-or-just-before the forward publish that makes the cluster
 * reachable through node_flag's slot.  No-op when @external_nodes is NULL
 * (callers commonly guard on metadata->external_nodes).
 *
 * Ordered list on: @external_nodes is an existing head, so its prev already
 * carries its cell; record the new parent into cell->parent (the head's
 * prev -- the cell pointer -- is unchanged).  All choke-point callers
 * re-parent an existing head (a fresh head's parent is wired by ft_set_parent
 * via ft_node_set_nth), so the cell is guaranteed present.  List off / non-cell:
 * the head's prev IS the flagged parent, so re-parent it directly.
 */
static inline
void ft_publish_external_nodes_prev(struct cds_ft *ft,
		struct cds_ft_inode_flag *node_flag,
		struct cds_ft_node *external_nodes)
{
	if (!external_nodes)
		return;
	if (ft->ordered_list)
		ft_ord_cell_set_parent(external_nodes, node_flag);
	else
		rcu_assign_pointer(external_nodes->prev, node_flag);
}

static
struct cds_ft_inode_flag *ft_compressed_node_flag(
		struct cds_ft_compressed_node *node)
{
	return (struct cds_ft_inode_flag *)
		(((unsigned long) node) | FT_COMPRESSED_MASK);
}

static inline_lookup
struct cds_ft_compressed_node *ft_compressed_node_ptr(
		struct cds_ft_inode_flag *node)
{
	ft_assert_resolved(node);
	return (struct cds_ft_compressed_node *)
		(((unsigned long) node) & ~(unsigned long) FT_TAG_MASK);
}

#ifdef FT_DEBUG_PARENT_VIOLATION
#include <stdio.h>
/*
 * The same violation the ordered up-walk dumps (ft-iter.h), at the OTHER load
 * that carries the invariant.  A bare assert here names only the READER -- the
 * thread that followed the link -- while the question is about the node whose
 * @parent_word produced the illegal value, and nothing on this frame survives
 * into a core to name it at -O1.
 *
 * @node is that node, i.e. the one suspected of having been retired and freed
 * while still referenced.  Print its metadata, state and rcu_head words at the
 * point of detection instead of reconstructing them afterwards.
 */
__attribute__((noinline, cold, unused))
static void ft_parent_rcu_violation(struct cds_ft *ft,
		struct cds_ft_inode_flag *node,
		struct cds_ft_inode_flag *bad)
{
	struct cds_ft_metadata *m = NULL;

	if (!ft_node_external(node))
		m = ft_node_compressed(node) ?
			cds_ft_item_to_metadata((struct cds_ft_inode *)
				ft_compressed_node_ptr(node)) :
			cds_ft_item_to_metadata(ft_node_ptr(node));
	fprintf(stderr, "FT_PARENT_RCU_VIOLATION: illegal parent %p\n",
		(void *) bad);
	fprintf(stderr, "  ft=%p node=%p meta=%p\n",
		(void *) ft, (void *) node, (void *) m);
	if (m) {
		fprintf(stderr, "  node parent_word=%p state=0x%lx\n",
			(void *) m->parent_word, (unsigned long) m->state);
		fprintf(stderr, "  node rcu_head words: %p %p\n",
			((void **) m)[-2], ((void **) m)[-1]);
	}
	fprintf(stderr, "  bad rcu_head words: %p %p\n",
		((void **) bad)[0], ((void **) bad)[1]);
	fflush(stderr);
	abort();
}
# define ft_parent_rcu_check(ft, node, parent)				\
	do {								\
		if (caa_unlikely((parent) && ft_node_external(parent)))	\
			ft_parent_rcu_violation((ft), (node), (parent));	\
	} while (0)
#else
# define ft_parent_rcu_check(ft, node, parent)				\
	do {								\
		(void) (node);						\
		assert(!(parent) || !ft_node_external(parent));		\
	} while (0)
#endif

/*
 * ft_get_parent_rcu: read the parent pointer of @node via
 * rcu_dereference.
 *
 * For external (leaf) nodes: returns cds_ft_node.prev.  @node must
 * be the head of its duplicate chain (non-head duplicates' prev
 * points to the preceding node in the chain, not to the parent).
 * Iterators and lookups maintain this invariant by convention --
 * iter->node always refers to the chain head.
 *
 * For internal/compressed nodes: returns metadata->parent.
 *
 * Returns NULL when @node is at the root position, or when @node
 * has been orphaned by a concurrent detach / graft_swap that
 * cleared its parent link.  A read-side parent-pointer walk that
 * observes NULL terminates cleanly in either case.
 *
 * Read-side safe; the caller must be in an RCU read-side critical
 * section (or QSBR equivalent).
 *
 * An assertion verifies the returned parent is never external: an
 * external result would mean the caller passed a non-head duplicate
 * chain entry (whose prev points at the preceding duplicate, not
 * at the parent).
 *
 * Config-agnostic (only the flip-proxy resolve, a merge primitive,
 * and basic accessors): kept here, ahead of the FEATURE_FT_SKIP_COMPRESSED
 * block, so cds_ft_merge_at can use it in all build configs.
 */
static inline
struct cds_ft_inode_flag *ft_get_parent_rcu(struct cds_ft *ft,
		struct cds_ft_inode_flag *node)
{
	struct cds_ft_inode_flag *parent;

	if (ft_node_external(node))
		parent = ft_resolve_head_prev(ft,
			ft_dereference_prev_resolved((struct cds_ft_node *) node));
	else if (ft_node_compressed(node))
		/*
		 * A compressed node's metadata lives at a FT_TAG_MASK-cleared
		 * offset, not the FT_TYPE_MASK-cleared one ft_node_ptr uses; a
		 * referenced compressed child re-parented by a merge reaches
		 * here (after the caller resolves any skip form to its raw
		 * compressed flag).
		 */
		parent = ft_parent_node(rcu_dereference(cds_ft_item_to_metadata(
			(struct cds_ft_inode *) ft_compressed_node_ptr(node))->parent_word));
	else
		parent = ft_parent_node(rcu_dereference(cds_ft_item_to_metadata(
			ft_node_ptr(node))->parent_word));
	/*
	 * The parent slot may transiently hold a flip-proxy during a
	 * cds_ft_merge_at commit; resolve it to the view-appropriate
	 * (old or merged) parent before returning.
	 */
	parent = ft_resolve_flip_proxy(parent);
#ifdef FT_ENABLE_TRACING
	/*
	 * FIRE AT DETECTION, not from a signal handler: the prior rig recorded
	 * that a SIGSEGV-handler snapshot lands long after the ring has wrapped.
	 * This is the same instant the assert below would abort on, but with the
	 * window still intact.
	 *
	 * A freelist link is 8-mod-16, so it clears the EXTERNAL tag -- which is
	 * exactly why a reclaimed parent reads as an external node here.  The
	 * round trip (item -> metadata -> item) is the discriminator: equal means
	 * a valid live object with a wrong link, unequal means recycled memory.
	 */
	/*
	 * POSITIVE CONTROL for the whole emit -> freeze -> stop -> snapshot ->
	 * decode chain.  A violation site that has never been shown to LAND in a
	 * readable trace is an instrument on trust, and this one has already
	 * produced two aborts whose snapshots contained no violation event.
	 * FT_TRACE_SELFTEST=1 fires the identical sequence on the first call,
	 * from a state that is perfectly healthy, so a snapshot WITHOUT the event
	 * indicts the rig and one WITH it clears the rig.
	 */
	{
		static int selftest = -1;

		if (caa_unlikely(selftest < 0))
			selftest = getenv("FT_TRACE_SELFTEST") ? 1 : 0;
		if (caa_unlikely(selftest == 1)) {
			selftest = 0;
			FT_TP(parent_external_violation, node, parent, parent,
				0xdeadbeefUL);
			ft_trace_capture();
			fprintf(stderr, "FT_TRACE_SELFTEST: fired\n");
			abort();
		}
	}
	if (caa_unlikely(parent && ft_node_external(parent))) {
		const void *pp = ft_node_ptr(parent);
		const void *rt = NULL;

		rt = cds_ft_metadata_to_item(cds_ft_item_to_metadata(
			(struct cds_ft_inode *) pp));
		FT_TP(parent_external_violation, node, pp, rt,
			(unsigned long) rcu_dereference(cds_ft_item_to_metadata(
				ft_node_ptr(node))->parent_word));
		/*
		 * STOP FIRST, then snapshot.  system() is a fork+exec costing
		 * milliseconds, and this workload emits millions of events per
		 * second -- the 64 KiB ring wraps several times over during it,
		 * so a plain "snapshot record" here dumps a window that no
		 * longer contains the violation that triggered it.  Measured:
		 * the snapshot was written and the event was already gone.
		 * lttng stop freezes every buffer before the dump, at the cost
		 * of ending tracing for the run -- which is exactly what we
		 * want, since this process is about to abort anyway.
		 */
		ft_trace_capture();
		abort();
	}
#endif
	ft_parent_rcu_check(ft, node, parent);
	return parent;
}

/* Skip-compressed pointer helpers. */

#ifdef FEATURE_FT_SKIP_COMPRESSED
static inline
bool ft_node_skip_compressed(struct cds_ft_inode_flag *node)
{
	return ((unsigned long) node >> FT_SKIP_LEN_SHIFT) != 0;
}

static inline
unsigned int ft_skip_len(struct cds_ft_inode_flag *node)
{
	return (unsigned long) node >> FT_SKIP_LEN_SHIFT;
}

/*
 * ft_skip_child_ptr: extract the child tagged pointer from a skip
 * pointer by clearing the skip-length bits.
 */
static inline
struct cds_ft_inode_flag *ft_skip_child_ptr(struct cds_ft_inode_flag *node)
{
	ft_assert_resolved(node);
	return (struct cds_ft_inode_flag *) ((unsigned long) node & FT_ADDR_MASK);
}

/*
 * ft_skip_compressed_flag: encode a skip pointer from a child pointer
 * and the compressed path length.
 */
static
struct cds_ft_inode_flag *ft_skip_compressed_flag(
		struct cds_ft_inode_flag *child, unsigned int len)
{
	assert(len > 0 && len <= FT_SKIP_LEN_MAX);
	/*
	 * The encoding ORs len into the high bits of child.  If child
	 * already carries skip-length bits (i.e., is itself a skip-
	 * compressed pointer), the OR conflicts with len and produces
	 * a corrupted nested encoding from which neither len nor child
	 * can be recovered cleanly.  Chain-compress canonicalization
	 * is responsible for ensuring that cn->child is never skip-
	 * compressed at publish time (the "no two adjacent compresseds"
	 * invariant).  Assert the invariant here so any future regression
	 * fails loudly under -UNDEBUG smoke tests rather than silently
	 * corrupting the trie.
	 */
	assert(((unsigned long) child >> FT_SKIP_LEN_SHIFT) == 0);
	return (struct cds_ft_inode_flag *)
		((unsigned long) child |
		 ((unsigned long) len << FT_SKIP_LEN_SHIFT));
}

/*
 * ft_skip_to_compressed: recover the compressed node from a skip
 * pointer by following the child's parent back-pointer.
 *
 * For internal/compressed children: uses metadata->parent.
 * For external (leaf) children: uses cds_ft_node.prev (which points
 * to the parent for the head of a duplicate chain).
 *
 * No validation against the slot's skip_len: callers (including
 * writers in mid-mutation, where the back-pointer and slot value
 * are intentionally inconsistent for a brief window) get whatever
 * the back-pointer currently says.  Reader paths that must observe
 * a self-consistent slot+cn pair re-anchor via ft_skip_reanchor,
 * which walks the skip child's live parent chain to the trie
 * position the slot's skip_len encodes.
 *
 * Read-side safe (rcu_dereference on both fields).  Callers must be
 * in an RCU read-side critical section (or QSBR equivalent).
 */
static inline
struct cds_ft_compressed_node *ft_skip_to_compressed(const struct cds_ft *ft,
		struct cds_ft_inode_flag *skip_ptr)
{
	struct cds_ft_inode_flag *child = ft_skip_child_ptr(skip_ptr);
	struct cds_ft_inode_flag *parent;

	if (ft_node_external(child))
		/*
		 * @child is a head; resolve its flagged parent through
		 * ft_resolve_head_prev, which branches on the group's ordered_list
		 * mode (cell-indirect vs prev-direct).  A structural tag test is NOT
		 * usable here: a skip pointer with a STALE target (a concurrent
		 * split/merge moved the encoded position) can transiently make this
		 * head's parent an internal node -- FT_INTERNAL_MASK, bit 0, the same
		 * tag a cell carries -- so only the mode flag disambiguates safely.
		 */
		parent = ft_resolve_head_prev(ft,
			ft_dereference_prev_resolved((struct cds_ft_node *) child));
	else
		parent = ft_parent_node(rcu_dereference(cds_ft_item_to_metadata(
			ft_node_ptr(child))->parent_word));
	/*
	 * The recovered back-pointer may transiently be a flip proxy during a
	 * cds_ft_merge_at commit -- a dst-origin subtree root wrapped under a
	 * freshly-built compressed node in the merged cluster has its parent
	 * staged through the latch.  Resolve it before masking to the compressed
	 * node (otherwise the proxy's tag bits fold into a bogus pointer),
	 * mirroring ft_get_parent_rcu and the descent's resolve-then-skip order.
	 * A concurrent reader up-walking through the merge's flip window resolves
	 * via the shared selector to the consistent old-or-merged view; a no-op
	 * when no merge is in flight.
	 */
	return ft_compressed_node_ptr(ft_resolve_flip_proxy(parent));
}


/*
 * ft_skip_reanchor: the single concurrency-handling mechanism for skip-
 * compressed pointers.  A skip slot encodes a length (skip_len), but the live
 * compressed node recovered via the skip child's back-pointer may no longer
 * match it (a concurrent split/merge changed the path between the slot and the
 * child), so readers resolve the slot by re-anchoring rather than trusting the
 * recovered node directly.
 *
 * Spinning to re-read the slot does NOT converge when the slot lives on a node
 * that was recompacted away (frozen-stale) while the skip child was reparented
 * to a different-length compressed by a concurrent split/merge: the frozen
 * slot is never republished, so the reader would loop forever.  The skip child
 * @G, however, is reachable in the LIVE trie (a live leaf or live internal), so
 * its parent chain runs through live nodes that converge.  Walk it up,
 * accumulating consumed path length (a compressed spans its len, an internal
 * one byte), until the accumulated length reaches the slot's skip_len: that
 * locates the live tree position the failing slot encoded.
 *
 *   - split (live path lengthened into prefix+branch+suffix at the same total
 *     length): the accumulation lands exactly, @*rewind == 0; re-anchor at the
 *     live node at the same depth.
 *   - merge (live path shortened by absorbing the slot's level into a longer
 *     compressed): the first hop already exceeds skip_len; the encoded position
 *     is now interior to that compressed.  Re-anchor shallower (its parent) and
 *     have the caller rewind its descent cursor by @*rewind bytes.
 *
 * @skip_ptr: the failing skip pointer (encodes child @G + skip_len).
 * @rewind:   out -- bytes the caller must back its descent cursor/level up by.
 * @at_pos:   out (may be NULL) -- the live node spanning/at the encoded position
 *            (the merge target for rewind > 0).  Accumulator walkers (nth /
 *            iter_skip) descend INTO it on rewind > 0, because re-scanning the
 *            shallower holder would re-count its already-counted contributions.
 *            Idempotent walkers (inequality minmax/sibling) and the precise
 *            lookup ignore it and just re-scan / re-read the returned holder.
 *
 * Returns the live node holding the slot equivalent to the failing one (the
 * caller re-anchors its descent there and re-reads / re-descends).  Never
 * returns NULL on a well-formed trie: the writer wires every fresh cluster's
 * parent (including the cluster top's, into the live parent) before the
 * cluster becomes reachable, so the up-walk never observes a NULL parent.  All
 * call sites assert anchor != NULL and treat any NULL return as a bug.  The
 * defensive `return NULL` paths inside the walk (assert(0) + return NULL under
 * NDEBUG; pathological guard exhaustion) exist only so a debug build aborts at
 * the violation site instead of dereferencing NULL.
 *
 * Read-side only (rcu_dereference on every back-pointer); the caller must be in
 * an RCU read-side critical section.
 */
static
struct cds_ft_inode_flag *ft_skip_reanchor(struct cds_ft *ft,
		struct cds_ft_inode_flag *skip_ptr,
		unsigned int *rewind, struct cds_ft_inode_flag **at_pos)
{
	unsigned int want = ft_skip_len(skip_ptr);
	unsigned int acc = 0;
	struct cds_ft_inode_flag *cur = ft_skip_child_ptr(skip_ptr);	/* G */
	int guard;

	*rewind = 0;
	if (at_pos)
		*at_pos = NULL;
	FT_TP(reanchor_enter, (const void *) skip_ptr, (const void *) cur, want);
	for (guard = 0; guard < (int) FT_MAX_DEPTH + 2; guard++) {
		struct cds_ft_inode_flag *parent;
		void *pitem;

		if (ft_node_external(cur))
			parent = ft_resolve_head_prev(ft,
				ft_dereference_prev_resolved((struct cds_ft_node *) cur));
		else
			parent = ft_parent_node(rcu_dereference(
				cds_ft_item_to_metadata(
				ft_node_ptr(cur))->parent_word));
		/* A picked child's parent may be a flip-proxy mid-merge. */
		parent = ft_resolve_flip_proxy(parent);
		FT_TP(reanchor_walk, (const void *) cur, (const void *) parent, acc);
		if (caa_unlikely(!parent)) {
			/*
			 * A NULL parent on the up-walk is a bug.  The walk runs
			 * through LIVE nodes whose parents are wired before the node
			 * becomes reader-reachable: a build-invisible commit connects
			 * the whole fresh cluster's parents (including the top's)
			 * before any live gateway exposes it
			 * (ft_graft_glue_apply_deferred), and a detach nulls parent
			 * only after a grace period (unobservable to an in-flight
			 * reader).  The only legitimate NULL parent is the root's,
			 * and the accumulation reaches @want at or below it -- there
			 * are no root-level skip pointers -- so the walk never steps
			 * onto it.
			 */
			assert(0);
			return NULL;		/* defensive under NDEBUG */
		}
		pitem = ft_node_compressed(parent) ?
			(void *) ft_compressed_node_ptr(parent) :
			(void *) ft_node_ptr(parent);
		acc += ft_node_compressed(parent) ?
			ft_compressed_node_ptr(parent)->len : 1U;
		if (acc >= want) {
			/*
			 * @parent is the node spanning (rewind > 0, a merge) or
			 * sitting at (rewind == 0) the failing slot's encoded
			 * position.  Return the node HOLDING the slot (its parent,
			 * = the scanned node's live version for rewind == 0); also
			 * hand back @parent itself via @at_pos so accumulator
			 * walkers can descend INTO it for rewind > 0 (where
			 * re-scanning the shallower holder would double-count).
			 */
			struct cds_ft_inode_flag *holder;

			*rewind = acc - want;
			if (at_pos)
				*at_pos = parent;
			/* Same flip-proxy resolve as the up-walk read above. */
			holder = ft_resolve_flip_proxy(ft_parent_node(
				rcu_dereference(cds_ft_item_to_metadata(
				(struct cds_ft_inode *) pitem)->parent_word)));
			/*
			 * The holder is NULL only if @parent is the root -- the
			 * encoded position is the root itself, i.e. a root-level
			 * skip pointer, which mutators never produce.
			 */
			assert(holder != NULL);
			return holder;
		}
		cur = parent;
	}
	return NULL;	/* pathological (cycle?): caller re-descends from root */
}

static inline
bool ft_group_skip_compressed(const struct cds_ft_group *group)
{
	return group->flags & CDS_FT_FLAG_SKIP_COMPRESSED;
}
#else
static inline
bool ft_node_skip_compressed(struct cds_ft_inode_flag *node __attribute__((unused)))
{
	return false;
}

static inline
unsigned int ft_skip_len(struct cds_ft_inode_flag *node __attribute__((unused)))
{
	return 0;
}

static inline
struct cds_ft_inode_flag *ft_skip_child_ptr(struct cds_ft_inode_flag *node)
{
	return node;
}

static
struct cds_ft_inode_flag *ft_skip_compressed_flag(
		struct cds_ft_inode_flag *child,
		unsigned int len __attribute__((unused)))
{
	return child;
}

static inline
struct cds_ft_compressed_node *ft_skip_to_compressed(const struct cds_ft *ft,
		struct cds_ft_inode_flag *skip_ptr)
{
	(void) ft;
	return ft_compressed_node_ptr(skip_ptr);
}


static inline
bool ft_group_skip_compressed(const struct cds_ft_group *group __attribute__((unused)))
{
	return false;
}

#endif /* FEATURE_FT_SKIP_COMPRESSED */

/*
 * Assert the root-is-internal invariant on a reader's root load.  See the
 * ft_root_dereference_* macros above: no mutator ever publishes a compressed /
 * skip-compressed node at the trie root, so a reader must never observe one.
 * Defined here (not at the macros) because it needs ft_node_skip_compressed,
 * whose real definition and no-skip stub both live in the block above.
 * Compiled out under NDEBUG.
 */
static inline
struct cds_ft_inode_flag *ft_root_assert_not_compressed(
		struct cds_ft_inode_flag *root)
{
	assert(!ft_node_compressed(root) && !ft_node_skip_compressed(root));
	return root;
}

/*
 * ft_set_parent_slot: record a node's slot within its parent as a
 * pointer-stride offset (parent_slot_offset).  The raw byte offset is
 * divided by sizeof(void *) (always 8 on 64-bit) so that the 8-bit
 * field can cover the full pigeon node (2048 bytes / 8 = 256 slots,
 * max index 255).
 *
 * Maintained for every internal/compressed node, not just
 * skip-compressed ones: it lets the parent-pointer backtrack recover a
 * node's parent slot in O(1) without re-descending, and it backs the
 * skip-compressed dual-pointer publish / chain-merge canonicalization.
 *
 * When parent is NULL (root's child), the offset is unused --
 * ft_get_parent_slot recovers &ft->root.
 */
/* Recover the branch byte indexing @slot in internal parent @node (defined
 * after the popcount layout helpers). */
static uint8_t ft_slot_to_byte(const struct cds_ft_type *type,
		struct cds_ft_inode *node, struct cds_ft_inode_flag **slot);

static inline
void ft_set_parent_slot(struct cds_ft_metadata *meta,
		struct cds_ft_inode_flag *parent,
		struct cds_ft_inode_flag **slot)
{
	struct cds_ft_inode_flag *p;
	bool parent_compressed;

	if (!slot)
		return;	/* Slot unknown -- preserve existing offset. */
	if (!parent) {
		ft_meta_parent_slot_offset_set(meta, 0);
		return;
	}
	ft_meta_parent_slot_offset_set(meta, (unsigned int)((char *) slot -
		(char *) ft_node_ptr(parent)) / sizeof(void *));
	/*
	 * Record this node's incoming branch byte for the up-walk key rebuild
	 * (ft_rebuild_key_upwalk).  This is THE central populate point for every
	 * slot-placed node (internal + compressed) -- it runs from ft_set_parent
	 * AND ft_publish_to_parent, so all publish paths are covered without
	 * threading the byte to each call site.  Derive it by inverting @slot
	 * against the parent's bitmap (cold path).  Only meaningful when the
	 * parent is an internal (slot-array) node: a compressed parent has no
	 * slot array -- the edge byte lives in its key_bytes -- so skip it (the
	 * up-walk likewise skips a node whose parent is compressed).
	 */
	p = parent;
	parent_compressed = ft_node_compressed(p);
#ifdef FEATURE_FT_SKIP_COMPRESSED
	parent_compressed = parent_compressed || ft_node_skip_compressed(p);
#endif
	if (!parent_compressed)
		meta->incoming_byte = ft_slot_to_byte(
			&ft_types[ft_node_type(p)], ft_node_ptr(p), slot);
}

/*
 * ft_get_parent_slot: recover a node's parent-slot address from the
 * stored pointer-stride offset.
 *
 * The node body IS the packed child-pointer array (metadata lives in a
 * sibling page), so offset 0 is a valid slot -- the node's first/lowest
 * child.  A placed non-root child therefore always has a meaningful
 * offset, including 0; the "no recorded slot" state is fully captured by
 * parent == NULL (the root's child, recovered as &ft->root below).  Do
 * NOT treat offset 0 as "unset": that aliases the lowest child of every
 * node and silently drops its skip re-encode (ft_publish_to_parent /
 * ft_node_recompact would skip it on a child-change, leaving a stale
 * skip pointer in the parent slot).
 *
 * @ft is needed for the root case (parent == NULL).
 */
/*
 * ft_resolve_parent_slot: recover a node's parent AND its parent-slot address as
 * a CONSISTENT snapshot, tolerating a mid-commit atomic re-home (Phase 4.3).
 *
 * A re-home commits @meta->parent (a type-7 flip-proxy) and @meta->parent_slot_
 * offset (FT_STATE_PROXY) as ONE 2-edge MCAS txn.  Resolving each field with a
 * SEPARATE status load can tear across the commit's status flip -- read parent ->
 * OLD, flip, read offset -> NEW -> a slot address computed off the wrong parent
 * body -> ft_slot_to_byte OOB.  So the two edges must be driven from a SINGLE
 * status snapshot: when @meta->parent carries the proxy, take its txn @t, read
 * urcu_txn_desc_status(t) ONCE, and resolve BOTH edges through it.  A re-home
 * that begins mid-snapshot is caught by the coherence re-read of @meta->parent
 * and retried.
 *
 * The §8.3 split SHARPENED this: the offset now has its own word, written ONLY
 * by a re-home, so a proxy parked there is unambiguously @t's.  While the offset
 * shared @state, that word was also parked by freezes and tombstones -- edges
 * that leave the offset untouched -- so the resolver had to tell a co-committed
 * offset edge apart from an unrelated state edge.  The same-descriptor check
 * below still earns its keep against a re-home racing the snapshot, but it no
 * longer has to disambiguate two different KINDS of parker.
 *
 * No re-home in flight (single writer, or between commits) => the fast path is a
 * plain parent load + a proxy-tolerant offset load, behaviour-identical to the
 * pre-4.3 raw recovery.  @parent_out (optional) receives the parent that matches
 * the returned slot.  Call from within an RCU read-side section.
 */
static inline
struct cds_ft_inode_flag **ft_resolve_parent_slot(
		const struct cds_ft_metadata *meta, struct cds_ft *ft,
		struct cds_ft_inode_flag **parent_out)
{
	struct cds_ft_inode_flag *parent;
	void *state;

	for (;;) {
		struct cds_ft_inode_flag *praw = rcu_dereference(meta->parent_word);
		void *sraw = uatomic_load(
				(void **) (uintptr_t) &meta->parent_slot_offset,
				CMM_ACQUIRE);

		if (caa_likely(!ft_node_flip_proxy(praw))) {
			/*
			 * No re-home parking @meta->parent.  The state word may
			 * still carry an unrelated proxy (a freeze that leaves the
			 * offset unchanged); resolve it independently.  A re-home
			 * that STARTS after this load is caught by the coherence
			 * re-read below.
			 */
			parent = praw;
			state = urcu_txn_resolve(sraw, FT_STATE_PROXY);
		} else {
			struct urcu_txn_record *rp = ft_flip_proxy_ptr(praw);
			struct urcu_txn_desc *t = rp->desc;
			unsigned long st = urcu_txn_desc_status(t);

			parent = (struct cds_ft_inode_flag *)
				(st == URCU_TXN_DESC_SUCCEEDED ? rp->new_ptr : rp->old_ptr);
			if (caa_likely(urcu_txn_is_proxy(sraw, FT_STATE_PROXY))) {
				struct urcu_txn_record *rs =
					urcu_txn_untag(sraw, FT_STATE_PROXY);

				if (caa_unlikely(rs->desc != t))
					continue;	/* stale offset edge: re-snapshot */
				state = st == URCU_TXN_DESC_SUCCEEDED ?
						rs->new_ptr : rs->old_ptr;
			} else {
				/* Parent parked but offset already settled: re-snapshot. */
				continue;
			}
		}
		/*
		 * Coherence guard: the snapshot is torn only if a re-home landed
		 * on @meta->parent between the two loads above.  A stable parent
		 * edge means (parent, offset) came from one consistent view.
		 */
		if (caa_likely(rcu_dereference(meta->parent_word) == praw))
			break;
	}

	/*
	 * The owner stamp is a WITNESS, never a navigation aid: the slot comes
	 * from the caller's @ft, exactly as it did when a root's parent was
	 * NULL.  A cross-trie move legitimately leaves the stamp naming the
	 * PREVIOUS owner until it commits, so deriving the slot from the stamp
	 * would answer with a slot in the wrong trie mid-move -- while @ft is
	 * the trie the caller is actually operating on.  Reading ownership is
	 * cds_ft_verify's job, and it reads the word itself.
	 *
	 * @parent_out is the parent NODE, so a root still yields NULL.
	 */
	if (parent_out)
		*parent_out = ft_parent_node(parent);
	if (ft_parent_is_root_position(parent))
		return &ft->root;
	return (struct cds_ft_inode_flag **)
		((char *) ft_node_ptr(parent) +
		 FT_PSO_DECODE(state) * sizeof(void *));
}

static inline
struct cds_ft_inode_flag **ft_get_parent_slot(const struct cds_ft_metadata *meta,
		struct cds_ft *ft)
{
	return ft_resolve_parent_slot(meta, ft, NULL);
}

/*
 * ft_txn_parent_slot: @meta's parent slot as @mtxn WILL LEAVE IT.
 *
 * ft_resolve_parent_slot above answers from the words as they stand: it
 * resolves a PEER's parked re-home through the flip proxy, but an edge this
 * op has merely RECORDED is not parked yet and lives only in the descriptor,
 * so a raw derivation cannot see it.  An op that re-parents @meta and then
 * derives @meta's parent slot in the same attempt therefore gets the PRE-OP
 * slot -- and when the re-parent came from a recompaction, that slot sits
 * inside the copy this very commit retires.
 *
 * So read both words READ-YOUR-OWN-WRITES.  The (parent, offset) pair needs no
 * coherence re-read here: a re-parent records them together, so the txn returns
 * one op's view of both, and any word without a pending edge falls through to
 * the same waiting load ft_resolve_parent_slot performs.
 *
 * @mtxn NULL answers exactly as ft_get_parent_slot does.
 */
static inline
struct cds_ft_inode_flag **ft_txn_parent_slot_at(const struct cds_ft_metadata *meta,
		struct cds_ft *ft, struct urcu_txn *mtxn,
		struct cds_ft_inode_flag **parent_out)
{
	struct cds_ft_inode_flag *parent;
	void *state;

	if (parent_out)
		*parent_out = NULL;
	if (!mtxn)
		return ft_resolve_parent_slot(meta, ft, parent_out);
	parent = urcu_txn_load(mtxn,
		(void **) (uintptr_t) &meta->parent_word, FT_FLIP_PROXY_TAG);
	state = urcu_txn_load(mtxn,
		(void **) (uintptr_t) &meta->parent_slot_offset, FT_STATE_PROXY);
	if (ft_parent_is_root_position(parent))
		return &ft->root;
	/*
	 * @parent_out is the node the returned slot LIVES IN -- the word that
	 * owns that slot (§8.2: a node's body is its own).  Handed back rather
	 * than re-derived by the caller, because only the RYW load above can see
	 * a re-parent this very commit recorded.
	 */
	if (parent_out)
		*parent_out = ft_parent_node(parent);
	return (struct cds_ft_inode_flag **)
		((char *) ft_node_ptr(parent) +
		 FT_PSO_DECODE(state) * sizeof(void *));
}

#define ft_txn_parent_slot(meta, ft, mtxn)				\
	ft_txn_parent_slot_at((meta), (ft), (mtxn), NULL)

/*
 * ft_slot_in_node: is @slot one of @node_flag's OWN child slots?
 *
 * The inverse of ft_get_parent_slot, which computes every slot as the node body
 * plus a pointer-stride offset: the node body IS the packed child-pointer array,
 * sized (1 << type->order) and naturally aligned, so containment is an address
 * range test.  A compressed node carries exactly one child slot, @cn->child.
 *
 * This is the pairing test for a plan that holds a slot address INSIDE one node
 * and a separately-resolved pointer TO that node.  Both can be individually
 * coherent while the PAIR is not: a peer that republishes the node between the
 * descent that produced the slot and the load that resolved the pointer leaves
 * the slot addressing the retired body while the pointer names the fresh copy.
 * No expected-old on the holder slot can see that -- the value there matches
 * itself; what is stale is the premise the descent established about the node.
 *
 * @node_flag must already be resolved (no flip proxy); a skip form resolves to
 * the compressed node it names.  A value that holds no child slot at all (NULL,
 * an external chain) answers false.
 */
static inline
bool ft_slot_in_node(struct cds_ft_inode_flag *node_flag,
		struct cds_ft_inode_flag **slot)
{
	const char *body;

	if (!node_flag || !slot)
		return false;
	/* Skip before external: a skip pointer's low bits read as external. */
	node_flag = ft_skip_child_ptr(node_flag);
	if (ft_node_compressed(node_flag))
		return slot == &ft_compressed_node_ptr(node_flag)->child;
	if (!ft_node_internal(node_flag))
		return false;
	body = (const char *) ft_node_ptr(node_flag);
	if (!body)
		return false;
	return (const char *) slot >= body &&
		(const char *) slot < body +
			((size_t) 1 << ft_types[ft_node_type(node_flag)].order);
}


#ifdef FT_ENABLE_TRACING
#include <stdio.h>
#include <stdlib.h>
/*
 * Flight-recorder WRITE-side mis-wire validator (tracing builds only): a
 * forward-slot publish whose NEW value is a PLAIN compressed flag must name
 * a live cn whose (parent, PSO) pair derives the very slot being written --
 * every legit producer (fresh publish, recompact republish) wires the pair
 * before publishing, and the SKIP_X dual writes the skip FORM (exempt via
 * the plain-only filter).  Publishing a plain cn flag into any OTHER slot,
 * or naming a tombstoned / len==0 target, is a mis-wire caught AT CREATION
 * -- the abort core shows exactly who built the edge, and the snapshot
 * carries the window.  Wired at the _ft_publish_to_parent(_meta) entries
 * (the canonical forward-slot recorders), NOT at the txn record choke point:
 * back-pointer FIELDS legitimately hold plain cn flags (a child's
 * meta->parent naming its fresh cn parent) and would false-fire there.
 */
static __attribute__((unused))
void ft_trace_pub_check(struct cds_ft *ft,
		struct cds_ft_inode_flag **slot, struct cds_ft_inode_flag *nf,
		unsigned int site)
{
	struct cds_ft_compressed_node *cn;
	struct cds_ft_metadata *meta;
	uintptr_t state;
	struct cds_ft_inode_flag *rt_parent;
	struct cds_ft_inode_flag **rt_slotp;

	if (!nf || !ft_node_compressed(nf))
		return;
	cn = ft_compressed_node_ptr(nf);
	meta = cds_ft_item_to_metadata((struct cds_ft_inode *) cn);
	state = (uintptr_t) urcu_txn_read((void **) &meta->state,
			FT_STATE_PROXY);
	rt_parent = ft_resolve_flip_proxy(ft_parent_node(
			rcu_dereference(meta->parent_word)));
	rt_slotp = rt_parent ? ft_get_parent_slot(meta, ft) : NULL;
	if (caa_likely(cn->len != 0 && !(state & FT_STATE_TOMBSTONE) &&
			rt_slotp == slot))
		return;
	FT_TP(miswire, site, (const void *) nf, (const void *) cn,
		(unsigned int) cn->len, state, (const void *) rt_parent,
		(const void *) rt_slotp);
	fprintf(stderr, "FT PUB-MISWIRE site %u slot %p new %p target %p "
		"len %u state %#lx rt_parent %p rt_slot %p\n",
		site, (void *) slot, (void *) nf, (void *) cn,
		(unsigned int) cn->len, (unsigned long) state,
		(void *) rt_parent, (void *) rt_slotp);
	(void) system("lttng snapshot record 1>&2");
	abort();
}
#ifndef FT_LIGHT_TRACING	/* see FT_TRACE_MISWIRE: drop the per-publish
				 * round-trip validator, keep the tracepoints. */
#define FT_TRACE_PUB_CHECK(ft, slot, nf, site) \
	ft_trace_pub_check(ft, slot, nf, site)
#else
#define FT_TRACE_PUB_CHECK(ft, slot, nf, site) do { (void) (ft); (void) (slot); (void) (nf); (void) (site); } while (0)
#endif
#else
#define FT_TRACE_PUB_CHECK(ft, slot, nf, site) do { } while (0)
#endif	/* FT_ENABLE_TRACING */

/*
 * ft_flag_to_metadata: get the metadata for any node flag, including
 * skip-compressed pointers.  For skip pointers, returns the
 * compressed node's metadata.  For all others, returns
 * cds_ft_item_to_metadata(ft_node_ptr(nf)).
 *
 * Caller must ensure nf is not NULL and not external.
 */
static inline
struct cds_ft_metadata *ft_flag_to_metadata(const struct cds_ft *ft,
		struct cds_ft_inode_flag *nf)
{
	(void) ft;
#ifdef FEATURE_FT_SKIP_COMPRESSED
	if (ft_node_skip_compressed(nf)) {
		struct cds_ft_compressed_node *cn =
			ft_skip_to_compressed(ft, nf);
		return cds_ft_item_to_metadata(
			(struct cds_ft_inode *) cn);
	}
#endif
	return cds_ft_item_to_metadata(ft_node_ptr(nf));
}

/*
 * ft_flag_tombstoned: has this node flag been RETIRED?
 *
 * A retired node keeps its body readable (RCU) but its state word carries
 * FT_STATE_TOMBSTONE, and every fence primitive refuses it -- so an op whose
 * plan names a retired node can never make progress and must re-derive rather
 * than retry.  An EXTERNAL head has no state word and is never "tombstoned" in
 * this sense (its removal is signalled on node->next, see ft_node_is_removed).
 */
static inline
bool ft_flag_tombstoned(const struct cds_ft *ft,
		struct cds_ft_inode_flag *nf)
{
	struct cds_ft_metadata *meta;

	if (!nf || ft_node_external(nf))
		return false;
	meta = ft_flag_to_metadata(ft, nf);
	return meta != NULL &&
		(CMM_LOAD_SHARED(meta->state) & FT_STATE_TOMBSTONE) != 0;
}

/*
 * If @nf is a skip-compressed pointer, return the underlying
 * compressed node's flag pointer.  Otherwise return @nf unchanged.
 *
 * Use to "see through" the skip-compressed encoding when about to
 * inspect or recurse into the underlying compressed node.  No-op for
 * non-skip pointers; on archs without FEATURE_FT_SKIP_COMPRESSED the
 * check is constant-folded to false and the call collapses to a copy.
 */
static inline
struct cds_ft_inode_flag *ft_resolve_skip_compressed(const struct cds_ft *ft,
		struct cds_ft_inode_flag *nf)
{
	(void) ft;
	if (ft_node_skip_compressed(nf))
		return ft_compressed_node_flag(ft_skip_to_compressed(ft, nf));
	return nf;
}

/*
 * ft_reanchor_flag: resolve an (already flip-proxy-resolved) child flag that may
 * be skip-compressed to a LIVE node, via the read side's ft_skip_reanchor --
 * which walks the skip child's live parent chain to the trie position skip_len
 * encodes -- instead of the unvalidated one-hop ft_skip_to_compressed.
 *
 * This is the MW-safe replacement for ft_resolve_skip_compressed on the WRITE
 * path.  Under mutual exclusion a writer could trust the one-hop back-pointer
 * (no peer could move the structure under its own descent); under MW a peer
 * split/merge can tear that back-pointer into internal memory (type confusion:
 * cn->len reads a bitmap byte) or a stale-length node (mis-file) -- exactly what
 * the read side already tolerates by re-anchoring.  The update side converges on
 * that same mechanism here.
 *
 * @*rewind_ret is set > 0 iff a concurrent chain-merge moved the encoded
 * position shallower than the dispatched child (a caller that captured the raw
 * slot then finds it at the wrong level, and re-descends).  Non-skip @child
 * returns unchanged with rewind 0.  ft_skip_reanchor never returns NULL on a
 * well-formed trie; the result is asserted non-NULL.  RCU-read-side safe.
 */
static inline
struct cds_ft_inode_flag *ft_reanchor_flag(struct cds_ft *ft,
		struct cds_ft_inode_flag *child, unsigned int *rewind_ret)
{
	*rewind_ret = 0;
#ifdef FEATURE_FT_SKIP_COMPRESSED
	if (caa_unlikely(child && ft_node_skip_compressed(child))) {
		struct cds_ft_inode_flag *at_pos, *anchor;

		anchor = ft_skip_reanchor(ft, child, rewind_ret, &at_pos);
		assert(anchor != NULL);
		return at_pos;
	}
#else
	(void) ft;
#endif
	return child;
}

#ifdef FEATURE_FT_SKIP_COMPRESSED
/*
 * ft_skip_to_compressed_meta: shorthand to get the compressed node's
 * metadata from a skip pointer.
 */
static inline
struct cds_ft_metadata *ft_skip_to_compressed_meta(struct cds_ft *ft,
		struct cds_ft_inode_flag *skip_ptr)
{
	return cds_ft_item_to_metadata(
		(struct cds_ft_inode *) ft_skip_to_compressed(ft, skip_ptr));
}
#endif

/*
 * ft_publish_to_parent: atomically publish @new_child into @parent_slot.
 *
 * If the parent is a compressed node, also update the skip pointer
 * at *skip_slot (if one exists) BEFORE writing *parent_slot.  This
 * ensures candidate readers (which follow the skip pointer) see the
 * new child before exact/inequality readers (which follow cn->child).
 *
 * For compressed-form @new_child (SKIP_X or plain COMPRESSED), also
 * maintains the underlying compressed node's parent_slot_offset so it
 * records @parent_slot's offset in @parent_nf -- required by
 * ft_get_parent_slot lookups (the dual-pointer dance above, and the
 * chain-merge canonicalization in ft_detach_node that publishes a
 * replacement at the cn's same grandparent slot).  Without this,
 * compressed nodes installed via ft_publish_to_parent rather than via
 * ft_node_set_nth -> ft_set_parent leave parent_slot_offset == 0 -- a
 * latent gap that silently disabled the dual-pointer SKIP_X update
 * and tripped chain-merge.  This intentionally does NOT update
 * @new_child's parent linkage; callers manage that via their own
 * ft_set_parent (or by direct meta->parent assignment), with
 * semantics that vary across call sites.
 *
 * Centralizes the dual-pointer RCU publication pattern so every
 * write to cn->child automatically maintains the skip pointer.
 *
 * When @rec is non-NULL, the (1-2) reader-visible stores -- the forward
 * parent slot and a compressed parent's SKIP_X dual pointer -- are RECORDED
 * into @rec instead of being performed, so a key-disappearing remove can
 * commit them in one flip with the dead head cell's ordered-list unsplice
 * (ft_detach_node).  All non-reader-visible bookkeeping (parent-slot offset,
 * trace events) still runs.  The public ft_publish_to_parent wrapper passes
 * NULL (direct stores, original behaviour).
 */
static
void ft_pub_rec_add(struct ft_pub_rec *rec, struct cds_ft_inode_flag **slot,
		struct cds_ft_inode_flag *expected_old,
		struct cds_ft_inode_flag *new_val, bool root,
		struct cds_ft_metadata *owner, bool owner_held)
{
	assert(rec->n < 3);
	/*
	 * @owner_held: does the OP hold @owner's lock?  Separate from @owner
	 * because a NULL @owner does NOT fail closed -- the dispatching
	 * recorder branches on the txn's structural_sw alone -- so naming no
	 * owner and holding no owner are the same to it.  false records MW.
	 */
	rec->owner_held[rec->n] = owner_held;
	rec->slot[rec->n] = slot;
	/*
	 * @owner: the node whose lock excludes every other writer of @slot.
	 * Every slot here is a BODY slot -- a forward child pointer or a SKIP_X
	 * dual -- and mw-writer-lock-escalation-model.md §8.2 puts a node's body
	 * under that node's own lock, so the owner is the node the slot LIVES IN,
	 * never the child it points at.
	 *
	 * NULL where the producer cannot name one, and for a ROOT slot, which has
	 * no owning node at all and takes the always-MW route through @root
	 * instead.  A producer that leaves this unset only declines to convert
	 * (see struct ft_pub_rec), so a missing owner is safe and merely counts
	 * OWN_MISS -- which is what it did at EVERY producer before this
	 * parameter existed.
	 */
	rec->owner[rec->n] = owner;
	/*
	 * @root: a TRIE ROOT slot, which every replay must record MW (see
	 * struct ft_pub_rec).  A parameter rather than a re-derivation,
	 * because only the caller holds the trie the slot would be compared
	 * against -- and a cross-trie op holds two.
	 */
	rec->root[rec->n] = root;
	/*
	 * @expected_old is the value the slot held in the snapshot the
	 * publishing PLAN was derived from -- NOT a fresh *slot re-read at
	 * record time.  Recording the plan snapshot makes the commit-time
	 * MCAS reject (abort) a peer that published into this slot between the
	 * plan and the record, instead of ratifying the stale plan because a
	 * fresh raw capture happens to match the peer's value (CORE_682870
	 * defect #1: "stale plan ratified by fresh expected-old").
	 */
	rec->old_val[rec->n] = expected_old;
	rec->new_val[rec->n] = new_val;
	rec->n++;
}

/*
 * IS THE SKIP_X DUAL'S HOME A NODE THIS COMMIT BUILT?
 *
 * The dual lives in @cn's OWN parent, and the publish resolves that parent
 * READ-YOUR-OWN-WRITES -- so when the SAME commit re-parents @cn, the dual's
 * home MOVES, from the live node the descent fenced to the fresh copy this op
 * built.  A fresh copy is BUILD-INVISIBLE: no peer can reach it, no lock is
 * needed, and none is taken -- so a transacted record naming it as owner names
 * a word the commit cannot be shown to own, and no owner encoding fixes that.
 * The txn publishes REACHABILITY, not INTERIORS: a private node's body is wired
 * by PLAIN STORES, and the dual write is exactly that.
 *
 * ☠ THE STORE IS DEMOTED, NEVER DROPPED.  The recompact copy loop resolved the
 * old home's slots to their COMMITTED values, so the fresh copy is born holding
 * the skip pointer to the OLD child -- stale the instant the forward publish
 * lands.  Skipping the write would leave a candidate reader a shortcut straight
 * to a retired node.
 *
 * ☞ THE TEST IS "DID THIS TXN RE-HOME @cn", asked of the DESCRIPTOR.
 * urcu_txn_load cannot answer it: with no record on the slot it falls through
 * to a fresh read, which is indistinguishable from "not re-homed".  The
 * divergence of the two derivations is a weaker witness of the same fact.
 *
 * A ROOT dual (&ft->root) is excluded whatever the descriptor says: it lives in
 * no node, it is reader-visible at all times, and it takes the always-MW root
 * route.
 *
 * ☠ THIS LEANS ON ONE INVARIANT: a re-parent target is a FRESH cluster, never a
 * live node (ft_reparent_record_meta says so, and every producer in the tree
 * obeys it today).  A future re-homer that targets a LIVE node would turn this
 * dispatch into a plain store into a live body -- silently.  The debug arm below
 * is what would catch it: a re-home that did not MOVE the home is the shape that
 * cannot be private.
 */
static inline
bool ft_dual_home_is_private(struct cds_ft *ft, const struct ft_pub_rec *rec,
		struct cds_ft_metadata *cn_meta,
		struct cds_ft_inode_flag **skip_slot,
		struct cds_ft_inode_flag *skip_owner_nf)
{
	struct urcu_txn_desc *desc;

	if (!rec || !rec->mtxn || skip_slot == &ft->root || !skip_owner_nf)
		return false;
	desc = rec->mtxn->desc;
	if (!desc || desc == URCU_TXN_ENOMEM)
		return false;
	if (!urcu_txn_find(desc, (void **) (uintptr_t) &cn_meta->parent_word))
		return false;
#if defined(DEBUG_RCU) || defined(CONFIG_RCU_DEBUG)
	{
		/*
		 * THE INVARIANT, CHECKED WHERE IT IS CHEAP: a re-home that did
		 * not MOVE the home cannot be private, and this is the arm that
		 * would notice a future re-parent target that is a LIVE node.
		 * Guarded on the same pair urcu_assert_debug itself is, so the
		 * raw re-resolution costs a release build nothing.
		 */
		struct cds_ft_inode_flag *raw_owner = NULL;

		(void) ft_txn_parent_slot_at(cn_meta, ft, NULL, &raw_owner);
		urcu_assert_debug(raw_owner != skip_owner_nf);
	}
#endif
	return true;
}

static
void _ft_publish_to_parent_meta(struct cds_ft *ft,
		struct cds_ft_inode_flag *parent_nf,
		struct cds_ft_inode_flag **parent_slot,
		struct cds_ft_inode_flag *new_child,
		struct cds_ft_inode_flag *expected_old,
		struct cds_ft_metadata *new_child_meta,
		void *folded_child_prev,
		struct ft_pub_rec *rec,
		struct cds_ft_inode_flag *slot_owner_nf,
		bool dual_owner_held)
{
	/*
	 * @slot_owner_nf: the node @parent_slot LIVES IN, i.e. the word that owns
	 * the forward edge (§8.2: a node's body is its own).  Almost always
	 * @parent_nf, which is why _ft_publish_to_parent defaults it -- but NOT
	 * always, and the difference cannot be derived here: a caller may pass
	 * @parent_nf for its OTHER job, deciding whether a compressed parent's
	 * SKIP_X dual is re-emitted, while publishing into a slot that lives
	 * somewhere else entirely (ft_store_at_graft_point_commit's relocation
	 * republish).  Deriving the owner from @parent_nf there names a node the op
	 * does not hold and the record reports an exclusion gap that is not real.
	 */
	/*
	 * @expected_old: the value @parent_slot held in the snapshot the
	 * caller's publish plan was derived from (the RECORDED path only; the
	 * direct rec==NULL arms below republish a same value and ignore it).
	 * The compressed-parent SKIP_X dual mirrors the same child, so its
	 * plan-snapshot value is ft_skip_compressed_flag(expected_old, cn->len)
	 * -- both edges commit against the plan, not a record-time re-read.
	 */
	/*
	 * @new_child_meta (optional): @new_child's metadata, supplied by the
	 * caller so we DON'T recover it from the slot value.  Required when the
	 * caller defers @new_child's back-pointer into the same flip-txn as this
	 * publish: a SKIP_X @new_child is resolved to its compressed node via
	 * ft_skip_to_compressed, which reads new_child's child's parent -- which
	 * is precisely the deferred (not-yet-stored) back-edge.  Passing the
	 * metadata directly avoids that stale read.  NULL = recover as before
	 * (the back-edge was wired up front).
	 *
	 * @folded_child_prev (optional): the EXTERNAL analogue -- an external
	 * @new_child carries no metadata (its parent resolves through prev ->
	 * cell -> parent), so when the caller folds @new_child's prev into this
	 * publish's flip-txn (a head promote / swap), the forward-before-parent
	 * check below would read the not-yet-stored prev.  Passing the prev's
	 * intended value lets the check validate the folded parent instead.
	 * NULL = read new_child->prev as before.  Debug-check only.
	 *
	 * Publication-ordering invariant: a child becomes observable by
	 * downward traversal the instant it is published into a live parent
	 * slot, so its parent back-pointer MUST already be wired.  Otherwise
	 * a concurrent reader that descends to it and walks back up
	 * (ft_skip_reanchor / ordered up-walk) reads a NULL/uninitialized
	 * parent.  Applies to ALL child kinds (external -> prev,
	 * internal/compressed -> metadata->parent); the root slot is the sole
	 * exception (the root has no parent).  Catches forward-before-parent
	 * bugs at their source.
	 */
	/* Flight-recorder write-side mis-wire validator (tracing builds only). */
	FT_TRACE_PUB_CHECK(ft, parent_slot, new_child, 4);
#ifndef NDEBUG
	if (new_child && parent_slot != &ft->root) {
		struct cds_ft_inode_flag *cp;

		/*
		 * Check skip-compressed FIRST: a SKIP_X flag carries its
		 * (external) child's low tag bits, so ft_node_external() would
		 * misclassify it and dereference the tagged flag as a node.
		 * The caller-supplied metadata short-circuits the SKIP_X recovery
		 * (which would read the deferred back-edge).
		 */
		if (new_child_meta) {
			cp = ft_parent_node(new_child_meta->parent_word);
		} else
#ifdef FEATURE_FT_SKIP_COMPRESSED
		if (ft_node_skip_compressed(new_child)) {
			cp = ft_parent_node(cds_ft_item_to_metadata(
				(struct cds_ft_inode *)
				ft_skip_to_compressed(ft, new_child))->parent_word);
		} else
#endif
		if (ft_node_external(new_child)) {
			/*
			 * Cell-always: prev is the (non-NULL) cell pointer even
			 * when the parent is unset, so resolve through the cell to
			 * preserve the forward-before-parent check on cell->parent.
			 * When the caller folds the prev into this publish's txn it
			 * supplies the intended value (@folded_child_prev) so we
			 * validate the folded parent, not the not-yet-stored slot.
			 */
			cp = ft_resolve_head_prev(ft, folded_child_prev ?
				folded_child_prev :
				((struct cds_ft_node *) new_child)->prev);
		} else {
			cp = ft_parent_node(cds_ft_item_to_metadata(
				ft_node_ptr(new_child))->parent_word);
		}
		assert(cp != NULL);
	}
#endif /* !NDEBUG */
	(void) folded_child_prev;	/* debug-check only (see above) */
	/*
	 * Maintain @new_child's parent-slot offset (parent_slot_offset) so
	 * that it records the slot holding it within its parent node.  This
	 * is the value ft_get_parent_slot(child_meta) recovers later -- used by
	 * the parent-pointer backtrack to find a node's slot in O(1) without
	 * re-descending, by dual-pointer publishes from cn->child
	 * (ft_publish_to_parent itself, when called with parent_nf = cn) and
	 * by chain-merge canonicalization (ft_detach_node) that publishes a
	 * replacement at the same slot.
	 *
	 * Maintained for EVERY internal/compressed child (not just
	 * compressed): the offset field is no longer skip-specific.  On
	 * skip-on builds, without this, compressed nodes installed via
	 * ft_publish_to_parent (rather than via ft_node_set_nth, which routes
	 * through ft_set_parent) leave parent_slot_offset == 0 -- a latent gap
	 * that silently disabled the dual-pointer SKIP_X update at the
	 * grandparent slot and tripped chain-merge that *needs* the slot.
	 * On plain-internal builds, the same gap would break the
	 * parent-pointer backtrack's O(1) slot recovery.
	 *
	 * Externals carry no metadata / offset; skip them.  Test
	 * skip-compressed FIRST: a SKIP_X flag carries its external child's
	 * low tag bits, so ft_node_external() would misclassify it.
	 *
	 * We do NOT touch @new_child's parent linkage here; callers manage
	 * that via their own ft_set_parent (or by direct meta->parent
	 * assignment) before calling us.  ft_set_parent_slot computes the
	 * offset relative to child_meta->parent, which callers have already
	 * pointed at @parent_nf (the node holding @parent_slot).
	 *
	 * Skip the update at the root slot (&ft->root): root nodes have
	 * no parent, and ft_set_parent_slot's offset computation assumes
	 * the slot lives inside a node-arena chunk.
	 */
	if (new_child && parent_slot != &ft->root) {
		struct cds_ft_metadata *child_meta = new_child_meta;

		if (!child_meta) {
			if (ft_node_skip_compressed(new_child))
				child_meta = cds_ft_item_to_metadata(
					(struct cds_ft_inode *)
						ft_skip_to_compressed(ft, new_child));
			else if (!ft_node_external(new_child))
				child_meta = cds_ft_item_to_metadata(
					ft_node_ptr(new_child));
		}
		if (child_meta && ft_parent_node(child_meta->parent_word))
			ft_set_parent_slot(child_meta,
				ft_parent_node(child_meta->parent_word),
				parent_slot);
	}

	if (parent_nf && ft_node_compressed(parent_nf)) {
		struct cds_ft_compressed_node *cn =
			ft_compressed_node_ptr(parent_nf);
		struct cds_ft_metadata *cn_meta =
			cds_ft_item_to_metadata(
				(struct cds_ft_inode *) cn);

		/* Consumed via FEATURE_FT_SKIP_COMPRESSED and FT_TP only. */
		(void) cn;
		(void) cn_meta;

#ifdef FEATURE_FT_SKIP_COMPRESSED
		{
			/*
			 * READ-YOUR-OWN-WRITES: a recorded re-parent of @cn is
			 * invisible to a raw derivation, and this op may be
			 * relocating @cn's parent in the same commit that
			 * refreshes the dual.  See ft_txn_parent_slot.
			 */
			struct cds_ft_inode_flag *skip_owner_nf = NULL;
			struct cds_ft_inode_flag **skip_slot =
				ft_txn_parent_slot_at(cn_meta, ft,
					rec ? rec->mtxn : NULL,
					&skip_owner_nf);
			if (skip_slot &&
			    ft_node_skip_compressed(*skip_slot)) {
				struct cds_ft_inode_flag *skip_new =
					ft_skip_compressed_flag(new_child,
						cn->len);

				if (rec && !ft_dual_home_is_private(ft, rec,
						cn_meta, skip_slot,
						skip_owner_nf))
					/* A COMPRESSED ROOT's dual slot IS
					 * &ft->root (ft_txn_parent_slot's root
					 * arm), so ask rather than assume. */
					/*
					 * ☠ THE DUAL'S OWNER IS DERIVED HERE,
					 * NOT DECLARED BY THE CALLER: it is the
					 * GRANDPARENT, reached through
					 * @cn_meta's back-pointer, and this
					 * frame cannot know whether the op
					 * acquired it.  So the held answer is
					 * the caller's -- @dual_owner_held --
					 * and its default is false, which
					 * records MW.
					 */
					ft_pub_rec_add(rec, skip_slot,
						ft_skip_compressed_flag(
							expected_old, cn->len),
						skip_new,
						skip_slot == &ft->root,
						skip_slot == &ft->root ||
						!skip_owner_nf ? NULL :
						ft_flag_to_metadata(ft,
							skip_owner_nf),
						dual_owner_held);
				else if (*skip_slot != skip_new)
					rcu_assign_pointer(*skip_slot, skip_new);
			}
		}
#endif
		/*
		 * Re-emit compressed_publish so consumers tracking
		 * cn -> child relationships pick up the new subtree
		 * attached under this compressed node.  The initial
		 * creation-time compressed_publish event has
		 * parent = NULL (the compressed node is not yet
		 * attached); here we report cn_meta->parent since the
		 * compressed node is already in the trie.
		 */
		FT_TP(compressed_publish,
			(const void *) ft_compressed_node_flag(cn),
			cn->len,
			cn->key_bytes,
			(const void *) new_child,
			(const void *) ft_parent_node(CMM_LOAD_SHARED(cn_meta->parent_word)));
	}
	FT_TP(publish_to_parent, (const void *) parent_nf,
		(const void *) parent_slot,
		(const void *) *parent_slot,
		(const void *) new_child);
	/*
	 * When parent_slot points at ft->root, emit root_publish so
	 * consumers can track the top of the trie through root
	 * rewrites that have no structural parent node.
	 */
	if (parent_slot == &ft->root)
		FT_TP(root_publish, (const void *) ft,
			(const void *) new_child);
	if (rec)
		/*
		 * The FORWARD edge's owner is the caller's own @slot_owner_nf
		 * declaration -- the node it says @parent_slot lives in -- so
		 * naming it IS the held answer here, and the record-time owner
		 * assert is what checks it.  Only the DUAL above needs a
		 * separate word, because only its owner is derived.
		 */
		ft_pub_rec_add(rec, parent_slot, expected_old, new_child,
			parent_slot == &ft->root,
			parent_slot == &ft->root || !slot_owner_nf ? NULL :
				ft_flag_to_metadata(ft, slot_owner_nf),
			parent_slot != &ft->root && slot_owner_nf != NULL);
	else if (*parent_slot != new_child)
		/*
		 * Direct (rec == NULL) publish.  The only two callers -- the
		 * in-place relocation else in ft_attach_node and graft's
		 * no-recompact else -- republish the value the slot already
		 * holds (Invariant-1: no live reader-visible bare publish
		 * remains; the SKIP_X dual above is likewise a no-op in
		 * lockstep).  Eliding the redundant store and its release fence
		 * is invisible to readers; the guard keeps the store correct
		 * should a value ever differ.  The recorded (rec) path captures
		 * the edge for the flip-txn instead of storing here.
		 */
		rcu_assign_pointer(*parent_slot, new_child);
}

/*
 * Publish, recovering @new_child's metadata from the slot value (the
 * back-pointer was wired up front) -- the original behaviour.
 */
static
void _ft_publish_to_parent(struct cds_ft *ft,
		struct cds_ft_inode_flag *parent_nf,
		struct cds_ft_inode_flag **parent_slot,
		struct cds_ft_inode_flag *new_child,
		struct cds_ft_inode_flag *expected_old,
		struct ft_pub_rec *rec,
		bool dual_owner_held)
{
	_ft_publish_to_parent_meta(ft, parent_nf, parent_slot, new_child,
		expected_old, NULL, NULL, rec, /*slot_owner_nf=*/ parent_nf,
		dual_owner_held);
}

/*
 * Direct publish (original behaviour): perform the stores immediately.
 * rec == NULL, so @expected_old is unused (the direct arm republishes the
 * value already present); pass the live slot value to satisfy the interface.
 */
static
void ft_publish_to_parent(struct cds_ft *ft,
		struct cds_ft_inode_flag *parent_nf,
		struct cds_ft_inode_flag **parent_slot,
		struct cds_ft_inode_flag *new_child)
{
	_ft_publish_to_parent(ft, parent_nf, parent_slot, new_child,
		*parent_slot, NULL, false);
}

/*
 * ft_publish_compressed: convert a compressed node flag to a skip
 * pointer if skip-compressed mode is enabled, the path length fits,
 * and the child has metadata (is not external).
 *
 * Call AFTER ft_set_parent has been done with the real compressed
 * flag (@cflag).  The returned value is what should be
 * published/stored in parent child slots.
 */
static
struct cds_ft_inode_flag *ft_publish_compressed(struct cds_ft *ft,
		struct cds_ft_compressed_node *cn,
		struct cds_ft_inode_flag *cflag)
{
	/*
	 * Emit creation-time compressed_publish so trace consumers
	 * learn the cn->child binding for every newly-allocated
	 * compressed node, regardless of which creation path built
	 * it (ft_build_compressed_node, compressed-split sfx/pfx/nb,
	 * graft-split suffix/prefix).  parent is NULL here: the cn
	 * is about to be returned to the caller for attachment;
	 * cn_meta->parent is still unset.  A subsequent
	 * ft_publish_to_parent / ft_node_set_nth on the slot that
	 * holds this cn fires tree_edge_set (with the cn as child),
	 * which -- paired with this event -- gives the consumer both
	 * ends: the parent->cn edge and the cn->child edge.
	 */
	FT_TP(compressed_publish,
		(const void *) ft_compressed_node_flag(cn),
		cn->len,
		cn->key_bytes,
		(const void *) cn->child,
		(const void *) NULL);
	if (ft_group_skip_compressed(ft->group) &&
	    cn->len <= FT_SKIP_LEN_MAX) {
		return ft_skip_compressed_flag(cn->child, cn->len);
	}
	return cflag;
}

/*
 * ft_set_parent: set the parent pointer in child's metadata.
 * Skips NULL children.
 *
 * External (leaf) nodes: sets cds_ft_node.prev (head of duplicate chain).
 *
 * For skip-compressed pointers: the skip pointer represents a
 * compressed node in the trie.  Set the compressed node's parent
 * (not the compressed node's child's parent, which is the
 * compressed node itself and was set at creation time).
 *
 * Skip-compressed must be checked before external: a skip pointer
 * whose child is external has low tag bits == 0, which would match
 * ft_node_external on the raw value.
 *
 * Write-side only (mutex-held).
 */
/*
 * ft_set_parent: set the parent pointer in child's metadata,
 * and optionally set skip_slot for skip-compressed children.
 *
 * @child_nf:  child node flag (may be skip-compressed, external, etc.)
 * @parent_nf: parent node flag to record.
 * @slot:      address of the slot in the parent that holds @child_nf.
 *             When @child_nf is skip-compressed and @slot is non-NULL,
 *             the compressed node's skip_slot is set to @slot.
 *             Pass NULL when the slot is unknown or irrelevant.
 */
static
void ft_set_parent(struct cds_ft *ft, struct cds_ft_inode_flag *child_nf,
		struct cds_ft_inode_flag *parent_nf,
		struct cds_ft_inode_flag **slot)
{
	/*
	 * A NULL @parent_nf is the root position (the publish goes into
	 * &ft->root): store the OWNING TRIE there rather than NULL, so the
	 * back-edge identifies its owner at depth 0 as it does everywhere else.
	 * @parent_nf itself stays as passed for the slot offset (a root has
	 * none) and for an external child, whose holder is always a real node.
	 */
	struct cds_ft_inode_flag *stored_parent = parent_nf ?
		parent_nf : ft_trie_parent(ft);

	if (!child_nf)
		return;
	/*
	 * A type-7 flip proxy is a transient slot VALUE (a one-commit insert
	 * or merge flip in progress), not a node: the REAL child's
	 * back-pointer is wired by the parking mutator itself.  No-op so the
	 * generic re-parent loops (recompact's child sweep, set_nth's
	 * post-store wiring) flow over a parked slot unharmed -- dispatching
	 * below would misread the proxy latch as internal-node metadata.
	 */
	if (caa_unlikely(ft_node_flip_proxy(child_nf)))
		return;
	FT_TP(set_parent, (const void *) child_nf, (const void *) parent_nf);
#ifdef FEATURE_FT_SKIP_COMPRESSED
	if (ft_node_skip_compressed(child_nf)) {
		struct cds_ft_compressed_node *cn =
			ft_skip_to_compressed(ft, child_nf);
		struct cds_ft_metadata *cn_meta =
			cds_ft_item_to_metadata(
				(struct cds_ft_inode *) cn);
		rcu_assign_pointer(cn_meta->parent_word, stored_parent);
		ft_set_parent_slot(cn_meta, parent_nf, slot);
		return;
	}
	if (ft_node_compressed(child_nf)) {
		/*
		 * Plain COMPRESSED form (no SKIP_X wrap): typically
		 * arises when ft_publish_compressed gates SKIP-X off
		 * for a non-spec EXT child.  Maintain the cn's
		 * parent_slot_offset just like the SKIP_X branch above so
		 * later ft_publish_to_parent / chain-merge calls can
		 * recover the slot in cn's parent via
		 * ft_get_parent_slot.
		 */
		struct cds_ft_compressed_node *cn =
			ft_compressed_node_ptr(child_nf);
		struct cds_ft_metadata *cn_meta =
			cds_ft_item_to_metadata(
				(struct cds_ft_inode *) cn);
		rcu_assign_pointer(cn_meta->parent_word, stored_parent);
		ft_set_parent_slot(cn_meta, parent_nf, slot);
		return;
	}
#endif
	if (ft_node_external(child_nf)) {
		/*
		 * Ordered list on: the head carries its cell in prev; record the
		 * parent into cell->parent (fresh head: cell pre-wired at insert;
		 * existing head re-parent: cell already present).  List off / non-cell:
		 * the parent is the head's prev directly.  rcu_assign either way:
		 * ft_set_parent re-parents live heads on the restructure path.
		 */
		if (ft->ordered_list)
			ft_ord_cell_set_parent((struct cds_ft_node *) child_nf,
				parent_nf);
		else
			rcu_assign_pointer(
				((struct cds_ft_node *) child_nf)->prev,
				parent_nf);
		return;
	}
	{
		/*
		 * Plain internal child: record its parent AND its slot offset
		 * within the parent, so the parent-pointer backtrack can recover
		 * the slot in O(1) (ft_get_parent_slot) without re-descending.
		 * ft_set_parent_slot reads meta->parent, so set it first.
		 */
		struct cds_ft_metadata *meta =
			cds_ft_item_to_metadata(ft_node_ptr(child_nf));

		/*
		 * Publish the up-walk key byte BEFORE the parent pointer.  A node
		 * re-homed from a COMPRESSED parent (which skips incoming_byte,
		 * leaving it 0) to an INTERNAL parent gets its real branch byte
		 * here.  If we published meta->parent first (as ft_set_parent_slot
		 * needs, to compute the offset) a concurrent up-walk that follows
		 * the new parent would read the still-stale 0 byte and reconstruct
		 * a key with a hole at this level.  Pre-store it under the explicit
		 * @parent_nf and let the rcu_assign release order it; ft_set_parent_
		 * slot below recomputes the same byte (idempotent) plus the offset.
		 */
		if (slot && parent_nf && !ft_node_compressed(parent_nf)
#ifdef FEATURE_FT_SKIP_COMPRESSED
				&& !ft_node_skip_compressed(parent_nf)
#endif
		   )
			meta->incoming_byte = ft_slot_to_byte(
				&ft_types[ft_node_type(parent_nf)],
				ft_node_ptr(parent_nf), slot);
		rcu_assign_pointer(meta->parent_word, stored_parent);
		ft_set_parent_slot(meta, parent_nf, slot);
	}
}

/*
 * Return codes for compressed node traversal helpers.
 * Used to tell callers which loop control action to take.
 */
enum ft_descent_action {
	FT_DESCENT_CONTINUE,		/* Continue loop iteration. */
	FT_DESCENT_BREAK,		/* Break from loop. */
	FT_DESCENT_END,		/* Jump to function end (status set). */
	FT_DESCENT_GOING_UP,		/* Jump to going_up backtracking. */
	FT_DESCENT_DESCEND_CHILDREN,	/* Jump to descend_children. */
	FT_DESCENT_FOUND_MINMAX,	/* Jump to found_minmax label. */
};

/*
 * Compare @cmp key bytes starting at @key against the compressed
 * node's path.  Returns the number of matching bytes.  A return
 * value == @cmp means full match; < @cmp means divergence at that
 * position.
 */
static inline
unsigned int ft_match_compressed_key(const uint8_t *key,
		const struct cds_ft_compressed_node *cn,
		unsigned int cmp)
{
	unsigned int pos;

	if (ft_key_cmp_ordinals(key, cn->key_bytes, cmp, cmp, false, &pos) != 0)
		return pos;
	return cmp;
}

/*
 * Fill ordinal_key for every level spanned by a compressed node.  Used
 * by read-side descent loops (lookup_nth, minmax, etc.) to record the
 * ordinal key bytes through compressed nodes; the going-up backtrack
 * recovers per-level nodes from the live parent chain, not a path array.
 */
static inline
void ft_fill_compressed_path(struct cds_ft_compressed_node *cn,
		uint8_t *ordinal_key, int base)
{
	int j;

	for (j = 0; j < cn->len; j++)
		ordinal_key[base + j] = cn->key_bytes[j];
}

static
bool valid_external_node(struct cds_ft_node *node)
{
	return node != NULL && ft_node_external((struct cds_ft_inode_flag *) node);
}

/*
 * Return the metadata of the root node.
 *
 * ft->root always points to an arena-allocated internal node, even
 * when the trie is empty (nr_child == 0).  The node itself may be
 * replaced by graft or graft-swap, but the invariant on the slot
 * is maintained across all operations.  Its metadata holds:
 *   - nr_child:       number of children in the root node.
 *   - external_nodes: list of NIL-key (key_len == 0) entries.
 *
 * The root is a regular internal node whose metadata is accessed the
 * same way as any other node's.  Its metadata carries the NIL-key
 * entries, so transplanting a root node between tries is a single
 * pointer swap with no metadata relocation.
 *
 * This function is only meant to be used from update functions, _not_
 * safe for use by read-side.
 */
static inline
struct cds_ft_metadata *ft_root_metadata(const struct cds_ft *ft)
{
	/*
	 * Resolve a peer's parked flip proxy on the root slot (Phase 4.3, a
	 * root recompact mid-commit): ft_node_ptr on the raw proxy would mask
	 * the tag and hand back a RECORD address as a node.
	 */
	return cds_ft_item_to_metadata(ft_node_ptr(
		ft_resolve_flip_proxy(rcu_dereference(ft->root))));
}

static
bool valid_key_len(struct cds_ft *ft, size_t key_len)
{
	size_t max_key_len = ft->group->max_key_len;

	assert(max_key_len != CDS_FT_MAX_LEN_UNLIMITED);
	if (key_len == CDS_FT_LEN_ERROR || key_len > max_key_len)
		return false;
	return true;
}

static
struct cds_ft_inode *alloc_cds_ft_node(struct cds_ft *ft,
		const struct cds_ft_type *ft_type,
		struct cds_ft_metadata **_metadata)
{
	struct cds_ft_metadata *metadata;
	void *p;

	metadata = cds_ft_alloc_item(ft, ft_type->order, ft_type->bitmap);
	if (!metadata) {
		return NULL;
	}
	p = cds_ft_metadata_to_item(metadata);
	FT_TP(item_alloc, (const void *) p, 0, ft_type->order);
	/*
	 * Popcount node data[] starts with a presence bitmap, followed
	 * by the pointer table.  The allocator returns zeroed memory,
	 * which is the initial "no children" state (bitmap = 0, so all
	 * lookups return NULL; nr_child derived from popcount returns 0).
	 */
	if (ft_debug_counters()) {
		uatomic_inc(&ft->group->nr_nodes_allocated);
		uatomic_inc(&ft->group->nr_internal_alloc);
	}
	*_metadata = metadata;
	return p;
}

static
void free_cds_ft_node(struct cds_ft *ft, struct cds_ft_inode *node)
{
	struct cds_ft_metadata *metadata = cds_ft_item_to_metadata(node);

	FT_TP(item_free, (const void *) node, 0);
	FT_TP(item_retire, (const void *) node, __builtin_return_address(0));

#ifdef FT_DEBUG_TOMBSTONE_AUDIT
	/*
	 * Freeze-on-free guard (doc §4.B, STEP 3): every PUBLISHED node retired
	 * through this path must carry its one-way LIVE->DEAD tombstone, set
	 * BEFORE the unlink commit that detached it.  Abandoned fresh
	 * (never-reader-visible) nodes use free_cds_ft_node_unpublished and do
	 * not reach here.  Build with -DFT_DEBUG_TOMBSTONE_AUDIT to enforce that
	 * no retire site is added without a mark (a no-op under one writer).
	 */
	assert(ft_meta_tombstone(metadata));
#endif
	cds_ft_free_item(ft, metadata);
	if (ft_debug_counters() && node) {
		uatomic_inc(&ft->group->nr_nodes_freed);
		uatomic_inc(&ft->group->nr_internal_freed);
	}
}

/*
 * Immediate-free variant for internal nodes that never escape the
 * writer's stack (e.g., nodes built by an attach/split/recompact
 * helper but freed by an -ENOMEM error path before publication).
 * See cds_ft_free_item_unpublished for the safety contract.
 */
static
void free_cds_ft_node_unpublished(struct cds_ft *ft, struct cds_ft_inode *node)
{
	struct cds_ft_metadata *metadata = cds_ft_item_to_metadata(node);

	FT_TP(item_free, (const void *) node, 2);
	cds_ft_free_item_unpublished(ft, metadata);
	if (ft_debug_counters() && node) {
		uatomic_inc(&ft->group->nr_nodes_freed);
		uatomic_inc(&ft->group->nr_internal_freed);
	}
}

/*
 * Compute the arena allocation order for a compressed node with
 * @path_len key bytes.  The compressed node layout is:
 *   [child pointer] [len byte] [key_bytes...]
 */
static
unsigned int ft_compressed_order(uint8_t path_len)
{
	size_t size = offsetof(struct cds_ft_compressed_node, key_bytes) + path_len;
	int order = urcu_get_count_order_ulong(size);

	if (order < 4)
		order = 4;	/* Minimum arena order. */
	return (unsigned int) order;
}

static
struct cds_ft_compressed_node *alloc_compressed_node(struct cds_ft *ft,
		uint8_t path_len,
		struct cds_ft_metadata **_metadata)
{
	struct cds_ft_metadata *metadata;
	void *p;
	unsigned int order = ft_compressed_order(path_len);

	metadata = cds_ft_alloc_compressed_item(ft, order);
	if (!metadata)
		return NULL;
	p = cds_ft_metadata_to_item(metadata);
	FT_TP(item_alloc, (const void *) p, 1, path_len);
	if (ft_debug_counters()) {
		uatomic_inc(&ft->group->nr_nodes_allocated);
		uatomic_inc(&ft->group->nr_compressed_alloc);
	}
	*_metadata = metadata;
	return p;
}

static
void free_compressed_node(struct cds_ft *ft,
		struct cds_ft_compressed_node *node)
{
	struct cds_ft_metadata *metadata =
		cds_ft_item_to_metadata((struct cds_ft_inode *) node);

	FT_TP(item_free, (const void *) node, 1);

#ifdef FT_DEBUG_TOMBSTONE_AUDIT
	/* See free_cds_ft_node: freeze-on-free guard (doc §4.B). */
	assert(ft_meta_tombstone(metadata));
#endif
	FT_TP(compressed_free, (const void *) ft_compressed_node_flag(node));
	cds_ft_free_item(ft, metadata);
	if (ft_debug_counters() && node) {
		uatomic_inc(&ft->group->nr_nodes_freed);
		uatomic_inc(&ft->group->nr_compressed_freed);
	}
}

/*
 * Immediate-free variant for compressed nodes that never escape the
 * writer's stack (e.g., -ENOMEM error paths in build/split helpers).
 * See cds_ft_free_item_unpublished for the safety contract.
 */
static
void free_compressed_node_unpublished(struct cds_ft *ft,
		struct cds_ft_compressed_node *node)
{
	FT_TP(item_free, (const void *) node, 3);
	struct cds_ft_metadata *metadata =
		cds_ft_item_to_metadata((struct cds_ft_inode *) node);

	FT_TP(compressed_free, (const void *) ft_compressed_node_flag(node));
	cds_ft_free_item_unpublished(ft, metadata);
	if (ft_debug_counters() && node) {
		uatomic_inc(&ft->group->nr_nodes_freed);
		uatomic_inc(&ft->group->nr_compressed_freed);
	}
}

#define __FT_ALIGN_MASK(v, mask)	(((v) + (mask)) & ~(mask))
#define FT_ALIGN(v, align)		__FT_ALIGN_MASK(v, (typeof(v)) (align) - 1)
#define __FT_FLOOR_MASK(v, mask)	((v) & ~(mask))
#define FT_FLOOR(v, align)		__FT_FLOOR_MASK(v, (typeof(v)) (align) - 1)

/*
 * Push a node and its depth onto the snapshot stack, maintaining the
 * parallel snapshot_depth[] array alongside snapshot[].
 */
#define ft_snapshot_push(snap, snap_depth, nr, node_flag, depth)	\
	do {								\
		(snap_depth)[(nr)] = (depth);				\
		(snap)[(nr)++] = (node_flag);				\
	} while (0)


/* Human-readable name for a cds_ft_status code (diagnostics, tests). */
const char *cds_ft_status_to_string(enum cds_ft_status status)
{
	switch (status) {
	/* Success return codes (>= 0). */
	case CDS_FT_STATUS_OK:
		return "Operation completed successfully";
	case CDS_FT_STATUS_NOT_FOUND:
		return "No node found";
	case CDS_FT_STATUS_DUPLICATE_FOUND:
		return "Duplicate node exists";
	case CDS_FT_STATUS_INTERNAL_MATCH:
		return "Match ends at an internal node";

	/* Error return codes (< 0). */
	case CDS_FT_STATUS_INVALID_ARGUMENT_ERROR:
		return "Invalid argument";
	case CDS_FT_STATUS_MEMORY_ERROR:
		return "Memory allocation failure";
	case CDS_FT_STATUS_OVERFLOW_ERROR:
		return "Buffer too small for key length";
	case CDS_FT_STATUS_BUSY_ERROR:
		return "Resource busy";
	case CDS_FT_STATUS_POPULATED_ERROR:
		return "Destination already populated";
	case CDS_FT_STATUS_INTEGRITY_ERROR:
		return "Integrity verification failure";
	case CDS_FT_STATUS_NOT_SUPPORTED:
		return "Feature not compiled in";

	default:
		return "Unknown status value";
	}
}

/* Emit @level indentation tabs to @out (shared by the show + stats renderers). */
static
void print_indent(FILE *out, int level)
{
	int i;

	for (i = 0; i < level; i++)
		fprintf(out, "	");
}
