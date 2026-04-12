// SPDX-FileCopyrightText: 2012-2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-only

/*
 * src/fractal-trie.c
 *
 * Userspace RCU library - Fractal Trie
 */

#define _LGPL_SOURCE
#include <stdint.h>
#include <errno.h>
#include <limits.h>
#include <string.h>
#include <assert.h>
#include <endian.h>
#include <stdbool.h>
#include <urcu/fractal-trie.h>
#include <urcu/compiler.h>
#include <urcu/arch.h>
#include <urcu-pointer.h>
#include <urcu/uatomic.h>
#include "urcu-utils.h"

#include "fractal-trie-internal.h"

#ifdef FT_DELAY_INJECT
#include <unistd.h>
#include <stdlib.h>
enum ft_delay_mode ft_delay_mode = FT_DELAY_NONE;
unsigned int ft_delay_us = 1;

static void __attribute__((constructor))
ft_delay_init(void)
{
	const char *mode = getenv("FT_DELAY_MODE");
	const char *us = getenv("FT_DELAY_US");

	if (mode) {
		if (!strcmp(mode, "writer"))
			ft_delay_mode = FT_DELAY_WRITER;
		else if (!strcmp(mode, "reader"))
			ft_delay_mode = FT_DELAY_READER;
		else if (!strcmp(mode, "both"))
			ft_delay_mode = FT_DELAY_BOTH;
		else if (!strcmp(mode, "random"))
			ft_delay_mode = FT_DELAY_RANDOM;
	}
	if (us)
		ft_delay_us = (unsigned int) atoi(us);
}
#endif
#include "bitmap.h"

#ifndef abs_int
#define abs_int(a)	((int) (a) > 0 ? (int) (a) : -((int) (a)))
#endif

#define CDS_FT_LEN_ERROR		SIZE_MAX

struct cds_ft_attr {
	size_t key_len;
	size_t max_key_len;
	struct cds_ft_key_map key_map;
	unsigned int flags;
};

enum cds_ft_type_class {
	FT_LINEAR = 0,		/* Small linear: bytewise scan */
	FT_LINEAR_WIDE = 1,	/* Large linear: SWAR or SIMD scan */
	FT_POOL = 2,		/* Pool: 1D/2D subnode dispatch */
	FT_PIGEON = 3,		/* Pigeon: direct indexed */
	/* Leaf nodes are implicit from their height in the tree */
	FT_NR_TYPES,

	FT_NULL,	/* not an encoded type, but keeps code regular */
};

#define ft_type_is_linear(tc)	((tc) == FT_LINEAR || (tc) == FT_LINEAR_WIDE)

/*
 * FT_WIDE_LINEAR_THRESHOLD: max_linear_child threshold for
 * FT_LINEAR_WIDE.  Used to assign type_class in ft_types[].
 */
#ifndef FT_SIMD_LINEAR_THRESHOLD
#define FT_SIMD_LINEAR_THRESHOLD	7
#endif
#ifndef FT_SWAR_LINEAR_THRESHOLD
#define FT_SWAR_LINEAR_THRESHOLD	14
#endif
#if defined(__SSE2__)
#define FT_WIDE_LINEAR_THRESHOLD	FT_SIMD_LINEAR_THRESHOLD
#else
#define FT_WIDE_LINEAR_THRESHOLD	FT_SWAR_LINEAR_THRESHOLD
#endif

/*
 * Macro to select FT_LINEAR or FT_LINEAR_WIDE based on the
 * type's max_linear_child at compile time.
 */
#define FT_LINEAR_CLASS(mlc) \
	((mlc) >= FT_WIDE_LINEAR_THRESHOLD ? FT_LINEAR_WIDE : FT_LINEAR)

struct cds_ft_type {
	enum cds_ft_type_class type_class;
	uint16_t min_child;		/* minimum number of children: 1 to 256 */
	uint16_t max_child;		/* maximum number of children: 1 to 256 */
	uint16_t max_linear_child;	/* per-pool max nr. children: 1 to 256 */
	uint16_t order;			/* node size is (1 << order), in bytes */
	uint16_t nr_pool_order;		/* number of pools */
	uint16_t pool_size_order;	/* pool size */
	bool bitmap;			/* allocate bitmap */
};

/*
 * Iteration on the array to find the right node size for the number of
 * children stops when it reaches .max_child == 256 (this is the largest
 * possible node size, which contains 256 children).
 * The min_child overlaps with the previous max_child to provide an
 * hysteresis loop to reallocation for patterns of cyclic add/removal
 * within the same node.
 * The node the index within the following arrays is represented on 3
 * bits. It identifies the node type, min/max number of children, and
 * the size order.
 * The max_child values for the FT_POOL below result from
 * statistical approximation: over million populations, the max_child
 * covers between 97% and 99% of the populations generated. Therefore, a
 * fallback should exist to cover the rare extreme population unbalance
 * cases, but it will not have a major impact on speed nor space
 * consumption, since those are rare cases.
 */

/*
 * The smallest allocation order we can use is 4:
 * - 1 bit is reserved for internal vs external flag,
 * - 3 bits are reserved to encode the node type.
 */

#if (CAA_BITS_PER_LONG < 64)

/* 32-bit pointers */
enum {
	ft_type_0_max_child = 3,
	ft_type_1_max_child = 6,
	ft_type_2_max_child = 12,
	ft_type_3_max_child = 25,
	ft_type_4_max_child = 48,
	ft_type_5_max_child = 92,
	ft_type_6_max_child = 256,
	ft_type_7_max_child = 0,	/* NULL */
};

enum {
	ft_type_0_max_linear_child = 3,
	ft_type_1_max_linear_child = 6,
	ft_type_2_max_linear_child = 12,
	ft_type_3_max_linear_child = 25,
	ft_type_4_max_linear_child = 24,
	ft_type_5_max_linear_child = 23,
};

enum {
	ft_type_4_nr_pool_order = 1,
	ft_type_5_nr_pool_order = 2,
};

const struct cds_ft_type ft_types[] = {
	[0] = { .type_class = FT_LINEAR_CLASS(ft_type_0_max_linear_child), .min_child = 1, .max_child = ft_type_0_max_child, .max_linear_child = ft_type_0_max_linear_child, .order = 4, .bitmap = FT_NO_BITMAP },
	[1] = { .type_class = FT_LINEAR_CLASS(ft_type_1_max_linear_child), .min_child = 3, .max_child = ft_type_1_max_child, .max_linear_child = ft_type_1_max_linear_child, .order = 5, .bitmap = FT_NO_BITMAP },
	[2] = { .type_class = FT_LINEAR_CLASS(ft_type_2_max_linear_child), .min_child = 4, .max_child = ft_type_2_max_child, .max_linear_child = ft_type_2_max_linear_child, .order = 6, .bitmap = FT_NO_BITMAP },
	[3] = { .type_class = FT_LINEAR_CLASS(ft_type_3_max_linear_child), .min_child = 10, .max_child = ft_type_3_max_child, .max_linear_child = ft_type_3_max_linear_child, .order = 7, .bitmap = FT_NO_BITMAP },

	/* Pools may fill sooner than max_child */
	/* This pool is hardcoded at index 4. See ft_node_ptr(). */
	[FT_POOL_IDX_A] = { .type_class = FT_POOL, .min_child = 20, .max_child = ft_type_4_max_child, .max_linear_child = ft_type_4_max_linear_child, .order = 8, .nr_pool_order = ft_type_4_nr_pool_order, .pool_size_order = 7, .bitmap = FT_NO_BITMAP },
	/* This pool is hardcoded at index 5. See ft_node_ptr(). */
	[FT_POOL_IDX_B] = { .type_class = FT_POOL, .min_child = 45, .max_child = ft_type_5_max_child, .max_linear_child = ft_type_5_max_linear_child, .order = 9, .nr_pool_order = ft_type_5_nr_pool_order, .pool_size_order = 7, .bitmap = FT_BITMAP },

	/*
	 * Upon node removal below min_child, if child pool is filled
	 * beyond capacity, we roll back to pigeon.
	 */
	[6] = { .type_class = FT_PIGEON, .min_child = 83, .max_child = ft_type_6_max_child, .order = 10, .bitmap = FT_BITMAP },

	[7] = { .type_class = FT_NULL, .min_child = 0, .max_child = ft_type_7_max_child, .bitmap = FT_NO_BITMAP },
};
#else /* !(CAA_BITS_PER_LONG < 64) */
/* 64-bit pointers */
enum {
	ft_type_0_max_child = 1,
	ft_type_1_max_child = 3,
	ft_type_2_max_child = 7,
	ft_type_3_max_child = 14,
	ft_type_4_max_child = 28,
	ft_type_5_max_child = 54,
	ft_type_6_max_child = 104,
	ft_type_7_max_child = 256,
	ft_type_8_max_child = 256,
};

enum {
	ft_type_0_max_linear_child = 1,
	ft_type_1_max_linear_child = 3,
	ft_type_2_max_linear_child = 7,
	ft_type_3_max_linear_child = 14,
	ft_type_4_max_linear_child = 28,
	ft_type_5_max_linear_child = 27,
	ft_type_6_max_linear_child = 26,
};

enum {
	ft_type_5_nr_pool_order = 1,
	ft_type_6_nr_pool_order = 2,
};

const struct cds_ft_type ft_types[] = {
	[0] = { .type_class = FT_LINEAR_CLASS(ft_type_0_max_linear_child), .min_child = 1, .max_child = ft_type_0_max_child, .max_linear_child = ft_type_0_max_linear_child, .order = 4, .bitmap = FT_NO_BITMAP },
	[1] = { .type_class = FT_LINEAR_CLASS(ft_type_1_max_linear_child), .min_child = 1, .max_child = ft_type_1_max_child, .max_linear_child = ft_type_1_max_linear_child, .order = 5, .bitmap = FT_NO_BITMAP },
	[2] = { .type_class = FT_LINEAR_CLASS(ft_type_2_max_linear_child), .min_child = 3, .max_child = ft_type_2_max_child, .max_linear_child = ft_type_2_max_linear_child, .order = 6, .bitmap = FT_NO_BITMAP },
	[3] = { .type_class = FT_LINEAR_CLASS(ft_type_3_max_linear_child), .min_child = 5, .max_child = ft_type_3_max_child, .max_linear_child = ft_type_3_max_linear_child, .order = 7, .bitmap = FT_NO_BITMAP },
	[4] = { .type_class = FT_LINEAR_CLASS(ft_type_4_max_linear_child), .min_child = 10, .max_child = ft_type_4_max_child, .max_linear_child = ft_type_4_max_linear_child, .order = 8, .bitmap = FT_NO_BITMAP },

	/* Pools may fill sooner than max_child. */
	/* This pool is hardcoded at index 5. See ft_node_ptr(). */
	[FT_POOL_IDX_A] = { .type_class = FT_POOL, .min_child = 22, .max_child = ft_type_5_max_child, .max_linear_child = ft_type_5_max_linear_child, .order = 9, .nr_pool_order = ft_type_5_nr_pool_order, .pool_size_order = 8, .bitmap = FT_NO_BITMAP },
	/* This pool is hardcoded at index 6. See ft_node_ptr(). */
	[FT_POOL_IDX_B] = { .type_class = FT_POOL, .min_child = 51, .max_child = ft_type_6_max_child, .max_linear_child = ft_type_6_max_linear_child, .order = 10, .nr_pool_order = ft_type_6_nr_pool_order, .pool_size_order = 8, .bitmap = FT_BITMAP },

	/*
	 * Upon node removal below min_child, if child pool is filled
	 * beyond capacity, we roll back to pigeon.
	 */
	[7] = { .type_class = FT_PIGEON, .min_child = 95, .max_child = ft_type_7_max_child, .order = 11, .bitmap = FT_BITMAP },

	[8] = { .type_class = FT_NULL, .min_child = 0, .max_child = ft_type_8_max_child, .bitmap = FT_NO_BITMAP },
};
#endif /* !(BITS_PER_LONG < 64) */

/*
 * The cds_ft_inode contains the compressed node data needed for
 * the read-side traversal. Because the actual layout depends on the
 * node's type_class (Linear, Pool, or Pigeon), the struct uses a single
 * pointer-aligned byte array. The allocator sizes this array dynamically
 * based on the type's order (1 << type->order).
 *
 * Crucially, the allocator guarantees that each node is naturally aligned
 * to its exact size boundary. For example, a node with order 6 (64 bytes)
 * is guaranteed to be aligned on a 64-byte boundary in memory. This ensures
 * optimal cache-line alignment, prevents false sharing, and guarantees that
 * a node never straddles a memory page boundary. This ensures that accessing
 * a single node hits at most one TLB entry, minimizing read-side latency.
 *
 * Memory Layouts by Type Class:
 *
 * 1. FT_LINEAR:
 * - data[0]: nr_child (number of populated slots in this node).
 * - data[1 .. max_linear_child]: uint8_t keys (child values).
 * - [Padding] to reach the next pointer-aligned (8-byte/4-byte) boundary.
 * - Array of (struct cds_ft_inode_flag *) pointers.
 *
 * 2. FT_POOL:
 * - An array of (1 << nr_pool_order) linear sub-nodes.
 * - Each sub-node has the exact same internal layout as FT_LINEAR.
 * - Each sub-node is sized and padded to exactly (1 << pool_size_order) bytes,
 *   allowing O(1) stride access via index << pool_size_order.
 * - The 'index' of the target sub-node is derived using a value population
 *   bit-select strategy. Specific bits of the search key (encoded in the
 *   pointer flag) are evaluated to optimally distribute children and
 *   minimize collisions within the pool.
 *
 * 3. FT_PIGEON:
 * - A direct, flat array of up to 256 (struct cds_ft_inode_flag *) pointers.
 * - No nr_child or key arrays are stored inside the node (the key is
 *   implicit from the pointer's array index).
 * - Because 'data' is explicitly pointer-aligned, it can be safely cast
 *   directly to (struct cds_ft_inode_flag **).
 *
 * Note: For all configurations, the true total number of children is
 * strictly maintained in the out-of-line metadata (struct cds_ft_metadata).
 *
 * Note on array sizing: The size of this array is the maximum possible
 * node size (Pigeon: 256 pointers) because C99 don't allow Flexible
 * Array Members as first fields within a structure.
 */

struct cds_ft_inode {
	uint8_t data[FT_ENTRY_PER_NODE * sizeof(struct cds_ft_inode_flag *)]
		__attribute__((__aligned__(sizeof(struct cds_ft_inode_flag *))));
};

static inline __attribute__((unused))
void static_array_size_check(void)
{
	CAA_BUILD_BUG_ON(CAA_ARRAY_SIZE(ft_types) < FT_TYPE_MAX_NR);
}

/*
 * Iterate through duplicates returned by cds_ft_lookup*()
 * Receives a struct cds_ft_node * as parameter, which is used as start
 * of duplicate list and loop cursor.
 */
#define cds_ft_for_each_duplicate(pos)				\
       for (; (pos) != NULL; (pos) = (pos)->next)

enum ft_recompact {
	FT_RECOMPACT_ADD_SAME,
	FT_RECOMPACT_ADD_NEXT,
	FT_RECOMPACT_DEL,
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
	size_t path_len;		/* Populated path_node array length. */
	size_t key_len;			/* Key length of the current node. */
	size_t prefix_len;		/* Key prefix length. */
	enum cds_ft_status status;	/* Iteration status. */
	enum cds_ft_iter_path_mode path_mode;	/* Path caching mode. */
	bool path_valid;		/* Whether this iterator has a valid path. */

#ifdef URCU_FRACTAL_TRIE_DEBUG_PATH
	struct urcu_gp_poll_state gp_state;	/* GP snapshot when path was populated. */
	bool gp_state_valid;			/* Whether gp_state holds a meaningful value. */
#endif

	/*
	 * Keep a copy of each rcu_dereferenced nodes encountered within
	 * traversal along with their associated keys, thus forming a
	 * path for backtracking.
	 *
	 * Flexible array member for trailing data.
	 * Explicitly aligned to safely cast the start of the buffer to a pointer array.
	 * Layout: [ path_node array ] followed immediately by [ key array ]
	 */
	char data[] __attribute__((__aligned__(sizeof(struct cds_ft_inode_flag *))));
};

/* Start of the data buffer to path_node pointer array. */
#define iter_path_node(iter) \
	((struct cds_ft_inode_flag **)((iter)->data))

/* Start of the uint8_t key array. */
#define iter_key(iter) \
	((uint8_t *)((iter)->data + ((iter)->ft->group->max_tree_depth * sizeof(struct cds_ft_inode_flag *))))

/*
 * Debug helpers for detecting stale cached iterator paths.
 *
 * Three entry-point roles mirror the rculfhash pattern:
 *
 *  iter_debug_path_snapshot() — unconditionally captures a fresh
 *      grace-period poll state.  Called at the entry of every
 *      fresh-population operation (lookup, longest-match lookup, and
 *      the slow-path / early-exit branches of inequality lookup).
 *      Because it always overwrites the snapshot, an iterator that is
 *      reused across RCU read-side critical sections gets a current
 *      baseline, preventing false positives on the next check.
 *
 *  iter_debug_path_check() — polls the existing snapshot.  Called at
 *      continuation entry points that consume a previously populated
 *      cached path (inequality fast-path, replace, remove).  If a full
 *      grace period has elapsed since the snapshot was taken, the RCU
 *      read-side lock must have been dropped and the cached pointers
 *      may reference freed memory — the check aborts.
 *
 *  iter_debug_path_update() — invalidates the snapshot when the path
 *      becomes invalid (node not found / end of traversal).  It never
 *      captures a new snapshot; the one taken at the operation's entry
 *      point persists as long as the path remains valid, giving a
 *      tighter detection window.
 *
 *  iter_debug_path_clear() — unconditionally resets the snapshot
 *      validity.  Used by iter_auto_invalidate_path() and by
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

	if (iter->path_mode != CDS_FT_ITER_PATH_CACHED)
		return;
	if (!iter->path_valid)
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
	if (!iter->path_valid)
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
 * Discard the cached path if the iterator is in uncached mode.
 * Called at the end of each public iterator-based operation.
 * Preserves iter->node so the caller can read the result.
 */
static inline
void iter_auto_invalidate_path(struct cds_ft_iter *iter)
{
	if (iter->path_mode == CDS_FT_ITER_PATH_UNCACHED) {
		iter->path_valid = false;
		iter->path_len = 0;
		iter_debug_path_clear(iter);
	}
}

#define BITMASK_2(a, b)					\
	{						\
		.mask = (1U << (a) | 1U << (b)),	\
		.bit = {				\
			[0] = (a),			\
			[1] = (b),			\
		},					\
	}

struct combination_table {
	uint8_t mask;
	uint8_t bit[2];
};

/*
 * Combination table C(n=8,r=2) = 28.
 */
static
const struct combination_table C_n8_r2[] = {
	BITMASK_2(0, 1), BITMASK_2(0, 2), BITMASK_2(0, 3), BITMASK_2(0, 4), BITMASK_2(0, 5), BITMASK_2(0, 6), BITMASK_2(0, 7),
	BITMASK_2(1, 2), BITMASK_2(1, 3), BITMASK_2(1, 4), BITMASK_2(1, 5), BITMASK_2(1, 6), BITMASK_2(1, 7),
	BITMASK_2(2, 3), BITMASK_2(2, 4), BITMASK_2(2, 5), BITMASK_2(2, 6), BITMASK_2(2, 7),
	BITMASK_2(3, 4), BITMASK_2(3, 5), BITMASK_2(3, 6), BITMASK_2(3, 7),
	BITMASK_2(4, 5), BITMASK_2(4, 6), BITMASK_2(4, 7),
	BITMASK_2(5, 6), BITMASK_2(5, 7),
	BITMASK_2(6, 7)
};

/* return an index within the combination table C(n=8,r) associated to mask. */
static inline
unsigned int mask_to_index_C_n8_r2(uint8_t mask)
{
	unsigned int i;

	assert(__builtin_popcount(mask) == 2);
	for (i = 0; i < CAA_ARRAY_SIZE(C_n8_r2); i++)
		if (C_n8_r2[i].mask == mask)
			return i;
	abort();
}

static inline
void index_to_bits_C_n8_r2(unsigned int index, uint8_t *bits)
{
	assert(index < CAA_ARRAY_SIZE(C_n8_r2));
	bits[0] = C_n8_r2[index].bit[0];
	bits[1] = C_n8_r2[index].bit[1];
}

/*
 * Keep only the requested 2 bits from value, and move them to LSB to
 * form a subclass index.
 */
static inline
unsigned int value_and_bits_to_subclass_index(uint8_t value, const uint8_t *bits)
{

	return (((value >> bits[0]) & 0x1) << 1) | ((value >> bits[1]) & 0x1);
}

static
unsigned long ft_node_pool_1d_bitsel(struct cds_ft_inode_flag *node)
{
	return ((unsigned long) node & FT_POOL_1D_MASK) >> (FT_TYPE_BITS + FT_INTERNAL_BITS);
}

static
void ft_node_pool_2d_index(struct cds_ft_inode_flag *node, unsigned int *index)
{
	*index = ((unsigned long) node & FT_POOL_2D_MASK) >> (FT_TYPE_BITS + FT_INTERNAL_BITS);
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

	assert(key_len <= 8);
	if (key_len > 8)
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

	assert(key_len <= 8);
	if (key_len > 8)
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

	assert(key_len <= 4);
	if (key_len > 4)
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

	assert(key_len <= 4);
	if (key_len > 4)
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

	assert(key_len <= 8);
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

	assert(key_len <= 8);
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

	assert(key_len <= 4);
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

	assert(key_len <= 4);
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
static inline void ft_key_to_ordinals(uint8_t *dst, const uint8_t *key,
		size_t len, const struct cds_ft_key_map *km)
{
	size_t i;

	if (caa_likely(km->identity)) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstringop-overflow"
#pragma GCC diagnostic ignored "-Wrestrict"
		memcpy(dst, key, len);
#pragma GCC diagnostic pop
		return;
	}
	for (i = 0; i < len; i++)
		dst[i] = km->key_to_ordinal[key[i]];
}

/*
 * Bulk ordinal-to-key conversion.  Converts @len ordinals in @src
 * back to external key bytes in @dst.  Identity maps short-circuit
 * to memcpy.
 */
static inline void ft_ordinals_to_key(uint8_t *dst, const uint8_t *ordinals,
		size_t len, const struct cds_ft_key_map *km)
{
	size_t i;

	if (caa_likely(km->identity)) {
		memcpy(dst, ordinals, len);
		return;
	}
	for (i = 0; i < len; i++)
		dst[i] = km->ordinal_to_key[ordinals[i]];
}

/*
 * Byte-swap an unsigned long for lexicographic word comparison on
 * little-endian.  On big-endian this is a no-op: natural word order
 * already matches memory (lexicographic) order.
 */
static inline unsigned long ft_bswap_long(unsigned long v)
{
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
#if __SIZEOF_LONG__ == 8
	return __builtin_bswap64(v);
#else
	return __builtin_bswap32(v);
#endif
#else
	return v;
#endif
}

/*
 * Given two mismatching words loaded from position @base, return the
 * appropriate non-zero result.
 *
 * When @signed_cmp is true, byte-swap on little-endian to get
 * lexicographic word order, then return <0 or >0.
 * When @signed_cmp is false, return 1 (unequal, sign unspecified).
 *
 * When @mismatch_pos is non-NULL, store the index of the first
 * differing byte using ctz/clz on the XOR of the two words.
 *
 * Both checks are constant-folded when the function is inlined with
 * literal arguments.
 */
static inline_lookup
int ft_word_mismatch(unsigned long va, unsigned long vb,
		unsigned int base, bool signed_cmp,
		unsigned int *mismatch_pos)
{
	if (mismatch_pos) {
		unsigned long diff = va ^ vb;
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
		*mismatch_pos = base + (unsigned int)__builtin_ctzl(diff) / 8;
#else
		*mismatch_pos = base + (unsigned int)__builtin_clzl(diff) / 8;
#endif
	}
	if (signed_cmp) {
		va = ft_bswap_long(va);
		vb = ft_bswap_long(vb);
		return va < vb ? -1 : 1;
	}
	return 1;
}

/*
 * ft_key_cmp_ordinals: compare @len bytes of ordinal data from two
 * sources.  Both @a and @b must be in ordinal space.
 *
 * Returns 0 when equal, non-zero when unequal.
 *
 * @signed_cmp: when true, the return value encodes lexicographic
 *   order (<0 means a < b, >0 means a > b).  When false, any
 *   non-zero value may be returned (allows the compiler to
 *   eliminate the bswap).
 *
 * @mismatch_pos: when non-NULL, receives the index of the first
 *   mismatching byte (undefined on full match).  Gates the
 *   bitscan instruction.
 *
 * All three use-cases (equality, mismatch position, signed
 * cardinality) share the same comparison logic.  Since this
 * function is force-inlined, both @signed_cmp and @mismatch_pos
 * checks are constant-folded at each call site.
 *
 * Dispatch is ordered by frequency: short keys (< 8 bytes) are the
 * most common case in trie traversal (collapsed suffixes, compressed
 * paths), followed by medium keys, then long keys where SIMD helps.
 */

/*
 * Helper: resolve a mismatch found at byte position @pos by
 * loading a full word from each array at that position and
 * delegating to ft_word_mismatch.  The word load is safe because
 * @remaining_key guarantees enough readable memory.
 */
static inline_lookup
int ft_byte_mismatch(const uint8_t *a, const uint8_t *b,
		unsigned int pos, bool signed_cmp,
		unsigned int *mismatch_pos)
{
	if (mismatch_pos)
		*mismatch_pos = pos;
	if (signed_cmp)
		return (int)a[pos] - (int)b[pos];
	return 1;
}

#if 0 /* AVX2 comparison — available but not used: SSE2 is sufficient
       * for typical key lengths and avoids an extra dispatch branch. */
#if defined(__AVX2__)
#ifndef FT_IMMINTRIN_INCLUDED
#define FT_IMMINTRIN_INCLUDED
#include <immintrin.h>
#endif
#endif
#if defined(__AVX2__) && !defined(FT_NO_SIMD_CMP)
/*
 * Helper: given a non-zero 32-bit mismatch mask from an AVX2
 * comparison starting at @base, resolve the first differing byte.
 */
static inline_lookup
int ft_avx2_mismatch(const uint8_t *a, const uint8_t *b,
		unsigned int base, unsigned int mask,
		bool signed_cmp, unsigned int *mismatch_pos)
{
	unsigned int pos = base + (unsigned int)__builtin_ctz(mask);

	return ft_byte_mismatch(a, b, pos, signed_cmp, mismatch_pos);
}
#endif /* __AVX2__ && !FT_NO_SIMD_CMP */
#endif /* AVX2 comparison disabled */

#if defined(__SSE2__)
#ifndef FT_IMMINTRIN_INCLUDED
#define FT_IMMINTRIN_INCLUDED
#include <immintrin.h>
#endif
#endif
#if defined(__SSE2__) && !defined(FT_NO_SIMD_CMP)
/*
 * Helper: given a non-zero 16-bit mismatch mask from an SSE2
 * comparison starting at @base, resolve the first differing byte.
 */
static inline_lookup
int ft_sse2_mismatch(const uint8_t *a, const uint8_t *b,
		unsigned int base, unsigned int mask,
		bool signed_cmp, unsigned int *mismatch_pos)
{
	unsigned int pos = base + (unsigned int)__builtin_ctz(mask);

	return ft_byte_mismatch(a, b, pos, signed_cmp, mismatch_pos);
}
#endif /* __SSE2__ */

/*
 * Building blocks for key comparison.  Each is self-contained and
 * handles its key length range completely, including tail.
 */

/* Compare len < 8 bytes.  Uses masked word or byte-by-byte. */
static inline_lookup
int ft_cmp_tiny(const uint8_t *a, const uint8_t *b,
		unsigned int len, unsigned int remaining_key,
		bool signed_cmp, unsigned int *mismatch_pos)
{
	if (remaining_key >= sizeof(unsigned long)) {
		unsigned long va, vb, mask;

		__builtin_memcpy(&va, a, sizeof(unsigned long));
		__builtin_memcpy(&vb, b, sizeof(unsigned long));
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
		mask = (1UL << (len * 8)) - 1;
#else
		mask = ~((1UL << ((sizeof(unsigned long) - len) * 8)) - 1);
#endif
		va &= mask;
		vb &= mask;
		if (va != vb)
			return ft_word_mismatch(va, vb, 0,
						signed_cmp, mismatch_pos);
	} else {
		unsigned int j;

		for (j = 0; j < len; j++) {
			if (a[j] != b[j])
				return ft_byte_mismatch(a, b, j,
						signed_cmp, mismatch_pos);
		}
	}
	return 0;
}

/* Compare len >= 8 bytes using word-at-a-time + overlapping tail. */
static inline_lookup
int ft_cmp_word(const uint8_t *a, const uint8_t *b,
		unsigned int len,
		bool signed_cmp, unsigned int *mismatch_pos)
{
	unsigned int j = 0;

	while (j + sizeof(unsigned long) <= len) {
		unsigned long va, vb;

		__builtin_memcpy(&va, a + j, sizeof(unsigned long));
		__builtin_memcpy(&vb, b + j, sizeof(unsigned long));
		if (va != vb)
			return ft_word_mismatch(va, vb, j,
						signed_cmp, mismatch_pos);
		j += sizeof(unsigned long);
	}
	if (j < len) {
		unsigned long va, vb;
		unsigned int tail = len - sizeof(unsigned long);

		__builtin_memcpy(&va, a + tail, sizeof(unsigned long));
		__builtin_memcpy(&vb, b + tail, sizeof(unsigned long));
		if (va != vb)
			return ft_word_mismatch(va, vb, tail,
						signed_cmp, mismatch_pos);
	}
	return 0;
}

#if defined(__SSE2__) && !defined(FT_NO_SIMD_CMP)
/* Compare len >= 16 bytes using SSE2 + overlapping 16-byte tail. */
static inline_lookup
int ft_cmp_sse2(const uint8_t *a, const uint8_t *b,
		unsigned int len,
		bool signed_cmp, unsigned int *mismatch_pos)
{
	unsigned int j = 0;

	while (j + 16 <= len) {
		__m128i va = _mm_loadu_si128((const __m128i *)(a + j));
		__m128i vb = _mm_loadu_si128((const __m128i *)(b + j));
		__m128i eq = _mm_cmpeq_epi8(va, vb);
		unsigned int mask = (unsigned int)_mm_movemask_epi8(eq);

		if (mask != 0xFFFFU)
			return ft_sse2_mismatch(a, b, j,
						~mask & 0xFFFF,
						signed_cmp, mismatch_pos);
		j += 16;
	}
	if (j < len) {
		unsigned int tail = len - 16;
		__m128i va = _mm_loadu_si128((const __m128i *)(a + tail));
		__m128i vb = _mm_loadu_si128((const __m128i *)(b + tail));
		__m128i eq = _mm_cmpeq_epi8(va, vb);
		unsigned int mask = (unsigned int)_mm_movemask_epi8(eq);

		if (mask != 0xFFFFU)
			return ft_sse2_mismatch(a, b, tail,
						~mask & 0xFFFF,
						signed_cmp, mismatch_pos);
	}
	return 0;
}
#endif /* __SSE2__ && !FT_NO_SIMD_CMP */

#if 0 /* AVX2 comparison — see ft_avx2_mismatch above. */
#if defined(__AVX2__) && !defined(FT_NO_SIMD_CMP)
/* Compare len >= 32 bytes using AVX2 + overlapping 32-byte tail. */
static inline_lookup
int ft_cmp_avx2(const uint8_t *a, const uint8_t *b,
		unsigned int len,
		bool signed_cmp, unsigned int *mismatch_pos)
{
	unsigned int j = 0;

	while (j + 32 <= len) {
		__m256i va = _mm256_loadu_si256((const __m256i *)(a + j));
		__m256i vb = _mm256_loadu_si256((const __m256i *)(b + j));
		__m256i eq = _mm256_cmpeq_epi8(va, vb);
		unsigned int mask = (unsigned int)_mm256_movemask_epi8(eq);

		if (mask != 0xFFFFFFFFU)
			return ft_avx2_mismatch(a, b, j,
						~mask, signed_cmp,
						mismatch_pos);
		j += 32;
	}
	if (j < len) {
		unsigned int tail = len - 32;
		__m256i va = _mm256_loadu_si256((const __m256i *)(a + tail));
		__m256i vb = _mm256_loadu_si256((const __m256i *)(b + tail));
		__m256i eq = _mm256_cmpeq_epi8(va, vb);
		unsigned int mask = (unsigned int)_mm256_movemask_epi8(eq);

		if (mask != 0xFFFFFFFFU)
			return ft_avx2_mismatch(a, b, tail,
						~mask, signed_cmp,
						mismatch_pos);
	}
	return 0;
}
#endif /* __AVX2__ && !FT_NO_SIMD_CMP */
#endif /* AVX2 comparison disabled */

/*
 * ft_key_cmp_ordinals: compare @len bytes in ordinal space.
 *
 * Dispatch in natural increasing order:
 *   < 8:  tiny (masked word or byte-by-byte) — hot for collapsed
 *         suffixes and short compressed paths
 *   < 16: word-at-a-time + overlapping tail
 *   >= 16: SSE2 loop + overlapping 16-byte tail
 */
static inline_lookup
int ft_key_cmp_ordinals(const uint8_t *a, const uint8_t *b,
		unsigned int len, unsigned int remaining_key,
		bool signed_cmp, unsigned int *mismatch_pos)
{
	if (len < sizeof(unsigned long))
		return ft_cmp_tiny(a, b, len, remaining_key,
				   signed_cmp, mismatch_pos);
	if (len < 16)
		return ft_cmp_word(a, b, len, signed_cmp, mismatch_pos);
#if defined(__SSE2__) && !defined(FT_NO_SIMD_CMP)
	return ft_cmp_sse2(a, b, len, signed_cmp, mismatch_pos);
#else
	return ft_cmp_word(a, b, len, signed_cmp, mismatch_pos);
#endif
}


static
struct cds_ft_inode_flag *ft_node_flag(struct cds_ft_inode *node,
		unsigned long type)
{
	assert(type < (1UL << FT_TYPE_BITS));
	return (struct cds_ft_inode_flag *) (((unsigned long) node) |
		(type << FT_INTERNAL_BITS) |
		FT_INTERNAL_MASK);
}

static
struct cds_ft_inode_flag *ft_node_flag_pool_1d(struct cds_ft_inode *node,
		unsigned long type, unsigned long bitsel)
{
	assert(type < (1UL << FT_TYPE_BITS));
	assert(bitsel < FT_BITS_PER_BYTE);
	return (struct cds_ft_inode_flag *) (((unsigned long) node) |
		(bitsel << (FT_TYPE_BITS + FT_INTERNAL_BITS)) |
		(type << FT_INTERNAL_BITS) |
		FT_INTERNAL_MASK);
}

static
struct cds_ft_inode_flag *ft_node_flag_pool_2d(struct cds_ft_inode *node,
		unsigned long type, unsigned int subclass_index)
{
	assert(type < (1UL << FT_TYPE_BITS));
	return (struct cds_ft_inode_flag *) (((unsigned long) node) |
		(subclass_index << (FT_TYPE_BITS + FT_INTERNAL_BITS)) |
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
	return ((unsigned long) node & FT_TAG_MASK_WIDE) == FT_COMPRESSED_MASK;
}
#else
static
bool ft_node_compressed(struct cds_ft_inode_flag *node __attribute__((unused)))
{
	return false;
}
#endif

#ifdef FEATURE_FT_COLLAPSE
static inline_lookup
bool ft_node_collapsed(struct cds_ft_inode_flag *node)
{
	return ((unsigned long) node & FT_TAG_MASK_WIDE) == FT_COLLAPSED_MASK;
}
#else
static
bool ft_node_collapsed(struct cds_ft_inode_flag *node __attribute__((unused)))
{
	return false;
}
#endif

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
 * This clears all tag and pool sub-index bits that sit below the
 * type's alignment boundary.  The shift amount is derived purely
 * from bits 1-3 with no dependency on bit 0.
 *
 * For non-internal nodes (bit 0 clear): external nodes are >= 8-byte
 * aligned (bits 0-2 zero), compressed/collapsed are >= 16-byte
 * aligned with tags in bits 1-2.  A fixed ~7UL mask suffices.
 *
 * The conditional select lets the two mask computations run in
 * parallel; the compiler emits a CMOV, keeping the critical path
 * to 4 cycles.
 */
static inline_lookup
struct cds_ft_inode *ft_node_ptr(struct cds_ft_inode_flag *node)
{
	unsigned long v = (unsigned long) node;

	/*
	 * Clear the top FT_SKIP_LEN_BITS (7 bits).
	 * Executed as `shl $7` + `shr $7` to save instruction cache
	 * bytes compared to a 10-byte `movabs` for FT_ADDR_MASK.
	 * The 2-cycle latency is completely hidden by the superscalar
	 * execution of the mask generation below.
	 */
	v = (v << FT_SKIP_LEN_BITS) >> FT_SKIP_LEN_BITS;

	/*
	 * Compute masks branchlessly using ILP.
	 * mask_internal generation runs in parallel with the bit 0 test.
	 */
	unsigned long mask_internal = (~15UL) << ((v >> 1) & 7);
	unsigned long mask = (v & 1) ? mask_internal : ~7UL;

	return (struct cds_ft_inode *) (v & mask);
}

static
struct cds_ft_inode *_ft_node_mask_ptr(struct cds_ft_inode_flag *node)
{
	unsigned long v = (unsigned long) node;

	v = (v << FT_SKIP_LEN_BITS) >> FT_SKIP_LEN_BITS;
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

	if (_ft_node_mask_ptr(node) == NULL) {
		return NODE_INDEX_NULL;
	}
	/* Compressed and collapsed nodes don't have a type index. */
	assert(!ft_node_compressed(node));
	assert(!ft_node_collapsed(node));
	type = (unsigned int) (((unsigned long) node & FT_TYPE_MASK) >> FT_INTERNAL_BITS);
	assert(type < (1UL << FT_TYPE_BITS));
	return type;
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
	return (struct cds_ft_compressed_node *)
		(((unsigned long) node) & ~(unsigned long) FT_TAG_MASK);
}

static
struct cds_ft_inode_flag *ft_collapsed_node_flag(
		struct cds_ft_collapsed_node *node)
{
	return (struct cds_ft_inode_flag *)
		(((unsigned long) node) | FT_COLLAPSED_MASK);
}

static inline_lookup
struct cds_ft_collapsed_node *ft_collapsed_node_ptr(
		struct cds_ft_inode_flag *node)
{
	return (struct cds_ft_collapsed_node *)
		(((unsigned long) node) & ~(unsigned long) FT_TAG_MASK_WIDE);
}

/* Skip-compressed pointer helpers. */

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
	return (struct cds_ft_inode_flag *)
		((unsigned long) child |
		 ((unsigned long) len << FT_SKIP_LEN_SHIFT));
}

/*
 * ft_skip_to_compressed: recover the compressed node from a skip
 * pointer by following the child's parent back-pointer.
 *
 * For internal/compressed/collapsed children: uses metadata->parent.
 * For external (leaf) children: uses cds_ft_node._ft_parent.
 *
 * Write-side or exact lookup path only.
 */
static inline
struct cds_ft_compressed_node *ft_skip_to_compressed(
		struct cds_ft_inode_flag *skip_ptr)
{
	struct cds_ft_inode_flag *child = ft_skip_child_ptr(skip_ptr);
	struct cds_ft_inode_flag *parent;

	if (ft_node_external(child))
		parent = rcu_dereference(((struct cds_ft_node *) child)->_ft_parent);
	else
		parent = rcu_dereference(cds_ft_item_to_metadata(
			ft_node_ptr(child))->parent);
	return ft_compressed_node_ptr(parent);
}

static inline
bool ft_group_skip_compressed(const struct cds_ft_group *group)
{
	return group->flags & CDS_FT_FLAG_SKIP_COMPRESSED;
}

/*
 * ft_flag_to_metadata: get the metadata for any node flag, including
 * skip-compressed pointers.  For skip pointers, returns the
 * compressed node's metadata.  For all others, returns
 * cds_ft_item_to_metadata(ft_node_ptr(nf)).
 *
 * Caller must ensure nf is not NULL and not external.
 */
static inline
struct cds_ft_metadata *ft_flag_to_metadata(struct cds_ft_inode_flag *nf)
{
	if (ft_node_skip_compressed(nf)) {
		struct cds_ft_compressed_node *cn =
			ft_skip_to_compressed(nf);
		return cds_ft_item_to_metadata(
			(struct cds_ft_inode *) cn);
	}
	return cds_ft_item_to_metadata(ft_node_ptr(nf));
}

/*
 * ft_skip_to_compressed_meta: shorthand to get the compressed node's
 * metadata from a skip pointer.
 */
static inline
struct cds_ft_metadata *ft_skip_to_compressed_meta(
		struct cds_ft_inode_flag *skip_ptr)
{
	return cds_ft_item_to_metadata(
		(struct cds_ft_inode *) ft_skip_to_compressed(skip_ptr));
}

/*
 * ft_update_skip_pointer: when a compressed node's child is replaced
 * (e.g., by recompact), update the skip pointer in the parent's slot
 * to encode the new child address.
 *
 * @parent_slot: pointer to the slot holding the skip pointer (in the
 *               grandparent node or root).
 * @cn: the compressed node whose child was replaced.
 *
 * If the slot doesn't hold a skip pointer, this is a no-op.
 */
static inline
void ft_update_skip_pointer(struct cds_ft_inode_flag **parent_slot,
		struct cds_ft_compressed_node *cn)
{
	struct cds_ft_inode_flag *slot_val;

	if (!parent_slot)
		return;
	slot_val = rcu_dereference(*parent_slot);
	if (!ft_node_skip_compressed(slot_val))
		return;
	rcu_assign_pointer(*parent_slot,
		ft_skip_compressed_flag(cn->child, cn->len));
}

/*
 * ft_publish_compressed: convert a compressed node flag to a skip
 * pointer if skip-compressed mode is enabled, the path length fits,
 * and the child has metadata (is not external).
 *
 * Call AFTER ft_set_parent and ft_init_node_density have been done
 * with the real compressed flag (@cflag).  The returned value is
 * what should be published/stored in parent child slots.
 */
static
struct cds_ft_inode_flag *ft_publish_compressed(struct cds_ft *ft,
		struct cds_ft_compressed_node *cn,
		struct cds_ft_inode_flag *cflag)
{
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
 * External (leaf) nodes: sets cds_ft_node._ft_parent.
 *
 * For skip-compressed pointers: the skip pointer represents a
 * compressed node in the tree.  Set the compressed node's parent
 * (not the compressed node's child's parent, which is the
 * compressed node itself and was set at creation time).
 *
 * Skip-compressed must be checked before external: a skip pointer
 * whose child is external has low tag bits == 0, which would match
 * ft_node_external on the raw value.
 *
 * Write-side only (mutex-held).
 */
static
void ft_set_parent(struct cds_ft_inode_flag *child_nf,
		struct cds_ft_inode_flag *parent_nf)
{
	if (!child_nf)
		return;
	if (ft_node_skip_compressed(child_nf)) {
		struct cds_ft_compressed_node *cn =
			ft_skip_to_compressed(child_nf);
		rcu_assign_pointer(
			cds_ft_item_to_metadata(
				(struct cds_ft_inode *) cn)->parent,
			parent_nf);
		return;
	}
	if (ft_node_external(child_nf)) {
		rcu_assign_pointer(
			((struct cds_ft_node *) child_nf)->_ft_parent,
			parent_nf);
		return;
	}
	rcu_assign_pointer(
		cds_ft_item_to_metadata(ft_node_ptr(child_nf))->parent,
		parent_nf);
}

/* Collapsed node accessors. */

static const unsigned int ft_collapsed_scan_sizes[] = {
	[FT_COLLAPSED_SCAN_32]  = 32,
	[FT_COLLAPSED_SCAN_64]  = 64,
	[FT_COLLAPSED_SCAN_128] = 128,
	[FT_COLLAPSED_SCAN_256] = 256,
};

/*
 * Load nr_entries with acquire ordering.  Returns the full byte
 * including the scan zone selector in bits 6-7.  Callers that
 * need just the entry count must mask with FT_COLLAPSED_NR_ENTRIES_MASK.
 * Callers that pass this to collapsed accessors (ft_collapsed_ptrs,
 * ft_collapsed_suffix, ft_collapsed_suffix_len, ft_collapsed_entry_dead,
 * ft_collapsed_scan_zone_size, ft_collapsed_offset_mask) must pass
 * the full value so the scan zone selector is preserved.
 */
static inline
unsigned int ft_collapsed_nr_entries(struct cds_ft_collapsed_node *cn)
{
	return uatomic_load(&cn->nr_entries, CMM_ACQUIRE);
}

/* Extract entry count from the full nr_entries value (mask out selector). */
static inline
unsigned int ft_collapsed_count(unsigned int nr_entries)
{
	return nr_entries & FT_COLLAPSED_NR_ENTRIES_MASK;
}

/*
 * Set the entry count, preserving the scan zone selector in bits 6-7.
 * Writer-only (not published to readers).
 */
static inline
void ft_collapsed_set_nr_entries(struct cds_ft_collapsed_node *cn,
		unsigned int count)
{
	cn->nr_entries = (cn->nr_entries & ~FT_COLLAPSED_NR_ENTRIES_MASK)
		| (count & FT_COLLAPSED_NR_ENTRIES_MASK);
}

/*
 * Publish an incremented entry count with store-release ordering.
 * Ensures readers (via load-acquire in ft_collapsed_nr_entries) see
 * the fully written suffix, offset, and pointer data before seeing
 * the incremented count.  Preserves the scan zone selector in bits 6-7.
 */
static inline
void ft_collapsed_publish_inc_nr_entries(struct cds_ft_collapsed_node *cn)
{
	/* Verify increment does not overflow into scan zone selector bits. */
	assert((cn->nr_entries >> FT_COLLAPSED_SCAN_SHIFT) ==
	       ((uint8_t)(cn->nr_entries + 1) >> FT_COLLAPSED_SCAN_SHIFT));
	uatomic_store(&cn->nr_entries, cn->nr_entries + 1, CMM_RELEASE);
}

/*
 * Scan zone size and offset mask derived from nr_entries.
 * Accept a preloaded nr_entries value to avoid reloading on
 * read-side fast paths.  Write-side callers can pass
 * cn->nr_entries directly.
 */
static inline
unsigned int ft_collapsed_scan_zone_size(unsigned int nr_entries)
{
	/* 32 << selector: selector 0=32B, 1=64B, 2=128B, 3=256B. */
	return 32U << (nr_entries >> FT_COLLAPSED_SCAN_SHIFT);
}

static inline
unsigned int ft_collapsed_offset_mask(unsigned int nr_entries)
{
	/*
	 * Branchless offset mask from the scan zone selector.
	 *
	 * Selectors 0-2 (32B/64B/128B): 7-bit offsets, bit 7 is
	 * the tombstone marker.  Mask = 0x7F.
	 *
	 * Selector 3 (256B): full 8-bit offsets (byte range 0-255
	 * needs all bits).  No tombstone bit.  Mask = 0xFF.
	 *
	 * Branchless: selector 3 has both bits set (binary 11).
	 * (sel >> 1) & sel & 1 is 1 only for sel=3, 0 otherwise.
	 * OR that into bit 7 of the base mask 0x7F.
	 */
	unsigned int sel = nr_entries >> FT_COLLAPSED_SCAN_SHIFT;

	return FT_COLLAPSED_OFFSET_MASK |
		(((sel >> 1) & sel & 1) << 7);
}

/*
 * Load entry @i's data byte once (relaxed atomic).  Use the returned
 * value to avoid reloading cn->data[i] across multiple accessors.
 */
static inline
uint8_t ft_collapsed_load_data(struct cds_ft_collapsed_node *cn,
		unsigned int i)
{
	return uatomic_load(&cn->data[i], CMM_RELAXED);
}

/*
 * Collapsed node accessors: accept preloaded data byte and nr_entries
 * to avoid reloading shared state.  nr_entries provides the scan zone
 * selector for offset mask and scan zone size computation.
 */

/* Suffix pointer from preloaded data byte and nr_entries. */
static inline
uint8_t *ft_collapsed_suffix(struct cds_ft_collapsed_node *cn,
		uint8_t data_i, unsigned int nr_entries)
{
	return ((uint8_t *) cn) + (data_i & ft_collapsed_offset_mask(nr_entries));
}

/* Suffix length from preloaded data byte and nr_entries. */
static inline
unsigned int ft_collapsed_suffix_len(struct cds_ft_collapsed_node *cn,
		uint8_t data_i, unsigned int i, unsigned int nr_entries)
{
	unsigned int mask = ft_collapsed_offset_mask(nr_entries);
	unsigned int start = data_i & mask;
	unsigned int end;

	if (i == 0)
		end = ft_collapsed_scan_zone_size(nr_entries);
	else
		end = ft_collapsed_load_data(cn, i - 1) & mask;
	assert(end >= start);
	return end - start;
}

/* Dead check from preloaded data byte and nr_entries. */
static inline
bool ft_collapsed_entry_dead(uint8_t data_i, unsigned int nr_entries)
{
	if ((nr_entries >> FT_COLLAPSED_SCAN_SHIFT) >= FT_COLLAPSED_SCAN_256)
		return false;
	return data_i & FT_COLLAPSED_TOMBSTONE;
}

static inline
struct cds_ft_inode_flag **ft_collapsed_ptrs(struct cds_ft_collapsed_node *cn,
		unsigned int nr_entries)
{
	return (struct cds_ft_inode_flag **)
		(((uint8_t *) cn) + ft_collapsed_scan_zone_size(nr_entries));
}

/*
 * Return codes for compressed node traversal helpers.
 * Used to tell callers which loop control action to take.
 */
enum ft_compressed_action {
	FT_COMPRESSED_CONTINUE,		/* Continue loop iteration. */
	FT_COMPRESSED_BREAK,		/* Break from loop. */
	FT_COMPRESSED_END,		/* Jump to function end (status set). */
	FT_COMPRESSED_GOING_UP,		/* Jump to going_up backtracking. */
	FT_COMPRESSED_DESCEND_CHILDREN,	/* Jump to descend_children. */
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
 * Fill ordinal_key and iter_path arrays for every level spanned by
 * a compressed node.  Used by read-side descent loops (lookup_nth,
 * minmax, etc.) to record the path through compressed nodes so that
 * going-up backtracking has valid entries at each level.
 */
static inline
void ft_fill_compressed_path(struct cds_ft_compressed_node *cn,
		uint8_t *ordinal_key, int base,
		struct cds_ft_inode_flag **iter_path, int path_base,
		struct cds_ft_inode_flag *node_flag)
{
	int j;

	for (j = 0; j < cn->len; j++) {
		ordinal_key[base + j] = cn->key_bytes[j];
		iter_path[path_base + j] = node_flag;
	}
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
	return cds_ft_item_to_metadata(ft_node_ptr(ft->root));
}

/*
 * Descent cursor — tracks current, parent, and grandparent positions
 * during a key-guided traversal of the trie.
 *
 * Each level stores both the flagged-pointer value (nf / pnf / ppnf)
 * and the address of the slot that holds it (nfp / pnfp / ppnfp).
 * Callers that do not need every field may leave the unused ones
 * NULL; the struct carries the superset so that a single descent
 * helper can serve graft, insert, remove, and detach paths.
 */
struct ft_descent {
	unsigned int depth;			/* Levels traversed (0 .. key_len). */
	struct cds_ft_inode_flag *nf;		/* Current node-flag value. */
	struct cds_ft_inode_flag **nfp;		/* Slot that holds @nf. */
	struct cds_ft_inode_flag *pnf;		/* Parent node-flag value. */
	struct cds_ft_inode_flag **pnfp;	/* Slot that holds @pnf. */
	struct cds_ft_inode_flag *ppnf;		/* Grandparent node-flag value. */
	struct cds_ft_inode_flag **ppnfp;	/* Slot that holds @ppnf. */
};

static
void ft_descent_init(struct ft_descent *d, struct cds_ft *ft)
{
	d->depth = 0;
	d->nf = ft->root;
	d->nfp = &ft->root;
	d->pnf = NULL;
	d->pnfp = NULL;
	d->ppnf = NULL;
	d->ppnfp = NULL;
}

/*
 * Advance descent state through a compressed node on full key match.
 * Updates parent chain, current pointer, and depth.  The caller is
 * responsible for snapshot, snapshot_n, and detach tracking before
 * calling this helper.
 */
static inline
void ft_descent_traverse_compressed(struct ft_descent *d,
		struct cds_ft_compressed_node *cn,
		const uint8_t **iter_key)
{
	d->ppnf  = d->pnf;
	d->ppnfp = d->pnfp;
	d->pnf   = d->nf;
	d->pnfp  = d->nfp;
	d->nf    = cn->child;
	d->nfp   = &cn->child;
	d->depth += cn->len;
	*iter_key += cn->len;
}

/*
 * Extended descent state for remove / detach operations.
 * Adds the detach-point bookkeeping used by ft_detach_node()
 * on top of the common descent cursor.
 *
 * During descent, the detach point is updated at potential
 * upward-walk termination points (multi-child nodes, nodes
 * with external_nodes, and the root).  After descent,
 * det_nfp / det_pfp are passed straight to ft_detach_node().
 */
struct ft_detach_descent {
	struct ft_descent d;
	struct cds_ft_inode_flag **det_nfp;	/* Detach-point node slot. */
	struct cds_ft_inode_flag **det_pfp;	/* Detach-point parent slot. */
	bool pending;				/* Waiting to capture det_nfp. */
};

static
void ft_detach_descent_init(struct ft_detach_descent *dd,
		struct cds_ft *ft)
{
	ft_descent_init(&dd->d, ft);
	dd->det_nfp = NULL;
	dd->det_pfp = &ft->root;
	dd->pending = true;
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

static inline uint8_t ft_linear_encode_nr_child(unsigned int nr_child,
		const struct cds_ft_type *type);

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
	/*
	 * Initialize data[0] with the ptr_offset encoding in the
	 * high bits (nr_child = 0 in the low bits).  This makes
	 * the pointer offset available from the first read,
	 * before any child is inserted.
	 */
	if (ft_type_is_linear(ft_type->type_class) ||
	    ft_type->type_class == FT_POOL)
		((struct cds_ft_inode *) p)->data[0] =
			ft_linear_encode_nr_child(0, ft_type);
	if (ft_debug_counters())
		uatomic_inc(&ft->nr_nodes_allocated);
	*_metadata = metadata;
	return p;
}

static
void free_cds_ft_node(struct cds_ft *ft, struct cds_ft_inode *node)
{
	struct cds_ft_metadata *metadata = cds_ft_item_to_metadata(node);

	cds_ft_free_item(metadata);
	if (ft_debug_counters() && node)
		uatomic_inc(&ft->nr_nodes_freed);
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

	metadata = cds_ft_alloc_item(ft, order, false);
	if (!metadata)
		return NULL;
	p = cds_ft_metadata_to_item(metadata);
	if (ft_debug_counters())
		uatomic_inc(&ft->nr_nodes_allocated);
	*_metadata = metadata;
	return p;
}

static
void free_compressed_node(struct cds_ft *ft,
		struct cds_ft_compressed_node *node)
{
	struct cds_ft_metadata *metadata =
		cds_ft_item_to_metadata((struct cds_ft_inode *) node);

	cds_ft_free_item(metadata);
	if (ft_debug_counters() && node)
		uatomic_inc(&ft->nr_nodes_freed);
}

/*
 * Collapsed node configurations:
 *
 *   Config  Alloc  Scan   Ptrs  Max entries  CL loads  slen req
 *   TINY    64B    32B    32B   4            1         slen >= 2
 *   SMALL   128B   64B    64B   8            2         slen >= 2
 *   WIDE    256B   128B   128B  16           3         slen >= 3
 *   MEDIUM  256B   64B    192B  24           2         slen >= 2
 *   XLARGE  512B   256B   256B  32           5         slen >= 5
 *
 * TINY is special: scan zone + pointers share a single cache line.
 * Lookup costs 1 CL load (vs 2 for SMALL/MEDIUM).  Selected via
 * post-walk compaction when entries fit in the 32B scan budget.
 *
 * Select smallest allocation fitting the density.  Walk with 64B
 * scan zone, then compact to TINY or upgrade to wider zone if
 * suffix quality allows.
 *
 * Cache-line guarantee: each entry's slen must justify the scan
 * zone's cache-line cost.  slen >= (scan_zone_size / 64) + 1
 * for ALL live entries (worst-case bound, not average).
 */
enum {
	FT_COLLAPSED_ORDER_TINY   = 6,	/* 64B alloc */
	FT_COLLAPSED_ORDER_SMALL  = 7,	/* 128B alloc */
	FT_COLLAPSED_ORDER_MEDIUM = 8,	/* 256B alloc */
	FT_COLLAPSED_ORDER_XLARGE = 9,	/* 512B alloc */

	/*
	 * Maximum entries for the largest configuration (order 9,
	 * 256B scan zone).  Used for stack buffer sizing only.
	 */
	FT_COLLAPSED_MAX_ENTRIES_MAX = ((1 << FT_COLLAPSED_ORDER_XLARGE) - 256)
				     / sizeof(struct cds_ft_inode_flag *),
};

static
struct cds_ft_collapsed_node *alloc_collapsed_node(struct cds_ft *ft,
		unsigned int order, unsigned int scan_sel,
		struct cds_ft_metadata **_metadata)
{
	struct cds_ft_metadata *metadata;
	struct cds_ft_collapsed_node *cn;
	unsigned int scan_sz = ft_collapsed_scan_sizes[scan_sel];

	metadata = cds_ft_alloc_item(ft, order, false);
	if (!metadata)
		return NULL;
	cn = (struct cds_ft_collapsed_node *) cds_ft_metadata_to_item(metadata);
	memset(cn, 0, scan_sz);
	/* Set scan zone selector in bits 6-7 of nr_entries (count starts at 0). */
	cn->nr_entries = scan_sel << FT_COLLAPSED_SCAN_SHIFT;
	if (ft_debug_counters())
		uatomic_inc(&ft->nr_nodes_allocated);
	*_metadata = metadata;
	return cn;
}

static
void free_collapsed_node(struct cds_ft *ft,
		struct cds_ft_collapsed_node *node)
{
	struct cds_ft_metadata *metadata =
		cds_ft_item_to_metadata((struct cds_ft_inode *) node);

	cds_ft_free_item(metadata);
	if (ft_debug_counters() && node)
		uatomic_inc(&ft->nr_nodes_freed);
}

/*
 * Maximum number of entries for a collapsed node of the given order
 * and scan zone selector.
 */
static inline
unsigned int ft_collapsed_max_entries(unsigned int order, unsigned int scan_sel)
{
	unsigned int scan_sz = ft_collapsed_scan_sizes[scan_sel];

	return ((1U << order) - scan_sz) / sizeof(struct cds_ft_inode_flag *);
}

#define __FT_ALIGN_MASK(v, mask)	(((v) + (mask)) & ~(mask))
#define FT_ALIGN(v, align)		__FT_ALIGN_MASK(v, (typeof(v)) (align) - 1)
#define __FT_FLOOR_MASK(v, mask)	((v) & ~(mask))
#define FT_FLOOR(v, align)		__FT_FLOOR_MASK(v, (typeof(v)) (align) - 1)

/*
 * Push a node and its depth onto the snapshot stack.
 * Used to maintain the parallel snapshot_depth[] array alongside
 * snapshot[] for local node density propagation.
 */
#define ft_snapshot_push(snap, snap_depth, nr, node_flag, depth)	\
	do {								\
		(snap_depth)[(nr)] = (depth);				\
		(snap)[(nr)++] = (node_flag);				\
	} while (0)

static inline_lookup
uint8_t *align_ptr_size(uint8_t *ptr)
{
	return (uint8_t *) FT_ALIGN((unsigned long) ptr, sizeof(void *));
}

/*
 * Linear node data[0] encoding:
 *   bits 0-4: nr_child (max 25)
 *   bits 5-7: pointer array offset / 8
 *
 * The pointer offset encodes where the child pointer array starts
 * within the node, precomputed from max_linear_child at node
 * creation.  This eliminates the ft_types[] load and alignment
 * computation from the lookup hot path — the offset is extracted
 * from the same byte as nr_child with a shift.
 */
#define FT_LINEAR_NR_CHILD_MASK		0x1F
#define FT_LINEAR_PTR_OFFSET_SHIFT	5

static inline_lookup
uint8_t ft_linear_node_get_nr_child(const struct cds_ft_type *type
		__attribute__((unused)),
		struct cds_ft_inode *node)
{
	/* load-acquire orders nr_child load before values and pointers */
	return uatomic_load(&node->data[0], CMM_ACQUIRE)
		& FT_LINEAR_NR_CHILD_MASK;
}

/*
 * Extract the pointer array base from node->data[0] high bits.
 * @raw: the acquire-loaded data[0] byte (containing both
 *       nr_child and ptr_offset).
 * Returns the pointer array as a struct cds_ft_inode_flag **,
 * suitable for indexing by child position.
 */
static inline_lookup
struct cds_ft_inode_flag **ft_linear_pointers(
		struct cds_ft_inode *node, uint8_t raw)
{
	unsigned int byte_offset = (raw >> FT_LINEAR_PTR_OFFSET_SHIFT)
		<< __builtin_ctz(sizeof(void *));
	return (struct cds_ft_inode_flag **)
		((uint8_t *) node + byte_offset);
}

/*
 * Encode nr_child and pointer offset into data[0].
 */
static inline
uint8_t ft_linear_encode_nr_child(unsigned int nr_child,
		const struct cds_ft_type *type)
{
	unsigned int ptr_offset = FT_ALIGN(
		1 + type->max_linear_child, sizeof(void *));
	unsigned int slots = ptr_offset / sizeof(void *);

	assert(nr_child <= FT_LINEAR_NR_CHILD_MASK);
	assert(slots < (1U << (8 - FT_LINEAR_PTR_OFFSET_SHIFT)));
	return (uint8_t)((slots << FT_LINEAR_PTR_OFFSET_SHIFT) | nr_child);
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
/*
 * Selective prefetch: dereference a child pointer and prefetch the
 * target node.  Use on read-side hot paths where the loaded pointer
 * will be immediately traversed.
 *
 * Skip prefetch for pool nodes (type 4-5) and pigeon nodes (type 6):
 * - Pool nodes encode subclass indices in pointer bits 4-8,
 *   offsetting the raw pointer by up to 496 bytes from the actual
 *   node — the prefetch would fetch a wrong cache line.
 * - Pigeon nodes are 2KB direct-indexed arrays (256 pointers);
 *   the dispatch byte selects a random position across 32 cache
 *   lines, so prefetching the base is almost always useless.
 *
 * Linear nodes (type 0-3) are small and benefit from prefetch.
 * Compressed/collapsed/external nodes (bit 0 clear) always benefit.
 *
 * The skip check tests: internal flag (bit 0) AND type high bit
 * (bit 3) both set → type >= 4 (pool or pigeon).  One AND + CMP.
 */
#define FT_TYPE_HIGH_BIT	(1UL << (FT_TYPE_BITS - 1 + FT_INTERNAL_BITS))
#define FT_PREFETCH_SKIP_MASK	(FT_TYPE_HIGH_BIT | FT_INTERNAL_MASK)

static inline void ft_maybe_prefetch(const void *ptr)
{
	unsigned long v = (unsigned long) ptr;

	/* Clear skip-compressed length bits. */
	v = (v << FT_SKIP_LEN_BITS) >> FT_SKIP_LEN_BITS;
	if ((v & FT_PREFETCH_SKIP_MASK) != FT_PREFETCH_SKIP_MASK)
		__builtin_prefetch((const void *) v);
}

/*
 * ft_dereference_prefetch: for tagged FT node pointers (may have
 * pool/pigeon subclass bits).  Uses ft_maybe_prefetch to skip
 * pool/pigeon types.
 *
 * ft_dereference_prefetch_external: for plain (non-tagged) pointers
 * like external_nodes.  Direct prefetch, no tag check needed.
 */
#define ft_dereference_prefetch(p)		\
	({							\
		__typeof__(p) __ft_tmp = rcu_dereference(p);	\
		ft_maybe_prefetch(__ft_tmp);			\
		__ft_tmp;					\
	})

#define ft_dereference_prefetch_external(p)	\
	({							\
		__typeof__(p) __ft_tmp = rcu_dereference(p);	\
		if (__ft_tmp)					\
			__builtin_prefetch(__ft_tmp);		\
		__ft_tmp;					\
	})

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
 * The order in which values and pointers are does does not matter: if
 * a value is missing, we return NULL. If a value is there, but its
 * associated pointers is still NULL, we return NULL too.
 */


/*
 * Adaptive lookup: dispatch bytewise, SWAR, or SIMD based on
 * the node type's max_linear_child.
 *
 * The thresholds define the minimum max_linear_child for each
 * strategy.  The data layout starts with nr_child (1 byte)
 * followed by key entries, so a 16-byte SSE2 load covers
 * 15 key entries and an 8-byte SWAR word covers 7.
 *
 * Tunable via -DFT_SIMD_LINEAR_THRESHOLD=N and
 * -DFT_SWAR_LINEAR_THRESHOLD=N at compile time.
 */
/* FT_SIMD/SWAR/WIDE_LINEAR_THRESHOLD defined near ft_types[]. */

/* SWAR constants. */
#define L_ONES_A (-1UL / 255)
#define L_HIGHS_A (L_ONES_A * 0x80)

static inline_lookup
struct cds_ft_inode_flag *ft_linear_node_get_nth_swar(
		struct cds_ft_inode *node,
		struct cds_ft_inode_flag ***node_flag_ptr,
		uint8_t n, uint8_t nr_child, uint8_t raw)
{
	unsigned long *data_words = (unsigned long *)node->data;
	unsigned long mask = n * L_ONES_A, xor_res, has_zero;
	unsigned long first_word = data_words[0];
	unsigned int i;

#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
	xor_res = first_word ^ (mask | 0xFFUL);
	has_zero = (xor_res - L_ONES_A) & ~xor_res & L_HIGHS_A;
	if (has_zero) {
		i = (__builtin_ctzl(has_zero) >> 3) - 1;
		if (i < nr_child)
			goto found;
	}
#else
	xor_res = first_word ^ (mask | (0xFFUL << ((sizeof(unsigned long) - 1) * 8)));
	has_zero = (xor_res - L_ONES_A) & ~xor_res & L_HIGHS_A;
	if (has_zero) {
		i = (__builtin_clzl(has_zero) >> 3) - 1;
		if (i < nr_child)
			goto found;
	}
#endif
	for (unsigned int w = 1; w * sizeof(unsigned long) < (unsigned long)((raw >> FT_LINEAR_PTR_OFFSET_SHIFT) << __builtin_ctz(sizeof(void *))); w++) {
		xor_res = data_words[w] ^ mask;
		has_zero = (xor_res - L_ONES_A) & ~xor_res & L_HIGHS_A;
		if (has_zero) {
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
			i = (w * sizeof(unsigned long)) + (__builtin_ctzl(has_zero) >> 3) - 1;
#else
			i = (w * sizeof(unsigned long)) + (__builtin_clzl(has_zero) >> 3) - 1;
#endif
			if (i < nr_child)
				goto found;
		}
	}
	if (caa_unlikely(node_flag_ptr))
		*node_flag_ptr = NULL;
	return NULL;
found:
	{
		struct cds_ft_inode_flag **pointers =
			ft_linear_pointers(node, raw);
		if (caa_unlikely(node_flag_ptr))
			*node_flag_ptr = &pointers[i];
		return ft_dereference_acquire(pointers[i]);
	}
}

#if defined(__SSE2__)
static inline_lookup
struct cds_ft_inode_flag *ft_linear_node_get_nth_simd(
		struct cds_ft_inode *node,
		struct cds_ft_inode_flag ***node_flag_ptr,
		uint8_t n, uint8_t nr_child, uint8_t raw)
{
	uint8_t *data = &node->data[0];
	__m128i target = _mm_set1_epi8(n);
	__m128i chunk;
	unsigned int mask, phys_idx;

	chunk = _mm_loadu_si128((__m128i *)data);
	mask = _mm_movemask_epi8(_mm_cmpeq_epi8(chunk, target));
	mask &= ~1U; /* Ignore nr_child byte at index 0. */
	if (mask) {
		phys_idx = __builtin_ctz(mask);
		if (phys_idx <= nr_child)
			goto found;
	}
	if (nr_child >= 16) {
		chunk = _mm_loadu_si128((__m128i *)(data + 16));
		mask = _mm_movemask_epi8(_mm_cmpeq_epi8(chunk, target));
		if (mask) {
			phys_idx = 16 + __builtin_ctz(mask);
			if (phys_idx <= nr_child)
				goto found;
		}
	}
	if (caa_unlikely(node_flag_ptr))
		*node_flag_ptr = NULL;
	return NULL;
found:
	{
		unsigned int i = phys_idx - 1;
		struct cds_ft_inode_flag **pointers =
			ft_linear_pointers(node, raw);
		if (caa_unlikely(node_flag_ptr))
			*node_flag_ptr = &pointers[i];
		return ft_dereference_acquire(pointers[i]);
	}
}
#endif /* __SSE2__ */

/*
 * ft_linear_node_get_nth: bytewise scan for small linear nodes
 * (FT_LINEAR, max_linear_child < threshold).
 */
static inline_lookup
struct cds_ft_inode_flag *ft_linear_node_get_nth(const struct cds_ft_type __attribute__((unused)) *type,
		struct cds_ft_inode *node,
		struct cds_ft_inode_flag ***node_flag_ptr,
		uint8_t n)
{
	uint8_t raw = uatomic_load(&node->data[0], CMM_ACQUIRE);
	uint8_t nr_child = raw & FT_LINEAR_NR_CHILD_MASK;
	uint8_t *values = &node->data[1];
	unsigned int i;

	if (nr_child == 0) {
		if (caa_unlikely(node_flag_ptr))
			*node_flag_ptr = NULL;
		return NULL;
	}
	for (i = 0; i < nr_child; i++) {
		if (uatomic_load(&values[i], CMM_RELAXED) == n)
			break;
	}
	if (i >= nr_child) {
		if (caa_unlikely(node_flag_ptr))
			*node_flag_ptr = NULL;
		return NULL;
	}
	{
		struct cds_ft_inode_flag **pointers =
			ft_linear_pointers(node, raw);
		if (caa_unlikely(node_flag_ptr))
			*node_flag_ptr = &pointers[i];
		return ft_dereference_acquire(pointers[i]);
	}
}

/*
 * ft_linear_wide_node_get_nth: SWAR or SIMD scan for large linear
 * nodes (FT_LINEAR_WIDE, max_linear_child >= threshold).
 */
static inline_lookup
struct cds_ft_inode_flag *ft_linear_wide_node_get_nth(const struct cds_ft_type __attribute__((unused)) *type,
		struct cds_ft_inode *node,
		struct cds_ft_inode_flag ***node_flag_ptr,
		uint8_t n)
{
	uint8_t raw = uatomic_load(&node->data[0], CMM_ACQUIRE);
	uint8_t nr_child = raw & FT_LINEAR_NR_CHILD_MASK;

	if (nr_child == 0) {
		if (caa_unlikely(node_flag_ptr))
			*node_flag_ptr = NULL;
		return NULL;
	}
#if defined(__SSE2__)
	return ft_linear_node_get_nth_simd(node, node_flag_ptr, n, nr_child, raw);
#else
	return ft_linear_node_get_nth_swar(node, node_flag_ptr, n, nr_child, raw);
#endif
}

static inline_lookup
struct cds_ft_inode_flag *ft_linear_node_get_direction(const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		int n, uint8_t *result_key,
		enum ft_direction dir)
{
	uint8_t nr_child;
	uint8_t *values;
	struct cds_ft_inode_flag **pointers;
	struct cds_ft_inode_flag *ptr, *match_ptr = NULL;
	unsigned int i;
	int match_v;

	assert(ft_type_is_linear(type->type_class) || type->type_class == FT_POOL);
	assert(dir == FT_LEFT || dir == FT_RIGHT);

	if (dir == FT_LEFT) {
		match_v = -1;
	} else {
		match_v = FT_ENTRY_PER_NODE;
	}

	nr_child = ft_linear_node_get_nr_child(type, node);
	cmm_smp_rmb();	/* read nr_child before values and pointers */
	assert(nr_child <= type->max_linear_child);
	assert(!ft_type_is_linear(type->type_class) || nr_child == 0 || nr_child >= type->min_child);

	values = &node->data[1];
	pointers = (struct cds_ft_inode_flag **) align_ptr_size(&values[type->max_linear_child]);
	for (i = 0; i < nr_child; i++) {
		unsigned int v;

		v = uatomic_load(&values[i], CMM_RELAXED);
		ptr = ft_dereference_acquire(pointers[i]);
		if (!ptr)
			continue;
		if (dir == FT_LEFT) {
			if ((int) v < n && (int) v > match_v) {
				match_v = v;
				match_ptr = ptr;
				/* Found value immediately left of n. */
				if (match_v == n - 1)
					break;
			}
		} else {
			if ((int) v > n && (int) v < match_v) {
				match_v = v;
				match_ptr = ptr;
				/* Found value immediately right of n. */
				if (match_v == n + 1)
					break;
			}
		}
	}

	if (!match_ptr) {
		return NULL;
	}
	assert(match_v >= 0 && match_v < FT_ENTRY_PER_NODE);

	*result_key = (uint8_t) match_v;
	return match_ptr;
}

static inline_lookup
void ft_linear_node_get_ith_pos(const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		uint8_t i,
		uint8_t *v,
		struct cds_ft_inode_flag **iter)
{
	uint8_t *values;
	struct cds_ft_inode_flag **pointers;

	assert(ft_type_is_linear(type->type_class) || type->type_class == FT_POOL);
	assert(i < ft_linear_node_get_nr_child(type, node));

	values = &node->data[1];
	*v = values[i];
	pointers = (struct cds_ft_inode_flag **) align_ptr_size(&values[type->max_linear_child]);
	*iter = ft_dereference_acquire(pointers[i]);
}

static inline_lookup
struct cds_ft_inode *ft_pool_get_linear_subnode(const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		struct cds_ft_inode_flag *node_flag,
		uint8_t n)
{
	switch (type->nr_pool_order) {
	case 1:
	{
		unsigned long bitsel = ft_node_pool_1d_bitsel(node_flag);
		unsigned long index = ((unsigned long) n >> bitsel) & 0x1;
		return (struct cds_ft_inode *) &node->data[index << type->pool_size_order];
	}
	case 2:
	{
		unsigned int C_n8_r2_index, subclass_index;
		uint8_t bits[2];

		ft_node_pool_2d_index(node_flag, &C_n8_r2_index);
		index_to_bits_C_n8_r2(C_n8_r2_index, bits);
		subclass_index = value_and_bits_to_subclass_index(n, bits);
		return (struct cds_ft_inode *) &node->data[subclass_index << type->pool_size_order];
	}
	default:
		assert(0);
		return NULL;
	}
}

static inline_lookup
struct cds_ft_inode_flag *ft_pool_node_get_nth(const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		struct cds_ft_inode_flag *node_flag,
		struct cds_ft_inode_flag ***node_flag_ptr,
		uint8_t n)
{
	struct cds_ft_inode *linear = ft_pool_get_linear_subnode(type, node, node_flag, n);
	/* Pool subnodes are always large enough for wide scan. */
	return ft_linear_wide_node_get_nth(type, linear, node_flag_ptr, n);
}

static inline_lookup
struct cds_ft_inode *ft_pool_node_get_ith_pool(const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		uint8_t i)
{
	assert(type->type_class == FT_POOL);
	return (struct cds_ft_inode *)
		&node->data[(unsigned int) i << type->pool_size_order];
}

static inline_lookup
struct cds_ft_inode_flag *ft_pool_node_get_direction(const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		struct cds_ft_inode_flag *node_flag __attribute__((unused)),
		int n, uint8_t *result_key,
		enum ft_direction dir)
{
	unsigned int pool_nr;
	int match_v;
	struct cds_ft_inode_flag *match_node_flag = NULL;

	assert(type->type_class == FT_POOL);
	assert(dir == FT_LEFT || dir == FT_RIGHT);

#ifdef FEATURE_USE_BITMAP_SCAN
	if (type->bitmap) {
		struct cds_ft_bitmap *bitmap = cds_ft_item_to_bitmap(node, type->order);
retry:
		if (dir == FT_LEFT)
			match_v = cds_find_prev_bit(bitmap->bitmap, FT_ENTRY_PER_NODE, n - 1);
		else
			match_v = cds_find_next_bit(bitmap->bitmap, FT_ENTRY_PER_NODE, n + 1);
		if (match_v >= 0) {
			match_node_flag = ft_pool_node_get_nth(type, node, node_flag, NULL, (uint8_t) match_v);
			/*
			 * The source of truth is the pointer load from
			 * get_nth. Continue the bitmap scan if the node
			 * is not found.
			 */
			if (!match_node_flag) {
				n = match_v;
				goto retry;
			}
		}
		goto end;
	}
#endif

	if (dir == FT_LEFT) {
		match_v = -1;
	} else {
		match_v = FT_ENTRY_PER_NODE;
	}

	for (pool_nr = 0; pool_nr < (1U << type->nr_pool_order); pool_nr++) {
		struct cds_ft_inode *pool =
			ft_pool_node_get_ith_pool(type,
				node, pool_nr);
		uint8_t nr_child =
			ft_linear_node_get_nr_child(type, pool);
		unsigned int j;

		for (j = 0; j < nr_child; j++) {
			struct cds_ft_inode_flag *iter;
			uint8_t v;

			ft_linear_node_get_ith_pos(type, pool,
					j, &v, &iter);
			if (!iter)
				continue;
			if (dir == FT_LEFT) {
				if ((int) v < n && (int) v > match_v) {
					match_v = v;
					match_node_flag = iter;
					/* Found value immediately left of n. */
					if (match_v == n - 1)
						goto end;
				}
			} else {
				if ((int) v > n && (int) v < match_v) {
					match_v = v;
					match_node_flag = iter;
					/* Found value immediately right of n. */
					if (match_v == n + 1)
						goto end;
				}
			}
		}
	}
end:
	if (match_node_flag)
		*result_key = (uint8_t) match_v;
	return match_node_flag;
}

static inline_lookup
struct cds_ft_inode_flag *ft_pigeon_node_get_nth(const struct cds_ft_type __attribute__((unused)) *type,
		struct cds_ft_inode *node,
		struct cds_ft_inode_flag ***node_flag_ptr,
		uint8_t n)
{
	struct cds_ft_inode_flag **child_node_flag_ptr;
	struct cds_ft_inode_flag *child_node_flag;

	assert(!type || type->type_class == FT_PIGEON);
	child_node_flag_ptr = &((struct cds_ft_inode_flag **) node->data)[n];
	child_node_flag = ft_dereference_acquire(*child_node_flag_ptr);
	//dbg_printf("ft_pigeon_node_get_nth child_node_flag_ptr %p\n",
	//	child_node_flag_ptr);
	if (caa_unlikely(node_flag_ptr))
		*node_flag_ptr = child_node_flag_ptr;
	return child_node_flag;
}

static inline_lookup
struct cds_ft_inode_flag *ft_pigeon_node_get_direction(const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		int n, uint8_t *result_key,
		enum ft_direction dir)
{
	struct cds_ft_inode_flag **child_node_flag_ptr;
	struct cds_ft_inode_flag *child_node_flag;
#ifdef FEATURE_USE_BITMAP_SCAN
	struct cds_ft_bitmap *bitmap = cds_ft_item_to_bitmap(node, type->order);
#endif
	int i;

	assert(type->type_class == FT_PIGEON);
	assert(dir == FT_LEFT || dir == FT_RIGHT);

#ifdef FEATURE_USE_BITMAP_SCAN
retry:
	if (dir == FT_LEFT)
		i = cds_find_prev_bit(bitmap->bitmap, FT_ENTRY_PER_NODE, n - 1);
	else
		i = cds_find_next_bit(bitmap->bitmap, FT_ENTRY_PER_NODE, n + 1);
	if (i >= 0) {
		child_node_flag_ptr = &((struct cds_ft_inode_flag **) node->data)[i];
		child_node_flag = ft_dereference_acquire(*child_node_flag_ptr);
		if (!child_node_flag) {
			/*
			 * The source of truth is the pointer load.
			 * Continue the bitmap scan if the node is
			 * not found.
			 */
			n = i;
			goto retry;
		}
		dbg_printf("ft_pigeon_node_get child_node_flag %p\n", child_node_flag);
		*result_key = (uint8_t) i;
		return child_node_flag;
	}
#else
	if (dir == FT_LEFT) {
		/* n - 1 is first value left of n */
		for (i = n - 1; i >= 0; i--) {
			child_node_flag_ptr = &((struct cds_ft_inode_flag **) node->data)[i];
			child_node_flag = ft_dereference_acquire(*child_node_flag_ptr);
			if (child_node_flag) {
				dbg_printf("ft_pigeon_node_get_left child_node_flag %p\n",
					child_node_flag);
				*result_key = (uint8_t) i;
				return child_node_flag;
			}
		}
	} else {
		/* n + 1 is first value right of n */
		for (i = n + 1; i < FT_ENTRY_PER_NODE; i++) {
			child_node_flag_ptr = &((struct cds_ft_inode_flag **) node->data)[i];
			child_node_flag = ft_dereference_acquire(*child_node_flag_ptr);
			if (child_node_flag) {
				dbg_printf("ft_pigeon_node_get_right child_node_flag %p\n",
					child_node_flag);
				*result_key = (uint8_t) i;
				return child_node_flag;
			}
		}
	}
#endif
	return NULL;
}

static inline_lookup
struct cds_ft_inode_flag *ft_pigeon_node_get_ith_pos(const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		uint8_t i)
{
	return ft_pigeon_node_get_nth(type, node, NULL, i);
}

/*
 * ft_node_get_nth: get nth item from a node.
 * node_flag is already rcu_dereference'd.
 */
/*
 * Tag-to-type_class table: maps bits 0-3 of a tagged pointer
 * directly to the type_class, bypassing the ft_types[] struct
 * load for the dispatch decision.  16 bytes, always in L1.
 *
 * Eliminates a dependent load chain from the critical path:
 * tag bits → ft_types[] address computation → struct load.
 * Instead: tag bits → single byte load from small table.
 *
 * Initialized at startup by ft_init_tag_to_class().
 */
static uint8_t ft_tag_to_class[16];

static void __attribute__((constructor))
ft_init_tag_to_class(void)
{
	unsigned int i;

	for (i = 0; i < 16; i++) {
		if (!(i & FT_INTERNAL_MASK)) {
			/* External, compressed, or collapsed. */
			ft_tag_to_class[i] = FT_NULL;
		} else {
			unsigned int type_idx = (i >> FT_INTERNAL_BITS) & 0x7;

			ft_tag_to_class[i] = ft_types[type_idx].type_class;
		}
	}
}

static inline_lookup
/*
 * ft_node_get_nth_skip: raw child slot access.  Returns the slot
 * value as-is, including skip-compressed pointers.  Used only by
 * candidate lookup which resolves skip pointers itself.
 */
struct cds_ft_inode_flag *ft_node_get_nth_skip(struct cds_ft_inode_flag *node_flag,
		struct cds_ft_inode_flag ***node_flag_ptr,
		uint8_t n)
{
	unsigned long tag = (unsigned long) node_flag & 0xF;
	uint8_t tc = ft_tag_to_class[tag];
	struct cds_ft_inode *node;

	if (caa_unlikely(tc == FT_NULL)) {
		if (caa_unlikely(node_flag_ptr))
			*node_flag_ptr = NULL;
		return NULL;
	}

	node = ft_node_ptr(node_flag);

	/*
	 * Dispatch on type_class from the tag table.
	 * Linear (bytewise scan) is the most common type in
	 * byte-indexed tries — check it first to minimize
	 * branch mispredictions on mixed-type workloads.
	 */
	{
		unsigned int type_index = (tag >> FT_INTERNAL_BITS) & 0x7;
		const struct cds_ft_type *type = &ft_types[type_index];

		if (caa_likely(tc == FT_LINEAR))
			return ft_linear_node_get_nth(type, node,
					node_flag_ptr, n);
		if (tc == FT_LINEAR_WIDE)
			return ft_linear_wide_node_get_nth(type, node,
					node_flag_ptr, n);
		if (tc == FT_POOL)
			return ft_pool_node_get_nth(type, node, node_flag,
					node_flag_ptr, n);
		return ft_pigeon_node_get_nth(NULL, node,
				node_flag_ptr, n);
	}
}

/*
 * ft_node_get_nth: child slot access with skip-compressed resolution.
 * If the child is a skip pointer, converts it to the underlying
 * compressed node flag so callers see it as a regular compressed node.
 * Used by all paths except candidate lookup.
 */
static inline_lookup
struct cds_ft_inode_flag *ft_node_get_nth(struct cds_ft_inode_flag *node_flag,
		struct cds_ft_inode_flag ***node_flag_ptr,
		uint8_t n)
{
	struct cds_ft_inode_flag *child;

	child = ft_node_get_nth_skip(node_flag, node_flag_ptr, n);
	if (ft_node_skip_compressed(child))
		child = ft_compressed_node_flag(
			ft_skip_to_compressed(child));
	return child;
}

/*
 * ft_node_find_child: reverse lookup — given a parent internal node and
 * a child pointer, find the key byte and slot that lead to that child.
 *
 * Returns true if found, with *n_ret set to the key byte and *slot_ret
 * set to a pointer to the slot (cds_ft_inode_flag **) within the parent.
 * Returns false if the child is not found (should not happen on a
 * well-formed trie).
 *
 * Only handles internal node types (linear, pool, pigeon).
 * Compressed and collapsed parents are handled separately by callers.
 * Write-side only (mutex-held).
 */
static
bool ft_node_find_child(struct cds_ft_inode_flag *parent_nf,
		struct cds_ft_inode_flag *child_nf,
		uint8_t *n_ret,
		struct cds_ft_inode_flag ***slot_ret)
{
	struct cds_ft_inode *node = ft_node_ptr(parent_nf);
	unsigned int type_index = ft_node_type(parent_nf);
	const struct cds_ft_type *type = &ft_types[type_index];

	switch (type->type_class) {
	case FT_LINEAR:
	case FT_LINEAR_WIDE:
	{
		uint8_t nr_child = ft_linear_node_get_nr_child(type, node);
		unsigned int i;

		for (i = 0; i < nr_child; i++) {
			struct cds_ft_inode_flag *iter;
			uint8_t v;

			ft_linear_node_get_ith_pos(type, node, i, &v, &iter);
			if (iter == child_nf) {
				*n_ret = v;
				if (slot_ret)
					ft_node_get_nth(parent_nf, slot_ret, v);
				return true;
			}
		}
		return false;
	}
	case FT_POOL:
	{
		unsigned int pool_nr;

		for (pool_nr = 0; pool_nr < (1U << type->nr_pool_order); pool_nr++) {
			struct cds_ft_inode *pool =
				ft_pool_node_get_ith_pool(type, node, pool_nr);
			uint8_t nr_child = ft_linear_node_get_nr_child(type, pool);
			unsigned int j;

			for (j = 0; j < nr_child; j++) {
				struct cds_ft_inode_flag *iter;
				uint8_t v;

				ft_linear_node_get_ith_pos(type, pool, j, &v, &iter);
				if (iter == child_nf) {
					*n_ret = v;
					if (slot_ret)
						ft_node_get_nth(parent_nf, slot_ret, v);
					return true;
				}
			}
		}
		return false;
	}
	case FT_PIGEON:
	{
		unsigned int i;

		for (i = 0; i < FT_ENTRY_PER_NODE; i++) {
			struct cds_ft_inode_flag *iter;

			iter = ft_pigeon_node_get_ith_pos(type, node, i);
			if (iter == child_nf) {
				*n_ret = (uint8_t) i;
				if (slot_ret)
					ft_node_get_nth(parent_nf, slot_ret, i);
				return true;
			}
		}
		return false;
	}
	default:
		assert(0);
		return false;
	}
}

static inline_lookup
struct cds_ft_inode_flag *ft_node_get_direction(struct cds_ft_inode_flag *node_flag,
		int n, uint8_t *result_key,
		enum ft_direction dir)
{
	unsigned int type_index;
	struct cds_ft_inode *node;
	const struct cds_ft_type *type;
	struct cds_ft_inode_flag *child;

	/*
	 * Compressed node: no branching at any level within the
	 * compressed path.  Return NULL to indicate no siblings,
	 * causing the going-up walk to continue ascending.
	 */
	if (ft_node_compressed(node_flag))
		return NULL;
	node = ft_node_ptr(node_flag);
	assert(node != NULL);
	type_index = ft_node_type(node_flag);
	type = &ft_types[type_index];

	switch (type->type_class) {
	case FT_LINEAR:
	case FT_LINEAR_WIDE:
		child = ft_linear_node_get_direction(type, node, n, result_key, dir);
		break;
	case FT_POOL:
		child = ft_pool_node_get_direction(type, node, node_flag, n, result_key, dir);
		break;
	case FT_PIGEON:
		child = ft_pigeon_node_get_direction(type, node, n, result_key, dir);
		break;
	default:
		assert(0);
		return (void *) -1UL;
	}
	if (ft_node_skip_compressed(child))
		child = ft_compressed_node_flag(
			ft_skip_to_compressed(child));
	return child;
}

static inline_lookup
struct cds_ft_inode_flag *ft_node_get_leftright(struct cds_ft_inode_flag *node_flag,
		unsigned int n, uint8_t *result_key,
		enum ft_direction dir)
{
	return ft_node_get_direction(node_flag, n, result_key, dir);
}

static inline_lookup
struct cds_ft_inode_flag *ft_node_get_minmax(struct cds_ft_inode_flag *node_flag,
		uint8_t *result_key,
		enum ft_direction dir)
{
	struct cds_ft_inode_flag *ret;

	switch (dir) {
	case FT_LEFTMOST:
		ret = ft_node_get_direction(node_flag,
				-1, result_key, FT_RIGHT);
		break;
	case FT_RIGHTMOST:
		ret = ft_node_get_direction(node_flag,
				FT_ENTRY_PER_NODE, result_key, FT_LEFT);
		break;
	default:
		assert(0);
	}
	return ret;
}

static
int ft_linear_node_set_nth(const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		struct cds_ft_metadata *metadata,
		uint8_t n,
		struct cds_ft_inode_flag *child_node_flag,
		bool *_replace_old_ptr)
{
	uint8_t nr_child;
	uint8_t *values, *nr_child_ptr;
	struct cds_ft_inode_flag **pointers;
	unsigned int i, unused = 0;
	bool replace_old_ptr = false;

	assert(ft_type_is_linear(type->type_class) || type->type_class == FT_POOL);

	nr_child_ptr = &node->data[0];
	dbg_printf("linear set nth: n %u, nr_child_ptr %p\n",
		(unsigned int) n, nr_child_ptr);
	nr_child = *nr_child_ptr & FT_LINEAR_NR_CHILD_MASK;
	assert(nr_child <= type->max_linear_child);

	values = &node->data[1];
	pointers = (struct cds_ft_inode_flag **) align_ptr_size(&values[type->max_linear_child]);
	/* Check if node value is already populated */
	for (i = 0; i < nr_child; i++) {
		if (values[i] == n) {
			if (pointers[i])
				replace_old_ptr = true;
			break;
		} else {
			if (!pointers[i])
				unused++;
		}
	}
	if (i == nr_child && nr_child >= type->max_linear_child) {
		if (unused)
			return -ERANGE;	/* recompact node */
		else
			return -ENOSPC;	/* No space left in this node type */
	}

	/* If we expanded the nr_child, increment it */
	if (i == nr_child) {
		assert(pointers[i] == NULL);
		uatomic_store(&pointers[i], child_node_flag, CMM_RELAXED);
		uatomic_store(&values[nr_child], n, CMM_RELAXED);
		/* store-release: write pointer and value before nr_child.
		 * Preserve the ptr_offset high bits. */
		uatomic_store(nr_child_ptr,
			ft_linear_encode_nr_child(nr_child + 1, type),
			CMM_RELEASE);
	} else {
		/* Replacing a NULL or external node pointer. */
		rcu_assign_pointer(pointers[i], child_node_flag);
	}
	if (!replace_old_ptr)
		metadata->nr_child++;
	dbg_printf("linear set nth: %u child, metadata: %u child, for node %p\n",
		(unsigned int) uatomic_load(nr_child_ptr, CMM_RELAXED),
		(unsigned int) metadata->nr_child,
		node);
	if (_replace_old_ptr)
		*_replace_old_ptr = replace_old_ptr;
	return 0;
}

static
int ft_pool_node_set_nth(const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		struct cds_ft_inode_flag *node_flag,
		struct cds_ft_metadata *metadata,
		uint8_t n,
		struct cds_ft_inode_flag *child_node_flag)
{
	struct cds_ft_inode *linear = ft_pool_get_linear_subnode(type, node, node_flag, n);
	bool replace_old_ptr = false;
	int ret;

	ret = ft_linear_node_set_nth(type, linear, metadata, n, child_node_flag, &replace_old_ptr);
#ifdef FEATURE_USE_BITMAP_SCAN
	if (ret == 0 && !replace_old_ptr && type->bitmap) {
		struct cds_ft_bitmap *bitmap = cds_ft_item_to_bitmap(node, type->order);

		/* Set n in bitmap. */
		cds_set_bit_relaxed(bitmap->bitmap, n);
	}
#endif
	return ret;
}

static
int ft_pigeon_node_set_nth(const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		struct cds_ft_metadata *metadata,
		uint8_t n,
		struct cds_ft_inode_flag *child_node_flag)
{
	struct cds_ft_inode_flag **ptr;
	bool replace_old_ptr = false;

	assert(type->type_class == FT_PIGEON);
	ptr = &((struct cds_ft_inode_flag **) node->data)[n];
	if (*ptr)
		replace_old_ptr = true;
	rcu_assign_pointer(*ptr, child_node_flag);
	if (!replace_old_ptr) {
#ifdef FEATURE_USE_BITMAP_SCAN
		struct cds_ft_bitmap *bitmap = cds_ft_item_to_bitmap(node, type->order);

		/* Set n in bitmap. */
		cds_set_bit_relaxed(bitmap->bitmap, n);
#endif
		metadata->nr_child++;
	}
	return 0;
}

/*
 * _ft_node_set_nth: set nth item within a node. Return an error
 * (negative error value) if it is already there.
 */
static
int _ft_node_set_nth(const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		struct cds_ft_inode_flag *node_flag,
		struct cds_ft_metadata *metadata,
		uint8_t n,
		struct cds_ft_inode_flag *child_node_flag)
{
	int ret;

	switch (type->type_class) {
	case FT_LINEAR:
	case FT_LINEAR_WIDE:
		ret = ft_linear_node_set_nth(type, node, metadata, n, child_node_flag, NULL);
		break;
	case FT_POOL:
		ret = ft_pool_node_set_nth(type, node, node_flag, metadata, n, child_node_flag);
		break;
	case FT_PIGEON:
		ret = ft_pigeon_node_set_nth(type, node, metadata, n, child_node_flag);
		break;
	case FT_NULL:
		return -ENOSPC;
	default:
		assert(0);
		return -EINVAL;
	}
	if (!ret) {
		ft_set_parent(child_node_flag, node_flag);
		/*
		 * Skip-compressed: record the slot address in the
		 * compressed node's metadata->skip_slot so the skip
		 * pointer can be updated when cn->child changes.
		 */
		if (ft_node_skip_compressed(child_node_flag)) {
			struct cds_ft_inode_flag **slot_ptr = NULL;

			ft_node_get_nth_skip(node_flag, &slot_ptr, n);
			if (slot_ptr) {
				struct cds_ft_compressed_node *cn =
					ft_skip_to_compressed(child_node_flag);
				cds_ft_item_to_metadata(
					(struct cds_ft_inode *) cn)->skip_slot =
					slot_ptr;
			}
		}
	}
	return ret;
}

static
int ft_linear_node_replace_ptr(const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		struct cds_ft_metadata *metadata,
		struct cds_ft_inode_flag **node_flag_ptr,
		struct cds_ft_inode_flag *newptr)
{
	uint8_t *nr_child_ptr;

	assert(ft_type_is_linear(type->type_class) || type->type_class == FT_POOL);

	nr_child_ptr = &node->data[0];
	assert((*nr_child_ptr & FT_LINEAR_NR_CHILD_MASK) <= type->max_linear_child);

	if (ft_type_is_linear(type->type_class) && !newptr) {
		assert(!metadata->fallback_removal_count);
		if (metadata->nr_child <= type->min_child) {
			/* We need to try recompacting the node */
			return -EFBIG;
		}
	}
	dbg_printf("linear replace ptr: nr_child_ptr %p\n", nr_child_ptr);
	assert(*node_flag_ptr != NULL);
	rcu_assign_pointer(*node_flag_ptr, newptr);
	/*
	 * Value and nr_child are never changed (would cause ABA issue).
	 * Instead, we leave the pointer to NULL and recompact the node
	 * once in a while. It is allowed to set a NULL pointer to a new
	 * value without recompaction though.
	 * Only update the metadata node accounting.
	 */
	if (!newptr)
		metadata->nr_child--;
	dbg_printf("linear replace ptr: %u child, metadata: %u child, for node %p newptr %p\n",
		(unsigned int) uatomic_load(nr_child_ptr, CMM_RELAXED),
		(unsigned int) metadata->nr_child,
		node, newptr);
	return 0;
}

static
int ft_pool_node_replace_ptr(const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		struct cds_ft_inode_flag *node_flag,
		struct cds_ft_metadata *metadata,
		struct cds_ft_inode_flag **node_flag_ptr,
		uint8_t n,
		struct cds_ft_inode_flag *newptr)
{
	struct cds_ft_inode *linear;
	int ret;

	if (!newptr) {
		if (metadata->fallback_removal_count) {
			metadata->fallback_removal_count--;
		} else {
			/* We should try recompacting the node */
			if (metadata->nr_child <= type->min_child)
				return -EFBIG;
		}
	}

	linear = ft_pool_get_linear_subnode(type, node, node_flag, n);
	ret = ft_linear_node_replace_ptr(type, linear, metadata, node_flag_ptr, newptr);
#ifdef FEATURE_USE_BITMAP_SCAN
	if (ret == 0 && !newptr && type->bitmap) {
		struct cds_ft_bitmap *bitmap = cds_ft_item_to_bitmap(node, type->order);

		/* Clear n in bitmap. */
		cds_clear_bit_relaxed(bitmap->bitmap, n);
	}
#endif
	return ret;
}

static
int ft_pigeon_node_replace_ptr(const struct cds_ft_type *type,
		struct cds_ft_inode *node __attribute__((unused)),
		struct cds_ft_metadata *metadata,
		struct cds_ft_inode_flag **node_flag_ptr,
		uint8_t n __attribute__((unused)),
		struct cds_ft_inode_flag *newptr)
{
	assert(type->type_class == FT_PIGEON);

	if (!newptr) {
		if (metadata->fallback_removal_count) {
			metadata->fallback_removal_count--;
		} else {
			/* We should try recompacting the node */
			if (metadata->nr_child <= type->min_child)
				return -EFBIG;
		}
	}
	dbg_printf("ft_pigeon_node_replace_ptr: replace ptr: %p by %p\n", *node_flag_ptr, newptr);
	assert(*node_flag_ptr != NULL);
	rcu_assign_pointer(*node_flag_ptr, newptr);
	if (!newptr) {
#ifdef FEATURE_USE_BITMAP_SCAN
		struct cds_ft_bitmap *bitmap = cds_ft_item_to_bitmap(node, type->order);

		/* Clear n in bitmap. */
		cds_clear_bit_relaxed(bitmap->bitmap, n);
#endif
		metadata->nr_child--;
	}
	return 0;
}

/*
 * _ft_node_replace_ptr: replace ptr item within a node. Return an error
 * (negative error value) if it is not found (-ENOENT).
 */
static
int _ft_node_replace_ptr(const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		struct cds_ft_inode_flag *node_flag,
		struct cds_ft_metadata *metadata,
		struct cds_ft_inode_flag **node_flag_ptr,
		uint8_t n, struct cds_ft_inode_flag *newptr)
{
	int ret;

	switch (type->type_class) {
	case FT_LINEAR:
	case FT_LINEAR_WIDE:
		ret = ft_linear_node_replace_ptr(type, node, metadata, node_flag_ptr, newptr);
		break;
	case FT_POOL:
		ret = ft_pool_node_replace_ptr(type, node, node_flag, metadata, node_flag_ptr, n, newptr);
		break;
	case FT_PIGEON:
		ret = ft_pigeon_node_replace_ptr(type, node, metadata, node_flag_ptr, n, newptr);
		break;
	case FT_NULL:
		return -ENOENT;
	default:
		assert(0);
		return -EINVAL;
	}
	if (!ret)
		ft_set_parent(newptr, node_flag);
	return ret;
}

/*
 * Calculate bit distribution. Returns the bit (0 to 7) that splits the
 * distribution in two sub-distributions containing as much elements one
 * compared to the other.
 */
static
unsigned int ft_node_sum_distribution_1d(enum ft_recompact mode,
		const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		uint8_t n,
		struct cds_ft_inode_flag **nullify_node_flag_ptr)
{
	uint8_t nr_one[FT_BITS_PER_BYTE];
	unsigned int bitsel = 0, bit_i, overall_best_distance = UINT_MAX;
	unsigned int distrib_nr_child = 0;

	memset(nr_one, 0, sizeof(nr_one));

	switch (type->type_class) {
	case FT_LINEAR:
	case FT_LINEAR_WIDE:
	{
		uint8_t nr_child =
			ft_linear_node_get_nr_child(type, node);
		unsigned int i;

		for (i = 0; i < nr_child; i++) {
			struct cds_ft_inode_flag *iter;
			uint8_t v;

			ft_linear_node_get_ith_pos(type, node, i, &v, &iter);
			if (!iter)
				continue;
			if (mode == FT_RECOMPACT_DEL && *nullify_node_flag_ptr == iter)
				continue;
			for (bit_i = 0; bit_i < FT_BITS_PER_BYTE; bit_i++) {
				if (v & (1U << bit_i))
					nr_one[bit_i]++;
			}
			distrib_nr_child++;
		}
		break;
	}
	case FT_POOL:
	{
		unsigned int pool_nr;

		for (pool_nr = 0; pool_nr < (1U << type->nr_pool_order); pool_nr++) {
			struct cds_ft_inode *pool =
				ft_pool_node_get_ith_pool(type,
					node, pool_nr);
			uint8_t nr_child =
				ft_linear_node_get_nr_child(type, pool);
			unsigned int j;

			for (j = 0; j < nr_child; j++) {
				struct cds_ft_inode_flag *iter;
				uint8_t v;

				ft_linear_node_get_ith_pos(type, pool,
						j, &v, &iter);
				if (!iter)
					continue;
				if (mode == FT_RECOMPACT_DEL && *nullify_node_flag_ptr == iter)
					continue;
				for (bit_i = 0; bit_i < FT_BITS_PER_BYTE; bit_i++) {
					if (v & (1U << bit_i))
						nr_one[bit_i]++;
				}
				distrib_nr_child++;
			}
		}
		break;
	}
	case FT_PIGEON:
	{
		unsigned int i;

		assert(mode == FT_RECOMPACT_DEL);
		for (i = 0; i < FT_ENTRY_PER_NODE; i++) {
			struct cds_ft_inode_flag *iter;

			iter = ft_pigeon_node_get_ith_pos(type, node, i);
			if (!iter)
				continue;
			if (mode == FT_RECOMPACT_DEL && *nullify_node_flag_ptr == iter)
				continue;
			for (bit_i = 0; bit_i < FT_BITS_PER_BYTE; bit_i++) {
				if (i & (1U << bit_i))
					nr_one[bit_i]++;
			}
			distrib_nr_child++;
		}
		break;
	}
	case FT_NULL:
		assert(mode == FT_RECOMPACT_ADD_NEXT);
		break;
	default:
		assert(0);
		break;
	}

	if (mode == FT_RECOMPACT_ADD_NEXT || mode == FT_RECOMPACT_ADD_SAME) {
		for (bit_i = 0; bit_i < FT_BITS_PER_BYTE; bit_i++) {
			if (n & (1U << bit_i))
				nr_one[bit_i]++;
		}
		distrib_nr_child++;
	}

	/*
	 * The best bit selector is that for which the number of ones is
	 * closest to half of the number of children in the
	 * distribution. We calculate the distance using the double of
	 * the sub-distribution sizes to eliminate truncation error.
	 */
	for (bit_i = 0; bit_i < FT_BITS_PER_BYTE; bit_i++) {
		unsigned int distance_to_best;

		distance_to_best = abs_int(((unsigned int) nr_one[bit_i] << 1U) - distrib_nr_child);
		if (distance_to_best < overall_best_distance) {
			overall_best_distance = distance_to_best;
			bitsel = bit_i;
		}
	}
	dbg_printf("1 dimension pool bit selection: (%u)\n", bitsel);
	return bitsel;
}

/*
 * Calculate bit distribution in two dimensions. Returns the two bits
 * (each 0 to 7) that splits the distribution in four sub-distributions
 * containing as much elements one compared to the other.
 */
static
void ft_node_sum_distribution_2d(enum ft_recompact mode,
		const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		uint8_t n,
		struct cds_ft_inode_flag **nullify_node_flag_ptr,
		unsigned int *_bitsel)
{
	uint8_t nr_2d_11[FT_BITS_PER_BYTE][FT_BITS_PER_BYTE],
		nr_2d_10[FT_BITS_PER_BYTE][FT_BITS_PER_BYTE],
		nr_2d_01[FT_BITS_PER_BYTE][FT_BITS_PER_BYTE],
		nr_2d_00[FT_BITS_PER_BYTE][FT_BITS_PER_BYTE];
	unsigned int bitsel[2] = { 0, 1 };
	unsigned int bit_i, bit_j;
	int overall_best_distance = INT_MAX;
	unsigned int distrib_nr_child = 0;

	memset(nr_2d_11, 0, sizeof(nr_2d_11));
	memset(nr_2d_10, 0, sizeof(nr_2d_10));
	memset(nr_2d_01, 0, sizeof(nr_2d_01));
	memset(nr_2d_00, 0, sizeof(nr_2d_00));

	switch (type->type_class) {
	case FT_LINEAR:
	case FT_LINEAR_WIDE:
	{
		uint8_t nr_child =
			ft_linear_node_get_nr_child(type, node);
		unsigned int i;

		for (i = 0; i < nr_child; i++) {
			struct cds_ft_inode_flag *iter;
			uint8_t v;

			ft_linear_node_get_ith_pos(type, node, i, &v, &iter);
			if (!iter)
				continue;
			if (mode == FT_RECOMPACT_DEL && *nullify_node_flag_ptr == iter)
				continue;
			for (bit_i = 0; bit_i < FT_BITS_PER_BYTE; bit_i++) {
				for (bit_j = bit_i + 1; bit_j < FT_BITS_PER_BYTE; bit_j++) {
					if (v & (1U << bit_i)) {
						if (v & (1U << bit_j)) {
							nr_2d_11[bit_i][bit_j]++;
						} else {
							nr_2d_10[bit_i][bit_j]++;
						}
					} else {
						if (v & (1U << bit_j)) {
							nr_2d_01[bit_i][bit_j]++;
						} else {
							nr_2d_00[bit_i][bit_j]++;
						}
					}
				}
			}
			distrib_nr_child++;
		}
		break;
	}
	case FT_POOL:
	{
		unsigned int pool_nr;

		for (pool_nr = 0; pool_nr < (1U << type->nr_pool_order); pool_nr++) {
			struct cds_ft_inode *pool =
				ft_pool_node_get_ith_pool(type,
					node, pool_nr);
			uint8_t nr_child =
				ft_linear_node_get_nr_child(type, pool);
			unsigned int j;

			for (j = 0; j < nr_child; j++) {
				struct cds_ft_inode_flag *iter;
				uint8_t v;

				ft_linear_node_get_ith_pos(type, pool,
						j, &v, &iter);
				if (!iter)
					continue;
				if (mode == FT_RECOMPACT_DEL && *nullify_node_flag_ptr == iter)
					continue;
				for (bit_i = 0; bit_i < FT_BITS_PER_BYTE; bit_i++) {
					for (bit_j = bit_i + 1; bit_j < FT_BITS_PER_BYTE; bit_j++) {
						if (v & (1U << bit_i)) {
							if (v & (1U << bit_j)) {
								nr_2d_11[bit_i][bit_j]++;
							} else {
								nr_2d_10[bit_i][bit_j]++;
							}
						} else {
							if (v & (1U << bit_j)) {
								nr_2d_01[bit_i][bit_j]++;
							} else {
								nr_2d_00[bit_i][bit_j]++;
							}
						}
					}
				}
				distrib_nr_child++;
			}
		}
		break;
	}
	case FT_PIGEON:
	{
		unsigned int i;

		assert(mode == FT_RECOMPACT_DEL);
		for (i = 0; i < FT_ENTRY_PER_NODE; i++) {
			struct cds_ft_inode_flag *iter;

			iter = ft_pigeon_node_get_ith_pos(type, node, i);
			if (!iter)
				continue;
			if (mode == FT_RECOMPACT_DEL && *nullify_node_flag_ptr == iter)
				continue;
			for (bit_i = 0; bit_i < FT_BITS_PER_BYTE; bit_i++) {
				for (bit_j = bit_i + 1; bit_j < FT_BITS_PER_BYTE; bit_j++) {
					if (i & (1U << bit_i)) {
						if (i & (1U << bit_j)) {
							nr_2d_11[bit_i][bit_j]++;
						} else {
							nr_2d_10[bit_i][bit_j]++;
						}
					} else {
						if (i & (1U << bit_j)) {
							nr_2d_01[bit_i][bit_j]++;
						} else {
							nr_2d_00[bit_i][bit_j]++;
						}
					}
				}
			}
			distrib_nr_child++;
		}
		break;
	}
	case FT_NULL:
		assert(mode == FT_RECOMPACT_ADD_NEXT);
		break;
	default:
		assert(0);
		break;
	}

	if (mode == FT_RECOMPACT_ADD_NEXT || mode == FT_RECOMPACT_ADD_SAME) {
		for (bit_i = 0; bit_i < FT_BITS_PER_BYTE; bit_i++) {
			for (bit_j = bit_i + 1; bit_j < FT_BITS_PER_BYTE; bit_j++) {
				if (n & (1U << bit_i)) {
					if (n & (1U << bit_j)) {
						nr_2d_11[bit_i][bit_j]++;
					} else {
						nr_2d_10[bit_i][bit_j]++;
					}
				} else {
					if (n & (1U << bit_j)) {
						nr_2d_01[bit_i][bit_j]++;
					} else {
						nr_2d_00[bit_i][bit_j]++;
					}
				}
			}
		}
		distrib_nr_child++;
	}

	/*
	 * The best bit selector is that for which the number of nodes
	 * in each sub-class is closest to one-fourth of the number of
	 * children in the distribution. We calculate the distance using
	 * 4 times the size of the sub-distribution to eliminate
	 * truncation error.
	 */
	for (bit_i = 0; bit_i < FT_BITS_PER_BYTE; bit_i++) {
		for (bit_j = bit_i + 1; bit_j < FT_BITS_PER_BYTE; bit_j++) {
			int distance_to_best[4];

			distance_to_best[0] = ((unsigned int) nr_2d_11[bit_i][bit_j] << 2U) - distrib_nr_child;
			distance_to_best[1] = ((unsigned int) nr_2d_10[bit_i][bit_j] << 2U) - distrib_nr_child;
			distance_to_best[2] = ((unsigned int) nr_2d_01[bit_i][bit_j] << 2U) - distrib_nr_child;
			distance_to_best[3] = ((unsigned int) nr_2d_00[bit_i][bit_j] << 2U) - distrib_nr_child;

			/* Consider worse distance above best */
			if (distance_to_best[1] > 0 && distance_to_best[1] > distance_to_best[0])
				distance_to_best[0] = distance_to_best[1];
			if (distance_to_best[2] > 0 && distance_to_best[2] > distance_to_best[0])
				distance_to_best[0] = distance_to_best[2];
			if (distance_to_best[3] > 0 && distance_to_best[3] > distance_to_best[0])
				distance_to_best[0] = distance_to_best[3];

			/*
			 * If our worse distance is better than overall,
			 * we become new best candidate.
			 */
			if (distance_to_best[0] < overall_best_distance) {
				overall_best_distance = distance_to_best[0];
				bitsel[0] = bit_i;
				bitsel[1] = bit_j;
			}
		}
	}

	dbg_printf("2 dimensions pool bit selection: (%u,%u)\n", bitsel[0], bitsel[1]);

	/* Return our bit selection */
	_bitsel[0] = bitsel[0];
	_bitsel[1] = bitsel[1];
}

static
unsigned int find_nearest_type_index(unsigned int type_index,
		unsigned int nr_nodes, bool is_root)
{
	const struct cds_ft_type *type;

	assert(type_index != NODE_INDEX_NULL);
	if (nr_nodes == 0) {
		/*
		 * The root node is kept alive with 0 children (smallest
		 * linear type).  All other nodes are pruned.
		 */
		return is_root ? 0 : NODE_INDEX_NULL;
	}
	for (;;) {
		type = &ft_types[type_index];
		if (nr_nodes < type->min_child)
			type_index--;
		else if (nr_nodes > type->max_child)
			type_index++;
		else
			break;
	}
	return type_index;
}

/*
 * ft_node_recompact_add: recompact a node, adding a new child.
 * Return 0 on success or negative error value on error.
 */
static
int ft_node_recompact(enum ft_recompact mode,
		struct cds_ft *ft,
		unsigned int old_type_index,
		const struct cds_ft_type *old_type,
		struct cds_ft_inode *old_node,
		struct cds_ft_metadata *metadata,
		struct cds_ft_inode_flag **old_node_flag_ptr, uint8_t n,
		struct cds_ft_inode_flag *child_node_flag,
		struct cds_ft_inode_flag **nullify_node_flag_ptr,
		struct cds_ft_inode **old_node_ret,
		bool is_root)
{
	unsigned int new_type_index;
	struct cds_ft_inode *new_node;
	struct cds_ft_metadata *new_metadata;
	const struct cds_ft_type *new_type;
	struct cds_ft_inode_flag *new_node_flag = NULL;
	int ret;
	int fallback = 0;

	/*
	 * Need to find nearest type index even for ADD_SAME, because
	 * this recompaction, when applied to linear nodes, will garbage
	 * collect dummy (NULL) entries, and can therefore cause a few
	 * linear representations to be skipped.
	 */
	switch (mode) {
	case FT_RECOMPACT_ADD_SAME:
		new_type_index = find_nearest_type_index(old_type_index,
			metadata->nr_child + 1, false);
		dbg_printf("Recompact for node with %u children\n",
			metadata->nr_child + 1);
		break;
	case FT_RECOMPACT_ADD_NEXT:
		if (!metadata || old_type_index == NODE_INDEX_NULL) {
			new_type_index = 0;
			dbg_printf("Recompact for NULL\n");
		} else {
			new_type_index = find_nearest_type_index(old_type_index,
				metadata->nr_child + 1, false);
			dbg_printf("Recompact for node with %u children\n",
				metadata->nr_child + 1);
		}
		break;
	case FT_RECOMPACT_DEL:
		new_type_index = find_nearest_type_index(old_type_index,
			metadata->nr_child - 1, is_root);
		dbg_printf("Recompact for node with %u children\n",
			metadata->nr_child - 1);
		break;
	default:
		assert(0);
	}

retry:		/* for fallback */
	new_metadata = NULL;
	dbg_printf("Recompact from type %d to type %d\n",
			old_type_index, new_type_index);
	new_type = &ft_types[new_type_index];
	if (new_type_index != NODE_INDEX_NULL) {
		new_node = alloc_cds_ft_node(ft, new_type, &new_metadata);
		if (!new_node)
			return -ENOMEM;

		if (new_type->type_class == FT_POOL) {
			switch (new_type->nr_pool_order) {
			case 1:
			{
				unsigned int node_distrib_bitsel;

				node_distrib_bitsel =
					ft_node_sum_distribution_1d(mode,
						old_type, old_node,
						n, nullify_node_flag_ptr);
				assert(!((unsigned long) new_node & FT_POOL_1D_MASK));
				new_node_flag = ft_node_flag_pool_1d(new_node,
					new_type_index, node_distrib_bitsel);
				break;
			}
			case 2:
			{
				unsigned int node_distrib_bitsel[2];
				unsigned int subclass_index;
				uint8_t mask;

				ft_node_sum_distribution_2d(mode,
					old_type, old_node,
					n, nullify_node_flag_ptr,
					node_distrib_bitsel);
				assert(!((unsigned long) new_node & FT_POOL_2D_MASK));
				mask = (1U << node_distrib_bitsel[0]) | (1U << node_distrib_bitsel[1]);
				subclass_index = mask_to_index_C_n8_r2(mask);
				new_node_flag = ft_node_flag_pool_2d(new_node,
					new_type_index, subclass_index);
				break;
			}
			default:
				assert(0);
			}
		} else {
			new_node_flag = ft_node_flag(new_node, new_type_index);
		}

		dbg_printf("Recompact inherit from %p\n", metadata);
		if (metadata) {
			new_metadata->parent = metadata->parent;
			new_metadata->fallback_removal_count = metadata->fallback_removal_count;
			new_metadata->external_nodes = metadata->external_nodes;
			uatomic_store(&new_metadata->nr_keys,
				metadata->nr_keys, CMM_RELAXED);
		}
		if (fallback)
			new_metadata->fallback_removal_count =
						FT_FALLBACK_REMOVAL_COUNT;
	} else {
		new_node = NULL;
		new_node_flag = NULL;
	}

	assert(mode != FT_RECOMPACT_ADD_NEXT || old_type->type_class != FT_PIGEON);

	if (new_type_index == NODE_INDEX_NULL)
		goto skip_copy;

	switch (old_type->type_class) {
	case FT_LINEAR:
	case FT_LINEAR_WIDE:
	{
		uint8_t nr_child =
			ft_linear_node_get_nr_child(old_type, old_node);
		unsigned int i;

		for (i = 0; i < nr_child; i++) {
			struct cds_ft_inode_flag *iter;
			uint8_t v;

			ft_linear_node_get_ith_pos(old_type, old_node, i, &v, &iter);
			if (!iter)
				continue;
			if (mode == FT_RECOMPACT_DEL && *nullify_node_flag_ptr == iter)
				continue;
			ret = _ft_node_set_nth(new_type, new_node, new_node_flag,
					new_metadata, v, iter);
			if (new_type->type_class == FT_POOL && ret) {
				goto fallback_toosmall;
			}
			assert(!ret);
		}
		break;
	}
	case FT_POOL:
	{
		unsigned int pool_nr;

		for (pool_nr = 0; pool_nr < (1U << old_type->nr_pool_order); pool_nr++) {
			struct cds_ft_inode *pool =
				ft_pool_node_get_ith_pool(old_type,
					old_node, pool_nr);
			uint8_t nr_child =
				ft_linear_node_get_nr_child(old_type, pool);
			unsigned int j;

			for (j = 0; j < nr_child; j++) {
				struct cds_ft_inode_flag *iter;
				uint8_t v;

				ft_linear_node_get_ith_pos(old_type, pool,
						j, &v, &iter);
				if (!iter)
					continue;
				if (mode == FT_RECOMPACT_DEL && *nullify_node_flag_ptr == iter)
					continue;
				ret = _ft_node_set_nth(new_type, new_node, new_node_flag,
						new_metadata, v, iter);
				if (new_type->type_class == FT_POOL
						&& ret) {
					goto fallback_toosmall;
				}
				assert(!ret);
			}
		}
		break;
	}
	case FT_NULL:
		assert(mode == FT_RECOMPACT_ADD_NEXT);
		break;
	case FT_PIGEON:
	{
		unsigned int i;

		assert(mode == FT_RECOMPACT_DEL);
		for (i = 0; i < FT_ENTRY_PER_NODE; i++) {
			struct cds_ft_inode_flag *iter;

			iter = ft_pigeon_node_get_ith_pos(old_type, old_node, i);
			if (!iter)
				continue;
			if (mode == FT_RECOMPACT_DEL && *nullify_node_flag_ptr == iter)
				continue;
			ret = _ft_node_set_nth(new_type, new_node, new_node_flag,
					new_metadata, i, iter);
			if (new_type->type_class == FT_POOL && ret) {
				goto fallback_toosmall;
			}
			assert(!ret);
		}
		break;
	}
	default:
		assert(0);
		ret = -EINVAL;
		goto end;
	}
skip_copy:

	if (mode == FT_RECOMPACT_ADD_NEXT || mode == FT_RECOMPACT_ADD_SAME) {
		/* add node */
		ret = _ft_node_set_nth(new_type, new_node, new_node_flag,
				new_metadata, n, child_node_flag);
		if (new_type->type_class == FT_POOL && ret) {
			goto fallback_toosmall;
		}
		assert(!ret);
	}

	if (fallback) {
		dbg_printf("Using fallback for %u children, node type index: %u, mode %s\n",
			new_metadata->nr_child, old_type_index, mode == FT_RECOMPACT_ADD_NEXT ? "add_next" :
				(mode == FT_RECOMPACT_DEL ? "del" : "add_same"));
		if (ft_debug_counters())
			uatomic_inc(&ft->node_fallback_count_distribution[new_metadata->nr_child]);
	}

	/*
	 * Inherit the old node's parent pointer so upward walks
	 * (density propagation, ft_skip_to_compressed) can find
	 * the parent from the new node.
	 *
	 * If the recompacted node was the child of a compressed
	 * node published as a skip pointer, update the skip
	 * pointer BEFORE updating cn->child.  This ensures
	 * candidate readers (which follow the skip pointer)
	 * see the new child before exact/inequality readers
	 * (which follow cn->child) do.  The old child remains
	 * alive until after a grace period.
	 */
	if (old_node) {
		struct cds_ft_metadata *old_meta =
			cds_ft_item_to_metadata(old_node);
		struct cds_ft_inode_flag *old_parent = old_meta->parent;

		new_metadata->parent = old_parent;

		if (old_parent && ft_node_compressed(old_parent)) {
			struct cds_ft_compressed_node *cn =
				ft_compressed_node_ptr(old_parent);
			struct cds_ft_metadata *cn_meta =
				cds_ft_item_to_metadata(
					(struct cds_ft_inode *) cn);

			if (cn_meta->skip_slot &&
			    ft_node_skip_compressed(*cn_meta->skip_slot))
				rcu_assign_pointer(*cn_meta->skip_slot,
					ft_skip_compressed_flag(
						new_node_flag, cn->len));
		}
	}
	/*
	 * Redirect skip_slot for children that are skip pointers.
	 *
	 * Children were copied value-for-value from the old node to the
	 * new node.  Any child that is a skip pointer still has its
	 * compressed node's skip_slot pointing to the old node's child
	 * slot—which will be freed after a grace period.  Redirect
	 * skip_slot to the corresponding slot in the new node.
	 */
	if (ft_group_skip_compressed(ft->group)) {
		switch (new_type->type_class) {
		case FT_LINEAR:
		case FT_LINEAR_WIDE:
		{
			uint8_t nc = ft_linear_node_get_nr_child(new_type,
					new_node);
			unsigned int i;

			for (i = 0; i < nc; i++) {
				struct cds_ft_inode_flag *iter;
				uint8_t v;

				ft_linear_node_get_ith_pos(new_type,
						new_node, i, &v, &iter);
				if (iter && ft_node_skip_compressed(iter)) {
					struct cds_ft_inode_flag **slot;

					ft_node_get_nth_skip(new_node_flag,
							&slot, v);
					ft_skip_to_compressed_meta(iter)->skip_slot =
						slot;
				}
			}
			break;
		}
		case FT_POOL:
		{
			unsigned int pool_nr;

			for (pool_nr = 0;
			     pool_nr < (1U << new_type->nr_pool_order);
			     pool_nr++) {
				struct cds_ft_inode *pool =
					ft_pool_node_get_ith_pool(new_type,
						new_node, pool_nr);
				uint8_t nc = ft_linear_node_get_nr_child(
						new_type, pool);
				unsigned int j;

				for (j = 0; j < nc; j++) {
					struct cds_ft_inode_flag *iter;
					uint8_t v;

					ft_linear_node_get_ith_pos(new_type,
							pool, j, &v, &iter);
					if (iter &&
					    ft_node_skip_compressed(iter)) {
						struct cds_ft_inode_flag **slot;

						ft_node_get_nth_skip(
							new_node_flag,
							&slot, v);
						ft_skip_to_compressed_meta(
							iter)->skip_slot =
							slot;
					}
				}
			}
			break;
		}
		case FT_PIGEON:
		{
			unsigned int i;

			for (i = 0; i < FT_ENTRY_PER_NODE; i++) {
				struct cds_ft_inode_flag *iter;

				iter = ft_pigeon_node_get_ith_pos(new_type,
						new_node, i);
				if (iter && ft_node_skip_compressed(iter)) {
					struct cds_ft_inode_flag **slot;

					ft_node_get_nth_skip(new_node_flag,
							&slot, i);
					ft_skip_to_compressed_meta(
						iter)->skip_slot = slot;
				}
			}
			break;
		}
		default:
			break;
		}
	}

	/* Return pointer to new recompacted node through old_node_flag_ptr */
	*old_node_flag_ptr = new_node_flag;
	if (old_node && old_node_ret)
		*old_node_ret = old_node;

	ret = 0;
end:
	return ret;

fallback_toosmall:
	/* fallback if next pool is too small */
	free_cds_ft_node(ft, new_node);

	switch (mode) {
	case FT_RECOMPACT_ADD_SAME:
		/*
		 * FT_RECOMPACT_ADD_SAME is only triggered if a linear
		 * node within a pool has unused entries. It should
		 * therefore _never_ be too small.
		 */
		assert(0);

		/* Fall-through */
	case FT_RECOMPACT_ADD_NEXT:
	{
		const struct cds_ft_type *next_type;

		/*
		 * Recompaction attempt on add failed. Should only
		 * happen if target node type is pool. Caused by
		 * hard-to-split distribution. Recompact using the next
		 * distribution size.
		 */
		assert(new_type->type_class == FT_POOL);
		next_type = &ft_types[new_type_index + 1];
		/*
		 * Try going to the next pool size if our population
		 * fits within its range. This is not flagged as a
		 * fallback.
		 */
		if (metadata->nr_child + 1 >= next_type->min_child
				&& metadata->nr_child + 1 <= next_type->max_child) {
			new_type_index++;
			goto retry;
		} else {
			new_type_index++;
			dbg_printf("Add fallback to type %d\n", new_type_index);
			if (ft_debug_counters())
				uatomic_inc(&ft->nr_fallback);
			fallback = 1;
			goto retry;
		}
		break;
	}
	case FT_RECOMPACT_DEL:
		/*
		 * Recompaction attempt on delete failed. Should only
		 * happen if target node type is pool. This is caused by
		 * a hard-to-split distribution. Recompact on same node
		 * size, but flag current node as "fallback" to ensure
		 * we don't attempt recompaction before some activity
		 * has reshuffled our node.
		 */
		assert(new_type->type_class == FT_POOL);
		new_type_index = old_type_index;
		dbg_printf("Delete fallback keeping type %d\n", new_type_index);
		uatomic_inc(&ft->nr_fallback);
		fallback = 1;
		goto retry;
	default:
		assert(0);
		return -EINVAL;
	}

	/*
	 * Last resort fallback: pigeon.
	 */
	new_type_index = (1UL << FT_TYPE_BITS) - 1;
	dbg_printf("Fallback to type %d\n", new_type_index);
	uatomic_inc(&ft->nr_fallback);
	fallback = 1;
	goto retry;
}

/*
 * Return 0 on success or negative error value on error.
 */
static
int ft_node_set_nth(struct cds_ft *ft,
		struct cds_ft_inode_flag **node_flag, uint8_t n,
		struct cds_ft_inode_flag *child_node_flag,
		struct cds_ft_inode **old_node_ret,
		struct cds_ft_metadata *metadata)
{
	int ret;
	unsigned int type_index;
	const struct cds_ft_type *type;
	struct cds_ft_inode *node;

	dbg_printf("ft_node_set_nth for n=%u, node %p\n", (unsigned int) n, ft_node_ptr(*node_flag));

	node = ft_node_ptr(*node_flag);
	type_index = ft_node_type(*node_flag);
	type = &ft_types[type_index];
	ret = _ft_node_set_nth(type, node, *node_flag, metadata, n, child_node_flag);
	switch (ret) {
	case -ENOSPC:
		/* Not enough space in node, need to recompact to next type. */
		ret = ft_node_recompact(FT_RECOMPACT_ADD_NEXT, ft, type_index, type, node,
					metadata, node_flag, n, child_node_flag, NULL,
					old_node_ret, false);
		break;
	case -ERANGE:
		/* Node needs to be recompacted. */
		ret = ft_node_recompact(FT_RECOMPACT_ADD_SAME, ft, type_index, type, node,
					metadata, node_flag, n, child_node_flag, NULL,
					old_node_ret, false);
		break;
	}
	return ret;
}

/*
 * Return 0 on success or negative error value on error.
 */
static
int ft_node_replace_ptr(struct cds_ft *ft,
		struct cds_ft_inode_flag **node_flag_ptr,		/* Pointer to location to nullify */
		struct cds_ft_inode_flag **parent_node_flag_ptr,	/* Address of parent ptr in its parent */
		struct cds_ft_inode **old_node_ret,
		struct cds_ft_metadata *metadata,			/* of parent */
		uint8_t n,
		struct cds_ft_inode_flag *newptr,
		bool is_root)
{
	int ret;
	unsigned int type_index;
	const struct cds_ft_type *type;
	struct cds_ft_inode *node;

	dbg_printf("ft_node_replace_ptr for node %p, target ptr %p\n",
		ft_node_ptr(*parent_node_flag_ptr), node_flag_ptr);

	node = ft_node_ptr(*parent_node_flag_ptr);
	type_index = ft_node_type(*parent_node_flag_ptr);
	type = &ft_types[type_index];
	ret = _ft_node_replace_ptr(type, node, *parent_node_flag_ptr, metadata, node_flag_ptr, n, newptr);
	if (ret == -EFBIG) {
		assert(!newptr);
		/* Should try recompaction. */
		ret = ft_node_recompact(FT_RECOMPACT_DEL, ft, type_index, type, node,
				metadata, parent_node_flag_ptr, n, NULL,
				node_flag_ptr, old_node_ret, is_root);
	}
	return ret;
}

enum ft_prefix_tracking {
	FT_PREFIX_TRACK_NONE,		/* No prefix tracking. */
	FT_PREFIX_TRACK_PARTIAL,	/* Track closest ancestor with external nodes. */
	FT_PREFIX_TRACK_LONGEST,	/* Track deepest match, even internal-only. */
};

/*
 * Sentinel value indicating that prefix tracking never recorded a
 * match. Used by FT_PREFIX_TRACK_LONGEST to distinguish "empty trie"
 * from "matched at root with no external nodes" (both have
 * match_node == NULL, but the latter sets match_len = 0).
 */
#define FT_MATCH_LEN_NONE	SIZE_MAX

/*
 * Handle a compressed node during exact lookup descent.
 *
 * Compares key bytes against the compressed path, tracks
 * partial/longest match if requested, advances key/index/node_flag
 * past the compressed path, and fills iter_path entries.
 *
 * Returns FT_COMPRESSED_CONTINUE to continue the loop,
 * FT_COMPRESSED_BREAK to break, or FT_COMPRESSED_END to jump to
 * the function's end label (with *status_ret and *found_ret set).
 */
static inline_lookup
enum ft_compressed_action ft_lookup_compressed(struct cds_ft_inode_flag **node_flag_p,
		const uint8_t **key_p, unsigned int *i_p,
		unsigned int key_depth,
		struct cds_ft_iter *iter, size_t *iter_path_len_p,
		bool track, bool track_longest,
		size_t *match_len_p, struct cds_ft_node **match_node_p,
		struct cds_ft_node **found_ret,
		enum cds_ft_status *status_ret,
		bool candidate)
{
	struct cds_ft_inode_flag *node_flag = *node_flag_p;
	const uint8_t *key = *key_p;
	unsigned int i = *i_p;
	struct cds_ft_compressed_node *cn = ft_compressed_node_ptr(node_flag);
	int remaining_key = key_depth - 1 - i;
	int cmp_len = cn->len < remaining_key ? cn->len : remaining_key;

	/* Check external_nodes at the compressed node's depth. */
	if (track) {
		struct cds_ft_metadata *cn_meta =
			cds_ft_item_to_metadata((struct cds_ft_inode *) cn);
		struct cds_ft_node *ext =
			ft_dereference_prefetch_external(cn_meta->external_nodes);

		if (ext || track_longest) {
			*match_len_p = i;
			*match_node_p = ext;
		}
	}

	/*
	 * In candidate mode, skip key comparison — just advance past
	 * the compressed path.  The caller verifies the key at the leaf.
	 */
	if (!candidate) {
		if (track_longest) {
			unsigned int mpos;
			int cmp = ft_key_cmp_ordinals(key, cn->key_bytes,
					cmp_len, remaining_key, false, &mpos);

			if (cmp != 0) {
				*match_len_p = i + mpos;
				*match_node_p = NULL;
				*status_ret = CDS_FT_STATUS_NOT_FOUND;
				return FT_COMPRESSED_END;
			}
			*match_len_p = i + cmp_len;
			*match_node_p = NULL;
		} else {
			if (ft_key_cmp_ordinals(key, cn->key_bytes,
					cmp_len, remaining_key, false, NULL) != 0) {
				*status_ret = CDS_FT_STATUS_NOT_FOUND;
				return FT_COMPRESSED_END;
			}
		}
	}
	if (cn->len > remaining_key) {
		struct cds_ft_metadata *cn_meta =
			cds_ft_item_to_metadata_fast(
				(struct cds_ft_inode *) cn,
				ft_compressed_order(cn->len));

		*found_ret = ft_dereference_prefetch_external(cn_meta->external_nodes);
		*status_ret = *found_ret ? CDS_FT_STATUS_OK :
				CDS_FT_STATUS_NOT_FOUND;
		if (track && (*found_ret || track_longest)) {
			*match_len_p = i;
			*match_node_p = *found_ret;
		}
		return FT_COMPRESSED_END;
	}

	/* Advance past the compressed path. */
	key += cn->len;
	if (iter) {
		int k;

		for (k = 1; k <= cn->len; k++)
			iter_path_node(iter)[i + k] =
				(struct cds_ft_inode_flag *)
				ft_compressed_node_flag(cn);
	}
	i += cn->len;
	node_flag = ft_dereference_acquire_prefetch(cn->child);
	if (!ft_node_ptr(node_flag)) {
		*status_ret = CDS_FT_STATUS_NOT_FOUND;
		return FT_COMPRESSED_END;
	}
	if (iter) {
		iter_path_node(iter)[i] = node_flag;
		*iter_path_len_p = i + 1;
	}

	*node_flag_p = node_flag;
	*key_p = key;
	*i_p = i;

	if (i >= key_depth)
		return FT_COMPRESSED_BREAK;

	/*
	 * External child before end of key: record for partial
	 * tracking, set NOT_FOUND, and tell the caller to end.
	 */
	if (i < key_depth - 1 && ft_node_external(node_flag)) {
		if (track) {
			*match_len_p = i;
			*match_node_p = (struct cds_ft_node *) node_flag;
		}
		*status_ret = CDS_FT_STATUS_NOT_FOUND;
		return FT_COMPRESSED_END;
	}

	/*
	 * Track prefix match at the child node (the node after the
	 * compressed path) so callers that skip the normal tracking
	 * code via continue don't miss it.
	 */
	if (track && i < key_depth - 1 && !ft_node_external(node_flag)) {
		struct cds_ft_metadata *metadata =
			cds_ft_item_to_metadata(ft_node_ptr(node_flag));
		struct cds_ft_node *ext =
			ft_dereference_prefetch_external(metadata->external_nodes);

		if (ext || track_longest) {
			*match_len_p = i;
			*match_node_p = ext;
		}
	}

	return FT_COMPRESSED_CONTINUE;
}

#ifdef FEATURE_FT_COLLAPSE
/*
 * Collapsed node lookup helper for do_cds_ft_lookup.
 *
 * Scans the collapsed node's entries for a suffix that matches the
 * remaining lookup key.  On match, advances key/index/node_flag past
 * the matched suffix and returns CONTINUE or BREAK.  On no match,
 * returns END with NOT_FOUND status.
 *
 * The caller decrements i before calling (same as compressed root
 * handling) when the collapsed node is encountered at the current
 * position rather than as a child of ft_node_get_nth.
 */
static inline_lookup
enum ft_compressed_action ft_lookup_collapsed(struct cds_ft_inode_flag **node_flag_p,
		const uint8_t **key_p, unsigned int *i_p,
		unsigned int key_depth,
		struct cds_ft_iter *iter, size_t *iter_path_len_p,
		bool track, bool track_longest,
		size_t *match_len_p, struct cds_ft_node **match_node_p,
		struct cds_ft_node __attribute__((unused)) **found_ret,
		enum cds_ft_status *status_ret)
{
	struct cds_ft_inode_flag *node_flag = *node_flag_p;
	const uint8_t *key = *key_p;
	unsigned int i = *i_p;
	struct cds_ft_collapsed_node *cn = ft_collapsed_node_ptr(node_flag);
	unsigned int nr_e = ft_collapsed_nr_entries(cn);
	unsigned int scan_sz = ft_collapsed_scan_zone_size(nr_e);
	unsigned int off_mask = ft_collapsed_offset_mask(nr_e);
	struct cds_ft_inode_flag **ptrs = ft_collapsed_ptrs(cn, nr_e);
	unsigned int remaining_key = key_depth - 1 - i;
	unsigned int e;

	/*
	 * Check external_nodes only when needed: for prefix
	 * tracking or when the key ends at this depth.  Avoid
	 * loading the metadata cache line during normal lookups
	 * that traverse through the collapsed node.
	 */
	if (remaining_key == 0)
		return FT_COMPRESSED_BREAK;
	if (track) {
		struct cds_ft_metadata *cn_meta =
			cds_ft_item_to_metadata((struct cds_ft_inode *) cn);
		struct cds_ft_node *ext =
			ft_dereference_prefetch_external(cn_meta->external_nodes);

		if (ext || track_longest) {
			*match_len_p = i;
			*match_node_p = ext;
		}
	}

	/* Scan entries for a matching suffix. */
	for (e = 0; e < ft_collapsed_count(nr_e); e++) {
		unsigned int slen, start, end;
		uint8_t *suffix;
		bool match;
		uint8_t data_e = ft_collapsed_load_data(cn, e);

		if (data_e & (off_mask ^ 0xFF))
			continue;	/* tombstone (only for non-256B zones) */
		start = data_e & off_mask;
		/*
		 * Previous entry's data byte is immutable once published,
		 * so loading it without caching is safe.
		 */
		end = (e == 0) ? scan_sz : (ft_collapsed_load_data(cn, e - 1) & off_mask);
		slen = end - start;
		if (slen > remaining_key)
			continue;
		suffix = ((uint8_t *) cn) + start;
		match = (ft_key_cmp_ordinals(key, suffix, slen, slen, false, NULL) == 0);
		if (!match)
			continue;

		/* Match found. Advance past the suffix. */
		key += slen;
		if (iter) {
			unsigned int k;

			for (k = 1; k <= slen; k++)
				iter_path_node(iter)[i + k] =
					(struct cds_ft_inode_flag *)
					ft_collapsed_node_flag(cn);
		}
		i += slen;
		node_flag = ft_dereference_acquire_prefetch(ptrs[e]);
		if (!ft_node_ptr(node_flag)) {
			*status_ret = CDS_FT_STATUS_NOT_FOUND;
			return FT_COMPRESSED_END;
		}
		if (ft_node_skip_compressed(node_flag))
			node_flag = ft_compressed_node_flag(
				ft_skip_to_compressed(node_flag));
		if (iter) {
			iter_path_node(iter)[i] = node_flag;
			*iter_path_len_p = i + 1;
		}

		*node_flag_p = node_flag;
		*key_p = key;
		*i_p = i;

		if (i >= key_depth)
			return FT_COMPRESSED_BREAK;

		/* External child before end of key. */
		if (i < key_depth - 1 && ft_node_external(node_flag)) {
			if (track) {
				*match_len_p = i;
				*match_node_p = (struct cds_ft_node *) node_flag;
			}
			*status_ret = CDS_FT_STATUS_NOT_FOUND;
			return FT_COMPRESSED_END;
		}

		/* Track prefix match at child node. */
		if (track && i < key_depth - 1 && !ft_node_external(node_flag)) {
			struct cds_ft_metadata *metadata =
				cds_ft_item_to_metadata(ft_node_ptr(node_flag));
			struct cds_ft_node *ext =
				ft_dereference_prefetch_external(metadata->external_nodes);

			if (ext || track_longest) {
				*match_len_p = i;
				*match_node_p = ext;
			}
		}

		return FT_COMPRESSED_CONTINUE;
	}

	/* No matching suffix found. */
	*status_ret = CDS_FT_STATUS_NOT_FOUND;
	return FT_COMPRESSED_END;
}
#endif /* FEATURE_FT_COLLAPSE */

/*
 * Simple compressed node traversal for read-side loops (replace,
 * count_keys_prefix).  Matches the key against the compressed path,
 * advances key/index/node_flag, and returns the loop action.
 *
 * On NOT_FOUND (mismatch or key shorter), returns FT_COMPRESSED_END
 * with *not_found set to true.
 * On full match with external child, returns FT_COMPRESSED_BREAK.
 * On full match with non-external child, returns FT_COMPRESSED_CONTINUE.
 */
static inline
enum ft_compressed_action ft_traverse_compressed(
		struct cds_ft_inode_flag **node_flag_p,
		struct cds_ft_inode_flag ***node_flag_ptr_p,
		const uint8_t **key_p, unsigned int *i_p,
		unsigned int key_depth, bool *not_found)
{
	struct cds_ft_inode_flag *node_flag = *node_flag_p;
	struct cds_ft_compressed_node *cn = ft_compressed_node_ptr(node_flag);
	const uint8_t *key = *key_p;
	unsigned int i = *i_p;
	unsigned int remaining = key_depth - i;
	unsigned int j;

	if (cn->len > remaining) {
		*not_found = true;
		return FT_COMPRESSED_END;
	}
	j = ft_match_compressed_key(key, cn, cn->len);
	if (j < cn->len) {
		*not_found = true;
		return FT_COMPRESSED_END;
	}
	*key_p = key + cn->len;
	*i_p = i + cn->len - 1; /* -1: for loop increments */
	*node_flag_p = cn->child;
	if (node_flag_ptr_p)
		*node_flag_ptr_p = &cn->child;
	if (!ft_node_ptr(cn->child)) {
		*not_found = true;
		return FT_COMPRESSED_END;
	}
	if (ft_node_external(cn->child))
		return FT_COMPRESSED_BREAK;
	return FT_COMPRESSED_CONTINUE;
}

#ifdef FEATURE_FT_COLLAPSE
/*
 * Simple collapsed node traversal for read-side descent loops
 * (replace, count_keys_prefix).  Scans entries for a suffix that
 * matches the remaining key, advances key/index/node_flag, and
 * returns the loop action.
 */
static
enum ft_compressed_action ft_traverse_collapsed(struct cds_ft_inode_flag **node_flag_p,
		struct cds_ft_inode_flag ***node_flag_ptr_p,
		const uint8_t **key_p, unsigned int *i_p,
		unsigned int key_depth, bool *not_found)
{
	struct cds_ft_inode_flag *node_flag = *node_flag_p;
	struct cds_ft_collapsed_node *cn = ft_collapsed_node_ptr(node_flag);
	const uint8_t *key = *key_p;
	unsigned int i = *i_p;
	unsigned int remaining = key_depth - i;
	unsigned int e;
	unsigned int nr_e = ft_collapsed_nr_entries(cn);
	struct cds_ft_inode_flag **ptrs = ft_collapsed_ptrs(cn, nr_e);

	for (e = 0; e < ft_collapsed_count(nr_e); e++) {
		unsigned int slen;
		uint8_t *suffix;
		bool match;
		uint8_t data_e = ft_collapsed_load_data(cn, e);

		if (ft_collapsed_entry_dead(data_e, nr_e))
			continue;
		slen = ft_collapsed_suffix_len(cn, data_e, e, nr_e);
		if (slen > remaining)
			continue;
		suffix = ft_collapsed_suffix(cn, data_e, nr_e);
		match = (ft_key_cmp_ordinals(key, suffix, slen, slen, false, NULL) == 0);
		if (!match)
			continue;

		*key_p = key + slen;
		*i_p = i + slen - 1;
		{
			struct cds_ft_inode_flag *child =
				ft_dereference_acquire_prefetch(ptrs[e]);
			if (ft_node_skip_compressed(child))
				child = ft_compressed_node_flag(
					ft_skip_to_compressed(child));
			*node_flag_p = child;
		}
		if (node_flag_ptr_p)
			*node_flag_ptr_p = &ptrs[e];
		if (!ft_node_ptr(ptrs[e])) {
			*not_found = true;
			return FT_COMPRESSED_END;
		}
		if (ft_node_external(ptrs[e]))
			return FT_COMPRESSED_BREAK;
		return FT_COMPRESSED_CONTINUE;
	}
	*not_found = true;
	return FT_COMPRESSED_END;
}
#endif /* FEATURE_FT_COLLAPSE */

/*
 * @candidate: when true, skip key comparison at compressed nodes
 * during traversal (patricia-like mode).  The returned node is a
 * candidate that must be verified by the caller against their
 * stored key.  Constant-folded at each call site.
 */
static inline_lookup
enum cds_ft_status do_cds_ft_lookup(struct cds_ft *ft,
		const uint8_t *key, size_t _key_len,
		struct cds_ft_node **result_node,
		struct cds_ft_iter *iter,
		enum ft_prefix_tracking tracking,
		size_t *tracking_match_len,
		struct cds_ft_node **tracking_match_node,
		bool candidate)
{
	size_t key_len = ft_key_len(ft, _key_len);
	struct cds_ft_inode_flag *node_flag;
	struct cds_ft_node *found = NULL;
	unsigned int key_depth, i;
	enum cds_ft_status status;
	size_t iter_path_len = 0;
	bool track = (tracking != FT_PREFIX_TRACK_NONE);
	bool track_longest = (tracking == FT_PREFIX_TRACK_LONGEST);
	bool skip_compressed = ft_group_skip_compressed(ft->group);
	size_t match_len = track_longest ? FT_MATCH_LEN_NONE : 0;
	struct cds_ft_node *match_node = NULL;

	CDS_FT_ASSERT_RCU_READ_LOCKED(ft);

	if (!valid_key_len(ft, key_len)) {
		status = CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
		goto end;
	}
	key_depth = key_len + 1;
	node_flag = ft_dereference_prefetch(ft->root);

	if (iter) {
		iter_debug_path_snapshot(iter);
		iter_path_node(iter)[0] = node_flag;
		iter_path_len = 1;
	}

	/*
	 * Root is always internal. For key_len == 0, return the root's
	 * metadata external_nodes (NIL-key entries).
	 */
	if (!key_len) {
		const struct cds_ft_type *type = &ft_types[ft_node_type(node_flag)];
		struct cds_ft_metadata *metadata = cds_ft_item_to_metadata_fast(ft_node_ptr(node_flag),
							type->order);
		found = ft_dereference_prefetch_external(metadata->external_nodes);
		status = found ? CDS_FT_STATUS_OK : CDS_FT_STATUS_NOT_FOUND;
		if (track) {
			match_len = 0;
			match_node = found;
		}
		goto end;
	}

	/*
	 * Consider external node in root metadata as possible match.
	 * For longest-match tracking, record the root position even
	 * when there are no external nodes.
	 */
	if (track) {
		const struct cds_ft_type *type = &ft_types[ft_node_type(node_flag)];
		struct cds_ft_metadata *metadata = cds_ft_item_to_metadata_fast(ft_node_ptr(node_flag),
							type->order);
		struct cds_ft_node *external_nodes = ft_dereference_prefetch_external(metadata->external_nodes);

		if (external_nodes || track_longest) {
			match_len = 0;
			match_node = external_nodes;
		}
	}

	for (i = 1; i < key_depth; i++) {
		uint8_t iter_key;

		/*
		 * Compressed node at current position (e.g. compressed
		 * root or compressed child from ft_node_get_nth).
		 */
		/*
		 * Non-internal nodes at this position (compressed or
		 * collapsed) need special handling.  Internal (bit 0
		 * set) is the common case — skip directly to the
		 * key dispatch below.
		 */
		/*
		 * Skip-compressed pointer at loop top: handles skip
		 * pointers returned by collapsed entry children or
		 * by ft_node_get_nth in the previous iteration.
		 *
		 * Non-candidate: convert to compressed flag so the
		 * compressed handler below processes it with full
		 * key comparison.
		 *
		 * Candidate: resolve the skip (advance past the
		 * compressed path without comparison).
		 */
		if (skip_compressed &&
		    caa_unlikely(ft_node_skip_compressed(node_flag))) {
			if (!candidate) {
				node_flag = ft_compressed_node_flag(
					ft_skip_to_compressed(node_flag));
			} else {
				unsigned int skip = ft_skip_len(node_flag);
				int remaining = key_depth - 1 - i;

				if ((int) skip > remaining) {
					status = CDS_FT_STATUS_NOT_FOUND;
					goto end;
				}
				key += skip;
				i += skip;
				node_flag = ft_skip_child_ptr(node_flag);
				if (iter) {
					iter_path_node(iter)[i] = node_flag;
					iter_path_len = i + 1;
				}
			}
		}
		/*
		 * Non-internal nodes need special handling.
		 * Internal (bit 0 set) is the common case.
		 *
		 * This single check handles both:
		 * - compressed/collapsed from previous iteration's
		 *   get_nth result
		 * - compressed/collapsed/external from compressed
		 *   or collapsed handler output
		 */
		if (caa_unlikely(!ft_node_internal(node_flag))) {
			if (ft_node_compressed(node_flag)) {
				enum ft_compressed_action act;

				i--;
				act = ft_lookup_compressed(&node_flag, &key, &i,
					key_depth, iter, &iter_path_len,
					track, track_longest,
					&match_len, &match_node, &found, &status,
					candidate);
				if (act == FT_COMPRESSED_END)
					goto end;
				if (act == FT_COMPRESSED_BREAK)
					break;
				continue;
			}
			if (ft_node_collapsed(node_flag)) {
				enum ft_compressed_action act;

				i--;
				act = ft_lookup_collapsed(&node_flag, &key, &i,
					key_depth, iter, &iter_path_len,
					track, track_longest,
					&match_len, &match_node, &found, &status);
				if (act == FT_COMPRESSED_END)
					goto end;
				if (act == FT_COMPRESSED_BREAK)
					break;
				continue;
			}
			/*
			 * External or NULL at loop top: the previous
			 * get_nth returned a non-internal, non-compressed,
			 * non-collapsed child.  This shouldn't happen in
			 * the normal flow (external is caught post-get_nth).
			 * Handle as not-found for robustness.
			 */
			status = CDS_FT_STATUS_NOT_FOUND;
			goto end;
		}

		iter_key = *(key++);
		node_flag = (candidate || !skip_compressed) ?
			ft_node_get_nth_skip(node_flag, NULL, iter_key) :
			ft_node_get_nth(node_flag, NULL, iter_key);
		dbg_printf("cds_ft_lookup iter key lookup %u finds node_flag %p\n",
				(unsigned int) iter_key, node_flag);
		if (!ft_node_ptr(node_flag)) {
			status = CDS_FT_STATUS_NOT_FOUND;
			goto end;
		}
		/*
		 * Skip-compressed pointer from child slot.
		 *
		 * Non-candidate: convert to compressed flag and
		 * continue so the compressed handler at the loop
		 * top processes it (key comparison, external_nodes
		 * check, etc.).
		 *
		 * Candidate: resolve the skip (advance past the
		 * compressed path without comparison).
		 */
		if (skip_compressed && caa_unlikely(ft_node_skip_compressed(node_flag))) {
			if (!candidate) {
				node_flag = ft_compressed_node_flag(
					ft_skip_to_compressed(node_flag));
				continue;
			}
			{
				unsigned int skip = ft_skip_len(node_flag);
				int remaining = key_depth - 1 - i;

				if ((int) skip > remaining) {
					status = CDS_FT_STATUS_NOT_FOUND;
					goto end;
				}
				key += skip;
				i += skip;
				node_flag = ft_skip_child_ptr(node_flag);
			}
		}
		if (iter) {
			iter_path_node(iter)[i] = node_flag;
			iter_path_len = i + 1;
		}
		/*
		 * External child before end of key: the key is
		 * longer than this branch.
		 */
		if (caa_unlikely(ft_node_external(node_flag)) &&
		    i < key_depth - 1) {
			if (track) {
				match_len = i;
				match_node = (struct cds_ft_node *) node_flag;
			}
			status = CDS_FT_STATUS_NOT_FOUND;
			goto end;
		}
		/*
		 * Track prefix match on the child node.  Only
		 * evaluated when tracking is requested — dead-code
		 * eliminated for cds_ft_lookup_key (track=false).
		 * The ft_node_internal check here is only reached
		 * by track=true callers; for track=false the
		 * compiler eliminates the entire block, leaving
		 * just one ft_node_internal check per iteration
		 * (at the loop top).
		 */
		if (track && caa_likely(ft_node_internal(node_flag))
		    && i < key_depth - 1) {
			const struct cds_ft_type *type = &ft_types[ft_node_type(node_flag)];
			struct cds_ft_metadata *metadata = cds_ft_item_to_metadata_fast(
					ft_node_ptr(node_flag), type->order);
			struct cds_ft_node *external_nodes = ft_dereference_prefetch_external(metadata->external_nodes);

			if (external_nodes || track_longest) {
				match_len = i;
				match_node = external_nodes;
			}
		}
	}

	/*
	 * Reached key_depth, check for terminal node: either external
	 * nodes or internal/compressed node associated with external nodes.
	 */
	if (ft_node_internal(node_flag)) {
		const struct cds_ft_type *type = &ft_types[ft_node_type(node_flag)];
		struct cds_ft_metadata *metadata = cds_ft_item_to_metadata_fast(ft_node_ptr(node_flag),
							type->order);
		found = ft_dereference_prefetch_external(metadata->external_nodes);
		status = found ? CDS_FT_STATUS_OK : CDS_FT_STATUS_NOT_FOUND;
		if (track && (found || track_longest)) {
			match_len = key_len;
			match_node = found;
		}
	} else if (ft_node_compressed(node_flag)) {
		struct cds_ft_compressed_node *cn =
			ft_compressed_node_ptr(node_flag);
		struct cds_ft_metadata *metadata =
			cds_ft_item_to_metadata_fast(
				(struct cds_ft_inode *) cn,
				ft_compressed_order(cn->len));
		found = ft_dereference_prefetch_external(metadata->external_nodes);
		status = found ? CDS_FT_STATUS_OK : CDS_FT_STATUS_NOT_FOUND;
		if (track && (found || track_longest)) {
			match_len = key_len;
			match_node = found;
		}
	} else if (ft_node_collapsed(node_flag)) {
		struct cds_ft_metadata *metadata =
			cds_ft_item_to_metadata_fast(
				ft_node_ptr(node_flag),
				cds_ft_item_order(
					(struct cds_ft_inode *) ft_collapsed_node_ptr(node_flag)));
		found = ft_dereference_prefetch_external(metadata->external_nodes);
		status = found ? CDS_FT_STATUS_OK : CDS_FT_STATUS_NOT_FOUND;
		if (track && (found || track_longest)) {
			match_len = key_len;
			match_node = found;
		}
	} else {
		found = (struct cds_ft_node *) node_flag;
		status = CDS_FT_STATUS_OK;
		if (track) {
			match_len = key_len;
			match_node = found;
		}
	}

end:
	if (result_node)
		*result_node = found;
	if (iter) {
		iter->node = found;
		iter->status = status;
		iter->path_len = iter_path_len;
		/*
		 * The path is valid for backtracking when we
		 * successfully descended into the trie, even if the
		 * exact key was not found.
		 */
		iter->path_valid = (status == CDS_FT_STATUS_OK);
		iter_debug_path_update(iter);
		iter_auto_invalidate_path(iter);
	}
	if (track) {
		*tracking_match_len = match_len;
		*tracking_match_node = match_node;
	}
	return status;
}

enum cds_ft_status cds_ft_lookup_key(struct cds_ft *ft,
		const uint8_t *key, size_t _key_len,
		struct cds_ft_node **result_node)
{
	size_t key_len = ft_key_len(ft, _key_len);
	uint8_t ordinals[FT_MAX_KEY_LEN];

	ft_key_to_ordinals(ordinals, key, key_len, &ft->group->key_map);
	return do_cds_ft_lookup(ft, ordinals, key_len, result_node, NULL,
				FT_PREFIX_TRACK_NONE, NULL, NULL, false);
}

/*
 * cds_ft_lookup_candidate_key - Fast candidate lookup.
 *
 * Skips key comparison at compressed nodes during traversal,
 * returning a candidate node that may not be an exact match.
 * The caller MUST verify the returned node's key matches the
 * lookup key.  If it does not match, the key is not in the trie.
 *
 * This is faster than cds_ft_lookup_key for workloads with long
 * compressed paths (e.g. reverse DNS, file paths) because it
 * eliminates per-node key comparisons, doing a single verification
 * at the end instead.
 */
enum cds_ft_status cds_ft_lookup_candidate_key(struct cds_ft *ft,
		const uint8_t *key, size_t _key_len,
		struct cds_ft_node **result_node)
{
	size_t key_len = ft_key_len(ft, _key_len);
	uint8_t ordinals[FT_MAX_KEY_LEN];

	ft_key_to_ordinals(ordinals, key, key_len, &ft->group->key_map);
	return do_cds_ft_lookup(ft, ordinals, key_len, result_node, NULL,
				FT_PREFIX_TRACK_NONE, NULL, NULL, true);
}

enum cds_ft_status cds_ft_lookup(struct cds_ft *ft,
		struct cds_ft_iter *iter)
{
	return do_cds_ft_lookup(ft, iter_key(iter), iter->key_len, NULL, iter,
				FT_PREFIX_TRACK_NONE, NULL, NULL, false);
}

enum cds_ft_status cds_ft_lookup_partial_key(struct cds_ft *ft,
		const uint8_t *key, size_t _key_len, size_t *match_len,
		struct cds_ft_node **result_node)
{
	struct cds_ft_node *partial_node = NULL;
	size_t partial_len = 0;
	size_t key_len = ft_key_len(ft, _key_len);
	uint8_t ordinals[FT_MAX_KEY_LEN];

	ft_key_to_ordinals(ordinals, key, key_len, &ft->group->key_map);
	do_cds_ft_lookup(ft, ordinals, key_len, NULL, NULL,
			 FT_PREFIX_TRACK_PARTIAL, &partial_len, &partial_node,
			 false);

	*match_len = partial_len;
	*result_node = partial_node;
	return partial_node ? CDS_FT_STATUS_OK : CDS_FT_STATUS_NOT_FOUND;
}

enum cds_ft_status cds_ft_lookup_partial(struct cds_ft *ft,
		struct cds_ft_iter *iter)
{
	struct cds_ft_node *partial_node = NULL;
	size_t partial_len = 0;

	/*
	 * Perform the full lookup (populating the iterator path for
	 * backtracking) while simultaneously tracking the closest
	 * ancestor with external nodes for partial-match semantics.
	 */
	do_cds_ft_lookup(ft, iter_key(iter), iter->key_len, NULL, iter,
			 FT_PREFIX_TRACK_PARTIAL, &partial_len, &partial_node,
			 false);

	/*
	 * Override the iterator's node and status with the partial-match
	 * result.
	 */
	iter->node = partial_node;
	iter->key_len = partial_len;
	iter->path_len = partial_len + 1;
	iter->status = partial_node ? CDS_FT_STATUS_OK : CDS_FT_STATUS_NOT_FOUND;
	return iter->status;
}

enum cds_ft_status cds_ft_lookup_longest_match_key(struct cds_ft *ft,
		const uint8_t *key, size_t _key_len, size_t *match_len,
		struct cds_ft_node **result_node)
{
	struct cds_ft_node *match_node = NULL;
	size_t longest_len = 0;
	enum cds_ft_status ret;
	size_t key_len = ft_key_len(ft, _key_len);
	uint8_t ordinals[FT_MAX_KEY_LEN];

	ft_key_to_ordinals(ordinals, key, key_len, &ft->group->key_map);
	ret = do_cds_ft_lookup(ft, ordinals, key_len, NULL, NULL,
			       FT_PREFIX_TRACK_LONGEST, &longest_len, &match_node,
			       false);

	if (ret < 0) {
		*match_len = 0;
		*result_node = NULL;
		return ret;
	}
	if (longest_len == FT_MATCH_LEN_NONE) {
		*match_len = 0;
		*result_node = NULL;
		return CDS_FT_STATUS_NOT_FOUND;
	}
	*match_len = longest_len;
	*result_node = match_node;
	return match_node ? CDS_FT_STATUS_OK : CDS_FT_STATUS_INTERNAL_MATCH;
}

enum cds_ft_status cds_ft_lookup_longest_match(struct cds_ft *ft,
		struct cds_ft_iter *iter)
{
	struct cds_ft_node *match_node = NULL;
	size_t longest_len = 0;
	enum cds_ft_status ret;

	ret = do_cds_ft_lookup(ft, iter_key(iter), iter->key_len, NULL, iter,
			       FT_PREFIX_TRACK_LONGEST, &longest_len, &match_node,
			       false);

	if (ret < 0) {
		iter->node = NULL;
		iter->status = ret;
		goto end;
	}
	if (longest_len == FT_MATCH_LEN_NONE) {
		iter->node = NULL;
		iter->status = CDS_FT_STATUS_NOT_FOUND;
		goto end;
	}
	iter->node = match_node;
	iter->key_len = longest_len;
	iter->path_len = longest_len + 1;
	iter->status = match_node ? CDS_FT_STATUS_OK : CDS_FT_STATUS_INTERNAL_MATCH;
	iter->path_valid = true;
	iter_debug_path_snapshot(iter);
end:
	iter_auto_invalidate_path(iter);
	return iter->status;
}

/*
 * ft_collapsed_find_nearest: scan a collapsed node's entries for the
 * nearest live entry in @dir relative to entry @ref_entry.
 *
 * Compares each candidate's suffix against @ref_entry's suffix to
 * determine direction.  Skips @ref_entry itself, dead (tombstoned)
 * entries, and NULL-pointer entries.
 *
 * Returns the index of the nearest entry, or -1 if none found.
 */
static int ft_collapsed_find_nearest(
		struct cds_ft_collapsed_node *col,
		struct cds_ft_inode_flag **cptrs,
		unsigned int ref_entry,
		unsigned int nr_e,
		enum ft_direction dir)
{
	uint8_t ref_data = ft_collapsed_load_data(col, ref_entry);
	uint8_t *ref_suffix = ft_collapsed_suffix(col, ref_data, nr_e);
	unsigned int ref_slen = ft_collapsed_suffix_len(col, ref_data, ref_entry, nr_e);
	int best = -1;
	uint8_t best_data = 0;
	unsigned int e;

	for (e = 0; e < nr_e; e++) {
		uint8_t *suffix;
		unsigned int slen, mc;
		int r;
		uint8_t data_e = ft_collapsed_load_data(col, e);

		if (e == ref_entry)
			continue;
		if (ft_collapsed_entry_dead(data_e, nr_e))
			continue;
		if (!ft_node_ptr(cptrs[e]))
			continue;
		suffix = ft_collapsed_suffix(col, data_e, nr_e);
		slen = ft_collapsed_suffix_len(col, data_e, e, nr_e);

		mc = slen < ref_slen ? slen : ref_slen;
		r = ft_key_cmp_ordinals(suffix, ref_suffix, mc, mc, true, NULL);
		if (r == 0)
			r = (slen > ref_slen) ? 1 : (slen < ref_slen) ? -1 : 0;

		if ((dir == FT_RIGHT && r > 0) ||
		    (dir == FT_LEFT && r < 0)) {
			if (best < 0) {
				best = (int)e;
				best_data = data_e;
			} else {
				uint8_t *bs = ft_collapsed_suffix(col, best_data, nr_e);
				unsigned int bl = ft_collapsed_suffix_len(col, best_data, (unsigned)best, nr_e);
				unsigned int mc2 = bl < slen ? bl : slen;
				int r2 = ft_key_cmp_ordinals(suffix, bs, mc2, mc2, true, NULL);

				if ((dir == FT_RIGHT && (r2 < 0 || (r2 == 0 && slen < bl))) ||
				    (dir == FT_LEFT && (r2 > 0 || (r2 == 0 && slen > bl)))) {
					best = (int)e;
					best_data = data_e;
				}
			}
		}
	}
	return best;
}

/*
 * Iterator-based inequality lookup. The input key and key_len are read
 * from @iter (set via cds_ft_iter_set_key). On success the result key,
 * key length, node, status, and path are written back into @iter so that
 * subsequent iteration / backtracking calls can reuse the state.
 *
 * @limit overrides the key: FT_LOOKUP_LIMIT_FIRST uses key_len
 * prefix_len, FT_LOOKUP_LIMIT_LAST uses max_key_len.
 *
 * Prefix-scoped traversal: when iter->prefix_len > 0, the traversal
 * is confined to the subtree rooted at the prefix. Backtracking stops
 * at the prefix boundary instead of the root. LIMIT_FIRST finds the
 * smallest key within the prefix subtree. LIMIT_LAST descends using
 * the actual prefix key bytes followed by 0xFF to find the greatest
 * key within the prefix subtree.
 */
/*
 * Handle a compressed node during inequality descent.
 *
 * Matches key bytes against the compressed path (respecting the
 * limit mode), fills ordinal_key and iter_path, and determines
 * the action: continue descent, break, go up, or descend into
 * children.
 *
 * On mismatch, the direction relative to the lookup mode decides
 * whether to backtrack (GOING_UP) or descend into the compressed
 * subtree (DESCEND_CHILDREN).
 */
static
enum ft_compressed_action ft_inequality_compressed(struct cds_ft_inode_flag **node_flag_p,
		ssize_t *level_p, ssize_t key_depth,
		enum ft_lookup_inequality mode,
		enum ft_lookup_limit limit,
		const uint8_t **iter_key_p,
		const uint8_t *input_key,
		struct cds_ft_iter *iter,
		uint8_t *ordinal_key,
		bool *skip_eq_external_nodes_p)
{
	struct cds_ft_inode_flag *node_flag = *node_flag_p;
	struct cds_ft_compressed_node *cn = ft_compressed_node_ptr(node_flag);
	int level = *level_p;
	int remaining = key_depth - level;
	int cmp = cn->len < remaining ? cn->len : remaining;
	const uint8_t *cmp_key;
	uint8_t last_key_buf[FT_MAX_KEY_LEN];
	unsigned int mpos = 0;
	int cmp_result;
	int j;

	/* Build contiguous comparison key for this limit mode. */
	switch (limit) {
	case FT_LOOKUP_LIMIT_NONE:
		cmp_key = *iter_key_p;
		break;
	case FT_LOOKUP_LIMIT_FIRST:
		cmp_key = input_key + level - 1;
		break;
	case FT_LOOKUP_LIMIT_LAST: {
		unsigned int prefix_bytes = 0;

		if ((size_t)level <= iter->prefix_len) {
			prefix_bytes = iter->prefix_len - level + 1;
			if (prefix_bytes > (unsigned int)cmp)
				prefix_bytes = cmp;
			memcpy(last_key_buf, input_key + level - 1,
				prefix_bytes);
		}
		if (prefix_bytes < (unsigned int)cmp)
			memset(last_key_buf + prefix_bytes, 0xff,
				cmp - prefix_bytes);
		cmp_key = last_key_buf;
		break;
	}
	default:
		cmp_key = NULL;
		assert(0);
	}

	cmp_result = ft_key_cmp_ordinals(cmp_key, cn->key_bytes, cmp, cmp,
					true, &mpos);

	/* Fill ordinal_key and iter_path for the matched prefix. */
	for (j = 0; j < (cmp_result ? (int)mpos : cmp); j++) {
		ordinal_key[level - 1 + j] = cmp_key[j];
		iter_path_node(iter)[level + j] = node_flag;
	}
	/* Advance iter_key for LIMIT_NONE. */
	if (limit == FT_LOOKUP_LIMIT_NONE)
		*iter_key_p += (cmp_result ? mpos + 1 : (unsigned int)cmp);

	if (cmp_result) {
		/*
		 * Mismatch at position mpos.  Check direction
		 * relative to the lookup mode.
		 *
		 * key > path at mismatch: GE/GT go up, LE/LT descend.
		 * key < path at mismatch: GE/GT descend, LE/LT go up.
		 */
		ordinal_key[level - 1 + mpos] = cmp_key[mpos];
		iter_path_node(iter)[level + mpos] = node_flag;
		if ((cmp_result > 0 && (mode == FT_LOOKUP_GE || mode == FT_LOOKUP_GT)) ||
		    (cmp_result < 0 && (mode == FT_LOOKUP_LE || mode == FT_LOOKUP_LT))) {
			*level_p = level + mpos;
			iter_debug_path_snapshot(iter);
			return FT_COMPRESSED_GOING_UP;
		}
		/* Descend into compressed subtree. */
		ordinal_key[level - 1 + mpos] = cn->key_bytes[mpos];
		for (j = mpos + 1; j < cn->len; j++) {
			ordinal_key[level - 1 + j] = cn->key_bytes[j];
			iter_path_node(iter)[level + j] = node_flag;
		}
		level += cn->len - 1;
		node_flag = ft_dereference_acquire_prefetch(cn->child);
		if (!ft_node_ptr(node_flag))
			goto out_break;
		iter_path_node(iter)[level + 1] = node_flag;
		*skip_eq_external_nodes_p = false;
		*node_flag_p = node_flag;
		*level_p = level;
		iter_debug_path_snapshot(iter);
		return FT_COMPRESSED_DESCEND_CHILDREN;
	}

	if (cn->len > remaining) {
		/* Key shorter than compressed path. */
		if (mode == FT_LOOKUP_GE || mode == FT_LOOKUP_GT) {
			int k;

			for (k = cmp; k < cn->len; k++) {
				ordinal_key[level - 1 + k] = cn->key_bytes[k];
				iter_path_node(iter)[level + k] = node_flag;
			}
			level += cn->len - 1;
			node_flag = ft_dereference_acquire_prefetch(cn->child);
			if (!ft_node_ptr(node_flag))
				goto out_break;
			iter_path_node(iter)[level + 1] = node_flag;
			*skip_eq_external_nodes_p = false;
			*node_flag_p = node_flag;
			*level_p = level;
			iter_debug_path_snapshot(iter);
			return FT_COMPRESSED_DESCEND_CHILDREN;
		}
		*level_p = level + cmp - 1;
		iter_debug_path_snapshot(iter);
		return FT_COMPRESSED_GOING_UP;
	}

	/* Full match: advance past compressed path. */
	level += cn->len - 1; /* -1: for loop increments */
	node_flag = ft_dereference_acquire_prefetch(cn->child);
	if (!ft_node_ptr(node_flag))
		goto out_break;
	iter_path_node(iter)[level + 1] = node_flag;
	if (ft_node_external(node_flag))
		goto out_break;

	*node_flag_p = node_flag;
	*level_p = level;
	return FT_COMPRESSED_CONTINUE;

out_break:
	*node_flag_p = node_flag;
	*level_p = level;
	return FT_COMPRESSED_BREAK;
}

#ifdef FEATURE_FT_COLLAPSE
/*
 * Collapsed node handling for cds_ft_lookup_inequality (slow path).
 *
 * Scans all entries for the best match given the inequality mode:
 *   exact match → fill path, CONTINUE into child
 *   no exact, nearest in direction → fill path, DESCEND_CHILDREN
 *   no match in direction → GOING_UP
 */
static
enum ft_compressed_action ft_inequality_collapsed(struct cds_ft_inode_flag **node_flag_p,
		ssize_t *level_p, ssize_t key_depth,
		enum ft_lookup_inequality mode,
		enum ft_lookup_limit limit,
		const uint8_t **iter_key_p,
		const uint8_t *input_key,
		struct cds_ft_iter *iter,
		uint8_t *ordinal_key,
		bool *skip_eq_external_nodes_p,
		unsigned int nr_e)
{
	struct cds_ft_inode_flag *node_flag = *node_flag_p;
	struct cds_ft_collapsed_node *cn = ft_collapsed_node_ptr(node_flag);
	int level = *level_p;
	unsigned int remaining = key_depth - level;
	unsigned int e;
	int best_match = -1;	/* index of best directional match */
	uint8_t best_match_data = 0;
	struct cds_ft_inode_flag **ptrs = ft_collapsed_ptrs(cn, nr_e);

	for (e = 0; e < ft_collapsed_count(nr_e); e++) {
		unsigned int slen, cmp;
		uint8_t *suffix;
		const uint8_t *cmp_key;
		uint8_t last_key_buf[FT_MAX_KEY_LEN];
		int cmp_result;

		uint8_t data_e = ft_collapsed_load_data(cn, e);

		if (ft_collapsed_entry_dead(data_e, nr_e))
			continue;
		if (!ft_node_ptr(ptrs[e]))
			continue;
		slen = ft_collapsed_suffix_len(cn, data_e, e, nr_e);
		suffix = ft_collapsed_suffix(cn, data_e, nr_e);
		cmp = slen < remaining ? slen : remaining;

		/* Build contiguous comparison key for this limit mode. */
		switch (limit) {
		case FT_LOOKUP_LIMIT_NONE:
			cmp_key = *iter_key_p;
			break;
		case FT_LOOKUP_LIMIT_FIRST:
			cmp_key = input_key + level - 1;
			break;
		case FT_LOOKUP_LIMIT_LAST: {
			unsigned int prefix_bytes = 0;

			if ((size_t)level <= iter->prefix_len) {
				prefix_bytes = iter->prefix_len - level + 1;
				if (prefix_bytes > cmp)
					prefix_bytes = cmp;
				memcpy(last_key_buf, input_key + level - 1,
					prefix_bytes);
			}
			if (prefix_bytes < cmp)
				memset(last_key_buf + prefix_bytes, 0xff,
					cmp - prefix_bytes);
			cmp_key = last_key_buf;
			break;
		}
		default:
			cmp_key = NULL;
			assert(0);
		}
		cmp_result = ft_key_cmp_ordinals(cmp_key, suffix, cmp, cmp,
						true, NULL);
		if (cmp_result == 0) {
			if (slen <= remaining) {
				/* Full suffix match. Descend into child. */
				unsigned int k;

				for (k = 0; k < slen; k++) {
					ordinal_key[level - 1 + k] = suffix[k];
					iter_path_node(iter)[level + k] = node_flag;
				}
				level += slen - 1;
				assert(level < (int) key_depth);
				/*
				 * Ensure path[level] has the collapsed flag
				 * so the going-up loop can detect the
				 * collapsed span at path[level-1] after
				 * the loop's level-- decrement.  For slen=1,
				 * level didn't advance (slen-1=0), so
				 * path[level] already has it from the loop
				 * above, but path[level-1] (the parent) does
				 * not.  Setting path[level-1] extends the
				 * visible collapsed span downward by one.
				 */
				if (slen == 1)
					iter_path_node(iter)[level - 1] = node_flag;
				node_flag = ft_dereference_acquire_prefetch(ptrs[e]);
				if (!ft_node_ptr(node_flag)) {
					*node_flag_p = node_flag;
					*level_p = level;
					return FT_COMPRESSED_BREAK;
				}
				if (ft_node_skip_compressed(node_flag))
					node_flag = ft_compressed_node_flag(
						ft_skip_to_compressed(node_flag));
				iter_path_node(iter)[level + 1] = node_flag;
				if (ft_node_external(node_flag)) {
					*node_flag_p = node_flag;
					*level_p = level;
					return FT_COMPRESSED_BREAK;
				}
				*node_flag_p = node_flag;
				*level_p = level;
				return FT_COMPRESSED_CONTINUE;
			}
			/* Suffix longer than remaining key (key is shorter). */
			if (mode == FT_LOOKUP_GE || mode == FT_LOOKUP_GT) {
				cmp_result = -1; /* treat as key < suffix */
			} else {
				cmp_result = 1; /* treat as key > suffix */
			}
		}

		/*
		 * Track best directional match:
		 * GE/GT: want smallest suffix > key → cmp_result < 0 (key < suffix)
		 * LE/LT: want largest suffix < key → cmp_result > 0 (key > suffix)
		 */
		if ((mode == FT_LOOKUP_GE || mode == FT_LOOKUP_GT) && cmp_result < 0) {
			if (best_match < 0) {
				best_match = (int)e;
				best_match_data = data_e;
			} else {
				/* Keep the smallest. */
				uint8_t *bs = ft_collapsed_suffix(cn, best_match_data, nr_e);
				unsigned int bl = ft_collapsed_suffix_len(cn, best_match_data, (unsigned)best_match, nr_e);
				unsigned int mc = bl < slen ? bl : slen;
				int r = ft_key_cmp_ordinals(suffix, bs, mc, mc,
							true, NULL);

				if (r < 0 || (r == 0 && slen < bl)) {
					best_match = (int)e;
					best_match_data = data_e;
				}
			}
		} else if ((mode == FT_LOOKUP_LE || mode == FT_LOOKUP_LT) && cmp_result > 0) {
			if (best_match < 0) {
				best_match = (int)e;
				best_match_data = data_e;
			} else {
				/* Keep the largest. */
				uint8_t *bs = ft_collapsed_suffix(cn, best_match_data, nr_e);
				unsigned int bl = ft_collapsed_suffix_len(cn, best_match_data, (unsigned)best_match, nr_e);
				unsigned int mc = bl < slen ? bl : slen;
				int r = ft_key_cmp_ordinals(suffix, bs, mc, mc,
							true, NULL);

				if (r > 0 || (r == 0 && slen > bl)) {
					best_match = (int)e;
					best_match_data = data_e;
				}
			}
		}
	}

	if (best_match >= 0) {
		/* Descend into the best directional match. */
		unsigned int slen = ft_collapsed_suffix_len(cn, best_match_data, (unsigned)best_match, nr_e);
		uint8_t *suffix = ft_collapsed_suffix(cn, best_match_data, nr_e);
		unsigned int k;

		for (k = 0; k < slen; k++) {
			ordinal_key[level - 1 + k] = suffix[k];
			iter_path_node(iter)[level + k] = node_flag;
		}
		level += slen - 1;
		assert(level < (int) key_depth);
		if (slen == 1)
			iter_path_node(iter)[level - 1] = node_flag;
		node_flag = ft_dereference_acquire_prefetch(ptrs[best_match]);
		if (!ft_node_ptr(node_flag)) {
			*node_flag_p = node_flag;
			*level_p = level;
			return FT_COMPRESSED_BREAK;
		}
		if (ft_node_skip_compressed(node_flag))
			node_flag = ft_compressed_node_flag(
				ft_skip_to_compressed(node_flag));
		iter_path_node(iter)[level + 1] = node_flag;
		*skip_eq_external_nodes_p = false;
		*node_flag_p = node_flag;
		*level_p = level;
		iter_debug_path_snapshot(iter);
		return FT_COMPRESSED_DESCEND_CHILDREN;
	}

	/* No match in the requested direction. */
	*level_p = level - 1;
	iter_debug_path_snapshot(iter);
	return FT_COMPRESSED_GOING_UP;
}
#endif /* FEATURE_FT_COLLAPSE */

static enum cds_ft_status cds_ft_lookup_inequality(struct cds_ft *ft,
		struct cds_ft_iter *iter,
		enum ft_lookup_inequality mode,
		enum ft_lookup_limit limit)
{
	ssize_t key_depth, level;
	struct cds_ft_inode_flag *node_flag;
	struct cds_ft_node *ret_node;
	uint8_t ordinal_key[FT_MAX_KEY_LEN];
	enum ft_direction dir;
	uint8_t input_key_buf[FT_MAX_KEY_LEN];
	const uint8_t *input_key;
	const uint8_t *iter_key;
	size_t key_len = 0;
	bool going_up = false, skip_eq_external_nodes;
	/*
	 * Cache nr_entries from the downward collapsed walk so the
	 * going-up handler uses the same acquire-loaded snapshot.
	 * 0 means unset (collapsed nodes always have >= 1 entry by
	 * construction).  Consumed and reset to 0 after use, so
	 * collapsed nodes encountered at higher levels during
	 * going-up get a fresh load.
	 */
	unsigned int cached_col_nr_e = 0;

	CDS_FT_ASSERT_RCU_READ_LOCKED(ft);

	switch (limit) {
	case FT_LOOKUP_LIMIT_NONE:
		key_len = ft_key_len(ft, iter->key_len);
		if (!valid_key_len(ft, key_len)) {
			iter->node = NULL;
			iter->path_valid = false;
			iter_debug_path_update(iter);
			iter->status = CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
			goto end;
		}
		break;
	case FT_LOOKUP_LIMIT_FIRST:
		key_len = iter->prefix_len;
		break;
	case FT_LOOKUP_LIMIT_LAST:
		key_len = ft->group->max_key_len;
		break;
	}

	key_depth = key_len + 1;

	switch (mode) {
	case FT_LOOKUP_GE:
	case FT_LOOKUP_LE:
	case FT_LOOKUP_GT:
	case FT_LOOKUP_LT:
		break;
	default:
		abort();	/* Internal library error. */
	}

	/*
	 * Snapshot the input key so that iter_key(iter) can be overwritten
	 * with the result key without corrupting the input during the
	 * backtracking phase (which re-reads the input via iter_key).
	 */
	memcpy(input_key_buf, iter_key(iter), key_len);
	input_key = input_key_buf;
	iter_key = input_key;

	memset(ordinal_key, 0, ft->group->max_key_len * sizeof(ordinal_key[0]));
	node_flag = ft_dereference_prefetch(ft->root);
	iter_path_node(iter)[0] = node_flag;

	/*
	 * Empty root short-circuit: when the root has no children,
	 * there is nothing to traverse and no inequality match is
	 * possible. An empty root is always a type-0 linear node
	 * with data[0] == 0.  Compressed roots are never empty
	 * (recompaction replaces emptied compressed roots with
	 * internal nodes).
	 */
	if (!ft_node_compressed(node_flag)) {
		unsigned int type_idx = ft_node_type(node_flag);
		const struct cds_ft_type *type = &ft_types[type_idx];

		if (ft_type_is_linear(type->type_class) &&
				ft_linear_node_get_nr_child(type, ft_node_ptr(node_flag)) == 0) {

			/* A NIL key might still be stored directly in the root's metadata. */
			struct cds_ft_metadata *metadata = cds_ft_item_to_metadata_fast(
					ft_node_ptr(node_flag), type->order);

			if (!uatomic_load(&metadata->external_nodes, CMM_RELAXED)) {
				iter->node = NULL;
				iter->path_valid = true;
				iter_debug_path_snapshot(iter);
				iter->path_len = 1;
				iter->status = CDS_FT_STATUS_NOT_FOUND;
				goto end;
			}
		}
	}

	/*
	 * Fast path: reuse the iterator's cached path from a prior
	 * traversal when it is still valid and covers the full key
	 * depth.  This avoids redundant per-level ft_node_get_nth()
	 * lookups (which are the expensive, cache-miss-prone part of
	 * the downward walk).  The caller must hold the RCU read-side
	 * lock continuously for the cached pointers to remain valid.
	 */
	iter_debug_path_check(iter);
	if (iter->path_valid && (ssize_t)iter->path_len >= key_depth &&
			key_depth > 1) {
		for (level = 1; level < key_depth; level++) {
			switch (limit) {
			case FT_LOOKUP_LIMIT_NONE:
				ordinal_key[level - 1] =
					input_key[level - 1];
				break;
			case FT_LOOKUP_LIMIT_FIRST:
				ordinal_key[level - 1] =
					input_key[level - 1];
				break;
			case FT_LOOKUP_LIMIT_LAST:
				if ((size_t) level <= iter->prefix_len)
					ordinal_key[level - 1] =
						input_key[level - 1];
				else
					ordinal_key[level - 1] = 0xff;
				break;
			}
		}
		node_flag = iter_path_node(iter)[key_depth - 1];
		/*
		 * If the cached path entry is a compressed node, the
		 * fast path cannot determine the correct loop exit
		 * state (compressed nodes span multiple levels).
		 * Fall back to the slow path.
		 */
		if (ft_node_compressed(node_flag) ||
		    ft_node_skip_compressed(node_flag) ||
		    ft_node_collapsed(node_flag)) {
			node_flag = ft_dereference_prefetch(ft->root);
			iter_key = input_key;
			goto slow_path;
		}
		/*
		 * Reconstruct the loop exit value of @level to match
		 * what the traversal loop would have produced:
		 *  - key_depth     if last node is internal (loop ran
		 *                  to completion),
		 *  - key_depth - 1 if last node is external, NULL, or
		 *                  the path ended early (loop broke).
		 */
		if (!ft_node_external(node_flag))
			level = key_depth;
		else
			level = key_depth - 1;
		goto post_traversal;
	}

slow_path:
	for (level = 1; level < key_depth; level++) {
		uint8_t key_value;

		if (ft_node_compressed(node_flag)) {
			enum ft_compressed_action act;

			act = ft_inequality_compressed(&node_flag,
				&level, key_depth, mode, limit,
				&iter_key, input_key, iter,
				ordinal_key, &skip_eq_external_nodes);
			if (act == FT_COMPRESSED_GOING_UP)
				goto going_up;
			if (act == FT_COMPRESSED_DESCEND_CHILDREN)
				goto descend_children;
			if (act == FT_COMPRESSED_BREAK)
				break;
			continue;
		}
		if (ft_node_collapsed(node_flag)) {
			enum ft_compressed_action act;

			cached_col_nr_e = ft_collapsed_nr_entries(
				ft_collapsed_node_ptr(node_flag));
			act = ft_inequality_collapsed(&node_flag,
				&level, key_depth, mode, limit,
				&iter_key, input_key, iter,
				ordinal_key, &skip_eq_external_nodes,
				cached_col_nr_e);
			if (act == FT_COMPRESSED_GOING_UP)
				goto going_up;
			if (act == FT_COMPRESSED_DESCEND_CHILDREN)
				goto descend_children;
			if (act == FT_COMPRESSED_BREAK)
				break;
			continue;
		}

		switch (limit) {
		case FT_LOOKUP_LIMIT_NONE:
			key_value = *(iter_key++);
			break;
		case FT_LOOKUP_LIMIT_FIRST:
			key_value = input_key[level - 1];
			break;
		case FT_LOOKUP_LIMIT_LAST:
			if ((size_t) level <= iter->prefix_len)
				key_value = input_key[level - 1];
			else
				key_value = 0xff;
			break;
		}
		node_flag = ft_node_get_nth(node_flag, NULL, key_value);
		if (!ft_node_ptr(node_flag))
			break;
		ordinal_key[level - 1] = key_value;
		iter_path_node(iter)[level] = node_flag;
		dbg_printf("cds_ft_lookup_inequality iter key lookup %u finds node_flag %p\n",
				(unsigned int) key_value, node_flag);
		if (ft_node_external(node_flag))
			break;
	}

	/*
	 * The slow-path traversal has freshly populated the iterator
	 * path.  Capture a grace-period snapshot so that subsequent
	 * check() calls can validate this new path.  The fast path
	 * (goto post_traversal above) skips this: its cached path was
	 * already validated by iter_debug_path_check().
	 */
	iter_debug_path_snapshot(iter);

post_traversal:
	ft_delay_reader();
	switch (mode) {
	case FT_LOOKUP_LE:
	case FT_LOOKUP_GE:
		if (level == key_depth - 1) {
			struct cds_ft_node *external_nodes;

			if (ft_node_internal(node_flag)) {
				struct cds_ft_metadata *metadata;
				const struct cds_ft_type *type = &ft_types[ft_node_type(node_flag)];

				metadata = cds_ft_item_to_metadata_fast(ft_node_ptr(node_flag), type->order);
				external_nodes = ft_dereference_prefetch_external(metadata->external_nodes);
			} else if (ft_node_compressed(node_flag)) {
				struct cds_ft_metadata *metadata =
					cds_ft_item_to_metadata(ft_node_ptr(node_flag));
				external_nodes = ft_dereference_prefetch_external(metadata->external_nodes);
			} else {
				external_nodes = (struct cds_ft_node *) node_flag;
			}
			if (external_nodes) {
				/* End of key lookup succeded. We got an equal match. */
				iter->key_len = key_len;
				memcpy(iter_key(iter), input_key, key_len);
				iter->node = external_nodes;
				iter->path_valid = true;
				iter_debug_path_update(iter);
				iter->path_len = level + 1;
				iter->status = CDS_FT_STATUS_OK;
				goto end;
			}
		}
		break;
	case FT_LOOKUP_LT:
	case FT_LOOKUP_GT:
		break;
	default:
		assert(0);
	}

	/* If we reach end of key, we need to go one level backward. */
	if (level >= key_depth)
		level = key_depth - 1;

	/*
	 * For GE/GT: if the descent completed and the node at end-of-key
	 * is internal, any descendant key is strictly longer and therefore
	 * strictly greater. Skip backtracking and descend into children
	 * directly.
	 *
	 * For GE, the post-traversal above already returned if the node
	 * had external_nodes (the equal match). Reaching this point means
	 * no equal match exists, so descendant keys are the closest >=.
	 *
	 * For GT, the skip_eq_external_nodes flag (set below) will
	 * prevent the minmax descent from returning this node's own
	 * external_nodes (which are the equal match, not GT).
	 *
	 * LE/LT do not need this: their upward backtracking already
	 * checks external_nodes at each internal node going up, which is
	 * the correct direction to find shorter (lesser) prefix keys.
	 */
	if ((mode == FT_LOOKUP_GT || mode == FT_LOOKUP_GE) &&
			!ft_node_external(node_flag))
		goto descend_children;

going_up:
	/* Ensure iter_key is exactly at the position matching the level we stopped at. */
	iter_key = input_key + level;

	/*
	 * Find highest value left/right of current node.
	 * Current node is iter_path_node(iter)[level].
	 * Start at current level. If we cannot find any key left/right
	 * of ours, go one level up, seek highest value left/right of
	 * current (recursively), and when we find one, get the
	 * rightmost/leftmost child of its rightmost/leftmost child
	 * (recursively).
	 *
	 * Prefix-scoped traversal: backtracking stops at
	 * iter->prefix_len instead of 0, confining the search to the
	 * prefix subtree. When prefix_len == 0 this is identical to
	 * the original behavior.
	 */
	switch (mode) {
	case FT_LOOKUP_LE:
	case FT_LOOKUP_LT:
		dir = FT_LEFT;
		break;
	case FT_LOOKUP_GE:
	case FT_LOOKUP_GT:
		dir = FT_RIGHT;
		break;
	default:
		assert(0);
	}
	for (; level > (ssize_t) iter->prefix_len; level--) {
		uint8_t key_value;

		ft_delay_reader();
		/*
		 * Return external node if trying to find LE/LT
		 * inequality and encountering an external node when
		 * going upward.
		 */
		if (going_up && dir == FT_LEFT &&
		    !ft_node_external(iter_path_node(iter)[level])) {
			struct cds_ft_metadata *metadata;

			if (ft_node_compressed(iter_path_node(iter)[level]) ||
			    ft_node_collapsed(iter_path_node(iter)[level]))
				metadata = cds_ft_item_to_metadata(
					ft_node_ptr(iter_path_node(iter)[level]));
			else {
				const struct cds_ft_type *type = &ft_types[ft_node_type(iter_path_node(iter)[level])];
				metadata = cds_ft_item_to_metadata_fast(
					ft_node_ptr(iter_path_node(iter)[level]),
					type->order);
			}
			{
			struct cds_ft_node *external_nodes = ft_dereference_prefetch_external(metadata->external_nodes);

			if (external_nodes) {
				int j;

				assert(ft->group->key_len == CDS_FT_LEN_VARIABLE || level <= (int) ft->group->key_len);
				iter->key_len = level;
				for (j = 0; j < level; j++)
					iter_key(iter)[j] = ordinal_key[j];
				iter->node = external_nodes;
				iter->path_valid = true;
				iter_debug_path_update(iter);
				iter->path_len = level + 1;
				iter->status = CDS_FT_STATUS_OK;
				goto end;
			}
			}
		}

		switch (limit) {
		case FT_LOOKUP_LIMIT_NONE:
			key_value = *(--iter_key);
			break;
		case FT_LOOKUP_LIMIT_FIRST:
			key_value = input_key[level - 1];
			break;
		case FT_LOOKUP_LIMIT_LAST:
			if ((size_t) level <= iter->prefix_len)
				key_value = input_key[level - 1];
			else
				key_value = 0xff;
			break;
		}
		/*
		 * Standard sibling lookup. Parent is level - 1. We are
		 * looking for sibling of the byte at ordinal_key[level - 1].
		 * Skip levels where the path entry is not an internal node
		 * (compressed or external entries from compressed path
		 * traversal have no siblings).
		 */
#ifdef FEATURE_FT_COLLAPSE
		if (ft_node_collapsed(iter_path_node(iter)[level - 1])) {
			/*
			 * Collapsed node: entries ARE siblings. Find the
			 * entry level (where the collapsed node pointer
			 * starts in the path) and search for the next
			 * entry in the inequality direction.
			 */
			struct cds_ft_collapsed_node *col =
				ft_collapsed_node_ptr(
					iter_path_node(iter)[level - 1]);
			/*
			 * Use cached nr_entries from the downward walk
			 * when available; fresh load for collapsed
			 * nodes encountered at higher levels.
			 */
			unsigned int col_nr_e = cached_col_nr_e ?
				cached_col_nr_e :
				ft_collapsed_nr_entries(col);
			struct cds_ft_inode_flag **cptrs =
				ft_collapsed_ptrs(col, col_nr_e);
			int entry_depth = level - 1;
			unsigned int e;

			cached_col_nr_e = 0;

			/* Walk back to the collapsed node's entry depth. */
			while (entry_depth > 0 &&
			       iter_path_node(iter)[entry_depth - 1] ==
			       iter_path_node(iter)[level - 1])
				entry_depth--;

			/*
			 * suffix_base: the ordinal_key position where
			 * collapsed suffix data starts.  Normally
			 * entry_depth, except when the collapsed node is
			 * a direct child of a compressed node (no
			 * intermediate internal dispatch).  In that case,
			 * the compressed handler wrote ordinal_key one
			 * position earlier.
			 *
			 * Detect by checking the cached iter_path: if
			 * the parent is compressed, suffix_base is one
			 * earlier.  No entry scan needed — the iter_path
			 * is stable (set during the downward walk).
			 */
			{
			int suffix_base = entry_depth;
			if (entry_depth > 0 &&
			    ft_node_compressed(iter_path_node(iter)[entry_depth - 1]))
				suffix_base = entry_depth - 1;

			{
			int current_entry = -1, best;
			unsigned int cur_slen = 0;

			/*
			 * Find the current entry by suffix match.
			 * Don't skip dead/tombstoned entries: the suffix
			 * data is immutable and we need the position even
			 * if a concurrent writer tombstoned the entry.
			 */
			for (e = 0; e < ft_collapsed_count(col_nr_e); e++) {
				uint8_t *suffix;
				unsigned int slen, j2;
				bool match2;
				uint8_t data_e = ft_collapsed_load_data(col, e);

				suffix = ft_collapsed_suffix(col, data_e, col_nr_e);
				slen = ft_collapsed_suffix_len(col, data_e, e, col_nr_e);
				match2 = true;
				for (j2 = 0; j2 < slen; j2++) {
					if (suffix[j2] != ordinal_key[suffix_base + j2]) {
						match2 = false;
						break;
					}
				}
				if (match2) {
					current_entry = (int)e;
					cur_slen = slen;
					break;
				}
			}

			/*
			 * The current entry must always be found: the
			 * suffix is immutable, and the downward walk
			 * wrote it to ordinal_key.  If not found, there
			 * is a bug in suffix_base computation or
			 * ordinal_key was corrupted.
			 */
			assert(current_entry >= 0);

			/*
			 * If we're past the current entry's suffix (in
			 * the child's subtree), check the child node
			 * for siblings first.
			 */
			if (level > (int)(suffix_base + cur_slen)) {
				struct cds_ft_inode_flag *child_flag =
					ft_dereference_acquire(
						cptrs[current_entry]);

				if (ft_node_skip_compressed(child_flag))
					child_flag = ft_compressed_node_flag(
						ft_skip_to_compressed(
							child_flag));
				if (ft_node_ptr(child_flag) &&
				    ft_node_internal(child_flag)) {
					uint8_t sib_key = 0;

					node_flag = ft_node_get_leftright(
						child_flag,
						ordinal_key[suffix_base + cur_slen],
						&sib_key, dir);
					if (ft_node_ptr(node_flag)) {
						ordinal_key[suffix_base + cur_slen] =
							sib_key;
						level = suffix_base + cur_slen + 1;
						assert(level < key_depth);
						iter_path_node(iter)[level] =
							node_flag;
						break;
					}
				}
			}

			/* Find the nearest live sibling in @dir. */
			best = ft_collapsed_find_nearest(col, cptrs,
				(unsigned)current_entry, col_nr_e, dir);

			if (best >= 0) {
				uint8_t best_d = ft_collapsed_load_data(col, (unsigned)best);
				unsigned int slen = ft_collapsed_suffix_len(
					col, best_d, (unsigned)best, col_nr_e);
				uint8_t *suffix = ft_collapsed_suffix(
					col, best_d, col_nr_e);
				unsigned int k;

				for (k = 0; k < slen; k++) {
					ordinal_key[suffix_base + k] = suffix[k];
					if (k > 0)
						iter_path_node(iter)[suffix_base + k] =
							iter_path_node(iter)[level - 1];
				}
				level = suffix_base + slen;
				assert(level < key_depth);
				node_flag = ft_dereference_acquire_prefetch(
					cptrs[best]);
				if (ft_node_skip_compressed(node_flag))
					node_flag = ft_compressed_node_flag(
						ft_skip_to_compressed(
							node_flag));
				iter_path_node(iter)[level] = node_flag;
				break;
			}
			/* No match found, continue going up.
			 *
			 * Reset iter_key to match the new level.
			 * The collapsed handler jumped level back from
			 * within the entry's suffix span to
			 * entry_depth + 1.  iter_key must point to
			 * ordinal_key + entry_depth so the next
			 * *(--iter_key) at level entry_depth reads the
			 * correct key byte.
			 */
			level = entry_depth + 1;
			iter_key = ordinal_key + entry_depth;
			going_up = true;
			continue;
		} /* current_entry scope */
		} /* suffix_base scope */
		}
#endif
		if (!ft_node_internal(iter_path_node(iter)[level - 1])) {
			going_up = true;
			continue;
		}
		node_flag = ft_node_get_leftright(iter_path_node(iter)[level - 1],
				key_value, &ordinal_key[level - 1], dir);
		dbg_printf("cds_ft_lookup_inequality find sibling from %u at %u finds node_flag %p\n",
				(unsigned int) key_value, (unsigned int) ordinal_key[level - 1],
				node_flag);
		/* If found left/right sibling, find rightmost/leftmost child. */
		if (ft_node_ptr(node_flag)) {
			/* Record the sibling in the path. */
			iter_path_node(iter)[level] = node_flag;
			break;
		}
		going_up = true;
	}

	/*
	 * Prefix-scoped traversal: if backtracking exhausted the
	 * scope without finding a sibling, handle the prefix
	 * boundary.
	 *
	 * For LE/LT the prefix key itself (shorter than the search
	 * key) may be the closest match: return its external_nodes
	 * if present.
	 *
	 * For GE/GT no key within the scope satisfies the inequality.
	 *
	 * When going_up is false (e.g. LIMIT_FIRST/LIMIT_LAST
	 * reaching the prefix node without backtracking), we fall
	 * through to the downward min/max search below.
	 */
	if (going_up && level == (ssize_t) iter->prefix_len) {
		if (dir == FT_LEFT) {
			struct cds_ft_inode_flag *pfx_flag =
				iter_path_node(iter)[iter->prefix_len];

			if (!ft_node_external(pfx_flag)) {
				struct cds_ft_metadata *metadata;

				if (ft_node_compressed(pfx_flag))
					metadata = cds_ft_item_to_metadata(
						ft_node_ptr(pfx_flag));
				else {
					const struct cds_ft_type *type =
						&ft_types[ft_node_type(pfx_flag)];
					metadata = cds_ft_item_to_metadata_fast(
						ft_node_ptr(pfx_flag),
						type->order);
				}
				struct cds_ft_node *external_nodes =
					ft_dereference_prefetch_external(metadata->external_nodes);

				if (external_nodes) {
					int j;

					iter->key_len = iter->prefix_len;
					for (j = 0; j < (int) iter->prefix_len; j++)
						iter_key(iter)[j] = ordinal_key[j];
					iter->node = external_nodes;
					iter->path_valid = true;
					iter_debug_path_update(iter);
					iter->path_len = iter->prefix_len + 1;
					iter->status = CDS_FT_STATUS_OK;
					goto end;
				}
			}
		}
		iter->node = NULL;
		iter->path_valid = true;
		iter_debug_path_update(iter);
		iter->path_len = iter->prefix_len + 1;
		iter->status = CDS_FT_STATUS_NOT_FOUND;
		goto end;
	}

descend_children:
	if (ft_node_external(node_flag)) {
		int j;

		assert(ft->group->key_len == CDS_FT_LEN_VARIABLE || level <= (int) ft->group->key_len);
		iter->key_len = level;
		for (j = 0; j < level; j++)
			iter_key(iter)[j] = ordinal_key[j];
		iter->node = (struct cds_ft_node *) ft_node_ptr(node_flag);
		iter->path_valid = true;
		iter_debug_path_update(iter);
		iter->path_len = level + 1;
		iter->status = CDS_FT_STATUS_OK;
		goto end;
	}

	level++;

	/*
	 * From this point, we are guaranteed to be able to find a
	 * "lower than"/"greater than" match. ft_attach_node() and
	 * ft_detach_node() both guarantee that it is not possible for a
	 * lookup to reach a dead-end.
	 */

	/*
	 * Find rightmost/leftmost child of rightmost/leftmost child
	 * (recursively).
	 *
	 * skip_eq_external_nodes: when entering the minmax descent
	 * without backtracking (going_up == false) and the mode is
	 * strictly GT, the external_nodes at the first node are at the
	 * same position as the search key — equal, not strictly
	 * greater. Skip them on the first iteration so the descent
	 * continues to a proper child.
	 */
	skip_eq_external_nodes = (!going_up && mode == FT_LOOKUP_GT);
	switch (mode) {
	case FT_LOOKUP_LE:
	case FT_LOOKUP_LT:
		dir = FT_RIGHTMOST;
		break;
	case FT_LOOKUP_GE:
	case FT_LOOKUP_GT:
		dir = FT_LEFTMOST;
		break;
	default:
		assert(0);
	}
	for (; level < (int) ft->group->max_tree_depth; level++) {
		/*
		 * Return external node associated to internal node if
		 * trying to find GE/GT inequality and encountering an
		 * external node when going downward.
		 */
		if (dir == FT_LEFTMOST && ft_node_internal(node_flag)
				&& !skip_eq_external_nodes) {
			const struct cds_ft_type *type = &ft_types[ft_node_type(node_flag)];
			struct cds_ft_metadata *metadata = cds_ft_item_to_metadata_fast(
					ft_node_ptr(node_flag), type->order);
			struct cds_ft_node *external_nodes = ft_dereference_prefetch_external(metadata->external_nodes);

			if (external_nodes) {
				ret_node = external_nodes;
				level--;
				goto found_minmax;
			}
		}
		/* Return external node. */
		if (ft_node_external(node_flag))
			break;
		/*
		 * Skip-compressed: convert to compressed flag.
		 */
		if (ft_node_skip_compressed(node_flag))
			node_flag = ft_compressed_node_flag(
				ft_skip_to_compressed(node_flag));
		/*
		 * Compressed node: traverse through the compressed
		 * path to reach the child.  Fill ordinal_key and
		 * iter path as we go.
		 */
		if (ft_node_compressed(node_flag)) {
			struct cds_ft_compressed_node *cn =
				ft_compressed_node_ptr(node_flag);

			/*
			 * Check external_nodes at the compressed
			 * node's entry depth (for LEFTMOST/GE/GT).
			 */
			if (dir == FT_LEFTMOST) {
				struct cds_ft_metadata *cn_meta =
					cds_ft_item_to_metadata(
						(struct cds_ft_inode *) cn);
				struct cds_ft_node *ext =
					rcu_dereference(
						cn_meta->external_nodes);

				if (ext && !skip_eq_external_nodes) {
					ret_node = ext;
					level--;
					goto found_minmax;
				}
			}
			/*
			 * Fill ordinal_key and path entries for every
			 * level spanned by the compressed path.  The
			 * going-up code needs a valid entry at each
			 * level to call ft_node_get_direction (which
			 * returns NULL for siblings, causing the
			 * going-up walk to continue ascending).
			 */
			ft_fill_compressed_path(cn, ordinal_key, level - 1,
				iter_path_node(iter), level, node_flag);
			level += cn->len - 1;
			node_flag = ft_dereference_acquire_prefetch(cn->child);
			if (!ft_node_ptr(node_flag))
				break;
			iter_path_node(iter)[level] = node_flag;
			if (ft_node_external(node_flag))
				break;
			skip_eq_external_nodes = false;
			/* Continue descent from the child. */
			continue;
		}
		if (ft_node_collapsed(node_flag)) {
			struct cds_ft_collapsed_node *col =
				ft_collapsed_node_ptr(node_flag);
			unsigned int col_nr_e = ft_collapsed_nr_entries(col);
			struct cds_ft_inode_flag **cptrs =
				ft_collapsed_ptrs(col, col_nr_e);
			unsigned int best = UINT_MAX, e;

			/*
			 * Check external_nodes at the collapsed
			 * node's depth (for LEFTMOST/GE/GT).
			 */
			if (dir == FT_LEFTMOST) {
				struct cds_ft_metadata *col_meta =
					cds_ft_item_to_metadata(
						(struct cds_ft_inode *) col);
				struct cds_ft_node *ext =
					rcu_dereference(
						col_meta->external_nodes);

				if (ext && !skip_eq_external_nodes) {
					ret_node = ext;
					level--;
					goto found_minmax;
				}
			}
			/*
			 * Find the min/max entry by suffix and
			 * descend into it.
			 */
			uint8_t best_d = 0;

			for (e = 0; e < ft_collapsed_count(col_nr_e); e++) {
				uint8_t data_e = ft_collapsed_load_data(col, e);

				if (ft_collapsed_entry_dead(data_e, col_nr_e))
					continue;
				if (!ft_node_ptr(cptrs[e]))
					continue;
				if (best == UINT_MAX) {
					best = e;
					best_d = data_e;
					continue;
				}
				{
					uint8_t *sa = ft_collapsed_suffix(col, best_d, col_nr_e);
					unsigned int la = ft_collapsed_suffix_len(col, best_d, best, col_nr_e);
					uint8_t *sb = ft_collapsed_suffix(col, data_e, col_nr_e);
					unsigned int lb = ft_collapsed_suffix_len(col, data_e, e, col_nr_e);
					unsigned int mc = la < lb ? la : lb;
					int r = memcmp(sb, sa, mc);

					if (dir == FT_LEFTMOST) {
						if (r < 0 || (r == 0 && lb < la)) {
							best = e;
							best_d = data_e;
						}
					} else {
						if (r > 0 || (r == 0 && lb > la)) {
							best = e;
							best_d = data_e;
						}
					}
				}
			}
			if (best == UINT_MAX)
				break;
			{
				unsigned int slen = ft_collapsed_suffix_len(col, best_d, best, col_nr_e);
				uint8_t *suffix = ft_collapsed_suffix(col, best_d, col_nr_e);
				unsigned int k;

				for (k = 0; k < slen; k++) {
					ordinal_key[level - 1 + k] = suffix[k];
					iter_path_node(iter)[level + k] =
						ft_collapsed_node_flag(col);
				}
				level += slen - 1;
				assert(level < key_depth);
				node_flag = ft_dereference_acquire_prefetch(cptrs[best]);
				if (!ft_node_ptr(node_flag))
					break;
				if (ft_node_skip_compressed(node_flag))
					node_flag = ft_compressed_node_flag(
						ft_skip_to_compressed(node_flag));
				iter_path_node(iter)[level + 1] = node_flag;
				if (ft_node_external(node_flag))
					break;
			}
			skip_eq_external_nodes = false;
			continue;
		}
		skip_eq_external_nodes = false;
		node_flag = ft_node_get_minmax(node_flag, &ordinal_key[level - 1], dir);
		/*
		 * If minmax returns NULL, it was an empty root. We found nothing.
		 */
		if (caa_unlikely(!ft_node_ptr(node_flag))) {
			iter->node = NULL;
			iter->path_valid = true;
			iter_debug_path_update(iter);
			iter->path_len = level;
			iter->status = CDS_FT_STATUS_NOT_FOUND;
			goto end;
		}
		iter_path_node(iter)[level] = node_flag;
		dbg_printf("cds_ft_lookup_inequality find minmax at %u finds node_flag %p\n",
				(unsigned int) ordinal_key[level - 1], node_flag);
		if (ft_node_external(node_flag))
			break;
	}
	/* attach/detach semantic guarantees that ft_node_get_minmax cannot return NULL. */
	assert(ft_node_ptr(node_flag));
	ret_node = (struct cds_ft_node *) node_flag;
	/*
	 * The trie should always have external nodes at the
	 * very last level, so level should never grow large enough to overflow
	 * max_key_len.
	 */
	assert(level <= (int) ft->group->max_key_len);
	assert(ft->group->key_len == CDS_FT_LEN_VARIABLE || level <= (int) ft->group->key_len);
found_minmax:
	{
		int j;

		iter->key_len = level;
		for (j = 0; j < level; j++)
			iter_key(iter)[j] = ordinal_key[j];
		iter->node = ret_node;
		iter->path_valid = true;
		iter_debug_path_update(iter);
		iter->path_len = level + 1;
		iter->status = ret_node ? CDS_FT_STATUS_OK : CDS_FT_STATUS_NOT_FOUND;
	}
end:
	iter_auto_invalidate_path(iter);
	return iter->status;
}

/*
 * Iterator-based inequality lookup public API.
 * The caller sets the key via cds_ft_iter_set_key() before calling.
 * On return the iterator holds the result key, key length, node, path,
 * and status.
 */
enum cds_ft_status cds_ft_lookup_le(struct cds_ft *ft,
		struct cds_ft_iter *iter)
{
	dbg_printf("cds_ft_lookup_le\n");
	return cds_ft_lookup_inequality(ft, iter,
			FT_LOOKUP_LE, FT_LOOKUP_LIMIT_NONE);
}

enum cds_ft_status cds_ft_lookup_ge(struct cds_ft *ft,
		struct cds_ft_iter *iter)
{
	dbg_printf("cds_ft_lookup_ge\n");
	return cds_ft_lookup_inequality(ft, iter,
			FT_LOOKUP_GE, FT_LOOKUP_LIMIT_NONE);
}

enum cds_ft_status cds_ft_lookup_lt(struct cds_ft *ft,
		struct cds_ft_iter *iter)
{
	dbg_printf("cds_ft_lookup_lt\n");
	return cds_ft_lookup_inequality(ft, iter,
			FT_LOOKUP_LT, FT_LOOKUP_LIMIT_NONE);
}

enum cds_ft_status cds_ft_lookup_gt(struct cds_ft *ft,
		struct cds_ft_iter *iter)
{
	dbg_printf("cds_ft_lookup_gt\n");
	return cds_ft_lookup_inequality(ft, iter,
			FT_LOOKUP_GT, FT_LOOKUP_LIMIT_NONE);
}

enum cds_ft_status cds_ft_lookup_first(struct cds_ft *ft,
		struct cds_ft_iter *iter)
{
	size_t saved_key_len = iter->key_len;
	enum cds_ft_status status;

	dbg_printf("cds_ft_lookup_first\n");
	/*
	 * LIMIT_FIRST sets key_len to prefix_len internally.
	 * When prefix_len == 0 this corresponds to a traversal of the
	 * entire trie.
	 * When prefix_len > 0 it descends through the prefix key
	 * bytes, then the GE post-traversal returns the prefix key
	 * itself if it has external_nodes, or the LEFTMOST minmax
	 * descent finds the smallest descendant.
	 */
	iter->key_len = iter->prefix_len;
	status = cds_ft_lookup_inequality(ft, iter,
			FT_LOOKUP_GE, FT_LOOKUP_LIMIT_FIRST);
	if (status < 0)
		iter->key_len = saved_key_len;
	return status;
}

enum cds_ft_status cds_ft_lookup_last(struct cds_ft *ft,
		struct cds_ft_iter *iter)
{
	size_t saved_key_len = iter->key_len;
	enum cds_ft_status status;

	dbg_printf("cds_ft_lookup_last\n");
	/*
	 * LIMIT_LAST always uses key_len = max_key_len. When
	 * prefix_len > 0, the traversal uses actual prefix key bytes
	 * for levels 1..prefix_len then 0xFF for the remaining
	 * levels, descending as deep as possible along the rightmost
	 * path within the prefix subtree. LE backtracking (bounded
	 * at prefix_len) then finds the greatest actual key.
	 */
	iter->key_len = ft->group->max_key_len;
	status = cds_ft_lookup_inequality(ft, iter,
			FT_LOOKUP_LE, FT_LOOKUP_LIMIT_LAST);
	if (status < 0)
		iter->key_len = saved_key_len;
	return status;
}

/*
 * Propagate a signed delta to nr_keys through a snapshot
 * of ancestor internal nodes collected during descent.
 *
 * There are two types of concurrent readers:
 *
 *  - Pointer-based readers (iteration, key lookup) follow pointers
 *    with rcu_dereference.  They are not affected by nr_keys and
 *    always see a structurally consistent trie via RCU.
 *
 *  - Count-based readers (lookup_nth, lookup_nth_last, skip,
 *    count_keys, count_keys_prefix) read nr_keys to guide their
 *    descent.  They use ft_dereference_acquire (CMM_ACQUIRE load)
 *    for both nr_keys loads and child pointer loads, rather than
 *    rcu_dereference, to obtain the memory ordering described below.
 *
 * Undercount property:
 *
 * The update ordering is chosen so that nr_keys transiently
 * undercounts (nr_keys <= actual reachable keys) rather than
 * overcounts.  This is the conservative direction for count-based
 * readers: they may transiently miss a key at the boundary of a
 * concurrent mutation, but they will never enter a subtree expecting
 * a key that does not exist.  The alternative (overcount) would cause
 * count-based readers to descend into a subtree with fewer keys than
 * expected, potentially yielding NOT_FOUND for a key that should be
 * reachable at that rank.
 *
 * Update ordering:
 *
 *   Insert: publish pointer (rcu_assign_pointer), then increment
 *           nr_keys (uatomic_store CMM_RELEASE).
 *   Remove: decrement nr_keys (uatomic_store CMM_RELEASE), then
 *           detach pointer (rcu_assign_pointer).
 *
 * Read-side patterns:
 *
 * Count-based readers traverse the trie both downward and upward.
 * The read-side ordering of nr_keys vs pointer loads depends on
 * the traversal pattern, and each pattern interacts differently
 * with the insert and remove orderings.  Three distinct patterns
 * arise:
 *
 * Pattern 1 — child pointer, then child's nr_keys
 *             (downward descent + upward walk):
 *
 *   Reader:
 *     R1: ft_dereference_acquire(child)        [load-acquire on parent's slot]
 *     R2: uatomic_load(child.nr_keys, CMM_ACQUIRE)
 *
 *   This is the standard message-passing order.  It occurs whenever
 *   the reader loads a child pointer from a parent node and then
 *   reads the child's own nr_keys (ft_child_key_count).  This
 *   happens in both the downward descent of lookup_nth and the
 *   upward walk of skip when iterating sibling subtrees.
 *
 *   Insert:  The new node's nr_keys is initialized before it is
 *     published via rcu_assign_pointer.  If R1 sees the new child
 *     (acquire pairs with the publish release), R2 sees the
 *     initial nr_keys.  If R1 sees NULL (not yet published), the
 *     reader skips — undercount.
 *
 *   Remove:  The writer decrements the child's nr_keys before
 *     detaching a deeper pointer.  At this level the child pointer
 *     itself is unchanged, so R1 always sees the child.  R2 sees
 *     either old or decremented nr_keys — both <= actual.
 *     Undercount holds trivially.
 *
 * Pattern 2 — external_nodes, then child pointers
 *             (downward descent only):
 *
 *   Reader:
 *     R1: ft_dereference_acquire(metadata->external_nodes)
 *     R2: ft_dereference_acquire(child)
 *
 *   At each internal node during downward descent, the reader
 *   first checks external_nodes (keys at this depth), then
 *   iterates children.  Both fields belong to the same node.
 *
 *   Insert (setting external_nodes):  The writer does
 *     rcu_assign_pointer(external_nodes, node) then increments
 *     ancestor nr_keys.  R1 acquire pairs with the publish
 *     release — if the reader sees the new external_nodes, the
 *     key is found.  If not, undercount.
 *
 *   Remove (clearing external_nodes):  The writer decrements
 *     nr_keys then rcu_assign_pointer(external_nodes, NULL).
 *     If R1 sees NULL, the acquire pairs with the release,
 *     making the nr_keys decrement visible to subsequent reads.
 *     If R1 sees the old external_nodes, the key is still
 *     reachable — consistent pre-remove snapshot.
 *
 * Pattern 3 — current node's nr_keys, then child pointers
 *             (skip_forward at_external_nodes case only):
 *
 *   Reader:
 *     R1: uatomic_load(node.nr_keys, CMM_ACQUIRE)
 *     R2: ft_dereference_acquire(child)
 *
 *   This inverted message-passing order occurs only in
 *   skip_forward when the current position is at an internal
 *   node's external_nodes: the reader reads the node's nr_keys
 *   to count remaining keys in the subtree, then iterates
 *   children.
 *
 *   Insert:
 *     Writer:
 *       W1: rcu_assign_pointer(child, new_node)  [store-release]
 *       W2: uatomic_store(node.nr_keys, ++, CMM_RELEASE)
 *     W2 release ensures W1 is visible when W2 becomes visible.
 *     If R1 sees the incremented nr_keys (acquire pairs with
 *     W2 release), all stores before W2 — including W1 — are
 *     visible.  R2 is ordered after R1 (by R1 acquire), so R2
 *     sees the published pointer.
 *     If R1 sees the old nr_keys, the reader does not know about
 *     the new key — undercount.
 *
 *   Remove:
 *     Writer:
 *       W1: uatomic_store(node.nr_keys, --, CMM_RELEASE)
 *       W2: rcu_assign_pointer(child, NULL)      [store-release]
 *     W2 release ensures W1 is visible when W2 becomes visible.
 *     If R2 sees the detached pointer (acquire pairs with W2
 *     release), W1 is visible.  On multi-copy-atomic
 *     architectures (x86 TSO, ARMv8), R1 acquire orders R1
 *     before R2, and the coherence guarantee ensures R1 observes
 *     at least the state that was globally visible when R2's
 *     value was stored — which includes W1.  So R1 sees the
 *     decremented nr_keys.
 *     If R2 sees the old pointer (child still present), the key
 *     is still reachable.  nr_keys may be old or decremented —
 *     either way <= actual (undercount).
 *     If R1 sees the decremented nr_keys but R2 sees the old
 *     pointer, nr_keys < actual — undercount.
 *
 * In all three patterns, regardless of which combination of
 * old/new values the reader observes, nr_keys <= actual reachable
 * keys (undercount property).
 *
 * The ft_dereference_acquire macro (CMM_ACQUIRE rather than
 * rcu_dereference) is specifically needed for Pattern 3's remove
 * case, where the reader loads nr_keys before the pointer at the
 * same level.  Without acquire on the pointer load, a weakly-
 * ordered architecture could observe the detached pointer without
 * the preceding nr_keys decrement, violating the undercount
 * property.  Patterns 1 and 2 would be safe with rcu_dereference
 * alone, but all patterns use ft_dereference_acquire uniformly
 * for simplicity.
 */
/*
 * ft_propagate_external_count_parent: propagate nr_keys delta
 * from @start up to the root via metadata->parent pointers.
 * instead of a pre-built snapshot array.
 *
 * @start: deepest internal/compressed/collapsed node on the path
 *         (the node where the external was attached, or the
 *         deepest ancestor with metadata).  Must not be an
 *         external node or NULL.
 * @delta: +1 for insert, -1 for remove.
 *
 * Same ordering guarantees as the snapshot-based variant:
 * bottom-up CMM_RELEASE stores preserve the undercount invariant.
 */
static
void ft_propagate_external_count_parent(struct cds_ft_inode_flag *start,
		long delta)
{
	struct cds_ft_inode_flag *cur = start;

	ft_delay_writer();

	while (cur) {
		struct cds_ft_metadata *m =
			cds_ft_item_to_metadata(ft_node_ptr(cur));
		uatomic_store(&m->nr_keys, m->nr_keys + delta, CMM_RELEASE);
		ft_delay_writer();
		cur = m->parent;
	}
}

/*
 * Compact density counter accessors.
 *
 * ft_density_get: read counter[idx] from compact or extended storage.
 * ft_density_promote: allocate extended storage, copy compact values.
 * ft_density_set: write counter[idx], promoting to extended on overflow.
 * ft_density_add: saturating add (clamped at 0 on underflow).
 * ft_density_sub: saturating subtract (clamped at 0 on underflow).
 * ft_density_free: free extended storage (called on node reclaim).
 *
 * All write-side only (mutex-held).
 */
static inline
unsigned long ft_density_get(const struct cds_ft_metadata *m, unsigned int idx)
{
	if (caa_unlikely(m->density_ext != NULL))
		return m->density_ext->nr_nodes_at_depth[idx];
	return m->nr_nodes_at_depth[idx];
}

static
void ft_density_promote(struct cds_ft_metadata *m)
{
	struct cds_ft_density_extended *ext;
	unsigned int i;

	ext = calloc(1, sizeof(*ext));
	if (!ext)
		abort();
	for (i = 0; i < FT_NODE_DENSITY_DEPTH; i++)
		ext->nr_nodes_at_depth[i] = m->nr_nodes_at_depth[i];
	m->density_ext = ext;
}

static inline
void ft_density_set(struct cds_ft_metadata *m, unsigned int idx, unsigned long val)
{
	if (caa_unlikely(m->density_ext != NULL)) {
		m->density_ext->nr_nodes_at_depth[idx] = val;
		return;
	}
	if (caa_unlikely(val > FT_DENSITY_COMPACT_MAX)) {
		ft_density_promote(m);
		m->density_ext->nr_nodes_at_depth[idx] = val;
		return;
	}
	m->nr_nodes_at_depth[idx] = (uint16_t) val;
}

static inline
void ft_density_add(struct cds_ft_metadata *m, unsigned int idx, long delta)
{
	unsigned long val = ft_density_get(m, idx);

	assert(delta >= 0 || val >= (unsigned long) -delta);
	val += delta;
	ft_density_set(m, idx, val);
}

static inline
void ft_density_sub(struct cds_ft_metadata *m, unsigned int idx, unsigned long sub)
{
	unsigned long val = ft_density_get(m, idx);

	assert(val >= sub);
	ft_density_set(m, idx, val - sub);
}

static inline
void ft_density_free(struct cds_ft_metadata *m)
{
	free(m->density_ext);
	m->density_ext = NULL;
}

/*
 * Compute the precise density contribution of a child node to its
 * parent.  The child is at @distance levels below the parent.
 *
 * The child's nr_nodes_at_depth[0] covers levels child+1 through
 * child+DEPTH.  From the parent's perspective, those are at levels
 * parent+(distance+1) through parent+(distance+DEPTH).  The parent's
 * window ends at parent+DEPTH, so levels beyond that must be
 * subtracted.  The excess is the sum of per-level counters at
 * child levels (DEPTH - distance + 1) through DEPTH, stored in
 * nr_nodes_at_depth[DEPTH - distance] through nr_nodes_at_depth[DEPTH - 1].
 */
static inline unsigned long ft_child_density_contribution(
		struct cds_ft_metadata *cm, unsigned int distance)
{
	unsigned long contrib = 1;	/* the child node itself */
	unsigned int j;

	if (distance >= FT_NODE_DENSITY_DEPTH)
		return contrib;
	contrib += ft_density_get(cm, 0);
	/* Subtract levels that overflow the parent's window. */
	for (j = FT_NODE_DENSITY_DEPTH - distance; j < FT_NODE_DENSITY_DEPTH; j++)
		contrib -= ft_density_get(cm, j);
	return contrib;
}

/*
 * ft_init_node_density: set a node's density counter[0] by counting
 * traversable children within FT_NODE_DENSITY_DEPTH levels.
 *
 * Uses ft_child_density_contribution() for precise window correction
 * at each child distance.
 *
 * Only sets counter[0] (cumulative sum).  Per-level counters [1..N-1]
 * are maintained by ft_propagate_node_density_parent().
 */
static
void ft_init_node_density(struct cds_ft_inode_flag *node_flag)
{
	struct cds_ft_inode *node;
	struct cds_ft_metadata *meta;
	unsigned int key;
	unsigned long total = 0;

	if (!ft_node_ptr(node_flag))
		return;
	/*
	 * Skip-compressed: initialize density on the underlying
	 * compressed node.  Must check before ft_node_external
	 * because a skip pointer with an external child has low
	 * tag bits == 0.
	 */
	if (ft_node_skip_compressed(node_flag)) {
		struct cds_ft_compressed_node *cn =
			ft_skip_to_compressed(node_flag);
		struct cds_ft_metadata *cn_meta =
			cds_ft_item_to_metadata(
				(struct cds_ft_inode *) cn);
		struct cds_ft_inode_flag *child =
			ft_skip_child_ptr(node_flag);

		if (ft_node_ptr(child) && !ft_node_external(child)) {
			if (cn->len <= FT_NODE_DENSITY_DEPTH) {
				struct cds_ft_metadata *cm =
					cds_ft_item_to_metadata(
						ft_node_ptr(child));
				total += ft_child_density_contribution(
					cm, cn->len);
			}
		}
		ft_density_set(cn_meta, 0, total);
		return;
	}
	if (ft_node_external(node_flag))
		return;
	if (ft_node_compressed(node_flag)) {
		struct cds_ft_compressed_node *cn = ft_compressed_node_ptr(node_flag);
		struct cds_ft_metadata *cn_meta = cds_ft_item_to_metadata((struct cds_ft_inode *) cn);

		if (ft_node_ptr(cn->child) && !ft_node_external(cn->child)) {
			if (cn->len <= FT_NODE_DENSITY_DEPTH) {
				struct cds_ft_metadata *cm = cds_ft_item_to_metadata(ft_node_ptr(cn->child));
				total += ft_child_density_contribution(cm, cn->len);
			}
		}
		ft_density_set(cn_meta, 0, total);
		return;
	}
	if (ft_node_collapsed(node_flag)) {
		struct cds_ft_collapsed_node *col = ft_collapsed_node_ptr(node_flag);
		struct cds_ft_metadata *col_meta = cds_ft_item_to_metadata((struct cds_ft_inode *) col);
		unsigned int e;

		unsigned int nr_e = ft_collapsed_nr_entries(col);

		for (e = 0; e < ft_collapsed_count(nr_e); e++) {
			uint8_t data_e = ft_collapsed_load_data(col, e);
			unsigned int slen;
			struct cds_ft_inode_flag *child;

			if (ft_collapsed_entry_dead(data_e, nr_e))
				continue;
			slen = ft_collapsed_suffix_len(col, data_e, e, nr_e);
			child = ft_collapsed_ptrs(col, nr_e)[e];
			if (!ft_node_ptr(child) || ft_node_external(child))
				continue;
			if (ft_node_skip_compressed(child)) {
				/* Zero cache line cost on lookup. */
				continue;
			}
			if (slen <= FT_NODE_DENSITY_DEPTH) {
				struct cds_ft_metadata *cm = cds_ft_item_to_metadata(ft_node_ptr(child));
				total += ft_child_density_contribution(cm, slen);
			}
		}
		ft_density_set(col_meta, 0, total);
		return;
	}
	/* Internal node: walk children (at distance 1). */
	node = ft_node_ptr(node_flag);
	meta = cds_ft_item_to_metadata(node);

	for (key = 0; key < 256; key++) {
		struct cds_ft_inode_flag *child = ft_node_get_nth(node_flag, NULL, (uint8_t) key);

		if (!ft_node_ptr(child))
			continue;
		if (ft_node_external(child))
			continue;
		/*
		 * Skip-compressed pointer: the compressed node is
		 * never accessed on the lookup fast path (zero cache
		 * line cost).  Contribute negative weight proportional
		 * to the skip length — longer compressed paths are
		 * more valuable to preserve, and collapsing their
		 * parent would replace free skip traversal with a
		 * costly collapsed suffix scan.
		 */
		if (ft_node_skip_compressed(child))
			continue;
		{
			struct cds_ft_metadata *cm = cds_ft_item_to_metadata(ft_node_ptr(child));
			total += ft_child_density_contribution(cm, 1);
		}
	}
	ft_density_set(meta, 0, total);
}

/*
 * ft_parent_depth_span: compute the number of trie depth levels
 * between a parent node and one of its children.
 *
 * @parent_nf: tagged pointer to the parent (internal, compressed,
 *             or collapsed).
 * @child_nf:  tagged pointer to the child (used only for collapsed
 *             parent to identify the entry).
 *
 * Returns: 1 for internal nodes (one key byte per level),
 *          cn->len for compressed nodes,
 *          suffix_len for the matching collapsed entry.
 *
 * Write-side only (mutex-held).
 */
static
unsigned int ft_parent_depth_span(struct cds_ft_inode_flag *parent_nf,
		struct cds_ft_inode_flag *child_nf)
{
	if (ft_node_compressed(parent_nf)) {
		struct cds_ft_compressed_node *cn =
			ft_compressed_node_ptr(parent_nf);
		return cn->len;
	}
	if (ft_node_collapsed(parent_nf)) {
		struct cds_ft_collapsed_node *col =
			ft_collapsed_node_ptr(parent_nf);
		unsigned int nr_e = ft_collapsed_nr_entries(col);
		struct cds_ft_inode_flag **cptrs =
			ft_collapsed_ptrs(col, nr_e);
		unsigned int e;

		for (e = 0; e < ft_collapsed_count(nr_e); e++) {
			if (cptrs[e] == child_nf) {
				uint8_t data_e =
					ft_collapsed_load_data(col, e);
				return ft_collapsed_suffix_len(
					col, data_e, e, nr_e);
			}
		}
		assert(0);
		return 1;
	}
	/* Internal node: dispatches on one key byte. */
	return 1;
}

/*
 * ft_propagate_node_density_parent: parent-pointer variant of
 * ft_propagate_node_density.  Walks up via metadata->parent
 * and computes ancestor depth on the fly using ft_parent_depth_span.
 *
 * @start: deepest ancestor with metadata on the path.
 * @start_depth: trie depth of @start.
 * @node_depth: depth of the created/destroyed node.
 * @delta: +1 for creation, -1 for destruction.
 *
 * Only called from the write-side (mutex-held).
 */
static
void ft_propagate_node_density_parent(struct cds_ft_inode_flag *start,
		unsigned int start_depth,
		unsigned int node_depth, long delta)
{
	struct cds_ft_inode_flag *cur = start;
	unsigned int cur_depth = start_depth;

	while (cur) {
		struct cds_ft_metadata *m;
		unsigned int distance;
		struct cds_ft_inode_flag *parent;

		if (cur_depth >= node_depth)
			goto next;
		distance = node_depth - cur_depth;
		if (distance > FT_NODE_DENSITY_DEPTH)
			break;

		m = ft_flag_to_metadata(cur);
		ft_density_add(m, 0, delta);
		if (distance >= 2)
			ft_density_add(m, distance - 1, delta);
next:
		m = ft_flag_to_metadata(cur);
		parent = m->parent;
		if (!parent)
			break;
		cur_depth -= ft_parent_depth_span(parent, cur);
		cur = parent;
	}
}

#ifdef FEATURE_FT_COLLAPSE
/*
 * Collapse threshold.
 *
 * FT_COLLAPSE_SUFFIX_MIN: minimum suffix length for at least one
 *   entry after recursive path enumeration.  Entries with slen=1
 *   (single ordinal byte) don't save any traversal hops over a
 *   normal internal node dispatch.
 */
#define FT_COLLAPSE_SUFFIX_MIN		2

/*
 * Minimum suffix length per scan zone size to maintain strict
 * cache-line load upper bounds.  Every live entry must satisfy:
 *   slen >= (scan_zone_size / 64) + 1
 * This is a WORST-CASE bound, not an average.
 */
static inline
unsigned int ft_collapsed_min_slen(unsigned int scan_sel)
{
	static const unsigned int min_slen[] = {
		[FT_COLLAPSED_SCAN_32]  = 2,	/* <1 CL: slen >= 2 (SUFFIX_MIN) */
		[FT_COLLAPSED_SCAN_64]  = 2,	/* 1 CL: slen >= 2 */
		[FT_COLLAPSED_SCAN_128] = 3,	/* 2 CL: slen >= 3 */
		[FT_COLLAPSED_SCAN_256] = 5,	/* 4 CL: slen >= 5 */
	};
	return min_slen[scan_sel];
}

/*
 * Collapsed node configuration table.
 *
 * Ordered by preference: smallest allocation first, within the same
 * allocation prefer wider scan zone (better suffix coverage at the
 * cost of stricter slen requirements).
 *
 * The selection walks this table and picks the first entry where:
 *   1. density fits in max_entries
 *   2. all suffix lengths >= min_slen (checked post-walk)
 */

/*
 * ft_collapse_walk_subtree: recursively enumerate paths from @walk,
 * appending suffix bytes to @suffix_buf.  Each leaf (external node)
 * or depth-limited node becomes a separate entry in the collapsed
 * node.  Multi-child internal nodes are recursed into, forking
 * separate entries per child path.
 *
 * Returns 0 on success, -1 if the collapsed node can't fit all
 * paths (too many entries or scan zone overflow).  On failure the
 * caller must discard the entire collapsed node — partial results
 * are not usable.
 */
static
int ft_collapse_walk_subtree(struct cds_ft *ft,
		struct cds_ft_inode_flag *walk,
		uint8_t *suffix_buf, unsigned int slen,
		unsigned int max_depth,
		struct cds_ft_collapsed_node *col,
		struct cds_ft_inode_flag **col_ptrs,
		unsigned int max_entries)
{
	while (slen < max_depth) {
		if (!ft_node_ptr(walk))
			return -1;

		/* Leaf: emit terminal entry. */
		if (ft_node_external(walk))
			goto emit_entry;

		/*
		 * Skip-compressed: follow to underlying compressed
		 * node so its path bytes are absorbed normally.
		 */
		if (ft_node_skip_compressed(walk))
			walk = ft_compressed_node_flag(
				ft_skip_to_compressed(walk));

		/* Compressed: absorb path bytes. */
		if (ft_node_compressed(walk)) {
			struct cds_ft_compressed_node *cn =
				ft_compressed_node_ptr(walk);
			struct cds_ft_metadata *cn_meta =
				cds_ft_item_to_metadata(
					(struct cds_ft_inode *) cn);
			unsigned int j;

			/*
			 * If a prefix key ends at this compressed
			 * node's depth (external_nodes is set), emit
			 * an intermediate entry to preserve it.
			 * Absorbing the compressed path would skip
			 * past the prefix entry.
			 */
			if (cn_meta->external_nodes)
				goto emit_entry;
			if (slen + cn->len > max_depth)
				goto emit_entry; /* Would exceed depth. */
			for (j = 0; j < cn->len; j++)
				suffix_buf[slen++] = cn->key_bytes[j];
			walk = ft_dereference_acquire(cn->child);
			continue;
		}

		/* Already-collapsed: emit intermediate entry. */
		if (ft_node_collapsed(walk))
			goto emit_entry;

		/* Internal node. */
		if (ft_node_internal(walk)) {
			struct cds_ft_metadata *wm =
				cds_ft_item_to_metadata(ft_node_ptr(walk));

			/*
			 * Single child: extend by 1 byte.  But if
			 * this node has external_nodes (a key ends
			 * at this depth), emit an intermediate entry
			 * to preserve them — can't skip past.
			 */
			if (wm->nr_child == 1 && !wm->external_nodes) {
				uint8_t wk = 0;
				struct cds_ft_inode_flag *wc;

				wc = ft_node_get_minmax(walk, &wk,
					FT_LEFTMOST);
				if (!ft_node_ptr(wc))
					return -1;
				suffix_buf[slen++] = wk;
				walk = wc;
				continue;
			}

			/*
			 * Multi-child: recurse into each child.
			 * Each child path becomes a separate entry.
			 *
			 * If this internal node has external_nodes
			 * (variable-length keys ending at this depth),
			 * don't recurse — emit an intermediate entry
			 * pointing to this node to preserve the
			 * external_nodes.
			 */
			if (wm->external_nodes)
				goto emit_entry;
			{
				uint8_t ck = 0;
				int pv = -1;
				struct cds_ft_inode_flag *c;

				c = ft_node_get_direction(walk, pv,
					&ck, FT_RIGHT);
				while (ft_node_ptr(c)) {
					suffix_buf[slen] = ck;
					if (ft_collapse_walk_subtree(ft,
							c, suffix_buf,
							slen + 1,
							max_depth,
							col, col_ptrs,
							max_entries))
						return -1;
					pv = ck;
					c = ft_node_get_direction(walk,
						pv, &ck, FT_RIGHT);
				}
			}
			return 0;
		}
		return -1; /* Unknown node type. */
	}

	/* Depth limit reached: emit intermediate entry. */
emit_entry:
	{
		unsigned int entry_idx = ft_collapsed_count(ft_collapsed_nr_entries(col));
		unsigned int suffix_start;
		uint8_t *suffix_pos;

		if (slen == 0)
			return -1;
		if (entry_idx >= max_entries)
			return -1;
		if (entry_idx == 0)
			suffix_start = ft_collapsed_scan_zone_size(uatomic_load(&col->nr_entries, CMM_RELAXED));
		else
			suffix_start = col->data[entry_idx - 1]
				& ft_collapsed_offset_mask(uatomic_load(&col->nr_entries, CMM_RELAXED));
		if (1 + entry_idx + 1 + slen > suffix_start)
			return -1;

		suffix_pos = ((uint8_t *) col) + suffix_start - slen;
		memcpy(suffix_pos, suffix_buf, slen);
		col->data[entry_idx] = (uint8_t)(suffix_pos
			- (uint8_t *) col);
		col_ptrs[entry_idx] = walk;
		ft_collapsed_set_nr_entries(col, entry_idx + 1);
		return 0;
	}
}

/*
 * ft_try_collapse_at_node: attempt to collapse the subtree rooted at
 * the given node into a collapsed node.  The node must be internal.
 *
 * Recursively enumerates paths through the subtree (up to
 * FT_NODE_DENSITY_DEPTH levels deep), forking at multi-child
 * internal nodes.  Each path to a leaf (external node) becomes a
 * terminal entry with a full suffix.  Paths that hit the depth
 * limit, an already-collapsed node, or a NULL pointer become
 * intermediate entries.
 *
 * All-or-nothing: if any path can't fit in the scan zone, the
 * entire collapse is aborted and the subtree is left untouched.
 *
 * Returns the collapsed node flag on success, NULL if the subtree
 * doesn't fit or collapsing is not beneficial.
 */
static
struct cds_ft_inode_flag *ft_try_collapse_at_node(struct cds_ft *ft,
		struct cds_ft_inode_flag *node_flag,
		unsigned int node_depth __attribute__((unused)))
{
	struct cds_ft_inode *node = ft_node_ptr(node_flag);
	struct cds_ft_metadata *metadata = cds_ft_item_to_metadata(node);
	struct cds_ft_collapsed_node *col;
	struct cds_ft_metadata *col_meta;
	struct cds_ft_inode_flag **col_ptrs;
	unsigned int nr_child = metadata->nr_child;
	unsigned int max_entries;
	uint8_t child_key = 0;
	int pivot;
	struct cds_ft_inode_flag *child;
	uint8_t suffix_buf[FT_COLLAPSED_SCAN_ZONE_MAX];

	/* Quick pre-filter: need at least 2 children. */
	if (nr_child < 2)
		return NULL;

	/*
	 * Skip-compressed children are free on the lookup fast path
	 * (zero cache line cost).  Collapsing a node that has any
	 * skip-compressed children would replace those free paths
	 * with costly collapsed suffix scans — never profitable.
	 */
	/*
	 * Select collapsed node configuration using density counters.
	 *
	 * Cost/benefit gate: density must exceed nr_child by a
	 * factor of 1.5.  Each collapsed entry adds scan overhead
	 * (tombstone check, offset compute, suffix compare) that
	 * must be offset by saving pointer chases.  density[0]
	 * counts traversable nodes absorbed; nr_child is roughly
	 * the number of entries.  When the ratio is too low (e.g.
	 * subtree is mostly compressed paths that each cost only
	 * 1 pointer chase), the collapsed scan overhead exceeds
	 * the savings.
	 *
	 * Steps:
	 * 1. Density → allocation order (smallest that fits).
	 * 2. Walk with 64B scan zone (most restrictive scan budget,
	 *    most generous pointer count for the allocation).
	 * 3. Post-walk: compute min suffix length across all entries.
	 * 4. If min_slen qualifies for a wider scan zone within the
	 *    same allocation AND the entry count fits the wider zone's
	 *    pointer budget, upgrade by copying entries to a new node
	 *    with the wider scan zone.  No re-walk needed.
	 */
	{
		unsigned long density = ft_density_get(metadata, 0);

		/*
		 * Skip collapse if the subtree doesn't have enough
		 * intermediate nodes per child to justify the per-entry
		 * scan cost.  density < nr_child * factor means the
		 * subtree is mostly direct paths (compressed or
		 * single-hop) that are cheaper uncollapsed.
		 */
		/*
		 * Weighted density: a traversable node at distance k
		 * saves k pointer chases when absorbed into a collapsed
		 * suffix.  Weight each depth level accordingly:
		 *
		 *   nodes_at_dist_1 = d[0] - sum(d[1..5])
		 *   weighted = nodes_at_dist_1 * 1
		 *            + d[1]*2 + d[2]*3 + d[3]*4 + d[4]*5 + d[5]*6
		 *
		 * Simplifies to: d[0] + d[1] + 2*d[2] + ... + 5*d[5]
		 *
		 * Compare to nr_child (total children including external
		 * leaves).  External children contribute 0 to weighted
		 * density but 1 to nr_child, so they act as a scan-cost
		 * penalty.  Collapse when the weighted hop savings from
		 * traversable children outweigh the scan cost of all
		 * entries.
		 */
		{
			unsigned long weighted = density;
			unsigned int j;

			for (j = 1; j < FT_NODE_DENSITY_DEPTH; j++)
				weighted += j * ft_density_get(metadata, j);
			if (weighted < (unsigned long) nr_child)
				return NULL;
		}
		unsigned int order;
		int try_depth;
		bool found = false;

		if (density <= ft_collapsed_max_entries(FT_COLLAPSED_ORDER_SMALL, FT_COLLAPSED_SCAN_64))
			order = FT_COLLAPSED_ORDER_SMALL;
		else if (density <= ft_collapsed_max_entries(FT_COLLAPSED_ORDER_MEDIUM, FT_COLLAPSED_SCAN_64))
			order = FT_COLLAPSED_ORDER_MEDIUM;
		else if (density <= ft_collapsed_max_entries(FT_COLLAPSED_ORDER_XLARGE, FT_COLLAPSED_SCAN_256))
			order = FT_COLLAPSED_ORDER_XLARGE;
		else
			return NULL;

		max_entries = ft_collapsed_max_entries(order, FT_COLLAPSED_SCAN_64);
		if (nr_child > max_entries)
			order = FT_COLLAPSED_ORDER_MEDIUM;
		max_entries = ft_collapsed_max_entries(order, FT_COLLAPSED_SCAN_64);
		if (nr_child > max_entries)
			order = FT_COLLAPSED_ORDER_XLARGE;
		max_entries = ft_collapsed_max_entries(order, FT_COLLAPSED_SCAN_64);
		if (nr_child > max_entries)
			return NULL;

		col = alloc_collapsed_node(ft, order, FT_COLLAPSED_SCAN_64, &col_meta);
		if (!col)
			return NULL;
		col_ptrs = ft_collapsed_ptrs(col,
			uatomic_load(&col->nr_entries, CMM_RELAXED));

		/*
		 * Try recursive leaf-path enumeration,
		 * progressively reducing the max depth from
		 * FT_NODE_DENSITY_DEPTH down to 2.
		 *
		 * The walk depth is bounded by FT_NODE_DENSITY_DEPTH
		 * (6 levels) to stay within the density counter
		 * horizon.  Collapsed suffixes spanning more than 6
		 * levels would break density counter propagation and
		 * explode reconstruction.
		 */
		for (try_depth = FT_NODE_DENSITY_DEPTH;
		     try_depth >= 2; try_depth--) {
			bool walk_ok = true;

			if (ft_density_get(metadata, 0) > max_entries)
				break;

			/* Reset for this depth attempt. */
			memset(col->data, 0,
				ft_collapsed_scan_zone_size(uatomic_load(&col->nr_entries, CMM_RELAXED)) - 1);
			ft_collapsed_set_nr_entries(col, 0);

			pivot = -1;
			child = ft_node_get_direction(node_flag,
				pivot, &child_key, FT_RIGHT);
			while (ft_node_ptr(child)) {
				suffix_buf[0] = child_key;
				if (ft_collapse_walk_subtree(ft,
						child, suffix_buf, 1,
						(unsigned int) try_depth,
						col, col_ptrs,
						max_entries)) {
					walk_ok = false;
					break;
				}
				pivot = child_key;
				child = ft_node_get_direction(
					node_flag, pivot,
					&child_key, FT_RIGHT);
			}
			if (walk_ok &&
			    ft_collapsed_count(ft_collapsed_nr_entries(col)) >= 2) {
				found = true;
				break;
			}
		}

		if (!found || ft_collapsed_count(ft_collapsed_nr_entries(col)) < 2) {
			free_collapsed_node(ft, col);
			return NULL;
		}

		/*
		 * Post-check and scan zone upgrade.
		 *
		 * Compute min suffix length across all entries.
		 * Require at least one entry with slen >= SUFFIX_MIN.
		 * Then try to upgrade to a wider scan zone within the
		 * same allocation order if suffix quality allows.
		 *
		 * This node was freshly built by
		 * ft_collapse_walk_subtree from an internal node's
		 * live children — no tombstoned entries exist.
		 * The reformat below therefore copies all entries
		 * without tombstone filtering.
		 */
		{
			unsigned int e, nr;
			unsigned int min_slen_seen = UINT_MAX;
			bool has_long_suffix = false;
			unsigned int best_scan_sel = FT_COLLAPSED_SCAN_64;

			nr = ft_collapsed_nr_entries(col);
			for (e = 0; e < ft_collapsed_count(nr); e++) {
				uint8_t data_e = ft_collapsed_load_data(col, e);
				unsigned int slen =
					ft_collapsed_suffix_len(col, data_e, e, nr);

				/* Fresh node: no tombstones expected. */
				assert(!ft_collapsed_entry_dead(data_e, nr));

				if (slen < min_slen_seen)
					min_slen_seen = slen;
				if (slen >= FT_COLLAPSE_SUFFIX_MIN)
					has_long_suffix = true;
			}
			if (!has_long_suffix) {
				free_collapsed_node(ft, col);
				return NULL;
			}

			/*
			 * Try to reformat into a better scan zone.
			 *
			 * TINY compaction (checked first): if entries
			 * fit in 32B scan budget, compact to order 6.
			 * Saves 1 CL load (scan+ptrs in 1 cache line).
			 *
			 * Wider zones: if all suffixes justify the
			 * extra CL loads, upgrade to 128B or 256B scan
			 * within the same allocation order.
			 */
			{
				unsigned int best_order = order;
				unsigned int old_scan_sz =
					FT_COLLAPSED_SCAN_ZONE_SIZE;
				unsigned int suffix_start, total_suffix;
				unsigned int header_end;

				if (ft_collapsed_count(nr) > 0)
					suffix_start = col->data[ft_collapsed_count(nr) - 1]
						& FT_COLLAPSED_OFFSET_MASK;
				else
					suffix_start = old_scan_sz;
				total_suffix = old_scan_sz - suffix_start;
				header_end = 1 + ft_collapsed_count(nr);

				/* TINY: 32B scan, order 6 (64B alloc). */
				if (ft_collapsed_count(nr) <= ft_collapsed_max_entries(
					    FT_COLLAPSED_ORDER_TINY,
					    FT_COLLAPSED_SCAN_32) &&
				    header_end + total_suffix <= 32) {
					best_scan_sel = FT_COLLAPSED_SCAN_32;
					best_order = FT_COLLAPSED_ORDER_TINY;
				}
				/* 256B scan within same order. */
				else if (order >= FT_COLLAPSED_ORDER_XLARGE &&
					 min_slen_seen >= ft_collapsed_min_slen(
						FT_COLLAPSED_SCAN_256) &&
					 ft_collapsed_count(nr) <= ft_collapsed_max_entries(order,
						FT_COLLAPSED_SCAN_256))
					best_scan_sel = FT_COLLAPSED_SCAN_256;
				/* 128B scan within same order. */
				else if (order >= FT_COLLAPSED_ORDER_MEDIUM &&
					 min_slen_seen >= ft_collapsed_min_slen(
						FT_COLLAPSED_SCAN_128) &&
					 ft_collapsed_count(nr) <= ft_collapsed_max_entries(order,
						FT_COLLAPSED_SCAN_128))
					best_scan_sel = FT_COLLAPSED_SCAN_128;

				if (best_scan_sel != FT_COLLAPSED_SCAN_64) {
					struct cds_ft_collapsed_node *new_col;
					struct cds_ft_metadata *new_meta;
					struct cds_ft_inode_flag **new_ptrs;
					unsigned int new_scan_sz =
						ft_collapsed_scan_sizes[best_scan_sel];

					new_col = alloc_collapsed_node(ft,
						best_order, best_scan_sel,
						&new_meta);
					if (!new_col)
						goto skip_reformat;
					new_ptrs = ft_collapsed_ptrs(new_col,
						uatomic_load(&new_col->nr_entries, CMM_RELAXED));

					/*
					 * Copy suffix data: shift from old
					 * scan zone end to new scan zone end.
					 */
					memcpy(((uint8_t *) new_col) +
						new_scan_sz - total_suffix,
					       ((uint8_t *) col) + suffix_start,
					       total_suffix);

					/*
					 * Adjust offsets by the scan zone
					 * size difference (may be negative
					 * for compaction to TINY).
					 */
					{
						int shift = (int) new_scan_sz
							  - (int) old_scan_sz;
						for (e = 0; e < ft_collapsed_count(nr); e++)
							new_col->data[e] =
								(col->data[e]
								 & FT_COLLAPSED_OFFSET_MASK)
								+ shift;
					}

					memcpy(new_ptrs, col_ptrs,
					       ft_collapsed_count(nr) * sizeof(*new_ptrs));
					ft_collapsed_set_nr_entries(new_col, ft_collapsed_count(nr));

					free_collapsed_node(ft, col);
					col = new_col;
					col_meta = new_meta;
					col_ptrs = new_ptrs;
				}
			skip_reformat:;
			}
		}

		if (ft_debug_counters())
			fprintf(stderr, "COLLAPSE: order=%u scan=%uB entries=%u\n",
				order,
				ft_collapsed_scan_zone_size(uatomic_load(&col->nr_entries, CMM_RELAXED)),
				ft_collapsed_count(ft_collapsed_nr_entries(col)));
	}

	col_meta->nr_child = ft_collapsed_count(ft_collapsed_nr_entries(col));
	uatomic_store(&col_meta->nr_keys, metadata->nr_keys, CMM_RELAXED);
	col_meta->external_nodes = metadata->external_nodes;

	{
		struct cds_ft_inode_flag *col_flag = ft_collapsed_node_flag(col);
		unsigned int e;

		for (e = 0; e < col_meta->nr_child; e++) {
			ft_set_parent(col_ptrs[e], col_flag);
			if (ft_node_skip_compressed(col_ptrs[e])) {
				struct cds_ft_compressed_node *scn =
					ft_skip_to_compressed(col_ptrs[e]);
				cds_ft_item_to_metadata(
					(struct cds_ft_inode *) scn)->skip_slot =
					&col_ptrs[e];
			}
			/*
			 * Re-publish compressed entries as skip pointers.
			 *
			 * The collapse walk resolves skip pointers to
			 * compressed flags (ft_collapse_walk_subtree),
			 * so entries store compressed_flag.
			 * But the compressed node's skip_slot still
			 * points to the old parent (internal node) slot,
			 * which is freed when the collapse publishes.
			 * Convert the entry back to a skip pointer and
			 * redirect skip_slot to the collapsed entry slot,
			 * preserving the skip-compressed optimization for
			 * lookups through this entry while preventing
			 * use-after-free.
			 */
			if (ft_node_compressed(col_ptrs[e]) &&
			    ft_group_skip_compressed(ft->group)) {
				struct cds_ft_compressed_node *cn =
					ft_compressed_node_ptr(col_ptrs[e]);
				if (cn->len <= FT_SKIP_LEN_MAX) {
					col_ptrs[e] = ft_skip_compressed_flag(
						cn->child, cn->len);
					cds_ft_item_to_metadata(
						(struct cds_ft_inode *) cn)->skip_slot =
						&col_ptrs[e];
				}
			}
		}
		return col_flag;
	}
}

/*
 * ft_check_collapse_on_path: after a mutation, walk the key path from
 * root to leaf through the LIVE trie, evaluating each internal node
 * for collapse.  This sees all nodes including freshly created
 * junctions from compressed splits, which may not be in the snapshot.
 *
 * Collapses are applied immediately top-down: when an internal node
 * qualifies, it is replaced with a collapsed node before continuing
 * deeper.  The walk then descends into the collapsed node's matching
 * entry, naturally handling multi-level collapse without a separate
 * collection pass.
 */
static
void ft_check_collapse_on_path(struct cds_ft *ft,
		const uint8_t *key, size_t key_len)
{
	struct cds_ft_inode_flag *node_flag;
	struct cds_ft_inode_flag **parent_slot;
	struct cds_ft_inode_flag *parent_nf = NULL;
	const uint8_t *ik = key;
	unsigned int depth = 0;

	node_flag = ft_dereference_prefetch(ft->root);
	parent_slot = &ft->root;

	while (depth < key_len + 1) {
		if (!ft_node_ptr(node_flag))
			break;
		if (ft_node_external(node_flag))
			break;
		if (ft_node_compressed(node_flag)) {
			struct cds_ft_compressed_node *cn =
				ft_compressed_node_ptr(node_flag);
			unsigned int remaining = key_len - depth;
			unsigned int cmp = cn->len < remaining ?
				cn->len : remaining;
			unsigned int j;

			for (j = 0; j < cmp; j++) {
				if (ik[j] !=
				    cn->key_bytes[j])
					break;
			}
			if (j < cmp)
				break;
			parent_slot = &cn->child;
			parent_nf = node_flag;
			node_flag = ft_dereference_acquire_prefetch(cn->child);
			depth += cn->len;
			ik += cn->len;
			continue;
		}
		if (ft_node_collapsed(node_flag)) {
			struct cds_ft_collapsed_node *col =
				ft_collapsed_node_ptr(node_flag);
			unsigned int remaining = key_len - depth;
			unsigned int e;
			bool found = false;

			unsigned int nr_e = ft_collapsed_nr_entries(col);
			struct cds_ft_inode_flag **cptrs =
				ft_collapsed_ptrs(col, nr_e);

			for (e = 0; e < ft_collapsed_count(nr_e); e++) {
				uint8_t data_e = ft_collapsed_load_data(col, e);
				unsigned int slen, j;
				uint8_t *suffix;

				if (ft_collapsed_entry_dead(data_e, nr_e))
					continue;
				slen = ft_collapsed_suffix_len(col, data_e, e, nr_e);
				if (slen > remaining)
					continue;
				suffix = ft_collapsed_suffix(col, data_e, nr_e);
				for (j = 0; j < slen; j++) {
					if (ik[j] !=
					    suffix[j])
						break;
				}
				if (j == slen) {
					struct cds_ft_inode_flag *entry_child;
					entry_child = ft_dereference_acquire_prefetch(cptrs[e]);
					if (ft_node_skip_compressed(entry_child))
						entry_child = ft_compressed_node_flag(
							ft_skip_to_compressed(entry_child));
					parent_slot = &cptrs[e];
					parent_nf = node_flag;
					node_flag = entry_child;
					depth += slen;
					ik += slen;
					found = true;
					break;
				}
			}
			if (!found)
				break;
			continue;
		}
		/*
		 * Internal node: try to collapse (skip root).
		 * Apply immediately and restart the loop — the
		 * new collapsed node will be handled by the
		 * collapsed branch above on the next iteration.
		 */
		if (depth > 0) {
			struct cds_ft_inode_flag *col_flag;

			col_flag = ft_try_collapse_at_node(ft, node_flag,
					depth);
			if (col_flag) {
				ft_set_parent(col_flag, parent_nf);
				rcu_assign_pointer(*parent_slot, col_flag);
				if (ft_node_compressed(parent_nf)) {
					struct cds_ft_compressed_node *pcn =
						ft_compressed_node_ptr(parent_nf);
					struct cds_ft_metadata *pcn_meta =
						cds_ft_item_to_metadata(
							(struct cds_ft_inode *) pcn);
					if (pcn_meta->skip_slot)
						rcu_assign_pointer(
							*pcn_meta->skip_slot,
							ft_skip_compressed_flag(
								col_flag, pcn->len));
				}
				free_cds_ft_node(ft, ft_node_ptr(node_flag));
				/*
				 * Recompute density for the new collapsed
				 * node: the absorbed intermediate nodes are
				 * gone, only entries' children remain.
				 */
				ft_init_node_density(col_flag);
				node_flag = col_flag;
				/* Restart loop: collapsed handler above
				 * will descend into the matching entry. */
				continue;
			}
		}
		{
			uint8_t kv = *(ik++);
			struct cds_ft_inode_flag **slot = NULL;
			struct cds_ft_inode_flag *child;

			child = ft_node_get_nth(node_flag, &slot, kv);
			if (!slot || !ft_node_ptr(child))
				break;
			parent_slot = slot;
			parent_nf = node_flag;
			node_flag = child;
			depth++;
		}
	}
}
#endif /* FEATURE_FT_COLLAPSE */

/*
 * Split a compressed node during insert when the new key diverges
 * from the compressed path at position @diverge_pos.
 *
 * Builds the following structure bottom-up:
 *
 *   [prefix compressed/internal] → [branch internal]
 *                                    ├─ old_ordinal → [suffix compressed/internal] → old_child
 *                                    └─ new_ordinal → [new branch compressed/internal] → new_leaf
 *
 * If diverge_pos == 0, no prefix is needed.  If the suffix or new
 * branch is 0 bytes, the child is placed directly.  If 1 byte, a
 * single-child internal node is used.  If >= 2 bytes, a compressed
 * node is created.
 *
 * The old compressed node's external_nodes (if any) are preserved
 * at the prefix level (or the branch if no prefix).
 *
 * Publishes the result at @parent_slot via rcu_assign_pointer and
 * frees the old compressed node.  Returns 0 on success, -ENOMEM
 * on allocation failure (compressed node left in place).
 */
static
int ft_split_compressed_insert(struct cds_ft *ft,
		struct cds_ft_inode_flag **parent_slot,
		struct cds_ft_inode_flag *compressed_flag,
		const uint8_t *iter_key,	/* key bytes at compressed node's depth */
		unsigned int remaining_key,	/* key bytes remaining from compressed depth */
		unsigned int diverge_pos,	/* position within compressed path */
		struct cds_ft_node *child_node,	/* new external node to insert */
		unsigned int node_depth)	/* depth of the compressed node */
{
	struct cds_ft_compressed_node *cn = ft_compressed_node_ptr(compressed_flag);
	struct cds_ft_metadata *cn_meta =
		cds_ft_item_to_metadata((struct cds_ft_inode *) cn);
	struct cds_ft_inode_flag *old_suffix_flag, *new_branch_flag;
	struct cds_ft_inode_flag *branch_flag, *top_flag;
	struct cds_ft_inode_flag *created[FT_MAX_DEPTH];
	int nr_created = 0;
	unsigned int suffix_len = cn->len - diverge_pos - 1;
	unsigned int new_len = remaining_key - diverge_pos - 1;
	uint8_t old_ordinal = cn->key_bytes[diverge_pos];
	uint8_t new_ordinal = iter_key[diverge_pos];
	unsigned long old_child_nr_keys;
	int ret;

	/* Compute old child's nr_keys for the new nodes. */
	if (!ft_node_external(cn->child)) {
		struct cds_ft_metadata *cm =
			cds_ft_item_to_metadata(ft_node_ptr(cn->child));
		old_child_nr_keys = cm->nr_keys;
	} else if (ft_node_ptr(cn->child)) {
		old_child_nr_keys = 1;	/* external leaf */
	} else {
		old_child_nr_keys = 0;
	}

	/* 1. Build old suffix → old child. */
	if (suffix_len >= 2) {
		struct cds_ft_compressed_node *sfx;
		struct cds_ft_metadata *sfx_meta;

		sfx = alloc_compressed_node(ft, suffix_len, &sfx_meta);
		if (!sfx) goto error;
		sfx->child = cn->child;
		sfx->len = suffix_len;
		memcpy(sfx->key_bytes, &cn->key_bytes[diverge_pos + 1], suffix_len);
		sfx_meta->nr_child = 1;
		uatomic_store(&sfx_meta->nr_keys, old_child_nr_keys, CMM_RELAXED);
		old_suffix_flag = ft_compressed_node_flag(sfx);
		ft_set_parent(cn->child, old_suffix_flag);
		old_suffix_flag = ft_publish_compressed(ft, sfx, old_suffix_flag);
		created[nr_created++] = old_suffix_flag;
	} else if (suffix_len == 1) {
		struct cds_ft_inode_flag *dest = NULL;

		ret = ft_node_set_nth(ft, &dest, cn->key_bytes[diverge_pos + 1],
				cn->child, NULL, NULL);
		if (ret) goto error;
		{
			struct cds_ft_metadata *m =
				cds_ft_item_to_metadata(ft_node_ptr(dest));
			uatomic_store(&m->nr_keys, old_child_nr_keys, CMM_RELAXED);
		}
		old_suffix_flag = dest;
		created[nr_created++] = dest;
	} else {
		/* suffix_len == 0: old child directly. */
		old_suffix_flag = cn->child;
	}

	/* 2. Build new branch → new leaf. */
	if (new_len >= 2) {
		struct cds_ft_compressed_node *nb;
		struct cds_ft_metadata *nb_meta;

		nb = alloc_compressed_node(ft, new_len, &nb_meta);
		if (!nb) goto error;
		nb->child = (struct cds_ft_inode_flag *) child_node;
		nb->len = new_len;
		{
			unsigned int k;

			for (k = 0; k < new_len; k++)
				nb->key_bytes[k] = iter_key[diverge_pos + 1 + k];
		}
		nb_meta->nr_child = 1;
		uatomic_store(&nb_meta->nr_keys, 1, CMM_RELAXED);
		new_branch_flag = ft_compressed_node_flag(nb);
		ft_set_parent(nb->child, new_branch_flag);
		new_branch_flag = ft_publish_compressed(ft, nb, new_branch_flag);
		created[nr_created++] = new_branch_flag;
	} else if (new_len == 1) {
		struct cds_ft_inode_flag *dest = NULL;

		ret = ft_node_set_nth(ft, &dest,
				iter_key[diverge_pos + 1],
				(struct cds_ft_inode_flag *) child_node, NULL, NULL);
		if (ret) goto error;
		{
			struct cds_ft_metadata *m =
				cds_ft_item_to_metadata(ft_node_ptr(dest));
			uatomic_store(&m->nr_keys, 1, CMM_RELAXED);
		}
		new_branch_flag = dest;
		created[nr_created++] = dest;
	} else {
		/* new_len == 0: child_node directly. */
		new_branch_flag = (struct cds_ft_inode_flag *) child_node;
	}

	/* 3. Build branch node with both children. */
	{
		struct cds_ft_inode_flag *dest = NULL;
		struct cds_ft_metadata *branch_meta;

		/* First child: old direction. */
		ret = ft_node_set_nth(ft, &dest, old_ordinal, old_suffix_flag, NULL, NULL);
		if (ret) goto error;
		created[nr_created++] = dest;
		branch_flag = dest;

		/* Second child: new direction. */
		{
			struct cds_ft_inode *old_recompacted = NULL;

			branch_meta = cds_ft_item_to_metadata(ft_node_ptr(dest));
			ret = ft_node_set_nth(ft, &dest, new_ordinal, new_branch_flag,
					&old_recompacted, branch_meta);
			if (ret) goto error;
			if (old_recompacted) {
				free_cds_ft_node(ft, old_recompacted);
				/* Update created entry to the recompacted node. */
				created[nr_created - 1] = dest;
			}
		}

		branch_meta = cds_ft_item_to_metadata(ft_node_ptr(dest));
		uatomic_store(&branch_meta->nr_keys, old_child_nr_keys + 1,
				CMM_RELAXED);
		branch_flag = dest;
	}

	/* 4. Build prefix → branch (if needed). */
	if (diverge_pos >= 2) {
		struct cds_ft_compressed_node *pfx;
		struct cds_ft_metadata *pfx_meta;

		pfx = alloc_compressed_node(ft, diverge_pos, &pfx_meta);
		if (!pfx) goto error;
		pfx->child = branch_flag;
		pfx->len = diverge_pos;
		memcpy(pfx->key_bytes, cn->key_bytes, diverge_pos);
		pfx_meta->nr_child = 1;
		uatomic_store(&pfx_meta->nr_keys, cn_meta->nr_keys + 1, CMM_RELAXED);
		if (cn_meta->external_nodes)
			pfx_meta->external_nodes = cn_meta->external_nodes;
		top_flag = ft_compressed_node_flag(pfx);
		ft_set_parent(branch_flag, top_flag);
		top_flag = ft_publish_compressed(ft, pfx, top_flag);
		created[nr_created++] = top_flag;
	} else if (diverge_pos == 1) {
		struct cds_ft_inode_flag *dest = NULL;
		struct cds_ft_metadata *pfx_meta;

		ret = ft_node_set_nth(ft, &dest, cn->key_bytes[0],
				branch_flag, NULL, NULL);
		if (ret) goto error;
		pfx_meta = cds_ft_item_to_metadata(ft_node_ptr(dest));
		uatomic_store(&pfx_meta->nr_keys, cn_meta->nr_keys + 1, CMM_RELAXED);
		if (cn_meta->external_nodes)
			pfx_meta->external_nodes = cn_meta->external_nodes;
		top_flag = dest;
		created[nr_created++] = dest;
	} else {
		/* diverge_pos == 0: branch IS the top. */
		struct cds_ft_metadata *branch_meta =
			cds_ft_item_to_metadata(ft_node_ptr(branch_flag));
		uatomic_store(&branch_meta->nr_keys, cn_meta->nr_keys + 1,
				CMM_RELAXED);
		if (cn_meta->external_nodes)
			branch_meta->external_nodes = cn_meta->external_nodes;
		top_flag = branch_flag;
	}

	/*
	 * Compute cn_parent_depth BEFORE publishing.  After publish,
	 * *parent_slot holds the new top_flag and the old compressed
	 * node is no longer reachable from the parent's entries.
	 * ft_parent_depth_span would fail to find it.
	 */
	unsigned int junction_depth = node_depth + diverge_pos;
	struct cds_ft_inode_flag *cn_parent = cn_meta->parent;
	unsigned int cn_parent_depth = cn_parent ?
		node_depth - ft_parent_depth_span(cn_parent,
			compressed_flag) : 0;

	/* 5. Publish the split structure, replacing the compressed node. */
	rcu_assign_pointer(*parent_slot, top_flag);
	if (ft_node_skip_compressed(top_flag)) {
		struct cds_ft_compressed_node *top_cn =
			ft_skip_to_compressed(top_flag);
		cds_ft_item_to_metadata(
			(struct cds_ft_inode *) top_cn)->skip_slot =
			parent_slot;
	}

	/*
	 * 6. Density: propagate each new node at its actual depth.
	 *
	 * The old compressed node at node_depth is replaced by the
	 * prefix (or junction if diverge_pos == 0): net 0 at
	 * node_depth.  The junction, old suffix, and new branch
	 * are NEW nodes at deeper depths.
	 *
	 * This is critical for long compressed paths (e.g. file
	 * paths): when diverge_pos is large, the junction is deep
	 * and its density must reach ancestors near the junction
	 * depth, not the compressed node's original depth.
	 */
	/*
	 * Initialize density counters on all created nodes, bottom-up
	 * (children first so parents see correct child counters).
	 */
	{
		int ci;

		for (ci = 0; ci < nr_created; ci++)
			ft_init_node_density(created[ci]);
	}

	{

		/*
		 * Junction: new node at junction_depth.
		 * If diverge_pos == 0, the junction replaces the old
		 * compressed node at node_depth (net 0).
		 */
		if (diverge_pos > 0 && cn_parent)
			ft_propagate_node_density_parent(cn_parent,
				cn_parent_depth, junction_depth, 1);
		/* Old suffix at junction_depth + 1 (if it exists). */
		if (suffix_len > 0 && cn_parent)
			ft_propagate_node_density_parent(cn_parent,
				cn_parent_depth, junction_depth + 1, 1);
		/* New branch at junction_depth + 1 (if it exists). */
		if (new_len > 0 && cn_parent)
			ft_propagate_node_density_parent(cn_parent,
				cn_parent_depth, junction_depth + 1, 1);
	}

	/* 7. Free the old compressed node. */
	free_compressed_node(ft, cn);

	return 0;

error:
	{
		int i;

		for (i = 0; i < nr_created; i++) {
			if (ft_node_compressed(created[i]))
				free_compressed_node(ft,
					ft_compressed_node_ptr(created[i]));
			else if (ft_node_skip_compressed(created[i]))
				free_compressed_node(ft,
					ft_skip_to_compressed(created[i]));
			else
				free_cds_ft_node(ft, ft_node_ptr(created[i]));
		}
	}
	return -ENOMEM;
}


/*
 * Split a compressed node when the insert key is shorter than the
 * compressed path (key terminates within the path).
 *
 * Builds: [prefix] → [junction] → [suffix] → old_child
 *
 * The junction is an internal node at the key endpoint depth with
 * one child (the suffix direction).  The caller stores the new
 * node as external_nodes on the junction and handles publication,
 * propagation, and freeing of the old compressed node.
 *
 * On success, sets *top_ret to the topmost node (prefix or junction)
 * and *jct_ret to the junction node.  Returns 0.
 * On failure, frees any partially created nodes and returns -ENOMEM.
 */
static
int ft_split_compressed_key_shorter(struct cds_ft *ft,
		struct cds_ft_inode_flag *compressed_flag,
		unsigned int remaining,
		struct cds_ft_inode_flag **top_ret,
		struct cds_ft_inode_flag **jct_ret)
{
	struct cds_ft_compressed_node *cn = ft_compressed_node_ptr(compressed_flag);
	struct cds_ft_metadata *cn_meta =
		cds_ft_item_to_metadata((struct cds_ft_inode *) cn);
	unsigned int suffix_len = cn->len - remaining - 1;
	struct cds_ft_inode_flag *suffix_flag;
	struct cds_ft_inode_flag *jct_flag;
	struct cds_ft_inode_flag *top_flag;
	struct cds_ft_inode_flag *created[FT_MAX_DEPTH];
	int nr_created = 0;
	unsigned long child_nr_keys;
	int ret;

	if (!ft_node_external(cn->child)) {
		struct cds_ft_metadata *cm =
			cds_ft_item_to_metadata(ft_node_ptr(cn->child));
		child_nr_keys = cm->nr_keys;
	} else if (ft_node_ptr(cn->child)) {
		child_nr_keys = 1;
	} else {
		child_nr_keys = 0;
	}

	/* Build suffix → old child. */
	if (suffix_len >= 2) {
		struct cds_ft_compressed_node *sfx;
		struct cds_ft_metadata *sfx_meta;

		sfx = alloc_compressed_node(ft, suffix_len, &sfx_meta);
		if (!sfx) goto error;
		sfx->child = cn->child;
		sfx->len = suffix_len;
		memcpy(sfx->key_bytes, &cn->key_bytes[remaining + 1],
			suffix_len);
		sfx_meta->nr_child = 1;
		uatomic_store(&sfx_meta->nr_keys, child_nr_keys,
			CMM_RELAXED);
		suffix_flag = ft_compressed_node_flag(sfx);
		ft_set_parent(cn->child, suffix_flag);
		suffix_flag = ft_publish_compressed(ft, sfx, suffix_flag);
		created[nr_created++] = suffix_flag;
	} else if (suffix_len == 1) {
		struct cds_ft_inode_flag *dest = NULL;

		ret = ft_node_set_nth(ft, &dest,
			cn->key_bytes[remaining + 1],
			cn->child, NULL, NULL);
		if (ret) goto error;
		{
			struct cds_ft_metadata *m =
				cds_ft_item_to_metadata(ft_node_ptr(dest));
			uatomic_store(&m->nr_keys, child_nr_keys,
				CMM_RELAXED);
		}
		suffix_flag = dest;
		created[nr_created++] = dest;
	} else {
		suffix_flag = cn->child;
	}

	/* Junction: internal node with suffix child. */
	{
		struct cds_ft_inode_flag *dest = NULL;
		struct cds_ft_metadata *jct_meta;

		ret = ft_node_set_nth(ft, &dest,
			cn->key_bytes[remaining],
			suffix_flag, NULL, NULL);
		if (ret) goto error;
		jct_meta = cds_ft_item_to_metadata(ft_node_ptr(dest));
		uatomic_store(&jct_meta->nr_keys, child_nr_keys,
			CMM_RELAXED);
		jct_flag = dest;
		created[nr_created++] = dest;
	}

	/* Prefix → junction. */
	if (remaining >= 2) {
		struct cds_ft_compressed_node *pfx;
		struct cds_ft_metadata *pfx_meta;

		pfx = alloc_compressed_node(ft, remaining, &pfx_meta);
		if (!pfx) goto error;
		pfx->child = jct_flag;
		pfx->len = remaining;
		memcpy(pfx->key_bytes, cn->key_bytes, remaining);
		pfx_meta->nr_child = 1;
		uatomic_store(&pfx_meta->nr_keys, cn_meta->nr_keys,
			CMM_RELAXED);
		if (cn_meta->external_nodes)
			pfx_meta->external_nodes = cn_meta->external_nodes;
		top_flag = ft_compressed_node_flag(pfx);
		ft_set_parent(jct_flag, top_flag);
		top_flag = ft_publish_compressed(ft, pfx, top_flag);
		created[nr_created++] = top_flag;
	} else {
		/* remaining == 1 */
		struct cds_ft_inode_flag *dest = NULL;
		struct cds_ft_metadata *pfx_meta;

		ret = ft_node_set_nth(ft, &dest, cn->key_bytes[0],
			jct_flag, NULL, NULL);
		if (ret) goto error;
		pfx_meta = cds_ft_item_to_metadata(ft_node_ptr(dest));
		uatomic_store(&pfx_meta->nr_keys, cn_meta->nr_keys,
			CMM_RELAXED);
		if (cn_meta->external_nodes)
			pfx_meta->external_nodes = cn_meta->external_nodes;
		top_flag = dest;
		created[nr_created++] = dest;
	}

	*top_ret = top_flag;
	*jct_ret = jct_flag;
	return 0;

error:
	{
		int i;

		for (i = 0; i < nr_created; i++) {
			if (ft_node_compressed(created[i]))
				free_compressed_node(ft,
					ft_compressed_node_ptr(created[i]));
			else
				free_cds_ft_node(ft,
					ft_node_ptr(created[i]));
		}
	}
	return -ENOMEM;
}

/*
 * We reached an unpopulated node. Create it and the children we need,
 * and then attach the entire branch to the current node. This may
 * trigger recompaction of the current node.
 *
 * ft_attach_node() ensures that a lookup will _never_ see a branch that
 * leads to a dead-end: before attaching a branch, the entire content of
 * the new branch is populated, thus creating a cluster, before
 * attaching the cluster to the rest of the tree, thus making it visible
 * to lookups.
 *
 * @external_node argument is either NULL or a pointer to the external
 * node we are replacing at the attachment location. We need to chain
 * this external node in the topmost internal node external node list in
 * that case.
 */
static struct cds_ft_inode_flag *ft_build_branch(struct cds_ft *ft,
		const uint8_t *key, unsigned int start, unsigned int end,
		struct cds_ft_inode_flag *leaf,
		unsigned long subtree_external_count);

/*
 * Try to create a compressed path for a chain of single-child nodes.
 * Returns the compressed node flag on success, NULL if compression
 * is not applicable (path too short) or disabled, -ENOMEM cast to
 * pointer on allocation failure.
 */
#ifdef FEATURE_FT_COMPRESS
static
struct cds_ft_inode_flag *ft_try_compress_chain(struct cds_ft *ft,
		const uint8_t *key, size_t key_len, unsigned int level,
		struct cds_ft_inode_flag *child,
		struct cds_ft_node *external_nodes)
{
	uint8_t path_len = (uint8_t)(key_len - level);
	struct cds_ft_compressed_node *cn;
	struct cds_ft_metadata *cn_meta;
	int j;

	if (path_len < 2)
		return NULL;
	cn = alloc_compressed_node(ft, path_len, &cn_meta);
	if (!cn)
		return (struct cds_ft_inode_flag *) (long) -ENOMEM;
	cn->child = child;
	cn->len = path_len;
	for (j = 0; j < path_len; j++)
		cn->key_bytes[j] = key[level + j];
	cn_meta->nr_child = 1;
	uatomic_store(&cn_meta->nr_keys, 1, CMM_RELAXED);
	if (external_nodes) {
		cn_meta->external_nodes = external_nodes;
		uatomic_store(&cn_meta->nr_keys,
			cn_meta->nr_keys + 1, CMM_RELAXED);
	}
	{
		struct cds_ft_inode_flag *cflag = ft_compressed_node_flag(cn);
		ft_set_parent(child, cflag);
		ft_init_node_density(cflag);
		return ft_publish_compressed(ft, cn, cflag);
	}
}
#else
static inline
struct cds_ft_inode_flag *ft_try_compress_chain(
		struct cds_ft *ft __attribute__((unused)),
		const uint8_t *key __attribute__((unused)),
		size_t key_len __attribute__((unused)),
		unsigned int level __attribute__((unused)),
		struct cds_ft_inode_flag *child __attribute__((unused)),
		struct cds_ft_node *external_nodes __attribute__((unused)))
{
	return NULL;
}
#endif

static struct cds_ft_inode_flag *ft_build_ordinal_chain(struct cds_ft *ft,
		const uint8_t *ordinals, unsigned int len,
		struct cds_ft_inode_flag *child,
		unsigned long nr_keys);

static struct cds_ft_inode_flag *ft_explode_entries(struct cds_ft *ft,
		struct cds_ft_collapsed_node *col,
		struct cds_ft_inode_flag **cptrs,
		unsigned int start, unsigned int end,
		unsigned int suffix_offset);

static
int ft_attach_node(struct cds_ft *ft,
		struct cds_ft_inode_flag **attach_node_flag_ptr,
		struct cds_ft_inode_flag *attach_node_flag,
		struct cds_ft_inode_flag **old_node_flag_ptr,
		struct cds_ft_inode_flag *old_node_flag,
		const uint8_t *key,
		size_t key_len,
		unsigned int level,
		struct cds_ft_node *child_node,
		struct cds_ft_node *external_nodes)
{
	struct cds_ft_metadata *metadata = NULL;
	struct cds_ft_inode_flag *iter_node_flag, *iter_dest_node_flag,
				*created_nodes[FT_MAX_DEPTH];
	struct cds_ft_inode *old_recompacted_node = NULL;
	int ret, i, nr_created_nodes = 0;
	const uint8_t *iter_key = key + key_len;

	dbg_printf("Attach node at level %u (old_node_flag %p, attach_node_flag_ptr %p attach_node_flag %p)\n",
		level, old_node_flag, attach_node_flag_ptr, attach_node_flag);

	assert(!old_node_flag || external_nodes);
	assert(level > 0);	/* Root is always internal; level 0 is handled directly. */
	if (attach_node_flag)
		metadata = cds_ft_item_to_metadata(ft_node_ptr(attach_node_flag));

	/* Concurrent update prevented by mutual exclusion. */
	assert(!(old_node_flag_ptr && (ft_node_ptr(*old_node_flag_ptr) && !external_nodes)));

	/* Concurrent update prevented by mutual exclusion. */
	assert(!(attach_node_flag_ptr && ft_node_ptr(*attach_node_flag_ptr) !=
			ft_node_ptr(attach_node_flag)));

	/* Create new branch, starting from bottom */
	iter_node_flag = (struct cds_ft_inode_flag *) child_node;

	{
		struct cds_ft_inode_flag *compressed;

		compressed = ft_try_compress_chain(ft, key, key_len,
			level, iter_node_flag, external_nodes);
		if (compressed == (void *) (long) -ENOMEM) {
			ret = -ENOMEM;
			goto check_error;
		}
		if (compressed) {
			iter_node_flag = compressed;
			created_nodes[nr_created_nodes++] = iter_node_flag;
			iter_key = key + level;
		}
	}
	if (!ft_node_compressed(iter_node_flag) &&
	    !ft_node_skip_compressed(iter_node_flag)) {
		for (i = key_len; i > (int) level; i--) {
			uint8_t key_value;

			key_value = *(--iter_key);
			dbg_printf("branch creation level %d, key %u\n",
					i, (unsigned int) key_value);
			iter_dest_node_flag = NULL;
			ret = ft_node_set_nth(ft, &iter_dest_node_flag, key_value, iter_node_flag, NULL, NULL);
			if (ret) {
				dbg_printf("branch creation error %d\n", ret);
				goto check_error;
			}
			{
				struct cds_ft_metadata *branch_meta =
					cds_ft_item_to_metadata(ft_node_ptr(iter_dest_node_flag));
				uatomic_store(&branch_meta->nr_keys, 1, CMM_RELAXED);
			}
			created_nodes[nr_created_nodes++] = iter_dest_node_flag;
			iter_node_flag = iter_dest_node_flag;
		}

		if (external_nodes) {
			struct cds_ft_metadata *iter_node_metadata;

			iter_node_metadata = cds_ft_item_to_metadata(ft_node_ptr(iter_node_flag));
			iter_node_metadata->external_nodes = external_nodes;
			uatomic_store(&iter_node_metadata->nr_keys,
				iter_node_metadata->nr_keys + 1, CMM_RELAXED);
		}
	}

	/* Publish branch. */
	{
		uint8_t key_value;

		key_value = *(--iter_key);
		dbg_printf("publish branch at level %d, key %u\n", level - 1, (unsigned int) key_value);

#ifdef FEATURE_FT_COLLAPSE
		/*
		 * If the parent is a collapsed node, publish the
		 * branch directly at the entry's child pointer.
		 * The collapsed entry's suffix already covers the
		 * path from the collapsed node to the child depth
		 * — only the child itself changes.
		 *
		 * Exploding the collapsed node and using
		 * ft_node_set_nth would attach at the wrong depth
		 * (last suffix byte vs. first-byte level),
		 * overwriting sibling entries that share the same
		 * first suffix byte.
		 */
		if (attach_node_flag &&
		    ft_node_collapsed(attach_node_flag)) {
			ft_set_parent(iter_node_flag, attach_node_flag);
			rcu_assign_pointer(*old_node_flag_ptr,
				iter_node_flag);
			goto publish_done;
		}
#endif

		/* We need to use set_nth on the previous level. */
		iter_dest_node_flag = attach_node_flag;
		ret = ft_node_set_nth(ft, &iter_dest_node_flag, key_value, iter_node_flag,
				&old_recompacted_node, metadata);
		if (ret) {
			dbg_printf("branch publish error %d\n", ret);
			goto check_error;
		}
		/*
		 * If the publish target is cn->child and the compressed
		 * node is published as a skip pointer, update the skip
		 * pointer BEFORE cn->child so candidate readers see the
		 * new child before exact/inequality readers do.
		 */
		if (old_recompacted_node && attach_node_flag &&
		    ft_node_compressed(attach_node_flag)) {
			struct cds_ft_compressed_node *cn =
				ft_compressed_node_ptr(attach_node_flag);
			struct cds_ft_metadata *cn_meta =
				cds_ft_item_to_metadata(
					(struct cds_ft_inode *) cn);

			if (cn_meta->skip_slot &&
			    ft_node_skip_compressed(*cn_meta->skip_slot))
				rcu_assign_pointer(*cn_meta->skip_slot,
					ft_skip_compressed_flag(
						iter_dest_node_flag,
						cn->len));
		}
		/* Attach branch (unlink the old node from the trie). */
		rcu_assign_pointer(*attach_node_flag_ptr, iter_dest_node_flag);

		/* Reclaim safely after unlink. */
		if (old_recompacted_node)
			free_cds_ft_node(ft, old_recompacted_node);
	}
publish_done:

	/*
	 * Propagate node density for each created traversable node.
	 * The snapshot contains ancestors with their depths.
	 */
	/*
	 * Initialize density counters bottom-up for each created node.
	 * Since nodes were created bottom-up, child counters are set
	 * before parent.  Then propagate the top node's density to
	 * snapshot ancestors.
	 */
	if (nr_created_nodes > 0) {
		int cn_idx;
		struct cds_ft_inode_flag *top_node =
			created_nodes[nr_created_nodes - 1];
		struct cds_ft_metadata *top_meta;

		for (cn_idx = 0; cn_idx < nr_created_nodes; cn_idx++)
			ft_init_node_density(created_nodes[cn_idx]);
		top_meta = ft_flag_to_metadata(top_node);
		ft_propagate_node_density_parent(top_node, level,
			level, (long)(1 + ft_density_get(top_meta, 0)));
	}

	/* Success */
	ret = 0;

check_error:
	if (ret) {
		for (i = 0; i < nr_created_nodes; i++) {
			if (ft_node_compressed(created_nodes[i]))
				free_compressed_node(ft,
					ft_compressed_node_ptr(created_nodes[i]));
			else
				free_cds_ft_node(ft, ft_node_ptr(created_nodes[i]));
		}
	}
	return ret;
}

static
void ft_chain_node(struct cds_ft_node *last_node, struct cds_ft_node *node)
{
	/*
	 * Add node to tail of list to ensure that RCU traversals will
	 * always see either the prior node or the newly added if
	 * executed concurrently with a sequence of add followed by del
	 * on the same key. Safe against concurrent RCU read traversals.
	 */
	node->next = NULL;
	rcu_assign_pointer(last_node->next, node);
}

/*
 * Advance the descent cursor one level down: rotate current → parent →
 * grandparent, then descend into child @key_value.
 *
 * Returns the new d->nf (the child's flagged pointer, possibly NULL).
 */
static inline
struct cds_ft_inode_flag *ft_descent_step(struct ft_descent *d,
		uint8_t key_value)
{
	d->ppnf  = d->pnf;
	d->ppnfp = d->pnfp;
	d->pnf   = d->nf;
	d->pnfp  = d->nfp;
	d->nf    = ft_node_get_nth(d->pnf, &d->nfp, key_value);
	d->depth++;
	return d->nf;
}

/*
 * Update detach-point pointers based on the node metadata at the
 * current position.  Call BEFORE ft_detach_descent_step().
 *
 * At call time, d.nf is the node being inspected, d.nfp is its slot
 * in its parent, d.pnfp is the parent's slot in the grandparent.
 */
static inline
void ft_detach_descent_track(struct ft_detach_descent *dd,
		const struct cds_ft_metadata *metadata)
{
	if (metadata->nr_child > 1) {
		/*
		 * Multi-child node: upward walk terminates here.
		 * Save this node's slot as the publish point; the
		 * actual det_nfp is captured on the next step
		 * (pending).
		 */
		dd->det_pfp = dd->d.nfp;
		dd->pending = true;
	} else if (dd->d.depth > 0 && metadata->external_nodes) {
		/*
		 * Single-child node with external_nodes: the upward
		 * walk terminates one level above, so save this
		 * node's slot and the parent's slot.
		 */
		dd->det_nfp = dd->d.nfp;
		dd->det_pfp = dd->d.pnfp;
		dd->pending = false;
	}
}

/*
 * Advance one level and resolve any pending detach-point capture.
 * Combines ft_descent_step() with the post-step pending logic.
 */
static inline
struct cds_ft_inode_flag *ft_detach_descent_step(
		struct ft_detach_descent *dd,
		uint8_t key_value)
{
	struct cds_ft_inode_flag *nf;

	nf = ft_descent_step(&dd->d, key_value);
	if (nf && dd->pending) {
		dd->det_nfp = dd->d.nfp;
		dd->pending = false;
	}
	return nf;
}

/*
 * There are a few cases to cover for add:
 *
 * 1) There is already an external node at that key. Chain this new node
 *    with the existing node (duplicate).
 * 2) There is already an internal node with associated external node at
 *    that key. Chain this new node with the existing node (duplicate).
 * 3) The traversal ends before reaching the end of the lookup key:
 *    3.1) The last node encountered during traversal is an internal
 *         node. Attach a new cluster as child of this internal node.
 *    3.2) The last node encountered during traversal is an external
 *         node. Need to transform this external node into an internal
 *         node with associated external node, attach a new cluster as
 *         child of this internal node, and populate this new internal
 *         node into the tree to replace the prior external node.
 */

/*
 * ft_insert_compressed_past_child: key continues past a compressed
 * node's external child.  Build a branch below the child and propagate
 * density / external count through the snapshot.
 *
 * Returns 0 on success, -ENOMEM on allocation failure.
 */
static
int ft_insert_compressed_past_child(struct cds_ft *ft,
		struct ft_descent *d,
		const uint8_t *key, size_t key_len,
		struct cds_ft_compressed_node *cn,
		struct cds_ft_node *node)
{
	struct cds_ft_inode_flag *branch;
	struct cds_ft_metadata *br_meta;

	branch = ft_build_branch(ft, key,
		d->depth + cn->len, key_len,
		(struct cds_ft_inode_flag *) node, 1);
	if (!branch)
		return -ENOMEM;
	br_meta = cds_ft_item_to_metadata(ft_node_ptr(branch));
	br_meta->external_nodes = (struct cds_ft_node *) cn->child;
	uatomic_store(&br_meta->nr_keys,
		br_meta->nr_keys + 1, CMM_RELAXED);
	ft_set_parent(branch, d->nf);
	rcu_assign_pointer(cn->child, branch);
	/* Propagate density for the new branch node. */
	ft_propagate_node_density_parent(branch,
		d->depth + cn->len, d->depth + cn->len, 1);
	ft_propagate_external_count_parent(branch, 1);
	return 0;
}

/*
 * ft_insert_compressed_diverge: key diverges from the compressed path
 * at position @j.  Split the compressed node and insert the new key.
 *
 * Returns 0 on success, negative errno on failure.
 */
static
int ft_insert_compressed_diverge(struct cds_ft *ft,
		struct ft_descent *d,
		const uint8_t *iter_key,
		unsigned int remaining, unsigned int j,
		struct cds_ft_node *node)
{
	int dret;

	dret = ft_split_compressed_insert(ft,
		d->nfp, d->nf, iter_key, remaining,
		j, node, d->depth);
	if (dret)
		return dret;
	ft_set_parent(*d->nfp, d->pnf);
	ft_propagate_external_count_parent(d->pnf, 1);
	return 0;
}

/*
 * ft_insert_compressed_key_shorter: key ends before the compressed
 * path.  Split the compressed node into prefix -> junction -> suffix,
 * then attach the new external node at the junction.
 *
 * Returns 0 on success, -EEXIST if duplicate detected (with
 * *unique_node_ret set), or negative errno on failure.
 */
static
int ft_insert_compressed_key_shorter(struct cds_ft *ft,
		struct ft_descent *d,
		unsigned int remaining,
		struct cds_ft_node *node,
		struct cds_ft_node **unique_node_ret)
{
	struct cds_ft_inode_flag *top_flag, *jct_flag;
	int sret;

	sret = ft_split_compressed_key_shorter(ft,
		d->nf, remaining, &top_flag, &jct_flag);
	if (sret)
		return sret;
	ft_set_parent(top_flag, d->pnf);
	rcu_assign_pointer(*d->nfp, top_flag);
	{
		struct cds_ft_metadata *jct_meta =
			cds_ft_item_to_metadata(
				ft_node_ptr(jct_flag));
		if (unique_node_ret &&
		    jct_meta->external_nodes) {
			*unique_node_ret =
				jct_meta->external_nodes;
			return -EEXIST;
		}
		node->next = NULL;
		rcu_assign_pointer(
			jct_meta->external_nodes, node);
	}
	/*
	 * Density: net = new nodes - old compressed.
	 * Count: junction(1) + prefix(if exists) + suffix(if len>1)
	 * minus old compressed(1).
	 */
	{
		long net = 0;	/* junction replaces compressed: net 0 */
		struct cds_ft_compressed_node *cn =
			ft_compressed_node_ptr(d->nf);

		if (top_flag != jct_flag)
			net++;	/* prefix added */
		if (cn->len > remaining + 1)
			net++;	/* suffix added */
		if (net != 0)
			ft_propagate_node_density_parent(jct_flag,
				d->depth + remaining, d->depth, net);
	}
	ft_propagate_external_count_parent(jct_flag, 1);
	{
		struct cds_ft_metadata *jct_meta =
			cds_ft_item_to_metadata(
				ft_node_ptr(jct_flag));
		uatomic_store(&jct_meta->nr_keys,
			jct_meta->nr_keys + 1, CMM_RELAXED);
	}
	free_compressed_node(ft, ft_compressed_node_ptr(d->nf));
	return 0;
}

/*
 * Handle a compressed node during insert descent.
 *
 * Full match + internal/compressed child: traverse through.
 * Full match + external child at end of key: break for duplicate handling.
 * Full match + external child, key continues: build branch inline.
 * Key diverges: split via ft_split_compressed_insert.
 * Key shorter: split via ft_split_compressed_key_shorter.
 *
 * Returns CONTINUE, BREAK, or END (with ret set via *ret_p).
 * On END, the caller should goto insert_done.
 * On error, returns END with *ret_p < 0.
 */
static
enum ft_compressed_action ft_insert_compressed(struct cds_ft *ft,
		struct ft_descent *d, const uint8_t **iter_key_p,
		const uint8_t *key, size_t key_len,
		unsigned int key_depth,
		struct cds_ft_node *node,
		struct cds_ft_node **unique_node_ret,
		struct cds_ft_inode_flag **snapshot,
		unsigned int *snapshot_depth,
		int *nr_snapshot_p,
		int *ret_p)
{
	struct cds_ft_compressed_node *cn = ft_compressed_node_ptr(d->nf);
	unsigned int remaining = key_depth - 1 - d->depth;
	unsigned int cmp = cn->len < remaining ? cn->len : remaining;
	unsigned int j;

	j = ft_match_compressed_key(*iter_key_p, cn, cmp);
	if (j == cmp && cn->len <= remaining) {
		/* Full match: traverse through if child is internal,
		 * compressed, or collapsed. */
		if (ft_node_ptr(cn->child) &&
		    (ft_node_internal(cn->child) ||
		     ft_node_compressed(cn->child) ||
		     ft_node_collapsed(cn->child))) {
			ft_snapshot_push(snapshot, snapshot_depth,
				*nr_snapshot_p, d->nf, d->depth);
			ft_descent_traverse_compressed(d, cn, iter_key_p);
			return FT_COMPRESSED_CONTINUE;
		}
		assert(ft_node_ptr(cn->child));
		if (cn->len == remaining) {
			/* Key ends at external child: duplicate. */
			ft_snapshot_push(snapshot, snapshot_depth,
				*nr_snapshot_p, d->nf, d->depth);
			ft_descent_traverse_compressed(d, cn, iter_key_p);
			return FT_COMPRESSED_BREAK;
		}
		/* Key continues past external child: build branch. */
		*ret_p = ft_insert_compressed_past_child(ft, d, key,
			key_len, cn, node);
		return FT_COMPRESSED_END;
	}
	if (j < cmp) {
		/* Key diverges: split at position j. */
		*ret_p = ft_insert_compressed_diverge(ft, d,
			*iter_key_p, remaining, j, node);
		return FT_COMPRESSED_END;
	}
	/* Key shorter: split into prefix -> junction -> suffix. */
	*ret_p = ft_insert_compressed_key_shorter(ft, d, remaining,
		node, unique_node_ret);
	return FT_COMPRESSED_END;
}

/*
 * ft_build_ordinal_chain: create a chain of nodes from an array of
 * ordinal bytes (already mapped, not raw key bytes).  Uses a
 * compressed node if FEATURE_FT_COMPRESS is enabled, otherwise
 * builds a chain of single-child internal nodes.
 *
 * Returns the chain's root flag, or NULL on allocation failure.
 */
static
struct cds_ft_inode_flag *ft_build_ordinal_chain(struct cds_ft *ft,
		const uint8_t *ordinals, unsigned int len,
		struct cds_ft_inode_flag *child,
		unsigned long nr_keys)
{
#ifdef FEATURE_FT_COMPRESS
	if (len >= 2) {
		struct cds_ft_compressed_node *cn;
		struct cds_ft_metadata *cn_meta;
		struct cds_ft_inode_flag *cflag;

		cn = alloc_compressed_node(ft, len, &cn_meta);
		if (!cn)
			return NULL;
		cn->child = child;
		cn->len = len;
		memcpy(cn->key_bytes, ordinals, len);
		cn_meta->nr_child = 1;
		uatomic_store(&cn_meta->nr_keys, nr_keys, CMM_RELAXED);
		cflag = ft_compressed_node_flag(cn);
		ft_set_parent(child, cflag);
		ft_init_node_density(cflag);
		return ft_publish_compressed(ft, cn, cflag);
	}
#endif
	{
		struct cds_ft_inode_flag *cur = child;
		int i;

		for (i = (int) len - 1; i >= 0; i--) {
			struct cds_ft_inode_flag *dest = NULL;
			int ret;

			ret = ft_node_set_nth(ft, &dest, ordinals[i],
				cur, NULL, NULL);
			if (ret) {
				/* Cleanup on failure. */
				while (cur != child) {
					struct cds_ft_inode_flag *next;

					next = ft_node_get_nth(cur, NULL,
						ordinals[i + 1]);
					free_cds_ft_node(ft, ft_node_ptr(cur));
					cur = next;
					i++;
				}
				return NULL;
			}
			{
				struct cds_ft_metadata *m =
					cds_ft_item_to_metadata(
						ft_node_ptr(dest));
				uatomic_store(&m->nr_keys, nr_keys,
					CMM_RELAXED);
				m->nr_child = 1;
			}
			cur = dest;
		}
		ft_init_node_density(cur);
		return cur;
	}
}

#ifdef FEATURE_FT_COLLAPSE
/*
 * ft_explode_entries: rebuild a trie from a range of collapsed node
 * entries [start, end).  Entries must be sorted by suffix.
 * @suffix_offset: number of leading suffix bytes already consumed
 * by parent levels.
 *
 * Returns the root of the rebuilt sub-trie, or NULL on error.
 *
 * For a single entry: creates a compressed path to the child.
 * For multiple entries sharing the same byte at suffix_offset:
 * groups them and recursively builds a sub-trie for each group.
 * Different first bytes get separate children of a new internal node.
 *
 * Recursion depth bounded by max suffix length (FT_NODE_DENSITY_DEPTH).
 */
static
struct cds_ft_inode_flag *ft_explode_entries(struct cds_ft *ft,
		struct cds_ft_collapsed_node *col,
		struct cds_ft_inode_flag **cptrs,
		unsigned int start, unsigned int end,
		unsigned int suffix_offset)
{
	unsigned int count = end - start;
	unsigned int nr_e = ft_collapsed_nr_entries(col);

	if (count == 0)
		return NULL;

	if (count == 1) {
		unsigned int e = start;
		uint8_t data_e = ft_collapsed_load_data(col, e);
		unsigned int slen = ft_collapsed_suffix_len(col, data_e, e, nr_e);
		uint8_t *sfx = ft_collapsed_suffix(col, data_e, nr_e);
		struct cds_ft_inode_flag *child = cptrs[e];
		unsigned long child_nr_keys;

		if (ft_node_skip_compressed(child)) {
			/*
			 * Clear skip_slot: the collapsed node (and its
			 * entry slots) will be freed after the explode.
			 */
			ft_skip_to_compressed_meta(child)->skip_slot =
				NULL;
			child = ft_compressed_node_flag(
				ft_skip_to_compressed(child));
		}
		if (slen <= suffix_offset)
			return child;

		if (!ft_node_external(child) && ft_node_ptr(child)) {
			struct cds_ft_metadata *cm =
				cds_ft_item_to_metadata(ft_node_ptr(child));
			child_nr_keys = uatomic_load(&cm->nr_keys,
				CMM_RELAXED);
		} else {
			child_nr_keys = ft_node_ptr(child) ? 1 : 0;
		}
		return ft_build_ordinal_chain(ft,
			sfx + suffix_offset, slen - suffix_offset,
			child, child_nr_keys);
	}

	/* Multiple entries: group by byte at suffix_offset. */
	{
		struct cds_ft_inode_flag *internal_flag = NULL;
		unsigned int i = start;
		unsigned long total_keys = 0;

		while (i < end) {
			uint8_t data_i = ft_collapsed_load_data(col, i);
			uint8_t *sfx_i = ft_collapsed_suffix(col, data_i, nr_e);
			unsigned int slen_i = ft_collapsed_suffix_len(col, data_i, i, nr_e);
			uint8_t first;
			unsigned int group_end;
			struct cds_ft_inode_flag *sub;

			if (slen_i <= suffix_offset) {
				/* Entry is a leaf at this depth. */
				/* Should not happen with sorted entries. */
				i++;
				continue;
			}

			first = sfx_i[suffix_offset];
			group_end = i + 1;

			while (group_end < end) {
				uint8_t data_g = ft_collapsed_load_data(col, group_end);
				uint8_t *sfx_g =
					ft_collapsed_suffix(col, data_g, nr_e);
				unsigned int slen_g =
					ft_collapsed_suffix_len(col, data_g, group_end, nr_e);

				if (slen_g <= suffix_offset ||
				    sfx_g[suffix_offset] != first)
					break;
				group_end++;
			}

			sub = ft_explode_entries(ft, col, cptrs,
					i, group_end,
					suffix_offset + 1);
			if (!sub)
				return NULL;

			{
				struct cds_ft_inode *old_recompacted = NULL;
				int ret;

				ret = ft_node_set_nth(ft, &internal_flag,
					first, sub, &old_recompacted,
					internal_flag ?
						cds_ft_item_to_metadata(
							ft_node_ptr(internal_flag))
						: NULL);
				if (ret)
					return NULL;
				if (old_recompacted)
					free_cds_ft_node(ft, old_recompacted);
			}

			/* Accumulate key counts. */
			if (ft_node_ptr(sub)) {
				if (ft_node_external(sub))
					total_keys += 1;
				else {
					struct cds_ft_metadata *sm =
						cds_ft_item_to_metadata(
							ft_node_ptr(sub));
					total_keys += uatomic_load(
						&sm->nr_keys, CMM_RELAXED);
				}
			}

			i = group_end;
		}

		if (internal_flag) {
			struct cds_ft_metadata *im =
				cds_ft_item_to_metadata(
					ft_node_ptr(internal_flag));
			uatomic_store(&im->nr_keys, total_keys,
				CMM_RELAXED);
			ft_init_node_density(internal_flag);
		}
		return internal_flag;
	}
}
#endif /* FEATURE_FT_COLLAPSE */

static
int _cds_ft_insert(struct cds_ft *ft,
		const uint8_t *_key, size_t _key_len,
		struct cds_ft_node *node,
		struct cds_ft_node **unique_node_ret)
{
	unsigned int key_depth;
	struct ft_descent d;
	size_t key_len = ft_key_len(ft, _key_len);
	uint8_t ordinal_buf[FT_MAX_KEY_LEN];
	const uint8_t *key = ordinal_buf;
	const uint8_t *iter_key;
	struct cds_ft_inode_flag *snapshot[FT_MAX_DEPTH];
	unsigned int snapshot_depth[FT_MAX_DEPTH]; /* parallel depth tracking */
	int nr_snapshot = 0;
	int ret;

	if (!valid_external_node(node) || !valid_key_len(ft, key_len))
		return -EINVAL;
	ft_key_to_ordinals(ordinal_buf, _key, key_len, &ft->group->key_map);
	iter_key = key;
	/* Expect zeroed next pointer. This catches some double-insert misuses. */
	if (node->next)
		return -EINVAL;

	key_depth = key_len + 1;

	dbg_printf("cds_ft_insert attempt: node %p\n", node);
	ft_descent_init(&d, ft);

	for (; d.depth < key_depth - 1; ) {
		uint8_t key_value;

		if (!ft_node_ptr(d.nf))
			break;
		/*
		 * Resolve skip-compressed pointer.  Can appear after
		 * descending through a collapsed entry whose child was
		 * later split/recompacted, or after a collapse publish
		 * updated the parent's skip pointer.  Convert to the
		 * underlying compressed flag so the compressed handler
		 * below processes it correctly.
		 */
		if (ft_node_skip_compressed(d.nf))
			d.nf = ft_compressed_node_flag(
				ft_skip_to_compressed(d.nf));
		/* Found external node. */
		if (ft_node_external(d.nf))
			break;
		/* Decompress compressed node before continuing descent. */
		/*
		 * Compressed node: compare remaining key bytes with
		 * the compressed path.  If they match, traverse
		 * through to the child.  If they diverge, decompress
		 * at this point and restart.
		 */
		if (ft_node_compressed(d.nf)) {
			enum ft_compressed_action act;

			act = ft_insert_compressed(ft, &d, &iter_key,
				key, key_len, key_depth, node,
				unique_node_ret, snapshot, snapshot_depth,
				&nr_snapshot, &ret);
			if (act == FT_COMPRESSED_END)
				goto insert_done;
			if (act == FT_COMPRESSED_BREAK)
				break;
			continue;
		}
		if (ft_node_collapsed(d.nf)) {
			struct cds_ft_collapsed_node *col =
				ft_collapsed_node_ptr(d.nf);
			unsigned int remaining = key_depth - 1 - d.depth;
			unsigned int e;
			bool found_entry = false;
			int prefix_match_entry = -1;
			unsigned int prefix_match_len = 0;

			unsigned int nr_e = ft_collapsed_nr_entries(col);
			struct cds_ft_inode_flag **cptrs =
				ft_collapsed_ptrs(col, nr_e);

			for (e = 0; e < ft_collapsed_count(nr_e); e++) {
				uint8_t data_e = ft_collapsed_load_data(col, e);
				unsigned int slen, j;
				uint8_t *suffix;

				if (ft_collapsed_entry_dead(data_e, nr_e))
					continue;
				slen = ft_collapsed_suffix_len(col, data_e, e, nr_e);
				suffix = ft_collapsed_suffix(col, data_e, nr_e);

				/* Check for prefix match. */
				{
					unsigned int cmp_len = slen < remaining ? slen : remaining;

					if (ft_key_cmp_ordinals(iter_key, suffix, cmp_len, cmp_len, false, &j) != 0) {
						if (j == 0)
							continue; /* No prefix overlap. */
					} else {
						j = cmp_len; /* Full match up to cmp_len. */
					}
				}

				if (j == slen && slen <= remaining) {
					/* Full suffix match: traverse through. */
					ft_snapshot_push(snapshot, snapshot_depth,
						nr_snapshot, d.nf, d.depth);
					d.ppnf  = d.pnf;
					d.ppnfp = d.pnfp;
					d.pnf   = d.nf;
					d.pnfp  = d.nfp;
					d.nf    = ft_dereference_acquire(cptrs[e]);
					d.nfp   = &cptrs[e];
					d.depth += slen;
					iter_key += slen;
					found_entry = true;
					break;
				}
				/*
				 * Partial prefix match: the new key shares
				 * j bytes with this entry's suffix but diverges
				 * after that.  Record the best (longest) prefix
				 * match for possible entry splitting.
				 */
				if (j > prefix_match_len) {
					prefix_match_entry = (int) e;
					prefix_match_len = j;
				}
			}
			if (found_entry)
				continue;
			/*
			 * Partial prefix match: explode the collapsed node
			 * to avoid creating conflicting entries.
			 * Fall through to the explode-or-break path which
			 * converts to an internal node, then the normal
			 * insert logic handles the remaining key.
			 */
			{
				unsigned int new_slen;
				unsigned int col_nr;
				unsigned int header_end;
				unsigned int cur_suffix_start;
				uint8_t *new_suffix_pos;
				struct cds_ft_inode_flag *branch;
				unsigned int k;
				int tombstone_reuse;

			if (prefix_match_entry >= 0)
				goto collapsed_explode;
			/*
			 * No matching entry.  First check for a
			 * tombstoned entry with the same suffix that
			 * we can reuse.  This avoids creating duplicate
			 * suffixes which would confuse the going-up
			 * inequality backtracking.
			 */
				new_slen = remaining;
				col_nr = ft_collapsed_nr_entries(col);
				header_end = 1 + ft_collapsed_count(col_nr) + 1;
				tombstone_reuse = -1;

				for (k = 0; k < ft_collapsed_count(col_nr); k++) {
					uint8_t data_k = ft_collapsed_load_data(col, k);
					uint8_t *ts_suffix;
					unsigned int ts_slen;

					if (!ft_collapsed_entry_dead(data_k, col_nr))
						continue;
					ts_slen = ft_collapsed_suffix_len(col, data_k, k, col_nr);
					if (ts_slen != new_slen)
						continue;
					ts_suffix = ft_collapsed_suffix(col, data_k, col_nr);
					if (ft_key_cmp_ordinals(iter_key, ts_suffix,
							new_slen, new_slen,
							false, NULL) == 0) {
						tombstone_reuse = (int)k;
						break;
					}
				}

				if (tombstone_reuse >= 0)
					goto collapsed_tombstone_reuse;

				if (ft_collapsed_count(col_nr) > 0)
					cur_suffix_start = col->data[ft_collapsed_count(col_nr) - 1] & ft_collapsed_offset_mask(col_nr);
				else
					cur_suffix_start = ft_collapsed_scan_zone_size(col_nr);

				if (header_end + new_slen > cur_suffix_start ||
				    ft_collapsed_count(col_nr) >= ft_collapsed_max_entries(
					cds_ft_item_order(col),
					col->nr_entries >> FT_COLLAPSED_SCAN_SHIFT))
					goto collapsed_explode;
				goto collapsed_inplace_add;

			collapsed_tombstone_reuse:
				{
					/*
					 * Reuse tombstoned entry: clear the
					 * tombstone and write the new child.
					 * The suffix data is already correct.
					 */
					if (d.depth + new_slen == key_len) {
						branch = (struct cds_ft_inode_flag *) node;
					} else {
						branch = ft_build_branch(ft, key,
							d.depth + new_slen, key_len,
							(struct cds_ft_inode_flag *) node, 1);
						if (!branch) {
							ret = -ENOMEM;
							goto insert_done;
						}
					}
					/*
					 * Clear tombstone.  256B zones don't use
					 * tombstones (the bit overlaps offsets).
					 */
					if ((col->nr_entries >> FT_COLLAPSED_SCAN_SHIFT) < FT_COLLAPSED_SCAN_256)
						uatomic_store(&col->data[tombstone_reuse],
							col->data[tombstone_reuse] &
							~FT_COLLAPSED_TOMBSTONE,
							CMM_RELAXED);
					ft_set_parent(branch, d.nf);
					rcu_assign_pointer(
						cptrs[tombstone_reuse], branch);
					{
						struct cds_ft_metadata *col_meta =
							cds_ft_item_to_metadata(
								(struct cds_ft_inode *) col);
						col_meta->nr_child++;
					}
					ft_propagate_external_count_parent(d.nf, 1);
					ret = 0;
					goto insert_done;
				}

			collapsed_explode:
				{
					/*
					 * Collapsed node full or has prefix
					 * conflict: explode into internal +
					 * compressed nodes.  Uses recursive
					 * trie rebuild to handle entries that
					 * may share first suffix bytes.
					 */
					struct cds_ft_inode_flag *internal_flag;
					struct cds_ft_metadata *col_meta =
						cds_ft_item_to_metadata(
							(struct cds_ft_inode *) col);

					internal_flag = ft_explode_entries(ft,
						col, cptrs,
						0, ft_collapsed_count(ft_collapsed_nr_entries(col)), 0);
					if (!internal_flag) {
						ret = -ENOMEM;
						goto insert_done;
					}
					{
						struct cds_ft_metadata *int_meta =
							cds_ft_item_to_metadata(
								ft_node_ptr(internal_flag));
						int_meta->external_nodes =
							col_meta->external_nodes;
					}
					ft_init_node_density(internal_flag);

					ft_set_parent(internal_flag, d.pnf);
					rcu_assign_pointer(*d.nfp, internal_flag);
					free_collapsed_node(ft, col);

					d.nf = internal_flag;
					continue;
				}

			collapsed_inplace_add:
				/*
				 * Build child for the new entry.  The suffix
				 * covers key bytes from d.depth to key_len.
				 * If suffix IS the full remaining key, the
				 * child is the external node directly.
				 * Otherwise, wrap remaining bytes in a
				 * compressed path.
				 */
				if (d.depth + new_slen == key_len) {
					branch = (struct cds_ft_inode_flag *) node;
				} else {
					branch = ft_build_branch(ft, key,
						d.depth + new_slen, key_len,
						(struct cds_ft_inode_flag *) node, 1);
					if (!branch) {
						ret = -ENOMEM;
						goto insert_done;
					}
				}

				/* Write suffix (grows leftward from existing suffixes). */
				new_suffix_pos = ((uint8_t *) col) + cur_suffix_start - new_slen;
				for (k = 0; k < new_slen; k++)
					new_suffix_pos[k] = iter_key[k];

				/* Set offset for new entry. */
				col->data[ft_collapsed_count(col_nr)] = (uint8_t)(new_suffix_pos - (uint8_t *) col);

				/* Set pointer. */
				cptrs[ft_collapsed_count(col_nr)] = branch;
				ft_set_parent(branch, d.nf);

				/* Publish: increment nr_entries (atomic store). */
				/* Store-release in publish_inc ensures
				 * readers see suffix/offset/pointer data. */
				ft_collapsed_publish_inc_nr_entries(col);

				{
					struct cds_ft_metadata *col_meta =
						cds_ft_item_to_metadata(
							(struct cds_ft_inode *) col);
					col_meta->nr_child++;
				}

				/* Propagate. */
				ft_propagate_external_count_parent(d.nf, 1);
				ret = 0;
				goto insert_done;
			}
		}
		dbg_printf("cds_ft_insert iter ppnf %p pnf %p nfp %p nf %p\n",
				d.ppnf, d.pnf, d.nfp, d.nf);
		ft_snapshot_push(snapshot, snapshot_depth,
			nr_snapshot, d.nf, d.depth);
		key_value = *(iter_key++);
		ft_descent_step(&d, key_value);
	}

	if (d.depth == key_depth - 1) {
		/* Found either an internal, external node or NULL at end of key. */
		if (!ft_node_ptr(d.nf)) {
			dbg_printf("cds_ft_insert NULL ppnf %p pnf %p nfp %p nf %p\n",
					d.ppnf, d.pnf, d.nfp, d.nf);

			ret = ft_attach_node(ft, d.pnfp, d.pnf,
					d.nfp, d.nf, key, key_len, d.depth, node,
					NULL);
			if (ret == 0)
				ft_propagate_external_count_parent(*d.pnfp, 1);

		} else if (!ft_node_external(d.nf)) {
			struct cds_ft_node *external_nodes;
			struct cds_ft_metadata *metadata;

			metadata = cds_ft_item_to_metadata(ft_node_ptr(d.nf));
			external_nodes = metadata->external_nodes;
			if (external_nodes) {
				struct cds_ft_node *iter_node, *last_node = NULL;

				if (unique_node_ret) {
					*unique_node_ret = external_nodes;
					return -EEXIST;
				}
				/* Find last duplicate */
				iter_node = external_nodes;
				cds_ft_for_each_duplicate(iter_node)
					last_node = iter_node;

				dbg_printf("cds_ft_insert duplicate internal ppnf %p pnf %p nfp %p nf %p\n",
						d.ppnf, d.pnf, d.nfp, d.nf);

				/* Adding duplicate at existing key: no key count change. */
				ft_chain_node(last_node, node);
				ret = 0;
			} else {
				/* New key at this internal node. */
				node->next = NULL;
				rcu_assign_pointer(metadata->external_nodes, node);
				ret = 0;
				ft_propagate_external_count_parent(d.nf, 1);
			}
		} else {
			struct cds_ft_node *iter_node, *last_node = NULL;

			if (unique_node_ret) {
				*unique_node_ret = (struct cds_ft_node *) ft_node_ptr(d.nf);
				return -EEXIST;
			}
			/* Find last duplicate */
			iter_node = (struct cds_ft_node *) ft_node_ptr(d.nf);
			cds_ft_for_each_duplicate(iter_node)
				last_node = iter_node;

			dbg_printf("cds_ft_insert duplicate external ppnf %p pnf %p nfp %p nf %p\n",
					d.ppnf, d.pnf, d.nfp, d.nf);

			/* Adding duplicate at existing key: no key count change. */
			ft_chain_node(last_node, node);
			ret = 0;
		}
	} else {
		/* Found NULL node or external node before end of key. */

		/*
		 * If the last node encountered during traversal is an external node,
		 * transform this external node into an internal node with associated
		 * external node, attach a new cluster as child of this internal node, and
		 * populate this new internal node into the tree to replace the prior
		 * external node.
		 * It's the same for NULL node, only that there is no need to chain any
		 * external node.
		 */

		dbg_printf("cds_ft_insert NULL or external ppnf %p pnf %p nfp %p nf %p\n",
				d.ppnf, d.pnf, d.nfp, d.nf);

		ret = ft_attach_node(ft, d.pnfp, d.pnf,
				d.nfp, d.nf, key, key_len, d.depth, node,
				(struct cds_ft_node *) ft_node_ptr(d.nf));
		if (ret == 0)
			ft_propagate_external_count_parent(*d.pnfp, 1);
	}

insert_done:
	if (ret == 0) {
		if (key_len > uatomic_load(&ft->max_used_key_len, CMM_RELAXED))
			uatomic_store(&ft->max_used_key_len, key_len, CMM_RELAXED);
#ifdef FEATURE_FT_COLLAPSE
		if (key_len > 0)
			ft_check_collapse_on_path(ft, key, key_len);
#endif
	}

	return ret;
}

enum cds_ft_status cds_ft_insert(struct cds_ft *ft,
		const uint8_t *key, size_t key_len,
		struct cds_ft_node *node)
{
	int ret = _cds_ft_insert(ft, key, key_len, node, NULL);

	if (ret == 0)
		return CDS_FT_STATUS_OK;
	if (ret == -EINVAL)
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	return CDS_FT_STATUS_MEMORY_ERROR;
}

enum cds_ft_status cds_ft_insert_unique(struct cds_ft *ft,
		const uint8_t *key, size_t key_len,
		struct cds_ft_node *node,
		struct cds_ft_node **result_node)
{
	int ret;
	struct cds_ft_node *ret_node = NULL;

	ret = _cds_ft_insert(ft, key, key_len, node, &ret_node);
	if (ret == -EEXIST) {
		*result_node = ret_node;
		return CDS_FT_STATUS_DUPLICATE_FOUND;
	}
	if (ret == -EINVAL) {
		*result_node = NULL;
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}
	if (ret) {
		*result_node = NULL;
		return CDS_FT_STATUS_MEMORY_ERROR;
	}
	*result_node = node;
	return CDS_FT_STATUS_OK;
}

/*
 * Insert a node, replacing the entire existing duplicate chain at the
 * same key if one exists.
 *
 * On success, *@old_node_ret is set to the head of the replaced chain
 * (or NULL if no prior node existed). The caller must wait for a grace
 * period before reclaiming the old chain.
 *
 * Returns 0 on success, -EINVAL on bad arguments, or a negative errno
 * on memory allocation failure.
 */
static
int _cds_ft_insert_replace(struct cds_ft *ft,
		const uint8_t *_key, size_t _key_len,
		struct cds_ft_node *node,
		struct cds_ft_node **old_node_ret)
{
	unsigned int key_depth;
	struct ft_descent d;
	const uint8_t *iter_key;
	size_t key_len = ft_key_len(ft, _key_len);
	uint8_t ordinal_buf[FT_MAX_KEY_LEN];
	const uint8_t *key = ordinal_buf;
	struct cds_ft_inode_flag *snapshot[FT_MAX_DEPTH];
	unsigned int snapshot_depth[FT_MAX_DEPTH];
	int nr_snapshot = 0;
	int ret;

	*old_node_ret = NULL;

	if (!valid_external_node(node) || !valid_key_len(ft, key_len))
		return -EINVAL;
	ft_key_to_ordinals(ordinal_buf, _key, key_len, &ft->group->key_map);
	/* Expect zeroed next pointer. */
	if (node->next)
		return -EINVAL;

	key_depth = key_len + 1;

	dbg_printf("_cds_ft_insert_replace attempt: node %p\n", node);
	iter_key = key;
	ft_descent_init(&d, ft);

	for (; d.depth < key_depth - 1; ) {
		uint8_t key_value;

		if (!ft_node_ptr(d.nf))
			break;
		/* Resolve skip-compressed (e.g. from collapsed entry). */
		if (ft_node_skip_compressed(d.nf))
			d.nf = ft_compressed_node_flag(
				ft_skip_to_compressed(d.nf));
		if (ft_node_external(d.nf))
			break;
		if (ft_node_compressed(d.nf)) {
			enum ft_compressed_action act;

			act = ft_insert_compressed(ft, &d, &iter_key,
				key, key_len, key_depth, node,
				NULL, snapshot, snapshot_depth,
				&nr_snapshot, &ret);
			if (act == FT_COMPRESSED_END)
				goto insert_replace_done;
			if (act == FT_COMPRESSED_BREAK)
				break;
			continue;
		}
		if (ft_node_collapsed(d.nf)) {
			struct cds_ft_collapsed_node *col =
				ft_collapsed_node_ptr(d.nf);
			unsigned int remaining = key_depth - 1 - d.depth;
			unsigned int e;
			bool found_entry = false;

			unsigned int nr_e = ft_collapsed_nr_entries(col);
			struct cds_ft_inode_flag **cptrs =
				ft_collapsed_ptrs(col, nr_e);

			for (e = 0; e < ft_collapsed_count(nr_e); e++) {
				uint8_t data_e = ft_collapsed_load_data(col, e);
				unsigned int slen;
				uint8_t *suffix;
				bool match;

				if (ft_collapsed_entry_dead(data_e, nr_e))
					continue;
				slen = ft_collapsed_suffix_len(col, data_e, e, nr_e);
				if (slen > remaining)
					continue;
				suffix = ft_collapsed_suffix(col, data_e, nr_e);
				match = (ft_key_cmp_ordinals(iter_key, suffix, slen, slen, false, NULL) == 0);
				if (!match)
					continue;
				ft_snapshot_push(snapshot, snapshot_depth,
				nr_snapshot, d.nf, d.depth);
				d.ppnf  = d.pnf;
				d.ppnfp = d.pnfp;
				d.pnf   = d.nf;
				d.pnfp  = d.nfp;
				d.nf    = ft_dereference_acquire(cptrs[e]);
				d.nfp   = &cptrs[e];
				d.depth += slen;
				iter_key += slen;
				found_entry = true;
				break;
			}
			if (found_entry)
				continue;
			break;
		}
		dbg_printf("_cds_ft_insert_replace iter ppnf %p pnf %p nfp %p nf %p\n",
				d.ppnf, d.pnf, d.nfp, d.nf);
		ft_snapshot_push(snapshot, snapshot_depth,
			nr_snapshot, d.nf, d.depth);
		key_value = *(iter_key++);
		ft_descent_step(&d, key_value);
	}

	if (d.depth == key_depth - 1) {
		/* Found either an internal, external node or NULL at end of key. */
		if (!ft_node_ptr(d.nf)) {
			/* No existing node. Regular attach. */
			dbg_printf("_cds_ft_insert_replace NULL at end of key\n");

			ret = ft_attach_node(ft, d.pnfp, d.pnf,
					d.nfp, d.nf, key, key_len, d.depth, node,
					NULL);
			if (ret == 0)
				ft_propagate_external_count_parent(*d.pnfp, 1);
		} else if (!ft_node_external(d.nf)) {
			struct cds_ft_node *external_nodes;
			struct cds_ft_metadata *metadata;

			metadata = cds_ft_item_to_metadata(ft_node_ptr(d.nf));
			external_nodes = metadata->external_nodes;
			if (external_nodes) {
				dbg_printf("_cds_ft_insert_replace: replacing internal metadata chain %p\n",
						external_nodes);
				/* Replace existing chain: key count unchanged. */
				*old_node_ret = external_nodes;
				node->next = NULL;
				rcu_assign_pointer(metadata->external_nodes, node);
			} else {
				/* No external nodes yet. New key. */
				node->next = NULL;
				rcu_assign_pointer(metadata->external_nodes, node);
				ft_propagate_external_count_parent(d.nf, 1);
			}
			ret = 0;
		} else {
			dbg_printf("_cds_ft_insert_replace: replacing external chain %p\n",
					ft_node_ptr(d.nf));
			/* External node at end of key. Replace chain: key count unchanged. */
			*old_node_ret = (struct cds_ft_node *) ft_node_ptr(d.nf);
			node->next = NULL;
			rcu_assign_pointer(*d.nfp, (struct cds_ft_inode_flag *) node);
			ret = 0;
		}
	} else {
		/*
		 * Found NULL node or external node before end of key.
		 * Attach a new branch, displacing any shorter-key
		 * external node into the new branch's metadata.
		 */
		dbg_printf("_cds_ft_insert_replace: attach before end of key\n");

		ret = ft_attach_node(ft, d.pnfp, d.pnf,
				d.nfp, d.nf, key, key_len, d.depth, node,
				(struct cds_ft_node *) ft_node_ptr(d.nf));
		if (ret == 0)
			ft_propagate_external_count_parent(*d.pnfp, 1);
	}

insert_replace_done:
	if (ret == 0) {
		if (key_len > uatomic_load(&ft->max_used_key_len, CMM_RELAXED))
			uatomic_store(&ft->max_used_key_len, key_len, CMM_RELAXED);
	}

	return ret;
}

enum cds_ft_status cds_ft_insert_replace(struct cds_ft *ft,
		const uint8_t *key, size_t key_len,
		struct cds_ft_node *node,
		struct cds_ft_node **result_node)
{
	struct cds_ft_node *old_node = NULL;
	int ret;

	ret = _cds_ft_insert_replace(ft, key, key_len, node, &old_node);
	if (ret == -EINVAL) {
		*result_node = NULL;
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}
	if (ret) {
		*result_node = NULL;
		return CDS_FT_STATUS_MEMORY_ERROR;
	}
	*result_node = old_node;
	if (old_node) {
		return CDS_FT_STATUS_DUPLICATE_FOUND;
	}
	return CDS_FT_STATUS_OK;
}

enum cds_ft_status cds_ft_replace(struct cds_ft *ft,
		struct cds_ft_iter *iter,
		struct cds_ft_node *old_node,
		struct cds_ft_node *new_node)
{
	unsigned int i, key_depth;
	struct cds_ft_inode_flag *node_flag;
	struct cds_ft_inode_flag **node_flag_ptr;
	struct cds_ft_node *iter_node, **iter_node_ptr, **prev_node_ptr, *match;
	const uint8_t *iter_key;
	size_t key_len = ft_key_len(ft, iter->key_len);

	/*
	 * If the iterator has a valid path, the RCU read-side lock must
	 * be held.
	 */
	if (iter->path_valid)
		CDS_FT_ASSERT_RCU_READ_LOCKED(ft);
	iter_debug_path_check(iter);

	if (!valid_external_node(old_node) || !valid_external_node(new_node)
			|| !valid_key_len(ft, key_len))
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	/* Expect zeroed next pointer on new_node. */
	if (new_node->next)
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;

	key_depth = key_len + 1;
	iter_key = iter_key(iter);

	dbg_printf("cds_ft_replace: old_node %p new_node %p\n", old_node, new_node);

	node_flag = ft->root;
	node_flag_ptr = &ft->root;

	/* Root is always present and always internal. */

	/*
	 * Handle NIL key (key_len == 0).
	 */
	if (!key_len) {
		struct cds_ft_metadata *metadata;

		metadata = cds_ft_item_to_metadata(ft_node_ptr(node_flag));
		if (!metadata->external_nodes)
			return CDS_FT_STATUS_NOT_FOUND;
		iter_node_ptr = (struct cds_ft_node **) &metadata->external_nodes;
		iter_node = metadata->external_nodes;
		goto find_and_replace;
	}

	/* Traverse internal levels. */
	for (i = 1; i < key_depth; i++) {
		uint8_t key_value;

		if (ft_node_external(node_flag))
			return CDS_FT_STATUS_NOT_FOUND;
		if (ft_node_compressed(node_flag)) {
			bool nf = false;
			enum ft_compressed_action act;

			act = ft_traverse_compressed(&node_flag,
				&node_flag_ptr, &iter_key, &i,
				key_depth, &nf);
			if (nf)
				return CDS_FT_STATUS_NOT_FOUND;
			if (act == FT_COMPRESSED_BREAK)
				break;
			continue;
		}
		if (ft_node_collapsed(node_flag)) {
			bool nf = false;
			enum ft_compressed_action act;

			act = ft_traverse_collapsed(&node_flag,
				&node_flag_ptr, &iter_key, &i,
				key_depth, &nf);
			if (nf)
				return CDS_FT_STATUS_NOT_FOUND;
			if (act == FT_COMPRESSED_BREAK)
				break;
			continue;
		}
		key_value = *(iter_key++);
		node_flag = ft_node_get_nth(node_flag, &node_flag_ptr, key_value);
		if (!ft_node_ptr(node_flag))
			return CDS_FT_STATUS_NOT_FOUND;
	}

	/* Reached end of key. Locate the duplicate chain. */
	if (!ft_node_external(node_flag)) {
		struct cds_ft_metadata *metadata;

		metadata = cds_ft_item_to_metadata(ft_node_ptr(node_flag));
		if (!metadata->external_nodes)
			return CDS_FT_STATUS_NOT_FOUND;
		iter_node_ptr = (struct cds_ft_node **) &metadata->external_nodes;
		iter_node = metadata->external_nodes;
	} else {
		iter_node_ptr = (struct cds_ft_node **) node_flag_ptr;
		iter_node = (struct cds_ft_node *) ft_node_ptr(node_flag);
	}

find_and_replace:
	/*
	 * Walk the duplicate chain to find old_node and track the
	 * pointer that references it (prev_node_ptr).
	 */
	prev_node_ptr = NULL;
	match = NULL;
	cds_ft_for_each_duplicate(iter_node) {
		if (match)
			continue;
		if (iter_node == old_node) {
			prev_node_ptr = iter_node_ptr;
			match = iter_node;
		}
		iter_node_ptr = &iter_node->next;
	}

	if (!match)
		return CDS_FT_STATUS_NOT_FOUND;

	/*
	 * Splice new_node into the chain in place of old_node.
	 * new_node inherits old_node's successor. The write barrier
	 * within rcu_assign_pointer ensures new_node->next is visible
	 * before the pointer that publishes new_node.
	 */
	new_node->next = old_node->next;
	rcu_assign_pointer(*prev_node_ptr, new_node);

	/*
	 * The trie structure is unchanged (no recompaction), so the
	 * iterator path remains valid in cached mode.
	 */
	iter_auto_invalidate_path(iter);
	return CDS_FT_STATUS_OK;
}

/*
 * ft_detach_node: detach a node from the trie and prune empty
 * single-child ancestors above it.
 *
 * Walks upward from the parent of the detached node via
 * metadata->parent pointers.  Prunes single-child ancestors until
 * reaching a node with multiple children, external nodes, or the
 * root.  The pruned branch is replaced by the topmost external
 * nodes found during the walk (or NULL).
 *
 * @detach_node_flag_ptr: slot in parent pointing to the detached node.
 * @detach_parent_flag_ptr: slot in grandparent pointing to the parent.
 * @detach_depth: trie depth of the detached node.
 */
static
int ft_detach_node(struct cds_ft *ft,
		struct cds_ft_inode_flag **detach_node_flag_ptr,
		struct cds_ft_inode_flag **detach_parent_flag_ptr,
		unsigned int detach_depth)
{
	struct cds_ft_metadata *metadata_stack[FT_MAX_DEPTH];
	struct cds_ft_inode_flag *iter_node_flag;
	struct cds_ft_inode *old_recompacted_node = NULL;
	int ret, nr_metadata = 0, nr_clear = 0, nr_branch = 0;
	uint8_t n = 0;
	struct cds_ft_node *topmost_external_nodes = NULL;
	bool prev_external_nodes_found = false;
	struct cds_ft_inode_flag *cur;
	unsigned int cur_depth;

	/*
	 * Walk upward from the parent of the detached node via
	 * metadata->parent.  At each ancestor, check if it has only
	 * one child left.  If so, mark it for pruning and continue.
	 * Stop when reaching a multi-child node, a node with
	 * external_nodes, or the root (parent == NULL).
	 */
	cur = *detach_parent_flag_ptr;
	cur_depth = detach_depth - ft_parent_depth_span(cur, *detach_node_flag_ptr);

	while (cur) {
		struct cds_ft_metadata *metadata;
		bool is_root;

		metadata = cds_ft_item_to_metadata(ft_node_ptr(cur));
		metadata_stack[nr_metadata++] = metadata;
		is_root = (metadata->parent == NULL);

		assert(metadata->nr_child > 0);
		if (!prev_external_nodes_found && (metadata->nr_child == 1 && !is_root)) {
			nr_clear++;
			topmost_external_nodes = metadata->external_nodes;
		}
		nr_branch++;
		if (prev_external_nodes_found || metadata->nr_child > 1 || is_root) {
			if (!is_root) {
				struct cds_ft_metadata *parent_meta =
					cds_ft_item_to_metadata(
						ft_node_ptr(metadata->parent));
				metadata_stack[nr_metadata++] = parent_meta;
			}
			/*
			 * Find the key byte for replace_ptr.  Only needed
			 * for internal parents (compressed/collapsed are
			 * handled separately below).
			 */
			if (!ft_node_compressed(cur) && !ft_node_collapsed(cur))
				ft_node_find_child(cur, *detach_node_flag_ptr,
					&n, NULL);
			break;
		}
		if (topmost_external_nodes)
			prev_external_nodes_found = true;

		/*
		 * Walk up: the current node becomes the child,
		 * update detach pointers to prune at this level.
		 */
		{
			struct cds_ft_inode_flag *parent_nf = metadata->parent;

			if (!parent_nf)
				break;
			/*
			 * Find the slot in the grandparent pointing to
			 * cur, which becomes the new detach_parent_flag_ptr.
			 * Find the slot in cur pointing to its child (the
			 * previous level), which becomes detach_node_flag_ptr.
			 */
			detach_node_flag_ptr = detach_parent_flag_ptr;
			if (is_root)
				detach_parent_flag_ptr = &ft->root;
			else {
				ft_node_find_child(parent_nf, cur, NULL,
					&detach_parent_flag_ptr);
			}
			cur_depth -= ft_parent_depth_span(parent_nf, cur);
			cur = parent_nf;
		}
	}

	iter_node_flag = *detach_parent_flag_ptr;
	/*
	 * Replace within parent.  If the parent is a compressed node
	 * (e.g. compressed root from detach/graft_swap), recompact it
	 * to an empty linear node so the root is always internal.
	 */
	if (ft_node_compressed(iter_node_flag) ||
	    ft_node_skip_compressed(iter_node_flag)) {
		struct cds_ft_inode *fresh;
		struct cds_ft_metadata *fresh_meta;

		fresh = alloc_cds_ft_node(ft, &ft_types[0], &fresh_meta);
		if (!fresh) {
			ret = -ENOMEM;
			goto end;
		}
		fresh_meta->parent = cds_ft_item_to_metadata(
			(struct cds_ft_inode *) ft_compressed_node_ptr(
				iter_node_flag))->parent;
		if (topmost_external_nodes) {
			fresh_meta->external_nodes = topmost_external_nodes;
			uatomic_store(&fresh_meta->nr_keys, 1, CMM_RELAXED);
		}
		rcu_assign_pointer(*detach_parent_flag_ptr,
			ft_node_flag(fresh, 0));
		free_compressed_node(ft,
			ft_compressed_node_ptr(iter_node_flag));
		ret = 0;
	} else if (ft_node_collapsed(iter_node_flag)) {
		/*
		 * Collapsed parent: tombstone the entry pointing to
		 * the detached child, and set the child pointer to
		 * the topmost_external_nodes (or NULL).
		 */
		struct cds_ft_collapsed_node *col =
			ft_collapsed_node_ptr(iter_node_flag);
		struct cds_ft_metadata *col_meta =
			cds_ft_item_to_metadata((struct cds_ft_inode *) col);
		unsigned int e;

		unsigned int nr_e = ft_collapsed_nr_entries(col);
		struct cds_ft_inode_flag **cptrs = ft_collapsed_ptrs(col, nr_e);

		for (e = 0; e < ft_collapsed_count(nr_e); e++) {
			if (&cptrs[e] == detach_node_flag_ptr) {
				if (topmost_external_nodes) {
					rcu_assign_pointer(cptrs[e],
						(struct cds_ft_inode_flag *)
						topmost_external_nodes);
				} else {
					rcu_assign_pointer(cptrs[e], NULL);
					/*
					 * Set tombstone.  256B zones don't use
					 * tombstones (the bit overlaps offsets).
					 * For 256B, readers rely on the NULL
					 * pointer check instead.
					 */
					if ((col->nr_entries >> FT_COLLAPSED_SCAN_SHIFT) < FT_COLLAPSED_SCAN_256)
						uatomic_store(&col->data[e],
							col->data[e] | FT_COLLAPSED_TOMBSTONE,
							CMM_RELAXED);
					col_meta->nr_child--;
				}
				break;
			}
		}
		/*
		 * If all entries are dead, replace the collapsed node
		 * with the topmost_external_nodes (or NULL) in the
		 * grandparent.
		 */
		if (col_meta->nr_child == 0) {
			struct cds_ft_inode_flag *replacement =
				topmost_external_nodes ?
				(struct cds_ft_inode_flag *) topmost_external_nodes : NULL;

			/* Propagate density -1 for the freed collapsed node. */
			ft_propagate_node_density_parent(
				iter_node_flag, cur_depth,
				cur_depth, -1);
			rcu_assign_pointer(*detach_parent_flag_ptr, replacement);
			free_collapsed_node(ft, col);
		}
		ret = 0;
	} else {
		ret = ft_node_replace_ptr(ft,
			detach_node_flag_ptr,
			&iter_node_flag,
			&old_recompacted_node,
			metadata_stack[nr_branch - 1],
			n, (struct cds_ft_inode_flag *) topmost_external_nodes,
			detach_parent_flag_ptr == &ft->root);
		/*
		 * Density is updated incrementally by
		 * ft_propagate_node_density; no full recount needed.
		 */
	}
	if (ret)
		goto end;

	/*
	 * Update address of parent ptr in its parent.
	 * Skip for compressed/collapsed parents: the replacement was
	 * already published inline above.
	 */
	if (!ft_node_compressed(iter_node_flag) &&
	    !ft_node_skip_compressed(iter_node_flag) &&
	    !ft_node_collapsed(iter_node_flag)) {
		dbg_printf("ft_detach_node: publish %p instead of %p\n",
			iter_node_flag, *detach_parent_flag_ptr);
		rcu_assign_pointer(*detach_parent_flag_ptr, iter_node_flag);
	}
end:
	/* Reclaim safely after replacement. */
	if (old_recompacted_node)
		free_cds_ft_node(ft, old_recompacted_node);

	if (!ret) {
		/*
		 * Propagate density: -nr_clear at the surviving parent's
		 * depth (where the pruned nodes were children).
		 */
		if (nr_clear > 0)
			ft_propagate_node_density_parent(cur, cur_depth,
				cur_depth, (long) -nr_clear);
		{
			int ci;

			for (ci = 0; ci < nr_clear; ci++)
				free_cds_ft_node(ft,
					cds_ft_metadata_to_item(metadata_stack[ci]));
		}
		/*
		 * Density on the surviving parent is updated
		 * incrementally by ft_propagate_node_density above;
		 * no full recount needed.
		 */
	}
	return ret;
}

static
void ft_unchain_node(struct cds_ft_node **prev_node_ptr,
		struct cds_ft_node *node)
{
	uatomic_store(prev_node_ptr, node->next, CMM_RELAXED);
}

/*
 * Called with RCU read lock held.
 *
 * There are a few cases to cover for delete:
 *
 * 1) The node belongs to a list of external nodes duplicates with two
 *    or more items. Remove the node by unlinking it from its list.
 * 2) There is only one external node within this node's list.
 *    2.1) The node is within an external nodes list for which the list
 *         head is an standalone external nodes pointer. The external
 *         nodes list for this key should be removed. Removing an
 *         external nodes list should prune the entire branch leading to
 *         that list so no lookup observe empty internal nodes. This is
 *         done by ft_detach_node(). Internal nodes are considered empty
 *         if they have no internal and no external node children, *and*
 *         their associated list of external nodes is empty. When
 *         detaching an internal node which has no children, but has
 *         an associated list of external nodes, it is replaced by a
 *         pointer to the external nodes.
 *    2.2) The node is within an external nodes list which is associated
 *         with an internal node. Unlink the node from its list, leaving
 *         the external nodes list empty.
 */
enum cds_ft_status cds_ft_remove(struct cds_ft *ft,
		struct cds_ft_iter *iter,
		struct cds_ft_node *node)
{
	struct ft_detach_descent dd;
	struct cds_ft_node *iter_node, **iter_node_ptr, **prev_node_ptr, *match;
	int ret, count = 0;
	const uint8_t *iter_key;
	size_t key_len = ft_key_len(ft, iter->key_len);

	/*
	 * If the iterator has a valid path, the RCU read-side lock must
	 * be held.
	 */
	if (iter->path_valid)
		CDS_FT_ASSERT_RCU_READ_LOCKED(ft);
	iter_debug_path_check(iter);

	if (!valid_external_node(node) || !valid_key_len(ft, key_len))
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;

	iter_key = iter_key(iter);
	dbg_printf("cds_ft_remove attempt: node %p\n", node);

	ft_detach_descent_init(&dd, ft);

	/* Iterate on all internal levels */
	for (; dd.d.depth < key_len; ) {
		uint8_t key_value;
		const struct cds_ft_metadata *metadata;

		dbg_printf("cds_ft_remove iter nf %p\n",
				dd.d.nf);
		if (!ft_node_ptr(dd.d.nf)) {
			return CDS_FT_STATUS_NOT_FOUND;
		}
		/* Resolve skip-compressed (e.g. from collapsed entry). */
		if (ft_node_skip_compressed(dd.d.nf))
			dd.d.nf = ft_compressed_node_flag(
				ft_skip_to_compressed(dd.d.nf));

		/*
		 * Compressed node: compare remaining key bytes with
		 * the compressed path.  If they match, traverse
		 * through to the child.  If they diverge, the key
		 * is not present.
		 */
		if (ft_node_compressed(dd.d.nf)) {
			struct cds_ft_compressed_node *cn =
				ft_compressed_node_ptr(dd.d.nf);
			const struct cds_ft_metadata *cn_meta =
				cds_ft_item_to_metadata(
					(struct cds_ft_inode *) cn);
			unsigned int remaining = key_len - dd.d.depth;
			unsigned int cmp = cn->len < remaining ?
				cn->len : remaining;
			unsigned int j;

			j = ft_match_compressed_key(iter_key, cn, cmp);
			if (j < cmp || cn->len > remaining ||
			    !ft_node_ptr(cn->child))
				return CDS_FT_STATUS_NOT_FOUND;
			/*
			 * Full match with non-NULL child: traverse
			 * through without decompressing.
			 */
			ft_detach_descent_track(&dd, cn_meta);
			ft_descent_traverse_compressed(&dd.d, cn, &iter_key);
			if (ft_node_ptr(dd.d.nf) && dd.pending) {
				dd.det_nfp = dd.d.nfp;
				dd.pending = false;
			}
			continue;
		}
		if (ft_node_collapsed(dd.d.nf)) {
			struct cds_ft_collapsed_node *col =
				ft_collapsed_node_ptr(dd.d.nf);
			const struct cds_ft_metadata *col_meta =
				cds_ft_item_to_metadata(
					(struct cds_ft_inode *) col);
			unsigned int remaining = key_len - dd.d.depth;
			unsigned int e;
			bool found = false;

			unsigned int nr_e = ft_collapsed_nr_entries(col);
			struct cds_ft_inode_flag **cptrs =
				ft_collapsed_ptrs(col, nr_e);

			for (e = 0; e < ft_collapsed_count(nr_e); e++) {
				uint8_t data_e = ft_collapsed_load_data(col, e);
				unsigned int slen;
				uint8_t *suffix;
				bool match2;

				if (ft_collapsed_entry_dead(data_e, nr_e))
					continue;
				slen = ft_collapsed_suffix_len(col, data_e, e, nr_e);
				if (slen > remaining)
					continue;
				suffix = ft_collapsed_suffix(col, data_e, nr_e);
				match2 = (ft_key_cmp_ordinals(iter_key, suffix, slen, slen, false, NULL) == 0);
				if (!match2)
					continue;
				if (!ft_node_ptr(cptrs[e]))
					return CDS_FT_STATUS_NOT_FOUND;
				ft_detach_descent_track(&dd, col_meta);
				dd.d.ppnf  = dd.d.pnf;
				dd.d.ppnfp = dd.d.pnfp;
				dd.d.pnf   = dd.d.nf;
				dd.d.pnfp  = dd.d.nfp;
				dd.d.nf    = ft_dereference_acquire(cptrs[e]);
				dd.d.nfp   = &cptrs[e];
				dd.d.depth += slen;
				iter_key += slen;
				if (ft_node_ptr(dd.d.nf) && dd.pending) {
					dd.det_nfp = dd.d.nfp;
					dd.pending = false;
				}
				found = true;
				break;
			}
			if (!found)
				return CDS_FT_STATUS_NOT_FOUND;
			continue;
		}

		/*
		 * Track pointers for the detach point during descent.
		 * Update when encountering a potential upward-walk
		 * termination point.
		 */
		metadata = cds_ft_item_to_metadata(ft_node_ptr(dd.d.nf));
		ft_detach_descent_track(&dd, metadata);

		key_value = *(iter_key++);
		ft_detach_descent_step(&dd, key_value);
		dbg_printf("cds_ft_remove iter key lookup %u finds nf %p, nfp %p\n",
				(unsigned int) key_value, dd.d.nf,
				dd.d.nfp);
	}
	/*
	 * We reached end of key, try to find the node we are trying to
	 * remove. Fail if we cannot find it.
	 */
	if (!ft_node_ptr(dd.d.nf)) {
		dbg_printf("cds_ft_remove: no node found for key\n");
		return CDS_FT_STATUS_NOT_FOUND;
	}

	if (!ft_node_external(dd.d.nf)) {
		/* Found internal or compressed node at end of key. */
		struct cds_ft_node *external_nodes;
		struct cds_ft_metadata *metadata;

		metadata = cds_ft_item_to_metadata(ft_node_ptr(dd.d.nf));
		external_nodes = metadata->external_nodes;
		if (external_nodes) {
			/*
			 * Find the previous node's next pointer pointing to our node,
			 * so we can update it.
			 */
			prev_node_ptr = NULL;
			iter_node_ptr = (struct cds_ft_node **) &metadata->external_nodes;
			iter_node = (struct cds_ft_node *) external_nodes;
			match = NULL;
			cds_ft_for_each_duplicate(iter_node) {
				if (match)
					continue;
				dbg_printf("cds_ft_remove: compare %p with iter_node %p\n", node, iter_node);
				if (iter_node == node) {
					prev_node_ptr = iter_node_ptr;
					match = iter_node;
				}
				iter_node_ptr = &iter_node->next;
			}
			if (!match) {
				dbg_printf("cds_ft_remove: no node match for node %p key\n", node);
				return CDS_FT_STATUS_NOT_FOUND;
			}
			/*
			 * Propagate -1 before unchain if this is the last
			 * entry in the chain (undercount ordering: decrement
			 * nr_keys before detaching the pointer).
			 */
			if (prev_node_ptr == (struct cds_ft_node **) &metadata->external_nodes
			    && !match->next) {
				ft_propagate_external_count_parent(dd.d.nf, -1);
			}
			ft_unchain_node(prev_node_ptr, match);
			ret = 0;
		} else {
			dbg_printf("cds_ft_remove: no metadata external node found for key\n");
			return CDS_FT_STATUS_NOT_FOUND;
		}
	} else {
		/* Found external node at end of key. */

		/*
		 * Find the previous node's next pointer pointing to our node,
		 * so we can update it.
		 */
		prev_node_ptr = NULL;
		iter_node_ptr = (struct cds_ft_node **) dd.d.nfp;
		iter_node = (struct cds_ft_node *) ft_node_ptr(dd.d.nf);
		count = 0;
		match = NULL;
		cds_ft_for_each_duplicate(iter_node) {
			count++;
			if (match)
				continue;
			dbg_printf("cds_ft_remove: compare %p with iter_node %p\n", node, iter_node);
			if (iter_node == node) {
				prev_node_ptr = iter_node_ptr;
				match = iter_node;
			}
			iter_node_ptr = &iter_node->next;
		}
		if (!match) {
			dbg_printf("cds_ft_remove: no node match for node %p key\n", node);
			return CDS_FT_STATUS_NOT_FOUND;
		}
		assert(count > 0);
		if (count == 1) {
			/*
			 * Removing last of duplicates. Last snapshot
			 * does not have metadata (external leafs).
			 *
			 * Propagate -1 before detach, which may free
			 * internal nodes in the snapshot.
			 */
			ft_propagate_external_count_parent(dd.d.pnf, -1);
			ret = ft_detach_node(ft,
					dd.det_nfp,
					dd.det_pfp,
					dd.d.depth);
			if (ret) {
				/* Undo propagation on failure. */
				ft_propagate_external_count_parent(dd.d.pnf, 1);
			}
		} else {
			/* Removing duplicate, not last: key count unchanged. */
			ft_unchain_node(prev_node_ptr, match);
			ret = 0;
		}
	}

	/*
	 * detach should not replace a NULL pointer because it has been
	 * found by a mutex-protected traversal within this function.
	 */
	assert(ret != -ENOENT);

	/*
	 * Invalidate the iterator path. The trie structure may have
	 * changed due to node recompaction during detach, making the
	 * cached path stale.
	 */
	iter->path_valid = false;
	iter_debug_path_clear(iter);
	iter->path_len = 0;

	switch (ret) {
	case 0:
#ifdef FEATURE_FT_COLLAPSE
		ft_check_collapse_on_path(ft, iter_key, key_len);
#endif
		return CDS_FT_STATUS_OK;
	case -ENOMEM:
		return CDS_FT_STATUS_MEMORY_ERROR;
	default:
		abort();
	}
}

enum cds_ft_status cds_ft_remove_all(struct cds_ft *ft,
		struct cds_ft_iter *iter,
		struct cds_ft_node **result_node)
{
	struct ft_detach_descent dd;
	int ret;
	const uint8_t *iter_key;
	size_t key_len = ft_key_len(ft, iter->key_len);

	/*
	 * If the iterator has a valid path, the RCU read-side lock must
	 * be held.
	 */
	if (iter->path_valid)
		CDS_FT_ASSERT_RCU_READ_LOCKED(ft);
	iter_debug_path_check(iter);

	if (!valid_key_len(ft, key_len)) {
		*result_node = NULL;
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}

	/*
	 * Handle NIL key (key_len == 0): root is always internal,
	 * remove its external_nodes chain.
	 */
	if (!key_len) {
		struct cds_ft_metadata *metadata;
		struct cds_ft_node *external_nodes;

		metadata = ft_root_metadata(ft);
		external_nodes = metadata->external_nodes;
		if (!external_nodes) {
			*result_node = NULL;
			return CDS_FT_STATUS_NOT_FOUND;
		}
		*result_node = external_nodes;
		/* Decrement before detach (undercount ordering). */
		uatomic_store(&metadata->nr_keys, metadata->nr_keys - 1,
			CMM_RELEASE);
		rcu_assign_pointer(metadata->external_nodes, NULL);
		return CDS_FT_STATUS_OK;
	}

	iter_key = iter_key(iter);
	dbg_printf("cds_ft_remove_all attempt\n");

	ft_detach_descent_init(&dd, ft);

	/* Iterate on all internal levels. */
	for (; dd.d.depth < key_len; ) {
		uint8_t key_value;
		const struct cds_ft_metadata *metadata;

		dbg_printf("cds_ft_remove_all iter nf %p\n", dd.d.nf);
		if (!ft_node_ptr(dd.d.nf)) {
			*result_node = NULL;
			return CDS_FT_STATUS_NOT_FOUND;
		}

		/*
		 * Compressed node: compare remaining key bytes with
		 * the compressed path.  If they match, traverse
		 * through to the child.  If they diverge, the key
		 * is not present.
		 */
		if (ft_node_compressed(dd.d.nf)) {
			struct cds_ft_compressed_node *cn =
				ft_compressed_node_ptr(dd.d.nf);
			const struct cds_ft_metadata *cn_meta =
				cds_ft_item_to_metadata(
					(struct cds_ft_inode *) cn);
			unsigned int remaining = key_len - dd.d.depth;
			unsigned int cmp = cn->len < remaining ?
				cn->len : remaining;
			unsigned int j;

			j = ft_match_compressed_key(iter_key, cn, cmp);
			if (j < cmp || cn->len > remaining ||
			    !ft_node_ptr(cn->child)) {
				*result_node = NULL;
				return CDS_FT_STATUS_NOT_FOUND;
			}
			ft_detach_descent_track(&dd, cn_meta);
			ft_descent_traverse_compressed(&dd.d, cn, &iter_key);
			if (ft_node_ptr(dd.d.nf) && dd.pending) {
				dd.det_nfp = dd.d.nfp;
				dd.pending = false;
			}
			continue;
		}
		if (ft_node_collapsed(dd.d.nf)) {
			struct cds_ft_collapsed_node *col =
				ft_collapsed_node_ptr(dd.d.nf);
			const struct cds_ft_metadata *col_meta =
				cds_ft_item_to_metadata(
					(struct cds_ft_inode *) col);
			unsigned int remaining = key_len - dd.d.depth;
			unsigned int e;
			bool found = false;

			unsigned int nr_e = ft_collapsed_nr_entries(col);
			struct cds_ft_inode_flag **cptrs =
				ft_collapsed_ptrs(col, nr_e);

			for (e = 0; e < ft_collapsed_count(nr_e); e++) {
				uint8_t data_e = ft_collapsed_load_data(col, e);
				unsigned int slen;
				uint8_t *suffix;
				bool match;

				if (ft_collapsed_entry_dead(data_e, nr_e))
					continue;
				slen = ft_collapsed_suffix_len(col, data_e, e, nr_e);
				if (slen > remaining)
					continue;
				suffix = ft_collapsed_suffix(col, data_e, nr_e);
				match = (ft_key_cmp_ordinals(iter_key, suffix, slen, slen, false, NULL) == 0);
				if (!match)
					continue;
				if (!ft_node_ptr(cptrs[e])) {
					*result_node = NULL;
					return CDS_FT_STATUS_NOT_FOUND;
				}
				ft_detach_descent_track(&dd, col_meta);
				dd.d.ppnf  = dd.d.pnf;
				dd.d.ppnfp = dd.d.pnfp;
				dd.d.pnf   = dd.d.nf;
				dd.d.pnfp  = dd.d.nfp;
				dd.d.nf    = ft_dereference_acquire(cptrs[e]);
				dd.d.nfp   = &cptrs[e];
				dd.d.depth += slen;
				iter_key += slen;
				if (ft_node_ptr(dd.d.nf) && dd.pending) {
					dd.det_nfp = dd.d.nfp;
					dd.pending = false;
				}
				found = true;
				break;
			}
			if (!found) {
				*result_node = NULL;
				return CDS_FT_STATUS_NOT_FOUND;
			}
			continue;
		}

		/* Track detach point pointers during descent. */
		metadata = cds_ft_item_to_metadata(ft_node_ptr(dd.d.nf));
		ft_detach_descent_track(&dd, metadata);

		key_value = *(iter_key++);
		ft_detach_descent_step(&dd, key_value);
		dbg_printf("cds_ft_remove_all iter key lookup %u finds nf %p, nfp %p\n",
				(unsigned int) key_value, dd.d.nf, dd.d.nfp);
	}

	/* Reached end of key. */
	if (!ft_node_ptr(dd.d.nf)) {
		dbg_printf("cds_ft_remove_all: no node found for key\n");
		*result_node = NULL;
		return CDS_FT_STATUS_NOT_FOUND;
	}

	if (!ft_node_external(dd.d.nf)) {
		/* Internal or compressed node at end of key. Remove all external nodes from metadata. */
		struct cds_ft_node *external_nodes;
		struct cds_ft_metadata *metadata;

		metadata = cds_ft_item_to_metadata(ft_node_ptr(dd.d.nf));
		external_nodes = metadata->external_nodes;
		if (!external_nodes) {
			dbg_printf("cds_ft_remove_all: no metadata external nodes for key\n");
			*result_node = NULL;
			return CDS_FT_STATUS_NOT_FOUND;
		}
		/*
		 * Atomically remove the entire chain. The internal
		 * node itself remains (it still has children). A grace
		 * period must be observed before reclaiming any node
		 * in the old chain.
		 */
		/*
		 * Removing one key (with all its duplicates).
		 * Decrement before detach (undercount ordering).
		 */
		*result_node = external_nodes;
		ft_propagate_external_count_parent(dd.d.nf, -1);
		rcu_assign_pointer(metadata->external_nodes, NULL);
		ret = 0;
	} else {
		/*
		 * External node at end of key. Detach the branch.
		 * Removing one key (with all its duplicates).
		 */
		*result_node = (struct cds_ft_node *) ft_node_ptr(dd.d.nf);
		/* Propagate before detach to avoid writing freed metadata. */
		ft_propagate_external_count_parent(dd.d.pnf, -1);
		ret = ft_detach_node(ft,
				dd.det_nfp,
				dd.det_pfp,
				dd.d.depth);
		if (ret) {
			/* Undo propagation on failure. */
			ft_propagate_external_count_parent(dd.d.pnf, 1);
		}
	}

	/*
	 * detach should not replace a NULL pointer because it has been
	 * found by a mutex-protected traversal within this function.
	 */
	assert(ret != -ENOENT);

	iter->path_valid = false;
	iter_debug_path_clear(iter);
	iter->path_len = 0;

	if (ret)
		return CDS_FT_STATUS_NOT_FOUND;

#ifdef FEATURE_FT_COLLAPSE
	ft_check_collapse_on_path(ft, iter_key, key_len);
#endif
	return CDS_FT_STATUS_OK;
}

/*
 * Split a compressed node during graft when the key diverges at
 * position @diverge_pos within the compressed path.
 *
 * Builds: [prefix] -> [branch] -> old_suffix -> old_child
 *
 * Unlike ft_split_compressed_insert, this does NOT create the graft
 * side.  Instead, it sets up the descent state so that
 * ft_store_at_graft_point can attach the graft payload to the
 * branch's empty slot for the key ordinal at the divergence point.
 *
 * On success, updates @d to point to the empty slot in the branch
 * node, adds new nodes to @snapshot, and returns 0.
 */
static
int ft_split_compressed_graft(struct cds_ft *ft,
		struct ft_descent *d,
		const uint8_t *iter_key,
		unsigned int diverge_pos)
{
	struct cds_ft_compressed_node *cn = ft_compressed_node_ptr(d->nf);
	struct cds_ft_metadata *cn_meta =
		cds_ft_item_to_metadata((struct cds_ft_inode *) cn);
	struct cds_ft_inode_flag *old_suffix_flag;
	struct cds_ft_inode_flag *branch_flag, *top_flag;
	struct cds_ft_inode_flag *created[FT_MAX_DEPTH];
	int nr_created = 0;
	unsigned int suffix_len = cn->len - diverge_pos - 1;
	uint8_t old_ordinal = cn->key_bytes[diverge_pos];
	unsigned long old_child_nr_keys;
	int ret;

	/* Compute old child's nr_keys. */
	if (!ft_node_external(cn->child)) {
		struct cds_ft_metadata *cm =
			cds_ft_item_to_metadata(ft_node_ptr(cn->child));
		old_child_nr_keys = cm->nr_keys;
	} else if (ft_node_ptr(cn->child)) {
		old_child_nr_keys = 1;
	} else {
		old_child_nr_keys = 0;
	}

	/* 1. Build old suffix -> old child. */
	if (suffix_len >= 2) {
		struct cds_ft_compressed_node *sfx;
		struct cds_ft_metadata *sfx_meta;

		sfx = alloc_compressed_node(ft, suffix_len, &sfx_meta);
		if (!sfx) goto error;
		sfx->child = cn->child;
		sfx->len = suffix_len;
		memcpy(sfx->key_bytes, &cn->key_bytes[diverge_pos + 1],
			suffix_len);
		sfx_meta->nr_child = 1;
		uatomic_store(&sfx_meta->nr_keys, old_child_nr_keys,
			CMM_RELAXED);
		old_suffix_flag = ft_compressed_node_flag(sfx);
		ft_set_parent(cn->child, old_suffix_flag);
		old_suffix_flag = ft_publish_compressed(ft, sfx, old_suffix_flag);
		created[nr_created++] = old_suffix_flag;
	} else if (suffix_len == 1) {
		struct cds_ft_inode_flag *dest = NULL;

		ret = ft_node_set_nth(ft, &dest,
				cn->key_bytes[diverge_pos + 1],
				cn->child, NULL, NULL);
		if (ret) goto error;
		{
			struct cds_ft_metadata *m =
				cds_ft_item_to_metadata(ft_node_ptr(dest));
			uatomic_store(&m->nr_keys, old_child_nr_keys,
				CMM_RELAXED);
		}
		old_suffix_flag = dest;
		created[nr_created++] = dest;
	} else {
		old_suffix_flag = cn->child;
	}

	/* 2. Build branch node with old direction only. */
	{
		struct cds_ft_inode_flag *dest = NULL;
		struct cds_ft_metadata *branch_meta;

		ret = ft_node_set_nth(ft, &dest, old_ordinal,
				old_suffix_flag, NULL, NULL);
		if (ret) goto error;
		created[nr_created++] = dest;
		branch_flag = dest;

		branch_meta = cds_ft_item_to_metadata(ft_node_ptr(dest));
		uatomic_store(&branch_meta->nr_keys, old_child_nr_keys,
			CMM_RELAXED);
	}

	/* 3. Build prefix -> branch (if needed). */
	if (diverge_pos >= 2) {
		struct cds_ft_compressed_node *pfx;
		struct cds_ft_metadata *pfx_meta;

		pfx = alloc_compressed_node(ft, diverge_pos, &pfx_meta);
		if (!pfx) goto error;
		pfx->child = branch_flag;
		pfx->len = diverge_pos;
		memcpy(pfx->key_bytes, cn->key_bytes, diverge_pos);
		pfx_meta->nr_child = 1;
		uatomic_store(&pfx_meta->nr_keys, cn_meta->nr_keys,
			CMM_RELAXED);
		if (cn_meta->external_nodes)
			pfx_meta->external_nodes = cn_meta->external_nodes;
		top_flag = ft_compressed_node_flag(pfx);
		ft_set_parent(branch_flag, top_flag);
		top_flag = ft_publish_compressed(ft, pfx, top_flag);
		created[nr_created++] = top_flag;
	} else if (diverge_pos == 1) {
		struct cds_ft_inode_flag *dest = NULL;
		struct cds_ft_metadata *pfx_meta;

		ret = ft_node_set_nth(ft, &dest, cn->key_bytes[0],
				branch_flag, NULL, NULL);
		if (ret) goto error;
		pfx_meta = cds_ft_item_to_metadata(ft_node_ptr(dest));
		uatomic_store(&pfx_meta->nr_keys, cn_meta->nr_keys,
			CMM_RELAXED);
		if (cn_meta->external_nodes)
			pfx_meta->external_nodes = cn_meta->external_nodes;
		top_flag = dest;
		created[nr_created++] = dest;
	} else {
		/* diverge_pos == 0: branch IS the top. */
		struct cds_ft_metadata *branch_meta =
			cds_ft_item_to_metadata(ft_node_ptr(branch_flag));
		uatomic_store(&branch_meta->nr_keys, cn_meta->nr_keys,
			CMM_RELAXED);
		if (cn_meta->external_nodes)
			branch_meta->external_nodes = cn_meta->external_nodes;
		top_flag = branch_flag;
	}

	/* 4. Publish the split structure. */
	ft_set_parent(top_flag, d->pnf);
	rcu_assign_pointer(*d->nfp, top_flag);

	/* 5. Density: net = nr_created - 1 at the split depth. */
	if (nr_created > 1)
		ft_propagate_node_density_parent(branch_flag,
			d->depth + diverge_pos, d->depth,
			(long) nr_created - 1);

	/*
	 * 6. Set descent state: branch has an empty slot for the
	 * key's ordinal at the divergence point.
	 */
	d->ppnf = d->pnf;
	d->ppnfp = d->pnfp;
	d->pnf = branch_flag;
	if (top_flag != branch_flag) {
		if (ft_node_compressed(top_flag))
			d->pnfp = &ft_compressed_node_ptr(top_flag)->child;
		else if (ft_node_skip_compressed(top_flag))
			d->pnfp = &ft_skip_to_compressed(top_flag)->child;
		else
			/* Single-child internal prefix: find the slot
			 * holding branch_flag within the prefix node. */
			ft_node_get_nth(top_flag, &d->pnfp,
					cn->key_bytes[0]);
	} else {
		d->pnfp = d->nfp;  /* branch IS the top, parent is the old parent */
	}
	{
		uint8_t new_ordinal = iter_key[diverge_pos];

		d->nf = ft_node_get_nth(branch_flag, &d->nfp, new_ordinal);
		/* nf should be NULL: the branch only has the old direction. */
	}
	d->depth += diverge_pos + 1;

	/* 7. Free old compressed node. */
	free_compressed_node(ft, cn);

	return 0;

error:
	{
		int i;

		for (i = 0; i < nr_created; i++) {
			if (ft_node_compressed(created[i]))
				free_compressed_node(ft,
					ft_compressed_node_ptr(created[i]));
			else if (ft_node_skip_compressed(created[i]))
				free_compressed_node(ft,
					ft_skip_to_compressed(created[i]));
			else
				free_cds_ft_node(ft, ft_node_ptr(created[i]));
		}
	}
	return -ENOMEM;
}

/*
 * Descend through the trie to the child slot at depth @key_len.
 * Stops early if the traversal hits NULL or an external node.
 *
 * On return, d->depth is the number of internal levels successfully
 * traversed (0..key_len).  If d->depth == key_len, the graft slot
 * is at d->nf / d->nfp.  Otherwise the path was incomplete.
 *
 * Used by cds_ft_graft, cds_ft_graft_swap, and cds_ft_detach.
 */
static int ft_split_compressed_graft_key_shorter(struct cds_ft *ft,
		struct ft_descent *d, unsigned int remaining);

static
void ft_descend_to_graft_point(struct cds_ft *ft,
		const uint8_t *key, size_t key_len,
		struct ft_descent *d,
		struct cds_ft_inode_flag **snapshot,
		unsigned int *snapshot_depth,
		int *nr_snapshot)
{
	const uint8_t *ik = key;

	ft_descent_init(d, ft);
	*nr_snapshot = 0;

	for (; d->depth < key_len; ) {
		uint8_t kv;

		if (ft_node_external(d->nf))
			break;
		/* Resolve skip-compressed (e.g. from collapsed entry). */
		if (ft_node_skip_compressed(d->nf))
			d->nf = ft_compressed_node_flag(
				ft_skip_to_compressed(d->nf));
		if (ft_node_compressed(d->nf)) {
			struct cds_ft_compressed_node *cn =
				ft_compressed_node_ptr(d->nf);
			int remaining = key_len - d->depth;
			int cmp = cn->len < remaining ? cn->len : remaining;
			int j;

			j = ft_match_compressed_key(ik, cn, cmp);
			if (j == cmp && cn->len <= remaining) {
				ft_snapshot_push(snapshot, snapshot_depth,
				*nr_snapshot, d->nf, d->depth);
				ft_descent_traverse_compressed(d, cn, &ik);
				continue;
			}
			if (j < cmp) {
				/*
				 * Divergence: split compressed node
				 * at the mismatch point.
				 */
				if (ft_split_compressed_graft(ft, d,
						ik, j))
					break;
				ik += j + 1;
				break;
			}
			/*
			 * Key shorter than compressed path: split
			 * into prefix → suffix at the key endpoint.
			 * The graft point is at the junction.
			 */
			if (ft_split_compressed_graft_key_shorter(
					ft, d, remaining))
				break;
			break;
		}
		if (ft_node_collapsed(d->nf)) {
			struct cds_ft_collapsed_node *col =
				ft_collapsed_node_ptr(d->nf);
			unsigned int remaining = key_len - d->depth;
			unsigned int e;
			bool found = false;

			unsigned int nr_e = ft_collapsed_nr_entries(col);
			struct cds_ft_inode_flag **cptrs =
				ft_collapsed_ptrs(col, nr_e);

			for (e = 0; e < ft_collapsed_count(nr_e); e++) {
				uint8_t data_e = ft_collapsed_load_data(col, e);
				unsigned int slen;
				uint8_t *suffix;
				bool match;

				if (ft_collapsed_entry_dead(data_e, nr_e))
					continue;
				slen = ft_collapsed_suffix_len(col, data_e, e, nr_e);
				if (slen > remaining)
					continue;
				suffix = ft_collapsed_suffix(col, data_e, nr_e);
				match = (ft_key_cmp_ordinals(ik, suffix, slen, slen, false, NULL) == 0);
				if (!match)
					continue;
				ft_snapshot_push(snapshot, snapshot_depth,
				*nr_snapshot, d->nf, d->depth);
				d->ppnf  = d->pnf;
				d->ppnfp = d->pnfp;
				d->pnf   = d->nf;
				d->pnfp  = d->nfp;
				d->nf    = ft_dereference_acquire(cptrs[e]);
				d->nfp   = &cptrs[e];
				d->depth += slen;
				ik += slen;
				found = true;
				break;
			}
			if (!found)
				break; /* Graft point: no matching entry. */
			continue;
		}

		ft_snapshot_push(snapshot, snapshot_depth,
			*nr_snapshot, d->nf, d->depth);
		kv = *(ik++);
		ft_descent_step(d, kv);
	}
}

/*
 * Split a compressed node for graft when the key is shorter than the
 * compressed path.  Builds: [prefix] → [suffix] → old_child.
 *
 * Unlike the insert key-shorter split, there is no junction node —
 * the graft point is at the boundary between prefix and suffix.
 *
 * On success, publishes the split, updates the descent state for
 * ft_store_at_graft_point, adds nodes to the snapshot, frees the
 * old compressed node, and returns 0.  On failure returns -1.
 */
static
int ft_split_compressed_graft_key_shorter(struct cds_ft *ft,
		struct ft_descent *d,
		unsigned int remaining)
{
	struct cds_ft_compressed_node *cn = ft_compressed_node_ptr(d->nf);
	struct cds_ft_metadata *cn_meta =
		cds_ft_item_to_metadata((struct cds_ft_inode *) cn);
	unsigned int prefix_len = remaining;
	unsigned int suffix_len = cn->len - remaining;
	struct cds_ft_inode_flag *suffix_flag;
	struct cds_ft_inode_flag *prefix_flag;
	unsigned long child_nr_keys;

	if (!ft_node_external(cn->child)) {
		struct cds_ft_metadata *cm =
			cds_ft_item_to_metadata(ft_node_ptr(cn->child));
		child_nr_keys = cm->nr_keys;
	} else if (ft_node_ptr(cn->child)) {
		child_nr_keys = 1;
	} else {
		child_nr_keys = 0;
	}

	/* Build suffix → old child. */
	if (suffix_len >= 2) {
		struct cds_ft_compressed_node *sfx;
		struct cds_ft_metadata *sfx_meta;

		sfx = alloc_compressed_node(ft, suffix_len, &sfx_meta);
		if (!sfx) return -1;
		sfx->child = cn->child;
		sfx->len = suffix_len;
		memcpy(sfx->key_bytes, &cn->key_bytes[remaining],
			suffix_len);
		sfx_meta->nr_child = 1;
		uatomic_store(&sfx_meta->nr_keys, child_nr_keys,
			CMM_RELAXED);
		suffix_flag = ft_compressed_node_flag(sfx);
		ft_set_parent(cn->child, suffix_flag);
		suffix_flag = ft_publish_compressed(ft, sfx, suffix_flag);
	} else {
		struct cds_ft_inode_flag *dest = NULL;
		int ret;

		ret = ft_node_set_nth(ft, &dest,
			cn->key_bytes[remaining],
			cn->child, NULL, NULL);
		if (ret) return -1;
		{
			struct cds_ft_metadata *m =
				cds_ft_item_to_metadata(ft_node_ptr(dest));
			uatomic_store(&m->nr_keys, child_nr_keys,
				CMM_RELAXED);
		}
		suffix_flag = dest;
	}

	/* Build prefix → suffix. */
	if (prefix_len >= 2) {
		struct cds_ft_compressed_node *pfx;
		struct cds_ft_metadata *pfx_meta;

		pfx = alloc_compressed_node(ft, prefix_len, &pfx_meta);
		if (!pfx) {
			if (ft_node_compressed(suffix_flag))
				free_compressed_node(ft,
					ft_compressed_node_ptr(suffix_flag));
			else
				free_cds_ft_node(ft, ft_node_ptr(suffix_flag));
			return -1;
		}
		pfx->child = suffix_flag;
		pfx->len = prefix_len;
		memcpy(pfx->key_bytes, cn->key_bytes, prefix_len);
		pfx_meta->nr_child = 1;
		uatomic_store(&pfx_meta->nr_keys, cn_meta->nr_keys,
			CMM_RELAXED);
		if (cn_meta->external_nodes)
			pfx_meta->external_nodes = cn_meta->external_nodes;
		prefix_flag = ft_compressed_node_flag(pfx);
		ft_set_parent(suffix_flag, prefix_flag);
		prefix_flag = ft_publish_compressed(ft, pfx, prefix_flag);
	} else {
		/* prefix_len == 1 */
		struct cds_ft_inode_flag *dest = NULL;
		struct cds_ft_metadata *pfx_meta;
		int ret;

		ret = ft_node_set_nth(ft, &dest, cn->key_bytes[0],
			suffix_flag, NULL, NULL);
		if (ret) {
			if (ft_node_compressed(suffix_flag))
				free_compressed_node(ft,
					ft_compressed_node_ptr(suffix_flag));
			else
				free_cds_ft_node(ft, ft_node_ptr(suffix_flag));
			return -1;
		}
		pfx_meta = cds_ft_item_to_metadata(ft_node_ptr(dest));
		uatomic_store(&pfx_meta->nr_keys, cn_meta->nr_keys,
			CMM_RELAXED);
		if (cn_meta->external_nodes)
			pfx_meta->external_nodes = cn_meta->external_nodes;
		prefix_flag = dest;
	}

	/* Publish and set descent state. */
	ft_set_parent(prefix_flag, d->pnf);
	rcu_assign_pointer(*d->nfp, prefix_flag);

	/* Density: net = +1 (old compressed → prefix + suffix = 2 nodes, net +1). */
	ft_propagate_node_density_parent(prefix_flag, d->depth, d->depth, 1);

	d->ppnf = d->pnf;
	d->ppnfp = d->pnfp;
	d->pnf = prefix_flag;
	if (ft_node_compressed(prefix_flag))
		d->pnfp = &ft_compressed_node_ptr(prefix_flag)->child;
	else
		ft_node_get_nth(prefix_flag, &d->pnfp,
				cn->key_bytes[0]);
	d->nf = suffix_flag;
	d->nfp = d->pnfp;
	d->depth += remaining;

	free_compressed_node(ft, cn);
	return 0;
}

/*
 * Build a branch for key[start .. end-1] with @leaf at the bottom.
 * When the path is 2+ bytes, a single compressed node is used instead
 * of a chain of single-child internal nodes.  Returns the topmost
 * flagged node, or NULL on allocation failure (all nodes freed).
 */
static
struct cds_ft_inode_flag *ft_build_branch(struct cds_ft *ft,
		const uint8_t *key, unsigned int start, unsigned int end,
		struct cds_ft_inode_flag *leaf,
		unsigned long subtree_external_count)
{
	unsigned int path_len = end - start;
	struct cds_ft_inode_flag *compressed;

	compressed = ft_try_compress_chain(ft, key, end, start,
		leaf, NULL);
	if (compressed == (void *) (long) -ENOMEM)
		return NULL;
	if (compressed) {
		struct cds_ft_metadata *m =
			cds_ft_item_to_metadata(ft_node_ptr(compressed));

		uatomic_store(&m->nr_keys, subtree_external_count,
			CMM_RELAXED);
		return compressed;
	}
	if (path_len >= 1) {
		struct cds_ft_inode_flag *cur = leaf;
		int i;

		for (i = (int) end - 1; i >= (int) start; i--) {
			struct cds_ft_inode_flag *dest = NULL;
			int ret;

			ret = ft_node_set_nth(ft, &dest,
				key[i],
				cur, NULL, NULL);
			if (ret) {
				while (cur != leaf) {
					struct cds_ft_inode_flag *next;
					uint8_t kv = key[i + 1];

					next = ft_node_get_nth(cur, NULL, kv);
					free_cds_ft_node(ft, ft_node_ptr(cur));
					cur = next;
					i++;
				}
				return NULL;
			}
			{
				struct cds_ft_metadata *m =
					cds_ft_item_to_metadata(
						ft_node_ptr(dest));
				uatomic_store(&m->nr_keys,
					subtree_external_count,
					CMM_RELAXED);
			}
			cur = dest;
		}
		return cur;
	}
	/* path_len == 0: leaf is the branch. */
	return leaf;
}

/*
 * Free a branch previously built by ft_build_branch().
 * If the top node is compressed, free just the compressed node.
 * Otherwise, walk down the single internal node freeing it.
 * The bottom-most leaf is left untouched.
 */
static
void ft_free_branch(struct cds_ft *ft,
		const uint8_t *key __attribute__((unused)),
		unsigned int start __attribute__((unused)),
		unsigned int end __attribute__((unused)),
		struct cds_ft_inode_flag *top_node)
{
	if (ft_node_compressed(top_node)) {
		free_compressed_node(ft,
			ft_compressed_node_ptr(top_node));
	} else if (ft_node_internal(top_node)) {
		free_cds_ft_node(ft, ft_node_ptr(top_node));
	}
}

/*
 * Store graft_payload at the graft point described by @d.
 *
 * Handles two cases:
 * - d->depth == key_len: the slot exists; add via ft_node_set_nth.
 * - d->depth < key_len: the path is incomplete; build intermediate
 *   internal nodes via ft_build_branch, displacing any external node
 *   on the path into the branch's metadata.
 *
 * Return CDS_FT_STATUS_OK on success, CDS_FT_STATUS_POPULATED_ERROR
 * if the slot is already occupied, CDS_FT_STATUS_MEMORY_ERROR on
 * allocation failure.
 */
static
enum cds_ft_status ft_store_at_graft_point(struct cds_ft *ft,
		const uint8_t *key, size_t key_len,
		struct ft_descent *d,
		struct cds_ft_inode_flag *graft_payload,
		unsigned long graft_external_count)
{
	struct cds_ft_inode *old_recompacted_node = NULL;

	if (d->depth == key_len) {
		struct cds_ft_metadata *pmeta;
		struct cds_ft_inode_flag *dest;
		int ret;

		if (ft_node_ptr(d->nf))
			return CDS_FT_STATUS_POPULATED_ERROR;

		pmeta = cds_ft_item_to_metadata(ft_node_ptr(d->pnf));

		dest = d->pnf;
		ret = ft_node_set_nth(ft, &dest,
			key[key_len - 1],
			graft_payload, &old_recompacted_node, pmeta);
		if (ret)
			return CDS_FT_STATUS_MEMORY_ERROR;

		rcu_assign_pointer(*d->pnfp, dest);

		if (old_recompacted_node)
			free_cds_ft_node(ft, old_recompacted_node);
	} else {
		unsigned int i = d->depth;
		struct cds_ft_inode_flag *branch;
		struct cds_ft_node *displaced = NULL;

		if (ft_node_ptr(d->nf) && ft_node_external(d->nf))
			displaced = (struct cds_ft_node *)
				ft_node_ptr(d->nf);

		branch = ft_build_branch(ft, key, i, key_len, graft_payload,
				graft_external_count);
		if (!branch)
			return CDS_FT_STATUS_MEMORY_ERROR;

		if (displaced) {
			struct cds_ft_metadata *bm =
				cds_ft_item_to_metadata(
					ft_node_ptr(branch));
			bm->external_nodes = displaced;
			uatomic_store(&bm->nr_keys, bm->nr_keys + 1,
				CMM_RELAXED);
		}

		if (displaced) {
			ft_set_parent(branch, d->pnf);
			rcu_assign_pointer(*d->nfp, branch);
		} else {
			struct cds_ft_inode_flag *dest = d->pnf;
			struct cds_ft_metadata *pmeta;
			int ret;

			pmeta = cds_ft_item_to_metadata(
					ft_node_ptr(d->pnf));

			ret = ft_node_set_nth(ft, &dest,
				key[i - 1],
				branch, &old_recompacted_node, pmeta);
			if (ret) {
				ft_free_branch(ft, key, i, key_len, branch);
				return CDS_FT_STATUS_MEMORY_ERROR;
			}
			rcu_assign_pointer(*d->pnfp, dest);

			if (old_recompacted_node)
				free_cds_ft_node(ft, old_recompacted_node);
		}
	}
	return CDS_FT_STATUS_OK;
}

enum cds_ft_status cds_ft_graft(struct cds_ft *dst_ft,
		const uint8_t *_key, size_t _key_len,
		struct cds_ft *src_ft)
{
	struct cds_ft_metadata *src_rmeta;
	size_t key_len, src_max;
	enum cds_ft_status status;

	if (!dst_ft || !src_ft || dst_ft == src_ft)
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	if (dst_ft->group != src_ft->group)
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;

	/*
	 * Root-level graft (key_len == 0) is valid for both
	 * variable-length and fixed-length groups: it swaps the entire
	 * root, so no key-length constraint applies.  Bypass
	 * ft_key_len() which would reject 0 != fixed_len.
	 */
	if (_key_len == 0) {
		key_len = 0;
	} else {
		key_len = ft_key_len(dst_ft, _key_len);
		if (!valid_key_len(dst_ft, key_len) ||
				dst_ft->group->key_len != CDS_FT_LEN_VARIABLE)
			return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}

	uint8_t ordinal_buf[FT_MAX_KEY_LEN];
	const uint8_t *key = ordinal_buf;

	ft_key_to_ordinals(ordinal_buf, _key, key_len, &dst_ft->group->key_map);

	src_max = uatomic_load(&src_ft->max_used_key_len, CMM_RELAXED);
	if (key_len > 0 && src_max > dst_ft->group->max_key_len - key_len)
		return CDS_FT_STATUS_OVERFLOW_ERROR;

	src_rmeta = ft_root_metadata(src_ft);

	/* Check if source trie is empty. */
	if (src_rmeta->nr_child == 0 && !src_rmeta->external_nodes)
		return CDS_FT_STATUS_OK;

	if (key_len == 0) {
		struct cds_ft_metadata *dst_rmeta = ft_root_metadata(dst_ft);
		struct cds_ft_inode *fresh_root;
		struct cds_ft_metadata *fresh_meta;

		/* Destination must be empty for a root-level graft. */
		if (dst_rmeta->nr_child != 0 || dst_rmeta->external_nodes)
			return CDS_FT_STATUS_POPULATED_ERROR;

		/*
		 * Allocate a fresh empty root for the source before
		 * swapping, so the source remains a valid trie.
		 */
		fresh_root = alloc_cds_ft_node(dst_ft, &ft_types[0], &fresh_meta);
		if (!fresh_root)
			return CDS_FT_STATUS_MEMORY_ERROR;

		/*
		 * Swap root pointers.  The source's root carries all
		 * metadata (nr_child, external_nodes) with it.
		 */
		rcu_assign_pointer(dst_ft->root, src_ft->root);
		rcu_assign_pointer(src_ft->root, ft_node_flag(fresh_root, 0));
		goto done;
	}

	{
		struct ft_descent d;
		struct cds_ft_inode *fresh_node;
		struct cds_ft_metadata *fresh_meta;
		struct cds_ft_inode_flag *graft_snapshot[FT_MAX_DEPTH];
		unsigned int graft_snapshot_depth[FT_MAX_DEPTH];
		int nr_graft_snapshot;
		unsigned long src_count = src_rmeta->nr_keys;

		/*
		 * Preallocate a fresh empty root for the source trie
		 * before the point of no return, so we can fail cleanly
		 * on memory shortage instead of calling abort().
		 */
		fresh_node = alloc_cds_ft_node(src_ft, &ft_types[0], &fresh_meta);
		if (!fresh_node)
			return CDS_FT_STATUS_MEMORY_ERROR;

		ft_descend_to_graft_point(dst_ft, key, key_len, &d,
				graft_snapshot, graft_snapshot_depth,
				&nr_graft_snapshot);

		/*
		 * The source root node becomes the graft payload.  Its
		 * metadata.external_nodes (NIL-key entries in the source)
		 * naturally becomes the entries at depth key_len in the
		 * destination.  No relocation needed.
		 */
		status = ft_store_at_graft_point(dst_ft, key, key_len,
						  &d, src_ft->root,
						  src_count);
		if (status != CDS_FT_STATUS_OK) {
			free_cds_ft_node(src_ft, fresh_node);
			return status;
		}

		ft_propagate_external_count_parent(*d.pnfp,
				(long) src_count);

		/*
		 * Propagate density addition for the grafted subtree.
		 * The source root's density contribution must be added
		 * to each ancestor within the density window.
		 */
		if (!ft_node_external(src_ft->root)) {
			struct cds_ft_metadata *graft_meta =
				cds_ft_item_to_metadata(
					ft_node_ptr(src_ft->root));
			int si;

			for (si = nr_graft_snapshot - 1; si >= 0; si--) {
				struct cds_ft_metadata *am;
				unsigned int distance;

				if (!ft_node_ptr(graft_snapshot[si]))
					continue;
				if (graft_snapshot_depth[si] >= key_len)
					continue;
				distance = key_len - graft_snapshot_depth[si];
				if (distance > FT_NODE_DENSITY_DEPTH)
					break;
				am = cds_ft_item_to_metadata(
					ft_node_ptr(graft_snapshot[si]));
				ft_density_add(am, 0,
					ft_child_density_contribution(
						graft_meta, distance));
			}
		}

		/* Give source a fresh empty root. */
		rcu_assign_pointer(src_ft->root, ft_node_flag(fresh_node, 0));
	}

done:
	{
		size_t nm = key_len + src_max;

		if (nm > uatomic_load(&dst_ft->max_used_key_len, CMM_RELAXED))
			uatomic_store(&dst_ft->max_used_key_len, nm,
				      CMM_RELAXED);
	}

	uatomic_store(&src_ft->max_used_key_len, 0, CMM_RELAXED);

#ifdef FEATURE_FT_COLLAPSE
	if (key_len > 0)
		ft_check_collapse_on_path(dst_ft, key, key_len);
#endif
	return CDS_FT_STATUS_OK;
}

enum cds_ft_status cds_ft_graft_swap(struct cds_ft *dst_ft,
		const uint8_t *_key, size_t _key_len,
		struct cds_ft *swap_ft)
{
	size_t key_len, swap_max;

	if (!dst_ft || !swap_ft || dst_ft == swap_ft)
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	if (dst_ft->group != swap_ft->group)
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;

	/*
	 * Root-level swap (key_len == 0) is valid for both
	 * variable-length and fixed-length groups.  See cds_ft_graft.
	 */
	if (_key_len == 0) {
		key_len = 0;
	} else {
		key_len = ft_key_len(dst_ft, _key_len);
		if (!valid_key_len(dst_ft, key_len) ||
				dst_ft->group->key_len != CDS_FT_LEN_VARIABLE)
			return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}

	uint8_t ordinal_buf[FT_MAX_KEY_LEN];
	const uint8_t *key = ordinal_buf;

	ft_key_to_ordinals(ordinal_buf, _key, key_len, &dst_ft->group->key_map);

	swap_max = uatomic_load(&swap_ft->max_used_key_len, CMM_RELAXED);
	if (key_len > 0 && swap_max > dst_ft->group->max_key_len - key_len)
		return CDS_FT_STATUS_OVERFLOW_ERROR;

	if (key_len == 0) {
		/*
		 * Swap entire tries: exchange root pointers.
		 * Each root carries its own metadata (nr_child,
		 * external_nodes), so no relocation is needed.
		 */
		struct cds_ft_inode_flag *tmp = dst_ft->root;
		size_t dm;

		rcu_assign_pointer(dst_ft->root, swap_ft->root);
		rcu_assign_pointer(swap_ft->root, tmp);

		dm = uatomic_load(&dst_ft->max_used_key_len, CMM_RELAXED);
		if (swap_max > dm)
			uatomic_store(&dst_ft->max_used_key_len,
				      swap_max, CMM_RELAXED);
		uatomic_store(&swap_ft->max_used_key_len, dm,
			      CMM_RELAXED);

		return CDS_FT_STATUS_OK;
	}

	{
		struct ft_descent d;
		struct cds_ft_metadata *pmeta, *swap_rmeta;
		struct cds_ft_inode_flag *old_child, *old_swap_root;
		struct cds_ft_inode *fresh = NULL;
		struct cds_ft_metadata *fresh_meta = NULL;
		struct cds_ft_inode_flag *graft_snapshot[FT_MAX_DEPTH];
		unsigned int graft_snapshot_depth[FT_MAX_DEPTH];
		int nr_graft_snapshot;
		bool swap_empty;
		bool need_fresh;
		unsigned long old_count, swap_count;

		ft_descend_to_graft_point(dst_ft, key, key_len, &d,
				graft_snapshot, graft_snapshot_depth,
				&nr_graft_snapshot);

		if (d.depth < key_len) {
			/*
			 * Path incomplete: nothing at or below the graft
			 * point.  swap_ft receives empty content.
			 * Delegate to graft for intermediate-node creation.
			 */
			return cds_ft_graft(dst_ft, key, _key_len, swap_ft);
		}

		/* Snapshot old content at the graft slot. */
		old_child = d.nf;

		old_swap_root = swap_ft->root;
		swap_rmeta = ft_root_metadata(swap_ft);
		swap_empty = (swap_rmeta->nr_child == 0
				&& !swap_rmeta->external_nodes);
		swap_count = swap_empty ? 0 : swap_rmeta->nr_keys;

		/* Compute old_count from the content being displaced. */
		if (!ft_node_external(old_child)) {
			struct cds_ft_metadata *old_meta =
				cds_ft_item_to_metadata(ft_node_ptr(old_child));
			old_count = old_meta->nr_keys;
		} else if (ft_node_ptr(old_child)) {
			old_count = 1;	/* One key (possibly with duplicates). */
		} else {
			old_count = 0;
		}

		/*
		 * A fresh root for swap_ft is needed when old_child
		 * is not an internal node and the swap trie is not
		 * empty (i.e. the old swap root is consumed by the
		 * graft).  Preallocate it here, before the point of
		 * no return, so we can fail cleanly.
		 */
		need_fresh = ft_node_external(old_child) && !swap_empty;
		if (need_fresh) {
			fresh = alloc_cds_ft_node(swap_ft,
				&ft_types[0], &fresh_meta);
			if (!fresh)
				return CDS_FT_STATUS_MEMORY_ERROR;
		}

		/*
		 * Atomic store at graft point.  If swap is empty,
		 * place NULL (removing the subtree); otherwise place
		 * the swap root node directly.
		 */
		if (!swap_empty)
			ft_set_parent(old_swap_root, d.pnf);
		rcu_assign_pointer(*d.nfp,
			swap_empty ? NULL : old_swap_root);

		/* Update parent nr_child on NULL <-> non-NULL transition. */
		pmeta = cds_ft_item_to_metadata(ft_node_ptr(d.pnf));
		if (!ft_node_ptr(old_child) && !swap_empty)
			pmeta->nr_child++;
		else if (ft_node_ptr(old_child) && swap_empty)
			pmeta->nr_child--;

		/* Propagate external node count delta through ancestors. */
		if (swap_count != old_count)
			ft_propagate_external_count_parent(d.pnf,
					(long) swap_count - (long) old_count);

		/*
		 * Propagate density delta for the swapped subtrees.
		 * Subtract the old child's contribution and add the
		 * new (swap) child's contribution at each ancestor
		 * within the density window.
		 */
		{
			struct cds_ft_metadata *old_meta = NULL, *new_meta = NULL;
			int si;

			if (!ft_node_external(old_child) && ft_node_ptr(old_child))
				old_meta = cds_ft_item_to_metadata(
						ft_node_ptr(old_child));
			if (!swap_empty && !ft_node_external(old_swap_root))
				new_meta = cds_ft_item_to_metadata(
						ft_node_ptr(old_swap_root));

			if (old_meta || new_meta) {
				for (si = nr_graft_snapshot - 1; si >= 0; si--) {
					struct cds_ft_metadata *am;
					unsigned int distance;
					unsigned long add = 0, sub = 0;

					if (!ft_node_ptr(graft_snapshot[si]))
						continue;
					if (graft_snapshot_depth[si] >= key_len)
						continue;
					distance = key_len -
						graft_snapshot_depth[si];
					if (distance > FT_NODE_DENSITY_DEPTH)
						break;
					am = cds_ft_item_to_metadata(
						ft_node_ptr(graft_snapshot[si]));
					if (old_meta)
						sub = ft_child_density_contribution(
							old_meta, distance);
					if (new_meta)
						add = ft_child_density_contribution(
							new_meta, distance);
					if (add >= sub)
						ft_density_add(am, 0, add - sub);
					else
						ft_density_sub(am, 0, sub - add);
				}
			}
		}

		/*
		 * Set up swap_ft to hold old content from the graft
		 * point.  If old_child is an internal node, it
		 * becomes swap_ft's root directly (its
		 * metadata.external_nodes carries the entries at the
		 * graft key).  Otherwise, use the preallocated fresh
		 * root and place any external node chain as NIL-key
		 * entries.
		 */
		if (!ft_node_external(old_child)) {
			/* Clear parent: old_child is now a root. */
			cds_ft_item_to_metadata(
				ft_node_ptr(old_child))->parent = NULL;
			rcu_assign_pointer(swap_ft->root, old_child);
			if (swap_empty)
				free_cds_ft_node(swap_ft,
					ft_node_ptr(old_swap_root));
		} else if (swap_empty) {
			if (ft_node_ptr(old_child)) {
				swap_rmeta->external_nodes =
					(struct cds_ft_node *)
					ft_node_ptr(old_child);
				uatomic_store(&swap_rmeta->nr_keys, old_count, CMM_RELEASE);
			}
		} else {
			rcu_assign_pointer(swap_ft->root, ft_node_flag(fresh, 0));
			if (ft_node_ptr(old_child)) {
				fresh_meta->external_nodes =
					(struct cds_ft_node *)
					ft_node_ptr(old_child);
				uatomic_store(&fresh_meta->nr_keys, old_count, CMM_RELEASE);
			}
		}

		{
			size_t nm = key_len + swap_max;
			size_t dm = uatomic_load(&dst_ft->max_used_key_len,
						 CMM_RELAXED);
			if (nm > dm)
				uatomic_store(&dst_ft->max_used_key_len, nm,
					      CMM_RELAXED);
			uatomic_store(&swap_ft->max_used_key_len,
				      dm > key_len ? dm - key_len : 0,
				      CMM_RELAXED);
		}
		return CDS_FT_STATUS_OK;
	}
}

enum cds_ft_status cds_ft_detach(struct cds_ft *ft,
		const uint8_t *_key, size_t _key_len,
		struct cds_ft **result_ft)
{
	struct cds_ft *detached;
	struct cds_ft_inode_flag *child;
	size_t key_len;
	enum cds_ft_status status;

	*result_ft = NULL;

	if (!ft)
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;

	/*
	 * Root-level detach (key_len == 0) is valid for both
	 * variable-length and fixed-length groups.  See cds_ft_graft.
	 */
	if (_key_len == 0) {
		key_len = 0;
	} else {
		key_len = ft_key_len(ft, _key_len);
		if (!valid_key_len(ft, key_len) ||
				ft->group->key_len != CDS_FT_LEN_VARIABLE)
			return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}

	uint8_t ordinal_buf[FT_MAX_KEY_LEN];
	const uint8_t *key = ordinal_buf;

	ft_key_to_ordinals(ordinal_buf, _key, key_len, &ft->group->key_map);

	if (key_len == 0) {
		struct cds_ft_metadata *rmeta = ft_root_metadata(ft);
		struct cds_ft_inode *fresh_node;
		struct cds_ft_metadata *fresh_meta;

		/* Check if source trie is empty. */
		if (rmeta->nr_child == 0 && !rmeta->external_nodes)
			return CDS_FT_STATUS_NOT_FOUND;

		status = cds_ft_create(ft->group, &detached);
		if (status != CDS_FT_STATUS_OK)
			return status;

		/*
		 * Allocate a fresh empty root for the source trie
		 * before swapping.
		 */
		fresh_node = alloc_cds_ft_node(ft, &ft_types[0], &fresh_meta);
		if (!fresh_node) {
			cds_ft_destroy(detached);
			return CDS_FT_STATUS_MEMORY_ERROR;
		}

		/*
		 * Move the source root into the detached trie.
		 * Free the empty root that cds_ft_create allocated for
		 * the detached trie, and replace it with the source root.
		 */
		free_cds_ft_node(detached, ft_node_ptr(detached->root));
		/* No readers in detached root yet. */
		detached->root = ft->root;
		/* Clear parent: this node is now a root. */
		cds_ft_item_to_metadata(ft_node_ptr(detached->root))->parent = NULL;
		uatomic_store(&detached->max_used_key_len,
			      uatomic_load(&ft->max_used_key_len, CMM_RELAXED),
			      CMM_RELAXED);

		/* Give source a fresh empty root. */
		rcu_assign_pointer(ft->root, ft_node_flag(fresh_node, 0));

		*result_ft = detached;
		return CDS_FT_STATUS_OK;
	}

	/*
	 * key_len > 0: descent with snapshot tracking for
	 * ft_detach_node's upward pruning walk.
	 */
	{
		struct ft_detach_descent dd;
		const uint8_t *ik = key;

		ft_detach_descent_init(&dd, ft);

		for (; dd.d.depth < key_len; ) {
			uint8_t kv;
			const struct cds_ft_metadata *meta;

			if (!ft_node_ptr(dd.d.nf))
				return CDS_FT_STATUS_NOT_FOUND;
			if (ft_node_external(dd.d.nf))
				return CDS_FT_STATUS_NOT_FOUND;
			if (ft_node_compressed(dd.d.nf)) {
				struct cds_ft_compressed_node *cn =
					ft_compressed_node_ptr(dd.d.nf);
				const struct cds_ft_metadata *cn_meta =
					cds_ft_item_to_metadata(
						(struct cds_ft_inode *) cn);

				ft_detach_descent_track(&dd, cn_meta);
				ft_descent_traverse_compressed(&dd.d, cn, &ik);
				if (ft_node_ptr(dd.d.nf) && dd.pending) {
					dd.det_nfp = dd.d.nfp;
					dd.pending = false;
				}
				continue;
			}
			if (ft_node_collapsed(dd.d.nf)) {
				struct cds_ft_collapsed_node *col =
					ft_collapsed_node_ptr(dd.d.nf);
				const struct cds_ft_metadata *col_meta =
					cds_ft_item_to_metadata(
						(struct cds_ft_inode *) col);
				unsigned int remaining = key_len - dd.d.depth;
				unsigned int e;
				bool found = false;

				unsigned int nr_e = ft_collapsed_nr_entries(col);
				struct cds_ft_inode_flag **cptrs =
					ft_collapsed_ptrs(col, nr_e);

				for (e = 0; e < ft_collapsed_count(nr_e); e++) {
					uint8_t data_e = ft_collapsed_load_data(col, e);
					unsigned int slen;
					uint8_t *suffix;
					bool match;

					if (ft_collapsed_entry_dead(data_e, nr_e))
						continue;
					slen = ft_collapsed_suffix_len(col, data_e, e, nr_e);
					suffix = ft_collapsed_suffix(col, data_e, nr_e);
					/*
					 * Partial prefix match (detach prefix
					 * falls within this suffix): explode the
					 * collapsed node and restart descent.
					 */
					if (slen > remaining) {
						if (ft_key_cmp_ordinals(ik, suffix, remaining, remaining, false, NULL) == 0)
							goto detach_collapsed_explode;
						continue;
					}
					match = (ft_key_cmp_ordinals(ik, suffix, slen, slen, false, NULL) == 0);
					if (!match)
						continue;
					if (!ft_node_ptr(cptrs[e]))
						return CDS_FT_STATUS_NOT_FOUND;
					ft_detach_descent_track(&dd, col_meta);
					dd.d.ppnf  = dd.d.pnf;
					dd.d.ppnfp = dd.d.pnfp;
					dd.d.pnf   = dd.d.nf;
					dd.d.pnfp  = dd.d.nfp;
					dd.d.nf    = ft_dereference_acquire(cptrs[e]);
					dd.d.nfp   = &cptrs[e];
					dd.d.depth += slen;
					ik += slen;
					if (ft_node_ptr(dd.d.nf) && dd.pending) {
						dd.det_nfp = dd.d.nfp;
						dd.pending = false;
					}
					found = true;
					break;
				}
				if (!found)
					return CDS_FT_STATUS_NOT_FOUND;
				continue;

			detach_collapsed_explode:
				{
					struct cds_ft_inode_flag *internal_flag;

					internal_flag = ft_explode_entries(ft,
						col, cptrs,
						0, ft_collapsed_count(ft_collapsed_nr_entries(col)), 0);
					if (!internal_flag)
						return CDS_FT_STATUS_MEMORY_ERROR;
					{
						struct cds_ft_metadata *int_meta =
							cds_ft_item_to_metadata(
								ft_node_ptr(internal_flag));
						int_meta->external_nodes =
							col_meta->external_nodes;
					}
					ft_init_node_density(internal_flag);

					ft_set_parent(internal_flag, dd.d.pnf);
					rcu_assign_pointer(*dd.d.nfp, internal_flag);
					free_collapsed_node(ft, col);

					dd.d.nf = internal_flag;
					continue;
				}
			}

			meta = cds_ft_item_to_metadata(ft_node_ptr(dd.d.nf));
			ft_detach_descent_track(&dd, meta);

			kv = *(ik++);
			ft_detach_descent_step(&dd, kv);
		}

		child = dd.d.nf;

		if (!ft_node_ptr(child))
			return CDS_FT_STATUS_NOT_FOUND;

		/*
		 * Compute the external node count of the subtree
		 * being detached before it is removed from the trie.
		 */
		{
			unsigned long detached_count;

			if (!ft_node_external(child)) {
				struct cds_ft_metadata *child_meta =
					cds_ft_item_to_metadata(
						ft_node_ptr(child));
				detached_count = child_meta->nr_keys;
			} else {
				detached_count = 1;	/* One key (possibly with duplicates). */
			}

			status = cds_ft_create(ft->group, &detached);
			if (status != CDS_FT_STATUS_OK)
				return status;

			/*
			 * Propagate count removal through ancestors
			 * before detach to avoid writing freed metadata.
			 */
			ft_propagate_external_count_parent(dd.d.pnf,
				-(long) detached_count);

			/*
			 * Propagate density removal for the detached
			 * subtree.  Walk up from the child's parent via
			 * metadata->parent, subtracting the child's
			 * density contribution at each ancestor within
			 * the 6-level window.
			 */
			if (!ft_node_external(child)) {
				struct cds_ft_metadata *child_meta =
					cds_ft_item_to_metadata(
						ft_node_ptr(child));
				struct cds_ft_inode_flag *anc = child;
				unsigned int anc_depth = key_len;

				while (anc) {
					struct cds_ft_metadata *am;
					unsigned int distance;
					struct cds_ft_inode_flag *parent_nf;

					am = cds_ft_item_to_metadata(
						ft_node_ptr(anc));
					parent_nf = am->parent;
					if (!parent_nf)
						break;
					if (anc_depth < key_len) {
						distance = key_len - anc_depth;
						if (distance > FT_NODE_DENSITY_DEPTH)
							break;
						ft_density_sub(am, 0,
							ft_child_density_contribution(
								child_meta, distance));
					}
					anc_depth -= ft_parent_depth_span(
						parent_nf, anc);
					anc = parent_nf;
				}
			}

			/*
			 * Detach child from the source trie and prune
			 * empty branches above.  After this, child is
			 * no longer reachable from the live trie for
			 * new readers.
			 */
			{
				int ret = ft_detach_node(ft,
							 dd.det_nfp,
							 dd.det_pfp,
							 key_len);
				assert(ret != -ENOENT);
				if (ret < 0) {
					/*
					 * Recompaction failed (-ENOMEM).
					 * Undo propagation and abort.
					 */
					ft_propagate_external_count_parent(
						dd.d.pnf,
						(long) detached_count);
					cds_ft_destroy(detached);
					return CDS_FT_STATUS_MEMORY_ERROR;
				}
			}

			/*
			 * If the detached child is an internal node, it
			 * becomes the detached trie's root directly.
			 * Its metadata.external_nodes carries the
			 * entries at the detach key, which become
			 * NIL-key entries in the detached trie.  Free
			 * the empty root that cds_ft_create allocated
			 * and replace it.
			 *
			 * If the child is an external node, place it in
			 * the detached trie's (empty) root metadata as
			 * a NIL-key entry.
			 */
			if (!ft_node_external(child)) {
				free_cds_ft_node(detached,
					ft_node_ptr(detached->root));
				/* No readers in detached root yet. */
				detached->root = child;
				/* Clear parent: this node is now a root. */
				cds_ft_item_to_metadata(
					ft_node_ptr(child))->parent = NULL;
			} else {
				struct cds_ft_metadata *dmeta =
					ft_root_metadata(detached);
				dmeta->external_nodes =
					(struct cds_ft_node *)
					ft_node_ptr(child);
				uatomic_store(&dmeta->nr_keys, detached_count, CMM_RELAXED);
			}
		}
		{
			size_t fm = uatomic_load(&ft->max_used_key_len,
						 CMM_RELAXED);
			uatomic_store(&detached->max_used_key_len,
				      fm > key_len ? fm - key_len : 0,
				      CMM_RELAXED);
		}

		*result_ft = detached;
#ifdef FEATURE_FT_COLLAPSE
		ft_check_collapse_on_path(ft, key, key_len);
#endif
		return CDS_FT_STATUS_OK;
	}
}

size_t cds_ft_key_len(const struct cds_ft *ft)
{
	return ft->group->key_len;
}

size_t cds_ft_max_key_len(const struct cds_ft *ft)
{
	return ft->group->max_key_len;
}

size_t cds_ft_max_used_key_len(const struct cds_ft *ft)
{
	return uatomic_load(&ft->max_used_key_len, CMM_RELAXED);
}

enum cds_ft_status cds_ft_key_map(const struct cds_ft *ft, uint8_t *key_to_ordinal, uint8_t *ordinal_to_key)
{
	if (ft->group->key_map.identity)
		return CDS_FT_STATUS_NOT_FOUND;
	memcpy(key_to_ordinal, ft->group->key_map.key_to_ordinal, sizeof(ft->group->key_map.key_to_ordinal));
	memcpy(ordinal_to_key, ft->group->key_map.ordinal_to_key, sizeof(ft->group->key_map.ordinal_to_key));
	return CDS_FT_STATUS_OK;
}

bool cds_ft_empty(struct cds_ft *ft)
{
	struct cds_ft_inode_flag *root_flag;
	struct cds_ft_inode *root_node;
	unsigned int type_idx;
	const struct cds_ft_type *type;
	struct cds_ft_metadata *rmeta;

	CDS_FT_ASSERT_RCU_READ_LOCKED(ft);

	root_flag = rcu_dereference(ft->root);
	root_node = ft_node_ptr(root_flag);

	/*
	 * Compressed root is never empty: recompaction in ft_detach_node
	 * replaces an emptied compressed root with an internal node.
	 */
	if (ft_node_compressed(root_flag))
		return false;

	type_idx = ft_node_type(root_flag);
	type = &ft_types[type_idx];
	rmeta = cds_ft_item_to_metadata(root_node);

	/*
	 * As a root node special-case, only a type-0 linear node with
	 * data[0] == 0 represents an empty node.
	 */
	if (!ft_type_is_linear(type->type_class))
		return false;
	if (ft_linear_node_get_nr_child(type, root_node) != 0)
		return false;
	return !uatomic_load(&rmeta->external_nodes, CMM_RELAXED);
}

/*
 * Handle compressed node in cds_ft_count_keys_prefix().
 *
 * Returns FT_COMPRESSED_CONTINUE to advance past the compressed path,
 * or FT_COMPRESSED_END when the prefix is fully consumed inside the
 * compressed node (count written to *count_ret).
 */
static
enum ft_compressed_action ft_count_prefix_compressed(
		struct cds_ft_inode_flag **node_flag_p,
		unsigned int *i_p, const uint8_t *prefix,
		size_t prefix_len, unsigned long *count_ret)
{
	struct cds_ft_inode_flag *node_flag = *node_flag_p;
	unsigned int i = *i_p;
	struct cds_ft_compressed_node *cn =
		ft_compressed_node_ptr(node_flag);
	unsigned int remaining = prefix_len - i;
	unsigned int cmp = cn->len < remaining ? cn->len : remaining;
	unsigned int j;

	j = ft_match_compressed_key(&prefix[i], cn, cmp);
	if (j < cmp) {
		*count_ret = 0;
		return FT_COMPRESSED_END;
	}
	if (cn->len >= remaining) {
		struct cds_ft_metadata *cn_meta =
			cds_ft_item_to_metadata(
				(struct cds_ft_inode *) cn);

		*count_ret = uatomic_load(&cn_meta->nr_keys, CMM_ACQUIRE);
		return FT_COMPRESSED_END;
	}
	*i_p = i + cn->len - 1;
	*node_flag_p = ft_dereference_acquire_prefetch(cn->child);
	return FT_COMPRESSED_CONTINUE;
}

#ifdef FEATURE_FT_COLLAPSE
/*
 * Collapsed node handling for cds_ft_count_keys_prefix.
 * Scan entries for a suffix that matches the remaining prefix bytes.
 * If prefix ends within a collapsed suffix span, return the child's nr_keys.
 * If no match, return 0.
 */
static
enum ft_compressed_action ft_count_prefix_collapsed(struct cds_ft_inode_flag **node_flag_p,
		unsigned int *i_p, const uint8_t *prefix,
		size_t prefix_len, unsigned long *count_ret)
{
	struct cds_ft_inode_flag *node_flag = *node_flag_p;
	unsigned int i = *i_p;
	struct cds_ft_collapsed_node *cn = ft_collapsed_node_ptr(node_flag);
	unsigned int nr_e = ft_collapsed_nr_entries(cn);
	struct cds_ft_inode_flag **ptrs = ft_collapsed_ptrs(cn, nr_e);
	unsigned int remaining = prefix_len - i;
	unsigned int e;

	for (e = 0; e < ft_collapsed_count(nr_e); e++) {
		unsigned int slen, j;
		uint8_t *suffix;
		bool match;
		uint8_t data_e = ft_collapsed_load_data(cn, e);

		if (ft_collapsed_entry_dead(data_e, nr_e))
			continue;
		slen = ft_collapsed_suffix_len(cn, data_e, e, nr_e);

		if (slen >= remaining) {
			/* Suffix covers the rest of the prefix — compare. */
			suffix = ft_collapsed_suffix(cn, data_e, nr_e);
			match = true;
			for (j = 0; j < remaining; j++) {
				if (prefix[i + j] != suffix[j]) {
					match = false;
					break;
				}
			}
			if (!match)
				continue;
			/* Prefix ends within or at this entry's span. */
			if (!ft_node_external(ptrs[e])) {
				struct cds_ft_metadata *m =
					cds_ft_item_to_metadata(ft_node_ptr(ptrs[e]));
				*count_ret = uatomic_load(&m->nr_keys, CMM_ACQUIRE);
			} else if (ft_node_ptr(ptrs[e])) {
				*count_ret = 1;
			} else {
				*count_ret = 0;
			}
			return FT_COMPRESSED_END;
		}
		/* Suffix is shorter than remaining prefix — check prefix. */
		suffix = ft_collapsed_suffix(cn, data_e, nr_e);
		match = true;
		for (j = 0; j < slen; j++) {
			if (prefix[i + j] != suffix[j]) {
				match = false;
				break;
			}
		}
		if (!match)
			continue;
		/* Full suffix match, continue descent into child. */
		*i_p = i + slen - 1;
		{
			struct cds_ft_inode_flag *child =
				ft_dereference_acquire_prefetch(ptrs[e]);
			if (ft_node_skip_compressed(child))
				child = ft_compressed_node_flag(
					ft_skip_to_compressed(child));
			*node_flag_p = child;
		}
		return FT_COMPRESSED_CONTINUE;
	}
	*count_ret = 0;
	return FT_COMPRESSED_END;
}
#endif /* FEATURE_FT_COLLAPSE */

unsigned long cds_ft_count_keys_prefix(struct cds_ft *ft,
		const uint8_t *_prefix, size_t prefix_len)
{
	struct cds_ft_inode_flag *node_flag;
	unsigned int i;
	uint8_t ordinal_buf[FT_MAX_KEY_LEN];
	const uint8_t *prefix = ordinal_buf;

	CDS_FT_ASSERT_RCU_READ_LOCKED(ft);

	if (prefix_len > ft->group->max_key_len)
		return 0;
	ft_key_to_ordinals(ordinal_buf, _prefix, prefix_len, &ft->group->key_map);

	node_flag = ft_dereference_acquire_prefetch(ft->root);

	for (i = 0; i < prefix_len; i++) {
		uint8_t kv;

		if (ft_node_external(node_flag))
			return 0;
		if (ft_node_compressed(node_flag)) {
			enum ft_compressed_action act;
			unsigned long count;

			act = ft_count_prefix_compressed(
				&node_flag, &i, prefix,
				prefix_len, &count);
			if (act == FT_COMPRESSED_END)
				return count;
			continue;
		}
		if (ft_node_collapsed(node_flag)) {
			enum ft_compressed_action act;
			unsigned long count;

			act = ft_count_prefix_collapsed(
				&node_flag, &i, prefix,
				prefix_len, &count);
			if (act == FT_COMPRESSED_END)
				return count;
			continue;
		}
		kv = prefix[i];
		node_flag = ft_node_get_nth(node_flag, NULL, kv);
	}

	if (!ft_node_ptr(node_flag))
		return 0;
	if (ft_node_internal(node_flag)) {
		struct cds_ft_metadata *metadata =
			cds_ft_item_to_metadata(ft_node_ptr(node_flag));
		return uatomic_load(&metadata->nr_keys, CMM_ACQUIRE);
	}
	if (ft_node_compressed(node_flag)) {
		struct cds_ft_metadata *cn_meta =
			cds_ft_item_to_metadata(ft_node_ptr(node_flag));
		return uatomic_load(&cn_meta->nr_keys, CMM_ACQUIRE);
	}
	if (ft_node_collapsed(node_flag)) {
		struct cds_ft_metadata *col_meta =
			cds_ft_item_to_metadata(ft_node_ptr(node_flag));
		return uatomic_load(&col_meta->nr_keys, CMM_ACQUIRE);
	}
	/* External node: one key (possibly with duplicates). */
	return 1;
}

unsigned long cds_ft_count_keys(struct cds_ft *ft)
{
	return cds_ft_count_keys_prefix(ft, NULL, 0);
}

/*
 * Count keys in a child node (nr_keys if internal, 1 if external).
 */
static inline
unsigned long ft_child_key_count(struct cds_ft_inode_flag *child)
{
	if (!ft_node_external(child)) {
		struct cds_ft_metadata *m =
			cds_ft_item_to_metadata(ft_node_ptr(child));
		return uatomic_load(&m->nr_keys, CMM_ACQUIRE);
	}
	return 1;
}

/*
 * Handle compressed node in cds_ft_lookup_nth().
 *
 * Fills the compressed path into ordinal_key and iter_path, advances
 * level and node_flag past the compressed segment.
 *
 * Returns FT_COMPRESSED_CONTINUE on success, FT_COMPRESSED_BREAK if
 * the child pointer is NULL.
 */
static
enum ft_compressed_action ft_lookup_nth_compressed(
		struct cds_ft_inode_flag **node_flag_p,
		int *level_p, uint8_t *ordinal_key,
		struct cds_ft_iter *iter)
{
	struct cds_ft_inode_flag *node_flag = *node_flag_p;
	int level = *level_p;
	struct cds_ft_compressed_node *cn =
		ft_compressed_node_ptr(node_flag);

	ft_fill_compressed_path(cn, ordinal_key, level - 1,
		iter_path_node(iter), level, node_flag);
	level += cn->len - 1;
	node_flag = ft_dereference_acquire_prefetch(cn->child);
	if (!ft_node_ptr(node_flag)) {
		*node_flag_p = node_flag;
		*level_p = level;
		return FT_COMPRESSED_BREAK;
	}
	iter_path_node(iter)[level] = node_flag;
	if (ft_node_external(node_flag)) {
		*node_flag_p = node_flag;
		*level_p = level;
		return FT_COMPRESSED_BREAK;
	}
	*node_flag_p = node_flag;
	*level_p = level;
	return FT_COMPRESSED_CONTINUE;
}

#ifdef FEATURE_FT_COLLAPSE
/*
 * Collapsed node handling for cds_ft_lookup_nth.
 *
 * Scans collapsed entries in ascending suffix order, accumulating
 * key counts.  When the target nth key falls within an entry's
 * subtree, fills ordinal_key and iter_path for the suffix, advances
 * node_flag to the entry's child, and returns CONTINUE.
 *
 * Returns BREAK if the target overflows all entries (shouldn't
 * happen in a consistent trie).
 */
static
enum ft_compressed_action ft_lookup_nth_collapsed(
		struct cds_ft_inode_flag **node_flag_p,
		int *level_p, uint8_t *ordinal_key,
		struct cds_ft_iter *iter,
		unsigned long *remaining_p,
		unsigned int nr)
{
	struct cds_ft_inode_flag *node_flag = *node_flag_p;
	int level = *level_p;
	struct cds_ft_collapsed_node *cn = ft_collapsed_node_ptr(node_flag);
	struct cds_ft_inode_flag **ptrs = ft_collapsed_ptrs(cn, nr);
	bool visited[FT_COLLAPSED_MAX_ENTRIES_MAX];
	unsigned int i, best, steps;

	memset(visited, 0, ft_collapsed_count(nr) * sizeof(visited[0]));

	/*
	 * Walk entries in ascending suffix order (selection sort style).
	 * N is small (≤ 24), all in 1 cache line.
	 */
	for (steps = 0; steps < ft_collapsed_count(nr); steps++) {
		unsigned long child_keys;
		unsigned int slen;
		uint8_t *suffix;
		uint8_t best_d = 0;

		/* Find the smallest unvisited live entry. */
		best = UINT_MAX;
		for (i = 0; i < ft_collapsed_count(nr); i++) {
			uint8_t data_i = ft_collapsed_load_data(cn, i);

			if (visited[i] || ft_collapsed_entry_dead(data_i, nr))
				continue;
			if (best == UINT_MAX) {
				best = i;
				best_d = data_i;
				continue;
			}
			/* Compare suffixes lexicographically. */
			{
				uint8_t *sa = ft_collapsed_suffix(cn, best_d, nr);
				unsigned int la = ft_collapsed_suffix_len(cn, best_d, best, nr);
				uint8_t *sb = ft_collapsed_suffix(cn, data_i, nr);
				unsigned int lb = ft_collapsed_suffix_len(cn, data_i, i, nr);
				unsigned int cmp = la < lb ? la : lb;
				int r = memcmp(sb, sa, cmp);

				if (r < 0 || (r == 0 && lb < la)) {
					best = i;
					best_d = data_i;
				}
			}
		}
		if (best == UINT_MAX)
			break;
		visited[best] = true;

		if (!ft_node_ptr(ptrs[best]))
			continue;
		child_keys = ft_child_key_count(ptrs[best]);
		if (*remaining_p < child_keys) {
			/* Target is in this entry's subtree. */
			slen = ft_collapsed_suffix_len(cn, best_d, best, nr);
			suffix = ft_collapsed_suffix(cn, best_d, nr);
			{
				unsigned int k;

				for (k = 0; k < slen; k++) {
					ordinal_key[level - 1 + k] = suffix[k];
					iter_path_node(iter)[level + k] =
						ft_collapsed_node_flag(cn);
				}
			}
			node_flag = ft_dereference_acquire_prefetch(ptrs[best]);
			if (ft_node_ptr(node_flag) &&
			    ft_node_skip_compressed(node_flag))
				node_flag = ft_compressed_node_flag(
					ft_skip_to_compressed(node_flag));
			if (!ft_node_ptr(node_flag) || ft_node_external(node_flag)) {
				level += slen;
				if (ft_node_ptr(node_flag))
					iter_path_node(iter)[level] = node_flag;
				*node_flag_p = node_flag;
				*level_p = level;
				return FT_COMPRESSED_BREAK;
			}
			level += slen - 1;
			iter_path_node(iter)[level + 1] = node_flag;
			*node_flag_p = node_flag;
			*level_p = level;
			return FT_COMPRESSED_CONTINUE;
		}
		*remaining_p -= child_keys;
	}
	return FT_COMPRESSED_BREAK;
}
#endif /* FEATURE_FT_COLLAPSE */

/*
 * Lookup the nth key (0-indexed) in forward (smallest-first) order.
 *
 * Descends through the trie using per-node nr_keys counters to skip
 * entire subtrees, yielding O(depth) time complexity rather than
 * O(n) iteration.
 *
 * At each internal node:
 * 1. If external_nodes are present, they represent the key at this
 *    depth (shortest in the subtree). If n == 0, found. Else n -= 1.
 * 2. Iterate children in ascending ordinal order. For each child,
 *    determine its key count (nr_keys if internal, 1 if external).
 *    If n < count, descend. Else n -= count and continue.
 */
enum cds_ft_status cds_ft_lookup_nth(struct cds_ft *ft,
		struct cds_ft_iter *iter,
		unsigned long n)
{
	struct cds_ft_inode_flag *node_flag;
	uint8_t ordinal_key[FT_MAX_KEY_LEN];
	int level;
	unsigned long remaining = n;

	CDS_FT_ASSERT_RCU_READ_LOCKED(ft);

	iter_debug_path_snapshot(iter);
	memset(ordinal_key, 0, ft->group->max_key_len * sizeof(ordinal_key[0]));

	node_flag = ft_dereference_acquire_prefetch(ft->root);
	iter_path_node(iter)[0] = node_flag;

	ft_delay_reader();

	for (level = 1; ; level++) {
		struct cds_ft_metadata *metadata;
		struct cds_ft_inode_flag *child;
		struct cds_ft_node *ext;
		uint8_t child_key = 0;
		int pivot;

		if (ft_node_external(node_flag))
			break;

		metadata = cds_ft_item_to_metadata(ft_node_ptr(node_flag));

		/* Keys at this node's depth come first in ordinal order. */
		ext = ft_dereference_acquire(metadata->external_nodes);
		if (ext) {
			if (remaining == 0) {
				/* Found: the key at this node's depth. */
				iter->key_len = level - 1;
				{
					int j;

					for (j = 0; j < level - 1; j++)
						iter_key(iter)[j] = ordinal_key[j];
				}
				iter->node = ext;
				iter->path_valid = true;
				iter_debug_path_update(iter);
				iter->path_len = level;
				iter->status = CDS_FT_STATUS_OK;
				goto end;
			}
			remaining--;
		}

		if (ft_node_compressed(node_flag)) {
			enum ft_compressed_action act;

			act = ft_lookup_nth_compressed(&node_flag,
				&level, ordinal_key, iter);
			if (act == FT_COMPRESSED_BREAK)
				break;
			continue;
		}
		if (ft_node_collapsed(node_flag)) {
			enum ft_compressed_action act;
			unsigned int col_nr_e = ft_collapsed_nr_entries(
				ft_collapsed_node_ptr(node_flag));

			act = ft_lookup_nth_collapsed(&node_flag,
				&level, ordinal_key, iter,
				&remaining, col_nr_e);
			if (act == FT_COMPRESSED_BREAK)
				break;
			continue;
		}

		/* Iterate children in ascending ordinal order. */
		pivot = -1;
		child = ft_node_get_direction(node_flag, pivot, &child_key, FT_RIGHT);
		while (ft_node_ptr(child)) {
			unsigned long child_keys;

			child_keys = ft_child_key_count(child);

			if (remaining < child_keys) {
				/* Target is in this child's subtree. Descend. */
				ordinal_key[level - 1] = child_key;
				iter_path_node(iter)[level] = child;
				node_flag = child;
				goto next_level;
			}
			remaining -= child_keys;
			pivot = child_key;
			child = ft_node_get_direction(node_flag, pivot, &child_key, FT_RIGHT);
		}

		/* Exhausted all children without finding. */
		break;

next_level:
		;
	}

	/* Reached a leaf (external node). */
	if (ft_node_ptr(node_flag) && ft_node_external(node_flag) && remaining == 0) {
		iter->key_len = level - 1;
		{
			int j;

			for (j = 0; j < level - 1; j++)
				iter_key(iter)[j] = ordinal_key[j];
		}
		iter->node = (struct cds_ft_node *) ft_node_ptr(node_flag);
		iter->path_valid = true;
		iter_debug_path_update(iter);
		iter->path_len = level;
		iter->status = CDS_FT_STATUS_OK;
		goto end;
	}

	iter->node = NULL;
	iter->path_valid = true;
	iter_debug_path_update(iter);
	iter->path_len = 0;
	iter->status = CDS_FT_STATUS_NOT_FOUND;

end:
	iter_auto_invalidate_path(iter);
	return iter->status;
}

/*
 * Handle compressed node in cds_ft_lookup_nth_last().
 *
 * In reverse order, process the child subtree first (larger keys),
 * then fall through to external_nodes (smallest = last).
 *
 * Returns FT_COMPRESSED_CONTINUE when descending into the child,
 * FT_COMPRESSED_BREAK when the child is NULL or external (leaf),
 * or FT_COMPRESSED_END to signal the caller to fall through to
 * check_ext_nth_last (remaining updated, child keys exhausted).
 */
static
enum ft_compressed_action ft_lookup_nth_last_compressed(
		struct cds_ft_inode_flag **node_flag_p,
		int *level_p, unsigned long *remaining_p,
		uint8_t *ordinal_key, struct cds_ft_iter *iter)
{
	struct cds_ft_inode_flag *node_flag = *node_flag_p;
	int level = *level_p;
	struct cds_ft_compressed_node *cn =
		ft_compressed_node_ptr(node_flag);

	if (ft_node_ptr(cn->child)) {
		unsigned long child_keys =
			ft_child_key_count(cn->child);

		if (*remaining_p < child_keys) {
			ft_fill_compressed_path(cn,
				ordinal_key, level - 1,
				iter_path_node(iter), level,
				node_flag);
			level += cn->len - 1;
			node_flag = ft_dereference_acquire_prefetch(cn->child);
			if (!ft_node_ptr(node_flag)) {
				*node_flag_p = node_flag;
				*level_p = level;
				return FT_COMPRESSED_BREAK;
			}
			iter_path_node(iter)[level] = node_flag;
			if (ft_node_external(node_flag)) {
				*node_flag_p = node_flag;
				*level_p = level;
				return FT_COMPRESSED_BREAK;
			}
			*node_flag_p = node_flag;
			*level_p = level;
			return FT_COMPRESSED_CONTINUE;
		}
		*remaining_p -= child_keys;
	}
	return FT_COMPRESSED_END;
}

#ifdef FEATURE_FT_COLLAPSE
/*
 * Collapsed node handling for cds_ft_lookup_nth_last.
 * Same as ft_lookup_nth_collapsed but walks entries in descending
 * suffix order.  Returns END if all entries exhausted (caller
 * should check external_nodes).
 */
static
enum ft_compressed_action ft_lookup_nth_last_collapsed(
		struct cds_ft_inode_flag **node_flag_p,
		int *level_p, uint8_t *ordinal_key,
		struct cds_ft_iter *iter,
		unsigned long *remaining_p,
		unsigned int nr)
{
	struct cds_ft_inode_flag *node_flag = *node_flag_p;
	int level = *level_p;
	struct cds_ft_collapsed_node *cn = ft_collapsed_node_ptr(node_flag);
	struct cds_ft_inode_flag **ptrs = ft_collapsed_ptrs(cn, nr);
	bool visited[FT_COLLAPSED_MAX_ENTRIES_MAX];
	unsigned int i, best, steps;

	memset(visited, 0, ft_collapsed_count(nr) * sizeof(visited[0]));

	for (steps = 0; steps < ft_collapsed_count(nr); steps++) {
		unsigned long child_keys;
		unsigned int slen;
		uint8_t *suffix;
		uint8_t best_d = 0;

		/* Find the largest unvisited live entry. */
		best = UINT_MAX;
		for (i = 0; i < ft_collapsed_count(nr); i++) {
			uint8_t data_i = ft_collapsed_load_data(cn, i);

			if (visited[i] || ft_collapsed_entry_dead(data_i, nr))
				continue;
			if (best == UINT_MAX) {
				best = i;
				best_d = data_i;
				continue;
			}
			{
				uint8_t *sa = ft_collapsed_suffix(cn, best_d, nr);
				unsigned int la = ft_collapsed_suffix_len(cn, best_d, best, nr);
				uint8_t *sb = ft_collapsed_suffix(cn, data_i, nr);
				unsigned int lb = ft_collapsed_suffix_len(cn, data_i, i, nr);
				unsigned int cmp = la < lb ? la : lb;
				int r = memcmp(sb, sa, cmp);

				if (r > 0 || (r == 0 && lb > la)) {
					best = i;
					best_d = data_i;
				}
			}
		}
		if (best == UINT_MAX)
			break;
		visited[best] = true;

		if (!ft_node_ptr(ptrs[best]))
			continue;
		child_keys = ft_child_key_count(ptrs[best]);
		if (*remaining_p < child_keys) {
			slen = ft_collapsed_suffix_len(cn, best_d, best, nr);
			suffix = ft_collapsed_suffix(cn, best_d, nr);
			{
				unsigned int k;

				for (k = 0; k < slen; k++) {
					ordinal_key[level - 1 + k] = suffix[k];
					iter_path_node(iter)[level + k] =
						ft_collapsed_node_flag(cn);
				}
			}
			node_flag = ft_dereference_acquire_prefetch(ptrs[best]);
			if (ft_node_ptr(node_flag) &&
			    ft_node_skip_compressed(node_flag))
				node_flag = ft_compressed_node_flag(
					ft_skip_to_compressed(node_flag));
			if (!ft_node_ptr(node_flag) || ft_node_external(node_flag)) {
				level += slen;
				if (ft_node_ptr(node_flag))
					iter_path_node(iter)[level] = node_flag;
				*node_flag_p = node_flag;
				*level_p = level;
				return FT_COMPRESSED_BREAK;
			}
			level += slen - 1;
			iter_path_node(iter)[level + 1] = node_flag;
			*node_flag_p = node_flag;
			*level_p = level;
			return FT_COMPRESSED_CONTINUE;
		}
		*remaining_p -= child_keys;
	}
	return FT_COMPRESSED_END;
}
#endif /* FEATURE_FT_COLLAPSE */

/*
 * Lookup the nth key (0-indexed) in reverse (largest-first) order.
 *
 * Same principle as cds_ft_lookup_nth but descends from the right:
 * at each internal node, iterate children in descending ordinal order
 * first, then check external_nodes last (they are the smallest key
 * in the subtree).
 */
enum cds_ft_status cds_ft_lookup_nth_last(struct cds_ft *ft,
		struct cds_ft_iter *iter,
		unsigned long n)
{
	struct cds_ft_inode_flag *node_flag;
	uint8_t ordinal_key[FT_MAX_KEY_LEN];
	int level;
	unsigned long remaining = n;

	CDS_FT_ASSERT_RCU_READ_LOCKED(ft);

	iter_debug_path_snapshot(iter);
	memset(ordinal_key, 0, ft->group->max_key_len * sizeof(ordinal_key[0]));

	node_flag = ft_dereference_acquire_prefetch(ft->root);
	iter_path_node(iter)[0] = node_flag;

	for (level = 1; ; level++) {
		struct cds_ft_metadata *metadata;
		struct cds_ft_inode_flag *child;
		struct cds_ft_node *ext;
		uint8_t child_key = 0;
		int pivot;

		if (ft_node_external(node_flag))
			break;

		metadata = cds_ft_item_to_metadata(ft_node_ptr(node_flag));

		if (ft_node_compressed(node_flag)) {
			enum ft_compressed_action act;

			act = ft_lookup_nth_last_compressed(
				&node_flag, &level, &remaining,
				ordinal_key, iter);
			if (act == FT_COMPRESSED_BREAK)
				break;
			if (act == FT_COMPRESSED_END)
				goto check_ext_nth_last;
			continue;
		}
		if (ft_node_collapsed(node_flag)) {
			enum ft_compressed_action act;
			unsigned int col_nr_e = ft_collapsed_nr_entries(
				ft_collapsed_node_ptr(node_flag));

			/* Reuse nth helper in reverse: walk entries largest-first. */
			act = ft_lookup_nth_last_collapsed(
				&node_flag, &level, ordinal_key,
				iter, &remaining, col_nr_e);
			if (act == FT_COMPRESSED_BREAK)
				break;
			if (act == FT_COMPRESSED_END)
				goto check_ext_nth_last;
			continue;
		}

		/* Iterate children in descending ordinal order first. */
		pivot = FT_ENTRY_PER_NODE;
		child = ft_node_get_direction(node_flag, pivot, &child_key, FT_LEFT);
		while (ft_node_ptr(child)) {
			unsigned long child_keys;

			child_keys = ft_child_key_count(child);

			if (remaining < child_keys) {
				/* Target is in this child's subtree. Descend. */
				ordinal_key[level - 1] = child_key;
				iter_path_node(iter)[level] = child;
				node_flag = child;
				goto next_level;
			}
			remaining -= child_keys;
			pivot = child_key;
			child = ft_node_get_direction(node_flag, pivot, &child_key, FT_LEFT);
		}

check_ext_nth_last:
		/* External_nodes at this depth are the smallest (last in reverse). */
		ext = ft_dereference_acquire(metadata->external_nodes);
		if (ext) {
			if (remaining == 0) {
				iter->key_len = level - 1;
				{
					int j;

					for (j = 0; j < level - 1; j++)
						iter_key(iter)[j] = ordinal_key[j];
				}
				iter->node = ext;
				iter->path_valid = true;
				iter_debug_path_update(iter);
				iter->path_len = level;
				iter->status = CDS_FT_STATUS_OK;
				goto end;
			}
			remaining--;
		}

		/* Exhausted all children without finding. */
		break;

next_level:
		;
	}

	/* Reached a leaf (external node). */
	if (ft_node_ptr(node_flag) && ft_node_external(node_flag) && remaining == 0) {
		iter->key_len = level - 1;
		{
			int j;

			for (j = 0; j < level - 1; j++)
				iter_key(iter)[j] = ordinal_key[j];
		}
		iter->node = (struct cds_ft_node *) ft_node_ptr(node_flag);
		iter->path_valid = true;
		iter_debug_path_update(iter);
		iter->path_len = level;
		iter->status = CDS_FT_STATUS_OK;
		goto end;
	}

	iter->node = NULL;
	iter->path_valid = true;
	iter_debug_path_update(iter);
	iter->path_len = 0;
	iter->status = CDS_FT_STATUS_NOT_FOUND;

end:
	iter_auto_invalidate_path(iter);
	return iter->status;
}

/*
 * Re-descend from the root following @key to rebuild the iterator path
 * and ordinal_key arrays.  Returns the depth reached, or -1 on error.
 */
static
int ft_rebuild_path(struct cds_ft *ft,
		struct cds_ft_iter *iter,
		const uint8_t *key, size_t key_len,
		uint8_t *ordinal_key)
{
	struct cds_ft_inode_flag *node_flag;
	unsigned int i;

	node_flag = ft_dereference_acquire_prefetch(ft->root);
	iter_path_node(iter)[0] = node_flag;

	for (i = 0; i < key_len; i++) {
		uint8_t ordinal;

		if (ft_node_external(node_flag))
			return -1;

		if (ft_node_compressed(node_flag)) {
			struct cds_ft_compressed_node *cn =
				ft_compressed_node_ptr(node_flag);
			unsigned int j;

			if (i + cn->len > key_len)
				return -1;
			for (j = 0; j < cn->len; j++) {
				uint8_t ord = key[i + j];
				if (ord != cn->key_bytes[j])
					return -1;
				ordinal_key[i + j] = ord;
				iter_path_node(iter)[i + j + 1] =
					node_flag;
			}
			i += cn->len - 1;
			node_flag = ft_dereference_acquire_prefetch(cn->child);
			if (!ft_node_ptr(node_flag))
				return -1;
			iter_path_node(iter)[i + 1] = node_flag;
			continue;
		}
		if (ft_node_collapsed(node_flag)) {
			struct cds_ft_collapsed_node *cn =
				ft_collapsed_node_ptr(node_flag);
			unsigned int nr_e = ft_collapsed_nr_entries(cn);
			struct cds_ft_inode_flag **ptrs =
				ft_collapsed_ptrs(cn, nr_e);
			unsigned int remaining = key_len - i;
			unsigned int e;
			bool found = false;

			for (e = 0; e < ft_collapsed_count(nr_e); e++) {
				unsigned int slen, j;
				uint8_t *suffix;
				bool match;
				uint8_t data_e = ft_collapsed_load_data(cn, e);

				if (ft_collapsed_entry_dead(data_e, nr_e))
					continue;
				slen = ft_collapsed_suffix_len(cn, data_e, e, nr_e);
				if (slen > remaining)
					continue;
				suffix = ft_collapsed_suffix(cn, data_e, nr_e);
				match = true;
				for (j = 0; j < slen; j++) {
					uint8_t ord = key[i + j];
					if (ord != suffix[j]) {
						match = false;
						break;
					}
					ordinal_key[i + j] = ord;
					iter_path_node(iter)[i + j + 1] =
						(struct cds_ft_inode_flag *)
						ft_collapsed_node_flag(cn);
				}
				if (!match)
					continue;
				i += slen - 1;
				node_flag = ft_dereference_acquire_prefetch(ptrs[e]);
				if (!ft_node_ptr(node_flag))
					return -1;
				if (ft_node_skip_compressed(node_flag))
					node_flag = ft_compressed_node_flag(
						ft_skip_to_compressed(node_flag));
				iter_path_node(iter)[i + 1] = node_flag;
				found = true;
				break;
			}
			if (!found)
				return -1;
			continue;
		}

		ordinal = key[i];
		ordinal_key[i] = ordinal;
		node_flag = ft_node_get_nth(node_flag, NULL, ordinal);
		if (!ft_node_ptr(node_flag))
			return -1;
		iter_path_node(iter)[i + 1] = node_flag;
	}
	return (int) key_len;
}

/*
 * Handle compressed node in cds_ft_iter_skip_forward's descend_forward
 * loop.
 *
 * Fills ordinal_key and iter_path entries for the compressed path,
 * then advances level and node_flag past the compressed segment.
 *
 * Returns FT_COMPRESSED_CONTINUE on success, FT_COMPRESSED_BREAK if
 * the child pointer is NULL or is an external (leaf) node.
 */
static
enum ft_compressed_action ft_skip_forward_compressed(
		struct cds_ft_inode_flag **node_flag_p,
		int *level_p, uint8_t *ordinal_key,
		struct cds_ft_iter *iter)
{
	struct cds_ft_inode_flag *node_flag = *node_flag_p;
	int level = *level_p;
	struct cds_ft_compressed_node *cn =
		ft_compressed_node_ptr(node_flag);
	int j;

	for (j = 0; j < cn->len; j++) {
		ordinal_key[level + j] = cn->key_bytes[j];
		if (j > 0)
			iter_path_node(iter)[level + j] = node_flag;
	}
	level += cn->len;
	node_flag = ft_dereference_acquire_prefetch(cn->child);
	if (!ft_node_ptr(node_flag)) {
		*node_flag_p = node_flag;
		*level_p = level;
		return FT_COMPRESSED_BREAK;
	}
	iter_path_node(iter)[level] = node_flag;
	if (ft_node_external(node_flag)) {
		*node_flag_p = node_flag;
		*level_p = level;
		return FT_COMPRESSED_BREAK;
	}
	*node_flag_p = node_flag;
	*level_p = level;
	return FT_COMPRESSED_CONTINUE;
}

/*
 * Skip forward by @n keys from the current iterator position using
 * local traversal.
 *
 * Walks up from the current leaf, at each ancestor counting keys in
 * rightward siblings until enough are accumulated, then descends into
 * the target subtree using the lookup_nth algorithm.  Only touches
 * nodes between the start and end positions, so concurrent mutations
 * in unrelated key ranges do not affect the result.
 */
enum cds_ft_status cds_ft_iter_skip_forward(struct cds_ft *ft,
		struct cds_ft_iter *iter,
		unsigned long n)
{
	uint8_t ordinal_key[FT_MAX_KEY_LEN];
	unsigned long remaining;
	int depth, level = 0;
	bool at_external_nodes;
	/*
	 * Cache nr_entries from the downward collapsed walk so the
	 * going-up handler uses the same acquire-loaded snapshot.
	 * 0 means unset (collapsed nodes always have >= 1 entry by
	 * construction).  Consumed and reset to 0 after use, so
	 * collapsed nodes encountered at higher levels during
	 * going-up get a fresh load.
	 */
	unsigned int cached_col_nr_e = 0;

	CDS_FT_ASSERT_RCU_READ_LOCKED(ft);

	if (!iter->node)
		return CDS_FT_STATUS_NOT_FOUND;
	if (n == 0)
		return CDS_FT_STATUS_OK;

	iter_debug_path_snapshot(iter);

	/* Rebuild path from root to current key. */
	depth = ft_rebuild_path(ft, iter, iter_key(iter), iter->key_len,
			ordinal_key);
	if (depth < 0)
		goto not_found;

	remaining = n;

	/*
	 * Determine whether the current key sits at an internal or
	 * compressed node's external_nodes (variable-length prefix key)
	 * or at a leaf child.
	 */
	at_external_nodes = !ft_node_external(iter_path_node(iter)[depth]);

	/*
	 * If at external_nodes of an internal/compressed node, all
	 * children of that node are to the right.  Try to satisfy the
	 * skip within them.
	 */
	if (at_external_nodes) {
		struct cds_ft_inode_flag *parent = iter_path_node(iter)[depth];
		struct cds_ft_metadata *pmeta =
			cds_ft_item_to_metadata(ft_node_ptr(parent));
		unsigned long right_keys = uatomic_load(&pmeta->nr_keys, CMM_ACQUIRE) - 1; /* exclude self */

		if (remaining <= right_keys) {
			/*
			 * Target is among the children.  Use lookup_nth
			 * descent within this subtree, skipping the
			 * external_nodes (already behind us).
			 */
			remaining--;  /* skip external_nodes key (us) */

			if (ft_node_compressed(parent)) {
				struct cds_ft_compressed_node *cn =
					ft_compressed_node_ptr(parent);
				unsigned long ck;
				int j;

				if (!ft_node_ptr(cn->child))
					goto skip_fwd_walk_up;
				ck = ft_child_key_count(cn->child);
				if (remaining < ck) {
					for (j = 0; j < cn->len; j++) {
						ordinal_key[depth + j] =
							cn->key_bytes[j];
						if (j > 0)
							iter_path_node(iter)[depth + j]
								= parent;
					}
					level = depth + cn->len;
					iter_path_node(iter)[level] =
						cn->child;
					goto descend_forward;
				}
				remaining -= ck;
			} else if (ft_node_collapsed(parent)) {
				/*
				 * Walk collapsed entries in ascending suffix
				 * order.  ft_lookup_nth_collapsed fills the
				 * iter path and ordinal_key through the
				 * matched suffix — don't overwrite them.
				 */
				enum ft_compressed_action act;
				int lv = depth;

				cached_col_nr_e = ft_collapsed_nr_entries(
					ft_collapsed_node_ptr(parent));
				act = ft_lookup_nth_collapsed(
					&parent, &lv, ordinal_key,
					iter, &remaining,
					cached_col_nr_e);
				if (act == FT_COMPRESSED_BREAK) {
					/* Leaf at path[lv]. */
					level = lv;
					goto descend_forward;
				}
				if (act == FT_COMPRESSED_CONTINUE) {
					/* Non-leaf child at path[lv+1]. */
					level = lv + 1;
					goto descend_forward;
				}
			} else {
				struct cds_ft_inode_flag *child;
				uint8_t child_key = 0;
				int pivot = -1;

				child = ft_node_get_direction(parent, pivot,
						&child_key, FT_RIGHT);
				while (ft_node_ptr(child)) {
					unsigned long ck = ft_child_key_count(child);

					if (remaining < ck) {
						ordinal_key[depth] = child_key;
						iter_path_node(iter)[depth + 1] = child;
						level = depth + 1;
						goto descend_forward;
					}
					remaining -= ck;
					pivot = child_key;
					child = ft_node_get_direction(parent, pivot,
							&child_key, FT_RIGHT);
				}
			}
		}
		remaining -= right_keys;
		/* Continue walking up from depth-1. */
		level = depth;
	} else {
		/* At a leaf child: start walking up from the parent. */
		level = depth;
	}

skip_fwd_walk_up:
	/*
	 * Walk up: at each ancestor, count keys in rightward siblings
	 * of the child we came from.
	 */
	for (level--; level >= 0; level--) {
		struct cds_ft_inode_flag *ancestor = iter_path_node(iter)[level];
		struct cds_ft_inode_flag *child;
		uint8_t child_key = 0;
		int pivot;

		if (ft_node_external(ancestor))
			continue;
		/*
		 * Compressed path levels have no siblings: skip.
		 */
		if (ft_node_compressed(ancestor))
			continue;
		/*
		 * Collapsed node: skip intermediate levels (same node
		 * at adjacent levels).  At the entry level, scan
		 * entries to the right and count their keys.
		 */
		if (ft_node_collapsed(ancestor)) {
			/* Skip if this is an intermediate level. */
			if (level > 0 &&
			    iter_path_node(iter)[level - 1] == ancestor)
				continue;
			/* Entry level: scan rightward entries. */
			{
				struct cds_ft_collapsed_node *col =
					ft_collapsed_node_ptr(ancestor);
				/*
				 * Use cached nr_entries from the downward
				 * walk when available; fresh load for
				 * collapsed nodes at higher levels.
				 */
				unsigned int nr_e = cached_col_nr_e ?
					cached_col_nr_e :
					ft_collapsed_nr_entries(col);
				struct cds_ft_inode_flag **cptrs =
					ft_collapsed_ptrs(col, nr_e);
				unsigned int e;

				cached_col_nr_e = 0;

				for (e = 0; e < ft_collapsed_count(nr_e); e++) {
					unsigned int slen;
					uint8_t *suffix;
					unsigned long ck;
					bool is_current;
					uint8_t data_e = ft_collapsed_load_data(col, e);

					if (ft_collapsed_entry_dead(data_e, nr_e))
						continue;
					if (!ft_node_ptr(cptrs[e]))
						continue;
					slen = ft_collapsed_suffix_len(col, data_e, e, nr_e);
					suffix = ft_collapsed_suffix(col, data_e, nr_e);

					/* Check if this is the current entry
					 * by comparing suffix to ordinal_key. */
					is_current = true;
					{
						unsigned int j;
						unsigned int cmp = slen;

						for (j = 0; j < cmp; j++) {
							if (suffix[j] != ordinal_key[level + j]) {
								is_current = false;
								break;
							}
						}
					}
					if (is_current)
						continue;

					/* Check if this entry is to the right
					 * (suffix > current key's suffix). */
					{
						unsigned int j;
						unsigned int cur_slen = depth - level;
						unsigned int cmp = slen < cur_slen ? slen : cur_slen;
						int r = 0;

						for (j = 0; j < cmp; j++) {
							if (suffix[j] > ordinal_key[level + j]) {
								r = 1;
								break;
							}
							if (suffix[j] < ordinal_key[level + j]) {
								r = -1;
								break;
							}
						}
						if (r == 0 && slen > cur_slen)
							r = 1;
						if (r <= 0)
							continue;
					}
					ck = ft_child_key_count(cptrs[e]);
					if (remaining <= ck) {
						remaining--;
						{
							unsigned int k;
							for (k = 0; k < slen; k++) {
								ordinal_key[level + k] = suffix[k];
								if (k > 0)
									iter_path_node(iter)[level + k + 1] = ancestor;
							}
						}
						level += slen;
						iter_path_node(iter)[level] = cptrs[e];
						goto descend_forward;
					}
					remaining -= ck;
				}
			}
			continue;
		}

		pivot = ordinal_key[level];
		child = ft_node_get_direction(ancestor, pivot,
				&child_key, FT_RIGHT);
		while (ft_node_ptr(child)) {
			unsigned long ck = ft_child_key_count(child);

			if (remaining <= ck) {
				remaining--;  /* enter this subtree (1-indexed within) */
				ordinal_key[level] = child_key;
				iter_path_node(iter)[level + 1] = child;
				level = level + 1;
				goto descend_forward;
			}
			remaining -= ck;
			pivot = child_key;
			child = ft_node_get_direction(ancestor, pivot,
					&child_key, FT_RIGHT);
		}
		/*
		 * No external_nodes to count going up in forward direction
		 * (they sort before children, so they're behind us).
		 */
	}

	/* Exhausted the trie. */
not_found:
	iter->node = NULL;
	iter->path_valid = true;
	iter_debug_path_update(iter);
	iter->path_len = 0;
	iter->status = CDS_FT_STATUS_NOT_FOUND;
	goto end;

descend_forward:
	/*
	 * We found the subtree containing the target.  Now descend into
	 * it using the forward lookup_nth algorithm: at each internal
	 * node, check external_nodes first, then iterate children in
	 * ascending ordinal order.
	 */
	{
		struct cds_ft_inode_flag *node_flag =
			iter_path_node(iter)[level];

		for (;;) {
			struct cds_ft_metadata *metadata;
			struct cds_ft_inode_flag *child;
			uint8_t child_key = 0;
			int pivot;

			if (ft_node_external(node_flag))
				break;

			metadata = cds_ft_item_to_metadata(
					ft_node_ptr(node_flag));

			{
				struct cds_ft_node *ext =
					ft_dereference_acquire(
						metadata->external_nodes);

				if (ext) {
				if (remaining == 0) {
					int j;

					iter->key_len = level;
					for (j = 0; j < level; j++)
						iter_key(iter)[j] = ordinal_key[j];
					iter->node = ext;
					iter->path_valid = true;
					iter_debug_path_update(iter);
					iter->path_len = level + 1;
					iter->status = CDS_FT_STATUS_OK;
					goto end;
				}
				remaining--;
			}
			} /* ext scope */

			if (ft_node_compressed(node_flag)) {
				enum ft_compressed_action act;

				act = ft_skip_forward_compressed(
					&node_flag, &level,
					ordinal_key, iter);
				if (act == FT_COMPRESSED_BREAK)
					break;
				continue;
			}
			if (ft_node_collapsed(node_flag)) {
				enum ft_compressed_action act;
				unsigned int col_nr_e = ft_collapsed_nr_entries(
					ft_collapsed_node_ptr(node_flag));

				act = ft_lookup_nth_collapsed(
					&node_flag, &level,
					ordinal_key, iter, &remaining,
					col_nr_e);
				if (act == FT_COMPRESSED_BREAK)
					break;
				continue;
			}

			pivot = -1;
			child = ft_node_get_direction(node_flag, pivot,
					&child_key, FT_RIGHT);
			while (ft_node_ptr(child)) {
				unsigned long ck = ft_child_key_count(child);

				if (remaining < ck) {
					ordinal_key[level] = child_key;
					level++;
					iter_path_node(iter)[level] = child;
					node_flag = child;
					goto next_forward_level;
				}
				remaining -= ck;
				pivot = child_key;
				child = ft_node_get_direction(node_flag, pivot,
						&child_key, FT_RIGHT);
			}
			break;

next_forward_level:
			;
		}

		/* Reached a leaf. */
		if (ft_node_ptr(iter_path_node(iter)[level]) &&
		    ft_node_external(iter_path_node(iter)[level]) &&
		    remaining == 0) {
			int j;

			iter->key_len = level;
			for (j = 0; j < level; j++)
				iter_key(iter)[j] = ordinal_key[j];
			iter->node = (struct cds_ft_node *)
				ft_node_ptr(iter_path_node(iter)[level]);
			iter->path_valid = true;
			iter_debug_path_update(iter);
			iter->path_len = level + 1;
			iter->status = CDS_FT_STATUS_OK;
			goto end;
		}
		goto not_found;
	}

end:
	iter_auto_invalidate_path(iter);
	return iter->status;
}

/*
 * Handle compressed node in cds_ft_iter_skip_reverse's descend_reverse
 * loop.
 *
 * In reverse, process the child subtree first (larger keys), then
 * fall through to external_nodes (smallest = last).
 *
 * Returns FT_COMPRESSED_CONTINUE when descending into the child,
 * FT_COMPRESSED_BREAK when the child is NULL or external (leaf),
 * or FT_COMPRESSED_END to signal the caller to fall through to
 * check_ext_descend_reverse (remaining updated, child keys exhausted).
 */
static
enum ft_compressed_action ft_skip_reverse_compressed(
		struct cds_ft_inode_flag **node_flag_p,
		int *level_p, unsigned long *remaining_p,
		uint8_t *ordinal_key, struct cds_ft_iter *iter)
{
	struct cds_ft_inode_flag *node_flag = *node_flag_p;
	int level = *level_p;
	struct cds_ft_compressed_node *cn =
		ft_compressed_node_ptr(node_flag);

	if (ft_node_ptr(cn->child)) {
		unsigned long ck = ft_child_key_count(cn->child);

		if (*remaining_p < ck) {
			int j;

			for (j = 0; j < cn->len; j++) {
				ordinal_key[level + j] = cn->key_bytes[j];
				if (j > 0)
					iter_path_node(iter)[level + j]
						= node_flag;
			}
			level += cn->len;
			node_flag = ft_dereference_acquire_prefetch(cn->child);
			if (!ft_node_ptr(node_flag)) {
				*node_flag_p = node_flag;
				*level_p = level;
				return FT_COMPRESSED_BREAK;
			}
			iter_path_node(iter)[level] = node_flag;
			if (ft_node_external(node_flag)) {
				*node_flag_p = node_flag;
				*level_p = level;
				return FT_COMPRESSED_BREAK;
			}
			*node_flag_p = node_flag;
			*level_p = level;
			return FT_COMPRESSED_CONTINUE;
		}
		*remaining_p -= ck;
	}
	return FT_COMPRESSED_END;
}

/*
 * Skip backward by @n keys from the current iterator position using
 * local traversal.
 *
 * Walks up from the current leaf, at each ancestor counting keys in
 * leftward siblings (and external_nodes, which sort before children).
 * When enough are accumulated, descends into the target subtree using
 * the reverse lookup_nth_last algorithm.
 */
enum cds_ft_status cds_ft_iter_skip_reverse(struct cds_ft *ft,
		struct cds_ft_iter *iter,
		unsigned long n)
{
	uint8_t ordinal_key[FT_MAX_KEY_LEN];
	unsigned long remaining;
	int depth, level;
	bool at_external_nodes;

	CDS_FT_ASSERT_RCU_READ_LOCKED(ft);

	if (!iter->node)
		return CDS_FT_STATUS_NOT_FOUND;
	if (n == 0)
		return CDS_FT_STATUS_OK;

	iter_debug_path_snapshot(iter);

	/* Rebuild path from root to current key. */
	depth = ft_rebuild_path(ft, iter, iter_key(iter), iter->key_len,
			ordinal_key);
	if (depth < 0)
		goto not_found;

	remaining = n;

	/*
	 * Determine whether the current key sits at an internal or
	 * compressed node's external_nodes or at a leaf child.
	 */
	at_external_nodes = !ft_node_external(iter_path_node(iter)[depth]);
	(void) at_external_nodes;

	/*
	 * Whether at external_nodes or at a leaf child, we start the
	 * upward walk from the same level.  (At external_nodes all
	 * children are to the right, so there's nothing to the left.)
	 */
	level = depth;

	/*
	 * Walk up: at each ancestor, count keys in leftward siblings
	 * of the child we came from, plus external_nodes at the ancestor
	 * (which sort before all children).
	 */
	for (level--; level >= 0; level--) {
		struct cds_ft_inode_flag *ancestor = iter_path_node(iter)[level];
		struct cds_ft_inode_flag *child;
		struct cds_ft_metadata *ameta;
		uint8_t child_key = 0;
		unsigned long left_keys = 0;
		int pivot;

		if (ft_node_external(ancestor))
			continue;
		/*
		 * Skip intermediate compressed path levels (same
		 * compressed node at adjacent levels).  At the entry
		 * level, only external_nodes matter (no siblings).
		 */
		if (ft_node_compressed(ancestor)) {
			if (level > 0 &&
			    iter_path_node(iter)[level - 1] == ancestor)
				continue;
			ameta = cds_ft_item_to_metadata(
					ft_node_ptr(ancestor));
			{
				struct cds_ft_node *a_ext =
					ft_dereference_acquire(
						ameta->external_nodes);

				if (a_ext) {
					if (remaining == 1) {
						int j;

						iter->key_len = level;
						for (j = 0; j < level; j++)
							iter_key(iter)[j] = ordinal_key[j];
						iter->node = a_ext;
						iter->path_valid = true;
						iter_debug_path_update(iter);
						iter->path_len = level + 1;
						iter->status =
							CDS_FT_STATUS_OK;
						goto end;
					}
					remaining--;
				}
			}
			continue;
		}
		if (ft_node_collapsed(ancestor)) {
			/* Skip intermediate levels. */
			if (level > 0 &&
			    iter_path_node(iter)[level - 1] == ancestor)
				continue;
			/* Entry level: count leftward entries. */
			{
				struct cds_ft_collapsed_node *col =
					ft_collapsed_node_ptr(ancestor);
				unsigned int nr_e = ft_collapsed_nr_entries(col);
				struct cds_ft_inode_flag **cptrs =
					ft_collapsed_ptrs(col, nr_e);
				struct cds_ft_metadata *col_meta =
					cds_ft_item_to_metadata(
						(struct cds_ft_inode *) col);
				unsigned int e;

				for (e = 0; e < ft_collapsed_count(nr_e); e++) {
					unsigned int slen;
					uint8_t *suffix;
					unsigned long ck;
					uint8_t data_e = ft_collapsed_load_data(col, e);

					if (ft_collapsed_entry_dead(data_e, nr_e))
						continue;
					if (!ft_node_ptr(cptrs[e]))
						continue;
					slen = ft_collapsed_suffix_len(col, data_e, e, nr_e);
					suffix = ft_collapsed_suffix(col, data_e, nr_e);

					/* Check if suffix < current key. */
					{
						unsigned int j;
						unsigned int cur_slen = depth - level;
						unsigned int cmp = slen < cur_slen ? slen : cur_slen;
						int r = 0;

						for (j = 0; j < cmp; j++) {
							if (suffix[j] < ordinal_key[level + j]) {
								r = -1;
								break;
							}
							if (suffix[j] > ordinal_key[level + j]) {
								r = 1;
								break;
							}
						}
						if (r == 0 && slen < cur_slen)
							r = -1;
						if (r >= 0)
							continue;
					}
					ck = ft_child_key_count(cptrs[e]);
					if (remaining <= ck) {
						remaining--;
						{
							unsigned int k;
							for (k = 0; k < slen; k++) {
								ordinal_key[level + k] = suffix[k];
								if (k > 0)
									iter_path_node(iter)[level + k + 1] = ancestor;
							}
						}
						level += slen;
						iter_path_node(iter)[level] = cptrs[e];
						goto descend_reverse;
					}
					remaining -= ck;
				}
				/* Then check external_nodes (smallest key). */
				{
					struct cds_ft_node *a_ext =
						ft_dereference_acquire(
							col_meta->external_nodes);
					if (a_ext) {
						if (remaining == 1) {
							int j;

							iter->key_len = level;
							for (j = 0; j < level; j++)
								iter_key(iter)[j] = ordinal_key[j];
							iter->node = a_ext;
							iter->path_valid = true;
							iter_debug_path_update(iter);
							iter->path_len = level + 1;
							iter->status = CDS_FT_STATUS_OK;
							goto end;
						}
						remaining--;
					}
				}
			}
			continue;
		}

		ameta = cds_ft_item_to_metadata(ft_node_ptr(ancestor));

		/* Count leftward siblings. */
		pivot = ordinal_key[level];
		child = ft_node_get_direction(ancestor, pivot,
				&child_key, FT_LEFT);
		while (ft_node_ptr(child)) {
			left_keys += ft_child_key_count(child);
			pivot = child_key;
			child = ft_node_get_direction(ancestor, pivot,
					&child_key, FT_LEFT);
		}

		/* External_nodes at ancestor sort before all children. */
		{
			struct cds_ft_node *a_ext =
				ft_dereference_acquire(ameta->external_nodes);

			if (a_ext)
				left_keys++;

			if (remaining <= left_keys) {
				/*
				 * Target is among the leftward siblings or
				 * external_nodes.  Descend in reverse order:
				 * iterate leftward siblings from the current
				 * child in descending ordinal order, then
				 * check external_nodes last.
				 */
				pivot = ordinal_key[level];
				child = ft_node_get_direction(ancestor, pivot,
						&child_key, FT_LEFT);
				while (ft_node_ptr(child)) {
					unsigned long ck =
						ft_child_key_count(child);

					if (remaining <= ck) {
						remaining--;
						ordinal_key[level] = child_key;
						iter_path_node(iter)[level + 1]
							= child;
						level = level + 1;
						goto descend_reverse;
					}
					remaining -= ck;
					pivot = child_key;
					child = ft_node_get_direction(
							ancestor, pivot,
							&child_key, FT_LEFT);
				}

				/* Must be the external_nodes. */
				if (a_ext && remaining == 1) {
					int j;

					iter->key_len = level;
					for (j = 0; j < level; j++)
						iter_key(iter)[j] = ordinal_key[j];
					iter->node = a_ext;
				iter->path_valid = true;
				iter_debug_path_update(iter);
				iter->path_len = level + 1;
				iter->status = CDS_FT_STATUS_OK;
				goto end;
			}
			/* Shouldn't happen if left_keys was correct. */
			goto not_found;
		}
		remaining -= left_keys;
		}
	}

	/* Exhausted the trie. */
not_found:
	iter->node = NULL;
	iter->path_valid = true;
	iter_debug_path_update(iter);
	iter->path_len = 0;
	iter->status = CDS_FT_STATUS_NOT_FOUND;
	goto end;

descend_reverse:
	/*
	 * Descend into the target subtree using the reverse algorithm:
	 * at each internal node, iterate children in descending ordinal
	 * order first, then check external_nodes last.
	 */
	{
		struct cds_ft_inode_flag *node_flag =
			iter_path_node(iter)[level];

		for (;;) {
			struct cds_ft_metadata *metadata;
			struct cds_ft_inode_flag *child;
			uint8_t child_key = 0;
			int pivot;

			if (ft_node_external(node_flag))
				break;

			metadata = cds_ft_item_to_metadata(
					ft_node_ptr(node_flag));

			if (ft_node_compressed(node_flag)) {
				enum ft_compressed_action act;

				act = ft_skip_reverse_compressed(
					&node_flag, &level, &remaining,
					ordinal_key, iter);
				if (act == FT_COMPRESSED_BREAK)
					break;
				if (act == FT_COMPRESSED_END)
					goto check_ext_descend_reverse;
				continue;
			}
			if (ft_node_collapsed(node_flag)) {
				enum ft_compressed_action act;
				unsigned int col_nr_e = ft_collapsed_nr_entries(
					ft_collapsed_node_ptr(node_flag));

				act = ft_lookup_nth_last_collapsed(
					&node_flag, &level,
					ordinal_key, iter, &remaining,
					col_nr_e);
				if (act == FT_COMPRESSED_BREAK)
					break;
				if (act == FT_COMPRESSED_END)
					goto check_ext_descend_reverse;
				continue;
			}

			pivot = FT_ENTRY_PER_NODE;
			child = ft_node_get_direction(node_flag, pivot,
					&child_key, FT_LEFT);
			while (ft_node_ptr(child)) {
				unsigned long ck = ft_child_key_count(child);

				if (remaining < ck) {
					ordinal_key[level] = child_key;
					level++;
					iter_path_node(iter)[level] = child;
					node_flag = child;
					goto next_reverse_level;
				}
				remaining -= ck;
				pivot = child_key;
				child = ft_node_get_direction(node_flag, pivot,
						&child_key, FT_LEFT);
			}

check_ext_descend_reverse:
			{
				struct cds_ft_node *ext =
					ft_dereference_acquire(
						metadata->external_nodes);

				if (ext && remaining == 0) {
				int j;

				iter->key_len = level;
				for (j = 0; j < level; j++)
					iter_key(iter)[j] = ordinal_key[j];
				iter->node = ext;
				iter->path_valid = true;
				iter_debug_path_update(iter);
				iter->path_len = level + 1;
				iter->status = CDS_FT_STATUS_OK;
				goto end;
			}
			}
			break;

next_reverse_level:
			;
		}

		/* Reached a leaf. */
		if (ft_node_ptr(iter_path_node(iter)[level]) &&
		    ft_node_external(iter_path_node(iter)[level]) &&
		    remaining == 0) {
			int j;

			iter->key_len = level;
			for (j = 0; j < level; j++)
				iter_key(iter)[j] = ordinal_key[j];
			iter->node = (struct cds_ft_node *)
				ft_node_ptr(iter_path_node(iter)[level]);
			iter->path_valid = true;
			iter_debug_path_update(iter);
			iter->path_len = level + 1;
			iter->status = CDS_FT_STATUS_OK;
			goto end;
		}
		goto not_found;
	}

end:
	iter_auto_invalidate_path(iter);
	return iter->status;
}

unsigned long cds_ft_count_entries(struct cds_ft *ft)
{
	struct cds_ft_iter *iter;
	enum cds_ft_status status;
	unsigned long count = 0;

	CDS_FT_ASSERT_RCU_READ_LOCKED(ft);

	status = cds_ft_iter_create(ft, &iter);
	if (status != CDS_FT_STATUS_OK)
		return 0;
	cds_ft_for_each_rcu(ft, iter) {
		struct cds_ft_node *node = cds_ft_iter_node(iter);

		cds_ft_for_each_duplicate_rcu(node)
			count++;
	}
	if (cds_ft_iter_status(iter) < 0)
		count = 0;
	cds_ft_iter_destroy(iter);
	return count;
}

enum cds_ft_status cds_ft_attr_create(struct cds_ft_attr **result)
{
	struct cds_ft_attr *attr = calloc(1, sizeof(struct cds_ft_attr));

	if (!attr) {
		*result = NULL;
		return CDS_FT_STATUS_MEMORY_ERROR;
	}
	attr->key_len = CDS_FT_LEN_DEFAULT;
	attr->max_key_len = FT_MAX_KEY_LEN;
	attr->key_map.identity = true;
	*result = attr;
	return CDS_FT_STATUS_OK;
}

void cds_ft_attr_destroy(struct cds_ft_attr *attr)
{
	free(attr);
}

enum cds_ft_status cds_ft_attr_set_key_len(struct cds_ft_attr *attr, size_t key_len)
{
	attr->key_len = key_len;
	return CDS_FT_STATUS_OK;
}

enum cds_ft_status cds_ft_attr_set_max_key_len(struct cds_ft_attr *attr, size_t max_key_len)
{
	if (max_key_len == CDS_FT_MAX_LEN_UNLIMITED) {
		attr->max_key_len = FT_MAX_KEY_LEN;
		return CDS_FT_STATUS_OK;
	}
	if (max_key_len > FT_MAX_KEY_LEN)
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	attr->max_key_len = max_key_len;
	return CDS_FT_STATUS_OK;
}

enum cds_ft_status cds_ft_attr_set_key_map(struct cds_ft_attr *attr,
		const uint8_t *key_to_ordinal, const uint8_t *ordinal_to_key)
{
	attr->key_map.identity = false;
	memcpy(attr->key_map.key_to_ordinal, key_to_ordinal, sizeof(attr->key_map.key_to_ordinal));
	memcpy(attr->key_map.ordinal_to_key, ordinal_to_key, sizeof(attr->key_map.ordinal_to_key));
	return CDS_FT_STATUS_OK;
}

enum cds_ft_status cds_ft_attr_set_flags(struct cds_ft_attr *attr,
		unsigned int flags)
{
	attr->flags = flags;
	return CDS_FT_STATUS_OK;
}

enum cds_ft_status _cds_ft_group_create(const struct cds_ft_attr *attr,
		struct cds_ft_group **result_ft_group,
		const struct rcu_flavor_struct *flavor)
{
	struct cds_ft_group *ft_group;
	size_t key_len = CDS_FT_LEN_DEFAULT,
	       max_key_len = FT_MAX_KEY_LEN;

	if (attr) {
		key_len = attr->key_len;
		max_key_len = attr->max_key_len;
	}
	/* max_tree_depth 0 is for pointer to root node */
	if (key_len != CDS_FT_LEN_VARIABLE && key_len > max_key_len) {
		*result_ft_group = NULL;
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}
	ft_group = calloc(1, sizeof(*ft_group));
	if (!ft_group) {
		*result_ft_group = NULL;
		return CDS_FT_STATUS_MEMORY_ERROR;
	}
	ft_group->key_len = key_len;
	ft_group->max_key_len = max_key_len;
	ft_group->max_tree_depth = max_key_len + 1;
	assert(ft_group->max_tree_depth <= FT_MAX_DEPTH);
	ft_group->flavor = flavor;
	if (attr) {
		ft_group->key_map = attr->key_map;
		ft_group->flags = attr->flags;
	} else {
		ft_group->key_map.identity = true;
	}
	*result_ft_group = ft_group;
	return CDS_FT_STATUS_OK;
}

enum cds_ft_status cds_ft_group_destroy(struct cds_ft_group *ft_group)
{
	if (uatomic_load(&ft_group->nr_ft_instances, CMM_RELAXED) != 0)
		return CDS_FT_STATUS_BUSY_ERROR;
	cds_ft_free_all_arenas(ft_group);
	free(ft_group);
	return CDS_FT_STATUS_OK;
}

enum cds_ft_status cds_ft_create(struct cds_ft_group *ft_group,
		struct cds_ft **result_ft)
{
	struct cds_ft *ft;
	struct cds_ft_inode *root_node;
	struct cds_ft_metadata *metadata;
	const struct cds_ft_type *type0 = &ft_types[0];

	ft = calloc(1, sizeof(*ft));
	if (!ft) {
		*result_ft = NULL;
		return CDS_FT_STATUS_MEMORY_ERROR;
	}
	ft->group = ft_group;

	/*
	 * Allocate the root node (smallest linear type, initially empty).
	 *
	 * ft->root always points to an internal node, even when the
	 * trie has no entries (nr_child == 0).  This is the one place
	 * where a node with 0 children is allowed; all other internal
	 * nodes are pruned when their last child is removed.  The node
	 * itself may be replaced by graft or graft-swap, but the
	 * invariant on the slot is maintained across all operations.
	 *
	 * The root is a regular internal node whose metadata
	 * (nr_child, external_nodes) is accessed the same way as any
	 * other node's.  Its metadata carries the NIL-key entries, so
	 * transplanting a root node between tries is a single pointer
	 * swap with no metadata relocation.
	 */
	root_node = alloc_cds_ft_node(ft, type0, &metadata);
	if (!root_node) {
		free(ft);
		*result_ft = NULL;
		return CDS_FT_STATUS_MEMORY_ERROR;
	}
	ft->root = ft_node_flag(root_node, 0);

	uatomic_inc(&ft_group->nr_ft_instances, CMM_RELAXED);
	*result_ft = ft;
	return CDS_FT_STATUS_OK;
}

static
void print_debug_fallback_distribution(struct cds_ft *ft)
{
	int i;

	fprintf(stderr, "Fallback node distribution:\n");
	for (i = 0; i < FT_ENTRY_PER_NODE; i++) {
		if (!ft->node_fallback_count_distribution[i])
			continue;
		fprintf(stderr, "	%3u: %4lu\n",
			i, ft->node_fallback_count_distribution[i]);
	}
}

static
void ft_final_checks(struct cds_ft *ft)
{
	double fallback_ratio;
	unsigned long na, nf, nr_fallback;

	if (!ft_debug_counters())
		return;

	fallback_ratio = (double) uatomic_read(&ft->nr_fallback);
	fallback_ratio /= (double) uatomic_read(&ft->nr_nodes_allocated);
	nr_fallback = uatomic_read(&ft->nr_fallback);
	if (nr_fallback)
		fprintf(stderr,
			"[warning] RCU Fractal Trie used %lu fallback node(s) (ratio: %g)\n",
			uatomic_read(&ft->nr_fallback),
			fallback_ratio);

	na = uatomic_read(&ft->nr_nodes_allocated);
	nf = uatomic_read(&ft->nr_nodes_freed);
	dbg_printf("Nodes allocated: %lu, Nodes freed: %lu.\n", na, nf);
	if (nr_fallback)
		print_debug_fallback_distribution(ft);

	if (na != nf) {
		fprintf(stderr, "[error] Fractal Trie leaked %ld nodes. Allocated: %lu, freed: %lu.\n",
			(long) na - nf, na, nf);
		abort();
	}
}

/*
 * There should be no more concurrent add, delete, nor look-up performed
 * on the Fractal Trie while it is being destroyed (ensured by the
 * caller).
 */
void cds_ft_destroy(struct cds_ft *ft)
{
	const struct rcu_flavor_struct *flavor = ft->group->flavor;

	/* Free root node. No concurrent readers at this point. */
	free_cds_ft_node(ft, ft_node_ptr(ft->root));
	/* Wait for in-flight call_rcu free to complete. */
	flavor->barrier();
	ft_final_checks(ft);
	uatomic_dec(&ft->group->nr_ft_instances, CMM_RELAXED);
	free(ft);
}

static
void print_indent(FILE *out, int level)
{
	int i;

	for (i = 0; i < level; i++)
		fprintf(out, "	");
}

static void show_node_recursive(const struct cds_ft *ft, FILE *out,
		struct cds_ft_inode_flag *node_flag, int level);

static void print_density(FILE *out, const struct cds_ft_metadata *m)
{
	int i;

	fprintf(out, "[");
	for (i = 0; i < FT_NODE_DENSITY_DEPTH; i++) {
		if (i) fprintf(out, " ");
		fprintf(out, "%lu", ft_density_get(m, i));
	}
	fprintf(out, "]");
}

static void show_collapsed_node(const struct cds_ft *ft, FILE *out,
		struct cds_ft_inode_flag *node_flag, int level)
{
	struct cds_ft_collapsed_node *col = ft_collapsed_node_ptr(node_flag);
	struct cds_ft_metadata *col_meta =
		cds_ft_item_to_metadata((struct cds_ft_inode *) col);
	unsigned int nr_e = ft_collapsed_nr_entries(col);
	struct cds_ft_inode_flag **ptrs = ft_collapsed_ptrs(col, nr_e);
	struct cds_ft_node *external_nodes = rcu_dereference(col_meta->external_nodes);
	unsigned int e;

	print_indent(out, level);
	fprintf(out, "Level %d, COLLAPSED node: %p, nr_entries: %u, scan=%uB, order=%zu, nr_keys: %lu, density: ",
		level, node_flag, ft_collapsed_count(nr_e),
		ft_collapsed_scan_zone_size(nr_e),
		cds_ft_item_order(col),
		uatomic_load(&col_meta->nr_keys, CMM_RELAXED));
	print_density(out, col_meta);
	fprintf(out, "\n");
	if (external_nodes) {
		print_indent(out, level);
		fprintf(out, "Level %d, (meta)external node list ptr: %p\n",
			level, external_nodes);
	}

	for (e = 0; e < ft_collapsed_count(nr_e); e++) {
		uint8_t data_e = ft_collapsed_load_data(col, e);
		unsigned int slen = ft_collapsed_suffix_len(col, data_e, e, nr_e);
		uint8_t *suffix = ft_collapsed_suffix(col, data_e, nr_e);
		struct cds_ft_inode_flag *child;
		unsigned int k;

		if (ft_collapsed_entry_dead(data_e, nr_e))
			continue;
		child = ft_dereference_acquire(ptrs[e]);
		print_indent(out, level);
		fprintf(out, "  entry[%u]: suffix=[", e);
		for (k = 0; k < slen; k++)
			fprintf(out, "%s%u", k ? "," : "", suffix[k]);
		fprintf(out, "] (len=%u), child=%p", slen, child);
		if (ft_node_ptr(child)) {
			if (ft_node_external(child))
				fprintf(out, " (external)");
			else if (ft_node_internal(child))
				fprintf(out, " (internal)");
			else if (ft_node_compressed(child))
				fprintf(out, " (compressed)");
			else if (ft_node_collapsed(child))
				fprintf(out, " (collapsed)");
		}
		fprintf(out, "\n");
		if (ft_node_ptr(child) && !ft_node_external(child))
			show_node_recursive(ft, out, child, level + slen);
	}
}

void show_node_recursive(const struct cds_ft *ft, FILE *out, struct cds_ft_inode_flag *node_flag, int level)
{
	unsigned int key;

	if (ft_node_collapsed(node_flag)) {
		show_collapsed_node(ft, out, node_flag, level);
		return;
	}

	print_indent(out, level);
	fprintf(out, "Level %d within node %p\n", level, node_flag);
	for (key = 0; key < 256; key++) {
		struct cds_ft_inode_flag *child_node_flag;

		child_node_flag = ft_node_get_nth(node_flag, NULL, (uint8_t) key);
		if (!ft_node_ptr(child_node_flag))
			continue;
		if (ft_node_collapsed(child_node_flag)) {
			print_indent(out, level);
			fprintf(out, "Level %d, key value: %u ->\n", level, key);
			show_collapsed_node(ft, out, child_node_flag, level + 1);
		} else if (ft_node_internal(child_node_flag)) {
			struct cds_ft_metadata *metadata = cds_ft_item_to_metadata(ft_node_ptr(child_node_flag));
			struct cds_ft_node *external_nodes = rcu_dereference(metadata->external_nodes);

			print_indent(out, level);
			fprintf(out, "Level %d, key value: %u, internal node: %p, nr_children: %u, density: ",
				level, key, child_node_flag, metadata->nr_child);
			print_density(out, metadata);
			fprintf(out, "\n");
			if (external_nodes) {
				print_indent(out, level);
				fprintf(out, "Level %d, key value: %u, (meta)external node list ptr: %p\n",
					level, key, external_nodes);
			}
			show_node_recursive(ft, out, child_node_flag, level + 1);
		} else if (ft_node_compressed(child_node_flag)) {
			struct cds_ft_compressed_node *cn =
				ft_compressed_node_ptr(child_node_flag);
			struct cds_ft_metadata *metadata =
				cds_ft_item_to_metadata((struct cds_ft_inode *) cn);
			struct cds_ft_node *external_nodes = rcu_dereference(metadata->external_nodes);

			print_indent(out, level);
			fprintf(out, "Level %d, key value: %u, compressed node: %p, path_len: %u, nr_keys: %lu, density: ",
				level, key, child_node_flag, (unsigned int) cn->len,
				uatomic_load(&metadata->nr_keys, CMM_RELAXED));
			print_density(out, metadata);
			fprintf(out, "\n");
			if (external_nodes) {
				print_indent(out, level);
				fprintf(out, "Level %d, key value: %u, (meta)external node list ptr: %p\n",
					level, key, external_nodes);
			}
			if (ft_node_ptr(cn->child) &&
			    !ft_node_external(cn->child))
				show_node_recursive(ft, out, cn->child, level + cn->len);
			else if (ft_node_ptr(cn->child)) {
				print_indent(out, level + cn->len);
				fprintf(out, "Level %d, compressed child: external node list ptr: %p\n",
					level + (int) cn->len, ft_node_ptr(cn->child));
			}
		} else {
			print_indent(out, level);
			fprintf(out, "Level %d, key value: %u, external node list ptr: %p\n",
				level, key, ft_node_ptr(child_node_flag));
		}
	}
}

void cds_ft_show(const struct cds_ft *ft, FILE *out)
{
	struct cds_ft_inode_flag *node_flag;
	int level = 0;

	fprintf(out, "Show Fractal Trie %p\n", ft);
	fprintf(out, "---------------------------------------------------\n");

	node_flag = rcu_dereference(ft->root);

	/* Root is always present and always internal. */
	{
		struct cds_ft_metadata *rm = cds_ft_item_to_metadata(ft_node_ptr(node_flag));

		print_indent(out, level);
		fprintf(out, "Level 0: root node %p, density: ", node_flag);
		print_density(out, rm);
		fprintf(out, "\n");
	}
	show_node_recursive(ft, out, node_flag, level + 1);
	fprintf(out, "---------------------------------------------------\n");
}

struct cds_ft_node_stats {
	uint64_t count;
	uint64_t distribution[257];
};

struct cds_ft_stats_level {
	uint64_t nr_external_nodes;
	uint64_t nr_metadata_external_nodes;
	uint64_t nr_duplicate_external_nodes;
	uint64_t nr_internal_nodes;
	uint64_t nr_compressed_nodes;
	uint64_t nr_collapsed_nodes;
	uint64_t nr_collapsed_scan32;
	uint64_t nr_collapsed_scan64;
	uint64_t nr_collapsed_scan128;
	uint64_t nr_collapsed_scan256;
	struct cds_ft_node_stats node_stats[FT_TYPE_MAX_NR];
	bool has_nodes;
};

struct cds_ft_stats {
	struct cds_ft_stats_level level[FT_MAX_DEPTH];
};

enum cds_ft_status cds_ft_recompute_stats(struct cds_ft *ft)
{
	struct cds_ft_iter *iter;
	enum cds_ft_status status;
	size_t max_len = 0;

	status = cds_ft_iter_create(ft, &iter);
	if (status != CDS_FT_STATUS_OK)
		return status;
	cds_ft_for_each_rcu(ft, iter) {
		if (iter->key_len > max_len)
			max_len = iter->key_len;
	}
	status = cds_ft_iter_status(iter);
	cds_ft_iter_destroy(iter);
	if (status < 0)
		return status;
	uatomic_store(&ft->max_used_key_len, max_len, CMM_RELAXED);
	return CDS_FT_STATUS_OK;
}

static
void calc_stats_node(const struct cds_ft *ft __attribute__((unused)),
		struct cds_ft_inode_flag *node_flag, struct cds_ft_stats *stats, int level)
{
	unsigned long node_type = ft_node_type(node_flag);
	struct cds_ft_node_stats *node_stats = &stats->level[level].node_stats[node_type];
	const struct cds_ft_metadata *metadata;

	metadata = cds_ft_item_to_metadata(ft_node_ptr(node_flag));
	node_stats->count++;
	node_stats->distribution[metadata->nr_child]++;
	stats->level[level].nr_internal_nodes++;
	stats->level[level].has_nodes = true;
}

static
void calc_stats_node_recursive(const struct cds_ft *ft, struct cds_ft_inode_flag *node_flag,
		struct cds_ft_stats *stats, int level);

static
void calc_stats_collapsed(const struct cds_ft *ft,
		struct cds_ft_inode_flag *node_flag,
		struct cds_ft_stats *stats, int level)
{
	struct cds_ft_collapsed_node *col = ft_collapsed_node_ptr(node_flag);
	struct cds_ft_metadata *col_meta =
		cds_ft_item_to_metadata((struct cds_ft_inode *) col);
	unsigned int nr_e = ft_collapsed_nr_entries(col);
	struct cds_ft_inode_flag **cptrs = ft_collapsed_ptrs(col, nr_e);
	struct cds_ft_node *external_nodes = rcu_dereference(col_meta->external_nodes);
	unsigned int e;

	stats->level[level].nr_internal_nodes++;
	stats->level[level].nr_collapsed_nodes++;
	switch (col->nr_entries >> FT_COLLAPSED_SCAN_SHIFT) {
	case FT_COLLAPSED_SCAN_32:
		stats->level[level].nr_collapsed_scan32++;
		break;
	case FT_COLLAPSED_SCAN_64:
		stats->level[level].nr_collapsed_scan64++;
		break;
	case FT_COLLAPSED_SCAN_128:
		stats->level[level].nr_collapsed_scan128++;
		break;
	case FT_COLLAPSED_SCAN_256:
		stats->level[level].nr_collapsed_scan256++;
		break;
	}
	stats->level[level].has_nodes = true;
	if (external_nodes) {
		struct cds_ft_node *iter_node;
		unsigned int count = 0;

		iter_node = external_nodes;
		cds_ft_for_each_duplicate(iter_node) {
			if (count++ == 0)
				stats->level[level].nr_metadata_external_nodes++;
			else
				stats->level[level].nr_duplicate_external_nodes++;
			stats->level[level].has_nodes = true;
		}
	}
	for (e = 0; e < ft_collapsed_count(nr_e); e++) {
		uint8_t data_e = ft_collapsed_load_data(col, e);
		unsigned int slen, j;
		struct cds_ft_inode_flag *child;

		if (ft_collapsed_entry_dead(data_e, nr_e))
			continue;
		slen = ft_collapsed_suffix_len(col, data_e, e, nr_e);
		child = ft_dereference_acquire(cptrs[e]);
		for (j = 1; j < slen; j++) {
			stats->level[level + j].nr_internal_nodes++;
			stats->level[level + j].nr_compressed_nodes++;
			stats->level[level + j].has_nodes = true;
		}
		if (ft_node_ptr(child) && !ft_node_external(child))
			calc_stats_node_recursive(ft, child,
				stats, level + slen);
		else if (ft_node_ptr(child)) {
			struct cds_ft_node *iter_node;
			unsigned int count = 0;

			iter_node = (struct cds_ft_node *) ft_node_ptr(child);
			cds_ft_for_each_duplicate(iter_node) {
				if (count++ == 0)
					stats->level[level + slen].nr_external_nodes++;
				else
					stats->level[level + slen].nr_duplicate_external_nodes++;
				stats->level[level + slen].has_nodes = true;
			}
		}
	}
}

static
void calc_stats_node_recursive(const struct cds_ft *ft, struct cds_ft_inode_flag *node_flag,
		struct cds_ft_stats *stats, int level)
{
	unsigned int key;

	if (ft_node_collapsed(node_flag)) {
		calc_stats_collapsed(ft, node_flag, stats, level);
		return;
	}

	for (key = 0; key < 256; key++) {
		struct cds_ft_inode_flag *child_node_flag;

		child_node_flag = ft_node_get_nth(node_flag, NULL, (uint8_t) key);
		if (!ft_node_ptr(child_node_flag))
			continue;
		if (ft_node_collapsed(child_node_flag)) {
			calc_stats_collapsed(ft, child_node_flag,
				stats, level);
			continue;
		}
		if (ft_node_internal(child_node_flag)) {
			struct cds_ft_metadata *metadata = cds_ft_item_to_metadata(ft_node_ptr(child_node_flag));
			struct cds_ft_node *external_nodes = rcu_dereference(metadata->external_nodes);

			calc_stats_node(ft, child_node_flag, stats, level);
			if (external_nodes) {
				struct cds_ft_node *iter_node;
				unsigned int count = 0;

				iter_node = external_nodes;
				cds_ft_for_each_duplicate(iter_node) {
					if (count++ == 0)
						stats->level[level].nr_metadata_external_nodes++;
					else
						stats->level[level].nr_duplicate_external_nodes++;
					stats->level[level].has_nodes = true;
				}
			}
			calc_stats_node_recursive(ft, child_node_flag, stats, level + 1);
		} else if (ft_node_compressed(child_node_flag)) {
			struct cds_ft_compressed_node *cn =
				ft_compressed_node_ptr(child_node_flag);
			struct cds_ft_metadata *metadata =
				cds_ft_item_to_metadata((struct cds_ft_inode *) cn);
			struct cds_ft_node *external_nodes = rcu_dereference(metadata->external_nodes);
			int j;

			stats->level[level].nr_internal_nodes++;
			stats->level[level].nr_compressed_nodes++;
			stats->level[level].has_nodes = true;
			if (external_nodes) {
				struct cds_ft_node *iter_node;
				unsigned int count = 0;

				iter_node = external_nodes;
				cds_ft_for_each_duplicate(iter_node) {
					if (count++ == 0)
						stats->level[level].nr_metadata_external_nodes++;
					else
						stats->level[level].nr_duplicate_external_nodes++;
					stats->level[level].has_nodes = true;
				}
			}
			for (j = 1; j < cn->len; j++) {
				stats->level[level + j].nr_internal_nodes++;
				stats->level[level + j].nr_compressed_nodes++;
				stats->level[level + j].has_nodes = true;
			}
			if (ft_node_ptr(cn->child) &&
			    !ft_node_external(cn->child))
				calc_stats_node_recursive(ft, cn->child, stats, level + cn->len);
			else if (ft_node_ptr(cn->child)) {
				struct cds_ft_node *iter_node;
				unsigned int count = 0;

				iter_node = (struct cds_ft_node *) ft_node_ptr(cn->child);
				cds_ft_for_each_duplicate(iter_node) {
					if (count++ == 0)
						stats->level[level + cn->len].nr_external_nodes++;
					else
						stats->level[level + cn->len].nr_duplicate_external_nodes++;
					stats->level[level + cn->len].has_nodes = true;
				}
			}
		} else {
			struct cds_ft_node *iter_node;
			unsigned int count = 0;

			iter_node = (struct cds_ft_node *) ft_node_ptr(child_node_flag);
			cds_ft_for_each_duplicate(iter_node) {
				if (count++ == 0)
					stats->level[level].nr_external_nodes++;
				else
					stats->level[level].nr_duplicate_external_nodes++;
				stats->level[level].has_nodes = true;
			}
		}
	}
}

static
void do_show_stats(const struct cds_ft *ft, FILE *out, const struct cds_ft_stats *stats)
{
	int level;

	fprintf(out, "Fractal Trie (%p) Statistics\n", ft);
	fprintf(out, "---------------------------------------------------\n");
	for (level = 0; level < FT_MAX_DEPTH; level++) {
		const struct cds_ft_stats_level *stats_level = &stats->level[level];
		unsigned long type;

		if (!stats_level->has_nodes)
			break;
		fprintf(out, "Level: %d\n", level);
		if (stats_level->nr_external_nodes) {
			print_indent(out, 1);
			fprintf(out, "External nodes: %" PRIu64 "\n", stats_level->nr_external_nodes);
		}
		if (stats_level->nr_metadata_external_nodes) {
			print_indent(out, 1);
			fprintf(out, "Metadata external nodes: %" PRIu64 "\n", stats_level->nr_metadata_external_nodes);
		}
		if (stats_level->nr_duplicate_external_nodes) {
			print_indent(out, 1);
			fprintf(out, "Duplicate external nodes: %" PRIu64 "\n", stats_level->nr_duplicate_external_nodes);
		}
		if (stats_level->nr_internal_nodes) {
			print_indent(out, 1);
			fprintf(out, "Internal nodes: %" PRIu64 "\n", stats_level->nr_internal_nodes);
		}
		if (stats_level->nr_compressed_nodes) {
			print_indent(out, 1);
			fprintf(out, "Compressed nodes: %" PRIu64 "\n", stats_level->nr_compressed_nodes);
		}
		if (stats_level->nr_collapsed_nodes) {
			print_indent(out, 1);
			fprintf(out, "Collapsed nodes: %" PRIu64
				" (32B: %" PRIu64 ", 64B: %" PRIu64
				", 128B: %" PRIu64 ", 256B: %" PRIu64 ")\n",
				stats_level->nr_collapsed_nodes,
				stats_level->nr_collapsed_scan32,
				stats_level->nr_collapsed_scan64,
				stats_level->nr_collapsed_scan128,
				stats_level->nr_collapsed_scan256);
		}
		for (type = 0; type < FT_TYPE_MAX_NR; type++) {
			const struct cds_ft_node_stats *node_stats = &stats->level[level].node_stats[type];
			uint64_t nr_nodes = node_stats->count;

			if (nr_nodes) {
				unsigned int i;
				bool first = true;

				print_indent(out, 2);
				fprintf(out, "Internal node type %lu: %" PRIu64 " (", type, nr_nodes);
				for (i = 0; i <= 256; i++) {
					if (node_stats->distribution[i]) {
						fprintf(out, "%s%u: %" PRIu64,
							(!first ? ", " : ""), i, node_stats->distribution[i]);
						first = false;
					}
				}
				fprintf(out, ")\n");
			}
		}
	}
	fprintf(out, "---------------------------------------------------\n");
}

void cds_ft_show_stats(const struct cds_ft *ft, FILE *out)
{
	struct cds_ft_inode_flag *node_flag;
	struct cds_ft_stats stats = {};
	int level = 0;

	node_flag = rcu_dereference(ft->root);

	/* Root is always present and always internal. */
	calc_stats_node(ft, node_flag, &stats, level);
	calc_stats_node_recursive(ft, node_flag, &stats, level + 1);
	do_show_stats(ft, out, &stats);
}

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

	default:
		return "Unknown status value";
	}
}

enum cds_ft_status cds_ft_iter_create(struct cds_ft *ft, struct cds_ft_iter **result_iter)
{
	size_t max_depth = ft->group->max_tree_depth;
	size_t max_key_len = ft->group->max_key_len;
	size_t path_size = max_depth * sizeof(struct cds_ft_inode_flag *);
	size_t key_size  = max_key_len * sizeof(uint8_t);
	struct cds_ft_iter *iter = calloc(1, sizeof(*iter) + path_size + key_size);

	if (!iter) {
		*result_iter = NULL;
		return CDS_FT_STATUS_MEMORY_ERROR;
	}
	iter->ft = ft;
	iter->path_mode = CDS_FT_ITER_PATH_CACHED;
	*result_iter = iter;
	return CDS_FT_STATUS_OK;
}

void cds_ft_iter_destroy(struct cds_ft_iter *iter)
{
	free(iter);
}

enum cds_ft_status cds_ft_iter_status(const struct cds_ft_iter *iter)
{
	return iter->status;
}

enum cds_ft_status cds_ft_iter_get_key(struct cds_ft_iter *iter,
		uint8_t *result_key, size_t result_key_max_len, size_t *result_key_len)
{
	*result_key_len = iter->key_len;
	if (iter->key_len > result_key_max_len)
		return CDS_FT_STATUS_OVERFLOW_ERROR;
	ft_ordinals_to_key(result_key, iter_key(iter), iter->key_len,
			&iter->ft->group->key_map);
	return CDS_FT_STATUS_OK;
}

enum cds_ft_status cds_ft_iter_get_prefix(struct cds_ft_iter *iter,
		uint8_t *result_key, size_t result_key_max_len, size_t *result_key_len)
{
	*result_key_len = iter->prefix_len;
	if (iter->prefix_len > result_key_max_len)
		return CDS_FT_STATUS_OVERFLOW_ERROR;
	ft_ordinals_to_key(result_key, iter_key(iter), iter->prefix_len,
			&iter->ft->group->key_map);
	return CDS_FT_STATUS_OK;
}

enum cds_ft_status cds_ft_iter_set_key(struct cds_ft_iter *iter, const uint8_t *key, size_t key_len)
{
	const struct cds_ft_key_map *km = &iter->ft->group->key_map;
	bool subset = false;
	uint8_t ordinal_buf[FT_MAX_KEY_LEN];

	key_len = ft_key_len(iter->ft, key_len);
	if (key_len > iter->ft->group->max_key_len)
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	ft_key_to_ordinals(ordinal_buf, key, key_len, km);
	if (key_len <= iter->key_len && !memcmp(ordinal_buf, iter_key(iter), key_len))
		subset = true;
	/*
	 * If new key is a subset of current key, the path stays valid,
	 * otherwise invalidate the path.
	 */
	if (!subset) {
		memcpy(iter_key(iter), ordinal_buf, key_len);
		iter->path_valid = false;
		iter_debug_path_clear(iter);
		iter->path_len = 0;
	} else {
		iter->path_len = key_len + 1;
	}
	iter->key_len = key_len;
	return CDS_FT_STATUS_OK;
}

/*
 * The prefix is a subset of the current key. Set the key before setting
 * the prefix length.
 */
enum cds_ft_status cds_ft_iter_set_prefix_len(struct cds_ft_iter *iter, size_t prefix_len)
{
	if (prefix_len > iter->key_len)
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	iter->prefix_len = prefix_len;
	return CDS_FT_STATUS_OK;
}

void cds_ft_iter_reset(struct cds_ft_iter *iter)
{
	iter->path_valid = false;
	iter_debug_path_clear(iter);
	iter->status = CDS_FT_STATUS_OK;
	iter->path_len = 0;
	iter->key_len = 0;
	iter->prefix_len = 0;
	iter->node = NULL;
#ifdef DEBUG_CLEAR_ITER
	{
		const struct cds_ft_group *ft_group = iter->ft->group;

		/* Reset to 0 for debugging. */
		memset(iter_path_node(iter), 0, ft_group->max_tree_depth * sizeof(struct cds_ft_inode_flag *));
		memset(iter_key(iter), 0, ft_group->max_key_len * sizeof(uint8_t));
	}
#endif
}

void cds_ft_iter_invalidate_path(struct cds_ft_iter *iter)
{
	iter->path_valid = false;
	iter_debug_path_clear(iter);
	iter->path_len = 0;
	iter->node = NULL;
}

void cds_ft_iter_copy(struct cds_ft_iter *dst, const struct cds_ft_iter *src)
{
	dst->status = src->status;
	dst->path_mode = src->path_mode;
	dst->path_valid = src->path_valid;
	dst->path_len = src->path_len;
	dst->key_len = src->key_len;
	dst->prefix_len = src->prefix_len;
	dst->node = src->node;
#ifdef URCU_FRACTAL_TRIE_DEBUG_PATH
	dst->gp_state = src->gp_state;
	dst->gp_state_valid = src->gp_state_valid;
#endif
	memcpy(iter_path_node(dst), iter_path_node(src), src->path_len * sizeof(struct cds_ft_inode_flag *));
	memcpy(iter_key(dst), iter_key(src), src->key_len);
}

struct cds_ft_node *cds_ft_iter_node(const struct cds_ft_iter *iter)
{
	return iter->node;
}

enum cds_ft_status cds_ft_iter_set_path_mode(struct cds_ft_iter *iter,
		enum cds_ft_iter_path_mode mode)
{
	switch (mode) {
	case CDS_FT_ITER_PATH_CACHED:
		break;
	case CDS_FT_ITER_PATH_UNCACHED:
		/*
		 * Switching to uncached mode: any previously cached
		 * path may become stale if the caller drops the RCU
		 * read-side lock, so invalidate it now.
		 */
		if (iter->path_mode == CDS_FT_ITER_PATH_CACHED) {
			iter->path_valid = false;
			iter_debug_path_clear(iter);
			iter->path_len = 0;
		}
		break;
	default:
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}
	iter->path_mode = mode;
	return CDS_FT_STATUS_OK;
}

enum cds_ft_iter_path_mode cds_ft_iter_get_path_mode(
		const struct cds_ft_iter *iter)
{
	return iter->path_mode;
}
