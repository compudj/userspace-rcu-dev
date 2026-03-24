// SPDX-FileCopyrightText: 2012-2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-only

#ifndef _URCU_FT_INTERNAL_H
#define _URCU_FT_INTERNAL_H

/*
 * src/fractal-trie-internal.h
 *
 * Userspace RCU library - Fractal Trie Internal Header
 */

#include <pthread.h>
#include <stdio.h>
#include <inttypes.h>
#include <unistd.h>
#include <urcu/rculfhash.h>
#include <assert.h>

/*
 * Configuration tweaks. Comment out those defines to disable features.
 */
#define FEATURE_USE_BITMAP_SCAN
#define FEATURE_INLINE_LOOKUP
/* #define FEATURE_SWAR_LOOKUP // disabled: slower than bytewise. */
/* #define FEATURE_SIMD_LOOKUP // disabled: slower than bytewise. */

/*
 * If the internal bit is set in a pointer, it points to an internal
 * Fractal Trie node, else it points to a node outside of the Fractal Trie.
 * This can be used for variable length keys to identify the end of key.
 */
#define FT_INTERNAL_BITS	1
#define FT_INTERNAL_MASK	(1U << 0)

/*
 * This if followed by a number of bits reserved to represent the child
 * type.
 */
#define FT_TYPE_BITS	3
#define FT_TYPE_MAX_NR	(1UL << FT_TYPE_BITS)
#define FT_TYPE_MASK	((FT_TYPE_MAX_NR - 1) << FT_INTERNAL_BITS)
#define FT_PTR_MASK	(~(FT_TYPE_MASK | FT_INTERNAL_MASK))

#define FT_ENTRY_PER_NODE	256
#define FT_LOG2_BITS_PER_BYTE	3U
#define FT_BITS_PER_BYTE	(1U << FT_LOG2_BITS_PER_BYTE)

#define FT_POOL_1D_MASK	((FT_BITS_PER_BYTE - 1) << (FT_TYPE_BITS + FT_INTERNAL_BITS))
/* 2D mask has C(n=8,r=2) = 28 possibilities (fits in 5 bits). */
#define FT_POOL_2D_MASK	(((1U << 5) - 1) << (FT_TYPE_BITS + FT_INTERNAL_BITS))

#define FT_MAX_KEY_LEN	256			/* Maximum key length supported. */
#define FT_MAX_DEPTH	(FT_MAX_KEY_LEN + 2)	/* Maximum depth, including root and leafs */

/*
 * Entry for NULL node is at index 7 (32-bit) or 8 (64-bit) of the
 * table. It is never encoded in flags.
 */
#if (CAA_BITS_PER_LONG < 64)
# define NODE_INDEX_NULL		7
#else
# define NODE_INDEX_NULL		8
#endif

/*
 * Number of removals needed on a fallback node before we try to shrink
 * it.
 */
#define FT_FALLBACK_REMOVAL_COUNT	8

#define FT_ALLOC_ORDER_MAX		12

#define FT_BITMAP_LEN			32

#ifdef FEATURE_INLINE_LOOKUP
#define inline_lookup	inline __attribute__((always_inline))
#else
#define inline_lookup
#endif

enum {
	FT_NO_BITMAP = false,
#ifdef FEATURE_USE_BITMAP_SCAN
	FT_BITMAP = true,
#else
	FT_BITMAP = false,
#endif
};

/* Never declared. Opaque type used to store flagged node pointers. */
struct cds_ft_inode_flag;
struct cds_ft_inode;

struct cds_ft_alloc_arena;
struct cds_ft_metadata_alloc;

struct cds_ft_metadata {
	struct cds_ft_node *external_nodes;	/* List of external nodes at this tree location. */
	unsigned int nr_child;			/* Number of children in node. */
	int fallback_removal_count;		/* Removals left keeping fallback. */
};

struct cds_ft_metadata_alloc {
	union {
		struct rcu_head rcu_head;			/* For deferred node reclaim. */
		struct cds_ft_metadata_alloc *free_list_next;	/* Free list next pointer. */
	};
	unsigned int alloc_index;
	struct cds_ft_metadata metadata;
};

struct cds_ft_bitmap {
	/*
	 * Bitmap is used by 2D pool and pigeon node configurations
	 * for ordered traversals. Here are the comparative costs for
	 * ordered traveral of a node:
	 *
	 * - For 2D pool, using the bitmap costs a total of 3 cache line
	 *   loads and 1 extra TLB hit, compared to a worse case of 5
	 *   cache line loads without the bitmap.
	 *
	 * - For pigeon, using the bitmap costs 2 cache line loads and
	 *   1 extra TLB hit, compared to 32 cache line loads worse case
	 *   without the bitmap.
	 *
	 *   Bitmap memory use (in bytes) (32-bit)
	 *                        bitmap size    node size       %
	 *   2D pool                   32            512       6.2
	 *   Pigeon                    32           1024       3.1
	 *
	 *   Bitmap memory use (in bytes) (64-bit)
	 *                        bitmap size    node size       %
	 *   2D pool                   32           1024       3.1
	 *   Pigeon                    32           2048       1.6
	 */
	unsigned long bitmap[FT_BITMAP_LEN / sizeof(unsigned long)];
} __attribute__((__aligned__(FT_BITMAP_LEN)));

struct cds_ft_key_map {
	bool identity;
	uint8_t key_to_ordinal[256];
	uint8_t ordinal_to_key[256];
};

struct cds_ft {
	struct cds_ft_inode_flag *root;
	struct cds_ft_metadata root_metadata;

	unsigned int max_tree_depth;
	unsigned int key_len;
	unsigned int max_key_len;	/* Maximum key length allowed. */
	unsigned long nr_fallback;	/* Number of fallback nodes used */

	const struct rcu_flavor_struct *flavor;

	/* Allocation arenas. */
	struct cds_ft_alloc_arena *arena_order[FT_ALLOC_ORDER_MAX + 1];

	struct cds_ft_key_map key_map;

	/* For debugging */
	unsigned long node_fallback_count_distribution[FT_ENTRY_PER_NODE];
	unsigned long nr_nodes_allocated, nr_nodes_freed;
};

static inline
struct cds_ft_inode_flag *ft_node_flag(struct cds_ft_inode *node,
		unsigned long type)
{
	assert(type < (1UL << FT_TYPE_BITS));
	return (struct cds_ft_inode_flag *) (((unsigned long) node) |
		(type << FT_INTERNAL_BITS) |
		FT_INTERNAL_MASK);
}

static inline
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

static inline
struct cds_ft_inode_flag *ft_node_flag_pool_2d(struct cds_ft_inode *node,
		unsigned long type, unsigned int subclass_index)
{
	assert(type < (1UL << FT_TYPE_BITS));
	return (struct cds_ft_inode_flag *) (((unsigned long) node) |
		(subclass_index << (FT_TYPE_BITS + FT_INTERNAL_BITS)) |
		(type << FT_INTERNAL_BITS) |
		FT_INTERNAL_MASK);
}

/* Hardcoded pool indexes for fast path */
#if (CAA_BITS_PER_LONG < 64)
# define FT_POOL_IDX_A	4
# define FT_POOL_IDX_B	5
#else
# define FT_POOL_IDX_A	5
# define FT_POOL_IDX_B	6
#endif
static inline
struct cds_ft_inode *ft_node_ptr(struct cds_ft_inode_flag *node)
{
	unsigned long v, type_idx;

	if (!node)
		return NULL;	/* FT_NULL */
	v = (unsigned long) node;
	type_idx = (v & FT_TYPE_MASK) >> FT_INTERNAL_BITS;

	switch (type_idx) {
	case FT_POOL_IDX_A:
		v &= ~(FT_POOL_1D_MASK | FT_TYPE_MASK | FT_INTERNAL_MASK);
		break;
	case FT_POOL_IDX_B:
		v &= ~(FT_POOL_2D_MASK | FT_TYPE_MASK | FT_INTERNAL_MASK);
		break;
	default:
		/* FT_LINEAR or FT_PIGEON */
		v &= FT_PTR_MASK;
		break;
	}
	return (struct cds_ft_inode *) v;
}

__attribute__((visibility("hidden")))
struct cds_ft_bitmap *cds_ft_item_to_bitmap(void *p, size_t item_len_order);

__attribute__((visibility("hidden")))
void cds_ft_free_all_arenas(struct cds_ft *ja);

__attribute__((visibility("hidden")))
struct cds_ft_metadata *cds_ft_item_to_metadata(void *p);

__attribute__((visibility("hidden")))
struct cds_ft_metadata *cds_ft_item_to_metadata_fast(void *p, size_t item_len_order);

__attribute__((visibility("hidden")))
void *cds_ft_metadata_to_item(struct cds_ft_metadata *metadata);

__attribute__((visibility("hidden")))
struct cds_ft_metadata *cds_ft_alloc_item(struct cds_ft *ja, size_t item_len_order, bool bitmap);

__attribute__((visibility("hidden")))
void cds_ft_free_item(struct cds_ft_metadata *metadata);

/*
 * Iterate through duplicates returned by cds_ft_lookup*()
 * Receives a struct cds_ft_node * as parameter, which is used as start
 * of duplicate list and loop cursor.
 */
#define cds_ft_for_each_duplicate(pos)				\
       for (; (pos) != NULL; (pos) = (pos)->next)

//#define DEBUG
//#define DEBUG_COUNTERS

#ifdef __linux__
#include <syscall.h>
#endif

#ifdef DEBUG
#define dbg_printf(fmt, args...)				\
	fprintf(stderr, "[debug fractal_trie %s()@%s:%u] " fmt,	\
		__func__, __FILE__, __LINE__, ## args)
#else
#define dbg_printf(fmt, args...)				\
do {								\
	/* do nothing but check printf format */		\
	if (0)							\
		fprintf(stderr, "[debug fractal_trie %s()@%s:%u] " fmt, \
			__func__, __FILE__, __LINE__, ## args);	\
} while (0)
#endif

#ifdef DEBUG_COUNTERS
static inline
int ft_debug_counters(void)
{
	return 1;
}
#else
static inline
int ft_debug_counters(void)
{
	return 0;
}
#endif

#endif /* _URCU_FT_INTERNAL_H */
