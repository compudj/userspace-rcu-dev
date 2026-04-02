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
#include <stdlib.h>
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
#define FT_MAX_DEPTH	(FT_MAX_KEY_LEN + 1)	/* Maximum depth, including root. */

/*
 * Entry for NULL node is at index 7 (32-bit) or 8 (64-bit) of the
 * table. It is never encoded in flags.
 */
#if (CAA_BITS_PER_LONG < 64)
# define NODE_INDEX_NULL		7
#else
# define NODE_INDEX_NULL		8
#endif

/* Hardcoded pool indexes for fast path. */
#if (CAA_BITS_PER_LONG < 64)
# define FT_POOL_IDX_A	4
# define FT_POOL_IDX_B	5
#else
# define FT_POOL_IDX_A	5
# define FT_POOL_IDX_B	6
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

struct cds_ft_metadata {
	struct cds_ft_node *external_nodes;	/* List of external nodes at this tree location. */
	unsigned int nr_child;			/* Number of children in node. */
	int fallback_removal_count;		/* Removals left keeping fallback. */
	unsigned long nr_keys;			/* Total unique keys in subtree. */
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

struct cds_ft_group {
	size_t max_tree_depth;
	size_t key_len;
	size_t max_key_len;		/* Maximum key length allowed. */
	const struct rcu_flavor_struct *flavor;
	/* Allocation arenas. */
	struct cds_ft_alloc_arena *arena_order[FT_ALLOC_ORDER_MAX + 1];
	struct cds_ft_key_map key_map;
	unsigned long nr_ft_instances;	/* Number of Fractal Trie instances in the group. */
};


struct cds_ft {
	struct cds_ft_group *group;

	struct cds_ft_inode_flag *root;		/* Root node (arena-allocated, always present, always internal). */
	size_t max_used_key_len;		/* Maximum key length inserted (conservative). */
	unsigned long nr_fallback;		/* Number of fallback nodes used */

	/* For debugging */
	unsigned long node_fallback_count_distribution[FT_ENTRY_PER_NODE];
	unsigned long nr_nodes_allocated, nr_nodes_freed;
};

__attribute__((visibility("hidden")))
struct cds_ft_bitmap *cds_ft_item_to_bitmap(void *p, size_t item_len_order);

__attribute__((visibility("hidden")))
void cds_ft_free_all_arenas(struct cds_ft_group *ft_group);

__attribute__((visibility("hidden")))
struct cds_ft_metadata *cds_ft_item_to_metadata(void *p);

__attribute__((visibility("hidden")))
struct cds_ft_metadata *cds_ft_item_to_metadata_fast(void *p, size_t item_len_order);

__attribute__((visibility("hidden")))
void *cds_ft_metadata_to_item(struct cds_ft_metadata *metadata);

__attribute__((visibility("hidden")))
struct cds_ft_metadata *cds_ft_alloc_item(struct cds_ft *ft, size_t item_len_order, bool bitmap);

__attribute__((visibility("hidden")))
void cds_ft_free_item(struct cds_ft_metadata *metadata);

//#define DEBUG
//#define DEBUG_COUNTERS
#define DEBUG_CLEAR_ITER

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

#ifdef URCU_FRACTAL_TRIE_DEBUG_LOCKING
# define CDS_FT_ASSERT_RCU_READ_LOCKED(ft)                                     \
	do {                                                                   \
		if (caa_unlikely(!(ft)->group->flavor->read_ongoing())) {      \
			fprintf(stderr, "[Fatal] Fractal Trie API violation: " \
					"RCU read-side lock not held at "      \
					"%s:%d\n", __FILE__, __LINE__);        \
			abort();                                               \
		}                                                              \
	} while (0)
#else
# define CDS_FT_ASSERT_RCU_READ_LOCKED(ft) do { } while (0)
#endif

/*
 * URCU_FRACTAL_TRIE_DEBUG_PATH:
 *
 * Define this at build time to enable debug checks that detect use of
 * an invalid cached iterator path.  When enabled, three complementary
 * helpers track grace-period state inside the iterator:
 *
 *  iter_debug_path_snapshot() — unconditionally captures a fresh
 *      grace-period poll state via the RCU flavor's
 *      update_start_poll_synchronize_rcu.  Called once at the entry of
 *      every fresh-population operation (lookup, longest-match lookup,
 *      inequality lookup slow path and early exit).  Because it always
 *      overwrites the snapshot, an iterator that is reused across
 *      distinct RCU read-side critical sections gets a current baseline.
 *
 *  iter_debug_path_check() — polls the existing snapshot via the
 *      flavor's update_poll_state_synchronize_rcu.  Called at
 *      continuation entry points that consume a previously populated
 *      cached path (inequality lookup fast path, replace, remove).  If
 *      a full grace period has elapsed since the snapshot, the RCU
 *      read-side lock must have been dropped and the cached path is
 *      invalid — this is reported and abort() is called.
 *
 *  iter_debug_path_update() — invalidates the snapshot when the path
 *      becomes invalid (node not found / end of traversal).  It never
 *      captures a new snapshot; the one taken at the operation's entry
 *      persists as long as the path remains valid, giving a tighter
 *      detection window.
 *
 * The check is probabilistic in one direction: a false return from
 * poll does not prove the lock was held continuously (the grace period
 * may simply not have completed yet), but a true return is a definitive
 * contract violation.  This makes the check useful as a debugging aid
 * without introducing false positives.
 *
 * This option adds fields to struct cds_ft_iter, which is opaque to
 * applications.  Only the library needs to be rebuilt; the application
 * ABI is not affected.
 *
 * Requires liburcu >= 0.14 for the poll_state_synchronize_rcu APIs
 * and a struct rcu_flavor_struct that provides
 * update_start_poll_synchronize_rcu and
 * update_poll_state_synchronize_rcu function pointers.
 */

#endif /* _URCU_FT_INTERNAL_H */
