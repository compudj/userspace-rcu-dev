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

/*
 * If the internal bit is set in a pointer, it points to an internal
 * Fractal Trie node, else it points to a node outside of the Fractal Trie.
 * This can be used for variable length keys to identify the end of key.
 */
/*
 * Pointer tag encoding (bits 0-2):
 *
 *   (ptr & 0b111) == 0b000  →  external node (leaf) or NULL
 *   (ptr & 0b001) == 0b001  →  internal node (bit 0 set), bits 1-3 = type index
 *   (ptr & 0b111) == 0b010  →  compressed path node
 *   (ptr & 0b111) == 0b110  →  collapsed subtree node
 *
 * Internal nodes always have bit 0 set; the type index encoding in
 * bits 1-3 is unchanged.  Compressed and collapsed nodes use bits 1-2
 * with bit 0 clear.  External nodes have bits 0-2 clear.
 * Both compressed and collapsed nodes are >= 16-byte aligned
 * (strided allocator minimum order 4), so bits 0-3 are available.
 */
#define FT_INTERNAL_BITS	1
#define FT_INTERNAL_MASK	(1U << 0)
#define FT_COMPRESSED_MASK	(1U << 1)
#define FT_COLLAPSED_MASK	((1U << 2) | (1U << 1))	/* 0b110 */
#define FT_TAG_MASK		(FT_COMPRESSED_MASK | FT_INTERNAL_MASK)	/* 0b011 — for compressed ptr unmasking */
#define FT_TAG_MASK_WIDE	(FT_COLLAPSED_MASK | FT_INTERNAL_MASK)	/* 0b111 — for collapsed ptr unmasking and type checks */

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

/*
 * FEATURE_FT_COMPRESS: enable prefix compression (path compaction).
 * When enabled, chains of single-child internal nodes are replaced
 * with compressed path nodes.  Disabling compiles out all compressed
 * node handling, producing a simpler trie with no path compaction.
 *
 * Enabled by default.  Disable with -DNO_FEATURE_FT_COMPRESS.
 */
#ifndef NO_FEATURE_FT_COMPRESS
# define FEATURE_FT_COMPRESS
#endif
#ifndef NO_FEATURE_FT_COLLAPSE
# define FEATURE_FT_COLLAPSE
#endif

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

/*
 * Extended density counters: lazily allocated when any compact
 * uint16_t counter would overflow FT_DENSITY_COMPACT_MAX.
 * Once allocated (monotonic promotion), never demoted back.
 * Write-side only (mutex-held), no RCU publish concerns.
 */
#define FT_NODE_DENSITY_DEPTH	6
#define FT_DENSITY_COMPACT_MAX	UINT16_MAX

struct cds_ft_density_extended {
	unsigned long nr_nodes_at_depth[FT_NODE_DENSITY_DEPTH];
};

struct cds_ft_metadata {
	struct cds_ft_node *external_nodes;	/* List of external nodes at this tree location. */
	unsigned int nr_child;			/* Number of children in node. */
	int fallback_removal_count;		/* Removals left keeping fallback. */
	unsigned long nr_keys;			/* Total unique keys in subtree.
						 * Stored with uatomic_store release,
						 * loaded by updaters without uatomic,
						 * loaded by readers with uatomic_load
						 * acquire. Transiently undercounts
						 * during concurrent mutations (see
						 * ft_propagate_external_count).
						 */

	/*
	 * Local node density counters: count of traversable nodes
	 * (internal + compressed + collapsed) within a bounded window
	 * of FT_NODE_DENSITY_DEPTH levels below this node.  External
	 * nodes are excluded (already tracked by nr_keys).
	 *
	 * nr_nodes_at_depth[0] = total traversable nodes at levels
	 *                        +1 through +N (cumulative sum).
	 *                        This is the pre-filter value for
	 *                        collapse decisions.
	 * nr_nodes_at_depth[j] = nodes at level +(j+1), for j = 1..N-1.
	 *
	 * Level +1 count = [0] - sum([1]..[N-1]).
	 *
	 * Updated by ft_propagate_node_density() on every traversable
	 * node creation/destruction.  Bounded propagation: only N
	 * ancestors are updated, cost O(N) per mutation.
	 *
	 * 6 levels covers typical collapsed node depth and allows
	 * precise density tracking for collapse decisions.
	 *
	 * Compact representation: uses uint16_t counters inline
	 * (12 bytes for 6 counters vs 48 bytes for unsigned long).
	 * When any counter exceeds FT_DENSITY_COMPACT_MAX, a
	 * separate cds_ft_density_extended struct is lazily
	 * allocated and density_ext points to it (monotonic
	 * promotion, never demoted back to compact).
	 */
	struct cds_ft_density_extended *density_ext;	/* NULL = compact uint16_t mode. */
	uint16_t nr_nodes_at_depth[FT_NODE_DENSITY_DEPTH];
};

/*
 * Compressed path node.  Replaces a chain of single-child internal
 * nodes with a single node storing the key bytes inline.
 *
 * Tagged in the parent's child pointer with FT_COMPRESSED_MASK
 * (bit 1 set, bit 0 clear).
 *
 * Layout: [child pointer] [len] [key_bytes...]
 */
struct cds_ft_compressed_node {
	struct cds_ft_inode_flag *child;	/* Child at end of compressed path. */
	uint8_t len;				/* Number of key bytes in path (1-255). */
	uint8_t key_bytes[];			/* Compressed key path (flexible array). */
};

/*
 * Collapsed subtree node.  Replaces a sparse subtree with a single
 * node storing variable-length key suffixes and child pointers.
 *
 * Tagged in the parent's child pointer with FT_COLLAPSED_MASK
 * (bits 1-2 set, bit 0 clear = 0b110).
 *
 * Two-zone layout (node allocation >= 128 bytes, cache-line aligned):
 *
 *   Zone 1 (scan zone, variable size: 64, 128, or 256 bytes):
 *     [nr_entries] [offset_0] [offset_1] ... → ← ... [suffix_1] [suffix_0]
 *     Offset array grows left-to-right; suffix data grows right-to-left.
 *     Scan zone size encoded in bits 6-7 of nr_entries.
 *
 *   Zone 2 (pointer zone, starts at scan zone end):
 *     [ptr_0] [ptr_1] ... [ptr_{nr_entries-1}]
 *     Child pointers (struct cds_ft_inode_flag *), any node type.
 *
 * nr_entries encoding (uint8_t):
 *   bits 7-6 = scan zone size selector:
 *     00 = 64B  (1 cache line, FT_COLLAPSED_SCAN_64)
 *     01 = 128B (2 cache lines, FT_COLLAPSED_SCAN_128)
 *     10 = 256B (4 cache lines, FT_COLLAPSED_SCAN_256)
 *     11 = reserved
 *   bits 5-0 = entry count (max 63)
 *
 * entry_offset[i] encoding (uint8_t):
 *   bit 7 (0x80) = tombstone marker (1 = dead entry, 0 = live)
 *   bits 0-6     = byte offset from start of node to entry i's suffix
 *                  (for 256B scan zone, full 8-bit offset without tombstone)
 *
 * Suffix length derivation:
 *   Entry 0: suffix_len = scan_zone_size - (offset[0] & offset_mask)
 *   Entry i: suffix_len = (offset[i-1] & offset_mask) - (offset[i] & offset_mask)
 *
 * Lookup: scan zone 1 to find matching suffix, then
 * load ptr[i] from zone 2.
 */
#define FT_COLLAPSED_SCAN_32		0	/* bits 7-6 = 00 */
#define FT_COLLAPSED_SCAN_64		1	/* bits 7-6 = 01 */
#define FT_COLLAPSED_SCAN_128		2	/* bits 7-6 = 10 */
#define FT_COLLAPSED_SCAN_256		3	/* bits 7-6 = 11 */
#define FT_COLLAPSED_SCAN_SHIFT		6
#define FT_COLLAPSED_NR_ENTRIES_MASK	0x3F

#define FT_COLLAPSED_TOMBSTONE		0x80
#define FT_COLLAPSED_OFFSET_MASK	0x7F

/* Default scan zone size (64B, 1 cache line). */
#define FT_COLLAPSED_SCAN_ZONE_SIZE	64

/* Maximum scan zone size across all selectors (for stack buffers). */
#define FT_COLLAPSED_SCAN_ZONE_MAX	256

struct cds_ft_collapsed_node {
	uint8_t nr_entries;			/* Bits 7-6: scan zone selector.
						 * Bits 5-0: entry count. */
	uint8_t data[];				/* Offset array + suffix data (zone 1). */
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
size_t cds_ft_item_order(void *p);

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

/*
 * Delay injection for race condition testing.  When enabled via
 * FT_DELAY_INJECT, inserts a usleep at critical points to widen
 * race windows between concurrent readers and writers.
 *
 * Injection sites:
 *   FT_DELAY_WRITER  - writer side (between publish and propagation)
 *   FT_DELAY_READER  - reader side (during descent/backtracking)
 *   FT_DELAY_BOTH    - both sides
 *   FT_DELAY_RANDOM  - randomly per call (50% chance each site)
 */
enum ft_delay_mode {
	FT_DELAY_NONE    = 0,
	FT_DELAY_WRITER  = (1 << 0),
	FT_DELAY_READER  = (1 << 1),
	FT_DELAY_BOTH    = FT_DELAY_WRITER | FT_DELAY_READER,
	FT_DELAY_RANDOM  = (1 << 2),
};

#ifdef FT_DELAY_INJECT
extern enum ft_delay_mode ft_delay_mode;
extern unsigned int ft_delay_us;

static inline
void ft_delay_writer(void)
{
	if ((ft_delay_mode & FT_DELAY_WRITER) ||
	    ((ft_delay_mode & FT_DELAY_RANDOM) && (rand() & 1)))
		usleep(ft_delay_us);
}

static inline
void ft_delay_reader(void)
{
	if ((ft_delay_mode & FT_DELAY_READER) ||
	    ((ft_delay_mode & FT_DELAY_RANDOM) && (rand() & 1)))
		usleep(ft_delay_us);
}
#else
static inline void ft_delay_writer(void) { }
static inline void ft_delay_reader(void) { }
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
