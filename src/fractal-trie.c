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
#include <sys/mman.h>
#include <urcu/fractal-trie.h>
#include <urcu/compiler.h>
#include <urcu/arch.h>
#include <urcu-pointer.h>
#include <urcu/uatomic.h>
#include "urcu-utils.h"

#include "fractal-trie-internal.h"

#ifdef FT_ENABLE_TRACING
#include "cds_ft_tp.h"
#define FT_TP(name, ...) lttng_ust_tracepoint(cds_ft, name, ##__VA_ARGS__)
/*
 * Emit a tracepoint with a key byte-sequence payload (LTTng's
 * sequence_hex field).  When tracing is disabled, the arguments are
 * discarded by the FT_TP no-op expansion, so no code is emitted.
 *
 * FT_TP_KEY_RESOLVED: caller has already resolved keylen to a real
 *   byte count (e.g. from ft_key_len() or iter->key_len).
 *
 * FT_TP_KEY: user-facing sentinel values (CDS_FT_LEN_DEFAULT ==
 *   SIZE_MAX, etc.) must be resolved via ft_key_len(ft_, keylen)
 *   before the sequence field reads keylen bytes from keybuf, or
 *   the sequence reader would attempt to serialize SIZE_MAX bytes.
 *   On resolution failure (LEN_ERROR), the key sequence is empty.
 */
#define FT_TP_KEY_RESOLVED(name, ft_, keybuf, keylen)			\
	FT_TP(name, (const void *) (ft_), (keybuf), (keylen))
#define FT_TP_KEY(name, ft_, keybuf, keylen)				\
	do {								\
		size_t _tp_klen = ft_key_len((ft_), (keylen));		\
		FT_TP_KEY_RESOLVED(name, (ft_), (keybuf),		\
			_tp_klen == CDS_FT_LEN_ERROR ? 0 : _tp_klen);	\
	} while (0)
/*
 * FT_TP_ITER_KEY: emit an iter-keyed key event.  Uses the resolved
 * iter->key_len; ft is carried alongside iter for snapshot safety
 * (iter_create may have already scrolled out of a flight recorder).
 */
#define FT_TP_ITER_KEY(name, iter)					\
	FT_TP(name, (const void *) (iter)->ft, (const void *) (iter),	\
		iter_key(iter), (iter)->key_len)
/*
 * enum ft_tp_node_kind (node-kind identifiers) lives in
 * fractal-trie-internal.h so the C side and the LTTng enum in
 * src/cds_ft_tp.h reference a single definition.  ft_tp_node_kind()
 * below maps a tagged cds_ft_inode_flag pointer to one of those
 * values.
 */
#else
#define FT_TP(name, ...)			do {} while (0)
#define FT_TP_KEY_RESOLVED(name, ft_, keybuf, keylen)	do {} while (0)
#define FT_TP_KEY(name, ft_, keybuf, keylen)	do {} while (0)
#define FT_TP_ITER_KEY(name, iter)		do {} while (0)
#endif

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

#ifdef FEATURE_FT_EXCL_VALIDATE
#include <stdarg.h>
__attribute__((noreturn, format(printf, 1, 2)))
void ft_excl_abort(const char *fmt, ...)
{
	va_list ap;

	fprintf(stderr, "FT access-discipline violation: ");
	va_start(ap, fmt);
	vfprintf(stderr, fmt, ap);
	va_end(ap);
	fflush(stderr);
	abort();
}
#endif

struct cds_ft_group_attr {
	size_t key_len;
	size_t max_key_len;
	struct cds_ft_key_map key_map;
	unsigned int flags;
	bool speculative;	/* See cds_ft_lookup_optimization. */
};

struct cds_ft_attr {
	bool exclusive;
};

enum cds_ft_type_class {
	FT_POPCOUNT = 0,	/* Popcount-bitmap: popcount_1l / popcount_2l */
	FT_PIGEON = 1,		/* Pigeon: direct indexed */
	/* Leaf nodes are implicit from their height in the tree */
	FT_NR_TYPES = 2,

	FT_NULL,	/* not an encoded type, but keeps code regular */
};

#define ft_type_is_popcount(tc)	((tc) == FT_POPCOUNT)
#define ft_type_is_pigeon(tc)	((tc) == FT_PIGEON)

struct cds_ft_type {
	enum cds_ft_type_class type_class;
	uint16_t min_child;		/* minimum number of children: 1 to 256 */
	uint16_t max_child;		/* maximum number of children: 1 to 256 */
	uint16_t max_linear_child;	/* max children with linear-scan layout: 1 to 256 */
	uint16_t order;			/* node size is (1 << order), in bytes */
	bool bitmap;			/* allocate bitmap */
	bool popcount_2l;		/* 2-level popcount-bitmap layout */
	bool popcount_1l;		/* 1-level byte popcount layout */
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
 */

/*
 * The smallest allocation order we can use is 4:
 * - 1 bit is reserved for internal vs external flag,
 * - 3 bits are reserved to encode the node type.
 */

/*
 * The popcount scanners hardcode the pointer offset that matches
 * their header layout (scan_16_16_max_3 at +8, scan_16_16_max_5 at
 * +12, scan_32_8 / scan_64_4 at +16, popcount_1l at +32), and
 * access pointers via sizeof(void *) stride.  Both offsets and
 * strides remain valid on 32-bit pointers, so the scanners
 * themselves are pointer-size agnostic.  What differs is per-node
 * capacity: 32-bit pointers leave trailing space, and large bp_1l
 * nodes can fit substantially more children for the same node size
 * (the 32 B bitmap header is fixed).  The 32-bit ft_types[] below
 * tunes max_linear_child accordingly.
 *
 * Layout naming: scan_<root_bits>_<sub_bm_bits>(_max_<N>).  The
 * trailing _max_<N> appears only on the generic-2L variants
 * (scan_16_16_*) where the ptr offset depends on the static
 * sub_bm[] tail length (max_lc).  The flat-layout scanners
 * (scan_32_8 with 5+3 byte split and u8 nibbles; scan_64_4 with
 * 6+2 byte split and u4 nibbles) pack root_bm + packed_bms into a
 * fixed 12 B or 16 B header, so the ptr offset is constant (+16
 * after alignment) and one function serves multiple max_lc values.
 *
 * 32-bit max_lc tuning: the flat scanners (scan_32_8, scan_64_4)
 * are layout-only -- headers do not grow with max_lc and packed_bms
 * supports up to 64 distinct (hi, lo) pairs.  On 32-bit ptrs,
 * scan_64_4 hosts max_lc=12 in 64 B and max_lc=16 in 128 B (the
 * latter is bitmap-bound; trailing 48 B is layout slack).  The
 * layout helpers (header_bytes, get_nr_child, get_ith_pos,
 * set_nth) accept all canonical max_lc values; the dispatcher
 * routes by max_lc.
 *
 * scan_16_16_max_3 stays at max_lc=3 on both arches: the 8 B header
 * packs root_bm + 3 sub_bm exactly and cannot grow without changing
 * the load shape.  scan_16_16_max_5 (32-bit only) extends the same
 * generic 2L layout with two extra sub_bm slots so the 12 B header
 * + 5 x 4 B ptrs fills the 32 B order-5 node exactly.
 */

#if (CAA_BITS_PER_LONG < 64)

/* 32-bit pointers */
enum {
	/*
	 * 32-bit max-density popcount tiers:
	 *   idx 0  scan_16_16_max_5  32 B  hdr 12 B +  5 x 4 B = 32 B exact
	 *   idx 1  scan_64_4         64 B  hdr 16 B + 12 x 4 B = 64 B exact
	 *                            (flat 6+2; node-size cap 12, bitmap cap 16)
	 *   idx 2  popcount_1l      128 B  hdr 32 B + 24 x 4 B = 128 B exact
	 *                            (was scan_64_4 max_lc=16 with 48 B slack;
	 *                            popcount_1l unlocks the full pointer table)
	 *   idx 3  popcount_1l      256 B  hdr 32 B + 56 x 4 B = 256 B exact
	 *   idx 4  popcount_1l      512 B  hdr 32 B + 120 x 4 B = 512 B exact
	 *   idx 5  pigeon          1024 B  256 x 4 B pointers
	 */
	ft_type_0_max_child = 5,
	ft_type_1_max_child = 12,
	ft_type_2_max_child = 24,
	ft_type_3_max_child = 56,
	ft_type_4_max_child = 120,
	ft_type_5_max_child = 256,
	ft_type_6_max_child = 0,	/* NULL */
};

enum {
	ft_type_0_max_linear_child = 5,
	ft_type_1_max_linear_child = 12,
};

const struct cds_ft_type ft_types[] = {
	[0] = {
		.type_class = FT_POPCOUNT,
		.popcount_2l = true,
		.min_child = 1,
		.max_child = ft_type_0_max_child, .max_linear_child = ft_type_0_max_linear_child, .order = 5, .bitmap = FT_NO_BITMAP,
	},
	[1] = {
		.type_class = FT_POPCOUNT,
		.popcount_2l = true,
		.min_child = 3,
		.max_child = ft_type_1_max_child, .max_linear_child = ft_type_1_max_linear_child, .order = 6, .bitmap = FT_NO_BITMAP,
	},
	[2] = {
		/* 32 B bitmap + 24 x 4 B ptrs = 128 B (order-7). */
		.type_class = FT_POPCOUNT,
		.popcount_1l = true,
		.min_child = 7,
		.max_child = ft_type_2_max_child, .max_linear_child = ft_type_2_max_child, .order = 7, .bitmap = FT_NO_BITMAP,
	},

	/*
	 * Indices 2, 3, and 4 use popcount_1l (32 B bitmap + 4 B ptr
	 * table fills the node exactly).
	 */
	[3] = {
		/* 32 B bitmap + 56 * 4 B ptrs = 256 B (order-8). */
		.type_class = FT_POPCOUNT,
		.max_linear_child = ft_type_3_max_child,
		.popcount_1l = true,
		.min_child = 10,
		.max_child = ft_type_3_max_child, .order = 8, .bitmap = FT_NO_BITMAP },
	[4] = {
		/* 32 B bitmap + 120 * 4 B ptrs = 512 B (order-9). */
		.type_class = FT_POPCOUNT,
		.max_linear_child = ft_type_4_max_child,
		.popcount_1l = true,
		.min_child = 28,
		.bitmap = FT_NO_BITMAP,
		.max_child = ft_type_4_max_child, .order = 9 },

	/*
	 * Upon node removal below min_child, if a popcount_1l node would
	 * otherwise be filled beyond capacity, we roll back to pigeon.
	 */
	[5] = { .type_class = FT_PIGEON,
		.min_child = 51,
		.max_child = ft_type_5_max_child, .order = 10, .bitmap = FT_BITMAP },

	[6] = { .type_class = FT_NULL, .min_child = 0, .max_child = ft_type_6_max_child, .bitmap = FT_NO_BITMAP },
	/*
	 * Slot 7 padding: FT_TYPE_BITS = 3 ⇒ tag-encodable type_index
	 * range is 0..7.  The 32-bit tier uses only indices 0..6 (real
	 * types + FT_NULL); slot 7 must exist so ft_types[] covers every
	 * tag value the encoder can produce.
	 */
	[7] = { .type_class = FT_NULL, .min_child = 0, .max_child = 0, .bitmap = FT_NO_BITMAP },
};
#else /* !(CAA_BITS_PER_LONG < 64) */
/* 64-bit pointers */
enum {
	ft_type_0_max_child = 3,
	ft_type_1_max_child = 6,	/* scan_32_8 (per-slot 5+3, qp_6) */
	ft_type_2_max_child = 14,	/* scan_64_4 (flat 6+2, qp_14) */
	ft_type_3_max_child = 28,
	ft_type_4_max_child = 60,
	ft_type_5_max_child = 124,
	ft_type_6_max_child = 256,
	ft_type_7_max_child = 256,
};

enum {
	ft_type_0_max_linear_child = 3,
	/*
	 * scan_32_8 (per-slot 5+3 byte split, max_lc=6): 12-byte popcount
	 * header (4B root + 8B packed sub_bms) + 6 x 8-byte pointers into
	 * the 64B order-6 node.
	 *
	 * scan_64_4 (flat 6+2 byte split, max_lc=14): 16-byte popcount
	 * header (8B root + 8B packed_bms with 14 x 4-bit sub_bms =
	 * 56 bits used) + 14 x 8-byte pointers into the 128B order-7
	 * node (16 + 14 * 8 = 128 exactly).
	 */
	ft_type_1_max_linear_child = 6,
	ft_type_2_max_linear_child = 14,
};

const struct cds_ft_type ft_types[] = {
	[0] = {
		.type_class = FT_POPCOUNT,
		.popcount_2l = true,
		.min_child = 1, .max_child = ft_type_0_max_child, .max_linear_child = ft_type_0_max_linear_child, .order = 5, .bitmap = FT_NO_BITMAP,
	},
	[1] = {
		.type_class = FT_POPCOUNT,
		.popcount_2l = true,
		.min_child = 3, .max_child = ft_type_1_max_child, .max_linear_child = ft_type_1_max_linear_child, .order = 6, .bitmap = FT_NO_BITMAP,
	},
	[2] = {
		.type_class = FT_POPCOUNT,
		.popcount_2l = true,
		.min_child = 5, .max_child = ft_type_2_max_child, .max_linear_child = ft_type_2_max_linear_child, .order = 7, .bitmap = FT_NO_BITMAP,
	},
	[3] = {
		.type_class = FT_POPCOUNT,
		.popcount_1l = true,
		.min_child = 10, .max_child = ft_type_3_max_child, .max_linear_child = ft_type_3_max_child, .order = 8, .bitmap = FT_NO_BITMAP,
	},

	/*
	 * Indices 4 and 5 use popcount_1l (32B bitmap + 8B ptr
	 * table fills the node).
	 */
	[4] = {
		/* 32B bitmap + 60 * 8B ptrs = 512B order-9. */
		.type_class = FT_POPCOUNT,
		.max_linear_child = ft_type_4_max_child,
		.popcount_1l = true,
		.min_child = 22,
		.max_child = ft_type_4_max_child,
		.order = 9, .bitmap = FT_NO_BITMAP },
	[5] = {
		/* 32B bitmap + 124 * 8B ptrs = 1024B order-10. */
		.type_class = FT_POPCOUNT,
		.max_linear_child = ft_type_5_max_child,
		.popcount_1l = true,
		.min_child = 51,
		.max_child = ft_type_5_max_child,
		.order = 10, .bitmap = FT_NO_BITMAP },

	/*
	 * Upon node removal below min_child, if a popcount_1l node would
	 * otherwise be filled beyond capacity, we roll back to pigeon.
	 */
	[6] = { .type_class = FT_PIGEON, .min_child = 95, .max_child = ft_type_6_max_child, .order = 11, .bitmap = FT_BITMAP },

	[7] = { .type_class = FT_NULL, .min_child = 0, .max_child = ft_type_7_max_child, .bitmap = FT_NO_BITMAP },
};
#endif /* !(BITS_PER_LONG < 64) */

/*
 * The cds_ft_inode contains the compressed node data needed for
 * the read-side traversal. Because the actual layout depends on the
 * node's type_class (Linear, Popcount, or Pigeon), the struct uses a single
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
 * 1. FT_POPCOUNT (two variants, popcount_2l and popcount_1l):
 * - popcount_2l: header with a root_bm + per-chunk sub_bm packed
 *   in a fixed-size header (8 B / 12 B / 16 B depending on
 *   scan_<rootbits>_<subbits> variant), followed by an array of
 *   child pointers indexed by bitmap rank.
 * - popcount_1l: 32 B flat 256-bit bitmap header, followed by an
 *   array of child pointers indexed by bitmap rank.
 *
 * 2. FT_PIGEON:
 * - A direct, flat array of up to 256 (struct cds_ft_inode_flag *) pointers.
 * - No key array is stored inside the node — the key is implicit
 *   from the pointer's array index.
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
#ifdef FEATURE_FT_SKIP_COMPRESSED
	/*
	 * skip_slot_offset is 8 bits and stores byte_offset / sizeof(void *).
	 * Ensure the largest node (pigeon, 2^11 = 2048 bytes) fits:
	 * 2048 / sizeof(void *) = 256 slots, max index 255.  Only enabled
	 * on 64-bit architectures, where sizeof(void *) == 8 and the
	 * quotient is exactly 256.
	 */
	CAA_BUILD_BUG_ON((1U << 11) / sizeof(void *) > 256);
#endif
	/*
	 * Metadata packed bitfield must fit in a uint32_t.
	 * Layout: nr_child(9) + [skip_slot_offset(8)] +
	 *         fallback_removal(3) + alloc_index(FT_ALLOC_INDEX_BITS).
	 */
	CAA_BUILD_BUG_ON(9 + FT_FALLBACK_REMOVAL_BITS + FT_ALLOC_INDEX_BITS
#ifdef FEATURE_FT_SKIP_COMPRESSED
		+ 8
#endif
		> 32);
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
 * most common case in trie traversal (compressed paths), followed
 * by medium keys, then long keys where SIMD helps.
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

#if defined(__AVX2__) && !defined(FT_NO_SIMD_CMP)
/*
 * Unmasked 32-byte AVX2 short-key compare for 1 <= len <= 32.
 *
 * Caller guarantees @a and @b each have at least 32 readable bytes.
 * Load is unmasked; mask is applied only to the comparison result
 * so bytes past @len don't influence the outcome.
 *
 * Cheapest short-key path when the contract permits it: one
 * vmovdqu pair + vpcmpeqb + vpmovmskb + bzhi + andn + jne.  No
 * page-cross check (caller-promised safe), no EVEX kmask setup,
 * no overlapping tail load.  ~10 cycles on Zen 4 for the matching
 * case.
 */
static inline_lookup
int ft_cmp_short_unmasked_avx2(const uint8_t *a, const uint8_t *b,
		unsigned int len,
		bool signed_cmp, unsigned int *mismatch_pos)
{
	__m256i va = _mm256_loadu_si256((const __m256i *) a);
	__m256i vb = _mm256_loadu_si256((const __m256i *) b);
	__m256i eq = _mm256_cmpeq_epi8(va, vb);
	uint32_t mask = (uint32_t) _mm256_movemask_epi8(eq);
	uint32_t want = (uint32_t) _bzhi_u32(0xFFFFFFFFU, len);

	if ((mask & want) == want)
		return 0;
	return ft_avx2_mismatch(a, b, 0, (~mask) & want,
				signed_cmp, mismatch_pos);
}
#endif

#if defined(__SSE2__) && !defined(FT_NO_SIMD_CMP)
/*
 * Unmasked 16-byte SSE2 short-key compare for 1 <= len <= 16.
 *
 * Same idea as ft_cmp_short_unmasked_avx2 but 16-byte load width.
 * Caller guarantees @a and @b each have at least 16 readable bytes.
 */
static inline_lookup
int ft_cmp_short_unmasked_sse2(const uint8_t *a, const uint8_t *b,
		unsigned int len,
		bool signed_cmp, unsigned int *mismatch_pos)
{
	__m128i va = _mm_loadu_si128((const __m128i *) a);
	__m128i vb = _mm_loadu_si128((const __m128i *) b);
	__m128i eq = _mm_cmpeq_epi8(va, vb);
	unsigned int mask = (unsigned int) _mm_movemask_epi8(eq);
	unsigned int want = (1U << len) - 1U;

	if ((mask & want) == want)
		return 0;
	return ft_sse2_mismatch(a, b, 0, (~mask) & want,
				signed_cmp, mismatch_pos);
}
#endif

#if defined(__AVX512VL__) && defined(__AVX512BW__) && !defined(FT_NO_SIMD_CMP)
/*
 * AVX-512 short-key compare for 1 <= len <= 32 (BW + VL).
 *
 * Predicated load on BOTH sides + predicated compare.  The k-mask
 * gates which byte lanes each load fetches — bytes outside the mask
 * are NOT read from memory (architectural guarantee in Intel SDM
 * and AMD APM for AVX-512 masked memory operands).  Page-cross safe
 * on both pointers by construction; no runtime check, no
 * @readable_bytes contract needed beyond @readable_bytes >= @len.
 *
 * Cheaper than the AVX2 fallback (no page-cross branch, no separate
 * mask-and-result step) and matches what glibc's __memcmp_evex_movbe
 * emits for len <= 32.
 */
static inline_lookup
int ft_cmp_short_avx512(const uint8_t *a, const uint8_t *b,
		unsigned int len,
		bool signed_cmp, unsigned int *mismatch_pos)
{
	__mmask32 k = (__mmask32) _bzhi_u32(0xFFFFFFFFU, len);
	__m256i va = _mm256_maskz_loadu_epi8(k, (const void *) a);
	__m256i vb = _mm256_maskz_loadu_epi8(k, (const void *) b);
	__mmask32 ne = _mm256_mask_cmpneq_epu8_mask(k, va, vb);

	if (ne == 0)
		return 0;
	return ft_avx2_mismatch(a, b, 0, (uint32_t) ne,
				signed_cmp, mismatch_pos);
}
#endif

#if defined(__AVX2__) && !defined(FT_NO_SIMD_CMP)
/*
 * Page-cross-safe single-vector compare for 16 <= len <= 31.
 *
 * Loads 32 bytes from both pointers, compares all 32, masks the
 * result to the first @len lanes.  Reading 7-15 bytes past the
 * requested length is safe iff neither pointer is in the last 32
 * bytes of its 4 KB page — the page-cross check below.  If the
 * check fails, fall back to the overlapping-pair SSE2 path which
 * never reads past byte @len-1.
 *
 * The branch is highly predictable: in practice both @a and @b
 * are typically in the first ~4000 bytes of their pages (slot
 * bodies start CL-aligned and never span pages for our slot
 * sizes), so the fast path is taken essentially 100% of the time.
 *
 * Used only when AVX-512 BW+VL is not available; the AVX-512 path
 * above does the same job without the page-cross check.
 */
static inline_lookup
int ft_cmp_short_avx2(const uint8_t *a, const uint8_t *b,
		unsigned int len,
		bool signed_cmp, unsigned int *mismatch_pos)
{
	/*
	 * Page-cross check: a 32-byte load at @p is safe (cannot cross
	 * a page boundary) iff (p & 0xFFF) <= 0xFE0.  Both pointers
	 * must be safe; OR-ing the low 12 bits picks up the worst of
	 * the two with a single AND.
	 */
	if (caa_likely((((uintptr_t)a | (uintptr_t)b) & 0xFFFu) <= 0xFE0u)) {
		__m256i va = _mm256_loadu_si256((const __m256i *)a);
		__m256i vb = _mm256_loadu_si256((const __m256i *)b);
		__m256i eq = _mm256_cmpeq_epi8(va, vb);
		uint32_t mask = (uint32_t)_mm256_movemask_epi8(eq);
		uint32_t want = (uint32_t)((1ULL << len) - 1ULL);

		if ((mask & want) != want)
			return ft_avx2_mismatch(a, b, 0,
						(~mask) & want,
						signed_cmp, mismatch_pos);
		return 0;
	}
	return ft_cmp_sse2(a, b, len, signed_cmp, mismatch_pos);
}

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
		uint32_t mask = (uint32_t)_mm256_movemask_epi8(eq);

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
		uint32_t mask = (uint32_t)_mm256_movemask_epi8(eq);

		if (mask != 0xFFFFFFFFU)
			return ft_avx2_mismatch(a, b, tail,
						~mask, signed_cmp,
						mismatch_pos);
	}
	return 0;
}
#endif /* __AVX2__ && !FT_NO_SIMD_CMP */

/*
 * ft_key_cmp_ordinals: compare @len bytes in ordinal space.
 *
 * @readable_bytes: contract from the caller — @a and @b each have
 *                  at least this many bytes safely readable (no
 *                  fault on load).  Conservative callers pass
 *                  @readable_bytes = @len (no over-read promised).
 *                  Callers that know their buffers have trailing
 *                  padding pass a larger value, unlocking the
 *                  widest unmasked single-load fast path.
 *
 * Dispatch ordered by call-site frequency (hottest first):
 *
 *   1. len <= 32 && readable >= 32:    ft_cmp_short_unmasked_avx2
 *                                      (1 vmovdqu pair + mask to @len)
 *                                      — HOT, single likely branch.
 *
 *   AVX-512 BW+VL build:
 *   2. len <= 32 (any readable):       ft_cmp_short_avx512
 *                                      (predicated load on both sides,
 *                                       handles 1..32 contract-free)
 *
 *   Non-AVX-512 build (SWAR ladder):
 *   2. len <= 16 && readable >= 16:    ft_cmp_short_unmasked_sse2
 *   3. len < 8:                        tiny (masked 8-B word if
 *                                       readable >= 8, else byte-by-byte)
 *   4. 8 <= len < 16:                  word-overlap-pair
 *   5. 16 <= len <= 32:                ft_cmp_short_avx2 (page-cross check)
 *                                      / SSE2 overlapping pair
 *
 *   Final (any build):
 *   6. len > 32:                       loop + overlapping tail
 *                                      (contract-independent; rare).
 *
 * Key insight: when @readable_bytes >= 32, every short key
 * (regardless of @len: 1..32) takes the same fast path.  The same
 * code is emitted for len=5 and len=25 — only the mask differs.
 * When AVX-512 BW+VL is available, the predicated short path
 * covers ALL of 1..32 without any contract, so the SWAR ladder is
 * elided entirely.
 */
static inline_lookup
int ft_key_cmp_ordinals(const uint8_t *a, const uint8_t *b,
		unsigned int len, unsigned int readable_bytes,
		bool signed_cmp, unsigned int *mismatch_pos)
{
	if (caa_likely(len <= 32)) {
		/*
		 * Hot path: caller-promised 32-B readable horizon on
		 * both sides — single unmasked AVX2 load each side.
		 */
#if defined(__AVX2__) && !defined(FT_NO_SIMD_CMP)
		if (caa_likely(readable_bytes >= 32))
			return ft_cmp_short_unmasked_avx2(a, b, len, signed_cmp, mismatch_pos);
#endif
#if defined(__AVX512VL__) && defined(__AVX512BW__) && !defined(FT_NO_SIMD_CMP)
		/*
		 * AVX-512 BW+VL: predicated loads on both sides are
		 * page-cross safe and cover all 1..32 without further
		 * dispatch.  Wins over the SWAR ladder for the common
		 * short-key range (16..32) and matches glibc's
		 * __memcmp_evex_movbe for shorter keys too.
		 */
		return ft_cmp_short_avx512(a, b, len, signed_cmp, mismatch_pos);
#else
		/* Short key, no 32-B contract.  Try 16-B contract. */
# if defined(__SSE2__) && !defined(FT_NO_SIMD_CMP)
		if (len <= 16 && readable_bytes >= 16)
			return ft_cmp_short_unmasked_sse2(a, b, len, signed_cmp, mismatch_pos);
# endif
		/* len < 8: tiny (byte/masked-word). */
		if (len < sizeof(unsigned long))
			return ft_cmp_tiny(a, b, len, readable_bytes,
					   signed_cmp, mismatch_pos);
		/* 8 <= len < 16: word-overlap-pair. */
		if (len < 16)
			return ft_cmp_word(a, b, len, signed_cmp, mismatch_pos);
		/* 16 <= len <= 32 without 32-B contract: safe fallback. */
# if defined(__AVX2__) && !defined(FT_NO_SIMD_CMP)
		return ft_cmp_short_avx2(a, b, len, signed_cmp, mismatch_pos);
# elif defined(__SSE2__) && !defined(FT_NO_SIMD_CMP)
		return ft_cmp_sse2(a, b, len, signed_cmp, mismatch_pos);
# else
		return ft_cmp_word(a, b, len, signed_cmp, mismatch_pos);
# endif
#endif
	}

	/* Long key (>32): contract-independent loop. */
#if defined(__AVX2__) && !defined(FT_NO_SIMD_CMP)
	return ft_cmp_avx2(a, b, len, signed_cmp, mismatch_pos);
#elif defined(__SSE2__) && !defined(FT_NO_SIMD_CMP)
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
 * ft_metadata_set_external_nodes: set external_nodes on a node's metadata.
 * Asserts that the node is not a compressed node (compressed nodes
 * must not carry metadata->external_nodes).
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
	if (external_nodes)
		external_nodes->prev = node_flag;
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
/* Forward declarations for nr_keys helpers. */
static inline unsigned long ft_nr_keys_get(const struct cds_ft_metadata *m);
static inline unsigned long ft_nr_keys_load(const struct cds_ft_metadata *m);
static inline void ft_nr_keys_store(struct cds_ft_metadata *m, unsigned long val, int mo);

/*
 * ft_parent_depth_span: number of key bytes a parent's slot covers.
 * Trivially 1 for every surviving node type (compressed/skip-compressed
 * still resolve via metadata->parent on the multi-byte hop, but the
 * caller of this helper iterates one ancestor at a time).
 */
#define ft_parent_depth_span(p, c)	((void)(p), (void)(c), 1U)

static inline_lookup
struct cds_ft_inode *ft_node_ptr(struct cds_ft_inode_flag *node)
{
	unsigned long v = (unsigned long) node;

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

	if (_ft_node_mask_ptr(node) == NULL) {
		return NODE_INDEX_NULL;
	}
	/* Compressed nodes don't have a type index. */
	assert(!ft_node_compressed(node));
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
 * a self-consistent slot+cn pair use ft_skip_to_compressed_validate
 * (with retry at the caller level).
 *
 * Read-side safe (rcu_dereference on both fields).  Callers must be
 * in an RCU read-side critical section (or QSBR equivalent).
 */
static inline
struct cds_ft_compressed_node *ft_skip_to_compressed(
		struct cds_ft_inode_flag *skip_ptr)
{
	struct cds_ft_inode_flag *child = ft_skip_child_ptr(skip_ptr);
	struct cds_ft_inode_flag *parent;

	if (ft_node_external(child))
		parent = rcu_dereference(((struct cds_ft_node *) child)->prev);
	else
		parent = rcu_dereference(cds_ft_item_to_metadata(
			ft_node_ptr(child))->parent);
	return ft_compressed_node_ptr(parent);
}

/*
 * ft_skip_to_compressed_validate: reader-only variant that validates
 * the recovery against the slot's skip_len.  Returns NULL on mismatch,
 * indicating a concurrent writer is mid-split/merge: the child's
 * back-pointer has been updated in-place but the parent slot value
 * hasn't been republished yet.  The caller is expected to be in the
 * reader role: re-read the slot from its source and retry, bounded
 * by the writer's structural mutation window (microseconds — between
 * ft_set_parent and ft_publish_to_parent on the writer side).
 *
 * The validation works because:
 *   - Split always shortens: sfx->len < CN->len.
 *   - Chain-merge always lengthens: CN_merged->len = CN1->len + CN2->len.
 * So every problematic in-place mutation lands at a cn whose len
 * differs from the slot's skip_len.  Same-length spurious matches
 * are correct: the only structurally relevant property for the
 * caller (ordinal_key byte count, cn->key_bytes content) is the
 * length; same-length cn replacement consumes the same byte count.
 *
 * Writers must NOT use this function: a writer mid-mutation reading
 * its own in-flight state would loop forever (it IS the race partner).
 * Writers use ft_skip_to_compressed (no validation).
 */
static inline
struct cds_ft_compressed_node *ft_skip_to_compressed_validate(
		struct cds_ft_inode_flag *skip_ptr)
{
	unsigned int skip_len_slot = ft_skip_len(skip_ptr);
	struct cds_ft_compressed_node *cn = ft_skip_to_compressed(skip_ptr);

	if (caa_unlikely(!cn || cn->len != skip_len_slot))
		return NULL;
	return cn;
}

/*
 * ft_get_parent_rcu: read the parent pointer of @node via
 * rcu_dereference.
 *
 * For external (leaf) nodes: returns cds_ft_node.prev.  @node must
 * be the head of its duplicate chain (non-head duplicates' prev
 * points to the preceding node in the chain, not to the parent).
 * Iterators and lookups maintain this invariant by convention —
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
 */
static inline
struct cds_ft_inode_flag *ft_get_parent_rcu(struct cds_ft_inode_flag *node)
{
	struct cds_ft_inode_flag *parent;

	if (ft_node_external(node))
		parent = rcu_dereference(((struct cds_ft_node *) node)->prev);
	else
		parent = rcu_dereference(cds_ft_item_to_metadata(
			ft_node_ptr(node))->parent);
	assert(!parent || !ft_node_external(parent));
	return parent;
}

static inline
bool ft_group_skip_compressed(const struct cds_ft_group *group)
{
	return group->flags & CDS_FT_FLAG_SKIP_COMPRESSED;
}

/*
 * ft_set_skip_slot: encode the skip pointer slot address as a
 * pointer-stride offset from the parent node.  The raw byte offset
 * is divided by sizeof(void *) (always 8 on 64-bit) so that the
 * 8-bit field can cover the full pigeon node (2048 bytes / 8 = 256
 * slots, max index 255).
 *
 * When parent is NULL (root's child), the offset is unused —
 * ft_get_skip_slot recovers &ft->root.
 */
static inline
void ft_set_skip_slot(struct cds_ft_metadata *meta,
		struct cds_ft_inode_flag **slot)
{
	if (!slot)
		return;	/* Slot unknown — preserve existing offset. */
	if (!meta->parent) {
		meta->skip_slot_offset = 0;
		return;
	}
	meta->skip_slot_offset = (unsigned int)((char *) slot -
		(char *) ft_node_ptr(meta->parent)) / sizeof(void *);
}

/*
 * ft_get_skip_slot: recover the skip pointer slot address from the
 * stored pointer-stride offset.  Returns NULL if skip_slot_offset
 * is 0 and parent is non-NULL (slot was never set).
 *
 * @ft is needed for the root case (parent == NULL).
 */
static inline
struct cds_ft_inode_flag **ft_get_skip_slot(const struct cds_ft_metadata *meta,
		struct cds_ft *ft)
{
	if (!meta->parent)
		return &ft->root;
	if (!meta->skip_slot_offset)
		return NULL;
	return (struct cds_ft_inode_flag **)
		((char *) ft_node_ptr(meta->parent) +
		 (unsigned int) meta->skip_slot_offset * sizeof(void *));
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
struct cds_ft_compressed_node *ft_skip_to_compressed(
		struct cds_ft_inode_flag *skip_ptr)
{
	return ft_compressed_node_ptr(skip_ptr);
}

static inline
struct cds_ft_compressed_node *ft_skip_to_compressed_validate(
		struct cds_ft_inode_flag *skip_ptr)
{
	return ft_compressed_node_ptr(skip_ptr);
}

static inline
bool ft_group_skip_compressed(const struct cds_ft_group *group __attribute__((unused)))
{
	return false;
}

static inline
void ft_set_skip_slot(struct cds_ft_metadata *meta __attribute__((unused)),
		struct cds_ft_inode_flag **slot __attribute__((unused)))
{
}

static inline
struct cds_ft_inode_flag **ft_get_skip_slot(
		const struct cds_ft_metadata *meta __attribute__((unused)),
		struct cds_ft *ft __attribute__((unused)))
{
	return NULL;
}
#endif /* FEATURE_FT_SKIP_COMPRESSED */

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
#ifdef FEATURE_FT_SKIP_COMPRESSED
	if (ft_node_skip_compressed(nf)) {
		struct cds_ft_compressed_node *cn =
			ft_skip_to_compressed(nf);
		return cds_ft_item_to_metadata(
			(struct cds_ft_inode *) cn);
	}
#endif
	return cds_ft_item_to_metadata(ft_node_ptr(nf));
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
struct cds_ft_inode_flag *ft_resolve_skip_compressed(
		struct cds_ft_inode_flag *nf)
{
	if (ft_node_skip_compressed(nf))
		return ft_compressed_node_flag(ft_skip_to_compressed(nf));
	return nf;
}

#ifdef FEATURE_FT_SKIP_COMPRESSED
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
#endif

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
 * ft_publish_to_parent: atomically publish @new_child into @parent_slot.
 *
 * If the parent is a compressed node, also update the skip pointer
 * at *skip_slot (if one exists) BEFORE writing *parent_slot.  This
 * ensures candidate readers (which follow the skip pointer) see the
 * new child before exact/inequality readers (which follow cn->child).
 *
 * For compressed-form @new_child (SKIP_X or plain COMPRESSED), also
 * maintains the underlying compressed node's skip_slot_offset so it
 * records @parent_slot's offset in @parent_nf — required by
 * ft_get_skip_slot lookups (the dual-pointer dance above, and the
 * chain-merge canonicalization in ft_detach_node that publishes a
 * replacement at the cn's same grandparent slot).  Without this,
 * compressed nodes installed via ft_publish_to_parent rather than via
 * ft_node_set_nth → ft_set_parent leave skip_slot_offset == 0 — a
 * latent gap that silently disabled the dual-pointer SKIP_X update
 * and tripped chain-merge.  This intentionally does NOT update
 * @new_child's parent linkage; callers manage that via their own
 * ft_set_parent (or by direct meta->parent assignment), with
 * semantics that vary across call sites.
 *
 * Centralizes the dual-pointer RCU publication pattern so every
 * write to cn->child automatically maintains the skip pointer.
 */
static
void ft_publish_to_parent(struct cds_ft *ft,
		struct cds_ft_inode_flag *parent_nf,
		struct cds_ft_inode_flag **parent_slot,
		struct cds_ft_inode_flag *new_child)
{
#ifdef FEATURE_FT_SKIP_COMPRESSED
	/*
	 * For compressed-form @new_child (SKIP_X or plain COMPRESSED):
	 * maintain the compressed node's skip_slot_offset so that it
	 * records the slot holding it in @new_child's eventual parent
	 * node.  This is the value ft_get_skip_slot(cn_meta) recovers
	 * later — used by dual-pointer publishes from cn->child
	 * (ft_publish_to_parent itself, when called with parent_nf = cn)
	 * and by chain-merge canonicalization (ft_detach_node) that
	 * publishes a replacement at the same slot.
	 *
	 * Without this, compressed nodes installed via ft_publish_to_parent
	 * (rather than via ft_node_set_nth, which routes through
	 * ft_set_parent) leave skip_slot_offset == 0 — a latent gap that
	 * silently disabled the dual-pointer SKIP_X update at the
	 * grandparent slot and tripped chain-merge that *needs* the slot.
	 *
	 * Both encodings (SKIP_X and plain COMPRESSED) are handled.
	 *
	 * We do NOT touch @new_child's parent linkage here; callers manage
	 * that via their own ft_set_parent (or by direct meta->parent
	 * assignment) before calling us.
	 *
	 * Skip the update at the root slot (&ft->root): root nodes have
	 * no parent, and ft_set_skip_slot's offset computation assumes
	 * the slot lives inside a node-arena chunk.
	 */
	if (parent_slot != &ft->root) {
		struct cds_ft_compressed_node *cn = NULL;

		if (ft_node_skip_compressed(new_child))
			cn = ft_skip_to_compressed(new_child);
		else if (ft_node_compressed(new_child))
			cn = ft_compressed_node_ptr(new_child);
		if (cn) {
			struct cds_ft_metadata *cn_meta =
				cds_ft_item_to_metadata(
					(struct cds_ft_inode *) cn);

			if (cn_meta->parent)
				ft_set_skip_slot(cn_meta, parent_slot);
		}
	}
#endif

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
			struct cds_ft_inode_flag **skip_slot =
				ft_get_skip_slot(cn_meta, ft);
			if (skip_slot &&
			    ft_node_skip_compressed(*skip_slot))
				rcu_assign_pointer(*skip_slot,
					ft_skip_compressed_flag(
						new_child, cn->len));
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
			(const void *) cn_meta->parent);
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
	rcu_assign_pointer(*parent_slot, new_child);
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
	 * which — paired with this event — gives the consumer both
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
void ft_set_parent(struct cds_ft_inode_flag *child_nf,
		struct cds_ft_inode_flag *parent_nf,
		struct cds_ft_inode_flag **slot)
{
#ifndef FEATURE_FT_SKIP_COMPRESSED
	(void) slot;	/* only used to record the skip-compressed slot */
#endif
	if (!child_nf)
		return;
	FT_TP(set_parent, (const void *) child_nf, (const void *) parent_nf);
#ifdef FEATURE_FT_SKIP_COMPRESSED
	if (ft_node_skip_compressed(child_nf)) {
		struct cds_ft_compressed_node *cn =
			ft_skip_to_compressed(child_nf);
		struct cds_ft_metadata *cn_meta =
			cds_ft_item_to_metadata(
				(struct cds_ft_inode *) cn);
		rcu_assign_pointer(cn_meta->parent, parent_nf);
		ft_set_skip_slot(cn_meta, slot);
		return;
	}
	if (ft_node_compressed(child_nf)) {
		/*
		 * Plain COMPRESSED form (no SKIP_X wrap): typically
		 * arises when ft_publish_compressed gates SKIP-X off
		 * for a non-spec EXT child.  Maintain the cn's
		 * skip_slot_offset just like the SKIP_X branch above so
		 * later ft_publish_to_parent / chain-merge calls can
		 * recover the slot in cn's parent via
		 * ft_get_skip_slot.
		 */
		struct cds_ft_compressed_node *cn =
			ft_compressed_node_ptr(child_nf);
		struct cds_ft_metadata *cn_meta =
			cds_ft_item_to_metadata(
				(struct cds_ft_inode *) cn);
		rcu_assign_pointer(cn_meta->parent, parent_nf);
		ft_set_skip_slot(cn_meta, slot);
		return;
	}
#endif
	if (ft_node_external(child_nf)) {
		rcu_assign_pointer(
			((struct cds_ft_node *) child_nf)->prev,
			parent_nf);
		return;
	}
	rcu_assign_pointer(
		cds_ft_item_to_metadata(ft_node_ptr(child_nf))->parent,
		parent_nf);
}

#ifdef FT_ENABLE_TRACING
/*
 * Map a tagged cds_ft_inode_flag pointer to a symbolic node-kind
 * value (enum ft_tp_node_kind, defined in fractal-trie-internal.h
 * and exposed as the LTTng enum ft_tp_node_kind in src/cds_ft_tp.h).
 * Uses a single 16-entry compile-time dispatch table indexed by the
 * type-selecting bits of the pointer, so the runtime helper reduces
 * to a NULL/skip check plus one table load.
 */

/*
 * Single dispatch table indexed by the low 4 bits of a tagged
 * cds_ft_inode_flag pointer — exactly the bits that select the node
 * type:
 *   - bit 0      = FT_INTERNAL_MASK (1 = internal node)
 *   - bits 1..3  = type index (when internal) or class selector
 *                  (when not: 00=external, 01=compressed)
 *
 * Compressed nodes are 16-byte aligned, so bit 3 is guaranteed zero
 * for them.  External nodes need only 8-byte alignment (low 3 bits
 * = 000), so bit 3 may be either value — both [0b0000] and [0b1000]
 * map to EXTERNAL.
 *
 * Skip-compressed pointers are special-cased before the table lookup
 * (the only exception); the table itself is a pure pointer-bits
 * dispatch.
 *
 * Slot value 0 (FT_TP_NODE_NULL) doubles as a "no entry" sentinel
 * that resolves to FT_TP_NODE_UNKNOWN; NULL pointers are caught by
 * the explicit nf != NULL check before any table access.
 */
#define FT_TP_KIND_TABLE_MASK	0xFU

/* Pointer-bits encoding for internal-node type index `idx` (0..7). */
#define FT_TP_INTERNAL_TAG(idx)	\
	(((unsigned int) (idx) << FT_INTERNAL_BITS) | FT_INTERNAL_MASK)

/*
 * Compile-time pickers that map a single ft_types[] entry's sizing
 * parameters to an FT_TP_NODE_* constant.  All arguments are integer
 * constant expressions (sizing enums and order constants), so each
 * conditional collapses to one constant during compilation.
 */
#define FT_TP_KIND_P2L(ord) (					\
	(ord) == 5 ? FT_TP_NODE_P2L_32 :			\
	(ord) == 6 ? FT_TP_NODE_P2L_64 :			\
	(ord) == 7 ? FT_TP_NODE_P2L_128 :			\
	FT_TP_NODE_UNKNOWN)
#define FT_TP_KIND_P1L(ord) (					\
	(ord) == 7  ? FT_TP_NODE_P1L_128 :			\
	(ord) == 8  ? FT_TP_NODE_P1L_256 :			\
	(ord) == 9  ? FT_TP_NODE_P1L_512 :			\
	(ord) == 10 ? FT_TP_NODE_P1L_1024 :			\
	FT_TP_NODE_UNKNOWN)
#define FT_TP_KIND_PIGEON(ord) (				\
	(ord) == 10 ? FT_TP_NODE_PIGEON_1024 :			\
	(ord) == 11 ? FT_TP_NODE_PIGEON_2048 :			\
	FT_TP_NODE_UNKNOWN)

static const uint8_t ft_tp_kind_table[FT_TP_KIND_TABLE_MASK + 1] = {
	/* External: low 3 bits = 000; bit 3 unconstrained. */
	[0x0]				= FT_TP_NODE_EXTERNAL,
	[0x8]				= FT_TP_NODE_EXTERNAL,
	/* Compressed: low 3 bits = 010, bit 3 = 0 (16-byte aligned). */
	[FT_COMPRESSED_MASK]		= FT_TP_NODE_COMPRESSED,
	/*
	 * Internal nodes: bit 0 set, bits 1..3 = type index.  Each
	 * arch-specific ft_types[] is mapped via FT_TP_KIND_*().
	 */
#if (CAA_BITS_PER_LONG < 64)
	[FT_TP_INTERNAL_TAG(0)]		= FT_TP_KIND_P2L(5),
	[FT_TP_INTERNAL_TAG(1)]		= FT_TP_KIND_P2L(6),
	[FT_TP_INTERNAL_TAG(2)]		= FT_TP_KIND_P1L(7),
	[FT_TP_INTERNAL_TAG(3)]		= FT_TP_KIND_P1L(8),
	[FT_TP_INTERNAL_TAG(4)]		= FT_TP_KIND_P1L(9),
	[FT_TP_INTERNAL_TAG(5)]		= FT_TP_KIND_PIGEON(10),
	/* idx 6 = NODE_INDEX_NULL: never encoded in a pointer. */
#else
	[FT_TP_INTERNAL_TAG(0)]		= FT_TP_KIND_P2L(5),
	[FT_TP_INTERNAL_TAG(1)]		= FT_TP_KIND_P2L(6),
	[FT_TP_INTERNAL_TAG(2)]		= FT_TP_KIND_P2L(7),
	[FT_TP_INTERNAL_TAG(3)]		= FT_TP_KIND_P1L(8),
	[FT_TP_INTERNAL_TAG(4)]		= FT_TP_KIND_P1L(9),
	[FT_TP_INTERNAL_TAG(5)]		= FT_TP_KIND_P1L(10),
	[FT_TP_INTERNAL_TAG(6)]		= FT_TP_KIND_PIGEON(11),
#endif
};

uint16_t ft_tp_node_kind(struct cds_ft_inode_flag *nf)
{
	uint8_t kind;

	if (!nf)
		return FT_TP_NODE_NULL;
#ifdef FEATURE_FT_SKIP_COMPRESSED
	/*
	 * Skip-compression is orthogonal to the underlying node type: a
	 * skip pointer still points to a real child (external or
	 * internal).  Strip the skip-length bits so the dispatch table
	 * sees the underlying child's tag bits; the companion
	 * ft_tp_node_skip_len() field exposes the skip length separately.
	 */
	if (ft_node_skip_compressed(nf))
		nf = ft_skip_child_ptr(nf);
#endif
	kind = ft_tp_kind_table[(unsigned long) nf & FT_TP_KIND_TABLE_MASK];
	return kind ? kind : FT_TP_NODE_UNKNOWN;
}

/*
 * Return the number of key bytes the skip pointer covers (i.e. the
 * length of the skipped compressed path).  Zero means "not a skip
 * pointer".  The value fits in a uint16_t since FT_SKIP_LEN_MAX is at
 * most 255 on any supported architecture.
 */
uint16_t ft_tp_node_skip_len(struct cds_ft_inode_flag *nf)
{
#ifdef FEATURE_FT_SKIP_COMPRESSED
	if (!nf || !ft_node_skip_compressed(nf))
		return 0;
	return (uint16_t) ft_skip_len(nf);
#else
	(void) nf;
	return 0;
#endif
}
#endif /* FT_ENABLE_TRACING */


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
static inline_lookup
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
	unsigned int det_depth;			/* Depth of *det_nfp when captured. */
	bool pending;				/* Waiting to capture det_nfp. */
};

static
void ft_detach_descent_init(struct ft_detach_descent *dd,
		struct cds_ft *ft)
{
	ft_descent_init(&dd->d, ft);
	dd->det_nfp = NULL;
	dd->det_pfp = &ft->root;
	dd->det_depth = 0;
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
	 * Popcount node data[] starts with a presence bitmap, followed
	 * by the pointer table.  The allocator returns zeroed memory,
	 * which is the initial "no children" state (bitmap = 0 ⇒ all
	 * lookups return NULL; nr_child derived from popcount returns 0).
	 */
	if (ft_debug_counters()) {
		uatomic_inc(&ft->nr_nodes_allocated);
		uatomic_inc(&ft->nr_internal_alloc);
	}
	*_metadata = metadata;
	return p;
}

static
void free_cds_ft_node(struct cds_ft *ft, struct cds_ft_inode *node)
{
	struct cds_ft_metadata *metadata = cds_ft_item_to_metadata(node);

	cds_ft_free_item(ft, metadata);
	if (ft_debug_counters() && node) {
		uatomic_inc(&ft->nr_nodes_freed);
		uatomic_inc(&ft->nr_internal_freed);
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

	cds_ft_free_item_unpublished(ft, metadata);
	if (ft_debug_counters() && node) {
		uatomic_inc(&ft->nr_nodes_freed);
		uatomic_inc(&ft->nr_internal_freed);
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
	if (ft_debug_counters()) {
		uatomic_inc(&ft->nr_nodes_allocated);
		uatomic_inc(&ft->nr_compressed_alloc);
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

	FT_TP(compressed_free, (const void *) ft_compressed_node_flag(node));
	cds_ft_free_item(ft, metadata);
	if (ft_debug_counters() && node) {
		uatomic_inc(&ft->nr_nodes_freed);
		uatomic_inc(&ft->nr_compressed_freed);
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
	struct cds_ft_metadata *metadata =
		cds_ft_item_to_metadata((struct cds_ft_inode *) node);

	FT_TP(compressed_free, (const void *) ft_compressed_node_flag(node));
	cds_ft_free_item_unpublished(ft, metadata);
	if (ft_debug_counters() && node) {
		uatomic_inc(&ft->nr_nodes_freed);
		uatomic_inc(&ft->nr_compressed_freed);
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

static inline_lookup
uint8_t *align_ptr_size(uint8_t *ptr)
{
	return (uint8_t *) FT_ALIGN((unsigned long) ptr, sizeof(void *));
}

/*
 * Forward declarations for the 2-level popcount-bitmap node helpers.
 * Definitions follow the linear-helper block (after the
 * ft_dereference_acquire_* macros they depend on).
 */
static inline_lookup
uint8_t ft_popcount_2l_node_get_nr_child(const struct cds_ft_type *type,
		struct cds_ft_inode *node);
static inline_lookup
void ft_popcount_2l_node_get_ith_pos(const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		uint8_t i,
		uint8_t *v,
		struct cds_ft_inode_flag **iter);
static inline_lookup
struct cds_ft_inode_flag *ft_popcount_2l_node_get_direction(
		const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		int n, uint8_t *result_key,
		enum ft_direction dir);
static
int ft_popcount_2l_node_set_nth(const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		struct cds_ft_metadata *metadata,
		uint8_t n,
		struct cds_ft_inode_flag *child_node_flag,
		bool is_init);

/*
 * Forward declarations for the 1-level byte-popcount node helpers
 * (single 256-bit bitmap, used for type-4 / qp_28).
 */
static inline_lookup
uint8_t ft_popcount_1l_node_get_nr_child(const struct cds_ft_type *type,
		struct cds_ft_inode *node);
static inline_lookup
void ft_popcount_1l_node_get_ith_pos(const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		uint8_t i,
		uint8_t *v,
		struct cds_ft_inode_flag **iter);
static inline_lookup
struct cds_ft_inode_flag *ft_popcount_1l_node_get_direction(
		const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		int n, uint8_t *result_key,
		enum ft_direction dir);
static
int ft_popcount_1l_node_set_nth(const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		struct cds_ft_metadata *metadata,
		uint8_t n,
		struct cds_ft_inode_flag *child_node_flag,
		bool is_init);

/*
 * Derive nr_child by scanning values[] for the first sentinel.
 *
 * Padding and never-touched slots hold bytes equal to values[0]
 * (maintained by the first-insert memset).  Real key positions
 * i > 0 hold values distinct from values[0] (linear-node
 * distinctness).
 */

/*
 * FT_POPCOUNT class nr_child: dispatch to the popcount_1l or
 * popcount_2l layout helper.  Both count populated slots via
 * popcount of the bitmap (independent of the soft-delete pointer
 * accounting in metadata->nr_child).
 */
static inline_lookup
uint8_t ft_popcount_node_get_nr_child(const struct cds_ft_type *type,
		struct cds_ft_inode *node)
{
	assert(ft_type_is_popcount(type->type_class));
	if (type->popcount_2l)
		return ft_popcount_2l_node_get_nr_child(type, node);
	assert(type->popcount_1l);
	return ft_popcount_1l_node_get_nr_child(type, node);
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
 * Skip when the scanner's first read does NOT live at the start of
 * the node body (so prefetching `v` would fetch the wrong cache
 * line):
 * - FT_PIGEON: dense `pointers[256]`; the scanner reads
 *   `pointers[n]` at offset n*8.  First cache line covers only
 *   pointers[0..7], ~3% of random keys.
 *
 * Layouts whose first read IS at offset 0 of the node body benefit
 * from prefetch:
 * - popcount_2l (types 0/1/2 on 64-bit, 0/1 on 32-bit): bitmap
 *   header at offset 0.
 * - popcount_1l (types 3/4/5 on 64-bit, 2/3/4 on 32-bit): 32-byte
 *   bitmap header at offset 0.
 *
 * Encoding:
 *   popcount build: no skip test -- always prefetch.  Measurement
 *     on dns ft_specv (single-thread and 192-thread load-names)
 *     shows the type-equality check earns nothing on this build:
 *     PIGEON is rare on real workloads, the wrong-cache-line cost
 *     for the few PIGEON visits that do happen is tiny relative to
 *     the per-call cost of running the test on every other call,
 *     and the hot-path branch is one fewer instruction without it.
 */

static inline void ft_maybe_prefetch(const void *ptr)
{
	/*
	 * Experiment: drop the skip-compressed high-bit clear.
	 * __builtin_prefetch doesn't fault on non-canonical addresses
	 * (it's a hint that silently drops invalid loads), so feeding
	 * a skip-encoded pointer directly is safe.  The "wasted"
	 * prefetch on skip-encoded externals is acceptable; the
	 * common case (clean high bits) is unchanged.
	 */
	__builtin_prefetch(ptr);
}

/*
 * ft_dereference_prefetch: for tagged FT node pointers.  Uses
 * ft_maybe_prefetch to strip any skip-compressed length bits before
 * issuing the prefetch.
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
 * Per-caller prefetch hint for ft_node_get_nth_skip / ft_node_get_nth
 * and the underlying scanners.  Compile-time constant at each call
 * site — the branches inside ft_maybe_prefetch_hint fold away, leaving
 * at most a single prefetch per caller.
 *
 *   FT_PF_NONE:        no prefetch.
 *   FT_PF_DATA:        prefetch child's data (node body).  Right for
 *                      candidate lookup and non-skip exact lookup
 *                      that traverse the returned child's data next.
 *   FT_PF_META:        prefetch child's metadata cache line.  Right
 *                      for inequality / lookup_nth / count_keys,
 *                      which read metadata->external_nodes /
 *                      metadata->nr_keys before further descent.
 *                      For external children, prefetches the node
 *                      body instead (no FT metadata exists).  For
 *                      compressed children, skips — their own
 *                      handlers prefetch cn->child.
 *   FT_PF_BITMAP_META: same as META plus prefetches the bitmap
 *                      cache line for bitmap-bearing children
 *                      (used by ordered get_direction traversal).
 *
 * The item's alloc order is derived from the tag bits directly
 * (type_index + FT_INTERNAL_ORDER_MIN) without loading ft_types[],
 * and the node base address is derived from the tagged pointer via
 * alignment masking (bits below the order are all zero because
 * allocations are order-aligned).
 */
enum ft_pf_target {
	FT_PF_NONE,
	FT_PF_DATA,
	FT_PF_META,
	FT_PF_BITMAP_META,
};

static inline __attribute__((always_inline))
void ft_prefetch_child_meta(const void *ptr)
{
	unsigned long v = (unsigned long) ptr;

	if (!v)
		return;
	/*
	 * Experiment: skip the high-bit clear.  The tag-bit tests below
	 * only consult low bits, and the align_mask path's metadata
	 * prefetch is a hint that tolerates non-canonical addresses.
	 */
	if ((v & FT_INTERNAL_MASK) == 0) {
		/*
		 * External (bits 0-2 == 0): no FT metadata.  Prefetch the
		 * node body, which the META-hint caller typically reads
		 * next (user_data / ->next for the duplicate chain).
		 * Compressed children: their handlers prefetch their own
		 * targets; skip here.
		 */
		if ((v & FT_TAG_MASK) == 0)
			__builtin_prefetch((const void *) v);
		return;
	}
	{
		size_t order = ((v & FT_TYPE_MASK) >> FT_INTERNAL_BITS)
				+ FT_INTERNAL_ORDER_MIN;
		unsigned long align_mask = ~((1UL << order) - 1UL);
		void *node = (void *) (v & align_mask);

		__builtin_prefetch(cds_ft_item_to_metadata_fast(node, order));
	}
}

static inline __attribute__((always_inline))
void ft_prefetch_child_bitmap_meta(const void *ptr)
{
	unsigned long v = (unsigned long) ptr;

	if (!v)
		return;
	/* Experiment: skip the high-bit clear (see ft_prefetch_child_meta). */
	if ((v & FT_INTERNAL_MASK) == 0) {
		if ((v & FT_TAG_MASK) == 0)
			__builtin_prefetch((const void *) v);
		return;
	}
	{
		unsigned int type_index = (v & FT_TYPE_MASK) >> FT_INTERNAL_BITS;
		size_t order = type_index + FT_INTERNAL_ORDER_MIN;
		unsigned long align_mask = ~((1UL << order) - 1UL);
		void *node = (void *) (v & align_mask);

		__builtin_prefetch(cds_ft_item_to_metadata_fast(node, order));
		/*
		 * Only pigeon attaches a 32B bitmap in metadata; popcount
		 * tiers carry their bitmaps inline in the node body.  Pigeon
		 * is the last type index on both arches (5 on 32-bit,
		 * 6 on 64-bit) so a single arch-conditional literal suffices.
		 */
#if (CAA_BITS_PER_LONG < 64)
		if (type_index == 5)
#else
		if (type_index == 6)
#endif
			__builtin_prefetch(cds_ft_item_to_bitmap(node, order));
	}
}

static inline __attribute__((always_inline))
void ft_maybe_prefetch_hint(const void *ptr, enum ft_pf_target hint)
{
	switch (hint) {
	case FT_PF_NONE:
		break;
	case FT_PF_DATA:
		ft_maybe_prefetch(ptr);
		break;
	case FT_PF_META:
		ft_prefetch_child_meta(ptr);
		break;
	case FT_PF_BITMAP_META:
		ft_prefetch_child_bitmap_meta(ptr);
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
 * The order in which values and pointers are does does not matter: if
 * a value is missing, we return NULL. If a value is there, but its
 * associated pointers is still NULL, we return NULL too.
 */


/*
 * Runtime validation of ft_types[] popcount_1l class membership assumed
 * by the per-type dispatcher.  Called from cds_ft_group_create at init.
 * Not a static assert because ft_types[].max_linear_child is a struct
 * member read, which gcc doesn't treat as an integer constant
 * expression.
 */
static inline __attribute__((unused))
void ft_specialized_scan_layout_assert(void)
{
#if CAA_BITS_PER_LONG >= 64
	assert(FT_ALIGN(ft_types[0].max_linear_child, sizeof(void *)) == 8);
	assert(FT_ALIGN(ft_types[1].max_linear_child, sizeof(void *)) == 8);
	assert(FT_ALIGN(ft_types[2].max_linear_child, sizeof(void *)) == 16);
	assert(FT_ALIGN(ft_types[3].max_linear_child, sizeof(void *)) == 32);
	assert(ft_types[4].popcount_1l);
	assert(ft_types[5].popcount_1l);
	assert(ft_type_is_pigeon(ft_types[6].type_class));
#else
	assert(ft_types[0].popcount_2l);
	assert(ft_types[1].popcount_2l);
	assert(ft_types[2].popcount_1l);
	assert(ft_types[3].popcount_1l);
	assert(ft_types[4].popcount_1l);
	assert(ft_type_is_pigeon(ft_types[5].type_class));
#endif
}

/* Forward declarations for the per-type popcount scanners. */
static inline_lookup
struct cds_ft_inode_flag *ft_popcount_2l_scan_16_16_max_3(
		struct cds_ft_inode *node,
		struct cds_ft_inode_flag ***node_flag_ptr,
		uint8_t n, enum ft_pf_target pf_hint);
static inline_lookup
struct cds_ft_inode_flag *ft_popcount_2l_scan_16_16_max_5(
		struct cds_ft_inode *node,
		struct cds_ft_inode_flag ***node_flag_ptr,
		uint8_t n, enum ft_pf_target pf_hint);
static inline_lookup
struct cds_ft_inode_flag *ft_popcount_2l_scan_32_8(
		struct cds_ft_inode *node,
		struct cds_ft_inode_flag ***node_flag_ptr,
		uint8_t n, enum ft_pf_target pf_hint);
static inline_lookup
struct cds_ft_inode_flag *ft_popcount_2l_scan_64_4(
		struct cds_ft_inode *node,
		struct cds_ft_inode_flag ***node_flag_ptr,
		uint8_t n, enum ft_pf_target pf_hint);
static inline_lookup
struct cds_ft_inode_flag *ft_popcount_1l_scan_28(
		struct cds_ft_inode *node,
		struct cds_ft_inode_flag ***node_flag_ptr,
		uint8_t n, enum ft_pf_target pf_hint);

/*
 * FT_POPCOUNT class get_direction: dispatch to popcount_1l or
 * popcount_2l layout helper.
 */
static inline_lookup
struct cds_ft_inode_flag *ft_popcount_node_get_direction(
		const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		int n, uint8_t *result_key,
		enum ft_direction dir)
{
	assert(ft_type_is_popcount(type->type_class));
	if (type->popcount_2l)
		return ft_popcount_2l_node_get_direction(
				type, node, n, result_key, dir);
	assert(type->popcount_1l);
	return ft_popcount_1l_node_get_direction(
			type, node, n, result_key, dir);
}

/*
 * FT_POPCOUNT class get_ith_pos: dispatch to popcount_1l or
 * popcount_2l layout helper.  Returns the byte value v and
 * the (acquire-loaded) child pointer for the i-th populated slot in
 * popcount order.  Stub in baseline builds.
 */
static inline_lookup
void ft_popcount_node_get_ith_pos(const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		uint8_t i,
		uint8_t *v,
		struct cds_ft_inode_flag **iter)
{
	assert(ft_type_is_popcount(type->type_class));
	assert(i < ft_popcount_node_get_nr_child(type, node));
	if (type->popcount_2l) {
		ft_popcount_2l_node_get_ith_pos(type, node, i, v, iter);
		return;
	}
	assert(type->popcount_1l);
	ft_popcount_1l_node_get_ith_pos(type, node, i, v, iter);
}

/*
 * Generic 2-level popcount-bitmap node header (4+4 byte split).
 *
 * Used by the scan_16_16_max_<N> family (max_lc=3 and max_lc=5).
 * Other 2L variants use flat layouts and access node->data
 * directly without this struct (scan_32_8 / scan_64_4).
 *
 *   root_bm   : 16-bit; bit i set iff some child key has high
 *               nibble == i (high 4 bits of the byte)
 *   sub_bm[k] : 16-bit; presence of the low nibble for the k-th
 *               popcount-ordered hi position
 *
 * Pointer slot index for byte n = (hi<<4)|lo equals popcount of all
 * bits set below the (hi, lo) position across the concatenated
 * sub_bm[] in popcount order.  This is the rank of (hi,lo) among
 * populated entries.
 *
 * For order-5 nodes (32 B, max_linear_child = 3):
 *   header = 8 B (root_bm + 3 sub_bm)
 *   ptrs   = 24 B (3 child pointers)
 *
 * The lookup is branch-free past the two presence tests and uses
 * portable __builtin_popcount{,ll} -- no SIMD intrinsics, so the
 * same code compiles for any architecture with a popcount intrinsic
 * (including software-emulated popcount on older targets).
 */
struct ft_popcount_2l_header {
	uint16_t root_bm;
	uint16_t sub_bm[];	/* length = type->max_linear_child */
};

static inline_lookup
unsigned int ft_popcount_2l_header_bytes(unsigned int max_lc)
{
	if (max_lc == 6)
		return 12;	/* 5+3 split: 4B root + 8B packed_bms */
	if (max_lc == 12 || max_lc == 14 || max_lc == 16)
		return 16;	/* 6+2 FLAT: 8B root + 8B packed_bms */
	return (unsigned int) (sizeof(uint16_t) * (1U + max_lc));
}

static inline_lookup
struct cds_ft_inode_flag **ft_popcount_2l_pointers(
		struct cds_ft_inode *node, const struct cds_ft_type *type)
{
	unsigned int max_lc = type->max_linear_child;
	unsigned int byte_offset;

	/*
	 * Flat scan_32_8 (12 B hdr + 4 B pad) and scan_64_4 (16 B hdr)
	 * scanners hardcode the ptr table at node+16.  max_lc=12 and 16
	 * reuse the scan_64_4 layout on 32-bit -- the 16-slot bitmap
	 * caps the safe distinct-hi count at 16 (cf. scan_32_8's 8-slot
	 * cap, which is too tight for 12+ children in the worst case).
	 * max_lc=16 lives in a 128 B order-7 node with 48 B trailing
	 * slack; the bitmap cap is binding, not the node size.
	 */
	if (max_lc == 6 || max_lc == 12 || max_lc == 14 || max_lc == 16)
		byte_offset = 16;
	else
		byte_offset = FT_ALIGN(
			ft_popcount_2l_header_bytes(max_lc),
			sizeof(void *));
	return (struct cds_ft_inode_flag **)
		((uint8_t *) node + byte_offset);
}

/*
 * Lookup primitive: 2-level popcount, max_linear_child = 3.
 *
 * One 64-bit load brings root_bm + sub_bm[0..2] into a single
 * register: low 16 bits = root_bm, upper 48 bits = sub_bm[0..2]
 * concatenated in popcount order.  Two presence tests (high then
 * low nibble) guard a single popcount-prefix-sum into the pointer
 * array.
 */
static inline_lookup
struct cds_ft_inode_flag *ft_popcount_2l_scan_16_16_max_3(
		struct cds_ft_inode *node,
		struct cds_ft_inode_flag ***node_flag_ptr,
		uint8_t n, enum ft_pf_target pf_hint)
{
	uint64_t bms = *(const uint64_t *) &node->data[0];
	uint16_t root = (uint16_t) bms;
	uint64_t subs = bms >> 16;
	unsigned int hi = (unsigned int) n >> 4;
	unsigned int lo = (unsigned int) n & 0xFU;
	unsigned int slot1, bit_pos, ptr_idx;
	struct cds_ft_inode_flag **pointers;

	/* 1. Root check (high nibble present?). */
	if (caa_unlikely(!((root >> hi) & 1U)))
		goto not_found;

	slot1 = (unsigned int) __builtin_popcount(root & ((1U << hi) - 1U));

	/*
	 * Absolute bit position of (hi, lo) in the 48-bit subs concat.
	 * (slot1 << 4) | lo is identical to slot1 * 16 + lo because
	 * lo < 16, but it folds the multiply+add into a single shift+OR
	 * dependency.
	 */
	bit_pos = (slot1 << 4) | lo;

	/* 2. Sub-bitmap check (low nibble present in this hi's slot?). */
	if (caa_unlikely(!((subs >> bit_pos) & 1ULL)))
		goto not_found;

	/* 3. Pointer index = rank of (hi, lo) among populated entries. */
	ptr_idx = (unsigned int) __builtin_popcountll(
			subs & ((1ULL << bit_pos) - 1ULL));
	pointers = (struct cds_ft_inode_flag **) ((uint8_t *) node + 8);
	if (caa_unlikely(node_flag_ptr))
		*node_flag_ptr = &pointers[ptr_idx];
	return ft_dereference_acquire_prefetch_hint(pointers[ptr_idx], pf_hint);

not_found:
	if (caa_unlikely(node_flag_ptr))
		*node_flag_ptr = NULL;
	return NULL;
}

/*
 * Lookup primitive: 2-level popcount, max_linear_child = 5 (32-bit only).
 *
 * Same generic 2L layout as scan_16_16_max_3, just two more slots:
 *   [0..1]   root_bm        (u16)
 *   [2..11]  sub_bm[0..4]   (5 x u16)
 *   [12..31] 5 x 4-byte pointers
 *
 * 32-bit-native: 16-bit loads for root_bm and sub_bm[], 32-bit
 * popcount.  No 64-bit ops on the hot path (which would compile to
 * register-pair handling on i386).  The prefix sum runs a short
 * loop of 0..4 iterations over sub_bm[0..slot1-1]; the compiler can
 * predict / unroll given the bounded count.
 *
 * Trade-off vs scan_16_16_max_3 (max_lc=3): one variable-length
 * prefix loop in exchange for max_lc 3 -> 5 (recovers the 12 B of
 * trailing waste per 32 B node).
 */
static inline_lookup
struct cds_ft_inode_flag *ft_popcount_2l_scan_16_16_max_5(
		struct cds_ft_inode *node,
		struct cds_ft_inode_flag ***node_flag_ptr,
		uint8_t n, enum ft_pf_target pf_hint)
{
	uint16_t root = *(const uint16_t *) &node->data[0];
	const uint16_t *subs = (const uint16_t *) (node->data + 2);
	unsigned int hi = (unsigned int) n >> 4;
	unsigned int lo = (unsigned int) n & 0xFU;
	unsigned int slot1, ptr_idx, k;
	uint16_t sub;
	struct cds_ft_inode_flag **pointers;

	if (caa_unlikely(!((root >> hi) & 1U)))
		goto not_found;

	slot1 = (unsigned int) __builtin_popcount(
			root & (uint16_t) ((1U << hi) - 1U));
	sub = subs[slot1];

	if (caa_unlikely(!((sub >> lo) & 1U)))
		goto not_found;

	ptr_idx = (unsigned int) __builtin_popcount(
			sub & (uint16_t) ((1U << lo) - 1U));
	for (k = 0; k < slot1; k++)
		ptr_idx += (unsigned int) __builtin_popcount(subs[k]);

	pointers = (struct cds_ft_inode_flag **) ((uint8_t *) node + 12);
	if (caa_unlikely(node_flag_ptr))
		*node_flag_ptr = &pointers[ptr_idx];
	return ft_dereference_acquire_prefetch_hint(pointers[ptr_idx], pf_hint);

not_found:
	if (caa_unlikely(node_flag_ptr))
		*node_flag_ptr = NULL;
	return NULL;
}

/*
 * Lookup primitive: per-slot flat popcount with 5+3 byte split,
 * max_linear_child = 6.
 *
 * Layout in the 64-byte order-6 node (FLAT layout, NOT shared with
 * scan_16_16_max_3 which uses the 4+4 sub_bm[] layout):
 *   [0..3]   root_bm (32-bit; bit set per populated hi = n >> 3)
 *   [4..11]  packed_bms (8 bytes = 8 x 8-bit sub_bm; only first 6 used)
 *   [12..15] padding
 *   [16..63] 6 x cds_ft_inode_flag *
 *
 * The byte split is 5-bit hi + 3-bit lo (departs from the QP 4+4
 * convention to fit all sub_bms in one u64).  packed_bms is a flat
 * 64-bit view: bit at position (slot1*8 + lo) is set iff (hi, lo)
 * is a populated child.  The single packed bitmap collapses prior
 * + chunk-select + sub_bm popcount into one popcount(bms & mask).
 *
 * Hot path:
 *   slot1   = popcount(root & ((1<<hi)-1))
 *   p       = (slot1 << 3) | lo
 *   ptr_idx = popcount(bms & ((1<<p)-1))
 *
 * No chunk-select cmov, no per-slot prior cache: a single packed
 * bitmap suffices because max_lc=6 fits in 48 bits (well within u64).
 */
static inline_lookup
struct cds_ft_inode_flag *ft_popcount_2l_scan_32_8(
		struct cds_ft_inode *node,
		struct cds_ft_inode_flag ***node_flag_ptr,
		uint8_t n, enum ft_pf_target pf_hint)
{
	uint32_t root = *(const uint32_t *) &node->data[0];
	uint64_t bms  = *(const uint64_t *) &node->data[4];
	unsigned int hi = (unsigned int) n >> 3;
	unsigned int lo = (unsigned int) n & 0x7U;
	unsigned int slot1, p, ptr_idx;
	struct cds_ft_inode_flag **pointers;

	if (caa_unlikely(!((root >> hi) & 1U)))
		goto not_found;

	slot1 = (unsigned int) __builtin_popcount(root & ((1U << hi) - 1U));
	p = (slot1 << 3) | lo;

	if (caa_unlikely(!((bms >> p) & 1ULL)))
		goto not_found;

	ptr_idx = (unsigned int) __builtin_popcountll(
			bms & ((1ULL << p) - 1ULL));

	pointers = (struct cds_ft_inode_flag **) ((uint8_t *) node + 16);
	if (caa_unlikely(node_flag_ptr))
		*node_flag_ptr = &pointers[ptr_idx];
	return ft_dereference_acquire_prefetch_hint(pointers[ptr_idx], pf_hint);

not_found:
	if (caa_unlikely(node_flag_ptr))
		*node_flag_ptr = NULL;
	return NULL;
}

/*
 * Lookup primitive: flat packed sub_bms with 6+2 byte split,
 * max_linear_child = 14.
 *
 * Layout in the 128-byte order-7 node (6+2 FLAT, not shared with
 * scan_16_16_max_3 / scan_32_8):
 *   [0..7]    root_bm (u64; bit set per populated hi = n >> 2)
 *   [8..15]   packed_bms (u64; 14 x 4-bit sub_bms, 56 bits used)
 *   [16..127] 14 x cds_ft_inode_flag *
 *
 * The 4-bit sub_bm width comes from lo = n & 3 (4 entries per chunk).
 * 14 chunks * 4 bits = 56 bits, fits a u64 with 8 bits to spare.
 * 14 is the largest count keeping the node within 128B (16 byte
 * header + 14 * 8 = 128 exactly).
 *
 * Both root and packed_bms load at constant offsets (no data-dep
 * load), and a single popcountll on (packed_bms & mask) yields
 * ptr_idx -- no chunk-select cmov, no per-slot prior cache to refresh.
 *
 * Hot path:
 *   slot1   = popcount(root & ((1<<hi)-1))
 *   p       = (slot1 << 2) | lo
 *   ptr_idx = popcount(packed_bms & ((1<<p)-1))
 */
static inline_lookup
struct cds_ft_inode_flag *ft_popcount_2l_scan_64_4(
		struct cds_ft_inode *node,
		struct cds_ft_inode_flag ***node_flag_ptr,
		uint8_t n, enum ft_pf_target pf_hint)
{
	uint64_t root = *(const uint64_t *) &node->data[0];
	uint64_t bms  = *(const uint64_t *) &node->data[8];
	unsigned int hi = (unsigned int) n >> 2;
	unsigned int lo = (unsigned int) n & 0x3U;
	unsigned int slot1, p, ptr_idx;
	struct cds_ft_inode_flag **pointers;

	if (caa_unlikely(!((root >> hi) & 1ULL)))
		goto not_found;

	slot1 = (unsigned int) __builtin_popcountll(
			root & ((1ULL << hi) - 1ULL));
	p = (slot1 << 2) | lo;

	if (caa_unlikely(!((bms >> p) & 1ULL)))
		goto not_found;

	ptr_idx = (unsigned int) __builtin_popcountll(
			bms & ((1ULL << p) - 1ULL));

	pointers = (struct cds_ft_inode_flag **) ((uint8_t *) node + 16);
	if (caa_unlikely(node_flag_ptr))
		*node_flag_ptr = &pointers[ptr_idx];
	return ft_dereference_acquire_prefetch_hint(pointers[ptr_idx], pf_hint);

not_found:
	if (caa_unlikely(node_flag_ptr))
		*node_flag_ptr = NULL;
	return NULL;
}

/*
 * Total populated entries = sum of popcount(sub_bm[k]) for k where
 * sub_bm[k] is in use.  Since sub_bm[k] for k >= popcount(root_bm)
 * is unused (held at zero), summing all sub_bm values is safe.
 *
 * For max_linear_child = 3, the three sub bitmaps fit in 6 bytes and
 * a single qword load lets us read them all alongside root_bm.
 */
static inline_lookup
uint8_t ft_popcount_2l_node_get_nr_child(const struct cds_ft_type *type,
		struct cds_ft_inode *node)
{
	unsigned int max_lc = type->max_linear_child;

	if (max_lc == 6) {
		/* Flat 5+3 layout: nr_child = popcount of packed_bms. */
		uint64_t bms = *(const uint64_t *) &node->data[4];
		return (uint8_t) __builtin_popcountll(bms);
	}
	if (max_lc == 12 || max_lc == 14 || max_lc == 16) {
		/* Flat 6+2 layout: nr_child = popcount of packed_bms. */
		uint64_t bms = *(const uint64_t *) &node->data[8];
		return (uint8_t) __builtin_popcountll(bms);
	}
	{
		struct ft_popcount_2l_header *hdr =
			(struct ft_popcount_2l_header *) &node->data[0];
		unsigned int total = 0, k;

		for (k = 0; k < max_lc; k++)
			total += (unsigned int) __builtin_popcount(hdr->sub_bm[k]);
		return (uint8_t) total;
	}
}

/*
 * Locate the i-th populated entry in (hi,lo)-ascending order and
 * return both the encoded byte v = (hi<<4)|lo and the child pointer.
 * Walks sub_bm[] in popcount order, accumulating bit counts; within
 * the sub_bm that contains the i-th entry, finds the relative bit
 * position via masked popcount expansion.
 */
static inline_lookup
void ft_popcount_2l_node_get_ith_pos(const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		uint8_t i,
		uint8_t *v,
		struct cds_ft_inode_flag **iter)
{
	unsigned int max_lc = type->max_linear_child;
	struct cds_ft_inode_flag **pointers = ft_popcount_2l_pointers(node, type);

	if (max_lc == 6) {
		/* Flat 5+3 layout: find i-th set bit in packed_bms. */
		uint32_t root = *(const uint32_t *) &node->data[0];
		uint64_t bms  = *(const uint64_t *) &node->data[4];
		unsigned int slot1, lo, hi, p, j;
		uint64_t walk = bms;
		uint32_t r_walk;

		for (j = 0; j < i; j++)
			walk &= walk - 1ULL;
		p = (unsigned int) __builtin_ctzll(walk);
		slot1 = p >> 3;
		lo = p & 0x7U;

		r_walk = root;
		for (j = 0; j < slot1; j++)
			r_walk &= r_walk - 1U;
		hi = (unsigned int) __builtin_ctz(r_walk);

		*v = (uint8_t) ((hi << 3) | lo);
		*iter = ft_dereference_acquire(pointers[i]);
		return;
	}
	if (max_lc == 12 || max_lc == 14 || max_lc == 16) {
		/* Flat 6+2 layout: find i-th set bit in packed_bms. */
		uint64_t root = *(const uint64_t *) &node->data[0];
		uint64_t bms  = *(const uint64_t *) &node->data[8];
		unsigned int slot1, lo, hi, p, j;
		uint64_t walk = bms, r_walk;

		for (j = 0; j < i; j++)
			walk &= walk - 1ULL;
		p = (unsigned int) __builtin_ctzll(walk);
		slot1 = p >> 2;
		lo = p & 0x3U;

		r_walk = root;
		for (j = 0; j < slot1; j++)
			r_walk &= r_walk - 1ULL;
		hi = (unsigned int) __builtin_ctzll(r_walk);

		*v = (uint8_t) ((hi << 2) | lo);
		*iter = ft_dereference_acquire(pointers[i]);
		return;
	}
	{
		struct ft_popcount_2l_header *hdr =
			(struct ft_popcount_2l_header *) &node->data[0];
		uint16_t root = hdr->root_bm;
		unsigned int target = i, k, hi, lo;
		uint16_t sub = 0, root_walk;
		unsigned int sub_pop = 0;

		for (k = 0; k < max_lc; k++) {
			sub = hdr->sub_bm[k];
			sub_pop = (unsigned int) __builtin_popcount(sub);
			if (target < sub_pop)
				break;
			target -= sub_pop;
		}
		assert(k < max_lc);
		/* hi = position of the k-th set bit in root_bm. */
		root_walk = root;
		hi = 0;
		for (unsigned int j = 0; j < k; j++) {
			unsigned int b = (unsigned int) __builtin_ctz(root_walk);
			root_walk &= root_walk - 1U;
			hi = b;
		}
		hi = (unsigned int) __builtin_ctz(root_walk);
		/* lo = position of the target-th set bit in sub. */
		{
			uint16_t s = sub;
			for (unsigned int j = 0; j < target; j++)
				s &= (uint16_t)(s - 1U);
			lo = (unsigned int) __builtin_ctz(s);
		}
		*v = (uint8_t) ((hi << 4) | lo);
		*iter = ft_dereference_acquire(pointers[i]);
	}
}

/*
 * Find the leftmost (FT_LEFT: largest v < n) or rightmost (FT_RIGHT:
 * smallest v > n) populated entry.  Walks all populated entries
 * since max_linear_child is small (3 or 6); a SWAR/bitmap-arithmetic
 * variant could be added later if profiling shows this on a hot path.
 */
static inline_lookup
struct cds_ft_inode_flag *ft_popcount_2l_node_get_direction(
		const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		int n, uint8_t *result_key,
		enum ft_direction dir)
{
	uint8_t nr_child;
	struct cds_ft_inode_flag *match_ptr = NULL;
	int match_v;
	unsigned int i;

	assert(dir == FT_LEFT || dir == FT_RIGHT);

	nr_child = ft_popcount_2l_node_get_nr_child(type, node);
	cmm_smp_rmb();	/* read counts/bitmaps before pointers */

	if (dir == FT_LEFT)
		match_v = -1;
	else
		match_v = FT_ENTRY_PER_NODE;

	for (i = 0; i < nr_child; i++) {
		struct cds_ft_inode_flag *ptr;
		uint8_t v;

		ft_popcount_2l_node_get_ith_pos(type, node, (uint8_t) i, &v, &ptr);
		if (!ptr)
			continue;
		if (dir == FT_LEFT) {
			if ((int) v < n && (int) v > match_v) {
				match_v = v;
				match_ptr = ptr;
				if (match_v == n - 1)
					break;
			}
		} else {
			if ((int) v > n && (int) v < match_v) {
				match_v = v;
				match_ptr = ptr;
				if (match_v == n + 1)
					break;
			}
		}
	}
	if (!match_ptr)
		return NULL;
	assert(match_v >= 0 && match_v < FT_ENTRY_PER_NODE);
	*result_key = (uint8_t) match_v;
	return match_ptr;
}

/*
 * Insert (n, child_node_flag) into a freshly-allocated, unpublished
 * QP node.  Called only from the recompact path: the new node is
 * not yet wired into the trie, so direct in-place mutation (shifts
 * of sub_bm[] and pointers[]) is race-free.
 *
 * is_init=true zeroes the bitmap header and writes pointers[0] for
 * the first entry.  Subsequent calls with is_init=false fold the new
 * entry into the existing layout, shifting sub_bm slots and pointer
 * slots as needed to keep popcount order.
 *
 * Returns 0 on success.  Asserts caller-precondition violations
 * (full node, duplicate key) since recompact paths are responsible
 * for choosing the right destination type.
 */
static
int ft_popcount_2l_node_set_nth(const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		struct cds_ft_metadata *metadata,
		uint8_t n,
		struct cds_ft_inode_flag *child_node_flag,
		bool is_init)
{
	struct cds_ft_inode_flag **pointers = ft_popcount_2l_pointers(node, type);
	unsigned int max_lc = type->max_linear_child;

	if (max_lc == 6) {
		/* Flat 5+3 layout. */
		unsigned int hi = (unsigned int) n >> 3;
		unsigned int lo = (unsigned int) n & 0x7U;
		uint32_t root;
		uint64_t bms;
		unsigned int slot1, p, ptr_idx, nr_child, k;

		if (is_init) {
			/* Alloc returned zeroed memory; just write new state. */
			*(uint32_t *) &node->data[0] = (uint32_t) (1U << hi);
			*(uint64_t *) &node->data[4] = (uint64_t) (1ULL << lo);
			pointers[0] = child_node_flag;
			metadata->nr_child++;
			return 0;
		}

		root = *(const uint32_t *) &node->data[0];
		bms  = *(const uint64_t *) &node->data[4];
		nr_child = (unsigned int) __builtin_popcountll(bms);
		assert(nr_child < max_lc);

		slot1 = (unsigned int) __builtin_popcount(root & ((1U << hi) - 1U));
		p = (slot1 << 3) | lo;

		if (!((root >> hi) & 1U)) {
			/* New hi: shift packed_bms bytes [slot1..) up by one byte. */
			uint64_t low_mask = (slot1 == 0) ? 0ULL
				: ((1ULL << (slot1 * 8)) - 1ULL);
			uint64_t low_part = bms & low_mask;
			uint64_t high_part = (bms & ~low_mask) << 8;
			uint64_t new_byte = ((uint64_t)(1U << lo)) << (slot1 * 8);
			bms = low_part | new_byte | high_part;
			root |= (1U << hi);
		} else {
			/* Existing hi: OR new lo bit into byte at slot1 position. */
			assert(!((bms >> p) & 1ULL));	/* duplicate would be a bug */
			bms |= ((uint64_t)(1U << lo)) << (slot1 * 8);
		}

		ptr_idx = (unsigned int) __builtin_popcountll(
				bms & ((1ULL << p) - 1ULL));
		/* Shift pointers[ptr_idx..nr_child) up. */
		for (k = nr_child; k > ptr_idx; k--)
			pointers[k] = pointers[k - 1];
		pointers[ptr_idx] = child_node_flag;

		*(uint32_t *) &node->data[0] = root;
		*(uint64_t *) &node->data[4] = bms;
		metadata->nr_child++;
		return 0;
	}

	if (max_lc == 12 || max_lc == 14 || max_lc == 16) {
		/* Flat 6+2 layout. */
		unsigned int hi = (unsigned int) n >> 2;
		unsigned int lo = (unsigned int) n & 0x3U;
		uint64_t root, bms;
		unsigned int slot1, p, ptr_idx, nr_child, k;

		if (is_init) {
			*(uint64_t *) &node->data[0] = 1ULL << hi;
			*(uint64_t *) &node->data[8] = (uint64_t) (1ULL << lo);
			pointers[0] = child_node_flag;
			metadata->nr_child++;
			return 0;
		}

		root = *(const uint64_t *) &node->data[0];
		bms  = *(const uint64_t *) &node->data[8];
		nr_child = (unsigned int) __builtin_popcountll(bms);
		assert(nr_child < max_lc);

		slot1 = (unsigned int) __builtin_popcountll(
				root & ((1ULL << hi) - 1ULL));
		p = (slot1 << 2) | lo;

		if (!((root >> hi) & 1ULL)) {
			/* New hi: shift packed_bms nibbles [slot1..) up by one nibble (4 bits). */
			uint64_t low_mask = (slot1 == 0) ? 0ULL
				: ((1ULL << (slot1 * 4)) - 1ULL);
			uint64_t low_part = bms & low_mask;
			uint64_t high_part = (bms & ~low_mask) << 4;
			uint64_t new_nibble = ((uint64_t)(1U << lo)) << (slot1 * 4);
			bms = low_part | new_nibble | high_part;
			root |= 1ULL << hi;
		} else {
			/* Existing hi: OR new lo bit into slot1's nibble. */
			assert(!((bms >> p) & 1ULL));	/* duplicate would be a bug */
			bms |= ((uint64_t)(1U << lo)) << (slot1 * 4);
		}

		ptr_idx = (unsigned int) __builtin_popcountll(
				bms & ((1ULL << p) - 1ULL));
		/* Shift pointers[ptr_idx..nr_child) up. */
		for (k = nr_child; k > ptr_idx; k--)
			pointers[k] = pointers[k - 1];
		pointers[ptr_idx] = child_node_flag;

		*(uint64_t *) &node->data[0] = root;
		*(uint64_t *) &node->data[8] = bms;
		metadata->nr_child++;
		return 0;
	}

	{
	struct ft_popcount_2l_header *hdr =
		(struct ft_popcount_2l_header *) &node->data[0];
	unsigned int hi = (unsigned int) n >> 4;
	unsigned int lo = (unsigned int) n & 0xFU;
	uint16_t root, sub;
	unsigned int slot1, ptr_idx, nr_child, k;

	if (is_init) {
		memset(hdr, 0, ft_popcount_2l_header_bytes(max_lc));
		hdr->root_bm = (uint16_t) (1U << hi);
		hdr->sub_bm[0] = (uint16_t) (1U << lo);
		pointers[0] = child_node_flag;
		metadata->nr_child++;
		return 0;
	}

	root = hdr->root_bm;
	nr_child = ft_popcount_2l_node_get_nr_child(type, node);
	assert(nr_child < max_lc);
	slot1 = (unsigned int) __builtin_popcount(root & ((1U << hi) - 1U));

	if (!((root >> hi) & 1U)) {
		/*
		 * New hi: shift sub_bm[slot1..nr_used) one slot up to
		 * make room.  nr_used = popcount(root) -- the number
		 * of currently-occupied sub_bm slots.
		 */
		unsigned int nr_used = (unsigned int) __builtin_popcount(root);
		for (k = nr_used; k > slot1; k--)
			hdr->sub_bm[k] = hdr->sub_bm[k - 1];
		hdr->sub_bm[slot1] = 0;
		hdr->root_bm = (uint16_t) (root | (1U << hi));
	}
	sub = hdr->sub_bm[slot1];
	assert(!((sub >> lo) & 1U));	/* duplicate key would be a bug */

	/* Compute pointer insertion index across full layout. */
	{
		uint64_t subs_below = 0;
		for (k = 0; k < slot1; k++)
			subs_below += (uint64_t) __builtin_popcount(hdr->sub_bm[k]);
		subs_below += (uint64_t) __builtin_popcount(
				(unsigned int) sub & ((1U << lo) - 1U));
		ptr_idx = (unsigned int) subs_below;
	}

	/* Shift pointers[ptr_idx..nr_child) up. */
	for (k = nr_child; k > ptr_idx; k--)
		pointers[k] = pointers[k - 1];
	pointers[ptr_idx] = child_node_flag;

	hdr->sub_bm[slot1] = (uint16_t) (sub | (1U << lo));
	metadata->nr_child++;
	return 0;
	}
}

/*
 * 1-level byte-popcount node header.
 *
 * One bit per possible byte value (0..255).  Used by larger
 * popcount nodes (orders 8/9/10) where the 2-level popcount_2l
 * layout would not fit alongside the pointer array.
 *
 * Layout in the order-8 (256-byte) node:
 *   [0..31]   bm[4] (256-bit bitmap, 4 x uint64_t)
 *   [32..255] 28 x cds_ft_inode_flag *
 */
struct ft_popcount_1l_header {
	uint64_t bm[4];
};

static inline_lookup
struct cds_ft_inode_flag **ft_popcount_1l_pointers(
		struct cds_ft_inode *node)
{
	return (struct cds_ft_inode_flag **) ((uint8_t *) node + 32);
}

/*
 * Lookup primitive: 1-level byte popcount, 256-bit bitmap.
 *
 * Four 64-bit loads bring the entire bitmap.  word_idx selects which
 * 64-bit chunk holds bit n; ptr_idx is the popcount of all chunks
 * before plus the masked prefix of the current chunk.  All four
 * popcountll calls can issue in parallel; the dependency chain is
 * load -> popcount -> add chain.
 */
static inline_lookup
struct cds_ft_inode_flag *ft_popcount_1l_scan_28(
		struct cds_ft_inode *node,
		struct cds_ft_inode_flag ***node_flag_ptr,
		uint8_t n, enum ft_pf_target pf_hint)
{
	uint64_t w0 = *(const uint64_t *) &node->data[0];
	uint64_t w1 = *(const uint64_t *) &node->data[8];
	uint64_t w2 = *(const uint64_t *) &node->data[16];
	uint64_t w3 = *(const uint64_t *) &node->data[24];
	unsigned int word_idx = (unsigned int) n >> 6;
	unsigned int bit_idx = (unsigned int) n & 63U;
	uint64_t word, prefix_mask;
	unsigned int ptr_idx;
	struct cds_ft_inode_flag **pointers;

	switch (word_idx) {
	case 0:
		word = w0;
		ptr_idx = 0;
		break;
	case 1:
		word = w1;
		ptr_idx = (unsigned int) __builtin_popcountll(w0);
		break;
	case 2:
		word = w2;
		ptr_idx = (unsigned int) __builtin_popcountll(w0)
			+ (unsigned int) __builtin_popcountll(w1);
		break;
	default:	/* case 3 */
		word = w3;
		ptr_idx = (unsigned int) __builtin_popcountll(w0)
			+ (unsigned int) __builtin_popcountll(w1)
			+ (unsigned int) __builtin_popcountll(w2);
		break;
	}

	if (caa_unlikely(!((word >> bit_idx) & 1ULL)))
		goto not_found;

	prefix_mask = (1ULL << bit_idx) - 1ULL;
	ptr_idx += (unsigned int) __builtin_popcountll(word & prefix_mask);

	pointers = (struct cds_ft_inode_flag **) ((uint8_t *) node + 32);
	if (caa_unlikely(node_flag_ptr))
		*node_flag_ptr = &pointers[ptr_idx];
	return ft_dereference_acquire_prefetch_hint(pointers[ptr_idx], pf_hint);

not_found:
	if (caa_unlikely(node_flag_ptr))
		*node_flag_ptr = NULL;
	return NULL;
}

static inline_lookup
uint8_t ft_popcount_1l_node_get_nr_child(const struct cds_ft_type *type,
		struct cds_ft_inode *node)
{
	const struct ft_popcount_1l_header *hdr =
		(const struct ft_popcount_1l_header *) &node->data[0];
	(void) type;
	return (uint8_t) (
		__builtin_popcountll(hdr->bm[0]) +
		__builtin_popcountll(hdr->bm[1]) +
		__builtin_popcountll(hdr->bm[2]) +
		__builtin_popcountll(hdr->bm[3]));
}

/*
 * Find the i-th set bit in the 256-bit bitmap and return its byte
 * value (0..255) and the corresponding pointer.  Iterates over the
 * four 64-bit words; uses a "skip the bottom popcount bits" pattern
 * (clear the lowest set bit i times) to land on the target.
 */
static inline_lookup
void ft_popcount_1l_node_get_ith_pos(const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		uint8_t i,
		uint8_t *v,
		struct cds_ft_inode_flag **iter)
{
	struct ft_popcount_1l_header *hdr =
		(struct ft_popcount_1l_header *) &node->data[0];
	struct cds_ft_inode_flag **pointers =
		ft_popcount_1l_pointers(node);
	unsigned int word_idx = 0;
	unsigned int rem = i;
	uint64_t word;
	unsigned int pop;
	unsigned int bit;
	(void) type;

	for (word_idx = 0; word_idx < 4; word_idx++) {
		pop = (unsigned int) __builtin_popcountll(hdr->bm[word_idx]);
		if (rem < pop)
			break;
		rem -= pop;
	}
	assert(word_idx < 4);
	word = hdr->bm[word_idx];
	while (rem--)
		word &= word - 1ULL;	/* clear lowest set bit */
	bit = (unsigned int) __builtin_ctzll(word);
	*v = (uint8_t) (word_idx * 64U + bit);
	*iter = ft_dereference_acquire(pointers[i]);
}

/*
 * Walk all populated entries in byte order (which matches popcount
 * order, since the bitmap iterates low-to-high) to find the
 * leftmost (largest v < n) or rightmost (smallest v > n) match.
 */
static inline_lookup
struct cds_ft_inode_flag *ft_popcount_1l_node_get_direction(
		const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		int n, uint8_t *result_key,
		enum ft_direction dir)
{
	uint8_t nr_child;
	struct cds_ft_inode_flag *match_ptr = NULL;
	int match_v;
	unsigned int i;

	assert(dir == FT_LEFT || dir == FT_RIGHT);

	nr_child = ft_popcount_1l_node_get_nr_child(type, node);
	cmm_smp_rmb();

	if (dir == FT_LEFT)
		match_v = -1;
	else
		match_v = FT_ENTRY_PER_NODE;

	for (i = 0; i < nr_child; i++) {
		struct cds_ft_inode_flag *ptr;
		uint8_t v;

		ft_popcount_1l_node_get_ith_pos(type, node, (uint8_t) i, &v, &ptr);
		if (!ptr)
			continue;
		if (dir == FT_LEFT) {
			if ((int) v < n && (int) v > match_v) {
				match_v = v;
				match_ptr = ptr;
				if (match_v == n - 1)
					break;
			}
		} else {
			if ((int) v > n && (int) v < match_v) {
				match_v = v;
				match_ptr = ptr;
				if (match_v == n + 1)
					break;
			}
		}
	}
	if (!match_ptr)
		return NULL;
	assert(match_v >= 0 && match_v < FT_ENTRY_PER_NODE);
	*result_key = (uint8_t) match_v;
	return match_ptr;
}

/*
 * Insert (n, child_node_flag) into a freshly-allocated, unpublished
 * byte-popcount node.  Called only from the recompact path.
 *
 * is_init=true zeroes the bitmap and writes pointers[0].  Subsequent
 * non-init calls fold a new entry into the existing layout, shifting
 * pointers as needed to keep popcount order.
 */
static
int ft_popcount_1l_node_set_nth(const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		struct cds_ft_metadata *metadata,
		uint8_t n,
		struct cds_ft_inode_flag *child_node_flag,
		bool is_init)
{
	struct ft_popcount_1l_header *hdr =
		(struct ft_popcount_1l_header *) &node->data[0];
	struct cds_ft_inode_flag **pointers =
		ft_popcount_1l_pointers(node);
	unsigned int word_idx = (unsigned int) n >> 6;
	unsigned int bit_idx = (unsigned int) n & 63U;
	unsigned int nr_child, ptr_idx, k;
	uint64_t word;

	if (is_init) {
		memset(hdr, 0, sizeof(*hdr));
		hdr->bm[word_idx] = 1ULL << bit_idx;
		pointers[0] = child_node_flag;
		metadata->nr_child++;
		return 0;
	}

	nr_child = ft_popcount_1l_node_get_nr_child(type, node);
	assert(nr_child < type->max_linear_child);
	word = hdr->bm[word_idx];
	assert(!((word >> bit_idx) & 1ULL));	/* duplicate key would be a bug */

	ptr_idx = 0;
	for (k = 0; k < word_idx; k++)
		ptr_idx += (unsigned int) __builtin_popcountll(hdr->bm[k]);
	ptr_idx += (unsigned int) __builtin_popcountll(
			word & ((1ULL << bit_idx) - 1ULL));

	for (k = nr_child; k > ptr_idx; k--)
		pointers[k] = pointers[k - 1];
	pointers[ptr_idx] = child_node_flag;

	hdr->bm[word_idx] = word | (1ULL << bit_idx);
	metadata->nr_child++;
	return 0;
}


static inline_lookup
struct cds_ft_inode_flag *ft_pigeon_node_get_nth(const struct cds_ft_type __attribute__((unused)) *type,
		struct cds_ft_inode *node,
		struct cds_ft_inode_flag ***node_flag_ptr,
		uint8_t n, enum ft_pf_target pf_hint)
{
	struct cds_ft_inode_flag **child_node_flag_ptr;
	struct cds_ft_inode_flag *child_node_flag;

	assert(!type || ft_type_is_pigeon(type->type_class));
	child_node_flag_ptr = &((struct cds_ft_inode_flag **) node->data)[n];
	child_node_flag = ft_dereference_acquire_prefetch_hint(*child_node_flag_ptr, pf_hint);
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

	assert(ft_type_is_pigeon(type->type_class));
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
	return ft_pigeon_node_get_nth(type, node, NULL, i, FT_PF_NONE);
}

/*
 * ft_node_get_nth: get nth item from a node.
 * node_flag is already rcu_dereference'd.
 */

/*
 * Compile-time bitmasks indexed by type_index: for each class,
 * the set of type_index values whose ft_types[] entry has that
 * class.  Dispatching via `(1U << type_index) & FT_MASK_X` keeps
 * the check in ALU ops only.  Each bit is derived from
 * ft_types[i].type_class directly, so an ft_types[] edit
 * (reclassification, new type) updates the masks automatically.
 */
#define FT_TC_BIT(n, tc) \
	(ft_types[(n)].type_class == (tc) ? (1U << (n)) : 0)

#if CAA_BITS_PER_LONG < 64
#define FT_MASK_CLASS(tc) \
	(FT_TC_BIT(0, tc) | FT_TC_BIT(1, tc) | FT_TC_BIT(2, tc) | \
	 FT_TC_BIT(3, tc) | FT_TC_BIT(4, tc) | FT_TC_BIT(5, tc) | \
	 FT_TC_BIT(6, tc))
#else
#define FT_MASK_CLASS(tc) \
	(FT_TC_BIT(0, tc) | FT_TC_BIT(1, tc) | FT_TC_BIT(2, tc) | \
	 FT_TC_BIT(3, tc) | FT_TC_BIT(4, tc) | FT_TC_BIT(5, tc) | \
	 FT_TC_BIT(6, tc) | FT_TC_BIT(7, tc))
#endif

#define FT_MASK_POPCOUNT    FT_MASK_CLASS(FT_POPCOUNT)

/*
 * Runtime assertions on the popcount_1l slots: both host
 * popcount_1l FT_POPCOUNT nodes.  Validated at library load so
 * any future ft_types[] regression fires before any lookup runs.
 */
static void __attribute__((constructor))
ft_check_popcount_1l_idx_assumptions(void)
{
#if (CAA_BITS_PER_LONG < 64)
	assert(ft_types[3].type_class == FT_POPCOUNT && ft_types[3].popcount_1l);
	assert(ft_types[4].type_class == FT_POPCOUNT && ft_types[4].popcount_1l);
#else
	assert(ft_types[4].type_class == FT_POPCOUNT && ft_types[4].popcount_1l);
	assert(ft_types[5].type_class == FT_POPCOUNT && ft_types[5].popcount_1l);
#endif
}

/*
 * ft_node_get_nth_skip: raw child slot access.  Returns the slot
 * value as-is, including skip-compressed pointers.  Used only by
 * candidate lookup which resolves skip pointers itself.
 *
 * Per-type dispatch on type_index (no ft_types[] field load).  Each
 * arch maps type_index to a scanner specialized for that tier's
 * layout.
 *
 * 64-bit type-index -> scanner:
 *   0:  popcount_2l_scan_16_16_max_3 (32 B,  max_lc=3)
 *   1:  popcount_2l_scan_32_8        (64 B,  max_lc=6)
 *   2:  popcount_2l_scan_64_4        (128 B, max_lc=14)
 *   3:  popcount_1l_scan_28          (256 B, max_lc=28)
 *   4:  popcount_1l_scan_28          (512 B, max_lc=60)
 *   5:  popcount_1l_scan_28         (1024 B, max_lc=124)
 *   6:  pigeon                      (2048 B)
 *
 * 32-bit type-index -> scanner:
 *   0:  popcount_2l_scan_16_16_max_5 (32 B,  max_lc=5)
 *   1:  popcount_2l_scan_64_4        (64 B,  max_lc=12)
 *   2:  popcount_1l_scan_28          (128 B, max_lc=24)
 *   3:  popcount_1l_scan_28          (256 B, max_lc=56)
 *   4:  popcount_1l_scan_28          (512 B, max_lc=120)
 *   5:  pigeon                      (1024 B)
 *
 * The popcount scanners are pure integer math (popcount + bitmap
 * indexing); no SSE / SIMD dependency.  The hot caa_likely branch
 * targets the most-frequent type observed across the comprehensive
 * benchmark (popcount_2l max_lc=3 on 64-bit: ~46% of dispatches).
 */

/*
 * FT_NODE_SUB_TAG: SUB-by-imm tag-clear.  At each dispatch case below
 * the type_index is a compile-time literal, so the tag value to
 * subtract is a compile-time constant — the compiler emits a single
 * SUB-by-immediate (or folds into an LEA with displacement) instead
 * of the data-dependent variable-shift AND used by
 * ft_node_ptr_internal.
 *
 * The subtraction is exact because every alignment bit covered by the
 * tag bits is zero in the raw address: type-T internal nodes are
 * 2^(4+T)-byte aligned, while the tag bits live in [0,3]
 * (FT_INTERNAL_MASK | FT_TYPE_MASK = 0xF).
 *
 * On FEATURE_FT_SKIP_COMPRESSED builds the skip-len high bits live
 * at [FT_SKIP_LEN_SHIFT, 63]; mask them off with FT_ADDR_MASK so
 * the result is a canonical address suitable for memory access.
 * Builds without FEATURE_FT_SKIP_COMPRESSED skip the mask entirely.
 */
#ifdef FEATURE_FT_SKIP_COMPRESSED
# define FT_NODE_SUB_TAG(nf, type_idx)					\
	((struct cds_ft_inode *)					\
		((((unsigned long) (nf) -				\
		   (((unsigned long) (type_idx) << FT_INTERNAL_BITS) | \
		    FT_INTERNAL_MASK)) & FT_ADDR_MASK)))
#else
# define FT_NODE_SUB_TAG(nf, type_idx)					\
	((struct cds_ft_inode *)					\
		((unsigned long) (nf) -					\
		 (((unsigned long) (type_idx) << FT_INTERNAL_BITS) |	\
		  FT_INTERNAL_MASK)))
#endif

/*
 * FT_NODE_SUB_TAG_NOSKIP: SUB-by-imm tag-clear variant *without* the
 * FT_ADDR_MASK (skip_len high-bit clear).  Safe only at descent sites
 * where the caller has already established skip_len = 0 (e.g. via the
 * loop-top ft_node_skip_compressed handler that either skips the entry
 * or resolves the skip through ft_skip_child_ptr).  The pure SUB folds
 * into the immediately-following load's displacement, avoiding the
 * movabs of the 64-bit FT_ADDR_MASK constant in the hot path.
 */
#define FT_NODE_SUB_TAG_NOSKIP(nf, type_idx)				\
	((struct cds_ft_inode *)					\
		((unsigned long) (nf) -					\
		 (((unsigned long) (type_idx) << FT_INTERNAL_BITS) |	\
		  FT_INTERNAL_MASK)))

static inline_lookup
struct cds_ft_inode_flag *ft_node_get_nth_skip(struct cds_ft_inode_flag *node_flag,
		struct cds_ft_inode_flag ***node_flag_ptr,
		uint8_t n, enum ft_pf_target pf_hint)
{
	unsigned long tag = (unsigned long) node_flag & 0xF;
	unsigned int type_index;

	/* External / compressed: internal flag clear. */
	if (caa_unlikely(!(tag & FT_INTERNAL_MASK))) {
		if (caa_unlikely(node_flag_ptr))
			*node_flag_ptr = NULL;
		return NULL;
	}

	type_index = (tag >> FT_INTERNAL_BITS) & 0x7;

#if CAA_BITS_PER_LONG >= 64
	/*
	 * Pure switch; expect gcc to emit a jump table for 0..6.  Each
	 * case computes FT_NODE_SUB_TAG with a compile-time-literal type
	 * so the SUB is by-immediate and has no data dependency on
	 * @node_flag's type bits.
	 */
	switch (type_index) {
	case 0:
		return ft_popcount_2l_scan_16_16_max_3(
			FT_NODE_SUB_TAG(node_flag, 0), node_flag_ptr, n, pf_hint);
	case 1:
		return ft_popcount_2l_scan_32_8(
			FT_NODE_SUB_TAG(node_flag, 1), node_flag_ptr, n, pf_hint);
	case 2:
		return ft_popcount_2l_scan_64_4(
			FT_NODE_SUB_TAG(node_flag, 2), node_flag_ptr, n, pf_hint);
	case 3:
		return ft_popcount_1l_scan_28(
			FT_NODE_SUB_TAG(node_flag, 3), node_flag_ptr, n, pf_hint);
	case 4:
		return ft_popcount_1l_scan_28(
			FT_NODE_SUB_TAG(node_flag, 4), node_flag_ptr, n, pf_hint);
	case 5:
		return ft_popcount_1l_scan_28(
			FT_NODE_SUB_TAG(node_flag, 5), node_flag_ptr, n, pf_hint);
	case 6:
		return ft_pigeon_node_get_nth(NULL,
			FT_NODE_SUB_TAG(node_flag, 6), node_flag_ptr, n, pf_hint);
	default:
		/*
		 * type_index is a 3-bit field; values 7+ are NODE_INDEX_NULL,
		 * never encoded in a tagged pointer.
		 */
		__builtin_unreachable();
	}
#else
	switch (type_index) {
	case 0:
		return ft_popcount_2l_scan_16_16_max_5(
			FT_NODE_SUB_TAG(node_flag, 0), node_flag_ptr, n, pf_hint);
	case 1:
		return ft_popcount_2l_scan_64_4(
			FT_NODE_SUB_TAG(node_flag, 1), node_flag_ptr, n, pf_hint);
	case 2:
		return ft_popcount_1l_scan_28(
			FT_NODE_SUB_TAG(node_flag, 2), node_flag_ptr, n, pf_hint);
	case 3:
		return ft_popcount_1l_scan_28(
			FT_NODE_SUB_TAG(node_flag, 3), node_flag_ptr, n, pf_hint);
	case 4:
		return ft_popcount_1l_scan_28(
			FT_NODE_SUB_TAG(node_flag, 4), node_flag_ptr, n, pf_hint);
	case 5:
		return ft_pigeon_node_get_nth(NULL,
			FT_NODE_SUB_TAG(node_flag, 5), node_flag_ptr, n, pf_hint);
	default:
		__builtin_unreachable();
	}
#endif
}

/*
 * ft_node_get_nth_skip_pretyped: same dispatch as ft_node_get_nth_skip,
 * but the caller has already extracted @type_index from the tag bits
 * and verified the FT_INTERNAL_MASK bit was set.  The parent pointer
 * @node_flag is still tagged — each switch case applies
 * FT_NODE_SUB_TAG_NOSKIP with a compile-time-literal type, so the
 * SUB-by-immediate folds into the immediately-following body load's
 * displacement (gcc emits e.g. `mov -5(%rcx), %rax`), avoiding an
 * explicit AND step on the per-step dependency chain to the body load.
 *
 * Skip-compressed safety: at this call site the loop-top
 * ft_node_skip_compressed handler has already either skipped this
 * entry or resolved the skip via ft_skip_child_ptr (which masks
 * FT_ADDR_MASK internally), so @node_flag's high-bit skip_len field is
 * zero and the FT_NODE_SUB_TAG_NOSKIP form is safe.
 */
static inline_lookup
struct cds_ft_inode_flag *ft_node_get_nth_skip_pretyped(
		struct cds_ft_inode_flag *node_flag,
		unsigned int type_index,
		struct cds_ft_inode_flag ***node_flag_ptr,
		uint8_t n, enum ft_pf_target pf_hint)
{
#if CAA_BITS_PER_LONG >= 64
	switch (type_index) {
	case 0:
		return ft_popcount_2l_scan_16_16_max_3(
			FT_NODE_SUB_TAG_NOSKIP(node_flag, 0),
			node_flag_ptr, n, pf_hint);
	case 1:
		return ft_popcount_2l_scan_32_8(
			FT_NODE_SUB_TAG_NOSKIP(node_flag, 1),
			node_flag_ptr, n, pf_hint);
	case 2:
		return ft_popcount_2l_scan_64_4(
			FT_NODE_SUB_TAG_NOSKIP(node_flag, 2),
			node_flag_ptr, n, pf_hint);
	case 3:
		return ft_popcount_1l_scan_28(
			FT_NODE_SUB_TAG_NOSKIP(node_flag, 3),
			node_flag_ptr, n, pf_hint);
	case 4:
		return ft_popcount_1l_scan_28(
			FT_NODE_SUB_TAG_NOSKIP(node_flag, 4),
			node_flag_ptr, n, pf_hint);
	case 5:
		return ft_popcount_1l_scan_28(
			FT_NODE_SUB_TAG_NOSKIP(node_flag, 5),
			node_flag_ptr, n, pf_hint);
	case 6:
		return ft_pigeon_node_get_nth(NULL,
			FT_NODE_SUB_TAG_NOSKIP(node_flag, 6),
			node_flag_ptr, n, pf_hint);
	default:
		__builtin_unreachable();
	}
#else
	switch (type_index) {
	case 0:
		return ft_popcount_2l_scan_16_16_max_5(
			FT_NODE_SUB_TAG_NOSKIP(node_flag, 0),
			node_flag_ptr, n, pf_hint);
	case 1:
		return ft_popcount_2l_scan_64_4(
			FT_NODE_SUB_TAG_NOSKIP(node_flag, 1),
			node_flag_ptr, n, pf_hint);
	case 2:
		return ft_popcount_1l_scan_28(
			FT_NODE_SUB_TAG_NOSKIP(node_flag, 2),
			node_flag_ptr, n, pf_hint);
	case 3:
		return ft_popcount_1l_scan_28(
			FT_NODE_SUB_TAG_NOSKIP(node_flag, 3),
			node_flag_ptr, n, pf_hint);
	case 4:
		return ft_popcount_1l_scan_28(
			FT_NODE_SUB_TAG_NOSKIP(node_flag, 4),
			node_flag_ptr, n, pf_hint);
	case 5:
		return ft_pigeon_node_get_nth(NULL,
			FT_NODE_SUB_TAG_NOSKIP(node_flag, 5),
			node_flag_ptr, n, pf_hint);
	default:
		__builtin_unreachable();
	}
#endif
}

/*
 * ft_node_get_nth: child slot access with skip-compressed resolution.
 * If the child is a skip pointer, converts it to the underlying
 * compressed node flag so callers see it as a regular compressed node.
 * Used by all paths except candidate lookup.
 *
 * No validation: callers may be writers reading their own in-flight
 * mutation state (between ft_set_parent and ft_publish_to_parent),
 * where the parent back-pointer chain is intentionally inconsistent
 * with the slot value.  Reader callers that need consistency wrap
 * with ft_skip_to_compressed and the validation retry locally.
 */
static inline_lookup
struct cds_ft_inode_flag *ft_node_get_nth(struct cds_ft_inode_flag *node_flag,
		struct cds_ft_inode_flag ***node_flag_ptr,
		uint8_t n, enum ft_pf_target pf_hint)
{
	struct cds_ft_inode_flag *child;

	child = ft_node_get_nth_skip(node_flag, node_flag_ptr, n, pf_hint);
	child = ft_resolve_skip_compressed(child);
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
 * Only handles internal node types (linear, popcount, pigeon).
 * Compressed parents are handled separately by callers.
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
	case FT_POPCOUNT:
	{
		uint8_t nr_child = ft_popcount_node_get_nr_child(type, node);
		unsigned int i;

		for (i = 0; i < nr_child; i++) {
			struct cds_ft_inode_flag *iter;
			uint8_t v;

			ft_popcount_node_get_ith_pos(type, node, i, &v, &iter);
			if (iter == child_nf) {
				if (n_ret)
					*n_ret = v;
				if (slot_ret)
					ft_node_get_nth(parent_nf, slot_ret, v, FT_PF_NONE);
				return true;
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
				if (n_ret)
					*n_ret = (uint8_t) i;
				if (slot_ret)
					ft_node_get_nth(parent_nf, slot_ret, i, FT_PF_NONE);
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

/*
 * @validate_lookup: when true, validate the skip-compressed resolution
 * against the slot's skip_len and retry on mismatch.  RCU readers
 * must pass true; writers must pass false (a writer mid-mutation
 * reading its own in-flight state would loop forever — the writer
 * IS the race partner).  With static inlining and a constant arg,
 * the unused branch is DCE'd at each call site.
 */
static inline_lookup
struct cds_ft_inode_flag *ft_node_get_direction(struct cds_ft_inode_flag *node_flag,
		int n, uint8_t *result_key,
		enum ft_direction dir, bool validate_lookup)
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

	for (;;) {
		switch (type->type_class) {
		case FT_POPCOUNT:
			child = ft_popcount_node_get_direction(type, node, n, result_key, dir);
			break;
		case FT_PIGEON:
			child = ft_pigeon_node_get_direction(type, node, n, result_key, dir);
			break;
		default:
			assert(0);
			return (void *) -1UL;
		}
		if (!validate_lookup)
			return ft_resolve_skip_compressed(child);
		if (!child || !ft_node_skip_compressed(child))
			return child;
#ifdef FEATURE_FT_SKIP_COMPRESSED
		{
			struct cds_ft_compressed_node *cn =
				ft_skip_to_compressed_validate(child);
			if (caa_likely(cn != NULL))
				return ft_compressed_node_flag(cn);
		}
		caa_cpu_relax();
#else
		return child;
#endif
	}
}

static inline_lookup
struct cds_ft_inode_flag *ft_node_get_leftright(struct cds_ft_inode_flag *node_flag,
		unsigned int n, uint8_t *result_key,
		enum ft_direction dir, bool validate_lookup)
{
	return ft_node_get_direction(node_flag, n, result_key, dir, validate_lookup);
}

static inline_lookup
struct cds_ft_inode_flag *ft_node_get_minmax(struct cds_ft_inode_flag *node_flag,
		uint8_t *result_key,
		enum ft_direction dir, bool validate_lookup)
{
	switch (dir) {
	case FT_LEFTMOST:
		return ft_node_get_direction(node_flag,
				-1, result_key, FT_RIGHT, validate_lookup);
	case FT_RIGHTMOST:
		return ft_node_get_direction(node_flag,
				FT_ENTRY_PER_NODE, result_key, FT_LEFT, validate_lookup);
	default:
		assert(0);
		return NULL;
	}
}

/*
 * Insert (or replace) child_node_flag at byte n within an FT_POPCOUNT
 * node.  Three outcome paths:
 *
 *   - is_init: caller-promised first set_nth on a freshly-allocated
 *     (unpublished) node; call the layout's helper directly with
 *     is_init=true to set up the bitmap and pointer array from
 *     scratch.
 *
 *   - non-init, key already present: in-place pointer replace
 *     (single-slot RCU-safe write).
 *
 *   - non-init, key not present, "no existing bit lies above n":
 *     safe-append.  Setting bit n does not shift any existing
 *     pointer's popcount-rank, so the new pointer can be written at
 *     the (current) end-of-array slot and the new bit then published.
 *     Reader sees either the old state, an "appendinflight" state
 *     that returns NULL (legitimate), or the fully-published new
 *     entry.  See the publish-stores below for the detailed ordering
 *     reasoning (release on the new pointer, relaxed on each bitmap
 *     publish; reader's address-dependency from bitmap-derived index
 *     to pointer load keeps the loads ordered on weakly-ordered
 *     architectures).
 *
 *   - otherwise: -ERANGE, forcing the caller to recompact.
 */
static
int ft_popcount_node_set_nth(const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		struct cds_ft_metadata *metadata,
		uint8_t n,
		struct cds_ft_inode_flag *child_node_flag,
		bool *_replace_old_ptr,
		bool is_init)
{
	assert(ft_type_is_popcount(type->type_class));

	if (type->popcount_2l && type->max_linear_child == 6) {
		/* Flat 5+3 layout (max_lc=6 on 64-bit). */
		struct cds_ft_inode_flag **qp_pointers =
			ft_popcount_2l_pointers(node, type);
		unsigned int qp_hi = (unsigned int) n >> 3;
		unsigned int qp_lo = (unsigned int) n & 0x7U;
		uint32_t qp_root;
		uint64_t qp_bms;
		unsigned int qp_slot1, qp_p, qp_ptr_idx;

		if (is_init) {
			int ret;
			ret = ft_popcount_2l_node_set_nth(type, node,
					metadata, n, child_node_flag, true);
			if (_replace_old_ptr)
				*_replace_old_ptr = false;
			return ret;
		}
		qp_root = *(const uint32_t *) &node->data[0];
		qp_bms  = *(const uint64_t *) &node->data[4];
		qp_slot1 = (unsigned int) __builtin_popcount(
				qp_root & ((1U << qp_hi) - 1U));
		qp_p = (qp_slot1 << 3) | qp_lo;

		if (qp_root >> qp_hi & 1U) {
			/* hi already present */
			if ((qp_bms >> qp_p) & 1ULL) {
				/* Case 1: key already present, in-place replace. */
				qp_ptr_idx = (unsigned int) __builtin_popcountll(
						qp_bms & ((1ULL << qp_p) - 1ULL));
				if (qp_pointers[qp_ptr_idx]) {
					if (_replace_old_ptr)
						*_replace_old_ptr = true;
				} else {
					if (_replace_old_ptr)
						*_replace_old_ptr = false;
					metadata->nr_child++;
				}
				rcu_assign_pointer(qp_pointers[qp_ptr_idx],
						child_node_flag);
				return 0;
			}
			/*
			 * Case 2A: safe-append within existing hi.
			 * Need: new lo is highest in slot, and no slot1' > slot1.
			 */
			{
				/* bits in current slot above qp_lo */
				unsigned int slot_byte_pos = qp_slot1 * 8;
				uint64_t in_byte_above = (qp_lo == 7) ? 0ULL :
					((qp_bms >> (slot_byte_pos + qp_lo + 1))
						& ((1ULL << (7 - qp_lo)) - 1ULL));
				/* bits in higher-slot bytes */
				uint64_t above_slot = (qp_slot1 == 7) ? 0ULL :
					(qp_bms >> ((qp_slot1 + 1) * 8));
				uint32_t root_above =
					(uint32_t) ((qp_root >> qp_hi) >> 1);
				if (in_byte_above != 0 || above_slot != 0
						|| root_above != 0)
					return -ERANGE;	/* Case 3. */
			}
			qp_ptr_idx = (unsigned int) __builtin_popcountll(qp_bms);
			if (qp_ptr_idx >= type->max_linear_child)
				return -ENOSPC;
			rcu_assign_pointer(qp_pointers[qp_ptr_idx], child_node_flag);
			uatomic_store((uint64_t *) &node->data[4],
				qp_bms | (((uint64_t)(1U << qp_lo)) << (qp_slot1 * 8)),
				CMM_RELAXED);
			metadata->nr_child++;
			if (_replace_old_ptr)
				*_replace_old_ptr = false;
			return 0;
		}
		/*
		 * Case 2B: hi not present.  Safe-append of a new hi.
		 * Need: no bit at a position > hi in root_bm.
		 */
		if ((qp_root >> qp_hi) != 0)
			return -ERANGE;	/* Case 3. */
		qp_slot1 = (unsigned int) __builtin_popcount(qp_root);
		qp_ptr_idx = (unsigned int) __builtin_popcountll(qp_bms);
		if (qp_ptr_idx >= type->max_linear_child)
			return -ENOSPC;
		/*
		 * slot1 == popcount(root) since all set his are below qp_hi.
		 * The byte at slot1 position is guaranteed zero (alloc-zeroed
		 * or recompact-into; never written for an unpopulated slot).
		 */
		uatomic_store((uint64_t *) &node->data[4],
			qp_bms | (((uint64_t)(1U << qp_lo)) << (qp_slot1 * 8)),
			CMM_RELAXED);
		rcu_assign_pointer(qp_pointers[qp_ptr_idx], child_node_flag);
		uatomic_store((uint32_t *) &node->data[0],
			qp_root | (1U << qp_hi), CMM_RELAXED);
		metadata->nr_child++;
		if (_replace_old_ptr)
			*_replace_old_ptr = false;
		return 0;
	}
	if (type->popcount_2l && (type->max_linear_child == 12
				|| type->max_linear_child == 14
				|| type->max_linear_child == 16)) {
		/* Flat 6+2 layout (max_lc=14 on 64-bit, 12/16 on 32-bit). */
		struct cds_ft_inode_flag **qp_pointers =
			ft_popcount_2l_pointers(node, type);
		unsigned int qp_hi = (unsigned int) n >> 2;
		unsigned int qp_lo = (unsigned int) n & 0x3U;
		uint64_t qp_root, qp_bms;
		unsigned int qp_slot1, qp_p, qp_ptr_idx;

		if (is_init) {
			int ret;
			ret = ft_popcount_2l_node_set_nth(type, node,
					metadata, n, child_node_flag, true);
			if (_replace_old_ptr)
				*_replace_old_ptr = false;
			return ret;
		}
		qp_root = *(const uint64_t *) &node->data[0];
		qp_bms  = *(const uint64_t *) &node->data[8];
		qp_slot1 = (unsigned int) __builtin_popcountll(
				qp_root & ((1ULL << qp_hi) - 1ULL));
		qp_p = (qp_slot1 << 2) | qp_lo;

		if ((qp_root >> qp_hi) & 1ULL) {
			/* hi already present */
			if ((qp_bms >> qp_p) & 1ULL) {
				/* Case 1: key already present, in-place replace. */
				qp_ptr_idx = (unsigned int) __builtin_popcountll(
						qp_bms & ((1ULL << qp_p) - 1ULL));
				if (qp_pointers[qp_ptr_idx]) {
					if (_replace_old_ptr)
						*_replace_old_ptr = true;
				} else {
					if (_replace_old_ptr)
						*_replace_old_ptr = false;
					metadata->nr_child++;
				}
				rcu_assign_pointer(qp_pointers[qp_ptr_idx],
						child_node_flag);
				return 0;
			}
			/*
			 * Case 2A: safe-append within existing hi.
			 * Need: new lo is highest in slot, no slot1' > slot1.
			 */
			{
				/* bits in current slot above qp_lo */
				unsigned int slot_nibble_pos = qp_slot1 * 4;
				uint64_t in_nibble_above = (qp_lo == 3) ? 0ULL :
					((qp_bms >> (slot_nibble_pos + qp_lo + 1))
						& ((1ULL << (3 - qp_lo)) - 1ULL));
				/* bits in higher-slot nibbles */
				uint64_t above_slot = (qp_slot1 == 13) ? 0ULL :
					(qp_bms >> ((qp_slot1 + 1) * 4));
				uint64_t root_above =
					(qp_root >> qp_hi) >> 1;
				if (in_nibble_above != 0 || above_slot != 0
						|| root_above != 0)
					return -ERANGE;	/* Case 3. */
			}
			qp_ptr_idx = (unsigned int) __builtin_popcountll(qp_bms);
			if (qp_ptr_idx >= type->max_linear_child)
				return -ENOSPC;
			rcu_assign_pointer(qp_pointers[qp_ptr_idx], child_node_flag);
			uatomic_store((uint64_t *) &node->data[8],
				qp_bms | (((uint64_t)(1U << qp_lo)) << (qp_slot1 * 4)),
				CMM_RELAXED);
			metadata->nr_child++;
			if (_replace_old_ptr)
				*_replace_old_ptr = false;
			return 0;
		}
		/*
		 * Case 2B: hi not present.  Safe-append of a new hi.
		 * Need: no bit at a position > hi in root_bm.
		 */
		if ((qp_root >> qp_hi) != 0ULL)
			return -ERANGE;	/* Case 3. */
		qp_slot1 = (unsigned int) __builtin_popcountll(qp_root);
		qp_ptr_idx = (unsigned int) __builtin_popcountll(qp_bms);
		if (qp_ptr_idx >= type->max_linear_child)
			return -ENOSPC;
		/*
		 * slot1 == popcount(root) since all set his are below qp_hi.
		 * The nibble at slot1 position is guaranteed zero (alloc-
		 * zeroed or recompact-into; never written for an unpopulated
		 * slot).
		 */
		uatomic_store((uint64_t *) &node->data[8],
			qp_bms | (((uint64_t)(1U << qp_lo)) << (qp_slot1 * 4)),
			CMM_RELAXED);
		rcu_assign_pointer(qp_pointers[qp_ptr_idx], child_node_flag);
		uatomic_store((uint64_t *) &node->data[0],
			qp_root | (1ULL << qp_hi), CMM_RELAXED);
		metadata->nr_child++;
		if (_replace_old_ptr)
			*_replace_old_ptr = false;
		return 0;
	}
	if (type->popcount_2l) {
		struct ft_popcount_2l_header *qp_hdr;
		struct cds_ft_inode_flag **qp_pointers;
		unsigned int qp_hi, qp_lo, qp_slot1, qp_ptr_idx;
		uint16_t qp_root, qp_sub;

		if (is_init) {
			int ret;
			ret = ft_popcount_2l_node_set_nth(type, node,
					metadata, n, child_node_flag, true);
			if (_replace_old_ptr)
				*_replace_old_ptr = false;
			return ret;
		}
		qp_hdr = (struct ft_popcount_2l_header *) &node->data[0];
		qp_pointers = ft_popcount_2l_pointers(node, type);
		qp_root = qp_hdr->root_bm;
		qp_hi = (unsigned int) n >> 4;
		qp_lo = (unsigned int) n & 0xFU;
		qp_slot1 = (unsigned int) __builtin_popcount(
				qp_root & ((1U << qp_hi) - 1U));

		if ((qp_root >> qp_hi) & 1U) {
			qp_sub = qp_hdr->sub_bm[qp_slot1];
			if ((qp_sub >> qp_lo) & 1U) {
				/* Case 1: key already present, in-place replace. */
				uint64_t subs_below = 0;
				unsigned int k;

				for (k = 0; k < qp_slot1; k++)
					subs_below += (uint64_t)
						__builtin_popcount(
							qp_hdr->sub_bm[k]);
				subs_below += (uint64_t) __builtin_popcount(
						(unsigned int) qp_sub
						& ((1U << qp_lo) - 1U));
				qp_ptr_idx = (unsigned int) subs_below;
				if (qp_pointers[qp_ptr_idx]) {
					if (_replace_old_ptr)
						*_replace_old_ptr = true;
				} else {
					if (_replace_old_ptr)
						*_replace_old_ptr = false;
					metadata->nr_child++;
				}
				rcu_assign_pointer(qp_pointers[qp_ptr_idx],
						child_node_flag);
				return 0;
			}
			/*
			 * Case 2A: try safe-append within existing hi.
			 * Need: no bit above lo in sub_bm[slot1], AND no bit
			 * above hi in root_bm (so sub_bm[k > slot1] is all 0).
			 */
			{
				uint16_t in_word_above = qp_lo == 15 ? 0
					: (uint16_t) (qp_sub >> (qp_lo + 1));
				uint16_t root_above = (uint16_t)
					((qp_root >> qp_hi) >> 1);

				if (in_word_above != 0 || root_above != 0)
					return -ERANGE;	/* Case 3. */
			}
			qp_ptr_idx = (unsigned int)
				ft_popcount_2l_node_get_nr_child(
					type, node);
			if (qp_ptr_idx >= type->max_linear_child)
				return -ENOSPC;
			rcu_assign_pointer(qp_pointers[qp_ptr_idx],
					child_node_flag);
			uatomic_store(&qp_hdr->sub_bm[qp_slot1],
					(uint16_t) (qp_sub | (1U << qp_lo)),
					CMM_RELAXED);
			metadata->nr_child++;
			if (_replace_old_ptr)
				*_replace_old_ptr = false;
			return 0;
		}
		/*
		 * Case 2B: hi not present.  Try safe-append of a new hi.
		 * Need: no bit at a position > hi in root_bm.
		 */
		if ((qp_root >> qp_hi) != 0)
			return -ERANGE;	/* Case 3. */
		/*
		 * slot1 == popcount(root_bm) since all currently-set hi
		 * positions are below qp_hi.  This is the first unused
		 * sub_bm slot, guaranteed zero from alloc / recompact-into.
		 */
		qp_slot1 = (unsigned int) __builtin_popcount(qp_root);
		qp_ptr_idx = (unsigned int)
			ft_popcount_2l_node_get_nr_child(type, node);
		if (qp_ptr_idx >= type->max_linear_child)
			return -ENOSPC;
		uatomic_store(&qp_hdr->sub_bm[qp_slot1],
				(uint16_t) (1U << qp_lo), CMM_RELAXED);
		rcu_assign_pointer(qp_pointers[qp_ptr_idx], child_node_flag);
		uatomic_store(&qp_hdr->root_bm,
				(uint16_t) (qp_root | (1U << qp_hi)),
				CMM_RELAXED);
		metadata->nr_child++;
		if (_replace_old_ptr)
			*_replace_old_ptr = false;
		return 0;
	}
	assert(type->popcount_1l);
	if (is_init) {
		int ret;
		ret = ft_popcount_1l_node_set_nth(type, node,
				metadata, n, child_node_flag, true);
		if (_replace_old_ptr)
			*_replace_old_ptr = false;
		return ret;
	}
	{
		uint64_t *bm = (uint64_t *) &node->data[0];
		struct cds_ft_inode_flag **bp_pointers =
			(struct cds_ft_inode_flag **)
				((uint8_t *) node + 32);
		unsigned int word_idx = (unsigned int) n >> 6;
		unsigned int bit_idx = (unsigned int) n & 63U;
		uint64_t word = bm[word_idx];
		uint64_t bit = 1ULL << bit_idx;
		unsigned int ptr_idx = 0;
		unsigned int k;

		if (word & bit) {
			/* Case 1: in-place pointer replace. */
			for (k = 0; k < word_idx; k++)
				ptr_idx += (unsigned int)
					__builtin_popcountll(bm[k]);
			ptr_idx += (unsigned int) __builtin_popcountll(
					word & (bit - 1ULL));
			if (bp_pointers[ptr_idx]) {
				if (_replace_old_ptr)
					*_replace_old_ptr = true;
			} else {
				if (_replace_old_ptr)
					*_replace_old_ptr = false;
				metadata->nr_child++;
			}
			rcu_assign_pointer(bp_pointers[ptr_idx], child_node_flag);
			return 0;
		}

		/*
		 * Case 2: try safe-append.  Condition is "no existing
		 * bit at a position > n" -- evaluated as "no bit
		 * above bit_idx in word_idx, AND no bit set in any
		 * higher word".
		 */
		{
			bool safe_append;
			uint64_t in_word_above = bit_idx == 63 ? 0
				: (word >> (bit_idx + 1));

			safe_append = (in_word_above == 0);
			for (k = word_idx + 1; safe_append && k < 4; k++)
				safe_append = (bm[k] == 0);
			if (!safe_append)
				return -ERANGE;	/* Case 3. */
		}

		/* Case 2: do the append. */
		ptr_idx = (unsigned int) __builtin_popcountll(bm[0])
			+ (unsigned int) __builtin_popcountll(bm[1])
			+ (unsigned int) __builtin_popcountll(bm[2])
			+ (unsigned int) __builtin_popcountll(bm[3]);
		if (ptr_idx >= type->max_linear_child)
			return -ENOSPC;
		/*
		 * Pointer store carries a release: the next-level child
		 * node's contents (initialized by the caller before this
		 * set_nth) must be visible to readers that follow the
		 * pointer (matching acquire on the scanner's pointer load).
		 *
		 * Bitmap bit set is relaxed: it is only a reachability flag
		 * for this slot.  If a reader observes the bit set but the
		 * pointer-store is not yet visible (still NULL), the lookup
		 * returns NULL -- a legitimate not-found result for an
		 * append that hasn't fully propagated.  No reader can
		 * compute ptr_idx == this new slot without also seeing the
		 * bit set, so the slot is invisible until the bit is.
		 */
		rcu_assign_pointer(bp_pointers[ptr_idx], child_node_flag);
		uatomic_store(&bm[word_idx], word | bit, CMM_RELAXED);
		metadata->nr_child++;
		if (_replace_old_ptr)
			*_replace_old_ptr = false;
		return 0;
	}
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

	assert(ft_type_is_pigeon(type->type_class));
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
 *
 * @is_init: caller guarantees this is the first set_nth on a
 * freshly-allocated unpublished (sub)node.  Used by recompact to
 * adopt the first inserted byte as values[0].  Ignored for
 * FT_PIGEON (dense 256-slot array, no reserved slot).
 *
 * This helper does NOT set @child_node_flag's parent pointer.  The
 * caller is responsible for ft_set_parent once @node is in a state
 * where linking it from @child's parent pointer is safe:
 *
 *   - Regular in-place insert: ft_node_set_nth does ft_set_parent
 *     right after this returns, since @node is already published
 *     and fully valid.
 *   - Recompact child-copy: new_node is UNPUBLISHED and being built
 *     slot by slot; linking children's parent pointers to new_node
 *     mid-build would expose transient nr_child < min_child to
 *     parent-pointer readers.  Recompact's post-copy reparent loop
 *     performs ft_set_parent after the whole new_node is assembled.
 */
static
int _ft_node_set_nth(const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		struct cds_ft_inode_flag *node_flag __attribute__((unused)),
		struct cds_ft_metadata *metadata,
		uint8_t n,
		struct cds_ft_inode_flag *child_node_flag,
		bool is_init)
{
	int ret;

	switch (type->type_class) {
	case FT_POPCOUNT:
		ret = ft_popcount_node_set_nth(type, node, metadata, n, child_node_flag, NULL, is_init);
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
	return ret;
}

/*
 * FT_POPCOUNT class replace_ptr: publishes via rcu_assign_pointer on
 * the slot, with min_child gating on delete and nr_child accounting
 * via the bitmap-popcount helper.
 */
static
int ft_popcount_node_replace_ptr(const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		struct cds_ft_metadata *metadata,
		struct cds_ft_inode_flag **node_flag_ptr,
		struct cds_ft_inode_flag *newptr)
{
	assert(ft_type_is_popcount(type->type_class));
	assert(ft_popcount_node_get_nr_child(type, node) <= type->max_linear_child);

	if (!newptr) {
		assert(!metadata->fallback_removal_count);
		if (metadata->nr_child <= type->min_child) {
			/* We need to try recompacting the node */
			return -EFBIG;
		}
	}
	dbg_printf("popcount replace ptr: node %p\n", node);
	assert(*node_flag_ptr != NULL);
	rcu_assign_pointer(*node_flag_ptr, newptr);
	if (!newptr)
		metadata->nr_child--;
	dbg_printf("popcount replace ptr: %u child, metadata: %u child, for node %p newptr %p\n",
		(unsigned int) ft_popcount_node_get_nr_child(type, node),
		(unsigned int) metadata->nr_child,
		node, newptr);
	return 0;
}

static
int ft_pigeon_node_replace_ptr(const struct cds_ft_type *type,
		struct cds_ft_inode *node __attribute__((unused)),
		struct cds_ft_metadata *metadata,
		struct cds_ft_inode_flag **node_flag_ptr,
		uint8_t n __attribute__((unused)),
		struct cds_ft_inode_flag *newptr)
{
	assert(ft_type_is_pigeon(type->type_class));

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
	case FT_POPCOUNT:
		ret = ft_popcount_node_replace_ptr(type, node, metadata, node_flag_ptr, newptr);
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
		ft_set_parent(newptr, node_flag, node_flag_ptr);
	return ret;
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
		 * popcount type).  All other nodes are pruned.
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
		bool is_root,
		unsigned int node_depth __attribute__((unused)))
{
	unsigned int new_type_index;
	struct cds_ft_inode *new_node;
	struct cds_ft_metadata *new_metadata;
	const struct cds_ft_type *new_type;
	struct cds_ft_inode_flag *new_node_flag = NULL;
	int ret;
	int fallback = 0;
	/*
	 * Track whether new_node has received its first child via
	 * is_init=true.  Popcount nodes use a single init-done flag
	 * (no per-subnode state).
	 */
	bool new_init_done = false;

	/*
	 * Need to find nearest type index even for ADD_SAME, so that
	 * recompaction can promote/demote across tier boundaries
	 * (e.g. a popcount node that no longer fits its current tier).
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

	new_metadata = NULL;
	dbg_printf("Recompact from type %d to type %d\n",
			old_type_index, new_type_index);
	new_type = &ft_types[new_type_index];
	if (new_type_index != NODE_INDEX_NULL) {
		new_node = alloc_cds_ft_node(ft, new_type, &new_metadata);
		if (!new_node)
			return -ENOMEM;

		new_node_flag = ft_node_flag(new_node, new_type_index);

		dbg_printf("Recompact inherit from %p\n", metadata);
		if (metadata) {
			new_metadata->parent = metadata->parent;
#ifdef FEATURE_FT_SKIP_COMPRESSED
			new_metadata->skip_slot_offset = metadata->skip_slot_offset;
#endif
			new_metadata->fallback_removal_count = metadata->fallback_removal_count;
			ft_metadata_set_external_nodes(new_node_flag,
				new_metadata, metadata->external_nodes);
			ft_nr_keys_store(new_metadata,
				ft_nr_keys_get(metadata), CMM_RELAXED);
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

/*
 * Derive is_init for a set_nth call into the freshly-allocated
 * new_node.  Updates init-done state so the next call returns false.
 */
#define RECOMPACT_IS_INIT(byte_value) ({				\
	bool __is_init = false;						\
	if (new_type->type_class == FT_POPCOUNT) {			\
		__is_init = !new_init_done;				\
		new_init_done = true;					\
	} /* FT_PIGEON, FT_NULL: is_init irrelevant */			\
	__is_init;							\
})

	switch (old_type->type_class) {
	case FT_POPCOUNT:
	{
		uint8_t nr_child =
			ft_popcount_node_get_nr_child(old_type, old_node);
		unsigned int i;

		for (i = 0; i < nr_child; i++) {
			struct cds_ft_inode_flag *iter;
			uint8_t v;

			ft_popcount_node_get_ith_pos(old_type, old_node, i, &v, &iter);
			if (!iter)
				continue;
			if (mode == FT_RECOMPACT_DEL && *nullify_node_flag_ptr == iter)
				continue;
			if (new_type->popcount_2l)
				ret = ft_popcount_2l_node_set_nth(new_type,
						new_node, new_metadata, v, iter,
						RECOMPACT_IS_INIT(v));
			else if (new_type->popcount_1l)
				ret = ft_popcount_1l_node_set_nth(new_type,
						new_node, new_metadata, v, iter,
						RECOMPACT_IS_INIT(v));
			else
			ret = _ft_node_set_nth(new_type, new_node, new_node_flag,
					new_metadata, v, iter, RECOMPACT_IS_INIT(v));
			assert(!ret);
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
			if (new_type->popcount_2l)
				ret = ft_popcount_2l_node_set_nth(new_type,
						new_node, new_metadata, (uint8_t)i, iter,
						RECOMPACT_IS_INIT((uint8_t)i));
			else if (new_type->popcount_1l)
				ret = ft_popcount_1l_node_set_nth(new_type,
						new_node, new_metadata, (uint8_t)i, iter,
						RECOMPACT_IS_INIT((uint8_t)i));
			else
			ret = _ft_node_set_nth(new_type, new_node, new_node_flag,
					new_metadata, i, iter, RECOMPACT_IS_INIT((uint8_t)i));
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
		if (new_type->popcount_2l)
			ret = ft_popcount_2l_node_set_nth(new_type,
					new_node, new_metadata, n, child_node_flag,
					RECOMPACT_IS_INIT(n));
		else if (new_type->popcount_1l)
			ret = ft_popcount_1l_node_set_nth(new_type,
					new_node, new_metadata, n, child_node_flag,
					RECOMPACT_IS_INIT(n));
		else
		ret = _ft_node_set_nth(new_type, new_node, new_node_flag,
				new_metadata, n, child_node_flag,
				RECOMPACT_IS_INIT(n));
		assert(!ret);
	}

#undef RECOMPACT_IS_INIT

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
#ifdef FEATURE_FT_SKIP_COMPRESSED
		new_metadata->skip_slot_offset = old_meta->skip_slot_offset;
#endif

#ifdef FEATURE_FT_SKIP_COMPRESSED
		if (old_parent && ft_node_compressed(old_parent)) {
			struct cds_ft_compressed_node *cn =
				ft_compressed_node_ptr(old_parent);
			struct cds_ft_metadata *cn_meta =
				cds_ft_item_to_metadata(
					(struct cds_ft_inode *) cn);
			struct cds_ft_inode_flag **skip_slot =
				ft_get_skip_slot(cn_meta, ft);

			if (skip_slot &&
			    ft_node_skip_compressed(*skip_slot))
				rcu_assign_pointer(*skip_slot,
					ft_skip_compressed_flag(
						new_node_flag, cn->len));
		}
#endif
	}
	/*
	 * Reparent children to the new node.
	 *
	 * Children were copied value-for-value from the old node.
	 * Their parent pointers and skip_slot (for skip pointer
	 * children) still reference the old node, which will be
	 * freed after a grace period.  ft_set_parent updates both
	 * parent and skip_slot in one call.
	 */
	{
		switch (new_type->type_class) {
		case FT_POPCOUNT:
		{
			uint8_t nc = ft_popcount_node_get_nr_child(new_type,
					new_node);
			unsigned int i;

			for (i = 0; i < nc; i++) {
				struct cds_ft_inode_flag *iter;
				struct cds_ft_inode_flag **slot = NULL;
				uint8_t v;

				ft_popcount_node_get_ith_pos(new_type,
						new_node, i, &v, &iter);
				if (!iter)
					continue;
				ft_node_get_nth_skip(new_node_flag,
						&slot, v, FT_PF_NONE);
				ft_set_parent(iter, new_node_flag, slot);
			}
			break;
		}
		case FT_PIGEON:
		{
			unsigned int i;

			for (i = 0; i < FT_ENTRY_PER_NODE; i++) {
				struct cds_ft_inode_flag *iter;
				struct cds_ft_inode_flag **slot = NULL;

				iter = ft_pigeon_node_get_ith_pos(new_type,
						new_node, i);
				if (!iter)
					continue;
				ft_node_get_nth_skip(new_node_flag,
						&slot, i, FT_PF_NONE);
				ft_set_parent(iter, new_node_flag, slot);
			}
			break;
		}
		default:
			break;
		}
	}

	FT_TP(node_recompact, (const void *) *old_node_flag_ptr,
		(const void *) new_node_flag, (int) new_type_index);

	/* Return pointer to new recompacted node through old_node_flag_ptr */
	*old_node_flag_ptr = new_node_flag;
	if (old_node && old_node_ret)
		*old_node_ret = old_node;

	ret = 0;
end:
	return ret;
}

/*
 * Return 0 on success or negative error value on error.
 */
static
int ft_node_set_nth(struct cds_ft *ft,
		struct cds_ft_inode_flag **node_flag, uint8_t n,
		struct cds_ft_inode_flag *child_node_flag,
		struct cds_ft_inode **old_node_ret,
		struct cds_ft_metadata *metadata,
		unsigned int node_depth)
{
	int ret;
	unsigned int type_index;
	const struct cds_ft_type *type;
	struct cds_ft_inode *node;

	dbg_printf("ft_node_set_nth for n=%u, node %p\n", (unsigned int) n, ft_node_ptr(*node_flag));

	node = ft_node_ptr(*node_flag);
	type_index = ft_node_type(*node_flag);
	type = &ft_types[type_index];
	/*
	 * Top-level entry: target node is always a published internal
	 * node (descent end-point or compressed-split destination),
	 * never a freshly-allocated unpublished node.  Pass is_init =
	 * false; fresh-init cases funnel here via -ENOSPC / -ERANGE to
	 * ft_node_recompact, which uses is_init internally.
	 */
	ret = _ft_node_set_nth(type, node, *node_flag, metadata, n, child_node_flag, false);
	switch (ret) {
	case 0:
	{
		/*
		 * In-place insert succeeded on the published target node.
		 * Safe to link child -> target via parent pointer now:
		 * target is already fully valid to readers.
		 *
		 * Fetch the parent's child slot for both SKIP_X and plain
		 * COMPRESSED children: ft_set_parent's compressed branches
		 * use it to record skip_slot_offset, which chain-merge
		 * canonicalization (ft_detach_node) recovers via
		 * ft_get_skip_slot to publish the replacement at the same
		 * slot.  Without it, plain-COMPRESSED children (the path
		 * taken when CDS_FT_FLAG_SKIP_COMPRESSED is unset on the
		 * group) leave skip_slot_offset == 0 — a latent gap that
		 * trips chain-merge with parent-cn=plain-COMPRESSED.
		 */
		struct cds_ft_inode_flag **slot_ptr = NULL;

		if (ft_node_skip_compressed(child_node_flag) ||
		    ft_node_compressed(child_node_flag))
			ft_node_get_nth_skip(*node_flag, &slot_ptr, n, FT_PF_NONE);
		ft_set_parent(child_node_flag, *node_flag, slot_ptr);
		break;
	}
	case -ENOSPC:
		/* Not enough space in node, need to recompact to next type. */
		ret = ft_node_recompact(FT_RECOMPACT_ADD_NEXT, ft, type_index, type, node,
					metadata, node_flag, n, child_node_flag, NULL,
					old_node_ret, false, node_depth);
		break;
	case -ERANGE:
		/* Node needs to be recompacted. */
		ret = ft_node_recompact(FT_RECOMPACT_ADD_SAME, ft, type_index, type, node,
					metadata, node_flag, n, child_node_flag, NULL,
					old_node_ret, false, node_depth);
		break;
	}
	if (ret == 0)
		FT_TP(tree_edge_set, (const void *) ft,
			(const void *) *node_flag,
			(unsigned int) node_depth, (uint8_t) n,
			(const void *) child_node_flag);
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
		bool is_root,
		unsigned int node_depth)
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
				node_flag_ptr, old_node_ret, is_root, node_depth);
	}
	if (ret == 0)
		FT_TP(tree_edge_set, (const void *) ft,
			(const void *) *parent_node_flag_ptr,
			(unsigned int) node_depth, (uint8_t) n,
			(const void *) newptr);
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
 * Returns FT_DESCENT_CONTINUE to continue the loop,
 * FT_DESCENT_BREAK to break, or FT_DESCENT_END to jump to
 * the function's end label (with *status_ret and *found_ret set).
 */
static inline_lookup
enum ft_descent_action ft_lookup_compressed(struct cds_ft_inode_flag **node_flag_p,
		const uint8_t **key_p, const uint8_t *key_end,
		const uint8_t *key_safe_end,
		struct cds_ft_inode_flag ***path_cur_pp,
		bool track, bool track_longest,
		const uint8_t **match_key_pos_p, struct cds_ft_node **match_node_p,
		struct cds_ft_node **found_ret,
		enum cds_ft_status *status_ret,
		bool candidate)
{
	struct cds_ft_inode_flag *node_flag = *node_flag_p;
	const uint8_t *key = *key_p;
	struct cds_ft_inode_flag **path_cur = *path_cur_pp;
	struct cds_ft_compressed_node *cn = ft_compressed_node_ptr(node_flag);
	int remaining_key = (int) (key_end - key);
	int remaining_safe = (int) (key_safe_end - key);
	int cmp_len = cn->len < remaining_key ? cn->len : remaining_key;

	/* Check external_nodes at the compressed node's depth. */
	if (track) {
		struct cds_ft_metadata *cn_meta =
			cds_ft_item_to_metadata((struct cds_ft_inode *) cn);
		struct cds_ft_node *ext =
			ft_dereference_prefetch_external(cn_meta->external_nodes);

		if (ext || track_longest) {
			*match_key_pos_p = key;
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
					cmp_len, remaining_safe, false, &mpos);

			if (cmp != 0) {
				*match_key_pos_p = key + mpos;
				*match_node_p = NULL;
				*status_ret = CDS_FT_STATUS_NOT_FOUND;
				return FT_DESCENT_END;
			}
			*match_key_pos_p = key + cmp_len;
			*match_node_p = NULL;
		} else {
			if (ft_key_cmp_ordinals(key, cn->key_bytes,
					cmp_len, remaining_safe, false, NULL) != 0) {
				*status_ret = CDS_FT_STATUS_NOT_FOUND;
				return FT_DESCENT_END;
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
			*match_key_pos_p = key;
			*match_node_p = *found_ret;
		}
		return FT_DESCENT_END;
	}

	/*
	 * Advance past the compressed path.  Caller passes @path_cur at
	 * its iter-top value (no `i--` shift), so the per-byte fill is
	 * path_cur[0..cn->len-1], the final child write lands on
	 * path_cur[cn->len - 1] (overwriting the last fill slot — matches
	 * the legacy i-based code's semantics), and we leave @path_cur
	 * advanced by cn->len for the caller's next iter.
	 */
	key += cn->len;
	if (path_cur) {
		int k;

		for (k = 0; k < cn->len; k++)
			path_cur[k] =
				(struct cds_ft_inode_flag *)
				ft_compressed_node_flag(cn);
	}
	node_flag = ft_dereference_acquire_prefetch(cn->child);
	if (!node_flag) {
		*status_ret = CDS_FT_STATUS_NOT_FOUND;
		return FT_DESCENT_END;
	}
	if (path_cur) {
		path_cur[cn->len - 1] = node_flag;
		path_cur += cn->len;
		*path_cur_pp = path_cur;
	}

	*node_flag_p = node_flag;
	*key_p = key;

	if (key > key_end)
		return FT_DESCENT_BREAK;

	/*
	 * External child before end of key: record for partial
	 * tracking, set NOT_FOUND, and tell the caller to end.
	 */
	if (key < key_end && ft_node_external(node_flag)) {
		if (track) {
			*match_key_pos_p = key;
			*match_node_p = (struct cds_ft_node *) node_flag;
		}
		*status_ret = CDS_FT_STATUS_NOT_FOUND;
		return FT_DESCENT_END;
	}

	/*
	 * Track prefix match at the child node (the node after the
	 * compressed path) so callers that skip the normal tracking
	 * code via continue don't miss it.
	 */
	if (track && key < key_end && !ft_node_external(node_flag)) {
		struct cds_ft_metadata *metadata =
			cds_ft_item_to_metadata(ft_node_ptr(node_flag));
		struct cds_ft_node *ext =
			ft_dereference_prefetch_external(metadata->external_nodes);

		if (ext || track_longest) {
			*match_key_pos_p = key;
			*match_node_p = ext;
		}
	}

	return FT_DESCENT_CONTINUE;
}

/*
 * Simple compressed node traversal for read-side loops (replace,
 * count_keys_prefix).  Matches the key against the compressed path,
 * advances key/index/node_flag, and returns the loop action.
 *
 * On NOT_FOUND (mismatch or key shorter), returns FT_DESCENT_END
 * with *not_found set to true.
 * On full match with external child, returns FT_DESCENT_BREAK.
 * On full match with non-external child, returns FT_DESCENT_CONTINUE.
 */
static inline
enum ft_descent_action ft_traverse_compressed(
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
		return FT_DESCENT_END;
	}
	j = ft_match_compressed_key(key, cn, cn->len);
	if (j < cn->len) {
		*not_found = true;
		return FT_DESCENT_END;
	}
	*key_p = key + cn->len;
	*i_p = i + cn->len - 1; /* -1: for loop increments */
	*node_flag_p = cn->child;
	if (node_flag_ptr_p)
		*node_flag_ptr_p = &cn->child;
	if (!cn->child) {
		*not_found = true;
		return FT_DESCENT_END;
	}
	if (ft_node_external(cn->child))
		return FT_DESCENT_BREAK;
	return FT_DESCENT_CONTINUE;
}

/*
 * do_cds_ft_lookup_inner: descent template.
 *
 * @descend_cand and @skip_compressed are compile-time constants at
 * every call site (the four specialization wrappers below pass true /
 * false literals).  always_inline + literal arguments lets the compiler
 * constant-fold the per-iter branches on these flags:
 *   - loop-top skip-compressed resolution (line 5621 area)
 *   - get_nth dispatch (line 5687 area)
 *   - post-get_nth skip-compressed resolution (line 5719 area)
 * Eliminates the per-iter `test %sil, %sil` hot spot identified via
 * perf annotate.
 */
static inline_lookup
enum cds_ft_status do_cds_ft_lookup_inner(struct cds_ft *ft,
		const uint8_t *key, size_t _key_len, size_t _key_readable_pad,
		struct cds_ft_node **result_node,
		struct cds_ft_iter *iter,
		enum ft_prefix_tracking tracking,
		size_t *tracking_match_len,
		struct cds_ft_node **tracking_match_node,
		bool descend_cand,
		bool skip_compressed)
{
	size_t key_len = ft_key_len(ft, _key_len);
	const uint8_t *orig_key = key;
	const uint8_t *key_end = orig_key + key_len;
	/*
	 * Caller-promised input over-read horizon: @_key_readable_pad
	 * bytes past @key + @key_len are safely loadable without faulting.
	 * Used as the input-side contract for ft_key_cmp_ordinals;
	 * descent through compressed nodes forwards the remaining-safe-
	 * bytes at each level.
	 */
	size_t key_readable_pad = _key_readable_pad;
	const uint8_t *key_safe_end = key_end + key_readable_pad;
	struct cds_ft_inode_flag *node_flag;
	struct cds_ft_node *found = NULL;
	enum cds_ft_status status;
	size_t iter_path_len = 0;
	bool track = (tracking != FT_PREFIX_TRACK_NONE);
	bool track_longest = (tracking == FT_PREFIX_TRACK_LONGEST);
	/*
	 * Pointer-form prefix-tracking state.  @match_key_pos == NULL is
	 * the "no match yet" sentinel (track_longest's initial state, was
	 * FT_MATCH_LEN_NONE in the size_t form).  Non-NULL points into the
	 * @orig_key buffer at the matched position; the size_t match_len
	 * reported to the caller is computed at end: as the difference.
	 */
	const uint8_t *match_key_pos = track_longest ? NULL : orig_key;
	struct cds_ft_node *match_node = NULL;

	CDS_FT_ASSERT_RCU_READ_LOCKED(ft);

	node_flag = ft_dereference_prefetch(ft->root);

	{
	/*
	 * Capture the path-node array base BEFORE spilling @iter so the
	 * loop body and the inlined compressed-node handler can write
	 * the per-step path entry through @path_nodes without rehydrating
	 * @iter from its stack slot at every access.  NULL @iter yields a
	 * NULL @path_nodes, and the path-write conditionals fold away at
	 * compile time for callers without an iter (cds_ft_eager_lookup_key).
	 */
	/*
	 * Gate path tracking on CACHED mode: UNCACHED iters have their
	 * path discarded by iter_auto_invalidate_path() in the epilogue,
	 * so populating it byte-by-byte across the descent is pure
	 * waste.  Set @path_nodes = NULL upfront and the existing
	 * `if (path_nodes)` gates in the loop / compressed handler all
	 * DCE.  Defensive: keeps the iter prologue snapshot intact for
	 * callers that read iter->node etc. — they're independent of
	 * the path array.
	 */
	struct cds_ft_inode_flag ** const path_nodes =
		(iter && iter->path_mode == CDS_FT_ITER_PATH_CACHED) ?
			iter_path_node(iter) : NULL;
	/*
	 * Optional path-write cursor.  Tracks the slot for the current
	 * iteration's path write (advances at skip and at iter-end via
	 * lockstep with the loop's level counter).  NULL when no iter,
	 * so the per-iter advance and per-write store are compile-time
	 * DCE'd for the no-iter callers.
	 */
	struct cds_ft_inode_flag **path_cur =
		path_nodes ? path_nodes + 1 : NULL;

	if (iter)
		iter_debug_path_snapshot(iter);
	if (path_nodes)
		path_nodes[0] = node_flag;
	/*
	 * Spill @iter to its stack slot after the prologue's last
	 * in-register use of it.  The register holding @iter is then
	 * free for the rest of the function; the matching reload at
	 * end: rehydrates it for the epilogue writes.  Validations
	 * below this line goto-end through the spilled path.
	 */
	FT_SPILL_TO_STACK(iter);

	if (caa_unlikely(!valid_key_len(ft, key_len))) {
		status = CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
		goto end;
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
			match_key_pos = orig_key;
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
			match_key_pos = orig_key;
			match_node = external_nodes;
		}
	}

	/*
	 * Pre-loop non-internal root handler.  graft_swap can place a
	 * compressed node directly at ft->root when it splits inside a
	 * compressed prefix and the displaced subtree becomes the swap's
	 * root.  The hot loop assumes the dispatch parent is an internal
	 * node; resolve a non-internal root once here so the loop body
	 * stays lean (no tag-bit branch before each dispatch).
	 *
	 * Skip-encoded root is not currently produced by any mutator
	 * path, but resolve it defensively for completeness — cost is
	 * one shr+jne, DCE'd when skip_compressed compile-time false.
	 */
	if (skip_compressed &&
	    caa_unlikely(ft_node_skip_compressed(node_flag))) {
		if (!descend_cand) {
			node_flag = ft_compressed_node_flag(
				ft_skip_to_compressed(node_flag));
		} else {
			unsigned int skip = ft_skip_len(node_flag);

			if ((int) skip > (int) (key_end - key)) {
				status = CDS_FT_STATUS_NOT_FOUND;
				goto end;
			}
			key += skip;
			if (path_nodes)
				path_cur += skip;
			node_flag = ft_skip_child_ptr(node_flag);
		}
	}
	if (caa_unlikely(!ft_node_internal(node_flag))) {
		if (ft_node_compressed(node_flag)) {
			enum ft_descent_action act;

			act = ft_lookup_compressed(&node_flag, &key, key_end,
				key_safe_end,
				&path_cur,
				track, track_longest,
				&match_key_pos, &match_node, &found, &status,
				descend_cand);
			if (act == FT_DESCENT_END)
				goto end;
			if (act == FT_DESCENT_BREAK)
				goto terminal;
			/* CONTINUE: node_flag is now plain, fall through to loop. */
		} else if (ft_node_external(node_flag)) {
			goto terminal;
		} else {
			status = CDS_FT_STATUS_NOT_FOUND;
			goto end;
		}
	}

	while (key < key_end) {
		uint8_t iter_key;

		/*
		 * Loop top is lean: node_flag is internal.  ft->root was
		 * normalized by the pre-loop check above; subsequent
		 * iterations land here only on the internal fall-through
		 * path of the post-step merged handler.  No tag-bit branch
		 * before dispatch.
		 */
		iter_key = *(key++);
		/*
		 * Dispatch returns the slot value as-is, including any
		 * skip-encoded high bits.  Skip resolution is handled
		 * uniformly by the post-step skip handler below (cand
		 * mode advances key + ft_skip_child_ptr; non-cand mode
		 * converts to compressed_flag).
		 *
		 * FT_PF_DATA: the prefetch fires on the raw slot value.
		 * For regular internal/external/compressed children (the
		 * dominant case — ~97% on dns) the address is clean and
		 * the prefetch hits the right target.  For skip-encoded
		 * children (~3%) the high bits carry skip-length, the
		 * address is non-canonical, and __builtin_prefetch
		 * silently drops it (one cheap uop, no fault).
		 */
		{
			/*
			 * Eager-split: extract type_index from the tagged
			 * @node_flag once, then dispatch via the pretyped
			 * scanner.  The pre-loop normalization (or the
			 * internal-fall-through of the post-step merged
			 * handler) guarantees @node_flag is internal here,
			 * so the scanner doesn't need to re-check tag bit 0.
			 * Per-case FT_NODE_SUB_TAG_NOSKIP with a compile-
			 * time literal folds the SUB into the body load's
			 * displacement.
			 */
			unsigned long _raw = (unsigned long) node_flag;
			unsigned int _type =
				(unsigned int) ((_raw >> FT_INTERNAL_BITS) & 0x7);

			node_flag = ft_node_get_nth_skip_pretyped(node_flag,
					_type, NULL, iter_key, FT_PF_DATA);
		}
		dbg_printf("cds_ft_lookup iter key lookup %u finds node_flag %p\n",
				(unsigned int) iter_key, node_flag);
		/*
		 * "Not found" iff the slot is NULL: ft_node_get_nth*
		 * returns NULL when the parent isn't internal, and an
		 * empty slot is stored as NULL.  All valid pointers
		 * (any tag) carry a non-zero underlying address, so the
		 * full ft_node_ptr() unmasking would always answer the
		 * same as a plain NULL check, at the cost of the (v & 1)
		 * branch in the per-step dependency chain.
		 */
		if (!node_flag) {
			status = CDS_FT_STATUS_NOT_FOUND;
			goto end;
		}
		/*
		 * Skip-compressed pointer from child slot.
		 *
		 * Non-candidate: convert to compressed flag and fall
		 * through to the merged handler below (which calls
		 * ft_lookup_compressed).
		 *
		 * Candidate: resolve the skip (advance past the
		 * compressed path without comparison).
		 */
		if (skip_compressed && caa_unlikely(ft_node_skip_compressed(node_flag))) {
			if (!descend_cand) {
				node_flag = ft_compressed_node_flag(
					ft_skip_to_compressed(node_flag));
			} else {
				unsigned int skip = ft_skip_len(node_flag);
				int remaining = (int) (key_end - key);

				if ((int) skip > remaining) {
					status = CDS_FT_STATUS_NOT_FOUND;
					goto end;
				}
				key += skip;
				if (path_nodes)
					path_cur += skip;
				node_flag = ft_skip_child_ptr(node_flag);
			}
		}
		if (path_nodes)
			*path_cur = node_flag;
		/*
		 * Merged post-step handler for non-internal results.
		 * Bundles what was a loop-top !internal slow path with
		 * the separate post-step external check.
		 *
		 * Compressed reachable in non-cand mode and also in
		 * cand mode for compressed paths longer than
		 * FT_SKIP_LEN_MAX: those keep the regular compressed
		 * pointer (skip-pointer length encoding wouldn't fit),
		 * so the slot does not carry a skip pointer and the
		 * pre-step skip handler above didn't resolve it.
		 * ft_lookup_compressed(candidate=true) advances past
		 * the compressed path without comparison.
		 */
		if (caa_unlikely(!ft_node_internal(node_flag))) {
			if (caa_unlikely(ft_node_compressed(node_flag))) {
				enum ft_descent_action act;

				/*
				 * The *path_cur = node_flag write above filled
				 * path[K] with compressed_flag.  Advance one slot
				 * so ft_lookup_compressed's fill/overwrite land
				 * at the right depth (matches OLD loop-top
				 * compressed-handler semantics).
				 */
				if (path_nodes)
					path_cur++;
				act = ft_lookup_compressed(&node_flag, &key, key_end,
					key_safe_end,
					&path_cur,
					track, track_longest,
					&match_key_pos, &match_node, &found, &status,
					descend_cand);
				if (act == FT_DESCENT_END)
					goto end;
				if (act == FT_DESCENT_BREAK)
					break;
				continue;
			}
			if (ft_node_external(node_flag)) {
				if (key < key_end) {
					if (track) {
						match_key_pos = key;
						match_node = (struct cds_ft_node *) node_flag;
					}
					status = CDS_FT_STATUS_NOT_FOUND;
					goto end;
				}
				/* terminal external — fall through to path_cur++ */
			} else {
				status = CDS_FT_STATUS_NOT_FOUND;
				goto end;
			}
		} else if (track && key < key_end) {
			/*
			 * Track prefix match on the internal child.  DCE'd
			 * when track=false (cds_ft_eager_lookup_key).
			 */
			const struct cds_ft_type *type = &ft_types[ft_node_type(node_flag)];
			struct cds_ft_metadata *metadata = cds_ft_item_to_metadata_fast(
					ft_node_ptr(node_flag), type->order);
			struct cds_ft_node *external_nodes = ft_dereference_prefetch_external(metadata->external_nodes);

			if (external_nodes || track_longest) {
				match_key_pos = key;
				match_node = external_nodes;
			}
		}
		/* Iter-end advance for lockstep with key/level. */
		if (path_nodes)
			path_cur++;
	}

terminal:
	/*
	 * Compute iter_path_len at end from @path_cur (last write
	 * position + 1 advance per iter, all in lockstep with the
	 * level counter).  Also reached from the pre-loop non-internal
	 * root handler, which already advanced path_cur via
	 * ft_lookup_compressed (compressed root) or left it at the
	 * initial slot (external root).
	 */
	if (path_nodes)
		iter_path_len = path_cur - path_nodes;
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
			match_key_pos = key_end;
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
			match_key_pos = key_end;
			match_node = found;
		}
	} else {
		found = (struct cds_ft_node *) node_flag;
		status = CDS_FT_STATUS_OK;
		if (track) {
			match_key_pos = key_end;
			match_node = found;
		}
	}

end:
	/* Bring @iter back into a register for the epilogue writes. */
	FT_RELOAD_FROM_STACK(iter);
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
		*tracking_match_len = match_key_pos ?
			(size_t) (match_key_pos - orig_key) :
			FT_MATCH_LEN_NONE;
		*tracking_match_node = match_node;
	}
	return status;
}

/*
 * Four specialized instantiations of do_cds_ft_lookup_inner.  Each
 * wrapper passes a const (descend_cand, skip_compressed) pair so the
 * always_inline body collapses to a single specialized descent loop.
 * inline_lookup (always_inline) keeps them inlined into the dispatcher
 * and ultimately into the public entry points, so per-caller constants
 * (iter == NULL, candidate, tracking) also DCE the body.
 */
static inline_lookup
enum cds_ft_status do_cds_ft_lookup_dc_sc(struct cds_ft *ft,
		const uint8_t *key, size_t _key_len, size_t _key_readable_pad,
		struct cds_ft_node **result_node,
		struct cds_ft_iter *iter,
		enum ft_prefix_tracking tracking,
		size_t *tracking_match_len,
		struct cds_ft_node **tracking_match_node)
{
	return do_cds_ft_lookup_inner(ft, key, _key_len, _key_readable_pad,
			result_node, iter,
			tracking, tracking_match_len, tracking_match_node,
			true, true);
}

static inline_lookup
enum cds_ft_status do_cds_ft_lookup_dc_nosc(struct cds_ft *ft,
		const uint8_t *key, size_t _key_len, size_t _key_readable_pad,
		struct cds_ft_node **result_node,
		struct cds_ft_iter *iter,
		enum ft_prefix_tracking tracking,
		size_t *tracking_match_len,
		struct cds_ft_node **tracking_match_node)
{
	return do_cds_ft_lookup_inner(ft, key, _key_len, _key_readable_pad,
			result_node, iter,
			tracking, tracking_match_len, tracking_match_node,
			true, false);
}

static inline_lookup
enum cds_ft_status do_cds_ft_lookup_nodc_sc(struct cds_ft *ft,
		const uint8_t *key, size_t _key_len, size_t _key_readable_pad,
		struct cds_ft_node **result_node,
		struct cds_ft_iter *iter,
		enum ft_prefix_tracking tracking,
		size_t *tracking_match_len,
		struct cds_ft_node **tracking_match_node)
{
	return do_cds_ft_lookup_inner(ft, key, _key_len, _key_readable_pad,
			result_node, iter,
			tracking, tracking_match_len, tracking_match_node,
			false, true);
}

static inline_lookup
enum cds_ft_status do_cds_ft_lookup_nodc_nosc(struct cds_ft *ft,
		const uint8_t *key, size_t _key_len, size_t _key_readable_pad,
		struct cds_ft_node **result_node,
		struct cds_ft_iter *iter,
		enum ft_prefix_tracking tracking,
		size_t *tracking_match_len,
		struct cds_ft_node **tracking_match_node)
{
	return do_cds_ft_lookup_inner(ft, key, _key_len, _key_readable_pad,
			result_node, iter,
			tracking, tracking_match_len, tracking_match_node,
			false, false);
}

/*
 * @candidate: when true, skip key comparison at compressed nodes
 * during traversal (patricia-like mode).  The returned node is a
 * candidate that must be verified by the caller against their
 * stored key.  Constant-folded at each call site.
 *
 * 4-way dispatch on (descend_cand, skip_compressed).
 */
static inline_lookup
enum cds_ft_status do_cds_ft_lookup(struct cds_ft *ft,
		const uint8_t *key, size_t _key_len, size_t _key_readable_pad,
		struct cds_ft_node **result_node,
		struct cds_ft_iter *iter,
		enum ft_prefix_tracking tracking,
		size_t *tracking_match_len,
		struct cds_ft_node **tracking_match_node,
		bool candidate)
{
	bool skip_compressed = ft_group_skip_compressed(ft->group);

	if (candidate) {
		if (skip_compressed)
			return do_cds_ft_lookup_dc_sc(ft, key, _key_len, _key_readable_pad,
					result_node, iter, tracking,
					tracking_match_len, tracking_match_node);
		return do_cds_ft_lookup_dc_nosc(ft, key, _key_len, _key_readable_pad,
				result_node, iter, tracking,
				tracking_match_len, tracking_match_node);
	}
	if (skip_compressed)
		return do_cds_ft_lookup_nodc_sc(ft, key, _key_len, _key_readable_pad,
				result_node, iter, tracking,
				tracking_match_len, tracking_match_node);
	return do_cds_ft_lookup_nodc_nosc(ft, key, _key_len, _key_readable_pad,
			result_node, iter, tracking,
			tracking_match_len, tracking_match_node);
}

/*
 * Per-API cluster sub-sections.  Each public-API stub lives in
 * .text.hot.cds_ft_<api>.0_stub; its primary inner in
 * .text.hot.cds_ft_<api>.1_primary; secondary variants in
 * .text.hot.cds_ft_<api>.2_secondary.
 *
 * The linker script ft-lookup-layout.ld groups each cluster
 * (per public API) into one output section, page-aligned, with
 * sub-sections sorted alphabetically — stub first, primary
 * second, secondary last.  This ensures the bench's hot path
 * (call → stub → primary inner) touches a single iTLB page on
 * the critical setup, and primary's tail spilling past 4 KiB
 * doesn't cost an extra iTLB miss on every lookup because the
 * descent's PC stays on page 1 during the first iterations.
 */
/*
 * Cluster layout: dispatch at page start, fast path next, slow paths
 * trailing.  The linker script (ft-lookup-layout.ld) gathers each
 * cluster's sub-sections in alphabetical name order, page-aligned
 * at the start:
 *
 *   ALIGN(4096) ┐
 *               │  .text.hot.cds_ft_<api>.0_dispatch  — public stub
 *               │  .text.hot.cds_ft_<api>.1_fast      — primary inner
 *               │                                      (spec_validated +
 *               │                                       skip_compressed)
 *               │  .text.hot.cds_ft_<api>.2_slow      — slow paths:
 *               │                                      precise_*, *_nosc,
 *               │                                      *_nonidentity
 *
 * For the bench's hot path (call → stub → primary inner), the call
 * lands at the cluster's page boundary; the stub's indirect jmp
 * forwards into the primary on the same page.  Slow paths trail
 * later in the cluster (later addresses, possibly later pages).
 * The whole cluster shares iTLB/icache locality.
 *
 * Direction of the indirect jmp (forward to primary vs backward to
 * a slow path) is not a factor — unconditional jmps don't use the
 * "backward predicted taken" heuristic (that applies only to
 * conditional branches with a cold BPB).  Layout experiments
 * 2026-05-22 confirmed stub-FIRST / stub-LAST / stub on different
 * page from primary all measure within ~1 ns on ft_specv_local
 * dns load-names ST.  Dispatch-FIRST was selected for clarity:
 * the bench's CALL lands at the page-aligned cluster start,
 * forward into the next-most-likely target.
 */
#define FT_LOOKUP_DISPATCH(name)	\
	__attribute__((section(".text.hot.cds_ft_" name ".0_dispatch")))
#define FT_LOOKUP_FAST_PATH(name)	\
	__attribute__((section(".text.hot.cds_ft_" name ".1_fast")))
#define FT_LOOKUP_SLOW_PATH(name)	\
	__attribute__((section(".text.hot.cds_ft_" name ".2_slow")))

/*
 * Specialized lookup_key/lookup_candidate_key inner functions.
 * Each bakes the (descend_cand, skip_compressed) pair into a
 * literal-arg call to the matching always_inline
 * wrapper — the wrapper then inlines into the inner with all
 * per-iter branches on those constants folded out.  Each is
 * referenced via the function pointers installed on struct cds_ft
 * by ft_install_lookup_ops (see cds_ft_create), so gcc cannot
 * inline them into the caller — they exist as real symbols and
 * the public entry point dispatches via an indirect tail call.
 *
 * Identity key_map is implied for the four hot-path inners
 * (FT_PREFIX_TRACK_NONE, key passed directly, no ordinals[]).
 * Non-identity callers route through ft_lookup_*_nonidentity
 * below, which carry a FT_MAX_KEY_LEN stack buffer.
 */
static FT_LOOKUP_FAST_PATH("lookup_key")
enum cds_ft_status ft_lookup_precise_sc(struct cds_ft *ft,
		const uint8_t *key, size_t key_len, size_t key_readable_pad,
		struct cds_ft_node **result_node)
{
	enum cds_ft_status status;
	key_len = ft_key_len(ft, key_len);
	if (!valid_key_len(ft, key_len))
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	CDS_FT_SCOPED_READER(ft);
	FT_TP_KEY(lookup_key_enter, ft, key, key_len);
	status = do_cds_ft_lookup_nodc_sc(ft, key, key_len, key_readable_pad,
			result_node, NULL,
			FT_PREFIX_TRACK_NONE, NULL, NULL);
	FT_TP(lookup_key_exit, (int) status);
	return status;
}

static FT_LOOKUP_SLOW_PATH("lookup_key")
enum cds_ft_status ft_lookup_precise_nosc(struct cds_ft *ft,
		const uint8_t *key, size_t key_len, size_t key_readable_pad,
		struct cds_ft_node **result_node)
{
	enum cds_ft_status status;
	key_len = ft_key_len(ft, key_len);
	if (!valid_key_len(ft, key_len))
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	CDS_FT_SCOPED_READER(ft);
	FT_TP_KEY(lookup_key_enter, ft, key, key_len);
	status = do_cds_ft_lookup_nodc_nosc(ft, key, key_len, key_readable_pad,
			result_node, NULL,
			FT_PREFIX_TRACK_NONE, NULL, NULL);
	FT_TP(lookup_key_exit, (int) status);
	return status;
}

static FT_LOOKUP_FAST_PATH("lookup_candidate_key")
enum cds_ft_status ft_lookup_cand_sc(struct cds_ft *ft,
		const uint8_t *key, size_t key_len, size_t key_readable_pad,
		struct cds_ft_node **result_node)
{
	key_len = ft_key_len(ft, key_len);
	if (!valid_key_len(ft, key_len))
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	CDS_FT_SCOPED_READER(ft);
	return do_cds_ft_lookup_dc_sc(ft, key, key_len, key_readable_pad,
			result_node, NULL,
			FT_PREFIX_TRACK_NONE, NULL, NULL);
}

static FT_LOOKUP_SLOW_PATH("lookup_candidate_key")
enum cds_ft_status ft_lookup_cand_nosc(struct cds_ft *ft,
		const uint8_t *key, size_t key_len, size_t key_readable_pad,
		struct cds_ft_node **result_node)
{
	key_len = ft_key_len(ft, key_len);
	if (!valid_key_len(ft, key_len))
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	CDS_FT_SCOPED_READER(ft);
	return do_cds_ft_lookup_dc_nosc(ft, key, key_len, key_readable_pad,
			result_node, NULL,
			FT_PREFIX_TRACK_NONE, NULL, NULL);
}

/*
 * Non-identity key_map fallback (key + candidate variants).
 * Allocates a FT_MAX_KEY_LEN stack buffer for the ordinals[]
 * remap and routes through do_cds_ft_lookup, which re-derives the
 * descend_cand path from group state at runtime.
 * Non-identity maps are rare (only set via
 * cds_ft_group_attr_set_key_map) so the extra dispatch hop is
 * not worth specializing further.
 */
static FT_LOOKUP_SLOW_PATH("lookup_key")
enum cds_ft_status ft_lookup_key_nonidentity(struct cds_ft *ft,
		const uint8_t *key, size_t key_len,
		size_t key_readable_pad,
		struct cds_ft_node **result_node)
{
	uint8_t ordinals[FT_MAX_KEY_LEN];
	enum cds_ft_status status;

	(void) key_readable_pad;	/* ordinals[] is a stack buffer of
					   fixed size; readable_pad doesn't
					   carry across the remap. */
	key_len = ft_key_len(ft, key_len);
	if (!valid_key_len(ft, key_len))
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	CDS_FT_SCOPED_READER(ft);
	ft_key_to_ordinals(ordinals, key, key_len, &ft->group->key_map);
	FT_TP_KEY(lookup_key_enter, ft, ordinals, key_len);
	status = do_cds_ft_lookup(ft, ordinals, key_len,
			FT_KEY_READABLE_PAD, result_node, NULL,
			FT_PREFIX_TRACK_NONE, NULL, NULL, false);
	FT_TP(lookup_key_exit, (int) status);
	return status;
}

static FT_LOOKUP_SLOW_PATH("lookup_candidate_key")
enum cds_ft_status ft_lookup_candidate_key_nonidentity(struct cds_ft *ft,
		const uint8_t *key, size_t key_len,
		size_t key_readable_pad,
		struct cds_ft_node **result_node)
{
	uint8_t ordinals[FT_MAX_KEY_LEN];

	(void) key_readable_pad;
	key_len = ft_key_len(ft, key_len);
	if (!valid_key_len(ft, key_len))
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	CDS_FT_SCOPED_READER(ft);
	ft_key_to_ordinals(ordinals, key, key_len, &ft->group->key_map);
	return do_cds_ft_lookup(ft, ordinals, key_len,
			FT_KEY_READABLE_PAD, result_node, NULL,
			FT_PREFIX_TRACK_NONE, NULL, NULL, true);
}

/*
 * Install per-API lookup function pointers on @ft based on group
 * flags.  Called once at cds_ft_create.  Pointers are stable for
 * the trie's lifetime because the group flags (key_map.identity,
 * CDS_FT_FLAG_SKIP_COMPRESSED) are immutable after group creation.
 */
/* Forward decls for ft_install_lookup_ops — definitions follow
 * after cds_ft_lookup_candidate_key. */
static enum cds_ft_status ft_lookup_iter_precise_sc(struct cds_ft *,
		struct cds_ft_iter *);
static enum cds_ft_status ft_lookup_iter_precise_nosc(struct cds_ft *,
		struct cds_ft_iter *);
static enum cds_ft_status ft_lookup_partial_key_sc(struct cds_ft *,
		const uint8_t *, size_t, size_t *, struct cds_ft_node **);
static enum cds_ft_status ft_lookup_partial_key_nosc(struct cds_ft *,
		const uint8_t *, size_t, size_t *, struct cds_ft_node **);
static enum cds_ft_status ft_lookup_partial_key_nonidentity(struct cds_ft *,
		const uint8_t *, size_t, size_t *, struct cds_ft_node **);
static enum cds_ft_status ft_lookup_partial_iter_sc(struct cds_ft *,
		struct cds_ft_iter *);
static enum cds_ft_status ft_lookup_partial_iter_nosc(struct cds_ft *,
		struct cds_ft_iter *);
static enum cds_ft_status ft_lookup_longest_match_key_sc(struct cds_ft *,
		const uint8_t *, size_t, size_t *, struct cds_ft_node **);
static enum cds_ft_status ft_lookup_longest_match_key_nosc(struct cds_ft *,
		const uint8_t *, size_t, size_t *, struct cds_ft_node **);
static enum cds_ft_status ft_lookup_longest_match_key_nonidentity(struct cds_ft *,
		const uint8_t *, size_t, size_t *, struct cds_ft_node **);
static enum cds_ft_status ft_lookup_longest_match_iter_sc(struct cds_ft *,
		struct cds_ft_iter *);
static enum cds_ft_status ft_lookup_longest_match_iter_nosc(struct cds_ft *,
		struct cds_ft_iter *);

static
void ft_install_lookup_ops(struct cds_ft *ft)
{
	const struct cds_ft_group *group = ft->group;
	bool sc = ft_group_skip_compressed(group);

	/*
	 * Iter-form lookup_iter_fn: iter always carries ordinals-mapped
	 * key bytes (see cds_ft_iter_set_key), so the non-identity key_map
	 * case is handled at iter-set time and the lookup-time inner only
	 * needs (skip_compressed) specialization.
	 */
	ft->lookup_iter_fn = sc
		? ft_lookup_iter_precise_sc : ft_lookup_iter_precise_nosc;
	/*
	 * Partial-match (tracking=PARTIAL) is precise descent.  Iter form
	 * has no non-identity issue (iter holds ordinals already).
	 */
	ft->lookup_partial_iter_fn = sc
		? ft_lookup_partial_iter_sc : ft_lookup_partial_iter_nosc;
	/*
	 * Longest-match (tracking=LONGEST) is also always precise
	 * descent.  Same shape as partial.
	 */
	ft->lookup_longest_match_iter_fn = sc
		? ft_lookup_longest_match_iter_sc
		: ft_lookup_longest_match_iter_nosc;
	if (caa_unlikely(!group->key_map.identity)) {
		ft->lookup_key_fn = ft_lookup_key_nonidentity;
		ft->lookup_candidate_key_fn = ft_lookup_candidate_key_nonidentity;
		ft->lookup_partial_key_fn = ft_lookup_partial_key_nonidentity;
		ft->lookup_longest_match_key_fn = ft_lookup_longest_match_key_nonidentity;
		return;
	}
	ft->lookup_key_fn = sc
		? ft_lookup_precise_sc : ft_lookup_precise_nosc;
	ft->lookup_candidate_key_fn = sc
		? ft_lookup_cand_sc : ft_lookup_cand_nosc;
	ft->lookup_partial_key_fn = sc
		? ft_lookup_partial_key_sc : ft_lookup_partial_key_nosc;
	ft->lookup_longest_match_key_fn = sc
		? ft_lookup_longest_match_key_sc
		: ft_lookup_longest_match_key_nosc;
}

/*
 * Public lookup_key / lookup_candidate_key entries.
 *
 * Reduced to a single indirect tail-call through the function
 * pointer installed on @ft at create time (see
 * ft_install_lookup_ops).  gcc -O2 emits a sibling call
 * (`mov 0x?(%rdi),%rax; jmp *%rax`) — no stack frame, no
 * callee-save save/restore, no per-call branch on group shape.
 * The branch predictor stores the fn-ptr target once per trie
 * and predicts it for every subsequent lookup.
 */
FT_LOOKUP_DISPATCH("lookup_key")
enum cds_ft_status cds_ft_eager_lookup_key(struct cds_ft *ft,
		const uint8_t *key, size_t _key_len, size_t _key_readable_pad,
		struct cds_ft_node **result_node)
{
	return (*ft->lookup_key_fn)(ft, key, _key_len,
			_key_readable_pad, result_node);
}

/*
 * cds_ft_lookup_candidate_key - Fast candidate lookup.
 *
 * Skips key comparison at compressed nodes during traversal,
 * returning a candidate node that may not be an exact match.
 * The caller MUST verify the returned node's key matches the
 * lookup key.  If it does not match, the key is not in the trie.
 *
 * This is faster than cds_ft_eager_lookup_key for workloads with long
 * compressed paths (e.g. reverse DNS, file paths) because it
 * eliminates per-node key comparisons, doing a single verification
 * at the end instead.
 */
FT_LOOKUP_DISPATCH("lookup_candidate_key")
enum cds_ft_status cds_ft_lookup_candidate_key(struct cds_ft *ft,
		const uint8_t *key, size_t _key_len, size_t _key_readable_pad,
		struct cds_ft_node **result_node)
{
	return (*ft->lookup_candidate_key_fn)(ft, key, _key_len,
			_key_readable_pad, result_node);
}

/*
 * Specialized iter-form lookup inners.  iter is always non-NULL on
 * this path (path tracking is the whole point of the iter API), so
 * each inner bakes that into the wrapper call along with the
 * (descend_cand, skip_compressed) pair.  iter already holds
 * ordinals-mapped key bytes (see cds_ft_iter_set_key: non-identity
 * key_map remapping happens at iter-set time, not at lookup time),
 * so there is no non-identity branch on this path.
 */
static FT_LOOKUP_FAST_PATH("lookup")
enum cds_ft_status ft_lookup_iter_precise_sc(struct cds_ft *ft,
		struct cds_ft_iter *iter)
{
	enum cds_ft_status status;
	CDS_FT_SCOPED_READER(ft);
	FT_TP_ITER_KEY(lookup_enter, iter);
	status = do_cds_ft_lookup_nodc_sc(ft, iter_key(iter), iter->key_len,
			FT_KEY_READABLE_PAD, NULL, iter,
			FT_PREFIX_TRACK_NONE, NULL, NULL);
	FT_TP(lookup_exit, (int) status);
	return status;
}

static FT_LOOKUP_SLOW_PATH("lookup")
enum cds_ft_status ft_lookup_iter_precise_nosc(struct cds_ft *ft,
		struct cds_ft_iter *iter)
{
	enum cds_ft_status status;
	CDS_FT_SCOPED_READER(ft);
	FT_TP_ITER_KEY(lookup_enter, iter);
	status = do_cds_ft_lookup_nodc_nosc(ft, iter_key(iter), iter->key_len,
			FT_KEY_READABLE_PAD, NULL, iter,
			FT_PREFIX_TRACK_NONE, NULL, NULL);
	FT_TP(lookup_exit, (int) status);
	return status;
}

FT_LOOKUP_DISPATCH("lookup")
enum cds_ft_status cds_ft_lookup(struct cds_ft *ft,
		struct cds_ft_iter *iter)
{
	return (*ft->lookup_iter_fn)(ft, iter);
}

/*
 * Specialized partial_key inners (tracking=PARTIAL, identity key_map).
 * Always precise descent (descend_cand=false) because partial-match
 * needs per-step compressed verification.
 */
static FT_LOOKUP_FAST_PATH("lookup_partial_key")
enum cds_ft_status ft_lookup_partial_key_sc(struct cds_ft *ft,
		const uint8_t *key, size_t _key_len, size_t *match_len,
		struct cds_ft_node **result_node)
{
	struct cds_ft_node *partial_node = NULL;
	size_t partial_len = 0;
	size_t key_len = ft_key_len(ft, _key_len);

	if (!valid_key_len(ft, key_len)) {
		*match_len = 0;
		*result_node = NULL;
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}
	CDS_FT_SCOPED_READER(ft);
	do_cds_ft_lookup_nodc_sc(ft, key, key_len, 0, NULL, NULL,
			FT_PREFIX_TRACK_PARTIAL, &partial_len, &partial_node);
	*match_len = partial_len;
	*result_node = partial_node;
	return partial_node ? CDS_FT_STATUS_OK : CDS_FT_STATUS_NOT_FOUND;
}

static FT_LOOKUP_SLOW_PATH("lookup_partial_key")
enum cds_ft_status ft_lookup_partial_key_nosc(struct cds_ft *ft,
		const uint8_t *key, size_t _key_len, size_t *match_len,
		struct cds_ft_node **result_node)
{
	struct cds_ft_node *partial_node = NULL;
	size_t partial_len = 0;
	size_t key_len = ft_key_len(ft, _key_len);

	if (!valid_key_len(ft, key_len)) {
		*match_len = 0;
		*result_node = NULL;
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}
	CDS_FT_SCOPED_READER(ft);
	do_cds_ft_lookup_nodc_nosc(ft, key, key_len, 0, NULL, NULL,
			FT_PREFIX_TRACK_PARTIAL, &partial_len, &partial_node);
	*match_len = partial_len;
	*result_node = partial_node;
	return partial_node ? CDS_FT_STATUS_OK : CDS_FT_STATUS_NOT_FOUND;
}

/*
 * Non-identity key_map fallback for partial_key.  Routes through
 * the generic do_cds_ft_lookup dispatcher because non-identity is
 * rare; ordinals[] is allocated on stack inside this fn so
 * call/return overhead is acceptable.
 */
static FT_LOOKUP_SLOW_PATH("lookup_partial_key")
enum cds_ft_status ft_lookup_partial_key_nonidentity(struct cds_ft *ft,
		const uint8_t *key, size_t _key_len, size_t *match_len,
		struct cds_ft_node **result_node)
{
	struct cds_ft_node *partial_node = NULL;
	size_t partial_len = 0;
	size_t key_len = ft_key_len(ft, _key_len);
	uint8_t ordinals[FT_MAX_KEY_LEN];

	if (!valid_key_len(ft, key_len)) {
		*match_len = 0;
		*result_node = NULL;
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}
	CDS_FT_SCOPED_READER(ft);
	ft_key_to_ordinals(ordinals, key, key_len, &ft->group->key_map);
	do_cds_ft_lookup(ft, ordinals, key_len, 0, NULL, NULL,
			FT_PREFIX_TRACK_PARTIAL, &partial_len, &partial_node,
			false);
	*match_len = partial_len;
	*result_node = partial_node;
	return partial_node ? CDS_FT_STATUS_OK : CDS_FT_STATUS_NOT_FOUND;
}

FT_LOOKUP_DISPATCH("lookup_partial_key")
enum cds_ft_status cds_ft_lookup_partial_key(struct cds_ft *ft,
		const uint8_t *key, size_t _key_len, size_t *match_len,
		struct cds_ft_node **result_node)
{
	return (*ft->lookup_partial_key_fn)(ft, key, _key_len, match_len,
			result_node);
}

/*
 * Specialized partial (iter form) inners.  Same as lookup_iter_*
 * but with tracking=PARTIAL and the post-descent iter override.
 */
static FT_LOOKUP_FAST_PATH("lookup_partial")
enum cds_ft_status ft_lookup_partial_iter_sc(struct cds_ft *ft,
		struct cds_ft_iter *iter)
{
	struct cds_ft_node *partial_node = NULL;
	size_t partial_len = 0;

	CDS_FT_SCOPED_READER(ft);
	do_cds_ft_lookup_nodc_sc(ft, iter_key(iter), iter->key_len,
			FT_KEY_READABLE_PAD, NULL, iter,
			FT_PREFIX_TRACK_PARTIAL, &partial_len, &partial_node);
	iter->node = partial_node;
	iter->key_len = partial_len;
	iter->path_len = partial_len + 1;
	iter->status = partial_node ? CDS_FT_STATUS_OK : CDS_FT_STATUS_NOT_FOUND;
	return iter->status;
}

static FT_LOOKUP_SLOW_PATH("lookup_partial")
enum cds_ft_status ft_lookup_partial_iter_nosc(struct cds_ft *ft,
		struct cds_ft_iter *iter)
{
	struct cds_ft_node *partial_node = NULL;
	size_t partial_len = 0;

	CDS_FT_SCOPED_READER(ft);
	do_cds_ft_lookup_nodc_nosc(ft, iter_key(iter), iter->key_len,
			FT_KEY_READABLE_PAD, NULL, iter,
			FT_PREFIX_TRACK_PARTIAL, &partial_len, &partial_node);
	iter->node = partial_node;
	iter->key_len = partial_len;
	iter->path_len = partial_len + 1;
	iter->status = partial_node ? CDS_FT_STATUS_OK : CDS_FT_STATUS_NOT_FOUND;
	return iter->status;
}

FT_LOOKUP_DISPATCH("lookup_partial")
enum cds_ft_status cds_ft_lookup_partial(struct cds_ft *ft,
		struct cds_ft_iter *iter)
{
	return (*ft->lookup_partial_iter_fn)(ft, iter);
}

/*
 * Helper: derive the public-API return value for longest_match
 * from the descent result.  Sets *match_len / *result_node and
 * returns CDS_FT_STATUS_OK / NOT_FOUND / INTERNAL_MATCH or a
 * negative status when the descent itself failed.
 */
static inline
enum cds_ft_status ft_lookup_longest_match_key_finish(
		enum cds_ft_status ret,
		size_t longest_len, struct cds_ft_node *match_node,
		size_t *match_len, struct cds_ft_node **result_node)
{
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

static FT_LOOKUP_FAST_PATH("lookup_longest_match_key")
enum cds_ft_status ft_lookup_longest_match_key_sc(struct cds_ft *ft,
		const uint8_t *key, size_t _key_len, size_t *match_len,
		struct cds_ft_node **result_node)
{
	struct cds_ft_node *match_node = NULL;
	size_t longest_len = 0;
	enum cds_ft_status ret;
	size_t key_len = ft_key_len(ft, _key_len);

	if (!valid_key_len(ft, key_len)) {
		*match_len = 0;
		*result_node = NULL;
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}
	CDS_FT_SCOPED_READER(ft);
	ret = do_cds_ft_lookup_nodc_sc(ft, key, key_len, 0, NULL, NULL,
			FT_PREFIX_TRACK_LONGEST, &longest_len, &match_node);
	return ft_lookup_longest_match_key_finish(ret, longest_len,
			match_node, match_len, result_node);
}

static FT_LOOKUP_SLOW_PATH("lookup_longest_match_key")
enum cds_ft_status ft_lookup_longest_match_key_nosc(struct cds_ft *ft,
		const uint8_t *key, size_t _key_len, size_t *match_len,
		struct cds_ft_node **result_node)
{
	struct cds_ft_node *match_node = NULL;
	size_t longest_len = 0;
	enum cds_ft_status ret;
	size_t key_len = ft_key_len(ft, _key_len);

	if (!valid_key_len(ft, key_len)) {
		*match_len = 0;
		*result_node = NULL;
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}
	CDS_FT_SCOPED_READER(ft);
	ret = do_cds_ft_lookup_nodc_nosc(ft, key, key_len, 0, NULL, NULL,
			FT_PREFIX_TRACK_LONGEST, &longest_len, &match_node);
	return ft_lookup_longest_match_key_finish(ret, longest_len,
			match_node, match_len, result_node);
}

static FT_LOOKUP_SLOW_PATH("lookup_longest_match_key")
enum cds_ft_status ft_lookup_longest_match_key_nonidentity(struct cds_ft *ft,
		const uint8_t *key, size_t _key_len, size_t *match_len,
		struct cds_ft_node **result_node)
{
	struct cds_ft_node *match_node = NULL;
	size_t longest_len = 0;
	enum cds_ft_status ret;
	size_t key_len = ft_key_len(ft, _key_len);
	uint8_t ordinals[FT_MAX_KEY_LEN];

	if (!valid_key_len(ft, key_len)) {
		*match_len = 0;
		*result_node = NULL;
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}
	CDS_FT_SCOPED_READER(ft);
	ft_key_to_ordinals(ordinals, key, key_len, &ft->group->key_map);
	ret = do_cds_ft_lookup(ft, ordinals, key_len, 0, NULL, NULL,
			FT_PREFIX_TRACK_LONGEST, &longest_len, &match_node, false);
	return ft_lookup_longest_match_key_finish(ret, longest_len,
			match_node, match_len, result_node);
}

FT_LOOKUP_DISPATCH("lookup_longest_match_key")
enum cds_ft_status cds_ft_lookup_longest_match_key(struct cds_ft *ft,
		const uint8_t *key, size_t _key_len, size_t *match_len,
		struct cds_ft_node **result_node)
{
	return (*ft->lookup_longest_match_key_fn)(ft, key, _key_len,
			match_len, result_node);
}

/*
 * Helper: write descent result back into iter for longest_match iter.
 */
static inline
enum cds_ft_status ft_lookup_longest_match_iter_finish(
		enum cds_ft_status ret,
		size_t longest_len, struct cds_ft_node *match_node,
		struct cds_ft_iter *iter)
{
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

static FT_LOOKUP_FAST_PATH("lookup_longest_match")
enum cds_ft_status ft_lookup_longest_match_iter_sc(struct cds_ft *ft,
		struct cds_ft_iter *iter)
{
	struct cds_ft_node *match_node = NULL;
	size_t longest_len = 0;
	enum cds_ft_status ret;

	CDS_FT_SCOPED_READER(ft);
	ret = do_cds_ft_lookup_nodc_sc(ft, iter_key(iter), iter->key_len,
			FT_KEY_READABLE_PAD, NULL, iter,
			FT_PREFIX_TRACK_LONGEST, &longest_len, &match_node);
	return ft_lookup_longest_match_iter_finish(ret, longest_len,
			match_node, iter);
}

static FT_LOOKUP_SLOW_PATH("lookup_longest_match")
enum cds_ft_status ft_lookup_longest_match_iter_nosc(struct cds_ft *ft,
		struct cds_ft_iter *iter)
{
	struct cds_ft_node *match_node = NULL;
	size_t longest_len = 0;
	enum cds_ft_status ret;

	CDS_FT_SCOPED_READER(ft);
	ret = do_cds_ft_lookup_nodc_nosc(ft, iter_key(iter), iter->key_len,
			FT_KEY_READABLE_PAD, NULL, iter,
			FT_PREFIX_TRACK_LONGEST, &longest_len, &match_node);
	return ft_lookup_longest_match_iter_finish(ret, longest_len,
			match_node, iter);
}

FT_LOOKUP_DISPATCH("lookup_longest_match")
enum cds_ft_status cds_ft_lookup_longest_match(struct cds_ft *ft,
		struct cds_ft_iter *iter)
{
	return (*ft->lookup_longest_match_iter_fn)(ft, iter);
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
static inline_lookup
enum ft_descent_action ft_inequality_compressed(struct cds_ft_inode_flag **node_flag_p,
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

	FT_TP(ineq_compressed_enter, (const void *) cn, level, (int) mode);

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
			FT_TP(ineq_compressed, (const void *) cn, level,
				cmp_result, mpos, (int) FT_DESCENT_GOING_UP);
			return FT_DESCENT_GOING_UP;
		}
		/* Descend into compressed subtree. */
		ordinal_key[level - 1 + mpos] = cn->key_bytes[mpos];
		for (j = mpos + 1; j < cn->len; j++) {
			ordinal_key[level - 1 + j] = cn->key_bytes[j];
			iter_path_node(iter)[level + j] = node_flag;
		}
		level += cn->len - 1;
		node_flag = ft_dereference_acquire_prefetch(cn->child);
		if (!node_flag)
			goto out_break;
		/*
		 * iter_path[level] is the dispatcher for byte at index
		 * level - 1 (i.e. the byte AFTER the compressed prefix).
		 * That dispatcher is cn->child, not the compressed node
		 * itself, so overwrite the fill-loop's compressed entry at
		 * this position. going_up relies on this to find the
		 * sibling of the failing byte in cn->child.
		 */
		iter_path_node(iter)[level] = node_flag;
		iter_path_node(iter)[level + 1] = node_flag;
		*skip_eq_external_nodes_p = false;
		*node_flag_p = node_flag;
		*level_p = level;
		iter_debug_path_snapshot(iter);
		FT_TP(ineq_compressed, (const void *) cn, level,
			cmp_result, mpos,
			(int) FT_DESCENT_DESCEND_CHILDREN);
		return FT_DESCENT_DESCEND_CHILDREN;
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
			if (!node_flag)
				goto out_break;
			iter_path_node(iter)[level] = node_flag;
			iter_path_node(iter)[level + 1] = node_flag;
			*skip_eq_external_nodes_p = false;
			*node_flag_p = node_flag;
			*level_p = level;
			iter_debug_path_snapshot(iter);
			FT_TP(ineq_compressed, (const void *) cn, level,
				cmp_result, (unsigned int) cmp,
				(int) FT_DESCENT_DESCEND_CHILDREN);
			return FT_DESCENT_DESCEND_CHILDREN;
		}
		*level_p = level + cmp - 1;
		iter_debug_path_snapshot(iter);
		FT_TP(ineq_compressed, (const void *) cn, level,
			cmp_result, (unsigned int) cmp,
			(int) FT_DESCENT_GOING_UP);
		return FT_DESCENT_GOING_UP;
	}

	/* Full match: advance past compressed path. */
	level += cn->len - 1; /* -1: for loop increments */
	node_flag = ft_dereference_acquire_prefetch(cn->child);
	if (!node_flag)
		goto out_break;
	iter_path_node(iter)[level] = node_flag;
	iter_path_node(iter)[level + 1] = node_flag;
	if (ft_node_external(node_flag))
		goto out_break;

	*node_flag_p = node_flag;
	*level_p = level;
	FT_TP(ineq_compressed, (const void *) cn, level,
		cmp_result, (unsigned int) cmp,
		(int) FT_DESCENT_CONTINUE);
	return FT_DESCENT_CONTINUE;

out_break:
	*node_flag_p = node_flag;
	*level_p = level;
	FT_TP(ineq_compressed, (const void *) cn, level,
		cmp_result, (unsigned int) cmp,
		(int) FT_DESCENT_BREAK);
	return FT_DESCENT_BREAK;
}

/*
 * Handle a compressed node during the inequality minmax descent.
 *
 * On LEFTMOST (GE/GT) descent, check external_nodes at the
 * compressed node's entry depth first; if present and not
 * suppressed, return FT_DESCENT_FOUND_MINMAX with *ret_node_p
 * set and *level_p decremented by one, so the caller can jump
 * straight to found_minmax.
 *
 * Otherwise fill ordinal_key and iter_path across every level
 * spanned by the compressed path, advance @level to the span's
 * end, and step to cn->child.  Returns FT_DESCENT_BREAK when
 * the child is external (descent is done), otherwise
 * FT_DESCENT_CONTINUE.
 */
static inline_lookup
enum ft_descent_action ft_inequality_minmax_compressed(
		struct cds_ft_inode_flag **node_flag_p,
		ssize_t *level_p,
		struct cds_ft_node **ret_node_p,
		bool *skip_eq_external_nodes_p,
		struct cds_ft_iter *iter,
		uint8_t *ordinal_key,
		enum ft_direction dir)
{
	struct cds_ft_inode_flag *node_flag = *node_flag_p;
	struct cds_ft_compressed_node *cn = ft_compressed_node_ptr(node_flag);
	ssize_t level = *level_p;

	if (dir == FT_LEFTMOST) {
		struct cds_ft_metadata *cn_meta = cds_ft_item_to_metadata(
			(struct cds_ft_inode *) cn);
		struct cds_ft_node *ext = rcu_dereference(cn_meta->external_nodes);

		if (ext && !*skip_eq_external_nodes_p) {
			*ret_node_p = ext;
			*level_p = level - 1;
			return FT_DESCENT_FOUND_MINMAX;
		}
	}
	/*
	 * Fill ordinal_key and path entries for every level spanned
	 * by the compressed path.  The going-up code needs a valid
	 * entry at each level to call ft_node_get_direction (which
	 * returns NULL for siblings, causing the going-up walk to
	 * continue ascending).
	 */
	ft_fill_compressed_path(cn, ordinal_key, level - 1,
		iter_path_node(iter), level, node_flag);
	level += cn->len - 1;
	node_flag = ft_dereference_acquire_prefetch(cn->child);
	if (!node_flag) {
		/*
		 * Invariant violation: a reachable compressed node always
		 * has a non-NULL child, *even transiently*.  cn->child is
		 * wired before the compressed is published to its parent's
		 * slot and is never cleared in place (empty compresseds
		 * are pruned by replacing the parent's slot with the
		 * compressed's external_nodes, or by detaching the whole
		 * branch -- either way the compressed itself is no longer
		 * reachable when it becomes empty).
		 *
		 * Unlike the internal minmax == NULL case (transiently
		 * observable and handled via going_up above/below), there
		 * is no race window in which a reachable compressed has
		 * cn->child == NULL.  Observing NULL here is a real bug:
		 * abort loudly instead of silently propagating a corrupt
		 * node_flag.
		 */
		fprintf(stderr,
			"BUG: cds_ft_lookup_inequality minmax: "
			"compressed %p has NULL child\n",
			(const void *) node_flag);
		abort();
	}
	iter_path_node(iter)[level] = node_flag;
	*node_flag_p = node_flag;
	*level_p = level;
	if (ft_node_external(node_flag))
		return FT_DESCENT_BREAK;
	*skip_eq_external_nodes_p = false;
	return FT_DESCENT_CONTINUE;
}

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

	FT_TP(ineq_enter, (int) mode, input_key, key_len);

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
		struct cds_ft_metadata *metadata = cds_ft_item_to_metadata_fast(
				ft_node_ptr(node_flag), type->order);

		/*
		 * Empty trie: no internal children and no NIL-key entries.
		 * Loading nr_child instead of probing the bitmap works for
		 * every internal node class.
		 */
		if (metadata->nr_child == 0 &&
				!uatomic_load(&metadata->external_nodes, CMM_RELAXED)) {
			iter->node = NULL;
			iter->path_valid = true;
			iter_debug_path_snapshot(iter);
			iter->path_len = 1;
			iter->status = CDS_FT_STATUS_NOT_FOUND;
			goto end;
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
		    ft_node_skip_compressed(node_flag)) {
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
		FT_TP(fastpath_enter, (int) mode,
			(const void *) node_flag, (int) level);
		goto post_traversal;
	}

slow_path:
	FT_TP(slowpath_enter, (int) mode, (int) iter->path_valid,
		(int) iter->path_len);
	for (level = 1; level < key_depth; level++) {
		uint8_t key_value;

		if (ft_node_compressed(node_flag)) {
			enum ft_descent_action act;

			act = ft_inequality_compressed(&node_flag,
				&level, key_depth, mode, limit,
				&iter_key, input_key, iter,
				ordinal_key, &skip_eq_external_nodes);
			if (act == FT_DESCENT_GOING_UP)
				goto going_up;
			if (act == FT_DESCENT_DESCEND_CHILDREN)
				goto descend_children;
			if (act == FT_DESCENT_BREAK)
				break;
			if (level + 1 >= key_depth) {
				skip_eq_external_nodes = false;
				goto descend_children;
			}
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
		/*
		 * Record ordinal_key[level - 1] BEFORE the NULL-break so the
		 * going-up phase has the dispatch byte even for a dead-end
		 * descent.  The parent-pointer walk keys off ordinal_key
		 * (iter_key desyncs once level steps by span > 1), so this
		 * must be set regardless of whether descent continues.
		 */
		ordinal_key[level - 1] = key_value;
		node_flag = ft_node_get_nth(node_flag, NULL, key_value, FT_PF_NONE);
		if (!node_flag) {
			FT_TP(slowpath_step, (int) level, key_value,
				(const void *) node_flag, 1);
			break;
		}
		iter_path_node(iter)[level] = node_flag;
		FT_TP(slowpath_step, (int) level, key_value,
			(const void *) node_flag, 0);
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
	FT_TP(post_traversal, (int) mode, (int) level,
		(const void *) node_flag);
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

			if (ft_node_compressed(iter_path_node(iter)[level]))
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

				assert(level <= (int) ft->group->max_key_len);
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
		if (!ft_node_internal(iter_path_node(iter)[level - 1])) {
			FT_TP(ineq_going_up_step, level,
				(const void *) iter_path_node(iter)[level - 1],
				0, (uint8_t) key_value);
			going_up = true;
			continue;
		}
		node_flag = ft_node_get_leftright(iter_path_node(iter)[level - 1],
				key_value, &ordinal_key[level - 1], dir,
				true /* validate_lookup */);
		dbg_printf("cds_ft_lookup_inequality find sibling from %u at %u finds node_flag %p\n",
				(unsigned int) key_value, (unsigned int) ordinal_key[level - 1],
				node_flag);
		/* If found left/right sibling, find rightmost/leftmost child. */
		if (node_flag) {
			/* Record the sibling in the path. */
			iter_path_node(iter)[level] = node_flag;
			FT_TP(ineq_going_up_step, level,
				(const void *) iter_path_node(iter)[level - 1],
				1, ordinal_key[level - 1]);
			break;
		}
		FT_TP(ineq_going_up_step, level,
			(const void *) iter_path_node(iter)[level - 1],
			0, (uint8_t) key_value);
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

		assert(level <= (int) ft->group->max_key_len);
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
		node_flag = ft_resolve_skip_compressed(node_flag);
		/*
		 * Compressed node: traverse through the compressed
		 * path to reach the child.  Fill ordinal_key and
		 * iter path as we go.
		 */
		if (ft_node_compressed(node_flag)) {
			enum ft_descent_action act;

			act = ft_inequality_minmax_compressed(
				&node_flag, &level, &ret_node,
				&skip_eq_external_nodes,
				iter, ordinal_key, dir);
			if (act == FT_DESCENT_FOUND_MINMAX)
				goto found_minmax;
			if (act == FT_DESCENT_BREAK)
				break;
			assert(act == FT_DESCENT_CONTINUE);
			continue;
		}
		skip_eq_external_nodes = false;
		node_flag = ft_node_get_minmax(node_flag, &ordinal_key[level - 1], dir,
				true /* validate_lookup */);
		/*
		 * Transiently empty internal node (linear/popcount/pigeon): a reader may
		 * observe every slot of a reachable internal node as
		 * NULL in the narrow window between a writer's per-slot
		 * detach and the upward walk's slot-replace at a higher
		 * ancestor.  Quiescently, reachable internal nodes have
		 * nr_child >= 1 (the upward walk prunes single-child
		 * chains wholesale and replaces at the first multi-child
		 * ancestor, so a slot-emptied internal is never left in
		 * place).  Treat as "empty at this step" and let
		 * going_up find the next sibling at a higher level.
		 * Back level one step so iter_path[level] holds the
		 * parent the prior iter recorded.
		 */
		if (caa_unlikely(!node_flag)) {
			level--;
			going_up = true;
			goto going_up;
		}
		iter_path_node(iter)[level] = node_flag;
		dbg_printf("cds_ft_lookup_inequality find minmax at %u finds node_flag %p\n",
				(unsigned int) ordinal_key[level - 1], node_flag);
		if (ft_node_external(node_flag))
			break;
	}
	/*
	 * Every break path in the descent loop sets node_flag to a
	 * validated external (compressed-branch external child or
	 * minmax-branch external return).  Transiently-empty
	 * intermediate steps (non-root minmax == NULL above) do not
	 * reach this assert -- they jump to going_up and re-enter the
	 * search at a higher level.
	 */
	assert(ft_node_ptr(node_flag));
	ret_node = (struct cds_ft_node *) node_flag;
	/*
	 * The trie should always have external nodes at the
	 * very last level, so level should never grow large enough to overflow
	 * max_key_len.
	 */
	assert(level <= (int) ft->group->max_key_len);
	assert(level <= (int) ft->group->max_key_len);
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
	FT_TP(ineq_result, (int) mode,
		input_key, key_len,
		iter->node ? iter_key(iter) : NULL,
		iter->node ? iter->key_len : 0,
		(int) iter->status);
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
	CDS_FT_SCOPED_READER(ft);
	dbg_printf("cds_ft_lookup_le\n");
	return cds_ft_lookup_inequality(ft, iter,
			FT_LOOKUP_LE, FT_LOOKUP_LIMIT_NONE);
}

enum cds_ft_status cds_ft_lookup_ge(struct cds_ft *ft,
		struct cds_ft_iter *iter)
{
	CDS_FT_SCOPED_READER(ft);
	dbg_printf("cds_ft_lookup_ge\n");
	return cds_ft_lookup_inequality(ft, iter,
			FT_LOOKUP_GE, FT_LOOKUP_LIMIT_NONE);
}

enum cds_ft_status cds_ft_lookup_lt(struct cds_ft *ft,
		struct cds_ft_iter *iter)
{
	CDS_FT_SCOPED_READER(ft);
	dbg_printf("cds_ft_lookup_lt\n");
	return cds_ft_lookup_inequality(ft, iter,
			FT_LOOKUP_LT, FT_LOOKUP_LIMIT_NONE);
}

enum cds_ft_status cds_ft_lookup_gt(struct cds_ft *ft,
		struct cds_ft_iter *iter)
{
	CDS_FT_SCOPED_READER(ft);
	dbg_printf("cds_ft_lookup_gt\n");
	return cds_ft_lookup_inequality(ft, iter,
			FT_LOOKUP_GT, FT_LOOKUP_LIMIT_NONE);
}

enum cds_ft_status cds_ft_lookup_first(struct cds_ft *ft,
		struct cds_ft_iter *iter)
{
	size_t saved_key_len = iter->key_len;
	enum cds_ft_status status;

	CDS_FT_SCOPED_READER(ft);
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

	CDS_FT_SCOPED_READER(ft);
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
static inline
unsigned long ft_nr_keys_get(const struct cds_ft_metadata *m)
{
	return m->nr_keys;
}

static inline
unsigned long ft_nr_keys_load(const struct cds_ft_metadata *m)
{
	return uatomic_load(&m->nr_keys, CMM_ACQUIRE);
}

static inline
void ft_nr_keys_store(struct cds_ft_metadata *m, unsigned long val, int mo)
{
	uatomic_store(&m->nr_keys, val, mo);
}

/*
 * ft_propagate_external_count_parent: propagate nr_keys delta
 * from @start up to the root via metadata->parent pointers.
 *
 * @start: deepest internal/compressed node on the path (the node
 *         where the external was attached, or the deepest ancestor
 *         with metadata).  Must not be an external node or NULL.
 * @delta: +1 for insert, -1 for remove.
 *
 * Same ordering guarantees as the snapshot-based variant:
 * bottom-up CMM_RELEASE stores preserve the undercount invariant.
 */
static
void ft_propagate_external_count_parent(struct cds_ft *ft,
		struct cds_ft_inode_flag *start, long delta)
{
	struct cds_ft_inode_flag *cur = start;

	(void) ft;
	ft_delay_writer();

	while (cur) {
		struct cds_ft_metadata *m =
			cds_ft_item_to_metadata(ft_node_ptr(cur));
		ft_nr_keys_store(m, ft_nr_keys_get(m) + delta, CMM_RELEASE);
		ft_delay_writer();
		cur = m->parent;
	}
}


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

	FT_TP(split_compressed_insert_enter, (const void *) cn,
		cn->len, diverge_pos);
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

	unsigned int junction_depth = node_depth + diverge_pos;

	/* Compute old child's nr_keys for the new nodes. */
	if (!ft_node_external(cn->child)) {
		struct cds_ft_metadata *cm =
			cds_ft_item_to_metadata(ft_node_ptr(cn->child));
		old_child_nr_keys = ft_nr_keys_get(cm);
	} else if (cn->child) {
		old_child_nr_keys = 1;	/* external leaf */
	} else {
		old_child_nr_keys = 0;
	}

	/* 1. Build old suffix → old child. */
	if (suffix_len >= 1) {
		struct cds_ft_compressed_node *sfx;
		struct cds_ft_metadata *sfx_meta;

		sfx = alloc_compressed_node(ft, suffix_len, &sfx_meta);
		if (!sfx) goto error;
		sfx->child = cn->child;
		sfx->len = suffix_len;
		memcpy(sfx->key_bytes, &cn->key_bytes[diverge_pos + 1], suffix_len);
		sfx_meta->nr_child = 1;
		ft_nr_keys_store(sfx_meta, old_child_nr_keys, CMM_RELAXED);
		old_suffix_flag = ft_compressed_node_flag(sfx);
		ft_set_parent(cn->child, old_suffix_flag, &sfx->child);
		old_suffix_flag = ft_publish_compressed(ft, sfx, old_suffix_flag);
		created[nr_created++] = old_suffix_flag;
	} else {
		/* suffix_len == 0: old child directly. */
		old_suffix_flag = cn->child;
	}

	/* 2. Build new branch → new leaf. */
	if (new_len >= 1) {
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
		ft_nr_keys_store(nb_meta, 1, CMM_RELAXED);
		new_branch_flag = ft_compressed_node_flag(nb);
		ft_set_parent(nb->child, new_branch_flag, NULL);
		new_branch_flag = ft_publish_compressed(ft, nb, new_branch_flag);
		created[nr_created++] = new_branch_flag;
	} else {
		/* new_len == 0: child_node directly. */
		new_branch_flag = (struct cds_ft_inode_flag *) child_node;
	}

	/* 3. Build branch node with both children. */
	{
		struct cds_ft_inode_flag *dest = NULL;
		struct cds_ft_metadata *branch_meta;

		/* First child: old direction. */
		ret = ft_node_set_nth(ft, &dest, old_ordinal, old_suffix_flag, NULL, NULL,
				junction_depth);
		if (ret) goto error;
		created[nr_created++] = dest;
		branch_flag = dest;

		/* Second child: new direction. */
		{
			struct cds_ft_inode *old_recompacted = NULL;

			branch_meta = cds_ft_item_to_metadata(ft_node_ptr(dest));
			ret = ft_node_set_nth(ft, &dest, new_ordinal, new_branch_flag,
					&old_recompacted, branch_meta, junction_depth);
			if (ret) goto error;
			if (old_recompacted) {
				free_cds_ft_node(ft, old_recompacted);
				/* Update created entry to the recompacted node. */
				created[nr_created - 1] = dest;
			}
		}

		branch_meta = cds_ft_item_to_metadata(ft_node_ptr(dest));
		ft_nr_keys_store(branch_meta, old_child_nr_keys + 1,
				CMM_RELAXED);
		branch_flag = dest;
	}

	/* 4. Build prefix → branch (if needed). */
	if (diverge_pos >= 2) {
		struct cds_ft_inode_flag *pfx_child = branch_flag;

		/*
		 * When external_nodes exist, the prefix compressed
		 * node must not carry them.  Shorten the prefix by
		 * 1 byte (compressed len = diverge_pos - 1) and add
		 * an internal node at node_depth that holds
		 * external_nodes and dispatches on the first byte.
		 */
		if (cn_meta->external_nodes && diverge_pos >= 3) {
			struct cds_ft_compressed_node *pfx;
			struct cds_ft_metadata *pfx_meta;

			pfx = alloc_compressed_node(ft, diverge_pos - 1, &pfx_meta);
			if (!pfx) goto error;
			pfx->child = branch_flag;
			pfx->len = diverge_pos - 1;
			memcpy(pfx->key_bytes, &cn->key_bytes[1], diverge_pos - 1);
			pfx_meta->nr_child = 1;
			ft_nr_keys_store(pfx_meta, ft_nr_keys_get(cn_meta) + 1, CMM_RELAXED);
			pfx_child = ft_compressed_node_flag(pfx);
			ft_set_parent(branch_flag, pfx_child, NULL);
			pfx_child = ft_publish_compressed(ft, pfx, pfx_child);
			created[nr_created++] = pfx_child;
		} else if (cn_meta->external_nodes) {
			/* diverge_pos == 2: sub-prefix is 1 byte, handled
			 * by the internal node dispatch below. */
		} else {
			struct cds_ft_compressed_node *pfx;
			struct cds_ft_metadata *pfx_meta;

			pfx = alloc_compressed_node(ft, diverge_pos, &pfx_meta);
			if (!pfx) goto error;
			pfx->child = branch_flag;
			pfx->len = diverge_pos;
			memcpy(pfx->key_bytes, cn->key_bytes, diverge_pos);
			pfx_meta->nr_child = 1;
			ft_nr_keys_store(pfx_meta, ft_nr_keys_get(cn_meta) + 1, CMM_RELAXED);
			pfx_child = ft_compressed_node_flag(pfx);
			ft_set_parent(branch_flag, pfx_child, NULL);
			pfx_child = ft_publish_compressed(ft, pfx, pfx_child);
			created[nr_created++] = pfx_child;
			top_flag = pfx_child;
			goto prefix_done;
		}

		/* Internal node at node_depth: dispatches on first
		 * prefix byte, holds external_nodes if present. */
		{
			struct cds_ft_inode_flag *dest = NULL;
			struct cds_ft_metadata *int_meta;

			ret = ft_node_set_nth(ft, &dest, cn->key_bytes[0],
					pfx_child, NULL, NULL, node_depth);
			if (ret) goto error;
			int_meta = cds_ft_item_to_metadata(ft_node_ptr(dest));
			ft_nr_keys_store(int_meta, ft_nr_keys_get(cn_meta) + 1, CMM_RELAXED);
			if (cn_meta->external_nodes)
				ft_metadata_set_external_nodes(dest, int_meta, cn_meta->external_nodes);
			top_flag = dest;
			created[nr_created++] = dest;
		}
	prefix_done:
		(void) 0;
	} else if (diverge_pos == 1 && !cn_meta->external_nodes) {
		/*
		 * 1-byte prefix without external_nodes: emit a 1-byte
		 * compressed node instead of a 1-child internal — canonical
		 * form under FEATURE_FT_SKIP_COMPRESSED.
		 */
		struct cds_ft_compressed_node *pfx;
		struct cds_ft_metadata *pfx_meta;
		struct cds_ft_inode_flag *pfx_flag;

		pfx = alloc_compressed_node(ft, 1, &pfx_meta);
		if (!pfx) goto error;
		pfx->child = branch_flag;
		pfx->len = 1;
		pfx->key_bytes[0] = cn->key_bytes[0];
		pfx_meta->nr_child = 1;
		ft_nr_keys_store(pfx_meta, ft_nr_keys_get(cn_meta) + 1, CMM_RELAXED);
		pfx_flag = ft_compressed_node_flag(pfx);
		ft_set_parent(branch_flag, pfx_flag, &pfx->child);
		pfx_flag = ft_publish_compressed(ft, pfx, pfx_flag);
		top_flag = pfx_flag;
		created[nr_created++] = top_flag;
	} else if (diverge_pos == 1) {
		/* diverge_pos == 1 with external_nodes: must remain internal
		 * (compressed nodes cannot carry external_nodes). */
		struct cds_ft_inode_flag *dest = NULL;
		struct cds_ft_metadata *pfx_meta;

		ret = ft_node_set_nth(ft, &dest, cn->key_bytes[0],
				branch_flag, NULL, NULL, node_depth);
		if (ret) goto error;
		pfx_meta = cds_ft_item_to_metadata(ft_node_ptr(dest));
		ft_nr_keys_store(pfx_meta, ft_nr_keys_get(cn_meta) + 1, CMM_RELAXED);
		ft_metadata_set_external_nodes(dest, pfx_meta, cn_meta->external_nodes);
		top_flag = dest;
		created[nr_created++] = dest;
	} else {
		/* diverge_pos == 0: branch IS the top. */
		struct cds_ft_metadata *branch_meta =
			cds_ft_item_to_metadata(ft_node_ptr(branch_flag));
		ft_nr_keys_store(branch_meta, ft_nr_keys_get(cn_meta) + 1,
				CMM_RELAXED);
		if (cn_meta->external_nodes)
			ft_metadata_set_external_nodes(branch_flag, branch_meta, cn_meta->external_nodes);
		top_flag = branch_flag;
	}

	/* 5. Publish the split structure, replacing the compressed node. */
	FT_TP(compressed_split, "insert", (const void *) cn, cn->len,
		(const void *) top_flag, diverge_pos);
	/*
	 * Compressed-split replaces the compressed node in its parent's
	 * slot via a direct ft_publish_to_parent call, bypassing
	 * ft_node_set_nth.  Emit tree_edge_set explicitly so consumers
	 * see the (parent, key_byte, top_flag) structural edge.  The
	 * compressed node sits at node_depth; iter_key points at the
	 * key bytes starting at that depth, so iter_key[-1] is the
	 * parent's key_byte that led to the compressed node (safe for
	 * node_depth >= 1, which always holds since compressed nodes
	 * are never at the root).
	 */
	FT_TP(tree_edge_set, (const void *) ft,
		(const void *) cn_meta->parent,
		(unsigned int) (node_depth - 1),
		(uint8_t) iter_key[-1],
		(const void *) top_flag);
	ft_publish_to_parent(ft, cn_meta->parent, parent_slot, top_flag);

	/* 7. Free the old compressed node. */
	free_compressed_node(ft, cn);

	return 0;

error:
	{
		int i;

		for (i = 0; i < nr_created; i++) {
			if (ft_node_compressed(created[i]))
				free_compressed_node_unpublished(ft,
					ft_compressed_node_ptr(created[i]));
			else if (ft_node_skip_compressed(created[i]))
				free_compressed_node_unpublished(ft,
					ft_skip_to_compressed(created[i]));
			else
				free_cds_ft_node_unpublished(ft, ft_node_ptr(created[i]));
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
		struct cds_ft_inode_flag **jct_ret,
		unsigned int node_depth)
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
		child_nr_keys = ft_nr_keys_get(cm);
	} else if (cn->child) {
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
		ft_nr_keys_store(sfx_meta, child_nr_keys,
			CMM_RELAXED);
		suffix_flag = ft_compressed_node_flag(sfx);
		ft_set_parent(cn->child, suffix_flag, &sfx->child);
		suffix_flag = ft_publish_compressed(ft, sfx, suffix_flag);
		created[nr_created++] = suffix_flag;
	} else if (suffix_len == 1) {
		struct cds_ft_inode_flag *dest = NULL;

		ret = ft_node_set_nth(ft, &dest,
			cn->key_bytes[remaining + 1],
			cn->child, NULL, NULL,
			node_depth + remaining + 1);
		if (ret) goto error;
		{
			struct cds_ft_metadata *m =
				cds_ft_item_to_metadata(ft_node_ptr(dest));
			ft_nr_keys_store(m, child_nr_keys,
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
			suffix_flag, NULL, NULL,
			node_depth + remaining);
		if (ret) goto error;
		jct_meta = cds_ft_item_to_metadata(ft_node_ptr(dest));
		ft_nr_keys_store(jct_meta, child_nr_keys,
			CMM_RELAXED);
		jct_flag = dest;
		created[nr_created++] = dest;
	}

	/* Prefix → junction. */
	if (remaining >= 2 && !cn_meta->external_nodes) {
		struct cds_ft_compressed_node *pfx;
		struct cds_ft_metadata *pfx_meta;

		pfx = alloc_compressed_node(ft, remaining, &pfx_meta);
		if (!pfx) goto error;
		pfx->child = jct_flag;
		pfx->len = remaining;
		memcpy(pfx->key_bytes, cn->key_bytes, remaining);
		pfx_meta->nr_child = 1;
		ft_nr_keys_store(pfx_meta, ft_nr_keys_get(cn_meta),
			CMM_RELAXED);
		top_flag = ft_compressed_node_flag(pfx);
		ft_set_parent(jct_flag, top_flag, NULL);
		top_flag = ft_publish_compressed(ft, pfx, top_flag);
		created[nr_created++] = top_flag;
	} else if (remaining >= 2 && cn_meta->external_nodes) {
		/*
		 * Compressed prefix must not carry external_nodes.
		 * Create compressed(len=remaining-1) + internal at
		 * node_depth holding external_nodes.
		 */
		struct cds_ft_inode_flag *pfx_child = jct_flag;

		if (remaining >= 3) {
			struct cds_ft_compressed_node *pfx;
			struct cds_ft_metadata *pfx_meta;

			pfx = alloc_compressed_node(ft, remaining - 1, &pfx_meta);
			if (!pfx) goto error;
			pfx->child = jct_flag;
			pfx->len = remaining - 1;
			memcpy(pfx->key_bytes, &cn->key_bytes[1], remaining - 1);
			pfx_meta->nr_child = 1;
			ft_nr_keys_store(pfx_meta, ft_nr_keys_get(cn_meta),
				CMM_RELAXED);
			pfx_child = ft_compressed_node_flag(pfx);
			ft_set_parent(jct_flag, pfx_child, NULL);
			pfx_child = ft_publish_compressed(ft, pfx, pfx_child);
			created[nr_created++] = pfx_child;
		}
		{
			struct cds_ft_inode_flag *dest = NULL;
			struct cds_ft_metadata *int_meta;

			ret = ft_node_set_nth(ft, &dest, cn->key_bytes[0],
				pfx_child, NULL, NULL, node_depth);
			if (ret) goto error;
			int_meta = cds_ft_item_to_metadata(ft_node_ptr(dest));
			ft_nr_keys_store(int_meta, ft_nr_keys_get(cn_meta),
				CMM_RELAXED);
			ft_metadata_set_external_nodes(dest, int_meta, cn_meta->external_nodes);
			top_flag = dest;
			created[nr_created++] = dest;
		}
	} else if (remaining == 1) {
		struct cds_ft_inode_flag *dest = NULL;
		struct cds_ft_metadata *pfx_meta;

		ret = ft_node_set_nth(ft, &dest, cn->key_bytes[0],
			jct_flag, NULL, NULL, node_depth);
		if (ret) goto error;
		pfx_meta = cds_ft_item_to_metadata(ft_node_ptr(dest));
		ft_nr_keys_store(pfx_meta, ft_nr_keys_get(cn_meta),
			CMM_RELAXED);
		if (cn_meta->external_nodes)
			ft_metadata_set_external_nodes(dest, pfx_meta, cn_meta->external_nodes);
		top_flag = dest;
		created[nr_created++] = dest;
	} else {
		/* remaining == 0: no prefix, junction IS the top.
		 * Transfer any existing external_nodes from the old
		 * compressed node to the junction.  Add +1 to nr_keys
		 * for the transferred external key (child_nr_keys
		 * counted cn->child's subtree but not cn_meta's own
		 * external_nodes).
		 */
		if (cn_meta->external_nodes) {
			struct cds_ft_metadata *jct_meta =
				cds_ft_item_to_metadata(ft_node_ptr(jct_flag));
			ft_metadata_set_external_nodes(jct_flag, jct_meta,
				cn_meta->external_nodes);
			ft_nr_keys_store(jct_meta,
				ft_nr_keys_get(jct_meta) + 1, CMM_RELAXED);
		}
		top_flag = jct_flag;
	}

	FT_TP(compressed_split, "key_shorter", (const void *) cn, cn->len,
		(const void *) top_flag, remaining);
	*top_ret = top_flag;
	*jct_ret = jct_flag;
	return 0;

error:
	{
		int i;

		for (i = 0; i < nr_created; i++) {
			if (ft_node_compressed(created[i]))
				free_compressed_node_unpublished(ft,
					ft_compressed_node_ptr(created[i]));
			else
				free_cds_ft_node_unpublished(ft,
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
		unsigned long subtree_external_count,
		bool has_external_nodes);

static
void ft_free_branch(struct cds_ft *ft,
		const uint8_t *key, unsigned int start, unsigned int end,
		struct cds_ft_inode_flag *top_node);


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
		struct cds_ft_node *external_nodes __attribute__((unused)))
{
	uint8_t path_len = (uint8_t)(key_len - level);
	struct cds_ft_compressed_node *cn;
	struct cds_ft_metadata *cn_meta;
	int j;

	/*
	 * Length-1 compressed nodes are canonical under
	 * FEATURE_FT_SKIP_COMPRESSED: the publish wraps cn into a
	 * SKIP_X-tagged slot pointer, dispatching for free relative to
	 * the 1-child internal node it replaces.
	 */
	if (path_len < 1)
		return NULL;
	cn = alloc_compressed_node(ft, path_len, &cn_meta);
	if (!cn)
		return (struct cds_ft_inode_flag *) (long) -ENOMEM;
	cn->child = child;
	cn->len = path_len;
	for (j = 0; j < path_len; j++)
		cn->key_bytes[j] = key[level + j];
	cn_meta->nr_child = 1;
	ft_nr_keys_store(cn_meta, 1, CMM_RELAXED);
	/* Compressed nodes must not carry external_nodes. */
	assert(!external_nodes);
	{
		struct cds_ft_inode_flag *cflag = ft_compressed_node_flag(cn);
		ft_set_parent(child, cflag, &cn->child);
		/* compressed_publish emitted by ft_publish_compressed. */
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

	FT_TP(attach_node_enter, (const void *) attach_node_flag,
		(const void *) old_node_flag, level);

	dbg_printf("Attach node at level %u (old_node_flag %p, attach_node_flag_ptr %p attach_node_flag %p)\n",
		level, old_node_flag, attach_node_flag_ptr, attach_node_flag);

	assert(!old_node_flag || external_nodes);
	assert(level > 0);	/* Root is always internal; level 0 is handled directly. */
	if (attach_node_flag)
		metadata = cds_ft_item_to_metadata(ft_node_ptr(attach_node_flag));

	/* Concurrent update prevented by mutual exclusion. */
	assert(!(old_node_flag_ptr && (ft_node_ptr(*old_node_flag_ptr) && !external_nodes)));
	assert(!(attach_node_flag_ptr && ft_node_ptr(*attach_node_flag_ptr) !=
			ft_node_ptr(attach_node_flag)));
	(void) old_node_flag_ptr;	/* Used by assert above; silence -DNDEBUG. */

	/* Create new branch, starting from bottom */
	iter_node_flag = (struct cds_ft_inode_flag *) child_node;

	{
		struct cds_ft_inode_flag *compressed;
		/*
		 * Compressed nodes must not carry metadata->external_nodes.
		 * When external_nodes exist at @level, compress from
		 * level+1 (one byte shorter) and let the loop below
		 * create an internal node at @level that holds the
		 * external_nodes.
		 */
		unsigned int compress_level = external_nodes ? level + 1 : level;

		compressed = ft_try_compress_chain(ft, key, key_len,
			compress_level, iter_node_flag, NULL);
		if (compressed == (void *) (long) -ENOMEM) {
			ret = -ENOMEM;
			goto check_error;
		}
		if (compressed) {
			iter_node_flag = compressed;
			created_nodes[nr_created_nodes++] = iter_node_flag;
			iter_key = key + compress_level;
			/*
			 * When external_nodes exist, compress_level = level + 1.
			 * iter_key points to key + level + 1.  The loop below
			 * runs one iteration to create an internal node at
			 * @level that dispatches on key[level] with the
			 * compressed node as child.  The loop then places
			 * external_nodes on this internal node.
			 */
		}
	}
	if ((!ft_node_compressed(iter_node_flag) &&
	     !ft_node_skip_compressed(iter_node_flag)) ||
	    external_nodes) {
		for (i = (ft_node_compressed(iter_node_flag) ||
			  ft_node_skip_compressed(iter_node_flag)) ?
				(int) level + 1 : (int) key_len;
		     i > (int) level; i--) {
			uint8_t key_value;

			key_value = *(--iter_key);
			dbg_printf("branch creation level %d, key %u\n",
					i, (unsigned int) key_value);
			iter_dest_node_flag = NULL;
			ret = ft_node_set_nth(ft, &iter_dest_node_flag, key_value, iter_node_flag, NULL, NULL,
					i - 1);
			if (ret) {
				dbg_printf("branch creation error %d\n", ret);
				goto check_error;
			}
			{
				struct cds_ft_metadata *branch_meta =
					cds_ft_item_to_metadata(ft_node_ptr(iter_dest_node_flag));
				ft_nr_keys_store(branch_meta, 1, CMM_RELAXED);
			}
			created_nodes[nr_created_nodes++] = iter_dest_node_flag;
			iter_node_flag = iter_dest_node_flag;
		}

		if (external_nodes) {
			struct cds_ft_metadata *iter_node_metadata;

			iter_node_metadata = cds_ft_item_to_metadata(ft_node_ptr(iter_node_flag));
			ft_metadata_set_external_nodes(iter_node_flag, iter_node_metadata, external_nodes);
			ft_nr_keys_store(iter_node_metadata,
				ft_nr_keys_get(iter_node_metadata) + 1, CMM_RELAXED);
		}
	}

	/* Publish branch. */
	{
		uint8_t key_value;

		key_value = *(--iter_key);
		dbg_printf("publish branch at level %d, key %u\n", level - 1, (unsigned int) key_value);


		/* We need to use set_nth on the previous level. */
		iter_dest_node_flag = attach_node_flag;
		ret = ft_node_set_nth(ft, &iter_dest_node_flag, key_value, iter_node_flag,
				&old_recompacted_node, metadata, level - 1);
		if (ret) {
			dbg_printf("branch publish error %d\n", ret);
			goto check_error;
		}
		/* Attach branch (unlink the old node from the trie).
		 * ft_publish_to_parent handles skip pointer update
		 * if the attach target is a compressed node's child.
		 */
		ft_publish_to_parent(ft, attach_node_flag,
			attach_node_flag_ptr, iter_dest_node_flag);

		/* Reclaim safely after unlink. */
		if (old_recompacted_node)
			free_cds_ft_node(ft, old_recompacted_node);
	}

	/* Success */
	ret = 0;

check_error:
	if (ret) {
		/*
		 * All goto-check_error paths in this function are before
		 * ft_publish_to_parent, so created_nodes[] never escaped
		 * the writer's stack — immediate-free is safe.
		 */
		for (i = 0; i < nr_created_nodes; i++) {
			if (ft_node_compressed(created_nodes[i]))
				free_compressed_node_unpublished(ft,
					ft_compressed_node_ptr(created_nodes[i]));
			else
				free_cds_ft_node_unpublished(ft, ft_node_ptr(created_nodes[i]));
		}
	}
	FT_TP(attach_node_exit, (int) ret);
	return ret;
}

static
void ft_chain_node(struct cds_ft_node *last_node, struct cds_ft_node *node)
{
	FT_TP(chain_node, (const void *) last_node, (const void *) node);
	/*
	 * Add node to tail of list to ensure that RCU traversals will
	 * always see either the prior node or the newly added if
	 * executed concurrently with a sequence of add followed by del
	 * on the same key. Safe against concurrent RCU read traversals.
	 *
	 * The prev pointer is write-side only (mutex-held), so a plain
	 * store is sufficient.
	 */
	node->prev = last_node;
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
	d->nf    = ft_node_get_nth(d->pnf, &d->nfp, key_value, FT_PF_NONE);
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
		dd->det_depth = dd->d.depth;
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
		dd->det_depth = dd->d.depth;
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
	int ret;

	/*
	 * Case 1 (external at END of compressed path): build a
	 * branch for the continuing key, with an internal node at
	 * d->depth + cn->len that holds the old external child as
	 * external_nodes and dispatches the next key byte.
	 */
	{
		unsigned int br_start = d->depth + cn->len;
		struct cds_ft_inode_flag *inner;
		struct cds_ft_inode_flag *dest = NULL;

		inner = ft_build_branch(ft, key,
			br_start + 1, key_len,
			(struct cds_ft_inode_flag *) node, 1, false);
		if (!inner)
			return -ENOMEM;
		ret = ft_node_set_nth(ft, &dest, key[br_start],
			inner, NULL, NULL, br_start);
		if (ret)
			return -ENOMEM;
		branch = dest;
		br_meta = cds_ft_item_to_metadata(ft_node_ptr(branch));
		ft_metadata_set_external_nodes(branch, br_meta,
			(struct cds_ft_node *) cn->child);
		/*
		 * Count only the pre-existing key (old external from
		 * the compressed child).  The new key's +1 is added
		 * by ft_propagate_external_count_parent below.
		 */
		ft_nr_keys_store(br_meta, 1, CMM_RELAXED);
	}
	ft_set_parent(branch, d->nf, &cn->child);
	ft_publish_to_parent(ft, d->nf, &cn->child, branch);
	ft_propagate_external_count_parent(ft, branch, 1);
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
	ft_set_parent(*d->nfp, d->pnf, d->nfp);

	ft_propagate_external_count_parent(ft, d->pnf, 1);
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
		d->nf, remaining, &top_flag, &jct_flag, d->depth);
	if (sret)
		return sret;
	ft_set_parent(top_flag, d->pnf, d->nfp);
	ft_publish_to_parent(ft, d->pnf, d->nfp, top_flag);
	{
		struct cds_ft_metadata *jct_meta =
			cds_ft_item_to_metadata(
				ft_node_ptr(jct_flag));
		assert(!ft_node_compressed(jct_flag));
		if (unique_node_ret &&
		    jct_meta->external_nodes) {
			*unique_node_ret =
				jct_meta->external_nodes;
			return -EEXIST;
		}
		if (jct_meta->external_nodes) {
			/*
			 * Junction already has external_nodes (transferred
			 * from old compressed node for remaining == 0).
			 * Chain new node as duplicate; no key count change.
			 */
			struct cds_ft_node *last = jct_meta->external_nodes;

			while (last->next)
				last = last->next;
			ft_chain_node(last, node);
			/* Skip ft_propagate_external_count_parent below:
			 * duplicate at existing key, no new unique key. */
			goto skip_key_count_propagation;
		}
		node->prev = jct_flag;
		node->next = NULL;
		rcu_assign_pointer(
			jct_meta->external_nodes, node);
	}
	ft_propagate_external_count_parent(ft, jct_flag, 1);
skip_key_count_propagation:
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
enum ft_descent_action ft_insert_compressed(struct cds_ft *ft,
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
		 * compressed, or skip-compressed (which encodes another
		 * compressed node deeper in the chain). */
		if (cn->child &&
		    (ft_node_skip_compressed(cn->child) ||
		     ft_node_internal(cn->child) ||
		     ft_node_compressed(cn->child))) {
			ft_snapshot_push(snapshot, snapshot_depth,
				*nr_snapshot_p, d->nf, d->depth);
			ft_descent_traverse_compressed(d, cn, iter_key_p);
			return FT_DESCENT_CONTINUE;
		}
		if (!cn->child) {
			struct cds_ft_metadata *cn_meta =
				cds_ft_item_to_metadata((struct cds_ft_inode *) cn);
			fprintf(stderr, "BUG: cn->child NULL, cn=%p cn->len=%u depth=%u external_nodes=%p nr_child=%u\n",
				cn, cn->len, d->depth, cn_meta->external_nodes, (unsigned)cn_meta->nr_child);
			abort();
		}
		if (cn->len == remaining) {
			/* Key ends at external child: duplicate. */
			ft_snapshot_push(snapshot, snapshot_depth,
				*nr_snapshot_p, d->nf, d->depth);
			ft_descent_traverse_compressed(d, cn, iter_key_p);
			return FT_DESCENT_BREAK;
		}
		/* Key continues past external child: build branch. */
		*ret_p = ft_insert_compressed_past_child(ft, d, key,
			key_len, cn, node);
		return FT_DESCENT_END;
	}
	if (j < cmp) {
		/* Key diverges: split at position j. */
		*ret_p = ft_insert_compressed_diverge(ft, d,
			*iter_key_p, remaining, j, node);
		return FT_DESCENT_END;
	}
	/* Key shorter: split into prefix -> junction -> suffix. */
	*ret_p = ft_insert_compressed_key_shorter(ft, d, remaining,
		node, unique_node_ret);
	return FT_DESCENT_END;
}


static
int _cds_ft_insert(struct cds_ft *ft,
		const uint8_t *_key, size_t _key_len,
		struct cds_ft_node *node,
		struct cds_ft_node **unique_node_ret)
{
	unsigned int key_depth;
	struct ft_descent d;
	size_t key_len = ft_key_len(ft, _key_len);
	const struct cds_ft_key_map *km = &ft->group->key_map;
	uint8_t ordinal_buf[FT_MAX_KEY_LEN];
	const uint8_t *key;
	const uint8_t *iter_key;
	struct cds_ft_inode_flag *snapshot[FT_MAX_DEPTH];
	unsigned int snapshot_depth[FT_MAX_DEPTH]; /* parallel depth tracking */
	int nr_snapshot = 0;
	int ret;

	if (!valid_external_node(node) || !valid_key_len(ft, key_len))
		return -EINVAL;
	if (caa_likely(km->identity)) {
		key = _key;
	} else {
		ft_key_to_ordinals(ordinal_buf, _key, key_len, km);
		key = ordinal_buf;
	}
	iter_key = key;
	/* Expect zeroed prev/next pointers. This catches some double-insert misuses. */
	if (node->prev || node->next)
		return -EINVAL;

	key_depth = key_len + 1;

	dbg_printf("cds_ft_insert attempt: node %p\n", node);
	ft_descent_init(&d, ft);

	for (; d.depth < key_depth - 1; ) {
		uint8_t key_value;

		if (!d.nf)
			break;
		/*
		 * Resolve skip-compressed pointer.  Convert to the
		 * underlying compressed flag so the compressed handler
		 * below processes it correctly.
		 */
		d.nf = ft_resolve_skip_compressed(d.nf);
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
			enum ft_descent_action act;

			act = ft_insert_compressed(ft, &d, &iter_key,
				key, key_len, key_depth, node,
				unique_node_ret, snapshot, snapshot_depth,
				&nr_snapshot, &ret);
			if (act == FT_DESCENT_END)
				goto insert_done;
			if (act == FT_DESCENT_BREAK)
				break;
			continue;
		}
		dbg_printf("cds_ft_insert iter ppnf %p pnf %p nfp %p nf %p\n",
				d.ppnf, d.pnf, d.nfp, d.nf);
		ft_snapshot_push(snapshot, snapshot_depth,
			nr_snapshot, d.nf, d.depth);
		key_value = *(iter_key++);
		ft_descent_step(&d, key_value);
	}

	/*
	 * Resolve any skip-compressed pointer left in d.nf by the descent
	 * loop's final step (e.g., ft_descent_traverse_compressed sets d.nf
	 * to cn->child raw, which may be skip-compressed).  The loop body's
	 * resolve at the top of each iteration only fires when the loop
	 * iterates again; a traverse that pushes d.depth to key_depth - 1
	 * exits the loop without re-entering.
	 */
	d.nf = ft_resolve_skip_compressed(d.nf);

	if (d.depth == key_depth - 1) {
		/* Found either an internal, external node or NULL at end of key. */
		if (!d.nf) {
			dbg_printf("cds_ft_insert NULL ppnf %p pnf %p nfp %p nf %p\n",
					d.ppnf, d.pnf, d.nfp, d.nf);

			ret = ft_attach_node(ft, d.pnfp, d.pnf,
					d.nfp, d.nf, key, key_len, d.depth, node,
					NULL);
			if (ret == 0) {
				ft_propagate_external_count_parent(ft, *d.pnfp, 1);
				if (d.depth >= 2)
					FT_TP(tree_edge_set, (const void *) ft,
						(const void *) d.ppnf,
						(unsigned int) (d.depth - 2),
						(uint8_t) key[d.depth - 2],
						(const void *) *d.pnfp);
			}

		} else if (ft_node_compressed(d.nf)) {
			/*
			 * Key ends at a compressed node's depth.
			 * Split: internal(external_nodes) + compressed(len-1).
			 */
			ret = ft_insert_compressed_key_shorter(ft, &d, 0,
				node, unique_node_ret);
		} else if (!ft_node_external(d.nf)) {
			struct cds_ft_node *external_nodes;
			struct cds_ft_metadata *metadata;

			assert(!ft_node_compressed(d.nf));
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
				node->prev = d.nf;
				node->next = NULL;
				rcu_assign_pointer(metadata->external_nodes, node);
				ret = 0;
				ft_propagate_external_count_parent(ft, d.nf, 1);
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
		if (ret == 0) {
			ft_propagate_external_count_parent(ft, *d.pnfp, 1);
			if (d.depth >= 2)
				FT_TP(tree_edge_set, (const void *) ft,
					(const void *) d.ppnf,
					(unsigned int) (d.depth - 2),
					(uint8_t) key[d.depth - 2],
					(const void *) *d.pnfp);
		}
	}

insert_done:
	if (ret == 0) {
		if (key_len > uatomic_load(&ft->max_used_key_len, CMM_RELAXED))
			uatomic_store(&ft->max_used_key_len, key_len, CMM_RELAXED);
	}

	return ret;
}

enum cds_ft_status cds_ft_insert(struct cds_ft *ft,
		const uint8_t *key, size_t key_len,
		struct cds_ft_node *node)
{
	int ret;

	CDS_FT_SCOPED_WRITER(ft);
	FT_TP_KEY(insert_enter, ft, key, key_len);
	ret = _cds_ft_insert(ft, key, key_len, node, NULL);

	if (ret == 0) {
		FT_TP(insert_exit, (int) CDS_FT_STATUS_OK);
		return CDS_FT_STATUS_OK;
	}
	if (ret == -EINVAL) {
		FT_TP(insert_exit, (int) CDS_FT_STATUS_INVALID_ARGUMENT_ERROR);
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}
	FT_TP(insert_exit, (int) CDS_FT_STATUS_MEMORY_ERROR);
	return CDS_FT_STATUS_MEMORY_ERROR;
}

enum cds_ft_status cds_ft_insert_unique(struct cds_ft *ft,
		const uint8_t *key, size_t key_len,
		struct cds_ft_node *node,
		struct cds_ft_node **result_node)
{
	int ret;
	struct cds_ft_node *ret_node = NULL;

	CDS_FT_SCOPED_WRITER(ft);
	FT_TP_KEY(insert_unique_enter, ft, key, key_len);
	ret = _cds_ft_insert(ft, key, key_len, node, &ret_node);
	if (ret == -EEXIST) {
		*result_node = ret_node;
		FT_TP(insert_unique_exit, (int) CDS_FT_STATUS_DUPLICATE_FOUND);
		return CDS_FT_STATUS_DUPLICATE_FOUND;
	}
	if (ret == -EINVAL) {
		*result_node = NULL;
		FT_TP(insert_unique_exit, (int) CDS_FT_STATUS_INVALID_ARGUMENT_ERROR);
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}
	if (ret) {
		*result_node = NULL;
		FT_TP(insert_unique_exit, (int) CDS_FT_STATUS_MEMORY_ERROR);
		return CDS_FT_STATUS_MEMORY_ERROR;
	}
	*result_node = node;
	FT_TP(insert_unique_exit, (int) CDS_FT_STATUS_OK);
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
	const struct cds_ft_key_map *km = &ft->group->key_map;
	uint8_t ordinal_buf[FT_MAX_KEY_LEN];
	const uint8_t *key;
	struct cds_ft_inode_flag *snapshot[FT_MAX_DEPTH];
	unsigned int snapshot_depth[FT_MAX_DEPTH];
	int nr_snapshot = 0;
	int ret;

	*old_node_ret = NULL;

	if (!valid_external_node(node) || !valid_key_len(ft, key_len))
		return -EINVAL;
	if (caa_likely(km->identity)) {
		key = _key;
	} else {
		ft_key_to_ordinals(ordinal_buf, _key, key_len, km);
		key = ordinal_buf;
	}
	/* Expect zeroed prev/next pointers. */
	if (node->prev || node->next)
		return -EINVAL;

	key_depth = key_len + 1;

	dbg_printf("_cds_ft_insert_replace attempt: node %p\n", node);
	iter_key = key;
	ft_descent_init(&d, ft);

	for (; d.depth < key_depth - 1; ) {
		uint8_t key_value;

		if (!d.nf)
			break;
		/* Resolve skip-compressed pointer. */
		d.nf = ft_resolve_skip_compressed(d.nf);
		if (ft_node_external(d.nf))
			break;
		if (ft_node_compressed(d.nf)) {
			enum ft_descent_action act;

			act = ft_insert_compressed(ft, &d, &iter_key,
				key, key_len, key_depth, node,
				NULL, snapshot, snapshot_depth,
				&nr_snapshot, &ret);
			if (act == FT_DESCENT_END)
				goto insert_replace_done;
			if (act == FT_DESCENT_BREAK)
				break;
			continue;
		}
		dbg_printf("_cds_ft_insert_replace iter ppnf %p pnf %p nfp %p nf %p\n",
				d.ppnf, d.pnf, d.nfp, d.nf);
		ft_snapshot_push(snapshot, snapshot_depth,
			nr_snapshot, d.nf, d.depth);
		key_value = *(iter_key++);
		ft_descent_step(&d, key_value);
	}

	/*
	 * Resolve any skip-compressed pointer left in d.nf by a final
	 * traverse that exited the loop without re-entering the loop's
	 * resolve step.
	 */
	d.nf = ft_resolve_skip_compressed(d.nf);

	if (d.depth == key_depth - 1) {
		/* Found either an internal, external node or NULL at end of key. */
		if (!d.nf) {
			/* No existing node. Regular attach. */
			dbg_printf("_cds_ft_insert_replace NULL at end of key\n");

			ret = ft_attach_node(ft, d.pnfp, d.pnf,
					d.nfp, d.nf, key, key_len, d.depth, node,
					NULL);
			if (ret == 0) {
				ft_propagate_external_count_parent(ft, *d.pnfp, 1);
				if (d.depth >= 2)
					FT_TP(tree_edge_set, (const void *) ft,
						(const void *) d.ppnf,
						(unsigned int) (d.depth - 2),
						(uint8_t) key[d.depth - 2],
						(const void *) *d.pnfp);
			}
		} else if (ft_node_compressed(d.nf)) {
			/*
			 * Key ends at a compressed node's depth.
			 * Split: internal(external_nodes) + compressed(len-1).
			 */
			ret = ft_insert_compressed_key_shorter(ft, &d, 0,
				node, old_node_ret);
			if (ret == -EEXIST)
				ret = 0;	/* Replace handled by key_shorter. */
		} else if (!ft_node_external(d.nf)) {
			struct cds_ft_node *external_nodes;
			struct cds_ft_metadata *metadata;

			assert(!ft_node_compressed(d.nf));
			metadata = cds_ft_item_to_metadata(ft_node_ptr(d.nf));
			external_nodes = metadata->external_nodes;
			if (external_nodes) {
				dbg_printf("_cds_ft_insert_replace: replacing internal metadata chain %p\n",
						external_nodes);
				/* Replace existing chain: key count unchanged. */
				*old_node_ret = external_nodes;
				node->prev = d.nf;
				node->next = NULL;
				rcu_assign_pointer(metadata->external_nodes, node);
			} else {
				/* No external nodes yet. New key. */
				node->prev = d.nf;
				node->next = NULL;
				rcu_assign_pointer(metadata->external_nodes, node);
				ft_propagate_external_count_parent(ft, d.nf, 1);
			}
			ret = 0;
		} else {
			dbg_printf("_cds_ft_insert_replace: replacing external chain %p\n",
					ft_node_ptr(d.nf));
			/* External node at end of key. Replace chain: key count unchanged. */
			*old_node_ret = (struct cds_ft_node *) ft_node_ptr(d.nf);
			node->prev = d.pnf;
			node->next = NULL;
			ft_publish_to_parent(ft, d.pnf, d.nfp,
				(struct cds_ft_inode_flag *) node);
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
		if (ret == 0) {
			ft_propagate_external_count_parent(ft, *d.pnfp, 1);
			if (d.depth >= 2)
				FT_TP(tree_edge_set, (const void *) ft,
					(const void *) d.ppnf,
					(unsigned int) (d.depth - 2),
					(uint8_t) key[d.depth - 2],
					(const void *) *d.pnfp);
		}
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

	CDS_FT_SCOPED_WRITER(ft);
	FT_TP_KEY(insert_replace_enter, ft, key, key_len);
	ret = _cds_ft_insert_replace(ft, key, key_len, node, &old_node);
	if (ret == -EINVAL) {
		*result_node = NULL;
		FT_TP(insert_replace_exit, (int) CDS_FT_STATUS_INVALID_ARGUMENT_ERROR);
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}
	if (ret) {
		*result_node = NULL;
		FT_TP(insert_replace_exit, (int) CDS_FT_STATUS_MEMORY_ERROR);
		return CDS_FT_STATUS_MEMORY_ERROR;
	}
	*result_node = old_node;
	if (old_node) {
		FT_TP(insert_replace_exit, (int) CDS_FT_STATUS_DUPLICATE_FOUND);
		return CDS_FT_STATUS_DUPLICATE_FOUND;
	}
	FT_TP(insert_replace_exit, (int) CDS_FT_STATUS_OK);
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
	struct cds_ft_node *iter_node, **head_slot, *match;
	const uint8_t *iter_key;
	size_t key_len = ft_key_len(ft, iter->key_len);
	enum cds_ft_status s;

	CDS_FT_SCOPED_WRITER(ft);
	FT_TP_ITER_KEY(replace_enter, iter);

	/*
	 * If the iterator has a valid path, the RCU read-side lock must
	 * be held.
	 */
	if (iter->path_valid)
		CDS_FT_ASSERT_RCU_READ_LOCKED(ft);
	iter_debug_path_check(iter);

	if (!valid_external_node(old_node) || !valid_external_node(new_node)
			|| !valid_key_len(ft, key_len)) {
		s = CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
		FT_TP(replace_exit, (int) s);
		return s;
	}
	/* Expect zeroed next and prev pointers on new_node. */
	if (new_node->next || new_node->prev) {
		s = CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
		FT_TP(replace_exit, (int) s);
		return s;
	}

	key_depth = key_len + 1;
	iter_key = iter_key(iter);

	dbg_printf("cds_ft_replace: old_node %p new_node %p\n", old_node, new_node);

	node_flag = ft->root;
	node_flag_ptr = &ft->root;

	/*
	 * Trie root is always an internal node (invariant enforced by
	 * ft_make_root_internal at the few sites that could otherwise
	 * publish a compressed or skip-compressed root).  No need to
	 * dispatch on tag here.
	 */

	/*
	 * Handle NIL key (key_len == 0).
	 */
	if (!key_len) {
		struct cds_ft_metadata *metadata;

		metadata = cds_ft_item_to_metadata(ft_node_ptr(node_flag));
		if (!metadata->external_nodes) {
			s = CDS_FT_STATUS_NOT_FOUND;
			FT_TP(replace_exit, (int) s);
			return s;
		}
		head_slot = (struct cds_ft_node **) &metadata->external_nodes;
		iter_node = metadata->external_nodes;
		goto find_and_replace;
	}

	/* Traverse internal levels. */
	for (i = 1; i < key_depth; i++) {
		uint8_t key_value;

		if (ft_node_external(node_flag)) {
			s = CDS_FT_STATUS_NOT_FOUND;
			FT_TP(replace_exit, (int) s);
			return s;
		}
		if (ft_node_compressed(node_flag)) {
			bool nf = false;
			enum ft_descent_action act;

			act = ft_traverse_compressed(&node_flag,
				&node_flag_ptr, &iter_key, &i,
				key_depth, &nf);
			if (nf) {
				s = CDS_FT_STATUS_NOT_FOUND;
				FT_TP(replace_exit, (int) s);
				return s;
			}
			if (act == FT_DESCENT_BREAK)
				break;
			continue;
		}
		key_value = *(iter_key++);
		node_flag = ft_node_get_nth(node_flag, &node_flag_ptr, key_value, FT_PF_NONE);
		if (!node_flag) {
			s = CDS_FT_STATUS_NOT_FOUND;
			FT_TP(replace_exit, (int) s);
			return s;
		}
	}

	/* Reached end of key. Locate the duplicate chain. */
	if (!ft_node_external(node_flag)) {
		struct cds_ft_metadata *metadata;

		metadata = cds_ft_item_to_metadata(ft_node_ptr(node_flag));
		if (!metadata->external_nodes) {
			s = CDS_FT_STATUS_NOT_FOUND;
			FT_TP(replace_exit, (int) s);
			return s;
		}
		head_slot = (struct cds_ft_node **) &metadata->external_nodes;
		iter_node = metadata->external_nodes;
	} else {
		head_slot = (struct cds_ft_node **) node_flag_ptr;
		iter_node = (struct cds_ft_node *) ft_node_ptr(node_flag);
	}

find_and_replace:
	/* Walk the duplicate chain to find old_node. */
	match = NULL;
	cds_ft_for_each_duplicate(iter_node) {
		if (iter_node == old_node) {
			match = iter_node;
			break;
		}
	}

	if (!match) {
		s = CDS_FT_STATUS_NOT_FOUND;
		FT_TP(replace_exit, (int) s);
		return s;
	}

	/*
	 * Splice new_node into the chain in place of old_node.
	 * new_node inherits old_node's prev and next pointers.
	 * The write barrier within rcu_assign_pointer ensures
	 * new_node->next is visible before the pointer that
	 * publishes new_node.
	 */
	new_node->prev = old_node->prev;
	new_node->next = old_node->next;
	if (new_node->next)
		new_node->next->prev = new_node;
	if (ft_node_external((struct cds_ft_inode_flag *) old_node->prev)) {
		/* Non-head: update predecessor's next pointer. */
		struct cds_ft_node *prev_node =
			(struct cds_ft_node *) old_node->prev;
		rcu_assign_pointer(prev_node->next, new_node);
	} else {
		/* Head: update the head slot. */
		rcu_assign_pointer(*head_slot, new_node);
	}

	/*
	 * The trie structure is unchanged (no recompaction), so the
	 * iterator path remains valid in cached mode.
	 */
	iter_auto_invalidate_path(iter);
	s = CDS_FT_STATUS_OK;
	FT_TP(replace_exit, (int) s);
	return s;
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

/*
 * Replace a compressed (or skip-compressed) parent in
 * ft_detach_node's structural-change phase.
 *
 * Two sub-cases:
 *
 *   - The detached child carried external_nodes that must be
 *     promoted to the compressed node's child slot
 *     (@topmost_external_nodes != NULL): keep the compressed node
 *     (its path is needed for lookups) and replace cn->child with
 *     the external chain head.  Reset *nr_clear so the
 *     free-intermediate walk does not run later.
 *
 *   - Otherwise: the compressed parent is no longer needed.
 *     Allocate a fresh empty internal, inherit parent + skip slot
 *     metadata, publish it in place of the compressed node, and
 *     free the compressed.  Returns -ENOMEM if the fresh
 *     allocation failed.
 */
static
int ft_detach_node_replace_compressed_parent(struct cds_ft *ft,
		struct cds_ft_inode_flag *iter_node_flag,
		struct cds_ft_inode_flag **detach_parent_flag_ptr,
		struct cds_ft_node *topmost_external_nodes,
		int *nr_clear)
{
	if (topmost_external_nodes) {
		/*
		 * Keep the compressed node — its path is needed for
		 * lookups to reach the correct depth.  Replace
		 * cn->child with the external node.
		 *
		 * Compressed nodes can have an external child
		 * (cn->child pointing to an external node) but must
		 * NOT have metadata->external_nodes set.
		 */
		struct cds_ft_compressed_node *cn;

		if (ft_node_skip_compressed(iter_node_flag))
			cn = ft_skip_to_compressed(iter_node_flag);
		else
			cn = ft_compressed_node_ptr(iter_node_flag);
		ft_publish_to_parent(ft, ft_compressed_node_flag(cn),
			&cn->child,
			(struct cds_ft_inode_flag *) topmost_external_nodes);
		/*
		 * Set the external's prev so that ft_skip_to_compressed
		 * can recover the compressed node from the skip pointer.
		 */
		ft_set_parent(
			(struct cds_ft_inode_flag *) topmost_external_nodes,
			ft_compressed_node_flag(cn), &cn->child);
		*nr_clear = 0;
		return 0;
	}
	{
		struct cds_ft_inode *fresh;
		struct cds_ft_metadata *fresh_meta;
		struct cds_ft_metadata *src_meta;

		fresh = alloc_cds_ft_node(ft, &ft_types[0], &fresh_meta);
		if (!fresh)
			return -ENOMEM;
		src_meta = cds_ft_item_to_metadata(
			(struct cds_ft_inode *) ft_compressed_node_ptr(
				iter_node_flag));
		fresh_meta->parent = src_meta->parent;
#ifdef FEATURE_FT_SKIP_COMPRESSED
		fresh_meta->skip_slot_offset = src_meta->skip_slot_offset;
#endif
		ft_publish_to_parent(ft, src_meta->parent,
			detach_parent_flag_ptr,
			ft_node_flag(fresh, 0));
		free_compressed_node(ft,
			ft_compressed_node_ptr(iter_node_flag));
	}
	return 0;
}

static
int ft_detach_node(struct cds_ft *ft,
		struct cds_ft_inode_flag **detach_node_flag_ptr,
		struct cds_ft_inode_flag **detach_parent_flag_ptr,
		unsigned int detach_depth,
		bool free_detached_subtree)
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
	 * Snapshot of the slot value at the detach point.  Set once the
	 * upward walk finishes elevating @detach_node_flag_ptr, before
	 * ft_node_replace_ptr overwrites the slot.  The free-walk below
	 * starts from this value so it covers BOTH the elevated single-
	 * child ancestors and the original detach target (the elevation
	 * walk only crosses nodes with nr_child==1, so the underlying
	 * chain reaches the original detach child).
	 */
	struct cds_ft_inode_flag *elevated_old_child;

	FT_TP(detach_node_enter, (const void *) *detach_node_flag_ptr, detach_depth);

	/*
	 * Check the node being replaced (the child at detach_node_flag_ptr)
	 * for external_nodes.  After the child's last internal child was
	 * removed (triggering this detach), the child may still hold
	 * variable-length key entries that must be preserved.
	 */
	{
		struct cds_ft_inode_flag *detach_child = *detach_node_flag_ptr;

		/*
		 * Resolve skip-compressed before type checks: a skip
		 * pointer with an external child has low bits == 0,
		 * falsely matching ft_node_external and skipping the
		 * external_nodes preservation entirely.
		 */
		detach_child = ft_resolve_skip_compressed(detach_child);

		if (detach_child && !ft_node_external(detach_child)) {
			struct cds_ft_metadata *child_meta =
				ft_flag_to_metadata(detach_child);
			if (child_meta && child_meta->external_nodes)
				topmost_external_nodes = child_meta->external_nodes;
		}
	}

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
		if (!prev_external_nodes_found && (metadata->nr_child == 1 && !metadata->external_nodes && !is_root)) {
			nr_clear++;
		}
		nr_branch++;
		if (prev_external_nodes_found || metadata->nr_child > 1 || metadata->external_nodes || is_root) {
			if (!is_root) {
				struct cds_ft_metadata *parent_meta =
					cds_ft_item_to_metadata(
						ft_node_ptr(metadata->parent));
				metadata_stack[nr_metadata++] = parent_meta;
			}
			/*
			 * Find the key byte for replace_ptr.  Only needed
			 * for internal parents (compressed handled
			 * separately below).
			 */
			if (!ft_node_compressed(cur))
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
#ifdef FT_IMMEDIATE_FREE
			{
				unsigned char *_p = (unsigned char *) ft_node_ptr(parent_nf);
				if (*_p == 0xfe) {
					fprintf(stderr, "ft_detach_node: stale parent detected! "
						"cur=%p cur_depth=%u parent_nf=%p (poisoned) "
						"detach_depth=%u nr_clear=%d\n",
						cur, cur_depth, parent_nf,
						detach_depth, nr_clear);
					abort();
				}
			}
#endif
			/*
			 * Find the slot in the grandparent pointing to
			 * cur, which becomes the new detach_parent_flag_ptr.
			 * Find the slot in cur pointing to its child (the
			 * previous level), which becomes detach_node_flag_ptr.
			 */
			{
				struct cds_ft_inode_flag **new_parent_flag_ptr;

				if (is_root)
					new_parent_flag_ptr = &ft->root;
				else if (ft_node_compressed(parent_nf) ||
					 ft_node_skip_compressed(parent_nf)) {
					struct cds_ft_compressed_node *pcn;

					if (ft_node_skip_compressed(parent_nf))
						pcn = ft_skip_to_compressed(parent_nf);
					else
						pcn = ft_compressed_node_ptr(parent_nf);
					new_parent_flag_ptr = &pcn->child;
				} else {
					ft_node_find_child(parent_nf, cur, NULL,
						&new_parent_flag_ptr);
				}
				/*
				 * If the resolved grandparent slot is the
				 * same as the current detach_parent_flag_ptr,
				 * stop: advancing would put both pointers at
				 * the same slot, breaking the replace which
				 * assumes detach_node_flag_ptr is WITHIN
				 * iter_node_flag's child array.
				 */
				if (new_parent_flag_ptr == detach_parent_flag_ptr)
					break;
				detach_node_flag_ptr = detach_parent_flag_ptr;
				detach_parent_flag_ptr = new_parent_flag_ptr;
			}
			cur_depth -= ft_parent_depth_span(parent_nf, cur);
			cur = parent_nf;
		}
	}

	iter_node_flag = *detach_parent_flag_ptr;
	elevated_old_child = *detach_node_flag_ptr;

	/*
	 * Replace within parent.  If the parent is a compressed node:
	 *
	 * If topmost_external_nodes is set, the child below the
	 * compressed node has variable-length key entries.  Keep the
	 * compressed node (its path is needed for lookups) and replace
	 * cn->child with the external node directly.
	 *
	 * Otherwise, replace the compressed node with a fresh internal
	 * node (e.g. compressed root from detach/graft_swap must remain
	 * internal).
	 */
	if (ft_node_compressed(iter_node_flag) ||
	    ft_node_skip_compressed(iter_node_flag)) {
		ret = ft_detach_node_replace_compressed_parent(ft,
			iter_node_flag, detach_parent_flag_ptr,
			topmost_external_nodes, &nr_clear);
		if (ret)
			goto end;
		/*
		 * Phase 2-style free walk for the orphaned chain below
		 * the compressed parent.  When a compressed cn's child
		 * was an internal node N with N->external_nodes
		 * promoted (or no external_nodes), the chain
		 * [N -> ... -> external_target] is now unreachable.
		 * Walk from @elevated_old_child (the original cn->child)
		 * down the single-child no-ext chain, collecting nodes
		 * to free.  The first iteration is special: the target
		 * itself may carry residual content (external_nodes)
		 * that was promoted as topmost_external_nodes.
		 */
		if (free_detached_subtree) {
			struct cds_ft_inode_flag *to_free[FT_MAX_DEPTH];
			int nr_to_free = 0, fi;
			struct cds_ft_inode_flag *walk_nf = elevated_old_child;
			bool phase2_first = true;

			while (walk_nf &&
			       !ft_node_external(walk_nf) &&
			       nr_to_free < FT_MAX_DEPTH) {
				struct cds_ft_inode_flag *next = NULL;
				unsigned int nr_child;
				struct cds_ft_node *ext_nodes;

				if (ft_node_compressed(walk_nf)) {
					struct cds_ft_compressed_node *cn;
					struct cds_ft_metadata *cm;

					cn = ft_compressed_node_ptr(
						ft_skip_child_ptr(walk_nf));
					cm = cds_ft_item_to_metadata(
						(struct cds_ft_inode *) cn);
					nr_child = cm->nr_child;
					ext_nodes = cm->external_nodes;
					next = cn->child;
				} else {
					struct cds_ft_metadata *m =
						cds_ft_item_to_metadata(
							ft_node_ptr(walk_nf));

					nr_child = m->nr_child;
					ext_nodes = m->external_nodes;
					if (nr_child == 1) {
						unsigned int key;

						for (key = 0; key < 256; key++) {
							next = ft_node_get_nth(
								walk_nf, NULL,
								(uint8_t) key,
								FT_PF_NONE);
							if (next)
								break;
						}
					}
				}

				if (!phase2_first &&
				    (nr_child > 1 || ext_nodes))
					break;
				phase2_first = false;
				to_free[nr_to_free++] = walk_nf;
				walk_nf = next;
			}
			for (fi = 0; fi < nr_to_free; fi++) {
				if (ft_node_compressed(to_free[fi]))
					free_compressed_node(ft,
						ft_compressed_node_ptr(
							ft_skip_child_ptr(to_free[fi])));
				else
					free_cds_ft_node(ft,
						ft_node_ptr(to_free[fi]));
			}
		}
	} else {
		/*
		 * Density was already propagated above (before
		 * structural changes).
		 */

		ret = ft_node_replace_ptr(ft,
			detach_node_flag_ptr,
			&iter_node_flag,
			&old_recompacted_node,
			metadata_stack[nr_branch - 1],
			n, (struct cds_ft_inode_flag *) topmost_external_nodes,
			detach_parent_flag_ptr == &ft->root,
			cur_depth);
		if (!ret) {
			/*
			 * Free the old detach subtree.  After
			 * ft_node_replace_ptr replaced it, the entire
			 * single-child chain from @elevated_old_child
			 * down is unreachable.  When the upward walk
			 * elevated the detach point, @elevated_old_child
			 * is the topmost ancestor that became orphaned;
			 * walking through its single-child chain reaches
			 * the original detach target (each elevated node
			 * has nr_child==1 by the walk's own invariant).
			 * Collect nodes first, then free after the walk
			 * completes (avoids use-after-free during
			 * traversal).  topmost_external_nodes (if any)
			 * was already saved and published at the
			 * replacement point.
			 */
			{
				struct cds_ft_inode_flag *to_free[FT_MAX_DEPTH];
				int nr_to_free = 0, fi;
				struct cds_ft_inode_flag *walk_nf = elevated_old_child;

				/*
				 * Free walk semantics:
				 *
				 *   Phase 1 — elevated ancestors (always
				 *   single-child no-external by the upward
				 *   walk's own invariant).  Free @nr_clear
				 *   nodes unconditionally.
				 *
				 *   Phase 2 — target and chain below.  For
				 *   destroy-style detach, walk the target's
				 *   single-child no-external chain (descent
				 *   tracking guarantees this) until we hit
				 *   an external, a multi-child node, a node
				 *   with external_nodes (its content was
				 *   either preserved as topmost_external_nodes
				 *   or still referenced), or nr_child == 0.
				 *   For move-style, stop — the target is the
				 *   new trie's root and must be preserved.
				 */
				/*
				 * Pointer classification used by the walks
				 * below:
				 *
				 *   ft_node_external() returns true for
				 *   genuine external leaves AND for
				 *   external skip-target encodings
				 *   (FT_SKIP_MASK on an external pointer):
				 *   in both cases the underlying node is an
				 *   external leaf that the caller (not the
				 *   free walk) is responsible for reclaiming.
				 *
				 *   ft_node_compressed() — checks
				 *   FT_COMPRESSED_MASK only — covers both
				 *   plain compressed and legacy high-bit
				 *   skip-compressed: both encodings keep
				 *   FT_COMPRESSED_MASK on the parent slot,
				 *   and ft_compressed_node_ptr() strips
				 *   FT_TAG_MASK to recover the cn pointer.
				 *
				 *   Otherwise (neither external nor
				 *   compressed) the pointer is an internal
				 *   node (possibly with FT_SKIP_MASK from a
				 *   Stage 4b internal skip-target; dormant
				 *   today).
				 */
				/* Phase 1: elevated ancestors. */
				while (nr_to_free < nr_clear &&
				       walk_nf &&
				       !ft_node_external(walk_nf) &&
				       nr_to_free < FT_MAX_DEPTH) {
					struct cds_ft_inode_flag *next = NULL;

					if (ft_node_compressed(walk_nf)) {
						struct cds_ft_compressed_node *cn;

						cn = ft_compressed_node_ptr(walk_nf);
						next = cn->child;
					} else {
						unsigned int key;

						for (key = 0; key < 256; key++) {
							next = ft_node_get_nth(
								walk_nf, NULL,
								(uint8_t) key, FT_PF_NONE);
							if (next)
								break;
						}
					}
					to_free[nr_to_free++] = walk_nf;
					walk_nf = next;
				}

				/* Phase 2: target and chain below. */
				if (free_detached_subtree) {
					bool phase2_first = true;

					while (walk_nf &&
					       !ft_node_external(walk_nf) &&
					       nr_to_free < FT_MAX_DEPTH) {
						struct cds_ft_inode_flag *next = NULL;
						unsigned int nr_child;
						struct cds_ft_node *ext_nodes;

						if (ft_node_compressed(walk_nf)) {
							struct cds_ft_compressed_node *cn;
							struct cds_ft_metadata *cm;

							cn = ft_compressed_node_ptr(walk_nf);
							cm = cds_ft_item_to_metadata(
								(struct cds_ft_inode *) cn);
							nr_child = cm->nr_child;
							ext_nodes = cm->external_nodes;
							next = cn->child;
						} else {
							struct cds_ft_metadata *m =
								cds_ft_item_to_metadata(
									ft_node_ptr(walk_nf));

							nr_child = m->nr_child;
							ext_nodes = m->external_nodes;
							if (nr_child == 1) {
								unsigned int key;

								for (key = 0; key < 256; key++) {
									next = ft_node_get_nth(
										walk_nf, NULL,
										(uint8_t) key, FT_PF_NONE);
									if (next)
										break;
								}
							}
						}

						/*
						 * Stop at content (multi-child
						 * or external_nodes), except
						 * on the very first phase-2
						 * iteration: that node is the
						 * detach target itself, which
						 * may carry residual content
						 * that was either promoted
						 * (topmost_external_nodes) or
						 * just cleared by the caller.
						 */
						if (!phase2_first &&
						    (nr_child > 1 || ext_nodes))
							break;
						phase2_first = false;
						to_free[nr_to_free++] = walk_nf;
						walk_nf = next;
					}
				}
				for (fi = 0; fi < nr_to_free; fi++) {
					if (ft_node_compressed(to_free[fi]))
						free_compressed_node(ft,
							ft_compressed_node_ptr(
								to_free[fi]));
					else
						free_cds_ft_node(ft,
							ft_node_ptr(to_free[fi]));
				}
			}
		}
	}
	if (ret)
		goto end;

	/*
	 * Update address of parent ptr in its parent.
	 * Skip for compressed parents: the replacement was already
	 * published inline above.
	 */
	if (!ft_node_compressed(iter_node_flag) &&
	    !ft_node_skip_compressed(iter_node_flag)) {
		struct cds_ft_metadata *iter_meta =
			cds_ft_item_to_metadata(ft_node_ptr(iter_node_flag));

		dbg_printf("ft_detach_node: publish %p instead of %p\n",
			iter_node_flag, *detach_parent_flag_ptr);
		ft_publish_to_parent(ft, iter_meta->parent,
			detach_parent_flag_ptr, iter_node_flag);

#ifdef FEATURE_FT_SKIP_COMPRESSED
		/*
		 * Post-detach canonicalization: if the surviving ancestor
		 * is now a non-root internal with exactly 1 live child and
		 * no external_nodes attached, replace the
		 * [parent_cn?, iter_internal, child_cn?] chain with a single
		 * compressed node — canonical form under
		 * FEATURE_FT_SKIP_COMPRESSED.
		 *
		 * Four sub-cases on whether the parent and surviving child
		 * are themselves compressed nodes:
		 *
		 *  - parent non-compressed, child non-compressed:
		 *      [iter_internal] → [new_cn(1 byte)]; child preserved.
		 *  - parent non-compressed, child compressed:
		 *      [iter_internal, child_cn] →
		 *      [new_cn(1 + child_cn.len bytes)];
		 *      new_cn.child = child_cn.child.
		 *  - parent compressed, child non-compressed:
		 *      [parent_cn, iter_internal] →
		 *      [new_cn(parent_cn.len + 1 bytes)];
		 *      new_cn.child = surviving_child.
		 *  - parent compressed, child compressed:
		 *      [parent_cn, iter_internal, child_cn] →
		 *      [new_cn(parent_cn.len + 1 + child_cn.len bytes)];
		 *      new_cn.child = child_cn.child.
		 *
		 * In all cases the merged compressed replaces the entire
		 * chain at the same trie position — preserving the
		 * "no two adjacent compresseds" invariant by absorbing any
		 * adjacent compressed neighbours into the new node.
		 *
		 * The chain-compress invariant guarantees that parent_cn's
		 * own parent (the grandparent) is non-compressed, so the
		 * publish at grandparent's slot does not need a recursive
		 * merge.
		 *
		 * Bounded by FT_SKIP_LEN_MAX (uint8_t cn->len): when the
		 * merged length would exceed the bound, fall back to leaving
		 * the residue.  Subsequent inserts may rebuild canonical
		 * form.
		 */
		if (iter_meta->nr_child == 1 &&
		    !iter_meta->external_nodes &&
		    iter_meta->parent != NULL) {
			uint8_t surviving_byte = 0;
			struct cds_ft_inode_flag *surviving_child =
				ft_node_get_minmax(iter_node_flag,
					&surviving_byte, FT_LEFTMOST,
					false /* writer; no validation */);
			bool parent_compressed = ft_node_compressed(
					iter_meta->parent);
			bool child_compressed = surviving_child
				&& ft_node_compressed(surviving_child);
			struct cds_ft_compressed_node *parent_cn =
				parent_compressed
				? ft_compressed_node_ptr(iter_meta->parent)
				: NULL;
			struct cds_ft_metadata *parent_cn_meta = parent_cn
				? cds_ft_item_to_metadata(
					(struct cds_ft_inode *) parent_cn)
				: NULL;
			struct cds_ft_compressed_node *child_cn =
				child_compressed
				? ft_compressed_node_ptr(surviving_child)
				: NULL;
			unsigned int parent_len = parent_cn ? parent_cn->len : 0;
			unsigned int child_len = child_cn ? child_cn->len : 0;
			unsigned int merged_len = parent_len + 1 + child_len;

			if (surviving_child && merged_len <= FT_SKIP_LEN_MAX) {
				struct cds_ft_metadata *new_cn_meta;
				struct cds_ft_compressed_node *new_cn =
					alloc_compressed_node(ft, merged_len,
						&new_cn_meta);
				if (new_cn) {
					struct cds_ft_inode_flag *new_cn_flag;
					struct cds_ft_inode_flag **publish_slot;
					struct cds_ft_inode_flag *publish_parent;

					/* Compose merged path bytes. */
					if (parent_cn)
						memcpy(new_cn->key_bytes,
							parent_cn->key_bytes,
							parent_len);
					new_cn->key_bytes[parent_len] = surviving_byte;
					if (child_cn)
						memcpy(&new_cn->key_bytes[parent_len + 1],
							child_cn->key_bytes,
							child_len);
					new_cn->len = (uint8_t) merged_len;
					new_cn->child = child_cn
						? child_cn->child
						: surviving_child;
					new_cn_meta->nr_child = 1;
					ft_nr_keys_store(new_cn_meta,
						ft_nr_keys_get(parent_cn_meta
							? parent_cn_meta
							: iter_meta),
						CMM_RELAXED);

					if (parent_cn) {
						/*
						 * Replace parent_cn at its
						 * own slot in the
						 * grandparent.  Inherit the
						 * grandparent context from
						 * parent_cn.
						 */
						new_cn_meta->parent =
							parent_cn_meta->parent;
						publish_parent =
							parent_cn_meta->parent;
						publish_slot = ft_get_skip_slot(
							parent_cn_meta, ft);
					} else {
						/*
						 * Replace iter_internal at
						 * its slot in the
						 * non-compressed parent.
						 */
						new_cn_meta->parent =
							iter_meta->parent;
						publish_parent =
							iter_meta->parent;
						publish_slot =
							detach_parent_flag_ptr;
					}
					ft_set_skip_slot(new_cn_meta,
						publish_slot);

					new_cn_flag = ft_compressed_node_flag(new_cn);
					ft_set_parent(new_cn->child, new_cn_flag,
						&new_cn->child);
					new_cn_flag = ft_publish_compressed(ft,
						new_cn, new_cn_flag);
					ft_publish_to_parent(ft, publish_parent,
						publish_slot, new_cn_flag);

					free_cds_ft_node(ft,
						ft_node_ptr(iter_node_flag));
					if (parent_cn)
						free_compressed_node(ft,
							parent_cn);
					if (child_cn)
						free_compressed_node(ft,
							child_cn);
				}
				/* Allocation failure: leave non-canonical
				 * residue in place; subsequent inserts may
				 * rebuild canonical form. */
			}
		}
#endif
	}
end:
	/* Reclaim safely after replacement. */
	if (old_recompacted_node)
		free_cds_ft_node(ft, old_recompacted_node);

	/*
	 * Density was already propagated before structural changes
	 * (above), while parent pointers were still valid.
	 */
	FT_TP(detach_node_exit, (int) ret);
	return ret;
}

/*
 * ft_unchain_node: remove @node from its duplicate chain using prev/next.
 *
 * @head_slot: address of the pointer that holds the head of the chain
 *             (e.g. &metadata->external_nodes or the parent's child slot).
 *             Only used when @node is the head of the chain.
 * @node:      the node to remove.
 *
 * For head nodes (node->prev is a flagged internal pointer): updates
 * *head_slot to point to node->next.
 * For non-head nodes (node->prev is a cds_ft_node): updates prev->next
 * to skip over node.
 * In both cases, if node->next exists, its prev pointer inherits
 * node->prev (either the parent pointer or the predecessor node).
 *
 * Ordering: next_node->prev is updated BEFORE the pointer publication
 * (*head_slot or prev_node->next).  If we published first, a concurrent
 * reader following the new head via a skip-compressed pointer could call
 * ft_skip_to_compressed and read the stale prev pointing to @node (the
 * node being removed, a cds_ft_node rather than the flagged parent),
 * returning a garbage compressed-node pointer.  rcu_assign_pointer on
 * the publication provides release semantics pairing with the reader's
 * rcu_dereference of child->prev.
 */
static
void ft_unchain_node(struct cds_ft_node **head_slot,
		struct cds_ft_node *node)
{
	struct cds_ft_node *next_node = node->next;

	FT_TP(unchain_node, (const void *) head_slot, (const void *) node,
		!ft_node_external((struct cds_ft_inode_flag *) node->prev));
	if (next_node)
		next_node->prev = node->prev;
	if (ft_node_external((struct cds_ft_inode_flag *) node->prev)) {
		/* Non-head: prev is a cds_ft_node. */
		struct cds_ft_node *prev_node =
			(struct cds_ft_node *) node->prev;
		rcu_assign_pointer(prev_node->next, next_node);
	} else {
		/* Head: prev is parent (flagged internal node pointer). */
		rcu_assign_pointer(*head_slot, next_node);
	}
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
/*
 * Handle a compressed node in cds_ft_remove / cds_ft_remove_all's
 * descent loop.  Match the remaining key bytes against the
 * compressed path:
 *
 *   - Divergence (j < cmp), key shorter than the compressed path
 *     (cn->len > remaining), or NULL child: the key is not
 *     present.  Return CDS_FT_STATUS_NOT_FOUND.
 *
 *   - Full match with non-NULL child: track the detach point,
 *     traverse through the compressed span, refresh the pending
 *     detach pointer if needed, and return CDS_FT_STATUS_OK so
 *     the caller continues the descent.
 */
static
enum cds_ft_status ft_remove_descent_compressed(
		struct ft_detach_descent *dd,
		const uint8_t **iter_key_p,
		size_t key_len)
{
	struct cds_ft_compressed_node *cn = ft_compressed_node_ptr(dd->d.nf);
	const struct cds_ft_metadata *cn_meta = cds_ft_item_to_metadata(
		(struct cds_ft_inode *) cn);
	unsigned int remaining = key_len - dd->d.depth;
	unsigned int cmp = cn->len < remaining ? cn->len : remaining;
	unsigned int j;

	j = ft_match_compressed_key(*iter_key_p, cn, cmp);
	if (j < cmp || cn->len > remaining || !cn->child)
		return CDS_FT_STATUS_NOT_FOUND;

	ft_detach_descent_track(dd, cn_meta);
	ft_descent_traverse_compressed(&dd->d, cn, iter_key_p);
	if (dd->d.nf && dd->pending) {
		dd->det_nfp = dd->d.nfp;
		dd->det_depth = dd->d.depth;
		dd->pending = false;
	}
	return CDS_FT_STATUS_OK;
}

enum cds_ft_status cds_ft_remove(struct cds_ft *ft,
		struct cds_ft_iter *iter,
		struct cds_ft_node *node)
{
	struct ft_detach_descent dd;
	struct cds_ft_node *iter_node, *match;
	int ret, count = 0;
	const uint8_t *iter_key;
	size_t key_len = ft_key_len(ft, iter->key_len);

	CDS_FT_SCOPED_WRITER(ft);
	FT_TP(remove_enter, (const void *) ft, (const void *) iter,
		iter_key(iter), key_len);

	/*
	 * If the iterator has a valid path, the RCU read-side lock must
	 * be held.
	 */
	if (iter->path_valid)
		CDS_FT_ASSERT_RCU_READ_LOCKED(ft);
	iter_debug_path_check(iter);

	if (!valid_external_node(node) || !valid_key_len(ft, key_len)) {
		FT_TP(remove_exit, (int) CDS_FT_STATUS_INVALID_ARGUMENT_ERROR);
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}

	iter_key = iter_key(iter);
	dbg_printf("cds_ft_remove attempt: node %p\n", node);

	ft_detach_descent_init(&dd, ft);

	/* Iterate on all internal levels */
	for (; dd.d.depth < key_len; ) {
		uint8_t key_value;
		const struct cds_ft_metadata *metadata;

		dbg_printf("cds_ft_remove iter nf %p\n",
				dd.d.nf);
		if (!dd.d.nf) {
			FT_TP(remove_exit, (int) CDS_FT_STATUS_NOT_FOUND);
			return CDS_FT_STATUS_NOT_FOUND;
		}
		/* Resolve skip-compressed pointer. */
		dd.d.nf = ft_resolve_skip_compressed(dd.d.nf);

		/*
		 * Compressed node: compare remaining key bytes with
		 * the compressed path.  If they match, traverse
		 * through to the child.  If they diverge, the key
		 * is not present.
		 */
		if (ft_node_compressed(dd.d.nf)) {
			enum cds_ft_status s;

			s = ft_remove_descent_compressed(&dd, &iter_key,
							 key_len);
			if (s != CDS_FT_STATUS_OK) {
				FT_TP(remove_exit, (int) s);
				return s;
			}
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
	if (!dd.d.nf) {
		dbg_printf("cds_ft_remove: no node found for key\n");
		FT_TP(remove_exit, (int) CDS_FT_STATUS_NOT_FOUND);
		return CDS_FT_STATUS_NOT_FOUND;
	}

	if (!ft_node_external(dd.d.nf)) {
		/* Found internal or compressed node at end of key. */
		struct cds_ft_node *external_nodes;
		struct cds_ft_metadata *metadata;

		metadata = cds_ft_item_to_metadata(ft_node_ptr(dd.d.nf));
		external_nodes = metadata->external_nodes;
		if (external_nodes) {
			/* Find our node in the duplicate chain. */
			iter_node = (struct cds_ft_node *) external_nodes;
			match = NULL;
			cds_ft_for_each_duplicate(iter_node) {
				dbg_printf("cds_ft_remove: compare %p with iter_node %p\n", node, iter_node);
				if (iter_node == node) {
					match = iter_node;
					break;
				}
			}
			if (!match) {
				dbg_printf("cds_ft_remove: no node match for node %p key\n", node);
				FT_TP(remove_exit, (int) CDS_FT_STATUS_NOT_FOUND);
				return CDS_FT_STATUS_NOT_FOUND;
			}
			/*
			 * Propagate -1 before unchain if this is the last
			 * entry in the chain (undercount ordering: decrement
			 * nr_keys before detaching the pointer).
			 */
			if (!ft_node_external((struct cds_ft_inode_flag *) match->prev)
			    && !match->next) {
				ft_propagate_external_count_parent(ft, dd.d.nf, -1);
			}
			ft_unchain_node((struct cds_ft_node **) &metadata->external_nodes, match);
			ret = 0;
		} else {
			dbg_printf("cds_ft_remove: no metadata external node found for key\n");
			FT_TP(remove_exit, (int) CDS_FT_STATUS_NOT_FOUND);
			return CDS_FT_STATUS_NOT_FOUND;
		}
	} else {
		/* Found external node at end of key. */

		/*
		 * Find our node in the duplicate chain and count
		 * total entries to decide detach vs unchain.
		 */
		iter_node = (struct cds_ft_node *) ft_node_ptr(dd.d.nf);
		count = 0;
		match = NULL;
		cds_ft_for_each_duplicate(iter_node) {
			count++;
			dbg_printf("cds_ft_remove: compare %p with iter_node %p\n", node, iter_node);
			if (!match && iter_node == node)
				match = iter_node;
		}
		if (!match) {
			dbg_printf("cds_ft_remove: no node match for node %p key\n", node);
			FT_TP(remove_exit, (int) CDS_FT_STATUS_NOT_FOUND);
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
			ft_propagate_external_count_parent(ft, dd.d.pnf, -1);
			ret = ft_detach_node(ft,
					dd.det_nfp,
					dd.det_pfp,
					dd.det_depth,
					true);
			if (ret) {
				/* Undo propagation on failure. */
				ft_propagate_external_count_parent(ft, dd.d.pnf, 1);
			}
		} else {
			/* Removing duplicate, not last: key count unchanged. */
			ft_unchain_node((struct cds_ft_node **) dd.d.nfp, match);
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
		FT_TP(remove_exit, (int) CDS_FT_STATUS_OK);
		return CDS_FT_STATUS_OK;
	case -ENOMEM:
		FT_TP(remove_exit, (int) CDS_FT_STATUS_MEMORY_ERROR);
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

	CDS_FT_SCOPED_WRITER(ft);
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
		ft_nr_keys_store(metadata, ft_nr_keys_get(metadata) - 1,
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
		if (!dd.d.nf) {
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
			enum cds_ft_status s;

			s = ft_remove_descent_compressed(&dd, &iter_key,
							 key_len);
			if (s != CDS_FT_STATUS_OK) {
				*result_node = NULL;
				return s;
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
	if (!dd.d.nf) {
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
		 * Atomically remove the entire chain.  The internal
		 * node itself remains as long as it still has
		 * children; if clearing external_nodes leaves it empty
		 * (nr_child == 0), detach it below.  A grace period
		 * must be observed before reclaiming any node in the
		 * old chain.
		 */
		/*
		 * Removing one key (with all its duplicates).
		 * Decrement before detach (undercount ordering).
		 */
		*result_node = external_nodes;
		ft_propagate_external_count_parent(ft, dd.d.nf, -1);
		rcu_assign_pointer(metadata->external_nodes, NULL);
		ret = 0;

		/*
		 * If the internal at end-of-key now has no children and
		 * no external_nodes, it is a leaf with no content — its
		 * entire single-child ancestor chain (up to the closest
		 * branch / external_nodes / root) is now orphaned and
		 * must be detached & freed.  Call ft_detach_node with
		 * the descent's current slot pointers: its upward walk
		 * will elevate detach_node_flag_ptr to the topmost
		 * orphaned ancestor and the free-walk will reclaim the
		 * whole chain.
		 */
		if (metadata->nr_child == 0 && dd.d.pnfp != NULL) {
			ret = ft_detach_node(ft,
					dd.d.nfp,
					dd.d.pnfp,
					dd.d.depth,
					true);
			if (ret) {
				/* Undo propagation on failure. */
				ft_propagate_external_count_parent(ft,
					dd.d.nf, 1);
			}
		}
	} else {
		/*
		 * External node at end of key. Detach the branch.
		 * Removing one key (with all its duplicates).
		 */
		*result_node = (struct cds_ft_node *) ft_node_ptr(dd.d.nf);
		/* Propagate before detach to avoid writing freed metadata. */
		ft_propagate_external_count_parent(ft, dd.d.pnf, -1);
		ret = ft_detach_node(ft,
				dd.det_nfp,
				dd.det_pfp,
				dd.det_depth,
				true);
		if (ret) {
			/* Undo propagation on failure. */
			ft_propagate_external_count_parent(ft, dd.d.pnf, 1);
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
		old_child_nr_keys = ft_nr_keys_get(cm);
	} else if (cn->child) {
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
		ft_nr_keys_store(sfx_meta, old_child_nr_keys,
			CMM_RELAXED);
		old_suffix_flag = ft_compressed_node_flag(sfx);
		ft_set_parent(cn->child, old_suffix_flag, &sfx->child);
		old_suffix_flag = ft_publish_compressed(ft, sfx, old_suffix_flag);
		created[nr_created++] = old_suffix_flag;
	} else if (suffix_len == 1) {
		struct cds_ft_inode_flag *dest = NULL;

		ret = ft_node_set_nth(ft, &dest,
				cn->key_bytes[diverge_pos + 1],
				cn->child, NULL, NULL,
				d->depth + diverge_pos + 1);
		if (ret) goto error;
		{
			struct cds_ft_metadata *m =
				cds_ft_item_to_metadata(ft_node_ptr(dest));
			ft_nr_keys_store(m, old_child_nr_keys,
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
				old_suffix_flag, NULL, NULL,
				d->depth + diverge_pos);
		if (ret) goto error;
		created[nr_created++] = dest;
		branch_flag = dest;

		branch_meta = cds_ft_item_to_metadata(ft_node_ptr(dest));
		ft_nr_keys_store(branch_meta, old_child_nr_keys,
			CMM_RELAXED);
	}

	/* 3. Build prefix -> branch (if needed). */
	if (diverge_pos >= 2 && !cn_meta->external_nodes) {
		struct cds_ft_compressed_node *pfx;
		struct cds_ft_metadata *pfx_meta;

		pfx = alloc_compressed_node(ft, diverge_pos, &pfx_meta);
		if (!pfx) goto error;
		pfx->child = branch_flag;
		pfx->len = diverge_pos;
		memcpy(pfx->key_bytes, cn->key_bytes, diverge_pos);
		pfx_meta->nr_child = 1;
		ft_nr_keys_store(pfx_meta, ft_nr_keys_get(cn_meta),
			CMM_RELAXED);
		top_flag = ft_compressed_node_flag(pfx);
		ft_set_parent(branch_flag, top_flag, NULL);
		top_flag = ft_publish_compressed(ft, pfx, top_flag);
		created[nr_created++] = top_flag;
	} else if (diverge_pos >= 2 && cn_meta->external_nodes) {
		/* Compressed prefix must not carry external_nodes.
		 * Internal wrapper at d->depth + compressed(len-1). */
		struct cds_ft_inode_flag *pfx_child = branch_flag;
		struct cds_ft_inode_flag *dest = NULL;
		struct cds_ft_metadata *int_meta;

		if (diverge_pos >= 3) {
			struct cds_ft_compressed_node *pfx;
			struct cds_ft_metadata *pfx_meta;

			pfx = alloc_compressed_node(ft, diverge_pos - 1, &pfx_meta);
			if (!pfx) goto error;
			pfx->child = branch_flag;
			pfx->len = diverge_pos - 1;
			memcpy(pfx->key_bytes, &cn->key_bytes[1], diverge_pos - 1);
			pfx_meta->nr_child = 1;
			ft_nr_keys_store(pfx_meta, ft_nr_keys_get(cn_meta),
				CMM_RELAXED);
			pfx_child = ft_compressed_node_flag(pfx);
			ft_set_parent(branch_flag, pfx_child, NULL);
			pfx_child = ft_publish_compressed(ft, pfx, pfx_child);
			created[nr_created++] = pfx_child;
		}
		ret = ft_node_set_nth(ft, &dest, cn->key_bytes[0],
				pfx_child, NULL, NULL, d->depth);
		if (ret) goto error;
		int_meta = cds_ft_item_to_metadata(ft_node_ptr(dest));
		ft_nr_keys_store(int_meta, ft_nr_keys_get(cn_meta),
			CMM_RELAXED);
		ft_metadata_set_external_nodes(dest, int_meta, cn_meta->external_nodes);
		top_flag = dest;
		created[nr_created++] = dest;
	} else if (diverge_pos == 1) {
		struct cds_ft_inode_flag *dest = NULL;
		struct cds_ft_metadata *pfx_meta;

		ret = ft_node_set_nth(ft, &dest, cn->key_bytes[0],
				branch_flag, NULL, NULL, d->depth);
		if (ret) goto error;
		pfx_meta = cds_ft_item_to_metadata(ft_node_ptr(dest));
		ft_nr_keys_store(pfx_meta, ft_nr_keys_get(cn_meta),
			CMM_RELAXED);
		if (cn_meta->external_nodes)
			ft_metadata_set_external_nodes(dest, pfx_meta, cn_meta->external_nodes);
		top_flag = dest;
		created[nr_created++] = dest;
	} else {
		/* diverge_pos == 0: branch IS the top. */
		struct cds_ft_metadata *branch_meta =
			cds_ft_item_to_metadata(ft_node_ptr(branch_flag));
		ft_nr_keys_store(branch_meta, ft_nr_keys_get(cn_meta),
			CMM_RELAXED);
		if (cn_meta->external_nodes)
			ft_metadata_set_external_nodes(branch_flag, branch_meta, cn_meta->external_nodes);
		top_flag = branch_flag;
	}

	/* 4. Publish the split structure. */
	ft_set_parent(top_flag, d->pnf, d->nfp);
	ft_publish_to_parent(ft, d->pnf, d->nfp, top_flag);

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
					cn->key_bytes[0], FT_PF_NONE);
	} else {
		d->pnfp = d->nfp;  /* branch IS the top, parent is the old parent */
	}
	{
		uint8_t new_ordinal = iter_key[diverge_pos];

		d->nf = ft_node_get_nth(branch_flag, &d->nfp, new_ordinal, FT_PF_NONE);
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
				free_compressed_node_unpublished(ft,
					ft_compressed_node_ptr(created[i]));
			else if (ft_node_skip_compressed(created[i]))
				free_compressed_node_unpublished(ft,
					ft_skip_to_compressed(created[i]));
			else
				free_cds_ft_node_unpublished(ft, ft_node_ptr(created[i]));
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

/*
 * Handle a compressed node in ft_descend_to_graft_point's descent
 * loop.  Three sub-cases:
 *
 *   - Exact / shorter-or-equal match (j == cmp && cn->len <=
 *     remaining): snapshot the compressed node, traverse it and
 *     return FT_DESCENT_CONTINUE so the caller continues the
 *     descent.
 *
 *   - Divergence (j < cmp): split the compressed node at the
 *     mismatch point.  Whether or not the split succeeded, the
 *     graft point has been identified; advance *ik_p past the
 *     diverged byte (only on split success) and return
 *     FT_DESCENT_BREAK so the caller exits the descent loop.
 *
 *   - Key shorter than the compressed path: split into
 *     prefix -> suffix at the key endpoint.  The graft point is
 *     the prefix/suffix boundary.  Return FT_DESCENT_BREAK in
 *     either outcome.
 */
static
enum ft_descent_action ft_descend_to_graft_point_compressed(
		struct cds_ft *ft,
		struct ft_descent *d,
		const uint8_t **ik_p,
		size_t key_len,
		struct cds_ft_inode_flag **snapshot,
		unsigned int *snapshot_depth,
		int *nr_snapshot)
{
	struct cds_ft_compressed_node *cn = ft_compressed_node_ptr(d->nf);
	const uint8_t *ik = *ik_p;
	int remaining = key_len - d->depth;
	int cmp = cn->len < remaining ? cn->len : remaining;
	int j = ft_match_compressed_key(ik, cn, cmp);

	if (j == cmp && cn->len <= remaining) {
		ft_snapshot_push(snapshot, snapshot_depth,
			*nr_snapshot, d->nf, d->depth);
		ft_descent_traverse_compressed(d, cn, &ik);
		*ik_p = ik;
		return FT_DESCENT_CONTINUE;
	}
	if (j < cmp) {
		/* Divergence: split compressed node at the mismatch point. */
		if (ft_split_compressed_graft(ft, d, ik, j))
			return FT_DESCENT_BREAK;
		*ik_p = ik + j + 1;
		return FT_DESCENT_BREAK;
	}
	/*
	 * Key shorter than compressed path: split into prefix ->
	 * suffix at the key endpoint.  The graft point is at the
	 * junction.
	 */
	(void) ft_split_compressed_graft_key_shorter(ft, d, remaining);
	return FT_DESCENT_BREAK;
}

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
		/* Resolve skip-compressed pointer. */
		d->nf = ft_resolve_skip_compressed(d->nf);
		if (ft_node_compressed(d->nf)) {
			enum ft_descent_action act;

			act = ft_descend_to_graft_point_compressed(ft, d,
				&ik, key_len, snapshot, snapshot_depth,
				nr_snapshot);
			if (act == FT_DESCENT_BREAK)
				break;
			assert(act == FT_DESCENT_CONTINUE);
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
		child_nr_keys = ft_nr_keys_get(cm);
	} else if (cn->child) {
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
		ft_nr_keys_store(sfx_meta, child_nr_keys,
			CMM_RELAXED);
		suffix_flag = ft_compressed_node_flag(sfx);
		ft_set_parent(cn->child, suffix_flag, &sfx->child);
		suffix_flag = ft_publish_compressed(ft, sfx, suffix_flag);
	} else {
		struct cds_ft_inode_flag *dest = NULL;
		int ret;

		ret = ft_node_set_nth(ft, &dest,
			cn->key_bytes[remaining],
			cn->child, NULL, NULL,
			d->depth + remaining);
		if (ret) return -1;
		{
			struct cds_ft_metadata *m =
				cds_ft_item_to_metadata(ft_node_ptr(dest));
			ft_nr_keys_store(m, child_nr_keys,
				CMM_RELAXED);
		}
		suffix_flag = dest;
	}

	/* Build prefix → suffix. */
	if (prefix_len >= 2 && !cn_meta->external_nodes) {
		struct cds_ft_compressed_node *pfx;
		struct cds_ft_metadata *pfx_meta;

		pfx = alloc_compressed_node(ft, prefix_len, &pfx_meta);
		if (!pfx) {
			if (ft_node_compressed(suffix_flag))
				free_compressed_node_unpublished(ft,
					ft_compressed_node_ptr(suffix_flag));
			else
				free_cds_ft_node_unpublished(ft, ft_node_ptr(suffix_flag));
			return -1;
		}
		pfx->child = suffix_flag;
		pfx->len = prefix_len;
		memcpy(pfx->key_bytes, cn->key_bytes, prefix_len);
		pfx_meta->nr_child = 1;
		ft_nr_keys_store(pfx_meta, ft_nr_keys_get(cn_meta),
			CMM_RELAXED);
		prefix_flag = ft_compressed_node_flag(pfx);
		ft_set_parent(suffix_flag, prefix_flag, &pfx->child);
		prefix_flag = ft_publish_compressed(ft, pfx, prefix_flag);
	} else if (prefix_len >= 2 && cn_meta->external_nodes) {
		/* Compressed prefix must not carry external_nodes.
		 * Internal wrapper at d->depth + compressed(len-1). */
		struct cds_ft_inode_flag *pfx_child = suffix_flag;
		struct cds_ft_inode_flag *dest = NULL;
		struct cds_ft_metadata *int_meta;
		int ret;

		if (prefix_len >= 3) {
			struct cds_ft_compressed_node *pfx;
			struct cds_ft_metadata *pfx_meta;

			pfx = alloc_compressed_node(ft, prefix_len - 1, &pfx_meta);
			if (!pfx) {
				if (ft_node_compressed(suffix_flag))
					free_compressed_node_unpublished(ft,
						ft_compressed_node_ptr(suffix_flag));
				else
					free_cds_ft_node_unpublished(ft, ft_node_ptr(suffix_flag));
				return -1;
			}
			pfx->child = suffix_flag;
			pfx->len = prefix_len - 1;
			memcpy(pfx->key_bytes, &cn->key_bytes[1], prefix_len - 1);
			pfx_meta->nr_child = 1;
			ft_nr_keys_store(pfx_meta, ft_nr_keys_get(cn_meta),
				CMM_RELAXED);
			pfx_child = ft_compressed_node_flag(pfx);
			ft_set_parent(suffix_flag, pfx_child, &pfx->child);
			pfx_child = ft_publish_compressed(ft, pfx, pfx_child);
		}
		ret = ft_node_set_nth(ft, &dest, cn->key_bytes[0],
				pfx_child, NULL, NULL, d->depth);
		if (ret) {
			if (prefix_len >= 3 && ft_node_compressed(pfx_child))
				free_compressed_node_unpublished(ft,
					ft_compressed_node_ptr(pfx_child));
			if (ft_node_compressed(suffix_flag))
				free_compressed_node_unpublished(ft,
					ft_compressed_node_ptr(suffix_flag));
			else
				free_cds_ft_node_unpublished(ft, ft_node_ptr(suffix_flag));
			return -1;
		}
		int_meta = cds_ft_item_to_metadata(ft_node_ptr(dest));
		ft_nr_keys_store(int_meta, ft_nr_keys_get(cn_meta),
			CMM_RELAXED);
		ft_metadata_set_external_nodes(dest, int_meta, cn_meta->external_nodes);
		prefix_flag = dest;
	} else {
		/* prefix_len == 1 */
		struct cds_ft_inode_flag *dest = NULL;
		struct cds_ft_metadata *pfx_meta;
		int ret;

		ret = ft_node_set_nth(ft, &dest, cn->key_bytes[0],
			suffix_flag, NULL, NULL, d->depth);
		if (ret) {
			if (ft_node_compressed(suffix_flag))
				free_compressed_node_unpublished(ft,
					ft_compressed_node_ptr(suffix_flag));
			else
				free_cds_ft_node_unpublished(ft, ft_node_ptr(suffix_flag));
			return -1;
		}
		pfx_meta = cds_ft_item_to_metadata(ft_node_ptr(dest));
		ft_nr_keys_store(pfx_meta, ft_nr_keys_get(cn_meta),
			CMM_RELAXED);
		if (cn_meta->external_nodes)
			ft_metadata_set_external_nodes(dest, pfx_meta, cn_meta->external_nodes);
		prefix_flag = dest;
	}

	/* Publish and set descent state. */
	ft_set_parent(prefix_flag, d->pnf, d->nfp);
	ft_publish_to_parent(ft, d->pnf, d->nfp, prefix_flag);

	d->ppnf = d->pnf;
	d->ppnfp = d->pnfp;
	d->pnf = prefix_flag;
	if (ft_node_compressed(prefix_flag))
		d->pnfp = &ft_compressed_node_ptr(prefix_flag)->child;
	else
		ft_node_get_nth(prefix_flag, &d->pnfp,
				cn->key_bytes[0], FT_PF_NONE);
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
		unsigned long subtree_external_count,
		bool has_external_nodes)
{
	/*
	 * When the caller will attach external_nodes to the top,
	 * the top must be an internal node (compressed nodes cannot
	 * carry metadata->external_nodes).  Bias compression to start
	 * one byte deeper so an internal node is created at `start`.
	 */
	unsigned int compress_start = has_external_nodes ? start + 1 : start;
	struct cds_ft_inode_flag *cur = leaf;
	int loop_top, i;

	if (start == end)
		return leaf;	/* path_len == 0. */

	/*
	 * Try compression over [compress_start, end).  Floor is len 1:
	 * a 1-byte compressed under FEATURE_FT_SKIP_COMPRESSED publishes
	 * as a SKIP_X-tagged slot pointer (free dispatch) and is the
	 * canonical replacement for what would otherwise be a non-root
	 * 1-child internal node.
	 */
	if (end >= compress_start + 1) {
		struct cds_ft_inode_flag *compressed;

		compressed = ft_try_compress_chain(ft, key, end,
			compress_start, leaf, NULL);
		if (compressed == (void *) (long) -ENOMEM)
			return NULL;
		if (compressed) {
			struct cds_ft_metadata *m =
				ft_flag_to_metadata(compressed);

			ft_nr_keys_store(m,
				subtree_external_count, CMM_RELAXED);
			cur = compressed;
			if (!has_external_nodes)
				return cur;
			/*
			 * has_external_nodes: fall through to create
			 * an internal node at `start` wrapping the
			 * compressed chunk.
			 */
		}
	}

	/*
	 * Create internal nodes from loop_top down to start.
	 *   Compression succeeded: only need an internal at `start`
	 *     (compress_start == start + 1, loop_top == start).
	 *   No compression:        create internal nodes for each
	 *     byte in [start, end).
	 */
	loop_top = (cur != leaf) ? (int) compress_start - 1 : (int) end - 1;
	for (i = loop_top; i >= (int) start; i--) {
		struct cds_ft_inode_flag *dest = NULL;
		int ret;

		ret = ft_node_set_nth(ft, &dest, key[i], cur,
			NULL, NULL, i);
		if (ret) {
			/*
			 * Free the created internal chain and, if
			 * present, the compressed chunk at the bottom.
			 */
			while (cur != leaf) {
				if (ft_node_compressed(cur)) {
					free_compressed_node(ft,
						ft_compressed_node_ptr(cur));
					cur = leaf;
				} else {
					struct cds_ft_inode_flag *next;
					uint8_t kv = key[i + 1];

					next = ft_node_get_nth(cur, NULL, kv, FT_PF_NONE);
					free_cds_ft_node(ft, ft_node_ptr(cur));
					cur = next;
					i++;
				}
			}
			return NULL;
		}
		ft_nr_keys_store(
			cds_ft_item_to_metadata(ft_node_ptr(dest)),
			subtree_external_count, CMM_RELAXED);
		/*
		 * Initialize density: this node's child (cur) may be
		 * the graft payload with an existing subtree.
		 * Bottom-up order ensures child density is set before
		 * parent.
		 */
		cur = dest;
	}
	return cur;
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
 * ft_make_root_internal: materialize an internal-node root for
 * assignment to ft->root, preserving the trie-wide invariant that
 * the root pointer always tags an internal node (never compressed,
 * never skip-compressed).
 *
 * @ft:    trie that will own the returned root (used for the
 *         allocator only — no concurrent readers are reached via
 *         @ft yet).
 * @child: incoming root candidate.  May be internal, regular
 *         compressed, or skip-compressed.  External @child must be
 *         filtered by the caller (externals go into the root's
 *         external_nodes slot, not via this helper).
 *
 * If @child is already internal, returns it unchanged.  Otherwise
 * peels the first byte off the compressed path (resolving skip
 * encoding first), allocates a fresh type-0 internal root, and
 * installs the slot at byte 0 with either:
 *   - cn->child directly (when cn->len == 1, the whole compressed
 *     collapses), or
 *   - a freshly-allocated shorter compressed node holding bytes
 *     [1..cn->len-1] → cn->child (when cn->len >= 2).
 *
 * The original cn is freed in the success path.  Subtree key count
 * (ft_nr_keys_get(cn_meta)) is propagated to the new root and to
 * any shorter compressed node so the parent count invariant is
 * preserved.
 *
 * Caller must be the sole owner of @child (no concurrent readers
 * may reach it via any published pointer); only valid for fresh /
 * exclusive trie roots.
 *
 * Returns the new internal-tagged root flag on success, NULL on
 * allocation failure (the caller's detach must roll back).
 */
static
struct cds_ft_inode_flag *
ft_make_root_internal(struct cds_ft *ft,
		struct cds_ft_inode_flag *child)
{
	struct cds_ft_compressed_node *cn;
	struct cds_ft_metadata *cn_meta;
	struct cds_ft_inode_flag *slot_value;
	struct cds_ft_compressed_node *new_cn = NULL;
	unsigned int suffix_len;
	uint8_t first_byte;
	unsigned long subtree_count;
	struct cds_ft_inode *root_node;
	struct cds_ft_metadata *root_meta;
	struct cds_ft_inode_flag *dest;
	int ret;

	/* Skip-compressed → regular compressed flag. */
	child = ft_resolve_skip_compressed(child);
	if (caa_likely(!ft_node_compressed(child)))
		return child;

	cn = ft_compressed_node_ptr(child);
	cn_meta = cds_ft_item_to_metadata((struct cds_ft_inode *) cn);
	subtree_count = ft_nr_keys_get(cn_meta);
	suffix_len = cn->len - 1;
	first_byte = cn->key_bytes[0];

	if (suffix_len == 0) {
		/* Single-byte compressed: cn collapses, cn->child plugs
		 * directly into the root slot. */
		slot_value = cn->child;
	} else {
		struct cds_ft_metadata *new_cn_meta;

		new_cn = alloc_compressed_node(ft, suffix_len, &new_cn_meta);
		if (!new_cn)
			return NULL;
		new_cn->len = suffix_len;
		new_cn->child = cn->child;
		memcpy(new_cn->key_bytes, &cn->key_bytes[1], suffix_len);
		new_cn_meta->nr_child = cn_meta->nr_child;
		ft_nr_keys_store(new_cn_meta, subtree_count, CMM_RELAXED);
		slot_value = ft_compressed_node_flag(new_cn);
		/*
		 * Re-parent the moved cn->child: its parent pointer
		 * still references the OLD cn (about to be freed).
		 * For the suffix_len == 0 case below, ft_node_set_nth
		 * does the equivalent when it places cn->child into
		 * the root slot.
		 */
		ft_set_parent(cn->child, slot_value, &new_cn->child);
	}

	root_node = alloc_cds_ft_node(ft, &ft_types[0], &root_meta);
	if (!root_node) {
		if (new_cn)
			free_compressed_node(ft, new_cn);
		return NULL;
	}
	dest = ft_node_flag(root_node, 0);
	ret = ft_node_set_nth(ft, &dest, first_byte, slot_value,
			NULL, root_meta, 0);
	if (ret) {
		if (new_cn)
			free_compressed_node(ft, new_cn);
		free_cds_ft_node(ft, root_node);
		return NULL;
	}
	ft_nr_keys_store(ft_flag_to_metadata(dest), subtree_count,
			CMM_RELAXED);
	free_compressed_node(ft, cn);
	return dest;
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
		unsigned long graft_external_count,
		struct cds_ft_inode_flag **attached_nf,
		unsigned int *attached_depth)
{
	struct cds_ft_inode *old_recompacted_node = NULL;

	if (d->depth == key_len) {
		struct cds_ft_metadata *pmeta;
		struct cds_ft_inode_flag *dest;
		int ret;

		if (d->nf)
			return CDS_FT_STATUS_POPULATED_ERROR;

		pmeta = cds_ft_item_to_metadata(ft_node_ptr(d->pnf));

		dest = d->pnf;
		ret = ft_node_set_nth(ft, &dest,
			key[key_len - 1],
			graft_payload, &old_recompacted_node, pmeta,
			d->depth - 1);
		if (ret)
			return CDS_FT_STATUS_MEMORY_ERROR;

		ft_publish_to_parent(ft, pmeta->parent, d->pnfp, dest);

		if (old_recompacted_node)
			free_cds_ft_node(ft, old_recompacted_node);
		*attached_nf = graft_payload;
		*attached_depth = key_len;
	} else {
		unsigned int i = d->depth;
		struct cds_ft_inode_flag *branch;
		struct cds_ft_node *displaced = NULL;

		if (d->nf && ft_node_external(d->nf))
			displaced = (struct cds_ft_node *)
				ft_node_ptr(d->nf);

		branch = ft_build_branch(ft, key, i, key_len, graft_payload,
				graft_external_count, displaced != NULL);
		if (!branch)
			return CDS_FT_STATUS_MEMORY_ERROR;

		if (displaced) {
			struct cds_ft_metadata *bm =
				ft_flag_to_metadata(branch);
			ft_metadata_set_external_nodes(branch, bm, displaced);
			ft_nr_keys_store(bm, ft_nr_keys_get(bm) + 1,
				CMM_RELAXED);
		}

		if (displaced) {
			ft_set_parent(branch, d->pnf, d->nfp);
			ft_publish_to_parent(ft, d->pnf, d->nfp, branch);
			if (i >= 1)
				FT_TP(tree_edge_set, (const void *) ft,
					(const void *) d->pnf,
					(unsigned int) (i - 1),
					(uint8_t) key[i - 1],
					(const void *) branch);
		} else {
			struct cds_ft_inode_flag *dest = d->pnf;
			struct cds_ft_metadata *pmeta;
			int ret;

			pmeta = cds_ft_item_to_metadata(
					ft_node_ptr(d->pnf));

			ret = ft_node_set_nth(ft, &dest,
				key[i - 1],
				branch, &old_recompacted_node, pmeta,
				d->depth - 1);
			if (ret) {
				ft_free_branch(ft, key, i, key_len, branch);
				return CDS_FT_STATUS_MEMORY_ERROR;
			}
			ft_publish_to_parent(ft, pmeta->parent,
				d->pnfp, dest);

			if (old_recompacted_node)
				free_cds_ft_node(ft, old_recompacted_node);
		}
		*attached_nf = branch;
		*attached_depth = d->depth;
	}
	return CDS_FT_STATUS_OK;
}

/*
 * ft_graft_keylen - Internal graft helper.
 *
 * Identical to cds_ft_graft except that:
 *   - @key_len is already resolved into bytes (no CDS_FT_LEN_DEFAULT).
 *   - The fixed-length-vs-non-root rejection is NOT performed.  This
 *     lets cds_ft_merge use a sub-prefix graft on fixed-length groups
 *     when paired with a matching ft_detach_keylen at the same prefix
 *     (the intermediate stripped-key state is purely internal and
 *     never visible to the caller).
 *   - Argument NULL/group/self checks and the FT_TP_KEY/FT_TP
 *     tracepoints are the public wrapper's responsibility.
 *
 * All other validation (overflow, memory, src empty) and the full
 * structural body — root-level swap or descent + ft_store_at_graft_point
 * + density propagation — are performed here, so this helper is the
 * single source of truth for what graft actually does.
 */
static
enum cds_ft_status ft_graft_keylen(struct cds_ft *dst_ft,
		const uint8_t *_key, size_t key_len,
		struct cds_ft *src_ft)
{
	struct cds_ft_metadata *src_rmeta;
	size_t src_max;
	enum cds_ft_status status;

	CDS_FT_SCOPED_WRITER(dst_ft);
	CDS_FT_SCOPED_WRITER(src_ft);

	const struct cds_ft_key_map *km = &dst_ft->group->key_map;
	uint8_t ordinal_buf[FT_MAX_KEY_LEN];
	const uint8_t *key;

	if (caa_likely(km->identity)) {
		key = _key;
	} else {
		ft_key_to_ordinals(ordinal_buf, _key, key_len, km);
		key = ordinal_buf;
	}

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
		struct cds_ft_inode *old_dst_root;

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
		 * Root-level graft: the source's root becomes the
		 * destination's root with no parent-pointer change
		 * (both are root positions with parent == NULL).  No
		 * "jump out" window, so no internal synchronize_rcu is
		 * required for this path.
		 *
		 * Swap root pointers.  The source's root carries all
		 * metadata (nr_child, external_nodes) with it.  The
		 * destination's old (empty) root is orphaned by the
		 * swap and must be reclaimed via call_rcu so concurrent
		 * readers that entered before the swap finish their
		 * descent first.
		 */
		old_dst_root = ft_node_ptr(dst_ft->root);
		rcu_assign_pointer(dst_ft->root, src_ft->root);
		FT_TP(root_publish, (const void *) dst_ft,
			(const void *) dst_ft->root);
		rcu_assign_pointer(src_ft->root, ft_node_flag(fresh_root, 0));
		FT_TP(root_publish, (const void *) src_ft,
			(const void *) src_ft->root);
		free_cds_ft_node(dst_ft, old_dst_root);
		goto done;
	}

	{
		struct ft_descent d;
		struct cds_ft_inode *fresh_node;
		struct cds_ft_metadata *fresh_meta;
		struct cds_ft_inode_flag *graft_snapshot[FT_MAX_DEPTH];
		unsigned int graft_snapshot_depth[FT_MAX_DEPTH];
		int nr_graft_snapshot;
		unsigned long src_count = ft_nr_keys_get(src_rmeta);
		struct cds_ft_inode_flag *old_src_root;

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
		 * "Jump out" prevention: a reader that has descended
		 * into src_ft's root subtree would, once the subtree's
		 * parent pointer is flipped to point into dst_ft,
		 * observe dst_ft's ancestor chain when backtracking via
		 * parent pointers.
		 *
		 * Correct ordering:
		 *   1. Unlink the old root from src_ft (publish a fresh
		 *      empty root) so no new reader can descend into
		 *      the payload via src_ft.
		 *   2. synchronize_rcu() drains readers that were
		 *      inside the payload before the unlink.
		 *   3. Re-parent and publish under dst_ft.  No reader
		 *      is present to observe the parent flip.
		 *
		 * Exclusive sources carry no RCU readers, so the sync
		 * is skipped in that case.
		 */
		old_src_root = src_ft->root;
		rcu_assign_pointer(src_ft->root, ft_node_flag(fresh_node, 0));
		FT_TP(root_publish, (const void *) src_ft,
			(const void *) src_ft->root);

		if (!src_ft->exclusive)
			src_ft->group->flavor->update_synchronize_rcu();

		/*
		 * The source's old root becomes the graft payload.  Its
		 * metadata.external_nodes (NIL-key entries in the source)
		 * naturally becomes the entries at depth key_len in the
		 * destination.  No relocation needed.
		 */
		struct cds_ft_inode_flag *attached_nf = NULL;
		unsigned int attached_depth = 0;

		status = ft_store_at_graft_point(dst_ft, key, key_len,
						  &d, old_src_root,
						  src_count,
						  &attached_nf,
						  &attached_depth);
		if (status != CDS_FT_STATUS_OK) {
			/*
			 * Roll back: restore old root in src_ft.  A
			 * second grace period drains readers that may
			 * have observed fresh_node before freeing it.
			 */
			rcu_assign_pointer(src_ft->root, old_src_root);
			FT_TP(root_publish, (const void *) src_ft,
				(const void *) src_ft->root);
			if (!src_ft->exclusive)
				src_ft->group->flavor->update_synchronize_rcu();
			free_cds_ft_node(src_ft, fresh_node);
			return status;
		}

		ft_propagate_external_count_parent(dst_ft, *d.pnfp,
				(long) src_count);

	}

done:
	{
		size_t nm = key_len + src_max;

		if (nm > uatomic_load(&dst_ft->max_used_key_len, CMM_RELAXED))
			uatomic_store(&dst_ft->max_used_key_len, nm,
				      CMM_RELAXED);
	}

	uatomic_store(&src_ft->max_used_key_len, 0, CMM_RELAXED);

	return CDS_FT_STATUS_OK;
}

enum cds_ft_status cds_ft_graft(struct cds_ft *dst_ft,
		const uint8_t *_key, size_t _key_len,
		struct cds_ft *src_ft)
{
	size_t key_len;
	enum cds_ft_status status;

	FT_TP_KEY(graft_enter, dst_ft, _key, _key_len);

	if (!dst_ft || !src_ft || dst_ft == src_ft) {
		FT_TP(graft_exit, (int) CDS_FT_STATUS_INVALID_ARGUMENT_ERROR);
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}
	if (dst_ft->group != src_ft->group) {
		FT_TP(graft_exit, (int) CDS_FT_STATUS_INVALID_ARGUMENT_ERROR);
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}

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
				dst_ft->group->key_len != CDS_FT_LEN_VARIABLE) {
			FT_TP(graft_exit, (int) CDS_FT_STATUS_INVALID_ARGUMENT_ERROR);
			return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
		}
	}

	status = ft_graft_keylen(dst_ft, _key, key_len, src_ft);
	FT_TP(graft_exit, (int) status);
	return status;
}

enum cds_ft_status cds_ft_graft_swap(struct cds_ft *dst_ft,
		const uint8_t *_key, size_t _key_len,
		struct cds_ft *swap_ft)
{
	size_t key_len, swap_max;

	FT_TP_KEY(graft_swap_enter, dst_ft, _key, _key_len);

	if (!dst_ft || !swap_ft || dst_ft == swap_ft) {
		FT_TP(graft_swap_exit, (int) CDS_FT_STATUS_INVALID_ARGUMENT_ERROR);
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}
	if (dst_ft->group != swap_ft->group) {
		FT_TP(graft_swap_exit, (int) CDS_FT_STATUS_INVALID_ARGUMENT_ERROR);
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}

	CDS_FT_SCOPED_WRITER(dst_ft);
	CDS_FT_SCOPED_WRITER(swap_ft);

	/*
	 * Root-level swap (key_len == 0) is valid for both
	 * variable-length and fixed-length groups.  See cds_ft_graft.
	 */
	if (_key_len == 0) {
		key_len = 0;
	} else {
		key_len = ft_key_len(dst_ft, _key_len);
		if (!valid_key_len(dst_ft, key_len) ||
				dst_ft->group->key_len != CDS_FT_LEN_VARIABLE) {
			FT_TP(graft_swap_exit, (int) CDS_FT_STATUS_INVALID_ARGUMENT_ERROR);
			return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
		}
	}

	const struct cds_ft_key_map *km = &dst_ft->group->key_map;
	uint8_t ordinal_buf[FT_MAX_KEY_LEN];
	const uint8_t *key;

	if (caa_likely(km->identity)) {
		key = _key;
	} else {
		ft_key_to_ordinals(ordinal_buf, _key, key_len, km);
		key = ordinal_buf;
	}

	swap_max = uatomic_load(&swap_ft->max_used_key_len, CMM_RELAXED);
	if (key_len > 0 && swap_max > dst_ft->group->max_key_len - key_len) {
		FT_TP(graft_swap_exit, (int) CDS_FT_STATUS_OVERFLOW_ERROR);
		return CDS_FT_STATUS_OVERFLOW_ERROR;
	}

	if (key_len == 0) {
		/*
		 * Swap entire tries: exchange root pointers.
		 * Each root carries its own metadata (nr_child,
		 * external_nodes), so no relocation is needed.
		 */
		struct cds_ft_inode_flag *tmp = dst_ft->root;
		size_t dm;
		bool dst_was_exclusive = dst_ft->exclusive;

		/*
		 * Drain concurrent readers of either side before
		 * re-parenting, to prevent readers in either trie from
		 * following parent pointers across the swap boundary.
		 */
		if (!swap_ft->exclusive || !dst_ft->exclusive)
			dst_ft->group->flavor->update_synchronize_rcu();

		rcu_assign_pointer(dst_ft->root, swap_ft->root);
		FT_TP(root_publish, (const void *) dst_ft,
			(const void *) dst_ft->root);
		rcu_assign_pointer(swap_ft->root, tmp);
		FT_TP(root_publish, (const void *) swap_ft,
			(const void *) swap_ft->root);

		dm = uatomic_load(&dst_ft->max_used_key_len, CMM_RELAXED);
		if (swap_max > dm)
			uatomic_store(&dst_ft->max_used_key_len,
				      swap_max, CMM_RELAXED);
		uatomic_store(&swap_ft->max_used_key_len, dm,
			      CMM_RELAXED);

		/*
		 * swap_ft now holds what was dst_ft's content; inherit
		 * dst_ft's prior access discipline.  dst_ft keeps its
		 * own discipline.
		 */
		swap_ft->exclusive = dst_was_exclusive;

		FT_TP(graft_swap_exit, (int) CDS_FT_STATUS_OK);
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
			enum cds_ft_status s;
			/*
			 * Path incomplete: nothing at or below the graft
			 * point.  swap_ft receives empty content.
			 * Delegate to graft for intermediate-node creation.
			 */
			s = cds_ft_graft(dst_ft, key, _key_len, swap_ft);
			FT_TP(graft_swap_exit, (int) s);
			return s;
		}

		/* Snapshot old content at the graft slot. */
		old_child = d.nf;

		old_swap_root = swap_ft->root;
		swap_rmeta = ft_root_metadata(swap_ft);
		swap_empty = (swap_rmeta->nr_child == 0
				&& !swap_rmeta->external_nodes);
		swap_count = swap_empty ? 0 : ft_nr_keys_get(swap_rmeta);

		/* Compute old_count from the content being displaced. */
		if (!ft_node_external(old_child)) {
			struct cds_ft_metadata *old_meta =
				cds_ft_item_to_metadata(ft_node_ptr(old_child));
			old_count = ft_nr_keys_get(old_meta);
		} else if (old_child) {
			old_count = 1;	/* One key (possibly with duplicates). */
		} else {
			old_count = 0;
		}

		/*
		 * A fresh empty root for swap_ft is needed in all
		 * non-empty swap cases.  Two consumers:
		 *   (i)  The upcoming unlink-before-sync step uses it
		 *        as the intermediate value of swap_ft->root
		 *        so that old_swap_root is no longer reachable
		 *        from swap before we change its parent pointer.
		 *   (ii) When old_child is external (the non-internal
		 *        case), old_child becomes a list of
		 *        external_nodes on fresh's metadata, and
		 *        swap_ft->root stays as fresh at the end.
		 * When old_child is an internal node, the final step
		 * overwrites swap_ft->root with old_child and frees
		 * fresh.
		 *
		 * Preallocate here, before the point of no return, so
		 * we can fail cleanly on memory shortage.
		 */
		need_fresh = !swap_empty;
		if (need_fresh) {
			fresh = alloc_cds_ft_node(swap_ft,
				&ft_types[0], &fresh_meta);
			if (!fresh) {
				FT_TP(graft_swap_exit, (int) CDS_FT_STATUS_MEMORY_ERROR);
				return CDS_FT_STATUS_MEMORY_ERROR;
			}
		}

		/*
		 * "Jump out" prevention: a reader inside swap_ft that
		 * has descended into old_swap_root's subtree would,
		 * once old_swap_root's parent pointer is flipped to
		 * point into dst_ft, observe dst_ft's ancestor chain
		 * when backtracking via parent pointers.
		 *
		 * Correct ordering (non-empty swap):
		 *   1. Unlink old_swap_root from swap_ft (install
		 *      fresh as swap_ft's root) so no new reader can
		 *      descend into old_swap_root via swap.
		 *   2. synchronize_rcu() drains readers that were
		 *      inside old_swap_root before the unlink.
		 *   3. Re-parent and publish under dst_ft.  No reader
		 *      is present to observe the parent flip.
		 *
		 * Consequence: readers on swap_ft briefly see an empty
		 * trie between steps 1 and the final installation of
		 * old_child (below).  This is a weaker atomicity than
		 * what the cached-path implementation provided, but
		 * preserves the "never jump out of the trie" invariant
		 * which matters for the parent-pointer backtracking
		 * read path.
		 */
		if (!swap_empty) {
			rcu_assign_pointer(swap_ft->root,
				ft_node_flag(fresh, 0));
			FT_TP(root_publish, (const void *) swap_ft,
				(const void *) swap_ft->root);
			if (!swap_ft->exclusive)
				swap_ft->group->flavor->update_synchronize_rcu();
		}

		/*
		 * Atomic store at graft point.  If swap is empty,
		 * place NULL (removing the subtree); otherwise place
		 * the swap root node directly.
		 */
		if (!swap_empty)
			ft_set_parent(old_swap_root, d.pnf, d.nfp);
		ft_publish_to_parent(dst_ft, d.pnf, d.nfp,
			swap_empty ? NULL : old_swap_root);
		/*
		 * graft_swap replaces the subtree at key_len in dst_ft
		 * via ft_publish_to_parent directly; no ft_node_set_nth
		 * call, so no tree_edge_set fires for the outer edit.
		 * Emit the structural edge for consumers.
		 */
		if (d.depth >= 1)
			FT_TP(tree_edge_set, (const void *) dst_ft,
				(const void *) d.pnf,
				(unsigned int) (d.depth - 1),
				(uint8_t) _key[d.depth - 1],
				(const void *) (swap_empty ? NULL :
					old_swap_root));

		/* Update parent nr_child on NULL <-> non-NULL transition. */
		pmeta = cds_ft_item_to_metadata(ft_node_ptr(d.pnf));
		if (!old_child && !swap_empty)
			pmeta->nr_child++;
		else if (old_child && swap_empty)
			pmeta->nr_child--;

		/* Propagate external node count delta through ancestors. */
		if (swap_count != old_count)
			ft_propagate_external_count_parent(dst_ft, d.pnf,
					(long) swap_count - (long) old_count);

		/*
		 * Set up swap_ft to hold old content from the graft
		 * point.  For non-empty swap, swap_ft->root was already
		 * set to @fresh above (as part of the unlink-before-
		 * sync step).  We now either overwrite it with old_child
		 * (when old_child is an internal node — @fresh becomes
		 * unused and is freed) or attach old_child as
		 * external_nodes on @fresh.
		 *
		 * For empty swap, swap_ft->root stays as old_swap_root
		 * (swap's original empty root) and old_child attaches
		 * there as external_nodes.
		 */
		if (!ft_node_external(old_child)) {
			/*
			 * Materialize an internal root from @old_child if
			 * needed (compressed / skip-compressed cases): the
			 * trie root invariant requires internal here.
			 * swap_ft has been drained of readers by the
			 * synchronize_rcu above, so @old_child is safe to
			 * rewrite structurally.
			 *
			 * On allocation failure we have no good roll-back
			 * path post-publish; abort.
			 */
			struct cds_ft_inode_flag *new_root_child;

			new_root_child = ft_make_root_internal(swap_ft, old_child);
			if (!new_root_child) {
				fprintf(stderr,
					"BUG: ft_make_root_internal OOM during graft_swap\n");
				abort();
			}
			old_child = new_root_child;
			/*
			 * Clear parent: old_child is now a root.  Use
			 * rcu_assign_pointer so read-side parent-pointer
			 * walks see a single atomic transition.
			 */
			{
				struct cds_ft_metadata *m = cds_ft_item_to_metadata(
					ft_node_ptr(old_child));
				rcu_assign_pointer(m->parent, NULL);
#ifdef FEATURE_FT_SKIP_COMPRESSED
				m->skip_slot_offset = 0;
#endif
			}
			rcu_assign_pointer(swap_ft->root, old_child);
			FT_TP(root_publish, (const void *) swap_ft,
				(const void *) swap_ft->root);
			if (swap_empty)
				free_cds_ft_node(swap_ft,
					ft_node_ptr(old_swap_root));
			else
				free_cds_ft_node(swap_ft, fresh);
		} else if (swap_empty) {
			if (old_child) {
				ft_metadata_set_external_nodes(old_swap_root, swap_rmeta,
					(struct cds_ft_node *)
					ft_node_ptr(old_child));
				ft_nr_keys_store(swap_rmeta, old_count, CMM_RELEASE);
			}
		} else {
			/* swap_ft->root is already @fresh from the unlink step. */
			if (old_child) {
				ft_metadata_set_external_nodes(ft_node_flag(fresh, 0), fresh_meta,
					(struct cds_ft_node *)
					ft_node_ptr(old_child));
				ft_nr_keys_store(fresh_meta, old_count, CMM_RELEASE);
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
		/*
		 * swap_ft now holds content displaced from dst_ft;
		 * inherit dst_ft's access discipline for that content.
		 * dst_ft keeps its own discipline.
		 */
		swap_ft->exclusive = dst_ft->exclusive;
		FT_TP(graft_swap_exit, (int) CDS_FT_STATUS_OK);
		return CDS_FT_STATUS_OK;
	}
}

/*
 * ft_detach_keylen - Internal detach helper.
 *
 * Identical to cds_ft_detach except that:
 *   - @key_len is already resolved into bytes (no CDS_FT_LEN_DEFAULT).
 *   - The fixed-length-vs-non-root rejection is NOT performed.  See
 *     ft_graft_keylen for the merge use case that motivates this.
 *   - Argument NULL check and the FT_TP_KEY/FT_TP tracepoints are
 *     the public wrapper's responsibility.
 *
 * The detached subtree handle returned for non-root detach in a
 * fixed-length group has keys shorter than the group's fixed length;
 * it is therefore an internal-use-only handle and must be re-grafted
 * (via ft_graft_keylen at the same prefix) before any public API
 * consumer interacts with it.
 */
static
enum cds_ft_status ft_detach_keylen(struct cds_ft *ft,
		const uint8_t *_key, size_t key_len,
		struct cds_ft **result_ft)
{
	struct cds_ft *detached;
	struct cds_ft_inode_flag *child;
	enum cds_ft_status status;

	*result_ft = NULL;

	CDS_FT_SCOPED_WRITER(ft);

	const struct cds_ft_key_map *km = &ft->group->key_map;
	uint8_t ordinal_buf[FT_MAX_KEY_LEN];
	const uint8_t *key;

	if (caa_likely(km->identity)) {
		key = _key;
	} else {
		ft_key_to_ordinals(ordinal_buf, _key, key_len, km);
		key = ordinal_buf;
	}

	if (key_len == 0) {
		struct cds_ft_metadata *rmeta = ft_root_metadata(ft);
		struct cds_ft_inode *fresh_node;
		struct cds_ft_metadata *fresh_meta;

		/* Check if source trie is empty. */
		if (rmeta->nr_child == 0 && !rmeta->external_nodes)
			return CDS_FT_STATUS_NOT_FOUND;

		status = cds_ft_create(ft->group, NULL, &detached);
		if (status != CDS_FT_STATUS_OK)
			return status;
		/*
		 * The detached trie is returned exclusive: no external
		 * handle to @detached existed before this call, so no
		 * RCU reader can be inside it at return.  A subsequent
		 * graft of @detached therefore skips its synchronize_rcu,
		 * coalescing the detach+graft pair to a single grace
		 * period.  Callers that publish @detached to concurrent
		 * readers must call cds_ft_make_concurrent first.
		 */
		detached->exclusive = true;
#ifdef FEATURE_FT_VERIFY_AT_MUTATION
		/*
		 * Carry the source's verify-at-mutation cadence into the
		 * detached trie.  Otherwise the detached trie would reset
		 * to the default period of 1 and re-introduce the O(N)
		 * per-mutation cost on the detached subtree, defeating the
		 * very reason the source was tuned to a larger period.
		 * Counter is reset (calloc'd in cds_ft_create).
		 */
		detached->verify_at_mutation_period = ft->verify_at_mutation_period;
#endif

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
		FT_TP(root_publish, (const void *) detached,
			(const void *) detached->root);
		/*
		 * Clear parent: this node is now a root.  Use
		 * rcu_assign_pointer so read-side parent-pointer walks
		 * see a single atomic transition.
		 */
		{
			struct cds_ft_metadata *m = cds_ft_item_to_metadata(
				ft_node_ptr(detached->root));
			rcu_assign_pointer(m->parent, NULL);
#ifdef FEATURE_FT_SKIP_COMPRESSED
			m->skip_slot_offset = 0;
#endif
		}
		uatomic_store(&detached->max_used_key_len,
			      uatomic_load(&ft->max_used_key_len, CMM_RELAXED),
			      CMM_RELAXED);

		/* Give source a fresh empty root. */
		rcu_assign_pointer(ft->root, ft_node_flag(fresh_node, 0));
		FT_TP(root_publish, (const void *) ft, (const void *) ft->root);

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

			if (!dd.d.nf)
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
				if (dd.d.nf && dd.pending) {
					dd.det_nfp = dd.d.nfp;
					dd.pending = false;
				}
				continue;
			}
			meta = cds_ft_item_to_metadata(ft_node_ptr(dd.d.nf));
			ft_detach_descent_track(&dd, meta);

			kv = *(ik++);
			ft_detach_descent_step(&dd, kv);
		}

		child = dd.d.nf;

		if (!child)
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
				detached_count = ft_nr_keys_get(child_meta);
			} else {
				detached_count = 1;	/* One key (possibly with duplicates). */
			}

			status = cds_ft_create(ft->group, NULL, &detached);
			if (status != CDS_FT_STATUS_OK)
				return status;
			/*
			 * The detached trie is returned exclusive: the
			 * synchronize_rcu below drains in-flight readers of
			 * the source before publishing @child as @detached's
			 * root, so no RCU reader is inside @detached at
			 * return.  Callers that publish @detached to
			 * concurrent readers must call cds_ft_make_concurrent
			 * first.
			 */
			detached->exclusive = true;
#ifdef FEATURE_FT_VERIFY_AT_MUTATION
			/* Mirror of the root-detach branch above; see rationale there. */
			detached->verify_at_mutation_period = ft->verify_at_mutation_period;
#endif

			/*
			 * Propagate count removal through ancestors
			 * before detach to avoid writing freed metadata.
			 */
			ft_propagate_external_count_parent(ft, dd.d.pnf,
				-(long) detached_count);

			/*
			 * Detach child from the source trie and prune
			 * empty branches above.  After this, child is
			 * no longer reachable from the live trie for
			 * new readers.
			 */
			{
				/*
				 * Save the slot value at @dd.det_nfp before
				 * ft_detach_node nulls it: this is the head
				 * of the single-child chain between the
				 * source's topmost branch point and the move
				 * target.  Move-style ft_detach_node
				 * preserves @child but does NOT free the
				 * intermediate chain; we free it explicitly
				 * after the detach so it is not leaked.
				 */
				struct cds_ft_inode_flag *chain_head = *dd.det_nfp;
				/*
				 * Subtree-move detach: preserve @child as
				 * the root of the new @detached trie.  The
				 * caller's saved @child pointer keeps the
				 * detached subtree alive; ft_detach_node
				 * must not free it.
				 */
				int ret = ft_detach_node(ft,
							 dd.det_nfp,
							 dd.det_pfp,
							 dd.det_depth,
							 false);
				assert(ret != -ENOENT);
				if (ret < 0) {
					/*
					 * Recompaction failed (-ENOMEM).
					 * Undo propagation and abort.
					 */
					ft_propagate_external_count_parent(ft,
						dd.d.pnf,
						(long) detached_count);
					cds_ft_destroy(detached);
					return CDS_FT_STATUS_MEMORY_ERROR;
				}
				/*
				 * Walk from @chain_head down toward @child,
				 * reclaiming every compressed / internal
				 * single-child no-external link in between.
				 * Stop at @child (move target, preserved) or
				 * any earlier divergence we did not expect.
				 *
				 * After ft_detach_node + ft_node_replace_ptr
				 * the chain is unreachable from the source
				 * root and call_rcu via free_*_node defers
				 * reclamation until in-flight readers have
				 * left.
				 */
				while (chain_head &&
				       chain_head != child &&
				       !ft_node_external(chain_head)) {
					struct cds_ft_inode_flag *next = NULL;

					if (ft_node_compressed(chain_head)) {
						struct cds_ft_compressed_node *cn;
						struct cds_ft_metadata *cm;

						cn = ft_compressed_node_ptr(
							ft_skip_child_ptr(chain_head));
						cm = cds_ft_item_to_metadata(
							(struct cds_ft_inode *) cn);
						if (cm->nr_child != 1 ||
						    cm->external_nodes)
							break;
						next = cn->child;
						free_compressed_node(ft, cn);
					} else {
						struct cds_ft_metadata *m =
							cds_ft_item_to_metadata(
								ft_node_ptr(chain_head));
						unsigned int kv;

						if (m->nr_child != 1 ||
						    m->external_nodes)
							break;
						for (kv = 0; kv < 256; kv++) {
							next = ft_node_get_nth(
								chain_head, NULL,
								(uint8_t) kv, FT_PF_NONE);
							if (next)
								break;
						}
						free_cds_ft_node(ft,
							ft_node_ptr(chain_head));
					}
					chain_head = next;
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
				/*
				 * Drain source-trie readers that entered
				 * before ft_detach_node published the unlink
				 * and may still hold pointers into @child's
				 * subtree.  Without this grace period, such a
				 * reader's going-up walk can observe the
				 * @child.parent = NULL store published below
				 * while its @cur_nf is still a node inside the
				 * subtree, break out of its walk as if it had
				 * reached @ft's root, and return a spurious
				 * result drawn from the now-detached internal
				 * pointer chain -- the escape that the
				 * inv_ordered_no_escape_graft invariant guards
				 * against.  Skip for exclusive sources: those
				 * carry no RCU readers by construction.
				 *
				 * After this grace period, no reader holds a
				 * pointer into @child's subtree; combined with
				 * @detached being a fresh handle, @detached has
				 * no concurrent readers and is returned in
				 * exclusive mode.  A subsequent graft of
				 * @detached therefore skips its own GP,
				 * coalescing detach+graft to a single grace
				 * period.
				 */
				if (!ft->exclusive)
					ft->group->flavor->update_synchronize_rcu();
				/*
				 * Materialize an internal root if @child is
				 * compressed or skip-compressed: the trie
				 * root invariant requires an internal node
				 * here.  See ft_make_root_internal.
				 */
				child = ft_make_root_internal(detached, child);
				if (!child) {
					/*
					 * Allocation failure during root
					 * materialization.  Re-attach the
					 * subtree to source and surface
					 * MEMORY_ERROR.  The propagate-down
					 * undo mirrors the original detach.
					 */
					ft_propagate_external_count_parent(ft,
						dd.d.pnf,
						(long) detached_count);
					cds_ft_destroy(detached);
					return CDS_FT_STATUS_MEMORY_ERROR;
				}
				free_cds_ft_node(detached,
					ft_node_ptr(detached->root));
				/* No readers in detached root yet. */
				detached->root = child;
				FT_TP(root_publish, (const void *) detached,
					(const void *) detached->root);
				/*
				 * Clear parent: this node is now a root.
				 * Use rcu_assign_pointer so read-side
				 * parent-pointer walks see a single atomic
				 * transition.
				 */
				{
					struct cds_ft_metadata *m = cds_ft_item_to_metadata(
						ft_node_ptr(child));
					rcu_assign_pointer(m->parent, NULL);
#ifdef FEATURE_FT_SKIP_COMPRESSED
					m->skip_slot_offset = 0;
#endif
				}
			} else {
				struct cds_ft_metadata *dmeta =
					ft_root_metadata(detached);
				ft_metadata_set_external_nodes(detached->root, dmeta,
					(struct cds_ft_node *)
					ft_node_ptr(child));
				ft_nr_keys_store(dmeta, detached_count, CMM_RELAXED);
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
		return CDS_FT_STATUS_OK;
	}
}

enum cds_ft_status cds_ft_detach(struct cds_ft *ft,
		const uint8_t *_key, size_t _key_len,
		struct cds_ft **result_ft)
{
	size_t key_len;
	enum cds_ft_status status;

	FT_TP_KEY(detach_enter, ft, _key, _key_len);

	*result_ft = NULL;

	if (!ft) {
		FT_TP(detach_exit, (int) CDS_FT_STATUS_INVALID_ARGUMENT_ERROR);
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}

	/*
	 * Root-level detach (key_len == 0) is valid for both
	 * variable-length and fixed-length groups.  See cds_ft_graft.
	 */
	if (_key_len == 0) {
		key_len = 0;
	} else {
		key_len = ft_key_len(ft, _key_len);
		if (!valid_key_len(ft, key_len) ||
				ft->group->key_len != CDS_FT_LEN_VARIABLE) {
			FT_TP(detach_exit, (int) CDS_FT_STATUS_INVALID_ARGUMENT_ERROR);
			return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
		}
	}

	status = ft_detach_keylen(ft, _key, key_len, result_ft);
	FT_TP(detach_exit, (int) status);
	return status;
}

enum cds_ft_status cds_ft_merge_at(struct cds_ft *dst_ft,
		const uint8_t *dst_key, size_t dst_key_len,
		struct cds_ft *src_ft,
		const uint8_t *src_key, size_t src_key_len)
{
	struct cds_ft *subtree = NULL;
	enum cds_ft_status status;

	FT_TP(merge_enter, (const void *) dst_ft, (const void *) src_ft);

	if (!dst_ft || !src_ft || dst_ft == src_ft) {
		FT_TP(merge_exit, (int) CDS_FT_STATUS_INVALID_ARGUMENT_ERROR);
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}
	if (dst_ft->group != src_ft->group) {
		FT_TP(merge_exit, (int) CDS_FT_STATUS_INVALID_ARGUMENT_ERROR);
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}
	if (dst_key_len > dst_ft->group->max_key_len ||
			src_key_len > dst_ft->group->max_key_len) {
		FT_TP(merge_exit, (int) CDS_FT_STATUS_INVALID_ARGUMENT_ERROR);
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}
	if ((dst_key_len > 0 && !dst_key) ||
			(src_key_len > 0 && !src_key)) {
		FT_TP(merge_exit, (int) CDS_FT_STATUS_INVALID_ARGUMENT_ERROR);
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}
	/*
	 * Fixed-length groups: each source key K has length fixed_len,
	 * the moved subtree's stripped keys have length
	 * (fixed_len - src_key_len), and the resulting destination key
	 * is dst_key || stripped, of length
	 * (dst_key_len + fixed_len - src_key_len).  For that result to
	 * equal fixed_len (the only key length the destination group
	 * accepts), src_key_len and dst_key_len must be equal.
	 */
	if (dst_ft->group->key_len != CDS_FT_LEN_VARIABLE
			&& dst_key_len != src_key_len) {
		FT_TP(merge_exit, (int) CDS_FT_STATUS_INVALID_ARGUMENT_ERROR);
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}

	dst_ft->group->flavor->read_lock();

	/*
	 * Step 1: detach @src_ft at @src_key into a transient @subtree.
	 *
	 * cds_ft_detach drains in-flight RCU readers of the moved
	 * sub-tree (one grace period unless @src_ft is exclusive), so
	 * @subtree is returned in exclusive mode.  This is what frees
	 * @src_ft from a "must be exclusive" requirement: the rest of
	 * the merge operates entirely on @subtree (which is provably
	 * exclusive) instead of touching @src_ft's payload.
	 *
	 * @subtree's keys are stripped of the @src_key prefix, so on a
	 * fixed-length group @subtree's per-key length is shorter than
	 * the group's nominal fixed length.  This handle does NOT
	 * conform to the group's public-API key-length invariant; we
	 * keep it strictly internal and only touch it via the
	 * keylen-bypassing helpers (ft_keys_lcp, ft_detach_keylen,
	 * ft_graft_keylen) and cds_ft_destroy.
	 *
	 * NOT_FOUND from the detach means @src_ft has no content under
	 * @src_key: the merge is a no-op.
	 */
	status = ft_detach_keylen(src_ft, src_key, src_key_len, &subtree);
	if (status == CDS_FT_STATUS_NOT_FOUND) {
		dst_ft->group->flavor->read_unlock();
		FT_TP(merge_exit, (int) CDS_FT_STATUS_OK);
		return CDS_FT_STATUS_OK;
	}
	if (status < 0)
		goto out_unlock;

	/*
	 * Step 2: pick the fast path or per-entry fallback based on
	 * whether @dst_ft has any content under @dst_key.
	 *
	 * Fast path: ft_graft_keylen(@dst_ft, @dst_key, @subtree)
	 * re-prepends @dst_key to each of @subtree's stripped keys,
	 * placing the original keys (or re-keyed ones for cross-key
	 * merges) in @dst_ft.
	 *
	 * The keylen-bypassing helpers skip the public API's
	 * fixed-length-vs-non-root rejection, so the fast path
	 * applies uniformly to variable-length and fixed-length
	 * groups.
	 *
	 * (A future optimization, not implemented here: when @dst_ft
	 * has content under @dst_key but not under @dst_key
	 * concatenated with @subtree's LCP, a nested detach+graft at
	 * the deeper attach point would let the fast path apply more
	 * often.  The current implementation conservatively falls
	 * back to per-entry in that case because ft_store_at_graft_point
	 * wraps an external-nodes-only graft payload in a fresh
	 * internal node, which leaves an invariant-breaking empty
	 * internal in @dst_ft after the chain is later removed.)
	 */
	if (cds_ft_count_keys_prefix(dst_ft, dst_key, dst_key_len) == 0) {
		status = ft_graft_keylen(dst_ft, dst_key, dst_key_len,
				subtree);
	} else {
		struct cds_ft_iter *iter = NULL;

		status = cds_ft_iter_create(subtree, &iter);
		if (status != CDS_FT_STATUS_OK)
			goto out_rollback;
		while (cds_ft_lookup_first(subtree, iter)
				== CDS_FT_STATUS_OK) {
			uint8_t sub_key[FT_MAX_KEY_LEN];
			uint8_t dst_full[FT_MAX_KEY_LEN];
			size_t sub_key_len = 0;
			struct cds_ft_node *head, *tmp;

			status = cds_ft_iter_get_key(iter, sub_key,
					sizeof(sub_key), &sub_key_len);
			if (status != CDS_FT_STATUS_OK)
				break;
			if (dst_key_len + sub_key_len > sizeof(dst_full)) {
				status = CDS_FT_STATUS_OVERFLOW_ERROR;
				break;
			}
			memcpy(dst_full, dst_key, dst_key_len);
			memcpy(dst_full + dst_key_len, sub_key, sub_key_len);

			status = cds_ft_remove_all(subtree, iter, &head);
			if (status != CDS_FT_STATUS_OK)
				break;
			cds_ft_for_each_duplicate_safe_rcu(head, tmp) {
				cds_ft_node_init(head);
				status = cds_ft_insert(dst_ft, dst_full,
						dst_key_len + sub_key_len,
						head);
				if (status != CDS_FT_STATUS_OK)
					break;
			}
			if (status != CDS_FT_STATUS_OK)
				break;
		}
		cds_ft_iter_destroy(iter);
	}

	if (status != CDS_FT_STATUS_OK)
		goto out_rollback;

	cds_ft_destroy(subtree);
	dst_ft->group->flavor->read_unlock();
	FT_TP(merge_exit, (int) CDS_FT_STATUS_OK);
	return CDS_FT_STATUS_OK;

out_rollback:
	/*
	 * Best-effort rollback: re-graft @subtree (with whatever
	 * content remains in it) back into @src_ft at @src_key.
	 * @src_ft was emptied of all content under @src_key by the
	 * initial detach, so this graft cannot collide and only fails
	 * on memory exhaustion.  On rollback failure @subtree's
	 * externals are leaked (cds_ft_destroy releases the wrapper
	 * but cannot drain externals).  The original error is
	 * propagated.
	 */
	(void) ft_graft_keylen(src_ft, src_key, src_key_len, subtree);
	cds_ft_destroy(subtree);
out_unlock:
	dst_ft->group->flavor->read_unlock();
	FT_TP(merge_exit, (int) status);
	return status;
}

enum cds_ft_status cds_ft_merge(struct cds_ft *dst_ft,
		const uint8_t *key, size_t key_len,
		struct cds_ft *src_ft)
{
	return cds_ft_merge_at(dst_ft, key, key_len, src_ft, key, key_len);
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
	struct cds_ft_metadata *rmeta;

	CDS_FT_SCOPED_READER(ft);
	CDS_FT_ASSERT_RCU_READ_LOCKED(ft);

	root_flag = rcu_dereference(ft->root);
	root_node = ft_node_ptr(root_flag);

	/*
	 * The root is always an internal node (invariant enforced by
	 * ft_make_root_internal at every site that publishes ft->root),
	 * so no tag dispatch is needed before reading metadata.
	 */
	rmeta = cds_ft_item_to_metadata(root_node);

	/*
	 * Empty trie: the root has no children and no NIL-key entries.
	 * For popcount root, nr_child is derived from the bitmap and a
	 * freshly-allocated (calloc'd) root with bitmap == 0 correctly
	 * reports nr_child == 0.
	 */
	if (rmeta->nr_child != 0)
		return false;
	return !uatomic_load(&rmeta->external_nodes, CMM_RELAXED);
}

/*
 * Handle compressed node in cds_ft_count_keys_prefix().
 *
 * Returns FT_DESCENT_CONTINUE to advance past the compressed path,
 * or FT_DESCENT_END when the prefix is fully consumed inside the
 * compressed node (count written to *count_ret).
 */
static
enum ft_descent_action ft_count_prefix_compressed(
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
		return FT_DESCENT_END;
	}
	if (cn->len >= remaining) {
		struct cds_ft_metadata *cn_meta =
			cds_ft_item_to_metadata(
				(struct cds_ft_inode *) cn);

		*count_ret = ft_nr_keys_load(cn_meta);
		return FT_DESCENT_END;
	}
	*i_p = i + cn->len - 1;
	*node_flag_p = ft_dereference_acquire_prefetch(cn->child);
	return FT_DESCENT_CONTINUE;
}

unsigned long cds_ft_count_keys_prefix(struct cds_ft *ft,
		const uint8_t *_prefix, size_t prefix_len)
{
	struct cds_ft_inode_flag *node_flag;
	unsigned int i;
	const struct cds_ft_key_map *km = &ft->group->key_map;
	uint8_t ordinal_buf[FT_MAX_KEY_LEN];
	const uint8_t *prefix;
	unsigned long count;

	CDS_FT_SCOPED_READER(ft);
	CDS_FT_ASSERT_RCU_READ_LOCKED(ft);

	if (prefix_len > ft->group->max_key_len) {
		count = 0;
		goto out;
	}
	if (caa_likely(km->identity)) {
		prefix = _prefix;
	} else {
		ft_key_to_ordinals(ordinal_buf, _prefix, prefix_len, km);
		prefix = ordinal_buf;
	}

	node_flag = ft_dereference_acquire_prefetch(ft->root);

	for (i = 0; i < prefix_len; i++) {
		uint8_t kv;

		if (ft_node_external(node_flag)) {
			count = 0;
			goto out;
		}
		if (ft_node_compressed(node_flag)) {
			enum ft_descent_action act;

			act = ft_count_prefix_compressed(
				&node_flag, &i, prefix,
				prefix_len, &count);
			if (act == FT_DESCENT_END)
				goto out;
			continue;
		}
		kv = prefix[i];
		node_flag = ft_node_get_nth(node_flag, NULL, kv, FT_PF_NONE);
	}

	if (!node_flag) {
		count = 0;
		goto out;
	}
	if (ft_node_internal(node_flag)) {
		struct cds_ft_metadata *metadata =
			cds_ft_item_to_metadata(ft_node_ptr(node_flag));
		count = ft_nr_keys_load(metadata);
		goto out;
	}
	if (ft_node_compressed(node_flag)) {
		struct cds_ft_metadata *cn_meta =
			cds_ft_item_to_metadata(ft_node_ptr(node_flag));
		count = ft_nr_keys_load(cn_meta);
		goto out;
	}
	/* External node: one key (possibly with duplicates). */
	count = 1;
out:
	FT_TP(count_prefix, count);
	return count;
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
		return ft_nr_keys_load(m);
	}
	return 1;
}

/*
 * Handle compressed node in cds_ft_lookup_nth().
 *
 * Fills the compressed path into ordinal_key and iter_path, advances
 * level and node_flag past the compressed segment.
 *
 * Returns FT_DESCENT_CONTINUE on success, FT_DESCENT_BREAK if
 * the child pointer is NULL.
 */
static
enum ft_descent_action ft_lookup_nth_compressed(
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
	if (!node_flag) {
		*node_flag_p = node_flag;
		*level_p = level;
		return FT_DESCENT_BREAK;
	}
	iter_path_node(iter)[level] = node_flag;
	if (ft_node_external(node_flag)) {
		*node_flag_p = node_flag;
		*level_p = level;
		return FT_DESCENT_BREAK;
	}
	*node_flag_p = node_flag;
	*level_p = level;
	return FT_DESCENT_CONTINUE;
}

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

	CDS_FT_SCOPED_READER(ft);
	FT_TP(lookup_nth_enter, n);

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
			enum ft_descent_action act;

			act = ft_lookup_nth_compressed(&node_flag,
				&level, ordinal_key, iter);
			if (act == FT_DESCENT_BREAK)
				break;
			continue;
		}
		/* Iterate children in ascending ordinal order. */
		pivot = -1;
		child = ft_node_get_direction(node_flag, pivot, &child_key, FT_RIGHT, true);
		while (child) {
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
			child = ft_node_get_direction(node_flag, pivot, &child_key, FT_RIGHT, true);
		}

		/* Exhausted all children without finding. */
		break;

next_level:
		;
	}

	/* Reached a leaf (external node). */
	if (node_flag && ft_node_external(node_flag) && remaining == 0) {
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
	FT_TP(lookup_nth_exit, (int) iter->status);
	return iter->status;
}

/*
 * Handle compressed node in cds_ft_lookup_nth_last().
 *
 * In reverse order, process the child subtree first (larger keys),
 * then fall through to external_nodes (smallest = last).
 *
 * Returns FT_DESCENT_CONTINUE when descending into the child,
 * FT_DESCENT_BREAK when the child is NULL or external (leaf),
 * or FT_DESCENT_END to signal the caller to fall through to
 * check_ext_nth_last (remaining updated, child keys exhausted).
 */
static
enum ft_descent_action ft_lookup_nth_last_compressed(
		struct cds_ft_inode_flag **node_flag_p,
		int *level_p, unsigned long *remaining_p,
		uint8_t *ordinal_key, struct cds_ft_iter *iter)
{
	struct cds_ft_inode_flag *node_flag = *node_flag_p;
	int level = *level_p;
	struct cds_ft_compressed_node *cn =
		ft_compressed_node_ptr(node_flag);

	if (cn->child) {
		unsigned long child_keys =
			ft_child_key_count(cn->child);

		if (*remaining_p < child_keys) {
			ft_fill_compressed_path(cn,
				ordinal_key, level - 1,
				iter_path_node(iter), level,
				node_flag);
			level += cn->len - 1;
			node_flag = ft_dereference_acquire_prefetch(cn->child);
			if (!node_flag) {
				*node_flag_p = node_flag;
				*level_p = level;
				return FT_DESCENT_BREAK;
			}
			iter_path_node(iter)[level] = node_flag;
			if (ft_node_external(node_flag)) {
				*node_flag_p = node_flag;
				*level_p = level;
				return FT_DESCENT_BREAK;
			}
			*node_flag_p = node_flag;
			*level_p = level;
			return FT_DESCENT_CONTINUE;
		}
		*remaining_p -= child_keys;
	}
	return FT_DESCENT_END;
}

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

	CDS_FT_SCOPED_READER(ft);
	FT_TP(lookup_nth_last_enter, n);

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
			enum ft_descent_action act;

			act = ft_lookup_nth_last_compressed(
				&node_flag, &level, &remaining,
				ordinal_key, iter);
			if (act == FT_DESCENT_BREAK)
				break;
			if (act == FT_DESCENT_END)
				goto check_ext_nth_last;
			continue;
		}
		/* Iterate children in descending ordinal order first. */
		pivot = FT_ENTRY_PER_NODE;
		child = ft_node_get_direction(node_flag, pivot, &child_key, FT_LEFT, true);
		while (child) {
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
			child = ft_node_get_direction(node_flag, pivot, &child_key, FT_LEFT, true);
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
	if (node_flag && ft_node_external(node_flag) && remaining == 0) {
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
	FT_TP(lookup_nth_last_exit, (int) iter->status);
	return iter->status;
}

/*
 * Match a compressed node's path bytes against @key starting at @i
 * and step past it.  On success, fills ordinal_key + iter_path
 * across the compressed span, advances *i_p so the caller's
 * for-loop increment lands on the next byte after the span,
 * updates *node_flag_p to cn->child, and returns
 * FT_DESCENT_CONTINUE.  On any mismatch (key shorter than the
 * compressed path, byte-mismatch, or NULL child) returns
 * FT_DESCENT_END.
 */
static inline_lookup
enum ft_descent_action ft_rebuild_path_compressed(
		struct cds_ft_inode_flag **node_flag_p,
		unsigned int *i_p,
		const uint8_t *key, size_t key_len,
		uint8_t *ordinal_key,
		struct cds_ft_iter *iter)
{
	struct cds_ft_inode_flag *node_flag = *node_flag_p;
	struct cds_ft_compressed_node *cn = ft_compressed_node_ptr(node_flag);
	unsigned int i = *i_p;
	unsigned int j;

	if (i + cn->len > key_len)
		return FT_DESCENT_END;
	for (j = 0; j < cn->len; j++) {
		uint8_t ord = key[i + j];

		if (ord != cn->key_bytes[j])
			return FT_DESCENT_END;
		ordinal_key[i + j] = ord;
		iter_path_node(iter)[i + j + 1] = node_flag;
	}
	i += cn->len - 1;
	node_flag = ft_dereference_acquire_prefetch(cn->child);
	if (!node_flag)
		return FT_DESCENT_END;
	iter_path_node(iter)[i + 1] = node_flag;
	*node_flag_p = node_flag;
	*i_p = i;
	return FT_DESCENT_CONTINUE;
}

/*
 * Re-descend from the root following @key to rebuild the iterator path
 * and ordinal_key arrays.  Returns the depth reached, or -1 on error.
 */
static inline_lookup
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
			enum ft_descent_action act;

			act = ft_rebuild_path_compressed(&node_flag, &i,
				key, key_len, ordinal_key, iter);
			if (act == FT_DESCENT_END)
				return -1;
			assert(act == FT_DESCENT_CONTINUE);
			continue;
		}
		ordinal = key[i];
		ordinal_key[i] = ordinal;
		node_flag = ft_node_get_nth(node_flag, NULL, ordinal, FT_PF_NONE);
		if (!node_flag)
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
 * Returns FT_DESCENT_CONTINUE on success, FT_DESCENT_BREAK if
 * the child pointer is NULL or is an external (leaf) node.
 */
static inline_lookup
enum ft_descent_action ft_skip_forward_compressed(
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
	if (!node_flag) {
		*node_flag_p = node_flag;
		*level_p = level;
		return FT_DESCENT_BREAK;
	}
	iter_path_node(iter)[level] = node_flag;
	if (ft_node_external(node_flag)) {
		*node_flag_p = node_flag;
		*level_p = level;
		return FT_DESCENT_BREAK;
	}
	*node_flag_p = node_flag;
	*level_p = level;
	return FT_DESCENT_CONTINUE;
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

	CDS_FT_SCOPED_READER(ft);
	FT_TP(iter_skip_forward_enter, n);

	CDS_FT_ASSERT_RCU_READ_LOCKED(ft);

	if (!iter->node) {
		FT_TP(iter_skip_forward_exit, (int) CDS_FT_STATUS_NOT_FOUND);
		return CDS_FT_STATUS_NOT_FOUND;
	}
	if (n == 0) {
		FT_TP(iter_skip_forward_exit, (int) CDS_FT_STATUS_OK);
		return CDS_FT_STATUS_OK;
	}

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
		unsigned long right_keys = ft_nr_keys_load(pmeta) - 1; /* exclude self */

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

				if (!cn->child)
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
			} else {
				struct cds_ft_inode_flag *child;
				uint8_t child_key = 0;
				int pivot = -1;

				child = ft_node_get_direction(parent, pivot,
						&child_key, FT_RIGHT, true);
				while (child) {
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
							&child_key, FT_RIGHT, true);
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

		pivot = ordinal_key[level];
		child = ft_node_get_direction(ancestor, pivot,
				&child_key, FT_RIGHT, true);
		while (child) {
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
					&child_key, FT_RIGHT, true);
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
				enum ft_descent_action act;

				act = ft_skip_forward_compressed(
					&node_flag, &level,
					ordinal_key, iter);
				if (act == FT_DESCENT_BREAK)
					break;
				continue;
			}
			pivot = -1;
			child = ft_node_get_direction(node_flag, pivot,
					&child_key, FT_RIGHT, true);
			while (child) {
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
						&child_key, FT_RIGHT, true);
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
	FT_TP(iter_skip_forward_exit, (int) iter->status);
	return iter->status;
}

/*
 * Handle compressed node in cds_ft_iter_skip_reverse's descend_reverse
 * loop.
 *
 * In reverse, process the child subtree first (larger keys), then
 * fall through to external_nodes (smallest = last).
 *
 * Returns FT_DESCENT_CONTINUE when descending into the child,
 * FT_DESCENT_BREAK when the child is NULL or external (leaf),
 * or FT_DESCENT_END to signal the caller to fall through to
 * check_ext_descend_reverse (remaining updated, child keys exhausted).
 */
static inline_lookup
enum ft_descent_action ft_skip_reverse_compressed(
		struct cds_ft_inode_flag **node_flag_p,
		int *level_p, unsigned long *remaining_p,
		uint8_t *ordinal_key, struct cds_ft_iter *iter)
{
	struct cds_ft_inode_flag *node_flag = *node_flag_p;
	int level = *level_p;
	struct cds_ft_compressed_node *cn =
		ft_compressed_node_ptr(node_flag);

	if (cn->child) {
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
			if (!node_flag) {
				*node_flag_p = node_flag;
				*level_p = level;
				return FT_DESCENT_BREAK;
			}
			iter_path_node(iter)[level] = node_flag;
			if (ft_node_external(node_flag)) {
				*node_flag_p = node_flag;
				*level_p = level;
				return FT_DESCENT_BREAK;
			}
			*node_flag_p = node_flag;
			*level_p = level;
			return FT_DESCENT_CONTINUE;
		}
		*remaining_p -= ck;
	}
	return FT_DESCENT_END;
}

/*
 * Handle a compressed ancestor in cds_ft_iter_skip_reverse's walk-up
 * loop.  Skips intermediate path levels (same compressed node at
 * adjacent levels).  At the entry level, the only candidate is the
 * compressed node's external_nodes (which sort before all children
 * — i.e. leftward of the current key); count or claim it.
 *
 * Returns FT_DESCENT_END when the external_nodes match: the iter
 * has been written and *iter_status_p set to OK.  Otherwise returns
 * FT_DESCENT_CONTINUE for the caller to keep walking up.
 */
static inline_lookup
enum ft_descent_action ft_skip_reverse_walk_up_compressed(
		struct cds_ft_inode_flag *ancestor,
		int level,
		unsigned long *remaining_p,
		uint8_t *ordinal_key,
		struct cds_ft_iter *iter)
{
	struct cds_ft_metadata *ameta;
	struct cds_ft_node *a_ext;

	/* Skip intermediate compressed path levels. */
	if (level > 0 && iter_path_node(iter)[level - 1] == ancestor)
		return FT_DESCENT_CONTINUE;

	ameta = cds_ft_item_to_metadata(ft_node_ptr(ancestor));
	a_ext = ft_dereference_acquire(ameta->external_nodes);
	if (a_ext) {
		if (*remaining_p == 1) {
			int j;

			iter->key_len = level;
			for (j = 0; j < level; j++)
				iter_key(iter)[j] = ordinal_key[j];
			iter->node = a_ext;
			iter->path_valid = true;
			iter_debug_path_update(iter);
			iter->path_len = level + 1;
			iter->status = CDS_FT_STATUS_OK;
			return FT_DESCENT_END;
		}
		(*remaining_p)--;
	}
	return FT_DESCENT_CONTINUE;
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

	CDS_FT_SCOPED_READER(ft);
	FT_TP(iter_skip_reverse_enter, n);

	CDS_FT_ASSERT_RCU_READ_LOCKED(ft);

	if (!iter->node) {
		FT_TP(iter_skip_reverse_exit, (int) CDS_FT_STATUS_NOT_FOUND);
		return CDS_FT_STATUS_NOT_FOUND;
	}
	if (n == 0) {
		FT_TP(iter_skip_reverse_exit, (int) CDS_FT_STATUS_OK);
		return CDS_FT_STATUS_OK;
	}

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
			enum ft_descent_action act;

			act = ft_skip_reverse_walk_up_compressed(ancestor,
				level, &remaining, ordinal_key, iter);
			if (act == FT_DESCENT_END)
				goto end;
			assert(act == FT_DESCENT_CONTINUE);
			continue;
		}

		ameta = cds_ft_item_to_metadata(ft_node_ptr(ancestor));

		/* Count leftward siblings. */
		pivot = ordinal_key[level];
		child = ft_node_get_direction(ancestor, pivot,
				&child_key, FT_LEFT, true);
		while (child) {
			left_keys += ft_child_key_count(child);
			pivot = child_key;
			child = ft_node_get_direction(ancestor, pivot,
					&child_key, FT_LEFT, true);
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
						&child_key, FT_LEFT, true);
				while (child) {
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
							&child_key, FT_LEFT, true);
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
				enum ft_descent_action act;

				act = ft_skip_reverse_compressed(
					&node_flag, &level, &remaining,
					ordinal_key, iter);
				if (act == FT_DESCENT_BREAK)
					break;
				if (act == FT_DESCENT_END)
					goto check_ext_descend_reverse;
				continue;
			}
			pivot = FT_ENTRY_PER_NODE;
			child = ft_node_get_direction(node_flag, pivot,
					&child_key, FT_LEFT, true);
			while (child) {
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
						&child_key, FT_LEFT, true);
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
	FT_TP(iter_skip_reverse_exit, (int) iter->status);
	return iter->status;
}

unsigned long cds_ft_count_entries(struct cds_ft *ft)
{
	struct cds_ft_iter *iter;
	enum cds_ft_status status;
	unsigned long count = 0;

	CDS_FT_SCOPED_READER(ft);
	CDS_FT_ASSERT_RCU_READ_LOCKED(ft);

	status = cds_ft_iter_create(ft, &iter);
	if (status != CDS_FT_STATUS_OK) {
		FT_TP(count_entries, (unsigned long) 0);
		return 0;
	}
	cds_ft_for_each_rcu(ft, iter) {
		struct cds_ft_node *node = cds_ft_iter_node(iter);

		cds_ft_for_each_duplicate_rcu(node)
			count++;
	}
	if (cds_ft_iter_status(iter) < 0)
		count = 0;
	cds_ft_iter_destroy(iter);
	FT_TP(count_entries, count);
	return count;
}

enum cds_ft_status cds_ft_group_attr_create(struct cds_ft_group_attr **result)
{
	struct cds_ft_group_attr *attr = calloc(1, sizeof(struct cds_ft_group_attr));

	if (!attr) {
		*result = NULL;
		return CDS_FT_STATUS_MEMORY_ERROR;
	}
	attr->key_len = CDS_FT_LEN_DEFAULT;
	attr->max_key_len = FT_MAX_KEY_LEN;
	attr->key_map.identity = true;
	/*
	 * Default lookup optimization is EAGER (preserves historical
	 * behavior).  Callers tuning for cds_ft_lookup_candidate_key or
	 * cds_ft_speculative_lookup_key should call
	 * cds_ft_group_attr_set_lookup_optimization(attr,
	 * CDS_FT_LOOKUP_OPTIMIZE_SPECULATIVE) explicitly.
	 *
	 * Flipping the default to SPECULATIVE is a future follow-up
	 * (gated on resolving a latent issue with concurrent iteration
	 * under SKIP_COMPRESSED — inv_iteration_order hangs on
	 * speculative tries).
	 */
	*result = attr;
	return CDS_FT_STATUS_OK;
}

void cds_ft_group_attr_destroy(struct cds_ft_group_attr *attr)
{
	free(attr);
}

enum cds_ft_status cds_ft_group_attr_set_key_len(struct cds_ft_group_attr *attr, size_t key_len)
{
	attr->key_len = key_len;
	return CDS_FT_STATUS_OK;
}

enum cds_ft_status cds_ft_group_attr_set_max_key_len(struct cds_ft_group_attr *attr, size_t max_key_len)
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

enum cds_ft_status cds_ft_group_attr_set_key_map(struct cds_ft_group_attr *attr,
		const uint8_t *key_to_ordinal, const uint8_t *ordinal_to_key)
{
	attr->key_map.identity = false;
	memcpy(attr->key_map.key_to_ordinal, key_to_ordinal, sizeof(attr->key_map.key_to_ordinal));
	memcpy(attr->key_map.ordinal_to_key, ordinal_to_key, sizeof(attr->key_map.ordinal_to_key));
	return CDS_FT_STATUS_OK;
}

/*
 * Validate that the pointer bits used by the skip-compressed encoding
 * are outside the kernel's virtual address range.  Attempt to mmap a
 * page at the encoding boundary; if the mapping succeeds or fails
 * with EEXIST the bit is within the VA range and skip-compressed
 * cannot be used safely.  Only ENOMEM (address beyond TASK_SIZE)
 * confirms the bit is available; any other failure (EPERM, EINVAL,
 * EAGAIN, seccomp, ...) is treated conservatively as unavailable.
 *
 * Called once from cds_ft_group_attr_set_flags when CDS_FT_FLAG_SKIP_COMPRESSED
 * is requested.
 */
#ifdef FEATURE_FT_SKIP_COMPRESSED
static
bool ft_skip_compressed_validate(void)
{
	void *p;

	p = mmap((void *)(1UL << FT_SKIP_LEN_SHIFT), urcu_get_page_len(),
		 PROT_NONE, MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE,
		 -1, 0);
	if (p == MAP_FAILED) {
		/* Only ENOMEM proves the address is outside the VA range. */
		return errno == ENOMEM;
	}
	/* Mapping succeeded: the bit is within the VA range. */
	munmap(p, urcu_get_page_len());
	return false;
}
#endif

enum cds_ft_status cds_ft_group_attr_set_lookup_optimization(
		struct cds_ft_group_attr *attr,
		enum cds_ft_lookup_optimization opt)
{
	switch (opt) {
	case CDS_FT_LOOKUP_OPTIMIZE_EAGER:
		attr->speculative = false;
		attr->flags &= ~CDS_FT_FLAG_SKIP_COMPRESSED;
		return CDS_FT_STATUS_OK;
	case CDS_FT_LOOKUP_OPTIMIZE_SPECULATIVE:
		/*
		 * Speculative descent always works.  Opportunistically
		 * enable the skip-compressed pointer encoding when the
		 * arch supports it; on archs without it the descent still
		 * skips per-step byte compares — just without the extra
		 * compressed-CL bypass.
		 */
		attr->speculative = true;
#ifdef FEATURE_FT_SKIP_COMPRESSED
		if (ft_skip_compressed_validate())
			attr->flags |= CDS_FT_FLAG_SKIP_COMPRESSED;
#endif
		return CDS_FT_STATUS_OK;
	}
	return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
}

enum cds_ft_status cds_ft_attr_create(struct cds_ft_attr **result)
{
	struct cds_ft_attr *attr = calloc(1, sizeof(struct cds_ft_attr));

	if (!attr) {
		*result = NULL;
		return CDS_FT_STATUS_MEMORY_ERROR;
	}
	/*
	 * Leave threshold and multipliers at 0 (calloc'd) as a "use
	 * library default" sentinel; cds_ft_create resolves them to
	 * mode-aware defaults that depend on the group's skip-compressed
	 * setting.  Fields explicitly set via the attr setters take
	 * precedence (the setters validate >= 100 or DISABLED, so 0
	 * cannot leak in by accident).
	 */
	*result = attr;
	return CDS_FT_STATUS_OK;
}

void cds_ft_attr_destroy(struct cds_ft_attr *attr)
{
	free(attr);
}

enum cds_ft_status cds_ft_attr_set_exclusive(struct cds_ft_attr *attr,
		bool exclusive)
{
	attr->exclusive = exclusive;
	return CDS_FT_STATUS_OK;
}

void cds_ft_make_exclusive(struct cds_ft *ft)
{
	CDS_FT_SCOPED_WRITER(ft);
	if (ft->exclusive)
		return;
	ft->group->flavor->update_synchronize_rcu();
	ft->exclusive = true;
}

void cds_ft_make_concurrent(struct cds_ft *ft)
{
	CDS_FT_SCOPED_WRITER(ft);
	ft->exclusive = false;
}

bool cds_ft_is_exclusive(const struct cds_ft *ft)
{
	return ft->exclusive;
}

bool cds_ft_excl_validate_enabled(void)
{
#ifdef FEATURE_FT_EXCL_VALIDATE
	return true;
#else
	return false;
#endif
}

bool cds_ft_verify_at_mutation_enabled(void)
{
#ifdef FEATURE_FT_VERIFY_AT_MUTATION
	return true;
#else
	return false;
#endif
}

/*
 * Set the verify-at-mutation sampling period for @ft.  When the
 * library is built with -DFEATURE_FT_VERIFY_AT_MUTATION, the writer
 * scope-exit hook runs cds_ft_verify once every @period mutations.
 *
 *   period == 0 : disable the verify walk on this trie (the
 *                 increment-and-compare still runs in the hook).
 *   period == 1 : verify at every mutation (the historical
 *                 -DFEATURE_FT_VERIFY_AT_MUTATION cadence).
 *   period >  1 : verify every @period mutations — useful on large
 *                 tries where O(N) per mutation is impractical.
 *
 * The counter is reset to 0 on each period boundary, so it never
 * exceeds @period - 1 and there is no overflow / cadence-drift
 * concern on long-running workloads.
 *
 * Returns CDS_FT_STATUS_OK on success, or
 * CDS_FT_STATUS_NOT_SUPPORTED if the library was built without
 * FEATURE_FT_VERIFY_AT_MUTATION — the call surfaces the mismatch
 * loudly rather than silently doing nothing, so a test that relies
 * on the verify cadence cannot accidentally run with verify-at-
 * mutation compiled out.  Use cds_ft_verify_at_mutation_enabled()
 * to gate the call.
 *
 * Write-side only (mutex-held); not safe to call concurrently
 * with writers on the same trie.
 */
enum cds_ft_status cds_ft_verify_at_mutation_period_set(struct cds_ft *ft,
		unsigned long period)
{
#ifdef FEATURE_FT_VERIFY_AT_MUTATION
	ft->verify_at_mutation_period = period;
	ft->verify_at_mutation_counter = 0;
	return CDS_FT_STATUS_OK;
#else
	(void) ft;
	(void) period;
	return CDS_FT_STATUS_NOT_SUPPORTED;
#endif
}

/*
 * Read the verify-at-mutation sampling period for @ft into
 * *@period.
 *
 * Returns CDS_FT_STATUS_OK on success, or
 * CDS_FT_STATUS_NOT_SUPPORTED if the library was built without
 * FEATURE_FT_VERIFY_AT_MUTATION — distinguishing the
 * build-disabled case from a runtime period == 0.
 */
enum cds_ft_status cds_ft_verify_at_mutation_period_get(
		const struct cds_ft *ft, unsigned long *period)
{
#ifdef FEATURE_FT_VERIFY_AT_MUTATION
	*period = ft->verify_at_mutation_period;
	return CDS_FT_STATUS_OK;
#else
	(void) ft;
	(void) period;
	return CDS_FT_STATUS_NOT_SUPPORTED;
#endif
}

enum cds_ft_status _cds_ft_group_create(const struct cds_ft_group_attr *attr,
		struct cds_ft_group **result_ft_group,
		const struct rcu_flavor_struct *flavor)
{
	struct cds_ft_group *ft_group;
	size_t key_len = CDS_FT_LEN_DEFAULT,
	       max_key_len = FT_MAX_KEY_LEN;

	ft_specialized_scan_layout_assert();
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
	pthread_mutex_init(&ft_group->arena_lock, NULL);
	if (attr) {
		ft_group->key_map = attr->key_map;
		ft_group->flags = attr->flags;
		ft_group->speculative = attr->speculative;
	} else {
		ft_group->key_map.identity = true;
	}
	*result_ft_group = ft_group;
	FT_TP(group_create, (const void *) ft_group);
	return CDS_FT_STATUS_OK;
}

enum cds_ft_status cds_ft_group_destroy(struct cds_ft_group *ft_group)
{
	if (uatomic_load(&ft_group->nr_ft_instances, CMM_RELAXED) != 0)
		return CDS_FT_STATUS_BUSY_ERROR;
	FT_TP(group_destroy, (const void *) ft_group);
	cds_ft_free_all_arenas(ft_group);
	pthread_mutex_destroy(&ft_group->arena_lock);
	free(ft_group);
	return CDS_FT_STATUS_OK;
}

enum cds_ft_status cds_ft_create(struct cds_ft_group *ft_group,
		const struct cds_ft_attr *attr,
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
	ft_install_lookup_ops(ft);
#ifdef FEATURE_FT_VERIFY_AT_MUTATION
	/*
	 * Default to verify-every-mutation cadence to preserve the
	 * historical -DFEATURE_FT_VERIFY_AT_MUTATION behavior; tests
	 * working with large tries can call
	 * cds_ft_set_verify_at_mutation_period() to dial it down.
	 * Counter is already zero from calloc.
	 */
	ft->verify_at_mutation_period = 1;
#endif
	if (attr)
		ft->exclusive = attr->exclusive;

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
	FT_TP(root_publish, (const void *) ft, (const void *) ft->root);


	uatomic_inc(&ft_group->nr_ft_instances, CMM_RELAXED);
	*result_ft = ft;
	FT_TP(ft_create, (const void *) ft, (const void *) ft_group);
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
	if (nr_fallback)
		print_debug_fallback_distribution(ft);

	if (na != nf) {
		fprintf(stderr, "[error] Fractal Trie leaked %ld nodes. Allocated: %lu, freed: %lu.\n",
			(long) na - nf, na, nf);
		fprintf(stderr, "  internal: alloc=%lu freed=%lu leaked=%ld\n",
			uatomic_read(&ft->nr_internal_alloc),
			uatomic_read(&ft->nr_internal_freed),
			(long)(uatomic_read(&ft->nr_internal_alloc) - uatomic_read(&ft->nr_internal_freed)));
		fprintf(stderr, "  compressed: alloc=%lu freed=%lu leaked=%ld\n",
			uatomic_read(&ft->nr_compressed_alloc),
			uatomic_read(&ft->nr_compressed_freed),
			(long)(uatomic_read(&ft->nr_compressed_alloc) - uatomic_read(&ft->nr_compressed_freed)));
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

	FT_TP(ft_destroy, (const void *) ft);
	/* Free root node. No concurrent readers at this point. */
	free_cds_ft_node(ft, ft_node_ptr(ft->root));
	/* Wait for in-flight call_rcu free to complete. */
	flavor->barrier();
	ft_final_checks(ft);
	uatomic_dec(&ft->group->nr_ft_instances, CMM_RELAXED);
	free(ft);
}

/*
 * Integrity verification.
 *
 * ft_verify_node_recursive: recursively verify structural invariants
 * starting at @node_flag (which may be internal or compressed).
 * Returns 0 on success, -1 on first detected error (with details
 * printed to @out).  Must be called with mutual exclusion wrt
 * updaters.
 *
 * Checks performed:
 * - nr_child matches the actual count of non-NULL child slots.
 * - nr_keys equals the sum of children's nr_keys plus the count
 *   of unique keys from external node chains attached to this node.
 * - Parent pointers of children point back to the correct parent.
 * - Compressed node invariants (len > 0, no external_nodes).
 *
 * @ft: the Fractal Trie (for group/flag access).
 * @out: file stream for diagnostic output (may be NULL to suppress).
 * @node_flag: tagged pointer to the node being verified.
 * @expected_parent: tagged pointer that the node's metadata->parent
 *                   should match (NULL for root).
 * @depth: current depth (used for diagnostics).
 * @out_nr_keys: output — total nr_keys in the subtree rooted here
 *               (written on success for parent aggregation).
 */
/*
 * Visited-pointer set for cds_ft_verify subtree-uniqueness check.
 *
 * Linear-probing open-addressing hash table keyed by node allocation
 * address (low tag bits stripped via ft_node_ptr).  Only used while a
 * single verify walk is in progress; the entire table is freed at the
 * end of cds_ft_verify.  Catches accidental sharing of a subtree
 * between two parents (a rebase/recompact bug class) and detects
 * parent-pointer cycles before the upward adjacency walk in
 * ft_verify_node_compressed gets a chance to loop forever.
 */
struct ft_visited_set {
	void **slots;		/* NULL = empty bucket. */
	size_t cap;		/* Power of two. */
	size_t mask;		/* cap - 1. */
	size_t count;
};

static
size_t ft_visited_hash(void *p)
{
	/*
	 * Drop the low alignment bits (arena items are at least 16-byte
	 * aligned, so the low 4 bits are zero), then mix with the 64-bit
	 * golden-ratio multiplier.
	 */
	uintptr_t v = (uintptr_t) p >> 4;
	return (size_t) (v * 11400714819323198485ULL);
}

static
int ft_visited_init(struct ft_visited_set *vs)
{
	vs->cap = 64;
	vs->mask = vs->cap - 1;
	vs->count = 0;
	vs->slots = calloc(vs->cap, sizeof(void *));
	return vs->slots ? 0 : -1;
}

static
void ft_visited_destroy(struct ft_visited_set *vs)
{
	free(vs->slots);
	vs->slots = NULL;
}

static
int ft_visited_grow(struct ft_visited_set *vs)
{
	size_t new_cap = vs->cap * 2;
	size_t new_mask = new_cap - 1;
	void **new_slots = calloc(new_cap, sizeof(void *));
	size_t i;

	if (!new_slots)
		return -1;
	for (i = 0; i < vs->cap; i++) {
		void *key = vs->slots[i];
		size_t j;

		if (!key)
			continue;
		j = ft_visited_hash(key) & new_mask;
		while (new_slots[j] != NULL)
			j = (j + 1) & new_mask;
		new_slots[j] = key;
	}
	free(vs->slots);
	vs->slots = new_slots;
	vs->cap = new_cap;
	vs->mask = new_mask;
	return 0;
}

/*
 * Returns 1 if @key was newly inserted, 0 if @key was already present
 * (duplicate visit), -1 on allocation failure.  NULL keys are not
 * tracked (they are filtered out by callers anyway).
 */
static
int ft_visited_add(struct ft_visited_set *vs, void *key)
{
	size_t i;

	if (key == NULL)
		return 1;
	/* Keep load factor below 0.5 for fast linear probing. */
	if ((vs->count + 1) * 2 > vs->cap) {
		if (ft_visited_grow(vs))
			return -1;
	}
	i = ft_visited_hash(key) & vs->mask;
	while (vs->slots[i] != NULL) {
		if (vs->slots[i] == key)
			return 0;
		i = (i + 1) & vs->mask;
	}
	vs->slots[i] = key;
	vs->count++;
	return 1;
}

/* Forward declaration so the per-kind helpers below can recurse. */
static
int ft_verify_node_recursive(const struct cds_ft *ft, FILE *out,
		struct ft_visited_set *visited,
		uint8_t *path,
		struct cds_ft_inode_flag *node_flag,
		struct cds_ft_inode_flag *expected_parent,
		unsigned int depth,
		unsigned long *out_nr_keys);

/*
 * Verify the doubly-linked external-node duplicate chain anchored at
 * @head, owned by @owner_flag (the flagged pointer to the
 * internal/compressed node, or its slot's parent).
 *
 *   - Head's prev must equal @owner_flag (the parent flagged-pointer
 *     convention used by ft_metadata_set_external_nodes and by the
 *     slot-attached external publish in cds_ft_insert).
 *   - Each non-head node's prev must point to its predecessor.
 *   - No node may appear twice (cycle / aliasing across chains).  We
 *     reuse @visited so a node accidentally referenced from a second
 *     chain elsewhere in the trie is also caught.
 *   - @path is currently always NULL.  Historical end-to-end
 *     path/key consistency relied on a group-known stored-key
 *     offset, which is no longer tracked at group level — callers
 *     now provide @key_offset per-call via
 *     cds_ft_speculative_lookup_key.
 *
 * Returns 0 on success, -1 on first violation.  No-op when @head is
 * NULL.
 */
static
int ft_verify_external_chain(const struct cds_ft *ft, FILE *out,
		struct ft_visited_set *visited,
		const uint8_t *path,
		struct cds_ft_inode_flag *owner_flag,
		struct cds_ft_node *head,
		unsigned int depth)
{
	struct cds_ft_node *node = head;
	struct cds_ft_node *prev = NULL;
	const struct cds_ft_group *group = ft->group;
	bool check_path = (path != NULL);

	/*
	 * Every external leaf reached at @depth represents a key of
	 * length @depth (NIL terminator at metadata depth, or full key
	 * at slot depth — both produce the same external chain).  That
	 * length must respect the group's max_key_len bound.  When the
	 * group is configured with CDS_FT_MAX_LEN_UNLIMITED
	 * (max_key_len == SIZE_MAX) this is a no-op since @depth is at
	 * most FT_MAX_KEY_LEN.
	 */
	if (head && (size_t) depth > group->max_key_len) {
		if (out)
			fprintf(out, "ft_verify: depth %u: external chain head %p exceeds group max_key_len %zu\n",
				depth, head, group->max_key_len);
		return -1;
	}
	while (node) {
		void *expected_prev = (prev == NULL) ?
			(void *) owner_flag : (void *) prev;
		int added = ft_visited_add(visited, node);

		if (added < 0) {
			if (out)
				fprintf(out, "ft_verify: depth %u: visited-set allocation failed in external chain at %p\n",
					depth, node);
			return -1;
		}
		if (added == 0) {
			if (out)
				fprintf(out, "ft_verify: depth %u: external chain node %p reached twice (cycle or alias)\n",
					depth, node);
			return -1;
		}
		if (node->prev != expected_prev) {
			if (out)
				fprintf(out, "ft_verify: depth %u: external chain node %p prev %p != expected %p (%s)\n",
					depth, node, node->prev,
					expected_prev,
					prev == NULL ?
						"head should point to owner" :
						"non-head should point to predecessor");
			return -1;
		}
		(void) check_path;
		(void) path;
		(void) group;
		prev = node;
		node = node->next;
	}
	return 0;
}

/*
 * If @slot_val is skip-encoded, verify the encoded slen equals
 * cn->len of the underlying compressed node.  Returns 0 when the
 * slot is not skip-encoded or the slen matches; -1 on mismatch
 * (with diagnostic to @out).  No-op on architectures without
 * FEATURE_FT_SKIP_COMPRESSED (ft_node_skip_compressed is constant
 * false and the body is dead-coded).
 *
 * Catches double-wrap of an already-skip-encoded child, stale skip
 * pointers left behind by a recompact that did not refresh the
 * encoded slen, and the parent_depth_span class of bug fixed by
 * ft_parent_depth_span match against skip-encoded child.
 */
static
int ft_verify_skip_encoding(FILE *out, struct cds_ft_inode_flag *slot_val,
		unsigned int depth)
{
	struct cds_ft_compressed_node *cn;
	unsigned int slen, cn_len;

	if (!ft_node_skip_compressed(slot_val))
		return 0;
	slen = ft_skip_len(slot_val);
	cn = ft_skip_to_compressed(slot_val);
	cn_len = cn->len;
	if (slen != cn_len) {
		if (out)
			fprintf(out, "ft_verify: depth %u: skip-encoded slot %p slen %u != cn->len %u (cn %p)\n",
				depth, slot_val, slen, cn_len, cn);
		return -1;
	}
	return 0;
}

/*
 * Verify a compressed node's invariants (cn->len >= 1, no
 * external_nodes, nr_child <= 1, parent pointer matches), recurse
 * into its child, and check the stored nr_keys against the child's
 * subtree count plus any local end-of-path key.
 */
static
int ft_verify_node_compressed(const struct cds_ft *ft, FILE *out,
		struct ft_visited_set *visited,
		uint8_t *path,
		struct cds_ft_inode_flag *node_flag,
		struct cds_ft_inode_flag *expected_parent,
		unsigned int depth,
		unsigned long *out_nr_keys)
{
	struct cds_ft_compressed_node *cn = ft_compressed_node_ptr(node_flag);
	struct cds_ft_metadata *cn_meta = cds_ft_item_to_metadata(
		(struct cds_ft_inode *) cn);
	struct cds_ft_node *external_nodes = cn_meta->external_nodes;
	unsigned long child_nr_keys = 0;
	unsigned long local_keys = 0;
	unsigned long stored_nr_keys;

	/* Compressed path must have length >= 1. */
	if (cn->len < 1) {
		if (out)
			fprintf(out, "ft_verify: depth %u: compressed node %p has len %u < 1\n",
				depth, node_flag, (unsigned int) cn->len);
		return -1;
	}
	/*
	 * key_bytes[] must fit in the allocated slot.  The arena
	 * allocation order encodes the slot size; subtract the fixed
	 * header (offsetof(..., key_bytes)) to get the capacity, then
	 * assert cn->len fits.  Catches a stale len byte after a
	 * size-class mismatch (e.g. a chain-compress that grew len
	 * without reallocating into a larger size class), which would
	 * otherwise silently overrun key_bytes[] on lookup.
	 */
	{
		size_t alloc_size = 1UL << cds_ft_item_order(cn);
		size_t header_size = offsetof(struct cds_ft_compressed_node,
				key_bytes);
		size_t key_bytes_capacity = alloc_size - header_size;

		if ((size_t) cn->len > key_bytes_capacity) {
			if (out)
				fprintf(out, "ft_verify: depth %u: compressed node %p len %u exceeds key_bytes capacity %zu (alloc size %zu)\n",
					depth, node_flag,
					(unsigned int) cn->len,
					key_bytes_capacity, alloc_size);
			return -1;
		}
	}
	/* Parent pointer check. */
	if (cn_meta->parent != expected_parent) {
		if (out)
			fprintf(out, "ft_verify: depth %u: compressed node %p parent mismatch: "
				"expected %p, got %p\n",
				depth, node_flag, expected_parent, cn_meta->parent);
		return -1;
	}
	/* nr_child must be 0 or 1. */
	if (cn_meta->nr_child > 1) {
		if (out)
			fprintf(out, "ft_verify: depth %u: compressed node %p nr_child %u > 1\n",
				depth, node_flag, cn_meta->nr_child);
		return -1;
	}
	/*
	 * cn->child / nr_child bookkeeping must agree:
	 *   nr_child == 1 implies cn->child is non-NULL (the one child);
	 *   nr_child == 0 implies cn->child is NULL.
	 * A drift between the two is a publish/clear bug that the
	 * subtree-key recursion would not catch on its own — the
	 * key-aggregation path simply skips a NULL cn->child and would
	 * accept a stored nr_child of 1 with cn->child = NULL as long
	 * as nr_keys also dropped to 0 in lockstep.
	 */
	if ((cn_meta->nr_child == 1) != (ft_node_ptr(cn->child) != NULL)) {
		if (out)
			fprintf(out, "ft_verify: depth %u: compressed node %p nr_child %u does not match cn->child %p presence\n",
				depth, node_flag,
				cn_meta->nr_child, cn->child);
		return -1;
	}
	/* Compressed nodes must not carry external_nodes. */
	if (external_nodes) {
		if (out)
			fprintf(out, "ft_verify: depth %u: compressed node %p has external_nodes %p (forbidden)\n",
				depth, node_flag, external_nodes);
		return -1;
	}
	/*
	 * Canonicalization: no two adjacent compressed nodes.  Check both
	 * directions of the adjacency at every visited compressed:
	 *
	 *  - ancestor-chain (works in both modes, including skip-compress):
	 *    walk metadata->parent upward; every consecutive compressed
	 *    ancestor is part of an adjacency run.  The chain walk is
	 *    necessary in skip-compress mode because a single skip pointer
	 *    can bypass two (or more) compresseds at once: the slot's
	 *    metadata-parent chase from the deepest underlying child only
	 *    surfaces the innermost compressed-being-skipped, so the
	 *    walker visits that one but not its compressed ancestor(s).
	 *    Walking the parent chain at the visited compressed re-exposes
	 *    every adjacency in the bypassed run.
	 *
	 *  - child-side (cheap and direct, primary in non-skip mode):
	 *    cn->child must not be a (raw or skip-encoded) compressed.  In
	 *    canonical post-fix tries cn->child is never skip-encoded; the
	 *    skip-compressed disjunct is defensive.
	 *
	 * Chain-compress is responsible for fusing adjacencies into a
	 * single compressed; a violation here means the canonicalization
	 * walk missed a case (and skip mode would silently double-wrap the
	 * grandparent's skip pointer).
	 */
	{
		struct cds_ft_inode_flag *child_in_chain = node_flag;
		struct cds_ft_inode_flag *anc = cn_meta->parent;
		bool adj_violation = false;

		while (anc && ft_node_compressed(anc)) {
			struct cds_ft_metadata *anc_meta =
				cds_ft_item_to_metadata(ft_node_ptr(anc));

			if (out)
				fprintf(out, "ft_verify: depth %u: compressed %p adjacent to compressed parent %p (no two adjacent compresseds)\n",
					depth, child_in_chain, anc);
			adj_violation = true;
			child_in_chain = anc;
			anc = anc_meta->parent;
		}
		if (adj_violation)
			return -1;
	}
	if (ft_node_skip_compressed(cn->child) ||
			(cn->child && ft_node_compressed(cn->child))) {
		if (out)
			fprintf(out, "ft_verify: depth %u: compressed node %p has %s child %p (no two adjacent compresseds)\n",
				depth, node_flag,
				ft_node_skip_compressed(cn->child) ?
					"skip-encoded compressed" : "compressed",
				cn->child);
		return -1;
	}
#ifdef FEATURE_FT_SKIP_COMPRESSED
	/*
	 * Skip-slot offset round-trip.  Each compressed records a
	 * pointer-stride offset from its parent to the slot that holds
	 * the (skip-encoded) pointer to itself; ft_publish_to_parent
	 * uses this to refresh the skip slot when cn->child is replaced.
	 * Verify the offset still resolves to a slot whose contents
	 * encode this very compressed (either skip-encoded or as a raw
	 * compressed flag — both are valid publish states).  A stale
	 * offset left after a recompact / graft would be the same bug
	 * family as the recent parent_depth_span and double-wrap fixes;
	 * surfacing it at the mutation that introduced it is much
	 * cheaper than chasing a corrupted skip pointer at lookup time.
	 *
	 * NULL slot means the offset was never set (offset == 0 with a
	 * non-NULL parent — e.g. compressed reached only via a raw
	 * compressed pointer that doesn't exercise the skip path).  No
	 * round-trip to verify in that case.  ft_get_skip_slot wants a
	 * non-const ft for the root case (parent == NULL); cast away
	 * const since we only read *slot.
	 */
	{
		struct cds_ft_inode_flag **skip_slot =
			ft_get_skip_slot(cn_meta, (struct cds_ft *) ft);

		if (skip_slot) {
			struct cds_ft_inode_flag *slot_val = *skip_slot;
			struct cds_ft_compressed_node *target_cn = NULL;

			if (ft_node_skip_compressed(slot_val))
				target_cn = ft_skip_to_compressed(slot_val);
			else if (slot_val &&
				 ft_node_compressed(slot_val))
				target_cn = ft_compressed_node_ptr(slot_val);
			if (target_cn != cn) {
				if (out)
					fprintf(out, "ft_verify: depth %u: compressed %p skip_slot_offset round-trip mismatch: slot %p holds %p (resolves to cn %p, expected %p)\n",
						depth, node_flag, skip_slot,
						slot_val, target_cn, cn);
				return -1;
			}
		}
	}
#endif
	/*
	 * Path tracking: write the compressed key bytes into the path
	 * buffer at positions [depth..depth+cn->len-1].  Subsequent
	 * recursion / external-chain compares read these bytes back
	 * against leaf-stored keys.
	 */
	if (path)
		memcpy(path + depth, cn->key_bytes, cn->len);
	/* Recurse into the child. */
	if (cn->child) {
		if (ft_node_external(cn->child)) {
			/* External leaf chain at end of compressed path. */
			if (ft_verify_external_chain(ft, out, visited, path,
					node_flag,
					(struct cds_ft_node *) ft_node_ptr(cn->child),
					depth + cn->len))
				return -1;
			local_keys = 1;	/* One unique key. */
		} else {
			/* Internal/compressed child. */
			if (ft_verify_node_recursive(ft, out, visited, path,
					cn->child,
					node_flag, depth + cn->len,
					&child_nr_keys))
				return -1;
		}
	}
	/* Verify nr_keys. */
	stored_nr_keys = ft_nr_keys_get(cn_meta);
	if (stored_nr_keys != child_nr_keys + local_keys) {
		if (out)
			fprintf(out, "ft_verify: depth %u: compressed node %p nr_keys mismatch: "
				"stored %lu, computed %lu (children %lu + local %lu)\n",
				depth, node_flag, stored_nr_keys,
				child_nr_keys + local_keys,
				child_nr_keys, local_keys);
		return -1;
	}
	*out_nr_keys = stored_nr_keys;
	return 0;
}

static
int ft_verify_node_recursive(const struct cds_ft *ft, FILE *out,
		struct ft_visited_set *visited,
		uint8_t *path,
		struct cds_ft_inode_flag *node_flag,
		struct cds_ft_inode_flag *expected_parent,
		unsigned int depth,
		unsigned long *out_nr_keys)
{
	/*
	 * Subtree-uniqueness / cycle check.  Every traversable node
	 * (compressed, internal) must be reached exactly once from
	 * the root.  A duplicate visit means either two
	 * parents share the same child subtree (rebase/recompact bug)
	 * or a parent-pointer cycle has been introduced — bail out
	 * before recursing further so the upward parent walks in the
	 * adjacency check cannot loop forever.
	 */
	{
		void *node_addr = ft_node_ptr(node_flag);
		int added = ft_visited_add(visited, node_addr);
		struct cds_ft_metadata *m = cds_ft_item_to_metadata(node_addr);

		if (added < 0) {
			if (out)
				fprintf(out, "ft_verify: depth %u: visited-set allocation failed at node %p\n",
					depth, node_flag);
			return -1;
		}
		if (added == 0) {
			if (out)
				fprintf(out, "ft_verify: depth %u: node %p reached twice (shared subtree or parent-pointer cycle)\n",
					depth, node_flag);
			return -1;
		}
		/*
		 * alloc_index round-trip: cds_ft_metadata_to_item walks back
		 * from the metadata to the arena slot using m->alloc_index.
		 * It must land on this very node; a corrupted alloc_index
		 * would otherwise survive verify and only fault later inside
		 * the allocator on free or recompact.
		 */
		if (cds_ft_metadata_to_item(m) != node_addr) {
			if (out)
				fprintf(out, "ft_verify: depth %u: node %p alloc_index round-trip yields %p (mismatch)\n",
					depth, node_flag,
					cds_ft_metadata_to_item(m));
			return -1;
		}
	}
	if (ft_node_compressed(node_flag))
		return ft_verify_node_compressed(ft, out, visited, path,
			node_flag, expected_parent, depth, out_nr_keys);

	/* --- Internal node (linear, popcount, pigeon) --- */
	{
		struct cds_ft_inode *node = ft_node_ptr(node_flag);
		struct cds_ft_metadata *metadata = cds_ft_item_to_metadata(node);
		struct cds_ft_node *external_nodes = metadata->external_nodes;
		unsigned long total_child_keys = 0;
		unsigned long local_keys = 0;
		unsigned int counted_children = 0;
		unsigned int key;

		/* Parent pointer check (root has NULL parent). */
		if (metadata->parent != expected_parent) {
			if (out)
				fprintf(out, "ft_verify: depth %u: internal node %p parent mismatch: "
					"expected %p, got %p\n",
					depth, node_flag, expected_parent,
					metadata->parent);
			return -1;
		}
#ifdef FT_IMMEDIATE_FREE
		/* Check parent target is not poisoned (freed). */
		if (metadata->parent) {
			unsigned char *p = (unsigned char *) ft_node_ptr(metadata->parent);
			if (*p == 0xfe) {
				if (out)
					fprintf(out, "ft_verify: depth %u: internal node %p parent %p points to freed (poisoned) node\n",
						depth, node_flag, metadata->parent);
				return -1;
			}
		}
#endif
		/*
		 * Type / alloc_index sanity.  ft_node_type already asserts
		 * the tag bits decode within FT_TYPE_BITS, but it does not
		 * verify the entry is a real internal class, that the arena
		 * order matches the type's expected order, or that nr_child
		 * fits the type's capacity.  A corrupted tag/bitfield write
		 * would otherwise survive verify and only manifest later as
		 * a wrong-sized scan or a min_child assertion.
		 */
		{
			unsigned int type_index = ft_node_type(node_flag);
			const struct cds_ft_type *type = &ft_types[type_index];
			size_t actual_order = cds_ft_item_order(node);

			if (type->type_class != FT_POPCOUNT &&
			    type->type_class != FT_PIGEON) {
				if (out)
					fprintf(out, "ft_verify: depth %u: internal node %p has non-internal type_class %d (type_index %u)\n",
						depth, node_flag,
						(int) type->type_class,
						type_index);
				return -1;
			}
			if (actual_order != type->order) {
				if (out)
					fprintf(out, "ft_verify: depth %u: internal node %p alloc order %zu mismatches type %u expected order %u\n",
						depth, node_flag,
						actual_order, type_index,
						(unsigned int) type->order);
				return -1;
			}
			if (metadata->nr_child > type->max_child) {
				if (out)
					fprintf(out, "ft_verify: depth %u: internal node %p nr_child %u exceeds type %u max_child %u\n",
						depth, node_flag,
						metadata->nr_child, type_index,
						(unsigned int) type->max_child);
				return -1;
			}
		}
		/* Count external nodes attached to this node's metadata. */
		if (external_nodes) {
			if (ft_verify_external_chain(ft, out, visited, path,
					node_flag, external_nodes, depth))
				return -1;
			local_keys = 1;	/* One unique key position. */
		}
		/* Walk all 256 child slots. */
		for (key = 0; key < 256; key++) {
			struct cds_ft_inode_flag *child_raw =
				ft_node_get_nth_skip(node_flag, NULL, (uint8_t) key, FT_PF_NONE);
			struct cds_ft_inode_flag *child;

			if (!child_raw)
				continue;
			/*
			 * Raw slot may be skip-encoded; verify the encoded
			 * slen matches the underlying compressed's cn->len
			 * before resolving for the recursion.
			 */
			if (ft_verify_skip_encoding(out, child_raw, depth + 1))
				return -1;
			child = ft_resolve_skip_compressed(child_raw);
			counted_children++;
			/*
			 * Path tracking: this slot's byte at @depth is
			 * the one being consumed to reach @child.
			 */
			if (path)
				path[depth] = (uint8_t) key;
			if (ft_node_external(child)) {
				/* External leaf chain at this slot. */
				if (ft_verify_external_chain(ft, out, visited,
						path, node_flag,
						(struct cds_ft_node *) ft_node_ptr(child),
						depth + 1))
					return -1;
				total_child_keys += 1;
			} else {
				unsigned long sub_keys = 0;

				if (ft_verify_node_recursive(ft, out, visited,
						path, child,
						node_flag, depth + 1,
						&sub_keys))
					return -1;
				total_child_keys += sub_keys;
			}
		}
		/* Verify nr_child. */
		if (metadata->nr_child != counted_children) {
			if (out)
				fprintf(out, "ft_verify: depth %u: internal node %p nr_child mismatch: "
					"stored %u, counted %u\n",
					depth, node_flag, metadata->nr_child,
					counted_children);
			return -1;
		}
		/*
		 * Pigeon bitmap consistency: pigeon nodes maintain a
		 * 256-bit live-slot bitmap (allocated alongside the node
		 * via ft_alloc_item with type->bitmap=true) that the
		 * directional / bitmap-scan readers consult.  The bitmap
		 * must agree slot-for-slot with the actual pointer array:
		 * bit i set iff node->data[i] holds a non-NULL pointer.
		 * ft_pigeon_node_get_nth reads node->data[n] directly
		 * (bitmap-independent), so cross-checking the two surfaces
		 * a desynchronised set / clear at the mutation site rather
		 * than letting it produce wrong directional results later.
		 */
		{
			unsigned int t = ft_node_type(node_flag);
			const struct cds_ft_type *t_type = &ft_types[t];

			if (ft_type_is_pigeon(t_type->type_class)) {
				struct cds_ft_bitmap *bm =
					cds_ft_item_to_bitmap(node, t_type->order);
				unsigned int b;

				for (b = 0; b < FT_ENTRY_PER_NODE; b++) {
					struct cds_ft_inode_flag *child =
						ft_pigeon_node_get_nth(NULL, node,
							NULL, (uint8_t) b,
							FT_PF_NONE);
					bool slot_set = ft_node_ptr(child) != NULL;
					bool bit_set = cds_test_bit(bm->bitmap, b);

					if (slot_set != bit_set) {
						if (out)
							fprintf(out, "ft_verify: depth %u: pigeon node %p slot %u: data %s, bitmap bit %s\n",
								depth, node_flag, b,
								slot_set ? "set" : "NULL",
								bit_set ? "set" : "clear");
						return -1;
					}
				}
			}
		}
#ifdef FEATURE_FT_COMPRESS
		/*
		 * Canonicalization (skip-compressed mode, non-root): a
		 * single-child internal node with no external_nodes attached
		 * should have been replaced by a 1-byte compressed node — in
		 * skip mode the compressed publishes as a skip-encoded
		 * pointer (zero read-side cost), strictly cheaper than the
		 * 1-child internal it stands in for.  The external_nodes
		 * carve-out is mandatory: compressed nodes cannot carry
		 * external_nodes, so an internal that hosts a NIL-key
		 * end-of-path and a single non-NIL branch must remain
		 * internal.
		 *
		 * The root is exempt: ft->root is read directly by the
		 * traversal entry, so the skip-encoded pointer's zero
		 * read-side cost has nowhere to attach (there is no parent
		 * slot to encode the slen into).  A 1-byte compressed at
		 * root costs the same CL as a 1-child internal, so the
		 * canonicalization policy is allowed to keep it internal.
		 *
		 * In non-skip mode, ft_build_ordinal_chain keeps a 1-byte
		 * compressed floor at len >= 2, so a 1-child internal at
		 * the head of a length-1 chain is canonical and must not
		 * trip this check; the runtime gate handles the distinction.
		 *
		 * Dual of the existing "no two adjacent compresseds" check.
		 */
		if (expected_parent != NULL &&
		    ft_group_skip_compressed(ft->group) &&
		    counted_children == 1 && !external_nodes) {
			if (out)
				fprintf(out, "ft_verify: depth %u: internal node %p has 1 child and no external_nodes (should be a 1-byte compressed in skip mode)\n",
					depth, node_flag);
			return -1;
		}
#endif
		/* Verify nr_keys. */
		{
			unsigned long stored_nr_keys = ft_nr_keys_get(metadata);

			if (stored_nr_keys != total_child_keys + local_keys) {
				if (out)
					fprintf(out, "ft_verify: depth %u: internal node %p nr_keys mismatch: "
						"stored %lu, computed %lu (children %lu + local %lu)\n",
						depth, node_flag, stored_nr_keys,
						total_child_keys + local_keys,
						total_child_keys, local_keys);
				return -1;
			}
			*out_nr_keys = stored_nr_keys;
		}
		return 0;
	}
}

/*
 * cds_ft_verify - Verify integrity of the entire Fractal Trie.
 *
 * Recursively walks every internal and compressed node starting
 * from the root, checking that nr_child, nr_keys, and parent
 * pointers are self-consistent.
 *
 * Must be called with mutual exclusion wrt updaters.
 *
 * @out: file stream for diagnostic output on failure (may be NULL
 *       to suppress output).
 *
 * Returns CDS_FT_STATUS_OK if the trie passes all checks, or
 * CDS_FT_STATUS_INTEGRITY_ERROR on integrity violation.
 */
enum cds_ft_status cds_ft_verify(const struct cds_ft *ft, FILE *out)
{
	struct cds_ft_inode_flag *root = ft->root;
	unsigned long root_nr_keys = 0;
	struct ft_visited_set visited;
	uint8_t *path = NULL;
	int ret;

	if (ft_visited_init(&visited)) {
		if (out)
			fprintf(out, "ft_verify: visited-set allocation failed\n");
		return CDS_FT_STATUS_INTEGRITY_ERROR;
	}
	/*
	 * End-to-end path/key consistency (invariant 10) requires an
	 * addressable copy of the inserted key at a group-known offset
	 * on each leaf, which the library no longer tracks (the user
	 * provides @key_offset per-call via cds_ft_speculative_lookup_key).
	 * Pass path = NULL so all path-tracking writes / compares
	 * short-circuit.
	 */
	ret = ft_verify_node_recursive(ft, out, &visited, path, root, NULL, 0,
			&root_nr_keys);
	ft_visited_destroy(&visited);
	if (ret)
		return CDS_FT_STATUS_INTEGRITY_ERROR;
	return CDS_FT_STATUS_OK;
}

#ifdef FEATURE_FT_VERIFY_AT_MUTATION
/*
 * Hook called from CDS_FT_SCOPED_WRITER's scope-exit, before the
 * writer claim is released.  Sampled by the per-trie
 * @verify_at_mutation_period: the cds_ft_verify walk runs once every
 * @period mutations.  The counter is incremented and reset on the
 * boundary so it never exceeds @period - 1, avoiding any overflow /
 * cadence-drift issue on long-running workloads.  Period 0 disables
 * the walk entirely (only the increment-and-compare runs).  On
 * mismatch, abort with diagnostic.
 */
void ft_writer_scope_verify(struct cds_ft *ft)
{
	unsigned long period = ft->verify_at_mutation_period;

	if (period == 0)
		return;
	ft->verify_at_mutation_counter++;
	if (ft->verify_at_mutation_counter < period)
		return;
	ft->verify_at_mutation_counter = 0;

	if (cds_ft_verify(ft, stderr) != CDS_FT_STATUS_OK) {
		fprintf(stderr, "FT verify-at-mutation: invariant violation on ft=%p\n",
			(void *) ft);
		abort();
	}
}
#endif

static
void print_indent(FILE *out, int level)
{
	int i;

	for (i = 0; i < level; i++)
		fprintf(out, "	");
}


static
void show_node_recursive(const struct cds_ft *ft, FILE *out, struct cds_ft_inode_flag *node_flag, int level)
{
	unsigned int key;

	print_indent(out, level);
	fprintf(out, "Level %d within node %p\n", level, node_flag);
	for (key = 0; key < 256; key++) {
		struct cds_ft_inode_flag *child_node_flag;

		child_node_flag = ft_node_get_nth(node_flag, NULL, (uint8_t) key, FT_PF_NONE);
		if (!child_node_flag)
			continue;
		if (ft_node_internal(child_node_flag)) {
			struct cds_ft_metadata *metadata = cds_ft_item_to_metadata(ft_node_ptr(child_node_flag));
			struct cds_ft_node *external_nodes = rcu_dereference(metadata->external_nodes);

			print_indent(out, level);
			fprintf(out, "Level %d, key value: %u, internal node: %p, nr_children: %u\n",
				level, key, child_node_flag, metadata->nr_child);
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
			fprintf(out, "Level %d, key value: %u, compressed node: %p, path_len: %u, nr_keys: %lu\n",
				level, key, child_node_flag, (unsigned int) cn->len,
				ft_nr_keys_get(metadata));
			if (external_nodes) {
				print_indent(out, level);
				fprintf(out, "Level %d, key value: %u, (meta)external node list ptr: %p\n",
					level, key, external_nodes);
			}
			if (cn->child &&
			    !ft_node_external(cn->child))
				show_node_recursive(ft, out, cn->child, level + cn->len);
			else if (cn->child) {
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

static
void show_pretty(const struct cds_ft *ft, FILE *out)
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
		fprintf(out, "Level 0: root node %p\n", node_flag);
		(void) rm;
	}
	show_node_recursive(ft, out, node_flag, level + 1);
	fprintf(out, "---------------------------------------------------\n");
}

/*
 * JSON emitter: walks the same trie structure as show_pretty() and
 * produces a JSON document describing it.  The output has no trailing
 * newline, so it can be embedded into other JSON contexts if desired.
 *
 * Schema summary:
 *   Root:     { "ft": "0xPTR", "root": <node> }
 *   Internal: { "ptr", "kind", "level", "nr_child", "density",
 *               "external_nodes"?, "children": [ {"key_byte", "child"} ] }
 *   Compressed: { "ptr", "kind": "COMPRESSED", "level", "path_len",
 *                 "key_bytes", "external_nodes"?, "child" }
 *   External:   { "ptr", "kind": "EXTERNAL", "level" }
 */

static void json_emit_node(const struct cds_ft *ft, FILE *out,
		struct cds_ft_inode_flag *node_flag, int level);

/*
 * Return a symbolic name for an internal-node type index.  Mirrors
 * the ft_tp_node_kind enum labels but is always compiled in (not
 * gated on FT_ENABLE_TRACING) since the JSON output is a supported
 * interface independent of tracing.
 */
static
const char *internal_type_name(unsigned int type_index)
{
	if (type_index >= sizeof(ft_types) / sizeof(ft_types[0]))
		return "UNKNOWN";
	{
		unsigned int cls = ft_types[type_index].type_class;
		unsigned int order = ft_types[type_index].order;

		switch (cls) {
		case FT_POPCOUNT:
			/*
			 * Orders 5/6 are 2-level popcount_2l layouts (P2L);
			 * orders 7/8/9/10 are 1-level popcount_1l layouts
			 * (P1L: a single 256-bit bitmap + ptr table).
			 */
			switch (order) {
			case 5:  return "P2L_32";
			case 6:  return "P2L_64";
			case 7:  return "P1L_128";
			case 8:  return "P1L_256";
			case 9:  return "P1L_512";
			case 10: return "P1L_1024";
			}
			break;
		case FT_PIGEON:
			switch (order) {
			case 10: return "PIGEON_1024";
			case 11: return "PIGEON_2048";
			}
			break;
		}
	}
	return "UNKNOWN";
}

static
void json_emit_density(FILE *out, const struct cds_ft_metadata *m __attribute__((unused)))
{
	fprintf(out, "[]");
}

static
void json_emit_node(const struct cds_ft *ft, FILE *out,
		struct cds_ft_inode_flag *node_flag, int level)
{
	if (!node_flag) {
		fprintf(out, "null");
		return;
	}
	if (ft_node_external(node_flag)) {
		fprintf(out, "{\"ptr\":\"%p\",\"kind\":\"EXTERNAL\","
			"\"level\":%d}", node_flag, level);
		return;
	}
	if (ft_node_compressed(node_flag)) {
		struct cds_ft_compressed_node *cn =
			ft_compressed_node_ptr(node_flag);
		struct cds_ft_metadata *metadata =
			cds_ft_item_to_metadata((struct cds_ft_inode *) cn);
		struct cds_ft_node *external_nodes =
			rcu_dereference(metadata->external_nodes);
		unsigned int j;

		fprintf(out, "{\"ptr\":\"%p\",\"kind\":\"COMPRESSED\","
			"\"level\":%d,\"path_len\":%u,\"nr_keys\":%lu,"
			"\"density\":",
			node_flag, level, (unsigned int) cn->len,
			ft_nr_keys_get(metadata));
		json_emit_density(out, metadata);
		fprintf(out, ",\"key_bytes\":[");
		for (j = 0; j < cn->len; j++) {
			if (j) fprintf(out, ",");
			fprintf(out, "%u", cn->key_bytes[j]);
		}
		fprintf(out, "]");
		if (external_nodes)
			fprintf(out, ",\"external_nodes\":\"%p\"",
				(void *) external_nodes);
		fprintf(out, ",\"child\":");
		if (cn->child)
			json_emit_node(ft, out, cn->child, level + cn->len);
		else
			fprintf(out, "null");
		fprintf(out, "}");
		return;
	}
	/* Internal. */
	{
		struct cds_ft_metadata *metadata =
			cds_ft_item_to_metadata(ft_node_ptr(node_flag));
		struct cds_ft_node *external_nodes =
			rcu_dereference(metadata->external_nodes);
		unsigned int type_index = ft_node_type(node_flag);
		unsigned int key, printed = 0;

		fprintf(out, "{\"ptr\":\"%p\",\"kind\":\"%s\",\"level\":%d,"
			"\"nr_child\":%u,\"density\":",
			node_flag, internal_type_name(type_index), level,
			metadata->nr_child);
		json_emit_density(out, metadata);
		if (external_nodes)
			fprintf(out, ",\"external_nodes\":\"%p\"",
				(void *) external_nodes);
		fprintf(out, ",\"children\":[");
		for (key = 0; key < 256; key++) {
			struct cds_ft_inode_flag *child;

			child = ft_node_get_nth(node_flag, NULL, (uint8_t) key, FT_PF_NONE);
			if (!child)
				continue;
			if (printed++) fprintf(out, ",");
			fprintf(out, "{\"key_byte\":%u,\"child\":", key);
			json_emit_node(ft, out, child, level + 1);
			fprintf(out, "}");
		}
		fprintf(out, "]}");
	}
}

static
void show_json(const struct cds_ft *ft, FILE *out)
{
	struct cds_ft_inode_flag *node_flag;

	node_flag = rcu_dereference(ft->root);
	fprintf(out, "{\"ft\":\"%p\",\"root\":", ft);
	json_emit_node(ft, out, node_flag, 0);
	fprintf(out, "}\n");
}

void cds_ft_show(const struct cds_ft *ft, FILE *out,
		enum cds_ft_show_format fmt)
{
	switch (fmt) {
	case CDS_FT_SHOW_JSON:
		show_json(ft, out);
		break;
	case CDS_FT_SHOW_PRETTY:
	default:
		show_pretty(ft, out);
		break;
	}
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

	CDS_FT_SCOPED_WRITER(ft);
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
void calc_stats_node_recursive(const struct cds_ft *ft, struct cds_ft_inode_flag *node_flag,
		struct cds_ft_stats *stats, int level)
{
	unsigned int key;

	for (key = 0; key < 256; key++) {
		struct cds_ft_inode_flag *child_node_flag;

		child_node_flag = ft_node_get_nth(node_flag, NULL, (uint8_t) key, FT_PF_NONE);
		if (!child_node_flag)
			continue;
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
			if (cn->child &&
			    !ft_node_external(cn->child))
				calc_stats_node_recursive(ft, cn->child, stats, level + cn->len);
			else if (cn->child) {
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
	case CDS_FT_STATUS_INTEGRITY_ERROR:
		return "Integrity verification failure";
	case CDS_FT_STATUS_NOT_SUPPORTED:
		return "Feature not compiled in";

	default:
		return "Unknown status value";
	}
}

enum cds_ft_status cds_ft_iter_create(struct cds_ft *ft, struct cds_ft_iter **result_iter)
{
	size_t max_depth = ft->group->max_tree_depth;
	size_t max_key_len = ft->group->max_key_len;
	size_t path_size = max_depth * sizeof(struct cds_ft_inode_flag *);
	/*
	 * Tail pad of FT_KEY_READABLE_PAD bytes after the key buffer so
	 * the descent can SIMD-load 32 bytes past key[key_len-1] without
	 * faulting.  The iter-based lookup paths pass FT_KEY_READABLE_PAD
	 * as @key_readable_pad (bytes safely loadable past key end).
	 */
	size_t key_size  = (max_key_len + FT_KEY_READABLE_PAD) * sizeof(uint8_t);
	struct cds_ft_iter *iter = calloc(1, sizeof(*iter) + path_size + key_size);

	CDS_FT_SCOPED_READER(ft);
	if (!iter) {
		*result_iter = NULL;
		return CDS_FT_STATUS_MEMORY_ERROR;
	}
	iter->ft = ft;
	iter->path_mode = CDS_FT_ITER_PATH_CACHED;
	*result_iter = iter;
	FT_TP(iter_create, (const void *) ft, (const void *) *result_iter);
	return CDS_FT_STATUS_OK;
}

void cds_ft_iter_destroy(struct cds_ft_iter *iter)
{
	FT_TP(iter_destroy, (const void *) iter->ft, (const void *) iter);
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
	const uint8_t *key_ordinals;

	key_len = ft_key_len(iter->ft, key_len);
	FT_TP(iter_set_key_enter, (const void *) iter->ft, (const void *) iter,
		key, key_len == CDS_FT_LEN_ERROR ? 0 : key_len,
		(int) iter->key_len, (int) iter->path_len);
	if (key_len > iter->ft->group->max_key_len)
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	if (caa_likely(km->identity)) {
		key_ordinals = key;
	} else {
		ft_key_to_ordinals(ordinal_buf, key, key_len, km);
		key_ordinals = ordinal_buf;
	}
	if (key_len <= iter->key_len && !memcmp(key_ordinals, iter_key(iter), key_len))
		subset = true;
	/*
	 * If new key is a subset of current key, the path stays valid,
	 * otherwise invalidate the path.
	 */
	if (!subset) {
		memcpy(iter_key(iter), key_ordinals, key_len);
		iter->path_valid = false;
		iter_debug_path_clear(iter);
		iter->path_len = 0;
	} else if (iter->path_len > key_len + 1) {
		/*
		 * Subset key reuses the existing path, but a prior
		 * lookup that returned NOT_FOUND may have truncated
		 * path_len below the previous key_len.  Only shrink
		 * path_len down to the new key's depth; never extend
		 * it past what was actually validated.
		 */
		iter->path_len = key_len + 1;
	}
	iter->key_len = key_len;
	FT_TP(iter_set_key_exit, (const void *) iter->ft, (const void *) iter,
		(int) subset, (int) iter->path_len);
	return CDS_FT_STATUS_OK;
}

/*
 * The prefix is a subset of the current key. Set the key before setting
 * the prefix length.
 */
enum cds_ft_status cds_ft_iter_set_prefix_len(struct cds_ft_iter *iter, size_t prefix_len)
{
	if (prefix_len > iter->key_len) {
		FT_TP(iter_set_prefix_len, (const void *) iter->ft,
			(const void *) iter, (int) prefix_len);
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}
	iter->prefix_len = prefix_len;
	FT_TP(iter_set_prefix_len, (const void *) iter->ft,
		(const void *) iter, (int) prefix_len);
	return CDS_FT_STATUS_OK;
}

void cds_ft_iter_reset(struct cds_ft_iter *iter)
{
	FT_TP(iter_reset, (const void *) iter->ft, (const void *) iter);
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
	FT_TP(iter_invalidate_path, (const void *) iter->ft, (const void *) iter);
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
