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
	bool speculative;
	bool speculative_validated;
	size_t speculative_key_offset;
	size_t speculative_key_len_offset;
};

struct cds_ft_attr {
	bool exclusive;
	unsigned int collapse_threshold_pct;
	unsigned int collapse_scan_mul_pct;
	unsigned int compress_scan_mul_pct;
};

enum cds_ft_type_class {
	FT_LINEAR = 0,		/* Linear: per-type specialized scan */
	FT_POPCOUNT = 1,	/* Popcount-bitmap: popcount_1l / popcount_2l */
	FT_PIGEON = 2,		/* Pigeon: direct indexed */
	/* Leaf nodes are implicit from their height in the tree */
	FT_NR_TYPES = 3,

	FT_NULL,	/* not an encoded type, but keeps code regular */
};

#define ft_type_is_linear(tc)	((tc) == FT_LINEAR)
#define ft_type_is_popcount(tc)	((tc) == FT_POPCOUNT)

/*
 * FT_HAVE_EFFICIENT_UNALIGNED_ACCESS: architectures where unaligned
 * loads that stay within a single cacheline have no measurable
 * overhead vs aligned loads.  Required for the SIMD/SWAR linear
 * node scans that load the values+padding region as one word.
 * Nodes are aligned to their size (>=64), so 16- or 32-byte scans
 * stay within one cacheline by construction.
 *
 * On strict-alignment architectures, all linear nodes fall back to
 * the bytewise scan.
 */
#if defined(__x86_64__) || defined(__i386__) || defined(__aarch64__) \
	|| (defined(__powerpc64__) && defined(__LITTLE_ENDIAN__))
#define FT_HAVE_EFFICIENT_UNALIGNED_ACCESS
#endif

struct cds_ft_type {
	enum cds_ft_type_class type_class;
	uint16_t min_child;		/* minimum number of children: 1 to 256 */
	uint16_t max_child;		/* maximum number of children: 1 to 256 */
	uint16_t max_linear_child;	/* max children with linear-scan layout: 1 to 256 */
	uint16_t order;			/* node size is (1 << order), in bytes */
	bool bitmap;			/* allocate bitmap */
	bool popcount_2l;		/* 2-level popcount-bitmap layout */
	bool popcount_1l;			/* 1-level byte popcount layout */
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
	ft_type_0_max_child = 3,
	/*
	 * 32-bit max-density popcount tiers:
	 *   idx 1  scan_16_16_max_5  32 B  hdr 12 B +  5 x 4 B = 32 B exact
	 *   idx 2  scan_64_4         64 B  hdr 16 B + 12 x 4 B = 64 B exact
	 *                            (flat 6+2; node-size cap 12, bitmap cap 16)
	 *   idx 3  scan_64_4        128 B  hdr 16 B + 16 x 4 B = 80 B used,
	 *                            48 B trailing slack (flat 6+2 reused in
	 *                            128 B; bitmap cap 16 is binding)
	 *   idx 4  bp_1l            256 B  hdr 32 B + 56 x 4 B = 256 B exact
	 *   idx 5  bp_1l            512 B  hdr 32 B + 120 x 4 B = 512 B exact
	 */
	ft_type_1_max_child = 5,
	ft_type_2_max_child = 12,
	ft_type_3_max_child = 16,
	ft_type_4_max_child = 56,
	ft_type_5_max_child = 120,
	ft_type_6_max_child = 256,
	ft_type_7_max_child = 0,	/* NULL */
};

enum {
	ft_type_0_max_linear_child = 3,
	ft_type_1_max_linear_child = 5,
	ft_type_2_max_linear_child = 12,
	ft_type_3_max_linear_child = 16,
};

const struct cds_ft_type ft_types[] = {
	[0] = { .type_class = FT_LINEAR, .min_child = 1, .max_child = ft_type_0_max_child, .max_linear_child = ft_type_0_max_linear_child, .order = 4, .bitmap = FT_NO_BITMAP },
	[1] = {
		.type_class = FT_POPCOUNT,
		.popcount_2l = true,
		.min_child = 1,
		.max_child = ft_type_1_max_child, .max_linear_child = ft_type_1_max_linear_child, .order = 5, .bitmap = FT_NO_BITMAP,
	},
	[2] = {
		.type_class = FT_POPCOUNT,
		.popcount_2l = true,
		.min_child = 3,
		.max_child = ft_type_2_max_child, .max_linear_child = ft_type_2_max_linear_child, .order = 6, .bitmap = FT_NO_BITMAP,
	},
	[3] = {
		.type_class = FT_POPCOUNT,
		.popcount_2l = true,
		.min_child = 5,
		.max_child = ft_type_3_max_child, .max_linear_child = ft_type_3_max_linear_child, .order = 7, .bitmap = FT_NO_BITMAP,
	},

	/*
	 * Indices 4 and 5 use popcount_1l (32 B bitmap + 4 B ptr
	 * table fills the node exactly).
	 */
	[4] = {
		/* 32 B bitmap + 56 * 4 B ptrs = 256 B (order-8). */
		.type_class = FT_POPCOUNT,
		.max_linear_child = ft_type_4_max_child,
		.popcount_1l = true,
		.min_child = 10,
		.max_child = ft_type_4_max_child, .order = 8, .bitmap = FT_NO_BITMAP },
	[5] = {
		/* 32 B bitmap + 120 * 4 B ptrs = 512 B (order-9). */
		.type_class = FT_POPCOUNT,
		.max_linear_child = ft_type_5_max_child,
		.popcount_1l = true,
		.min_child = 28,
		.bitmap = FT_NO_BITMAP,
		.max_child = ft_type_5_max_child, .order = 9 },

	/*
	 * Upon node removal below min_child, if a popcount_1l node would
	 * otherwise be filled beyond capacity, we roll back to pigeon.
	 */
	[6] = { .type_class = FT_PIGEON,
		.min_child = 51,
		.max_child = ft_type_6_max_child, .order = 10, .bitmap = FT_BITMAP },

	[7] = { .type_class = FT_NULL, .min_child = 0, .max_child = ft_type_7_max_child, .bitmap = FT_NO_BITMAP },
};
#else /* !(CAA_BITS_PER_LONG < 64) */
/* 64-bit pointers */
enum {
	ft_type_0_max_child = 1,
	ft_type_1_max_child = 3,
	ft_type_2_max_child = 6,	/* scan_32_8 (per-slot 5+3, qp_6) */
	ft_type_3_max_child = 14,	/* scan_64_4 (flat 6+2, qp_14) */
	ft_type_4_max_child = 28,
	ft_type_5_max_child = 54,
	ft_type_6_max_child = 104,
	ft_type_7_max_child = 256,
	ft_type_8_max_child = 256,
};

enum {
	ft_type_0_max_linear_child = 1,
	ft_type_1_max_linear_child = 3,
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
	ft_type_2_max_linear_child = 6,
	ft_type_3_max_linear_child = 14,
};

const struct cds_ft_type ft_types[] = {
	[0] = { .type_class = FT_LINEAR, .min_child = 1, .max_child = ft_type_0_max_child, .max_linear_child = ft_type_0_max_linear_child, .order = 4, .bitmap = FT_NO_BITMAP },
	[1] = {
		.type_class = FT_POPCOUNT,
		.popcount_2l = true,
		.min_child = 1, .max_child = ft_type_1_max_child, .max_linear_child = ft_type_1_max_linear_child, .order = 5, .bitmap = FT_NO_BITMAP,
	},
	[2] = {
		.type_class = FT_POPCOUNT,
		.popcount_2l = true,
		.min_child = 3, .max_child = ft_type_2_max_child, .max_linear_child = ft_type_2_max_linear_child, .order = 6, .bitmap = FT_NO_BITMAP,
	},
	[3] = {
		.type_class = FT_POPCOUNT,
		.popcount_2l = true,
		.min_child = 5, .max_child = ft_type_3_max_child, .max_linear_child = ft_type_3_max_linear_child, .order = 7, .bitmap = FT_NO_BITMAP,
	},
	[4] = {
		.type_class = FT_POPCOUNT,
		.popcount_1l = true,
		.min_child = 10, .max_child = ft_type_4_max_child, .max_linear_child = ft_type_4_max_child, .order = 8, .bitmap = FT_NO_BITMAP,
	},

	/*
	 * Indices 5 and 6 use popcount_1l (32B bitmap + 8B ptr
	 * table fills the node).
	 */
	[5] = {
		/* 32B bitmap + 54 * 8B ptrs = 464B, fits 512B order-9. */
		.type_class = FT_POPCOUNT,
		.max_linear_child = ft_type_5_max_child,
		.popcount_1l = true,
		.min_child = 22,
		.max_child = ft_type_5_max_child,
		.order = 9, .bitmap = FT_NO_BITMAP },
	[6] = {
		/* 32B bitmap + 104 * 8B ptrs = 864B, fits 1024B order-10. */
		.type_class = FT_POPCOUNT,
		.max_linear_child = ft_type_6_max_child,
		.popcount_1l = true,
		.min_child = 51,
		.max_child = ft_type_6_max_child,
		.order = 10, .bitmap = FT_NO_BITMAP },

	/*
	 * Upon node removal below min_child, if a popcount_1l node would
	 * otherwise be filled beyond capacity, we roll back to pigeon.
	 */
	[7] = { .type_class = FT_PIGEON, .min_child = 95, .max_child = ft_type_7_max_child, .order = 11, .bitmap = FT_BITMAP },

	[8] = { .type_class = FT_NULL, .min_child = 0, .max_child = ft_type_8_max_child, .bitmap = FT_NO_BITMAP },
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
 * 1. FT_LINEAR:
 * - data[0 .. max_linear_child-1]: uint8_t keys (child values).
 *   The populated count is derived at lookup time via the
 *   sentinel scheme in ft_linear_node_get_nr_child: positions i > 0
 *   hold values distinct from values[0]; the first i >= 1 where
 *   values[i] == values[0] marks the unwritten tail.  No explicit
 *   nr_child field lives in the node — the authoritative count is
 *   in the out-of-line metadata (struct cds_ft_metadata).
 * - [Padding] to reach the next pointer-aligned (8-byte) boundary.
 * - Array of (struct cds_ft_inode_flag *) pointers.
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
 * aligned (bits 0-2 zero), compressed/collapsed are >= 16-byte
 * aligned with tags in bits 1-2.  A fixed ~7UL mask suffices.
 *
 * The conditional select lets the two mask computations run in
 * parallel; the compiler emits a CMOV, keeping the critical path
 * to 4 cycles.
 */
/* Forward declarations for density and nr_keys helpers (defined after density accessors). */
static inline bool ft_density_is_extended(const struct cds_ft_metadata *m);
static void ft_density_promote(struct cds_ft *ft, struct cds_ft_metadata *m);
static inline unsigned long ft_density_get(const struct cds_ft_metadata *m, unsigned int idx);
static inline void ft_density_set(struct cds_ft *ft, struct cds_ft_metadata *m, unsigned int idx, unsigned long val);
static inline unsigned long ft_nr_keys_get(const struct cds_ft_metadata *m);
static inline unsigned long ft_nr_keys_load(const struct cds_ft_metadata *m);
static inline void ft_nr_keys_store(struct cds_ft *ft, struct cds_ft_metadata *m, unsigned long val, int mo);
static void ft_propagate_node_density_parent(struct cds_ft *ft, struct cds_ft_inode_flag *start,
		unsigned int start_depth, unsigned int node_depth, long delta);
static unsigned int ft_parent_depth_span(struct cds_ft_inode_flag *parent_nf,
		struct cds_ft_inode_flag *child_nf);

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

/*
 * Recover the tier (0..3) from a collapsed-node tagged pointer.
 * Tier sits in pointer bits 4-5; collapsed nodes are 64-byte aligned
 * (smallest tier is 64B) so those bits are tag-safe.
 *
 * Phase 1: no caller sets tier bits yet, so this currently returns 0
 * for every collapsed pointer in the codebase.  The tier helper is
 * here so that Phase 2+ rewrites can encode and decode it without
 * further header churn.
 */
static inline_lookup
unsigned int ft_collapsed_tier(struct cds_ft_inode_flag *node)
{
	return (unsigned int) (((unsigned long) node) >> FT_COL_TIER_SHIFT)
			& ((1U << FT_COL_TIER_BITS) - 1U);
}

/*
 * Recover the stride bit (NARROW / WIDE) from a tagged collapsed-node
 * pointer.  Returns FT_COL_STRIDE_NARROW (0) for the existing 16-byte
 * entry layout and FT_COL_STRIDE_WIDE (1) for the 32-byte wide layout.
 */
static inline_lookup
unsigned int ft_collapsed_stride(struct cds_ft_inode_flag *node)
{
	return (unsigned int) (((unsigned long) node) >> FT_COL_STRIDE_SHIFT)
			& ((1U << FT_COL_STRIDE_BITS) - 1U);
}

/*
 * Used only by tracepoint payloads in free_collapsed_node /
 * free_collapsed_node_unpublished, which compile out when tracing is
 * disabled.  Marked unused so configs without FT_ENABLE_TRACING don't
 * warn.
 */
static __attribute__((unused))
struct cds_ft_inode_flag *ft_collapsed_node_flag(
		struct cds_ft_collapsed_node *node)
{
	return (struct cds_ft_inode_flag *)
		(((unsigned long) node) | FT_COLLAPSED_MASK);
}

/*
 * Tier-aware variant for use by Phase 2+ allocation / publication
 * paths.  Encodes the tier index (0..3) into pointer bits 4-5.
 *
 * Stride defaults to NARROW (bit 3 = 0); use ft_collapsed_node_flag_tier_stride
 * to publish a wide-stride collapsed node.
 */
static inline
struct cds_ft_inode_flag *ft_collapsed_node_flag_tier(
		struct cds_ft_collapsed_node *node, unsigned int tier)
{
	return (struct cds_ft_inode_flag *)
		(((unsigned long) node)
			| FT_COLLAPSED_MASK
			| ((unsigned long) tier << FT_COL_TIER_SHIFT));
}

/*
 * Encode tier and stride bits into a tagged collapsed-node pointer.
 * stride = FT_COL_STRIDE_NARROW (0) reproduces ft_collapsed_node_flag_tier;
 * stride = FT_COL_STRIDE_WIDE (1) sets bit 3 of the pointer so the
 * read side dispatches on the wide entry layout.
 */
static inline
struct cds_ft_inode_flag *ft_collapsed_node_flag_tier_stride(
		struct cds_ft_collapsed_node *node,
		unsigned int tier, unsigned int stride)
{
	return (struct cds_ft_inode_flag *)
		(((unsigned long) node)
			| FT_COLLAPSED_MASK
			| ((unsigned long) stride << FT_COL_STRIDE_SHIFT)
			| ((unsigned long) tier << FT_COL_TIER_SHIFT));
}

static inline_lookup
struct cds_ft_collapsed_node *ft_collapsed_node_ptr(
		struct cds_ft_inode_flag *node)
{
	return (struct cds_ft_collapsed_node *)
		(((unsigned long) node) & ~(unsigned long) FT_TAG_MASK_COLLAPSED);
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
 * For internal/compressed/collapsed children: uses metadata->parent.
 * For external (leaf) children: uses cds_ft_node.prev (which points
 * to the parent for the head of a duplicate chain).
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
 * ft_get_parent_rcu: read the parent pointer of @node via
 * rcu_dereference.
 *
 * For external (leaf) nodes: returns cds_ft_node.prev.  @node must
 * be the head of its duplicate chain (non-head duplicates' prev
 * points to the preceding node in the chain, not to the parent).
 * Iterators and lookups maintain this invariant by convention —
 * iter->node always refers to the chain head.
 *
 * For internal/compressed/collapsed nodes: returns metadata->parent.
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
 * Centralizes the dual-pointer RCU publication pattern so every
 * write to cn->child automatically maintains the skip pointer.
 */
static
void ft_publish_to_parent(struct cds_ft *ft,
		struct cds_ft_inode_flag *parent_nf,
		struct cds_ft_inode_flag **parent_slot,
		struct cds_ft_inode_flag *new_child)
{
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
 * Call AFTER ft_set_parent and ft_init_node_density have been done
 * with the real compressed flag (@cflag).  The returned value is
 * what should be published/stored in parent child slots.
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

/* Collapsed node accessors. */

/*
 * Per-tier metadata tables (read-only, compile-time-constant).
 *
 * Indexed by tier (0..3) recovered from a collapsed-node tagged
 * pointer via ft_collapsed_tier().
 */
static const uint8_t ft_col_tier_capacity[FT_COL_NR_STRIDES][FT_COL_NR_TIERS] = {
	[FT_COL_STRIDE_NARROW] = {
		[0] = FT_COL_T0_CAPACITY,
		[1] = FT_COL_T1_CAPACITY,
		[2] = FT_COL_T2_CAPACITY,
		[3] = FT_COL_T3_CAPACITY,
	},
	[FT_COL_STRIDE_WIDE] = {
		[0] = FT_COL_W_T0_CAPACITY,
		[1] = FT_COL_W_T1_CAPACITY,
		[2] = FT_COL_W_T2_CAPACITY,
		[3] = FT_COL_W_T3_CAPACITY,
	},
};

static const uint8_t __attribute__((unused)) ft_col_tier_alloc_order[FT_COL_NR_TIERS] = {
	[0] = FT_COL_T0_ALLOC_ORDER,
	[1] = FT_COL_T1_ALLOC_ORDER,
	[2] = FT_COL_T2_ALLOC_ORDER,
	[3] = FT_COL_T3_ALLOC_ORDER,
};

static const uint8_t ft_col_tier_header_size[FT_COL_NR_TIERS] = {
	[0] = FT_COL_T0_HEADER_SIZE,
	[1] = FT_COL_T1_HEADER_SIZE,
	[2] = FT_COL_T2_HEADER_SIZE,
	[3] = FT_COL_T3_HEADER_SIZE,
};

static const uint8_t ft_col_tier_prefix_stride[FT_COL_NR_TIERS] = {
	[0] = FT_COL_T0_PREFIX_STRIDE,
	[1] = FT_COL_T1_PREFIX_STRIDE,
	[2] = FT_COL_T2_PREFIX_STRIDE,
	[3] = FT_COL_T3_PREFIX_STRIDE,
};

/* Per-stride entry size: 16B narrow / 32B wide. */
static const uint8_t ft_col_stride_entry_size[FT_COL_NR_STRIDES]
		__attribute__((unused)) = {
	[FT_COL_STRIDE_NARROW]	= FT_COL_ENTRY_SIZE,
	[FT_COL_STRIDE_WIDE]	= FT_COL_W_ENTRY_SIZE,
};

/* Per-stride suffix-byte cap: 7 narrow / 23 wide. */
static const uint8_t ft_col_stride_suffix_max[FT_COL_NR_STRIDES]
		__attribute__((unused)) = {
	[FT_COL_STRIDE_NARROW]	= FT_COL_SUFFIX_MAX,
	[FT_COL_STRIDE_WIDE]	= FT_COL_W_SUFFIX_MAX,
};

/*
 * Capacity of a collapsed node at the given (tier, stride).  Narrow
 * is the existing 16B-entry layout; wide halves the entry count per
 * tier in exchange for a 23-byte inline suffix.
 */
static inline_lookup
unsigned int ft_collapsed_capacity_stride(unsigned int tier, unsigned int stride)
{
	return ft_col_tier_capacity[stride][tier];
}

/*
 * Pointer to the narrow entries[] array.  Each entry is 16 bytes;
 * the array starts at a tier-dependent header offset (16B for T0/T1,
 * 64B for T2/T3) so it is 16-byte aligned for SWAR.
 *
 * Wide nodes use ft_collapsed_entries_wide() instead — the header
 * offset is identical, but the per-entry stride is 32B and the typed
 * pointer differs.
 */
static inline_lookup
struct cds_ft_collapsed_entry *ft_collapsed_entries(
		struct cds_ft_collapsed_node *col, unsigned int tier)
{
	return (struct cds_ft_collapsed_entry *)
		(((uint8_t *) col) + ft_col_tier_header_size[tier]);
}

/*
 * Pointer to the wide entries[] array.  Each entry is 32 bytes; the
 * array starts at the same tier-dependent header offset as the
 * narrow case (header size is stride-invariant).
 */
static inline_lookup
struct cds_ft_collapsed_entry_wide *ft_collapsed_entries_wide(
		struct cds_ft_collapsed_node *col, unsigned int tier)
{
	return (struct cds_ft_collapsed_entry_wide *)
		(((uint8_t *) col) + ft_col_tier_header_size[tier]);
}

/* Pointer to prefix_0 array (always at offset 0). */
static inline_lookup
uint8_t *ft_collapsed_prefix_0(struct cds_ft_collapsed_node *col,
		unsigned int tier __attribute__((unused)))
{
	return (uint8_t *) col;
}

/*
 * Pointer to prefix_1 array.  Immediately follows prefix_0; the
 * stride per tier accounts for SIMD-load alignment (8B for T0/T1,
 * 16B for T2, 32B for T3).
 */
static inline_lookup
uint8_t *ft_collapsed_prefix_1(struct cds_ft_collapsed_node *col,
		unsigned int tier)
{
	return ((uint8_t *) col) + ft_col_tier_prefix_stride[tier];
}

/*
 * SIMD prefix-cache match: AND of (prefix_0 == k0) and (prefix_1 == k1),
 * returned as a per-slot bitmask (bit e set iff slot e matches both).
 *
 * Caller must guarantee at least one of {k0, k1} is non-zero.  Under
 * that precondition, padding bytes (zero-init) and uninitialized slots
 * cannot generate set bits, so no dynamic per-tier tail mask is needed
 * for T0/T1/T2.  T3 only applies a static 0x0FFFFFFF mask to clear the
 * 4 high bits in the 32-bit movemask result.
 *
 * The {0x00, 0x00} target case is handled by a separate fast-path
 * bypass in the caller (only slot 0 can hold a \0\0-prefix entry under
 * the strict sort invariant), so this helper is never called with
 * k0 == 0 && k1 == 0.
 */
#if defined(__SSE2__) && !defined(FT_NO_SIMD_CMP)
static inline_lookup
unsigned int ft_collapsed_simd_match(struct cds_ft_collapsed_node *col,
		unsigned int tier, uint8_t k0, uint8_t k1)
{
	const uint8_t *p0 = ft_collapsed_prefix_0(col, tier);
	const uint8_t *p1 = ft_collapsed_prefix_1(col, tier);
	__m128i b0 = _mm_set1_epi8((char) k0);
	__m128i b1 = _mm_set1_epi8((char) k1);
	__m128i v0, v1, eq0, eq1, eqand;
	unsigned int m;

	switch (tier) {
	case 0:
	case 1: {
		/*
		 * 8-byte prefix arrays: load via 64-bit move into the low
		 * half of an XMM, high half zero-extended.  cmpeq against
		 * a non-zero broadcast leaves the high half as all-zero
		 * lanes, contributing 0 bits to movemask.
		 */
		v0 = _mm_loadl_epi64((const __m128i *) p0);
		v1 = _mm_loadl_epi64((const __m128i *) p1);
		eq0 = _mm_cmpeq_epi8(v0, b0);
		eq1 = _mm_cmpeq_epi8(v1, b1);
		eqand = _mm_and_si128(eq0, eq1);
		return (unsigned int) _mm_movemask_epi8(eqand);
	}
	case 2: {
		v0 = _mm_load_si128((const __m128i *) p0);
		v1 = _mm_load_si128((const __m128i *) p1);
		eq0 = _mm_cmpeq_epi8(v0, b0);
		eq1 = _mm_cmpeq_epi8(v1, b1);
		eqand = _mm_and_si128(eq0, eq1);
		return (unsigned int) _mm_movemask_epi8(eqand);
	}
	case 3:
	default: {
		__m128i v0a = _mm_load_si128((const __m128i *)(p0 + 0));
		__m128i v0b = _mm_load_si128((const __m128i *)(p0 + 16));
		__m128i v1a = _mm_load_si128((const __m128i *)(p1 + 0));
		__m128i v1b = _mm_load_si128((const __m128i *)(p1 + 16));
		__m128i ea = _mm_and_si128(_mm_cmpeq_epi8(v0a, b0),
					   _mm_cmpeq_epi8(v1a, b1));
		__m128i eb = _mm_and_si128(_mm_cmpeq_epi8(v0b, b0),
					   _mm_cmpeq_epi8(v1b, b1));
		unsigned int ma = (unsigned int) _mm_movemask_epi8(ea);
		unsigned int mb = (unsigned int) _mm_movemask_epi8(eb);

		m = ma | (mb << 16);
		/*
		 * T3's second 16B vector contributes 4 bytes of trailing
		 * padding (capacity 28, vector width 32).  Clear bits
		 * 28..31 so the candidate iterator never visits a
		 * non-existent slot.
		 */
		return m & 0x0FFFFFFFU;
	}
	}
}
#else
static inline_lookup
unsigned int ft_collapsed_simd_match(struct cds_ft_collapsed_node *col,
		unsigned int tier, uint8_t k0, uint8_t k1)
{
	const uint8_t *p0 = ft_collapsed_prefix_0(col, tier);
	const uint8_t *p1 = ft_collapsed_prefix_1(col, tier);
	/*
	 * Prefix arrays are sized for narrow capacity regardless of the
	 * node's stride — the layout is stride-invariant.  Walking narrow
	 * lanes on a wide node visits zero-init padding for slots beyond
	 * the wide capacity; those only match on (k0, k1) = (0, 0), which
	 * is handled by the (0, 0) bypass walk and never reaches this
	 * prefilter.
	 */
	unsigned int cap = ft_collapsed_capacity_stride(tier,
			FT_COL_STRIDE_NARROW);
	unsigned int m = 0, e;

	for (e = 0; e < cap; e++) {
		if (p0[e] == k0 && p1[e] == k1)
			m |= 1U << e;
	}
	return m;
}
#endif /* __SSE2__ && !FT_NO_SIMD_CMP */

/*
 * Count live entries (slots with child != NULL).
 *
 * Relaxed load: only the NULL/non-NULL status of @child is consumed,
 * never the per-slot suffix/len bytes, so no acquire is needed.
 */
static inline
unsigned int ft_collapsed_count_live(struct cds_ft_collapsed_node *col,
		unsigned int tier, unsigned int stride)
{
	unsigned int cap = ft_collapsed_capacity_stride(tier, stride);
	unsigned int e, n = 0;

	if (stride == FT_COL_STRIDE_NARROW) {
		struct cds_ft_collapsed_entry *entries = ft_collapsed_entries(col, tier);

		for (e = 0; e < cap; e++) {
			if (uatomic_load(&entries[e].child, CMM_RELAXED) != NULL)
				n++;
		}
	} else {
		struct cds_ft_collapsed_entry_wide *entries = ft_collapsed_entries_wide(col, tier);

		for (e = 0; e < cap; e++) {
			if (uatomic_load(&entries[e].child, CMM_RELAXED) != NULL)
				n++;
		}
	}
	return n;
}

/*
 * High-water-mark: index of the first never-written slot.
 * Equivalent to the count of slots ever published.
 *
 * Once-live-then-dead slots still have len > 0 (since min suffix
 * len is 1), so they are counted as "ever written".  Born-dead
 * slots have len == 0 (zero-init by allocator).
 *
 * Used by the writer to find the next free slot for in-order
 * tail-append.  Read with relaxed ordering — len bytes are
 * immutable once published.
 */
static inline
unsigned int ft_collapsed_high_water(struct cds_ft_collapsed_node *col,
		unsigned int tier, unsigned int stride)
{
	unsigned int cap = ft_collapsed_capacity_stride(tier, stride);
	unsigned int e;

	if (stride == FT_COL_STRIDE_NARROW) {
		struct cds_ft_collapsed_entry *entries = ft_collapsed_entries(col, tier);

		for (e = 0; e < cap; e++) {
			if (entries[e].len == 0)
				return e;
		}
	} else {
		struct cds_ft_collapsed_entry_wide *entries = ft_collapsed_entries_wide(col, tier);

		for (e = 0; e < cap; e++) {
			if (entries[e].len == 0)
				return e;
		}
	}
	return cap;
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
 *                  (when not: 00=external, 01=compressed, 11=collapsed)
 *
 * Compressed and collapsed nodes are 16-byte aligned, so bit 3 is
 * guaranteed zero for them.  External nodes need only 8-byte
 * alignment (low 3 bits = 000), so bit 3 may be either value — both
 * [0b0000] and [0b1000] map to EXTERNAL.
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
#define FT_TP_KIND_LINEAR(ord) (				\
	(ord) == 4 ? FT_TP_NODE_LINEAR_16 :			\
	FT_TP_NODE_UNKNOWN)
#define FT_TP_KIND_P2L(ord) (					\
	(ord) == 5 ? FT_TP_NODE_P2L_32 :			\
	(ord) == 6 ? FT_TP_NODE_P2L_64 :			\
	(ord) == 7 ? FT_TP_NODE_P2L_128 :			\
	FT_TP_NODE_UNKNOWN)
#define FT_TP_KIND_P1L(ord) (					\
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
	/* Collapsed: low 3 bits = 110, bit 3 = 0 (16-byte aligned). */
	[FT_COLLAPSED_MASK]		= FT_TP_NODE_COLLAPSED,
	/*
	 * Internal nodes: bit 0 set, bits 1..3 = type index.  Each
	 * arch-specific ft_types[] is mapped via FT_TP_KIND_*().
	 */
#if (CAA_BITS_PER_LONG < 64)
	[FT_TP_INTERNAL_TAG(0)]		= FT_TP_KIND_LINEAR(4),
	[FT_TP_INTERNAL_TAG(1)]		= FT_TP_KIND_P2L(5),
	[FT_TP_INTERNAL_TAG(2)]		= FT_TP_KIND_P2L(6),
	[FT_TP_INTERNAL_TAG(3)]		= FT_TP_KIND_P2L(7),
	[FT_TP_INTERNAL_TAG(4)]		= FT_TP_KIND_P1L(8),
	[FT_TP_INTERNAL_TAG(5)]		= FT_TP_KIND_P1L(9),
	[FT_TP_INTERNAL_TAG(6)]		= FT_TP_KIND_PIGEON(10),
	/* idx 7 = NODE_INDEX_NULL: never encoded in a pointer. */
#else
	[FT_TP_INTERNAL_TAG(0)]		= FT_TP_KIND_LINEAR(4),
	[FT_TP_INTERNAL_TAG(1)]		= FT_TP_KIND_P2L(5),
	[FT_TP_INTERNAL_TAG(2)]		= FT_TP_KIND_P2L(6),
	[FT_TP_INTERNAL_TAG(3)]		= FT_TP_KIND_P2L(7),
	[FT_TP_INTERNAL_TAG(4)]		= FT_TP_KIND_P1L(8),
	[FT_TP_INTERNAL_TAG(5)]		= FT_TP_KIND_P1L(9),
	[FT_TP_INTERNAL_TAG(6)]		= FT_TP_KIND_P1L(10),
	[FT_TP_INTERNAL_TAG(7)]		= FT_TP_KIND_PIGEON(11),
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
	 * skip pointer still points to a real child (external, internal,
	 * collapsed, ...).  Strip the skip-length bits so the dispatch
	 * table sees the underlying child's tag bits; the companion
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
 * Suffix bytes for a collapsed entry — &entry->suffix[0].  Returned
 * pointer covers FT_COL_SUFFIX_MAX (7) bytes; the valid prefix length
 * is given by ft_collapsed_suffix_len().
 */
static inline
uint8_t *ft_collapsed_suffix(struct cds_ft_collapsed_entry *entry)
{
	return entry->suffix;
}

/* Suffix length (1..7) for a collapsed entry. */
static inline
unsigned int ft_collapsed_suffix_len(struct cds_ft_collapsed_entry *entry)
{
	return entry->len;
}

/* Liveness check: dead iff child == NULL.  Relaxed load is fine for
 * filtering during iteration; sites that descend into the child must
 * use ft_dereference_acquire on the child pointer. */
static inline
bool ft_collapsed_entry_dead(struct cds_ft_collapsed_entry *entry)
{
	return uatomic_load(&entry->child, CMM_RELAXED) == NULL;
}

/* Wide variant of ft_collapsed_entry_dead. */
static inline
bool ft_collapsed_wide_entry_dead(struct cds_ft_collapsed_entry_wide *entry)
{
	return uatomic_load(&entry->child, CMM_RELAXED) == NULL;
}

/*
 * Write the SIMD prefix-cache bytes for a slot.  The prefix arrays
 * are a best-effort fast-path filter; the per-entry (suffix, len)
 * block is the source of truth.  Stores are relaxed and may be
 * observed torn by SIMD readers — verify on the atomically-published
 * subkey catches any false acceptance.
 */
static inline
void ft_collapsed_write_prefix_bytes(struct cds_ft_collapsed_node *col,
		unsigned int tier, unsigned int e,
		uint8_t b0, uint8_t b1)
{
	uint8_t *p0 = ft_collapsed_prefix_0(col, tier);
	uint8_t *p1 = ft_collapsed_prefix_1(col, tier);

	p0[e] = b0;
	p1[e] = b1;
}

/*
 * Subkey publication primitives.  The per-entry (suffix[7], len) block
 * occupies the first 8 bytes of a 16-byte aligned cds_ft_collapsed_entry
 * and is the source of truth for the subkey.
 *
 * 64-bit:  suffix + len fit in one 8-byte aligned word.  Writer does
 *          a single relaxed atomic 8-byte store; reader does a single
 *          relaxed atomic 8-byte load.  The slen=0 → slen!=0 transition
 *          is the safety property: slot can't expose a matchable
 *          intermediate state because the 8-byte aligned write is
 *          atomic.  No release-acquire pair needed on the subkey
 *          itself — rcu_dereference on the child pointer is the only
 *          synchronization required (it pairs with writer's release on
 *          child to publish liveness).
 *
 * 32-bit:  suffix + len span two 4-byte words.  Writer plain-stores
 *          the suffix bytes, then release-stores len to publish the
 *          subkey across word boundaries.  Reader acquire-loads len
 *          first; subsequent plain reads of suffix are synchronized
 *          via the len release-acquire pair.
 *
 * Endian-portability: the in-memory layout is suffix[0..6] then len at
 * byte 7.  Pack/unpack via memcpy through a uint8_t[8] staging array
 * to avoid host-endian assumptions.
 */
/*
 * Subkey word view: a uint8_t[8] / uint64_t union.  Field-by-field
 * access via .b[k] is endian-portable (byte at memory offset k is
 * always the same byte regardless of host endianness); the bit-
 * position of each byte within .v varies by endianness, but that
 * is irrelevant to our use because both writer and reader always
 * pack / unpack through the union, so values compare equal if and
 * only if the underlying byte arrays do.
 */
union ft_collapsed_subkey_word {
	uint8_t b[8];
	uint64_t v;
};

static inline
uint64_t ft_collapsed_subkey_pack(const uint8_t *suffix,
		unsigned int slen)
{
	union ft_collapsed_subkey_word u = { .v = 0 };
	unsigned int k;

	for (k = 0; k < slen; k++)
		u.b[k] = suffix[k];
	u.b[FT_COL_SUFFIX_MAX] = (uint8_t) slen;
	return u.v;
}

static inline_lookup
unsigned int ft_collapsed_subkey_unpack_slen(uint64_t val)
{
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
	/* LE: byte 7 sits at the high 8 bits of the uint64_t. */
	return (unsigned int) (val >> (FT_COL_SUFFIX_MAX * 8));
#elif __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
	/* BE: byte 7 sits at the low 8 bits of the uint64_t. */
	return (unsigned int) (val & 0xFFU);
#else
	union ft_collapsed_subkey_word u = { .v = val };

	return (unsigned int) u.b[FT_COL_SUFFIX_MAX];
#endif
}

/*
 * Compare an unpacked subkey value's suffix bytes [0..slen-1] against
 * @key bytes [0..slen-1].  Returns true on full match.
 *
 * 64-bit SWAR fast path: pack @key with the same byte layout as the
 * subkey (bytes 0..slen-1 from the source, bytes slen..6 zero, byte 7
 * holding slen), then a single uint64_t equality check covers all
 * suffix bytes plus the implicit slen byte at position 7.  The
 * packing is endian-portable via the subkey-word union, so the
 * comparison works on any host endianness as long as both sides use
 * the same packing function.
 */
static inline_lookup
bool ft_collapsed_subkey_match(uint64_t val, const uint8_t *key,
		unsigned int slen)
{
#if (CAA_BITS_PER_LONG >= 64)
	return val == ft_collapsed_subkey_pack(key, slen);
#else
	union ft_collapsed_subkey_word u = { .v = val };
	unsigned int k;

	for (k = 0; k < slen; k++) {
		if (u.b[k] != key[k])
			return false;
	}
	return true;
#endif
}

/*
 * Publish the subkey of a freshly-written entry.  Caller must already
 * have written the prefix-cache bytes; readers will verify against the
 * atomically-published subkey, catching any torn-prefix observation.
 *
 * Slots are append-only-no-reuse; the slot transitions exactly once
 * from slen=0 (cannot match) to slen=N (can match).
 *
 * The child pointer is published separately by ft_collapsed_publish_entry.
 */
static inline
void ft_collapsed_publish_subkey(struct cds_ft_collapsed_node *col,
		unsigned int tier, unsigned int e,
		const uint8_t *suffix, unsigned int slen)
{
	struct cds_ft_collapsed_entry *entries = ft_collapsed_entries(col, tier);

#if (CAA_BITS_PER_LONG >= 64)
	{
		uint64_t val = ft_collapsed_subkey_pack(suffix, slen);

		uatomic_store((uint64_t *) &entries[e], val, CMM_RELAXED);
	}
#else
	{
		unsigned int k;

		for (k = 0; k < slen; k++)
			entries[e].suffix[k] = suffix[k];
		uatomic_store(&entries[e].len, (uint8_t) slen, CMM_RELEASE);
	}
#endif
}

/*
 * Read-side subkey load.  Pairs with ft_collapsed_publish_subkey.
 * Returns a packed uint64_t with bytes 0..6 = suffix and byte 7 = len
 * regardless of host endianness.  Use ft_collapsed_subkey_unpack_slen
 * and ft_collapsed_subkey_match to interpret.
 */
static inline_lookup
uint64_t ft_collapsed_subkey_load(struct cds_ft_collapsed_node *col,
		unsigned int tier, unsigned int e)
{
	struct cds_ft_collapsed_entry *entries = ft_collapsed_entries(col, tier);

#if (CAA_BITS_PER_LONG >= 64)
	{
		/*
		 * Relaxed atomic 8-byte load.  No release-acquire pair on
		 * the subkey — the rcu_dereference on the child pointer
		 * provides the only synchronization needed.
		 */
		return uatomic_load((uint64_t *) &entries[e], CMM_RELAXED);
	}
#else
	{
		union ft_collapsed_subkey_word u = { .v = 0 };
		unsigned int slen, k;

		slen = uatomic_load(&entries[e].len, CMM_ACQUIRE);
		u.b[FT_COL_SUFFIX_MAX] = (uint8_t) slen;
		/*
		 * Acquire on len pairs with writer's release; suffix bytes
		 * are now safe to read via plain loads.
		 */
		for (k = 0; k < slen && k < FT_COL_SUFFIX_MAX; k++)
			u.b[k] = entries[e].suffix[k];
		return u.v;
	}
#endif
}

/*
 * Publish a freshly-written entry as live by release-storing its child
 * pointer.  Subkey (via ft_collapsed_publish_subkey) and prefix bytes
 * must be written before this call.  Pairs with the reader's
 * rcu_dereference on entry->child.
 */
static inline
void ft_collapsed_publish_entry(struct cds_ft_collapsed_node *col,
		unsigned int tier, unsigned int e,
		struct cds_ft_inode_flag *child)
{
	struct cds_ft_collapsed_entry *entries = ft_collapsed_entries(col, tier);

	uatomic_store(&entries[e].child, child, CMM_RELEASE);
}

/*
 * Mark a live entry dead.  Single CMM_RELEASE store of NULL into
 * the entry's child pointer.  The entry's (suffix, len) bytes
 * remain in place but are filtered out by every reader's
 * child-NULL check.  Slots are never reused.
 */
static inline
void ft_collapsed_kill_entry(struct cds_ft_collapsed_node *col,
		unsigned int tier, unsigned int e)
{
	struct cds_ft_collapsed_entry *entries = ft_collapsed_entries(col, tier);

	uatomic_store(&entries[e].child, NULL, CMM_RELAXED);
}

/*
 * Wide variants of the publication / load / kill helpers.
 *
 * The wide (suffix,len) span is 24 bytes — too large for the 64-bit
 * single-atomic-store trick used by the narrow path on 64-bit hosts.
 * Wide therefore always publishes via release on @len (mirroring the
 * narrow 32-bit path on every host): writer plain-stores suffix bytes
 * and then CMM_RELEASE-stores @len; reader CMM_ACQUIRE-loads @len and
 * then plain-reads suffix[0..len-1].  The slot transitions exactly
 * once from slen=0 (cannot match) to slen=N (can match); slen=0 acts
 * as the unpublished gate.
 *
 * Liveness publication via @child stays separate (CMM_RELEASE store
 * of the child pointer; reader rcu_dereference) — the same contract
 * as narrow.
 */
static inline
void ft_collapsed_wide_publish_subkey(struct cds_ft_collapsed_node *col,
		unsigned int tier, unsigned int e,
		const uint8_t *suffix, unsigned int slen)
{
	struct cds_ft_collapsed_entry_wide *entries = ft_collapsed_entries_wide(col, tier);
	unsigned int k;

	for (k = 0; k < slen; k++)
		entries[e].suffix[k] = suffix[k];
	uatomic_store(&entries[e].len, (uint8_t) slen, CMM_RELEASE);
}

/*
 * Read-side slen load for wide entries.  CMM_ACQUIRE pairs with the
 * writer's CMM_RELEASE on @len in ft_collapsed_wide_publish_subkey,
 * making subsequent plain reads of @suffix[0..slen-1] safe.  Returns
 * 0 for unpublished slots (caller must skip).
 */
static inline_lookup
unsigned int ft_collapsed_wide_load_slen(struct cds_ft_collapsed_node *col,
		unsigned int tier, unsigned int e)
{
	struct cds_ft_collapsed_entry_wide *entries = ft_collapsed_entries_wide(col, tier);

	return uatomic_load(&entries[e].len, CMM_ACQUIRE);
}

static inline
void ft_collapsed_wide_publish_entry(struct cds_ft_collapsed_node *col,
		unsigned int tier, unsigned int e,
		struct cds_ft_inode_flag *child)
{
	struct cds_ft_collapsed_entry_wide *entries = ft_collapsed_entries_wide(col, tier);

	uatomic_store(&entries[e].child, child, CMM_RELEASE);
}

static inline
void ft_collapsed_wide_kill_entry(struct cds_ft_collapsed_node *col,
		unsigned int tier, unsigned int e)
{
	struct cds_ft_collapsed_entry_wide *entries = ft_collapsed_entries_wide(col, tier);

	uatomic_store(&entries[e].child, NULL, CMM_RELAXED);
}

/*
 * Writer-side iteration over a wide collapsed node.  Walks every live
 * entry in lex order; body sees @e (slot index) and @entry
 * (cds_ft_collapsed_entry_wide *).  Dead slots (child == NULL) are
 * skipped.
 *
 * Wrap the body in `{ ... }` to avoid dangling-else hazards.
 */
#define ft_for_each_live_collapsed_entry_wide(_col, _tier, _e, _entry)	\
	for ((_e) = 0; (_e) < ft_collapsed_capacity_stride((_tier), FT_COL_STRIDE_WIDE); (_e)++)	\
		if (((_entry) = &ft_collapsed_entries_wide((_col), (_tier))[(_e)]),	\
		    ft_collapsed_wide_entry_dead((_entry)))		\
			continue;					\
		else

/*
 * Reader-side iteration over a wide collapsed node.  Per slot:
 *   - acquire-load @len via ft_collapsed_wide_load_slen, exposing it
 *     as @slen (slot is "unpublished" if slen == 0; skipped),
 *   - rcu_dereference the child pointer, exposing it as @child,
 *   - skip slots that are unpublished or dead (child == NULL).
 *
 * The body may safely read @entry->suffix[0..slen-1] (synchronized
 * via the acquire on @len in load_slen).  @entry->child must not be
 * re-read with a plain load — use @child.
 *
 * No packed @subkey uint64_t is exposed (unlike the narrow path used
 * inside ft_collapsed_load_entry_view_rcu): the 23-byte wide suffix
 * does not fit in a single SWAR word, so verification is byte-by-byte
 * against @entry->suffix.
 */
#define ft_for_each_live_collapsed_entry_wide_rcu(_col, _tier, _e, _entry, _slen, _child) \
	for ((_e) = 0; (_e) < ft_collapsed_capacity_stride((_tier), FT_COL_STRIDE_WIDE); (_e)++)	\
		if (((_entry) = &ft_collapsed_entries_wide((_col), (_tier))[(_e)]),	\
		    ((_slen) = ft_collapsed_wide_load_slen((_col), (_tier), (_e))),	\
		    ((_child) = ft_dereference_prefetch((_entry)->child)),	\
		    (_slen) == 0 || (_child) == NULL)			\
			continue;					\
		else

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
	 * Linear node data[] is the keys array; the allocator
	 * returns zeroed memory, which is the initial state expected
	 * by the sentinel-based count derivation in
	 * ft_linear_node_get_nr_child (values[0] == 0 with no other
	 * keys written matches the i=1 sentinel hit).  The
	 * pointer-array offset is derived from type->max_linear_child
	 * at lookup time.
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

/*
 * ft_node_readside_footprint: return the read-side memory footprint
 * of a traversable node, in 16-byte units.
 *
 * Skip-compressed nodes return 0: the reader never loads the
 * compressed node — the skip length is encoded in the pointer's
 * high bits, so traversal is free.
 *
 * For other types, returns the arena allocation size (1 << order)
 * in 16-byte units: (1 << order) / 16 = 1 << (order - 4).
 *
 * Write-side only (may chase parent pointers for skip resolution).
 */
static
unsigned int ft_node_readside_footprint(const struct cds_ft *ft,
		struct cds_ft_inode_flag *node_flag)
{
	unsigned int order;

	if (ft_node_compressed(node_flag)) {
		struct cds_ft_compressed_node *cn =
			ft_compressed_node_ptr(node_flag);
		if (ft_group_skip_compressed(ft->group) &&
		    cn->len <= FT_SKIP_LEN_MAX)
			return 0;
		order = ft_compressed_order(cn->len);
	} else if (ft_node_skip_compressed(node_flag)) {
		return 0;
	} else if (ft_node_collapsed(node_flag)) {
		order = cds_ft_item_order(
			(void *) ft_collapsed_node_ptr(node_flag));
	} else {
		/* Internal: linear, popcount, or pigeon. */
		order = ft_types[ft_node_type(node_flag)].order;
	}
	assert(order >= 4);
	return 1U << (order - 4);
}

/*
 * ft_node_readside_cl_pct: number of cache lines a read-side lookup
 * loads when traversing this node, returning a pct-scaled CL cost.
 *
 * Layout-sensitive nodes (linear, collapsed) compose their cost
 * from a scan term plus an average-pointer term:
 *
 *   cost_pct = avg_scan_cl × scan_mul_pct + avg_ptr_cl_pct
 *
 * avg_ptr_cl_pct is the percent-CL probability that the matched
 * entry's CL is *not* already loaded by the scan, computed under a
 * uniform distribution over slot positions.
 *
 *   Skip-encoded compressed   : 0  (resolved from pointer bits)
 *   Compressed alloc <= 64 B  : 1 × @compress_scan_mul_pct
 *   Compressed alloc >= 128 B : 2 × @compress_scan_mul_pct
 *   Linear                    : avg_scan_cl × 100 + avg_ptr_cl_pct
 *   Pigeon                    : 100  (1 fixed CL: matched slot only)
 *   Collapsed (per (stride, tier)):
 *     avg_scan_cl × @collapse_scan_mul_pct + ptr_cl_pct
 *
 * Linear layout:
 *
 *   keys_zone : bytes [0, max_lc)            (uint8_t keys)
 *   ptr_zone  : bytes [P, P + max_lc × sizeof(void *)),
 *                 P = roundup(max_lc, sizeof(void *))
 *
 * The SIMD/SWAR scan loads the keys_zone, touching ceil(max_lc/64)
 * CLs (every shipped ft_types[] entry keeps max_lc <= 28, so
 * avg_scan_cl = 1 today).  The matched pointer is at slot
 * s ∈ [0, max_lc); its CL is "free" iff it lies in the same CL as
 * the scan, i.e. P + s × sizeof(void *) < 64.  Under a uniform
 * distribution over s,
 *
 *   ptrs_in_first_CL = max(0, min(max_lc, (64 - P) / sizeof(void *)))
 *   avg_ptr_cl_pct   = (max_lc - ptrs_in_first_CL) / max_lc × 100
 *
 * Pigeon: no key scan; the matched pointer's CL is always a fresh
 * load.  avg_scan_cl = 0, avg_ptr_cl_pct = 100 → 100 pct CLs.
 *
 * Collapsed (per (stride, tier)) avg_ptr_cl_pct breakdown:
 *
 *     ptr_cl_pct is the per-tier-per-stride percent-CL probability
 *     that a matched entry's CL is *not* already loaded by the
 *     SIMD prefilter's header CL touch.
 *
 *     T0: header (16B) + entries (3 × 16B narrow / disabled wide)
 *         all fit in one 64B CL — ptr_cl_pct = 0 for any matched slot.
 *     T1 narrow (cap 7, 16B/entry): slots 0-2 are in the header CL
 *         (free); slots 3-6 in CL 1.  Avg = 4/7 ≈ 57%.
 *     T1 wide (cap 3, 32B/entry):  slot 0 is in the header CL (free),
 *         slot 1 straddles into CL 1, slot 2 in CL 1.  Avg = 2/3 ≈ 67%.
 *     T2/T3: 64B header = a full CL; matched entry is always in some
 *         later CL — ptr_cl_pct = 100 for both strides.
 *
 * Internals (linear, popcount, pigeon) use scan_mul_pct = 100
 * (the scan is just dependent loads, not a tunable scan loop).
 * Compressed and collapsed scans get their respective multiplier on
 * the scan portion, matching the candidate-side accounting in
 * ft_try_collapse_at_node and ft_compress_chain_at.  This keeps
 * absorbed-side and candidate-side cost comparisons symmetric: if a
 * scan-bearing node is on the absorbed path, it is priced the same
 * way as a candidate of the same type would be priced.
 *
 * Externals are leaves of every read path, paid the same on the
 * collapsed and absorbed sides — callers exclude them from the
 * latency comparison.
 */
static __attribute__((unused))
unsigned int ft_node_readside_cl_pct(const struct cds_ft *ft,
		struct cds_ft_inode_flag *node_flag,
		unsigned int collapse_scan_mul_pct,
		unsigned int compress_scan_mul_pct)
{
	/*
	 * Per-(stride, tier) read-side CL cost for the SoA collapsed
	 * layout.  See the function-header comment for the derivation
	 * of each ptr_cl_pct entry.
	 */
	static const unsigned int collapsed_avg_scan_cl_by_tier[FT_COL_NR_TIERS] = {
		[0] = 1,	/* 64B alloc — header + entries on 1 CL */
		[1] = 1,	/* 128B alloc — 16B header CL */
		[2] = 1,	/* 256B alloc — 64B header = 1 CL */
		[3] = 1,	/* 512B alloc — 64B header = 1 CL */
	};
	static const unsigned int collapsed_ptr_cl_pct_by_tier_stride
			[FT_COL_NR_STRIDES][FT_COL_NR_TIERS] = {
		[FT_COL_STRIDE_NARROW] = {
			[0] = 0,	/* shared CL with scan */
			[1] = 57,	/* 4/7 slots in CL 1 */
			[2] = 100,	/* matched entry always past header CL */
			[3] = 100,
		},
		[FT_COL_STRIDE_WIDE] = {
			[0] = 0,	/* wide-T0 disabled — value never queried */
			[1] = 67,	/* 2/3 slots reach CL 1 (slot 1 straddles) */
			[2] = 100,
			[3] = 100,
		},
	};
	unsigned int order;

	if (ft_node_compressed(node_flag)) {
		struct cds_ft_compressed_node *cn =
			ft_compressed_node_ptr(node_flag);
		unsigned int cl;

		if (ft_group_skip_compressed(ft->group) &&
		    cn->len <= FT_SKIP_LEN_MAX)
			return 0;
		order = ft_compressed_order(cn->len);
		cl = (order >= 7) ? 2U : 1U;
		return cl * compress_scan_mul_pct;
	} else if (ft_node_skip_compressed(node_flag)) {
		return 0;
	} else if (ft_node_collapsed(node_flag)) {
		unsigned int tier = ft_collapsed_tier(node_flag);
		unsigned int stride = ft_collapsed_stride(node_flag);

		return collapsed_avg_scan_cl_by_tier[tier]
				* collapse_scan_mul_pct
			+ collapsed_ptr_cl_pct_by_tier_stride[stride][tier];
	} else {
		unsigned int type_index = ft_node_type(node_flag);
		const struct cds_ft_type *type = &ft_types[type_index];
		unsigned int max_lc, p, by_space, ptrs_in_cl0, ptrs_out;
		unsigned int avg_scan_cl, avg_ptr_cl_pct;

		if (type->type_class == FT_PIGEON) {
			/*
			 * Direct-indexed; no scan loop.  Matched pointer's
			 * CL is always a fresh dependent load.
			 */
			return 100U;
		}

		/*
		 * Linear layout: keys followed by aligned pointer table.
		 */
		max_lc = type->max_linear_child;
		p = (max_lc + sizeof(void *) - 1U) & ~(sizeof(void *) - 1U);
		by_space = (p < 64U) ? (64U - p) / sizeof(void *) : 0U;
		ptrs_in_cl0 = (by_space < max_lc) ? by_space : max_lc;
		ptrs_out = max_lc - ptrs_in_cl0;
		avg_ptr_cl_pct = (ptrs_out * 100U) / max_lc;
		avg_scan_cl = (max_lc + 63U) / 64U;	/* always 1 today */
		return avg_scan_cl * 100U + avg_ptr_cl_pct;
	}
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

/*
 * Collapsed node configurations (SoA layout — see header for the
 * full per-tier byte map).
 *
 *   Tier  Alloc  Header  Entries  Capacity  Read-side CLs (typical)
 *   T0    64B    16B     48B      3          1 (header+entries share line)
 *   T1    128B   16B     112B     7          2 (header CL + 1 entry CL)
 *   T2    256B   64B     192B     12         2 (header CL + 1 entry CL)
 *   T3    512B   64B     448B     28         2 (header CL + 1 entry CL)
 *
 * Header holds two prefix-cache vectors (prefix_0[capacity] and
 * prefix_1[capacity], padded to a tier-dependent SIMD-load stride).
 * Entries are 16-byte fixed-stride records (suffix[7] | len | child).
 *
 * Capacity is implicit from the tier — there is no in-node count
 * byte.  Liveness is per-slot (entry.child == NULL marks a dead
 * slot).  Live entries appear in lexicographic order across slots.
 */

#ifdef FEATURE_FT_COLLAPSE
static
struct cds_ft_collapsed_node *alloc_collapsed_node(struct cds_ft *ft,
		unsigned int tier,
		struct cds_ft_metadata **_metadata)
{
	struct cds_ft_metadata *metadata;
	struct cds_ft_collapsed_node *col;
	unsigned int order;
	size_t alloc_sz;

	assert(tier < FT_COL_NR_TIERS);
	order = ft_col_tier_alloc_order[tier];
	alloc_sz = (size_t) 1U << order;

	metadata = cds_ft_alloc_item(ft, order, false);
	if (!metadata)
		return NULL;
	col = (struct cds_ft_collapsed_node *) cds_ft_metadata_to_item(metadata);
	/*
	 * Zero the entire allocation: prefix-cache bytes start clean,
	 * every entry is born-dead (suffix=0, len=0, child=NULL) so
	 * readers' SIMD-prefix matches against an unwritten slot are
	 * filtered by the child-NULL check.
	 */
	memset(col, 0, alloc_sz);
	if (ft_debug_counters()) {
		uatomic_inc(&ft->nr_nodes_allocated);
		uatomic_inc(&ft->nr_collapsed_alloc);
	}
	*_metadata = metadata;
	return col;
}
#endif /* FEATURE_FT_COLLAPSE */

static
void free_collapsed_node(struct cds_ft *ft,
		struct cds_ft_collapsed_node *node)
{
	struct cds_ft_metadata *metadata =
		cds_ft_item_to_metadata((struct cds_ft_inode *) node);

	FT_TP(collapsed_free, (const void *) ft_collapsed_node_flag(node));
	cds_ft_free_item(ft, metadata);
	if (ft_debug_counters() && node) {
		uatomic_inc(&ft->nr_nodes_freed);
		uatomic_inc(&ft->nr_collapsed_freed);
	}
}

/*
 * Immediate-free variant for speculative collapsed candidates that
 * never escape the writer's stack (rejected by gates inside
 * ft_try_collapse_at_node).  Bypasses call_rcu so the arena slot is
 * available to subsequent allocations on the same writer's path,
 * preventing the free-pending queue from growing under high mutation
 * rates with rejection-heavy gate settings.
 */
static __attribute__((unused))
void free_collapsed_node_unpublished(struct cds_ft *ft,
		struct cds_ft_collapsed_node *node)
{
	struct cds_ft_metadata *metadata =
		cds_ft_item_to_metadata((struct cds_ft_inode *) node);

	FT_TP(collapsed_free, (const void *) ft_collapsed_node_flag(node));
	cds_ft_free_item_unpublished(ft, metadata);
	if (ft_debug_counters() && node) {
		uatomic_inc(&ft->nr_nodes_freed);
		uatomic_inc(&ft->nr_collapsed_freed);
	}
}

/*
 * Maximum number of entries for a collapsed node at the given tier
 * and stride.  Equivalent to ft_collapsed_capacity_stride(); kept
 * under this name for call sites that read better when phrased as
 * a "capacity check".
 */
static inline
unsigned int ft_collapsed_max_entries(unsigned int tier, unsigned int stride)
{
	return ft_collapsed_capacity_stride(tier, stride);
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
 * distinctness).  The first position i >= 1 where values[i]
 * matches values[0] is therefore the count of slots that have
 * been written at some point -- the "touched count" that used
 * to live in data[0] low bits.
 *
 * Edge case: a fresh (never-inserted, calloc'd) node has
 * values[0] = values[1] = 0, so derive returns 1 rather than
 * 0.  Callers that specifically distinguish "empty" from
 * "one-touched" must use ft_linear_node_is_empty(), which
 * scans the pointer array for any non-NULL slot -- the
 * read-side equivalent of "no live children".
 */
static inline_lookup
uint8_t ft_linear_node_get_nr_child(const struct cds_ft_type *type,
		struct cds_ft_inode *node)
{
	uint8_t *values;
	uint8_t v0;
	unsigned int max_lc = type->max_linear_child;
	unsigned int i;

	values = &node->data[0];
	v0 = uatomic_load(&values[0], CMM_RELAXED);
	for (i = 1; i < max_lc; i++) {
		if (uatomic_load(&values[i], CMM_RELAXED) == v0)
			return (uint8_t)i;
	}
	return (uint8_t)max_lc;
}

/*
 * FT_POPCOUNT class nr_child: dispatch to the popcount_1l or
 * popcount_2l layout helper.  Both count populated slots via
 * popcount of the bitmap (independent of the soft-delete pointer
 * accounting in metadata->nr_child).  Stub in baseline builds (where
 * FT_POPCOUNT class types do not exist and the dispatcher case is
 * unreachable).
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
 * Derive the pointer array base from type->max_linear_child.
 * When @type is a compile-time-constant pointer (from the per-type
 * dispatcher), the offset folds to a literal.
 */
static inline_lookup
struct cds_ft_inode_flag **ft_linear_pointers(
		struct cds_ft_inode *node, const struct cds_ft_type *type)
{
	unsigned int byte_offset =
		FT_ALIGN(type->max_linear_child, sizeof(void *));
	return (struct cds_ft_inode_flag **)
		((uint8_t *) node + byte_offset);
}

/*
 * Read-side test for "this linear node has no live children" --
 * scan the pointer array for any non-NULL slot.  Avoids depending
 * on metadata->nr_child (write-side accounting) from the read
 * side.  O(max_linear_child) pointer loads, which for the only
 * expected caller (type[0] root empty-check) is one or three
 * depending on the tier.
 */
static inline_lookup
bool ft_linear_node_is_empty(const struct cds_ft_type *type,
		struct cds_ft_inode *node)
{
	struct cds_ft_inode_flag **pointers = ft_linear_pointers(node, type);
	unsigned int max_lc = type->max_linear_child;
	unsigned int i;

	for (i = 0; i < max_lc; i++) {
		if (uatomic_load(&pointers[i], CMM_RELAXED) != NULL)
			return false;
	}
	return true;
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
 * - FT_LINEAR (type 0, max_lc=3): values[] at offset 0.
 * - popcount_2l (types 1/2/3): bitmap header at offset 0.
 * - popcount_1l (type 4 in popcount build, plus the former
 *   POOL_A / POOL_B at indices 5/6 in popcount build): 32-byte
 *   bitmap header at offset 0.
 *
 * Encoding:
 *   non-popcount build: skip when internal bit AND type high bit
 *     (bit 3) both set, i.e. type_index >= 4 (LINEAR_256, POOL_A,
 *     POOL_B, PIGEON).  Type 4 (LINEAR_256) is included in the
 *     simple-mask skip; the prefetch win there is small enough
 *     that the cheaper test is preferred.
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
	unsigned long v = (unsigned long) ptr;

#ifdef FEATURE_FT_SKIP_COMPRESSED
	/* Clear skip-compressed length bits. */
	v = (v << FT_SKIP_LEN_BITS) >> FT_SKIP_LEN_BITS;
#endif
	__builtin_prefetch((const void *) v);
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
 * Stride-agnostic loaded-entry view.  Used by reader sites that need
 * to inspect the full suffix bytes (for ordered comparisons, ranged
 * scans, n-th selection, etc.) without specializing their bodies on
 * the per-stride struct types.
 *
 * @suffix is sized for FT_COL_W_SUFFIX_MAX (the maximum across both
 * strides); narrow loads populate the first FT_COL_SUFFIX_MAX bytes
 * (bytes 7..22 are uninitialized but @slen <= 7 keeps callers from
 * inspecting them).
 *
 * Populated by ft_collapsed_load_entry_view_rcu and consumed by the
 * ft_for_each_live_collapsed_entry_view_rcu iteration macro.
 *
 * Defined unconditionally so that reader sites which dispatch on
 * ft_node_collapsed() outside FEATURE_FT_COLLAPSE blocks still parse
 * (the dispatch is dead-coded by a constant-false ft_node_collapsed
 * when collapse is disabled).
 */
struct ft_col_view {
	uint8_t suffix[FT_COL_W_SUFFIX_MAX];
	unsigned int slen;		/* 0 = unpublished slot */
	struct cds_ft_inode_flag *child; /* NULL = dead slot */
};

#ifdef FEATURE_FT_COLLAPSE
/*
 * Load a slot's atomic snapshot into @view.  Stride dispatch:
 *
 *   Narrow: subkey 8B SWAR load; suffix bytes are the low 7 bytes
 *           of the packed word, copied byte-by-byte into @view.
 *
 *   Wide:   acquire-load @len, then plain reads of suffix bytes
 *           [0..slen-1] (synchronized via the acquire).
 *
 * For both strides @child is rcu_dereference'd (with prefetch).
 *
 * Callers must filter unpublished (slen == 0) and dead (child ==
 * NULL) slots — that is the job of the iteration macro below.
 */
static inline_lookup
void ft_collapsed_load_entry_view_rcu(struct cds_ft_collapsed_node *col,
		unsigned int tier, unsigned int stride, unsigned int e,
		struct ft_col_view *view)
{
	if (stride == FT_COL_STRIDE_NARROW) {
		struct cds_ft_collapsed_entry *entry =
				&ft_collapsed_entries(col, tier)[e];
		uint64_t subkey = ft_collapsed_subkey_load(col, tier, e);
		unsigned int slen = ft_collapsed_subkey_unpack_slen(subkey);
		union ft_collapsed_subkey_word u;
		unsigned int k;

		u.v = subkey;
		view->slen = slen;
		for (k = 0; k < FT_COL_SUFFIX_MAX; k++)
			view->suffix[k] = u.b[k];
		view->child = ft_dereference_prefetch(entry->child);
	} else {
		struct cds_ft_collapsed_entry_wide *entry =
				&ft_collapsed_entries_wide(col, tier)[e];
		unsigned int slen = ft_collapsed_wide_load_slen(col, tier, e);
		unsigned int k;

		view->slen = slen;
		for (k = 0; k < slen; k++)
			view->suffix[k] = entry->suffix[k];
		view->child = ft_dereference_prefetch(entry->child);
	}
}

/*
 * Stride-agnostic reader iteration macro.  Per slot:
 *   - load the entry view (suffix, slen, child) via the
 *     stride-aware loader;
 *   - skip slots that are unpublished (slen == 0) or dead
 *     (child == NULL);
 *   - body sees the @view structure; @suffix[0..slen-1] is
 *     valid, @child is the live pointer.
 *
 * @view must be a struct ft_col_view declared by the caller.
 *
 * Wrap the body in `{ ... }` to avoid dangling-else hazards.
 */
#define ft_for_each_live_collapsed_entry_view_rcu(_col, _tier, _stride, _e, _view)	\
	for ((_e) = 0; (_e) < ft_collapsed_capacity_stride((_tier), (_stride)); (_e)++)	\
		if ((ft_collapsed_load_entry_view_rcu((_col), (_tier), (_stride), (_e), &(_view)),	\
		     (_view).slen == 0 || (_view).child == NULL))	\
			continue;					\
		else

/*
 * Stride-aware reader-side candidate match.  Given a slot index @e
 * (typically yielded by the SIMD prefilter or the (0,0) bypass scan),
 * loads the slot's atomic snapshot, verifies the suffix bytes match
 * @key[0..slen-1], and returns the live child pointer.
 *
 * Narrow path (FT_COL_STRIDE_NARROW): SWAR pack/match via the 8-byte
 * subkey load; entry pointer not needed by the verify itself but is
 * still used for the child fetch.
 *
 * Wide path (FT_COL_STRIDE_WIDE): acquire-load @len, byte-by-byte
 * compare suffix[0..slen-1] against @key, then fetch @child.  No
 * SWAR — the 23-byte wide suffix does not fit in one uint64_t.
 *
 * Returns true on a live, suffix-matching slot whose @slen <=
 * @remaining_key, with *@slen_p and *@child_p set.  Returns false
 * for unpublished, dead, length-overflow, and suffix-mismatch slots.
 *
 * Defined here (after ft_dereference_prefetch) rather than alongside
 * the iteration macros so it can use the macro directly.
 */
static inline_lookup
bool ft_collapsed_match_candidate(struct cds_ft_collapsed_node *col,
		unsigned int tier, unsigned int stride, unsigned int e,
		const uint8_t *key, unsigned int remaining_key,
		unsigned int *slen_p, struct cds_ft_inode_flag **child_p,
		bool skip_verify)
{
	if (stride == FT_COL_STRIDE_NARROW) {
		struct cds_ft_collapsed_entry *entry =
				&ft_collapsed_entries(col, tier)[e];
		uint64_t subkey = ft_collapsed_subkey_load(col, tier, e);
		unsigned int slen = ft_collapsed_subkey_unpack_slen(subkey);
		struct cds_ft_inode_flag *child;

		if (slen == 0 || slen > remaining_key)
			return false;
		if (!skip_verify &&
		    !ft_collapsed_subkey_match(subkey, key, slen))
			return false;
		child = ft_dereference_prefetch(entry->child);
		if (child == NULL)
			return false;
		*slen_p = slen;
		*child_p = child;
		return true;
	} else {
		struct cds_ft_collapsed_entry_wide *entry =
				&ft_collapsed_entries_wide(col, tier)[e];
		unsigned int slen = ft_collapsed_wide_load_slen(col, tier, e);
		struct cds_ft_inode_flag *child;

		if (slen == 0 || slen > remaining_key)
			return false;
		if (!skip_verify) {
			unsigned int k;

			for (k = 0; k < slen; k++) {
				if (entry->suffix[k] != key[k])
					return false;
			}
		}
		child = ft_dereference_prefetch(entry->child);
		if (child == NULL)
			return false;
		*slen_p = slen;
		*child_p = child;
		return true;
	}
}
#else /* FEATURE_FT_COLLAPSE */

/*
 * Stubs for non-collapse builds.  These never execute (the calling
 * sites are guarded by ft_node_collapsed which evaluates to constant
 * false when FEATURE_FT_COLLAPSE is undefined), but the source must
 * still parse.  The macro expands to an empty for-loop body; the
 * stub functions are declared but never defined or called.
 */
#define ft_for_each_live_collapsed_entry_view_rcu(_col, _tier, _stride, _e, _view) \
	for ((_e) = 0, (void) (_col), (void) (_tier), (void) (_stride),	\
	     (void) (_view).slen; 0; (_e)++)

static inline_lookup
bool ft_collapsed_match_candidate(struct cds_ft_collapsed_node *col,
		unsigned int tier, unsigned int stride, unsigned int e,
		const uint8_t *key, unsigned int remaining_key,
		unsigned int *slen_p, struct cds_ft_inode_flag **child_p,
		bool skip_verify)
{
	(void) col; (void) tier; (void) stride; (void) e;
	(void) key; (void) remaining_key; (void) slen_p; (void) child_p;
	(void) skip_verify;
	return false;
}

#endif /* FEATURE_FT_COLLAPSE */

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
 *                      compressed / collapsed children, skips —
 *                      their own handlers prefetch cn->child /
 *                      col->data.
 *   FT_PF_BITMAP_META: same as META plus prefetches the bitmap
 *                      cache line for bitmap-bearing children
 *                      (used by ordered get_direction traversal).
 *
 * The item's alloc order is derived from the tag bits directly
 * (type_index + FT_ALLOC_ORDER_MIN) without loading ft_types[],
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
#ifdef FEATURE_FT_SKIP_COMPRESSED
	v = (v << FT_SKIP_LEN_BITS) >> FT_SKIP_LEN_BITS;
#endif
	if ((v & FT_INTERNAL_MASK) == 0) {
		/*
		 * External (bits 0-2 == 0): no FT metadata.  Prefetch the
		 * node body, which the META-hint caller typically reads
		 * next (user_data / ->next for the duplicate chain).
		 * Compressed / collapsed children: their handlers
		 * prefetch their own targets; skip here.
		 */
		if ((v & FT_TAG_MASK_WIDE) == 0)
			__builtin_prefetch((const void *) v);
		return;
	}
	{
		size_t order = ((v & FT_TYPE_MASK) >> FT_INTERNAL_BITS)
				+ FT_ALLOC_ORDER_MIN;
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
#ifdef FEATURE_FT_SKIP_COMPRESSED
	v = (v << FT_SKIP_LEN_BITS) >> FT_SKIP_LEN_BITS;
#endif
	if ((v & FT_INTERNAL_MASK) == 0) {
		if ((v & FT_TAG_MASK_WIDE) == 0)
			__builtin_prefetch((const void *) v);
		return;
	}
	{
		unsigned int type_index = (v & FT_TYPE_MASK) >> FT_INTERNAL_BITS;
		size_t order = type_index + FT_ALLOC_ORDER_MIN;
		unsigned long align_mask = ~((1UL << order) - 1UL);
		void *node = (void *) (v & align_mask);

		__builtin_prefetch(cds_ft_item_to_metadata_fast(node, order));
		/*
		 * Only pigeon attaches a 32B bitmap in metadata; popcount
		 * tiers carry their bitmaps inline in the node body.  Pigeon
		 * is the last type index on both arches (6 on 32-bit,
		 * 7 on 64-bit) so a single arch-conditional literal suffices.
		 */
#if (CAA_BITS_PER_LONG < 64)
		if (type_index == 6)
#else
		if (type_index == 7)
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
 * Adaptive lookup: dispatch bytewise, SWAR, or SIMD based on
 * the node type's max_linear_child.
 *
 * The thresholds define the minimum max_linear_child for each
 * strategy.  The key array starts at node->data[0]; SIMD/SWAR
 * scanners load from data+0 using unaligned loads.  A 16-byte SSE2
 * load covers 16 key entries and an 8-byte SWAR word covers 8.
 *
 * Nodes are aligned to their size (>=64 bytes), so a scan of
 * max_linear_child <= 32 bytes starting at data+0 stays within
 * a single cacheline by construction.
 *
 * Tunable via -DFT_SIMD_LINEAR_THRESHOLD=N and
 * -DFT_SWAR_LINEAR_THRESHOLD=N at compile time.
 */
/* FT_SIMD/SWAR_LINEAR_THRESHOLD defined near ft_types[]. */

/*
 * SWAR constants.  Pure bit-manipulation helpers below operate on
 * a word that the caller has already loaded -- no unaligned-access
 * dependency, so they live outside the UNALIGNED gate.
 */
#define L_ONES_A (-1UL / 255)
#define L_HIGHS_A (L_ONES_A * 0x80)

/* SWAR "byte == target" probe; returns a has_zero bitmap. */
static inline_lookup
unsigned long ft_swar_byteq(unsigned long word, unsigned long target_ones)
{
	unsigned long xor_res = word ^ target_ones;
	return (xor_res - L_ONES_A) & ~xor_res & L_HIGHS_A;
}

/* Position of the first matching byte within the word. */
static inline_lookup
unsigned int ft_swar_match_idx(unsigned long has_zero)
{
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
	return (unsigned int)(__builtin_ctzl(has_zero) >> 3);
#else
	return (unsigned int)(__builtin_clzl(has_zero) >> 3);
#endif
}

#if defined(FT_HAVE_EFFICIENT_UNALIGNED_ACCESS)

/*
 * Scan values[0..nr_child) for byte @n, using word-at-a-time SWAR
 * with an overlapping tail for nr_child >= sizeof(long), and a
 * single masked word for nr_child < sizeof(long).  The scan covers
 * exactly the valid key range, so every returned index is
 * guaranteed < nr_child.
 */
static inline_lookup
struct cds_ft_inode_flag *ft_linear_node_get_nth_swar(
		const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		struct cds_ft_inode_flag ***node_flag_ptr,
		uint8_t n, enum ft_pf_target pf_hint)
{
	uint8_t *values = &node->data[0];
	unsigned long target_ones = n * L_ONES_A;
	unsigned long word, has_zero;
	unsigned int i;
	const unsigned int word_sz = sizeof(unsigned long);
	const unsigned int max_lc = type->max_linear_child;

	if (max_lc < word_sz) {
		/*
		 * Single-word load: values[] is padded to
		 * sizeof(void *) >= word_sz on strict-align archs
		 * (gated by FT_HAVE_EFFICIENT_UNALIGNED_ACCESS),
		 * so this is always a safe in-node read.  The
		 * post-ctz bound `i < max_lc` rejects matches in
		 * pointer bytes that the load over-reads;  matches
		 * in sentinel / padding bytes (= values[0]) can
		 * only occur when target == values[0], in which
		 * case ctz picks position 0 first.  Deleted-slot
		 * matches within [0, max_lc) are rejected at the
		 * pointer dereference (NULL) by the caller.
		 */
		__builtin_memcpy(&word, values, word_sz);
		has_zero = ft_swar_byteq(word, target_ones);
		if (has_zero) {
			i = ft_swar_match_idx(has_zero);
			if (caa_likely(i < max_lc))
				goto found;
		}
	} else {
		unsigned int j = 0;

		while (j + word_sz <= max_lc) {
			__builtin_memcpy(&word, values + j, word_sz);
			has_zero = ft_swar_byteq(word, target_ones);
			if (has_zero) {
				i = j + ft_swar_match_idx(has_zero);
				goto found;
			}
			j += word_sz;
		}
		if (j < max_lc) {
			unsigned int tail = max_lc - word_sz;

			__builtin_memcpy(&word, values + tail, word_sz);
			has_zero = ft_swar_byteq(word, target_ones);
			if (has_zero) {
				i = tail + ft_swar_match_idx(has_zero);
				goto found;
			}
		}
	}
	if (caa_unlikely(node_flag_ptr))
		*node_flag_ptr = NULL;
	return NULL;
found:
	{
		struct cds_ft_inode_flag **pointers =
			ft_linear_pointers(node, type);
		if (caa_unlikely(node_flag_ptr))
			*node_flag_ptr = &pointers[i];
		return ft_dereference_acquire_prefetch_hint(pointers[i], pf_hint);
	}
}

#if defined(__SSE2__)
/*
 * Scan values[0..nr_child) for byte @n using SSE2 + overlapping
 * 16-byte tail.  For nr_child <= 16, reject matches in the
 * trailing garbage slots with `phys_idx < nr_child` (garbage
 * matches are rare, typically < 10/256 per call, so the
 * predicted-taken branch on real matches is cheaper than
 * unconditionally masking movemask).  For nr_child > 16, the
 * second load overlaps with the first so every bit inspected
 * corresponds to an index in [0, nr_child).
 */
static inline_lookup
struct cds_ft_inode_flag *ft_linear_node_get_nth_simd(
		const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		struct cds_ft_inode_flag ***node_flag_ptr,
		uint8_t n, enum ft_pf_target pf_hint)
{
	uint8_t *values = &node->data[0];
	const unsigned int max_lc = type->max_linear_child;
	__m128i target = _mm_set1_epi8(n);
	__m128i chunk;
	unsigned int mask, phys_idx;

	chunk = _mm_loadu_si128((__m128i *)values);
	mask = _mm_movemask_epi8(_mm_cmpeq_epi8(chunk, target));
	if (max_lc <= 16) {
		if (mask) {
			phys_idx = __builtin_ctz(mask);
			if (caa_likely(phys_idx < max_lc))
				goto found;
		}
	} else {
		unsigned int tail;

		if (mask) {
			phys_idx = __builtin_ctz(mask);
			goto found;
		}
		tail = max_lc - 16;
		chunk = _mm_loadu_si128((__m128i *)(values + tail));
		mask = _mm_movemask_epi8(_mm_cmpeq_epi8(chunk, target));
		if (mask) {
			phys_idx = tail + __builtin_ctz(mask);
			goto found;
		}
	}
	if (caa_unlikely(node_flag_ptr))
		*node_flag_ptr = NULL;
	return NULL;
found:
	{
		struct cds_ft_inode_flag **pointers =
			ft_linear_pointers(node, type);
		if (caa_unlikely(node_flag_ptr))
			*node_flag_ptr = &pointers[phys_idx];
		return ft_dereference_acquire_prefetch_hint(pointers[phys_idx], pf_hint);
	}
}
#endif /* __SSE2__ */

#endif /* FT_HAVE_EFFICIENT_UNALIGNED_ACCESS */

/*
 * FT_USE_SPECIALIZED_SCAN: 64-bit tier-2 layout is power-of-two in
 * ptr_offset per type_index:
 *   type_index 0..2: ptr_offset=8   (SWAR 8-byte)
 *   type_index 3:    ptr_offset=16  (SIMD 16-byte)
 *   type_index 4:    ptr_offset=32  (SIMD 32-byte)
 *   type_index 5..6: popcount_1l, ptr_offset=32  (32B bitmap header)
 *   type_index 7:    pigeon (dense)
 *
 * Each scanner uses a compile-time-constant scan width equal to the
 * ptr_offset.  Scanning the whole values+padding region is safe: real
 * slots hold values distinct from values[0] (linear distinctness
 * invariant), padding bytes all equal values[0] (first-insert memset
 * or post-copy sweep), so a match at a padding byte can only occur
 * when target == values[0] -- and ctz picks position 0 first because
 * values[0] itself matches.  No post-check is needed.
 *
 * 32-bit tier has non-power-of-two ptr_offsets, so this optimization
 * is gated off there; the generic type-parameterized scanner handles
 * that tier.
 */
#if CAA_BITS_PER_LONG >= 64 && defined(__SSE2__)
#define FT_USE_SPECIALIZED_SCAN

/*
 * scan_1: bytewise scan for the single-slot type (type_index 0,
 * max_linear_child=1 on 64-bit tier-2).  At one slot the SWAR
 * dependency chain (imul / xor / sub / and / and / tzcnt / load)
 * costs more than a single compare-and-load.  ptr_offset is 8.
 */
static inline_lookup
struct cds_ft_inode_flag *ft_linear_scan_1(
		struct cds_ft_inode *node,
		struct cds_ft_inode_flag ***node_flag_ptr,
		uint8_t n, enum ft_pf_target pf_hint)
{
	uint8_t *values = &node->data[0];
	struct cds_ft_inode_flag **pointers;

	if (values[0] != n) {
		if (caa_unlikely(node_flag_ptr))
			*node_flag_ptr = NULL;
		return NULL;
	}
	pointers = (struct cds_ft_inode_flag **) ((uint8_t *) node + 8);
	if (caa_unlikely(node_flag_ptr))
		*node_flag_ptr = &pointers[0];
	return ft_dereference_acquire_prefetch_hint(pointers[0], pf_hint);
}

/*
 * scan_3: SSE2 cmpeq + movemask + ctz over the 3-slot type (type_index
 * 1, max_linear_child=3 on 64-bit tier-2).  ptr_offset is 8.
 *
 * The earlier scalar implementation chained three independent
 * load+compare pairs and relied on the front end pipelining them in
 * parallel.  That shape is fastest under predictable input but
 * generates one taken/untaken decision per slot, so on workloads with
 * unpredictable slot occupancy (real DNS keys hitting any of the three
 * slots with similar probability) the per-slot mispredictions
 * dominate.  perf record on cds_ft_lookup_key shows the first byte's
 * je was the single largest branch-miss site.
 *
 * The SIMD form replaces three branches with one masked movemask + ctz
 * + a single not-found branch.  The 8-byte _mm_loadl_epi64 reads the
 * 3 keys plus 5 bytes of inter-slot padding; the static mask & 0x7
 * clears any matches in the padding lanes so the result is always
 * within [0, 2].  scan_3 is the dominant dispatch (46% of dispatches
 * in bench_comprehensive), so reducing its branch-miss footprint
 * lowers the lookup-path branch-miss budget materially for
 * candidate-mode descents.
 */
static inline_lookup
struct cds_ft_inode_flag *ft_linear_scan_3(
		struct cds_ft_inode *node,
		struct cds_ft_inode_flag ***node_flag_ptr,
		uint8_t n, enum ft_pf_target pf_hint)
{
	uint8_t *values = &node->data[0];
	__m128i target = _mm_set1_epi8((char) n);
	__m128i chunk = _mm_loadl_epi64((const __m128i *) values);
	unsigned int mask = (unsigned int) _mm_movemask_epi8(
			_mm_cmpeq_epi8(chunk, target)) & 0x7U;
	struct cds_ft_inode_flag **pointers;
	unsigned int i;

	if (!mask) {
		if (caa_unlikely(node_flag_ptr))
			*node_flag_ptr = NULL;
		return NULL;
	}
	i = (unsigned int) __builtin_ctz(mask);
	pointers = (struct cds_ft_inode_flag **) ((uint8_t *) node + 8);
	if (caa_unlikely(node_flag_ptr))
		*node_flag_ptr = &pointers[i];
	return ft_dereference_acquire_prefetch_hint(pointers[i], pf_hint);
}

/*
 * Runtime validation of ft_types[] ptr_offset layout assumed by the
 * specialized scanners.  Called from cds_ft_group_create at init.
 * Not a static assert because ft_types[].max_linear_child is a
 * struct member read, which gcc doesn't treat as an integer constant
 * expression.
 */
static inline __attribute__((unused))
void ft_specialized_scan_layout_assert(void)
{
	assert(FT_ALIGN(ft_types[0].max_linear_child, sizeof(void *)) == 8);
	assert(FT_ALIGN(ft_types[1].max_linear_child, sizeof(void *)) == 8);
	assert(FT_ALIGN(ft_types[2].max_linear_child, sizeof(void *)) == 8);
	assert(FT_ALIGN(ft_types[3].max_linear_child, sizeof(void *)) == 16);
	assert(FT_ALIGN(ft_types[4].max_linear_child, sizeof(void *)) == 32);
	assert(ft_types[5].popcount_1l);
	assert(ft_types[6].popcount_1l);
	assert(ft_types[7].type_class == FT_PIGEON);
}

#endif /* FT_USE_SPECIALIZED_SCAN */

/*
 * Generic linear node scanner used by the non-specialized dispatch
 * fallback (32-bit, strict-alignment architectures, or when
 * FT_USE_SPECIALIZED_SCAN is unavailable).  Switches between bytewise,
 * SWAR, and SIMD based on max_linear_child; the branch is on a
 * compile-time-constant type->max_linear_child from the dispatcher's
 * &ft_types[N], so the compiler prunes the dead arm at each call
 * site.
 */
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

static inline_lookup
struct cds_ft_inode_flag *ft_linear_node_get_nth(const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		struct cds_ft_inode_flag ***node_flag_ptr,
		uint8_t n, enum ft_pf_target pf_hint)
{
	assert(ft_type_is_linear(type->type_class));
#if defined(FT_HAVE_EFFICIENT_UNALIGNED_ACCESS) && defined(__SSE2__)
	/*
	 * Types with max_linear_child <= sizeof(unsigned long) have a
	 * values+padding region (ptr_offset) that fits in one SWAR
	 * word; a 16-byte SIMD load would over-read into the pointer
	 * array.  SWAR handles them; SIMD handles the rest.
	 */
	if (type->max_linear_child <= sizeof(unsigned long))
		return ft_linear_node_get_nth_swar(type, node, node_flag_ptr, n, pf_hint);
	return ft_linear_node_get_nth_simd(type, node, node_flag_ptr, n, pf_hint);
#elif defined(FT_HAVE_EFFICIENT_UNALIGNED_ACCESS)
	return ft_linear_node_get_nth_swar(type, node, node_flag_ptr, n, pf_hint);
#else
	{
		uint8_t *values = &node->data[0];
		const unsigned int max_lc = type->max_linear_child;
		unsigned int i;

		for (i = 0; i < max_lc; i++) {
			if (uatomic_load(&values[i], CMM_RELAXED) == n)
				break;
		}
		if (i >= max_lc) {
			if (caa_unlikely(node_flag_ptr))
				*node_flag_ptr = NULL;
			return NULL;
		}
		{
			struct cds_ft_inode_flag **pointers =
				ft_linear_pointers(node, type);
			if (caa_unlikely(node_flag_ptr))
				*node_flag_ptr = &pointers[i];
			return ft_dereference_acquire_prefetch_hint(pointers[i], pf_hint);
		}
	}
#endif
}

/*
 * FT_POPCOUNT class get_nth: dispatch to popcount_1l or
 * popcount_2l layout helper.  Reached via the generic
 * fallback dispatch on non-x86 64-bit (where FT_USE_SPECIALIZED_SCAN
 * is inactive); on x86-64 the specialized switch in
 * ft_node_get_nth_skip routes by type_index directly.  Stub in
 * baseline builds.
 */
static inline_lookup
struct cds_ft_inode_flag *ft_popcount_node_get_nth(const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		struct cds_ft_inode_flag ***node_flag_ptr,
		uint8_t n, enum ft_pf_target pf_hint)
{
	unsigned int max_lc = type->max_linear_child;

	assert(ft_type_is_popcount(type->type_class));
	if (type->popcount_2l) {
		/*
		 * Dispatch by max_lc to the layout-matching scanner.  Each
		 * scanner enforces a slot cap based on its bitmap geometry:
		 *
		 *   max_lc=3   scan_16_16_max_3  generic 2L (4+4) u16 root,
		 *                                u16 sub_bm[], hdr 8 B, ptr@8.
		 *                                Slot cap = 3 (sub_bm[3]).
		 *   max_lc=5   scan_16_16_max_5  generic 2L (4+4) u16 root,
		 *                                u16 sub_bm[], hdr 12 B, ptr@12
		 *                                (32-bit). Slot cap = 5.
		 *   max_lc=6   scan_32_8         flat 5+3 u32 root, u8 nibbles
		 *                                in u64 packed_bms, hdr 12 B
		 *                                + 4 B pad, ptr@16 (64-bit).
		 *                                Slot cap = 8 (packed_bms).
		 *   max_lc=12  scan_64_4         flat 6+2 u64 root, u4 nibbles
		 *                                in u64 packed_bms, hdr 16 B,
		 *                                ptr@16 (32-bit; flat 6+2 in
		 *                                64 B order-6 node).
		 *                                Slot cap = 16.
		 *   max_lc=14  scan_64_4         flat 6+2, 64-bit only.
		 *                                Slot cap = 16.
		 *   max_lc=16  scan_64_4         flat 6+2 reused in 128 B
		 *                                order-7 node (32-bit). Bitmap
		 *                                cap 16 is binding; 48 B
		 *                                trailing slack.
		 */
		switch (max_lc) {
		case 3:
			return ft_popcount_2l_scan_16_16_max_3(node, node_flag_ptr, n, pf_hint);
		case 5:
			return ft_popcount_2l_scan_16_16_max_5(node, node_flag_ptr, n, pf_hint);
		case 6:
			return ft_popcount_2l_scan_32_8(node, node_flag_ptr, n, pf_hint);
		case 12:
		case 14:
		case 16:
			return ft_popcount_2l_scan_64_4(node, node_flag_ptr, n, pf_hint);
		default:
			assert(0);
			return NULL;
		}
	}
	assert(type->popcount_1l);
	return ft_popcount_1l_scan_28(node, node_flag_ptr, n, pf_hint);
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

	assert(ft_type_is_linear(type->type_class));
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

	values = &node->data[0];
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

/*
 * FT_POPCOUNT class get_direction: dispatch to popcount_1l or
 * popcount_2l layout helper.  Stub in baseline builds (where
 * FT_POPCOUNT class types do not exist and the dispatcher case is
 * unreachable).
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

static inline_lookup
void ft_linear_node_get_ith_pos(const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		uint8_t i,
		uint8_t *v,
		struct cds_ft_inode_flag **iter)
{
	uint8_t *values;
	struct cds_ft_inode_flag **pointers;

	assert(ft_type_is_linear(type->type_class));
	assert(i < ft_linear_node_get_nr_child(type, node));

	values = &node->data[0];
	*v = values[i];
	pointers = (struct cds_ft_inode_flag **) align_ptr_size(&values[type->max_linear_child]);
	*iter = ft_dereference_acquire(pointers[i]);
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

	assert(!type || type->type_class == FT_PIGEON);
	child_node_flag_ptr = &((struct cds_ft_inode_flag **) node->data)[n];
	child_node_flag = ft_dereference_acquire_prefetch_hint(*child_node_flag_ptr, pf_hint);
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

#define FT_MASK_LINEAR      FT_MASK_CLASS(FT_LINEAR)
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
	assert(ft_types[4].type_class == FT_POPCOUNT && ft_types[4].popcount_1l);
	assert(ft_types[5].type_class == FT_POPCOUNT && ft_types[5].popcount_1l);
#else
	assert(ft_types[5].type_class == FT_POPCOUNT && ft_types[5].popcount_1l);
	assert(ft_types[6].type_class == FT_POPCOUNT && ft_types[6].popcount_1l);
#endif
}

static inline_lookup
/*
 * ft_node_get_nth_skip: raw child slot access.  Returns the slot
 * value as-is, including skip-compressed pointers.  Used only by
 * candidate lookup which resolves skip pointers itself.
 */
struct cds_ft_inode_flag *ft_node_get_nth_skip(struct cds_ft_inode_flag *node_flag,
		struct cds_ft_inode_flag ***node_flag_ptr,
		uint8_t n, enum ft_pf_target pf_hint)
{
	unsigned long tag = (unsigned long) node_flag & 0xF;
	struct cds_ft_inode *node;
	unsigned int type_index;

	/* External / compressed / collapsed: internal flag clear. */
	if (caa_unlikely(!(tag & FT_INTERNAL_MASK))) {
		if (caa_unlikely(node_flag_ptr))
			*node_flag_ptr = NULL;
		return NULL;
	}

	node = ft_node_ptr_internal(node_flag);
	type_index = (tag >> FT_INTERNAL_BITS) & 0x7;

#ifdef FT_USE_SPECIALIZED_SCAN
	/*
	 * Per-type dispatch on type_index (no ft_types[] field load).
	 *
	 * 64-bit tier-2 layout (validated at cds_ft_group_create time):
	 *   0:  max_lc=1,  ptr_offset=8   -> scan_1 (bytewise)
	 *   1:  max_lc=3,  ptr_offset=8   -> scan_3 (bytewise unrolled)
	 *   2:  max_lc=7,  ptr_offset=8   -> scan_8 (SWAR)
	 *   3:  max_lc=14, ptr_offset=16  -> scan_16 (SSE2)
	 *   4:  max_lc=28, ptr_offset=32  -> scan_32 (dual-SSE2)
	 *   5:  POOL_A (1D)               -> pool_scan_1d
	 *   6:  POOL_B (2D)               -> pool_scan_2d
	 *   7:  PIGEON                    -> pigeon
	 *
	 * Hybrid shape: caa_likely gate on the two most frequent linear
	 * types, switch/jump-table on the rest.
	 *
	 * Measured distribution over bench_comprehensive
	 * (u32d/u32s/u64d/u64s/dns/dict/paths, ~1.2B total dispatches):
	 *   type_index  1 (scan_3)   46.4%  <-- dominant
	 *   type_index  0 (scan_1)   21.6%
	 *   type_index  2 (scan_8)   16.8%
	 *   type_index  3 (scan_16)   9.5%
	 *   type_index  7 (pigeon)    4.3%
	 *   type_index  4 (scan_32)   1.2%
	 *   type_index 5/6 (pools)   <0.2%
	 *
	 * Types 1 and 0 cover 68% of dispatches.  Short-circuiting them
	 * with explicit predicted-not-taken branches keeps the hot path
	 * to 1-2 branches, and lets the compiler emit a jump-table for
	 * the remaining six cases.
	 */
	if (caa_likely(type_index == 1)) {
		return ft_popcount_2l_scan_16_16_max_3(node, node_flag_ptr, n, pf_hint);
	}
	/*
	 * Conditional on "not type 1", type 0 is the mode of the
	 * remaining distribution (~40% of arrivals at this point).
	 * caa_likely arranges scan_1 on the fall-through for better
	 * icache locality; even when the runtime target isn't exactly
	 * type 0, the layout cost is a single jump for the other paths.
	 */
	if (caa_likely(type_index == 0))
		return ft_linear_scan_1(node, node_flag_ptr, n, pf_hint);
	switch (type_index) {
	case 2:
		return ft_popcount_2l_scan_32_8(node, node_flag_ptr, n, pf_hint);
	case 3:
		return ft_popcount_2l_scan_64_4(node, node_flag_ptr, n, pf_hint);
	case 4:
		return ft_popcount_1l_scan_28(node, node_flag_ptr, n, pf_hint);
	case 5:
		/* popcount_1l 512B: 256-bit bitmap + 54-pointer table. */
		return ft_popcount_1l_scan_28(node, node_flag_ptr, n, pf_hint);
	case 6:
		/* popcount_1l 1024B: 256-bit bitmap + 104-pointer table. */
		return ft_popcount_1l_scan_28(node, node_flag_ptr, n, pf_hint);
	case 7:
		return ft_pigeon_node_get_nth(NULL, node, node_flag_ptr, n, pf_hint);
	default:
		/*
		 * type_index is a 3-bit field masked from tag above, so
		 * values outside 0..7 cannot occur.  Telling the compiler
		 * lets it drop the switch-table bound check.
		 */
		__builtin_unreachable();
	}
#else
	{
	unsigned int bit = 1U << type_index;
	/*
	 * Linear (bytewise scan) is the most common type in
	 * byte-indexed tries -- predicted-taken fast path.  FT_POPCOUNT
	 * (32/64/256-bit popcount layouts) is dispatched separately so
	 * the helper does not re-discover the class identity that the
	 * type_index already carries.  Pigeon is the catch-all for
	 * type_index values that match neither linear nor popcount.
	 */
	if (caa_likely(bit & FT_MASK_LINEAR))
		return ft_linear_node_get_nth(&ft_types[type_index], node,
				node_flag_ptr, n, pf_hint);
	if (bit & FT_MASK_POPCOUNT)
		return ft_popcount_node_get_nth(&ft_types[type_index], node,
				node_flag_ptr, n, pf_hint);
	return ft_pigeon_node_get_nth(NULL, node, node_flag_ptr, n, pf_hint);
	}
#endif
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
	case FT_LINEAR:
	{
		uint8_t nr_child = ft_linear_node_get_nr_child(type, node);
		unsigned int i;

		for (i = 0; i < nr_child; i++) {
			struct cds_ft_inode_flag *iter;
			uint8_t v;

			ft_linear_node_get_ith_pos(type, node, i, &v, &iter);
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
	case FT_POPCOUNT:
		child = ft_popcount_node_get_direction(type, node, n, result_key, dir);
		break;
	case FT_LINEAR:
		child = ft_linear_node_get_direction(type, node, n, result_key, dir);
		break;
	case FT_PIGEON:
		child = ft_pigeon_node_get_direction(type, node, n, result_key, dir);
		break;
	default:
		assert(0);
		return (void *) -1UL;
	}
	child = ft_resolve_skip_compressed(child);
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
int ft_linear_node_set_nth(const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		struct cds_ft_metadata *metadata,
		uint8_t n,
		struct cds_ft_inode_flag *child_node_flag,
		bool *_replace_old_ptr,
		bool is_init)
{
	uint8_t nr_child;
	uint8_t *values;
	struct cds_ft_inode_flag **pointers;
	unsigned int i, unused = 0;
	bool replace_old_ptr = false;

	assert(ft_type_is_linear(type->type_class));

	values = &node->data[0];
	pointers = (struct cds_ft_inode_flag **) align_ptr_size(&values[type->max_linear_child]);

	/*
	 * is_init: caller guarantees this is the first set_nth on a
	 * freshly-allocated (unpublished) (sub)node.  No concurrent
	 * readers can observe the node yet, so the bulk memset and
	 * pointer store are race-free -- no release needed.  The
	 * publication of this node to its parent (rcu_assign_pointer
	 * on the parent slot) provides the single release barrier
	 * that makes all these stores visible to readers.  Establishes
	 * the padding = values[0] = n invariant; slot 0 is adopted for
	 * the first inserted byte.
	 */
	if (is_init) {
		assert(pointers[0] == NULL);
		memset(values, n, (uint8_t *)pointers - values);
		pointers[0] = child_node_flag;
		metadata->nr_child++;
		if (_replace_old_ptr)
			*_replace_old_ptr = false;
		return 0;
	}

	/*
	 * Non-init path: node is touched (post-init or post-recompact).
	 * Derive slot count (monotonic, grows on append, never shrinks
	 * on delete) from the sentinel scan.  metadata->nr_child (live
	 * count) diverges from slot count after deletes.
	 */
	nr_child = ft_linear_node_get_nr_child(type, node);
	assert(nr_child <= type->max_linear_child);

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
		uatomic_store(&values[nr_child], n, CMM_RELAXED);
		/*
		 * Release on the pointer: values above happen-before the
		 * reader's acquire on pointers[i].  Readers derive
		 * nr_child from values[] via sentinel scan, so the
		 * pointer release is the sole synchronization surface.
		 */
		rcu_assign_pointer(pointers[i], child_node_flag);
	} else {
		/* Replacing a NULL or external node pointer. */
		rcu_assign_pointer(pointers[i], child_node_flag);
	}
	if (!replace_old_ptr)
		metadata->nr_child++;
	dbg_printf("linear set nth: %u child, metadata: %u child, for node %p\n",
		(unsigned int) ft_linear_node_get_nr_child(type, node),
		(unsigned int) metadata->nr_child,
		node);
	if (_replace_old_ptr)
		*_replace_old_ptr = replace_old_ptr;
	return 0;
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
	case FT_LINEAR:
		ret = ft_linear_node_set_nth(type, node, metadata, n, child_node_flag, NULL, is_init);
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

static
int ft_linear_node_replace_ptr(const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		struct cds_ft_metadata *metadata,
		struct cds_ft_inode_flag **node_flag_ptr,
		struct cds_ft_inode_flag *newptr)
{
	assert(ft_type_is_linear(type->type_class));
	assert(ft_linear_node_get_nr_child(type, node) <= type->max_linear_child);

	if (ft_type_is_linear(type->type_class) && !newptr) {
		assert(!metadata->fallback_removal_count);
		if (metadata->nr_child <= type->min_child) {
			/* We need to try recompacting the node */
			return -EFBIG;
		}
	}
	dbg_printf("linear replace ptr: node %p\n", node);
	assert(*node_flag_ptr != NULL);
	rcu_assign_pointer(*node_flag_ptr, newptr);
	/*
	 * Value is never changed (would cause ABA issue).  Instead,
	 * we leave the pointer to NULL and recompact the node once
	 * in a while.  It is allowed to set a NULL pointer to a new
	 * value without recompaction though.  Only update the
	 * metadata node accounting.
	 */
	if (!newptr)
		metadata->nr_child--;
	dbg_printf("linear replace ptr: %u child, metadata: %u child, for node %p newptr %p\n",
		(unsigned int) ft_linear_node_get_nr_child(type, node),
		(unsigned int) metadata->nr_child,
		node, newptr);
	return 0;
}

/*
 * FT_POPCOUNT class replace_ptr: same publish protocol as the linear
 * replace (rcu_assign_pointer on the slot), with min_child gating on
 * delete and nr_child accounting via the bitmap-popcount helper.
 * Stub in baseline builds (where FT_POPCOUNT class types do not
 * exist and the dispatcher case is unreachable).
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
	case FT_POPCOUNT:
		ret = ft_popcount_node_replace_ptr(type, node, metadata, node_flag_ptr, newptr);
		break;
	case FT_LINEAR:
		ret = ft_linear_node_replace_ptr(type, node, metadata, node_flag_ptr, newptr);
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
		bool is_root,
		unsigned int node_depth)
{
	unsigned int new_type_index;
	struct cds_ft_inode *new_node;
	struct cds_ft_metadata *new_metadata;
	const struct cds_ft_type *new_type;
	struct cds_ft_inode_flag *new_node_flag = NULL;
	int ret;
	int fallback = 0;
	/*
	 * Track which (sub)nodes within new_node have received their
	 * first child via is_init=true.  For FT_LINEAR:
	 * `new_linear_init_done` is the single flag.
	 */
	bool new_linear_init_done = false;

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
			unsigned int di;

			new_metadata->parent = metadata->parent;
#ifdef FEATURE_FT_SKIP_COMPRESSED
			new_metadata->skip_slot_offset = metadata->skip_slot_offset;
#endif
			new_metadata->fallback_removal_count = metadata->fallback_removal_count;
			ft_metadata_set_external_nodes(new_node_flag,
				new_metadata, metadata->external_nodes);
			/*
			 * Copy density counters before nr_keys: nr_keys
			 * is the compact/extended discriminator and
			 * ft_density_set may promote to extended, so the
			 * values must be written first.
			 */
			for (di = 0; di < FT_NODE_DENSITY_DEPTH; di++)
				ft_density_set(ft, new_metadata, di,
					ft_density_get(metadata, di));
			ft_nr_keys_store(ft, new_metadata,
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
	switch (new_type->type_class) {					\
	/*								\
	 * FT_POPCOUNT and FT_LINEAR share the single-init-flag	\
	 * scheme (no per-subnode state); fall-through is		\
	 * intentional and the bodies are exact copies.		\
	 */								\
	case FT_POPCOUNT:						\
	case FT_LINEAR:							\
		__is_init = !new_linear_init_done;			\
		new_linear_init_done = true;				\
		break;							\
	default:							\
		break;  /* FT_PIGEON, FT_NULL: is_init irrelevant */	\
	}								\
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
	case FT_LINEAR:
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
		case FT_LINEAR:
		{
			uint8_t nc = ft_linear_node_get_nr_child(new_type,
					new_node);
			unsigned int i;

			for (i = 0; i < nc; i++) {
				struct cds_ft_inode_flag *iter;
				struct cds_ft_inode_flag **slot = NULL;
				uint8_t v;

				ft_linear_node_get_ith_pos(new_type,
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

	/*
	 * Save old flag before overwriting: the parent (e.g. a collapsed
	 * node) still holds this value in its child slots, so
	 * ft_parent_depth_span needs it to find the entry.
	 */
	{
		struct cds_ft_inode_flag *old_flag = *old_node_flag_ptr;

		FT_TP(node_recompact, (const void *) old_flag,
			(const void *) new_node_flag, (int) new_type_index);

		/* Return pointer to new recompacted node through old_node_flag_ptr */
		*old_node_flag_ptr = new_node_flag;
		if (old_node && old_node_ret)
			*old_node_ret = old_node;

		/*
		 * Propagate footprint change to ancestors when the node
		 * changes type.  Density tracks readside footprint, not
		 * hop count — a type change alters the footprint.
		 *
		 * Walk from the old metadata's parent using old_flag for
		 * the depth span calculation: the parent's child slot
		 * still holds old_flag (the caller publishes the new
		 * pointer later).
		 *
		 * When recompacting to NULL, use old metadata's parent
		 * chain with a negative delta equal to the old footprint.
		 */
		if (new_type_index != NODE_INDEX_NULL &&
		    old_type_index != NODE_INDEX_NULL && metadata &&
		    old_type->order != new_type->order) {
			long fp_delta = (long) (1U << (new_type->order - 4))
				      - (long) (1U << (old_type->order - 4));
			struct cds_ft_inode_flag *parent = metadata->parent;

			if (parent) {
				unsigned int parent_depth =
					node_depth - ft_parent_depth_span(parent,
						old_flag);
				ft_propagate_node_density_parent(ft, parent,
					parent_depth, node_depth, fp_delta);
			}
		} else if (new_type_index == NODE_INDEX_NULL &&
			   old_type_index != NODE_INDEX_NULL && metadata) {
			/*
			 * Node disappears entirely: subtract its full density
			 * contribution (own footprint + subtree density[0])
			 * from ancestors.
			 */
			long fp_delta = -((long) (1U << (old_type->order - 4))
					+ (long) ft_density_get(metadata, 0));
			struct cds_ft_inode_flag *parent = metadata->parent;

			if (parent) {
				unsigned int parent_depth =
					node_depth - ft_parent_depth_span(parent,
						old_flag);
				ft_propagate_node_density_parent(ft, parent,
					parent_depth, node_depth, fp_delta);
			}
		}
	}

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
		 */
		struct cds_ft_inode_flag **slot_ptr = NULL;

		if (ft_node_skip_compressed(child_node_flag))
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
				return FT_DESCENT_END;
			}
			*match_len_p = i + cmp_len;
			*match_node_p = NULL;
		} else {
			if (ft_key_cmp_ordinals(key, cn->key_bytes,
					cmp_len, remaining_key, false, NULL) != 0) {
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
			*match_len_p = i;
			*match_node_p = *found_ret;
		}
		return FT_DESCENT_END;
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
	if (!node_flag) {
		*status_ret = CDS_FT_STATUS_NOT_FOUND;
		return FT_DESCENT_END;
	}
	if (iter) {
		iter_path_node(iter)[i] = node_flag;
		*iter_path_len_p = i + 1;
	}

	*node_flag_p = node_flag;
	*key_p = key;
	*i_p = i;

	if (i >= key_depth)
		return FT_DESCENT_BREAK;

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
		return FT_DESCENT_END;
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

	return FT_DESCENT_CONTINUE;
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
enum ft_descent_action ft_lookup_collapsed(struct cds_ft_inode_flag **node_flag_p,
		const uint8_t **key_p, unsigned int *i_p,
		unsigned int key_depth,
		struct cds_ft_iter *iter, size_t *iter_path_len_p,
		bool track, bool track_longest,
		size_t *match_len_p, struct cds_ft_node **match_node_p,
		struct cds_ft_node __attribute__((unused)) **found_ret,
		enum cds_ft_status *status_ret,
		bool candidate)
{
	struct cds_ft_inode_flag *node_flag = *node_flag_p;
	const uint8_t *key = *key_p;
	unsigned int i = *i_p;
	unsigned int tier = ft_collapsed_tier(node_flag);
	unsigned int stride = ft_collapsed_stride(node_flag);
	struct cds_ft_collapsed_node *col = ft_collapsed_node_ptr(node_flag);
	unsigned int remaining_key = key_depth - 1 - i;
	unsigned int candidate_mask;
	uint16_t target_prefix;
	bool skip_verify;

	/*
	 * Check external_nodes only when needed: for prefix
	 * tracking or when the key ends at this depth.  Avoid
	 * loading the metadata cache line during normal lookups
	 * that traverse through the collapsed node.
	 */
	if (remaining_key == 0)
		return FT_DESCENT_BREAK;
	if (track) {
		struct cds_ft_metadata *cn_meta =
			cds_ft_item_to_metadata((struct cds_ft_inode *) col);
		struct cds_ft_node *ext =
			ft_dereference_prefetch_external(cn_meta->external_nodes);

		if (ext || track_longest) {
			*match_len_p = i;
			*match_node_p = ext;
		}
	}

	/*
	 * Every live entry has slen >= FT_COLLAPSE_SUFFIX_MIN (= 2),
	 * enforced by the build path's per-tier suffix-min gate and
	 * the in-place append gate.  When fewer than 2 key bytes
	 * remain, no entry can match — fail fast.
	 */
	if (remaining_key < 2) {
		*status_ret = CDS_FT_STATUS_NOT_FOUND;
		return FT_DESCENT_END;
	}

	/*
	 * Compose the target's first two bytes into a 16-bit prefix.
	 * The {0x00, 0x00} case takes a scalar bypass: zero-init memory
	 * (unwritten slots) reads as (0,0) in the prefix arrays, which
	 * would saturate the SIMD candidate mask with ghost matches.
	 * Sort invariant places live (0,0)-prefix entries contiguously
	 * starting at slot 0; the bypass walks until the first slot
	 * with a non-zero prefix byte (or capacity), building a
	 * candidate mask that the unified verify loop processes.
	 *
	 * For non-(0,0) targets the SIMD prefix scan returns a
	 * candidate mask whose set bits identify slots whose
	 * prefix_0/prefix_1 bytes both match the target.  Prefix
	 * stores are relaxed; the verify on the atomically-published
	 * subkey (full bytes 0..slen-1) is the source of truth and
	 * catches any torn-prefix false acceptance.
	 *
	 * The SIMD prefix scan is stride-invariant: wide nodes share
	 * the narrow header layout exactly, only halving the entry
	 * count.  Trailing prefix lanes beyond the wide capacity are
	 * zero-init and produce no false positives against any
	 * non-zero target.  The (0,0) bypass uses stride-aware
	 * capacity to bound the scalar walk correctly.
	 */
	target_prefix = (uint16_t) key[0] | ((uint16_t) key[1] << 8);
	if (caa_unlikely(target_prefix == 0)) {
		const uint8_t *p0 = ft_collapsed_prefix_0(col, tier);
		const uint8_t *p1 = ft_collapsed_prefix_1(col, tier);
		unsigned int cap = ft_collapsed_capacity_stride(tier, stride);
		unsigned int e;

		candidate_mask = 0;
		for (e = 0; e < cap; e++) {
			if (p0[e] != 0 || p1[e] != 0)
				break;
			candidate_mask |= 1U << e;
		}
	} else {
		candidate_mask = ft_collapsed_simd_match(col, tier,
				key[0], key[1]);
	}

	/*
	 * Candidate-mode fast path: if the prefilter narrowed to exactly
	 * one entry, skip the suffix byte verify and descend speculatively.
	 * Any wrong descent (caused by a torn prefix or by suffix bytes
	 * past prefix_0/prefix_1 not matching the lookup key) is caught at
	 * the leaf full-key compare in candidate-mode lookups.  The slen
	 * load itself is the publish handshake (slen != 0) and remains.
	 *
	 * Multi-candidate (popcount > 1) cannot skip the verify: it would
	 * mis-disambiguate between sibling branches and return a false
	 * NOT_FOUND for keys that exist via the correct sibling.
	 */
	skip_verify = candidate
			&& candidate_mask != 0
			&& (candidate_mask & (candidate_mask - 1)) == 0;

	/*
	 * Iterate set bits in the candidate mask.  For each candidate:
	 * (1) load the subkey (narrow: 8B SWAR; wide: acquire on @len);
	 * (2) skip if slen=0 (slot unwritten or unpublished);
	 * (3) verify suffix bytes 0..slen-1 against the full target
	 *     (narrow: SWAR equality on the packed word; wide:
	 *     byte-by-byte compare against entry->suffix);
	 * (4) rcu_dereference child for liveness, skip if NULL.
	 *
	 * ft_collapsed_match_candidate encapsulates steps (1)-(4) and
	 * dispatches on @stride.  When @skip_verify is true (cand mode,
	 * popcount==1), step (3) is omitted.
	 */
	while (candidate_mask) {
		unsigned int e = (unsigned int) __builtin_ctz(candidate_mask);
		struct cds_ft_inode_flag *child;
		unsigned int slen;

		candidate_mask &= candidate_mask - 1;

		if (!ft_collapsed_match_candidate(col, tier, stride, e,
				key, remaining_key, &slen, &child,
				skip_verify))
			continue;

		/* Match found. Advance past the suffix. */
		key += slen;
		if (iter) {
			unsigned int k;

			for (k = 1; k <= slen; k++)
				iter_path_node(iter)[i + k] =
					(struct cds_ft_inode_flag *)
					ft_collapsed_node_flag_tier_stride(col, tier, stride);
		}
		i += slen;
		node_flag = child;
		if (!node_flag) {
			*status_ret = CDS_FT_STATUS_NOT_FOUND;
			return FT_DESCENT_END;
		}
		node_flag = ft_resolve_skip_compressed(node_flag);
		if (iter) {
			iter_path_node(iter)[i] = node_flag;
			*iter_path_len_p = i + 1;
		}

		*node_flag_p = node_flag;
		*key_p = key;
		*i_p = i;

		if (i >= key_depth)
			return FT_DESCENT_BREAK;

		/* External child before end of key. */
		if (i < key_depth - 1 && ft_node_external(node_flag)) {
			if (track) {
				*match_len_p = i;
				*match_node_p = (struct cds_ft_node *) node_flag;
			}
			*status_ret = CDS_FT_STATUS_NOT_FOUND;
			return FT_DESCENT_END;
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

		return FT_DESCENT_CONTINUE;
	}

	/* No matching suffix found. */
	*status_ret = CDS_FT_STATUS_NOT_FOUND;
	return FT_DESCENT_END;
}
#else
static inline_lookup
enum ft_descent_action ft_lookup_collapsed(
		struct cds_ft_inode_flag **node_flag_p __attribute__((unused)),
		const uint8_t **key_p __attribute__((unused)),
		unsigned int *i_p __attribute__((unused)),
		unsigned int key_depth __attribute__((unused)),
		struct cds_ft_iter *iter __attribute__((unused)),
		size_t *iter_path_len_p __attribute__((unused)),
		bool track __attribute__((unused)),
		bool track_longest __attribute__((unused)),
		size_t *match_len_p __attribute__((unused)),
		struct cds_ft_node **match_node_p __attribute__((unused)),
		struct cds_ft_node **found_ret __attribute__((unused)),
		enum cds_ft_status *status_ret __attribute__((unused)),
		bool candidate __attribute__((unused)))
{
	return FT_DESCENT_END;
}
#endif /* FEATURE_FT_COLLAPSE */

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

#ifdef FEATURE_FT_COLLAPSE
/*
 * Simple collapsed node traversal for read-side descent loops
 * (replace, count_keys_prefix).  Scans entries for a suffix that
 * matches the remaining key, advances key/index/node_flag, and
 * returns the loop action.
 */
static
enum ft_descent_action ft_traverse_collapsed(struct cds_ft_inode_flag **node_flag_p,
		struct cds_ft_inode_flag ***node_flag_ptr_p,
		const uint8_t **key_p, unsigned int *i_p,
		unsigned int key_depth, bool *not_found)
{
	struct cds_ft_inode_flag *node_flag = *node_flag_p;
	unsigned int tier = ft_collapsed_tier(node_flag);
	unsigned int stride = ft_collapsed_stride(node_flag);
	struct cds_ft_collapsed_node *col = ft_collapsed_node_ptr(node_flag);
	const uint8_t *key = *key_p;
	unsigned int i = *i_p;
	unsigned int remaining = key_depth - i;
	unsigned int candidate_mask;
	uint16_t target_prefix;

	/*
	 * Live entries have slen >= 2.  Fewer than 2 remaining key
	 * bytes means no entry can match.
	 */
	if (remaining < 2) {
		*not_found = true;
		return FT_DESCENT_END;
	}

	/*
	 * (0,0) bypass walks slots [0..first non-zero prefix byte) since
	 * sort invariant + zero-init unwritten slots would otherwise
	 * saturate the SIMD candidate mask.  Non-(0,0) targets use the
	 * SIMD prefilter directly.  Verify on the atomically-published
	 * subkey is the source of truth; SIMD pre-filter is best-effort.
	 */
	target_prefix = (uint16_t) key[0] | ((uint16_t) key[1] << 8);
	if (caa_unlikely(target_prefix == 0)) {
		const uint8_t *p0 = ft_collapsed_prefix_0(col, tier);
		const uint8_t *p1 = ft_collapsed_prefix_1(col, tier);
		unsigned int cap = ft_collapsed_capacity_stride(tier, stride);
		unsigned int e;

		candidate_mask = 0;
		for (e = 0; e < cap; e++) {
			if (p0[e] != 0 || p1[e] != 0)
				break;
			candidate_mask |= 1U << e;
		}
	} else {
		candidate_mask = ft_collapsed_simd_match(col, tier,
				key[0], key[1]);
	}

	while (candidate_mask) {
		unsigned int e = (unsigned int) __builtin_ctz(candidate_mask);
		struct cds_ft_inode_flag *child;
		struct cds_ft_inode_flag **child_addr;
		unsigned int slen;

		candidate_mask &= candidate_mask - 1;

		if (!ft_collapsed_match_candidate(col, tier, stride, e,
				key, remaining, &slen, &child,
				false /* skip_verify */))
			continue;

		*key_p = key + slen;
		*i_p = i + slen - 1;
		{
			struct cds_ft_inode_flag *resolved =
				ft_resolve_skip_compressed(child);
			*node_flag_p = resolved;
		}
		if (node_flag_ptr_p) {
			if (stride == FT_COL_STRIDE_NARROW)
				child_addr = &ft_collapsed_entries(col, tier)[e].child;
			else
				child_addr = &ft_collapsed_entries_wide(col, tier)[e].child;
			*node_flag_ptr_p = child_addr;
		}
		if (!child) {
			*not_found = true;
			return FT_DESCENT_END;
		}
		if (ft_node_external(child))
			return FT_DESCENT_BREAK;
		return FT_DESCENT_CONTINUE;
	}
	*not_found = true;
	return FT_DESCENT_END;
}
#else
static
enum ft_descent_action ft_traverse_collapsed(
		struct cds_ft_inode_flag **node_flag_p __attribute__((unused)),
		struct cds_ft_inode_flag ***node_flag_ptr_p __attribute__((unused)),
		const uint8_t **key_p __attribute__((unused)),
		unsigned int *i_p __attribute__((unused)),
		unsigned int key_depth __attribute__((unused)),
		bool *not_found __attribute__((unused)))
{
	return FT_DESCENT_END;
}
#endif /* FEATURE_FT_COLLAPSE */

/*
 * @candidate: when true, skip key comparison at compressed nodes
 * during traversal (patricia-like mode).  The returned node is a
 * candidate that must be verified by the caller against their
 * stored key.  Constant-folded at each call site.
 *
 * When the group enables speculative_validated and the caller did
 * not request candidate semantics, descend with cand-mode anyway and
 * validate the result against the external node's stored key before
 * returning.  Limited to equality lookups (tracking == NONE) and
 * identity key_map (the user's stored key matches the trie's byte
 * order without reverse-mapping).
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
	const uint8_t *orig_key = key;
	struct cds_ft_inode_flag *node_flag;
	struct cds_ft_node *found = NULL;
	unsigned int key_depth, i;
	enum cds_ft_status status;
	bool spec_validate = !candidate
			&& ft->group->speculative_validated
			&& ft->group->key_map.identity
			&& tracking == FT_PREFIX_TRACK_NONE;
	bool descend_cand = candidate || spec_validate;
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
			if (!descend_cand) {
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
				/*
				 * If the skip's child is external and
				 * we've consumed the full key, exit the
				 * loop to the terminal check below.
				 */
				if (ft_node_external(node_flag))
					break;
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
				enum ft_descent_action act;

				i--;
				act = ft_lookup_compressed(&node_flag, &key, &i,
					key_depth, iter, &iter_path_len,
					track, track_longest,
					&match_len, &match_node, &found, &status,
					descend_cand);
				if (act == FT_DESCENT_END)
					goto end;
				if (act == FT_DESCENT_BREAK)
					break;
				continue;
			}
			if (ft_node_collapsed(node_flag)) {
				enum ft_descent_action act;

				i--;
				act = ft_lookup_collapsed(&node_flag, &key, &i,
					key_depth, iter, &iter_path_len,
					track, track_longest,
					&match_len, &match_node, &found, &status,
					descend_cand);
				if (act == FT_DESCENT_END)
					goto end;
				if (act == FT_DESCENT_BREAK)
					break;
				continue;
			}
			/*
			 * External or NULL at loop top.  Can happen when
			 * a compressed node's cn->child is an external
			 * node (key terminates at the compressed path
			 * end).  Break to the post-loop terminal handler.
			 */
			if (ft_node_external(node_flag))
				break;
			status = CDS_FT_STATUS_NOT_FOUND;
			goto end;
		}

		iter_key = *(key++);
		if (descend_cand)
			node_flag = ft_node_get_nth_skip(node_flag, NULL, iter_key, FT_PF_DATA);
		else if (!skip_compressed)
			node_flag = ft_node_get_nth_skip(node_flag, NULL, iter_key, FT_PF_DATA);
		else
			node_flag = ft_node_get_nth(node_flag, NULL, iter_key, FT_PF_NONE);
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
		 * Non-candidate: convert to compressed flag and
		 * continue so the compressed handler at the loop
		 * top processes it (key comparison, external_nodes
		 * check, etc.).
		 *
		 * Candidate: resolve the skip (advance past the
		 * compressed path without comparison).
		 */
		if (skip_compressed && caa_unlikely(ft_node_skip_compressed(node_flag))) {
			if (!descend_cand) {
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
	/*
	 * Speculative-validated lookup: when the group enables library-
	 * side validation and the descent ran in cand mode (spec_validate
	 * implies descend_cand was set internally), verify the candidate
	 * found at the leaf against the user's stored key bytes via the
	 * inline SIMD/SWAR comparator before reporting success.  A
	 * mismatch — caused by a wrong descent through a multi-byte
	 * compressed/collapsed prefix that the cand-mode descent did not
	 * verify — is reported as NOT_FOUND.
	 */
	if (spec_validate && status == CDS_FT_STATUS_OK && found) {
		const struct cds_ft_group *group = ft->group;
		const uint8_t *stored_key =
			(const uint8_t *) found + group->speculative_key_offset;
		bool match = true;

		if (group->speculative_key_len_offset !=
				CDS_FT_SPECULATIVE_OFFSET_NONE) {
			size_t stored_len = *(const size_t *)
				((const uint8_t *) found +
				 group->speculative_key_len_offset);
			if (stored_len != _key_len)
				match = false;
		}
		if (match && key_len > 0 &&
		    ft_key_cmp_ordinals(orig_key, stored_key,
				(unsigned int) key_len,
				(unsigned int) key_len,
				false, NULL) != 0)
			match = false;
		if (!match) {
			found = NULL;
			status = CDS_FT_STATUS_NOT_FOUND;
		}
	}
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
	const struct cds_ft_key_map *km = &ft->group->key_map;
	enum cds_ft_status status;

	if (!valid_key_len(ft, key_len))
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	CDS_FT_SCOPED_READER(ft);
	FT_TP_KEY(lookup_key_enter, ft, key, _key_len);
	if (caa_likely(km->identity)) {
		status = do_cds_ft_lookup(ft, key, key_len, result_node, NULL,
					FT_PREFIX_TRACK_NONE, NULL, NULL, false);
	} else {
		uint8_t ordinals[FT_MAX_KEY_LEN];

		ft_key_to_ordinals(ordinals, key, key_len, km);
		status = do_cds_ft_lookup(ft, ordinals, key_len, result_node, NULL,
					FT_PREFIX_TRACK_NONE, NULL, NULL, false);
	}
	FT_TP(lookup_key_exit, (int) status);
	return status;
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
	const struct cds_ft_key_map *km = &ft->group->key_map;

	if (!valid_key_len(ft, key_len))
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	CDS_FT_SCOPED_READER(ft);
	/*
	 * Identity key-map fast path: the ordinals[] buffer would be
	 * a byte-for-byte copy of @key; skip the 256-byte stack
	 * allocation and the entry memcpy by passing the caller's
	 * key directly to the descent.  Common case -- non-identity
	 * maps are only set up for key_maps that reorder bytes.
	 */
	if (caa_likely(km->identity))
		return do_cds_ft_lookup(ft, key, key_len, result_node, NULL,
					FT_PREFIX_TRACK_NONE, NULL, NULL, true);
	{
		uint8_t ordinals[FT_MAX_KEY_LEN];

		ft_key_to_ordinals(ordinals, key, key_len, km);
		return do_cds_ft_lookup(ft, ordinals, key_len, result_node, NULL,
					FT_PREFIX_TRACK_NONE, NULL, NULL, true);
	}
}

enum cds_ft_status cds_ft_lookup(struct cds_ft *ft,
		struct cds_ft_iter *iter)
{
	enum cds_ft_status status;

	CDS_FT_SCOPED_READER(ft);
	FT_TP_ITER_KEY(lookup_enter, iter);
	status = do_cds_ft_lookup(ft, iter_key(iter), iter->key_len, NULL, iter,
				FT_PREFIX_TRACK_NONE, NULL, NULL, false);
	FT_TP(lookup_exit, (int) status);
	return status;
}

enum cds_ft_status cds_ft_lookup_partial_key(struct cds_ft *ft,
		const uint8_t *key, size_t _key_len, size_t *match_len,
		struct cds_ft_node **result_node)
{
	struct cds_ft_node *partial_node = NULL;
	size_t partial_len = 0;
	size_t key_len = ft_key_len(ft, _key_len);
	const struct cds_ft_key_map *km = &ft->group->key_map;

	if (!valid_key_len(ft, key_len)) {
		*match_len = 0;
		*result_node = NULL;
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}
	CDS_FT_SCOPED_READER(ft);
	if (caa_likely(km->identity)) {
		do_cds_ft_lookup(ft, key, key_len, NULL, NULL,
				 FT_PREFIX_TRACK_PARTIAL, &partial_len,
				 &partial_node, false);
	} else {
		uint8_t ordinals[FT_MAX_KEY_LEN];

		ft_key_to_ordinals(ordinals, key, key_len, km);
		do_cds_ft_lookup(ft, ordinals, key_len, NULL, NULL,
				 FT_PREFIX_TRACK_PARTIAL, &partial_len,
				 &partial_node, false);
	}

	*match_len = partial_len;
	*result_node = partial_node;
	return partial_node ? CDS_FT_STATUS_OK : CDS_FT_STATUS_NOT_FOUND;
}

enum cds_ft_status cds_ft_lookup_partial(struct cds_ft *ft,
		struct cds_ft_iter *iter)
{
	struct cds_ft_node *partial_node = NULL;
	size_t partial_len = 0;

	CDS_FT_SCOPED_READER(ft);
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
	const struct cds_ft_key_map *km = &ft->group->key_map;

	if (!valid_key_len(ft, key_len)) {
		*match_len = 0;
		*result_node = NULL;
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}
	CDS_FT_SCOPED_READER(ft);
	if (caa_likely(km->identity)) {
		ret = do_cds_ft_lookup(ft, key, key_len, NULL, NULL,
				       FT_PREFIX_TRACK_LONGEST, &longest_len,
				       &match_node, false);
	} else {
		uint8_t ordinals[FT_MAX_KEY_LEN];

		ft_key_to_ordinals(ordinals, key, key_len, km);
		ret = do_cds_ft_lookup(ft, ordinals, key_len, NULL, NULL,
				       FT_PREFIX_TRACK_LONGEST, &longest_len,
				       &match_node, false);
	}

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

	CDS_FT_SCOPED_READER(ft);
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
 * determine direction.  Skips @ref_entry itself and dead (child ==
 * NULL) entries.
 *
 * Race-free pointer selection: each candidate entry is filtered by
 * child != NULL at scan time, and we SAVE the acquire-loaded child
 * pointer into *@best_child so the caller uses the same pointer the
 * scan validated.  Re-reading the entry's child pointer later could
 * race with a concurrent delete (CMM_RELEASE of NULL into child).
 *
 * Returns the index of the nearest entry, or -1 if none found.
 * On success, *@best_child receives the validated child pointer.
 */
#ifdef FEATURE_FT_COLLAPSE
static inline_lookup
int ft_collapsed_find_nearest(
		struct cds_ft_collapsed_node *col,
		unsigned int tier,
		unsigned int stride,
		unsigned int ref_entry,
		enum ft_direction dir,
		struct cds_ft_inode_flag **best_child,
		uint8_t *best_suffix_out,
		unsigned int *best_slen_out)
{
	struct ft_col_view ref_view;
	struct ft_col_view view;
	uint8_t best_suffix[FT_COL_W_SUFFIX_MAX];
	unsigned int best_slen = 0;
	struct cds_ft_inode_flag *saved_best_child = NULL;
	int best = -1;
	unsigned int e;

	ft_collapsed_load_entry_view_rcu(col, tier, stride, ref_entry, &ref_view);

	ft_for_each_live_collapsed_entry_view_rcu(col, tier, stride, e, view) {
		unsigned int slen = view.slen;
		unsigned int mc;
		int r;

		if (e == ref_entry)
			continue;
		if (!view.child)
			continue;

		mc = slen < ref_view.slen ? slen : ref_view.slen;
		r = ft_key_cmp_ordinals(view.suffix, ref_view.suffix,
					mc, mc, true, NULL);
		if (r == 0)
			r = (slen > ref_view.slen) ? 1 :
			    (slen < ref_view.slen) ? -1 : 0;

		if ((dir == FT_RIGHT && r > 0) ||
		    (dir == FT_LEFT && r < 0)) {
			if (best < 0) {
				best = (int)e;
				memcpy(best_suffix, view.suffix, slen);
				best_slen = slen;
				saved_best_child = view.child;
			} else {
				unsigned int bl = best_slen;
				unsigned int mc2 = bl < slen ? bl : slen;
				int r2;

				r2 = ft_key_cmp_ordinals(view.suffix, best_suffix,
							 mc2, mc2, true, NULL);

				if ((dir == FT_RIGHT && (r2 < 0 || (r2 == 0 && slen < bl))) ||
				    (dir == FT_LEFT && (r2 > 0 || (r2 == 0 && slen > bl)))) {
					best = (int)e;
					memcpy(best_suffix, view.suffix, slen);
					best_slen = slen;
					saved_best_child = view.child;
				}
			}
		}
	}
	if (best >= 0) {
		*best_child = saved_best_child;
		if (best_suffix_out)
			memcpy(best_suffix_out, best_suffix, best_slen);
		if (best_slen_out)
			*best_slen_out = best_slen;
	}
	return best;
}
#endif /* FEATURE_FT_COLLAPSE */

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

#ifdef FEATURE_FT_COLLAPSE
/*
 * Collapsed node handling for cds_ft_lookup_inequality (slow path).
 *
 * Scans all entries for the best match given the inequality mode:
 *   exact match → fill path, CONTINUE into child
 *   no exact, nearest in direction → fill path, DESCEND_CHILDREN
 *   no match in direction → GOING_UP
 */
static inline_lookup
enum ft_descent_action ft_inequality_collapsed(struct cds_ft_inode_flag **node_flag_p,
		ssize_t *level_p, ssize_t key_depth,
		ssize_t max_tree_depth __attribute__((unused)),
		enum ft_lookup_inequality mode,
		enum ft_lookup_limit limit,
		const uint8_t **iter_key_p,
		const uint8_t *input_key,
		struct cds_ft_iter *iter,
		uint8_t *ordinal_key,
		bool *skip_eq_external_nodes_p)
{
	struct cds_ft_inode_flag *node_flag = *node_flag_p;
	unsigned int tier = ft_collapsed_tier(node_flag);
	unsigned int stride = ft_collapsed_stride(node_flag);
	struct cds_ft_collapsed_node *col = ft_collapsed_node_ptr(node_flag);
	int level = *level_p;
	unsigned int remaining = key_depth - level;
	unsigned int e;
	struct ft_col_view view;
	int best_match = -1;	/* index of best directional match */
	uint8_t best_match_suffix[FT_COL_W_SUFFIX_MAX];
	unsigned int best_match_slen = 0;
	struct cds_ft_inode_flag *best_match_child = NULL;

	ft_for_each_live_collapsed_entry_view_rcu(col, tier, stride, e, view) {
		unsigned int slen = view.slen, cmp;
		const uint8_t *suffix = view.suffix;
		const uint8_t *cmp_key;
		uint8_t last_key_buf[FT_MAX_KEY_LEN];
		int cmp_result;

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
				assert(level < (int) max_tree_depth);
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
				node_flag = view.child;
				if (!node_flag) {
					*node_flag_p = node_flag;
					*level_p = level;
					return FT_DESCENT_BREAK;
				}
				node_flag = ft_resolve_skip_compressed(node_flag);
				iter_path_node(iter)[level + 1] = node_flag;
				if (ft_node_external(node_flag)) {
					*node_flag_p = node_flag;
					*level_p = level;
					return FT_DESCENT_BREAK;
				}
				*node_flag_p = node_flag;
				*level_p = level;
				return FT_DESCENT_CONTINUE;
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
				memcpy(best_match_suffix, suffix, slen);
				best_match_slen = slen;
				best_match_child = view.child;
			} else {
				/* Keep the smallest. */
				unsigned int bl = best_match_slen;
				unsigned int mc = bl < slen ? bl : slen;
				int r;

				r = ft_key_cmp_ordinals(suffix, best_match_suffix,
							mc, mc, true, NULL);

				if (r < 0 || (r == 0 && slen < bl)) {
					best_match = (int)e;
					memcpy(best_match_suffix, suffix, slen);
					best_match_slen = slen;
					best_match_child = view.child;
				}
			}
		} else if ((mode == FT_LOOKUP_LE || mode == FT_LOOKUP_LT) && cmp_result > 0) {
			if (best_match < 0) {
				best_match = (int)e;
				memcpy(best_match_suffix, suffix, slen);
				best_match_slen = slen;
				best_match_child = view.child;
			} else {
				/* Keep the largest. */
				unsigned int bl = best_match_slen;
				unsigned int mc = bl < slen ? bl : slen;
				int r;

				r = ft_key_cmp_ordinals(suffix, best_match_suffix,
							mc, mc, true, NULL);

				if (r > 0 || (r == 0 && slen > bl)) {
					best_match = (int)e;
					memcpy(best_match_suffix, suffix, slen);
					best_match_slen = slen;
					best_match_child = view.child;
				}
			}
		}
	}

	if (best_match >= 0) {
		/* Descend into the best directional match. */
		unsigned int slen = best_match_slen;
		unsigned int k;

		for (k = 0; k < slen; k++) {
			ordinal_key[level - 1 + k] = best_match_suffix[k];
			iter_path_node(iter)[level + k] = node_flag;
		}
		level += slen - 1;
		assert(level < (int) max_tree_depth);
		if (slen == 1)
			iter_path_node(iter)[level - 1] = node_flag;
		node_flag = best_match_child;
		if (!node_flag) {
			*node_flag_p = node_flag;
			*level_p = level;
			return FT_DESCENT_BREAK;
		}
		node_flag = ft_resolve_skip_compressed(node_flag);
		iter_path_node(iter)[level + 1] = node_flag;
		*skip_eq_external_nodes_p = false;
		*node_flag_p = node_flag;
		*level_p = level;
		iter_debug_path_snapshot(iter);
		return FT_DESCENT_DESCEND_CHILDREN;
	}

	/* No match in the requested direction. */
	*level_p = level - 1;
	iter_debug_path_snapshot(iter);
	return FT_DESCENT_GOING_UP;
}
#else
static inline_lookup
enum ft_descent_action ft_inequality_collapsed(
		struct cds_ft_inode_flag **node_flag_p __attribute__((unused)),
		ssize_t *level_p __attribute__((unused)),
		ssize_t key_depth __attribute__((unused)),
		ssize_t max_tree_depth __attribute__((unused)),
		enum ft_lookup_inequality mode __attribute__((unused)),
		enum ft_lookup_limit limit __attribute__((unused)),
		const uint8_t **iter_key_p __attribute__((unused)),
		const uint8_t *input_key __attribute__((unused)),
		struct cds_ft_iter *iter __attribute__((unused)),
		uint8_t *ordinal_key __attribute__((unused)),
		bool *skip_eq_external_nodes_p __attribute__((unused)))
{
	return FT_DESCENT_GOING_UP;
}
#endif /* FEATURE_FT_COLLAPSE */

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
		 * Unlike the collapsed empty-scan case and the internal
		 * minmax == NULL case (both transiently observable and
		 * handled via going_up above/below), there is no race
		 * window in which a reachable compressed has cn->child ==
		 * NULL.  Observing NULL here is a real bug: abort loudly
		 * instead of silently propagating a corrupt node_flag.
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

#ifdef FEATURE_FT_COLLAPSE
/*
 * Handle a collapsed node during the inequality minmax descent.
 *
 * On LEFTMOST (GE/GT) descent, check external_nodes at the
 * collapsed node's depth first.  Otherwise scan entries for the
 * min/max suffix in @dir, update ordinal_key and iter_path with
 * the selected suffix, advance @level across the suffix span,
 * and step to the selected child.
 *
 * Returns FT_DESCENT_FOUND_MINMAX on external-at-entry-depth
 * hit (with *ret_node_p / *level_p set), FT_DESCENT_GOING_UP
 * on transiently-empty collapsed (decrements @level and sets
 * *going_up_p), FT_DESCENT_BREAK when the selected child is
 * external, FT_DESCENT_CONTINUE otherwise.
 */
static inline_lookup
enum ft_descent_action ft_inequality_minmax_collapsed(
		struct cds_ft_inode_flag **node_flag_p,
		ssize_t *level_p,
		struct cds_ft_node **ret_node_p,
		bool *skip_eq_external_nodes_p,
		bool *going_up_p,
		struct cds_ft_iter *iter,
		uint8_t *ordinal_key,
		enum ft_direction dir,
		ssize_t max_tree_depth __attribute__((unused)))
{
	struct cds_ft_inode_flag *node_flag = *node_flag_p;
	unsigned int tier = ft_collapsed_tier(node_flag);
	unsigned int stride = ft_collapsed_stride(node_flag);
	struct cds_ft_collapsed_node *col = ft_collapsed_node_ptr(node_flag);
	ssize_t level = *level_p;
	unsigned int best, e;
	uint8_t best_suffix[FT_COL_W_SUFFIX_MAX];
	unsigned int best_slen = 0;
	struct cds_ft_inode_flag *best_child;
	struct ft_col_view view;

	if (dir == FT_LEFTMOST) {
		struct cds_ft_metadata *col_meta = cds_ft_item_to_metadata(
			(struct cds_ft_inode *) col);
		struct cds_ft_node *ext = rcu_dereference(col_meta->external_nodes);

		if (ext && !*skip_eq_external_nodes_p) {
			*ret_node_p = ext;
			*level_p = level - 1;
			return FT_DESCENT_FOUND_MINMAX;
		}
	}
	/*
	 * Find the min/max entry by suffix and descend.
	 *
	 * Race-free pointer selection: each candidate entry is
	 * filtered by slen != 0 (subkey published) and child != NULL
	 * (rcu_dereference) at scan time; we save the validated
	 * child pointer alongside the winning suffix so descent uses
	 * the snapshotted values.  No re-read of entry data later.
	 */
	best = UINT_MAX;
	best_child = NULL;

	ft_for_each_live_collapsed_entry_view_rcu(col, tier, stride, e, view) {
		if (!view.child)
			continue;
		if (best == UINT_MAX) {
			best = e;
			memcpy(best_suffix, view.suffix, view.slen);
			best_slen = view.slen;
			best_child = view.child;
			continue;
		}
		{
			unsigned int la = best_slen;
			unsigned int lb = view.slen;
			unsigned int mc = la < lb ? la : lb;
			int r;

			r = memcmp(view.suffix, best_suffix, mc);

			if (dir == FT_LEFTMOST) {
				if (r < 0 || (r == 0 && lb < la)) {
					best = e;
					memcpy(best_suffix, view.suffix, view.slen);
					best_slen = view.slen;
					best_child = view.child;
				}
			} else {
				if (r > 0 || (r == 0 && lb > la)) {
					best = e;
					memcpy(best_suffix, view.suffix, view.slen);
					best_slen = view.slen;
					best_child = view.child;
				}
			}
		}
	}
	if (best == UINT_MAX) {
		/*
		 * Transiently empty collapsed: a racing writer's
		 * sub-case A sequence (NULL the cptrs slot, then set
		 * the tombstone bit) can leave every entry observed as
		 * either dead-bit-set or cptrs==NULL at the moment we
		 * scanned them, even though the node is not quiescently
		 * empty (sub-case A guards nr_child > 1, and sub-cases
		 * B/C handle the last-entry case without tombstoning).
		 * Treat as "empty at this step" and let going_up find
		 * the next sibling in @dir at a higher level.  Back
		 * level one step so iter_path[level] is the collapsed
		 * flag the prior iter recorded (going_up uses
		 * iter_path[level] and iter_path[level-1] and needs
		 * both valid).
		 */
		*level_p = level - 1;
		*going_up_p = true;
		return FT_DESCENT_GOING_UP;
	}
	{
		unsigned int slen = best_slen;
		unsigned int k;

		for (k = 0; k < slen; k++) {
			ordinal_key[level - 1 + k] = best_suffix[k];
			iter_path_node(iter)[level + k] = node_flag;
		}
		level += slen - 1;
		assert(level < max_tree_depth);
		node_flag = best_child;
		node_flag = ft_resolve_skip_compressed(node_flag);
		iter_path_node(iter)[level + 1] = node_flag;
		*node_flag_p = node_flag;
		*level_p = level;
		if (ft_node_external(node_flag))
			return FT_DESCENT_BREAK;
	}
	*skip_eq_external_nodes_p = false;
	return FT_DESCENT_CONTINUE;
}

/*
 * Handle the collapsed-node branch of cds_ft_lookup_inequality's
 * going-up (sibling-search) phase.  When the path's parent at
 * level-1 is a collapsed node, its entries *are* the siblings, so
 * the regular ft_node_get_leftright does not apply — scan the
 * collapsed entries in @dir for the nearest live sibling of the
 * current entry.
 *
 * On success (sibling found) updates @level, *node_flag_p,
 * iter_path_node, and ordinal_key; returns FT_DESCENT_BREAK.
 *
 * Otherwise resets @level to entry_depth + 1, points *iter_key_p
 * at ordinal_key + entry_depth, sets *going_up_p = true, clears
 * *cached_col_nr_e_p, and returns FT_DESCENT_CONTINUE.
 */
static inline_lookup
enum ft_descent_action ft_inequality_going_up_collapsed(
		struct cds_ft_inode_flag **node_flag_p,
		ssize_t *level_p,
		const uint8_t **iter_key_p,
		bool *going_up_p,
		struct cds_ft_iter *iter,
		uint8_t *ordinal_key,
		enum ft_direction dir,
		ssize_t max_tree_depth __attribute__((unused)))
{
	ssize_t level = *level_p;
	struct cds_ft_inode_flag *col_node_flag = iter_path_node(iter)[level - 1];
	unsigned int tier = ft_collapsed_tier(col_node_flag);
	unsigned int stride = ft_collapsed_stride(col_node_flag);
	struct cds_ft_collapsed_node *col = ft_collapsed_node_ptr(col_node_flag);
	unsigned int cap = ft_collapsed_capacity_stride(tier, stride);
	int entry_depth = level - 1;
	int suffix_base;
	int current_entry = -1, best;
	unsigned int cur_slen = 0;
	struct cds_ft_inode_flag *cur_child_at_match = NULL;
	unsigned int e;
	struct cds_ft_inode_flag *best_child = NULL;

	/* Walk back to the collapsed node's entry depth. */
	while (entry_depth > 0 &&
	       iter_path_node(iter)[entry_depth - 1] ==
	       iter_path_node(iter)[level - 1])
		entry_depth--;

	/*
	 * suffix_base: the ordinal_key position where collapsed
	 * suffix data starts.  ft_inequality_collapsed (and the
	 * minmax variant) wrote suffix at ordinal_key[level_entry - 1]
	 * where level_entry is the level passed into the descent.
	 *
	 * The collapsed handler writes iter_path[level_entry..
	 * level_entry + slen - 1] = collapsed.  The previous
	 * iter_path[level_entry - 1] is also collapsed in every case
	 * we see here:
	 *   - root = collapsed: iter_path[0] = collapsed (root itself);
	 *   - internal -> collapsed: iter_path[level_entry - 1] = collapsed
	 *     (set by the main loop's previous-iter dispatch
	 *     ft_node_get_nth that fetched this collapsed child);
	 *   - compressed -> collapsed: iter_path[level_entry - 1] =
	 *     collapsed (overwritten by ft_inequality_compressed's
	 *     post-loop iter_path[level] = cn->child).
	 *
	 * Hence the walk-back above always lands entry_depth at
	 * level_entry - 1, and suffix_base = entry_depth.
	 */
	suffix_base = entry_depth;

	/*
	 * Find the current entry by suffix match.  Don't filter dead
	 * entries: the (suffix,len) pair is immutable once published,
	 * so a concurrent delete (child=NULL) can leave a dead slot
	 * whose suffix still matches the path the downward walk
	 * recorded.  We need the slot position to drive the sibling
	 * search regardless of liveness.  Filter only on slen==0
	 * (unpublished slot).
	 */
	for (e = 0; e < cap; e++) {
		struct ft_col_view view;
		unsigned int j2;
		bool match2;

		ft_collapsed_load_entry_view_rcu(col, tier, stride, e, &view);
		if (view.slen == 0)
			continue;
		match2 = true;
		for (j2 = 0; j2 < view.slen; j2++) {
			if (view.suffix[j2] != ordinal_key[suffix_base + j2]) {
				match2 = false;
				break;
			}
		}
		if (match2) {
			current_entry = (int) e;
			cur_slen = view.slen;
			cur_child_at_match = view.child;
			break;
		}
	}

	/*
	 * The current entry must always be found: the suffix is
	 * immutable, and the downward walk wrote it to ordinal_key.
	 * If not found, there is a bug in suffix_base computation or
	 * ordinal_key was corrupted.
	 */
	assert(current_entry >= 0);

	/*
	 * If we're past the current entry's suffix (in the child's
	 * subtree), check the child node for siblings first.
	 */
	if (level > (ssize_t)(suffix_base + cur_slen)) {
		struct cds_ft_inode_flag *child_flag = cur_child_at_match;

		child_flag = ft_resolve_skip_compressed(child_flag);
		if (child_flag && ft_node_internal(child_flag)) {
			struct cds_ft_inode_flag *node_flag;
			uint8_t sib_key = 0;

			node_flag = ft_node_get_leftright(child_flag,
				ordinal_key[suffix_base + cur_slen],
				&sib_key, dir);
			if (node_flag) {
				ordinal_key[suffix_base + cur_slen] = sib_key;
				level = suffix_base + cur_slen + 1;
				assert(level < max_tree_depth);
				iter_path_node(iter)[level] = node_flag;
				*node_flag_p = node_flag;
				*level_p = level;
				return FT_DESCENT_BREAK;
			}
		}
	}

	/*
	 * Find the nearest live sibling in @dir.
	 *
	 * Race-free pointer selection: find_nearest filters
	 * candidates by (dead-bit clear AND cptrs non-NULL) at scan
	 * time and returns the cptrs value it validated via
	 * @best_child.  Use that value directly here rather than
	 * re-reading cptrs[best]: a concurrent writer's sub-case A
	 * NULLs cptrs[e] *before* setting the tombstone bit, so the
	 * re-read window can return NULL while the scan-time check
	 * observed live.
	 */
	{
		uint8_t best_suffix[FT_COL_W_SUFFIX_MAX];
		unsigned int best_slen;

		best = ft_collapsed_find_nearest(col, tier, stride,
			(unsigned) current_entry, dir, &best_child,
			best_suffix, &best_slen);

		if (best >= 0) {
			unsigned int slen = best_slen;
			struct cds_ft_inode_flag *node_flag;
			unsigned int k;

			for (k = 0; k < slen; k++) {
				ordinal_key[suffix_base + k] = best_suffix[k];
				if (k > 0)
					iter_path_node(iter)[suffix_base + k] =
						iter_path_node(iter)[level - 1];
			}
			level = suffix_base + slen;
			assert(level < max_tree_depth);
			node_flag = best_child;
			node_flag = ft_resolve_skip_compressed(node_flag);
			iter_path_node(iter)[level] = node_flag;
			*node_flag_p = node_flag;
			*level_p = level;
			return FT_DESCENT_BREAK;
		}
	}

	/*
	 * No match found, continue going up.
	 *
	 * Reset iter_key to match the new level.  The collapsed
	 * handler jumped level back from within the entry's suffix
	 * span to entry_depth + 1.  iter_key must point to
	 * ordinal_key + entry_depth so the next *(--iter_key) at
	 * level entry_depth reads the correct key byte.
	 */
	*level_p = entry_depth + 1;
	*iter_key_p = ordinal_key + entry_depth;
	*going_up_p = true;
	return FT_DESCENT_CONTINUE;
}
#else
static inline_lookup
enum ft_descent_action ft_inequality_minmax_collapsed(
		struct cds_ft_inode_flag **node_flag_p __attribute__((unused)),
		ssize_t *level_p __attribute__((unused)),
		struct cds_ft_node **ret_node_p __attribute__((unused)),
		bool *skip_eq_external_nodes_p __attribute__((unused)),
		bool *going_up_p __attribute__((unused)),
		struct cds_ft_iter *iter __attribute__((unused)),
		uint8_t *ordinal_key __attribute__((unused)),
		enum ft_direction dir __attribute__((unused)),
		ssize_t max_tree_depth __attribute__((unused)))
{
	return FT_DESCENT_GOING_UP;
}

static inline_lookup
enum ft_descent_action ft_inequality_going_up_collapsed(
		struct cds_ft_inode_flag **node_flag_p __attribute__((unused)),
		ssize_t *level_p __attribute__((unused)),
		const uint8_t **iter_key_p __attribute__((unused)),
		bool *going_up_p __attribute__((unused)),
		struct cds_ft_iter *iter __attribute__((unused)),
		uint8_t *ordinal_key __attribute__((unused)),
		enum ft_direction dir __attribute__((unused)),
		ssize_t max_tree_depth __attribute__((unused)))
{
	return FT_DESCENT_CONTINUE;
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

		if (ft_type_is_linear(type->type_class) &&
				ft_linear_node_is_empty(type, ft_node_ptr(node_flag))) {

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
				level++;
				skip_eq_external_nodes = false;
				goto descend_children;
			}
			continue;
		}
		if (ft_node_collapsed(node_flag)) {
			enum ft_descent_action act;

			act = ft_inequality_collapsed(&node_flag,
				&level, key_depth,
				ft->group->max_tree_depth,
				mode, limit,
				&iter_key, input_key, iter,
				ordinal_key, &skip_eq_external_nodes);
			if (act == FT_DESCENT_GOING_UP)
				goto going_up;
			if (act == FT_DESCENT_DESCEND_CHILDREN)
				goto descend_children;
			if (act == FT_DESCENT_BREAK)
				break;
			/*
			 * CONTINUE: the suffix advanced level.  If the
			 * for-loop increment would push level past
			 * key_depth, descend into the child to find
			 * the min/max leaf rather than exiting the
			 * loop with a non-leaf node.
			 */
			if (level + 1 >= key_depth) {
				level++;
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
		if (ft_node_collapsed(iter_path_node(iter)[level - 1])) {
			enum ft_descent_action act;

			act = ft_inequality_going_up_collapsed(
				&node_flag, &level, &iter_key,
				&going_up,
				iter, ordinal_key, dir,
				(ssize_t) ft->group->max_tree_depth);
			if (act == FT_DESCENT_BREAK)
				break;
			assert(act == FT_DESCENT_CONTINUE);
			continue;
		}
		if (!ft_node_internal(iter_path_node(iter)[level - 1])) {
			FT_TP(ineq_going_up_step, level,
				(const void *) iter_path_node(iter)[level - 1],
				0, (uint8_t) key_value);
			going_up = true;
			continue;
		}
		node_flag = ft_node_get_leftright(iter_path_node(iter)[level - 1],
				key_value, &ordinal_key[level - 1], dir);
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
		if (ft_node_collapsed(node_flag)) {
			enum ft_descent_action act;

			act = ft_inequality_minmax_collapsed(
				&node_flag, &level, &ret_node,
				&skip_eq_external_nodes, &going_up,
				iter, ordinal_key, dir,
				(ssize_t) ft->group->max_tree_depth);
			if (act == FT_DESCENT_FOUND_MINMAX)
				goto found_minmax;
			if (act == FT_DESCENT_GOING_UP)
				goto going_up;
			if (act == FT_DESCENT_BREAK)
				break;
			assert(act == FT_DESCENT_CONTINUE);
			continue;
		}
		skip_eq_external_nodes = false;
		node_flag = ft_node_get_minmax(node_flag, &ordinal_key[level - 1], dir);
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
	 * validated external (compressed-branch external child,
	 * collapsed-branch best_child that turned out external, or
	 * minmax-branch external return).  Transiently-empty
	 * intermediate steps (see collapsed best == UINT_MAX and
	 * non-root minmax == NULL above) do not reach this assert
	 * -- they jump to going_up and re-enter the search at a
	 * higher level.
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
bool ft_density_is_extended(const struct cds_ft_metadata *m)
{
	return m->nr_keys == UINT32_MAX;
}

static inline
unsigned long ft_density_get(const struct cds_ft_metadata *m, unsigned int idx)
{
	if (caa_unlikely(ft_density_is_extended(m)))
		return m->density_ext->nr_nodes_at_depth[idx];
	return m->nr_nodes_at_depth[idx];
}

/*
 * Density pool: pre-allocated free list of cds_ft_density_extended
 * structs, per trie.  Topped up at mutation entry where -ENOMEM can
 * be cleanly returned; drawn from in ft_density_promote after the
 * point of no return.
 */
static
int ft_density_pool_ensure(struct cds_ft *ft, unsigned int needed)
{
	struct cds_ft_density_extended *ext;

	while (ft->density_pool_count < needed) {
		ext = calloc(1, sizeof(*ext));
		if (!ext)
			return -ENOMEM;
		ext->next = ft->density_pool;
		ft->density_pool = ext;
		ft->density_pool_count++;
	}
	return 0;
}

static inline
struct cds_ft_density_extended *ft_density_pool_alloc(struct cds_ft *ft)
{
	struct cds_ft_density_extended *ext = ft->density_pool;

	assert(ext);
	ft->density_pool = ext->next;
	ft->density_pool_count--;
	memset(ext, 0, sizeof(*ext));
	return ext;
}

static
void ft_density_pool_destroy(struct cds_ft *ft)
{
	struct cds_ft_density_extended *ext, *next;

	for (ext = ft->density_pool; ext; ext = next) {
		next = ext->next;
		free(ext);
	}
	ft->density_pool = NULL;
}

static
void ft_density_promote(struct cds_ft *ft, struct cds_ft_metadata *m)
{
	struct cds_ft_density_extended *ext;
	unsigned int i;

	ext = ft_density_pool_alloc(ft);
	for (i = 0; i < FT_NODE_DENSITY_DEPTH; i++)
		ext->nr_nodes_at_depth[i] = m->nr_nodes_at_depth[i];
	ext->nr_keys = m->nr_keys;
	/*
	 * Store density_ext pointer into the union first, then
	 * publish via nr_keys = UINT32_MAX with release.  Readers
	 * doing acquire-load on nr_keys that see UINT32_MAX are
	 * guaranteed to see the valid density_ext pointer.
	 */
	m->density_ext = ext;
	uatomic_store(&m->nr_keys, UINT32_MAX, CMM_RELEASE);
}

/*
 * ft_nr_keys_get: read nr_keys from compact or extended storage.
 * Write-side only (non-atomic read under mutex).
 */
static inline
unsigned long ft_nr_keys_get(const struct cds_ft_metadata *m)
{
	if (caa_unlikely(ft_density_is_extended(m)))
		return m->density_ext->nr_keys;
	return m->nr_keys;
}

/*
 * ft_nr_keys_load: read nr_keys with acquire semantics.
 * Read-side safe (concurrent with writers).
 */
static inline
unsigned long ft_nr_keys_load(const struct cds_ft_metadata *m)
{
	uint32_t val = uatomic_load(&m->nr_keys, CMM_ACQUIRE);

	if (caa_unlikely(val == UINT32_MAX))
		return uatomic_load(&m->density_ext->nr_keys, CMM_ACQUIRE);
	return val;
}

/*
 * ft_nr_keys_store: write nr_keys with specified memory order.
 * If compact and val would reach UINT32_MAX, promotes first.
 * Write-side only (mutex-held).
 */
static inline
void ft_nr_keys_store(struct cds_ft *ft, struct cds_ft_metadata *m, unsigned long val, int mo)
{
	if (caa_unlikely(ft_density_is_extended(m))) {
		uatomic_store(&m->density_ext->nr_keys, val, mo);
		return;
	}
	if (caa_unlikely(val >= UINT32_MAX)) {
		ft_density_promote(ft, m);
		uatomic_store(&m->density_ext->nr_keys, val, mo);
		return;
	}
	uatomic_store(&m->nr_keys, (uint32_t) val, mo);
}

static inline
void ft_density_set(struct cds_ft *ft, struct cds_ft_metadata *m, unsigned int idx, unsigned long val)
{
	if (caa_unlikely(ft_density_is_extended(m))) {
		m->density_ext->nr_nodes_at_depth[idx] = val;
		return;
	}
	if (caa_unlikely(val > FT_DENSITY_COMPACT_MAX)) {
		ft_density_promote(ft, m);
		m->density_ext->nr_nodes_at_depth[idx] = val;
		return;
	}
	m->nr_nodes_at_depth[idx] = (uint8_t) val;
}

static inline
void ft_density_add(struct cds_ft *ft, struct cds_ft_metadata *m, unsigned int idx, long delta)
{
	unsigned long val = ft_density_get(m, idx);

	assert(delta >= 0 || val >= (unsigned long) -delta);
	val += delta;
	ft_density_set(ft, m, idx, val);
}

static inline
void ft_density_sub(struct cds_ft *ft, struct cds_ft_metadata *m, unsigned int idx, unsigned long sub)
{
	unsigned long val = ft_density_get(m, idx);

	/*
	 * Density underflow breaks the core counter invariant. Assert
	 * loudly in debug builds; clamp to 0 in release builds so a
	 * broken invariant does not wrap into a giant counter and
	 * silently corrupt subsequent density math.
	 */
	assert(val >= sub);
	if (caa_unlikely(val < sub)) {
		ft_density_set(ft, m, idx, 0);
		return;
	}
	ft_density_set(ft, m, idx, val - sub);
}

static inline
void ft_density_free(struct cds_ft_metadata *m)
{
	if (ft_density_is_extended(m))
		free(m->density_ext);
}

/*
 * ft_propagate_external_count_parent: propagate nr_keys delta
 * from @start up to the root via metadata->parent pointers.
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
void ft_propagate_external_count_parent(struct cds_ft *ft,
		struct cds_ft_inode_flag *start, long delta)
{
	struct cds_ft_inode_flag *cur = start;

	ft_delay_writer();

	while (cur) {
		struct cds_ft_metadata *m =
			cds_ft_item_to_metadata(ft_node_ptr(cur));
		ft_nr_keys_store(ft, m, ft_nr_keys_get(m) + delta, CMM_RELEASE);
		ft_delay_writer();
		cur = m->parent;
	}
}

/*
 * Compute the precise density contribution of a child node to its
 * parent's density[0].  The child is at @distance levels below
 * the parent.
 * @child_footprint: the child's read-side footprint in 16B units.
 *
 * With cumulative density counters (density[j] = footprint at
 * levels j+1..DEPTH), the contribution visible within the
 * parent's window is:
 *   child_footprint + density[0] - density[DEPTH - distance]
 *
 * density[0] covers the child's full subtree.
 * density[DEPTH - distance] is the overflow beyond the parent's
 * window, subtracted in one operation.
 */
static inline unsigned long ft_child_density_contribution(
		struct cds_ft_metadata *cm, unsigned int distance,
		unsigned int child_footprint)
{
	if (distance >= FT_NODE_DENSITY_DEPTH)
		return child_footprint;
	return child_footprint
		+ ft_density_get(cm, 0)
		- ft_density_get(cm, FT_NODE_DENSITY_DEPTH - distance);
}

/*
 * ft_child_density_contribution_all: compute the contribution of a
 * child node to ALL cumulative density levels of its parent.
 *
 * For parent.density[j] (cumulative: levels j+1..DEPTH):
 *   j < distance: child_fp + child.density[0] - child.density[DEPTH-D]
 *                  (child itself + subtree within window)
 *   j >= distance: child.density[j-D] - child.density[DEPTH-D]
 *                  (subtree only, child is outside range j+1..DEPTH)
 *
 * Accumulates into @accum[0..DEPTH-1].
 */
static inline void ft_child_density_contribution_all(
		struct cds_ft_metadata *cm, unsigned int distance,
		unsigned int child_footprint,
		unsigned long *accum)
{
	unsigned int j;

	if (distance >= FT_NODE_DENSITY_DEPTH) {
		/* Child beyond window: only its own footprint visible. */
		for (j = 0; j < FT_NODE_DENSITY_DEPTH; j++)
			accum[j] += child_footprint;
		return;
	}
	{
		unsigned long overflow =
			ft_density_get(cm, FT_NODE_DENSITY_DEPTH - distance);

		for (j = 0; j < distance; j++)
			accum[j] += child_footprint
				+ ft_density_get(cm, 0) - overflow;
		for (j = distance; j < FT_NODE_DENSITY_DEPTH; j++)
			accum[j] += ft_density_get(cm, j - distance)
				- overflow;
	}
}

/*
 * ft_child_density_contribution_all_snapshot: same as
 * ft_child_density_contribution_all but reads from a saved density
 * array instead of live metadata.  Used when the child has been freed.
 */
static inline void ft_child_density_contribution_all_snapshot(
		const unsigned long *density, unsigned int distance,
		unsigned int child_footprint,
		unsigned long *accum)
{
	unsigned int j;

	if (distance >= FT_NODE_DENSITY_DEPTH) {
		for (j = 0; j < FT_NODE_DENSITY_DEPTH; j++)
			accum[j] += child_footprint;
		return;
	}
	{
		unsigned long overflow =
			density[FT_NODE_DENSITY_DEPTH - distance];

		for (j = 0; j < distance; j++)
			accum[j] += child_footprint
				+ density[0] - overflow;
		for (j = distance; j < FT_NODE_DENSITY_DEPTH; j++)
			accum[j] += density[j - distance]
				- overflow;
	}
}

/*
 * ft_propagate_density_replace: walk up the parent chain and update
 * density at each ancestor to account for replacing an old child
 * with a new child at depth @child_depth.
 *
 * Either or both of @old_density / @new_meta may be NULL:
 *   old_density != NULL, new_meta == NULL  → subtraction only (detach)
 *   old_density == NULL, new_meta != NULL  → addition only (graft)
 *   both non-NULL                          → replacement (collapse, explode, graft-swap)
 *
 * Two starting modes:
 *   @start != NULL: start from the child node and walk to its parent
 *     first (normal mode — used when the child still exists).
 *   @start == NULL: start directly from @first_anc at @first_anc_depth
 *     (parent mode — used when the child has been freed and only the
 *     surviving parent is available).
 *
 * @start: child node to start walking from (may be NULL).
 * @child_depth: depth of the old/new child being replaced.
 * @old_density: saved density array of the old child (may be NULL).
 * @old_fp: footprint of the old child (ignored if @old_density is NULL).
 * @new_meta: live metadata of the new child (may be NULL).
 * @new_fp: footprint of the new child (ignored if @new_meta is NULL).
 * @first_anc: first ancestor to update (used when @start is NULL).
 * @first_anc_depth: depth of @first_anc (used when @start is NULL).
 */
static
void ft_propagate_density_replace(struct cds_ft *ft,
		struct cds_ft_inode_flag *start,
		unsigned int child_depth,
		const unsigned long *old_density,
		unsigned int old_fp,
		struct cds_ft_metadata *new_meta,
		unsigned int new_fp,
		struct cds_ft_inode_flag *first_anc,
		unsigned int first_anc_depth)
{
	struct cds_ft_inode_flag *anc;
	unsigned int anc_depth;

	if (start) {
		struct cds_ft_metadata *sm = ft_flag_to_metadata(start);

		anc = sm->parent;
		if (!anc)
			return;
		anc_depth = child_depth
			- ft_parent_depth_span(anc, start);
	} else {
		anc = first_anc;
		anc_depth = first_anc_depth;
	}

	while (anc) {
		struct cds_ft_metadata *am = ft_flag_to_metadata(anc);
		unsigned int distance = child_depth - anc_depth;
		unsigned int j;
		unsigned long old_c[FT_NODE_DENSITY_DEPTH] = { 0 };
		unsigned long new_c[FT_NODE_DENSITY_DEPTH] = { 0 };

		if (distance > FT_NODE_DENSITY_DEPTH)
			break;
		if (old_density)
			ft_child_density_contribution_all_snapshot(
				old_density, distance, old_fp, old_c);
		if (new_meta)
			ft_child_density_contribution_all(
				new_meta, distance, new_fp, new_c);
		for (j = 0; j < FT_NODE_DENSITY_DEPTH; j++) {
			long delta = (long) new_c[j] - (long) old_c[j];

			if (delta > 0)
				ft_density_add(ft, am, j, delta);
			else if (delta < 0)
				ft_density_sub(ft, am, j, (unsigned long) -delta);
		}
		{
			struct cds_ft_inode_flag *parent = am->parent;

			if (!parent)
				break;
			anc_depth -= ft_parent_depth_span(parent, anc);
			anc = parent;
		}
	}
}

/*
 * ft_init_node_density: set all cumulative density counters
 * [0..DEPTH-1] for a node by computing contributions from
 * traversable children within FT_NODE_DENSITY_DEPTH levels.
 *
 * density[j] = total read-side footprint (16B units) of all
 * traversable nodes at levels j+1 through DEPTH below this node.
 */
static
void ft_init_node_density(struct cds_ft *ft,
		struct cds_ft_inode_flag *node_flag)
{
	struct cds_ft_inode *node;
	struct cds_ft_metadata *meta;
	unsigned int key, j;
	unsigned long accum[FT_NODE_DENSITY_DEPTH] = { 0 };

	if (!node_flag)
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

		if (child && !ft_node_external(child)) {
			if (cn->len <= FT_NODE_DENSITY_DEPTH) {
				struct cds_ft_metadata *cm =
					cds_ft_item_to_metadata(
						ft_node_ptr(child));
				ft_child_density_contribution_all(
					cm, cn->len,
					ft_node_readside_footprint(ft, child),
					accum);
			}
		}
		for (j = 0; j < FT_NODE_DENSITY_DEPTH; j++)
			ft_density_set(ft, cn_meta, j, accum[j]);
		return;
	}
	if (ft_node_external(node_flag))
		return;
	if (ft_node_compressed(node_flag)) {
		struct cds_ft_compressed_node *cn = ft_compressed_node_ptr(node_flag);
		struct cds_ft_metadata *cn_meta = cds_ft_item_to_metadata((struct cds_ft_inode *) cn);

		if (cn->child && !ft_node_external(cn->child)) {
			if (cn->len <= FT_NODE_DENSITY_DEPTH) {
				struct cds_ft_metadata *cm = cds_ft_item_to_metadata(ft_node_ptr(cn->child));
				ft_child_density_contribution_all(cm, cn->len,
					ft_node_readside_footprint(ft, cn->child),
					accum);
			}
		}
		for (j = 0; j < FT_NODE_DENSITY_DEPTH; j++)
			ft_density_set(ft, cn_meta, j, accum[j]);
		return;
	}
#ifdef FEATURE_FT_COLLAPSE
	if (ft_node_collapsed(node_flag)) {
		unsigned int tier = ft_collapsed_tier(node_flag);
		unsigned int stride = ft_collapsed_stride(node_flag);
		struct cds_ft_collapsed_node *col = ft_collapsed_node_ptr(node_flag);
		struct cds_ft_metadata *col_meta = cds_ft_item_to_metadata((struct cds_ft_inode *) col);
		struct ft_col_view view;
		unsigned int e;

		ft_for_each_live_collapsed_entry_view_rcu(col, tier, stride, e, view) {
			unsigned int slen = view.slen;
			struct cds_ft_inode_flag *child = view.child;

			if (!child)
				continue;
			/*
			 * Resolve skip-compressed before the external
			 * check: a skip pointer with an external child
			 * falsely matches ft_node_external, losing the
			 * density contribution of the subtree the
			 * compressed node represents.
			 */
			child = ft_resolve_skip_compressed(child);
			if (ft_node_external(child))
				continue;
			if (slen <= FT_NODE_DENSITY_DEPTH) {
				struct cds_ft_metadata *cm = ft_flag_to_metadata(child);
				ft_child_density_contribution_all(cm, slen,
					ft_node_readside_footprint(ft, child),
					accum);
			}
		}
		for (j = 0; j < FT_NODE_DENSITY_DEPTH; j++)
			ft_density_set(ft, col_meta, j, accum[j]);
		return;
	}
#endif /* FEATURE_FT_COLLAPSE */
	/* Internal node: walk children (at distance 1). */
	node = ft_node_ptr(node_flag);
	meta = cds_ft_item_to_metadata(node);

	for (key = 0; key < 256; key++) {
		struct cds_ft_inode_flag *child = ft_node_get_nth(node_flag, NULL, (uint8_t) key, FT_PF_NONE);

		if (!child)
			continue;
		if (ft_node_external(child))
			continue;
		{
			struct cds_ft_metadata *cm = cds_ft_item_to_metadata(ft_node_ptr(child));
			ft_child_density_contribution_all(cm, 1,
				ft_node_readside_footprint(ft, child),
				accum);
		}
	}
	for (j = 0; j < FT_NODE_DENSITY_DEPTH; j++)
		ft_density_set(ft, meta, j, accum[j]);
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
		unsigned int tier = ft_collapsed_tier(parent_nf);
		unsigned int stride = ft_collapsed_stride(parent_nf);
		struct cds_ft_collapsed_node *col =
			ft_collapsed_node_ptr(parent_nf);
		unsigned int cap = ft_collapsed_capacity_stride(tier, stride);
		unsigned int e;
		/*
		 * Resolve the search key once.  Callers may hand us either
		 * the bare compressed flag (e.g. metadata->parent loads) or
		 * the skip-encoded form (e.g. *slot or just-published top).
		 * Slot entries can carry either form too.  Comparing both
		 * sides in their resolved (bare) form is the unambiguous
		 * match.
		 */
		struct cds_ft_inode_flag *target = ft_resolve_skip_compressed(child_nf);

		if (stride == FT_COL_STRIDE_NARROW) {
			struct cds_ft_collapsed_entry *entries =
				ft_collapsed_entries(col, tier);

			for (e = 0; e < cap; e++) {
				struct cds_ft_collapsed_entry *entry = &entries[e];
				struct cds_ft_inode_flag *child = entry->child;

				if (child == NULL)
					continue;
				child = ft_resolve_skip_compressed(child);
				if (child == target)
					return ft_collapsed_suffix_len(entry);
			}
		} else {
			struct cds_ft_collapsed_entry_wide *entries =
				ft_collapsed_entries_wide(col, tier);

			for (e = 0; e < cap; e++) {
				struct cds_ft_collapsed_entry_wide *entry = &entries[e];
				struct cds_ft_inode_flag *child = entry->child;

				if (child == NULL)
					continue;
				child = ft_resolve_skip_compressed(child);
				if (child == target)
					return entry->len;
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
 * @delta: readside footprint change in 16-byte units (positive for
 *   creation/growth, negative for destruction/shrinkage).
 *
 * Only called from the write-side (mutex-held).
 */
static
void ft_propagate_node_density_parent(struct cds_ft *ft,
		struct cds_ft_inode_flag *start,
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
		/*
		 * Cumulative density: density[j] = footprint at
		 * levels j+1..DEPTH.  A node at @distance
		 * contributes to all density[0..distance-1].
		 */
		{
			unsigned int j;

			for (j = 0; j < distance; j++)
				ft_density_add(ft, m, j, delta);
		}
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
 * Maximum number of absorbed nodes tracked during a collapse walk.
 * Bounded by: max suffix length (256) + branching nodes (limited by
 * FT_NODE_DENSITY_DEPTH).  512 is generous for any practical subtree.
 */
#define FT_COLLAPSE_ABSORBED_MAX	512

static
int ft_collapsed_explode_node(struct cds_ft *ft,
		struct cds_ft_collapsed_node *col,
		struct cds_ft_inode_flag *col_flag,
		struct cds_ft_inode_flag *parent_nf,
		struct cds_ft_inode_flag **slot,
		unsigned int depth,
		struct cds_ft_inode_flag **out_internal_flag);

#ifdef FEATURE_FT_COMPRESS
static
int ft_compress_chain_at(struct cds_ft *ft,
		struct cds_ft_inode_flag *top_flag,
		struct cds_ft_inode_flag *parent_nf,
		struct cds_ft_inode_flag **slot,
		unsigned int top_depth);
#endif

static inline
void ft_record_absorbed(struct cds_ft_inode_flag **absorbed,
		unsigned int *absorbed_depths,
		unsigned int *nr_absorbed,
		struct cds_ft_inode_flag *node_flag,
		unsigned int node_depth)
{
	if (*nr_absorbed < FT_COLLAPSE_ABSORBED_MAX) {
		absorbed[*nr_absorbed] = node_flag;
		absorbed_depths[*nr_absorbed] = node_depth;
		(*nr_absorbed)++;
	}
}

/*
 * Minimum suffix length per tier to ensure the collapsed lookup CL
 * cost never exceeds the worst-case uncollapsed cost.
 *
 * Uncollapsed: each key byte traverses one internal node, costing
 * at most 2 CL loads (node dispatch + child pointer chase).
 * For S key bytes: worst-case uncollapsed cost = 2S CL.
 *
 * Collapsed: header CL(s) (prefix vectors) + 1 entry CL per lookup.
 *
 * Floor at FT_COLLAPSE_SUFFIX_MIN (2) since 1-byte suffixes don't
 * save over a direct dispatch.
 */
static inline
unsigned int ft_collapsed_min_slen(unsigned int tier)
{
	(void) tier;
	return FT_COLLAPSE_SUFFIX_MIN;
}

/*
 * Collapsed node configuration table.
 *
 * Ordered by preference: smallest allocation first.  The selection
 * walks this table and picks the first entry where:
 *   1. all entries fit in tier capacity
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
 *
 * @absorbed_footprint: accumulated read-side footprint (in 16B
 *   units) of every node consumed by the walk.  Updated on each
 *   absorption; not touched on emit_entry.  The caller should
 *   initialize this to 0 before the first call, and add the
 *   decision-point node's own footprint separately.
 * @absorbed: array to record absorbed node flags for later freeing.
 *   May be NULL to skip recording.
 * @absorbed_depths: parallel array of slen values (depth offset from
 *   the decision point) for each absorbed node.
 * @nr_absorbed: count of entries in @absorbed.
 * @current_path_cl_pct: pct-scaled CL cost a read-side lookup
 *   incurs on the current absorbed path (by value, includes the
 *   decision-point CL × 100).  Bumped on each absorption by
 *   ft_node_readside_cl_pct(...) so the per-node-type
 *   scan-cost multipliers are baked in.
 * @weighted_absorbed_cl_pct: accumulator for per-path pct-CL
 *   totals.  At each emit_entry, current_path_cl_pct is added in.
 *   Sum across all emit-entries equals N * avg_path_cl_pct,
 *   suitable for direct comparison with collapsed_cl_pct * N.
 *   Initialize to 0 before the first call.
 * @collapse_scan_mul_pct, @compress_scan_mul_pct: per-trie
 *   tunables (snapshot once by the caller); applied via
 *   ft_node_readside_cl_pct().
 * @branch_depth: number of multi-child branching points traversed
 *   from the decision point (passed by value).
 * @max_branch_depth: maximum branching depth before emitting.
 *   Only multi-child internal and collapsed nodes increment this.
 *   Compressed paths (which add to slen but not branching) are
 *   free — bounded only by max_slen.
 */
static
int ft_collapse_walk_subtree(struct cds_ft *ft,
		struct cds_ft_inode_flag *walk,
		uint8_t *suffix_buf, unsigned int slen,
		unsigned int max_slen,
		struct cds_ft_collapsed_node *col,
		unsigned int tier,
		unsigned int stride,
		unsigned int *entry_idx,
		unsigned int max_entries,
		unsigned int *absorbed_footprint,
		struct cds_ft_inode_flag **absorbed,
		unsigned int *absorbed_depths,
		unsigned int *nr_absorbed,
		unsigned int current_path_cl_pct,
		unsigned int *weighted_absorbed_cl_pct,
		unsigned int collapse_scan_mul_pct,
		unsigned int compress_scan_mul_pct,
		unsigned int branch_depth,
		unsigned int max_branch_depth)
{
	while (slen < max_slen) {
		struct cds_ft_inode_flag *orig_walk = walk;

		if (!walk)
			return -1;

		/* Leaf: emit terminal entry. */
		if (ft_node_external(walk))
			goto emit_entry;

		/*
		 * Skip-compressed: emit as intermediate entry.
		 * The skip path is free on the candidate fast path
		 * (0 CL — skip length in pointer bits).  Absorbing
		 * would add suffix bytes (scan cost) for no read-side
		 * gain.  Subtrees below the skip are collapsed at
		 * their own level via nested collapsed absorption.
		 */
		if (ft_node_skip_compressed(walk))
			goto emit_entry;

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
			if (slen + cn->len > max_slen)
				goto emit_entry; /* Would exceed suffix buffer. */
			*absorbed_footprint +=
				ft_node_readside_footprint(ft, orig_walk);
			current_path_cl_pct +=
				ft_node_readside_cl_pct(ft, orig_walk,
					collapse_scan_mul_pct,
					compress_scan_mul_pct);
			ft_record_absorbed(absorbed, absorbed_depths, nr_absorbed, orig_walk, slen);
			for (j = 0; j < cn->len; j++)
				suffix_buf[slen++] = cn->key_bytes[j];
			walk = ft_dereference_acquire(cn->child);
			continue;
		}

		/*
		 * Already-collapsed: absorb by iterating its
		 * entries, prepending each suffix to suffix_buf,
		 * and recursing into each entry's child.
		 * Same logic as multi-child internal, but dispatch
		 * is on collapsed entries rather than key bytes.
		 *
		 * If the collapsed node has external_nodes, emit
		 * to preserve prefix keys at this depth.
		 */
		if (ft_node_collapsed(walk)) {
			unsigned int child_tier = ft_collapsed_tier(walk);
			unsigned int child_stride = ft_collapsed_stride(walk);
			struct cds_ft_collapsed_node *col_child =
				ft_collapsed_node_ptr(walk);
			struct cds_ft_metadata *col_child_meta =
				cds_ft_item_to_metadata(
					(struct cds_ft_inode *) col_child);
			struct ft_col_view child_view;
			unsigned int e;

			if (col_child_meta->external_nodes)
				goto emit_entry;
			if (branch_depth >= max_branch_depth)
				goto emit_entry;
			*absorbed_footprint +=
				ft_node_readside_footprint(ft, walk);
			current_path_cl_pct +=
				ft_node_readside_cl_pct(ft, walk,
					collapse_scan_mul_pct,
					compress_scan_mul_pct);
			ft_record_absorbed(absorbed, absorbed_depths, nr_absorbed, walk, slen);
			ft_for_each_live_collapsed_entry_view_rcu(col_child,
					child_tier, child_stride, e, child_view) {
				unsigned int child_slen = child_view.slen;
				struct cds_ft_inode_flag *child_ptr = child_view.child;
				unsigned int k;

				if (!child_ptr)
					continue;
				if (slen + child_slen > max_slen)
					return -1;
				for (k = 0; k < child_slen; k++)
					suffix_buf[slen + k] = child_view.suffix[k];
				if (ft_collapse_walk_subtree(ft,
						child_ptr, suffix_buf,
						slen + child_slen,
						max_slen,
						col, tier, stride, entry_idx,
						max_entries,
						absorbed_footprint,
						absorbed, absorbed_depths,
						nr_absorbed,
						current_path_cl_pct,
						weighted_absorbed_cl_pct,
						collapse_scan_mul_pct,
						compress_scan_mul_pct,
						branch_depth + 1,
						max_branch_depth))
					return -1;
			}
			return 0;
		}

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
				if (!wc)
					return -1;
				*absorbed_footprint +=
					ft_node_readside_footprint(ft, walk);
				current_path_cl_pct +=
					ft_node_readside_cl_pct(ft, walk,
						collapse_scan_mul_pct,
						compress_scan_mul_pct);
				ft_record_absorbed(absorbed, absorbed_depths, nr_absorbed, walk, slen);
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
			if (branch_depth >= max_branch_depth)
				goto emit_entry;
			*absorbed_footprint +=
				ft_node_readside_footprint(ft, walk);
			current_path_cl_pct +=
				ft_node_readside_cl_pct(ft, walk,
					collapse_scan_mul_pct,
					compress_scan_mul_pct);
			ft_record_absorbed(absorbed, absorbed_depths, nr_absorbed, walk, slen);
			{
				uint8_t ck = 0;
				int pv = -1;
				struct cds_ft_inode_flag *c;

				c = ft_node_get_direction(walk, pv,
					&ck, FT_RIGHT);
				while (c) {
					suffix_buf[slen] = ck;
					if (ft_collapse_walk_subtree(ft,
							c, suffix_buf,
							slen + 1,
							max_slen,
							col, tier, stride, entry_idx,
							max_entries,
							absorbed_footprint,
							absorbed, absorbed_depths,
							nr_absorbed,
							current_path_cl_pct,
							weighted_absorbed_cl_pct,
							collapse_scan_mul_pct,
							compress_scan_mul_pct,
							branch_depth + 1,
							max_branch_depth))
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
		unsigned int idx = *entry_idx;
		unsigned int suffix_max = ft_col_stride_suffix_max[stride];
		unsigned int k;

		if (slen == 0)
			return -1;
		if (slen > suffix_max)
			return -1;
		if (idx >= max_entries)
			return -1;

		if (stride == FT_COL_STRIDE_NARROW) {
			struct cds_ft_collapsed_entry *entries =
					ft_collapsed_entries(col, tier);
			struct cds_ft_collapsed_entry *entry = &entries[idx];

			for (k = 0; k < slen; k++)
				entry->suffix[k] = suffix_buf[k];
			entry->len = (uint8_t) slen;
			/*
			 * Pre-publish: candidate is private to the builder.
			 * Plain store; CMM_RELEASE on the parent's graft
			 * pairs with readers' acquire on the parent.
			 */
			entry->child = walk;
		} else {
			struct cds_ft_collapsed_entry_wide *entries =
					ft_collapsed_entries_wide(col, tier);
			struct cds_ft_collapsed_entry_wide *entry = &entries[idx];

			for (k = 0; k < slen; k++)
				entry->suffix[k] = suffix_buf[k];
			entry->len = (uint8_t) slen;
			entry->child = walk;
		}
		/*
		 * Prefix-cache bytes for the SIMD reader fast path.  The
		 * prefix array layout is stride-invariant (sized for narrow
		 * capacity; wide entries past the wide capacity are zero-
		 * init and contribute no false matches).
		 */
		ft_collapsed_write_prefix_bytes(col, tier, idx,
			suffix_buf[0], suffix_buf[1]);
		*entry_idx = idx + 1;
		*weighted_absorbed_cl_pct += current_path_cl_pct;
		FT_TP(collapsed_entry,
			(const void *) ft_collapsed_node_flag(col), idx,
			suffix_buf, slen, (const void *) walk, 0);
		FT_TP(collapsed_publish,
			(const void *) ft_collapsed_node_flag(col), idx + 1,
			(unsigned int) ft_col_stride_entry_size[stride]);
		return 0;
	}
}

/*
 * ft_try_collapse_at_node: attempt to collapse the subtree rooted at
 * the given node into a collapsed node.  The node must be internal.
 *
 * Tries all feasible (order, scan_sel) configurations and picks the
 * one that maximizes the read-side footprint ratio:
 *   ratio = absorbed_footprint / collapsed_footprint
 *
 * absorbed_footprint: total read-side bytes (in 16B units) of every
 *   node consumed by the walk, plus the decision-point node itself.
 * collapsed_footprint: allocation size of the collapsed candidate
 *   (in 16B units).
 *
 * Selection by cross-multiplication (no FP, no division):
 *   candidate A beats B when  A.absorbed * B.collapsed
 *                            > B.absorbed * A.collapsed
 *
 * Only accepts candidates where ratio > 1 (absorbed > collapsed).
 *
 * No density pre-filter: every internal node with >= 2 children is
 * evaluated.  Walks self-limit via scan zone capacity.
 *
 * Returns the collapsed node flag on success, NULL if no configuration
 * improves cache footprint.
 *
 * @absorbed_out: on success, filled with flags of intermediate nodes
 *   that were absorbed into the collapsed node.  The caller must free
 *   them (via the appropriate free function) after publishing the
 *   collapsed node.  Array must have FT_COLLAPSE_ABSORBED_MAX entries.
 * @nr_absorbed_out: on success, set to the number of entries in
 *   @absorbed_out.
 */
static
struct cds_ft_inode_flag *ft_try_collapse_at_node(struct cds_ft *ft,
		struct cds_ft_inode_flag *node_flag,
		unsigned int node_depth,
		struct cds_ft_inode_flag **absorbed_out,
		unsigned int *absorbed_depths_out,
		unsigned int *nr_absorbed_out)
{
	struct cds_ft_inode *node = ft_node_ptr(node_flag);
	struct cds_ft_metadata *metadata = cds_ft_item_to_metadata(node);
	unsigned int nr_child = metadata->nr_child;
	uint8_t child_key = 0;
	int pivot;
	struct cds_ft_inode_flag *child;
	uint8_t suffix_buf[FT_COL_W_SUFFIX_MAX];
	unsigned int threshold_pct = uatomic_load(&ft->collapse_threshold_pct,
			CMM_RELAXED);

	*nr_absorbed_out = 0;

	/* Collapse disabled on this trie. */
	if (threshold_pct == CDS_FT_COLLAPSE_THRESHOLD_DISABLED)
		return NULL;

	/* Need at least 2 children. */
	if (nr_child < 2)
		return NULL;

	/*
	 * Quick footprint pre-filter: the subtree's total
	 * read-side footprint (in 16B units) plus the decision
	 * point must exceed the smallest collapsed allocation
	 * (T0 = 64B = 4 units).  Avoids entering the config
	 * loop when no config can possibly win.
	 */
	if (ft_density_get(metadata, 0)
	    + ft_node_readside_footprint(ft, node_flag)
	    <= (1U << (FT_COL_T0_ALLOC_ORDER - 4)))
		return NULL;

	{
		/*
		 * Footprint of the decision-point node itself: it will
		 * be freed if any collapse succeeds.
		 */
		unsigned int decision_fp = ft_node_readside_footprint(ft, node_flag);
		/*
		 * Snapshot the per-trie scan-cost multipliers.  Both
		 * apply throughout the walk (compress_mul on absorbed
		 * compressed nodes, collapse_mul on absorbed already-
		 * collapsed nodes) and to the collapsed candidate
		 * itself, keeping absorbed-side and candidate-side cost
		 * accounting symmetric.
		 */
		unsigned int collapse_scan_mul_pct = uatomic_load(
				&ft->collapse_scan_mul_pct, CMM_RELAXED);
		unsigned int compress_scan_mul_pct = uatomic_load(
				&ft->compress_scan_mul_pct, CMM_RELAXED);
		/*
		 * Pct-scaled read-side CL cost of the decision-point
		 * node (the absorbed path's first node).  Reset to be
		 * the starting CL on each collapse_walk_subtree call.
		 */
		unsigned int decision_cl_pct = ft_node_readside_cl_pct(ft,
				node_flag, collapse_scan_mul_pct,
				compress_scan_mul_pct);

		/*
		 * max_walk_slen: maximum suffix length.  Bounded by
		 * remaining trie depth and the suffix buffer size.
		 * Compressed paths consume slen but not branch depth.
		 * The inequality lookup assertions use max_tree_depth,
		 * so longer suffixes are safe.
		 *
		 * Per-stride cap is applied below in the (stride, tier)
		 * inner loop: narrow caps at FT_COL_SUFFIX_MAX (7), wide
		 * caps at FT_COL_W_SUFFIX_MAX (23).
		 */
		unsigned int max_walk_slen_tree;
		if (ft->group->max_tree_depth > node_depth + 1)
			max_walk_slen_tree = ft->group->max_tree_depth
				- node_depth - 1;
		else
			return NULL;

		/*
		 * max_branch_depth: limits combinatorial explosion
		 * from multi-child nodes.  Compressed paths are free
		 * (they don't add entries).  Only multi-child internal
		 * and collapsed nodes count as branching points.
		 */
		unsigned int max_branch_depth = FT_NODE_DENSITY_DEPTH;

		/* Best candidate tracking. */
		struct cds_ft_collapsed_node *best_col = NULL;
		struct cds_ft_metadata *best_meta = NULL;
		unsigned int best_tier = 0;
		unsigned int best_stride = FT_COL_STRIDE_NARROW;
		unsigned int best_nr_entries = 0;
		unsigned int best_absorbed = 0;
		unsigned int best_collapsed = 1;	/* denominator for first comparison */

		/*
		 * Track absorbed nodes for the winning config so they
		 * can be freed after the collapse is published.
		 * Two arrays: cur_ is filled by each walk, best_ holds
		 * the winning config's absorbed nodes.
		 */
		struct cds_ft_inode_flag *absorbed_cur[FT_COLLAPSE_ABSORBED_MAX];
		unsigned int absorbed_depths_cur[FT_COLLAPSE_ABSORBED_MAX];
		struct cds_ft_inode_flag *absorbed_best[FT_COLLAPSE_ABSORBED_MAX];
		unsigned int absorbed_depths_best[FT_COLLAPSE_ABSORBED_MAX];
		unsigned int nr_absorbed_cur;
		unsigned int nr_absorbed_best = 0;

		/*
		 * All valid (stride, tier) configurations.  Strides are
		 * tried narrow-then-wide; tiers are ordered small-to-large
		 * by allocation size, so equal-ratio candidates prefer
		 * the smallest footprint, then the smallest stride.
		 */
		unsigned int si, ci;

		/*
		 * Cache cumulative density for depth-aware filtering.
		 * density[k] = footprint at levels k+1..DEPTH.
		 */
		unsigned long density_total = ft_density_get(metadata, 0);

		/*
		 * Per-(stride, tier) candidate-side CL cost for the
		 * per-path latency gate, in pct-CL units
		 * (scan multiplier × avg_scan_CL + ptr_CL_pct).  Mirrors
		 * the absorbed-side accounting in ft_node_readside_cl_pct
		 * — see the function-header comment there for the
		 * derivation of each ptr_cl_pct entry.
		 *
		 * Each lookup pays:
		 *   - prefix-vector header CL load(s): 1 CL for every
		 *     tier (16B header for T0/T1, 64B header = full CL
		 *     for T2/T3).
		 *   - matched entry CL: 0 for T0 (entries share the
		 *     header CL), variable for T1 (per-stride straddle
		 *     accounting), 100 for T2/T3 (matched entry always
		 *     past the header CL).
		 */
		static const unsigned int avg_scan_cl_by_tier[FT_COL_NR_TIERS] = {
			[0] = 1, [1] = 1, [2] = 1, [3] = 1,
		};
		static const unsigned int ptr_cl_pct_by_tier_stride
				[FT_COL_NR_STRIDES][FT_COL_NR_TIERS] = {
			[FT_COL_STRIDE_NARROW] = {
				[0] = 0,	/* shared CL with scan */
				[1] = 57,	/* 4/7 slots in CL 1 */
				[2] = 100,
				[3] = 100,
			},
			[FT_COL_STRIDE_WIDE] = {
				[0] = 0,	/* wide-T0 disabled */
				[1] = 67,	/* 2/3 slots reach CL 1 */
				[2] = 100,
				[3] = 100,
			},
		};
		for (si = 0; si < FT_COL_NR_STRIDES; si++)
		for (ci = 0; ci < FT_COL_NR_TIERS; ci++) {
			unsigned int stride_si = si;
			unsigned int tier_ci = ci;
			unsigned int order = ft_col_tier_alloc_order[tier_ci];
			unsigned int collapsed_fp = 1U << (order - 4);
			unsigned int collapsed_cl_pct =
				collapse_scan_mul_pct * avg_scan_cl_by_tier[tier_ci]
				+ ptr_cl_pct_by_tier_stride[stride_si][tier_ci];
			unsigned int max_entries =
				ft_collapsed_max_entries(tier_ci, stride_si);
			unsigned int stride_suffix_max =
				ft_col_stride_suffix_max[stride_si];
			unsigned int max_walk_slen = max_walk_slen_tree;
			struct cds_ft_collapsed_node *col;
			struct cds_ft_metadata *col_meta;
			unsigned int entry_idx;
			unsigned int absorbed;
			unsigned int weighted_absorbed_cl_pct;
			bool walk_ok;

			if (max_walk_slen > stride_suffix_max)
				max_walk_slen = stride_suffix_max;

			if (max_entries < 2)
				continue;

			/*
			 * Per-config footprint gate using cumulative
			 * density.
			 *
			 * Total check: the full subtree + decision
			 * point must exceed this config's allocation.
			 */
			if (density_total + decision_fp <= collapsed_fp)
				continue;

			/*
			 * Depth-aware check: for configs with few
			 * entries (TINY/SMALL), the walk can only
			 * explore shallow branching.  Check that the
			 * shallow footprint alone justifies this
			 * config.
			 *
			 * density[0] - density[k] = footprint at
			 * levels 1..k (reachable within k branching
			 * levels).
			 *
			 * Effective depth heuristic based on max_entries:
			 *   <= 4 entries: ~2 branching levels
			 *   <= 8 entries: ~3 branching levels
			 *   <= 24 entries: ~4 branching levels
			 *   > 24 entries: use total (deep enough)
			 */
			{
				unsigned int eff_depth;
				unsigned long reachable_fp;

				if (max_entries <= 4)
					eff_depth = 2;
				else if (max_entries <= 8)
					eff_depth = 3;
				else if (max_entries <= 24)
					eff_depth = 4;
				else
					eff_depth = FT_NODE_DENSITY_DEPTH;

				if (eff_depth < FT_NODE_DENSITY_DEPTH)
					reachable_fp = density_total
						- ft_density_get(metadata,
							eff_depth);
				else
					reachable_fp = density_total;
				if (reachable_fp + decision_fp
				    <= collapsed_fp)
					continue;
			}

			col = alloc_collapsed_node(ft, tier_ci, &col_meta);
			if (!col)
				continue;

			absorbed = decision_fp;
			weighted_absorbed_cl_pct = 0;
			nr_absorbed_cur = 0;
			entry_idx = 0;
			walk_ok = true;

			pivot = -1;
			child = ft_node_get_direction(node_flag,
				pivot, &child_key, FT_RIGHT);
			while (child) {
				suffix_buf[0] = child_key;
				if (ft_collapse_walk_subtree(ft,
						child, suffix_buf, 1,
						max_walk_slen,
						col, tier_ci, stride_si, &entry_idx,
						max_entries,
						&absorbed,
						absorbed_cur, absorbed_depths_cur,
						&nr_absorbed_cur,
						decision_cl_pct,
						&weighted_absorbed_cl_pct,
						collapse_scan_mul_pct,
						compress_scan_mul_pct,
						0, /* branch_depth */
						max_branch_depth)) {
					walk_ok = false;
					break;
				}
				pivot = child_key;
				child = ft_node_get_direction(
					node_flag, pivot,
					&child_key, FT_RIGHT);
			}

			if (!walk_ok || entry_idx < 2) {
				free_collapsed_node_unpublished(ft, col);
				continue;
			}

			/*
			 * Suffix-min checks:
			 * 1. At least one entry with slen >= SUFFIX_MIN.
			 * 2. ALL entries meet the per-config min_slen.
			 *    This bounds the CL-per-byte cost: wider
			 *    scan zones (more CL loads) require longer
			 *    suffixes to justify the cost.
			 */
			{
				unsigned int e;
				bool has_long_suffix = false;
				unsigned int cfg_min_slen =
					ft_collapsed_min_slen(tier_ci);
				bool all_meet_min = true;

				for (e = 0; e < entry_idx; e++) {
					unsigned int sl;

					if (stride_si == FT_COL_STRIDE_NARROW) {
						struct cds_ft_collapsed_entry *en =
							&ft_collapsed_entries(col, tier_ci)[e];
						sl = ft_collapsed_suffix_len(en);
					} else {
						struct cds_ft_collapsed_entry_wide *en =
							&ft_collapsed_entries_wide(col, tier_ci)[e];
						sl = en->len;
					}
					if (sl >= FT_COLLAPSE_SUFFIX_MIN)
						has_long_suffix = true;
					if (sl < cfg_min_slen)
						all_meet_min = false;
				}
				if (!has_long_suffix || !all_meet_min) {
					free_collapsed_node_unpublished(ft, col);
					continue;
				}
			}

			/*
			 * Per-path CL latency safety gate: the absorbed
			 * paths' average CL load count must be at least as
			 * large as the collapsed candidate's CL load count.
			 *
			 * weighted_absorbed_cl_pct = Σ_e path_cl_pct(e),
			 * summed across emit-entries.  Each path_cl_pct
			 * includes the decision-point pct-CL plus every
			 * absorbed interior node on that path, all priced
			 * with the same per-node-type multipliers used for
			 * the collapsed candidate (collapse_scan_mul_pct on
			 * scan zones, compress_scan_mul_pct on compressed
			 * scan).  For N emit-entries:
			 *   avg_path_cl_pct = weighted_absorbed_cl_pct / N
			 *   gate: avg_path_cl_pct >= collapsed_cl_pct
			 *   i.e., weighted_absorbed_cl_pct
			 *         >= collapsed_cl_pct × N
			 *
			 * Both sides are already in pct units, so no extra
			 * cross-multiplication is needed.
			 *
			 * Non-strict (>=): a tie means the read-side
			 * latency is unchanged on average, while the
			 * (strict) footprint ratio gate above can still
			 * accept the candidate for the cache-line / arena
			 * footprint win.  Reject only collapses that would
			 * strictly increase per-path latency.
			 *
			 * Without this safety gate, the footprint ratio
			 * alone accepts collapses that consolidate many
			 * tiny absorbed paths (e.g., long chains of low-CL
			 * skip-compresseds and 1-child internals) into a
			 * multi-CL collapsed scan even when the absorbed
			 * lookup was strictly cheaper per query.
			 */
			{
				unsigned int nr_emit = entry_idx;

				if ((unsigned long) weighted_absorbed_cl_pct <
				    (unsigned long) collapsed_cl_pct * nr_emit) {
					free_collapsed_node_unpublished(ft, col);
					continue;
				}
			}

			/*
			 * Footprint ratio check: candidate must clear
			 * the per-trie threshold
			 *   absorbed * 100 > collapsed * threshold_pct
			 * (threshold_pct == 100 reproduces the historical
			 * "absorbed > collapsed" gate) and beat the current
			 * best.  Cross-multiply to avoid division.  The
			 * best-candidate comparison itself is independent
			 * of threshold_pct: it picks the largest ratio
			 * among the candidates that already cleared the
			 * gate.
			 */
			if ((unsigned long) absorbed * 100UL >
			    (unsigned long) collapsed_fp * threshold_pct &&
			    (unsigned long) absorbed * best_collapsed >
			    (unsigned long) best_absorbed * collapsed_fp) {
				if (best_col)
					free_collapsed_node_unpublished(ft, best_col);
				best_col = col;
				best_meta = col_meta;
				best_tier = tier_ci;
				best_stride = stride_si;
				best_nr_entries = entry_idx;
				best_absorbed = absorbed;
				best_collapsed = collapsed_fp;
				memcpy(absorbed_best, absorbed_cur,
					nr_absorbed_cur * sizeof(absorbed_best[0]));
				memcpy(absorbed_depths_best, absorbed_depths_cur,
					nr_absorbed_cur * sizeof(absorbed_depths_best[0]));
				nr_absorbed_best = nr_absorbed_cur;
			} else {
				free_collapsed_node_unpublished(ft, col);
			}
		}

		if (!best_col)
			return NULL;

		if (ft_debug_counters())
			fprintf(stderr, "COLLAPSE: tier=%u stride=%u entries=%u "
				"fp_ratio=%.2f (absorbed=%u collapsed=%u)\n",
				best_tier, best_stride,
				best_nr_entries,
				(double) best_absorbed / best_collapsed,
				best_absorbed, best_collapsed);

		best_meta->nr_child = best_nr_entries;
		ft_nr_keys_store(ft, best_meta, ft_nr_keys_get(metadata),
			CMM_RELAXED);
		ft_metadata_set_external_nodes(
			ft_collapsed_node_flag_tier_stride(best_col, best_tier, best_stride),
			best_meta, metadata->external_nodes);

		{
			struct cds_ft_inode_flag *col_flag =
				ft_collapsed_node_flag_tier_stride(best_col, best_tier, best_stride);
			unsigned int e;

			for (e = 0; e < best_nr_entries; e++) {
				struct cds_ft_inode_flag *cf;
				struct cds_ft_inode_flag **child_addr;

				if (best_stride == FT_COL_STRIDE_NARROW) {
					struct cds_ft_collapsed_entry *be =
						&ft_collapsed_entries(best_col, best_tier)[e];
					cf = be->child;
					child_addr = &be->child;
				} else {
					struct cds_ft_collapsed_entry_wide *be =
						&ft_collapsed_entries_wide(best_col, best_tier)[e];
					cf = be->child;
					child_addr = &be->child;
				}

				/*
				 * Re-publish compressed entries as skip
				 * pointers.
				 *
				 * The collapse walk resolves skip pointers
				 * to compressed flags, so entries store
				 * compressed_flag.  Convert back to skip
				 * pointer to preserve the optimization.
				 */
				if (ft_node_compressed(cf) &&
				    ft_group_skip_compressed(ft->group)) {
					struct cds_ft_compressed_node *cn =
						ft_compressed_node_ptr(cf);
					if (cn->len <= FT_SKIP_LEN_MAX) {
						cf = ft_skip_compressed_flag(
							cn->child, cn->len);
						*child_addr = cf;
					}
				}
				ft_set_parent(cf, col_flag, child_addr);
			}
			memcpy(absorbed_out, absorbed_best,
				nr_absorbed_best * sizeof(absorbed_out[0]));
			memcpy(absorbed_depths_out, absorbed_depths_best,
				nr_absorbed_best * sizeof(absorbed_depths_out[0]));
			*nr_absorbed_out = nr_absorbed_best;
			return col_flag;
		}
	}
}

/*
 * ft_free_absorbed_node: free a single node that was absorbed during
 * a collapse walk, dispatching to the correct free function based on
 * the node type encoded in the flag.
 */
static
void ft_free_absorbed_node(struct cds_ft *ft,
		struct cds_ft_inode_flag *node_flag)
{
	if (ft_node_compressed(node_flag))
		free_compressed_node(ft,
			ft_compressed_node_ptr(node_flag));
	else if (ft_node_collapsed(node_flag))
		free_collapsed_node(ft,
			ft_collapsed_node_ptr(node_flag));
	else if (ft_node_internal(node_flag))
		free_cds_ft_node(ft, ft_node_ptr(node_flag));
}

/*
 * ft_check_collapse_on_path: after a mutation, walk the key path from
 * root to leaf through the LIVE trie in three passes.  This sees all
 * nodes including freshly created junctions from compressed splits,
 * which may not be in the snapshot.
 *
 * Pass 1 (dissolve, skip-compress mode only): explode any collapsed
 * node on the path back into structured internals so subsequent
 * passes see explicit shape.
 *
 * Pass 2 (chain-compress): at every visited internal, fold chains
 * of single-child no-external-nodes internals into compressed (or
 * skip-encoded compressed) nodes.  Self-compress when the visited
 * node is itself a chain head (except the root, which must stay
 * internal); otherwise iterate immediate children and compress each
 * chain-head child to canonicalize the absorbed footprint pass 3
 * will see.
 *
 * Pass 3 (collapse-attempt): evaluate each internal node on the
 * canonicalized path for collapse acceptance.  When an internal node
 * qualifies, it is replaced with a collapsed node before continuing
 * deeper.  The walk then descends into the collapsed node's matching
 * entry, naturally handling multi-level collapse without a separate
 * collection pass.  Skipped at CDS_FT_COLLAPSE_THRESHOLD_DISABLED;
 * passes 1 and 2 still run to keep canonical form across
 * insert+remove sequences regardless of acceptance threshold.
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

#ifdef FEATURE_FT_COMPRESS
	/*
	 * Pass 1+2 (canonicalization): walk the mutation path top-down,
	 * dissolving collapseds (skip-compress mode only) and
	 * opportunistically compressing chains of single-child
	 * no-external-nodes internals into compressed (or skip-encoded
	 * compressed) nodes.  Pass 3's collapse-acceptance gate then
	 * evaluates against canonical layout — its absorbed_footprint
	 * count is no longer inflated by uncompressed chains, so it
	 * makes the same decision a fresh-canonical trie would make.
	 *
	 * Pass 2 (chain-compress) runs at every visited internal:
	 *
	 *   - If the visited node itself is a chain head (single-child
	 *     no-external-nodes), compress its chain in place; the
	 *     replacement compressed becomes the new visited node.
	 *
	 *   - Otherwise, iterate its children and compress each child
	 *     that is a chain head.  This canonicalizes one step out
	 *     from the descent path so pass 3's absorbed_footprint
	 *     counts neighbors at their canonical (compressed) cost,
	 *     not at chain cost.
	 *
	 * Pass 1 (dissolve) is gated on ft_group_skip_compressed.
	 * Without skip-compress the cost lattice is monotonic
	 * (collapsed strictly cheaper than the chain it absorbs),
	 * greedy collapse is correct, and dissolve adds pure
	 * write-side overhead with no read-side benefit.
	 */
	{
		bool can_dissolve = ft_group_skip_compressed(ft->group);
		struct cds_ft_inode_flag *p1_nf;
		struct cds_ft_inode_flag **p1_slot;
		struct cds_ft_inode_flag *p1_pnf = NULL;
		const uint8_t *p1_ik = key;
		unsigned int p1_depth = 0;

		p1_nf = ft_dereference_prefetch(ft->root);
		p1_slot = &ft->root;

		while (p1_depth < key_len + 1) {
			/*
			 * Resolve skip-encoded compressed up-front: a slot may
			 * hold a skip-encoded pointer whose low tag bits inherit
			 * the wrapped child's tag, so untreated dispatch via
			 * ft_node_compressed/ft_node_external would mis-classify.
			 */
			p1_nf = ft_resolve_skip_compressed(p1_nf);
			if (!p1_nf)
				break;
			if (ft_node_external(p1_nf))
				break;
			if (ft_node_compressed(p1_nf)) {
				struct cds_ft_compressed_node *cn;
				unsigned int remaining;
				unsigned int cmp;
				unsigned int j;

				/*
				 * Self-canonicalize the compressed: if its
				 * child is itself a chain head (or skip-/
				 * compressed), absorb it into a single bigger
				 * compressed.  Without this, a chain head
				 * emerging beneath a compressed could not be
				 * canonicalized via ft_compress_chain_at on
				 * the chain head itself (the function refuses
				 * to publish a compressed adjacent to a
				 * compressed parent, which would produce two
				 * adjacent compresseds and a double-wrapped
				 * skip pointer in skip mode).
				 */
				{
					int r = ft_compress_chain_at(ft, p1_nf,
						p1_pnf, p1_slot, p1_depth);

					if (r < 0)
						return;
					if (r > 0) {
						p1_nf = ft_dereference_acquire(
							*p1_slot);
						continue;
					}
				}
				cn = ft_compressed_node_ptr(p1_nf);
				remaining = key_len - p1_depth;
				cmp = cn->len < remaining ? cn->len : remaining;
				j = ft_match_compressed_key(p1_ik, cn, cmp);
				if (j < cmp)
					break;
				p1_slot = &cn->child;
				p1_pnf = p1_nf;
				p1_nf = ft_dereference_acquire_prefetch(
					cn->child);
				p1_depth += cn->len;
				p1_ik += cn->len;
				continue;
			}
			if (ft_node_collapsed(p1_nf)) {
				struct cds_ft_collapsed_node *col;
				struct cds_ft_inode_flag *internal_flag;
				int err;

				if (!can_dissolve)
					break;
				col = ft_collapsed_node_ptr(p1_nf);
				err = ft_collapsed_explode_node(ft, col,
					p1_nf, p1_pnf, p1_slot,
					p1_depth, &internal_flag);
				if (err < 0)
					return;
				p1_nf = internal_flag;
				/* Re-process at same depth. */
				continue;
			}
			/* Internal node. */
			{
				struct cds_ft_metadata *m =
					cds_ft_item_to_metadata(
						ft_node_ptr(p1_nf));
				bool is_root = (p1_pnf == NULL);
				bool is_chain_head =
					(m->nr_child == 1 && !m->external_nodes);

				/*
				 * Self-compress: visited node is itself a
				 * chain head.  After replace, p1_nf becomes
				 * the new compressed; re-enter the loop to
				 * walk through its bytes.  Skipped at root
				 * because root must stay internal.
				 */
				if (is_chain_head && !is_root) {
					int r = ft_compress_chain_at(ft,
						p1_nf, p1_pnf, p1_slot,
						p1_depth);

					if (r < 0)
						return;
					if (r > 0) {
						p1_nf = ft_dereference_acquire(
							*p1_slot);
						continue;
					}
					/* r == 0: chain too short, fall through. */
				}
				/*
				 * Iterate children and chain-compress each
				 * chain-head internal child.  Compressed
				 * children are already canonical; collapsed
				 * and external children are not chain heads.
				 * Skip-encoded children are resolved by
				 * ft_node_get_direction to their underlying
				 * compressed.
				 */
				{
					int prev = -1;
					uint8_t k = 0;
					struct cds_ft_inode_flag *c;

					c = ft_node_get_direction(p1_nf,
						prev, &k, FT_RIGHT);
					while (c) {
						if (ft_node_internal(c)) {
							struct cds_ft_metadata *cm =
								cds_ft_item_to_metadata(
									ft_node_ptr(c));

							if (cm->nr_child == 1
							    && !cm->external_nodes) {
								struct cds_ft_inode_flag **child_slot = NULL;

								ft_node_get_nth(p1_nf,
									&child_slot, k,
									FT_PF_NONE);
								if (child_slot) {
									int r = ft_compress_chain_at(
										ft, c, p1_nf,
										child_slot,
										p1_depth + 1);
									if (r < 0)
										return;
								}
							}
						}
						prev = (int) k;
						c = ft_node_get_direction(p1_nf,
							prev, &k, FT_RIGHT);
					}
				}
			}
			/* Descend by key byte. */
			{
				uint8_t kv = *(p1_ik++);
				struct cds_ft_inode_flag **slot = NULL;
				struct cds_ft_inode_flag *child;

				child = ft_node_get_nth(p1_nf, &slot, kv,
					FT_PF_NONE);
				if (!slot || !child)
					break;
				p1_slot = slot;
				p1_pnf = p1_nf;
				p1_nf = child;
				p1_depth++;
			}
		}
	}
#endif

	/*
	 * Pass 3 (collapse-attempt) is the only stage that depends on
	 * the threshold.  Pass 1+2 (canonicalization) above always run
	 * so insert+remove sequences and fresh inserts converge to the
	 * same skip-encoded canonical form regardless of whether
	 * collapse is enabled.
	 */
	if (uatomic_load(&ft->collapse_threshold_pct, CMM_RELAXED)
	    == CDS_FT_COLLAPSE_THRESHOLD_DISABLED)
		return;

	node_flag = ft_dereference_prefetch(ft->root);
	parent_slot = &ft->root;

	while (depth < key_len + 1) {
		if (!node_flag)
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

			j = ft_match_compressed_key(ik, cn, cmp);
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
			unsigned int tier = ft_collapsed_tier(node_flag);
			unsigned int stride = ft_collapsed_stride(node_flag);
			struct cds_ft_collapsed_node *col =
				ft_collapsed_node_ptr(node_flag);
			unsigned int cap = ft_collapsed_capacity_stride(tier, stride);
			unsigned int remaining = key_len - depth;
			unsigned int e;
			bool found = false;

			if (stride == FT_COL_STRIDE_NARROW) {
				struct cds_ft_collapsed_entry *entries =
					ft_collapsed_entries(col, tier);

				for (e = 0; e < cap; e++) {
					struct cds_ft_collapsed_entry *entry = &entries[e];
					struct cds_ft_inode_flag *entry_child;
					unsigned int slen, j;
					uint8_t *suffix;

					entry_child = ft_dereference_acquire_prefetch(entry->child);
					if (entry_child == NULL)
						continue;
					slen = ft_collapsed_suffix_len(entry);
					if (slen > remaining)
						continue;
					suffix = ft_collapsed_suffix(entry);
					for (j = 0; j < slen; j++) {
						if (ik[j] != suffix[j])
							break;
					}
					if (j == slen) {
						entry_child = ft_resolve_skip_compressed(entry_child);
						parent_slot = &entry->child;
						parent_nf = node_flag;
						node_flag = entry_child;
						depth += slen;
						ik += slen;
						found = true;
						break;
					}
				}
			} else {
				struct cds_ft_collapsed_entry_wide *entries =
					ft_collapsed_entries_wide(col, tier);

				for (e = 0; e < cap; e++) {
					struct cds_ft_collapsed_entry_wide *entry = &entries[e];
					struct cds_ft_inode_flag *entry_child;
					unsigned int slen, j;

					entry_child = ft_dereference_acquire_prefetch(entry->child);
					if (entry_child == NULL)
						continue;
					slen = entry->len;
					if (slen > remaining)
						continue;
					for (j = 0; j < slen; j++) {
						if (ik[j] != entry->suffix[j])
							break;
					}
					if (j == slen) {
						entry_child = ft_resolve_skip_compressed(entry_child);
						parent_slot = &entry->child;
						parent_nf = node_flag;
						node_flag = entry_child;
						depth += slen;
						ik += slen;
						found = true;
						break;
					}
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
			struct cds_ft_inode_flag *col_absorbed[FT_COLLAPSE_ABSORBED_MAX];
			unsigned int col_absorbed_depths[FT_COLLAPSE_ABSORBED_MAX];
			unsigned int nr_col_absorbed;

			col_flag = ft_try_collapse_at_node(ft, node_flag,
					depth, col_absorbed, col_absorbed_depths,
					&nr_col_absorbed);
			if (col_flag) {
				unsigned int ai;
				/*
				 * Save old decision point's density profile
				 * and footprint before freeing.
				 */
				unsigned int old_fp =
					ft_node_readside_footprint(ft, node_flag);
				unsigned long old_density[FT_NODE_DENSITY_DEPTH];
				{
					struct cds_ft_metadata *old_meta =
						cds_ft_item_to_metadata(
							ft_node_ptr(node_flag));
					unsigned int di;

					for (di = 0; di < FT_NODE_DENSITY_DEPTH; di++)
						old_density[di] = ft_density_get(old_meta, di);
				}

				ft_set_parent(col_flag, parent_nf, NULL);
				ft_publish_to_parent(ft, parent_nf,
					parent_slot, col_flag);
				/*
				 * Collapse installs the new collapsed node
				 * in the parent's slot via direct publish,
				 * bypassing ft_node_set_nth.  Emit the
				 * structural edge so trace consumers see the
				 * parent -> collapsed attachment.  When
				 * parent_nf is NULL, the slot is &ft->root
				 * and root_publish already fires from inside
				 * ft_publish_to_parent.
				 */
				if (parent_nf && depth >= 1)
					FT_TP(tree_edge_set, (const void *) ft,
						(const void *) parent_nf,
						(unsigned int) (depth - 1),
						(uint8_t) key[depth - 1],
						(const void *) col_flag);
				/*
				 * Free intermediate nodes absorbed by the
				 * collapse.  Done after publish so readers
				 * see the new path; actual frees are
				 * deferred via call_rcu.
				 */
				for (ai = 0; ai < nr_col_absorbed; ai++)
					ft_free_absorbed_node(ft,
						col_absorbed[ai]);
				free_cds_ft_node(ft, ft_node_ptr(node_flag));
				/*
				 * Entry reparenting (parent + skip_slot) was
				 * already done inside ft_try_collapse_at_node
				 * after skip-compressed conversion.
				 */
				/*
				 * Recompute density for the new collapsed
				 * node: the absorbed intermediate nodes are
				 * gone, only entries' children remain.
				 */
				ft_init_node_density(ft, col_flag);
				ft_propagate_density_replace(ft,
					col_flag, depth,
					old_density, old_fp,
					cds_ft_item_to_metadata(
						ft_node_ptr(col_flag)),
					ft_node_readside_footprint(ft, col_flag),
					NULL, 0);
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

			child = ft_node_get_nth(node_flag, &slot, kv, FT_PF_NONE);
			if (!slot || !child)
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
	if (suffix_len >= 2) {
		struct cds_ft_compressed_node *sfx;
		struct cds_ft_metadata *sfx_meta;

		sfx = alloc_compressed_node(ft, suffix_len, &sfx_meta);
		if (!sfx) goto error;
		sfx->child = cn->child;
		sfx->len = suffix_len;
		memcpy(sfx->key_bytes, &cn->key_bytes[diverge_pos + 1], suffix_len);
		sfx_meta->nr_child = 1;
		ft_nr_keys_store(ft, sfx_meta, old_child_nr_keys, CMM_RELAXED);
		old_suffix_flag = ft_compressed_node_flag(sfx);
		ft_set_parent(cn->child, old_suffix_flag, &sfx->child);
		old_suffix_flag = ft_publish_compressed(ft, sfx, old_suffix_flag);
		created[nr_created++] = old_suffix_flag;
	} else if (suffix_len == 1) {
		struct cds_ft_inode_flag *dest = NULL;

		ret = ft_node_set_nth(ft, &dest, cn->key_bytes[diverge_pos + 1],
				cn->child, NULL, NULL, junction_depth + 1);
		if (ret) goto error;
		{
			struct cds_ft_metadata *m =
				cds_ft_item_to_metadata(ft_node_ptr(dest));
			ft_nr_keys_store(ft, m, old_child_nr_keys, CMM_RELAXED);
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
		ft_nr_keys_store(ft, nb_meta, 1, CMM_RELAXED);
		new_branch_flag = ft_compressed_node_flag(nb);
		ft_set_parent(nb->child, new_branch_flag, NULL);
		new_branch_flag = ft_publish_compressed(ft, nb, new_branch_flag);
		created[nr_created++] = new_branch_flag;
	} else if (new_len == 1) {
		struct cds_ft_inode_flag *dest = NULL;

		ret = ft_node_set_nth(ft, &dest,
				iter_key[diverge_pos + 1],
				(struct cds_ft_inode_flag *) child_node, NULL, NULL,
				junction_depth + 1);
		if (ret) goto error;
		{
			struct cds_ft_metadata *m =
				cds_ft_item_to_metadata(ft_node_ptr(dest));
			ft_nr_keys_store(ft, m, 1, CMM_RELAXED);
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
		ft_nr_keys_store(ft, branch_meta, old_child_nr_keys + 1,
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
			ft_nr_keys_store(ft, pfx_meta, ft_nr_keys_get(cn_meta) + 1, CMM_RELAXED);
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
			ft_nr_keys_store(ft, pfx_meta, ft_nr_keys_get(cn_meta) + 1, CMM_RELAXED);
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
			ft_nr_keys_store(ft, int_meta, ft_nr_keys_get(cn_meta) + 1, CMM_RELAXED);
			if (cn_meta->external_nodes)
				ft_metadata_set_external_nodes(dest, int_meta, cn_meta->external_nodes);
			top_flag = dest;
			created[nr_created++] = dest;
		}
	prefix_done:
		(void) 0;
	} else if (diverge_pos == 1) {
		struct cds_ft_inode_flag *dest = NULL;
		struct cds_ft_metadata *pfx_meta;

		ret = ft_node_set_nth(ft, &dest, cn->key_bytes[0],
				branch_flag, NULL, NULL, node_depth);
		if (ret) goto error;
		pfx_meta = cds_ft_item_to_metadata(ft_node_ptr(dest));
		ft_nr_keys_store(ft, pfx_meta, ft_nr_keys_get(cn_meta) + 1, CMM_RELAXED);
		if (cn_meta->external_nodes)
			ft_metadata_set_external_nodes(dest, pfx_meta, cn_meta->external_nodes);
		top_flag = dest;
		created[nr_created++] = dest;
	} else {
		/* diverge_pos == 0: branch IS the top. */
		struct cds_ft_metadata *branch_meta =
			cds_ft_item_to_metadata(ft_node_ptr(branch_flag));
		ft_nr_keys_store(ft, branch_meta, ft_nr_keys_get(cn_meta) + 1,
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

	/*
	 * 6. Density: initialize created nodes bottom-up, then
	 * propagate the per-level contribution change from the old
	 * compressed node to the new top node (prefix, junction, or
	 * branch depending on diverge_pos) to all ancestors.
	 */
	{
		int ci;

		for (ci = 0; ci < nr_created; ci++)
			ft_init_node_density(ft, created[ci]);
	}

	/*
	 * Density propagation is done by the caller after ft_set_parent
	 * establishes the top node's parent pointer.
	 */

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
		ft_nr_keys_store(ft, sfx_meta, child_nr_keys,
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
			ft_nr_keys_store(ft, m, child_nr_keys,
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
		ft_nr_keys_store(ft, jct_meta, child_nr_keys,
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
		ft_nr_keys_store(ft, pfx_meta, ft_nr_keys_get(cn_meta),
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
			ft_nr_keys_store(ft, pfx_meta, ft_nr_keys_get(cn_meta),
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
			ft_nr_keys_store(ft, int_meta, ft_nr_keys_get(cn_meta),
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
		ft_nr_keys_store(ft, pfx_meta, ft_nr_keys_get(cn_meta),
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
			ft_nr_keys_store(ft, jct_meta,
				ft_nr_keys_get(jct_meta) + 1, CMM_RELAXED);
		}
		top_flag = jct_flag;
	}

	/*
	 * Initialize density counters on all created nodes,
	 * bottom-up (children first).
	 */
	{
		int ci;

		for (ci = 0; ci < nr_created; ci++)
			ft_init_node_density(ft, created[ci]);
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

#ifdef FEATURE_FT_COLLAPSE
static
int ft_collapsed_recompact_with_insert(struct cds_ft *ft,
		struct cds_ft_collapsed_node *col,
		struct cds_ft_inode_flag *col_flag,
		struct cds_ft_inode_flag *parent_nf,
		struct cds_ft_inode_flag **slot,
		unsigned int depth,
		const uint8_t *new_suffix,
		unsigned int new_slen,
		struct cds_ft_inode_flag *new_branch,
		struct cds_ft_inode_flag **out_replacement_flag);
#endif

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
	ft_nr_keys_store(ft, cn_meta, 1, CMM_RELAXED);
	/* Compressed nodes must not carry external_nodes. */
	assert(!external_nodes);
	{
		struct cds_ft_inode_flag *cflag = ft_compressed_node_flag(cn);
		ft_set_parent(child, cflag, &cn->child);
		ft_init_node_density(ft, cflag);
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

#ifdef FEATURE_FT_COLLAPSE
static struct cds_ft_inode_flag *ft_build_ordinal_chain(struct cds_ft *ft,
		const uint8_t *ordinals, unsigned int len,
		struct cds_ft_inode_flag *child,
		unsigned long nr_keys,
		unsigned int base_depth);
#endif

static struct cds_ft_inode_flag *ft_explode_entries(struct cds_ft *ft,
		struct cds_ft_collapsed_node *col,
		unsigned int tier,
		unsigned int stride,
		unsigned int start, unsigned int end,
		unsigned int suffix_offset,
		unsigned int collapse_depth);

/*
 * Publish a freshly-created branch under a collapsed parent in
 * ft_attach_node's publish phase.
 *
 * The collapsed entry's suffix already covers the path from the
 * collapsed node to the child depth — only the child itself
 * changes.  Exploding the collapsed node and using ft_node_set_nth
 * would attach at the wrong depth (last suffix byte vs. first-byte
 * level), overwriting sibling entries that share the same first
 * suffix byte; instead, write the branch directly at the entry's
 * child-pointer slot.
 *
 * Collapsed-parent attach bypasses ft_node_set_nth, so emit the
 * structural edge tracepoint inline: the collapsed node sits at
 * level - 1 and dispatches on key_value to iter_node_flag.
 */
#ifdef FEATURE_FT_COLLAPSE
static
void ft_attach_node_publish_collapsed_parent(struct cds_ft *ft,
		struct cds_ft_inode_flag *attach_node_flag,
		struct cds_ft_inode_flag **old_node_flag_ptr,
		struct cds_ft_inode_flag *iter_node_flag,
		unsigned int level __attribute__((unused)),
		uint8_t key_value __attribute__((unused)))
{
	ft_set_parent(iter_node_flag, attach_node_flag, old_node_flag_ptr);
	ft_publish_to_parent(ft, attach_node_flag,
		old_node_flag_ptr, iter_node_flag);
	FT_TP(tree_edge_set, (const void *) ft,
		(const void *) attach_node_flag,
		(unsigned int) (level - 1),
		(uint8_t) key_value,
		(const void *) iter_node_flag);
}
#endif /* FEATURE_FT_COLLAPSE */

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

	/* Concurrent update prevented by mutual exclusion. */
	assert(!(attach_node_flag_ptr && ft_node_ptr(*attach_node_flag_ptr) !=
			ft_node_ptr(attach_node_flag)));

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
				ft_nr_keys_store(ft, branch_meta, 1, CMM_RELAXED);
			}
			created_nodes[nr_created_nodes++] = iter_dest_node_flag;
			iter_node_flag = iter_dest_node_flag;
		}

		if (external_nodes) {
			struct cds_ft_metadata *iter_node_metadata;

			iter_node_metadata = cds_ft_item_to_metadata(ft_node_ptr(iter_node_flag));
			ft_metadata_set_external_nodes(iter_node_flag, iter_node_metadata, external_nodes);
			ft_nr_keys_store(ft, iter_node_metadata,
				ft_nr_keys_get(iter_node_metadata) + 1, CMM_RELAXED);
		}
	}

	/* Publish branch. */
	{
		uint8_t key_value;

		key_value = *(--iter_key);
		dbg_printf("publish branch at level %d, key %u\n", level - 1, (unsigned int) key_value);

#ifdef FEATURE_FT_COLLAPSE
		/* Collapsed parent: bypass ft_node_set_nth (see helper). */
		if (attach_node_flag &&
		    ft_node_collapsed(attach_node_flag)) {
			ft_attach_node_publish_collapsed_parent(ft,
				attach_node_flag, old_node_flag_ptr,
				iter_node_flag, level, key_value);
			goto publish_done;
		}
#endif

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
#ifdef FEATURE_FT_COLLAPSE
publish_done:
#endif

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
			ft_init_node_density(ft, created_nodes[cn_idx]);
		top_meta = ft_flag_to_metadata(top_node);
		ft_propagate_density_replace(ft, top_node, level,
			NULL, 0,
			top_meta, ft_node_readside_footprint(ft, top_node),
			NULL, 0);
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
		ft_nr_keys_store(ft, br_meta, 1, CMM_RELAXED);
		ft_init_node_density(ft, branch);
	}
	ft_set_parent(branch, d->nf, &cn->child);
	ft_publish_to_parent(ft, d->nf, &cn->child, branch);
	/*
	 * Propagate density: the old child was an external (zero
	 * footprint, zero density).  The new branch has its own
	 * footprint plus subtree density from ft_init_node_density.
	 * Use ft_propagate_density_replace to propagate the full
	 * density profile, not just the node's own footprint.
	 */
	ft_propagate_density_replace(ft, branch, d->depth + cn->len,
		NULL, 0,
		br_meta, ft_node_readside_footprint(ft, branch),
		NULL, 0);
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
	/* Save old compressed node's density before split frees it. */
	unsigned long old_cn_density[FT_NODE_DENSITY_DEPTH];
	unsigned int old_cn_fp = ft_node_readside_footprint(ft, d->nf);
	{
		struct cds_ft_metadata *old_meta =
			cds_ft_item_to_metadata(
				ft_node_ptr(d->nf));
		unsigned int di;

		for (di = 0; di < FT_NODE_DENSITY_DEPTH; di++)
			old_cn_density[di] = ft_density_get(old_meta, di);
	}

	dret = ft_split_compressed_insert(ft,
		d->nfp, d->nf, iter_key, remaining,
		j, node, d->depth);
	if (dret)
		return dret;
	ft_set_parent(*d->nfp, d->pnf, d->nfp);

	/*
	 * Propagate per-level density change: old compressed node
	 * replaced by the new top node at the same depth.
	 */
	{
		struct cds_ft_inode_flag *top = *d->nfp;

		ft_propagate_density_replace(ft,
			top, d->depth,
			old_cn_density, old_cn_fp,
			ft_flag_to_metadata(top),
			ft_node_readside_footprint(ft, top),
			NULL, 0);
	}

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
	/* Save old compressed density before split frees it. */
	unsigned long old_cn_density[FT_NODE_DENSITY_DEPTH];
	unsigned int old_cn_fp = ft_node_readside_footprint(ft, d->nf);
	{
		struct cds_ft_metadata *old_meta =
			cds_ft_item_to_metadata(ft_node_ptr(d->nf));
		unsigned int di;

		for (di = 0; di < FT_NODE_DENSITY_DEPTH; di++)
			old_cn_density[di] = ft_density_get(old_meta, di);
	}

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
	/*
	 * Density: use per-level propagation.  The top node (which
	 * may be the junction or a prefix above it) replaces the
	 * old compressed node at d->depth.
	 */
	ft_propagate_density_replace(ft,
		top_flag, d->depth,
		old_cn_density, old_cn_fp,
		ft_flag_to_metadata(top_flag),
		ft_node_readside_footprint(ft, top_flag),
		NULL, 0);
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
		 * compressed, collapsed, or skip-compressed (which
		 * encodes another compressed node deeper in the
		 * chain). */
		if (cn->child &&
		    (ft_node_skip_compressed(cn->child) ||
		     ft_node_internal(cn->child) ||
		     ft_node_compressed(cn->child) ||
		     ft_node_collapsed(cn->child))) {
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

#ifdef FEATURE_FT_COLLAPSE
/*
 * Handle a collapsed node in _cds_ft_insert's descent loop.
 *
 * Two passes over the collapsed entries:
 *
 *   Pass 1 — full-suffix match scan: for each live entry, compare
 *     its suffix prefix-by-prefix against the remaining key.  On
 *     full suffix match (j == slen && slen <= remaining): snapshot,
 *     advance d and *iter_key_p past the matched span, return
 *     FT_DESCENT_CONTINUE so the caller continues the descent.
 *     Track the longest partial-prefix match (suffix shares a
 *     non-zero prefix with the key but diverges before slen) for
 *     the explode decision below.
 *
 *   Pass 2 — outcome dispatch (no full match found):
 *     - prefix_match_entry >= 0: go to collapsed_explode (a
 *       partial-prefix conflict means the new key cannot be added
 *       in-place without creating ambiguous suffixes).
 *     - tombstone with identical suffix exists: revive it
 *       (clear the tombstone bit, install the new child); set
 *       *ret_p = 0 and return FT_DESCENT_END so the caller goes
 *       to insert_done.
 *     - in-place fit: append a new entry; *ret_p = 0,
 *       FT_DESCENT_END.
 *     - no fit (header/suffix collision OR max entries reached):
 *       goto collapsed_explode.
 *
 *   collapsed_explode: convert the collapsed node to internal +
 *     compressed via ft_explode_entries, propagate density, set
 *     d->nf = internal_flag, return FT_DESCENT_CONTINUE so the
 *     caller's next iteration re-dispatches via the regular
 *     internal-node path.  On allocation failure: *ret_p = -ENOMEM,
 *     FT_DESCENT_END.
 *
 *   ft_build_branch failures during tombstone-reuse / in-place-add
 *   paths: *ret_p = -ENOMEM, FT_DESCENT_END.
 */
static
enum ft_descent_action ft_insert_collapsed(struct cds_ft *ft,
		struct ft_descent *d, const uint8_t **iter_key_p,
		const uint8_t *key, size_t key_len,
		unsigned int key_depth,
		struct cds_ft_node *node,
		struct cds_ft_inode_flag **snapshot,
		unsigned int *snapshot_depth,
		int *nr_snapshot_p,
		int *ret_p)
{
	unsigned int tier = ft_collapsed_tier(d->nf);
	unsigned int stride = ft_collapsed_stride(d->nf);
	struct cds_ft_collapsed_node *col = ft_collapsed_node_ptr(d->nf);
	unsigned int cap = ft_collapsed_capacity_stride(tier, stride);
	unsigned int stride_suffix_max = ft_col_stride_suffix_max[stride];
	unsigned int remaining = key_depth - 1 - d->depth;
	const uint8_t *iter_key = *iter_key_p;
	int prefix_match_entry = -1;
	unsigned int prefix_match_len = 0;
	unsigned int new_slen;
	unsigned int high_water;
	struct cds_ft_inode_flag *branch;
	bool last_live_seen = false;
	int last_live_cmp = -1;	/* sign of new key vs last live entry */
	struct ft_col_view view;
	unsigned int e;

	ft_for_each_live_collapsed_entry_view_rcu(col, tier, stride, e, view) {
		struct cds_ft_inode_flag *child = view.child;
		unsigned int slen = view.slen;
		unsigned int j;
		struct cds_ft_inode_flag **child_addr;

		/* Check for prefix match. */
		{
			unsigned int cmp_len = slen < remaining ? slen : remaining;
			int rc;

			rc = ft_key_cmp_ordinals(iter_key, view.suffix,
					cmp_len, cmp_len, false, &j);
			if (rc != 0) {
				if (j == 0) {
					/* No prefix overlap.  Track ordering
					 * vs. last live for sorted-append
					 * eligibility. */
					last_live_seen = true;
					last_live_cmp = rc;
					continue;
				}
			} else {
				j = cmp_len; /* Full match up to cmp_len. */
			}

			if (j == slen && slen <= remaining) {
				/* Full suffix match: traverse through. */
				if (stride == FT_COL_STRIDE_NARROW)
					child_addr = &ft_collapsed_entries(col, tier)[e].child;
				else
					child_addr = &ft_collapsed_entries_wide(col, tier)[e].child;
				ft_snapshot_push(snapshot, snapshot_depth,
					*nr_snapshot_p, d->nf, d->depth);
				d->ppnf  = d->pnf;
				d->ppnfp = d->pnfp;
				d->pnf   = d->nf;
				d->pnfp  = d->nfp;
				d->nf    = child;
				d->nfp   = child_addr;
				d->depth += slen;
				iter_key += slen;
				*iter_key_p = iter_key;
				return FT_DESCENT_CONTINUE;
			}
			/*
			 * Partial prefix match: the new key shares j bytes
			 * with this entry's suffix but diverges after that.
			 * Record the best (longest) prefix match for possible
			 * entry splitting.
			 */
			if (j > prefix_match_len) {
				prefix_match_entry = (int) e;
				prefix_match_len = j;
			}
			last_live_seen = true;
			last_live_cmp = rc != 0 ? rc : 1;
		}
	}
	/*
	 * Partial prefix match: explode the collapsed node to
	 * avoid creating conflicting entries.  Fall through to the
	 * explode-or-break path which converts to an internal node,
	 * then the normal insert logic handles the remaining key.
	 */
	if (prefix_match_entry >= 0)
		goto collapsed_explode;

	/*
	 * No matching entry.  Compute the high-water slot index
	 * (one past the highest-index live or once-live slot).
	 * SoA slots are append-only-no-reuse, so a fresh insert
	 * lands at high_water.
	 *
	 * The new entry must sort strictly after the highest live
	 * entry (lexicographic by suffix bytes); otherwise the
	 * sorted-slot invariant is violated and we must rebuild —
	 * fall back to explode+rebuild.
	 */
	new_slen = remaining;
	high_water = ft_collapsed_high_water(col, tier, stride);

	/*
	 * Suffix length must be in [SUFFIX_MIN..stride_suffix_max].  The
	 * lower bound preserves the SIMD path invariant that every live
	 * entry has a valid prefix_1 byte; the upper bound is the per-
	 * stride per-entry cap (7 narrow, 23 wide).  Anything outside
	 * falls through to explode.
	 */
	if (new_slen < FT_COLLAPSE_SUFFIX_MIN || new_slen > stride_suffix_max)
		goto collapsed_explode;

	{
		struct cds_ft_metadata *col_meta = cds_ft_item_to_metadata(
				(struct cds_ft_inode *) col);
		bool need_recompact = (high_water >= cap) ||
				(last_live_seen && last_live_cmp <= 0);

		/*
		 * Pre-check recompact viability before allocating a child
		 * branch: if the tier is full of live entries, neither
		 * in-place sorted-append nor recompact can fit a new entry,
		 * and we must explode.
		 */
		if (need_recompact && (unsigned int) col_meta->nr_child + 1 > cap)
			goto collapsed_explode;

		/*
		 * Build child for the new entry.  The suffix covers key
		 * bytes from d->depth to key_len.  If suffix IS the full
		 * remaining key, the child is the external node directly.
		 * Otherwise, wrap remaining bytes in a compressed path.
		 */
		if (d->depth + new_slen == key_len) {
			branch = (struct cds_ft_inode_flag *) node;
		} else {
			branch = ft_build_branch(ft, key,
				d->depth + new_slen, key_len,
				(struct cds_ft_inode_flag *) node,
				1, false);
			if (!branch) {
				*ret_p = -ENOMEM;
				return FT_DESCENT_END;
			}
		}

		if (need_recompact) {
			struct cds_ft_inode_flag *new_col_flag;
			int rc;

			rc = ft_collapsed_recompact_with_insert(ft, col, d->nf,
					d->pnf, d->nfp, d->depth,
					iter_key, new_slen, branch,
					&new_col_flag);
			if (rc == 0) {
				FT_TP(collapsed_entry,
					(const void *) new_col_flag,
					(unsigned int) col_meta->nr_child,
					iter_key, new_slen,
					(const void *) branch, 0);
				d->nf = new_col_flag;
				*ret_p = 0;
				return FT_DESCENT_END;
			}
			/*
			 * Pre-check above guarantees rc != 1, so any
			 * non-zero is an allocation failure.  The child
			 * branch we just built is unpublished; release it.
			 */
			assert(rc < 0);
			if (branch != (struct cds_ft_inode_flag *) node)
				ft_free_branch(ft, key,
					d->depth + new_slen, key_len,
					branch);
			*ret_p = rc;
			return FT_DESCENT_END;
		}
	}

	/* Append at high_water slot. */
	{
		struct cds_ft_inode_flag **child_addr;

		/*
		 * Prefix-cache bytes for the SIMD reader fast path —
		 * relaxed stores; readers may observe torn states.  The
		 * verify on the atomically-published subkey (next call)
		 * is the source of truth and catches any false acceptance
		 * from the SIMD pre-filter.  Stride-invariant prefix layout.
		 *
		 * Entries always have slen >= FT_COLLAPSE_SUFFIX_MIN (= 2),
		 * enforced by the build path and the SUFFIX_MIN gate above.
		 */
		ft_collapsed_write_prefix_bytes(col, tier, high_water,
			iter_key[0], iter_key[1]);
		/*
		 * Publish the subkey via the stride-appropriate path:
		 * narrow uses the 64-bit one-shot SWAR (relaxed) on 64-bit
		 * hosts or release-on-len on 32-bit; wide always uses
		 * release-on-len (24-byte span doesn't fit in one atomic
		 * store).  Both transition the slot from slen=0 (cannot
		 * match) to slen=N (can match).
		 */
		if (stride == FT_COL_STRIDE_NARROW) {
			ft_collapsed_publish_subkey(col, tier, high_water,
					iter_key, new_slen);
			child_addr = &ft_collapsed_entries(col, tier)[high_water].child;
		} else {
			ft_collapsed_wide_publish_subkey(col, tier, high_water,
					iter_key, new_slen);
			child_addr = &ft_collapsed_entries_wide(col, tier)[high_water].child;
		}
		ft_set_parent(branch, d->nf, child_addr);
		/*
		 * Publish liveness: release-store on child pairs with
		 * reader's rcu_dereference.  Readers reaching this point
		 * observe the subkey already published and the entry now
		 * live.
		 */
		if (stride == FT_COL_STRIDE_NARROW)
			ft_collapsed_publish_entry(col, tier, high_water, branch);
		else
			ft_collapsed_wide_publish_entry(col, tier, high_water, branch);
		FT_TP(collapsed_entry,
			(const void *) ft_collapsed_node_flag_tier_stride(col, tier, stride),
			high_water,
			iter_key, new_slen,
			(const void *) branch, 0);
		FT_TP(collapsed_publish,
			(const void *) ft_collapsed_node_flag_tier_stride(col, tier, stride),
			high_water + 1,
			(unsigned int) ft_col_stride_entry_size[stride]);
	}

	{
		struct cds_ft_metadata *col_meta =
			cds_ft_item_to_metadata((struct cds_ft_inode *) col);
		col_meta->nr_child++;
	}

	ft_propagate_external_count_parent(ft, d->nf, 1);
	*ret_p = 0;
	return FT_DESCENT_END;

collapsed_explode:
	{
		/*
		 * Collapsed node full / out-of-order / prefix conflict:
		 * explode into internal + compressed nodes.
		 */
		struct cds_ft_inode_flag *internal_flag;
		int err;

		err = ft_collapsed_explode_node(ft, col, d->nf,
			d->pnf, d->nfp, d->depth, &internal_flag);
		if (err < 0) {
			*ret_p = err;
			return FT_DESCENT_END;
		}
		d->nf = internal_flag;
		return FT_DESCENT_CONTINUE;
	}
}
#endif /* FEATURE_FT_COLLAPSE */

/*
 * ft_build_ordinal_chain: create a chain of nodes from an array of
 * ordinal bytes (already mapped, not raw key bytes).  Uses a
 * compressed node if FEATURE_FT_COMPRESS is enabled, otherwise
 * builds a chain of single-child internal nodes.
 *
 * Returns the chain's root flag, or NULL on allocation failure.
 */
#ifdef FEATURE_FT_COLLAPSE
static
struct cds_ft_inode_flag *ft_build_ordinal_chain(struct cds_ft *ft,
		const uint8_t *ordinals, unsigned int len,
		struct cds_ft_inode_flag *child,
		unsigned long nr_keys,
		unsigned int base_depth)
{
#ifdef FEATURE_FT_COMPRESS
	/*
	 * In skip mode allow len == 1: the 1-byte compressed publishes
	 * as a skip-encoded pointer (0 CL on the read side), which is
	 * strictly cheaper than the 1-child internal it replaces.  In
	 * non-skip mode a 1-byte compressed is just an extra indirection
	 * over a 1-child internal — keep the historical len >= 2 floor.
	 */
	{
		unsigned int min_compress_len =
			ft_group_skip_compressed(ft->group) ? 1 : 2;

		if (len >= min_compress_len) {
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
			ft_nr_keys_store(ft, cn_meta, nr_keys, CMM_RELAXED);
			cflag = ft_compressed_node_flag(cn);
			ft_set_parent(child, cflag, NULL);
			ft_init_node_density(ft, cflag);
			return ft_publish_compressed(ft, cn, cflag);
		}
	}
#endif
	{
		struct cds_ft_inode_flag *cur = child;
		int i;

		for (i = (int) len - 1; i >= 0; i--) {
			struct cds_ft_inode_flag *dest = NULL;
			int ret;

			ret = ft_node_set_nth(ft, &dest, ordinals[i],
				cur, NULL, NULL, base_depth + i);
			if (ret) {
				/*
				 * Cleanup on failure.  Chain is bottom-up
				 * and unpublished until the final return,
				 * so any partial chain we built is local.
				 */
				while (cur != child) {
					struct cds_ft_inode_flag *next;

					next = ft_node_get_nth(cur, NULL,
						ordinals[i + 1], FT_PF_NONE);
					free_cds_ft_node_unpublished(ft, ft_node_ptr(cur));
					cur = next;
					i++;
				}
				return NULL;
			}
			{
				struct cds_ft_metadata *m =
					cds_ft_item_to_metadata(
						ft_node_ptr(dest));
				ft_nr_keys_store(ft, m, nr_keys,
					CMM_RELAXED);
				m->nr_child = 1;
			}
			/*
			 * Initialize density for each newly-created node in
			 * the chain.  Build is bottom-up (cur is the child
			 * already initialized in the previous iteration), so
			 * each dest's density propagates from cur's already
			 * up-to-date counters.
			 */
			ft_init_node_density(ft, dest);
			cur = dest;
		}
		return cur;
	}
}
#endif /* FEATURE_FT_COLLAPSE */

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
		unsigned int tier,
		unsigned int stride,
		unsigned int start, unsigned int end,
		unsigned int suffix_offset,
		unsigned int collapse_depth)
{
	unsigned int count = end - start;

	if (count == 0)
		return NULL;

	if (count == 1) {
		unsigned int e = start;
		unsigned int slen;
		const uint8_t *sfx;
		struct cds_ft_inode_flag *child;
		unsigned long child_nr_keys;

		if (stride == FT_COL_STRIDE_NARROW) {
			struct cds_ft_collapsed_entry *entry =
				&ft_collapsed_entries(col, tier)[e];
			slen = ft_collapsed_suffix_len(entry);
			sfx = ft_collapsed_suffix(entry);
			child = entry->child;
		} else {
			struct cds_ft_collapsed_entry_wide *entry =
				&ft_collapsed_entries_wide(col, tier)[e];
			slen = entry->len;
			sfx = entry->suffix;
			child = entry->child;
		}

#ifdef FEATURE_FT_SKIP_COMPRESSED
		if (ft_node_skip_compressed(child)) {
			/*
			 * Clear skip_slot: the collapsed node (and its
			 * entry slots) will be freed after the explode.
			 */
			ft_skip_to_compressed_meta(child)->skip_slot_offset =
				0;
			child = ft_compressed_node_flag(
				ft_skip_to_compressed(child));
		}
#endif
		if (!child)
			return NULL;	/* Tombstoned entry — no subtree. */
		if (slen <= suffix_offset)
			return child;

		if (!ft_node_external(child)) {
			struct cds_ft_metadata *cm =
				cds_ft_item_to_metadata(ft_node_ptr(child));
			child_nr_keys = ft_nr_keys_get(cm);
		} else {
			child_nr_keys = 1;
		}
		return ft_build_ordinal_chain(ft,
			sfx + suffix_offset, slen - suffix_offset,
			child, child_nr_keys,
			collapse_depth + suffix_offset);
	}

	/* Multiple entries: group by byte at suffix_offset. */
	{
		struct cds_ft_inode_flag *internal_flag = NULL;
		unsigned int i = start;
		unsigned long total_keys = 0;

		while (i < end) {
			const uint8_t *sfx_i;
			unsigned int slen_i;
			uint8_t first;
			unsigned int group_end;
			struct cds_ft_inode_flag *sub;

			if (stride == FT_COL_STRIDE_NARROW) {
				struct cds_ft_collapsed_entry *entry_i =
					&ft_collapsed_entries(col, tier)[i];
				sfx_i = ft_collapsed_suffix(entry_i);
				slen_i = ft_collapsed_suffix_len(entry_i);
			} else {
				struct cds_ft_collapsed_entry_wide *entry_i =
					&ft_collapsed_entries_wide(col, tier)[i];
				sfx_i = entry_i->suffix;
				slen_i = entry_i->len;
			}

			if (slen_i <= suffix_offset) {
				/* Entry is a leaf at this depth. */
				/* Should not happen with sorted entries. */
				i++;
				continue;
			}

			first = sfx_i[suffix_offset];
			group_end = i + 1;

			while (group_end < end) {
				const uint8_t *sfx_g;
				unsigned int slen_g;

				if (stride == FT_COL_STRIDE_NARROW) {
					struct cds_ft_collapsed_entry *entry_g =
						&ft_collapsed_entries(col, tier)[group_end];
					sfx_g = ft_collapsed_suffix(entry_g);
					slen_g = ft_collapsed_suffix_len(entry_g);
				} else {
					struct cds_ft_collapsed_entry_wide *entry_g =
						&ft_collapsed_entries_wide(col, tier)[group_end];
					sfx_g = entry_g->suffix;
					slen_g = entry_g->len;
				}

				if (slen_g <= suffix_offset ||
				    sfx_g[suffix_offset] != first)
					break;
				group_end++;
			}

			sub = ft_explode_entries(ft, col, tier, stride,
					i, group_end,
					suffix_offset + 1,
					collapse_depth);
			if (!sub) {
				i = group_end;
				continue;	/* Skip tombstoned/empty group. */
			}

			{
				struct cds_ft_inode *old_recompacted = NULL;
				int ret;

				ret = ft_node_set_nth(ft, &internal_flag,
					first, sub, &old_recompacted,
					internal_flag ?
						cds_ft_item_to_metadata(
							ft_node_ptr(internal_flag))
						: NULL,
					collapse_depth + suffix_offset);
				if (ret)
					return NULL;
				if (old_recompacted)
					free_cds_ft_node(ft, old_recompacted);
			}

			/* Accumulate key counts. */
			if (sub) {
				if (ft_node_external(sub))
					total_keys += 1;
				else {
					struct cds_ft_metadata *sm =
						cds_ft_item_to_metadata(
							ft_node_ptr(sub));
					total_keys += ft_nr_keys_get(sm);
				}
			}

			i = group_end;
		}

		if (internal_flag) {
			struct cds_ft_metadata *im =
				cds_ft_item_to_metadata(
					ft_node_ptr(internal_flag));
			ft_nr_keys_store(ft, im, total_keys,
				CMM_RELAXED);
			ft_init_node_density(ft, internal_flag);
		}
		return internal_flag;
	}
}

/*
 * ft_collapsed_recompact_with_insert: rebuild a collapsed node in
 * place at the same tier, copying live entries in sort order and
 * inserting one new entry at its sorted position.  Used by the
 * insert path when the in-place sorted-append is unsuitable
 * (high_water at capacity due to dead slots, or new key sorts
 * before an existing live entry).
 *
 * @new_suffix:           suffix bytes of the new entry (must not match
 *                        any existing live entry's prefix — caller has
 *                        filtered partial prefix overlaps to explode).
 * @new_slen:             length of @new_suffix (in [SUFFIX_MIN..SUFFIX_MAX]).
 * @new_branch:           child pointer to install for the new entry
 *                        (already has parent set; we update it below
 *                        to point at the new collapsed flag).
 * @out_replacement_flag: receives the new collapsed flag on success.
 *
 * Returns:
 *   0  on success — *@out_replacement_flag receives the new collapsed
 *      flag, the old collapsed has been freed via call_rcu, density and
 *      nr_keys are propagated.
 *   1  if recompact is not viable at this tier (live count + 1 > cap).
 *      Caller state unchanged; caller should fall through to explode.
 *  -ENOMEM on allocation failure.  Caller state unchanged.
 *
 * Concurrency: write-side only (mutex-held).  All stores into the new
 * collapsed happen on memory that has not yet been published; the
 * rcu_assign_pointer in ft_publish_to_parent provides the single
 * release fence ordering all of them before readers can observe the
 * replacement.
 */
static
int ft_collapsed_recompact_with_insert(struct cds_ft *ft,
		struct cds_ft_collapsed_node *col,
		struct cds_ft_inode_flag *col_flag,
		struct cds_ft_inode_flag *parent_nf,
		struct cds_ft_inode_flag **slot,
		unsigned int depth,
		const uint8_t *new_suffix,
		unsigned int new_slen,
		struct cds_ft_inode_flag *new_branch,
		struct cds_ft_inode_flag **out_replacement_flag)
{
	unsigned int tier = ft_collapsed_tier(col_flag);
	unsigned int stride = ft_collapsed_stride(col_flag);
	unsigned int cap = ft_collapsed_capacity_stride(tier, stride);
	struct cds_ft_collapsed_node *new_col;
	struct cds_ft_metadata *new_meta;
	struct cds_ft_metadata *col_meta = cds_ft_item_to_metadata(
			(struct cds_ft_inode *) col);
	struct cds_ft_inode_flag *new_col_flag;
	unsigned long old_density[FT_NODE_DENSITY_DEPTH];
	unsigned int old_fp = ft_node_readside_footprint(ft, col_flag);
	unsigned int e, ne;
	unsigned int nr_live = 0;
	bool inserted = false;
	unsigned int di;

	if (stride == FT_COL_STRIDE_NARROW) {
		struct cds_ft_collapsed_entry *old_entries =
				ft_collapsed_entries(col, tier);

		for (e = 0; e < cap; e++) {
			if (old_entries[e].child != NULL)
				nr_live++;
		}
	} else {
		struct cds_ft_collapsed_entry_wide *old_entries =
				ft_collapsed_entries_wide(col, tier);

		for (e = 0; e < cap; e++) {
			if (old_entries[e].child != NULL)
				nr_live++;
		}
	}
	if (nr_live + 1 > cap)
		return 1;

	for (di = 0; di < FT_NODE_DENSITY_DEPTH; di++)
		old_density[di] = ft_density_get(col_meta, di);

	new_col = alloc_collapsed_node(ft, tier, &new_meta);
	if (!new_col)
		return -ENOMEM;
	new_col_flag = ft_collapsed_node_flag_tier_stride(new_col, tier, stride);

	/*
	 * Walk old entries in slot order (= lex order across live
	 * entries), inserting the new entry at its sorted position.
	 * Body is stride-specific because the entry struct differs;
	 * the new node is the same stride as the old, since the
	 * caller's recompact decision was a same-tier same-stride
	 * rebuild.
	 */
	ne = 0;
	if (stride == FT_COL_STRIDE_NARROW) {
		struct cds_ft_collapsed_entry *old_entries =
				ft_collapsed_entries(col, tier);

		for (e = 0; e < cap; e++) {
			struct cds_ft_collapsed_entry *old_entry = &old_entries[e];
			struct cds_ft_inode_flag *child = old_entry->child;
			unsigned int slen;
			uint8_t *suffix;

			if (child == NULL)
				continue;
			slen = ft_collapsed_suffix_len(old_entry);
			suffix = ft_collapsed_suffix(old_entry);

			if (!inserted) {
				unsigned int cmp_len = slen < new_slen ? slen : new_slen;
				int cmp = ft_key_cmp_ordinals(suffix, new_suffix,
						cmp_len, cmp_len, false, NULL);

				if (cmp == 0)
					cmp = (slen < new_slen) ? -1 :
							(slen > new_slen) ? 1 : 0;
				assert(cmp != 0);
				if (cmp > 0) {
					ft_collapsed_write_prefix_bytes(new_col, tier,
							ne, new_suffix[0],
							new_suffix[1]);
					ft_collapsed_publish_subkey(new_col, tier, ne,
							new_suffix, new_slen);
					ft_collapsed_publish_entry(new_col, tier, ne,
							new_branch);
					ne++;
					inserted = true;
				}
			}
			ft_collapsed_write_prefix_bytes(new_col, tier, ne,
					suffix[0], suffix[1]);
			ft_collapsed_publish_subkey(new_col, tier, ne, suffix, slen);
			ft_collapsed_publish_entry(new_col, tier, ne, child);
			ne++;
		}
		if (!inserted) {
			ft_collapsed_write_prefix_bytes(new_col, tier, ne,
					new_suffix[0], new_suffix[1]);
			ft_collapsed_publish_subkey(new_col, tier, ne,
					new_suffix, new_slen);
			ft_collapsed_publish_entry(new_col, tier, ne, new_branch);
			ne++;
		}
	} else {
		struct cds_ft_collapsed_entry_wide *old_entries =
				ft_collapsed_entries_wide(col, tier);

		for (e = 0; e < cap; e++) {
			struct cds_ft_collapsed_entry_wide *old_entry = &old_entries[e];
			struct cds_ft_inode_flag *child = old_entry->child;
			unsigned int slen;
			const uint8_t *suffix;

			if (child == NULL)
				continue;
			slen = old_entry->len;
			suffix = old_entry->suffix;

			if (!inserted) {
				unsigned int cmp_len = slen < new_slen ? slen : new_slen;
				int cmp = ft_key_cmp_ordinals(suffix, new_suffix,
						cmp_len, cmp_len, false, NULL);

				if (cmp == 0)
					cmp = (slen < new_slen) ? -1 :
							(slen > new_slen) ? 1 : 0;
				assert(cmp != 0);
				if (cmp > 0) {
					ft_collapsed_write_prefix_bytes(new_col, tier,
							ne, new_suffix[0],
							new_suffix[1]);
					ft_collapsed_wide_publish_subkey(new_col, tier, ne,
							new_suffix, new_slen);
					ft_collapsed_wide_publish_entry(new_col, tier, ne,
							new_branch);
					ne++;
					inserted = true;
				}
			}
			ft_collapsed_write_prefix_bytes(new_col, tier, ne,
					suffix[0], suffix[1]);
			ft_collapsed_wide_publish_subkey(new_col, tier, ne, suffix, slen);
			ft_collapsed_wide_publish_entry(new_col, tier, ne, child);
			ne++;
		}
		if (!inserted) {
			ft_collapsed_write_prefix_bytes(new_col, tier, ne,
					new_suffix[0], new_suffix[1]);
			ft_collapsed_wide_publish_subkey(new_col, tier, ne,
					new_suffix, new_slen);
			ft_collapsed_wide_publish_entry(new_col, tier, ne, new_branch);
			ne++;
		}
	}
	assert(ne == nr_live + 1);

	/*
	 * Re-parent every child onto the new collapsed flag so that
	 * back-pointers (used by skip-compressed and parent-walk paths)
	 * stay valid after the swap.  external_nodes head, if any, is
	 * re-parented by ft_metadata_set_external_nodes below.
	 */
	if (stride == FT_COL_STRIDE_NARROW) {
		struct cds_ft_collapsed_entry *new_entries =
				ft_collapsed_entries(new_col, tier);

		for (e = 0; e < ne; e++) {
			ft_set_parent(new_entries[e].child, new_col_flag,
					&new_entries[e].child);
		}
	} else {
		struct cds_ft_collapsed_entry_wide *new_entries =
				ft_collapsed_entries_wide(new_col, tier);

		for (e = 0; e < ne; e++) {
			ft_set_parent(new_entries[e].child, new_col_flag,
					&new_entries[e].child);
		}
	}

	new_meta->nr_child = ne;
	if (col_meta->external_nodes) {
		ft_metadata_set_external_nodes(new_col_flag, new_meta,
				col_meta->external_nodes);
	}
	/*
	 * Seed nr_keys to the old value; the +1 for the inserted key
	 * (and propagation up the tree) is added by
	 * ft_propagate_external_count_parent below.
	 */
	ft_nr_keys_store(ft, new_meta, ft_nr_keys_get(col_meta),
			CMM_RELAXED);
	ft_init_node_density(ft, new_col_flag);

	ft_set_parent(new_col_flag, parent_nf, slot);
	ft_publish_to_parent(ft, parent_nf, slot, new_col_flag);
	free_collapsed_node(ft, col);

	ft_propagate_density_replace(ft, new_col_flag, depth,
			old_density, old_fp,
			ft_flag_to_metadata(new_col_flag),
			ft_node_readside_footprint(ft, new_col_flag),
			NULL, 0);
	ft_propagate_external_count_parent(ft, new_col_flag, 1);

	*out_replacement_flag = new_col_flag;
	return 0;
}

/*
 * ft_collapsed_explode_node: explode a collapsed node into an
 * internal/compressed subtree at the same parent slot.  Used by the
 * insert / detach / dissolve paths where a collapsed becomes
 * unsuitable (full, prefix conflict, or canonicalization).
 *
 * On success @*out_internal_flag receives the published replacement
 * (an internal node with the entries laid out as children).  Density
 * is propagated; the old collapsed is freed via call_rcu.  Any
 * resulting chains of single-child internals are not folded here —
 * that is left to the chain-compress pass in
 * ft_check_collapse_on_path.
 *
 * Returns 0 on success, -ENOMEM on allocation failure.  The collapsed
 * is left untouched on failure — callers must propagate the error.
 */
static
int ft_collapsed_explode_node(struct cds_ft *ft,
		struct cds_ft_collapsed_node *col,
		struct cds_ft_inode_flag *col_flag,
		struct cds_ft_inode_flag *parent_nf,
		struct cds_ft_inode_flag **slot,
		unsigned int depth,
		struct cds_ft_inode_flag **out_internal_flag)
{
	struct cds_ft_inode_flag *internal_flag;
	unsigned int tier = ft_collapsed_tier(col_flag);
	unsigned int stride = ft_collapsed_stride(col_flag);
	struct cds_ft_metadata *col_meta = cds_ft_item_to_metadata(
		(struct cds_ft_inode *) col);
	unsigned int cap = ft_collapsed_capacity_stride(tier, stride);
	unsigned long old_col_density[FT_NODE_DENSITY_DEPTH];
	unsigned int old_col_fp = ft_node_readside_footprint(ft, col_flag);
	unsigned int di;

	for (di = 0; di < FT_NODE_DENSITY_DEPTH; di++)
		old_col_density[di] = ft_density_get(col_meta, di);

	internal_flag = ft_explode_entries(ft, col, tier, stride,
		0, cap,
		0, depth);
	if (!internal_flag)
		return -ENOMEM;
	{
		struct cds_ft_metadata *int_meta =
			ft_flag_to_metadata(internal_flag);

		if (col_meta->external_nodes) {
			ft_metadata_set_external_nodes(internal_flag,
				int_meta, col_meta->external_nodes);
			ft_nr_keys_store(ft, int_meta,
				ft_nr_keys_get(int_meta) + 1,
				CMM_RELAXED);
		}
	}
	ft_init_node_density(ft, internal_flag);

	ft_set_parent(internal_flag, parent_nf, slot);
	ft_publish_to_parent(ft, parent_nf, slot, internal_flag);
	free_collapsed_node(ft, col);

	ft_propagate_density_replace(ft, internal_flag, depth,
		old_col_density, old_col_fp,
		ft_flag_to_metadata(internal_flag),
		ft_node_readside_footprint(ft, internal_flag),
		NULL, 0);

	*out_internal_flag = internal_flag;
	return 0;
}

#ifdef FEATURE_FT_COMPRESS
/*
 * ft_compress_chain_at: opportunistically replace a chain of single-child
 * no-external-nodes internals (extending through any adjacent compressed
 * or skip-compressed nodes) starting at @top_flag with a single compressed
 * (or skip-encoded compressed) node holding the accumulated path bytes.
 *
 * Walks down from @top_flag, accumulating one byte per single-child
 * internal and cn->len bytes per compressed/skip-compressed.  Stops at
 * the first node that is multi-child, has external_nodes, or is a
 * collapsed/external/null leaf — that node becomes the new compressed's
 * cn->child.  In skip mode the minimum useful length is 1 (a 1-byte
 * compressed publishes as a skip-encoded pointer, 0 CL on the read
 * side); in non-skip mode the minimum is 2 (a 1-byte compressed there
 * is just an extra indirection).  If the accumulated length clears
 * that floor, ft_build_ordinal_chain builds the new compressed
 * (auto-encoded as skip-compressed for short paths in skip-mode
 * tries), the new flag is atomically swapped into @parent_nf's @slot,
 * and absorbed nodes are freed.
 *
 * Why "extend through compressed":  the trie invariant forbids two
 * adjacent compressed nodes.  If the chain bottoms out at an existing
 * compressed, the new compressed must absorb that compressed's bytes
 * to preserve the invariant.  The absorbed compressed is freed.
 *
 * Returns 0 if not eligible (top is not a chain head, or accumulated
 * length is below the per-mode floor), 1 if compression succeeded,
 * -ENOMEM on allocation failure (chain unchanged).
 *
 * @top_flag must be an internal node when entering; the chain-head
 * eligibility test (nr_child == 1 and !external_nodes) is performed
 * inside.  @parent_nf may be NULL when @slot == &ft->root.
 *
 * Write-side only (mutex-held).
 */
static
int ft_compress_chain_at(struct cds_ft *ft,
		struct cds_ft_inode_flag *top_flag,
		struct cds_ft_inode_flag *parent_nf,
		struct cds_ft_inode_flag **slot,
		unsigned int top_depth)
{
	struct cds_ft_metadata *top_meta;
	uint8_t bytes_buf[FT_MAX_KEY_LEN];
	struct cds_ft_inode_flag *absorbed[FT_MAX_KEY_LEN];
	unsigned int len = 0;
	unsigned int nr_absorbed = 0;
	struct cds_ft_inode_flag *walk;
	struct cds_ft_inode_flag *compressed_flag;
	unsigned long old_density[FT_NODE_DENSITY_DEPTH];
	unsigned int old_fp;
	unsigned int di;
	unsigned long leaf_nr_keys;
	unsigned int i;
	unsigned int initial_top_len = 0;

	/*
	 * Top must be either an internal chain head (single-child, no
	 * external nodes) or a compressed node.  Starting from a
	 * compressed top is the canonicalization handle for the
	 * "compressed → internal chain head → ..." pattern: the walk loop
	 * below absorbs the top compressed's bytes and then continues
	 * downward into any chain head, fusing them all into one new
	 * compressed without ever creating two adjacent compresseds.
	 */
	if (!ft_node_internal(top_flag) && !ft_node_compressed(top_flag))
		return 0;
	top_meta = cds_ft_item_to_metadata(ft_node_ptr(top_flag));
	if (top_meta->nr_child != 1 || top_meta->external_nodes)
		return 0;
	/*
	 * Defensive: if top is internal and parent is compressed, the
	 * walk would build a new compressed adjacent to the compressed
	 * parent and ft_publish_to_parent would double-wrap the
	 * grandparent's skip pointer.  Pass-2's compressed-branch call
	 * canonicalizes from the compressed level instead and normally
	 * absorbs this chain head before we ever land here, but the
	 * absorb can refuse in edge cases (len would exceed 255, or the
	 * parent compressed has external_nodes).  Bail out cleanly in
	 * those residual cases.
	 */
	if (ft_node_internal(top_flag) && parent_nf
			&& ft_node_compressed(parent_nf))
		return 0;
	if (ft_node_compressed(top_flag))
		initial_top_len = ft_compressed_node_ptr(top_flag)->len;

	old_fp = ft_node_readside_footprint(ft, top_flag);
	for (di = 0; di < FT_NODE_DENSITY_DEPTH; di++)
		old_density[di] = ft_density_get(top_meta, di);

	walk = top_flag;
	for (;;) {
#ifdef FEATURE_FT_SKIP_COMPRESSED
		if (ft_node_skip_compressed(walk)) {
			struct cds_ft_compressed_node *cn =
				ft_skip_to_compressed(walk);
			unsigned int j;

			if (len + cn->len > 255)
				break;
			for (j = 0; j < cn->len; j++)
				bytes_buf[len++] = cn->key_bytes[j];
			absorbed[nr_absorbed++] =
				ft_compressed_node_flag(cn);
			walk = ft_dereference_acquire(cn->child);
			continue;
		}
#endif
		if (ft_node_external(walk))
			break;
		if (ft_node_compressed(walk)) {
			struct cds_ft_compressed_node *cn =
				ft_compressed_node_ptr(walk);
			struct cds_ft_metadata *cn_meta =
				cds_ft_item_to_metadata(
					(struct cds_ft_inode *) cn);
			unsigned int j;

			if (cn_meta->external_nodes)
				break;
			if (len + cn->len > 255)
				break;
			for (j = 0; j < cn->len; j++)
				bytes_buf[len++] = cn->key_bytes[j];
			absorbed[nr_absorbed++] = walk;
			walk = ft_dereference_acquire(cn->child);
			continue;
		}
		if (ft_node_collapsed(walk))
			break;
		if (ft_node_internal(walk)) {
			struct cds_ft_metadata *m =
				cds_ft_item_to_metadata(ft_node_ptr(walk));
			uint8_t k = 0;
			struct cds_ft_inode_flag *c;

			if (m->nr_child != 1 || m->external_nodes)
				break;
			if (len + 1 > 255)
				break;
			c = ft_node_get_minmax(walk, &k, FT_LEFTMOST);
			if (!c)
				return 0;
			bytes_buf[len++] = k;
			absorbed[nr_absorbed++] = walk;
			walk = c;
			continue;
		}
		break;
	}

	{
		unsigned int min_chain_len =
			ft_group_skip_compressed(ft->group) ? 1 : 2;

		if (len < min_chain_len || nr_absorbed == 0)
			return 0;
	}

	/*
	 * Compressed-top no-progress guard: if the only thing absorbed
	 * was the top compressed itself (no chain head, no further
	 * compresseds below), rebuilding would yield the same byte
	 * sequence in a same-size compressed.  Skip the useless work.
	 */
	if (initial_top_len > 0 && len <= initial_top_len)
		return 0;

	/*
	 * Per-path CL latency gate for non-skip publication.
	 *
	 * Skip-encoded publication (skip mode + len <= FT_SKIP_LEN_MAX)
	 * is always a strict win: 0 CL on the read side, replacing N
	 * absorbed nodes that each cost >= 0 CL.  Bypass the gate.
	 *
	 * Non-skip publication (non-skip mode, or len > FT_SKIP_LEN_MAX
	 * in skip mode) creates a compressed node that costs
	 *
	 *   candidate_cl = ceil((header + len) / 64)
	 *
	 * cache lines on a successful read (full key-byte scan with
	 * cn->child colocated in the header CL).  Compare against the
	 * accumulated CL load of the absorbed chain nodes; reject if
	 * the chain replacement would strictly increase per-path
	 * latency.
	 *
	 * compress_scan_mul_pct (per-trie tunable, default
	 * CDS_FT_COMPRESS_SCAN_MUL_PCT_DEFAULT = 150) scales the
	 * candidate's scan portion to model scan-loop overhead beyond
	 * raw CL bandwidth.
	 *
	 * Both muls are also passed to ft_node_readside_cl_pct() when
	 * pricing the absorbed-side nodes so that per-node-type costs
	 * stay symmetric between gates: a compressed (or collapsed)
	 * node priced as the candidate in one gate carries the same
	 * pct-CL cost when it appears as absorbed in another gate.
	 */
	{
		bool skip_publish =
			ft_group_skip_compressed(ft->group) &&
			len <= FT_SKIP_LEN_MAX;

		if (!skip_publish) {
			unsigned int collapse_scan_mul_pct = uatomic_load(
				&ft->collapse_scan_mul_pct, CMM_RELAXED);
			unsigned int compress_scan_mul_pct = uatomic_load(
				&ft->compress_scan_mul_pct, CMM_RELAXED);
			unsigned int absorbed_cl_pct = 0;
			unsigned int candidate_cl;
			unsigned int candidate_cl_pct;
			unsigned int j;

			for (j = 0; j < nr_absorbed; j++)
				absorbed_cl_pct +=
					ft_node_readside_cl_pct(ft, absorbed[j],
						collapse_scan_mul_pct,
						compress_scan_mul_pct);
			candidate_cl = (offsetof(struct cds_ft_compressed_node,
					key_bytes) + len + 63U) / 64U;
			candidate_cl_pct = compress_scan_mul_pct * candidate_cl;
			if ((unsigned long) absorbed_cl_pct <
			    (unsigned long) candidate_cl_pct)
				return 0;
		}
	}

	if (ft_node_external(walk))
		leaf_nr_keys = 1;
	else
		leaf_nr_keys = ft_nr_keys_get(ft_flag_to_metadata(walk));

	compressed_flag = ft_build_ordinal_chain(ft, bytes_buf, len, walk,
		leaf_nr_keys, top_depth);
	if (!compressed_flag)
		return -ENOMEM;

	ft_set_parent(compressed_flag, parent_nf, slot);
	ft_publish_to_parent(ft, parent_nf, slot, compressed_flag);

	for (i = 0; i < nr_absorbed; i++) {
		struct cds_ft_inode_flag *abs = absorbed[i];

		if (ft_node_compressed(abs))
			free_compressed_node(ft, ft_compressed_node_ptr(abs));
		else
			free_cds_ft_node(ft, ft_node_ptr(abs));
	}

	ft_propagate_density_replace(ft, compressed_flag, top_depth,
		old_density, old_fp,
		ft_flag_to_metadata(compressed_flag),
		ft_node_readside_footprint(ft, compressed_flag),
		NULL, 0);

	return 1;
}
#endif

#else
static __attribute__((unused))
struct cds_ft_inode_flag *ft_explode_entries(
		struct cds_ft *ft __attribute__((unused)),
		struct cds_ft_collapsed_node *col __attribute__((unused)),
		unsigned int tier __attribute__((unused)),
		unsigned int stride __attribute__((unused)),
		unsigned int start __attribute__((unused)),
		unsigned int end __attribute__((unused)),
		unsigned int suffix_offset __attribute__((unused)),
		unsigned int collapse_depth __attribute__((unused)))
{
	return NULL;
}
static __attribute__((unused))
int ft_collapsed_explode_node(struct cds_ft *ft __attribute__((unused)),
		struct cds_ft_collapsed_node *col __attribute__((unused)),
		struct cds_ft_inode_flag *col_flag __attribute__((unused)),
		struct cds_ft_inode_flag *parent_nf __attribute__((unused)),
		struct cds_ft_inode_flag **slot __attribute__((unused)),
		unsigned int depth __attribute__((unused)),
		struct cds_ft_inode_flag **out_internal_flag __attribute__((unused)))
{
	return -1;
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
	if (ft_density_pool_ensure(ft, FT_MAX_DEPTH))
		return -ENOMEM;

	key_depth = key_len + 1;

	dbg_printf("cds_ft_insert attempt: node %p\n", node);
	ft_descent_init(&d, ft);

	for (; d.depth < key_depth - 1; ) {
		uint8_t key_value;

		if (!d.nf)
			break;
		/*
		 * Resolve skip-compressed pointer.  Can appear after
		 * descending through a collapsed entry whose child was
		 * later split/recompacted, or after a collapse publish
		 * updated the parent's skip pointer.  Convert to the
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
#ifdef FEATURE_FT_COLLAPSE
		if (ft_node_collapsed(d.nf)) {
			enum ft_descent_action act;

			act = ft_insert_collapsed(ft, &d, &iter_key,
				key, key_len, key_depth, node,
				snapshot, snapshot_depth,
				&nr_snapshot, &ret);
			if (act == FT_DESCENT_END)
				goto insert_done;
			assert(act == FT_DESCENT_CONTINUE);
			continue;
		}
#endif /* FEATURE_FT_COLLAPSE */
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

/* Defined later (alongside ft_descend_to_graft_point's helpers). */
static
enum ft_descent_action ft_descent_step_collapsed(
		struct ft_descent *d,
		const uint8_t **ik_p,
		size_t key_len,
		struct cds_ft_inode_flag **snapshot,
		unsigned int *snapshot_depth,
		int *nr_snapshot);

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
	if (ft_density_pool_ensure(ft, FT_MAX_DEPTH))
		return -ENOMEM;

	key_depth = key_len + 1;

	dbg_printf("_cds_ft_insert_replace attempt: node %p\n", node);
	iter_key = key;
	ft_descent_init(&d, ft);

	for (; d.depth < key_depth - 1; ) {
		uint8_t key_value;

		if (!d.nf)
			break;
		/* Resolve skip-compressed (e.g. from collapsed entry). */
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
		if (ft_node_collapsed(d.nf)) {
			enum ft_descent_action act;

			act = ft_descent_step_collapsed(&d, &iter_key,
				key_len, snapshot, snapshot_depth,
				&nr_snapshot);
			if (act == FT_DESCENT_BREAK)
				break;
			assert(act == FT_DESCENT_CONTINUE);
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

	/* Root is always present and always internal. */

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
		if (ft_node_collapsed(node_flag)) {
			bool nf = false;
			enum ft_descent_action act;

			act = ft_traverse_collapsed(&node_flag,
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

/*
 * Replace a collapsed parent in ft_detach_node's structural-change
 * phase.  See the long comment in the caller for the three
 * sub-cases (A: tombstone the entry, B: repurpose with promoted
 * externals, C: replace col with col.external_nodes in
 * grandparent's slot).
 *
 * Always returns success (the caller's existing post-publish skip
 * for collapsed iter_node_flag also covers sub-case C, where col
 * is freed: ft_node_collapsed inspects only the pointer's tag bits
 * and does not access the freed memory).
 */
static
void ft_detach_node_replace_collapsed_parent(struct cds_ft *ft,
		struct cds_ft_inode_flag *iter_node_flag,
		struct cds_ft_inode_flag **detach_parent_flag_ptr,
		struct cds_ft_inode_flag **detach_node_flag_ptr,
		struct cds_ft_node *topmost_external_nodes,
		unsigned int cur_depth)
{
	unsigned int tier = ft_collapsed_tier(iter_node_flag);
	unsigned int stride = ft_collapsed_stride(iter_node_flag);
	struct cds_ft_collapsed_node *col = ft_collapsed_node_ptr(iter_node_flag);
	unsigned int cap = ft_collapsed_capacity_stride(tier, stride);
	struct cds_ft_metadata *col_meta = cds_ft_item_to_metadata(
		(struct cds_ft_inode *) col);
	unsigned int e;

	/*
	 * Sub-case C: last-entry detach on a collapsed with its own
	 * external_nodes and no promoted chain from the detached
	 * child.  Replace col directly with col.external_nodes in
	 * grandparent's slot.
	 */
	if (col_meta->nr_child == 1 && !topmost_external_nodes
			&& col_meta->external_nodes) {
		struct cds_ft_node *ext = col_meta->external_nodes;

		/*
		 * Propagate density for col (about to be freed).
		 * This mirrors the propagation path in the former
		 * nr_child == 0 branch.
		 */
		ft_propagate_node_density_parent(ft,
			iter_node_flag, cur_depth, cur_depth,
			-(long) ft_node_readside_footprint(ft,
				iter_node_flag));

		/*
		 * Reparent ext to col's parent before publishing.
		 * Otherwise ext->prev would keep pointing at col, which
		 * is about to be freed.
		 */
		ft_set_parent((struct cds_ft_inode_flag *) ext,
			col_meta->parent, detach_parent_flag_ptr);
		ft_publish_to_parent(ft, col_meta->parent,
			detach_parent_flag_ptr,
			(struct cds_ft_inode_flag *) ext);
		free_collapsed_node(ft, col);
		return;
	}

	/*
	 * Sub-cases A and B: kill (A) or repurpose (B) the detached
	 * entry.  nr_child stays >= 1 in both.
	 */
	for (e = 0; e < cap; e++) {
		struct cds_ft_inode_flag **child_addr;

		if (stride == FT_COL_STRIDE_NARROW)
			child_addr = &ft_collapsed_entries(col, tier)[e].child;
		else
			child_addr = &ft_collapsed_entries_wide(col, tier)[e].child;
		if (child_addr != detach_node_flag_ptr)
			continue;
		if (topmost_external_nodes) {
			/*
			 * Sub-case B: repurpose entry with the promoted
			 * external chain head.  nr_child unchanged; col
			 * stays live.
			 *
			 * Reparent the external chain head to col before
			 * publishing: otherwise topmost_external_nodes->prev
			 * would keep pointing to the (about-to-be-freed)
			 * detached subtree root.
			 */
			ft_set_parent(
				(struct cds_ft_inode_flag *) topmost_external_nodes,
				iter_node_flag, child_addr);
			ft_publish_to_parent(ft, iter_node_flag,
				child_addr,
				(struct cds_ft_inode_flag *) topmost_external_nodes);
		} else {
			/*
			 * Sub-case A: mark slot dead by release-storing NULL
			 * into entry->child.  Slot keeps its (suffix,len) and
			 * is never reused.  Only reached when col.nr_child > 1.
			 */
			assert(col_meta->nr_child > 1);
			if (stride == FT_COL_STRIDE_NARROW)
				ft_collapsed_kill_entry(col, tier, e);
			else
				ft_collapsed_wide_kill_entry(col, tier, e);
			FT_TP(collapsed_entry,
				(const void *) ft_collapsed_node_flag_tier(col, tier), e,
				(const uint8_t *) NULL, 0U,
				(const void *) NULL, 1);
			col_meta->nr_child--;
		}
		break;
	}
}

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
	/* Saved density snapshot of the detached child for per-level propagation. */
	struct cds_ft_metadata *old_detach_cm = NULL;
	unsigned int old_detach_fp = 0;
	unsigned long old_detach_density[FT_NODE_DENSITY_DEPTH] = { 0 };
	uint8_t n = 0;
	struct cds_ft_node *topmost_external_nodes = NULL;
	bool prev_external_nodes_found = false;
	struct cds_ft_inode_flag *cur;
	unsigned int cur_depth;
	/*
	 * Save the original detach child before the upward walk may
	 * shift detach_node_flag_ptr to a higher level.  Used for the
	 * free-intermediate walk below.
	 */
	struct cds_ft_inode_flag *orig_detach_child = *detach_node_flag_ptr;

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

	/*
	 * Capture and propagate the detached child's density BEFORE
	 * any structural changes (replace, free).  At this point all
	 * parent pointers in the ancestor chain are still valid.
	 * After the replace / free-intermediate steps below, freed
	 * nodes may poison metadata and make the parent walk unsafe.
	 */
	{
		struct cds_ft_inode_flag *detach_child_nf =
			*detach_node_flag_ptr;

		/*
		 * Resolve skip-compressed before the external check:
		 * a skip pointer whose encoded child is external has
		 * low tag bits == 0, which falsely matches
		 * ft_node_external, causing the entire density
		 * subtraction to be skipped.  This loses the density
		 * profile of the subtree the compressed node replaced
		 * (not the compressed node's own zero contribution);
		 * without the subtraction, ancestors retain stale
		 * density from nodes that no longer exist below them.
		 */
		detach_child_nf = ft_resolve_skip_compressed(detach_child_nf);

		if (detach_child_nf &&
		    !ft_node_external(detach_child_nf)) {
			unsigned int j;

			old_detach_cm = ft_flag_to_metadata(
				detach_child_nf);
			old_detach_fp = ft_node_readside_footprint(ft,
				detach_child_nf);
			for (j = 0; j < FT_NODE_DENSITY_DEPTH; j++)
				old_detach_density[j] =
					ft_density_get(old_detach_cm, j);
			ft_propagate_density_replace(ft,
				NULL, cur_depth + 1,
				old_detach_density, old_detach_fp,
				NULL, 0,
				iter_node_flag, cur_depth);
		}
		/*
		 * Mark as already propagated so the end-of-function
		 * path does not propagate again.
		 */
		old_detach_cm = NULL;
	}

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
	} else if (ft_node_collapsed(iter_node_flag)) {
		/*
		 * Collapsed parent.  The upward walk stopped here for
		 * one of: col.nr_child > 1, col.external_nodes is set,
		 * or a descendant's external_nodes was propagated up
		 * (topmost_external_nodes non-NULL).
		 *
		 * A collapsed with nr_child == 0 would be a dead-end
		 * under ordered traversal -- an invariant violation.
		 * The three sub-cases handled in
		 * ft_detach_node_replace_collapsed_parent ensure
		 * nr_child stays >= 1 *quiescently* at all times a
		 * reader can observe col in the trie:
		 *
		 *   A. col.nr_child > 1: tombstone the one entry being
		 *      detached.  nr_child-- leaves nr_child >= 1; col
		 *      remains live with fewer entries.
		 *
		 *   B. col.nr_child == 1 AND topmost_external_nodes
		 *      (detached child had external_nodes to promote):
		 *      repurpose the entry -- cptrs[e] becomes the
		 *      promoted external chain head.  nr_child stays at
		 *      1; col's own external_nodes (if any) is
		 *      unchanged.  No tombstone, no nr_child decrement.
		 *
		 *   C. col.nr_child == 1 AND col.external_nodes (no
		 *      topmost_external_nodes): replace col in its
		 *      grandparent's slot with col.external_nodes.
		 *      col is freed; grandparent now points directly
		 *      at col's NIL-key chain.  col is never observed
		 *      quiescently with nr_child == 0.
		 *
		 * The remaining case (nr_child == 1 AND no external
		 * anywhere) is handled earlier by the upward walk:
		 * col is marked for prune (nr_clear++), the whole
		 * single-child chain up to col's non-prunable ancestor
		 * is replaced via ft_node_replace_ptr, and col is
		 * freed via the free-intermediate walk without ever
		 * being iter_node_flag here.
		 *
		 * Reader-side transient exception: a reader's entry
		 * scan is not atomic, and sub-case A writes cptrs[e] =
		 * NULL *before* setting the tombstone bit.  With the
		 * right interleaving, a reader can observe every entry
		 * as dead-bit-set or NULL-pointer simultaneously --
		 * even though at no wall-clock instant were all entries
		 * quiescently dead.  cds_ft_lookup_inequality treats
		 * this as "empty at this step" and backs out via
		 * going_up to find the next sibling at a higher level;
		 * it does NOT abort.  Abort here is reserved for the
		 * compressed-child-NULL case, where the invariant
		 * holds even transiently (cn->child is set before the
		 * compressed is published and never cleared in place).
		 *
		 * Density was already propagated above (before
		 * structural changes).
		 */
		ft_detach_node_replace_collapsed_parent(ft, iter_node_flag,
			detach_parent_flag_ptr, detach_node_flag_ptr,
			topmost_external_nodes, cur_depth);
		ret = 0;
	} else {
		/*
		 * Density was already propagated above (before
		 * structural changes).
		 *
		 * Use orig_detach_child (saved before the upward walk)
		 * for the free-intermediate walk.  When the walk elevated
		 * detach_node_flag_ptr, *detach_node_flag_ptr equals
		 * iter_node_flag (the parent we are about to modify).
		 * Freeing iter_node_flag would corrupt the trie.
		 * orig_detach_child always points to the actual child
		 * subtree that needs freeing.
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
			 * single-child chain from old_detach_child
			 * down is unreachable.  Collect nodes first,
			 * then free after the walk completes (avoids
			 * use-after-free during traversal).
			 * topmost_external_nodes (if any) was already
			 * saved and published at the replacement point.
			 */
			{
				struct cds_ft_inode_flag *to_free[FT_MAX_DEPTH];
				int nr_to_free = 0, fi;
				struct cds_ft_inode_flag *walk_nf = orig_detach_child;

				while (walk_nf &&
				       !ft_node_external(walk_nf) &&
				       nr_to_free < FT_MAX_DEPTH) {
					struct cds_ft_inode_flag *next = NULL;

					if (ft_node_compressed(walk_nf) ||
					    ft_node_skip_compressed(walk_nf)) {
						struct cds_ft_compressed_node *cn;
						struct cds_ft_metadata *cm;

						if (ft_node_skip_compressed(walk_nf))
							cn = ft_skip_to_compressed(walk_nf);
						else
							cn = ft_compressed_node_ptr(walk_nf);
						cm = cds_ft_item_to_metadata(
							(struct cds_ft_inode *) cn);
						/*
						 * When nr_clear == 0, stop at
						 * nodes with content (they're
						 * still reachable).  When
						 * nr_clear > 0, the upward
						 * pruning made the entire chain
						 * unreachable — free everything.
						 */
						if (!nr_clear &&
						    (cm->nr_child > 0 ||
						     cm->external_nodes))
							break;
						next = cn->child;
					} else if (ft_node_collapsed(walk_nf)) {
						break;
					} else {
						struct cds_ft_metadata *m =
							cds_ft_item_to_metadata(
								ft_node_ptr(walk_nf));

						if (!nr_clear &&
						    (m->nr_child > 0 ||
						     m->external_nodes))
							break;
						if (m->nr_child == 1) {
							unsigned int key;

							for (key = 0; key < 256; key++) {
								next = ft_node_get_nth(
									walk_nf, NULL,
									(uint8_t) key, FT_PF_NONE);
								if (next)
									break;
							}
						} else if (m->nr_child > 1) {
							break;
						}
					}
					to_free[nr_to_free++] = walk_nf;
					walk_nf = next;
				}
				for (fi = 0; fi < nr_to_free; fi++) {
					if (ft_node_compressed(to_free[fi]) ||
					    ft_node_skip_compressed(to_free[fi]))
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
	 * Skip for compressed/collapsed parents: the replacement was
	 * already published inline above.
	 */
	if (!ft_node_compressed(iter_node_flag) &&
	    !ft_node_skip_compressed(iter_node_flag) &&
	    !ft_node_collapsed(iter_node_flag)) {
		struct cds_ft_metadata *iter_meta =
			cds_ft_item_to_metadata(ft_node_ptr(iter_node_flag));

		dbg_printf("ft_detach_node: publish %p instead of %p\n",
			iter_node_flag, *detach_parent_flag_ptr);
		ft_publish_to_parent(ft, iter_meta->parent,
			detach_parent_flag_ptr, iter_node_flag);
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

/*
 * Handle a collapsed node in cds_ft_remove / cds_ft_remove_all's
 * descent loop.  Scan entries for a suffix match against
 * key[dd->d.depth..key_len-1]; only entries with slen <= remaining
 * are considered (no partial-prefix explode here, since remove
 * targets an exact key).
 *
 * On match: track the detach point, advance dd and *iter_key_p
 * past the matched span, refresh the pending detach pointer if
 * needed, and return CDS_FT_STATUS_OK so the caller continues
 * the descent.  On no-match (or match with NULL pointer slot):
 * return CDS_FT_STATUS_NOT_FOUND.
 */
static
enum cds_ft_status ft_remove_descent_collapsed(
		struct ft_detach_descent *dd,
		const uint8_t **iter_key_p,
		size_t key_len)
{
	unsigned int tier = ft_collapsed_tier(dd->d.nf);
	unsigned int stride = ft_collapsed_stride(dd->d.nf);
	struct cds_ft_collapsed_node *col = ft_collapsed_node_ptr(dd->d.nf);
	const struct cds_ft_metadata *col_meta = cds_ft_item_to_metadata(
		(struct cds_ft_inode *) col);
	unsigned int remaining = key_len - dd->d.depth;
	const uint8_t *iter_key = *iter_key_p;
	struct ft_col_view view;
	unsigned int e;

	ft_for_each_live_collapsed_entry_view_rcu(col, tier, stride, e, view) {
		struct cds_ft_inode_flag *child = view.child;
		struct cds_ft_inode_flag **child_addr;
		unsigned int slen = view.slen;
		bool match;

		if (slen > remaining)
			continue;
		match = (ft_key_cmp_ordinals(iter_key, view.suffix, slen, slen,
					     false, NULL) == 0);
		if (!match)
			continue;
		if (!child)
			return CDS_FT_STATUS_NOT_FOUND;
		if (stride == FT_COL_STRIDE_NARROW)
			child_addr = &ft_collapsed_entries(col, tier)[e].child;
		else
			child_addr = &ft_collapsed_entries_wide(col, tier)[e].child;
		ft_detach_descent_track(dd, col_meta);
		dd->d.ppnf  = dd->d.pnf;
		dd->d.ppnfp = dd->d.pnfp;
		dd->d.pnf   = dd->d.nf;
		dd->d.pnfp  = dd->d.nfp;
		dd->d.nf    = child;
		dd->d.nfp   = child_addr;
		dd->d.depth += slen;
		iter_key += slen;
		if (dd->d.nf && dd->pending) {
			dd->det_nfp = dd->d.nfp;
			dd->pending = false;
		}
		*iter_key_p = iter_key;
		return CDS_FT_STATUS_OK;
	}
	return CDS_FT_STATUS_NOT_FOUND;
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
	if (ft_density_pool_ensure(ft, FT_MAX_DEPTH)) {
		FT_TP(remove_exit, (int) CDS_FT_STATUS_MEMORY_ERROR);
		return CDS_FT_STATUS_MEMORY_ERROR;
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
		/* Resolve skip-compressed (e.g. from collapsed entry). */
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
		if (ft_node_collapsed(dd.d.nf)) {
			enum cds_ft_status s;

			s = ft_remove_descent_collapsed(&dd, &iter_key,
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
					dd.det_depth);
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
#ifdef FEATURE_FT_COLLAPSE
		/*
		 * iter_key was advanced byte-by-byte during the descent
		 * loop above, so it now points key_len bytes past the
		 * iterator's key buffer.  Re-read iter_key(iter) to pass
		 * the ORIGINAL key path; otherwise the canonicalization
		 * walk reads zeros / garbage past the end of the key and
		 * bails before reaching the slot of the removed key.
		 */
		ft_check_collapse_on_path(ft, iter_key(iter), key_len);
#endif
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
	if (ft_density_pool_ensure(ft, FT_MAX_DEPTH)) {
		*result_node = NULL;
		return CDS_FT_STATUS_MEMORY_ERROR;
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
		ft_nr_keys_store(ft, metadata, ft_nr_keys_get(metadata) - 1,
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
		if (ft_node_collapsed(dd.d.nf)) {
			enum cds_ft_status s;

			s = ft_remove_descent_collapsed(&dd, &iter_key,
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
		ft_propagate_external_count_parent(ft, dd.d.nf, -1);
		rcu_assign_pointer(metadata->external_nodes, NULL);
		ret = 0;
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
				dd.det_depth);
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

#ifdef FEATURE_FT_COLLAPSE
	/*
	 * iter_key was advanced byte-by-byte during the descent loop
	 * above and now points key_len bytes past the iterator's key
	 * buffer.  Re-read iter_key(iter) to pass the ORIGINAL key
	 * path; otherwise the canonicalization walk reads zeros / garbage
	 * past the end of the key and bails before reaching the slot of
	 * the removed key, leaving a chain-head internal that the
	 * post-remove fold should have absorbed into the parent
	 * compressed.
	 */
	ft_check_collapse_on_path(ft, iter_key(iter), key_len);
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
		ft_nr_keys_store(ft, sfx_meta, old_child_nr_keys,
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
			ft_nr_keys_store(ft, m, old_child_nr_keys,
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
		ft_nr_keys_store(ft, branch_meta, old_child_nr_keys,
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
		ft_nr_keys_store(ft, pfx_meta, ft_nr_keys_get(cn_meta),
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
			ft_nr_keys_store(ft, pfx_meta, ft_nr_keys_get(cn_meta),
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
		ft_nr_keys_store(ft, int_meta, ft_nr_keys_get(cn_meta),
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
		ft_nr_keys_store(ft, pfx_meta, ft_nr_keys_get(cn_meta),
			CMM_RELAXED);
		if (cn_meta->external_nodes)
			ft_metadata_set_external_nodes(dest, pfx_meta, cn_meta->external_nodes);
		top_flag = dest;
		created[nr_created++] = dest;
	} else {
		/* diverge_pos == 0: branch IS the top. */
		struct cds_ft_metadata *branch_meta =
			cds_ft_item_to_metadata(ft_node_ptr(branch_flag));
		ft_nr_keys_store(ft, branch_meta, ft_nr_keys_get(cn_meta),
			CMM_RELAXED);
		if (cn_meta->external_nodes)
			ft_metadata_set_external_nodes(branch_flag, branch_meta, cn_meta->external_nodes);
		top_flag = branch_flag;
	}

	/* 4. Initialize density on created nodes and save old profile. */
	{
		unsigned long old_cn_density[FT_NODE_DENSITY_DEPTH];
		unsigned int old_cn_fp = ft_node_readside_footprint(ft, d->nf);
		unsigned int di;
		int ci;

		for (di = 0; di < FT_NODE_DENSITY_DEPTH; di++)
			old_cn_density[di] = ft_density_get(cn_meta, di);
		for (ci = 0; ci < nr_created; ci++)
			ft_init_node_density(ft, created[ci]);

		/* 5. Publish the split structure. */
		ft_set_parent(top_flag, d->pnf, d->nfp);
		ft_publish_to_parent(ft, d->pnf, d->nfp, top_flag);

		/* 6. Propagate per-level density replacement. */
		ft_propagate_density_replace(ft, top_flag, d->depth,
			old_cn_density, old_cn_fp,
			ft_flag_to_metadata(top_flag),
			ft_node_readside_footprint(ft, top_flag),
			NULL, 0);
	}

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

/*
 * Match a single collapsed-entry suffix against key[d->depth..key_len-1]
 * and step into the matched child.  Mirrors ft_descent_step (single-
 * byte step) and ft_descent_traverse_compressed (multi-byte compressed
 * step) for the collapsed-node case.
 *
 * Only entries with slen <= remaining are considered (no partial-
 * prefix split is performed here — callers that need an explode
 * handle that case before calling this helper).  On match: snapshot
 * the collapsed node, advance d and *ik_p past the matched span,
 * return FT_DESCENT_CONTINUE.  On no-match: return
 * FT_DESCENT_BREAK, leaving d at the collapsed node so the
 * caller can decide what that means in its phase (graft point,
 * insert-replace key terminus, ...).
 *
 * Used by ft_descend_to_graft_point and _cds_ft_insert_replace.
 */
static
enum ft_descent_action ft_descent_step_collapsed(
		struct ft_descent *d,
		const uint8_t **ik_p,
		size_t key_len,
		struct cds_ft_inode_flag **snapshot,
		unsigned int *snapshot_depth,
		int *nr_snapshot)
{
	unsigned int tier = ft_collapsed_tier(d->nf);
	unsigned int stride = ft_collapsed_stride(d->nf);
	struct cds_ft_collapsed_node *col = ft_collapsed_node_ptr(d->nf);
	const uint8_t *ik = *ik_p;
	unsigned int remaining = key_len - d->depth;
	struct ft_col_view view;
	unsigned int e;

	ft_for_each_live_collapsed_entry_view_rcu(col, tier, stride, e, view) {
		struct cds_ft_inode_flag *child = view.child;
		struct cds_ft_inode_flag **child_addr;
		unsigned int slen = view.slen;
		bool match;

		if (slen > remaining)
			continue;
		match = (ft_key_cmp_ordinals(ik, view.suffix, slen, slen,
					     false, NULL) == 0);
		if (!match)
			continue;
		if (stride == FT_COL_STRIDE_NARROW)
			child_addr = &ft_collapsed_entries(col, tier)[e].child;
		else
			child_addr = &ft_collapsed_entries_wide(col, tier)[e].child;
		ft_snapshot_push(snapshot, snapshot_depth,
			*nr_snapshot, d->nf, d->depth);
		d->ppnf  = d->pnf;
		d->ppnfp = d->pnfp;
		d->pnf   = d->nf;
		d->pnfp  = d->nfp;
		d->nf    = child;
		d->nfp   = child_addr;
		d->depth += slen;
		ik += slen;
		*ik_p = ik;
		return FT_DESCENT_CONTINUE;
	}
	/* Graft point: no matching entry. */
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
		/* Resolve skip-compressed (e.g. from collapsed entry). */
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
		if (ft_node_collapsed(d->nf)) {
			enum ft_descent_action act;

			act = ft_descent_step_collapsed(d, &ik,
				key_len, snapshot, snapshot_depth,
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
		ft_nr_keys_store(ft, sfx_meta, child_nr_keys,
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
			ft_nr_keys_store(ft, m, child_nr_keys,
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
		ft_nr_keys_store(ft, pfx_meta, ft_nr_keys_get(cn_meta),
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
			ft_nr_keys_store(ft, pfx_meta, ft_nr_keys_get(cn_meta),
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
		ft_nr_keys_store(ft, int_meta, ft_nr_keys_get(cn_meta),
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
		ft_nr_keys_store(ft, pfx_meta, ft_nr_keys_get(cn_meta),
			CMM_RELAXED);
		if (cn_meta->external_nodes)
			ft_metadata_set_external_nodes(dest, pfx_meta, cn_meta->external_nodes);
		prefix_flag = dest;
	}

	/* Publish and set descent state. */
	ft_set_parent(prefix_flag, d->pnf, d->nfp);
	ft_publish_to_parent(ft, d->pnf, d->nfp, prefix_flag);

	/* Density: init created nodes and propagate replacement profile. */
	{
		unsigned long old_cn_density[FT_NODE_DENSITY_DEPTH];
		unsigned int old_cn_fp = ft_node_readside_footprint(ft, d->nf);
		unsigned int di;

		for (di = 0; di < FT_NODE_DENSITY_DEPTH; di++)
			old_cn_density[di] = ft_density_get(cn_meta, di);
		ft_init_node_density(ft, suffix_flag);
		ft_init_node_density(ft, prefix_flag);

		ft_propagate_density_replace(ft, prefix_flag, d->depth,
			old_cn_density, old_cn_fp,
			ft_flag_to_metadata(prefix_flag),
			ft_node_readside_footprint(ft, prefix_flag),
			NULL, 0);
	}

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

	/* Try compression over [compress_start, end). */
	if (end >= compress_start + 2) {
		struct cds_ft_inode_flag *compressed;

		compressed = ft_try_compress_chain(ft, key, end,
			compress_start, leaf, NULL);
		if (compressed == (void *) (long) -ENOMEM)
			return NULL;
		if (compressed) {
			struct cds_ft_metadata *m =
				ft_flag_to_metadata(compressed);

			ft_nr_keys_store(ft, m,
				subtree_external_count, CMM_RELAXED);
			ft_init_node_density(ft, compressed);
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
		ft_nr_keys_store(ft,
			cds_ft_item_to_metadata(ft_node_ptr(dest)),
			subtree_external_count, CMM_RELAXED);
		/*
		 * Initialize density: this node's child (cur) may be
		 * the graft payload with an existing subtree.
		 * Bottom-up order ensures child density is set before
		 * parent.
		 */
		ft_init_node_density(ft, dest);
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
			ft_nr_keys_store(ft, bm, ft_nr_keys_get(bm) + 1,
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
	if (ft_density_pool_ensure(dst_ft, FT_MAX_DEPTH))
		return CDS_FT_STATUS_MEMORY_ERROR;

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
		 * Root-level graft: the source's root becomes the
		 * destination's root with no parent-pointer change
		 * (both are root positions with parent == NULL).  No
		 * "jump out" window, so no internal synchronize_rcu is
		 * required for this path.
		 *
		 * Swap root pointers.  The source's root carries all
		 * metadata (nr_child, external_nodes) with it.
		 */
		rcu_assign_pointer(dst_ft->root, src_ft->root);
		FT_TP(root_publish, (const void *) dst_ft,
			(const void *) dst_ft->root);
		rcu_assign_pointer(src_ft->root, ft_node_flag(fresh_root, 0));
		FT_TP(root_publish, (const void *) src_ft,
			(const void *) src_ft->root);
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

		/*
		 * Propagate per-level density addition for the node
		 * actually attached at the graft point (add-only: no
		 * old child to subtract).  attached_nf is either the
		 * graft payload (direct attach) or the top of the
		 * branch built by ft_build_branch (ancestors above the
		 * branch top are what need updating; the branch's own
		 * internals already have their density initialized by
		 * ft_build_branch, and walking from attached_nf skips
		 * the node itself).
		 */
		if (!ft_node_external(attached_nf))
			ft_propagate_density_replace(dst_ft,
				attached_nf, attached_depth,
				NULL, 0,
				cds_ft_item_to_metadata(
					ft_node_ptr(attached_nf)),
				ft_node_readside_footprint(dst_ft, attached_nf),
				NULL, 0);
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
	if (ft_density_pool_ensure(dst_ft, FT_MAX_DEPTH)) {
		FT_TP(graft_swap_exit, (int) CDS_FT_STATUS_MEMORY_ERROR);
		return CDS_FT_STATUS_MEMORY_ERROR;
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
		 * Propagate per-level density delta for the swapped
		 * subtrees.  Save old child's density to a snapshot
		 * (it's still live but will be moved to swap_ft below),
		 * then use ft_propagate_density_replace from the new
		 * child (old_swap_root, whose parent is now d.pnf).
		 */
		{
			unsigned long old_snap[FT_NODE_DENSITY_DEPTH] = { 0 };
			unsigned int old_fp_snap = 0;
			struct cds_ft_metadata *new_cm = NULL;
			unsigned int new_fp_snap = 0;

			if (!ft_node_external(old_child) && ft_node_ptr(old_child)) {
				struct cds_ft_metadata *old_cm =
					cds_ft_item_to_metadata(
						ft_node_ptr(old_child));
				unsigned int di;

				old_fp_snap = ft_node_readside_footprint(dst_ft, old_child);
				for (di = 0; di < FT_NODE_DENSITY_DEPTH; di++)
					old_snap[di] = ft_density_get(old_cm, di);
			}
			if (!swap_empty && !ft_node_external(old_swap_root)) {
				new_cm = cds_ft_item_to_metadata(
						ft_node_ptr(old_swap_root));
				new_fp_snap = ft_node_readside_footprint(dst_ft, old_swap_root);
			}
			if (old_fp_snap || new_cm) {
				/*
				 * Normal mode when new child exists
				 * (old_swap_root has parent set to d.pnf).
				 * Parent mode when swap is empty (child
				 * freed, start from surviving parent d.pnf).
				 */
				ft_propagate_density_replace(dst_ft,
					new_cm ? old_swap_root : NULL,
					key_len,
					old_fp_snap ? old_snap : NULL,
					old_fp_snap,
					new_cm, new_fp_snap,
					new_cm ? NULL : d.pnf,
					new_cm ? 0 : key_len - 1);
			}
		}

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
				ft_nr_keys_store(dst_ft, swap_rmeta, old_count, CMM_RELEASE);
			}
		} else {
			/* swap_ft->root is already @fresh from the unlink step. */
			if (old_child) {
				ft_metadata_set_external_nodes(ft_node_flag(fresh, 0), fresh_meta,
					(struct cds_ft_node *)
					ft_node_ptr(old_child));
				ft_nr_keys_store(dst_ft, fresh_meta, old_count, CMM_RELEASE);
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
 * Handle a collapsed node in cds_ft_detach's descent loop.  Scans
 * entries for a suffix match against key[dd->d.depth..key_len-1]:
 *
 *   - Exact / shorter-or-equal suffix match: advance dd and *ik_p
 *     past the matched span, return CDS_FT_STATUS_OK so the caller
 *     continues the descent loop.
 *
 *   - Partial-prefix match (collapsed entry's suffix is longer than
 *     the remaining key but matches as a prefix): explode the
 *     collapsed node into internals so the detach point lands on a
 *     real internal-node boundary, install the result in dd->d.nf,
 *     return CDS_FT_STATUS_OK so the caller's next iteration
 *     re-dispatches via the regular internal-node path.
 *
 *   - No match found, or matched entry has a NULL pointer slot:
 *     return CDS_FT_STATUS_NOT_FOUND.
 *
 *   - Allocation failure during explode: return
 *     CDS_FT_STATUS_MEMORY_ERROR.
 */
static
enum cds_ft_status ft_detach_descent_collapsed(struct cds_ft *ft,
		struct ft_detach_descent *dd,
		const uint8_t **ik_p,
		size_t key_len)
{
	unsigned int tier = ft_collapsed_tier(dd->d.nf);
	unsigned int stride = ft_collapsed_stride(dd->d.nf);
	struct cds_ft_collapsed_node *col = ft_collapsed_node_ptr(dd->d.nf);
	struct cds_ft_metadata *col_meta = cds_ft_item_to_metadata(
		(struct cds_ft_inode *) col);
	unsigned int remaining = key_len - dd->d.depth;
	const uint8_t *ik = *ik_p;
	bool need_explode = false;
	bool found = false;
	struct ft_col_view view;
	unsigned int e;

	ft_for_each_live_collapsed_entry_view_rcu(col, tier, stride, e, view) {
		struct cds_ft_inode_flag *child = view.child;
		struct cds_ft_inode_flag **child_addr;
		unsigned int slen = view.slen;
		bool match;

		/*
		 * Partial prefix match (detach prefix falls within
		 * this suffix): explode the collapsed node and
		 * restart descent.
		 */
		if (slen > remaining) {
			if (ft_key_cmp_ordinals(ik, view.suffix, remaining,
						remaining, false, NULL) == 0) {
				need_explode = true;
				break;
			}
			continue;
		}
		match = (ft_key_cmp_ordinals(ik, view.suffix, slen, slen,
					     false, NULL) == 0);
		if (!match)
			continue;
		if (!child)
			return CDS_FT_STATUS_NOT_FOUND;
		if (stride == FT_COL_STRIDE_NARROW)
			child_addr = &ft_collapsed_entries(col, tier)[e].child;
		else
			child_addr = &ft_collapsed_entries_wide(col, tier)[e].child;
		ft_detach_descent_track(dd, col_meta);
		dd->d.ppnf  = dd->d.pnf;
		dd->d.ppnfp = dd->d.pnfp;
		dd->d.pnf   = dd->d.nf;
		dd->d.pnfp  = dd->d.nfp;
		dd->d.nf    = child;
		dd->d.nfp   = child_addr;
		dd->d.depth += slen;
		ik += slen;
		if (dd->d.nf && dd->pending) {
			dd->det_nfp = dd->d.nfp;
			dd->pending = false;
		}
		*ik_p = ik;
		found = true;
		break;
	}
	if (!found && !need_explode)
		return CDS_FT_STATUS_NOT_FOUND;
	if (found)
		return CDS_FT_STATUS_OK;

	/*
	 * Partial-prefix match: explode the collapsed node into
	 * internals.  After explode, the freshly published internal
	 * subtree carries the same external_nodes (if any) that the
	 * collapsed node held, and density propagation reflects the
	 * footprint change.  dd->d.nf points at the new internal so
	 * the caller's next iteration re-dispatches via the regular
	 * internal-node path.
	 */
	{
		struct cds_ft_inode_flag *internal_flag;
		int err;

		err = ft_collapsed_explode_node(ft, col, dd->d.nf,
			dd->d.pnf, dd->d.nfp, dd->d.depth, &internal_flag);
		if (err < 0)
			return CDS_FT_STATUS_MEMORY_ERROR;
		dd->d.nf = internal_flag;
	}
	return CDS_FT_STATUS_OK;
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

	if (ft_density_pool_ensure(ft, FT_MAX_DEPTH))
		return CDS_FT_STATUS_MEMORY_ERROR;

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
		detached->collapse_threshold_pct = ft->collapse_threshold_pct;
		detached->collapse_scan_mul_pct = ft->collapse_scan_mul_pct;
		detached->compress_scan_mul_pct = ft->compress_scan_mul_pct;
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
			if (ft_node_collapsed(dd.d.nf)) {
				enum cds_ft_status s;

				s = ft_detach_descent_collapsed(ft, &dd, &ik,
						key_len);
				if (s != CDS_FT_STATUS_OK)
					return s;
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
			detached->collapse_threshold_pct = ft->collapse_threshold_pct;
			detached->collapse_scan_mul_pct = ft->collapse_scan_mul_pct;
			detached->compress_scan_mul_pct = ft->compress_scan_mul_pct;
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
				int ret = ft_detach_node(ft,
							 dd.det_nfp,
							 dd.det_pfp,
							 dd.det_depth);
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
				ft_nr_keys_store(ft, dmeta, detached_count, CMM_RELAXED);
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
	unsigned int type_idx;
	const struct cds_ft_type *type;
	struct cds_ft_metadata *rmeta;

	CDS_FT_SCOPED_READER(ft);
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
	 * A linear root with no live children and no external node
	 * represents an empty trie.  Use the pointer-array scan
	 * (ft_linear_node_is_empty) rather than the derive-via-sentinel
	 * nr_child: the latter over-reports "1" on a freshly-calloc'd
	 * root where values[0] == values[1] == 0.  Going through the
	 * pointer array is also read-side-safe (the metadata nr_child
	 * counter is write-side accounting).
	 */
	if (!ft_type_is_linear(type->type_class))
		return false;
	if (!ft_linear_node_is_empty(type, root_node))
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

#ifdef FEATURE_FT_COLLAPSE
/*
 * Collapsed node handling for cds_ft_count_keys_prefix.
 * Scan entries for a suffix that matches the remaining prefix bytes.
 * If prefix ends within a collapsed suffix span, return the child's nr_keys.
 * If no match, return 0.
 */
static
enum ft_descent_action ft_count_prefix_collapsed(struct cds_ft_inode_flag **node_flag_p,
		unsigned int *i_p, const uint8_t *prefix,
		size_t prefix_len, unsigned long *count_ret)
{
	struct cds_ft_inode_flag *node_flag = *node_flag_p;
	unsigned int i = *i_p;
	unsigned int tier = ft_collapsed_tier(node_flag);
	unsigned int stride = ft_collapsed_stride(node_flag);
	struct cds_ft_collapsed_node *col = ft_collapsed_node_ptr(node_flag);
	unsigned int remaining = prefix_len - i;
	struct ft_col_view view;
	unsigned int e;

	ft_for_each_live_collapsed_entry_view_rcu(col, tier, stride, e, view) {
		unsigned int slen = view.slen, j;
		bool match;

		if (slen >= remaining) {
			/* Suffix covers the rest of the prefix — compare. */
			match = true;
			for (j = 0; j < remaining; j++) {
				if (prefix[i + j] != view.suffix[j]) {
					match = false;
					break;
				}
			}
			if (!match)
				continue;
			/* Prefix ends within or at this entry's span. */
			if (!ft_node_external(view.child)) {
				struct cds_ft_metadata *m =
					cds_ft_item_to_metadata(ft_node_ptr(view.child));
				*count_ret = ft_nr_keys_load(m);
			} else if (view.child) {
				*count_ret = 1;
			} else {
				*count_ret = 0;
			}
			return FT_DESCENT_END;
		}
		/* Suffix is shorter than remaining prefix — check prefix. */
		match = true;
		for (j = 0; j < slen; j++) {
			if (prefix[i + j] != view.suffix[j]) {
				match = false;
				break;
			}
		}
		if (!match)
			continue;
		/* Full suffix match, continue descent into child. */
		*i_p = i + slen - 1;
		{
			struct cds_ft_inode_flag *resolved =
				ft_resolve_skip_compressed(view.child);
			*node_flag_p = resolved;
		}
		return FT_DESCENT_CONTINUE;
	}
	*count_ret = 0;
	return FT_DESCENT_END;
}
#else
static
enum ft_descent_action ft_count_prefix_collapsed(
		struct cds_ft_inode_flag **node_flag_p __attribute__((unused)),
		unsigned int *i_p __attribute__((unused)),
		const uint8_t *prefix __attribute__((unused)),
		size_t prefix_len __attribute__((unused)),
		unsigned long *count_ret __attribute__((unused)))
{
	return FT_DESCENT_END;
}
#endif /* FEATURE_FT_COLLAPSE */

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
		if (ft_node_collapsed(node_flag)) {
			enum ft_descent_action act;

			act = ft_count_prefix_collapsed(
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
	if (ft_node_collapsed(node_flag)) {
		struct cds_ft_metadata *col_meta =
			cds_ft_item_to_metadata(ft_node_ptr(node_flag));
		count = ft_nr_keys_load(col_meta);
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
static inline_lookup
enum ft_descent_action ft_lookup_nth_collapsed(
		struct cds_ft_inode_flag **node_flag_p,
		int *level_p, uint8_t *ordinal_key,
		struct cds_ft_iter *iter,
		unsigned long *remaining_p)
{
	struct cds_ft_inode_flag *node_flag = *node_flag_p;
	int level = *level_p;
	unsigned int tier = ft_collapsed_tier(node_flag);
	unsigned int stride = ft_collapsed_stride(node_flag);
	struct cds_ft_collapsed_node *col = ft_collapsed_node_ptr(node_flag);
	struct ft_col_view view;
	unsigned int e;

	/*
	 * Live entries appear in lexicographic order across slot
	 * indices, so a single forward pass visits them in ascending
	 * suffix order — no sort needed.
	 */
	ft_for_each_live_collapsed_entry_view_rcu(col, tier, stride, e, view) {
		struct cds_ft_inode_flag *child = view.child;
		unsigned long child_keys;
		unsigned int slen = view.slen;

		if (!child)
			continue;
		child_keys = ft_child_key_count(child);
		if (*remaining_p < child_keys) {
			/* Target is in this entry's subtree. */
			unsigned int k;

			for (k = 0; k < slen; k++) {
				ordinal_key[level - 1 + k] = view.suffix[k];
				iter_path_node(iter)[level + k] =
					ft_collapsed_node_flag_tier_stride(col, tier, stride);
			}
			if (child &&
			    ft_node_skip_compressed(child))
				child = ft_compressed_node_flag(
					ft_skip_to_compressed(child));
			node_flag = child;
			if (!node_flag || ft_node_external(node_flag)) {
				level += slen;
				if (node_flag)
					iter_path_node(iter)[level] = node_flag;
				*node_flag_p = node_flag;
				*level_p = level;
				return FT_DESCENT_BREAK;
			}
			level += slen - 1;
			iter_path_node(iter)[level + 1] = node_flag;
			*node_flag_p = node_flag;
			*level_p = level;
			return FT_DESCENT_CONTINUE;
		}
		*remaining_p -= child_keys;
	}
	return FT_DESCENT_BREAK;
}
#else
static inline_lookup
enum ft_descent_action ft_lookup_nth_collapsed(
		struct cds_ft_inode_flag **node_flag_p __attribute__((unused)),
		int *level_p __attribute__((unused)),
		uint8_t *ordinal_key __attribute__((unused)),
		struct cds_ft_iter *iter __attribute__((unused)),
		unsigned long *remaining_p __attribute__((unused)))
{
	return FT_DESCENT_BREAK;
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
		if (ft_node_collapsed(node_flag)) {
			enum ft_descent_action act;

			act = ft_lookup_nth_collapsed(&node_flag,
				&level, ordinal_key, iter,
				&remaining);
			if (act == FT_DESCENT_BREAK)
				break;
			continue;
		}

		/* Iterate children in ascending ordinal order. */
		pivot = -1;
		child = ft_node_get_direction(node_flag, pivot, &child_key, FT_RIGHT);
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
			child = ft_node_get_direction(node_flag, pivot, &child_key, FT_RIGHT);
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

#ifdef FEATURE_FT_COLLAPSE
/*
 * Collapsed node handling for cds_ft_lookup_nth_last.
 * Same as ft_lookup_nth_collapsed but walks entries in descending
 * suffix order.  Returns END if all entries exhausted (caller
 * should check external_nodes).
 */
static inline_lookup
enum ft_descent_action ft_lookup_nth_last_collapsed(
		struct cds_ft_inode_flag **node_flag_p,
		int *level_p, uint8_t *ordinal_key,
		struct cds_ft_iter *iter,
		unsigned long *remaining_p)
{
	struct cds_ft_inode_flag *node_flag = *node_flag_p;
	int level = *level_p;
	unsigned int tier = ft_collapsed_tier(node_flag);
	unsigned int stride = ft_collapsed_stride(node_flag);
	struct cds_ft_collapsed_node *col = ft_collapsed_node_ptr(node_flag);
	unsigned int cap = ft_collapsed_capacity_stride(tier, stride);
	unsigned int e;

	/*
	 * Live entries appear in lexicographic order across slot
	 * indices.  A reverse pass (high-to-low) visits them in
	 * descending suffix order — no sort needed.
	 */
	for (e = cap; e-- > 0; ) {
		struct ft_col_view view;
		struct cds_ft_inode_flag *child;
		unsigned long child_keys;
		unsigned int slen;

		ft_collapsed_load_entry_view_rcu(col, tier, stride, e, &view);
		if (view.slen == 0 || view.child == NULL)
			continue;
		child = view.child;
		slen = view.slen;
		if (!child)
			continue;
		child_keys = ft_child_key_count(child);
		if (*remaining_p < child_keys) {
			unsigned int k;

			for (k = 0; k < slen; k++) {
				ordinal_key[level - 1 + k] = view.suffix[k];
				iter_path_node(iter)[level + k] =
					ft_collapsed_node_flag_tier_stride(col, tier, stride);
			}
			if (child &&
			    ft_node_skip_compressed(child))
				child = ft_compressed_node_flag(
					ft_skip_to_compressed(child));
			node_flag = child;
			if (!node_flag || ft_node_external(node_flag)) {
				level += slen;
				if (node_flag)
					iter_path_node(iter)[level] = node_flag;
				*node_flag_p = node_flag;
				*level_p = level;
				return FT_DESCENT_BREAK;
			}
			level += slen - 1;
			iter_path_node(iter)[level + 1] = node_flag;
			*node_flag_p = node_flag;
			*level_p = level;
			return FT_DESCENT_CONTINUE;
		}
		*remaining_p -= child_keys;
	}
	return FT_DESCENT_END;
}
#else
static inline_lookup
enum ft_descent_action ft_lookup_nth_last_collapsed(
		struct cds_ft_inode_flag **node_flag_p __attribute__((unused)),
		int *level_p __attribute__((unused)),
		uint8_t *ordinal_key __attribute__((unused)),
		struct cds_ft_iter *iter __attribute__((unused)),
		unsigned long *remaining_p __attribute__((unused)))
{
	return FT_DESCENT_END;
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
		if (ft_node_collapsed(node_flag)) {
			enum ft_descent_action act;

			/* Reuse nth helper in reverse: walk entries largest-first. */
			act = ft_lookup_nth_last_collapsed(
				&node_flag, &level, ordinal_key,
				iter, &remaining);
			if (act == FT_DESCENT_BREAK)
				break;
			if (act == FT_DESCENT_END)
				goto check_ext_nth_last;
			continue;
		}

		/* Iterate children in descending ordinal order first. */
		pivot = FT_ENTRY_PER_NODE;
		child = ft_node_get_direction(node_flag, pivot, &child_key, FT_LEFT);
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
 * Match a collapsed node's entries against @key starting at @i.
 * Scans entries for a suffix match against key[i..], filling
 * ordinal_key + iter_path across the matched suffix span.  On hit,
 * advances *i_p past the matched span, follows the entry's child
 * (resolving skip-compressed if needed) into *node_flag_p, and
 * returns FT_DESCENT_CONTINUE.  Returns FT_DESCENT_END if no
 * entry's suffix matches or the matched child is NULL.
 */
static inline_lookup
enum ft_descent_action ft_rebuild_path_collapsed(
		struct cds_ft_inode_flag **node_flag_p,
		unsigned int *i_p,
		const uint8_t *key, size_t key_len,
		uint8_t *ordinal_key,
		struct cds_ft_iter *iter)
{
	struct cds_ft_inode_flag *node_flag = *node_flag_p;
	unsigned int tier = ft_collapsed_tier(node_flag);
	unsigned int stride = ft_collapsed_stride(node_flag);
	struct cds_ft_collapsed_node *col = ft_collapsed_node_ptr(node_flag);
	unsigned int i = *i_p;
	unsigned int remaining = key_len - i;
	struct ft_col_view view;
	unsigned int e;

	ft_for_each_live_collapsed_entry_view_rcu(col, tier, stride, e, view) {
		struct cds_ft_inode_flag *child;
		unsigned int slen = view.slen, j;
		bool match;

		if (slen > remaining)
			continue;
		match = true;
		for (j = 0; j < slen; j++) {
			uint8_t ord = key[i + j];

			if (ord != view.suffix[j]) {
				match = false;
				break;
			}
			ordinal_key[i + j] = ord;
			iter_path_node(iter)[i + j + 1] =
				(struct cds_ft_inode_flag *)
				ft_collapsed_node_flag_tier_stride(col, tier, stride);
		}
		if (!match)
			continue;
		i += slen - 1;
		child = view.child;
		if (!child)
			return FT_DESCENT_END;
		child = ft_resolve_skip_compressed(child);
		iter_path_node(iter)[i + 1] = child;
		*node_flag_p = child;
		*i_p = i;
		return FT_DESCENT_CONTINUE;
	}
	return FT_DESCENT_END;
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
		if (ft_node_collapsed(node_flag)) {
			enum ft_descent_action act;

			act = ft_rebuild_path_collapsed(&node_flag, &i,
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
 * Handle a collapsed ancestor in cds_ft_iter_skip_forward's walk-up
 * loop.  Skips intermediate path levels (same collapsed node at
 * adjacent levels).  At the entry level, scans entries to the right
 * of the current key's suffix and counts their keys.  When the
 * target falls within a rightward entry's subtree, fills
 * ordinal_key + iter_path across the suffix span, advances *level_p
 * and returns FT_DESCENT_BREAK so the caller can goto
 * descend_forward.  Otherwise returns FT_DESCENT_CONTINUE for
 * the caller to keep walking up.
 */
static inline_lookup
enum ft_descent_action ft_skip_forward_walk_up_collapsed(
		struct cds_ft_inode_flag *ancestor,
		int *level_p,
		int depth,
		unsigned long *remaining_p,
		uint8_t *ordinal_key,
		struct cds_ft_iter *iter)
{
	int level = *level_p;
	unsigned int tier = ft_collapsed_tier(ancestor);
	unsigned int stride = ft_collapsed_stride(ancestor);
	struct cds_ft_collapsed_node *col;
	struct ft_col_view view;
	unsigned int e;

	/* Skip intermediate levels (same node at adjacent levels). */
	if (level > 0 && iter_path_node(iter)[level - 1] == ancestor)
		return FT_DESCENT_CONTINUE;

	col = ft_collapsed_node_ptr(ancestor);

	ft_for_each_live_collapsed_entry_view_rcu(col, tier, stride, e, view) {
		struct cds_ft_inode_flag *child = view.child;
		unsigned int slen = view.slen;
		unsigned long ck;
		bool is_current;

		/*
		 * Check if this is the current entry by comparing
		 * suffix to ordinal_key.
		 */
		is_current = true;
		{
			unsigned int j;
			unsigned int cmp = slen;

			for (j = 0; j < cmp; j++) {
				if (view.suffix[j] != ordinal_key[level + j]) {
					is_current = false;
					break;
				}
			}
		}
		if (is_current)
			continue;

		/*
		 * Check if this entry is to the right (suffix > current
		 * key's suffix).
		 */
		{
			unsigned int j;
			unsigned int cur_key_slen = depth - level;
			unsigned int cmp = slen < cur_key_slen ? slen : cur_key_slen;
			int r = 0;

			for (j = 0; j < cmp; j++) {
				if (view.suffix[j] > ordinal_key[level + j]) {
					r = 1;
					break;
				}
				if (view.suffix[j] < ordinal_key[level + j]) {
					r = -1;
					break;
				}
			}
			if (r == 0 && slen > cur_key_slen)
				r = 1;
			if (r <= 0)
				continue;
		}
		ck = ft_child_key_count(child);
		if (*remaining_p <= ck) {
			unsigned int k;

			(*remaining_p)--;
			for (k = 0; k < slen; k++) {
				ordinal_key[level + k] = view.suffix[k];
				if (k > 0)
					iter_path_node(iter)[level + k + 1] = ancestor;
			}
			level += slen;
			iter_path_node(iter)[level] = child;
			*level_p = level;
			return FT_DESCENT_BREAK;
		}
		*remaining_p -= ck;
	}
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
			} else if (ft_node_collapsed(parent)) {
				/*
				 * Walk collapsed entries in ascending suffix
				 * order.  ft_lookup_nth_collapsed fills the
				 * iter path and ordinal_key through the
				 * matched suffix — don't overwrite them.
				 */
				enum ft_descent_action act;
				int lv = depth;

				act = ft_lookup_nth_collapsed(
					&parent, &lv, ordinal_key,
					iter, &remaining);
				if (act == FT_DESCENT_BREAK) {
					/* Leaf at path[lv]. */
					level = lv;
					goto descend_forward;
				}
				if (act == FT_DESCENT_CONTINUE) {
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
			enum ft_descent_action act;

			act = ft_skip_forward_walk_up_collapsed(ancestor,
				&level, depth, &remaining,
				ordinal_key, iter);
			if (act == FT_DESCENT_BREAK)
				goto descend_forward;
			assert(act == FT_DESCENT_CONTINUE);
			continue;
		}

		pivot = ordinal_key[level];
		child = ft_node_get_direction(ancestor, pivot,
				&child_key, FT_RIGHT);
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
				enum ft_descent_action act;

				act = ft_skip_forward_compressed(
					&node_flag, &level,
					ordinal_key, iter);
				if (act == FT_DESCENT_BREAK)
					break;
				continue;
			}
			if (ft_node_collapsed(node_flag)) {
				enum ft_descent_action act;

				act = ft_lookup_nth_collapsed(
					&node_flag, &level,
					ordinal_key, iter, &remaining);
				if (act == FT_DESCENT_BREAK)
					break;
				continue;
			}

			pivot = -1;
			child = ft_node_get_direction(node_flag, pivot,
					&child_key, FT_RIGHT);
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
 * Handle a collapsed ancestor in cds_ft_iter_skip_reverse's walk-up
 * loop.  Skips intermediate levels (same collapsed node at adjacent
 * levels).  At the entry level, scans entries to the left of the
 * current key's suffix and counts their keys; when the target falls
 * within a leftward entry's subtree, fills ordinal_key + iter_path
 * across the suffix span, advances *level_p and returns
 * FT_DESCENT_BREAK so the caller can goto descend_reverse.
 * Then checks the collapsed node's external_nodes (smallest key);
 * on match returns FT_DESCENT_END with the iter written and
 * *iter_status_p set to OK.  Otherwise returns FT_DESCENT_CONTINUE.
 */
static inline_lookup
enum ft_descent_action ft_skip_reverse_walk_up_collapsed(
		struct cds_ft_inode_flag *ancestor,
		int *level_p,
		int depth,
		unsigned long *remaining_p,
		uint8_t *ordinal_key,
		struct cds_ft_iter *iter)
{
	int level = *level_p;
	unsigned int tier = ft_collapsed_tier(ancestor);
	unsigned int stride = ft_collapsed_stride(ancestor);
	struct cds_ft_collapsed_node *col;
	struct ft_col_view view;
	struct cds_ft_metadata *col_meta;
	struct cds_ft_node *a_ext;
	unsigned int e;

	/* Skip intermediate levels. */
	if (level > 0 && iter_path_node(iter)[level - 1] == ancestor)
		return FT_DESCENT_CONTINUE;

	col = ft_collapsed_node_ptr(ancestor);
	col_meta = cds_ft_item_to_metadata((struct cds_ft_inode *) col);

	ft_for_each_live_collapsed_entry_view_rcu(col, tier, stride, e, view) {
		struct cds_ft_inode_flag *child = view.child;
		unsigned int slen = view.slen;
		unsigned long ck;

		/* Check if suffix < current key. */
		{
			unsigned int j;
			unsigned int cur_key_slen = depth - level;
			unsigned int cmp = slen < cur_key_slen ? slen : cur_key_slen;
			int r = 0;

			for (j = 0; j < cmp; j++) {
				if (view.suffix[j] < ordinal_key[level + j]) {
					r = -1;
					break;
				}
				if (view.suffix[j] > ordinal_key[level + j]) {
					r = 1;
					break;
				}
			}
			if (r == 0 && slen < cur_key_slen)
				r = -1;
			if (r >= 0)
				continue;
		}
		ck = ft_child_key_count(child);
		if (*remaining_p <= ck) {
			unsigned int k;

			(*remaining_p)--;
			for (k = 0; k < slen; k++) {
				ordinal_key[level + k] = view.suffix[k];
				if (k > 0)
					iter_path_node(iter)[level + k + 1] = ancestor;
			}
			level += slen;
			iter_path_node(iter)[level] = child;
			*level_p = level;
			return FT_DESCENT_BREAK;
		}
		*remaining_p -= ck;
	}

	/* Then check external_nodes (smallest key). */
	a_ext = ft_dereference_acquire(col_meta->external_nodes);
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
		if (ft_node_collapsed(ancestor)) {
			enum ft_descent_action act;

			act = ft_skip_reverse_walk_up_collapsed(ancestor,
				&level, depth, &remaining,
				ordinal_key, iter);
			if (act == FT_DESCENT_BREAK)
				goto descend_reverse;
			if (act == FT_DESCENT_END)
				goto end;
			assert(act == FT_DESCENT_CONTINUE);
			continue;
		}

		ameta = cds_ft_item_to_metadata(ft_node_ptr(ancestor));

		/* Count leftward siblings. */
		pivot = ordinal_key[level];
		child = ft_node_get_direction(ancestor, pivot,
				&child_key, FT_LEFT);
		while (child) {
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
			if (ft_node_collapsed(node_flag)) {
				enum ft_descent_action act;

				act = ft_lookup_nth_last_collapsed(
					&node_flag, &level,
					ordinal_key, iter, &remaining);
				if (act == FT_DESCENT_BREAK)
					break;
				if (act == FT_DESCENT_END)
					goto check_ext_descend_reverse;
				continue;
			}

			pivot = FT_ENTRY_PER_NODE;
			child = ft_node_get_direction(node_flag, pivot,
					&child_key, FT_LEFT);
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
	attr->speculative_key_len_offset = CDS_FT_SPECULATIVE_OFFSET_NONE;
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

enum cds_ft_status cds_ft_group_attr_set_speculative(struct cds_ft_group_attr *attr)
{
	/*
	 * Speculative-only mode requires skip-compressed pointer encoding
	 * to be useful: without it, cds_ft_lookup_candidate_key already
	 * does cand-mode descent regardless of the group attr (the
	 * candidate flag is gated by the API entry point, not the group),
	 * and cds_ft_lookup_key stays on the precise path (no offsets to
	 * validate against).  When skip-compressed is unavailable on the
	 * build/host, this attribute would be a no-op, so report it as
	 * unsupported rather than silently doing nothing.
	 */
#ifndef FEATURE_FT_SKIP_COMPRESSED
	(void) attr;
	return CDS_FT_STATUS_NOT_SUPPORTED;
#else
	if (!ft_skip_compressed_validate())
		return CDS_FT_STATUS_NOT_SUPPORTED;
	attr->speculative = true;
	attr->flags |= CDS_FT_FLAG_SKIP_COMPRESSED;
	return CDS_FT_STATUS_OK;
#endif
}

enum cds_ft_status cds_ft_group_attr_set_speculative_validated(
		struct cds_ft_group_attr *attr,
		size_t key_offset,
		size_t key_len_offset)
{
	/*
	 * Reject a non-NONE key_len_offset on a fixed-length-key group
	 * to catch misuse early — fixed-length groups derive key length
	 * from the trie and never read it from the external node.
	 */
	if (attr->key_len != CDS_FT_LEN_VARIABLE &&
	    key_len_offset != CDS_FT_SPECULATIVE_OFFSET_NONE)
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	/*
	 * Variable-length groups must provide a key-length offset; without
	 * it the library cannot know how many bytes to compare.
	 */
	if (attr->key_len == CDS_FT_LEN_VARIABLE &&
	    key_len_offset == CDS_FT_SPECULATIVE_OFFSET_NONE)
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	/*
	 * Opportunistically enable skip-compressed pointer encoding when
	 * available — it stacks with library-side validation to also
	 * avoid the compressed-node cache-line load.  Tolerate
	 * NOT_SUPPORTED here: validated speculative descent is still
	 * profitable without skip-compressed (cand-mode descent skips
	 * the compressed/collapsed byte verify, and the leaf compare
	 * uses the inline SIMD/SWAR comparator).
	 */
#ifdef FEATURE_FT_SKIP_COMPRESSED
	if (ft_skip_compressed_validate()) {
		attr->speculative = true;
		attr->flags |= CDS_FT_FLAG_SKIP_COMPRESSED;
	}
#endif
	attr->speculative_validated = true;
	attr->speculative_key_offset = key_offset;
	attr->speculative_key_len_offset = key_len_offset;
	return CDS_FT_STATUS_OK;
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

enum cds_ft_status cds_ft_attr_set_collapse_threshold(struct cds_ft_attr *attr,
		unsigned int threshold_pct)
{
	if (threshold_pct < CDS_FT_COLLAPSE_THRESHOLD_DEFAULT)
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	attr->collapse_threshold_pct = threshold_pct;
	return CDS_FT_STATUS_OK;
}

enum cds_ft_status cds_ft_attr_set_collapse_scan_mul(struct cds_ft_attr *attr,
		unsigned int scan_mul_pct)
{
	if (scan_mul_pct < 100U)
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	attr->collapse_scan_mul_pct = scan_mul_pct;
	return CDS_FT_STATUS_OK;
}

enum cds_ft_status cds_ft_attr_set_compress_scan_mul(struct cds_ft_attr *attr,
		unsigned int scan_mul_pct)
{
	if (scan_mul_pct < 100U)
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	attr->compress_scan_mul_pct = scan_mul_pct;
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
 * scope-exit hook runs cds_ft_verify + cds_ft_verify_density once
 * every @period mutations.
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

enum cds_ft_status cds_ft_collapse_threshold_set(struct cds_ft *ft,
		unsigned int threshold_pct)
{
	if (threshold_pct < CDS_FT_COLLAPSE_THRESHOLD_DEFAULT)
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	uatomic_store(&ft->collapse_threshold_pct, threshold_pct, CMM_RELAXED);
	return CDS_FT_STATUS_OK;
}

unsigned int cds_ft_collapse_threshold_get(struct cds_ft *ft)
{
	return uatomic_load(&ft->collapse_threshold_pct, CMM_RELAXED);
}

enum cds_ft_status cds_ft_collapse_scan_mul_set(struct cds_ft *ft,
		unsigned int scan_mul_pct)
{
	if (scan_mul_pct < 100U)
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	uatomic_store(&ft->collapse_scan_mul_pct, scan_mul_pct, CMM_RELAXED);
	return CDS_FT_STATUS_OK;
}

unsigned int cds_ft_collapse_scan_mul_get(struct cds_ft *ft)
{
	return uatomic_load(&ft->collapse_scan_mul_pct, CMM_RELAXED);
}

enum cds_ft_status cds_ft_compress_scan_mul_set(struct cds_ft *ft,
		unsigned int scan_mul_pct)
{
	if (scan_mul_pct < 100U)
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	uatomic_store(&ft->compress_scan_mul_pct, scan_mul_pct, CMM_RELAXED);
	return CDS_FT_STATUS_OK;
}

unsigned int cds_ft_compress_scan_mul_get(struct cds_ft *ft)
{
	return uatomic_load(&ft->compress_scan_mul_pct, CMM_RELAXED);
}

enum cds_ft_status _cds_ft_group_create(const struct cds_ft_group_attr *attr,
		struct cds_ft_group **result_ft_group,
		const struct rcu_flavor_struct *flavor)
{
	struct cds_ft_group *ft_group;
	size_t key_len = CDS_FT_LEN_DEFAULT,
	       max_key_len = FT_MAX_KEY_LEN;

#ifdef FT_USE_SPECIALIZED_SCAN
	ft_specialized_scan_layout_assert();
#endif
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
		ft_group->speculative_validated = attr->speculative_validated;
		ft_group->speculative_key_offset = attr->speculative_key_offset;
		ft_group->speculative_key_len_offset = attr->speculative_key_len_offset;
	} else {
		ft_group->key_map.identity = true;
		ft_group->speculative_key_len_offset = CDS_FT_SPECULATIVE_OFFSET_NONE;
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
	{
		/*
		 * Pick the collapse-threshold default based on the descent
		 * mode the trie will run in.  Cand-mode descent — used by
		 * skip-compressed groups (cds_ft_lookup_candidate_key) and by
		 * speculative_validated groups (cds_ft_lookup_key internally
		 * cand-descends + validates at leaf) — benefits from
		 * collapse, so use the skip-mode default (T=100).  Plain
		 * non-skip / non-speculative groups stick with the precise-
		 * descent default (T=DISABLED).
		 */
		unsigned int dflt_T = (ft_group_skip_compressed(ft_group) ||
				ft_group->speculative_validated)
			? CDS_FT_COLLAPSE_THRESHOLD_DEFAULT
			: CDS_FT_COLLAPSE_THRESHOLD_NONSKIP_DEFAULT;
		unsigned int dflt_c = CDS_FT_COLLAPSE_SCAN_MUL_PCT_DEFAULT;
		unsigned int dflt_m = CDS_FT_COMPRESS_SCAN_MUL_PCT_DEFAULT;

		if (attr) {
			ft->exclusive = attr->exclusive;
			ft->collapse_threshold_pct =
				attr->collapse_threshold_pct
				? attr->collapse_threshold_pct
				: dflt_T;
			ft->collapse_scan_mul_pct =
				attr->collapse_scan_mul_pct
				? attr->collapse_scan_mul_pct
				: dflt_c;
			ft->compress_scan_mul_pct =
				attr->compress_scan_mul_pct
				? attr->compress_scan_mul_pct
				: dflt_m;
		} else {
			ft->collapse_threshold_pct = dflt_T;
			ft->collapse_scan_mul_pct = dflt_c;
			ft->compress_scan_mul_pct = dflt_m;
		}
	}

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

	if (ft_density_pool_ensure(ft, FT_MAX_DEPTH)) {
		free_cds_ft_node(ft, root_node);
		free(ft);
		*result_ft = NULL;
		return CDS_FT_STATUS_MEMORY_ERROR;
	}

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
		fprintf(stderr, "  collapsed: alloc=%lu freed=%lu leaked=%ld\n",
			uatomic_read(&ft->nr_collapsed_alloc),
			uatomic_read(&ft->nr_collapsed_freed),
			(long)(uatomic_read(&ft->nr_collapsed_alloc) - uatomic_read(&ft->nr_collapsed_freed)));
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
	ft_density_pool_destroy(ft);
	uatomic_dec(&ft->group->nr_ft_instances, CMM_RELAXED);
	free(ft);
}

/*
 * Integrity verification.
 *
 * ft_verify_node_recursive: recursively verify structural invariants
 * starting at @node_flag (which may be internal, compressed, or
 * collapsed).  Returns 0 on success, -1 on first detected error
 * (with details printed to @out).  Must be called with mutual
 * exclusion wrt updaters.
 *
 * Checks performed:
 * - nr_child matches the actual count of non-NULL child slots.
 * - nr_keys equals the sum of children's nr_keys plus the count
 *   of unique keys from external node chains attached to this node.
 * - Parent pointers of children point back to the correct parent.
 * - Compressed node invariants (len > 0, no external_nodes).
 * - Collapsed node invariants (live entries consistent).
 *
 * Density counters are not verified: they are maintained
 * incrementally and serve as a heuristic for collapse decisions.
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
 * internal/collapsed/compressed node, or its slot's parent).
 *
 *   - Head's prev must equal @owner_flag (the parent flagged-pointer
 *     convention used by ft_metadata_set_external_nodes and by the
 *     slot-attached external publish in cds_ft_insert).
 *   - Each non-head node's prev must point to its predecessor.
 *   - No node may appear twice (cycle / aliasing across chains).  We
 *     reuse @visited so a node accidentally referenced from a second
 *     chain elsewhere in the trie is also caught.
 *   - When @path is non-NULL (path verification is enabled), every
 *     chain entry's stored key (at group->speculative_key_offset) is
 *     compared byte-for-byte against @path[0..@depth-1].  For
 *     variable-length groups, the leaf's stored length (read from
 *     speculative_key_len_offset) must equal @depth.  This is the
 *     end-to-end path/key consistency check: it catches a corrupted
 *     cn->key_bytes write or a wrong child-slot byte that would
 *     otherwise be silent under speculative-validated lookup (since
 *     the descent skips per-step key comparison and only the leaf
 *     compare at the lookup tail would notice).
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
		if (check_path) {
			const uint8_t *stored_key = (const uint8_t *) node +
					group->speculative_key_offset;

			if (group->speculative_key_len_offset !=
					CDS_FT_SPECULATIVE_OFFSET_NONE) {
				size_t stored_len = *(const size_t *)
					((const uint8_t *) node +
					 group->speculative_key_len_offset);

				if (stored_len != depth) {
					if (out)
						fprintf(out, "ft_verify: depth %u: external node %p stored key length %zu != trie depth %u\n",
							depth, node,
							stored_len, depth);
					return -1;
				}
			}
			if (depth > 0 && memcmp(stored_key, path, depth) != 0) {
				if (out)
					fprintf(out, "ft_verify: depth %u: external node %p stored key bytes diverge from trie path\n",
						depth, node);
				return -1;
			}
		}
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
			/* Internal/compressed/collapsed child. */
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

/*
 * Verify a collapsed node's invariants (parent pointer matches,
 * nr_child counts live entries, nr_keys aggregates child counts
 * plus any local NIL-key chain).  Recurse into each entry's child
 * (resolving skip-compressed if needed).
 */
static
int ft_verify_node_collapsed(const struct cds_ft *ft, FILE *out,
		struct ft_visited_set *visited,
		uint8_t *path,
		struct cds_ft_inode_flag *node_flag,
		struct cds_ft_inode_flag *expected_parent,
		unsigned int depth,
		unsigned long *out_nr_keys)
{
	unsigned int tier = ft_collapsed_tier(node_flag);
	unsigned int stride = ft_collapsed_stride(node_flag);
	struct cds_ft_collapsed_node *col = ft_collapsed_node_ptr(node_flag);
	struct cds_ft_metadata *col_meta = cds_ft_item_to_metadata(
		(struct cds_ft_inode *) col);
	struct cds_ft_node *external_nodes = col_meta->external_nodes;
	struct ft_col_view view;
	unsigned long total_child_keys = 0;
	unsigned long local_keys = 0;
	unsigned int live_children = 0;
	unsigned int e;

	/* Parent pointer check. */
	if (col_meta->parent != expected_parent) {
		if (out)
			fprintf(out, "ft_verify: depth %u: collapsed node %p parent mismatch: "
				"expected %p, got %p\n",
				depth, node_flag, expected_parent,
				col_meta->parent);
		return -1;
	}
	/* Count external nodes attached to this collapsed node's metadata. */
	if (external_nodes) {
		if (ft_verify_external_chain(ft, out, visited, path, node_flag,
				external_nodes, depth))
			return -1;
		local_keys = 1;	/* One unique key position. */
	}
#ifdef FEATURE_FT_COLLAPSE
	/*
	 * Per-slot encoding sanity (all capacity slots, not just the
	 * live ones the iterator yields).  Three valid states:
	 *   slen == 0, child == NULL : unused slot;
	 *   slen >  0, child == NULL : dead slot (post-removal);
	 *   slen >  0, child != NULL : live slot, slen <= stride max.
	 * The fourth combination (slen == 0, child != NULL) is invalid:
	 * a child published without its slen would be unreachable on the
	 * read side (slen drives suffix-byte consumption and depth
	 * advancement) and indicates a torn publish.
	 */
	{
		unsigned int capacity =
			ft_collapsed_capacity_stride(tier, stride);
		unsigned int max_suffix = ft_col_stride_suffix_max[stride];
		unsigned int slot;

		for (slot = 0; slot < capacity; slot++) {
			struct ft_col_view sv;

			ft_collapsed_load_entry_view_rcu(col, tier, stride,
					slot, &sv);
			if (sv.slen == 0 && sv.child != NULL) {
				if (out)
					fprintf(out, "ft_verify: depth %u: collapsed node %p slot %u has child %p with slen 0 (torn publish)\n",
						depth, node_flag, slot, sv.child);
				return -1;
			}
			if (sv.slen > 0 && sv.child != NULL &&
			    sv.slen > max_suffix) {
				if (out)
					fprintf(out, "ft_verify: depth %u: collapsed node %p slot %u live slen %u exceeds stride %u max %u\n",
						depth, node_flag, slot,
						sv.slen, stride, max_suffix);
				return -1;
			}
		}
	}
#endif
	/* Walk each collapsed entry. */
	ft_for_each_live_collapsed_entry_view_rcu(col, tier, stride, e, view) {
		unsigned int slen = view.slen;
		struct cds_ft_inode_flag *child = view.child;

		if (!child)
			continue;
		live_children++;
		/*
		 * Path tracking for this entry: write the entry's suffix
		 * bytes into the path buffer at [depth..depth+slen-1].
		 * Each live entry's suffix differs, so this must be
		 * refreshed per iteration.
		 */
		if (path)
			memcpy(path + depth, view.suffix, slen);
		if (ft_node_external(child)) {
			/* External leaf chain at this entry's slot. */
			if (ft_verify_external_chain(ft, out, visited, path,
					node_flag,
					(struct cds_ft_node *) ft_node_ptr(child),
					depth + slen))
				return -1;
			total_child_keys += 1;
		} else {
			struct cds_ft_inode_flag *child_resolved;
			unsigned long sub_keys = 0;

			if (ft_verify_skip_encoding(out, child, depth + slen))
				return -1;
			child_resolved = ft_resolve_skip_compressed(child);
			if (ft_verify_node_recursive(ft, out, visited, path,
					child_resolved,
					node_flag, depth + slen,
					&sub_keys))
				return -1;
			total_child_keys += sub_keys;
		}
	}
	/* Verify nr_child. */
	if (col_meta->nr_child != live_children) {
		if (out)
			fprintf(out, "ft_verify: depth %u: collapsed node %p nr_child mismatch: "
				"stored %u, counted %u\n",
				depth, node_flag, col_meta->nr_child,
				live_children);
		return -1;
	}
	/* Verify nr_keys. */
	{
		unsigned long stored_nr_keys = ft_nr_keys_get(col_meta);

		if (stored_nr_keys != total_child_keys + local_keys) {
			if (out)
				fprintf(out, "ft_verify: depth %u: collapsed node %p nr_keys mismatch: "
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
	 * (compressed, collapsed, internal) must be reached exactly
	 * once from the root.  A duplicate visit means either two
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
		 * Density / nr_keys promotion sanity.  The compact-vs-extended
		 * discriminator is m->nr_keys == UINT32_MAX; ft_density_promote
		 * assigns m->density_ext before publishing the sentinel via a
		 * release store.  A torn or rolled-back promotion that leaves
		 * the sentinel set without a backing density_ext would cause
		 * the very next ft_density_get / ft_nr_keys_get to dereference
		 * NULL — flag it here with a clean diagnostic instead.
		 */
		if (ft_density_is_extended(m) && m->density_ext == NULL) {
			if (out)
				fprintf(out, "ft_verify: depth %u: node %p has nr_keys==UINT32_MAX (promoted) but density_ext is NULL\n",
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
	if (ft_node_collapsed(node_flag))
		return ft_verify_node_collapsed(ft, out, visited, path,
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

			if (type->type_class != FT_LINEAR &&
			    type->type_class != FT_POPCOUNT &&
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
			/*
			 * Linear-node sentinel-array bounds.  The
			 * sentinel-derived count returned by
			 * ft_linear_node_get_nr_child is the position of
			 * the unwritten tail in the values[] array, not the
			 * live-children count: removals set the pointer to
			 * NULL but leave the value byte in place (avoiding
			 * an ABA on the value), and only a recompact reclaims
			 * the slot.  The well-defined invariants are:
			 *
			 *   - in_array <= type->max_linear_child   (capacity)
			 *   - in_array >= metadata->nr_child       (lazy slack)
			 *
			 * The upper bound is asserted on the
			 * ft_linear_node_replace_ptr hot path; the lower
			 * bound is the verify-time invariant — a stored
			 * nr_child larger than the touched-slot count would
			 * imply removal accounting got ahead of the array
			 * itself, which is impossible under the lazy-NULL
			 * scheme.
			 */
			if (type->type_class == FT_LINEAR) {
				uint8_t in_array =
					ft_linear_node_get_nr_child(type, node);

				if (in_array > type->max_linear_child) {
					if (out)
						fprintf(out, "ft_verify: depth %u: linear node %p (type %u) sentinel-derived nr_child %u exceeds max_linear_child %u\n",
							depth, node_flag,
							type_index,
							(unsigned int) in_array,
							(unsigned int) type->max_linear_child);
					return -1;
				}
				if (in_array < metadata->nr_child) {
					if (out)
						fprintf(out, "ft_verify: depth %u: linear node %p (type %u) sentinel-derived nr_child %u < metadata nr_child %u\n",
							depth, node_flag,
							type_index,
							(unsigned int) in_array,
							metadata->nr_child);
					return -1;
				}
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

			if (t_type->type_class == FT_PIGEON) {
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
 * Recursively walks every internal, compressed, and collapsed node
 * starting from the root, checking that nr_child, nr_keys, and
 * parent pointers are self-consistent.
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
	uint8_t path_buf[FT_MAX_KEY_LEN];
	uint8_t *path;
	int ret;

	if (ft_visited_init(&visited)) {
		if (out)
			fprintf(out, "ft_verify: visited-set allocation failed\n");
		return CDS_FT_STATUS_INTEGRITY_ERROR;
	}
	/*
	 * End-to-end path/key consistency (invariant 10) is only
	 * meaningful when the group is configured for speculative
	 * validation: in that mode the user's leaf node carries an
	 * addressable copy of the inserted key at
	 * group->speculative_key_offset, in ordinal space (key_map
	 * identity is the gating condition the spec_validate lookup
	 * itself enforces — non-identity maps would require a per-byte
	 * conversion of the path at compare time, which we skip).
	 *
	 * When enabled, the path buffer is filled in during descent and
	 * compared at each external leaf in ft_verify_external_chain.
	 * When disabled, leave path = NULL and all path-tracking writes
	 * / compares short-circuit.
	 */
	if (ft->group->speculative_validated && ft->group->key_map.identity)
		path = path_buf;
	else
		path = NULL;
	ret = ft_verify_node_recursive(ft, out, &visited, path, root, NULL, 0,
			&root_nr_keys);
	ft_visited_destroy(&visited);
	if (ret)
		return CDS_FT_STATUS_INTEGRITY_ERROR;
	return CDS_FT_STATUS_OK;
}

/* Forward declaration so the per-kind helpers below can recurse. */
static
int ft_verify_density_recursive(const struct cds_ft *ft, FILE *out,
		struct cds_ft_inode_flag *node_flag,
		unsigned int depth);

/*
 * Recompute density for a compressed node from its single child's
 * contribution and compare against the stored counters.  Recurse
 * into the child first so children are checked before parents.
 * Returns the number of mismatches found in this subtree (0 =
 * clean).
 */
static
int ft_verify_density_compressed(const struct cds_ft *ft, FILE *out,
		struct cds_ft_inode_flag *node_flag,
		unsigned int depth)
{
	struct cds_ft_compressed_node *cn = ft_compressed_node_ptr(node_flag);
	struct cds_ft_metadata *cn_meta = cds_ft_item_to_metadata(
		(struct cds_ft_inode *) cn);
	unsigned long accum[FT_NODE_DENSITY_DEPTH] = { 0 };
	int errors = 0;
	unsigned int j;

	if (cn->child && !ft_node_external(cn->child)) {
		errors += ft_verify_density_recursive(ft, out,
				cn->child, depth + cn->len);
		if (cn->len <= FT_NODE_DENSITY_DEPTH) {
			struct cds_ft_metadata *cm = cds_ft_item_to_metadata(
				ft_node_ptr(cn->child));

			ft_child_density_contribution_all(cm, cn->len,
				ft_node_readside_footprint(ft, cn->child),
				accum);
		}
	}
	for (j = 0; j < FT_NODE_DENSITY_DEPTH; j++) {
		unsigned long stored = ft_density_get(cn_meta, j);

		if (stored != accum[j]) {
			if (out)
				fprintf(out, "ft_verify_density: depth %u: compressed node %p "
					"density[%u] mismatch: stored %lu, computed %lu\n",
					depth, node_flag, j, stored, accum[j]);
			errors++;
			break;	/* one message per node */
		}
	}
	return errors;
}

/*
 * Recompute density for a collapsed node from each entry's child
 * contribution (resolving skip-compressed children) and compare
 * against the stored counters.  Returns the number of mismatches
 * found in this subtree (0 = clean).
 */
static
int ft_verify_density_collapsed(const struct cds_ft *ft, FILE *out,
		struct cds_ft_inode_flag *node_flag,
		unsigned int depth)
{
	unsigned int tier = ft_collapsed_tier(node_flag);
	unsigned int stride = ft_collapsed_stride(node_flag);
	struct cds_ft_collapsed_node *col = ft_collapsed_node_ptr(node_flag);
	struct cds_ft_metadata *col_meta = cds_ft_item_to_metadata(
		(struct cds_ft_inode *) col);
	struct ft_col_view view;
	unsigned long accum[FT_NODE_DENSITY_DEPTH] = { 0 };
	int errors = 0;
	unsigned int e, j;

	ft_for_each_live_collapsed_entry_view_rcu(col, tier, stride, e, view) {
		unsigned int slen = view.slen;
		struct cds_ft_inode_flag *child = view.child;

		if (!child)
			continue;
		/* Resolve skip-compressed: see ft_init_node_density. */
		child = ft_resolve_skip_compressed(child);
		if (ft_node_external(child))
			continue;
		errors += ft_verify_density_recursive(ft, out,
				child, depth + slen);
		if (slen <= FT_NODE_DENSITY_DEPTH) {
			struct cds_ft_metadata *cm = ft_flag_to_metadata(child);

			ft_child_density_contribution_all(cm, slen,
				ft_node_readside_footprint(ft, child),
				accum);
		}
	}
	for (j = 0; j < FT_NODE_DENSITY_DEPTH; j++) {
		unsigned long stored = ft_density_get(col_meta, j);

		if (stored != accum[j]) {
			if (out)
				fprintf(out, "ft_verify_density: depth %u: collapsed node %p "
					"density[%u] mismatch: stored %lu, computed %lu\n",
					depth, node_flag, j, stored, accum[j]);
			errors++;
			break;
		}
	}
	return errors;
}

/*
 * ft_verify_density_recursive: walk the trie bottom-up, recompute
 * density counters from children, compare with stored values.
 *
 * Returns the number of nodes with density mismatches (0 = clean).
 * Prints diagnostics to @out when non-NULL.
 */
static
int ft_verify_density_recursive(const struct cds_ft *ft, FILE *out,
		struct cds_ft_inode_flag *node_flag,
		unsigned int depth)
{
	int errors = 0;

	if (ft_node_compressed(node_flag))
		return ft_verify_density_compressed(ft, out, node_flag, depth);
	if (ft_node_collapsed(node_flag))
		return ft_verify_density_collapsed(ft, out, node_flag, depth);

	/* Internal node. */
	{
		struct cds_ft_metadata *metadata =
			cds_ft_item_to_metadata(ft_node_ptr(node_flag));
		unsigned long accum[FT_NODE_DENSITY_DEPTH] = { 0 };
		unsigned int key, j;

		for (key = 0; key < 256; key++) {
			struct cds_ft_inode_flag *child =
				ft_node_get_nth(node_flag, NULL, (uint8_t) key, FT_PF_NONE);

			if (!child || ft_node_external(child))
				continue;
			errors += ft_verify_density_recursive(ft, out,
					child, depth + 1);
			{
				struct cds_ft_metadata *cm =
					cds_ft_item_to_metadata(
						ft_node_ptr(child));
				ft_child_density_contribution_all(
					cm, 1,
					ft_node_readside_footprint(ft, child),
					accum);
			}
		}
		for (j = 0; j < FT_NODE_DENSITY_DEPTH; j++) {
			unsigned long stored = ft_density_get(metadata, j);

			if (stored != accum[j]) {
				if (out)
					fprintf(out, "ft_verify_density: depth %u: internal node %p "
						"density[%u] mismatch: stored %lu, computed %lu\n",
						depth, node_flag, j, stored, accum[j]);
				errors++;
				break;
			}
		}
		return errors;
	}
}

enum cds_ft_status cds_ft_verify_density(const struct cds_ft *ft, FILE *out)
{
	if (ft_verify_density_recursive(ft, out, ft->root, 0) > 0)
		return CDS_FT_STATUS_INTEGRITY_ERROR;
	return CDS_FT_STATUS_OK;
}

#ifdef FEATURE_FT_VERIFY_AT_MUTATION
/*
 * Hook called from CDS_FT_SCOPED_WRITER's scope-exit, before the
 * writer claim is released.  Sampled by the per-trie
 * @verify_at_mutation_period: the full cds_ft_verify +
 * cds_ft_verify_density walk runs once every @period mutations.  The
 * counter is incremented and reset on the boundary so it never
 * exceeds @period - 1, avoiding any overflow / cadence-drift issue
 * on long-running workloads.  Period 0 disables the walk entirely
 * (only the increment-and-compare runs).  On any mismatch, both
 * verifiers run to completion before aborting so we get the full
 * diagnostic.
 */
void ft_writer_scope_verify(struct cds_ft *ft)
{
	unsigned long period = ft->verify_at_mutation_period;
	bool fail = false;

	if (period == 0)
		return;
	ft->verify_at_mutation_counter++;
	if (ft->verify_at_mutation_counter < period)
		return;
	ft->verify_at_mutation_counter = 0;

	if (cds_ft_verify(ft, stderr) != CDS_FT_STATUS_OK)
		fail = true;
	if (cds_ft_verify_density(ft, stderr) != CDS_FT_STATUS_OK)
		fail = true;
	if (fail) {
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
	unsigned int tier = ft_collapsed_tier(node_flag);
	unsigned int stride = ft_collapsed_stride(node_flag);
	struct cds_ft_collapsed_node *col = ft_collapsed_node_ptr(node_flag);
	struct cds_ft_metadata *col_meta =
		cds_ft_item_to_metadata((struct cds_ft_inode *) col);
	unsigned int cap = ft_collapsed_capacity_stride(tier, stride);
	unsigned int live = ft_collapsed_count_live(col, tier, stride);
	struct cds_ft_node *external_nodes = rcu_dereference(col_meta->external_nodes);
	struct ft_col_view view;
	unsigned int e;

	print_indent(out, level);
	fprintf(out, "Level %d, COLLAPSED node: %p, tier=%u, stride=%u, capacity=%u, live=%u, order=%zu, nr_keys: %lu, density: ",
		level, node_flag, tier, stride, cap, live,
		cds_ft_item_order(col),
		ft_nr_keys_get(col_meta));
	print_density(out, col_meta);
	fprintf(out, "\n");
	if (external_nodes) {
		print_indent(out, level);
		fprintf(out, "Level %d, (meta)external node list ptr: %p\n",
			level, external_nodes);
	}

	ft_for_each_live_collapsed_entry_view_rcu(col, tier, stride, e, view) {
		unsigned int slen = view.slen;
		struct cds_ft_inode_flag *child = view.child;
		unsigned int k;

		print_indent(out, level);
		fprintf(out, "  entry[%u]: suffix=[", e);
		for (k = 0; k < slen; k++)
			fprintf(out, "%s%u", k ? "," : "", view.suffix[k]);
		fprintf(out, "] (len=%u), child=%p", slen, child);
		if (child) {
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
		if (child && !ft_node_external(child))
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

		child_node_flag = ft_node_get_nth(node_flag, NULL, (uint8_t) key, FT_PF_NONE);
		if (!child_node_flag)
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
				ft_nr_keys_get(metadata));
			print_density(out, metadata);
			fprintf(out, "\n");
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
		fprintf(out, "Level 0: root node %p, density: ", node_flag);
		print_density(out, rm);
		fprintf(out, "\n");
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
 *   Collapsed:  { "ptr", "kind": "COLLAPSED", "level", "scan_zone_size",
 *                 "external_nodes"?, "entries": [{"suffix", "child"}] }
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
		case FT_LINEAR:
			switch (order) {
			case 4: return "LINEAR_16";
			}
			break;
		case FT_POPCOUNT:
			/*
			 * Orders 5/6/7 are 2-level popcount_2l layouts
			 * (P2L); orders 8/9/10 are 1-level popcount_1l
			 * layouts (P1L: a single 256-bit bitmap + ptr table).
			 */
			switch (order) {
			case 5:  return "P2L_32";
			case 6:  return "P2L_64";
			case 7:  return "P2L_128";
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
void json_emit_density(FILE *out, const struct cds_ft_metadata *m)
{
	int i;

	fprintf(out, "[");
	for (i = 0; i < FT_NODE_DENSITY_DEPTH; i++) {
		if (i) fprintf(out, ",");
		fprintf(out, "%lu", ft_density_get(m, i));
	}
	fprintf(out, "]");
}

static
void json_emit_collapsed(const struct cds_ft *ft, FILE *out,
		struct cds_ft_inode_flag *node_flag, int level)
{
	unsigned int tier = ft_collapsed_tier(node_flag);
	unsigned int stride = ft_collapsed_stride(node_flag);
	struct cds_ft_collapsed_node *col = ft_collapsed_node_ptr(node_flag);
	struct cds_ft_metadata *col_meta =
		cds_ft_item_to_metadata((struct cds_ft_inode *) col);
	unsigned int cap = ft_collapsed_capacity_stride(tier, stride);
	unsigned int live = ft_collapsed_count_live(col, tier, stride);
	struct cds_ft_node *external_nodes = rcu_dereference(col_meta->external_nodes);
	struct ft_col_view view;
	unsigned int e, printed = 0;

	fprintf(out, "{\"ptr\":\"%p\",\"kind\":\"COLLAPSED\",\"level\":%d,"
		"\"tier\":%u,\"stride\":%u,\"capacity\":%u,\"nr_live\":%u",
		node_flag, level, tier, stride, cap, live);
	if (external_nodes)
		fprintf(out, ",\"external_nodes\":\"%p\"",
			(void *) external_nodes);
	fprintf(out, ",\"entries\":[");
	ft_for_each_live_collapsed_entry_view_rcu(col, tier, stride, e, view) {
		unsigned int slen = view.slen;
		struct cds_ft_inode_flag *child = view.child;
		unsigned int k;

		if (printed++) fprintf(out, ",");
		fprintf(out, "{\"slot\":%u,\"suffix\":[", e);
		for (k = 0; k < slen; k++) {
			if (k) fprintf(out, ",");
			fprintf(out, "%u", view.suffix[k]);
		}
		fprintf(out, "],\"child\":");
		if (child)
			json_emit_node(ft, out, child, level + slen);
		else
			fprintf(out, "null");
		fprintf(out, "}");
	}
	fprintf(out, "]}");
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
	if (ft_node_collapsed(node_flag)) {
		json_emit_collapsed(ft, out, node_flag, level);
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
	uint64_t nr_collapsed_nodes;
	uint64_t nr_collapsed_tier[FT_COL_NR_TIERS];
	struct cds_ft_node_stats node_stats[FT_TYPE_MAX_NR];
	bool has_nodes;
};

/*
 * Trie-wide collapsed-node distributions.
 *
 * collapsed_nr_entries_dist: histogram of nr_entries (= leaf count)
 *   per collapsed node, log2-bucketed:
 *     [0]=[2..3] [1]=[4..7] [2]=[8..15] [3]=[16..31]
 *     [4]=[32..63] [5]=[64..127] [6]=[128..255] [7]=[256]
 *
 * collapsed_suffix_len_dist: histogram of per-entry suffix lengths
 *   (in bytes), bucket FT_COLLAPSE_SLEN_MAX_BIN holds suffix_len >=
 *   that bin.
 */
#define FT_COLLAPSE_NR_ENTRIES_BUCKETS	8
#define FT_COLLAPSE_SLEN_MAX_BIN	33

struct cds_ft_stats {
	struct cds_ft_stats_level level[FT_MAX_DEPTH];
	uint64_t collapsed_nr_entries_dist[FT_COLLAPSE_NR_ENTRIES_BUCKETS];
	uint64_t collapsed_suffix_len_dist[FT_COLLAPSE_SLEN_MAX_BIN + 1];
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
void calc_stats_collapsed(const struct cds_ft *ft,
		struct cds_ft_inode_flag *node_flag,
		struct cds_ft_stats *stats, int level)
{
	unsigned int tier = ft_collapsed_tier(node_flag);
	unsigned int stride = ft_collapsed_stride(node_flag);
	struct cds_ft_collapsed_node *col = ft_collapsed_node_ptr(node_flag);
	struct cds_ft_metadata *col_meta =
		cds_ft_item_to_metadata((struct cds_ft_inode *) col);
	struct cds_ft_node *external_nodes = rcu_dereference(col_meta->external_nodes);
	struct ft_col_view view;
	unsigned int e;

	stats->level[level].nr_internal_nodes++;
	stats->level[level].nr_collapsed_nodes++;
	if (tier < FT_COL_NR_TIERS)
		stats->level[level].nr_collapsed_tier[tier]++;
	stats->level[level].has_nodes = true;
	{
		unsigned int count = ft_collapsed_count_live(col, tier, stride);
		unsigned int bucket;

		if (count >= 256)
			bucket = 7;
		else if (count >= 128)
			bucket = 6;
		else if (count >= 64)
			bucket = 5;
		else if (count >= 32)
			bucket = 4;
		else if (count >= 16)
			bucket = 3;
		else if (count >= 8)
			bucket = 2;
		else if (count >= 4)
			bucket = 1;
		else
			bucket = 0;
		stats->collapsed_nr_entries_dist[bucket]++;
	}
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
	ft_for_each_live_collapsed_entry_view_rcu(col, tier, stride, e, view) {
		unsigned int slen = view.slen;
		unsigned int j;
		struct cds_ft_inode_flag *child = view.child;
		int child_level;

		{
			unsigned int slen_bin = (slen > FT_COLLAPSE_SLEN_MAX_BIN)
				? FT_COLLAPSE_SLEN_MAX_BIN : slen;

			stats->collapsed_suffix_len_dist[slen_bin]++;
		}
		for (j = 1; j < slen; j++) {
			stats->level[level + j].nr_internal_nodes++;
			stats->level[level + j].nr_compressed_nodes++;
			stats->level[level + j].has_nodes = true;
		}
		child_level = level + slen;
		/*
		 * Skip-compressed: the slot stores cn->child OR'd with
		 * cn->len in the high bits.  Account for cn (one
		 * explicit compressed) and its implicit path
		 * (skip_len - 1 levels), then strip and re-classify the
		 * underlying child.
		 */
		if (ft_node_skip_compressed(child)) {
			unsigned int skip = ft_skip_len(child);
			unsigned int k;

			for (k = 0; k < skip; k++) {
				stats->level[child_level + k].nr_internal_nodes++;
				stats->level[child_level + k].nr_compressed_nodes++;
				stats->level[child_level + k].has_nodes = true;
			}
			child = ft_skip_child_ptr(child);
			child_level += skip;
		}
		/*
		 * The collapsed entry's child slot, after skip strip,
		 * may directly carry an explicit compressed pointer
		 * (paths longer than FT_SKIP_LEN_MAX, or non-skip
		 * groups).  Mirror the compressed branch from
		 * calc_stats_node_recursive: count cn + implicit path,
		 * then descend into cn->child.  calc_stats_node_recursive
		 * cannot iterate compressed children itself
		 * (ft_node_get_nth_skip returns NULL on a non-internal
		 * tag), so the descent must happen here.
		 */
		if (child && ft_node_compressed(child)) {
			struct cds_ft_compressed_node *cn =
				ft_compressed_node_ptr(child);
			unsigned int k;

			stats->level[child_level].nr_internal_nodes++;
			stats->level[child_level].nr_compressed_nodes++;
			stats->level[child_level].has_nodes = true;
			for (k = 1; k < cn->len; k++) {
				stats->level[child_level + k].nr_internal_nodes++;
				stats->level[child_level + k].nr_compressed_nodes++;
				stats->level[child_level + k].has_nodes = true;
			}
			if (cn->child &&
			    !ft_node_external(cn->child)) {
				calc_stats_node_recursive(ft, cn->child,
					stats, child_level + cn->len);
			} else if (cn->child) {
				struct cds_ft_node *iter_node;
				unsigned int count = 0;

				iter_node = (struct cds_ft_node *) ft_node_ptr(cn->child);
				cds_ft_for_each_duplicate(iter_node) {
					if (count++ == 0)
						stats->level[child_level + cn->len].nr_external_nodes++;
					else
						stats->level[child_level + cn->len].nr_duplicate_external_nodes++;
					stats->level[child_level + cn->len].has_nodes = true;
				}
			}
		} else if (child && !ft_node_external(child)) {
			calc_stats_node_recursive(ft, child,
				stats, child_level);
		} else if (child) {
			struct cds_ft_node *iter_node;
			unsigned int count = 0;

			iter_node = (struct cds_ft_node *) ft_node_ptr(child);
			cds_ft_for_each_duplicate(iter_node) {
				if (count++ == 0)
					stats->level[child_level].nr_external_nodes++;
				else
					stats->level[child_level].nr_duplicate_external_nodes++;
				stats->level[child_level].has_nodes = true;
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

		child_node_flag = ft_node_get_nth(node_flag, NULL, (uint8_t) key, FT_PF_NONE);
		if (!child_node_flag)
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
		if (stats_level->nr_collapsed_nodes) {
			print_indent(out, 1);
			fprintf(out, "Collapsed nodes: %" PRIu64
				" (T0: %" PRIu64 ", T1: %" PRIu64
				", T2: %" PRIu64 ", T3: %" PRIu64 ")\n",
				stats_level->nr_collapsed_nodes,
				stats_level->nr_collapsed_tier[0],
				stats_level->nr_collapsed_tier[1],
				stats_level->nr_collapsed_tier[2],
				stats_level->nr_collapsed_tier[3]);
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
	{
		static const char *const nr_entries_labels[
			FT_COLLAPSE_NR_ENTRIES_BUCKETS] = {
			"2-3", "4-7", "8-15", "16-31",
			"32-63", "64-127", "128-255", "256",
		};
		uint64_t total = 0;
		unsigned int b;
		bool any_collapse = false;

		for (b = 0; b < FT_COLLAPSE_NR_ENTRIES_BUCKETS; b++) {
			total += stats->collapsed_nr_entries_dist[b];
			if (stats->collapsed_nr_entries_dist[b])
				any_collapse = true;
		}
		if (any_collapse) {
			fprintf(out, "Collapsed nodes (trie-wide): %"
				PRIu64 "\n", total);
			print_indent(out, 1);
			fprintf(out, "nr_entries distribution:");
			for (b = 0; b < FT_COLLAPSE_NR_ENTRIES_BUCKETS; b++) {
				if (stats->collapsed_nr_entries_dist[b])
					fprintf(out, " [%s]=%" PRIu64,
						nr_entries_labels[b],
						stats->collapsed_nr_entries_dist[b]);
			}
			fprintf(out, "\n");
			{
				uint64_t slen_total = 0;
				unsigned int s;

				for (s = 0; s <= FT_COLLAPSE_SLEN_MAX_BIN; s++)
					slen_total +=
						stats->collapsed_suffix_len_dist[s];
				print_indent(out, 1);
				fprintf(out,
					"entry suffix_len distribution (n=%"
					PRIu64 "):", slen_total);
				for (s = 0; s <= FT_COLLAPSE_SLEN_MAX_BIN; s++) {
					if (stats->collapsed_suffix_len_dist[s]) {
						const char *suffix =
							(s == FT_COLLAPSE_SLEN_MAX_BIN)
							? "+" : "";
						fprintf(out, " [%u%s]=%"
							PRIu64,
							s, suffix,
							stats->collapsed_suffix_len_dist[s]);
					}
				}
				fprintf(out, "\n");
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
	size_t key_size  = max_key_len * sizeof(uint8_t);
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
