// SPDX-FileCopyrightText: 2012-2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-only

/*
 * src/fractal-trie.c
 *
 * Userspace RCU library - Fractal Trie
 *
 * ===================== High-level architecture =====================
 *
 * The Fractal Trie is a concurrent, RCU-protected ordered map from
 * opaque byte keys to application-owned nodes.  Lookups and ordered
 * traversals are wait-free under the RCU read-side lock and run
 * concurrently with mutations; writers are serialized by a
 * caller-provided mutex.  The public contract (semantics, guarantees,
 * locking rules, error codes) lives in <urcu/fractal-trie.h>; this
 * comment is the implementation map.
 *
 * Source layout
 * -------------
 *   fractal-trie.c          core: read descent; point / range / rank
 *                           lookups; ordered iteration; insert / remove /
 *                           replace; the bulk ops; compaction.
 *   fractal-trie-internal.h struct layouts, tagged-pointer encodings, the
 *                           node-type table, and inline helpers.
 *   fractal-trie-alloc.c    the strided internal-node allocator and the
 *                           external (leaf) buddy arena.
 *   urcu-flip-latch.h       the atomic multi-pointer "flip" primitive
 *                           used to commit a set of edges at once.
 *
 * Node model
 * ----------
 * Internal nodes self-adapt to child density: cascaded popcount bitmaps
 * for small / medium fan-out and a 256-entry "pigeon" array for dense
 * nodes, sized in powers of two.  The node type / configuration is
 * encoded in the low (tag) bits of the child pointer, so the read path
 * dispatches with no extra load.  External (leaf) nodes are
 * application-owned (they embed struct cds_ft_node); same-key duplicates
 * form a next-linked chain off the head.
 *
 * Path compression
 * ----------------
 * Single-child chains collapse into compressed (Patricia / ART-style)
 * nodes that store the shared bytes inline.  On 64-bit arches with free
 * high pointer bits, the "skip-compressed" encoding packs the skip
 * length and child pointer into the parent slot, so a speculative
 * descent bypasses the compressed node's cache line entirely
 * (FEATURE_FT_SKIP_COMPRESSED; -DNO_FEATURE_FT_COMPRESS disables
 * compression altogether).
 *
 * Memory layout
 * -------------
 * The internal-node allocator strides item data and metadata onto
 * separate cache lines, so the read hot path touches only the dense item
 * region -- which is why resident memory overstates the cache-hot
 * working set (see fractal-trie-alloc.c).  Internal nodes are reclaimed
 * via call_rcu.
 *
 * Ordered iteration
 * -----------------
 * When enabled (the default), the library threads the duplicate-chain
 * heads into a key-ordered list of small library-owned "ordinal cells",
 * kept off the descent hot path; cds_ft_next / cds_ft_prev and the
 * batched cell walk step that list.  Disabling it
 * (cds_ft_group_attr_set_ordered_list false) drops the per-key cell for
 * lower memory and faster mutations, at the cost of ordered iteration.
 *
 * Read descent
 * ------------
 * Two descent encodings per group (enum cds_ft_lookup_optimization):
 * SPECULATIVE skips per-node byte comparison and returns a candidate the
 * caller (or the speculative-lookup wrapper) validates; EAGER compares
 * exactly at each step.  Both return verified results.  Inequality and
 * rank / skip queries use per-node key counters to skip whole subtrees
 * in O(depth).  Going back up -- for next / prev / remove and for key
 * reconstruction -- follows parent back-pointers (and the cell list for
 * ordered walks) rather than a recorded descent path.
 *
 * Mutation and concurrency model
 * ------------------------------
 * Writers are serialized by a caller-provided mutex (CDS_FT_SCOPED_WRITER
 * only VALIDATES that exclusion; it is not itself a lock).  Every change
 * to published state follows a build-invisibly -> publish -> reclaim
 * discipline: a new node cluster is assembled where readers cannot reach
 * it, made visible by a single release store (or, for a multi-edge
 * commit such as a non-empty merge, one urcu-flip-latch commit that flips
 * all affected edges at once), and the displaced nodes are freed after a
 * grace period.  A reader therefore always observes a complete
 * prior-or-result state, never a partial one.  The discipline is spelled
 * out in the rcu-mutation rules and checked by the verify-at-mutation
 * build option (-DFEATURE_FT_VERIFY_AT_MUTATION).
 *
 * The bulk ops (graft, graft-swap, detach, merge, merge_at) move or
 * combine whole sub-tries between tries of one group; the union cases
 * spine-copy the overlap and commit through the flip latch.  A trie is
 * either exclusive (no concurrent readers; reclaim is synchronous and a
 * source graft skips its drain) or concurrent (RCU readers permitted).
 *
 * Reclamation
 * -----------
 * Deferred frees go through call_rcu; a fully-drained allocator range
 * releases its pages (MADV_DONTNEED on Linux) from inside the callback.
 * cds_ft_compact relocates live nodes into dense fresh ranges to recover
 * fragmentation; the emptied ranges reclaim after a grace period.
 * ===================================================================
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
#include "urcu-flip-latch.h"

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
	/*
	 * Byte offset from the (struct cds_ft_node *) stored in the trie to
	 * the start of the caller-stored key bytes, and whether it was
	 * configured.  Lets the speculative inequality path copy a result
	 * key from the leaf instead of rebuilding it from compressed-node
	 * bytes.  See cds_ft_group_attr_set_speculative_key_offset.
	 */
	size_t speculative_key_offset;
	bool speculative_key_offset_set;
	/*
	 * Offset from the (struct cds_ft_node *) to a size_t holding the leaf's
	 * key length in the caller's leaf, and whether configured.  Lets the
	 * ordered-list fast path materialize a variable-length result key from
	 * the leaf without the descent's per-level length computation.  See
	 * cds_ft_group_attr_set_key_len_offset.
	 */
	size_t key_len_offset;
	bool key_len_offset_set;
	/*
	 * Enable the library-owned ordered sibling list (FEATURE_FT_ORD_CELL):
	 * order links live in library-owned ordinal cells, not the app leaf.
	 * See cds_ft_group_attr_set_ordered_list.
	 */
	bool ordered_list_set;
	enum cds_ft_numa_policy numa_policy;	/* See cds_ft_group_attr_set_numa_policy. */
	enum cds_ft_optimize optimize;		/* See cds_ft_group_attr_set_optimize. */
};

struct cds_ft_attr {
	bool exclusive;
};

enum cds_ft_type_class {
	FT_POPCOUNT = 0,	/* Popcount-bitmap: popcount_1l / popcount_2l */
	FT_PIGEON = 1,		/* Pigeon: direct indexed */
	/* Leaf nodes are implicit from their height in the trie */
	FT_NR_TYPES = 2,

	FT_NULL,	/* not an encoded type, but keeps code regular */
};

#define ft_type_is_popcount(tc)	((tc) == FT_POPCOUNT)
#define ft_type_is_pigeon(tc)	((tc) == FT_PIGEON)

struct cds_ft_type {
	enum cds_ft_type_class type_class;
	uint16_t min_child;		/* minimum number of children: 1 to 256 */
	uint16_t max_child;		/* maximum number of children: 1 to 256 */
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
 * tunes max_child accordingly.
 *
 * Layout naming: scan_<root_bits>_<sub_bm_bits>(_max_<N>).  The
 * trailing _max_<N> appears only on the generic-2L variants
 * (scan_16_16_*) where the ptr offset depends on the static
 * sub_bm[] tail length (max_child).  The flat-layout scanners
 * (scan_32_8 with 5+3 byte split and u8 nibbles; scan_64_4 with
 * 6+2 byte split and u4 nibbles) pack root_bm + packed_bms into a
 * fixed 12 B or 16 B header, so the ptr offset is constant (+16
 * after alignment) and one function serves multiple max_child values.
 *
 * 32-bit max_child tuning: the flat scanners (scan_32_8, scan_64_4)
 * are layout-only -- headers do not grow with max_child and packed_bms
 * supports up to 64 distinct (hi, lo) pairs.  On 32-bit ptrs,
 * scan_64_4 hosts max_child=12 in 64 B and max_child=16 in 128 B (the
 * latter is bitmap-bound; trailing 48 B is layout slack).  The
 * layout helpers (header_bytes, get_nr_child, get_ith_pos,
 * set_nth) accept all canonical max_child values; the dispatcher
 * routes by max_child.
 *
 * scan_16_16_max_3 stays at max_child=3 on both arches: the 8 B header
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
	 *                            (was scan_64_4 max_child=16 with 48 B slack;
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

static const struct cds_ft_type ft_types[] = {
	[0] = {
		.type_class = FT_POPCOUNT,
		.popcount_2l = true,
		.min_child = 1,
		.max_child = ft_type_0_max_child, .order = 5, .bitmap = FT_NO_BITMAP,
	},
	[1] = {
		.type_class = FT_POPCOUNT,
		.popcount_2l = true,
		.min_child = 3,
		.max_child = ft_type_1_max_child, .order = 6, .bitmap = FT_NO_BITMAP,
	},
	[2] = {
		/* 32 B bitmap + 24 x 4 B ptrs = 128 B (order-7). */
		.type_class = FT_POPCOUNT,
		.popcount_1l = true,
		.min_child = 7,
		.max_child = ft_type_2_max_child, .order = 7, .bitmap = FT_NO_BITMAP,
	},

	/*
	 * Indices 2, 3, and 4 use popcount_1l (32 B bitmap + 4 B ptr
	 * table fills the node exactly).
	 */
	[3] = {
		/* 32 B bitmap + 56 * 4 B ptrs = 256 B (order-8). */
		.type_class = FT_POPCOUNT,
		.popcount_1l = true,
		.min_child = 10,
		.max_child = ft_type_3_max_child, .order = 8, .bitmap = FT_NO_BITMAP },
	[4] = {
		/* 32 B bitmap + 120 * 4 B ptrs = 512 B (order-9). */
		.type_class = FT_POPCOUNT,
		.popcount_1l = true,
		.min_child = 28,
		.bitmap = FT_NO_BITMAP,
		.max_child = ft_type_4_max_child, .order = 9 },

	/*
	 * Pigeon shrinks to the popcount_1l type directly below it once a
	 * removal brings it down to min_child.  That target's max_child is
	 * >= pigeon's min_child, and popcount_1l capacity is purely
	 * count-bound (its 256-bit bitmap addresses any byte value, and the
	 * pointer table is sized to max_child), so the shrink target always
	 * has room regardless of key distribution: the recompaction cannot
	 * fail for capacity and never rolls back to pigeon.
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
	ft_type_1_max_child = 6,	/* scan_32_8 (per-slot 5+3) */
	ft_type_2_max_child = 14,	/* scan_64_4 (flat 6+2) */
	ft_type_3_max_child = 28,
	ft_type_4_max_child = 60,
	ft_type_5_max_child = 124,
	ft_type_6_max_child = 256,
	ft_type_7_max_child = 256,
};

/*
 * scan_32_8 (per-slot 5+3 byte split, max_child=6): 12-byte popcount
 * header (4B root + 8B packed sub_bms) + 6 x 8-byte pointers into
 * the 64B order-6 node.
 *
 * scan_64_4 (flat 6+2 byte split, max_child=14): 16-byte popcount
 * header (8B root + 8B packed_bms with 14 x 4-bit sub_bms =
 * 56 bits used) + 14 x 8-byte pointers into the 128B order-7
 * node (16 + 14 * 8 = 128 exactly).
 */

static const struct cds_ft_type ft_types[] = {
	[0] = {
		.type_class = FT_POPCOUNT,
		.popcount_2l = true,
		.min_child = 1, .max_child = ft_type_0_max_child, .order = 5, .bitmap = FT_NO_BITMAP,
	},
	[1] = {
		.type_class = FT_POPCOUNT,
		.popcount_2l = true,
		.min_child = 3, .max_child = ft_type_1_max_child, .order = 6, .bitmap = FT_NO_BITMAP,
	},
	[2] = {
		.type_class = FT_POPCOUNT,
		.popcount_2l = true,
		.min_child = 5, .max_child = ft_type_2_max_child, .order = 7, .bitmap = FT_NO_BITMAP,
	},
	[3] = {
		.type_class = FT_POPCOUNT,
		.popcount_1l = true,
		.min_child = 10, .max_child = ft_type_3_max_child, .order = 8, .bitmap = FT_NO_BITMAP,
	},

	/*
	 * Indices 4 and 5 use popcount_1l (32B bitmap + 8B ptr
	 * table fills the node).
	 */
	[4] = {
		/* 32B bitmap + 60 * 8B ptrs = 512B order-9. */
		.type_class = FT_POPCOUNT,
		.popcount_1l = true,
		.min_child = 22,
		.max_child = ft_type_4_max_child,
		.order = 9, .bitmap = FT_NO_BITMAP },
	[5] = {
		/* 32B bitmap + 124 * 8B ptrs = 1024B order-10. */
		.type_class = FT_POPCOUNT,
		.popcount_1l = true,
		.min_child = 51,
		.max_child = ft_type_5_max_child,
		.order = 10, .bitmap = FT_NO_BITMAP },

	/*
	 * Pigeon shrinks to the popcount_1l type directly below it once a
	 * removal brings it down to min_child.  That target's max_child is
	 * >= pigeon's min_child, and popcount_1l capacity is purely
	 * count-bound (its 256-bit bitmap addresses any byte value, and the
	 * pointer table is sized to max_child), so the shrink target always
	 * has room regardless of key distribution: the recompaction cannot
	 * fail for capacity and never rolls back to pigeon.
	 */
	[6] = { .type_class = FT_PIGEON, .min_child = 95, .max_child = ft_type_6_max_child, .order = 11, .bitmap = FT_BITMAP },

	[7] = { .type_class = FT_NULL, .min_child = 0, .max_child = ft_type_7_max_child, .bitmap = FT_NO_BITMAP },
};
#endif /* !(BITS_PER_LONG < 64) */

/*
 * The cds_ft_inode contains the compressed node data needed for
 * the read-side traversal. Because the actual layout depends on the
 * node's type_class (Popcount or Pigeon), the struct uses a single
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
 * Writer-side helpers for the cds_ft_node.next removal tombstone (low
 * bit, see CDS_FT_NODE_REMOVED_FLAG).  These run under the writer mutex
 * (or RCU read lock on the chain-walk side), so a plain masked load is
 * sufficient; readers use cds_ft_node_next_rcu() instead.
 *
 *   ft_node_next        masked successor (the actual chain link)
 *   ft_node_is_removed  has @node been removed from the trie?
 *   ft_node_mark_removed set the tombstone, preserving the successor
 *                        pointer (relaxed store; readers mask the bit
 *                        and the pointer value is unchanged).
 */
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

static inline
void ft_node_mark_removed(struct cds_ft_node *node)
{
	CMM_STORE_SHARED(node->next, (struct cds_ft_node *)
			((uintptr_t) node->next | CDS_FT_NODE_REMOVED_FLAG));
}

/*
 * Mark every node in a duplicate chain as removed (used by
 * cds_ft_remove_all, which detaches a whole chain at once).  The
 * successor pointers stay intact so the caller can still traverse the
 * returned chain to reclaim it.
 */
static inline
void ft_chain_mark_removed(struct cds_ft_node *head)
{
	while (head) {
		struct cds_ft_node *next = ft_node_next(head);

		ft_node_mark_removed(head);
		head = next;
	}
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
	 * iter->node->prev — the one leaf touch per walk entry).  No stale cell is
	 * dereferenced: validity is a node-pointer compare, not a cell read.
	 */
	struct ft_ord_cell *ord_cell;
	struct cds_ft_node *ord_cell_node;

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
 * ft_metadata_set_external_nodes: Phase 1 — set the cluster-internal
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
 * ft_publish_external_nodes_prev: Phase 2 — publish the back-channel from
 * the displaced/transferred external head up to its (re-)parent node.
 * Defined below, after the ordinal-cell accessors it depends on in a cell
 * build (the head's prev is its cell, so the parent is recorded into
 * cell->parent rather than overwriting prev).
 */
static inline
void ft_publish_external_nodes_prev(struct cds_ft *ft,
		struct cds_ft_inode_flag *node_flag,
		struct cds_ft_node *external_nodes);

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

/*
 * Flip-proxy encoding (see src/urcu-flip-latch.h).  cds_ft_merge_at needs
 * to switch a whole set of back-pointers (and the merge-point forward
 * slot) from their old to their new target with no mixed-regime window.
 * Each such slot transiently holds a tagged pointer to a urcu_flip_proxy
 * latch; a single urcu_flip_commit store flips them all atomically.
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
#define FT_FLIP_PROXY_TYPE	7U
#define FT_FLIP_PROXY_TAG	(FT_INTERNAL_MASK | (FT_FLIP_PROXY_TYPE << FT_INTERNAL_BITS))

static inline_lookup
bool ft_node_flip_proxy(struct cds_ft_inode_flag *node)
{
	return ((unsigned long) node & (FT_INTERNAL_MASK | FT_TYPE_MASK))
		== FT_FLIP_PROXY_TAG;
}

static inline_lookup
struct urcu_flip_proxy *ft_flip_proxy_ptr(struct cds_ft_inode_flag *node)
{
	return (struct urcu_flip_proxy *) _ft_node_mask_ptr(node);
}

static
struct cds_ft_inode_flag *ft_flip_proxy_flag(struct urcu_flip_proxy *proxy)
{
	return (struct cds_ft_inode_flag *)
		((unsigned long) proxy | FT_FLIP_PROXY_TAG);
}

/*
 * Resolve a possibly-proxied flag to its current target.  Sits right
 * after a parent / root pointer load on the read side; the common case
 * (no merge in flight) is a single predicted-not-taken mask-compare, and
 * the proxy deref is reached only during a merge's brief flip window.
 */
static inline_lookup
struct cds_ft_inode_flag *ft_resolve_flip_proxy(struct cds_ft_inode_flag *node)
{
	if (caa_unlikely(ft_node_flip_proxy(node)))
		return (struct cds_ft_inode_flag *)
			urcu_flip_proxy_get(ft_flip_proxy_ptr(node));
	return node;
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

static inline_lookup
void *ft_ord_cell_flag(struct ft_ord_cell *cell)
{
	return (void *) ((unsigned long) cell | FT_ORD_CELL_TAG);
}

static inline_lookup
struct ft_ord_cell *ft_ord_cell_ptr(const void *prev)
{
	return (struct ft_ord_cell *)
		((unsigned long) prev & ~(unsigned long) FT_ORD_CELL_TAG);
}

/*
 * Resolve a head's flagged parent from its (already-rcu_dereference'd) prev.
 * When the group runs an ordinal-cell list, prev is a cell and the parent is
 * rcu_dereference(cell->parent); otherwise prev IS the flagged parent (no cell
 * interposed).  @ft selects the mode (the runtime cell-optional gate is added
 * later; for now the cell branch is unconditional in cell builds).  Valid only
 * for a head; a non-head dup's prev is the preceding node (callers gate on
 * ft_node_external like before).
 */
static inline_lookup
struct cds_ft_inode_flag *ft_resolve_head_prev(const struct cds_ft *ft, void *prev)
{
	if (ft->ordered_list)
		return rcu_dereference(ft_ord_cell_ptr(prev)->parent);
	return (struct cds_ft_inode_flag *) prev;
}

/*
 * Read an ordinal-cell ord_next / ord_prev slot, resolving an in-flight
 * flip-proxy.  The slots hold RAW (untagged) ft_ord_cell pointers, but a
 * point-op splice transiently installs a tagged flip-proxy (the same type-7
 * encoding as ft_resolve_flip_proxy) so the two directional edges flip
 * atomically for a bidirectional ordered reader.  Raw cells are >= 8-byte
 * aligned (bits 0-1 clear, like an external node) and proxies carry the
 * type-7 tag, so the proxy test is unambiguous.  Under writer exclusion no
 * proxy is installed at rest (a no-op on the write side).
 */
static inline_lookup
struct ft_ord_cell *ft_ord_cell_resolve_ord(struct ft_ord_cell *const *slot)
{
	struct ft_ord_cell *p = rcu_dereference(*slot);

	if (caa_unlikely(ft_node_flip_proxy((struct cds_ft_inode_flag *) p)))
		p = (struct ft_ord_cell *) urcu_flip_proxy_get(
			ft_flip_proxy_ptr((struct cds_ft_inode_flag *) p));
	return p;
}

/*
 * Cell lifecycle (FT-allocator arena).
 *
 * The cell is an item of the group's dedicated cell arena (a uniform 32 B
 * item region, FT_ORD_CELL_ALLOC_ORDER), so cells pack contiguously for the
 * dense ord-walk and are RELOCATABLE by cds_ft_compact.  The paired metadata
 * slot is unused except its rcu_head, which cds_ft_free_item reuses to defer
 * the free past a grace period; cds_ft_metadata_to_item / _item_to_metadata
 * map between the cell and its metadata via the range header.
 */

/*
 * Allocate a head's cell and wire it to @node with parent @parent (which
 * may be NULL — a root head — or set later via ft_ord_cell_set_parent).
 * The ord_prev / ord_next list links start empty; the ordered-list splice
 * (runtime-gated by ordered_list_set) populates them later.  Returns the
 * cell-tagged pointer to store into node->prev, or NULL on allocation
 * failure (the caller fails the insert before mutating the trie).
 */
static
void *ft_ord_cell_alloc(struct cds_ft *ft, struct cds_ft_node *node,
		struct cds_ft_inode_flag *parent)
{
	struct cds_ft_metadata *meta = cds_ft_alloc_cell_item(ft);
	struct ft_ord_cell *cell;

	if (!meta)
		return NULL;
	cell = (struct ft_ord_cell *) cds_ft_metadata_to_item(meta);
	cell->ord_prev = NULL;
	cell->ord_next = NULL;
	cell->node = node;
	cell->parent = parent;
	if (ft_debug_counters())
		uatomic_inc(&ft->group->nr_cells_allocated);
	return ft_ord_cell_flag(cell);
}

/*
 * Release a head's cell after its key leaves the trie.  Routes through
 * cds_ft_free_item, which defers the free past a grace period (concurrent
 * mode) so an in-flight reader parked on a head it up-walks (head->prev ->
 * cell -> cell->parent) never dereferences freed memory, frees synchronously
 * in exclusive mode, and drains the cell range's nr_live for reclaim.
 */
static
void ft_ord_cell_free(struct cds_ft *ft, struct ft_ord_cell *cell)
{
	if (ft_debug_counters())
		uatomic_inc(&ft->group->nr_cells_freed);
	cds_ft_free_item(ft, cds_ft_item_to_metadata(cell));
}

/*
 * Immediate free for a cell that was never published (an insert that ended
 * a duplicate or failed before @node became reachable): no reader can hold a
 * reference, so the grace-period defer would only delay the free.
 */
static inline
void ft_ord_cell_free_unpublished(struct cds_ft *ft, struct ft_ord_cell *cell)
{
	if (ft_debug_counters())
		uatomic_inc(&ft->group->nr_cells_freed);
	cds_ft_free_item_unpublished(ft, cds_ft_item_to_metadata(cell));
}

/*
 * Set a head's relocated parent: in the cell-always model head->prev is the
 * cell (set once at alloc) and the flagged parent lives in cell->parent.
 * rcu_assign_pointer for the read-side up-walk (ft_resolve_head_prev does
 * rcu_dereference(cell->parent)); @head must already carry its cell.
 */
static inline
void ft_ord_cell_set_parent(struct cds_ft_node *head,
		struct cds_ft_inode_flag *parent)
{
	rcu_assign_pointer(ft_ord_cell_ptr(head->prev)->parent, parent);
}

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
	void *prev = node->prev;

	if (ft_node_external((struct cds_ft_inode_flag *) prev))
		return (struct cds_ft_inode_flag *) prev;
	return ft_resolve_head_prev(ft, prev);
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
 * ft_publish_external_nodes_prev: Phase 2 — publish the back-channel pointer
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
 * prev — the cell pointer — is unchanged).  All choke-point callers
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
	return (struct cds_ft_compressed_node *)
		(((unsigned long) node) & ~(unsigned long) FT_TAG_MASK);
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
			rcu_dereference(((struct cds_ft_node *) node)->prev));
	else if (ft_node_compressed(node))
		/*
		 * A compressed node's metadata lives at a FT_TAG_MASK-cleared
		 * offset, not the FT_TYPE_MASK-cleared one ft_node_ptr uses; a
		 * referenced compressed child re-parented by a merge reaches
		 * here (after the caller resolves any skip form to its raw
		 * compressed flag).
		 */
		parent = rcu_dereference(cds_ft_item_to_metadata(
			(struct cds_ft_inode *) ft_compressed_node_ptr(node))->parent);
	else
		parent = rcu_dereference(cds_ft_item_to_metadata(
			ft_node_ptr(node))->parent);
	/*
	 * The parent slot may transiently hold a flip-proxy during a
	 * cds_ft_merge_at commit; resolve it to the view-appropriate
	 * (old or merged) parent before returning.
	 */
	parent = ft_resolve_flip_proxy(parent);
	assert(!parent || !ft_node_external(parent));
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
			rcu_dereference(((struct cds_ft_node *) child)->prev));
	else
		parent = rcu_dereference(cds_ft_item_to_metadata(
			ft_node_ptr(child))->parent);
	return ft_compressed_node_ptr(parent);
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
 * @rewind:   out — bytes the caller must back its descent cursor/level up by.
 * @at_pos:   out (may be NULL) — the live node spanning/at the encoded position
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
				rcu_dereference(((struct cds_ft_node *) cur)->prev));
		else
			parent = rcu_dereference(cds_ft_item_to_metadata(
				ft_node_ptr(cur))->parent);
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
			holder = ft_resolve_flip_proxy(rcu_dereference(
				cds_ft_item_to_metadata(
				(struct cds_ft_inode *) pitem)->parent));
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
 * When parent is NULL (root's child), the offset is unused —
 * ft_get_parent_slot recovers &ft->root.
 */
/* Recover the branch byte indexing @slot in internal parent @node (defined
 * after the popcount layout helpers). */
static uint8_t ft_slot_to_byte(const struct cds_ft_type *type,
		struct cds_ft_inode *node, struct cds_ft_inode_flag **slot);

static inline
void ft_set_parent_slot(struct cds_ft_metadata *meta,
		struct cds_ft_inode_flag **slot)
{
	struct cds_ft_inode_flag *p;
	bool parent_compressed;

	if (!slot)
		return;	/* Slot unknown — preserve existing offset. */
	if (!meta->parent) {
		meta->parent_slot_offset = 0;
		return;
	}
	meta->parent_slot_offset = (unsigned int)((char *) slot -
		(char *) ft_node_ptr(meta->parent)) / sizeof(void *);
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
	p = meta->parent;
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
 * sibling page), so offset 0 is a valid slot — the node's first/lowest
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
static inline
struct cds_ft_inode_flag **ft_get_parent_slot(const struct cds_ft_metadata *meta,
		struct cds_ft *ft)
{
	if (!meta->parent)
		return &ft->root;
	return (struct cds_ft_inode_flag **)
		((char *) ft_node_ptr(meta->parent) +
		 (unsigned int) meta->parent_slot_offset * sizeof(void *));
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
 * maintains the underlying compressed node's parent_slot_offset so it
 * records @parent_slot's offset in @parent_nf — required by
 * ft_get_parent_slot lookups (the dual-pointer dance above, and the
 * chain-merge canonicalization in ft_detach_node that publishes a
 * replacement at the cn's same grandparent slot).  Without this,
 * compressed nodes installed via ft_publish_to_parent rather than via
 * ft_node_set_nth → ft_set_parent leave parent_slot_offset == 0 — a
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
	/*
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
#ifndef NDEBUG
	if (new_child && parent_slot != &ft->root) {
		struct cds_ft_inode_flag *cp;

		/*
		 * Check skip-compressed FIRST: a SKIP_X flag carries its
		 * (external) child's low tag bits, so ft_node_external() would
		 * misclassify it and dereference the tagged flag as a node.
		 */
#ifdef FEATURE_FT_SKIP_COMPRESSED
		if (ft_node_skip_compressed(new_child)) {
			cp = cds_ft_item_to_metadata((struct cds_ft_inode *)
				ft_skip_to_compressed(ft, new_child))->parent;
		} else
#endif
		if (ft_node_external(new_child)) {
			/*
			 * Cell-always: prev is the (non-NULL) cell pointer even
			 * when the parent is unset, so resolve through the cell to
			 * preserve the forward-before-parent check on cell->parent.
			 */
			cp = ft_resolve_head_prev(ft,
				((struct cds_ft_node *) new_child)->prev);
		} else {
			cp = cds_ft_item_to_metadata(
				ft_node_ptr(new_child))->parent;
		}
		assert(cp != NULL);
	}
#endif /* !NDEBUG */
	/*
	 * Maintain @new_child's parent-slot offset (parent_slot_offset) so
	 * that it records the slot holding it within its parent node.  This
	 * is the value ft_get_parent_slot(child_meta) recovers later — used by
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
	 * through ft_set_parent) leave parent_slot_offset == 0 — a latent gap
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
		struct cds_ft_metadata *child_meta = NULL;

		if (ft_node_skip_compressed(new_child))
			child_meta = cds_ft_item_to_metadata(
				(struct cds_ft_inode *)
					ft_skip_to_compressed(ft, new_child));
		else if (!ft_node_external(new_child))
			child_meta = cds_ft_item_to_metadata(
				ft_node_ptr(new_child));
		if (child_meta && child_meta->parent)
			ft_set_parent_slot(child_meta, parent_slot);
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
			struct cds_ft_inode_flag **skip_slot =
				ft_get_parent_slot(cn_meta, ft);
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
	(void) ft;
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
		rcu_assign_pointer(cn_meta->parent, parent_nf);
		ft_set_parent_slot(cn_meta, slot);
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
		rcu_assign_pointer(cn_meta->parent, parent_nf);
		ft_set_parent_slot(cn_meta, slot);
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
		rcu_assign_pointer(meta->parent, parent_nf);
		ft_set_parent_slot(meta, slot);
	}
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

static inline_lookup
uint8_t *align_ptr_size(uint8_t *ptr)
{
	return (uint8_t *) FT_ALIGN((unsigned long) ptr, sizeof(void *));
}

/*
 * Forward declarations for the 2-level popcount-bitmap node helpers.
 * Definitions follow further down (after the ft_dereference_acquire_*
 * macros they depend on).
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
 * (a single 256-bit occupancy bitmap, no second-level summary).
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
static inline void ft_maybe_prefetch(const void *ptr)
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
static inline void ft_maybe_prefetch_nta(const void *ptr)
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
 * external_nodes.  Now a plain rcu_dereference -- external children are
 * deliberately NOT prefetched (see ft_maybe_prefetch: random/use-once
 * leaves drop the prefetch on a 4 KiB TLB miss or flood the memory
 * controller under 2 MiB).  Kept as a distinct name to mark leaf loads.
 */
#define ft_dereference_prefetch(p)		\
	({							\
		__typeof__(p) __ft_tmp = rcu_dereference(p);	\
		ft_maybe_prefetch(__ft_tmp);			\
		__ft_tmp;					\
	})

#define ft_dereference_external(p)	rcu_dereference(p)

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
 * Root-slot dereference for read-side descents: like
 * ft_dereference_*(ft->root) but additionally resolves a flip-proxy that
 * a cds_ft_merge_at commit may transiently install at the merge-point
 * forward slot (here, the root).  The resolved flag then flows into the
 * cached path, the key_len==0 / longest-match metadata access, and the
 * child dispatch.  The common case (no merge in flight) is a single
 * predicted-not-taken mask-compare in ft_resolve_flip_proxy.
 */
#define ft_root_dereference_prefetch(ft)				\
	ft_resolve_flip_proxy(ft_dereference_prefetch((ft)->root))
#define ft_root_dereference_acquire_prefetch(ft)			\
	ft_resolve_flip_proxy(ft_dereference_acquire_prefetch((ft)->root))
#define ft_root_dereference(ft)						\
	ft_resolve_flip_proxy(rcu_dereference((ft)->root))

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
 */
enum ft_pf_target {
	FT_PF_NONE,
	FT_PF_DATA,
};

static inline __attribute__((always_inline))
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
 * The order in which values and pointers are does does not matter: if
 * a value is missing, we return NULL. If a value is there, but its
 * associated pointers is still NULL, we return NULL too.
 */


/*
 * Runtime validation of ft_types[] popcount_1l class membership assumed
 * by the per-type dispatcher.  Called from cds_ft_group_create at init.
 * Not a static assert because ft_types[].max_child is a struct
 * member read, which gcc doesn't treat as an integer constant
 * expression.
 */
static inline __attribute__((unused))
void ft_specialized_scan_layout_assert(void)
{
#if CAA_BITS_PER_LONG >= 64
	assert(FT_ALIGN(ft_types[0].max_child, sizeof(void *)) == 8);
	assert(FT_ALIGN(ft_types[1].max_child, sizeof(void *)) == 8);
	assert(FT_ALIGN(ft_types[2].max_child, sizeof(void *)) == 16);
	assert(FT_ALIGN(ft_types[3].max_child, sizeof(void *)) == 32);
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
 * Used by the scan_16_16_max_<N> family (max_child=3 and max_child=5).
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
 * For order-5 nodes (32 B, max_child = 3):
 *   header = 8 B (root_bm + 3 sub_bm)
 *   ptrs   = 24 B (3 child pointers)
 *
 * scan_16_16_max_3 reads the header as one u64 load and decomposes
 * via fixed bit-shifts: low 16 bits = root_bm, upper 48 bits =
 * sub_bm[0..2] in popcount order with sub_bm[0] at bits 16..31.  The
 * rank `popcount(subs & ((1ULL << bit_pos) - 1))` thus requires
 * sub_bm[0] in the low bits of the loaded u64.  In memory that maps
 * to first-to-last byte order on little-endian and last-to-first on
 * big-endian, so the writer flips the in-memory offsets accordingly
 * (see ft_popcount_2l_root_bm_addr / ft_popcount_2l_sub_bm_addr).
 * max_child=5 uses per-u16 reads (scan_16_16_max_5) and keeps the
 * canonical "root_bm at offset 0, sub_bm[k] at 2+2k" layout on both
 * arches.
 *
 * The lookup is branch-free past the two presence tests and uses
 * portable __builtin_popcount{,ll} -- no SIMD intrinsics, so the
 * same code compiles for any architecture with a popcount intrinsic
 * (including software-emulated popcount on older targets).
 */
struct ft_popcount_2l_header {
	uint16_t root_bm;
	uint16_t sub_bm[];	/* length = type->max_child */
};

/*
 * Endian-aware addressing of the root_bm and sub_bm[k] u16 slots in
 * a 2-level popcount header.  See the layout comment above for why
 * max_child=3 reverses on big-endian.
 */
static inline_lookup
uint16_t *ft_popcount_2l_root_bm_addr(struct cds_ft_inode *node,
		unsigned int max_lc)
{
	unsigned int offset = 0;

#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
	if (max_lc == 3)
		offset = 2U * max_lc;	/* root_bm follows reversed sub_bm[] */
#endif
	(void) max_lc;
	return (uint16_t *) &node->data[offset];
}

static inline_lookup
uint16_t *ft_popcount_2l_sub_bm_addr(struct cds_ft_inode *node,
		unsigned int max_lc, unsigned int k)
{
	unsigned int offset = 2U + 2U * k;

#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
	if (max_lc == 3)
		offset = 2U * (max_lc - 1U - k);	/* sub_bm[0] highest */
#endif
	(void) max_lc;
	return (uint16_t *) &node->data[offset];
}

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
	unsigned int max_lc = type->max_child;
	unsigned int byte_offset;

	/*
	 * Flat scan_32_8 (12 B hdr + 4 B pad) and scan_64_4 (16 B hdr)
	 * scanners hardcode the ptr table at node+16.  max_child=12 and 16
	 * reuse the scan_64_4 layout on 32-bit -- the 16-slot bitmap
	 * caps the safe distinct-hi count at 16 (cf. scan_32_8's 8-slot
	 * cap, which is too tight for 12+ children in the worst case).
	 * max_child=16 lives in a 128 B order-7 node with 48 B trailing
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
 * Lookup primitive: 2-level popcount, max_child = 3.
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
 * Lookup primitive: 2-level popcount, max_child = 5 (32-bit only).
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
 * Trade-off vs scan_16_16_max_3 (max_child=3): one variable-length
 * prefix loop in exchange for max_child 3 -> 5 (recovers the 12 B of
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
 * max_child = 6.
 *
 * Layout in the 64-byte order-6 node (FLAT layout, NOT shared with
 * scan_16_16_max_3 which uses the 4+4 sub_bm[] layout):
 *   [0..3]   root_bm (32-bit; bit set per populated hi = n >> 3)
 *   [4..11]  packed_bms (8 bytes = 8 x 8-bit sub_bm; only first 6 used)
 *   [12..15] padding
 *   [16..63] 6 x cds_ft_inode_flag *
 *
 * The byte split is 5-bit hi + 3-bit lo (departs from the quadbit-popcount
 * (QP) 4+4 convention to fit all sub_bms in one u64).  packed_bms is a flat
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
 * bitmap suffices because max_child=6 fits in 48 bits (well within u64).
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
 * max_child = 14.
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
 * For max_child = 3, the three sub bitmaps fit in 6 bytes and
 * a single qword load lets us read them all alongside root_bm.
 */
static inline_lookup
uint8_t ft_popcount_2l_node_get_nr_child(const struct cds_ft_type *type,
		struct cds_ft_inode *node)
{
	unsigned int max_lc = type->max_child;

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
		unsigned int total = 0, k;

		for (k = 0; k < max_lc; k++)
			total += (unsigned int) __builtin_popcount(
					*ft_popcount_2l_sub_bm_addr(node, max_lc, k));
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
	unsigned int max_lc = type->max_child;
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
		uint16_t root = *ft_popcount_2l_root_bm_addr(node, max_lc);
		unsigned int target = i, k, hi, lo;
		uint16_t sub = 0, root_walk;
		unsigned int sub_pop = 0;

		for (k = 0; k < max_lc; k++) {
			sub = *ft_popcount_2l_sub_bm_addr(node, max_lc, k);
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
 * Bit-scan core for the 2-level layouts.  Their sub-bitmaps are
 * concatenated in popcount order: bit (slot1*G + lo) is set iff lo-value
 * @lo is present under the slot1-th populated hi-group, where G =
 * 1 << @lo_bits and slot1 = popcount(root & ((1<<hi)-1)).  The concat is
 * passed split as @subs_lo (bits 0..63) and @subs_hi (bits 64..127); the
 * three u64-packed layouts (scan_16_16_max_3, scan_32_8, scan_64_4) fit in
 * @subs_lo alone (@subs_hi = 0), while scan_16_16_max_5 (32-bit; 5x16 = 80
 * bits) carries its top group in @subs_hi.
 *
 * Find the nearest set bit toward @dir from byte @n and return its byte
 * value (hi << lo_bits | lo, with hi recovered by selecting the slot1-th
 * set bit of @root), or -1 if there is none in that direction.  The scan
 * itself is the shared cds_find_{next,prev}_bit primitive (the same one
 * popcount_1l / pigeon use) over the concat materialized as an unsigned
 * long array; it naturally spans the >64-bit max_5 concat and bounds the
 * scan at @total = nr_groups*G, so a transient stray bit in an unused slot
 * (>= total) during a concurrent insert shift is ignored.
 *
 * @n is the EXCLUSIVE byte bound and may be the get_minmax sentinels (-1
 * for FT_RIGHT/leftmost, FT_ENTRY_PER_NODE for FT_LEFT/rightmost), which
 * are NOT valid bytes -- decode hi/lo only for an in-range byte.  The
 * resulting inclusive start bit maps 1:1 onto the primitive's start_bit
 * (which also handles the out-of-range start that the sentinels produce).
 */
static inline_lookup
int ft_popcount_2l_dir_byte(uint64_t root, uint64_t subs_lo, uint64_t subs_hi,
		unsigned int lo_bits, unsigned int nr_groups,
		int n, enum ft_direction dir)
{
	unsigned int lo_mask = (1U << lo_bits) - 1U;
	unsigned int total = nr_groups << lo_bits;	/* <= 80 */
	unsigned int slot1p, lop, hip, j;
	uint64_t r;
	int start, p;
#if (CAA_BITS_PER_LONG >= 64)
	unsigned long bm[2] = { (unsigned long) subs_lo, (unsigned long) subs_hi };
#else
	unsigned long bm[4] = {
		(unsigned long) subs_lo, (unsigned long) (subs_lo >> 32),
		(unsigned long) subs_hi, (unsigned long) (subs_hi >> 32),
	};
#endif

	if (total == 0)
		return -1;
	if (dir == FT_RIGHT) {
		if (n < 0) {
			start = 0;				/* find first (smallest byte) */
		} else if (n >= 255) {
			return -1;				/* no byte > 255 */
		} else {
			unsigned int hi = (unsigned int) n >> lo_bits;
			unsigned int lo = (unsigned int) n & lo_mask;
			unsigned int slot1 = (unsigned int) __builtin_popcountll(
					root & (((uint64_t) 1 << hi) - 1U));

			start = ((root >> hi) & 1ULL) ?
				(int) ((slot1 << lo_bits) + lo + 1U) : (int) (slot1 << lo_bits);
		}
		p = cds_find_next_bit(bm, total, start);
	} else {
		if (n > 255) {
			start = (int) total - 1;		/* find last (largest byte) */
		} else if (n <= 0) {
			return -1;				/* no byte < 0 */
		} else {
			unsigned int hi = (unsigned int) n >> lo_bits;
			unsigned int lo = (unsigned int) n & lo_mask;
			unsigned int slot1 = (unsigned int) __builtin_popcountll(
					root & (((uint64_t) 1 << hi) - 1U));

			start = (((root >> hi) & 1ULL) ?
				(int) ((slot1 << lo_bits) + lo) : (int) (slot1 << lo_bits)) - 1;
		}
		p = cds_find_prev_bit(bm, total, start);
	}
	if (p < 0)
		return -1;
	/* p -> (slot1p, lop); recover hi by selecting the slot1p-th set bit. */
	slot1p = (unsigned int) p >> lo_bits;
	lop = (unsigned int) p & lo_mask;
	r = root;
	for (j = 0; j < slot1p; j++)
		r &= r - 1;		/* clear the lowest set bit */
	hip = (unsigned int) __builtin_ctzll(r);
	return (int) ((hip << lo_bits) | lop);
}

/*
 * Find the leftmost (FT_LEFT: largest v < n) or rightmost (FT_RIGHT:
 * smallest v > n) populated entry.
 *
 * Every 2-level layout bit-scans its concatenated sub-bitmap toward @dir
 * with ft_popcount_2l_dir_byte (which delegates the scan to the shared
 * cds_find_{next,prev}_bit, the same primitive popcount_1l / pigeon use):
 * the sub_bms are stored contiguously in popcount order, so one scan
 * crosses hi-groups and a single select on root_bm recovers the hi nibble.
 * The byte is mapped to its pointer by the layout's scan primitive; the
 * bitmap is only a hint, so a soft-deleted entry (bit set, pointer NULL)
 * makes the scan continue past it.
 */
static inline_lookup
struct cds_ft_inode_flag *ft_popcount_2l_node_get_direction(
		const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		int n, uint8_t *result_key,
		enum ft_direction dir)
{
	unsigned int max_lc = type->max_child;
	uint64_t root, subs_lo, subs_hi = 0;
	unsigned int lo_bits, nr_groups;

	assert(dir == FT_LEFT || dir == FT_RIGHT);

	if (max_lc == 6) {			/* scan_32_8: 5+3 */
		root = *(const uint32_t *) &node->data[0];
		subs_lo = *(const uint64_t *) &node->data[4];
		lo_bits = 3;
	} else if (max_lc == 3) {		/* scan_16_16_max_3: 4+4 packed */
		uint64_t bms = *(const uint64_t *) &node->data[0];

		root = (uint16_t) bms;
		subs_lo = bms >> 16;
		lo_bits = 4;
#if (CAA_BITS_PER_LONG < 64)
	} else if (max_lc == 5) {		/* scan_16_16_max_5: 5 x u16, 80-bit concat */
		const uint16_t *s = (const uint16_t *) &node->data[2];

		root = *(const uint16_t *) &node->data[0];
		subs_lo = (uint64_t) s[0] | ((uint64_t) s[1] << 16)
			| ((uint64_t) s[2] << 32) | ((uint64_t) s[3] << 48);
		subs_hi = s[4];		/* top group -> bits 64..79 */
		lo_bits = 4;
#endif
	} else {				/* scan_64_4: 6+2 (max 12/14/16) */
		root = *(const uint64_t *) &node->data[0];
		subs_lo = *(const uint64_t *) &node->data[8];
		lo_bits = 2;
	}
	nr_groups = (unsigned int) __builtin_popcountll(root);
	cmm_smp_rmb();	/* read bitmaps before pointers */
	for (;;) {
		int byte = ft_popcount_2l_dir_byte(root, subs_lo, subs_hi,
				lo_bits, nr_groups, n, dir);
		struct cds_ft_inode_flag *ptr;
		struct cds_ft_inode_flag **slot = NULL;

		if (byte < 0)
			return NULL;
		ptr = (max_lc == 6) ?
			ft_popcount_2l_scan_32_8(node, &slot,
				(uint8_t) byte, FT_PF_NONE) :
		      (max_lc == 3) ?
			ft_popcount_2l_scan_16_16_max_3(node, &slot,
				(uint8_t) byte, FT_PF_NONE) :
#if (CAA_BITS_PER_LONG < 64)
		      (max_lc == 5) ?
			ft_popcount_2l_scan_16_16_max_5(node, &slot,
				(uint8_t) byte, FT_PF_NONE) :
#endif
			ft_popcount_2l_scan_64_4(node, &slot,
				(uint8_t) byte, FT_PF_NONE);
		if (ptr) {
			/*
			 * Spatial prefetch: the child pointers are a dense
			 * byte-ordered array, so the adjacent sibling in
			 * iteration order is slot[+1] (FT_RIGHT / next) or
			 * slot[-1] (FT_LEFT / prev) -- a near-future
			 * cds_ft_next/prev target.
			 *
			 * No bounds check: the one-element over/under-read cannot
			 * fault, by allocator construction.
			 *   - slot[+1] past the last pointer of a full node lands,
			 *     in the worst case (node at the end of the 2 MiB
			 *     node-body run), in the FT_FAR_METADATA metadata array
			 *     the allocator places immediately after that run;
			 *     within a node it is just an unused allocated slot.
			 *   - slot[-1] before pointers[0] lands in the node's
			 *     bitmap header, which precedes the pointer array.
			 * Either way the load hits allocated memory; the word read
			 * is unrelated to a real child pointer, but ft_maybe_prefetch
			 * (__builtin_prefetch) silently drops NULL / non-canonical
			 * addresses without faulting.
			 */
			ft_maybe_prefetch_nta(dir == FT_RIGHT ? slot[1] : slot[-1]);
			*result_key = (uint8_t) byte;
			return ptr;
		}
		n = byte;	/* soft-deleted: scan past this bit */
	}
}

/*
 * Insert (n, child_node_flag) into a freshly-allocated, unpublished
 * popcount_2l node.  Called only from the recompact path: the new node is
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
	unsigned int max_lc = type->max_child;

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
	unsigned int hi = (unsigned int) n >> 4;
	unsigned int lo = (unsigned int) n & 0xFU;
	uint16_t root, sub;
	unsigned int slot1, ptr_idx, nr_child, k;

	if (is_init) {
		memset(&node->data[0], 0, ft_popcount_2l_header_bytes(max_lc));
		*ft_popcount_2l_root_bm_addr(node, max_lc) = (uint16_t) (1U << hi);
		*ft_popcount_2l_sub_bm_addr(node, max_lc, 0) = (uint16_t) (1U << lo);
		pointers[0] = child_node_flag;
		metadata->nr_child++;
		return 0;
	}

	root = *ft_popcount_2l_root_bm_addr(node, max_lc);
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
			*ft_popcount_2l_sub_bm_addr(node, max_lc, k) =
				*ft_popcount_2l_sub_bm_addr(node, max_lc, k - 1);
		*ft_popcount_2l_sub_bm_addr(node, max_lc, slot1) = 0;
		*ft_popcount_2l_root_bm_addr(node, max_lc) =
			(uint16_t) (root | (1U << hi));
	}
	sub = *ft_popcount_2l_sub_bm_addr(node, max_lc, slot1);
	assert(!((sub >> lo) & 1U));	/* duplicate key would be a bug */

	/* Compute pointer insertion index across full layout. */
	{
		uint64_t subs_below = 0;
		for (k = 0; k < slot1; k++)
			subs_below += (uint64_t) __builtin_popcount(
				*ft_popcount_2l_sub_bm_addr(node, max_lc, k));
		subs_below += (uint64_t) __builtin_popcount(
				(unsigned int) sub & ((1U << lo) - 1U));
		ptr_idx = (unsigned int) subs_below;
	}

	/* Shift pointers[ptr_idx..nr_child) up. */
	for (k = nr_child; k > ptr_idx; k--)
		pointers[k] = pointers[k - 1];
	pointers[ptr_idx] = child_node_flag;

	*ft_popcount_2l_sub_bm_addr(node, max_lc, slot1) =
		(uint16_t) (sub | (1U << lo));
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
 * Recover the branch byte indexing @slot within internal parent @node (the
 * inverse of byte -> child slot).  Pigeon: the body is a 256-entry array
 * indexed by byte, so the slot offset IS the byte.  Popcount: children are
 * packed by set-bit rank, so the slot's index in the packed pointer array
 * (slot - pointers_base) is its rank, mapped to the byte via get_ith_pos.
 * @node is always an internal (pigeon/popcount) node -- ft_set_parent_slot
 * guards out compressed parents (which have no slot array).
 */
static
uint8_t ft_slot_to_byte(const struct cds_ft_type *type,
		struct cds_ft_inode *node, struct cds_ft_inode_flag **slot)
{
	if (ft_type_is_pigeon(type->type_class))
		return (uint8_t) (slot -
			(struct cds_ft_inode_flag **) node->data);
	{
		struct cds_ft_inode_flag **base = type->popcount_2l ?
			ft_popcount_2l_pointers(node, type) :
			ft_popcount_1l_pointers(node);
		uint8_t v;
		struct cds_ft_inode_flag *child;

		ft_popcount_node_get_ith_pos(type, node,
			(uint8_t) (slot - base), &v, &child);
		return v;
	}
}

/*
 * Find the leftmost (largest v < n) or rightmost (smallest v > n)
 * populated entry.
 *
 * Scan the inline 256-bit occupancy bitmap for the nearest set bit toward
 * @dir with cds_find_prev_bit / cds_find_next_bit (as
 * ft_pigeon_node_get_direction does), then map that byte value to its
 * compact pointer via ft_popcount_1l_scan_28.  The bitmap is only a hint:
 * a soft-deleted entry keeps its bit set but holds a NULL pointer, so on
 * NULL continue the scan past that bit.
 */
static inline_lookup
struct cds_ft_inode_flag *ft_popcount_1l_node_get_direction(
		const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		int n, uint8_t *result_key,
		enum ft_direction dir)
{
	assert(dir == FT_LEFT || dir == FT_RIGHT);

	{
		struct ft_popcount_1l_header *hdr =
			(struct ft_popcount_1l_header *) &node->data[0];
		unsigned long *bm = (unsigned long *) hdr->bm;
		int i;

		(void) type;	/* inline bitmap: the vtable type arg is unused here */
retry:
		if (dir == FT_LEFT)
			i = cds_find_prev_bit(bm, FT_ENTRY_PER_NODE, n - 1);
		else
			i = cds_find_next_bit(bm, FT_ENTRY_PER_NODE, n + 1);
		if (i < 0)
			return NULL;
		{
			struct cds_ft_inode_flag **slot = NULL;
			struct cds_ft_inode_flag *ptr =
				ft_popcount_1l_scan_28(node, &slot,
					(uint8_t) i, FT_PF_NONE);

			if (!ptr) {
				/*
				 * Soft-deleted: the bit is still set but the
				 * pointer load (source of truth) is NULL.
				 * Continue the bitmap scan past this bit.
				 */
				n = i;
				goto retry;
			}
			/*
			 * Spatial prefetch (dense byte-ordered pointers): the
			 * adjacent sibling is slot[+1] (FT_RIGHT) / slot[-1]
			 * (FT_LEFT) -- a near-future cds_ft_next/prev target.
			 * The unchecked slot[+-1] over/under-read is fault-free by
			 * the same allocator construction documented in
			 * ft_popcount_2l_node_get_direction (FT_FAR_METADATA after
			 * the 2 MiB run for +1; bitmap header before pointers
			 * for -1).
			 */
			ft_maybe_prefetch_nta(dir == FT_RIGHT ? slot[1] : slot[-1]);
			*result_key = (uint8_t) i;
			return ptr;
		}
	}
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
	assert(nr_child < type->max_child);
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
	struct cds_ft_bitmap *bitmap = cds_ft_item_to_bitmap(node, type->order);
	int i;

	assert(ft_type_is_pigeon(type->type_class));
	assert(dir == FT_LEFT || dir == FT_RIGHT);

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
		/*
		 * Spatial prefetch: pigeon slots are byte-indexed (sparse), so
		 * unlike the dense popcount layouts the adjacent present child is
		 * NOT slot[+-1] -- one more bitmap scan in the iteration direction
		 * locates it, and we prefetch it as a near-future cds_ft_next/prev
		 * target.  The extra scan is cheap against a far-metadata/body miss.
		 */
		{
			int adj = (dir == FT_RIGHT) ?
				cds_find_next_bit(bitmap->bitmap,
					FT_ENTRY_PER_NODE, i + 1) :
				cds_find_prev_bit(bitmap->bitmap,
					FT_ENTRY_PER_NODE, i - 1);

			if (adj >= 0)
				ft_maybe_prefetch_nta(((struct cds_ft_inode_flag **)
					node->data)[adj]);
		}
		dbg_printf("ft_pigeon_node_get child_node_flag %p\n", child_node_flag);
		*result_key = (uint8_t) i;
		return child_node_flag;
	}
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
 *   0:  popcount_2l_scan_16_16_max_3 (32 B,  max_child=3)
 *   1:  popcount_2l_scan_32_8        (64 B,  max_child=6)
 *   2:  popcount_2l_scan_64_4        (128 B, max_child=14)
 *   3:  popcount_1l_scan_28          (256 B, max_child=28)
 *   4:  popcount_1l_scan_28          (512 B, max_child=60)
 *   5:  popcount_1l_scan_28         (1024 B, max_child=124)
 *   6:  pigeon                      (2048 B)
 *
 * 32-bit type-index -> scanner:
 *   0:  popcount_2l_scan_16_16_max_5 (32 B,  max_child=5)
 *   1:  popcount_2l_scan_64_4        (64 B,  max_child=12)
 *   2:  popcount_1l_scan_28          (128 B, max_child=24)
 *   3:  popcount_1l_scan_28          (256 B, max_child=56)
 *   4:  popcount_1l_scan_28          (512 B, max_child=120)
 *   5:  pigeon                      (1024 B)
 *
 * The popcount scanners are pure integer math (popcount + bitmap
 * indexing); no SSE / SIMD dependency.  The hot caa_likely branch
 * targets the most-frequent type observed across the comprehensive
 * benchmark (popcount_2l max_child=3 on 64-bit: ~46% of dispatches).
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
 * ft_node_get_nth: child slot access with skip-compressed resolution
 * for writer / debug callers.
 *
 * Resolves any skip pointer via the slot value directly (no validation,
 * no concurrency handling).  All RCU reader paths use
 * ft_node_get_nth_reanchor instead, which validates the skip pointer
 * against the live compressed node and re-anchors via the child's
 * parent chain on a mismatch (the single concurrency mechanism — see
 * ft_skip_reanchor).  The historical validate=true spin loop here is
 * therefore gone; the @validate_lookup parameter is dropped.
 */
static inline_lookup
struct cds_ft_inode_flag *ft_node_get_nth(const struct cds_ft *ft, struct cds_ft_inode_flag *node_flag,
		struct cds_ft_inode_flag ***node_flag_ptr,
		uint8_t n, enum ft_pf_target pf_hint)
{
	struct cds_ft_inode_flag *child =
		ft_node_get_nth_skip(node_flag, node_flag_ptr, n, pf_hint);

	return ft_resolve_skip_compressed(ft, child);
}

/*
 * Surgical reader descent step: fetch @node_flag's child at byte @n, resolving
 * a skip-compressed slot mismatch via the live skip-child parent chain instead
 * of spinning on a (possibly frozen) slot like ft_node_get_nth's validate path.
 * On a mismatch ft_skip_reanchor locates the live position; *@rewind_ret is how
 * many byte-depths the resolved node lies ABOVE the dispatched child (0 in the
 * common split / recompaction case; > 0 only when a concurrent chain-merge
 * moved the encoded position shallower -- the caller backs its descent cursor
 * up by that much, or re-descends).  ft_skip_reanchor never returns NULL on a
 * well-formed trie (the writer wires every fresh cluster's parent before the
 * cluster becomes reachable, so the up-walk never observes a NULL parent);
 * the result is asserted non-NULL.  Returns the resolved child flag
 * (compressed/internal/external), or NULL for an empty slot.
 */
static inline_lookup
struct cds_ft_inode_flag *ft_node_get_nth_reanchor(struct cds_ft *ft,
		struct cds_ft_inode_flag *node_flag, uint8_t n,
		unsigned int *rewind_ret)
{
	struct cds_ft_inode_flag *child =
		ft_node_get_nth_skip(node_flag, NULL, n, FT_PF_NONE);

	(void) ft;
	*rewind_ret = 0;
	/*
	 * Resolve a type-7 flip proxy transiently occupying the slot (a
	 * cds_ft_merge_at flip, or an ordered-list insert's one-commit
	 * publish) BEFORE the skip handler classifies the child: an
	 * unresolved proxy's tag bits read as internal type 7 and would
	 * dereference the proxy latch as node memory.  Predicted-not-taken
	 * when no flip is in flight; the resolved value may itself be
	 * skip-encoded, hence resolve-then-skip order.
	 */
	child = ft_resolve_flip_proxy(child);
#ifdef FEATURE_FT_SKIP_COMPRESSED
	if (caa_unlikely(child && ft_node_skip_compressed(child))) {
		struct cds_ft_inode_flag *at_pos, *anchor;

		anchor = ft_skip_reanchor(ft, child, rewind_ret, &at_pos);
		assert(anchor != NULL);
		return at_pos;
	}
#endif
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
 * Only handles internal node types (popcount, pigeon).
 * Compressed parents are handled separately by callers.
 * Write-side only (mutex-held).
 */
static
bool ft_node_find_child(struct cds_ft *ft, struct cds_ft_inode_flag *parent_nf,
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
					ft_node_get_nth(ft, parent_nf, slot_ret, v, FT_PF_NONE);
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
					ft_node_get_nth(ft, parent_nf, slot_ret, i, FT_PF_NONE);
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
 * against the slot's skip_len.  On mismatch the RAW (still skip-compressed)
 * pointer is returned as a re-anchor signal; the reader-level caller resolves
 * it via ft_skip_reanchor (a validated result is never skip-compressed, so the
 * signal is unambiguous).  RCU readers must pass true and MUST handle the
 * skip-compressed return.  Writers must pass false (no validation: a writer
 * reads its own in-flight state and is the race partner, not a victim of it).
 * With static inlining and a constant arg, the unused branch is DCE'd at each
 * call site.
 */
static inline_lookup
struct cds_ft_inode_flag *ft_node_get_direction(struct cds_ft *ft, struct cds_ft_inode_flag *node_flag,
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
		/*
		 * Resolve a type-7 flip proxy transiently occupying a slot (a
		 * cds_ft_merge_at flip, or a one-commit insert's parked
		 * publish), so every ordered traversal reaching a directional
		 * child through this accessor (iteration via
		 * ft_node_get_leftright / ft_node_get_minmax and the relational
		 * lookups) sees the view-appropriate old-or-new child.
		 * Predicted-not-taken when no flip is in flight; NULL and
		 * skip-encoded children pass through unchanged (a skip child's
		 * low nibble is its tag, never the 0xF proxy).
		 *
		 * A parked one-commit insert proxy resolves to NULL (fresh key,
		 * empty old slot): that slot is INVISIBLE, not end-of-scan --
		 * continue the directional scan past it, else a rank walk
		 * (lookup_nth / nth_last) or going-up sibling scan would
		 * silently drop every child beyond the parked slot.
		 */
		if (caa_unlikely(ft_node_flip_proxy(child))) {
			child = ft_resolve_flip_proxy(child);
			if (!child) {
				/*
				 * Skip the parked slot: directional scans pivot
				 * past it; a minmax entry continues INWARD from
				 * it as a pivoted scan.
				 */
				if (dir == FT_LEFTMOST)
					dir = FT_RIGHT;
				else if (dir == FT_RIGHTMOST)
					dir = FT_LEFT;
				n = *result_key;
				continue;
			}
		}
		break;
	}
	if (!validate_lookup)
		return ft_resolve_skip_compressed(ft, child);
	/*
	 * Precise (validating) lookup: return a skip-compressed child RAW as a
	 * re-anchor signal.  It is still skip-compressed, which a resolved
	 * result never is, so the reader-level caller detects it and resolves
	 * via ft_skip_reanchor (the single skip concurrency mechanism).  We must
	 * NOT resolve it here by assuming the skip child's back-pointer is a
	 * compressed node: mid-split (e.g. a suffix_len==0 branch re-parenting
	 * the old external child before the slot is republished) it can
	 * transiently be an INTERNAL node, and reading that as a compressed
	 * len/key_bytes yields a garbage key.
	 */
	return child;
}

static inline_lookup
struct cds_ft_inode_flag *ft_node_get_leftright(struct cds_ft *ft, struct cds_ft_inode_flag *node_flag,
		unsigned int n, uint8_t *result_key,
		enum ft_direction dir, bool validate_lookup)
{
	return ft_node_get_direction(ft, node_flag, n, result_key, dir, validate_lookup);
}

static inline_lookup
struct cds_ft_inode_flag *ft_node_get_minmax(struct cds_ft *ft, struct cds_ft_inode_flag *node_flag,
		uint8_t *result_key,
		enum ft_direction dir, bool validate_lookup)
{
	switch (dir) {
	case FT_LEFTMOST:
		return ft_node_get_direction(ft, node_flag,
				-1, result_key, FT_RIGHT, validate_lookup);
	case FT_RIGHTMOST:
		return ft_node_get_direction(ft, node_flag,
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
/* Forward decl: ft_set_parent_raw is defined later (merge area). */
static void ft_set_parent_raw(struct cds_ft *ft, struct cds_ft_inode_flag *child,
		struct cds_ft_inode_flag *value);

static
int ft_popcount_node_set_nth(struct cds_ft *ft, const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		struct cds_ft_inode_flag *node_flag,
		struct cds_ft_metadata *metadata,
		uint8_t n,
		struct cds_ft_inode_flag *child_node_flag,
		bool *_replace_old_ptr,
		bool is_init,
		bool defer_parent)
{
	assert(ft_type_is_popcount(type->type_class));

	/*
	 * Parent-first: wire the (fresh, not-yet-published) child's parent
	 * back-pointer before any in-place store below, so a reader that
	 * descends to it and walks back up never observes a NULL/stale parent.
	 * Skipped when @defer_parent: a recompact child-copy (new node is
	 * unpublished, reparented post-assembly) or a build-invisible cluster-
	 * leaf (mutator wires parents at publish).  skip_slot is still set by
	 * the wrapper's post-store ft_set_parent.
	 */
	if (!defer_parent)
		ft_set_parent_raw(ft, child_node_flag, node_flag);

	if (type->popcount_2l && type->max_child == 6) {
		/* Flat 5+3 layout (max_child=6 on 64-bit). */
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
			if (qp_ptr_idx >= type->max_child)
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
		if (qp_ptr_idx >= type->max_child)
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
	if (type->popcount_2l && (type->max_child == 12
				|| type->max_child == 14
				|| type->max_child == 16)) {
		/* Flat 6+2 layout (max_child=14 on 64-bit, 12/16 on 32-bit). */
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
			if (qp_ptr_idx >= type->max_child)
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
		if (qp_ptr_idx >= type->max_child)
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
		struct cds_ft_inode_flag **qp_pointers;
		unsigned int qp_max_lc = type->max_child;
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
		qp_pointers = ft_popcount_2l_pointers(node, type);
		qp_root = *ft_popcount_2l_root_bm_addr(node, qp_max_lc);
		qp_hi = (unsigned int) n >> 4;
		qp_lo = (unsigned int) n & 0xFU;
		qp_slot1 = (unsigned int) __builtin_popcount(
				qp_root & ((1U << qp_hi) - 1U));

		if ((qp_root >> qp_hi) & 1U) {
			qp_sub = *ft_popcount_2l_sub_bm_addr(node,
					qp_max_lc, qp_slot1);
			if ((qp_sub >> qp_lo) & 1U) {
				/* Case 1: key already present, in-place replace. */
				uint64_t subs_below = 0;
				unsigned int k;

				for (k = 0; k < qp_slot1; k++)
					subs_below += (uint64_t)
						__builtin_popcount(
							*ft_popcount_2l_sub_bm_addr(
								node, qp_max_lc, k));
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
			if (qp_ptr_idx >= type->max_child)
				return -ENOSPC;
			rcu_assign_pointer(qp_pointers[qp_ptr_idx],
					child_node_flag);
			uatomic_store(ft_popcount_2l_sub_bm_addr(node,
					qp_max_lc, qp_slot1),
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
		if (qp_ptr_idx >= type->max_child)
			return -ENOSPC;
		uatomic_store(ft_popcount_2l_sub_bm_addr(node,
				qp_max_lc, qp_slot1),
				(uint16_t) (1U << qp_lo), CMM_RELAXED);
		rcu_assign_pointer(qp_pointers[qp_ptr_idx], child_node_flag);
		uatomic_store(ft_popcount_2l_root_bm_addr(node, qp_max_lc),
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
		if (ptr_idx >= type->max_child)
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
int ft_pigeon_node_set_nth(struct cds_ft *ft, const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		struct cds_ft_inode_flag *node_flag,
		struct cds_ft_metadata *metadata,
		uint8_t n,
		struct cds_ft_inode_flag *child_node_flag,
		bool defer_parent)
{
	struct cds_ft_inode_flag **ptr;
	bool replace_old_ptr = false;

	assert(ft_type_is_pigeon(type->type_class));
	/* Parent-first (see ft_popcount_node_set_nth). */
	if (!defer_parent)
		ft_set_parent_raw(ft, child_node_flag, node_flag);
	ptr = &((struct cds_ft_inode_flag **) node->data)[n];
	if (*ptr)
		replace_old_ptr = true;
	rcu_assign_pointer(*ptr, child_node_flag);
	if (!replace_old_ptr) {
		struct cds_ft_bitmap *bitmap = cds_ft_item_to_bitmap(node, type->order);

		/* Set n in bitmap. */
		cds_set_bit_relaxed(bitmap->bitmap, n);
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
int _ft_node_set_nth(struct cds_ft *ft, const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		struct cds_ft_inode_flag *node_flag,
		struct cds_ft_metadata *metadata,
		uint8_t n,
		struct cds_ft_inode_flag *child_node_flag,
		bool is_init,
		bool defer_parent)
{
	int ret;

	switch (type->type_class) {
	case FT_POPCOUNT:
		ret = ft_popcount_node_set_nth(ft, type, node, node_flag, metadata, n, child_node_flag, NULL, is_init, defer_parent);
		break;
	case FT_PIGEON:
		ret = ft_pigeon_node_set_nth(ft, type, node, node_flag, metadata, n, child_node_flag, defer_parent);
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
int ft_popcount_node_replace_ptr(struct cds_ft *ft, const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		struct cds_ft_inode_flag *node_flag,
		struct cds_ft_metadata *metadata,
		struct cds_ft_inode_flag **node_flag_ptr,
		struct cds_ft_inode_flag *newptr)
{
	assert(ft_type_is_popcount(type->type_class));
	assert(ft_popcount_node_get_nr_child(type, node) <= type->max_child);

	if (!newptr) {
		if (metadata->nr_child <= type->min_child) {
			/* We need to try recompacting the node */
			return -EFBIG;
		}
	}
	dbg_printf("popcount replace ptr: node %p\n", node);
	assert(*node_flag_ptr != NULL);
	/*
	 * Parent-first: wire the replacement's back-pointer before the
	 * forward publish (a reader descending here then walking back up must
	 * not see a NULL/stale parent).  Placed past the -EFBIG check: the
	 * recompaction path re-parents via its rebuilt copy itself, so setting
	 * it here would re-parent through a copy recompaction frees.  (NULL
	 * newptr == delete: ft_set_parent is a no-op.)
	 */
	ft_set_parent(ft, newptr, node_flag, node_flag_ptr);
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
int ft_pigeon_node_replace_ptr(struct cds_ft *ft, const struct cds_ft_type *type,
		struct cds_ft_inode *node __attribute__((unused)),
		struct cds_ft_inode_flag *node_flag,
		struct cds_ft_metadata *metadata,
		struct cds_ft_inode_flag **node_flag_ptr,
		uint8_t n __attribute__((unused)),
		struct cds_ft_inode_flag *newptr)
{
	assert(ft_type_is_pigeon(type->type_class));

	if (!newptr) {
		/* We should try recompacting the node */
		if (metadata->nr_child <= type->min_child)
			return -EFBIG;
	}
	dbg_printf("ft_pigeon_node_replace_ptr: replace ptr: %p by %p\n", *node_flag_ptr, newptr);
	assert(*node_flag_ptr != NULL);
	/* Parent-first: wire the back-pointer before the forward publish,
	 * past the -EFBIG recompaction check (see popcount variant). */
	ft_set_parent(ft, newptr, node_flag, node_flag_ptr);
	rcu_assign_pointer(*node_flag_ptr, newptr);
	if (!newptr) {
		struct cds_ft_bitmap *bitmap = cds_ft_item_to_bitmap(node, type->order);

		/* Clear n in bitmap. */
		cds_clear_bit_relaxed(bitmap->bitmap, n);
		metadata->nr_child--;
	}
	return 0;
}

/*
 * _ft_node_replace_ptr: replace ptr item within a node. Return an error
 * (negative error value) if it is not found (-ENOENT).
 */
static
int _ft_node_replace_ptr(struct cds_ft *ft, const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		struct cds_ft_inode_flag *node_flag,
		struct cds_ft_metadata *metadata,
		struct cds_ft_inode_flag **node_flag_ptr,
		uint8_t n, struct cds_ft_inode_flag *newptr)
{
	int ret;

	switch (type->type_class) {
	case FT_POPCOUNT:
		ret = ft_popcount_node_replace_ptr(ft, type, node, node_flag, metadata, node_flag_ptr, newptr);
		break;
	case FT_PIGEON:
		ret = ft_pigeon_node_replace_ptr(ft, type, node, node_flag, metadata, node_flag_ptr, n, newptr);
		break;
	case FT_NULL:
		return -ENOENT;
	default:
		assert(0);
		return -EINVAL;
	}
	/* Parent back-pointer is now wired inside the per-class body, ahead of
	 * the forward store (previously set here, after it). */
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
 *
 * @cluster_leaf: when true, the target node sits at the lower boundary of an
 * as-yet-unpublished cluster (a cluster-leaf): its children point at live
 * nodes from the old structure, and the mutator wires every one of those
 * back-pointers itself at publish time.  Children are still copied into the
 * new node's slots, but the re-parent loop is skipped entirely.  See
 * ft_node_set_nth and the rcu-mutation build-invisible pattern.
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
		unsigned int node_depth __attribute__((unused)),
		bool cluster_leaf)
{
	unsigned int new_type_index;
	struct cds_ft_inode *new_node;
	struct cds_ft_metadata *new_metadata;
	const struct cds_ft_type *new_type;
	struct cds_ft_inode_flag *new_node_flag = NULL;
	int ret;
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
	case FT_RECOMPACT_RELOCATE:
		new_type_index = old_type_index;	/* same type, pure relocation */
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
			/* The retyped node keeps its own incoming edge byte. */
			new_metadata->incoming_byte = metadata->incoming_byte;
#ifdef FEATURE_FT_SKIP_COMPRESSED
			new_metadata->parent_slot_offset = metadata->parent_slot_offset;
#endif
			/*
			 * Recompact: new_metadata->parent is already inherited
			 * above, so the back-channel prev = new_node_flag is
			 * safe to publish here (up-walkers reach a parent-wired
			 * node).  Split into the Phase-1 metadata write + the
			 * Phase-2 prev publish for consistency with the other
			 * external-nodes attach sites.
			 */
			ft_metadata_set_external_nodes(new_node_flag,
				new_metadata, metadata->external_nodes);
			ft_publish_external_nodes_prev(ft, new_node_flag,
				metadata->external_nodes);
			ft_nr_keys_store(new_metadata,
				ft_nr_keys_get(metadata), CMM_RELAXED);
		}
	} else {
		new_node = NULL;
		new_node_flag = NULL;
	}

	assert(mode != FT_RECOMPACT_ADD_NEXT || old_type->type_class != FT_PIGEON);

	/*
	 * A DEL must never prune the node to NODE_INDEX_NULL: the remove
	 * paths route a node emptying its last child through ft_detach_node
	 * (which unlinks the whole branch) instead of a DEL recompact, so
	 * @new_node below is non-NULL whenever the copy/parent-inherit tail
	 * dereferences it.  That invariant is enforced several call layers
	 * away -- catch a regression here, at the dereference site.
	 */
	assert(mode != FT_RECOMPACT_DEL || new_type_index != NODE_INDEX_NULL);

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
			ret = _ft_node_set_nth(ft, new_type, new_node, new_node_flag,
					new_metadata, v, iter,
					RECOMPACT_IS_INIT(v), true);
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

		assert(mode == FT_RECOMPACT_DEL ||
			mode == FT_RECOMPACT_RELOCATE);
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
			ret = _ft_node_set_nth(ft, new_type, new_node, new_node_flag,
					new_metadata, i, iter,
					RECOMPACT_IS_INIT((uint8_t)i), true);
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
		ret = _ft_node_set_nth(ft, new_type, new_node, new_node_flag,
				new_metadata, n, child_node_flag,
				RECOMPACT_IS_INIT(n), true);
		assert(!ret);
	}

#undef RECOMPACT_IS_INIT

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
		/*
		 * The recompacted node replaces the old node at the SAME slot
		 * in the SAME parent, so its parent-slot offset is identical.
		 * Inherit it on every build (the offset is no longer
		 * skip-specific — it backs the parent-pointer backtrack's O(1)
		 * slot recovery for plain-internal nodes too).
		 */
		new_metadata->parent_slot_offset = old_meta->parent_slot_offset;

#ifdef FEATURE_FT_SKIP_COMPRESSED
		if (old_parent && ft_node_compressed(old_parent)) {
			struct cds_ft_compressed_node *cn =
				ft_compressed_node_ptr(old_parent);
			struct cds_ft_metadata *cn_meta =
				cds_ft_item_to_metadata(
					(struct cds_ft_inode *) cn);
			struct cds_ft_inode_flag **skip_slot =
				ft_get_parent_slot(cn_meta, ft);

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
	 *
	 * Skip this entirely for a cluster-leaf node: it sits at the
	 * lower boundary of an as-yet-unpublished cluster, its children
	 * point at live nodes, and the mutator wires every one of those
	 * back-pointers itself at publish time.  Re-parenting any of them
	 * here would expose the unpublished cluster from below.
	 */
	if (!cluster_leaf) {
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
				ft_set_parent(ft, iter, new_node_flag, slot);
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
				ft_set_parent(ft, iter, new_node_flag, slot);
			}
			break;
		}
		default:
			break;
		}
	}

	FT_TP(node_recompact, (const void *) *old_node_flag_ptr,
		(const void *) new_node_flag, (int) new_type_index);

	/*
	 * Return the new recompacted node through old_node_flag_ptr.  For the
	 * ADD/SAME/DEL mutators this is a local out-param, re-published with
	 * rcu_assign_pointer by the caller; but ft_compact_relocate_at passes
	 * the LIVE slot (&ft->root, a parent child slot, &cn->child) and never
	 * re-publishes, so this store IS the reader-visible publication of the
	 * relocated node: use a release store so the node-body and metadata
	 * stores above are ordered before it (a plain store would let a
	 * weakly-ordered architecture expose an unwired copy).  Free for the
	 * local-out-param callers.
	 */
	rcu_assign_pointer(*old_node_flag_ptr, new_node_flag);
	if (old_node && old_node_ret)
		*old_node_ret = old_node;

	ret = 0;
end:
	return ret;
}

/*
 * Return 0 on success or negative error value on error.
 *
 * @cluster_leaf: when true, the target node is a cluster-leaf — the lower
 * boundary of an as-yet-unpublished cluster (rcu-mutation build-invisible
 * pattern).  Its children are live nodes also still reachable through the old
 * structure, so writing their back-pointers to this unpublished node would
 * expose the cluster from below and, on a later allocation failure, leave a
 * dangling back-pointer.  The forward slot is still set, but NO child's
 * back-pointer is written here (neither in-place nor through recompaction);
 * the mutator wires every child of the node itself at publish time, using the
 * final node flag it tracks across recompactions.  Callers building such a
 * node must pass true for ALL of its set_nth calls (no per-child exception).
 * false for the ordinary published-node case.
 */
static
int ft_node_set_nth(struct cds_ft *ft,
		struct cds_ft_inode_flag **node_flag, uint8_t n,
		struct cds_ft_inode_flag *child_node_flag,
		struct cds_ft_inode **old_node_ret,
		struct cds_ft_metadata *metadata,
		unsigned int node_depth,
		bool cluster_leaf)
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
	ret = _ft_node_set_nth(ft, type, node, *node_flag, metadata, n,
			child_node_flag, false, cluster_leaf);
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
		 * use it to record parent_slot_offset, which chain-merge
		 * canonicalization (ft_detach_node) recovers via
		 * ft_get_parent_slot to publish the replacement at the same
		 * slot.  Without it, plain-COMPRESSED children (the path
		 * taken when CDS_FT_FLAG_SKIP_COMPRESSED is unset on the
		 * group) leave parent_slot_offset == 0 — a latent gap that
		 * trips chain-merge with parent-cn=plain-COMPRESSED.
		 */
		struct cds_ft_inode_flag **slot_ptr = NULL;

		if (cluster_leaf)
			break;	/* child back-pointers set by mutator at publish */
		/*
		 * Fetch the parent's child slot for every non-external child
		 * (internal, compressed, or skip): ft_set_parent records the
		 * slot offset so the parent-pointer backtrack can recover it.
		 * (Externals carry no metadata / offset.)  Test skip FIRST: a
		 * SKIP_X flag carries its external child's low tag bits, so
		 * ft_node_external() would misclassify it.
		 */
		if (ft_node_skip_compressed(child_node_flag) ||
		    !ft_node_external(child_node_flag))
			ft_node_get_nth_skip(*node_flag, &slot_ptr, n, FT_PF_NONE);
		ft_set_parent(ft, child_node_flag, *node_flag, slot_ptr);
		break;
	}
	case -ENOSPC:
		/* Not enough space in node, need to recompact to next type. */
		ret = ft_node_recompact(FT_RECOMPACT_ADD_NEXT, ft, type_index, type, node,
					metadata, node_flag, n, child_node_flag, NULL,
					old_node_ret, false, node_depth, cluster_leaf);
		break;
	case -ERANGE:
		/* Node needs to be recompacted. */
		ret = ft_node_recompact(FT_RECOMPACT_ADD_SAME, ft, type_index, type, node,
					metadata, node_flag, n, child_node_flag, NULL,
					old_node_ret, false, node_depth, cluster_leaf);
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
	ret = _ft_node_replace_ptr(ft, type, node, *parent_node_flag_ptr, metadata, node_flag_ptr, n, newptr);
	if (ret == -EFBIG) {
		assert(!newptr);
		/* Should try recompaction. */
		ret = ft_node_recompact(FT_RECOMPACT_DEL, ft, type_index, type, node,
				metadata, parent_node_flag_ptr, n, NULL,
				node_flag_ptr, old_node_ret, is_root, node_depth,
				false);
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
		bool track, bool track_longest,
		const uint8_t **match_key_pos_p, struct cds_ft_node **match_node_p,
		struct cds_ft_node **found_ret,
		enum cds_ft_status *status_ret,
		bool candidate)
{
	struct cds_ft_inode_flag *node_flag = *node_flag_p;
	const uint8_t *key = *key_p;
	struct cds_ft_compressed_node *cn = ft_compressed_node_ptr(node_flag);
	int remaining_key = (int) (key_end - key);
	int remaining_safe = (int) (key_safe_end - key);
	int cmp_len = cn->len < remaining_key ? cn->len : remaining_key;

	/* Check external_nodes at the compressed node's depth. */
	if (track) {
		struct cds_ft_metadata *cn_meta =
			cds_ft_item_to_metadata((struct cds_ft_inode *) cn);
		struct cds_ft_node *ext =
			ft_dereference_external(cn_meta->external_nodes);

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

		*found_ret = ft_dereference_external(cn_meta->external_nodes);
		*status_ret = *found_ret ? CDS_FT_STATUS_OK :
				CDS_FT_STATUS_NOT_FOUND;
		if (track && (*found_ret || track_longest)) {
			*match_key_pos_p = key;
			*match_node_p = *found_ret;
		}
		return FT_DESCENT_END;
	}

	/* Advance past the compressed path. */
	key += cn->len;
	node_flag = ft_dereference_acquire_prefetch(cn->child);
	assert(node_flag != NULL);	/* compressed node always has a live child (by construction) */

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
			ft_dereference_external(metadata->external_nodes);

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
	assert(cn->child != NULL);	/* compressed node always has a live child (by construction) */
	*node_flag_p = cn->child;
	if (node_flag_ptr_p)
		*node_flag_ptr_p = &cn->child;
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
 *   - loop-top skip-compressed resolution
 *   - get_nth dispatch
 *   - post-get_nth skip-compressed resolution
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

	node_flag = ft_root_dereference_prefetch(ft);

	{
	/*
	 * Whether this iter caches its position for continuation reuse
	 * (CACHED mode under a continuously-held RCU read lock).  Captured
	 * before spilling @iter so the terminal path_len computation need
	 * not rehydrate @iter.  No path array is populated during descent:
	 * the going-up backtrack recovers per-level nodes from the live
	 * parent chain, and path_len is derived from the consumed key
	 * length at the terminal.
	 */
	const bool cache_path =
		iter && iter->cache_mode == CDS_FT_ITER_CACHED;

	if (iter)
		iter_debug_path_snapshot(iter);
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
		found = ft_dereference_external(metadata->external_nodes);
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
		struct cds_ft_node *external_nodes = ft_dereference_external(metadata->external_nodes);

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
#ifdef FEATURE_FT_SKIP_COMPRESSED
			/*
			 * Mutators do not produce a skip-encoded root.  Resolve
			 * it defensively for completeness via the non-validating
			 * resolver: a root has no concurrent re-parent of its
			 * skip child, so the back-pointer is a stable compressed
			 * node (no mid-split internal-node transient as in the
			 * interior dispatch, which re-anchors instead).
			 */
			node_flag = ft_resolve_skip_compressed(ft, node_flag);
#endif
		} else {
			unsigned int skip = ft_skip_len(node_flag);

			if ((int) skip > (int) (key_end - key)) {
				status = CDS_FT_STATUS_NOT_FOUND;
				goto end;
			}
			key += skip;
			node_flag = ft_skip_child_ptr(node_flag);
		}
	}
	if (caa_unlikely(!ft_node_internal(node_flag))) {
		if (ft_node_compressed(node_flag)) {
			enum ft_descent_action act;

			act = ft_lookup_compressed(&node_flag, &key, key_end,
				key_safe_end,
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

#ifdef FEATURE_FT_SKIP_COMPRESSED
descend_loop:
#endif
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
			 *
			 * Precise-descent skip-encoded resolution: validate the
			 * skip slot against the live compressed node.  On a
			 * mismatch (a concurrent writer split/merge reparented the
			 * skip child, possibly while @parent_node_flag was
			 * recompacted away) re-anchor on the live structure via
			 * ft_skip_reanchor instead of spinning — a frozen slot may
			 * never republish.  Candidate descent doesn't need the cn
			 * (it advances via ft_skip_child_ptr below), so only the
			 * !descend_cand branch resolves/re-anchors here.
			 */
			struct cds_ft_inode_flag *parent_node_flag = node_flag;
			unsigned long _raw = (unsigned long) parent_node_flag;
			unsigned int _type =
				(unsigned int) ((_raw >> FT_INTERNAL_BITS) & 0x7);

			node_flag = ft_node_get_nth_skip_pretyped(parent_node_flag,
					_type, NULL, iter_key, FT_PF_DATA);
			if (!node_flag) {
				status = CDS_FT_STATUS_NOT_FOUND;
				goto end;
			}
			/*
			 * A cds_ft_merge_at flip transiently installs a type-7
			 * proxy in an interior merge-point slot; resolve it to the
			 * view-appropriate (old or merged) child before the skip /
			 * kind handlers classify it.  Predicted-not-taken when no
			 * merge is in flight (the proxy tag 0xF never matches an
			 * internal / external / compressed / skip child).
			 */
			node_flag = ft_resolve_flip_proxy(node_flag);
#ifdef FEATURE_FT_SKIP_COMPRESSED
			if (skip_compressed && !descend_cand
			    && caa_unlikely(ft_node_skip_compressed(node_flag))) {
				{
					/*
					 * Resolve the skip-compressed slot via the live
					 * skip-child parent chain (ft_skip_reanchor, the
					 * single skip concurrency mechanism).  Never assume
					 * the skip child's back-pointer is a compressed node
					 * and read its len/key_bytes directly: mid-split it
					 * can transiently be an INTERNAL node (e.g. a
					 * suffix_len==0 branch re-parenting the old external
					 * child before the slot's skip pointer is
					 * republished), which would resolve to a garbage key.
					 */
					unsigned int rewind;
					struct cds_ft_inode_flag *at_pos;
					struct cds_ft_inode_flag *anchor =
						ft_skip_reanchor(ft, node_flag, &rewind, &at_pos);

					assert(anchor != NULL);
					if (rewind == 0) {
						/*
						 * Split (or same-length replace): @at_pos is
						 * the live node at the failing slot's encoded
						 * depth — exactly what the validate-success
						 * path resolves @cn to.  Descend INTO it by
						 * falling through to the post-step handler with
						 * @key unchanged (identical to the
						 * cn != NULL case, just sourced from the live
						 * parent-chain walk).  This bypasses the
						 * holder's stale slot, so we never re-read a
						 * slot that may mismatch again: each re-anchor
						 * advances the descent one level — wait-free.
						 */
						node_flag = at_pos;
					} else {
						/*
						 * Merge overshoot: the failing level was
						 * absorbed into a longer compressed that begins
						 * ABOVE this depth, so @at_pos cannot be entered
						 * with the unchanged cursor.  Re-anchor at the
						 * holder and re-descend through the normal loop
						 * (path bookkeeping stays on the proven path;
						 * this rarer case remains lock-free).
						 */
						node_flag = anchor;
						/*
						 * The merged run begins within the
						 * span this descent already consumed:
						 * a rewind past @orig_key would mean
						 * the re-anchor walked above the
						 * search root -- impossible on a
						 * well-formed trie; catch a
						 * corruption-driven underflow here
						 * rather than reading before the
						 * caller's buffer.
						 */
						assert((size_t) (key - orig_key) >=
							(size_t) rewind + 1);
						key -= (size_t) rewind + 1;
						goto descend_loop;
					}
				}
			}
#endif
		}
		dbg_printf("cds_ft_lookup iter key lookup %u finds node_flag %p\n",
				(unsigned int) iter_key, node_flag);
#ifdef FEATURE_FT_SKIP_COMPRESSED
		/*
		 * Candidate-mode skip-advance for skip-encoded child slots.
		 * Precise-mode resolution / re-anchor happened in the
		 * dispatcher above.
		 *
		 * Tempting follow-up that does NOT work: adding a
		 * ft_maybe_prefetch(node_flag) here to prefetch the skip
		 * target's body (the 1-step-ahead FT_PF_DATA prefetch saw the
		 * raw skip pointer and dropped it as non-canonical; node_flag
		 * is cleared here so it would fire).  Measured a NET LOSS of
		 * ~10-13% on dns ft_specv at T1 AND T192 (interleaved A/B,
		 * 2026-05-24).  A prefetch on only ~3% of steps cannot cost
		 * that directly: adding the instruction perturbs the codegen /
		 * code layout of this always-inline descent template (which is
		 * iTLB/layout-sensitive) and regresses every step.  Besides,
		 * the lead time is tiny — a skip target is consumed almost
		 * immediately (validated, for a leaf) — so it could not hide
		 * the leaf's DRAM latency anyway.  Do not add a prefetch here.
		 */
		if (skip_compressed && descend_cand && caa_unlikely(ft_node_skip_compressed(node_flag))) {
			unsigned int skip = ft_skip_len(node_flag);
			int remaining = (int) (key_end - key);

			if ((int) skip > remaining) {
				status = CDS_FT_STATUS_NOT_FOUND;
				goto end;
			}
			key += skip;
			node_flag = ft_skip_child_ptr(node_flag);
		}
#endif
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

				act = ft_lookup_compressed(&node_flag, &key, key_end,
					key_safe_end,
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
				/* terminal external — fall through. */
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
			struct cds_ft_node *external_nodes = ft_dereference_external(metadata->external_nodes);

			if (external_nodes || track_longest) {
				match_key_pos = key;
				match_node = external_nodes;
			}
		}
	}

terminal:
	/*
	 * path_len is the number of path levels (root + one per consumed
	 * key byte) used by a CACHED iter's continuation fast path; derive
	 * it from the consumed key length.  Zero for UNCACHED / no-iter
	 * (unused there: UNCACHED clears cache_valid in the epilogue).
	 */
	if (cache_path)
		iter_path_len = (size_t) (key - orig_key) + 1;
	}

	/*
	 * Reached key_depth, check for terminal node: either external
	 * nodes or internal/compressed node associated with external nodes.
	 */
	if (ft_node_internal(node_flag)) {
		const struct cds_ft_type *type = &ft_types[ft_node_type(node_flag)];
		struct cds_ft_metadata *metadata = cds_ft_item_to_metadata_fast(ft_node_ptr(node_flag),
							type->order);
		found = ft_dereference_external(metadata->external_nodes);
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
		found = ft_dereference_external(metadata->external_nodes);
		status = found ? CDS_FT_STATUS_OK : CDS_FT_STATUS_NOT_FOUND;
		if (track && (found || track_longest)) {
			match_key_pos = key_end;
			match_node = found;
		}
	} else {
		found = (struct cds_ft_node *) node_flag;
		/*
		 * NULL also matches the external tag: pathological re-anchor
		 * exhaustion can deliver it here.  Report NOT_FOUND rather
		 * than OK with a NULL result node (an NDEBUG build has no
		 * assert left to catch the contradiction).
		 */
		status = found ? CDS_FT_STATUS_OK : CDS_FT_STATUS_NOT_FOUND;
		if (track && (found || track_longest)) {
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
		iter->cache_valid = (status == CDS_FT_STATUS_OK);
		iter_debug_path_update(iter);
		iter_auto_invalidate_cache(iter);
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
/* Limit-none relational specializations (defined after the impl below). */
static enum cds_ft_status ft_ineq_le_keycopy(struct cds_ft *, struct cds_ft_iter *);
static enum cds_ft_status ft_ineq_le_eager(struct cds_ft *, struct cds_ft_iter *);
static enum cds_ft_status ft_ineq_ge_keycopy(struct cds_ft *, struct cds_ft_iter *);
static enum cds_ft_status ft_ineq_ge_eager(struct cds_ft *, struct cds_ft_iter *);
static enum cds_ft_status ft_ineq_lt_keycopy(struct cds_ft *, struct cds_ft_iter *);
static enum cds_ft_status ft_ineq_lt_eager(struct cds_ft *, struct cds_ft_iter *);
static enum cds_ft_status ft_ineq_gt_keycopy(struct cds_ft *, struct cds_ft_iter *);
static enum cds_ft_status ft_ineq_gt_eager(struct cds_ft *, struct cds_ft_iter *);

static
void ft_install_lookup_ops(struct cds_ft *ft)
{
	const struct cds_ft_group *group = ft->group;
	bool sc = ft_group_skip_compressed(group);
	/*
	 * Limit-none relational entries: resolve the immutable use_keycopy config
	 * once and install the matching specialization, so cds_ft_lookup_le/ge/lt/gt
	 * (and cds_ft_next/prev) tail-call through @ft with no per-call config
	 * branch.  Independent of key_map.identity (ft_speculative_keycopy handles
	 * non-identity via ft_key_to_ordinals), so installed before the early
	 * non-identity return below.
	 */
	bool kc = group->speculative_key_offset_set && group->speculative &&
			(group->flags & CDS_FT_FLAG_SKIP_COMPRESSED);

	ft->lookup_le_fn = kc ? ft_ineq_le_keycopy : ft_ineq_le_eager;
	ft->lookup_ge_fn = kc ? ft_ineq_ge_keycopy : ft_ineq_ge_eager;
	ft->lookup_lt_fn = kc ? ft_ineq_lt_keycopy : ft_ineq_lt_eager;
	ft->lookup_gt_fn = kc ? ft_ineq_gt_keycopy : ft_ineq_gt_eager;

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
	ft_iter_assert_bound(ft, iter);
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
	ft_iter_assert_bound(ft, iter);
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
	iter->cache_valid = true;
	iter_debug_path_snapshot(iter);
end:
	iter_auto_invalidate_cache(iter);
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
	ft_iter_assert_bound(ft, iter);
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
		bool *skip_eq_external_nodes_p,
		const bool fill_ordinal)
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
	if (fill_ordinal)
		for (j = 0; j < (cmp_result ? (int)mpos : cmp); j++)
			ordinal_key[level - 1 + j] = cmp_key[j];
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
		if (fill_ordinal)
			ordinal_key[level - 1 + mpos] = cmp_key[mpos];
		if ((cmp_result > 0 && (mode == FT_LOOKUP_GE || mode == FT_LOOKUP_GT)) ||
		    (cmp_result < 0 && (mode == FT_LOOKUP_LE || mode == FT_LOOKUP_LT))) {
			/*
			 * Going-up: the whole compressed subtree is on the wrong
			 * side of the key, so back-track from the compressed node
			 * itself.  Return its SHALLOW boundary depth (@level, where
			 * its internal parent is the dispatcher at @level-1), not the
			 * interior divergence position @level+mpos: the divergence
			 * offset inside a compressed span is irrelevant to going-up
			 * (a compressed node has no within-span siblings).  The
			 * parent-pointer cursor seeds up_parent = the internal parent
			 * directly and proceeds to the sibling search.
			 */
			*level_p = level;
			iter_debug_path_snapshot(iter);
			FT_TP(ineq_compressed, (const void *) cn, level,
				cmp_result, mpos, (int) FT_DESCENT_GOING_UP);
			return FT_DESCENT_GOING_UP;
		}
		/* Descend into compressed subtree. */
		if (fill_ordinal) {
			ordinal_key[level - 1 + mpos] = cn->key_bytes[mpos];
			for (j = mpos + 1; j < cn->len; j++)
				ordinal_key[level - 1 + j] = cn->key_bytes[j];
		}
		level += cn->len - 1;
		node_flag = ft_dereference_acquire_prefetch(cn->child);
		assert(node_flag != NULL);	/* compressed node always has a live child */
		/*
		 * The descent result at this position is cn->child -- the
		 * dispatcher for the byte at index level - 1 (the byte AFTER
		 * the compressed prefix), not the compressed node itself.
		 * going_up relies on this to find the sibling of the failing
		 * byte in cn->child.
		 */
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

			if (fill_ordinal)
				for (k = cmp; k < cn->len; k++)
					ordinal_key[level - 1 + k] = cn->key_bytes[k];
			level += cn->len - 1;
			node_flag = ft_dereference_acquire_prefetch(cn->child);
			assert(node_flag != NULL);	/* compressed node always has a live child */
			*skip_eq_external_nodes_p = false;
			*node_flag_p = node_flag;
			*level_p = level;
			iter_debug_path_snapshot(iter);
			FT_TP(ineq_compressed, (const void *) cn, level,
				cmp_result, (unsigned int) cmp,
				(int) FT_DESCENT_DESCEND_CHILDREN);
			return FT_DESCENT_DESCEND_CHILDREN;
		}
		/* Going-up from the compressed node: shallow boundary (see
		 * the mismatch case above) -- not the interior @level+cmp-1. */
		*level_p = level;
		iter_debug_path_snapshot(iter);
		FT_TP(ineq_compressed, (const void *) cn, level,
			cmp_result, (unsigned int) cmp,
			(int) FT_DESCENT_GOING_UP);
		return FT_DESCENT_GOING_UP;
	}

	/* Full match: advance past compressed path. */
	level += cn->len - 1; /* -1: for loop increments */
	node_flag = ft_dereference_acquire_prefetch(cn->child);
	assert(node_flag != NULL);	/* compressed node always has a live child (by construction) */
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
		uint8_t *ordinal_key,
		enum ft_direction dir,
		const bool fill_ordinal)
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
	if (fill_ordinal)
		ft_fill_compressed_path(cn, ordinal_key, level - 1);
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
	*node_flag_p = node_flag;
	*level_p = level;
	if (ft_node_external(node_flag))
		return FT_DESCENT_BREAK;
	*skip_eq_external_nodes_p = false;
	return FT_DESCENT_CONTINUE;
}

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
 * @leaf stores the full result key — in the byte order the application passed
 * to cds_ft_insert() — at that offset.  Transform @level bytes of it to the
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
	if (!group->speculative_key_offset_set || !group->speculative ||
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
 * Lazy-ref accessor model (scoped to the
 * ordinal-cell ordered list).  In a cell group with a leaf-key offset and an
 * identity key map, the ordered-iteration result key is held as a LIVE
 * REFERENCE into the matched leaf (iter->node + speculative_key_offset) instead
 * of being copied into iter_key(iter) on every cell-walk step.  That removes
 * the per-step key copy — the cell walk never touches the leaf otherwise (its
 * node + ord_next co-reside in the 32B cell), so unlike the descent path the
 * leaf load is genuinely saved (the descent's going-up anchor would load it
 * regardless, which is why the by-reference key was a wash there).
 *
 * iter_key(iter) is therefore NOT the current key for such a position; every
 * reader of the current-position key MUST go through ft_iter_read_key().
 * Missing one silently corrupts (e.g. cds_ft_remove_all locating a wrong key).
 */
static inline
bool ft_iter_key_referenced(const struct cds_ft_iter *iter)
{
	const struct cds_ft_group *group = iter->ft->group;

	return group->ordered_list_set && group->speculative_key_offset_set &&
		group->key_map.identity && iter->cache_valid && iter->node;
}

/*
 * Read the iterator's CURRENT-POSITION key.  Returns the live leaf reference
 * when referenced, else the iter_key value.  Correct because every cell-walk /
 * descent result store sets iter->node to the matched leaf (whose stored key IS
 * the current key) and cds_ft_iter_set_key() clears cache_valid AND iter->node,
 * so iter->node + offset is authoritative exactly when cache_valid && node.
 * Valid only while the RCU lock that produced iter->node is held (cross-CS
 * callers cds_ft_iter_bind_key() first).
 */
static inline_lookup
struct ft_ord_cell *ft_ord_cell_cursor(const struct cds_ft_iter *iter);
static size_t ft_rebuild_key_upwalk(const struct cds_ft *ft,
		struct ft_ord_cell *cell, uint8_t *out, size_t max_len);

/*
 * Sentinel stored in iter->key_len by the ordinal-cell land for a VARIABLE-
 * length identity group: the length is DEFERRED and resolved on demand by
 * ft_iter_resolve_key_len() -- from the leaf (key_len_offset) when present, else
 * from the parent up-walk -- so a keyless cell walk reads neither key nor length.
 * SIZE_MAX is never a valid key length (bounded by max_key_len), so a consumer
 * that forgets to resolve hits an obvious overflow, not a silently-stale value.
 */
#define FT_ITER_KEY_LEN_LAZY	((size_t) -1)

/*
 * One-shot structural materialization for a VARIABLE-length EAGER ordered-list
 * iterator (no in-leaf key, no key_len_offset): the parent up-walk derives BOTH
 * the key bytes (into iter_key) AND the length in a SINGLE walk.  Caches the
 * length in iter->key_len (clearing the LAZY sentinel) so a later read_key /
 * resolve_key_len in the same step reuses it instead of walking again.  Returns
 * the length (0 on a NIL key / overflow / missing cell).
 */
static
size_t ft_iter_upwalk_into_buf(struct cds_ft_iter *iter)
{
	size_t max_len = iter->ft->group->max_key_len;
	struct ft_ord_cell *cell = ft_ord_cell_cursor(iter);
	size_t n = 0;

	if (cell)
		n = ft_rebuild_key_upwalk(iter->ft, cell, iter_key(iter), max_len);
	iter->key_len = n;
	iter->key_off = max_len - n;	/* key lives at iter_key[key_off ..) */
	iter->path_len = n + 1;
	return n;
}

static inline
const uint8_t *ft_iter_read_key(const struct cds_ft_iter *iter)
{
	const struct cds_ft_group *group = iter->ft->group;

	if (ft_iter_key_referenced(iter))
		return (const uint8_t *) iter->node + group->speculative_key_offset;
	/*
	 * EAGER ordered-list walk (no in-leaf key): rematerialize the current
	 * key STRUCTURALLY via the parent up-walk into iter_key.  Reached only
	 * when a key consumer asks for the key -- a keyless/count walk never
	 * calls this, so the O(depth) walk is paid strictly on demand.  The
	 * walk recovers ORDINAL bytes from the trie structure, which is what
	 * iter_key holds for ANY key map (consumers remap via
	 * ft_ordinals_to_key), so no identity requirement.  FIXED-length walks
	 * into the buffer (length is group->key_len).  VARIABLE-length derives
	 * the length from the SAME walk, cached via ft_iter_upwalk_into_buf and
	 * coordinated with ft_iter_resolve_key_len through the LAZY sentinel so
	 * one walk serves both.
	 */
	if (group->ordered_list_set && !group->speculative_key_offset_set &&
			iter->cache_valid && iter->node) {
		if (group->key_len == CDS_FT_LEN_VARIABLE) {
			/*
			 * The up-walk fills the key at the buffer TAIL and records
			 * iter->key_off; coordinated with ft_iter_resolve_key_len
			 * through the LAZY sentinel so one walk serves both.
			 */
			if (iter->key_len == FT_ITER_KEY_LEN_LAZY)
				ft_iter_upwalk_into_buf(
					(struct cds_ft_iter *) iter);
			return iter_key(iter) + iter->key_off;
		} else {
			struct ft_ord_cell *cell = ft_ord_cell_cursor(iter);
			size_t max_len = group->max_key_len;
			size_t n;

			if (cell && (n = ft_rebuild_key_upwalk(iter->ft, cell,
					iter_key(iter), max_len)) != 0) {
				((struct cds_ft_iter *) iter)->key_off =
					max_len - n;
				return iter_key(iter) + (max_len - n);
			}
		}
	}
	return iter_key(iter) + iter->key_off;
}

/*
 * Rebuild the ORDINAL key for a cell head @cell of length @key_len by walking
 * UP the parent chain, recovering each level's branch byte structurally:
 *   - external head: its last byte = the head's cell-metadata incoming_byte
 *     (unless its parent is a compressed node, whose key_bytes already span
 *     through the head's position).
 *   - internal node: metadata->incoming_byte (skip the root, which has none).
 *   - compressed node: its key_bytes[] span PLUS metadata->incoming_byte (the
 *     slot byte under which it hangs in its parent -- separate from key_bytes).
 * Fills @out[0..key_len) (caller-sized >= key_len) and returns true when the
 * walk accounts for exactly key_len bytes.
 *
 * This is the structural key source for the ordered-list walk that needs NO
 * speculative_key_offset (in-leaf key) and NO parent-bitmap inversion -- it
 * makes the ordered list usable on an EAGER / no-leaf-key trie.  Valid only
 * while the RCU lock that produced @cell is held continuously.
 */
static
size_t ft_rebuild_key_upwalk(const struct cds_ft *ft, struct ft_ord_cell *cell,
		uint8_t *out, size_t max_len)
{
	struct cds_ft_inode_flag *nf;
	size_t pos = max_len;	/* fill DEEPEST-byte-first leftward from the end */

	(void) ft;
	if (!cell)
		return 0;

	/*
	 * Resolve a cds_ft_merge / graft_swap flip proxy on every parent load: a
	 * concurrent bulk op re-parents nodes via a type-7 proxy installed BEFORE
	 * its drain, so an up-walk running under the reader's RCU lock would
	 * otherwise dereference the proxy as a node.  Gives the view-appropriate
	 * (old-or-merged) parent, consistent across the walk.
	 */
	nf = ft_resolve_flip_proxy(rcu_dereference(cell->parent));

	/*
	 * Fill the buffer FROM THE END: write the deepest (leaf-edge) byte at
	 * out[max_len-1] and grow leftward, so the key ends up in correct order
	 * occupying out[pos .. max_len) with NO reversal and the length DERIVED
	 * from the walk (pos drops by however many bytes the walk contributes --
	 * no key_len needed, so variable-length keys with no in-leaf length work).
	 * A compressed span lands as one contiguous forward memcpy (its key_bytes
	 * are already in key order); on underflow past out[0], fail (return 0).
	 * The key STARTS at out[max_len - returned]; the caller keeps that offset.
	 *
	 * Head's last byte: when it hangs off an internal node it sits in a slot
	 * whose byte is the head's cell-metadata incoming_byte; when its parent is
	 * a compressed node the head is that node's child and carries no separate
	 * edge byte (the compressed key_bytes run through the head's position).
	 */
	if (!nf) {
		/* Parentless head: its byte stands alone. */
		struct cds_ft_metadata *hmeta = cds_ft_item_to_metadata(cell);

		if (pos == 0)
			return 0;
		out[--pos] = (uint8_t) hmeta->incoming_byte;
	} else if (!ft_node_compressed(ft_resolve_skip_compressed(ft, nf))) {
		/*
		 * Parent is an internal (slot-array) node.  Write the head's edge
		 * byte ONLY when the head hangs off a SLOT.  A PREFIX key sits at the
		 * parent's external_nodes -- the key ENDS at the parent, so its last
		 * byte IS the parent's own incoming edge (written when the parent is
		 * processed below) and must not be double-counted here.  Fixed-length
		 * tries have no prefix keys, so this is always a slot head there.
		 */
		struct cds_ft_metadata *nmeta = ft_flag_to_metadata(ft, nf);

		if (cell->node !=
				ft_dereference_external(nmeta->external_nodes)) {
			struct cds_ft_metadata *hmeta =
				cds_ft_item_to_metadata(cell);

			if (pos == 0)
				return 0;
			out[--pos] = (uint8_t) hmeta->incoming_byte;
		}
	}
	/* else parent compressed: the head byte is covered by its key_bytes. */

	while (nf) {
		struct cds_ft_inode_flag *rnf = ft_resolve_skip_compressed(ft, nf);
		struct cds_ft_metadata *meta = ft_flag_to_metadata(ft, nf);

		if (ft_node_compressed(rnf)) {
			const struct cds_ft_compressed_node *cn =
				(const struct cds_ft_compressed_node *)
				ft_node_ptr(rnf);
			unsigned int len = cn->len;

			/*
			 * Compressed span in key order (key_bytes[0..len)): it sits
			 * immediately to the LEFT of what we've written so far, so a
			 * single forward memcpy places it correctly.
			 */
			if (pos < len)
				return 0;
			pos -= len;
			memcpy(&out[pos], cn->key_bytes, len);
		}
		/*
		 * This node's incoming edge byte (the slot byte in its parent).
		 * Read meta->parent ONCE (resolving a concurrent bulk op's flip
		 * proxy) and use it for both the compressed-parent test and the
		 * advance.  Contributed only when the PARENT is an internal
		 * (slot-array) node: a compressed parent's last key_byte already IS
		 * this edge, counted when that parent is processed -- so skip it here
		 * (mirrors the head's skip when its parent is compressed).  The root
		 * has no parent and contributes none.
		 */
		{
			struct cds_ft_inode_flag *parent =
				ft_resolve_flip_proxy(rcu_dereference(meta->parent));

			if (parent && !ft_node_compressed(
					ft_resolve_skip_compressed(ft, parent))) {
				if (pos == 0)
					return 0;
				out[--pos] = (uint8_t) meta->incoming_byte;
			}
			nf = parent;
		}
	}

	/* Key now occupies out[pos .. max_len) in key order; length = max_len - pos. */
	return max_len - pos;
}

/*
 * Resolve (and cache) the iterator's current-position key length.  For a
 * deferred-length cell position it reads node->key_len from the leaf once and
 * caches it into iter->key_len (and path_len); otherwise returns iter->key_len
 * unchanged (a no-op for fixed-length, non-identity, descent and non-cell
 * positions).  EVERY reader of the current-position length (cds_ft_iter_get_key,
 * cds_ft_remove*, the skip rebuilds, bind, the max-key-len scan) must call this
 * before reading iter->key_len.  The LAZY sentinel is only ever set with
 * cache_valid && node, so the leaf read is safe.
 */
static inline
size_t ft_iter_resolve_key_len(struct cds_ft_iter *iter)
{
	if (caa_unlikely(iter->key_len == FT_ITER_KEY_LEN_LAZY)) {
		/*
		 * VARIABLE-length EAGER ordered-list (no in-leaf KEY at
		 * speculative_key_offset): the key source is the iter buffer, and
		 * clearing the LAZY sentinel doubles as "buffer filled" for
		 * ft_iter_read_key -- so the length MUST come from the parent
		 * up-walk, which fills iter_key (+ key_off) in the same walk that
		 * derives the length, even when an in-leaf LENGTH (key_len_offset)
		 * is configured.  Only a leaf-referenced key (in-leaf key present)
		 * may take the leaf-length shortcut: its key reads never touch the
		 * buffer.
		 */
		if (!iter->ft->group->key_len_offset_set ||
				!iter->ft->group->speculative_key_offset_set) {
			ft_iter_upwalk_into_buf(iter);
			return iter->key_len;
		}
		iter->key_len = *(const size_t *) ((const char *) iter->node +
			iter->ft->group->key_len_offset);
		iter->path_len = iter->key_len + 1;
	}
	return iter->key_len;
}

/*
 * Copy a live leaf-referenced key into iter_key so it survives the position
 * being detached (UNCACHED auto-invalidate, bind, any cache_valid clear).  A
 * no-op self-copy when the key is already a value there.  Bytes are ordinal
 * (ft_iter_key_referenced requires an identity map).  Resolves the length first
 * so a deferred-length position materializes both before its node is dropped.
 */
static inline
void ft_iter_materialize_key(struct cds_ft_iter *iter)
{
	size_t klen = ft_iter_resolve_key_len(iter);
	const uint8_t *cur = ft_iter_read_key(iter);

	/*
	 * Pin the current key at the FRONT of iter_key (key_off = 0).  The hot
	 * cell-walk read keeps the up-walk key at the buffer TAIL (no move), but
	 * materialize is the bind / UNCACHED path: the saved key must then be
	 * re-descended from, and the relational descent reads its search key and
	 * writes its result into the SAME iter_key buffer -- which is only safe
	 * (result == search before divergence) when the key starts at offset 0.
	 * memmove because @cur (the up-walk tail, or a leaf reference) may overlap.
	 */
	if (cur != iter_key(iter)) {
		memmove(iter_key(iter), cur, klen);
		iter->key_off = 0;
	}
}


/*
 * Land an ordinal-cell walk result on @iter: materialize the head's key from
 * its leaf and cache the cell as the walk cursor.  @cell == NULL reports
 * NOT_FOUND (end of list).  Shared by the cell fast path and the O(1)
 * lookup_first / lookup_last endpoints.  Returns iter->status.
 */
static inline_lookup
enum cds_ft_status ft_ord_cell_iter_land(struct cds_ft *ft,
		struct cds_ft_iter *iter, struct ft_ord_cell *cell)
{
	struct cds_ft_node *node;
	size_t rlen;

	if (!cell) {
		iter->node = NULL;
		iter->ord_cell = NULL;
		iter->ord_cell_node = NULL;
		iter->cache_valid = false;
		iter_debug_path_update(iter);
		iter->status = CDS_FT_STATUS_NOT_FOUND;
		return iter->status;
	}
	node = cell->node;
	iter->node = node;
	iter->ord_cell = cell;
	iter->ord_cell_node = node;
	iter->cache_valid = true;
	/*
	 * Materialize as little as possible — the cell walk's point is that a
	 * keyless / count traversal touches NO leaf:
	 *
	 *  - Identity map (key read in place from the leaf when an in-leaf key
	 *    is configured, see ft_iter_key_referenced) or EAGER group (no
	 *    in-leaf key; the up-walk in ft_iter_read_key recovers the ordinal
	 *    bytes structurally, identity or not): no key copy.  A VARIABLE-
	 *    length group also DEFERS the length: iter->key_len is the LAZY
	 *    sentinel, resolved on demand by ft_iter_resolve_key_len() only in
	 *    the key-consuming ops (get_key / remove / skip / bind).  So a
	 *    keyless variable-length walk reads neither the key nor the length
	 *    from the leaf.  A FIXED-length group takes its length from
	 *    group->key_len (no leaf touch either).
	 *  - Non-identity map WITH an in-leaf key: the leaf bytes are in
	 *    application order, so they must be remapped to ordinal order now,
	 *    which needs the length now — read it (leaf, for variable) and copy
	 *    into iter_key (ft_iter_key_referenced is then false, consumers use
	 *    the buffer).
	 */
	if (caa_likely(ft->group->key_map.identity ||
			!ft->group->speculative_key_offset_set)) {
		if (ft->group->key_len != CDS_FT_LEN_VARIABLE) {
			iter->key_len = ft->group->key_len;
			iter->path_len = ft->group->key_len + 1;
		} else {
			iter->key_len = FT_ITER_KEY_LEN_LAZY;
			iter->path_len = 0;	/* materialized with the length */
		}
	} else {
		rlen = (ft->group->key_len != CDS_FT_LEN_VARIABLE) ?
			ft->group->key_len :
			*(const size_t *) ((const char *) node +
				ft->group->key_len_offset);
		ft_speculative_keycopy_unconditional(ft, node, iter_key(iter),
			(ssize_t) rlen);
		iter->key_len = rlen;
		iter->path_len = rlen + 1;
	}
	iter_debug_path_update(iter);
	iter->status = CDS_FT_STATUS_OK;
	return iter->status;
}

/*
 * True when the ordinal-cell O(1) endpoints / fast path can serve @iter: the
 * list is enabled, the result key + length are recoverable from the leaf, and
 * the traversal is unscoped.
 */
static inline_lookup
bool ft_ord_cell_fastpath_ok(const struct cds_ft *ft,
		const struct cds_ft_iter *iter)
{
	/*
	 * EAGER structural up-walk: an ordered-list group with NO in-leaf key
	 * (no speculative_key_offset) gets BOTH the result key AND its length
	 * from the parent up-walk (ft_iter_read_key / ft_iter_resolve_key_len),
	 * for fixed OR variable length -- no in-leaf key and no key_len_offset
	 * needed.  The walk recovers ORDINAL bytes, so it serves any key map
	 * (a non-identity map is applied on result-key copy-out).
	 */
	bool up_walk = ft->group->ordered_list_set &&
		!ft->group->speculative_key_offset_set;

	return ft->group->ordered_list_set &&
		(up_walk ||
		 /*
		  * In-leaf key (ft_ord_cell_iter_land reads/remaps it from the
		  * leaf): needs the length too -- fixed, or variable with an
		  * in-leaf length (key_len_offset).
		  */
		 (ft->group->key_len != CDS_FT_LEN_VARIABLE ||
			ft->group->key_len_offset_set)) &&
		iter->prefix_len == 0;
}

/*
 * Resolve the current head's cell for a cell-walk step: use the cached cursor
 * when it still refers to iter->node (no leaf touch), else re-enter the walk
 * via the head's prev (one leaf load — the per-walk-entry cost).
 */
static inline_lookup
struct ft_ord_cell *ft_ord_cell_cursor(const struct cds_ft_iter *iter)
{
	if (iter->ord_cell_node == iter->node)
		return iter->ord_cell;
	return ft_ord_cell_ptr(rcu_dereference(iter->node->prev));
}

static inline_lookup
enum cds_ft_status cds_ft_lookup_inequality_impl(struct cds_ft *ft,
		struct cds_ft_iter *iter,
		enum ft_lookup_inequality mode,
		enum ft_lookup_limit limit,
		const bool use_keycopy,
		const bool seed_from_node)
{
	ssize_t key_depth, level;
	struct cds_ft_inode_flag *node_flag;
	struct cds_ft_node *ret_node;
	uint8_t ordinal_key[FT_MAX_KEY_LEN];
	/*
	 * @keep_ordinal: compile-time true for every instantiation EXCEPT
	 * (use_keycopy && limit == LIMIT_NONE).  In that one case the matched leaf
	 * is the sole result-key source (ft_speculative_keycopy_unconditional) and
	 * going-up dispatch reads input_key, so ordinal_key is dead: its memset,
	 * per-level fills (here and in the compressed helpers via fill_ordinal),
	 * byte-record output slots, and fallback copies all DCE.  @ord_scratch is
	 * the write-only sink for the going-up sibling / minmax byte-record on that
	 * path (ft_node_get_direction needs a valid output slot).
	 */
	const bool keep_ordinal = !(use_keycopy &&
			limit == FT_LOOKUP_LIMIT_NONE);
	uint8_t ord_scratch;
	enum ft_direction dir;
	const uint8_t *input_key = NULL;	/* set below; init for the hoisted cell-fastpath goto end */
	const uint8_t *iter_key;
	size_t key_len = 0;
	bool going_up = false, skip_eq_external_nodes;
	/*
	 * Parent-pointer going-up cursor (structural up_node form,
	 * mirroring the iter_skip walk-up).  @up_node is the deepest live
	 * node descent established; @up_node_lo is the shallowest depth it
	 * covers -- its TRUE shallow boundary (several levels below @up_node
	 * for a compressed run).  Tracked as descent proceeds so the going-up
	 * seed is live (taken from the descent, not read back from a
	 * recorded path), then climbed via ft_get_parent_rcu up to the
	 * going-up @level.  Seeding from the
	 * deepest node + true shallow boundary (rather than from @node_flag
	 * and the end-of-key-adjusted @level) avoids both the level
	 * adjustment desync and the compressed-span boundary ambiguity.
	 */
	struct cds_ft_inode_flag *up_node = NULL;
	ssize_t up_node_lo = 0;
	/*
	 * @use_keycopy is a compile-time literal at each instantiation (see the
	 * two cds_ft_lookup_inequality_impl callers in the dispatcher below), so
	 * always_inline constant-folds every per-call gate on it.  When set, a
	 * configured speculative skip-compressed group recovers the result key
	 * from the matched leaf and the min-descent follows skip pointers without
	 * reading the compressed node or filling ordinal_key; otherwise the
	 * descent rebuilds ordinal_key from the live compressed nodes (the fill +
	 * re-anchor path).
	 */

	CDS_FT_ASSERT_RCU_READ_LOCKED(ft);

	/*
	 * Ordinal-cell fast path: cds_ft_next / cds_ft_prev (GT/LT,
	 * LIMIT_NONE) on a cached head collapse to a single dependent load of the
	 * cell's ord_next / ord_prev — no descent, no leaf touch for the step
	 * (cell->node + cell->ord_* co-reside in the 32B cell).  Hoisted ABOVE the
	 * key_len computation so the common walk does NOT resolve the (deferred,
	 * LAZY) length: the fast path advances via ord_next and never needs it.
	 * Only a fall-through to the descent (cache invalid / scoped) resolves the
	 * length where it is first used.
	 *
	 * @seed_from_node (a compile-time literal at every public instantiation, so
	 * the gate DCEs there) suppresses this fast path for the splice-time
	 * predecessor seed: that caller positions @iter at a FRESH head whose cell
	 * is not yet spliced (its ord_prev/ord_next are unset), so the cell cursor
	 * would resolve garbage.  It instead wants the cross-call node-recovery fast
	 * path below, which reconstructs the going-up seed from iter->node and runs
	 * the structural backtrack -- the predecessor among the ALREADY-linked keys.
	 */
	if (!seed_from_node &&
			(mode == FT_LOOKUP_GT || mode == FT_LOOKUP_LT) &&
			limit == FT_LOOKUP_LIMIT_NONE &&
			iter->cache_valid && iter->node &&
			ft_ord_cell_fastpath_ok(ft, iter)) {
		struct ft_ord_cell *cur = ft_ord_cell_cursor(iter);
		struct ft_ord_cell *nxt = (mode == FT_LOOKUP_GT) ?
			ft_ord_cell_resolve_ord(&cur->ord_next) :
			ft_ord_cell_resolve_ord(&cur->ord_prev);

		ft_ord_cell_iter_land(ft, iter, nxt);
#ifndef FT_NO_ORD_PREFETCH
		/* One-hop NTA prefetch of the cell the next call will land on. */
		if (nxt) {
			struct ft_ord_cell *nn = (mode == FT_LOOKUP_GT) ?
				ft_ord_cell_resolve_ord(&nxt->ord_next) :
				ft_ord_cell_resolve_ord(&nxt->ord_prev);
			if (nn)
				__builtin_prefetch((const void *) nn, 0, 0);
		}
#endif
		goto end;
	}

	switch (limit) {
	case FT_LOOKUP_LIMIT_NONE:
		/*
		 * Continuation from the current position: its length may be the
		 * deferred LAZY sentinel (a cell walk that fell to the descent on a
		 * scoped step) -- resolve it from the leaf.  No-op for fixed /
		 * descent / set_key positions.
		 */
		key_len = ft_key_len(ft, ft_iter_resolve_key_len(iter));
		if (!valid_key_len(ft, key_len)) {
			iter->node = NULL;
			iter->cache_valid = false;
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
	 * Read the input key in place from the iterator buffer -- no snapshot
	 * copy.  Every write to iter_key(iter) (the result) is terminal (goto
	 * end) or sourced from the separate ordinal_key / leaf-copy buffer, and
	 * all input reads (the descent and the going-up backtrack) precede any
	 * terminal write, so the input bytes are never overwritten while still
	 * needed.  The lone self-aliasing case -- the equal-match write below --
	 * is a no-op and is guarded.
	 *
	 * Input-key source: only a LIMIT_NONE continuation (cds_ft_next/prev, the
	 * relational lookups) iterates from the iterator's CURRENT key, which on a
	 * lazy-ref cell group is read IN PLACE from the live node (no prior copy).
	 * LIMIT_FIRST/LIMIT_LAST are absolute and key off the prefix in iter_key,
	 * NOT the current position, so they must NOT reference the node (a reused
	 * iterator's stale node would mis-seed the search).
	 */
	if (limit == FT_LOOKUP_LIMIT_NONE)
		input_key = ft_iter_read_key(iter);
	else
		input_key = iter_key(iter);
	iter_key = input_key;

	FT_TP(ineq_enter, (int) mode, input_key, key_len);

	if (keep_ordinal)
		memset(ordinal_key, 0, ft->group->max_key_len * sizeof(ordinal_key[0]));
	node_flag = ft_root_dereference_prefetch(ft);
	up_node = node_flag;		/* root covers depth 0 */
	up_node_lo = 0;

	/*
	 * Empty root short-circuit: when the root has no children,
	 * there is nothing to traverse and no inequality match is
	 * possible. An empty root is always a type-0 popcount_2l node
	 * with an all-zero bitmap header.  Compressed roots are never
	 * empty (recompaction replaces emptied compressed roots with
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
			iter->cache_valid = true;
			iter_debug_path_snapshot(iter);
			iter->path_len = 1;
			iter->status = CDS_FT_STATUS_NOT_FOUND;
			goto end;
		}
	}

	/*
	 * Fast path: reuse the iterator's cached position from a prior
	 * lookup when it is still valid and covers the full key
	 * depth.  This avoids redundant per-level ft_node_get_nth(ft, )
	 * lookups (which are the expensive, cache-miss-prone part of
	 * the downward walk).  The caller must hold the RCU read-side
	 * lock continuously for the cached pointers to remain valid.
	 */
	iter_debug_path_check(iter);
	/*
	 * Continuation fast path: recover the position from iter->node by
	 * backtracking up the live parent chain.  Reuse it only when the
	 * cached position is EXACTLY at this key (path_len == key_depth and
	 * iter->node set) -- iter->node is then the deepest position for the
	 * key.  A deeper cached position (path_len > key_depth, e.g. an
	 * inequality result that landed on a longer key) is not the right
	 * cursor for this key and falls to slow_path.  The caller must hold
	 * the RCU read-side lock continuously for iter->node to stay valid.
	 */
	if (iter->cache_valid && iter->node &&
			(ssize_t)iter->path_len == key_depth &&
			key_depth > 1) {
		for (level = 1; level < key_depth; level++) {
			if (!keep_ordinal)
				continue;
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
		{
			/*
			 * Cross-call continuation: recover the deepest trie node
			 * for iter->key from the cached position iter->node (the
			 * dup-chain head), not from a recorded descent path.  A
			 * prefix key sits at an internal/compressed holder whose
			 * external_nodes == iter->node; otherwise iter->node is a
			 * leaf child (a tag-0 external flag).  ft_get_parent_rcu
			 * on the head reaches the holder in O(1) (head->prev ==
			 * holder) and never walks the dup chain.
			 */
			struct cds_ft_inode_flag *cur =
				(struct cds_ft_inode_flag *) iter->node;
			struct cds_ft_inode_flag *holder =
				ft_get_parent_rcu(ft, cur);

			if (holder && !ft_node_external(holder) &&
			    ft_node_external_nodes(holder) ==
					(struct cds_ft_node *) iter->node)
				node_flag = holder;
			else
				node_flag = cur;
		}
		/*
		 * If the cached path entry is a compressed node, the
		 * fast path cannot determine the correct loop exit
		 * state (compressed nodes span multiple levels).
		 * Fall back to the slow path.
		 */
		if (ft_node_compressed(node_flag) ||
		    ft_node_skip_compressed(node_flag)) {
			node_flag = ft_root_dereference_prefetch(ft);
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
		/*
		 * Cross-call fast path: the cached deepest node is the going-up
		 * seed.  It is never compressed/skip here (those fall back to
		 * slow_path above), so its shallow boundary is its own depth.
		 */
		up_node = node_flag;
		up_node_lo = key_depth - 1;
		FT_TP(fastpath_enter, (int) mode,
			(const void *) node_flag, (int) level);
		goto post_traversal;
	}

slow_path:
	FT_TP(slowpath_enter, (int) mode, (int) iter->cache_valid,
		(int) iter->path_len);
	for (level = 1; level < key_depth; level++) {
		uint8_t key_value;

		if (ft_node_compressed(node_flag)) {
			enum ft_descent_action act;
			ssize_t cmp_entry_level = level;

			act = ft_inequality_compressed(&node_flag,
				&level, key_depth, mode, limit,
				&iter_key, input_key, iter,
				ordinal_key, &skip_eq_external_nodes,
				keep_ordinal);
			if (act == FT_DESCENT_GOING_UP) {
				/*
				 * @node_flag is the compressed node; it occupies
				 * depths [cmp_entry_level-1, +cn->len).  Seed the
				 * cursor at its TRUE shallow boundary so the
				 * going-up climb reaches its internal parent.
				 */
				up_node = node_flag;
				up_node_lo = cmp_entry_level - 1;
				goto going_up;
			}
			/*
			 * Descend / break / continue: @node_flag is cn->child
			 * at the advanced @level (span 1 -- chain-merge forbids
			 * compressed-under-compressed).
			 */
			up_node = node_flag;
			up_node_lo = level;
			if (act == FT_DESCENT_DESCEND_CHILDREN)
				goto descend_children;
			if (act == FT_DESCENT_BREAK)
				break;
			if (level + 1 >= key_depth) {
				skip_eq_external_nodes = false;
				/*
				 * A compressed full-match consumed the whole key and
				 * landed on cn->child (which may hold this key as a
				 * prefix-key external).  For GE/GT (RIGHT) descend into
				 * that subtree: its leftmost is the smallest key >= the
				 * search key.  But for LE/LT (LEFT) every key in the
				 * subtree is >= the search key, so descending would
				 * return the subtree MAX, which is strictly greater --
				 * wrong.  Break to post_traversal instead, exactly as the
				 * non-compressed path does: it returns cn->child's own
				 * external_nodes for LE (the equal match) and climbs for
				 * LT's strict predecessor.
				 */
				if (mode == FT_LOOKUP_LE || mode == FT_LOOKUP_LT)
					break;
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
		if (keep_ordinal)
			ordinal_key[level - 1] = key_value;
		{
			unsigned int rewind;

			/*
			 * Surgical re-anchor on a skip-compressed mismatch (no
			 * spin on a frozen slot).  rewind > 0: a concurrent
			 * chain-merge moved the encoded position shallower;
			 * re-descend the fixed key path from the root, exactly
			 * like the fast-path fallback above.
			 */
			node_flag = ft_node_get_nth_reanchor(ft, node_flag,
					key_value, &rewind);
			if (caa_unlikely(rewind)) {
				node_flag = ft_root_dereference_prefetch(ft);
				iter_key = input_key;
				goto slow_path;
			}
		}
		if (!node_flag) {
			FT_TP(slowpath_step, (int) level, key_value,
				(const void *) node_flag, 1);
			break;
		}
		up_node = node_flag;		/* child established at @level (span 1) */
		up_node_lo = level;
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
	/*
	 * If the descent consumed the whole key, normalize @level to the
	 * end-of-key depth (key_depth - 1) BEFORE the equal-match check below.
	 * The loop overshoots by one when the key terminates on an INTERNAL
	 * node (level reaches key_depth) rather than breaking on an external
	 * leaf (level == key_depth - 1): an internal node that holds the key as
	 * a prefix-key external (it also has a longer-key subtree) is the exact
	 * match, and the LE/GE arm's ft_node_internal branch returns its
	 * external_nodes.  Without this normalization that arm is skipped and
	 * the exact prefix-key match is missed (the non-compressed analogue of
	 * the compressed full-match handled in the slow-path loop above).
	 */
	if (level >= key_depth)
		level = key_depth - 1;
	switch (mode) {
	case FT_LOOKUP_LE:
	case FT_LOOKUP_GE:
		if (level == key_depth - 1) {
			struct cds_ft_node *external_nodes;

			if (ft_node_internal(node_flag)) {
				struct cds_ft_metadata *metadata;
				const struct cds_ft_type *type = &ft_types[ft_node_type(node_flag)];

				metadata = cds_ft_item_to_metadata_fast(ft_node_ptr(node_flag), type->order);
				external_nodes = ft_dereference_external(metadata->external_nodes);
			} else if (ft_node_compressed(node_flag)) {
				struct cds_ft_metadata *metadata =
					cds_ft_item_to_metadata(ft_node_ptr(node_flag));
				external_nodes = ft_dereference_external(metadata->external_nodes);
			} else {
				external_nodes = (struct cds_ft_node *) node_flag;
			}
			if (external_nodes) {
				/* End of key lookup succeded. We got an equal match.
				 * The result key equals the input, which is read in
				 * place from iter_key(iter), so the copy is a no-op
				 * self-copy unless a separate buffer is in use.
				 * memmove: @input_key may be the up-walk key at this
				 * same buffer's TAIL (iter_key + key_off), which the
				 * front write overlaps. */
				iter->key_len = key_len;
				iter->key_off = 0;
				if (!ft_speculative_keycopy(ft, external_nodes,
						iter_key(iter), (ssize_t) key_len) &&
						input_key != iter_key(iter))
					memmove(iter_key(iter), input_key, key_len);
				iter->node = external_nodes;
				iter->cache_valid = true;
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

	/*
	 * LE/LT dead-ended at an external LEAF that is a PROPER PREFIX of the
	 * search key: the descent matched the leaf's whole key (depth @level)
	 * but the search key is longer, so leaf_key < search_key.  A leaf has
	 * no extensions, so no key lies strictly between it and the search key
	 * on this path -- it is the largest key <= the search key, larger than
	 * any left-sibling branch (which diverges at a smaller byte).  going_up
	 * skips external leaves (it only returns internal/compressed nodes'
	 * external_nodes), so return the leaf here.  The exact-length match
	 * (@level == key_depth - 1) is already handled by the LE/GE arm above
	 * (LE) or excluded by strictness (LT), so this is only the proper-prefix
	 * case.
	 *
	 * node_flag may be NULL here (the descent's empty-slot break), which
	 * ft_node_external() also matches: that is a mid-key dead-end, NOT a
	 * proper-prefix leaf -- fall through to going_up, which backtracks to
	 * the nearest lesser key.
	 */
	if (limit == FT_LOOKUP_LIMIT_NONE &&
			(mode == FT_LOOKUP_LE || mode == FT_LOOKUP_LT) &&
			node_flag && ft_node_external(node_flag) &&
			level < key_depth - 1) {
		int j;

		assert(level <= (int) ft->group->max_key_len);
		iter->key_len = level;
		iter->key_off = 0;
		if (!keep_ordinal)
			ft_speculative_keycopy_unconditional(ft,
				(const struct cds_ft_node *) ft_node_ptr(node_flag),
				iter_key(iter), level);
		else if (!ft_speculative_keycopy(ft,
				(const struct cds_ft_node *) ft_node_ptr(node_flag),
				iter_key(iter), level)) {
			for (j = 0; j < level; j++)
				iter_key(iter)[j] = ordinal_key[j];
		}
		iter->node = (struct cds_ft_node *) ft_node_ptr(node_flag);
		iter->cache_valid = true;
		iter_debug_path_update(iter);
		iter->path_len = level + 1;
		iter->status = CDS_FT_STATUS_OK;
		goto end;
	}

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

	/*
	 * LE/LT, relational (LIMIT_NONE), with the search key exhausted at or
	 * above the prefix boundary: the empty key (level == prefix_len == 0), or
	 * a query equal to the scope prefix.  The going-up loop below would not
	 * run (no shorter key to back-track to) and control would fall through to
	 * descend_children, wrongly returning the subtree MAX.  No key is strictly
	 * less than the boundary; the only <= match is the boundary node's own
	 * external_nodes, which LE returns and LT excludes -> NOT_FOUND otherwise.
	 * (LIMIT_FIRST/LAST want the subtree min/max here, so they are excluded.)
	 */
	if (limit == FT_LOOKUP_LIMIT_NONE &&
			(mode == FT_LOOKUP_LE || mode == FT_LOOKUP_LT) &&
			level <= (ssize_t) iter->prefix_len && node_flag &&
			!ft_node_external(node_flag)) {
		struct cds_ft_node *ext = NULL;

		if (mode == FT_LOOKUP_LE) {
			struct cds_ft_metadata *m;

			if (ft_node_compressed(node_flag))
				m = cds_ft_item_to_metadata(ft_node_ptr(node_flag));
			else
				m = cds_ft_item_to_metadata_fast(ft_node_ptr(node_flag),
					ft_types[ft_node_type(node_flag)].order);
			ext = ft_dereference_external(m->external_nodes);
		}
		if (ext) {
			int j;

			iter->key_len = iter->prefix_len;
			iter->key_off = 0;
			if (!keep_ordinal)
				ft_speculative_keycopy_unconditional(ft, ext,
					iter_key(iter), (ssize_t) iter->prefix_len);
			else if (!ft_speculative_keycopy(ft, ext, iter_key(iter),
					(ssize_t) iter->prefix_len)) {
				for (j = 0; j < (int) iter->prefix_len; j++)
					iter_key(iter)[j] = ordinal_key[j];
			}
			iter->node = ext;
		} else {
			iter->node = NULL;
		}
		iter->cache_valid = true;
		iter_debug_path_update(iter);
		iter->path_len = iter->prefix_len + 1;
		iter->status = ext ? CDS_FT_STATUS_OK : CDS_FT_STATUS_NOT_FOUND;
		goto end;
	}

going_up:
	/* Ensure iter_key is exactly at the position matching the level we stopped at. */
	iter_key = input_key + level;

	/*
	 * Find highest value left/right of current node.
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
	/*
	 * Parent-pointer going-up cursor.  The node at @level and the
	 * dispatcher at @level-1 (scanned for a sibling) are obtained via live
	 * back-pointer reads,
	 * which may observe a fresher version than the descent snapshot
	 * (intentional -- freshness -- and semantically valid).
	 *
	 * Anchor on @up_parent = node at depth @level-1: it is always valid
	 * for level > prefix_len (descent filled it), unlike node-at-level
	 * which is NULL on a dead-end descent.  @up_parent_lo is the shallowest
	 * depth @up_parent covers (> 1 for a compressed run).  As the loop
	 * decrements @level, climb @up_parent to its own parent via
	 * ft_get_parent_rcu once @level-1 drops below @up_parent_lo, accumulating
	 * span (compressed = cn->len, internal = 1).  @up_node (node at @level)
	 * is carried from the previous iteration's @up_parent -- the node we
	 * just climbed past -- so it is never derived by dereferencing a
	 * possibly-NULL node-at-level.  It is read only inside the LE/LT block
	 * below, which the first iteration (going_up == false) skips.
	 */
	{	/* Scope the going-up cursor locals so they fall out of scope before descend_children/end (avoids -Wjump-misses-init false positives). */
		struct cds_ft_inode_flag *up_parent;
		ssize_t up_parent_lo;
		bool up_first = true;

		/*
		 * Climb @up_node (the deepest live node descent established) up to
		 * the node covering the going-up @level: the loop @level may have
		 * been knocked back one by the end-of-key adjustment, and a
		 * compressed run spans several levels.  @up_node then is node-at-level
		 * (read by the LE/LT block, carried by the loop).
		 */
		while (level < up_node_lo) {
			struct cds_ft_inode_flag *gp = ft_get_parent_rcu(ft, up_node);

			up_node_lo -= (gp && ft_node_compressed(gp)) ?
				(ssize_t) ft_compressed_node_ptr(gp)->len : 1;
			up_node = gp;
		}
		/*
		 * Derive @up_parent = node at @level-1 (the dispatcher scanned for a
		 * sibling).  If @up_node already covers @level-1 it IS the dispatcher
		 * -- a compressed run (no within-span sibling, skipped by the
		 * !ft_node_internal test below) or a dead-end whose dispatcher we
		 * never descended past.  Otherwise it is @up_node's parent, one depth
		 * shallower (parenthood is by slot).
		 */
		if (up_node_lo <= level - 1) {
			up_parent = up_node;
			up_parent_lo = up_node_lo;
		} else {
			up_parent = ft_get_parent_rcu(ft, up_node);
			up_parent_lo = (up_parent && ft_node_compressed(up_parent)) ?
				level - (ssize_t) ft_compressed_node_ptr(up_parent)->len :
				level - 1;
		}
		for (; level > (ssize_t) iter->prefix_len; level--) {
			uint8_t key_value;

			ft_delay_reader();
			if (!up_first) {
				/*
				 * Climbed one level (level-- since last iter): the old
				 * dispatcher becomes the new node-at-level, and the new
				 * dispatcher is its parent once we cross below its span.
				 */
				up_node = up_parent;
				if (level - 1 < up_parent_lo) {
					struct cds_ft_inode_flag *gp =
						ft_get_parent_rcu(ft, up_parent);

					up_parent_lo -= (gp && ft_node_compressed(gp)) ?
						(ssize_t) ft_compressed_node_ptr(gp)->len : 1;
					up_parent = gp;
				}
			}
			up_first = false;
			/*
			 * Return external node if trying to find LE/LT
			 * inequality and encountering an external node when
			 * going upward.
			 */
			if (going_up && dir == FT_LEFT &&
			    !ft_node_external(up_node)) {
				struct cds_ft_metadata *metadata;

				if (ft_node_compressed(up_node))
					metadata = cds_ft_item_to_metadata(
						ft_node_ptr(up_node));
				else {
					const struct cds_ft_type *type = &ft_types[ft_node_type(up_node)];
					metadata = cds_ft_item_to_metadata_fast(
						ft_node_ptr(up_node),
						type->order);
				}
				{
				struct cds_ft_node *external_nodes = ft_dereference_external(metadata->external_nodes);

				if (external_nodes) {
					int j;

					assert(level <= (int) ft->group->max_key_len);
					iter->key_len = level;
					iter->key_off = 0;
					if (!keep_ordinal)
						ft_speculative_keycopy_unconditional(ft,
							external_nodes, iter_key(iter), level);
					else if (!ft_speculative_keycopy(ft, external_nodes,
							iter_key(iter), level)) {
						for (j = 0; j < level; j++)
							iter_key(iter)[j] = ordinal_key[j];
					}
					iter->node = external_nodes;
					iter->cache_valid = true;
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
			if (!ft_node_internal(up_parent)) {
				FT_TP(ineq_going_up_step, level,
					(const void *) up_parent,
					0, (uint8_t) key_value);
				going_up = true;
				continue;
			}
			node_flag = ft_node_get_leftright(ft, up_parent, key_value,
					keep_ordinal ? &ordinal_key[level - 1] : &ord_scratch,
					dir, true /* validate_lookup */);
	#ifdef FEATURE_FT_SKIP_COMPRESSED
			/*
			 * Fill path only (!use_keycopy): a skip-encoded sibling must be
			 * resolved to fill ordinal_key.  Re-anchor the scanned parent on
			 * the live structure via the child's parent chain and re-scan it
			 * (the single skip concurrency mechanism), rather than spin.
			 * @rewind > 0 (merge) means the parent merged shallower: drop
			 * @level and recompute the dispatch byte at the new level.
			 * ft_skip_reanchor never returns NULL on a well-formed trie.
			 *
			 * Under use_keycopy the skip-encoded sibling is left raw and
			 * followed by the descend_children skip-follow (the leaf copy
			 * supplies the spanned bytes), so no re-anchor is needed here.
			 */
			while (!use_keycopy && node_flag &&
					caa_unlikely(ft_node_skip_compressed(node_flag))) {
				unsigned int rewind;
				struct cds_ft_inode_flag *at_pos;
				struct cds_ft_inode_flag *anchor =
					ft_skip_reanchor(ft, node_flag, &rewind, &at_pos);

				assert(anchor != NULL);
				if (caa_likely(rewind == 0)) {
					/*
					 * Surgical: @at_pos is the live resolved sibling at
					 * this depth that re-scanning @anchor for the
					 * unchanged dispatch byte ordinal_key[level - 1] would
					 * find -- the sibling byte was already recorded by the
					 * ft_node_get_leftright above, so use @at_pos directly
					 * instead of re-scanning.  Keep the path's parent entry
					 * live.
					 */
					node_flag = at_pos;
					break;
				}
				/*
				 * @rewind > 0 (merge): the parent merged shallower; drop
				 * @level and re-scan the live parent at the new level.
				 */
				level -= (ssize_t) rewind;
				/*
				 * The merge rewound @level and re-anchored the live
				 * holder at the new level-1.  Re-sync the parent cursor to
				 * @anchor (an internal node, span 1).  If the re-scan below
				 * finds no sibling, the for-loop's level-- then carries
				 * up_node = anchor = node at the new level, keeping the walk
				 * consistent with the rewound descent position.
				 */
				up_parent = anchor;
				up_parent_lo = level - 1;
				switch (limit) {
				case FT_LOOKUP_LIMIT_NONE:
					key_value = ordinal_key[level - 1];
					break;
				case FT_LOOKUP_LIMIT_FIRST:
					key_value = input_key[level - 1];
					break;
				case FT_LOOKUP_LIMIT_LAST:
					key_value = ((size_t) level <= iter->prefix_len) ?
						input_key[level - 1] : (uint8_t) 0xff;
					break;
				}
				node_flag = ft_node_get_leftright(ft, anchor, key_value,
						&ordinal_key[level - 1], dir,
						true /* validate_lookup */);
			}
	#endif
			if (keep_ordinal)
				dbg_printf("cds_ft_lookup_inequality find sibling from %u at %u finds node_flag %p\n",
						(unsigned int) key_value, (unsigned int) ordinal_key[level - 1],
						node_flag);
			else
				dbg_printf("cds_ft_lookup_inequality find sibling from %u finds node_flag %p\n",
						(unsigned int) key_value, node_flag);
			/* If found left/right sibling, find rightmost/leftmost child. */
			if (node_flag) {
				/* Record the sibling in the path. */
				/*
				 * Seed the cursor at the found sibling (depth @level, its
				 * shallow boundary) before descend_children -> minmax, so a
				 * transiently-empty first minmax step re-enters going-up
				 * with the cursor live.
				 */
				up_node = node_flag;
				up_node_lo = level;
				/*
				 * Reaching a sibling IS backtracking (up to the
				 * parent, then over to a strictly-greater/lesser
				 * sibling subtree), even when the sibling is found on
				 * the first iteration without climbing further.  Mark
				 * @going_up so the minmax descent below does NOT set
				 * skip_eq_external_nodes: the sibling subtree's first
				 * external_nodes is a different (strictly greater for
				 * GT) prefix key, not the search key's equal match, so
				 * it must not be skipped.  Without this, GT into an
				 * immediate sibling whose subtree root is a prefix key
				 * (e.g. next("y") with keys "z" < "zz1") skips that
				 * prefix key entirely.
				 */
				going_up = true;
				if (keep_ordinal)
					FT_TP(ineq_going_up_step, level,
						(const void *) up_parent,
						1, ordinal_key[level - 1]);
				else
					FT_TP(ineq_going_up_step_nokey, level,
						(const void *) up_parent, 1);
				break;
			}
			FT_TP(ineq_going_up_step, level,
				(const void *) up_parent,
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
				/*
				 * Node at prefix_len is the going-up cursor's @up_parent.
				 * This block is reached only via normal loop exit (no
				 * sibling found while backtracking down to prefix_len),
				 * whose last iteration ran at level == prefix_len + 1 with
				 * the invariant up_parent == node at level - 1 ==
				 * node at prefix_len.  Obtained via a live back-pointer
				 * read.
				 */
				struct cds_ft_inode_flag *pfx_flag = up_parent;

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
						ft_dereference_external(metadata->external_nodes);

					if (external_nodes) {
						int j;

						iter->key_len = iter->prefix_len;
						iter->key_off = 0;
						if (!keep_ordinal)
							ft_speculative_keycopy_unconditional(ft,
								external_nodes, iter_key(iter),
								(ssize_t) iter->prefix_len);
						else if (!ft_speculative_keycopy(ft, external_nodes,
								iter_key(iter),
								(ssize_t) iter->prefix_len)) {
							for (j = 0; j < (int) iter->prefix_len; j++)
								iter_key(iter)[j] = ordinal_key[j];
						}
						iter->node = external_nodes;
						iter->cache_valid = true;
						iter_debug_path_update(iter);
						iter->path_len = iter->prefix_len + 1;
						iter->status = CDS_FT_STATUS_OK;
						goto end;
					}
				}
			}
			iter->node = NULL;
			iter->cache_valid = true;
			iter_debug_path_update(iter);
			iter->path_len = iter->prefix_len + 1;
			iter->status = CDS_FT_STATUS_NOT_FOUND;
			goto end;
		}
	}

	/*
	 * Fall-through dead-end within the scope prefix: the descent broke
	 * on an empty slot at level <= prefix_len (the prefix path itself
	 * does not exist), so the going-up loop above -- bounded at
	 * prefix_len -- never ran and the going_up NOT_FOUND arm did not
	 * trigger.  descend_children below would misread the NULL (tag 0)
	 * as an external-node match and return OK with iter->node == NULL.
	 * No key with the scope prefix exists: report NOT_FOUND.  Every
	 * `goto descend_children` jumps past this guard with a non-NULL
	 * @node_flag; only the fall-through path can carry NULL.
	 */
	if (!node_flag) {
		iter->node = NULL;
		iter->cache_valid = true;
		iter_debug_path_update(iter);
		iter->path_len = iter->prefix_len + 1;
		iter->status = CDS_FT_STATUS_NOT_FOUND;
		goto end;
	}

descend_children:
	/*
	 * A skip-encoded sibling carried over from going_up (use_keycopy) is
	 * resolved by the loop's skip-follow below, not tested as external here
	 * -- its tag bits would otherwise be misread as an external node.
	 */
	if (!(use_keycopy && ft_node_skip_compressed(node_flag))
			&& ft_node_external(node_flag)) {
		int j;

		assert(level <= (int) ft->group->max_key_len);
		iter->key_len = level;
		iter->key_off = 0;
		if (!keep_ordinal)
			ft_speculative_keycopy_unconditional(ft,
				(const struct cds_ft_node *) ft_node_ptr(node_flag),
				iter_key(iter), level);
		else if (!ft_speculative_keycopy(ft,
				(const struct cds_ft_node *) ft_node_ptr(node_flag),
				iter_key(iter), level)) {
			for (j = 0; j < level; j++)
				iter_key(iter)[j] = ordinal_key[j];
		}
		iter->node = (struct cds_ft_node *) ft_node_ptr(node_flag);
		iter->cache_valid = true;
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
#ifdef FEATURE_FT_SKIP_COMPRESSED
		/*
		 * Skip-encoded compressed child: follow it directly without
		 * reading the compressed node.  The spanned length is in the
		 * pointer's upper bits (ft_skip_len == cn->len), so this
		 * reproduces ft_inequality_minmax_compressed's level advance
		 * exactly; the only thing dropped is the ordinal_key span fill,
		 * which the leaf-key copy supplies for the result.  A
		 * skip-compressed node cannot carry external_nodes (a terminating
		 * fork the encoding cannot express), so nothing terminates inside
		 * the span -- no min/max candidate is skipped.  Resolving here, at
		 * the loop top, also keeps the external / internal tests below
		 * from misreading the skip pointer's tag bits.
		 */
		if (use_keycopy && caa_unlikely(ft_node_skip_compressed(node_flag))) {
			level += (ssize_t) ft_skip_len(node_flag) - 1;
			node_flag = ft_skip_child_ptr(node_flag);
			skip_eq_external_nodes = false;
			up_node = node_flag;	/* live child below the span */
			up_node_lo = level;
			/*
			 * Mirror ft_inequality_minmax_compressed: an external child
			 * is the leaf at the span end -- break with @level already at
			 * the key length (no loop level++); an internal child
			 * continues the descent (the loop level++ lands on it).
			 */
			if (ft_node_external(node_flag))
				break;
			continue;
		}
#endif
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
			struct cds_ft_node *external_nodes = ft_dereference_external(metadata->external_nodes);

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
		 * Resolve a skip-encoded node to its compressed form.  No-op under
		 * use_keycopy (skip pointers were already followed at the loop
		 * top); on the fill path it converts a skip pointer so the
		 * compressed handler can read cn->key_bytes.
		 */
		node_flag = ft_resolve_skip_compressed(ft, node_flag);
		/*
		 * Compressed node: traverse the compressed path to reach the
		 * child, filling ordinal_key.  Under use_keycopy this is only the
		 * regular (non-skip-encoded) form for spans longer than
		 * FT_SKIP_LEN_MAX, and the fill is harmless (the result key comes
		 * from the leaf copy).
		 */
		if (ft_node_compressed(node_flag)) {
			enum ft_descent_action act;

			act = ft_inequality_minmax_compressed(
				&node_flag, &level, &ret_node,
				&skip_eq_external_nodes,
				ordinal_key, dir, keep_ordinal);
			if (act == FT_DESCENT_FOUND_MINMAX)
				goto found_minmax;
			if (act == FT_DESCENT_BREAK)
				break;
			assert(act == FT_DESCENT_CONTINUE);
			up_node = node_flag;	/* minmax cn->child at the advanced @level */
			up_node_lo = level;
			continue;
		}
		skip_eq_external_nodes = false;
		node_flag = ft_node_get_minmax(ft, node_flag,
				keep_ordinal ? &ordinal_key[level - 1] : &ord_scratch, dir,
				true /* validate_lookup */);
		/*
		 * Prefetch the min/max child's body for the next iteration's scan.
		 * ft_maybe_prefetch fires only on internal/compressed children
		 * (external/NULL/non-canonical-skip are dropped), so it never
		 * reintroduces the harmful external-leaf prefetch.  The lead comes
		 * from the GE/GT external-nodes metadata read at the loop top, which
		 * is enough to overlap the body miss: measured ~+3-4% on
		 * single-thread cds_ft_next over the 1M-key DNS set (the going-up
		 * sibling scan has no such lead and did NOT benefit -- not added).
		 */
		ft_maybe_prefetch(node_flag);
#ifdef FEATURE_FT_SKIP_COMPRESSED
		if (node_flag && caa_unlikely(ft_node_skip_compressed(node_flag))) {
			if (use_keycopy) {
				/*
				 * Leaf-copy path: a skip-encoded min/max child is returned
				 * raw and followed at the loop top next iteration -- the
				 * key drives the descent and the leaf copy supplies the
				 * spanned bytes, so no re-anchor.
				 */
				up_node = node_flag;
				up_node_lo = level;
				continue;
			}
			{
				/*
				 * Fill path (no leaf-key offset): ordinal_key must be
				 * filled from the live compressed node, so re-anchor the
				 * scanned node on the live structure via the child's parent
				 * chain and re-scan, rather than spin.  @rewind > 0 (merge)
				 * re-anchors shallower; @level -= rewind + 1 then the loop's
				 * level++ nets a -rewind step.  ft_skip_reanchor never
				 * returns NULL on a well-formed trie.
				 */
				unsigned int rewind;
				struct cds_ft_inode_flag *at_pos;
				struct cds_ft_inode_flag *anchor =
					ft_skip_reanchor(ft, node_flag, &rewind, &at_pos);

				assert(anchor != NULL);
				if (caa_likely(rewind == 0)) {
					node_flag = at_pos;
				} else {
					level -= (ssize_t) rewind + 1;
					node_flag = anchor;
					up_node = anchor;
					up_node_lo = level;
					continue;
				}
			}
		}
#endif
		/*
		 * Transiently empty internal node (popcount/pigeon): a reader may
		 * observe every slot of a reachable internal node as
		 * NULL in the narrow window between a writer's per-slot
		 * detach and the upward walk's slot-replace at a higher
		 * ancestor.  Quiescently, reachable internal nodes have
		 * nr_child >= 1 (the upward walk prunes single-child
		 * chains wholesale and replaces at the first multi-child
		 * ancestor, so a slot-emptied internal is never left in
		 * place).  Treat as "empty at this step" and let
		 * going_up find the next sibling at a higher level.
		 * Back level one step so the going-up cursor resumes at
		 * the parent.
		 */
		if (caa_unlikely(!node_flag)) {
			level--;
			going_up = true;
			goto going_up;
		}
		up_node = node_flag;		/* minmax child established at @level */
		up_node_lo = level;
		if (keep_ordinal)
			dbg_printf("cds_ft_lookup_inequality find minmax at %u finds node_flag %p\n",
					(unsigned int) ordinal_key[level - 1], node_flag);
		else
			dbg_printf("cds_ft_lookup_inequality find minmax finds node_flag %p\n",
					node_flag);
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
		iter->key_off = 0;
		if (!keep_ordinal)
			ft_speculative_keycopy_unconditional(ft, ret_node,
				iter_key(iter), level);
		else if (!ft_speculative_keycopy(ft, ret_node, iter_key(iter),
				level)) {
			for (j = 0; j < level; j++)
				iter_key(iter)[j] = ordinal_key[j];
		}
		iter->node = ret_node;
		iter->cache_valid = true;
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
	iter_auto_invalidate_cache(iter);
	return iter->status;
}

/*
 * Dispatcher: resolve the immutable @use_keycopy config (a speculative
 * skip-compressed group with a leaf-key offset) once, then tail-call the
 * matching always_inline instantiation so each variant's hot loop has the
 * per-call use_keycopy gates constant-folded away.  Mirrors the
 * do_cds_ft_lookup_inner (descend_cand, skip_compressed) specialization.
 * The config is immutable after group create, so this branch is perfectly
 * predicted and amortized over a full traversal.
 *
 * inline_lookup (force-inline) into each entry point: cds_ft_lookup_le/ge/lt/gt
 * pass compile-time-constant @mode and @limit (and cds_ft_next/prev resolve to
 * _gt/_lt), so inlining lets @mode AND @limit -- not just @use_keycopy -- DCE
 * each wrapper down to its single arm.  The six specialized bodies cost .so size
 * but the HOT footprint shrinks (a workload runs one DCE'd wrapper, smaller than
 * the shared generic): measured ~+10% ST and +4-22% MT on 1M-key DNS iterate.
 */
static inline_lookup
enum cds_ft_status cds_ft_lookup_inequality(struct cds_ft *ft,
		struct cds_ft_iter *iter,
		enum ft_lookup_inequality mode,
		enum ft_lookup_limit limit)
{
	if (ft->group->speculative_key_offset_set && ft->group->speculative &&
			(ft->group->flags & CDS_FT_FLAG_SKIP_COMPRESSED))
		return cds_ft_lookup_inequality_impl(ft, iter, mode, limit, true, false);
	return cds_ft_lookup_inequality_impl(ft, iter, mode, limit, false, false);
}

/*
 * Limit-none relational specializations, installed as fn-pointers by
 * ft_install_lookup_ops.  Each bakes its mode, LIMIT_NONE, and use_keycopy
 * into the force-inline cds_ft_lookup_inequality_impl so the body DCEs to a
 * single arm; referenced only via the installed pointer, so they stay real
 * out-of-line symbols and the public entry is a bare indirect tail-call.  The
 * RCU read lock is taken here (mirroring the lookup-API inners).
 */
#define FT_INEQ_SPEC(name, mode, kc)					\
	static enum cds_ft_status name(struct cds_ft *ft,		\
			struct cds_ft_iter *iter)			\
	{								\
		CDS_FT_SCOPED_READER(ft);				\
		return cds_ft_lookup_inequality_impl(ft, iter,		\
				(mode), FT_LOOKUP_LIMIT_NONE, (kc), false); \
	}
FT_INEQ_SPEC(ft_ineq_le_keycopy, FT_LOOKUP_LE, true)
FT_INEQ_SPEC(ft_ineq_le_eager,   FT_LOOKUP_LE, false)
FT_INEQ_SPEC(ft_ineq_ge_keycopy, FT_LOOKUP_GE, true)
FT_INEQ_SPEC(ft_ineq_ge_eager,   FT_LOOKUP_GE, false)
FT_INEQ_SPEC(ft_ineq_lt_keycopy, FT_LOOKUP_LT, true)
FT_INEQ_SPEC(ft_ineq_lt_eager,   FT_LOOKUP_LT, false)
FT_INEQ_SPEC(ft_ineq_gt_keycopy, FT_LOOKUP_GT, true)
FT_INEQ_SPEC(ft_ineq_gt_eager,   FT_LOOKUP_GT, false)
#undef FT_INEQ_SPEC

/*
 * Iterator-based inequality lookup public API.
 * The caller sets the key via cds_ft_iter_set_key() before calling.
 * On return the iterator holds the result key, key length, node, path,
 * and status.
 */
enum cds_ft_status cds_ft_lookup_le(struct cds_ft *ft,
		struct cds_ft_iter *iter)
{
	ft_iter_assert_bound(ft, iter);
	return (*ft->lookup_le_fn)(ft, iter);
}

enum cds_ft_status cds_ft_lookup_ge(struct cds_ft *ft,
		struct cds_ft_iter *iter)
{
	ft_iter_assert_bound(ft, iter);
	return (*ft->lookup_ge_fn)(ft, iter);
}

enum cds_ft_status cds_ft_lookup_lt(struct cds_ft *ft,
		struct cds_ft_iter *iter)
{
	ft_iter_assert_bound(ft, iter);
	return (*ft->lookup_lt_fn)(ft, iter);
}

enum cds_ft_status cds_ft_lookup_gt(struct cds_ft *ft,
		struct cds_ft_iter *iter)
{
	ft_iter_assert_bound(ft, iter);
	return (*ft->lookup_gt_fn)(ft, iter);
}

enum cds_ft_status cds_ft_lookup_first(struct cds_ft *ft,
		struct cds_ft_iter *iter)
{
	size_t saved_key_len = iter->key_len;
	enum cds_ft_status status;

	ft_iter_assert_bound(ft, iter);
	CDS_FT_SCOPED_READER(ft);
	dbg_printf("cds_ft_lookup_first\n");
	/* O(1) endpoint: the ordinal-cell list's minimum cell (unscoped only).
	 * Resolve a flip proxy: head/tail transition atomically with the
	 * neighbour edges during a concurrent splice/unsplice/run move.
	 * iter_auto_invalidate_cache: honour the UNCACHED contract like every
	 * other epilogue -- materialize the lazy leaf-referenced key and drop
	 * the cached position so it is not reused across a lock window. */
	if (ft_ord_cell_fastpath_ok(ft, iter)) {
		status = ft_ord_cell_iter_land(ft, iter,
			ft_ord_cell_resolve_ord(&ft->ord_cell_head));
		iter_auto_invalidate_cache(iter);
		return status;
	}
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
	if (status != CDS_FT_STATUS_OK)
		iter->key_len = saved_key_len;
	return status;
}

enum cds_ft_status cds_ft_lookup_last(struct cds_ft *ft,
		struct cds_ft_iter *iter)
{
	size_t saved_key_len = iter->key_len;
	enum cds_ft_status status;

	ft_iter_assert_bound(ft, iter);
	CDS_FT_SCOPED_READER(ft);
	dbg_printf("cds_ft_lookup_last\n");
	/* O(1) endpoint: the ordinal-cell list's maximum cell (unscoped only).
	 * Resolve a flip proxy (see cds_ft_lookup_first, incl. the UNCACHED
	 * auto-invalidate rationale). */
	if (ft_ord_cell_fastpath_ok(ft, iter)) {
		status = ft_ord_cell_iter_land(ft, iter,
			ft_ord_cell_resolve_ord(&ft->ord_cell_tail));
		iter_auto_invalidate_cache(iter);
		return status;
	}
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
	if (status != CDS_FT_STATUS_OK)
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
 * One-commit insert state (ordered-list fresh-head insert): the attach
 * machinery parks a flip proxy in the structural slot (resolving to the OLD
 * value, so the key stays invisible) and records the settle information here;
 * insert_done then adds the ordered-list neighbour edges to the SAME batch and
 * makes the key reachable in the structural index AND spliced into the cell
 * list with one urcu_flip_commit -- a reader can never observe the fresh head
 * without its cell in the list (2026-06 review, 2.13).  @batch == NULL: legacy
 * direct publish (ordered list off, or a shape not yet converted).
 */
struct ft_insert_commit {
	struct ft_flip_batch *batch;		/* armed at the publish site */
	struct cds_ft_inode_flag **slot;	/* parked slot (settle target) */
	struct cds_ft_inode_flag *slot_value;	/* canonical value to settle */
	struct cds_ft_inode_flag *parent_nf;	/* @slot's owner (publish settle) */
	/*
	 * Count-propagation base for the post-commit +1 (the key only counts
	 * once reachable): the deepest node whose nr_keys must reflect the
	 * fresh key.  NULL = the caller's *d.pnfp.
	 */
	struct cds_ft_inode_flag *count_from;
	/*
	 * Old compressed node replaced by the parked publish: readers keep
	 * resolving the proxy to it until the commit, so its (grace-period-
	 * deferred) free must be queued only AFTER the commit -- a free queued
	 * pre-commit would not cover readers that pick the proxy up later.
	 */
	struct cds_ft_compressed_node *free_old_cn;
	bool publish_to_parent;			/* settle via ft_publish_to_parent */
	bool spliced;				/* cell already spliced (B-lite shape) */
};

/* Flip-batch helpers (defined with the flip machinery, after the readers). */
struct ft_flip_batch;
static struct ft_flip_batch *ft_flip_batch_alloc(struct cds_ft *ft,
		unsigned int cap);
static struct cds_ft_inode_flag *ft_flip_batch_add(struct ft_flip_batch *b,
		struct cds_ft_inode_flag *old_nf,
		struct cds_ft_inode_flag *new_nf);
static void ft_flip_batch_free_unpublished(struct ft_flip_batch *b);
static void ft_flip_batch_commit(struct ft_flip_batch *b);
static void ft_flip_batch_reclaim(struct ft_flip_batch *b);
static void ft_insert_one_commit(struct cds_ft *ft, const uint8_t *key,
		size_t key_len, struct ft_ord_cell *cell,
		struct ft_insert_commit *ic);

/*
 * Publish @new_top into @slot (owned by @parent_nf): direct via
 * ft_publish_to_parent, or -- one-commit insert, @ic armed -- park a flip
 * proxy that keeps resolving to the old slot value until insert_done's single
 * commit, recording the settle (which then runs the real ft_publish_to_parent,
 * including its dual skip-slot maintenance).  The caller must have wired
 * @new_top's parent back-pointer already (parent-before-publish; with a parked
 * proxy the cluster only becomes reachable at the commit, by which time the
 * wiring is complete either way).
 */
static
void ft_insert_publish_or_park(struct cds_ft *ft,
		struct cds_ft_inode_flag *parent_nf,
		struct cds_ft_inode_flag **slot,
		struct cds_ft_inode_flag *new_top,
		struct ft_insert_commit *ic)
{
	if (ic && ic->batch) {
		rcu_assign_pointer(*slot,
			ft_flip_batch_add(ic->batch, *slot, new_top));
		ic->slot = slot;
		ic->slot_value = new_top;
		ic->parent_nf = parent_nf;
		ic->publish_to_parent = true;
		return;
	}
	ft_publish_to_parent(ft, parent_nf, slot, new_top);
}

/*
 * Arm the one-commit batch just before a fresh-head publish (fallible; the
 * caller's error unwind runs with nothing published).  No-op when the ordered
 * list is off or no @ic is threaded (insert_replace, bulk builders).  Returns
 * 0, or -ENOMEM.
 */
static
int ft_insert_commit_arm(struct cds_ft *ft, struct ft_insert_commit *ic)
{
	if (!ic || !ft->ordered_list)
		return 0;
	ic->batch = ft_flip_batch_alloc(ft, 5);
	if (!ic->batch)
		return -ENOMEM;
	return 0;
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
		unsigned int node_depth,	/* depth of the compressed node */
		struct ft_insert_commit *ic)	/* one-commit insert, may be NULL */
{
	struct cds_ft_compressed_node *cn = ft_compressed_node_ptr(compressed_flag);
	struct cds_ft_metadata *cn_meta =
		cds_ft_item_to_metadata((struct cds_ft_inode *) cn);

	/*
	 * Compressed metadata never carries external_nodes
	 * (ft_metadata_set_external_nodes aborts on a compressed target).
	 * The prefix builders below rely on it; the dead external-carrying
	 * arms they used to carry hid latent bugs (wrong nr_keys, a dropped
	 * prefix byte) that this assert retires.
	 */
	assert(!cn_meta->external_nodes);
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
	/*
	 * Two-phase publish: build the cluster invisibly, then wire the
	 * deferred back-pointers and swing the parent slot, at the end.
	 * See the rcu-mutation build-invisible pattern.
	 *
	 * suffix_len >= 1: the branch's children (suffix, new) are new cluster
	 * nodes — set their back-pointers normally; only the live old child
	 * into the new suffix (cn->child -> sfx) is deferred (deferred edge 1).
	 * suffix_len == 0: the branch is a cluster-leaf (old direction is the
	 * live cn->child, new direction the new subtree); set_nth defers BOTH
	 * its children, wired here at publish to the final branch_flag (deferred
	 * edges 1 and 2).
	 */
	struct cds_ft_inode_flag *sfx_skip_flag = NULL;
	struct cds_ft_inode_flag *deferred_child = NULL;
	struct cds_ft_inode_flag *deferred_parent = NULL;
	struct cds_ft_inode_flag **deferred_slot = NULL;
	struct cds_ft_inode_flag *deferred_child2 = NULL;
	struct cds_ft_inode_flag **deferred_slot2 = NULL;
	bool branch_cluster_leaf = (suffix_len == 0);

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
		old_suffix_flag = ft_compressed_node_flag(sfx);	/* PLAIN: install + recover sfx directly */
		sfx_skip_flag = ft_publish_compressed(ft, sfx, old_suffix_flag);	/* skip form for the slot */
		created[nr_created++] = old_suffix_flag;	/* track PLAIN so the error path frees sfx directly */
		/*
		 * Defer re-parenting the live old child to publish: writing
		 * cn->child's back-pointer now would make this unpublished
		 * cluster observable from the bottom (and the branch install
		 * below would otherwise recover sfx through it).  sfx->child
		 * already points at cn->child (a write into the new sfx only).
		 */
		deferred_child = cn->child;
		deferred_parent = old_suffix_flag;
		deferred_slot = &sfx->child;
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
		ft_set_parent(ft, nb->child, new_branch_flag, NULL);
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
				junction_depth, branch_cluster_leaf);
		if (ret) goto error;
		created[nr_created++] = dest;
		branch_flag = dest;

		/* Second child: new direction. */
		{
			struct cds_ft_inode *old_recompacted = NULL;

			branch_meta = cds_ft_item_to_metadata(ft_node_ptr(dest));
			ret = ft_node_set_nth(ft, &dest, new_ordinal, new_branch_flag,
					&old_recompacted, branch_meta, junction_depth,
					branch_cluster_leaf);
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

		/*
		 * sfx was installed via its PLAIN flag so ft_set_parent could
		 * recover it directly (without reading cn->child's back-pointer,
		 * which still points at the old cn).  Re-encode the old-direction
		 * slot to sfx's skip form now — a value write into the still-
		 * unpublished branch; it becomes recoverable once cn->child's
		 * back-pointer is set at publish.
		 */
		if (suffix_len >= 1 && sfx_skip_flag != old_suffix_flag) {
			struct cds_ft_inode_flag **oslot = NULL;

			ft_node_get_nth_skip(branch_flag, &oslot, old_ordinal,
					FT_PF_NONE);
			if (oslot)
				rcu_assign_pointer(*oslot, sfx_skip_flag);
		}

		/*
		 * suffix_len == 0: the branch is a cluster-leaf, so set_nth left
		 * both of its children unparented.  Record both deferred edges
		 * (old = live cn->child, new = the new subtree) against the now-
		 * final branch_flag; phase 2 wires them.  The forward slots are
		 * already correct (value-copied by set_nth / recompaction).
		 */
		if (branch_cluster_leaf) {
			ft_node_get_nth_skip(branch_flag, &deferred_slot,
					old_ordinal, FT_PF_NONE);
			ft_node_get_nth_skip(branch_flag, &deferred_slot2,
					new_ordinal, FT_PF_NONE);
			deferred_child = cn->child;
			deferred_parent = branch_flag;
			deferred_child2 = new_branch_flag;
		}
	}

	/*
	 * 4. Build prefix -> branch (if needed).  @cn carries no
	 * external_nodes (asserted at entry), so the prefix is always a
	 * plain compressed run over key_bytes[0 .. diverge_pos).
	 */
	if (diverge_pos >= 1) {
		struct cds_ft_compressed_node *pfx;
		struct cds_ft_metadata *pfx_meta;
		struct cds_ft_inode_flag *pfx_child;

		pfx = alloc_compressed_node(ft, diverge_pos, &pfx_meta);
		if (!pfx) goto error;
		pfx->child = branch_flag;
		pfx->len = diverge_pos;
		memcpy(pfx->key_bytes, cn->key_bytes, diverge_pos);
		pfx_meta->nr_child = 1;
		ft_nr_keys_store(pfx_meta, ft_nr_keys_get(cn_meta) + 1, CMM_RELAXED);
		pfx_child = ft_compressed_node_flag(pfx);
		ft_set_parent(ft, branch_flag, pfx_child, &pfx->child);
		pfx_child = ft_publish_compressed(ft, pfx, pfx_child);
		created[nr_created++] = pfx_child;
		top_flag = pfx_child;
	} else {
		/* diverge_pos == 0: branch IS the top. */
		struct cds_ft_metadata *branch_meta =
			cds_ft_item_to_metadata(ft_node_ptr(branch_flag));
		ft_nr_keys_store(branch_meta, ft_nr_keys_get(cn_meta) + 1,
				CMM_RELAXED);
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
	/*
	 * Phase 2 (publish) — no failures past here.  Wire every back-pointer
	 * before swinging the parent's forward slot, so an up-walk that lands
	 * on the new cluster from either direction sees the back-pointers wired
	 * before the cluster becomes reader-reachable.
	 *
	 * ORDER among the deferred edges matters.  deferred_child is always the
	 * LIVE old child (cn->child) re-parented into the cluster; setting its
	 * back-pointer is itself a back-channel publish — a reader up-walking
	 * from cn->child immediately enters the new cluster and can then scan
	 * the cluster's other (sibling) slots.  deferred_child2 is the FRESH
	 * new subtree, observable only through the cluster.  So wire the cluster
	 * top's own back-pointer and the fresh edge FIRST, and the live edge
	 * LAST: otherwise a reader entering via the live child reads a sibling
	 * slot pointing at the fresh subtree whose parent is not yet set, and
	 * its consume chain — anchored at the live back-pointer store — has no
	 * happens-before edge to the later fresh-parent store, so it observes a
	 * stale NULL parent (ft_skip_reanchor holder == NULL).
	 *
	 * suffix_len >= 1: only the live edge exists (cn->child -> sfx).
	 * suffix_len == 0: cluster-leaf branch's two children (live cn->child
	 * and fresh new subtree), both -> branch_flag.
	 */
	ret = ft_insert_commit_arm(ft, ic);
	if (ret)
		goto error;
	ft_set_parent(ft, top_flag, cn_meta->parent, parent_slot);
	if (deferred_child2)
		ft_set_parent(ft, deferred_child2, deferred_parent, deferred_slot2);
	if (deferred_child)
		ft_set_parent(ft, deferred_child, deferred_parent, deferred_slot);
	ft_insert_publish_or_park(ft, cn_meta->parent, parent_slot, top_flag, ic);

	/*
	 * 7. Free the old compressed node.  Parked publish: readers resolve
	 * the proxy to @cn until the commit, so defer the free past it.
	 */
	if (ic && ic->slot)
		ic->free_old_cn = cn;
	else
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
					ft_skip_to_compressed(ft, created[i]));
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
 * @parent_slot: address of the slot in cn's parent that holds cn.  Used
 * to wire top_flag's own back-pointer into the live parent BEFORE the
 * deferred (back-channel) re-parent of cn->child into the new suffix.
 * Otherwise an up-walk from cn->child enters the new cluster and walks
 * up to top_flag, which would have parent == NULL.
 *
 * On success, sets *top_ret to the topmost node (prefix or junction)
 * and *jct_ret to the junction node.  Returns 0.
 * On failure, frees any partially created nodes and returns -ENOMEM.
 */
static
int ft_split_compressed_key_shorter(struct cds_ft *ft,
		struct cds_ft_inode_flag *compressed_flag,
		struct cds_ft_inode_flag **parent_slot,
		unsigned int remaining,
		struct cds_ft_inode_flag **top_ret,
		struct cds_ft_inode_flag **jct_ret,
		unsigned int node_depth)
{
	struct cds_ft_compressed_node *cn = ft_compressed_node_ptr(compressed_flag);
	struct cds_ft_metadata *cn_meta =
		cds_ft_item_to_metadata((struct cds_ft_inode *) cn);
	unsigned int suffix_len = cn->len - remaining - 1;

	/* Compressed metadata never carries external_nodes (see
	 * ft_split_compressed_insert); the prefix builders rely on it. */
	assert(!cn_meta->external_nodes);
	struct cds_ft_inode_flag *suffix_flag;
	struct cds_ft_inode_flag *jct_flag;
	struct cds_ft_inode_flag *top_flag;
	struct cds_ft_inode_flag *created[FT_MAX_DEPTH];
	int nr_created = 0;
	unsigned long child_nr_keys;
	int ret;
	/*
	 * Build-invisible / publish / reclaim (see rcu-mutation pattern).
	 * The live old child (cn->child) is re-parented into the new suffix or,
	 * for suffix_len == 0, straight into the junction.  Defer that single
	 * back-pointer to the failure-free tail so a later allocation failure
	 * frees the never-observed cluster with cn->child untouched.  The caller
	 * publishes the top right after we return, so this deferred (bottom)
	 * publish precedes the top forward publish.
	 */
	uint8_t jct_ordinal = cn->key_bytes[remaining];
	bool jct_cluster_leaf = (suffix_len == 0);
	struct cds_ft_inode_flag *sfx_skip_flag = NULL;
	struct cds_ft_inode_flag *deferred_child = NULL;
	struct cds_ft_inode_flag *deferred_parent = NULL;
	struct cds_ft_inode_flag **deferred_slot = NULL;

	if (!ft_node_external(cn->child)) {
		struct cds_ft_metadata *cm =
			cds_ft_item_to_metadata(ft_node_ptr(cn->child));
		child_nr_keys = ft_nr_keys_get(cm);
	} else if (cn->child) {
		child_nr_keys = 1;
	} else {
		child_nr_keys = 0;
	}

	/*
	 * Build suffix → old child.
	 *
	 * Under SKIP_COMPRESSED, suffix_len >= 1 must produce a
	 * compressed (skip-encoded) node; a 1-child internal at this
	 * level would violate the chain-compress invariant (verify
	 * rejects it).  Under non-SC mode, suffix_len == 1 historically
	 * produced an internal node — that's still acceptable since the
	 * invariant only applies in skip mode.
	 */
	if (suffix_len >= 2
#ifdef FEATURE_FT_SKIP_COMPRESSED
			|| (suffix_len == 1 && ft_group_skip_compressed(ft->group))
#endif
	   ) {
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
		suffix_flag = ft_compressed_node_flag(sfx);	/* PLAIN: install + recover sfx directly */
		sfx_skip_flag = ft_publish_compressed(ft, sfx, suffix_flag);	/* skip form for the slot */
		created[nr_created++] = suffix_flag;	/* track PLAIN so the error path frees sfx directly */
		/*
		 * Defer re-parenting the live old child into the new suffix:
		 * writing cn->child's back-pointer now would expose the
		 * unpublished cluster from below, and the junction install
		 * below would recover sfx through it.  sfx->child already
		 * points at cn->child (a write into the new sfx only).
		 */
		deferred_child = cn->child;
		deferred_parent = suffix_flag;	/* PLAIN sfx flag */
		deferred_slot = &sfx->child;
	} else if (suffix_len == 1) {
		struct cds_ft_inode_flag *dest = NULL;

		/*
		 * 1-child internal suffix (non-SC): the live cn->child is its
		 * only child, so this node is a cluster-leaf — defer cn->child's
		 * back-pointer.
		 */
		ret = ft_node_set_nth(ft, &dest,
			cn->key_bytes[remaining + 1],
			cn->child, NULL, NULL,
			node_depth + remaining + 1, true);
		if (ret) goto error;
		{
			struct cds_ft_metadata *m =
				cds_ft_item_to_metadata(ft_node_ptr(dest));
			ft_nr_keys_store(m, child_nr_keys,
				CMM_RELAXED);
		}
		suffix_flag = dest;
		created[nr_created++] = dest;
		deferred_child = cn->child;
		deferred_parent = dest;
		ft_node_get_nth_skip(dest, &deferred_slot,
			cn->key_bytes[remaining + 1], FT_PF_NONE);
	} else {
		suffix_flag = cn->child;
	}

	/* Junction: internal node with suffix child. */
	{
		struct cds_ft_inode_flag *dest = NULL;
		struct cds_ft_metadata *jct_meta;

		ret = ft_node_set_nth(ft, &dest,
			jct_ordinal,
			suffix_flag, NULL, NULL,
			node_depth + remaining, jct_cluster_leaf);
		if (ret) goto error;
		jct_meta = cds_ft_item_to_metadata(ft_node_ptr(dest));
		ft_nr_keys_store(jct_meta, child_nr_keys,
			CMM_RELAXED);
		jct_flag = dest;
		created[nr_created++] = dest;

		if (jct_cluster_leaf) {
			/*
			 * suffix_len == 0: the junction is the cluster-leaf and
			 * its suffix-direction child is the live cn->child.
			 * Record the deferred edge against the final junction.
			 */
			deferred_child = cn->child;
			deferred_parent = jct_flag;
			ft_node_get_nth_skip(jct_flag, &deferred_slot,
				jct_ordinal, FT_PF_NONE);
		} else if (sfx_skip_flag && sfx_skip_flag != suffix_flag) {
			/*
			 * suffix_len >= 1 compressed: sfx was installed via its
			 * PLAIN flag so ft_set_parent recovered it directly.
			 * Re-encode the junction's slot to sfx's skip form — a
			 * value write into the still-unpublished junction; it
			 * resolves once cn->child's back-pointer is set at the tail.
			 */
			struct cds_ft_inode_flag **oslot = NULL;

			ft_node_get_nth_skip(jct_flag, &oslot, jct_ordinal,
				FT_PF_NONE);
			if (oslot)
				rcu_assign_pointer(*oslot, sfx_skip_flag);
		}
	}

	/* Prefix → junction (no external_nodes on @cn: asserted at entry). */
	if (remaining >= 2) {
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
		ft_set_parent(ft, jct_flag, top_flag, NULL);
		top_flag = ft_publish_compressed(ft, pfx, top_flag);
		created[nr_created++] = top_flag;
	} else if (remaining == 1) {
#ifdef FEATURE_FT_SKIP_COMPRESSED
		if (ft_group_skip_compressed(ft->group)) {
			/*
			 * 1-byte prefix: emit a 1-byte compressed instead of a
			 * 1-child internal (canonical form under
			 * SKIP_COMPRESSED).
			 */
			struct cds_ft_compressed_node *pfx;
			struct cds_ft_metadata *pfx_meta;

			pfx = alloc_compressed_node(ft, 1, &pfx_meta);
			if (!pfx) goto error;
			pfx->child = jct_flag;
			pfx->len = 1;
			pfx->key_bytes[0] = cn->key_bytes[0];
			pfx_meta->nr_child = 1;
			ft_nr_keys_store(pfx_meta, ft_nr_keys_get(cn_meta),
				CMM_RELAXED);
			top_flag = ft_compressed_node_flag(pfx);
			ft_set_parent(ft, jct_flag, top_flag, &pfx->child);
			top_flag = ft_publish_compressed(ft, pfx, top_flag);
			created[nr_created++] = top_flag;
		} else
#endif
		{
			struct cds_ft_inode_flag *dest = NULL;
			struct cds_ft_metadata *pfx_meta;

			ret = ft_node_set_nth(ft, &dest, cn->key_bytes[0],
				jct_flag, NULL, NULL, node_depth, false);
			if (ret) goto error;
			pfx_meta = cds_ft_item_to_metadata(ft_node_ptr(dest));
			ft_nr_keys_store(pfx_meta, ft_nr_keys_get(cn_meta),
				CMM_RELAXED);
			top_flag = dest;
			created[nr_created++] = dest;
		}
	} else {
		/* remaining == 0: no prefix, junction IS the top. */
		top_flag = jct_flag;
	}

	/*
	 * Failure-free tail.  First wire top_flag's own back-pointer into
	 * cn's live parent (so an up-walk that enters the cluster via the
	 * deferred back-channel below finds a parent-wired top), THEN wire
	 * the single deferred back-pointer (the live old child into the new
	 * suffix / junction).  No allocation happens past here; the caller
	 * publishes the top forward immediately after we return.
	 */
	ft_set_parent(ft, top_flag, cn_meta->parent, parent_slot);
	if (deferred_child)
		ft_set_parent(ft, deferred_child, deferred_parent, deferred_slot);

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
			else if (ft_node_skip_compressed(created[i]))
				free_compressed_node_unpublished(ft,
					ft_skip_to_compressed(ft, created[i]));
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
 * attaching the cluster to the rest of the trie, thus making it visible
 * to lookups.
 *
 * @external_node argument is either NULL or a pointer to the external
 * node we are replacing at the attachment location. We need to chain
 * this external node in the topmost internal node external node list in
 * that case.
 */
/*
 * Graft-transaction glue (build-invisible / publish / reclaim — see the
 * rcu-mutation discipline).  A graft attaches a payload subtrie at a
 * non-root key.  The attach cluster ("glue") is built entirely from
 * fresh, unobservable nodes BEFORE the source root is unlinked, so an
 * allocation failure frees the glue with both tries pristine — there is
 * nothing to roll back, and no past-sync abort().
 *
 * Every edge from the glue into LIVE data is a back-pointer re-parent
 * that must be deferred to the failure-free commit and applied only
 * after the source is unlinked and a grace period has drained its
 * readers.  Two flavours of live data:
 *   - the displaced dst old-child of a split compressed node, and
 *   - the live payload nodes pulled from the source (its old root, and
 *     any sub-compressed absorbed during canonicalization).
 * Forward edges into live data (a fresh node's child slot pointing at a
 * live node) ARE set during the build: they live in unobservable glue
 * nodes, so no reader follows them until the single commit-time publish.
 *
 * @built tracks every fresh glue node so the abort path can free them
 * (immediate free — never observed).  @deferred records the live
 * back-pointers to wire at commit.  @free_list records old (replaced)
 * live nodes to reclaim deferred after the publish.
 *
 * The struct is instantiable more than once: cds_ft_graft uses a single
 * glue for the dst-side attach; cds_ft_graft_swap commits two (the
 * dst-side insert glue + the swap-side extracted-root glue) together.
 *
 * Defined up here (rather than with its helper bodies further down)
 * because ft_try_compress_chain, ft_build_branch and the build-only
 * graft split all reference the complete type.
 */
struct ft_graft_deferred_edge {
	struct cds_ft_inode_flag *child;	/* live node to re-parent */
	struct cds_ft_inode_flag *parent;	/* glue node it will point to */
	struct cds_ft_inode_flag **slot;	/* slot in parent holding child */
	/*
	 * cds_ft_merge_at references live subtrees from BOTH tries.  A
	 * src-origin child is drained by the early src unlink (applied at
	 * apply_deferred); a dst-origin child stays reachable via the old dst
	 * spine until the forward publish + dst drain, so its back-pointer flip
	 * must wait (applied at apply_deferred_dst).  graft / graft_swap only
	 * ever re-parent src-origin nodes, so this defaults to false and their
	 * single apply_deferred call still wires every edge.
	 */
	bool dst_origin;
};

struct ft_graft_free_item {
	void *node;		/* cds_ft_inode * or cds_ft_compressed_node * */
	bool compressed;
};

/*
 * A deferred duplicate-chain splice, used only by cds_ft_merge_at when the
 * SAME full key exists in both tries: the two LIVE external chains must be
 * concatenated under the fresh merged node @owner.  graft / graft_swap never
 * concatenate two live chains, so this is merge-only.
 *
 * @dst_head is kept as the surviving chain head: its forward owner (a fresh
 * merged node's metadata->external_nodes, or a fresh merged node's child slot)
 * and its back-pointer (@dst_head->prev = that node) are wired by the ordinary
 * Phase-1 set + deferred edge, exactly like any other re-parented external.
 * This struct carries ONLY the concatenation, which is publication-visible on
 * two live chains and is applied at commit by ft_graft_glue_apply_splices,
 * AFTER the source has been detached + drained: the @src_head chain is appended
 * to @dst_head's tail (prev-before-next, the ft_chain_node idiom, but preserving
 * src_head->next so the rest of the src chain rides along).
 */
struct ft_graft_splice {
	struct cds_ft_node *dst_head;		/* surviving head (kept first) */
	struct cds_ft_node *src_head;		/* appended to dst_head's tail */
	/*
	 * The demoted @src_head's ordered-list cell, captured by
	 * ft_graft_glue_apply_splices.  It stays REACHABLE through its src-run
	 * neighbours' stale ord_prev/ord_next until the post-publish interleave
	 * rewires them, so it is freed only by
	 * ft_graft_glue_free_collided_cells, called after the interleave, via
	 * the grace-period-deferred cell free.  NULL when the list is off.
	 */
	struct ft_ord_cell *src_cell;
};

/*
 * Inline floor sizing: a graft / graft_swap attach cluster spans at most a
 * compressed prefix + branch + suffix + a payload path of up to FT_MAX_DEPTH
 * nodes + a canonicalization wrapper, with few deferred edges and freed nodes
 * (old-child; payload top / grandchild; old cn, old src root, absorbed
 * sub-cn).  These fit the inline arrays, so graft / graft_swap never allocate
 * a backing buffer and never grow past the floor.
 *
 * cds_ft_merge_at instead builds a TREE-shaped spine (one deferred edge per
 * disjoint subtree, one free per copied node), which can far exceed the floor.
 * It calls ft_graft_glue_reserve() to move the three arrays onto a malloc'd
 * backing sized by a read-only counting pre-pass; ft_graft_glue_abort() and
 * ft_graft_glue_fini() release it.  Every accessor indexes through the
 * pointers, so the growth is invisible to the helpers.
 */
#define FT_GRAFT_GLUE_FLOOR_BUILT	(2 * FT_MAX_DEPTH + 8)
#define FT_GRAFT_GLUE_FLOOR_DEFERRED	8
#define FT_GRAFT_GLUE_FLOOR_FREE	8
#define FT_GRAFT_GLUE_FLOOR_SPLICE	8

struct ft_graft_glue {
	struct ft_graft_deferred_edge *deferred;
	int nr_deferred;
	int cap_deferred;
	struct ft_graft_free_item *free_list;
	int nr_free;
	int cap_free;
	struct cds_ft_inode_flag **built;
	int nr_built;
	int cap_built;
	struct ft_graft_splice *splices;
	int nr_splices;
	int cap_splices;
	/*
	 * The single forward store that splices the cluster into dst at
	 * commit: parent_slot is swung to top.  publish_parent is the
	 * node flag owning the slot (for compressed skip bookkeeping;
	 * NULL at the root).  The cluster top's own back-pointer into
	 * publish_parent is recorded as an ordinary deferred edge.
	 */
	struct cds_ft_inode_flag *publish_parent;
	struct cds_ft_inode_flag **publish_slot;
	struct cds_ft_inode_flag *top;
	/*
	 * Node whose nr_keys == the grafted payload's key count, and from
	 * whose parent the external-count propagation starts at commit.
	 */
	struct cds_ft_inode_flag *attached_nf;
	/*
	 * Inline floor backing.  ft_graft_glue_init points the three arrays
	 * here; graft / graft_swap never outgrow it.  ft_graft_glue_reserve
	 * repoints to a malloc'd buffer when a count would exceed its floor.
	 */
	struct ft_graft_deferred_edge deferred_floor[FT_GRAFT_GLUE_FLOOR_DEFERRED];
	struct ft_graft_free_item free_floor[FT_GRAFT_GLUE_FLOOR_FREE];
	struct cds_ft_inode_flag *built_floor[FT_GRAFT_GLUE_FLOOR_BUILT];
	struct ft_graft_splice splices_floor[FT_GRAFT_GLUE_FLOOR_SPLICE];
};

static void ft_graft_glue_track(struct ft_graft_glue *g,
		struct cds_ft_inode_flag *nf);
static void ft_graft_glue_untrack(struct cds_ft *ft, struct ft_graft_glue *g, void *node_ptr);
static void ft_graft_glue_defer_edge(struct cds_ft *ft, struct ft_graft_glue *g,
		struct cds_ft_inode_flag *child,
		struct cds_ft_inode_flag *parent,
		struct cds_ft_inode_flag **slot);
static void ft_graft_glue_defer_free(struct ft_graft_glue *g,
		void *node, bool compressed);
static void ft_graft_glue_set_publish(struct cds_ft *ft, struct ft_graft_glue *g,
		struct cds_ft_inode_flag *parent_nf,
		struct cds_ft_inode_flag **parent_slot,
		struct cds_ft_inode_flag *top);

static struct cds_ft_inode_flag *ft_build_branch(struct cds_ft *ft,
		const uint8_t *key, unsigned int start, unsigned int end,
		struct cds_ft_inode_flag *leaf,
		unsigned long subtree_external_count,
		bool has_external_nodes,
		struct ft_graft_glue *glue);
static void ft_free_branch_unpublished(struct cds_ft *ft,
		struct cds_ft_inode_flag *top, struct cds_ft_inode_flag *leaf);

static struct cds_ft_inode_flag *ft_compress_single_child_if_needed(
		struct cds_ft *ft, struct cds_ft_inode_flag *child,
		struct ft_graft_glue *glue);

/*
 * Try to create a compressed path for a chain of single-child nodes.
 * Returns the compressed node flag on success, NULL if compression
 * is not applicable (path too short) or disabled, -ENOMEM cast to
 * pointer on allocation failure.
 *
 * @glue: when non-NULL (graft build-invisible mode), the live child's
 * back-pointer is recorded as a deferred edge rather than set now, and an
 * absorbed compressed @child (a fresh canonicalization wrapper) is
 * untracked from the glue as it is freed during the merge.
 */
#ifdef FEATURE_FT_COMPRESS
static
struct cds_ft_inode_flag *ft_try_compress_chain(struct cds_ft *ft,
		const uint8_t *key, size_t key_len, unsigned int level,
		struct cds_ft_inode_flag *child,
		struct cds_ft_node *external_nodes __attribute__((unused)),
		struct ft_graft_glue *glue)
{
	uint8_t path_len = (uint8_t)(key_len - level);
	struct cds_ft_compressed_node *cn;
	struct cds_ft_metadata *cn_meta;
	struct cds_ft_compressed_node *child_cn = NULL;
	unsigned int child_len = 0;
	uint8_t merged_len;
	int j;

	/*
	 * Length-1 compressed nodes are canonical under
	 * FEATURE_FT_SKIP_COMPRESSED: the publish wraps cn into a
	 * SKIP_X-tagged slot pointer, dispatching for free relative to
	 * the 1-child internal node it replaces.
	 */
	if (path_len < 1)
		return NULL;

	/*
	 * Chain-merge: if @child is already a compressed (or skip-
	 * compressed) node, wrapping it in another compressed prefix
	 * would violate the "no two adjacent compresseds" invariant.
	 * Absorb the child's path bytes into the outer cn so the
	 * result is a single compressed spanning
	 * (key[level..key_len-1] ++ child_cn->key_bytes) →
	 * child_cn->child.  Bounded by FT_SKIP_LEN_MAX; on overflow,
	 * fall back to the un-merged form (rare; the residue may be
	 * cleaned up by a subsequent mutation).
	 */
#ifdef FEATURE_FT_SKIP_COMPRESSED
	if (ft_node_skip_compressed(child))
		child_cn = ft_skip_to_compressed(ft, child);
	else
#endif
	if (ft_node_compressed(child))
		child_cn = ft_compressed_node_ptr(child);
	if (child_cn) {
		child_len = child_cn->len;
		if ((unsigned int) path_len + child_len > FT_SKIP_LEN_MAX) {
			/* Overflow: leave adjacency in place. */
			child_cn = NULL;
			child_len = 0;
		}
	}
	merged_len = (uint8_t)(path_len + child_len);

	cn = alloc_compressed_node(ft, merged_len, &cn_meta);
	if (!cn)
		return (struct cds_ft_inode_flag *) (long) -ENOMEM;
	if (child_cn)
		cn->child = child_cn->child;
	else
		cn->child = child;
	cn->len = merged_len;
	for (j = 0; j < path_len; j++)
		cn->key_bytes[j] = key[level + j];
	if (child_cn)
		memcpy(&cn->key_bytes[path_len],
			child_cn->key_bytes, child_len);
	cn_meta->nr_child = 1;
	ft_nr_keys_store(cn_meta, 1, CMM_RELAXED);
	/* Compressed nodes must not carry external_nodes. */
	assert(!external_nodes);
	{
		struct cds_ft_inode_flag *cflag = ft_compressed_node_flag(cn);

		if (glue) {
			/*
			 * Build-invisible (graft): cn->child is LIVE — either
			 * the merged-away child_cn's grandchild or the leaf
			 * itself.  Record its back-pointer for the post-sync
			 * commit instead of flipping it now.  An absorbed
			 * child_cn is a fresh canonicalization wrapper: drop it
			 * from glue tracking as it is freed here so the abort
			 * path cannot double-free it.
			 *
			 * Track the PLAIN @cflag (not the skip form returned to
			 * the caller): the abort path resolves a tracked node
			 * via ft_compressed_node_ptr, so it must not depend on
			 * cn->child's still-deferred back-pointer (which is how
			 * a skip pointer recovers its compressed node).
			 */
			ft_graft_glue_defer_edge(ft, glue, cn->child, cflag,
				&cn->child);
			if (child_cn) {
				ft_graft_glue_untrack(ft, glue, child_cn);
				free_compressed_node_unpublished(ft, child_cn);
			}
			ft_graft_glue_track(glue, cflag);
			/*
			 * Emit the creation trace but return the PLAIN flag:
			 * the caller installs @cn directly and resolves it via
			 * ft_compressed_node_ptr (cn->child's back-pointer is
			 * deferred, so the skip form would not yet resolve).
			 * The caller re-encodes the holding slot to skip before
			 * publish.
			 */
			(void) ft_publish_compressed(ft, cn, cflag);
			return cflag;
		}
		ft_set_parent(ft, cn->child, cflag, &cn->child);
		if (child_cn)
			free_compressed_node_unpublished(ft, child_cn);
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
		struct cds_ft_node *external_nodes __attribute__((unused)),
		struct ft_graft_glue *glue __attribute__((unused)))
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
		struct cds_ft_node *external_nodes,
		struct ft_insert_commit *ic)
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
			compress_level, iter_node_flag, NULL, NULL);
		if (compressed == (void *) (long) -ENOMEM) {
			ret = -ENOMEM;
			goto check_error;
		}
		if (compressed) {
			iter_node_flag = compressed;
			/*
			 * Track the PLAIN compressed flag, never the skip form
			 * ft_try_compress_chain returns in skip-compressed
			 * groups: a SKIP pointer encodes the CHILD's address
			 * (the application's external node for a leaf attach),
			 * so the kind dispatch in check_error's unwind would
			 * misread it as a plain node and run arena arithmetic
			 * on the application's pointer.  Same convention as the
			 * glue builders (see ft_try_compress_chain's glue arm).
			 */
			created_nodes[nr_created_nodes++] =
				ft_node_skip_compressed(compressed) ?
				ft_compressed_node_flag(
					ft_skip_to_compressed(ft, compressed)) :
				compressed;
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
					i - 1, false);
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
			/*
			 * Phase 1 (build-invisible): write the cluster top's
			 * cluster-internal external_nodes pointer.  The
			 * back-channel publish (external_nodes->prev =
			 * iter_node_flag) is deferred to Phase 2 below, after
			 * set_nth wires iter_node_flag's parent — otherwise an
			 * up-walk from external_nodes (still reachable through
			 * the old slot at attach_node_flag_ptr) lands on
			 * iter_node_flag with parent == NULL.
			 */
			ft_metadata_set_external_nodes(iter_node_flag,
				iter_node_metadata, external_nodes);
			ft_nr_keys_store(iter_node_metadata,
				ft_nr_keys_get(iter_node_metadata) + 1, CMM_RELAXED);
		}
	}

	/* Publish branch. */
	{
		uint8_t key_value;
		struct cds_ft_inode_flag *slot_child = iter_node_flag;

		key_value = *(--iter_key);
		dbg_printf("publish branch at level %d, key %u\n", level - 1, (unsigned int) key_value);

		ret = ft_insert_commit_arm(ft, ic);
		if (ret)
			goto check_error;
		if (ic && ic->batch) {
			/*
			 * One-commit insert: park a flip proxy in the slot
			 * instead of the cluster top.  It resolves to the OLD
			 * slot value (the displaced external, or NULL) until
			 * insert_done commits it together with the ordered-
			 * list edges, so the fresh key stays invisible here.
			 * The set_nth machinery skips proxy children (see
			 * ft_set_parent); the real top's wiring follows below,
			 * once the final slot is known.
			 */
			slot_child = ft_flip_batch_add(ic->batch,
				old_node_flag, iter_node_flag);
			ic->slot_value = iter_node_flag;
		}

		/* We need to use set_nth on the previous level. */
		iter_dest_node_flag = attach_node_flag;
		ret = ft_node_set_nth(ft, &iter_dest_node_flag, key_value, slot_child,
				&old_recompacted_node, metadata, level - 1, false);
		if (ret) {
			dbg_printf("branch publish error %d\n", ret);
			goto check_error;
		}
		if (ic && ic->batch) {
			struct cds_ft_inode_flag **slot_ptr = NULL;

			/*
			 * Fully wire the real top NOW (parent, slot offset,
			 * incoming_byte) -- every write lands in the FRESH
			 * top's own metadata/cell, invisible while the slot
			 * holds the proxy, and the splice-position search at
			 * insert_done depends on it (the structural up-walk
			 * rebuilds the new key through these fields).  A
			 * recompact replaced the target with a fresh copy (the
			 * proxy slot rode along; the copied LIVE children were
			 * re-parented by the recompact sweep, which skips the
			 * proxy) -- wire against the copy.
			 */
			ft_node_get_nth_skip(iter_dest_node_flag, &slot_ptr,
				key_value, FT_PF_NONE);
			assert(slot_ptr);
			ft_set_parent(ft, iter_node_flag, iter_dest_node_flag,
				slot_ptr);
			ic->slot = slot_ptr;
		}
		/*
		 * Phase 2: iter_node_flag's parent is now wired (by
		 * ft_node_set_nth above, either in-place or via recompact's
		 * reparent loop; by the explicit raw store for a one-commit
		 * insert).  Wire the back-channel from the live displaced
		 * external before the outer forward publish.
		 */
		ft_publish_external_nodes_prev(ft, iter_node_flag, external_nodes);
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
		 * the writer's stack — immediate-free is safe.  An armed
		 * one-commit batch was never parked anywhere visible (the
		 * final set_nth failed before storing): release it.
		 */
		if (ic && ic->batch) {
			ft_flip_batch_free_unpublished(ic->batch);
			ic->batch = NULL;
		}
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
struct cds_ft_inode_flag *ft_descent_step(struct cds_ft *ft, struct ft_descent *d,
		uint8_t key_value)
{
	d->ppnf  = d->pnf;
	d->ppnfp = d->pnfp;
	d->pnf   = d->nf;
	d->pnfp  = d->nfp;
	d->nf    = ft_node_get_nth(ft, d->pnf, &d->nfp, key_value, FT_PF_NONE);
	d->depth++;
	return d->nf;
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
 *         node into the trie to replace the prior external node.
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
		struct cds_ft_node *node,
		struct ft_insert_commit *ic)
{
	struct cds_ft_inode_flag *branch;
	struct cds_ft_metadata *br_meta;
	int ret;

	ret = ft_insert_commit_arm(ft, ic);
	if (ret)
		return ret;	/* nothing built yet */

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
			(struct cds_ft_inode_flag *) node, 1, false, NULL);
		if (!inner) {
			ret = -ENOMEM;
			goto arm_unwind;
		}
		ret = ft_node_set_nth(ft, &dest, key[br_start],
			inner, NULL, NULL, br_start, false);
		if (ret) {
			/*
			 * Free the built branch (it was leaked before): the
			 * cluster is writer-private, nothing was published.
			 * insert_done resets node->prev for the retry.
			 */
			ft_free_branch_unpublished(ft, inner,
				(struct cds_ft_inode_flag *) node);
			ret = -ENOMEM;
			goto arm_unwind;
		}
		branch = dest;
		br_meta = cds_ft_item_to_metadata(ft_node_ptr(branch));
		/*
		 * Phase 1 (build-invisible): wire branch's own parent and its
		 * cluster-internal external_nodes pointer.  The back-channel
		 * publish (cn->child->prev = branch) is deferred to Phase 2
		 * below — otherwise an up-walk from cn->child (still reachable
		 * through the unmodified cn) lands on branch with parent NULL.
		 */
		ft_set_parent(ft, branch, d->nf, &cn->child);
		ft_metadata_set_external_nodes(branch, br_meta,
			(struct cds_ft_node *) cn->child);
		/*
		 * Count only the pre-existing key (old external from
		 * the compressed child).  The new key's +1 is added
		 * by ft_propagate_external_count_parent below.
		 */
		ft_nr_keys_store(br_meta, 1, CMM_RELAXED);
	}
	/* Phase 2: back-channel + forward publish. */
	ft_publish_external_nodes_prev(ft, branch, (struct cds_ft_node *) cn->child);
	ft_insert_publish_or_park(ft, d->nf, &cn->child, branch, ic);
	/* One-commit (parked): the +1 follows the commit at insert_done. */
	if (ic && ic->slot)
		ic->count_from = branch;
	else
		ft_propagate_external_count_parent(ft, branch, 1);
	return 0;
arm_unwind:
	if (ic && ic->batch) {
		ft_flip_batch_free_unpublished(ic->batch);
		ic->batch = NULL;
	}
	return ret;
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
		struct cds_ft_node *node,
		struct ft_insert_commit *ic)
{
	int dret;

	dret = ft_split_compressed_insert(ft,
		d->nfp, d->nf, iter_key, remaining,
		j, node, d->depth, ic);
	if (dret)
		return dret;
	/*
	 * ft_split_compressed_insert wires top_flag's back-pointer into the
	 * live parent before publishing the cluster's forward slot, so the
	 * caller does not need to set the parent here.
	 *
	 * One-commit (parked): the +1 follows the commit at insert_done.
	 */
	if (ic && ic->slot)
		ic->count_from = d->pnf;
	else
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
		struct cds_ft_node **unique_node_ret,
		struct ft_insert_commit *ic)
{
	struct cds_ft_inode_flag *top_flag, *jct_flag;
	struct cds_ft_metadata *jct_meta;
	bool fresh_head;
	int sret;

	sret = ft_split_compressed_key_shorter(ft,
		d->nf, d->nfp, remaining, &top_flag, &jct_flag, d->depth);
	if (sret)
		return sret;
	jct_meta = cds_ft_item_to_metadata(ft_node_ptr(jct_flag));
	assert(!ft_node_compressed(jct_flag));
	/*
	 * Fresh head iff the junction carries no external_nodes (a non-empty
	 * set was transferred from the old compressed node for remaining == 0:
	 * the key already exists).  Attach a fresh head to the junction NOW,
	 * while the whole cluster is still invisible, so the parked one-commit
	 * publish below makes the structural attach and the ordered-list
	 * splice atomic; duplicates publish directly (no splice).
	 */
	fresh_head = (jct_meta->external_nodes == NULL);
	if (fresh_head) {
		ft_external_head_set_parent(ft, node, jct_flag);
		node->next = NULL;
		/* Cluster-internal store: the junction is unpublished. */
		jct_meta->external_nodes = node;
		sret = ft_insert_commit_arm(ft, ic);
		if (sret) {
			/*
			 * Arm failed: roll the head attach back (the cluster
			 * is still invisible) and publish the split WITHOUT
			 * the new key -- a key-neutral restructure of the same
			 * content -- then surface the failure (the caller's
			 * insert_done resets node->prev for a retry).
			 */
			jct_meta->external_nodes = NULL;
			node->next = NULL;
			ft_publish_to_parent(ft, d->pnf, d->nfp, top_flag);
			free_compressed_node(ft,
				ft_compressed_node_ptr(d->nf));
			return sret;
		}
	}
	/*
	 * ft_split_compressed_key_shorter wires top_flag's back-pointer into
	 * the live parent before its deferred back-channel re-parent of
	 * cn->child, so the caller does not need to set the parent here.
	 */
	ft_insert_publish_or_park(ft, d->pnf, d->nfp, top_flag,
		fresh_head ? ic : NULL);
	if (!fresh_head) {
		if (unique_node_ret) {
			*unique_node_ret = jct_meta->external_nodes;
			free_compressed_node(ft,
				ft_compressed_node_ptr(d->nf));
			return -EEXIST;
		}
		{
			/*
			 * Junction already has external_nodes (transferred
			 * from old compressed node for remaining == 0).
			 * Chain new node as duplicate; no key count change.
			 */
			struct cds_ft_node *last = jct_meta->external_nodes;

			while (ft_node_next(last))
				last = ft_node_next(last);
			ft_chain_node(last, node);
		}
		free_compressed_node(ft, ft_compressed_node_ptr(d->nf));
		return 0;
	}
	/* One-commit (parked): the +1 follows the commit at insert_done. */
	if (ic && ic->slot) {
		ic->count_from = jct_flag;
		ic->free_old_cn = ft_compressed_node_ptr(d->nf);
	} else {
		ft_propagate_external_count_parent(ft, jct_flag, 1);
		free_compressed_node(ft, ft_compressed_node_ptr(d->nf));
	}
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
		int *ret_p,
		struct ft_insert_commit *ic)
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
			key_len, cn, node, ic);
		return FT_DESCENT_END;
	}
	if (j < cmp) {
		/* Key diverges: split at position j. */
		*ret_p = ft_insert_compressed_diverge(ft, d,
			*iter_key_p, remaining, j, node, ic);
		return FT_DESCENT_END;
	}
	/* Key shorter: split into prefix -> junction -> suffix. */
	*ret_p = ft_insert_compressed_key_shorter(ft, d, remaining,
		node, unique_node_ret, ic);
	return FT_DESCENT_END;
}



/*
 * Ordinal-cell point-op list helpers.  Defined after the flip-batch +
 * inequality-lookup helpers (which they use); forward-declared here for the
 * insert / remove / replace mutators below.  All gated by ordered_list_set.
 */
static void ft_ord_cell_splice(struct cds_ft *ft, const uint8_t *key,
		size_t key_len, struct ft_ord_cell *cell);
static void ft_ord_cell_prefill_by_key(struct cds_ft *ft, const uint8_t *key,
		size_t key_len, struct ft_ord_cell *cell,
		struct ft_ord_cell **pred_out, struct ft_ord_cell **succ_out);
static void ft_iter_set_key_ordinals(struct cds_ft_iter *iter,
		const uint8_t *ordinals, size_t key_len);
static void ft_ord_cell_splice_at(struct cds_ft *ft, struct ft_ord_cell *cell,
		struct ft_ord_cell *pred, struct ft_ord_cell *succ);
static void ft_ord_cell_unsplice(struct cds_ft *ft, struct ft_ord_cell *cell);
static void ft_ord_cell_swap(struct cds_ft *ft, struct ft_ord_cell *old_cell,
		struct ft_ord_cell *new_cell);
/* Bulk-op ordered-list maintenance. */
static struct cds_ft_node *ft_subtree_minmax_head(
		struct cds_ft *ft, struct cds_ft_inode_flag *nf, bool want_max);
static void ft_ord_cell_run_detach(struct cds_ft *ft, struct cds_ft *into,
		struct cds_ft_node *first_head, struct cds_ft_node *last_head);
static void ft_ord_cell_run_splice(struct cds_ft *dst,
		struct ft_ord_cell *run_first, struct ft_ord_cell *run_last,
		struct ft_ord_cell *pred, struct ft_ord_cell *succ);
static void ft_ord_cell_find_splice_pos(struct cds_ft *dst,
		const uint8_t *key, size_t key_len,
		struct ft_ord_cell **pred_out, struct ft_ord_cell **succ_out);
static void ft_ord_cell_run_replace(struct cds_ft *dst,
		struct ft_ord_cell *d_first, struct ft_ord_cell *d_last,
		struct ft_ord_cell *s_first, struct ft_ord_cell *s_last);
struct ft_ord_cell_edge;
struct ft_flip_batch;
/* @dst_key in ORDINAL form (converted once at the cds_ft_merge_at entry). */
static void ft_merge_ord_interleave(struct cds_ft *dst, const uint8_t *dst_key,
		size_t dst_key_len, unsigned long merged_keys,
		struct ft_ord_cell *ord_cursor, struct ft_ord_cell *prev_placed,
		struct ft_ord_cell_edge *edges, struct ft_flip_batch *flip_b);
static void ft_ord_cell_run_unlink(struct cds_ft *ft,
		struct cds_ft_node *first_head, struct cds_ft_node *last_head);

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
	struct ft_ord_cell *precell;
	struct ft_insert_commit ic = { NULL, NULL, NULL, NULL, NULL, NULL, false, false };

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
	if (node->prev || ft_node_next(node))
		return -EINVAL;

	/*
	 * Ordered-list trie: pre-wire @node's ordinal cell before any structural
	 * mutation, so the only failure-prone allocation happens up front (a
	 * clean -ENOMEM, nothing to roll back) and every fresh-head wiring site
	 * downstream just records the flagged parent into the cell (node->prev
	 * already carries it).  If @node ends up a duplicate (chained, not a
	 * head) or the insert fails, the unused @precell is freed at insert_done
	 * (a chained @node has its prev repointed at the predecessor, losing the
	 * cell from node->prev, so the handle is kept here).
	 *
	 * List off: no cell -- @node behaves like a non-cell build (its prev is
	 * wired to the flagged parent directly by the fresh-head sites), saving
	 * the per-key cell.  @precell stays NULL and the cell paths below no-op.
	 */
	precell = NULL;
	if (ft->ordered_list) {
		void *cell = ft_ord_cell_alloc(ft, node, NULL);

		if (!cell)
			return -ENOMEM;
		node->prev = cell;
		precell = ft_ord_cell_ptr(cell);
		/*
		 * Head's last edge byte for the up-walk key rebuild: the cell is
		 * the head's metadata record and @key is ordinal here, so
		 * key[key_len - 1] is the byte the head hangs under (ignored by the
		 * up-walk when the head's parent is a compressed node, whose
		 * key_bytes already span the head's position).
		 */
		if (key_len)
			cds_ft_item_to_metadata(precell)->incoming_byte =
				(uint8_t) key[key_len - 1];
	}

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
		d.nf = ft_resolve_skip_compressed(ft, d.nf);
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
				&nr_snapshot, &ret, &ic);
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
		ft_descent_step(ft, &d, key_value);
	}

	/*
	 * Resolve any skip-compressed pointer left in d.nf by the descent
	 * loop's final step (e.g., ft_descent_traverse_compressed sets d.nf
	 * to cn->child raw, which may be skip-compressed).  The loop body's
	 * resolve at the top of each iteration only fires when the loop
	 * iterates again; a traverse that pushes d.depth to key_depth - 1
	 * exits the loop without re-entering.
	 */
	d.nf = ft_resolve_skip_compressed(ft, d.nf);

	if (d.depth == key_depth - 1) {
		/* Found either an internal, external node or NULL at end of key. */
		if (!d.nf) {
			dbg_printf("cds_ft_insert NULL ppnf %p pnf %p nfp %p nf %p\n",
					d.ppnf, d.pnf, d.nfp, d.nf);

			ret = ft_attach_node(ft, d.pnfp, d.pnf,
					d.nfp, d.nf, key, key_len, d.depth, node,
					NULL, &ic);
			if (ret == 0) {
				/*
				 * One-commit insert (ic.slot parked): the key is
				 * not reachable until insert_done's commit, so
				 * the count propagation moves there (an early +1
				 * would overcount -- the inverse of the nr_keys
				 * undercount discipline).
				 */
				if (!ic.slot)
					ft_propagate_external_count_parent(ft,
						*d.pnfp, 1);
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
				node, unique_node_ret, &ic);
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
					ret = -EEXIST;
					goto insert_done;
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
				ft_external_head_set_parent(ft, node, d.nf);
				node->next = NULL;
				if (ft->ordered_list) {
					/*
					 * Readers do not resolve flip proxies
					 * on external_nodes loads, so this
					 * shape cannot park a one-commit
					 * proxy.  B-lite instead: locate the
					 * splice position and pre-fill the
					 * cell's links BEFORE the head becomes
					 * reachable -- a reader landing on the
					 * fresh head always sees valid links
					 * -- and flip the neighbour edges
					 * right after the store.
					 */
					struct ft_ord_cell *pred, *succ;

					ft_ord_cell_prefill_by_key(ft, _key,
						_key_len, precell, &pred, &succ);
					rcu_assign_pointer(
						metadata->external_nodes, node);
					ft_ord_cell_splice_at(ft, precell,
						pred, succ);
					ic.spliced = true;
				} else {
					rcu_assign_pointer(
						metadata->external_nodes, node);
				}
				ret = 0;
				ft_propagate_external_count_parent(ft, d.nf, 1);
			}
		} else {
			struct cds_ft_node *iter_node, *last_node = NULL;

			if (unique_node_ret) {
				*unique_node_ret = (struct cds_ft_node *) ft_node_ptr(d.nf);
				ret = -EEXIST;
				goto insert_done;
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
		 * populate this new internal node into the trie to replace the prior
		 * external node.
		 * It's the same for NULL node, only that there is no need to chain any
		 * external node.
		 */

		dbg_printf("cds_ft_insert NULL or external ppnf %p pnf %p nfp %p nf %p\n",
				d.ppnf, d.pnf, d.nfp, d.nf);

		ret = ft_attach_node(ft, d.pnfp, d.pnf,
				d.nfp, d.nf, key, key_len, d.depth, node,
				(struct cds_ft_node *) ft_node_ptr(d.nf), &ic);
		if (ret == 0) {
			/* One-commit: count propagation deferred (see above). */
			if (!ic.slot)
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
	/*
	 * @node became a fresh head iff node->prev is still its (cell) carrier
	 * — i.e. not external.  A duplicate append (ft_chain_node repointed
	 * node->prev at the predecessor) or a failed insert leaves @precell
	 * orphaned: free it, and on failure restore node->prev to its zeroed
	 * state so the application may retry, then splice the kept cell into the
	 * ordered list.  List off: no cell was allocated -- @node->prev is the
	 * flagged parent (fresh head) or the predecessor (dup), so there is
	 * nothing to free or splice; a FAILED insert may still have wired
	 * node->prev early (the build paths set the raw parent before their
	 * fallible publish, e.g. ft_try_compress_chain -> ft_set_parent, the
	 * split branch builders), and the failure unwind frees that cluster:
	 * reset it so the dangling pointer cannot leak into a retry -- the
	 * zeroed-prev check at entry would otherwise reject the node with
	 * -EINVAL forever.
	 */
	if (!ft->ordered_list) {
		if (ret != 0)
			node->prev = NULL;
	} else {
		if (ret != 0) {
			node->prev = NULL;
			ft_ord_cell_free_unpublished(ft, precell);
			if (ic.batch)
				ft_flip_batch_free_unpublished(ic.batch);
		} else if (ft_node_external((struct cds_ft_inode_flag *) node->prev)) {
			ft_ord_cell_free_unpublished(ft, precell);
			if (ic.batch)
				ft_flip_batch_free_unpublished(ic.batch);
		} else if (ic.slot) {
			/*
			 * One-commit: the structural slot is parked as a flip
			 * proxy (the key is still invisible).  Splice position
			 * + cell links first, then ONE commit flips the
			 * structural slot AND the ordered-list edges -- a
			 * reader never sees the head without its cell in the
			 * list.  Count propagation follows the commit (the key
			 * only now counts), from the shape's recorded base.
			 */
			ft_insert_one_commit(ft, _key, _key_len, precell, &ic);
			ft_propagate_external_count_parent(ft,
				ic.count_from ? ic.count_from : *d.pnfp, 1);
		} else if (ic.spliced) {
			/* B-lite shape: spliced at the attach site. */
		} else {
			/*
			 * @node became a fresh head through a shape that
			 * publishes without a parkable slot (insert_replace's
			 * paths): legacy post-publish splice.
			 */
			ft_ord_cell_splice(ft, _key, _key_len, precell);
			if (ic.batch)
				ft_flip_batch_free_unpublished(ic.batch);
		}
	}
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
	struct ft_ord_cell *precell;

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
	if (node->prev || ft_node_next(node))
		return -EINVAL;

	/* Ordered-list trie: pre-wire @node's cell (see _cds_ft_insert).  A replace
	 * always lands @node as the sole head on success, so the cell is kept
	 * unless the insert fails (or the key_shorter path finds the key and
	 * leaves @node uninstalled — both freed below).  List off: no cell. */
	precell = NULL;
	if (ft->ordered_list) {
		void *cell = ft_ord_cell_alloc(ft, node, NULL);

		if (!cell)
			return -ENOMEM;
		node->prev = cell;
		precell = ft_ord_cell_ptr(cell);
		/*
		 * Head's last edge byte for the up-walk key rebuild: the cell is
		 * the head's metadata record and @key is ordinal here, so
		 * key[key_len - 1] is the byte the head hangs under (ignored by the
		 * up-walk when the head's parent is a compressed node, whose
		 * key_bytes already span the head's position).
		 */
		if (key_len)
			cds_ft_item_to_metadata(precell)->incoming_byte =
				(uint8_t) key[key_len - 1];
	}

	key_depth = key_len + 1;

	dbg_printf("_cds_ft_insert_replace attempt: node %p\n", node);
	iter_key = key;
	ft_descent_init(&d, ft);

	for (; d.depth < key_depth - 1; ) {
		uint8_t key_value;

		if (!d.nf)
			break;
		/* Resolve skip-compressed pointer. */
		d.nf = ft_resolve_skip_compressed(ft, d.nf);
		if (ft_node_external(d.nf))
			break;
		if (ft_node_compressed(d.nf)) {
			enum ft_descent_action act;

			act = ft_insert_compressed(ft, &d, &iter_key,
				key, key_len, key_depth, node,
				NULL, snapshot, snapshot_depth,
				&nr_snapshot, &ret, NULL);
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
		ft_descent_step(ft, &d, key_value);
	}

	/*
	 * Resolve any skip-compressed pointer left in d.nf by a final
	 * traverse that exited the loop without re-entering the loop's
	 * resolve step.
	 */
	d.nf = ft_resolve_skip_compressed(ft, d.nf);

	if (d.depth == key_depth - 1) {
		/* Found either an internal, external node or NULL at end of key. */
		if (!d.nf) {
			/* No existing node. Regular attach. */
			dbg_printf("_cds_ft_insert_replace NULL at end of key\n");

			ret = ft_attach_node(ft, d.pnfp, d.pnf,
					d.nfp, d.nf, key, key_len, d.depth, node,
					NULL, NULL);
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
				node, old_node_ret, NULL);
			if (ret == -EEXIST) {
				ret = 0;	/* Replace handled by key_shorter. */
				/*
				 * key_shorter found the key already present and
				 * left @node uninstalled (*old_node_ret names the
				 * existing chain, which keeps its own cell).  Drop
				 * the pre-wired cell from node->prev; insert_replace_done
				 * frees the now-orphaned @precell.
				 */
				node->prev = NULL;
			}
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
				ft_external_head_set_parent(ft, node, d.nf);
				node->next = NULL;
				rcu_assign_pointer(metadata->external_nodes, node);
				/*
				 * Ordered list on: the replaced head's cell leaves the
				 * trie; @node's pre-wired cell takes its list slot (O(1)
				 * swap, no re-descent), then the old cell is freed.  List
				 * off: no cells, nothing to swap or free.
				 */
				if (ft->ordered_list) {
					struct ft_ord_cell *old_cell =
						ft_ord_cell_ptr(external_nodes->prev);

					ft_ord_cell_swap(ft, old_cell, precell);
					ft_ord_cell_free(ft, old_cell);
				}
			} else {
				/* No external nodes yet. New key. */
				ft_external_head_set_parent(ft, node, d.nf);
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
			ft_external_head_set_parent(ft, node, d.pnf);
			node->next = NULL;
			ft_publish_to_parent(ft, d.pnf, d.nfp,
				(struct cds_ft_inode_flag *) node);
			/* Ordered list on: replaced head's cell leaves; @node's cell
			 * takes its slot, then the old cell is freed.  List off: none. */
			if (ft->ordered_list) {
				struct ft_ord_cell *old_cell =
					ft_ord_cell_ptr((*old_node_ret)->prev);

				ft_ord_cell_swap(ft, old_cell, precell);
				ft_ord_cell_free(ft, old_cell);
			}
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
				(struct cds_ft_node *) ft_node_ptr(d.nf), NULL);
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
	/*
	 * @node became the installed head iff node->prev still carries its
	 * pre-wired cell (not external).  A duplicate append (descent through a
	 * compressed node chained @node), the uninstalled key_shorter -EEXIST
	 * path (prev NULLed above), or a failed insert leaves @precell orphaned
	 * — free it, and on failure restore node->prev to its zeroed state.
	 * List off: no cell, nothing to free or splice (the swap sites above are
	 * gated too) -- but a FAILED insert may have wired node->prev early in
	 * a build path whose cluster the unwind then freed: reset it so the
	 * retry does not hit the zeroed-prev entry check (see _cds_ft_insert).
	 */
	if (!ft->ordered_list) {
		if (ret != 0)
			node->prev = NULL;
	} else {
		if (ret != 0) {
			node->prev = NULL;
			ft_ord_cell_free_unpublished(ft, precell);
		} else if (ft_node_external((struct cds_ft_inode_flag *) node->prev)) {
			ft_ord_cell_free_unpublished(ft, precell);
		} else if (*old_node_ret == NULL) {
			/*
			 * Fresh head (no chain replaced): splice its kept cell.  A replace
			 * (*old_node_ret set) already swapped @precell into the replaced
			 * head's list slot at the replace site, so it must NOT splice again.
			 */
			ft_ord_cell_splice(ft, _key, _key_len, precell);
		}
	}
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
	struct cds_ft_inode_flag *holder_flag;
	struct cds_ft_inode_flag **pub_slot;
	struct cds_ft_compressed_node *cn = NULL;
	const uint8_t *iter_key;
	size_t key_len = ft_key_len(ft, ft_iter_resolve_key_len(iter));
	enum cds_ft_status s;

	CDS_FT_SCOPED_WRITER(ft);
	FT_TP_ITER_KEY(replace_enter, iter);

	/*
	 * If the iterator has a valid path, the RCU read-side lock must
	 * be held.
	 */
	if (iter->cache_valid)
		CDS_FT_ASSERT_RCU_READ_LOCKED(ft);
	iter_debug_path_check(iter);

	if (!valid_external_node(old_node) || !valid_external_node(new_node)
			|| !valid_key_len(ft, key_len)) {
		s = CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
		FT_TP(replace_exit, (int) s);
		return s;
	}
	/* Expect zeroed next and prev pointers on new_node. */
	if (ft_node_next(new_node) || new_node->prev) {
		s = CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
		FT_TP(replace_exit, (int) s);
		return s;
	}

	iter_key = ft_iter_read_key(iter);

	dbg_printf("cds_ft_replace: old_node %p new_node %p\n", old_node, new_node);

	/*
	 * No top-down descent.  As in cds_ft_remove, @old_node is
	 * application-owned and -- with the RCU read-side lock held
	 * continuously since it was obtained -- alive; the writer mutex held
	 * here freezes the structure, so @old_node->prev is a settled live
	 * pointer to its holder.  @new_node takes @old_node's exact place in
	 * the duplicate chain, so the key count and trie shape are unchanged:
	 * only the chain link (or head slot) that points at @old_node is
	 * repointed at @new_node.  That slot is derived from the holder -- the
	 * predecessor's next for a non-head duplicate, else the head slot
	 * cds_ft_remove recovers (a compressed holder's &cn->child, an
	 * internal holder's external_nodes, or an internal body slot keyed by
	 * the last key byte).
	 */
	if (ft_node_is_removed(old_node)) {
		/* Already unlinked from the trie. */
		s = CDS_FT_STATUS_NOT_FOUND;
		FT_TP(replace_exit, (int) s);
		return s;
	}
	holder_flag = ft_node_holder(ft, old_node);
	if (!holder_flag) {
		/* Never inserted (a freshly-initialized node). */
		s = CDS_FT_STATUS_NOT_FOUND;
		FT_TP(replace_exit, (int) s);
		return s;
	}

	if (ft_node_external(holder_flag)) {
		/* Non-head duplicate: repoint the predecessor's next. */
		pub_slot = (struct cds_ft_inode_flag **)
			&((struct cds_ft_node *) holder_flag)->next;
	} else if (ft_node_compressed(holder_flag) ||
		   ft_node_skip_compressed(holder_flag)) {
		/* Compressed holder: @old_node is its single external child. */
		cn = ft_node_skip_compressed(holder_flag) ?
			ft_skip_to_compressed(ft, holder_flag) :
			ft_compressed_node_ptr(holder_flag);
		if ((struct cds_ft_node *) ft_node_ptr(cn->child) != old_node) {
			s = CDS_FT_STATUS_NOT_FOUND;
			FT_TP(replace_exit, (int) s);
			return s;
		}
		pub_slot = &cn->child;
	} else if (ft_node_external_nodes(holder_flag) ==
			(struct cds_ft_node *) old_node) {
		/* Internal holder: @old_node heads its external_nodes chain. */
		pub_slot = (struct cds_ft_inode_flag **)
			&cds_ft_item_to_metadata(ft_node_ptr(holder_flag))->external_nodes;
	} else {
		/* Internal holder: @old_node is a body child (leaf key). */
		struct cds_ft_inode_flag *child;

		child = ft_node_get_nth_skip(holder_flag, &pub_slot,
			iter_key[key_len - 1], FT_PF_NONE);
		if (!child ||
		    (struct cds_ft_node *) ft_node_ptr(child) != old_node) {
			s = CDS_FT_STATUS_NOT_FOUND;
			FT_TP(replace_exit, (int) s);
			return s;
		}
	}

	/*
	 * Splice @new_node into the chain in place of @old_node: it inherits
	 * @old_node's prev and next, the successor (if any) is repointed back
	 * at it, and rcu_assign_pointer publishes it into the slot, ordering
	 * those stores before it becomes reachable.
	 */
	new_node->prev = old_node->prev;
	new_node->next = ft_node_next(old_node);
	if (new_node->next)
		new_node->next->prev = new_node;
	rcu_assign_pointer(*pub_slot, (struct cds_ft_inode_flag *) new_node);

	/*
	 * Cell transfer: @new_node inherited @old_node's prev (its cell, when a
	 * head) via the copy above, so it shares the same cell — now fully
	 * assembled and published.  Retarget the cell at @new_node so up-walks
	 * and ordered iteration resolve to the live node; the cell's parent and
	 * ord-list position are preserved (no list surgery, no free).  A
	 * non-head duplicate replace copied an external prev — nothing to do.
	 * List off: @new_node->prev is the flagged parent directly (inherited),
	 * no cell to retarget.
	 */
	if (ft->ordered_list &&
	    !ft_node_external((struct cds_ft_inode_flag *) new_node->prev))
		rcu_assign_pointer(ft_ord_cell_ptr(new_node->prev)->node, new_node);

#ifdef FEATURE_FT_SKIP_COMPRESSED
	/*
	 * A compressed-head replace changed cn->child; re-encode the
	 * grandparent's skip-compressed slot to the new child so a skip
	 * descent or ft_skip_reanchor up-walk does not follow the stale
	 * pointer into the about-to-be-freed @old_node (the dangling-skip
	 * UAF that cds_ft_remove guards against on its compressed-head
	 * unchain).  No-ops for a plain (non-skip) compressed slot.
	 */
	if (cn)
		ft_update_skip_pointer(ft_get_parent_slot(
			cds_ft_item_to_metadata((struct cds_ft_inode *) cn), ft), cn);
#endif

	/*
	 * @old_node has left the trie (replaced by @new_node): tombstone it.
	 * Its next pointer is preserved so a concurrent reader positioned on
	 * @old_node still follows the chain.
	 */
	ft_node_mark_removed(old_node);


	/*
	 * The trie structure is unchanged (no recompaction), so the iterator
	 * path remains valid in cached mode.
	 */
	iter_auto_invalidate_cache(iter);
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
			cn = ft_skip_to_compressed(ft, iter_node_flag);
		else
			cn = ft_compressed_node_ptr(iter_node_flag);
		/*
		 * Wire the external's prev to cn BEFORE publishing cn->child:
		 * after the publish, a skip pointer at cn's grandparent slot
		 * resolves through cn->child = topmost_external_nodes, and
		 * ft_skip_to_compressed walks topmost->prev to recover cn.
		 * Setting prev after the publish leaves a window where prev
		 * still points at the about-to-be-detached old holder, so
		 * skip-recovery returns the wrong compressed node.
		 */
		ft_set_parent(ft, 
			(struct cds_ft_inode_flag *) topmost_external_nodes,
			ft_compressed_node_flag(cn), &cn->child);
		ft_publish_to_parent(ft, ft_compressed_node_flag(cn),
			&cn->child,
			(struct cds_ft_inode_flag *) topmost_external_nodes);
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
		fresh_meta->parent_slot_offset = src_meta->parent_slot_offset;
#endif
		ft_publish_to_parent(ft, src_meta->parent,
			detach_parent_flag_ptr,
			ft_node_flag(fresh, 0));
		free_compressed_node(ft,
			ft_compressed_node_ptr(iter_node_flag));
	}
	return 0;
}

#ifdef FEATURE_FT_SKIP_COMPRESSED
/*
 * ft_canonicalize_chain_compress: collapse a non-root 1-child internal
 * node with no external_nodes into a single compressed node, merging
 * with adjacent compressed parent/child via the 4-case chain-merge:
 *
 *  - parent non-compressed, child non-compressed:
 *      [iter_internal] → [new_cn(1 byte)]; child preserved.
 *  - parent non-compressed, child compressed:
 *      [iter_internal, child_cn] → [new_cn(1 + child_cn.len bytes)];
 *      new_cn.child = child_cn.child.
 *  - parent compressed, child non-compressed:
 *      [parent_cn, iter_internal] → [new_cn(parent_cn.len + 1 bytes)];
 *      new_cn.child = surviving_child.
 *  - parent compressed, child compressed:
 *      [parent_cn, iter_internal, child_cn] →
 *      [new_cn(parent_cn.len + 1 + child_cn.len bytes)];
 *      new_cn.child = child_cn.child.
 *
 * Preserves the "no two adjacent compresseds" invariant by absorbing
 * adjacent compressed neighbours into the new node.  Bounded by
 * FT_SKIP_LEN_MAX (uint8_t cn->len): when the merged length would
 * exceed the bound, leaves non-canonical residue (subsequent inserts
 * may rebuild canonical form).  Allocation failure: same fallback.
 *
 * Preconditions (caller asserts):
 *   - @iter_meta->nr_child == 1
 *   - @iter_meta->external_nodes == NULL
 *   - @iter_meta->parent != NULL  (non-root)
 *
 * @iter_node_flag: the 1-child internal being collapsed.
 * @iter_meta:      its metadata.
 * @slot_ptr:       parent slot pointing to @iter_node_flag.  Used only
 *                  when the parent is non-compressed; compressed parents
 *                  resolve their own slot via parent_slot_offset.
 */
static
void ft_canonicalize_chain_compress(struct cds_ft *ft,
		struct cds_ft_inode_flag *iter_node_flag,
		struct cds_ft_metadata *iter_meta,
		struct cds_ft_inode_flag **slot_ptr)
{
	uint8_t surviving_byte = 0;
	struct cds_ft_inode_flag *surviving_child;
	bool parent_compressed, child_compressed;
	struct cds_ft_compressed_node *parent_cn, *child_cn;
	struct cds_ft_metadata *parent_cn_meta;
	unsigned int parent_len, child_len, merged_len;
	struct cds_ft_metadata *new_cn_meta;
	struct cds_ft_compressed_node *new_cn;
	struct cds_ft_inode_flag *new_cn_flag;
	struct cds_ft_inode_flag **publish_slot;
	struct cds_ft_inode_flag *publish_parent;

	surviving_child = ft_node_get_minmax(ft, iter_node_flag,
		&surviving_byte, FT_LEFTMOST,
		false /* writer; no validation */);
	if (!surviving_child)
		return;
	parent_compressed = ft_node_compressed(iter_meta->parent);
	child_compressed = ft_node_compressed(surviving_child);
	parent_cn = parent_compressed
		? ft_compressed_node_ptr(iter_meta->parent)
		: NULL;
	parent_cn_meta = parent_cn
		? cds_ft_item_to_metadata((struct cds_ft_inode *) parent_cn)
		: NULL;
	child_cn = child_compressed
		? ft_compressed_node_ptr(surviving_child)
		: NULL;
	parent_len = parent_cn ? parent_cn->len : 0;
	child_len = child_cn ? child_cn->len : 0;
	merged_len = parent_len + 1 + child_len;

	if (merged_len > FT_SKIP_LEN_MAX)
		return;
	new_cn = alloc_compressed_node(ft, merged_len, &new_cn_meta);
	if (!new_cn)
		return;

	/* Compose merged path bytes. */
	if (parent_cn)
		memcpy(new_cn->key_bytes, parent_cn->key_bytes, parent_len);
	new_cn->key_bytes[parent_len] = surviving_byte;
	if (child_cn)
		memcpy(&new_cn->key_bytes[parent_len + 1],
			child_cn->key_bytes, child_len);
	new_cn->len = (uint8_t) merged_len;
	new_cn->child = child_cn ? child_cn->child : surviving_child;
	new_cn_meta->nr_child = 1;
	ft_nr_keys_store(new_cn_meta,
		ft_nr_keys_get(parent_cn_meta
			? parent_cn_meta
			: iter_meta),
		CMM_RELAXED);

	if (parent_cn) {
		/*
		 * Replace parent_cn at its own slot in the grandparent.
		 * Inherit grandparent context from parent_cn.
		 */
		new_cn_meta->parent = parent_cn_meta->parent;
		publish_parent = parent_cn_meta->parent;
		publish_slot = ft_get_parent_slot(parent_cn_meta, ft);
	} else {
		new_cn_meta->parent = iter_meta->parent;
		publish_parent = iter_meta->parent;
		publish_slot = slot_ptr;
	}
	ft_set_parent_slot(new_cn_meta, publish_slot);

	new_cn_flag = ft_compressed_node_flag(new_cn);
	ft_set_parent(ft, new_cn->child, new_cn_flag, &new_cn->child);
	new_cn_flag = ft_publish_compressed(ft, new_cn, new_cn_flag);
	ft_publish_to_parent(ft, publish_parent, publish_slot, new_cn_flag);

	free_cds_ft_node(ft, ft_node_ptr(iter_node_flag));
	if (parent_cn)
		free_compressed_node(ft, parent_cn);
	if (child_cn)
		free_compressed_node(ft, child_cn);
}
#endif

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
	 * variable-length key entries that must be preserved by promoting
	 * them to the replacement slot.
	 *
	 * Only for a destroy-style detach (free_detached_subtree): a
	 * move-style detach PRESERVES the target subtree (it becomes another
	 * trie's content), so the target keeps its own external_nodes and
	 * must NOT have them promoted into the source.  This matters once a
	 * move-style caller bootstraps from the target itself (the climb
	 * starts at *detach_node_flag_ptr == the target); the descent-based
	 * callers passed the single-child chain head here, which never
	 * carries external_nodes, so this gate is a no-op for them.
	 */
	if (free_detached_subtree) {
		struct cds_ft_inode_flag *detach_child = *detach_node_flag_ptr;

		/*
		 * Resolve skip-compressed before type checks: a skip
		 * pointer with an external child has low bits == 0,
		 * falsely matching ft_node_external and skipping the
		 * external_nodes preservation entirely.
		 */
		detach_child = ft_resolve_skip_compressed(ft, detach_child);

		if (detach_child && !ft_node_external(detach_child)) {
			struct cds_ft_metadata *child_meta =
				ft_flag_to_metadata(ft, detach_child);
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
	/*
	 * Resolve a skip-compressed initial parent slot.  When the detach is
	 * bootstrapped from a leaf whose holder is a compressed node (e.g.
	 * cds_ft_remove deriving the position from node->prev), the slot that
	 * holds the holder is skip-encoded (SKIP_X(cn->child)); ft_node_ptr
	 * does not strip the skip high bits, so resolve to the plain
	 * compressed flag here.  No-op for a plain (descent-supplied) slot.
	 */
	cur = ft_resolve_skip_compressed(ft, *detach_parent_flag_ptr);
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
		/*
		 * Stop the upward prune at a surviving boundary: a multi-child
		 * node, the root, a level past one whose external_nodes were
		 * promoted (prev_external_nodes_found), or a node that keeps its
		 * own key (external_nodes) once a deeper promotion is already in
		 * flight (topmost_external_nodes set).  A SINGLE-child node that
		 * carries external_nodes does NOT stop here: pruning its only
		 * (on-path) child empties it, so it cannot stay — its external
		 * chain is promoted into its own parent slot and the climb
		 * continues past it (handled just below).  When the detach was
		 * bootstrapped from a node that already carried external_nodes
		 * (descent callers, topmost set at start), that node is the one
		 * being promoted and a further external ancestor is a genuine
		 * boundary — hence the `&& topmost_external_nodes` guard.
		 */
		if (prev_external_nodes_found || metadata->nr_child > 1 ||
		    (metadata->external_nodes && topmost_external_nodes) ||
		    is_root) {
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
				ft_node_find_child(ft, cur, *detach_node_flag_ptr,
					&n, NULL);
			break;
		}
		/*
		 * Single-child node made childless by the prune that carries a
		 * shorter key: promote its external chain (the deepest such node
		 * on the path is the one promoted; prev_external_nodes_found then
		 * stops the climb at the next, surviving level).
		 */
		if (metadata->external_nodes && !topmost_external_nodes)
			topmost_external_nodes = metadata->external_nodes;
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
				/*
				 * Climb one level: cur (and its child-on-path,
				 * already at detach_node_flag_ptr's level) is pruned,
				 * so the detach target becomes cur and its parent
				 * becomes parent_nf.  The new detach_parent_flag_ptr
				 * must therefore point to PARENT_NF's slot in its own
				 * parent (so *detach_parent_flag_ptr == parent_nf ==
				 * the new cur), recovered in O(1) from parent_nf's
				 * metadata parent-slot offset.  ft_get_parent_slot
				 * handles all kinds: root (parent == NULL) -> &ft->root,
				 * compressed -> its grandparent slot, plain internal ->
				 * the body slot.  (The previous code recovered cur's own
				 * slot here, which aliases detach_parent_flag_ptr and
				 * left *detach_parent_flag_ptr != cur after the climb — a
				 * latent bug, never reached because descent callers pass
				 * the surviving-ancestor detach point and break above.)
				 */
				struct cds_ft_metadata *parent_nf_meta =
					cds_ft_item_to_metadata(ft_node_ptr(parent_nf));
				struct cds_ft_inode_flag **new_parent_flag_ptr =
					ft_get_parent_slot(parent_nf_meta, ft);
				/*
				 * Defensive: if the recovered slot aliases the current
				 * detach_parent_flag_ptr, advancing would put both
				 * pointers at the same slot, breaking the replace which
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
							next = ft_node_get_nth(ft, 
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
			/*
			 * As in the replace-ptr free-walk below: a skip-compressed
			 * external leaf at the chain end keeps its path bytes in a
			 * separate, now-orphaned skip-target compressed node that the
			 * walk stops short of.  Free it (the external leaf stays
			 * caller-owned).
			 */
			if (walk_nf && ft_node_skip_compressed(walk_nf))
				free_compressed_node(ft,
					ft_skip_to_compressed(ft, walk_nf));
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
				 *   node (possibly with FT_SKIP_MASK from an
				 *   internal skip-target; dormant
				 *   today).
				 */
				/* Phase 1: elevated ancestors. */
				while (nr_to_free < nr_clear &&
				       walk_nf &&
				       !ft_node_external(walk_nf) &&
				       nr_to_free < FT_MAX_DEPTH) {
					struct cds_ft_inode_flag *next = NULL;

					if (ft_node_skip_compressed(walk_nf)) {
						/*
						 * Skip-encoded elevated link: the slot
						 * value encodes the child BELOW the
						 * elided skip-target compressed node
						 * (which carries the path bytes).  The
						 * orphaned node is that skip-target; the
						 * child is the next link down.  Queue the
						 * target's plain compressed flag and
						 * advance to the child.  ft_node_ptr /
						 * ft_compressed_node_ptr on the raw skip
						 * value would mis-free the child (taken
						 * from its low tag) and leak the target --
						 * this is the move-style detach's single-
						 * byte-prefix case, where the child below
						 * is the preserved move target.
						 */
						struct cds_ft_compressed_node *cn =
							ft_skip_to_compressed(ft, walk_nf);

						to_free[nr_to_free++] =
							ft_compressed_node_flag(cn);
						walk_nf = ft_skip_child_ptr(walk_nf);
						continue;
					}
					if (ft_node_compressed(walk_nf)) {
						struct cds_ft_compressed_node *cn;

						cn = ft_compressed_node_ptr(walk_nf);
						next = cn->child;
					} else {
						unsigned int key;

						for (key = 0; key < 256; key++) {
							next = ft_node_get_nth(ft, 
								walk_nf, NULL,
								(uint8_t) key,
								FT_PF_NONE);
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
									next = ft_node_get_nth(ft, 
										walk_nf, NULL,
										(uint8_t) key,
										FT_PF_NONE);
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
					/*
					 * The chain stops at @walk_nf.  If that is a
					 * skip-compressed external leaf, its path bytes
					 * live in a separate skip-target compressed node:
					 * the external leaf is caller-owned, but that
					 * skip-target is trie-owned and now orphaned, and
					 * the walk above stopped at the external pointer
					 * without seeing it.  Free it here.
					 */
					if (walk_nf && ft_node_skip_compressed(walk_nf))
						free_compressed_node(ft,
							ft_skip_to_compressed(ft, walk_nf));
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
		 * no external_nodes attached, fold it via the chain-compress
		 * 4-case merge (canonical form under SKIP_COMPRESSED).
		 */
		if (iter_meta->nr_child == 1 &&
		    !iter_meta->external_nodes &&
		    iter_meta->parent != NULL) {
			ft_canonicalize_chain_compress(ft, iter_node_flag,
				iter_meta, detach_parent_flag_ptr);
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
void ft_unchain_node(struct cds_ft *ft, struct cds_ft_node **head_slot,
		struct cds_ft_node *node)
{
	struct cds_ft_node *next_node = ft_node_next(node);

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
		/* Head: prev is the (cell-build) cell flag or the flagged parent. */
		/*
		 * Head promotion: @next_node inherited @node's prev (the cell) via
		 * the copy above, so it becomes the new head sharing the same cell;
		 * retarget the cell at the promoted head (ord-list position and
		 * parent are preserved — no list surgery).  When @next_node is NULL
		 * the key disappears and the caller frees the cell.  List off:
		 * @next_node->prev is the flagged parent directly (inherited), no cell.
		 */
		if (ft->ordered_list && next_node)
			rcu_assign_pointer(ft_ord_cell_ptr(node->prev)->node,
				next_node);
		rcu_assign_pointer(*head_slot, next_node);
	}
	/*
	 * @node has left the trie: tombstone it.  Its next pointer is
	 * preserved (still == next_node) so a concurrent reader positioned
	 * on @node still follows the chain; the bit only marks removal for a
	 * later position-based remove.
	 */
	ft_node_mark_removed(node);
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
	struct cds_ft_inode_flag *holder_flag;
	struct cds_ft_metadata *holder_meta;
	struct cds_ft_inode_flag **head_slot = NULL;
	const uint8_t *iter_key;
	size_t key_len = ft_key_len(ft, ft_iter_resolve_key_len(iter));
	int ret;

	CDS_FT_SCOPED_WRITER(ft);
	FT_TP(remove_enter, (const void *) ft, (const void *) iter,
		iter_key(iter), key_len);

	/*
	 * If the iterator has a valid path, the RCU read-side lock must
	 * be held.
	 */
	if (iter->cache_valid)
		CDS_FT_ASSERT_RCU_READ_LOCKED(ft);
	iter_debug_path_check(iter);

	if (!valid_external_node(node) || !valid_key_len(ft, key_len)) {
		FT_TP(remove_exit, (int) CDS_FT_STATUS_INVALID_ARGUMENT_ERROR);
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}

	iter_key = ft_iter_read_key(iter);
	dbg_printf("cds_ft_remove attempt: node %p\n", node);

	/*
	 * No top-down descent.  @node is application-owned and, with the RCU
	 * read-side lock held continuously since it was obtained, stays
	 * alive; the writer mutex held here freezes the structure, so
	 * node->prev is a settled live pointer to the node's holder and the
	 * slot that holds @node can be derived directly:
	 *
	 *  - INTERNAL ancestors recover their parent slot from their own
	 *    metadata (parent + parent_slot_offset), used by ft_detach_node's
	 *    upward prune walk.
	 *  - the metadata-less EXTERNAL head's slot in its holder is
	 *    re-derived from the key here (a compressed holder's &cn->child,
	 *    or an internal holder's body slot for the key's last byte).
	 */

	/*
	 * A removed node carries the tombstone on node->next (set by a prior
	 * unchain/detach).  Re-removing it is an idempotent miss; this also
	 * guards against operating on a node already unlinked from the trie.
	 */
	if (ft_node_is_removed(node)) {
		dbg_printf("cds_ft_remove: node %p already removed\n", node);
		FT_TP(remove_exit, (int) CDS_FT_STATUS_NOT_FOUND);
		return CDS_FT_STATUS_NOT_FOUND;
	}

	/*
	 * Resolve @node's holder (its parent), transparently across the cell
	 * indirection: a head's prev is its cell (parent in cell->parent), a
	 * non-head duplicate's prev is its predecessor.  NULL => never inserted.
	 */
	holder_flag = ft_node_holder(ft, node);
	if (!holder_flag) {
		/* Never inserted (a freshly-initialized node). */
		dbg_printf("cds_ft_remove: node %p has no parent\n", node);
		FT_TP(remove_exit, (int) CDS_FT_STATUS_NOT_FOUND);
		return CDS_FT_STATUS_NOT_FOUND;
	}

	/*
	 * Cell-always: @node heads its chain iff its prev is the cell (not an
	 * external predecessor).  Capture the head's cell + successor BEFORE the
	 * unlink: a head promotion retargets the cell at the successor (done in
	 * ft_unchain_node), and a key disappearance (no successor) frees the
	 * cell after the removal commits (ret == 0).
	 */
	bool cell_was_head = ft->ordered_list &&
		!ft_node_external((struct cds_ft_inode_flag *) node->prev);
	struct ft_ord_cell *dead_cell = cell_was_head ?
		ft_ord_cell_ptr(node->prev) : NULL;
	struct cds_ft_node *cell_succ = cell_was_head ? ft_node_next(node) : NULL;

	if (ft_node_external(holder_flag)) {
		/*
		 * node->prev is a cds_ft_node: @node is a non-head duplicate.
		 * Unlink it from its chain (ft_unchain_node relinks the pinned
		 * predecessor/successor and tombstones @node).  The key count is
		 * unchanged (other duplicates remain) and the chain head — and
		 * any grandparent skip pointer to it — is untouched, so no head
		 * slot is needed.
		 */
		ft_unchain_node(ft, NULL, node);
		ret = 0;
	} else if (ft_node_compressed(holder_flag) ||
		   ft_node_skip_compressed(holder_flag)) {
		/*
		 * Compressed holder: @node is its single external child
		 * (cn->child).  Leaf key.
		 */
		struct cds_ft_compressed_node *cn =
			ft_node_skip_compressed(holder_flag) ?
				ft_skip_to_compressed(ft, holder_flag) :
				ft_compressed_node_ptr(holder_flag);

		holder_meta = cds_ft_item_to_metadata((struct cds_ft_inode *) cn);
		head_slot = &cn->child;
		if ((struct cds_ft_node *) ft_node_ptr(*head_slot) != node) {
			dbg_printf("cds_ft_remove: node %p not at compressed child slot\n", node);
			FT_TP(remove_exit, (int) CDS_FT_STATUS_NOT_FOUND);
			return CDS_FT_STATUS_NOT_FOUND;
		}
		if (!ft_node_next(node)) {
			/*
			 * Last/only entry: prune the now-empty branch.
			 * ft_detach_node bootstraps from the holder slot (recovered
			 * from the holder's own metadata offset) and walks up via
			 * metadata->parent.  Propagate -1 before detach, which may
			 * free internal nodes.
			 */
			ft_propagate_external_count_parent(ft, holder_flag, -1);
			ret = ft_detach_node(ft, head_slot,
				ft_get_parent_slot(holder_meta, ft),
				key_len, true);
			if (ret)
				ft_propagate_external_count_parent(ft, holder_flag, 1);
			else
				ft_node_mark_removed(node);
		} else {
			/* Removing the head, duplicates remain: key count unchanged. */
			ft_unchain_node(ft, (struct cds_ft_node **) head_slot, node);
#ifdef FEATURE_FT_SKIP_COMPRESSED
			/*
			 * Unchaining replaced cn->child with the next entry, but
			 * the grandparent's skip-compressed slot still encodes the
			 * OLD head, which the caller is about to call_rcu-free.  A
			 * candidate descent or ft_skip_reanchor up-walk following
			 * the stale skip pointer would dereference the freed node
			 * (the dangling-skip-slot UAF).  Re-encode the grandparent
			 * slot (recovered from cn's parent-slot offset) to the new
			 * cn->child; ordered before the caller's free.
			 * ft_update_skip_pointer no-ops when the slot holds a plain
			 * (non-skip) compressed pointer.
			 */
			ft_update_skip_pointer(ft_get_parent_slot(holder_meta, ft),
				cn);
#endif
			ret = 0;
		}
	} else if (ft_node_external_nodes(holder_flag) ==
			(struct cds_ft_node *) node) {
		/*
		 * Internal holder whose external_nodes chain head is @node:
		 * prefix key (the key terminates at an internal node that also
		 * has longer-key children).  The holder stays; unlink @node from
		 * its external_nodes chain.  Propagate -1 only when @node is the
		 * last entry (the chain becomes empty).
		 */
		holder_meta = cds_ft_item_to_metadata(ft_node_ptr(holder_flag));
		if (!ft_node_next(node))
			ft_propagate_external_count_parent(ft, holder_flag, -1);
		ft_unchain_node(ft, (struct cds_ft_node **) &holder_meta->external_nodes,
			node);
#ifdef FEATURE_FT_SKIP_COMPRESSED
		/*
		 * If the unchain emptied the external chain and the holder now
		 * has exactly one child + no external, canonicalize via
		 * chain-compress to restore the SKIP_COMPRESSED invariant (no
		 * non-root 1-child internal without external).  The holder's own
		 * slot in its parent is recovered from its metadata offset.
		 */
		if (ft_group_skip_compressed(ft->group) &&
		    !holder_meta->external_nodes &&
		    holder_meta->nr_child == 1 &&
		    holder_meta->parent != NULL) {
			ft_canonicalize_chain_compress(ft, holder_flag,
				holder_meta, ft_get_parent_slot(holder_meta, ft));
		}
#endif
		ret = 0;
	} else {
		/*
		 * Internal holder, @node is a body child: leaf key.  Recover the
		 * holder's body slot for @node from the key's last byte.
		 */
		struct cds_ft_inode_flag *child;

		holder_meta = cds_ft_item_to_metadata(ft_node_ptr(holder_flag));
		child = ft_node_get_nth_skip(holder_flag, &head_slot,
			iter_key[key_len - 1], FT_PF_NONE);
		if (!child ||
		    (struct cds_ft_node *) ft_node_ptr(child) != node) {
			dbg_printf("cds_ft_remove: node %p not at key slot\n", node);
			FT_TP(remove_exit, (int) CDS_FT_STATUS_NOT_FOUND);
			return CDS_FT_STATUS_NOT_FOUND;
		}
		if (!ft_node_next(node)) {
			/*
			 * Last/only entry: prune the now-empty branch.
			 * Propagate -1 before detach, which may free internal nodes.
			 */
			ft_propagate_external_count_parent(ft, holder_flag, -1);
			ret = ft_detach_node(ft, head_slot,
				ft_get_parent_slot(holder_meta, ft),
				key_len, true);
			if (ret)
				ft_propagate_external_count_parent(ft, holder_flag, 1);
			else
				ft_node_mark_removed(node);
		} else {
			/* Removing the head, duplicates remain: key count unchanged. */
			ft_unchain_node(ft, (struct cds_ft_node **) head_slot, node);
			ret = 0;
		}
	}

	/*
	 * Head with no successor: the key disappeared, so its cell is unspliced
	 * from the ordered list (when enabled) and freed (deferred, for parked
	 * up-walkers).  A promotion (cell_succ) keeps the cell in place — same
	 * key, only cell->node retargeted in ft_unchain_node — so no list op.
	 */
	if (ret == 0 && cell_was_head && !cell_succ) {
		/* cell_was_head implies ordered_list, so the list op always runs. */
		ft_ord_cell_unsplice(ft, dead_cell);
		ft_ord_cell_free(ft, dead_cell);
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
	iter->cache_valid = false;
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

/*
 * ft_locate_chain_head: derive an external chain head's position with no
 * descent, from the head itself (head->prev is its holder) and the key.
 *
 * Sets *holder_flag_p (the head's internal/compressed holder), *is_prefix_p
 * (true when the chain hangs off the holder's external_nodes — a prefix key —
 * rather than a body/compressed child slot), and, for the non-prefix case,
 * *head_slot_p (the holder slot that holds @head: &cn->child for a compressed
 * holder, or the body slot for the key's last byte for an internal holder).
 *
 * Returns true iff @head is the live chain head at that position (so a cached
 * iter->node is still current under the writer mutex): the prefix case is
 * validated by holder->external_nodes == head; the leaf case by *head_slot ==
 * head.  Returns false when @head is a non-head duplicate, has no holder, or
 * the holder no longer points at it (stale cache) — the caller then re-seeds
 * via a fresh lookup.
 */
static
bool ft_locate_chain_head(struct cds_ft *ft, struct cds_ft_node *head,
		const uint8_t *iter_key, size_t key_len,
		struct cds_ft_inode_flag **holder_flag_p,
		struct cds_ft_inode_flag ***head_slot_p,
		bool *is_prefix_p)
{
	struct cds_ft_inode_flag *holder_flag = ft_node_holder(ft, head);

	if (!holder_flag || ft_node_external(holder_flag))
		return false;	/* no holder, or @head is a non-head duplicate */
	*holder_flag_p = holder_flag;
	if (ft_node_compressed(holder_flag) ||
	    ft_node_skip_compressed(holder_flag)) {
		struct cds_ft_compressed_node *cn =
			ft_node_skip_compressed(holder_flag) ?
				ft_skip_to_compressed(ft, holder_flag) :
				ft_compressed_node_ptr(holder_flag);

		*is_prefix_p = false;
		*head_slot_p = &cn->child;
	} else if (ft_node_external_nodes(holder_flag) ==
			(struct cds_ft_node *) head) {
		*is_prefix_p = true;
		*head_slot_p = NULL;	/* external_nodes, not a body slot */
		return true;
	} else {
		struct cds_ft_inode_flag *child;

		*is_prefix_p = false;
		child = ft_node_get_nth_skip(holder_flag, head_slot_p,
			iter_key[key_len - 1], FT_PF_NONE);
		if (!child)
			return false;
	}
	return (struct cds_ft_node *) ft_node_ptr(**head_slot_p) == head;
}

enum cds_ft_status cds_ft_remove_all(struct cds_ft *ft,
		struct cds_ft_iter *iter,
		struct cds_ft_node **result_node)
{
	struct cds_ft_node *chain_head;
	struct cds_ft_inode_flag *holder_flag;
	struct cds_ft_metadata *holder_meta;
	struct cds_ft_inode_flag **head_slot;
	bool is_prefix;
	int ret;
	const uint8_t *iter_key;
	size_t key_len = ft_key_len(ft, ft_iter_resolve_key_len(iter));

	CDS_FT_SCOPED_WRITER(ft);
	/*
	 * If the iterator has a valid path, the RCU read-side lock must
	 * be held.
	 */
	if (iter->cache_valid)
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
		/* Ordered list on: the head's cell leaves the trie (capture before
		 * mark_removed, though that only tombstones ->next).  List off: none. */
		if (ft->ordered_list) {
			struct ft_ord_cell *dead = ft_ord_cell_ptr(external_nodes->prev);

			ft_ord_cell_unsplice(ft, dead);
			ft_ord_cell_free(ft, dead);
		}
		/* The whole chain has left the trie: tombstone every node. */
		ft_chain_mark_removed(external_nodes);
		/* The mutation invalidates the cached position (general-path parity). */
		iter->cache_valid = false;
		iter_debug_path_clear(iter);
		iter->path_len = 0;
		return CDS_FT_STATUS_OK;
	}

	iter_key = ft_iter_read_key(iter);
	dbg_printf("cds_ft_remove_all attempt\n");

	/*
	 * No top-down descent: anchor on the cached chain head (iter->node)
	 * under the writer mutex.  If it is still the live head at iter->key
	 * (ft_locate_chain_head validates *head_slot == iter->node), use it;
	 * otherwise re-seed via a fresh exact lookup (the standard positioning
	 * descent, NOT a remove-specific re-descent) and locate from there.
	 */
	chain_head = NULL;
	if (iter->cache_valid && iter->node && !ft_node_is_removed(iter->node) &&
	    ft_locate_chain_head(ft, iter->node, iter_key, key_len,
		    &holder_flag, &head_slot, &is_prefix))
		chain_head = iter->node;
	if (!chain_head) {
		enum cds_ft_status s = (*ft->lookup_iter_fn)(ft, iter);

		if (s != CDS_FT_STATUS_OK || !iter->node ||
		    !ft_locate_chain_head(ft, iter->node, iter_key, key_len,
			    &holder_flag, &head_slot, &is_prefix)) {
			*result_node = NULL;
			return CDS_FT_STATUS_NOT_FOUND;
		}
		chain_head = iter->node;
	}
	holder_meta = cds_ft_item_to_metadata(ft_node_ptr(holder_flag));
	*result_node = chain_head;

	if (is_prefix) {
		/*
		 * Prefix key: the whole chain hangs off the internal holder's
		 * external_nodes.  Clear it (one key) and tombstone the chain.
		 * The holder stays while it has children; if clearing leaves it
		 * empty (nr_child == 0) detach its branch, else canonicalize a
		 * now-single-child node (skip builds).  Propagate -1 before any
		 * detach (undercount ordering).
		 */
		ft_propagate_external_count_parent(ft, holder_flag, -1);
		rcu_assign_pointer(holder_meta->external_nodes, NULL);
		ft_chain_mark_removed(chain_head);
		ret = 0;
		if (holder_meta->nr_child == 0 && holder_meta->parent) {
			struct cds_ft_metadata *parent_meta =
				cds_ft_item_to_metadata(
					ft_node_ptr(holder_meta->parent));

			ret = ft_detach_node(ft,
				ft_get_parent_slot(holder_meta, ft),
				ft_get_parent_slot(parent_meta, ft),
				key_len, true);
			if (ret) {
				/*
				 * Pruning the emptied holder branch failed
				 * (recompact ENOMEM).  The REMOVAL itself
				 * already committed above -- count propagated,
				 * external_nodes cleared, chain tombstoned --
				 * so the count must NOT be re-added (the key is
				 * gone; the old +1 here left a permanent
				 * ancestor overcount) and the operation did not
				 * fail: report success, run the ordered-list
				 * unsplice below, and leave the holder as a
				 * reachable-but-empty internal (readers
				 * dead-end at it, cds_ft_verify accepts it; a
				 * later mutation through the slot prunes it).
				 */
				ret = 0;
			}
		}
#ifdef FEATURE_FT_SKIP_COMPRESSED
		else if (ft_group_skip_compressed(ft->group) &&
			 holder_meta->nr_child == 1 &&
			 holder_meta->parent != NULL) {
			ft_canonicalize_chain_compress(ft, holder_flag,
				holder_meta, ft_get_parent_slot(holder_meta, ft));
		}
#endif
	} else {
		/*
		 * Leaf key: the whole chain sits at head_slot (a body slot, or
		 * a compressed holder's &cn->child).  Removing it empties the
		 * slot, so prune the branch via ft_detach_node bootstrapped from
		 * the holder (it climbs via metadata->parent).  Propagate -1
		 * before detach (which may free internal nodes).
		 */
		ft_propagate_external_count_parent(ft, holder_flag, -1);
		ret = ft_detach_node(ft, head_slot,
			ft_get_parent_slot(holder_meta, ft), key_len, true);
		if (ret)
			ft_propagate_external_count_parent(ft, holder_flag, 1);
		else
			ft_chain_mark_removed(chain_head);
	}

	/*
	 * detach should not replace a NULL pointer because it has been
	 * found by a mutex-protected traversal within this function.
	 */
	assert(ret != -ENOENT);

	/* Ordered list on: the whole key left the trie: its head's cell is
	 * unspliced and freed (deferred).  chain_head->prev still carries the cell
	 * (detach reshapes ancestors and head_slot, not the head's prev).  List
	 * off: chain_head->prev is the flagged parent, no cell. */
	if (ret == 0 && ft->ordered_list) {
		struct ft_ord_cell *dead = ft_ord_cell_ptr(chain_head->prev);

		ft_ord_cell_unsplice(ft, dead);
		ft_ord_cell_free(ft, dead);
	}

	iter->cache_valid = false;
	iter_debug_path_clear(iter);
	iter->path_len = 0;

	if (ret) {
		/*
		 * Leaf-key detach ENOMEM: nothing was published (the chain is
		 * still live in the trie; the count undo above restored the
		 * ancestors).  Surface the real error with a NULL out-param --
		 * the header contract -- so the caller cannot reclaim the
		 * still-reachable chain.
		 */
		*result_node = NULL;
		return CDS_FT_STATUS_MEMORY_ERROR;
	}

	return CDS_FT_STATUS_OK;
}


/*
 * Build-invisible diverge split for cds_ft_graft (the merge variant of
 * ft_split_compressed_graft).  Where the legacy split publishes a 1-child
 * branch for ft_store_at_graft_point to complete — leaving a non-canonical
 * internal live if that later, fallible attach OOMs — this builds the
 * COMPLETE attach cluster invisibly:
 *
 *   [prefix] -> branch{ old_ordinal -> old_suffix -> old_child,
 *                       new_ordinal -> [path] -> payload }
 *
 * Nothing is published, @cn is not freed, and every edge into LIVE data
 * (the displaced @old_child and the live @payload nodes) is recorded as a
 * deferred back-pointer in @glue.  An OOM frees the cluster via the
 * caller's ft_graft_glue_abort with both tries pristine.
 *
 * The branch is built with BOTH children up front (two ft_node_set_nth
 * calls, the second possibly reallocating the node) using cluster_leaf so
 * neither child's back-pointer is set during the build; both are then
 * deferred against the FINAL branch.  This avoids the
 * publish-then-complete window and keeps the deferred old-child edge
 * anchored to a stable node.
 *
 * @key/@key_len: full ordinal key being grafted.
 * @diverge_pos:  divergence offset within cn's path (cn->key_bytes).
 * @payload:      source's old root (live), attached at depth @key_len.
 * @src_count:    payload key count, for the deferred propagate.
 *
 * Returns 0 (glue holds the cluster, its publish, and attached_nf), or
 * -ENOMEM (caller runs ft_graft_glue_abort).
 */
static
int ft_split_compressed_graft_build(struct cds_ft *ft,
		struct ft_descent *d,
		const uint8_t *key, size_t key_len,
		unsigned int diverge_pos,
		struct cds_ft_inode_flag *payload,
		unsigned long src_count,
		struct ft_graft_glue *glue)
{
	struct cds_ft_compressed_node *cn = ft_compressed_node_ptr(d->nf);
	struct cds_ft_metadata *cn_meta =
		cds_ft_item_to_metadata((struct cds_ft_inode *) cn);

	/* Compressed metadata never carries external_nodes (see
	 * ft_split_compressed_insert); the dead external-carrying prefix
	 * arms this retires included a latent diverge_pos == 2 sub-case
	 * that dropped prefix byte key_bytes[1]. */
	assert(!cn_meta->external_nodes);
	unsigned int suffix_len = cn->len - diverge_pos - 1;
	uint8_t old_ordinal = cn->key_bytes[diverge_pos];
	uint8_t new_ordinal = key[d->depth + diverge_pos];
	unsigned int new_depth = d->depth + diverge_pos + 1;
	bool branch_cluster_leaf = (suffix_len == 0);
	struct cds_ft_inode_flag *old_suffix_flag;
	struct cds_ft_inode_flag *sfx_skip_flag = NULL;
	struct cds_ft_inode_flag *branch_flag, *top_flag;
	struct cds_ft_inode_flag *new_dir, *payload_canon;
	struct cds_ft_inode_flag **slot;
	struct cds_ft_inode *old_branch = NULL;
	unsigned long old_child_nr_keys;
	int ret;

	(void) branch_cluster_leaf;	/* documents intent; both set_nth defer */

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

	/*
	 * 1. Build the OLD-direction suffix -> old child (mirrors the legacy
	 * split).  cn->child (live) is deferred into @glue.
	 */
	if (suffix_len >= 2
#ifdef FEATURE_FT_SKIP_COMPRESSED
			|| (suffix_len == 1 && ft_group_skip_compressed(ft->group))
#endif
	   ) {
		struct cds_ft_compressed_node *sfx;
		struct cds_ft_metadata *sfx_meta;

		sfx = alloc_compressed_node(ft, suffix_len, &sfx_meta);
		if (!sfx)
			return -ENOMEM;
		sfx->child = cn->child;
		sfx->len = suffix_len;
		memcpy(sfx->key_bytes, &cn->key_bytes[diverge_pos + 1],
			suffix_len);
		sfx_meta->nr_child = 1;
		ft_nr_keys_store(sfx_meta, old_child_nr_keys, CMM_RELAXED);
		old_suffix_flag = ft_compressed_node_flag(sfx);
		sfx_skip_flag = ft_publish_compressed(ft, sfx, old_suffix_flag);
		ft_graft_glue_track(glue, old_suffix_flag);
		ft_graft_glue_defer_edge(ft, glue, cn->child, old_suffix_flag,
			&sfx->child);
	} else if (suffix_len == 1) {
		struct cds_ft_inode_flag *dest = NULL;

		/* 1-child internal suffix (non-SC): cluster-leaf. */
		ret = ft_node_set_nth(ft, &dest,
				cn->key_bytes[diverge_pos + 1],
				cn->child, NULL, NULL,
				d->depth + diverge_pos + 1, true);
		if (ret)
			return -ENOMEM;
		ft_nr_keys_store(cds_ft_item_to_metadata(ft_node_ptr(dest)),
			old_child_nr_keys, CMM_RELAXED);
		old_suffix_flag = dest;
		ft_graft_glue_track(glue, dest);
		ft_node_get_nth_skip(dest, &slot,
			cn->key_bytes[diverge_pos + 1], FT_PF_NONE);
		ft_graft_glue_defer_edge(ft, glue, cn->child, dest, slot);
	} else {
		old_suffix_flag = cn->child;	/* suffix_len == 0 */
	}

	/*
	 * 2. Build the NEW-direction subtree: canonicalize the payload, then
	 * (when the key extends past the branch) a path down to it.  All
	 * build-invisible; payload back-pointers deferred via @glue.
	 */
	payload_canon = ft_compress_single_child_if_needed(ft, payload, glue);
	if (payload_canon == (struct cds_ft_inode_flag *) (long) -ENOMEM)
		return -ENOMEM;
	if (new_depth == key_len) {
		new_dir = payload_canon;
	} else {
		new_dir = ft_build_branch(ft, key, new_depth, key_len,
			payload_canon, src_count, false, glue);
		if (!new_dir)
			return -ENOMEM;
	}

	/*
	 * 3. Build the branch with BOTH children.  cluster_leaf on both
	 * set_nth: no child back-pointer is set during the build (the second
	 * set_nth may reallocate the branch).  Both are deferred below
	 * against the final branch.
	 */
	branch_flag = NULL;
	ret = ft_node_set_nth(ft, &branch_flag, old_ordinal, old_suffix_flag,
			NULL, NULL, d->depth + diverge_pos, true);
	if (ret)
		return -ENOMEM;
	ft_graft_glue_track(glue, branch_flag);
	/*
	 * The branch is a fresh, unpublished node with no parent yet (it is
	 * wired to its prefix only at commit).  Clear its parent / skip_slot
	 * before the second child may reallocate it: ft_node_recompact
	 * inherits the old node's parent and, if that parent looks like a
	 * compressed node, writes through its skip_slot — a recycled
	 * allocation can leave stale, non-NULL values there and corrupt an
	 * unrelated live node.
	 */
	{
		struct cds_ft_metadata *bm =
			cds_ft_item_to_metadata(ft_node_ptr(branch_flag));

		bm->parent = NULL;
#ifdef FEATURE_FT_SKIP_COMPRESSED
		bm->parent_slot_offset = 0;
#endif
	}
	/*
	 * Second child: the branch is now an existing (unpublished) node,
	 * so pass its metadata for the in-place update; on overflow it
	 * reallocates (old order-1 copy returned via @old_branch).
	 */
	ret = ft_node_set_nth(ft, &branch_flag, new_ordinal, new_dir,
			&old_branch,
			cds_ft_item_to_metadata(ft_node_ptr(branch_flag)),
			d->depth + diverge_pos, true);
	if (ret)
		return -ENOMEM;
	if (old_branch) {
		/* Reallocated: drop the order-1 copy from tracking + free it. */
		ft_graft_glue_untrack(ft, glue, old_branch);
		free_cds_ft_node_unpublished(ft, old_branch);
		ft_graft_glue_track(glue, branch_flag);
	}
	ft_nr_keys_store(cds_ft_item_to_metadata(ft_node_ptr(branch_flag)),
		old_child_nr_keys, CMM_RELAXED);

	/* Wire the OLD direction (re-encode compressed slot to skip form). */
	ft_node_get_nth_skip(branch_flag, &slot, old_ordinal, FT_PF_NONE);
	if (sfx_skip_flag && sfx_skip_flag != old_suffix_flag && slot)
		rcu_assign_pointer(*slot, sfx_skip_flag);
	ft_graft_glue_defer_edge(ft, glue, old_suffix_flag, branch_flag, slot);
	/* Wire the NEW direction. */
	ft_node_get_nth_skip(branch_flag, &slot, new_ordinal, FT_PF_NONE);
	if (ft_node_compressed(new_dir) && slot) {
		/*
		 * @new_dir is a PLAIN compressed flag (compress and the
		 * build-invisible ft_build_branch both return the plain form
		 * so the deferred edge recovers it via ft_compressed_node_ptr).
		 * Re-encode the holding slot to the skip form so the published
		 * trie is canonical; it resolves once the deferred grandchild
		 * back-pointer is applied at commit.
		 */
		struct cds_ft_inode_flag *skip = ft_publish_compressed(ft,
			ft_compressed_node_ptr(new_dir), new_dir);

		if (skip != new_dir)
			rcu_assign_pointer(*slot, skip);
	}
	ft_graft_glue_defer_edge(ft, glue, new_dir, branch_flag, slot);

	/* 4. Build prefix -> branch (no external_nodes on @cn). */
	if (diverge_pos >= 2) {
		struct cds_ft_compressed_node *pfx;
		struct cds_ft_metadata *pfx_meta;

		pfx = alloc_compressed_node(ft, diverge_pos, &pfx_meta);
		if (!pfx)
			return -ENOMEM;
		pfx->child = branch_flag;
		pfx->len = diverge_pos;
		memcpy(pfx->key_bytes, cn->key_bytes, diverge_pos);
		pfx_meta->nr_child = 1;
		ft_nr_keys_store(pfx_meta, ft_nr_keys_get(cn_meta), CMM_RELAXED);
		top_flag = ft_compressed_node_flag(pfx);
		ft_set_parent(ft, branch_flag, top_flag, NULL);
		/* Track the PLAIN form; the skip form is for the publish. */
		ft_graft_glue_track(glue, top_flag);
		top_flag = ft_publish_compressed(ft, pfx, top_flag);
	} else if (diverge_pos == 1) {
#ifdef FEATURE_FT_SKIP_COMPRESSED
		if (ft_group_skip_compressed(ft->group)) {
			struct cds_ft_compressed_node *pfx;
			struct cds_ft_metadata *pfx_meta;

			pfx = alloc_compressed_node(ft, 1, &pfx_meta);
			if (!pfx)
				return -ENOMEM;
			pfx->child = branch_flag;
			pfx->len = 1;
			pfx->key_bytes[0] = cn->key_bytes[0];
			pfx_meta->nr_child = 1;
			ft_nr_keys_store(pfx_meta, ft_nr_keys_get(cn_meta),
				CMM_RELAXED);
			top_flag = ft_compressed_node_flag(pfx);
			ft_set_parent(ft, branch_flag, top_flag, &pfx->child);
			/* Track the PLAIN form; skip form for the publish. */
			ft_graft_glue_track(glue, top_flag);
			top_flag = ft_publish_compressed(ft, pfx, top_flag);
			goto after_prefix;
		}
#endif
		{
		struct cds_ft_inode_flag *dest = NULL;
		struct cds_ft_metadata *pfx_meta;

		ret = ft_node_set_nth(ft, &dest, cn->key_bytes[0],
				branch_flag, NULL, NULL, d->depth, false);
		if (ret)
			return -ENOMEM;
		pfx_meta = cds_ft_item_to_metadata(ft_node_ptr(dest));
		ft_nr_keys_store(pfx_meta, ft_nr_keys_get(cn_meta), CMM_RELAXED);
		top_flag = dest;
		ft_graft_glue_track(glue, dest);
		}
#ifdef FEATURE_FT_SKIP_COMPRESSED
	after_prefix:
		(void) 0;
#endif
	} else {
		/* diverge_pos == 0: branch IS the top. */
		ft_nr_keys_store(cds_ft_item_to_metadata(ft_node_ptr(branch_flag)),
			ft_nr_keys_get(cn_meta), CMM_RELAXED);
		top_flag = branch_flag;
	}

	/*
	 * 5. Record the single publish (top -> d->pnf's slot) and the old
	 * compressed node to free at commit.  attached_nf == new_dir: it
	 * carries the payload's key count, and the propagate starts from its
	 * parent.
	 */
	ft_graft_glue_set_publish(ft, glue, d->pnf, d->nfp, top_flag);
	ft_graft_glue_defer_free(glue, cn, true);
	glue->attached_nf = new_dir;
	return 0;
}


/*
 * Graft-transaction glue helpers.  The struct and the rationale are
 * defined up near ft_build_branch (the type is referenced by the
 * build-invisible builders that precede this point).
 */
static
void ft_graft_glue_init(struct ft_graft_glue *g)
{
	g->deferred = g->deferred_floor;
	g->nr_deferred = 0;
	g->cap_deferred = FT_GRAFT_GLUE_FLOOR_DEFERRED;
	g->free_list = g->free_floor;
	g->nr_free = 0;
	g->cap_free = FT_GRAFT_GLUE_FLOOR_FREE;
	g->built = g->built_floor;
	g->nr_built = 0;
	g->cap_built = FT_GRAFT_GLUE_FLOOR_BUILT;
	g->splices = g->splices_floor;
	g->nr_splices = 0;
	g->cap_splices = FT_GRAFT_GLUE_FLOOR_SPLICE;
	g->publish_parent = NULL;
	g->publish_slot = NULL;
	g->top = NULL;
	g->attached_nf = NULL;
}

/*
 * Release a glue's malloc'd backing (when it grew past the inline floor) and
 * reset the arrays to the floor, so a second call is a no-op.  Both the abort
 * path (via ft_graft_glue_abort) and the success path call this; graft /
 * graft_swap stay on the floor, so it does nothing for them.
 */
static
void ft_graft_glue_fini(struct ft_graft_glue *g)
{
	if (g->deferred != g->deferred_floor) {
		free(g->deferred);
		g->deferred = g->deferred_floor;
		g->cap_deferred = FT_GRAFT_GLUE_FLOOR_DEFERRED;
	}
	if (g->free_list != g->free_floor) {
		free(g->free_list);
		g->free_list = g->free_floor;
		g->cap_free = FT_GRAFT_GLUE_FLOOR_FREE;
	}
	if (g->built != g->built_floor) {
		free(g->built);
		g->built = g->built_floor;
		g->cap_built = FT_GRAFT_GLUE_FLOOR_BUILT;
	}
	if (g->splices != g->splices_floor) {
		free(g->splices);
		g->splices = g->splices_floor;
		g->cap_splices = FT_GRAFT_GLUE_FLOOR_SPLICE;
	}
}

/*
 * Grow the three arrays onto a malloc'd backing so the build can hold a
 * cluster larger than the inline floor (cds_ft_merge_at's tree-shaped spine).
 * Sizes come from a read-only counting pre-pass; call once, right after init,
 * before any track / defer.  A request at or below a floor leaves that array
 * inline.  Returns 0, or -ENOMEM (whatever already grew is released by a
 * later abort / fini, so the caller need only surface the error).
 */
static
int ft_graft_glue_reserve(struct ft_graft_glue *g,
		int nr_built, int nr_deferred, int nr_free, int nr_splices)
{
	if (nr_built > g->cap_built) {
		struct cds_ft_inode_flag **p =
			malloc((size_t) nr_built * sizeof(*p));

		if (!p)
			return -ENOMEM;
		memcpy(p, g->built, (size_t) g->nr_built * sizeof(*p));
		if (g->built != g->built_floor)
			free(g->built);
		g->built = p;
		g->cap_built = nr_built;
	}
	if (nr_deferred > g->cap_deferred) {
		struct ft_graft_deferred_edge *p =
			malloc((size_t) nr_deferred * sizeof(*p));

		if (!p)
			return -ENOMEM;
		memcpy(p, g->deferred, (size_t) g->nr_deferred * sizeof(*p));
		if (g->deferred != g->deferred_floor)
			free(g->deferred);
		g->deferred = p;
		g->cap_deferred = nr_deferred;
	}
	if (nr_free > g->cap_free) {
		struct ft_graft_free_item *p =
			malloc((size_t) nr_free * sizeof(*p));

		if (!p)
			return -ENOMEM;
		memcpy(p, g->free_list, (size_t) g->nr_free * sizeof(*p));
		if (g->free_list != g->free_floor)
			free(g->free_list);
		g->free_list = p;
		g->cap_free = nr_free;
	}
	if (nr_splices > g->cap_splices) {
		struct ft_graft_splice *p =
			malloc((size_t) nr_splices * sizeof(*p));

		if (!p)
			return -ENOMEM;
		memcpy(p, g->splices, (size_t) g->nr_splices * sizeof(*p));
		if (g->splices != g->splices_floor)
			free(g->splices);
		g->splices = p;
		g->cap_splices = nr_splices;
	}
	return 0;
}

/*
 * Record the single forward publish that splices the built cluster into
 * dst, and wire the cluster top's back-pointer into its (live)
 * publish_parent.  The builders call this once the cluster is fully
 * built.  top is fresh and unpublished, so ft_graft_glue_defer_edge
 * stores top->parent IMMEDIATELY (its fresh-child fast path); the store
 * lands before any other commit-time mutation, so by the time
 * apply_deferred flips any live back-pointer into the cluster, the
 * up-walk path from cluster nodes through top into publish_parent is
 * already wired.
 */
static
void ft_graft_glue_set_publish(struct cds_ft *ft, struct ft_graft_glue *g,
		struct cds_ft_inode_flag *parent_nf,
		struct cds_ft_inode_flag **parent_slot,
		struct cds_ft_inode_flag *top)
{
	g->publish_parent = parent_nf;
	g->publish_slot = parent_slot;
	g->top = top;
	ft_graft_glue_defer_edge(ft, g, top, parent_nf, parent_slot);
}

/* Record a fresh glue node so the abort path can free it. */
static
void ft_graft_glue_track(struct ft_graft_glue *g,
		struct cds_ft_inode_flag *nf)
{
	assert(g->nr_built < g->cap_built);
	/*
	 * PLAIN forms only: a SKIP pointer encodes the CHILD's address, so
	 * the abort path's kind dispatch would free the wrong node (the 2.1
	 * corruption shape) and the identity helpers would have to chase the
	 * child's back-pointer mid-build.  Callers track the plain compressed
	 * flag and re-encode separately for the slot publish.
	 */
	assert(!ft_node_skip_compressed(nf));
	g->built[g->nr_built++] = nf;
}

/*
 * Drop a tracked glue node that has been consumed (chain-merge absorbs a
 * freshly-built compressed wrapper and frees it during the build).  Match
 * by underlying node identity so a plain-flag tracking entry is found
 * even when the absorbed reference is skip-encoded.  No-op if absent.
 */
static
void ft_graft_glue_untrack(struct cds_ft *ft, struct ft_graft_glue *g, void *node_ptr)
{
	int i;

	for (i = 0; i < g->nr_built; i++) {
		struct cds_ft_inode_flag *nf = g->built[i];
		void *p;

		if (ft_node_compressed(nf))
			p = ft_compressed_node_ptr(nf);
		else if (ft_node_skip_compressed(nf))
			p = ft_skip_to_compressed(ft, nf);
		else
			p = ft_node_ptr(nf);
		if (p == node_ptr) {
			g->built[i] = g->built[--g->nr_built];
			return;
		}
	}
}

/*
 * Test whether @child names a node currently in the glue's tracked
 * (fresh, unpublished) set.  Match by underlying node identity so a
 * plain-flag tracking entry is found even when @child is the skip-
 * encoded form (or vice versa).
 */
static
bool ft_graft_glue_is_fresh(struct cds_ft *ft, struct ft_graft_glue *g,
		struct cds_ft_inode_flag *child)
{
	void *cp;
	int i;

	if (!child)
		return false;
	if (ft_node_compressed(child))
		cp = ft_compressed_node_ptr(child);
	else if (ft_node_skip_compressed(child))
		cp = ft_skip_to_compressed(ft, child);
	else if (ft_node_external(child))
		return false;	/* externals are never glue-tracked */
	else
		cp = ft_node_ptr(child);
	for (i = 0; i < g->nr_built; i++) {
		struct cds_ft_inode_flag *nf = g->built[i];
		void *bp;

		if (ft_node_compressed(nf))
			bp = ft_compressed_node_ptr(nf);
		else if (ft_node_skip_compressed(nf))
			bp = ft_skip_to_compressed(ft, nf);
		else
			bp = ft_node_ptr(nf);
		if (bp == cp)
			return true;
	}
	return false;
}

/*
 * Wire a back-pointer (@child->parent = @parent, slot @slot) within a
 * build-invisible graft cluster.
 *
 * Two regimes by @child kind, the rule that the user pinned down:
 *
 *   - Internal-to-glue (@child is a freshly-allocated, glue-tracked node):
 *     store IMMEDIATELY.  The child is unpublished, so the rcu_assign_pointer
 *     is invisible to readers; doing it now (rather than deferring) means
 *     the entire intra-cluster back-pointer chain is fully wired by the
 *     time apply_deferred flips any LIVE-data back-pointer into the
 *     cluster.  An up-walk from re-parented live data then sees a coherent
 *     chain from the live edge through the cluster up into publish_parent
 *     -- no transient NULL parents along the way.
 *
 *   - External-to-glue (@child is a LIVE node being re-parented INTO the
 *     glue cluster, e.g. the displaced old child of a diverge split or
 *     the source-payload leaf at the bottom of the new branch): record
 *     the (child, parent, slot) tuple and defer the ft_set_parent until
 *     ft_graft_glue_apply_deferred at commit time, after the source has
 *     been drained.  Flipping a live back-pointer during the build would
 *     be a publication-visible mutation before the cluster is observable.
 *
 *   Deferred entries are de-duplicated by @child so a canonicalization
 *   wrapper later absorbed by a chain-merge keeps only its final mapping.
 */
static
void ft_graft_glue_defer_edge_origin(struct cds_ft *ft, struct ft_graft_glue *g,
		struct cds_ft_inode_flag *child,
		struct cds_ft_inode_flag *parent,
		struct cds_ft_inode_flag **slot,
		bool dst_origin)
{
	int i;

	if (ft_graft_glue_is_fresh(ft, g, child)) {
		ft_set_parent(ft, child, parent, slot);
		return;
	}
	for (i = 0; i < g->nr_deferred; i++) {
		if (g->deferred[i].child == child) {
			g->deferred[i].parent = parent;
			g->deferred[i].slot = slot;
			g->deferred[i].dst_origin = dst_origin;
			return;
		}
	}
	assert(g->nr_deferred < g->cap_deferred);
	g->deferred[g->nr_deferred].child = child;
	g->deferred[g->nr_deferred].parent = parent;
	g->deferred[g->nr_deferred].slot = slot;
	g->deferred[g->nr_deferred].dst_origin = dst_origin;
	g->nr_deferred++;
}

static
void ft_graft_glue_defer_edge(struct cds_ft *ft, struct ft_graft_glue *g,
		struct cds_ft_inode_flag *child,
		struct cds_ft_inode_flag *parent,
		struct cds_ft_inode_flag **slot)
{
	ft_graft_glue_defer_edge_origin(ft, g, child, parent, slot,
		/*dst_origin*/ false);
}

/* Record an old (replaced) live node to reclaim deferred at commit. */
static
void ft_graft_glue_defer_free(struct ft_graft_glue *g,
		void *node, bool compressed)
{
	assert(g->nr_free < g->cap_free);
	g->free_list[g->nr_free].node = node;
	g->free_list[g->nr_free].compressed = compressed;
	g->nr_free++;
}

/*
 * Abort the build: free every freshly-built (never-observed) glue node, then
 * release the malloc'd backing.  Both tries are left pristine — no deferred
 * edge was applied, so no live back-pointer references the glue.
 */
static
void ft_graft_glue_abort(struct cds_ft *ft, struct ft_graft_glue *g)
{
	int i;

	for (i = 0; i < g->nr_built; i++) {
		struct cds_ft_inode_flag *nf = g->built[i];

		if (ft_node_compressed(nf))
			free_compressed_node_unpublished(ft,
				ft_compressed_node_ptr(nf));
		else if (ft_node_skip_compressed(nf))
			free_compressed_node_unpublished(ft,
				ft_skip_to_compressed(ft, nf));
		else
			free_cds_ft_node_unpublished(ft, ft_node_ptr(nf));
	}
	ft_graft_glue_fini(g);
}

/*
 * Commit step 1: wire the deferred LIVE back-pointers.  Call after the
 * source unlink + grace period, before the forward publish of the
 * cluster top, so an up-walk from any re-parented live node enters the
 * new cluster before the old nodes are forward-detached and freed.
 *
 * Only edges whose CHILD is a live (observable) node are deferred --
 * setting a live node's parent is a publication-visible mutation that
 * must wait for sync_rcu.  Fresh-to-fresh edges inside the cluster are
 * set IMMEDIATELY during the build (the child is unpublished, the store
 * has no reader-visible effect, and by commit time the entire internal
 * chain from any live re-parent target up to publish_parent is already
 * in place).  Iteration order here therefore does not matter: each live
 * back-pointer flip lands on an already-fully-wired cluster.
 */
static
void ft_graft_glue_apply_deferred(struct cds_ft *ft, struct ft_graft_glue *g)
{
	int i;

	/*
	 * Apply only src-origin edges (dst_origin == false).  graft and
	 * graft_swap record every edge as src-origin (the default), so this
	 * wires all of theirs.  cds_ft_merge_at additionally records
	 * dst-origin edges, which it does NOT re-parent here: a dst child
	 * stays reachable via the old dst spine until the forward publish, so
	 * its back-pointer is switched atomically (with the merge-point
	 * forward slot) by the flip-latch, after this call.
	 */
	for (i = 0; i < g->nr_deferred; i++)
		if (!g->deferred[i].dst_origin)
			ft_set_parent(ft, g->deferred[i].child, g->deferred[i].parent,
				g->deferred[i].slot);
}

/*
 * Record a deferred duplicate-chain splice (cds_ft_merge_at, same full key in
 * both tries): the @src_head chain is to be appended to @dst_head's chain.
 * The @dst_head chain's forward owner and back-pointer are wired separately
 * (Phase-1 set + deferred edge), like any other re-parented external; this
 * records only the concatenation, applied at commit.
 */
static
void ft_graft_glue_record_splice(struct ft_graft_glue *g,
		struct cds_ft_node *dst_head,
		struct cds_ft_node *src_head)
{
	assert(g->nr_splices < g->cap_splices);
	g->splices[g->nr_splices].dst_head = dst_head;
	g->splices[g->nr_splices].src_head = src_head;
	g->splices[g->nr_splices].src_cell = NULL;
	g->nr_splices++;
}

/*
 * Commit: apply the deferred duplicate-chain splices.  Call AFTER the source
 * has been detached + drained (so the appended @src_head chain has no second
 * owner traversing it from the source tree).
 *
 * Per splice: append the whole src chain to dst's tail, prev-before-next (the
 * ft_chain_node idiom, but preserving src_head->next so the rest of the src
 * chain rides along).  @dst_head stays the head, so in-flight dst snapshots
 * keep their ordering.
 */
static
void ft_graft_glue_apply_splices(struct cds_ft *ft __attribute__((unused)),
		struct ft_graft_glue *g)
{
	int i;

	for (i = 0; i < g->nr_splices; i++) {
		struct cds_ft_node *dst_head = g->splices[i].dst_head;
		struct cds_ft_node *src_head = g->splices[i].src_head;
		struct cds_ft_node *tail = dst_head;
		/*
		 * Ordered list on: @src_head was a head in src (prev is its cell);
		 * it becomes a non-head duplicate of @dst_head, so its cell leaves
		 * the trie.  The cell is NOT unreachable yet: on the merge spine
		 * path this runs after the structural flip, and the surviving src
		 * heads' cells -- already reachable in dst -- still carry the old
		 * src-run ord_prev/ord_next, including links to THIS cell, until
		 * the post-publish interleave rewires them.  Capture it in the
		 * splice record; ft_graft_glue_free_collided_cells frees it after
		 * the interleave through the grace-period-deferred cell free.
		 * List off: src_head->prev is the flagged parent, no cell.
		 */
		g->splices[i].src_cell = ft->ordered_list ?
			ft_ord_cell_ptr(src_head->prev) : NULL;

		while (ft_node_next(tail))
			tail = ft_node_next(tail);
		src_head->prev = tail;	/* write-side only, plain store */
		rcu_assign_pointer(tail->next, src_head);
	}
}

/*
 * Free the collided (demoted) src heads' cells.  Call AFTER the ordered-list
 * interleave: only then has every surviving cell's stale src-run link been
 * rewired away from these cells, making them unreachable to NEW readers; the
 * grace-period defer inside ft_ord_cell_free then covers readers already
 * holding a stale link or parked on a demoted head.
 */
static
void ft_graft_glue_free_collided_cells(struct cds_ft *ft,
		struct ft_graft_glue *g)
{
	int i;

	for (i = 0; i < g->nr_splices; i++)
		if (g->splices[i].src_cell)
			ft_ord_cell_free(ft, g->splices[i].src_cell);
}

/*
 * Commit step 2: the single forward publish that makes the whole cluster
 * reachable in dst.  Call after ft_graft_glue_apply_deferred.  The
 * cluster top's parent is wired into publish_parent at set_publish time
 * (build phase, fresh-child store) and the rest of the cluster's
 * internal back-pointers are also already set, so by the time we publish
 * every back-pointer needed for an up-walk from any re-parented live
 * node up through the cluster to publish_parent is in place.
 */
static
void ft_graft_glue_publish(struct cds_ft *ft, struct ft_graft_glue *g)
{
	ft_publish_to_parent(ft, g->publish_parent, g->publish_slot, g->top);
}

/*
 * Commit step 3: reclaim the old (replaced) live nodes, deferred via the
 * normal grace-period free.  Call after the forward publish.
 */
static
void ft_graft_glue_free_old(struct cds_ft *ft, struct ft_graft_glue *g)
{
	int i;

	for (i = 0; i < g->nr_free; i++) {
		if (g->free_list[i].compressed)
			free_compressed_node(ft, g->free_list[i].node);
		else
			free_cds_ft_node(ft, g->free_list[i].node);
	}
}

/*
 * Build a branch for key[start .. end-1] with @leaf at the bottom.
 * When the path is 2+ bytes, a single compressed node is used instead
 * of a chain of single-child internal nodes.  Returns the topmost
 * flagged node, or NULL on allocation failure.
 *
 * @glue: when non-NULL (graft build-invisible mode), @leaf is LIVE
 * payload data: its back-pointer into the bottom branch node is deferred
 * to the post-sync commit, every fresh branch node is tracked in @glue,
 * and on OOM the function returns NULL WITHOUT freeing — the caller's
 * ft_graft_glue_abort reclaims the tracked nodes.  When NULL, the legacy
 * immediate path runs (leaf back-pointer set now, self-free on OOM).
 */
static
struct cds_ft_inode_flag *ft_build_branch(struct cds_ft *ft,
		const uint8_t *key, unsigned int start, unsigned int end,
		struct cds_ft_inode_flag *leaf,
		unsigned long subtree_external_count,
		bool has_external_nodes,
		struct ft_graft_glue *glue)
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
	 * 1-child internal node.  In glue mode ft_try_compress_chain
	 * defers the live leaf's back-pointer (and absorbs a compressed
	 * canonicalization-wrapper @leaf into this compressed).
	 */
	if (end >= compress_start + 1) {
		struct cds_ft_inode_flag *compressed;

		compressed = ft_try_compress_chain(ft, key, end,
			compress_start, leaf, NULL, glue);
		if (compressed == (void *) (long) -ENOMEM)
			return NULL;
		if (compressed) {
			struct cds_ft_metadata *m =
				ft_flag_to_metadata(ft, compressed);

			ft_nr_keys_store(m,
				subtree_external_count, CMM_RELAXED);
			/*
			 * ft_try_compress_chain already tracked the compressed
			 * node by its PLAIN flag in glue mode (the @compressed
			 * return here is the skip form, unsafe to track).
			 */
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
		/*
		 * Bottom internal whose child is the live @leaf (no
		 * compression happened): defer @leaf's back-pointer
		 * (cluster_leaf) and record it for commit.  Higher internals
		 * and the compressed-wrapping case have only fresh children,
		 * whose back-pointers are safe to set during the build.
		 */
		bool leaf_edge = (glue != NULL) && (cur == leaf);
		int ret;

		ret = ft_node_set_nth(ft, &dest, key[i], cur,
			NULL, NULL, i, leaf_edge);
		if (ret) {
			if (glue)
				return NULL;	/* abort frees tracked nodes */
			/*
			 * Legacy: free the created internal chain and, if
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

					next = ft_node_get_nth(ft, cur, NULL, kv, FT_PF_NONE);
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
		if (glue) {
			ft_graft_glue_track(glue, dest);
			if (leaf_edge) {
				struct cds_ft_inode_flag **slot = NULL;

				ft_node_get_nth_skip(dest, &slot, key[i],
					FT_PF_NONE);
				ft_graft_glue_defer_edge(ft, glue, leaf, dest, slot);
			}
		}
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
 * Free a FRESH, never-published single-path branch built by ft_build_branch
 * (legacy glue == NULL mode) after a LATER fallible step failed: walk the
 * single-child chain from @top down to -- but not including -- @leaf (the
 * caller's payload), freeing every fresh node.  Writer-private memory, so
 * immediate frees are safe.  A skip-encoded link resolves through the leaf's
 * back-pointer, which the build wired before returning.
 */
static
void ft_free_branch_unpublished(struct cds_ft *ft,
		struct cds_ft_inode_flag *top, struct cds_ft_inode_flag *leaf)
{
	while (top && top != leaf) {
		struct cds_ft_inode_flag *next;

		if (ft_node_skip_compressed(top)) {
			struct cds_ft_compressed_node *cn =
				ft_skip_to_compressed(ft, top);

			next = cn->child;
			free_compressed_node_unpublished(ft, cn);
		} else if (ft_node_compressed(top)) {
			struct cds_ft_compressed_node *cn =
				ft_compressed_node_ptr(top);

			next = cn->child;
			free_compressed_node_unpublished(ft, cn);
		} else if (ft_node_external(top)) {
			/* Only @leaf may be external on a fresh branch. */
			assert(top == leaf);
			break;
		} else {
			uint8_t v;

			next = ft_node_get_direction(ft, top, -1, &v,
				FT_RIGHT, false);
			free_cds_ft_node_unpublished(ft, ft_node_ptr(top));
		}
		top = next;
	}
}


/*
 * ft_compress_single_child_if_needed: convert a 1-child internal node
 * (no external_nodes) to a compressed/skip-encoded node when the trie
 * group has skip-compressed enabled.  Used at graft sites where a
 * moved root (which was an internal at trie-root position) is being
 * placed at a non-root position: under skip mode, non-root 1-child
 * internals without external_nodes must be a compressed (chain-
 * compress invariant).
 *
 * Two cases on the single child:
 *  - non-compressed:    build a 1-byte cn(byte) → single_child.
 *  - compressed (or
 *    skip-encoded):     chain-merge — build a cn(byte ++
 *                       single_cn.key_bytes) → single_cn.child,
 *                       free the absorbed cn.  Bounded by
 *                       FT_SKIP_LEN_MAX; on overflow leave @child
 *                       unchanged.
 *
 * Returns the converted compressed/skip-encoded flag on success.
 * Returns @child unchanged when conversion isn't applicable (and the
 * unchanged @child is itself canonical at the target position):
 *   - skip-compressed mode is disabled,
 *   - @child is not an internal node,
 *   - @child has more than one child,
 *   - @child has external_nodes attached,
 *   - chain-merge length would overflow FT_SKIP_LEN_MAX.
 * Returns (void *)(long)-ENOMEM on allocation failure: conversion WAS
 * required (a non-root 1-child internal) but could not be done, so the
 * caller must NOT publish @child non-canonically -- it must fail the
 * mutation.  @child is left untouched in that case (nothing freed), so
 * the caller can still roll it back.
 *
 * On successful conversion the old internal is freed.  Write-side
 * only (mutex held); the caller is the sole owner of @child.
 *
 * @glue: when non-NULL, build-invisible mode for the graft transaction.
 * The wrapper @cn is built but its re-parent of the live @cn->child, and
 * the frees of the old internal and any absorbed sub-cn, are DEFERRED to
 * the post-sync commit (recorded in @glue) rather than performed here.
 * The wrapper is tracked in @glue so an OOM elsewhere in the build frees
 * it.  When NULL, the legacy immediate path runs (caller has already
 * drained the source).
 */
static
struct cds_ft_inode_flag *ft_compress_single_child_if_needed(struct cds_ft *ft,
		struct cds_ft_inode_flag *child,
		struct ft_graft_glue *glue)
{
#ifdef FEATURE_FT_SKIP_COMPRESSED
	struct cds_ft_inode *node;
	unsigned int type_index;
	const struct cds_ft_type *type;
	struct cds_ft_metadata *meta;
	uint8_t byte;
	struct cds_ft_inode_flag *single_child;
	struct cds_ft_compressed_node *single_cn = NULL;
	unsigned int single_len = 0;
	struct cds_ft_compressed_node *cn;
	struct cds_ft_metadata *cn_meta;
	struct cds_ft_inode_flag *cflag;
	unsigned int cn_len;

	if (!ft_group_skip_compressed(ft->group))
		return child;
	if (!ft_node_internal(child))
		return child;
	node = ft_node_ptr(child);
	type_index = ft_node_type(child);
	type = &ft_types[type_index];
	meta = cds_ft_item_to_metadata_fast(node, type->order);
	if (meta->nr_child != 1 || meta->external_nodes != NULL)
		return child;

	/*
	 * Find the single set byte.  ft_node_get_minmax returns the
	 * resolved child (skip-encoded → compressed flag); but we
	 * need the raw slot value to preserve any skip encoding when
	 * placing it as cn->child.  Walk via the low-level get_ith.
	 */
	switch (type->type_class) {
	case FT_POPCOUNT:
		ft_popcount_node_get_ith_pos(type, node, 0, &byte, &single_child);
		break;
	case FT_PIGEON:
	{
		unsigned int i;

		single_child = NULL;
		byte = 0;
		for (i = 0; i < FT_ENTRY_PER_NODE; i++) {
			struct cds_ft_inode_flag *v =
				ft_pigeon_node_get_ith_pos(type, node, i);
			if (v) {
				byte = (uint8_t) i;
				single_child = v;
				break;
			}
		}
		break;
	}
	default:
		return child;
	}
	if (!single_child)
		return child;

	/*
	 * Chain-merge when the single child is itself a compressed
	 * (or skip-encoded) cn: absorb its path bytes so the result
	 * is one compressed spanning [byte ++ single_cn->key_bytes]
	 * → single_cn->child.  Preserves the "no two adjacent
	 * compresseds" invariant.
	 */
	if (ft_node_skip_compressed(single_child))
		single_cn = ft_skip_to_compressed(ft, single_child);
	else if (ft_node_compressed(single_child))
		single_cn = ft_compressed_node_ptr(single_child);
	if (single_cn) {
		single_len = single_cn->len;
		if (1U + single_len > FT_SKIP_LEN_MAX) {
			/* Overflow: leave un-canonicalized. */
			return child;
		}
	}
	cn_len = 1U + single_len;

	cn = alloc_compressed_node(ft, cn_len, &cn_meta);
	if (!cn)
		return (struct cds_ft_inode_flag *) (long) -ENOMEM;
	cn->len = (uint8_t) cn_len;
	cn->key_bytes[0] = byte;
	if (single_cn) {
		memcpy(&cn->key_bytes[1], single_cn->key_bytes, single_len);
		cn->child = single_cn->child;
	} else {
		cn->child = single_child;
	}
	cn_meta->nr_child = 1;
	ft_nr_keys_store(cn_meta, ft_nr_keys_get(meta), CMM_RELAXED);
	cflag = ft_compressed_node_flag(cn);
	if (glue) {
		/*
		 * Build-invisible path (graft transaction).  @node and
		 * @single_cn are LIVE source data that must survive an OOM
		 * elsewhere in the build, and @cn->child's back-pointer must
		 * not be flipped until the post-sync commit.  Defer all three;
		 * track @cn so the abort path frees it.  The deferred edge
		 * records the PLAIN @cflag — ft_set_parent recovers @cn
		 * directly and records its skip_slot at commit.
		 */
		ft_graft_glue_track(glue, cflag);
		ft_graft_glue_defer_edge(ft, glue, cn->child, cflag, &cn->child);
		ft_graft_glue_defer_free(glue, node, false);
		if (single_cn)
			ft_graft_glue_defer_free(glue, single_cn, true);
		/*
		 * Return the PLAIN flag, NOT the skip form: @cn->child's
		 * back-pointer is deferred, so a skip pointer would not yet
		 * resolve (ft_skip_to_compressed recovers @cn via that stale
		 * back-pointer).  The caller installs @cn directly (chain-merge
		 * recovers it via ft_compressed_node_ptr) and re-encodes the
		 * holding slot to the skip form before publish — same dance as
		 * the split's suffix.
		 */
		return cflag;
	}
	ft_set_parent(ft, cn->child, cflag, &cn->child);
	/*
	 * Legacy immediate path: the caller has already unlinked @node
	 * from src and waited a grace period (cds_ft_graft / graft_swap
	 * protocol); @node has not been published to dst.  No reader can
	 * be inside it, so the immediate-free path is safe — saves a grace
	 * period of deferred-free pressure.  Same applies to single_cn
	 * (the compressed sub-node we just absorbed): it was reachable
	 * only via @node, which is itself unpublished here.
	 */
	free_cds_ft_node_unpublished(ft, node);
	if (single_cn)
		free_compressed_node_unpublished(ft, single_cn);
	return ft_publish_compressed(ft, cn, cflag);
#else
	(void) ft;
	(void) glue;
	return child;
#endif
}

/*
 * Store graft_payload at the graft point described by @d (the
 * non-diverging "NOSPLIT" attach; the diverging case is handled
 * build-invisibly by ft_split_compressed_graft_build).
 *
 * Handles two cases:
 * - d->depth == key_len: the slot exists; add via ft_node_set_nth.
 * - d->depth < key_len: the path is incomplete; build intermediate
 *   internal nodes via ft_build_branch, displacing any external node
 *   on the path into the branch's metadata.
 *
 * Runs AFTER the source root is unlinked and a grace period has drained
 * its readers, so the payload (live source data) may be restructured;
 * but its back-pointers are still routed through @glue so that an
 * allocation failure here leaves the payload pristine for the caller's
 * rollback (ft_compress_single_child_if_needed is non-destructive in
 * glue mode, and ft_build_branch defers the payload leaf's back-pointer).
 * The deferred edges are applied just before this function's forward
 * publish, and the replaced source root (if canonicalized) is reclaimed
 * after it.  On OOM the caller runs ft_graft_glue_abort + rolls back.
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
		unsigned int *attached_depth,
		struct ft_graft_glue *glue)
{
	struct cds_ft_inode *old_recompacted_node = NULL;

	/*
	 * Skip-mode chain-compress invariant: under SPECULATIVE-mode
	 * tries, non-root 1-child internals without external_nodes
	 * must be a compressed.  graft_payload here may be the source
	 * trie's old root (a 1-child internal is permitted at root,
	 * forbidden at the non-root position we're placing it in).
	 * Canonicalize per branch — deferred past the POPULATED_ERROR
	 * check so that on failure the caller can still reach the
	 * original payload for rollback.
	 */
	if (d->depth == key_len) {
		struct cds_ft_metadata *pmeta;
		struct cds_ft_inode_flag *dest;
		struct cds_ft_inode_flag **slot = NULL;
		struct cds_ft_inode_flag *slot_value, *pf;
		struct ft_flip_batch *b;
		int ret;

		if (d->nf)
			return CDS_FT_STATUS_POPULATED_ERROR;

		graft_payload = ft_compress_single_child_if_needed(ft,
			graft_payload, glue);
		if (graft_payload == (struct cds_ft_inode_flag *) (long) -ENOMEM)
			return CDS_FT_STATUS_MEMORY_ERROR;

		pmeta = cds_ft_item_to_metadata(ft_node_ptr(d->pnf));

		/*
		 * R8 ordering (no publish completed by a later fallible step):
		 * the slot store and its possible recompact are the LAST
		 * fallible steps of the graft, so they run FIRST, parking a
		 * flip proxy that keeps resolving to the empty slot.  An ENOMEM
		 * here leaves both tries untouched -- no live edge was flipped,
		 * so the caller's ft_graft_glue_abort + source-root republish
		 * find everything pristine (previously the payload's parent and
		 * the deferred grandchild flips were applied BEFORE the fallible
		 * set_nth: an OOM left a live grandchild's parent dangling into
		 * the freed canonicalization wrapper and the restored source
		 * root's parent pointing into dst).
		 *
		 * The wiring then runs failure-free behind the parked proxy:
		 *
		 *   (a) graft_payload's back-pointer into @dest (parent + slot
		 *       offset + incoming_byte; ft_node_set_nth skipped it for
		 *       the proxy child, see ft_set_parent);
		 *
		 *   (b) apply_deferred: flip G.parent = graft_payload.  G is
		 *       still invisible via dst (the parked slot resolves to
		 *       NULL), and src's readers were drained by the caller;
		 *
		 *   (c) the commit: one release store flips the slot empty ->
		 *       payload, with the chain G -> graft_payload -> dest
		 *       already wired -- preserving the ordering that keeps a
		 *       skip-validating reader's up-walk from ever reaching the
		 *       orphaned source root (the holder != NULL fix).
		 */
		slot_value = graft_payload;
		if (ft_node_compressed(graft_payload))
			/* Slot-canonical (skip) encoding; pure arithmetic. */
			slot_value = ft_publish_compressed(ft,
				ft_compressed_node_ptr(graft_payload),
				graft_payload);
		b = ft_flip_batch_alloc(ft, 1);
		if (!b)
			return CDS_FT_STATUS_MEMORY_ERROR;
		pf = ft_flip_batch_add(b, NULL, slot_value);
		dest = d->pnf;
		ret = ft_node_set_nth(ft, &dest,
			key[key_len - 1],
			pf, &old_recompacted_node, pmeta,
			d->depth - 1, false);
		if (ret) {
			ft_flip_batch_free_unpublished(b);
			return CDS_FT_STATUS_MEMORY_ERROR;
		}

		/* ===== failure-free commit tail ===== */
		ft_node_get_nth_skip(dest, &slot, key[key_len - 1],
			FT_PF_NONE);
		assert(slot);
		ft_set_parent(ft, graft_payload, dest, slot);
		ft_graft_glue_apply_deferred(ft, glue);
		/*
		 * Recompact case: swing the fresh copy live (its parked slot
		 * still resolves to NULL); in-place case this is a same-value
		 * no-op store.
		 */
		ft_publish_to_parent(ft, pmeta->parent, d->pnfp, dest);
		/* THE publish: empty slot -> fully-wired payload. */
		ft_flip_batch_commit(b);
		rcu_assign_pointer(*slot, slot_value);
		ft_flip_batch_reclaim(b);

		if (old_recompacted_node)
			free_cds_ft_node(ft, old_recompacted_node);
		ft_graft_glue_free_old(ft, glue);
		*attached_nf = graft_payload;
		*attached_depth = key_len;
	} else {
		unsigned int i = d->depth;
		struct cds_ft_inode_flag *branch;
		struct cds_ft_node *displaced = NULL;

		if (d->nf && ft_node_external(d->nf))
			displaced = (struct cds_ft_node *)
				ft_node_ptr(d->nf);

		graft_payload = ft_compress_single_child_if_needed(ft,
			graft_payload, glue);
		if (graft_payload == (struct cds_ft_inode_flag *) (long) -ENOMEM)
			return CDS_FT_STATUS_MEMORY_ERROR;

		branch = ft_build_branch(ft, key, i, key_len, graft_payload,
				graft_external_count, displaced != NULL, glue);
		if (!branch)
			return CDS_FT_STATUS_MEMORY_ERROR;

		if (displaced) {
			struct cds_ft_metadata *bm =
				ft_flag_to_metadata(ft, branch);
			/*
			 * Phase 1 (build-invisible): wire branch's own back-
			 * pointer into d->pnf and the cluster-internal
			 * external_nodes pointer.  The back-channel publish
			 * (displaced->prev = branch) is deferred to Phase 2
			 * below; otherwise an up-walk from displaced (still
			 * reachable through d->pnf's unmodified slot) lands on
			 * branch with parent == NULL.  The payload leaf's
			 * back-pointer is applied via @glue just before the
			 * forward publish below.
			 */
			ft_set_parent(ft, branch, d->pnf, d->nfp);
			ft_metadata_set_external_nodes(branch, bm, displaced);
			ft_nr_keys_store(bm, ft_nr_keys_get(bm) + 1,
				CMM_RELAXED);
		}

		if (displaced) {
			/*
			 * Phase 2: glue deferred FIRST -- the payload's live
			 * back-pointers must be wired before any dst-REACHABLE
			 * live edge (displaced->prev) flips into the fresh
			 * cluster: a reader up-walking from @displaced can
			 * skip-validate through the branch's slots into the
			 * payload, whose grandchild back-pointer must no longer
			 * point into the orphaned source root.  Then the
			 * back-channel, then the forward publish.
			 */
			ft_graft_glue_apply_deferred(ft, glue);
			ft_publish_external_nodes_prev(ft, branch, displaced);
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
			struct cds_ft_inode_flag **slot = NULL;
			struct cds_ft_inode_flag *pf;
			struct ft_flip_batch *b;
			int ret;

			pmeta = cds_ft_item_to_metadata(
					ft_node_ptr(d->pnf));

			/*
			 * Same R8 + fresh-before-live discipline as the
			 * d->depth == key_len arm: park a flip proxy so the
			 * fallible slot store runs FIRST, and the deferred
			 * live flips plus the branch's own back-pointer wiring
			 * complete invisibly before the one-store commit.  The
			 * in-place set_nth otherwise exposed the branch before
			 * apply_deferred wired the payload's back-pointers --
			 * a skip-validating reader could up-walk into the
			 * orphaned source root.
			 */
			b = ft_flip_batch_alloc(ft, 1);
			if (!b)
				return CDS_FT_STATUS_MEMORY_ERROR;
			pf = ft_flip_batch_add(b, NULL, branch);
			ret = ft_node_set_nth(ft, &dest,
				key[i - 1],
				pf, &old_recompacted_node, pmeta,
				d->depth - 1, false);
			if (ret) {
				ft_flip_batch_free_unpublished(b);
				/* branch + path are tracked in @glue; the
				 * caller's abort reclaims them. */
				return CDS_FT_STATUS_MEMORY_ERROR;
			}
			/* ===== failure-free commit tail ===== */
			ft_node_get_nth_skip(dest, &slot, key[i - 1],
				FT_PF_NONE);
			assert(slot);
			ft_set_parent(ft, branch, dest, slot);
			ft_graft_glue_apply_deferred(ft, glue);
			ft_publish_to_parent(ft, pmeta->parent,
				d->pnfp, dest);
			ft_flip_batch_commit(b);
			rcu_assign_pointer(*slot, branch);
			ft_flip_batch_reclaim(b);

			if (old_recompacted_node)
				free_cds_ft_node(ft, old_recompacted_node);
		}
		ft_graft_glue_free_old(ft, glue);
		*attached_nf = branch;
		*attached_depth = d->depth;
	}
	return CDS_FT_STATUS_OK;
}

/*
 * Outcome of ft_graft_build's build-invisible prep descent.
 */
enum ft_graft_prep {
	FT_GRAFT_PREP_GLUE,	/* diverge: full attach cluster built into @glue */
	FT_GRAFT_PREP_NOSPLIT,	/* graft point located in @d; legacy attach */
	FT_GRAFT_PREP_POPULATED,/* graft point occupied; tries pristine */
	FT_GRAFT_PREP_OOM,	/* allocation failed; caller runs glue_abort */
};

/*
 * Build-invisible prep for cds_ft_graft.  Descends dst to the graft point
 * for @key.  When the key diverges inside a compressed node, builds the
 * COMPLETE attach cluster (split rearrangement + the @payload subtrie)
 * into @glue without publishing or freeing anything — dst and the source
 * stay pristine, so an OOM frees the glue with nothing to roll back
 * (FT_GRAFT_PREP_GLUE / _OOM).  Otherwise it just locates the graft point
 * in @d (FT_GRAFT_PREP_NOSPLIT — graft_keylen completes via the legacy
 * post-sync ft_store_at_graft_point) or reports an occupied point
 * (FT_GRAFT_PREP_POPULATED: the key ends inside an existing compressed
 * path).
 *
 * Read-only on dst for the NOSPLIT / POPULATED outcomes.
 */
static
enum ft_graft_prep ft_graft_build(struct cds_ft *ft,
		const uint8_t *key, size_t key_len,
		struct cds_ft_inode_flag *payload, unsigned long src_count,
		struct ft_descent *d, struct ft_graft_glue *glue)
{
	const uint8_t *ik = key;

	ft_descent_init(d, ft);
	for (; d->depth < key_len; ) {
		if (ft_node_external(d->nf))
			break;
		d->nf = ft_resolve_skip_compressed(ft, d->nf);
		if (ft_node_compressed(d->nf)) {
			struct cds_ft_compressed_node *cn =
				ft_compressed_node_ptr(d->nf);
			int remaining = (int) (key_len - d->depth);
			int cmp = cn->len < remaining ? cn->len : remaining;
			int j = ft_match_compressed_key(ik, cn, cmp);

			if (j == cmp && cn->len <= remaining) {
				ft_descent_traverse_compressed(d, cn, &ik);
				continue;
			}
			if (j < cmp) {
				if (ft_split_compressed_graft_build(ft, d, key,
						key_len, j, payload, src_count,
						glue))
					return FT_GRAFT_PREP_OOM;
				return FT_GRAFT_PREP_GLUE;
			}
			/*
			 * Key shorter than the compressed path: the graft
			 * point lies inside an existing compressed (occupied).
			 */
			return FT_GRAFT_PREP_POPULATED;
		}
		ft_descent_step(ft, d, *(ik++));
	}
	return FT_GRAFT_PREP_NOSPLIT;
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
 * structural body are performed here, so this helper is the single
 * source of truth for what graft actually does.
 *
 * Transaction shape (non-root): build the dst-side attach invisibly
 * (ft_graft_build), then the failure-free commit — unlink the source
 * root, synchronize, apply the deferred live back-pointers, publish the
 * cluster, reclaim the old nodes.  A diverge split is fully build-
 * invisible (no rollback); the non-split attach still uses the legacy
 * post-sync store with a clean rollback.
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
		/*
		 * Ordered list: dst was empty (checked above), so src's WHOLE
		 * ordered list becomes dst's.  Cells' internal links unchanged;
		 * only the head/tail endpoints transfer (mirrors the root swap,
		 * which likewise needs no synchronize_rcu).
		 */
		if (dst_ft->group->ordered_list_set) {
			rcu_assign_pointer(dst_ft->ord_cell_head,
				src_ft->ord_cell_head);
			rcu_assign_pointer(dst_ft->ord_cell_tail,
				src_ft->ord_cell_tail);
			src_ft->ord_cell_head = NULL;
			src_ft->ord_cell_tail = NULL;
		}
		goto done;
	}

	{
		struct ft_descent d;
		struct cds_ft_inode *fresh_node;
		struct cds_ft_metadata *fresh_meta;
		struct ft_graft_glue glue;
		enum ft_graft_prep prep;
		unsigned long src_count = ft_nr_keys_get(src_rmeta);
		struct cds_ft_inode_flag *old_src_root;
		struct cds_ft_inode_flag *attached_nf = NULL;
		struct ft_ord_cell *graft_run_first = NULL, *graft_run_last = NULL;
		struct ft_ord_cell *graft_pred = NULL, *graft_succ = NULL;

		/*
		 * Preallocate a fresh empty root for the source trie
		 * before the point of no return, so we can fail cleanly
		 * on memory shortage instead of calling abort().
		 */
		fresh_node = alloc_cds_ft_node(src_ft, &ft_types[0], &fresh_meta);
		if (!fresh_node)
			return CDS_FT_STATUS_MEMORY_ERROR;

		/*
		 * PREP (dst + source pristine): build the dst-side attach.
		 * A diverge split builds its whole cluster invisibly into
		 * @glue; otherwise just locate the graft point in @d.
		 */
		ft_graft_glue_init(&glue);
		prep = ft_graft_build(dst_ft, key, key_len, src_ft->root,
				src_count, &d, &glue);
		if (prep == FT_GRAFT_PREP_OOM) {
			ft_graft_glue_abort(dst_ft, &glue);
			free_cds_ft_node(src_ft, fresh_node);
			return CDS_FT_STATUS_MEMORY_ERROR;
		}
		if (prep == FT_GRAFT_PREP_POPULATED) {
			free_cds_ft_node(src_ft, fresh_node);
			return CDS_FT_STATUS_POPULATED_ERROR;
		}

		/*
		 * Ordered list: locate the dst splice neighbours NOW, while dst is
		 * still payload-free (the attach is built invisibly / not yet
		 * published) -- a relational descent after the payload is live
		 * would return a payload head as the boundary.
		 */
		if (dst_ft->group->ordered_list_set)
			ft_ord_cell_find_splice_pos(dst_ft, _key, key_len,
				&graft_pred, &graft_succ);

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

		/*
		 * Ordered list: capture src's whole list (the run to graft) and
		 * unlink it from src here, paired with the structural src-root
		 * unlink, so the synchronize_rcu below drains src ord-readers too.
		 * The run is spliced into dst after the structural publish (same
		 * commit point).  Restored on the OOM rollback below.
		 */
		if (dst_ft->group->ordered_list_set) {
			graft_run_first = src_ft->ord_cell_head;
			graft_run_last = src_ft->ord_cell_tail;
			src_ft->ord_cell_head = NULL;
			src_ft->ord_cell_tail = NULL;
		}

		if (!src_ft->exclusive)
			src_ft->group->flavor->update_synchronize_rcu();

		if (prep == FT_GRAFT_PREP_GLUE) {
			/*
			 * Failure-free commit of the build-invisible diverge
			 * cluster: wire the deferred live back-pointers (the
			 * displaced old child, the payload, and the cluster
			 * top), splice the cluster into dst with a single
			 * forward publish, then reclaim the old compressed
			 * node and the source's old root.  Nothing can fail.
			 */
			ft_graft_glue_apply_deferred(dst_ft, &glue);
			ft_graft_glue_publish(dst_ft, &glue);
			attached_nf = glue.attached_nf;
			ft_graft_glue_free_old(dst_ft, &glue);
		} else {
			/*
			 * Non-split attach: the payload subtrie is built into
			 * @glue with its back-pointers deferred, recompacted
			 * into the live graft-point node, and published — all
			 * inside ft_store_at_graft_point.  On OOM nothing was
			 * published and the payload is pristine (deferred edges
			 * never applied): free the glue and roll the source root
			 * back cleanly (a second grace period drains readers
			 * that may have observed the fresh empty root).
			 */
			unsigned int attached_depth = 0;

			status = ft_store_at_graft_point(dst_ft, key, key_len,
							  &d, old_src_root,
							  src_count,
							  &attached_nf,
							  &attached_depth,
							  &glue);
			if (status != CDS_FT_STATUS_OK) {
				ft_graft_glue_abort(dst_ft, &glue);
				rcu_assign_pointer(src_ft->root, old_src_root);
				/* Roll the captured run back into src's list. */
				if (graft_run_first) {
					rcu_assign_pointer(src_ft->ord_cell_head,
						graft_run_first);
					rcu_assign_pointer(src_ft->ord_cell_tail,
						graft_run_last);
				}
				FT_TP(root_publish, (const void *) src_ft,
					(const void *) src_ft->root);
				if (!src_ft->exclusive)
					src_ft->group->flavor->update_synchronize_rcu();
				free_cds_ft_node(src_ft, fresh_node);
				return status;
			}
		}

		/*
		 * Propagate src_count up the ancestor chain, starting
		 * from @attached_nf's parent (skipping @attached_nf
		 * itself, whose nr_keys is already the payload count).
		 *
		 * Note: *d.pnfp can't be used as the start because under
		 * SKIP_COMPRESSED, ft_publish_to_parent may have updated
		 * the grandparent slot (via cn's skip_slot mechanism) to
		 * point directly at @attached_nf — starting propagation
		 * there would double-count @attached_nf's subtree.
		 */
		{
			struct cds_ft_metadata *am =
				ft_flag_to_metadata(dst_ft, attached_nf);
			if (am->parent)
				ft_propagate_external_count_parent(dst_ft,
					am->parent, (long) src_count);
		}

		/*
		 * Ordered list: src is now structurally empty + drained; the
		 * payload is published under @key in dst.  Splice the captured run
		 * (src's whole former list) into dst's ordered cell list at the
		 * @key position (an empty range in dst -> no interleave).  Same
		 * commit point as the structural publish above.
		 */
		if (graft_run_first)
			ft_ord_cell_run_splice(dst_ft, graft_run_first,
				graft_run_last, graft_pred, graft_succ);

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

/*
 * Build (invisibly) a swap_ft root node holding the extracted subtree
 *   @first_byte ++ @rest[0 .. @rest_len-1]  ->  @child
 * i.e. an internal root whose @first_byte slot leads (via a fresh compressed
 * suffix when @rest_len >= 1) to the LIVE @child.  Used (via
 * ft_make_root_internal_glue) by the extract side of cds_ft_graft_swap and by
 * ft_detach_keylen's pre-publish root materialization.
 *
 * @child is LIVE data relocated into swap_ft: its back-pointer is recorded as
 * a deferred edge in @glue rather than flipped now, and every fresh node is
 * tracked so an OOM elsewhere in the swap build frees the cluster (both tries
 * pristine, nothing published).  Nothing is freed here.
 *
 * @child is always a plain (internal / external) node — a compressed node's
 * child is plain by the chain-merge invariant, and the suffix path is held by
 * the fresh compressed @rest — so no chain-merge is needed.
 *
 * Returns the new internal-tagged root flag, or (void *)(long)-ENOMEM.
 */
static
struct cds_ft_inode_flag *ft_build_extracted_root_glue(struct cds_ft *ft,
		struct ft_graft_glue *glue,
		uint8_t first_byte, const uint8_t *rest, unsigned int rest_len,
		struct cds_ft_inode_flag *child, unsigned long subtree_count)
{
	struct cds_ft_inode_flag *slot_value;
	struct cds_ft_inode_flag *skip_value = NULL;
	struct cds_ft_compressed_node *new_cn = NULL;
	struct cds_ft_inode *root_node;
	struct cds_ft_metadata *root_meta;
	struct cds_ft_inode_flag *dest;
	struct cds_ft_inode_flag **slot = NULL;
	int ret;

	if (rest_len == 0) {
		slot_value = child;	/* LIVE; back-pointer deferred below. */
	} else {
		struct cds_ft_metadata *new_cn_meta;

		new_cn = alloc_compressed_node(ft, rest_len, &new_cn_meta);
		if (!new_cn)
			return (struct cds_ft_inode_flag *) (long) -ENOMEM;
		new_cn->len = (uint8_t) rest_len;
		new_cn->child = child;
		memcpy(new_cn->key_bytes, rest, rest_len);
		new_cn_meta->nr_child = 1;
		ft_nr_keys_store(new_cn_meta, subtree_count, CMM_RELAXED);
		slot_value = ft_compressed_node_flag(new_cn);	/* PLAIN */
		ft_graft_glue_track(glue, slot_value);
		/* @child (live) -> new_cn, deferred to the post-sync commit. */
		ft_graft_glue_defer_edge(ft, glue, child, slot_value, &new_cn->child);
		/* Skip form for the root slot (resolves once the edge applies). */
		skip_value = ft_publish_compressed(ft, new_cn, slot_value);
	}

	root_node = alloc_cds_ft_node(ft, &ft_types[0], &root_meta);
	if (!root_node)
		/* new_cn (if any) is tracked in @glue; the caller's abort frees it. */
		return (struct cds_ft_inode_flag *) (long) -ENOMEM;
	dest = ft_node_flag(root_node, 0);
	/*
	 * rest_len == 0: @child is live, so defer its back-pointer (cluster_leaf).
	 * rest_len >= 1: the slot holds the fresh new_cn, whose own back-pointer
	 * into @dest is a fresh-to-fresh edge that is safe to set during the build.
	 */
	ret = ft_node_set_nth(ft, &dest, first_byte, slot_value,
			NULL, root_meta, 0, rest_len == 0 /* cluster_leaf */);
	if (ret)
		return (struct cds_ft_inode_flag *) (long) -ENOMEM;
	ft_graft_glue_track(glue, dest);
	ft_nr_keys_store(ft_flag_to_metadata(ft, dest), subtree_count, CMM_RELAXED);
	ft_node_get_nth_skip(dest, &slot, first_byte, FT_PF_NONE);
	if (rest_len == 0) {
		ft_graft_glue_defer_edge(ft, glue, child, dest, slot);
	} else {
		/* Re-encode the root slot to the skip form (new_cn is compressed). */
		if (skip_value && skip_value != slot_value && slot)
			rcu_assign_pointer(*slot, skip_value);
		ft_set_parent(ft, slot_value, dest, slot);
	}
	return dest;
}

/*
 * Build-invisible internal-root materialization, preserving the trie-wide
 * invariant that the root pointer always tags an internal node (never
 * compressed, never skip-compressed).  Used by cds_ft_graft_swap's extract
 * side and by ft_detach_keylen.  Materializes an internal-node root from the
 * LIVE displaced @old_child without publishing or mutating live data: fresh nodes are tracked in @glue, the moved grandchild's
 * back-pointer is deferred, and the peeled-away compressed node is recorded for
 * deferred free.  Nothing is freed here.
 *
 *   - @old_child internal:    returned unchanged (already a valid internal
 *                             root; the caller clears its parent at commit).
 *                             No glue node, no deferred edge.
 *   - @old_child compressed:  peel the first path byte into a fresh internal
 *                             root, the rest (if any) into a fresh compressed
 *                             node; defer the live grandchild's back-pointer;
 *                             record the old compressed node for deferred free.
 *
 * External @old_child must be filtered by the caller (externals attach as
 * external_nodes, not via this helper).
 *
 * Returns the new internal-tagged root flag, or (void *)(long)-ENOMEM.
 */
static
struct cds_ft_inode_flag *ft_make_root_internal_glue(struct cds_ft *ft,
		struct ft_graft_glue *glue, struct cds_ft_inode_flag *old_child)
{
	struct cds_ft_compressed_node *cn;
	struct cds_ft_metadata *cn_meta;
	struct cds_ft_inode_flag *root;

	old_child = ft_resolve_skip_compressed(ft, old_child);
	if (caa_likely(!ft_node_compressed(old_child)))
		return old_child;	/* already internal */
	cn = ft_compressed_node_ptr(old_child);
	cn_meta = cds_ft_item_to_metadata((struct cds_ft_inode *) cn);
	root = ft_build_extracted_root_glue(ft, glue,
			cn->key_bytes[0], &cn->key_bytes[1], cn->len - 1,
			cn->child, ft_nr_keys_get(cn_meta));
	if (root == (struct cds_ft_inode_flag *) (long) -ENOMEM)
		return root;
	/* Reclaim the peeled-away compressed node after the commit. */
	ft_graft_glue_defer_free(glue, cn, true);
	return root;
}

/*
 * Outcome of ft_graft_swap_descend's read-only descent toward the swap key.
 */
enum ft_graft_swap_case {
	FT_GRAFT_SWAP_EXACT,		/* reached key_len at a live subtree (d->nf) */
	FT_GRAFT_SWAP_KEY_SHORTER,	/* key ends strictly inside compressed d->nf */
	FT_GRAFT_SWAP_DELEGATE,		/* diverge / dead-end: no content at key */
};

/*
 * Read-only descent to the graft point for cds_ft_graft_swap.  Unlike
 * ft_descend_to_graft_point it publishes nothing: a key-shorter or diverging
 * key is reported, never split in place, so the whole swap can be assembled as
 * a build-invisible transaction.
 *
 *   FT_GRAFT_SWAP_EXACT:       d->depth == key_len and d->nf is the existing
 *                              subtree at @key (the displaced old-child).
 *   FT_GRAFT_SWAP_KEY_SHORTER: @key ends inside the compressed node d->nf
 *                              (d->depth is the node's start depth, d->pnf /
 *                              d->nfp hold it).
 *   FT_GRAFT_SWAP_DELEGATE:    the path diverges, dead-ends, or the slot at
 *                              @key is empty — there is nothing to extract, so
 *                              the swap reduces to an insert (the caller routes
 *                              to the now-atomic cds_ft_graft).
 */
static
enum ft_graft_swap_case ft_graft_swap_descend(struct cds_ft *ft,
		const uint8_t *key, size_t key_len, struct ft_descent *d)
{
	const uint8_t *ik = key;

	ft_descent_init(d, ft);
	for (; d->depth < key_len; ) {
		if (ft_node_external(d->nf))
			return FT_GRAFT_SWAP_DELEGATE;
		d->nf = ft_resolve_skip_compressed(ft, d->nf);
		if (ft_node_compressed(d->nf)) {
			struct cds_ft_compressed_node *cn =
				ft_compressed_node_ptr(d->nf);
			int remaining = (int) (key_len - d->depth);
			int cmp = cn->len < remaining ? cn->len : remaining;
			int j = ft_match_compressed_key(ik, cn, cmp);

			if (j < cmp)
				return FT_GRAFT_SWAP_DELEGATE;	/* diverge */
			if (cn->len <= remaining) {
				ft_descent_traverse_compressed(d, cn, &ik);
				continue;
			}
			/* j == cmp == remaining < cn->len: key ends inside cn. */
			return FT_GRAFT_SWAP_KEY_SHORTER;
		}
		if (!ft_descent_step(ft, d, *(ik++)))
			return FT_GRAFT_SWAP_DELEGATE;	/* dead-end */
	}
	if (!d->nf)
		return FT_GRAFT_SWAP_DELEGATE;	/* empty slot at key */
	return FT_GRAFT_SWAP_EXACT;
}

/*
 * Read-only locate of a merge point at the end of @key.  Reuses
 * ft_graft_swap_descend (the same three outcomes, publishes nothing) and
 * additionally reports the cursor's compressed offset and the subtree's key
 * count, both needed by cds_ft_merge_at:
 *
 *   FT_GRAFT_SWAP_EXACT:       @d->nf is the live subtree at @key; @off_ret 0;
 *                              @count_ret its nr_keys (1 for an external / dup
 *                              chain — one unique key).
 *   FT_GRAFT_SWAP_KEY_SHORTER: @key ends inside compressed @d->nf; @off_ret =
 *                              key_len - d->depth (bytes consumed into the
 *                              node); @count_ret the node's subtree nr_keys.
 *   FT_GRAFT_SWAP_DELEGATE:    no content under @key (src side -> the merge is
 *                              a no-op; dst side -> the atomic fast-path graft).
 *
 * @off_ret / @count_ret are 0 for DELEGATE.
 */
static
enum ft_graft_swap_case ft_merge_descend(struct cds_ft *ft,
		const uint8_t *key, size_t key_len, struct ft_descent *d,
		unsigned int *off_ret, unsigned long *count_ret)
{
	enum ft_graft_swap_case kase = ft_graft_swap_descend(ft, key, key_len, d);

	switch (kase) {
	case FT_GRAFT_SWAP_KEY_SHORTER:
	{
		struct cds_ft_compressed_node *cn = ft_compressed_node_ptr(d->nf);

		*off_ret = (unsigned int) (key_len - d->depth);
		*count_ret = ft_nr_keys_get(
			cds_ft_item_to_metadata((struct cds_ft_inode *) cn));
		break;
	}
	case FT_GRAFT_SWAP_EXACT:
		*off_ret = 0;
		if (ft_node_external(d->nf))
			*count_ret = 1;	/* one key (possibly a dup chain) */
		else
			*count_ret = ft_nr_keys_get(ft_flag_to_metadata(ft, d->nf));
		break;
	default:	/* FT_GRAFT_SWAP_DELEGATE */
		*off_ret = 0;
		*count_ret = 0;
		break;
	}
	return kase;
}

/*
 * cds_ft_merge_at build-invisible spine-copy.  ft_merge_build recursively
 * COPIES the overlapping spine of two subtrees S (from src) and D (from dst),
 * both rooted at the same suffix position, and REFERENCES every disjoint
 * subtree by pointer (recorded as a deferred re-parent edge applied at commit).
 * All allocation happens here, build-invisibly; OOM frees the fresh copies and
 * leaves both tries pristine -- no rollback.
 *
 * Returns the PLAIN flag of the freshly-built merged node (or, when the merged
 * position is leaf-only, the surviving external head), or FT_MERGE_OOM on
 * allocation failure (the caller aborts both glues).  On success *nr_keys_ret
 * holds the merged subtree's unique-key count.
 */
#define FT_MERGE_OOM		((struct cds_ft_inode_flag *) (long) -ENOMEM)

struct ft_merge_ctx {
	struct cds_ft *dst_ft;		/* all fresh merged nodes live here */
	struct ft_graft_glue *gd;	/* dst cluster: built/deferred/dst frees/splices */
	struct ft_graft_glue *gs;	/* src side: src-overlap frees (+ prune later) */
};

/* Upper-bound counters for the read-only pre-pass that sizes the glues. */
struct ft_merge_counts {
	int nb;		/* fresh built nodes (-> gd) */
	int nd;		/* deferred re-parent edges (-> gd) */
	int nf_dst;	/* dst-overlap frees (-> gd) */
	int nf_src;	/* src-overlap frees (-> gs) */
	int ns;		/* duplicate-chain splices (-> gd) */
};

/* nr_keys of a (possibly skip-encoded) child subtree referenced by pointer. */
static
unsigned long ft_merge_child_count(struct cds_ft *ft, struct cds_ft_inode_flag *c)
{
	if (ft_node_external(ft_resolve_skip_compressed(ft, c)))
		return 1;
	return ft_nr_keys_get(ft_flag_to_metadata(ft, c));
}

static
struct cds_ft_inode_flag *ft_merge_build(struct ft_merge_ctx *c,
		struct cds_ft_inode_flag *S, unsigned int off_s,
		struct cds_ft_inode_flag *D, unsigned int off_d,
		unsigned int depth, unsigned long *nr_keys_ret);

/*
 * Advance a compressed cursor one byte past @next_off - 1: while still inside
 * the run report the run's flag at @next_off, else fall through to the run's
 * (internal/external, never compressed) child at offset 0.
 */
static inline
void ft_merge_advance(struct cds_ft_compressed_node *cn, unsigned int next_off,
		struct cds_ft_inode_flag **flag_ret, unsigned int *off_ret)
{
	if (next_off < cn->len) {
		*flag_ret = ft_compressed_node_flag(cn);
		*off_ret = next_off;
	} else {
		*flag_ret = cn->child;
		*off_ret = 0;
	}
}

/*
 * Materialize, build-invisibly, the subtree a single-side compressed run
 * contributes to a fresh branch slot: the bytes cn->key_bytes[off+1 .. len)
 * down to cn's (live) child.  The byte cn->key_bytes[off] is consumed by the
 * parent branch's slot, so this builds the part below it:
 *
 *   suffix_len == 0:                  the bare live child (no fresh node; the
 *                                     parent frame's Pass 2 wires it).
 *   suffix_len == 1 (non-skip):       a 1-child internal node.
 *   suffix_len >= 2, or == 1 (skip):  a fresh compressed node, returned in its
 *                                     skip-published form.
 *
 * Mirrors the old-direction suffix of ft_split_compressed_graft_build.  cn's
 * (live) child back-pointer is deferred into @c->gd with @dst_origin (false
 * for a src run, true for a dst run); a fresh wrapper is tracked there too.
 * @child_depth is the depth of the materialized node itself (parent + 1).
 * Returns the flag to store in the parent slot, or FT_MERGE_OOM.
 */
static
struct cds_ft_inode_flag *ft_merge_materialize_suffix(struct ft_merge_ctx *c,
		struct cds_ft_compressed_node *cn, unsigned int off,
		unsigned int child_depth, bool dst_origin,
		unsigned long *ck_ret)
{
	struct cds_ft *ft = c->dst_ft;
	struct ft_graft_glue *g = c->gd;	/* all fresh merged nodes -> gd */
	unsigned int suffix_len = cn->len - off - 1;
	unsigned long child_keys = ft_merge_child_count(ft, cn->child);
	struct cds_ft_inode_flag **slot;
	int ret;

	*ck_ret = child_keys;
	if (suffix_len >= 2
#ifdef FEATURE_FT_SKIP_COMPRESSED
			|| (suffix_len == 1 && ft_group_skip_compressed(ft->group))
#endif
	   ) {
		struct cds_ft_compressed_node *sfx;
		struct cds_ft_metadata *sfx_meta;
		struct cds_ft_inode_flag *plain;

		sfx = alloc_compressed_node(ft, (uint8_t) suffix_len, &sfx_meta);
		if (!sfx)
			return FT_MERGE_OOM;
		sfx->child = cn->child;
		sfx->len = (uint8_t) suffix_len;
		memcpy(sfx->key_bytes, &cn->key_bytes[off + 1], suffix_len);
		sfx_meta->nr_child = 1;
		ft_nr_keys_store(sfx_meta, child_keys, CMM_RELAXED);
		plain = ft_compressed_node_flag(sfx);
		ft_graft_glue_track(g, plain);
		ft_graft_glue_defer_edge_origin(ft, g, cn->child, plain, &sfx->child,
				dst_origin);
		/*
		 * Return the PLAIN flag: a fresh compressed node's skip form
		 * cannot be resolved during the build (its child's back-pointer
		 * is deferred), so the parent stores the plain form -- which
		 * ft_graft_glue_is_fresh matches by direct identity -- and the
		 * parent's Pass 2 re-encodes the slot to the skip form.
		 */
		return plain;
	} else if (suffix_len == 1) {
		/* 1-child internal suffix (non-skip-compressed). */
		struct cds_ft_inode_flag *dest = NULL;

		ret = ft_node_set_nth(ft, &dest, cn->key_bytes[off + 1],
				cn->child, NULL, NULL, child_depth, true);
		if (ret)
			return FT_MERGE_OOM;
		ft_nr_keys_store(cds_ft_item_to_metadata(ft_node_ptr(dest)),
				child_keys, CMM_RELAXED);
		ft_graft_glue_track(g, dest);
		ft_node_get_nth_skip(dest, &slot, cn->key_bytes[off + 1],
				FT_PF_NONE);
		ft_graft_glue_defer_edge_origin(ft, g, cn->child, dest, slot,
				dst_origin);
		return dest;
	}
	/* suffix_len == 0: the bare child; parent Pass 2 wires its edge. */
	return cn->child;
}

/*
 * Both overlap sides are compressed runs sharing a @p (>= 1) byte prefix from
 * their cursors.  Emit ONE fresh compressed run for the shared bytes and
 * recurse on what follows each cursor (the next byte, or the run's child once
 * a run is exhausted).  The shared run's child is internal/external/another
 * fresh branch -- never another compressed -- because a canonical compressed
 * node's child is never compressed, so no two adjacent compresseds result.
 * Returns the run's PLAIN flag (parent Pass 2 re-encodes the slot to skip), or
 * FT_MERGE_OOM.
 */
static
struct cds_ft_inode_flag *ft_merge_build_run(struct ft_merge_ctx *c,
		struct cds_ft_compressed_node *cn_s, unsigned int off_s,
		struct cds_ft_compressed_node *cn_d, unsigned int off_d,
		unsigned int p, unsigned int depth, unsigned long *nr_keys_ret)
{
	struct cds_ft *ft = c->dst_ft;
	struct cds_ft_compressed_node *run;
	struct cds_ft_metadata *run_meta;
	struct cds_ft_inode_flag *adv_s, *adv_d, *child, *plain;
	unsigned int aoff_s, aoff_d;
	unsigned long ck = 0;

	ft_merge_advance(cn_s, off_s + p, &adv_s, &aoff_s);
	ft_merge_advance(cn_d, off_d + p, &adv_d, &aoff_d);
	child = ft_merge_build(c, adv_s, aoff_s, adv_d, aoff_d, depth + p, &ck);
	if (child == FT_MERGE_OOM)
		return child;

#ifndef FEATURE_FT_SKIP_COMPRESSED
	if (p == 1) {
		/*
		 * Non-skip-compressed: a 1-byte run is a 1-child internal node,
		 * not a len-1 compressed node (matching insert/split canonical
		 * form).  Same node+edge budget as the compressed run.
		 */
		struct cds_ft_inode_flag *dest = NULL, **slot;
		int ret;

		ret = ft_node_set_nth(ft, &dest, cn_s->key_bytes[off_s], child,
				NULL, NULL, depth, true);
		if (ret)
			return FT_MERGE_OOM;
		ft_nr_keys_store(cds_ft_item_to_metadata(ft_node_ptr(dest)), ck,
				CMM_RELAXED);
		ft_graft_glue_track(c->gd, dest);
		ft_node_get_nth_skip(dest, &slot, cn_s->key_bytes[off_s],
				FT_PF_NONE);
		ft_graft_glue_defer_edge_origin(ft, c->gd, child, dest, slot,
				/*dst_origin=*/ true);
		*nr_keys_ret = ck;
		return dest;
	}
#endif
	run = alloc_compressed_node(ft, (uint8_t) p, &run_meta);
	if (!run) {
		/*
		 * The recursion's cluster is already tracked in the glue; the
		 * caller's abort frees it.  Only this frame's run failed.
		 */
		return FT_MERGE_OOM;
	}
	run->child = child;
	run->len = (uint8_t) p;
	memcpy(run->key_bytes, &cn_s->key_bytes[off_s], p);
	run_meta->nr_child = 1;
	ft_nr_keys_store(run_meta, ck, CMM_RELAXED);
	plain = ft_compressed_node_flag(run);
	ft_graft_glue_track(c->gd, plain);
	/*
	 * Wire run->child's back-pointer.  A fresh recursion result is stored
	 * immediately (is_fresh); a live result can only be the dst splice head
	 * the recursion returns, so dst_origin is safe either way.
	 */
	ft_graft_glue_defer_edge_origin(ft, c->gd, child, plain, &run->child,
			/*dst_origin=*/ true);
	/*
	 * Return the PLAIN flag (like ft_merge_materialize_suffix): the parent
	 * frame's Pass 2 re-encodes the holding slot to the skip form once the
	 * run is wired, and is_fresh can match it by identity in the meantime.
	 */
	*nr_keys_ret = ck;
	return plain;
}

static
struct cds_ft_inode_flag *ft_merge_build(struct ft_merge_ctx *c,
		struct cds_ft_inode_flag *S, unsigned int off_s,
		struct cds_ft_inode_flag *D, unsigned int off_d,
		unsigned int depth, unsigned long *nr_keys_ret)
{
	struct cds_ft *ft = c->dst_ft;
	struct cds_ft_node *S_leaf, *D_leaf, *M_ext;
	struct cds_ft_compressed_node *cn_s, *cn_d;
	bool S_ext, D_ext, S_comp, D_comp;
	struct cds_ft_inode_flag *M = NULL;
	struct cds_ft_metadata *Mmeta = NULL;
	unsigned long total_keys = 0;
	bool tracked = false;
	unsigned int b, s_fb = 0, d_fb = 0;

	S = ft_resolve_skip_compressed(ft, S);
	D = ft_resolve_skip_compressed(ft, D);

	S_comp = ft_node_compressed(S);
	D_comp = ft_node_compressed(D);
	cn_s = S_comp ? ft_compressed_node_ptr(S) : NULL;
	cn_d = D_comp ? ft_compressed_node_ptr(D) : NULL;

	/*
	 * Record each compressed overlap node's reclaim exactly once, on first
	 * entry (off == 0): a run may be re-entered at a deeper cursor by the
	 * shared-run recursion, but the node is freed whole.  src -> gs,
	 * dst -> gd.  Internal overlap nodes are recorded at the tail instead.
	 */
	if (S_comp && off_s == 0)
		ft_graft_glue_defer_free(c->gs, cn_s, true);
	if (D_comp && off_d == 0)
		ft_graft_glue_defer_free(c->gd, cn_d, true);

	/*
	 * Both compressed and sharing a prefix from their cursors -> collapse
	 * the shared bytes into one fresh run and recurse past it.  A zero-byte
	 * common prefix means the two runs diverge at the cursor: fall through
	 * to build a 2-way branch.
	 */
	if (S_comp && D_comp) {
		unsigned int rem_s = cn_s->len - off_s;
		unsigned int rem_d = cn_d->len - off_d;
		unsigned int maxp = rem_s < rem_d ? rem_s : rem_d;
		unsigned int p = 0;

		while (p < maxp &&
		       cn_s->key_bytes[off_s + p] == cn_d->key_bytes[off_d + p])
			p++;
		if (p >= 1)
			return ft_merge_build_run(c, cn_s, off_s, cn_d, off_d,
					p, depth, nr_keys_ret);
	}

	S_ext = ft_node_external(S);
	D_ext = ft_node_external(D);
	S_leaf = S_comp ? NULL : (S_ext ? (struct cds_ft_node *) ft_node_ptr(S)
		: cds_ft_item_to_metadata(ft_node_ptr(S))->external_nodes);
	D_leaf = D_comp ? NULL : (D_ext ? (struct cds_ft_node *) ft_node_ptr(D)
		: cds_ft_item_to_metadata(ft_node_ptr(D))->external_nodes);

	/*
	 * Both leaf-only -> the SAME full key terminates on both sides.
	 * Splice src after dst and return the (leaf-only) dst head; the
	 * parent frame wires the slot and the dst head's back-pointer.
	 */
	if (S_ext && D_ext) {
		ft_graft_glue_record_splice(c->gd, D_leaf, S_leaf);
		*nr_keys_ret = 1;
		return D;
	}

	/*
	 * Otherwise build a fresh internal branch M (a compressed side
	 * contributes its single forced byte; never external_nodes, which a
	 * compressed node may not carry).  Pass 1: add every union child's
	 * forward slot (cluster_leaf: no child back-pointer is written here --
	 * a later set_nth may reallocate M).  Recurse on shared bytes;
	 * reference (internal) or materialize (compressed) one-side bytes.
	 */
	if (S_comp)
		s_fb = cn_s->key_bytes[off_s];
	if (D_comp)
		d_fb = cn_d->key_bytes[off_d];
	for (b = 0; b < FT_ENTRY_PER_NODE; b++) {
		struct cds_ft_inode_flag *sc = NULL, *dc = NULL, *child;
		struct cds_ft_inode *old = NULL;
		bool sc_present, dc_present;
		unsigned long ck = 0;
		int ret;

		if (S_comp)
			sc_present = (b == s_fb);
		else if (!S_ext)
			sc_present = (sc = ft_node_get_nth_skip(S, NULL,
					(uint8_t) b, FT_PF_NONE)) != NULL;
		else
			sc_present = false;
		if (D_comp)
			dc_present = (b == d_fb);
		else if (!D_ext)
			dc_present = (dc = ft_node_get_nth_skip(D, NULL,
					(uint8_t) b, FT_PF_NONE)) != NULL;
		else
			dc_present = false;
		if (!sc_present && !dc_present)
			continue;

		if (sc_present && dc_present) {
			/*
			 * Both present -> recurse.  At most one side is
			 * compressed here (two divergent runs have different
			 * forced bytes), so the other side's target is its
			 * internal child at offset 0.
			 */
			struct cds_ft_inode_flag *ts, *td;
			unsigned int os, od;

			if (S_comp)
				ft_merge_advance(cn_s, off_s + 1, &ts, &os);
			else {
				ts = sc;
				os = 0;
			}
			if (D_comp)
				ft_merge_advance(cn_d, off_d + 1, &td, &od);
			else {
				td = dc;
				od = 0;
			}
			child = ft_merge_build(c, ts, os, td, od, depth + 1, &ck);
			if (child == FT_MERGE_OOM)
				return child;
		} else if (sc_present) {
			if (S_comp) {
				child = ft_merge_materialize_suffix(c, cn_s,
						off_s, depth + 1,
						/*dst_origin=*/ false, &ck);
				if (child == FT_MERGE_OOM)
					return child;
			} else {
				child = sc;	/* reference live src subtree */
				ck = ft_merge_child_count(ft, sc);
			}
		} else {
			if (D_comp) {
				child = ft_merge_materialize_suffix(c, cn_d,
						off_d, depth + 1,
						/*dst_origin=*/ true, &ck);
				if (child == FT_MERGE_OOM)
					return child;
			} else {
				child = dc;	/* reference live dst subtree */
				ck = ft_merge_child_count(ft, dc);
			}
		}
		ret = ft_node_set_nth(ft, &M, (uint8_t) b, child, &old, Mmeta,
				depth, /*cluster_leaf*/ true);
		if (ret)
			return FT_MERGE_OOM;
		if (old) {
			ft_graft_glue_untrack(ft, c->gd, old);
			free_cds_ft_node_unpublished(ft, old);
		}
		if (!tracked) {
			ft_graft_glue_track(c->gd, M);
			tracked = true;
			Mmeta = cds_ft_item_to_metadata(ft_node_ptr(M));
			/*
			 * Clear the recycled allocation's stale parent before
			 * any later set_nth reallocation copies it forward.
			 */
			rcu_assign_pointer(Mmeta->parent, NULL);
#ifdef FEATURE_FT_SKIP_COMPRESSED
			Mmeta->parent_slot_offset = 0;
#endif
		} else if (old) {
			ft_graft_glue_track(c->gd, M);
		}
		Mmeta = cds_ft_item_to_metadata(ft_node_ptr(M));
		total_keys += ck;
	}

	/* Merged external_nodes (the key terminating at M itself). */
	if (S_leaf && D_leaf) {
		M_ext = D_leaf;
		ft_graft_glue_record_splice(c->gd, D_leaf, S_leaf);
	} else if (D_leaf) {
		M_ext = D_leaf;
	} else {
		M_ext = S_leaf;		/* may be NULL */
	}
	if (M_ext) {
		ft_metadata_set_external_nodes(M, Mmeta, M_ext);
		total_keys += 1;
	}

	/*
	 * Pass 2: wire every child's back-pointer (deferred) plus M's
	 * external back-channel.  Fetch slots only now -- the Pass-1 set_nth
	 * reallocations may have moved them.  A fresh (recursed/materialized)
	 * child stores its parent immediately via defer_edge's is_fresh fast
	 * path; a referenced live child truly defers to commit.
	 */
	for (b = 0; b < FT_ENTRY_PER_NODE; b++) {
		struct cds_ft_inode_flag **slot;
		struct cds_ft_inode_flag *child =
			ft_node_get_nth_skip(M, &slot, (uint8_t) b, FT_PF_NONE);
		bool dst_origin;

		if (!child)
			continue;
		/*
		 * A referenced one-side child is dst-origin iff D still holds a
		 * child at this byte (a shared byte was recursed/materialized
		 * into a fresh cluster node, applied immediately by defer_edge's
		 * is_fresh path, so its origin is irrelevant).  dst-origin
		 * back-pointers are switched by the flip-latch after the forward
		 * publish, not by apply_deferred.
		 */
		if (D_comp)
			dst_origin = (b == d_fb);
		else
			dst_origin = !D_ext && ft_node_get_nth_skip(D, NULL,
					(uint8_t) b, FT_PF_NONE) != NULL;
		ft_graft_glue_defer_edge_origin(ft, c->gd, child, M, slot, dst_origin);
		/*
		 * A freshly-built compressed child (run / suffix) was stored as
		 * its PLAIN flag so is_fresh could match it by identity above;
		 * now that its edge is wired, re-encode the slot to the canonical
		 * skip form.  Referenced compressed children are already stored
		 * skip-encoded (a skip flag is not ft_node_compressed).
		 */
		if (ft_node_compressed(child)) {
			struct cds_ft_inode_flag *skip = ft_publish_compressed(ft,
				ft_compressed_node_ptr(child), child);

			if (skip != child)
				rcu_assign_pointer(*slot, skip);
		}
	}
	if (M_ext)
		ft_graft_glue_defer_edge_origin(ft, c->gd,
			(struct cds_ft_inode_flag *) M_ext, M, NULL,
			/*dst_origin=*/ D_leaf != NULL);

	/*
	 * Reclaim the INTERNAL overlap nodes M copied (compressed ones were
	 * recorded on entry above).  src -> gs, dst -> gd.
	 */
	if (!S_ext && !S_comp)
		ft_graft_glue_defer_free(c->gs, ft_node_ptr(S), false);
	if (!D_ext && !D_comp)
		ft_graft_glue_defer_free(c->gd, ft_node_ptr(D), false);

	ft_nr_keys_store(Mmeta, total_keys, CMM_RELAXED);
	*nr_keys_ret = total_keys;
	return M;
}

/*
 * Read-only pre-pass mirroring ft_merge_build's control flow, accumulating
 * upper bounds for ft_graft_glue_reserve so the build never asserts on a full
 * inline floor.  No allocation, no mutation.
 */
static
void ft_merge_count(struct cds_ft *ft, struct cds_ft_inode_flag *S, unsigned int off_s,
		struct cds_ft_inode_flag *D, unsigned int off_d,
		struct ft_merge_counts *cnt)
{
	struct cds_ft_compressed_node *cn_s, *cn_d;
	bool S_ext, D_ext, S_comp, D_comp;
	struct cds_ft_node *S_leaf, *D_leaf;
	unsigned int b, s_fb = 0, d_fb = 0;

	S = ft_resolve_skip_compressed(ft, S);
	D = ft_resolve_skip_compressed(ft, D);
	S_comp = ft_node_compressed(S);
	D_comp = ft_node_compressed(D);
	cn_s = S_comp ? ft_compressed_node_ptr(S) : NULL;
	cn_d = D_comp ? ft_compressed_node_ptr(D) : NULL;

	/* Compressed overlap nodes are freed whole, recorded on first entry. */
	if (S_comp && off_s == 0)
		cnt->nf_src++;
	if (D_comp && off_d == 0)
		cnt->nf_dst++;

	/* Shared run: one fresh run node + its child edge, then recurse past. */
	if (S_comp && D_comp) {
		unsigned int rem_s = cn_s->len - off_s;
		unsigned int rem_d = cn_d->len - off_d;
		unsigned int maxp = rem_s < rem_d ? rem_s : rem_d;
		unsigned int p = 0;

		while (p < maxp &&
		       cn_s->key_bytes[off_s + p] == cn_d->key_bytes[off_d + p])
			p++;
		if (p >= 1) {
			struct cds_ft_inode_flag *as, *ad;
			unsigned int aos, aod;

			cnt->nb++;
			cnt->nd++;
			ft_merge_advance(cn_s, off_s + p, &as, &aos);
			ft_merge_advance(cn_d, off_d + p, &ad, &aod);
			ft_merge_count(ft, as, aos, ad, aod, cnt);
			return;
		}
	}

	S_ext = ft_node_external(S);
	D_ext = ft_node_external(D);
	S_leaf = S_comp ? NULL : (S_ext ? (struct cds_ft_node *) ft_node_ptr(S)
		: cds_ft_item_to_metadata(ft_node_ptr(S))->external_nodes);
	D_leaf = D_comp ? NULL : (D_ext ? (struct cds_ft_node *) ft_node_ptr(D)
		: cds_ft_item_to_metadata(ft_node_ptr(D))->external_nodes);
	if (S_ext && D_ext) {
		cnt->ns++;
		return;
	}
	cnt->nb++;		/* fresh branch M */
	if (S_comp)
		s_fb = cn_s->key_bytes[off_s];
	if (D_comp)
		d_fb = cn_d->key_bytes[off_d];
	for (b = 0; b < FT_ENTRY_PER_NODE; b++) {
		struct cds_ft_inode_flag *sc = NULL, *dc = NULL;
		bool sc_present, dc_present;

		if (S_comp)
			sc_present = (b == s_fb);
		else if (!S_ext)
			sc_present = (sc = ft_node_get_nth_skip(S, NULL,
					(uint8_t) b, FT_PF_NONE)) != NULL;
		else
			sc_present = false;
		if (D_comp)
			dc_present = (b == d_fb);
		else if (!D_ext)
			dc_present = (dc = ft_node_get_nth_skip(D, NULL,
					(uint8_t) b, FT_PF_NONE)) != NULL;
		else
			dc_present = false;
		if (!sc_present && !dc_present)
			continue;
		cnt->nd++;		/* M -> child back-pointer edge */
		if (sc_present && dc_present) {
			struct cds_ft_inode_flag *ts, *td;
			unsigned int os, od;

			if (S_comp)
				ft_merge_advance(cn_s, off_s + 1, &ts, &os);
			else {
				ts = sc;
				os = 0;
			}
			if (D_comp)
				ft_merge_advance(cn_d, off_d + 1, &td, &od);
			else {
				td = dc;
				od = 0;
			}
			ft_merge_count(ft, ts, os, td, od, cnt);
		} else if ((sc_present && S_comp) || (dc_present && D_comp)) {
			/* Materialized suffix; a fresh wrapper adds a node+edge. */
			struct cds_ft_compressed_node *cn = sc_present ? cn_s : cn_d;
			unsigned int off = sc_present ? off_s : off_d;
			unsigned int suffix_len = cn->len - off - 1;

			if (suffix_len >= 1) {
				cnt->nb++;
				cnt->nd++;
			}
		}
	}
	if (S_leaf && D_leaf) {
		cnt->ns++;
		cnt->nd++;
	} else if (S_leaf || D_leaf) {
		cnt->nd++;
	}
	if (!S_ext && !S_comp)
		cnt->nf_src++;
	if (!D_ext && !D_comp)
		cnt->nf_dst++;
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

		/* Ordered list: swap whole lists (head/tail), mirroring the roots. */
		if (dst_ft->group->ordered_list_set) {
			struct ft_ord_cell *dh = dst_ft->ord_cell_head;
			struct ft_ord_cell *dt = dst_ft->ord_cell_tail;

			rcu_assign_pointer(dst_ft->ord_cell_head,
				swap_ft->ord_cell_head);
			rcu_assign_pointer(dst_ft->ord_cell_tail,
				swap_ft->ord_cell_tail);
			rcu_assign_pointer(swap_ft->ord_cell_head, dh);
			rcu_assign_pointer(swap_ft->ord_cell_tail, dt);
		}

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
		enum ft_graft_swap_case kase;
		struct cds_ft_metadata *pmeta, *swap_rmeta;
		struct cds_ft_inode_flag *old_child, *old_swap_root;
		struct cds_ft_inode *fresh = NULL;
		struct cds_ft_metadata *fresh_meta = NULL;
		struct ft_graft_glue glue_insert, glue_extract;
		struct cds_ft_inode_flag *canon = NULL;
		struct cds_ft_inode_flag *top_B = NULL;	/* extracted swap root, NULL = external/none */
		struct cds_ft_compressed_node *ks_cn = NULL;	/* key-shorter original cn */
		bool swap_empty;
		bool old_child_external = false;
		bool have_insert = false;
		unsigned long old_count = 0, swap_count;
		/* run_D = dst's subtree-at-key heads; run_S = swap's whole list. */
		struct ft_ord_cell *gs_d_first = NULL, *gs_d_last = NULL;
		struct ft_ord_cell *gs_s_first = NULL, *gs_s_last = NULL;
		bool gs_ord = dst_ft->group->ordered_list_set;

		/*
		 * Read-only descent: nothing is published, so the whole swap can be
		 * assembled as a build-invisible transaction and an allocation failure
		 * leaves both tries pristine.
		 */
		kase = ft_graft_swap_descend(dst_ft, key, key_len, &d);
		if (kase == FT_GRAFT_SWAP_DELEGATE) {
			/*
			 * No content at @key: the swap reduces to inserting swap_ft's
			 * content at @key, which empties swap_ft.  cds_ft_graft is itself
			 * a build-invisible transaction and empties the source.
			 * Pass the ORIGINAL application key: cds_ft_graft applies
			 * the key map itself, and the already-remapped @key would
			 * be remapped twice on a non-identity group (wrong graft
			 * point, wrong splice position).
			 */
			enum cds_ft_status s = cds_ft_graft(dst_ft, _key, _key_len,
					swap_ft);

			FT_TP(graft_swap_exit, (int) s);
			return s;
		}

		old_swap_root = swap_ft->root;
		swap_rmeta = ft_root_metadata(swap_ft);
		swap_empty = (swap_rmeta->nr_child == 0 && !swap_rmeta->external_nodes);
		swap_count = swap_empty ? 0 : ft_nr_keys_get(swap_rmeta);

		/*
		 * Identify the displaced old-child and its key count.  KEY_SHORTER: the
		 * extracted subtree is everything below the prefix, i.e. the suffix of
		 * the compressed node d.nf (its whole subtree count).  EXACT: d.nf is
		 * the displaced node.
		 */
		if (kase == FT_GRAFT_SWAP_KEY_SHORTER) {
			ks_cn = ft_compressed_node_ptr(d.nf);
			old_count = ft_nr_keys_get(
				cds_ft_item_to_metadata((struct cds_ft_inode *) ks_cn));
			old_child = ks_cn->child;
		} else {	/* FT_GRAFT_SWAP_EXACT */
			old_child = d.nf;
			if (!ft_node_external(old_child))
				old_count = ft_nr_keys_get(
					cds_ft_item_to_metadata(ft_node_ptr(old_child)));
			else
				old_count = 1;	/* one key (possibly a dup chain) */
			/*
			 * Skip-encoded externals carry a compressed prefix; treat them as
			 * non-external so the prefix is materialized into swap_ft's root.
			 */
			old_child_external = ft_node_external(old_child) &&
				!ft_node_skip_compressed(old_child);
		}

		ft_graft_glue_init(&glue_insert);
		ft_graft_glue_init(&glue_extract);

		/* ===== PREP: build clusters A and B (both tries pristine) ===== */

		/*
		 * Insert side (cluster A): canonicalized swap content, placed at the
		 * graft point in dst.  Empty swap inserts nothing (a remove).
		 */
		if (!swap_empty) {
			canon = ft_compress_single_child_if_needed(dst_ft,
				old_swap_root, &glue_insert);
			if (canon == (struct cds_ft_inode_flag *) (long) -ENOMEM)
				goto prep_oom;
		}

#ifdef FEATURE_FT_SKIP_COMPRESSED
		{
			/*
			 * EXACT + compressed parent + compressed canon: the slot already
			 * sits under a compressed node, so placing another compressed
			 * there would violate "no two adjacent compresseds".  Fuse them
			 * into one compressed at the grandparent slot.  Build-invisible:
			 * the merged cn's live child (canon's grandchild) is deferred,
			 * @canon (a fresh absorbed wrapper) is freed now, and the live
			 * parent cn is reclaimed at commit.
			 */
			struct cds_ft_compressed_node *pcn = NULL, *ccn = NULL;

			if (!swap_empty && kase == FT_GRAFT_SWAP_EXACT) {
				if (ft_node_skip_compressed(d.pnf))
					pcn = ft_skip_to_compressed(dst_ft, d.pnf);
				else if (ft_node_compressed(d.pnf))
					pcn = ft_compressed_node_ptr(d.pnf);
				if (ft_node_compressed(canon))
					ccn = ft_compressed_node_ptr(canon);
			}
			if (pcn && ccn &&
			    (unsigned int) pcn->len + ccn->len <= FT_SKIP_LEN_MAX) {
				struct cds_ft_metadata *pcn_meta =
					cds_ft_item_to_metadata((struct cds_ft_inode *) pcn);
				unsigned int merged_len = pcn->len + ccn->len;
				struct cds_ft_compressed_node *merged;
				struct cds_ft_metadata *merged_meta;
				struct cds_ft_inode_flag *merged_flag, *merged_skip;
				struct cds_ft_inode_flag **pub_slot;
				struct cds_ft_inode_flag *pub_parent;

				merged = alloc_compressed_node(dst_ft, merged_len,
						&merged_meta);
				if (!merged)
					goto prep_oom;
				memcpy(merged->key_bytes, pcn->key_bytes, pcn->len);
				memcpy(&merged->key_bytes[pcn->len], ccn->key_bytes,
					ccn->len);
				merged->len = (uint8_t) merged_len;
				merged->child = ccn->child;	/* live swap grandchild */
				merged_meta->nr_child = 1;
				ft_nr_keys_store(merged_meta,
					ft_nr_keys_get(pcn_meta), CMM_RELAXED);
				merged_meta->parent = pcn_meta->parent;
				pub_parent = pcn_meta->parent;
				pub_slot = ft_get_parent_slot(pcn_meta, dst_ft);
				ft_set_parent_slot(merged_meta, pub_slot);
				merged_flag = ft_compressed_node_flag(merged);
				ft_graft_glue_track(&glue_insert, merged_flag);
				ft_graft_glue_defer_edge(dst_ft, &glue_insert, ccn->child,
					merged_flag, &merged->child);
				/*
				 * @canon is the fresh wrapper just absorbed: drop it from
				 * tracking and free it (its deferred child edge is superseded
				 * by the one above via the defer-edge de-dup on @child).
				 */
				ft_graft_glue_untrack(dst_ft, &glue_insert, ccn);
				free_compressed_node_unpublished(dst_ft, ccn);
				merged_skip = ft_publish_compressed(dst_ft, merged,
						merged_flag);
				ft_graft_glue_set_publish(dst_ft, &glue_insert, pub_parent,
					pub_slot, merged_skip);
				ft_graft_glue_defer_free(&glue_insert, pcn, true);
				d.pnf = merged_flag;	/* count updates land on merged */
				have_insert = true;
			}
		}
#endif /* FEATURE_FT_SKIP_COMPRESSED */

		if (!swap_empty && !have_insert) {
			struct cds_ft_inode_flag *top_A;

			if (kase == FT_GRAFT_SWAP_KEY_SHORTER) {
				/*
				 * Replace the whole compressed node with a fresh prefix
				 * [d.depth, key_len) wrapping @canon (the chain-merge folds the
				 * prefix bytes into @canon when it is compressed).  d.pnf is the
				 * cn's parent (never compressed), so no grandparent fuse.
				 */
				top_A = ft_build_branch(dst_ft, key, d.depth, key_len,
						canon, swap_count, false, &glue_insert);
				if (!top_A)
					goto prep_oom;
				ft_graft_glue_defer_free(&glue_insert, ks_cn, true);
			} else {
				/* EXACT, simple replace of d.nf at d.nfp by @canon. */
				top_A = canon;
			}
			/*
			 * A compressed cluster top installs as the SKIP form in the live
			 * parent slot; its child's back-pointer is deferred, so the skip
			 * only resolves once ft_graft_glue_apply_deferred has run — which
			 * it does (before the forward publish) at commit.  The set_publish
			 * deferred edge (top -> d.pnf) is recorded LAST, so by the time it
			 * is applied the child back-pointer is already in place.
			 */
			if (ft_node_compressed(top_A))
				top_A = ft_publish_compressed(dst_ft,
					ft_compressed_node_ptr(top_A), top_A);
			ft_graft_glue_set_publish(dst_ft, &glue_insert, d.pnf, d.nfp, top_A);
			have_insert = true;
		}

		/*
		 * Extract side (cluster B): materialize the displaced subtree as
		 * swap_ft's new root.  KEY_SHORTER builds the root from the suffix path
		 * + live grandchild; EXACT runs the build-invisible make_root_internal
		 * on the displaced node.  External (or absent) content attaches as
		 * external_nodes at commit instead (no build).
		 */
		if (kase == FT_GRAFT_SWAP_KEY_SHORTER) {
			unsigned int prefix_len = (unsigned int) (key_len - d.depth);

			top_B = ft_build_extracted_root_glue(swap_ft, &glue_extract,
				ks_cn->key_bytes[prefix_len],
				&ks_cn->key_bytes[prefix_len + 1],
				ks_cn->len - prefix_len - 1,
				ks_cn->child, old_count);
			if (top_B == (struct cds_ft_inode_flag *) (long) -ENOMEM)
				goto prep_oom;
		} else if (!old_child_external && old_child) {
			top_B = ft_make_root_internal_glue(swap_ft, &glue_extract,
					old_child);
			if (top_B == (struct cds_ft_inode_flag *) (long) -ENOMEM)
				goto prep_oom;
		}

		/* Transient empty swap root for the unlink window (fallible). */
		if (!swap_empty) {
			fresh = alloc_cds_ft_node(swap_ft, &ft_types[0], &fresh_meta);
			if (!fresh)
				goto prep_oom;
		}

		/* ===== COMMIT (failure-free) ===== */

		/*
		 * Ordered list: capture both runs while both lists are intact.
		 * run_D = dst's subtree-at-key (old_child's heads), which becomes
		 * swap_ft's whole list; run_S = swap_ft's whole list, which replaces
		 * run_D in dst.  The mutations land at the matching structural
		 * sub-points below so the existing per-side syncs drain each side.
		 */
		if (gs_ord) {
			gs_d_first = ft_ord_cell_ptr(rcu_dereference(
				ft_subtree_minmax_head(dst_ft, old_child, false)->prev));
			gs_d_last = ft_ord_cell_ptr(rcu_dereference(
				ft_subtree_minmax_head(dst_ft, old_child, true)->prev));
			gs_s_first = swap_ft->ord_cell_head;	/* NULL if swap empty */
			gs_s_last = swap_ft->ord_cell_tail;
		}

		/*
		 * "Jump out" prevention: unlink old_swap_root from swap_ft (install
		 * @fresh) and drain its readers BEFORE its parent pointer is flipped
		 * into dst_ft.  Readers see an empty swap_ft between here and the final
		 * root install below.
		 */
		if (!swap_empty) {
			rcu_assign_pointer(swap_ft->root, ft_node_flag(fresh, 0));
			FT_TP(root_publish, (const void *) swap_ft,
				(const void *) swap_ft->root);
			/*
			 * run_S is captured; unlink it from swap's ordered list here
			 * (paired with the structural root unlink) so this sync drains
			 * swap ord-readers of run_S too.  run_D is installed as swap's
			 * list after the extract publish below.
			 */
			if (gs_ord) {
				swap_ft->ord_cell_head = NULL;
				swap_ft->ord_cell_tail = NULL;
			}
			if (!swap_ft->exclusive)
				swap_ft->group->flavor->update_synchronize_rcu();
		}

		/*
		 * Insert side: wire the deferred live back-pointers, then the single
		 * forward publish that splices cluster A into dst (detaching the old
		 * content).  Empty swap publishes NULL (a remove).
		 */
		if (have_insert) {
			ft_graft_glue_apply_deferred(dst_ft, &glue_insert);
			ft_graft_glue_publish(dst_ft, &glue_insert);
		} else {
			ft_publish_to_parent(dst_ft, d.pnf, d.nfp, NULL);
		}

		/*
		 * graft_swap edits the subtree at @key via ft_publish_to_parent
		 * directly (no ft_node_set_nth), so emit the structural edge for
		 * consumers.
		 */
		if (d.depth >= 1)
			FT_TP(tree_edge_set, (const void *) dst_ft,
				(const void *) d.pnf,
				(unsigned int) (d.depth - 1),
				(uint8_t) _key[d.depth - 1],
				(const void *) (have_insert ? glue_insert.top : NULL));

		/* Parent nr_child on the non-NULL -> NULL transition (remove). */
		pmeta = cds_ft_item_to_metadata(ft_node_ptr(d.pnf));
		if (!have_insert)
			pmeta->nr_child--;

		/* Propagate the external-count delta through the ancestors. */
		if (swap_count != old_count)
			ft_propagate_external_count_parent(dst_ft, d.pnf,
					(long) swap_count - (long) old_count);

		/*
		 * Replace run_D with run_S in dst's ordered list (run_S now lives at
		 * @key structurally; run_S NULL for an empty swap -> run_D just
		 * leaves).  Paired with the dst-side drain below, which removes any
		 * reader still holding run_D in dst.
		 */
		if (gs_ord)
			ft_ord_cell_run_replace(dst_ft, gs_d_first, gs_d_last,
				gs_s_first, gs_s_last);

		/*
		 * Drain dst-side readers that may still hold the displaced
		 * subtree (or any node within it) in their RCU snapshot with
		 * its OLD parent pointing into dst.  Without this sync, the
		 * extract apply_deferred below rewires that parent to point
		 * into cluster B (in swap_ft), and a reader walking up via
		 * the rewired pointer would CROSS-TRIE-ESCAPE from dst into
		 * swap_ft — observing top_B's NULL parent at non-root depth.
		 * The earlier sync at the swap unlink only drains swap_ft
		 * readers; this one drains dst_ft readers that captured the
		 * displaced data before it was detached from dst by the
		 * insert-side publish above.
		 *
		 * Exclusive dst carries no RCU readers, so the sync is
		 * skipped in that case.
		 */
		if (!dst_ft->exclusive)
			dst_ft->group->flavor->update_synchronize_rcu();

		/*
		 * Extract side: wire cluster B's deferred back-pointer, then install
		 * swap_ft's new root.  This re-parents the displaced subtree AFTER it
		 * has been detached from dst by the publish above and after the
		 * dst-side drain above.
		 */
		ft_graft_glue_apply_deferred(dst_ft, &glue_extract);
		if (top_B) {
			struct cds_ft_metadata *bm =
				cds_ft_item_to_metadata(ft_node_ptr(top_B));

			rcu_assign_pointer(bm->parent, NULL);
#ifdef FEATURE_FT_SKIP_COMPRESSED
			bm->parent_slot_offset = 0;
#endif
			rcu_assign_pointer(swap_ft->root, top_B);
			FT_TP(root_publish, (const void *) swap_ft,
				(const void *) swap_ft->root);
			if (swap_empty)
				free_cds_ft_node(swap_ft, ft_node_ptr(old_swap_root));
			else
				free_cds_ft_node(swap_ft, fresh);
		} else {
			/*
			 * External (or absent) displaced content: attach it as
			 * external_nodes on swap_ft's root (the transient @fresh for a
			 * non-empty swap, or old_swap_root's empty root for an empty swap).
			 */
			struct cds_ft_inode_flag *root_nf = swap_empty ?
				old_swap_root : ft_node_flag(fresh, 0);
			struct cds_ft_metadata *rm = swap_empty ?
				swap_rmeta : fresh_meta;

			if (old_child) {
				ft_metadata_set_external_nodes(root_nf, rm,
					(struct cds_ft_node *) ft_node_ptr(old_child));
				/*
				 * Root: parent is legitimately NULL.  Publishing
				 * prev here is safe (no fresh non-root cluster
				 * node in this back-pointer chain); kept paired
				 * with the metadata write for consistency with
				 * the other attach sites.
				 */
				ft_publish_external_nodes_prev(dst_ft, root_nf,
					(struct cds_ft_node *) ft_node_ptr(old_child));
				ft_nr_keys_store(rm, old_count, CMM_RELEASE);
			}
		}

		/*
		 * Install run_D (the extracted subtree's heads) as swap_ft's whole
		 * ordered list, mirroring the extract root publish above.  swap_ft
		 * was drained at the unlink sync, so clearing run_D's boundary links
		 * is a plain store; the head/tail publish uses rcu_assign.
		 */
		if (gs_ord) {
			gs_d_first->ord_prev = NULL;
			gs_d_last->ord_next = NULL;
			rcu_assign_pointer(swap_ft->ord_cell_head, gs_d_first);
			rcu_assign_pointer(swap_ft->ord_cell_tail, gs_d_last);
		}

		/* Reclaim the old (replaced) live nodes after the publishes. */
		ft_graft_glue_free_old(dst_ft, &glue_insert);
		ft_graft_glue_free_old(swap_ft, &glue_extract);

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
		 * swap_ft now holds content displaced from dst_ft; inherit dst_ft's
		 * access discipline for that content.  dst_ft keeps its own.
		 */
		swap_ft->exclusive = dst_ft->exclusive;
		FT_TP(graft_swap_exit, (int) CDS_FT_STATUS_OK);
		return CDS_FT_STATUS_OK;

	prep_oom:
		/*
		 * Allocation failed during the build: free every fresh glue node (both
		 * clusters), drop the transient swap root, and surface MEMORY_ERROR.
		 * No deferred edge was applied and nothing was published, so dst_ft and
		 * swap_ft are both pristine — there is nothing to roll back.
		 */
		ft_graft_glue_abort(dst_ft, &glue_insert);
		ft_graft_glue_abort(swap_ft, &glue_extract);
		if (fresh)
			free_cds_ft_node(swap_ft, fresh);
		FT_TP(graft_swap_exit, (int) CDS_FT_STATUS_MEMORY_ERROR);
		return CDS_FT_STATUS_MEMORY_ERROR;
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
			m->parent_slot_offset = 0;
#endif
		}
		uatomic_store(&detached->max_used_key_len,
			      uatomic_load(&ft->max_used_key_len, CMM_RELAXED),
			      CMM_RELAXED);

		/* Give source a fresh empty root. */
		rcu_assign_pointer(ft->root, ft_node_flag(fresh_node, 0));
		FT_TP(root_publish, (const void *) ft, (const void *) ft->root);

		/*
		 * Ordered list: a root detach moves the WHOLE trie, so @ft's
		 * entire ordered cell list becomes @detached's.  The cells'
		 * internal links are unchanged; only the head/tail endpoints
		 * transfer.  Matches the root-swap above (a concurrent reader
		 * mid-iteration follows its RCU snapshot into @detached).
		 */
		if (ft->group->ordered_list_set) {
			detached->ord_cell_head = ft->ord_cell_head;
			detached->ord_cell_tail = ft->ord_cell_tail;
			ft->ord_cell_head = NULL;
			ft->ord_cell_tail = NULL;
		}

		/*
		 * Drain @ft's readers that entered before the root swap and may
		 * still be inside the moved subtree (or parked in the moved
		 * ordered run): the exclusivity promise on @detached -- which a
		 * subsequent graft relies on to skip ITS grace period, and
		 * which makes mutation frees on @detached SYNCHRONOUS -- must
		 * hold at return, not eventually.  Skip for exclusive sources,
		 * which carry no RCU readers by construction.
		 */
		if (!ft->exclusive)
			ft->group->flavor->update_synchronize_rcu();

		*result_ft = detached;
		return CDS_FT_STATUS_OK;
	}

	/*
	 * key_len > 0: a plain key-guided descent to the detach target
	 * @child.  No branch-point snapshot is tracked here: ft_detach_node
	 * is bootstrapped from @child's own slot and recovers the surviving
	 * ancestor by climbing parent pointers (the same upward walk used by
	 * cds_ft_remove's count==1 prune), so the descent only has to locate
	 * @child, its slot, and its parent.
	 */
	{
		struct ft_descent d;
		const uint8_t *ik = key;

		ft_descent_init(&d, ft);

		for (; d.depth < key_len; ) {
			uint8_t kv;

			if (!d.nf)
				return CDS_FT_STATUS_NOT_FOUND;
			if (ft_node_external(d.nf))
				return CDS_FT_STATUS_NOT_FOUND;
			if (ft_node_compressed(d.nf)) {
				struct cds_ft_compressed_node *cn =
					ft_compressed_node_ptr(d.nf);

				ft_descent_traverse_compressed(&d, cn, &ik);
				continue;
			}
			kv = *(ik++);
			ft_descent_step(ft, &d, kv);
		}

		child = d.nf;

		if (!child)
			return CDS_FT_STATUS_NOT_FOUND;

		/*
		 * Compute the external node count of the subtree
		 * being detached before it is removed from the trie.
		 */
		{
			unsigned long detached_count;
			struct ft_graft_glue glue;
			struct cds_ft_inode_flag *new_root = NULL;

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
			ft_graft_glue_init(&glue);
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
			 * Materialize the detached trie's internal root NOW,
			 * build-invisibly, while nothing has been published:
			 * the trie root invariant requires an internal node,
			 * but a compressed/skip-compressed @child needs fresh
			 * allocations to peel its first path byte.  This is
			 * the LAST fallible step -- doing it after the detach
			 * publish would have no rollback (the subtree would be
			 * unreachable from both tries: silent data loss).  The
			 * fresh nodes are tracked in @glue, the live
			 * grandchild's back-pointer flip is deferred to the
			 * post-drain commit below, and the peeled compressed
			 * node's free is deferred likewise; an abort leaves
			 * the source pristine.
			 */
			if (!ft_node_external(child)) {
				new_root = ft_make_root_internal_glue(detached,
						&glue, child);
				if (new_root ==
				    (struct cds_ft_inode_flag *) (long) -ENOMEM) {
					ft_graft_glue_abort(detached, &glue);
					cds_ft_destroy(detached);
					return CDS_FT_STATUS_MEMORY_ERROR;
				}
			}

			/*
			 * Propagate count removal through ancestors
			 * before detach to avoid writing freed metadata.
			 */
			ft_propagate_external_count_parent(ft, d.pnf,
				-(long) detached_count);

			/*
			 * Detach child from the source trie and prune
			 * empty branches above.  After this, child is
			 * no longer reachable from the live trie for
			 * new readers.
			 */
			{
				/*
				 * Subtree-move detach (free_detached_subtree
				 * == false): preserve @child as the root of
				 * the new @detached trie.  Bootstrapped from
				 * @child's own slot, ft_detach_node climbs
				 * parent pointers to the surviving ancestor,
				 * unlinks the branch there, and its free-walk
				 * phase 1 reclaims the intermediate single-
				 * child chain between that ancestor and
				 * @child (the @nr_clear elevated links) while
				 * phase 2 -- which would free @child and
				 * below -- is gated off for move-style.  No
				 * explicit chain reclaim is needed here.
				 */
				int ret = ft_detach_node(ft,
							 d.nfp,
							 d.pnfp,
							 d.depth,
							 false);
				assert(ret != -ENOENT);
				if (ret < 0) {
					/*
					 * Recompaction failed (-ENOMEM).
					 * Undo propagation and abort.  The
					 * glue cluster is still invisible:
					 * the abort leaves @ft pristine.
					 */
					ft_propagate_external_count_parent(ft,
						d.pnf,
						(long) detached_count);
					ft_graft_glue_abort(detached, &glue);
					cds_ft_destroy(detached);
					return CDS_FT_STATUS_MEMORY_ERROR;
				}
			}

			/*
			 * Ordered list: the detached subtree's keys form a
			 * contiguous run in @ft's ordered cell list.  Move that
			 * run out of @ft and install it as @detached's entire
			 * list.  @child's subtree is intact (move-style detach),
			 * so its structural min/max heads are the run endpoints
			 * (the detach-point external_nodes, if any, are the run
			 * minimum -- they become @detached's NIL-key entries).
			 * The flip is atomic for a concurrent ordered reader; the
			 * internal-child branch's synchronize_rcu below then drains
			 * any @ft reader parked in the run.
			 */
			if (ft->group->ordered_list_set) {
				struct cds_ft_node *rfirst =
					ft_subtree_minmax_head(ft, child, false);
				struct cds_ft_node *rlast =
					ft_subtree_minmax_head(ft, child, true);

				ft_ord_cell_run_detach(ft, detached, rfirst, rlast);
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
				 * COMMIT (failure-free): the internal root was
				 * materialized build-invisibly BEFORE the
				 * detach published anything (see the
				 * ft_make_root_internal_glue call above).
				 * Wire the deferred live back-pointer (the
				 * grandchild moved under the fresh cluster --
				 * safe now, the drain above guarantees no
				 * reader still up-walks from inside the
				 * subtree), install the root, then reclaim the
				 * peeled-away compressed node.
				 */
				ft_graft_glue_apply_deferred(detached, &glue);
				free_cds_ft_node(detached,
					ft_node_ptr(detached->root));
				/* No readers in detached root yet. */
				detached->root = new_root;
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
						ft_node_ptr(new_root));
					rcu_assign_pointer(m->parent, NULL);
#ifdef FEATURE_FT_SKIP_COMPRESSED
					m->parent_slot_offset = 0;
#endif
				}
				ft_graft_glue_free_old(detached, &glue);
				ft_graft_glue_fini(&glue);
			} else {
				struct cds_ft_metadata *dmeta =
					ft_root_metadata(detached);

				/*
				 * Drain source-trie readers here too: they may
				 * still be parked on the detached external
				 * chain (a lookup that returned the head, a
				 * duplicate-chain walk) or in the moved ordered
				 * run.  The exclusivity promise on @detached
				 * must hold AT RETURN -- a subsequent graft of
				 * @detached legitimately skips its own grace
				 * period, and exclusive-mode mutations free
				 * SYNCHRONOUSLY, so a parked reader would
				 * dereference freed memory.  The drain also
				 * precedes the prev re-point below, so no
				 * reader's up-walk from the chain head can
				 * escape into @detached's root.  Skip for
				 * exclusive sources (no RCU readers by
				 * construction).
				 */
				if (!ft->exclusive)
					ft->group->flavor->update_synchronize_rcu();
				ft_metadata_set_external_nodes(detached->root, dmeta,
					(struct cds_ft_node *)
					ft_node_ptr(child));
				/*
				 * detached->root: parent is legitimately NULL.
				 * Pair the prev publish for consistency.
				 */
				ft_publish_external_nodes_prev(ft, detached->root,
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

/*
 * Flip-latch batch for cds_ft_merge_at: a urcu_flip_group plus its
 * proxies, allocated as one block and reclaimed together via call_rcu
 * once the proxied slots have been settled to their direct new targets.
 * ft_flip_proxy is 16-byte aligned so each proxy's address carries the
 * type-7 proxy tag (ft_flip_proxy_flag); malloc returns 16-byte-aligned
 * blocks at userspace addresses with the skip-len high bits clear.
 */
struct ft_flip_proxy {
	struct urcu_flip_proxy proxy;
} __attribute__((aligned(16)));

struct ft_flip_batch {
	struct rcu_head rcu_head;
	struct cds_ft *ft;
	struct urcu_flip_group group;
	unsigned int nr;
	unsigned int cap;
	struct ft_flip_proxy proxies[];
};

static
void ft_flip_batch_free_rcu(struct rcu_head *head)
{
	free(caa_container_of(head, struct ft_flip_batch, rcu_head));
}

static
struct ft_flip_batch *ft_flip_batch_alloc(struct cds_ft *ft, unsigned int cap)
{
	struct ft_flip_batch *b;

	b = malloc(sizeof(*b) + (size_t) cap * sizeof(struct ft_flip_proxy));
	if (!b)
		return NULL;
	b->ft = ft;
	urcu_flip_group_init(&b->group);
	b->nr = 0;
	b->cap = cap;
	return b;
}

/*
 * Free a flip batch that was allocated but never installed (no proxy stored
 * in any live slot, group never committed): a plain free, no grace period,
 * since no reader can reference it.  Used by the merge's last-fallible src
 * unlink abort path.
 */
static
void ft_flip_batch_free_unpublished(struct ft_flip_batch *b)
{
	free(b);
}

/* One release store: every proxy in the batch flips old -> new atomically. */
static
void ft_flip_batch_commit(struct ft_flip_batch *b)
{
	urcu_flip_commit(&b->group);
}

/*
 * Record a proxy {old_nf, new_nf} and return its tagged flag, to be stored
 * (sel == 0 -> old_nf, transparent) into the slot being flipped.
 */
static
struct cds_ft_inode_flag *ft_flip_batch_add(struct ft_flip_batch *b,
		struct cds_ft_inode_flag *old_nf,
		struct cds_ft_inode_flag *new_nf)
{
	struct urcu_flip_proxy *p;

	assert(b->nr < b->cap);
	p = &b->proxies[b->nr++].proxy;
	urcu_flip_proxy_init(p, &b->group, old_nf, new_nf);
	return ft_flip_proxy_flag(p);
}

/* Reclaim the batch after settle (deferred for in-flight readers). */
static
void ft_flip_batch_reclaim(struct ft_flip_batch *b)
{
	if (b->ft->exclusive)
		free(b);
	else
		b->ft->group->flavor->update_call_rcu(&b->rcu_head,
			ft_flip_batch_free_rcu);
}


/*
 * Ordinal-cell list maintenance.
 *
 * Mirrors the ORD_CHAIN chain maintenance, but the key-ordered doubly-linked
 * list threads the library-owned cells (one per distinct-key head) via
 * ft_ord_cell.ord_next / ord_prev instead of in-leaf fields.  Each point op
 * flips the (<=2) live neighbour edges through one flip-batch so a
 * bidirectional ordered reader sees the splice atomically; the spliced-in /
 * replacement cell pre-sets its own links with plain stores (not yet ord-
 * reachable), while an unspliced cell keeps its links for parked readers
 * until its deferred free.  Runtime-gated by group->ordered_list_set: a
 * point op consults these only when the list is enabled.
 *
 * RUNS UNDER WRITER EXCLUSION; no concurrent writer races, no proxy at rest.
 * Promotion (ft_unchain_node) and replace (cds_ft_replace) need NO list op:
 * the cell stays put and only cell->node is retargeted.  Bulk ops (merge /
 * graft / graft_swap / detach) maintain the list through the run helpers
 * below (run_detach / run_splice / run_replace / run_unlink) and the merge
 * interleave.
 */

struct ft_ord_cell_edge {
	struct ft_ord_cell **slot;	/* a neighbour's ord_next / ord_prev slot */
	struct ft_ord_cell *old_target;
	struct ft_ord_cell *new_target;
};

static void ft_ord_cell_flip(struct cds_ft *ft, struct ft_ord_cell_edge *edges,
		unsigned int n);

/*
 * Append an endpoint (ord_cell_head / ord_cell_tail) update to a flip-edge batch
 * when @slot currently holds @match, so the endpoint transitions ATOMICALLY with
 * the neighbour edges in the same flip: a reader resolving ord_cell_head/tail via
 * ft_ord_cell_resolve_ord then sees a consistent old-XOR-new view (it never
 * observes the old head pointer together with an already-flipped back-edge).
 * @match/@newval may be NULL (empty-list transitions); the proxy mechanism and
 * the *slot == @match guard both handle NULL.  Returns the new edge count.
 */
static
unsigned int ft_ord_cell_endpoint_edge(struct ft_ord_cell **slot,
		struct ft_ord_cell *match, struct ft_ord_cell *newval,
		struct ft_ord_cell_edge *edges, unsigned int n)
{
	if (*slot == match) {
		edges[n].slot = slot;
		edges[n].old_target = match;
		edges[n].new_target = newval;
		n++;
	}
	return n;
}

/*
 * Structural min/max dup-chain HEAD of the subtree rooted at @nf, under WRITER
 * EXCLUSION (no concurrent mutation -> no skip re-anchor / flip-proxy / transient
 * empty states to handle, unlike the reader-side minmax descent).  Mirrors the
 * key ordering the ordered cell list uses: a key that ends at an internal node
 * (metadata->external_nodes, a prefix key) sorts BEFORE every longer key under
 * it, so it is the subtree minimum.  Used to locate the endpoints of the
 * contiguous ordered-list run a bulk op relocates.
 */
static
struct cds_ft_node *ft_subtree_minmax_head(struct cds_ft *ft, struct cds_ft_inode_flag *nf,
		bool want_max)
{
	enum ft_direction dir = want_max ? FT_RIGHTMOST : FT_LEFTMOST;
	uint8_t scratch;

	for (;;) {
		nf = ft_resolve_skip_compressed(ft, nf);
		if (ft_node_external(nf))
			return (struct cds_ft_node *) ft_node_ptr(nf);
		if (ft_node_compressed(nf)) {
			struct cds_ft_compressed_node *cn =
				ft_compressed_node_ptr(nf);

			nf = rcu_dereference(cn->child);
			continue;
		}
		/* Internal node. */
		if (!want_max) {
			struct cds_ft_metadata *m =
				cds_ft_item_to_metadata(ft_node_ptr(nf));
			struct cds_ft_node *ext =
				ft_dereference_external(m->external_nodes);

			if (ext)
				return ext;	/* prefix key: subtree minimum */
		}
		nf = ft_node_get_minmax(ft, nf, &scratch, dir, false);
		assert(nf != NULL);
	}
}

/*
 * Move the contiguous ordered-list run whose endpoints are the cells of
 * @first_head .. @last_head (heads, in key order) OUT of @ft's ordered cell
 * list and install it as the ENTIRE ordered list of @into -- the cds_ft_detach
 * shape, where @into is a fresh EXCLUSIVE trie receiving exactly that subtree.
 * The run's internal ord links are preserved; only its two boundary edges in
 * @ft are flipped (atomic for a concurrent ordered reader, per the flip-latch),
 * @ft's head/tail are repaired, and @into's head/tail are set.  @into being
 * exclusive, clearing the run's new boundary links is a plain store.  Caller
 * gates on ordered_list_set.
 */
static
void ft_ord_cell_run_detach(struct cds_ft *ft, struct cds_ft *into,
		struct cds_ft_node *first_head, struct cds_ft_node *last_head)
{
	struct ft_ord_cell *first =
		ft_ord_cell_ptr(rcu_dereference(first_head->prev));
	struct ft_ord_cell *last =
		ft_ord_cell_ptr(rcu_dereference(last_head->prev));
	struct ft_ord_cell *pred = ft_ord_cell_resolve_ord(&first->ord_prev);
	struct ft_ord_cell *succ = ft_ord_cell_resolve_ord(&last->ord_next);
	struct ft_ord_cell_edge edges[4];
	unsigned int n = 0;

	if (pred) {
		edges[n].slot = &pred->ord_next;
		edges[n].old_target = first;
		edges[n].new_target = succ;
		n++;
	}
	if (succ) {
		edges[n].slot = &succ->ord_prev;
		edges[n].old_target = last;
		edges[n].new_target = pred;
		n++;
	}
	n = ft_ord_cell_endpoint_edge(&ft->ord_cell_head, first, succ, edges, n);
	n = ft_ord_cell_endpoint_edge(&ft->ord_cell_tail, last, pred, edges, n);
	ft_ord_cell_flip(ft, edges, n);
	/* @into is exclusive: no readers, plain stores. */
	first->ord_prev = NULL;
	last->ord_next = NULL;
	into->ord_cell_head = first;
	into->ord_cell_tail = last;
}

static
void ft_ord_cell_flip_prealloc(struct cds_ft *ft __attribute__((unused)),
		struct ft_ord_cell_edge *edges, unsigned int n,
		struct ft_flip_batch *b)
{
	unsigned int i;

	if (n == 0) {
		ft_flip_batch_free_unpublished(b);
		return;
	}
	assert(n <= b->cap);
	for (i = 0; i < n; i++)
		rcu_assign_pointer(*edges[i].slot,
			(struct ft_ord_cell *) ft_flip_batch_add(b,
				(struct cds_ft_inode_flag *) edges[i].old_target,
				(struct cds_ft_inode_flag *) edges[i].new_target));
	urcu_flip_commit(&b->group);
	for (i = 0; i < n; i++)
		rcu_assign_pointer(*edges[i].slot, edges[i].new_target);
	ft_flip_batch_reclaim(b);
}

static
void ft_ord_cell_flip(struct cds_ft *ft, struct ft_ord_cell_edge *edges,
		unsigned int n)
{
	struct ft_flip_batch *b;
	unsigned int i;

	if (n == 0)
		return;
	b = ft_flip_batch_alloc(ft, n);
	if (caa_unlikely(!b)) {
		/*
		 * Degraded fallback (point-op splices only, <= 3 edges; the
		 * merge interleave pre-allocates its batch in the fallible
		 * build phase and never lands here): sequential edge stores.
		 * A bidirectional reader between two stores can observe one
		 * neighbour's edge updated and the mirrored one not yet --
		 * transient and self-healing, never a dangling pointer.
		 */
		for (i = 0; i < n; i++)
			rcu_assign_pointer(*edges[i].slot, edges[i].new_target);
		return;
	}
	ft_ord_cell_flip_prealloc(ft, edges, n, b);
}

/*
 * Find the cell of the in-order predecessor (mode LT) / successor (mode GT)
 * of @key via the eager relational descent on the writer's cell scratch
 * iterator.  Returns NULL when none exists (@key is the new minimum/maximum).
 */
static
struct ft_ord_cell *ft_ord_cell_find_rel(struct cds_ft *ft, const uint8_t *key,
		size_t key_len, enum ft_lookup_inequality mode)
{
	struct cds_ft_iter *it = ft->ord_cell_scratch_iter;
	struct cds_ft_node *head;

	if (cds_ft_iter_set_key(it, key, key_len) != CDS_FT_STATUS_OK)
		return NULL;
	it->prefix_len = 0;
	it->node = NULL;
	if (cds_ft_lookup_inequality_impl(ft, it, mode, FT_LOOKUP_LIMIT_NONE,
			false, false) != CDS_FT_STATUS_OK)
		return NULL;
	head = cds_ft_iter_node(it);
	if (!head)
		return NULL;
	return ft_ord_cell_ptr(rcu_dereference(head->prev));
}

/*
 * Find the cell of the in-order predecessor of the freshly-inserted head
 * carried by @cell, WITHOUT re-descending from the root.  The insert just
 * walked root->leaf to attach the head, so its deepest node is the going-up
 * seed: position the writer's scratch iterator AT the new head (a live cursor)
 * and re-enter the relational lookup, which recovers the deepest node from
 * iter->node via the parent chain (its cross-call fast path) and runs the SAME
 * structural backtrack the key-based descent would -- but starting at the
 * divergence point instead of the root.  @seed_from_node suppresses the
 * ordinal-cell fast path (the new head's cell is not yet spliced).
 *
 * @key / @key_len are the head's APPLICATION-form key (set_key remaps): the
 * search key is written straight into iter_key (a memcpy, no up-walk), and the
 * cursor fields are seeded on top so read_key returns that buffer.
 *
 * Returns the predecessor cell, or NULL when the head's key is the new minimum.
 * A compressed/skip-compressed holder makes the impl fall back to a root
 * re-descent internally (correctness preserved, no descent saved for that key).
 */
static
struct ft_ord_cell *ft_ord_cell_find_pred_from_head(struct cds_ft *ft,
		const uint8_t *key, size_t key_len, struct ft_ord_cell *cell)
{
	struct cds_ft_iter *it = ft->ord_cell_scratch_iter;
	struct cds_ft_node *pred_head;

	/*
	 * Write the search key into iter_key (set_key clears cache_valid/node and
	 * sets key_len + key_off=0, path_len=0), then seed a live cursor AT the new
	 * head on top: cache_valid + node + path_len==key_depth drive the
	 * cross-call node-recovery fast path; prefix 0 = unscoped; ord_cell_node
	 * cleared so a fall-back cell cursor re-resolves from node->prev.
	 */
	it->node = NULL;
	if (cds_ft_iter_set_key(it, key, key_len) != CDS_FT_STATUS_OK)
		return NULL;
	it->node = cell->node;
	it->cache_valid = true;
	it->ord_cell_node = NULL;
	it->prefix_len = 0;
	it->path_len = it->key_len + 1;
	if (cds_ft_lookup_inequality_impl(ft, it, FT_LOOKUP_LT,
			FT_LOOKUP_LIMIT_NONE, false, true) != CDS_FT_STATUS_OK)
		return NULL;
	pred_head = cds_ft_iter_node(it);
	if (!pred_head)
		return NULL;
	return ft_ord_cell_ptr(rcu_dereference(pred_head->prev));
}

/*
 * Locate @cell's splice neighbours from its (parent-chain-wired) head and
 * pre-set the cell's own links -- invisible until the neighbour edges flip.
 * Shared by the legacy post-publish splice, the one-commit park and the
 * B-lite (external_nodes shape) pre-publish fill.
 */
static
void ft_ord_cell_prefill(struct cds_ft *ft, const uint8_t *key, size_t key_len,
		struct ft_ord_cell *cell, struct ft_ord_cell **pred_out,
		struct ft_ord_cell **succ_out)
{
	struct ft_ord_cell *pred, *succ;

	pred = ft_ord_cell_find_pred_from_head(ft, key, key_len, cell);
	if (pred)
		succ = ft_ord_cell_resolve_ord(&pred->ord_next);
	else
		/* New minimum: successor is the old list head (O(1), no descent). */
		succ = ft_ord_cell_resolve_ord(&ft->ord_cell_head);
	/* Pre-set @cell's own links; not yet reachable via the list. */
	cell->ord_prev = pred;
	cell->ord_next = succ;
	*pred_out = pred;
	*succ_out = succ;
}

/* Build the <= 4 visible neighbour edges for splicing @cell between
 * @pred / @succ.  Returns the edge count. */
static
unsigned int ft_ord_cell_splice_edges(struct cds_ft *ft,
		struct ft_ord_cell *cell, struct ft_ord_cell *pred,
		struct ft_ord_cell *succ, struct ft_ord_cell_edge *edges)
{
	unsigned int n = 0;

	if (pred) {
		edges[n].slot = &pred->ord_next;
		edges[n].old_target = succ;
		edges[n].new_target = cell;
		n++;
	}
	if (succ) {
		edges[n].slot = &succ->ord_prev;
		edges[n].old_target = pred;
		edges[n].new_target = cell;
		n++;
	}
	/* New min (!pred) => head was @succ; new max (!succ) => tail was @pred. */
	n = ft_ord_cell_endpoint_edge(&ft->ord_cell_head, succ, cell, edges, n);
	n = ft_ord_cell_endpoint_edge(&ft->ord_cell_tail, pred, cell, edges, n);
	return n;
}

/*
 * As ft_ord_cell_prefill, but locates the neighbours by a relational descent
 * on the key (the new key is still absent).  For the shapes whose fresh head
 * attaches as a holder's external_nodes (prefix / NIL keys): the from-head
 * seed needs the holder relation, which is only wired by the attach itself.
 */
static
void ft_ord_cell_prefill_by_key(struct cds_ft *ft, const uint8_t *key,
		size_t key_len, struct ft_ord_cell *cell,
		struct ft_ord_cell **pred_out, struct ft_ord_cell **succ_out)
{
	struct ft_ord_cell *pred, *succ;

	pred = ft_ord_cell_find_rel(ft, key, key_len, FT_LOOKUP_LT);
	if (pred)
		succ = ft_ord_cell_resolve_ord(&pred->ord_next);
	else
		succ = ft_ord_cell_resolve_ord(&ft->ord_cell_head);
	cell->ord_prev = pred;
	cell->ord_next = succ;
	*pred_out = pred;
	*succ_out = succ;
}

static
void ft_ord_cell_splice_at(struct cds_ft *ft, struct ft_ord_cell *cell,
		struct ft_ord_cell *pred, struct ft_ord_cell *succ)
{
	struct ft_ord_cell_edge edges[4];
	unsigned int n;

	n = ft_ord_cell_splice_edges(ft, cell, pred, succ, edges);
	ft_ord_cell_flip(ft, edges, n);
}

static
void ft_ord_cell_splice(struct cds_ft *ft, const uint8_t *key, size_t key_len,
		struct ft_ord_cell *cell)
{
	struct ft_ord_cell *pred, *succ;

	ft_ord_cell_prefill(ft, key, key_len, cell, &pred, &succ);
	ft_ord_cell_splice_at(ft, cell, pred, succ);
}

/*
 * One-commit insert tail (see struct ft_insert_commit): the structural slot
 * already holds a parked flip proxy (the fresh head is invisible -- the proxy
 * resolves to the old slot value), and the head's parent chain is fully wired,
 * so the splice-position search runs exactly as the post-publish splice did
 * (the from-head seed walks the parent chain, never the parked slot).  Park
 * the <= 4 ordered-list neighbour edges into the SAME batch, commit once --
 * the head becomes reachable in the structural index AND spliced into the
 * cell list atomically for every reader -- then settle all slots to their
 * direct values and finalize the real top's parent bookkeeping (skip_slot,
 * incoming_byte).
 */
static
void ft_insert_one_commit(struct cds_ft *ft, const uint8_t *key,
		size_t key_len, struct ft_ord_cell *cell,
		struct ft_insert_commit *ic)
{
	struct ft_ord_cell *pred, *succ;
	struct ft_ord_cell_edge edges[4];
	unsigned int i, n = 0;

	pred = ft_ord_cell_find_pred_from_head(ft, key, key_len, cell);
	if (pred)
		succ = ft_ord_cell_resolve_ord(&pred->ord_next);
	else
		/* New minimum: successor is the old list head (O(1), no descent). */
		succ = ft_ord_cell_resolve_ord(&ft->ord_cell_head);
	/* Pre-set @cell's own links; not yet reachable via the list. */
	cell->ord_prev = pred;
	cell->ord_next = succ;
	if (pred) {
		edges[n].slot = &pred->ord_next;
		edges[n].old_target = succ;
		edges[n].new_target = cell;
		n++;
	}
	if (succ) {
		edges[n].slot = &succ->ord_prev;
		edges[n].old_target = pred;
		edges[n].new_target = cell;
		n++;
	}
	/* New min (!pred) => head was @succ; new max (!succ) => tail was @pred. */
	n = ft_ord_cell_endpoint_edge(&ft->ord_cell_head, succ, cell, edges, n);
	n = ft_ord_cell_endpoint_edge(&ft->ord_cell_tail, pred, cell, edges, n);
	/* Park the ordered-list edges (each resolves to OLD until the commit). */
	for (i = 0; i < n; i++)
		rcu_assign_pointer(*edges[i].slot,
			(struct ft_ord_cell *) ft_flip_batch_add(ic->batch,
				(struct cds_ft_inode_flag *) edges[i].old_target,
				(struct cds_ft_inode_flag *) edges[i].new_target));

	/* THE commit: structural slot + ordered-list edges, atomically. */
	urcu_flip_commit(&ic->batch->group);

	/*
	 * Settle: direct values in every parked slot (idempotent for readers,
	 * the proxies already resolve to the new targets).  The real top's
	 * full wiring (parent, slot offset, incoming_byte) was done at park
	 * time, while still invisible.  A split-shape park settles through
	 * ft_publish_to_parent for its dual skip-slot maintenance; the attach
	 * shape's set_nth already did its own bookkeeping, so a direct store
	 * of the same canonical value suffices.
	 */
	if (ic->publish_to_parent)
		ft_publish_to_parent(ft, ic->parent_nf, ic->slot,
			ic->slot_value);
	else
		rcu_assign_pointer(*ic->slot, ic->slot_value);
	for (i = 0; i < n; i++)
		rcu_assign_pointer(*edges[i].slot, edges[i].new_target);
	ft_flip_batch_reclaim(ic->batch);
	ic->batch = NULL;
	/*
	 * The old compressed node a split replaced: readers resolved the
	 * proxy to it until the commit above, so only now may its grace-
	 * period-deferred free be queued.
	 */
	if (ic->free_old_cn)
		free_compressed_node(ft, ic->free_old_cn);
}

/* Remove @cell from the ordered cell list (its key disappeared). */
static
void ft_ord_cell_unsplice(struct cds_ft *ft, struct ft_ord_cell *cell)
{
	struct ft_ord_cell *pred = ft_ord_cell_resolve_ord(&cell->ord_prev);
	struct ft_ord_cell *succ = ft_ord_cell_resolve_ord(&cell->ord_next);
	struct ft_ord_cell_edge edges[4];
	unsigned int n = 0;

	if (pred) {
		edges[n].slot = &pred->ord_next;
		edges[n].old_target = cell;
		edges[n].new_target = succ;
		n++;
	}
	if (succ) {
		edges[n].slot = &succ->ord_prev;
		edges[n].old_target = cell;
		edges[n].new_target = pred;
		n++;
	}
	n = ft_ord_cell_endpoint_edge(&ft->ord_cell_head, cell, succ, edges, n);
	n = ft_ord_cell_endpoint_edge(&ft->ord_cell_tail, cell, pred, edges, n);
	/* @cell keeps its links for parked readers until its deferred free. */
	ft_ord_cell_flip(ft, edges, n);
}

/*
 * Replace @old_cell with @new_cell at the same list position (insert_replace:
 * a fresh head's cell takes the replaced head's cell slot).  @new_cell
 * inherits @old_cell's neighbours; @old_cell keeps its links for parked
 * readers until its deferred free.  O(1): reuses @old_cell's neighbours, no
 * relational descent.
 */
static
void ft_ord_cell_swap(struct cds_ft *ft, struct ft_ord_cell *old_cell,
		struct ft_ord_cell *new_cell)
{
	struct ft_ord_cell *pred = ft_ord_cell_resolve_ord(&old_cell->ord_prev);
	struct ft_ord_cell *succ = ft_ord_cell_resolve_ord(&old_cell->ord_next);
	struct ft_ord_cell_edge edges[4];
	unsigned int n = 0;

	new_cell->ord_prev = pred;
	new_cell->ord_next = succ;
	if (pred) {
		edges[n].slot = &pred->ord_next;
		edges[n].old_target = old_cell;
		edges[n].new_target = new_cell;
		n++;
	}
	if (succ) {
		edges[n].slot = &succ->ord_prev;
		edges[n].old_target = old_cell;
		edges[n].new_target = new_cell;
		n++;
	}
	n = ft_ord_cell_endpoint_edge(&ft->ord_cell_head, old_cell, new_cell, edges, n);
	n = ft_ord_cell_endpoint_edge(&ft->ord_cell_tail, old_cell, new_cell, edges, n);
	ft_ord_cell_flip(ft, edges, n);
}

/*
 * Locate the ordered-list neighbours (@pred, @succ) that a run grafted at @key
 * will splice between.  MUST be called while @dst is still payload-free (before
 * the structural attach publishes the grafted subtree), else the relational
 * descent would return a payload head as the boundary.  @key is APPLICATION form
 * (find_rel remaps).  Since the attach point is empty, @pred = last @dst key <
 * @key and @succ = first @dst key > @key (nothing of @dst's lies in the run's
 * range in between).
 */
static
void ft_ord_cell_find_splice_pos(struct cds_ft *dst, const uint8_t *key,
		size_t key_len, struct ft_ord_cell **pred_out,
		struct ft_ord_cell **succ_out)
{
	struct ft_ord_cell *pred, *succ;
	size_t flen = dst->group->key_len;

	if (flen != CDS_FT_LEN_VARIABLE && key_len != flen) {
		/*
		 * Internal graft on a FIXED-length group (ft_graft_keylen, e.g.
		 * cds_ft_merge_at's detach+graft path): @key is the graft
		 * PREFIX, shorter than the group's key length, which the public
		 * relational lookups reject -- probing with it verbatim came
		 * back empty and the grafted run was silently never spliced
		 * (merged keys invisible to ordered iteration).  Probe with the
		 * prefix PADDED to the fixed length instead: the attach point
		 * is empty (graft returns POPULATED_ERROR otherwise), so no
		 * @dst key starts with @key, and
		 *   LT(key . min..min) = last @dst key below the prefix range,
		 *   GT(key . max..max) = first @dst key above it.
		 * Pad bytes are the ordinal-space extremes mapped back to
		 * application form (find_rel remaps app -> ordinal).
		 */
		const struct cds_ft_key_map *km = &dst->group->key_map;
		uint8_t pad_min = km->identity ? 0x00 : km->ordinal_to_key[0x00];
		uint8_t pad_max = km->identity ? 0xff : km->ordinal_to_key[0xff];
		uint8_t pad[FT_MAX_KEY_LEN];

		assert(key_len < flen && flen <= FT_MAX_KEY_LEN);
		memcpy(pad, key, key_len);
		memset(pad + key_len, pad_min, flen - key_len);
		pred = ft_ord_cell_find_rel(dst, pad, flen, FT_LOOKUP_LT);
		if (pred) {
			succ = ft_ord_cell_resolve_ord(&pred->ord_next);
		} else {
			memset(pad + key_len, pad_max, flen - key_len);
			succ = ft_ord_cell_find_rel(dst, pad, flen,
					FT_LOOKUP_GT);
		}
		*pred_out = pred;
		*succ_out = succ;
		return;
	}

	pred = ft_ord_cell_find_rel(dst, key, key_len, FT_LOOKUP_LT);
	if (pred)
		succ = ft_ord_cell_resolve_ord(&pred->ord_next);
	else
		succ = ft_ord_cell_find_rel(dst, key, key_len, FT_LOOKUP_GT);
	*pred_out = pred;
	*succ_out = succ;
}

/*
 * Splice the contiguous ordered-list run [@run_first .. @run_last] (already
 * linked internally, in key order) into @dst's ordered cell list BETWEEN the
 * given neighbours @pred and @succ -- the cds_ft_graft shape, where the run is
 * the source trie's whole list attached at an EMPTY point in @dst (graft returns
 * POPULATED_ERROR otherwise, so no @dst key interleaves the run's range).
 *
 * @pred / @succ MUST be located BEFORE the structural attach publishes the
 * payload into @dst (see ft_ord_cell_find_splice_pos): a relational descent run
 * after the payload is live would return a PAYLOAD head (part of the run itself)
 * as the boundary.  @pred / @succ are @dst-original cells, which graft never
 * moves, so they stay valid until this splice.
 *
 * Pre-sets the run's outer links (run not yet reachable in @dst), flips the
 * <=2 boundary edges atomically (for @dst's live readers), and repairs @dst
 * head/tail.  The run's source trie must already have released it (head/tail
 * cleared + a grace period) so no source reader is mid-run.
 */
static
void ft_ord_cell_run_splice(struct cds_ft *dst, struct ft_ord_cell *run_first,
		struct ft_ord_cell *run_last, struct ft_ord_cell *pred,
		struct ft_ord_cell *succ)
{
	struct ft_ord_cell_edge edges[4];
	unsigned int n = 0;

	/* Pre-set the run's outer links; not yet reachable via @dst's list. */
	run_first->ord_prev = pred;
	run_last->ord_next = succ;
	if (pred) {
		edges[n].slot = &pred->ord_next;
		edges[n].old_target = succ;
		edges[n].new_target = run_first;
		n++;
	}
	if (succ) {
		edges[n].slot = &succ->ord_prev;
		edges[n].old_target = pred;
		edges[n].new_target = run_last;
		n++;
	}
	n = ft_ord_cell_endpoint_edge(&dst->ord_cell_head, succ, run_first, edges, n);
	n = ft_ord_cell_endpoint_edge(&dst->ord_cell_tail, pred, run_last, edges, n);
	ft_ord_cell_flip(dst, edges, n);
}

/*
 * Replace the run [@d_first .. @d_last] currently in @dst's ordered list with
 * the run [@s_first .. @s_last] at the SAME position -- the cds_ft_graft_swap
 * shape, where @dst's subtree-at-key (run_D) is swapped out for the swap trie's
 * content (run_S).  @s_first may be NULL (empty swap -> run_D just leaves and the
 * gap closes).  The position is taken from run_D's own neighbours (no relational
 * descent: the swap exchanges two subtrees at the same key, so run_S lands
 * exactly where run_D was).  Atomic for @dst's live readers via one flip of the
 * <=2 boundary edges.  run_D keeps its links for parked readers; the caller
 * re-homes run_D into the swap trie afterwards.
 */
static
void ft_ord_cell_run_replace(struct cds_ft *dst,
		struct ft_ord_cell *d_first, struct ft_ord_cell *d_last,
		struct ft_ord_cell *s_first, struct ft_ord_cell *s_last)
{
	struct ft_ord_cell *pred = ft_ord_cell_resolve_ord(&d_first->ord_prev);
	struct ft_ord_cell *succ = ft_ord_cell_resolve_ord(&d_last->ord_next);
	struct ft_ord_cell *new_first = s_first ? s_first : succ;
	struct ft_ord_cell *new_last = s_last ? s_last : pred;
	struct ft_ord_cell_edge edges[4];
	unsigned int n = 0;

	if (s_first) {
		/* Pre-set run_S's outer links; not yet reachable via @dst. */
		s_first->ord_prev = pred;
		s_last->ord_next = succ;
	}
	if (pred) {
		edges[n].slot = &pred->ord_next;
		edges[n].old_target = d_first;
		edges[n].new_target = new_first;
		n++;
	}
	if (succ) {
		edges[n].slot = &succ->ord_prev;
		edges[n].old_target = d_last;
		edges[n].new_target = new_last;
		n++;
	}
	n = ft_ord_cell_endpoint_edge(&dst->ord_cell_head, d_first, new_first, edges, n);
	n = ft_ord_cell_endpoint_edge(&dst->ord_cell_tail, d_last, new_last, edges, n);
	ft_ord_cell_flip(dst, edges, n);
}

/*
 * Remove the contiguous run [@first_head .. @last_head] from @ft's ordered list
 * WITHOUT re-homing it -- the cds_ft_merge source side, where the run's cells
 * disperse (survivors are spliced into dst, collided heads are freed).  Relink
 * the two boundary edges (atomic for @ft's live readers) and repair head/tail;
 * the run cells keep their stale links (caller no longer references them as a
 * run).  Whole-list removal (pred == succ == NULL) clears head/tail.
 */
static
void ft_ord_cell_run_unlink(struct cds_ft *ft, struct cds_ft_node *first_head,
		struct cds_ft_node *last_head)
{
	struct ft_ord_cell *first =
		ft_ord_cell_ptr(rcu_dereference(first_head->prev));
	struct ft_ord_cell *last =
		ft_ord_cell_ptr(rcu_dereference(last_head->prev));
	struct ft_ord_cell *pred = ft_ord_cell_resolve_ord(&first->ord_prev);
	struct ft_ord_cell *succ = ft_ord_cell_resolve_ord(&last->ord_next);
	struct ft_ord_cell_edge edges[4];
	unsigned int n = 0;

	if (pred) {
		edges[n].slot = &pred->ord_next;
		edges[n].old_target = first;
		edges[n].new_target = succ;
		n++;
	}
	if (succ) {
		edges[n].slot = &succ->ord_prev;
		edges[n].old_target = last;
		edges[n].new_target = pred;
		n++;
	}
	n = ft_ord_cell_endpoint_edge(&ft->ord_cell_head, first, succ, edges, n);
	n = ft_ord_cell_endpoint_edge(&ft->ord_cell_tail, last, pred, edges, n);
	ft_ord_cell_flip(ft, edges, n);
}

/*
 * Interleave the surviving source cells into @dst's ordered list after a
 * cds_ft_merge_at spine-copy commit.  Walks the merged subtree at @dst_key in
 * key order (@merged_keys distinct heads) via the structural inequality oracle,
 * two-pointering against @dst's ORIGINAL region cells: @ord_cursor steps through
 * those (captured before the commit, min head of the dst merge subtree), and any
 * walked head that is NOT the cursor cell is a surviving src head -> splice it
 * after the last placed cell.  Dst-original cells are left untouched (their
 * relative order is preserved by the merge); collided src heads were demoted to
 * duplicates + their cells freed, so the walk never sees them.  @prev_placed
 * starts at the region's predecessor (@ord_cursor's ord_prev).
 *
 * Identity key_map only (the seed uses @dst_key directly; matches the rest of
 * the ordered-list machinery).  Uses @dst's writer-exclusive scratch iterator.
 */
static
void ft_merge_ord_interleave(struct cds_ft *dst, const uint8_t *dst_key,
		size_t dst_key_len, unsigned long merged_keys,
		struct ft_ord_cell *ord_cursor, struct ft_ord_cell *prev_placed,
		struct ft_ord_cell_edge *edges, struct ft_flip_batch *flip_b)
{
	struct cds_ft_iter *it = dst->ord_cell_scratch_iter;
	unsigned long i;

	/*
	 * Seed at the merge region's minimum, via the descent oracle
	 * (cache_valid = false: the cell list is mid-update, so the cell fast
	 * path must not be used).  A root merge (@dst_key_len == 0) merges the
	 * whole trie -> seed at the GLOBAL minimum with LIMIT_FIRST, which ignores
	 * the search key (a zero-length key is invalid for cds_ft_iter_set_key /
	 * a relational GE on a FIXED-length group, so the relational path would
	 * bail and leave the src run unspliced).  A scoped merge (@dst_key_len >
	 * 0) needs the first key >= @dst_key, so it sets @dst_key and uses the
	 * relational GE (LIMIT_NONE), which honors @dst_key.
	 */
	it->node = NULL;
	it->cache_valid = false;
	if (dst_key_len == 0) {
		it->key_len = 0;
		it->prefix_len = 0;
		it->key_off = 0;
		if (cds_ft_lookup_inequality_impl(dst, it, FT_LOOKUP_GE,
				FT_LOOKUP_LIMIT_FIRST, false, false) != CDS_FT_STATUS_OK)
			goto unused;
	} else {
		const uint8_t *seed_key = dst_key;
		size_t seed_len = dst_key_len;
		size_t flen = dst->group->key_len;
		uint8_t padbuf[FT_MAX_KEY_LEN];

		/*
		 * FIXED-length group with a mid-key merge point: @dst_key is
		 * shorter than the group's key length, which the relational GE
		 * rejects -- pad it to the fixed length with the ordinal-space
		 * minimum so GE finds the merged region's first key (every key
		 * in the region extends @dst_key, hence sorts at or above the
		 * padded probe; everything below the region sorts under it).
		 * @dst_key is ALREADY ORDINAL here (cds_ft_merge_at converts
		 * once at entry), so pad with raw 0x00 and set the iterator key
		 * without the public set_key's key-map application.
		 */
		if (flen != CDS_FT_LEN_VARIABLE && dst_key_len != flen) {
			assert(dst_key_len < flen && flen <= FT_MAX_KEY_LEN);
			memcpy(padbuf, dst_key, dst_key_len);
			memset(padbuf + dst_key_len, 0x00,
				flen - dst_key_len);
			seed_key = padbuf;
			seed_len = flen;
		}
		ft_iter_set_key_ordinals(it, seed_key, seed_len);
		it->prefix_len = 0;
		it->node = NULL;
		it->cache_valid = false;
		if (cds_ft_lookup_inequality_impl(dst, it, FT_LOOKUP_GE,
				FT_LOOKUP_LIMIT_NONE, false, false) != CDS_FT_STATUS_OK)
			goto unused;
	}
	/*
	 * Single-flip re-weave.  Walk the merged region in key order; pre-set
	 * each surviving src cell's links with plain stores (the cell is not yet
	 * ord-reachable in @dst, so this is invisible) and accumulate ONLY the
	 * VISIBLE boundary edges -- a dst-original cell's ord_next / ord_prev, or
	 * @dst's head / tail -- into one batch.  A single flip then commits the
	 * entire interleave atomically, so a concurrent ordered reader never
	 * observes a partially re-woven list (the per-splice path was N
	 * independent flips).  Dst-original cells keep their relative order, so a
	 * dst<->dst step needs no edge; each survivor RUN costs at most two edges
	 * (one entering, one leaving), so 2 * merged_keys + 2 bounds the batch.
	 * @edges and @flip_b are pre-allocated by the caller in the merge's
	 * fallible BUILD phase (this runs in the failure-free commit tail, where
	 * an alloc failure would force a non-atomic fallback).
	 */
	{
		struct ft_ord_cell *prev = prev_placed;
		bool prev_is_dst = (prev_placed != NULL);
		unsigned int n = 0;

		for (i = 0; i < merged_keys; i++) {
			struct cds_ft_node *head = cds_ft_iter_node(it);
			struct ft_ord_cell *cell;

			if (!head)
				break;
			cell = ft_ord_cell_ptr(rcu_dereference(head->prev));
			if (cell == ord_cursor) {
				/*
				 * Dst-original cell: stays put, already linked in
				 * key order.  Its back edge changes only when a
				 * survivor run was just placed before it.
				 */
				if (prev && !prev_is_dst) {
					prev->ord_next = cell;	/* survivor: invisible */
					edges[n].slot = &cell->ord_prev;
					edges[n].old_target =
						ft_ord_cell_resolve_ord(&cell->ord_prev);
					edges[n].new_target = prev;
					n++;
				}
				prev = cell;
				prev_is_dst = true;
				ord_cursor = ft_ord_cell_resolve_ord(
					&ord_cursor->ord_next);
			} else {
				/* Surviving src cell: pre-set its back link. */
				cell->ord_prev = prev;	/* invisible */
				if (!prev) {
					/* new list minimum: flip @dst head. */
					edges[n].slot = &dst->ord_cell_head;
					edges[n].old_target =
						ft_ord_cell_resolve_ord(&dst->ord_cell_head);
					edges[n].new_target = cell;
					n++;
				} else if (prev_is_dst) {
					/* dst -> survivor: flip the dst cell's fwd edge. */
					edges[n].slot = &prev->ord_next;
					edges[n].old_target =
						ft_ord_cell_resolve_ord(&prev->ord_next);
					edges[n].new_target = cell;
					n++;
				} else {
					prev->ord_next = cell;	/* survivor: invisible */
				}
				prev = cell;
				prev_is_dst = false;
			}
			it->cache_valid = false;	/* force the descent oracle */
			if (cds_ft_lookup_inequality_impl(dst, it, FT_LOOKUP_GT,
					FT_LOOKUP_LIMIT_NONE, false, false) != CDS_FT_STATUS_OK)
				break;
		}
		/*
		 * Close the trailing edge: if the last placed cell is a survivor,
		 * link it to the region successor (@ord_cursor, advanced past the
		 * last dst-original; NULL at the list tail) and flip that
		 * neighbour's back edge -- or @dst's tail when there is none.
		 */
		if (prev && !prev_is_dst) {
			prev->ord_next = ord_cursor;	/* survivor: invisible */
			if (!ord_cursor) {
				edges[n].slot = &dst->ord_cell_tail;
				edges[n].old_target =
					ft_ord_cell_resolve_ord(&dst->ord_cell_tail);
				edges[n].new_target = prev;
				n++;
			} else {
				edges[n].slot = &ord_cursor->ord_prev;
				edges[n].old_target =
					ft_ord_cell_resolve_ord(&ord_cursor->ord_prev);
				edges[n].new_target = prev;
				n++;
			}
		}
		ft_ord_cell_flip_prealloc(dst, edges, n, flip_b);
	}
	return;
unused:
	/* Seed failed (empty region): the pre-allocated batch goes unused. */
	ft_flip_batch_free_unpublished(flip_b);
}

/*
 * Set @child's parent back-pointer to a raw flag @value (a flip-proxy)
 * verbatim, WITHOUT touching parent_slot_offset.  Mirrors ft_set_parent's
 * child-kind dispatch; the settle step later replaces @value with the
 * real parent via ft_set_parent (which does maintain skip_slot).
 */
static
void ft_set_parent_raw(struct cds_ft *ft, struct cds_ft_inode_flag *child,
		struct cds_ft_inode_flag *value)
{
	(void) ft;
	if (!child)
		return;
	/* Flip-proxy child: transient slot value, not a node (see
	 * ft_set_parent); the parking mutator wires the real child. */
	if (caa_unlikely(ft_node_flip_proxy(child)))
		return;
#ifdef FEATURE_FT_SKIP_COMPRESSED
	if (ft_node_skip_compressed(child)) {
		struct cds_ft_compressed_node *cn = ft_skip_to_compressed(ft, child);

		rcu_assign_pointer(cds_ft_item_to_metadata(
			(struct cds_ft_inode *) cn)->parent, value);
		return;
	}
	if (ft_node_compressed(child)) {
		struct cds_ft_compressed_node *cn = ft_compressed_node_ptr(child);

		rcu_assign_pointer(cds_ft_item_to_metadata(
			(struct cds_ft_inode *) cn)->parent, value);
		return;
	}
#endif
	if (ft_node_external(child)) {
		/*
		 * Ordered list on: store the raw flag (a flip-proxy) into the head's
		 * cell->parent, not over its prev (which is the cell pointer).  The
		 * read path resolves cell->parent THEN the flip-proxy, so a proxy
		 * parked in cell->parent settles correctly; the later ft_set_parent
		 * replaces it with the real parent.  List off / non-cell: the head's
		 * prev IS the flagged parent, so store the proxy directly there.
		 */
		if (ft->ordered_list)
			ft_ord_cell_set_parent((struct cds_ft_node *) child, value);
		else
			rcu_assign_pointer(((struct cds_ft_node *) child)->prev, value);
		return;
	}
	rcu_assign_pointer(cds_ft_item_to_metadata(ft_node_ptr(child))->parent,
		value);
}

/*
 * Unlink the EXACT subtree at @src_key from @src_ft IN PLACE, preserving the
 * subtree node so the spine-copy merge can keep referencing it (it is
 * re-parented into the merged cluster at commit).  This is the non-root-src
 * analogue of the root-src ft->root swap: it removes the merge source from
 * @src_ft and prunes the now-empty single-child branch above it.
 *
 * The descent + ft_detach_node + chain reclaim mirror ft_detach_keylen's
 * non-root path, but with @free_detached_subtree = false and WITHOUT wrapping
 * the subtree in a transient trie (the merge owns it via @gd's referenced
 * edges; wrapping it would double-own it).  @detached_count is the subtree's
 * unique-key count (from ft_merge_descend), propagated out of the ancestors.
 *
 * The caller invokes this as the LAST fallible commit step: on -ENOMEM
 * (ft_detach_node recompaction failed) @src_ft is left pristine, so the caller
 * aborts the still-invisible build with both tries intact -- no rollback.  On
 * success the caller drains @src_ft and runs the failure-free commit tail.
 */
static
int ft_merge_unlink_src_subtree(struct cds_ft *src_ft,
		const uint8_t *_src_key, size_t src_key_len,
		unsigned long detached_count)
{
	/*
	 * @src_key is ALREADY ORDINAL (cds_ft_merge_at converts once at its
	 * entry); a second key-map application here would descend a different
	 * subtree than the one the spine build copied.
	 */
	const uint8_t *key = _src_key, *ik;
	struct ft_descent d;
	int ret;

	ik = key;

	/*
	 * Plain key-guided descent to the merge source's subtree root.  As in
	 * ft_detach_keylen, no branch-point snapshot is tracked: ft_detach_node
	 * is bootstrapped from the target's own slot and climbs parent pointers
	 * to the surviving ancestor.  A KEY_SHORTER source (the key ends inside
	 * a compressed node) overshoots that node -- @d.nf becomes its child --
	 * and the climb's free walk reclaims the whole compressed node while
	 * preserving @d.nf, matching the old explicit chain reclaim.
	 */
	ft_descent_init(&d, src_ft);
	for (; d.depth < src_key_len; ) {
		uint8_t kv;

		/* Caller already established EXACT, so the path is present. */
		if (ft_node_compressed(d.nf)) {
			struct cds_ft_compressed_node *cn =
				ft_compressed_node_ptr(d.nf);

			ft_descent_traverse_compressed(&d, cn, &ik);
			continue;
		}
		kv = *(ik++);
		ft_descent_step(src_ft, &d, kv);
	}

	/* Propagate the removal through ancestors before touching freed slots. */
	ft_propagate_external_count_parent(src_ft, d.pnf,
			-(long) detached_count);

	/*
	 * Unlink the branch in place, preserving the move target (@d.nf, the
	 * subtree root the spine-copy merge keeps referencing).  Bootstrapped
	 * from the target's own slot, ft_detach_node climbs to the surviving
	 * ancestor and its free-walk phase 1 reclaims the intermediate single-
	 * child chain (compressed / skip-target nodes included) while phase 2 --
	 * which would free the target -- stays gated off for move-style.
	 */
	ret = ft_detach_node(src_ft, d.nfp, d.pnfp, d.depth,
			/*free_detached_subtree=*/ false);
	if (ret < 0) {
		/* Recompaction OOM: undo the propagation; src is pristine. */
		ft_propagate_external_count_parent(src_ft, d.pnf,
				(long) detached_count);
		return -ENOMEM;
	}
	return 0;
}

/*
 * KEY_SHORTER dst: the merge point sits @off bytes inside a compressed dst
 * node whose prefix bytes @key[@d_depth .. @key_len) lie ABOVE it.  Wrap the
 * freshly-built merged node @M (with @mk unique keys) under that prefix and
 * return the PLAIN flag to publish into the old compressed node's forward slot
 * (the caller skip-encodes it), or FT_MERGE_OOM.
 *
 * @M internal: ft_build_branch lays a compressed prefix over the fresh
 * internal M -- M's edge into the prefix applies immediately via defer_edge's
 * is_fresh fast path, so ft_build_branch's dst_origin=false leaf edge is
 * correct, and M's own children keep the dst_origin ft_merge_build gave them.
 * A 1-child-no-external M is first canonicalized to compressed (its lone child
 * is always a FRESH recursion result, so the fold is dst_origin-safe too).
 *
 * @M compressed: M is a fresh run produced by ft_merge_build_run (the sole
 * path returning a compressed top), whose child may be a LIVE dst splice head
 * carrying dst_origin=true (a duplicate-key merge through the compressed node).
 * ft_build_branch's chain-merge would re-defer that child as dst_origin=false
 * and demote a live re-parent out of the flip latch, so fuse the prefix into M
 * here by hand, carrying the run child's dst_origin (always true).  The fused
 * length @off + M->len <= cn_d->len, so it is a valid compressed length.
 */
static
struct cds_ft_inode_flag *ft_merge_wrap_prefix(struct ft_merge_ctx *c,
		const uint8_t *key, unsigned int d_depth, unsigned int key_len,
		struct cds_ft_inode_flag *M, unsigned long mk)
{
	struct cds_ft *ft = c->dst_ft;
	unsigned int prefix_len = key_len - d_depth;

	if (ft_node_compressed(M)) {
		struct cds_ft_compressed_node *mcn = ft_compressed_node_ptr(M);
		unsigned int merged_len = prefix_len + mcn->len;
		struct cds_ft_compressed_node *merged;
		struct cds_ft_metadata *merged_meta;
		struct cds_ft_inode_flag *mflag;

		merged = alloc_compressed_node(ft, (uint8_t) merged_len,
				&merged_meta);
		if (!merged)
			return FT_MERGE_OOM;
		memcpy(merged->key_bytes, &key[d_depth], prefix_len);
		memcpy(&merged->key_bytes[prefix_len], mcn->key_bytes, mcn->len);
		merged->len = (uint8_t) merged_len;
		merged->child = mcn->child;
		merged_meta->nr_child = 1;
		ft_nr_keys_store(merged_meta, mk, CMM_RELAXED);
		mflag = ft_compressed_node_flag(merged);
		ft_graft_glue_track(c->gd, mflag);
		/*
		 * Carry the run child's dst_origin (ft_merge_build_run records
		 * it true): a fresh child applies via is_fresh, a live splice
		 * head flips with the forward slot.  The defer de-dup supersedes
		 * M's stale &mcn->child edge with this one (same @child).
		 */
		ft_graft_glue_defer_edge_origin(ft, c->gd, mcn->child, mflag,
				&merged->child, /*dst_origin=*/ true);
		ft_graft_glue_untrack(ft, c->gd, mcn);
		free_compressed_node_unpublished(ft, mcn);
		return mflag;	/* PLAIN; caller skip-encodes before publish */
	} else {
		struct cds_ft_inode_flag *canon, *top;

		canon = ft_compress_single_child_if_needed(ft, M, c->gd);
		if (canon == (struct cds_ft_inode_flag *) (long) -ENOMEM)
			return FT_MERGE_OOM;
		top = ft_build_branch(ft, key, d_depth, key_len, canon, mk,
				/*has_external_nodes=*/ false, c->gd);
		if (!top)
			return FT_MERGE_OOM;
		return top;	/* PLAIN compressed; caller skip-encodes */
	}
}

/*
 * Keys in ORDINAL form (cds_ft_merge_at converts the caller's application
 * keys once at its entry; every consumer below -- the spine build, the source
 * unlink, the ordered-list interleave -- expects ordinal bytes).
 *
 * Build-invisible spine-copy merge of @src_ft's subtree at @src_key into
 * @dst_ft at the live EXACT subtree @d_dst.  All allocation is in the build
 * phase (plus the single fallible src unlink at commit for a non-root src), so
 * any OOM frees the fresh copies and leaves both tries pristine -- no rollback.
 *
 * @src_ft merged at @src_key (@d_src its subtree, @cnt_src its key count) into
 * @d_dst, a non-empty dst subtree.  A root src (src_key_len == 0) unlinks via
 * the ft->root swap; a non-root src unlinks its branch in place at commit (the
 * only post-build fallible step).  @off_src selects the src merge-point shape:
 * 0 is an EXACT subtree at @d_src->nf; > 0 is KEY_SHORTER (the src key ends
 * @off_src bytes inside the compressed node @d_src->nf -- the build enters that
 * node at the cursor, and ft_merge_unlink_src_subtree reclaims the whole node
 * via its chain-reclaim, the move-style detach preserving cn_s->child).  Both
 * root and non-root dst merge points are handled: a non-root point flips the
 * interior forward slot through a type-7 proxy that the read-side descent
 * resolves at every child fetch.  Compressed overlaps are handled.  @off_dst
 * selects the dst merge-point shape: 0 is an EXACT subtree at @d_dst->nf; > 0
 * is KEY_SHORTER (the dst key ends @off_dst bytes inside the compressed node
 * @d_dst->nf, whose prefix bytes are wrapped around the merged cluster by
 * ft_merge_wrap_prefix).
 *
 * Every (EXACT | KEY_SHORTER) src x (EXACT | KEY_SHORTER) dst shape is handled.
 *
 * Returns OK on a committed merge, or MEMORY_ERROR on OOM (both tries pristine).
 */
static
enum cds_ft_status ft_merge_spine_copy(struct cds_ft *dst_ft,
		struct cds_ft *src_ft, struct ft_descent *d_src,
		const uint8_t *src_key, size_t src_key_len, unsigned long cnt_src,
		unsigned int off_src, struct ft_descent *d_dst,
		unsigned long cnt_dst, unsigned int off_dst,
		const uint8_t *dst_key, size_t dst_key_len)
{
	struct ft_graft_glue gd, gs;
	struct ft_merge_ctx ctx = { .dst_ft = dst_ft, .gd = &gd, .gs = &gs };
	struct ft_merge_counts cnt = { 0, 0, 0, 0, 0 };
	bool root_src = (src_key_len == 0);
	bool ks_dst = (off_dst > 0);
	bool ed;
	struct cds_ft_inode_flag *S = d_src->nf;
	struct cds_ft_inode_flag *D = d_dst->nf;
	struct cds_ft_inode_flag *M, *M_slot = NULL, *pub, *D_old;
	struct cds_ft_inode_flag *pub_parent = d_dst->pnf, **pub_slot = d_dst->nfp;
	struct cds_ft_inode *fresh_root = NULL;
	struct cds_ft_metadata *fresh_meta;
	struct ft_flip_batch *flip;
	unsigned long merged_keys = 0;
	bool ms_ord = dst_ft->group->ordered_list_set;
	struct ft_ord_cell *ms_cursor = NULL, *ms_prev = NULL;
	struct cds_ft_node *ms_s_first = NULL, *ms_s_last = NULL;
	struct ft_ord_cell_edge *ms_edges = NULL;
	struct ft_flip_batch *ms_flip = NULL;

	/*
	 * Every dst merge-point shape is handled.  The flip proxies the publish
	 * slot @pub_slot, the read-side descent resolves the type-7 proxy at every
	 * child fetch (ft_resolve_flip_proxy, before the skip handler), and the
	 * published node's parent is wired to @pub_parent by
	 * ft_graft_glue_set_publish, so a descent and an up-walk see a coherent
	 * old-XOR-merged view across the flip.
	 *
	 * Edge D: the merge point's PARENT is a COMPRESSED node cn_p reached via a
	 * grandparent skip slot.  cn_p's own parent cannot be compressed ("no two
	 * adjacent compressed" invariant), so the grandparent slot d_dst->pnfp is a
	 * plain internal slot that merely *holds* skip(cn_p).  Handle it one level
	 * up: the merge is EXACT at d_dst->nf (off_dst == 0), M is wrapped under a
	 * fresh copy of the WHOLE cn_p, and that copy is published into d_dst->pnfp
	 * in place of skip(cn_p) -- identical machinery to a KEY_SHORTER dst wrap,
	 * just with cn_p as the wrapped node and the grandparent as the publish
	 * point.  ks_dst and ed are mutually exclusive (a KEY_SHORTER merge point
	 * sits inside cn_d, whose parent is internal).
	 */
	ed = (d_dst->pnf &&
	      ft_node_compressed(ft_resolve_skip_compressed(dst_ft, d_dst->pnf)));

	/* Size both glues from a read-only pre-pass (with headroom). */
	ft_merge_count(dst_ft, S, off_src, D, off_dst, &cnt);
	ft_graft_glue_init(&gd);
	ft_graft_glue_init(&gs);
	if (ft_graft_glue_reserve(&gd, cnt.nb + 8, cnt.nd + 8,
				cnt.nf_dst + 8, cnt.ns + 8) ||
	    ft_graft_glue_reserve(&gs, 0, 0, cnt.nf_src + 8, 0)) {
		ft_graft_glue_fini(&gd);
		ft_graft_glue_fini(&gs);
		return CDS_FT_STATUS_MEMORY_ERROR;
	}

	/* Root src: pre-allocate the fresh empty root for the commit swap. */
	if (root_src) {
		fresh_root = alloc_cds_ft_node(src_ft, &ft_types[0], &fresh_meta);
		if (!fresh_root) {
			ft_graft_glue_fini(&gd);
			ft_graft_glue_fini(&gs);
			return CDS_FT_STATUS_MEMORY_ERROR;
		}
		rcu_assign_pointer(fresh_meta->parent, NULL);
		ft_nr_keys_store(fresh_meta, 0, CMM_RELAXED);
	}

	/* Build the merged cluster invisibly (the only build-phase fallible step). */
	M = ft_merge_build(&ctx, S, off_src, D, off_dst, 0, &merged_keys);
	if (M == FT_MERGE_OOM) {
		if (fresh_root)
			free_cds_ft_node_unpublished(src_ft, fresh_root);
		ft_graft_glue_abort(dst_ft, &gd);
		ft_graft_glue_abort(src_ft, &gs);
		return CDS_FT_STATUS_MEMORY_ERROR;
	}

	/*
	 * Compute @pub (the value to publish), @pub_parent / @pub_slot (where) and,
	 * for the compressed shapes, reclaim the replaced node.
	 *
	 * KEY_SHORTER dst (off_dst > 0): the merge point sits off_dst bytes inside
	 * the compressed node cn_d = D, whose prefix bytes lie above it; wrap M
	 * under cn_d->key_bytes[0..off_dst) and replace cn_d at its own slot
	 * (d_dst->nfp).  Edge D (ed): the merge point's parent is the compressed
	 * node cn_p; wrap M under the WHOLE cn_p and replace cn_p at the grandparent
	 * slot d_dst->pnfp (pub_parent = d_dst->ppnf).  Either way reclaim the
	 * replaced compressed node -- ft_merge_build entered it (or, for Edge D, did
	 * not touch it) without recording the free.  ft_merge_wrap_prefix
	 * canonicalizes a single-child M and preserves the dst_origin of a
	 * compressed M's child.
	 *
	 * EXACT dst with an internal/absent parent (off_dst == 0, !ed): M itself
	 * replaces the subtree.  Canonicalize a non-root single-child internal merge
	 * top: when S and D contribute exactly one shared byte (a churned "ab"-prefix
	 * shape), ft_merge_build returns M as a 1-child internal with no
	 * external_nodes -- forbidden at a non-root position under skip mode
	 * (chain-compress invariant; cds_ft_verify catches it at the merge depth).
	 * Collapse it to a 1-byte compressed via the glue.  Only the TOP M can hit
	 * this; its lone child is always a FRESH recursion result, so the deferred
	 * child edge carries dst_origin=false correctly and disturbs no flip edge.
	 * A root dst merge point (d_dst->pnf == NULL) is exempt: a 1-child internal
	 * is canonical at the root.
	 */
	if (ks_dst || ed) {
		struct cds_ft_compressed_node *wrap_cn;
		uint8_t kbuf[FT_MAX_KEY_LEN];
		unsigned int wrap_depth, wrap_len;

		if (ed) {
			wrap_cn = ft_compressed_node_ptr(
				ft_resolve_skip_compressed(dst_ft, d_dst->pnf));
			wrap_len = wrap_cn->len;
			wrap_depth = (unsigned int) d_dst->depth - wrap_len;
			pub_parent = d_dst->ppnf;
			pub_slot = d_dst->pnfp;
		} else {	/* ks_dst */
			wrap_cn = ft_compressed_node_ptr(D);
			wrap_len = off_dst;
			wrap_depth = (unsigned int) d_dst->depth;
		}
		ft_graft_glue_defer_free(&gd, wrap_cn, true);
		memcpy(&kbuf[wrap_depth], wrap_cn->key_bytes, wrap_len);
		pub = ft_merge_wrap_prefix(&ctx, kbuf, wrap_depth,
				wrap_depth + wrap_len, M, merged_keys);
		if (pub == FT_MERGE_OOM) {
			if (fresh_root)
				free_cds_ft_node_unpublished(src_ft, fresh_root);
			ft_graft_glue_abort(dst_ft, &gd);
			ft_graft_glue_abort(src_ft, &gs);
			return CDS_FT_STATUS_MEMORY_ERROR;
		}
	} else if (pub_parent) {
		pub = ft_compress_single_child_if_needed(dst_ft, M, &gd);
		if (pub == (struct cds_ft_inode_flag *) (long) -ENOMEM) {
			if (fresh_root)
				free_cds_ft_node_unpublished(src_ft, fresh_root);
			ft_graft_glue_abort(dst_ft, &gd);
			ft_graft_glue_abort(src_ft, &gs);
			return CDS_FT_STATUS_MEMORY_ERROR;
		}
	} else {
		pub = M;
	}

	/*
	 * Allocate the flip batch: one proxy per dst-origin re-parent edge,
	 * plus one for the merge-point forward slot.  Fallible -> abort the
	 * still-invisible build; both tries stay pristine.
	 */
	{
		unsigned int nr_dst = 0;
		int j;

		for (j = 0; j < gd.nr_deferred; j++)
			if (gd.deferred[j].dst_origin)
				nr_dst++;
		flip = ft_flip_batch_alloc(dst_ft, nr_dst + 1);
	}
	if (!flip) {
		if (fresh_root)
			free_cds_ft_node_unpublished(src_ft, fresh_root);
		ft_graft_glue_abort(dst_ft, &gd);
		ft_graft_glue_abort(src_ft, &gs);
		return CDS_FT_STATUS_MEMORY_ERROR;
	}
	ft_graft_glue_set_publish(dst_ft, &gd, pub_parent, pub_slot, pub);

	/*
	 * Slot-canonical form of @pub for the @pub_slot stores (both the flip
	 * proxy's new target and the settle): a compressed @pub is published
	 * SKIP-ENCODED, exactly as a direct slot write would store it, so a reader
	 * resolving the proxy gets a value identical in encoding to a normal slot
	 * read and runs the same skip handling.  set_publish wired @pub's parent +
	 * skip_slot via the plain flag already; an internal @pub (EXACT only -- a
	 * compressed-wrap @pub is always compressed) needs no re-encode.
	 *
	 * @D_old is the flip proxy's old target -- the value @pub_slot currently
	 * holds.  Read it from the live slot: for the compressed shapes the descent
	 * resolved d->nf / d->pnf to the PLAIN flag while the slot holds the SKIP
	 * form, and *pub_slot is untouched until the flip (the build is invisible
	 * and apply_deferred wires back-pointers, not this forward slot).
	 */
	if (ft_node_compressed(pub))
		M_slot = ft_publish_compressed(dst_ft,
				ft_compressed_node_ptr(pub), pub);
	else
		M_slot = pub;
	D_old = *pub_slot;

	/*
	 * Ordered list: capture the dst merge subtree's min head (the cursor for
	 * the post-commit interleave walk) and the region predecessor, while D is
	 * still intact (the build only copied its spine; the flip below moves its
	 * leaves into M).  The surviving src cells are spliced in after the commit.
	 */
	if (ms_ord) {
		ms_cursor = ft_ord_cell_ptr(rcu_dereference(
			ft_subtree_minmax_head(dst_ft, D, false)->prev));
		ms_prev = ft_ord_cell_resolve_ord(&ms_cursor->ord_prev);
		/*
		 * Capture src's merged-subtree (S) run endpoints now, while S is
		 * still intact, but defer the actual run-unlink until AFTER the
		 * last fallible step (ft_merge_unlink_src_subtree).  The run
		 * unlink is an externally observable ordered-list mutation; doing
		 * it here would leave src's list inconsistent if the src subtree
		 * unlink below OOMs and we roll the whole merge back.  The unlink
		 * still happens before the drain, so sync drains src ord-readers
		 * of the run too.
		 */
		ms_s_first = ft_subtree_minmax_head(dst_ft, S, false);
		ms_s_last = ft_subtree_minmax_head(dst_ft, S, true);
		/*
		 * Pre-allocate the interleave's edge batch + flip batch NOW,
		 * while the build is still abortable.  The interleave runs in
		 * the failure-free commit tail; an alloc failure there would
		 * have to degrade to a non-atomic per-edge fallback, exposing a
		 * half-re-woven list to bidirectional ordered readers under
		 * memory pressure.  Each survivor run costs at most two visible
		 * edges, so 2 * merged_keys + 2 bounds both.
		 */
		{
			unsigned int ms_cap =
				2u * (unsigned int) merged_keys + 2u;

			ms_edges = malloc((size_t) ms_cap * sizeof(*ms_edges));
			if (ms_edges)
				ms_flip = ft_flip_batch_alloc(dst_ft, ms_cap);
			if (!ms_edges || !ms_flip) {
				free(ms_edges);
				ft_flip_batch_free_unpublished(flip);
				if (fresh_root)
					free_cds_ft_node_unpublished(src_ft,
						fresh_root);
				ft_graft_glue_abort(dst_ft, &gd);
				ft_graft_glue_abort(src_ft, &gs);
				return CDS_FT_STATUS_MEMORY_ERROR;
			}
		}
	}

	/*
	 * 1. Unlink the merge source from src.  Root src: swap in the pre-
	 *    allocated empty root.  Non-root src: detach its branch in place --
	 *    the LAST fallible step.  On its OOM both tries are pristine (the
	 *    detach self-undoes, the cluster is still unpublished), so abort the
	 *    build; this preserves the no-rollback property.  Then drain src
	 *    readers of the moved content.
	 *
	 *    ===== Everything from the drain onward is failure-free. =====
	 */
	if (root_src) {
		rcu_assign_pointer(src_ft->root, ft_node_flag(fresh_root, 0));
		FT_TP(root_publish, (const void *) src_ft, (const void *) src_ft->root);
	} else if (ft_merge_unlink_src_subtree(src_ft, src_key, src_key_len,
				cnt_src) < 0) {
		free(ms_edges);
		if (ms_flip)
			ft_flip_batch_free_unpublished(ms_flip);
		ft_flip_batch_free_unpublished(flip);
		ft_graft_glue_abort(dst_ft, &gd);
		ft_graft_glue_abort(src_ft, &gs);
		return CDS_FT_STATUS_MEMORY_ERROR;
	}
	/*
	 * Now that the last fallible step has committed, remove src's merged
	 * subtree (S) run from src's ordered list: its cells disperse to dst
	 * (survivors) or are freed (collisions).  Deferred to here so an OOM in
	 * the src unlink above leaves src's list untouched on rollback; done
	 * before the drain so sync drains src ord-readers of the run too.
	 */
	if (ms_ord)
		ft_ord_cell_run_unlink(src_ft, ms_s_first, ms_s_last);
	if (!src_ft->exclusive)
		src_ft->group->flavor->update_synchronize_rcu();

	/*
	 * 2. Re-parent the SRC-origin referenced subtrees directly: the src
	 *    drain above made them unreachable to readers, and the cluster is
	 *    not yet forward-published, so this is invisible.  dst-origin
	 *    edges are NOT applied here -- they go through the flip.
	 */
	ft_graft_glue_apply_deferred(dst_ft, &gd);

	/*
	 * 3. Stage the flip: point every dst-origin child's parent, and the
	 *    merge-point forward slot, at a proxy that still resolves to the
	 *    OLD target (selector 0).  Transparent to readers: up-walks and
	 *    the root descent still see the pre-merge dst.
	 */
	{
		int j;

		for (j = 0; j < gd.nr_deferred; j++) {
			struct cds_ft_inode_flag *child, *old_parent, *pf;

			if (!gd.deferred[j].dst_origin)
				continue;
			child = gd.deferred[j].child;
			/*
			 * Current parent = the old dst overlap node.  Resolve a
			 * skip-encoded picked child to its compressed node first:
			 * ft_get_parent_rcu reads node metadata and cannot take a
			 * skip pointer (ft_set_parent_raw below resolves itself).
			 */
			old_parent = ft_get_parent_rcu(dst_ft,
				ft_resolve_skip_compressed(dst_ft, child));
			pf = ft_flip_batch_add(flip, old_parent,
				gd.deferred[j].parent);
			ft_set_parent_raw(dst_ft, child, pf);
		}
		/* Publish slot: old dst subtree -> merged cluster. */
		rcu_assign_pointer(*pub_slot,
				ft_flip_batch_add(flip, D_old, M_slot));
	}

	/*
	 * 4. Flip: one release store switches every dst-origin parent AND the
	 *    forward slot from old to merged, atomically.  Because the forward
	 *    slot flips with the back-pointers, a reader (which descends then
	 *    walks up) only ever progresses old->merged, never regresses.
	 */
	urcu_flip_commit(&flip->group);

	/* 5. Concatenate same-key duplicate chains (dst now reachable via M). */
	ft_graft_glue_apply_splices(dst_ft, &gd);

	/*
	 * 6. Propagate the dst key-count delta through the ancestors, starting at
	 *    @pub_parent (the parent of the replaced node -- d_dst->pnf for an
	 *    EXACT / KEY_SHORTER point, the grandparent d_dst->ppnf for Edge D).
	 *    @pub is a fresh node already carrying merged_keys, so the walk must
	 *    begin one level up; @pub_parent is a plain internal flag (cn_p's
	 *    parent cannot be compressed), safe for ft_node_ptr.
	 */
	if (pub_parent && merged_keys != cnt_dst)
		ft_propagate_external_count_parent(dst_ft, pub_parent,
			(long) merged_keys - (long) cnt_dst);

	/*
	 * 7. Settle: rewrite each proxied slot to its direct merged target
	 *    (idempotent for readers -- the proxy already resolves to merged),
	 *    so the proxies become unreferenced and reclaimable.
	 */
	{
		int j;

		for (j = 0; j < gd.nr_deferred; j++)
			if (gd.deferred[j].dst_origin)
				ft_set_parent(dst_ft, gd.deferred[j].child,
					gd.deferred[j].parent,
					gd.deferred[j].slot);
		rcu_assign_pointer(*pub_slot, M_slot);
	}

	/*
	 * 8. Reclaim, all deferred: the flip batch (proxies now unreferenced)
	 *    and the old overlap spines (src-side to src, dst-side to dst).
	 *    No dst synchronize_rcu -- the flip subsumed the dst drain.
	 */
	ft_flip_batch_reclaim(flip);
	ft_graft_glue_free_old(src_ft, &gs);
	ft_graft_glue_free_old(dst_ft, &gd);

	/*
	 * 9. Ordered list: splice the surviving src cells into dst's ordered
	 * list at their merged positions.  Done AFTER the settle (step 7) so the
	 * merged structure carries direct pointers -- the interleave's GT
	 * continuation backtracks via parent pointers, which are flip proxies
	 * until settled.  Collided src cells were demoted to duplicates by
	 * apply_splices (step 5), so the merged-region walk never sees them;
	 * dst-original cells keep their links.  The edge + flip batches were
	 * pre-allocated in the build phase, so this cannot fail.
	 *
	 * 10. Only now free the collided cells: the interleave rewired the
	 * surviving cells' stale src-run links away from them, and the
	 * grace-period defer inside the free covers readers already holding
	 * such a link.
	 */
	if (ms_ord) {
		ft_merge_ord_interleave(dst_ft, dst_key, dst_key_len, merged_keys,
			ms_cursor, ms_prev, ms_edges, ms_flip);
		free(ms_edges);
	}
	ft_graft_glue_free_collided_cells(dst_ft, &gd);

	ft_graft_glue_fini(&gd);
	ft_graft_glue_fini(&gs);
	return CDS_FT_STATUS_OK;
}

enum cds_ft_status cds_ft_merge_at(struct cds_ft *dst_ft,
		const uint8_t *dst_key, size_t dst_key_len,
		struct cds_ft *src_ft,
		const uint8_t *src_key, size_t src_key_len)
{
	struct cds_ft *subtree = NULL;
	enum cds_ft_status status;
	struct ft_descent d_src, d_dst;
	unsigned int off_src, off_dst;
	unsigned long cnt_src, cnt_dst;
	enum ft_graft_swap_case ks, kd;
	uint8_t okey_dst_buf[FT_MAX_KEY_LEN], okey_src_buf[FT_MAX_KEY_LEN];
	const uint8_t *okey_dst = dst_key, *okey_src = src_key;

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
	/*
	 * Combined-length overflow validation (mirrors cds_ft_graft): a moved
	 * key K becomes dst_key || (K - src_key prefix), of length
	 * dst_key_len + len(K) - src_key_len.  Bound len(K) by the source's
	 * max_used_key_len; without this check a variable-length merge with
	 * dst_key_len > src_key_len could create keys exceeding the group's
	 * max_key_len, overflowing the fixed-size key buffers downstream
	 * (the spine's compressed-wrap kbuf, the iterator buffers).
	 */
	{
		size_t src_max = uatomic_load(&src_ft->max_used_key_len,
				CMM_RELAXED);

		if (src_max > src_key_len &&
				src_max - src_key_len >
				dst_ft->group->max_key_len - dst_key_len) {
			FT_TP(merge_exit, (int) CDS_FT_STATUS_OVERFLOW_ERROR);
			return CDS_FT_STATUS_OVERFLOW_ERROR;
		}
	}

	/*
	 * merge_at is a mutator; the application provides mutual exclusion
	 * between mutators.  Take the reentrant writer-validation scope (like
	 * cds_ft_graft_swap), NOT a flavor read lock -- the spine-copy commit
	 * calls update_synchronize_rcu, which would deadlock inside a
	 * read-side critical section.
	 */
	CDS_FT_SCOPED_WRITER(dst_ft);
	CDS_FT_SCOPED_WRITER(src_ft);

	/*
	 * Convert both keys to ORDINAL form ONCE; every internal consumer
	 * (the merge-point descents, the spine build, the source-subtree
	 * unlink, the ordered-list interleave) takes the ordinal form.  The
	 * detach+graft fallback below instead receives the ORIGINAL
	 * application keys -- those entry points remap internally.
	 * Previously the descents consumed the application bytes raw while
	 * the unlink remapped: on a non-identity key map the build copied one
	 * subtree and the unlink targeted another.
	 */
	{
		const struct cds_ft_key_map *km = &dst_ft->group->key_map;

		if (caa_unlikely(!km->identity)) {
			ft_key_to_ordinals(okey_dst_buf, dst_key, dst_key_len,
				km);
			ft_key_to_ordinals(okey_src_buf, src_key, src_key_len,
				km);
			okey_dst = okey_dst_buf;
			okey_src = okey_src_buf;
		}
	}

	/*
	 * Locate both merge points read-only (a writer descends its own
	 * stable state).  No content under @src_key -> the merge is a no-op;
	 * cnt_src == 0 covers an empty @src_ft root (src_key_len == 0, where
	 * the descent still reports EXACT at the always-present root).
	 */
	ks = ft_merge_descend(src_ft, okey_src, src_key_len, &d_src,
			&off_src, &cnt_src);
	if (ks == FT_GRAFT_SWAP_DELEGATE || cnt_src == 0) {
		FT_TP(merge_exit, (int) CDS_FT_STATUS_OK);
		return CDS_FT_STATUS_OK;
	}
	kd = ft_merge_descend(dst_ft, okey_dst, dst_key_len, &d_dst,
			&off_dst, &cnt_dst);

	/*
	 * Atomic build-invisible spine-copy of @src_ft's subtree at @src_key into
	 * @dst_ft's non-empty subtree at @dst_key.  Each side is EXACT (@off == 0,
	 * the subtree at @d->nf) or KEY_SHORTER (@off > 0, the key ends inside a
	 * compressed node -- the src node is reclaimed by the commit's unlink, the
	 * dst node is wrapped under its prefix).  ft_merge_spine_copy handles every
	 * such shape (internal, external, compressed and compressed-parent dst
	 * merge points); only a dst that is empty under @dst_key falls through to
	 * the detach-based graft below.
	 */
	if ((ks == FT_GRAFT_SWAP_EXACT || ks == FT_GRAFT_SWAP_KEY_SHORTER)
			&& cnt_dst > 0
			&& (kd == FT_GRAFT_SWAP_EXACT
				|| kd == FT_GRAFT_SWAP_KEY_SHORTER)) {
		status = ft_merge_spine_copy(dst_ft, src_ft, &d_src,
				okey_src, src_key_len, cnt_src, off_src,
				&d_dst, cnt_dst, off_dst, okey_dst, dst_key_len);
		if (status == CDS_FT_STATUS_OK) {
			/*
			 * Raise dst's max_used_key_len for the moved keys
			 * (dst_key_len + the longest stripped suffix), as the
			 * graft paths do -- later graft overflow validations
			 * feed off it.
			 */
			size_t src_max = uatomic_load(&src_ft->max_used_key_len,
					CMM_RELAXED);
			size_t nm = src_max > src_key_len ?
				dst_key_len + (src_max - src_key_len) :
				dst_key_len;
			size_t dm = uatomic_load(&dst_ft->max_used_key_len,
					CMM_RELAXED);

			if (nm > dm)
				uatomic_store(&dst_ft->max_used_key_len, nm,
					CMM_RELAXED);
		}
		FT_TP(merge_exit, (int) status);
		return status;
	}

	/*
	 * Detach-based graft: @dst has no subtree at @dst_key here -- either
	 * cnt_dst == 0 (an empty @dst root) or the descent diverged / dead-ended
	 * (kd == FT_GRAFT_SWAP_DELEGATE), both of which mean no key has @dst_key as
	 * a prefix.  Move @src_ft@src_key into a transient exclusive @subtree and
	 * graft it atomically at @dst_key (ft_graft is itself a build-invisible
	 * transaction).  Every NON-empty-dst shape committed in ft_merge_spine_copy
	 * above, so the old non-atomic per-entry merge loop is gone (a non-empty
	 * graft point would return POPULATED_ERROR and roll back, never corrupt).
	 * @subtree's keys are stripped of @src_key, so it is touched only via
	 * keylen-bypassing helpers.
	 */
	status = ft_detach_keylen(src_ft, src_key, src_key_len, &subtree);
	if (status < 0)
		goto out;	/* NOT_FOUND impossible: @src_ft had content. */
	status = ft_graft_keylen(dst_ft, dst_key, dst_key_len, subtree);
	if (status != CDS_FT_STATUS_OK)
		goto out_rollback;

	cds_ft_destroy(subtree);
	FT_TP(merge_exit, (int) CDS_FT_STATUS_OK);
	return CDS_FT_STATUS_OK;

out_rollback:
	/*
	 * Best-effort rollback: re-graft @subtree back into @src_ft at
	 * @src_key.  @src_ft was emptied under @src_key by the detach, so
	 * this graft cannot collide and only fails on memory exhaustion (on
	 * which @subtree's externals are leaked).  The original error is
	 * propagated.
	 */
	(void) ft_graft_keylen(src_ft, src_key, src_key_len, subtree);
	cds_ft_destroy(subtree);
out:
	FT_TP(merge_exit, (int) status);
	return status;
}

enum cds_ft_status cds_ft_merge(struct cds_ft *dst_ft,
		const uint8_t *key, size_t key_len,
		struct cds_ft *src_ft)
{
	return cds_ft_merge_at(dst_ft, key, key_len, src_ft, key, key_len);
}

size_t cds_ft_group_key_len(const struct cds_ft_group *group)
{
	return group->key_len;
}

size_t cds_ft_group_max_key_len(const struct cds_ft_group *group)
{
	return group->max_key_len;
}

size_t cds_ft_max_used_key_len(const struct cds_ft *ft)
{
	return uatomic_load(&ft->max_used_key_len, CMM_RELAXED);
}

enum cds_ft_status cds_ft_group_key_map(const struct cds_ft_group *group, uint8_t *key_to_ordinal, uint8_t *ordinal_to_key)
{
	if (group->key_map.identity)
		return CDS_FT_STATUS_NOT_FOUND;
	memcpy(key_to_ordinal, group->key_map.key_to_ordinal, sizeof(group->key_map.key_to_ordinal));
	memcpy(ordinal_to_key, group->key_map.ordinal_to_key, sizeof(group->key_map.ordinal_to_key));
	return CDS_FT_STATUS_OK;
}

bool cds_ft_empty(struct cds_ft *ft)
{
	struct cds_ft_inode_flag *root_flag;
	struct cds_ft_inode *root_node;
	struct cds_ft_metadata *rmeta;

	CDS_FT_SCOPED_READER(ft);
	CDS_FT_ASSERT_RCU_READ_LOCKED(ft);

	root_flag = ft_root_dereference(ft);
	root_node = ft_node_ptr(root_flag);

	/*
	 * The root is always an internal node (invariant enforced by
	 * ft_make_root_internal_glue at every site that publishes
	 * ft->root), so no tag dispatch is needed before reading metadata.
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

	node_flag = ft_root_dereference_acquire_prefetch(ft);

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
		{
			unsigned int rewind;

			/*
			 * Surgical re-anchor on a skip-compressed mismatch (no
			 * spin on a frozen slot).  rewind > 0 means a concurrent
			 * chain-merge moved the encoded position shallower; the
			 * count is over a fixed prefix, so just re-descend from
			 * the root (idempotent, no rank to undercount).
			 */
			node_flag = ft_node_get_nth_reanchor(ft, node_flag, kv,
					&rewind);
			if (caa_unlikely(rewind)) {
				node_flag = ft_root_dereference_acquire_prefetch(ft);
				i = (unsigned int) -1;	/* loop ++ -> restart at 0 */
				continue;
			}
		}
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
		int *level_p, uint8_t *ordinal_key)
{
	struct cds_ft_inode_flag *node_flag = *node_flag_p;
	int level = *level_p;
	struct cds_ft_compressed_node *cn =
		ft_compressed_node_ptr(node_flag);

	ft_fill_compressed_path(cn, ordinal_key, level - 1);
	level += cn->len - 1;
	node_flag = ft_dereference_acquire_prefetch(cn->child);
	assert(node_flag != NULL);	/* compressed node always has a live child (by construction) */
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

	node_flag = ft_root_dereference_acquire_prefetch(ft);

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
				iter->key_off = 0;
				{
					int j;

					for (j = 0; j < level - 1; j++)
						iter_key(iter)[j] = ordinal_key[j];
				}
				iter->node = ext;
				iter->cache_valid = true;
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
				&level, ordinal_key);
			if (act == FT_DESCENT_BREAK)
				break;
			continue;
		}
		/* Iterate children in ascending ordinal order. */
		pivot = -1;
		child = ft_node_get_direction(ft, node_flag, pivot, &child_key, FT_RIGHT, true);
		while (child) {
			unsigned long child_keys;

#ifdef FEATURE_FT_SKIP_COMPRESSED
			/*
			 * Skip-validate failure (a concurrent split/merge reparented
			 * the skip child, possibly while @node_flag was recompacted
			 * away): re-anchor on the live structure via the skip child's
			 * parent chain — the same single mechanism the precise /
			 * inequality readers use, never a spin or a frozen re-read.
			 *   rewind == 0: @node_flag was recompacted in place; re-scan its
			 *     live version from the same pivot (rank accumulation
			 *     unchanged — we only re-scan children past @pivot).
			 *   rewind > 0: a transient single-child @node_flag (pivot == -1,
			 *     nothing accumulated for it yet) merged into a longer
			 *     compressed; descend INTO that merged node (re-scanning the
			 *     shallower holder would double-count).  level -= rewind + 1
			 *     so the loop's level++ lands the compressed handler at the
			 *     merged node's depth; remaining is unchanged.
			 *   detached / above root: re-find rank @n on the live trie.
			 */
			if (caa_unlikely(ft_node_skip_compressed(child))) {
				unsigned int rewind;
				struct cds_ft_inode_flag *merged;
				struct cds_ft_inode_flag *anchor;

				/*
				 * Surgical re-anchor: resolve @child via the live skip
				 * child's parent chain and continue forward — never
				 * re-scan a slot or restart from the root.  Root re-descent
				 * would re-count a concurrently-growing trie and undercount
				 * the rank.  ft_skip_reanchor never returns NULL on a
				 * well-formed trie (the writer wires every fresh cluster's
				 * parent before the cluster becomes reachable).
				 *   rewind == 0: @merged is the resolved live child at
				 *     @child_key; use it and fall through to the rank
				 *     accumulation (which only advances).
				 *   rewind > 0:  @node_flag (a transient single child) was
				 *     absorbed into a longer compressed; descend INTO it.
				 */
				anchor = ft_skip_reanchor(ft, child, &rewind, &merged);
				assert(anchor != NULL);
				if (caa_likely(rewind == 0)) {
					node_flag = anchor;
					child = merged;
				} else {
					node_flag = merged;
					level -= (int) rewind + 1;
					goto next_level;
				}
			}
#endif
			child_keys = ft_child_key_count(child);

			if (remaining < child_keys) {
				/* Target is in this child's subtree. Descend. */
				ordinal_key[level - 1] = child_key;
				node_flag = child;
				goto next_level;
			}
			remaining -= child_keys;
			pivot = child_key;
			child = ft_node_get_direction(ft, node_flag, pivot, &child_key, FT_RIGHT, true);
		}

		/* Exhausted all children without finding. */
		break;

next_level:
		;
	}

	/* Reached a leaf (external node). */
	if (node_flag && ft_node_external(node_flag) && remaining == 0) {
		iter->key_len = level - 1;
		iter->key_off = 0;
		{
			int j;

			for (j = 0; j < level - 1; j++)
				iter_key(iter)[j] = ordinal_key[j];
		}
		iter->node = (struct cds_ft_node *) ft_node_ptr(node_flag);
		iter->cache_valid = true;
		iter_debug_path_update(iter);
		iter->path_len = level;
		iter->status = CDS_FT_STATUS_OK;
		goto end;
	}

	iter->node = NULL;
	iter->cache_valid = true;
	iter_debug_path_update(iter);
	iter->path_len = 0;
	iter->status = CDS_FT_STATUS_NOT_FOUND;

end:
	iter_auto_invalidate_cache(iter);
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
		uint8_t *ordinal_key)
{
	struct cds_ft_inode_flag *node_flag = *node_flag_p;
	int level = *level_p;
	struct cds_ft_compressed_node *cn =
		ft_compressed_node_ptr(node_flag);

	unsigned long child_keys;

	assert(cn->child != NULL);	/* compressed node always has a live child */
	child_keys = ft_child_key_count(cn->child);

	if (*remaining_p < child_keys) {
		ft_fill_compressed_path(cn,
			ordinal_key, level - 1);
		level += cn->len - 1;
		node_flag = ft_dereference_acquire_prefetch(cn->child);
		assert(node_flag != NULL);	/* compressed node always has a live child */
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

	node_flag = ft_root_dereference_acquire_prefetch(ft);

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
				ordinal_key);
			if (act == FT_DESCENT_BREAK)
				break;
			if (act == FT_DESCENT_END)
				goto check_ext_nth_last;
			continue;
		}
		/* Iterate children in descending ordinal order first. */
		pivot = FT_ENTRY_PER_NODE;
		child = ft_node_get_direction(ft, node_flag, pivot, &child_key, FT_LEFT, true);
		while (child) {
			unsigned long child_keys;

#ifdef FEATURE_FT_SKIP_COMPRESSED
			/* Skip-validate failure: re-anchor on the live structure
			 * (see cds_ft_lookup_nth — same single mechanism). */
			if (caa_unlikely(ft_node_skip_compressed(child))) {
				unsigned int rewind;
				struct cds_ft_inode_flag *merged;
				struct cds_ft_inode_flag *anchor;

				/*
				 * Surgical re-anchor (see cds_ft_lookup_nth): resolve
				 * @child via the live skip child's parent chain and
				 * continue forward — never re-scan or restart from root.
				 * ft_skip_reanchor never returns NULL on a well-formed
				 * trie (writer wires parents before publishing).
				 */
				anchor = ft_skip_reanchor(ft, child, &rewind, &merged);
				assert(anchor != NULL);
				if (caa_likely(rewind == 0)) {
					node_flag = anchor;
					child = merged;
				} else {
					node_flag = merged;
					level -= (int) rewind + 1;
					goto next_level;
				}
			}
#endif
			child_keys = ft_child_key_count(child);

			if (remaining < child_keys) {
				/* Target is in this child's subtree. Descend. */
				ordinal_key[level - 1] = child_key;
				node_flag = child;
				goto next_level;
			}
			remaining -= child_keys;
			pivot = child_key;
			child = ft_node_get_direction(ft, node_flag, pivot, &child_key, FT_LEFT, true);
		}

check_ext_nth_last:
		/* External_nodes at this depth are the smallest (last in reverse). */
		ext = ft_dereference_acquire(metadata->external_nodes);
		if (ext) {
			if (remaining == 0) {
				iter->key_len = level - 1;
				iter->key_off = 0;
				{
					int j;

					for (j = 0; j < level - 1; j++)
						iter_key(iter)[j] = ordinal_key[j];
				}
				iter->node = ext;
				iter->cache_valid = true;
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
		iter->key_off = 0;
		{
			int j;

			for (j = 0; j < level - 1; j++)
				iter_key(iter)[j] = ordinal_key[j];
		}
		iter->node = (struct cds_ft_node *) ft_node_ptr(node_flag);
		iter->cache_valid = true;
		iter_debug_path_update(iter);
		iter->path_len = level;
		iter->status = CDS_FT_STATUS_OK;
		goto end;
	}

	iter->node = NULL;
	iter->cache_valid = true;
	iter_debug_path_update(iter);
	iter->path_len = 0;
	iter->status = CDS_FT_STATUS_NOT_FOUND;

end:
	iter_auto_invalidate_cache(iter);
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
		uint8_t *ordinal_key)
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
	}
	i += cn->len - 1;
	node_flag = ft_dereference_acquire_prefetch(cn->child);
	assert(node_flag != NULL);	/* compressed node always has a live child (by construction) */
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
		const uint8_t *key, size_t key_len,
		uint8_t *ordinal_key,
		struct cds_ft_inode_flag **deepest_p)
{
	struct cds_ft_inode_flag *node_flag;
	unsigned int i;

	node_flag = ft_root_dereference_acquire_prefetch(ft);

	for (i = 0; i < key_len; i++) {
		uint8_t ordinal;

		if (ft_node_external(node_flag))
			return -1;

		if (ft_node_compressed(node_flag)) {
			enum ft_descent_action act;

			act = ft_rebuild_path_compressed(&node_flag, &i,
				key, key_len, ordinal_key);
			if (act == FT_DESCENT_END)
				return -1;
			assert(act == FT_DESCENT_CONTINUE);
			continue;
		}
		ordinal = key[i];
		ordinal_key[i] = ordinal;
		{
			unsigned int rewind;

			/*
			 * Surgical re-anchor (no frozen-slot spin).  rewind > 0:
			 * a concurrent chain-merge moved the position shallower;
			 * rebuild this fixed path from the root (idempotent).
			 */
			node_flag = ft_node_get_nth_reanchor(ft, node_flag, ordinal,
					&rewind);
			if (caa_unlikely(rewind)) {
				node_flag = ft_root_dereference_acquire_prefetch(ft);
				i = (unsigned int) -1;	/* loop ++ -> restart at 0 */
				continue;
			}
		}
		if (!node_flag)
			return -1;
	}
	/*
	 * Deepest node reached (the node at depth key_len); always placed at
	 * depth key_len as a child, so its shallow boundary is key_len.
	 * Lets the going-up cursor seed live from the descent.
	 */
	*deepest_p = node_flag;
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
		int *level_p, uint8_t *ordinal_key)
{
	struct cds_ft_inode_flag *node_flag = *node_flag_p;
	int level = *level_p;
	struct cds_ft_compressed_node *cn =
		ft_compressed_node_ptr(node_flag);
	int j;

	for (j = 0; j < cn->len; j++) {
		ordinal_key[level + j] = cn->key_bytes[j];
	}
	level += cn->len;
	node_flag = ft_dereference_acquire_prefetch(cn->child);
	assert(node_flag != NULL);	/* compressed node always has a live child (by construction) */
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
	struct cds_ft_inode_flag *deepest = NULL;
	struct cds_ft_inode_flag *descend_from = NULL;

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
	depth = ft_rebuild_path(ft, ft_iter_read_key(iter), ft_iter_resolve_key_len(iter),
			ordinal_key, &deepest);
	if (depth < 0)
		goto not_found;
	(void) deepest;		/* used by the going-up seed under PP backtrack */

	remaining = n;

	/*
	 * Determine whether the current key sits at an internal or
	 * compressed node's external_nodes (variable-length prefix key)
	 * or at a leaf child.
	 */
	at_external_nodes = !ft_node_external(deepest);

	/*
	 * If at external_nodes of an internal/compressed node, all
	 * children of that node are to the right.  Try to satisfy the
	 * skip within them.
	 */
	if (at_external_nodes) {
		struct cds_ft_inode_flag *parent = deepest;
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

				assert(cn->child != NULL);	/* compressed node always has a live child */
				ck = ft_child_key_count(cn->child);
				if (remaining < ck) {
					for (j = 0; j < cn->len; j++) {
						ordinal_key[depth + j] =
							cn->key_bytes[j];
					}
					level = depth + cn->len;
					descend_from = cn->child;
					goto descend_forward;
				}
				remaining -= ck;
			} else {
				struct cds_ft_inode_flag *child;
				uint8_t child_key = 0;
				int pivot = -1;

				child = ft_node_get_direction(ft, parent, pivot,
						&child_key, FT_RIGHT, true);
				while (child) {
					unsigned long ck;

#ifdef FEATURE_FT_SKIP_COMPRESSED
					/*
					 * Surgical re-anchor: @parent carries external_nodes so
					 * it cannot merge — rewind is always 0 (recompacted in
					 * place).  @at_pos is the live resolved sibling at
					 * @child_key; use it directly.  ft_skip_reanchor never
					 * returns NULL on a well-formed trie.
					 */
					if (caa_unlikely(ft_node_skip_compressed(child))) {
						unsigned int rewind;
						struct cds_ft_inode_flag *at_pos;
						struct cds_ft_inode_flag *anchor =
							ft_skip_reanchor(ft, child, &rewind, &at_pos);

						assert(anchor != NULL);
						assert(rewind == 0);
						parent = anchor;
						child = at_pos;
						continue;
					}
#endif
					ck = ft_child_key_count(child);
					if (remaining < ck) {
						ordinal_key[depth] = child_key;
						level = depth + 1;
						descend_from = child;
						goto descend_forward;
					}
					remaining -= ck;
					pivot = child_key;
					child = ft_node_get_direction(ft, parent, pivot,
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

	/*
	 * Walk up: at each ancestor, count keys in rightward siblings
	 * of the child we came from.  Parent-pointer backtrack:
	 * @up_node is the live node covering depth @level, climbed via
	 * ft_get_parent_rcu as @level decrements (span compressed=cn->len,
	 * internal=1).  Seeded live from ft_rebuild_path's deepest node
	 * (@deepest == node at @level==@depth, placed there as a child so its
	 * shallow boundary is @level).
	 */
	{	/* Scope the going-up cursor locals so they are out of scope at not_found/descend_* (avoids -Wjump-misses-init false positives). */
		struct cds_ft_inode_flag *up_node = deepest;
		ssize_t up_node_lo = level;
		for (level--; level >= 0; level--) {
			struct cds_ft_inode_flag *ancestor;
			struct cds_ft_inode_flag *child;
			uint8_t child_key = 0;
			int pivot;

			while (level < up_node_lo) {
				struct cds_ft_inode_flag *gp = ft_get_parent_rcu(ft, up_node);

				up_node_lo -= (gp && ft_node_compressed(gp)) ?
					(ssize_t) ft_compressed_node_ptr(gp)->len : 1;
				up_node = gp;
			}
			ancestor = up_node;
			if (ft_node_external(ancestor))
				continue;
			/*
			 * Compressed path levels have no siblings: skip.
			 */
			if (ft_node_compressed(ancestor))
				continue;

			pivot = ordinal_key[level];
			child = ft_node_get_direction(ft, ancestor, pivot,
					&child_key, FT_RIGHT, true);
			while (child) {
				unsigned long ck;

	#ifdef FEATURE_FT_SKIP_COMPRESSED
				/*
				 * Surgical re-anchor: @ancestor has siblings to scan, so it is
				 * multi-child and cannot merge — rewind is 0.  @at_pos is the
				 * live resolved sibling at @child_key; use it directly.
				 * ft_skip_reanchor never returns NULL on a well-formed trie.
				 */
				if (caa_unlikely(ft_node_skip_compressed(child))) {
					unsigned int rewind;
					struct cds_ft_inode_flag *at_pos;
					struct cds_ft_inode_flag *anchor =
						ft_skip_reanchor(ft, child, &rewind, &at_pos);

					assert(anchor != NULL);
					assert(rewind == 0);
					ancestor = anchor;
					up_node = anchor;	/* live holder at @level (internal, span 1) */
					up_node_lo = level;
					child = at_pos;
					continue;
				}
	#endif
				ck = ft_child_key_count(child);
				if (remaining <= ck) {
					remaining--;  /* enter this subtree (1-indexed within) */
					ordinal_key[level] = child_key;
					level = level + 1;
					descend_from = child;
					goto descend_forward;
				}
				remaining -= ck;
				pivot = child_key;
				child = ft_node_get_direction(ft, ancestor, pivot,
						&child_key, FT_RIGHT, true);
			}
			/*
			 * No external_nodes to count going up in forward direction
			 * (they sort before children, so they're behind us).
			 */
		}
	}

	/* Exhausted the trie. */
not_found:
	iter->node = NULL;
	iter->cache_valid = true;
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
		/*
		 * @descend_from is the node placed at @level by the going-up
		 * / at-external block right before its goto here; it is the
		 * node at descent level @level.
		 */
		struct cds_ft_inode_flag *node_flag = descend_from;

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
					iter->key_off = 0;
					for (j = 0; j < level; j++)
						iter_key(iter)[j] = ordinal_key[j];
					iter->node = ext;
					iter->cache_valid = true;
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
					ordinal_key);
				if (act == FT_DESCENT_BREAK)
					break;
				continue;
			}
			pivot = -1;
			child = ft_node_get_direction(ft, node_flag, pivot,
					&child_key, FT_RIGHT, true);
			while (child) {
				unsigned long ck;

#ifdef FEATURE_FT_SKIP_COMPRESSED
				/*
				 * Surgical re-anchor (descend phase; see cds_ft_lookup_nth).
				 *   rewind == 0: @merged is the live resolved child at
				 *     @child_key; use it directly.
				 *   rewind > 0: a transient single-child @node_flag
				 *     (pivot == -1, nothing accumulated) merged into a
				 *     longer compressed; descend INTO it.  level -= rewind
				 *     lands the compressed handler at the merged node's
				 *     depth (ft_skip_forward_compressed fills from level);
				 *     remaining unchanged.
				 * ft_skip_reanchor never returns NULL on a well-formed trie.
				 */
				if (caa_unlikely(ft_node_skip_compressed(child))) {
					unsigned int rewind;
					struct cds_ft_inode_flag *merged;
					struct cds_ft_inode_flag *anchor =
						ft_skip_reanchor(ft, child, &rewind, &merged);

					assert(anchor != NULL);
					if (caa_likely(rewind == 0)) {
						node_flag = anchor;
						child = merged;
						continue;
					}
					node_flag = merged;
					level -= (int) rewind;
					goto next_forward_level;
				}
#endif
				ck = ft_child_key_count(child);
				if (remaining < ck) {
					ordinal_key[level] = child_key;
					level++;
					node_flag = child;
					goto next_forward_level;
				}
				remaining -= ck;
				pivot = child_key;
				child = ft_node_get_direction(ft, node_flag, pivot,
						&child_key, FT_RIGHT, true);
			}
			break;

next_forward_level:
			;
		}

		/* Reached a leaf. */
		if (ft_node_ptr(node_flag) &&
		    ft_node_external(node_flag) &&
		    remaining == 0) {
			int j;

			iter->key_len = level;
			iter->key_off = 0;
			for (j = 0; j < level; j++)
				iter_key(iter)[j] = ordinal_key[j];
			iter->node = (struct cds_ft_node *)
				ft_node_ptr(node_flag);
			iter->cache_valid = true;
			iter_debug_path_update(iter);
			iter->path_len = level + 1;
			iter->status = CDS_FT_STATUS_OK;
			goto end;
		}
		goto not_found;
	}

end:
	iter_auto_invalidate_cache(iter);
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
		uint8_t *ordinal_key)
{
	struct cds_ft_inode_flag *node_flag = *node_flag_p;
	int level = *level_p;
	struct cds_ft_compressed_node *cn =
		ft_compressed_node_ptr(node_flag);

	unsigned long ck;

	assert(cn->child != NULL);	/* compressed node always has a live child */
	ck = ft_child_key_count(cn->child);

	if (*remaining_p < ck) {
		int j;

		for (j = 0; j < cn->len; j++) {
			ordinal_key[level + j] = cn->key_bytes[j];
		}
		level += cn->len;
		node_flag = ft_dereference_acquire_prefetch(cn->child);
		assert(node_flag != NULL);	/* compressed node always has a live child */
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

	/*
	 * Caller (cds_ft_iter_skip_reverse) guarantees this is the
	 * compressed ancestor's shallow boundary -- the intermediate-level
	 * skip is decided there, so this code need not re-derive it.
	 */
	ameta = cds_ft_item_to_metadata(ft_node_ptr(ancestor));
	a_ext = ft_dereference_acquire(ameta->external_nodes);
	if (a_ext) {
		if (*remaining_p == 1) {
			int j;

			iter->key_len = level;
			iter->key_off = 0;
			for (j = 0; j < level; j++)
				iter_key(iter)[j] = ordinal_key[j];
			iter->node = a_ext;
			iter->cache_valid = true;
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
	struct cds_ft_inode_flag *deepest = NULL;
	struct cds_ft_inode_flag *descend_from = NULL;

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
	depth = ft_rebuild_path(ft, ft_iter_read_key(iter), ft_iter_resolve_key_len(iter),
			ordinal_key, &deepest);
	if (depth < 0)
		goto not_found;
	(void) deepest;		/* used by the going-up seed under PP backtrack */

	remaining = n;

	/*
	 * Determine whether the current key sits at an internal or
	 * compressed node's external_nodes or at a leaf child.
	 */
	at_external_nodes = !ft_node_external(deepest);
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
	 * (which sort before all children).  Parent-pointer backtrack:
	 * @up_node is the live node covering depth @level, climbed via
	 * ft_get_parent_rcu as @level decrements.  Seeded live from
	 * ft_rebuild_path's deepest node (@deepest == node at @level==@depth,
	 * placed there as a child so its shallow boundary is @level).
	 */
	{	/* Scope the going-up cursor locals so they are out of scope at not_found/descend_* (avoids -Wjump-misses-init false positives). */
		struct cds_ft_inode_flag *up_node = deepest;
		ssize_t up_node_lo = level;
		for (level--; level >= 0; level--) {
			struct cds_ft_inode_flag *ancestor;
			struct cds_ft_inode_flag *child;
			struct cds_ft_metadata *ameta;
			uint8_t child_key = 0;
			unsigned long left_keys = 0;
			int pivot;

			while (level < up_node_lo) {
				struct cds_ft_inode_flag *gp = ft_get_parent_rcu(ft, up_node);

				up_node_lo -= (gp && ft_node_compressed(gp)) ?
					(ssize_t) ft_compressed_node_ptr(gp)->len : 1;
				up_node = gp;
			}
			ancestor = up_node;
			if (ft_node_external(ancestor))
				continue;
			/*
			 * Skip intermediate compressed path levels (same
			 * compressed node at adjacent levels).  At the entry
			 * level, only external_nodes matter (no siblings).
			 */
			if (ft_node_compressed(ancestor)) {
				enum ft_descent_action act;
				bool at_shallow_boundary;

				/*
				 * Process the compressed ancestor's external_nodes only
				 * at its shallow boundary; skip the intermediate in-span
				 * levels.  The live cursor's shallow bound up_node_lo
				 * equals level there.
				 */
				at_shallow_boundary = (level == 0) ||
					(up_node_lo == level);
				if (!at_shallow_boundary)
					continue;
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
			child = ft_node_get_direction(ft, ancestor, pivot,
					&child_key, FT_LEFT, true);
			while (child) {
	#ifdef FEATURE_FT_SKIP_COMPRESSED
				/*
				 * Surgical re-anchor: @ancestor has siblings to count ->
				 * multi-child, cannot merge -> rewind 0.  @at_pos is the live
				 * resolved sibling at @child_key; use it directly.
				 * ft_skip_reanchor never returns NULL on a well-formed trie.
				 */
				if (caa_unlikely(ft_node_skip_compressed(child))) {
					unsigned int rewind;
					struct cds_ft_inode_flag *at_pos;
					struct cds_ft_inode_flag *anchor =
						ft_skip_reanchor(ft, child, &rewind, &at_pos);

					assert(anchor != NULL);
					assert(rewind == 0);
					ancestor = anchor;
					up_node = anchor;
					up_node_lo = level;
					child = at_pos;
					continue;
				}
	#endif
				left_keys += ft_child_key_count(child);
				pivot = child_key;
				child = ft_node_get_direction(ft, ancestor, pivot,
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
					child = ft_node_get_direction(ft, ancestor, pivot,
							&child_key, FT_LEFT, true);
					while (child) {
						unsigned long ck;

	#ifdef FEATURE_FT_SKIP_COMPRESSED
						/*
						 * Surgical re-anchor: @ancestor is the multi-child node
						 * whose leftward siblings we iterate -> cannot merge ->
						 * rewind 0.  @at_pos is the live resolved sibling at
						 * @child_key; use it directly.  ft_skip_reanchor never
						 * returns NULL on a well-formed trie.
						 */
						if (caa_unlikely(ft_node_skip_compressed(child))) {
							unsigned int rewind;
							struct cds_ft_inode_flag *at_pos;
							struct cds_ft_inode_flag *anchor =
								ft_skip_reanchor(ft, child, &rewind, &at_pos);

							assert(anchor != NULL);
							assert(rewind == 0);
							ancestor = anchor;
							up_node = anchor;
							up_node_lo = level;
							child = at_pos;
							continue;
						}
	#endif
						ck = ft_child_key_count(child);
						if (remaining <= ck) {
							remaining--;
							ordinal_key[level] = child_key;
							level = level + 1;
							descend_from = child;
							goto descend_reverse;
						}
						remaining -= ck;
						pivot = child_key;
						child = ft_node_get_direction(ft, 
								ancestor, pivot,
								&child_key, FT_LEFT, true);
					}

					/* Must be the external_nodes. */
					if (a_ext && remaining == 1) {
						int j;

						iter->key_len = level;
						iter->key_off = 0;
						for (j = 0; j < level; j++)
							iter_key(iter)[j] = ordinal_key[j];
						iter->node = a_ext;
					iter->cache_valid = true;
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
	}

	/* Exhausted the trie. */
not_found:
	iter->node = NULL;
	iter->cache_valid = true;
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
		/*
		 * @descend_from carries the node placed at @level by the
		 * going-up block right before its goto here; it is the
		 * node at descent level @level.
		 */
		struct cds_ft_inode_flag *node_flag = descend_from;

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
					ordinal_key);
				if (act == FT_DESCENT_BREAK)
					break;
				if (act == FT_DESCENT_END)
					goto check_ext_descend_reverse;
				continue;
			}
			pivot = FT_ENTRY_PER_NODE;
			child = ft_node_get_direction(ft, node_flag, pivot,
					&child_key, FT_LEFT, true);
			while (child) {
				unsigned long ck;

#ifdef FEATURE_FT_SKIP_COMPRESSED
				/*
				 * Surgical re-anchor (descend phase; see cds_ft_lookup_nth).
				 *   rewind == 0: @merged is the live resolved child at
				 *     @child_key; use it directly.
				 *   rewind > 0: a transient single-child @node_flag merged
				 *     into a longer compressed; descend INTO it (level -=
				 *     rewind; remaining unchanged).
				 * ft_skip_reanchor never returns NULL on a well-formed trie.
				 */
				if (caa_unlikely(ft_node_skip_compressed(child))) {
					unsigned int rewind;
					struct cds_ft_inode_flag *merged;
					struct cds_ft_inode_flag *anchor =
						ft_skip_reanchor(ft, child, &rewind, &merged);

					assert(anchor != NULL);
					if (caa_likely(rewind == 0)) {
						node_flag = anchor;
						child = merged;
						continue;
					}
					node_flag = merged;
					level -= (int) rewind;
					goto next_reverse_level;
				}
#endif
				ck = ft_child_key_count(child);
				if (remaining < ck) {
					ordinal_key[level] = child_key;
					level++;
					node_flag = child;
					goto next_reverse_level;
				}
				remaining -= ck;
				pivot = child_key;
				child = ft_node_get_direction(ft, node_flag, pivot,
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
				iter->key_off = 0;
				for (j = 0; j < level; j++)
					iter_key(iter)[j] = ordinal_key[j];
				iter->node = ext;
				iter->cache_valid = true;
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
		if (ft_node_ptr(node_flag) &&
		    ft_node_external(node_flag) &&
		    remaining == 0) {
			int j;

			iter->key_len = level;
			iter->key_off = 0;
			for (j = 0; j < level; j++)
				iter_key(iter)[j] = ordinal_key[j];
			iter->node = (struct cds_ft_node *)
				ft_node_ptr(node_flag);
			iter->cache_valid = true;
			iter_debug_path_update(iter);
			iter->path_len = level + 1;
			iter->status = CDS_FT_STATUS_OK;
			goto end;
		}
		goto not_found;
	}

end:
	iter_auto_invalidate_cache(iter);
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

/*
 * Validate that the pointer bits used by the skip-compressed encoding
 * are outside the process's virtual address range.  Attempt to mmap a
 * page at the encoding boundary; if the mapping succeeds (or fails with
 * EEXIST) the bit is within the VA range and skip-compressed cannot be
 * used safely.  Only ENOMEM (address beyond TASK_SIZE) confirms the bit
 * is available, and only after a positive control rules out a spurious
 * ENOMEM from a global mmap failure (see below); any other failure
 * (EPERM, EINVAL, EAGAIN, seccomp, ...) is treated conservatively as
 * unavailable.
 */
#ifdef FEATURE_FT_SKIP_COMPRESSED
static
bool ft_skip_compressed_validate(void)
{
	size_t page = urcu_get_page_len();
	void *p, *ctrl;

	p = mmap((void *)(1UL << FT_SKIP_LEN_SHIFT), page, PROT_NONE,
		 MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE, -1, 0);
	if (p != MAP_FAILED) {
		/* Mapping succeeded: the bit is within the VA range. */
		munmap(p, page);
		return false;
	}
	if (errno != ENOMEM)
		return false;	/* EEXIST / EPERM / EINVAL / seccomp / ... */
	/*
	 * ENOMEM is overloaded: besides "address beyond TASK_SIZE" (what we
	 * want to confirm) it is also returned for a global mmap failure --
	 * notably vm.max_map_count exhaustion -- regardless of the address.
	 * Cross-check with a positive control: map one page at a
	 * kernel-chosen address.  If the control succeeds, mmap is healthy
	 * and the boundary ENOMEM was a genuine out-of-range rejection, so
	 * the bit is available.  If the control also fails, mmap is failing
	 * globally and the boundary ENOMEM proves nothing -- stay
	 * conservative and leave skip-compressed disabled.
	 */
	ctrl = mmap(NULL, page, PROT_NONE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
	if (ctrl == MAP_FAILED)
		return false;
	munmap(ctrl, page);
	return true;
}
#endif

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
	 * Default lookup optimization is SPECULATIVE: speculative
	 * descent with validate-and-retry, plus opportunistic
	 * skip-compressed pointer encoding on supported archs.
	 * Callers that need strict EAGER (per-step exact compare, no
	 * skip-compressed) must call
	 * cds_ft_group_attr_set_lookup_optimization(attr,
	 * CDS_FT_LOOKUP_OPTIMIZE_EAGER) explicitly.
	 */
	attr->speculative = true;
#ifdef FEATURE_FT_SKIP_COMPRESSED
	if (ft_skip_compressed_validate())
		attr->flags |= CDS_FT_FLAG_SKIP_COMPRESSED;
#endif
	/*
	 * Ordered sibling list ON by default: the library-owned cell is allocated
	 * per head regardless, so the key-ordered cds_ft_next / cds_ft_prev /
	 * cds_ft_for_each*_batched fast paths are available out of the box.  A
	 * group that never iterates in key order and wants to skip the per-mutation
	 * splice / unsplice opts out with cds_ft_group_attr_set_ordered_list(attr, false).
	 */
	attr->ordered_list_set = true;
	/*
	 * NUMA placement default: defer to process / libnuma policy.
	 * The library applies no mbind() of its own; the kernel honors
	 * numactl wrappers, set_mempolicy() calls, or falls back to
	 * first-touch when no process policy is set.  Applications that
	 * want explicit library-side placement should opt into
	 * CDS_FT_NUMA_INTERLEAVE (multi-reader workloads) or
	 * CDS_FT_NUMA_LOCAL (single-threaded / sharded workloads) via
	 * cds_ft_group_attr_set_numa_policy.
	 */
	attr->numa_policy = CDS_FT_NUMA_DEFAULT;
	attr->optimize = CDS_FT_OPTIMIZE_THROUGHPUT;
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
	size_t i;

	if (!key_to_ordinal || !ordinal_to_key)
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	/*
	 * Verify the two maps are exact inverse permutations.  Checking
	 * ordinal_to_key[key_to_ordinal[i]] == i for every i is sufficient:
	 * it forces key_to_ordinal to be injective (hence a bijection over
	 * the byte values) and ordinal_to_key to be its inverse (hence also
	 * a bijection), so both maps are valid permutations.
	 */
	for (i = 0; i < sizeof(attr->key_map.key_to_ordinal); i++) {
		if (ordinal_to_key[key_to_ordinal[i]] != i)
			return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}
	attr->key_map.identity = false;
	memcpy(attr->key_map.key_to_ordinal, key_to_ordinal, sizeof(attr->key_map.key_to_ordinal));
	memcpy(attr->key_map.ordinal_to_key, ordinal_to_key, sizeof(attr->key_map.ordinal_to_key));
	return CDS_FT_STATUS_OK;
}

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

enum cds_ft_status cds_ft_group_attr_set_speculative_key_offset(
		struct cds_ft_group_attr *attr,
		size_t key_offset)
{
	if (!attr)
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	attr->speculative_key_offset = key_offset;
	attr->speculative_key_offset_set = true;
	return CDS_FT_STATUS_OK;
}

enum cds_ft_status cds_ft_group_attr_set_key_len_offset(
		struct cds_ft_group_attr *attr,
		size_t offset)
{
	if (!attr)
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	attr->key_len_offset = offset;
	attr->key_len_offset_set = true;
	return CDS_FT_STATUS_OK;
}

enum cds_ft_status cds_ft_group_attr_set_ordered_list(
		struct cds_ft_group_attr *attr, bool ordered_list)
{
	if (!attr)
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	attr->ordered_list_set = ordered_list;
	return CDS_FT_STATUS_OK;
}

enum cds_ft_status cds_ft_group_attr_set_numa_policy(
		struct cds_ft_group_attr *attr,
		enum cds_ft_numa_policy policy)
{
	switch (policy) {
	case CDS_FT_NUMA_DEFAULT:
	case CDS_FT_NUMA_INTERLEAVE:
	case CDS_FT_NUMA_LOCAL:
		attr->numa_policy = policy;
		return CDS_FT_STATUS_OK;
	}
	return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
}

enum cds_ft_status cds_ft_group_attr_set_optimize(
		struct cds_ft_group_attr *attr,
		enum cds_ft_optimize opt)
{
	switch (opt) {
	case CDS_FT_OPTIMIZE_THROUGHPUT:
	case CDS_FT_OPTIMIZE_RSS:
		attr->optimize = opt;
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
	 * The per-instance attr currently carries only the exclusive-
	 * access flag (see cds_ft_attr_set_exclusive); calloc leaves it
	 * false (concurrent RCU readers permitted) as the default.
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
		ft_group->speculative_key_offset = attr->speculative_key_offset;
		ft_group->speculative_key_offset_set = attr->speculative_key_offset_set;
		ft_group->key_len_offset = attr->key_len_offset;
		ft_group->key_len_offset_set = attr->key_len_offset_set;
		ft_group->ordered_list_set = attr->ordered_list_set;
		ft_group->numa_policy = attr->numa_policy;
		ft_group->optimize = attr->optimize;
	} else {
		/*
		 * NULL attr: mirror the defaults set by
		 * cds_ft_group_attr_create — identity key map plus
		 * SPECULATIVE lookup optimization with opportunistic
		 * SKIP_COMPRESSED on supported archs, and the DEFAULT NUMA
		 * policy (interleave unless the process set an explicit
		 * preference — see ft_apply_interleave).
		 */
		ft_group->key_map.identity = true;
		ft_group->speculative = true;
#ifdef FEATURE_FT_SKIP_COMPRESSED
		if (ft_skip_compressed_validate())
			ft_group->flags |= CDS_FT_FLAG_SKIP_COMPRESSED;
#endif
		ft_group->ordered_list_set = true;	/* on by default; see attr_create */
		ft_group->numa_policy = CDS_FT_NUMA_DEFAULT;
		ft_group->optimize = CDS_FT_OPTIMIZE_THROUGHPUT;
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
	/*
	 * All tries are destroyed (nr_ft_instances == 0) and each
	 * cds_ft_destroy drained its deferred frees, so every ordered-list
	 * cell and every internal/compressed node that was allocated should
	 * have had its free issued.  Both balances are group-scoped because
	 * the bulk ops migrate cells and nodes between the group's tries.
	 */
	if (ft_debug_counters() &&
	    ft_group->nr_cells_allocated != ft_group->nr_cells_freed) {
		fprintf(stderr,
			"[error] Fractal Trie leaked %ld ordered-list cells. Allocated: %lu, freed: %lu.\n",
			(long) (ft_group->nr_cells_allocated - ft_group->nr_cells_freed),
			ft_group->nr_cells_allocated, ft_group->nr_cells_freed);
	}
	if (ft_debug_counters() &&
	    ft_group->nr_nodes_allocated != ft_group->nr_nodes_freed) {
		fprintf(stderr, "[error] Fractal Trie leaked %ld nodes. Allocated: %lu, freed: %lu.\n",
			(long) (ft_group->nr_nodes_allocated - ft_group->nr_nodes_freed),
			ft_group->nr_nodes_allocated, ft_group->nr_nodes_freed);
		fprintf(stderr, "  internal: alloc=%lu freed=%lu leaked=%ld\n",
			ft_group->nr_internal_alloc, ft_group->nr_internal_freed,
			(long) (ft_group->nr_internal_alloc - ft_group->nr_internal_freed));
		fprintf(stderr, "  compressed: alloc=%lu freed=%lu leaked=%ld\n",
			ft_group->nr_compressed_alloc, ft_group->nr_compressed_freed,
			(long) (ft_group->nr_compressed_alloc - ft_group->nr_compressed_freed));
	}
	cds_ft_free_all_arenas(ft_group);
	pthread_mutex_destroy(&ft_group->arena_lock);
	free(ft_group);
	return CDS_FT_STATUS_OK;
}

#ifdef DEBUG_COUNTERS
/*
 * DEBUG_COUNTERS-only cell leak introspection (no public header decl; tests
 * weak-reference it).  Reports the group's ordered-list cell alloc / free
 * balance.  Cells migrate between the group's tries via the bulk ops, so the
 * balance is meaningful only at group granularity; after every trie is drained
 * @allocated should equal @freed.  Caller must ensure no concurrent cell
 * mutation.
 */
void cds_ft_debug_cell_balance(const struct cds_ft_group *group,
		unsigned long *allocated, unsigned long *freed);
void cds_ft_debug_cell_balance(const struct cds_ft_group *group,
		unsigned long *allocated, unsigned long *freed)
{
	if (allocated)
		*allocated = group->nr_cells_allocated;
	if (freed)
		*freed = group->nr_cells_freed;
}

/*
 * DEBUG_COUNTERS-only node leak introspection (no public header decl; tests
 * weak-reference it).  Reports the group's internal/compressed node alloc /
 * free balance.  Nodes migrate between the group's tries via the bulk ops, so
 * the balance is meaningful only at group granularity; after every trie is
 * drained @allocated should equal @freed.  Caller must ensure no concurrent
 * node mutation.
 */
void cds_ft_debug_node_balance(const struct cds_ft_group *group,
		unsigned long *allocated, unsigned long *freed);
void cds_ft_debug_node_balance(const struct cds_ft_group *group,
		unsigned long *allocated, unsigned long *freed)
{
	if (allocated)
		*allocated = group->nr_nodes_allocated;
	if (freed)
		*freed = group->nr_nodes_freed;
}
#endif

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
	/* Cache the group's ordered-list mode for the read-side cell gate. */
	ft->ordered_list = ft_group->ordered_list_set;
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
	 * Allocate the root node (smallest popcount_2l type, initially empty).
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

	/*
	 * Ordinal-cell list enabled: eagerly allocate the writer-side scratch
	 * iterator used for cell predecessor discovery (ft_ord_cell_splice).
	 * ord_cell_head / ord_cell_tail are NULL from calloc.
	 */
	if (ft_group->ordered_list_set &&
	    cds_ft_iter_create(ft, &ft->ord_cell_scratch_iter) != CDS_FT_STATUS_OK) {
		free_cds_ft_node_unpublished(ft, root_node);
		free(ft);
		*result_ft = NULL;
		return CDS_FT_STATUS_MEMORY_ERROR;
	}

	uatomic_inc(&ft_group->nr_ft_instances, CMM_RELAXED);
	*result_ft = ft;
	FT_TP(ft_create, (const void *) ft, (const void *) ft_group);
	return CDS_FT_STATUS_OK;
}

/*
 * There should be no more concurrent add, delete, nor look-up performed
 * on the Fractal Trie while it is being destroyed (ensured by the
 * caller).
 */
void cds_ft_destroy(struct cds_ft *ft)
{
	const struct rcu_flavor_struct *flavor = ft->group->flavor;

	/*
	 * A compaction the caller never ended would otherwise strand its
	 * private ranges (relocated nodes in arena-untracked ranges) and leak
	 * its state/iter.  Finalize it here: cds_ft_compact_end merges the
	 * private ranges back into the arenas and frees the state, leaving a
	 * consistent trie to tear down.
	 */
	if (caa_unlikely(ft->active_compact != NULL)) {
		fprintf(stderr,
			"cds_ft_destroy: trie %p destroyed with a compaction still in progress; finalizing (missing cds_ft_compact_end)\n",
			(void *) ft);
		cds_ft_compact_end(ft->active_compact);
	}
	FT_TP(ft_destroy, (const void *) ft);
	/* Free root node. No concurrent readers at this point. */
	free_cds_ft_node(ft, ft_node_ptr(ft->root));
	/*
	 * Wait for in-flight call_rcu free to complete, so every deferred
	 * node free this trie issued is counted (group-scoped) before the
	 * trie's instance count is dropped.  The group-level node-leak check
	 * runs in cds_ft_group_destroy, once all tries have drained.
	 */
	flavor->barrier();
	if (ft->ord_cell_scratch_iter)
		cds_ft_iter_destroy(ft->ord_cell_scratch_iter);
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
		if (prev == NULL && ft->ordered_list) {
			/*
			 * Ordered-list head: prev is the head's cell (cell-tagged),
			 * whose ->parent is the owner and ->node is this head.
			 */
			struct ft_ord_cell *cell = ft_ord_cell_ptr(node->prev);

			if (ft_node_external((struct cds_ft_inode_flag *) node->prev) ||
			    (void *) cell->parent != (void *) owner_flag ||
			    cell->node != node) {
				if (out)
					fprintf(out, "ft_verify: depth %u: head %p cell %p {parent %p, node %p} != expected {owner %p, node %p}\n",
						depth, node, (void *) cell,
						(void *) (ft_node_external((struct cds_ft_inode_flag *) node->prev) ? NULL : cell->parent),
						(void *) (ft_node_external((struct cds_ft_inode_flag *) node->prev) ? NULL : cell->node),
						(void *) owner_flag, (void *) node);
				return -1;
			}
		} else if (prev == NULL) {
			/* List off: a head's prev is the owner (flagged parent) directly. */
			if ((void *) node->prev != (void *) owner_flag) {
				if (out)
					fprintf(out, "ft_verify: depth %u: head %p prev %p != owner %p\n",
						depth, node, node->prev, (void *) owner_flag);
				return -1;
			}
		} else if (node->prev != expected_prev) {
			if (out)
				fprintf(out, "ft_verify: depth %u: external chain node %p prev %p != predecessor %p\n",
					depth, node, node->prev, expected_prev);
			return -1;
		}
		(void) check_path;
		(void) path;
		(void) group;
		prev = node;
		node = ft_node_next(node);
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
int ft_verify_skip_encoding(const struct cds_ft *ft, FILE *out, struct cds_ft_inode_flag *slot_val,
		unsigned int depth)
{
	struct cds_ft_compressed_node *cn;
	unsigned int slen, cn_len;

	if (!ft_node_skip_compressed(slot_val))
		return 0;
	slen = ft_skip_len(slot_val);
	cn = ft_skip_to_compressed(ft, slot_val);
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
	/*
	 * Parent-slot offset round-trip.  Each compressed node records a
	 * pointer-stride offset from its parent to the slot that holds the
	 * pointer to itself (skip-encoded under FEATURE_FT_SKIP_COMPRESSED,
	 * a raw compressed flag otherwise).  ft_publish_to_parent uses it to
	 * refresh the slot when cn->child is replaced, and the parent-pointer
	 * backtrack (e.g. position-based remove) uses it to recover the slot.
	 * Verify the offset still resolves to a slot whose contents encode
	 * this very compressed.  A stale offset left after a recompact /
	 * graft / relocate is surfaced at the mutation that introduced it
	 * rather than as a corrupted slot at lookup / remove time.  Checked
	 * on every build (the offset is no longer skip-specific).
	 *
	 * NULL slot means the offset was never set (offset == 0 with a
	 * non-NULL parent).  No round-trip to verify in that case.
	 * ft_get_parent_slot wants a non-const ft for the root case
	 * (parent == NULL); cast away const since we only read *slot.
	 */
	{
		struct cds_ft_inode_flag **skip_slot =
			ft_get_parent_slot(cn_meta, (struct cds_ft *) ft);

		if (skip_slot) {
			struct cds_ft_inode_flag *slot_val = *skip_slot;
			struct cds_ft_compressed_node *target_cn = NULL;

			if (ft_node_skip_compressed(slot_val))
				target_cn = ft_skip_to_compressed(ft, slot_val);
			else if (slot_val &&
				 ft_node_compressed(slot_val))
				target_cn = ft_compressed_node_ptr(slot_val);
			if (target_cn != cn) {
				if (out)
					fprintf(out, "ft_verify: depth %u: compressed %p parent_slot_offset round-trip mismatch: slot %p holds %p (resolves to cn %p, expected %p)\n",
						depth, node_flag, skip_slot,
						slot_val, target_cn, cn);
				return -1;
			}
		}
	}
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

	/* --- Internal node (popcount, pigeon) --- */
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
		/*
		 * Slot-offset round-trip (non-root): the recorded
		 * parent_slot_offset must resolve, in the parent body, to the
		 * slot that holds this node.  Catches a stale/unset offset
		 * (e.g. a placement that passed a NULL slot to ft_set_parent)
		 * at the mutation that introduced it, rather than as a
		 * corrupted parent-pointer backtrack later.
		 */
		if (metadata->parent) {
			struct cds_ft_inode_flag **slot =
				ft_get_parent_slot(metadata,
						(struct cds_ft *) ft);

			if (!slot ||
			    ft_node_ptr(*slot) != ft_node_ptr(node_flag)) {
				if (out)
					fprintf(out, "ft_verify: depth %u: internal node %p slot_offset round-trip mismatch: slot %p holds %p\n",
						depth, node_flag, (void *) slot,
						slot ? (void *) *slot : NULL);
				return -1;
			}
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
			struct cds_ft_inode_flag **slot = NULL;
			struct cds_ft_inode_flag *child_raw =
				ft_node_get_nth_skip(node_flag, &slot, (uint8_t) key, FT_PF_NONE);
			struct cds_ft_inode_flag *child;

			if (!child_raw)
				continue;
			/*
			 * Raw slot may be skip-encoded; verify the encoded
			 * slen matches the underlying compressed's cn->len
			 * before resolving for the recursion.
			 */
			if (ft_verify_skip_encoding(ft, out, child_raw, depth + 1))
				return -1;
#ifdef FEATURE_FT_SKIP_COMPRESSED
			/*
			 * Slot-centric skip-slot invariant: a slot holding
			 * skip(cn) must be the very slot cn records as its
			 * skip_slot (the inverse of the cn-centric round-trip
			 * check above).  rec != slot names a dangling/aliased
			 * skip slot directly: a reparent left cn's skip_slot
			 * pointing elsewhere, or a placement never recorded it.
			 */
			if (ft_node_skip_compressed(child_raw) && slot) {
				struct cds_ft_compressed_node *scn =
					ft_skip_to_compressed(ft, child_raw);
				struct cds_ft_metadata *scnm =
					cds_ft_item_to_metadata(
						(struct cds_ft_inode *) scn);
				struct cds_ft_inode_flag **rec =
					ft_get_parent_slot(scnm,
						(struct cds_ft *) ft);

				if (rec != slot) {
					if (out)
						fprintf(out, "ft_verify: depth %u: node %p key %u slot %p holds skip(cn %p) but cn->skip_slot names %p (dangling skip slot)\n",
							depth, node_flag, key,
							(void *) slot, (void *) scn,
							(void *) rec);
					return -1;
				}
			}
#endif
			child = ft_resolve_skip_compressed(ft, child_raw);
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
 * ft_verify_ord_cells: verify the ordinal-cell list against the trie.
 *
 * Walks the trie in key order via the relational descent and the cell list in
 * lockstep, asserting: the list visits exactly the trie's distinct-key heads
 * in the same order; each visited cell is the head's own cell (head->prev) and
 * cell->node points back at that head; the back-edge invariant
 * ord_next(c)->ord_prev == c holds; the minimum cell has ord_prev == NULL and
 * the cached ord_cell_head / ord_cell_tail equal the trie minimum / maximum.
 *
 * The oracle stays independent of the cell list (cache_valid cleared before
 * each step forces the full descent).  Runs under the caller's writer
 * exclusion.  Returns 0 on success, -1 on the first violation.
 */
static
int ft_verify_ord_cells(const struct cds_ft *cft, FILE *out)
{
	struct cds_ft *ft = (struct cds_ft *) cft;
	struct cds_ft_iter *iter;
	struct cds_ft_node *trie_head;
	struct ft_ord_cell *cell, *max_cell = NULL;
	int ret = 0;

	if (cds_ft_iter_create(ft, &iter) != CDS_FT_STATUS_OK) {
		if (out)
			fprintf(out, "ft_verify: ord-cell iter allocation failed\n");
		return -1;
	}
	iter->key_len = 0;
	iter->prefix_len = 0;
	iter->cache_valid = false;	/* force descent oracle */
	cds_ft_lookup_inequality_impl(ft, iter, FT_LOOKUP_GE,
			FT_LOOKUP_LIMIT_FIRST, false, false);
	trie_head = cds_ft_iter_node(iter);
	cell = ft->ord_cell_head;
	if (trie_head &&
	    ft_ord_cell_resolve_ord(&ft_ord_cell_ptr(trie_head->prev)->ord_prev)
		!= NULL) {
		if (out)
			fprintf(out, "ft_verify: ord-cell min head %p cell has ord_prev != NULL\n",
				(void *) trie_head);
		ret = -1;
		goto out;
	}
	while ((trie_head = cds_ft_iter_node(iter)) != NULL) {
		struct ft_ord_cell *head_cell =
			ft_ord_cell_ptr(rcu_dereference(trie_head->prev));
		struct ft_ord_cell *next_cell;

		if (cell != head_cell) {
			if (out)
				fprintf(out, "ft_verify: ord-cell order mismatch: list cell %p vs trie head %p cell %p\n",
					(void *) cell, (void *) trie_head,
					(void *) head_cell);
			ret = -1;
			goto out;
		}
		if (cell->node != trie_head) {
			if (out)
				fprintf(out, "ft_verify: ord-cell %p node %p != trie head %p\n",
					(void *) cell, (void *) cell->node,
					(void *) trie_head);
			ret = -1;
			goto out;
		}
		max_cell = cell;
		next_cell = ft_ord_cell_resolve_ord(&cell->ord_next);
		if (next_cell &&
		    ft_ord_cell_resolve_ord(&next_cell->ord_prev) != cell) {
			if (out)
				fprintf(out, "ft_verify: ord-cell back-edge broken at cell %p (ord_next %p whose ord_prev is %p)\n",
					(void *) cell, (void *) next_cell,
					(void *) ft_ord_cell_resolve_ord(&next_cell->ord_prev));
			ret = -1;
			goto out;
		}
		cell = next_cell;
		iter->cache_valid = false;	/* force descent oracle */
		cds_ft_lookup_inequality_impl(ft, iter, FT_LOOKUP_GT,
				FT_LOOKUP_LIMIT_NONE, false, false);
	}
	if (cell != NULL) {
		if (out)
			fprintf(out, "ft_verify: ord-cell list longer than trie (extra cell %p)\n",
				(void *) cell);
		ret = -1;
	}
	if (ret == 0 && ft->ord_cell_tail != max_cell) {
		if (out)
			fprintf(out, "ft_verify: ord-cell ord_cell_tail %p != trie maximum cell %p\n",
				(void *) ft->ord_cell_tail, (void *) max_cell);
		ret = -1;
	}
out:
	cds_ft_iter_destroy(iter);
	return ret;
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
	if (ft->group->ordered_list_set && ft_verify_ord_cells(ft, out))
		return CDS_FT_STATUS_INTEGRITY_ERROR;
	return CDS_FT_STATUS_OK;
}

/*
 * cds_ft_compact internals.
 *
 * Relocate every internal node of a trie into fresh, densely-packed
 * allocations in DFS descent order, so the node arenas defragment: the old
 * ranges drain as relocated nodes are freed, and the allocator reclaims the
 * ones that empty (see cds_ft_do_free_item).  The relative layout order is
 * not a measurable performance lever (DFS, BFS+DFS and density-DFS all tie);
 * the win is recovering locality lost to churn/graft, so we use plain DFS.
 *
 * Concurrency: the caller's writer exclusion keeps the trie quiescent w.r.t.
 * other writers for the whole walk; concurrent RCU readers are fine.  Each
 * node is republished and its old copy RCU-freed (ft_node_recompact +
 * cds_ft_free_item), so a reader observes the old or the new node, never a
 * freed one.  Compressed nodes are relocated too, via
 * ft_compact_relocate_compressed: a traditional compressed node through its
 * grandparent slot, a skip-compressed node recovered from its skip pointer
 * with ft_skip_to_compressed with no grandparent repoint (the slot holds a
 * skip pointer to the target, not to cn).  This covers a skip whose target is
 * an external leaf: cn is relocated, the leaf is not (leaves are
 * application-owned), and the descent ends at the loop's ft_node_external
 * check.  An internal skip target is then relocated through cn->child, where
 * ft_node_recompact's dual-pointer publish updates both cn->child and the skip
 * pointer.
 */
/* Relocate the internal node at *@holder into a fresh slot; RCU-free the old. */
static
void ft_compact_relocate_at(struct cds_ft *ft, struct cds_ft_inode_flag **holder)
{
	struct cds_ft_inode_flag *nf = *holder;
	unsigned int type_index = ft_node_type(nf);
	struct cds_ft_inode *node = ft_node_ptr(nf), *old_ret = NULL;
	struct cds_ft_metadata *meta = cds_ft_item_to_metadata(node);

	(void) ft_node_recompact(FT_RECOMPACT_RELOCATE, ft, type_index,
			&ft_types[type_index], node, meta, holder,
			0, NULL, NULL, &old_ret, holder == &ft->root, 0,
			false);
	/*
	 * The old node was just unpublished; concurrent readers may still
	 * hold it, so free it after a grace period.  Its range's nr_live
	 * decrements in the callback, so a fully-drained range self-reclaims.
	 * ALWAYS deferred, even on an exclusive trie: the step's walk keeps
	 * navigating relative to nodes it has just unpublished, and the
	 * exclusive-mode synchronous free threads the freelist link through
	 * the freed slot immediately.
	 */
	if (old_ret)
		cds_ft_free_item_deferred(ft, cds_ft_item_to_metadata(old_ret));
}

/*
 * Relocate a compressed node @cn into a fresh slot, keeping the same length.
 * Because the length is unchanged, the skip pointer's encoded length still
 * matches the relocated node (cn2->len == skip_len, whether a reader resolves
 * the old or the new node via the skip child's back-pointer), and the
 * parent-pointer flip is exactly the transient that ft_skip_reanchor already
 * tolerates; the old node stays alive until its grace period.  A compressed
 * node never carries external_nodes (that would fork a path that must remain
 * skippable), so the only reference to redirect is its child's back-pointer.
 * The grandparent slot is repointed only for a traditional (non-skip)
 * compressed node -- a skip pointer addresses the target, not @cn, so it needs
 * no change.
 *
 * @gp_slot: grandparent slot holding the cn flag (traditional), or NULL (skip).
 * Returns the new compressed node (or @cn unchanged on allocation failure).
 */
static
struct cds_ft_compressed_node *ft_compact_relocate_compressed(struct cds_ft *ft,
		struct cds_ft_compressed_node *cn,
		struct cds_ft_inode_flag **gp_slot)
{
	struct cds_ft_metadata *cn_meta =
		cds_ft_item_to_metadata((struct cds_ft_inode *) cn);
	struct cds_ft_metadata *cn2_meta;
	struct cds_ft_compressed_node *cn2;
	struct cds_ft_inode_flag *cn2_flag;
	uint8_t len = cn->len;

	cn2 = alloc_compressed_node(ft, len, &cn2_meta);
	if (!cn2)
		return cn;	/* OOM: leave in place (best-effort) */
	cn2->len = len;
	cn2->child = cn->child;
	memcpy(cn2->key_bytes, cn->key_bytes, len);
	cn2_meta->parent = cn_meta->parent;
	/*
	 * The relocated compressed node keeps the SAME slot in the SAME
	 * parent, so its parent-slot offset is identical.  Copy it on every
	 * build: the offset is no longer skip-specific (it backs the
	 * parent-pointer backtrack's O(1) slot recovery), and a position-
	 * based remove that climbs via ft_get_parent_slot would otherwise
	 * read a fresh-zeroed offset and resolve the wrong slot.
	 */
	cn2_meta->parent_slot_offset = cn_meta->parent_slot_offset;
	/* Same slot in the same parent => same incoming edge byte (up-walk source). */
	cn2_meta->incoming_byte = cn_meta->incoming_byte;
	cn2_meta->nr_child = cn_meta->nr_child;		/* == 1 for a compressed node */
	cn2_meta->external_nodes = NULL;		/* never set on a compressed node */
	ft_nr_keys_store(cn2_meta, ft_nr_keys_get(cn_meta), CMM_RELAXED);
	cn2_flag = ft_compressed_node_flag(cn2);
	/* Redirect the child's back-reference (internal: parent; external: prev). */
	ft_set_parent(ft, cn2->child, cn2_flag, &cn2->child);
	if (gp_slot)
		rcu_assign_pointer(*gp_slot, cn2_flag);
	/* Always-deferred free: see ft_compact_relocate_at. */
	FT_TP(compressed_free, (const void *) ft_compressed_node_flag(cn));
	cds_ft_free_item_deferred(ft, cn_meta);
	if (ft_debug_counters()) {
		uatomic_inc(&ft->group->nr_nodes_freed);
		uatomic_inc(&ft->group->nr_compressed_freed);
	}
	return cn2;
}

/*
 * Relocate one ordinal cell into a fresh slot from the dedicated cell arena.
 * The active recompaction context routes the allocation into a private cell
 * range, so cells relocated in key-traversal order pack densely there -- the
 * dense ord-walk stride that makes ordered iteration a sequential scan rather
 * than a random pointer chase.  Returns the new cell, or @old unchanged on
 * allocation failure (best-effort: leave it in place).
 *
 * Atomicity reuses ft_ord_cell_swap for the two ordered-list edges
 * (pred->ord_next / succ->ord_prev flip together via the flip-latch, so a
 * bidirectional ordered reader never sees a half-relocated list; ord_cell_head
 * /tail follow).  The head's UPWARD reference (head->prev) is then re-pointed
 * with a PLAIN RCU store: an up-walk reader resolves the old or the new cell,
 * both carrying an IDENTICAL parent (compaction runs under writer exclusion, so
 * @old->parent is settled), and @old stays live until its grace period -- so no
 * flip is needed, and head->prev must never hold a flip-proxy (ft_resolve_head_
 * prev does not resolve one).  @old keeps its own links for parked ordered
 * readers and is RCU-freed (its general-arena range drains for reclaim).
 */
static
struct ft_ord_cell *ft_compact_relocate_cell(struct cds_ft *ft,
		struct ft_ord_cell *old)
{
	struct cds_ft_metadata *meta = cds_ft_alloc_cell_item(ft);
	struct cds_ft_node *head = old->node;
	struct ft_ord_cell *new_cell;

	if (!meta)
		return old;		/* OOM: best-effort, leave in place */
	if (ft_debug_counters())
		uatomic_inc(&ft->group->nr_cells_allocated);
	new_cell = (struct ft_ord_cell *) cds_ft_metadata_to_item(meta);
	new_cell->node = head;
	new_cell->parent = old->parent;
	/* Carry the head's edge byte across the relocation (up-walk key source). */
	meta->incoming_byte = cds_ft_item_to_metadata(old)->incoming_byte;
	/* ord_prev / ord_next are set from @old's neighbours by the swap. */
	ft_ord_cell_swap(ft, old, new_cell);
	rcu_assign_pointer(head->prev, ft_ord_cell_flag(new_cell));
	/* Always-deferred free: see ft_compact_relocate_at. */
	if (ft_debug_counters())
		uatomic_inc(&ft->group->nr_cells_freed);
	cds_ft_free_item_deferred(ft, cds_ft_item_to_metadata(old));
	return new_cell;
}

/*
 * Forward relocate-descent: walk from the root to the leaf for @key
 * (the user key), relocating every internal node on the path that has
 * not already been relocated this pass (recompact_private).  Mirrors the
 * lookup descent's ordinal mapping and skip/compressed advancement, but
 * tracks the holder at each step (which the read descent does not) so it
 * can republish.  A skip target is relocated through the compressed
 * node's cn->child slot, so ft_node_recompact republishes both cn->child
 * and the skip pointer.  Increments *@relocated per node moved.
 */
static
void ft_compact_descend(struct cds_ft *ft, const uint8_t *key,
		size_t key_len, unsigned long *relocated)
{
	const struct cds_ft_key_map *km = &ft->group->key_map;
	struct cds_ft_inode_flag **holder = &ft->root;
	size_t depth = 0;

	for (;;) {
		struct cds_ft_inode_flag *nf = rcu_dereference(*holder);
		struct cds_ft_inode_flag **child_slot = NULL;
		struct cds_ft_inode_flag *raw;
		uint8_t ord;

		if (ft_node_external(nf))
			return;		/* reached a leaf */
		if (!cds_ft_metadata_in_recompact_private(
				cds_ft_item_to_metadata(ft_node_ptr(nf)))) {
			ft_compact_relocate_at(ft, holder);
			(*relocated)++;
			nf = rcu_dereference(*holder);	/* the relocated node */
		}
		if (depth >= key_len)
			return;		/* consumed the whole key */
		ord = key_to_ordinal(key[depth], km);
		raw = ft_node_get_nth_skip(nf, &child_slot, ord, FT_PF_NONE);
		if (!raw)
			return;		/* child absent (e.g. concurrent removal) */
		depth++;		/* child-index byte (matches iter_key = *key++) */
		if (ft_node_skip_compressed(raw)) {
			struct cds_ft_compressed_node *cn =
				ft_skip_to_compressed(ft, raw);

			depth += ft_skip_len(raw);
			/*
			 * Relocate the compressed node carrying the skip.
			 * ft_skip_to_compressed recovers it through the child's
			 * back-pointer, so a skip whose target is an external leaf is
			 * handled too (its parent is cell-indirect; compaction's
			 * writer exclusion keeps the back-pointer settled).  The
			 * grandparent slot is a skip pointer addressing the target,
			 * not cn, so it needs no repoint (NULL).  Continue at
			 * cn->child: an internal target is relocated next iteration,
			 * while an external leaf (not itself relocatable) ends the
			 * descent at the loop's ft_node_external check.
			 */
			if (!cds_ft_metadata_in_recompact_private(
					cds_ft_item_to_metadata((struct cds_ft_inode *) cn))) {
				cn = ft_compact_relocate_compressed(ft, cn, NULL);
				(*relocated)++;
			}
			holder = &cn->child;
		} else if (ft_node_compressed(raw)) {
			struct cds_ft_compressed_node *cn = ft_compressed_node_ptr(raw);

			depth += cn->len;
			/* Traditional: the grandparent slot (child_slot) holds the cn flag. */
			if (!cds_ft_metadata_in_recompact_private(
					cds_ft_item_to_metadata((struct cds_ft_inode *) cn))) {
				cn = ft_compact_relocate_compressed(ft, cn, child_slot);
				(*relocated)++;
			}
			holder = &cn->child;
		} else {
			holder = child_slot;	/* plain internal or external child */
		}
	}
}

/* Default per-step relocation budget when cds_ft_compact_step(batch == 0). */
#define FT_COMPACT_BATCH_DEFAULT	64

/*
 * Resumable compaction state.  Heap-allocated by cds_ft_compact_begin so the
 * caller treats it as opaque.  The navigation iterator is left in its default
 * CACHED mode: within a step's read-lock window it advances incrementally on
 * its cached path (the relocated-away nodes it navigates stay alive until the
 * read-unlock, and relocation preserves key order), and it retains the cursor
 * key itself.  Between steps the path is invalidated, so the iterator's key is
 * the only resume token; the private-range context persists, merged at end.
 */
struct cds_ft_compact_state {
	struct cds_ft *ft;
	struct cds_ft_iter *iter;
	struct ft_recompact_alloc_ctx ctx;
	bool started;
	bool done;
};

struct cds_ft_compact_state *cds_ft_compact_begin(struct cds_ft *ft)
{
	struct cds_ft_compact_state *st;

	if (caa_unlikely(ft->active_compact != NULL)) {
		/*
		 * A compaction is already in flight on this trie (a previous
		 * one was never ended, or two are being started).  Programmer
		 * error: assert in debug, and refuse in release rather than
		 * abandon the in-flight one.
		 */
		assert(!"cds_ft_compact_begin: a compaction is already in progress on this trie");
		return NULL;
	}
	st = calloc(1, sizeof(*st));
	if (!st)
		return NULL;
	if (cds_ft_iter_create(ft, &st->iter) != CDS_FT_STATUS_OK) {
		free(st);
		return NULL;
	}
	ft_recompact_alloc_init(&st->ctx);
	st->ft = ft;
	ft->active_compact = st;
	return st;
}

bool cds_ft_compact_step(struct cds_ft_compact_state *st, size_t batch)
{
	struct cds_ft *ft = st->ft;
	const struct rcu_flavor_struct *flavor = ft->group->flavor;
	unsigned long relocated = 0;

	if (st->done)
		return false;
	if (batch == 0)
		batch = FT_COMPACT_BATCH_DEFAULT;

	/*
	 * Route this step's relocations into private ranges, and hold the
	 * RCU read lock for the whole batch: it keeps the iterator's reads
	 * and our descents safe, and defers our own call_rcu node frees until
	 * the read-unlock between steps (where the drained ranges reclaim and
	 * concurrent mutations get their window).
	 */
	ft_recompact_alloc_set_active(&st->ctx);
	flavor->read_lock();
	while (relocated < batch) {
		uint8_t key[FT_MAX_KEY_LEN];
		size_t key_len;
		enum cds_ft_status s;

		/*
		 * Cached iter: lookup_gt advances incrementally on the cached
		 * path within this read-lock window.  The first lookup of each
		 * batch re-descends the current structure (the path was
		 * invalidated at the previous read-unlock) from the iterator's
		 * retained key.
		 */
		s = st->started ? cds_ft_lookup_gt(ft, st->iter)
				: cds_ft_lookup_first(ft, st->iter);
		st->started = true;
		if (s != CDS_FT_STATUS_OK) {	/* NOT_FOUND or error: finished */
			st->done = true;
			break;
		}
		if (cds_ft_iter_get_key(st->iter, key, sizeof(key),
				&key_len) != CDS_FT_STATUS_OK) {
			st->done = true;
			break;
		}
		ft_compact_descend(ft, key, key_len, &relocated);
		/*
		 * Relocate this key's cell into a dense private cell range, in the
		 * same key order the iterator visits -- so the ordered cell list
		 * becomes a near-sequential scan.  iter->node is the chain head;
		 * its cell is head->prev.  Skip a cell already moved this pass
		 * (its range is recompact_private), mirroring the node descent, so
		 * the pass is idempotent and re-visits do not re-allocate.
		 */
		if (ft->group->ordered_list_set && st->iter->node) {
			struct ft_ord_cell *cell = ft_ord_cell_ptr(
				rcu_dereference(st->iter->node->prev));

			if (!cds_ft_metadata_in_recompact_private(
					cds_ft_item_to_metadata(cell))) {
				ft_compact_relocate_cell(ft, cell);
				relocated++;
			}
		}
	}
	/*
	 * Drop the cached path before releasing the read lock: the nodes it
	 * references become eligible for the grace-period free once unlocked.
	 * Bind (not just invalidate) so the iterator's key is materialized into
	 * its own buffer -- on a reference-keycopy / ordinal-cell group the live
	 * key is a leaf reference that does NOT survive the unlock, and the next
	 * step re-descends from that key.
	 */
	cds_ft_iter_bind_key(st->iter);
	flavor->read_unlock();
	ft_recompact_alloc_set_active(NULL);
	return !st->done;
}

void cds_ft_compact_end(struct cds_ft_compact_state *st)
{
	st->ft->active_compact = NULL;
	ft_recompact_alloc_merge(&st->ctx);
	cds_ft_iter_destroy(st->iter);
	free(st);
}

void cds_ft_compact(struct cds_ft *ft)
{
	CDS_FT_SCOPED_WRITER(ft);
	struct cds_ft_compact_state *st = cds_ft_compact_begin(ft);

	if (!st)
		return;		/* OOM: best-effort, leave the trie as-is */
	while (cds_ft_compact_step(st, 0))
		;
	cds_ft_compact_end(st);
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

		child_node_flag = ft_node_get_nth(ft, node_flag, NULL, (uint8_t) key, FT_PF_NONE);
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

			child = ft_node_get_nth(ft, node_flag, NULL, (uint8_t) key, FT_PF_NONE);
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
		size_t klen = ft_iter_resolve_key_len(iter);

		if (klen > max_len)
			max_len = klen;
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

		child_node_flag = ft_node_get_nth(ft, node_flag, NULL, (uint8_t) key, FT_PF_NONE);
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
	struct cds_ft_stats *stats;
	int level = 0;

	/*
	 * struct cds_ft_stats is several MiB: a per-level node-type
	 * distribution histogram (257 buckets) for each of FT_MAX_DEPTH
	 * levels.  That is far too large for the stack on threads with a
	 * small stack (e.g. musl's 128 KiB default), so heap-allocate it.
	 */
	stats = calloc(1, sizeof(*stats));
	if (!stats) {
		fprintf(out, "Fractal Trie (%p) Statistics: out of memory\n", ft);
		return;
	}

	node_flag = rcu_dereference(ft->root);

	/* Root is always present and always internal. */
	calc_stats_node(ft, node_flag, stats, level);
	calc_stats_node_recursive(ft, node_flag, stats, level + 1);
	do_show_stats(ft, out, stats);
	free(stats);
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
	size_t max_key_len = ft->group->max_key_len;
	/*
	 * Tail pad of FT_KEY_READABLE_PAD bytes after the key buffer so
	 * the descent can SIMD-load 32 bytes past key[key_len-1] without
	 * faulting.  The iter-based lookup paths pass FT_KEY_READABLE_PAD
	 * as @key_readable_pad (bytes safely loadable past key end).
	 */
	size_t key_size  = (max_key_len + FT_KEY_READABLE_PAD) * sizeof(uint8_t);
	struct cds_ft_iter *iter = calloc(1, sizeof(*iter) + key_size);

	CDS_FT_SCOPED_READER(ft);
	if (!iter) {
		*result_iter = NULL;
		return CDS_FT_STATUS_MEMORY_ERROR;
	}
	iter->ft = ft;
	iter->cache_mode = CDS_FT_ITER_CACHED;
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
	size_t klen = ft_iter_resolve_key_len(iter);

	*result_key_len = klen;
	if (klen > result_key_max_len)
		return CDS_FT_STATUS_OVERFLOW_ERROR;
	/*
	 * Lazy-ref: when the current key is a live reference into iter->node,
	 * read it from the leaf; else from the iter_key value.  ft_iter_resolve_
	 * key_len above materialized a deferred (variable-length) length from the
	 * same leaf.  Same continuous-RCU-lock contract as reusing the cached
	 * position (cross-CS callers cds_ft_iter_bind_key() first).
	 */
	ft_ordinals_to_key(result_key, ft_iter_read_key(iter), klen,
			&iter->ft->group->key_map);
	return CDS_FT_STATUS_OK;
}

enum cds_ft_status cds_ft_node_get_key(const struct cds_ft *ft,
		const struct cds_ft_node *node, uint8_t *result_key,
		size_t result_key_max_len, size_t *result_key_len)
{
	const struct cds_ft_group *group = ft->group;
	const uint8_t *ordinals;
	size_t klen;
	uint8_t scratch[FT_MAX_KEY_LEN];

	if (!node)
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	if (group->speculative_key_offset_set) {
		/*
		 * In-leaf key: the head stores its ordinal key bytes (and, for a
		 * variable-length group, its length) directly.  Read them in place.
		 */
		ordinals = (const uint8_t *) node + group->speculative_key_offset;
		klen = (group->key_len != CDS_FT_LEN_VARIABLE) ? group->key_len :
			*(const size_t *) ((const char *) node +
				group->key_len_offset);
	} else {
		/*
		 * No in-leaf key: rebuild the ordinal key by the structural parent
		 * up-walk from the head's cell (the same EAGER source cds_ft_iter_
		 * get_key uses).  Requires the cell to exist -- i.e. the ordered list
		 * enabled, since a list-off trie allocates no cells (head->prev is the
		 * flagged parent directly, which is NOT an up-walk source).  The walk
		 * fills @scratch from the tail and returns the length; the key starts
		 * at @scratch[max-len].
		 */
		if (ft->ordered_list) {
			struct ft_ord_cell *cell = ft_ord_cell_ptr(
				rcu_dereference(((struct cds_ft_node *) node)->prev));
			size_t max_len = group->max_key_len;

			klen = ft_rebuild_key_upwalk(ft, cell, scratch, max_len);
			ordinals = scratch + (max_len - klen);
		} else {
			/* List off: no cell, no in-leaf key -> not materializable. */
			return CDS_FT_STATUS_NOT_FOUND;
		}
	}
	*result_key_len = klen;
	if (klen > result_key_max_len)
		return CDS_FT_STATUS_OVERFLOW_ERROR;
	ft_ordinals_to_key(result_key, ordinals, klen, &group->key_map);
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

/*
 * Internal: set the iterator's search key from ALREADY-ORDINAL bytes (no key
 * map application; the public cds_ft_iter_set_key remaps and validates).  Used
 * by the bulk-op machinery, which converts the caller's key once at the public
 * entry and threads the ordinal form internally.
 */
static
void ft_iter_set_key_ordinals(struct cds_ft_iter *iter, const uint8_t *ordinals,
		size_t key_len)
{
	/*
	 * Setting a new key invalidates the cached position: the next
	 * operation re-descends from the root by the new key.
	 */
	memcpy(iter_key(iter), ordinals, key_len);
	iter->cache_valid = false;
	iter_debug_path_clear(iter);
	iter->path_len = 0;
	iter->key_len = key_len;
	iter->key_off = 0;	/* search key sits at the front of iter_key */
}

enum cds_ft_status cds_ft_iter_set_key(struct cds_ft_iter *iter, const uint8_t *key, size_t key_len)
{
	const struct cds_ft_key_map *km = &iter->ft->group->key_map;
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
	ft_iter_set_key_ordinals(iter, key_ordinals, key_len);
	FT_TP(iter_set_key_exit, (const void *) iter->ft, (const void *) iter,
		(int) iter->path_len);
	return CDS_FT_STATUS_OK;
}

/*
 * The prefix is a subset of the current key. Set the key before setting
 * the prefix length.
 */
enum cds_ft_status cds_ft_iter_set_prefix_len(struct cds_ft_iter *iter, size_t prefix_len)
{
	if (prefix_len > ft_iter_resolve_key_len(iter)) {
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
	iter->cache_valid = false;
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
		memset(iter_key(iter), 0, ft_group->max_key_len * sizeof(uint8_t));
	}
#endif
}

void cds_ft_iter_bind_key(struct cds_ft_iter *iter)
{
	FT_TP(iter_bind_key, (const void *) iter->ft, (const void *) iter);
	/*
	 * Materialize a live leaf-referenced key into the iterator's own buffer
	 * BEFORE detaching, so a cross-critical-section resume re-descends from
	 * the stable buffer rather than the (post-unlock, possibly reclaimed)
	 * leaf.  A no-op self-copy for groups whose key is already a value, in
	 * which case this is a plain invalidate.
	 */
	ft_iter_materialize_key(iter);
	iter->cache_valid = false;
	iter_debug_path_clear(iter);
	iter->path_len = 0;
	iter->node = NULL;
}

void cds_ft_iter_copy(struct cds_ft_iter *dst, const struct cds_ft_iter *src)
{
	dst->status = src->status;
	dst->cache_mode = src->cache_mode;
	dst->cache_valid = src->cache_valid;
	dst->path_len = src->path_len;
	dst->key_len = src->key_len;
	dst->key_off = src->key_off;
	dst->prefix_len = src->prefix_len;
	dst->node = src->node;
	dst->ord_cell = src->ord_cell;
	dst->ord_cell_node = src->ord_cell_node;
#ifdef URCU_FRACTAL_TRIE_DEBUG_PATH
	dst->gp_state = src->gp_state;
	dst->gp_state_valid = src->gp_state_valid;
#endif
	/*
	 * Copy the key buffer only when the source key is a VALUE there.  It is
	 * NOT one when:
	 *  - the key is a live leaf reference (identity cell position with an
	 *    in-leaf key): iter_key(src) is stale; @dst shares src->node +
	 *    cache_valid, so it reads the key from the same leaf;
	 *  - the length is the deferred LAZY sentinel (EAGER identity cell
	 *    position, no in-leaf key): nothing was materialized yet; @dst
	 *    shares the position and re-derives key + length from the up-walk
	 *    on demand.  Copying would be a SIZE_MAX memcpy.
	 * Copy at key_off: an up-walk-materialized key lives at the buffer
	 * TAIL, mirrored to the same offset in @dst (key_off copied above).
	 */
	if (!ft_iter_key_referenced(src) &&
			src->key_len != FT_ITER_KEY_LEN_LAZY)
		memcpy(iter_key(dst) + src->key_off,
			iter_key(src) + src->key_off, src->key_len);
}

struct cds_ft_node *cds_ft_iter_node(const struct cds_ft_iter *iter)
{
	return iter->node;
}

/*
 * Iterator-free ordered batched gather, shared by cds_ft_cell_next_batch /
 * cds_ft_cell_prev_batch.  Walks @ft's ordered cell list from @cursor (NULL =
 * the list minimum for forward, maximum for reverse), emits up to @cap opaque
 * CELL handles into @buf, and writes the resume cell -- the one after the last
 * emitted, NULL at the end -- to @next_cursor.  The cmm_ptr_eq physical-next
 * prediction (post-compaction the cells are contiguous in key order at @stride,
 * so a cell's ord_next/ord_prev usually resolves to cur ± stride; cmm_ptr_eq
 * validates the arithmetic guess and lets the compiler address the NEXT load off
 * it, breaking the dependent-load chain so the gather pipelines) pays only on a
 * compacted arena and is a no-op otherwise.  @forward is a literal at both call
 * sites so always_inline folds the direction out.
 *
 * Cell-native: the cursor, the emitted handles, and the resume cursor are all
 * CELLS, so the walk never touches an external head node -- not to emit, not to
 * resume (no node<->cell round-trip, hence no resume cache is needed).  The
 * caller recovers the node from a cell via cds_ft_cell_node() (an inlined load
 * of the hot cell line, not a cold node->prev) and materializes keys lazily via
 * cds_ft_cell_get_key(); a keyless (no in-leaf key) ordered scan thus touches
 * ZERO external-head cachelines in the library.
 *
 * ORDERED-LIST ONLY: the step IS the cell ord_next / ord_prev walk.  A list-off
 * trie has no cell list, so it yields CDS_FT_STATUS_NOT_SUPPORTED (*count = 0,
 * *next_cursor = NULL): use the iterator (cds_ft_next / cds_ft_prev, structural
 * descent) there.
 *
 * RCU CONTRACT: @cursor and every emitted cell are valid only while the read
 * lock that produced @cursor is held continuously (see cds_ft_cell_get_key).
 */
static inline __attribute__((always_inline))
enum cds_ft_status ft_cell_batch_dir(struct cds_ft *ft,
		const struct cds_ft_cell *cursor, const struct cds_ft_cell **buf,
		size_t cap, size_t *count,
		const struct cds_ft_cell **next_cursor, const bool forward)
{
	size_t n = 0;
	const uintptr_t stride = (uintptr_t) 1 << FT_ORD_CELL_ALLOC_ORDER;
	struct ft_ord_cell *cur;

	CDS_FT_ASSERT_RCU_READ_LOCKED(ft);
	*next_cursor = NULL;
	*count = 0;
	if (caa_unlikely(!ft->ordered_list))
		return CDS_FT_STATUS_NOT_SUPPORTED;
	if (caa_unlikely(cap == 0))
		return CDS_FT_STATUS_OK;
	if (cursor)
		cur = (struct ft_ord_cell *) cursor;	/* resume AT the cell */
	else
		cur = ft_ord_cell_resolve_ord(forward ?
			&ft->ord_cell_head : &ft->ord_cell_tail);
	while (n < cap && cur) {
		struct ft_ord_cell *loaded, *guess;

		buf[n++] = (const struct cds_ft_cell *) cur;
		loaded = forward ?
			ft_ord_cell_resolve_ord(&cur->ord_next) :
			ft_ord_cell_resolve_ord(&cur->ord_prev);
		guess = (struct ft_ord_cell *) (forward ?
			(uintptr_t) cur + stride :
			(uintptr_t) cur - stride);
		cur = (loaded && cmm_ptr_eq(loaded, guess)) ? guess : loaded;
	}
	*next_cursor = (const struct cds_ft_cell *) cur;
	*count = n;
	return CDS_FT_STATUS_OK;
}

enum cds_ft_status cds_ft_cell_next_batch(struct cds_ft *ft,
		const struct cds_ft_cell *cursor, const struct cds_ft_cell **buf,
		size_t cap, size_t *count, const struct cds_ft_cell **next_cursor)
{
	return ft_cell_batch_dir(ft, cursor, buf, cap, count, next_cursor, true);
}

enum cds_ft_status cds_ft_cell_prev_batch(struct cds_ft *ft,
		const struct cds_ft_cell *cursor, const struct cds_ft_cell **buf,
		size_t cap, size_t *count, const struct cds_ft_cell **next_cursor)
{
	return ft_cell_batch_dir(ft, cursor, buf, cap, count, next_cursor, false);
}

/* Invariant offset of the head-node pointer within the opaque cell. */
size_t cds_ft_cell_node_offset(void)
{
	return offsetof(struct ft_ord_cell, node);
}

/*
 * Materialize a key from an opaque cell handle (the lazy companion to the cell
 * batch).  Same sources as cds_ft_node_get_key -- in-leaf key when configured,
 * else the parent up-walk -- but driven from the CELL, so the up-walk path never
 * touches the external head node.
 */
enum cds_ft_status cds_ft_cell_get_key(const struct cds_ft *ft,
		const struct cds_ft_cell *cell_opaque, uint8_t *result_key,
		size_t result_key_max_len, size_t *result_key_len)
{
	const struct cds_ft_group *group = ft->group;
	struct ft_ord_cell *cell = (struct ft_ord_cell *) cell_opaque;
	const uint8_t *ordinals;
	size_t klen;
	uint8_t scratch[FT_MAX_KEY_LEN];

	if (!cell)
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	if (group->speculative_key_offset_set) {
		const struct cds_ft_node *node = rcu_dereference(cell->node);

		ordinals = (const uint8_t *) node + group->speculative_key_offset;
		klen = (group->key_len != CDS_FT_LEN_VARIABLE) ? group->key_len :
			*(const size_t *) ((const char *) node +
				group->key_len_offset);
	} else {
		size_t max_len = group->max_key_len;

		klen = ft_rebuild_key_upwalk(ft, cell, scratch, max_len);
		ordinals = scratch + (max_len - klen);
	}
	*result_key_len = klen;
	if (klen > result_key_max_len)
		return CDS_FT_STATUS_OVERFLOW_ERROR;
	ft_ordinals_to_key(result_key, ordinals, klen, &group->key_map);
	return CDS_FT_STATUS_OK;
}

bool cds_ft_ordered_list(const struct cds_ft *ft)
{
	return ft->ordered_list;
}

enum cds_ft_status cds_ft_iter_set_cache_mode(struct cds_ft_iter *iter,
		enum cds_ft_iter_cache_mode mode)
{
	switch (mode) {
	case CDS_FT_ITER_CACHED:
		break;
	case CDS_FT_ITER_UNCACHED:
		/*
		 * Switching to uncached mode: any previously cached
		 * path may become stale if the caller drops the RCU
		 * read-side lock, so invalidate it now.  Materialize a
		 * reference-keycopy key first (same as the per-op uncached
		 * auto-invalidate), so the next re-descent reads the saved
		 * key rather than a stale leaf.
		 */
		if (iter->cache_mode == CDS_FT_ITER_CACHED) {
			ft_iter_materialize_key(iter);
			iter->cache_valid = false;
			iter_debug_path_clear(iter);
			iter->path_len = 0;
		}
		break;
	default:
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}
	iter->cache_mode = mode;
	return CDS_FT_STATUS_OK;
}

enum cds_ft_iter_cache_mode cds_ft_iter_get_cache_mode(
		const struct cds_ft_iter *iter)
{
	return iter->cache_mode;
}
