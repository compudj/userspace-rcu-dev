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
#include "fractal-trie-trace.h"

#include "bitmap.h"

#ifndef abs_int
#define abs_int(a)	((int) (a) > 0 ? (int) (a) : -((int) (a)))
#endif

struct cds_ft_group_attr {
	size_t key_len;		/* Fixed key length in bytes, or CDS_FT_LEN_VARIABLE. */
	size_t max_key_len;	/* Maximum key length allowed (bounds key buffers). */
	struct cds_ft_key_map key_map;	/* Per-position key<->ordinal byte remap; see cds_ft_group_attr_set_key_map. */
	unsigned int flags;	/* CDS_FT_FLAG_* creation-time flags. */
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
	bool exclusive;		/* Exclusive (single-writer, no concurrent readers) vs concurrent; see cds_ft_attr_set_exclusive. */
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
 * Node-size tiers (the ft_types[] arrays below) and the tag budget that
 * bounds them.
 *
 * Each node carries a type index naming its tier: the [min_child, max_child]
 * child-count range it serves, its type class, and its size order (node size
 * == 1 << order bytes).  Picking a tier on add/remove is NOT a search of the
 * array: find_nearest_type_index() STEPS from the node's current tier to the
 * adjacent one -- up one tier when a child add exceeds max_child, down one when
 * a remove drops below min_child -- so the common +/- 1-child change moves at
 * most one tier (it loops further only when the child count jumps by more).
 * Each tier's min_child overlaps the previous tier's max_child: an intentional
 * hysteresis that damps reallocation under cyclic add/remove within a node.
 * The top tier has max_child == 256 (a full node); the bottom tier (index 0)
 * holds the smallest nodes, including the 0-child root that is kept alive.
 *
 * The type index is carried in 3 bits of the node pointer's low tag bits, so
 * it has 8 possible values: the real tiers plus NODE_INDEX_NULL (the absent /
 * pruned-node sentinel) -- indices 0..6 real + 7 == NULL on 64-bit, 0..5 real
 * + 6 == NULL on 32-bit.  The full pointer-tag scheme (canonical masks in
 * fractal-trie-internal.h: FT_INTERNAL_MASK / FT_COMPRESSED_MASK) is:
 *
 *   bit0 = 0, bit1 = 0   external (leaf) pointer, or NULL
 *   bit0 = 0, bit1 = 1   compressed (path) node
 *   bit0 = 1             internal node; bits 1-3 carry the 3-bit type index
 *
 * Internal nodes spend the most tag bits -- 1 (internal/external flag) + 3
 * (type index) = 4 low bits -- so every node must be 16-byte aligned to keep
 * those bits free for the tag.  That is why the smallest usable allocation
 * order is 4 (node size 1 << 4 == 16 bytes); compressed and external pointers
 * fit within the same 4-bit budget (bit 1 set, and 0b00, respectively).
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
 * Two-level popcount bitmap.  A node's child-presence bitmap (which of the
 * up-to-256 child slots are occupied) is stored two-level so that mapping an
 * occupied slot to its dense child-pointer index costs a couple of popcounts
 * rather than a scan: a ROOT bitmap (root_bm) sits over a row of SUB-bitmaps
 * (sub_bm).  Each sub_bm holds the presence bits for one contiguous block of
 * slots; the corresponding root_bm bit is set iff that sub_bm has any child.
 * A slot's index is then popcount(sub_bm below the slot) plus the children in
 * all lower non-empty sub-bitmaps (the root_bm summarizes which to add).
 *
 * Layout naming: scan_<root_bits>_<sub_bm_bits>(_max_<N>) -- e.g. scan_16_16
 * is a 16-bit root_bm over 16-bit sub_bms (16 x 16 = 256 slots).  The
 * trailing _max_<N> appears only on the generic-2L variants
 * (scan_16_16_*) where the ptr offset depends on the static
 * sub_bm[] tail length (max_child).  The flat-layout scanners split the
 * 8-bit slot index (a byte) into a high field that selects the root_bm
 * entry and a low field that selects the bit within the sub_bm -- a "5+3
 * byte-split" is 5 high bits + 3 low bits, etc.  scan_32_8 uses a 5+3
 * byte-split with u8 nibbles; scan_64_4 a 6+2 byte-split with u4 nibbles.
 * Both pack root_bm + packed_bms into a fixed 12 B or 16 B header, so the
 * ptr offset is constant (+16 after alignment) and one function serves
 * multiple max_child values.
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

/*
 * 32-bit pointers.  Max-density popcount tiers (max_child inlined into
 * ft_types[] below):
 *   idx 0  scan_16_16_max_5  32 B  hdr 12 B +  5 x 4 B = 32 B exact
 *   idx 1  scan_64_4         64 B  hdr 16 B + 12 x 4 B = 64 B exact
 *                            (flat 6+2; node-size cap 12, bitmap cap 16)
 *   idx 2  popcount_1l      128 B  hdr 32 B + 24 x 4 B = 128 B exact
 *   idx 3  popcount_1l      256 B  hdr 32 B + 56 x 4 B = 256 B exact
 *   idx 4  popcount_1l      512 B  hdr 32 B + 120 x 4 B = 512 B exact
 *   idx 5  pigeon          1024 B  256 x 4 B pointers
 */
static const struct cds_ft_type ft_types[] = {
	[0] = {
		.type_class = FT_POPCOUNT,
		.popcount_2l = true,
		.min_child = 1,
		.max_child = 5, .order = 5, .bitmap = FT_NO_BITMAP,
	},
	[1] = {
		.type_class = FT_POPCOUNT,
		.popcount_2l = true,
		.min_child = 3,
		.max_child = 12, .order = 6, .bitmap = FT_NO_BITMAP,
	},
	[2] = {
		/* 32 B bitmap + 24 x 4 B ptrs = 128 B (order-7). */
		.type_class = FT_POPCOUNT,
		.popcount_1l = true,
		.min_child = 7,
		.max_child = 24, .order = 7, .bitmap = FT_NO_BITMAP,
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
		.max_child = 56, .order = 8, .bitmap = FT_NO_BITMAP },
	[4] = {
		/* 32 B bitmap + 120 * 4 B ptrs = 512 B (order-9). */
		.type_class = FT_POPCOUNT,
		.popcount_1l = true,
		.min_child = 28,
		.bitmap = FT_NO_BITMAP,
		.max_child = 120, .order = 9 },

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
		.max_child = 256, .order = 10, .bitmap = FT_BITMAP },

	[6] = { .type_class = FT_NULL, .min_child = 0, .max_child = 0 /* NULL */, .bitmap = FT_NO_BITMAP },
	/*
	 * Slot 7 padding: FT_TYPE_BITS = 3, so the tag-encodable type_index
	 * range is 0..7.  The 32-bit tier uses only indices 0..6 (real
	 * types + FT_NULL); slot 7 must exist so ft_types[] covers every
	 * tag value the encoder can produce.
	 */
	[7] = { .type_class = FT_NULL, .min_child = 0, .max_child = 0, .bitmap = FT_NO_BITMAP },
};
#else /* !(CAA_BITS_PER_LONG < 64) */
/* 64-bit pointers.  Max-child caps are inlined into ft_types[] below. */

/*
 * scan_32_8 (per-slot 5+3 byte-split, max_child=6): 12-byte popcount
 * header (4B root + 8B packed sub_bms) + 6 x 8-byte pointers into
 * the 64B order-6 node.
 *
 * scan_64_4 (flat 6+2 byte-split, max_child=14): 16-byte popcount
 * header (8B root + 8B packed_bms with 14 x 4-bit sub_bms =
 * 56 bits used) + 14 x 8-byte pointers into the 128B order-7
 * node (16 + 14 * 8 = 128 exactly).
 */

static const struct cds_ft_type ft_types[] = {
	[0] = {
		.type_class = FT_POPCOUNT,
		.popcount_2l = true,
		.min_child = 1, .max_child = 3, .order = 5, .bitmap = FT_NO_BITMAP,
	},
	[1] = {	/* scan_32_8 (per-slot 5+3) */
		.type_class = FT_POPCOUNT,
		.popcount_2l = true,
		.min_child = 3, .max_child = 6, .order = 6, .bitmap = FT_NO_BITMAP,
	},
	[2] = {	/* scan_64_4 (flat 6+2) */
		.type_class = FT_POPCOUNT,
		.popcount_2l = true,
		.min_child = 5, .max_child = 14, .order = 7, .bitmap = FT_NO_BITMAP,
	},
	[3] = {
		.type_class = FT_POPCOUNT,
		.popcount_1l = true,
		.min_child = 10, .max_child = 28, .order = 8, .bitmap = FT_NO_BITMAP,
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
		.max_child = 60,
		.order = 9, .bitmap = FT_NO_BITMAP },
	[5] = {
		/* 32B bitmap + 124 * 8B ptrs = 1024B order-10. */
		.type_class = FT_POPCOUNT,
		.popcount_1l = true,
		.min_child = 51,
		.max_child = 124,
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
	[6] = { .type_class = FT_PIGEON, .min_child = 95, .max_child = 256, .order = 11, .bitmap = FT_BITMAP },

	[7] = { .type_class = FT_NULL, .min_child = 0, .max_child = 256, .bitmap = FT_NO_BITMAP },
};
#endif /* !(BITS_PER_LONG < 64) */

/*
 * The cds_ft_inode contains the compressed node data needed for
 * the read-side traversal. Because the actual layout depends on the
 * node's type_class (Popcount or Pigeon), the struct uses a single
 * pointer-aligned byte array. The allocator sizes this array dynamically
 * based on the type's order (1 << type->order).
 *
 * The allocator guarantees that each node is naturally aligned
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
 * - No key array is stored inside the node -- the key is implicit
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


/*
 * The implementation is split into per-module units, #included below in
 * dependency order into this single translation unit -- so the compiler still
 * inlines across module boundaries (e.g. the descent into each lookup), exactly
 * as when this was one 25k-line file.  FRACTAL_TRIE_IMPL gates each unit against
 * stray standalone inclusion.
 */
#define FRACTAL_TRIE_IMPL
#include "ft-delay.h"
#include "ft-helpers.h"
#include "ft-scanners.h"
#include "ft-descent.h"
#include "ft-lookup.h"
#include "ft-inequality.h"
#include "ft-insert.h"
#include "ft-remove.h"
#include "ft-graft.h"
#include "ft-detach.h"
#include "ft-merge.h"
#include "ft-lifecycle.h"
#include "ft-verify.h"
#include "ft-compact.h"
