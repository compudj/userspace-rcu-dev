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
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <unistd.h>
#include <urcu/list.h>
#include <urcu/rculfhash.h>
#include <urcu/arch.h>
#include <urcu/call-rcu.h>
#include <urcu/uatomic.h>
#include <urcu/fractal-trie.h>	/* enum cds_ft_numa_policy, cds_ft_optimize */
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
 * Pointer tag encoding (bits 0-1):
 *
 *   (ptr & 0b011) == 0b00   →  external node (leaf) or NULL
 *   (ptr & 0b001) == 0b001  →  internal node (bit 0 set), bits 1-3 = type index
 *   (ptr & 0b011) == 0b010  →  compressed path node
 *
 * Internal nodes always have bit 0 set; the type index encoding lives
 * in bits 1-3.  Compressed nodes use bit 1 with bit 0 clear (16-byte
 * alignment).  External nodes have bits 0-1 clear (8-byte aligned).
 */
#define FT_INTERNAL_BITS	1
#define FT_INTERNAL_MASK	(1U << 0)
#define FT_COMPRESSED_MASK	(1U << 1)
#define FT_TAG_MASK		(FT_COMPRESSED_MASK | FT_INTERNAL_MASK)	/* 0b011 — for compressed ptr unmasking */

/*
 * This if followed by a number of bits reserved to represent the child
 * type.
 */
#define FT_TYPE_BITS	3
#define FT_TYPE_MAX_NR	(1UL << FT_TYPE_BITS)
#define FT_TYPE_MASK	((FT_TYPE_MAX_NR - 1) << FT_INTERNAL_BITS)
#define FT_PTR_MASK	(~(FT_TYPE_MASK | FT_INTERNAL_MASK))

/*
 * Internal group flag: skip-compressed pointer encoding.
 *
 * Bit set in struct cds_ft_group::flags when the SPECULATIVE lookup
 * optimization (default; cds_ft_group_attr_set_lookup_optimization)
 * is selected AND the arch supports skip-encoded pointers.  Not
 * part of the public API.
 */
#define CDS_FT_FLAG_SKIP_COMPRESSED	(1U << 0)

/*
 * Skip-compressed pointer encoding.
 *
 * When CDS_FT_FLAG_SKIP_COMPRESSED is set, compressed node pointers
 * are replaced by "skip pointers" that point directly to the
 * compressed node's child, skipping the compressed node on the read
 * fast path (candidate lookup).
 *
 * Encoding: the high bits of a pointer (starting at FT_SKIP_LEN_SHIFT)
 * store the compressed path length.  These bits must be zero in normal
 * userspace pointers.  A non-zero value identifies a skip pointer.
 * The number of available bits (FT_SKIP_LEN_BITS) and thus the
 * maximum encodable path length (FT_SKIP_LEN_MAX) are architecture-
 * dependent.  Compressed paths longer than FT_SKIP_LEN_MAX keep the
 * traditional compressed node pointer (no skip optimization).
 *
 * Per-architecture parameters:
 *   x86-64:    shift=57, bits=7, max path=127  (LA57: 57-bit VA)
 *   AArch64:   shift=56, bits=8, max path=255  (LVA: 52-bit VA)
 *   PPC64:     shift=56, bits=8, max path=255  (Radix: 52-bit VA)
 *   riscv64:   shift=56, bits=8, max path=255  (Sv57: 57-bit VA)
 *   MIPS64:    shift=56, bits=8, max path=255  (48-bit VA)
 *   LoongArch: shift=56, bits=8, max path=255  (48-bit VA)
 *
 * New architectures can be added by defining FT_SKIP_LEN_SHIFT and
 * FT_SKIP_LEN_BITS below, enabling FEATURE_FT_SKIP_COMPRESSED in
 * the architecture gate, and verifying that userspace pointers have
 * the selected bits clear.  Architectures with fewer available high
 * bits can still benefit from skip-compressed with a smaller
 * FT_SKIP_LEN_BITS; the fallback to traditional compressed pointers
 * handles longer paths transparently.
 *
 * The compressed node remains allocated (for key bytes, inequality
 * lookup, exact lookup) and is accessible via the child node's
 * metadata->parent pointer (or cds_ft_node.prev for the head of
 * an external duplicate chain).
 *
 * Dual-pointer RCU publication:
 *
 * A skip pointer and cn->child are two views of the same child
 * pointer.  When cn->child is replaced (recompact, attach), both
 * must be updated.  The update order is:
 *
 *   1. rcu_assign_pointer(*skip_slot, new_skip_ptr)
 *   2. rcu_assign_pointer(cn->child, new_child)  [or *parent_slot]
 *
 * Skip pointer first ensures candidate readers (which follow the
 * skip pointer) immediately see the new child.  Exact and
 * inequality readers (which follow cn->child via the compressed
 * handler) see the old child until step 2.  The old child remains
 * alive until after a grace period.
 *
 * Between steps 1 and 2, the two reader paths see different but
 * individually consistent tree states (old vs. new subtree).  No
 * reader sees a freed node.  This is the standard RCU guarantee:
 * concurrent readers may observe pre-mutation or post-mutation
 * state, never a mix of both within a single traversal.
 *
 * The child's metadata->parent (and cds_ft_node.prev for the head
 * of an external duplicate chain) is read by ft_skip_to_compressed
 * on the read side and written by ft_set_parent on the write side.
 * Both use rcu_dereference / rcu_assign_pointer for proper ordering.
 */

/* Per-architecture skip-length encoding parameters (64-bit only). */
#if defined(URCU_ARCH_AMD64)
# define FT_SKIP_LEN_SHIFT	57	/* Bits 57-63 (7 bits). LA57: 57-bit VA. */
# define FT_SKIP_LEN_BITS	7
#elif defined(URCU_ARCH_AARCH64)
# define FT_SKIP_LEN_SHIFT	56	/* Bits 56-63 (8 bits). LVA: 52-bit VA. */
# define FT_SKIP_LEN_BITS	8
#elif defined(URCU_ARCH_PPC64)
# define FT_SKIP_LEN_SHIFT	56	/* Bits 56-63 (8 bits). Radix: 52-bit VA. */
# define FT_SKIP_LEN_BITS	8
#elif defined(URCU_ARCH_RISCV) && CAA_BITS_PER_LONG >= 64
# define FT_SKIP_LEN_SHIFT	56	/* Bits 56-63 (8 bits). Sv57: 57-bit VA. */
# define FT_SKIP_LEN_BITS	8
#elif defined(URCU_ARCH_MIPS) && CAA_BITS_PER_LONG >= 64
# define FT_SKIP_LEN_SHIFT	56	/* Bits 56-63 (8 bits). 48-bit VA. */
# define FT_SKIP_LEN_BITS	8
#elif defined(URCU_ARCH_LOONGARCH)
# define FT_SKIP_LEN_SHIFT	56	/* Bits 56-63 (8 bits). 48-bit VA. */
# define FT_SKIP_LEN_BITS	8
#endif

#ifdef FT_SKIP_LEN_BITS
# define FT_SKIP_LEN_MAX	((1U << FT_SKIP_LEN_BITS) - 1)
# define FT_SKIP_LEN_MASK	(((unsigned long) FT_SKIP_LEN_MAX) << FT_SKIP_LEN_SHIFT)
# define FT_ADDR_MASK		((1UL << FT_SKIP_LEN_SHIFT) - 1)
#else
/*
 * Fallback on architectures without skip-compressed support (notably
 * 32-bit).  FEATURE_FT_SKIP_COMPRESSED is also undefined in that
 * case, so call sites guarded by ft_group_skip_compressed() short-
 * circuit before evaluating FT_SKIP_LEN_MAX; the fallback value
 * keeps those expressions type-correct at compile time without
 * changing runtime behavior.
 */
# define FT_SKIP_LEN_MAX	0U
#endif

#define FT_ENTRY_PER_NODE	256
#define FT_LOG2_BITS_PER_BYTE	3U
#define FT_BITS_PER_BYTE	(1U << FT_LOG2_BITS_PER_BYTE)

#define FT_MAX_KEY_LEN	256			/* Maximum key length supported. */
#define FT_MAX_DEPTH	(FT_MAX_KEY_LEN + 1)	/* Maximum depth, including root. */

/*
 * Number of bytes of safe over-read past a caller's key_len that the
 * descent SIMD comparator may load.  Library-internal: used to size
 * the iter-allocated key buffer (struct cds_ft_iter) and as the
 * implicit horizon for library-internal key buffers.  Sized for a
 * 32-byte AVX2 load.
 */
#define FT_KEY_READABLE_PAD	32U

/*
 * Entry for NULL node is at index 6 (32-bit) or 7 (64-bit) of the
 * table. It is never encoded in flags.
 */
#if (CAA_BITS_PER_LONG < 64)
# define NODE_INDEX_NULL		6
#else
# define NODE_INDEX_NULL		7
#endif

/*
 * FT_INTERNAL_ORDER_MIN: alloc order of the smallest internal node type
 * (index 0).  Internal-node orders increase by 1 per index (the
 * ft_types[] table is constructed so this holds), so the prefetch fast
 * path can derive an item's order as (type_index + FT_INTERNAL_ORDER_MIN)
 * without loading ft_types[].
 *
 * Distinct from FT_ALLOC_ORDER_MIN, which is the allocator's minimum
 * order (16 B) used by compressed nodes with short key tails.
 */
#define FT_INTERNAL_ORDER_MIN		5

#define FT_ALLOC_ORDER_MAX		12
#define FT_ALLOC_ORDER_MIN		4	/* Minimum item size: 16 bytes. */

/*
 * Maximum page size order supported per architecture.  Used to size the
 * alloc_index bitfield so that the maximum number of items per range
 * (page_size >> FT_ALLOC_ORDER_MIN) fits without truncation.
 *
 * A runtime check in cds_ft_arena_create() rejects page sizes larger
 * than (1 << FT_MAX_PAGE_ORDER).
 */
#if defined(URCU_ARCH_AMD64)
# define FT_MAX_PAGE_ORDER	12	/* x86-64: 4 KiB pages. */
#elif defined(URCU_ARCH_AARCH64)
# define FT_MAX_PAGE_ORDER	16	/* ARM64: up to 64 KiB pages. */
#elif defined(URCU_ARCH_PPC64)
# define FT_MAX_PAGE_ORDER	16	/* POWER: up to 64 KiB pages. */
#else
# define FT_MAX_PAGE_ORDER	16	/* Conservative default: 64 KiB. */
#endif

#define FT_ALLOC_INDEX_BITS	(FT_MAX_PAGE_ORDER - FT_ALLOC_ORDER_MIN)

/*
 * Far-metadata is the default internal-arena layout: it gives the same lookup
 * throughput as the near (page_size-range) layout but a tighter multi-thread
 * tail (fewer outliers) and marginally lower RSS on large tries.  Define
 * FT_NEAR_METADATA to fall back to the original page_size-range layout.
 */
#if !defined(FT_NEAR_METADATA) && !defined(FT_FAR_METADATA)
#define FT_FAR_METADATA
#endif

#ifdef FT_FAR_METADATA
/*
 * FT_FAR_METADATA: the exact near range layout, but with the page unit scaled
 * from page_size (4 KiB) to a 2 MiB macro.  A range becomes:
 *
 *   [ ITEMS  N x 2^order  @base            ]   (N = 2 MiB >> order, one dense
 *   [ header + METADATA[] @base + 2 MiB    ]    2 MiB-aligned run of node bodies)
 *   [ BITMAP grows backward from base + 4 MiB (bitmap arenas only)         ]
 *
 * So the items are one contiguous 2 MiB run (vs the default 4 KiB items page
 * diluted by an interleaved metadata page every 8 KiB), and the metadata array
 * starts at the CONSTANT offset base + 2 MiB — no per-order offset table, the
 * hot-path item->metadata / item->bitmap helpers are the default formulas with
 * page_size replaced by FT_FAR_MACRO_SIZE.
 *
 * Goal: test whether a dense 2 MiB body run + internal-arena THP lets the HW
 * prefetcher's (now unfenced within the 2 MiB) speculative neighbour fetches
 * land on useful node bodies instead of interleaved metadata pages.
 */
#define FT_FAR_MACRO_ORDER	21			/* 2 MiB macro page. */
#define FT_FAR_MACRO_SIZE	(1UL << FT_FAR_MACRO_ORDER)
#define FT_FAR_MACRO_MASK	(FT_FAR_MACRO_SIZE - 1)
#endif /* FT_FAR_METADATA */

#define FT_BITMAP_LEN			32

/*
 * On x86_64 / i386, warn loudly when popcount or BMI/BMI2 ISA flags
 * are missing from the compile.  Several hot-path inlines silently
 * fall back to software emulation when the compiler isn't told the
 * target supports the native instruction:
 *
 *   - __builtin_popcount* on the popcount-node byte-step.
 *   - bzhi-style masked popcount on the popcount_2l byte-step
 *     (scan_6 / scan_16 / scan_32_8 / scan_64_4).
 *   - Generic codegen on bit-manipulation helpers.
 *
 * The fallbacks cost roughly 10-20% on lookup throughput in our
 * microbenches.  Override with -DCDS_FT_SUPPRESS_ISA_WARNING when
 * intentionally building for a stripped-down target.
 */
#if !defined(CDS_FT_SUPPRESS_ISA_WARNING) && \
		(defined(__x86_64__) || defined(__i386__))
# if !defined(__POPCNT__)
#  warning "Fractal trie: building for x86 without -mpopcnt; __builtin_popcount* will use a software fallback. Expect ~10-20% lookup-throughput regression. Add -mpopcnt or -march=native, or define CDS_FT_SUPPRESS_ISA_WARNING to silence."
# endif
# if !defined(__BMI__)
#  warning "Fractal trie: building for x86 without -mbmi; bit-manipulation helpers will use generic codegen. Add -mbmi or -march=native, or define CDS_FT_SUPPRESS_ISA_WARNING to silence."
# endif
# if !defined(__BMI2__)
#  warning "Fractal trie: building for x86 without -mbmi2; bzhi-style masked popcount on the popcount_2l byte-step degrades to a 5-insn fallback (~10% slower on the dns workload). Add -mbmi2 or -march=native, or define CDS_FT_SUPPRESS_ISA_WARNING to silence."
# endif
#endif

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

/*
 * Skip-compressed pointers encode the compressed path length in the
 * high bits of pointers (bits 57-63).  This requires architectures
 * where those bits are guaranteed zero for userspace pointers.
 *
 * Enabled on architectures that define FT_SKIP_LEN_BITS (see
 * per-architecture encoding parameters above).  A runtime
 * validation (mmap probe) at flag-set time rejects the feature
 * if the encoding bits fall within the kernel's VA range.
 *
 * Not supported on s390x (full 64-bit virtual addresses).
 *
 * Requires FEATURE_FT_COMPRESS (skip-compressed is meaningless
 * without compressed nodes).
 *
 * Override with -DNO_FEATURE_FT_SKIP_COMPRESSED to force-disable.
 */
#ifdef NO_FEATURE_FT_COMPRESS
# ifndef NO_FEATURE_FT_SKIP_COMPRESSED
#  define NO_FEATURE_FT_SKIP_COMPRESSED
# endif
#endif
#ifndef NO_FEATURE_FT_SKIP_COMPRESSED
# if defined(FT_SKIP_LEN_BITS)
#  define FEATURE_FT_SKIP_COMPRESSED
# endif
#endif

/*
 * FEATURE_FT_EXCL_VALIDATE: runtime validation of the access-discipline
 * contract.  Writers claim a per-trie owner via atomic CAS at the
 * public API boundary; writer/writer overlap aborts the process with
 * a violation report.
 *
 * Reader validation depends on the trie mode:
 *
 *   Exclusive mode: readers are counted; a writer entering with any
 *   reader present (or a reader entering with a writer present)
 *   aborts.
 *
 *   Concurrent mode: a well-formed reader either holds the RCU
 *   read-side lock or guarantees external mutual exclusion against
 *   mutators.  At reader entry the validator queries the RCU flavor's
 *   read_ongoing(); if true the reader is RCU-protected and may
 *   overlap with a single writer (no-op).  If false the reader is
 *   asserting the mutex-claim path and is treated like an
 *   exclusive-mode reader: a concurrent writer aborts.  The
 *   validator only catches the deterministic case where the bad
 *   interleaving actually occurs in this run; a reader that holds
 *   neither protection but does not overlap with any writer cannot
 *   be detected from a single observation.
 *
 * Off by default (zero overhead).  Enable with -DFEATURE_FT_EXCL_VALIDATE.
 */

/*
 * FEATURE_FT_VERIFY_AT_MUTATION: walk the entire trie at the exit of
 * every public write API (insert / remove / replace / graft / detach)
 * and run cds_ft_verify.  On any invariant mismatch, print a
 * diagnostic to stderr and abort the process.
 *
 * Catches structural / nr_keys / parent-pointer regressions
 * at the mutation that introduced them, instead of via downstream
 * symptoms.  The recursive walk is O(N) per mutation, so this is for
 * testing / debugging only.
 *
 * Off by default (zero overhead).  Enable with -DFEATURE_FT_VERIFY_AT_MUTATION.
 */
struct cds_ft;
#ifdef FEATURE_FT_VERIFY_AT_MUTATION
void ft_writer_scope_verify(struct cds_ft *ft);
#endif

#ifdef FEATURE_INLINE_LOOKUP
#define inline_lookup	inline __attribute__((always_inline))
#else
#define inline_lookup
#endif

/*
 * Spill/reload helpers for breaking the live range of a variable.
 *
 * FT_SPILL_TO_STACK(var) stores @var to its own stack slot through
 * a volatile lvalue, forcing gcc to actually emit the store and to
 * stop assuming any register copy is in sync with memory.  With
 * no normal use of @var until the matching reload, gcc is free to
 * recycle the register that held @var.
 *
 * FT_RELOAD_FROM_STACK(var) is the matching reload — a volatile
 * load from @var's stack slot into a fresh register.
 *
 * The two MUST be paired.  Every code path that reaches the
 * reload must have executed the matching spill first; otherwise
 * the reload returns an uninitialized stack-slot value.
 *
 * Implementation: an empty inline asm with `"=m"`/`"=r"`
 * constraints would declare the memory or register as touched
 * but emit no instructions — gcc's data-flow then folds the
 * pair away and keeps @var live across the gap, defeating the
 * purpose.  The volatile cast emits the actual store and load.
 *
 * Caveat: if the freed register would be useful inside the gap,
 * the gap must not derive its work-state from @var (e.g. via
 * macros that dereference @var), since each such reference would
 * force a reload from the spill slot.  Capture work-state into
 * local pointers BEFORE the spill and refer to those locals
 * inside the gap.
 */
#define FT_SPILL_TO_STACK(var)						\
	do {								\
		*(volatile __typeof__(var) *) &(var) = (var);		\
	} while (0)

#define FT_RELOAD_FROM_STACK(var)					\
	do {								\
		(var) = *(volatile __typeof__(var) *) &(var);		\
	} while (0)

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
 * Struct layout (32 bytes, zero internal padding):
 *   offset  0: 8-byte parent pointer
 *   offset  8: 8-byte external_nodes pointer
 *   offset 16: 8-byte nr_keys (unsigned long, total keys in subtree)
 *   offset 24: 4-byte packed bitfield (nr_child, skip_slot_offset,
 *              alloc_index)
 *   offset 28: 4-byte tail padding
 *
 * In cds_ft_metadata_alloc, rcu_head is a separate field placed
 * before the metadata union — no overlap with metadata fields.
 * All metadata fields remain valid throughout the RCU grace period.
 */
struct cds_ft_metadata {
	/* 8-byte aligned fields. */
	struct cds_ft_inode_flag *parent;	/*
						 * Tagged pointer to parent node.  NULL at
						 * the root node and during the brief window
						 * between a detach / graft_swap clearing the
						 * link and the new placement completing.
						 * Written by the mutation side via
						 * rcu_assign_pointer; read by the read side
						 * (ft_skip_to_compressed, ft_get_parent_rcu)
						 * via rcu_dereference.
						 */
	struct cds_ft_node *external_nodes;	/* List of external nodes at this tree location. */

	/*
	 * Total unique keys in subtree.
	 * Stored with uatomic_store release, loaded with acquire.
	 */
	unsigned long nr_keys;

	/*
	 * Packed bitfield — small fields in a single uint32_t.
	 *
	 * nr_child:               9 bits (max 256)
	 * skip_slot_offset:       8 bits — pointer-stride offset of this
	 *                         node's slot within its parent node body
	 *                         (byte_offset / sizeof(void *)).  Maintained
	 *                         for every internal/compressed node (not just
	 *                         skip-compressed): it lets a backtrack recover
	 *                         the parent slot in O(1) without re-descending.
	 *                         0 (and unused) at the root.
	 * alloc_index:            near: FT_ALLOC_INDEX_BITS + 3 spare bits of
	 *                         headroom above the page_size >>
	 *                         FT_ALLOC_ORDER_MIN minimum; far: a separate
	 *                         uint32_t (see below).
	 */
	uint32_t nr_child:9;
	uint32_t skip_slot_offset:8;
#ifdef FT_FAR_METADATA
	/*
	 * A 2 MiB far macro-block holds far more than 256 items (e.g. ~18 700
	 * order-5 nodes), overflowing the FT_ALLOC_INDEX_BITS (8-bit) packed
	 * field.  Store alloc_index as its own uint32_t — it lands in the
	 * struct's existing 4-byte tail padding, so the struct stays 32 B and
	 * the hot descent bitfield (nr_child/skip_slot_offset) is untouched.
	 */
	uint32_t alloc_index;
#else
	/*
	 * Near metadata: alloc_index is packed into this word, sized with
	 * 3 spare bits of headroom above the FT_ALLOC_INDEX_BITS minimum
	 * needed to index a page-sized range.
	 */
	uint32_t alloc_index:(FT_ALLOC_INDEX_BITS + 3);
#endif
};

/*
 * Compressed path node.  Replaces a chain of single-child internal
 * nodes with a single node storing the key bytes inline.
 *
 * cn->child can point to any node type: internal, compressed,
 * or external.  An external child means a key terminates at the
 * end of the compressed path.  However, the compressed node's
 * metadata->external_nodes must NOT be used — variable-length key
 * entries belong on internal nodes (which can have
 * metadata->external_nodes), or as child pointer of a compressed
 * node.
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

struct cds_ft_bitmap {
	/*
	 * Bitmap is attached to pigeon nodes only.  Pigeon has no key
	 * array and no internal bitmap header -- the bitmap-scan path
	 * needs this metadata bitmap to find populated slots without
	 * a 256-pointer linear scan.  Cost: 2 cache line loads + 1
	 * extra TLB hit, compared to 32 cache line loads worst case
	 * without the bitmap.  Popcount tiers carry their own bitmaps
	 * inline in the node body and do not need a metadata bitmap.
	 *
	 *   Pigeon bitmap memory use (32-bit): 32 B / 1024 B node =  3.1%
	 *   Pigeon bitmap memory use (64-bit): 32 B / 2048 B node =  1.6%
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
	unsigned int flags;		/* CDS_FT_FLAG_* creation-time flags. */
	const struct rcu_flavor_struct *flavor;
	/* Allocation arenas. */
	struct cds_ft_alloc_arena *arena_order[FT_ALLOC_ORDER_MAX + 1];
	/*
	 * Separate arena set for compressed nodes in skip-compressed
	 * groups (speculative=true).  Cand-mode descent never reads
	 * compressed-node bodies (skip-encoded pointers carry skip_len
	 * in the high bits and the descent jumps past the compressed
	 * key bytes), so compressed-node pages would otherwise pollute
	 * the same allocator pages as the internal nodes that ARE on
	 * the descent hot path.  Routing compressed allocations to a
	 * dedicated arena keeps them off the descent's I/D-cache and
	 * TLB working set.  NULL slots fall back to arena_order[] for
	 * groups where speculative is false.
	 */
	struct cds_ft_alloc_arena *compressed_arena_order[FT_ALLOC_ORDER_MAX + 1];
	pthread_mutex_t arena_lock;	/* Protects lazy arena creation. */
	struct cds_ft_key_map key_map;
	unsigned long nr_ft_instances;	/* Number of Fractal Trie instances in the group. */

	/*
	 * @speculative: SPECULATIVE lookup optimization (default).  When
	 *   true, the group's compressed-node allocations are routed to a
	 *   dedicated arena set for improved descent locality.
	 *   Orthogonal to CDS_FT_FLAG_SKIP_COMPRESSED (the skip-encoded
	 *   pointer optimization, which is also gated on
	 *   arch-availability).  See
	 *   cds_ft_group_attr_set_lookup_optimization.
	 */
	bool speculative;
	/*
	 * @numa_policy: NUMA placement policy for the group's internal
	 *   allocator superblocks.  See
	 *   cds_ft_group_attr_set_numa_policy.  Default: INTERLEAVE at
	 *   2 MiB chunk granularity.  Overridden by the env var
	 *   CDS_FT_NUMA_INTERLEAVE=0 to force LOCAL placement (for
	 *   benchmarking without recompiling).
	 */
	enum cds_ft_numa_policy numa_policy;
	/*
	 * @optimize: page-size policy for the internal + compressed node
	 *   arenas (CDS_FT_OPTIMIZE_THROUGHPUT -> 2 MiB, _RSS -> 4 KiB).
	 *   See cds_ft_group_attr_set_optimize.  Default: THROUGHPUT.
	 */
	enum cds_ft_optimize optimize;
};


/*
 * Per-API lookup function pointers cached on struct cds_ft so the
 * public entry points dispatch via one indirect tail-call to a
 * fully-specialized inner (no per-call runtime branch on group
 * shape — the library picks the right inner once at cds_ft_create
 * and caches it here).  See ft_install_lookup_ops in fractal-trie.c.
 *
 * Each pointer's signature matches the corresponding cds_ft_*
 * public API.  The pointer is stable for the lifetime of the trie:
 * group flags (key_map.identity, CDS_FT_FLAG_SKIP_COMPRESSED bit) are
 * immutable after group creation, so a single load + indirect jmp on
 * the hot path is all the dispatch cost vs the ~22 ns regression seen
 * from runtime-gated dispatch (see [[ft-lookup-inline-keep]]).
 */
struct cds_ft;
struct cds_ft_node;
struct cds_ft_iter;
typedef enum cds_ft_status (*cds_ft_lookup_key_fn)(
		struct cds_ft *ft, const uint8_t *key,
		size_t key_len, size_t key_readable_pad,
		struct cds_ft_node **result_node);
typedef enum cds_ft_status (*cds_ft_lookup_iter_fn)(
		struct cds_ft *ft, struct cds_ft_iter *iter);
/*
 * Signature shared by cds_ft_lookup_partial_key and
 * cds_ft_lookup_longest_match_key: returns the partial/longest match
 * length via @match_len and the matched node via @result_node.
 */
typedef enum cds_ft_status (*cds_ft_lookup_prefix_key_fn)(
		struct cds_ft *ft, const uint8_t *key,
		size_t key_len, size_t *match_len,
		struct cds_ft_node **result_node);

struct cds_ft_compact_state;

struct cds_ft {
	struct cds_ft_group *group;

	struct cds_ft_inode_flag *root;		/* Root node (arena-allocated, always present, always internal). */

	/*
	 * Specialized lookup dispatch pointers — installed at
	 * cds_ft_create based on group flags.  See the typedef
	 * comment above.  Placed near @root so a single cache-line
	 * fetch on lookup entry serves the descent.
	 */
	cds_ft_lookup_key_fn lookup_key_fn;
	cds_ft_lookup_key_fn lookup_candidate_key_fn;
	cds_ft_lookup_iter_fn lookup_iter_fn;
	cds_ft_lookup_prefix_key_fn lookup_partial_key_fn;
	cds_ft_lookup_iter_fn lookup_partial_iter_fn;
	cds_ft_lookup_prefix_key_fn lookup_longest_match_key_fn;
	cds_ft_lookup_iter_fn lookup_longest_match_iter_fn;

	size_t max_used_key_len;		/* Maximum key length inserted (conservative). */

	/*
	 * Access discipline. When true, access is serialized
	 * externally (single-threaded or mutex-protected) and no
	 * concurrent RCU readers exist; mutation operations that
	 * would otherwise require a grace period to re-parent a
	 * subtree (graft, graft_swap) may skip synchronize_rcu().
	 * When false, concurrent RCU readers are permitted.
	 */
	bool exclusive;

	/*
	 * In-progress compaction state (cds_ft_compact_begin), or NULL.
	 * Set at begin, cleared at end.  Lets cds_ft_compact_begin reject a
	 * second concurrent compaction on the same trie, and cds_ft_destroy
	 * finalize one the caller forgot to end (merging its private ranges
	 * back so they are not leaked).  Written under the caller's writer
	 * exclusion, like every other mutation.
	 */
	struct cds_ft_compact_state *active_compact;

#ifdef FEATURE_FT_EXCL_VALIDATE
	/*
	 * Access-discipline validator state.  @excl_owner holds the
	 * pthread_self() of the thread currently inside a writer API
	 * (claimed via atomic CAS), or 0 when no writer is active.
	 * @excl_writer_depth is a reentry depth counter, accessed only
	 * by the owning thread.  @excl_nr_readers counts readers that
	 * are currently inside a reader API on an exclusive-mode trie;
	 * concurrent-mode readers do not touch it (RCU handles them).
	 */
	unsigned long excl_owner;
	unsigned long excl_writer_depth;
	unsigned long excl_nr_readers;
#endif

	/* For debugging */
	unsigned long nr_nodes_allocated, nr_nodes_freed;
	unsigned long nr_internal_alloc, nr_internal_freed;
	unsigned long nr_compressed_alloc, nr_compressed_freed;

#ifdef FEATURE_FT_VERIFY_AT_MUTATION
	/*
	 * Verify-at-mutation sampling.  ft_writer_scope_verify runs
	 * the full O(N) cds_ft_verify walk once every
	 * @verify_at_mutation_period mutations.
	 * @verify_at_mutation_counter increments on every writer-scope
	 * exit and is reset to 0 each time the period is reached, so
	 * it never exceeds @verify_at_mutation_period - 1 (no overflow
	 * concerns even on long-running workloads).  Period 1
	 * reproduces the historical "every mutation" cadence; larger
	 * periods are useful on large tries where O(N) per mutation
	 * is impractical.  Period 0 disables the walk entirely.  Both
	 * fields are write-side only (mutex-held during the writer
	 * scope), so plain accesses are safe.
	 */
	unsigned long verify_at_mutation_period;
	unsigned long verify_at_mutation_counter;
#endif
};

/*
 * Access-discipline validator.  See FEATURE_FT_EXCL_VALIDATE above.
 *
 * The helpers form two nested pairs: ft_excl_writer_enter / _exit
 * around every writer API body, and ft_excl_reader_enter /
 * ft_excl_reader_scope_exit around every reader API body.  The
 * CDS_FT_SCOPED_{READER,WRITER}(ft) macros wrap the pair in a GCC
 * cleanup-attribute variable so any return path in the body
 * automatically runs the matching _exit.
 *
 * Reader entry returns a per-call scope (struct ft_excl_reader_scope)
 * that records whether the reader claimed the trie's reader counter.
 * In concurrent mode a reader that holds the RCU read-side lock at
 * entry does NOT claim (it may safely overlap a writer); a reader
 * that does not is treated like an exclusive-mode reader and claims.
 *
 * When FEATURE_FT_EXCL_VALIDATE is off the helpers are empty and
 * the compiler inlines them away.
 */

struct ft_excl_reader_scope {
	struct cds_ft *ft;
#ifdef FEATURE_FT_EXCL_VALIDATE
	bool claimed;	/* true iff we incremented ft->excl_nr_readers. */
#endif
};

#ifdef FEATURE_FT_EXCL_VALIDATE

__attribute__((noreturn, format(printf, 1, 2)))
void ft_excl_abort(const char *fmt, ...);

static inline
void ft_excl_writer_enter(struct cds_ft *ft)
{
	unsigned long self = (unsigned long) pthread_self();
	unsigned long prev, nr;

	prev = uatomic_cmpxchg(&ft->excl_owner, 0, self);
	if (prev != 0) {
		if (prev == self) {
			/* Reentry from the same thread (e.g. graft_swap
			 * delegating to graft). */
			ft->excl_writer_depth++;
			return;
		}
		ft_excl_abort("cds_ft=%p: writer conflict — owner 0x%lx, entering thread 0x%lx\n",
			(void *) ft, prev, self);
	}
	ft->excl_writer_depth = 1;
	/*
	 * A non-zero reader count means a reader is asserting mutual
	 * exclusion against us: in exclusive mode every reader claims;
	 * in concurrent mode only readers that did not hold the RCU
	 * read-side lock at entry claim.  Either way, overlap is a
	 * contract violation.
	 */
	nr = uatomic_load(&ft->excl_nr_readers, CMM_ACQUIRE);
	if (nr != 0)
		ft_excl_abort("cds_ft=%p: writer 0x%lx entering with %lu concurrent reader(s) (%s mode)\n",
			(void *) ft, self, nr,
			ft->exclusive ? "exclusive" : "concurrent without RCU read-side lock");
}

static inline
void ft_excl_writer_exit(struct cds_ft *ft)
{
	if (--ft->excl_writer_depth == 0)
		uatomic_store(&ft->excl_owner, 0, CMM_RELEASE);
}

static inline
struct ft_excl_reader_scope ft_excl_reader_enter(struct cds_ft *ft)
{
	struct ft_excl_reader_scope scope = { .ft = ft, .claimed = false };
	unsigned long owner;

	if (!ft->exclusive) {
		/*
		 * Concurrent mode: a well-formed reader either holds
		 * the RCU read-side lock or guarantees external mutual
		 * exclusion against mutators.  RCU-protected readers
		 * may overlap a single writer, so they're a no-op here.
		 * Mutex-claim readers fall through and are validated
		 * just like exclusive-mode readers.
		 */
		if (ft->group->flavor->read_ongoing())
			return scope;
	}
	scope.claimed = true;
	uatomic_add(&ft->excl_nr_readers, 1);
	owner = uatomic_load(&ft->excl_owner, CMM_ACQUIRE);
	if (owner != 0 && owner != (unsigned long) pthread_self())
		ft_excl_abort("cds_ft=%p: reader 0x%lx entering with writer 0x%lx active (%s mode)\n",
			(void *) ft, (unsigned long) pthread_self(), owner,
			ft->exclusive ? "exclusive" : "concurrent without RCU read-side lock");
	return scope;
}

static inline
void ft_excl_reader_exit_scope(const struct ft_excl_reader_scope *scope)
{
	if (scope->claimed)
		uatomic_sub(&scope->ft->excl_nr_readers, 1);
}

#else /* !FEATURE_FT_EXCL_VALIDATE */

static inline void ft_excl_writer_enter(struct cds_ft *ft) { (void) ft; }
static inline void ft_excl_writer_exit(struct cds_ft *ft)  { (void) ft; }
static inline
struct ft_excl_reader_scope ft_excl_reader_enter(struct cds_ft *ft)
{
	struct ft_excl_reader_scope scope = { .ft = ft };

	return scope;
}
static inline
void ft_excl_reader_exit_scope(const struct ft_excl_reader_scope *scope)
{
	(void) scope;
}

#endif /* FEATURE_FT_EXCL_VALIDATE */

static inline
void ft_excl_writer_scope_exit(struct cds_ft **ft)
{
#ifdef FEATURE_FT_VERIFY_AT_MUTATION
	/*
	 * Verify before releasing the writer claim so a concurrent
	 * writer cannot start mutating while we walk the trie.
	 */
	ft_writer_scope_verify(*ft);
#endif
	ft_excl_writer_exit(*ft);
}
static inline
void ft_excl_reader_scope_exit(struct ft_excl_reader_scope *scope)
{
	ft_excl_reader_exit_scope(scope);
}

/*
 * Two-level token-paste so __COUNTER__ is expanded before the
 * concatenation, producing a unique variable name for every use.
 * This allows multiple CDS_FT_SCOPED_* per function body (e.g. the
 * two-ft graft / graft_swap paths).
 */
#define CDS_FT_CAT2_(a, b) a ## b
#define CDS_FT_CAT_(a, b) CDS_FT_CAT2_(a, b)

#define CDS_FT_SCOPED_WRITER(ft)					\
	struct cds_ft *CDS_FT_CAT_(_ft_excl_scope_, __COUNTER__)	\
		__attribute__((unused,					\
			cleanup(ft_excl_writer_scope_exit))) =		\
		(ft_excl_writer_enter(ft), (ft))

#define CDS_FT_SCOPED_READER(ft)					\
	struct ft_excl_reader_scope CDS_FT_CAT_(_ft_excl_scope_, __COUNTER__) \
		__attribute__((unused,					\
			cleanup(ft_excl_reader_scope_exit))) =		\
		ft_excl_reader_enter(ft)

/*
 * Allocator layout (see fractal-trie-alloc.c for the full picture).
 *
 * Each 2*page_size range holds the items array (page 0), then the
 * per-range header (struct cds_ft_alloc_range) followed by the
 * per-item metadata array (page 1).  Optional bitmap array grows
 * backward from (range base + 2*page_size) on bitmap-bearing arenas.
 *
 * The two cache lines most often prefetched from a tagged child
 * pointer — the item's metadata and (for bitmap types) its bitmap —
 * are derivable with pure pointer arithmetic given the item's order.
 * The helpers below live in this header so that the prefetch-hint
 * path can compute their addresses inline, without a cross-TU call
 * into fractal-trie-alloc.c.
 *
 * struct cds_ft_alloc_arena remains opaque here: only out-of-line
 * slow paths (cds_ft_item_to_metadata, cds_ft_item_order,
 * cds_ft_metadata_to_item) need its fields, and those stay in
 * fractal-trie-alloc.c.
 */
struct cds_ft_alloc_arena;

struct cds_ft_metadata_alloc {
	struct rcu_head rcu_head;
	union {
		struct cds_ft_metadata_alloc *free_list_next;
		struct cds_ft_metadata metadata;
	};
};

struct cds_ft_alloc_range {
	struct cds_list_head node;			/* Linked list of ranges. */
	struct cds_ft_alloc_arena *arena;		/* Backward reference to arena. */
	size_t next_unused;
	/*
	 * Number of currently-live (allocated, not free-listed) items in
	 * this range.  Incremented in cds_ft_arena_alloc (both the
	 * free-list reuse and bump paths) and decremented in
	 * cds_ft_do_free_item, all under arena->lock.  A range whose
	 * nr_live reaches 0 holds no live nodes and is a candidate for
	 * reclaiming its backing pages to the OS.
	 */
	size_t nr_live;
	/*
	 * Per-range LIFO freelist of returned slots, threaded through
	 * cds_ft_metadata_alloc::free_list_next.  A per-range (rather than
	 * per-arena) freelist lets a drained range be reclaimed in O(1):
	 * its freed slots leave the allocatable set together with the
	 * range, with no shared-list walk and no dangling entries.  The
	 * range is linked onto arena->partial_ranges via @partial_node
	 * exactly while @free_list_head != NULL.
	 */
	struct cds_ft_metadata_alloc *free_list_head;
	struct cds_list_head partial_node;
	/*
	 * Set while this range is a private destination of an in-progress
	 * cds_ft_compact (allocated through the recompaction context, not yet
	 * merged into the arena).  The compactor reads it to tell, in O(1),
	 * whether a node it is descending past has already been relocated this
	 * pass (so it relocates each node exactly once).  Cleared when the
	 * range is merged back into the arena's general pool.
	 */
	bool recompact_private;

	struct cds_ft_metadata_alloc metadata[];
};

/*
 * Architectures with a fixed kernel page size: declare the size as a
 * compile-time constant so cds_ft_get_page_size() folds away, and the
 * mask/shift arithmetic in the inline helpers below collapses into
 * immediate operands.  Runtime validation in cds_ft_arena_create()
 * rejects a mismatched kernel page size.
 */
#if defined(URCU_ARCH_X86) || defined(URCU_ARCH_S390)
# define FT_PAGE_SIZE_FIXED	4096UL
#endif

__attribute__((visibility("hidden")))
extern size_t cds_ft_page_size;

static inline size_t cds_ft_get_page_size(void)
{
#ifdef FT_PAGE_SIZE_FIXED
	return FT_PAGE_SIZE_FIXED;
#else
	return cds_ft_page_size;
#endif
}

/*
 * FT_FAR_METADATA scales the layout's page unit from page_size to a 2 MiB
 * macro: items fill [base, base+2MiB), the range header + metadata[] start at
 * base+2MiB, and bitmaps grow backward from base+4MiB.  The helpers are the
 * default formulas with cds_ft_get_page_size() replaced by FT_FAR_MACRO_SIZE.
 */
#ifdef FT_FAR_METADATA
# define FT_RANGE_PAGE_UNIT	FT_FAR_MACRO_SIZE
#else
# define FT_RANGE_PAGE_UNIT	cds_ft_get_page_size()
#endif

static inline
struct cds_ft_alloc_range *cds_ft_item_to_range(void *p)
{
	size_t pg = FT_RANGE_PAGE_UNIT;
	void *base = (void *)((unsigned long) p & ~(pg - 1));

	return (struct cds_ft_alloc_range *) ((char *) base + pg);
}

static inline
struct cds_ft_metadata *cds_ft_item_to_metadata_fast(void *p, size_t item_len_order)
{
	struct cds_ft_alloc_range *range = cds_ft_item_to_range(p);
	size_t page_offset = (unsigned long) p & (FT_RANGE_PAGE_UNIT - 1);
	size_t index = page_offset >> item_len_order;

	return &range->metadata[index].metadata;
}

/*
 * bitmap array is indexed backwards from range base + (2 * page unit).
 */
static inline
struct cds_ft_bitmap *cds_ft_item_to_bitmap(void *p, size_t item_len_order)
{
	size_t pg = FT_RANGE_PAGE_UNIT;
	void *base = (void *)((unsigned long) p & ~(pg - 1));
	size_t index = ((unsigned long) p & (pg - 1)) >> item_len_order;

	return (struct cds_ft_bitmap *) ((char *) base + (2 * pg) -
			((index + 1) * sizeof(struct cds_ft_bitmap)));
}

__attribute__((visibility("hidden")))
void cds_ft_free_all_arenas(struct cds_ft_group *ft_group);

__attribute__((visibility("hidden")))
struct cds_ft_metadata *cds_ft_item_to_metadata(void *p);

__attribute__((visibility("hidden")))
size_t cds_ft_item_order(void *p);

__attribute__((visibility("hidden")))
void *cds_ft_metadata_to_item(struct cds_ft_metadata *metadata);

__attribute__((visibility("hidden")))
struct cds_ft_metadata *cds_ft_alloc_item(struct cds_ft *ft, size_t item_len_order, bool bitmap);

/*
 * Same as cds_ft_alloc_item but routes to the per-group compressed-
 * node arena set when the group is speculative (skip-compressed).
 * Compressed-node pages are off the descent hot path in that mode,
 * so keeping them out of the internal-node arenas reduces I-cache /
 * D-cache / TLB interference for cand-mode lookups.
 */
__attribute__((visibility("hidden")))
struct cds_ft_metadata *cds_ft_alloc_compressed_item(struct cds_ft *ft, size_t item_len_order);

__attribute__((visibility("hidden")))
void cds_ft_free_item(struct cds_ft *ft, struct cds_ft_metadata *metadata);

/*
 * cds_ft_free_item_unpublished - immediate free for items that were
 * never published (no reader can possibly hold a reference).  Bypasses
 * call_rcu and returns the slot directly to the arena free list.
 *
 * Use only for nodes that never escaped the writer's stack (i.e.,
 * speculative candidate nodes built but rejected before publication).
 * Calling this on a published node corrupts concurrent readers.
 */
__attribute__((visibility("hidden")))
void cds_ft_free_item_unpublished(struct cds_ft *ft, struct cds_ft_metadata *metadata);

/*
 * Recompaction allocation context (cds_ft_compact).  While active on the
 * recompacting thread (set via ft_recompact_alloc_begin), cds_ft_arena_alloc
 * routes that thread's allocations into per-order private fresh ranges held
 * here -- kept off the arena's general range lists so the relocated nodes
 * pack densely, and concurrent unrelated allocations neither land among them
 * nor get forced to bump.  ft_recompact_alloc_end splices the private ranges
 * into their arenas and clears the context.  @cur is indexed by the arena's
 * item_len_order.
 */
struct ft_recompact_alloc_ctx {
	/*
	 * Per-order current private range, separately for the internal-node
	 * arenas (@cur) and the dedicated compressed-node arena (@cur_compressed),
	 * since the two share item-length orders but are distinct arenas.
	 */
	struct cds_ft_alloc_range *cur[FT_ALLOC_ORDER_MAX + 1];
	struct cds_ft_alloc_range *cur_compressed[FT_ALLOC_ORDER_MAX + 1];
	struct cds_list_head all;
};

/* Initialize a context (no ranges yet, not active). */
__attribute__((visibility("hidden")))
void ft_recompact_alloc_init(struct ft_recompact_alloc_ctx *ctx);

/*
 * Route this thread's subsequent internal-node allocations into @ctx's private
 * ranges (pass NULL to stop routing).  Called around each compaction step so
 * the caller's other mutations between steps allocate normally.
 */
__attribute__((visibility("hidden")))
void ft_recompact_alloc_set_active(struct ft_recompact_alloc_ctx *ctx);

/*
 * Splice @ctx's private ranges into their arenas' general pools and clear
 * their recompact_private flag.  Called once when compaction completes.
 */
__attribute__((visibility("hidden")))
void ft_recompact_alloc_merge(struct ft_recompact_alloc_ctx *ctx);

/* True if @metadata's item lives in a range currently flagged recompact_private. */
__attribute__((visibility("hidden")))
bool cds_ft_metadata_in_recompact_private(struct cds_ft_metadata *metadata);

//#define DEBUG
//#define DEBUG_COUNTERS
//#define DEBUG_CLEAR_ITER

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

/*
 * Tracepoint node-kind identifiers.
 *
 * These values are exposed as an LTTng-UST enumeration in src/ft_tp.h
 * (via the LTTNG_UST_TRACEPOINT_ENUM declaration which references
 * these C enum labels).  Keeping both in the same enum guarantees the
 * C side and the trace-metadata side cannot drift out of sync.
 *
 * Labels describe the underlying structure without requiring the
 * reader to know the build's pointer width:
 *   - P2L_<bytes>: popcount_2l (2-level root_bm + sub_bm[] popcount
 *     layout; sub_bm bit width varies by tier and target pointer
 *     width); total node size in bytes.
 *   - P1L_<bytes>: popcount_1l (32-byte 256-bit bitmap +
 *     ptr-table); total node size in bytes.
 *   - PIGEON_<bytes>: 256-entry direct table; total node size in
 *     bytes (1024 on 32-bit, 2048 on 64-bit).
 *
 * Skip-compression is an orthogonal property (a skip-compressed
 * pointer can point to any underlying node type), so it is not
 * encoded in this enum.  Tracepoints that emit node_kind expose a
 * companion "skip_compressed" boolean field alongside it.
 *
 * The superset listed here covers every variant realizable by any
 * 32-bit or 64-bit build configuration; a given build may not emit
 * all of them.
 */
enum ft_tp_node_kind {
	FT_TP_NODE_NULL			=  0,
	FT_TP_NODE_EXTERNAL		=  1,
	FT_TP_NODE_COMPRESSED		=  2,
	FT_TP_NODE_P2L_32		=  5,
	FT_TP_NODE_P2L_64		=  6,
	FT_TP_NODE_P2L_128		=  7,
	FT_TP_NODE_P1L_128		=  4,
	FT_TP_NODE_P1L_256		=  8,
	FT_TP_NODE_P1L_512		=  9,
	FT_TP_NODE_P1L_1024		= 10,
	FT_TP_NODE_PIGEON_1024		= 11,
	FT_TP_NODE_PIGEON_2048		= 12,
	FT_TP_NODE_UNKNOWN		= 13,
};

#endif /* _URCU_FT_INTERNAL_H */
