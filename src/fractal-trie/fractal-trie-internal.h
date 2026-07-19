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

/*
 * ============================ Build options ============================
 *
 * Compile-time toggles for the Fractal Trie.  This is the index; the
 * rationale and exact semantics of each option live at its definition
 * site further down (or in the noted file).  None of these change the
 * public API or the on-the-wire contract -- they trade implementation
 * features, memory layout, or diagnostics.
 *
 * Functional features (enabled by default; disable for a simpler trie):
 *
 *   FEATURE_FT_COMPRESS         Prefix (path) compression of single-child
 *                               chains.  Disable with -DNO_FEATURE_FT_COMPRESS.
 *   FEATURE_FT_SKIP_COMPRESSED  Pack the skip length + child pointer into
 *                               the parent slot's high bits so a descent
 *                               bypasses the compressed cache line.  Auto
 *                               on 64-bit arches that define FT_SKIP_LEN_BITS
 *                               and pass the runtime VA probe; requires
 *                               FEATURE_FT_COMPRESS.
 *                               Disable with -DNO_FEATURE_FT_SKIP_COMPRESSED.
 *   FEATURE_INLINE_LOOKUP       Force-inline the lookup hot path (no call
 *                               boundaries on descent).
 *                               Disable with -DNO_FEATURE_INLINE_LOOKUP.
 *
 * (The library-owned ordered-cell index is always compiled in; it is
 * gated per group at runtime via cds_ft_group_attr_set_ordered_list,
 * not at build time.)
 *
 * Memory layout (architecture-defaulted; override to force):
 *
 *   FT_NEAR_METADATA            page_size-range item/metadata striding;
 *                               default on 64-bit.
 *                               Force selection with -DFT_NEAR_METADATA.
 *   FT_FAR_METADATA             2 MiB macro-range layout; default on 32-bit.
 *                               Force selection with -DFT_FAR_METADATA.
 *
 * Validation and debugging (disabled by default; testing only, non-trivial
 * overhead):
 *
 *   FEATURE_FT_VERIFY_AT_MUTATION  cds_ft_verify the whole trie at the exit
 *                                  of every public write.
 *                                  Enable with -DFEATURE_FT_VERIFY_AT_MUTATION.
 *   FEATURE_FT_EXCL_VALIDATE       Runtime check of the writer/reader
 *                                  access-discipline contract.
 *                                  Enable with -DFEATURE_FT_EXCL_VALIDATE.
 *   DEBUG_COUNTERS                 Group-scoped node / cell alloc balance
 *                                  accounting.
 *                                  Enable with -DDEBUG_COUNTERS.
 *   DEBUG / DEBUG_CLEAR_ITER       Verbose dbg_printf / poison recycled
 *                                  iterator state.
 *                                  Enable with -DDEBUG, -DDEBUG_CLEAR_ITER.
 *
 * Diagnostics and tracing:
 *
 *   CDS_FT_SUPPRESS_ISA_WARNING    Silence the x86 -mpopcnt / -mbmi build
 *                                  #warnings.
 *                                  Enable with -DCDS_FT_SUPPRESS_ISA_WARNING.
 *   FT_ENABLE_TRACING              Emit LTTng-UST tracepoints from the read /
 *                                  mutation paths (see fractal-trie.c).
 *                                  Enable with -DFT_ENABLE_TRACING.
 * =======================================================================
 */

#include <pthread.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <unistd.h>
#include <urcu/list.h>
/*
 * FT parks a concurrent MCAS proxy (struct urcu_mcas_record *, 16-byte
 * aligned -> low 4 bits free) under its own type-7 / 0xF pointer tag (see
 * FT_FLIP_PROXY_TAG in ft-helpers.h) rather than the engine's bit-0 default:
 * bit 0 alone marks an internal node here, so a bit-0 proxy would alias one,
 * whereas a 0xF low nibble is an encoding no real node, NULL or skip pointer can
 * carry.  The tag is now carried PER RECORD -- FT passes FT_FLIP_PROXY_TAG to
 * every urcu_txn_store()/load_validate() (see ft_flip_txn_commit), the engine
 * stores it in urcu_mcas_record.proxy_tag and forms the parked proxy as
 * (record | proxy_tag), and the read hot paths recognise it with the SAME
 * ft_node_flip_proxy() low-nibble test they already run.  No per-TU macro
 * override of the engine is needed.
 */
/*
 * FT is a flavor-agnostic library: it brackets its RCU read-side sections
 * through the group's RCU flavor bound at RUNTIME.  An op-scoped persistent
 * txn handle binds that flavor (ft_txn_op_init -> urcu_txn_init_flavor), so
 * urcu_txn_begin() / urcu_txn_end() open the section in it directly (doc
 * §11: the FT owns the bracket).  An EXCLUSIVE trie binds no flavor -- there
 * is no reader to defend and frees are synchronous -- so its begin()/end()
 * fall back to the URCU_TXN_RCU_READ_LOCK macros: override them to no-ops so
 * the exclusive bracket opens nothing and no compile-time flavor's
 * rcu_read_lock symbol is bound into the flavor-agnostic build.
 */
#define URCU_TXN_RCU_READ_LOCK()	do { } while (0)
#define URCU_TXN_RCU_READ_UNLOCK()	do { } while (0)
#include <urcu/fair-mutex.h>	/* MW coarse lock-mode FT-wide writer lock */
#include <urcu/rcu-txn.h>
#include <urcu/rcu-txn-list.h>
#include <urcu/rculfhash.h>
#include <urcu/arch.h>
#include <urcu/call-rcu.h>
#include <urcu/uatomic.h>
#include <urcu/fractal-trie.h>	/* enum cds_ft_numa_policy, cds_ft_optimize */
#include <assert.h>

/*
 * FT-wide-lock DROP is the DEFAULT for a FINE-locking trie (§11 rollout,
 * 2026-07-19).  A FINE trie skips the FT-wide writer mutex and relies solely
 * on its per-node COPYING lock-sets + MCAS arbitration for writer exclusion --
 * so disjoint writers run in parallel instead of serialising on one mutex
 * (doc/design/ft-wide-lock-drop-mechanics.md; certified by the §11.4 point-op
 * 1600/1600 gate and the cross-trie oracles).  COARSE / OPTIMISTIC tries are
 * unaffected (COARSE keeps the mutex; OPTIMISTIC never took it).
 *
 * Build with -DFEATURE_FT_MW_LOCK_FINE_KEEP to OPT OUT (retain the FT-wide
 * mutex under FINE) -- reversible, for bisecting a regression to the drop.
 * An explicit -DFEATURE_FT_MW_LOCK_FINE_DROP is still honoured (idempotent).
 * This header is included first in the TU (before the impl headers), so the
 * definition reaches every `#ifdef FEATURE_FT_MW_LOCK_FINE_DROP` use site.
 */
#ifndef FEATURE_FT_MW_LOCK_FINE_KEEP
# ifndef FEATURE_FT_MW_LOCK_FINE_DROP
#  define FEATURE_FT_MW_LOCK_FINE_DROP 1
# endif
#endif

/*
 * Internal sentinel returned by ft_key_len() when a key-length argument
 * cannot be resolved (e.g. CDS_FT_LEN_DEFAULT passed to a variable-length
 * group).  Numerically equal to the public CDS_FT_LEN_DEFAULT /
 * CDS_FT_LEN_VARIABLE sentinels; it is distinguished by being an internal
 * resolution *result* rather than a caller-supplied argument.
 */
#define CDS_FT_LEN_ERROR	SIZE_MAX

/*
 * If the internal bit is set in a pointer, it points to an internal
 * Fractal Trie node, else it points to a node outside of the Fractal Trie.
 * This can be used for variable length keys to identify the end of key.
 */
/*
 * Pointer tag encoding (bits 0-1):
 *
 *   (ptr & 0b011) == 0b00   ->  external node (leaf) or NULL
 *   (ptr & 0b001) == 0b001  ->  internal node (bit 0 set), bits 1-3 = type index
 *   (ptr & 0b011) == 0b010  ->  compressed path node
 *
 * Internal nodes always have bit 0 set; the type index encoding lives
 * in bits 1-3.  Compressed nodes use bit 1 with bit 0 clear (16-byte
 * alignment).  External nodes have bits 0-1 clear (8-byte aligned).
 */
#define FT_INTERNAL_BITS	1
#define FT_INTERNAL_MASK	(1U << 0)
#define FT_COMPRESSED_MASK	(1U << 1)
#define FT_TAG_MASK		(FT_COMPRESSED_MASK | FT_INTERNAL_MASK)	/* 0b011 -- for compressed ptr unmasking */

/*
 * This is followed by a number of bits reserved to represent the child
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
 * This ordered two-step publish is for SINGLE-EDGE replacement.  A
 * multi-edge commit that includes this edge (cds_ft_merge_at) instead
 * switches the slot through the flip latch (FT_FLIP_PROXY, see
 * <urcu/rcu-txn-sw.h>): the slot transiently holds a flip proxy, one
 * commit flips the whole set old->new atomically, and the skip view is
 * re-established when the flip settles.
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
 *
 * "Page size" here is the kernel BASE page size (urcu_get_page_len),
 * never a huge page.  Transparent huge pages are orthogonal: THP only
 * remaps the same frames at a coarser TLB granularity, so it changes
 * neither cds_ft_get_page_size() nor the items-per-range count.  (The
 * 2 MiB FT_FAR_METADATA layout is a separate compile-time path that
 * sizes alloc_index independently; it does not use FT_MAX_PAGE_ORDER.)
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
# if (CAA_BITS_PER_LONG < 64)
/*
 * 32-bit: FT_FAR carves the address space into 2 MiB macro ranges, reserving
 * virtual address space in 2 MiB units even though each range faults in
 * lazily.  That is too VA-hungry for a ~3 GiB user address space, so default
 * to the page_size-range near layout there.
 */
#  define FT_NEAR_METADATA
# else
#  define FT_FAR_METADATA
# endif
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
 * starts at the CONSTANT offset base + 2 MiB -- no per-order offset table, the
 * hot-path item->metadata / item->bitmap helpers are the default formulas with
 * page_size replaced by FT_FAR_MACRO_SIZE.
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
 * FEATURE_INLINE_LOOKUP: force-inline the lookup hot-path helpers --
 * inline_lookup expands to inline __attribute__((always_inline)), so the
 * descent dispatches through no call boundaries.  Disabling lets the
 * compiler choose, trading lookup latency for smaller code.
 *
 * Enabled by default.  Disable with -DNO_FEATURE_INLINE_LOOKUP.
 */
#ifndef NO_FEATURE_INLINE_LOOKUP
# define FEATURE_INLINE_LOOKUP
#endif

/*
 * FEATURE_INLINE_INEQUALITY_LOOKUP: also force-inline (and per-mode specialize)
 * the inequality descent -- the relational seek and its up/down tree traversal.
 *
 * OFF by default (opt-in).  The ordered-cell list is the fast path for ordered
 * traversal (cds_ft_next / cds_ft_prev resolve a cell pointer with no descent),
 * so for groups using the ordered list the tree descent is only the amortized
 * seek / seed and is better left as one shared, non-specialized function -- far
 * smaller code.  Enable it for inequality-heavy workloads WITHOUT the ordered
 * list, where the descent is the per-operation cost.  Governs only the tier-2
 * descent (ft_ineq_descend); the tier-1 ordered-cell fast path is always inlined
 * under FEATURE_INLINE_LOOKUP.
 */

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
 * FEATURE_FT_KEY_MAP: support a non-identity key map (custom byte ordering set
 * via cds_ft_group_attr_set_key_map -- e.g. case-insensitive or collation
 * orders).  Identity maps (plain byte-ordinal keys) work either way.
 *
 * Enabled by default.  Disable with -DNO_FEATURE_FT_KEY_MAP for byte-key-only
 * builds: it compiles out the per-key-API non-identity lookup specializations
 * (~25 KiB of .text), and cds_ft_group_attr_set_key_map then returns
 * CDS_FT_STATUS_NOT_SUPPORTED.
 */
#ifndef NO_FEATURE_FT_KEY_MAP
# define FEATURE_FT_KEY_MAP
#endif

/*
 * FEATURE_FT_MERGE: the cds_ft_merge / cds_ft_merge_at bulk operation
 * (atomic spine-copy merge of one trie into another at a key position).
 *
 * Enabled by default.  Disable with -DNO_FEATURE_FT_MERGE for builds
 * that never merge tries: it compiles out the whole merge subsystem
 * (~20 KiB of .text -- the recursive spine-copy build/count, the
 * ordered-cell interleave, the in-place src unlink and the reserve
 * pre-pass), and cds_ft_merge / cds_ft_merge_at then return
 * CDS_FT_STATUS_NOT_SUPPORTED.  graft, graft_swap and detach are
 * unaffected -- they remain part of the core bulk API.
 */
#ifndef NO_FEATURE_FT_MERGE
# define FEATURE_FT_MERGE
#endif

/*
 * FEATURE_FT_INSERT_IN_PLACE: in-place occupancy-bitmap safe-append (the
 * single-writer insert fast path).  OPT-IN; RECOMPACT-ON-INSERT is the DEFAULT.
 *
 * When enabled (-DFEATURE_FT_INSERT_IN_PLACE), an insert that lands at a node's
 * tail rank with spare tier capacity is applied IN PLACE: the child slot is
 * published and the node's occupancy-bitmap bit is set with a relaxed store on
 * the LIVE node's metadata word (ft_popcount_node_set_nth Cases 2A/2B,
 * ft_pigeon_node_set_nth).  This is the cheap O(1) insert tier, but it mutates a
 * live node's words disjointly, so it is NOT safe against concurrent writers.
 *
 * By DEFAULT (the macro undefined) every new-occupancy insert (i.e. not an
 * in-place pointer replace at an already-occupied slot) instead reports -ERANGE,
 * so the setter wrapper (ft_node_set_nth_rec) routes it through
 * ft_node_recompact(ADD_SAME) -- a fresh node carrying the new entry AND its
 * bitmap is built build-invisibly and the parent edge is flipped, exactly as a
 * non-tail insert already does.  This removes the last reader-visible in-place
 * node-word mutation (the Invariant-2 disjoint-word hazard, see
 * doc/design/mcas-multiwriter-readiness.md S4): every node change becomes a
 * whole-node replacement via the parent flip-txn edge -- the multi-writer-safe
 * shape.  Behaviour-identical under a single writer (the recompact path is the
 * same one a non-tail insert takes), at the cost of turning the O(1) in-place
 * insert into an O(node) alloc-and-copy recompact.  Re-enabling the in-place
 * fast path for a single-writer trie (a runtime gate) is a future perf knob.
 *
 * Both popcount and pigeon nodes recompact uniformly here.  FUTURE (noted,
 * not done -- kept simple for now): a PIGEON slot is direct-indexed, so its
 * pointer store is already a clean flip edge and the bitmap is only an
 * occupancy HINT (point lookups read the slot; iteration rescans past a set
 * bit whose slot is NULL).  The pigeon recompact is therefore avoidable: make
 * the hint bitmap STICKY instead -- set with an atomic OR, never cleared in
 * place (a delete leaves the bit; only a recompact rebuilds a clean bitmap),
 * optionally triggering a cleanup recompact once stale bits get high -- to
 * keep pigeon's O(1) insert/delete.  See doc/design/mcas-multiwriter-
 * readiness.md S4.
 *
 * Default: recompact-on-insert (multi-writer-safe).  Opt into the in-place
 * fast path with -DFEATURE_FT_INSERT_IN_PLACE.  -DNO_FEATURE_FT_INSERT_IN_PLACE
 * forces recompact even if the fast path was requested (otherwise a no-op,
 * kept for back-compat).
 */
#ifdef NO_FEATURE_FT_INSERT_IN_PLACE
# undef FEATURE_FT_INSERT_IN_PLACE
#endif

/*
 * Skip-compressed pointers encode the compressed path length in the
 * high bits of pointers (FT_SKIP_LEN_BITS bits starting at
 * FT_SKIP_LEN_SHIFT -- e.g. bits 57-63 on x86-64, 56-63 on AArch64;
 * see the per-architecture encoding parameters above).  This requires
 * architectures where those bits are guaranteed zero for userspace
 * pointers.
 *
 * Enabled on architectures that define FT_SKIP_LEN_BITS.  A runtime
 * validation (mmap probe) at flag-set time rejects the feature
 * if the encoding bits fall within the kernel's VA range.
 *
 * Not supported on 32-bit architectures (no spare high pointer bits;
 * FT_SKIP_LEN_BITS is left undefined there) nor on s390x (full 64-bit
 * virtual addresses).
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
 * Library-owned ordered sibling list.
 *
 * Threads the duplicate-chain heads (one per distinct key) into a
 * key-ordered doubly-linked list of LIBRARY-OWNED "ordinal cells"
 * (struct ft_ord_cell), so cds_ft_next / cds_ft_prev walk the cell list
 * in O(1) per step instead of an O(depth) trie descent + backtrack.  The
 * order links live OUTSIDE the application leaf: struct cds_ft_node is
 * unchanged (no ABI growth, no app offset), and the cell is reached via
 * the head's cds_ft_node.prev.  A head's prev now points to its cell
 * (tagged FT_INTERNAL_MASK, so the head-vs-dup test
 * ft_node_external(prev)==false is preserved) and the head's parent moves
 * into ft_ord_cell.parent.  The trie's DOWNWARD child slots still point
 * directly at the external node, skipping the cell; the cell is interposed
 * only on the UPWARD walk (parent recovery) and the ordered traversal.
 *
 * Because the cells are library-owned they are RELOCATABLE: cds_ft_compact
 * packs them in key order so ordered iteration becomes a dense scan
 * (the per-step random leaf load dependency is what makes the in-leaf
 * chain latency-bound).  Cells are kept reader-coherent via the flip-latch.
 *
 * Always compiled in (mandatory; there is no non-cell build).  Both the cell
 * and its ordered LIST are runtime-gated per group via
 * cds_ft_group_attr_set_ordered_list, recorded as the trie's immutable
 * ft->ordered_list flag (read on the hot path): an enabled group allocates a
 * cell per distinct-key head (the head's parent is reached through the cell)
 * and maintains the key-ordered list; an unset group allocates NO cells
 * (head->prev IS the flagged parent directly) and skips all list maintenance,
 * so it pays neither the per-head cell nor the indirection.
 */

/*
 * Library-owned ordinal cell: one per distinct-key duplicate-chain head.
 *   @ord_prev/@ord_next: key-ordered doubly-linked list of cells.
 *   @node:   the external head this cell indexes (cell -> leaf, for the key).
 *   @parent: the head's flagged parent internal node (relocated out of
 *            cds_ft_node.prev, which now points at this cell).
 * Reached from a head as ft_ord_cell_ptr(rcu_dereference(head->prev)).
 */
struct ft_ord_cell {
	/*
	 * Key-ordered doubly-linked list links, embedded as the public
	 * concurrent bidir-list node (<urcu/rcu-txn-list.h>): lnode.next
	 * is the old ord_next, lnode.prev the old ord_prev.  The cell is recovered
	 * from a link with ft_ord_cell_of() (container_of) and its link node with
	 * ft_ord_cell_lnode().  Embedding the public node lets the single-cell
	 * splice/unsplice/replace ride the list's composable _prepare ops, folded
	 * into the FT structural flip-txn; the cell edges carry the concurrent
	 * list's engine proxy tag (URCU_MCAS_TAG, bit 0) and readers resolve them
	 * with urcu_txn_list_resolve (ft_ord_cell_resolve_ord).
	 */
	struct urcu_txn_list_node lnode;
	struct cds_ft_node *node;
	struct cds_ft_inode_flag *parent;
};

/* Recover the cell owning an embedded ordered-list link node, and vice versa. */
static inline
struct ft_ord_cell *ft_ord_cell_of(const struct urcu_txn_list_node *lnode)
{
	return caa_container_of(lnode, struct ft_ord_cell, lnode);
}

static inline
struct urcu_txn_list_node *ft_ord_cell_lnode(struct ft_ord_cell *cell)
{
	return &cell->lnode;
}

/*
 * Item-length order for the dedicated cell arena: the smallest power of two
 * that holds a struct ft_ord_cell (32 B => order 5).  A 1<<order-aligned cell
 * keeps bits 0-2 clear, so the FT_ORD_CELL_TAG (bit 0) and flip-proxy (type 7)
 * encodings stay unambiguous.  Bump it if the cell grows (the assert guards).
 */
#define FT_ORD_CELL_ALLOC_ORDER		5
urcu_static_assert(sizeof(struct ft_ord_cell) <= (1U << FT_ORD_CELL_ALLOC_ORDER),
		"struct ft_ord_cell must fit the cell arena item size",
		ord_cell_fits_alloc_order);
/*
 * lnode MUST be the first field (offset 0): ft_ord_cell_of() is then a 0-offset
 * cast, so ft_ord_cell_of(NULL) == NULL and ft_ord_cell_of(&ft->ord_sentinel.node)
 * == &ft->ord_sentinel.node.  Both aliasings are load-bearing -- ft_ord_is_end()
 * recognises a NULL link and the trie's own sentinel as "end" with a single
 * pointer compare, and the cross-trie run moves NULL-terminate a moved run's
 * outer links as a universal end.  A field placed before lnode would turn
 * ft_ord_cell_of(NULL) into a small non-NULL garbage pointer that ft_ord_is_end()
 * would mistake for a real cell.
 */
urcu_static_assert(offsetof(struct ft_ord_cell, lnode) == 0,
		"struct ft_ord_cell.lnode must be the first field (offset 0)",
		ord_cell_lnode_offset_zero);

/*
 * FEATURE_FT_EXCL_VALIDATE: runtime validation of the access-discipline
 * contract.  Writers claim a per-trie owner via atomic CAS at the
 * public API boundary; writer/writer overlap aborts the process with
 * a violation report.
 *
 * The writer/writer check above is mode-independent (the contract
 * serializes writers in both modes); only reader validation depends on
 * the trie mode:
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
 * inline_ineq: like inline_lookup but gated on FEATURE_INLINE_INEQUALITY_LOOKUP,
 * for the tier-2 inequality descent (ft_ineq_descend).  Default-off -> plain
 * static, one shared copy across all modes/limits.
 */
#ifdef FEATURE_INLINE_INEQUALITY_LOOKUP
#define inline_ineq	inline_lookup
#else
#define inline_ineq
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
 * FT_RELOAD_FROM_STACK(var) is the matching reload -- a volatile
 * load from @var's stack slot into a fresh register.
 *
 * The two MUST be paired.  Every code path that reaches the
 * reload must have executed the matching spill first; otherwise
 * the reload returns an uninitialized stack-slot value.
 *
 * Implementation: an empty inline asm with `"=m"`/`"=r"`
 * constraints would declare the memory or register as touched
 * but emit no instructions -- gcc's data-flow then folds the
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
	FT_BITMAP = true,
};

/* Never declared. Opaque type used to store flagged node pointers. */
struct cds_ft_inode_flag;
struct cds_ft_inode;
struct cds_ft_alloc_arena;

/*
 * Recorder for ft_publish_to_parent's reader-visible edges.  When a
 * key-disappearing remove recompacts a node, the rebuilt node must be
 * published into its parent slot ATOMICALLY with the dead head cell's
 * ordered-list unsplice (the remove dual of the insert splice window).  A
 * non-NULL rec makes ft_publish_to_parent perform all of its non-reader-
 * visible bookkeeping (parent-slot offset, trace events) but RECORD its 1-2
 * reader-visible stores -- the forward parent slot, plus a compressed
 * parent's SKIP_X dual pointer -- here instead of doing them, so the caller
 * (ft_detach_node) can commit them in one flip with the cell edges.  At most
 * two edges: forward slot + skip dual.
 */
struct ft_pub_rec {
	struct cds_ft_inode_flag **slot[3];
	struct cds_ft_inode_flag *old_val[3];
	struct cds_ft_inode_flag *new_val[3];
	unsigned int n;
};

/*
 * Struct layout (32 bytes, zero internal padding):
 *   offset  0: 8-byte parent pointer
 *   offset  8: 8-byte external_nodes pointer
 *   offset 16: 8-byte nr_keys (unsigned long, total keys in subtree)
 *   offset 24: 4-byte packed bitfield (nr_child, parent_slot_offset,
 *              alloc_index)
 *   offset 28: 4-byte tail padding
 *
 * In cds_ft_metadata_alloc, rcu_head is a separate field placed
 * before the metadata union -- no overlap with metadata fields.
 * All metadata fields remain valid throughout the RCU grace period.
 */
/*
 * Per-node MCAS state word (struct cds_ft_metadata.state) bit layout.
 * bit 0 = proxy (in-band flip marker), bit 1 = tombstone (LIVE->DEAD),
 * bits 2-10 = nr_child, bits 11-18 = parent_slot_offset, bit 19 = COPYING
 * (the reversible per-node writer lock; bits 20+ free).
 * See doc/design/mcas-multiwriter-readiness.md §4.2 and, for bit 19,
 * doc/design/mw-writer-lock-escalation-model.md §0/§2.
 */
#define FT_STATE_PROXY			((uintptr_t) 1 << 0)
#define FT_STATE_TOMBSTONE		((uintptr_t) 1 << 1)
#define FT_STATE_NR_CHILD_SHIFT		2
#define FT_STATE_NR_CHILD_BITS		9	/* live-child count, max 256 (bits 2-10) */
#define FT_STATE_NR_CHILD_VALMASK	(((uintptr_t) 1 << FT_STATE_NR_CHILD_BITS) - 1)
#define FT_STATE_NR_CHILD_MASK		(FT_STATE_NR_CHILD_VALMASK << FT_STATE_NR_CHILD_SHIFT)
#define FT_STATE_NR_CHILD_ONE		((uintptr_t) 1 << FT_STATE_NR_CHILD_SHIFT)
/*
 * parent_slot_offset (0..255): the pointer-stride offset of this node's slot in
 * its parent, packed ABOVE nr_child so that a re-home -- which changes both the
 * parent pointer and the slot offset -- commits parent-edge + offset as ONE
 * atomic MCAS state edge (a concurrent backtracker never straddles a
 * new-parent/old-offset window).  Access ONLY via the ft_meta_parent_slot_offset*
 * helpers (they mask/preserve the tag + nr_child bits).
 */
#define FT_STATE_PSO_SHIFT		(FT_STATE_NR_CHILD_SHIFT + FT_STATE_NR_CHILD_BITS)
#define FT_STATE_PSO_BITS		8
#define FT_STATE_PSO_VALMASK		(((uintptr_t) 1 << FT_STATE_PSO_BITS) - 1)
#define FT_STATE_PSO_MASK		(FT_STATE_PSO_VALMASK << FT_STATE_PSO_SHIFT)
/*
 * FT_STATE_COPYING (bit 19, above parent_slot_offset): the REVERSIBLE per-node
 * WRITER LOCK.  It began as the copy fence (MW campaign, Option A -- doc/design
 * + CORE_682870 fix plan F2) and the MW lock-escalation model (§0) is the
 * observation that the fence already IS a lock: CAS to acquire, held across the
 * build/reparent plan window, -EAGAIN on contention, resolved at commit.
 *
 * ACQUIRE: a standalone CAS {clean -> |COPYING}, taken BEFORE the first read of
 * the node it protects.  Every peer publish into the node then aborts on its
 * §4.B guard (the guard's clean-LIVE expectation masks this bit like the
 * tombstone), so the protected body cannot go stale under the holder without
 * someone aborting.  The mark CAS failing (bit already set, a parked proxy, or a
 * real retire) = "could not acquire" -> -EAGAIN, re-descend.
 *
 * TERMINALS -- exactly two on a successful commit, both expressed as a RECORDED
 * MCAS edge on the state word whose expected old is the mark's CLEAN snapshot
 * (never a fresh read -- the CORE_682870 defect-1 contract):
 *
 *   RETIRE  {COPYING|s -> TOMBSTONE|s}  ft_flip_txn_record_tombstone_copying()
 *           The holder copied the node away; it dies at the commit.  One-way
 *           tombstone semantics (exactly-once retire token, freeze-on-free) are
 *           UNCHANGED.
 *   RELEASE {COPYING|s -> s}            ft_flip_txn_record_release_copying()
 *           The holder only needed exclusion; the node SURVIVES the commit.
 *           This is what a lock-set member that is edited but not retired needs
 *           (recompact's parent P, §9.3) -- a lock that unlocks.
 *
 * On ABORT / a pre-commit bail the bit is CAS-cleared instead (the registry
 * drain, ft_flip_txn_copying_clear_all) and the node stays live -- so the
 * ABORT terminal and the RELEASE terminal agree on the resulting word, they
 * differ only in who writes it (a bare CAS vs the atomic commit).
 *
 * Unlike the tombstone, COPYING is reversible BY DESIGN and never implies
 * death; it is never set at rest (ft-verify.h reports a leaked fence).
 */
#define FT_STATE_COPYING		((uintptr_t) 1 << \
					(FT_STATE_PSO_SHIFT + FT_STATE_PSO_BITS))
#define FT_STATE_TAG_MASK		(FT_STATE_PROXY | FT_STATE_TOMBSTONE)

struct cds_ft_metadata {
	/* 8-byte aligned fields. */
	struct cds_ft_inode_flag *parent;	/*
						 * Tagged pointer to parent node.  The only
						 * NULL a reader can observe is at the root.
						 * It is also transiently NULL on the write
						 * side (between a detach / graft_swap
						 * clearing the link and the new placement
						 * completing), but such a node is not
						 * reader-reachable: publication wires the
						 * parent before the node is reachable, and
						 * synchronize_rcu separates the phases.
						 * Written by the mutation side via
						 * rcu_assign_pointer; read by the read side
						 * (ft_skip_to_compressed, ft_get_parent_rcu)
						 * via rcu_dereference.
						 */
	struct cds_ft_node *external_nodes;	/* List of external nodes at this trie location. */

	/*
	 * Total unique keys in subtree.
	 * Stored with uatomic_store release, loaded with acquire.
	 */
	unsigned long nr_keys;

	/*
	 * Per-node MCAS state word (doc/design/mcas-multiwriter-readiness.md
	 * §4.2).  One uintptr_t so a single committed flip-latch edge can carry
	 * a node-state change:
	 *   bit 0   FT_STATE_PROXY     -- in-band flip marker (set only mid-flip;
	 *                                 lets the scalar ride the flip-latch).
	 *   bit 1   FT_STATE_TOMBSTONE -- one-way LIVE->DEAD deleted latch (§4.B).
	 *   bits 2-10  nr_child        -- live-child count (max 256, 9 bits).
	 *   bits 11-18 parent_slot_offset -- pointer-stride offset of this node's
	 *                                 slot in its parent body (8 bits), so a
	 *                                 re-home commits parent + offset as one
	 *                                 atomic state edge.
	 * Access nr_child / parent_slot_offset ONLY via the ft_meta_nr_child* /
	 * ft_meta_parent_slot_offset* helpers -- they mask their own field and
	 * preserve the others; never read/write the word directly.
	 */
	uintptr_t state;

	/*
	 * Packed bitfield -- small fields in a single uint32_t.
	 * (nr_child AND parent_slot_offset both live in @state above, not here.
	 * parent_slot_offset -- the pointer-stride offset of this node's slot in
	 * its parent -- moved into the state word so a re-home commits the parent
	 * edge and the slot offset as ONE atomic MCAS state edge; access it via
	 * the ft_meta_parent_slot_offset* helpers.)
	 *
	 * alloc_index:            near: FT_ALLOC_INDEX_BITS + 3 spare bits of
	 *                         headroom above the page_size >>
	 *                         FT_ALLOC_ORDER_MIN minimum; far: a separate
	 *                         uint32_t (see below).
	 */
#ifdef FT_FAR_METADATA
	/*
	 * A 2 MiB far macro-block holds far more than 256 items (e.g. ~18 700
	 * order-5 nodes), overflowing the FT_ALLOC_INDEX_BITS (8-bit) packed
	 * field.  Store alloc_index in its own word -- it lands in the struct's
	 * existing 4-byte tail padding, so the struct stays 32 B and the hot
	 * descent bitfield (nr_child/parent_slot_offset) is untouched.  24 bits
	 * (16 M) is ample for a 2 MiB block; the top 8 bits carry incoming_byte
	 * (below), keeping the struct 32 B without a separate tail byte.
	 */
	uint32_t alloc_index:24;
	uint32_t incoming_byte:8;
#else
	/*
	 * Near metadata: alloc_index is packed into this word, sized with
	 * 3 spare bits of headroom above the FT_ALLOC_INDEX_BITS minimum
	 * needed to index a page-sized range.
	 */
	uint32_t alloc_index:(FT_ALLOC_INDEX_BITS + 3);
	/*
	 * Branch byte on the parent->this edge: the key byte consumed entering
	 * this node.  Maintained at every child placement so an UPWARD key
	 * rebuild (ft_rebuild_key_upwalk) recovers each level's byte in O(1) --
	 * no in-leaf key copy and no parent-bitmap inversion.  This is the
	 * structural key source that lets the ordered-list walk materialize its
	 * result key WITHOUT a speculative_key_offset (e.g. an EAGER trie).
	 *   - internal node: the single incoming edge byte.
	 *   - compressed node: unused -- key_bytes[0] already IS that byte.
	 *   - external head: stored in the head's CELL metadata (the cell is the
	 *     head's metadata record), set to the key's last byte at insert.
	 * Near layout: occupies the 32 B struct's offset-28 tail padding; the
	 * hot descent bitfield is untouched.
	 */
	uint8_t incoming_byte;
#endif
};

/*
 * nr_child accessors -- nr_child lives in @state bits 2+, with the proxy
 * (bit 0) and tombstone (bit 1) tags below it.  Reads shift the tags out;
 * writes preserve them.  Inc/dec add/subtract one count unit, which leaves
 * the low tag bits untouched.
 */
static inline
unsigned int ft_state_nr_child(uintptr_t state)
{
	return (unsigned int) ((state >> FT_STATE_NR_CHILD_SHIFT)
			& FT_STATE_NR_CHILD_VALMASK);
}

static inline
unsigned int ft_meta_nr_child(const struct cds_ft_metadata *meta)
{
	return ft_state_nr_child(meta->state);
}

static inline
void ft_meta_nr_child_set(struct cds_ft_metadata *meta, unsigned int n)
{
	/* Replace only the nr_child field; preserve tags + parent_slot_offset. */
	meta->state = ((uintptr_t) n << FT_STATE_NR_CHILD_SHIFT)
		| (meta->state & ~FT_STATE_NR_CHILD_MASK);
}

/*
 * nr_child +/- 1 as a LATCH-HONORING CAS transition (Phase 4.3).  The former
 * plain read-modify-write raced concurrent writers on SHARED nodes -- the
 * set_nth type arms run on a LIVE published node whenever an in-place insert
 * (the case-0 reserve) targets a spine node two writers share, so a plain +=
 * both LOSES increments (nr_child < popcount, feeding garbage to every
 * rank-bounded walk) and, worse, does ARITHMETIC ON A PARKED FT_STATE_PROXY
 * pointer when a peer's state edge is mid-commit -- minting a corrupted
 * near-pointer with tag bits that a resolver then chases.  The CAS loop
 * re-derives from the current word and waits out a parked proxy (bounded by
 * the owner's settle), mirroring ft_meta_state_transition.  A fresh
 * single-owner node pays one uncontended CAS.
 */
static inline
void ft_meta_nr_child_inc(struct cds_ft_metadata *meta)
{
	for (;;) {
		uintptr_t s = CMM_LOAD_SHARED(meta->state);

		if (caa_unlikely(s & FT_STATE_PROXY)) {
			caa_cpu_relax();
			continue;
		}
		if (caa_likely(uatomic_cmpxchg(&meta->state, s,
				s + FT_STATE_NR_CHILD_ONE) == s))
			return;
	}
}

static inline
void ft_meta_nr_child_dec(struct cds_ft_metadata *meta)
{
	for (;;) {
		uintptr_t s = CMM_LOAD_SHARED(meta->state);

		if (caa_unlikely(s & FT_STATE_PROXY)) {
			caa_cpu_relax();
			continue;
		}
		if (caa_likely(uatomic_cmpxchg(&meta->state, s,
				s - FT_STATE_NR_CHILD_ONE) == s))
			return;
	}
}

/*
 * parent_slot_offset: the pointer-stride offset of this node's slot in its
 * parent body, held in @state bits FT_STATE_PSO_SHIFT.. (above nr_child).  This
 * raw reader suits a node the caller owns / that is quiescent; a reader that may
 * race a mid-commit proxy uses the resolving ft_meta_parent_slot_offset_load,
 * and a backtracker recovering the (parent, offset) PAIR must use
 * ft_resolve_parent_slot (same-mcas + stability re-read).  Root nodes
 * (parent == NULL) leave this 0 and never read it.
 */
static inline
unsigned int ft_meta_parent_slot_offset(const struct cds_ft_metadata *meta)
{
	return (unsigned int) ((meta->state >> FT_STATE_PSO_SHIFT)
			& FT_STATE_PSO_VALMASK);
}

/*
 * The setter replaces only the offset field and preserves nr_child + the tag
 * bits.  It CASes for the two reasons ft_meta_nr_child_inc above documents:
 * @state is shared with nr_child, TOMBSTONE and COPYING, which peers update by
 * cmpxchg, so a plain read-modify-write would LOSE a peer's concurrent update;
 * and when a peer has parked an FT_STATE_PROXY the word holds a PROXY POINTER,
 * so masking an offset into it would mint a corrupted near-pointer that a
 * resolver later chases.
 *
 * Both hazards are REACHABLE TODAY on the concurrent point-op path: the
 * in-place reserve insert republishes the LIVE attach node at its own slot
 * (ft-insert.h, "In-place reserve (dest == attach node)"), reaching this setter
 * via _ft_publish_to_parent_meta on a node peers are mutating.  A peer
 * reserving a different byte in that same node CASes its nr_child
 * (_ft_node_set_nth -> ft_meta_nr_child_inc), and a peer re-homing it parks an
 * FT_STATE_PROXY on its state (ft_reparent_record_meta).  The node is not
 * COPYING-fenced there, and the op guards its GRANDparent, not it.
 *
 * The remaining LIVE-node callers -- ft_glue_record_back_edge and the whole-trie
 * detach root -- are safe only because merge / graft / compaction still run
 * under the application's mutual exclusion between mutators, and are not yet
 * MW-hardened.  Making this word CAS-only removes its last plain RMW, which is
 * also the precondition for letting those bulk ops run concurrently.
 *
 * Wait out a parked proxy rather than skipping it: the offset must land on the
 * settled word, and the wait is bounded by the owner's settle.  Skip the CAS
 * entirely when the offset is unchanged -- the in-place reserve's same-value
 * republish is the hot case.  A fresh single-owner node pays one uncontended
 * CAS.
 */
static inline
void ft_meta_parent_slot_offset_set(struct cds_ft_metadata *meta, unsigned int off)
{
	for (;;) {
		uintptr_t s = CMM_LOAD_SHARED(meta->state);
		uintptr_t n;

		if (caa_unlikely(s & FT_STATE_PROXY)) {
			caa_cpu_relax();
			continue;
		}
		n = (s & ~FT_STATE_PSO_MASK)
			| (((uintptr_t) off & FT_STATE_PSO_VALMASK)
				<< FT_STATE_PSO_SHIFT);
		if (n == s)
			return;		/* same-value republish: nothing to do */
		if (caa_likely(uatomic_cmpxchg(&meta->state, s, n) == s))
			return;
	}
}

/*
 * Read the one-way LIVE->DEAD tombstone (state bit 1, §4.B freeze-on-free).
 * True once the node has been marked dead at retire (before its unlink commit).
 */
static inline
bool ft_meta_tombstone(const struct cds_ft_metadata *meta)
{
	return (meta->state & FT_STATE_TOMBSTONE) != 0;
}

/*
 * Compressed path node.  Replaces a chain of single-child internal
 * nodes with a single node storing the key bytes inline.
 *
 * cn->child can point to any node type: internal, compressed,
 * or external.  An external child means a key terminates at the
 * end of the compressed path.  However, the compressed node's
 * metadata->external_nodes must NOT be used -- variable-length key
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

/*
 * Per-group byte collation map: a bijection between raw key bytes and
 * their ordinal (sort) position, letting the trie order keys by the
 * application's collation instead of by raw byte value.  @key_to_ordinal
 * and @ordinal_to_key are inverse 256-entry permutations; @identity is
 * set when the map is the identity (byte == ordinal), which the hot
 * paths special-case to skip the remap.
 */
struct cds_ft_key_map {
	bool identity;
	uint8_t key_to_ordinal[256];
	uint8_t ordinal_to_key[256];
};

/*
 * A group of Fractal Trie instances (struct cds_ft) that share
 * allocation arenas, a key collation map, and creation-time
 * configuration (key length, lookup optimization, NUMA / page-size
 * policy, ordered-list mode).  The bulk operations (graft, graft_swap,
 * detach, merge) move whole sub-tries between tries of the SAME group,
 * which is why the arenas and the leak accounting live here rather than
 * per-trie.  Created by cds_ft_group_create (configured via
 * cds_ft_group_attr_create); individual tries are added with
 * cds_ft_create.
 */

/*
 * Node-allocation reserve (see cds_ft_group::active_reserve).  Holds nodes
 * pre-allocated by a bulk op so its commit phase can draw them without an
 * arena allocation that could fail.  Keyed by [kind][item_len_order]: kind 0 is
 * the regular internal-node arena (CDS_FT_ALLOC_KIND_NODE), kind 1 the
 * compressed-node arena (CDS_FT_ALLOC_KIND_COMPRESSED).  Cells are never
 * reserved (the bulk ops move existing cells, they do not allocate new ones).
 * Per-bucket capacity is small because each bulk op's exact node need is O(1)
 * per (kind, order); reserve fills assert against it.
 */
#define CDS_FT_ALLOC_RESERVE_NR_KIND	2
#define CDS_FT_ALLOC_RESERVE_CAP	8

enum cds_ft_alloc_kind {
	CDS_FT_ALLOC_KIND_NODE = 0,
	CDS_FT_ALLOC_KIND_COMPRESSED = 1,
	CDS_FT_ALLOC_KIND_CELL = 2,	/* never reserved */
};

struct cds_ft_alloc_reserve {
	struct cds_ft_metadata *items[CDS_FT_ALLOC_RESERVE_NR_KIND]
			[FT_ALLOC_ORDER_MAX + 1][CDS_FT_ALLOC_RESERVE_CAP];
	unsigned int count[CDS_FT_ALLOC_RESERVE_NR_KIND][FT_ALLOC_ORDER_MAX + 1];
};

struct cds_ft_group {
	size_t max_tree_depth;
	size_t key_len;
	size_t max_key_len;		/* Maximum key length allowed. */
	unsigned int flags;		/* CDS_FT_FLAG_* creation-time flags. */
	const struct rcu_flavor_struct *flavor;
	/*
	 * Concurrent-engine escalation domain for the group's structural
	 * transactions (urcu_txn_*).  Shared by every trie in the group: a
	 * writer that keeps losing the optimistic race escalates to the
	 * domain's fair lock so progress is bounded.  Currently exercised
	 * under retained caller exclusion (no contention), so the fast path
	 * never escalates.
	 */
	struct urcu_txn_domain domain;
	/*
	 * Structural-writer concurrency strategy for the group's tries (MW
	 * lock-escalation model).  Copied to each trie at create; a locking
	 * strategy (COARSE or FINE) makes the trie take the FT-wide writer lock
	 * at every mutation.  Default CDS_FT_WRITER_OPTIMISTIC (calloc-zero).
	 */
	enum cds_ft_writer_strategy writer_strategy;
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
	/*
	 * Dedicated arena for ordinal cells (ordered_list_set groups).
	 * Separate from arena_order[] so the uniform 32 B cell
	 * bodies pack contiguously in their own item region -- the dense
	 * ord-walk stride that cds_ft_compact exploits -- instead of being
	 * diluted among the internal-node ranges.  Lazily created on the
	 * first cell allocation (single order: the cell is uniform 32 B).
	 * cds_ft_free_item recovers the range (and its rcu_head) from the
	 * cell pointer for reader-safe deferred reclaim + nr_live draining.
	 */
	struct cds_ft_alloc_arena *cell_arena;
	/*
	 * Cell leak accounting (ft_debug_counters only): cells migrate
	 * between the group's tries via the bulk ops (allocated in one
	 * trie, freed in another), so they are accounted at group
	 * granularity.  cds_ft_group_destroy reports a mismatch.
	 * Counted synchronously at the free request (not the deferred
	 * reclaim), so allocated == freed means every cell's free was
	 * issued.
	 */
	unsigned long nr_cells_allocated, nr_cells_freed;
	/*
	 * Node leak accounting (ft_debug_counters only).  Like the cells
	 * above, internal and compressed nodes migrate between the group's
	 * tries via the bulk ops (a graft allocates a node accounted to the
	 * source trie and frees it accounted to the destination), so they
	 * are accounted at group granularity.  cds_ft_group_destroy reports
	 * a mismatch once every trie has drained its deferred frees.
	 */
	unsigned long nr_nodes_allocated, nr_nodes_freed;
	unsigned long nr_internal_alloc, nr_internal_freed;
	unsigned long nr_compressed_alloc, nr_compressed_freed;
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
	 * @speculative_key_offset: byte offset from the (struct cds_ft_node *)
	 *   stored in the trie to the caller's key bytes, used by the
	 *   speculative inequality lookup to copy a result key directly from
	 *   the leaf (the leaf is the single source of the key, so the
	 *   descent need not load compressed-node cache lines to rebuild it).
	 *   @speculative_key_offset_set records whether it was configured;
	 *   offset 0 is a valid value, so a separate flag is required.  Only
	 *   consulted when @speculative and CDS_FT_FLAG_SKIP_COMPRESSED are
	 *   set.  See cds_ft_group_attr_set_speculative_key_offset.
	 */
	size_t speculative_key_offset;
	bool speculative_key_offset_set;
	/*
	 * @key_len_offset: byte offset from the (struct cds_ft_node *) to a
	 *   size_t holding the leaf's key length, and whether configured.  Lets
	 *   the ordered cell list materialize a VARIABLE-length result key from
	 *   the leaf (the descent normally derives the length as it walks; the
	 *   cell fast path skips the descent).  Unused for fixed-length groups
	 *   (length is group->key_len).  See cds_ft_group_attr_set_key_len_offset.
	 */
	size_t key_len_offset;
	bool key_len_offset_set;
	/*
	 * @ordered_list_set: enable the library-owned ordered sibling list
	 *   (FEATURE_FT_ORD_CELL).  The order links live in a
	 *   library-owned relocatable "ordinal cell" (struct ft_ord_cell) hung off
	 *   each duplicate-chain head's cds_ft_node.prev; the application leaf is
	 *   unchanged (no ord fields, no offset to declare).  The result key is
	 *   materialized from the leaf when speculative_key_offset is configured
	 *   (plus key_len_offset for a variable-length group), otherwise
	 *   structurally by the parent up-walk (ft_rebuild_key_upwalk) -- the
	 *   walk recovers ordinal bytes, so it serves any key map.  See
	 *   cds_ft_group_attr_set_ordered_list.
	 */
	bool ordered_list_set;
	/*
	 * @rank_stats_set: maintain the per-node order-statistics key counts
	 *   (cds_ft_metadata.nr_keys) that back the rank / select / count
	 *   queries (cds_ft_count_keys / _prefix, cds_ft_lookup_nth / _last,
	 *   cds_ft_iter_skip_forward / _reverse).  Default OFF.  When off the
	 *   library performs no count propagation and the queries fall back to
	 *   a full enumeration (count) or first/last + next/prev iteration
	 *   (select / skip).  Immutable after group creation.  See
	 *   cds_ft_group_attr_set_rank_stats.
	 */
	bool rank_stats_set;
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
 * shape -- the library picks the right inner once at cds_ft_create
 * and caches it here).  See ft_install_lookup_ops in fractal-trie.c.
 *
 * Each pointer's signature matches the corresponding cds_ft_*
 * public API.  The pointer is stable for the lifetime of the trie:
 * group flags (key_map.identity, CDS_FT_FLAG_SKIP_COMPRESSED bit) are
 * immutable after group creation, so a single load + indirect jmp on
 * the hot path is all the dispatch cost vs the ~22 ns regression seen
 * from runtime-gated dispatch.
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

/*
 * A single Fractal Trie instance: one ordered map from byte keys to
 * application-owned nodes, rooted at @root and belonging to a @group
 * (whose arenas and configuration it uses).  Caches the specialized
 * per-API lookup dispatch pointers (installed once at cds_ft_create),
 * the access-discipline mode (@exclusive vs concurrent RCU readers), the
 * ordered-list mirror flag with the cached list endpoints, and any
 * in-progress compaction state.  Created by cds_ft_create.
 */
struct cds_ft {
	struct cds_ft_group *group;

	struct cds_ft_inode_flag *root;		/* Root node (arena-allocated, always present, always internal). */

	/*
	 * Specialized lookup dispatch pointers -- installed at
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
	/*
	 * Limit-none relational entries (cds_ft_lookup_le/ge/lt/gt, and
	 * cds_ft_next/prev which resolve to gt/lt).  Installed once at create
	 * (ft_install_lookup_ops) to the use_keycopy-appropriate specialization,
	 * so the public entry is a single indirect tail-call with no per-call
	 * config branch.  first/last stay on the inline dispatcher (cold path).
	 */
	cds_ft_lookup_iter_fn lookup_le_fn;
	cds_ft_lookup_iter_fn lookup_ge_fn;
	cds_ft_lookup_iter_fn lookup_lt_fn;
	cds_ft_lookup_iter_fn lookup_gt_fn;

	size_t max_used_key_len;		/* Maximum key length inserted (conservative). */

	/*
	 * The active node-allocation reserve (the bulk-op OOM-avoidance pool for
	 * merge_at's sub-position residual / graft) is NOT stored here: it is
	 * THREAD-LOCAL (fractal-trie-alloc.c, ft_tls_reserves[]).  A reserve is a
	 * stack-local owned by one bulk op on one thread, so keeping its
	 * activation state per-thread lets concurrent bulk ops on the same live
	 * dst (post FT-wide-lock drop, §11) each own their reserve without
	 * colliding on a shared field.  Query via cds_ft_alloc_reserve_covers().
	 */

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
	 * Mirror of group->ordered_list_set, cached on the trie so the read-side
	 * head-parent resolver (ft_resolve_head_prev, on the descent / skip /
	 * backtrack hot paths) tests one local flag instead of chasing
	 * ft->group->ordered_list_set.  Immutable after cds_ft_create (the group's
	 * mode is fixed at attr time), so a concurrent reader's branch is race-free.
	 * When false the trie allocates NO ordinal cells: a head's prev IS its
	 * flagged parent directly, exactly as in a non-cell build -- the per-key
	 * cell RSS and its list maintenance are paid only when the list is enabled.
	 */
	bool ordered_list;

	/*
	 * Mirror of group->rank_stats_set, cached on the trie so the order-
	 * statistics query hot paths and the per-mutation count propagation
	 * test one local flag instead of chasing ft->group.  Immutable after
	 * cds_ft_create.  When false the trie maintains no per-node nr_keys and
	 * the rank / select / count queries use the enumeration / iteration
	 * fallback.
	 */
	bool rank_stats;

	/*
	 * Effective per-trie speculative-leaf-key state: the group has a
	 * speculative_key_offset AND this trie was NOT created with
	 * cds_ft_attr_set_speculative_keys(attr, false).  When false, every
	 * lookup on this trie reconstructs the result key from the trie
	 * structure (the EAGER path) and NEVER reads the leaf's stored key
	 * field, so the trie may safely hold leaves whose stored key does not
	 * match their position (a staging graft source whose leaves are stamped
	 * with their future destination key; a detach result whose leaves carry
	 * their pre-detach key at a now-stripped position).  Set at create
	 * before ft_install_lookup_ops; immutable after, so a concurrent
	 * reader's branch is race-free.  Mirrors group->speculative_key_offset_set
	 * but per-trie so individual tries in a speculative group can opt out.
	 */
	bool speculative_key_offset_active;

	/*
	 * In-progress compaction state (cds_ft_compact_begin), or NULL.
	 * Set at begin, cleared at end.  Lets cds_ft_compact_begin reject a
	 * second concurrent compaction on the same trie, and cds_ft_destroy
	 * finalize one the caller forgot to end (merging its private ranges
	 * back so they are not leaked).  Written under the caller's writer
	 * exclusion, like every other mutation.
	 */
	struct cds_ft_compact_state *active_compact;

	/*
	 * Per-trie writer-contention escalation domain (<urcu/rcu-txn.h>): a
	 * concurrent-mode op's persistent txn handle binds it (ft_txn_op_init)
	 * so a starved writer escalates into the FIFO fair-mutex lane after
	 * URCU_TXN_FALLBACK aborted attempts (doc/design/
	 * mcas-multiwriter-readiness.md §11).  Per-trie because writer
	 * contention is per-trie (ops on different tries share no slots).
	 * Initialized at create; no destructor (futex/word state only).
	 * Unused (never escalates) on an exclusive trie -- ft_txn_op_init
	 * binds NULL there.
	 */
	struct urcu_txn_domain txn_domain;

	/*
	 * MW COARSE lock-mode FT-wide writer lock (doc/design/
	 * mw-writer-lock-escalation-model.md §10.5 / §11.3 step 2): ONE lock per
	 * trie, taken by every mutator on a lock-mode trie; readers never take it
	 * (classic RCU single-writer).
	 *
	 * A PLAIN FIFO-fair mutex (<urcu/fair-mutex.h>), deliberately NOT a
	 * transacted word driven through the txn engine.  A lock acquire cannot
	 * ride the MCAS/escalation engine: that engine's progress proof assumes a
	 * transaction AT THE LANE HEAD WILL SUCCEED (nothing behind it can
	 * invalidate it), but an acquire cannot succeed until an EXTERNAL event --
	 * the holder's release.  An escalated acquirer would hold its FIFO turn
	 * while spinning, and domain->active then funnels every txn on the domain,
	 * INCLUDING THE HOLDER'S OWN commit and release, into the lane behind its
	 * own waiter: circular wait, plus a fresh MCAS descriptor allocated per
	 * retry (unbounded slab growth).  Both were observed.  A mutex is a mutex.
	 *
	 * GRACE-PERIOD RULE (the invariant rcu-txn.h:1126 also relies on): a
	 * holder must NEVER wait on a grace period while holding this lock -- the
	 * writers parked on it are RCU-online and non-quiescent, so they are
	 * exactly what prevents that grace period from completing (deadlock).  Ops
	 * that synchronize_rcu mid-flight drop and retake the lock across the wait
	 * via ft_writer_lock_gp_wait(); the GP always sits at a seam BETWEEN two
	 * distinct commits, so releasing there costs no atomicity.
	 */
	struct cds_fair_mutex writer_lock;
	/*
	 * Hot-path gate: writer_strategy != CDS_FT_WRITER_OPTIMISTIC (a COARSE
	 * or FINE locking strategy).  Copied from the group at create so the
	 * writer-scope hook branches on one trie field.  false (calloc-zero) =
	 * optimistic = the hook is a no-op.
	 */
	bool lock_mode;
	/*
	 * Hot-path gate: writer_strategy == CDS_FT_WRITER_LOCK_FINE.  The
	 * fine-grained (per-node lock-set) conversions read THIS, not @lock_mode:
	 * COARSE deliberately derives no lock-set (§10.5 -- one FT-wide lock, no
	 * per-node locks), so a per-node acquire there would be pure cost.
	 *
	 * A FINE trie still takes the FT-wide writer lock as well, for now: the
	 * op-domains convert one at a time (§11.3 steps 3-6) and an unconverted op
	 * must not race a converted one (the §11.1 coexistence hazard).  So the
	 * per-node lock-sets acquired below FINE are, until every domain is
	 * converted, exercised under FT-wide serialization rather than contended.
	 * They are dropped from serialization op-domain by op-domain as the
	 * conversions land.
	 */
	bool lock_fine;


	/*
	 * Per-trie circular sentinel of the ordinal-cell list.  The list is a
	 * circular doubly-linked list of cells (struct ft_ord_cell.lnode) anchored
	 * at this embedded sentinel node: ord_sentinel.node.next is the first cell's
	 * lnode (the old ord_cell_head), .prev the last cell's lnode (the old
	 * ord_cell_tail); the first cell's lnode.prev and the last cell's lnode.next
	 * point back at &ord_sentinel.node.  An empty (or disabled) list has the
	 * sentinel pointing at itself (urcu_txn_list_init).  A resolved link that
	 * equals &ord_sentinel.node is "off the end" (ft_ord_is_end); the head/tail
	 * endpoint flips are now just the sentinel node's next/prev edges, produced
	 * naturally by a normal splice whose boundary neighbour is the sentinel.  Read
	 * the endpoints via ft_ord_first / ft_ord_last (NULL when empty).
	 */
	struct urcu_txn_list_head ord_sentinel;

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

/*
 * MW COARSE lock-mode: per-thread FT-wide writer-lock state.
 *
 * @ft_wlock_waiter is this thread's FIFO queue node.  It lives in TLS because
 * cds_fair_mutex_lock() enqueues the node BY ADDRESS and the address must stay
 * stable from lock to unlock -- which happen in two different functions (the
 * writer-scope enter and its cleanup).  @ft_wlock_held names the trie whose lock
 * this thread holds (NULL = none) and doubles as the reentrancy test; a thread
 * holds AT MOST ONE FT-wide lock.  Two complements keep that true for cross-trie
 * graft / merge / graft_swap (the only ops that scope two tries):
 * ft_crosstrie_lock_mode_guard rejects a BOTH-LIVE lock-mode pair, and step 6's
 * exclusive-source contract requires the CONSUMED source (graft/merge src,
 * graft_swap swap) to be EXCLUSIVE -- a live source is rejected with BUSY at the
 * op entry, and an exclusive trie's writer scope SKIPS the lock
 * (ft_writer_lock_scope_enter) -- so a cross-trie op holds only the live dst's
 * lock.  @ft_wlock_depth counts reentrant writer scopes on that one trie
 * (a same-thread nest such as graft_swap -> graft) so the lock is taken once at
 * the outermost enter and released once at the outermost exit.
 */
static __thread struct cds_fair_mutex_node ft_wlock_waiter;
static __thread struct cds_ft *ft_wlock_held;
static __thread unsigned long ft_wlock_depth;

/*
 * Take the FT-wide writer lock at the OUTERMOST writer scope of a lock-mode
 * trie; reentrant no-op on a nested scope for the same trie.  No-op on an
 * optimistic trie (the common path -- one predictable branch).
 */
static inline
void ft_writer_lock_scope_enter(struct cds_ft *ft)
{
	if (caa_likely(!ft->lock_mode))
		return;
	if (ft_wlock_held == ft) {
		ft_wlock_depth++;		/* reentry on the trie we hold */
		return;
	}
	if (ft->exclusive) {
		/*
		 * An exclusive trie is private / single-writer by contract
		 * (step 6, §9.5): no concurrent writers, so no FT-wide lock
		 * is needed.  Skipping it lets a cross-trie op hold only the
		 * LIVE side's lock while the exclusive consumed source (the
		 * caller's cds_ft_make_exclusive'd src, or a detach product)
		 * rides through the fused op body -- one lock at a time, no
		 * cross-trie deadlock.
		 *
		 * Checked AFTER the reentrancy test on purpose: a trie whose
		 * lock we already hold and that then flips exclusive mid-scope
		 * (cds_ft_make_exclusive) must keep depth-counting its nested
		 * scopes.  And ft_writer_lock_scope_exit keys its release off
		 * @ft_wlock_held identity -- never a re-read of @exclusive --
		 * so a mid-scope flip of the flag cannot unbalance the lock.
		 */
		return;
	}
#ifdef FEATURE_FT_MW_LOCK_FINE_DROP
	if (ft->lock_fine) {
		/*
		 * FT-WIDE-LOCK DROP (§11 drop-mechanics, MCAS-first): a FINE trie's
		 * op-domains are ALL converted to per-node lock-sets (COPYING
		 * try-locks) that arbitrate writers directly, so the FT-wide mutex
		 * is redundant -- SKIP it and let the per-node locks be the sole
		 * exclusion.  Dropped ALL-AT-ONCE for FINE (not op-domain by
		 * op-domain): the FT-wide lock sits at the per-OP writer scope, not
		 * a per-domain sub-scope, and one MCAS-abort-safe net covers every
		 * domain uniformly.  Residual §11.1 under-count sites (I-1
		 * in-place nr_child, I-4b skip-dual P, §8.3 word-sharing) stay
		 * MCAS-abort-safe here: expected-old catches the race as a spurious
		 * retry, not a lost update (that safety net is what the later sw
		 * cutover removes, where full lock-set completeness becomes
		 * mandatory).  ft_writer_lock_scope_exit no-ops (keys off
		 * @ft_wlock_held identity, never set) and ft_writer_lock_gp_wait
		 * degrades to a plain synchronize_rcu (held == NULL).
		 */
		return;
	}
#endif
	if (caa_unlikely(ft_wlock_held != NULL)) {
		/*
		 * Two FT-wide locks at once: only a cross-trie op could nest
		 * them, and those are rejected on a lock-mode trie, so reaching
		 * here means that guard was bypassed.  Fail loudly rather than
		 * deadlock (thread 1 holds A wants B; thread 2 holds B wants A).
		 */
		fprintf(stderr, "cds_ft: thread already holds the FT-wide writer "
			"lock of cds_ft=%p while entering cds_ft=%p\n",
			(void *) ft_wlock_held, (void *) ft);
		fflush(stderr);
		abort();
	}
	cds_fair_mutex_lock(&ft->writer_lock, &ft_wlock_waiter);
	ft_wlock_held = ft;
	ft_wlock_depth = 1;
}

/* Release at the OUTERMOST scope only; a nested exit just unwinds the depth. */
static inline
void ft_writer_lock_scope_exit(struct cds_ft *ft)
{
	if (caa_likely(!ft->lock_mode))
		return;
	/*
	 * Unwind only the lock THIS thread actually holds for @ft.  When the
	 * enter SKIPPED the lock (an exclusive trie) or we hold a DIFFERENT
	 * trie's lock, @ft_wlock_held != ft and there is nothing to release.
	 * Keying off identity (not a re-read of @ft->exclusive) means an
	 * @exclusive flip between enter and exit -- cds_ft_make_exclusive
	 * (false->true) or cds_ft_make_concurrent (true->false) -- can never
	 * unbalance us: make_exclusive took the lock at enter (exclusive was
	 * false) and releases it here (held == ft), make_concurrent skipped it
	 * at enter (exclusive was true) and skips it here (held != ft).
	 */
	if (ft_wlock_held != ft)
		return;
	if (--ft_wlock_depth != 0)
		return;			/* nested scope: keep the lock held */
	ft_wlock_held = NULL;
	(void) cds_fair_mutex_unlock(&ft->writer_lock, &ft_wlock_waiter);
}

/*
 * Wait for a grace period from inside a writer scope, DROPPING the FT-wide lock
 * across the wait and retaking it after.
 *
 * This is mandatory, not an optimization: writers parked on the lock are
 * RCU-online and non-quiescent, so they are precisely what stops this grace
 * period from ever completing -- a holder that waits on a GP while holding the
 * lock deadlocks against its own waiters.  (Same invariant rcu-txn.h:1126 leans
 * on: "the lane owner never blocks on a GP while holding the mutex.")
 *
 * Dropping here costs no atomicity: every mid-op grace period in this codebase
 * sits at a seam BETWEEN two distinct commits, with the structure already
 * published-consistent and the subtree being drained already unreachable, so a
 * peer that mutates while we wait sees a coherent trie.  On an optimistic trie
 * (or outside any writer scope) this is a plain synchronize_rcu.
 */
static inline
void ft_writer_lock_gp_wait(struct cds_ft *ft)
{
	struct cds_ft *held = ft_wlock_held;
	unsigned long depth = ft_wlock_depth;

	if (held) {
		ft_wlock_held = NULL;
		ft_wlock_depth = 0;
		(void) cds_fair_mutex_unlock(&held->writer_lock, &ft_wlock_waiter);
	}
	ft->group->flavor->update_synchronize_rcu();
	if (held) {
		cds_fair_mutex_lock(&held->writer_lock, &ft_wlock_waiter);
		ft_wlock_held = held;
		ft_wlock_depth = depth;
	}
}

struct ft_excl_reader_scope {
	struct cds_ft *ft;
#ifdef FEATURE_FT_EXCL_VALIDATE
	bool claimed;	/* true iff we incremented ft->excl_nr_readers. */
#endif
};

#ifdef FEATURE_FT_EXCL_VALIDATE

/*
 * Report an access-discipline violation and abort.  A macro rather than a
 * vfprintf wrapper: the caller's varargs forward straight to fprintf (which
 * format-checks them via its own attribute), and abort() keeps it noreturn.
 */
#define ft_excl_abort(...)						\
	do {								\
		fprintf(stderr, "FT access-discipline violation: "	\
			__VA_ARGS__);					\
		fflush(stderr);						\
		abort();						\
	} while (0)

static inline
void ft_excl_writer_enter(struct cds_ft *ft)
{
	unsigned long self = (unsigned long) pthread_self();
	unsigned long prev, nr;

	/*
	 * MW lock-mode: take the FT-wide writer lock at the outermost scope
	 * before the discipline checks, so they run single-writer.  No-op on an
	 * optimistic trie.
	 */
	ft_writer_lock_scope_enter(ft);
	prev = uatomic_cmpxchg(&ft->excl_owner, 0, self);
	if (prev != 0) {
		if (prev == self) {
			/* Reentry from the same thread (e.g. graft_swap
			 * delegating to graft). */
			ft->excl_writer_depth++;
			return;
		}
		ft_excl_abort("cds_ft=%p: writer conflict -- owner 0x%lx, entering thread 0x%lx\n",
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

static inline void ft_excl_writer_enter(struct cds_ft *ft)
{
	/* MW lock-mode FT-wide lock; no-op on an optimistic trie. */
	ft_writer_lock_scope_enter(ft);
}
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
	 * writer cannot start mutating while we walk the trie.  Runs under
	 * the FT-wide lock (released last, below).
	 */
	ft_writer_scope_verify(*ft);
#endif
	ft_excl_writer_exit(*ft);
	/*
	 * MW lock-mode: release the FT-wide writer lock at the outermost
	 * scope, after the discipline checks / verify walked the trie under
	 * it.  No-op on an optimistic trie.
	 */
	ft_writer_lock_scope_exit(*ft);
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
 * MW lock-mode: hard guard at a CROSS-trie op entry (graft / merge / graft_swap
 * between DISTINCT tries @a and @b, both scoped before their two per-ft writer
 * scopes).  Fires only when BOTH tries are LIVE (non-exclusive) lock-mode: that
 * is the pair whose two writer scopes would take two FT-wide locks with no lock
 * order and could deadlock (thread 1 grafts a->b while thread 2 grafts b->a).
 *
 * Step 6 (§9.5) avoids that pair by CONTRACT: a cross-trie op requires its
 * consumed source (graft/merge src, graft_swap swap) to be EXCLUSIVE, and the
 * public entry rejects a live source with CDS_FT_STATUS_BUSY_ERROR before taking
 * any lock.  An exclusive source's writer scope skips the lock
 * (ft_writer_lock_scope_enter), so one lock is held and there is no deadlock.
 * By the time an op reaches the fused body the source is always exclusive, so
 * this guard is a defense-in-depth assert: a both-live pair reaching here means
 * a BUSY gate was missed (a bug), and it aborts rather than deadlock.
 *
 * A SAME-trie merge (@a == @b) takes one lock reentrantly and is allowed, so
 * this fires only on distinct tries.  An op touching an EXCLUSIVE side is
 * already single-domain (only the live side's lock is taken).
 */
static inline
void ft_crosstrie_lock_mode_guard(const struct cds_ft *a, const struct cds_ft *b)
{
	if (caa_unlikely(a != b
			&& (a->lock_mode && !a->exclusive)
			&& (b->lock_mode && !b->exclusive))) {
		fprintf(stderr,
			"cds_ft: both-live cross-trie op on lock-mode tries reached "
			"the fused body (cds_ft=%p / %p); the source must be "
			"exclusive -- a live source should have been rejected with "
			"CDS_FT_STATUS_BUSY_ERROR\n", (const void *) a, (const void *) b);
		fflush(stderr);
		abort();
	}
}

/*
 * Allocator layout (see fractal-trie-alloc.c for the full picture).
 *
 * Each 2*page_size range holds the items array (page 0), then the
 * per-range header (struct cds_ft_alloc_range) followed by the
 * per-item metadata array (page 1).  Optional bitmap array grows
 * backward from (range base + 2*page_size) on bitmap-bearing arenas.
 *
 * The two cache lines most often prefetched from a tagged child
 * pointer -- the item's metadata and (for bitmap types) its bitmap --
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
	/*
	 * cds_ft_page_size is lazily initialized with an atomic check-and-set
	 * (see the allocator entry points) and never changes afterward, so a
	 * relaxed load suffices: it cannot tear and carries no dependent data.
	 * Routing every read through this accessor keeps all accesses to the
	 * global atomic.
	 */
	return uatomic_load(&cds_ft_page_size, CMM_RELAXED);
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

/*
 * Allocate one ordinal cell from the group's dedicated cell arena
 * (FT_ORD_CELL_ALLOC_ORDER, no bitmap).  Returns a metadata whose item is the
 * 32 B cell (cds_ft_metadata_to_item); freed via cds_ft_free_item.
 */
__attribute__((visibility("hidden")))
struct cds_ft_metadata *cds_ft_alloc_cell_item(struct cds_ft *ft);

__attribute__((visibility("hidden")))
void cds_ft_free_item(struct cds_ft *ft, struct cds_ft_metadata *metadata);

/*
 * Node-allocation reserve API (see struct cds_ft_alloc_reserve).  A bulk op
 * with a deterministic node need pre-fills a zero-initialized reserve with the
 * exact items it will allocate, activates it, runs its commit drawing from the
 * reserve (no arena allocation, no failure), deactivates, then drains any
 * unused items.  Fill is the only fallible step; on its failure the caller
 * drains and aborts with nothing mutated.
 *
 * Add @n items of (@kind, @item_len_order) to @r.  @bitmap is the node type's
 * bitmap flag (order uniquely determines it; ignored for COMPRESSED).  Returns
 * 0, or -ENOMEM (caller drains @r and aborts).  Must run with the reserve NOT
 * yet active (these are real arena allocations).  Asserts the running total
 * fits CDS_FT_ALLOC_RESERVE_CAP.
 */
__attribute__((visibility("hidden")))
int cds_ft_alloc_reserve_add(struct cds_ft *ft, struct cds_ft_alloc_reserve *r,
		enum cds_ft_alloc_kind kind, size_t item_len_order, bool bitmap,
		unsigned int n);

/*
 * Make @r the active reserve for @ft in THIS THREAD's reserve set.  A bulk op
 * allocating into several tries (e.g. merge same-trie rekey: dst + the transient
 * detach product) activates the SAME @r on each; the reserve is thread-local, so
 * a concurrent bulk op on the same live dst owns its own reserve and does not
 * collide (the FT-wide-lock drop, §11, removed the mutex this once relied on).
 * A trie may be activated at most once at a time (asserted).
 */
__attribute__((visibility("hidden")))
void cds_ft_alloc_reserve_activate(struct cds_ft *ft,
		struct cds_ft_alloc_reserve *r);

/* Remove @ft from this thread's active reserve set. */
__attribute__((visibility("hidden")))
void cds_ft_alloc_reserve_deactivate(struct cds_ft *ft);

/* True iff this thread has an active reserve covering @ft. */
__attribute__((visibility("hidden")))
bool cds_ft_alloc_reserve_covers(const struct cds_ft *ft);

/* Free every item still held in @r (the op drew fewer than it reserved). */
__attribute__((visibility("hidden")))
void cds_ft_alloc_reserve_drain(struct cds_ft *ft, struct cds_ft_alloc_reserve *r);

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
	/*
	 * Current private range for the dedicated cell arena (single: cells are
	 * uniform FT_ORD_CELL_ALLOC_ORDER).  Keeps relocated cells off the
	 * internal/compressed cursors of the same order so they pack into their
	 * own dense range.
	 */
	struct cds_ft_alloc_range *cur_cell;
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

/* Always-deferred (call_rcu) free, even on exclusive tries: compactor use. */
__attribute__((visibility("hidden")))
void cds_ft_free_item_deferred(struct cds_ft *ft,
		struct cds_ft_metadata *metadata);

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
 *  iter_debug_path_snapshot() -- unconditionally captures a fresh
 *      grace-period poll state via the RCU flavor's
 *      update_start_poll_synchronize_rcu.  Called once at the entry of
 *      every fresh-population operation (lookup, longest-match lookup,
 *      inequality lookup slow path and early exit).  Because it always
 *      overwrites the snapshot, an iterator that is reused across
 *      distinct RCU read-side critical sections gets a current baseline.
 *
 *  iter_debug_path_check() -- polls the existing snapshot via the
 *      flavor's update_poll_state_synchronize_rcu.  Called at
 *      continuation entry points that consume a previously populated
 *      cached path (inequality lookup fast path, replace, remove).  If
 *      a full grace period has elapsed since the snapshot, the RCU
 *      read-side lock must have been dropped and the cached path is
 *      invalid -- this is reported and abort() is called.
 *
 *  iter_debug_path_update() -- invalidates the snapshot when the path
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

/*
 * Central node-type definitions and small helpers moved here from the
 * fractal-trie.c top matter so the master file is just the assembly unit.
 * (The ft_types[] data table itself stays a single-TU definition in
 * fractal-trie/ft-tables.h -- it cannot live in this multi-TU header.)
 */
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
	/*
	 * Maintain the per-node order-statistics key counts (nr_keys) that back
	 * cds_ft_count_keys / _prefix, cds_ft_lookup_nth / _last, and
	 * cds_ft_iter_skip_forward / _reverse.  Default OFF: when disabled the
	 * library keeps no per-node count and those queries fall back to
	 * enumeration (count) or first/last + next/prev iteration (select /
	 * skip).  See cds_ft_group_attr_set_rank_stats.
	 */
	bool rank_stats_set;
	enum cds_ft_numa_policy numa_policy;	/* See cds_ft_group_attr_set_numa_policy. */
	enum cds_ft_optimize optimize;		/* See cds_ft_group_attr_set_optimize. */
	/*
	 * Structural-writer concurrency strategy (MW lock-escalation model).
	 * Default CDS_FT_WRITER_OPTIMISTIC (calloc-zero).  See
	 * cds_ft_group_attr_set_writer_strategy.
	 */
	enum cds_ft_writer_strategy writer_strategy;
};

struct cds_ft_attr {
	bool exclusive;		/* Exclusive (single-writer, no concurrent readers) vs concurrent; see cds_ft_attr_set_exclusive. */
	bool speculative_keys_disabled;	/* This trie ignores the group's speculative_key_offset (EAGER lookups); see cds_ft_attr_set_speculative_keys.  calloc default false = inherit the group. */
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

#endif /* _URCU_FT_INTERNAL_H */
