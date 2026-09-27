// SPDX-FileCopyrightText: 2012-2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-only

#ifndef _URCU_FT_INTERNAL_H
#define _URCU_FT_INTERNAL_H

/*
 * For FT_DT_ARMED alone: fractal-trie-alloc.c includes ONLY this header, and
 * the non-owner-clear detector's forget hook lives there.  Without a shared
 * gate the allocator would compile that forget out of an armed build and every
 * recycled address would read as a stolen lock.
 */
#include "ft-dt-arm.h"

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
 *   FEATURE_FT_INSERT_IN_PLACE  Point inserts append into the live node under
 *                               the held lock instead of recompacting it.
 *                               Disable with -DNO_FEATURE_FT_INSERT_IN_PLACE.
 *   FEATURE_FT_DELETE_IN_PLACE  Point deletes soft-delete from the live node
 *                               under the held lock instead of recompacting.
 *                               Disable with -DNO_FEATURE_FT_DELETE_IN_PLACE.
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
#include <limits.h>
#include <unistd.h>
#include <urcu/list.h>
/*
 * FT parks a concurrent MCAS proxy (struct urcu_txn_record *, 16-byte
 * aligned -> low 4 bits free) under its own type-7 / 0xF pointer tag (see
 * FT_FLIP_PROXY_TAG in ft-helpers.h) rather than the engine's bit-0 default:
 * bit 0 alone marks an internal node here, so a bit-0 proxy would alias one,
 * whereas a 0xF low nibble is an encoding no real node, NULL or skip pointer can
 * carry.  The tag is now carried PER RECORD -- FT passes FT_FLIP_PROXY_TAG to
 * every urcu_txn_store()/load_validate() (see ft_flip_txn_commit), the engine
 * stores it in urcu_txn_record.proxy_tag and forms the parked proxy as
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

/*
 * THE FT OP HANDLE.  One per operation, spanning its retry loop.  @txn is the
 * commit engine's handle: each attempt opens and closes it (ft_op_begin /
 * ft_op_end), and a flip txn built on the op (ft_flip_txn_create_on) records
 * into it.  The op is FT's own type so that FT policy -- the escalation lane
 * that serializes a starving op, and the retry age that earns it -- can live
 * with FT rather than with one engine: the SW engine (<urcu/rcu-txn-sw.h>)
 * has no lane.
 */
struct ft_op {
	struct urcu_txn txn;
};
#include <urcu/rculfhash.h>
#include <urcu/arch.h>
#include <urcu/assert.h>	/* urcu_assert_debug: the engine self-checks' arm */
#include <urcu/call-rcu.h>
#include <urcu/uatomic.h>
#include <urcu/fractal-trie.h>

#include <assert.h>

/*
 * FT-wide-lock DROP is the DEFAULT for a FINE-locking trie (§11 rollout,
 * 2026-07-19).  A FINE trie skips the FT-wide writer mutex and relies solely
 * on its per-node node lock-sets + MCAS arbitration for writer exclusion --
 * so disjoint writers run in parallel instead of serialising on one mutex
 * (doc/design/ft-wide-lock-drop-mechanics.md; certified by the §11.4 point-op
 * 1600/1600 gate and the cross-trie oracles).  COARSE tries are unaffected
 * (COARSE keeps the mutex).
 *
 * This is now UNCONDITIONAL: the drop was the only shipping behaviour, and its
 * opt-out (-DFEATURE_FT_MW_LOCK_FINE_KEEP) had no gate config, so the retained
 * mutex was untested by construction.  Both flags are gone -- FINE always
 * drops the FT-wide mutex, COARSE always keeps it, decided at RUNTIME by the
 * group's writer strategy.
 */
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

/*
 * The longest run ONE compressed node can spell: its 8-bit len.  A run past
 * FT_SKIP_LEN_MAX is not unspellable -- ft_publish_compressed publishes it
 * through a PLAIN compressed flag, which is the shape cds_ft_insert builds for
 * a long unique suffix and cds_ft_verify accepts.  A site that FUSES runs caps
 * the fused length at this, never at FT_SKIP_LEN_MAX: declining there leaves
 * the one-child internal or the adjacent compresseds the fuse exists to
 * remove.  -DFT_DEBUG_OVERLONG_BULK_CAP restores the old cap (red control).
 */
#ifndef FT_DEBUG_OVERLONG_BULK_CAP
# define FT_CN_LEN_MAX		UINT8_MAX
#else
# define FT_CN_LEN_MAX		FT_SKIP_LEN_MAX
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
 * FEATURE_FT_INSERT_IN_PLACE: in-place occupancy-bitmap safe-append (the O(1)
 * insert tier).  FEATURE_FT_DELETE_IN_PLACE: in-place soft-delete (the delete
 * tier).  BOTH ON BY DEFAULT since 2026-09-26, and independent (split the same
 * day -- the one macro used to enable both -- so root-cause work can switch
 * either tier alone): -DNO_FEATURE_FT_INSERT_IN_PLACE /
 * -DNO_FEATURE_FT_DELETE_IN_PLACE turn one back into recompact-on-insert /
 * recompact-on-delete.
 *
 * With the insert tier, a POINT insert that lands at a node's tail rank with
 * spare tier capacity is applied IN PLACE: the child slot is published and the
 * node's occupancy-bitmap bit is set with a relaxed store on the LIVE node's
 * body (ft_popcount_node_set_nth Cases 2A/2B, ft_pigeon_node_set_nth).  With
 * the delete tier, a point delete that leaves the holder above min_child NULLs
 * the slot and decrements nr_child through the commit instead of rebuilding
 * the node, on shared tries too (the three point-remove ft_detach_node calls,
 * since 2026-09-16).  Delete-on / insert-off is a supported mix: refilling a
 * soft-deleted hole is itself an insert-tier store, so with the insert tier
 * off the refill reports -ERANGE and the recompact drops the hole.  On EVERY
 * trie type: what makes the insert store safe
 * against concurrent WRITERS is that the op HOLDS the node before it writes --
 * the node's DLM lock, or its anchor at a coarser spacing, or the FT-wide lock
 * under the coarse strategy (ft_attach_node's acquire hoist; ft_in_place_ok's
 * header) -- and what makes it safe against concurrent READERS is that an
 * in-place mutation never removes a bitmap bit, so no existing rank moves.
 * The bulk ops (graft / merge / rekey reserves and detaches) and the build-path
 * wrapper stay on the exclusive-only tier (ft_in_place_excl_ok) until each is
 * converted to lock-before-write; a same-trie move's dst attach parent must
 * relocate on a shared trie anyway, for the reader-coherence witness.
 *
 * Without the insert tier every new-occupancy insert (i.e. not an
 * in-place pointer replace at an already-occupied slot) instead reports -ERANGE,
 * so the setter wrapper (ft_node_set_nth_rec) routes it through
 * ft_node_recompact(ADD_SAME) -- a fresh node carrying the new entry AND its
 * bitmap is built build-invisibly and the parent edge is flipped, exactly as a
 * non-tail insert already does.  That was the shape the MW-with-CAS model
 * needed (the Invariant-2 disjoint-word hazard, see
 * doc/design/mcas-multiwriter-readiness.md S4): every node change a whole-node
 * replacement via the parent flip-txn edge, so one CAS could arbitrate every
 * word.  The DLM locks now provide that exclusion, and that build still pays
 * the cost: the O(1) in-place insert turned into an O(node) alloc-and-copy
 * recompact.  See doc/design/ft-reintroduce-in-place-mutations.md.
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
 * Default: both tiers on.  Lock-before-write is what makes them safe against
 * concurrent writers, and the sticky bitmap bit against readers (above).  The
 * gate keeps the recompact paths covered with its no-in-place,
 * no-in-place-insert and no-in-place-delete configs.
 */
#if defined(NO_FEATURE_FT_INSERT_IN_PLACE) && defined(FEATURE_FT_INSERT_IN_PLACE)
# error "FEATURE_FT_INSERT_IN_PLACE and NO_FEATURE_FT_INSERT_IN_PLACE both defined"
#endif
#if defined(NO_FEATURE_FT_DELETE_IN_PLACE) && defined(FEATURE_FT_DELETE_IN_PLACE)
# error "FEATURE_FT_DELETE_IN_PLACE and NO_FEATURE_FT_DELETE_IN_PLACE both defined"
#endif
#if !defined(NO_FEATURE_FT_INSERT_IN_PLACE) && !defined(FEATURE_FT_INSERT_IN_PLACE)
# define FEATURE_FT_INSERT_IN_PLACE
#endif
#if !defined(NO_FEATURE_FT_DELETE_IN_PLACE) && !defined(FEATURE_FT_DELETE_IN_PLACE)
# define FEATURE_FT_DELETE_IN_PLACE
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
	 * ☞ THE ONE FAMILY WHOSE MW IS ARGUED, NOT ASSERTED (register note 6):
	 * @lnode's two links have NO owning node lock at per-node OR exponential
	 * spacing, because a splice rewrites the cells of the NEIGHBOURING KEYS,
	 * whose holders this op never acquires.  MW by design, at every spacing;
	 * only a root-only lock-set or the FT-wide bulk gate would cover them.
	 * The deletion MARK on a retired cell's next is load-bearing: it is what
	 * a peer splice that captured that dead cell as pred/succ fails on.
	 * ☠ Feed expected-olds through the txn handle, not through
	 * ft_ord_cell_resolve_ord: an optimistic read of a slot this txn also
	 * WRITES is a doomed install under contention (aborts, not corruption).
	 *
	 * @parent, by contrast, HAS a single owner -- the holder P -- and is MW
	 * only by debt; it is also still written raw on ft_set_parent's path, so
	 * both paths are legal only while each holds P or the bulk gate.
	 *
	 * Key-ordered doubly-linked list links, embedded as the public
	 * concurrent bidir-list node (<urcu/rcu-txn-list.h>): lnode.next
	 * is the old ord_next, lnode.prev the old ord_prev.  The cell is recovered
	 * from a link with ft_ord_cell_of() (container_of) and its link node with
	 * ft_ord_cell_lnode().  Embedding the public node lets the single-cell
	 * splice/unsplice/replace ride the list's composable _prepare ops, folded
	 * into the FT structural flip-txn; the cell edges carry the concurrent
	 * list's engine proxy tag (URCU_TXN_TAG, bit 0) and readers resolve them
	 * with urcu_txn_list_resolve (ft_ord_cell_resolve_ord).
	 */
	struct urcu_txn_list_node lnode;
	struct cds_ft_node *node;
	struct cds_ft_inode_flag *parent;
};

/*
 * PREFIX-HEAD bit on an external head's OWN parent word.
 *
 * A head's key is reconstructed structurally (ft_rebuild_key_upwalk,
 * ft_upwalk_edge_bytes) from the word that names its parent, and the ONE fact
 * that word did not carry is which of two shapes the head has: a SLOT head
 * (key = path(parent) + its own edge byte) or a PREFIX head sitting at the
 * parent's external_nodes (key = path(parent), the key ENDS at the parent).
 *
 * ☠ THE SHAPE ANSWER MUST TRAVEL WITH THE HEAD, NOT WITH THE HOLDER.  Inferring
 * it from the holder's LIVE external_nodes is a second word, and a key-
 * disappearing remove (or a replace) of a prefix head DESTROYS that answer IN
 * PLACE while a parked reader still holds the head: the dead prefix head then
 * reads as a slot head and its key gains a spurious trailing byte -- a LEGAL
 * key of the trie, on the WRONG node.  Measured, default build, one writer:
 * a present key answering NOT_FOUND, and a key absent at every instant
 * answering with a neighbour's node.
 *
 * So the bit is carried by the head itself, on the word it already loads:
 * cell->parent with the ordered list ON, node->prev with it OFF -- the SAME
 * encoding either way, so the reader needs no ordered_list dispatch and no
 * second load, and the answer is snapshot-consistent with the pointer by
 * construction.
 *
 * ★ SET AT INSTALL, NOT AT REMOVAL.  The bit is written when the head is
 * INSTALLED at an internal holder's external_nodes (ft_publish_external_nodes_
 * prev and the insert/replace install sites), which is the only moment the
 * shape is decided; a re-home CARRIES it, a re-home into a body SLOT clears
 * it, and a head whose holder later drops it is never written again -- so a
 * dead prefix head keeps its last correct answer for as long as a parked
 * reader can reach it.  That is what makes the removal-time mark unnecessary.
 *
 * ★ AND IT IS VERIFIABLE AT REST: cds_ft_verify asserts, on every LIVE head,
 * bit == (holder->external_nodes == head).  An install site that forgets the
 * bit -- or a re-home that carries a stale one -- is an ENUMERABLE invariant
 * violation, not a rare reader-visible wrong answer.
 *
 * Bit 4: above the four in-band tag bits (FT_PARENT_TAG_MASK) and below the
 * smallest internal node's alignment (order 5, 32 bytes) -- and a prefix head
 * only ever hangs at an INTERNAL node (a compressed node carries no
 * external_nodes), so the bit never aliases a compressed node's 16-byte
 * alignment, and a re-home under a compressed parent drops it by construction.
 */
#define FT_PARENT_PREFIX_HEAD	\
	((uintptr_t) (FT_INTERNAL_MASK | FT_TYPE_MASK) + 1)

urcu_static_assert(!(FT_PARENT_PREFIX_HEAD &
			(FT_INTERNAL_MASK | FT_TYPE_MASK | FT_COMPRESSED_MASK)),
		"the prefix-head bit must not overlap the parent tag nibble",
		ft_parent_prefix_head_above_tag);
/*
 * ★ AND THE ALIGNMENT PREMISE IS MACHINE-CHECKED, not left to the comment
 * above.  The mark is only sound because every INTERNAL node is allocated at
 * ft_types[].order >= FT_INTERNAL_ORDER_MIN (5 = 32 B) and the allocator hands
 * out order-aligned items (item = range_base + (n << order), range_base page-
 * or 2 MiB-aligned), so bit 4 is never part of an internal node's address.
 * FT_ALLOC_ORDER_MIN (4 = 16 B) is the COMPRESSED tier, which is exactly why
 * the helpers below refuse to touch a compressed flag.  Raising the mark or
 * lowering FT_INTERNAL_ORDER_MIN breaks this and must fail the build.
 */
urcu_static_assert(FT_PARENT_PREFIX_HEAD <
			((uintptr_t) 1 << FT_INTERNAL_ORDER_MIN),
		"the prefix-head bit must fit below the smallest internal node's alignment",
		ft_parent_prefix_head_below_internal_align);

/*
 * All four helpers take a RESOLVED parent word (no flip proxy: a descriptor
 * pointer's bit 4 is address) and read the bit ONLY on an INTERNAL flag: a
 * compressed node is 16-byte aligned, so bit 4 of a compressed parent flag is
 * part of its address and must never be tested, set or cleared.
 */
static inline
bool ft_parent_prefix_head(const struct cds_ft_inode_flag *parent)
{
	return ((uintptr_t) parent & (FT_INTERNAL_MASK |
			FT_PARENT_PREFIX_HEAD)) ==
		(FT_INTERNAL_MASK | FT_PARENT_PREFIX_HEAD);
}

static inline
struct cds_ft_inode_flag *ft_parent_prefix_strip(struct cds_ft_inode_flag *parent)
{
	if (!((uintptr_t) parent & FT_INTERNAL_MASK))
		return parent;
	return (struct cds_ft_inode_flag *)
		((uintptr_t) parent & ~FT_PARENT_PREFIX_HEAD);
}

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
struct ft_lock_ctx;

struct ft_pub_rec {
	/*
	 * ☞ THE OP'S LOCK CONTEXT, carried for the SAME reason @root and
	 * @owner are: the producer is the only frame that has it, and the
	 * replays are several and far away.  Consumed ONLY by
	 * FT_OWNER_ASSERT_OWNED_CTX, which needs the WIDE witness --
	 * ft_flip_txn_owns reads @locks[] alone, so a hold filed in @extra, in
	 * the glue, or in an OUTER frame answers false and the assert fires on
	 * a word the op really does own.  ft_lock_ctx_holds consults all four.
	 *
	 * MEASURED: the SKIP_X dual's acquire exits SHARED 533642 times against
	 * REGISTERED 148705 over one ft_inv run -- 78% of the honest "I hold
	 * this" answers are invisible to the narrow witness.  NULL where the
	 * producer has no ctx, which leaves the assert exactly as narrow as it
	 * was.
	 */
	const struct ft_lock_ctx *ctx;
	struct cds_ft_inode_flag **slot[3];
	struct cds_ft_inode_flag *old_val[3];
	struct cds_ft_inode_flag *new_val[3];
	/*
	 * Per-edge: this slot is a TRIE ROOT (&ft->root), so every replay of
	 * this rec must record it MW (ft_flip_txn_record_root).  A root lives
	 * in no node, so no lock-set can own it and no structural_sw op may
	 * park it.  The publish is the only place that KNOWS -- it branches on
	 * the NULL parent already (the root_publish tracepoint) -- and the
	 * replays are several and far away, so the answer travels with the
	 * edge rather than being re-derived at each of them.
	 */
	bool root[3];
	/*
	 * Per-edge: the node whose DLM lock OWNS this slot (§8), for the
	 * record-time owner check every replay of this rec runs
	 * (FT_OWNER_ASSERT_OWNED).  Carried per EDGE for the same reason
	 * @root is: the publish is the only place that knows, and the replays
	 * are several and far away.
	 *
	 * NULL is the SAFE default and it means "this producer does not name
	 * an owner" -- the record is then never eligible for a per-op SW park,
	 * which is the all-MW behaviour every unconverted path already has.
	 * ☠ It is NOT the mirror of @root: an unset @root would wrongly PARK a
	 * root, while an unset @owner only declines to convert.
	 */
	struct cds_ft_metadata *owner[3];
	/*
	 * Per-edge: does the OP HOLD @owner's DLM lock?  The third answer of
	 * the parentage triple, and the one @owner cannot stand in for.
	 *
	 * ☠ A NULL @owner DOES NOT FAIL CLOSED.  The dispatching recorder
	 * (ft_flip_txn_record_tag) branches on @t->structural_sw ALONE; @owner
	 * feeds the debug assert and the counters and nothing else.  So an
	 * armed txn parks an unnamed slot SW just as readily as a named one,
	 * and in a build without --enable-rcu-debug it does so silently.  The
	 * comment that used to stand on @owner -- "the record is then never
	 * eligible for a per-op SW park" -- was a STALE MECHANISM.
	 *
	 * This is the word that decides: false => the replays record MW, which
	 * is stricter and always sound.  The FORWARD edge inherits the
	 * caller's own @slot_owner_nf declaration; the SKIP_X DUAL cannot, and
	 * that is the whole reason this field exists -- its owner is the
	 * GRANDPARENT, DERIVED inside the publish helper from a back-pointer,
	 * so only the op can say whether it acquired it.
	 *
	 * ☠ AND TODAY EVERY PRODUCER SAYS FALSE -- all twelve
	 * _ft_publish_to_parent callers plus ft_node_recompact's own dual site,
	 * the insert lane included.  The text that used to stand here, "
	 * ft_detach_node's republish does (the recompact takes {C,P,GP} exactly
	 * when P is compressed)", was a STALE CLAIM: that site passes false and
	 * says why in capitals, because the acquire is a PLAN-TIME fact while
	 * the dual's owner is DERIVED FRESH at publish (ft-remove.h, the
	 * "☠ FALSE, AND `old_recompacted_node != NULL` WAS NOT SOUND" note).
	 *
	 * ☞ SO THIS FIELD IS ABOUT KIND ONLY, AND NEVER ABOUT EXCLUSION.  The
	 * dual's home is an MW slot globally; holding GP is what EXCLUDES a peer
	 * recompaction from copying that body out from under the record, and
	 * that is ft_lock_skip_dual_gp's job (§9.3's third lock-set member), not
	 * this word's.  A reader of this field who concludes "false, so the
	 * record is MW, so it is safe" has answered the kind question and left
	 * the exclusion one open -- which is exactly how the stale-dual livelock
	 * got in.
	 */
	bool owner_held[3];
	unsigned int n;
	/*
	 * The commit engine handle this rec's edges will be recorded into, when
	 * the caller has one.  It is not plumbing for the record -- the caller
	 * does that itself -- it is what lets the SKIP_X dual slot be derived
	 * READ-YOUR-OWN-WRITES.
	 *
	 * The dual lives in the compressed parent's OWN parent, at the offset
	 * that parent's metadata records; both words are ordinary transacted
	 * slots, so an op that RE-PARENTS the compressed node in this very txn
	 * has pending edges on them and a raw derivation answers with the
	 * PRE-OP slot -- a word inside the node the same commit retires.  The
	 * refreshed dual then lands in the copy nobody will read, and the live
	 * trie keeps a dual naming a superseded child.
	 *
	 * NULL keeps the raw derivation (a direct-store publish has no txn to
	 * consult, and no pending edges to miss).
	 */
	struct urcu_txn *mtxn;
};

/*
 * Struct layout, ONE WORD PER LINE so it holds on both ABIs (LP64 byte
 * offsets bracketed; 48 B on LP64, 24 B on ILP32 -- the same six words at
 * half the width, zero internal padding):
 *   word 0 [ 0]: parent_word
 *   word 1 [ 8]: external_nodes
 *   word 2 [16]: parent_slot_offset -- its OWN word since the §8.3 split
 *   word 3 [24]: nr_keys
 *   word 4 [32]: state -- nr_child lives here (bits 2-10), not in a bitfield
 *   word 5 [40]: alloc_index + incoming_byte packed, plus tail padding
 *
 * In cds_ft_metadata_alloc (64 B on LP64 -- one cache line), rcu_head is a
 * SEPARATE field placed before the metadata union: it overlaps no metadata
 * field, so every metadata field stays valid for the whole RCU grace period.
 *
 * The union's OTHER member has no such property, and the difference matters.
 * @free_list_next occupies exactly the bytes of @parent_word, so returning an
 * item to its range freelist overwrites the parent link with a pointer to
 * another cds_ft_metadata_alloc.  That write lands only AFTER the grace
 * period -- cds_ft_free_item defers it through call_rcu, and the
 * exclusive-mode synchronous push has no concurrent readers by construction
 * -- so no reader that respects its pin can observe it.  A reader that does
 * NOT, one holding a reference past its grace period, reads a freelist link;
 * and because that link is >= 8-byte aligned, ft_node_external() and every
 * other structural predicate accept it.  It does not fault at the load, it
 * MASQUERADES as a legal external parent and faults later, in
 * cds_ft_item_to_metadata, on an address in the metadata region.
 *
 * So the overlap is what makes such a use-after-free LOUD.  Separating the
 * union would leave @parent_word holding the node's last real parent, and the
 * same stale reader would then walk a plausible wrong tree in silence.
 */
/*
 * Per-node MCAS state word (struct cds_ft_metadata.state) bit layout.
 * bit 0 = proxy (in-band flip marker), bit 1 = tombstone (LIVE->DEAD),
 * bits 2-10 = nr_child, bits 11-18 FREE, bit 19 = LOCK
 * (the reversible per-node writer lock; bits 20+ free).
 * ☞ bits 11-18 no longer hold parent_slot_offset -- it moved to its own word
 * (cds_ft_metadata::parent_slot_offset); see the FT_PSO_* block below, which is
 * where that split and its reason are recorded.
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
 * Bits 11-18 are FREE.  They used to hold parent_slot_offset, which now lives in
 * its own word (cds_ft_metadata::parent_slot_offset, FT_PSO_* below).  The old
 * packing existed so a re-home committed the parent edge and the offset "as ONE
 * atomic MCAS state edge" -- but @parent was ALWAYS a separate word, so that
 * pairing never came from the packing: it comes from both edges riding one
 * flip-txn (ft_reparent_record_meta records &meta->parent and the offset word
 * into the same commit), and ft_get_parent_slot's reader-side coherence loop
 * validates the pair explicitly (same-descriptor check + a parent re-read).
 * Splitting therefore preserves the guarantee while ending the CROSS-LOCK RMW
 * that sharing forced: nr_child is NODE-owned, parent_slot_offset is
 * PARENT-owned per the edge principle (doc §8.3), and one word cannot be owned
 * by two locks.  It also removes a SELF-DEADLOCK: the offset setter had to spin
 * on FT_STATE_INPLACE_WAIT_MASK, which includes FT_STATE_LOCK
 * unconditionally, so an op holding that node's own lock waited on
 * itself (see the reverted b20c471e).  The offset word carries no LOCK bit,
 * so its wait is over the engine proxy alone.
 */
#define FT_PSO_SHIFT			1	/* bit 0 stays clear: engine proxy tag */
#define FT_PSO_BITS			8
#define FT_PSO_VALMASK			(((uintptr_t) 1 << FT_PSO_BITS) - 1)
#define FT_PSO_ENCODE(off)		(((uintptr_t) (off) & FT_PSO_VALMASK) \
						<< FT_PSO_SHIFT)
#define FT_PSO_DECODE(word)		((unsigned int) (((uintptr_t) (word) \
						>> FT_PSO_SHIFT) & FT_PSO_VALMASK))
/*
 * FT_STATE_LOCK (bit 19, above parent_slot_offset): the REVERSIBLE per-node
 * WRITER LOCK (MW lock-escalation model, §0).  CAS to acquire, held across the
 * build/reparent plan window, -EAGAIN on contention, resolved at commit.
 *
 * ACQUIRE: a standalone CAS {clean -> |LOCK}, taken BEFORE the first read of
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
 *   RETIRE  {LOCK|s -> TOMBSTONE|s}  ft_flip_txn_record_tombstone_locked()
 *           The holder copied the node away; it dies at the commit.  One-way
 *           tombstone semantics (exactly-once retire token, freeze-on-free) are
 *           UNCHANGED.
 *   RELEASE {LOCK|s -> s}            ft_flip_txn_record_release_lock()
 *           The holder only needed exclusion; the node SURVIVES the commit.
 *           This is what a lock-set member that is edited but not retired needs
 *           (recompact's parent P, §9.3) -- a lock that unlocks.
 *
 * On ABORT / a pre-commit bail the bit is CAS-cleared instead (the registry
 * drain, ft_flip_txn_lock_release_all) and the node stays live -- so the
 * ABORT terminal and the RELEASE terminal agree on the resulting word, they
 * differ only in who writes it (a bare CAS vs the atomic commit).
 *
 * Unlike the tombstone, LOCK is reversible BY DESIGN and never implies
 * death; it is never set at rest (ft-verify.h reports a leaked lock).
 */
/*
 * Bit 19, pinned LITERALLY rather than derived from any other field's width:
 * moving the writer lock -- to bit 11, say -- would otherwise be an invisible,
 * silently-compiling change to the meaning of every state word.
 * Bits 11-18 stay free (see the free-bits note above).
 */
#define FT_STATE_LOCK		((uintptr_t) 1 << 19)
#define FT_STATE_TAG_MASK		(FT_STATE_PROXY | FT_STATE_TOMBSTONE)

/*
 * FT_STATE_INPLACE_WAIT_MASK: the state-word bits on which the standalone
 * in-place CAS primitives (ft_meta_nr_child_inc / _dec, ft_meta_nr_child_dec_flip
 * via ft_meta_state_transition, ft_meta_parent_slot_offset_set) must WAIT (spin
 * + re-derive) rather than CAS blindly, because a concurrent writer is
 * mid-operation on the word:
 *   - FT_STATE_PROXY: an engine MCAS flip is parked here (the word holds a record
 *     pointer, not a plain state); CASing would write f(latch-pointer) back.
 *     Always waited out, in every build.
 *   - FT_STATE_LOCK: a peer holds the per-node writer lock
 *     and may be SW-parking this word (a retire / lock release / re-home) in an
 *     in-flight mixed sw/mw commit.  An SW park is a plain store that never
 *     validates, so a racing CAS here would clobber it (<urcu/rcu-txn.h>: "if any
 *     other transaction may store_mw() the same slot, this park races that CAS").
 *     Waiting for the holder's commit to settle the word first, then CASing onto
 *     the settled value, is correct: after a RELEASE the word is the pre-lock
 *     snapshot (a legitimate +/-1 lands correctly); after a RETIRE the word is
 *     TOMBSTONE|snap (a harmless bump on an about-to-be-freed node, and the
 *     racing op's own §4.B guard then aborts and re-descends past the dead node).
 *
 * INVARIANT this relies on (exhaustively audited 2026-07-23): NO op calls these
 * standalone primitives on a node it ITSELF holds LOCK on.  A LOCK-held
 * node's count / offset change always rides the flip-txn as a RECORDED edge
 * (ft_flip_txn_record_tag_mw / ft_flip_txn_record_release_lock / ft_flip_txn_record_count_
 * parent), never these primitives -- so the spin is on a PEER's lock only and
 * cannot self-deadlock.  A future guard->lock conversion that routes a locked
 * node's nr_child through these standalone primitives (instead of a recorded
 * edge) would break this and must not be done.
 *
 * LOCK IS IN THE MASK UNCONDITIONALLY -- per-node lock-sets are the only
 * multi-writer implementation, so there is no build in which this bit is
 * merely a copy fence.  The consequence is load-bearing: an in-place nr_child
 * update SPINS while a peer holds the lock, i.e. it HONORS the lock rather
 * than racing it.
 */
#define FT_STATE_INPLACE_WAIT_MASK	(FT_STATE_PROXY | FT_STATE_LOCK)

/*
 * ============================================================================
 * THE TRANSACTED-SLOT REGISTER: who owns each word, and what KIND it takes
 * ============================================================================
 *
 * Every word below is written through the txn engine, so every one of them
 * answers three questions -- and they are THREE, not two.  @owner names the
 * node whose lock excludes other writers (§8.2 "a node's body is its own");
 * @owner_held says whether THIS op holds it, which picks the record KIND; and
 * neither is the same as "does this op EXCLUDE the word", which is what a
 * missing lock or guard loses.  The stale-SKIP_X-dual livelock came from
 * answering the first two and skipping the third.
 *
 * KIND IS A PROPERTY OF THE SLOT, NOT OF THE RECORD.  A slot is SW (parked
 * under a held lock; the park cannot fail) XOR MW (CAS-arbitrated).  One thread
 * parking SW while another CASes MW is a lost or torn write the engine cannot
 * detect.  MW is always safe; SW is a PROMISE of exclusion.
 *
 * ☞ AND THERE IS EXACTLY ONE LEGITIMATE MIX ON ONE WORD -- the LOCK-BEARING
 * WORD PROTOCOL, which the @state rows below USE and which the old blanket
 * phrasing ("the same kind in EVERY txn and EVERY thread") contradicted:
 *
 *     an MW record TAKES the lock, and every write to that word AFTER it, while
 *     the lock is held, may be SW -- the bit updates, the other sub-fields, and
 *     the SW release that drops the lock.
 *
 * The MW take is what establishes the exclusion, so everything sequenced after
 * it is already excluded and needs no CAS.
 *
 * ☞ AND STAYING MW AFTER THE TAKE IS CORRECT, JUST WASTEFUL.  The protocol
 * PERMITS the SW writes, it does not require them: a CAS under an exclusion
 * that already holds simply arbitrates against nobody and still lands the right
 * value.  So converting these to SW is an EFFICIENCY step, and a producer left
 * MW is never a correctness bug -- which is why every row here may sit at MW
 * indefinitely and why MW is the safe default for a new one.  What IS a bug is
 * the reverse pairing: an SW park on a word some OTHER writer may CAS, because
 * the park does not arbitrate and the CAS's expected-old was read before it.
 *
 * ☠ IT DOES NOT HAVE TO BE ONE TXN:
 * txn 1 takes the lock, txn 2 changes state under it, txn 3 unlocks, and that is
 * correct -- the lock, not the txn boundary, is what holds across them.  The
 * ORDER is the whole content of the rule: MW-then-SW is the protocol, SW-then-MW
 * is the bug, which is exactly what the engine's debug police now trap
 * (urcu_txn_record_chain accepts `r->kind == kind || r->kind == MW`).
 *
 * ⇒ SO THE RULE SPLITS BY WHETHER THE WORD CARRIES ITS OWN LOCK:
 *
 *   - A LOCK-BEARING word (@state): the protocol above.  Its rows legitimately
 *     read MW for the LOCK take and SW for the tombstone / release / nr_child++
 *     that follow it on the SAME word.  That is not a kind conflict.
 *   - Every OTHER word (an internal body child slot, the SKIP_X dual, the chain
 *     pointers, @external_nodes, the back edge): NOTHING written to the slot
 *     establishes exclusion over it -- the exclusion lives in ANOTHER word's
 *     lock -- so there is no MW take to sequence behind, and every writer must
 *     agree.  A conversion to SW is legal only once EVERY producer holds the
 *     owner, never one lane at a time.
 *
 * ☠ Do not read the first bullet as a licence for the second.  A body word has
 * no lock bit, so "the lock serialises an SW park against an MW CAS" is FALSE of
 * it: the peer's CAS is arbitrated against nothing.  See <urcu/rcu-txn.h>
 * urcu_txn_store_sw -- "a slot is SW xor MW, globally" -- which is the rule for
 * every word that cannot take a lock in its own bits.
 *
 * WHEN SW IS REACHABLE AT ALL -- three doors:
 *   1. ft_txn_content_sw_ok(): a COARSE or exclusive trie is armed SW
 *      trie-wide, at EVERY spacing.
 *   2. ft_flip_txn_arm_per_op(): under LOCK_FINE, at EVERY SPACING since
 *      2026-09-17, and the KIND IS THEN DECIDED PER RECORD -- a record parks
 *      SW only where the txn owns its owner (__ft_flip_txn_record_tag_ctx),
 *      and falls back to MW otherwise.
 *      ☞ This door used to refuse every spacing but per-node, and the reason
 *      was the PREDICATE: ft_flip_txn_owns compared the registry's words
 *      against the record's node, while a coarse spacing registers that
 *      node's ANCHOR.  The acquire's answer now travels with the
 *      registration (ft_held_anchor's @member, plus @covered for a member
 *      whose word deduped onto another's, plus the root-only anchor, which
 *      needs no descent), so the answer is exact where the acquire filed it
 *      and conservative beyond it.  The old spacing column's "MW by constr."
 *      rows are therefore SW-capable now; what still decides them is
 *      ownership, record by record.
 *   3. ft_flip_txn_arm_structural(): the rekey writer and the root-COW driver
 *      arm SW under FINE at ANY spacing (self-labelled "Phase E's DEBT").
 *      This is the one door through which an SW park and an MW CAS can meet
 *      on one structural word; what keeps them apart today is the FT-wide
 *      bulk gate, not the DLM.  Such a writer is marked @sw_body and is
 *      EXEMPT from door 2's per-record gate: its edges sit on words it fenced
 *      itself, so falling back to MW would abort its own commit forever.
 *
 * ☠ OWNERSHIP IS TAKEN, NEVER OBSERVED.  An MW {live->live} VALIDATE on a word
 * a peer holds is not arbitrated against that peer's SW parks: the validator
 * is the loser, its proxy overwritten and its settle-back CAS silently failing.
 * A guard is a weaker representative of a lock only for a word nobody copies
 * under it; it is NEVER a substitute for the take.  See the state-word
 * protocol in ft-mutation-helpers.h.
 *
 * ☠ AND THE ANCHOR MOVES THE WORD.  Under a coarse spacing ft_anchor_meta maps
 * a lock-set member to an ANCHOR ANCESTOR, so the word a peer must contend for
 * is NOT the node's own state word.  A fix that validates a node's own word is
 * therefore untested above per-node by construction; an ACQUIRE gets §1's
 * AGREEMENT property (every op resolves the same anchor) and a guard does not.
 *
 *   SLOT                          OWNER (§8.2)        per-node   exponential
 *   ----                          -----------         --------   -----------
 *   internal body child slot      the node it is in   SW armed   SW if owned
 *   cds_ft_compressed_node.child  that cn             SW armed   SW if owned
 *   cds_ft.root                   NONE (no node)      MW         MW   [DESIGN]
 *   SKIP_X dual (GP body word)    the GRANDparent     SW armed   SW if owned (1)
 *   cds_ft_metadata.external_nodes the node it is in  SW armed   SW if owned
 *   metadata.state  LOCK take     node / its anchor   MW         MW   [DESIGN]
 *   metadata.state  tombstone     ditto               SW armed   SW if owned
 *   metadata.state  lock release  ditto               SW armed   SW if owned
 *   metadata.state  nr_child++    ditto               SW armed   SW if owned
 *   metadata.state  nr_child--    ditto               MW  (2)    MW   [DESIGN]
 *   metadata.state  {live->live}  --                  MW         MW   [DESIGN]
 *   metadata.parent_word          the PARENT (3)      SW if parent held
 *   metadata.parent_slot_offset   the PARENT (3)      SW if parent held
 *   metadata.nr_keys              FT-wide lock (9)    MW         MW   [debt]
 *   metadata.incoming_byte        not transacted (7)
 *   metadata.alloc_index          ALLOCATOR-PRIVATE (7) -- never transacted
 *   cds_ft_node.next              the chain HOLDER    MW  (4)    MW   [debt]
 *   cds_ft_node.prev  (member)    the chain HOLDER    MW / SW(5) MW   [debt]
 *   cds_ft_node.prev  (head)      the holder P        SW if parent held (5)
 *   ft_ord_cell.lnode.next/prev   NONE (6)            MW         MW   [DESIGN]
 *   ft_ord_cell.parent            the holder P        MW + raw   MW   [debt]
 *   ft_ord_cell.node              BUILD-INVISIBLE (8) -- never transacted
 *
 * ☞ "[DESIGN]" means the word can never convert: it has no single owner (the
 * root slot lives in no node; an
 * ordered-list splice rewrites the cells of NEIGHBOURING keys, whose holders
 * the op never acquires) or it IS the arbitration point (the lock take).
 * Everything else marked [debt] has a named owner and is on the conversion
 * surface; "MW by constr." is neither -- it is door 2 above, and it changes
 * the moment that gate lifts.
 *
 * (1) The dual's owner is DERIVED at publish from cn's back-pointer, so only
 *     the op can vouch for it: every producer passes @owner_held false, the
 *     insert lane and the remove lane alike.  Both now ACQUIRE it
 *     (ft_insert_lock_skip_dual_gp, ft_lock_skip_dual_gp).
 *     ☑ AND THE TWO NAMED BLOCKERS ARE DISCHARGED -- this row said it "stays MW
 *     until ft_node_recompact's own dual site (FT_OWNER_UNPLUMBED) and the glue
 *     publishes can vouch too".  The glue publishes vouch through the FT-wide
 *     writer lock, and the relocation site vouches from its own acquire (it
 *     always HELD the word; it was asked through a registry that structurally
 *     could not answer).  MEASURED at per-node AND exponential: every record
 *     that COMMITS is now excluded.  The only producers still answering no were
 *     ft_promote_head's two arms, and only on an acquire MISS; they now BAIL at
 *     the miss rather than emit a record they cannot vouch for, so the kind is
 *     decided where the record is made and not by whether some later commit
 *     discards it.  @root keeps MW by design.
 *
 *     ☑ SO THIS ROW IS OFF [debt], MEASURED TWO WAYS, per-node AND exponential:
 *     (a) -DFT_DEBUG_DUAL_SITE, at the dual's OWN recorder -- named_unheld = 0
 *     AND unnamed = 0 for EVERY producer; (b) -DFT_DEBUG_TXN_KIND, the G4
 *     surface -- DUAL, DUAL_ROOT and DUAL_NAMED (the last being "a producer
 *     that could vouch if it acquired") all read ZERO, so no dual record lands
 *     in MW_ALWAYS at all.
 *     ☞ "SW armed" here means what it means in the rows above: SW once the txn
 *     armed per-op, MW_STRUCT when it did not.  The measurement says the dual
 *     is no longer MW BY CLASS; it does not claim every record is SW.
 *
 *     ☠ `DUAL_UNNAMED` (~470k-620k per ft_inv leg) WAS READ AS "NOT THIS ROW",
 *     on the claim that every dual is recorded through ft_pub_rec_add_at
 *     (unnamed = 0 in the same run).  That claim was FALSE.
 *     _cds_ft_insert_replace's leaf replace builds its edges BY HAND (the
 *     forward edge, or the SKIP_X dual plus cn->child) and names no owner, so
 *     its dual never reached that recorder.  The per-site split (FT_SA_REC,
 *     keyed on the txn site) puts the whole bucket on that one op.
 *     -DFT_DEBUG_STRUCT_ANCHOR asked each edge's owner.  The forward edge and
 *     cn->child were held.  The dual's GRANDPARENT was not: 15,729 (per-node) /
 *     10,279 (exponential) records per ft_inv leg, every one committed.
 *     ☑ The op now takes GP (ft_lock_skip_dual_gp) in both leaf arms and derives
 *     the edges after that acquire: 0 uncovered at every spacing.  The records
 *     stay MW and still carry no owner.
 *     ☠ NOT a backtrace at the classification: backtrace_symbols_fd takes the
 *     loader lock, and from 207 threads inside the commit loop it WEDGED the
 *     leg -- 0 of 152 tests in 1800 s.
 * (2) ☑ WAS FT-SLOT-1 (fixed), not a conversion question: ft_state_edge leaves .tag 0
 *     and ft_edge_tag defaults an untagged edge to FT_FLIP_PROXY_TAG (0xF),
 *     so the remove's fused nr_child-- parks a 0xF-tagged proxy on a word
 *     every other producer and resolver tags FT_STATE_PROXY (0x1).  is_proxy
 *     with 0x1 accepts it and untag yields desc|0xE -- a misaligned record
 *     pointer.  Debug builds trap tag aliasing on the RECORD path only.
 * (3) ☑ FT-SLOT-3 SETTLED 2026-09-17 (Mathieu): the KIND rests on the PARENT
 *     whose slot names the child -- §8.2's owner -- and no longer on holding
 *     the CHILD.  The two predicates no longer disagree, because there is one.
 *
 *     WHY THE PARENT.  A re-home changes WHICH SLOT names the child, so every
 *     writer of this word rewrites that slot in the parent and therefore holds
 *     the parent; only SOME also hold the child (the recompaction sweep
 *     re-homes every child of the node it copies and holds none of them).  The
 *     parent is what makes those writers exclude each other; the child was a
 *     sufficient extra licence that left the sweep MW forever.
 *
 *     MEASURED (-DFT_DEBUG_SLOT3, per ft_inv leg): the parent is held for
 *     1,246,594 of 1,246,659 records at per-node, 1,390,410 of 1,390,475 at
 *     exponential, 1,272,852 of 1,272,917 at root-only, and EVERY record the
 *     child licensed was one the parent licensed too.  After the switch the
 *     lane reads PARENT_WORD 65 and PSO 0 per leg at every spacing -- the 65
 *     being compaction's own txns (nr_locks 0), whose exclusion the API still
 *     leaves to the caller.
 *     ☞ @child_held survives as a LEGACY branch, counted by
 *     @ft_slot3_child_only: 5-18 per ft_unit leg, FINE SHARED = 0, i.e. only
 *     on tries whose exclusion is trie-wide, where door 1 decides the kind and
 *     the named owner does not.  Both branches record the same kind, so they
 *     cannot disagree on one word.
 * (4) ☠ See FT-SLOT-2 for this word's dropped mark check.  And MW is
 *     load-bearing here TODAY: ft_hlist_freeze_sole_prepare's derived
 *     NULL is the only thing that turns an UNHELD sole-entry derivation into
 *     an abort.  Re-derive under the holder lock before any SW park.
 * (5) ☑ FT-SLOT-4 (text corrected).  ft_promote_head already parks a head's prev
 *     SW (owner = the holder) while ft-txn-hlist.h records the same word class
 *     MW.  Not a live race (every producer holds the holder, or is bulk-gated,
 *     or the head is build-invisible) -- but "genuinely unlocked" is FALSE of
 *     this word, and the kind-stats HEAD_BACK entry now says so.
 *
 *     ☑ AND A SECOND PRODUCER HAS NOW CONVERTED: ft_glue_record_splices' store
 *     to @src_head->prev, via ft_hlist_store_sw.  Its MW was a BACKSTOP against
 *     cds_ft_compact_step -- the one peer the bulk gate does not park -- and
 *     compaction is LAST in the plan and excluded by its own contract, so the
 *     CAS was arbitrating against nobody.  Measured before flipping: 5401 of
 *     5401 stores with the FT-wide writer lock held inside a bulk body, 0 drain
 *     seams in-window (-DFT_DEBUG_SPLICE_SEAM).  ⇒ Keeping an MW CAS to survive
 *     an op the contract excludes is how a [debt] word stays MW forever.
 *
 *     ☠ The REST of the class is NOT converted, and must not be flipped site by
 *     site: rcu-txn.h makes a slot SW xor MW GLOBALLY, and the point-op chain
 *     sites run against each other under per-node holder locks with RAW
 *     producers beside them (ft_set_parent's external arm).  That is one
 *     whole-class step, not six small ones.
 * (6) The one family whose design-MW is ARGUED rather than asserted.
 * (7) ☑ THESE TWO SHARE ONE WORD, AND THAT IS THE CLASSIFICATION.  Neither is
 *     transacted, so neither took a KIND and neither was listed here -- but
 *     @incoming_byte IS written on a PLACED node (ft_set_parent_slot, from
 *     ft_set_parent AND ft_publish_to_parent, i.e. every publish path), and it
 *     is a BITFIELD: each write is a read-modify-write of the whole tail word,
 *     @alloc_index included.  A register that lists only transacted words hides
 *     that, which is why @alloc_index is named here rather than left out.
 *
 *     The struct's own note rests the sharing on "no two writers ever race
 *     here".  The load-bearing half is narrower and checkable: @alloc_index is
 *     IMMUTABLE while the node is reachable -- written once by the allocator
 *     (fractal-trie-alloc.c, at alloc; saved and restored around the free-list
 *     memset, where the item is unreachable) and never again.  So a racing
 *     @incoming_byte RMW writes the SAME @alloc_index back and cannot lose it,
 *     whatever the byte does.  ☠ That is what a new field packed into this word
 *     would have to inherit, and a mutable one could NOT: it would need
 *     @incoming_byte's writers to exclude each other, which nothing here does.
 *     A value that must be coherent with a structural flip belongs in @state.
 * (8) ☑ Written RAW, twice, and both sites are legal by the last rule in this
 *     block (a node no reader can reach): ft_ord_cell_alloc, and
 *     ft-compact.h's relocation, which stores into a cell freshly returned by
 *     cds_ft_alloc_cell_item and published only by the later ft_ord_cell_swap.
 *     No site writes a LIVE cell's @node, so it never needed a kind -- listed
 *     because "absent from the register" and "cannot be written" are different
 *     claims, and only the second one is true here.
 * (9) Decided 2026-09-17: this row was [DESIGN] on "the count climbs
 *     ancestors nobody locked", which is true only of a FINE writer, and
 *     ft-lifecycle.h never lets a rank-stats trie be one: the group is coerced
 *     to CDS_FT_WRITER_LOCK_COARSE.  Every writer of the word therefore holds
 *     the FT-wide writer lock (door 1 above), so it may be SW under that
 *     condition, and ONLY that one: a rank-stats trie that ever runs FINE
 *     puts the word back to MW.
 *
 * ----------------------------------------------------------------------------
 * OPEN DEFECTS AND OPEN QUESTIONS IN THIS TABLE, tagged so they can be found:
 * grep -rn 'FT-SLOT-' src/fractal-trie/
 *
 *   FT-SLOT-1  ☑ FIXED.  A state-word TAG MISMATCH: ft_state_edge left .tag 0
 *              and ft_edge_tag defaults an untagged edge to FT_FLIP_PROXY_TAG
 *              (0xF), so the remove's fused nr_child-- parked a 0xF-tagged
 *              proxy on a word every other producer and resolver tags
 *              FT_STATE_PROXY (0x1) -- including its own caller's
 *              expected-old load two lines up.  urcu_txn_is_proxy(v, 0x1)
 *              ACCEPTS 0xF and urcu_txn_untag then yields desc|0xE, a
 *              misaligned record pointer the resolve path dereferences; the
 *              engine's debug net covers the RECORD path only.
 *              ☠ AND ITS REACH WAS NARROWER THAN IT LOOKED, which is the
 *              part worth keeping: the delete arm returns -EFBIG unless
 *              ft_in_place_ok(), i.e. -DFEATURE_FT_INSERT_IN_PLACE on an
 *              EXCLUSIVE trie -- NOT the default, which recompacts on
 *              remove.  Measured: 0 reaches across ft_unit and ft_inv's four
 *              list/rank modes with the detector armed, and the assert fires
 *              at ft_unit test 24 of the in-place build.  So it was LATENT,
 *              never live: the one config that plants the bad tag has no
 *              concurrent resolver by that feature's own contract.  In-place
 *              IS now extended to concurrent tries for the point ops
 *              (ft_in_place_ok), which is what the assert beside the edge
 *              is armed for.
 *              ☞ ft_state_edge; its one caller (ft_remove_one_commit's
 *              not-held nr_child--) now records with FT_STATE_PROXY spelled
 *              explicitly and no longer builds the edge.
 *
 *   FT-SLOT-2  ☑ CLOSED IN BOTH PLACES.  The MARK CHECK is BACK in
 *              ft_hlist_insert_after_prepare -- as a REFUSAL (-ENOENT), not an
 *              assert -- so the prepare once again enforces what
 *              ft-txn-hlist.h's header has always promised ("a concurrent
 *              insert_after(H) onto a sole-node chain sees the mark and
 *              aborts").  Unchecked it is a LOST INSERT: MARK(NULL) reads back
 *              as the bare value 2, which passes `succ != NULL` and makes the
 *              second store record slot &((struct cds_ft_node *) 2)->prev.
 *              ft_hlist_del_prepare carries the SAME refusal, where the shape
 *              reproduced 8/8 as a SEGV once _cds_ft_insert_replace began
 *              tombstoning the chains it displaces.
 *              ☞ THE CALLER-SIDE HALF STANDS TOO: both duplicate-append arms
 *              RE-VALIDATE under the holder lock and refuse a retired head
 *              (ft_node_is_removed), so no walk can END on one.  Two
 *              enforcement points, which is what the header describes.
 *              ☠ AND THE PREPARE-SIDE REFUSAL IS UNEXERCISED BY THE SUITE: a
 *              counter on both arms (-DFT_DEBUG_MARK_REFUSAL) reads 0 across
 *              every point-op row measured 2026-09-14 (insert_replace,
 *              same_key append / inserts / removes / replace, insert_unique,
 *              remove_all nolist/list/prefix).  It is a GUARD with no witness,
 *              not a path with coverage -- do not read its presence as
 *              evidence the shape is reachable today, and do not delete it on
 *              the strength of that zero either.
 *              ☞ ft_hlist_insert_after_prepare, and note the sibling shapes
 *              the survey flagged there -- del_prepare / replace_prepare can
 *              still carry a marked next into a LIVE pred->next, excluded only
 *              by the lock and never checked.
 *
 *   FT-SLOT-3  ☑ SOUND, AND NOW THE REASON IS NAMED -- which it was not, and
 *              that is why it read as a defect.  @parent_word /
 *              @parent_slot_offset carry two ownership predicates (the model's
 *              PARENT, the code's @child_held), and ft_glue_record_back_edge
 *              passes child_held TRUE unconditionally, including for a
 *              displaced PUBLISHED child (measured, from ft_merge_at_inner) --
 *              so on an armed txn it PARKS SW on a live node's parent word.
 *              That is legal, but NOT because the child is in the DLM
 *              lock-set: it is not.  Every caller is a BULK op, and under
 *              lock_fine a POINT op RE-TAKES the FT-wide lock while a bulk op
 *              is live (FT_BULK_WIDE_LOCK + ft_bulk_active), so bulk and point
 *              writers arbitrate on one word again -- "the whole of G5.5's
 *              exclusion".
 *              ☞ THE CALLER SET, RE-ENUMERATED FROM THE TREE (an earlier text
 *              said "merge, rekey, the glue commit", which names a MECHANISM
 *              rather than the ops that reach it and so omitted GRAFT).  Two
 *              direct -- ft-merge.h, ft-rekey.h -- plus ft_glue_txn_commit_edges,
 *              reached from ft_glue_txn_commit (merge, graft x2, rekey),
 *              ft_glue_txn_commit_replace (graft) and ft-rekey.h directly.  So:
 *              MERGE, REKEY, GRAFT, all three bulk-gated.  (cds_ft_detach
 *              reaches ft_glue_apply_deferred, a DIFFERENT function, and does
 *              not record a back edge.)  This list is what a future reader must
 *              re-check when the lock below is relaxed -- verify it, do not
 *              trust it.
 *              The old comment said "by construction" without naming the
 *              construction, which is how this came to look unverified.
 *              ☠ SO IT IS A DEPENDENCY, NOT AN INVARIANT.  The day the FT-wide
 *              lock is relaxed to only flipping the dual-descent state -- the
 *              recorded direction -- this SW park loses its exclusion and
 *              @child_held here must become a real answer about the lock-set.
 *              Revisit it WITH that change, not after.
 *              ☞ ft_glue_record_back_edge.
 *
 *   FT-SLOT-4  ☑ CORRECTED (the texts; the code they described is unchanged
 *              except for one deletion).
 *              HEAD_BACK is documented "NEVER converts / permanent false
 *              arm" while ft_promote_head parks that same word SW with
 *              owner = the holder, and ft_back_edge_owner names P as its
 *              owner.  Three statements, one slot.  Related drift:
 *              ft_chain_next_flip is dead code whose header still names
 *              callers, and the DUAL counter class now receives HELD duals
 *              plus forward edges plus &ft->root, so readings of that column
 *              no longer mean what its definition says.
 * ----------------------------------------------------------------------------
 *
 * ☠ AND NO WORD ABOVE MAY BE WRITTEN RAW once it is reachable.  A plain store
 * does not resolve a peer's parked proxy -- it overwrites the descriptor
 * pointer and breaks atomicity for every txn naming the slot.  The lone-edge
 * fast paths (ft_ord_cell_flip_one and its callers) are raw by construction
 * and are legal only for a bulk op under the FT-wide gate or a node no reader
 * can reach.  Each remaining raw site must say which of those two it is.
 * ============================================================================
 */
struct cds_ft_metadata {
	/* 8-byte aligned fields. */
	/*
	 * ☞ Register: the MODEL owns this edge by the PARENT (§8.2, decision
	 * 09-03); the CODE keys its record kind on holding the CHILD
	 * (@child_held).  Two predicates for one word -- do not add a third, and
	 * a re-parent that already recorded it chains by the EXISTING record's
	 * kind.  MW here is a LOCK-SET REACH question (class PARENT_WORD),
	 * convertible by widening the set, never by arming.
	 * ☠ The glue passes @child_held true for every back edge, including a
	 * displaced PUBLISHED child; an SW park there is legal only while that
	 * child is in the op's lock-set.
	 */
	struct cds_ft_inode_flag *parent_word;	/*
						 * Tagged pointer to parent node.  ☠ A ROOT
						 * DOES NOT STORE NULL HERE -- it stores the
						 * TRIE pointer (ft_parent_word /
						 * ft_trie_parent), so the three states are
						 * DISTINGUISHABLE: a tagged node is a real
						 * parent, ft_parent_is_trie() is the ROOT,
						 * and NULL is a TRANSIENT RE-HOME (between a
						 * detach / graft_swap clearing the link and
						 * the new placement completing).  An up-walk
						 * may therefore STOP at the trie and REFUSE
						 * on NULL instead of mistaking one for the
						 * other -- which is what used to make such a
						 * walk truncate silently.  Such a node is
						 * not reader-reachable anyway: publication
						 * wires the
						 * parent before the node is reachable, and
						 * synchronize_rcu separates the phases.
						 * Written by the mutation side via
						 * rcu_assign_pointer; read by the read side
						 * (ft_skip_to_compressed, ft_get_parent_rcu)
						 * via rcu_dereference.
						 */
	/*
	 * ☞ Register: the PREFIX-HEAD word -- the forward edge a lookup follows
	 * to the key that ENDS at this node, and the head of its same-key chain.
	 * A structural slot of THIS node, so the excluding lock is this node's
	 * state word at per-node spacing and its ANCHOR above; SW on an armed
	 * txn, MW otherwise (MW by construction for a FINE point op above
	 * per-node).  The chain links BEHIND it are a different family entirely
	 * (cds_ft_node.next/prev, owned by the holder).
	 * ☠ With the ordered list AND rank stats both off, a lone edge on this
	 * word is committed by the raw lone-store path -- a plain store on a word
	 * other ops transact, which overwrites a parked proxy.  Every write here
	 * must resolve through the engine or hold an exclusion covering every
	 * other writer, list on or off.
	 */
	struct cds_ft_node *external_nodes;	/* List of external nodes at this trie location. */

	/*
	 * parent_slot_offset (0..255): the pointer-stride offset of this node's
	 * slot in its parent, PARENT-owned per the edge principle.  Its own word
	 * since the §8.3 split -- see the FT_PSO_* block above @state for why it
	 * left the state word.  TRANSACTED: a re-home records it into the same
	 * flip-txn as &meta->parent (ft_reparent_record_meta), so the slot can
	 * hold a parked FT_STATE_PROXY and every access must go through the
	 * ft_meta_parent_slot_offset* helpers -- never a raw load or store.  The
	 * value is stored SHIFTED (FT_PSO_SHIFT) to keep bit 0 clear for the
	 * engine's in-band proxy tag, as the engine requires of every live value.
	 */
	/*
	 * ☞ Register: same owner and same kind predicate as @parent_word -- the
	 * model owns it by the PARENT (§8.3 split it out of @state precisely
	 * because one word cannot be owned by two locks), the code keys its kind
	 * on holding the CHILD.  Recorded ONLY by a live re-home, into the same
	 * txn as @parent_word, so the (parent, offset) pair flips atomically; a
	 * reader must resolve the pair together, never field by field.
	 * ☠ Its raw setter waits out a parked FT_STATE_PROXY but deliberately NOT
	 * the LOCK bit; do not "fix" that -- waiting on the lock self-deadlocked.
	 */
	uintptr_t parent_slot_offset;

	/*
	 * Total unique keys in subtree.
	 * Stored with uatomic_store release, loaded with acquire.
	 */
	/*
	 * ☞ Register: the count is propagated up the ancestor chain to the root,
	 * through nodes a FINE op never acquires -- which is why a rank-stats
	 * group is COERCED to CDS_FT_WRITER_LOCK_COARSE (ft-lifecycle.h).  So
	 * every writer of this word holds the FT-wide writer lock, and the word
	 * may be SW under that condition and that condition only.  Its MW today
	 * is debt, not design.  Stored as (count << 1); bit 0 is
	 * FT_NR_KEYS_PROXY_TAG.
	 * ☠ One txn may walk one ancestor TWICE (-1 then +1): the expected old
	 * must come from the DESCRIPTOR first, or the second walk poisons the
	 * commit and the op livelocks with no contention at all.
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
	 *   bits 11-18 FREE -- they used to hold parent_slot_offset, which now
	 *                                 lives in its OWN word (@parent_slot_offset
	 *                                 below): a word cannot be owned by two
	 *                                 locks, and the offset is PARENT-owned
	 *                                 while this word is node-owned.
	 *   bit 19     FT_STATE_LOCK  -- the reversible per-node writer lock.
	 * Access nr_child / parent_slot_offset ONLY via the ft_meta_nr_child* /
	 * ft_meta_parent_slot_offset* helpers -- they mask their own field and
	 * preserve the others; never read/write the word directly.
	 */
	/*
	 * ☞ THE SLOT REGISTER above is authoritative; this word is the one with
	 * a PROTOCOL rather than a single kind, and ft-mutation-helpers.h's
	 * "THE STATE-WORD PROTOCOL" states it step by step: the LOCK TAKE is MW
	 * at every spacing (it IS the arbitration point -- an SW take would hand
	 * two ops the same node), while the tombstone, the lock release and a
	 * recorded nr_child delta are SW on an armed txn.  Under a coarse spacing
	 * the TAKE and RELEASE land on this node's ANCHOR ANCESTOR's state word
	 * while the tombstone and nr_child edges land here, unlocked -- what
	 * excludes them is the anchor plus the agreement that every writer of this
	 * node resolves the same one.
	 *
	 * ☠ DO NOT SUBSTITUTE A GUARD FOR THE TAKE.  An MW {live->live} validate
	 * on this word is not arbitrated against a peer's SW park: the validator
	 * loses silently.  Ownership is taken, never observed.
	 * ☠ A state edge routed through an ft_ord_cell_edge MUST set
	 * .tag = FT_STATE_PROXY -- see note (2) in the register.
	 */
	uintptr_t state;

	/*
	 * Packed bitfield -- small fields in a single uint32_t.
	 * (nr_child lives in @state above, not here.  parent_slot_offset does NOT:
	 * §8.3 gave it its OWN word (@parent_slot_offset) because a word cannot be
	 * owned by two locks -- the slot offset is PARENT-owned while @state is
	 * node-owned, and sharing them self-deadlocked the offset setter against
	 * its own node's LOCK bit.  Access it via the
	 * ft_meta_parent_slot_offset* helpers.)
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
	 * field.  Store alloc_index in the struct's TAIL WORD, which it shares
	 * with @incoming_byte: it costs no additional word and leaves every field
	 * above it untouched.  24 bits (16 M) is ample for a 2 MiB block; the top
	 * 8 bits carry incoming_byte (below), so the tail needs no separate byte.
	 */
	/*
	 * ☞ Register: THIS TAIL WORD IS NOT TRANSACTED.  @alloc_index is written
	 * once by the allocator; @incoming_byte is a pure function of (parent,
	 * slot) and is re-stored plainly at every child placement -- a same-value
	 * write across a recompact, which is what makes an unlocked store legal.
	 * Its only ordering is the release of the parent pointer that follows it,
	 * so a reader's up-walk must acquire @parent_word BEFORE reading the byte.
	 * ☠ IT IS A BITFIELD: every write is a read-modify-write of the whole
	 * word, so these two fields can only share it because no two writers ever
	 * race here.  A new field packed in inherits that obligation -- and a
	 * value that must be coherent with a structural flip belongs in @state.
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
	 * Near layout: shares the struct's tail word with @alloc_index, so it
	 * costs no additional word and leaves every field above it untouched.
	 */
	uint8_t incoming_byte;
#endif
#ifdef FEATURE_FT_HOLD_TRACE
	/*
	 * E.2 exclusion-oracle stamp: which thread currently claims this
	 * NODE's exclusion, and from where.  Claimed at the acquire
	 * primitives (ft_hold_trace_note), yielded when the covering lock's
	 * release records (ft_hold_trace_drop).  MEMBER-keyed, not
	 * anchor-keyed: two writers covering one node through DIFFERENT
	 * anchors -- the anchor disagreement the spacing refusal exists to
	 * prevent -- both stamp HERE, and the second claim aborts with both
	 * owners named.  Zeroed by the allocator's metadata memset; the
	 * free-poison lands only after the GP that follows the release, so a
	 * yield always precedes it.
	 */
	unsigned long dbg_owner_tid;
	const char *dbg_owner_fn;
	int dbg_owner_line;
	/*
	 * The ANCHOR the owner's site DERIVED for this member -- the word it
	 * actually locked.  Two ops covering one node through DIFFERENT anchor
	 * derivations is the class the member-keyed stamp exists to catch, and
	 * only the anchors distinguish it from a genuine two-system collision.
	 */
	const struct cds_ft_metadata *dbg_owner_anchor;
	/*
	 * The last anchor ANY op derived for this node, kept across yields.
	 * At a fixed spacing the anchor is a function of the node and its
	 * DEPTH, so two ops deriving different words for one node is a
	 * derivation disagreement -- the class that leaves each op excluding
	 * on a word the other never takes.  Detecting it here needs no
	 * collision, so it measures the DISAGREEMENT RATE rather than waiting
	 * for two ops to lose the race on the same node.
	 */
	const struct cds_ft_metadata *dbg_last_anchor;
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

		if (caa_unlikely(s & FT_STATE_INPLACE_WAIT_MASK)) {
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

		if (caa_unlikely(s & FT_STATE_INPLACE_WAIT_MASK)) {
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
 * parent body, held in its OWN word (@parent_slot_offset), NOT in @state --
 * §8.3 split it out; @state keeps nr_child and the lock/proxy bits.  This
 * raw reader suits a node the caller owns / that is quiescent; a reader that may
 * race a mid-commit proxy uses the resolving ft_meta_parent_slot_offset_load,
 * and a backtracker recovering the (parent, offset) PAIR must use
 * ft_resolve_parent_slot (same-mcas + stability re-read).  Root nodes
 * (parent == NULL) leave this 0 and never read it.
 */
static inline
unsigned int ft_meta_parent_slot_offset(const struct cds_ft_metadata *meta)
{
	return FT_PSO_DECODE(meta->parent_slot_offset);
}

/*
 * The setter owns the whole word now, so it no longer has to preserve nr_child
 * or the state tag bits -- the §8.3 split moved the offset out of @state.  What
 * it still must respect is the ENGINE: this slot is transacted, so a peer can
 * have an FT_STATE_PROXY parked here, and a blind store over a proxy pointer
 * would drop a live descriptor a resolver is about to chase.  Wait the proxy out
 * rather than skipping it -- the offset must land on the SETTLED word, and the
 * wait is bounded by the owner's settle.
 *
 * ★ The wait is over FT_STATE_PROXY ALONE, deliberately, NOT
 * FT_STATE_INPLACE_WAIT_MASK.  That mask includes FT_STATE_LOCK under
 * the DLM lock-sets, and while the offset shared @state an op holding
 * this node's own node lock spun on itself forever -- the self-deadlock that
 * reverted b20c471e (single-threaded, ft_unit test_lookup_nth_varlen: a prefix
 * insert builds a glue node and re-parents a node the op has locked).  The
 * offset word has no LOCK bit to wait on, which is the whole point of the
 * split, so DO NOT reintroduce that mask here.
 *
 * A LIVE, reader-reachable node must not come through here at all: its re-home
 * records the offset into the same flip-txn as &meta->parent
 * (ft_reparent_record_meta), so the pair flips atomically.  This setter is for
 * fresh/invisible nodes and for the bulk-op roots that still run under the
 * application's mutual exclusion between mutators (detach root, graft branch
 * re-root).  Skip the store when the offset is unchanged -- the in-place
 * reserve's same-value republish is the hot case.
 */
#ifdef FT_DEBUG_PAIR_STORE
/*
 * The glue's same-op torn-window ring (ft-helpers.h, ft_dt_glue_check).  Filled
 * by a bulk op's glue, cleared when its outermost FT-wide writer scope releases,
 * so an entry never outlives the op that noted it.
 */
#define FT_DT_GLUE_RING	128
static __thread struct {
	const struct cds_ft_metadata *meta;
	struct cds_ft_inode_flag *new_parent;
} ft_dt_glue_ring[FT_DT_GLUE_RING];
static __thread unsigned int ft_dt_glue_ring_n;
#endif
#if defined(FT_DEBUG_DEL_TOMB) || defined(FT_DEBUG_PAIR_STORE)
/*
 * PROBE: score the rule this setter's header states -- a LIVE, reader-reachable
 * child must not come through here.  Defined next to ft_slot_in_node in
 * ft-helpers.h, which is where the predicates it needs live.
 * -DFT_DEBUG_PAIR_STORE builds these two probes alone, without the rest of the
 * FT_DEBUG_DEL_TOMB family.
 */
static void ft_dt_pso_store_probe(const struct cds_ft_metadata *meta,
		unsigned int off);
/* The same question for @incoming_byte, which the up-walk reads beside the parent. */
static unsigned int ft_dt_ib_store_probe(const struct cds_ft_metadata *meta,
		unsigned int byte);
# define FT_DT_IB_STORE(meta, byte)	ft_dt_ib_store_probe((meta), (byte))
#else
# define FT_DT_IB_STORE(meta, byte)	(byte)
#endif

static inline
void ft_meta_parent_slot_offset_set(struct cds_ft_metadata *meta, unsigned int off)
{
	uintptr_t n = FT_PSO_ENCODE(off);

#if defined(FT_DEBUG_DEL_TOMB) || defined(FT_DEBUG_PAIR_STORE)
	ft_dt_pso_store_probe(meta, off);
#endif
	for (;;) {
		uintptr_t s = CMM_LOAD_SHARED(meta->parent_slot_offset);

		if (caa_unlikely(s & FT_STATE_PROXY)) {
			caa_cpu_relax();
			continue;
		}
		if (n == s)
			return;		/* same-value republish: nothing to do */
		if (caa_likely(uatomic_cmpxchg(&meta->parent_slot_offset, s, n) == s))
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
	/*
	 * ☞ Register: @child is this node's OWN body word, so the lock that
	 * excludes a writer is this cn's state word at per-node spacing and its
	 * ANCHOR above.  SW on an armed txn, MW otherwise -- and MW by
	 * construction for a FINE point op above per-node.
	 *
	 * ☠ IT NEVER TRAVELS ALONE.  While this cn is reached through a SKIP_X
	 * word, the GRANDPARENT slot encodes THIS value, and a republish must
	 * refresh both in ONE commit.  The dual is owned by the GRANDPARENT, not
	 * by this node: hold GP (ft_lock_skip_dual_gp / ft_insert_lock_skip_dual_gp)
	 * before recording it.  A recompaction of GP copies its body under GP's
	 * lock and drops the per-slot read-set on that strength, so a dual
	 * recorded without holding GP lands in the body being retired while the
	 * fresh copy keeps the stale word -- and then this field and the
	 * reachable dual disagree forever.
	 */
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
 *
 * ONE exception, without FEATURE_FT_COMPRESS: ft_build_branch cannot fold a
 * branch into one compressed node, so it builds a chain of single-child nodes,
 * one per key byte -- up to FT_MAX_KEY_LEN of the smallest node type, not O(1).
 * That bucket's excess lives in @branch (cds_ft_alloc_reserve_add_branch): the
 * draw path takes from it once the ordinary bucket is empty, and a refund spills
 * into it once the ordinary bucket is full.
 */
#define CDS_FT_ALLOC_RESERVE_NR_KIND	2
#define CDS_FT_ALLOC_RESERVE_CAP	8
#ifndef FEATURE_FT_COMPRESS
#define CDS_FT_ALLOC_RESERVE_BRANCH_CAP	FT_MAX_KEY_LEN
#endif

enum cds_ft_alloc_kind {
	CDS_FT_ALLOC_KIND_NODE = 0,
	CDS_FT_ALLOC_KIND_COMPRESSED = 1,
	CDS_FT_ALLOC_KIND_CELL = 2,	/* never reserved */
};

struct cds_ft_alloc_reserve {
	struct cds_ft_metadata *items[CDS_FT_ALLOC_RESERVE_NR_KIND]
			[FT_ALLOC_ORDER_MAX + 1][CDS_FT_ALLOC_RESERVE_CAP];
	unsigned int count[CDS_FT_ALLOC_RESERVE_NR_KIND][FT_ALLOC_ORDER_MAX + 1];
#ifndef FEATURE_FT_COMPRESS
	/* Excess of the (NODE, @branch_order) bucket; 0 until add_branch. */
	struct cds_ft_metadata *branch[CDS_FT_ALLOC_RESERVE_BRANCH_CAP];
	unsigned int branch_count;
	size_t branch_order;
#endif
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
	 * at every mutation.  Default CDS_FT_WRITER_LOCK_FINE (DLM), resolved at
	 * group create from @writer_strategy_set.
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
	/* up-walks that started on a duplicate-chain MEMBER (ft_ext_head_word) */
	unsigned long nr_ext_member_hops;
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
	 * @rekey_set: the application opted this group's tries into the in-trie
	 * MOVE ops (cds_ft_rekey_graft / cds_ft_rekey_merge).  DEFAULT OFF --
	 * see cds_ft_group_attr_set_rekey.  A trie that cannot host a move
	 * needs no reader coherence at all, so this also decides
	 * @rekey_coherence and with it which lookup specializations
	 * ft_install_lookup_ops selects.
	 */
	bool rekey_set;
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

	/*
	 * ☞ Register: THE ROOT SLOT IS THE ONE CAS-ONLY STRUCTURAL WORD.  It
	 * lives in no node, so no state word can make a plain park legal at ANY
	 * spacing -- root-only spacing locks the root NODE's state word, which is
	 * a different word.  Every record of it is MW whatever the txn's mode, so
	 * route it through ft_flip_txn_record_root or carry the per-edge .root /
	 * rec->root[] flag so the replays do.  MW by DESIGN: it never converts.
	 * ☠ The LONE-EDGE fast paths store this word RAW (ft_ord_cell_flip_one
	 * and its callers).  That is legal only for a bulk op under the FT-wide
	 * gate: a raw store does not resolve a peer's parked proxy, it overwrites
	 * the descriptor pointer.  A point op reaching a raw root store has left
	 * the engine's atomicity for everyone.
	 */
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
	 * The value @max_used_key_len held immediately after the last EXACT walk
	 * (ft_recompute_max_used_key_len), so a repair can tell an INFLATED hint
	 * from a merely LARGE one.  Equal means "already walked at this value":
	 * nothing has raised the hint since, so a walk would recompute the same
	 * number and the refusal it feeds is TRUE, not an artifact.
	 *
	 * ☠ WITHOUT IT THE REPAIR IS AN O(n) STALL PER CALL.  The same-trie
	 * rekey's unequal-length gate is also true whenever the trie genuinely
	 * HOLDS a long key, and a refused move raises nothing -- so every
	 * repeated call walked the whole trie under the FT-wide lock, for a
	 * refusal that was correct the first time.  MEASURED at 300k keys plus
	 * one 255-byte key: 0.005 ms/call before the repair existed, 232.6
	 * ms/call with the unguarded repair, and concurrent point inserters fell
	 * from 16,841/s to 562/s behind the lock.
	 *
	 * Written only where @max_used_key_len is written by that same walk, hint
	 * first: a peer raise landing between the two stores leaves them UNEQUAL,
	 * which costs one extra walk and never skips a needed one.  Both are
	 * relaxed; the pair is a heuristic about the hint, not a lock.
	 */
	size_t max_used_key_len_walked;

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
	 * REKEY (in-trie move) coherence, opt-in per-trie (doc: rekey-second-walk):
	 * when true, every EXACT lookup on this trie runs a second walk -- the
	 * parent-pointer up-walk that rematerializes the key from the leaf -- and
	 * compares those bytes against the ones it descended with; a mismatch means
	 * a concurrent rekey restructured the descent's path, so the reader
	 * re-descends from the root.  This is what lets a same-trie merge_at move a
	 * subtree with a fallible reattach commit (readers self-verify instead of
	 * relying on a synchronize_rcu gap or a global seqcount); it turns this
	 * trie's reads from wait-free into lock-free (bounded by writer move
	 * progress).  Requires the up-walk, so it is ANDed with ->ordered_list at
	 * create; immutable after ft_install_lookup_ops (the coherent lookup
	 * specializations are keyed off it), so a concurrent reader's branch --
	 * really the fn-ptr choice -- is race-free.
	 */
	bool rekey_coherence;

	/*
	 * ALWAYS-COHERENT: readers on this trie take the two-pass path
	 * unconditionally, never consulting @move_active (ft_move_active
	 * returns true outright).  Set for an EXTERNAL_SYNC trie that opted
	 * into rekey.
	 *
	 * ☠ IT IS NOT A PERFORMANCE KNOB -- IT IS WHAT MAKES REKEY LEGAL THERE.
	 * The move gate's discipline is publish @move_active, wait ONE FULL
	 * GRACE PERIOD, only then mutate; the GP is there so no reader is still
	 * running under the old rule.  Under CDS_FT_WRITER_EXTERNAL_SYNC that
	 * grace period is taken while the APPLICATION holds its own writer
	 * exclusion, and the library can neither drop that lock nor place
	 * itself outside it -- which is precisely the deadlock
	 * ft_writer_lock_gp_wait exists to avoid for the library's OWN mutex
	 * ("writers parked on the lock are RCU-online and non-quiescent, so
	 * they are precisely what stops this grace period from ever
	 * completing").  Measured 2026-09-20: 2 gate grace periods per pair of
	 * rekeys, taken under the app's lock.
	 *
	 * With every reader ALREADY on the coherent path there is no reader
	 * running under the old rule, so there is nothing to wait for and the
	 * gate owns no grace period at all.  The trade the move gate makes --
	 * pay one GP, keep the steady-state reader on the fast path -- is
	 * simply the wrong way round in this mode.
	 *
	 * The price is honest: every exact lookup on such a trie runs the
	 * second walk, always.  That is the cost of rekey under an exclusion
	 * the library does not own.
	 */
	bool rekey_always_coherent;

	/*
	 * MOVE MODE GATE (doc/design: the per-trie move refcount).  An in-trie MOVE
	 * (rekey) cannot be made coherent for free on the read side, and it must not
	 * make the STEADY-STATE reader pay: so readers run in one of two modes, and
	 * this word is the mode.
	 *
	 *   @move_active == 0  =>  no move can be in flight: readers take the FAST
	 *                          path and perform NO coherence check at all.
	 *   @move_active != 0  =>  readers take the COHERENT path.
	 *
	 * The switch is made safe by a GRACE PERIOD, not by ordering tricks: a mover
	 * sets @move_active, then waits a GP, and only THEN mutates the structure.  A
	 * reader therefore cannot observe move-induced incoherence while believing it
	 * is in fast mode -- if it sampled 0, the mover's GP is still waiting for that
	 * reader's own critical section to end before touching anything.  One sample
	 * per read section is enough for the same reason.
	 *
	 * Cost is confined to move windows, and the steady state pays ONE STABLE READ
	 * of a word nobody writes -- which is the whole point of a refcount here
	 * rather than a per-step sequence counter: no shared-write cache-line bounce
	 * on the read path, and no unbounded reader retry under a stream of moves.
	 *
	 * @move_gate_lock / @move_gate_cond / @move_gate_nr / @move_gate_gp serialize
	 * the movers and let them PIGGYBACK: only the 0->1 transition pays a grace
	 * period, and a mover arriving while that GP is in flight waits for it instead
	 * of starting its own, so a burst of moves costs ~one GP in total (the batching
	 * shape of src/urcu-call-rcu-impl.h's splice + single synchronize_rcu).  Access
	 * only through ft_move_gate_enter / ft_move_gate_exit / ft_move_active.
	 */
	unsigned long move_active;
	pthread_mutex_t move_gate_lock;
	pthread_cond_t move_gate_cond;
	unsigned long move_gate_nr;	/* COHERENT holders (rekey) */
	/*
	 * THE WRITER-SIDE WORD, beside @move_active and deliberately NOT it.
	 * @move_active is READER-facing: nonzero puts every reader on the
	 * COHERENT path, a cost a detach / graft / merge has no reason to
	 * impose.  G5.5's exclusion is a WRITER question -- "is any bulk op
	 * live?" -- so it gets its own trie-level word, read once per writer
	 * scope behind caa_likely and free in steady state.
	 *
	 * ☠ ONE GATE, ONE GP, TWO WORDS, TWO REFCOUNTS.  The machinery below
	 * (@move_gate_lock / @move_gate_cond / @gate_gp_nr) is SHARED, so a
	 * rekey burst and a detach burst still amortize into one grace period.
	 * What must not be shared is the refcount deciding when each WORD
	 * clears: a single counter would either leave @move_active set by a
	 * detach -- the exact cost this split exists to avoid -- or clear it
	 * while a rekey is still live, which is a correctness bug.
	 *
	 * @bulk_gate_nr is the count under @move_gate_lock; @bulk_state is the
	 * SAME count published for peers to read without it, in ONE load
	 * (ft_bulk_active).  Two words because the gate's own arithmetic is
	 * serialized by the mutex while the predicate must be readable from a
	 * writer scope that holds nothing.
	 */
	unsigned long bulk_state;
	unsigned long bulk_gate_nr;	/* ALL bulk holders, rekey included */
	/*
	 * In-flight grace periods, NOT a boolean.  Each runs OUTSIDE the lock,
	 * so TWO owners can overlap: a detach publishing @bulk_active and a
	 * rekey then publishing @move_active each own one.  A boolean cannot
	 * name two owners, and the second clear would release waiters whose
	 * word has not yet had a full grace period.
	 */
	unsigned long gate_gp_nr;
#ifdef FT_DEBUG_BULK_WINDOW
	/*
	 * Bulk-window measurement state.  PER-TRIE, because the gate is: two
	 * tries can be inside ft_move_gate_enter at once (every cross-trie
	 * scenario does exactly that), and a process-wide @t_open would be
	 * clobbered by the other trie's window.  Written only under
	 * @move_gate_lock, so it inherits the gate's own serialization.
	 */
	uint64_t bw_t_open;		/* when this trie's window opened (0->1) */
	uint64_t bw_t_close;		/* when it last closed (1->0), for the gap */
	/*
	 * Counted PER TRIE and folded into the global histograms once per
	 * WINDOW, not once per ENTER.  Atomics on process-wide counters inside
	 * this lock cost 2x throughput here (measured, 8 movers): the critical
	 * section is already contended, and a shared line bounced between
	 * sockets lengthens it enough to convoy every mover behind it.  These
	 * are plain increments of trie-local words under @move_gate_lock.
	 */
	unsigned long bw_enters;
	unsigned long bw_piggyback;
	unsigned long bw_peak;
#endif

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
	 * ☑ BULK-vs-BULK, AND IT IS A SECOND LOCK ON PURPOSE.  @writer_lock
	 * cannot carry this: ft_writer_lock_gp_wait DROPS it across every
	 * mid-body grace period (seven live sites in detach / graft / rekey), so
	 * two bulk ops meet at any drain seam.  It cannot simply be HELD across
	 * those GPs either -- since G5.25 a point op takes @writer_lock too
	 * while a bulk op is live, and that wait stays ONLINE by contract
	 * (ft_writer_lock_take: the caller may hold reader-derived references
	 * across it), so a holder waiting on a GP would be waiting on the very
	 * readers it is blocking.
	 *
	 * This lock restores that property BY CONSTRUCTION instead of assuming
	 * it: ONLY bulk ops ever wait here, and a bulk entry forbids a
	 * read-section caller (fractal-trie.h: "Do NOT call these operations
	 * from within an RCU read-side critical section"), so its waiters park
	 * OFFLINE and never hold a grace period up.  It is therefore held
	 * CONTINUOUSLY across the drain seams, which is what closes them.
	 *
	 * ORDER: strictly OUTSIDE @writer_lock.  Taken after the bulk gate (so
	 * the gate keeps its refcounted one-GP-per-burst piggyback) and before
	 * any writer scope; never taken while holding @writer_lock.
	 */
	struct cds_fair_mutex bulk_lock;
	/*
	 * Hot-path gate: writer_strategy == CDS_FT_WRITER_LOCK_FINE.  The
	 * fine-grained (per-node lock-set) conversions read THIS:
	 * COARSE deliberately derives no lock-set (§10.5 -- one FT-wide lock, no
	 * per-node locks), so a per-node acquire there would be pure cost.
	 *
	 * ☠ STALE AS WRITTEN -- a FINE trie NO LONGER takes the FT-wide writer
	 * lock: ft_writer_lock_scope_enter drops it ALL-AT-ONCE for FINE, and has
	 * since the §11 drop landed.  What remains true is the reason it once
	 * did.  ☑ The ONE case that takes it back is a live BULK op (G5.25): the
	 * refcount decides, so point ops keep the lock-free fast path in steady
	 * state and re-serialize only inside a bulk window.
	 */
	bool lock_fine;
	/*
	 * CDS_FT_WRITER_EXTERNAL_SYNC: the APPLICATION provides writer exclusion
	 * -- a single thread, its own mutex around every mutating call, or any
	 * other means.  The library therefore takes NO writer lock of its own.
	 *
	 * ☠ THIS IS NOT @exclusive, and the difference is the READERS.
	 * cds_ft_make_exclusive additionally promises NO CONCURRENT RCU READERS,
	 * which licenses skipping reader-visible publication discipline.  This
	 * mode keeps readers, so proxies, tags and grace periods all stay: it is
	 * a COARSE trie in every respect except who provides the exclusion.
	 * @lock_fine is false here, so door 1 arms it SW trie-wide and every
	 * consumer of @exclusive is untouched.
	 *
	 * ☞ AND THE PROMOTION LANES ARE INERT, not special-cased: rekey and the
	 * bulk gate exist to promote FINE locking to a wider exclusion, and
	 * there is no fine locking in this mode (nor in COARSE) to promote.
	 */
	bool external_sync;


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
	/*
	 * ☞ THE THREE WORDS NO NODE OWNS, GIVEN OWNERS.
	 *
	 * @ord_sentinel's two list ends and @root are lock-BEARING words with
	 * no lock-bearing ancestor: no node owns them, which is why they were
	 * the permanent MW residue (the MW_ALWAYS ROOT class, and the cell
	 * census's `sentinel` bucket).  "No node owns it" describes the absence
	 * of an owner, not an impossibility -- so give them owners.
	 *
	 * Full struct cds_ft_metadata rather than a bare word so every existing
	 * primitive works unchanged: ft_meta_lock_acquire/release CAS the same
	 * @state bit, ft_flip_txn_lock_register_held files the same anchor, and
	 * ft_dlm_acquire_set sorts them by address alongside node anchors.
	 *
	 * ☠ ONE LOCK PER END, NOT ONE FOR THE LIST.  @ord_sentinel.node.next
	 * (the list BEGIN) and .prev (the END) are independent words; sharing a
	 * lock would serialise a head splice against a tail splice for nothing.
	 *
	 * These are lock ANCHORS only -- never reachable as trie nodes, never
	 * handed to a descent, and named by a lock-only ft_dlm_member.
	 */
	struct cds_ft_metadata ord_begin_lock;	/* &ord_sentinel.node.next */
	struct cds_ft_metadata ord_end_lock;	/* &ord_sentinel.node.prev */
	struct cds_ft_metadata root_lock;	/* &root */

#ifdef FEATURE_FT_EXCL_VALIDATE
	/*
	 * Access-discipline validator state.  @excl_owner holds the
	 * pthread_self() of the thread currently inside a writer API
	 * (claimed via atomic CAS), or 0 when no writer is active.
	 * @excl_writer_depth is a reentry depth counter, accessed only
	 * by the owning thread.  Both are meaningful ONLY where writers
	 * actually serialize -- see @excl_nr_writers.  @excl_nr_readers
	 * counts readers that are currently inside a reader API on an
	 * exclusive-mode trie; concurrent-mode readers do not touch it
	 * (RCU handles them).
	 *
	 * @excl_nr_writers is the FINE-trie replacement for @excl_owner: a
	 * FINE trie skips the FT-wide writer mutex, so DISJOINT WRITERS RUN
	 * IN PARALLEL BY DESIGN and no single owner word can represent them.
	 * Count them instead, so the reader check can still ask the only
	 * question that matters there -- is a DIFFERENT thread writing?
	 */
	unsigned long excl_owner;
	unsigned long excl_writer_depth;
	unsigned long excl_nr_readers;
	unsigned long excl_nr_writers;
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
} __attribute__((aligned(16)));	/* >= FT_PARENT_TRIE_ALIGN: a trie pointer
				 * is stored in the TRANSACTED parent slot and
				 * must clear FT's whole in-band tag. */

/*
 * Root ownership: a root's metadata->parent names its OWNING TRIE.
 *
 * Every non-root node's parent identifies the node above it, so a node
 * reachable from two tries contradicts one of them and cds_ft_verify catches
 * it.  A root has no node above it, and encoding that as NULL makes the
 * back-edge carry no identity at exactly the one position where the owner is
 * the TRIE rather than a node: two tries rooted at one node would then agree
 * with their own expected parent, and no per-trie walk could tell.  Naming the
 * trie closes that, so the check the walk already performs at every other
 * depth also holds at depth 0.
 *
 * ENCODING.  metadata->parent is a TRANSACTED slot: it carries FT's in-band
 * flip proxy, whose tag is FT_INTERNAL_MASK | FT_TYPE_MASK -- the low FOUR
 * bits, not one (the MCAS engine owns bit 0 by default and lets an embedder
 * widen it; FT does, with type 7 == 0xF).  So a value stored here is
 * unambiguous only when its whole low nibble is clear:
 *
 *   internal          bit 0 set
 *   flip proxy        low nibble == FT_FLIP_PROXY_TAG (0xF)
 *   compressed        bit 1 set
 *   skip-compressed   carries the OWNING node's own tag in bits 0-1, because a
 *                     skip pointer names the compressed node's child -- which
 *                     is the very node whose parent field this is, and only
 *                     internal / compressed nodes have a metadata (an external
 *                     cds_ft_node is {prev, next}).  So never 0000.
 *   owning trie       low nibble clear
 *
 * That is what FT_PARENT_TRIE_ALIGN and the assertions below buy: a
 * struct cds_ft * only qualifies while it is aligned past the whole tag, and
 * plain malloc alignment (_Alignof(max_align_t)) is 16 on LP64 but 8 on common
 * 32-bit ABIs -- not enough, and not something to leave to the allocator.
 *
 * The classification is one mask, matching the cost of the NULL test it
 * replaces on the parent-pointer backtrack path.  NULL keeps its meaning of
 * "not yet parented" for a node under build, which is never reachable.
 */

#ifdef FEATURE_FT_PROBE_REANCHOR
extern unsigned long ft_probe_mrg_desc[3], ft_probe_mrg_conf[3];
#define MRG_SKIPCONF_PROBE(i, d)					\
	do {								\
		__atomic_fetch_add(&ft_probe_mrg_desc[i], 1,		\
				__ATOMIC_RELAXED);			\
		if ((d).skip_conflict)					\
			__atomic_fetch_add(&ft_probe_mrg_conf[i], 1,	\
					__ATOMIC_RELAXED);		\
	} while (0)
extern unsigned long ft_probe_ranch[2], ft_probe_rewind[2];
#define MRG_REANCHOR_PROBE(i, rw)					\
	do {								\
		__atomic_fetch_add(&ft_probe_ranch[i], 1,		\
				__ATOMIC_RELAXED);			\
		if ((rw) != 0)						\
			__atomic_fetch_add(&ft_probe_rewind[i], 1,	\
					__ATOMIC_RELAXED);		\
	} while (0)
/*
 * merge_spine_retry's spin depth.  ft-merge.h brackets no engine transaction,
 * so that loop ages nothing and has no termination argument the way every other
 * mutator's retry does; before adding one, measure whether it ever actually
 * spins.  [0] merge ops, [1] retries from a declined dup-chain lock set,
 * [2] retries from a reanchor level-move, [3] the deepest single op's retries.
 *
 * [4] splits [1] by SOURCE CONTRACT, which is what decides whether the fix is
 * even expressible: one body serves two: cds_ft_merge_at consumes an EXCLUSIVE
 * src, so every ft_writer_lock_gp_wait on its path is !exclusive-gated and the
 * spine copy runs GP-free UNDER A READ LOCK; cds_ft_rekey_* share it with
 * src_ft == dst_ft, a LIVE trie, where those waits DO run and a read section
 * held across one would be a writer waiting on its own grace period.
 * urcu_txn_begin() takes the read side, so it can only bracket the first.
 */
extern unsigned long ft_probe_mspin[5];
#define MRG_SPIN_PROBE(i)						\
	__atomic_fetch_add(&ft_probe_mspin[i], 1, __ATOMIC_RELAXED)
#define MRG_SPIN_MAX(v)							\
	do {								\
		unsigned long v_ = (v);					\
		unsigned long o_ = __atomic_load_n(&ft_probe_mspin[3],	\
				__ATOMIC_RELAXED);			\
		while (v_ > o_ && !__atomic_compare_exchange_n(		\
				&ft_probe_mspin[3], &o_, v_, 0,		\
				__ATOMIC_RELAXED, __ATOMIC_RELAXED))	\
			;						\
	} while (0)
/*
 * The OTHER unbracketed retry loops, probed as a CLASS after merge_spine_retry
 * turned out to spin 377 deep with no aging: ft_merge_graft_subpos_inplace's
 * retry_merge, ft_graft_keylen's retry_attach and cds_ft_graft_swap's
 * retry_swap -- the two the merge source itself calls "the same plan->commit
 * retry shape", in a file (ft-graft.h) that brackets no engine txn at all.
 * Counted at the LABEL rather than at each of the 24 goto edges, so the count
 * cannot drift from the control flow.  Pairs: {entries beyond the first,
 * deepest single op}.  [0,1] retry_merge  [2,3] retry_attach  [4,5] retry_swap.
 */
extern unsigned long ft_probe_rspin[6];
#define RSPIN_ENTER(i, v)						\
	do {								\
		unsigned long d_, o_;					\
									\
		(v)++;							\
		if ((v) < 2)						\
			break;						\
		d_ = (v) - 1;						\
		__atomic_fetch_add(&ft_probe_rspin[i], 1,		\
				__ATOMIC_RELAXED);			\
		o_ = __atomic_load_n(&ft_probe_rspin[(i) + 1],		\
				__ATOMIC_RELAXED);			\
		while (d_ > o_ && !__atomic_compare_exchange_n(		\
				&ft_probe_rspin[(i) + 1], &o_, d_, 0,	\
				__ATOMIC_RELAXED, __ATOMIC_RELAXED))	\
			;						\
	} while (0)
/*
 * Split each of those retry counts by the SOURCE CONTRACT, which is what
 * decides whether a persistent-handle fix is EXPRESSIBLE at all rather than
 * merely desirable -- the same question ft_probe_mspin[4] answered for
 * merge_spine_retry (exclsrc=585250 livesrc=0).
 *
 * urcu_txn_begin() opens an RCU read section AND urcu_txn_conflict() ages the
 * handle into the domain's FIFO fallback lane; ft_writer_lock_gp_wait() both
 * self-deadlocks under the first and asserts !urcu_txn_in_fallback() under the
 * second.  So a bracket may only span a body that takes NO grace period, and
 * every grace period on these three paths is gated on a source being LIVE:
 *
 *   [0] retry_merge   dst_ft->lock_fine && src_ft->exclusive
 *   [1] retry_attach  dst_ft->lock_fine && src_ft->exclusive
 *   [2] retry_swap    dst_ft->lock_fine && swap_ft->exclusive
 *
 * which are exactly the conditions the three bodies already read_lock() under
 * (ft-merge.h's spine pin, ft-graft.h's @gs_rlock).  A loop whose retries all
 * land OUTSIDE its condition cannot be fixed this way, and that is a
 * measurement, not an argument.
 *
 * ft_probe_rspin_n[] is this split's RED CONTROL, and it is not optional:
 * live == 0 would read identically whether no retry ever happens off-contract
 * or @cond is simply TRUE BY CONSTRUCTION at that label, and the second reading
 * makes the whole measurement vacuous.  So count the ops that ENTER each loop
 * with @cond false (first entry, v == 1).  A non-zero n proves the condition
 * varies at that site and the counter can tell the two apart; n == 0 means the
 * split says nothing at all there.
 */
extern unsigned long ft_probe_rspin_x[3], ft_probe_rspin_n[3];
/*
 * PROBE (2026-08-20): ft_probe_rspin_n[] counts only OFF-CONTRACT entries, so a
 * zero there cannot distinguish "the loop is never entered off-contract" from
 * "the loop is never entered AT ALL".  Both read as offc=0, and the second is a
 * statement about reachability, not about the contract.  Count TOTAL entries per
 * SITE, split by @cond -- slot 0 is shared by ft_merge_graft_subpos_inplace and
 * ft_rekey_subpos_inplace, so a per-slot number cannot say which one ran.
 *   [site][0] = entries with cond FALSE, [site][1] = entries with cond TRUE
 *   site 0 = merge subpos, site 1 = rekey subpos, site 2 = graft_swap
 */
extern unsigned long ft_probe_rspin_e[3][2];
#define RSPIN_SITE_ENTER(site, v, cond)					\
	do {								\
		if ((v) == 1)						\
			__atomic_fetch_add(				\
				&ft_probe_rspin_e[site][(cond) ? 1 : 0],\
				1, __ATOMIC_RELAXED);			\
	} while (0)
#define RSPIN_ENTER_X(i, v, xi, cond)					\
	do {								\
		bool c_ = (cond);					\
									\
		RSPIN_ENTER(i, v);					\
		if ((v) >= 2 && c_)					\
			__atomic_fetch_add(&ft_probe_rspin_x[xi], 1,	\
					__ATOMIC_RELAXED);		\
		if ((v) == 1 && !c_)					\
			__atomic_fetch_add(&ft_probe_rspin_n[xi], 1,	\
					__ATOMIC_RELAXED);		\
	} while (0)
#else
#define RSPIN_ENTER(i, v)		do { } while (0)
#define RSPIN_ENTER_X(i, v, xi, cond)	do { } while (0)
#define RSPIN_SITE_ENTER(site, v, cond)	do { } while (0)
#define MRG_SKIPCONF_PROBE(i, d)	do { } while (0)
#define MRG_REANCHOR_PROBE(i, rw)	do { } while (0)
#define MRG_SPIN_PROBE(i)		do { } while (0)
#define MRG_SPIN_MAX(v)			do { } while (0)
#endif

#define FT_PARENT_TAG_MASK	((uintptr_t) (FT_INTERNAL_MASK | FT_TYPE_MASK))
#define FT_PARENT_TRIE_ALIGN	(FT_PARENT_TAG_MASK + 1)

urcu_static_assert(!(FT_PARENT_TRIE_ALIGN & FT_PARENT_TAG_MASK),
		"FT_PARENT_TRIE_ALIGN must be a power of two covering the whole parent tag",
		ft_parent_trie_align_pow2);
static inline
struct cds_ft_inode_flag *ft_trie_parent(const struct cds_ft *ft)
{
	return (struct cds_ft_inode_flag *) (uintptr_t) ft;
}

static inline
bool ft_parent_is_trie(const struct cds_ft_inode_flag *parent)
{
	return parent && !((uintptr_t) parent & FT_PARENT_TAG_MASK);
}

urcu_static_assert(!(__alignof__(struct cds_ft) % FT_PARENT_TRIE_ALIGN),
		"struct cds_ft must be aligned past FT's in-band parent tag: a trie pointer lives in the transacted metadata->parent slot",
		ft_trie_alignment_covers_parent_tag);

static inline
struct cds_ft *ft_parent_trie(struct cds_ft_inode_flag *parent)
{
	return (struct cds_ft *) (uintptr_t) parent;
}

/*
 * True for a node at the top of a trie -- the position whose slot is
 * &ft->root.  Replaces the "parent == NULL" root test.
 */
static inline
bool ft_parent_is_root_position(const struct cds_ft_inode_flag *parent)
{
	return !parent || ft_parent_is_trie(parent);
}

/*
 * The node ABOVE this position, or NULL at a root.
 *
 * Every consumer asking "who is my parent NODE" -- an up-walk, a republish
 * into the parent's slot, a parent-slot offset -- goes through here, because a
 * root's stored parent names its owning TRIE, which is not a node and must
 * never be walked or dereferenced as one.  Takes the value rather than the
 * metadata so each caller keeps its own load and memory ordering.
 *
 * The two readers that want the raw word instead: ft_resolve_parent_slot,
 * which turns it into &ft->root, and cds_ft_verify, which checks ownership.
 * A transacted store also reads it raw -- the expected-old must be the word
 * as it stands.
 */
static inline
struct cds_ft_inode_flag *ft_parent_node(struct cds_ft_inode_flag *parent)
{
	return ft_parent_is_trie(parent) ? NULL : parent;
}

/*
 * The word to STORE in a metadata->parent, given the parent NODE (NULL at a
 * root).  Inverse of ft_parent_node, and the reason the two are a pair: a
 * publish path computes its parent as a node -- NULL meaning "into
 * &ft->root" -- and must not write that NULL through, or the node it builds
 * becomes a root belonging to nobody and the depth-0 ownership check has
 * nothing to compare.
 *
 * A copy that inherits an existing parent word verbatim does not need this;
 * the word it copies already names whatever it should.
 */
static inline
struct cds_ft_inode_flag *ft_parent_word(const struct cds_ft *ft,
		struct cds_ft_inode_flag *parent_node)
{
	return parent_node ? parent_node : ft_trie_parent(ft);
}

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
/*
 * The same, for @bulk_lock.  A SEPARATE node because one thread holds both at
 * once (bulk_lock outside, writer_lock inside) and a FIFO node serves one queue
 * at a time.  @ft_bulk_lock_held / @ft_bulk_lock_depth make the take reentrant:
 * bulk bodies NEST (a rekey's staged fallback re-enters the gate), which is why
 * ft_bulk_self_depth is depth-counted, and a non-reentrant take here would wedge
 * the op on its own lock.
 */
static __thread struct cds_fair_mutex_node ft_bulk_lock_waiter;
extern __thread unsigned long ft_bulk_self_depth;
static __thread struct cds_ft *ft_bulk_lock_held;
static __thread unsigned long ft_bulk_lock_depth;

#ifdef FT_DEBUG_BULK_LOCK
/*
 * TEMPORARY: is the bulk-vs-bulk lock actually EXERCISED, and is it HELD across
 * the drain seams it exists to close?  A green run proves nothing if the lock
 * is never taken.  @seam_under is the number that matters: gp_waits reached
 * while holding it -- each one is a seam a peer bulk op can no longer enter.
 */
/* Weak: this header is an impl unit included by more than one TU. */
__attribute__((weak)) unsigned long ft_bl_takes;
__attribute__((weak)) unsigned long ft_bl_nested;
__attribute__((weak)) unsigned long ft_bl_seam_under;
__attribute__((weak)) unsigned long ft_bl_seam_all;
__attribute__((weak)) unsigned long ft_bl_seam_excl;
__attribute__((weak)) unsigned long ft_bl_seam_selfdepth;
__attribute__((weak)) unsigned long ft_bl_seam_selfdepth;
__attribute__((weak)) unsigned long ft_bl_reported;
static void ft_bl_report(void) __attribute__((destructor));
static void ft_bl_report(void)
{
	if (!ft_bl_takes && !ft_bl_seam_all)
		return;
	if (__atomic_fetch_add(&ft_bl_reported, 1, __ATOMIC_RELAXED))
		return;			/* one destructor per TU; print once */
	fprintf(stderr, "FT_BULK_LOCK  takes=%lu nested=%lu | gp_wait seams: "
		"total=%lu UNDER the bulk lock=%lu (of the rest: "
		"exclusive=%lu in-a-bulk-body=%lu)\n",
		ft_bl_takes, ft_bl_nested, ft_bl_seam_all, ft_bl_seam_under,
		ft_bl_seam_excl, ft_bl_seam_selfdepth);
}
# define FT_BL_TALLY(c)	__atomic_fetch_add(&(c), 1, __ATOMIC_RELAXED)
#else
# define FT_BL_TALLY(c)	do { } while (0)
#endif
static __thread struct cds_ft *ft_wlock_held;
static __thread unsigned long ft_wlock_depth;

/*
 * RED CONTROL for the rekey's single-recompaction fold -- a deliberately BROKEN
 * build, never shipped and never a default.  It stops
 * ft_rekey_graft_simple_attempt arming @pending_del_slot, so a same-junction
 * move ATTACHES to BP and then DETACHES from it as two recompactions again: the
 * second reads the child count of the copy the first retired, fuses a boundary
 * that keeps two children, and retires a child the attach just re-parented.
 * test_merge_rekey_same_trie then aborts on the engine's SW-xor-MW slot rule
 * (urcu_txn_record_chain, r->kind == kind) -- which is the failure the fold
 * exists to remove, so this knob is how that claim stays falsifiable.
 */
#ifdef FT_RED_REKEY_NOLOCK
/*
 * RED CONTROL for inv_rekey_coarse_mixed_writers -- a deliberately BROKEN
 * build, never shipped and never a default.  Set across ft_rekey_dispatch, it
 * makes the rekey take no FT-wide writer lock, which is exactly the state the
 * atomic rekey was in before the @lock_fine gate was understood: on a COARSE
 * trie every OTHER writer excludes through that mutex, so a rekey that skips it
 * arbitrates with nobody and its edge installs are lost updates against a
 * concurrent insert or remove.
 *
 * Injected HERE rather than by deleting a CDS_FT_SCOPED_WRITER because the
 * question the control asks is about the whole op -- "does this writer join the
 * protocol at all" -- and the staged path nests several scopes.
 */
static __thread int ft_red_rekey_nolock;
extern unsigned long ft_red_rekey_nolock_taken;	/* GLOBAL: outlives the writers */
#endif

#ifdef FT_DEBUG_BULK_ELEV
/*
 * ★ COUNT THE ARM (G5.25 bulk gate).  A test that mixes bulk and point writers
 * is only testing the bulk-vs-point pairing if the point ops actually LAND in a
 * bulk window; otherwise its green says nothing about the gate.  These count
 * every writer scope that took the FT-wide lock BECAUSE the gate was up, split
 * by who is asking:
 *
 *   @ft_bulk_elev_self  the bulk op's OWN scope, inside its own gate.  Nonzero
 *                       says the gate-holder elevates too -- i.e. the FT-wide
 *                       lock really is the word both sides meet on.  Zero says
 *                       the bulk op opted out and the gate only serializes
 *                       point ops against each other for its duration.
 *   @ft_bulk_elev_peer  a PEER op that found the gate up and elevated.  This is
 *                       the coverage number: it is how many point ops ran
 *                       inside a bulk window.
 *
 * Debug-only, and the thread depth is the only way to tell the two apart (the
 * gate word itself cannot say who raised it -- it is refcounted).
 */
extern unsigned long ft_bulk_elev_self;
extern unsigned long ft_bulk_elev_peer;
static __thread unsigned long ft_bulk_gate_depth;
/*
 * And the same split over the OUTERMOST scopes that actually TAKE the FT-wide
 * lock (reentrant nested scopes are excluded -- they hold it already).  These
 * say who is on the lock, where the pair above says who was asked to elevate.
 */
extern unsigned long ft_bulk_take_self;
extern unsigned long ft_bulk_take_peer;
extern unsigned long ft_bulk_gate_calls;	/* ft_bulk_gate_enter calls */
extern unsigned long ft_bulk_scope_under_gate;	/* outer scopes with depth > 0 */
#endif

/*
 * The access validator's writer OWNER word follows the FT-WIDE LOCK, not the
 * writer scope, and ft_writer_lock_gp_wait is why.  That function drops the lock
 * mid-scope by design, so on a COARSE trie two writer SCOPES legitimately overlap
 * -- and the validator's premise for keeping the single-owner CAS there ("a COARSE
 * trie serializes under the FT-wide mutex, so its owner word is accurate anyway")
 * stops holding.  It fired the moment writers began making progress at all:
 * "writer conflict -- owner ..., entering thread ...".
 *
 * So hand the claim back with the lock and retake it with the lock.  A no-op where
 * the validator counts instead of owning (concurrent + FINE), and where there is no
 * lock to drop (an exclusive trie skips it, and has no concurrent writer to race).
 */
#ifdef FEATURE_FT_EXCL_VALIDATE
static inline
unsigned long ft_excl_owner_release(struct cds_ft *ft)
{
	unsigned long depth;

	if (ft->lock_fine && !ft->exclusive)
		return 0;			/* counted, not owned */
	depth = ft->excl_writer_depth;
	ft->excl_writer_depth = 0;
	uatomic_store(&ft->excl_owner, 0, CMM_RELEASE);
	return depth;
}

static inline
void ft_excl_owner_reclaim(struct cds_ft *ft, unsigned long depth)
{
	if (!depth)
		return;
	/* We hold the lock again, so we ARE the owner: store, do not contend. */
	uatomic_store(&ft->excl_owner, (unsigned long) pthread_self(),
		CMM_RELEASE);
	ft->excl_writer_depth = depth;
}
#else
static inline
unsigned long ft_excl_owner_release(struct cds_ft *ft __attribute__((unused)))
{
	return 0;
}

static inline
void ft_excl_owner_reclaim(struct cds_ft *ft __attribute__((unused)),
		unsigned long depth __attribute__((unused)))
{
}
#endif

/*
 * THE TWO WAITS ON THE FT-WIDE WRITER LOCK, AND WHY ONLY ONE OF THEM QUIESCES.
 *
 * Both waits below block on the same fair mutex; they differ in what the
 * WAITING THREAD is still answering for, so they differ in whether going
 * RCU-offline is permitted.
 *
 *   ft_writer_lock_take()  SCOPE ENTRY -- stays ONLINE.  The caller may be
 *                          inside its own RCU read-side critical section and
 *                          may hold reader-derived references across the call;
 *                          fractal-trie.h's reference-lifetime contract
 *                          REQUIRES that section be continuous, because
 *                          cds_ft_remove / cds_ft_replace dereference the
 *                          caller's @node (via @node->prev) with no
 *                          re-descent.  Under QSBR thread_offline() reports a
 *                          quiescent state, so parking offline HERE would end
 *                          that section and expose @node -- and the caller's
 *                          key with it, since ft_iter_read_key() may return a
 *                          pointer INTO @node.  So this wait does not quiesce.
 *
 *   ft_writer_lock_park()  gp_wait RE-ACQUIRE -- goes OFFLINE.  Sound here and
 *                          only here: the thread has just waited out a full
 *                          grace period, so everything it holds is ALREADY
 *                          exposed to reclamation and must already be kept
 *                          valid by the writer lock / exclusivity rather than
 *                          by this thread's online-ness.  Being offline for
 *                          the subsequent wait adds no exposure.
 *
 * WHAT THE ONLINE ENTRY WAIT COSTS, MEASURED (2026-08-28, -O2 -DNDEBUG, QSBR,
 * inv_concurrent_writers_coarse_lock: 16 writers, one coarse trie, 200 ms):
 * the lock runs ~75% utilized (5811 holds, mean 26 us), and a CONTENDED entry
 * wait is p50 >= 524 us, p99 >= 1.05 ms, max 1.99 ms (31% of acquisitions are
 * uncontended at ~32 ns).  An online waiter is a non-quiescent QSBR reader for
 * exactly that long, so every grace period in the process is delayed by it.
 * ACCEPTED DELIBERATELY (Mathieu, 2026-08-28): correctness of the caller's
 * reference contract outranks grace-period latency on a coarse trie.
 *
 * ☠ IT IS A DELAY, NOT A WEDGE, and that is a property of the GRACE-PERIOD
 * RULE above (@writer_lock): a holder NEVER waits on a grace period while
 * holding, because ft_writer_lock_gp_wait drops the lock across every one.  So
 * an online entry waiter is never waiting on a holder that is itself waiting
 * on that waiter -- it acquires within queue-depth x hold and then completes.
 * The group wedge 781e0b9a measured (four concurrent cds_ft_rekey_graft
 * writers making ZERO moves) had THREE THREADS IN THE gp_wait RE-ACQUIRE, and
 * that site keeps its offline park, so that fix is untouched.
 *
 * Both are a no-op distinction on the memb / mb / bp flavors, where
 * thread_offline() is an empty function: those flavors have always waited
 * online here, and their callers' read sections have always survived.
 */
#ifdef FT_DEBUG_WLOCK_HOLD
/*
 * =====================================================================
 * MEASUREMENT (opt-in: -DFT_DEBUG_WLOCK_HOLD).  NOT part of the library.
 * =====================================================================
 *
 * PRICED THE ENTRY-WAIT DECISION, and it is the source of the numbers
 * quoted in the two-waits comment above.  The entry wait now stays ONLINE
 * (ft_writer_lock_take), so a waiter is a non-quiescent QSBR reader for its
 * whole WAIT and delays every grace period in the process for that long.
 * WAIT is therefore the grace-period latency the coarse path imposes, and
 * HOLD is what generates it.  Re-run this whenever the coarse hold
 * distribution could have moved -- the accepted cost is only as good as
 * the distribution it was accepted against.
 *
 * So WAIT is the number the decision needs, and HOLD is what generates it.
 * Both are recorded in log2(ns) buckets and dumped at process exit.  There
 * is exactly one acquire path (this function, reached from both sites) and
 * two release sites (scope_exit and gp_wait's drop).
 *
 * ☠ COARSE ONLY BY CONSTRUCTION: a FINE trie returns from
 * ft_writer_lock_scope_enter before the park, so a FINE run records ZERO
 * -- which is a CONFIGURATION check, not a measurement.  Read the n= line
 * before reading anything else.
 */
#include <stdio.h>
#include <time.h>

#define FT_WLH_BUCKETS	40
static unsigned long ft_wlh_hold[FT_WLH_BUCKETS];
static unsigned long ft_wlh_wait[FT_WLH_BUCKETS];
static unsigned long ft_wlh_hold_n, ft_wlh_wait_n;
static unsigned long ft_wlh_hold_sum, ft_wlh_wait_sum;
static unsigned long ft_wlh_hold_max, ft_wlh_wait_max;
static __thread uint64_t ft_wlh_acq_ns;

static inline
uint64_t ft_wlh_now(void)
{
	struct timespec ts;

	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint64_t) ts.tv_sec * 1000000000ULL + (uint64_t) ts.tv_nsec;
}

static inline
void ft_wlh_record(unsigned long *hist, unsigned long *n, unsigned long *sum,
		unsigned long *max, uint64_t ns)
{
	unsigned int b = 0;
	uint64_t v = ns;

	while (v >>= 1)
		b++;
	if (b >= FT_WLH_BUCKETS)
		b = FT_WLH_BUCKETS - 1;
	uatomic_inc(&hist[b]);
	uatomic_inc(n);
	uatomic_add(sum, (unsigned long) ns);
	if (ns > uatomic_load(max, CMM_RELAXED))
		uatomic_store(max, (unsigned long) ns, CMM_RELAXED);
}

static
void ft_wlh_dump_one(const char *tag, const unsigned long *hist,
		unsigned long n, unsigned long sum, unsigned long max)
{
	unsigned long cum = 0, p50 = 0, p90 = 0, p99 = 0;
	unsigned int i;

	fprintf(stderr, "FT WLOCK %s: n=%lu mean=%luns max=%luns\n",
		tag, n, n ? sum / n : 0UL, max);
	for (i = 0; i < FT_WLH_BUCKETS; i++) {
		if (!hist[i])
			continue;
		cum += hist[i];
		if (!p50 && cum * 100 >= n * 50)
			p50 = 1UL << i;
		if (!p90 && cum * 100 >= n * 90)
			p90 = 1UL << i;
		if (!p99 && cum * 100 >= n * 99)
			p99 = 1UL << i;
		fprintf(stderr, "  >=2^%-2u (%12lu ns) %10lu\n",
			i, 1UL << i, hist[i]);
	}
	/* log2 buckets: each percentile is a LOWER bound, exact to 2x. */
	fprintf(stderr, "FT WLOCK %s: p50>=%luns p90>=%luns p99>=%luns\n",
		tag, p50, p90, p99);
}

__attribute__((destructor))
static
void ft_wlh_dump(void)
{
	if (!ft_wlh_hold_n && !ft_wlh_wait_n) {
		fprintf(stderr, "FT WLOCK: NO SAMPLES -- the FT-wide lock was "
			"never taken (a FINE or exclusive trie).  This is a "
			"configuration miss, not a measurement.\n");
		return;
	}
	ft_wlh_dump_one("HOLD", ft_wlh_hold, ft_wlh_hold_n, ft_wlh_hold_sum,
			ft_wlh_hold_max);
	ft_wlh_dump_one("WAIT", ft_wlh_wait, ft_wlh_wait_n, ft_wlh_wait_sum,
			ft_wlh_wait_max);
}
#endif /* FT_DEBUG_WLOCK_HOLD */

#ifdef FT_DEBUG_WLOCK_HOLD
# define FT_WLH_WAIT_BEGIN	uint64_t ft_wlh_t0 = ft_wlh_now()
# define FT_WLH_WAIT_END						\
	do {								\
		uint64_t t1__ = ft_wlh_now();				\
									\
		ft_wlh_record(ft_wlh_wait, &ft_wlh_wait_n,		\
				&ft_wlh_wait_sum, &ft_wlh_wait_max,	\
				t1__ - ft_wlh_t0);			\
		ft_wlh_acq_ns = t1__;					\
	} while (0)
#else
# define FT_WLH_WAIT_BEGIN	do { } while (0)
# define FT_WLH_WAIT_END	do { } while (0)
#endif

/* SCOPE ENTRY: stays ONLINE -- see the two-waits comment above. */
static inline
void ft_writer_lock_take(struct cds_ft *ft)
{
	FT_WLH_WAIT_BEGIN;

	cds_fair_mutex_lock(&ft->writer_lock, &ft_wlock_waiter);
	FT_WLH_WAIT_END;
}

/* gp_wait RE-ACQUIRE: goes OFFLINE -- see the two-waits comment above. */
static inline
void ft_writer_lock_park(struct cds_ft *ft)
{
	const struct rcu_flavor_struct *flavor = ft->group->flavor;
	FT_WLH_WAIT_BEGIN;

	flavor->thread_offline();
	cds_fair_mutex_lock(&ft->writer_lock, &ft_wlock_waiter);
	flavor->thread_online();
	FT_WLH_WAIT_END;
}

/*
 * Take the FT-wide writer lock at the OUTERMOST writer scope; reentrant no-op
 * on a nested scope for the same trie.  Every trie is lock-mode now (COARSE or
 * FINE), so there is no optimistic early-out; a FINE trie skips the FT-wide
 * lock further down, under the lock-set drop.
 */
/*
 * G5.25: while a bulk op is live, a FINE trie RE-TAKES the FT-wide writer lock
 * instead of widening every point op's lock-set to the root.  On by default --
 * it is the shipping bulk-vs-point exclusion, and the ONLY one -- the
 * per-op lock-set widening it replaced was removed once this landed.  Set to 0
 * only to measure its cost; a 0 build has NO bulk-vs-point exclusion at all.
 */
#ifndef FT_BULK_WIDE_LOCK
# define FT_BULK_WIDE_LOCK	1
#endif

/* Defined below with the rest of the bulk gate; needed by the scope enter. */
static inline bool ft_bulk_active(const struct cds_ft *ft);

/*
 * The writer scopes' own read sections this thread holds (see
 * ft_writer_lock_scope_enter), and the flavor they were taken with.
 * ft_writer_lock_gp_wait drops them across its grace period, exactly as it
 * drops the FT-wide lock: a writer scope that waits for a grace period outside
 * a bulk gate (cds_ft_make_exclusive) would otherwise wait for itself.
 */
static __thread unsigned long ft_wscope_rcu_held;
static __thread const struct rcu_flavor_struct *ft_wscope_rcu_flavor;
#ifdef URCU_FRACTAL_TRIE_DEBUG_LOCKING
/*
 * Those sections would also blind CDS_FT_ASSERT_RCU_READ_LOCKED at the point
 * ops that require the CALLER to hold one (a cached iterator's node must stay
 * live): the library's section satisfies read_ongoing().  So record the
 * caller's state at the outermost such section, and have those sites ask it
 * (CDS_FT_ASSERT_CALLER_RCU_READ_LOCKED).
 */
static __thread int ft_wscope_caller_reading;
#endif

/*
 * Returns true when the scope holds a read-side section of @ft's flavor,
 * which ft_writer_lock_scope_exit's caller must drop (see the FINE arm).
 */
static inline
bool ft_writer_lock_scope_enter(struct cds_ft *ft)
{
	if (ft_wlock_held == ft) {
		ft_wlock_depth++;		/* reentry on the trie we hold */
		return false;
	}
	if (ft->external_sync) {
		/*
		 * CDS_FT_WRITER_EXTERNAL_SYNC: the caller excludes every writer,
		 * so there is no FT-wide lock to take -- that is the whole
		 * point of the mode.  Readers are UNAFFECTED (see @external_sync):
		 * this skips the writer mutex, nothing else.
		 *
		 * Placed beside the @exclusive early-out and after the
		 * reentrancy test for the same reason, and safe for the same
		 * reason: ft_writer_lock_scope_exit keys its release off
		 * @ft_wlock_held IDENTITY, never a re-read of either flag.
		 */
		return false;
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
		return false;
	}
#ifdef FT_RED_REKEY_NOLOCK
	/*
	 * RED CONTROL, never a shipped configuration.  See @ft_red_rekey_nolock:
	 * with it set this thread's writer scopes take NO FT-wide lock, which is
	 * precisely the defect inv_rekey_coarse_mixed_writers exists to detect.
	 * Placed FIRST so no lock state is recorded at all -- the exit keys off
	 * @ft_wlock_held identity and so no-ops by itself, and
	 * ft_writer_lock_gp_wait sees held == NULL and degrades to a plain
	 * synchronize_rcu.  The injection is therefore state-balanced.
	 */
	if (ft_red_rekey_nolock) {
		/*
		 * ★ COUNT THE ARM.  A red control that is never TAKEN is
		 * indistinguishable from a green one, and this one's whole
		 * purpose is to make a green mean something -- so report how
		 * many scopes it actually skipped, as FT_RED_PARENT_WORD_SW
		 * does.  Measured 68,655 on inv_rekey_coarse_mixed_writers.
		 */
		uatomic_inc(&ft_red_rekey_nolock_taken);
		return false;
	}
#endif
#ifdef FT_DEBUG_BULK_ELEV
	if (ft_bulk_gate_depth)
		uatomic_inc(&ft_bulk_scope_under_gate);
	if (ft->lock_fine && FT_BULK_WIDE_LOCK
			&& caa_unlikely(ft_bulk_active(ft)))
		uatomic_inc(ft_bulk_gate_depth ? &ft_bulk_elev_self
			: &ft_bulk_elev_peer);
#endif
	/*
	 * ☠ THE GATE IS SAMPLED INSIDE A READ SECTION THAT SPANS THE OP.  The
	 * bulk gate publishes @bulk_state and then waits ONE grace period, so it
	 * excludes exactly the point ops that were inside a read-side section
	 * when they read it as clear.  The per-attempt section urcu_txn_begin
	 * takes opens only AFTER this sample, and closes between attempts -- and
	 * a caller need not hold one of its own (cds_ft_insert and friends
	 * require none).  Under QSBR an online thread is a reader throughout, so
	 * that gap was invisible; under memb / mb / bp it let a point op that
	 * sampled "clear" run LOCK-FREE through a bulk body.  MEASURED with a
	 * memb build of inv_rekey_fine_mixed_writers (rekeys paced 2 ms apart)
	 * and a probe flagging a point attempt that starts unlocked while a bulk
	 * op holds the FT-wide lock past its gate: up to 2 per run as is, and
	 * 17-166 in 6 of 6 runs with a 2 ms preemption injected after the
	 * sample; 0 in 6 of 6 with the section taken first.
	 *
	 * So take the section BEFORE the sample.  Clear: keep it until the scope
	 * exits -- every attempt's own section nests inside it.  Raised: drop it
	 * and take the FT-wide lock, the exclusion that case uses.  That branch
	 * is also the one a BULK op's own scope takes (its gate is up), so a
	 * bulk op, which waits for grace periods, never holds this section.  A
	 * point op holding it never waits on anything a GP-waiting bulk op holds:
	 * the gate's GP runs before the bulk op takes any lock, and
	 * ft_writer_lock_gp_wait drops the FT-wide lock across every later one.
	 */
	if (ft->lock_fine) {
#ifndef FT_DEBUG_NO_SCOPE_READ_SECTION
#ifdef URCU_FRACTAL_TRIE_DEBUG_LOCKING
		if (!ft_wscope_rcu_held)
			ft_wscope_caller_reading =
				ft->group->flavor->read_ongoing();
#endif
		ft->group->flavor->read_lock();
#endif
		if (!(FT_BULK_WIDE_LOCK && caa_unlikely(ft_bulk_active(ft)))) {
			/*
			 * ☑ G5.25 -- AND THE ONE CASE THAT TAKES IT BACK.  While a BULK
			 * op is live, a FINE trie RE-TAKES the FT-wide lock, so bulk and
			 * point writers arbitrate on one word again.  That is the whole
			 * of G5.5's exclusion, obtained by NOT dropping a lock that
			 * already exists rather than by widening every point op's
			 * lock-set to the root:
			 *   - no ancestor ledger to consume, no up-walk to date members;
			 *   - NO RELEASE-OWNER PROBLEM AT ALL -- a scoped mutex has no
			 *     registry, no reservation to size, and no "acquire before
			 *     the txn exists" blocker (the four sites that defeated the
			 *     widening's coverage);
			 *   - and it QUEUES.  The widened DLM acquire spins
			 *     URCU_TXN_WAIT_PATIENCE and then ABORTS, which is the
			 *     retry-storm hazard; cds_fair_mutex is FIFO.
			 * ★ Sound against the seam rule for free: ft_writer_lock_gp_wait
			 * DROPS this lock across every grace period (@writer_lock: "the
			 * GP always sits at a seam BETWEEN two distinct commits, so
			 * releasing there costs no atomicity").
			 * ★ And the sample races nothing: a point op that read the gate
			 * as clear is inside the read section taken just above (NOT the
			 * one urcu_txn_begin takes later -- see above), which is exactly
			 * what the gate's publish-then-one-GP waits for.
			 * ☠ The cost is honest and accepted (2026-08-29): while any bulk
			 * op is live, point ops serialize trie-wide.  At level 0 the
			 * widening serialized them on the ROOT's lock anyway, so this
			 * trades an equivalent exclusion for far less machinery.
			 */
			/*
			 * FT-WIDE-LOCK DROP (§11 drop-mechanics, MCAS-first): a FINE trie's
			 * op-domains are ALL converted to per-node lock-sets (LOCK
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
#ifndef FT_DEBUG_NO_SCOPE_READ_SECTION
			/* Nested sections of one thread share one flavor. */
			assert(!ft_wscope_rcu_held ||
				ft_wscope_rcu_flavor == ft->group->flavor);
			if (!ft_wscope_rcu_held++)
				ft_wscope_rcu_flavor = ft->group->flavor;
			return true;
#else
			return false;
#endif
		}
#ifndef FT_DEBUG_NO_SCOPE_READ_SECTION
		ft->group->flavor->read_unlock();
#endif
	}
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
#ifdef FT_DEBUG_BULK_ELEV
	uatomic_inc(ft_bulk_gate_depth ? &ft_bulk_take_self
		: &ft_bulk_take_peer);
#endif
	ft_writer_lock_take(ft);
	ft_wlock_held = ft;
	ft_wlock_depth = 1;
	return false;
}

/* Release at the OUTERMOST scope only; a nested exit just unwinds the depth. */
static inline
void ft_writer_lock_scope_exit(struct cds_ft *ft)
{
	/*
	 * Unwind only the lock THIS thread actually holds for @ft.  When the
	 * enter SKIPPED the lock (an exclusive trie) or we hold a DIFFERENT
	 * trie's lock, @ft_wlock_held != ft and there is nothing to release.
	 * Keying off identity (not a re-read of @ft->exclusive) means an
	 * @exclusive flip between enter and exit -- cds_ft_make_exclusive
	 * (false->true) or cds_ft_make_shared (true->false) -- can never
	 * unbalance us: make_exclusive took the lock at enter (exclusive was
	 * false) and releases it here (held == ft), make_shared skipped it
	 * at enter (exclusive was true) and skips it here (held != ft).
	 */
	if (ft_wlock_held != ft)
		return;
	if (--ft_wlock_depth != 0)
		return;			/* nested scope: keep the lock held */
	ft_wlock_held = NULL;
#ifdef FT_DEBUG_PAIR_STORE
	if (ft_dt_glue_ring_n) {
		memset(ft_dt_glue_ring, 0, sizeof(ft_dt_glue_ring));
		ft_dt_glue_ring_n = 0;
	}
#endif
#ifdef FT_DEBUG_WLOCK_HOLD
	ft_wlh_record(ft_wlh_hold, &ft_wlh_hold_n, &ft_wlh_hold_sum,
			&ft_wlh_hold_max, ft_wlh_now() - ft_wlh_acq_ns);
#endif
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
#ifdef FT_DEBUG_REMOVE_RETRY_CAP
/* ★ See the remove retry exit: -EAGAIN (contention) vs -ENOENT (dead derivation). */
static __thread unsigned long ft_dbg_rm_eagain;
static __thread unsigned long ft_dbg_rm_enoent;
#endif

#ifdef FT_DEBUG_REMOVE_RETRY_CAP
# include <time.h>
static __thread uint64_t ft_dbg_gp_ns;
static __thread unsigned int ft_dbg_gp_calls;
/* Arena-lock wait accounting; defined in fractal-trie-alloc.c (its TU). */
extern __thread uint64_t ft_dbg_arena_ns;
extern __thread unsigned int ft_dbg_arena_waits;

static inline uint64_t ft_dbg_gp_clock(void)
{
	struct timespec ts;

	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint64_t) ts.tv_sec * 1000000000ULL + (uint64_t) ts.tv_nsec;
}
#endif

#ifdef FT_DEBUG_BULK_WINDOW
/*
 * =====================================================================
 * MEASUREMENT (opt-in: -DFT_DEBUG_BULK_WINDOW).  NOT part of the library.
 * =====================================================================
 *
 * PRICES G5.5.  Under G5.5 a point op, while a bulk op holds the move gate,
 * ADDS every ancestor lock up to the ROOT -- so the trie serializes for as
 * long as the gate is held.  The number that decides whether that is
 * affordable is the UNION WINDOW: the interval during which move_gate_nr > 0.
 *
 * ☠ IT IS NOT per-op duration x rate.  The gate is REFCOUNTED, so concurrent
 * movers COALESCE: a burst can hold ONE window open end to end, and the 0->1
 * owner's grace period is paid ONCE for all of them.  Multiplying a per-op
 * duration by a rate therefore overcounts windows and undercounts their
 * length.  Only the transitions can answer it, so that is what is recorded:
 *
 *   WINDOW  0->1 .. 1->0, the interval point ops would pay for;
 *   GAP     1->0 .. next 0->1, the trie's free time between windows;
 *   GP      the 0->1 owner's own update_synchronize_rcu().
 *
 * The GP split is the ACTIONABLE half: if a window is nearly all GP, G5.5's
 * cost is grace-period latency and the lever is opening fewer windows, not
 * shortening bulk bodies.  Attribution is exact -- the GP is timed by the
 * owner around its own call, never inferred across threads.
 *
 * Both transitions already happen under @move_gate_lock and there is exactly
 * one 0->1 and one 1->0 per window, so the open/close pairing is TOTAL and
 * needs no lock of its own.  @bw_t_open lives in struct cds_ft, not here,
 * because the gate is per-trie.
 *
 * ☠☠ THE GATE ALONE MEASURES THE CHEAPEST MEMBER OF THE FAMILY, so it is NOT
 * reported alone.  Only rekey enters the gate today (ft-rekey.h:3667, :5991,
 * plus the ft_unit-only hold-open shim at fractal-trie.c:303, whose windows are
 * a test fixture and not a measurement).  G5.5 must extend it to the other six
 * public bulk entries -- and THOSE have the long windows: exactly two functions
 * in the FT reach update_synchronize_rcu (ft-merge.h:2587), and the second,
 * ft_writer_lock_gp_wait, is called from INSIDE bulk bodies -- graft 4 sites,
 * detach 3, and rekey's own staged fallback 2.  The rekey ONE-DECIDE path the
 * gate hook does see was built to avoid mid-op GPs entirely (ft-rekey.h:3543),
 * so measuring it and calling it the family would bias the answer toward the
 * shortest member, in the direction that flatters G5.5.
 *
 * So the gate hook is the CALIBRATION POINT, and the number G5.5 actually needs
 * is the PER-CLASS BODY: a wall-clock bracket around each of the 7 public bulk
 * entries, which is the window each would open once the gate is extended over
 * it.  BODY already contains that op's in-body GPs; BODY_GP reports how much of
 * it they are.  A run without a given class records ZERO for it, which is a
 * CONFIGURATION MISS, not "that op has no window" -- so the dump says which
 * classes ran, in words.  Read the n= line before reading anything else.
 *
 * ☠ AND IT PRICES ONE HALF ONLY.  This measures EXPOSURE (how often, how long
 * the trie is serialized).  It cannot price the LIVENESS risk -- point ops
 * widened to the root spin-abort-retrying on a queueless acquire -- because
 * that regime DOES NOT EXIST in this binary: point ops do not widen yet.  No
 * hook on today's gate can observe it, and duration does not bound it (a
 * starving lane is on record here: the remove retry lane does not drain under
 * lock-holder preemption, and acquire backoff was refuted).  Liveness is the
 * first gate on the G5.5 BUILD, not a measurement that precedes it.
 */
#include <stdio.h>
#include <time.h>

#define FT_BW_BUCKETS	40
static unsigned long ft_bw_win[FT_BW_BUCKETS];
static unsigned long ft_bw_gap[FT_BW_BUCKETS];
static unsigned long ft_bw_gp[FT_BW_BUCKETS];
static unsigned long ft_bw_win_n, ft_bw_gap_n, ft_bw_gp_n;
static unsigned long ft_bw_win_sum, ft_bw_gap_sum, ft_bw_gp_sum;
static unsigned long ft_bw_win_max, ft_bw_gap_max, ft_bw_gp_max;
static unsigned long ft_bw_opens;	/* 0->1 transitions */
static unsigned long ft_bw_enters;	/* every enter: /opens = PIGGYBACK */
static unsigned long ft_bw_peak;	/* max movers coalesced in one window */
static unsigned long ft_bw_piggyback;	/* movers that WAITED on the owner's GP */
static uint64_t ft_bw_first_open, ft_bw_last_close;

/*
 * PER-CLASS BODY.  One bracket per public bulk entry; the index order is the
 * order they are declared in include/urcu/fractal-trie.h.
 */
enum ft_bw_class {
	FT_BW_GRAFT, FT_BW_GRAFT_SWAP, FT_BW_DETACH,
	FT_BW_MERGE_AT, FT_BW_REKEY_GRAFT, FT_BW_REKEY_MERGE, FT_BW_NR_CLASS,
};
/*
 * cds_ft_merge has no class of its own: it is the same-prefix case and
 * delegates to cds_ft_merge_at (ft-merge.h:3822), so it is counted there.
 * Bracketing both would count one op twice.
 */
static const char * const ft_bw_class_name[FT_BW_NR_CLASS] = {
	"graft", "graft_swap", "detach", "merge_at",
	"rekey_graft", "rekey_merge",
};
static unsigned long ft_bw_body[FT_BW_NR_CLASS][FT_BW_BUCKETS];
static unsigned long ft_bw_body_n[FT_BW_NR_CLASS];
static unsigned long ft_bw_body_sum[FT_BW_NR_CLASS];
static unsigned long ft_bw_body_max[FT_BW_NR_CLASS];
static unsigned long ft_bw_body_gp_sum[FT_BW_NR_CLASS];
static unsigned long ft_bw_body_gp_n[FT_BW_NR_CLASS];	/* ops with >=1 in-body GP */

/*
 * IN-BODY GRACE PERIODS, accumulated by the calling thread at BOTH sites that
 * reach update_synchronize_rcu (ft_writer_lock_gp_wait and the gate's own 0->1
 * wait).  __thread is correct HERE -- an op's body GPs are run by the op's own
 * thread -- whereas the window's open/close are routinely DIFFERENT threads,
 * which is why @bw_t_open is per-trie instead.
 */
static __thread uint64_t ft_bw_gp_acc;
static __thread unsigned long ft_bw_gp_acc_n;

static inline
uint64_t ft_bw_now(void)
{
	struct timespec ts;

	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint64_t) ts.tv_sec * 1000000000ULL + (uint64_t) ts.tv_nsec;
}

static inline
void ft_bw_record(unsigned long *hist, unsigned long *n, unsigned long *sum,
		unsigned long *max, uint64_t ns)
{
	unsigned int b = 0;
	uint64_t v = ns;

	while (v >>= 1)
		b++;
	if (b >= FT_BW_BUCKETS)
		b = FT_BW_BUCKETS - 1;
	uatomic_inc(&hist[b]);
	uatomic_inc(n);
	uatomic_add(sum, (unsigned long) ns);
	if (ns > uatomic_load(max, CMM_RELAXED))
		uatomic_store(max, (unsigned long) ns, CMM_RELAXED);
}

static __attribute__((unused))
void ft_bw_dump_one(const char *tag, const unsigned long *hist,
		unsigned long n, unsigned long sum, unsigned long max)
{
	unsigned long cum = 0, p50 = 0, p90 = 0, p99 = 0;
	unsigned int i;

	fprintf(stderr, "FT BULKWIN %s: n=%lu mean=%luns max=%luns\n",
		tag, n, n ? sum / n : 0UL, max);
	for (i = 0; i < FT_BW_BUCKETS; i++) {
		if (!hist[i])
			continue;
		cum += hist[i];
		if (!p50 && cum * 100 >= n * 50)
			p50 = 1UL << i;
		if (!p90 && cum * 100 >= n * 90)
			p90 = 1UL << i;
		if (!p99 && cum * 100 >= n * 99)
			p99 = 1UL << i;
		fprintf(stderr, "  >=2^%-2u (%12lu ns) %10lu\n",
			i, 1UL << i, hist[i]);
	}
	/* log2 buckets: each percentile is a LOWER bound, exact to 2x. */
	fprintf(stderr, "FT BULKWIN %s: p50>=%luns p90>=%luns p99>=%luns\n",
		tag, p50, p90, p99);
}

/*
 * The bracket.  Wall clock plus the GP time this thread accumulated inside the
 * body, so BODY and BODY_GP are reported against the same interval.
 */
struct ft_bw_op {
	uint64_t t0;
	uint64_t gp0;
	unsigned long gpn0;
	enum ft_bw_class cls;
};

static inline
void ft_bw_op_begin(struct ft_bw_op *op, enum ft_bw_class cls)
{
	op->cls = cls;
	op->gp0 = ft_bw_gp_acc;
	op->gpn0 = ft_bw_gp_acc_n;
	op->t0 = ft_bw_now();
}

static inline
void ft_bw_op_end(struct ft_bw_op *op)
{
	uint64_t d = ft_bw_now() - op->t0;

	ft_bw_record(ft_bw_body[op->cls], &ft_bw_body_n[op->cls],
			&ft_bw_body_sum[op->cls], &ft_bw_body_max[op->cls], d);
	uatomic_add(&ft_bw_body_gp_sum[op->cls],
			(unsigned long) (ft_bw_gp_acc - op->gp0));
	if (ft_bw_gp_acc_n != op->gpn0)
		uatomic_inc(&ft_bw_body_gp_n[op->cls]);
}

static inline
void ft_bw_op_cleanup(struct ft_bw_op *op)
{
	ft_bw_op_end(op);
}

/*
 * Scoped so that EVERY return path is covered: these entries reject bad
 * arguments with early returns, and a bracket that only wrapped the success
 * path would silently omit them.  Rejections land in the low buckets (they do
 * no work); the workloads here pass valid arguments, so they are cold.
 */
#ifdef FT_BW_ISOLATE_GATE
/* Perturbation A/B only: gate hooks armed, per-class body bracket removed. */
# define FT_BW_OP(cls)		do { } while (0)
#else
#define FT_BW_OP(cls)							\
	struct ft_bw_op ft_bw_op__					\
		__attribute__((cleanup(ft_bw_op_cleanup)));		\
	ft_bw_op_begin(&ft_bw_op__, (cls))
#endif

#ifdef FT_BW_OWNS_DUMP
__attribute__((destructor))
static
void ft_bw_dump(void)
{
	uint64_t span;

	unsigned int c;
	unsigned long any_body = 0;

	for (c = 0; c < FT_BW_NR_CLASS; c++)
		any_body += ft_bw_body_n[c];
	/*
	 * THE PER-CLASS BODY IS THE G5.5 NUMBER: the window each op would open
	 * once the gate is extended over it.  Printed FIRST, and printed even
	 * when the gate itself was never entered, because a graft-only or
	 * detach-only run has no windows today and still has the windows G5.5
	 * would create.
	 */
	if (any_body) {
		fprintf(stderr, "FT BULKWIN: --- PER-CLASS BODY (the PREDICTED "
			"G5.5 window per bulk op) ---\n");
		for (c = 0; c < FT_BW_NR_CLASS; c++) {
			if (!ft_bw_body_n[c]) {
				fprintf(stderr, "FT BULKWIN BODY %-12s: NOT "
					"EXERCISED (configuration miss, not a "
					"zero window)\n", ft_bw_class_name[c]);
				continue;
			}
			ft_bw_dump_one(ft_bw_class_name[c], ft_bw_body[c],
					ft_bw_body_n[c], ft_bw_body_sum[c],
					ft_bw_body_max[c]);
			fprintf(stderr, "FT BULKWIN BODY %-12s: in-body GP "
				"%.2f%% of body time, %lu/%lu ops took >=1 GP\n",
				ft_bw_class_name[c],
				ft_bw_body_sum[c] ? 100.0 *
					(double) ft_bw_body_gp_sum[c] /
					(double) ft_bw_body_sum[c] : 0.0,
				ft_bw_body_gp_n[c], ft_bw_body_n[c]);
		}
	} else {
		fprintf(stderr, "FT BULKWIN: NO BULK OP RAN AT ALL -- "
			"CONFIGURATION MISS, not a measurement.\n");
	}
	if (!ft_bw_opens) {
		fprintf(stderr, "FT BULKWIN: NO WINDOWS -- the move gate was "
			"never entered.  Only rekey enters it today "
			"(ft-rekey.h:3667,:5991); a graft/detach/merge run "
			"legitimately shows zero here while still having the "
			"per-class bodies above.  CONFIGURATION MISS, not a "
			"measurement.\n");
		return;
	}
	ft_bw_dump_one("WINDOW", ft_bw_win, ft_bw_win_n, ft_bw_win_sum,
			ft_bw_win_max);
	ft_bw_dump_one("GP", ft_bw_gp, ft_bw_gp_n, ft_bw_gp_sum, ft_bw_gp_max);
	ft_bw_dump_one("GAP", ft_bw_gap, ft_bw_gap_n, ft_bw_gap_sum,
			ft_bw_gap_max);
	/*
	 * COALESCING, the reason a rate cannot stand in for this: @enters
	 * movers were served by @opens windows and @opens grace periods.
	 */
	fprintf(stderr, "FT BULKWIN: enters=%lu opens=%lu movers_per_window=%.2f "
		"waited_on_owner_GP=%lu peak_concurrent=%lu\n",
		ft_bw_enters, ft_bw_opens,
		ft_bw_opens ? (double) ft_bw_enters / (double) ft_bw_opens : 0.0,
		ft_bw_piggyback, ft_bw_peak);
	if (ft_bw_win_n != ft_bw_opens)
		fprintf(stderr, "FT BULKWIN: %lu window(s) STILL OPEN at exit "
			"and therefore NOT counted -- duty cycle is a LOWER "
			"bound.\n", ft_bw_opens - ft_bw_win_n);
	/*
	 * DUTY CYCLE over the measured span (first open .. last close), which
	 * is the fraction of the time a point op would have been serialized
	 * trie-wide.  Over the SPAN, not over process lifetime: setup and
	 * teardown are not workload.
	 */
	span = ft_bw_last_close - ft_bw_first_open;
	fprintf(stderr, "FT BULKWIN: span=%luns open=%luns DUTY=%.2f%% "
		"(GP share of open time=%.2f%%)\n",
		(unsigned long) span, ft_bw_win_sum,
		span ? 100.0 * (double) ft_bw_win_sum / (double) span : 0.0,
		ft_bw_win_sum ? 100.0 * (double) ft_bw_gp_sum /
				(double) ft_bw_win_sum : 0.0);
}
#endif /* FT_BW_OWNS_DUMP */
#endif /* FT_DEBUG_BULK_WINDOW */

#ifndef FT_DEBUG_BULK_WINDOW
# define FT_BW_OP(cls)		do { } while (0)
#endif

#ifdef FEATURE_FT_HOLD_TRACE

/*
 * THE SEAM-RULE ARM: no NODE lock may be held across a grace period.  Defined
 * with the hold ledger in ft-mutation-helpers.h, which is included long after
 * this header; forward-declared here the way ft_hold_trace_leak_canary is.
 */
static void ft_seam_check(const char *site);
#endif

#ifdef FT_DEBUG_SPLICE_SEAM
/*
 * THE SPLICE-SEAM PROBE (build knob; -DFT_DEBUG_SPLICE_SEAM).
 *
 * ft_glue_record_splices writes @src_head->prev with the SRC head's chain
 * holder NOT acquired (its own comment says so): the exclusion is the FT-wide
 * writer lock plus the bulk gate, and the MW expected-old is the backstop.
 * Converting cds_ft_node.prev to SW removes that backstop, so the question to
 * answer FIRST is whether a DRAIN SEAM -- ft_writer_lock_gp_wait, which DROPS
 * the FT-wide lock and lets a point op in -- can land inside the window where a
 * splice is recorded but not yet committed.
 *
 * ☞ The in-tree claim is STATIC ("No ft_writer_lock_gp_wait exists anywhere
 * under the attempt", ft-rekey.h), and a static claim about a call graph is a
 * test you can run.  @ft_splice_window is the window; the seam counts it.
 *
 * Declared as an opaque pointer because struct ft_glue is defined in
 * ft-mutation-helpers.h, which this header precedes.
 */
static __thread void *ft_splice_window;
static unsigned long ft_ss_seam_all __attribute__((unused)),
	ft_ss_seam_in_window __attribute__((unused));
#endif

#ifdef FT_DEBUG_BULK_ACQ
/*
 * -DFT_DEBUG_BULK_ACQ: drain seams this thread has taken, and the count when its
 * OUTERMOST bulk gate opened -- so an acquire inside a bulk op can say whether a
 * seam already happened earlier in that op.
 */
static __thread unsigned long ft_ba_seam_n, ft_ba_seam_at_gate;
/* ARM YIELD: seams taken inside a bulk op, and with an acquire after them. */
static unsigned long ft_ba_seams_in_bulk __attribute__((unused));
# define FT_BA_GATE_NOTE()						\
	do {								\
		if (!ft_bulk_self_depth)				\
			ft_ba_seam_at_gate = ft_ba_seam_n;		\
	} while (0)
#else
# define FT_BA_GATE_NOTE()	do { } while (0)
#endif
/*
 * ☠ THE NON-OWNER-CLEAR TABLE IS KEYED BY ADDRESS, so a freed and recycled item
 * would leave a stale "held by tid T" entry and the next legitimate take of the
 * new node would be reported as a steal.  Forget the word when its item goes
 * back to the allocator.  Declared here because ft-helpers.h, where the frees
 * live, precedes the table.
 *
 * FT_DT_ARMED comes from ft-txn-rec-dbg.h (included at the top of this file so
 * the allocator's TU, which includes only this header, cannot see a different
 * gate than the table does -- see the arm's own comment).
 */
#if FT_DT_ARMED
void ft_dt_note_freed(const struct cds_ft_metadata *m);
void ft_dt_note_forgotten(const struct cds_ft_metadata *m);
# define FT_DT_NOTE_FREED(m_)	ft_dt_note_freed(m_)
# define FT_DT_NOTE_FORGOTTEN(m_)	ft_dt_note_forgotten(m_)
#else
# define FT_DT_NOTE_FREED(m_)	do { } while (0)
# define FT_DT_NOTE_FORGOTTEN(m_)	do { } while (0)
#endif

/*
 * INCARNATION MARKS FOR THE LOCK RING.  The ring is keyed by ADDRESS, so a
 * freed and recycled item makes ONE history out of TWO logical nodes; without a
 * boundary marker every replay across a reuse is ambiguous, and one such replay
 * was misread as a single node's story (2026-09-23).  Declared here because
 * ft-helpers.h -- where the FT's node alloc/free live -- is included long
 * before the ring itself; defined with it, non-static, the way ft_seam_check is.
 */
#if defined(FT_DEBUG_LOCK_LEAK) || defined(FT_ENABLE_TRACING)
/*
 * ☞ ALSO IN TRACE-ONLY MODE.  Gated on the RING alone, these marks vanished
 * from exactly the build that needs them: the captured history of a word shows
 * two threads taking it with no release between, and whether the bit vanished
 * because the ITEM WAS RECYCLED is precisely what a reuse boundary answers.
 */
void ft_ll_mark_reuse(const struct cds_ft_metadata *m);
void ft_ll_mark_freed(const struct cds_ft_metadata *m);
# define FT_LL_MARK_REUSE(m_)	ft_ll_mark_reuse(m_)
# define FT_LL_MARK_FREED(m_)	ft_ll_mark_freed(m_)
#else
# define FT_LL_MARK_REUSE(m_)	do { } while (0)
# define FT_LL_MARK_FREED(m_)	do { } while (0)
#endif

#ifdef FT_DEBUG_SEAM
/*
 * ☞ -DFT_DEBUG_SEAM: THE SEAM RULE, CHECKED WHERE IT IS RELIED ON.
 *
 * ft_writer_lock_gp_wait drops the writer lock of @h -- the trie this thread
 * actually HOLDS, which is not always the argument (graft / rekey pass @src_ft
 * while holding the destination's) -- for a grace period, and G5.25 lets FINE
 * point writers of @h through while it is down, on the rule that the seam sits
 * BETWEEN two consistent commits.  The part of that rule a point writer can trip
 * over structurally is the ordered list: every writer primitive assumes it is
 * circular through @h's own sentinel.  A bulk op that leaves a NULL or foreign
 * end on @h here broke the rule (the whole-trie graft_swap did:
 * inv_graft_swap_whole_seam_points), so name the seam and stop.  Two resolving
 * loads per end; the list cannot move under us (point writers of @h are held
 * off by the lock we still hold, the rest drained by the bulk gate).
 */
/*
 * ...AND THE ROOT THE POINT WRITERS THIS SEAM ADMITS WILL RE-DESCEND FROM.
 * Defined with the FT helpers (ft-mutation-helpers.h) and forward-declared
 * here the way ft_seam_check is: this header precedes ft_flag_to_metadata.
 */
/* NOT static: other units of the library reach this seam too. */
void ft_seam_check_root(struct cds_ft *h, const void *site);

static __attribute__((noinline, unused))
void ft_seam_check_list(struct cds_ft *h, const void *site)
{
	struct urcu_txn_list_node *s, *first, *last, *fp, *ln;

	if (!h || !h->ordered_list)
		return;
	s = &h->ord_sentinel.node;
	first = urcu_txn_list_unmark(urcu_txn_list_next_rcu(s));
	last = urcu_txn_list_unmark(urcu_txn_list_prev_rcu(s));
	if (first == s && last == s)
		return;				/* empty: self-looped */
	fp = first ? urcu_txn_list_unmark(urcu_txn_list_prev_rcu(first)) : NULL;
	ln = last ? urcu_txn_list_unmark(urcu_txn_list_next_rcu(last)) : NULL;
	if (caa_likely(first && last && fp == s && ln == s))
		return;
	fprintf(stderr, "FT SEAM RULE: trie %p dropped its writer lock for a grace "
		"period with its ordered list NOT circular (first %p ->prev %p, "
		"last %p ->next %p, sentinel %p) at %p -- a bulk op left a "
		"transient end visible to the point writers this seam admits\n",
		(void *) h, (void *) first, (void *) fp, (void *) last,
		(void *) ln, (void *) s, site);
	fflush(stderr);
	abort();
}
#endif

static inline
void ft_writer_lock_gp_wait(struct cds_ft *ft)
{
#ifdef FT_DEBUG_BULK_ACQ
	ft_ba_seam_n++;
	if (ft_bulk_self_depth)
		uatomic_inc(&ft_ba_seams_in_bulk);
#endif
#ifdef FT_DEBUG_SPLICE_SEAM
	uatomic_inc(&ft_ss_seam_all);
	if (ft_splice_window)
		uatomic_inc(&ft_ss_seam_in_window);
#endif
#ifdef FT_DEBUG_REMOVE_RETRY_CAP
	uint64_t ft_dbg_gp_t0 = ft_dbg_gp_clock();
#endif
	struct cds_ft *held = ft_wlock_held;

#ifdef FT_DEBUG_SEAM
	ft_seam_check_list(held, __builtin_return_address(0));
	ft_seam_check_root(held, __builtin_return_address(0));
#endif
	FT_BL_TALLY(ft_bl_seam_all);
	if (ft_bulk_lock_held != NULL) {
		FT_BL_TALLY(ft_bl_seam_under);
	} else {
		if (ft->exclusive)
			FT_BL_TALLY(ft_bl_seam_excl);
		if (ft_bulk_self_depth)
			FT_BL_TALLY(ft_bl_seam_selfdepth);

	}
	unsigned long depth = ft_wlock_depth;
	unsigned long own = 0;

	if (held) {
		own = ft_excl_owner_release(held);
		ft_wlock_held = NULL;
		ft_wlock_depth = 0;
#ifdef FT_DEBUG_WLOCK_HOLD
		ft_wlh_record(ft_wlh_hold, &ft_wlh_hold_n, &ft_wlh_hold_sum,
				&ft_wlh_hold_max, ft_wlh_now() - ft_wlh_acq_ns);
#endif
		(void) cds_fair_mutex_unlock(&held->writer_lock, &ft_wlock_waiter);
	}
	/*
	 * NEVER a grace period while holding a txn escalation fallback: a peer
	 * parked on that lock is an ONLINE, non-quiescent QSBR reader, so it is
	 * exactly what would stop this grace period -- and it is waiting for the
	 * lock this thread holds.  The two then wait on each other forever, which
	 * is the freeze already on record for the escalation path.  Measured 0 of
	 * these across every concurrent oracle (against ~11000 escalations in the
	 * heaviest), so this asserts a property the code HAS; it is here to name
	 * the cause the one time it is broken, because the wedge's own stacks
	 * blame the innocent.
	 */
	assert(!urcu_txn_in_fallback());
#ifdef FEATURE_FT_HOLD_TRACE
	ft_seam_check("ft_writer_lock_gp_wait");
#endif
	/*
	 * And NEVER a grace period inside this thread's own writer-scope read
	 * sections: drop them for the wait and re-take them after, as the lock
	 * above.  Only a scope that waits for a grace period WITHOUT a bulk gate
	 * holds one here (cds_ft_make_exclusive on a FINE trie); a bulk op's own
	 * scope never takes it (see ft_writer_lock_scope_enter).
	 */
	unsigned long nsec = ft_wscope_rcu_held;
	unsigned long isec;

	/*
	 * A scope that keeps a section never takes the FT-wide lock, so the
	 * re-acquire below (OFFLINE, ft_writer_lock_park) never runs with one.
	 */
	assert(!(held && nsec));
#ifdef FT_DEBUG_GP_KEEPS_SCOPE_SECTIONS
	nsec = 0;	/* red control: wait inside them */
#endif
	for (isec = 0; isec < nsec; isec++)
		ft_wscope_rcu_flavor->read_unlock();
#ifdef FT_DEBUG_BULK_WINDOW
	{
		uint64_t t0__ = ft_bw_now();

		ft->group->flavor->update_synchronize_rcu();
		ft_bw_gp_acc += ft_bw_now() - t0__;
		ft_bw_gp_acc_n++;
	}
#else
	ft->group->flavor->update_synchronize_rcu();
#endif
	for (isec = 0; isec < nsec; isec++)
		ft_wscope_rcu_flavor->read_lock();
	if (held) {
		/*
		 * OFFLINE for the re-acquire, or the drop above buys nothing when
		 * there is more than one writer: see ft_writer_lock_park.
		 */
		ft_writer_lock_park(held);
		ft_wlock_held = held;
		ft_wlock_depth = depth;
		ft_excl_owner_reclaim(held, own);
	}
#ifdef FT_DEBUG_REMOVE_RETRY_CAP
	ft_dbg_gp_ns += ft_dbg_gp_clock() - ft_dbg_gp_t0;
	ft_dbg_gp_calls++;
#endif
}

/*
 * VISITED-NODE WITNESS for the two-from-root coherence check.
 *
 * A traversal that runs while an in-trie move is in flight can be TORN: it can
 * resolve one edge on the old side of the move's commit and another on the new
 * side, and land somewhere that was never correct at any instant.  The witness is
 * how a second traversal detects that the first one was torn: each pass
 * accumulates the ADDRESSES of the nodes it visits, and the two passes are
 * compared.  Two SEQUENTIAL passes cannot both straddle the same commit (the
 * second starts after the first ended), so agreement means neither was torn.
 *
 * IDENTITY, not value: a move COWs its stitch points, so a moved subtree's
 * junction and top get FRESH addresses, and RCU cannot recycle a node inside a
 * read section -- which makes the witness immune to the away-and-back
 * (oscillating rekey) case that fools any comparison of keys or result nodes.
 *
 * Accumulated as an order-dependent 64-bit hash plus a visit count rather than a
 * stored set: constant space, no allocation on the read path, and order-sensitive
 * (two passes over an unchanged structure visit the same nodes in the same order).
 * A hash collision would accept a torn pass, at ~2^-64 per comparison.
 */
struct ft_visit_witness {
	uint64_t h;
	unsigned long n;
};

static inline
void ft_witness_init(struct ft_visit_witness *w)
{
	w->h = 0xcbf29ce484222325ULL;	/* FNV-1a offset basis */
	w->n = 0;
}

static inline
void ft_witness_visit(struct ft_visit_witness *w, const void *node)
{
	uint64_t x = (uint64_t) (uintptr_t) node;

	/*
	 * splitmix64 finalizer before folding: node addresses are arena-strided
	 * and share their low and high bits, so mixing first keeps the fold from
	 * cancelling structurally-similar addresses.
	 */
	x += 0x9e3779b97f4a7c15ULL;
	x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9ULL;
	x = (x ^ (x >> 27)) * 0x94d049bb133111ebULL;
	x ^= x >> 31;
	w->h = (w->h ^ x) * 0x100000001b3ULL;
	w->n++;
}

static inline
bool ft_witness_equal(const struct ft_visit_witness *a,
		const struct ft_visit_witness *b)
{
	return a->h == b->h && a->n == b->n;
}

/*
 * Is an in-trie MOVE in flight (or about to be)?  The reader-side mode read of
 * the move gate: 0 means fast path with no coherence check.  See
 * struct cds_ft::move_active for why one sample per read section is sound, and
 * why this is a plain load of a quiet word rather than a sequence counter.
 */
static inline
bool ft_move_active(const struct cds_ft *ft)
{
	/*
	 * ☞ ALWAYS-COHERENT TRIES ANSWER YES WITHOUT LOOKING.  See
	 * @rekey_always_coherent: on such a trie readers never leave the
	 * coherent path, so there is no mode to sample -- and, more to the
	 * point, no mode TRANSITION for the move gate to fence with a grace
	 * period.  That is the whole reason the flag exists.
	 *
	 * Immutable after cds_ft_create, so this is a predictable branch on a
	 * value the reader already has in cache beside @move_active.
	 */
	if (ft->rekey_always_coherent)
		return true;
	return CMM_LOAD_SHARED(ft->move_active) != 0;
}

/*
 * Is any BULK op live?  The writer-side tier-1 gate G5.5 turns on: false in
 * steady state, ONE trie-level load, and when true a FINE trie's point op
 * RE-TAKES the FT-wide writer lock instead of dropping it.  Its ONE consumer is
 * ft_writer_lock_scope_enter (G5.25), which is where the whole bulk-vs-point
 * exclusion lives.
 */
static inline
bool ft_bulk_active(const struct cds_ft *ft)
{
	return CMM_LOAD_SHARED(ft->bulk_state) != 0;
}

/*
 * ☞ DOES @bulk_state HAVE A CONSUMER ON THIS TRIE?
 *
 * The word has exactly one, and it is ft_writer_lock_scope_enter's FINE branch:
 * a FINE trie skips the FT-wide writer mutex UNLESS a bulk op is live.  Every
 * other writer mode already excludes the point writer without ever loading it:
 *
 *   COARSE         every writer scope takes the FT-wide mutex unconditionally,
 *                  so bulk and point writers already arbitrate on one word;
 *   EXTERNAL_SYNC  the caller excludes every writer by contract, and the scope
 *                  returns BEFORE the FINE branch;
 *   exclusive      single writer by contract, same early return.
 *
 * On those tries the word is published, FENCED WITH A GRACE PERIOD, and read by
 * nobody.  (MATHIEU, 2026-09-20: "in external sync mode and coarse trie, there
 * is no fine locking writers to contend with, so the bulk lock and fence is
 * useless.")  Every other reference to ft_bulk_active in the tree is inside
 * FT_ANC_LEDGER / FT_DEBUG_WIDEN_OWNER / FT_DEBUG_TXN_KIND instrumentation.
 *
 * ☠ AND THE FENCE IS THE EXPENSIVE HALF.  A FT_BULK_WRITER_ONLY op -- detach,
 * graft, graft_swap, merge_at -- publishes NO reader word, so @bulk_state is
 * the only thing that can make the 0->1 opener own a grace period.  On a coarse
 * or external-sync trie that is a full synchronize_rcu per bulk window, paid to
 * fence a word nothing reads.  A rekey (FT_BULK_COHERENT) is unaffected: it
 * owns its GP for @move_active, which is a READER word and is needed in every
 * mode -- readers stay concurrent under EXTERNAL_SYNC, which excludes writers
 * and nothing else.
 *
 * ☠ THE STORE ITSELF STAYS UNCONDITIONAL, and deliberately.  It is one relaxed
 * word, and keeping it means a trie that leaves @exclusive / @external_sync
 * mid-window (cds_ft_make_shared) degrades to "peers take the FT-wide lock",
 * which is the conservative side.  Only the fence is dropped, exactly as the
 * exit already drops it ("No grace period is needed on the way out").
 */
static inline
bool ft_bulk_state_has_consumer(const struct cds_ft *ft)
{
#ifdef FT_RED_BULK_FENCE_ALWAYS
	/*
	 * CONTROL, never a shipped configuration: fence unconditionally, which
	 * is the pre-2026-09-20 behaviour.  Here so the saving can be measured
	 * as an A/B in one tree rather than against a remembered number.
	 */
	(void) ft;
	return true;
#else
	return FT_BULK_WIDE_LOCK && ft->lock_fine && !ft->exclusive
		&& !ft->external_sync;
#endif
}

#ifdef FT_DEBUG_BULK_GATE_GP
/*
 * ARM YIELD for the fence above: a "saved" count with no "owned" beside it
 * cannot tell a real saving from a gate that never opens.
 */
__attribute__((weak)) unsigned long ft_bg_gp_owned, ft_bg_gp_saved,
	ft_bg_gp_coherent, ft_bg_gp_coherent_saved, ft_bg_reported;
static void ft_bg_report(void) __attribute__((destructor));
static void ft_bg_report(void)
{
	if (!ft_bg_gp_owned && !ft_bg_gp_saved && !ft_bg_gp_coherent
			&& !ft_bg_gp_coherent_saved)
		return;
	if (__atomic_fetch_add(&ft_bg_reported, 1, __ATOMIC_RELAXED))
		return;
	fprintf(stderr, "FT_BULK_GATE_GP  writer-word opens: fenced=%lu "
		"SAVED(no consumer)=%lu | reader-word (move_active) GPs=%lu "
		"SAVED(always-coherent)=%lu\n",
		ft_bg_gp_owned, ft_bg_gp_saved, ft_bg_gp_coherent,
		ft_bg_gp_coherent_saved);
}
# define FT_BG_TALLY(c)	__atomic_fetch_add(&(c), 1, __ATOMIC_RELAXED)
#else
# define FT_BG_TALLY(c)	do { } while (0)
#endif

/*
 * Which words a bulk op publishes.  Only a REKEY needs reader coherence; every
 * other bulk op publishes the writer word alone.
 */
enum ft_bulk_kind {
	FT_BULK_WRITER_ONLY,	/* detach / graft / graft_swap / merge_at */
	FT_BULK_COHERENT,	/* rekey: also publishes @move_active */
};

/*
 * ☠ AM I MYSELF A BULK OP?  A bulk op must not widen its own point-op helpers
 * against the gate it is holding.  Depth-counted, not a flag: bulk bodies nest
 * (a rekey's staged fallback re-enters), and a flag would be cleared by the
 * inner exit while the outer op is still live.
 */
extern __thread unsigned long ft_bulk_self_depth;

#ifdef FT_DEBUG_WIDEN_OWNER
extern unsigned long ft_wo_gate_enters;
#endif

/*
 * Enter the move mode gate: publish "expect a move" to readers and make sure
 * EVERY live reader has observed it before the caller mutates anything.
 *
 * The 0->1 mover owns the grace period; movers that arrive while that GP is in
 * flight PIGGYBACK it (wait, do not start another), and movers that arrive after
 * it completed proceed immediately -- @move_active has been set for at least one
 * full GP by then, so the "all live readers see it" invariant already holds.  A
 * burst of moves therefore costs ~one GP, not one per move.
 *
 * BLOCKING, and therefore NOT callable from an RCU read-side critical section:
 * the GP would wait for the caller's own section.  A mover takes its read lock
 * (for the descents it then does) AFTER this returns.
 */
/*
 * THE RECLAIM ROUTE.  Shared by the climb audit and by FT_ENABLE_TRACING's
 * item_reclaim event, so it must not live inside either one's guard.
 */
#ifdef FT_ENABLE_TRACING
#define FT_DBG_VIA_RCU		1	/* call_rcu callback: GP paid */
#define FT_DBG_VIA_EXCLUSIVE	2	/* exclusive trie: no readers by contract */
#define FT_DBG_VIA_UNPUB	3	/* "never published", freed immediately */
#define FT_DBG_VIA_DRAIN	4	/* reserve drain: no GP, no defer */
extern __thread unsigned long ft_dbg_free_via;
#endif



static inline
void ft_bulk_gate_enter_gp(struct cds_ft *ft, enum ft_bulk_kind kind)
{
	bool own_gp = false;

#ifdef FT_DEBUG_BULK_ELEV
	ft_bulk_gate_depth++;		/* see @ft_bulk_elev_self */
	uatomic_inc(&ft_bulk_gate_calls);
#endif

#ifdef FT_DEBUG_WIDEN_OWNER
	/*
	 * ☠ THE CONTROL FOR THE OTHER ZERO, and it must live under THIS flag.
	 * Without it "no acquire ever saw a live bulk op" cannot be told from
	 * "no bulk op ever ran" -- and the bulk-window instrument's own enter
	 * counter is behind a DIFFERENT -D, so reading it here would be a
	 * feature-flag-matrix miss reported as a measurement.
	 */
	uatomic_inc(&ft_wo_gate_enters);
#endif
	pthread_mutex_lock(&ft->move_gate_lock);
#ifdef FT_DEBUG_BULK_WINDOW
	/*
	 * The window measured is the WRITER word's -- @bulk_gate_nr > 0 --
	 * because that is the interval G5.5 makes point ops pay for.  Under
	 * @move_gate_lock, so it inherits the gate's serialization.  A
	 * PIGGYBACK is an enter that actually WAITED on an owner's GP, not
	 * merely any non-owner: an arrival after the GP completed waits for
	 * nothing and would overstate "a burst pays one GP".
	 */
	ft->bw_enters++;
	if (ft->bulk_gate_nr == 0)
		ft->bw_t_open = ft_bw_now();
	else if (ft->gate_gp_nr)
		ft->bw_piggyback++;
	if (ft->bulk_gate_nr + 1 > ft->bw_peak)
		ft->bw_peak = ft->bulk_gate_nr + 1;
#endif
	/*
	 * Publish each word this op needs that is not already published, and
	 * OWN A GRACE PERIOD if it published anything.  Both words need the
	 * same discipline -- publish, one full GP, only THEN mutate -- because
	 * a peer that sampled the word as clear before the store is still
	 * running on the old rule: a reader on the fast path for @move_active,
	 * a point op that has NOT taken the FT-wide writer lock for
	 * @bulk_active (G5.25 -- it sampled the gate clear at its writer scope
	 * and skipped the lock, so it is not excluded against this op yet).
	 */
	if (ft->bulk_gate_nr++ == 0) {
		/*
		 * Fence @bulk_state only where something LOADS it; see
		 * ft_bulk_state_has_consumer.  Note this decides the FENCE
		 * alone -- the store below is unconditional.
		 */
		if (ft_bulk_state_has_consumer(ft)) {
			own_gp = true;
			FT_BG_TALLY(ft_bg_gp_owned);
		} else
			FT_BG_TALLY(ft_bg_gp_saved);
	}
	if (kind == FT_BULK_COHERENT && ft->move_gate_nr++ == 0) {
		CMM_STORE_SHARED(ft->move_active, 1);
		/*
		 * A READER word, so every mode that has a reader to TRANSITION
		 * needs the fence -- and an ALWAYS-COHERENT trie has none.  Its
		 * readers were on the two-pass path before this store and will
		 * be after it; ft_move_active does not even load the word.  The
		 * grace period would be waiting for a mode change that is not
		 * happening.
		 *
		 * ☠ AND ON SUCH A TRIE IT IS NOT MERELY WASTE.  @external_sync
		 * means the APPLICATION's writer exclusion is held across this
		 * call, so this GP would wait while holding a lock the library
		 * cannot drop -- see @rekey_always_coherent.
		 *
		 * The store stays, for the reason the @bulk_state store does:
		 * one relaxed word, and it keeps the two paths reading alike.
		 */
		if (!ft->rekey_always_coherent) {
			own_gp = true;
			FT_BG_TALLY(ft_bg_gp_coherent);
		} else
			FT_BG_TALLY(ft_bg_gp_coherent_saved);
	}
	CMM_STORE_SHARED(ft->bulk_state, ft->bulk_gate_nr);
	if (own_gp) {
#ifdef FT_DEBUG_ES_GP
		{
			/* The GATE's own grace period, same question. */
			static struct ft_es_gp_site _es_gate = {
				.file = __FILE__, .line = __LINE__ };

			ft_es_gp_note(&_es_gate, ft);
		}
#endif
		ft->gate_gp_nr++;
		pthread_mutex_unlock(&ft->move_gate_lock);
		/*
		 * The barrier the whole gate exists for: peers that were
		 * already inside a critical section when the store landed may
		 * still believe they are in the old mode, so let them finish.
		 */
		assert(!urcu_txn_in_fallback());	/* see gp_wait */
		/*
		 * A bulk op enters its gate before any writer scope, so it
		 * holds none of the scope read sections this GP would wait on.
		 */
		assert(!ft_wscope_rcu_held);
#ifdef FEATURE_FT_HOLD_TRACE
		ft_seam_check("ft_bulk_gate_enter");
#endif
#ifdef FT_DEBUG_REMOVE_RETRY_CAP
		{
			uint64_t t0__ = ft_dbg_gp_clock();

			ft->group->flavor->update_synchronize_rcu();
			ft_dbg_gp_ns += ft_dbg_gp_clock() - t0__;
			ft_dbg_gp_calls++;
		}
#elif defined(FT_DEBUG_BULK_WINDOW)
		{
			uint64_t t0__ = ft_bw_now(), d__;

			ft->group->flavor->update_synchronize_rcu();
			d__ = ft_bw_now() - t0__;
			ft_bw_record(ft_bw_gp, &ft_bw_gp_n, &ft_bw_gp_sum,
					&ft_bw_gp_max, d__);
			ft_bw_gp_acc += d__;
			ft_bw_gp_acc_n++;
		}
#else
		ft->group->flavor->update_synchronize_rcu();
#endif
		pthread_mutex_lock(&ft->move_gate_lock);
		ft->gate_gp_nr--;
		pthread_cond_broadcast(&ft->move_gate_cond);
		pthread_mutex_unlock(&ft->move_gate_lock);
		FT_BA_GATE_NOTE();
		ft_bulk_self_depth++;
		return;
	}
	if (ft->gate_gp_nr) {
		/*
		 * PIGGYBACK, and go QUIESCENT while doing it.  Under a
		 * quiescent-state flavor (QSBR) a registered thread counts as
		 * being inside a read section until it reports otherwise, so a
		 * waiter that simply blocked here would be a reader the owner's
		 * grace period waits for -- while the owner waits for us and we
		 * wait for the owner.  That is a hard deadlock, and it is what
		 * happens (measured: the 16-writer oracle wedged) without these
		 * two calls.  Sound because a bulk op holds NO read section at
		 * this point: the gate is entered BEFORE the read lock, by
		 * contract.  A no-op on the memb / mb flavors.
		 *
		 * ☠ WAIT FOR ALL in-flight GPs, not for "the" one.  Our words
		 * are already published, but a peer may still be inside the
		 * very GP that makes one of them safe to act on, and a waiter
		 * cannot cheaply tell which GP covers which word.  Waiting for
		 * every one is conservative and never wrong; waiting for the
		 * wrong one would let us mutate against a peer that has not yet
		 * observed our word.
		 */
		ft->group->flavor->thread_offline();
		do {
			pthread_cond_wait(&ft->move_gate_cond,
					&ft->move_gate_lock);
		} while (ft->gate_gp_nr);
		pthread_mutex_unlock(&ft->move_gate_lock);
		ft->group->flavor->thread_online();
		FT_BA_GATE_NOTE();
		ft_bulk_self_depth++;
		return;
	}
	pthread_mutex_unlock(&ft->move_gate_lock);
	FT_BA_GATE_NOTE();
		ft_bulk_self_depth++;
}

/*
 * Take @bulk_lock: the BULK-vs-BULK serializer, held continuously across the
 * drain seams.  Parks OFFLINE, which is legal here and NOT at the writer
 * scope's entry: every waiter on this lock is another bulk op, and a bulk entry
 * forbids a read-section caller, so no reader-derived reference is exposed by
 * quiescing and no grace period is held up by a waiter.
 *
 * Reentrant on the same trie (bulk bodies nest).  An EXCLUSIVE trie skips it
 * for the same reason it skips the writer lock: no concurrent writer exists.
 */

static inline
void ft_bulk_lock_enter(struct cds_ft *ft)
{
	const struct rcu_flavor_struct *flavor = ft->group->flavor;

	if (ft->exclusive || ft->external_sync)
		return;
	if (ft_bulk_lock_held == ft) {
		ft_bulk_lock_depth++;
		FT_BL_TALLY(ft_bl_nested);
		return;
	}
	if (caa_unlikely(ft_bulk_lock_held != NULL)) {
		/*
		 * Two bulk locks at once: a bulk op gates ONE live trie (a
		 * cross-trie op's source is exclusive and skips this), so a
		 * second live trie here would be a lock-order inversion waiting
		 * to deadlock.  Same disposition as the FT-wide lock's nest
		 * check above: refuse loudly rather than leak a lock in a
		 * release build, where an assert would be compiled out.
		 */
		fprintf(stderr, "[Fatal] Fractal Trie: bulk lock already held "
				"on %p while entering %p\n",
			(void *) ft_bulk_lock_held, (void *) ft);
		abort();
	}
	flavor->thread_offline();
	cds_fair_mutex_lock(&ft->bulk_lock, &ft_bulk_lock_waiter);
	flavor->thread_online();
	ft_bulk_lock_held = ft;
	ft_bulk_lock_depth = 1;
	FT_BL_TALLY(ft_bl_takes);
}

static inline
void ft_bulk_lock_exit(struct cds_ft *ft)
{
	/*
	 * Keys off HOLDER IDENTITY, never off @ft->exclusive -- exactly as
	 * ft_writer_lock_scope_exit does.  Re-reading @exclusive here would
	 * LEAK the lock if it flipped between enter and exit: the op took the
	 * lock on a live trie and would then decline to release it.
	 */
	if (ft_bulk_lock_held != ft)
		return;
	if (--ft_bulk_lock_depth)
		return;
	ft_bulk_lock_held = NULL;
	(void) cds_fair_mutex_unlock(&ft->bulk_lock, &ft_bulk_lock_waiter);
}

/*
 * Enter the bulk window: the GATE first, then the BULK-vs-BULK lock.
 *
 * ORDER IS THE POINT.  The gate is refcounted and owns the grace period, so
 * entering it first keeps the piggyback that makes "a burst of concurrent moves
 * pays about ONE grace period between them, not one each" true -- peers share
 * the GP inside the gate and only then queue on @bulk_lock.  Taking the lock
 * first would serialize the GPs and make the burst pay one EACH.
 */
static inline
void ft_bulk_gate_enter(struct cds_ft *ft, enum ft_bulk_kind kind)
{
	ft_bulk_gate_enter_gp(ft, kind);
	ft_bulk_lock_enter(ft);
}

/*
 * The rekey spelling, unchanged for its callers: a move needs BOTH words.
 */
static inline
void ft_move_gate_enter(struct cds_ft *ft)
{
	ft_bulk_gate_enter(ft, FT_BULK_COHERENT);
}

/*
 * Leave the move gate; the LAST mover out returns readers to the fast path.  No
 * grace period is needed on the way out: a reader that still sees @move_active
 * set merely runs the coherent path once more, which is never wrong, only slower.
 */
/*
 * THE GATE'S EXIT PREMISE, MADE CHECKABLE.
 *
 * ft_bulk_gate_exit clears @move_active with NO grace period, and the reason it
 * is allowed to is stated below: a reader that still sees the word set merely
 * runs the conservative rule once more, and a reader that sees it CLEAR starts
 * after the structure is final.  That second half is a claim about the MOVER --
 * that it has published everything before it clears -- and nothing checked it.
 *
 * A flip-txn that is still open is exactly "something not published yet", so
 * count them per thread and assert none is left at the last bracket.  The
 * counter is also REPORTED (max seen), because a silent zero here would be
 * indistinguishable between "the premise holds" and "this TU's copy of the
 * counter was never incremented" -- the per-TU wrong zero this codebase has
 * been bitten by more than once.
 */
#if defined(DEBUG_RCU) || defined(CONFIG_RCU_DEBUG) || defined(FT_DEBUG_GATE_PENDING)
static __thread unsigned long ft_flip_txn_open_nr;
static unsigned long ft_flip_txn_open_max;
# define FT_TXN_OPEN_INC()						\
	do {								\
		if (++ft_flip_txn_open_nr >				\
				uatomic_read(&ft_flip_txn_open_max))	\
			uatomic_set(&ft_flip_txn_open_max,		\
				ft_flip_txn_open_nr);			\
	} while (0)
# define FT_TXN_OPEN_DEC()	(ft_flip_txn_open_nr--)
static __attribute__((destructor)) void ft_txn_open_report(void)
{
	if (uatomic_read(&ft_flip_txn_open_max))
		fprintf(stderr, "FT GATE PENDING: deepest open flip-txn nesting seen %lu (0 left at every gate exit)\n",
			uatomic_read(&ft_flip_txn_open_max));
}
#else
# define FT_TXN_OPEN_INC()	do { } while (0)
# define FT_TXN_OPEN_DEC()	do { } while (0)
#endif

static inline
void ft_bulk_gate_exit(struct cds_ft *ft, enum ft_bulk_kind kind)
{
	/* Reverse of the acquire order: @bulk_lock is INSIDE the gate. */
	ft_bulk_lock_exit(ft);
	ft_bulk_self_depth--;
#ifdef FT_DEBUG_BULK_ELEV
	ft_bulk_gate_depth--;		/* see @ft_bulk_elev_self */
#endif
	pthread_mutex_lock(&ft->move_gate_lock);
	/*
	 * Each word clears on ITS OWN refcount reaching zero.  No grace period
	 * on the way out: a peer that still sees a word set merely runs the
	 * more conservative rule once more, which is never wrong, only slower.
	 */
	if (kind == FT_BULK_COHERENT && --ft->move_gate_nr == 0) {
#if defined(DEBUG_RCU) || defined(CONFIG_RCU_DEBUG) || defined(FT_DEBUG_GATE_PENDING)
		/* Nothing of this mover's may still be unpublished here. */
		urcu_assert_debug(!ft_flip_txn_open_nr);
#endif
		CMM_STORE_SHARED(ft->move_active, 0);
	}
	if (--ft->bulk_gate_nr) {
		CMM_STORE_SHARED(ft->bulk_state, ft->bulk_gate_nr);
	} else {
		CMM_STORE_SHARED(ft->bulk_state, 0);
#ifdef FT_DEBUG_BULK_WINDOW
		{
			uint64_t now__ = ft_bw_now();

			ft_bw_record(ft_bw_win, &ft_bw_win_n, &ft_bw_win_sum,
					&ft_bw_win_max, now__ - ft->bw_t_open);
			uatomic_inc(&ft_bw_opens);
			if (ft->bw_t_close)
				ft_bw_record(ft_bw_gap, &ft_bw_gap_n,
						&ft_bw_gap_sum, &ft_bw_gap_max,
						ft->bw_t_open - ft->bw_t_close);
			uatomic_add(&ft_bw_enters, ft->bw_enters);
			uatomic_add(&ft_bw_piggyback, ft->bw_piggyback);
			if (ft->bw_peak > uatomic_load(&ft_bw_peak, CMM_RELAXED))
				uatomic_store(&ft_bw_peak, ft->bw_peak,
						CMM_RELAXED);
			ft->bw_enters = 0;
			ft->bw_piggyback = 0;
			ft->bw_peak = 0;
			if (!ft_bw_first_open)
				ft_bw_first_open = ft->bw_t_open;
			ft->bw_t_close = now__;
			ft_bw_last_close = now__;
		}
#endif
	}
	pthread_mutex_unlock(&ft->move_gate_lock);
}

static inline
void ft_move_gate_exit(struct cds_ft *ft)
{
	ft_bulk_gate_exit(ft, FT_BULK_COHERENT);
}

struct ft_bulk_gate_scope {
	struct cds_ft *ft;
	enum ft_bulk_kind kind;
};

static inline
void ft_bulk_gate_scope_end(struct ft_bulk_gate_scope *s)
{
	ft_bulk_gate_exit(s->ft, s->kind);
}

/*
 * Scoped, because the public bulk entries reject arguments with EARLY RETURNS
 * and a bracket that only covered the success path would leak the gate --
 * leaving @bulk_active set for the life of the trie and every point op
 * serialized on the FT-wide writer lock forever.
 *
 * PLACE IT AFTER the cheap argument checks and BEFORE any lock or read section:
 * the gate BLOCKS on a grace period, so a caller already inside an RCU read
 * section would have the grace period wait on itself, and a caller already
 * holding a node lock would break the seam rule (internal.h: no node lock may
 * be held across a GP).
 */
#define CDS_FT_SCOPED_BULK_GATE(ft_, kind_)				\
	struct ft_bulk_gate_scope ft_bulk_gate_scope__			\
		__attribute__((cleanup(ft_bulk_gate_scope_end))) =	\
		{ (ft_), (kind_) };					\
	ft_bulk_gate_enter((ft_), (kind_))

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

/*
 * This thread's writer nesting, for the FINE path below: @ft_excl_self_ft /
 * @ft_excl_self_depth mirror ft_wlock_held / ft_wlock_depth (one trie, the
 * live dst of a cross-trie op), and @ft_excl_self_any counts writer scopes on
 * ANY trie.  The reader check aborts only when this thread's scopes are ALL
 * accounted for on the trie being read -- so an untracked second trie costs a
 * missed report, never a false one.
 */
static __thread struct cds_ft *ft_excl_self_ft;
static __thread unsigned long ft_excl_self_depth;
static __thread unsigned long ft_excl_self_any;

static inline
unsigned long ft_excl_self_writers(const struct cds_ft *ft)
{
	return ft_excl_self_ft == ft ? ft_excl_self_depth : 0;
}

static inline
bool ft_excl_writer_enter(struct cds_ft *ft)
{
	unsigned long self = (unsigned long) pthread_self();
	unsigned long prev, nr;
	bool rcu;

	/*
	 * MW lock-mode: take the FT-wide writer lock at the outermost scope
	 * before the discipline checks, so they run single-writer.  No-op on an
	 * optimistic trie.
	 */
	rcu = ft_writer_lock_scope_enter(ft);
	ft_excl_self_any++;
	if (ft_excl_self_ft == ft) {
		ft_excl_self_depth++;
	} else if (!ft_excl_self_ft) {
		ft_excl_self_ft = ft;
		ft_excl_self_depth = 1;
	}
	if (ft->lock_fine && !ft->exclusive) {
		/*
		 * CONCURRENT + FINE is the ONLY combination where writer/writer
		 * overlap is contract-conforming: a FINE trie skips the FT-wide
		 * writer mutex, so writers on disjoint subtrees proceed in
		 * parallel and only structural collisions serialize -- the POINT
		 * of fine locking.  The single-owner CAS aborted on that, so
		 * every concurrent oracle tripped this validator by construction
		 * and it could only ever run single-threaded.  Count writers
		 * instead; the reader check below uses the count.
		 *
		 * The other combinations KEEP the owner CAS, and it is the whole
		 * point there: an EXCLUSIVE trie is one whose caller promised to
		 * serialize access, so overlap is exactly the user error this
		 * validator exists to catch (and an exclusive trie is FINE by
		 * default, so gating on lock_fine alone would have disabled the
		 * check where it matters most).  A COARSE trie serializes under
		 * the FT-wide mutex, so its owner word is accurate anyway.
		 */
		uatomic_add(&ft->excl_nr_writers, 1);
	} else {
		prev = uatomic_cmpxchg(&ft->excl_owner, 0, self);
		if (prev != 0) {
			if (prev == self) {
				/* Reentry from the same thread (e.g. graft_swap
				 * delegating to graft). */
				ft->excl_writer_depth++;
				return rcu;
			}
			ft_excl_abort("cds_ft=%p: writer conflict -- owner 0x%lx, entering thread 0x%lx\n",
				(void *) ft, prev, self);
		}
		ft->excl_writer_depth = 1;
	}
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
	return rcu;
}

static inline
void ft_excl_writer_exit(struct cds_ft *ft)
{
	if (ft->lock_fine && !ft->exclusive) {
		uatomic_sub(&ft->excl_nr_writers, 1);
	} else if (--ft->excl_writer_depth == 0) {
		uatomic_store(&ft->excl_owner, 0, CMM_RELEASE);
	}
	ft_excl_self_any--;
	if (ft_excl_self_ft == ft && --ft_excl_self_depth == 0)
		ft_excl_self_ft = NULL;
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
	if (ft->lock_fine && !ft->exclusive) {
		unsigned long mine = ft_excl_self_writers(ft);
		unsigned long nw = uatomic_load(&ft->excl_nr_writers, CMM_ACQUIRE);

		/*
		 * A FINE trie claims no owner, so ask the count.  Abort only
		 * when the excess provably belongs to ANOTHER thread: this
		 * thread's own scopes must all be accounted for on THIS trie
		 * (@ft_excl_self_any == @mine), else an untracked second trie
		 * would make us report our own write as a peer's.
		 */
		if (nw > mine && ft_excl_self_any == mine)
			ft_excl_abort("cds_ft=%p: reader 0x%lx entering with %lu concurrent writer(s) (concurrent without RCU read-side lock)\n",
				(void *) ft, (unsigned long) pthread_self(),
				nw - mine);
	}
	return scope;
}

static inline
void ft_excl_reader_exit_scope(const struct ft_excl_reader_scope *scope)
{
	if (scope->claimed)
		uatomic_sub(&scope->ft->excl_nr_readers, 1);
}

#else /* !FEATURE_FT_EXCL_VALIDATE */

static inline bool ft_excl_writer_enter(struct cds_ft *ft)
{
	/* MW lock-mode FT-wide lock; no-op on an optimistic trie. */
	return ft_writer_lock_scope_enter(ft);
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

/*
 * One writer scope: the trie, and whether its enter took a read-side section of
 * the trie's flavor (ft_writer_lock_scope_enter's FINE arm).  Carried in the
 * scope object rather than re-derived at exit, so the exit drops exactly what
 * this enter took, whatever the gate reads by then.
 */
struct ft_writer_scope {
	struct cds_ft *ft;
	bool rcu;
};

static inline
struct ft_writer_scope ft_excl_writer_scope_enter(struct cds_ft *ft)
{
	struct ft_writer_scope scope = { .ft = ft };

	scope.rcu = ft_excl_writer_enter(ft);
	return scope;
}

static inline
void ft_excl_writer_scope_exit(struct ft_writer_scope *scope)
{
	struct cds_ft *ft = scope->ft;

#ifdef FEATURE_FT_VERIFY_AT_MUTATION
	/*
	 * Verify before releasing the writer claim so a concurrent
	 * writer cannot start mutating while we walk the trie.  Runs under
	 * the FT-wide lock (released last, below).
	 */
	ft_writer_scope_verify(ft);
#endif
	ft_excl_writer_exit(ft);
	/*
	 * MW lock-mode: release the FT-wide writer lock at the outermost
	 * scope, after the discipline checks / verify walked the trie under
	 * it.  No-op on an optimistic trie.
	 */
	ft_writer_lock_scope_exit(ft);
	/* Last out: the section the enter took first. */
	if (scope->rcu) {
		ft_wscope_rcu_held--;
		ft->group->flavor->read_unlock();
	}
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
	struct ft_writer_scope CDS_FT_CAT_(_ft_excl_scope_, __COUNTER__)	\
		__attribute__((unused,					\
			cleanup(ft_excl_writer_scope_exit))) =		\
		ft_excl_writer_scope_enter(ft)

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
	if (caa_unlikely(a != b && !a->exclusive && !b->exclusive)) {
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
	struct cds_ft_alloc_range *range;

	/* A node, not a parked proxy -- see cds_ft_item_to_metadata. */
	assert(((uintptr_t) p & FT_PARENT_TAG_MASK) != FT_PARENT_TAG_MASK);
	range = cds_ft_item_to_range(p);
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

#ifndef FEATURE_FT_COMPRESS
/*
 * Add @n single-child branch nodes (CDS_FT_ALLOC_KIND_NODE, @item_len_order) to
 * @r's branch excess (see struct cds_ft_alloc_reserve).  One order per reserve;
 * same contract as cds_ft_alloc_reserve_add, asserted against
 * CDS_FT_ALLOC_RESERVE_BRANCH_CAP.
 */
__attribute__((visibility("hidden")))
int cds_ft_alloc_reserve_add_branch(struct cds_ft *ft,
		struct cds_ft_alloc_reserve *r, size_t item_len_order, bool bitmap,
		unsigned int n);
#endif

/*
 * Make @r the active reserve for @ft in THIS THREAD's reserve set.  A bulk op
 * allocating into several tries (e.g. merge same-trie rekey: dst + the transient
 * detach product) activates the SAME @r on each; the reserve is thread-local, so
 * a concurrent bulk op on the same live dst owns its own reserve and does not
 * collide -- which is what makes it safe with no FT-wide mutex (§11).
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
 * ft_rekey_one_decide - move @src_key's subtree to @dst_key as ONE decide.
 *
 * The atomic rekey writer, defined in fractal-trie.c because it composes detach,
 * graft and merge and so must sit below all three; declared here so the public
 * rekey entry points in ft-merge.h can dispatch to it.  Returns 0 (committed
 * atomically), -EINVAL (shape outside its cut -- fall back to the staged
 * writer), -EEXIST (@require_empty and the destination holds content), -ENOMEM
 * or -ENOTSUP.  Caller holds the move gate, not a read section; see the
 * definition for the full contract.
 */
__attribute__((visibility("hidden")))
int ft_rekey_one_decide(struct cds_ft *ft,
		const uint8_t *src_key, size_t src_len,
		const uint8_t *dst_key, size_t dst_len, bool require_empty);

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
	/*
	 * The ACQUIRE SEAM: a writer loads a pointer / child count BEFORE it
	 * takes the node's lock and RE-READS it after, and every decision made
	 * from the first sample (which node to route to, how big the copy must
	 * be) is only as good as the second.  Widening exactly that gap turns
	 * the rarest of these races into a reliable reproducer -- see
	 * ft_delay_seam().  Separate from FT_DELAY_WRITER because a
	 * blanket writer delay perturbs the whole op and reproduces nothing in
	 * particular.
	 */
	FT_DELAY_ACQUIRE = (1 << 3),
};

/*
 * WHICH seam delays.  ☠ ARMING THEM ALL AT ONCE SUPPRESSES THE VERY RACES IT
 * IS MEANT TO EXPOSE: every thread passes through many acquire sites, so a
 * delay armed everywhere slows all the racers by a similar total and the SKEW
 * -- the only thing that widens a window -- averages out.  MEASURED on
 * inv_concurrent_remove_all_nolist with the NODE_INDEX_NULL refusal ablated:
 * no delay 3 SIGSEGVs / 100 runs, every site armed at 2us/50% ZERO / 100.
 * So arm ONE site (FT_DELAY_SITES=recompact), and treat a quieter run under
 * injection as evidence the injection is wrong, never as evidence of a fix.
 */
/*
 * ☠ FT_DELAY_SITE_ACQUIRE DOES NOT OPEN A DERIVE-BEFORE-LOCK WINDOW -- the
 * "averages out" rule above, measured on the class it looks made for.  A
 * stale-plan lost insert (the sole-entry freeze, 1d74625e) reproduces at
 * ~0.4% of rows when the remover is PREEMPTED between its unheld read and its
 * first take (taskset 2 cpus + 2 spinners).  With the fix disabled
 * (-DFT_RED_NO_CCF_PLAN_CHECK) and this site sleeping instead
 * (FT_DELAY_SPIN=0 FT_DELAY_US=50 FT_DELAY_PCT=10, 122M firings): 0 losses
 * in 2,000 rows where ~8 were due.  It sleeps the PEER at its own takes too,
 * so the peer rarely finishes an append inside the gap.  Use CPU pinning
 * with competing spinners for that class.  (Also: without FT_DELAY_SPIN=0 a
 * site SPINS ~200 pauses and FT_DELAY_US is ignored; and a sleep inside a
 * point op stalls grace periods, so bound the rows per process or the
 * call_rcu backlog outgrows a memory cage.)
 */
enum ft_delay_site {
	FT_DELAY_SITE_ACQUIRE   = (1 << 0),	/* the 3 shared lock helpers */
	FT_DELAY_SITE_RECOMPACT = (1 << 1),	/* post-lock child-count re-read */
	FT_DELAY_SITE_INSERT    = (1 << 2),	/* in-place reserve's hoisted lock */
	/*
	 * POST-LOCK: after an acquire, before the re-read it protects.  Cannot
	 * widen a peer's window (the acquire excludes them) -- kept only to
	 * study an op's OWN ordering, never to reproduce a lost race.
	 */
	FT_DELAY_SITE_POSTLOCK  = (1 << 3),
	/*
	 * THE STALE-EXPECTED-OLD SEAM, for the class where an acquire does NOT
	 * exclude the writer: a STATE-word record samples the word when it is
	 * built and its settle writes that sample back -- blind, if the record
	 * is SW.  A peer that TAKES the lock between the two has its LOCK bit
	 * erased by that settle and asserts at its own release
	 * (ft_meta_lock_release: s & FT_STATE_LOCK).  Widening THIS gap is what
	 * a POSTLOCK delay cannot do (its comment assumes the acquire excludes
	 * everyone, which is exactly the assumption under test).
	 *
	 *   STATEREC  after a state-word record is filed, before its commit.
	 *   HELD      after a lock take succeeds, before the op's own records,
	 *             so a peer's already-built record settles inside the hold.
	 */
	FT_DELAY_SITE_STATEREC  = (1 << 4),
	FT_DELAY_SITE_HELD      = (1 << 5),
	/*
	 * THE ACQUIRE LANE'S PARK.  Armed with -DFT_RED_ACQ_LANE_OFFLINE (the
	 * pre-2026-09-23 behaviour, which parked the point op OFFLINE) this
	 * widens the window in which the op is QUIESCENT mid-operation, so a
	 * peer's call_rcu-deferred frees can reclaim the very @node its retry
	 * re-derives from.  Armed WITHOUT it, the same delay lengthens an
	 * ONLINE park and nothing is exposed -- which is exactly the A/B.
	 * Use the usleep form (FT_DELAY_SPIN=0 FT_DELAY_US=...) with a small
	 * FT_DELAY_PCT: a grace period has to fit inside the park.
	 */
	FT_DELAY_SITE_LANE      = (1 << 6),
	FT_DELAY_SITE_ALL       = 0x7f,
};

#ifdef FT_DELAY_INJECT
extern enum ft_delay_mode ft_delay_mode;
extern unsigned int ft_delay_us;
extern unsigned int ft_delay_pct;
extern unsigned int ft_delay_sites;
extern unsigned int ft_delay_spin;

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

/*
 * Per-thread xorshift, because the delay must be decided WITHOUT serializing
 * the threads it is meant to skew: glibc rand() takes a global lock, so using
 * it here would order the very racers whose interleaving is under test (and
 * FT_DELAY_RANDOM above has that flaw).  Seeded off the TLS slot's own address.
 */
static inline
unsigned int ft_delay_rand(void)
{
	static __thread unsigned int seed;

	if (caa_unlikely(!seed))
		seed = (unsigned int) (uintptr_t) &seed ^ 0x9e3779b9u;
	seed ^= seed << 13;
	seed ^= seed >> 17;
	seed ^= seed << 5;
	return seed;
}

/*
 * Call between a writer's PRE-LOCK sample and the POST-LOCK re-read that is
 * supposed to supersede it.
 *
 * ☠ DELAY ONLY SOME OF THE CALLERS.  Delaying every one slows all the racers
 * equally and reproduces nothing: the peer that has to slip INTO the gap is
 * sitting in the same usleep.  What exposes the race is SKEW -- one thread
 * pauses at the seam while another runs through at full speed -- so each call
 * tosses its own coin (FT_DELAY_PCT, default 50).
 */
static unsigned long ft_delay_fired[8];
static const char *const ft_delay_site_name[8] = {
	"acquire", "recompact", "insert", "postlock", "staterec", "held",
	"lane", "?7"
};
static __attribute__((destructor)) void ft_delay_fired_report(void)
{
	unsigned int i;

	for (i = 0; i < 8; i++)
		if (uatomic_read(&ft_delay_fired[i]))
			fprintf(stderr, "FT DELAY: site %s fired %lu times\n",
				ft_delay_site_name[i],
				uatomic_read(&ft_delay_fired[i]));
}

static inline
void ft_delay_seam(enum ft_delay_site site)
{
	if (caa_likely(!(ft_delay_mode & (FT_DELAY_ACQUIRE | FT_DELAY_RANDOM))))
		return;
	if (!(ft_delay_sites & (unsigned int) site))
		return;
	if (ft_delay_pct < 100 && (ft_delay_rand() % 100u) >= ft_delay_pct)
		return;
	/*
	 * ☠ COUNT THE FIRINGS.  A quiet run under injection is only evidence
	 * about the code if the injection HAPPENED; without this, a site that
	 * is never reached and a race that does not exist read identically --
	 * and this file already warns that a quieter run under injection is
	 * evidence the injection is wrong.
	 */
	uatomic_inc(&ft_delay_fired[__builtin_ctz((unsigned int) site) & 7]);
	/*
	 * ☠ A SLEEP IS THE WRONG SHAPE FOR A NANOSECOND WINDOW.  usleep() cannot
	 * resolve below the scheduler's granularity (~50us however small the
	 * argument) AND it parks the thread, so the racer that was supposed to
	 * be merely NUDGED is descheduled and the interleaving under test is
	 * destroyed rather than widened -- MEASURED: with the NODE_INDEX_NULL
	 * refusal ablated, no delay reproduces 1-3 SIGSEGVs per 100 runs while
	 * usleep at ANY single site, 5us, reproduces ZERO.
	 *
	 * So the default is a SPIN of @ft_delay_spin pause instructions: it stays
	 * on the CPU, costs tens of nanoseconds to microseconds, and is the only
	 * form that can widen a window of this size.  FT_DELAY_US is honoured
	 * only when explicitly asked for (FT_DELAY_SPIN=0), for the rare race
	 * whose window really is a scheduling event.
	 */
	if (caa_likely(ft_delay_spin)) {
		unsigned int i;

		for (i = 0; i < ft_delay_spin; i++)
			caa_cpu_relax();
		return;
	}
	usleep(ft_delay_us);
}
#else
static inline void ft_delay_writer(void) { }
static inline void ft_delay_reader(void) { }
static inline void ft_delay_seam(enum ft_delay_site site) { (void) site; }
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
/*
 * The same check for the point ops whose CONTRACT puts the section on the
 * CALLER (a cached iterator's node must stay live): asked of the caller's state
 * recorded when the writer scope took its own section, which would otherwise
 * satisfy read_ongoing() on the caller's behalf.
 */
# define CDS_FT_ASSERT_CALLER_RCU_READ_LOCKED(ft)                              \
	do {                                                                   \
		if (caa_unlikely(!(ft_wscope_rcu_held ?                        \
				ft_wscope_caller_reading :                     \
				(ft)->group->flavor->read_ongoing()))) {       \
			fprintf(stderr, "[Fatal] Fractal Trie API violation: " \
					"RCU read-side lock not held by the "  \
					"caller at %s:%d\n", __FILE__,         \
					__LINE__);                             \
			abort();                                               \
		}                                                              \
	} while (0)
#else
# define CDS_FT_ASSERT_RCU_READ_LOCKED(ft) do { } while (0)
# define CDS_FT_ASSERT_CALLER_RCU_READ_LOCKED(ft) do { } while (0)
#endif

/*
 * The CONVERSE: entries that BLOCK on a grace period must NOT be called from
 * inside an RCU read-side critical section -- the grace period would wait on
 * the caller's own section.  ft_rekey_one_decide is one: its caller holds the
 * move gate, which waits a grace period, and its retry loop lets the escalation
 * park take the thread OFFLINE (urcu_txn_set_park_quiescent), which silently
 * quiesces a caller that believes it is pinned.
 *
 * ☠ ONLY MEANINGFUL FOR FLAVORS WHOSE read_ongoing() COUNTS SECTIONS (memb,
 * mb, bp).  Under QSBR it is `urcu_qsbr_reader.ctr` -- ONLINE-ness -- which is
 * non-zero for any registered thread whether or not it is in a section, so this
 * would fire on every correct call.  Do not enable
 * URCU_FRACTAL_TRIE_DEBUG_LOCKING on a QSBR build.
 */
#ifdef URCU_FRACTAL_TRIE_DEBUG_LOCKING
# define CDS_FT_ASSERT_RCU_NOT_READ_LOCKED(ft)                                 \
	do {                                                                   \
		if (caa_unlikely((ft)->group->flavor->read_ongoing())) {       \
			fprintf(stderr, "[Fatal] Fractal Trie API violation: " \
					"called from inside an RCU read-side " \
					"critical section at %s:%d -- this "   \
					"entry waits for a grace period\n",    \
					__FILE__, __LINE__);                   \
			abort();                                               \
		}                                                              \
	} while (0)
#else
# define CDS_FT_ASSERT_RCU_NOT_READ_LOCKED(ft) do { } while (0)
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
	 * Opt this group's tries into the in-trie MOVE ops (rekey).  DEFAULT
	 * FALSE -- cds_ft_rekey_graft / cds_ft_rekey_merge answer
	 * CDS_FT_STATUS_NOT_SUPPORTED unless it is set.  See
	 * cds_ft_group_attr_set_rekey.
	 */
	bool rekey_set;
	/*
	 * Maintain the per-node order-statistics key counts (nr_keys) that back
	 * cds_ft_count_keys / _prefix, cds_ft_lookup_nth / _last, and
	 * cds_ft_iter_skip_forward / _reverse.  Default OFF: when disabled the
	 * library keeps no per-node count and those queries fall back to
	 * enumeration (count) or first/last + next/prev iteration (select /
	 * skip).  See cds_ft_group_attr_set_rank_stats.
	 */
	bool rank_stats_set;
	/*
	 * Distinguishes "the caller never chose a strategy" from "the caller
	 * explicitly chose a strategy", which calloc-zero alone cannot: unset
	 * resolves to the DLM default.
	 */
	bool writer_strategy_set;
	enum cds_ft_numa_policy numa_policy;	/* See cds_ft_group_attr_set_numa_policy. */
	enum cds_ft_optimize optimize;		/* See cds_ft_group_attr_set_optimize. */
	/*
	 * Structural-writer concurrency strategy (MW lock-escalation model).
	 * Meaningful only when @writer_strategy_set; otherwise the group takes
	 * the CDS_FT_WRITER_LOCK_FINE default.  See
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

#ifdef FT_DEBUG_TOMBSTONE_AUDIT
extern unsigned long ft_unpub_free_calls;
#endif

#endif /* _URCU_FT_INTERNAL_H */
