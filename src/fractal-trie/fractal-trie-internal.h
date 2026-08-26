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
#include <urcu/rculfhash.h>
#include <urcu/arch.h>
#include <urcu/assert.h>	/* urcu_assert_debug: the engine self-checks' arm */
#include <urcu/call-rcu.h>
#include <urcu/uatomic.h>
#include <urcu/fractal-trie.h>	/* enum cds_ft_numa_policy, cds_ft_optimize */
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
 * DLM lock coarseness (doc/design/ft-dlm-lock-coarseness.md).  Lock levels sit
 * at key-byte depths 0, 1, 2, 4, 8, ... -- dense near the root, sparse deeper.
 * A structural writer anchors its lock-set on an ancestor at one of these
 * levels instead of on every node it mutates, so members sharing a level
 * collapse onto ONE lock word.  Key BYTES, not node hops: a lock's reach is
 * bounded by the key space below it, and a compressed node advances several
 * byte levels at the cost of one lock.
 *
 * Levels 0,1,2,4,...,256 for FT_MAX_KEY_LEN 256, so a per-descent table indexed
 * by ft_lock_level_index() needs FT_LOCK_LEVEL_MAX slots.
 */
#define FT_LOCK_LEVEL_MAX	10

/*
 * The deepest lock level at or above @depth: all but the top set bit cleared.
 */
static inline
unsigned int ft_lock_level(unsigned int depth)
{
	if (!depth)
		return 0;
	return 1U << ((sizeof(unsigned int) * CHAR_BIT - 1) -
			(unsigned int) __builtin_clz(depth));
}

/*
 * @depth's slot in a lock-level table: level 0 at index 0, level (1 << (i - 1))
 * at index i.  Every depth sharing a level shares a slot.
 */
static inline
unsigned int ft_lock_level_index(unsigned int depth)
{
	if (!depth)
		return 0;
	return (sizeof(unsigned int) * CHAR_BIT) -
		(unsigned int) __builtin_clz(depth);
}

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
 * fast path with -DFEATURE_FT_INSERT_IN_PLACE (the gate's `in-place` config).
 */

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
	 * list's engine proxy tag (URCU_TXN_TAG, bit 0) and readers resolve them
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
	 * so only the op can say whether it acquired it.  ft_detach_node's
	 * republish does (the recompact takes {C,P,GP} exactly when P is
	 * compressed, which is exactly when the dual arises); ft_promote_head
	 * does not.
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
 * bits 2-10 = nr_child, bits 11-18 = parent_slot_offset, bit 19 = LOCK
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
 * (ft_state_edge / ft_flip_txn_record_release_lock / ft_flip_txn_record_count_
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

struct cds_ft_metadata {
	/* 8-byte aligned fields. */
	struct cds_ft_inode_flag *parent_word;	/*
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
	uintptr_t parent_slot_offset;

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
static inline
void ft_meta_parent_slot_offset_set(struct cds_ft_metadata *meta, unsigned int off)
{
	uintptr_t n = FT_PSO_ENCODE(off);

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
	 * at every mutation.  Default CDS_FT_WRITER_LOCK_FINE (DLM), resolved at
	 * group create from @writer_strategy_set.
	 */
	enum cds_ft_writer_strategy writer_strategy;
	/*
	 * Granularity of the per-node lock-sets a FINE trie takes: how far up
	 * the descent a writer anchors each lock-set member
	 * (doc/design/ft-dlm-lock-coarseness.md).  Copied to each trie at
	 * create; resolved at group create from @lock_spacing_set, default
	 * CDS_FT_LOCK_SPACING_PER_NODE.  Inert under COARSE, which derives no
	 * lock-set.
	 */
	enum cds_ft_lock_spacing lock_spacing;
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
	unsigned long move_gate_nr;	/* movers holding the gate */
	bool move_gate_gp;		/* the 0->1 owner is inside its GP */

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
	 * Hot-path gate: writer_strategy == CDS_FT_WRITER_LOCK_FINE.  The
	 * fine-grained (per-node lock-set) conversions read THIS:
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
	 * Hot-path copy of the group's lock-set granularity, read by the
	 * mutation descent (ft_descent_init) to decide how far up it anchors
	 * each lock-set member.  Inert unless @lock_fine.
	 */
	enum cds_ft_lock_spacing lock_spacing;


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
 * Take the FT-wide writer lock, parked OFFLINE.
 *
 * MANDATORY, and the invariant is the one ft_writer_lock_gp_wait states: "writers
 * parked on the lock are RCU-online and non-quiescent, so they are precisely what
 * stops this grace period from ever completing."  Dropping the lock across a GP
 * (which gp_wait does) fixes only the SELF-deadlock -- a holder waiting on its own
 * waiters.  It does nothing about the group: N writers whose ops each contain a
 * grace period wedge each other, because whichever of them is parked here is an
 * online reader the GP waits for, and the GP is what the lock holder is waiting
 * on.  Measured: four concurrent cds_ft_rekey_graft writers on a COARSE trie made
 * ZERO moves, permanently, while three sat here and the call_rcu thread sat in
 * wait_for_readers.
 *
 * SAFE, because a park dereferences NOTHING and the caller already survives a full
 * grace period across it.  Under QSBR, synchronize_rcu marks its caller quiescent
 * for the duration anyway, so every pointer a writer holds across
 * ft_writer_lock_gp_wait is ALREADY exposed to reclamation and must already be
 * kept valid by the writer lock / exclusivity rather than by this thread's
 * online-ness.  Being offline for the subsequent park adds no exposure.  The
 * caller's own @node argument is app-owned and never library-reclaimed.  A no-op
 * on the memb / mb flavors.
 */
static inline
void ft_writer_lock_park(struct cds_ft *ft)
{
	const struct rcu_flavor_struct *flavor = ft->group->flavor;

	flavor->thread_offline();
	cds_fair_mutex_lock(&ft->writer_lock, &ft_wlock_waiter);
	flavor->thread_online();
}

/*
 * Take the FT-wide writer lock at the OUTERMOST writer scope; reentrant no-op
 * on a nested scope for the same trie.  Every trie is lock-mode now (COARSE or
 * FINE), so there is no optimistic early-out; a FINE trie skips the FT-wide
 * lock further down, under the lock-set drop.
 */
static inline
void ft_writer_lock_scope_enter(struct cds_ft *ft)
{
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
		return;
	}
#endif
	if (ft->lock_fine) {
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
		return;
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
	ft_writer_lock_park(ft);
	ft_wlock_held = ft;
	ft_wlock_depth = 1;
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
	unsigned long own = 0;

	if (held) {
		own = ft_excl_owner_release(held);
		ft_wlock_held = NULL;
		ft_wlock_depth = 0;
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
	ft->group->flavor->update_synchronize_rcu();
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
	return CMM_LOAD_SHARED(ft->move_active) != 0;
}

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
void ft_move_gate_enter(struct cds_ft *ft)
{
	pthread_mutex_lock(&ft->move_gate_lock);
	if (ft->move_gate_nr++ == 0) {
		CMM_STORE_SHARED(ft->move_active, 1);
		ft->move_gate_gp = true;
		pthread_mutex_unlock(&ft->move_gate_lock);
		/*
		 * The barrier the whole gate exists for: readers that were
		 * already inside a critical section when the store landed may
		 * still believe they are in fast mode, so let them finish.
		 */
		assert(!urcu_txn_in_fallback());	/* see gp_wait */
		ft->group->flavor->update_synchronize_rcu();
		pthread_mutex_lock(&ft->move_gate_lock);
		ft->move_gate_gp = false;
		pthread_cond_broadcast(&ft->move_gate_cond);
		pthread_mutex_unlock(&ft->move_gate_lock);
		return;
	}
	if (ft->move_gate_gp) {
		/*
		 * PIGGYBACK, and go QUIESCENT while doing it.  Under a
		 * quiescent-state flavor (QSBR) a registered thread counts as
		 * being inside a read section until it reports otherwise, so a
		 * mover that simply blocked here would be a reader the owner's
		 * grace period waits for -- while the owner waits for us and we
		 * wait for the owner.  That is a hard deadlock, and it is what
		 * happens (measured: the 16-writer oracle wedged) without these
		 * two calls.  Sound because a mover holds NO read section at this
		 * point: the gate is entered BEFORE the read lock, by contract.
		 * A no-op on the memb / mb flavors.
		 */
		ft->group->flavor->thread_offline();
		do {
			pthread_cond_wait(&ft->move_gate_cond,
					&ft->move_gate_lock);
		} while (ft->move_gate_gp);
		pthread_mutex_unlock(&ft->move_gate_lock);
		ft->group->flavor->thread_online();
		return;
	}
	pthread_mutex_unlock(&ft->move_gate_lock);
}

/*
 * Leave the move gate; the LAST mover out returns readers to the fast path.  No
 * grace period is needed on the way out: a reader that still sees @move_active
 * set merely runs the coherent path once more, which is never wrong, only slower.
 */
static inline
void ft_move_gate_exit(struct cds_ft *ft)
{
	pthread_mutex_lock(&ft->move_gate_lock);
	if (--ft->move_gate_nr == 0)
		CMM_STORE_SHARED(ft->move_active, 0);
	pthread_mutex_unlock(&ft->move_gate_lock);
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
				return;
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
	/*
	 * Granularity of the per-node lock-sets under CDS_FT_WRITER_LOCK_FINE.
	 * Meaningful only when @lock_spacing_set; otherwise the group takes the
	 * CDS_FT_LOCK_SPACING_PER_NODE default.  See
	 * cds_ft_group_attr_set_lock_spacing.
	 */
	enum cds_ft_lock_spacing lock_spacing;
	bool lock_spacing_set;
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
