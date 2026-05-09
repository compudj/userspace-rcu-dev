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
#include <assert.h>

/*
 * Configuration tweaks. Comment out those defines to disable features.
 */
#define FEATURE_USE_BITMAP_SCAN
#define FEATURE_INLINE_LOOKUP

/*
 * Tag-bit kind encoding (5-bit, "Candidate E") in the low bits of
 * every cds_ft_inode_flag pointer.  The enum value IS the pointer's
 * low 5 bits.  See doc/design/qp-5bit-tag-encoding.md for design
 * rationale.
 *
 *   FT_KIND_EXT (0x00)         external leaf chain head; pointer is
 *                              the raw struct cds_ft_node *.
 *                              Matches NULL.
 *   FT_KIND_PIGEON (0x01)      pigeon (dense 256-pointer) node;
 *                              cds_ft_inode *.
 *   FT_KIND_SKIP_EXT (0x02)    skip-compressed pointer; resolved
 *                              target is EXT.
 *   FT_KIND_SKIP_PIGEON (0x03) skip-compressed pointer; resolved
 *                              target is PIGEON.
 *   FT_KIND_COMPRESSED (0x04)  pointer to a struct
 *                              cds_ft_compressed_node.
 *   FT_KIND_QP (0x05)          QP-nibble node (hi or lo, all tiers);
 *                              cds_ft_qp16_node *.  HI vs LO is
 *                              recovered from the lo-node's metadata
 *                              is_lo bit during the upward parent
 *                              walk.
 *   FT_KIND_SKIP_QP (0x07)     skip-compressed pointer; resolved
 *                              target is QP_HI.
 *
 * Reserved for future Stage 3 wire-up (assert-on-encode):
 *   FT_KIND_POPCOUNT_32 (0x09), FT_KIND_SKIP_POPCOUNT_32 (0x0B),
 *   FT_KIND_POPCOUNT_64 (0x11), FT_KIND_SKIP_POPCOUNT_64 (0x13).
 *
 * Other reserved patterns: 0x06, 0x08, 0x0A, 0x0C-0x0F, 0x10, 0x12,
 * 0x14-0x1F.  Bit pattern 0x06 (bit 0=0, bit 1=1, bit 2=1) is
 * reserved: it would mean "skip-compressed of COMPRESSED", forbidden
 * by the chain-compress invariant.
 *
 * Bit-pattern rationale:
 *   - bit 0 partitions the encoding by underlying alignment:
 *     external-aligned (EXT, SKIP_EXT, COMPRESSED, 16-byte aligned)
 *     has bit 0 clear; internal-aligned (PIGEON, QP, POPCOUNT_*,
 *     and their SKIP variants — 32-byte aligned) has bit 0 set.
 *   - bit 1 is the universal "is skip-compressed" predicate; setting
 *     bit 1 on a child kind tag yields the matching SKIP_* tag
 *     (skip = child | 0x2; child = skip - 0x2).
 *   - bit 2 in the external-aligned half identifies COMPRESSED;
 *     in the internal-aligned half it is QP's one-hot class bit.
 *   - bit 3 / bit 4 are POPCOUNT_32 / POPCOUNT_64 one-hot class
 *     bits in the internal-aligned half.
 *
 * Internal nodes are 32-byte aligned (FT_QP16_T0_ALLOC_ORDER = 5
 * and FT_PIGEON_ORDER ≥ 10) so the low 5 bits are guaranteed zero
 * in raw addresses.  External nodes (struct cds_ft_node) and
 * compressed nodes are 16-byte aligned — bits 0-3 are kind bits,
 * bit 4 of the underlying address remains free (for external-
 * aligned tags 0x00/0x02/0x04 it does not contaminate the kind
 * extraction at the bit-2 level used by the read-side predicates).
 */
enum ft_kind {
	FT_KIND_EXT		= 0x00,
	FT_KIND_PIGEON		= 0x01,
	FT_KIND_SKIP_EXT	= 0x02,
	FT_KIND_SKIP_PIGEON	= 0x03,
	FT_KIND_COMPRESSED	= 0x04,
	FT_KIND_QP		= 0x05,
	FT_KIND_SKIP_QP		= 0x07,
};

#define FT_KIND_SKIP_BIT	0x2UL	/* skip = child | FT_KIND_SKIP_BIT */

/*
 * Kind extraction mask.  Under Candidate E both slot-context and
 * direct-internal sites use a 5-bit mask: internal kinds are 32-byte
 * aligned (clean 5-bit kind), and external-aligned kinds (EXT,
 * SKIP_EXT, COMPRESSED) keep bits 0-2 as clean kind bits — bit 4
 * remains an address bit but is filtered out by the bit-aware
 * predicates (ft_node_external uses `& 0x05`, ft_node_compressed
 * uses `& 0x07`, ft_node_skip_compressed uses bit 1).  Direct
 * extraction of `(v & FT_KIND_MASK)` against an external-aligned
 * value still risks a bit-4 leak, so callers compare against the
 * specific kind tag they expect (where bit 4 is 0) and rely on the
 * predicate-level bit-aware filtering above instead.
 */
#define FT_KIND_MASK		0x1FUL
#define FT_KIND_PTR_MASK	(~FT_KIND_MASK)

/*
 * Legacy alias for sites that have already established the value is
 * a direct internal kind.  Equal to FT_KIND_MASK under Candidate E
 * (both 0x1F).  Kept as a documentation hint for caller intent.
 */
#define FT_KIND_MASK_INTERNAL	FT_KIND_MASK

/*
 * Skip-compressed pointer encoding.
 *
 * When FEATURE_FT_SKIP_COMPRESSED is defined, compressed node pointers
 * are replaced by "skip pointers" that point directly to the
 * compressed node's child, skipping the compressed node on the read
 * fast path (candidate lookup).
 *
 * The skip pointer's kind nibble is FT_KIND_SKIP_EXT / FT_KIND_SKIP_QP
 * / FT_KIND_SKIP_PIGEON depending on the resolved child class.  Length
 * recovery uses the per-item compress-cache page (16 bytes per item,
 * one page after the items page in each arena range): SKIP_QP and
 * SKIP_PIGEON readers fetch cn->len from cache->skip_len with a
 * single ALU-add (child + page_size); SKIP_EXT readers fall back to
 * the external_node->prev → cn->len chain (external nodes may live
 * outside our arenas, so the page-offset cache slot is not safe to
 * read for them).
 *
 * The high bits of a skip pointer are zero — the legacy
 * `len << shift` encoding (and its per-arch shift table) is gone,
 * so the read-side ft_node_ptr family carries no defensive high-bit
 * strip on the lookup hot path's load-address dependency chain.
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

/*
 * Maximum compressed-path length that can be skip-compressed.  Bound
 * by the compress-cache's uint8_t skip_len field (FF == 255).  Paths
 * longer than this keep the traditional cn_flag pointer; readers
 * descend through the compressed node's child slot.
 */
#define FT_SKIP_LEN_MAX		255U

#define FT_ENTRY_PER_NODE	256
#define FT_LOG2_BITS_PER_BYTE	3U
#define FT_BITS_PER_BYTE	(1U << FT_LOG2_BITS_PER_BYTE)

#define FT_MAX_KEY_LEN	256			/* Maximum key length supported. */
#define FT_MAX_DEPTH	(FT_MAX_KEY_LEN + 1)	/* Maximum depth, including root. */

/*
 * ft_types[] internal-class slot count.  Per-class entries (one
 * per node class), indexed by class — not per QP tier.  QP's four
 * allocation tiers (T0..T3, orders 5..8) are handled internally
 * via the parallel ft_qp16_tiers[] table; the recompact framework
 * sees QP as a single entry that the QP-internal tier picker
 * (ft_qp16_alloc_order, popcount-driven) sub-resolves to a tier.
 *
 * Lo-nodes share the FT_KIND_QP slot-tag with hi-nodes (HI/LO
 * disambiguated via metadata is_lo bit during the parent walk)
 * and are never indexed into ft_types[].
 *
 * NODE_INDEX_NULL is the one-past-the-end sentinel, used by
 * recompact / verify code to encode "elide this node"; ft_types[]
 * keeps a trailing FT_NULL entry at that index so
 * &ft_types[NODE_INDEX_NULL] is in-bounds.
 *
 * Indices: FT_QP_INDEX = 0, FT_PIGEON_INDEX = 1.  Future Stage 3
 * (POPCOUNT_32 / POPCOUNT_64) reserves indices 0..1 for the
 * popcount-byte classes and shifts QP / PIGEON to 2 / 3.
 */
#define FT_QP_INDEX		0U
#define FT_PIGEON_INDEX		1U
#define FT_NUM_INTERNAL_TYPES	2U
#define NODE_INDEX_NULL		FT_NUM_INTERNAL_TYPES

/*
 * Number of removals needed on a fallback node before we try to shrink
 * it.  Derived from FT_FALLBACK_REMOVAL_BITS to always use the full
 * range of the bitfield.
 */
#define FT_FALLBACK_REMOVAL_BITS	3
#define FT_FALLBACK_REMOVAL_COUNT	((1U << FT_FALLBACK_REMOVAL_BITS) - 1)

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

/*
 * QP-nibble internal node layout.
 *
 * Internal nodes come in two flavors:
 *
 *   - QP-nibble : 16-bit popcount bitmap + popcount-indexed pointer
 *                 array.  Each level dispatches on one nibble (4 bits).
 *                 Multiple size tiers driven by popcount(bitmap).
 *   - PIGEON    : direct ptrs[256], promoted from QP-nibble pairs
 *                 when the byte-level fan-out crosses a threshold.
 *
 * Trie depth doubles for byte-keyed inputs (each byte becomes two
 * nibble levels); the depth-doubling cost at runs is absorbed by
 * skip-compressed pointers, which collapse compressed-node chains
 * out of the read-side descent on architectures that support them.
 */

/*
 * Skip-compressed pointers tag a child slot's pointer with FT_KIND_
 * SKIP_* and bypass the compressed node on the candidate-lookup fast
 * path.  Length / subkey recovery is inline in the destination node
 * (cds_ft_qp16_node::skip_len + subkey[] for QP targets,
 * cds_ft_metadata::pigeon_skip_len for PIGEON; SKIP_EXT routes
 * through the external_node->prev → cn->len chain).
 *
 * Length / subkey recovery is inline in the destination node since
 * Phase B.3/B.4 retired the high-bit pointer encoding, so the
 * mechanics are no longer tied to specific 64-bit virtual-address
 * layouts.  Enabled on both 32-bit and 64-bit; a fair perf
 * comparison between the two ABIs guides any follow-up work on
 * pointer compression on 64-bit.
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
# define FEATURE_FT_SKIP_COMPRESSED
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
#ifdef FEATURE_FT_VERIFY_AT_MUTATION
void ft_writer_scope_verify(struct cds_ft *ft);
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
 * Struct layout (32 bytes, zero internal padding on 64-bit):
 *   offset  0: 8-byte parent pointer
 *   offset  8: 8-byte external_nodes pointer
 *   offset 16: 8-byte nr_keys (unsigned long)
 *   offset 24: 4-byte packed bitfield (nr_child, skip_slot_offset,
 *              fallback_removal_count, alloc_index)
 *   offset 28: pigeon_skip_len OR qp_subtree_half_cls (1 byte each,
 *              union — see field comments) + 2 bytes trailing pad
 *
 * In cds_ft_metadata_alloc, rcu_head is a separate field placed
 * before the metadata union — no overlap with metadata fields.
 * All metadata fields remain valid throughout the RCU grace period.
 *
 * nr_keys is full unsigned-long width and accessed with release on
 * the write side / acquire on the read side.  It carries no
 * sentinel value: the historical UINT32_MAX "promoted" sentinel and
 * the per-depth uint8_t density counters were removed when the
 * collapse heuristic that drove them was retired.
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
	 * Total unique keys in subtree.  Stored with uatomic_store
	 * release on the write side, loaded with acquire on the read
	 * side.
	 */
	unsigned long nr_keys;

	/*
	 * Packed bitfield — small fields in a single uint32_t (or uint64_t
	 * on big-page archs where the total bit count exceeds 32).
	 *
	 * nr_child:               9 bits (max 256)
	 * skip_slot_offset:       8 bits (byte_offset / sizeof(void *)
	 *                         from parent node; ifdef-gated, 0 when
	 *                         disabled)
	 * fallback_removal_count: 3 bits (max 7)
	 * alloc_index:            FT_ALLOC_INDEX_BITS (architecture-dependent,
	 *                         sized for max page_size >> FT_ALLOC_ORDER_MIN)
	 * is_lo:                  1 bit  (set on QP-nibble lo-node metadata;
	 *                         used in the parent-walk to recover the
	 *                         HI/LO distinction now that FT_KIND_QP_LO
	 *                         no longer occupies a slot-tag value)
	 */
	uint32_t nr_child:9;
#ifdef FEATURE_FT_SKIP_COMPRESSED
	uint32_t skip_slot_offset:8;
#endif
	uint32_t fallback_removal_count:FT_FALLBACK_REMOVAL_BITS;
	uint32_t alloc_index:FT_ALLOC_INDEX_BITS;
	uint32_t is_lo:1;
	/*
	 * Trailing-pad byte at offset 28.  Two mutually-exclusive uses
	 * — a metadata is either a QP-hi's or a PIGEON's, never both:
	 *
	 *   FT_KIND_PIGEON skip target → pigeon_skip_len: cn->len when
	 *     this PIGEON is the target of a skip-compressed pointer.
	 *     PIGEON has no spare bytes in its dense ptrs[256] layout,
	 *     so the skip length lives here.  No subkey is cached for
	 *     PIGEON skip targets — validation falls back to the leaf-
	 *     bytes compare.  Gated on FEATURE_FT_SKIP_COMPRESSED.
	 *
	 *   FT_KIND_QP hi-node → qp_subtree_half_cls: total
	 *     half-cacheline (32 B) footprint of the hi+lo subtree
	 *     rooted at this hi.  Used by the QP→PIGEON up-trigger:
	 *     when the count exceeds PIGEON's flat footprint (64
	 *     half-CLs = 2 KB on 64-bit; 32 half-CLs = 1 KB on 32-bit),
	 *     the recompact framework swaps to PIGEON.  Set on hi
	 *     creation, +=/-= on Path 1 (new T0 lo), Path 2b (lo CoW
	 *     grow), and ft_qp_byte_clear (last-byte lo elide).
	 */
	union {
#ifdef FEATURE_FT_SKIP_COMPRESSED
		uint8_t pigeon_skip_len;
#endif
		uint8_t qp_subtree_half_cls;
	};
};

/*
 * Compressed path node.  Replaces a chain of single-child internal
 * nodes with a single node storing the key bytes inline.
 *
 * cn->child can point to any node type: internal, compressed,
 * collapsed, or external.  An external child means a key terminates
 * at the end of the compressed path.  However, the compressed node's
 * metadata->external_nodes must NOT be used — variable-length key
 * entries belong on internal or collapsed nodes (which can have
 * metadata->external_nodes), or as child pointer of a compressed
 * or collapsed node.
 *
 * Tagged in the parent's child pointer with FT_KIND_COMPRESSED (low
 * nibble = 0x1).
 *
 * Layout: [child pointer] [len] [key_bytes...]
 */
struct cds_ft_compressed_node {
	struct cds_ft_inode_flag *child;	/* Child at end of compressed path. */
	uint8_t len;				/* Number of key bytes in path (1-255). */
	uint8_t key_bytes[];			/* Compressed key path (flexible array). */
};

/*
 * QP-nibble internal node — 16-bit popcount bitmap + popcount-indexed
 * pointer array.  Each level dispatches on one nibble (4 bits, values
 * 0..15).  Lookup is:
 *
 *   if (!(bitmap & (1u << n))) return NULL;
 *   idx = popcount(bitmap & ((1u << n) - 1u));
 *   return ptrs[idx];
 *
 * Two dependent loads on the hot path: bitmap (in the header CL) and
 * ptrs[idx] (in the pointer-array CL — typically the same CL for the
 * smaller tiers).
 *
 * Tier sizing (popcount → alloc):
 *
 *   T0 (32B):  popcount  ≤ 3   (header 8B + 3 × 8B  = 32B)
 *   T1 (64B):  popcount  ≤ 7   (header 8B + 7 × 8B  = 64B)
 *   T2 (128B): popcount ≤ 15   (header 8B + 15 × 8B = 128B)
 *   T3 (256B): popcount  = 16  (header 8B + 16 × 8B = 136B; padded to 256B)
 *
 * popcount = 0 is valid only for the root node of an empty trie
 * (no parent slot exists to clear).  Non-root internal nodes never
 * reach popcount = 0: when a removal would drop nr_child to 0, the
 * detach path replaces the node in its grandparent's slot and frees
 * it.  popcount = 1 is common at byte-aligned compressed boundaries
 * (high-nibble level when the parent compressed node ends mid-byte)
 * and at root before the second key is inserted.
 *
 * The 8-byte header reserves 6 bytes after the bitmap.  Under
 * FEATURE_FT_SKIP_COMPRESSED, bytes 2-7 carry an inline skip cache:
 *
 *   byte 2     : skip_len   (cn->len when this QP is a skip target;
 *                            0 otherwise.  Same CL as bitmap, so the
 *                            skip-resolver pays no extra CL load.)
 *   bytes 3-7  : subkey[5]  (leading bytes of the parent cn's
 *                            key_bytes.  Skip-compressed candidate
 *                            descents validate immediately against
 *                            this subkey when skip_len ≤ 5; longer
 *                            paths and PIGEON / EXT skips defer to a
 *                            single end-of-descent leaf-bytes compare.)
 *
 * Without FEATURE_FT_SKIP_COMPRESSED the same 6 bytes remain reserved
 * padding so the layout (and ptrs[] offset) stays identical across
 * configs.
 */
#define FT_QP16_HEADER_SIZE	8U
#define FT_QP16_NR_TIERS	4U
#define FT_QP16_T0_CAPACITY	3U
#define FT_QP16_T0_ALLOC_ORDER	5U	/* 32B */
#define FT_QP16_T1_CAPACITY	7U
#define FT_QP16_T1_ALLOC_ORDER	6U	/* 64B */
#define FT_QP16_T2_CAPACITY	15U
#define FT_QP16_T2_ALLOC_ORDER	7U	/* 128B */
#define FT_QP16_T3_CAPACITY	16U
#define FT_QP16_T3_ALLOC_ORDER	8U	/* 256B */

#define FT_QP16_SUBKEY_INLINE_LEN	5U

struct cds_ft_qp16_node {
	uint16_t bitmap;			/* bytes 0-1: nibble-presence bitmap */
#ifdef FEATURE_FT_SKIP_COMPRESSED
	uint8_t  skip_len;			/* byte 2: cn->len when skip target, 0 otherwise */
	uint8_t  subkey[FT_QP16_SUBKEY_INLINE_LEN]; /* bytes 3-7: leading cn->key_bytes */
#else
	uint8_t  _pad[FT_QP16_HEADER_SIZE - 2];	/* bytes 2-7: reserved */
#endif
	struct cds_ft_inode_flag *ptrs[];	/* bytes 8+: popcount(bitmap) entries */
} __attribute__((__aligned__(8)));

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
	/*
	 * Allocation arenas, indexed by item_len_order.  All node
	 * classes share a single arena per order; skip-compressed
	 * metadata is stored inline in the destination node.
	 */
	struct cds_ft_alloc_arena *arena_order[FT_ALLOC_ORDER_MAX + 1];
	pthread_mutex_t arena_lock;	/* Protects lazy arena creation. */
	struct cds_ft_key_map key_map;
	unsigned long nr_ft_instances;	/* Number of Fractal Trie instances in the group. */

	/*
	 * Speculative-descent / library-side validation attributes.
	 * @speculative_validated: lookups (cds_ft_lookup_key,
	 *   iterator-based cds_ft_lookup) descend speculatively and
	 *   validate the result against the external node's stored key
	 *   bytes via the inline SIMD/SWAR comparator before returning.
	 *   Skip-compressed pointer encoding is enabled at build time via
	 *   FEATURE_FT_SKIP_COMPRESSED; no per-group runtime gate.
	 * @speculative_key_offset: byte offset from the external node's
	 *   address to the start of the stored key.  Used only when
	 *   @speculative_validated is true.
	 * @speculative_key_len_offset: byte offset (same base) to a
	 *   size_t holding the key length, for variable-length-key
	 *   groups.  CDS_FT_SPECULATIVE_OFFSET_NONE for fixed-length.
	 */
	bool speculative_validated;
	size_t speculative_key_offset;
	size_t speculative_key_len_offset;
};


struct cds_ft {
	struct cds_ft_group *group;

	struct cds_ft_inode_flag *root;		/* Root node (arena-allocated, always present, always internal). */
	size_t max_used_key_len;		/* Maximum key length inserted (conservative). */
	unsigned long nr_fallback;		/* Number of fallback nodes used */

	/*
	 * Access discipline. When true, access is serialized
	 * externally (single-threaded or mutex-protected) and no
	 * concurrent RCU readers exist; mutation operations that
	 * would otherwise require a grace period to re-parent a
	 * subtree (graft, graft_swap) may skip synchronize_rcu().
	 * When false, concurrent RCU readers are permitted.
	 */
	bool exclusive;

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
	unsigned long node_fallback_count_distribution[FT_ENTRY_PER_NODE];
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
 * Each 2*page_size range holds:
 *
 *   page 0:  items array
 *   page 1:  cds_ft_alloc_range header + cds_ft_metadata_alloc array
 *            (optional cds_ft_bitmap array grows backward from end of
 *            page 1 on 2D-pool / pigeon arenas)
 *
 * Skip-compressed metadata (skip_len, optional subkey) is stored
 * inline in the destination node — in cds_ft_qp16_node::skip_len /
 * subkey[] for QP nodes, and in cds_ft_metadata::pigeon_skip_len for
 * PIGEON nodes.  No dedicated compress-cache page is needed.
 *
 * The cache lines most often prefetched from a tagged child pointer —
 * the item's metadata and (for bitmap types) bitmap — are derivable
 * with pure pointer arithmetic.  The helpers below live in this
 * header so the prefetch-hint path can compute their addresses
 * inline, without a cross-TU call into fractal-trie-alloc.c.
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
 * Range header lives one page above the items page.
 * Skip-compressed metadata lives inline in the destination node;
 * no extra cache page is needed.
 */
#define FT_RANGE_HDR_PAGE_OFFSET	(1UL * cds_ft_get_page_size())
#define FT_RANGE_END_PAGE_OFFSET	(2UL * cds_ft_get_page_size())

static inline
struct cds_ft_alloc_range *cds_ft_item_to_range(void *p)
{
	size_t pg = cds_ft_get_page_size();
	void *base = (void *)((unsigned long) p & ~(pg - 1));

	return (struct cds_ft_alloc_range *) ((char *) base + FT_RANGE_HDR_PAGE_OFFSET);
}

static inline
struct cds_ft_metadata *cds_ft_item_to_metadata_fast(void *p, size_t item_len_order)
{
	struct cds_ft_alloc_range *range = cds_ft_item_to_range(p);
	size_t page_offset = (unsigned long) p & (cds_ft_get_page_size() - 1);
	size_t index = page_offset >> item_len_order;

	return &range->metadata[index].metadata;
}

/*
 * bitmap array is indexed backwards from end of range header page.
 */
static inline
struct cds_ft_bitmap *cds_ft_item_to_bitmap(void *p, size_t item_len_order)
{
	size_t pg = cds_ft_get_page_size();
	void *base = (void *)((unsigned long) p & ~(pg - 1));
	size_t index = ((unsigned long) p & (pg - 1)) >> item_len_order;

	return (struct cds_ft_bitmap *) ((char *) base + FT_RANGE_END_PAGE_OFFSET -
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

/*
 * Allocate an item from the per-order arena.
 *
 * The arena is created lazily on first use.
 */
__attribute__((visibility("hidden")))
struct cds_ft_metadata *cds_ft_alloc_item(struct cds_ft *ft,
		size_t item_len_order, bool bitmap);

__attribute__((visibility("hidden")))
void cds_ft_free_item(struct cds_ft *ft, struct cds_ft_metadata *metadata);

/*
 * cds_ft_free_item_unpublished - immediate free for items that were
 * never published (no reader can possibly hold a reference).  Bypasses
 * call_rcu and returns the slot directly to the arena free list.
 *
 * Use only for nodes that never escaped the writer's stack (e.g.,
 * speculative candidate collapsed nodes evaluated by
 * ft_try_collapse_at_node and rejected before publication).  Calling
 * this on a published node corrupts concurrent readers.
 */
__attribute__((visibility("hidden")))
void cds_ft_free_item_unpublished(struct cds_ft *ft, struct cds_ft_metadata *metadata);

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

/*
 * Tracepoint node-kind identifiers.
 *
 * These values are exposed as an LTTng-UST enumeration in src/ft_tp.h
 * (via the LTTNG_UST_TRACEPOINT_ENUM declaration which references
 * these C enum labels).  Keeping both in the same enum guarantees the
 * C side and the trace-metadata side cannot drift out of sync.
 *
 * Labels are designed to describe the underlying structure without
 * requiring the reader to know the build's pointer width:
 *   - LINEAR/LINEAR_WIDE: total node size in bytes (= 2^order).
 *   - POOL_<dim>D_<bytes>: dimensionality (1D = single sub-array,
 *     2D = matrix of sub-arrays) and total node size in bytes.
 *     POOL_1D_512 (a 64-bit POOL_IDX_A) is structurally distinct
 *     from POOL_2D_512 (a 32-bit POOL_IDX_B) even though both have
 *     the same byte size.
 *   - PIGEON: 256-entry direct table; the byte size depends on
 *     pointer width (1024 on 32-bit, 2048 on 64-bit).
 *   - COLLAPSED: single label; the scan-size variant requires reading
 *     the node header and is intentionally not exposed in this enum.
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
	FT_TP_NODE_COLLAPSED		=  3,
	FT_TP_NODE_LINEAR_16		=  4,
	FT_TP_NODE_LINEAR_32		=  5,
	FT_TP_NODE_LINEAR_64		=  6,
	FT_TP_NODE_LINEAR_128		=  7,
	FT_TP_NODE_LINEAR_256		=  8,
	FT_TP_NODE_POOL_1D_256		=  9,
	FT_TP_NODE_POOL_1D_512		= 10,
	FT_TP_NODE_POOL_2D_512		= 11,
	FT_TP_NODE_POOL_2D_1024		= 12,
	FT_TP_NODE_PIGEON_1024		= 13,
	FT_TP_NODE_PIGEON_2048		= 14,
	FT_TP_NODE_UNKNOWN		= 15,
};

#endif /* _URCU_FT_INTERNAL_H */
