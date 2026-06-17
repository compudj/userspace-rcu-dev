// SPDX-FileCopyrightText: 2012-2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-only

/*
 * src/fractal-trie/ft-scanners.h
 *
 * Userspace RCU library - Fractal Trie: internal node scanners (popcount 1L/2L, pigeon) + node grow / recompact / set_nth.
 *
 * Implementation unit: #included once by fractal-trie.c, in dependency
 * order, into a single translation unit (preserves cross-module inlining).
 * Not a standalone header.
 */
#ifndef FRACTAL_TRIE_IMPL
#error "ft-scanners.h is an implementation unit; #include it from fractal-trie.c only"
#endif

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
 * site -- the branches inside ft_maybe_prefetch_hint fold away, leaving
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
 * Generic 2-level popcount-bitmap node header (4+4 byte-split).
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
 * Lookup primitive: per-slot flat popcount with 5+3 byte-split,
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
 * Lookup primitive: flat packed sub_bms with 6+2 byte-split,
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
 * subtract is a compile-time constant -- the compiler emits a single
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
 * @node_flag is still tagged -- each switch case applies
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
 * parent chain on a mismatch (the single concurrency mechanism -- see
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
 * ft_node_find_child: reverse lookup -- given a parent internal node and
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
	/* Parent back-pointer is wired inside the per-class body, ahead of
	 * the forward store. */
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
		 * skip-specific -- it backs the parent-pointer backtrack's O(1)
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
 * @cluster_leaf: when true, the target node is a cluster-leaf -- the lower
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
		 * group) leave parent_slot_offset == 0 -- a latent gap that
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
	 * In candidate mode, skip key comparison -- just advance past
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
