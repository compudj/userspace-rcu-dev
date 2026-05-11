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
	bool speculative_validated;
	size_t speculative_key_offset;
	size_t speculative_key_len_offset;
};

struct cds_ft_attr {
	bool exclusive;
};

enum cds_ft_type_class {
	FT_PIGEON = 0,		/* Pigeon: direct indexed */
	FT_QP = 1,		/* QP-nibble: 16-bit popcount + ptrs[] */
	FT_POPCOUNT = 2,	/*
				 * Popcount-byte: 2-level nibble bitmap +
				 * popcount-indexed ptrs[].  Reserved for
				 * Step 4 sub-stage B2 wire-up; helpers
				 * and ft_types[] entries land later.
				 */
	/* Leaf nodes are implicit from their height in the tree */
	FT_NR_TYPES,

	FT_NULL,	/* not an encoded type, but keeps code regular */
};

struct cds_ft_type {
	enum cds_ft_type_class type_class;
	uint16_t min_child;		/*
					 * Minimum / maximum number of children
					 * for the DIRECT variant (1..256).
					 * Used by find_nearest_type_index for
					 * nodes that are NOT skip targets.
					 */
	uint16_t max_child;
	uint16_t min_child_skip;	/*
					 * Same, for the SKIP-target variant.
					 * For types where skip mode reserves
					 * inline space (FT_POPCOUNT — slot 0
					 * holds ft_pc32_skip_meta), max_child_-
					 * skip is one less than max_child.  For
					 * types with no per-variant capacity
					 * difference (FT_QP, FT_PIGEON), these
					 * mirror the direct bounds so the
					 * lattice walk lands on the same type
					 * entries regardless of mode.
					 */
	uint16_t max_child_skip;
	uint16_t order;			/* node size is (1 << order), in bytes */
	bool bitmap;			/* allocate bitmap */
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
 * The smallest allocation order is 4 (16-byte alignment): the low 4
 * bits of every tagged pointer carry the kind nibble (FT_KIND_*).
 */

enum {
	ft_type_pigeon_max_child = 256,
	ft_type_null_max_child = 256,
};

/*
 * PIGEON node alloc order: dense 256-entry pointer array.
 * 64-bit: 256 * 8 = 2048 B = 1 << 11.
 * 32-bit: 256 * 4 = 1024 B = 1 << 10.
 */
#if CAA_BITS_PER_LONG >= 64
# define FT_PIGEON_ORDER	11U
#else
# define FT_PIGEON_ORDER	10U
#endif

/*
 * ft_types[]: write-side per-class metadata (allocator order, child
 * count bounds, bitmap requirement).  One entry per node class —
 * QP's four tier orders are an internal detail captured by the
 * parallel ft_qp16_tiers[] table.  The kind tag bits encode the
 * dispatch class directly, so this table is not consulted on the
 * read-side hot path.
 *
 *   [FT_POPCOUNT_32_INDEX] = POPCOUNT_32 class — order 5 (32 B),
 *            2-level nibble bitmap with popcount-indexed ptrs[]
 *            (scan_3, 4+4 byte split).  Direct variant max_lc = 3.
 *            Used as the smallest internal node, replacing fresh-
 *            allocation QP T0.
 *   [FT_POPCOUNT_64_INDEX] = POPCOUNT_64 class — order 6 (64 B),
 *            scan_6 flat-packed layout (5+3 byte split, single
 *            packed_bms u64).  Direct variant max_lc = 6.  Sits
 *            between POPCOUNT_32 and QP in the lattice walk: nr_-
 *            child in [4, 6] lands here.
 *   [FT_QP_INDEX]     = QP class — 16-nibble bitmap, popcount-
 *            indexed ptrs[].  Internal tier picker (T0..T3 at
 *            orders 5..8) is popcount-driven via
 *            ft_qp16_alloc_order; ft_types[FT_QP_INDEX].order
 *            holds the T0 default for fresh allocations.  Upper
 *            bound is enforced by the QP→PIGEON CL-footprint
 *            trigger (qp_subtree_half_cls > FT_PIGEON_HALF_CLS),
 *            not by max_child.
 *   [FT_PIGEON_INDEX] = PIGEON class — order 11 (2048 B), dense
 *            256-entry pointer array.  min_child = 24 is the
 *            conservative hysteresis demote threshold, lower than
 *            the forward QP→PIGEON CL-footprint trigger (~ 60+
 *            children depending on subtree shape) so a PIGEON →
 *            QP demotion lands in a QP that won't immediately
 *            re-fire the forward trigger.
 *   [NODE_INDEX_NULL] = FT_NULL sentinel — &ft_types[NODE_INDEX_NULL]
 *            is materialized by recompact when eliding a node, but
 *            never dereferenced.
 */
const struct cds_ft_type ft_types[] = {
	[FT_POPCOUNT_32_INDEX] = { .type_class = FT_POPCOUNT,
		.min_child = 1, .max_child = FT_PC32_MAX_LC_DIRECT,
		.min_child_skip = 1, .max_child_skip = FT_PC32_MAX_LC_SKIP,
		.order = FT_PC32_ALLOC_ORDER, .bitmap = FT_NO_BITMAP },
	/*
	 * FT_POPCOUNT_64 entry: scan_6 layout (5+3 byte split, single
	 * packed_bms u64 holding up to 6 sub_bms × 8 bits).  min_child
	 * overlaps with POPCOUNT_32's max_child (=3) for hysteresis;
	 * nr_child in [2,3] can stay in either class (the recompact
	 * framework prefers POPCOUNT_32 since the lattice walk lands
	 * there first).  Skip variant reserves slot 0 for skip-meta,
	 * dropping max_lc by one (5).
	 */
	[FT_POPCOUNT_64_INDEX] = { .type_class = FT_POPCOUNT,
		.min_child = 2, .max_child = FT_PC64_MAX_LC_DIRECT,
		.min_child_skip = 2, .max_child_skip = FT_PC64_MAX_LC_SKIP,
		.order = FT_PC64_ALLOC_ORDER, .bitmap = FT_NO_BITMAP },
	/*
	 * FT_QP entry: max_child uses the byte-count ceiling at QP T3
	 * (16 hi-buckets * 16 lo-children = 256) for find_nearest_type_index
	 * walks coming from PIGEON.  min_child = 2 gives a hysteresis
	 * window with POPCOUNT_32 (max=3): nr_child in [2,3] stays in
	 * the current class.
	 *
	 * Tier-up within QP (T0→T1→T2→T3) is bypassed in the lattice walk:
	 * see the FT_QP special-cases in ft_node_recompact's ADD_NEXT /
	 * ADD_SAME / DEL switches, which use ft_qp16_tiers[] / popcount
	 * directly.
	 *
	 * QP has no per-variant capacity difference: the SKIP_QP wrapper
	 * stores skip_len + subkey in spare bytes of the QP header, not
	 * by reserving a pointer slot.  Skip bounds therefore mirror the
	 * direct bounds.
	 */
	[FT_QP_INDEX] = { .type_class = FT_QP,
		.min_child = 2, .max_child = FT_QP16_T3_CAPACITY * 16,
		.min_child_skip = 2, .max_child_skip = FT_QP16_T3_CAPACITY * 16,
		.order = FT_QP16_T0_ALLOC_ORDER, .bitmap = FT_NO_BITMAP },
	/*
	 * PIGEON has no per-variant capacity difference either: skip
	 * length lives in metadata::pigeon_skip_len, no slot reserved.
	 */
	[FT_PIGEON_INDEX] = { .type_class = FT_PIGEON,
		.min_child = 24, .max_child = ft_type_pigeon_max_child,
		.min_child_skip = 24, .max_child_skip = ft_type_pigeon_max_child,
		.order = FT_PIGEON_ORDER, .bitmap = FT_BITMAP },
	/* NULL sentinel at NODE_INDEX_NULL (= FT_NUM_INTERNAL_TYPES). */
	[NODE_INDEX_NULL] = { .type_class = FT_NULL,
		.min_child = 0, .max_child = ft_type_null_max_child,
		.min_child_skip = 0, .max_child_skip = ft_type_null_max_child,
		.bitmap = FT_NO_BITMAP },
};

/*
 * The cds_ft_inode contains the compressed node data needed for
 * the read-side traversal.  Layout depends on the node's
 * type_class (QP-nibble or PIGEON); the struct uses a single
 * pointer-aligned byte array sized dynamically by the allocator
 * from the type's order (1 << type->order).
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
 * 1. FT_QP (qp16): see struct cds_ft_qp16_node — header (sub_bm[16]
 *    + nibble bitmap) followed by a packed pointer array.  Hi/lo
 *    pair encodes one byte step in two nibble levels.
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
	CAA_BUILD_BUG_ON(CAA_ARRAY_SIZE(ft_types) < NODE_INDEX_NULL + 1);
#ifdef FEATURE_FT_SKIP_COMPRESSED
	/*
	 * skip_slot_offset is 8 bits and stores byte_offset / sizeof(void *).
	 * Ensure the largest node (pigeon) fits 256 slots: PIGEON's 256
	 * dense entries scale with sizeof(void *) (FT_PIGEON_ORDER), so
	 * the quotient is exactly 256 on both 64-bit (2 KB / 8) and
	 * 32-bit (1 KB / 4).
	 */
	CAA_BUILD_BUG_ON((1U << FT_PIGEON_ORDER) / sizeof(void *) > 256);
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
	/*
	 * Clone a node into a freshly-allocated sibling of the same type,
	 * with all children copied and reparented to the new node.  No
	 * child is added or removed; old node is intact post-call and the
	 * caller is responsible for RCU-freeing it via *old_node_ret.
	 *
	 * Used to safely move a node across the COMPRESSED↔non-COMPRESSED
	 * parent-kind boundary: the CALLER places the cloned node under its
	 * new parent (potentially demoting parent kind), while the OLD node
	 * stays attached to its OLD COMPRESSED parent for the benefit of
	 * concurrent readers holding stale SKIP_X pointers.  Crucially, this
	 * mode does NOT update the upstream skip slot above the old parent
	 * (unlike ADD modes), since the cloned node will live at a different
	 * tree position than the original.
	 */
	FT_RECOMPACT_REPARENT,
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
	unsigned long tag;

	assert(type < FT_NUM_INTERNAL_TYPES);
	/*
	 * Map the ft_types[] slot index to the 5-bit kind tag.  All four
	 * QP hi-tiers (T0..T3) carry FT_KIND_QP uniformly; the per-tier
	 * alloc order is recoverable from cds_ft_item_order() when
	 * needed (write side / verify).  POPCOUNT_32 carries FT_KIND_-
	 * POPCOUNT_32 (direct variant; SKIP variant gets the SKIP tag
	 * via ft_skip_compressed_flag at publish time).  PIGEON carries
	 * FT_KIND_PIGEON.  ft_types[].type_class is a constant load —
	 * folded by the compiler when @type is a compile-time literal
	 * (fresh-alloc call sites pass 0).
	 */
	switch (ft_types[type].type_class) {
	case FT_POPCOUNT:
		switch (type) {
		case FT_POPCOUNT_32_INDEX:
			tag = FT_KIND_POPCOUNT_32;
			break;
		case FT_POPCOUNT_64_INDEX:
			tag = FT_KIND_POPCOUNT_64;
			break;
		default:
			assert(0);
			__builtin_unreachable();
		}
		break;
	case FT_QP:
		tag = FT_KIND_QP;
		break;
	case FT_PIGEON:
		tag = FT_KIND_PIGEON;
		break;
	default:
		assert(0);
		__builtin_unreachable();
	}
	return (struct cds_ft_inode_flag *) (((unsigned long) node) | tag);
}

/*
 * Test whether @node has the raw external tag (kind == FT_KIND_EXT,
 * low nibble = 0x0).  Strict form: rejects FT_KIND_SKIP_EXT.  This
 * matches both non-NULL external leaf pointers AND NULL, since NULL
 * has all bits clear.  Callers that need to distinguish NULL from a
 * valid external node should also check ft_node_ptr().
 *
 * Use this variant when you need to operate on the EXT pointer
 * directly (e.g. dereferencing the cds_ft_node leaf).  For the
 * "external-side" classification (EXT or SKIP_EXT), use the
 * direction-agnostic ft_node_external().
 */
static inline_lookup
bool ft_node_external_direct(struct cds_ft_inode_flag *node)
{
	/*
	 * Candidate E: EXT has bits 0-2 = 000.  Bit 4 of the underlying
	 * address can be set (16-byte alignment leaves it free), so mask
	 * with 0x07 instead of FT_KIND_MASK (0x1F) which would leak that
	 * address bit and misclassify EXT pointers whose addresses have
	 * bit 4 set.  Excludes COMPRESSED (bit 2), SKIP_EXT (bit 1),
	 * and all internal kinds (bit 0).
	 */
	return ((unsigned long) node & 0x07UL) == FT_KIND_EXT;
}

/*
 * Test whether @node has the SKIP_EXT tag (a skip-compressed pointer
 * resolving to an external leaf).  Strict form: rejects raw EXT.
 *
 * Use when the slot's encoding matters (e.g., when about to recover
 * the cn metadata via ft_skip_to_compressed).  Outside of
 * FEATURE_FT_SKIP_COMPRESSED builds, slots never carry the SKIP_EXT
 * tag and this predicate folds to constant false.
 */
static inline_lookup
bool ft_node_external_skip(struct cds_ft_inode_flag *node)
{
#ifdef FEATURE_FT_SKIP_COMPRESSED
	/*
	 * Candidate E: SKIP_EXT has bits 0-2 = 010 (bit 1 set, others
	 * clear).  Mask with 0x07 to avoid the bit-4 address leak —
	 * external-aligned (16B) values keep bit 4 as an address bit.
	 */
	return ((unsigned long) node & 0x07UL) == FT_KIND_SKIP_EXT;
#else
	(void) node;
	return false;
#endif
}

/*
 * Test whether @node is "external-side" — either a raw external leaf
 * (FT_KIND_EXT, including NULL) or a skip-compressed pointer that
 * resolves to one (FT_KIND_SKIP_EXT).  Bit-0 partition: bit 0 = 0
 * means external-side, bit 0 = 1 means internal-side.
 *
 * Use this variant when you only care about "this slot does not
 * resolve to a branching internal node" — e.g., end-of-descent
 * detection.  Callers that need to operate on the underlying EXT
 * pointer must additionally resolve SKIP_EXT (via
 * ft_resolve_skip_compressed) or use ft_node_external_direct() to
 * filter out the SKIP case.
 */
static inline_lookup
bool ft_node_external(struct cds_ft_inode_flag *node)
{
	/*
	 * Candidate E: external-side means EXT (0x00) or SKIP_EXT (0x02)
	 * — bit 0 clear AND bit 2 clear (excludes COMPRESSED 0x04 which
	 * also has bit 0 clear).  Single AND + CMP.  Bit 4 leaks for
	 * external-aligned values but does not change the bit-2 result.
	 */
	return ((unsigned long) node & 0x05UL) == 0;
}

#ifdef FEATURE_FT_COMPRESS
static inline_lookup
bool ft_node_compressed(struct cds_ft_inode_flag *node)
{
	/*
	 * Candidate E: COMPRESSED is 0x04 (bit 2 set, bits 0-1 clear).
	 * Mask with 0x07 — bit 4 of a 16-byte-aligned compressed
	 * pointer can be set (address bit) and would leak through a
	 * 5-bit mask.  This predicate is now context-agnostic: SKIP_X
	 * variants all have bit 1 set and fail the bit-1 part of the
	 * test, so the same body works in node-context (post-resolve)
	 * and slot-context (pre-resolve).
	 */
	return ((unsigned long) node & 0x07UL) == FT_KIND_COMPRESSED;
}
#else
static
bool ft_node_compressed(struct cds_ft_inode_flag *node __attribute__((unused)))
{
	return false;
}
#endif

/*
 * Direction-aware variant of ft_node_compressed for "node-context" call
 * sites: the input has already been resolved to a concrete node — it is
 * either a parent pointer pulled from a child's metadata, a value
 * returned from ft_resolve_skip_compressed, or a raw slot value in a
 * non-skip-compressed build where the COMPRESSED tag is unambiguous.
 *
 * In such contexts there is no SKIP_X tag to disambiguate, so this
 * predicate's interpretation is unconditional: "is the underlying node
 * a compressed-cluster node?".
 *
 * Currently aliases ft_node_compressed.  Once the 5-bit tag encoding
 * lands, COMPRESSED and SKIP_PIGEON share the bit pattern 0x03; the two
 * predicates will diverge — slot-context callers must use the SKIP-aware
 * variant, node-context callers stay on this one.
 */
static inline_lookup
bool ft_node_compressed_in_node(struct cds_ft_inode_flag *node)
{
	return ft_node_compressed(node);
}

/*
 * Direction-aware variant of ft_node_compressed for "slot-context" call
 * sites: the input is a raw value loaded from a parent's child slot
 * (or a value that may carry a SKIP_X tag — e.g. a publish_compressed
 * return value, ft->root, or an iter_path[0] entry sourced from the
 * root).  In such contexts a SKIP_X tag must NOT be reported as
 * "compressed", because the slot encodes a skip pointer through the
 * compressed cluster, not a raw COMPRESSED tag.
 *
 * Currently aliases ft_node_compressed.  Once the 5-bit tag encoding
 * lands, COMPRESSED and SKIP_PIGEON share the low-nibble bit pattern
 * 0x03; this slot-context variant gains an additional SKIP-bit
 * disambiguation step so SKIP_PIGEON is correctly reported as
 * non-compressed.  Under FEATURE_FT_SKIP_COMPRESSED the slot encoding
 * never carries a raw COMPRESSED tag (publish_compressed always wraps
 * in SKIP_X), so the predicate folds to constant false in that build.
 */
static inline_lookup
bool ft_node_compressed_in_slot(struct cds_ft_inode_flag *node)
{
	return ft_node_compressed(node);
}

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
	if (ft_node_compressed_in_node(node_flag)) {
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
 * This clears all tag and pool sub-index bits that sit below the
 * type's alignment boundary.  The shift amount is derived purely
 * from bits 1-3 with no dependency on bit 0.
 *
 * For non-internal nodes (bit 0 clear): external nodes are >= 8-byte
 * aligned (bits 0-2 zero), compressed/collapsed are >= 16-byte
 * aligned with tags in bits 1-2.  A fixed ~7UL mask suffices.
 *
 * The conditional select lets the two mask computations run in
 * parallel; the compiler emits a CMOV, keeping the critical path
 * to 4 cycles.
 */
/* Forward declarations for nr_keys helpers. */
static inline unsigned long ft_nr_keys_get(const struct cds_ft_metadata *m);
static inline unsigned long ft_nr_keys_load(const struct cds_ft_metadata *m);
static inline void ft_nr_keys_store(struct cds_ft *ft, struct cds_ft_metadata *m, unsigned long val, int mo);
static unsigned int ft_parent_depth_span(struct cds_ft_inode_flag *parent_nf,
		struct cds_ft_inode_flag *child_nf);
struct cds_ft_qp16_node;
static inline struct cds_ft_inode_flag *ft_qp16_lo_flag(
		struct cds_ft_qp16_node *lo);

static inline_lookup
struct cds_ft_inode *ft_node_ptr(struct cds_ft_inode_flag *node)
{
	/*
	 * Candidate E alignment-aware mask: bit 0 of the kind tag
	 * partitions the encoding by underlying alignment.  External-
	 * aligned kinds (EXT 0x00, SKIP_EXT 0x02, COMPRESSED 0x04 — all
	 * 16-byte aligned per __aligned__(16) on cds_ft_node and
	 * ft_compressed_order's >= 4 floor) have bit 0 = 0; their kind
	 * lives in bits 0-3 and bit 4 is an address bit, so a `~15UL`
	 * mask recovers the underlying pointer.
	 *
	 * Internal-aligned kinds (PIGEON 0x01, QP 0x05, the future
	 * POPCOUNT_* 0x09/0x11 — all 32-byte aligned per
	 * FT_QP16_T0_ALLOC_ORDER = 5 and FT_PIGEON_ORDER ≥ 10) have
	 * bit 0 = 1; the full 5-bit tag lives in bits 0-4 and `~31UL`
	 * recovers the pointer.
	 *
	 * Single bit-0 test + CMOV between the two masks; the
	 * compiler usually folds this into a 4-cycle dependency on
	 * the next-level load.  Caller-side fast variant
	 * ft_node_ptr_internal drops the test for known-internal
	 * inputs.
	 */
	unsigned long v = (unsigned long) node;
	unsigned long mask = ((v & 0x01UL) == 0) ? ~15UL : ~31UL;

	return (struct cds_ft_inode *) (v & mask);
}

/*
 * Lookup-hot variant: caller has already established that the
 * input is an internal-node flag (e.g. ft_node_get_nth_skip
 * filters tags < FT_KIND_QP and returns NULL before this
 * call).  Skips the (v & 1) ? ... : ~7UL branch in ft_node_ptr()
 * above, shaving the cmov/branch from the per-visit dependency
 * chain on the lookup hot path.
 */
static inline_lookup
struct cds_ft_inode *ft_node_ptr_internal(struct cds_ft_inode_flag *node)
{
	unsigned long v = (unsigned long) node;
	unsigned long mask = ~31UL;

	/*
	 * Candidate E: direct internal kinds have bit 0 set
	 * (FT_KIND_PIGEON 0x01, FT_KIND_QP 0x05, FT_KIND_POPCOUNT_*
	 * 0x09/0x11) and bit 1 clear (rejects SKIP_X).  Internal
	 * allocations are 32-byte aligned, so the 5-bit `~31UL` mask
	 * cleanly recovers the underlying pointer.  NULL slips through
	 * the assert because callers may legitimately pass an empty
	 * slot value here.
	 */
	assert((((v & 0x03UL) == 0x01UL)) || node == NULL);
	return (struct cds_ft_inode *) (v & mask);
}

static
struct cds_ft_inode *_ft_node_mask_ptr(struct cds_ft_inode_flag *node)
{
	return (struct cds_ft_inode *) ((unsigned long) node & FT_KIND_PTR_MASK);
}

static inline_lookup
bool ft_node_internal(struct cds_ft_inode_flag *node)
{
	/*
	 * Candidate E: direct internal kinds (PIGEON 0x01, QP 0x05, the
	 * future POPCOUNT_* 0x09/0x11) have bit 0 set AND bit 1 clear.
	 * Single AND + CMP; rejects EXT (bit 0 clear), COMPRESSED (bit 0
	 * clear), SKIP_X variants (bit 1 set).
	 *
	 * Callers that may have a SKIP_X-tagged input must resolve via
	 * ft_node_skip_compressed first; the bit-1 part of the test
	 * defends against accidental SKIP_X inputs but the assert
	 * documents the precondition.
	 */
	assert(((unsigned long) node & FT_KIND_SKIP_BIT) == 0);
	return ((unsigned long) node & 0x03UL) == 0x01UL;
}

/*
 * Predicate: does this tagged pointer point to a QP-class node,
 * either direct (FT_KIND_QP = 0x05) or skip-variant (FT_KIND_SKIP_QP
 * = 0x07)?  Used by up-walk / parent-tag dispatch sites that need to
 * accept both variants — the QP layout differs only in ptrs[] offset
 * (8 vs 16) and cached subkey reach, never in the underlying class.
 *
 * Mask is (FT_KIND_MASK & ~FT_KIND_SKIP_BIT) = 0x1F & ~0x02 = 0x1D:
 * clears the skip-bit before comparing the residual to FT_KIND_QP.
 * Predict-friendly: single AND + compare; no internal data-dependent
 * branch.  Equivalent expression for readability is to AND with the
 * mask explicitly: (tag & 0x1DUL) == FT_KIND_QP.
 */
static inline
bool ft_kind_is_qp_variant(unsigned long tag)
{
	return (tag & (FT_KIND_MASK & ~FT_KIND_SKIP_BIT)) == FT_KIND_QP;
}

/*
 * Recover the ft_types[] class index for an internal node flag.
 * Returns NODE_INDEX_NULL for a NULL pointer.  Asserts on COMPRESSED
 * (compressed nodes have no type-table slot) and on any non-internal
 * kind.
 *
 * One index per class: FT_KIND_POPCOUNT_32 → FT_POPCOUNT_32_INDEX,
 * FT_KIND_POPCOUNT_64 → FT_POPCOUNT_64_INDEX, FT_KIND_QP →
 * FT_QP_INDEX, FT_KIND_PIGEON → FT_PIGEON_INDEX.  QP's tier (T0..T3,
 * alloc orders 5..8) is recovered separately via cds_ft_item_order()
 * and indexed into ft_qp16_tiers[].  Descent dispatches on the kind
 * tag directly without indexing ft_types[].
 */
static inline_lookup
unsigned int ft_node_type_index(struct cds_ft_inode_flag *node)
{
	unsigned long tag;

	if (_ft_node_mask_ptr(node) == NULL)
		return NODE_INDEX_NULL;
	assert(!ft_node_compressed_in_node(node));

	tag = (unsigned long) node & FT_KIND_MASK_INTERNAL;
	/*
	 * Accept FT_KIND_SKIP_QP alongside FT_KIND_QP: the type index
	 * identifies the QP class, not the direct/skip layout (the
	 * layout is recovered separately from the slot tag at the call
	 * site).  Without this, a parent walk that lands on a
	 * SKIP_QP-tagged inode_flag would trip the trailing assert.
	 * SKIP_POPCOUNT_* / SKIP_PIGEON do not need the same treatment:
	 * the POPCOUNT skip variant uses a metadata bit
	 * (is_skip), and PIGEON has no skip-variant layout.
	 */
	if (ft_kind_is_qp_variant(tag))
		return FT_QP_INDEX;
	if (tag == FT_KIND_PIGEON)
		return FT_PIGEON_INDEX;
	if (tag == FT_KIND_POPCOUNT_32)
		return FT_POPCOUNT_32_INDEX;
	if (tag == FT_KIND_POPCOUNT_64)
		return FT_POPCOUNT_64_INDEX;
	assert(0);
	__builtin_unreachable();
}

/*
 * Recover the QP tier index (0..FT_QP16_NR_TIERS-1) for a QP-tagged
 * node flag.  Tier is encoded in the alloc order: T0 = order 5,
 * T1 = 6, T2 = 7, T3 = 8.  Caller must have established FT_KIND_QP.
 */
static inline_lookup
unsigned int ft_node_qp_tier(struct cds_ft_inode_flag *node)
{
	size_t order = cds_ft_item_order(ft_node_ptr_internal(node));

	assert(((unsigned long) node & FT_KIND_MASK_INTERNAL) == FT_KIND_QP);
	assert(order >= FT_QP16_T0_ALLOC_ORDER
		&& order < FT_QP16_T0_ALLOC_ORDER + FT_QP16_NR_TIERS);
	return (unsigned int) (order - FT_QP16_T0_ALLOC_ORDER);
}

static
struct cds_ft_inode_flag *ft_compressed_node_flag(
		struct cds_ft_compressed_node *node)
{
	return (struct cds_ft_inode_flag *)
		(((unsigned long) node) | (unsigned long) FT_KIND_COMPRESSED);
}

static inline_lookup
struct cds_ft_compressed_node *ft_compressed_node_ptr(
		struct cds_ft_inode_flag *node)
{
	/*
	 * Inverse of ft_compressed_node_flag (which OR's in
	 * FT_KIND_COMPRESSED = 0x04).  Subtract the known constant tag
	 * rather than masking — pointer arithmetic is recognized by the
	 * prefetcher's stride detector as a linear offset, while a mask
	 * confuses it.
	 *
	 * Candidate E: assert with the bit-aware predicate (`& 0x07`,
	 * not `& FT_KIND_MASK`) because FT_KIND_MASK = 0x1F leaks bit 4
	 * of the underlying 16-byte-aligned compressed address.
	 */
	assert(((unsigned long) node & 0x07UL) == FT_KIND_COMPRESSED);
	return (struct cds_ft_compressed_node *)
		((unsigned long) node - FT_KIND_COMPRESSED);
}

/* Skip-compressed pointer helpers. */

#ifdef FEATURE_FT_SKIP_COMPRESSED
static inline
bool ft_node_skip_compressed(struct cds_ft_inode_flag *node)
{
	/*
	 * FT_KIND_SKIP_BIT (bit 1) is the universal "is skip-compressed"
	 * predicate: set on FT_KIND_SKIP_EXT (0x2), FT_KIND_SKIP_QP
	 * (0x7), FT_KIND_SKIP_PIGEON (0xB); clear on every non-skip
	 * kind (EXT 0x0, COMPRESSED 0x1, QP 0x5, PIGEON 0x9).  One
	 * AND on the lookup hot path.
	 */
	return ((unsigned long) node & FT_KIND_SKIP_BIT) != 0;
}

/*
 * Direction-aware variant of ft_node_skip_compressed for "slot-context"
 * call sites: the input is the raw value loaded from a parent's child
 * slot during descent, where SKIP_X tags are the in-band marker that
 * the slot stores a skip-compressed pointer rather than a direct child.
 *
 * Currently aliases ft_node_skip_compressed.  Once the 5-bit tag
 * encoding lands, SKIP_PIGEON shares the bit pattern 0x03 with
 * COMPRESSED; this predicate's interpretation is "the slot stores a
 * SKIP_X — including SKIP_PIGEON 0x03", whereas a node-context
 * COMPRESSED check on the same value must report false.  Use this
 * variant in descent paths and any code that consults the tag of a
 * value sourced directly from a child slot (no ft_node_ptr / resolve
 * step in between).
 */
static inline
bool ft_node_skip_compressed_in_slot(struct cds_ft_inode_flag *node)
{
	return ft_node_skip_compressed(node);
}

/*
 * Recover the skip length for a skip-compressed pointer.
 *
 * SKIP_QP: skip_len lives inline in the QP node header (byte 2 of
 * cds_ft_qp16_node, same CL as the bitmap that descent already
 * loads).  No extra CL load.
 *
 * SKIP_PIGEON: PIGEON has no spare bytes in its dense ptrs[256]
 * layout, so skip_len lives in cds_ft_metadata::pigeon_skip_len.
 * One extra CL load (the metadata page).
 *
 * SKIP_POPCOUNT_32: POPCOUNT_32's direct-variant 8-byte header has
 * no spare bytes either; skip_len lives in
 * cds_ft_metadata::popcount_skip_len (aliases pigeon_skip_len in
 * the metadata union).  One extra CL load.
 *
 * SKIP_EXT: the resolved child can be user-allocated outside our
 * arenas, so we cannot rely on inline storage.  Length recovery
 * routes through ft_skip_to_compressed (the external_node->prev →
 * cn->len chain).
 */
static inline
struct cds_ft_compressed_node *ft_skip_to_compressed(
		struct cds_ft_inode_flag *skip_ptr);
static inline
unsigned int ft_skip_len(struct cds_ft_inode_flag *node)
{
	unsigned long v = (unsigned long) node;
	/*
	 * 5-bit mask: internal-aligned skip kinds (SKIP_PIGEON 0x03,
	 * SKIP_QP 0x07, SKIP_POPCOUNT_32 0x0B, SKIP_POPCOUNT_64 0x13)
	 * are 32-byte-aligned so bits 0..4 are clean kind bits.
	 * SKIP_EXT (0x02) is 16-byte-aligned, so the leaf-address bit 4
	 * may leak in giving 0x02 or 0x12; that is distinguishable from
	 * the internal-aligned variants by bit 0 (clear for SKIP_EXT,
	 * set for all internal-aligned skip kinds), and SKIP_EXT falls
	 * out of the cascade as the unmatched cold-path tail.
	 */
	unsigned long kind = v & 0x1FUL;
	void *natural;

	/*
	 * Caller has already established that @node is a skip-compressed
	 * kind.
	 */
	assert(kind == FT_KIND_SKIP_EXT || kind == (FT_KIND_SKIP_EXT | 0x10UL)
		|| kind == FT_KIND_SKIP_QP
		|| kind == FT_KIND_SKIP_PIGEON
		|| kind == FT_KIND_SKIP_POPCOUNT_32
		|| kind == FT_KIND_SKIP_POPCOUNT_64);
	/*
	 * Branch order optimized for the lookup hot path: SKIP_QP is the
	 * dominant input since the SKIP_EXT fast path in cds_ft_lookup
	 * never reaches here.  SKIP_POPCOUNT_32 / SKIP_POPCOUNT_64 /
	 * SKIP_PIGEON come next (small / medium / large dense subtries).
	 * SKIP_EXT falls through last and is only reached from
	 * off-hot-path callers (verify / show_stats).
	 *
	 * SUB by the constant tag inside each branch: lets the prefetcher
	 * recognize a constant-stride access and frees `kind` from being
	 * held live across the load.
	 */
	if (caa_likely(kind == FT_KIND_SKIP_QP)) {
		natural = (void *) (v - FT_KIND_SKIP_QP);
		return ((const struct cds_ft_qp16_node *) natural)->skip_len;
	}
	if (kind == FT_KIND_SKIP_POPCOUNT_32) {
		natural = (void *) (v - FT_KIND_SKIP_POPCOUNT_32);
		return cds_ft_item_to_metadata(natural)->popcount_skip_len;
	}
	if (kind == FT_KIND_SKIP_POPCOUNT_64) {
		natural = (void *) (v - FT_KIND_SKIP_POPCOUNT_64);
		return cds_ft_item_to_metadata(natural)->popcount_skip_len;
	}
	if (kind == FT_KIND_SKIP_PIGEON) {
		natural = (void *) (v - FT_KIND_SKIP_PIGEON);
		return cds_ft_item_to_metadata(natural)->pigeon_skip_len;
	}
	/* FT_KIND_SKIP_EXT — fall-through cold path. */
	return ft_skip_to_compressed(node)->len;
}

/*
 * Map a child kind tag (FT_KIND_EXT / FT_KIND_QP / FT_KIND_PIGEON /
 * FT_KIND_POPCOUNT_32) to the corresponding skip-target kind tag
 * (FT_KIND_SKIP_EXT / FT_KIND_SKIP_QP / FT_KIND_SKIP_PIGEON /
 * FT_KIND_SKIP_POPCOUNT_32).
 *
 * Skip-target kinds all have FT_KIND_SKIP_BIT (bit 1) set; the
 * conversion is a single OR (or, equivalently, ADD 0x2 since bit 1
 * is always clear in a child kind).  Asserted because the encoder
 * never receives a non-resolvable kind.
 */
static inline
unsigned long ft_kind_to_skip_kind(unsigned long child_kind)
{
	assert(child_kind == FT_KIND_EXT ||
	       child_kind == FT_KIND_QP ||
	       child_kind == FT_KIND_PIGEON ||
	       child_kind == FT_KIND_POPCOUNT_32 ||
	       child_kind == FT_KIND_POPCOUNT_64);
	return child_kind | FT_KIND_SKIP_BIT;
}

/*
 * Inverse of ft_kind_to_skip_kind: clear the skip bit to recover the
 * child kind.  Single-instruction `skip - 0x2` (or `skip & ~0x2`).
 */
static inline
unsigned long ft_skip_kind_to_child_kind(unsigned long skip_kind)
{
	assert(skip_kind == FT_KIND_SKIP_EXT ||
	       skip_kind == FT_KIND_SKIP_QP ||
	       skip_kind == FT_KIND_SKIP_PIGEON ||
	       skip_kind == FT_KIND_SKIP_POPCOUNT_32 ||
	       skip_kind == FT_KIND_SKIP_POPCOUNT_64);
	return skip_kind - FT_KIND_SKIP_BIT;
}

/*
 * ft_skip_child_ptr: extract the child tagged pointer from a skip
 * pointer.  With the encoding skip = child | FT_KIND_SKIP_BIT, the
 * child pointer is recovered by subtracting FT_KIND_SKIP_BIT (= 2)
 * from the skip pointer's low nibble.  We use pointer subtraction
 * rather than a mask-and-or because the prefetcher's stride detector
 * treats SUB on an address as a natural linear offset, while a mask
 * confuses the prediction.
 */
static inline
struct cds_ft_inode_flag *ft_skip_child_ptr(struct cds_ft_inode_flag *node)
{
	assert(((unsigned long) node & FT_KIND_SKIP_BIT) != 0);
	return (struct cds_ft_inode_flag *) ((unsigned long) node - FT_KIND_SKIP_BIT);
}

/*
 * ft_skip_compressed_flag: encode a skip pointer from a child pointer
 * and the parent compressed node @cn.  The child's kind tag is
 * rewritten to the matching FT_KIND_SKIP_* variant; readers can
 * dispatch on the skip-target class without consulting cn metadata.
 *
 * Length recovery and short-path validation use inline storage in
 * the destination node:
 *   FT_KIND_QP  → cds_ft_qp16_node::skip_len + subkey[5] in the
 *                    8B header CL (same CL as the bitmap descent
 *                    already loads).  Skip lengths ≤ 5 are validated
 *                    immediately during cand-mode descent against
 *                    subkey[]; longer paths defer to leaf bytes.
 *   FT_KIND_PIGEON → cds_ft_metadata::pigeon_skip_len (PIGEON has no
 *                    spare bytes in its dense ptrs[256] layout; no
 *                    subkey, validation always defers to leaf bytes).
 *   FT_KIND_EXT    → external_node->prev → cn->len chain (cn is
 *                    located via the back-pointer; no inline write,
 *                    since external nodes may live outside our
 *                    arenas).  Validation defers to leaf bytes.
 *
 * Side effect: writes the inline skip_len (and subkey for QP_HI) for
 * @child when @child is FT-arena-allocated.  Writes happen BEFORE
 * this function returns; the caller's subsequent rcu_assign_pointer
 * of the resulting flag provides the release ordering between the
 * inline write and any reader that observes the skip pointer.
 */
#ifdef FEATURE_FT_SKIP_COMPRESSED
/*
 * Forward declarations for QP-skip Phase 2 helpers used below.  Full
 * definitions live near the QP read-side helpers / allocator wrappers.
 */
static inline
struct cds_ft_inode_flag **
ft_qp16_ptrs(struct cds_ft_qp16_node *node, bool is_skip);
static inline
struct ft_qp16_skip_meta *
ft_qp16_skip_meta_at(struct cds_ft_qp16_node *node);
static inline
unsigned int ft_qp16_alloc_order_skip(unsigned int popcount);
static __attribute__((unused))
struct cds_ft_qp16_node *ft_qp16_node_alloc(struct cds_ft *ft,
		unsigned int order,
		struct cds_ft_metadata **meta_p);
static __attribute__((unused))
void ft_qp16_node_free_rcu(struct cds_ft *ft, struct cds_ft_qp16_node *node);
#endif

static
struct cds_ft_inode_flag *ft_skip_compressed_flag(
		struct cds_ft_inode_flag *child,
		const struct cds_ft_compressed_node *cn)
{
	/*
	 * Candidate E: child is one of EXT (0x00, 16B-aligned), PIGEON
	 * (0x01, 32B), QP (0x05, 32B), POPCOUNT_32 (0x09, 32B), or
	 * POPCOUNT_64 (0x11, 32B).  Internal-aligned kinds (bit 0 set)
	 * are 32-byte aligned so all 5 low bits are clean kind bits.
	 * EXT (bit 0 clear) is 16-byte aligned and bit 4 of its address
	 * may leak under a 5-bit mask — force child_kind to FT_KIND_EXT
	 * for that case so the SUB encoding below preserves bit 4 of
	 * the underlying address.
	 */
	unsigned long v = (unsigned long) child;
	unsigned long child_kind = (v & 0x01UL) ? (v & 0x1FUL) : FT_KIND_EXT;
	unsigned long skip_kind = ft_kind_to_skip_kind(child_kind);
	unsigned int len = cn->len;

	/* Bounded by FT_SKIP_LEN_MAX (uint8_t skip_len field). */
	assert(len > 0 && len <= FT_SKIP_LEN_MAX);
	/*
	 * Only EXT / QP / PIGEON / POPCOUNT_32 / POPCOUNT_64 are valid
	 * skip targets.  COMPRESSED is forbidden by the chain-compress
	 * invariant; ft_kind_to_skip_kind asserts.  (QP-nibble lo-nodes
	 * share the QP tag and are not a separate kind in the slot-tag
	 * space.)
	 */
	if (child_kind == FT_KIND_QP) {
		/* QP is 32-byte aligned: ~31UL strips kind cleanly. */
		struct cds_ft_qp16_node *qp = (struct cds_ft_qp16_node *)
			(v & ~31UL);
		/*
		 * Write up to FT_QP16_SUBKEY_INLINE_LEN (= 13) cached bytes
		 * via the skip-meta view (skip_len at byte 2, subkey[13] at
		 * bytes 3-15).  Bytes 8-15 of the QP body overlay what would
		 * otherwise be ptrs[0] in the direct layout — the caller
		 * (ft_publish_compressed) migrates the QP to a skip-variant
		 * allocation before reaching here so that ptrs[] starts at
		 * byte 16 and the overlay region is dedicated to the cached
		 * subkey.  metadata->is_skip on the qp is the authoritative
		 * signal that the overlay is safe to write.
		 */
		struct ft_qp16_skip_meta *meta = ft_qp16_skip_meta_at(qp);
		size_t copy_len = len < FT_QP16_SUBKEY_INLINE_LEN
				? len : FT_QP16_SUBKEY_INLINE_LEN;

		if (copy_len)
			memcpy(meta->subkey, cn->key_bytes, copy_len);
		meta->skip_len = (uint8_t) len;
	} else if (child_kind == FT_KIND_PIGEON) {
		/* PIGEON is 32-byte aligned: ~31UL strips kind cleanly. */
		void *natural_child = (void *) (v & ~31UL);
		struct cds_ft_metadata *meta =
			cds_ft_item_to_metadata(natural_child);

		meta->pigeon_skip_len = (uint8_t) len;
	} else if (child_kind == FT_KIND_POPCOUNT_32
			|| child_kind == FT_KIND_POPCOUNT_64) {
		/*
		 * POPCOUNT_32 and POPCOUNT_64 share the metadata
		 * popcount_skip_len field; both are 32-byte aligned
		 * (~31UL strips the kind cleanly).
		 */
		void *natural_child = (void *) (v & ~31UL);
		struct cds_ft_metadata *meta =
			cds_ft_item_to_metadata(natural_child);

		meta->popcount_skip_len = (uint8_t) len;
	}
	/*
	 * Tag-encode by SUB of the child's tag value rather than masking
	 * with FT_KIND_PTR_MASK (= ~0x1F).  Under Candidate E,
	 * 16-byte-aligned EXT pointers may legitimately have bit 4 set
	 * as an address bit; ~0x1F would clear it and corrupt the
	 * underlying pointer.  SUB by the kind value (0x00 for EXT,
	 * 0x01 for PIGEON, 0x05 for QP, 0x09 for POPCOUNT_32, 0x11 for
	 * POPCOUNT_64) preserves bits 4+ of the underlying address
	 * regardless of alignment.  The skip-tag is OR'd in at the end
	 * since bit 1 (FT_KIND_SKIP_BIT) was clear in every child kind.
	 */
	return (struct cds_ft_inode_flag *) ((v - child_kind) | skip_kind);
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

	if (ft_node_external_direct(child))
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

	if (ft_node_external_direct(node))
		parent = rcu_dereference(((struct cds_ft_node *) node)->prev);
	else
		parent = rcu_dereference(cds_ft_item_to_metadata(
			ft_node_ptr(node))->parent);
	assert(!parent || !ft_node_external_direct(parent));
	return parent;
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
bool ft_node_skip_compressed_in_slot(struct cds_ft_inode_flag *node)
{
	return ft_node_skip_compressed(node);
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
		const struct cds_ft_compressed_node *cn __attribute__((unused)))
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
	if (ft_node_skip_compressed_in_slot(nf)) {
		struct cds_ft_compressed_node *cn =
			ft_skip_to_compressed(nf);
		return cds_ft_item_to_metadata(
			(struct cds_ft_inode *) cn);
	}
#endif
	return cds_ft_item_to_metadata(ft_node_ptr(nf));
}

/*
 * ft_flag_to_metadata_fast: variant for callers that know @nf is an
 * internal node (qp_hi / qp_lo / pigeon).  Bypasses the skip-compressed
 * branch in ft_flag_to_metadata.
 *
 * TODO: post-tag-bit refactor (Stage F) the legacy fast path —
 * deriving alloc order from the tag's type-index — is gone, so this
 * helper now falls through to cds_ft_item_to_metadata which loads
 * range->arena->item_len_order (2 dependent loads).  This regresses
 * the metadata-prefetch sites that used to amortize the order load
 * via tag-bit math.  Repair this before relying on the fast path for
 * the skip-compressed-descent length recovery (cn->len read on a
 * SKIP_* tag), since that path needs a single-load metadata fetch to
 * stay in budget.  Candidate fix: encode item_len_order in a small
 * per-page header byte so it is reachable without the
 * range->arena pointer chase.
 */
static inline
struct cds_ft_metadata *ft_flag_to_metadata_fast(struct cds_ft_inode_flag *nf)
{
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
	if (ft_node_skip_compressed_in_slot(nf))
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
	if (!ft_node_skip_compressed_in_slot(slot_val))
		return;
	rcu_assign_pointer(*parent_slot,
		ft_skip_compressed_flag(cn->child, cn));
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
	 * Both encodings (SKIP_X and plain COMPRESSED) are handled: a
	 * plain-COMPRESSED publish (ft_publish_compressed gate hit on
	 * non-spec EXT child) still populates skip_slot_offset so a later
	 * cn → non-EXT child transition (which would re-encode the slot
	 * as SKIP_X via the dual-pointer dance) can update the right
	 * grandparent slot.
	 *
	 * We do NOT touch @new_child's parent linkage here; existing
	 * callers manage that via their own ft_set_parent (or by direct
	 * meta->parent assignment) before calling us, with semantics that
	 * vary across call sites.  This keeps the bookkeeping fix narrow.
	 *
	 * Skip the update at the root slot (&ft->root): root nodes have
	 * no parent, and ft_set_skip_slot's offset computation assumes
	 * the slot lives inside a node-arena chunk.
	 */
	if (parent_slot != &ft->root && parent_nf) {
		struct cds_ft_compressed_node *cn = NULL;

		if (ft_node_skip_compressed_in_slot(new_child))
			cn = ft_skip_to_compressed(new_child);
		else if (ft_node_compressed_in_node(new_child))
			cn = ft_compressed_node_ptr(new_child);
		if (cn) {
			struct cds_ft_metadata *cn_meta =
				cds_ft_item_to_metadata(
					(struct cds_ft_inode *) cn);

			/*
			 * Compute the offset directly from @parent_nf rather
			 * than via cn_meta->parent.  Some callers publish the
			 * cn before calling ft_set_parent on the cn (or assign
			 * cn_meta->parent themselves), so cn_meta->parent may
			 * still be NULL at this point.  parent_nf is already
			 * the correct grandparent for the published slot.
			 *
			 * QP HI/LO rebase: byte-keyed children of a QP hi-node
			 * carry the tagged lo-pointer as their parent (slot is
			 * in the lo arena, not in the hi node).  Mirror the
			 * rebase in ft_set_parent so skip_slot_offset is bounded
			 * by the lo's alloc size and matches the value
			 * cn_meta->parent will eventually hold once
			 * ft_set_parent (called by the caller) rebases to the
			 * lo-flag.  Without this, the offset is computed
			 * against the hi base while cn_meta->parent ends up at
			 * the lo flag — ft_get_skip_slot then resolves the
			 * stored offset against the lo, returning a slot in
			 * an unrelated arena range.
			 */
			{
				struct cds_ft_inode_flag *eff_parent_nf = parent_nf;

				/*
				 * Match both FT_KIND_QP (direct) and
				 * FT_KIND_SKIP_QP (skip variant): the
				 * HI/LO rebase is about disambiguating
				 * the slot's containing allocation, not
				 * about the variant.  Rebasing to LO
				 * always yields an FT_KIND_QP-tagged
				 * flag since LO is never skip-variant.
				 */
				if (ft_kind_is_qp_variant((unsigned long) parent_nf)) {
					void *p_addr = ft_node_ptr(parent_nf);
					size_t lo_order =
						cds_ft_item_order(parent_slot);
					void *lo_base = (void *)
						((unsigned long) parent_slot
						 & ~((1UL << lo_order) - 1UL));

					if (lo_base != p_addr)
						eff_parent_nf = ft_qp16_lo_flag(
							(struct cds_ft_qp16_node *)
								lo_base);
				}
				cn_meta->skip_slot_offset = (unsigned int)
					((char *) parent_slot -
					 (char *) ft_node_ptr(eff_parent_nf))
					/ sizeof(void *);
			}
		}
	}
#endif

	if (parent_nf && ft_node_compressed_in_node(parent_nf)) {
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
			    ft_node_skip_compressed_in_slot(*skip_slot)) {
				struct cds_ft_inode_flag *new_skip_val;

				/*
				 * SKIP_EXT gating: in non-spec-validate groups,
				 * cn → EXT chains use plain COMPRESSED in the
				 * upstream slot, not SKIP_EXT.  When new_child
				 * transitions to EXT in such a group, demote
				 * the slot from SKIP_X (set up earlier when
				 * cn->child was non-EXT) back to plain
				 * COMPRESSED.  See ft_publish_compressed for
				 * the matching creation-time gate.
				 */
				if (ft_node_external_direct(new_child) &&
				    !ft->group->speculative_validated)
					new_skip_val =
						ft_compressed_node_flag(cn);
				else
					new_skip_val =
						ft_skip_compressed_flag(
							new_child, cn);
				rcu_assign_pointer(*skip_slot, new_skip_val);
			}
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

#ifdef FEATURE_FT_SKIP_COMPRESSED
/*
 * QP-skip Phase 2 migration: when ft_publish_compressed wraps a
 * chain-compressed cn around a direct-variant QP HI child, allocate a
 * fresh skip-variant HI at the appropriate tier, copy children into
 * the ptrs-at-byte-16 layout, reparent LO children's meta->parent to
 * the new HI (SKIP_QP-tagged), atomic-swap cn->child, and RCU-free
 * the old direct HI.  The new HI's metadata::is_skip is set to 1 so
 * subsequent mutators that descend through cn->child (FT_KIND_QP-
 * tagged, since cn->child does not carry the skip tag) still pick the
 * correct layout via the metadata bit.
 *
 * Returns true on success.  Returns false when:
 *   - popcount > FT_QP16_T3_CAPACITY_SKIP: no skip-variant tier holds
 *     the children (the overlay drops capacity by one per tier).
 *     Rare on chain-compress targets (those are typically degenerate
 *     single-child paths), but possible.
 *   - allocator returns NULL.
 * The caller interprets false as a signal to fall back to plain
 * COMPRESSED publication (no SKIP_QP wrapper); descent then walks
 * cn byte-by-byte to the still-direct QP.
 *
 * cn is not yet attached to the tree at this point (the caller has
 * just constructed it and is about to publish), so the cn->child swap
 * is invisible to readers — the new HI becomes reader-visible only
 * when the caller subsequently publishes the SKIP_QP wrapper in the
 * grandparent slot.  rcu_assign_pointer on cn->child still provides
 * the release fence for the migration's stores.
 */
static
bool ft_qp_migrate_to_skip_variant(struct cds_ft *ft,
		struct cds_ft_compressed_node *cn)
{
	struct cds_ft_qp16_node *old_hi = (struct cds_ft_qp16_node *)
		((unsigned long) cn->child & ~31UL);
	struct cds_ft_metadata *old_meta = cds_ft_item_to_metadata(old_hi);
	uint16_t old_bm = uatomic_load(&old_hi->bitmap, CMM_RELAXED);
	unsigned int pop = (unsigned int)
		__builtin_popcount((unsigned int) old_bm);
	unsigned int new_order;
	struct cds_ft_metadata *new_meta;
	struct cds_ft_qp16_node *new_hi;
	struct cds_ft_inode_flag **old_ptrs;
	struct cds_ft_inode_flag **new_ptrs;
	struct cds_ft_inode_flag *new_hi_skip_flag;
	unsigned int b;

	/*
	 * Chain-compress re-publish over an already-skip qp HI:
	 * compressed-split / chain-merge can build a fresh cn whose
	 * child slot already points to a skip-variant qp HI (migrated
	 * by an earlier publish under the now-replaced cn).  Treat
	 * this as a no-op migration — the qp is already in the right
	 * layout and the caller's subsequent ft_skip_compressed_flag
	 * will refresh the cached subkey to match the new cn.
	 * Mirrors the "refresh slot 0 in both the initial transition
	 * AND on chain-compress re-publish" pattern the POPCOUNT_32 /
	 * POPCOUNT_64 arms already implement.
	 *
	 * Reading old_hi->ptrs[] under the direct layout (offset 8)
	 * for an already-skip qp returns subkey bytes — caller would
	 * then dereference those bytes as LO pointers and crash.
	 */
	if (old_meta->is_skip)
		return true;
	if (pop > FT_QP16_T3_CAPACITY_SKIP)
		return false;
	/*
	 * No cap on cn->len: the SKIP_QP wrapper is still worth publishing
	 * when cn->len > FT_QP16_SUBKEY_INLINE_LEN because cand-mode
	 * descent (spec_validate) handles the long-subkey case via
	 * needs_leaf_validate + first_skip_offset (deferred end-of-descent
	 * leaf-bytes compare).  Non-cand descent splits the validation:
	 * first FT_QP16_SUBKEY_INLINE_LEN bytes against the qp's cached
	 * subkey (hot, on the qp body), tail bytes recovered from
	 * cn->key_bytes via ft_skip_to_compressed (cold, but only fires
	 * for cn->len > 13 in non-cand mode).
	 */
	new_order = ft_qp16_alloc_order_skip(pop);
	new_hi = ft_qp16_node_alloc(ft, new_order, &new_meta);
	if (!new_hi)
		return false;

	/* Copy bitmap; ptrs from byte 8 (direct) to byte 16 (skip). */
	new_hi->bitmap = old_bm;
	old_ptrs = ft_qp16_ptrs(old_hi, false);
	new_ptrs = ft_qp16_ptrs(new_hi, true);
	for (b = 0; b < pop; b++)
		new_ptrs[b] = old_ptrs[b];

	/*
	 * Preserve metadata fields the migration cannot reconstruct:
	 *   nr_child, nr_keys, parent — same subtree semantics.
	 *   qp_subtree_half_cls — the lo-subtree footprint is unchanged
	 *     (we did not touch LOs), so the accumulated total stays.
	 *   is_lo — 0 (HI).
	 *   is_skip — 1 (new node is skip-variant).
	 * fallback_removal_count / alloc_index / skip_slot_offset are
	 * either reset by ft_qp16_node_alloc or re-derived by
	 * ft_set_parent / ft_set_skip_slot in the publish path.
	 */
	new_meta->nr_child = old_meta->nr_child;
	new_meta->qp_subtree_half_cls = old_meta->qp_subtree_half_cls;
	new_meta->parent = old_meta->parent;
	uatomic_store(&new_meta->nr_keys,
		uatomic_load(&old_meta->nr_keys, CMM_RELAXED),
		CMM_RELAXED);
	new_meta->is_lo = 0;
	new_meta->is_skip = 1;

	/*
	 * Reparent each populated LO child's meta->parent to point at
	 * the new HI with the SKIP_QP slot-tag.  Children walking up via
	 * meta->parent then discover the variant directly from the tag
	 * (faster than the metadata bit on the up-walk hot path).
	 */
	new_hi_skip_flag = (struct cds_ft_inode_flag *)
		((unsigned long) new_hi | FT_KIND_SKIP_QP);
	for (b = 0; b < 16U; b++) {
		unsigned int idx;
		struct cds_ft_qp16_node *lo;
		struct cds_ft_metadata *lo_meta;

		if (!(old_bm & (uint16_t) (1U << b)))
			continue;
		idx = (unsigned int) __builtin_popcount(
				(unsigned int) (old_bm & ((1U << b) - 1U)));
		lo = (struct cds_ft_qp16_node *) new_ptrs[idx];
		if (!lo)
			continue;
		lo_meta = cds_ft_item_to_metadata(lo);
		rcu_assign_pointer(lo_meta->parent, new_hi_skip_flag);
	}

	/*
	 * Atomic-swap cn->child to the new HI (FT_KIND_QP-tagged, since
	 * the SKIP_QP wrap lives in the grandparent slot — applied by
	 * ft_skip_compressed_flag after we return).  Then RCU-free the
	 * old direct HI.
	 */
	rcu_assign_pointer(cn->child, (struct cds_ft_inode_flag *)
			((unsigned long) new_hi | FT_KIND_QP));
	ft_qp16_node_free_rcu(ft, old_hi);
	return true;
}
#endif /* FEATURE_FT_SKIP_COMPRESSED */

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
#ifdef FEATURE_FT_SKIP_COMPRESSED
	/*
	 * SKIP_EXT is gated on speculative-validate: only in spec-validate
	 * groups can the read path recover the skipped compressed node's
	 * length without dereferencing ext->prev (which is mutator-only,
	 * via the EXT's stored key at @speculative_key_offset).  In non-
	 * spec-validate groups, leave cn → EXT chains as plain COMPRESSED
	 * in the parent slot so the read path descends through cn and
	 * recovers cn->len directly.  SKIP_QP and SKIP_PIGEON remain
	 * unconditional — their length lives inline in the resolved
	 * target (qp->skip_len / pigeon_skip_len).
	 */
	if (ft_node_external_direct(cn->child) &&
	    !ft->group->speculative_validated)
		return cflag;
	/*
	 * POPCOUNT_{32,64} skip-variant transition: when cn->child is a
	 * POPCOUNT node with nr_child ≤ MAX_LC_SKIP for that layout,
	 * repurpose slot 0 to hold ft_pc32_skip_meta (skip_len + cached
	 * subkey) so speculative-validate descent can validate the
	 * skipped path inline against the cached subkey, no parent walk
	 * to cn required.  Set is_skip on the child's metadata
	 * so subsequent set_nth uses the reduced max_lc.  Slot 0
	 * (highest popcount-rank) is guaranteed unused at this point
	 * because nr_child ≤ MAX_LC_SKIP — write is non-destructive.
	 *
	 * For nr_child > MAX_LC_SKIP, the slot-0 reservation would
	 * overwrite a live pointer; fall back to the SKIP_POPCOUNT_*
	 * wrapper without slot-0 caching (skip_len still in
	 * popcount_skip_len metadata, validation defers to leaf-bytes).
	 *
	 * pc32 and pc64 share struct ft_pc32_skip_meta as the slot-0
	 * cached-subkey form; layout differs only in the byte offset
	 * of slot 0 inside the node (8 vs 16) which the union access
	 * handles transparently.
	 */
	{
		unsigned long child_kind =
			(unsigned long) cn->child & 0x1FUL;

		if (child_kind == FT_KIND_POPCOUNT_32) {
			struct ft_pc32_node *pc = (struct ft_pc32_node *)
				((unsigned long) cn->child & ~31UL);
			struct cds_ft_metadata *pc_meta =
				cds_ft_item_to_metadata(pc);

			/*
			 * Refresh slot 0 in both the initial transition
			 * (is_skip == 0) and on chain-compress
			 * re-publish over an already-skip popcount: a new cn
			 * wrapping the same popcount with different key bytes
			 * (compressed-split shortening, chain-merge) must
			 * overwrite the cached subkey/skip_len so spec-
			 * validate descent compares against the new cn's
			 * bytes.  The nr_child <= MAX_LC_SKIP guard ensures
			 * slot 0 is not allocated to a child via the reverse-
			 * indexed layout, so the write is non-destructive
			 * regardless of the current is_skip state.
			 */
			if (pc_meta->nr_child <= FT_PC32_MAX_LC_SKIP) {
				struct ft_pc32_skip_meta *meta = &pc->u.skip.meta;
				size_t copy_len = cn->len < sizeof(meta->subkey)
					? cn->len
					: sizeof(meta->subkey);

				memset(meta, 0, sizeof(*meta));
				meta->skip_len = (uint8_t) cn->len;
				if (copy_len)
					memcpy(meta->subkey, cn->key_bytes, copy_len);
				pc_meta->is_skip = 1;
			}
		} else if (child_kind == FT_KIND_POPCOUNT_64) {
			struct ft_pc64_node *pc = (struct ft_pc64_node *)
				((unsigned long) cn->child & ~31UL);
			struct cds_ft_metadata *pc_meta =
				cds_ft_item_to_metadata(pc);

			if (pc_meta->nr_child <= FT_PC64_MAX_LC_SKIP) {
				struct ft_pc32_skip_meta *meta = &pc->u.skip.meta;
				size_t copy_len = cn->len < sizeof(meta->subkey)
					? cn->len
					: sizeof(meta->subkey);

				memset(meta, 0, sizeof(*meta));
				meta->skip_len = (uint8_t) cn->len;
				if (copy_len)
					memcpy(meta->subkey, cn->key_bytes, copy_len);
				pc_meta->is_skip = 1;
			}
		} else if (child_kind == FT_KIND_QP) {
			/*
			 * QP-skip Phase 2 migration: replace the direct QP
			 * HI under cn with a skip-variant HI (ptrs at byte
			 * 16, subkey extension at bytes 8-15) so the
			 * subsequent ft_skip_compressed_flag below can
			 * write a 13B cached subkey into the overlay
			 * region without colliding with a live ptr.  If
			 * the migration cannot proceed (popcount too high
			 * for any skip tier, or allocator failure), fall
			 * back to publishing plain COMPRESSED — the
			 * SKIP_QP wrapper is bypassed and descent walks
			 * the cn byte-by-byte to the still-direct QP.
			 */
			if (!ft_qp_migrate_to_skip_variant(ft, cn))
				return cflag;
		}
	}
	return ft_skip_compressed_flag(cn->child, cn);
#else
	return cflag;
#endif
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
 * ft_node_external_direct on the raw value.
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
	/*
	 * Option 3: byte-keyed children of a QP hi-node carry the tagged
	 * lo-pointer as their parent (slot is in the lo arena, not in
	 * the hi node).  When callers pass the hi-flag with a lo-side
	 * @slot — the convention used by every byte-step insert/replace
	 * path — rebase parent_nf to the lo-arena base derived from the
	 * slot's containing allocation so that meta->parent's address is
	 * the lo and ft_set_skip_slot's 8-bit offset is bounded by the
	 * lo's alloc size.
	 *
	 * The tag stays FT_KIND_QP (HI and LO share one slot-tag kind
	 * since the QP_LO retirement); the parent walk recovers HI vs LO
	 * via the lo-node's metadata is_lo bit.
	 */
	/*
	 * Match both FT_KIND_QP (direct) and FT_KIND_SKIP_QP (skip
	 * variant).  The rebase to LO produces an FT_KIND_QP-tagged
	 * flag — LO is never skip-variant.  Callers store the resulting
	 * tag verbatim into meta->parent.
	 */
	if (slot && parent_nf
	    && ft_kind_is_qp_variant((unsigned long) parent_nf)) {
		void *p_addr = ft_node_ptr(parent_nf);
		size_t lo_order = cds_ft_item_order(slot);
		void *lo_base = (void *) ((unsigned long) slot
			& ~((1UL << lo_order) - 1UL));

		if (lo_base != p_addr) {
			parent_nf = ft_qp16_lo_flag(
				(struct cds_ft_qp16_node *) lo_base);
		}
	}
	FT_TP(set_parent, (const void *) child_nf, (const void *) parent_nf);
#ifdef FEATURE_FT_SKIP_COMPRESSED
	if (ft_node_skip_compressed_in_slot(child_nf)) {
		struct cds_ft_compressed_node *cn =
			ft_skip_to_compressed(child_nf);
		struct cds_ft_metadata *cn_meta =
			cds_ft_item_to_metadata(
				(struct cds_ft_inode *) cn);
		rcu_assign_pointer(cn_meta->parent, parent_nf);
		ft_set_skip_slot(cn_meta, slot);
		return;
	}
	if (ft_node_compressed_in_node(child_nf)) {
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
	if (ft_node_external_direct(child_nf)) {
		rcu_assign_pointer(
			((struct cds_ft_node *) child_nf)->prev,
			parent_nf);
		return;
	}
	{
		struct cds_ft_metadata *child_meta =
			cds_ft_item_to_metadata(ft_node_ptr(child_nf));

		rcu_assign_pointer(child_meta->parent, parent_nf);
	}
}


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
	return node != NULL && ft_node_external_direct((struct cds_ft_inode_flag *) node);
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
	/*
	 * Resolve a SKIP-encoded root once at descent start.  ft->root is
	 * the only pointer in the trie that's read raw without going
	 * through ft_node_get_nth (which resolves), so a compressed root
	 * (stored as SKIP_X via ft_publish_compressed) would otherwise
	 * leave d.nf SKIP-tagged.  Resolving here normalises d.nf to
	 * post-resolve form and lets the descent loop drop its redundant
	 * top-of-iteration resolves.
	 *
	 * Under the 4-bit encoding this is semantics-preserving — the
	 * loop-top resolves were no-ops on already-resolved values.
	 * Under the planned 5-bit encoding (where COMPRESSED 0x03 and
	 * SKIP_PIGEON 0x03 share bit patterns), this restructuring is
	 * required: the loop-top resolves would otherwise erroneously
	 * fire on COMPRESSED-tagged values left by ft_descent_step.
	 */
	d->nf = ft_resolve_skip_compressed(d->nf);
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
	/*
	 * cn->child can be SKIP-encoded (e.g. when the compressed cluster
	 * was built over a child that was itself wrapped via
	 * ft_skip_compressed_flag).  Resolve here so d.nf at the descent
	 * loop's top is always post-resolve, matching the contract
	 * established by ft_descent_init and ft_descent_step.  Without
	 * this, the surrounding loop has to call ft_resolve_skip_compressed
	 * itself — which under the planned 5-bit tag encoding would
	 * erroneously fire on COMPRESSED-tagged values returned by
	 * ft_descent_step (because COMPRESSED and SKIP_PIGEON share the
	 * bit pattern 0x03 and the bit-1 SKIP test no longer
	 * distinguishes them).
	 */
	d->nf = ft_resolve_skip_compressed(d->nf);
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
struct cds_ft_inode *alloc_cds_ft_node_at_order(struct cds_ft *ft,
		unsigned int order, bool bitmap,
		struct cds_ft_metadata **_metadata)
{
	struct cds_ft_metadata *metadata;
	void *p;

	metadata = cds_ft_alloc_item(ft, order, bitmap);
	if (!metadata) {
		return NULL;
	}
	p = cds_ft_metadata_to_item(metadata);
	if (ft_debug_counters()) {
		uatomic_inc(&ft->nr_nodes_allocated);
		uatomic_inc(&ft->nr_internal_alloc);
	}
	*_metadata = metadata;
	return p;
}

static
struct cds_ft_inode *alloc_cds_ft_node(struct cds_ft *ft,
		const struct cds_ft_type *ft_type,
		struct cds_ft_metadata **_metadata)
{
	return alloc_cds_ft_node_at_order(ft, ft_type->order,
			ft_type->bitmap, _metadata);
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
 * - FT_PIGEON / SKIP_PIGEON: dense `pointers[256]`; the scanner reads
 *   `pointers[n]` at offset n*8.  First cache line covers only
 *   pointers[0..7], ~3% of random keys.
 */
static inline void ft_maybe_prefetch(const void *ptr)
{
	unsigned long v = (unsigned long) ptr;
	unsigned long kind = v & FT_KIND_MASK;

	if (kind == FT_KIND_PIGEON || kind == FT_KIND_SKIP_PIGEON)
		return;
	__builtin_prefetch((const void *) v);
}

/*
 * ft_dereference_prefetch: for tagged FT node pointers (may have
 * pool/pigeon subclass bits).  Uses ft_maybe_prefetch to skip
 * pool/pigeon types.
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
 *                      compressed / collapsed children, skips —
 *                      their own handlers prefetch cn->child /
 *                      col->data.
 *   FT_PF_BITMAP_META: same as META plus prefetches the bitmap
 *                      cache line for pigeon children (used by
 *                      ordered get_direction traversal).
 *
 * The item's alloc order is recovered via cds_ft_item_order(node),
 * which reads the arena range header at the page boundary; the node
 * base address is derived from the tagged pointer via FT_KIND_PTR_MASK
 * (low 4 bits hold the kind tag; allocations are >= 16-byte aligned).
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
	 * Candidate E dispatch: bit 0 partitions external-aligned
	 * (EXT 0x00, SKIP_EXT 0x02, COMPRESSED 0x04 — bit 0 clear)
	 * from internal-aligned (PIGEON 0x01, QP 0x05, SKIP_PIGEON
	 * 0x03, SKIP_QP 0x07, POPCOUNT_* 0x09/0x11/... — bit 0 set).
	 * Two-test sequential dispatch, well-predicted on most paths.
	 */
	if (!(v & 1)) {
		/*
		 * External-aligned half.  Bit 2 distinguishes COMPRESSED
		 * (0x04) from EXT/SKIP_EXT.  16-byte alignment leaves bit
		 * 4 as an address bit, but bits 0-2 are clean kind bits.
		 */
		if (v & 0x04UL) {
			/* COMPRESSED: handler prefetches its own
			 * target; skip. */
			return;
		}
		/*
		 * EXT or SKIP_EXT: no FT metadata.  Prefetch the node
		 * body, which the META-hint caller typically reads next
		 * (user_data / ->next for the duplicate chain).
		 */
		__builtin_prefetch((const void *) v);
		return;
	}
	/*
	 * Internal-aligned half.  Mask 5 bits cleanly (32-byte
	 * alignment).  We don't need to know the specific kind here —
	 * cds_ft_item_to_metadata works for any internal node base.
	 */
	__builtin_prefetch(cds_ft_item_to_metadata(
		(void *) (v & ~31UL)));
}

static inline __attribute__((always_inline))
void ft_prefetch_child_bitmap_meta(const void *ptr)
{
	unsigned long v = (unsigned long) ptr;

	if (!v)
		return;
	/*
	 * Candidate E dispatch — see ft_prefetch_child_meta for the
	 * bit-0 partition rationale.
	 */
	if (!(v & 1)) {
		/* External-aligned half. */
		if (v & 0x04UL)
			return;	/* COMPRESSED: skip. */
		__builtin_prefetch((const void *) v);
		return;
	}
	/*
	 * Internal-aligned half.  PIGEON-family (PIGEON 0x01,
	 * SKIP_PIGEON 0x03) has bits 2-4 all zero; QP-family and the
	 * future POPCOUNT_* set one of those class bits.  Single AND
	 * + jz, no CMP.
	 */
	{
		void *node = (void *) (v & ~31UL);
		size_t order = cds_ft_item_order(node);

		__builtin_prefetch(cds_ft_item_to_metadata_fast(node, order));
		if ((v & 0x1CUL) == 0)
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
 * POPCOUNT_32 helpers.  See struct ft_pc32_node and the layout
 * comment in fractal-trie-internal.h.  All accessors take @max_lc
 * so the same code serves direct (max_lc = FT_PC32_MAX_LC_DIRECT)
 * and skip (max_lc = FT_PC32_MAX_LC_SKIP) variants — the only
 * difference is the valid popcount range and (write side) the
 * skip-meta slot at physical offset 0.
 *
 * Reverse-indexed pointer slot: ptr_offset = (max_lc_direct - 1) -
 * popcount_idx.  Constant max_lc_direct (= 3) keeps the lookup
 * formula identical across variants; SKIP variant just disallows
 * popcount_idx == max_lc_direct - 1 (= 2), which would land at
 * physical offset 0 (the skip-meta slot).
 */

/* Slot 0 is at physical offset FT_PC32_HEADER_SIZE (= 8 B). */
static inline_lookup
struct cds_ft_inode_flag **ft_pc32_node_slot(struct ft_pc32_node *node,
		unsigned int ptr_offset)
{
	return (struct cds_ft_inode_flag **)
		((uint8_t *) node + FT_PC32_HEADER_SIZE
		 + ptr_offset * sizeof(struct cds_ft_inode_flag *));
}

#ifdef FEATURE_FT_SKIP_COMPRESSED
/*
 * Skip-variant slot 0 access: ft_pc32_skip_meta lives at byte offset
 * FT_PC32_HEADER_SIZE (= 8) from the node base, sharing storage with
 * the direct variant's popcount-rank-2 pointer slot.  The union arm
 * makes the aliasing intent explicit; the byte arithmetic in
 * ft_pc32_node_slot still works for ptr_offset 0 readers because the
 * skip-mode bitmap excludes popcount-rank 2 (max_lc enforced at
 * write time).
 */
static inline_lookup
struct ft_pc32_skip_meta *ft_pc32_node_skip_meta(struct ft_pc32_node *node)
{
	return &node->u.skip.meta;
}
#endif

/*
 * 2-level nibble popcount lookup, max_lc_direct = 3.
 *
 * One 64-bit load brings root_bm + sub_bm[0..2] into a single
 * register: low 16 bits = root_bm, upper 48 bits = sub_bm[0..2]
 * concatenated in popcount order.  Two presence tests (high then
 * low nibble) guard a single popcount-prefix-sum into the pointer
 * array via the reverse-indexed offset formula.
 */
static inline_lookup
struct cds_ft_inode_flag *ft_pc32_node_get_nth_skip(struct ft_pc32_node *node,
		struct cds_ft_inode_flag ***node_flag_ptr,
		uint8_t n, enum ft_pf_target pf_hint)
{
	uint64_t bms = *(const uint64_t *) node;
	uint16_t root = (uint16_t) bms;
	uint64_t subs = bms >> 16;
	unsigned int hi = (unsigned int) n >> 4;
	unsigned int lo = (unsigned int) n & 0xFU;
	unsigned int slot1, bit_pos, ptr_idx, ptr_offset;
	struct cds_ft_inode_flag **slot;

	/* 1. Root check (high nibble present?). */
	if (caa_unlikely(!((root >> hi) & 1U)))
		goto not_found;

	slot1 = (unsigned int) __builtin_popcount(root & ((1U << hi) - 1U));
	bit_pos = (slot1 << 4) | lo;

	/* 2. Sub-bitmap check (low nibble present in this hi's slot?). */
	if (caa_unlikely(!((subs >> bit_pos) & 1ULL)))
		goto not_found;

	/* 3. Pointer index = rank of (hi, lo) among populated entries. */
	ptr_idx = (unsigned int) __builtin_popcountll(
			subs & ((1ULL << bit_pos) - 1ULL));
	/* Reverse-indexed: physical offset 2 = popcount idx 0, etc. */
	ptr_offset = (FT_PC32_MAX_LC_DIRECT - 1U) - ptr_idx;
	slot = ft_pc32_node_slot(node, ptr_offset);
	if (caa_unlikely(node_flag_ptr))
		*node_flag_ptr = slot;
	return ft_dereference_acquire_prefetch_hint(*slot, pf_hint);

not_found:
	if (caa_unlikely(node_flag_ptr))
		*node_flag_ptr = NULL;
	return NULL;
}

/*
 * Number of children in the node — popcount of the sub_bm
 * concatenation.  root_bm popcount equals the count of populated
 * sub-bitmaps and is not a child count by itself.
 */
static inline_lookup
unsigned int ft_pc32_node_get_nr_child(struct ft_pc32_node *node)
{
	uint64_t bms = *(const uint64_t *) node;
	uint64_t subs = bms >> 16;

	return (unsigned int) __builtin_popcountll(subs);
}

/*
 * Insert (n, child) into a freshly-allocated or rebuild-target
 * POPCOUNT_32 node, in safe-append discipline: the byte n must be
 * strictly greater than all existing bytes in the node (highest
 * popcount rank).  Used by:
 *   - recompact's rebuild copy phase (walks old entries in popcount
 *     order, appending to the freshly-zeroed new node).
 *   - the framework's post-copy "add new (n, child)" tail when the
 *     new entry is at the highest popcount rank.
 *
 * Returns 0 on success, -ENOSPC if popcount(new_subs) > @max_lc.
 *
 * Non-safe-append regular inserts (existing node + lower-rank entry)
 * route through recompact (FT_RECOMPACT_ADD_SAME) — the framework
 * builds a new POPCOUNT_32 node off-tree and atomically swaps in.
 *
 * Caller writes the bitmap and pointer slot directly; readers may
 * observe partial state when this is called on a tree-attached node,
 * which is why the caller must guarantee the safe-append precondition
 * or build off-tree.  No release ordering needed for individual
 * stores — the eventual rcu_assign_pointer at publish time provides
 * release ordering for the entire node contents.
 */
static
int ft_pc32_node_set_nth_safe(struct ft_pc32_node *node,
		struct cds_ft_metadata *metadata,
		uint8_t n,
		struct cds_ft_inode_flag *child,
		unsigned int max_lc)
{
	uint16_t root = node->root_bm;
	uint64_t subs = ((uint64_t) node->sub_bm[0])
		      | ((uint64_t) node->sub_bm[1] << 16)
		      | ((uint64_t) node->sub_bm[2] << 32);
	unsigned int hi = (unsigned int) n >> 4;
	unsigned int lo = (unsigned int) n & 0xFU;
	uint16_t hi_bit = (uint16_t) (1U << hi);
	unsigned int slot1, bit_pos, ptr_idx, ptr_offset;
	uint64_t new_subs;

	if (!(root & hi_bit))
		root |= hi_bit;
	slot1 = (unsigned int) __builtin_popcount(root & (hi_bit - 1U));
	bit_pos = (slot1 << 4) | lo;

	/* Compose new_subs: insert the lo bit at popcount rank slot1. */
	if (!((node->root_bm) & hi_bit)) {
		uint64_t low_mask = (slot1 == 0) ? 0ULL
			: ((1ULL << (slot1 << 4)) - 1ULL);
		uint64_t low_part = subs & low_mask;
		uint64_t high_part = (subs & ~low_mask) << 16;

		new_subs = low_part | high_part;
	} else {
		new_subs = subs;
	}
	/*
	 * If the byte n is already present, this is a slot rewrite
	 * (replace semantics, matching FT_PIGEON / FT_QP behavior).
	 * The bitmap stays the same; just rewrite the slot pointer
	 * and skip the nr_child bump.
	 */
	if ((new_subs >> bit_pos) & 1ULL) {
		ptr_idx = (unsigned int) __builtin_popcountll(
				new_subs & ((1ULL << bit_pos) - 1ULL));
		ptr_offset = (FT_PC32_MAX_LC_DIRECT - 1U) - ptr_idx;
		rcu_assign_pointer(*ft_pc32_node_slot(node, ptr_offset),
				child);
		return 0;
	}
	new_subs |= 1ULL << bit_pos;

	if (caa_unlikely((unsigned int) __builtin_popcountll(new_subs) > max_lc))
		return -ENOSPC;

	/*
	 * Safe-append precondition: the new entry must have the highest
	 * popcount rank.  popcount of bits >= bit_pos in new_subs must
	 * equal 1 (just the new bit).  Caller violations return -ERANGE
	 * so the framework can route through recompact ADD_SAME to
	 * rebuild off-tree.
	 */
	if ((unsigned int) __builtin_popcountll(
			new_subs & ~((1ULL << bit_pos) - 1ULL)) != 1U)
		return -ERANGE;
	ptr_idx = (unsigned int) __builtin_popcountll(
			new_subs & ((1ULL << bit_pos) - 1ULL));
	ptr_offset = (FT_PC32_MAX_LC_DIRECT - 1U) - ptr_idx;

	*ft_pc32_node_slot(node, ptr_offset) = child;
	node->root_bm = root;
	node->sub_bm[0] = (uint16_t) (new_subs & 0xFFFFU);
	node->sub_bm[1] = (uint16_t) ((new_subs >> 16) & 0xFFFFU);
	node->sub_bm[2] = (uint16_t) ((new_subs >> 32) & 0xFFFFU);
	metadata->nr_child++;
	return 0;
}

/*
 * Walk POPCOUNT_32 children in popcount order: returns the (byte, ptr)
 * pair at popcount index @i (0 = lowest rank).  Used by recompact
 * copy phase to walk old entries before writing them to the new
 * node, and by iter helpers (get_direction).
 */
static inline_lookup
struct cds_ft_inode_flag *ft_pc32_node_get_ith_pos(struct ft_pc32_node *node,
		unsigned int i, uint8_t *byte_out)
{
	uint16_t root = node->root_bm;
	uint64_t subs = ((uint64_t) node->sub_bm[0])
		      | ((uint64_t) node->sub_bm[1] << 16)
		      | ((uint64_t) node->sub_bm[2] << 32);
	unsigned int bit_pos, slot1, lo, hi;
	uint64_t shifted;
	unsigned int ptr_offset;

	if (i >= (unsigned int) __builtin_popcountll(subs))
		return NULL;

	/* Find the i-th set bit in subs (lowest first). */
	shifted = subs;
	while (i--) {
		shifted &= shifted - 1;
	}
	bit_pos = (unsigned int) __builtin_ctzll(shifted);
	slot1 = bit_pos >> 4;
	lo = bit_pos & 0xFU;

	/* Recover hi = position of the slot1-th set bit in root. */
	{
		uint16_t r = root;
		unsigned int k;

		for (k = 0; k < slot1; k++)
			r &= (uint16_t) (r - 1U);
		hi = (unsigned int) __builtin_ctz((unsigned int) r);
	}

	if (byte_out)
		*byte_out = (uint8_t) ((hi << 4) | lo);

	ptr_offset = (FT_PC32_MAX_LC_DIRECT - 1U)
		   - (unsigned int) __builtin_popcountll(
				subs & ((1ULL << bit_pos) - 1ULL));
	return *ft_pc32_node_slot(node, ptr_offset);
}

/*
 * Replace the slot at byte @n.  In-place rewrite for non-NULL
 * @newptr.  For @newptr == NULL (remove), returns -EFBIG to route
 * through recompact DEL (which copies surviving entries to a fresh
 * node, dropping the removed one).  This avoids the in-place shift
 * needed to maintain reverse-indexed-popcount layout — same
 * simplification as set_nth's -ERANGE path.
 */
static
int ft_pc32_node_replace_ptr(struct ft_pc32_node *node,
		struct cds_ft_metadata *metadata,
		struct cds_ft_inode_flag **node_flag_ptr,
		uint8_t n __attribute__((unused)),
		struct cds_ft_inode_flag *newptr)
{
	if (!newptr) {
		if (metadata->fallback_removal_count) {
			metadata->fallback_removal_count--;
			rcu_assign_pointer(*node_flag_ptr, NULL);
			return 0;
		}
		return -EFBIG;
	}
	rcu_assign_pointer(*node_flag_ptr, newptr);
	return 0;
}

/*
 * Iterator-direction lookup: find the populated child whose key is
 * the leftmost > @n (FT_RIGHT) or rightmost < @n (FT_LEFT).  Returns
 * the child pointer with the matched key in @result_key, or NULL if
 * no such child exists.
 *
 * Strategy: probe the bitmap directly.  POPCOUNT_32 holds at most 3
 * children, so a sequential scan of populated bits is cheap (~6
 * popcounts and bit tests at worst).
 */
static
struct cds_ft_inode_flag *ft_pc32_node_get_direction(struct ft_pc32_node *node,
		int n, uint8_t *result_key, enum ft_direction dir)
{
	unsigned int nr = ft_pc32_node_get_nr_child(node);
	unsigned int i;

	if (dir == FT_RIGHT) {
		for (i = 0; i < nr; i++) {
			uint8_t k = 0;
			struct cds_ft_inode_flag *child =
				ft_pc32_node_get_ith_pos(node, i, &k);

			if ((int) k > n) {
				if (result_key)
					*result_key = k;
				return child;
			}
		}
		return NULL;
	}
	/* FT_LEFT: find the rightmost (highest-byte) child < n. */
	for (i = nr; i > 0; i--) {
		uint8_t k = 0;
		struct cds_ft_inode_flag *child =
			ft_pc32_node_get_ith_pos(node, i - 1, &k);

		if ((int) k < n) {
			if (result_key)
				*result_key = k;
			return child;
		}
	}
	return NULL;
}


/*
 * POPCOUNT_64 helpers (scan_6 layout, 5+3 byte split).  See struct
 * ft_pc64_node and the layout comment in fractal-trie-internal.h.
 *
 * Reverse-indexed pointer slot: ptr_offset = (max_lc_direct - 1) -
 * popcount_idx.  Constant max_lc_direct (= 6) keeps the lookup
 * formula identical across direct and skip variants; SKIP variant
 * just disallows popcount_idx == max_lc_direct - 1 (= 5), which
 * would land at physical offset 0 (the skip-meta slot).
 *
 * Hot-path uses (slot1 << 3) | lo as the absolute bit position in
 * packed_bms, and a single popcount on (packed_bms & mask) yields
 * ptr_idx — no chunk-select cmov, no per-slot prior cache (max_lc=6
 * fits in 48 bits, well within u64).
 */

/* Slot 0 is at physical offset FT_PC64_HEADER_SIZE (= 16 B). */
static inline_lookup
struct cds_ft_inode_flag **ft_pc64_node_slot(struct ft_pc64_node *node,
		unsigned int ptr_offset)
{
	return (struct cds_ft_inode_flag **)
		((uint8_t *) node + FT_PC64_HEADER_SIZE
		 + ptr_offset * sizeof(struct cds_ft_inode_flag *));
}

#ifdef FEATURE_FT_SKIP_COMPRESSED
/* Skip-variant slot 0 access; symmetric to ft_pc32_node_skip_meta. */
static inline_lookup
struct ft_pc32_skip_meta *ft_pc64_node_skip_meta(struct ft_pc64_node *node)
{
	return &node->u.skip.meta;
}
#endif

/*
 * Lookup primitive: 5+3 flat-packed popcount, max_lc_direct = 6.
 *
 * One 32-bit load brings root_bm into a register; one 64-bit load
 * brings packed_bms.  Two presence tests guard a single popcount-
 * prefix-sum into the pointer array via the reverse-indexed offset
 * formula.
 */
static inline_lookup
struct cds_ft_inode_flag *ft_pc64_node_get_nth_skip(struct ft_pc64_node *node,
		struct cds_ft_inode_flag ***node_flag_ptr,
		uint8_t n, enum ft_pf_target pf_hint)
{
	uint32_t root = node->root_bm;
	uint64_t bms = node->packed_bms;
	unsigned int hi = (unsigned int) n >> FT_PC64_LO_BITS;
	unsigned int lo = (unsigned int) n & FT_PC64_LO_MASK;
	unsigned int slot1, p, ptr_idx, ptr_offset;
	struct cds_ft_inode_flag **slot;

	/* 1. Root check (high 5-bit prefix present?). */
	if (caa_unlikely(!((root >> hi) & 1U)))
		goto not_found;

	slot1 = (unsigned int) __builtin_popcount(root & ((1U << hi) - 1U));
	p = (slot1 << FT_PC64_LO_BITS) | lo;

	/* 2. Sub-bitmap check (low 3-bit suffix present in this slot?). */
	if (caa_unlikely(!((bms >> p) & 1ULL)))
		goto not_found;

	/* 3. Pointer index = rank of (hi, lo) among populated entries. */
	ptr_idx = (unsigned int) __builtin_popcountll(
			bms & ((1ULL << p) - 1ULL));
	ptr_offset = (FT_PC64_MAX_LC_DIRECT - 1U) - ptr_idx;
	slot = ft_pc64_node_slot(node, ptr_offset);
	if (caa_unlikely(node_flag_ptr))
		*node_flag_ptr = slot;
	return ft_dereference_acquire_prefetch_hint(*slot, pf_hint);

not_found:
	if (caa_unlikely(node_flag_ptr))
		*node_flag_ptr = NULL;
	return NULL;
}

/*
 * Number of children: popcount of packed_bms.  No need to consult
 * root_bm — it tracks distinct hi-buckets, not child count.
 */
static inline_lookup
unsigned int ft_pc64_node_get_nr_child(struct ft_pc64_node *node)
{
	return (unsigned int) __builtin_popcountll(node->packed_bms);
}

/*
 * Insert (n, child) into a freshly-allocated or rebuild-target
 * POPCOUNT_64 node, in safe-append discipline.  Mirrors
 * ft_pc32_node_set_nth_safe; returns -ENOSPC if popcount(new_bms)
 * exceeds @max_lc, -ERANGE on non-safe-append insert (the framework
 * routes through recompact ADD_SAME), 0 on success.
 *
 * A duplicate byte triggers a slot rewrite (no nr_child bump),
 * matching FT_PIGEON / FT_QP / POPCOUNT_32 replace semantics.
 */
static
int ft_pc64_node_set_nth_safe(struct ft_pc64_node *node,
		struct cds_ft_metadata *metadata,
		uint8_t n,
		struct cds_ft_inode_flag *child,
		unsigned int max_lc)
{
	uint32_t root = node->root_bm;
	uint64_t bms = node->packed_bms;
	unsigned int hi = (unsigned int) n >> FT_PC64_LO_BITS;
	unsigned int lo = (unsigned int) n & FT_PC64_LO_MASK;
	uint32_t hi_bit = 1U << hi;
	unsigned int slot1, p, ptr_idx, ptr_offset;
	uint64_t new_bms;

	if (!(root & hi_bit))
		root |= hi_bit;
	slot1 = (unsigned int) __builtin_popcount(root & (hi_bit - 1U));
	p = (slot1 << FT_PC64_LO_BITS) | lo;

	/* Compose new_bms: insert the lo bit at popcount rank slot1. */
	if (!(node->root_bm & hi_bit)) {
		uint64_t low_mask = (slot1 == 0) ? 0ULL
			: ((1ULL << (slot1 << FT_PC64_LO_BITS)) - 1ULL);
		uint64_t low_part = bms & low_mask;
		uint64_t high_part = (bms & ~low_mask) << (1U << FT_PC64_LO_BITS);

		new_bms = low_part | high_part;
	} else {
		new_bms = bms;
	}
	/* Duplicate-byte rewrite — same semantics as POPCOUNT_32. */
	if ((new_bms >> p) & 1ULL) {
		ptr_idx = (unsigned int) __builtin_popcountll(
				new_bms & ((1ULL << p) - 1ULL));
		ptr_offset = (FT_PC64_MAX_LC_DIRECT - 1U) - ptr_idx;
		rcu_assign_pointer(*ft_pc64_node_slot(node, ptr_offset),
				child);
		return 0;
	}
	new_bms |= 1ULL << p;

	if (caa_unlikely((unsigned int) __builtin_popcountll(new_bms) > max_lc))
		return -ENOSPC;

	/* Safe-append: only the new bit may be at-or-above bit_pos. */
	if ((unsigned int) __builtin_popcountll(
			new_bms & ~((1ULL << p) - 1ULL)) != 1U)
		return -ERANGE;
	ptr_idx = (unsigned int) __builtin_popcountll(
			new_bms & ((1ULL << p) - 1ULL));
	ptr_offset = (FT_PC64_MAX_LC_DIRECT - 1U) - ptr_idx;

	*ft_pc64_node_slot(node, ptr_offset) = child;
	node->root_bm = root;
	node->packed_bms = new_bms;
	metadata->nr_child++;
	return 0;
}

/*
 * Walk POPCOUNT_64 children in popcount order: returns the (byte, ptr)
 * pair at popcount index @i (0 = lowest rank).
 */
static inline_lookup
struct cds_ft_inode_flag *ft_pc64_node_get_ith_pos(struct ft_pc64_node *node,
		unsigned int i, uint8_t *byte_out)
{
	uint32_t root = node->root_bm;
	uint64_t bms = node->packed_bms;
	unsigned int p, slot1, lo, hi;
	uint64_t shifted;
	unsigned int ptr_offset;

	if (i >= (unsigned int) __builtin_popcountll(bms))
		return NULL;

	/* Find the i-th set bit in bms (lowest first). */
	shifted = bms;
	while (i--)
		shifted &= shifted - 1;
	p = (unsigned int) __builtin_ctzll(shifted);
	slot1 = p >> FT_PC64_LO_BITS;
	lo = p & FT_PC64_LO_MASK;

	/* Recover hi = position of the slot1-th set bit in root. */
	{
		uint32_t r = root;
		unsigned int k;

		for (k = 0; k < slot1; k++)
			r &= r - 1U;
		hi = (unsigned int) __builtin_ctz(r);
	}

	if (byte_out)
		*byte_out = (uint8_t) ((hi << FT_PC64_LO_BITS) | lo);

	ptr_offset = (FT_PC64_MAX_LC_DIRECT - 1U)
		   - (unsigned int) __builtin_popcountll(
				bms & ((1ULL << p) - 1ULL));
	return *ft_pc64_node_slot(node, ptr_offset);
}

/*
 * Replace the slot at byte @n.  Same semantics as POPCOUNT_32:
 * -EFBIG on remove (route through recompact DEL), in-place rewrite
 * for non-NULL @newptr.
 */
static
int ft_pc64_node_replace_ptr(struct ft_pc64_node *node __attribute__((unused)),
		struct cds_ft_metadata *metadata,
		struct cds_ft_inode_flag **node_flag_ptr,
		uint8_t n __attribute__((unused)),
		struct cds_ft_inode_flag *newptr)
{
	if (!newptr) {
		if (metadata->fallback_removal_count) {
			metadata->fallback_removal_count--;
			rcu_assign_pointer(*node_flag_ptr, NULL);
			return 0;
		}
		return -EFBIG;
	}
	rcu_assign_pointer(*node_flag_ptr, newptr);
	return 0;
}

/*
 * Iterator-direction lookup, mirroring the POPCOUNT_32 version.
 * POPCOUNT_64 holds at most 6 children, so a sequential walk over
 * populated bits is still cheap.
 */
static
struct cds_ft_inode_flag *ft_pc64_node_get_direction(struct ft_pc64_node *node,
		int n, uint8_t *result_key, enum ft_direction dir)
{
	unsigned int nr = ft_pc64_node_get_nr_child(node);
	unsigned int i;

	if (dir == FT_RIGHT) {
		for (i = 0; i < nr; i++) {
			uint8_t k = 0;
			struct cds_ft_inode_flag *child =
				ft_pc64_node_get_ith_pos(node, i, &k);

			if ((int) k > n) {
				if (result_key)
					*result_key = k;
				return child;
			}
		}
		return NULL;
	}
	for (i = nr; i > 0; i--) {
		uint8_t k = 0;
		struct cds_ft_inode_flag *child =
			ft_pc64_node_get_ith_pos(node, i - 1, &k);

		if ((int) k < n) {
			if (result_key)
				*result_key = k;
			return child;
		}
	}
	return NULL;
}


/*
 * QP-nibble tier table.  Parallel to ft_types[]; describes the four
 * QP-nibble allocation tiers (T0..T3) by popcount range and node
 * size.  Indexed by tier number (0..3); FT_QP16_NR_TIERS == 4.
 *
 * The hysteresis (min_child < previous max_child) gives a window
 * during shrinkage where we stay in the larger tier — same idea as
 * ft_types[]'s overlapping min/max ranges for the byte-keyed types.
 */
struct cds_ft_qp16_tier {
	uint16_t min_child;	/* hysteresis lower bound (inclusive) */
	uint16_t max_child;	/* highest popcount this tier holds */
	uint16_t order;		/* node size = (1 << order) bytes */
};

static const struct cds_ft_qp16_tier ft_qp16_tiers[FT_QP16_NR_TIERS] = {
	[0] = {	.min_child = 1,
		.max_child = FT_QP16_T0_CAPACITY,
		.order     = FT_QP16_T0_ALLOC_ORDER },
	[1] = {	.min_child = 2,
		.max_child = FT_QP16_T1_CAPACITY,
		.order     = FT_QP16_T1_ALLOC_ORDER },
	[2] = {	.min_child = 5,
		.max_child = FT_QP16_T2_CAPACITY,
		.order     = FT_QP16_T2_ALLOC_ORDER },
	[3] = {	.min_child = 11,
		.max_child = FT_QP16_T3_CAPACITY,
		.order     = FT_QP16_T3_ALLOC_ORDER },
};

/*
 * QP-skip Phase 2 base pointer helper.  Direct variant starts at byte
 * 8 (immediately after the 8B header); skip variant starts at byte 16
 * (after the overlay subkey extension).  Hot-path callers fold the
 * branch by passing a compile-time constant for is_skip.  Defined here
 * because ft_qp16_node_descend below uses it; the SKIP-COMPRESSED-only
 * helpers (ft_qp16_skip_meta_at, ft_qp16_alloc_order_skip) live after
 * the descent body — see further below.
 */
static inline
struct cds_ft_inode_flag **
ft_qp16_ptrs(struct cds_ft_qp16_node *node, bool is_skip)
{
	size_t offset = is_skip ? 16U : FT_QP16_HEADER_SIZE;
	return (struct cds_ft_inode_flag **) ((char *) node + offset);
}

/*
 * QP-nibble read-side scanner — handles both hi- and lo-nibble
 * descent in one inlinable helper.  Two dependent loads on the hot
 * path:
 *   1. relaxed-load bitmap (header CL) — pre-filter only
 *   2. acquire-load ptrs[idx] (pointer-array CL — same CL as bitmap
 *      for tiers T0/T1) — source of truth, paired with the writer's
 *      rcu_assign_pointer release at insert / removal.
 *
 * @nibble must be in 0..15.  Returns the child pointer, or NULL if
 * the nibble has no child (bit clear in bitmap, or slot is a
 * tombstone-NULL).  When @ptr_slot_p is non-NULL, also returns the
 * address of the pointer slot — used by writers that need to
 * atomically rewrite the slot (e.g. graft, recompact-publish).
 * Pure-read callers pass NULL.
 *
 * The bitmap is a pre-filter: a clear bit guarantees absence (skip
 * the pointer load); a set bit is an "is-this-maybe-here" hint with
 * the acquire-load on the slot as the source of truth.  Acquire
 * (rather than relaxed) on the slot pairs with the count-based
 * readers' undercount guarantee: writers do the nr_keys decrement
 * BEFORE the rcu_assign_pointer release at removal, so a reader that
 * observes the detached pointer also observes the preceding nr_keys
 * update on weakly-ordered architectures.
 *
 * Pointer encoding (writer-side discipline):
 *   - Hi-nibble slots store the lo-nibble qp16_node pointer raw and
 *     untagged.  Reader casts directly; no tag-bit work.
 *   - Lo-nibble slots store a tagged cds_ft_inode_flag * (external,
 *     compressed, internal, or skip-compressed).  Reader inspects
 *     tag bits to dispatch.
 *
 * @is_lo_nibble (0 = hi, non-zero = lo): compile-time constant at the
 * call site.  Selects the prefetch path:
 *   - hi: direct __builtin_prefetch on the loaded ptr (no tag work,
 *     slot is known untagged).
 *   - lo: ft_maybe_prefetch_hint with the caller's @pf_hint (tag
 *     dispatch, may skip prefetch for compressed-class children).
 * The acquire-load itself is identical in both branches.
 */
static inline_lookup
struct cds_ft_inode_flag *ft_qp16_node_descend(
		struct cds_ft_qp16_node *node,
		struct cds_ft_inode_flag ***ptr_slot_p,
		uint8_t nibble, enum ft_pf_target pf_hint,
		int is_lo_nibble, bool is_skip)
{
	uint16_t bm = uatomic_load(&node->bitmap, CMM_RELAXED);
	uint16_t bit = (uint16_t) (1U << (nibble & 0xFU));
	struct cds_ft_inode_flag **slot, *child;
	unsigned int idx;

	if (!(bm & bit)) {
		if (caa_unlikely(ptr_slot_p))
			*ptr_slot_p = NULL;
		return NULL;
	}
	idx = (unsigned int) __builtin_popcount(
			(unsigned int) (bm & (bit - 1U)));
	slot = &ft_qp16_ptrs(node, is_skip)[idx];
	if (caa_unlikely(ptr_slot_p))
		*ptr_slot_p = slot;
	if (is_lo_nibble) {
		child = ft_dereference_acquire_prefetch_hint(*slot, pf_hint);
	} else {
		(void) pf_hint;
		child = ft_dereference_acquire(*slot);
		if (caa_likely(child))
			__builtin_prefetch(child);
	}
	return child;
}

/*
 * QP-nibble alloc-order picker — maps popcount(bitmap) to the
 * smallest tier whose capacity fits.  Used by writers to size newly
 * allocated nodes; declared static-inline here so the compiler can
 * fold it into a chain of compares (or a lookup table) at the call
 * site.
 */
static inline
unsigned int ft_qp16_alloc_order(unsigned int popcount)
{
	if (popcount <= FT_QP16_T0_CAPACITY)
		return FT_QP16_T0_ALLOC_ORDER;
	if (popcount <= FT_QP16_T1_CAPACITY)
		return FT_QP16_T1_ALLOC_ORDER;
	if (popcount <= FT_QP16_T2_CAPACITY)
		return FT_QP16_T2_ALLOC_ORDER;
	return FT_QP16_T3_ALLOC_ORDER;
}

/*
 * QP-nibble alloc-order → capacity inverse.  T0..T3 alloc orders are
 * sequential (5..8), so the tier index is just (order - T0_order) and
 * the capacity comes from the parallel ft_qp16_tiers[] table.  Static
 * inline so the lookup folds at the call site.
 */
static inline
unsigned int ft_qp16_capacity_from_order(unsigned int order)
{
	unsigned int tier = order - FT_QP16_T0_ALLOC_ORDER;

	assert(tier < FT_QP16_NR_TIERS);
	return ft_qp16_tiers[tier].max_child;
}

#ifdef FEATURE_FT_SKIP_COMPRESSED
/*
 * QP-skip Phase 2 skip-only helpers.  The base pointer helper
 * ft_qp16_ptrs is defined earlier (before ft_qp16_node_descend) so it
 * is callable from the descent body; the view + picker below are
 * skip-variant-only and gated on FEATURE_FT_SKIP_COMPRESSED.
 *
 * Skip-variant header view of a QP node.  Anchored at byte offset 2,
 * giving skip_len at byte 2 and subkey[13] at bytes 3-15 — the same
 * memory the direct layout would expose as the prior 5B inline subkey
 * (bytes 3-7) plus the highest-rank ptr slot (bytes 8-15).
 *
 * Callers must hold that the node was reached via an FT_KIND_SKIP_QP
 * slot tag before dereferencing the extension bytes; otherwise the
 * overlay aliases a live pointer.
 */
static inline
struct ft_qp16_skip_meta *
ft_qp16_skip_meta_at(struct cds_ft_qp16_node *node)
{
	return (struct ft_qp16_skip_meta *) ((char *) node + 2);
}

/*
 * QP-nibble alloc-order picker for skip-variant nodes.  Same tier
 * lattice as ft_qp16_alloc_order, but capacity bounds drop by one to
 * account for the subkey overlay.  Returns T3 order for popcount
 * values up to FT_QP16_T3_CAPACITY_SKIP; popcount > 15 is not
 * representable by the skip variant — callers must fall back to
 * direct-variant alloc with a leaf-validate descent.
 */
static inline
unsigned int ft_qp16_alloc_order_skip(unsigned int popcount)
{
	if (popcount <= FT_QP16_T0_CAPACITY_SKIP)
		return FT_QP16_T0_ALLOC_ORDER;
	if (popcount <= FT_QP16_T1_CAPACITY_SKIP)
		return FT_QP16_T1_ALLOC_ORDER;
	if (popcount <= FT_QP16_T2_CAPACITY_SKIP)
		return FT_QP16_T2_ALLOC_ORDER;
	return FT_QP16_T3_ALLOC_ORDER;
}
#endif /* FEATURE_FT_SKIP_COMPRESSED */

/*
 * Half-cacheline footprint of a node allocation order.  Half-CL = 32 B,
 * the unit used by qp_subtree_half_cls accounting:
 *
 *   T0  (order 5 = 32 B)   → 1
 *   T1  (order 6 = 64 B)   → 2
 *   T2  (order 7 = 128 B)  → 4
 *   T3  (order 8 = 256 B)  → 8
 *   PIGEON (order 11 = 2 KB on 64-bit, 1 KB on 32-bit) → 64 / 32
 *
 * The QP→PIGEON up-trigger fires when the hi+lo half-CL sum exceeds
 * PIGEON's flat footprint.
 */
#define FT_PIGEON_HALF_CLS	(1U << (FT_PIGEON_ORDER - 5U))

static inline
unsigned int ft_node_half_cls(unsigned int order)
{
	return 1U << (order - 5U);
}

/*
 * QP-nibble lo-flag builder.
 *
 * Lo-nodes are stored RAW (untagged) in their parent hi-node's ptrs[i]
 * so the reader's hi-side descent casts the slot value directly with
 * no tag-strip.  The tagged form (built here) is used only on the
 * upward path: meta->parent of every byte-keyed direct child of a lo
 * holds the tagged lo-flag, and ft_set_parent's slot-based resolver
 * derives the tag from the lo-arena base when callers pass the hi-flag.
 *
 * Lo-nodes share the FT_KIND_QP kind tag with hi-nodes — the
 * HI/LO distinction has been retired from the slot-tag space and
 * is recovered from the lo-node's metadata is_lo bit during the
 * upward parent walk (ft_parent_depth_span).
 */
static inline
struct cds_ft_inode_flag *ft_qp16_lo_flag(struct cds_ft_qp16_node *lo)
{
	return (struct cds_ft_inode_flag *)
		(((unsigned long) lo) | (unsigned long) FT_KIND_QP);
}

/*
 * QP-nibble nr_child accessor — popcount of the bitmap.  Bitmap
 * field is read relaxed; this is a pre-filter count, not a
 * synchronizing load.  Callers needing a child pointer with
 * publication-safe semantics use ft_qp16_node_descend above (which
 * acquire-loads the slot).
 */
static inline
unsigned int ft_qp16_node_nr_child(const struct cds_ft_qp16_node *node)
{
	return (unsigned int) __builtin_popcount(
			(unsigned int) uatomic_load(&node->bitmap, CMM_RELAXED));
}

/*
 * QP-nibble directional lookup — find the first nibble strictly
 * greater (or strictly less) than @nibble that has a child, and
 * return that child along with the matched nibble.
 *
 * @dir == FT_RIGHT: smallest nibble strictly greater than @nibble.
 * @dir == FT_LEFT:  largest nibble strictly less than @nibble.
 *
 * Returns NULL when no such nibble exists (caller is at the boundary
 * of the node).  When non-NULL, *@result_nibble holds the matched
 * nibble (0..15).
 *
 * Used by inequality lookup (cds_ft_lookup_lt / _gt / _le / _ge):
 * after a non-matching nibble, the descent picks the directional
 * neighbor to continue.
 *
 * Implementation: mask the bitmap to the candidate range, then ctz
 * (right) or 31-clz (left) the result to find the matching set bit,
 * and use popcount on the lower side to derive the pointer index.
 *
 * Range masks (16-bit):
 *   right of nibble n: bits (n+1)..15  →  ~((1<<(n+1)) - 1) & 0xFFFF
 *   left  of nibble n: bits 0..(n-1)   →  (1<<n) - 1
 */
static inline_lookup
struct cds_ft_inode_flag *ft_qp16_node_get_direction(
		struct cds_ft_qp16_node *node,
		uint8_t nibble, uint8_t *result_nibble,
		enum ft_direction dir)
{
	uint16_t bm = uatomic_load(&node->bitmap, CMM_RELAXED);
	uint16_t side;
	unsigned int n;
	unsigned int matched_bit;
	unsigned int idx;

	n = (unsigned int) (nibble & 0xFU);
	if (dir == FT_RIGHT) {
		/* bits strictly greater than n */
		uint16_t hi_mask = (uint16_t) (~((1U << (n + 1U)) - 1U) & 0xFFFFU);
		side = (uint16_t) (bm & hi_mask);
		if (!side)
			return NULL;
		matched_bit = (unsigned int) __builtin_ctz((unsigned int) side);
	} else {
		/* bits strictly less than n */
		uint16_t lo_mask = (uint16_t) ((1U << n) - 1U);
		side = (uint16_t) (bm & lo_mask);
		if (!side)
			return NULL;
		matched_bit = 31U - (unsigned int) __builtin_clz((unsigned int) side);
	}
	*result_nibble = (uint8_t) matched_bit;
	idx = (unsigned int) __builtin_popcount(
			(unsigned int) (bm & ((1U << matched_bit) - 1U)));
	return ft_dereference_acquire(node->ptrs[idx]);
}

/*
 * QP-nibble ith-position accessor — return the i-th live child in
 * popcount order (i ∈ [0, popcount(bitmap))) and the nibble it
 * dispatches on.
 *
 * Used by ordered iteration (cds_ft_for_each_rcu, lookup_nth) and
 * by debug walks.  Symmetric to ft_pigeon_node_get_ith_pos but
 * resolves nibble via popcount-bit-select instead of a 256-bit
 * bitmap scan.
 *
 * Bit-select via popcount: walk the bitmap finding the i-th set bit.
 * On x86-64 with BMI2, this could compile to a PDEP + TZCNT pair;
 * portable code uses an unrolled bit-scan loop.  For QP-16 the
 * worst case is 16 bits, so a simple loop is fine.
 */
static inline_lookup
struct cds_ft_inode_flag *ft_qp16_node_get_ith_pos(
		struct cds_ft_qp16_node *node,
		unsigned int i, uint8_t *result_nibble, bool is_skip)
{
	uint16_t bm = uatomic_load(&node->bitmap, CMM_RELAXED);
	unsigned int j, count = 0, matched = 16U;

	for (j = 0; j < 16U; j++) {
		if (!(bm & (1U << j)))
			continue;
		if (count == i) {
			matched = j;
			break;
		}
		count++;
	}
	if (matched == 16U)
		return NULL;
	*result_nibble = (uint8_t) matched;
	return ft_dereference_acquire(ft_qp16_ptrs(node, is_skip)[i]);
}

/*
 * QP-nibble ordered iteration — convenience wrapper that returns
 * the leftmost (FT_LEFT) or rightmost (FT_RIGHT) live child.  Used
 * to start a forward / reverse iteration at a node.  Equivalent to
 * ft_qp16_node_get_ith_pos(node, 0) / (..., nr_child - 1).
 *
 * Returns NULL only on an empty node (bitmap == 0), which is the
 * empty-root case.
 */
static inline_lookup
struct cds_ft_inode_flag *ft_qp16_node_get_extremum(
		struct cds_ft_qp16_node *node,
		uint8_t *result_nibble, enum ft_direction dir, bool is_skip)
{
	uint16_t bm = uatomic_load(&node->bitmap, CMM_RELAXED);
	unsigned int matched_bit;

	if (!bm)
		return NULL;
	if (dir == FT_LEFT) {
		matched_bit = (unsigned int) __builtin_ctz((unsigned int) bm);
		*result_nibble = (uint8_t) matched_bit;
		return ft_dereference_acquire(ft_qp16_ptrs(node, is_skip)[0]);
	}
	matched_bit = 31U - (unsigned int) __builtin_clz((unsigned int) bm);
	*result_nibble = (uint8_t) matched_bit;
	{
		unsigned int idx = (unsigned int) __builtin_popcount(
				(unsigned int) bm) - 1U;
		return ft_dereference_acquire(ft_qp16_ptrs(node, is_skip)[idx]);
	}
}

/*
 * QP-nibble writer: initialize a freshly-allocated node.  Allocator
 * delivers zeroed memory (bitmap = 0, ptrs[] = NULL); this helper is
 * a relaxed-store no-op that documents the invariant.
 */
static inline
void ft_qp16_node_init(struct cds_ft_qp16_node *node)
{
	uatomic_store(&node->bitmap, (uint16_t) 0, CMM_RELAXED);
}

/*
 * QP-nibble writer: insert / replace / revive in-place.
 *
 *   - Bit already set (live or tombstone): rcu_assign_pointer the
 *     slot.  This handles graft-replace and re-insert at a previously
 *     deleted nibble uniformly.  The bitmap is not touched.
 *
 *   - Bit not set, safe-append (no higher bit set in the bitmap):
 *     bitmap |= bit (relaxed) FIRST, then rcu_assign_pointer the
 *     slot.  Safety: the slot at popcount(bm) was zeroed at
 *     allocation and never written since (the bitmap monotonically
 *     grows under this design — delete is a slot-NULL, not a bit
 *     clear), so a reader that observes the new bit and races to
 *     the slot sees either NULL (returns not-found) or the new
 *     child (returns found).  rcu_assign_pointer is the publishing
 *     release.
 *
 *   - Bit not set, non-safe-append: returns -ERANGE.  Caller falls
 *     back to CoW (ft_qp16_node_cow_insert) — building a new node
 *     with the inserted slot in popcount order avoids the race that
 *     in-place insertion-in-the-middle would create on existing
 *     slot indices above @nibble.
 *
 *   - Capacity exhaustion (popcount == @capacity, bit not set):
 *     returns -ENOSPC.  Caller recompacts — building a fresh node
 *     of appropriate tier, dropping tombstones in the process.
 *
 * @capacity is the node's tier capacity (T0..T3 → 3, 7, 15, 16);
 * caller derives it from the parent's pointer-tag bookkeeping.
 */
static inline
int ft_qp16_node_set_nth_safe(struct cds_ft_qp16_node *node,
		uint8_t nibble, struct cds_ft_inode_flag *child,
		unsigned int capacity, bool is_skip)
{
	uint16_t bm = uatomic_load(&node->bitmap, CMM_RELAXED);
	uint16_t bit = (uint16_t) (1U << (nibble & 0xFU));
	struct cds_ft_inode_flag **ptrs = ft_qp16_ptrs(node, is_skip);
	unsigned int idx;

	if (bm & bit) {
		/* Replace live or revive tombstone — bitmap unchanged. */
		idx = (unsigned int) __builtin_popcount(
				(unsigned int) (bm & (bit - 1U)));
		rcu_assign_pointer(ptrs[idx], child);
		return 0;
	}
	/* Bit not set — must be safe-append. */
	if (bm & (uint16_t) ~((1U << (nibble & 0xFU)) - 1U))
		return -ERANGE;	/* higher bit already set; not safe-append */
	idx = (unsigned int) __builtin_popcount((unsigned int) bm);
	if (idx >= capacity)
		return -ENOSPC;
	uatomic_store(&node->bitmap, (uint16_t) (bm | bit), CMM_RELAXED);
	rcu_assign_pointer(ptrs[idx], child);
	return 0;
}

/*
 * QP-nibble writer: tombstone-style delete.
 *
 * The bitmap bit is left set; only the pointer slot is NULL'd (via
 * rcu_assign_pointer, store-release).  Readers that observe the
 * (still-set) bit acquire-load the slot and see either the old child
 * (still RCU-valid, will be reclaimed after the matching grace
 * period at the deletion site) or NULL (return not-found).
 *
 * The bitmap doubles as a pre-filter: a clear bit guarantees absence;
 * a set bit only indicates "may be present", with the slot load as
 * the source of truth.  Same pattern as FT_PIGEON's bitmap_scan path.
 *
 * Returns -ENOENT if the bit is clear or the slot is already NULL
 * (idempotent).  Recompaction (recovering tombstoned slots into a
 * tighter bitmap) happens when set_nth_safe returns -ENOSPC.
 */
static inline
int ft_qp16_node_clear_nth(struct cds_ft_qp16_node *node, uint8_t nibble,
		bool is_skip)
{
	uint16_t bm = uatomic_load(&node->bitmap, CMM_RELAXED);
	uint16_t bit = (uint16_t) (1U << (nibble & 0xFU));
	struct cds_ft_inode_flag **ptrs = ft_qp16_ptrs(node, is_skip);
	unsigned int idx;

	if (!(bm & bit))
		return -ENOENT;
	idx = (unsigned int) __builtin_popcount(
			(unsigned int) (bm & (bit - 1U)));
	if (!ptrs[idx])
		return -ENOENT;
	rcu_assign_pointer(ptrs[idx], NULL);
	return 0;
}

/*
 * QP-nibble writer: copy-on-write insert for the non-safe-append
 * fallback.  Caller has allocated @new_node (zeroed via
 * ft_qp16_node_init or allocator) and passes the source @src_node
 * (still published; readers may be concurrently descending it).
 *
 * Walks the new bitmap (src_bm | bit) in popcount order; for each
 * set bit either copies the corresponding ptrs[] entry from src
 * (preserving tombstones — NULL slots stay NULL) or substitutes
 * @child at the inserted nibble.  All stores are plain (the
 * destination is unpublished); caller publishes by atomically
 * swapping the parent's child pointer and RCU-freeing @src_node.
 *
 * Returns 0 on success, -EEXIST if @nibble is already set in the
 * source bitmap (caller should have used set_nth_safe), or -ENOSPC
 * if the destination tier is too small for popcount(new_bm).
 *
 * Recompact (tombstone-cleanup) is a separate op: it walks src
 * dropping NULL slots; not in this helper.
 */
static
int ft_qp16_node_cow_insert(struct cds_ft_qp16_node *new_node,
		const struct cds_ft_qp16_node *src_node,
		uint8_t nibble, struct cds_ft_inode_flag *child,
		unsigned int new_capacity, bool is_skip)
{
	uint16_t src_bm = src_node->bitmap;
	uint16_t bit = (uint16_t) (1U << (nibble & 0xFU));
	uint16_t new_bm;
	struct cds_ft_inode_flag **new_ptrs = ft_qp16_ptrs(new_node, is_skip);
	struct cds_ft_inode_flag **src_ptrs = ft_qp16_ptrs(
			(struct cds_ft_qp16_node *) src_node, is_skip);
	unsigned int src_idx = 0, new_idx = 0;
	unsigned int b;

	if (src_bm & bit)
		return -EEXIST;
	new_bm = (uint16_t) (src_bm | bit);
	if ((unsigned int) __builtin_popcount((unsigned int) new_bm)
			> new_capacity)
		return -ENOSPC;

	for (b = 0; b < 16U; b++) {
		uint16_t mask = (uint16_t) (1U << b);

		if (b == (nibble & 0xFU)) {
			new_ptrs[new_idx++] = child;
			continue;
		}
		if (src_bm & mask)
			new_ptrs[new_idx++] = src_ptrs[src_idx++];
	}
	new_node->bitmap = new_bm;
	return 0;
}

/*
 * QP-nibble writer: recompact a node into @new_node, dropping
 * tombstones (NULL slots).  Walks src in popcount order; emits only
 * slots whose ptr is non-NULL, building the corresponding tightened
 * bitmap.
 *
 * Caller has allocated @new_node (zeroed) at a tier sized for the
 * live-child popcount (recompute via ft_qp16_alloc_order); publishing
 * is the same swap-parent-then-RCU-free pattern as cow_insert.
 *
 * Returns 0 on success, -ENOSPC if the destination tier can't hold
 * the live-child popcount.
 */
static
int ft_qp16_node_recompact(struct cds_ft_qp16_node *new_node,
		const struct cds_ft_qp16_node *src_node,
		unsigned int new_capacity, bool is_skip)
{
	uint16_t src_bm = src_node->bitmap;
	uint16_t new_bm = 0;
	struct cds_ft_inode_flag **new_ptrs = ft_qp16_ptrs(new_node, is_skip);
	struct cds_ft_inode_flag **src_ptrs = ft_qp16_ptrs(
			(struct cds_ft_qp16_node *) src_node, is_skip);
	unsigned int src_idx = 0, new_idx = 0;
	unsigned int b;

	for (b = 0; b < 16U; b++) {
		struct cds_ft_inode_flag *p;

		if (!(src_bm & (uint16_t) (1U << b)))
			continue;
		p = src_ptrs[src_idx++];
		if (!p)
			continue;	/* tombstone: drop */
		if (new_idx >= new_capacity)
			return -ENOSPC;
		new_ptrs[new_idx++] = p;
		new_bm |= (uint16_t) (1U << b);
	}
	new_node->bitmap = new_bm;
	return 0;
}

/*
 * QP-nibble writer: recompact + insert in a single walk.
 *
 * Combines ft_qp16_node_recompact (drop tombstones) and an insert at
 * @nibble in popcount order.  Saves an alloc on the -ENOSPC path:
 * instead of "alloc new at same tier → recompact → still no room →
 * alloc new at next tier → cow_insert", the caller goes "alloc at
 * the speculative tier → recompact_and_insert" once.
 *
 * Caller has allocated @new_node (zeroed) at the tier whose capacity
 * is at least the live-child popcount of @src_node + 1.  Publishing
 * uses the same swap-parent-then-RCU-free protocol as cow_insert /
 * recompact.
 *
 * Returns 0 on success, -EEXIST if @nibble is already set in
 * @src_node (caller intended set_nth_safe instead), or -ENOSPC if the
 * destination tier can't hold (live-children + 1).
 */
static __attribute__((unused))
int ft_qp16_node_recompact_and_insert(struct cds_ft_qp16_node *new_node,
		const struct cds_ft_qp16_node *src_node,
		uint8_t nibble, struct cds_ft_inode_flag *child,
		unsigned int new_capacity, bool is_skip)
{
	uint16_t src_bm = src_node->bitmap;
	uint16_t bit = (uint16_t) (1U << (nibble & 0xFU));
	uint16_t new_bm = 0;
	struct cds_ft_inode_flag **new_ptrs = ft_qp16_ptrs(new_node, is_skip);
	struct cds_ft_inode_flag **src_ptrs = ft_qp16_ptrs(
			(struct cds_ft_qp16_node *) src_node, is_skip);
	unsigned int src_idx = 0, new_idx = 0;
	unsigned int b;

	if (src_bm & bit)
		return -EEXIST;

	for (b = 0; b < 16U; b++) {
		struct cds_ft_inode_flag *p;

		if (b == (nibble & 0xFU)) {
			/* Insert here; src had no slot at this nibble. */
			if (new_idx >= new_capacity)
				return -ENOSPC;
			new_ptrs[new_idx++] = child;
			new_bm |= (uint16_t) (1U << b);
			continue;
		}
		if (!(src_bm & (uint16_t) (1U << b)))
			continue;
		p = src_ptrs[src_idx++];
		if (!p)
			continue;	/* tombstone: drop */
		if (new_idx >= new_capacity)
			return -ENOSPC;
		new_ptrs[new_idx++] = p;
		new_bm |= (uint16_t) (1U << b);
	}
	new_node->bitmap = new_bm;
	return 0;
}

/*
 * QP-nibble allocator wrappers.  Thin layer over cds_ft_alloc_item /
 * cds_ft_free_item that:
 *   - returns the allocator's metadata alongside the node so callers
 *     can avoid a redundant cds_ft_item_to_metadata round-trip;
 *   - bumps the per-ft debug counters consistently with the byte-keyed
 *     alloc_cds_ft_node / free_cds_ft_node helpers;
 *   - documents the always-zeroed contract: the allocator delivers
 *     zeroed memory, so bitmap = 0 and ptrs[] = NULL on return — this
 *     matches the empty-node invariant ft_qp16_node_init asserts.
 *
 * @order is one of FT_QP16_T{0,1,2,3}_ALLOC_ORDER (or whatever the
 * caller picks via ft_qp16_alloc_order from the live-child popcount).
 * No bitmap arena slot is requested: QP nodes carry their bitmap
 * inline in the header, unlike the byte-keyed FT_PIGEON / pool nodes
 * where the bitmap lives at the page footer.
 */
static __attribute__((unused))
struct cds_ft_qp16_node *ft_qp16_node_alloc(struct cds_ft *ft,
		unsigned int order,
		struct cds_ft_metadata **meta_p)
{
	struct cds_ft_metadata *metadata;
	struct cds_ft_qp16_node *node;

	metadata = cds_ft_alloc_item(ft, (size_t) order, false);
	if (!metadata)
		return NULL;
	node = (struct cds_ft_qp16_node *) cds_ft_metadata_to_item(metadata);
	if (ft_debug_counters()) {
		uatomic_inc(&ft->nr_nodes_allocated);
		uatomic_inc(&ft->nr_internal_alloc);
	}
	*meta_p = metadata;
	return node;
}

/*
 * QP-nibble allocator: deferred (call_rcu) free of a published node.
 * Use after the parent's slot has been atomically swapped to a new
 * node — readers that captured the old slot will continue descending
 * into @node and must complete a grace period before @node's memory
 * can be reused.
 */
static __attribute__((unused))
void ft_qp16_node_free_rcu(struct cds_ft *ft, struct cds_ft_qp16_node *node)
{
	struct cds_ft_metadata *metadata = cds_ft_item_to_metadata(node);

	cds_ft_free_item(ft, metadata);
	if (ft_debug_counters() && node) {
		uatomic_inc(&ft->nr_nodes_freed);
		uatomic_inc(&ft->nr_internal_freed);
	}
}

/*
 * QP-nibble allocator: immediate free for a node that never escaped
 * the writer's stack (e.g. a CoW destination abandoned mid-build by
 * an -ENOMEM error path).  See cds_ft_free_item_unpublished for the
 * safety contract.
 */
static __attribute__((unused))
void ft_qp16_node_free_unpublished(struct cds_ft *ft, struct cds_ft_qp16_node *node)
{
	struct cds_ft_metadata *metadata = cds_ft_item_to_metadata(node);

	cds_ft_free_item_unpublished(ft, metadata);
	if (ft_debug_counters() && node) {
		uatomic_inc(&ft->nr_nodes_freed);
		uatomic_inc(&ft->nr_internal_freed);
	}
}

/*
 * QP-nibble byte-step descent — chains hi+lo to produce byte-keyed
 * get_nth semantics on a QP byte stage.  Caller passes the hi-nibble
 * head node; the helper does:
 *
 *   1. hi descend (untagged lo-node ptr; bitmap is pre-filter, slot
 *      load is acquire and authoritative).  NULL → byte absent.
 *   2. lo descend on the resolved lo-node (tagged child; tag dispatch
 *      and prefetch hint applied).  NULL → byte absent (clear bit or
 *      tombstone slot).
 *
 * @ptr_slot_p, when non-NULL, returns the *lo-side* pointer slot — the
 * one a graft / replace caller would overwrite.  The hi-side slot
 * holds the lo-node and is not exposed: byte-level mutations operate
 * at the lo level.  Pure-read callers pass NULL.
 *
 * Two acquire-loads on the slot path; the bitmap pre-filters keep the
 * early-NULL exits predictable.
 */
static inline_lookup
struct cds_ft_inode_flag *ft_qp_byte_get(
		struct cds_ft_qp16_node *hi,
		struct cds_ft_inode_flag ***ptr_slot_p,
		uint8_t byte, enum ft_pf_target pf_hint, bool is_skip)
{
	struct cds_ft_inode_flag *lo_flag;
	struct cds_ft_qp16_node *lo;

	lo_flag = ft_qp16_node_descend(hi, NULL,
			(uint8_t) (byte >> 4), FT_PF_NONE, 0, is_skip);
	if (caa_unlikely(!lo_flag)) {
		if (caa_unlikely(ptr_slot_p))
			*ptr_slot_p = NULL;
		return NULL;
	}
	lo = (struct cds_ft_qp16_node *) lo_flag;
	/*
	 * LO is always direct: it is reached only via HI byte-step, never
	 * as a SKIP target.  Pass false unconditionally.
	 */
	return ft_qp16_node_descend(lo, ptr_slot_p,
			(uint8_t) (byte & 0xFU), pf_hint, 1, false);
}

/*
 * QP-nibble byte-step directional lookup — return the next/prev byte
 * with a live child relative to @byte_in, and that child.
 *
 * @dir == FT_RIGHT: smallest byte strictly greater than @byte_in.
 * @dir == FT_LEFT:  largest byte strictly less than @byte_in.
 *
 * Sentinel inputs (matching the byte-keyed get_minmax convention):
 *   @byte_in == -1  with FT_RIGHT → leftmost (smallest) byte.
 *   @byte_in == 256 with FT_LEFT  → rightmost (largest) byte.
 *
 * Returns NULL when no such byte exists in the node.  On success,
 * *@byte_out holds the matched byte (0..255).
 *
 * Walks the (hi, lo) bitmap lattice in @dir order.  Tombstones are
 * skipped two ways:
 *   - a NULL hi-slot (lo-node detached) skips the entire bucket;
 *   - a NULL lo-slot (tombstoned child) skips that lo bit and we
 *     keep scanning within the same lo-node.
 */
static inline_lookup
struct cds_ft_inode_flag *ft_qp_byte_get_direction(
		struct cds_ft_qp16_node *hi, int byte_in,
		uint8_t *byte_out, enum ft_direction dir, bool is_skip)
{
	uint16_t hi_bm = uatomic_load(&hi->bitmap, CMM_RELAXED);
	int hi_n_start, lo_n_start;
	int hi_iter, hi_step;
	int in_range = (byte_in >= 0 && byte_in < FT_ENTRY_PER_NODE);

	assert(dir == FT_LEFT || dir == FT_RIGHT);

	if (in_range) {
		hi_n_start = byte_in >> 4;
		lo_n_start = byte_in & 0xF;
	} else if (byte_in < 0) {
		/* leftmost sentinel: walk all 16 buckets in @dir order */
		hi_n_start = (dir == FT_RIGHT) ? 0 : -1;
		lo_n_start = 0;
	} else {
		/* rightmost sentinel (byte_in >= 256) */
		hi_n_start = (dir == FT_RIGHT) ? 16 : 15;
		lo_n_start = 0;
	}

	hi_step = (dir == FT_RIGHT) ? 1 : -1;
	for (hi_iter = hi_n_start;
			hi_iter >= 0 && hi_iter < 16;
			hi_iter += hi_step) {
		uint16_t hi_bit = (uint16_t) (1U << hi_iter);
		unsigned int hi_idx;
		struct cds_ft_inode_flag *lo_flag;
		struct cds_ft_qp16_node *lo;
		uint16_t lo_bm, side;

		if (!(hi_bm & hi_bit))
			continue;
		hi_idx = (unsigned int) __builtin_popcount(
				(unsigned int) (hi_bm & (hi_bit - 1U)));
		lo_flag = ft_dereference_acquire(
				ft_qp16_ptrs(hi, is_skip)[hi_idx]);
		if (!lo_flag)
			continue;
		lo = (struct cds_ft_qp16_node *) lo_flag;
		lo_bm = uatomic_load(&lo->bitmap, CMM_RELAXED);

		/*
		 * On the cursor's bucket, exclude @byte_in itself by
		 * masking the lo bitmap to the strict @dir half.  In any
		 * later bucket, the whole lo bitmap is in scope.
		 */
		if (in_range && hi_iter == hi_n_start) {
			if (dir == FT_RIGHT)
				side = (uint16_t) (lo_bm & (uint16_t)
					~((1U << ((unsigned int) lo_n_start + 1U))
						- 1U));
			else
				side = (uint16_t) (lo_bm &
					(uint16_t) ((1U << (unsigned int) lo_n_start) - 1U));
		} else {
			side = lo_bm;
		}

		while (side) {
			unsigned int lo_match;
			unsigned int lo_idx;
			struct cds_ft_inode_flag *child;

			if (dir == FT_RIGHT)
				lo_match = (unsigned int) __builtin_ctz(
						(unsigned int) side);
			else
				lo_match = 31U - (unsigned int) __builtin_clz(
						(unsigned int) side);
			lo_idx = (unsigned int) __builtin_popcount(
					(unsigned int) (lo_bm
						& ((1U << lo_match) - 1U)));
			child = ft_dereference_acquire(lo->ptrs[lo_idx]);
			if (child) {
				*byte_out = (uint8_t) (((unsigned int) hi_iter << 4)
						| lo_match);
				return child;
			}
			side &= (uint16_t) ~(1U << lo_match);
		}
	}
	return NULL;
}

/*
 * QP-nibble byte-step extremum — convenience wrapper that returns the
 * leftmost (smallest) or rightmost (largest) byte with a live child.
 *
 * Convention matches ft_qp16_node_get_extremum:
 *   @dir == FT_LEFT  → leftmost (smallest) byte.
 *   @dir == FT_RIGHT → rightmost (largest) byte.
 *
 * Implemented via the directional walk with sentinel @byte_in.
 */
static inline_lookup
struct cds_ft_inode_flag *ft_qp_byte_get_extremum(
		struct cds_ft_qp16_node *hi,
		uint8_t *byte_out, enum ft_direction dir, bool is_skip)
{
	assert(dir == FT_LEFT || dir == FT_RIGHT);
	if (dir == FT_LEFT)
		return ft_qp_byte_get_direction(hi, -1, byte_out, FT_RIGHT,
				is_skip);
	return ft_qp_byte_get_direction(hi, FT_ENTRY_PER_NODE, byte_out,
			FT_LEFT, is_skip);
}

/*
 * QP-nibble byte-step ith-position — return the i-th byte in
 * popcount-lattice order across (hi-bit, lo-bit) pairs, skipping
 * buckets whose hi-slot is tombstoned.
 *
 * Mirrors ft_pigeon_node_get_ith_pos / ft_qp16_node_get_ith_pos: i
 * indexes the popcount-counted lattice positions; a returned NULL
 * indicates an out-of-range i OR a tombstoned lo-slot at position i,
 * and the caller filters NULL just as for the other node types.
 *
 * Tombstoned hi-slots (lo-node detached) contribute zero to the
 * lattice count — the entire bucket is skipped.  This matches the
 * design: when a lo-node is freed, no lo bits are visible anymore.
 *
 * Linear walk; at most 16*16 = 256 bit tests in the worst case.  Used
 * by recompact / iter / debug paths, not by the hot lookup path.
 */
static inline_lookup
struct cds_ft_inode_flag *ft_qp_byte_get_ith_pos(
		struct cds_ft_qp16_node *hi, unsigned int i,
		uint8_t *byte_out, bool is_skip)
{
	uint16_t hi_bm = uatomic_load(&hi->bitmap, CMM_RELAXED);
	unsigned int count = 0;
	unsigned int hi_iter;

	for (hi_iter = 0; hi_iter < 16U; hi_iter++) {
		uint16_t hi_bit = (uint16_t) (1U << hi_iter);
		unsigned int hi_idx;
		struct cds_ft_inode_flag *lo_flag;
		struct cds_ft_qp16_node *lo;
		uint16_t lo_bm;
		unsigned int j;

		if (!(hi_bm & hi_bit))
			continue;
		hi_idx = (unsigned int) __builtin_popcount(
				(unsigned int) (hi_bm & (hi_bit - 1U)));
		lo_flag = ft_dereference_acquire(
				ft_qp16_ptrs(hi, is_skip)[hi_idx]);
		if (!lo_flag)
			continue;
		lo = (struct cds_ft_qp16_node *) lo_flag;
		lo_bm = uatomic_load(&lo->bitmap, CMM_RELAXED);

		for (j = 0; j < 16U; j++) {
			unsigned int lo_idx;

			if (!(lo_bm & (1U << j)))
				continue;
			if (count == i) {
				lo_idx = (unsigned int) __builtin_popcount(
						(unsigned int) (lo_bm
							& ((1U << j) - 1U)));
				*byte_out = (uint8_t) ((hi_iter << 4) | j);
				return ft_dereference_acquire(lo->ptrs[lo_idx]);
			}
			count++;
		}
	}
	return NULL;
}

/*
 * QP-nibble byte-step delete — tombstone the lo-side slot at @byte
 * and, if the lo-node's live-child count drops to zero, detach the
 * lo-node from the hi-node and RCU-free it (per design 4.6.1).
 *
 * @lo_slot is the lo-side pointer slot returned by a previous
 * ft_qp_byte_get; the caller has located it via the read-side
 * descent and passes it through to share the work.
 *
 * Mutation order matches ft_pigeon_node_replace_ptr / the byte-keyed
 * helpers — slot publish first, accounting after — and is safe under
 * the count-based readers' undercount guarantee: a reader that
 * acquire-loads a tombstoned slot has already passed (or is about to
 * skip) the bitmap pre-filter, so it observes "not found" without
 * needing the metadata count to be in sync.
 *
 *   1. rcu_assign_pointer(*lo_slot, NULL) — tombstones the byte.
 *   2. lo_meta->nr_child-- — local live count.
 *   3. If lo_meta->nr_child == 0 (lo-node fully tombstoned):
 *      a. rcu_assign_pointer(*hi_slot, NULL) — tombstones the bucket
 *         (hi-bit stays set per the monotonic-bitmap rule).
 *      b. hi_meta->nr_child-- — bucket count.
 *      c. ft_qp16_node_free_rcu(ft, lo) — defer free to the next
 *         grace period.  Concurrent readers either resolve the OLD
 *         hi-slot (descend into the now-empty lo-node — every lo-slot
 *         is NULL, returns not-found) or the NEW NULL hi-slot
 *         (returns not-found immediately).  Both correct.
 *
 * The lo-node's metadata is recovered via the slow pointer-mask
 * accessor (cds_ft_item_to_metadata) — the hi→lo pointer is raw and
 * untagged, so the tag-derived "_fast" accessor is unavailable.  This
 * is write-side only; read-side never needs lo-node metadata.
 *
 * Returns 0 on success.  The byte-stage recompact threshold
 * (-EFBIG-style trigger comparing hi_meta->nr_child against the QP
 * tier's min_child) is left to the dispatch arm and lands alongside
 * the case FT_QP: arms; this helper unconditionally completes the
 * detach when the live count hits zero.
 */
static __attribute__((unused))
int ft_qp_byte_clear(struct cds_ft *ft,
		struct cds_ft_qp16_node *hi, struct cds_ft_metadata *hi_meta,
		struct cds_ft_inode_flag **lo_slot, uint8_t byte, bool is_skip)
{
	uint8_t hi_n = (uint8_t) (byte >> 4);
	struct cds_ft_inode_flag **hi_slot;
	struct cds_ft_inode_flag *lo_flag;
	struct cds_ft_qp16_node *lo;
	struct cds_ft_metadata *lo_meta;

	assert(*lo_slot != NULL);
	rcu_assign_pointer(*lo_slot, NULL);

	lo_flag = ft_qp16_node_descend(hi, &hi_slot, hi_n, FT_PF_NONE, 0,
			is_skip);
	assert(lo_flag);
	lo = (struct cds_ft_qp16_node *) lo_flag;
	lo_meta = cds_ft_item_to_metadata(lo);

	assert(lo_meta->nr_child > 0);
	lo_meta->nr_child--;
	/*
	 * hi_meta->nr_child tracks total byte children (sum of lo_meta
	 * nr_child across hi-buckets), not popcount(hi_bm).  This matches
	 * the semantic used by ft_detach_node and ft_detach_descent_track.
	 */
	assert(hi_meta->nr_child > 0);
	hi_meta->nr_child--;
	if (lo_meta->nr_child > 0)
		return 0;

	/* Lo-node fully tombstoned: detach + RCU-free. */
	assert(*hi_slot != NULL);
	rcu_assign_pointer(*hi_slot, NULL);
	/* Half-CL accounting: lo leaves the subtree. */
	hi_meta->qp_subtree_half_cls -= (uint8_t)
		ft_node_half_cls(cds_ft_item_order(lo));
	ft_qp16_node_free_rcu(ft, lo);
	return 0;
}

/*
 * QP-nibble byte-step replace — atomic pointer replace at a byte
 * that is already present in the node, sharing the lo-slot pointer
 * the caller obtained via ft_qp_byte_get.
 *
 * @newptr non-NULL is a live replace (graft swap, candidate
 * promotion).  @newptr NULL routes to ft_qp_byte_clear so the
 * dispatch arm can use the same _ft_node_replace_ptr-style call site
 * for both replace and delete; the lo-node detach + free-RCU
 * machinery stays encapsulated here.
 *
 * Returns 0 on success.
 */
static __attribute__((unused))
int ft_qp_byte_replace(struct cds_ft *ft,
		struct cds_ft_qp16_node *hi, struct cds_ft_metadata *hi_meta,
		struct cds_ft_inode_flag **lo_slot,
		uint8_t byte, struct cds_ft_inode_flag *newptr, bool is_skip)
{
	if (!newptr)
		return ft_qp_byte_clear(ft, hi, hi_meta, lo_slot, byte, is_skip);
	assert(*lo_slot != NULL);
	rcu_assign_pointer(*lo_slot, newptr);
	return 0;
}

/*
 * QP-nibble byte-step insert — three paths sharing one entry point.
 *
 *   Path 1 — lo-node missing (hi-bit clear OR hi-tombstone):
 *     1. Allocate fresh lo at T0.
 *     2. Plant (lo_n, child) directly into the unpublished node:
 *        ptrs[0] = child, bitmap = 1<<lo_n, nr_child = 1, parent = hi_flag.
 *     3. Try ft_qp16_node_set_nth_safe on hi to install (hi_n, new_lo).
 *        - Success: hi_meta->nr_child++, return 0.
 *        - -ERANGE / -ENOSPC: free new_lo unpublished and return the
 *          error.  Hi-side tier-up is the dispatch arm's responsibility
 *          (same recompact-or-grow protocol as other byte-keyed types).
 *
 *   Path 2 — lo-node exists, in-place set_nth_safe in lo:
 *     - Success with the slot previously NULL (clear bit OR tombstone):
 *       lo_meta->nr_child++ and hi_meta->nr_child++ — both track
 *       byte-children counts (sum of lo nr_child across hi-buckets at
 *       the hi level), so each new live byte bumps both.
 *     - Success with the slot previously non-NULL: live replace,
 *       no metadata accounting change.
 *     - -ERANGE / -ENOSPC: lo-side CoW (Path 2b).
 *
 *   Path 2b — lo-side CoW with recompact_and_insert:
 *     1. new_live = lo_meta->nr_child + 1.
 *     2. new_order = ft_qp16_alloc_order(new_live), new_capacity from
 *        the tier table.
 *     3. Allocate new_lo at new_order.  recompact_and_insert drops
 *        any lo-tombstones and inserts (lo_n, child) in popcount order.
 *     4. Set new_lo_meta->nr_child = new_live and parent = hi_flag.
 *     5. rcu_assign_pointer(*hi_slot, new_lo) atomically swaps the
 *        bucket; the OLD lo is RCU-freed.  hi_meta->nr_child++ for
 *        the freshly-introduced byte at lo_n.
 *
 * @hi_flag is the tagged hi-node inode_flag (used to set the lo-node's
 * parent pointer).  @hi_capacity is the hi-node's tier capacity.
 *
 * Returns 0 on success.  -ERANGE / -ENOSPC are reserved for hi-side
 * tier-up signaling — they propagate from Path 1's set_nth_safe on
 * the hi-node.  Lo-side CoW is fully handled internally and never
 * surfaces those errors to the caller.
 */
static __attribute__((unused))
int ft_qp_byte_set(struct cds_ft *ft,
		struct cds_ft_inode_flag *hi_flag,
		struct cds_ft_qp16_node *hi, struct cds_ft_metadata *hi_meta,
		uint8_t byte, struct cds_ft_inode_flag *child,
		unsigned int hi_capacity, bool is_skip)
{
	uint8_t hi_n = (uint8_t) (byte >> 4);
	uint8_t lo_n = (uint8_t) (byte & 0xFU);
	struct cds_ft_inode_flag **hi_slot;
	struct cds_ft_inode_flag *lo_flag;
	struct cds_ft_qp16_node *lo;
	struct cds_ft_metadata *lo_meta;
	int ret;

	lo_flag = ft_qp16_node_descend(hi, &hi_slot, hi_n, FT_PF_NONE, 0,
			is_skip);

	if (!lo_flag) {
		/* Path 1: lo-node missing — lazy alloc + install in hi. */
		struct cds_ft_qp16_node *new_lo;
		struct cds_ft_metadata *new_lo_meta;
		struct cds_ft_inode_flag *new_lo_flag;

		new_lo = ft_qp16_node_alloc(ft,
				FT_QP16_T0_ALLOC_ORDER,
				&new_lo_meta);
		if (!new_lo)
			return -ENOMEM;
		new_lo->ptrs[0] = child;
		new_lo->bitmap = (uint16_t) (1U << lo_n);
		new_lo_meta->nr_child = 1;
		new_lo_meta->parent = hi_flag;
		new_lo_meta->is_lo = 1;	/* mark this metadata as a QP-nibble lo-node */
		new_lo_flag = ft_qp16_lo_flag(new_lo);

		/*
		 * Install RAW lo pointer in hi.ptrs[hi_n].  Reader's hi-side
		 * descent casts directly with no tag-strip; lo-flag is only
		 * needed for the upward parent walk via meta->parent.
		 */
		ret = ft_qp16_node_set_nth_safe(hi, hi_n,
				(struct cds_ft_inode_flag *) new_lo,
				hi_capacity, is_skip);
		if (ret < 0) {
			ft_qp16_node_free_unpublished(ft, new_lo);
			return ret;
		}
		/* hi_meta->nr_child = total byte children. +1 for the new byte. */
		hi_meta->nr_child++;
		/* Half-CL accounting: new T0 lo joins the subtree. */
		hi_meta->qp_subtree_half_cls += (uint8_t)
			ft_node_half_cls(FT_QP16_T0_ALLOC_ORDER);
		/*
		 * Direct child of new_lo: parent = tagged lo-flag, slot is in
		 * the lo-arena (8-bit skip_slot_offset is bounded).  Must run
		 * after new_lo is published in hi so a concurrent reader can
		 * resolve the parent walk back to the live tree.
		 */
		ft_set_parent(child, new_lo_flag, &new_lo->ptrs[0]);
		return 0;
	}

	lo = (struct cds_ft_qp16_node *) lo_flag;
	lo_meta = cds_ft_item_to_metadata(lo);

	{
		/* Peek the lo-slot to distinguish live-replace from
		 * insert/revival for nr_child accounting.  Relaxed load is
		 * sufficient on the write side (mutex-held). */
		uint16_t lo_bm = uatomic_load(&lo->bitmap, CMM_RELAXED);
		uint16_t lo_bit = (uint16_t) (1U << lo_n);
		struct cds_ft_inode_flag *existing = NULL;
		unsigned int lo_order =
			(unsigned int) cds_ft_item_order(lo);
		unsigned int lo_capacity =
			ft_qp16_capacity_from_order(lo_order);

		if (lo_bm & lo_bit) {
			unsigned int lo_idx = (unsigned int) __builtin_popcount(
					(unsigned int) (lo_bm & (lo_bit - 1U)));
			existing = uatomic_load(&lo->ptrs[lo_idx], CMM_RELAXED);
		}

		/* LO is always direct: never a SKIP target. */
		ret = ft_qp16_node_set_nth_safe(lo, lo_n, child, lo_capacity,
				false);
		if (ret == 0) {
			unsigned int lo_idx_after;
			uint16_t lo_bm_after =
				uatomic_load(&lo->bitmap, CMM_RELAXED);

			if (!existing) {
				lo_meta->nr_child++;
				hi_meta->nr_child++;
			}
			lo_idx_after = (unsigned int) __builtin_popcount(
					(unsigned int) (lo_bm_after
						& (lo_bit - 1U)));
			/*
			 * Build the tagged lo-flag for the upward parent
			 * walk; hi.ptrs[hi_n] now holds the raw lo pointer
			 * so the local @lo_flag variable is also raw.
			 */
			ft_set_parent(child, ft_qp16_lo_flag(lo),
					&lo->ptrs[lo_idx_after]);
			return 0;
		}
		if (ret != -ERANGE && ret != -ENOSPC)
			return ret;

		/* Path 2b: lo-side CoW. */
		{
			struct cds_ft_qp16_node *new_lo;
			struct cds_ft_metadata *new_lo_meta;
			struct cds_ft_inode_flag *new_lo_flag;
			unsigned int new_live = lo_meta->nr_child + 1U;
			unsigned int new_order = ft_qp16_alloc_order(new_live);
			unsigned int new_capacity =
				ft_qp16_capacity_from_order(new_order);
			uint16_t new_bm;
			unsigned int b;

			new_lo = ft_qp16_node_alloc(ft, new_order,
					&new_lo_meta);
			if (!new_lo)
				return -ENOMEM;
			/* LO is always direct: never a SKIP target. */
			ret = ft_qp16_node_recompact_and_insert(new_lo, lo,
					lo_n, child, new_capacity, false);
			if (ret < 0) {
				ft_qp16_node_free_unpublished(ft, new_lo);
				return ret;
			}
			new_lo_meta->nr_child = new_live;
			new_lo_meta->parent = hi_flag;
			new_lo_meta->is_lo = 1;	/* mark this metadata as a QP-nibble lo-node */
			new_lo_flag = ft_qp16_lo_flag(new_lo);
			/*
			 * Half-CL accounting: replace old lo's footprint with
			 * the new (potentially larger) lo's.  When the old
			 * order equals the new order (in-place rebuild without
			 * tier-up), this is a no-op.
			 */
			hi_meta->qp_subtree_half_cls += (uint8_t)
				ft_node_half_cls(new_order);
			hi_meta->qp_subtree_half_cls -= (uint8_t)
				ft_node_half_cls(lo_order);
			/*
			 * Publish RAW lo pointer in hi.  Children's
			 * meta->parent (set below) keeps the tagged lo-flag.
			 */
			rcu_assign_pointer(*hi_slot,
				(struct cds_ft_inode_flag *) new_lo);
			/*
			 * Reparent every surviving child of new_lo to the new
			 * lo-flag.  Includes the freshly-inserted @child.  The
			 * old lo's slot addresses are stale; children must
			 * track the new lo-arena slot (skip_slot_offset).
			 */
			new_bm = new_lo->bitmap;
			for (b = 0; b < 16U; b++) {
				unsigned int new_idx;
				struct cds_ft_inode_flag *iter;

				if (!(new_bm & (uint16_t) (1U << b)))
					continue;
				new_idx = (unsigned int) __builtin_popcount(
						(unsigned int) (new_bm
							& ((1U << b) - 1U)));
				iter = new_lo->ptrs[new_idx];
				if (!iter)
					continue;
				ft_set_parent(iter, new_lo_flag,
					&new_lo->ptrs[new_idx]);
			}
			ft_qp16_node_free_rcu(ft, lo);
			hi_meta->nr_child++;
			return 0;
		}
	}
}

/*
 * ft_node_get_nth_skip: raw child slot access.  Returns the slot
 * value as-is, including skip-compressed pointers.  node_flag is
 * already rcu_dereference'd.  Used only by candidate lookup which
 * resolves skip pointers itself.
 */
static inline_lookup
struct cds_ft_inode_flag *ft_node_get_nth_skip(struct cds_ft_inode_flag *node_flag,
		struct cds_ft_inode_flag ***node_flag_ptr,
		uint8_t n, enum ft_pf_target pf_hint)
{
	unsigned long v = (unsigned long) node_flag;

	/*
	 * Candidate E: direct-internal kinds (PIGEON 0x01, QP 0x05,
	 * future POPCOUNT_* 0x09 / 0x11) have bit 0 set AND bit 1
	 * clear.  Single AND + CMP rejects EXT (bit 0 clear),
	 * COMPRESSED (bit 0 clear), and all SKIP_X (bit 1 set) — none
	 * of which have a byte-step descent.  In normal descent,
	 * callers route skip pointers through the skip handler before
	 * reaching here, so this filter is just a safety net.
	 */
	if (caa_unlikely((v & 0x03UL) != 0x01UL)) {
		if (caa_unlikely(node_flag_ptr))
			*node_flag_ptr = NULL;
		return NULL;
	}

	/*
	 * Internal dispatch: FT_KIND_QP (0x05) routes to QP byte-step;
	 * FT_KIND_PIGEON (0x01) routes to PIGEON.  The full 5-bit tag
	 * is clean here because direct-internal kinds are 32-byte+
	 * aligned (POPCOUNT_64 is 64-byte aligned).  Lo-nodes share the
	 * QP tag (HI/LO disambiguated via metadata is_lo bit during the
	 * parent walk) but never appear as a slot value here — the
	 * descent reaches them only inside ft_qp_byte_get's HI→LO chain.
	 *
	 * Untag with a constant SUB inside each branch: the SUB has no
	 * dependency on `tag`, lets the address compute issue one cycle
	 * earlier, and the prefetcher's stride detector treats the
	 * result as a linear pointer offset.
	 *
	 * Internal-direct kinds at this point: PIGEON (0x01), QP (0x05),
	 * POPCOUNT_32 (0x09), POPCOUNT_64 (0x11).  Distinguishing tag
	 * bits (one-hot on bits 2-4, plus bit 0 alignment):
	 *   bit 2 = QP (0x05 only)
	 *   bit 3 = POPCOUNT_32 (0x09 only)
	 *   bit 4 = POPCOUNT_64 (0x11 only)
	 *   none of bits 2/3/4 = PIGEON (0x01)
	 * Hot path likely caa_likely(QP); POPCOUNT_* second; PIGEON last.
	 */
	if (caa_likely((v & 0x04UL) != 0)) {
		struct cds_ft_qp16_node *qp = (struct cds_ft_qp16_node *)
				((unsigned long) node_flag - FT_KIND_QP);
#ifdef FEATURE_FT_SKIP_COMPRESSED
		/*
		 * metadata->is_skip is authoritative: cn->child carries
		 * FT_KIND_QP regardless of variant, so the tag alone cannot
		 * distinguish skip-variant from direct.  Skip-variant ptrs[]
		 * are at offset 16 instead of 8 — wrong is_skip here reads
		 * the bitmap of bogus memory.
		 */
		bool is_skip = cds_ft_item_to_metadata(qp)->is_skip;
#else
		bool is_skip = false;
#endif
		return ft_qp_byte_get(qp, node_flag_ptr, n, pf_hint, is_skip);
	}
	if ((v & 0x08UL) != 0)
		return ft_pc32_node_get_nth_skip(
				(struct ft_pc32_node *)
				((unsigned long) node_flag - FT_KIND_POPCOUNT_32),
				node_flag_ptr, n, pf_hint);
	if ((v & 0x10UL) != 0)
		return ft_pc64_node_get_nth_skip(
				(struct ft_pc64_node *)
				((unsigned long) node_flag - FT_KIND_POPCOUNT_64),
				node_flag_ptr, n, pf_hint);
	return ft_pigeon_node_get_nth(NULL,
			(struct cds_ft_inode *)
			((unsigned long) node_flag - FT_KIND_PIGEON),
			node_flag_ptr, n, pf_hint);
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
 * Only handles internal node types (linear, pool, pigeon).
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
	unsigned int type_index = ft_node_type_index(parent_nf);
	const struct cds_ft_type *type = &ft_types[type_index];

	switch (type->type_class) {
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
	case FT_QP:
	{
		struct cds_ft_qp16_node *hi = (struct cds_ft_qp16_node *) node;
		uint16_t hi_bm = uatomic_load(&hi->bitmap, CMM_RELAXED);
		/*
		 * is_skip comes from metadata->is_skip (authoritative across
		 * cn->child / SKIP_QP wrapper access paths) rather than the
		 * parent_nf slot tag, which only carries FT_KIND_SKIP_QP on
		 * the SKIP wrapper path.
		 */
#ifdef FEATURE_FT_SKIP_COMPRESSED
		bool is_skip = cds_ft_item_to_metadata(node)->is_skip;
#else
		bool is_skip = false;
#endif
		unsigned int hi_iter;

		/*
		 * Walk the (hi, lo) bitmap lattice directly — get_ith_pos
		 * conflates "past end" with "tombstone slot at this i", and
		 * we want to skip tombstones cleanly.  At most 16 * 16 =
		 * 256 bit tests with early exit on the first matching child.
		 */
		for (hi_iter = 0; hi_iter < 16U; hi_iter++) {
			uint16_t hi_bit = (uint16_t) (1U << hi_iter);
			unsigned int hi_idx;
			struct cds_ft_inode_flag *lo_flag;
			struct cds_ft_qp16_node *lo;
			uint16_t lo_bm;
			unsigned int j;

			if (!(hi_bm & hi_bit))
				continue;
			hi_idx = (unsigned int) __builtin_popcount(
					(unsigned int) (hi_bm & (hi_bit - 1U)));
			lo_flag = ft_dereference_acquire(
					ft_qp16_ptrs(hi, is_skip)[hi_idx]);
			if (!lo_flag)
				continue;
			lo = (struct cds_ft_qp16_node *) lo_flag;
			lo_bm = uatomic_load(&lo->bitmap, CMM_RELAXED);

			for (j = 0; j < 16U; j++) {
				unsigned int lo_idx;
				struct cds_ft_inode_flag *iter;

				if (!(lo_bm & (1U << j)))
					continue;
				lo_idx = (unsigned int) __builtin_popcount(
						(unsigned int) (lo_bm
							& ((1U << j) - 1U)));
				iter = ft_dereference_acquire(lo->ptrs[lo_idx]);
				if (iter == child_nf) {
					uint8_t byte = (uint8_t) ((hi_iter << 4) | j);

					if (n_ret)
						*n_ret = byte;
					if (slot_ret)
						ft_node_get_nth(parent_nf, slot_ret,
								byte, FT_PF_NONE);
					return true;
				}
			}
		}
		return false;
	}
	case FT_POPCOUNT:
	{
		unsigned int nr;
		unsigned int i;

		if (type_index == FT_POPCOUNT_64_INDEX) {
			struct ft_pc64_node *pc = (struct ft_pc64_node *) node;

			nr = ft_pc64_node_get_nr_child(pc);
			for (i = 0; i < nr; i++) {
				uint8_t v = 0;
				struct cds_ft_inode_flag *iter =
					ft_pc64_node_get_ith_pos(pc, i, &v);

				if (iter == child_nf) {
					if (n_ret)
						*n_ret = v;
					if (slot_ret)
						ft_node_get_nth(parent_nf, slot_ret,
								v, FT_PF_NONE);
					return true;
				}
			}
		} else {
			struct ft_pc32_node *pc = (struct ft_pc32_node *) node;

			nr = ft_pc32_node_get_nr_child(pc);
			for (i = 0; i < nr; i++) {
				uint8_t v = 0;
				struct cds_ft_inode_flag *iter =
					ft_pc32_node_get_ith_pos(pc, i, &v);

				if (iter == child_nf) {
					if (n_ret)
						*n_ret = v;
					if (slot_ret)
						ft_node_get_nth(parent_nf, slot_ret,
								v, FT_PF_NONE);
					return true;
				}
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
	if (ft_node_compressed_in_node(node_flag))
		return NULL;
	node = ft_node_ptr(node_flag);
	assert(node != NULL);
	type_index = ft_node_type_index(node_flag);
	type = &ft_types[type_index];

	switch (type->type_class) {
	case FT_PIGEON:
		child = ft_pigeon_node_get_direction(type, node, n, result_key, dir);
		break;
	case FT_QP:
	{
		/* metadata->is_skip is authoritative; see FT_QP arms in
		 * _ft_node_set_nth and ft_node_find_child.
		 */
#ifdef FEATURE_FT_SKIP_COMPRESSED
		bool is_skip = cds_ft_item_to_metadata(node)->is_skip;
#else
		bool is_skip = false;
#endif

		child = ft_qp_byte_get_direction(
				(struct cds_ft_qp16_node *) node,
				n, result_key, dir, is_skip);
		break;
	}
	case FT_POPCOUNT:
		if (type_index == FT_POPCOUNT_64_INDEX)
			child = ft_pc64_node_get_direction(
					(struct ft_pc64_node *) node,
					n, result_key, dir);
		else
			child = ft_pc32_node_get_direction(
					(struct ft_pc32_node *) node,
					n, result_key, dir);
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
int _ft_node_set_nth(struct cds_ft *ft,
		const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		struct cds_ft_inode_flag *node_flag,
		struct cds_ft_metadata *metadata,
		uint8_t n,
		struct cds_ft_inode_flag *child_node_flag)
{
	int ret;

	switch (type->type_class) {
	case FT_PIGEON:
		ret = ft_pigeon_node_set_nth(type, node, metadata, n, child_node_flag);
		break;
	case FT_POPCOUNT:
	{
		/*
		 * Two POPCOUNT layouts share FT_POPCOUNT type_class — pc32
		 * (scan_3, max_lc=3) and pc64 (scan_6, max_lc=6).  Dispatch
		 * by type->order: 5 = pc32, 6 = pc64.  Each picks max_lc
		 * from the metadata skip-mode bit (skip variant reserves
		 * slot 0 for ft_pc32_skip_meta, dropping max_lc by one).
		 *
		 * Returns -ERANGE on non-safe-append insert, routing
		 * through recompact ADD_SAME for an off-tree rebuild.
		 * Returns -ENOSPC on capacity overflow, routing through
		 * recompact ADD_NEXT (pc32 → pc64 → QP escalation).
		 */
		if (type->order == FT_PC64_ALLOC_ORDER) {
			unsigned int max_lc = FT_PC64_MAX_LC_DIRECT;
#ifdef FEATURE_FT_SKIP_COMPRESSED
			if (metadata->is_skip)
				max_lc = FT_PC64_MAX_LC_SKIP;
#endif
			ret = ft_pc64_node_set_nth_safe(
					(struct ft_pc64_node *) node, metadata,
					n, child_node_flag, max_lc);
		} else {
			unsigned int max_lc = FT_PC32_MAX_LC_DIRECT;
#ifdef FEATURE_FT_SKIP_COMPRESSED
			if (metadata->is_skip)
				max_lc = FT_PC32_MAX_LC_SKIP;
#endif
			ret = ft_pc32_node_set_nth_safe(
					(struct ft_pc32_node *) node, metadata,
					n, child_node_flag, max_lc);
		}
		break;
	}
	case FT_QP:
	{
		/*
		 * hi_capacity is the structural hi-bucket count
		 * (popcount(hi_bm) cap), derived from the tier order:
		 * T0..T3 = 3 / 7 / 15 / 16.  Recover the actual order
		 * from cds_ft_item_order(node) — the flattened
		 * ft_types[FT_QP_INDEX].order carries only the T0
		 * default for fresh allocations and would mis-cap nodes
		 * promoted to higher tiers.  type->max_child is the
		 * *byte-children* count (hi_capacity * 16) used by the
		 * recompact framework's nr_child accounting; passing it
		 * as hi_capacity would let set_nth_safe overflow the
		 * pointer array.
		 *
		 * is_skip comes from metadata->is_skip — the authoritative
		 * per-node signal set by ft_publish_compressed's QP
		 * migration.  The slot tag in node_flag is unreliable here:
		 * descent reaching this QP HI via cn->child carries
		 * FT_KIND_QP regardless of variant, while descent via the
		 * SKIP_QP wrapper carries FT_KIND_SKIP_QP.  Skip-variant
		 * capacity drops by one (the highest-rank ptr slot is
		 * overlaid by the cached subkey extension).
		 */
#ifdef FEATURE_FT_SKIP_COMPRESSED
		bool is_skip = metadata && metadata->is_skip;
#else
		bool is_skip = false;
#endif
		unsigned int hi_capacity = ft_qp16_capacity_from_order(
				(unsigned int) cds_ft_item_order(node));

		if (is_skip)
			hi_capacity -= 1;

		ret = ft_qp_byte_set(ft, node_flag,
				(struct cds_ft_qp16_node *) node, metadata,
				n, child_node_flag,
				hi_capacity, is_skip);
		break;
	}
	case FT_NULL:
		return -ENOSPC;
	default:
		assert(0);
		return -EINVAL;
	}
	return ret;
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
 *
 * @ft is needed only by the FT_QP arm — ft_qp_byte_replace routes
 * NULL @newptr to ft_qp_byte_clear, which may RCU-free a now-empty
 * lo-node (decision 4.6.1).  Other arms ignore @ft.
 */
static
int _ft_node_replace_ptr(struct cds_ft *ft __attribute__((unused)),
		const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		struct cds_ft_inode_flag *node_flag,
		struct cds_ft_metadata *metadata,
		struct cds_ft_inode_flag **node_flag_ptr,
		uint8_t n, struct cds_ft_inode_flag *newptr)
{
	int ret;

	switch (type->type_class) {
	case FT_PIGEON:
		ret = ft_pigeon_node_replace_ptr(type, node, metadata, node_flag_ptr, n, newptr);
		break;
	case FT_POPCOUNT:
		if (type->order == FT_PC64_ALLOC_ORDER)
			ret = ft_pc64_node_replace_ptr(
					(struct ft_pc64_node *) node, metadata,
					node_flag_ptr, n, newptr);
		else
			ret = ft_pc32_node_replace_ptr(
					(struct ft_pc32_node *) node, metadata,
					node_flag_ptr, n, newptr);
		break;
	case FT_QP:
	{
		/* See _ft_node_set_nth FT_QP arm: metadata->is_skip is
		 * authoritative across cn->child / SKIP_QP wrapper paths.
		 */
#ifdef FEATURE_FT_SKIP_COMPRESSED
		bool is_skip = metadata && metadata->is_skip;
#else
		bool is_skip = false;
#endif

		ret = ft_qp_byte_replace(ft,
				(struct cds_ft_qp16_node *) node, metadata,
				node_flag_ptr, n, newptr, is_skip);
		break;
	}
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
		unsigned int nr_nodes, bool is_root, bool is_skip)
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
	/*
	 * Each type entry carries direct- and skip-mode bounds.  When the
	 * node is a skip target (e.g., POPCOUNT_32 with slot 0 reserved
	 * for ft_pc32_skip_meta), use the skip pair, which has reduced
	 * max_child for types that lose a slot to inline metadata.  For
	 * types with identical bounds across modes (QP, PIGEON), this is
	 * equivalent to the direct walk.
	 */
	for (;;) {
		unsigned int min, max;

		type = &ft_types[type_index];
		min = is_skip ? type->min_child_skip : type->min_child;
		max = is_skip ? type->max_child_skip : type->max_child;
		if (nr_nodes < min)
			type_index--;
		else if (nr_nodes > max)
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
		bool is_root)
{
	unsigned int new_type_index;
	struct cds_ft_inode *new_node;
	struct cds_ft_metadata *new_metadata;
	const struct cds_ft_type *new_type;
	struct cds_ft_inode_flag *new_node_flag = NULL;
	/*
	 * Inherit is_skip from the old metadata: if the old
	 * node was a skip-target POPCOUNT_32 with slot 0 reserved for
	 * cached subkey, the recompacted node is in the same skip-target
	 * context and uses the same reduced max_child for its lattice
	 * walk and copy-loop max_lc.  Cleared for non-POPCOUNT classes
	 * (the bit is meaningless there) and for fresh allocations.
	 */
#ifdef FEATURE_FT_SKIP_COMPRESSED
	bool is_skip = metadata && metadata->is_skip;
#else
	bool is_skip = false;
#endif
	/*
	 * Effective allocation order for the new node.  For non-QP
	 * classes equals new_type->order; for QP, the QP-internal
	 * tier picker (popcount-driven) overrides ft_types' default
	 * since the flattened ft_types[FT_QP_INDEX] only carries the
	 * T0 default.  Initialised to a sentinel; assigned by every
	 * arm before the alloc.
	 */
	unsigned int new_alloc_order = 0;
	int ret;

	switch (mode) {
	case FT_RECOMPACT_ADD_SAME:
		/*
		 * FT_QP -ERANGE recompact: rebuild + insert new hi-bucket
		 * below an existing higher bucket.  popcount grows by 1.
		 * If old's hi was at full structural capacity (popcount ==
		 * hi_capacity), tier up; else same-tier rebuild suffices
		 * (the merged walk weaves the new byte in nibble order).
		 * At T3 + full capacity, escalate to PIGEON (no higher QP
		 * tier).
		 */
		if (old_type->type_class == FT_QP) {
			struct cds_ft_qp16_node *qp_old =
				(struct cds_ft_qp16_node *) old_node;
			unsigned int old_pop = (unsigned int) __builtin_popcount(
					uatomic_load(&qp_old->bitmap, CMM_RELAXED));
			unsigned int old_order = (unsigned int)
				cds_ft_item_order(old_node);
			unsigned int old_tier = old_order - FT_QP16_T0_ALLOC_ORDER;
			unsigned int old_cap = ft_qp16_capacity_from_order(old_order);

			if (old_pop >= old_cap
			    && old_tier + 1 >= FT_QP16_NR_TIERS) {
				/* T3 full -> PIGEON escalation. */
				new_type_index = FT_PIGEON_INDEX;
				dbg_printf("Recompact FT_QP add-same T3 -> PIGEON (pop=%u cap=%u)\n",
					old_pop, old_cap);
			} else {
				unsigned int new_tier =
					(old_pop >= old_cap) ?
					old_tier + 1 : old_tier;

				new_type_index = FT_QP_INDEX;
				new_alloc_order = ft_qp16_tiers[new_tier].order;
				dbg_printf("Recompact FT_QP add-same: tier %u -> %u (pop=%u cap=%u)\n",
					old_tier, new_tier, old_pop, old_cap);
			}
			break;
		}
		new_type_index = find_nearest_type_index(old_type_index,
			metadata->nr_child + 1, false, is_skip);
		dbg_printf("Recompact for node with %u children\n",
			metadata->nr_child + 1);
		break;
	case FT_RECOMPACT_ADD_NEXT:
		if (!metadata || old_type_index == NODE_INDEX_NULL) {
			new_type_index = 0;
			dbg_printf("Recompact for NULL\n");
		} else {
			/*
			 * FT_QP -ENOSPC recompact: structural hi-bucket overflow
			 * (popcount(hi_bm) == hi_capacity, new bit needed).  Tier
			 * up within QP by exactly one — byte-count semantic on
			 * max_child does not capture the structural trigger.  At
			 * QP T3 (highest tier, 16-bit hi-bitmap full), there is
			 * no higher QP tier; escalate to PIGEON directly.  Note
			 * that the CL-footprint trigger (qp_subtree_half_cls >
			 * FT_PIGEON_HALF_CLS) usually fires first, but a worst-
			 * case shape can still reach T3 ENOSPC without crossing
			 * the footprint threshold.
			 */
			if (old_type->type_class == FT_QP) {
				unsigned int old_order = (unsigned int)
					cds_ft_item_order(old_node);
				unsigned int old_tier = old_order
					- FT_QP16_T0_ALLOC_ORDER;

				if (old_tier + 1 < FT_QP16_NR_TIERS) {
					unsigned int new_tier = old_tier + 1;

					new_type_index = FT_QP_INDEX;
					new_alloc_order =
						ft_qp16_tiers[new_tier].order;
					dbg_printf("Recompact FT_QP tier-up %u -> %u\n",
						old_tier, new_tier);
				} else {
					/* T3 -> PIGEON escalation. */
					new_type_index = FT_PIGEON_INDEX;
					dbg_printf("Recompact FT_QP T3 -> PIGEON\n");
				}
				break;
			}
			new_type_index = find_nearest_type_index(old_type_index,
				metadata->nr_child + 1, false, is_skip);
			/*
			 * POPCOUNT_X -> QP escalation: pick a QP tier whose
			 * structural hi-bucket capacity admits the worst-case
			 * byte distribution (each byte in a distinct hi-nibble).
			 * Worst-case popcount(hi_bm) = nr_child + 1.  Default
			 * ft_types[FT_QP_INDEX].order (T0, cap 3) is too small
			 * for any source with nr_child >= 3.
			 */
			if (new_type_index == FT_QP_INDEX
			    && old_type->type_class == FT_POPCOUNT)
				new_alloc_order = ft_qp16_alloc_order(
						metadata->nr_child + 1);
			dbg_printf("Recompact for node with %u children\n",
				metadata->nr_child + 1);
		}
		break;
	case FT_RECOMPACT_DEL:
		/*
		 * FT_QP DEL recompact: byte_clear returned -EFBIG signalling
		 * either tier-down (post-decrement byte count) or full
		 * elision when removing the last byte child.  When the new
		 * byte count would be 0, return NODE_INDEX_NULL (elide);
		 * otherwise stay at the current tier — there is no fine-
		 * grained tier-down policy yet (deferred).
		 */
		if (old_type->type_class == FT_QP) {
			if (metadata->nr_child <= 1) {
				new_type_index = is_root ? FT_QP_INDEX : NODE_INDEX_NULL;
				if (new_type_index != NODE_INDEX_NULL)
					new_alloc_order = FT_QP16_T0_ALLOC_ORDER;
			} else {
				new_type_index = FT_QP_INDEX;
				new_alloc_order = (unsigned int)
					cds_ft_item_order(old_node);
			}
			dbg_printf("Recompact FT_QP del tier (nr_child %u)\n",
				metadata->nr_child);
			break;
		}
		new_type_index = find_nearest_type_index(old_type_index,
			metadata->nr_child - 1, is_root, is_skip);
		/*
		 * PIGEON -> QP demotion: lattice walk lands at FT_QP_INDEX
		 * but ft_types[FT_QP_INDEX].order is the T0 default (3 hi-
		 * bucket capacity) — too small for any PIGEON population
		 * that crossed the hysteresis threshold.  Pick T3 (16 hi-
		 * bucket capacity) to accommodate any byte-child
		 * distribution.  Subsequent FT_QP DEL passes stay at the
		 * current order (no internal tier-down policy yet).
		 */
		if (new_type_index == FT_QP_INDEX
		    && old_type->type_class == FT_PIGEON)
			new_alloc_order = FT_QP16_T3_ALLOC_ORDER;
		dbg_printf("Recompact for node with %u children\n",
			metadata->nr_child - 1);
		break;
	case FT_RECOMPACT_REPARENT:
		/* Same-type clone: no add, no delete, no resize. */
		new_type_index = old_type_index;
		if (old_type->type_class == FT_QP)
			new_alloc_order = (unsigned int)
				cds_ft_item_order(old_node);
		break;
	default:
		assert(0);
	}

	new_metadata = NULL;
	dbg_printf("Recompact from type %d to type %d\n",
			old_type_index, new_type_index);
	new_type = &ft_types[new_type_index];
	if (new_type_index != NODE_INDEX_NULL) {
		/*
		 * Switch arms above leave new_alloc_order at 0 unless they
		 * have a tier-specific override (QP popcount-driven tier-up,
		 * PIGEON->QP demote landing at T3).  When unset, fall back
		 * to new_type->order — which is correct for non-QP types and
		 * the T0 default for fresh QP allocations.
		 */
		if (new_alloc_order == 0)
			new_alloc_order = new_type->order;
		new_node = alloc_cds_ft_node_at_order(ft, new_alloc_order,
				new_type->bitmap, &new_metadata);
		if (!new_node)
			return -ENOMEM;

		new_node_flag = ft_node_flag(new_node, new_type_index);

		dbg_printf("Recompact inherit from %p\n", metadata);
		if (metadata) {
			new_metadata->parent = metadata->parent;
#ifdef FEATURE_FT_SKIP_COMPRESSED
			new_metadata->skip_slot_offset = metadata->skip_slot_offset;
			/*
			 * Inherit skip-target marker.  Only meaningful for
			 * FT_POPCOUNT new types; on FT_QP / FT_PIGEON the bit
			 * is harmless (they read pigeon_skip_len /
			 * qp->skip_len, not is_skip).  Cleared if the
			 * recompact is changing type_class to something that
			 * doesn't honor the bit, since it would be stale on the
			 * new type.
			 *
			 * REPARENT mode: clone_for_reparent moves the node to a
			 * new parent context built by the caller.  The new
			 * parent may be non-COMPRESSED (e.g., compressed-split's
			 * branch internal in the suffix_len == 0 path), in
			 * which case is_skip would falsely persist on
			 * a node no longer reached via SKIP_POPCOUNT_X.  Clear
			 * here unconditionally for REPARENT; if the caller
			 * re-installs the clone under a fresh CN and calls
			 * ft_publish_compressed, the bit gets re-set with a
			 * fresh slot-0 skip_meta.
			 */
			new_metadata->is_skip =
				(mode != FT_RECOMPACT_REPARENT
				 && new_type_index != NODE_INDEX_NULL
				 && (new_type->type_class == FT_POPCOUNT
				     || new_type->type_class == FT_QP))
				? metadata->is_skip
				: 0;
			/*
			 * Copy the cached subkey (slot 0) when both old and
			 * new are FT_POPCOUNT in skip mode.  The COPY phase
			 * below walks children only; slot 0 in skip-mode
			 * holds ft_pc32_skip_meta (skip_len + subkey), not
			 * a child ptr, so it would otherwise be left zeroed
			 * on the new node.  Subsequent SKIP_POPCOUNT_*
			 * lookups would see zero subkey and reject inline.
			 *
			 * pc32 and pc64 share struct ft_pc32_skip_meta as
			 * the slot-0 form; the byte offset of slot 0 differs
			 * (8 vs 16) so the source/dest pointer is computed
			 * per type.
			 */
			if (new_metadata->is_skip
			    && old_type->type_class == FT_POPCOUNT) {
				const struct ft_pc32_skip_meta *old_meta;
				struct ft_pc32_skip_meta *new_meta;

				if (old_type->order == FT_PC64_ALLOC_ORDER) {
					old_meta = &((const struct ft_pc64_node *) old_node)->u.skip.meta;
				} else {
					old_meta = &((const struct ft_pc32_node *) old_node)->u.skip.meta;
				}
				if (new_type->order == FT_PC64_ALLOC_ORDER) {
					new_meta = &((struct ft_pc64_node *) new_node)->u.skip.meta;
				} else {
					new_meta = &((struct ft_pc32_node *) new_node)->u.skip.meta;
				}
				*new_meta = *old_meta;
			}
#endif
			new_metadata->fallback_removal_count = metadata->fallback_removal_count;
			ft_metadata_set_external_nodes(new_node_flag,
				new_metadata, metadata->external_nodes);
			ft_nr_keys_store(ft, new_metadata,
				ft_nr_keys_get(metadata), CMM_RELAXED);
		}
		/*
		 * Initialize half-CL accounting for the new node.  For
		 * FT_QP, the lattice walk's Path-1 / Path-2b calls in
		 * _ft_node_set_nth accumulate the lo costs as bytes are
		 * inserted; the hi's own footprint is seeded here.  Use
		 * the QP-internal alloc order (per-tier) rather than the
		 * flattened ft_types[FT_QP_INDEX].order (T0 default).  For
		 * FT_PIGEON the field is unused (a different union arm).
		 */
		if (new_type->type_class == FT_QP)
			new_metadata->qp_subtree_half_cls =
				(uint8_t) ft_node_half_cls(new_alloc_order);
	} else {
		new_node = NULL;
		new_node_flag = NULL;
	}

	assert(mode != FT_RECOMPACT_ADD_NEXT || old_type->type_class != FT_PIGEON);

	if (new_type_index == NODE_INDEX_NULL)
		goto skip_copy;

	switch (old_type->type_class) {
	case FT_NULL:
		assert(mode == FT_RECOMPACT_ADD_NEXT);
		break;
	case FT_PIGEON:
	{
		unsigned int i;

		assert(mode == FT_RECOMPACT_DEL ||
		       mode == FT_RECOMPACT_REPARENT);
		for (i = 0; i < FT_ENTRY_PER_NODE; i++) {
			struct cds_ft_inode_flag *iter;

			iter = ft_pigeon_node_get_ith_pos(old_type, old_node, i);
			if (!iter)
				continue;
			if (mode == FT_RECOMPACT_DEL && *nullify_node_flag_ptr == iter)
				continue;
			ret = _ft_node_set_nth(ft, new_type, new_node, new_node_flag,
					new_metadata, i, iter);
			assert(!ret);
		}
		break;
	}
	case FT_POPCOUNT:
	{
		bool insert_new = (mode == FT_RECOMPACT_ADD_NEXT
				|| mode == FT_RECOMPACT_ADD_SAME);
		bool new_inserted = false;
		unsigned int old_nr;
		unsigned int i;

		/*
		 * Walk old POPCOUNT entries in popcount-rank order (low
		 * rank first).  For ADD modes, weave the new (n,
		 * child_node_flag) at the right rank so the destination
		 * is filled strictly safe-append.  Recompact DEL skips
		 * the to-be-removed entry.  Two layouts share
		 * FT_POPCOUNT type_class: pc32 (order 5) vs pc64 (order
		 * 6); dispatch by old_type->order.
		 */
		if (old_type->order == FT_PC64_ALLOC_ORDER) {
			struct ft_pc64_node *old_pc =
				(struct ft_pc64_node *) old_node;

			old_nr = ft_pc64_node_get_nr_child(old_pc);
			for (i = 0; i < old_nr; i++) {
				uint8_t v = 0;
				struct cds_ft_inode_flag *iter =
					ft_pc64_node_get_ith_pos(old_pc, i, &v);

				if (insert_new && !new_inserted
				    && (unsigned int) v > (unsigned int) n) {
					ret = _ft_node_set_nth(ft, new_type,
						new_node, new_node_flag,
						new_metadata, n, child_node_flag);
					assert(!ret);
					new_inserted = true;
				}
				if (mode == FT_RECOMPACT_DEL
				    && *nullify_node_flag_ptr == iter)
					continue;
				ret = _ft_node_set_nth(ft, new_type, new_node,
					new_node_flag, new_metadata, v, iter);
				assert(!ret);
			}
		} else {
			struct ft_pc32_node *old_pc =
				(struct ft_pc32_node *) old_node;

			old_nr = ft_pc32_node_get_nr_child(old_pc);
			for (i = 0; i < old_nr; i++) {
				uint8_t v = 0;
				struct cds_ft_inode_flag *iter =
					ft_pc32_node_get_ith_pos(old_pc, i, &v);

				if (insert_new && !new_inserted
				    && (unsigned int) v > (unsigned int) n) {
					ret = _ft_node_set_nth(ft, new_type,
						new_node, new_node_flag,
						new_metadata, n, child_node_flag);
					assert(!ret);
					new_inserted = true;
				}
				if (mode == FT_RECOMPACT_DEL
				    && *nullify_node_flag_ptr == iter)
					continue;
				ret = _ft_node_set_nth(ft, new_type, new_node,
					new_node_flag, new_metadata, v, iter);
				assert(!ret);
			}
		}
		if (insert_new && !new_inserted) {
			ret = _ft_node_set_nth(ft, new_type, new_node,
				new_node_flag, new_metadata, n,
				child_node_flag);
			assert(!ret);
			new_inserted = true;
		}
		(void) new_inserted;
		break;
	}
	case FT_QP:
	{
		struct cds_ft_qp16_node *old_hi =
			(struct cds_ft_qp16_node *) old_node;
		uint16_t old_hi_bm = uatomic_load(&old_hi->bitmap, CMM_RELAXED);
		bool insert_new = (mode == FT_RECOMPACT_ADD_NEXT
				|| mode == FT_RECOMPACT_ADD_SAME);
		unsigned int new_hi_n = (unsigned int) (n >> 4);
		unsigned int new_lo_n = (unsigned int) (n & 0xFU);
		bool new_inserted = false;
		unsigned int hi_iter;
		/*
		 * Route HI-side ptrs reads through ft_qp16_ptrs so the skip
		 * variant (ptrs at byte 16) is handled correctly.  LO nodes
		 * are always direct (LO is reached only via HI byte-step and
		 * is never a SKIP target), so lo->ptrs[] stays raw.
		 */
		struct cds_ft_inode_flag **old_hi_ptrs =
			ft_qp16_ptrs(old_hi, is_skip);

		/*
		 * Walk the old (hi, lo) lattice in popcount order, building
		 * the new node by inserting each live (byte, child) pair via
		 * _ft_node_set_nth.  For ADD modes, weave the new (n,
		 * child_node_flag) into the walk at its lattice position so
		 * the destination's bitmap is filled in strictly increasing
		 * nibble order — set_nth_safe requires the inserted bit to
		 * be the highest set bit ("safe-append"), which the post-
		 * copy add at the framework level violates when @n is below
		 * any of the old node's nibbles.
		 *
		 * The framework's post-copy add (line 8056) re-inserts the
		 * same (n, child) by routing through ft_qp_byte_set Path 2
		 * (lo-bucket now exists, set_nth_safe bit-already-set
		 * branch) and observes existing == child_node_flag, so the
		 * redundant call lands in the live-replace path and is a
		 * pure no-op for accounting.
		 */
		for (hi_iter = 0; hi_iter < 16U; hi_iter++) {
			uint16_t hi_bit = (uint16_t) (1U << hi_iter);
			bool old_has_bucket = (old_hi_bm & hi_bit);
			bool new_in_bucket = (insert_new
					&& new_hi_n == hi_iter
					&& !new_inserted);
			unsigned int hi_idx;
			struct cds_ft_inode_flag *lo_flag = NULL;
			struct cds_ft_qp16_node *lo = NULL;
			uint16_t lo_bm = 0;
			unsigned int j;

			if (!old_has_bucket && !new_in_bucket)
				continue;
			if (old_has_bucket) {
				hi_idx = (unsigned int) __builtin_popcount(
						(unsigned int) (old_hi_bm & (hi_bit - 1U)));
				lo_flag = ft_dereference_acquire(old_hi_ptrs[hi_idx]);
				if (lo_flag) {
					lo = (struct cds_ft_qp16_node *) lo_flag;
					lo_bm = uatomic_load(&lo->bitmap,
							CMM_RELAXED);
				}
			}

			for (j = 0; j < 16U; j++) {
				unsigned int lo_idx;
				struct cds_ft_inode_flag *iter;
				uint8_t v;
				bool old_has = (lo != NULL)
					&& (lo_bm & (1U << j));
				bool new_at_pos = new_in_bucket
					&& new_lo_n == j
					&& !new_inserted;

				if (!old_has && !new_at_pos)
					continue;
				if (new_at_pos) {
					iter = child_node_flag;
					new_inserted = true;
				} else {
					lo_idx = (unsigned int) __builtin_popcount(
							(unsigned int) (lo_bm
								& ((1U << j) - 1U)));
					iter = ft_dereference_acquire(lo->ptrs[lo_idx]);
					if (!iter)
						continue;
					if (mode == FT_RECOMPACT_DEL
							&& *nullify_node_flag_ptr == iter)
						continue;
				}
				v = (uint8_t) ((hi_iter << 4) | j);
				ret = _ft_node_set_nth(ft, new_type, new_node,
						new_node_flag, new_metadata,
						v, iter);
				assert(!ret);
			}
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
		ret = _ft_node_set_nth(ft, new_type, new_node, new_node_flag,
				new_metadata, n, child_node_flag);
		assert(!ret);
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
		/*
		 * For ADD/DEL modes, the new node REPLACES the old at the
		 * same tree position, so the upstream SKIP_X (if any) must
		 * be repointed at the new node.  For REPARENT, the new node
		 * lives at a DIFFERENT position (caller installs it
		 * elsewhere); leave the upstream SKIP_X pointing to the old
		 * node so concurrent readers holding stale SKIP_X (loaded
		 * before the caller's later publish) keep resolving via the
		 * intact old node + its untouched COMPRESSED parent.
		 */
		if (mode != FT_RECOMPACT_REPARENT &&
		    old_parent && ft_node_compressed_in_node(old_parent)) {
			struct cds_ft_compressed_node *cn =
				ft_compressed_node_ptr(old_parent);
			struct cds_ft_metadata *cn_meta =
				cds_ft_item_to_metadata(
					(struct cds_ft_inode *) cn);
			struct cds_ft_inode_flag **skip_slot =
				ft_get_skip_slot(cn_meta, ft);

			if (skip_slot &&
			    ft_node_skip_compressed_in_slot(*skip_slot))
				rcu_assign_pointer(*skip_slot,
					ft_skip_compressed_flag(
						new_node_flag, cn));
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
		case FT_QP:
		{
			struct cds_ft_qp16_node *new_hi =
				(struct cds_ft_qp16_node *) new_node;
			uint16_t new_hi_bm =
				uatomic_load(&new_hi->bitmap, CMM_RELAXED);
			unsigned int hi_iter;
			/*
			 * Route new_hi ptrs reads through ft_qp16_ptrs.  is_skip
			 * propagates from the source variant — recompact does
			 * not change variant (the migration path is the only
			 * one that does).  For skip-variant nodes, LO children
			 * need their meta->parent set to the SKIP_QP-tagged hi
			 * flag (so up-walks recover the variant from the slot
			 * tag); the direct path stores new_node_flag verbatim.
			 */
#ifdef FEATURE_FT_SKIP_COMPRESSED
			bool new_hi_is_skip = new_metadata->is_skip;
#else
			bool new_hi_is_skip = false;
#endif
			struct cds_ft_inode_flag **new_hi_ptrs =
				ft_qp16_ptrs(new_hi, new_hi_is_skip);
			struct cds_ft_inode_flag *lo_parent_flag =
				new_hi_is_skip
				? (struct cds_ft_inode_flag *)
				  ((unsigned long) new_node | FT_KIND_SKIP_QP)
				: new_node_flag;

			/*
			 * Walk new node's (hi, lo) lattice and reparent each
			 * live child under the new node's published flag.
			 * Same shape as the FT_PIGEON arm but indexed via
			 * the popcount-bitmap walk.
			 */
			for (hi_iter = 0; hi_iter < 16U; hi_iter++) {
				uint16_t hi_bit = (uint16_t) (1U << hi_iter);
				unsigned int hi_idx;
				struct cds_ft_inode_flag *lo_flag;
				struct cds_ft_qp16_node *lo;
				uint16_t lo_bm;
				unsigned int j;

				if (!(new_hi_bm & hi_bit))
					continue;
				hi_idx = (unsigned int) __builtin_popcount(
						(unsigned int) (new_hi_bm
							& (hi_bit - 1U)));
				lo_flag = new_hi_ptrs[hi_idx];
				if (!lo_flag)
					continue;
				lo = (struct cds_ft_qp16_node *) lo_flag;
				lo_bm = uatomic_load(&lo->bitmap, CMM_RELAXED);

				/*
				 * Lo-nodes are reused from the old hi (recompact
				 * only swaps the hi).  Repoint each surviving
				 * lo's parent at the freshly published new hi
				 * before the old hi is RCU-freed.
				 */
				{
					struct cds_ft_metadata *lo_meta =
						cds_ft_item_to_metadata(lo);

					rcu_assign_pointer(lo_meta->parent,
						lo_parent_flag);
				}

				for (j = 0; j < 16U; j++) {
					unsigned int lo_idx;
					struct cds_ft_inode_flag *iter;

					if (!(lo_bm & (1U << j)))
						continue;
					lo_idx = (unsigned int) __builtin_popcount(
							(unsigned int) (lo_bm
								& ((1U << j) - 1U)));
					iter = lo->ptrs[lo_idx];
					if (!iter)
						continue;
					/*
					 * Direct child of lo: parent = tagged
					 * lo-flag (not new_node_flag).  Slot
					 * is in the lo-arena allocation.  The
					 * local @lo_flag is RAW (read from
					 * hi.ptrs[]); build the tagged form
					 * for meta->parent.
					 */
					ft_set_parent(iter,
						ft_qp16_lo_flag(lo),
						&lo->ptrs[lo_idx]);
				}
			}
			break;
		}
		case FT_POPCOUNT:
		{
			unsigned int new_nr;
			unsigned int i;

			/*
			 * Walk new POPCOUNT children in popcount-rank
			 * order and reparent each under the freshly
			 * published new_node_flag, mirroring the FT_PIGEON
			 * arm.  Children copied from the old node still
			 * reference the old node via meta->parent until
			 * this loop runs.  pc32 (order 5) vs pc64 (order
			 * 6) layouts share FT_POPCOUNT type_class; dispatch
			 * by new_type->order.
			 */
			if (new_type->order == FT_PC64_ALLOC_ORDER) {
				struct ft_pc64_node *new_pc =
					(struct ft_pc64_node *) new_node;

				new_nr = ft_pc64_node_get_nr_child(new_pc);
				for (i = 0; i < new_nr; i++) {
					uint8_t v;
					struct cds_ft_inode_flag *iter;
					struct cds_ft_inode_flag **slot = NULL;

					iter = ft_pc64_node_get_ith_pos(
							new_pc, i, &v);
					if (!iter)
						continue;
					ft_node_get_nth_skip(new_node_flag,
							&slot, v, FT_PF_NONE);
					ft_set_parent(iter, new_node_flag, slot);
				}
			} else {
				struct ft_pc32_node *new_pc =
					(struct ft_pc32_node *) new_node;

				new_nr = ft_pc32_node_get_nr_child(new_pc);
				for (i = 0; i < new_nr; i++) {
					uint8_t v;
					struct cds_ft_inode_flag *iter;
					struct cds_ft_inode_flag **slot = NULL;

					iter = ft_pc32_node_get_ith_pos(
							new_pc, i, &v);
					if (!iter)
						continue;
					ft_node_get_nth_skip(new_node_flag,
							&slot, v, FT_PF_NONE);
					ft_set_parent(iter, new_node_flag, slot);
				}
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
 * ft_node_clone_for_reparent: allocate a fresh sibling of @old_node_flag
 * (same type, all children copied, child meta->parent updated to the
 * clone).  The original node is intact and the caller is responsible
 * for RCU-freeing it via *old_node_ret.  The clone's own meta->parent
 * inherits from the original; the caller must overwrite via
 * ft_set_parent / ft_node_set_nth when installing the clone.
 *
 * Used to safely move a child across the COMPRESSED↔non-COMPRESSED
 * parent-kind boundary in compressed-split paths.  In-place demote
 * (rcu_assign_pointer of meta->parent from a COMPRESSED to a non-
 * COMPRESSED) would race with concurrent readers holding stale
 * SKIP_X(child) pointers from the upstream slot, since the readers
 * would observe the stale skip pointer paired with the freshly-demoted
 * non-COMPRESSED parent and trip ft_skip_to_compressed's COMPRESSED
 * assertion.  Cloning preserves the OLD (child, parent=COMPRESSED)
 * relationship for those readers until a grace period drains them, at
 * which point the original is freed.
 *
 * Returns NULL on -ENOMEM.  Only meaningful for internal node kinds
 * (FT_KIND_QP, FT_KIND_PIGEON); EXT children are user-allocated and
 * cannot be cloned.
 */
static
struct cds_ft_inode_flag *ft_node_clone_for_reparent(struct cds_ft *ft,
		struct cds_ft_inode_flag *old_node_flag,
		struct cds_ft_inode **old_node_ret)
{
	struct cds_ft_inode *old_node = ft_node_ptr(old_node_flag);
	struct cds_ft_metadata *old_meta = cds_ft_item_to_metadata(old_node);
	unsigned int type_index = ft_node_type_index(old_node_flag);
	const struct cds_ft_type *type = &ft_types[type_index];
	struct cds_ft_inode_flag *new_node_flag = old_node_flag;
	int ret;

	ret = ft_node_recompact(FT_RECOMPACT_REPARENT, ft, type_index, type,
			old_node, old_meta, &new_node_flag,
			0, NULL, NULL, old_node_ret, false);
	if (ret)
		return NULL;
	return new_node_flag;
}

/*
 * QP→PIGEON conversion.  Used by the post-success up-trigger in
 * ft_node_set_nth when an FT_QP hi-node's qp_subtree_half_cls exceeds
 * PIGEON's flat 64 half-CLs.
 *
 * Allocates a fresh PIGEON, walks the (hi, lo, byte) lattice into it,
 * inherits hi-meta state, swaps the parent's slot, reparents children,
 * and queues the old hi + 16 lo's for RCU-free.  Caller passes
 * @old_node_ret so the framework's existing post-recompact free dance
 * applies to the old hi.
 *
 * Returns 0 on success.  -ENOMEM on PIGEON alloc failure (the QP stays
 * intact; the caller may try again later).
 */
static
int ft_qp_to_pigeon_convert(struct cds_ft *ft,
		struct cds_ft_inode_flag **old_node_flag_ptr,
		struct cds_ft_metadata *old_meta,
		struct cds_ft_inode **old_node_ret)
{
	const struct cds_ft_type *pigeon_type =
			&ft_types[FT_NUM_INTERNAL_TYPES - 1];
	struct cds_ft_inode_flag *old_hi_flag = *old_node_flag_ptr;
	struct cds_ft_qp16_node *old_hi =
			(struct cds_ft_qp16_node *) ft_node_ptr(old_hi_flag);
	struct cds_ft_inode *new_pigeon;
	struct cds_ft_metadata *new_meta;
	struct cds_ft_inode_flag *new_pigeon_flag;
	uint16_t old_hi_bm;
	unsigned int hi_iter;
#ifdef FEATURE_FT_SKIP_COMPRESSED
	/*
	 * QP→PIGEON escalation may fire on a skip-variant qp HI (the
	 * migrated cn->child under a SKIP_QP-wrapped chain).  ptrs[] is at
	 * byte 16 in the skip layout, not at the struct-member offset of 8.
	 * Read is_skip from old_meta and use ft_qp16_ptrs(old_hi, is_skip)
	 * for every HI ptr access below.  Write-side (mutex held); the
	 * cold metadata load is acceptable here.
	 */
	bool old_hi_is_skip = old_meta->is_skip;
#else
	bool old_hi_is_skip = false;
#endif
	struct cds_ft_inode_flag **old_hi_ptrs =
			ft_qp16_ptrs(old_hi, old_hi_is_skip);

	assert(pigeon_type->type_class == FT_PIGEON);

	new_pigeon = alloc_cds_ft_node(ft, pigeon_type, &new_meta);
	if (!new_pigeon)
		return -ENOMEM;
	new_pigeon_flag = ft_node_flag(new_pigeon,
			FT_NUM_INTERNAL_TYPES - 1);

	new_meta->parent = old_meta->parent;
#ifdef FEATURE_FT_SKIP_COMPRESSED
	new_meta->skip_slot_offset = old_meta->skip_slot_offset;
	new_meta->pigeon_skip_len = 0;	/* set below if old hi was a skip target */
#endif
	new_meta->fallback_removal_count = old_meta->fallback_removal_count;
	ft_metadata_set_external_nodes(new_pigeon_flag, new_meta,
			old_meta->external_nodes);
	ft_nr_keys_store(ft, new_meta, ft_nr_keys_get(old_meta), CMM_RELAXED);
	new_meta->nr_child = old_meta->nr_child;

	/* Walk old (hi, lo, byte) lattice, populate PIGEON. */
	old_hi_bm = uatomic_load(&old_hi->bitmap, CMM_RELAXED);
	for (hi_iter = 0; hi_iter < 16U; hi_iter++) {
		uint16_t hi_bit = (uint16_t) (1U << hi_iter);
		unsigned int hi_idx;
		struct cds_ft_qp16_node *lo;
		uint16_t lo_bm;
		unsigned int j;

		if (!(old_hi_bm & hi_bit))
			continue;
		hi_idx = (unsigned int) __builtin_popcount(
				(unsigned int) (old_hi_bm & (hi_bit - 1U)));
		lo = (struct cds_ft_qp16_node *) ft_dereference_acquire(
				old_hi_ptrs[hi_idx]);
		if (!lo)
			continue;
		lo_bm = uatomic_load(&lo->bitmap, CMM_RELAXED);
		for (j = 0; j < 16U; j++) {
			uint16_t lo_bit = (uint16_t) (1U << j);
			unsigned int lo_idx;
			struct cds_ft_inode_flag *iter;
			uint8_t v;

			if (!(lo_bm & lo_bit))
				continue;
			lo_idx = (unsigned int) __builtin_popcount(
					(unsigned int) (lo_bm & (lo_bit - 1U)));
			iter = ft_dereference_acquire(lo->ptrs[lo_idx]);
			if (!iter)
				continue;
			v = (uint8_t) ((hi_iter << 4) | j);
			/*
			 * ft_pigeon_node_set_nth bumps new_meta->nr_child
			 * for every slot it sets.  We pre-set nr_child from
			 * old_meta above, so undo each bump as we go to keep
			 * the final count consistent.
			 */
			(void) ft_pigeon_node_set_nth(pigeon_type, new_pigeon,
					new_meta, v, iter);
			new_meta->nr_child--;
		}
	}

	/*
	 * If old hi was the target of a skip-compressed pointer in its
	 * parent compressed node, repoint that skip slot at new PIGEON
	 * BEFORE swapping the regular slot (mirrors the recompact
	 * framework's ordering).
	 */
#ifdef FEATURE_FT_SKIP_COMPRESSED
	if (old_meta->parent && ft_node_compressed_in_node(old_meta->parent)) {
		struct cds_ft_compressed_node *cn =
				ft_compressed_node_ptr(old_meta->parent);
		struct cds_ft_metadata *cn_meta =
				cds_ft_item_to_metadata(
					(struct cds_ft_inode *) cn);
		struct cds_ft_inode_flag **skip_slot =
				ft_get_skip_slot(cn_meta, ft);

		if (skip_slot && ft_node_skip_compressed_in_slot(*skip_slot))
			rcu_assign_pointer(*skip_slot,
				ft_skip_compressed_flag(new_pigeon_flag, cn));
	}
#endif

	/* Swap the regular parent slot to point at new PIGEON. */
	rcu_assign_pointer(*old_node_flag_ptr, new_pigeon_flag);

	/* Reparent children: each child's metadata->parent → new PIGEON. */
	for (hi_iter = 0; hi_iter < FT_ENTRY_PER_NODE; hi_iter++) {
		struct cds_ft_inode_flag *iter;
		struct cds_ft_inode_flag **slot = NULL;

		iter = ft_pigeon_node_get_ith_pos(pigeon_type, new_pigeon,
				hi_iter);
		if (!iter)
			continue;
		ft_node_get_nth_skip(new_pigeon_flag, &slot,
				(uint8_t) hi_iter, FT_PF_NONE);
		ft_set_parent(iter, new_pigeon_flag, slot);
	}

	/* RCU-free old lo's. */
	for (hi_iter = 0; hi_iter < 16U; hi_iter++) {
		uint16_t hi_bit = (uint16_t) (1U << hi_iter);
		unsigned int hi_idx;
		struct cds_ft_qp16_node *old_lo;

		if (!(old_hi_bm & hi_bit))
			continue;
		hi_idx = (unsigned int) __builtin_popcount(
				(unsigned int) (old_hi_bm & (hi_bit - 1U)));
		old_lo = (struct cds_ft_qp16_node *) ft_dereference_acquire(
				old_hi_ptrs[hi_idx]);
		if (old_lo)
			ft_qp16_node_free_rcu(ft, old_lo);
	}

	FT_TP(node_recompact, (const void *) old_hi_flag,
		(const void *) new_pigeon_flag,
		(int) (FT_NUM_INTERNAL_TYPES - 1));

	*old_node_flag_ptr = new_pigeon_flag;
	if (old_node_ret)
		*old_node_ret = (struct cds_ft_inode *) old_hi;
	return 0;
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
		unsigned int node_depth __attribute__((unused)))
{
	int ret;
	unsigned int type_index;
	const struct cds_ft_type *type;
	struct cds_ft_inode *node;

	dbg_printf("ft_node_set_nth for n=%u, node %p\n", (unsigned int) n, ft_node_ptr(*node_flag));

	node = ft_node_ptr(*node_flag);
	type_index = ft_node_type_index(*node_flag);
	type = &ft_types[type_index];
	ret = _ft_node_set_nth(ft, type, node, *node_flag, metadata, n, child_node_flag);
	switch (ret) {
	case 0:
	{
		/*
		 * In-place insert succeeded on the published target node.
		 * Safe to link child -> target via parent pointer now:
		 * target is already fully valid to readers.
		 *
		 * The FT_QP arm of _ft_node_set_nth (ft_qp_byte_set) already
		 * records the child's parent as the tagged lo-flag —
		 * overwriting it here with @node_flag (the hi-flag) would
		 * clobber the (hi, lo) parent semantics, so skip the
		 * secondary ft_set_parent for QP nodes.
		 */
		if (type->type_class != FT_QP) {
			struct cds_ft_inode_flag **slot_ptr = NULL;

			/*
			 * Compute slot_ptr for any compressed-form child
			 * (SKIP_X or plain COMPRESSED) so ft_set_parent →
			 * ft_set_skip_slot can record the slot offset.
			 * Without this, plain-COMPRESSED children installed
			 * under non-QP parents leave skip_slot_offset == 0
			 * and break later chain-merge canonicalization in
			 * ft_detach_node, which recovers the slot via
			 * ft_get_skip_slot.
			 */
			if (ft_node_skip_compressed_in_slot(child_node_flag)
			    || ft_node_compressed_in_node(child_node_flag))
				ft_node_get_nth_skip(*node_flag, &slot_ptr, n, FT_PF_NONE);
			ft_set_parent(child_node_flag, *node_flag, slot_ptr);
		} else if (caa_unlikely(metadata->qp_subtree_half_cls >
				FT_PIGEON_HALF_CLS)) {
			/*
			 * QP→PIGEON up-trigger: hi+lo subtree footprint just
			 * crossed PIGEON's flat 2 KB.  Convert in place.  The
			 * trigger only fires post-success — never inside the
			 * recompact framework's lattice walk — so the
			 * conversion's own lattice walk runs uninterrupted.
			 */
			ret = ft_qp_to_pigeon_convert(ft, node_flag, metadata,
					old_node_ret);
		}
		break;
	}
	case -ENOSPC:
		/* Not enough space in node, need to recompact to next type. */
		ret = ft_node_recompact(FT_RECOMPACT_ADD_NEXT, ft, type_index, type, node,
					metadata, node_flag, n, child_node_flag, NULL,
					old_node_ret, false);
		break;
	case -ERANGE:
		/* Node needs to be recompacted. */
		ret = ft_node_recompact(FT_RECOMPACT_ADD_SAME, ft, type_index, type, node,
					metadata, node_flag, n, child_node_flag, NULL,
					old_node_ret, false);
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
		unsigned int node_depth __attribute__((unused)))
{
	int ret;
	unsigned int type_index;
	const struct cds_ft_type *type;
	struct cds_ft_inode *node;

	dbg_printf("ft_node_replace_ptr for node %p, target ptr %p\n",
		ft_node_ptr(*parent_node_flag_ptr), node_flag_ptr);

	node = ft_node_ptr(*parent_node_flag_ptr);
	type_index = ft_node_type_index(*parent_node_flag_ptr);
	type = &ft_types[type_index];
	ret = _ft_node_replace_ptr(ft, type, node, *parent_node_flag_ptr, metadata, node_flag_ptr, n, newptr);
	if (ret == -EFBIG) {
		assert(!newptr);
		/* Should try recompaction. */
		ret = ft_node_recompact(FT_RECOMPACT_DEL, ft, type_index, type, node,
				metadata, parent_node_flag_ptr, n, NULL,
				node_flag_ptr, old_node_ret, is_root);
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
 *
 * In candidate mode (@candidate=true), the compressed path is not
 * compared during descent and *needs_leaf_validate (when non-NULL) is
 * set so the caller knows to validate against the leaf bytes at
 * end-of-descent.
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
		bool candidate,
		bool *needs_leaf_validate,
		size_t *first_skip_offset_p)
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
	 * Non-candidate mode: compare cn->key_bytes against the lookup
	 * key as we descend.  Candidate mode skips the compare and flags
	 * needs_leaf_validate so the end-of-descent leaf-bytes compare
	 * catches any mis-descent.
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
	} else if (needs_leaf_validate) {
		*needs_leaf_validate = true;
		/*
		 * Anchor first_skip_offset at the depth of this cn-handler
		 * step.  In spec_validate mode this path runs with the
		 * compare suppressed, so cn->key_bytes are trusted-but-
		 * unvalidated — same status as SKIP_* bytes.
		 */
		if (first_skip_offset_p) {
			size_t cur = *i_p;
			if (*first_skip_offset_p > cur)
				*first_skip_offset_p = cur;
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
	if (i < key_depth - 1 && ft_node_external_direct(node_flag)) {
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
	if (track && i < key_depth - 1 && !ft_node_external_direct(node_flag)) {
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
	if (ft_node_external_direct(cn->child))
		return FT_DESCENT_BREAK;
	return FT_DESCENT_CONTINUE;
}

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
	size_t match_len = track_longest ? FT_MATCH_LEN_NONE : 0;
	struct cds_ft_node *match_node = NULL;
	/*
	 * Speculative-validate state: when set, at least one cand-mode
	 * descent step did not validate inline (skip_len > 5, PIGEON or
	 * EXT skip target, or a cn handler in cand mode).  A single
	 * end-of-descent leaf-bytes compare covers all those steps.
	 * QP skips with skip_len ≤ 5 validate immediately against the
	 * inline subkey on the QP node header CL — if every cand-mode
	 * step takes the inline path, the leaf load is avoided.
	 */
	bool needs_leaf_validate = false;
	/*
	 * first_skip_offset: byte position of the first deferred-validate
	 * skip during descent.  Bytes before this offset were validated
	 * implicitly by the descent (QP/PIGEON byte-steps consume one
	 * byte each; FT_KIND_COMPRESSED steps validate cn->len bytes
	 * inline; SKIP_QP with skip_len ≤ 5 validates against the inline
	 * subkey).  Bytes at and after this offset may include trusted-
	 * but-unvalidated bytes from one or more deferred skip steps.
	 *
	 * Sentinel: key_len means "no deferred skip yet" — the end-of-
	 * descent leaf-bytes compare runs over [first_skip_offset, key_len)
	 * which collapses to an empty range when needs_leaf_validate is
	 * also false (no deferred skip happened).  Guarded with key_len > 0
	 * downstream.
	 */
	size_t first_skip_offset;

	CDS_FT_ASSERT_RCU_READ_LOCKED(ft);

	if (!valid_key_len(ft, key_len)) {
		status = CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
		goto end;
	}
	key_depth = key_len + 1;
	first_skip_offset = key_len;	/* sentinel: no deferred skip yet */
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
		struct cds_ft_metadata *metadata = ft_flag_to_metadata_fast(node_flag);
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
		struct cds_ft_metadata *metadata = ft_flag_to_metadata_fast(node_flag);
		struct cds_ft_node *external_nodes = ft_dereference_prefetch_external(metadata->external_nodes);

		if (external_nodes || track_longest) {
			match_len = 0;
			match_node = external_nodes;
		}
	}

	/*
	 * Per-iter post-step bookkeeping shared by every byte-step fast
	 * path (QP_HI, PIGEON, legacy ft_node_get_nth_skip tail).
	 *
	 * - Update iter_path[i] with the post-step node_flag (sparse:
	 *   skip-elided depths are not filled).
	 * - Mid-descent EXT detection: input key extends past a leaf →
	 *   NOT_FOUND.  Goto end with status set; track callers also
	 *   record the partial match.
	 * - track-mode prefix tracking on internal hops: compiles out
	 *   for track=false because track is a constant-folded literal
	 *   at every do_cds_ft_lookup callsite.
	 *
	 * `goto end` from inside the macro keeps the early-exit flow
	 * inline at each call site without requiring a returned status
	 * flag.  Undef after the for-loop to keep the name local.
	 */
#define FT_BYTE_STEP_POST() do { \
	if (iter) { \
		iter_path_node(iter)[i] = node_flag; \
		iter_path_len = i + 1; \
	} \
	if (caa_unlikely(ft_node_external_direct(node_flag)) \
	    && i < key_depth - 1) { \
		if (track) { \
			match_len = i; \
			match_node = (struct cds_ft_node *) node_flag; \
		} \
		status = CDS_FT_STATUS_NOT_FOUND; \
		goto end; \
	} \
	/*  \
	 * Filter SKIP-X tags before consulting ft_node_internal: the \
	 * QP byte-step above can return SKIP-tagged children (per the \
	 * "Skip-compressed children produced by ft_qp_byte_get stay \
	 * tagged here" contract), and ft_node_internal asserts a \
	 * post-resolve input.  In track mode the SKIP-X step represents \
	 * a multi-level compressed cluster traversal; recording its \
	 * external_nodes via ft_flag_to_metadata_fast would also be \
	 * incorrect because that fast variant strips the tag and indexes \
	 * the underlying child, which for SKIP_EXT is an external leaf \
	 * (not in our arena).  Skip the track update for SKIP-X iters; \
	 * the subsequent SKIP handler resolves the value and the next \
	 * iter records the resolved node. \
	 */ \
	if (track && !ft_node_skip_compressed_in_slot(node_flag) \
	    && caa_likely(ft_node_internal(node_flag)) \
	    && i < key_depth - 1) { \
		struct cds_ft_metadata *metadata = \
			ft_flag_to_metadata_fast(node_flag); \
		struct cds_ft_node *external_nodes = \
			ft_dereference_prefetch_external(metadata->external_nodes); \
		if (external_nodes || track_longest) { \
			match_len = i; \
			match_node = external_nodes; \
		} \
	} \
} while (0)

	for (i = 1; i < key_depth; i++) {
		uint8_t iter_key;

		/*
		 * QP_HI fast path (1/5 in the flat dispatch sequence): the
		 * dominant byte-step in spec_validate descent and a major
		 * case in candidate / track / exact-lookup descent too.
		 * Constant-tag SUB strips FT_KIND_QP with no dispatch;
		 * ft_qp_byte_get inlines the bitmap+ptr load.
		 *
		 * Absorbs the per-iter post-step bookkeeping (iter-path,
		 * mid-descent EXT detection, track prefix tracking) so all
		 * modes share this path.  Skip-compressed children produced
		 * by ft_qp_byte_get stay tagged here; the next iter's SKIP
		 * handler resolves them (mode-aware).
		 */
		if (caa_likely(((unsigned long) node_flag & FT_KIND_MASK)
				== FT_KIND_QP)) {
			struct cds_ft_qp16_node *qp =
				(struct cds_ft_qp16_node *)
				((unsigned long) node_flag - FT_KIND_QP);

			iter_key = *(key++);
			node_flag = ft_qp_byte_get(qp, NULL, iter_key,
					FT_PF_DATA, false);
			if (caa_unlikely(!node_flag)) {
				status = CDS_FT_STATUS_NOT_FOUND;
				goto end;
			}
			FT_BYTE_STEP_POST();
			continue;
		}
		/*
		 * SKIP handler (2/5 in the flat dispatch sequence): after
		 * QP_HI miss, a skip-compressed pointer is the next most
		 * likely case (skip-tagged children produced by the
		 * previous iter's QP_HI byte-step or by collapsed entries).
		 * No caa_unlikely: at this point in the dispatch chain the
		 * skip case is the second-most-frequent.
		 *
		 * Non-candidate: convert to compressed flag so the
		 * compressed handler below processes it with full key
		 * comparison.
		 *
		 * Candidate: resolve the skip (advance past the compressed
		 * path without comparison).
		 */
		if (ft_node_skip_compressed_in_slot(node_flag)) {
#ifdef FEATURE_FT_SKIP_COMPRESSED
			/*
			 * Candidate E: bit 0 partitions by alignment.  Internal-
			 * aligned skips (SKIP_PIGEON 0x03, SKIP_QP 0x07,
			 * SKIP_POPCOUNT_32 0x0B, SKIP_POPCOUNT_64 0x13) are
			 * 32-byte aligned with clean 5-bit kind; SKIP_EXT
			 * (0x02) is 16-byte aligned and bit 4 of the leaf
			 * address may leak under a 5-bit mask, but bit 0 is
			 * always clear for SKIP_EXT so the conditional folds
			 * the leak into a fixed FT_KIND_SKIP_EXT value.
			 */
			unsigned long skip_kind =
				((unsigned long) node_flag & 0x01UL)
				? ((unsigned long) node_flag & 0x1FUL)
				: FT_KIND_SKIP_EXT;
#endif
			if (!descend_cand) {
#ifdef FEATURE_FT_SKIP_COMPRESSED
				/*
				 * Non-cand mode + SKIP_EXT: SKIP_EXT only
				 * exists in spec-validate groups (Phase 1
				 * gate in ft_publish_compressed and
				 * ft_publish_to_parent), so
				 * speculative_key_offset is always available
				 * here.  Recover the implicit cn_len from
				 * the leaf's stored key length and run the
				 * cn-prefix compare against the leaf's
				 * stored bytes — never dereference
				 * ext->prev (mutator-only after this
				 * redesign).
				 *
				 * Gated on key_map.identity: the leaf's
				 * stored bytes are in user space, while
				 * the trie's path bytes (and the input
				 * key) are in ordinal space.  When
				 * identity, the two coincide and the
				 * compare is a direct memcmp.  Non-identity
				 * spec groups in track mode are rare; they
				 * fall through to ft_skip_to_compressed
				 * (the still-racy path) until a future
				 * follow-up reverse-maps before compare.
				 *
				 * Track-mode bookkeeping is inline:
				 * full match terminates at the leaf at
				 * depth leaf_key_len; partial match
				 * (input longer than leaf) records the
				 * leaf as the longest-match candidate
				 * for track_longest before NOT_FOUND.
				 */
				if (skip_kind == FT_KIND_SKIP_EXT
				    && ft->group->key_map.identity) {
					struct cds_ft_node *leaf =
						(struct cds_ft_node *)
						((unsigned long) node_flag - FT_KIND_SKIP_EXT);
					const uint8_t *leaf_key =
						(const uint8_t *) leaf +
						ft->group->speculative_key_offset;
					size_t leaf_key_len;
					size_t input_remaining = key_len - i;
					size_t cn_len_implied;

					assert(ft->group->speculative_validated);
					if (ft->group->speculative_key_len_offset !=
							CDS_FT_SPECULATIVE_OFFSET_NONE) {
						leaf_key_len = *(const size_t *)
							((const uint8_t *) leaf +
							 ft->group->speculative_key_len_offset);
					} else {
						/*
						 * Fixed-length group: leaf
						 * stores the full fixed_key_len.
						 * For non-cand descent, key_len
						 * == fixed_key_len (length is
						 * group invariant), so leaf
						 * sits at exactly key_len.
						 */
						leaf_key_len = key_len;
					}
					assert(leaf_key_len > i);
					cn_len_implied = leaf_key_len - i;
					if (cn_len_implied > input_remaining) {
						/* Leaf longer than remaining input. */
						status = CDS_FT_STATUS_NOT_FOUND;
						goto end;
					}
					if (memcmp(key, leaf_key + i, cn_len_implied) != 0) {
						status = CDS_FT_STATUS_NOT_FOUND;
						goto end;
					}
					if (cn_len_implied < input_remaining) {
						/*
						 * Input longer than leaf — partial
						 * match.  For track_longest, leaf
						 * at depth leaf_key_len is the
						 * longest matching prefix.
						 */
						if (track) {
							match_len = leaf_key_len;
							match_node = leaf;
						}
						status = CDS_FT_STATUS_NOT_FOUND;
						goto end;
					}
					/*
					 * cn_len_implied == input_remaining:
					 * exact match at leaf depth.  Advance
					 * i/key by cn_len_implied and let the
					 * post-loop terminal handler (line
					 * ~5888) record the leaf and set OK.
					 */
					key += cn_len_implied;
					i += cn_len_implied - 1;
					node_flag = (struct cds_ft_inode_flag *) leaf;
					if (iter) {
						iter_path_node(iter)[i + 1] = node_flag;
						iter_path_len = i + 2;
					}
					/*
					 * Next iter top: EXT direct test
					 * (line ~5852) breaks to the post-
					 * loop terminal handler.
					 */
					continue;
				}
				/*
				 * Non-cand SKIP_QP inline arm.  Non-cand mode
				 * has no end-of-descent leaf compare, so the
				 * skipped compressed prefix must be validated
				 * here.  Split the compare:
				 *
				 *   - First min(skip, 13) bytes against the
				 *     qp's cached subkey (hot, on the qp body
				 *     CL the next byte-step will touch too).
				 *   - Tail bytes (if skip > 13) against
				 *     cn->key_bytes via ft_skip_to_compressed —
				 *     cold cn CL, but only fired when the
				 *     prefix exceeds the inline reach.
				 *
				 * cand mode (the spec_validate arm below) does
				 * not need this split: skip > 13 sets
				 * needs_leaf_validate + first_skip_offset, and
				 * the end-of-descent leaf-bytes compare covers
				 * the entire unverified tail in one memcmp
				 * against the leaf's stored key.
				 *
				 * Bypassing cn (when skip <= 13) keeps the
				 * inline QP_HI fast path correct with
				 * is_skip = false: descent never feeds an
				 * FT_KIND_QP-tagged skip-variant qp back into
				 * the loop top, because the SKIP_QP arm
				 * byte-steps directly into the qp (with
				 * is_skip = true) and the resulting next-iter
				 * node_flag is the LO's child — a fresh-level
				 * node, not the skip-variant qp.
				 */
				if (caa_likely(skip_kind == FT_KIND_SKIP_QP)) {
					struct cds_ft_qp16_node *qp =
						(struct cds_ft_qp16_node *)
						((unsigned long) node_flag - FT_KIND_SKIP_QP);
					const struct ft_qp16_skip_meta *meta =
						ft_qp16_skip_meta_at(qp);
					unsigned int skip = meta->skip_len;
					int remaining_key = key_depth - 1 - i;
					int cmp_len = (int) skip < remaining_key
						? (int) skip : remaining_key;
					int inline_cmp_len = cmp_len < (int) FT_QP16_SUBKEY_INLINE_LEN
						? cmp_len : (int) FT_QP16_SUBKEY_INLINE_LEN;
					struct cds_ft_compressed_node *cn = NULL;

					/*
					 * Inline cmp: first inline_cmp_len bytes
					 * against the qp's cached subkey.
					 */
					if (track_longest) {
						unsigned int mpos;
						int cmp = ft_key_cmp_ordinals(
								key, meta->subkey,
								inline_cmp_len, remaining_key,
								false, &mpos);

						if (cmp != 0) {
							match_len = i + mpos;
							match_node = NULL;
							status = CDS_FT_STATUS_NOT_FOUND;
							goto end;
						}
					} else {
						if (caa_unlikely(ft_key_cmp_ordinals(
								key, meta->subkey,
								inline_cmp_len, remaining_key,
								false, NULL) != 0)) {
							status = CDS_FT_STATUS_NOT_FOUND;
							goto end;
						}
					}

					/*
					 * Tail cmp: bytes beyond the inline reach
					 * against cn->key_bytes.  Loads cn on the
					 * long-prefix path only (cn->len > 13).
					 */
					if (caa_unlikely(cmp_len > inline_cmp_len)) {
						int tail_len = cmp_len - inline_cmp_len;

						cn = ft_skip_to_compressed(node_flag);
						if (track_longest) {
							unsigned int mpos;
							int cmp = ft_key_cmp_ordinals(
									key + inline_cmp_len,
									cn->key_bytes + inline_cmp_len,
									tail_len,
									remaining_key - inline_cmp_len,
									false, &mpos);

							if (cmp != 0) {
								match_len = i + inline_cmp_len + mpos;
								match_node = NULL;
								status = CDS_FT_STATUS_NOT_FOUND;
								goto end;
							}
						} else {
							if (caa_unlikely(ft_key_cmp_ordinals(
									key + inline_cmp_len,
									cn->key_bytes + inline_cmp_len,
									tail_len,
									remaining_key - inline_cmp_len,
									false, NULL) != 0)) {
								status = CDS_FT_STATUS_NOT_FOUND;
								goto end;
							}
						}
					}

					if (track_longest) {
						match_len = i + cmp_len;
						match_node = NULL;
					}

					if (caa_unlikely((int) skip > remaining_key)) {
						/*
						 * Input terminates inside the
						 * compressed prefix.  Recover cn
						 * via qp->meta->parent (re-use
						 * the cn already loaded for the
						 * tail cmp if applicable) and
						 * read cn's external_nodes for
						 * keys ending at this depth.
						 */
						struct cds_ft_metadata *cn_meta;

						if (!cn)
							cn = ft_skip_to_compressed(node_flag);
						cn_meta = cds_ft_item_to_metadata_fast(
								(struct cds_ft_inode *) cn,
								ft_compressed_order(cn->len));

						found = ft_dereference_prefetch_external(
								cn_meta->external_nodes);
						status = found ? CDS_FT_STATUS_OK
								: CDS_FT_STATUS_NOT_FOUND;
						if (track && (found || track_longest)) {
							match_len = i;
							match_node = found;
						}
						goto end;
					}

					/* Advance past the compressed prefix. */
					key += skip;
					i += skip;

					/*
					 * Consume one more byte via the
					 * skip-variant byte-step (ptrs at
					 * byte 16).  The for-loop's i++
					 * advances past this byte; next iter
					 * dispatches the returned child via
					 * the loop-top fast paths.  Result is
					 * the LO's child slot value — a
					 * fresh-level node, not the
					 * skip-variant qp.
					 */
					iter_key = *(key++);
					node_flag = ft_qp_byte_get(qp, NULL,
							iter_key, FT_PF_DATA,
							true);
					if (caa_unlikely(!node_flag)) {
						status = CDS_FT_STATUS_NOT_FOUND;
						goto end;
					}
					FT_BYTE_STEP_POST();
					continue;
				}
#endif
				node_flag = ft_compressed_node_flag(
					ft_skip_to_compressed(node_flag));
			} else {
#ifdef FEATURE_FT_SKIP_COMPRESSED
				/*
				 * SKIP_EXT fast path: target is a leaf, so
				 * the descent is about to terminate anyway.
				 * Don't recover skip_len (would force an
				 * extra CL load via ext->prev → cn->len —
				 * and ext->prev is mutator-only); don't
				 * advance key/i (dead, the loop breaks on
				 * ft_node_external_direct next iter); for
				 * spec_validate, the end-of-descent leaf-
				 * bytes compare validates the full key,
				 * including any bytes covered by this
				 * skip.  For pure candidate (no
				 * spec_validate), the caller validates
				 * against its stored key.
				 *
				 * Length mismatch (input shorter than leaf
				 * key) is caught by the leaf compare's
				 * length check (spec_validate) or the
				 * caller's stored-key compare (pure
				 * candidate).
				 */
				if (caa_likely(skip_kind == FT_KIND_SKIP_EXT)) {
					if (spec_validate) {
						if (first_skip_offset == key_len)
							first_skip_offset = i;
						needs_leaf_validate = true;
					}
					node_flag = ft_skip_child_ptr(node_flag);
					if (iter) {
						/*
						 * Iter path: leaf sits at
						 * depth key_len for an OK
						 * lookup (spec_validate
						 * enforces equal length;
						 * pure candidate trusts
						 * caller).  If status ends
						 * up NOT_FOUND, path_valid
						 * is cleared at end so this
						 * value is moot.
						 */
						iter_path_node(iter)[key_len] = node_flag;
						iter_path_len = key_len + 1;
					}
					if (node_flag && spec_validate)
						/*
						 * After ft_skip_child_ptr(SKIP_EXT),
						 * node_flag has tag 0 (FT_KIND_EXT)
						 * and the external node is 16B-
						 * aligned, so the low nibble is
						 * already 0 — skip ft_node_ptr's
						 * tag-strip dispatch and cast
						 * directly.
						 */
						__builtin_prefetch(
							(const uint8_t *) node_flag +
							ft->group->speculative_key_offset +
							first_skip_offset);
					break;
				}
#endif
				unsigned int skip = ft_skip_len(node_flag);
				int remaining = key_depth - 1 - i;

				if ((int) skip > remaining) {
					status = CDS_FT_STATUS_NOT_FOUND;
					goto end;
				}
#ifdef FEATURE_FT_SKIP_COMPRESSED
				/*
				 * Dedicated SKIP_QP arm (Phase 2): the
				 * chain-compress migration has reshaped the
				 * QP target to skip-variant layout (ptrs at
				 * byte 16, subkey extension at bytes 8-15),
				 * so we inline-cmp up to 13B of cached
				 * subkey, advance past the compressed prefix,
				 * and consume one more byte via the
				 * skip-variant byte-step (is_skip=true) — all
				 * in this block.  The SKIP_QP node_flag stays
				 * tagged here so the QP_HI fast path's narrow
				 * test (FT_KIND_QP only) does NOT match
				 * SKIP_QP-tagged input; this arm is the one
				 * place SKIP_QP descent lives.  Tag-equality
				 * branch is predict-friendly (a single
				 * compare-equal against a constant).
				 */
				if (caa_likely(skip_kind == FT_KIND_SKIP_QP)) {
					struct cds_ft_qp16_node *qp =
						(struct cds_ft_qp16_node *)
						/* SUB: tag known to be FT_KIND_SKIP_QP. */
						((unsigned long) node_flag - FT_KIND_SKIP_QP);

					if (spec_validate) {
						if (skip <= FT_QP16_SUBKEY_INLINE_LEN) {
							const struct ft_qp16_skip_meta *meta =
								ft_qp16_skip_meta_at(qp);

							if (caa_unlikely(ft_key_cmp_ordinals(
									key, meta->subkey,
									skip, skip,
									false, NULL) != 0)) {
								status = CDS_FT_STATUS_NOT_FOUND;
								goto end;
							}
						} else {
							if (first_skip_offset == key_len)
								first_skip_offset = i;
							needs_leaf_validate = true;
						}
					}

					/* Advance past the compressed prefix. */
					key += skip;
					i += skip;

					/*
					 * Consume one more byte via the skip-
					 * variant byte-step (ptrs at byte 16).
					 * The for-loop's i++ then advances past
					 * this byte; the next iter dispatches
					 * the returned child via the loop-top
					 * fast paths.
					 */
					iter_key = *(key++);
					node_flag = ft_qp_byte_get(qp, NULL,
							iter_key, FT_PF_DATA,
							true);
					if (caa_unlikely(!node_flag)) {
						status = CDS_FT_STATUS_NOT_FOUND;
						goto end;
					}
					FT_BYTE_STEP_POST();
					continue;
				}
				/*
				 * Speculative-validated lookup only:
				 * QP skip with skip_len ≤ 5 → validate
				 * immediately against the inline subkey on
				 * the QP node header CL (already loaded by
				 * the descent that brought us here).  Other
				 * cases (PIGEON, QP with skip_len > 5)
				 * defer to the end-of-descent leaf-bytes
				 * compare via needs_leaf_validate.  The
				 * SKIP_EXT case is handled by the fast path
				 * above and never reaches here.
				 *
				 * Pure candidate mode (caller-validated)
				 * leaves the skip unchecked here and lets
				 * the caller do its own leaf compare.
				 */
				if (spec_validate) {
					if (skip_kind == FT_KIND_SKIP_POPCOUNT_32 &&
					    skip <= FT_PC32_SUBKEY_INLINE_LEN) {
						/*
						 * POPCOUNT_32 skip variant: cached
						 * subkey lives in slot 0 of the
						 * target node, written by
						 * ft_publish_compressed when the
						 * cn was published — but only when
						 * pc_meta->nr_child <=
						 * FT_PC32_MAX_LC_SKIP at publish
						 * time (otherwise slot 0 holds a
						 * live ptr).  The is_skip
						 * metadata bit signals subkey
						 * validity; without it, slot 0 is
						 * a live ptr and the inline compare
						 * would read garbage.  Fall back to
						 * needs_leaf_validate when not
						 * skip-variant.
						 */
						const struct ft_pc32_node *pc =
							(const struct ft_pc32_node *)
							/* SUB: tag known to be FT_KIND_SKIP_POPCOUNT_32. */
							((unsigned long) node_flag - FT_KIND_SKIP_POPCOUNT_32);

						if (cds_ft_item_to_metadata((void *) pc)->is_skip) {
							if (caa_unlikely(ft_key_cmp_ordinals(
									key, pc->u.skip.meta.subkey,
									skip, skip,
									false, NULL) != 0)) {
								status = CDS_FT_STATUS_NOT_FOUND;
								goto end;
							}
						} else {
							if (first_skip_offset == key_len)
								first_skip_offset = i;
							needs_leaf_validate = true;
						}
					} else if (skip_kind == FT_KIND_SKIP_POPCOUNT_64 &&
					    skip <= FT_PC32_SUBKEY_INLINE_LEN) {
						/*
						 * POPCOUNT_64 skip variant: same
						 * cached-subkey layout as
						 * POPCOUNT_32 (both reuse struct
						 * ft_pc32_skip_meta in slot 0).
						 * Same is_skip gate —
						 * publish only writes slot 0 when
						 * nr_child <= FT_PC64_MAX_LC_SKIP.
						 */
						const struct ft_pc64_node *pc =
							(const struct ft_pc64_node *)
							/* SUB: tag known to be FT_KIND_SKIP_POPCOUNT_64. */
							((unsigned long) node_flag - FT_KIND_SKIP_POPCOUNT_64);

						if (cds_ft_item_to_metadata((void *) pc)->is_skip) {
							if (caa_unlikely(ft_key_cmp_ordinals(
									key, pc->u.skip.meta.subkey,
									skip, skip,
									false, NULL) != 0)) {
								status = CDS_FT_STATUS_NOT_FOUND;
								goto end;
							}
						} else {
							if (first_skip_offset == key_len)
								first_skip_offset = i;
							needs_leaf_validate = true;
						}
					} else {
						if (first_skip_offset == key_len)
							first_skip_offset = i;
						needs_leaf_validate = true;
					}
				}
#endif
				key += skip;
				/*
				 * i += skip - 1 (NOT skip): the for-loop's
				 * i++ at end-of-iter brings the total advance
				 * to `skip`, matching `key += skip`.  Preserves
				 * the descent loop invariant `i - 1 == bytes
				 * consumed` after the byte-step block was
				 * eliminated; the resolved skip target is now
				 * dispatched by next iter's loop-top fast paths
				 * (QP_HI / PIGEON / EXT / COMPRESSED).
				 */
				i += skip - 1;
				node_flag = ft_skip_child_ptr(node_flag);
				if (iter) {
					iter_path_node(iter)[i] = node_flag;
					iter_path_len = i + 1;
				}
#ifdef FEATURE_FT_SKIP_COMPRESSED
				/*
				 * Prefetch the next descent target.  For an
				 * external child under spec_validate, also
				 * warm the leaf-key region so the end-of-
				 * descent compare hides its load behind the
				 * rest of the descent.
				 */
				if (node_flag) {
					if (caa_likely(!ft_node_external_direct(node_flag))) {
						__builtin_prefetch(
							ft_node_ptr(node_flag));
					} else if (needs_leaf_validate) {
						__builtin_prefetch(
							(const uint8_t *) ft_node_ptr(node_flag) +
							ft->group->speculative_key_offset +
							first_skip_offset);
					}
				}
#endif
				/*
				 * Resolved kind (QP_HI / PIGEON / POPCOUNT_32 /
				 * POPCOUNT_64 / COMPRESSED) is dispatched by the
				 * next iter's loop-top fast paths.  Without
				 * `continue` here, the in-iter PIGEON / POPCOUNT
				 * fast paths below would steal an extra byte step
				 * that the off-by-one in `i += skip - 1` accounts
				 * for in the natural for-loop i++; FT_BYTE_STEP_-
				 * POST would then mistake a key-terminating
				 * external for mid-descent EXT and bail.  SKIP_EXT
				 * (any cand mode) was already absorbed by the
				 * fast path above with `break`; this code path
				 * only sees SKIP_QP / SKIP_PIGEON / SKIP_POPCOUNT_-
				 * 32 / SKIP_POPCOUNT_64, whose resolved targets
				 * are internal nodes.
				 */
				continue;
			}
		}
		/*
		 * PIGEON fast path (3/5 in the flat dispatch sequence):
		 * rare on sparse workloads but a major case on dense ones
		 * (post-QP→PIGEON escalation).  After QP_HI / SKIP / EXT
		 * filters, node_flag may be PIGEON (0x01), COMPRESSED
		 * (0x04), or — when the previous iter's SKIP handler in
		 * descend_cand mode resolved a SKIP_QP / SKIP_PIGEON —
		 * an internal-aligned QP (0x05) or PIGEON (0x01) target
		 * passed through.  A bit-0 test would conflate PIGEON
		 * with QP under Candidate E (both have bit 0 = 1).  Use
		 * the full 5-bit mask check for PIGEON exclusively;
		 * resolved QP targets fall through this block and the
		 * COMPRESSED handler, ending the iter so the next iter's
		 * QP_HI fast path catches them.
		 *
		 * Constant-tag SUB strips FT_KIND_PIGEON; ft_pigeon_node_get_nth
		 * inlines the data[n] load.  Same iter-path / mid-EXT /
		 * track-prefix bookkeeping as QP_HI.
		 */
		if (((unsigned long) node_flag & 0x1FUL) == FT_KIND_PIGEON) {
			struct cds_ft_inode *node =
				(struct cds_ft_inode *)
				((unsigned long) node_flag - FT_KIND_PIGEON);

			iter_key = *(key++);
			node_flag = ft_pigeon_node_get_nth(NULL, node, NULL,
					iter_key, FT_PF_DATA);
			if (caa_unlikely(!node_flag)) {
				status = CDS_FT_STATUS_NOT_FOUND;
				goto end;
			}
			FT_BYTE_STEP_POST();
			continue;
		}
		/*
		 * POPCOUNT_32 fast path: 2L nibble bitmap byte step on a
		 * sparse internal node (1-3 children).  Tag-specific test
		 * `(v & 0x1F) == FT_KIND_POPCOUNT_32` (= 0x09) — POPCOUNT_-
		 * 32 has bit 0 + bit 3 set, distinct from QP / PIGEON.  Skip
		 * variant (SKIP_POPCOUNT_32, 0x0B) is filtered upstream by
		 * the SKIP handler (bit 1 set).
		 */
		if (((unsigned long) node_flag & 0x1FUL) == FT_KIND_POPCOUNT_32) {
			struct ft_pc32_node *pc =
				(struct ft_pc32_node *)
				((unsigned long) node_flag - FT_KIND_POPCOUNT_32);

			iter_key = *(key++);
			node_flag = ft_pc32_node_get_nth_skip(pc, NULL,
					iter_key, FT_PF_DATA);
			if (caa_unlikely(!node_flag)) {
				status = CDS_FT_STATUS_NOT_FOUND;
				goto end;
			}
			FT_BYTE_STEP_POST();
			continue;
		}
		/*
		 * POPCOUNT_64 fast path: scan_6 (5+3 byte split) flat-packed
		 * popcount byte step on a medium-density internal node (4-6
		 * children).  Tag-specific test `(v & 0x1F) == FT_KIND_-
		 * POPCOUNT_64` (= 0x11) — POPCOUNT_64 has bit 0 + bit 4 set,
		 * distinct from QP / PIGEON / POPCOUNT_32.  Skip variant
		 * (SKIP_POPCOUNT_64, 0x13) is filtered upstream by the SKIP
		 * handler (bit 1 set).
		 */
		if (((unsigned long) node_flag & 0x1FUL) == FT_KIND_POPCOUNT_64) {
			struct ft_pc64_node *pc =
				(struct ft_pc64_node *)
				((unsigned long) node_flag - FT_KIND_POPCOUNT_64);

			iter_key = *(key++);
			node_flag = ft_pc64_node_get_nth_skip(pc, NULL,
					iter_key, FT_PF_DATA);
			if (caa_unlikely(!node_flag)) {
				status = CDS_FT_STATUS_NOT_FOUND;
				goto end;
			}
			FT_BYTE_STEP_POST();
			continue;
		}
		/*
		 * EXT terminal (4/5 in the flat dispatch sequence): an
		 * external child at loop top means the key terminates at
		 * a compressed path end (e.g. a compressed node's cn->child
		 * is EXT, surfaced in node_flag by the COMPRESSED handler
		 * below in a previous iter).  Break to the post-loop
		 * terminal handler.
		 *
		 * Candidate E: use ft_node_external_direct (`& 0x07 == 0`)
		 * — a direct `& FT_KIND_MASK == FT_KIND_EXT` would leak
		 * bit 4 of the 16-byte-aligned leaf address.
		 */
		if (ft_node_external_direct(node_flag))
			break;
		/*
		 * COMPRESSED handler (5/5 in the flat dispatch sequence):
		 * residual after QP_HI / SKIP / PIGEON / EXT.  Reachable
		 * for COMPRESSED slots (chain-compress middles, including
		 * non-cand SKIP fall-through that converted SKIP_* → COMPRESSED)
		 * and as harmless no-op for QP_HI from cand SKIP fall-through
		 * (continues to next iter where QP_HI fast path catches).
		 *
		 * Candidate E: use ft_node_compressed_in_node (`& 0x07 ==
		 * 0x04`) — bit 4 of a 16-byte-aligned compressed address
		 * leaks through `& FT_KIND_MASK` (= 0x1F).
		 */
		assert(((unsigned long) node_flag & FT_KIND_SKIP_BIT) == 0);
		if (ft_node_compressed_in_node(node_flag)) {
			enum ft_descent_action act;

			i--;
			act = ft_lookup_compressed(&node_flag, &key, &i,
				key_depth, iter, &iter_path_len,
				track, track_longest,
				&match_len, &match_node, &found, &status,
				descend_cand,
				spec_validate ? &needs_leaf_validate : NULL,
				spec_validate ? &first_skip_offset : NULL);
			if (act == FT_DESCENT_END)
				goto end;
			if (act == FT_DESCENT_BREAK)
				break;
			continue;
		}

	}
#undef FT_BYTE_STEP_POST

	/*
	 * Reached key_depth, check for terminal node: either external
	 * nodes or internal/compressed node associated with external nodes.
	 */
	if (ft_node_internal(node_flag)) {
		struct cds_ft_metadata *metadata = ft_flag_to_metadata_fast(node_flag);
		found = ft_dereference_prefetch_external(metadata->external_nodes);
		status = found ? CDS_FT_STATUS_OK : CDS_FT_STATUS_NOT_FOUND;
		if (track && (found || track_longest)) {
			match_len = key_len;
			match_node = found;
		}
	} else if (ft_node_compressed_in_node(node_flag)) {
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
	 * Speculative-validated lookup: a single end-of-descent leaf-
	 * bytes compare catches mis-descents from cand-mode steps that
	 * did not validate inline.  Skip-compressed steps with QP
	 * targets and skip_len ≤ 5 have already validated against the
	 * inline subkey on the QP node header CL during descent; only
	 * descents that flagged needs_leaf_validate (PIGEON / EXT skips,
	 * QP skips with skip_len > 5, or any cn-handler step in cand
	 * mode) need the leaf load here.  When every cand-mode step
	 * matched inline, the leaf load is skipped entirely.
	 */
	if (spec_validate && needs_leaf_validate
	    && status == CDS_FT_STATUS_OK && found
	    && first_skip_offset < key_len) {
		const uint8_t *leaf_key = (const uint8_t *) found +
				ft->group->speculative_key_offset;
		size_t cmp_len = key_len - first_skip_offset;

		/*
		 * For variable-length groups, also verify the leaf's
		 * stored key length matches the input key length.  This
		 * catches the input-shorter-than-leaf case that the
		 * SKIP_EXT fast path no longer detects via the descent
		 * bounds check (skip_len > remaining).  For fixed-length
		 * groups, length match is implicit.
		 */
		if (ft->group->speculative_key_len_offset !=
				CDS_FT_SPECULATIVE_OFFSET_NONE) {
			size_t stored_len = *(const size_t *)
				((const uint8_t *) found +
				 ft->group->speculative_key_len_offset);

			if (caa_unlikely(stored_len != key_len)) {
				found = NULL;
				status = CDS_FT_STATUS_NOT_FOUND;
				goto specv_done;
			}
		}
		/*
		 * Compare only [first_skip_offset, key_len): bytes before
		 * first_skip_offset were validated implicitly by descent
		 * (QP/PIGEON byte-steps each consume one byte; FT_KIND_-
		 * COMPRESSED steps validate cn->len bytes inline; SKIP_QP
		 * with skip_len ≤ 5 validates inline against qp->subkey).
		 * Bytes at and after first_skip_offset may include trusted-
		 * but-unvalidated bytes from one or more deferred skips.
		 *
		 * Short-tail widening: if cmp_len < 8 but key_len >= 8,
		 * extend the compare to 8 bytes anchored at (key_len - 8).
		 * The leading "extra" bytes [key_len - 8, first_skip_offset)
		 * were already validated by descent, so they match by
		 * construction; widening costs nothing semantically and
		 * collapses ft_key_cmp_ordinals' length dispatch to the
		 * 8-byte word path (single unaligned 64-bit load + cmp).
		 * spec_validate requires km.identity, so raw-byte unaligned
		 * access is correct.
		 */
		size_t cmp_off = first_skip_offset;

		if (cmp_len < 8 && key_len >= 8) {
			cmp_off = key_len - 8;
			cmp_len = 8;
		}
		if (ft_key_cmp_ordinals(orig_key + cmp_off,
				leaf_key + cmp_off,
				(unsigned int) cmp_len,
				(unsigned int) cmp_len,
				false, NULL) != 0) {
			found = NULL;
			status = CDS_FT_STATUS_NOT_FOUND;
		}
	}
specv_done:
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
		 * cn->child is a slot-context value; resolve once here so the
		 * caller's descent loop can drop its top-of-iteration resolve.
		 * See ft_descent_traverse_compressed for the same contract.
		 */
		node_flag = ft_resolve_skip_compressed(node_flag);
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
			node_flag = ft_resolve_skip_compressed(node_flag);
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
	node_flag = ft_resolve_skip_compressed(node_flag);
	iter_path_node(iter)[level] = node_flag;
	iter_path_node(iter)[level + 1] = node_flag;
	if (ft_node_external_direct(node_flag))
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
	if (caa_likely(node_flag != NULL))
		node_flag = ft_resolve_skip_compressed(node_flag);
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
	if (ft_node_external_direct(node_flag))
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
		if (ft_node_compressed_in_node(node_flag) ||
		    ft_node_skip_compressed_in_slot(node_flag)) {
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
		if (!ft_node_external_direct(node_flag))
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

		if (ft_node_compressed_in_node(node_flag)) {
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
		if (ft_node_external_direct(node_flag))
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
				struct cds_ft_metadata *metadata =
					ft_flag_to_metadata_fast(node_flag);

				external_nodes = ft_dereference_prefetch_external(metadata->external_nodes);
			} else if (ft_node_compressed_in_node(node_flag)) {
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
			!ft_node_external_direct(node_flag))
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
		    !ft_node_external_direct(iter_path_node(iter)[level])) {
			struct cds_ft_metadata *metadata;

			if (ft_node_compressed_in_node(iter_path_node(iter)[level]))
				metadata = cds_ft_item_to_metadata(
					ft_node_ptr(iter_path_node(iter)[level]));
			else
				metadata = ft_flag_to_metadata_fast(
					iter_path_node(iter)[level]);
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

			if (!ft_node_external_direct(pfx_flag)) {
				struct cds_ft_metadata *metadata;

				if (ft_node_compressed_in_node(pfx_flag))
					metadata = cds_ft_item_to_metadata(
						ft_node_ptr(pfx_flag));
				else
					metadata = ft_flag_to_metadata_fast(pfx_flag);
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
	if (ft_node_external_direct(node_flag)) {
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
			struct cds_ft_metadata *metadata = ft_flag_to_metadata_fast(node_flag);
			struct cds_ft_node *external_nodes = ft_dereference_prefetch_external(metadata->external_nodes);

			if (external_nodes) {
				ret_node = external_nodes;
				level--;
				goto found_minmax;
			}
		}
		/* Return external node. */
		if (ft_node_external_direct(node_flag))
			break;
		/*
		 * node_flag is post-resolve at every loop iteration:
		 *  - first iter: from descend_children entry, where any
		 *    cn->child read in ft_inequality_compressed already
		 *    resolves; or from the going_up phase via
		 *    ft_node_get_leftright (resolves through
		 *    ft_node_get_direction);
		 *  - subsequent iters: from ft_node_get_minmax (below) or
		 *    ft_inequality_minmax_compressed, which both resolve
		 *    cn->child before exposing it.
		 *
		 * Compressed node: traverse through the compressed
		 * path to reach the child.  Fill ordinal_key and
		 * iter path as we go.
		 */
		if (ft_node_compressed_in_node(node_flag)) {
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
		node_flag = ft_node_get_minmax(node_flag, &ordinal_key[level - 1], dir);
		/*
		 * Transiently empty internal/pool/pigeon: a reader may
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
		if (ft_node_external_direct(node_flag))
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
 * nr_keys accessors.
 *
 * ft_nr_keys_get: write-side load (under mutex).
 * ft_nr_keys_load: read-side acquire load.
 * ft_nr_keys_store: write-side store with caller-specified memory order.
 *
 * nr_keys is full unsigned-long width; the historical UINT32_MAX
 * "promoted" sentinel and the per-depth uint8_t density counters were
 * removed when the collapse heuristic that drove them was retired.
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
void ft_nr_keys_store(struct cds_ft *ft __attribute__((unused)),
		struct cds_ft_metadata *m, unsigned long val, int mo)
{
	uatomic_store(&m->nr_keys, val, mo);
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
 * Lo-nodes contribute 0: a (hi, lo) pair shares one byte step in the
 * trie — hi handles the hi-nibble, lo handles the lo-nibble — so the
 * upward walk treats the pair as one tier.
 *
 * Write-side only (mutex-held).
 */
static
unsigned int ft_parent_depth_span(struct cds_ft_inode_flag *parent_nf,
		struct cds_ft_inode_flag *child_nf __attribute__((unused)))
{
	if (ft_node_compressed_in_node(parent_nf)) {
		struct cds_ft_compressed_node *cn =
			ft_compressed_node_ptr(parent_nf);
		return cn->len;
	}
	/*
	 * QP-nibble lo-node: contributes 0 depth (the (hi, lo) pair shares
	 * one byte step).  Recovered via the is_lo metadata bit rather than
	 * a slot-tag — FT_KIND_QP_LO has been retired from the kind tag
	 * space.  Pigeon and other internal kinds have is_lo = 0 by
	 * zero-init at alloc.
	 */
	{
		struct cds_ft_metadata *m =
			cds_ft_item_to_metadata(ft_node_ptr(parent_nf));
		if (m->is_lo)
			return 0;
	}
	/* Internal node: dispatches on one key byte. */
	return 1;
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
	struct cds_ft_inode *cloned_orig_child = NULL;
	int ret;

	unsigned int junction_depth = node_depth + diverge_pos;

	/* Compute old child's nr_keys for the new nodes. */
	if (!ft_node_external_direct(cn->child)) {
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
		ft_nr_keys_store(ft, sfx_meta, old_child_nr_keys, CMM_RELAXED);
		old_suffix_flag = ft_compressed_node_flag(sfx);
		ft_set_parent(cn->child, old_suffix_flag, &sfx->child);
		old_suffix_flag = ft_publish_compressed(ft, sfx, old_suffix_flag);
		created[nr_created++] = old_suffix_flag;
	} else {
		/* suffix_len == 0: old child becomes a direct child of the
		 * junction (no compressed wrapper).  For internal kinds
		 * (QP/PIGEON), placing cn->child directly would demote its
		 * meta->parent across the COMPRESSED→non-COMPRESSED boundary
		 * in place, racing with concurrent readers holding stale
		 * SKIP_X(cn->child) pointers.  Clone instead: the clone takes
		 * the new parent context (placed under junction), while the
		 * original keeps its old COMPRESSED parent (cn) and stays
		 * alive for the grace period until cn is RCU-freed.
		 *
		 * EXT children: in non-spec groups the upstream slot has no
		 * SKIP_EXT (Phase 1 mutator gate), so no race; in spec groups
		 * the read path will validate against the EXT's stored key
		 * directly (length-recovery redesign), so ext->prev is
		 * mutator-only and the in-place demote is safe.
		 */
		if (cn->child && !ft_node_external_direct(cn->child)) {
			old_suffix_flag = ft_node_clone_for_reparent(ft,
				cn->child, &cloned_orig_child);
			if (!old_suffix_flag) goto error;
			created[nr_created++] = old_suffix_flag;
		} else {
			old_suffix_flag = cn->child;
		}
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
		ft_nr_keys_store(ft, nb_meta, 1, CMM_RELAXED);
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
		ft_nr_keys_store(ft, pfx_meta, ft_nr_keys_get(cn_meta) + 1, CMM_RELAXED);
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
		ft_nr_keys_store(ft, pfx_meta, ft_nr_keys_get(cn_meta) + 1, CMM_RELAXED);
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
	 * RCU-free the original cn->child if we cloned it.  The clone
	 * lives in the new structure (junction's slot); the original is
	 * unreferenced from the new tree but remains the target of any
	 * stale SKIP_X(orig) reader holding cn (still alive below) as
	 * its parent.  Deferred free so those readers drain first.
	 */
	if (cloned_orig_child)
		free_cds_ft_node(ft, cloned_orig_child);

	/* 7. Free the old compressed node. */
	free_compressed_node(ft, cn);

	return 0;

error:
	{
		int i;

		for (i = 0; i < nr_created; i++) {
			if (ft_node_compressed_in_slot(created[i]))
				free_compressed_node_unpublished(ft,
					ft_compressed_node_ptr(created[i]));
			else if (ft_node_skip_compressed_in_slot(created[i]))
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
		struct cds_ft_inode **cloned_orig_child_ret,
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
	struct cds_ft_inode *cloned_orig_child = NULL;
	int ret;

	*cloned_orig_child_ret = NULL;

	if (!ft_node_external_direct(cn->child)) {
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
		struct cds_ft_inode_flag *child_for_dest;

		/*
		 * Same parent-kind boundary concern as ft_split_compressed_-
		 * insert (suffix_len == 0): cn->child gets installed under a
		 * fresh non-COMPRESSED dest, demoting its meta->parent across
		 * the boundary in place.  Clone for QP/PIGEON; EXT is handled
		 * by the Phase 1 SKIP_EXT gate (non-spec) or the read-side
		 * length-recovery redesign (spec).
		 */
		if (cn->child && !ft_node_external_direct(cn->child)) {
			child_for_dest = ft_node_clone_for_reparent(ft,
				cn->child, &cloned_orig_child);
			if (!child_for_dest) goto error;
			created[nr_created++] = child_for_dest;
		} else {
			child_for_dest = cn->child;
		}
		ret = ft_node_set_nth(ft, &dest,
			cn->key_bytes[remaining + 1],
			child_for_dest, NULL, NULL,
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
		/* suffix_len == 0: cn->child becomes a direct child of the
		 * junction.  Same boundary-crossing concern; clone for
		 * QP/PIGEON. */
		if (cn->child && !ft_node_external_direct(cn->child)) {
			suffix_flag = ft_node_clone_for_reparent(ft,
				cn->child, &cloned_orig_child);
			if (!suffix_flag) goto error;
			created[nr_created++] = suffix_flag;
		} else {
			suffix_flag = cn->child;
		}
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

	FT_TP(compressed_split, "key_shorter", (const void *) cn, cn->len,
		(const void *) top_flag, remaining);
	*top_ret = top_flag;
	*jct_ret = jct_flag;
	*cloned_orig_child_ret = cloned_orig_child;
	return 0;

error:
	{
		int i;

		for (i = 0; i < nr_created; i++) {
			if (ft_node_compressed_in_slot(created[i]))
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
	ft_nr_keys_store(ft, cn_meta, 1, CMM_RELAXED);
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

static
int ft_attach_node(struct cds_ft *ft,
		struct cds_ft_inode_flag **attach_node_flag_ptr,
		struct cds_ft_inode_flag *attach_node_flag,
		struct cds_ft_inode_flag **old_node_flag_ptr __attribute__((unused)),
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
	if ((!ft_node_compressed_in_slot(iter_node_flag) &&
	     !ft_node_skip_compressed_in_slot(iter_node_flag)) ||
	    external_nodes) {
		for (i = (ft_node_compressed_in_slot(iter_node_flag) ||
			  ft_node_skip_compressed_in_slot(iter_node_flag)) ?
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
			if (ft_node_compressed_in_slot(created_nodes[i]))
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
	struct cds_ft_inode *cloned_orig_child = NULL;
	int sret;

	sret = ft_split_compressed_key_shorter(ft,
		d->nf, remaining, &top_flag, &jct_flag,
		&cloned_orig_child, d->depth);
	if (sret)
		return sret;
	ft_set_parent(top_flag, d->pnf, d->nfp);
	ft_publish_to_parent(ft, d->pnf, d->nfp, top_flag);
	{
		struct cds_ft_metadata *jct_meta =
			cds_ft_item_to_metadata(
				ft_node_ptr(jct_flag));
		assert(!ft_node_compressed_in_node(jct_flag));
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
	/*
	 * RCU-free the cloned original child (if any) before freeing cn.
	 * After ft_publish_to_parent above, both cn and the original
	 * child are unreferenced from the new tree, but stale SKIP_X
	 * readers still resolve via original → original->parent = cn.
	 * Both must outlive the grace period; both are call_rcu-deferred.
	 */
	if (cloned_orig_child)
		free_cds_ft_node(ft, cloned_orig_child);
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
		    (ft_node_skip_compressed_in_slot(cn->child) ||
		     ft_node_internal(cn->child) ||
		     ft_node_compressed_in_node(cn->child))) {
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
		 * d.nf is post-resolve at every loop top: ft_descent_init
		 * resolves ft->root, ft_descent_step calls ft_node_get_nth
		 * which resolves, and ft_descent_traverse_compressed
		 * resolves cn->child before returning.  No additional
		 * resolve needed here.
		 */
		/* Found external node. */
		if (ft_node_external_direct(d.nf))
			break;
		/* Decompress compressed node before continuing descent. */
		/*
		 * Compressed node: compare remaining key bytes with
		 * the compressed path.  If they match, traverse
		 * through to the child.  If they diverge, decompress
		 * at this point and restart.
		 */
		if (ft_node_compressed_in_node(d.nf)) {
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
	 * d.nf is post-resolve at exit: every path that sets it
	 * (ft_descent_init, ft_descent_step, ft_descent_traverse_compressed)
	 * resolves SKIP_X internally.
	 */

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

		} else if (ft_node_compressed_in_node(d.nf)) {
			/*
			 * Key ends at a compressed node's depth.
			 * Split: internal(external_nodes) + compressed(len-1).
			 */
			ret = ft_insert_compressed_key_shorter(ft, &d, 0,
				node, unique_node_ret);
		} else if (!ft_node_external_direct(d.nf)) {
			struct cds_ft_node *external_nodes;
			struct cds_ft_metadata *metadata;

			assert(!ft_node_compressed_in_node(d.nf));
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
		/*
		 * d.nf is post-resolve at every loop top: see ft_descent_init
		 * / ft_descent_step / ft_descent_traverse_compressed.
		 */
		if (ft_node_external_direct(d.nf))
			break;
		if (ft_node_compressed_in_node(d.nf)) {
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
	/* d.nf is post-resolve at loop exit (every setter resolves). */

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
		} else if (ft_node_compressed_in_node(d.nf)) {
			/*
			 * Key ends at a compressed node's depth.
			 * Split: internal(external_nodes) + compressed(len-1).
			 */
			ret = ft_insert_compressed_key_shorter(ft, &d, 0,
				node, old_node_ret);
			if (ret == -EEXIST)
				ret = 0;	/* Replace handled by key_shorter. */
		} else if (!ft_node_external_direct(d.nf)) {
			struct cds_ft_node *external_nodes;
			struct cds_ft_metadata *metadata;

			assert(!ft_node_compressed_in_node(d.nf));
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

		if (ft_node_external_direct(node_flag)) {
			s = CDS_FT_STATUS_NOT_FOUND;
			FT_TP(replace_exit, (int) s);
			return s;
		}
		if (ft_node_compressed_in_node(node_flag)) {
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
	if (!ft_node_external_direct(node_flag)) {
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
	if (ft_node_external_direct((struct cds_ft_inode_flag *) old_node->prev)) {
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

		if (ft_node_skip_compressed_in_slot(iter_node_flag))
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
		unsigned int detach_depth)
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
		 * falsely matching ft_node_external_direct and skipping the
		 * external_nodes preservation entirely.
		 */
		detach_child = ft_resolve_skip_compressed(detach_child);

		if (detach_child && !ft_node_external_direct(detach_child)) {
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
			if (!ft_node_compressed_in_node(cur))
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
				else if (ft_node_compressed_in_node(parent_nf) ||
					 ft_node_skip_compressed_in_slot(parent_nf)) {
					struct cds_ft_compressed_node *pcn;

					if (ft_node_skip_compressed_in_slot(parent_nf))
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
	if (ft_node_compressed_in_slot(iter_node_flag) ||
	    ft_node_skip_compressed_in_slot(iter_node_flag)) {
		ret = ft_detach_node_replace_compressed_parent(ft,
			iter_node_flag, detach_parent_flag_ptr,
			topmost_external_nodes, &nr_clear);
		if (ret)
			goto end;
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
				       !ft_node_external_direct(walk_nf) &&
				       nr_to_free < FT_MAX_DEPTH) {
					struct cds_ft_inode_flag *next = NULL;

					if (ft_node_compressed_in_node(walk_nf) ||
					    ft_node_skip_compressed_in_slot(walk_nf)) {
						struct cds_ft_compressed_node *cn;
						struct cds_ft_metadata *cm;

						if (ft_node_skip_compressed_in_slot(walk_nf))
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
					if (ft_node_compressed_in_node(to_free[fi]) ||
					    ft_node_skip_compressed_in_slot(to_free[fi]))
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
	 * Skip for compressed parents: the replacement was
	 * already published inline above.
	 */
	if (!ft_node_compressed_in_slot(iter_node_flag) &&
	    !ft_node_skip_compressed_in_slot(iter_node_flag)) {
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
					&surviving_byte, FT_LEFTMOST);
			bool parent_compressed = ft_node_compressed_in_node(
					iter_meta->parent);
			bool child_compressed = surviving_child
				&& ft_node_compressed_in_node(surviving_child);
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
					ft_nr_keys_store(ft, new_cn_meta,
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
						free_cds_ft_node(ft,
							(struct cds_ft_inode *)
							parent_cn);
					if (child_cn)
						free_cds_ft_node(ft,
							(struct cds_ft_inode *)
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
		!ft_node_external_direct((struct cds_ft_inode_flag *) node->prev));
	if (next_node)
		next_node->prev = node->prev;
	if (ft_node_external_direct((struct cds_ft_inode_flag *) node->prev)) {
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
		/*
		 * dd.d.nf is post-resolve at every loop top:
		 * ft_detach_descent_init -> ft_descent_init resolves;
		 * ft_descent_step / ft_descent_traverse_compressed resolve.
		 */

		/*
		 * Compressed node: compare remaining key bytes with
		 * the compressed path.  If they match, traverse
		 * through to the child.  If they diverge, the key
		 * is not present.
		 */
		if (ft_node_compressed_in_node(dd.d.nf)) {
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

	if (!ft_node_external_direct(dd.d.nf)) {
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
			if (!ft_node_external_direct((struct cds_ft_inode_flag *) match->prev)
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
		if (ft_node_compressed_in_node(dd.d.nf)) {
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

	if (!ft_node_external_direct(dd.d.nf)) {
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
	struct cds_ft_inode *cloned_orig_child = NULL;
	int ret;

	/* Compute old child's nr_keys. */
	if (!ft_node_external_direct(cn->child)) {
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
		struct cds_ft_inode_flag *child_for_dest;

		/* Clone QP/PIGEON cn->child to avoid in-place parent-kind
		 * boundary demote (see ft_node_clone_for_reparent). */
		if (cn->child && !ft_node_external_direct(cn->child)) {
			child_for_dest = ft_node_clone_for_reparent(ft,
				cn->child, &cloned_orig_child);
			if (!child_for_dest) goto error;
			created[nr_created++] = child_for_dest;
		} else {
			child_for_dest = cn->child;
		}
		ret = ft_node_set_nth(ft, &dest,
				cn->key_bytes[diverge_pos + 1],
				child_for_dest, NULL, NULL,
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
		/* suffix_len == 0: clone if QP/PIGEON; same boundary concern. */
		if (cn->child && !ft_node_external_direct(cn->child)) {
			old_suffix_flag = ft_node_clone_for_reparent(ft,
				cn->child, &cloned_orig_child);
			if (!old_suffix_flag) goto error;
			created[nr_created++] = old_suffix_flag;
		} else {
			old_suffix_flag = cn->child;
		}
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

	/* 5. Publish the split structure. */
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
		if (ft_node_compressed_in_slot(top_flag))
			d->pnfp = &ft_compressed_node_ptr(top_flag)->child;
		else if (ft_node_skip_compressed_in_slot(top_flag))
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

	/* RCU-free the cloned original child, if any (mirrors
	 * ft_split_compressed_insert and ft_split_compressed_key_shorter). */
	if (cloned_orig_child)
		free_cds_ft_node(ft, cloned_orig_child);

	/* 7. Free old compressed node. */
	free_compressed_node(ft, cn);

	return 0;

error:
	{
		int i;

		for (i = 0; i < nr_created; i++) {
			if (ft_node_compressed_in_slot(created[i]))
				free_compressed_node_unpublished(ft,
					ft_compressed_node_ptr(created[i]));
			else if (ft_node_skip_compressed_in_slot(created[i]))
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

	/*
	 * d->nf is post-resolve at every loop iteration: ft_descent_init
	 * resolves on entry; ft_descent_step (via ft_node_get_nth) and
	 * ft_descend_to_graft_point_compressed (via
	 * ft_descent_traverse_compressed) resolve before storing.
	 */
	for (; d->depth < key_len; ) {
		uint8_t kv;

		if (ft_node_external_direct(d->nf))
			break;
		if (ft_node_compressed_in_node(d->nf)) {
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
	struct cds_ft_inode *cloned_orig_child = NULL;

	if (!ft_node_external_direct(cn->child)) {
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
		struct cds_ft_inode_flag *child_for_dest;
		int ret;

		/* Clone QP/PIGEON cn->child to avoid in-place parent-kind
		 * boundary demote (see ft_node_clone_for_reparent). */
		if (cn->child && !ft_node_external_direct(cn->child)) {
			child_for_dest = ft_node_clone_for_reparent(ft,
				cn->child, &cloned_orig_child);
			if (!child_for_dest) return -1;
		} else {
			child_for_dest = cn->child;
		}
		ret = ft_node_set_nth(ft, &dest,
			cn->key_bytes[remaining],
			child_for_dest, NULL, NULL,
			d->depth + remaining);
		if (ret) {
			if (cloned_orig_child)
				free_cds_ft_node_unpublished(ft,
					ft_node_ptr(child_for_dest));
			return -1;
		}
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
			if (ft_node_compressed_in_node(suffix_flag))
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
				if (ft_node_compressed_in_node(suffix_flag))
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
			if (prefix_len >= 3 && ft_node_compressed_in_node(pfx_child))
				free_compressed_node_unpublished(ft,
					ft_compressed_node_ptr(pfx_child));
			if (ft_node_compressed_in_node(suffix_flag))
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
			if (ft_node_compressed_in_node(suffix_flag))
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

	d->ppnf = d->pnf;
	d->ppnfp = d->pnfp;
	d->pnf = prefix_flag;
	if (ft_node_compressed_in_node(prefix_flag))
		d->pnfp = &ft_compressed_node_ptr(prefix_flag)->child;
	else
		ft_node_get_nth(prefix_flag, &d->pnfp,
				cn->key_bytes[0], FT_PF_NONE);
	/*
	 * suffix_flag is the newly built sub-trie published into the
	 * parent's slot via ft_publish_compressed, so it may be
	 * SKIP-encoded under FEATURE_FT_SKIP_COMPRESSED.  Normalise
	 * d->nf to post-resolve form to match the contract upheld by
	 * ft_descent_init / ft_descent_step / ft_descent_traverse_compressed.
	 */
	d->nf = ft_resolve_skip_compressed(suffix_flag);
	d->nfp = d->pnfp;
	d->depth += remaining;

	/* RCU-free the cloned original child (if any). */
	if (cloned_orig_child)
		free_cds_ft_node(ft, cloned_orig_child);

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

			ft_nr_keys_store(ft, m,
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
				if (ft_node_compressed_in_node(cur)) {
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
	if (ft_node_compressed_in_node(top_node)) {
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
		unsigned long graft_external_count)
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
	} else {
		unsigned int i = d->depth;
		struct cds_ft_inode_flag *branch;
		struct cds_ft_node *displaced = NULL;

		if (d->nf && ft_node_external_direct(d->nf))
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
		status = ft_store_at_graft_point(dst_ft, key, key_len,
						  &d, old_src_root,
						  src_count);
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
		/*
		 * d.nf is post-resolve at ft_descend_to_graft_point exit:
		 * every path that sets it (ft_descent_init, ft_descent_step,
		 * ft_descent_traverse_compressed, and the split helpers) now
		 * stores the post-resolve form.  No additional resolve here.
		 */

		old_swap_root = swap_ft->root;
		swap_rmeta = ft_root_metadata(swap_ft);
		swap_empty = (swap_rmeta->nr_child == 0
				&& !swap_rmeta->external_nodes);
		swap_count = swap_empty ? 0 : ft_nr_keys_get(swap_rmeta);

		/* Compute old_count from the content being displaced. */
		if (!ft_node_external_direct(old_child)) {
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
				&ft_types[0],
				&fresh_meta);
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
		if (!ft_node_external_direct(old_child)) {
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
			if (ft_node_external_direct(dd.d.nf))
				return CDS_FT_STATUS_NOT_FOUND;
			if (ft_node_compressed_in_node(dd.d.nf)) {
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

			if (!ft_node_external_direct(child)) {
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
			if (!ft_node_external_direct(child)) {
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
	if (ft_node_compressed_in_slot(root_flag))
		return false;

	type_idx = ft_node_type_index(root_flag);
	type = &ft_types[type_idx];
	rmeta = cds_ft_item_to_metadata(root_node);

	/*
	 * For FT_QP the hi-bitmap can carry tombstones (bits left set
	 * when the lo-bucket was reclaimed via §4.6.1) so a bitmap != 0
	 * does not imply non-empty.  rmeta->nr_child is the live
	 * byte-children count (sum of lo nr_child across hi-buckets)
	 * and drops to zero only when all descendant bytes have been
	 * removed.  FT_POPCOUNT carries no tombstones; nr_child is
	 * authoritative.  PIGEON returns false here (a PIGEON root
	 * implies many populated entries — no empty PIGEON case worth
	 * handling).
	 */
	if (type->type_class != FT_QP && type->type_class != FT_POPCOUNT)
		return false;
	if (rmeta->nr_child)
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

		if (ft_node_external_direct(node_flag)) {
			count = 0;
			goto out;
		}
		if (ft_node_compressed_in_node(node_flag)) {
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
	if (ft_node_compressed_in_node(node_flag)) {
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
	if (!ft_node_external_direct(child)) {
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
	node_flag = ft_resolve_skip_compressed(node_flag);
	iter_path_node(iter)[level] = node_flag;
	if (ft_node_external_direct(node_flag)) {
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

		if (ft_node_external_direct(node_flag))
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

		if (ft_node_compressed_in_node(node_flag)) {
			enum ft_descent_action act;

			act = ft_lookup_nth_compressed(&node_flag,
				&level, ordinal_key, iter);
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
	if (node_flag && ft_node_external_direct(node_flag) && remaining == 0) {
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
			node_flag = ft_resolve_skip_compressed(node_flag);
			iter_path_node(iter)[level] = node_flag;
			if (ft_node_external_direct(node_flag)) {
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

		if (ft_node_external_direct(node_flag))
			break;

		metadata = cds_ft_item_to_metadata(ft_node_ptr(node_flag));

		if (ft_node_compressed_in_node(node_flag)) {
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
	if (node_flag && ft_node_external_direct(node_flag) && remaining == 0) {
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

		if (ft_node_external_direct(node_flag))
			return -1;

		if (ft_node_compressed_in_node(node_flag)) {
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
	if (ft_node_external_direct(node_flag)) {
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
	at_external_nodes = !ft_node_external_direct(iter_path_node(iter)[depth]);

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

			if (ft_node_compressed_in_node(parent)) {
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

		if (ft_node_external_direct(ancestor))
			continue;
		/*
		 * Compressed path levels have no siblings: skip.
		 */
		if (ft_node_compressed_in_node(ancestor))
			continue;
		/*
		 * Collapsed node: skip intermediate levels (same node
		 * at adjacent levels).  At the entry level, scan
		 * entries to the right and count their keys.
		 */

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

			if (ft_node_external_direct(node_flag))
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

			if (ft_node_compressed_in_node(node_flag)) {
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
		    ft_node_external_direct(iter_path_node(iter)[level]) &&
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
			if (ft_node_external_direct(node_flag)) {
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
	at_external_nodes = !ft_node_external_direct(iter_path_node(iter)[depth]);
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

		if (ft_node_external_direct(ancestor))
			continue;
		/*
		 * Skip intermediate compressed path levels (same
		 * compressed node at adjacent levels).  At the entry
		 * level, only external_nodes matter (no siblings).
		 */
		if (ft_node_compressed_in_node(ancestor)) {
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

			if (ft_node_external_direct(node_flag))
				break;

			metadata = cds_ft_item_to_metadata(
					ft_node_ptr(node_flag));

			if (ft_node_compressed_in_node(node_flag)) {
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
		    ft_node_external_direct(iter_path_node(iter)[level]) &&
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
	 * Skip-compressed pointer encoding is decided at build time via
	 * FEATURE_FT_SKIP_COMPRESSED — no per-group runtime gate.
	 * Skip-compressed no longer relies on high-bit pointer encoding
	 * (since Phase B.3/B.4), so the runtime mmap probe that used to
	 * validate VA-range availability is gone — the feature gate
	 * alone decides applicability.
	 */
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
	/* Half-CL accounting: T0 hi alone, no lo's yet. */
	metadata->qp_subtree_half_cls = (uint8_t) ft_node_half_cls(type0->order);
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

	if (!ft_node_skip_compressed_in_slot(slot_val))
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

		while (anc && ft_node_compressed_in_node(anc)) {
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
	if (ft_node_skip_compressed_in_slot(cn->child) ||
			(cn->child && ft_node_compressed_in_slot(cn->child))) {
		if (out)
			fprintf(out, "ft_verify: depth %u: compressed node %p has %s child %p (no two adjacent compresseds)\n",
				depth, node_flag,
				ft_node_skip_compressed_in_slot(cn->child) ?
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

			if (ft_node_skip_compressed_in_slot(slot_val))
				target_cn = ft_skip_to_compressed(slot_val);
			else if (slot_val &&
				 ft_node_compressed_in_slot(slot_val))
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
		if (ft_node_external_direct(cn->child)) {
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
	if (ft_node_compressed_in_node(node_flag))
		return ft_verify_node_compressed(ft, out, visited, path,
			node_flag, expected_parent, depth, out_nr_keys);

	/* --- Internal node (linear, pool, pigeon) --- */
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
		 * Type / alloc_index sanity.  ft_node_type_index already
		 * asserts the kind tag is a real internal kind, but does
		 * not verify the entry's type_class, that the arena order
		 * matches the type's expected order, or that nr_child fits
		 * the type's capacity.  A corrupted tag/bitfield write would
		 * otherwise survive verify and only manifest later as a
		 * wrong-sized scan or a min_child assertion.
		 */
		{
			unsigned int type_index = ft_node_type_index(node_flag);
			const struct cds_ft_type *type = &ft_types[type_index];
			size_t actual_order = cds_ft_item_order(node);
			/*
			 * Expected order: for FT_QP, the per-tier order
			 * lives in ft_qp16_tiers[]; ft_types[FT_QP_INDEX].order
			 * is only the T0 default for fresh allocations.
			 */
			unsigned int expected_order = type->order;

			if (type->type_class != FT_PIGEON
			    && type->type_class != FT_QP
			    && type->type_class != FT_POPCOUNT) {
				if (out)
					fprintf(out, "ft_verify: depth %u: internal node %p has non-internal type_class %d (type_index %u)\n",
						depth, node_flag,
						(int) type->type_class,
						type_index);
				return -1;
			}
			if (type->type_class == FT_QP) {
				if (actual_order < FT_QP16_T0_ALLOC_ORDER
				    || actual_order >= FT_QP16_T0_ALLOC_ORDER + FT_QP16_NR_TIERS) {
					if (out)
						fprintf(out, "ft_verify: depth %u: internal QP node %p alloc order %zu out of T0..T3 range\n",
							depth, node_flag,
							actual_order);
					return -1;
				}
				expected_order = (unsigned int) actual_order;
			}
			if (actual_order != expected_order) {
				if (out)
					fprintf(out, "ft_verify: depth %u: internal node %p alloc order %zu mismatches type %u expected order %u\n",
						depth, node_flag,
						actual_order, type_index,
						expected_order);
				return -1;
			}
			{
				unsigned int max_child;

#ifdef FEATURE_FT_SKIP_COMPRESSED
				/*
				 * Skip-target POPCOUNT nodes reserve slot 0 for
				 * cached subkey, dropping max by one.  The bit
				 * is meaningful only for FT_POPCOUNT; on QP /
				 * PIGEON it is harmless inheritance.
				 */
				max_child = (type->type_class == FT_POPCOUNT
					     && metadata->is_skip)
					? type->max_child_skip
					: type->max_child;
#else
				max_child = type->max_child;
#endif
				if (metadata->nr_child > max_child) {
					if (out)
						fprintf(out, "ft_verify: depth %u: internal node %p nr_child %u exceeds type %u max_child %u\n",
							depth, node_flag,
							metadata->nr_child, type_index,
							max_child);
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
			struct cds_ft_inode_flag *child_expected_parent =
				node_flag;

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
			/*
			 * A byte step traverses (hi, lo); the byte-keyed
			 * child's parent points at the lo-flag derived from
			 * the raw lo pointer in hi.ptrs[hi_idx], not at
			 * @node_flag.  PIGEON children are unchanged.
			 */
			if (((unsigned long) node_flag & FT_KIND_MASK) == FT_KIND_QP) {
				struct cds_ft_qp16_node *hi_n =
					(struct cds_ft_qp16_node *) node;
				uint16_t hi_bm = uatomic_load(&hi_n->bitmap,
						CMM_RELAXED);
				uint16_t hi_bit = (uint16_t) (1U << (key >> 4));
				unsigned int hi_idx;
				struct cds_ft_qp16_node *lo_raw;
#ifdef FEATURE_FT_SKIP_COMPRESSED
				/*
				 * Skip-variant qp HI puts ptrs[] at byte
				 * offset 16, not at the direct-layout
				 * struct-member offset 8.  Read is_skip from
				 * metadata (verify is the slow path; cold
				 * metadata load is fine here) and pick the
				 * correct base via ft_qp16_ptrs.
				 */
				bool hi_is_skip = metadata->is_skip;
#else
				bool hi_is_skip = false;
#endif

				assert(hi_bm & hi_bit);
				hi_idx = (unsigned int) __builtin_popcount(
						(unsigned int) (hi_bm
							& (hi_bit - 1U)));
				lo_raw = (struct cds_ft_qp16_node *)
					ft_qp16_ptrs(hi_n, hi_is_skip)[hi_idx];
				child_expected_parent =
					ft_qp16_lo_flag(lo_raw);
			}
			if (ft_node_external_direct(child)) {
				/* External leaf chain at this slot. */
				if (ft_verify_external_chain(ft, out, visited,
						path, child_expected_parent,
						(struct cds_ft_node *) ft_node_ptr(child),
						depth + 1))
					return -1;
				total_child_keys += 1;
			} else {
				unsigned long sub_keys = 0;

				if (ft_verify_node_recursive(ft, out, visited,
						path, child,
						child_expected_parent, depth + 1,
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
		if (((unsigned long) node_flag & FT_KIND_MASK) == FT_KIND_PIGEON) {
			struct cds_ft_bitmap *bm =
				cds_ft_item_to_bitmap(node, cds_ft_item_order(node));
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
#ifdef FEATURE_FT_COMPRESS
		/*
		 * Canonicalization (skip-compressed build, non-root): a
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
		 * In non-skip-compressed builds, ft_build_ordinal_chain
		 * keeps a 1-byte compressed floor at len >= 2, so a 1-child
		 * internal at the head of a length-1 chain is canonical and
		 * must not trip this check; this verify branch is
		 * compiled out via the inner FEATURE_FT_SKIP_COMPRESSED
		 * gate.
		 *
		 * Dual of the existing "no two adjacent compresseds" check.
		 */
#ifdef FEATURE_FT_SKIP_COMPRESSED
		if (expected_parent != NULL &&
		    counted_children == 1 && !external_nodes) {
			if (out)
				fprintf(out, "ft_verify: depth %u: internal node %p has 1 child and no external_nodes (should be a 1-byte compressed in skip mode)\n",
					depth, node_flag);
			return -1;
		}
#endif
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

#ifdef FEATURE_FT_VERIFY_AT_MUTATION
/*
 * Hook called from CDS_FT_SCOPED_WRITER's scope-exit, before the
 * writer claim is released.  Sampled by the per-trie
 * @verify_at_mutation_period: the full cds_ft_verify walk runs once
 * every @period mutations.  The counter is incremented and reset on
 * the boundary so it never exceeds @period - 1, avoiding any overflow
 * / cadence-drift issue on long-running workloads.  Period 0 disables
 * the walk entirely (only the increment-and-compare runs).  On any
 * mismatch the verifier runs to completion before aborting so we get
 * the full diagnostic.
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
		} else if (ft_node_compressed_in_node(child_node_flag)) {
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
			    !ft_node_external_direct(cn->child))
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

		(void) rm;
		print_indent(out, level);
		fprintf(out, "Level 0: root node %p\n", node_flag);
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
void json_emit_node(const struct cds_ft *ft, FILE *out,
		struct cds_ft_inode_flag *node_flag, int level)
{
	if (!node_flag) {
		fprintf(out, "null");
		return;
	}
	if (ft_node_external_direct(node_flag)) {
		fprintf(out, "{\"ptr\":\"%p\",\"kind\":\"EXTERNAL\","
			"\"level\":%d}", node_flag, level);
		return;
	}
	if (ft_node_compressed_in_node(node_flag)) {
		struct cds_ft_compressed_node *cn =
			ft_compressed_node_ptr(node_flag);
		struct cds_ft_metadata *metadata =
			cds_ft_item_to_metadata((struct cds_ft_inode *) cn);
		struct cds_ft_node *external_nodes =
			rcu_dereference(metadata->external_nodes);
		unsigned int j;

		fprintf(out, "{\"ptr\":\"%p\",\"kind\":\"COMPRESSED\","
			"\"level\":%d,\"path_len\":%u,\"nr_keys\":%lu,"
			"\"key_bytes\":[",
			node_flag, level, (unsigned int) cn->len,
			ft_nr_keys_get(metadata));
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
		unsigned int type_index = ft_node_type_index(node_flag);
		unsigned int key, printed = 0;

		fprintf(out, "{\"ptr\":\"%p\",\"kind\":\"%s\",\"level\":%d,"
			"\"nr_child\":%u",
			node_flag, internal_type_name(type_index), level,
			metadata->nr_child);
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
	struct cds_ft_node_stats node_stats[FT_NUM_INTERNAL_TYPES];
	bool has_nodes;
};

struct cds_ft_stats {
	struct cds_ft_stats_level level[FT_MAX_DEPTH];
	uint64_t compressed_len_dist[256];
	uint64_t skip_compressed_len_dist[256];
	uint64_t nr_skip_compressed_total;
	uint64_t nr_skip_compressed_qp;
	uint64_t nr_skip_compressed_pigeon;
	uint64_t nr_skip_compressed_popcount_32;
	uint64_t nr_skip_compressed_popcount_64;
	uint64_t nr_skip_compressed_ext;
	uint64_t qp_hi_half_cls_dist[256];
	uint64_t nr_qp_hi_total;
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
	unsigned long node_type = ft_node_type_index(node_flag);
	struct cds_ft_node_stats *node_stats = &stats->level[level].node_stats[node_type];
	const struct cds_ft_metadata *metadata;

	metadata = cds_ft_item_to_metadata(ft_node_ptr(node_flag));
	node_stats->count++;
	node_stats->distribution[metadata->nr_child]++;
	stats->level[level].nr_internal_nodes++;
	stats->level[level].has_nodes = true;
	if (((unsigned long) node_flag & FT_KIND_MASK) == FT_KIND_QP) {
		stats->qp_hi_half_cls_dist[metadata->qp_subtree_half_cls]++;
		stats->nr_qp_hi_total++;
	}
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

		child_node_flag = ft_node_get_nth_skip(node_flag, NULL, (uint8_t) key, FT_PF_NONE);
		if (!child_node_flag)
			continue;
#ifdef FEATURE_FT_SKIP_COMPRESSED
		if (ft_node_skip_compressed_in_slot(child_node_flag)) {
			/*
			 * Candidate E: bit 0 partitions by alignment.  Internal-
			 * aligned skips are 32-byte aligned with clean 5-bit
			 * kind; SKIP_EXT is 16-byte aligned and bit 4 may leak
			 * under a 5-bit mask but bit 0 is always clear so the
			 * conditional folds the leak into FT_KIND_SKIP_EXT.
			 */
			unsigned long v = (unsigned long) child_node_flag;
			unsigned long tag = (v & 0x01UL)
				? (v & 0x1FUL) : FT_KIND_SKIP_EXT;
			unsigned int skip = ft_skip_len(child_node_flag);

			stats->nr_skip_compressed_total++;
			if (skip < 256)
				stats->skip_compressed_len_dist[skip]++;
			if (tag == FT_KIND_SKIP_QP)
				stats->nr_skip_compressed_qp++;
			else if (tag == FT_KIND_SKIP_POPCOUNT_32)
				stats->nr_skip_compressed_popcount_32++;
			else if (tag == FT_KIND_SKIP_POPCOUNT_64)
				stats->nr_skip_compressed_popcount_64++;
			else if (tag == FT_KIND_SKIP_PIGEON)
				stats->nr_skip_compressed_pigeon++;
			else if (tag == FT_KIND_SKIP_EXT)
				stats->nr_skip_compressed_ext++;
			child_node_flag = ft_resolve_skip_compressed(child_node_flag);
		}
#endif
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
		} else if (ft_node_compressed_in_node(child_node_flag)) {
			struct cds_ft_compressed_node *cn =
				ft_compressed_node_ptr(child_node_flag);
			struct cds_ft_metadata *metadata =
				cds_ft_item_to_metadata((struct cds_ft_inode *) cn);
			struct cds_ft_node *external_nodes = rcu_dereference(metadata->external_nodes);
			int j;

			stats->compressed_len_dist[cn->len]++;
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
			    !ft_node_external_direct(cn->child))
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
		for (type = 0; type < FT_NUM_INTERNAL_TYPES; type++) {
			const struct cds_ft_node_stats *node_stats = &stats->level[level].node_stats[type];
			uint64_t nr_nodes = node_stats->count;
			const char *name;

			switch (type) {
			case FT_POPCOUNT_32_INDEX:	name = "POPCOUNT_32"; break;
			case FT_POPCOUNT_64_INDEX:	name = "POPCOUNT_64"; break;
			case FT_QP_INDEX:		name = "QP"; break;
			case FT_PIGEON_INDEX:		name = "PIGEON"; break;
			default:			name = "UNKNOWN"; break;
			}
			if (nr_nodes) {
				unsigned int i;
				bool first = true;

				print_indent(out, 2);
				fprintf(out, "Internal node type %lu (%s): %" PRIu64
					" (", type, name, nr_nodes);
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
		uint64_t cn_total = 0;
		uint64_t cumulative = 0;
		unsigned int n;

		for (n = 1; n < 256; n++)
			cn_total += stats->compressed_len_dist[n];
		if (cn_total) {
			fprintf(out,
				"Compressed nodes (trie-wide, distinct cn): %"
				PRIu64 "\n", cn_total);
			print_indent(out, 1);
			fprintf(out, "cn->len distribution:");
			for (n = 1; n < 256; n++) {
				if (stats->compressed_len_dist[n])
					fprintf(out, " [%u]=%" PRIu64,
						n,
						stats->compressed_len_dist[n]);
			}
			fprintf(out, "\n");
			print_indent(out, 1);
			fprintf(out, "cn->len <= K cumulative %%:");
			for (n = 1; n < 256; n++) {
				cumulative += stats->compressed_len_dist[n];
				if (n == 4 || n == 8 || n == 12 ||
				    n == 16 || n == 20 || n == 24 ||
				    n == 32 || n == 48 || n == 64 ||
				    n == 96 || n == 128) {
					fprintf(out, " <=%u: %.1f%%",
						n,
						100.0 * (double) cumulative
							/ (double) cn_total);
				}
			}
			fprintf(out, "\n");
		}
	}
	if (stats->nr_qp_hi_total) {
		uint64_t total = stats->nr_qp_hi_total;
		uint64_t cumulative = 0;
		unsigned int n;

		fprintf(out, "QP-hi nodes: %" PRIu64 "\n", total);
		print_indent(out, 1);
		fprintf(out, "qp_subtree_half_cls distribution:");
		for (n = 0; n < 256; n++) {
			if (stats->qp_hi_half_cls_dist[n])
				fprintf(out, " [%u]=%" PRIu64,
					n, stats->qp_hi_half_cls_dist[n]);
		}
		fprintf(out, "\n");
		print_indent(out, 1);
		fprintf(out, "qp_subtree_half_cls <= K cumulative %%:");
		for (n = 0; n < 256; n++) {
			cumulative += stats->qp_hi_half_cls_dist[n];
			if (n == 1 || n == 2 || n == 4 || n == 8 ||
			    n == 16 || n == 24 || n == 32 || n == 48 ||
			    n == 64) {
				fprintf(out, " <=%u: %.1f%%",
					n,
					100.0 * (double) cumulative
						/ (double) total);
			}
		}
		fprintf(out, "\n");
	}
	if (stats->nr_skip_compressed_total) {
		uint64_t total = stats->nr_skip_compressed_total;
		uint64_t cumulative = 0;
		unsigned int n;

		fprintf(out,
			"Skip-compressed pointers: %" PRIu64
			" (qp:%" PRIu64 " pigeon:%" PRIu64 " popcount_32:%" PRIu64
			" popcount_64:%" PRIu64 " ext:%" PRIu64 ")\n",
			total,
			stats->nr_skip_compressed_qp,
			stats->nr_skip_compressed_pigeon,
			stats->nr_skip_compressed_popcount_32,
			stats->nr_skip_compressed_popcount_64,
			stats->nr_skip_compressed_ext);
		print_indent(out, 1);
		fprintf(out, "skip_len distribution:");
		for (n = 1; n < 256; n++) {
			if (stats->skip_compressed_len_dist[n])
				fprintf(out, " [%u]=%" PRIu64,
					n, stats->skip_compressed_len_dist[n]);
		}
		fprintf(out, "\n");
		print_indent(out, 1);
		fprintf(out, "skip_len <= K cumulative %%:");
		for (n = 1; n < 256; n++) {
			cumulative += stats->skip_compressed_len_dist[n];
			if (n == 4 || n == 5 || n == 8 || n == 12 ||
			    n == 16 || n == 20 || n == 24 ||
			    n == 32 || n == 48 || n == 64 ||
			    n == 96 || n == 128) {
				fprintf(out, " <=%u: %.1f%%",
					n,
					100.0 * (double) cumulative
						/ (double) total);
			}
		}
		fprintf(out, "\n");
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
