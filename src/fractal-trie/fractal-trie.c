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
 *   <urcu/rcu-txn.h>        the atomic multi-pointer "flip" primitive
 *   <urcu/rcu-txn-list.h>   (concurrent MCAS transaction + coherent bidir
 *                           list) used to commit a set of edges at once.
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

#include "fractal-trie-internal.h"
#include "fractal-trie-trace.h"

#include "bitmap.h"

/*
 * The implementation is split into per-module units, #included below in
 * dependency order into this single translation unit -- so the compiler still
 * inlines across module boundaries (e.g. the descent into each lookup), exactly
 * as when this was one 25k-line file.  FRACTAL_TRIE_IMPL gates each unit against
 * stray standalone inclusion.
 */
#define FRACTAL_TRIE_IMPL
#include "ft-tables.h"
#include "ft-delay.h"
#include "ft-helpers.h"
#include "ft-txn-hlist.h"	/* duplicate-chain transactional hlist (unused until wired) */
#include "ft-trace-helpers.h"
#include "ft-lookup-node.h"
#include "ft-descent.h"
#include "ft-iter.h"
#include "ft-lookup.h"

/*
 * When the inequality descent is shared (FEATURE_INLINE_INEQUALITY_LOOKUP off)
 * it is itself a cold tier-2 path, so route its up/down scanners through the
 * same shared copies the write path uses -- one get_direction / get_minmax for
 * the whole library, not an extra inlined copy in ft_ineq_descend.  With the
 * flag on, the descent inlines them for speed.
 */
#ifndef FEATURE_INLINE_INEQUALITY_LOOKUP
#define ft_node_get_direction	ft_node_get_direction_shared
#define ft_node_get_minmax	ft_node_get_minmax_shared
#endif
#include "ft-inequality.h"
#ifndef FEATURE_INLINE_INEQUALITY_LOOKUP
#undef ft_node_get_direction
#undef ft_node_get_minmax
#endif

/*
 * Redirect the write path to the shared, non-inlined copies of the large
 * read-path helpers it would otherwise force-inline at every site: the node
 * scanners (ft-lookup-node.h) and the inequality descent (ft-inequality.h).  The
 * mutation modules call each once instead of duplicating it -- about -23% .text
 * together.  The read modules above keep the inlined originals.  The redirect
 * stays open through every module below -- the mutation path, compaction, and
 * the ft-show.h display renderers are all cold or write paths, so they all
 * share the out-of-line copies; nothing hot follows, so it is never closed.
 */
#define ft_node_get_nth           ft_node_get_nth_shared
#define ft_node_get_nth_skip      ft_node_get_nth_skip_shared
#define ft_node_get_nth_reanchor  ft_node_get_nth_reanchor_shared
#define ft_node_get_direction     ft_node_get_direction_shared
#define ft_node_get_minmax        ft_node_get_minmax_shared
#define cds_ft_lookup_inequality_impl cds_ft_lookup_inequality_impl_shared
#include "ft-mutation-helpers.h"
#include "ft-mutation-node.h"
#include "ft-cluster-build.h"
#include "ft-insert.h"
#include "ft-remove.h"
#include "ft-graft.h"
#include "ft-detach.h"
#include "ft-merge.h"
#include "ft-ordered-query.h"
#include "ft-lifecycle.h"
#include "ft-verify.h"
#include "ft-compact.h"
#include "ft-show.h"

/*
 * Public API back-end for cds_ft_node_next_rcu()'s proxy path (declared in
 * <urcu/fractal-trie.h>).  A duplicate successor whose raw "next" value carries
 * the engine proxy tag -- a bulk commit (e.g. a merge appending a src run at
 * this chain's tail) is mid-flight on the slot -- is resolved to the committed
 * successor, with the removal tombstone stripped.  Kept out of line so the
 * transactional engine (and its headers) stay opaque to API users; a
 * duplicate-chain walk is not a fast path, so the call is immaterial.
 */
urcu_static_assert(CDS_FT_NODE_TXN_PROXY_TAG == URCU_TXN_TAG,
		"FT public duplicate-next proxy tag must equal the engine proxy tag",
		ft_node_next_proxy_tag_matches_engine);
struct cds_ft_node *cds_ft_node_next_resolve(void *raw)
{
	return ft_hlist_resolve(raw);
}

#ifdef FEATURE_FT_MW_DLM_ACQUIRE
/*
 * TEST/DEBUG (coherent-rekey sub-step 2, NOT public API): read the trie root as
 * an opaque address, so a test can observe a COW relocation moved its identity.
 */
void *_cds_ft_debug_root(struct cds_ft *ft)
{
	return (void *) rcu_dereference(ft->root);
}

/*
 * TEST/DEBUG (coherent-rekey sub-step 2, NOT public API): drive ft_rekey_cow_stop
 * on the trie ROOT as a self-contained in-place clone -- build a fresh-address
 * copy of the root, re-parent its children onto it, retire the old root, and
 * republish at &ft->root, all as ONE mixed SW/MW commit.  This is the smallest
 * complete consumer of the S_top COW primitive: the root has no parent to lock
 * (its forward slot is auto-guarded by its own CAS), so it isolates the COW +
 * interior re-parent + retire from the rest of a rekey stitch.
 *
 * SINGLE-WRITER use only (no concurrent root relocation).  Returns 0 (root now at
 * a fresh address, contents unchanged), -EINVAL if the root is not an internal
 * POPCOUNT/PIGEON node without a co-located external list (sub-step-2 scope), or a
 * negative errno on a bail (not expected single-threaded).
 */
int _cds_ft_debug_cow_replace_root(struct cds_ft *ft)
{
	struct cds_ft_inode_flag *root = rcu_dereference(ft->root);
	struct cds_ft_inode_flag *root_prime;
	struct cds_ft_metadata *old_root_meta;
	struct ft_flip_txn *txn;
	struct cds_ft_metadata *marks[FT_ENTRY_PER_NODE + 1];
	uintptr_t snaps[FT_ENTRY_PER_NODE + 1];
	unsigned int nr_marks = 0, i, ti;
	enum urcu_txn_status st;
	int ret;

	if (!root || ft_node_flip_proxy(root))
		return -EINVAL;
	ti = ft_node_type(root);
	if (ft_types[ti].type_class != FT_POPCOUNT &&
			ft_types[ti].type_class != FT_PIGEON)
		return -EINVAL;
	old_root_meta = cds_ft_item_to_metadata(ft_node_ptr(root));
	if (old_root_meta->external_nodes)
		return -EINVAL;			/* sub-step-2 scope */

	txn = ft_flip_txn_create();
	if (!txn)
		return -ENOMEM;
	ft_flip_txn_set_structural_sw(txn, true);

	ret = ft_rekey_cow_stop(ft, txn, root, &root_prime, marks, snaps,
			&nr_marks);
	if (ret) {
		ft_flip_txn_destroy(txn);	/* pre-commit bail: destroy caller-owned txn */
		goto sweep;
	}

	/* Forward publish ft->root: root -> root' (SW; root slot self-guarded). */
	if (!ft_flip_txn_reserve_extra(txn, 1)) {
		free_cds_ft_node_unpublished(ft, ft_node_ptr(root_prime));
		ft_flip_txn_destroy(txn);
		ret = -ENOMEM;
		goto sweep;
	}
	ft_flip_txn_record_reserved(txn, (void **) &ft->root, root, root_prime);

	st = ft_flip_txn_commit(ft, txn);		/* consumes txn */
	if (st == URCU_TXN_STATUS_OK) {
		cds_ft_free_item_deferred(ft, old_root_meta);	/* old root freed after GP */
		ret = 0;
	} else {
		/*
		 * Abort (a peer won a raced slot): the SW parks never flipped, so
		 * root_prime stayed unpublished -- reclaim it (its shared interior
		 * children are untouched, still owned by the live old root).
		 * Unreachable under the single-writer, all-SW, pre-reserved contract
		 * here (the commit is deterministically OK), but kept leak-free for a
		 * future concurrent reuse of this path.
		 */
		free_cds_ft_node_unpublished(ft, ft_node_ptr(root_prime));
		ret = -EAGAIN;
	}

sweep:
	/*
	 * Clear any COPYING mark a successful commit did not consume (a no-op then,
	 * a release on every mark on a bail/abort) -- the caller-owned sweep the
	 * primitive's contract requires, since the marks are not txn-registered.
	 */
	for (i = 0; i < nr_marks; i++)
		ft_meta_copying_clear_if_held(marks[i]);
	return ret;
}

/*
 * TEST/DEBUG (coherent-rekey sub-step 3, NOT public API): descend @key (converted
 * to ordinal) and return the node flag AT that key as an opaque address, so a test
 * can observe that a rekey moved the moved-subtree top (S_top) to a FRESH address.
 * NULL if the key is absent or its path traverses a non-plain-internal node.
 */
void *_cds_ft_debug_child_at(struct cds_ft *ft, const uint8_t *key,
		size_t key_len)
{
	uint8_t ord[FT_MAX_KEY_LEN];
	struct ft_descent d;
	const uint8_t *ik = ord;

	if (key_len == 0 || key_len > FT_MAX_KEY_LEN)
		return NULL;
	ft_key_to_ordinals(ord, key, key_len, &ft->group->key_map);
	ft_descent_init(&d, ft);
	while (d.depth < key_len) {
		if (!d.nf || ft_node_external(d.nf) || ft_node_compressed(d.nf))
			return NULL;
#ifdef FEATURE_FT_SKIP_COMPRESSED
		if (ft_node_skip_compressed(d.nf))
			return NULL;
#endif
		ft_descent_step(ft, &d, *(ik++));
	}
	return (void *) d.nf;
}

/*
 * Does the located ordered-list splice pair (@pred, @succ) really BRACKET the dst
 * key range in key order?  ft_ord_cell_find_splice_pos derives the pair from a
 * RELATIONAL (inequality) descent, and relational reads are NOT coherent under
 * concurrent structural mutation -- a peer rekey in flight can make that descent
 * return a pair that is genuinely ADJACENT in the list but sits at the WRONG key
 * position.  Every edge of the splice then validates (the pair IS adjacent) and
 * the commit installs the run in the wrong place: an ordered list that is a
 * well-formed doubly-linked list yet no longer key-ordered.
 *
 * So re-derive each neighbour's key from the STRUCTURE (the same up-walk the
 * rekey-coherent reader uses, ordinal space, right-aligned in @scratch) and
 * require pred < the dst range < succ.  Fixed-length groups only (the hook's
 * scope): the dst key is a PREFIX, so the range is @dst_ord padded with the
 * ordinal extremes.
 *
 * A sentinel / absent neighbour has no key, but it is NOT exempt: it ASSERTS that
 * the run belongs at the list end, so the assertion itself is what gets checked --
 * an absent pred means succ must really be the list MINIMUM, an absent succ means
 * pred must really be the list MAXIMUM.  Skipping that (a first cut did) leaves the
 * incoherent derivation a way through: a spurious "no key below the dst range"
 * (find_splice_pos returns pred == NULL and then probes GT) paired with a succ that
 * legitimately sorts above the range passes a key-only check, and the pair reaches
 * the endpoint-adjacency guard as a bogus PERMANENT -EINVAL.  That was the last
 * residual failure of the shared-junction oracle (~1 run in 70).
 */
static
bool ft_rekey_splice_pos_brackets(struct cds_ft *ft, const uint8_t *dst_ord,
		size_t dst_len, struct ft_ord_cell *pred,
		struct ft_ord_cell *succ)
{
	size_t flen = ft->group->key_len, max_len = ft->group->max_key_len;
	uint8_t lo[FT_MAX_KEY_LEN], hi[FT_MAX_KEY_LEN];
	uint8_t scratch[FT_MAX_KEY_LEN];
	struct ft_ord_cell *sentinel = ft_ord_sentinel_cell(ft);
	bool pred_end = !pred || pred == sentinel;
	bool succ_end = !succ || succ == sentinel;
	size_t klen;

	if (flen == CDS_FT_LEN_VARIABLE || dst_len > flen)
		return true;			/* out of this hook's scope */
	if (pred_end && succ_end)
		return false;			/* "empty list" -- the run is IN it */
	if (pred_end && ft_ord_first(ft) != succ)
		return false;			/* head insert, but succ is not the min */
	if (succ_end && ft_ord_last(ft) != pred)
		return false;			/* tail insert, but pred is not the max */
	memcpy(lo, dst_ord, dst_len);
	memset(lo + dst_len, 0x00, flen - dst_len);
	memcpy(hi, dst_ord, dst_len);
	memset(hi + dst_len, 0xff, flen - dst_len);
	if (pred && pred != sentinel) {
		klen = ft_rebuild_key_upwalk(ft, pred, scratch, max_len);
		if (klen != flen || memcmp(scratch + (max_len - klen), lo,
				flen) >= 0)
			return false;
	}
	if (succ && succ != sentinel) {
		klen = ft_rebuild_key_upwalk(ft, succ, scratch, max_len);
		if (klen != flen || memcmp(scratch + (max_len - klen), hi,
				flen) <= 0)
			return false;
	}
	return true;
}

/*
 * TEST/DEBUG (coherent-rekey sub-step 3, NOT public API): the SIMPLEST complete
 * one-decide rekey-graft -- move the subtree "S_top" hanging at @src_key to the
 * ABSENT @dst_key, as ONE mixed SW/MW flip-txn: COW S_top to a fresh S_top' (SW
 * re-parents + retire, locked), graft S_top' at the dst point (MW forward publish,
 * dst parent unlocked), and clear the src slot (MW detach, unlocked) -- all
 * recorded into one txn and committed once, so a reader sees the subtree at src
 * XOR dst atomically.  This is the analog of _cds_ft_debug_cow_replace_root for
 * the full stitch, scoped to the simplest shape so it needs no cell / GLUE / merge
 * machinery:
 *   - S_top is a plain internal POPCOUNT/PIGEON node with no co-located external
 *     list (ft_rekey_cow_stop's sub-step-2 scope), hanging at @src_key.
 *   - the SRC JUNCTION BP (= S_top's parent) and the DST PARENT SHARE A PARENT
 *     (d_src.ppnf == d_dst.ppnf) -- the enforced precondition of the held-parent
 *     reuse (see the shape gate below).  In the default, concurrent-safe build
 *     EVERY popcount delete recompacts, so BP is rebuilt on the removal and
 *     republished into that shared parent; the graft's dst-parent recompaction
 *     COPYING-holds that same shared parent FIRST, so the detach's BP recompaction
 *     REUSES the held lock (@src_held_hint) instead of re-acquiring it.
 *   - @dst_key is ABSENT and reached by a NOSPLIT graft into a spare slot with
 *     append room (no compressed-divergence GLUE split).
 *   - the trie's ordered list may be ON: the moved subtree's contiguous cell run
 *     unsplices from the src ordered position and re-splices at the dst position
 *     as SIX recorded cell edges in the SAME commit (all MW, so a peer's cell
 *     conflict aborts clean).  The dst splice position must be clear of the run's
 *     own ordered neighbourhood (adjacency guard below).
 * structural_sw STAYS TRUE the whole txn: cow_stop's S_top edges, the graft's
 * dst-parent recompaction, and the detach's BP recompaction all hold their DLM
 * COPYING locks (BP's shared parent via @src_held_hint), so every structural edge
 * is legitimately SW; the mixed engine still sorts the (MW) count edges first.
 *
 * @src_key / @dst_key are APPLICATION keys (converted to ordinal internally); for
 * a fixed-length group @src_len must equal @dst_len.
 *
 * CONCURRENCY.  Abort-clean and single-shot: every bail leaves the trie byte-for-
 * byte as before, so a CONCURRENT caller retries by simply calling again (there is
 * no internal retry loop).  Returns 0; -EINVAL when the controlled shape above is
 * not met; or a TRANSIENT contention code to retry on -- -EIO (the graft's up-front
 * acquire lost a race), -EAGAIN (a cow_stop / detach acquire, an incoherently
 * derived splice position, or the final commit aborted), -ENOMEM (reserve).  The
 * concurrent oracles (inv_rekey_graft_{disjoint,shared,coherent_readers}) exercise
 * exactly that contract.  What is NOT yet multi-writer safe in general: a dst
 * splice neighbour INTERIOR to a peer's concurrently-moving run (see the splice
 * position validation below) -- out of reach in the tested layouts, and closed
 * only by the per-FT move seqcount / relational coherence work.
 */
int _cds_ft_debug_rekey_graft_simple(struct cds_ft *ft,
		const uint8_t *src_key, size_t src_len,
		const uint8_t *dst_key, size_t dst_len)
{
	uint8_t src_ord[FT_MAX_KEY_LEN], dst_ord[FT_MAX_KEY_LEN];
	struct cds_ft_inode_flag *s_top, *s_top_prime = NULL, *attached_nf = NULL;
	struct cds_ft_node *run_rfirst = NULL, *run_rlast = NULL;
	struct ft_ord_cell *run_dpred = NULL, *run_dsucc = NULL;
	struct cds_ft_metadata *s_top_meta, *bp_meta;
	struct ft_detach_recompact_out detach_rc = { 0 };
	struct ft_flip_txn *txn;
	struct ft_glue glue;
	struct ft_graft_store_state gst_st;
	struct ft_descent d_src, d_dst;
	struct cds_ft_alloc_reserve reserve;
	struct ft_remove_pub pub = { .armed = false };
	struct cds_ft_metadata *marks[FT_ENTRY_PER_NODE + 1];
	uintptr_t snaps[FT_ENTRY_PER_NODE + 1];
	const uint8_t *ik;
	unsigned long move_seq;
	unsigned int nr_marks = 0, adepth = 0, i, ti;
	enum ft_graft_prep prep;
	enum cds_ft_status gst;
	enum urcu_txn_status gcst = URCU_TXN_STATUS_OK, st;
	unsigned long cnt;
	int ret;

	if (!ft->lock_fine || src_len == 0 || dst_len == 0 ||
			src_len > FT_MAX_KEY_LEN || dst_len > FT_MAX_KEY_LEN)
		return -EINVAL;
	ft_key_to_ordinals(src_ord, src_key, src_len, &ft->group->key_map);
	ft_key_to_ordinals(dst_ord, dst_key, dst_len, &ft->group->key_map);

	/*
	 * Descend src to S_top through plain internal nodes, capturing (nfp, pnfp,
	 * depth) so ft_detach_node can bootstrap the src-slot clear + BP nr_child--.
	 */
	ft_descent_init(&d_src, ft);
	ik = src_ord;
	while (d_src.depth < src_len) {
		if (!d_src.nf || ft_node_external(d_src.nf) ||
				ft_node_compressed(d_src.nf))
			return -EINVAL;
#ifdef FEATURE_FT_SKIP_COMPRESSED
		if (ft_node_skip_compressed(d_src.nf))
			return -EINVAL;
#endif
		ft_descent_step(ft, &d_src, *(ik++));
	}
	s_top = d_src.nf;
	if (!s_top || d_src.depth != src_len || ft_node_flip_proxy(s_top) ||
			ft_node_external(s_top) || ft_node_compressed(s_top))
		return -EINVAL;
#ifdef FEATURE_FT_SKIP_COMPRESSED
	if (ft_node_skip_compressed(s_top))
		return -EINVAL;
#endif
	ti = ft_node_type(s_top);
	if (ft_types[ti].type_class != FT_POPCOUNT &&
			ft_types[ti].type_class != FT_PIGEON)
		return -EINVAL;
	s_top_meta = cds_ft_item_to_metadata(ft_node_ptr(s_top));
	if (s_top_meta->external_nodes)
		return -EINVAL;			/* cow_stop sub-step-2 scope */

	/* BP (= S_top's parent) must be plain and stay above min_child on removal. */
	if (!d_src.pnf || ft_node_flip_proxy(d_src.pnf) ||
			ft_node_external(d_src.pnf) || ft_node_compressed(d_src.pnf))
		return -EINVAL;
	bp_meta = cds_ft_item_to_metadata(ft_node_ptr(d_src.pnf));
	/*
	 * PROXY-SAFE count read: the raw ft_meta_nr_child() would decode a peer's
	 * parked FT_STATE_PROXY as a garbage child count and reject a perfectly good
	 * shape as PERMANENTLY invalid (measured: 10-12 spurious -EINVAL per stress
	 * run).  ft_meta_nr_child_load resolves the proxy to the committed value.
	 */
	if (ft_meta_nr_child_load(bp_meta) < 3)
		return -EINVAL;

	/*
	 * A POPULATED dst is a PERMANENT shape error, so reject it HERE.  The later shape
	 * gate does check d_dst.nf, but only after ft_graft_build -- i.e. after the splice
	 * position validation below, whose failure code is the TRANSIENT -EAGAIN.  A key
	 * inside the dst range makes that validation's succ check fail first, so a caller
	 * that (correctly) retries -EAGAIN would spin forever on a shape that can never
	 * work.  Probe read-only and conservatively: a descent that cannot reach the dst
	 * depth plainly proves nothing, so leave those shapes to the later gate.
	 */
	{
		struct ft_descent d_probe;
		const uint8_t *pk = dst_ord;

		ft_descent_init(&d_probe, ft);
		while (d_probe.depth < dst_len && d_probe.nf &&
				!ft_node_external(d_probe.nf) &&
				!ft_node_compressed(d_probe.nf)
#ifdef FEATURE_FT_SKIP_COMPRESSED
				&& !ft_node_skip_compressed(d_probe.nf)
#endif
		      )
			ft_descent_step(ft, &d_probe, *(pk++));
		if (d_probe.depth == dst_len && d_probe.nf)
			return -EINVAL;		/* dst occupied: permanent, not a race */
	}

	/*
	 * PIN the whole decision against concurrent moves: sample the per-FT move
	 * counter HERE, before anything is derived from a read that is not coherence-
	 * hardened (the ordered-list splice position below comes from a RELATIONAL
	 * descent), and record its bump into the commit at the end.  Any peer move that
	 * commits inside that window makes this commit's CAS on the counter mismatch, so
	 * this op aborts and re-derives instead of committing a decision that went stale.
	 * See struct cds_ft::move_seq: for the splice position this is the ONLY sound
	 * mechanism -- a read-only validator is built from the same incoherent reads and
	 * can be fooled the same way (measured: mis-ordered splices still committed with
	 * the bracket check alone).  The bracket check below stays as a cheap early
	 * reject and as defence in depth.
	 */
	move_seq = ft_move_seq_load(ft);

	/*
	 * List on: capture the moved subtree's contiguous ordered-cell run endpoints
	 * (the structural min/max external heads under S_top) from the still-pristine
	 * list, so the ONE commit can unsplice the run from the src ordered position
	 * and re-splice it at the dst position (six MW boundary edges) atomically with
	 * the structural move -- a coherent reader never sees a moved key gone from the
	 * structure but still in the list (or vice versa).  cow_stop SHARES S_top's
	 * leaves (only S_top's own node relocates), so these heads stay valid across it.
	 *
	 * Locate the dst splice neighbours NOW, on the pristine list (find_splice_pos
	 * needs the dst attach point empty, which it still is -- nothing is published
	 * until the final commit), and REJECT an adjacency shape up front (before any
	 * txn / COPYING mark / record, so the bail is a clean no-op -EINVAL): because
	 * find_splice_pos runs while the run is STILL at src, a dst gap that abuts the
	 * run resolves the run's own ENDPOINT cell as a splice neighbour (dsucc == run
	 * first, or dpred == run last), which would record a duplicate-slot MW edge and
	 * plain-store a self-cyclic run link.  This hook only supports a dst position
	 * clear of the run's current ordered neighbourhood; a general rekey would locate
	 * the splice against the run-removed list instead.
	 *
	 * Completeness of the "no run cell is a splice neighbour" guarantee is JOINT:
	 * this guard rejects the two ENDPOINT-adjacency shapes, while the INTERIOR case
	 * (a dst gap whose neighbour is a run cell strictly between rfc and rlc) is
	 * excluded by the later d_src.ppnf == d_dst.ppnf shape gate -- ppnf equality
	 * forces src_len == dst_len, so an interior dst (which needs the full src prefix
	 * plus a longer key descending into S_top) never reaches the cell record.  A
	 * future relaxation of that shape gate MUST re-add an interior check here.
	 *
	 * VALIDATE the located pair FIRST, and bail -EAGAIN (transient, re-derive) when
	 * it does not bracket the dst key range: find_splice_pos derives the pair from a
	 * RELATIONAL descent, which is NOT coherent under concurrent structural
	 * mutation, so a peer rekey in flight can return a pair that is adjacent in the
	 * list but sits at the WRONG key position.  ADJACENT + BRACKETING is the full
	 * correctness condition for a splice, and only adjacency was checked: the
	 * splice's own boundary edge (pred->next expect succ) validates adjacency at
	 * commit, and ft_rekey_splice_pos_brackets validates the other half here.  The
	 * bracket check must precede the endpoint-adjacency check so that a racy pair
	 * involving a run endpoint surfaces as the transient -EAGAIN it is rather than
	 * the permanent -EINVAL of the genuine (correctly-derived, abutting) shape --
	 * a racy pred == rlc drags succ = resolve(rlc->next) along, which then sorts
	 * BELOW the dst range and fails the bracket.
	 *
	 * WHAT IS STILL OPEN (general rekey, not this hook's tested shapes): a pair that
	 * brackets and is adjacent when validated can still be invalidated afterwards if
	 * a neighbour is INTERIOR to a peer's moving run -- the peer's move leaves an
	 * interior cell's own links untouched, so no edge of this commit detects it, and
	 * the run lands inside the peer's run.  Every neighbour reachable in the tested
	 * layouts is either a never-moving key or a peer run's ENDPOINT (whose outer link
	 * IS one of these edges, hence detected).  Closing it in general needs the
	 * per-FT move seqcount / relational coherence (doc: in-trie-move-seqcount.md),
	 * which would also let a rekey validate the derivation itself.
	 */
	if (ft->ordered_list) {
		struct ft_ord_cell *rfc, *rlc;

		run_rfirst = ft_subtree_minmax_head(ft, s_top, false);
		run_rlast = ft_subtree_minmax_head(ft, s_top, true);
		rfc = ft_ord_cell_ptr(rcu_dereference(run_rfirst->prev));
		rlc = ft_ord_cell_ptr(rcu_dereference(run_rlast->prev));
		ft_ord_cell_find_splice_pos(ft, dst_key, dst_len, &run_dpred,
				&run_dsucc);
		if (!ft_rekey_splice_pos_brackets(ft, dst_ord, dst_len, run_dpred,
				run_dsucc))
			return -EAGAIN;		/* incoherent relational derivation */
		if (run_dsucc == rfc || run_dpred == rlc)
			return -EINVAL;
	}

	cnt = ft_nr_keys_get(s_top_meta);	/* subtree key count (count edges no-op if rank off) */

	txn = ft_flip_txn_create();
	if (!txn)
		return -ENOMEM;

	/* 1. COW S_top -> S_top' (SW; records re-parents + retire, LOCKED). */
	ft_flip_txn_set_structural_sw(txn, true);
	ret = ft_rekey_cow_stop(ft, txn, s_top, &s_top_prime, marks, snaps,
			&nr_marks);
	if (ret) {
		ft_flip_txn_destroy(txn);	/* pre-commit bail: destroy caller-owned */
		goto sweep;
	}

	/*
	 * 2. + 3.  structural_sw STAYS TRUE for the rest: the graft ALWAYS relocates
	 * the dst attach node (a reserve recompaction), which ACQUIRES COPYING locks
	 * over the dst parent + the republish grandparent and records its re-parents /
	 * release / retire as SW under those locks -- so the graft forward publish and
	 * recompact edges are correctly SW.  The detach's src-junction (BP) edges are
	 * UNLOCKED, but ft_ord_cell_record_into forces them MW regardless of
	 * structural_sw, so no toggle is needed (toggling OFF would wrongly demote the
	 * recompact's COPYING-expecting edges to MW -> expected-old mismatch -> abort).
	 * The mixed commit installs the MW (detach) edges first, then parks the SW
	 * (graft + cow_stop) edges before the flip.
	 */

	/* 2. Graft-fold: record-only NOSPLIT attach of S_top' at @dst_key. */
	ft_glue_init(&glue);
	glue.txn = txn;
	glue.record_only = true;
	memset(&reserve, 0, sizeof(reserve));
	if (ft_bulk_node_reserve_fill(ft, &reserve)) {
		ft_flip_txn_destroy(txn);
		ret = -ENOMEM;
		goto sweep;
	}
	prep = ft_graft_build(ft, dst_ord, dst_len, s_top_prime, cnt, &d_dst, &glue);
	/*
	 * SHAPE GATE.  This first cut supports only:
	 *  - an absent, append-in-place NOSPLIT dst point (no compressed-divergence
	 *    GLUE split), and
	 *  - the src junction BP (= d_src.pnf) and the dst parent (= d_dst.pnf)
	 *    sharing the SAME parent (d_src.ppnf == d_dst.ppnf, both non-NULL).  That
	 *    shared parent is what the graft's dst-parent recompaction COPYING-holds
	 *    and what the detach's BP recompaction reuses via src_parent_held below;
	 *    the reuse is UNSOUND if BP's parent is not the graft-held node.  A shape
	 *    where the two junctions diverge is rejected here (the general rekey needs
	 *    a verified / fallback acquire, not this hook's unconditional reuse).
	 */
	if (prep != FT_GRAFT_PREP_NOSPLIT || d_dst.depth != dst_len || d_dst.nf ||
			!d_src.ppnf || d_src.ppnf != d_dst.ppnf) {
		cds_ft_alloc_reserve_drain(ft, &reserve);
		ft_glue_abort(ft, &glue);
		free_cds_ft_node_unpublished(ft, ft_node_ptr(s_top_prime));
		ft_flip_txn_destroy(txn);
		ret = -EINVAL;
		goto sweep;
	}
	/*
	 * Reserve the graft slot edge + the detach struct/state edges, plus (list on)
	 * the six ordered-cell run boundary edges (src unsplice + dst splice).
	 */
	if (!ft_flip_txn_reserve_extra(txn, FT_REMOVE_COMMIT_REC_MAX_EDGES + 4 +
			1 /* the move-counter pin edge (3c) */ +
			(ft->ordered_list ? FT_ORD_CELL_RUN_DETACH_MAX_EDGES +
				FT_ORD_CELL_RUN_RESPLICE_MAX_EDGES : 0))) {
		cds_ft_alloc_reserve_drain(ft, &reserve);
		ft_glue_abort(ft, &glue);
		free_cds_ft_node_unpublished(ft, ft_node_ptr(s_top_prime));
		ft_flip_txn_destroy(txn);
		ret = -ENOMEM;
		goto sweep;
	}
	/*
	 * Drive prepare + commit SEPARATELY (not the combined ft_store_at_graft_point
	 * wrapper) so the reserve recompaction's relocated old dst-parent copy
	 * (@gst_st.old_recompacted_node) is visible here: the graft ALWAYS relocates
	 * the attach node for its atomic publish, and under record_only its old copy
	 * stays LIVE until the caller's commit, so the caller frees it post-commit.
	 */
	cds_ft_alloc_reserve_activate(ft, &reserve);
	gst = ft_store_at_graft_point_prepare(ft, dst_ord, dst_len, &d_dst,
			s_top_prime, cnt, &glue, &gst_st);
	if (gst == CDS_FT_STATUS_OK)
		gcst = ft_store_at_graft_point_commit(ft, &attached_nf, &adepth,
				NULL /*run*/, &gst_st, (long) cnt);
	cds_ft_alloc_reserve_deactivate(ft);
	cds_ft_alloc_reserve_drain(ft, &reserve);
	if (gst != CDS_FT_STATUS_OK || gcst != URCU_TXN_STATUS_OK) {
		/*
		 * Not expected single-threaded with the reserve pre-filled.  Record-only
		 * commit leaves the shared txn intact (terminal commit gated off), so the
		 * caller owns cleanup: free S_top', abort the glue build, destroy the txn.
		 * (prepare failure freed its own invisible build + left glue clean.)
		 */
		free_cds_ft_node_unpublished(ft, ft_node_ptr(s_top_prime));
		ft_glue_abort(ft, &glue);
		ft_flip_txn_destroy(txn);
		ret = -EIO;
		goto sweep;
	}

	/*
	 * 3. Detach-fold: remove S_top from BP (clear its slot + nr_child--).  In the
	 * default (concurrent-safe) build EVERY popcount delete recompacts BP, and
	 * that recompaction republishes into BP's parent -- which is the SAME shared
	 * spine ancestor the graft's dst-parent recompaction already COPYING-holds
	 * (both BP and the dst parent are children of it in this depth-2 shape, the
	 * d_src.ppnf == d_dst.ppnf gate).  Hand the detach that HELD NODE'S IDENTITY
	 * (@src_held_hint) so its recompaction REUSES the held lock instead of
	 * re-acquiring it (a second acquire would abort -EAGAIN).
	 *
	 * The identity is passed EXPLICITLY, and its slot with it, rather than letting
	 * the recompaction resolve BP's current parent: that resolve is racy, and a
	 * peer that re-homed BP since this descent would make "BP's parent" a node
	 * this op does NOT hold -- an SW park into an unheld word.  The recompaction's
	 * acquire commit carries a BP.parent == @parent read-set guard, so a re-home
	 * ABORTS it (-EAGAIN, trie pristine) and the caller re-descends.
	 */
	ret = ft_detach_node(ft, d_src.nfp, d_src.pnfp, d_src.depth,
			false /*free_detached_subtree: S_top is retired by cow_stop*/,
			NULL /*fuse_cell: list off*/, &pub, NULL /*run*/,
			NULL /*retire_glue*/, NULL /*freeze_leaf*/,
			-(long) cnt, txn /*shared_txn*/, true /*record_only*/,
			&(const struct ft_parent_hint){	/* graft-held shared parent */
				.parent = d_src.ppnf, .slot = d_src.pnfp,
				.gp = NULL, .gp_slot = NULL,
				.parent_held = true },
			&detach_rc /*old + fresh BP copies, reclaimed post-commit*/);
	if (ret) {
		/*
		 * Pre-commit bail.  Reclaim EVERY unpublished fresh copy built so far --
		 * S_top' AND the graft's relocated dst-parent copy (@gst_st.dest, whose
		 * old counterpart is @gst_st.old_recompacted_node): the graft always
		 * relocates the attach node, and under record_only nothing it built is
		 * published until the caller's commit, so this arm owns the fresh copy
		 * exactly as the commit-abort arm below does.  (@glue's own build is
		 * covered by ft_glue_abort; nr_built is 0 for the NOSPLIT shape.)
		 * REACHABLE single-threaded, deterministically: a same-junction rekey
		 * (src and dst under the SAME junction, which every shape gate admits)
		 * makes BP the graft's own attach node, so the graft COPYING-locks BP and
		 * the fold's ft_dlm_lock(BP) then returns -EAGAIN right here.
		 */
		free_cds_ft_node_unpublished(ft, ft_node_ptr(s_top_prime));
		if (gst_st.old_recompacted_node)
			free_cds_ft_node_unpublished(ft, ft_node_ptr(gst_st.dest));
		ft_glue_abort(ft, &glue);
		ft_flip_txn_destroy(txn);
		goto sweep;
	}

	/*
	 * 3b. Cell-fold (list on): record the six ordered-cell run boundary edges into
	 * the SHARED txn so the run unsplices from src + re-splices at dst ATOMICALLY
	 * with the structural move.  Driver-managed (run == NULL to the structural folds
	 * above) rather than threaded through them, because the dst-splice PLAIN-STORES
	 * the run's outer links (rfc->prev, rlc->next) at record time, so the src unsplice
	 * -- which READS those links to find the src neighbours -- must record FIRST.  The
	 * structural folds' order (the graft acquires the shared parent lock before the
	 * detach reuses it via src_parent_held) can't provide that, so the cells are
	 * recorded here, in the required order, on the still-pristine live list (no
	 * structural fold above published anything).
	 *
	 * All SIX edges ride the ONE commit (plan Q3): the src unsplice's two neighbour
	 * back-edges, the dst splice's two neighbour back-edges, AND -- via the same-trie
	 * ft_ord_cell_run_resplice_edges instead of the cross-trie
	 * ft_ord_cell_run_splice_edges -- the run's own two OUTER links, which the
	 * cross-trie form would PLAIN-STORE.  A plain store is wrong for a LIVE run: it
	 * publishes the dst neighbours before the boundary flip and survives an abort,
	 * permanently breaking the back-edges.  Cell edges are always MW, so a peer's
	 * conflicting splice aborts this commit clean and the caller re-descends.
	 */
	if (ft->ordered_list) {
		struct ft_ord_cell_edge cedges[FT_ORD_CELL_RUN_DETACH_MAX_EDGES +
			FT_ORD_CELL_RUN_RESPLICE_MAX_EDGES];
		struct ft_ord_cell *rfc, *rlc, *src_pred, *src_succ;
		unsigned int cn;

		/*
		 * DISTINCT-SLOT precondition (the engine's, rcu-txn-mcas.h: a txn's records
		 * must target pairwise-distinct slots).  Two of the six edges coincide iff a
		 * dst neighbour IS a src neighbour: dst_pred == src_pred puts two records on
		 * &src_pred->lnode.next, dst_succ == src_succ two on &src_succ->lnode.prev.
		 * The endpoint-adjacency guard does NOT exclude that, because the dst pair was
		 * derived earlier than these src reads and a peer move can have shifted the
		 * neighbourhood in between -- MEASURED at 1.5-2% of commits before the move
		 * counter pinned the window shut.  ft_flip_txn_create's expect_conflict makes
		 * the engine RECONCILE rather than corrupt (it poisons -> clean abort), but
		 * relying on that is relying on a fallback: bail explicitly instead, so the
		 * distinctness the engine requires holds BY CONSTRUCTION.  Clean -EAGAIN: the
		 * txn has recorded structural edges but published nothing, so the bail below
		 * unwinds exactly like the other pre-commit bails.
		 */
		rfc = ft_ord_cell_ptr(rcu_dereference(run_rfirst->prev));
		rlc = ft_ord_cell_ptr(rcu_dereference(run_rlast->prev));
		src_pred = ft_ord_cell_resolve_ord(&rfc->lnode.prev);
		src_succ = ft_ord_cell_resolve_ord(&rlc->lnode.next);
		if (src_pred == ft_ord_or_sentinel(ft, run_dpred) ||
				src_succ == ft_ord_or_sentinel(ft, run_dsucc)) {
			free_cds_ft_node_unpublished(ft, ft_node_ptr(s_top_prime));
			if (gst_st.old_recompacted_node)
				free_cds_ft_node_unpublished(ft,
					ft_node_ptr(gst_st.dest));
			if (detach_rc.new_flag)
				free_cds_ft_node_unpublished(ft,
					ft_node_ptr(detach_rc.new_flag));
			ft_glue_abort(ft, &glue);
			ft_flip_txn_destroy(txn);
			ret = -EAGAIN;
			goto sweep;
		}

		/* src unsplice FIRST: reads the run's pristine outer links -> src neighbours. */
		cn = ft_ord_cell_run_detach_edges(ft, run_rfirst, run_rlast,
				&rfc, &rlc, cedges, 0);
		/*
		 * dst splice into the gap located up front on the pristine list.  Records the
		 * run's outer links rather than storing them, so the whole move is atomic and
		 * abort-clean; all six slots are distinct per the check above.
		 */
		cn = ft_ord_cell_run_resplice_edges(ft, rfc, rlc, run_dpred,
				run_dsucc, cedges, cn);
		ft_ord_cell_record_into(txn, cedges, cn);
	}

	/*
	 * 3c. PIN: bump the per-FT move counter from the value sampled before any
	 * derivation.  A peer move that committed since then makes this MW edge's CAS
	 * mismatch, so the whole commit backs out clean (before any SW park) and the
	 * caller re-descends -- the coherence the relational splice-position derivation
	 * cannot get from reads.  Reserved with the cell edges below.
	 */
	ft_flip_txn_record_move_seq(ft, txn, move_seq);

	/* 4. ONE commit of the whole stitch (consumes txn). */
	st = ft_flip_txn_commit(ft, txn);
	if (st == URCU_TXN_STATUS_OK) {
		ft_glue_free_old(ft, &glue);		/* graft old copies */
		/*
		 * The reserve recompaction's relocated old dst-parent copy: its retire
		 * committed with the flip (deferred past readers via the recompact's
		 * fenced tombstone), so reclaim it now -- mirrors ft_store_at_graft_
		 * point_commit's own post-commit free (ft-graft.h).
		 */
		if (gst_st.old_recompacted_node)
			free_cds_ft_node(ft, gst_st.old_recompacted_node);
		if (detach_rc.old_node)
			free_cds_ft_node(ft, detach_rc.old_node);	/* old BP copy */
		cds_ft_free_item_deferred(ft, s_top_meta);	/* old S_top after GP */
		ret = 0;
	} else {
		/*
		 * Abort (a peer won a raced MW slot): NOTHING published, so reclaim every
		 * UNPUBLISHED fresh copy -- S_top', the graft's relocated dst-parent copy
		 * (@gst_st.dest), and the detach's relocated BP copy (@detach_rc.new_flag)
		 * -- and abort the glue build.  The retired old copies stay LIVE (their
		 * tombstones rolled back), so they are NOT freed here.  Unreachable under
		 * the single-writer contract, kept leak-free for future concurrent use.
		 */
		free_cds_ft_node_unpublished(ft, ft_node_ptr(s_top_prime));
		if (gst_st.old_recompacted_node)
			free_cds_ft_node_unpublished(ft, ft_node_ptr(gst_st.dest));
		if (detach_rc.new_flag)
			free_cds_ft_node_unpublished(ft, ft_node_ptr(detach_rc.new_flag));
		ft_glue_abort(ft, &glue);
		ret = -EAGAIN;
	}
	ft_glue_fini(&glue);

sweep:
	for (i = 0; i < nr_marks; i++)
		ft_meta_copying_clear_if_held(marks[i]);
	return ret;
}
#endif /* FEATURE_FT_MW_DLM_ACQUIRE */
