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
#endif /* FEATURE_FT_MW_DLM_ACQUIRE */
