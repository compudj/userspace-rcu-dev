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

/*
 * ☠ BEFORE fractal-trie-internal.h, which pulls <urcu/rcu-txn.h>: this header
 * DEFINES the engine's abort-attribution hooks, and the engine's static inlines
 * are compiled where that header is parsed.  Included later it would expand to
 * the engine's own inert defaults and the instrument would build clean and
 * count nothing.
 */
#include "ft-txn-rec-dbg.h"
#include "fractal-trie-internal.h"

#ifdef FEATURE_FT_PROBE_REANCHOR
/*
 * Reanchor level-move coverage.  A descent raises skip_conflict only when a
 * reanchor lands SHALLOWER (rewind != 0) because a peer chain-merge moved the
 * encoded position -- the condition graft and insert bail on and ft-merge.h
 * does not.  Measured over the suite the rewind counters read ZERO against
 * millions of reanchors, so neither the existing bails nor a new one is
 * validated by any test: the counters are here so an oracle built to drive
 * that race can say when it finally does.
 */
unsigned long ft_probe_mrg_desc[3], ft_probe_mrg_conf[3];
unsigned long ft_probe_ranch[2], ft_probe_rewind[2];
unsigned long ft_probe_mspin[5];
unsigned long ft_probe_rspin[6];
unsigned long ft_probe_rspin_x[3], ft_probe_rspin_n[3];
unsigned long ft_probe_rspin_e[3][2];
static const char *const ft_probe_mrg_name[3] = { "src", "mergepoint", "dst" };
static __attribute__((destructor))
void ft_probe_mrg_report(void)
{
	unsigned int i;

	fprintf(stderr, "MRGPROBE");
	for (i = 0; i < 3; i++)
		fprintf(stderr, " %s: descents=%lu conflicts=%lu",
			ft_probe_mrg_name[i], ft_probe_mrg_desc[i],
			ft_probe_mrg_conf[i]);
	fprintf(stderr, " | reanchor compressed=%lu rewinds=%lu slot=%lu rewinds=%lu",
		ft_probe_ranch[0], ft_probe_rewind[0],
		ft_probe_ranch[1], ft_probe_rewind[1]);
	fprintf(stderr, " | mergespin ops=%lu contended=%lu (exclsrc=%lu livesrc=%lu) levelmove=%lu deepest=%lu\n",
		ft_probe_mspin[0], ft_probe_mspin[1], ft_probe_mspin[4],
		ft_probe_mspin[1] - ft_probe_mspin[4], ft_probe_mspin[2],
		ft_probe_mspin[3]);
	/*
	 * Each loop's retries split by the contract that decides whether a
	 * persistent-handle bracket is expressible there at all: excl = the
	 * retries that landed under it, live = the retries that did not.
	 */
	fprintf(stderr, "RSPIN retry_merge=%lu deepest=%lu (excl=%lu live=%lu offc=%lu)"
		" | retry_attach=%lu deepest=%lu (excl=%lu live=%lu offc=%lu)"
		" | retry_swap=%lu deepest=%lu (excl=%lu live=%lu offc=%lu)\n",
		ft_probe_rspin[0], ft_probe_rspin[1],
		ft_probe_rspin_x[0], ft_probe_rspin[0] - ft_probe_rspin_x[0],
		ft_probe_rspin_n[0],
		ft_probe_rspin[2], ft_probe_rspin[3],
		ft_probe_rspin_x[1], ft_probe_rspin[2] - ft_probe_rspin_x[1],
		ft_probe_rspin_n[1],
		ft_probe_rspin[4], ft_probe_rspin[5],
		ft_probe_rspin_x[2], ft_probe_rspin[4] - ft_probe_rspin_x[2],
		ft_probe_rspin_n[2]);
	/*
	 * Slot 0 is shared by TWO loops, so its offc cannot say which one ran --
	 * nor whether either ran at all.  Total ENTRIES per site, split by the
	 * contract, answers both.
	 */
	fprintf(stderr, "RSPIN-ENTRIES merge_subpos{oncontract=%lu offcontract=%lu}"
		" rekey_subpos{oncontract=%lu offcontract=%lu}"
		" graft_swap{oncontract=%lu offcontract=%lu}\n",
		ft_probe_rspin_e[0][1], ft_probe_rspin_e[0][0],
		ft_probe_rspin_e[1][1], ft_probe_rspin_e[1][0],
		ft_probe_rspin_e[2][1], ft_probe_rspin_e[2][0]);
}
#endif

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
#include "ft-txn-kind-stats.h"	/* -DFT_DEBUG_TXN_KIND record-kind counters */
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

#ifdef FEATURE_FT_FAULT_INJECT
extern long cds_ft_fault_commit_countdown;
#endif

#include "ft-graft.h"
#include "ft-detach.h"
#include "ft-merge.h"
#include "ft-rekey.h"	/* same-trie move: see the unit header */
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

/*
 * TEST/DEBUG: hold the move mode gate open without performing a move, so a
 * SINGLE-THREADED test can exercise the coherent reader path at all (with no
 * mover the gate is closed and readers correctly take the fast path, which would
 * make the coherence tests assert against dead code).  Same blocking contract as
 * a real move: not from an RCU read-side critical section.
 */
void _cds_ft_debug_move_gate_enter(struct cds_ft *ft)
{
	ft_move_gate_enter(ft);
}

void _cds_ft_debug_move_gate_exit(struct cds_ft *ft)
{
	ft_move_gate_exit(ft);
}
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
	struct ft_held_anchor marks[FT_ENTRY_PER_NODE + 1];
	unsigned int nr_marks = 0, i, ti;
	bool marks_consumed = false;
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

	txn = ft_flip_txn_create(ft);
	if (!txn)
		return -ENOMEM;
	/* PHASE B, STEP B6: the one door, at creation (see the rekey writer). */
	ft_flip_txn_arm_structural(ft, txn);

	/* The ROOT is its own anchor under every spacing: byte-depth 0. */
	ret = ft_rekey_cow_stop(ft, NULL, txn, root, 0, 0 /*cut*/, &root_prime,
			marks, &nr_marks);
	if (ret) {
		ft_flip_txn_destroy(txn);	/* pre-commit bail: destroy caller-owned txn */
		goto sweep;
	}

	/* Forward publish ft->root: root -> root' (MW by construction -- the
	 * root slot is arbitrated by its own CAS, and an SW park is neither a
	 * CAS nor visible to one; see ft_flip_txn_record_root). */
	if (!ft_flip_txn_reserve_extra(txn, 1)) {
		free_cds_ft_node_unpublished(ft, ft_node_ptr(root_prime));
		ft_flip_txn_destroy(txn);
		ret = -ENOMEM;
		goto sweep;
	}
	/*
	 * @root_prime becomes @ft's root: name the owner while it is still
	 * unpublished (the bail above frees it as such).
	 */
	cds_ft_item_to_metadata(ft_node_ptr(root_prime))->parent_word =
		ft_trie_parent(ft);
	ft_flip_txn_record_root(txn, (void **) &ft->root, root, root_prime);

	st = ft_flip_txn_commit(ft, txn);		/* consumes txn */
	if (st == URCU_TXN_STATUS_OK) {
		marks_consumed = true;	/* every mark released by its state edge */
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
	 * The caller-owned release, on bail/abort paths ONLY: a successful commit already
	 * released every mark through its state edge, so clearing again would take a
	 * peer's fresh mark off a node that is LIVE and CLEAN by then.
	 */
	if (!marks_consumed)
		for (i = 0; i < nr_marks; i++)
			/*
			 * ☠ @txn_owned is skipped: ft_rekey_cow_stop registers
			 * its marks now, and registration TRANSFERS the clear to
			 * the txn's terminals.  ONE OWNER PER FENCE.
			 */
			if (!marks[i].shared && !marks[i].txn_owned) {
				ft_meta_lock_release_if_held(marks[i].lock);
				/*
				 * SCRUB only RELEASED-LIVE (finding A); a
				 * TOMBSTONED word is a CONSUMED fence and must
				 * keep answering holds() -- dedupe-on-dead is
				 * the designed flow, and taking a dead word
				 * hard-refuses forever (the exp-MW storm).
				 */
				if (!(CMM_LOAD_SHARED(marks[i].lock->state) &
						FT_STATE_TOMBSTONE))
					marks[i].shared = true;
			}
	return ret;
}

/*
 * TEST/DEBUG (coherent-rekey sub-step 3, NOT public API): descend @key (converted
 * to ordinal) and return the node flag AT that key as an opaque address, so a test
 * can observe that a rekey moved the moved-subtree top (S_top) to a FRESH address.
 * NULL if the key is absent or its path traverses a non-plain-internal node.
 */
/*
 * Does @flag (a value from _cds_ft_debug_child_at) carry a CO-LOCATED external
 * chain -- a key ending exactly at this node's position?
 *
 * Test-only, and for the same reason as the compressed query beside it: a test
 * that builds this shape to reach a branch gated on it must be able to say it
 * built it, or asserting the outcome alone passes on any other refusal.
 */
int _cds_ft_debug_flag_has_external_chain(struct cds_ft *ft, void *flag)
{
	struct cds_ft_inode_flag *nf = (struct cds_ft_inode_flag *) flag;
	struct cds_ft_metadata *meta;

	if (!nf || ft_node_external(nf))
		return 0;
	nf = ft_resolve_flip_proxy(nf);
	if (ft_node_compressed(nf))
		return 0;	/* a compressed node never carries one */
	meta = ft_flag_to_metadata(ft, nf);
	return meta && meta->external_nodes ? 1 : 0;
}

/*
 * Is path compression compiled in?
 *
 * Test-only.  A test that builds a compressed shape must tell "this build cannot
 * make one" (skip) from "this build should have made one and did not" (fail) --
 * a bare structural check collapses the two and would skip silently through a
 * regression.
 */
int _cds_ft_debug_compress_enabled(void)
{
#ifdef FEATURE_FT_COMPRESS
	return 1;
#else
	return 0;
#endif
}

/*
 * Is the in-place occupancy-bitmap tier compiled in?
 *
 * Test-only, and it must be ASKED OF THE LIBRARY rather than reproduced as a
 * test-side #ifdef: a test that maintains its own copy of a build flag reports
 * on its own copy, and a build whose library and tests disagree reads as a pass.
 *
 * ☠ THE FLAG IS ONLY HALF THE CONDITION.  ft_in_place_ok() also requires an
 * EXCLUSIVE trie, so a test that wants the in-place path must ALSO create one
 * -- which is why the gate's `in-place` config alone never reached
 * ft_store_at_graft_point_commit's in-place arm.
 */
int _cds_ft_debug_in_place_enabled(void)
{
#ifdef FEATURE_FT_INSERT_IN_PLACE
	return 1;
#else
	return 0;
#endif
}

/*
 * Is @flag (a value from _cds_ft_debug_child_at) a COMPRESSED node?
 *
 * Test-only shape introspection.  A test that builds a geometry to reach a
 * kind-specific branch has to be able to say it built it: without this, asserting
 * only the outcome passes whenever ANY other branch produces the same outcome.
 */
int _cds_ft_debug_flag_is_compressed(void *flag)
{
	struct cds_ft_inode_flag *nf = (struct cds_ft_inode_flag *) flag;

	if (!nf)
		return 0;
#ifdef FEATURE_FT_SKIP_COMPRESSED
	if (ft_node_skip_compressed(nf))
		return 1;
#endif
	return ft_node_compressed(nf) ? 1 : 0;
}

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


#ifdef FEATURE_FT_MERGE
#endif /* FEATURE_FT_MERGE */


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
 *   - the SRC JUNCTION BP (= S_top's parent) and the DST PARENT are DISTINCT and
 *     do not alias each other's parent (the shape gate below).  They MAY share a
 *     parent or not: in the default, concurrent-safe build EVERY popcount delete
 *     recompacts, so BP is rebuilt on the removal and republished into its parent,
 *     and that parent is either the node the graft's dst-parent recompaction
 *     already holds the lock -- REUSED via @src_held_hint rather than re-acquired --
 *     or a node this detach acquires itself, guarded.  Both are try-locks that
 *     abort rather than block, so their order is deadlock-free.
 *   - @dst_key is ABSENT and reached by a NOSPLIT graft into a spare slot with
 *     append room (no compressed-divergence GLUE split).
 *   - the trie's ordered list may be ON: the moved subtree's contiguous cell run
 *     unsplices from the src ordered position and re-splices at the dst position
 *     as SIX recorded cell edges in the SAME commit (all MW, so a peer's cell
 *     conflict aborts clean).  The dst splice position must be clear of the run's
 *     own ordered neighbourhood (adjacency guard below).
 * structural_sw STAYS TRUE the whole txn: cow_stop's S_top edges, the graft's
 * dst-parent recompaction, and the detach's BP recompaction all hold their DLM
 * node locks (BP's parent held by the graft via @src_held_hint, or acquired
 * here), so every structural edge is legitimately SW; the mixed engine still
 * sorts the (MW) count edges first.
 *
 * @src_key / @dst_key are APPLICATION keys (converted to ordinal internally),
 * DISJOINT (neither a prefix of the other), of EQUAL length, on a FIXED-length
 * group -- see the scope checks at the top of the body for why each is required.
 *
 * CONCURRENCY.  Abort-clean: every bail leaves the trie byte-for-byte as before.
 * This function is ONE ATTEMPT; ft_rekey_graft_simple_locked wraps it in the
 * retry loop that owns the persistent handle and the escalation turn, and
 * ABSORBS the transient codes -- -EIO (the graft's up-front acquire lost a race)
 * and -EAGAIN (a cow_stop / detach acquire, an incoherently derived splice
 * position, or the final commit aborted).  Callers therefore see 0, -EINVAL
 * (the controlled shape above is not met), or -ENOMEM (reserve).  Retrying
 * externally is NOT equivalent and was the old arrangement: a fresh handle per
 * call never ages, so a contended writer never qualifies for its turn.  The
 * concurrent oracles (inv_rekey_graft_{disjoint,shared,coherent_readers}) exercise
 * exactly that contract.  What is NOT yet multi-writer safe in general: a dst
 * splice neighbour INTERIOR to a peer's concurrently-moving run (see the splice
 * position validation below) -- out of reach in the tested layouts, and closed
 * only by the per-FT move seqcount / relational coherence work.
 */


/*
 * TEST-ONLY: strip @leaf's holder of every child, leaving the holder itself
 * WIRED where it is -- the "dead interior node" shape.
 *
 * Why a hook rather than a test that provokes it: the shape is a library
 * DEFECT's output, so once the defect is fixed no sequence of public calls
 * produces it, and the reader code that copes with it becomes untestable.  It
 * was untested: a counter on ft-inequality.h's empty-subtree arm reads ZERO
 * across ft_unit and ft_inv in every list mode.  That arm decides what an
 * ordered walk does when it descends into a childless internal, and an
 * unexercised arm is where the next defect of this class hides -- the
 * compressed-parent one hid behind a comment claiming zero hits.
 *
 * The holder's external children are handed back through @out rather than
 * freed: they are CALLER-OWNED nodes (the application allocated them), so the
 * library must not free them, and the test needs them to reclaim its own
 * memory.  Returns -1 if the holder is not an internal node, if @out is too
 * small, or if the trie is empty; otherwise 0 with *@out_n set.
 *
 * Leaves the trie in a state cds_ft_verify REJECTS, on purpose.  Callers are
 * expected to walk it, assert whatever they are testing, and destroy it.
 */
int _cds_ft_debug_empty_holder(struct cds_ft *ft, struct cds_ft_node *leaf,
		struct cds_ft_node **out, unsigned int out_max,
		unsigned int *out_n)
{
	struct cds_ft_inode_flag *holder_flag;
	struct cds_ft_metadata *meta;
	unsigned int b, n = 0;

	if (!ft || !leaf || !out || !out_n)
		return -1;
	holder_flag = ft_node_holder(ft, leaf);
	if (!holder_flag || !ft_node_internal(holder_flag))
		return -1;
	meta = cds_ft_item_to_metadata(ft_node_ptr(holder_flag));

	/*
	 * Collect first, clear after: a slot cleared mid-scan would change what
	 * the remaining ft_node_get_nth_skip calls see on a popcount layout,
	 * where the slot index is a rank over the occupancy bitmap.
	 */
	for (b = 0; b < 256; b++) {
		struct cds_ft_inode_flag **slot = NULL;
		struct cds_ft_inode_flag *child =
			ft_node_get_nth_skip(holder_flag, &slot, (uint8_t) b,
					FT_PF_NONE);

		if (!child || !slot)
			continue;
		if (!ft_node_external(child))
			return -1;	/* subtree, not a leaf: caller picked wrong */
		if (n >= out_max)
			return -1;
		out[n++] = (struct cds_ft_node *) ft_node_ptr(child);
	}
	for (b = 0; b < 256; b++) {
		struct cds_ft_inode_flag **slot = NULL;

		if (!ft_node_get_nth_skip(holder_flag, &slot, (uint8_t) b,
				FT_PF_NONE) || !slot)
			continue;
		rcu_assign_pointer(*slot, NULL);
	}
	if (meta->external_nodes) {
		if (n >= out_max)
			return -1;
		out[n++] = meta->external_nodes;
		rcu_assign_pointer(meta->external_nodes, NULL);
	}
	ft_meta_nr_child_set(meta, 0);
	*out_n = n;
	return 0;
}
