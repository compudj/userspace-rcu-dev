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
	fprintf(stderr, " | reanchor compressed=%lu rewinds=%lu slot=%lu rewinds=%lu\n",
		ft_probe_ranch[0], ft_probe_rewind[0],
		ft_probe_ranch[1], ft_probe_rewind[1]);
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

	txn = ft_flip_txn_create();
	if (!txn)
		return -ENOMEM;
	ft_flip_txn_set_structural_sw(txn, true);

	/* The ROOT is its own anchor under every spacing: byte-depth 0. */
	ret = ft_rekey_cow_stop(ft, NULL, txn, root, 0, &root_prime, marks,
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
	/*
	 * @root_prime becomes @ft's root: name the owner while it is still
	 * unpublished (the bail above frees it as such).
	 */
	cds_ft_item_to_metadata(ft_node_ptr(root_prime))->parent_word =
		ft_trie_parent(ft);
	ft_flip_txn_record_reserved(txn, (void **) &ft->root, root, root_prime);

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
	 * The caller-owned release the primitive's contract requires (the marks are
	 * not txn-registered), on bail/abort paths ONLY: a successful commit already
	 * released every mark through its state edge, so clearing again would take a
	 * peer's fresh mark off a node that is LIVE and CLEAN by then.
	 */
	if (!marks_consumed)
		for (i = 0; i < nr_marks; i++)
			if (!marks[i].shared)
				ft_meta_lock_release_if_held(marks[i].lock);
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
 * require pred < the dst range < succ.  @dst_key is a PREFIX, so the range is
 * every key that extends it, and ft_rekey_prefix_range_cmp is what places a
 * neighbour against that range in either group flavour.
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
/*
 * Where does the ordinal key @kb (@klen bytes) sit relative to the RANGE of keys
 * the prefix @dst_ord (@dst_len bytes) covers?  -1 below it, 0 inside it, 1 above.
 *
 * The trie's key order compares bytes over the COMMON length and, on a tie, puts
 * the SHORTER key first (measured: 'aa' < 'ab' < 'abc' < 'abd' < 'b').  So the
 * range is [@dst_ord itself .. every key extending it]: a strict PREFIX of
 * @dst_ord sorts BELOW the range, and @dst_ord itself is INSIDE.  A fixed-length
 * group is the special case where no key is shorter than @dst_len, which is why
 * this used to be expressible as a plain compare against @dst_ord padded with the
 * ordinal extremes -- that padding is what tied the check to fixed lengths, and
 * comparing over the common length instead needs no padding at all.
 */
static
int ft_rekey_prefix_range_cmp(const uint8_t *kb, size_t klen,
		const uint8_t *dst_ord, size_t dst_len)
{
	size_t n = klen < dst_len ? klen : dst_len;
	int cmp = memcmp(kb, dst_ord, n);

	if (cmp != 0)
		return cmp < 0 ? -1 : 1;
	return klen < dst_len ? -1 : 0;
}

/*
 * Free the UNPUBLISHED S_top copy ft_rekey_cow_stop made, whichever kind it is.
 * The two kinds come from different arenas, and the compressed one is handed back
 * as a plain node flag precisely so this inversion is safe (see cow_stop).
 */
static
void ft_rekey_free_stop_prime(struct cds_ft *ft, struct cds_ft_inode_flag *nf)
{
	if (!nf)
		return;
	if (ft_node_compressed(nf))
		free_compressed_node_unpublished(ft, ft_compressed_node_ptr(nf));
	else
		free_cds_ft_node_unpublished(ft, ft_node_ptr(nf));
}

#ifdef FEATURE_FT_MERGE
/*
 * Build the ORDERED-CELL edge set for an IN-TRIE interleave: the moved run's keys
 * thread individually between the destination region's, and every relink is
 * RECORDED so the whole reorder lands in the move's one commit.
 *
 * This is the case a run splice cannot express.  When the moved suffixes do not
 * disjointly precede or follow the region's, the moved cells do not stay
 * contiguous, so there is no single pair of boundary edges to write -- each
 * survivor lands between two region cells.  ft_merge_ord_interleave_collect
 * computes exactly that order; what it needs from an in-trie caller is
 * @record_all, because its survivors are live in the list being rebuilt rather
 * than arriving from a consumed source list.
 *
 * ON TOP OF THE COLLECT, two things the cross-trie merge never needs:
 *  - the SRC GAP.  A cross-trie merge throws its source list away; here the run
 *    vacates a position in the same list, so its old neighbours must be stitched
 *    to each other (ft_ord_cell_run_detach_edges, 2 edges).
 *  - an ADJACENCY refusal.  The run and the region are each contiguous and
 *    disjoint in the CURRENT list (their key prefixes are disjoint; they only
 *    interleave AFTER the move), so the ONLY way an edge slot can repeat is the
 *    two runs abutting -- then the gap closure and the collect's boundary edges
 *    name the same links, and one of the collect's seeds would be a cell that is
 *    itself moving.  Refused rather than special-cased.
 *
 * A COLLISION is refused too, and for a reason the collect's own contract states:
 * it drops a colliding src head on the premise that the cell becomes an
 * unreachable floating duplicate, which holds only when the src list is consumed.
 * In-trie that cell stays linked where it is and would keep answering as a
 * distinct key, so it would have to be unlinked as well -- a further step this
 * does not take.  With @record_all the collect stores nothing, so running it and
 * discarding the result is how the check is made.
 *
 * Returns 0 with *@edges_ret (caller frees) and *@n_ret, or -EINVAL (a shape
 * above), -EAGAIN (a torn read) or -ENOMEM.  Records nothing itself.
 */
static
int ft_rekey_ord_interleave(struct cds_ft *ft, struct cds_ft_inode_flag *D,
		size_t dst_len, size_t src_len,
		struct cds_ft_node *run_rfirst, struct cds_ft_node *run_rlast,
		unsigned long merged_keys,
		struct ft_ord_cell_edge **edges_ret, unsigned int *n_ret)
{
	size_t max_len = ft->group->max_key_len;
	struct ft_ord_cell *rfc, *rlc, *run_pred, *run_succ;
	struct ft_ord_cell *dfirst, *dlast, *reg_pred, *reg_succ;
	struct ft_merge_src_cap *caps = NULL;
	uint8_t *pool = NULL;
	size_t pool_cap = 0, pool_len = 0;
	struct ft_ord_cell_edge *edges = NULL;
	unsigned long nsrc = 0, ncollide = 0, cap_n;
	unsigned int n;
	struct ft_ord_cell *sc, *slast;
	int ret;

	*edges_ret = NULL;
	*n_ret = 0;
	rfc = ft_ord_cell_ptr(rcu_dereference(run_rfirst->prev));
	rlc = ft_ord_cell_ptr(rcu_dereference(run_rlast->prev));
	dfirst = ft_ord_cell_ptr(rcu_dereference(
		ft_subtree_minmax_head(ft, D, false)->prev));
	dlast = ft_ord_cell_ptr(rcu_dereference(
		ft_subtree_minmax_head(ft, D, true)->prev));
	if (!rfc || !rlc || !dfirst || !dlast)
		return -EAGAIN;
	run_pred = ft_ord_cell_resolve_ord(&rfc->lnode.prev);
	run_succ = ft_ord_cell_resolve_ord(&rlc->lnode.next);
	reg_pred = ft_ord_cell_resolve_ord(&dfirst->lnode.prev);
	reg_succ = ft_ord_cell_resolve_ord(&dlast->lnode.next);
	/* The two runs must not abut, in either order (see the header). */
	if (run_succ == dfirst || run_pred == dlast ||
			reg_pred == rlc || reg_succ == rfc)
		return -EINVAL;

	/*
	 * Capture the run's key SUFFIXES while it is still attached and
	 * up-walkable, exactly as ft_merge_spine_copy does: the merge order is
	 * rebuilt from the two live runs, never from the about-to-be-published
	 * structure, so nothing needs a proxy installed first.
	 */
	caps = (struct ft_merge_src_cap *) malloc((merged_keys + 8) *
			sizeof(*caps));
	if (!caps)
		return -ENOMEM;
	sc = rfc;
	slast = rlc;
	for (;;) {
		uint8_t sbuf[FT_MAX_KEY_LEN];
		size_t sfl = ft_rebuild_key_upwalk(ft, sc, sbuf, max_len);
		size_t suf_len;

		if (sfl < src_len || nsrc >= merged_keys + 8) {
			ret = -EAGAIN;		/* torn up-walk, or the run grew */
			goto out;
		}
		suf_len = sfl - src_len;
		if (pool_len + suf_len > pool_cap) {
			size_t ncap = pool_cap ? pool_cap * 2 : 256;
			uint8_t *np;

			while (ncap < pool_len + suf_len)
				ncap *= 2;
			np = (uint8_t *) realloc(pool, ncap);
			if (!np) {
				ret = -ENOMEM;
				goto out;
			}
			pool = np;
			pool_cap = ncap;
		}
		memcpy(pool + pool_len, sbuf + (max_len - sfl) + src_len,
			suf_len);
		caps[nsrc].cell = sc;
		caps[nsrc].suffix_off = pool_len;
		caps[nsrc].suffix_len = suf_len;
		pool_len += suf_len;
		nsrc++;
		if (sc == slast)
			break;
		sc = ft_ord_cell_resolve_ord(&sc->lnode.next);
		if (!sc) {
			ret = -EAGAIN;
			goto out;
		}
	}

	/*
	 * <= 2 visible edges per survivor run + 2 boundary, plus (record_all) up to
	 * 2 per survivor for its own links, plus the 2 src-gap edges.
	 */
	cap_n = 2 * merged_keys + 2 + 2 * nsrc + FT_ORD_CELL_RUN_DETACH_MAX_EDGES;
	edges = (struct ft_ord_cell_edge *) calloc(cap_n, sizeof(*edges));
	if (!edges) {
		ret = -ENOMEM;
		goto out;
	}
	n = ft_merge_ord_interleave_collect(ft, dst_len, dfirst, reg_succ,
			reg_pred, caps, nsrc, pool, edges, /*record_all=*/ true,
			&ncollide);
	if (ncollide) {
		ret = -EINVAL;			/* see the header */
		goto out;
	}
	/* Close the gap the run vacates. */
	n = ft_ord_cell_run_detach_edges(ft, run_rfirst, run_rlast, &rfc, &rlc,
			edges, n);
	assert(n <= cap_n);
	*edges_ret = edges;
	*n_ret = n;
	edges = NULL;
	ret = 0;
out:
	free(edges);
	free(pool);
	free(caps);
	return ret;
}
#endif /* FEATURE_FT_MERGE */

/*
 * Order two key SUFFIXES the way the trie orders keys: bytes over the common
 * length, and on a tie the shorter one first.
 */
static
int ft_rekey_suffix_cmp(const uint8_t *a, size_t alen, const uint8_t *b,
		size_t blen)
{
	size_t n = alen < blen ? alen : blen;
	int cmp = memcmp(a, b, n);

	if (cmp != 0)
		return cmp < 0 ? -1 : 1;
	if (alen == blen)
		return 0;
	return alen < blen ? -1 : 1;
}

/*
 * An OCCUPIED destination: does the moved run land entirely BELOW the merge
 * region already there (-1), entirely ABOVE it (1), or INTERLEAVED with it (0)?
 *
 * Both sides are compared by the suffix BELOW their own merge point, which is what
 * the merged order is decided on -- the moved keys become @dst_key ++ suffix, and
 * the region's are @dst_key ++ their own, so the shared prefix cancels.
 *
 * ★ WHY THE ANSWER MATTERS SO MUCH.  Entirely below or entirely above, the moved
 * cells stay ONE CONTIGUOUS list range and the move is the same run splice an empty
 * destination gets -- at the region's front or back rather than into a gap.
 * INTERLEAVED, it is not a run move at all: the cells have to be threaded
 * individually between the region's, which is ft_merge_ord_interleave_collect's
 * job, and that helper PLAIN-STORES each surviving cell's links on the premise
 * that the cell "is not ord-reachable in @dst -- never was".  True for the
 * cross-trie merge it was written for, FALSE here: an in-trie rekey's source cells
 * are live in the very list being rebuilt, so those stores would be
 * reader-visible and non-atomic.  Interleaving in-trie needs a mode that RECORDS
 * every relink instead, so it is refused rather than approximated.
 *
 * A COLLISION cannot occur in the two cases this admits: identical full keys mean
 * identical suffixes, which strict disjointness excludes.  That is what keeps the
 * duplicate-chain absorption out of the picture here.
 */
static
int ft_rekey_run_vs_region(struct cds_ft *ft,
		struct ft_ord_cell *rfc, struct ft_ord_cell *rlc, size_t src_len,
		struct ft_ord_cell *dfirst, struct ft_ord_cell *dlast,
		size_t dst_len)
{
	size_t max_len = ft->group->max_key_len;
	uint8_t rb[FT_MAX_KEY_LEN], db[FT_MAX_KEY_LEN];
	size_t rl, dl;

	/* run MAX vs region MIN: below iff strictly less. */
	rl = ft_rebuild_key_upwalk(ft, rlc, rb, max_len);
	dl = ft_rebuild_key_upwalk(ft, dfirst, db, max_len);
	if (!rl || !dl || rl < src_len || dl < dst_len)
		return 0;			/* unreadable: treat as interleaved */
	if (ft_rekey_suffix_cmp(rb + (max_len - rl) + src_len, rl - src_len,
			db + (max_len - dl) + dst_len, dl - dst_len) < 0)
		return -1;
	/* run MIN vs region MAX: above iff strictly greater. */
	rl = ft_rebuild_key_upwalk(ft, rfc, rb, max_len);
	dl = ft_rebuild_key_upwalk(ft, dlast, db, max_len);
	if (!rl || !dl || rl < src_len || dl < dst_len)
		return 0;
	if (ft_rekey_suffix_cmp(rb + (max_len - rl) + src_len, rl - src_len,
			db + (max_len - dl) + dst_len, dl - dst_len) > 0)
		return 1;
	return 0;
}

static
bool ft_rekey_splice_pos_brackets(struct cds_ft *ft, const uint8_t *dst_ord,
		size_t dst_len, struct ft_ord_cell *pred,
		struct ft_ord_cell *succ)
{
	size_t max_len = ft->group->max_key_len;
	uint8_t scratch[FT_MAX_KEY_LEN];
	struct ft_ord_cell *sentinel = ft_ord_sentinel_cell(ft);
	bool pred_end = !pred || pred == sentinel;
	bool succ_end = !succ || succ == sentinel;
	size_t klen;

	if (pred_end && succ_end)
		return false;			/* "empty list" -- the run is IN it */
	if (pred_end && ft_ord_first(ft) != succ)
		return false;			/* head insert, but succ is not the min */
	if (succ_end && ft_ord_last(ft) != pred)
		return false;			/* tail insert, but pred is not the max */
	/*
	 * A neighbour INSIDE the range is the torn derivation this exists to catch:
	 * the dst prefix is empty here (an occupied one is not a graft, and the
	 * merge arm refuses the list), so no key legitimately extends it.
	 */
	if (pred && pred != sentinel) {
		klen = ft_rebuild_key_upwalk(ft, pred, scratch, max_len);
		if (!klen || ft_rekey_prefix_range_cmp(scratch + (max_len - klen),
				klen, dst_ord, dst_len) >= 0)
			return false;
	}
	if (succ && succ != sentinel) {
		klen = ft_rebuild_key_upwalk(ft, succ, scratch, max_len);
		if (!klen || ft_rekey_prefix_range_cmp(scratch + (max_len - klen),
				klen, dst_ord, dst_len) <= 0)
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
static
int ft_rekey_graft_simple_attempt(struct cds_ft *ft,
		const uint8_t *src_key, size_t src_len,
		const uint8_t *dst_key, size_t dst_len,
		bool require_empty, struct urcu_txn *optxn)
{
	uint8_t src_ord[FT_MAX_KEY_LEN], dst_ord[FT_MAX_KEY_LEN];
	struct cds_ft_inode_flag *s_top, *s_top_prime = NULL, *attached_nf = NULL;
	struct cds_ft_node *run_rfirst = NULL, *run_rlast = NULL;
	struct ft_ord_cell *run_dpred = NULL, *run_dsucc = NULL;
	struct cds_ft_metadata *s_top_meta, *bp_meta;
	struct ft_detach_recompact_out detach_rc = { 0 };
	struct ft_flip_txn *txn;
	struct ft_glue glue;
	struct ft_graft_store_state gst_st = { 0 };	/* GLUE never runs prepare */
	struct ft_descent d_src, d_dst;
	/*
	 * The src descent is the anchor source for every acquire this attempt
	 * makes; @d_src is only valid once the walk below has run, so the
	 * context is (re)bound where it is used.
	 */
	struct ft_lock_ctx lctx_src;
	struct cds_ft_alloc_reserve reserve;
	struct ft_remove_pub pub = { .armed = false };
	/*
	 * The two nodes the graft's own step locks, whichever dst shape it took:
	 * NOSPLIT {dst parent, its parent}, GLUE {split compressed node, publish
	 * parent}.  @graft_c is the one it retires, @graft_p the one it releases.
	 */
	struct cds_ft_inode_flag *graft_c = NULL, *graft_p = NULL, *cn_flag = NULL;
	struct cds_ft_metadata *pp_meta = NULL;	/* GLUE publish-parent fence WE own */
	uintptr_t pp_snap = 0;
	/* ...unless it DEDUPED onto a word an earlier step of this op took. */
	bool pp_shared = false;
	/*
	 * INCREMENT 3: the dst point is OCCUPIED, so step 2 UNIONS into it with
	 * ft_merge_build instead of grafting a COW'd S_top' into a spare slot.  The
	 * merged cluster reuses @glue as its dst side (so every abort / free_old /
	 * fini path below applies unchanged); @src_glue is the extra src side, which
	 * carries the retire of S_top itself -- the merge does what cow_stop would.
	 */
	struct ft_glue src_glue;
#ifdef FEATURE_FT_MERGE
	struct ft_merge_ctx mctx;
	struct ft_merge_counts mcnt = { 0, 0, 0, 0, 0 };
#endif
	struct cds_ft_inode_flag *merged_nf = NULL;
	struct cds_ft_inode_flag *probe_D = NULL;	/* occupied dst merge point */
	unsigned long merged_keys = 0;
	bool merge_dst = false, src_glue_live = false;
	/*
	 * The dst position abuts the moved run, so the move leaves the run's ordered
	 * position alone -- see the splice-position derivation.
	 */
	bool run_keeps_pos = false;
	bool s_top_compressed = false;	/* the moved top is a compressed run */
#ifdef FEATURE_FT_MERGE
	/* Occupied dst: thread the cells individually, do not splice a run. */
	bool run_interleaves = false;
#endif
	/*
	 * +2, not +1: cow_stop can fill S_top plus all FT_ENTRY_PER_NODE of its
	 * children, and the GLUE shape adds the split cluster's one displaced child.
	 */
	struct ft_held_anchor marks[FT_ENTRY_PER_NODE + 2];
	const uint8_t *ik;
	unsigned int nr_marks = 0, adepth = 0, i, ti;
	bool marks_consumed = false;
	bool src_parent_held;
	enum ft_graft_prep prep;
	enum cds_ft_status gst;
	enum urcu_txn_status gcst = URCU_TXN_STATUS_OK, st;
	unsigned long cnt;
	int ret;

	if (!ft->lock_fine || src_len == 0 || dst_len == 0 ||
			src_len > FT_MAX_KEY_LEN || dst_len > FT_MAX_KEY_LEN)
		return -EINVAL;
	/*
	 * SCOPE (permanent -EINVAL), stated HERE now that the shared-parent shape
	 * gate below no longer stands in for it: EQUAL lengths, which ppnf equality
	 * used to force.  A shorter or longer @dst_key rewrites every moved key's
	 * LENGTH -- which a fixed-length group cannot express at all, and which a
	 * variable-length one would need the max_used_key_len fold for.  With the
	 * lengths equal every moved key keeps its own, so neither applies.
	 *
	 * The group flavour is NOT a scope limit: both are in.  The one thing that
	 * tied this to fixed-length groups was ft_rekey_splice_pos_brackets, which
	 * bounded the dst key range by padding @dst_ord with the ordinal extremes;
	 * it now compares over the common length instead, which needs no padding and
	 * so places a neighbour correctly whatever the key lengths are.
	 */
	if (src_len != dst_len)
		return -EINVAL;
	ft_key_to_ordinals(src_ord, src_key, src_len, &ft->group->key_map);
	ft_key_to_ordinals(dst_ord, dst_key, dst_len, &ft->group->key_map);
	/*
	 * DISJOINT keys: neither may be a prefix of the other (equal keys being
	 * the degenerate case).  A prefix relationship puts one key inside the
	 * other's subtree, so the move is circular -- and it is ALSO what keeps
	 * the ordered-list splice sound now that the shape gate no longer forces
	 * src_len == dst_len by construction.  The run this move re-splices is
	 * exactly the keys under @src_key, an ordinally CONTIGUOUS range, so a
	 * splice neighbour strictly INTERIOR to that run would have to be
	 * bracketed by two run keys -- which puts @dst_key inside the run's own
	 * range, i.e. makes @src_key a prefix of it.  Rejecting that here leaves
	 * only the two ENDPOINT-adjacency shapes for the guard below to catch, so
	 * "no cell of the moved run is a splice neighbour" holds by construction.
	 * (A neighbour interior to a PEER's concurrently-moving run is a
	 * different, still-open question -- see the splice validation below.)
	 */
	if (memcmp(src_ord, dst_ord, src_len < dst_len ? src_len : dst_len) == 0)
		return -EINVAL;

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
			ft_node_external(s_top))
		return -EINVAL;
	/*
	 * A COMPRESSED S_top is in scope: ft_rekey_cow_stop copies the run and its
	 * one child, and the one re-parent it records is also what re-points the
	 * skip back-channel, so nothing about the encoding needs recomputing.
	 *
	 * The SKIP-ENCODED slot form is refused, and that refusal guards a shape a
	 * REKEY CANNOT REACH rather than narrowing this cut.  The encoding is a GROUP
	 * property, and cds_ft_group_attr_set_lookup_optimization CLEARS
	 * CDS_FT_FLAG_SKIP_COMPRESSED for EAGER; a rekey requires EAGER, because it
	 * re-parents a leaf without being able to rewrite an app-owned stored key.
	 * So every compressed run a rekey meets was published PLAIN.  Keep the check:
	 * it is cheap, and it is what stops the accessor mismatch below from becoming
	 * a wild read if that coupling ever changes.
	 */
	if (ft_node_compressed(s_top)) {
		s_top_compressed = true;
	} else {
#ifdef FEATURE_FT_SKIP_COMPRESSED
		if (ft_node_skip_compressed(s_top))
			return -EINVAL;
#endif
		ti = ft_node_type(s_top);
		if (ft_types[ti].type_class != FT_POPCOUNT &&
				ft_types[ti].type_class != FT_PIGEON)
			return -EINVAL;
	}
	/*
	 * A key ending exactly AT @src_key -- @s_top's co-located external chain --
	 * moves with the subtree: ft_rekey_cow_stop carries the forward pointer and
	 * records the head's back edge.  The head itself is app-owned and is never
	 * copied, which is also why an S_top that IS an external head stays out:
	 * there would be no library node to give a fresh address to, and the
	 * coherent reader's witness is built on that freshness.
	 */
	s_top_meta = s_top_compressed ?
		cds_ft_item_to_metadata((struct cds_ft_inode *)
			ft_compressed_node_ptr(s_top)) :
		cds_ft_item_to_metadata(ft_node_ptr(s_top));

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
	 * INCREMENT 3: an OCCUPIED dst is a MERGE, not an error -- the moved subtree
	 * unions into whatever already sits at @dst_key instead of being grafted into
	 * a spare slot.  Probe read-only and conservatively (a descent that cannot
	 * reach the dst depth plainly proves nothing, so those shapes are left to the
	 * later gate) and record the answer for step 2.
	 *
	 * ★ WHY THE MERGE PATH DOES NOT COW S_top, and it is not an oversight.
	 * ft_rekey_cow_stop exists to give the moved subtree's top a FRESH ADDRESS, so
	 * the coherent reader's two-descent witness sees the move as a changed visited
	 * -node set.  A merge already does that BY CONSTRUCTION: entered with an
	 * internal, non-compressed S_top, ft_merge_build cannot take either of the two
	 * exits that return a live node (the shared-run collapse needs both sides
	 * compressed; the leaf-splice needs both external), so it falls to its tail and
	 * returns a freshly allocated M -- and it retires S_top outright on the way.
	 * Interposing a COW would allocate a copy for the merge to consume and free.
	 * cow_stop's OTHER job -- marking the children whose state words the commit
	 * parks into -- is NOT redundant, and is done by the glue's own acquire.
	 *
	 * SCOPE OF THIS FIRST CUT (each a PERMANENT -EINVAL, checked at the gate below,
	 * not silently degraded):
	 *  - with the list ON, a moved run whose suffixes DISJOINTLY precede or follow
	 *    the destination region's.  Then the moved cells stay one contiguous range
	 *    and splice at a region boundary, which is the same run move an empty
	 *    destination gets.  An INTERLEAVED range is refused: threading the cells
	 *    individually is ft_merge_ord_interleave_collect's job, and that helper
	 *    plain-stores each surviving cell's links because they are not
	 *    ord-reachable in a cross-trie merge -- which is false here, where the
	 *    source cells live in the very list being rebuilt (ft_rekey_run_vs_region).
	 *  - a PLAIN INTERNAL node at the dst point.  Compressed / skip / external
	 *    merge points bring the KEY_SHORTER wrap and Edge-D shapes, which are
	 *    ft_merge_spine_copy's job.
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
		merge_dst = (d_probe.depth == dst_len && d_probe.nf != NULL);
		probe_D = merge_dst ? d_probe.nf : NULL;
		/*
		 * @require_empty is the GRAFT caller's semantics (cds_ft_rekey_graft
		 * refuses an occupied destination rather than unioning into it), and
		 * -EEXIST is how it travels back: a distinct code, because the
		 * dispatcher must tell "the destination holds content" (terminal,
		 * POPULATED_ERROR) apart from "this shape is outside the one-decide
		 * cut" (-EINVAL, try the staged writer).  Read-only so far, so this is
		 * a clean no-op bail.
		 */
		if (merge_dst && require_empty)
			return -EEXIST;
		/*
		 * A COMPRESSED S_top is in scope for the GRAFT arm only.  The merge arm
		 * skips ft_rekey_cow_stop on the argument that ft_merge_build gives the
		 * moved top a fresh address BY CONSTRUCTION -- and that argument names
		 * its premise: "entered with an internal, non-compressed S_top,
		 * ft_merge_build cannot take either of the two exits that return a live
		 * node (the shared-run collapse needs BOTH SIDES COMPRESSED...)".  A
		 * compressed S_top is exactly what unlocks that exit, so the freshness
		 * the coherent reader's witness depends on would be gone.
		 */
		if (merge_dst && s_top_compressed)
			return -EINVAL;
		/*
		 * A co-located external chain is carried by ft_rekey_cow_stop, which
		 * the MERGE arm skips -- ft_merge_build would have to union that key
		 * into the destination's own chain, and nothing here has tested it.
		 * The graft arm takes it.
		 */
		if (merge_dst && s_top_meta->external_nodes)
			return -EINVAL;
#ifndef FEATURE_FT_MERGE
		/*
		 * An OCCUPIED destination IS a merge (INCREMENT 3 unions S_top
		 * into it with ft_merge_build), so -DNO_FEATURE_FT_MERGE compiles
		 * that fold out along with the rest of the subsystem.  Report it
		 * the way the merge API itself does rather than building a fold
		 * whose machinery is not there.  The EMPTY-dst rekey below is a
		 * graft and stays available.
		 */
		if (merge_dst)
			return -ENOTSUP;
#endif
	}

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
	 * txn / lock acquire / record, so the bail is a clean no-op -EINVAL): because
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
	 * excluded by the DISJOINT-key rule at the top of this function -- an interior
	 * neighbour would have to be bracketed by two run keys, which puts @dst_key
	 * inside the run's own contiguous ordinal range and so makes @src_key a prefix
	 * of it.  The rule is stated here rather than left implicit in a shape
	 * gate, because no gate forces src_len == dst_len.
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
		if (merge_dst) {
			/*
			 * OCCUPIED destination: the splice position is not a gap to
			 * search for, it is a BOUNDARY of the merge region already there,
			 * so derive it from that region's own endpoint cells instead of
			 * relationally.  That is strictly better than find_splice_pos --
			 * a structural read of the region needs no two-pass agreement and
			 * no bracket check to be trusted -- and it is available only here,
			 * where the region exists.
			 *
			 * Which boundary depends on where the moved run sorts relative to
			 * the region, and an INTERLEAVE is refused: see
			 * ft_rekey_run_vs_region for why that one needs machinery this
			 * writer does not have.
			 *
			 * THE DISJOINTNESS SURVIVES TO THE COMMIT, and the splice's own
			 * edges are what make it.  The only peer insert that can break it
			 * is one landing BELOW the region's minimum (for a run spliced in
			 * front) -- anything inside the region still sorts above the whole
			 * run, so the order holds -- and such an insert splices between
			 * @run_dpred and @run_dsucc, which is the very pair
			 * ft_ord_cell_run_resplice_edges records as pred->next == succ and
			 * succ->prev == pred.  It therefore ABORTS this commit rather than
			 * mis-ordering the list.  Symmetrically for a run spliced behind.
			 */
			struct ft_ord_cell *dfirst, *dlast;

			dfirst = ft_ord_cell_ptr(rcu_dereference(
				ft_subtree_minmax_head(ft, probe_D, false)->prev));
			dlast = ft_ord_cell_ptr(rcu_dereference(
				ft_subtree_minmax_head(ft, probe_D, true)->prev));
			if (!dfirst || !dlast)
				return -EAGAIN;		/* region read torn */
			switch (ft_rekey_run_vs_region(ft, rfc, rlc, src_len,
					dfirst, dlast, dst_len)) {
			case -1:			/* run below: splice in front */
				run_dpred = ft_ord_cell_resolve_ord(
						&dfirst->lnode.prev);
				run_dsucc = dfirst;
				break;
			case 1:				/* run above: splice behind */
				run_dpred = dlast;
				run_dsucc = ft_ord_cell_resolve_ord(
						&dlast->lnode.next);
				break;
			default:
				/*
				 * INTERLEAVED: not a run move.  The cells thread
				 * individually between the region's, which
				 * ft_rekey_ord_interleave builds at the cell step
				 * below (and which refuses the shapes it cannot
				 * express).  No splice pair applies.
				 */
#ifdef FEATURE_FT_MERGE
				run_interleaves = true;
				break;
#else
				return -EINVAL;		/* no merge: no interleave */
#endif
			}
		} else {
			/*
			 * COHERENT derivation: two from-root traversals, compared by their
			 * visited-node witness (ft_ord_cell_find_splice_pos_coherent).
			 * This is what makes the splice position trustworthy -- the
			 * single-pass relational answer is not, and no amount of checking
			 * the ANSWER repairs that (measured: the key-bracket check below,
			 * which is a relational-era read itself, was fooled too).
			 * Disagreement means a peer move is restructuring this
			 * neighbourhood: bail and re-derive.
			 */
			if (!ft_ord_cell_find_splice_pos_coherent(ft, dst_key,
					dst_len, &run_dpred, &run_dsucc))
				return -EAGAIN;	/* torn derivation: re-descend */
			/*
			 * Belt and braces, and cheap: the pair must also BRACKET the dst
			 * key range.  Adjacent + bracketing is the full correctness
			 * condition for a splice; the two-pass agreement establishes the
			 * pair was not read torn, this establishes it is the RIGHT pair.
			 */
			if (!ft_rekey_splice_pos_brackets(ft, dst_ord, dst_len,
					run_dpred, run_dsucc))
				return -EAGAIN;
		}
		/*
		 * THE DST POSITION ABUTS THE RUN, and that is not a shape to refuse: it
		 * means the moved keys sort into the SAME list slot the run already
		 * occupies, so the unsplice and the re-splice CANCEL and the run STAYS
		 * PUT.  Recorded as @run_keeps_pos and honoured at the splice below.
		 *
		 * Why it cancels, for the two ways it arises.  The located pair is
		 * ADJACENT in the list, so @run_dsucc == the run's first cell forces
		 * @run_dpred to be the run's own predecessor A, and @run_dpred == its
		 * last cell forces @run_dsucc to be its successor B.  Either way the dst
		 * key range lies strictly inside (A, B) -- and the run is the ONLY thing
		 * in (A, B) -- so the run's NEW keys sort between A and B exactly where
		 * its old ones did.  Nothing about the list has to change.
		 *
		 * Emitting the six edges anyway is what made this look unsupportable:
		 * the unsplice's A->next = B and the re-splice's A->next = rfc are TWO
		 * RECORDS ON ONE SLOT, which breaks the engine's distinct-slot rule and
		 * would plain-store a self-cyclic run link.  Locating the pair against
		 * the run-REMOVED list -- the other repair this comment used to propose
		 * -- yields (A, B) and therefore those same cancelling edges; the
		 * cancellation is the answer, not a different derivation.
		 */
		if (run_dsucc == rfc || run_dpred == rlc)
			run_keeps_pos = true;
	}

	cnt = ft_nr_keys_get(s_top_meta);	/* subtree key count (count edges no-op if rank off) */

	/*
	 * ON the op's persistent handle, not a standalone one: retry aging and
	 * the FIFO escalation turn live in @optxn and must survive this attempt.
	 * ft_flip_txn_create() inits a handle with NO domain, so every attempt
	 * started fresh, never qualified for a turn, and simply spun.
	 */
	txn = ft_flip_txn_create_on(optxn);
	if (!txn)
		return -ENOMEM;

	/* 1. COW S_top -> S_top' (SW; records re-parents + retire, LOCKED). */
	ft_flip_txn_set_structural_sw(txn, true);
	if (!merge_dst) {
		ft_lock_ctx_init(&lctx_src, &d_src, txn);
		/*
		 * NOT bound to @optxn.  ft_flip_txn_create_*_on sets t->mtxn =
		 * op -- an "_on" txn SHARES the handle rather than making its own
		 * -- and this driver's CONTENT txn is already
		 * ft_flip_txn_create_on(optxn).  Binding here would put the
		 * DEDICATED acquire txn on that same live handle and commit it
		 * mid-op (measured: SIGSEGV in test_rekey_graft_liston, every
		 * spacing).  The acquire's escalation aging and a content txn on
		 * one handle are mutually exclusive; giving this path both needs
		 * a SECOND handle, not a binding.
		 */
		ret = ft_rekey_cow_stop(ft, &lctx_src, txn, s_top, d_src.depth,
				&s_top_prime, marks, &nr_marks);
		if (ret) {
			ft_flip_txn_destroy(txn);	/* pre-commit bail: destroy caller-owned */
			goto sweep;
		}
	}

	/*
	 * 2. + 3.  structural_sw STAYS TRUE for the rest: the graft ALWAYS relocates
	 * the dst attach node (a reserve recompaction), which ACQUIRES node locks
	 * over the dst parent + the republish grandparent and records its re-parents /
	 * release / retire as SW under those locks -- so the graft forward publish and
	 * recompact edges are correctly SW.  The detach's src-junction (BP) edges are
	 * UNLOCKED, and no toggle is needed (toggling OFF would wrongly demote the
	 * recompact's lock-expecting edges to MW -> expected-old mismatch -> abort).
	 *
	 * ★ NOT because "ft_ord_cell_record_into forces them MW regardless of
	 * structural_sw", which this said and which is false for exactly these
	 * edges: ft_ord_cell_record_into dispatches on ft_edge_tag(), and
	 * ft_edge_tag maps tag 0 -> FT_FLIP_PROXY_TAG, taking the
	 * ft_flip_txn_record_tag (structural_sw-honouring) branch, NOT
	 * ft_flip_txn_record_tag_mw.  Only a non-zero (ordered-cell) tag is forced
	 * MW.  What actually makes the src-junction edges safe here is the
	 * exclusive-source gate, not the record path -- so do not weaken that gate
	 * on the strength of the old sentence.
	 * The mixed commit installs the MW (detach) edges first, then parks the SW
	 * (graft + cow_stop) edges before the flip.
	 */

	/* 2. Graft-fold: record-only attach of S_top' at @dst_key. */
	ft_glue_init(&glue);
	glue.txn = txn;
	glue.record_only = true;
	/*
	 * GLUE (compressed-divergence) shape: have the build FENCE the compressed
	 * node it splits before it reads its plan, so the whole build runs under
	 * that fence and its retire rides our commit (ft_split_compressed_graft_build).
	 * The fence is @glue's until the commit registers it; ft_glue_abort is the
	 * single release point, and every bail below routes through it.
	 */
	glue.fence_split_cn = true;
	memset(&reserve, 0, sizeof(reserve));
	/*
	 * The node reserve exists for the GRAFT arm, whose recompaction allocates
	 * under a no-fail contract and so activates it.  The merge arm never
	 * activates it -- ft_merge_build allocates its cluster from the arena and is
	 * allowed to fail, which is the whole reason the fold can still bail there --
	 * so filling it would allocate a batch per move only to drain it untouched.
	 */
	if (!merge_dst && ft_bulk_node_reserve_fill(ft, &reserve)) {
		ft_flip_txn_destroy(txn);
		ret = -ENOMEM;
		goto sweep;
	}
	if (merge_dst) {
#ifdef FEATURE_FT_MERGE
		/*
		 * INCREMENT 3, step 2': UNION S_top into the occupied dst.
		 *
		 * Descend the dst ourselves -- ft_graft_build would report POPULATED and
		 * hand back nothing to publish into.  Plain internal nodes only, the same
		 * restriction the src descent above makes, so d_dst names {D, publish
		 * parent, publish slot, grandparent} with no compressed shape in between.
		 */
		const uint8_t *dk = dst_ord;

		ft_descent_init(&d_dst, ft);
		while (d_dst.depth < dst_len) {
			if (!d_dst.nf || ft_node_external(d_dst.nf) ||
					ft_node_compressed(d_dst.nf)
#ifdef FEATURE_FT_SKIP_COMPRESSED
					|| ft_node_skip_compressed(d_dst.nf)
#endif
			   ) {
				ret = -EINVAL;
				goto bail_build;
			}
			ft_descent_step(ft, &d_dst, *(dk++));
		}
		/*
		 * The merge point itself must be a PLAIN INTERNAL node: a compressed,
		 * skip or external D is the KEY_SHORTER / Edge-D / leaf-splice family,
		 * which belongs to ft_merge_spine_copy.  d_dst.nf is non-NULL by the
		 * probe, but re-checked because the probe ran outside this txn.
		 */
		if (d_dst.depth != dst_len || !d_dst.nf ||
				ft_node_flip_proxy(d_dst.nf) ||
				ft_node_external(d_dst.nf) ||
				ft_node_compressed(d_dst.nf) ||
#ifdef FEATURE_FT_SKIP_COMPRESSED
				ft_node_skip_compressed(d_dst.nf) ||
#endif
				!d_dst.pnf || !d_dst.nfp) {
			ret = -EINVAL;
			goto bail_build;
		}
		{
			unsigned int dti = ft_node_type(d_dst.nf);

			if (ft_types[dti].type_class != FT_POPCOUNT &&
					ft_types[dti].type_class != FT_PIGEON) {
				ret = -EINVAL;
				goto bail_build;
			}
		}
		/*
		 * FENCE the publish parent OURSELVES, exactly as the GLUE arm does and
		 * for the same reason: this txn is structural_sw, so the forward publish
		 * PARKS a plain store into that node's slot, and the fold's rule is SW
		 * iff the op holds the slot's lock.  ft_glue_txn_commit_edges would
		 * otherwise degrade an acquire miss to a §4.B guard.
		 */
		/*
		 * Hand the fold's glue its anchor source, here, the single place
		 * holding both (ft_merge_spine_copy does the same at its own
		 * gd.lock_d).  The fold's members are reached from the BUILD --
		 * a deferred re-parent child above all -- so nothing else can
		 * date them, and a glue with no descent leaves every one of them
		 * undatable: the acquire bails, the caller re-descends, and the
		 * next attempt is identical.
		 */
		glue.lock_d = &d_dst;
		/*
		 * And the SRC path for the members still on it.  The fold's
		 * deferred re-parents come from BOTH sides -- the src children
		 * being moved and the dst children being absorbed -- and each
		 * must anchor where it is NOW, which is the only position a peer
		 * can descend to before this commit lands (§3).
		 */
		glue.lock_d_src = &d_src;
		pp_meta = ft_flag_to_metadata(ft, d_dst.pnf);
		{
			struct ft_lock_ctx dctx;
			struct ft_held_anchor pph;

			/* The DST descent dates this one: it IS its parent slot. */
			ft_lock_ctx_init(&dctx, &d_dst, txn);
			if (!pp_meta || ft_acquire_member(ft, &dctx, d_dst.pnf,
					pp_meta, d_dst.pdepth, &pph) ||
					pph.shared) {
				pp_meta = NULL;
				ret = -EAGAIN;
				goto bail_build;
			}
			pp_meta = pph.lock;
			pp_snap = pph.lock_snap;
		}
		glue.publish_parent_holder = pp_meta;
		glue.publish_parent_snap = pp_snap;

		/* Size both glues from the read-only pre-pass, with headroom. */
		ft_glue_init(&src_glue);
		src_glue_live = true;
		/*
		 * ONE op, ONE held set.  The two glues commit together, so a mark
		 * either takes is a mark the other's dedupe must see -- and under
		 * coarsening they DO meet: a src child anchors on the src node the
		 * merge build fenced as an overlap, so the re-parent mark refuses
		 * against this op's own fence and every retry rebuilds it.
		 */
		glue.peer = &src_glue;
		src_glue.peer = &glue;
		ft_merge_count(ft, s_top, 0, d_dst.nf, 0, &mcnt);
		if (ft_glue_reserve(&glue, mcnt.nb + 8, mcnt.nd + 8,
					mcnt.nf_dst + 8, mcnt.ns + 8) ||
		    ft_glue_reserve(&src_glue, 0, 0, mcnt.nf_src + 8, 0)) {
			ret = -ENOMEM;
			goto bail_build;
		}
		/*
		 * FUSE both free lists into the shared txn.  The dst side MUST (a fenced
		 * overlap retire records its {LOCK|s -> TOMBSTONE|s} terminal into
		 * g->txn and asserts on this flag), and the src side must for the reason
		 * the whole fold exists: its free list carries S_top, whose retire has to
		 * flip WITH the publish rather than as a standalone lone-edge store the
		 * commit could not roll back.  The reserve above sized both.
		 */
		glue.fuse_free_list = true;
		src_glue.txn = txn;
		src_glue.fuse_free_list = true;
		mctx.dst_ft = ft;
		mctx.gd = &glue;
		mctx.gs = &src_glue;
		/*
		 * FENCE the dst overlap spine.  Unlike the STAGED rekey -- which reaches
		 * ft_merge_spine_copy already past its point of no return and so must
		 * skip this -- the one-decide fold's single commit is still ahead of it,
		 * so a missed fence is a clean re-descend and the fence is affordable.
		 */
		mctx.fence_overlap = ft->lock_fine;
		/*
		 * ★ And the SRC spine too, which the cross-trie merge does not need.
		 * cds_ft_merge_at owns its source exclusively; THIS source is the live
		 * in-trie subtree -- the detach is only RECORDED into the same txn, so
		 * S_top and everything under it stays reachable to peers for the whole
		 * build window.  Unfenced, a peer inserting below S_top during the copy
		 * is retired along with the node it was inserted into: silent key loss
		 * (ft_merge_lock_overlap's header states the mechanism for the dst side;
		 * it is the same mechanism).
		 */
		mctx.fence_src = ft->lock_fine;
		mctx.overlap_contended = false;
		merged_nf = ft_merge_build(&mctx, s_top, 0, d_dst.nf, 0, 0,
				&merged_keys);
		if (merged_nf == FT_MERGE_OOM) {
			merged_nf = NULL;
			ret = mctx.overlap_contended ? -EAGAIN : -ENOMEM;
			goto bail_build;
		}
		/*
		 * The merged top replaces D in the publish parent's slot.  Recorded, not
		 * stored: ft_glue_txn_commit_edges runs at step 3c below, after the
		 * detach, like the GLUE arm.
		 */
		ft_glue_set_publish(ft, &glue, d_dst.pnf, d_dst.nfp, merged_nf);
		glue.attached_nf = merged_nf;
		glue.count_delta = (long) cnt;
		/*
		 * COLLIDED KEYS: a full key present on BOTH sides makes ft_merge_build
		 * splice the src leaf onto the dst head's DUPLICATE CHAIN.  That append
		 * walks a LIVE chain, so it runs under the chain holder's node lock --
		 * the exclusion @cc91bd8b added when it closed the last unlocked chain
		 * mutation.  ft_merge_spine_copy takes it before its own point of no
		 * return and SKIPS it for a pre-reserved caller, whose placement cannot
		 * fail; this fold is the case its comment anticipated -- a single commit
		 * still ahead of us, so failing here is free and the lock is ours to take.
		 * No-op on a collision-free merge, which is every disjoint union.
		 */
		if (ft_glue_acquire_splice_holders(ft, &glue)) {
			ret = -EAGAIN;
			goto bail_build;
		}
		attached_nf = merged_nf;
		adepth = (unsigned int) dst_len;
		prep = FT_GRAFT_PREP_NOSPLIT;	/* not a graft; keeps the arms below off */
#endif /* FEATURE_FT_MERGE */
	} else
	{
		/*
		 * The op's outstanding marks -- ft_rekey_cow_stop's @stop fence
		 * and one per COW'd child -- reach no registry until the sweep
		 * below, so the build's own acquires can only see them through
		 * this frame (the same frame the store-prepare arm passes).  This
		 * is an IN-TRIE move: src and dst share a root, so under a coarse
		 * spacing those marks and the split-CN fence are ONE word, and
		 * without the frame the build refuses the op's own fence and the
		 * caller re-descends onto the identical shape forever.
		 */
		struct ft_lock_ctx bctx;

		ft_lock_ctx_init(&bctx, &d_src, txn);
		bctx.held.extra = marks;
		bctx.held.nr_extra = nr_marks;
		prep = ft_graft_build(ft, dst_ord, dst_len, s_top_prime, cnt,
			&d_dst, &glue, &bctx.held);
	}
	/*
	 * Non-buildable outcomes, mapped to the driver's contract.  A build that
	 * reports OOM or a lost split-fence race has published NOTHING (and, on the
	 * RETRY arm, has not even marked the compressed node), so each is the clean
	 * transient the caller re-descends on; an occupied graft point is the
	 * permanent shape error the up-front dst probe already rejects for the
	 * shapes it can prove, and this is the same answer for the rest.
	 */
	if (prep == FT_GRAFT_PREP_OOM) {
		ret = -ENOMEM;
		goto bail_build;
	}
	if (prep == FT_GRAFT_PREP_RETRY) {
		ret = -EAGAIN;
		goto bail_build;
	}
	if (prep == FT_GRAFT_PREP_POPULATED) {
		/*
		 * Content at or below the graft point that the up-front probe could
		 * not prove (a compressed or skip-encoded occupant it declines to
		 * walk).  For a GRAFT that is the destination-occupied answer its
		 * caller documents; for a MERGE it is a dst shape this cut leaves to
		 * ft_merge_spine_copy.
		 */
		ret = require_empty ? -EEXIST : -EINVAL;
		goto bail_build;
	}
	/*
	 * A reanchoring descent step landed a live node SHALLOWER than the
	 * dispatched child, so the captured publish chain names the wrong level --
	 * the same bail the graft and insert take.  Nothing is reader-visible yet.
	 */
	if (caa_unlikely(d_dst.skip_conflict)) {
		ret = -EAGAIN;
		goto bail_build;
	}
	/*
	 * SHAPE GATE.  Two dst shapes are supported, and what distinguishes them is
	 * only WHICH TWO NODES the graft locks:
	 *
	 *  - NOSPLIT: an absent, append-in-place dst point.  The reserve
	 *    recompaction retires and RELOCATES the dst parent d_dst.pnf, holding
	 *    its parent d_dst.ppnf and releasing it at the flip.  (Its optional
	 *    third member, the SKIP_X great-grandparent, is excluded by requiring a
	 *    PLAIN d_dst.ppnf -- which the pre-lift gate got for free from the src
	 *    descent's own plainness checks, ppnf being shared.)
	 *  - GLUE: the dst key diverges INSIDE a compressed node, so the build
	 *    assembled the whole split cluster invisibly.  It holds the split node
	 *    @cn (fenced by the build, retired by our commit) and -- fenced just
	 *    below -- @glue.publish_parent, the live node whose slot the forward
	 *    publish replaces.  A COMPRESSED publish_parent is refused: the publish
	 *    would then also rewrite the SKIP_X dual, a slot in a THIRD node this
	 *    op does not hold.
	 *
	 * Against that pair, the src junction BP (= d_src.pnf) and its parent:
	 *   - BP's parent IS the graft-held node: REUSE the held lock
	 *     (@src_parent_held).  This is the shape the hook started with.
	 *   - BP's parent is any other node: the detach ACQUIRES it itself, guarded
	 *     (@parent_guard).  Deadlock-free -- both acquires are try-locks that
	 *     abort rather than block.
	 *   - BP, or BP's parent, IS one of the graft's two nodes: rejected here,
	 *     PERMANENTLY.  Re-locking a held node aborts -EAGAIN every time, so a
	 *     caller retrying that transient code would spin forever; and the
	 *     same-junction shape (BP == the node the graft relocates or retires) is
	 *     worse than unlockable -- the detach would edit the copy the flip
	 *     retires.  (Cross-depth aliases are unreachable while the scope keeps
	 *     src_len == dst_len, which puts both junctions on the same level; they
	 *     are rejected rather than asserted so that a later relaxation of the
	 *     length rule cannot silently reach them.)
	 *
	 * Plus, for both: the junctions must be BELOW the root (d_src.ppnf and the
	 * graft's own publish target non-NULL) -- a root-level junction republishes
	 * into &ft->root, a slot with no node word to lock, so its SW park would be
	 * an unguarded plain store.
	 */
	if (merge_dst) {
		/*
		 * The merge's two nodes, in the same roles the graft's pair plays: @D is
		 * the one it RETIRES (through the fenced overlap terminal) and the
		 * publish parent is the one it RELEASES at the flip.  Both are already
		 * held -- D by the overlap fence, the parent by the mark above -- so the
		 * gate below only has to keep the detach's junctions clear of them.
		 */
		graft_c = d_dst.nf;
		graft_p = d_dst.pnf;
	} else if (prep == FT_GRAFT_PREP_GLUE) {
		cn_flag = d_dst.nf;		/* the compressed node the build split */
		graft_c = cn_flag;
		graft_p = glue.publish_parent;
	} else {
		graft_c = d_dst.pnf;
		graft_p = d_dst.ppnf;
		if (d_dst.depth != dst_len || d_dst.nf) {
			ret = -EINVAL;		/* occupied / short NOSPLIT point */
			goto bail_build;
		}
	}
	src_parent_held = d_src.ppnf == graft_p;
	if (!d_src.ppnf || !graft_p || !graft_c ||
			ft_node_compressed(graft_p) ||
			ft_node_skip_compressed(graft_p) ||
			d_src.pnf == graft_c || d_src.pnf == graft_p ||
			d_src.ppnf == graft_c) {
		ret = -EINVAL;
		goto bail_build;
	}
	/*
	 * Reserve the graft slot edge + the detach struct/state edges, plus (list on)
	 * the six ordered-cell run boundary edges (src unsplice + dst splice), plus
	 * -- for the GLUE shape -- the split cluster's own bound: its deferred
	 * back-edges, forward publish, split retire and free-list tombstones, the
	 * same floor ft_graft_keylen reserves its standalone txn to.  The count walk
	 * is depth-bounded and only exists when the trie keeps rank stats.
	 */
	if (!ft_flip_txn_reserve_extra(txn, FT_REMOVE_COMMIT_REC_MAX_EDGES + 4 +
			(ft->ordered_list ? FT_ORD_CELL_RUN_DETACH_MAX_EDGES +
				FT_ORD_CELL_RUN_RESPLICE_MAX_EDGES : 0) +
			/*
			 * Merged cluster: every deferred re-parent costs up to THREE
			 * records (parent, state guard, offset) now that the fold routes
			 * them through ft_reparent_record, plus one tombstone per retired
			 * overlap node on either side, the forward publish, and the
			 * depth-bounded count walk.  Sized from the read-only pre-pass,
			 * with the same +8 headroom the glue arrays get.
			 */
#ifdef FEATURE_FT_MERGE
			(merge_dst ? 3 * (unsigned int) (mcnt.nd + 8) +
				(unsigned int) (mcnt.nf_dst + mcnt.nf_src + 16) + 8 +
				(unsigned int) (mcnt.ns + 8) +	/* dup-chain splices */
				(ft->rank_stats ? (unsigned int) dst_len + 1 : 0) : 0) +
#endif
			(prep == FT_GRAFT_PREP_GLUE ?
				FT_GLUE_FLOOR_DEFERRED + 7 + 1 + FT_GLUE_FLOOR_FREE +
				(ft->rank_stats ? (unsigned int) dst_len + 1 : 0) : 0))) {
		ret = -ENOMEM;
		goto bail_build;
	}
	if (merge_dst) {
		/*
		 * Nothing more to prepare: the merged cluster is built, its publish is
		 * set, and its publish parent was fenced with the descent (both above).
		 * The single reserve is already drained by ft_merge_build's own
		 * allocations, so there is no graft reserve to activate here.
		 */
		cds_ft_alloc_reserve_drain(ft, &reserve);
	} else if (prep == FT_GRAFT_PREP_GLUE) {
		/*
		 * FENCE the publish parent OURSELVES, and fail the move on a miss.
		 * ft_glue_txn_commit_edges would otherwise route an unheld parent
		 * through ft_flip_txn_lock_or_guard_parent, which DEGRADES an acquire
		 * miss to a §4.B guard -- correct for the cross-trie graft, wrong here:
		 * this txn is structural_sw, so the forward publish PARKS a plain store
		 * into that node's slot, and the fold's rule is SW iff the op holds the
		 * slot's lock.  A miss (retired / proxied / peer-locked) is the clean
		 * transient: nothing is published, and ft_meta_lock_acquire did not set
		 * the fence.  Mirrors ft_graft_keylen's own pre-swap fence.
		 */
		{
			struct ft_held_anchor pph;

			ft_lock_ctx_init(&lctx_src, &d_src, txn);
			/*
			 * The REST of the op's held set, exactly as the
			 * store-prepare and detach arms name it:
			 * ft_rekey_cow_stop's marks reach no registry until the
			 * sweep, and the glue holds the split-CN fence.  Under a
			 * coarse spacing this publish parent anchors onto one of
			 * them -- the trie root, for an in-trie move -- and a
			 * frame naming neither refuses the op's own fence.
			 */
			lctx_src.held.extra = marks;
			lctx_src.held.nr_extra = nr_marks;
			lctx_src.held.glue = &glue;
			pp_meta = ft_flag_to_metadata(ft, glue.publish_parent);
			unsigned int ppd;

			/*
			 * The publish parent came from the glue, not from a
			 * descent step, so the window is what dates it; a node
			 * this descent never passed voids the attempt.
			 */
			if (!ft_lock_ctx_depth_of(ft, &lctx_src,
						glue.publish_parent, &ppd) ||
					ft_acquire_member(ft, &lctx_src,
						glue.publish_parent, pp_meta,
						ppd, &pph)) {
				pp_meta = NULL;
				ret = -EAGAIN;
				goto bail_build;
			}
			/*
			 * A SHARED hit is the dedupe WORKING: the fence is in
			 * force from an earlier acquire of this same op, so the
			 * publish may park its SW store -- the fold's rule is SW
			 * iff the op holds the slot's word, and it does.  What it
			 * must NOT do is settle the word twice, which is what
			 * @publish_parent_shared tells the commit.
			 */
			pp_meta = pph.lock;
			pp_snap = pph.lock_snap;
			pp_shared = pph.shared;
		}
		glue.publish_parent_holder = pp_meta;
		glue.publish_parent_snap = pp_snap;
		glue.publish_parent_shared = pp_shared;
		/*
		 * The one LIVE node the split cluster re-parents: @cn's displaced
		 * child, which moves onto the fresh suffix (or straight under the fresh
		 * branch when there is no suffix).  Its (parent, offset) pair parks SW,
		 * and the offset lives in the state word ft_meta_nr_child_inc CASes
		 * from an insert BELOW it -- which @cn's fence does not exclude -- so
		 * MARK it, exactly as ft_rekey_cow_stop marks the children whose state
		 * words it parks into.  Released by the re-parent's {live_state ->
		 * live_state} STATE edge, which is recorded unconditionally -- NOT by
		 * the pso edge, which since @118245b0 is its own word and is recorded
		 * only when the slot index CHANGES (resting the release on it leaks a
		 * permanent LOCK on every child that lands at the same index; see
		 * ft_rekey_cow_stop's release-attribution note).  Every bail path
		 * releases it through the @marks sweep.  An external child has no state word and
		 * no metadata, so there is nothing to mark and nothing to clobber.
		 */
		{
			struct cds_ft_compressed_node *cn =
				ft_compressed_node_ptr(cn_flag);
			struct cds_ft_metadata *cm =
				ft_child_state_meta(ft, cn->child);

			if (cm) {
				/*
				 * The DST descent dates this one, and @cn IS its
				 * cursor -- so the displaced child is the cursor's
				 * immediate child, the below-cursor case the window
				 * cannot name (§7.1).  Dating it from the SRC
				 * descent asked the wrong path entirely; that was
				 * inert only because per-node reads no depth at all.
				 */
				struct ft_lock_ctx dctx;
				unsigned int cd;

				ft_lock_ctx_init(&dctx, &d_dst, txn);
					/*
				 * The REST of the op's held set: the cow_stop
				 * marks reach no registry until the sweep, and
				 * the glue holds the split-CN fence.  Under a
				 * coarse spacing this child anchors onto one of
				 * them, and a frame naming neither refuses a word
				 * the op took two steps earlier.  @nr_extra is
				 * read at the acquire because this same array is
				 * still growing (the mark lands at @nr_marks).
				 */
				dctx.held.extra = marks;
				dctx.held.nr_extra = nr_marks;
				dctx.held.glue = &glue;
				if (!ft_lock_ctx_depth_of(ft, &dctx, cn->child,
						&cd)) {
					struct cds_ft_inode_flag *lp = NULL;

					(void) ft_resolve_parent_slot(cm, ft,
						&lp);
					if (!ft_lock_ctx_depth_of_cursor_child(
							ft, &dctx, lp, &cd)) {
						ret = -EAGAIN;
						goto bail_build;
					}
				}
				if (ft_acquire_member(ft, &dctx, cn->child, cm,
						cd, &marks[nr_marks])) {
					ret = -EAGAIN;
					goto bail_build;
				}
				nr_marks++;
				/*
				 * Tell the glue we already hold this one.  The split
				 * build DEFERS this same child, and
				 * ft_glue_acquire_reparent_marks marks every deferred
				 * entry -- without this it would fail against our own
				 * fence and bail -EAGAIN on every attempt.  Read-only
				 * to the glue: @marks below stays its sole release.
				 */
				glue.caller_holder = cm;
			}
		}
		/*
		 * The NOSPLIT path passes the moved subtree's key count through
		 * ft_store_at_graft_point_commit; the GLUE publish takes it here, and
		 * ft_glue_txn_commit_edges records the +count walk from the stable
		 * publish parent into the same commit.
		 */
		glue.count_delta = (long) cnt;
		cds_ft_alloc_reserve_drain(ft, &reserve);	/* GLUE builds its own cluster */
	} else {
		/*
		 * Drive prepare + commit SEPARATELY (not the combined
		 * ft_store_at_graft_point wrapper) so the reserve recompaction's
		 * relocated old dst-parent copy (@gst_st.old_recompacted_node) is
		 * visible here: the graft ALWAYS relocates the attach node for its
		 * atomic publish, and under record_only its old copy stays LIVE until
		 * the caller's commit, so the caller frees it post-commit.
		 */
		cds_ft_alloc_reserve_activate(ft, &reserve);
		{
			/*
			 * The op's outstanding marks -- ft_rekey_cow_stop's @stop
			 * fence and one per COW'd child -- reach no registry until
			 * the sweep below, so the store's own recompactions can
			 * only see them through this frame.  Under a coarse spacing
			 * they collapse onto one word and the store refuses its own
			 * fence: the CDS_FT_STATUS_BUSY_ERROR that "is not expected
			 * single-threaded".
			 */
			struct ft_lock_ctx octx;

			ft_lock_ctx_init(&octx, &d_src, txn);
			octx.held.extra = marks;
			octx.held.nr_extra = nr_marks;
			gst = ft_store_at_graft_point_prepare(ft, dst_ord,
				dst_len, &d_dst, s_top_prime, cnt, &glue,
				&octx.held, &gst_st);
		}
		if (gst == CDS_FT_STATUS_OK)
			gcst = ft_store_at_graft_point_commit(ft, &attached_nf, &adepth,
					NULL /*run*/, &gst_st, (long) cnt);
		cds_ft_alloc_reserve_deactivate(ft);
		cds_ft_alloc_reserve_drain(ft, &reserve);
		if (gst != CDS_FT_STATUS_OK || gcst != URCU_TXN_STATUS_OK) {
			/*
			 * Not expected single-threaded with the reserve pre-filled.
			 * Record-only commit leaves the shared txn intact (terminal commit
			 * gated off), so the caller owns cleanup: free S_top', abort the
			 * glue build, destroy the txn.  (prepare failure freed its own
			 * invisible build + left glue clean.)
			 */
			/* NULL on the merge path: no COW */
			ft_rekey_free_stop_prime(ft, s_top_prime);
			ft_glue_abort(ft, &glue);
	if (src_glue_live) {		/* merged cluster's src side */
		ft_glue_abort(ft, &src_glue);
		src_glue_live = false;
	}
			ft_flip_txn_destroy(txn);
			ret = -EIO;
			goto sweep;
		}
	}

	/*
	 * 3. Detach-fold: remove S_top from BP (clear its slot + nr_child--).  In the
	 * default (concurrent-safe) build EVERY popcount delete recompacts BP, and
	 * that recompaction republishes into BP's parent.  Two shapes, decided by the
	 * gate above and carried by @src_parent_held:
	 *   - BP's parent IS the spine ancestor the graft's dst-parent recompaction
	 *     already holds the lock (both junctions are its children).  REUSE the held
	 *     lock; re-acquiring it would abort -EAGAIN.
	 *   - BP's parent is a node this op holds nothing on.  The recompaction
	 *     acquires and releases it itself, in its own up-front lock-set commit.
	 * Either way the identity is passed EXPLICITLY, and its slot with it, rather
	 * than letting the recompaction resolve BP's current parent: that resolve is
	 * racy, and a peer that re-homed BP since this descent would make "BP's
	 * parent" a node this op does NOT hold -- an SW park into a slot that no
	 * longer holds BP.  @parent_guard puts the BP.parent == @parent read-set
	 * guard on the acquire commit in BOTH shapes (the held arm guards
	 * unconditionally), so a re-home ABORTS it (-EAGAIN, trie pristine) and the
	 * caller re-descends.  BP's parent is never compressed (the src descent
	 * rejects compressed nodes at every level it walks), so no SKIP_X dual and no
	 * @gp member.
	 */
	ft_lock_ctx_init(&lctx_src, &d_src, NULL);
	/*
	 * The op's marks so far -- ft_rekey_cow_stop's @stop fence and one per
	 * COW'd child -- reach no txn registry until the sweep below, so the
	 * detach's own acquires can only see them through this frame.  Under a
	 * coarse spacing S_top's fence lands on BP, which is exactly the node
	 * this detach recompacts.
	 */
	lctx_src.held.extra = marks;
	lctx_src.held.nr_extra = nr_marks;
	/*
	 * And the GLUE, which holds the rest -- the publish-parent fence above
	 * all.  Under a coarse spacing that fence and BP's recompaction are ONE
	 * word, so without this the detach refuses a fence this op took three
	 * steps earlier.
	 */
	lctx_src.held.glue = &glue;
	ret = ft_detach_node(ft, &lctx_src, d_src.nfp, d_src.pnfp, d_src.depth,
			false /*free_detached_subtree: S_top is retired by cow_stop*/,
			NULL /*fuse_cell: list off*/, &pub, NULL /*run*/,
			NULL /*retire_glue*/, NULL /*freeze_leaf*/,
			-(long) cnt, txn /*shared_txn*/, true /*record_only*/,
			&(const struct ft_parent_hint){	/* BP's parent: held or acquired */
				.parent = d_src.ppnf, .slot = d_src.pnfp,
				.gp = NULL, .gp_slot = NULL,
				.parent_held = src_parent_held,
				.parent_guard = true },
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
		 * REACHABLE: a peer holding BP -- or, in the non-shared-parent shape,
		 * BP's own parent -- makes the detach's up-front lock-set acquire abort
		 * -EAGAIN right here, as does a peer that re-homed BP since this
		 * descent (the @parent_guard read-set validation).  The
		 * single-threaded route -- a same-junction move (BP == the graft's
		 * own attach node, hence already LOCK-held) -- is excluded: the shape
		 * gate rejects it up front, before any of this is built.
		 */
		pp_meta = NULL;		/* ft_glue_abort below is the single owner */
		/* NULL on the merge path: no COW */
		ft_rekey_free_stop_prime(ft, s_top_prime);
		if (gst_st.old_recompacted_node)
			free_cds_ft_node_unpublished(ft, ft_node_ptr(gst_st.dest));
		ft_glue_abort(ft, &glue);
			if (src_glue_live) {	/* merged cluster's src side */
				ft_glue_abort(ft, &src_glue);
				src_glue_live = false;
			}
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
	 * structural folds' order (the graft acquires its lock set before the detach
	 * reuses or acquires BP's parent) can't provide that, so the cells are
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
		if (run_keeps_pos) {
			/*
			 * The run keeps its ordered position (the dst abuts it), so there
			 * is nothing to splice -- but "nothing changed" still has to be
			 * ASSERTED at the decide.  Between the derivation above and this
			 * commit a peer can splice a key into the run's own boundary, and
			 * the new keys would then sort on the wrong side of it: a list
			 * left well-formed and no longer key-ordered, with no edge of this
			 * commit noticing.  On the moving path the six edges are what
			 * notices; here two VALIDATE edges (old == new) on the run's OUTER
			 * links do it, so such a peer aborts this commit clean and the
			 * caller re-derives.
			 *
			 * The run's own links rather than A's and B's: a peer may REMOVE a
			 * neighbour, and an edge on a dying cell's slot is worse than one
			 * on a cell this move already owns.  Distinct by construction --
			 * ->prev and ->next are different fields even for a one-cell run.
			 */
			cedges[0].tag = URCU_TXN_TAG;
			cedges[0].slot = (struct ft_ord_cell **) &rfc->lnode.prev;
			cedges[0].old_target = src_pred;
			cedges[0].new_target = src_pred;
			cedges[1].tag = URCU_TXN_TAG;
			cedges[1].slot = (struct ft_ord_cell **) &rlc->lnode.next;
			cedges[1].old_target = src_succ;
			cedges[1].new_target = src_succ;
			ft_ord_cell_record_into(txn, cedges, 2);
			goto cells_done;
		}
#ifdef FEATURE_FT_MERGE
		if (run_interleaves) {
			/*
			 * INTERLEAVE: every relink is recorded, so the cells reorder
			 * with the structural publish in the one commit.  The edge set
			 * is unbounded in the run length, hence heap-allocated and
			 * freed here; a refusal is a clean pre-commit bail like the
			 * distinct-slot one below.
			 */
			struct ft_ord_cell_edge *iedges = NULL;
			unsigned int in = 0;
			int iret = ft_rekey_ord_interleave(ft, probe_D, dst_len,
					src_len, run_rfirst, run_rlast,
					merged_keys, &iedges, &in);

			if (!iret && !ft_flip_txn_reserve_extra(txn, in)) {
				free(iedges);
				iedges = NULL;
				iret = -ENOMEM;
			}
			if (iret) {
				pp_meta = NULL;	/* ft_glue_abort: single owner */
				ft_rekey_free_stop_prime(ft, s_top_prime);
				if (gst_st.old_recompacted_node)
					free_cds_ft_node_unpublished(ft,
						ft_node_ptr(gst_st.dest));
				if (detach_rc.new_flag)
					free_cds_ft_node_unpublished(ft,
						ft_node_ptr(detach_rc.new_flag));
				ft_glue_abort(ft, &glue);
				if (src_glue_live) {
					ft_glue_abort(ft, &src_glue);
					src_glue_live = false;
				}
				ft_flip_txn_destroy(txn);
				ret = iret;
				goto sweep;
			}
			ft_ord_cell_record_into(txn, iedges, in);
			free(iedges);
			goto cells_done;
		}
#endif /* FEATURE_FT_MERGE: an occupied dst is a merge */
		if (src_pred == ft_ord_or_sentinel(ft, run_dpred) ||
				src_succ == ft_ord_or_sentinel(ft, run_dsucc)) {
			pp_meta = NULL;		/* ft_glue_abort: single owner */
			/* NULL on the merge path: no COW */
			ft_rekey_free_stop_prime(ft, s_top_prime);
			if (gst_st.old_recompacted_node)
				free_cds_ft_node_unpublished(ft,
					ft_node_ptr(gst_st.dest));
			if (detach_rc.new_flag)
				free_cds_ft_node_unpublished(ft,
					ft_node_ptr(detach_rc.new_flag));
			ft_glue_abort(ft, &glue);
			if (src_glue_live) {	/* merged cluster's src side */
				ft_glue_abort(ft, &src_glue);
				src_glue_live = false;
			}
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
cells_done:
		;
	}

	/*
	 * 3c. GLUE shape only: record the split cluster's publish -- its hidden
	 * back-pointers (plain stores on unpublished nodes), the displaced child's
	 * live re-parent, the forward edge into the fenced publish parent, @cn's
	 * retire and the count walk -- into the SAME txn.  It runs LAST, after the
	 * detach and cell folds, because it is the step that does live bookkeeping
	 * and is specified to be called where abort is impossible; every arm that
	 * can still bail is above it.  (The NOSPLIT shape is the other way round --
	 * its prepare/commit is where its recompaction ACQUIRES the lock set the
	 * detach then reuses, so it has to run first.)  Under record_only this
	 * records and returns OK without committing; the caller's one commit below
	 * publishes it, and from here the txn registry -- not this function -- owns
	 * the publish parent's fence.
	 */
	if (prep == FT_GRAFT_PREP_GLUE || merge_dst) {
		/*
		 * The status is CHECKED, not discarded: the fold's re-parent mark
		 * acquire runs inside this call, before it records anything, and CAN
		 * miss on a contended child.
		 * Discarding that would carry on to the commit below with children this
		 * op does not hold, SW-parking their state words unexcluded: precisely
		 * the silent-success shape (ignored commit status, op reports OK) that
		 * cost a whole source subtree once already.
		 *
		 * The miss is clean: it happens before the first record and before
		 * ft_glue_tombstone_free_list, so nothing of this cluster is in @txn and
		 * NOTHING is published -- the txn still carries only the detach and cell
		 * records, which this bail discards with it.  @pp_meta ownership has NOT
		 * transferred (that happens by recording), so we still owe its release
		 * and must not NULL it here.
		 */
		if (ft_glue_txn_commit_edges(ft, &glue, NULL, 0) !=
				URCU_TXN_STATUS_OK) {
			/*
			 * ☠ DO NOT release @pp_meta here.  An earlier version did,
			 * reasoning that a failure could only come from the mark
			 * acquire at the top of ft_glue_txn_commit_edges -- before
			 * any record, so before ownership passes to the txn.  That
			 * is an ARGUMENT about which failure happens, and under a
			 * SHARED destination it is false: commit_edges also fails
			 * AFTER routing the publish parent through the txn, which
			 * NULLs g->publish_parent_holder and takes the fence with
			 * it.  Clearing it then asserts on an unheld word.
			 *
			 * ft_glue_abort below is the choke point and already does
			 * the right thing either way -- clear_IF_HELD, guarded on
			 * the holder field the transfer NULLs.  Just disown it so
			 * bail_build's own clear cannot double up.
			 */
			pp_meta = NULL;
			/* NULL on the merge path: no COW */
			ft_rekey_free_stop_prime(ft, s_top_prime);
			if (detach_rc.new_flag)
				free_cds_ft_node_unpublished(ft,
					ft_node_ptr(detach_rc.new_flag));
			ft_glue_abort(ft, &glue);
			if (src_glue_live) {	/* merged cluster's src side */
				ft_glue_abort(ft, &src_glue);
				src_glue_live = false;
			}
			ft_flip_txn_destroy(txn);
			ret = -EAGAIN;
			goto sweep;
		}
		pp_meta = NULL;		/* ownership transferred to @txn */
		/*
		 * The merged cluster's SRC side has no publish of its own -- only
		 * retires, S_top's among them -- so it never goes through
		 * ft_glue_txn_commit_edges.  Record its freezes into the shared txn
		 * directly, here, on the committing path and after the last bail above,
		 * exactly where the dst side's own tombstone step just ran.
		 */
		if (src_glue_live)
			ft_glue_tombstone_free_list(&src_glue);
		/*
		 * The dup-chain appends, recorded (not stored) so the src duplicates
		 * become reachable ATOMICALLY with the merged structure -- a collided
		 * key never momentarily shows only its dst side.  After the cluster's
		 * edges, exactly as ft_merge_spine_copy orders it.
		 */
#ifdef FEATURE_FT_MERGE
		if (merge_dst)
			ft_glue_record_splices(ft, &glue, txn);
#endif
	}

	/* 4. ONE commit of the whole stitch (consumes txn). */
#ifdef FEATURE_FT_FAULT_INJECT
	/*
	 * Test-only: abort this commit exactly as a peer winning a raced MW slot
	 * would.  Without it the fold's abort branch below is DEAD -- and not for
	 * want of contention: the shared-destination merge oracle drove 62342
	 * publish-parent fence misses and 1079 commit_edges misses over 71218
	 * merges and still took this exit 0 times, because every acquire is AHEAD
	 * of the commit and turns the peer away first.  @acquire_miss is the
	 * engine's own discard-unpublished route, so what runs below is the real
	 * unwind, not a synthesised status.
	 */
	if (merge_dst && cds_ft_fault_commit_countdown >= 0) {
		if (cds_ft_fault_commit_countdown == 0) {
			cds_ft_fault_commit_countdown = -1;
			txn->acquire_miss = true;
		} else {
			cds_ft_fault_commit_countdown--;
		}
	}
#endif
	/*
	 * The marks' anchor releases.
	 *
	 * `marks_consumed` below claims the commit consumed every mark, and its
	 * two justifications -- a child's {live_state -> live_state} re-parent
	 * edge, @stop's retire -- are claims about each NODE's OWN word.  Under a
	 * coarse spacing the word the acquire TOOK is the node's ANCHOR, and no
	 * node terminal touches it: without this the LOCK survives the commit and
	 * leaks, so the next op to anchor there refuses it forever and a later
	 * release of it asserts.
	 *
	 * Placed here because this is past the LAST acquire -- @nr_marks is still
	 * growing up to the displaced-child mark above -- NOT for ordering:
	 * ft_flip_txn_record_anchor_release_held reads the word through the txn,
	 * so it chains onto whatever the op has already recorded there and is
	 * order-independent by construction (measured: recording it right after
	 * ft_rekey_cow_stop is equally green).  Self-guarding too, so an
	 * UNcoarsened mark -- whose node terminal IS its release -- records
	 * nothing and the default granularity stays byte-identical.
	 */
	for (i = 0; i < nr_marks; i++) {
		if (marks[i].shared)
			continue;	/* an earlier acquire owns its release */
		if (!ft_flip_txn_reserve_extra(txn, 1)) {
			ret = -ENOMEM;
			goto bail_build;
		}
		ft_flip_txn_record_anchor_release_held(txn, marks[i].lock);
	}
	st = ft_flip_txn_commit(ft, txn);
	if (st == URCU_TXN_STATUS_OK) {
		/*
		 * The commit CONSUMED every mark in @marks: each child's release is
		 * ft_reparent_record_meta's {live_state -> live_state} state edge
		 * (live_state has LOCK masked, and the edge is recorded
		 * unconditionally), and @stop's is its retire.  So the sweep below
		 * must NOT run -- see its own comment.
		 */
		marks_consumed = true;
		{	/* TEMPORARY PROBE: did the commit really settle every mark? */
			unsigned int k;

			for (k = 0; k < nr_marks; k++) {
				uintptr_t st2;

				if (marks[k].shared)
					continue;
				st2 = CMM_LOAD_SHARED(marks[k].lock->state);
				if (st2 & FT_STATE_LOCK) {
					static __thread unsigned long _n;

					if (_n++ < 8)
						fprintf(stderr,
							"[POST] mark %u/%u word=%p STILL LOCKED state=%lx\n",
							k, nr_marks,
							(void *) marks[k].lock,
							(unsigned long) st2);
				}
			}
		}
		ft_glue_free_old(ft, &glue);		/* graft old copies */
		/*
		 * The merged cluster's SRC side: its free list holds S_top itself (and
		 * any src overlap node the build copied), retired by this commit.  That
		 * is what makes the cds_ft_free_item_deferred below a MERGE-path double
		 * free -- the merge already owns S_top's reclaim, so only the cow_stop
		 * path still owes it.
		 */
		if (src_glue_live)
			ft_glue_free_old(ft, &src_glue);
		/*
		 * Chain holders are NOT in the txn registry, so the commit does not
		 * consume them: release them here, the committed path's own point.
		 * Every non-committed path reaches ft_glue_abort, which owns the same
		 * release -- the choke point, not the exits that happen to be visible.
		 */
		ft_glue_release_splice_holders(&glue);
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
		/*
		 * Old S_top after the grace period -- but ONLY on the cow_stop path.
		 * The merge retires S_top through @src_glue's free list, so
		 * ft_glue_free_old above already owns that reclaim; doing it here too
		 * is a double free.
		 */
		if (!merge_dst)
			cds_ft_free_item_deferred(ft, s_top_meta);
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
		/* NULL on the merge path: no COW */
		ft_rekey_free_stop_prime(ft, s_top_prime);
		if (gst_st.old_recompacted_node)
			free_cds_ft_node_unpublished(ft, ft_node_ptr(gst_st.dest));
		if (detach_rc.new_flag)
			free_cds_ft_node_unpublished(ft, ft_node_ptr(detach_rc.new_flag));
		ft_glue_abort(ft, &glue);
			if (src_glue_live) {	/* merged cluster's src side */
				ft_glue_abort(ft, &src_glue);
				src_glue_live = false;
			}
		ret = -EAGAIN;
	}
	ft_glue_fini(&glue);
	if (src_glue_live)
		ft_glue_fini(&src_glue);
	goto sweep;

bail_build:
	/*
	 * Common unwind for every bail between the graft build and the point where
	 * the graft's own step has committed its records: the build published
	 * nothing, so drop the (possibly filled) node reserve, release the publish
	 * parent's fence if this function still owns it -- once
	 * ft_glue_txn_commit_edges has run, the txn registry owns it and clearing
	 * here would race a peer's re-mark -- free the unpublished S_top' copy, and
	 * let ft_glue_abort free the invisible cluster and release the split node's
	 * fence (its single clear point).  @marks is swept below, as on every path.
	 */
	cds_ft_alloc_reserve_drain(ft, &reserve);
	/*
	 * The publish-parent fence has EXACTLY ONE owner on this path, and it is
	 * ft_glue_abort below: @glue.publish_parent_holder is set in the same breath
	 * as @pp_meta, ft_glue_abort clears it IF HELD, and the txn NULLs that field
	 * when a record transfers ownership.  A second clear here was right only
	 * while every bail above was known to still hold the fence -- under a SHARED
	 * destination that stopped being true, and it asserted on an unheld word.
	 * Disown and let the choke point do it.
	 */
	pp_meta = NULL;
	/* NULL on the merge path: no COW */
	ft_rekey_free_stop_prime(ft, s_top_prime);
	ft_glue_abort(ft, &glue);
			if (src_glue_live) {	/* merged cluster's src side */
				ft_glue_abort(ft, &src_glue);
				src_glue_live = false;
			}
	ft_flip_txn_destroy(txn);

sweep:
	/*
	 * Release every mark the commit did NOT consume.
	 *
	 * ONLY on paths that did not reach a successful commit.  This used to run
	 * unconditionally, on the reading that a consumed mark makes
	 * clear_if_held "a no-op" -- true only with no peers.  After a successful
	 * commit each of these nodes is LIVE and CLEAN, so a peer that re-marked
	 * one in the commit->sweep window has its fence CLEARED here: fence theft,
	 * the same shape as the publish-parent holder's, and the precise thing
	 * ft_glue_release_reparent_marks' own contract ("call ONLY on paths that
	 * did NOT reach a successful commit") exists to avoid.  The fold is not
	 * concurrent yet, which is why this was invisible; it has to be right
	 * before it is.
	 */
	if (!marks_consumed)
		for (i = 0; i < nr_marks; i++)
			if (!marks[i].shared)
				ft_meta_lock_release_if_held(marks[i].lock);
	return ret;
}

/*
 * Retry wrapper: the escalation lane the fold never had.
 *
 * Every bail in the attempt above is abort-clean (the trie is byte-for-byte as
 * before), so retrying is just calling again -- but calling again is not enough
 * on its own.  Progress under contention comes from AGING a PERSISTENT handle:
 * urcu_txn_conflict() advances @optxn->retry, and once it reaches the fallback
 * budget the writer takes its FIFO turn on the trie's escalation domain and
 * commits without competition.  A fresh handle per attempt -- which is what the
 * caller's external "just call again" loop produced -- resets that age to zero
 * every time, so the writer never qualifies and spins instead.
 *
 * SCOPE: this arbitrates COMMITS.  It deliberately does NOT wrap the per-node
 * LOCK acquires -- an escalated acquirer would hold its FIFO turn while
 * spinning for a holder that is itself funnelled behind that turn (the circular
 * wait documented at the FT-wide writer lock).  An acquire miss stays a clean
 * bail that re-descends.
 *
 * -EINVAL (shape) and -ENOMEM are terminal; the transient contention codes the
 * attempt documents (-EAGAIN, -EIO) are what this loop absorbs.
 */
static
int ft_rekey_graft_simple_locked(struct cds_ft *ft,
		const uint8_t *src_key, size_t src_len,
		const uint8_t *dst_key, size_t dst_len, bool require_empty)
{
	const struct rcu_flavor_struct *flavor = ft->group->flavor;
	struct urcu_txn optxn;
	int ret;

	ft_txn_op_init(ft, &optxn);
	/*
	 * PARK THE ESCALATION QUIESCENT, which this op may do and insert / remove
	 * may not.  urcu_txn_begin escalates -- and therefore blocks -- before it
	 * opens the txn's own read section, so the only thing that could be pinned
	 * across that park is an RCU section the CALLER holds.  This entry has none:
	 * cds_ft_rekey_graft already forbids being called from a read section (its
	 * move gate waits for a grace period), and the per-attempt pin below is
	 * taken AFTER begin.  A writer parked online is what stops every grace
	 * period in the process, so quiescing here is what keeps a contended move
	 * from wedging the peers that wait on one.
	 */
	urcu_txn_set_park_quiescent(&optxn, 1);
	for (;;) {
		urcu_txn_begin(&optxn);
		/*
		 * PER ATTEMPT, not around the loop.  The pin exists to keep the
		 * nodes ONE attempt captures alive from descent through commit, and
		 * each attempt re-descends -- so per-attempt is both sufficient and
		 * what leaves begin's park unpinned.
		 */
		flavor->read_lock();
		ret = ft_rekey_graft_simple_attempt(ft, src_key, src_len,
			dst_key, dst_len, require_empty, &optxn);
		flavor->read_unlock();
		if (ret != -EAGAIN && ret != -EIO)
			break;
		/* Age the conflict, as cds_ft_replace does; the turn is forfeited. */
		urcu_txn_conflict(&optxn);
		urcu_txn_end(&optxn);
	}
	urcu_txn_end(&optxn);
	return ret;
}

/*
 * The MOVE entry point: bracket the move in the mode gate, then run it under our
 * OWN read lock.
 *
 * Order matters and is the gate's whole purpose: ft_move_gate_enter publishes
 * "expect a move" to readers and waits a grace period, so every reader still in a
 * critical section has finished before the body below mutates anything -- readers
 * that start after it see the gate and switch to the coherent path.  A burst of
 * concurrent moves pays ~one grace period in total (they piggyback the first).
 *
 * CALLER CONTRACT (new, and inherent to the gate): a move BLOCKS on a grace
 * period, so it must NOT be called from inside an RCU read-side critical section
 * -- the GP would wait for the caller's own section.  This entry takes the read
 * lock the body needs itself, AFTER the gate.  The gate is entered before the
 * shape gates run, so a rejected move also pays the GP; the public entry
 * (ft_rekey_one_decide's caller) cheap-checks the shape first instead.
 *
 * An occupied destination MERGES here (@require_empty false), which is what the
 * oracles driving this entry expect; the graft-semantics refusal belongs to the
 * public entry that documents it.
 */
int _cds_ft_debug_rekey_graft_simple(struct cds_ft *ft,
		const uint8_t *src_key, size_t src_len,
		const uint8_t *dst_key, size_t dst_len)
{
	int ret;

	ft_move_gate_enter(ft);
	ret = ft_rekey_graft_simple_locked(ft, src_key, src_len, dst_key, dst_len,
			false);	/* pins per attempt: see the locked wrapper */
	ft_move_gate_exit(ft);
	return ret;
}

/*
 * The ATOMIC rekey, for the public entry points: move @src_key's subtree to
 * @dst_key as ONE decide, or report that this shape is not one it covers.
 *
 * Everything the caller needs to know is in the return code.  0 committed the
 * move atomically -- a reader sees the subtree at the source XOR the
 * destination, with no instant where it is at neither, which is the property
 * the staged writer (a committed detach, then a merge back) cannot provide.
 * -EINVAL means the shape is outside this writer's cut and the caller should
 * fall back; -EEXIST is a GRAFT caller's occupied destination; -ENOMEM and
 * -ENOTSUP are terminal.  The transient contention codes never surface: the
 * retry wrapper absorbs them.
 *
 * It takes the READ LOCK the body needs but NOT the move gate, so one gate
 * bracket in the caller covers this attempt and any fallback -- two brackets
 * would pay two grace periods for one move.  The caller must therefore already
 * hold the gate and, by the gate's own contract, not be inside a read section.
 *
 * Hidden-visibility here rather than static because the public entries live in
 * ft-merge.h, which is included ABOVE the composition this wraps (it needs
 * detach, graft and merge all in scope); the declaration in
 * fractal-trie-internal.h is what bridges that.
 */
__attribute__((visibility("hidden")))
int ft_rekey_one_decide(struct cds_ft *ft,
		const uint8_t *src_key, size_t src_len,
		const uint8_t *dst_key, size_t dst_len, bool require_empty)
{
	/*
	 * NO read lock here: ft_rekey_graft_simple_locked pins PER ATTEMPT, which
	 * is what lets its escalation park quiesce.  Wrapping the loop instead --
	 * which this did -- pinned the park and made a contended move able to wedge
	 * every peer waiting on a grace period.
	 */
	return ft_rekey_graft_simple_locked(ft, src_key, src_len, dst_key, dst_len,
			require_empty);
}

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
