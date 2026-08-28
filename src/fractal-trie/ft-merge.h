// SPDX-FileCopyrightText: 2012-2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-only

/*
 * src/fractal-trie/ft-merge.h
 *
 * Userspace RCU library - Fractal Trie: merge / merge_at (spine-copy, same-trie rekey).
 *
 * Implementation unit: #included once by fractal-trie.c, in dependency
 * order, into a single translation unit (preserves cross-module inlining).
 * Not a standalone header.
 */
#ifndef FRACTAL_TRIE_IMPL
#error "ft-merge.h is an implementation unit; #include it from fractal-trie.c only"
#endif

/*
 * Same-trie rekey mode threaded into ft_merge_at_inner.  FT_REKEY_NONE is a
 * plain CROSS-trie cds_ft_merge_at (src_ft != dst_ft enforced at entry).
 * FT_REKEY_MERGE / FT_REKEY_GRAFT are the cds_ft_rekey_merge / cds_ft_rekey_graft
 * entry points (src_ft == dst_ft == the trie): GRAFT additionally REQUIRES an
 * empty destination (POPULATED_ERROR otherwise, mirroring cds_ft_graft), MERGE
 * unions into an occupied one (mirroring cds_ft_merge_at).
 */
enum ft_rekey_mode {
	FT_REKEY_NONE = 0,
	FT_REKEY_MERGE,
	FT_REKEY_GRAFT,
};

/*
 * ★ OUTSIDE THE MERGE GUARD ON PURPOSE.  This is REKEY's vocabulary, not
 * merge's: with -DNO_FEATURE_FT_MERGE the rekey GRAFT stays available (only the
 * occupied-destination MERGE mode goes away), so FT_REKEY_NONE / FT_REKEY_GRAFT
 * must still be declared.  FT_REKEY_MERGE remains declared too -- the arms that
 * ACT on it are what the feature guard removes, and a mode nobody can reach
 * costs nothing.
 */

/*
 * ★ SHARED INFRASTRUCTURE, OUTSIDE THE MERGE GUARD.  A read-only descent and a
 * source-subtree unlink: the REKEY paths that survive -DNO_FEATURE_FT_MERGE
 * (only the occupied-destination merge mode goes away) call both, so they are
 * not part of the feature.  Neither calls another merge-internal helper, which
 * is what makes the split at this line exact.
 */
/*
 * Read-only locate of a merge point at the end of @key.  Reuses
 * ft_graft_swap_descend (the same three outcomes, publishes nothing) and
 * additionally reports the cursor's compressed offset and the subtree's key
 * count, both needed by cds_ft_merge_at:
 *
 *   FT_GRAFT_SWAP_EXACT:       @d->nf is the live subtree at @key; @off_ret 0;
 *                              @count_ret its nr_keys (1 for an external / dup
 *                              chain -- one unique key).
 *   FT_GRAFT_SWAP_KEY_SHORTER: @key ends inside compressed @d->nf; @off_ret =
 *                              key_len - d->depth (bytes consumed into the
 *                              node); @count_ret the node's subtree nr_keys.
 *   FT_GRAFT_SWAP_DELEGATE:    no content under @key (src side -> the merge is
 *                              a no-op; dst side -> the atomic fast-path graft).
 *
 * @off_ret / @count_ret are 0 for DELEGATE.
 */
static
enum ft_graft_swap_case ft_merge_descend(struct cds_ft *ft,
		const uint8_t *key, size_t key_len, struct ft_descent *d,
		unsigned int *off_ret, unsigned long *count_ret)
{
	struct cds_ft_inode_flag *raw;	/* the merge has no forward-edge plan to quote it */
	enum ft_graft_swap_case kase = ft_graft_swap_descend(ft, key, key_len, d,
			&raw);

	switch (kase) {
	case FT_GRAFT_SWAP_KEY_SHORTER:
		*off_ret = (unsigned int) (key_len - d->depth);
		/* Whole subtree of the compressed node the short key lands in. */
		*count_ret = ft_node_key_count(ft, d->nf);
		break;
	case FT_GRAFT_SWAP_EXACT:
		*off_ret = 0;
		/* External -> one key (possibly a dup chain); else subtree count. */
		*count_ret = ft_node_key_count(ft, d->nf);
		break;
	default:	/* FT_GRAFT_SWAP_DELEGATE */
		*off_ret = 0;
		*count_ret = 0;
		break;
	}
	return kase;
}

/*
 * Unlink the EXACT subtree at @src_key from @src_ft IN PLACE, preserving the
 * subtree node so the spine-copy merge can keep referencing it (it is
 * re-parented into the merged cluster at commit).  This is the non-root-src
 * analogue of the root-src ft->root swap: it removes the merge source from
 * @src_ft and prunes the now-empty single-child branch above it.
 *
 * The descent + ft_detach_node + chain reclaim mirror ft_detach_keylen's
 * non-root path, but with @free_detached_subtree = false and WITHOUT wrapping
 * the subtree in a transient trie (the merge owns it via @gd's referenced
 * edges; wrapping it would double-own it).  @detached_count is the subtree's
 * unique-key count (from ft_merge_descend), propagated out of the ancestors.
 *
 * The caller invokes this as the LAST fallible commit step: on -ENOMEM
 * (ft_detach_node recompaction failed) @src_ft is left pristine, so the caller
 * aborts the still-invisible build with both tries intact -- no rollback.  On
 * success the caller drains @src_ft and runs the failure-free commit tail.
 */
static
int ft_merge_unlink_src_subtree(struct cds_ft *src_ft,
		const uint8_t *_src_key, size_t src_key_len,
		unsigned long detached_count, struct ft_detach_run *run,
		struct ft_glue *retire_glue)
{
	/*
	 * @src_key is ALREADY ORDINAL (cds_ft_merge_at converts once at its
	 * entry); a second key-map application here would descend a different
	 * subtree than the one the spine build copied.
	 */
	const uint8_t *key = _src_key, *ik;
	struct ft_descent d;
	int ret;

	ik = key;

	/*
	 * Plain key-guided descent to the merge source's subtree root.  As in
	 * ft_detach_keylen, no branch-point snapshot is tracked: ft_detach_node
	 * is bootstrapped from the target's own slot and climbs parent pointers
	 * to the surviving ancestor.  A KEY_SHORTER source (the key ends inside
	 * a compressed node) overshoots that node -- @d.nf becomes its child --
	 * and the climb's free walk reclaims the whole compressed node while
	 * preserving @d.nf, matching the old explicit chain reclaim.
	 */
	ft_descent_init(&d, src_ft);
	for (; d.depth < src_key_len; ) {
		uint8_t kv;

		/* Caller already established EXACT, so the path is present. */
		if (ft_node_compressed(d.nf)) {
			struct cds_ft_compressed_node *cn =
				ft_compressed_node_ptr(d.nf);

			ft_descent_traverse_compressed(src_ft, &d, cn, &ik);
			continue;
		}
		kv = *(ik++);
		ft_descent_step(src_ft, &d, kv);
	}


	/*
	 * Unlink the branch in place, preserving the move target (@d.nf, the
	 * subtree root the spine-copy merge keeps referencing).  Bootstrapped
	 * from the target's own slot, ft_detach_node climbs to the surviving
	 * ancestor and its free-walk phase 1 reclaims the intermediate single-
	 * child chain (compressed / skip-target nodes included) while phase 2 --
	 * which would free the target -- stays gated off for move-style.
	 */
	{
		/*
		 * @run (EXCISE-ONLY, into==NULL) fuses the structural unlink with the
		 * run's removal from src's ordered list in ONE flip -- but the fusion
		 * in ft_detach_node is gated on a non-NULL @pub (the structural-publish
		 * deferral it records and commits alongside the cell edges), so supply
		 * one here exactly as ft_detach's move-style unlink does.  @run NULL
		 * (list off) leaves both NULL = the unfused two-store unlink.
		 */
		struct ft_remove_pub pub = { .armed = false };
		struct ft_remove_pub *pubp = run ? &pub : NULL;

		struct ft_lock_ctx lctx;

		ft_lock_ctx_init(&lctx, &d, NULL,
			retire_glue ? retire_glue->op : NULL);
		ret = ft_detach_node(src_ft, &lctx, d.nfp, d.pnfp, d.depth,
				/*free_detached_subtree=*/ false, NULL, pubp, run,
				retire_glue, NULL,
				/*
				 * Fold the whole-subtree count removal onto the unlink:
				 * -detached_count rides ft_detach_node's own commit (exact
				 * under concurrent writers; magnitude-agnostic leaf machinery).
				 */
				-(long) detached_count, NULL, false, NULL, NULL);
	}
	if (ret < 0) {
		/*
		 * Recompaction OOM: the folded count edges rode the
		 * uncommitted txn (an abort applies nothing), so src is
		 * pristine -- no propagation to undo.
		 */
		return -ENOMEM;
	}
	return 0;
}

#ifdef FEATURE_FT_MERGE


/*
 * cds_ft_merge_at build-invisible spine-copy.  ft_merge_build recursively
 * COPIES the overlapping spine of two subtrees S (from src) and D (from dst),
 * both rooted at the same suffix position, and REFERENCES every disjoint
 * subtree by pointer (recorded as a deferred re-parent edge applied at commit).
 * All allocation happens here, build-invisibly; OOM frees the fresh copies and
 * leaves both tries pristine -- no rollback.
 *
 * Returns the PLAIN flag of the freshly-built merged node (or, when the merged
 * position is leaf-only, the surviving external head), or FT_MERGE_OOM on
 * allocation failure (the caller aborts both glues).  On success *nr_keys_ret
 * holds the merged subtree's unique-key count.
 */
#define FT_MERGE_OOM		((struct cds_ft_inode_flag *) (long) -ENOMEM)

struct ft_merge_ctx {
	struct cds_ft *dst_ft;		/* all fresh merged nodes live here */
	struct ft_glue *gd;	/* dst cluster: built/deferred/dst frees/splices */
	struct ft_glue *gs;	/* src side: src-overlap frees (+ prune later) */
	/*
	 * DLM overlap-spine plan-lock (§9.4 M-2): fence each DST overlap node
	 * before reading its body, and retire it through the fenced terminal.
	 * Set by the caller when the dst trie is lock_fine.
	 */
	bool fence_overlap;
	/*
	 * Fence each SRC overlap node too, and retire it through the fenced
	 * terminal -- the mirror of @fence_overlap for the S side.
	 *
	 * A CROSS-TRIE merge does not need this: cds_ft_merge_at requires the src
	 * exclusive, so no peer can touch S while the build copies it.  The
	 * SAME-TRIE REKEY fold does: it RECORDS its detach into the same txn as
	 * everything else, so its source is still LIVE and reader/writer-reachable
	 * for the whole build window.  Without the fence, the hazard is the one
	 * ft_merge_lock_overlap's header describes for the dst side -- a peer
	 * inserting BELOW S in the copy window is invisible, the late expected-old
	 * still matches, the commit succeeds and the peer's child is retired with
	 * the node.  Silent key loss.
	 *
	 * Set by the caller when its source is live (the rekey fold); left false by
	 * ft_merge_spine_copy.  See feedback_rekey_src_is_live_shared_helpers.
	 */
	bool fence_src;
	/*
	 * Set when a fence acquire MISSED: the build returns FT_MERGE_OOM like any
	 * other failure (one sentinel, one caller unwind), but this distinguishes
	 * CONTENTION from a real allocation failure so the caller can re-descend
	 * instead of reporting MEMORY_ERROR.
	 */
	bool overlap_contended;
	/*
	 * The ABSOLUTE byte-depth each side's merge point sits at, so a fence can
	 * date its node.  ft_merge_build's @depth counts bytes from the merge
	 * point -- it is where the RECURSION is, not where the NODE is -- and an
	 * anchor is a function of the node's absolute depth (§2).  Handing the
	 * relative value to ft_anchor_meta dates the top overlap node as byte-depth
	 * 0, which is not "undated" but THE ROOT, so it anchors on ITSELF while
	 * every op that dates it correctly anchors on an ancestor: two ops, two
	 * words, no exclusion.
	 *
	 * Set at the single construction site of each caller, where the two
	 * descents are in scope.  @off is included because a KEY_SHORTER merge
	 * point sits INSIDE a compressed run: the run starts at @d->depth, the
	 * cursor is @off bytes further, and it is the cursor that relative depth 0
	 * names.  The run itself is not fenced at that frame (a compressed node is
	 * fenced only when entered at offset 0), so no fence reads a base that
	 * skipped its own node's start.
	 */
	unsigned int dst_base_depth;
	unsigned int src_base_depth;
};

/* Upper-bound counters for the read-only pre-pass that sizes the glues. */
struct ft_merge_counts {
	int nb;		/* fresh built nodes (-> gd) */
	int nd;		/* deferred re-parent edges (-> gd) */
	int nf_dst;	/* dst-overlap frees (-> gd) */
	int nf_src;	/* src-overlap frees (-> gs) */
	int ns;		/* duplicate-chain splices (-> gd) */
};

/* nr_keys of a (possibly skip-encoded) child subtree referenced by pointer. */
static
unsigned long ft_merge_child_count(struct cds_ft *ft, struct cds_ft_inode_flag *c)
{
	/*
	 * One for an external leaf; the maintained nr_keys when the trie keeps
	 * order statistics, else a structural recount -- so the merge's count-
	 * driven sizing / short-circuits are correct on a rank-stats-off trie.
	 */
	return ft_node_key_count(ft, c);
}

static
struct cds_ft_inode_flag *ft_merge_build(struct ft_merge_ctx *c,
		struct cds_ft_inode_flag *S, unsigned int off_s,
		struct cds_ft_inode_flag *D, unsigned int off_d,
		unsigned int depth, unsigned int d_prov,
		unsigned long *nr_keys_ret);

/*
 * Advance a compressed cursor one byte past @next_off - 1: while still inside
 * the run report the run's flag at @next_off, else fall through to the run's
 * (internal/external, never compressed) child at offset 0.
 */
static inline
void ft_merge_advance(struct cds_ft_compressed_node *cn, unsigned int next_off,
		struct cds_ft_inode_flag **flag_ret, unsigned int *off_ret)
{
	if (next_off < cn->len) {
		*flag_ret = ft_compressed_node_flag(cn);
		*off_ret = next_off;
	} else {
		*flag_ret = cn->child;
		*off_ret = 0;
	}
}

/*
 * Materialize, build-invisibly, the subtree a single-side compressed run
 * contributes to a fresh branch slot: the bytes cn->key_bytes[off+1 .. len)
 * down to cn's (live) child.  The byte cn->key_bytes[off] is consumed by the
 * parent branch's slot, so this builds the part below it:
 *
 *   suffix_len == 0:                  the bare live child (no fresh node; the
 *                                     parent frame's Pass 2 wires it).
 *   suffix_len == 1 (non-skip):       a 1-child internal node.
 *   suffix_len >= 2, or == 1 (skip):  a fresh compressed node, returned in its
 *                                     skip-published form.
 *
 * Mirrors the old-direction suffix of ft_split_compressed_graft_build.  cn's
 * (live) child back-pointer is deferred into @c->gd with @dst_origin (false
 * for a src run, true for a dst run); a fresh wrapper is tracked there too.
 * @child_depth is the depth of the materialized node itself (parent + 1).
 * Returns the flag to store in the parent slot, or FT_MERGE_OOM.
 */
static
struct cds_ft_inode_flag *ft_merge_materialize_suffix(struct ft_merge_ctx *c,
		struct cds_ft_compressed_node *cn, unsigned int off,
		unsigned int child_depth, bool dst_origin,
		unsigned long *ck_ret)
{
	struct cds_ft *ft = c->dst_ft;
	struct ft_glue *g = c->gd;	/* all fresh merged nodes -> gd */
	unsigned int suffix_len = cn->len - off - 1;
	unsigned long child_keys = ft_merge_child_count(ft, cn->child);
	struct cds_ft_inode_flag **slot;
	int ret;

	*ck_ret = child_keys;
	if (suffix_len >= 2
#ifdef FEATURE_FT_SKIP_COMPRESSED
			|| (suffix_len == 1 && ft_group_skip_compressed(ft->group))
#endif
	   ) {
		struct cds_ft_compressed_node *sfx;
		struct cds_ft_metadata *sfx_meta;
		struct cds_ft_inode_flag *plain;

		sfx = alloc_compressed_node(ft, (uint8_t) suffix_len, &sfx_meta);
		if (!sfx)
			return FT_MERGE_OOM;
		sfx->child = cn->child;
		sfx->len = (uint8_t) suffix_len;
		memcpy(sfx->key_bytes, &cn->key_bytes[off + 1], suffix_len);
		ft_meta_nr_child_set(sfx_meta, 1);
		ft_nr_keys_store(ft, sfx_meta, child_keys, CMM_RELAXED);
		plain = ft_compressed_node_flag(sfx);
		ft_glue_track(g, plain);
		ft_glue_defer_edge_origin(ft, g, cn->child, plain, &sfx->child,
				dst_origin);
		/*
		 * Return the PLAIN flag: a fresh compressed node's skip form
		 * cannot be resolved during the build (its child's back-pointer
		 * is deferred), so the parent stores the plain form -- which
		 * ft_glue_is_fresh matches by direct identity -- and the
		 * parent's Pass 2 re-encodes the slot to the skip form.
		 */
		return plain;
	} else if (suffix_len == 1) {
		/* 1-child internal suffix (non-skip-compressed). */
		struct cds_ft_inode_flag *dest = NULL;

		ret = ft_node_set_nth(ft, &dest, cn->key_bytes[off + 1],
				cn->child, NULL, NULL, child_depth, true);
		if (ret)
			return FT_MERGE_OOM;
		ft_nr_keys_store(ft, cds_ft_item_to_metadata(ft_node_ptr(dest)),
				child_keys, CMM_RELAXED);
		ft_glue_track(g, dest);
		ft_node_get_nth_skip(dest, &slot, cn->key_bytes[off + 1],
				FT_PF_NONE);
		ft_glue_defer_edge_origin(ft, g, cn->child, dest, slot,
				dst_origin);
		return dest;
	}
	/* suffix_len == 0: the bare child; parent Pass 2 wires its edge. */
	return cn->child;
}

/*
 * Both overlap sides are compressed runs sharing a @p (>= 1) byte prefix from
 * their cursors.  Emit ONE fresh compressed run for the shared bytes and
 * recurse on what follows each cursor (the next byte, or the run's child once
 * a run is exhausted).  The shared run's child is internal/external/another
 * fresh branch -- never another compressed -- because a canonical compressed
 * node's child is never compressed, so no two adjacent compresseds result.
 * Returns the run's PLAIN flag (parent Pass 2 re-encodes the slot to skip), or
 * FT_MERGE_OOM.
 */
static
struct cds_ft_inode_flag *ft_merge_build_run(struct ft_merge_ctx *c,
		struct cds_ft_compressed_node *cn_s, unsigned int off_s,
		struct cds_ft_compressed_node *cn_d, unsigned int off_d,
		unsigned int p, unsigned int depth, unsigned long *nr_keys_ret)
{
	struct cds_ft *ft = c->dst_ft;
	struct cds_ft_compressed_node *run;
	struct cds_ft_metadata *run_meta;
	struct cds_ft_inode_flag *adv_s, *adv_d, *child, *plain;
	unsigned int aoff_s, aoff_d;
	unsigned long ck = 0;

	ft_merge_advance(cn_s, off_s + p, &adv_s, &aoff_s);
	ft_merge_advance(cn_d, off_d + p, &adv_d, &aoff_d);
	child = ft_merge_build(c, adv_s, aoff_s, adv_d, aoff_d, depth + p,
			c->dst_base_depth + depth - off_d, &ck);
	if (child == FT_MERGE_OOM)
		return child;

#ifndef FEATURE_FT_SKIP_COMPRESSED
	if (p == 1) {
		/*
		 * Non-skip-compressed: a 1-byte run is a 1-child internal node,
		 * not a len-1 compressed node (matching insert/split canonical
		 * form).  Same node+edge budget as the compressed run.
		 */
		struct cds_ft_inode_flag *dest = NULL, **slot;
		int ret;

		ret = ft_node_set_nth(ft, &dest, cn_s->key_bytes[off_s], child,
				NULL, NULL, depth, true);
		if (ret)
			return FT_MERGE_OOM;
		ft_nr_keys_store(ft, cds_ft_item_to_metadata(ft_node_ptr(dest)), ck,
				CMM_RELAXED);
		ft_glue_track(c->gd, dest);
		ft_node_get_nth_skip(dest, &slot, cn_s->key_bytes[off_s],
				FT_PF_NONE);
		ft_glue_defer_edge_origin(ft, c->gd, child, dest, slot,
				/*dst_origin=*/ true);
		*nr_keys_ret = ck;
		return dest;
	}
#endif
	run = alloc_compressed_node(ft, (uint8_t) p, &run_meta);
	if (!run) {
		/*
		 * The recursion's cluster is already tracked in the glue; the
		 * caller's abort frees it.  Only this frame's run failed.
		 */
		return FT_MERGE_OOM;
	}
	run->child = child;
	run->len = (uint8_t) p;
	memcpy(run->key_bytes, &cn_s->key_bytes[off_s], p);
	ft_meta_nr_child_set(run_meta, 1);
	ft_nr_keys_store(ft, run_meta, ck, CMM_RELAXED);
	plain = ft_compressed_node_flag(run);
	ft_glue_track(c->gd, plain);
	/*
	 * Wire run->child's back-pointer.  A fresh recursion result is stored
	 * immediately (is_fresh); a live result can only be the dst splice head
	 * the recursion returns, so dst_origin is safe either way.
	 */
	ft_glue_defer_edge_origin(ft, c->gd, child, plain, &run->child,
			/*dst_origin=*/ true);
	/*
	 * Return the PLAIN flag (like ft_merge_materialize_suffix): the parent
	 * frame's Pass 2 re-encodes the holding slot to the skip form once the
	 * run is wired, and is_fresh can match it by identity in the meantime.
	 */
	*nr_keys_ret = ck;
	return plain;
}

/*
 * DLM overlap-spine plan-lock (§9.4 M-2): acquire a DST overlap node's LOCK
 * fence BEFORE ft_merge_build reads its body into the merged cluster, so the copy
 * plan and the retire that ratifies it bracket the same world.
 *
 * WHAT IT CLOSES.  The retire was a PLAIN {s -> s|TOMBSTONE} whose expected-old is
 * read fresh at commit, with NO lock over the window in which the build copied the
 * node's children.  A peer adding a child in that window is invisible: the late
 * expected-old still matches, the commit succeeds, and the peer's child is retired
 * along with the node.  Silent key loss, and exactly the "unlocked retire" class
 * the escalation model separates from the lock_or_guard publish sites (which do
 * abort on a peer touch).
 *
 * Returns -EAGAIN on a dirty word (a peer's parked proxy, a concurrent copier, a
 * real retire).  That aborts the whole build: ft_merge_build returns FT_MERGE_OOM,
 * the caller ft_glue_aborts, and ft_glue_clear_fenced releases the marks already
 * taken.  Only DST overlap nodes are fenced -- a cross-trie merge REQUIRES its
 * source exclusive (and the rekey's source is a detach product), so the src side
 * has no peer to exclude and its glue keeps the plain retire.
 */
static inline
int ft_merge_lock_overlap(const struct cds_ft *ft,
		const struct ft_lock_ctx *ctx, struct cds_ft_inode_flag *nf,
		unsigned int depth, void *node, struct ft_held_anchor *held)
{
	struct cds_ft_metadata *m = cds_ft_item_to_metadata(
		(struct cds_ft_inode *) node);

	/*
	 * Hand the WHOLE held anchor back, not just @node's clean word: the
	 * fenced retire this fence pays for is recorded against the word the
	 * acquire LOCKED, which coarsening makes an ancestor of @node.
	 */
	return ft_acquire_member(ft, ctx, nf, m, depth, held);
}

/*
 * @d_prov: ABSOLUTE byte-depth of the OLD dst node whose slot provided @D --
 * the holder of @D when @D is EXTERNAL (a dup-chain head hangs off its
 * provider).  An anchor is a function of the holder's absolute depth (see
 * @dst_base_depth); handing a merge-relative value to the splice record dated
 * the holder as a near-root node, which anchors on ITSELF at coarse spacings
 * while the fence dated it correctly and coarsened to an ancestor: one op,
 * two exclusion words for one node, and the splice acquire refuses its own
 * fence forever (measured: the exponential-MW occupied-dst graft storm).
 */
static
struct cds_ft_inode_flag *ft_merge_build(struct ft_merge_ctx *c,
		struct cds_ft_inode_flag *S, unsigned int off_s,
		struct cds_ft_inode_flag *D, unsigned int off_d,
		unsigned int depth, unsigned int d_prov,
		unsigned long *nr_keys_ret)
{
	struct cds_ft *ft = c->dst_ft;
	struct cds_ft_node *S_leaf, *D_leaf, *M_ext;
	struct cds_ft_compressed_node *cn_s, *cn_d;
	bool S_ext, D_ext, S_comp, D_comp;
	struct cds_ft_inode_flag *M = NULL;
	struct cds_ft_metadata *Mmeta = NULL;
	unsigned long total_keys = 0;
	bool tracked = false;
	unsigned int b, s_fb = 0, d_fb = 0;
	struct ft_held_anchor d_ov_held = { 0 };	/* this frame's dst overlap fence */
	bool d_ov_fenced = false;
	struct ft_held_anchor s_ov_held = { 0 };	/* and the src side's, when @fence_src */
	bool s_ov_fenced = false;

	S = ft_resolve_skip_compressed(ft, S);
	D = ft_resolve_skip_compressed(ft, D);

	S_comp = ft_node_compressed(S);
	D_comp = ft_node_compressed(D);
	cn_s = S_comp ? ft_compressed_node_ptr(S) : NULL;
	cn_d = D_comp ? ft_compressed_node_ptr(D) : NULL;

	/*
	 * Record each compressed overlap node's reclaim exactly once, on first
	 * entry (off == 0): a run may be re-entered at a deeper cursor by the
	 * shared-run recursion, but the node is freed whole.  src -> gs,
	 * dst -> gd.  Internal overlap nodes are recorded at the tail instead.
	 */
	/*
	 * Plan-lock THIS FRAME's SRC overlap node, on exactly the terms the dst
	 * block below states -- before any body read, and recording the retire in
	 * the same breath as the mark so a frame that fails before its tail cannot
	 * leave an unrecorded fence.  Only the rekey fold sets @fence_src; a
	 * cross-trie merge owns its source exclusively and skips this.
	 */
	if (c->fence_src) {
		void *snode = NULL;

		if (S_comp) {
			if (off_s == 0)
				snode = cn_s;
		} else if (!ft_node_external(S)) {
			snode = ft_node_ptr(S);
		}
		if (snode) {
			struct ft_lock_ctx sctx;

			ft_glue_lock_ctx(c->gs, &sctx);
			if (ft_merge_lock_overlap(ft, &sctx, S,
					c->src_base_depth + depth, snode,
					&s_ov_held)) {
				c->overlap_contended = true;
				return FT_MERGE_OOM;
			}
			ft_glue_defer_free_fenced(c->gs, snode, S_comp,
				&s_ov_held);
			s_ov_fenced = true;
		}
	}
	if (S_comp && off_s == 0 && !s_ov_fenced)
		ft_glue_defer_free(c->gs, cn_s, true);
	/*
	 * Plan-lock THIS FRAME's dst overlap node BEFORE any body read below -- the
	 * compressed prefix scan, the Pass-1 ft_node_get_nth_skip sweep, and the
	 * external_nodes fetch all read @D.  Acquiring here (not at the tail
	 * defer_free, which runs after those reads) is what makes the copy plan and
	 * the retire that ratifies it bracket the same world.
	 *
	 * A compressed run re-entered at off_d > 0 by the shared-run recursion was
	 * already fenced by the frame that entered it at 0, so it must NOT be
	 * re-marked: ft_meta_lock_acquire refuses an already-locked word and we
	 * would fail against our own fence.  An EXTERNAL D retires nothing here, so
	 * it takes no lock.
	 */
	if (c->fence_overlap) {
		void *dnode = NULL;

		if (D_comp) {
			if (off_d == 0)
				dnode = cn_d;
		} else if (!ft_node_external(D)) {
			dnode = ft_node_ptr(D);
		}
		if (dnode) {
			struct ft_lock_ctx dctx;

			ft_glue_lock_ctx(c->gd, &dctx);
			if (ft_merge_lock_overlap(ft, &dctx, D,
					c->dst_base_depth + depth, dnode,
					&d_ov_held)) {
				c->overlap_contended = true;
				return FT_MERGE_OOM;
			}
			/*
			 * RECORD THE RETIRE IN THE SAME BREATH AS THE MARK.  The
			 * free-list entry is what makes the fence visible to
			 * ft_glue_clear_fenced, and this frame has failure paths
			 * BEFORE its tail -- a child recursion returning
			 * FT_MERGE_OOM, a set_nth failure -- that return straight
			 * out.  Recording at the tail (where the plain retire sits)
			 * would leave those paths holding an unrecorded fence: a
			 * node left permanently LOCK, which no later publish into
			 * it can ever survive.  Recording early is harmless on the
			 * failing path, since the free list only tombstones anything
			 * if this build reaches its commit.
			 */
			ft_glue_defer_free_fenced(c->gd, dnode, D_comp,
				&d_ov_held);
			d_ov_fenced = true;
		}
	}
	if (D_comp && off_d == 0 && !d_ov_fenced)
		ft_glue_defer_free(c->gd, cn_d, true);

	/*
	 * Both compressed and sharing a prefix from their cursors -> collapse
	 * the shared bytes into one fresh run and recurse past it.  A zero-byte
	 * common prefix means the two runs diverge at the cursor: fall through
	 * to build a 2-way branch.
	 */
	if (S_comp && D_comp) {
		unsigned int rem_s = cn_s->len - off_s;
		unsigned int rem_d = cn_d->len - off_d;
		unsigned int maxp = rem_s < rem_d ? rem_s : rem_d;
		unsigned int p = 0;

		while (p < maxp &&
		       cn_s->key_bytes[off_s + p] == cn_d->key_bytes[off_d + p])
			p++;
		if (p >= 1)
			return ft_merge_build_run(c, cn_s, off_s, cn_d, off_d,
					p, depth, nr_keys_ret);
	}

	S_ext = ft_node_external(S);
	D_ext = ft_node_external(D);
	S_leaf = S_comp ? NULL : (S_ext ? (struct cds_ft_node *) ft_node_ptr(S)
		: cds_ft_item_to_metadata(ft_node_ptr(S))->external_nodes);
	D_leaf = D_comp ? NULL : (D_ext ? (struct cds_ft_node *) ft_node_ptr(D)
		: cds_ft_item_to_metadata(ft_node_ptr(D))->external_nodes);

	/*
	 * Both leaf-only -> the SAME full key terminates on both sides.
	 * Splice src after dst and return the (leaf-only) dst head; the
	 * parent frame wires the slot and the dst head's back-pointer.
	 */
	if (S_ext && D_ext) {
		ft_glue_record_splice(c->gd, D_leaf, S_leaf, d_prov);
		*nr_keys_ret = 1;
		return D;
	}

	/*
	 * Otherwise build a fresh internal branch M (a compressed side
	 * contributes its single forced byte; never external_nodes, which a
	 * compressed node may not carry).  Pass 1: add every union child's
	 * forward slot (cluster_leaf: no child back-pointer is written here --
	 * a later set_nth may reallocate M).  Recurse on shared bytes;
	 * reference (internal) or materialize (compressed) one-side bytes.
	 */
	if (S_comp)
		s_fb = cn_s->key_bytes[off_s];
	if (D_comp)
		d_fb = cn_d->key_bytes[off_d];
	for (b = 0; b < FT_ENTRY_PER_NODE; b++) {
		struct cds_ft_inode_flag *sc = NULL, *dc = NULL, *child;
		struct cds_ft_inode *old = NULL;
		bool sc_present, dc_present;
		unsigned long ck = 0;
		int ret;

		if (S_comp)
			sc_present = (b == s_fb);
		else if (!S_ext)
			sc_present = (sc = ft_node_get_nth_skip(S, NULL,
					(uint8_t) b, FT_PF_NONE)) != NULL;
		else
			sc_present = false;
		if (D_comp)
			dc_present = (b == d_fb);
		else if (!D_ext)
			dc_present = (dc = ft_node_get_nth_skip(D, NULL,
					(uint8_t) b, FT_PF_NONE)) != NULL;
		else
			dc_present = false;
		if (!sc_present && !dc_present)
			continue;
		/*
		 * F1 / RESOLVED-POINTER CONTRACT, at the build's source of children.
		 * ft_node_get_nth_skip is the RAW scanner: every other caller resolves
		 * its result (ft_node_get_nth and ft_node_get_nth_reanchor_slot both
		 * do), and this one does not.  A peer mid-commit on a live dst slot
		 * therefore hands the build a parked type-7 proxy, whose low nibble
		 * reads as an internal node.
		 *
		 * Do NOT resolve it.  @dc is EMBEDDED into the merged cluster below
		 * (child = dc, published by ft_node_set_nth), and resolving picks the
		 * old or new target by the peer's CURRENT status -- a status still free
		 * to flip, which would leave the merged node naming a retired child.
		 * Unresolved it is worse: the count and glue-origin reads dereference
		 * it, computing off the latch (ft_node_key_count, ft_glue_is_fresh).
		 *
		 * Bail on the CONTENTION channel instead, exactly as a missed overlap
		 * fence does: FT_MERGE_OOM is the one unwind sentinel and
		 * @overlap_contended is what tells the caller this was a peer and not
		 * memory, so it re-descends rather than reporting MEMORY_ERROR.  Both
		 * tries are pristine at this point -- the build has published nothing
		 * and ft_glue_abort releases every fence it took.
		 *
		 * @sc is guarded too though no capture has ever named it: a cross-trie
		 * source is exclusive, but the rekey fold's source is LIVE (the reason
		 * @fence_src exists), and the test is one predicted-not-taken compare.
		 */
		if (caa_unlikely(ft_node_flip_proxy(sc) ||
				ft_node_flip_proxy(dc))) {
			c->overlap_contended = true;
			return FT_MERGE_OOM;
		}

		if (sc_present && dc_present) {
			/*
			 * Both present -> recurse.  At most one side is
			 * compressed here (two divergent runs have different
			 * forced bytes), so the other side's target is its
			 * internal child at offset 0.
			 */
			struct cds_ft_inode_flag *ts, *td;
			unsigned int os, od;

			if (S_comp)
				ft_merge_advance(cn_s, off_s + 1, &ts, &os);
			else {
				ts = sc;
				os = 0;
			}
			if (D_comp)
				ft_merge_advance(cn_d, off_d + 1, &td, &od);
			else {
				td = dc;
				od = 0;
			}
			child = ft_merge_build(c, ts, os, td, od, depth + 1,
					c->dst_base_depth + depth -
					(D_comp ? off_d : 0), &ck);
			if (child == FT_MERGE_OOM)
				return child;
		} else if (sc_present) {
			if (S_comp) {
				child = ft_merge_materialize_suffix(c, cn_s,
						off_s, depth + 1,
						/*dst_origin=*/ false, &ck);
				if (child == FT_MERGE_OOM)
					return child;
			} else {
				child = sc;	/* reference live src subtree */
				ck = ft_merge_child_count(ft, sc);
			}
		} else {
			if (D_comp) {
				child = ft_merge_materialize_suffix(c, cn_d,
						off_d, depth + 1,
						/*dst_origin=*/ true, &ck);
				if (child == FT_MERGE_OOM)
					return child;
			} else {
				child = dc;	/* reference live dst subtree */
				ck = ft_merge_child_count(ft, dc);
			}
		}
		ret = ft_node_set_nth(ft, &M, (uint8_t) b, child, &old, Mmeta,
				depth, /*cluster_leaf*/ true);
		if (ret)
			return FT_MERGE_OOM;
		if (old) {
			/*
			 * @old is not a displaced CHILD: ft_node_set_nth returns
			 * the node's own SUPERSEDED BODY here when the slot did
			 * not fit and the node had to GROW (ft_node_recompact,
			 * @old_node_ret).  That recompact has already marked it
			 * DEAD -- §4.B freeze-on-free, set for every retire it
			 * performs -- and states the matching obligation: "the
			 * caller ... frees @old_node AFTER A GRACE PERIOD".
			 *
			 * So take the RETIRE path, not the unpublished one.  The
			 * immediate path's contract is "never published, no
			 * reader can hold a reference", and it PROVES that by
			 * asserting the node carries no tombstone -- which this
			 * body does, so the immediate free read as a live-node
			 * free (unit test 60, every FT_DEBUG_TOMBSTONE_AUDIT
			 * build).  The cluster being build-invisible does not
			 * change the ownership: the mark says the retire is
			 * already owned, and the grace period is what that
			 * ownership costs, once per node GROWTH in a merge build.
			 */
			ft_glue_untrack(ft, c->gd, old);
			free_cds_ft_node(ft, old);
		}
		if (!tracked) {
			ft_glue_track(c->gd, M);
			tracked = true;
			Mmeta = cds_ft_item_to_metadata(ft_node_ptr(M));
			/*
			 * Clear the recycled allocation's stale parent before
			 * any later set_nth reallocation copies it forward.
			 */
			Mmeta->parent_word = NULL;
#ifdef FEATURE_FT_SKIP_COMPRESSED
			ft_meta_parent_slot_offset_set(Mmeta, 0);
#endif
		} else if (old) {
			ft_glue_track(c->gd, M);
		}
		Mmeta = cds_ft_item_to_metadata(ft_node_ptr(M));
		total_keys += ck;
	}

	/* Merged external_nodes (the key terminating at M itself). */
	if (S_leaf && D_leaf) {
		M_ext = D_leaf;
		ft_glue_record_splice(c->gd, D_leaf, S_leaf,
			D_ext ? d_prov : c->dst_base_depth + depth);
	} else if (D_leaf) {
		M_ext = D_leaf;
	} else {
		M_ext = S_leaf;		/* may be NULL */
	}
	if (M_ext) {
		ft_metadata_set_external_nodes(M, Mmeta, M_ext);
		total_keys += 1;
	}

	/*
	 * Pass 2: wire every child's back-pointer (deferred) plus M's
	 * external back-channel.  Fetch slots only now -- the Pass-1 set_nth
	 * reallocations may have moved them.  A fresh (recursed/materialized)
	 * child stores its parent immediately via defer_edge's is_fresh fast
	 * path; a referenced live child truly defers to commit.
	 */
	for (b = 0; b < FT_ENTRY_PER_NODE; b++) {
		struct cds_ft_inode_flag **slot;
		struct cds_ft_inode_flag *child =
			ft_node_get_nth_skip(M, &slot, (uint8_t) b, FT_PF_NONE);
		bool dst_origin;

		if (!child)
			continue;
		/*
		 * A referenced one-side child is dst-origin iff D still holds a
		 * child at this byte (a shared byte was recursed/materialized
		 * into a fresh cluster node, applied immediately by defer_edge's
		 * is_fresh path, so its origin is irrelevant).  dst-origin
		 * back-pointers are switched by the flip-latch after the forward
		 * publish, not by apply_deferred.
		 */
		if (D_comp)
			dst_origin = (b == d_fb);
		else
			dst_origin = !D_ext && ft_node_get_nth_skip(D, NULL,
					(uint8_t) b, FT_PF_NONE) != NULL;
		ft_glue_defer_edge_origin(ft, c->gd, child, M, slot, dst_origin);
		/*
		 * A freshly-built compressed child (run / suffix) was stored as
		 * its PLAIN flag so is_fresh could match it by identity above;
		 * now that its edge is wired, re-encode the slot to the canonical
		 * skip form.  Referenced compressed children are already stored
		 * skip-encoded (a skip flag is not ft_node_compressed).
		 */
		if (ft_node_compressed(child)) {
			struct cds_ft_inode_flag *skip = ft_publish_compressed(ft,
				ft_compressed_node_ptr(child), child);

			if (skip != child)
				*slot = skip;
		}
	}
	if (M_ext)
		ft_glue_defer_edge_origin(ft, c->gd,
			(struct cds_ft_inode_flag *) M_ext, M, NULL,
			/*dst_origin=*/ D_leaf != NULL);

	/*
	 * Reclaim the INTERNAL overlap nodes M copied (compressed ones were
	 * recorded on entry above).  src -> gs, dst -> gd.
	 */
	if (!S_ext && !S_comp
			/* Already recorded at entry, together with its fence. */
			&& !s_ov_fenced
	   )
		ft_glue_defer_free(c->gs, ft_node_ptr(S), false);
	if (!D_ext && !D_comp
			/* Already recorded at entry, together with its fence. */
			&& !d_ov_fenced
	   )
		ft_glue_defer_free(c->gd, ft_node_ptr(D), false);

	ft_nr_keys_store(ft, Mmeta, total_keys, CMM_RELAXED);
	*nr_keys_ret = total_keys;
	return M;
}

/*
 * Read-only pre-pass mirroring ft_merge_build's control flow, accumulating
 * upper bounds for ft_glue_reserve so the build never asserts on a full
 * inline floor.  No allocation, no mutation.
 */
static
void ft_merge_count(struct cds_ft *ft, struct cds_ft_inode_flag *S, unsigned int off_s,
		struct cds_ft_inode_flag *D, unsigned int off_d,
		struct ft_merge_counts *cnt)
{
	struct cds_ft_compressed_node *cn_s, *cn_d;
	bool S_ext, D_ext, S_comp, D_comp;
	struct cds_ft_node *S_leaf, *D_leaf;
	unsigned int b, s_fb = 0, d_fb = 0;

	S = ft_resolve_skip_compressed(ft, S);
	D = ft_resolve_skip_compressed(ft, D);
	S_comp = ft_node_compressed(S);
	D_comp = ft_node_compressed(D);
	cn_s = S_comp ? ft_compressed_node_ptr(S) : NULL;
	cn_d = D_comp ? ft_compressed_node_ptr(D) : NULL;

	/* Compressed overlap nodes are freed whole, recorded on first entry. */
	if (S_comp && off_s == 0)
		cnt->nf_src++;
	if (D_comp && off_d == 0)
		cnt->nf_dst++;

	/* Shared run: one fresh run node + its child edge, then recurse past. */
	if (S_comp && D_comp) {
		unsigned int rem_s = cn_s->len - off_s;
		unsigned int rem_d = cn_d->len - off_d;
		unsigned int maxp = rem_s < rem_d ? rem_s : rem_d;
		unsigned int p = 0;

		while (p < maxp &&
		       cn_s->key_bytes[off_s + p] == cn_d->key_bytes[off_d + p])
			p++;
		if (p >= 1) {
			struct cds_ft_inode_flag *as, *ad;
			unsigned int aos, aod;

			cnt->nb++;
			cnt->nd++;
			ft_merge_advance(cn_s, off_s + p, &as, &aos);
			ft_merge_advance(cn_d, off_d + p, &ad, &aod);
			ft_merge_count(ft, as, aos, ad, aod, cnt);
			return;
		}
	}

	S_ext = ft_node_external(S);
	D_ext = ft_node_external(D);
	S_leaf = S_comp ? NULL : (S_ext ? (struct cds_ft_node *) ft_node_ptr(S)
		: cds_ft_item_to_metadata(ft_node_ptr(S))->external_nodes);
	D_leaf = D_comp ? NULL : (D_ext ? (struct cds_ft_node *) ft_node_ptr(D)
		: cds_ft_item_to_metadata(ft_node_ptr(D))->external_nodes);
	if (S_ext && D_ext) {
		cnt->ns++;
		return;
	}
	cnt->nb++;		/* fresh branch M */
	if (S_comp)
		s_fb = cn_s->key_bytes[off_s];
	if (D_comp)
		d_fb = cn_d->key_bytes[off_d];
	for (b = 0; b < FT_ENTRY_PER_NODE; b++) {
		struct cds_ft_inode_flag *sc = NULL, *dc = NULL;
		bool sc_present, dc_present;

		if (S_comp)
			sc_present = (b == s_fb);
		else if (!S_ext)
			sc_present = (sc = ft_node_get_nth_skip(S, NULL,
					(uint8_t) b, FT_PF_NONE)) != NULL;
		else
			sc_present = false;
		if (D_comp)
			dc_present = (b == d_fb);
		else if (!D_ext)
			dc_present = (dc = ft_node_get_nth_skip(D, NULL,
					(uint8_t) b, FT_PF_NONE)) != NULL;
		else
			dc_present = false;
		if (!sc_present && !dc_present)
			continue;
		cnt->nd++;		/* M -> child back-pointer edge */
		if (sc_present && dc_present) {
			struct cds_ft_inode_flag *ts, *td;
			unsigned int os, od;

			if (S_comp)
				ft_merge_advance(cn_s, off_s + 1, &ts, &os);
			else {
				ts = sc;
				os = 0;
			}
			if (D_comp)
				ft_merge_advance(cn_d, off_d + 1, &td, &od);
			else {
				td = dc;
				od = 0;
			}
			ft_merge_count(ft, ts, os, td, od, cnt);
		} else if ((sc_present && S_comp) || (dc_present && D_comp)) {
			/* Materialized suffix; a fresh wrapper adds a node+edge. */
			struct cds_ft_compressed_node *cn = sc_present ? cn_s : cn_d;
			unsigned int off = sc_present ? off_s : off_d;
			unsigned int suffix_len = cn->len - off - 1;

			if (suffix_len >= 1) {
				cnt->nb++;
				cnt->nd++;
			}
		}
	}
	if (S_leaf && D_leaf) {
		cnt->ns++;
		cnt->nd++;
	} else if (S_leaf || D_leaf) {
		cnt->nd++;
	}
	if (!S_ext && !S_comp)
		cnt->nf_src++;
	if (!D_ext && !D_comp)
		cnt->nf_dst++;
}

/*
 * A surviving source head captured for the spine-copy interleave: its cell plus
 * the key SUFFIX below the src merge point (@suffix_off into the caller's packed
 * pool, @suffix_len bytes).  Captured BEFORE the src unlink, while S is still
 * attached and up-walkable; the merge below compares these against the LIVE dst
 * suffixes.  See ft_merge_ord_interleave_collect.
 */
struct ft_merge_src_cap {
	struct ft_ord_cell *cell;
	size_t suffix_off;
	size_t suffix_len;
};

/*
 * Compare two ordinal key suffixes.  Matches the ordered-list key ordering: a
 * shorter key that is a prefix of a longer one sorts FIRST (the prefix-key rule
 * ft_subtree_minmax_head relies on).  Returns <0 / 0 / >0.
 */
static inline
int ft_merge_suffix_cmp(const uint8_t *a, size_t la, const uint8_t *b, size_t lb)
{
	size_t m = la < lb ? la : lb;
	int c = m ? memcmp(a, b, m) : 0;

	if (c != 0)
		return c;
	if (la == lb)
		return 0;
	return la < lb ? -1 : 1;
}

/*
 * A SURVIVOR cell's own forward / back link during the interleave: a plain store
 * when the cell is not ord-reachable (the cross-trie merge, whose survivor came
 * from a consumed source list), a RECORDED edge when it is (an in-trie rekey,
 * whose survivors are live in the list being rebuilt).  See @record_all at
 * ft_merge_ord_interleave_collect.
 */
static
unsigned int ft_ord_survivor_link(struct ft_ord_cell *cell,
		struct ft_ord_cell *next, struct ft_ord_cell_edge *edges,
		unsigned int n, bool record_all)
{
	if (!record_all) {
		cell->lnode.next = ft_ord_cell_lnode(next);
		return n;
	}
	edges[n].tag = URCU_TXN_TAG;		/* ordered-cell edge */
	edges[n].slot = (struct ft_ord_cell **) &cell->lnode.next;
	edges[n].old_target = ft_ord_cell_resolve_ord(&cell->lnode.next);
	edges[n].new_target = next;
	return n + 1;
}

static
unsigned int ft_ord_survivor_back(struct ft_ord_cell *cell,
		struct ft_ord_cell *prev, struct ft_ord_cell_edge *edges,
		unsigned int n, bool record_all)
{
	if (!record_all) {
		cell->lnode.prev = ft_ord_cell_lnode(prev);
		return n;
	}
	edges[n].tag = URCU_TXN_TAG;		/* ordered-cell edge */
	edges[n].slot = (struct ft_ord_cell **) &cell->lnode.prev;
	edges[n].old_target = ft_ord_cell_resolve_ord(&cell->lnode.prev);
	edges[n].new_target = prev;
	return n + 1;
}

/*
 * COLLECT the ordered-list edges that interleave the surviving source cells into
 * @dst's ordered list for a cds_ft_merge_at spine-copy, by a two-pointer
 * KEY-ORDER MERGE of the two already-sorted LIVE cell runs:
 *   - the dst merge region: @dst_first .. (exclusive) @dst_succ, walked via
 *     ord_next.  Its heads are up-walked LIVE (ft_rebuild_key_upwalk over the
 *     intact D, no proxy installed) and their key SUFFIX below the dst merge
 *     point is suffix = full_key[@dst_key_len ..].
 *   - the surviving src run: @src_caps[0 .. @nsrc), each carrying its
 *     pre-captured suffix below the src merge point (captured by the caller
 *     BEFORE ft_merge_unlink_src_subtree, while S was attached -- the detach
 *     leaves S-root's parent stale, so a post-unlink src up-walk is unsafe).
 * Both runs share the merge-point prefix, so they merge by comparing those
 * suffixes; an equal-suffix step is a COLLISION (the dst head wins -- its dup
 * chain absorbs the src head via ft_glue_record_splices -- so the src head is
 * dropped, a floating duplicate never reachable as a distinct head).
 *
 * Runs in the txn's PREPARE phase, with NO structural proxy installed: the
 * merged ORDER is reconstructed from the two live runs rather than by walking
 * the about-to-be-published merged structure, so the collect needs neither the
 * staged proxies nor a writer-side merged-view resolution -- and there is no
 * append-after-install (every edge is recorded before install).  It accumulates
 * the reader-VISIBLE boundary edges -- a dst-original cell's ord_next / ord_prev,
 * or @dst's head / tail -- into @edges, RETURNED as a count for the caller to
 * record into the structural flip txn so structure + interleave commit in ONE
 * flip.  @prev_placed seeds at the region predecessor (@dst_first's ord_prev).
 *
 * ★ @record_all DECIDES WHAT A SURVIVOR'S OWN LINKS COST.  With it FALSE the
 * survivor's next/prev are PLAIN STORES, which is sound only because a cross-trie
 * merge's survivor "is not ord-reachable in @dst -- never was -- and the caller
 * already unlinked it from src".  An IN-TRIE rekey breaks both halves: its
 * survivors are live in the very list being rebuilt, so a plain store there is a
 * reader-visible, non-atomic mutation.  TRUE records those links as edges too, at
 * up to 2 more per survivor, so the whole reorder lands in the one commit.  The
 * caller sizes @edges accordingly: <= 2*merged_keys+2 visible, plus 2*nsrc when
 * @record_all.
 *
 * @ncollide (optional) counts the equal-suffix steps.  A collision drops the src
 * head from the merged order on the premise that it became a DUPLICATE on the dst
 * head's chain and is "a floating duplicate never reachable as a distinct head" --
 * true when the src list is consumed, FALSE in-trie, where that cell stays linked
 * where it was and would keep answering as a distinct key.  An in-trie caller must
 * therefore check this and decline; with @record_all the collect stores nothing, so
 * a declined run costs only the walk.
 *
 * Identity key_map only (matches the rest of the ordered-list machinery).
 */
static
unsigned int ft_merge_ord_interleave_collect(struct cds_ft *dst,
		size_t dst_key_len, struct ft_ord_cell *dst_first,
		struct ft_ord_cell *dst_succ, struct ft_ord_cell *prev_placed,
		const struct ft_merge_src_cap *src_caps, unsigned long nsrc,
		const uint8_t *src_pool, struct ft_ord_cell_edge *edges,
		bool record_all, unsigned long *ncollide)
{
	size_t max_len = dst->group->max_key_len;
	uint8_t dbuf[FT_MAX_KEY_LEN];
	struct ft_ord_cell *dcur = dst_first;
	struct ft_ord_cell *prev = prev_placed;
	/*
	 * Sentinel topology: @prev_placed is the region predecessor, which now
	 * resolves to @dst's sentinel pseudo-cell (never NULL) when the region is
	 * at the list head.  prev_is_dst tracks "the previously-placed cell is
	 * already linked to its successor and needs no forward edge" -- true at the
	 * start because the predecessor->region-first link pre-exists (whether the
	 * predecessor is a real dst cell or the sentinel: sentinel.next already
	 * points at the region's first cell).  So the old "new list minimum" /
	 * "list tail" head/tail special cases fold into the general neighbour-edge
	 * path (the sentinel IS the neighbour).
	 */
	bool prev_is_dst = true;
	unsigned long si = 0;
	unsigned int n = 0;
	const uint8_t *dsuf = NULL;
	size_t dsuf_len = 0;
	bool dsuf_valid = false;

	/*
	 * Two-pointer key-order merge of the dst region run and the src survivor
	 * run.  Each step emits the smaller-suffix head: a dst-original head stays
	 * put (its back edge changes only when a survivor run precedes it); a
	 * surviving src head splices in after the last placed cell.  A tie is a
	 * collision -> emit the dst head, drop the src head.  Dst-original cells
	 * keep their relative order, so a dst<->dst step needs no edge; each
	 * survivor RUN costs at most two edges (one entering, one leaving), so
	 * 2 * merged_keys + 2 bounds @edges.
	 */
	while (dcur != dst_succ || si < nsrc) {
		bool take_dst;

		if (dcur == dst_succ) {
			take_dst = false;		/* dst run exhausted */
		} else if (si == nsrc) {
			take_dst = true;		/* src run exhausted */
		} else {
			int cmp;

			if (!dsuf_valid) {
				size_t dfl = ft_rebuild_key_upwalk(dst, dcur,
						dbuf, max_len);

				assert(dfl >= dst_key_len);
				dsuf = dbuf + (max_len - dfl) + dst_key_len;
				dsuf_len = dfl - dst_key_len;
				dsuf_valid = true;
			}
			cmp = ft_merge_suffix_cmp(dsuf, dsuf_len,
					src_pool + src_caps[si].suffix_off,
					src_caps[si].suffix_len);
			/* Tie: dst head wins, drop the colliding src head. */
			if (cmp == 0) {
				si++;
				if (ncollide)
					(*ncollide)++;
			}
			take_dst = (cmp <= 0);
		}

		if (take_dst) {
			struct ft_ord_cell *cell = dcur;

			/*
			 * Dst-original cell: stays put, already linked in key
			 * order.  Its back edge changes only when a survivor run
			 * was just placed before it (prev is a survivor).
			 */
			if (!prev_is_dst) {
				n = ft_ord_survivor_link(prev, cell, edges, n,
						record_all);
				edges[n].tag = URCU_TXN_TAG;	/* ordered-cell edge */
				edges[n].slot = (struct ft_ord_cell **) &cell->lnode.prev;
				edges[n].old_target =
					ft_ord_cell_resolve_ord(&cell->lnode.prev);
				edges[n].new_target = prev;
				n++;
			}
			prev = cell;
			prev_is_dst = true;
			dcur = ft_ord_cell_resolve_ord(&dcur->lnode.next);
			dsuf_valid = false;
		} else {
			struct ft_ord_cell *cell = src_caps[si].cell;

			/* Surviving src cell: pre-set its back link (prev may be the
			 * sentinel: a new list minimum links its prev to &sentinel). */
			n = ft_ord_survivor_back(cell, prev, edges, n, record_all);
			if (prev_is_dst) {
				/*
				 * prev is a dst cell OR the sentinel (region at head):
				 * flip its forward edge to the survivor.  When prev is the
				 * sentinel this IS the old "new list minimum: flip head".
				 */
				edges[n].tag = URCU_TXN_TAG;	/* ordered-cell edge */
				edges[n].slot = (struct ft_ord_cell **) &prev->lnode.next;
				edges[n].old_target =
					ft_ord_cell_resolve_ord(&prev->lnode.next);
				edges[n].new_target = cell;
				n++;
			} else {
				n = ft_ord_survivor_link(prev, cell, edges, n,
						record_all);
			}
			prev = cell;
			prev_is_dst = false;
			si++;
		}
	}
	/*
	 * Close the trailing edge: if the last placed cell is a survivor, link it
	 * to the region successor @dst_succ (the sentinel at the list tail) and flip
	 * that neighbour's back edge.  Sentinel topology: when @dst_succ is the
	 * sentinel this IS the old "flip @dst's tail".
	 */
	if (!prev_is_dst) {
		n = ft_ord_survivor_link(prev, dst_succ, edges, n, record_all);
		edges[n].tag = URCU_TXN_TAG;	/* ordered-cell edge */
		edges[n].slot = (struct ft_ord_cell **) &dst_succ->lnode.prev;
		edges[n].old_target =
			ft_ord_cell_resolve_ord(&dst_succ->lnode.prev);
		edges[n].new_target = prev;
		n++;
	}
	return n;
}


/*
 * KEY_SHORTER dst: the merge point sits @off bytes inside a compressed dst
 * node whose prefix bytes @key[@d_depth .. @key_len) lie ABOVE it.  Wrap the
 * freshly-built merged node @M (with @mk unique keys) under that prefix and
 * return the PLAIN flag to publish into the old compressed node's forward slot
 * (the caller skip-encodes it), or FT_MERGE_OOM.
 *
 * @M internal: ft_build_branch lays a compressed prefix over the fresh
 * internal M -- M's edge into the prefix applies immediately via defer_edge's
 * is_fresh fast path, so ft_build_branch's dst_origin=false leaf edge is
 * correct, and M's own children keep the dst_origin ft_merge_build gave them.
 * A 1-child-no-external M is first canonicalized to compressed (its lone child
 * is always a FRESH recursion result, so the fold is dst_origin-safe too).
 *
 * @M compressed: M is a fresh run produced by ft_merge_build_run (the sole
 * path returning a compressed top), whose child may be a LIVE dst splice head
 * carrying dst_origin=true (a duplicate-key merge through the compressed node).
 * ft_build_branch's chain-merge would re-defer that child as dst_origin=false
 * and demote a live re-parent out of the flip latch, so fuse the prefix into M
 * here by hand, carrying the run child's dst_origin (always true).  The fused
 * length @off + M->len <= cn_d->len, so it is a valid compressed length.
 */
static
struct cds_ft_inode_flag *ft_merge_wrap_prefix(struct ft_merge_ctx *c,
		const uint8_t *key, unsigned int d_depth, unsigned int key_len,
		struct cds_ft_inode_flag *M, unsigned long mk)
{
	struct cds_ft *ft = c->dst_ft;
	unsigned int prefix_len = key_len - d_depth;

	if (ft_node_compressed(M)) {
		struct cds_ft_compressed_node *mcn = ft_compressed_node_ptr(M);
		unsigned int merged_len = prefix_len + mcn->len;
		struct cds_ft_compressed_node *merged;
		struct cds_ft_metadata *merged_meta;
		struct cds_ft_inode_flag *mflag;

		merged = alloc_compressed_node(ft, (uint8_t) merged_len,
				&merged_meta);
		if (!merged)
			return FT_MERGE_OOM;
		memcpy(merged->key_bytes, &key[d_depth], prefix_len);
		memcpy(&merged->key_bytes[prefix_len], mcn->key_bytes, mcn->len);
		merged->len = (uint8_t) merged_len;
		merged->child = mcn->child;
		ft_meta_nr_child_set(merged_meta, 1);
		ft_nr_keys_store(ft, merged_meta, mk, CMM_RELAXED);
		mflag = ft_compressed_node_flag(merged);
		ft_glue_track(c->gd, mflag);
		/*
		 * Carry the run child's dst_origin (ft_merge_build_run records
		 * it true): a fresh child applies via is_fresh, a live splice
		 * head flips with the forward slot.  The defer de-dup supersedes
		 * M's stale &mcn->child edge with this one (same @child).
		 */
		ft_glue_defer_edge_origin(ft, c->gd, mcn->child, mflag,
				&merged->child, /*dst_origin=*/ true);
		ft_glue_untrack(ft, c->gd, mcn);
		free_compressed_node_unpublished(ft, mcn);
		return mflag;	/* PLAIN; caller skip-encodes before publish */
	} else {
		struct cds_ft_inode_flag *canon, *top;

		canon = ft_compress_single_child_if_needed(ft, M, c->gd);
		if (canon == (struct cds_ft_inode_flag *) (long) -ENOMEM)
			return FT_MERGE_OOM;
		top = ft_build_branch(ft, key, d_depth, key_len, canon, mk,
				/*has_external_nodes=*/ false, c->gd);
		if (!top)
			return FT_MERGE_OOM;
		return top;	/* PLAIN compressed; caller skip-encodes */
	}
}

/*
 * Keys in ORDINAL form (cds_ft_merge_at converts the caller's application
 * keys once at its entry; every consumer below -- the spine build, the source
 * unlink, the ordered-list interleave -- expects ordinal bytes).
 *
 * Build-invisible spine-copy merge of @src_ft's subtree at @src_key into
 * @dst_ft at the live EXACT subtree @d_dst.  All allocation is in the build
 * phase (plus the single fallible src unlink at commit for a non-root src), so
 * any OOM frees the fresh copies and leaves both tries pristine -- no rollback.
 *
 * @src_ft merged at @src_key (@d_src its subtree, @cnt_src its key count) into
 * @d_dst, a non-empty dst subtree.  A root src (src_key_len == 0) unlinks via
 * the ft->root swap; a non-root src unlinks its branch in place at commit (the
 * only post-build fallible step).  @off_src selects the src merge-point shape:
 * 0 is an EXACT subtree at @d_src->nf; > 0 is KEY_SHORTER (the src key ends
 * @off_src bytes inside the compressed node @d_src->nf -- the build enters that
 * node at the cursor, and ft_merge_unlink_src_subtree reclaims the whole node
 * via its chain-reclaim, the move-style detach preserving cn_s->child).  Both
 * root and non-root dst merge points are handled: a non-root point flips the
 * interior forward slot through a type-7 proxy that the read-side descent
 * resolves at every child fetch.  Compressed overlaps are handled.  @off_dst
 * selects the dst merge-point shape: 0 is an EXACT subtree at @d_dst->nf; > 0
 * is KEY_SHORTER (the dst key ends @off_dst bytes inside the compressed node
 * @d_dst->nf, whose prefix bytes are wrapped around the merged cluster by
 * ft_merge_wrap_prefix).
 *
 * Every (EXACT | KEY_SHORTER) src x (EXACT | KEY_SHORTER) dst shape is handled.
 *
 * Returns OK on a committed merge, or MEMORY_ERROR on OOM (both tries pristine).
 *
 * @contended is the CONTENTION channel, deliberately kept OUT of enum
 * cds_ft_status.  A dup-chain holder lock this attempt could not acquire (MW
 * LOCK_FINE) means "the tree moved, rebuild and try again" -- not one of merge's
 * outcomes.  cds_ft_merge_at consumes an EXCLUSIVE source, so it has no
 * transient failure to report: an attempt that bails has moved nothing and is
 * trivially undone, and the caller just re-descends.  Merge's one public
 * BUSY_ERROR says something else entirely -- the source is LIVE, call
 * cds_ft_make_exclusive first -- and it is a precondition: checked before any
 * allocation, and permanent, since retrying the identical call returns the
 * identical answer.  Overloading it would leave a caller unable to tell "fix
 * your source" from "just loop".
 *
 * So on contention @contended is set and the RETURN is MEMORY_ERROR -- the
 * conservative surface every other bail here already uses, so a caller that
 * forgets to check @contended still reports a no-op on two pristine tries
 * rather than a false success.  The retry belongs to the caller, which is the
 * only one that can re-descend.
 */
static
enum cds_ft_status ft_merge_spine_copy(struct cds_ft *dst_ft,
		struct cds_ft *src_ft, struct ft_descent *d_src,
		const uint8_t *src_key, size_t src_key_len, unsigned long cnt_src,
		unsigned int off_src, struct ft_descent *d_dst,
		unsigned long cnt_dst, unsigned int off_dst,
		size_t dst_key_len,
		struct ft_flip_txn **pre_txn, bool *contended,
		struct urcu_txn *op)
{
	struct ft_glue gd, gs;
	struct ft_merge_ctx ctx = { .dst_ft = dst_ft, .gd = &gd, .gs = &gs };
	struct ft_merge_counts cnt = { 0, 0, 0, 0, 0 };
	bool root_src = (src_key_len == 0);
	bool ks_dst = (off_dst > 0);
	bool ed;
	struct cds_ft_inode_flag *S = d_src->nf;
	struct cds_ft_inode_flag *D = d_dst->nf;
	struct cds_ft_inode_flag *M, *M_slot = NULL, *pub, *D_old;
	struct cds_ft_inode_flag *pub_parent = d_dst->pnf, **pub_slot = d_dst->nfp;
	struct cds_ft_inode *fresh_root = NULL;
	struct cds_ft_metadata *fresh_meta;
	struct ft_flip_txn *txn;
	unsigned long merged_keys = 0;
	bool ms_ord = dst_ft->group->ordered_list_set;
	struct ft_ord_cell *ms_cursor = NULL, *ms_prev = NULL, *ms_succ = NULL;
	struct cds_ft_node *ms_s_first = NULL, *ms_s_last = NULL;
	struct ft_ord_cell_edge *ms_edges = NULL;
	struct ft_merge_src_cap *ms_src_caps = NULL;	/* src survivor suffixes, pre-captured */
	uint8_t *ms_src_pool = NULL;			/* packed suffix byte pool */
	unsigned long ms_nsrc = 0;			/* src run cells captured */
	unsigned int ms_cap = 0;	/* interleave edge cap, set once merged_keys is known */
	unsigned int ms_n = 0;		/* interleave edges collected (staged pre-commit) */
	/*
	 * The caller pre-reserved this op's flip-txn, i.e. it is already PAST its
	 * own point of no return and pre-built everything so this placement cannot
	 * fail.  Gates the dup-chain acquire below -- the only step here that can
	 * decline.
	 */
	bool unfailable = (pre_txn && *pre_txn);

	*contended = false;

	/*
	 * Every dst merge-point shape is handled.  The flip proxies the publish
	 * slot @pub_slot, the read-side descent resolves the type-7 proxy at every
	 * child fetch (ft_resolve_flip_proxy, before the skip handler), and the
	 * published node's parent is wired to @pub_parent by
	 * ft_glue_set_publish, so a descent and an up-walk see a coherent
	 * old-XOR-merged view across the flip.
	 *
	 * Edge D: the merge point's PARENT is a COMPRESSED node cn_p reached via a
	 * grandparent skip slot.  cn_p's own parent cannot be compressed ("no two
	 * adjacent compressed" invariant), so the grandparent slot d_dst->pnfp is a
	 * plain internal slot that merely *holds* skip(cn_p).  Handle it one level
	 * up: the merge is EXACT at d_dst->nf (off_dst == 0), M is wrapped under a
	 * fresh copy of the WHOLE cn_p, and that copy is published into d_dst->pnfp
	 * in place of skip(cn_p) -- identical machinery to a KEY_SHORTER dst wrap,
	 * just with cn_p as the wrapped node and the grandparent as the publish
	 * point.  ks_dst and ed are mutually exclusive (a KEY_SHORTER merge point
	 * sits inside cn_d, whose parent is internal).
	 */
	ed = (d_dst->pnf &&
	      ft_node_compressed(ft_resolve_skip_compressed(dst_ft, d_dst->pnf)));

	/* Size both glues from a read-only pre-pass (with headroom). */
	ft_merge_count(dst_ft, S, off_src, D, off_dst, &cnt);
	ft_glue_init(&gd);
	ft_glue_init(&gs);
	gd.op = gs.op = op;	/* both glues' lock ctxs age this op */
	/*
	 * The dst glue's own acquires -- @pub_parent, above all -- fire from
	 * commit helpers that never see a descent, so hand them the destination
	 * one here, the single place holding both (the graft does the same at its
	 * ft_glue_set_publish).  @pub_parent is @d_dst's own parent or
	 * grandparent, so the window dates it; without this the acquire has no
	 * depth under a coarse spacing and its miss aborts a commit the unfailable
	 * arm cannot retry -- which is a LIVELOCK, not a failure.
	 */
	gd.lock_d = d_dst;
	/*
	 * And the SOURCE glue's, for the same reason: a src overlap fence is
	 * dated on the path the node is on NOW, which only @d_src describes.
	 * Inert for a cross-trie merge (@fence_src stays false -- the source is
	 * exclusive), set so the two glues answer from the same rule.
	 */
	gs.lock_d = d_src;
	/*
	 * Where relative depth 0 IS, on each side (see @dst_base_depth).
	 */
	ctx.dst_base_depth = (unsigned int) d_dst->depth + off_dst;
	ctx.src_base_depth = (unsigned int) d_src->depth + off_src;
	if (ft_glue_reserve(&gd, cnt.nb + 8, cnt.nd + 8,
				cnt.nf_dst + 8, cnt.ns + 8) ||
	    ft_glue_reserve(&gs, 0, 0, cnt.nf_src + 8, 0)) {
		ft_glue_fini(&gd);
		ft_glue_fini(&gs);
		return CDS_FT_STATUS_MEMORY_ERROR;
	}

	/* Root src: pre-allocate the fresh empty root for the commit swap. */
	if (root_src) {
		fresh_root = alloc_cds_ft_node(src_ft, &ft_types[0], &fresh_meta);
		if (!fresh_root) {
			ft_glue_fini(&gd);
			ft_glue_fini(&gs);
			return CDS_FT_STATUS_MEMORY_ERROR;
		}
		fresh_meta->parent_word = ft_trie_parent(src_ft);
		ft_nr_keys_store(src_ft, fresh_meta, 0, CMM_RELAXED);
	}

	/* Build the merged cluster invisibly (the only build-phase fallible step). */
	/*
	 * DLM overlap-spine plan-lock (§9.4 M-2): fence each dst overlap node before
	 * the build reads it, and retire it through the fenced terminal, so a peer
	 * growing a node this merge is copying cannot be silently retired with it.
	 * Skipped for the unfailable caller, which has no bail left to take.
	 */
	ctx.fence_overlap = dst_ft->lock_fine && !unfailable;
	ctx.fence_src = false;		/* cross-trie: the source is exclusive */
	M = ft_merge_build(&ctx, S, off_src, D, off_dst, 0, d_dst->pdepth,
			&merged_keys);
	if (M == FT_MERGE_OOM) {
		if (fresh_root)
			free_cds_ft_node_unpublished(src_ft, fresh_root);
		ft_glue_abort(dst_ft, &gd);
		ft_glue_abort(src_ft, &gs);
		/*
		 * A missed overlap fence shares FT_MERGE_OOM's unwind but is
		 * CONTENTION, not memory: both tries are pristine (the build published
		 * nothing and ft_glue_abort released every fence it took), so report it
		 * on the retry channel and let the caller re-descend.
		 */
		if (ctx.overlap_contended)
			*contended = true;
		return CDS_FT_STATUS_MEMORY_ERROR;
	}
	/*
	 * The ordered-list interleave is folded into the structural flip (one
	 * commit for structure + ordered list), so its <= 2*merged_keys+2 cell
	 * edges share the structural flip batch.  Each survivor run costs at most
	 * two visible edges; bound it now that @merged_keys is known.
	 */
	if (ms_ord)
		ms_cap = 2u * (unsigned int) merged_keys + 2u;

	/*
	 * Compute @pub (the value to publish), @pub_parent / @pub_slot (where) and,
	 * for the compressed shapes, reclaim the replaced node.
	 *
	 * KEY_SHORTER dst (off_dst > 0): the merge point sits off_dst bytes inside
	 * the compressed node cn_d = D, whose prefix bytes lie above it; wrap M
	 * under cn_d->key_bytes[0..off_dst) and replace cn_d at its own slot
	 * (d_dst->nfp).  Edge D (ed): the merge point's parent is the compressed
	 * node cn_p; wrap M under the WHOLE cn_p and replace cn_p at the grandparent
	 * slot d_dst->pnfp (pub_parent = d_dst->ppnf).  Either way reclaim the
	 * replaced compressed node -- ft_merge_build entered it (or, for Edge D, did
	 * not touch it) without recording the free.  ft_merge_wrap_prefix
	 * canonicalizes a single-child M and preserves the dst_origin of a
	 * compressed M's child.
	 *
	 * EXACT dst with an internal/absent parent (off_dst == 0, !ed): M itself
	 * replaces the subtree.  Canonicalize a non-root single-child internal merge
	 * top: when S and D contribute exactly one shared byte (a churned "ab"-prefix
	 * shape), ft_merge_build returns M as a 1-child internal with no
	 * external_nodes -- forbidden at a non-root position under skip mode
	 * (chain-compress invariant; cds_ft_verify catches it at the merge depth).
	 * Collapse it to a 1-byte compressed via the glue.  Only the TOP M can hit
	 * this; its lone child is always a FRESH recursion result, so the deferred
	 * child edge carries dst_origin=false correctly and disturbs no flip edge.
	 * A root dst merge point (d_dst->pnf == NULL) is exempt: a 1-child internal
	 * is canonical at the root.
	 */
	if (ks_dst || ed) {
		struct cds_ft_compressed_node *wrap_cn;
		uint8_t kbuf[FT_MAX_KEY_LEN];
		unsigned int wrap_depth, wrap_len;

		if (ed) {
			wrap_cn = ft_compressed_node_ptr(
				ft_resolve_skip_compressed(dst_ft, d_dst->pnf));
			wrap_len = wrap_cn->len;
			wrap_depth = (unsigned int) d_dst->depth - wrap_len;
			pub_parent = d_dst->ppnf;
			pub_slot = d_dst->pnfp;
		} else {	/* ks_dst */
			wrap_cn = ft_compressed_node_ptr(D);
			wrap_len = off_dst;
			wrap_depth = (unsigned int) d_dst->depth;
		}
		ft_glue_defer_free(&gd, wrap_cn, true);
		memcpy(&kbuf[wrap_depth], wrap_cn->key_bytes, wrap_len);
		pub = ft_merge_wrap_prefix(&ctx, kbuf, wrap_depth,
				wrap_depth + wrap_len, M, merged_keys);
		if (pub == FT_MERGE_OOM) {
			if (fresh_root)
				free_cds_ft_node_unpublished(src_ft, fresh_root);
			ft_glue_abort(dst_ft, &gd);
			ft_glue_abort(src_ft, &gs);
			return CDS_FT_STATUS_MEMORY_ERROR;
		}
	} else if (pub_parent) {
		pub = ft_compress_single_child_if_needed(dst_ft, M, &gd);
		if (pub == (struct cds_ft_inode_flag *) (long) -ENOMEM) {
			if (fresh_root)
				free_cds_ft_node_unpublished(src_ft, fresh_root);
			ft_glue_abort(dst_ft, &gd);
			ft_glue_abort(src_ft, &gs);
			return CDS_FT_STATUS_MEMORY_ERROR;
		}
	} else {
		pub = M;
	}

	/*
	 * Take or create the flip-txn: one latch per dst-origin re-parent edge,
	 * one for the merge-point forward slot, @ms_cap for the ordered-list
	 * interleave's boundary edges, and one per collided duplicate-chain splice
	 * (the src run tail-append, folded in below so the concatenation flips with
	 * the structure) -- structure, ordered list AND duplicate chains commit in
	 * ONE flip, so they share this txn.  Reserve it up front to that bound so
	 * every post-drain record (the structural edges, the INSTALLED-state cell
	 * edges and the splice edges, which append into the reserved head chunk) is
	 * allocation-free.
	 * The rekey hands in a pre-reserved txn (cannot fail); otherwise create one
	 * here, where failure aborts the still-invisible build (both tries pristine).
	 */
	{
		unsigned int nr_dst = 0;
		int j;

		for (j = 0; j < gd.nr_deferred; j++)
			if (gd.deferred[j].dst_origin)
				nr_dst++;
		txn = ft_flip_txn_take(pre_txn);
		if (!txn) {
			txn = ft_flip_txn_create(dst_ft);
			/*
			 * SIZE THE LOCK REGISTRY FROM THE PLAN, here where
			 * -ENOMEM still propagates.  The fenced overlap retires
			 * are what load it -- ft_glue_tombstone_free_list
			 * registers one word per !shared fenced holder, on BOTH
			 * sides -- and @cap_free is exactly that plan-time
			 * count.  Without this the registry grows mid-commit,
			 * past the abort-impossible point, where an allocation
			 * failure has nowhere to unwind to.
			 */
			if (txn && !ft_flip_txn_reserve_locks(txn,
					gd.cap_free + gs.cap_free
						+ FT_FLIP_TXN_FLOOR_LOCKS)) {
				ft_flip_txn_destroy(txn);
				txn = NULL;
			}
			if (txn && !ft_flip_txn_reserve(txn,
					nr_dst + 1 + ms_cap + gd.cap_free
						+ gd.nr_splices
						+ 1 /* §4.B parent guard */
						/* + count walk: the (merged_keys - cnt_dst) nr_keys ancestor
						 * edges (BULK fold), bounded by the merge-point depth */
						+ (dst_ft->rank_stats ? (int) dst_key_len + 1 : 0)
				/* the split-retire terminal's second word, if any */
				+ ft_glue_split_cn_reserve(dst_ft))) {
				ft_flip_txn_destroy(txn);
				txn = NULL;
			} else if (txn) {
				/*
				 * Created + reserved gd.cap_free free-list headroom:
				 * fuse each dst-overlap retire's freeze into @txn (atomic
				 * detach, doc/design/mcas-multiwriter-readiness.md §4.B),
				 * so the whole retired dst spine freezes dead atomically
				 * with the forward publish that unlinks it.  The rekey
				 * take() path keeps the standalone flip -- its pre_txn is
				 * pre-sized by the rekey with no room to grow here.
				 */
				gd.fuse_free_list = true;
			}
		}
	}
	if (!txn) {
		if (fresh_root)
			free_cds_ft_node_unpublished(src_ft, fresh_root);
		ft_glue_abort(dst_ft, &gd);
		ft_glue_abort(src_ft, &gs);
		return CDS_FT_STATUS_MEMORY_ERROR;
	}
	ft_glue_set_publish(dst_ft, &gd, pub_parent, pub_slot, pub);
	/*
	 * The dst forward publish commits through @txn (step 4 below); point
	 * gd->txn at it so ft_glue_apply_deferred's tombstone_free_list records
	 * the fused dst retires into the same txn.  A no-op alias when
	 * fuse_free_list stayed false (the rekey take() path).
	 */
	gd.txn = txn;

	/*
	 * Slot-canonical form of @pub for the @pub_slot stores (both the flip
	 * proxy's new target and the settle): a compressed @pub is published
	 * SKIP-ENCODED, exactly as a direct slot write would store it, so a reader
	 * resolving the proxy gets a value identical in encoding to a normal slot
	 * read and runs the same skip handling.  set_publish wired @pub's parent +
	 * skip_slot via the plain flag already; an internal @pub (EXACT only -- a
	 * compressed-wrap @pub is always compressed) needs no re-encode.
	 *
	 * @D_old is the flip proxy's old target -- the value @pub_slot currently
	 * holds.  Read it from the live slot: for the compressed shapes the descent
	 * resolved d->nf / d->pnf to the PLAIN flag while the slot holds the SKIP
	 * form, and *pub_slot is untouched until the flip (the build is invisible
	 * and apply_deferred wires back-pointers, not this forward slot).
	 */
	if (ft_node_compressed(pub))
		M_slot = ft_publish_compressed(dst_ft,
				ft_compressed_node_ptr(pub), pub);
	else
		M_slot = pub;
	/*
	 * WAITING load, not a raw one: @pub_slot is this flip's forward edge, so
	 * it enters @txn's own write set.  "Untouched until the flip" is true of
	 * THIS op's stores and says nothing about a PEER parking a flip proxy
	 * there; that proxy is a descriptor-record POINTER, and as an
	 * expected-old it trips urcu_txn_add's !urcu_txn_is_proxy self-check.
	 */
	D_old = urcu_txn_load(txn->mtxn, (void **) pub_slot,
		FT_FLIP_PROXY_TAG);

	/*
	 * Ordered list: capture the dst merge subtree's min head (the cursor for
	 * the post-commit interleave walk) and the region predecessor, while D is
	 * still intact (the build only copied its spine; the flip below moves its
	 * leaves into M).  The surviving src cells are spliced in after the commit.
	 */
	if (ms_ord) {
		ms_cursor = ft_ord_cell_ptr(rcu_dereference(
			ft_subtree_minmax_head(dst_ft, D, false)->prev));
		ms_prev = ft_ord_cell_resolve_ord(&ms_cursor->lnode.prev);
		/*
		 * The dst region run is [@ms_cursor .. D's max head]; @ms_succ is
		 * the cell following it (NULL at the list tail), the stop boundary
		 * for the interleave walk and the trailing survivor's successor.
		 */
		ms_succ = ft_ord_cell_resolve_ord(&ft_ord_cell_ptr(rcu_dereference(
			ft_subtree_minmax_head(dst_ft, D, true)->prev))->lnode.next);
		/*
		 * Capture src's merged-subtree (S) run endpoints now, while S is
		 * still intact, but defer the actual run-unlink until AFTER the
		 * last fallible step (ft_merge_unlink_src_subtree).  The run
		 * unlink is an externally observable ordered-list mutation; doing
		 * it here would leave src's list inconsistent if the src subtree
		 * unlink below OOMs and we roll the whole merge back.  The unlink
		 * still happens before the drain, so sync drains src ord-readers
		 * of the run too.
		 */
		ms_s_first = ft_subtree_minmax_head(dst_ft, S, false);
		ms_s_last = ft_subtree_minmax_head(dst_ft, S, true);
		/*
		 * Pre-allocate the interleave's edge scratch NOW, while the build
		 * is still abortable.  The interleave is collected pre-commit and
		 * its proxies recorded into the (already-reserved) structural @txn, so
		 * the commit tail has no allocation left and cannot degrade to a
		 * non-atomic per-edge fallback.  Each survivor run costs at most
		 * two visible edges, so @ms_cap (2*merged_keys+2) bounds @ms_edges.
		 */
		ms_edges = malloc((size_t) ms_cap * sizeof(*ms_edges));
		if (!ms_edges) {
			ft_flip_txn_destroy(txn);
			if (fresh_root)
				free_cds_ft_node_unpublished(src_ft, fresh_root);
			ft_glue_abort(dst_ft, &gd);
			ft_glue_abort(src_ft, &gs);
			return CDS_FT_STATUS_MEMORY_ERROR;
		}
		/*
		 * Capture the src run's key SUFFIXES (below the src merge point)
		 * NOW, while S is still attached and up-walkable.  The post-commit
		 * interleave merges them against the LIVE dst suffixes to rebuild
		 * the merged order WITHOUT walking the about-to-be-published merged
		 * structure -- so no proxy need be installed before the collect.
		 * Capturing them here (not at the collect) is mandatory:
		 * ft_merge_unlink_src_subtree below leaves S-root's parent stale,
		 * so a src up-walk after the unlink could run into freed / relocated
		 * structure.  The run length is bounded by @cnt_src (the src
		 * subtree's key count >= its distinct heads); the variable-length
		 * suffix bytes pack into a realloc-growable pool addressed by offset
		 * (a byte pointer would dangle across the realloc).  Fallible
		 * pre-pass before the last fallible step -> any OOM aborts the
		 * still-invisible build, both tries pristine.
		 */
		ms_src_caps = malloc((size_t) (cnt_src + 8) * sizeof(*ms_src_caps));
		if (ms_src_caps) {
			struct ft_ord_cell *sc = ft_ord_cell_ptr(
				rcu_dereference(ms_s_first->prev));
			struct ft_ord_cell *slast = ft_ord_cell_ptr(
				rcu_dereference(ms_s_last->prev));
			size_t s_max_len = dst_ft->group->max_key_len;
			size_t pool_cap = 0, pool_len = 0;
			uint8_t sbuf[FT_MAX_KEY_LEN];
			bool oom = false;

			for (;;) {
				size_t sfl = ft_rebuild_key_upwalk(dst_ft, sc,
						sbuf, s_max_len);
				size_t suf_len;

				assert(sfl >= src_key_len &&
					ms_nsrc < cnt_src + 8);
				suf_len = sfl - src_key_len;
				if (pool_len + suf_len > pool_cap) {
					size_t ncap = pool_cap ? pool_cap * 2 : 256;
					uint8_t *np;

					while (ncap < pool_len + suf_len)
						ncap *= 2;
					np = realloc(ms_src_pool, ncap);
					if (!np) {
						oom = true;
						break;
					}
					ms_src_pool = np;
					pool_cap = ncap;
				}
				memcpy(ms_src_pool + pool_len,
					sbuf + (s_max_len - sfl) + src_key_len,
					suf_len);
				ms_src_caps[ms_nsrc].cell = sc;
				ms_src_caps[ms_nsrc].suffix_off = pool_len;
				ms_src_caps[ms_nsrc].suffix_len = suf_len;
				pool_len += suf_len;
				ms_nsrc++;
				if (sc == slast)
					break;
				sc = ft_ord_cell_resolve_ord(&sc->lnode.next);
			}
			if (oom) {
				free(ms_src_pool);
				ms_src_pool = NULL;
				free(ms_src_caps);
				ms_src_caps = NULL;
			}
		}
		if (!ms_src_caps) {
			free(ms_edges);
			ft_flip_txn_destroy(txn);
			if (fresh_root)
				free_cds_ft_node_unpublished(src_ft, fresh_root);
			ft_glue_abort(dst_ft, &gd);
			ft_glue_abort(src_ft, &gs);
			return CDS_FT_STATUS_MEMORY_ERROR;
		}
	}

	/*
	 * 1. Unlink the merge source from src.  Root src: swap in the pre-
	 *    allocated empty root.  Non-root src: detach its branch in place --
	 *    the LAST fallible step.  On its OOM both tries are pristine (the
	 *    detach self-undoes, the cluster is still unpublished), so abort the
	 *    build; this preserves the no-rollback property.  Then drain src
	 *    readers of the moved content.
	 *
	 *    ===== Everything from the drain onward is failure-free. =====
	 */
	struct ft_detach_run sdrun = { .into = NULL };

	if (ms_ord && !root_src) {
		/*
		 * Non-root src: fuse the src run-unlink into ft_merge_unlink_src_
		 * subtree's structural unlink flip (EXCISE-ONLY, into == NULL -- the
		 * run cells disperse to dst via the post-commit interleave), exactly
		 * as the subpos src side does, closing the src-side disappear window.
		 */
		sdrun.rfirst = ms_s_first;
		sdrun.rlast = ms_s_last;
	}
	/*
	 * Pre-reserve the src-side ordered-list commit txn before the (still
	 * fallible) src unlink.  Whichever fires -- the root-src root+endpoint
	 * swap or the rare unfused non-root run-unlink -- is the src side's
	 * commit, un-abortable once the structural unlink is public, so it commits
	 * through this pre-reserved txn (ft_ord_cell_flip_into).  Sized to the
	 * larger bound (4-edge run-unlink >= 3-edge root swap).  OOM here aborts
	 * the still-invisible build (both tries pristine).  The lone-edge list-off
	 * paths (ft_root_edge_flip) need no txn, so reserve only when ms_ord.
	 *
	 * ☠ WHY THE SRC SIDE IS TRANSACTED AT ALL, given cds_ft_merge_at and
	 * cds_ft_graft both REJECT a non-exclusive src (BUSY_ERROR), and an
	 * exclusive trie has no concurrent reader or writer to be atomic against:
	 *
	 * because that rejection is `src_ft != dst_ft` -- SAME-TRIE is excluded
	 * from it on purpose.  cds_ft_rekey_{graft,merge} reach this worker
	 * through ft_merge_at_inner(ft, ..., ft, ...) with @rekey set, so on that
	 * path @src_ft IS @dst_ft: the LIVE, SHARED, concurrently-read trie.  Its
	 * "src side" is therefore exactly as contended as its dst side, and every
	 * edge here needs the same atomicity a cross-trie merge's src does not.
	 *
	 * So "the src is always exclusive now, drop the src-side txn" is a sound
	 * reading of the entry gates and a WRONG conclusion about this body.  The
	 * exclusivity that would justify it belongs to the CROSS-TRIE callers
	 * only; this code is shared with the one caller that has none.
	 */
	struct ft_flip_txn *src_side_txn = NULL;

	/*
	 * Size the src-side commit txn per shape.  A root src fuses the gs
	 * free-list freeze into its root swap / lone root flip (atomic detach,
	 * doc §4.B): + gs.cap_free tombstone edges (cnt.nf_src + 8, the exact
	 * upper bound ft_glue_defer_free asserts against).  A non-root src
	 * commits its run-unlink through a plain FT_ORD_CELL_RUN_UNLINK txn; its
	 * gs freeze stays standalone, applied below AFTER
	 * ft_merge_unlink_src_subtree's (still fallible) unlink so a rolled-back
	 * merge never freezes-then-strands (full non-root fusion -- into
	 * ft_detach_node's flip -- is a §4.B residual).  A list-off root with an
	 * EMPTY gs keeps its bare lone-edge store (no txn, no grace period).
	 * gs is fully populated by ft_merge_build above, so nr_free/cap_free are
	 * final here.  (gd is stamped in ft_glue_apply_deferred for the dst
	 * forward publish.)
	 */
	if (ms_ord) {
		src_side_txn = ft_flip_txn_create_bounded(src_ft, root_src ?
			FT_ROOT_LIST_SWAP_MAX_EDGES + (unsigned int) gs.cap_free :
			FT_ORD_CELL_RUN_UNLINK_MAX_EDGES);
		if (!src_side_txn) {
			free(ms_src_pool);
			free(ms_src_caps);
			free(ms_edges);
			ft_flip_txn_destroy(txn);
			if (fresh_root)
				free_cds_ft_node_unpublished(src_ft, fresh_root);
			ft_glue_abort(dst_ft, &gd);
			ft_glue_abort(src_ft, &gs);
			return CDS_FT_STATUS_MEMORY_ERROR;
		}
	} else if (root_src && gs.nr_free) {
		/* List-off root with retires: 1 root edge + gs.cap_free tombstones. */
		src_side_txn = ft_flip_txn_create_bounded(src_ft,
			1u + (unsigned int) gs.cap_free);
		if (!src_side_txn) {
			free(ms_src_pool);
			free(ms_src_caps);
			free(ms_edges);
			ft_flip_txn_destroy(txn);
			if (fresh_root)
				free_cds_ft_node_unpublished(src_ft, fresh_root);
			ft_glue_abort(dst_ft, &gd);
			ft_glue_abort(src_ft, &gs);
			return CDS_FT_STATUS_MEMORY_ERROR;
		}
	}

	/*
	 * Dup-chain lock-set (MW LOCK_FINE): take the node lock of every
	 * distinct holder whose chain step 3c appends a collided src run to, so
	 * the tail walk + append run under the same per-node lock insert, remove,
	 * promote and replace take on that chain.  HERE is the last point where a
	 * miss is free: every fallible allocation above has succeeded and the src
	 * unlink below is the point of no return, so an -EAGAIN unwinds a build
	 * that is still entirely invisible.  The locks are held across the unlink,
	 * the drain and the single commit that installs the appends, and released
	 * just after it.
	 *
	 * UNDOING THE ATTEMPT IS TRIVIAL, which is what makes the bail cheap:
	 * cds_ft_merge_at consumes an EXCLUSIVE source, so nothing is published and
	 * nothing has moved -- discarding the fresh cluster leaves both tries
	 * byte-for-byte as they were, and the caller just re-descends.  That is why
	 * contention needs no place in enum cds_ft_status (see @contended).
	 *
	 * SKIPPED when the CALLER pre-reserved @pre_txn.  That marks a caller which
	 * has ALREADY passed its own point of no return and pre-built everything so
	 * this placement cannot fail -- today the staged rekey (detach -> GP ->
	 * merge back), whose content is already out of the trie and has nowhere to
	 * go if we bail.  A failable acquire under an unfailable placement is a
	 * contradiction, and retrying there would re-draw a node reserve that a
	 * discarded attempt returns to the ARENA, not to the reserve.  Same-trie
	 * rekey gets its chain exclusion from the in-place one-decide writer's
	 * up-front DLM lock set instead -- a single commit with reader two-pass
	 * coherence, no detach and so no unfailable tail -- not from a retrofit
	 * here.  So this leaves the staged rekey's appends where they are today.
	 *
	 * No-op on a non-lock_fine trie or a collision-free merge, which is every
	 * merge of disjoint key sets -- the batch-staging workload pays nothing.
	 */
	if (!unfailable && ft_glue_acquire_splice_holders(dst_ft, &gd)) {
		free(ms_src_pool);
		free(ms_src_caps);
		free(ms_edges);
		ft_flip_txn_destroy(txn);
		if (src_side_txn)
			ft_flip_txn_destroy(src_side_txn);
		if (fresh_root)
			free_cds_ft_node_unpublished(src_ft, fresh_root);
		ft_glue_abort(dst_ft, &gd);
		ft_glue_abort(src_ft, &gs);
		*contended = true;
		return CDS_FT_STATUS_MEMORY_ERROR;
	}

	/*
	 * PRE-ACQUIRE THE PUBLISH TARGET, here, for the same reason: this is the
	 * last point at which failing to get it is free.
	 *
	 * The forward publish used to acquire @pub_parent at step 3 -- AFTER the src
	 * unlink -- through ft_flip_txn_lock_or_guard_parent, whose miss is not a
	 * failure it can report: it sets @acquire_miss, and the commit then ABORTS.
	 * By then the src subtree is unlinked and the merged cluster is unpublished,
	 * so the keys exist in NEITHER trie, and the ignored commit status let this
	 * function return CDS_FT_STATUS_OK on top of it.  Silent data loss with a
	 * success code, measured across most merge shapes.
	 *
	 * @pub_parent is above the overlap spine, so it is never in the fenced set;
	 * it CAN be a dup-chain holder we just locked (an external dst merge point),
	 * so take that fence over rather than re-marking our own word -- the
	 * self-deadlock this file has now hit twice.  Either way the holder rides
	 * @gd to the publish, which records the {LOCK|s -> s} release directly
	 * and so never consults lock_or_guard.  With no acquire left to miss, that
	 * abort cause is gone rather than merely less likely.
	 *
	 * A miss here bails exactly like the splice acquire above: both tries
	 * pristine, contention reported, caller re-descends.
	 */
	if (!unfailable && dst_ft->lock_fine && pub_parent) {
		struct cds_ft_metadata *pm = ft_flag_to_metadata(dst_ft, pub_parent);
		uintptr_t psnap = 0;

		struct ft_lock_ctx pctx;
		struct ft_held_anchor ph;
		unsigned int pdep;
		bool took = ft_glue_splice_holder_take(&gd, pm, &psnap);

		ft_glue_lock_ctx(&gd, &pctx);
		if (!took && (!ft_lock_ctx_depth_of(dst_ft, &pctx, pub_parent,
					&pdep) ||
				ft_acquire_member(dst_ft, &pctx, pub_parent, pm,
					pdep, &ph))) {
			free(ms_src_pool);
			free(ms_src_caps);
			free(ms_edges);
			ft_flip_txn_destroy(txn);
			if (src_side_txn)
				ft_flip_txn_destroy(src_side_txn);
			if (fresh_root)
				free_cds_ft_node_unpublished(src_ft, fresh_root);
			ft_glue_abort(dst_ft, &gd);
			ft_glue_abort(src_ft, &gs);
			*contended = true;
			return CDS_FT_STATUS_MEMORY_ERROR;
		}
		/*
		 * The holder is whichever word protects @pub_parent: the splice
		 * fence this op already took over, or the anchor just acquired.
		 */
		/*
		 * A SHARED acquire owes no release.  The claim above -- that
		 * @pub_parent is above the overlap spine and so never in the
		 * fenced set -- holds for the NODE and not for its ANCHOR:
		 * coarsening can put that anchor on an overlap node this op has
		 * already fenced, and root-only puts EVERY member on one word.
		 * Claiming it here would record a second terminal on a word whose
		 * fenced retire already owns one, which the engine poisons.
		 */
		if (!took && ph.shared) {
			gd.publish_parent_holder = NULL;
			gd.publish_parent_snap = 0;
		} else {
			gd.publish_parent_holder = took ? pm : ph.lock;
			gd.publish_parent_snap = took ? psnap : ph.lock_snap;
		}
	}

	if (root_src) {
		/*
		 * A root src moves the WHOLE source, so its run is the whole src
		 * ordered list: fuse the src->root swap with the head/tail clear into
		 * ONE flip (ft_root_list_swap_publish), so a src reader never sees src
		 * structurally empty but its ordered-list front still populated.  The
		 * run cells keep their internal links (only the head/tail endpoints
		 * flip), so the post-commit interleave still re-homes them to dst.
		 */
		if (ms_ord) {
			/*
			 * Empty src's sentinel (relink_dest NULL): the run cells are
			 * re-homed into dst by the post-commit interleave, which sets each
			 * survivor's links individually (collided heads are freed).
			 *
			 * Fuse the gs free-list freeze into this root-swap txn (atomic
			 * detach, doc §4.B): record each retired src-overlap node's
			 * tombstone into src_side_txn first, then the swap records the
			 * root + sentinel endpoint edges into the same txn and commits
			 * them all in ONE flip.  An empty gs records nothing.
			 */
			gs.txn = src_side_txn;
			gs.fuse_free_list = true;
			ft_glue_tombstone_free_list(&gs);
			ft_root_list_swap_publish(src_ft, src_side_txn,
				&src_ft->root,
				src_ft->root, ft_node_flag(fresh_root, 0),
				ft_ord_first(src_ft), NULL,
				ft_ord_last(src_ft), NULL, NULL, false);
			src_side_txn = NULL;	/* consumed */
		} else if (gs.nr_free) {
			/*
			 * List-off root with retires: fuse the gs free-list freeze into
			 * the root flip (atomic detach, doc §4.B).  Record the lone root
			 * edge + each retired node's tombstone into src_side_txn and
			 * commit them in ONE flip (a group flip, like the list-on path),
			 * rather than a bare store followed by standalone freezes.  The
			 * root edge normalizes to the same FT_FLIP_PROXY_TAG the swap uses.
			 */
			ft_flip_txn_record_root(src_side_txn,
				(void **) &src_ft->root,
				(void *) src_ft->root,
				(void *) ft_node_flag(fresh_root, 0));
			gs.txn = src_side_txn;
			gs.fuse_free_list = true;
			ft_glue_tombstone_free_list(&gs);
			ft_flip_txn_commit(src_ft, src_side_txn);
			src_side_txn = NULL;	/* consumed */
		} else {
			/*
			 * No ordered list, no retires: src->root is the only reader-
			 * visible slot.  Express the lone root edge as a single-edge flip
			 * descriptor (one release store, like a bare rcu_assign_pointer)
			 * so the swap is MCAS-expressible like the list-on path.
			 */
			ft_root_edge_flip(src_ft, &src_ft->root,
				src_ft->root, ft_node_flag(fresh_root, 0));
		}
		FT_TP(root_publish, (const void *) src_ft, (const void *) src_ft->root);
	} else if (ft_merge_unlink_src_subtree(src_ft, src_key, src_key_len,
				cnt_src, ms_ord ? &sdrun : NULL, &gs) < 0) {
		/*
		 * OOM in the last fallible step: @src_ft is left pristine (the run was
		 * not yet applied -- it commits in ft_detach_node's flip, past the
		 * fallible alloc), so abort the still-invisible build.
		 */
		free(ms_src_pool);
		free(ms_src_caps);
		free(ms_edges);
		ft_flip_txn_destroy(txn);
		if (src_side_txn)
			ft_flip_txn_destroy(src_side_txn);
		ft_glue_abort(dst_ft, &gd);
		ft_glue_abort(src_ft, &gs);
		return CDS_FT_STATUS_MEMORY_ERROR;
	}
	/*
	 * Non-root src: gs (the src overlap-spine free-list) is now frozen dead
	 * ATOMICALLY with the unlink -- ft_merge_unlink_src_subtree threaded gs
	 * into ft_detach_node as its @retire_glue, so each retired node's
	 * tombstone rode the SAME commit_txn flip that unlinked S (doc §4.B
	 * atomic detach).  On the OOM abort above ft_detach_node left gs unmarked
	 * (src stays pristine, gs still live).  A root src froze gs fused into its
	 * root swap / flip above.
	 */
	/*
	 * Now that the last fallible step has committed, remove src's merged
	 * subtree (S) run from src's ordered list: its cells disperse to dst
	 * (survivors) or are freed (collisions).  Deferred to here so an OOM in
	 * the src unlink above leaves src's list untouched on rollback; done
	 * before the drain so sync drains src ord-readers of the run too.  Skipped
	 * for a root src (fused into the root swap above) and for a non-root src
	 * whose unlink already fused the run (sdrun.armed).
	 */
	if (ms_ord && !root_src && !sdrun.armed) {
		ft_ord_cell_run_unlink(src_ft, src_side_txn, ms_s_first,
			ms_s_last);
		src_side_txn = NULL;	/* consumed */
	}
	/* Reserved but unused: a non-root run whose unlink already fused it. */
	if (src_side_txn)
		ft_flip_txn_destroy(src_side_txn);
	/*
	 * NO SRC DRAIN.  This worker is now CROSS-TRIE ONLY -- the same-trie
	 * rekey moved to ft-rekey.h -- and ft_merge_at_inner's gates leave both
	 * src_ft != dst_ft and src_ft->exclusive holding unconditionally by the
	 * time it calls here.  An exclusive trie has no concurrent reader, so
	 * there is nothing to wait out: the grace period that used to stand here
	 * was serving the same-trie caller, which no longer reaches this body.
	 *
	 * Asserted rather than deleted outright so the precondition stays
	 * VISIBLE: a future caller that reaches this worker with a shared src
	 * would otherwise silently skip a drain it needs.
	 */
	assert(src_ft->exclusive);

	/*
	 * 2. Re-parent the SRC-origin referenced subtrees directly: the src
	 *    drain above made them unreachable to readers, and the cluster is
	 *    not yet forward-published, so this is invisible.  dst-origin
	 *    edges are NOT applied here -- they go through the flip.
	 */
	ft_glue_apply_deferred(dst_ft, &gd);

	/*
	 * 3. Record the structural edges into @txn (still PREPARE): every
	 *    dst-origin child's parent re-parent and the merge-point forward slot.
	 *    ft_glue_record_back_edge mirrors ft_set_parent's child-kind dispatch
	 *    (reading the field's current value as the old target) and runs the
	 *    writer-only slot bookkeeping early; the commit's settle below writes
	 *    each field to its new parent, subsuming the old step-7 ft_set_parent.
	 *    Nothing is parked yet -- commit's auto-install stages every proxy
	 *    (selector 0, resolving to old) so up-walks and the root descent still
	 *    see the pre-merge dst until the flip.
	 */
	{
		int j;

		for (j = 0; j < gd.nr_deferred; j++) {
			if (!gd.deferred[j].dst_origin)
				continue;
			ft_glue_record_back_edge(dst_ft, txn,
				gd.deferred[j].child, gd.deferred[j].parent,
				gd.deferred[j].slot);
		}
		/* Forward publish: old dst subtree -> merged cluster. */
		/*
		 * VALIDATE (§4.B) / LOCK_FINE (step 6, §9.4 M-2): acquire the LIVE dst
		 * parent above the overlap spine as a RELEASE lock.  The forward publish
		 * is a same-slot REPLACE (nr_child invariant, §9.4 finding 1) so
		 * pub_parent is a value-swap survivor not recompacted -- guard-fallback
		 * on an acquire miss is correct; non-lock_fine falls to the §4.B guard.
		 *
		 * UNLESS WE ALREADY HOLD IT.  When the dst merge point is an EXTERNAL
		 * node, the collided head IS that node and its chain holder is its
		 * immediate parent -- @pub_parent.  Re-marking a word this op already
		 * fenced MISSES, and a miss now sets acquire_miss and ABORTS a commit
		 * with no bail path left (the src is already unlinked).  Hand the held
		 * fence to the txn instead: hold_or_lock records the {LOCK|s -> s}
		 * release, the guard's strictly stronger twin, and the txn owns the
		 * unlock from here (so the take clears our entry).
		 */
		/*
		 * The holder was acquired BEFORE the src unlink (see the pre-acquire
		 * above), whether by marking @pub_parent here or by taking over the
		 * dup-chain fence when the two coincide.  Recording its release
		 * directly is what keeps this publish off lock_or_guard, whose miss
		 * would abort a commit that has no way left to fail safely.  NULL
		 * holder = non-lock_fine or the unfailable caller, which routes to the
		 * ordinary acquire-or-guard exactly as before.
		 */
		{
			struct ft_lock_ctx mctx;

			ft_glue_lock_ctx(&gd, &mctx);
			ft_flip_txn_hold_or_lock_parent(dst_ft, txn, &mctx,
				pub_parent, FT_DEPTH_FROM_DESCENT,
				gd.publish_parent_holder,
				gd.publish_parent_snap);
		}
		/*
		 * OWNERSHIP TRANSFER (mirrors the graft): @txn's registry now owns
		 * this fence -- a commit consumes it through the recorded release, an
		 * abort clears it -- so drop the glue's claim.  There is no bail left
		 * between here and the commit, but leaving a stale holder behind is
		 * the foot-gun that makes the NEXT bail added here a double clear.
		 */
		gd.publish_parent_holder = NULL;
		gd.publish_parent_snap = 0;
		/*
		 * @pub_slot is d_dst->nfp: &dst_ft->root at depth 0 (whose
		 * record is always MW and ignores the owner), and a child slot
		 * inside @pub_parent below it -- the node the hold_or_lock
		 * above put in this txn's registry.
		 */
		ft_flip_txn_record_publish(txn, dst_ft,
			pub_parent ? ft_flag_to_metadata(dst_ft, pub_parent) :
				NULL,
			pub_slot, D_old, M_slot);
	}

	/*
	 * 3b. Ordered list: collect the interleave and record its cell edges into
	 *    the SAME txn, STILL IN PREPARE -- no install before the collect.
	 *    The merged order is reconstructed by a two-pointer merge of the two
	 *    LIVE cell runs (the dst region up-walked over the intact D, and the
	 *    src run's suffixes captured pre-unlink), so the collect does NOT need
	 *    the about-to-be-published merged structure -- the last
	 *    append-after-install is gone, and every edge is recorded before
	 *    install.  Structure + ordered list thus commit in ONE flip (commit
	 *    auto-installs every recorded proxy, then flips): a reader never sees
	 *    the merged structural minimum ahead of the ordered-list front.  The
	 *    collect pre-sets the surviving cells' own links invisibly (not yet
	 *    ord-reachable) and returns only the visible boundary edges.  It runs
	 *    before the splice fold (step 3c), but collisions are invariant: a
	 *    collided src head is a floating duplicate (only in the splice record),
	 *    never a distinct reachable head, so the merge enumerates the same heads
	 *    either way -- and 3c reads each demoted head's cell from its still-intact
	 *    src-run prev, which this collect leaves untouched.
	 */
	if (ms_ord) {
		unsigned int i;

		ms_n = ft_merge_ord_interleave_collect(dst_ft, dst_key_len,
			ms_cursor, ms_succ, ms_prev, ms_src_caps, ms_nsrc,
			ms_src_pool, ms_edges, /*record_all=*/ false,
			/*ncollide=*/ NULL);
		/*
		 * ☠ A RAW record_tag LOOP OVER ORD EDGES, which the sibling
		 * graft path deliberately does NOT do (see the comment at
		 * ft_ord_cell_record_into_ft's caller there): every edge here
		 * takes the structural_sw dispatch, so a CELL edge parks SW
		 * under an armed txn even though no cell carries a node lock.
		 * Sound today only because the modes that arm -- COARSE and
		 * exclusive -- exclude trie-wide.  The per-edge @owner is what
		 * stops it at the PHASE B arm: a cell's owner is NULL, so the
		 * record-time check refuses the park instead of taking it
		 * silently.  Routing this loop through the tag-dispatching
		 * recorder is the real fix and belongs with the site's arm.
		 */
		for (i = 0; i < ms_n; i++)
			ft_flip_txn_record_tag(txn, ms_edges[i].owner,
				(void **) ms_edges[i].slot,
				(void *) ms_edges[i].old_target,
				(void *) ms_edges[i].new_target,
				ft_edge_tag(&ms_edges[i]));
	}

	/*
	 * 3c. Duplicate-chain concatenation: fold each collided src run's
	 *    tail-append into the SAME txn, still in PREPARE.  Recorded AFTER the
	 *    interleave collect (3b) so each demoted src head's cell is captured
	 *    from its intact src-run prev before the append overwrites it, and
	 *    BEFORE the commit so the src duplicates become reachable ATOMICALLY
	 *    with the merged structure -- a collided key never momentarily shows
	 *    only its dst duplicates (the old post-commit apply's window).  Each
	 *    splice is one forward edge (dst tail -> src run), so only the tail
	 *    carries a proxy; src_head->next rides along.  The rekey take() path
	 *    reaches this too -- cds_ft_rekey_merge unions into an OCCUPIED
	 *    destination, so a full key present on both sides collides there like
	 *    any other merge, which is why that path's pre-reservation budgets one
	 *    splice edge per moved key.  (Only cds_ft_rekey_graft demands an empty
	 *    destination, and it never reaches the spine copy.)
	 *
	 *    Every append here runs under the chain holder's node lock, taken
	 *    before the point of no return by ft_glue_acquire_splice_holders and
	 *    released after the commit below -- so the walk to the tail cannot race
	 *    a peer's append/unchain/promote on the same chain.
	 */
	ft_glue_record_splices(dst_ft, &gd, txn);

	/*
	 * Order-statistics fold (BULK): record the dst net key-count delta
	 * (merged_keys - cnt_dst) walk from @pub_parent up into the SAME txn, so
	 * the aggregate flips ATOMICALLY with the merged spine's forward publish
	 * (step 4) -- exact under concurrent writers.  @pub is a fresh node
	 * already carrying merged_keys, so the walk begins one level up at the
	 * stable @pub_parent.  A no-op when rank stats off or the count is
	 * unchanged.
	 */
	if (pub_parent && merged_keys != cnt_dst)
		ft_flip_txn_record_count_parent(dst_ft, txn, pub_parent,
			(long) merged_keys - (long) cnt_dst);

	/*
	 * 4. Commit: one selector flip switches every dst-origin parent, the
	 *    forward slot, AND every interleave cell edge from old to merged,
	 *    atomically, then settles each slot to its direct merged target.  (List
	 *    off with no dst-origin re-parent and no collided splice reduces to a
	 *    single release store of the forward slot -- no proxy, no grace period.)
	 *    Because the forward
	 *    slot flips with the back-pointers, a reader (descend then up-walk) only
	 *    progresses old->merged; and the ordered-list front advances in the same
	 *    instant the merged minimum becomes reachable.
	 */
	{
		enum urcu_txn_status mst = ft_flip_txn_commit(dst_ft, txn);

		/*
		 * The flip did not happen, so no fenced retire took effect and this
		 * op owns none of those frees -- renounce them before the step-7
		 * reclaim.  The dominant reason a fenced terminal aborts is a PEER
		 * retiring the node under our fence (the retire primitives do not
		 * honour LOCK), and that peer owns the reclaim: freeing here
		 * would be a double free on top of an already-lost merge.
		 *
		 * The merge itself still cannot recover -- the src is unlinked by
		 * now, which is the pre-existing abort-after-point-of-no-return
		 * exposure this shares with the unlocked pub_parent publish -- but
		 * it must not corrupt the arena on the way out.
		 */
		if (mst != URCU_TXN_STATUS_OK)
			ft_glue_fenced_renounce_free(&gd);
	}

	/*
	 * 5. Drop the dup-chain holder locks: the appends are installed, so peers
	 *    may mutate those chains again.  BEFORE the step-7 reclaim below --
	 *    most holders are dst overlap-spine nodes this merge retires, and the
	 *    fence has to come off while the node is still there to clear.  (The
	 *    fence survives the commit either way: the free-list retire records a
	 *    plain {LOCK|s -> LOCK|s|TOMBSTONE} upgrade, and an aborted
	 *    commit leaves {LOCK|s}.  The one holder that IS the publish target
	 *    was handed to @txn above and is already released by its flip.)
	 */
	ft_glue_release_splice_holders(&gd);
	/*
	 *    Same for the overlap-spine plan-locks: a COMMITTED fenced retire
	 *    consumed each fence into TOMBSTONE (clear_if_held no-ops), while an
	 *    ABORTED commit left {LOCK|s} that must come off or every later peer
	 *    publish into that node fails forever.  One unconditional sweep covers
	 *    both, which is why no per-outcome bookkeeping is kept.  Before the
	 *    step-7 reclaim, while the nodes are still addressable.
	 */
	ft_glue_clear_fenced(&gd);

	/*
	 * 6. The dst net key-count delta (merged_keys - cnt_dst) is FOLDED into
	 *    the step-4 commit above (recorded from @pub_parent into @txn before
	 *    the flip), so it goes live ATOMICALLY with the merged spine -- no
	 *    post-commit propagate walk.
	 */

	/*
	 * 7. Reclaim the old overlap spines (src-side to src, dst-side to dst).
	 *    The flip-txn was already committed-and-reclaimed at step 4 (commit
	 *    settled every proxied slot -- the dst-origin parents, the forward slot
	 *    and the interleave cell edges -- to its direct merged target, so the
	 *    proxies are unreferenced; its parked block is deferred through the FT
	 *    flavor, or freed in place when the commit owed no grace period).  No dst
	 *    synchronize_rcu -- the flip subsumed the dst drain.
	 */
	ft_glue_free_old(src_ft, &gs);
	ft_glue_free_old(dst_ft, &gd);

	/*
	 * 8. Free the collided (demoted) src heads' cells.  The interleave (folded
	 * into the flip above) already rewired the surviving cells' stale src-run
	 * links away from these cells, so they are unreachable to new readers, and
	 * the grace-period defer inside the free covers readers already holding such
	 * a link or parked on a demoted head.
	 */
	if (ms_ord) {
		free(ms_edges);
		free(ms_src_caps);
		free(ms_src_pool);
	}
	ft_glue_free_collided_cells(dst_ft, &gd);

	ft_glue_fini(&gd);
	ft_glue_fini(&gs);
	return CDS_FT_STATUS_OK;
}

/*
 * Build-invisible graft of a SUB-position source subtree into a dst position
 * ABSENT at @dst_key (the diverged-dst case) WITHOUT re-rooting the payload --
 * the leak-free reorder for cds_ft_merge_at's residual shapes.  Modeled on
 * ft_graft_keylen's commit, but the source side is ft_merge_spine_copy's: the
 * payload stays @payload (= d_src->nf, referenced in place, not detached into a
 * fresh root), and ft_merge_unlink_src_subtree is the single last fallible step
 * (it self-undoes on OOM, preserving @payload).  Because the dst attach is built
 * invisibly BEFORE that unlink and published failure-free AFTER the drain,
 * nothing can strand the moved externals on an allocation failure -- no
 * rollback, hence no leak (contrast the legacy detach-then-graft fallback).
 *
 * Covers both diverged-dst shapes: a GLUE diverge (the dst key diverges inside a
 * compressed node, so ft_graft_build assembles the whole split cluster invisibly
 * -- no reserve) and a NOSPLIT point (an empty slot at @dst_key, or a built
 * branch / displaced external -- ft_store_at_graft_point grafts it post-drain
 * drawing from a pre-filled reserve so it cannot fail on the arena).  *handled is
 * always set true here, so the caller never falls through.
 *
 * @okey_dst / @okey_src are ORDINAL; @dst_key is the APPLICATION dst key (the
 * ordered-list splice-pos descent remaps it).  @payload is the source subtree to
 * move with @cnt_src unique keys: d_src->nf for an EXACT source, or, for a
 * KEY_SHORTER source, cn_s->child with @okey_dst / @dst_key / @dst_key_len
 * already EXTENDED by the residual cn_s bytes (the caller's reduction to EXACT).
 * @okey_src / @src_key_len remain the ORIGINAL source key (the unlink overshoots
 * cn_s on its own).
 */
static
enum cds_ft_status ft_merge_graft_subpos_inplace(struct cds_ft *dst_ft,
		struct cds_ft *src_ft,
		const uint8_t *okey_dst, const uint8_t *dst_key, size_t dst_key_len,
		const uint8_t *okey_src, size_t src_key_len,
		struct cds_ft_inode_flag *payload, unsigned long cnt_src,
		bool *handled)
{
	struct ft_glue glue;
	struct cds_ft_alloc_reserve reserve;
	struct ft_descent d;
	enum ft_graft_prep prep;
	struct cds_ft_inode_flag *attached_nf;
	bool ms_ord = dst_ft->group->ordered_list_set;
	struct ft_ord_cell *pred = NULL, *succ = NULL;
	struct ft_ord_cell *run_first = NULL, *run_last = NULL;
	struct cds_ft_node *s_first = NULL, *s_last = NULL;
	struct ft_graft_run mrun;
	struct ft_graft_run *run_arg = NULL;
	struct ft_detach_run srun = { .into = NULL, .armed = false };
	struct ft_detach_run *srunp = NULL;
	size_t src_max, nm, dm;
	/*
	 * MW LOCK_FINE drop: the "failure-free" commit below is only unfailable
	 * under the FT-wide lock -- under the drop its MCAS commit (GLUE) or its
	 * recompact prepare (NOSPLIT) can conflict with a peer and fail AFTER
	 * @ft_merge_unlink_src_subtree already excised the source subtree.  That
	 * unlink PRESERVES @payload (owned by this op, src pristine-empty), so
	 * the whole dst-side attach (re-descend -> build -> commit) is a
	 * retryable one-shot: @already_unlinked guards the src-side excise + the
	 * ordered-list run capture so a retry re-runs only the dst attach, and
	 * @glue.fence_split_cn fences the compressed divergence node (as
	 * cds_ft_graft) so a concurrent grow of it is arbitrated (fence miss ->
	 * FT_GRAFT_PREP_RETRY -> re-descend).  Mirrors cds_ft_graft's post-swap
	 * store retry.
	 */
	bool already_unlinked = false;
	struct ft_flip_txn *run_unlink_txn = NULL;
	struct ft_flip_txn *run_splice_txn = NULL;

	*handled = false;

	/*
	 * Classify the dst graft point with a build-invisible prep.  A GLUE
	 * diverge builds the whole split cluster into @glue (referencing @payload
	 * by deferred edge), published failure-free after the drain.  A NOSPLIT
	 * point is grafted post-drain by ft_store_at_graft_point drawing from a
	 * pre-filled reserve, so it likewise cannot fail on an arena allocation.
	 * Either way the source unlink is the single last fallible step, so an OOM
	 * leaves both tries pristine -- no rollback, no leak.
	 */
	unsigned long rm_depth __attribute__((unused)) = 0;
	/*
	 * PERSISTENT OP HANDLE for this retry loop.  Without one, every attempt
	 * built its transaction with the UNBOUND ft_flip_txn_create(): a fresh
	 * handle whose retry age restarts at zero, so urcu_txn_conflict() never
	 * advances it, urcu_txn__self_qualifies() is never reached, and the
	 * writer can never take its per-trie FIFO turn.  A contended writer then
	 * livelocks by construction -- ft_flip_txn_create's own docstring says so,
	 * and this loop was measured spinning 266-587 attempts deep on ONE op in
	 * runs that PASS, with the tail reaching 1036 under load.  The sibling
	 * loops that already have a handle top out around 90 on the same workload.
	 *
	 * THE BRACKET IS CONDITIONAL, and @rm_open is a VARIABLE rather than a
	 * re-test of @rm_bracket at each exit: a re-test is a second chance to
	 * disagree with the arm actually opened.
	 *
	 * WHY THE BODY MAY BE BRACKETED AT ALL.  urcu_txn_begin() enters the RCU
	 * read side, and urcu_txn_conflict() ages into the domain's FIFO fallback
	 * lane -- where ft_writer_lock_gp_wait asserts !urcu_txn_in_fallback().
	 * Either one is fatal over a grace period, so the body must take none.
	 * Established by REACHABILITY CLOSURE over the whole translation unit, not
	 * by grep: seeding {ft_writer_lock_gp_wait, ft_move_gate_enter} (the only
	 * two functions in the FT that reach update_synchronize_rcu, and they
	 * reach it INDIRECTLY through the flavor struct, which no call graph sees)
	 * and taking the fixpoint gives 19 GP-reaching functions FT-wide; this
	 * function reaches 432, and the intersection is EMPTY.  The same tool
	 * reproduces ft_graft_keylen's known answer -- its own direct
	 * ft_writer_lock_gp_wait -- which is what makes the empty set here
	 * credible rather than merely convenient.
	 *
	 * @dst_ft->lock_fine is the contract the class uses, and the source is
	 * always exclusive at this label.  The coarse arm is left unbracketed on
	 * purpose: there ft_merge_at_inner holds the FT-wide writer lock across
	 * this whole loop, so no peer can make its commit abort and there is
	 * nothing for an escalation turn to win.
	 */
	const bool rm_bracket = dst_ft->lock_fine;
	struct urcu_txn optxn;
	bool rm_open = false;

	if (rm_bracket)
		ft_txn_op_init(dst_ft, &optxn);
retry_merge:
	if (rm_bracket) {
		urcu_txn_begin(&optxn);
		rm_open = true;
	}
	RSPIN_ENTER_X(0, rm_depth, 0, rm_bracket);
	RSPIN_SITE_ENTER(0, rm_depth, rm_bracket);
	ft_glue_init(&glue);
	glue.op = &optxn;
	/*
	 * Fence the compressed divergence node (like cds_ft_graft), so a
	 * concurrent grow of it is arbitrated and the build re-descends on a
	 * miss (FT_GRAFT_PREP_RETRY); ft_glue_init reset it, so set each attempt.
	 */
	glue.fence_split_cn = true;
	memset(&reserve, 0, sizeof(reserve));
	/*
	 * Every attach shape -- GLUE diverge and the NOSPLIT in-place store --
	 * commits through @glue.txn (exactly like cds_ft_graft): the build (below)
	 * tags its displaced old child dst_origin, and the NOSPLIT store reserves
	 * its slot proxy + run-splice edges, all out of this txn.  Create it BEFORE
	 * the build; destroyed on a POPULATED point (nothing to commit).
	 */
	glue.txn = ft_flip_txn_create(dst_ft);
	if (!glue.txn || !ft_flip_txn_reserve(glue.txn,
			/* +1: fused recompact-relocate tombstone (§4.B);
			 * + FLOOR_FREE: fused free-list tombstones;
			 * + count walk: the +cnt_src nr_keys ancestor edges (BULK fold) */
			FT_GLUE_FLOOR_DEFERRED + 7 + 1 /* +1 §4.B parent guard */ + FT_GLUE_FLOOR_FREE
				+ (dst_ft->rank_stats ? (int) dst_key_len + 1 : 0))) {
		if (glue.txn)
			ft_flip_txn_destroy(glue.txn);
		ft_glue_fini(&glue);
		if (already_unlinked) {
			ft_txn_attempt_bail(&optxn, rm_open);
			rm_open = false;
			goto retry_merge;	/* src consumed: OOM is transient */
		}
		ft_txn_attempt_end(&optxn, rm_open);
		rm_open = false;
		return CDS_FT_STATUS_MEMORY_ERROR;
	}
	glue.fuse_free_list = true;	/* reserved free-list headroom above (§4.B) */
	prep = ft_graft_build(dst_ft, okey_dst, dst_key_len, payload, cnt_src,
			&d, &glue, /*outer*/ NULL);
	*handled = true;
	if (prep == FT_GRAFT_PREP_RETRY) {
		/*
		 * Compressed-divergence @cn fence miss: a peer owns @cn.  Nothing
		 * built; a clean re-descend (pre-unlink both tries pristine, post-
		 * unlink src is empty + @payload owned).
		 */
		ft_glue_abort(dst_ft, &glue);
		ft_flip_txn_destroy(glue.txn);
		ft_txn_attempt_bail(&optxn, rm_open);
		rm_open = false;
		goto retry_merge;
	}
	if (prep == FT_GRAFT_PREP_OOM) {
		ft_glue_abort(dst_ft, &glue);
		ft_flip_txn_destroy(glue.txn);
		if (already_unlinked) {
			ft_txn_attempt_bail(&optxn, rm_open);
			rm_open = false;
			goto retry_merge;	/* src consumed: OOM is transient */
		}
		ft_txn_attempt_end(&optxn, rm_open);
		rm_open = false;
		return CDS_FT_STATUS_MEMORY_ERROR;	/* both tries pristine */
	}
	if (prep == FT_GRAFT_PREP_POPULATED) {
		/*
		 * Defensive: cnt_dst == 0 should never yield an occupied point.
		 * Impossible post-unlink (the payload is already excised + owned;
		 * no clean rc remains) -- assert rather than orphan it.
		 */
		assert(!already_unlinked);
		ft_flip_txn_destroy(glue.txn);
		ft_glue_fini(&glue);
		ft_txn_attempt_end(&optxn, rm_open);
		rm_open = false;
		return CDS_FT_STATUS_POPULATED_ERROR;
	}
	if (prep == FT_GRAFT_PREP_NOSPLIT) {
		/*
		 * Pre-fill the store's node reserve while both tries are still
		 * pristine.  The store's slot proxy + run-splice edges draw from the
		 * already-reserved @glue.txn, so no separate flip batch is needed.
		 * Securing the reserve here -- before the source unlink -- makes the
		 * post-drain store wholly failure-free, so the unlink is the true last
		 * fallible step and nothing strands.
		 *
		 * Use the GENEROUS superset fill (as ft_graft_keylen's NOSPLIT attach
		 * does for the same ft_store_at_graft_point), not a hand-rolled exact
		 * manifest: predicting the store's node set is fragile (e.g. attaching
		 * a high byte forces a RANGE recompact of the attach node even within
		 * its child capacity -- a grow an exact count heuristic misses, which
		 * underflowed the reserve and aborted).  A bulk op already pays a grace
		 * period, so the throwaway pops/pushes are negligible.
		 */
		if (ft_bulk_node_reserve_fill(dst_ft, &reserve)) {
			cds_ft_alloc_reserve_drain(dst_ft, &reserve);
			ft_glue_abort(dst_ft, &glue);
			ft_flip_txn_destroy(glue.txn);
			if (already_unlinked) {
				ft_txn_attempt_bail(&optxn, rm_open);
				rm_open = false;
				goto retry_merge;	/* src consumed: transient */
			}
			ft_txn_attempt_end(&optxn, rm_open);
			rm_open = false;
			return CDS_FT_STATUS_MEMORY_ERROR;
		}
	}
	/* GLUE: cluster built invisibly.  NOSPLIT: reserve secured (+ glue.txn). */

	/*
	 * Graft-parity (cds_ft_graft hoists the same test pre-swap): an exact-
	 * depth OCCUPIED NOSPLIT point is a PERMANENT POPULATED condition.  Catch
	 * it HERE, on the FIRST pass BEFORE the irreversible source unlink, so a
	 * misusing (non-disjoint) caller gets a clean POPULATED_ERROR with BOTH
	 * tries pristine -- rather than reaching the post-unlink store, asserting,
	 * and stranding @payload.  For the supported absent/diverged dst point
	 * (d.nf == NULL at key_len -- the only shape this helper is dispatched for)
	 * this is dead code.  A retry (post-unlink) cannot newly occupy the point
	 * under the disjoint exclusive-src contract, so the store's own POPULATED
	 * discrimination (below) stays a dead assert there too.
	 */
	if (!already_unlinked && prep == FT_GRAFT_PREP_NOSPLIT
			&& d.depth == dst_key_len && d.nf) {
		cds_ft_alloc_reserve_drain(dst_ft, &reserve);
		ft_glue_abort(dst_ft, &glue);
		ft_flip_txn_destroy(glue.txn);
		ft_txn_attempt_end(&optxn, rm_open);
		rm_open = false;
		return CDS_FT_STATUS_POPULATED_ERROR;
	}

	/*
	 * Ordered list: locate the dst splice neighbours (RE-FOUND each attempt --
	 * dst may have grown across a retry) and, on the FIRST attempt only,
	 * capture the source subtree's run endpoints + cells while the subtree is
	 * still intact in src.  @run_first / @run_last / @s_first / @s_last persist
	 * across retries: the run is excised ONCE by the unlink below and then
	 * owned (out of both lists), so a retry re-splices the same owned run.
	 */
	if (ms_ord) {
		ft_ord_cell_find_splice_pos(dst_ft, dst_key, dst_key_len,
			&pred, &succ, NULL);
		if (!already_unlinked) {
			s_first = ft_subtree_minmax_head(src_ft, payload, false);
			s_last = ft_subtree_minmax_head(src_ft, payload, true);
			run_first = ft_ord_cell_ptr(rcu_dereference(s_first->prev));
			run_last = ft_ord_cell_ptr(rcu_dereference(s_last->prev));
			/* Src side: EXCISE-ONLY run so the structural unlink and the
			 * run's removal from src's ordered list commit in ONE flip
			 * (the disappear-side cross-view fix). */
			srun.rfirst = s_first;
			srun.rlast = s_last;
			srunp = &srun;
		}
		mrun.run_first = run_first;
		mrun.run_last = run_last;
		mrun.pred = pred;
		mrun.succ = succ;
		mrun.armed = false;
		run_arg = &mrun;
	}

	/*
	 * Source-side excise (ONE-SHOT, @already_unlinked): reserve the run txns,
	 * unlink the source subtree in place (preserving @payload), remove the run
	 * from src's list, drain src readers, and stamp an external payload's edge
	 * byte.  A retry after a post-unlink commit abort skips all of this -- src
	 * is already empty and @payload is owned -- and re-runs only the dst attach.
	 */
	if (!already_unlinked) {
		if (ms_ord) {
			/*
			 * Pre-reserve the standalone run-unlink + run-splice txns (the
			 * rare unfused shapes touch src's / dst's list AFTER the
			 * structural flip is public, un-abortable).  OOM here aborts the
			 * still-invisible build (both tries pristine).
			 */
			run_unlink_txn = ft_flip_txn_create_bounded(src_ft,
				FT_ORD_CELL_RUN_UNLINK_MAX_EDGES);
			run_splice_txn = ft_flip_txn_create_bounded(dst_ft,
				FT_ORD_CELL_RUN_SPLICE_MAX_EDGES);
			if (!run_unlink_txn || !run_splice_txn) {
				if (run_unlink_txn)
					ft_flip_txn_destroy(run_unlink_txn);
				if (run_splice_txn)
					ft_flip_txn_destroy(run_splice_txn);
				run_unlink_txn = run_splice_txn = NULL;
				cds_ft_alloc_reserve_drain(dst_ft, &reserve);
				ft_glue_abort(dst_ft, &glue);
				ft_flip_txn_destroy(glue.txn);
				ft_txn_attempt_end(&optxn, rm_open);
				rm_open = false;
				return CDS_FT_STATUS_MEMORY_ERROR;
			}
		}

		/*
		 * Last fallible SRC step: unlink the source subtree in place,
		 * preserving @payload.  On OOM the unlink self-undoes (src pristine)
		 * and the still-invisible cluster / reserve is released -- no rollback.
		 */
		if (ft_merge_unlink_src_subtree(src_ft, okey_src, src_key_len,
				cnt_src, srunp, /*retire_glue=*/ NULL) < 0) {
			if (run_unlink_txn)
				ft_flip_txn_destroy(run_unlink_txn);
			if (run_splice_txn)
				ft_flip_txn_destroy(run_splice_txn);
			run_unlink_txn = run_splice_txn = NULL;
			cds_ft_alloc_reserve_drain(dst_ft, &reserve);
			ft_glue_abort(dst_ft, &glue);
			ft_flip_txn_destroy(glue.txn);
			ft_txn_attempt_end(&optxn, rm_open);
			rm_open = false;
			return CDS_FT_STATUS_MEMORY_ERROR;
		}
		already_unlinked = true;	/* POINT OF NO RETURN: @payload owned */

		/*
		 * Remove the source run from src's ordered list (cells keep their
		 * internal links for the dst splice) -- BEFORE the drain, so sync
		 * drains src ord-readers of the run too.  The structural unlink FUSED
		 * this into its flip (@srun.armed); the standalone remains a fallback.
		 */
		if (ms_ord && !srun.armed) {
			ft_ord_cell_run_unlink(src_ft, run_unlink_txn, s_first,
				s_last);
			run_unlink_txn = NULL;	/* consumed */
		}
		if (run_unlink_txn) {
			ft_flip_txn_destroy(run_unlink_txn);
			run_unlink_txn = NULL;
		}

		/* No src drain: cross-trie only, src is exclusive.  See
		 * ft_merge_spine_copy's assert for the full reasoning. */
		assert(src_ft->exclusive);

		/*
		 * An EXTERNAL payload's edge byte changes (src_key's last byte ->
		 * dst_key's); stamp it in the head's CELL metadata now -- the cell is
		 * in NEITHER list and structurally invisible, so no reader rebuilds a
		 * key from it (stamping while still in src's list would let a src
		 * reader rematerialize an out-of-namespace key).  Internal/compressed
		 * payloads keep every leaf's edge byte (the subtree moves wholesale).
		 */
		if (ms_ord && ft_node_external(payload))
			cds_ft_item_to_metadata(run_first)->incoming_byte =
				okey_dst[dst_key_len - 1];
	}

	if (prep == FT_GRAFT_PREP_GLUE) {
		/*
		 * Failure-free commit of the build-invisible diverge cluster: the
		 * displaced old child is LIVE (still reachable through the compressed
		 * node being split until the forward publish), so via @glue.txn it
		 * flips atomically with the forward publish and the ordered-list
		 * run-splice -- the whole attach observed old XOR new.  The payload's
		 * hidden back-pointers are wired immediately inside the commit.  Then
		 * reclaim the replaced compressed node.
		 */
		/*
		 * Order-statistics fold (BULK): the diverge cluster raises
		 * @glue.publish_parent's subtree by +cnt_src; record that walk
		 * into the same commit (a no-op when rank stats off).
		 */
		enum urcu_txn_status cst;

		glue.count_delta = (long) cnt_src;
		cst = ft_glue_txn_commit(dst_ft, &glue, run_arg);
		if (cst != URCU_TXN_STATUS_OK) {
			/*
			 * MW LOCK_FINE drop: the GLUE commit's MCAS footprint (the
			 * forward publish + fused run-splice + fenced retires) can
			 * conflict with a peer and ABORT even though it allocates
			 * nothing.  The flip rolled back (dst byte-for-byte unchanged,
			 * the run-splice not applied); ft_glue_txn_commit consumed
			 * glue.txn.  Free the failed attempt's invisible cluster and
			 * re-attach the owned @payload via the retry.  @run_splice_txn
			 * / @run_first / @run_last persist (src already excised).
			 */
			ft_glue_abort(dst_ft, &glue);
			ft_txn_attempt_bail(&optxn, rm_open);
			rm_open = false;
			goto retry_merge;
		}
		attached_nf = glue.attached_nf;
		ft_glue_free_old(dst_ft, &glue);
		ft_glue_fini(&glue);
	} else {
		/*
		 * NOSPLIT: graft the in-place payload at @d.  Node allocations draw
		 * from the reserve and the slot proxy + run-splice edges from the
		 * pre-reserved @glue.txn.  Under the drop the store's recompact of
		 * the (contended) spine can still FAIL its prepare -- ft_store_at_
		 * graft_point returns -EAGAIN / MEMORY_ERROR (not the FT-wide-lock
		 * "cannot fail") -- after the source is already excised; re-attach
		 * the owned @payload via the retry.
		 */
		unsigned int adepth = 0;
		enum cds_ft_status st;

		cds_ft_alloc_reserve_activate(dst_ft, &reserve);
		st = ft_store_at_graft_point(dst_ft, okey_dst, dst_key_len, &d,
				payload, cnt_src, &attached_nf, &adepth, &glue,
				run_arg,
				(long) cnt_src);
		cds_ft_alloc_reserve_deactivate(dst_ft);
		cds_ft_alloc_reserve_drain(dst_ft, &reserve);
		if (st != CDS_FT_STATUS_OK) {
			/*
			 * ft_store_at_graft_point already freed its invisible build +
			 * txn (and, on a commit abort, the relocated copy) and cleaned
			 * up @glue.  Discriminate the failure class:
			 *
			 * - POPULATED is a PERMANENT occupied-point condition, NOT a
			 *   transient conflict -- retrying it would LIVELOCK and strand
			 *   the owned @payload.  Post-unlink it is impossible for the
			 *   supported disjoint exclusive-src contract (no peer creates
			 *   the moved key), so assert as the pre-unlink PREP_POPULATED
			 *   arm does.  A general (non-disjoint) caller -- a peer racing
			 *   the moved key into the target during the owned-but-unattached
			 *   window -- would still orphan @payload here: this retry-based
			 *   recovery excises the source BEFORE the dst attach, so the
			 *   window exists.  cds_ft_graft no longer has it: its exclusive
			 *   src-swap-fused arm lands the src retire ATOMICALLY with the
			 *   attach (only on commit success), so the same race MCAS-aborts,
			 *   retries, and re-detects POPULATED pre-commit with the source
			 *   never swapped.  Closing this gap for the sub-position move --
			 *   fusing ft_merge_unlink_src_subtree into the attach flip -- is a
			 *   deferred parity follow-up (not needed for any disjoint use).
			 * - Everything else is transient -- re-descend and re-attach
			 *   the owned @payload: BUSY_ERROR is contention (a peer
			 *   filled the reserve's byte, a recompact could not lock, or
			 *   the store's MCAS commit ABORTed), MEMORY_ERROR is a real
			 *   allocation failure.  Both retry; they are kept distinct so
			 *   a conflict never has to be read as an OOM.
			 */
			if (st == CDS_FT_STATUS_POPULATED_ERROR) {
				assert(!already_unlinked);
				ft_txn_attempt_end(&optxn, rm_open);
				rm_open = false;
				return CDS_FT_STATUS_POPULATED_ERROR;
			}
			ft_txn_attempt_bail(&optxn, rm_open);
		rm_open = false;
		goto retry_merge;
		}
		ft_glue_fini(&glue);
	}

	/*
	 * Order-statistics: the +cnt_src ancestor walk is now FOLDED onto the
	 * attach commit above (glue.count_delta for the GLUE diverge and
	 * displaced-external shapes; ft_store_at_graft_point's count_delta for
	 * the in-place / recompact-relocate slot shapes), so it flips ATOMICALLY
	 * with the structural publish -- exact under concurrent writers.
	 */

	/*
	 * Ordered list: every attach shape (GLUE, displaced-external, in-place
	 * slot) FUSED the run-splice into its structural flip (@armed) -- the
	 * external edge-byte was stamped before that splice, above.  The standalone
	 * two-commit splice remains as a defensive fallback for any not-yet-fused
	 * shape (none today); without it an unfused shape would strand the moved
	 * run out of the ordered list.  It commits through the pre-reserved
	 * @run_splice_txn (un-abortable post-drain); the fused common case frees
	 * that txn unused.
	 */
	if (ms_ord && !mrun.armed) {
		ft_ord_cell_run_splice(dst_ft, run_splice_txn, run_first,
			run_last, pred, succ);
		run_splice_txn = NULL;	/* consumed */
	}
	if (run_splice_txn)
		ft_flip_txn_destroy(run_splice_txn);	/* fused: unused */

	/* Raise dst's max_used_key_len for the moved keys (dst_key || suffix). */
	src_max = uatomic_load(&src_ft->max_used_key_len, CMM_RELAXED);
	nm = src_max > src_key_len ? dst_key_len + (src_max - src_key_len) :
		dst_key_len;
	dm = uatomic_load(&dst_ft->max_used_key_len, CMM_RELAXED);
	if (nm > dm)
		uatomic_store(&dst_ft->max_used_key_len, nm, CMM_RELAXED);

	ft_txn_attempt_end(&optxn, rm_open);
	rm_open = false;
	return CDS_FT_STATUS_OK;
}

/*
 * @pre_txn carries a flip-txn the caller reserved before its own last fallible
 * step, so the spine-copy / graft commit below draws an unfailable txn instead
 * of allocating one.  NULL on the public paths (cds_ft_merge_at and the outer
 * cds_ft_rekey_* call), set only by the same-trie rekey's post-detach re-merge,
 * which pre-reserves it (sized from the O(1) subtree key counts:
 * the structural re-parent + folded ordered-list interleave, or the graft
 * cluster floor) so its post-detach merge cannot fail -- no reader-observable
 * rollback.  The consume site NULLs the slot it takes, so the rekey frees the
 * txn only when a given merge shape left it unused.
 */
static enum cds_ft_status ft_merge_at_inner(struct cds_ft *dst_ft,
		const uint8_t *dst_key, size_t dst_key_len,
		struct cds_ft *src_ft,
		const uint8_t *src_key, size_t src_key_len,
		struct ft_flip_txn **pre_txn)
{
	struct cds_ft *subtree = NULL;
	enum cds_ft_status status;
	struct ft_descent d_src, d_dst;
	unsigned int off_src, off_dst;
	unsigned long cnt_src, cnt_dst;
	enum ft_graft_swap_case ks, kd;
	uint8_t okey_dst_buf[FT_MAX_KEY_LEN], okey_src_buf[FT_MAX_KEY_LEN];
	const uint8_t *okey_dst = dst_key, *okey_src = src_key;

	FT_TP(merge_enter, (const void *) dst_ft, (const void *) src_ft);

	if (!dst_ft || !src_ft) {
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
	/*
	 * merge_at is CROSS-TRIE only.  A same-trie move (src_ft == dst_ft) is a
	 * REKEY -- expressed by cds_ft_rekey_graft / cds_ft_rekey_merge, which reach
	 * this worker with @rekey set.  A same-trie cds_ft_merge_at (@rekey ==
	 * FT_REKEY_NONE) is rejected; use the dedicated rekey entry points instead.
	 */
	if (src_ft == dst_ft) {
		FT_TP(merge_exit, (int) CDS_FT_STATUS_INVALID_ARGUMENT_ERROR);
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}
	/*
	 * A rekey on a SPECULATIVE (leaf-stored-key) trie is refused: the move
	 * re-parents each leaf under @dst_key WITHOUT rewriting its app-owned stored
	 * key -- the library never writes that field -- so every moved leaf would
	 * carry a key that no longer matches its structural position, and a
	 * speculative lookup would then return the wrong key
	 * (ft_verify_speculative_key catches this only under
	 * FEATURE_FT_VERIFY_AT_MUTATION).  Only EAGER tries -- which reconstruct the
	 * key from structure and never read the stored field -- may rekey.
	 */
	/*
	 * The STAGED rekey needs a VARIABLE-length group, and the refusal belongs
	 * HERE, before anything is read or reserved.  The move is staged as a
	 * detach of @src_key's subtree into a transient trie followed by a merge of
	 * that trie back in at @dst_key, and a detached subtree carries keys
	 * STRIPPED of the prefix -- shorter than a fixed-length group's one key
	 * length, which is exactly why cds_ft_detach and cds_ft_graft take a
	 * non-root key on variable-length groups only.  The staged rekey is
	 * composed of those two operations, so it inherits the restriction.
	 *
	 * A fixed-length group is served by the ATOMIC writer instead
	 * (ft_rekey_one_decide, dispatched before this worker), which stages
	 * through no transient trie and so has no such restriction -- it is the
	 * only rekey a fixed-length group gets, and reaching here means its cut did
	 * not cover the shape.
	 *
	 * Enforcing it at the entry is what makes the refusal SAFE.  The placement
	 * is a merge of that transient at @dst_key, so it meets the fixed-length
	 * equal-prefix-length guard above with src_key_len == 0 != dst_key_len and
	 * refuses -- but only AFTER the detach has committed, leaving the caller an
	 * INVALID_ARGUMENT_ERROR (an "argument rejected, nothing happened" status)
	 * for a trie that has just lost every moved key to the destroyed transient.
	 */
	/*
	 * Combined-length overflow validation (mirrors cds_ft_graft): a moved
	 * key K becomes dst_key || (K - src_key prefix), of length
	 * dst_key_len + len(K) - src_key_len.  Bound len(K) by the source's
	 * max_used_key_len; without this check a variable-length merge with
	 * dst_key_len > src_key_len could create keys exceeding the group's
	 * max_key_len, overflowing the fixed-size key buffers downstream
	 * (the spine's compressed-wrap kbuf, the iterator buffers).
	 */
	{
		size_t src_max = uatomic_load(&src_ft->max_used_key_len,
				CMM_RELAXED);

		if (src_max > src_key_len &&
				src_max - src_key_len >
				dst_ft->group->max_key_len - dst_key_len) {
			FT_TP(merge_exit, (int) CDS_FT_STATUS_OVERFLOW_ERROR);
			return CDS_FT_STATUS_OVERFLOW_ERROR;
		}
	}

	/*
	 * MW LOCK_FINE (step 6, §9.5): a CROSS-trie merge consumes @src_ft, so it
	 * must be EXCLUSIVE -- an exclusive source skips its FT-wide lock, leaving
	 * only dst's lock (one lock, no cross-trie deadlock).  A live (lock-mode,
	 * non-exclusive) cross-trie source is REJECTED with BUSY before any lock is
	 * taken; the caller makes it exclusive first (cds_ft_make_exclusive).  A
	 * SAME-trie rekey (src == dst) is excluded -- it takes one lock reentrantly.
	 * Inert outside lock-mode.
	 */
	/*
	 * THE GATE that makes src exclusivity an invariant for everything below
	 * (and for ft_merge_spine_copy / ft_merge_graft_subpos_inplace, which
	 * only this function calls).  The former `src_ft != dst_ft &&` term is
	 * gone: same-trie is rejected above, so it was constant-true here.
	 */
	if (!src_ft->exclusive) {
		FT_TP(merge_exit, (int) CDS_FT_STATUS_BUSY_ERROR);
		return CDS_FT_STATUS_BUSY_ERROR;
	}

	/*
	 * merge_at is a mutator; the application provides mutual exclusion
	 * between mutators.  Take the reentrant writer-validation scope (like
	 * cds_ft_graft_swap), NOT a blanket flavor read lock.
	 *
	 * The reason is the REKEY entries, not merge_at.  cds_ft_merge_at
	 * consumes an EXCLUSIVE source, so every ft_writer_lock_gp_wait on its
	 * path -- both here and both in ft_graft_keylen -- is
	 * !src_ft->exclusive-gated and never runs; that path syncs nowhere, and
	 * the spine copy pins it under a read lock for exactly that reason.
	 * cds_ft_rekey_{graft,merge} share this body with src_ft == dst_ft, a
	 * LIVE trie: there those waits DO run, and a read lock held across one
	 * would be a writer waiting on its own grace period.  One body, two
	 * source contracts -- so the pin is taken per path, not here.
	 */
	ft_crosstrie_lock_mode_guard(dst_ft, src_ft);
	CDS_FT_SCOPED_WRITER(dst_ft);
	CDS_FT_SCOPED_WRITER(src_ft);

	/*
	 * Convert both keys to ORDINAL form ONCE; every internal consumer
	 * (the merge-point descents, the spine build, the source-subtree
	 * unlink, the ordered-list interleave) takes the ordinal form.  The
	 * detach+graft fallback below instead receives the ORIGINAL
	 * application keys -- those entry points remap internally.
	 * Previously the descents consumed the application bytes raw while
	 * the unlink remapped: on a non-identity key map the build copied one
	 * subtree and the unlink targeted another.
	 */
	{
		const struct cds_ft_key_map *km = &dst_ft->group->key_map;

		if (caa_unlikely(!km->identity)) {
			ft_key_to_ordinals(okey_dst_buf, dst_key, dst_key_len,
				km);
			ft_key_to_ordinals(okey_src_buf, src_key, src_key_len,
				km);
			okey_dst = okey_dst_buf;
			okey_src = okey_src_buf;
		}
	}

	/*
	 * Same-trie "rekey" (src_ft == dst_ft): moving @src_key's subtree to
	 * @dst_key within one trie is allowed, but the two keys must be DISJOINT
	 * -- neither a prefix of the other.  A prefix relationship means one key
	 * lies inside the other's subtree, so the move would be circular (and
	 * @dst_key would not be absent), and equal keys are a degenerate self-
	 * move.  The check is byte-position-wise, identical in application and
	 * ordinal space (the key map is a per-position bijection).  Cross-trie
	 * subtrees are disjoint by construction, so the guard is same-trie only.
	 */

	/*
	 * Locate both merge points read-only (a writer descends its own
	 * stable state).  No content under @src_key -> the merge is a no-op;
	 * cnt_src == 0 covers an empty @src_ft root (src_key_len == 0, where
	 * the descent still reports EXACT at the always-present root).
	 */
	ks = ft_merge_descend(src_ft, okey_src, src_key_len, &d_src,
			&off_src, &cnt_src);
	MRG_SKIPCONF_PROBE(0, d_src);
	if (ks == FT_GRAFT_SWAP_DELEGATE || cnt_src == 0) {
		FT_TP(merge_exit, (int) CDS_FT_STATUS_OK);
		return CDS_FT_STATUS_OK;
	}

	/*
	 * Same-trie "rekey": detach @src_key's subtree into a transient trie,
	 * then merge that whole trie back in at @dst_key.  The detach fully
	 * commits -- recompacting the shared common-prefix ancestor and draining
	 * -- BEFORE the placement reads it, so the move's two sides no longer
	 * alias the same node.  The in-place reorder CANNOT be used here: its
	 * source unlink would recompact that shared ancestor out from under the
	 * build-invisible destination placement (a stale-slot publish).  The
	 * cross-trie merge handles an occupied @dst_key by MERGING and an absent
	 * one by grafting; it is made UNFAILABLE (its nodes + flip batches are
	 * pre-reserved before the detach below), so it always commits the move --
	 * there is no failure path that would restore the content to @src_key, i.e.
	 * no reader-observable rollback.
	 *
	 * A KEY_SHORTER source (off_src > 0, @src_key ends inside the compressed
	 * node @d_src.nf) is reduced to the EXACT node boundary first, exactly as
	 * the cross-trie path does: detach @cn_s->child via the full-compressed
	 * key (src prefix ++ all of cn_s), and merge at @dst_key extended by the
	 * residual cn_s bytes -- ft_detach_keylen overshoots a compressed node, so
	 * it must be handed a boundary key, not an interior one.
	 */
	/*
	 * The STAGED rekey lived here: a detach of @src_key's subtree into a
	 * transient trie followed by a merge of that trie back in at @dst_key,
	 * plus the argument checks that only it needed.  It moved to
	 * ft_rekey_at_inner (ft-rekey.h) with the rest of the same-trie writer,
	 * so this worker is now CROSS-TRIE ONLY and no longer recursive -- the
	 * recursive ft_merge_at_inner() call was inside that block.
	 */

	bool md_rlock = false;
	bool md_contended;
	unsigned long md_spin = 0;	/* this op's merge_spine_retry depth (probe) */
	/*
	 * The op's PERSISTENT engine handle, spanning the whole merge_spine_retry
	 * loop the way ft_txn_op_init does for insert, remove and replace (doc
	 * §11).  Without it this loop ages nothing: every attempt started at
	 * retry 0, so the domain never escalated the writer into the per-trie
	 * FIFO fair-mutex lane and the loop had no termination argument at all --
	 * measured at 585250 declined lock sets over 235141 merges under an
	 * exponential spacing, one op re-descending 348 times.
	 *
	 * It is bound and bracketed ONLY on the arm that takes the read pin
	 * below, and that is not an optimization: urcu_txn_begin() enters the
	 * RCU read side, and the SAME-TRIE rekey shares this body with a LIVE
	 * src whose ft_writer_lock_gp_wait calls do run -- a read section held
	 * across one is a writer waiting on its own grace period.  The condition
	 * `dst_ft->lock_fine && src_ft->exclusive` is exactly the source
	 * contract that syncs nowhere, and it is where every one of those
	 * 585250 declines was measured (livesrc=0).
	 */
	struct urcu_txn optxn;

	MRG_SPIN_PROBE(0);
	ft_txn_op_init(dst_ft, &optxn);

	/*
	 * Re-entered when a spine-copy attempt could not take its dup-chain lock
	 * set (@md_contended).  The attempt moved nothing -- an EXCLUSIVE source
	 * means the merge is build-invisible until its one commit, so a declined
	 * attempt is trivially undone and both tries are byte-for-byte as they
	 * were.  Re-descend, because the whole reason the acquire declined is that
	 * a peer is reshaping the dst spine we planned against, and rebuild.  Same
	 * plan->commit retry shape as cds_ft_graft's retry_attach and
	 * cds_ft_graft_swap's retry_swap.
	 */
merge_spine_retry:
	/*
	 * §11 cross-trie RCU-pinning: the spine-copy path below descends the live
	 * dst here and node locks a descent-captured dst node (@d_dst->pnf /
	 * ->ppnf) inside ft_merge_spine_copy.  A node lock false-succeeds on a
	 * reclaimed+recycled node (arena re-zeroes metadata), so pin the captured
	 * nodes with the flavor read side across descent -> lock, as cds_ft_graft
	 * does.  The dup-chain holders the splice lock set acquires are captured
	 * during that same window, so this pin covers them too.  With an EXCLUSIVE
	 * src the spine-copy's only grace period (its src drain) is
	 * !src_ft->exclusive-gated and skipped, so the whole ft_merge_spine_copy
	 * runs GP-free under the read lock.  Released before the spine-copy return,
	 * on the fall-through to the detach/graft paths, and before each retry
	 * above (a retry re-descends, so it must re-pin what it re-reads).
	 */
	if (dst_ft->lock_fine) {		/* src exclusive: gated above */
		/*
		 * Open the attempt on the persistent handle FIRST: a retry that
		 * has aged escalates here, and escalation blocks on the domain's
		 * fair mutex, which must not happen holding the pin below.  The
		 * explicit read_lock stays -- it is what pins the descent-captured
		 * dst nodes, and it must not become conditional on the handle
		 * having a flavour bound (ft_txn_op_init binds NULL on an
		 * exclusive dst).  RCU read sections nest; @md_rlock now marks
		 * both, and every site that releases it closes the txn too.
		 */
		urcu_txn_begin(&optxn);
		dst_ft->group->flavor->read_lock();
		md_rlock = true;
	}
	kd = ft_merge_descend(dst_ft, okey_dst, dst_key_len, &d_dst,
			&off_dst, &cnt_dst);
	MRG_SKIPCONF_PROBE(2, d_dst);
	/*
	 * Reanchor level-move: a peer chain-merge absorbed this slot's level
	 * into a longer compressed node while the descent walked it, so @d_dst
	 * names a position that has moved and every count and offset derived
	 * from it describes the old shape.  Insert and graft already bail on
	 * this; merge read it and continued.
	 *
	 * Re-descend down the SAME unwind the spine copy's contention path
	 * uses: nothing is built here (the label is above the read-lock pin,
	 * and the detach branch returned long before), so this is the state
	 * that path already returns to.  Release the pin first -- a retry
	 * re-reads, so it must re-pin.
	 */
	if (caa_unlikely(d_dst.skip_conflict)) {
		if (md_rlock) {
			dst_ft->group->flavor->read_unlock();
			/* Nothing moved: age the conflict, close the attempt. */
			ft_txn_attempt_bail(&optxn, true);
			md_rlock = false;
		}
		md_spin++;
		MRG_SPIN_PROBE(2);
		MRG_SPIN_MAX(md_spin);
		goto merge_spine_retry;
	}

	/*
	 * Atomic build-invisible spine-copy of @src_ft's subtree at @src_key into
	 * @dst_ft's non-empty subtree at @dst_key.  Each side is EXACT (@off == 0,
	 * the subtree at @d->nf) or KEY_SHORTER (@off > 0, the key ends inside a
	 * compressed node -- the src node is reclaimed by the commit's unlink, the
	 * dst node is wrapped under its prefix).  ft_merge_spine_copy handles every
	 * such shape (internal, external, compressed and compressed-parent dst
	 * merge points); only a dst that is empty under @dst_key falls through to
	 * the detach-based graft below.
	 */
	if ((ks == FT_GRAFT_SWAP_EXACT || ks == FT_GRAFT_SWAP_KEY_SHORTER)
			&& cnt_dst > 0
			&& (kd == FT_GRAFT_SWAP_EXACT
				|| kd == FT_GRAFT_SWAP_KEY_SHORTER)) {
		md_contended = false;
		status = ft_merge_spine_copy(dst_ft, src_ft, &d_src,
				okey_src, src_key_len, cnt_src, off_src,
				&d_dst, cnt_dst, off_dst, dst_key_len,
				pre_txn, &md_contended, &optxn);
		if (md_contended) {
			/* Contention, nothing moved: re-pin, re-descend, rebuild. */
			if (md_rlock) {
				dst_ft->group->flavor->read_unlock();
				/*
				 * AGE IT.  This is the arm that spun 348 deep with
				 * nothing to make it terminate: urcu_txn_conflict
				 * carries the retry count on the persistent handle, so
				 * the domain escalates this writer into the FIFO lane
				 * and the contention drains.  end() then FORFEITS the
				 * turn -- this is a pre-commit bail, and a bail that
				 * keeps its turn while the peer it waits on queues
				 * behind that same turn is the insert livelock.
				 */
				ft_txn_attempt_bail(&optxn, true);
				md_rlock = false;
			}
			md_spin++;
			MRG_SPIN_PROBE(1);
			MRG_SPIN_PROBE(4);	/* src always exclusive here */
			MRG_SPIN_MAX(md_spin);
			goto merge_spine_retry;
		}
		if (status == CDS_FT_STATUS_OK) {
			/*
			 * Raise dst's max_used_key_len for the moved keys
			 * (dst_key_len + the longest stripped suffix), as the
			 * graft paths do -- later graft overflow validations
			 * feed off it.
			 */
			size_t src_max = uatomic_load(&src_ft->max_used_key_len,
					CMM_RELAXED);
			size_t nm = src_max > src_key_len ?
				dst_key_len + (src_max - src_key_len) :
				dst_key_len;
			size_t dm = uatomic_load(&dst_ft->max_used_key_len,
					CMM_RELAXED);

			if (nm > dm)
				uatomic_store(&dst_ft->max_used_key_len, nm,
					CMM_RELAXED);
		}
		if (md_rlock) {
			dst_ft->group->flavor->read_unlock();
			urcu_txn_end(&optxn);
			md_rlock = false;
		}
		FT_TP(merge_exit, (int) status);
		return status;
	}
	/*
	 * Fall-through: the spine-copy shape did not apply (whole-source move,
	 * or an empty dst under @dst_key -> the detach graft below).  Release the
	 * spine-copy descent pin here; the src_key_len==0 graft re-descends under
	 * its own bracket above, and the detach-graft path re-resolves its own
	 * (parent, slot).
	 */
	if (md_rlock) {
		dst_ft->group->flavor->read_unlock();
		urcu_txn_end(&optxn);
		md_rlock = false;
	}

	/*
	 * Whole-source move (src_key_len == 0): the entire @src_ft is the
	 * payload, so graft it directly -- no detach, no fallible rollback.
	 * ft_graft is itself leak-free: a diverge split is built invisibly and
	 * a NOSPLIT attach restores the saved old source root allocation-free
	 * on OOM, so an allocation failure leaves both tries pristine.  This
	 * covers an empty dst root and a diverged dst_key alike, and is the
	 * leak-free path for "rekey within a trie" (detach a sub-trie, then
	 * merge_at it at a new, currently-absent key in the same group).  A
	 * dst_key occupied by an empty-internal leftover yields POPULATED_ERROR
	 * here, exactly as the detach-then-graft path did.
	 */
	if (src_key_len == 0) {
		/*
		 * A live (non-exclusive) cross-trie source was already rejected with
		 * BUSY at merge_at's entry, so this graft sees an exclusive source;
		 * ft_graft_keylen runs its fused body directly.
		 */
		if (dst_ft->lock_fine) {		/* src exclusive: gated above */
			const struct rcu_flavor_struct *flavor =
				dst_ft->group->flavor;

			/*
			 * §11 cross-trie RCU-pinning: this is the SAME live-dst
			 * descent + node lock as cds_ft_graft, reached through
			 * cds_ft_merge / cds_ft_merge_at, so it needs the same read-
			 * side bracket that pins descent-captured dst spine nodes
			 * against a peer relocate+free+recycle (see the bracket in
			 * cds_ft_graft).  With an EXCLUSIVE src the fused body takes
			 * no grace period (ft_writer_lock_gp_wait is !exclusive-
			 * gated), so the whole-op read lock cannot self-deadlock.
			 *
			 * That is the SAME condition, and the same reason, as the
			 * spine-copy pin above -- both branches read_lock() under
			 * exactly `dst_ft->lock_fine && src_ft->exclusive`.  What
			 * decides it is the SOURCE CONTRACT, not the branch: a
			 * cross-trie src is exclusive or the op was already refused
			 * with BUSY, and every grace period on that path is
			 * !exclusive-gated.  Only the same-trie rekey (src == dst,
			 * a LIVE src) actually runs those waits, and it is exactly
			 * the case this condition excludes from the read section.
			 */
			flavor->read_lock();
			status = ft_graft_keylen(dst_ft, dst_key, dst_key_len,
					src_ft, pre_txn);
			flavor->read_unlock();
		} else
			status = ft_graft_keylen(dst_ft, dst_key, dst_key_len,
					src_ft, pre_txn);
		FT_TP(merge_exit, (int) status);
		return status;
	}

	/*
	 * Empty dst ROOT (dst_key_len == 0, so the whole @dst_ft is empty at
	 * the merge point): move @src_ft@src_key in WITHOUT a fallible graft,
	 * so no rollback can strand the moved externals.  Pre-allocate the
	 * source's replacement root (fallible while @src_ft is still pristine),
	 * detach the source subtree (clean on its own failure), then SWAP it
	 * into @dst_ft's root.  The swap is a failure-free root-to-root move:
	 * no parent-pointer change and no "jump out" window (both ends are
	 * roots with parent NULL), and the detach already drained @src_ft's
	 * readers -- the same reasoning as ft_graft's root path.  Because
	 * nothing fallible follows the detach, this path has no rollback, so it
	 * cannot leak (contrast the diverged path below).
	 */
	if (dst_key_len == 0 && cnt_dst == 0) {
		struct cds_ft_inode *fresh_root;
		struct cds_ft_metadata *fresh_meta;
		struct cds_ft_metadata *dst_rmeta;
		struct cds_ft_inode_flag *dst_root_fenced;
		uintptr_t dst_root_snap;
		struct cds_ft_inode *old_dst_root;
		struct ft_flip_txn *appear_txn = NULL;
		size_t sm;
		int fence_ret;

		/*
		 * @cnt_dst was sampled far above, and an ENTIRE ft_detach_keylen of
		 * the source runs before the swap below -- the widest empty-dst
		 * window in the FT.  Re-decide emptiness UNDER the old root's
		 * node lock, so a contract-legal peer attach can no longer land
		 * inside that window and be freed with the root it landed on.  On a
		 * peer-populated dst, fall through to the DIVERGED path below (which
		 * merges into a populated destination) exactly as an up-front
		 * cnt_dst != 0 would have; on a held root, report BUSY.
		 */
		fence_ret = ft_root_attach_fence_empty(dst_ft, &dst_root_fenced,
			&dst_rmeta, &dst_root_snap, &optxn);
		if (fence_ret == -EEXIST)
			goto diverged;
		if (fence_ret) {
			status = CDS_FT_STATUS_BUSY_ERROR;
			goto out;
		}

		fresh_root = alloc_cds_ft_node(src_ft, &ft_types[0], &fresh_meta);
		if (!fresh_root) {
			ft_meta_lock_release(dst_rmeta);
			status = CDS_FT_STATUS_MEMORY_ERROR;
			goto out;
		}
		fresh_meta->parent_word = ft_trie_parent(src_ft);
		ft_nr_keys_store(src_ft, fresh_meta, 0, CMM_RELAXED);

		/*
		 * Pre-reserve the dst-appear root-swap txn BEFORE the detach: the
		 * swap is failure-free (post-detach) so it commits through this
		 * pre-reserved txn (ft_ord_cell_flip_into).  OOM here aborts while
		 * @src_ft is still pristine (no detach yet).  The dst old-root
		 * freeze-on-free tombstone rides the SAME flip (atomic detach,
		 * §4.B): +1 edge list-on; list-off is a 2-edge txn (root edge +
		 * tombstone) instead of the former lone ft_root_edge_flip store.
		 */
		if (dst_ft->group->ordered_list_set)
			appear_txn = ft_flip_txn_create_bounded(dst_ft,
				FT_ROOT_LIST_SWAP_MAX_EDGES + 1);
		else
			appear_txn = ft_flip_txn_create_bounded(dst_ft, 2);
		if (!appear_txn) {
			ft_meta_lock_release(dst_rmeta);
			free_cds_ft_node_unpublished(src_ft, fresh_root);
			status = CDS_FT_STATUS_MEMORY_ERROR;
			goto out;
		}

		status = ft_detach_keylen(src_ft, src_key, src_key_len, &subtree);
		if (status < 0) {
			/* NOT_FOUND impossible: @src_ft had content. */
			ft_meta_lock_release(dst_rmeta);
			if (appear_txn)
				ft_flip_txn_destroy(appear_txn);
			free_cds_ft_node_unpublished(src_ft, fresh_root);
			goto out;
		}

		/*
		 * Failure-free swap.  @subtree->root carries the moved content
		 * with parent already NULL (set by the detach), so it drops
		 * straight into @dst_ft's empty root slot.  @subtree keeps the
		 * pre-allocated empty root so its destroy below frees nothing of
		 * the moved content.  @dst_ft was empty, so it adopts @subtree's
		 * whole ordered cell list wholesale (head/tail endpoints only).
		 */
		/*
		 * The FENCED root, not a fresh read: under the fence the two are
		 * equal by construction, and using the fenced value keeps the
		 * retire, the swap's expected-old and the free naming ONE node.
		 */
		old_dst_root = ft_node_ptr(dst_root_fenced);
		/*
		 * Fuse the structural root swap with the ordered-list head/tail
		 * transfer into ONE flip (ft_root_list_swap_publish), so a reader
		 * never sees the moved keys reachable in @dst_ft's structure but
		 * its ordered list still empty -- the dst-appear cross-view window,
		 * closed the same way as the graft empty-dst path.  Only @dst_ft
		 * has concurrent readers here: @subtree is the fresh EXCLUSIVE trie
		 * the detach above just produced, so its root reset + head/tail
		 * clear are plain stores (no src-side disappear window, unlike
		 * ft_graft's cross-trie empty-dst).
		 *
		 * Freeze-on-free (doc §4.B): the dst old root this swap retires
		 * gets its tombstone recorded INTO the swap txn, so the mark and
		 * the unlink flip atomically (atomic detach).  Failure-free past
		 * the detach above, so this runs only on the committing path.
		 */
		/*
		 * @subtree->root becomes @dst_ft's root, so re-name its owner
		 * before either branch publishes it -- a cross-trie move leaves
		 * the back-edge naming the trie it came FROM, which is exactly
		 * the aliasing cds_ft_verify reports at depth 0.  @subtree is the
		 * fresh EXCLUSIVE detach product with no readers, so the store is
		 * safe ahead of the flip.
		 */
		cds_ft_item_to_metadata(ft_node_ptr(subtree->root))->parent_word =
			ft_trie_parent(dst_ft);
		if (dst_ft->group->ordered_list_set) {
			/*
			 * dst FILLS by adopting @subtree's whole list.  @subtree is the
			 * fresh EXCLUSIVE detach product (no readers), so its incoming run
			 * relinks to dst's sentinel in the SAME flip (relink_dest = subtree,
			 * relink_incoming = true); @subtree's sentinel resets to empty with
			 * a plain store.
			 */
			ft_flip_txn_lock_register(appear_txn, dst_rmeta,
				dst_root_snap);
			ft_flip_txn_record_tombstone_locked(appear_txn,
				dst_rmeta, dst_root_snap);
			ft_root_list_swap_publish(dst_ft, appear_txn, &dst_ft->root,
				dst_root_fenced, subtree->root,
				NULL, ft_ord_first(subtree),
				NULL, ft_ord_last(subtree), subtree, true);
			urcu_txn_list_init(&subtree->ord_sentinel);
		} else {
			/*
			 * No ordered list: dst's root is the only reader-visible
			 * structural slot.  Commit the appear root edge and the old-
			 * root tombstone as ONE 2-edge flip through the pre-reserved
			 * txn (a lone root edge would reduce to a release store, but
			 * the fused tombstone makes it multi-edge -- readers resolve
			 * the transient root proxy exactly as on the list-on path).
			 * (@subtree is the fresh EXCLUSIVE trie, so its root reset
			 * below stays a plain store.)
			 */
			ft_flip_txn_record_root(appear_txn,
				(void **) &dst_ft->root,
				(void *) dst_root_fenced, (void *) subtree->root);
			ft_flip_txn_lock_register(appear_txn, dst_rmeta,
				dst_root_snap);
			ft_flip_txn_record_tombstone_locked(appear_txn,
				dst_rmeta, dst_root_snap);
			ft_flip_txn_commit(dst_ft, appear_txn);
		}
		FT_TP(root_publish, (const void *) dst_ft,
			(const void *) dst_ft->root);
		subtree->root = ft_node_flag(fresh_root, 0);
		free_cds_ft_node(dst_ft, old_dst_root);

		/* dst_key_len == 0: dst keys equal the moved keys, same lengths. */
		sm = uatomic_load(&subtree->max_used_key_len, CMM_RELAXED);
		if (sm > uatomic_load(&dst_ft->max_used_key_len, CMM_RELAXED))
			uatomic_store(&dst_ft->max_used_key_len, sm, CMM_RELAXED);

		cds_ft_destroy(subtree);
		FT_TP(merge_exit, (int) CDS_FT_STATUS_OK);
		return CDS_FT_STATUS_OK;
	}

diverged:
	/*
	 * Sub-position source into a dst absent at @dst_key: graft the source
	 * subtree IN PLACE (no detach, no re-root) so the build-invisible cluster
	 * references @d_src.nf directly and the source unlink is the last fallible
	 * step -- the leak-free reorder.  Handles every source-root shape (plain
	 * multi-child internal, an internal carrying external_nodes, external,
	 * compressed, 1-child) at both a GLUE diverge and a NOSPLIT (at-node /
	 * build-branch) dst point.
	 *
	 * A KEY_SHORTER source (the src key ends @off_src bytes inside the
	 * compressed node @d_src.nf) reduces to the EXACT shape: the moved subtree
	 * is @cn_s->child, and its keys gain the residual bytes cn_s->key_bytes
	 * [off_src .. len) as a prefix, so graft @cn_s->child at @dst_key extended
	 * by that residual.  ft_merge_unlink_src_subtree overshoots @cn_s (preserves
	 * @cn_s->child) so the source side is identical; @cn_s itself is reclaimed
	 * by that unlink.
	 */
	if (ks == FT_GRAFT_SWAP_EXACT || ks == FT_GRAFT_SWAP_KEY_SHORTER) {
		bool handled;
		struct cds_ft_inode_flag *payload;
		const uint8_t *gokey = okey_dst, *gappkey = dst_key;
		size_t glen = dst_key_len;
		uint8_t ext_okey[FT_MAX_KEY_LEN], ext_app[FT_MAX_KEY_LEN];

		if (ks == FT_GRAFT_SWAP_EXACT) {
			payload = d_src.nf;	/* off_src == 0 by construction */
		} else {
			struct cds_ft_compressed_node *cn_s =
				ft_compressed_node_ptr(d_src.nf);
			unsigned int rlen = cn_s->len - off_src, j;
			const struct cds_ft_key_map *km =
				&dst_ft->group->key_map;

			payload = cn_s->child;
			memcpy(ext_okey, okey_dst, dst_key_len);
			memcpy(&ext_okey[dst_key_len], &cn_s->key_bytes[off_src],
				rlen);
			memcpy(ext_app, dst_key, dst_key_len);
			if (km->identity) {
				memcpy(&ext_app[dst_key_len],
					&cn_s->key_bytes[off_src], rlen);
			} else {
				for (j = 0; j < rlen; j++)
					ext_app[dst_key_len + j] =
						km->ordinal_to_key[
						cn_s->key_bytes[off_src + j]];
			}
			gokey = ext_okey;
			gappkey = ext_app;
			glen = dst_key_len + rlen;
		}

		status = ft_merge_graft_subpos_inplace(dst_ft, src_ft,
			gokey, gappkey, glen, okey_src, src_key_len,
			payload, cnt_src, &handled);
		if (handled) {
			FT_TP(merge_exit, (int) status);
			return status;
		}
	}

	/*
	 * Unreachable.  ft_merge_descend reports EXACT, KEY_SHORTER or DELEGATE;
	 * DELEGATE returned OK at the top (cnt_src == 0), and every EXACT /
	 * KEY_SHORTER source is handled above: a non-empty dst by the build-
	 * invisible spine copy, an empty dst root or whole-source move by their
	 * failure-free root swaps, and a diverged / absent dst point by
	 * ft_merge_graft_subpos_inplace (which always reports handled).  No shape
	 * falls through, so the old detach-then-graft fallback -- the last path
	 * that carried the double-OOM leak -- and its rollback are gone.
	 */
	assert(0 && "cds_ft_merge_at: unhandled merge shape");
	status = CDS_FT_STATUS_OK;
out:
	FT_TP(merge_exit, (int) status);
	return status;
}

#endif /* FEATURE_FT_MERGE */

enum cds_ft_status cds_ft_merge_at(struct cds_ft *dst_ft,
		const uint8_t *dst_key, size_t dst_key_len,
		struct cds_ft *src_ft,
		const uint8_t *src_key, size_t src_key_len)
{
#ifdef FEATURE_FT_MERGE
	return ft_merge_at_inner(dst_ft, dst_key, dst_key_len, src_ft,
			src_key, src_key_len, NULL);
#else
	(void) dst_ft; (void) dst_key; (void) dst_key_len;
	(void) src_ft; (void) src_key; (void) src_key_len;
	return CDS_FT_STATUS_NOT_SUPPORTED;
#endif
}

#ifdef FEATURE_FT_MERGE
/*
 * The in-trie move, over TWO writers: the ATOMIC one (one decide) where it
 * applies, the staged one everywhere else.
 *
 * WHY THE ATOMIC WRITER IS TRIED FIRST, and it is not an optimization.  The
 * property a rekey exists to provide is that a key present throughout the move
 * is never read as ABSENT -- at neither the old nor the new position.  The staged
 * writer cannot provide it by construction: it detaches the subtree into a
 * transient trie as ONE COMMIT and merges that trie back as ANOTHER, so between
 * the two the moved keys are in no trie at all, for at least the grace period the
 * detach drains (measured by inv_rekey_public_no_gap: ~745 absence observations
 * per move).  ft_rekey_one_decide commits the src-slot clear, the destination
 * publish and the re-parents as ONE flip, so a reader sees the subtree at the
 * source XOR the destination and never at neither.
 *
 * The atomic writer covers a CUT of the shapes, not all of them, and reports
 * -EINVAL for the rest (a compressed or external S_top, a source junction that
 * would collapse, an ordered-list interleave, a destination abutting the moved
 * run's own ordered neighbourhood, an unequal-length destination).  Those fall
 * back here.  Either group flavour may take it: a fixed-length group MUST, having
 * nothing to fall back TO -- the staged writer is variable-length-only, since a
 * detached subtree's keys are stripped of the prefix -- so ft_merge_at_inner
 * refuses those rather than losing the subtree between its two commits.
 *
 * ARM THE MOVE GATE around both.  ft_move_gate_enter publishes @move_active and
 * waits ONE grace period, so every reader already inside a critical section --
 * which may have branched to the FAST, non-verifying lookup -- finishes before
 * anything moves; readers that start after it see the gate and take the coherent
 * two-pass path, which re-descends when its two traversals disagree.  A burst of
 * concurrent rekeys pays ~one grace period in total.  Without the gate the
 * two-pass machinery is unreachable in production: the coherent specializations
 * are selected by @rekey_coherence but ENTERED only under ft_move_active().  ONE
 * bracket covers the attempt AND the fallback -- two would pay two grace periods
 * for one move.
 *
 * CALLER CONTRACT, inherent to the gate: this BLOCKS on a grace period, so it
 * must not be called from inside an RCU read-side critical section -- the grace
 * period would wait on the caller's own section.  The cross-trie cds_ft_merge_at
 * has no such contract: its source is exclusive, so it moves no live key and arms
 * no gate.
 */

/*
 * ============================================================================
 * REKEY TWINS of the three merge workers that carry ATOMICITY.
 *
 * A same-trie rekey (cds_ft_rekey_{graft,merge}) reaches these bodies with
 * src_ft == dst_ft, so its "src side" IS the live shared destination; a
 * cross-trie merge reaches them with an EXCLUSIVE src that the entry gates
 * enforce (src_ft != dst_ft && !src_ft->exclusive -> BUSY_ERROR).  The two
 * callers therefore need OPPOSITE src-side atomicity, and the shared bodies
 * had NO fork on which caller they had -- 0 tests on @rekey or src_ft ==
 * dst_ft across 599 lines of worker code -- so both were served by the
 * stricter contract, unconditionally and invisibly.
 *
 * Split by DUPLICATION first, deliberately: these are byte-identical copies
 * modulo the three names, so this step changes no behaviour and the
 * specialisation of each side can then be reviewed as its own diff against a
 * known-equal base.
 *
 * The boundary is DERIVED, not chosen: exactly the functions that COMMIT,
 * DRAIN, or CREATE A TXN are twinned.  The other 15 functions reachable from
 * here (ft_merge_build, ft_merge_count, ft_merge_ord_interleave_collect, ...)
 * compute shape only and carry no atomicity contract, so they stay SHARED --
 * as the atomic rekey path already demonstrates by calling three of them.
 * ============================================================================
 */


#endif /* FEATURE_FT_MERGE */


enum cds_ft_status cds_ft_merge(struct cds_ft *dst_ft,
		const uint8_t *key, size_t key_len,
		struct cds_ft *src_ft)
{
	return cds_ft_merge_at(dst_ft, key, key_len, src_ft, key, key_len);
}
