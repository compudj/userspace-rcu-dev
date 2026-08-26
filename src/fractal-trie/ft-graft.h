// SPDX-FileCopyrightText: 2012-2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-only

/*
 * src/fractal-trie/ft-graft.h
 *
 * Userspace RCU library - Fractal Trie: graft / graft_swap + store-at-graft-point + glue.
 *
 * Implementation unit: #included once by fractal-trie.c, in dependency
 * order, into a single translation unit (preserves cross-module inlining).
 * Not a standalone header.
 */
#ifndef FRACTAL_TRIE_IMPL
#error "ft-graft.h is an implementation unit; #include it from fractal-trie.c only"
#endif

/*
 * Build-invisible diverge split for cds_ft_graft (the merge variant of
 * ft_split_compressed_graft).  Where the legacy split publishes a 1-child
 * branch for ft_store_at_graft_point to complete -- leaving a non-canonical
 * internal live if that later, fallible attach OOMs -- this builds the
 * COMPLETE attach cluster invisibly:
 *
 *   [prefix] -> branch{ old_ordinal -> old_suffix -> old_child,
 *                       new_ordinal -> [path] -> payload }
 *
 * Nothing is published, @cn is not freed, and every edge into LIVE data
 * (the displaced @old_child and the live @payload nodes) is recorded as a
 * deferred back-pointer in @glue.  An OOM frees the cluster via the
 * caller's ft_glue_abort with both tries pristine.
 *
 * The branch is built with BOTH children up front (two ft_node_set_nth
 * calls, the second possibly reallocating the node) using cluster_leaf so
 * neither child's back-pointer is set during the build; both are then
 * deferred against the FINAL branch.  This avoids the
 * publish-then-complete window and keeps the deferred old-child edge
 * anchored to a stable node.
 *
 * @key/@key_len: full ordinal key being grafted.
 * @diverge_pos:  divergence offset within cn's path (cn->key_bytes).
 * @payload:      source's old root (live), attached at depth @key_len.
 * @src_count:    payload key count, for the deferred propagate.
 *
 * Returns 0 (glue holds the cluster, its publish, and attached_nf), or
 * -ENOMEM (caller runs ft_glue_abort).
 */
static
int ft_split_compressed_graft_build(struct cds_ft *ft,
		struct ft_descent *d,
		const uint8_t *key, size_t key_len,
		unsigned int diverge_pos,
		struct cds_ft_inode_flag *payload,
		unsigned long src_count,
		struct ft_glue *glue,
		const struct ft_held_set *outer)
{
	struct cds_ft_compressed_node *cn = ft_compressed_node_ptr(d->nf);
	struct cds_ft_metadata *cn_meta =
		cds_ft_item_to_metadata((struct cds_ft_inode *) cn);

	/* Compressed metadata never carries external_nodes (see
	 * ft_split_compressed_insert). */
	assert(!cn_meta->external_nodes);
	/*
	 * F2 node lock, split-retire (MW LOCK_FINE drop): fence @cn BEFORE
	 * any plan read -- this build derives its WHOLE plan from @cn (the
	 * diverge slicing over cn->key_bytes, the cn_child snapshot, the
	 * deferred-edge captures) and RETIRES @cn, exactly as
	 * ft_split_compressed_insert fences the node it splits.  A peer that
	 * splits / grows / recompacts @cn, OR changes cn_child (X->X') and
	 * releases @cn, must either hold the fence (mark fails -> -EAGAIN,
	 * re-descend, NOTHING built) or abort at commit against the fenced
	 * tombstone's precise expected old (ft_glue_txn_commit_edges records
	 * {LOCK|s -> TOMBSTONE|s} from @split_cn_snap).  Marking here (not in
	 * the caller's pre-swap fence block) closes the read-then-fence window:
	 * the whole build runs under the fence, so a cn_child change during the
	 * build is caught too.  Gated on a txn'd graft under the drop: the
	 * txn-less merge-rekey (glue->txn NULL) and the FT-wide-lock builds keep
	 * the prior behaviour.  On the fence-miss path nothing is
	 * built and @cn is NOT marked (a clean re-descend); an OOM AFTER the mark
	 * leaves @cn marked for the caller (ft_graft_keylen) to clear.
	 */
	if (ft->lock_fine && glue->txn && glue->fence_split_cn) {
		struct ft_lock_ctx sctx;
		struct ft_held_anchor sh;

		ft_glue_lock_ctx(glue, &sctx);
		sctx.d = d;
		/*
		 * @outer is the REST of the op's held set: a caller whose EARLIER
		 * step took marks reaching no registry (the rekey fold's
		 * ft_rekey_cow_stop set) chains its frame here, exactly as
		 * ft_store_at_graft_point_prepare does.  Under a coarse spacing
		 * those marks and @cn's fence collapse onto one word -- the trie
		 * root, for an in-trie move -- and without the frame this acquire
		 * refuses a word the op itself holds.  That refusal is not a
		 * failure but a SPIN: the caller re-descends, rebuilds the
		 * identical shape and refuses again.
		 */
		sctx.held.outer = outer;
		if (ft_acquire_member(ft, &sctx, d->nf, cn_meta, d->depth, &sh))
			return -EAGAIN;	/* peer owns @cn; nothing built */
		/*
		 * A SHARED hit is the dedupe WORKING, not a refusal: the op
		 * already holds @cn's anchor, so this member owes NO release and
		 * NO anchor terminal (the first acquire recorded both).  @cn's
		 * OWN word is still retired here -- the tombstone lands on the
		 * node, not on the shared anchor -- so the node fields are
		 * recorded either way and @split_cn_shared gates the release half.
		 */
		glue->split_cn_holder = sh.lock;
		glue->split_cn_snap = sh.lock_snap;
		glue->split_cn_shared = sh.shared;
		glue->split_cn_node = cn_meta;
		glue->split_cn_node_snap = sh.node_snap;
	}
	/*
	 * DROP THE OLD DIRECTION (@glue->drop_old_dir_of): @cn's one child is
	 * a subtree this same decide MOVES, and @payload is its copy.  So the
	 * old half of the split has nothing to hold -- build ONLY the new
	 * key's path over the span @cn covered and publish that in its place.
	 *
	 * The path is @key[d->depth .. key_len), not the usual
	 * [new_depth .. key_len): the prefix @cn shares with the new key is
	 * that key's own bytes, so one ft_build_branch lays prefix, branch
	 * byte and remainder in a single canonical run -- no branch node is
	 * built for a fork that has only one arm left.
	 *
	 * @cn is fenced and deferred-freed exactly as the two-armed split
	 * does, so the retire side is unchanged; only the built shape differs.
	 * The displaced child gets NO deferred edge -- it is not re-parented,
	 * it is left behind, and the caller retires it as part of the move.
	 */
	if (glue->drop_old_dir_of && glue->drop_old_dir_of == d->nf) {
		struct cds_ft_inode_flag *canon, *top;
		unsigned long moved_keys = src_count;

		canon = ft_compress_single_child_if_needed(ft, payload, glue);
		if (canon == (struct cds_ft_inode_flag *) (long) -ENOMEM)
			return -ENOMEM;
		top = ft_build_branch(ft, key, d->depth, (unsigned int) key_len,
			canon, moved_keys, false, glue);
		if (!top)
			return -ENOMEM;
		ft_glue_set_publish(ft, glue, d->pnf, d->nfp, top);
		ft_glue_defer_free(glue, cn, true);
		glue->attached_nf = top;
		glue->old_dir_dropped = true;
		return 0;
	}
	unsigned int suffix_len = cn->len - diverge_pos - 1;
	uint8_t old_ordinal = cn->key_bytes[diverge_pos];
	uint8_t new_ordinal = key[d->depth + diverge_pos];
	unsigned int new_depth = d->depth + diverge_pos + 1;
	bool branch_cluster_leaf = (suffix_len == 0);
	struct cds_ft_inode_flag *old_suffix_flag;
	struct cds_ft_inode_flag *sfx_skip_flag = NULL;
	struct cds_ft_inode_flag *branch_flag, *top_flag;
	struct cds_ft_inode_flag *new_dir, *payload_canon;
	struct cds_ft_inode_flag **slot;
	struct cds_ft_inode *old_branch = NULL;
	unsigned long old_child_nr_keys;
	int ret;
	/*
	 * THE ONE @cn->child LOAD, SETTLED.  This build derives its whole plan
	 * from @cn under the F2 fence, and the child is part of that plan: it is
	 * dereferenced for the old nr_keys, wired into the fresh suffix node, and
	 * handed to the deferred back-edges.  A raw load can catch a peer's parked
	 * engine proxy -- a descriptor-record POINTER -- and the fence on @cn's
	 * state word does not exclude one: a peer mid-commit on this slot has
	 * already parked, and only its COMMIT is arbitrated (against the fenced
	 * tombstone's expected-old).  Dereferencing that pointer as a node faults;
	 * copying it into @sfx->child would publish it.  Resolving yields the value
	 * the slot denotes, and the commit still ratifies the plan.
	 */
	struct cds_ft_inode_flag *cn_child = ft_resolve_flip_proxy(cn->child);

	(void) branch_cluster_leaf;	/* documents intent; both set_nth defer */

	/* Compute old child's nr_keys. */
	if (!ft_node_external(cn_child)) {
		struct cds_ft_metadata *cm =
			cds_ft_item_to_metadata(ft_node_ptr(cn_child));
		old_child_nr_keys = ft_nr_keys_get(cm);
	} else if (cn_child) {
		old_child_nr_keys = 1;
	} else {
		old_child_nr_keys = 0;
	}

	/*
	 * 1. Build the OLD-direction suffix -> old child (mirrors the legacy
	 * split).  cn_child (live) is deferred into @glue.
	 */
	if (suffix_len >= 2
#ifdef FEATURE_FT_SKIP_COMPRESSED
			|| (suffix_len == 1 && ft_group_skip_compressed(ft->group))
#endif
	   ) {
		struct cds_ft_compressed_node *sfx;
		struct cds_ft_metadata *sfx_meta;

		sfx = alloc_compressed_node(ft, suffix_len, &sfx_meta);
		if (!sfx)
			return -ENOMEM;
		sfx->child = cn_child;
		sfx->len = suffix_len;
		memcpy(sfx->key_bytes, &cn->key_bytes[diverge_pos + 1],
			suffix_len);
		ft_meta_nr_child_set(sfx_meta, 1);
		ft_nr_keys_store(ft, sfx_meta, old_child_nr_keys, CMM_RELAXED);
		old_suffix_flag = ft_compressed_node_flag(sfx);
		sfx_skip_flag = ft_publish_compressed(ft, sfx, old_suffix_flag);
		ft_glue_track(glue, old_suffix_flag);
		/*
		 * The displaced child @cn_child is LIVE: a reader can still descend
		 * to it through @cn (the compressed node being split, untouched until
		 * the forward publish replaces it).  So its re-parent onto the fresh
		 * @sfx is a reader-observable pointer -- ride it on the flip-txn
		 * (dst_origin) so it flips atomically with the forward edge.  Holds for
		 * an external @cn_child too: the up-walk readers (ft_get_parent_rcu /
		 * ft_skip_to_compressed / ft_skip_reanchor) resolve a flip proxy parked
		 * on an external's parent.  The merge-rekey path has no txn (glue->txn
		 * NULL); there it stays on the legacy fresh-before-live immediate store.
		 */
		ft_glue_defer_edge_origin(ft, glue, cn_child, old_suffix_flag,
			&sfx->child, glue->txn != NULL);
	} else if (suffix_len == 1) {
		struct cds_ft_inode_flag *dest = NULL;

		/* 1-child internal suffix (non-SC): cluster-leaf. */
		ret = ft_node_set_nth(ft, &dest,
				cn->key_bytes[diverge_pos + 1],
				cn_child, NULL, NULL,
				d->depth + diverge_pos + 1, true);
		if (ret)
			return -ENOMEM;
		ft_nr_keys_store(ft, cds_ft_item_to_metadata(ft_node_ptr(dest)),
			old_child_nr_keys, CMM_RELAXED);
		old_suffix_flag = dest;
		ft_glue_track(glue, dest);
		ft_node_get_nth_skip(dest, &slot,
			cn->key_bytes[diverge_pos + 1], FT_PF_NONE);
		/* Displaced child: LIVE, ride the txn -- see the suffix_len>1 case. */
		ft_glue_defer_edge_origin(ft, glue, cn_child, dest, slot,
			glue->txn != NULL);
	} else {
		old_suffix_flag = cn_child;	/* suffix_len == 0 */
	}

	/*
	 * 2. Build the NEW-direction subtree: canonicalize the payload, then
	 * (when the key extends past the branch) a path down to it.  All
	 * build-invisible; payload back-pointers deferred via @glue.
	 */
	payload_canon = ft_compress_single_child_if_needed(ft, payload, glue);
	if (payload_canon == (struct cds_ft_inode_flag *) (long) -ENOMEM)
		return -ENOMEM;
	if (new_depth == key_len) {
		new_dir = payload_canon;
	} else {
		new_dir = ft_build_branch(ft, key, new_depth, key_len,
			payload_canon, src_count, false, glue);
		if (!new_dir)
			return -ENOMEM;
	}

	/*
	 * 3. Build the branch with BOTH children.  cluster_leaf on both
	 * set_nth: no child back-pointer is set during the build (the second
	 * set_nth may reallocate the branch).  Both are deferred below
	 * against the final branch.
	 */
	branch_flag = NULL;
	ret = ft_node_set_nth(ft, &branch_flag, old_ordinal, old_suffix_flag,
			NULL, NULL, d->depth + diverge_pos, true);
	if (ret)
		return -ENOMEM;
	ft_glue_track(glue, branch_flag);
	/*
	 * The branch is a fresh, unpublished node with no parent yet (it is
	 * wired to its prefix only at commit).  Clear its parent / skip_slot
	 * before the second child may reallocate it: ft_node_recompact
	 * inherits the old node's parent and, if that parent looks like a
	 * compressed node, writes through its skip_slot -- a recycled
	 * allocation can leave stale, non-NULL values there and corrupt an
	 * unrelated live node.
	 */
	{
		struct cds_ft_metadata *bm =
			cds_ft_item_to_metadata(ft_node_ptr(branch_flag));

		bm->parent_word = NULL;
#ifdef FEATURE_FT_SKIP_COMPRESSED
		ft_meta_parent_slot_offset_set(bm, 0);
#endif
	}
	/*
	 * Second child: the branch is now an existing (unpublished) node,
	 * so pass its metadata for the in-place update; on overflow it
	 * reallocates (old order-1 copy returned via @old_branch).
	 */
	ret = ft_node_set_nth(ft, &branch_flag, new_ordinal, new_dir,
			&old_branch,
			cds_ft_item_to_metadata(ft_node_ptr(branch_flag)),
			d->depth + diverge_pos, true);
	if (ret)
		return -ENOMEM;
	if (old_branch) {
		/*
		 * Reallocated: drop the order-1 copy from tracking + RETIRE it.
		 *
		 * The grow ran through ft_node_recompact, which marks every body
		 * it supersedes DEAD (§4.B freeze-on-free) and states the matching
		 * obligation -- "the caller ... frees @old_node after a grace
		 * period".  The unpublished path is the wrong one even though this
		 * cluster is build-invisible: its contract is proven by asserting
		 * the node carries NO tombstone, so a marked body freed there
		 * reads as a live-node free.  ft-insert.h's own recompact-on-grow
		 * (@old_recompacted) already frees through here.
		 */
		ft_glue_untrack(ft, glue, old_branch);
		free_cds_ft_node(ft, old_branch);
		ft_glue_track(glue, branch_flag);
	}
	/*
	 * Order-statistics fold (BULK): build the fresh cluster's junction /
	 * prefix with their FULL post-commit count -- the old span's keys PLUS
	 * the +src_count payload -- so the graft/merge attach fold records only
	 * the +src_count walk from the STABLE @publish_parent above the cluster
	 * (a build-invisible plain store; a no-op when rank stats are off).  The
	 * OLD-direction suffix nodes keep old_child_nr_keys (they hold no
	 * payload).
	 */
	ft_nr_keys_store(ft, cds_ft_item_to_metadata(ft_node_ptr(branch_flag)),
		old_child_nr_keys + src_count, CMM_RELAXED);

	/* Wire the OLD direction (re-encode compressed slot to skip form). */
	ft_node_get_nth_skip(branch_flag, &slot, old_ordinal, FT_PF_NONE);
	if (sfx_skip_flag && sfx_skip_flag != old_suffix_flag && slot)
		*slot = sfx_skip_flag;
	/*
	 * suffix_len == 0: @old_suffix_flag IS the live @cn_child wired directly
	 * under the branch (no fresh suffix node), so this edge re-parents a
	 * reader-reachable node -- dst_origin (ride the txn).  suffix_len > 0:
	 * @old_suffix_flag is the fresh sfx/dest, a hidden edge (src-origin).
	 */
	ft_glue_defer_edge_origin(ft, glue, old_suffix_flag, branch_flag, slot,
		suffix_len == 0 && glue->txn != NULL);
	/* Wire the NEW direction. */
	ft_node_get_nth_skip(branch_flag, &slot, new_ordinal, FT_PF_NONE);
	if (ft_node_compressed(new_dir) && slot) {
		/*
		 * @new_dir is a PLAIN compressed flag (compress and the
		 * build-invisible ft_build_branch both return the plain form
		 * so the deferred edge recovers it via ft_compressed_node_ptr).
		 * Re-encode the holding slot to the skip form so the published
		 * trie is canonical; it resolves once the deferred grandchild
		 * back-pointer is applied at commit.
		 */
		struct cds_ft_inode_flag *skip = ft_publish_compressed(ft,
			ft_compressed_node_ptr(new_dir), new_dir);

		if (skip != new_dir)
			*slot = skip;
	}
	ft_glue_defer_edge(ft, glue, new_dir, branch_flag, slot);

	/* 4. Build prefix -> branch (no external_nodes on @cn). */
	if (diverge_pos >= 2) {
		struct cds_ft_compressed_node *pfx;
		struct cds_ft_metadata *pfx_meta;

		pfx = alloc_compressed_node(ft, diverge_pos, &pfx_meta);
		if (!pfx)
			return -ENOMEM;
		pfx->child = branch_flag;
		pfx->len = diverge_pos;
		memcpy(pfx->key_bytes, cn->key_bytes, diverge_pos);
		ft_meta_nr_child_set(pfx_meta, 1);
		ft_nr_keys_store(ft, pfx_meta,
			ft_nr_keys_get(cn_meta) + src_count, CMM_RELAXED);
		top_flag = ft_compressed_node_flag(pfx);
		ft_set_parent(ft, branch_flag, top_flag, NULL);
		/* Track the PLAIN form; the skip form is for the publish. */
		ft_glue_track(glue, top_flag);
		top_flag = ft_publish_compressed(ft, pfx, top_flag);
	} else if (diverge_pos == 1) {
#ifdef FEATURE_FT_SKIP_COMPRESSED
		if (ft_group_skip_compressed(ft->group)) {
			struct cds_ft_compressed_node *pfx;
			struct cds_ft_metadata *pfx_meta;

			pfx = alloc_compressed_node(ft, 1, &pfx_meta);
			if (!pfx)
				return -ENOMEM;
			pfx->child = branch_flag;
			pfx->len = 1;
			pfx->key_bytes[0] = cn->key_bytes[0];
			ft_meta_nr_child_set(pfx_meta, 1);
			ft_nr_keys_store(ft, pfx_meta,
				ft_nr_keys_get(cn_meta) + src_count, CMM_RELAXED);
			top_flag = ft_compressed_node_flag(pfx);
			ft_set_parent(ft, branch_flag, top_flag, &pfx->child);
			/* Track the PLAIN form; skip form for the publish. */
			ft_glue_track(glue, top_flag);
			top_flag = ft_publish_compressed(ft, pfx, top_flag);
			goto after_prefix;
		}
#endif
		{
		struct cds_ft_inode_flag *dest = NULL;
		struct cds_ft_metadata *pfx_meta;

		ret = ft_node_set_nth(ft, &dest, cn->key_bytes[0],
				branch_flag, NULL, NULL, d->depth, false);
		if (ret)
			return -ENOMEM;
		pfx_meta = cds_ft_item_to_metadata(ft_node_ptr(dest));
		ft_nr_keys_store(ft, pfx_meta,
			ft_nr_keys_get(cn_meta) + src_count, CMM_RELAXED);
		top_flag = dest;
		ft_glue_track(glue, dest);
		}
#ifdef FEATURE_FT_SKIP_COMPRESSED
	after_prefix:
		(void) 0;
#endif
	} else {
		/* diverge_pos == 0: branch IS the top. */
		ft_nr_keys_store(ft, cds_ft_item_to_metadata(ft_node_ptr(branch_flag)),
			ft_nr_keys_get(cn_meta) + src_count, CMM_RELAXED);
		top_flag = branch_flag;
	}

	/*
	 * 5. Record the single publish (top -> d->pnf's slot) and the old
	 * compressed node to free at commit.  attached_nf == new_dir: it
	 * carries the payload's key count, and the propagate starts from its
	 * parent.
	 */
	ft_glue_set_publish(ft, glue, d->pnf, d->nfp, top_flag);
	ft_glue_defer_free(glue, cn, true);
	glue->attached_nf = new_dir;
	return 0;
}

/*
 * The NOSPLIT graft-point store, split into a fallible build (prepare) and a
 * failure-free publish (commit) so the two halves can straddle the caller's
 * source mutation.  cds_ft_graft runs them back-to-back via the combined
 * ft_store_at_graft_point wrapper below (after it has unlinked the source root
 * and drained); cds_ft_merge_at's diverged-dst move runs prepare while the
 * source is still pristine, then unlinks the source subtree + drains, then
 * commits -- so its only fallible dst step finishes before the source is
 * touched and there is no rollback to leak from.  (The diverging case is built
 * invisibly by ft_split_compressed_graft_build, not here.)
 *
 * Two non-diverging attach shapes:
 * - d->depth == key_len: the slot exists; add via ft_node_set_nth.
 * - d->depth < key_len: build intermediate internal nodes via ft_build_branch,
 *   displacing any external node on the path into the branch's metadata.
 *
 * prepare is non-destructive to the payload (live source data): its
 * back-pointers route through @glue, ft_compress_single_child_if_needed is
 * non-destructive in glue mode, and the forward slot is parked behind a flip
 * proxy resolving to the (empty) old value -- so an OOM in prepare leaves both
 * the payload and the destination pristine (the caller runs ft_glue_abort).
 * commit applies the deferred edges, swings the forward publish and reclaims the
 * replaced source root (if canonicalized); it cannot fail.
 */
struct ft_graft_store_state {
	struct ft_glue *glue;
	bool displaced_shape;
	struct cds_ft_inode_flag *attached;		/* payload (at-node) or branch */
	unsigned int attached_depth;
	/* flip publish (slot-at-node and built-branch-flip shapes): */
	struct cds_ft_inode_flag *dest;
	struct cds_ft_inode *old_recompacted_node;
	/*
	 * A recompact-relocating reserve records a compressed grandparent's
	 * SKIP_X dual here (instead of a premature bare store): the commit folds
	 * the forward grandparent slot edge in alongside it so both flip
	 * ATOMICALLY in glue->txn.
	 */
	struct ft_pub_rec reserve_rec;
	/*
	 * Set iff the reserve added its byte's occupancy IN PLACE on the LIVE
	 * @dest, i.e. this graft still owes @dest an nr_child++ -- recorded into
	 * glue->txn by the commit's in-place arm so the count goes live with the
	 * publish and is dropped with an aborted attempt.  A reserve that
	 * RELOCATED built the count into its fresh copy and leaves this false.
	 * See ft_flip_txn_record_nr_child_inc.
	 */
	bool count_deferred;
	struct cds_ft_metadata *publish_pmeta;
	struct cds_ft_inode_flag **pnfp;
	struct cds_ft_inode_flag *slot_value;
	uint8_t slot_byte;
	/* displaced publish (built branch absorbing an existing external): */
	struct cds_ft_node *displaced;
	struct cds_ft_inode_flag *pnf;
	struct cds_ft_inode_flag **nfp;
	const uint8_t *tp_key;
	unsigned int tp_i;
};

/*
 * Flip-batch capacity for a run-fused graft store: 1 structural slot edge +
 * the <=4 boundary edges of the ordered-list run-splice (ft_graft_run), all
 * committed in ONE flip.  A run-less store needs only the single slot edge.
 */
#define FT_GRAFT_RUN_FLIP_CAP	5

static
enum cds_ft_status ft_store_at_graft_point_prepare(struct cds_ft *ft,
		const uint8_t *key, size_t key_len,
		struct ft_descent *d,
		struct cds_ft_inode_flag *graft_payload,
		unsigned long graft_external_count,
		struct ft_glue *glue,
		const struct ft_held_set *outer,
		struct ft_graft_store_state *st)
{
	struct ft_lock_ctx gctx;

	memset(st, 0, sizeof(*st));
	st->glue = glue;
	/*
	 * The recompactions below lock {p, its parent, its grandparent}; @d is
	 * their anchor source, and @glue->txn the registry naming what this op
	 * already holds.
	 *
	 * @outer is the REST of what it holds: a caller whose EARLIER step took
	 * marks that reach no registry (the rekey fold's ft_rekey_cow_stop set)
	 * chains its frame here, because under a coarse spacing those marks and
	 * these recompactions collapse onto one word.  NULL for a caller with
	 * nothing outstanding.
	 */
	ft_lock_ctx_init(&gctx, d, glue->txn, glue->op);
	gctx.held.outer = outer;
	/*
	 * The glue's own acquires (its publish parent, its split CN) fire from
	 * commit helpers that never see @d, so hand it the anchor source here --
	 * the one place holding both.
	 */
	glue->lock_d = d;

	/*
	 * Skip-mode chain-compress invariant: under SPECULATIVE-mode tries,
	 * non-root 1-child internals without external_nodes must be a
	 * compressed.  graft_payload may be the source trie's old root (a
	 * 1-child internal is permitted at root, forbidden at the non-root
	 * position we are placing it in).  Canonicalize per branch -- deferred
	 * past the POPULATED_ERROR check so that on failure the caller can
	 * still reach the original payload for rollback.
	 */
	if (d->depth == key_len) {
		struct cds_ft_metadata *pmeta;
		struct cds_ft_inode_flag *dest;
		struct cds_ft_inode_flag *slot_value;
		int ret;

		if (d->nf)
			return CDS_FT_STATUS_POPULATED_ERROR;

		graft_payload = ft_compress_single_child_if_needed(ft,
			graft_payload, glue);
		if (graft_payload == (struct cds_ft_inode_flag *) (long) -ENOMEM)
			return CDS_FT_STATUS_MEMORY_ERROR;

		pmeta = cds_ft_item_to_metadata(ft_node_ptr(d->pnf));

		/*
		 * R8 ordering: the slot store and its possible recompact are the
		 * LAST fallible steps, so they run FIRST (reserved-byte model):
		 * key[key_len-1]'s slot is occupied with a reserved bit-set+NULL
		 * byte that reads as not-present.  An ENOMEM here leaves both
		 * tries untouched (no live edge flipped); the failure-free wiring
		 * runs behind the still-empty slot in commit.  The slot edge
		 * (NULL -> slot_value) is recorded against the FINAL slot in
		 * commit (once the recompact-relocated address is known), so the
		 * slot store flips atomically with the back-pointers and the
		 * ordered-list run-splice -- the appear-side cross-view fix -- in
		 * one txn commit.  Nothing is stored in the slot during the build.
		 */
		slot_value = graft_payload;
		if (ft_node_compressed(graft_payload))
			slot_value = ft_publish_compressed(ft,
				ft_compressed_node_ptr(graft_payload),
				graft_payload);
		dest = d->pnf;
		/*
		 * @glue->txn as the retire txn: a reserve that RECOMPACTS the
		 * live dst attach node must record its reparent sweep and the
		 * external-head back-channel INTO the glue commit (co-committed
		 * (parent, offset) pairs), not apply them as immediate stores
		 * onto the unpublished copy -- those were reader-visible via
		 * up-walks from the still-reachable children for the whole
		 * build-to-commit window.  NULL only on the txn-less merge-rekey
		 * flow, which keeps the legacy immediate wiring (its own item).
		 */
		ret = ft_node_set_nth_rec(ft, &dest, key[key_len - 1], NULL,
			&st->old_recompacted_node, pmeta, d->depth - 1, false,
			&st->reserve_rec, glue->txn,
			/* §11: recompact {p} under its LIVE reanchored parent
			 * (d->ppnf, d->pnfp), not {p}'s stale back-pointer; and,
			 * for the SKIP_X dual one level up, the LIVE great-
			 * grandparent (d->pppnf) + {p}'s-parent's coherent slot
			 * in it (d->ppnfp). */
			&(const struct ft_parent_hint){
				.parent = d->ppnf, .slot = d->pnfp,
				.gp = d->pppnf, .gp_slot = d->ppnfp,
				/*
				 * An ordinary hint deliberately skips the
				 * C.parent == P read-set guard: the MW
				 * cross-trie graft's parent identity is the
				 * CALLER's and must not be validated against
				 * C's lazily-updated back-pointer.  But the
				 * SAME hint serves the FOLD's NOSPLIT
				 * dst-parent recompaction, whose republish
				 * PARKS SW into @parent's slot -- and an SW
				 * park into a slot whose ownership rests on a
				 * shape argument is exactly what the guard
				 * exists to stop.  Ask for it when this is the
				 * fold (record_only), which is byte-neutral for
				 * the MW graft.
				 */
				.parent_guard = glue->record_only },
			&gctx, &st->count_deferred);
		/*
		 * -EAGAIN is a TRANSIENT peer conflict (the reserve found its
		 * byte filled under it), not an allocation failure: report it
		 * as BUSY so a retry-driving caller need not read "out of
		 * memory" to mean "re-descend".  Both callers already retry
		 * every non-OK status but POPULATED (cds_ft_merge_at's
		 * retry_merge, cds_ft_graft's retry_attach), so this is a
		 * truthfulness fix, not a control-flow change.
		 */
		if (ret)
			return ret == -EAGAIN ?
				CDS_FT_STATUS_BUSY_ERROR :
				CDS_FT_STATUS_MEMORY_ERROR;

		st->attached = graft_payload;
		st->attached_depth = (unsigned int) key_len;
		st->dest = dest;
		st->publish_pmeta = pmeta;
		st->pnfp = d->pnfp;
		st->slot_value = slot_value;
		st->slot_byte = key[key_len - 1];
	} else {
		unsigned int i = d->depth;
		struct cds_ft_inode_flag *branch;
		struct cds_ft_node *displaced = NULL;

		if (d->nf && ft_node_external(d->nf))
			displaced = (struct cds_ft_node *) ft_node_ptr(d->nf);

		graft_payload = ft_compress_single_child_if_needed(ft,
			graft_payload, glue);
		if (graft_payload == (struct cds_ft_inode_flag *) (long) -ENOMEM)
			return CDS_FT_STATUS_MEMORY_ERROR;

		branch = ft_build_branch(ft, key, i, key_len, graft_payload,
				graft_external_count, displaced != NULL, glue);
		if (!branch)
			return CDS_FT_STATUS_MEMORY_ERROR;

		if (displaced) {
			struct cds_ft_metadata *bm =
				ft_flag_to_metadata(ft, branch);
			/*
			 * Phase 1 (build-invisible): wire branch's own
			 * back-pointer into d->pnf and the cluster-internal
			 * external_nodes pointer.  The back-channel publish
			 * (displaced->prev = branch) and the deferred live
			 * flips are applied in commit, fresh-before-live.
			 */
			ft_set_parent(ft, branch, d->pnf, d->nfp);
			ft_metadata_set_external_nodes(branch, bm, displaced);
			ft_nr_keys_store(ft, bm, ft_nr_keys_get(bm) + 1,
				CMM_RELAXED);

			st->displaced_shape = true;
			st->attached = branch;
			st->attached_depth = d->depth;
			st->displaced = displaced;
			st->pnf = d->pnf;
			st->nfp = d->nfp;
			st->tp_key = key;
			st->tp_i = i;
		} else {
			struct cds_ft_inode_flag *dest = d->pnf;
			struct cds_ft_metadata *pmeta;
			int ret;

			pmeta = cds_ft_item_to_metadata(ft_node_ptr(d->pnf));

			/*
			 * Same R8 + fresh-before-live discipline as the
			 * d->depth == key_len arm (reserved-byte model):
			 * reserve key[i-1]'s slot (bit-set+NULL, reads
			 * not-present) so the fallible slot store runs FIRST;
			 * the slot edge (NULL -> branch) is recorded against
			 * the final slot and the wiring completes invisibly in
			 * commit.
			 */
			/* Same retire-txn routing as the depth == key_len arm. */
			ret = ft_node_set_nth_rec(ft, &dest, key[i - 1], NULL,
				&st->old_recompacted_node, pmeta,
				d->depth - 1, false, &st->reserve_rec,
				glue->txn,
				/* §11: recompact {p} under its LIVE reanchored
				 * parent (d->ppnf, d->pnfp), not the stale
				 * back-pointer; + the LIVE great-grandparent
				 * (d->pppnf) and {p}'s-parent's coherent slot in
				 * it (d->ppnfp) for the SKIP_X dual one level up. */
				&(const struct ft_parent_hint){
					.parent = d->ppnf,
					.slot = d->pnfp,
					.gp = d->pppnf,
					.gp_slot = d->ppnfp,
					/* Fold republish parks SW into
					 * @parent's slot: guard the identity.
					 * See the depth == key_len arm. */
					.parent_guard = glue->record_only },
				&gctx, &st->count_deferred);
			/* Transient peer conflict, not OOM: see the
			 * depth == key_len arm above. */
			if (ret)
				return ret == -EAGAIN ?
					CDS_FT_STATUS_BUSY_ERROR :
					CDS_FT_STATUS_MEMORY_ERROR;

			st->attached = branch;
			st->attached_depth = d->depth;
			st->dest = dest;
			st->publish_pmeta = pmeta;
			st->pnfp = d->pnfp;
			st->slot_value = branch;
			st->slot_byte = key[i - 1];
		}
	}
	return CDS_FT_STATUS_OK;
}

static
enum urcu_txn_status ft_store_at_graft_point_commit(struct cds_ft *ft,
		struct cds_ft_inode_flag **attached_nf,
		unsigned int *attached_depth,
		struct ft_graft_run *run,
		struct ft_graft_store_state *st,
		long count_delta)
{
	enum urcu_txn_status cst;

	if (st->displaced_shape) {
		/*
		 * The displaced external is LIVE -- the dst leaf stays reachable
		 * through the still-old slot until the forward publish replaces it --
		 * so its re-parent onto the fresh @branch (its back-channel
		 * displaced->prev / cell->parent = branch) is a reader-observable
		 * pointer.  Record it as a dst_origin edge so ft_glue_txn_commit flips
		 * it atomically with the forward publish + the run-splice cell edges,
		 * rather than a fresh-before-live store ahead of them.  (A flip proxy
		 * parked on an external's parent is resolved by the up-walk readers --
		 * ft_get_parent_rcu / ft_skip_to_compressed / ft_skip_reanchor.)  The
		 * payload's hidden back-pointers are wired immediately by
		 * ft_glue_apply_deferred inside the commit.
		 */
		ft_glue_defer_edge_origin(ft, st->glue,
			(struct cds_ft_inode_flag *) st->displaced,
			st->attached, NULL, /*dst_origin=*/ true);
		st->glue->publish_parent = st->pnf;
		st->glue->publish_slot = st->nfp;
		st->glue->top = st->attached;
		/*
		 * Order-statistics fold (BULK): the fresh @branch (which
		 * absorbs the displaced external) raises @st->pnf's subtree by
		 * +count_delta; @publish_parent == st->pnf, so the glue commit
		 * records that +count_delta walk into the same flip.
		 */
		st->glue->count_delta = count_delta;
		cst = ft_glue_txn_commit(ft, st->glue, run);
		if (st->tp_i >= 1)
			FT_TP(tree_edge_set, (const void *) ft,
				(const void *) st->pnf,
				(unsigned int) (st->tp_i - 1),
				(uint8_t) st->tp_key[st->tp_i - 1],
				(const void *) st->attached);
	} else {
		struct cds_ft_inode_flag **slot = NULL;
		struct ft_ord_cell_edge redges[FT_ORD_CELL_RUN_SPLICE_MAX_EDGES];
		struct cds_ft_inode_flag *count_base = NULL;
		unsigned int rn = 0;

		ft_node_get_nth_skip(st->dest, &slot, st->slot_byte, FT_PF_NONE);
		assert(slot);
		ft_set_parent(ft, st->attached, st->dest, slot);
		ft_glue_apply_deferred(ft, st->glue);
		if (st->old_recompacted_node) {
			/*
			 * MW LOCK_FINE drop (§11, cross-trie): the publish slot must
			 * come from st->dest's OWN (parent, offset) -- the COHERENT pair
			 * the recompact locked and stored into st->dest's meta
			 * (ft_node_recompact inh_parent / inh_slot) -- NOT the descent-
			 * time st->pnfp.  Under the FT-wide-lock drop a peer can relocate
			 * the graft-point grandparent between this graft's descent (which
			 * captured st->pnfp) and the recompact that re-anchored st->dest
			 * onto the NEW grandparent, leaving st->pnfp naming the OLD
			 * grandparent's slot while st->dest->parent names the new one --
			 * ft_set_parent_slot -> ft_slot_to_byte would then index the wrong
			 * body out of bounds (the SAME coherence rule as the graft_swap
			 * merged publish above and the remove-path PSO fix).  The
			 * grandparent is release-locked by the recompact, so this resolve
			 * is stable through the commit.
			 */
			struct cds_ft_metadata *dest_meta =
				cds_ft_item_to_metadata(ft_node_ptr(st->dest));
			struct cds_ft_inode_flag *pub_parent;
			struct cds_ft_inode_flag **pub_slot =
				ft_resolve_parent_slot(dest_meta, ft, &pub_parent);

			/*
			 * The reserve recompacted (relocated) the dst attach node.  A
			 * compressed grandparent's SKIP_X dual was RECORDED into
			 * st->reserve_rec by the reserve ft_node_set_nth_rec (deferred,
			 * not bare-stored prematurely ahead of the forward).  Add the
			 * forward grandparent slot edge to that same rec -- @parent_nf =
			 * st->dest, the relocated node, a plain internal, so
			 * _ft_publish_to_parent does NOT re-emit the SKIP_X dual (no
			 * duplicate edge) -- then record both into glue->txn (pre-reserved
			 * before the build, so no allocation here) so the relocation flips
			 * ATOMICALLY with the grafted slot edge and the run-splice in the
			 * single commit below.  The src drain does not cover dst, and the
			 * old dst node stays resolved-to via the parked grandparent proxy
			 * until the commit (freed below, after it).
			 */
			/*
			 * VALIDATE (§4.B): guard the LIVE grandparent this relocation
			 * republishes into (a concurrent freeze of it aborts the commit).
			 *
			 * LOCK_FINE (§9.3, step 6): this is the SAME word the recompact
			 * driven by ft_store_at_graft_point_prepare's ft_node_set_nth_rec
			 * acquired and recorded a release on -- its P -- earlier in this
			 * same glue->txn.  The release SUPERSEDES this guard (it makes peers
			 * abort up front, strictly stronger), so under lock_fine we DROP the
			 * guard exactly as insert (ft_insert_publish_or_park) and remove
			 * (ft_detach_node's recompact arm) do.  Ordering is safe by
			 * construction: the release was recorded first, so were the guard
			 * kept it would chain as a read-your-writes no-op -- never the
			 * poisoning guard-then-release order (rule at
			 * ft_flip_txn_record_release_lock).  Non-lock_fine keeps the guard.
			 */
			if (!ft->lock_fine)
				ft_flip_txn_guard_parent(ft, st->glue->txn,
					pub_parent);
			/*
			 * SETTLED, not raw, for the expected-old -- the same rule as
			 * the graft_swap merged publish and the five metadata reads
			 * fixed with it.  @pub_slot is a TRANSACTED word, and when the
			 * republish grandparent is the trie ROOT it is &ft->root, which
			 * a peer's root-level graft parks a proxy in.  A raw load then
			 * records that descriptor POINTER as the expected-old, and
			 * urcu_txn_settle stores it back blind on a matching commit --
			 * where nothing ever clears it, because its owning transaction
			 * decided and settled long before, so every later acquire of
			 * that word bails forever.  (Reached by a KEYED graft racing a
			 * ROOT graft into one destination -- inv_empty_dst_root_graft_
			 * peer, ~1 run in 40 under -DDEBUG_RCU.)
			 */
			/*
			 * ☠ @parent_nf IS st->dest FOR THE DUAL, @slot_owner_nf IS
			 * @pub_parent FOR THE OWNER, and they are different nodes.
			 * st->dest is passed above only so a compressed grandparent's
			 * SKIP_X dual is not re-emitted (it was already recorded by the
			 * reserve); the slot being published lives in @pub_parent, which
			 * is the word the recompact acquired and this txn holds.  Naming
			 * st->dest as the owner reports an exclusion gap that is not real.
			 */
			_ft_publish_to_parent_meta(ft, st->dest,
				pub_slot, st->dest,
				ft_resolve_flip_proxy(*pub_slot),
				NULL, NULL, &st->reserve_rec,
				/*slot_owner_nf=*/ pub_parent, false);
			ft_flip_txn_record_pub_rec(st->glue->txn,
				&st->reserve_rec);
			/*
			 * Order-statistics fold (BULK): the reserve relocated the
			 * attach parent to the fresh @st->dest, recompacted with
			 * only the OLD subtree count (the payload slot read
			 * not-present during the reserve).  Bake +count_delta into
			 * that fresh copy (a build-invisible plain store, off the
			 * walk -- the insert-I7 reader-safety invariant: a node's
			 * own nr_keys is only read root-descended, landing on the
			 * old node via the parked grandparent proxy) and walk the
			 * +count_delta up from the STABLE grandparent this
			 * relocation republishes into.
			 */
			if (count_delta) {
				struct cds_ft_metadata *dm =
					cds_ft_item_to_metadata(ft_node_ptr(st->dest));
				ft_nr_keys_store(ft, dm,
					ft_nr_keys_get(dm) + count_delta, CMM_RELAXED);
				count_base = pub_parent;
			}
		} else {
			/*
			 * In-place reserve: a redundant same-value republish.  Resolve
			 * the grandparent slot COHERENTLY from st->dest's own (parent,
			 * offset) rather than the stale descent st->pnfp -- same
			 * coherence rule as the relocation arm above (a peer may have
			 * relocated the grandparent since this graft's descent).
			 */
			struct cds_ft_metadata *dest_meta =
				cds_ft_item_to_metadata(ft_node_ptr(st->dest));
			struct cds_ft_inode_flag *pub_parent;
			struct cds_ft_inode_flag **pub_slot =
				ft_resolve_parent_slot(dest_meta, ft, &pub_parent);

			ft_publish_to_parent(ft, pub_parent, pub_slot, st->dest);
			/*
			 * The in-place reserve owes @st->dest its nr_child++:
			 * record it as an edge in glue->txn so the count goes
			 * live with the slot edge below and is discarded with an
			 * aborted attempt -- reserve-then-ABORT must be
			 * idempotent, or a re-descend reserves the byte a second
			 * time and double-counts it (see
			 * ft_flip_txn_record_nr_child_inc).  Reservation is
			 * net-zero: this arm records no §4.B parent guard (only
			 * the relocation arm above does), so it consumes that
			 * arm's already-reserved guard record.
			 */
			if (st->count_deferred)
				ft_flip_txn_record_nr_child_inc(st->glue->txn,
					dest_meta);
			/*
			 * Order-statistics fold (BULK): @st->dest (== d->pnf, a
			 * stable existing node) gains +count_delta; walk from it up.
			 */
			if (count_delta)
				count_base = st->dest;
		}
		/*
		 * Record the slot edge against the FINAL slot (now that the
		 * recompact-relocated address is known): NULL -> slot_value, so
		 * the reserved bit-set+NULL byte settles to the grafted child at
		 * commit.  Then fuse the ordered-list run-splice into the SAME
		 * glue->txn: record the run's <=4 boundary edges so one commit
		 * makes the grafted key appear in the structure and the ordered
		 * list atomically.  The reserve set_nth was the last fallible
		 * step in prepare; the slot + cell edges have none, and all draw
		 * from the reserved txn (no allocation here).
		 */
		/*
		 * ☠ WHETHER THIS SLOT IS RECORDED AT ALL IS AN ARM QUESTION,
		 * because the two arms write two different KINDS of word.
		 *
		 * RELOCATION: @st->dest is the fresh copy the reserve's recompact
		 * produced -- build-invisible, reachable by nothing until the
		 * grandparent forward edge above publishes it.  A slot in its
		 * INTERIOR is not a reachability edge, so it takes a PLAIN STORE
		 * like every other write into that private body (the copy loops,
		 * ft_set_parent at the resolve above, the nr_keys bake below).
		 * Recording it buys no atomicity: the engine flips a SET of slots,
		 * and the set that matters -- the grandparent edge plus the cell
		 * run-splice -- is already this one commit, so a reader sees
		 * nothing and then a fully wired @st->dest either way.  This is
		 * ft_glue_apply_deferred's per-edge rule ("unreachable until the
		 * forward flip is what licenses the plain store"), which this arm
		 * was the one outlier from.
		 *
		 * IN-PLACE: @st->dest is LIVE, so its slot IS reader-visible now
		 * and the write must ride the flip -- that is what makes the
		 * grafted key appear atomically with the ordered-list splice.
		 * @slot was resolved out of @st->dest itself, so the node that
		 * owns it is @st->dest, the same node whose nr_child this arm
		 * records.
		 */
		if (st->old_recompacted_node)
			*slot = st->slot_value;
		else
			ft_flip_txn_record_reserved(st->glue->txn,
				/*owner=*/ ft_flag_to_metadata(ft, st->dest),
				(void **) slot, NULL, (void *) st->slot_value);
		if (run) {
			rn = ft_ord_cell_run_splice_edges(ft, run->run_first,
				run->run_last, run->pred, run->succ, redges, 0);
			/*
			 * Through the tag-dispatching recorder, NOT a raw
			 * ft_flip_txn_record_tag loop: a CELL edge must be MW
			 * whatever this txn's structural_sw mode is (the ordered
			 * list is lock-free -- no cell carries a node lock to
			 * park an SW store under), and a raw record_tag keys off
			 * structural_sw alone, so it would silently demote these
			 * to an unvalidated SW park under a caller that opted in.
			 */
			ft_ord_cell_record_into_ft(ft, st->glue->txn, redges, rn);
		}
		/*
		 * Freeze-on-free (doc §4.B): a reserve recompaction relocated the
		 * dst attach node; its old copy @old_recompacted_node, resolved-to
		 * via the parked grandparent proxy until this commit, gets its
		 * tombstone recorded INTO glue->txn (reserved +1 above), so the mark
		 * and the commit that unlinks it flip atomically (atomic detach).
		 * (Not surfaced by ft_unit; the recompact-relocation graft shape is
		 * exercised by ft_inv.)
		 *
		 * LOCK_FINE (§9.3): under the FT-wide-lock drop the reserve's
		 * ft_node_recompact ALREADY records @old_recompacted_node's retire
		 * as its C-half {LOCK|s -> TOMBSTONE|s} terminal, registered on
		 * THIS glue->txn (ft-mutation-node.h).  That IS the atomic-detach
		 * tombstone; a second plain ft_flip_txn_record_tombstone here would
		 * DOUBLE-record the same word with a conflicting expected-old (plain
		 * current-state vs the mark snapshot) -- the double-tombstone poison
		 * (lost key + double free).  Insert relies solely on the recompact's
		 * retire (ft-insert.h, free_old_node, no commit-time tombstone); the
		 * graft matches it under lock_fine and keeps the standalone tombstone
		 * only for the non-fenced (non-lock_fine) recompact.
		 */
		if (st->old_recompacted_node && !ft->lock_fine)
			ft_flip_txn_record_tombstone(st->glue->txn,
				cds_ft_item_to_metadata(st->old_recompacted_node));
		if (count_delta)
			ft_flip_txn_record_count_parent(ft, st->glue->txn,
				count_base, count_delta);
		if (st->glue->record_only) {
			/*
			 * FOLD (coherent rekey one-decide writer): the whole dst-attach
			 * (slot NULL -> S_top', the reserve recompaction's re-parents +
			 * fenced retire, count) is now recorded into the caller's SHARED
			 * txn; the caller runs the ONE commit that also carries the
			 * src-unlink + S_top COW.  A reserve recompaction relocated the dst
			 * attach node -- its OLD copy stays LIVE (resolved through the
			 * parked grandparent proxy) until the CALLER's commit, so its free
			 * is DEFERRED to the caller: hand it out via @st->old_recompacted_
			 * node (which the caller reads off its own @st).  The caller owns
			 * the cells (run NULL here, list off).  Leave st->glue->txn intact
			 * and report the recorded outputs.
			 */
			assert(!run);
			*attached_nf = st->attached;
			*attached_depth = st->attached_depth;
			return URCU_TXN_STATUS_OK;
		}
		cst = ft_flip_txn_commit(ft, st->glue->txn);
		st->glue->txn = NULL;
		/*
		 * ARM ONLY ON A COMMITTED FLIP.  The run-splice edges rode this
		 * txn, so an ABORT rolled them back with everything else: the run
		 * is NOT in @dst's ordered list.  @armed is what makes the caller
		 * skip its standalone ft_ord_cell_run_splice fallback, so arming
		 * unconditionally would strand the run OUT of the list -- keys
		 * present in the structure, invisible to ordered iteration (and a
		 * cds_ft_verify ord-cell mismatch).  The same gate the retire /
		 * glue_free_old reclaims below already use.
		 */
		if (run && cst == URCU_TXN_STATUS_OK)
			run->armed = true;
		/*
		 * Only the writer whose commit actually retired
		 * @old_recompacted_node ({LOCK|s -> TOMBSTONE|s}) may free
		 * it.  An aborted commit rolled the retire back (the registered
		 * fence was cleared, the node stays LIVE); freeing it here would
		 * double-free the still-live node against the peer that
		 * legitimately retires it next.  The caller re-descends and
		 * re-commits on abort (ft_graft_keylen post-swap store retry).
		 */
		if (st->old_recompacted_node && cst == URCU_TXN_STATUS_OK)
			free_cds_ft_node(ft, st->old_recompacted_node);
	}
	/*
	 * @st->glue's OLD nodes (the pre-recompact copies gathered on its
	 * free list) are equally only reclaimable when the commit committed:
	 * an aborted commit left them LIVE.  Gate on @cst.  Under record_only
	 * (the displaced branch above returns here without committing) the OLD
	 * nodes stay LIVE until the CALLER's commit, so defer their free to the
	 * caller's post-commit finalize -- freeing here would reclaim a
	 * still-reachable node.
	 */
	if (!st->glue->record_only && cst == URCU_TXN_STATUS_OK)
		ft_glue_free_old(ft, st->glue);
	*attached_nf = st->attached;
	*attached_depth = st->attached_depth;
	return cst;
}

/*
 * Combined NOSPLIT store: prepare + commit back-to-back -- cds_ft_graft's call
 * site, run after the source-root unlink + drain.  The slot proxy and the
 * ordered-list run-splice both ride @glue->txn (which the caller created /
 * took and reserved before its last fallible step, so this post-drain store
 * has no fallible allocation left -- node allocations draw from the reserve,
 * flip latches from the reserved txn).  @run, when non-NULL, fuses an
 * ordered-list run-splice into the structural publish flip (both store shapes:
 * the in-place slot proxy and the displaced-external forward publish).
 */
static
enum cds_ft_status ft_store_at_graft_point(struct cds_ft *ft,
		const uint8_t *key, size_t key_len,
		struct ft_descent *d,
		struct cds_ft_inode_flag *graft_payload,
		unsigned long graft_external_count,
		struct cds_ft_inode_flag **attached_nf,
		unsigned int *attached_depth,
		struct ft_glue *glue,
			struct ft_graft_run *run,
		long count_delta)
{
	struct ft_graft_store_state st;
	enum cds_ft_status status;
	enum urcu_txn_status cst;

	status = ft_store_at_graft_point_prepare(ft, key, key_len, d,
			graft_payload, graft_external_count, glue,
			/*outer*/ NULL, &st);
	if (status != CDS_FT_STATUS_OK) {
		/*
		 * MW LOCK_FINE drop: prepare can FAIL (the recompact of the
		 * contended spine could not lock/reserve -- -EAGAIN / MEMORY_ERROR),
		 * unlike the FT-wide-lock "cannot fail".  Nothing is committed and
		 * @glue->txn is still alive; free the invisible build + txn so the
		 * caller (cds_ft_merge_at's post-unlink retry) only needs to
		 * re-descend.  (cds_ft_graft drives prepare / commit separately and
		 * does its own cleanup; only cds_ft_merge_at calls this combined
		 * wrapper.)
		 */
		ft_glue_abort(ft, glue);
		if (glue->txn)
			ft_flip_txn_destroy(glue->txn);
		return status;
	}
	cst = ft_store_at_graft_point_commit(ft, attached_nf, attached_depth,
			run, &st, count_delta);
	if (cst != URCU_TXN_STATUS_OK) {
		/*
		 * Drop: the store commit's MCAS footprint conflicted with a peer and
		 * ABORTED (@glue->txn consumed by the commit; the retire rolled back,
		 * the OLD nodes stay LIVE -- reclaimed by whoever finally commits).
		 * Free this attempt's UNPUBLISHED new nodes: the glue cluster (every
		 * fresh node ft_glue_track'd) plus -- if the reserve RELOCATED the
		 * attach node -- the unpublished relocated copy @st.dest (allocated
		 * directly by ft_node_recompact, NOT glue-tracked; an in-place
		 * recompact leaves @st.dest == the LIVE d->pnf, do NOT free).  Signal
		 * the caller to re-descend + retry.
		 */
		ft_glue_abort(ft, glue);
		if (st.old_recompacted_node)
			free_cds_ft_node_unpublished(ft, ft_node_ptr(st.dest));
		/*
		 * Keep the engine's two outcome spaces apart instead of collapsing
		 * both into "out of memory": URCU_TXN_STATUS_ABORT is contention --
		 * "NOT an error", as rcu-txn-status.h puts it -- and nothing was
		 * published, so it is BUSY; only a genuine
		 * URCU_TXN_STATUS_MEMORY_ERROR is an allocation failure.  Both are
		 * transient here and both callers already retry on either, so this
		 * changes only what the wrapper REPORTS -- but a conflict surfacing
		 * as ENOMEM is exactly the kind of misreport that sends a future
		 * reader hunting an allocator bug that never existed.
		 */
		return cst == URCU_TXN_STATUS_ABORT ?
			CDS_FT_STATUS_BUSY_ERROR :
			CDS_FT_STATUS_MEMORY_ERROR;
	}
	return CDS_FT_STATUS_OK;
}

/*
 * Outcome of ft_graft_build's build-invisible prep descent.
 */
enum ft_graft_prep {
	FT_GRAFT_PREP_GLUE,	/* diverge: full attach cluster built into @glue */
	FT_GRAFT_PREP_NOSPLIT,	/* graft point located in @d; legacy attach */
	FT_GRAFT_PREP_POPULATED,/* graft point occupied; tries pristine */
	FT_GRAFT_PREP_OOM,	/* allocation failed; caller runs glue_abort */
	FT_GRAFT_PREP_RETRY,	/* split-retire cn fence miss; re-descend (nothing
				 * built, cn NOT marked -- clean re-descend) */
};

/*
 * Build-invisible prep for cds_ft_graft.  Descends dst to the graft point
 * for @key.  When the key diverges inside a compressed node, builds the
 * COMPLETE attach cluster (split rearrangement + the @payload subtrie)
 * into @glue without publishing or freeing anything -- dst and the source
 * stay pristine, so an OOM frees the glue with nothing to roll back
 * (FT_GRAFT_PREP_GLUE / _OOM).  Otherwise it just locates the graft point
 * in @d (FT_GRAFT_PREP_NOSPLIT -- graft_keylen completes via the legacy
 * post-sync ft_store_at_graft_point) or reports an occupied point
 * (FT_GRAFT_PREP_POPULATED: the key ends inside an existing compressed
 * path).
 *
 * Read-only on dst for the NOSPLIT / POPULATED outcomes.
 */
static
enum ft_graft_prep ft_graft_build(struct cds_ft *ft,
		const uint8_t *key, size_t key_len,
		struct cds_ft_inode_flag *payload, unsigned long src_count,
		struct ft_descent *d, struct ft_glue *glue,
		const struct ft_held_set *outer)
{
	const uint8_t *ik = key;

	ft_descent_init(d, ft);
	/*
	 * Hand @glue the anchor source as the descent that will supply it is
	 * created: the glue's own acquires -- its publish parent above all --
	 * fire from commit helpers that never see @d, and an acquire with no
	 * depth under a coarse spacing MISSES, which aborts a commit the
	 * unfailable arms cannot retry.  That is a livelock, not a failure.
	 * ft_store_at_graft_point_prepare repeats it for the callers that reach
	 * it without building.
	 */
	glue->lock_d = d;
	for (; d->depth < key_len; ) {
		if (ft_node_external(d->nf))
			break;
		d->nf = ft_resolve_skip_compressed(ft, d->nf);
		if (ft_node_compressed(d->nf)) {
			struct cds_ft_compressed_node *cn =
				ft_compressed_node_ptr(d->nf);
			int remaining = (int) (key_len - d->depth);
			int cmp = cn->len < remaining ? cn->len : remaining;
			int j = ft_match_compressed_key(ik, cn, cmp);

			if (j == cmp && cn->len <= remaining) {
				ft_descent_traverse_compressed(ft, d, cn, &ik);
				continue;
			}
			if (j < cmp) {
				int bret = ft_split_compressed_graft_build(ft,
					d, key, key_len, j, payload, src_count,
					glue, outer);

				if (bret == -EAGAIN)
					return FT_GRAFT_PREP_RETRY;
				if (bret)
					return FT_GRAFT_PREP_OOM;
				return FT_GRAFT_PREP_GLUE;
			}
			/*
			 * Key shorter than the compressed path: the graft
			 * point lies inside an existing compressed (occupied).
			 */
			return FT_GRAFT_PREP_POPULATED;
		}
		ft_descent_step(ft, d, *(ik++));
	}
	return FT_GRAFT_PREP_NOSPLIT;
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
 * structural body are performed here, so this helper is the single
 * source of truth for what graft actually does.
 *
 * Transaction shape (non-root): build the dst-side attach invisibly
 * (ft_graft_build), then the failure-free commit -- unlink the source
 * root, synchronize, apply the deferred live back-pointers, publish the
 * cluster, reclaim the old nodes.  A diverge split is fully build-
 * invisible (no rollback); the non-split attach still uses the legacy
 * post-sync store with a clean rollback.
 */
static
enum cds_ft_status ft_graft_keylen(struct cds_ft *dst_ft,
		const uint8_t *_key, size_t key_len,
		struct cds_ft *src_ft,
		struct ft_flip_txn **pre_txn)
{
	/*
	 * FUNCTION SCOPE on purpose: the key_len == 0 root-attach path below
	 * returns before the retry loop that used to own this handle, and its
	 * root fence takes the dst ROOT's lock -- the most contended word under
	 * root-only spacing.  Scoping the handle to the retry loop left that
	 * acquire with nothing to age.
	 */
	struct urcu_txn optxn;
	struct cds_ft_metadata *src_rmeta;
	size_t src_max;

	/*
	 * MW LOCK_FINE (step 6, §9.5): a cross-trie graft CONSUMES the whole
	 * source, so the source must be EXCLUSIVE -- no concurrent readers or
	 * writers.  An exclusive src skips its FT-wide lock
	 * (ft_writer_lock_scope_enter), so only dst's lock is ever taken: one
	 * lock, no cross-trie deadlock.  Two LIVE lock-mode tries would instead
	 * need BOTH FT-wide locks with no lock order (thread 1 grafts a->b while
	 * thread 2 grafts b->a -> circular wait), so a live (lock-mode,
	 * non-exclusive) source is REJECTED with BUSY before anything is touched
	 * -- both tries are byte-for-byte unchanged.  The caller makes the source
	 * exclusive first (cds_ft_make_exclusive), which is the zone-graft "build
	 * private, graft" pattern.  With an exclusive src the fused body below
	 * runs directly and is build-invisible, so a rejected graft (POPULATED /
	 * OVERFLOW / OOM) leaves the source PRISTINE -- no residual sink needed.
	 *
	 * Gated on the SOURCE alone: dst may be a LIVE concurrent trie (the
	 * supported case).  A SAME-trie op (src == dst, reachable via
	 * cds_ft_merge_at's whole-source rekey) is excluded -- it takes one lock
	 * reentrantly, like the crosstrie guard's a != b test.  Inert outside
	 * lock-mode -- optimistic groups have lock_mode == false and never take
	 * FT-wide locks for a cross-trie graft.
	 */
	if (src_ft != dst_ft && !src_ft->exclusive)
		return CDS_FT_STATUS_BUSY_ERROR;

	ft_crosstrie_lock_mode_guard(dst_ft, src_ft);
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
	if (ft_meta_nr_child(src_rmeta) == 0 && !src_rmeta->external_nodes)
		return CDS_FT_STATUS_OK;

	if (key_len == 0) {
		struct cds_ft_metadata *dst_rmeta;
		struct cds_ft_inode_flag *dst_root_fenced;
		uintptr_t dst_root_snap;
		struct cds_ft_inode *fresh_root;
		struct cds_ft_metadata *fresh_meta;
		struct cds_ft_inode *old_dst_root;
		int fence_ret;

		/*
		 * Destination must be empty for a root-level graft -- decided
		 * UNDER the old root's node lock, which the swap below then
		 * consumes as its fenced retire.  Testing emptiness unfenced and
		 * swapping later let a contract-legal peer attach land in the
		 * window and be freed with the old root.
		 */
		ft_txn_op_init(dst_ft, &optxn);
		fence_ret = ft_root_attach_fence_empty(dst_ft, &dst_root_fenced,
			&dst_rmeta, &dst_root_snap, &optxn);
		if (fence_ret == -EEXIST)
			return CDS_FT_STATUS_POPULATED_ERROR;
		if (fence_ret)
			return CDS_FT_STATUS_BUSY_ERROR;

		/*
		 * Allocate a fresh empty root for the source before
		 * swapping, so the source remains a valid trie.
		 */
		fresh_root = alloc_cds_ft_node(dst_ft, &ft_types[0], &fresh_meta);
		if (!fresh_root) {
			ft_meta_lock_release(dst_rmeta);
			return CDS_FT_STATUS_MEMORY_ERROR;
		}
		/* Fresh root for @dst_ft: name its owner while still invisible. */
		fresh_meta->parent_word = ft_trie_parent(dst_ft);

		/*
		 * The cross-trie dual root-swap txn.  Take the caller's PRE-RESERVED
		 * @pre_txn when one is supplied (cds_ft_merge_at's rekey pre-reserves a
		 * glue txn before its own fallible steps and passes it down so this
		 * forward root swap is node-infallible) -- else create+reserve one here
		 * while both tries are still pristine (the swap is the op's sole
		 * reader-visible change, so this is the abort boundary).
		 * The dual always flips both roots, so it is always multi-edge even
		 * list-off -- it cannot reduce to a lone store.  +1 edge for the dst
		 * old-root freeze-on-free tombstone fused into the same swap (atomic
		 * detach, §4.B).  A create OOM aborts cleanly (free the fresh root, both
		 * tries untouched); a taken txn cannot fail.
		 */
		struct ft_flip_txn *dual_txn = ft_flip_txn_take(pre_txn);

		if (!dual_txn) {
			/*
			 * NAMED FOR @dst_ft, WRITES BOTH ROOTS: the dual flips
			 * &dst_ft->root AND &src_ft->root in one commit.  The
			 * trie named here decides only ARMING
			 * (ft_txn_content_sw_ok); both roots record MW by
			 * construction (ft_root_list_swap_publish_dual marks
			 * them), so the named trie need not stand for the
			 * foreign one.
			 */
			dual_txn = ft_flip_txn_create_bounded(dst_ft,
				FT_ROOT_LIST_SWAP_DUAL_MAX_EDGES + 1);
			if (!dual_txn) {
				ft_meta_lock_release(dst_rmeta);
				free_cds_ft_node_unpublished(dst_ft, fresh_root);
				return CDS_FT_STATUS_MEMORY_ERROR;
			}
		}

		/*
		 * Root-level graft: the source's root becomes the
		 * destination's root with no parent-pointer change
		 * (both are root positions with parent == NULL).  No
		 * "jump out" window, so the STRUCTURAL swap needs no
		 * internal synchronize_rcu.  (The ordered-list run move
		 * below adds its own post-flip drain when @src_ft is
		 * concurrent -- see the finalize after the publish.)
		 *
		 * Swap root pointers.  The source's root carries all
		 * metadata (nr_child, external_nodes) with it.  The
		 * destination's old (empty) root is orphaned by the
		 * swap and must be reclaimed via call_rcu so concurrent
		 * readers that entered before the swap finish their
		 * descent first.
		 */
		/*
		 * The FENCED root -- not a fresh read of @dst_ft->root.  Under the
		 * fence the two are equal by construction, and taking the fenced
		 * value is what keeps the retire, the swap's expected-old and the
		 * free all naming the SAME node.
		 */
		old_dst_root = ft_node_ptr(dst_root_fenced);
		/*
		 * Ordered list: dst was empty (checked above), so src's WHOLE
		 * ordered list becomes dst's.  Cells' internal links are
		 * unchanged; the head/tail endpoints transfer and the moved run's
		 * outer links are NULL-terminated (the dual does not relink them
		 * to a foreign sentinel -- there is no drain between detach and
		 * attach here, so a src straddler must see a universal end, not
		 * dst's sentinel).  Fuse BOTH sides into ONE cross-trie flip (dst
		 * and src share a group, so a single epoch flip settles every
		 * root/endpoint proxy at once):
		 *  - dst (appear): src->root AND src's head/tail (dst's were NULL);
		 *  - src (disappear): retire src->root to a fresh empty root AND
		 *    clear src's head/tail.
		 * A reader resolves all slots to ONE phase, so it never sees the
		 * key reachable in BOTH tries (appear done, disappear pending) or
		 * in NEITHER.  Capture src's live root and endpoints first -- the
		 * flip overwrites them.  After the flip, drain src (if concurrent)
		 * and finalize dst's adopted run to dst's sentinel (below).  (List
		 * off: head/tail are unused; the swap reduces to the two lone root
		 * edges, MCAS-expressible like the list-on path.  Both are root
		 * positions with parent == NULL, so no internal synchronize_rcu is
		 * required.)
		 */
		{
			struct cds_ft_inode_flag *dst_old = dst_root_fenced;
			struct cds_ft_inode_flag *src_root = src_ft->root;
			struct ft_ord_cell *src_head = ft_ord_first(src_ft);
			struct ft_ord_cell *src_tail = ft_ord_last(src_ft);
			struct ft_root_swap_side appear = {
				.ft = dst_ft, .slot = &dst_ft->root,
				.old_root = dst_old, .new_root = src_root,
				.head_old = NULL, .head_new = src_head,
				.tail_old = NULL, .tail_new = src_tail,
			};
			struct ft_root_swap_side disappear = {
				.ft = src_ft, .slot = &src_ft->root,
				.old_root = src_root,
				.new_root = ft_node_flag(fresh_root, 0),
				.head_old = src_head, .head_new = NULL,
				.tail_old = src_tail, .tail_new = NULL,
			};

			/*
			 * Freeze-on-free (doc §4.B): the dst old root this swap
			 * retires gets its one-way tombstone recorded INTO dual_txn,
			 * so the mark and the root-swap unlink flip atomically (atomic
			 * detach).  src_root MOVES to dst, fresh_root is the new src
			 * root -- neither is freed here; only @old_dst_root == @dst_old.
			 *
			 * FENCED form: the expected-old is the mark's clean snapshot
			 * with the fence bit, so a peer state change that slipped under
			 * the fence ABORTS this commit instead of being ratified by a
			 * late re-read.  Registering hands the fence to the txn -- the
			 * commit consumes it in the {LOCK|s -> TOMBSTONE|s}
			 * transition, an abort CAS-clears it back to LIVE -- so no bail
			 * path past this point owes a clear.
			 */
			ft_flip_txn_lock_register(dual_txn, dst_rmeta,
				dst_root_snap);
			ft_flip_txn_record_tombstone_locked(dual_txn, dst_rmeta,
				dst_root_snap);
			ft_root_list_swap_publish_dual(dual_txn, &appear,
				&disappear);
		}
		FT_TP(root_publish, (const void *) dst_ft,
			(const void *) dst_ft->root);
		FT_TP(root_publish, (const void *) src_ft,
			(const void *) src_ft->root);
		/*
		 * Ordered-list run finalize: the dual NULL-terminated src's moved
		 * run (universal end).  Drain src readers straddling that run, then
		 * repoint its outer links at dst's sentinel so dst's list is circular
		 * again (the remove folding / reverse walk need first->prev ==
		 * sentinel).  src exclusive -> no straddler, no drain; the finalize
		 * still runs (dst is live: rcu_assign_pointer, benign race with dst's
		 * own readers, for whom both NULL and dst's sentinel mark the end).
		 */
		if (dst_ft->group->ordered_list_set) {
			if (!src_ft->exclusive)
				ft_writer_lock_gp_wait(src_ft);
			ft_ord_finalize_circular(dst_ft);
		}
		free_cds_ft_node(dst_ft, old_dst_root);
		goto done;
	}

	{
		struct ft_descent d;
		struct cds_ft_inode *fresh_node;
		struct cds_ft_metadata *fresh_meta;
		struct ft_glue glue;
		enum ft_graft_prep prep;
		unsigned long src_count = ft_nr_keys_get(src_rmeta);
		struct cds_ft_inode_flag *old_src_root;
		struct cds_ft_inode_flag *attached_nf = NULL;
		struct ft_ord_cell *graft_run_first = NULL, *graft_run_last = NULL;
		struct ft_ord_cell *graft_pred = NULL, *graft_succ = NULL;
		struct ft_graft_run graft_run;
		struct ft_graft_run *run_arg = NULL;
		/*
		 * NOSPLIT store split across the src-root swap (§11 drop): PREPARE
		 * (the fallible recompact of the graft-point node) runs BEFORE the
		 * swap so a concurrent {p} relocation / stale descent is a clean
		 * -EAGAIN re-descend with src untouched; @st carries its result to
		 * the unfailable COMMIT after the swap.
		 */
		struct ft_graft_store_state st;
		bool nosplit_prepared = false;
		/*
		 * Post-swap store commit status (§11 drop abort-safety): under the
		 * drop the attach commit CAN abort (its MCAS footprint conflicts with
		 * a peer beyond the pre-swap fence's single fenced word).  Captured
		 * from both commit arms so the OLD-node reclaim is gated on a real
		 * retire and the store is re-attempted on abort.
		 */
		enum urcu_txn_status store_cst = URCU_TXN_STATUS_OK;
		/*
		 * Post-swap store RETRY (§11 drop, doc ft-graft-abort-safe-poststore-
		 * retry): once the source-root swap has committed (below), the payload
		 * is detached from the EXCLUSIVE (reader-free) src and OWNED by this
		 * writer -- src stays empty across retries with no reader-visible
		 * flicker.  So the whole dst-side attach (re-descend -> build -> prepare
		 * -> commit) is a self-contained, retryable operation: on a commit ABORT
		 * we re-run ONLY the dst-side steps via @retry_attach and never touch
		 * src again.  @already_swapped guards the one-shot src-side steps
		 * (fresh-root alloc, the swap, the src-retire txn) so a retry skips them;
		 * it also converts the pre-swap OOM exits (which would otherwise return
		 * MEMORY_ERROR and orphan the payload) into re-attempts, since src is
		 * already consumed.
		 */
		bool already_swapped = false;
		/*
		 * MW LOCK_FINE drop, FUSED cross-trie move (doc follow-up b): for an
		 * EXCLUSIVE src (no readers -> no jump-out drain), the src-root retire
		 * is recorded INTO @glue.txn instead of a separate pre-commit flip, so
		 * the src unlink AND the dst attach commit as ONE atomic flip -- no
		 * orphan window, and a commit ABORT rolls BOTH back (src still full),
		 * making the retry a clean whole-op re-descend (identical to a pre-swap
		 * failure) with no owned-payload bookkeeping.  @already_swapped is left
		 * FALSE until that fused commit SUCCEEDS (set post-commit), so the
		 * existing @already_swapped-guarded cleanup handles the abort.  A
		 * non-exclusive src still needs the unlink -> drain -> attach ordering,
		 * so it keeps the separate swap (@already_swapped set at the swap).
		 */
		bool src_swap_fused = false;
		/*
		 * Self-secured NOSPLIT attach: when no caller reserve is active, this
		 * graft reserves its own commit nodes before publishing the empty
		 * source root, so the post-publish store cannot fail and needs no
		 * reader-observable source-root rollback.  Skipped under a caller
		 * reserve -- the rekey, whose reserve + pre-reserved @glue.txn already
		 * make the store unfailable.  (The store's flip latches all draw from
		 * @glue.txn, reserved below, so no separate flip batch is needed.)
		 */
		struct cds_ft_alloc_reserve graft_reserve;
		bool self_secured = false;
		/*
		 * The src-root retire below (ft_root_list_swap_publish, list on)
		 * runs in the failure-free section AFTER every fallible step has
		 * returned -- it cannot abort -- so PRE-RESERVE its bounded txn
		 * here, co-located with @glue.txn in the fallible prefix.  Reserved
		 * only when the ordered list is on (list off retires via the
		 * lone-edge ft_root_edge_flip).  Freed at every fallible exit below;
		 * consumed by the retire.
		 */
		struct ft_flip_txn *src_retire_txn = NULL;
		/*
		 * Standalone run-splice fallback txn.  Every attach shape FUSES the
		 * run-splice into its structural flip (graft_run.armed), so this
		 * fallback is not taken today -- but it is kept defensively, runs in
		 * the failure-free section (post src-unlink + drain), and is hence
		 * un-abortable, so PRE-RESERVE its bounded txn here.  Reserved only
		 * when the ordered list is on; consumed by the standalone splice or
		 * freed-unused when the run was fused (the common case).
		 */
		struct ft_flip_txn *run_splice_txn = NULL;
		/*
		 * NIL-key-only source: the whole source is a single prefix key,
		 * stored as the root's external_nodes (a childless internal -- valid
		 * only AT a root).  Grafting that wrapper internal to a non-root
		 * position would leave a non-canonical childless internal there
		 * (cds_ft_remove_all's invariant).  Graft the external chain head
		 * DIRECTLY instead, so the placed node is a plain external, and free
		 * the orphaned wrapper on success.  (Cross-trie graft never hits this:
		 * a real source root always has children.)
		 */
		bool nil_key_root = (ft_meta_nr_child(src_rmeta) == 0
				&& src_rmeta->external_nodes != NULL);
		struct cds_ft_inode_flag *graft_payload = nil_key_root ?
			(struct cds_ft_inode_flag *) ft_dereference_external(
				src_rmeta->external_nodes) : src_ft->root;

		/*
		 * MW LOCK_FINE (§11 drop-mechanics): re-entry point for the
		 * contention retry.  Under the FT-wide-lock drop a concurrent
		 * writer can RELOCATE the shared graft-point-parent {p} between
		 * this graft's reserve-fill and its store, staling the reserve
		 * manifest -> the store fails (MEMORY_ERROR).  The store is
		 * build-invisible (both tries untouched, graft.h:436) and the
		 * consumed source is EXCLUSIVE (step 6, no readers), so on that
		 * failure we roll the src-root swap back -- reader-invisible --
		 * and retry the whole attach on a fresh descent.  @graft_payload
		 * and @nil_key_root are attach-invariant (computed above); every
		 * per-attempt resource is (re)acquired below and freed on the
		 * retry path.
		 *
		 * Progress: {p}'s node lock guarantees a winner each contention
		 * round -- a SYSTEM claim, not a per-op one.  A winner every round
		 * is fully compatible with one particular op losing every round.
		 * Per-op progress is a different property, and @optxn supplies it.
		 */
		/*
		 * The op's PERSISTENT engine handle (doc §11), spanning the whole
		 * @retry_attach loop as ft_txn_op_init does for insert, remove and
		 * replace.  It is what makes this loop terminate: urcu_txn_conflict()
		 * ages contention ACROSS attempts, so the domain escalates this
		 * writer into its per-trie FIFO fair-mutex lane and the contention
		 * drains.  Unaged, every attempt restarts at retry 0 and the loop
		 * has no termination argument at all -- 679819 retries over one inv
		 * run, 4214 of them for a single op.  That is STARVATION, not
		 * livelock: the system still progresses, which is exactly why a
		 * green suite is not evidence about it.
		 *
		 * @ra_txn is the bracket's condition, evaluated ONCE.  A bracket may
		 * only span a body that takes NO grace period, for two independent
		 * reasons: urcu_txn_begin() enters the RCU read side (a GP under it
		 * waits on this very thread), and an aged handle escalates into the
		 * fallback lane (ft_writer_lock_gp_wait asserts
		 * !urcu_txn_in_fallback(), a peer parked on that lane being an
		 * ONLINE, non-quiescent reader that holds the GP open).  The only
		 * grace period reachable from this body is its own src drain, which
		 * is !src_ft->exclusive-gated, and dst_ft->lock_fine is what binds
		 * the escalation domain at all -- so the conjunction IS the GP-free
		 * contract, and it is the same condition ft_merge_at_inner already
		 * read_lock()s this call under.  It covers every retry the loop
		 * takes: excl=679819 live=0, against 225 ops entering off-contract
		 * in the same run, so the split is not vacuous.
		 */
		const bool ra_txn = dst_ft->lock_fine && src_ft->exclusive;
		unsigned long ra_depth __attribute__((unused)) = 0;

		ft_txn_op_init(dst_ft, &optxn);

retry_attach:
		/*
		 * Open the attempt BEFORE anything it must undo: an aged retry
		 * escalates here, and escalation blocks on the domain's fair mutex.
		 */
		if (ra_txn)
			urcu_txn_begin(&optxn);
		RSPIN_ENTER_X(2, ra_depth, 1, ra_txn);
		/*
		 * Preallocate a fresh empty root for the source trie
		 * before the point of no return, so we can fail cleanly
		 * on memory shortage instead of calling abort().  One-shot:
		 * a post-swap retry keeps src's already-published fresh root
		 * (@already_swapped), so @fresh_node stays NULL and every
		 * src-side free below is skipped.
		 */
		if (!already_swapped) {
			fresh_node = alloc_cds_ft_node(src_ft, &ft_types[0], &fresh_meta);
			if (!fresh_node) {
				ft_txn_attempt_end(&optxn, ra_txn);
				return CDS_FT_STATUS_MEMORY_ERROR;
			}
			/* Fresh root for @src_ft: named while still invisible. */
			fresh_meta->parent_word = ft_trie_parent(src_ft);
		}

		/*
		 * PREP (dst + source pristine): build the dst-side attach.
		 * A diverge split builds its whole cluster invisibly into
		 * @glue; otherwise just locate the graft point in @d.
		 */
		ft_glue_init(&glue);
		glue.op = &optxn;
		/*
		 * Enable the split-retire @cn fence for this graft: ft_graft_keylen's
		 * retry_attach loop handles a fence-miss (FT_GRAFT_PREP_RETRY) as a
		 * clean re-descend.  ft_glue_init reset it, so set it each attempt.
		 */
		glue.fence_split_cn = true;
		/*
		 * Every attach shape -- GLUE diverge, displaced-external, and the
		 * in-place NOSPLIT slot store -- commits through @glue.txn, so its
		 * live re-parents + forward publish + ordered-list run-splice flip
		 * atomically with no deferred-edge ordering window.  Take the
		 * caller's pre-reserved txn (the rekey, pre-sized before its detach so
		 * the post-drain commit cannot fail) or create one here, reserved to
		 * the floor-bounded cluster size up front (records then can't fail
		 * mid-build, like the glue floor arrays); the + 7 headroom covers the
		 * forward edge's 1-2 stores plus the <=4 run-splice cell edges (also
		 * the in-place slot proxy + its run edges, FT_GRAFT_RUN_FLIP_CAP), plus
		 * the +1 recompact-relocate retire tombstone fused into the commit
		 * (atomic detach, §4.B; st->old_recompacted_node below).
		 * Created before the build only so its lifecycle is co-located here;
		 * the build records nothing into it -- the commit replays g->deferred
		 * (and the store reserves its slot proxy) through it post-drain.
		 */
		glue.txn = ft_flip_txn_take(pre_txn);
		if (!glue.txn) {
			glue.txn = ft_flip_txn_create(dst_ft);
			if (!glue.txn || !ft_flip_txn_reserve(glue.txn,
					/* + FLOOR_FREE: fused free-list tombstones (§4.B) */
					FT_GLUE_FLOOR_DEFERRED + 7 + 1 /* +1 §4.B parent guard */ + FT_GLUE_FLOOR_FREE
					/* +1 fused src-root retire edge (exclusive src, follow-up b) */
					+ (src_ft->exclusive ? 1 : 0)
					/* +1 fused nil-key wrapper tombstone (exclusive nil-key src) */
					+ ((src_ft->exclusive && nil_key_root) ? 1 : 0)
					/* + count walk: the +src_count nr_keys ancestor edges (BULK fold) */
					+ (dst_ft->rank_stats ? (int) key_len + 1 : 0)
					/* the split-retire terminal's second word, if any */
					+ ft_glue_split_cn_reserve(dst_ft))) {
				if (glue.txn)
					ft_flip_txn_destroy(glue.txn);
				if (already_swapped) {
					ft_txn_attempt_bail(&optxn, ra_txn);
					goto retry_attach;	/* src consumed: OOM is transient */
				}
				free_cds_ft_node_unpublished(src_ft, fresh_node);
				ft_txn_attempt_end(&optxn, ra_txn);
				return CDS_FT_STATUS_MEMORY_ERROR;
			}
		}
		/*
		 * Fuse each retired free-list node's freeze into glue.txn (atomic
		 * detach, §4.B): each tombstone rides the same commit as the forward
		 * publish that unlinks it, so the retired set freezes dead atomically
		 * with the unlink.  Both txn sources carry the <= FT_GLUE_FLOOR_FREE
		 * tombstone headroom: the create path reserves + FLOOR_FREE just above,
		 * and the rekey take() path's pre_txn is pre-sized to the same
		 * floor-bounded cluster + FLOOR_FREE by ft_merge_at_inner's m == 0
		 * graft branch (ft-merge.h).  The graft never calls ft_glue_reserve, so
		 * cap_free stays FT_GLUE_FLOOR_FREE and that headroom is the exact bound.
		 */
		glue.fuse_free_list = true;
		prep = ft_graft_build(dst_ft, key, key_len, graft_payload,
				src_count, &d, &glue, /*outer*/ NULL);
		if (prep == FT_GRAFT_PREP_OOM) {
			/*
			 * The GLUE build may have marked @cn's split-retire LOCK
			 * fence before hitting OOM; ft_glue_abort below releases it
			 * (single clear point), so @cn stays LIVE for the re-descend.
			 */
			ft_glue_abort(dst_ft, &glue);
			if (glue.txn)
				ft_flip_txn_destroy(glue.txn);
			if (already_swapped) {
				ft_txn_attempt_bail(&optxn, ra_txn);
				goto retry_attach;	/* src consumed: OOM is transient */
			}
			free_cds_ft_node_unpublished(src_ft, fresh_node);
			ft_txn_attempt_end(&optxn, ra_txn);
			return CDS_FT_STATUS_MEMORY_ERROR;
		}
		if (prep == FT_GRAFT_PREP_RETRY) {
			/*
			 * Split-retire @cn fence miss (MW LOCK_FINE drop): a peer owns
			 * @cn (splitting / growing / recompacting it, or changing
			 * cn->child).  ft_split_compressed_graft_build built NOTHING and
			 * did NOT mark @cn, so this is a clean re-descend -- dst is
			 * byte-for-byte unchanged, src pristine (pre-swap) or empty +
			 * owned by this writer (post-swap).  @cn's try-or-bail LOCK
			 * mark guarantees a winner each contention round (no livelock).
			 */
			ft_glue_abort(dst_ft, &glue);
			if (glue.txn)
				ft_flip_txn_destroy(glue.txn);
			if (src_retire_txn) {
				ft_flip_txn_destroy(src_retire_txn);
				src_retire_txn = NULL;
			}
			if (run_splice_txn) {
				ft_flip_txn_destroy(run_splice_txn);
				run_splice_txn = NULL;
			}
			if (!already_swapped)
				free_cds_ft_node_unpublished(src_ft, fresh_node);
			ft_txn_attempt_bail(&optxn, ra_txn);
			goto retry_attach;
		}
		if (prep == FT_GRAFT_PREP_POPULATED) {
			/*
			 * Post-swap POPULATED is impossible for the supported contract
			 * (an EXCLUSIVE src; a live dst whose graft key is this writer's
			 * own -- no peer creates @key): the payload is already detached,
			 * so there is no clean status left.  Assert rather than silently
			 * orphan it; a general (non-disjoint) caller must move to the
			 * fused src-retire+dst-store txn (doc, open follow-up).
			 */
			assert(!already_swapped);
			if (glue.txn)
				ft_flip_txn_destroy(glue.txn);
			if (!already_swapped)	/* NDEBUG: @fresh_node is NULL post-swap */
				free_cds_ft_node_unpublished(src_ft, fresh_node);
			ft_txn_attempt_end(&optxn, ra_txn);
			return CDS_FT_STATUS_POPULATED_ERROR;
		}
		/*
		 * Reanchor level-move (ft_descent skip_conflict): a peer chain-merge
		 * moved an encoded position SHALLOWER mid-descent, so the captured
		 * publish chain (@d->pnf / @d->pnfp, and the coherent graft-point
		 * parent @d->ppnf the recompact inherits) is at the wrong level.  A
		 * mutating caller must re-descend against the now-current tree -- the
		 * same bail insert takes (ft_insert.h skip_conflict).  Nothing is
		 * reader-visible yet (the build is invisible / d is a pure descent),
		 * so the abort boundary is byte-for-byte clean: drop the invisible
		 * build + reserved txn + fresh root and re-descend.
		 */
		if (caa_unlikely(d.skip_conflict)) {
			ft_glue_abort(dst_ft, &glue);
			if (glue.txn)
				ft_flip_txn_destroy(glue.txn);
			if (!already_swapped)
				free_cds_ft_node_unpublished(src_ft, fresh_node);
			ft_txn_attempt_bail(&optxn, ra_txn);
			goto retry_attach;
		}
		/*
		 * NOSPLIT graft point already occupied (the store would report this
		 * post-swap): surface POPULATED here, BEFORE the source-root swap, so
		 * the source stays pristine -- no rollback, no reader-observable
		 * empty-then-full flicker.  Same condition the store checks at
		 * d->depth == key_len.
		 */
		if (prep == FT_GRAFT_PREP_NOSPLIT && d.depth == key_len && d.nf) {
			assert(!already_swapped);	/* see the PREP_POPULATED note above */
			ft_flip_txn_destroy(glue.txn);
			if (!already_swapped)	/* NDEBUG: @fresh_node is NULL post-swap */
				free_cds_ft_node_unpublished(src_ft, fresh_node);
			ft_txn_attempt_end(&optxn, ra_txn);
			return CDS_FT_STATUS_POPULATED_ERROR;
		}

		/*
		 * Pre-reserve the failure-free src-root retire's txn now -- after
		 * the last POPULATED / OOM exit, before the only remaining fallible
		 * step (the self-secure reserve below).  The retire is past the
		 * point of no return and cannot abort, so it commits through this
		 * pre-reserved txn (ft_ord_cell_flip_into).  List off retires via a
		 * lone-edge root store, so reserve only when the list is on -- EXCEPT
		 * a NON-exclusive nil-key root retire fuses the orphaned wrapper's
		 * tombstone into the retire (atomic detach, §4.B), needing a 2-edge
		 * txn even list-off; list-on grows by +1 for that same fused
		 * tombstone.  An EXCLUSIVE nil-key src instead fuses BOTH edges into
		 * @glue.txn (the src-swap-fused arm below), so it needs no separate
		 * retire txn -- reserved for the wrapper tombstone above.
		 *
		 * One-shot (src-side): the src-retire txn is consumed by the swap
		 * below, so a post-swap retry keeps the src empty and does NOT
		 * re-reserve it (@already_swapped -- both txns stay NULL, the fused
		 * commit needs neither: @src_retire_txn was consumed and every attach
		 * shape fuses the run into glue.txn, so the standalone splice that
		 * would use @run_splice_txn is never taken).
		 */
		if (!already_swapped && dst_ft->group->ordered_list_set) {
			src_retire_txn = ft_flip_txn_create_bounded(src_ft,
				FT_ROOT_LIST_SWAP_MAX_EDGES + 1);
			run_splice_txn = ft_flip_txn_create_bounded(dst_ft,
				FT_ORD_CELL_RUN_SPLICE_MAX_EDGES);
			if (!src_retire_txn || !run_splice_txn) {
				/*
				 * Release the GLUE build's @cn split-retire fence (if
				 * held) so @cn stays LIVE -- this OOM bail does not route
				 * through ft_glue_abort.
				 */
				if (glue.split_cn_holder) {
					/* SHARED: the caller's earlier acquire owns the release. */
					if (!glue.split_cn_shared)
						ft_meta_lock_release(glue.split_cn_holder);
					glue.split_cn_holder = NULL;
					glue.split_cn_shared = false;
					glue.split_cn_snap = 0;
					glue.split_cn_node = NULL;
					glue.split_cn_node_snap = 0;
				}
				if (src_retire_txn)
					ft_flip_txn_destroy(src_retire_txn);
				if (run_splice_txn)
					ft_flip_txn_destroy(run_splice_txn);
				ft_flip_txn_destroy(glue.txn);
				free_cds_ft_node_unpublished(src_ft, fresh_node);
				ft_txn_attempt_end(&optxn, ra_txn);
				return CDS_FT_STATUS_MEMORY_ERROR;
			}
		} else if (!already_swapped && nil_key_root && !src_ft->exclusive) {
			src_retire_txn = ft_flip_txn_create_bounded(src_ft, 2);
			if (!src_retire_txn) {
				if (glue.split_cn_holder) {
					/* SHARED: the caller's earlier acquire owns the release. */
					if (!glue.split_cn_shared)
						ft_meta_lock_release(glue.split_cn_holder);
					glue.split_cn_holder = NULL;
					glue.split_cn_shared = false;
					glue.split_cn_snap = 0;
					glue.split_cn_node = NULL;
					glue.split_cn_node_snap = 0;
				}
				ft_flip_txn_destroy(glue.txn);
				free_cds_ft_node_unpublished(src_ft, fresh_node);
				ft_txn_attempt_end(&optxn, ra_txn);
				return CDS_FT_STATUS_MEMORY_ERROR;
			}
		}

		/*
		 * Ordered list: locate the dst splice neighbours NOW, while dst is
		 * still payload-free (the attach is built invisibly / not yet
		 * published) -- a relational descent after the payload is live
		 * would return a payload head as the boundary.
		 */
		if (dst_ft->group->ordered_list_set)
			ft_ord_cell_find_splice_pos(dst_ft, _key, key_len,
				&graft_pred, &graft_succ, NULL);

		/*
		 * Self-secure the NOSPLIT store BEFORE the point of no return (the
		 * source-root swap below).  A generous node reserve, drawn here where
		 * failure is clean (nothing published yet), makes the post-swap
		 * ft_store_at_graft_point unfailable -- so the old rollback that
		 * re-published the source root on a store OOM (a reader-observable
		 * flicker of the source: empty, then full again) is gone.  The store's
		 * flip latches draw from the pre-reserved @glue.txn.  Skipped when a
		 * caller reserve is already active (the rekey), which secures it via
		 * that reserve + the pre-reserved txn.
		 */
		if (prep == FT_GRAFT_PREP_NOSPLIT
				&& !cds_ft_alloc_reserve_covers(dst_ft)) {
			memset(&graft_reserve, 0, sizeof(graft_reserve));
			if (ft_bulk_node_reserve_fill(dst_ft, &graft_reserve)) {
				cds_ft_alloc_reserve_drain(dst_ft, &graft_reserve);
				ft_glue_abort(dst_ft, &glue);
				ft_flip_txn_destroy(glue.txn);
				if (src_retire_txn)
					ft_flip_txn_destroy(src_retire_txn);
				if (run_splice_txn)
					ft_flip_txn_destroy(run_splice_txn);
				if (already_swapped) {
					ft_txn_attempt_bail(&optxn, ra_txn);
					goto retry_attach;	/* src consumed: OOM is transient */
				}
				free_cds_ft_node_unpublished(src_ft, fresh_node);
				ft_txn_attempt_end(&optxn, ra_txn);
				return CDS_FT_STATUS_MEMORY_ERROR;
			}
			self_secured = true;
		}

		/*
		 * MW LOCK_FINE drop (§11, cross-trie): run the NOSPLIT store's
		 * FALLIBLE half -- ft_store_at_graft_point_prepare, i.e. the
		 * recompact of the graft-point node {p} -- BEFORE the point-of-no-
		 * return src-root swap.  A concurrent relocation of {p}, a peer that
		 * holds {p}'s (or the grandparent's) node lock, or a stale
		 * descent then surfaces as a clean re-descend with NOTHING on src
		 * touched (no reader-visible src flicker, no rollback).  The
		 * remaining commit is unfailable and runs after the swap through
		 * @st.  The reserve (self-secured above, or the caller's) covers the
		 * recompact copy; it is deactivated + drained here -- the copy
		 * survives in @st, the commit allocates nothing.
		 *
		 * Staleness guard: a NOSPLIT graft point is an INTERNAL slot owner.
		 * A compressed d.pnf means the descent raced a concurrent chain-
		 * compress / relocation (ft_node_set_nth_rec cannot target a
		 * compressed node) -- re-descend.  A tombstoned / proxied / peer-
		 * locked {p} is caught INSIDE the recompact's lock acquire
		 * (ft_meta_lock_acquire -> -EAGAIN -> prepare MEMORY_ERROR).
		 */
		if (prep == FT_GRAFT_PREP_NOSPLIT) {
			enum cds_ft_status pstatus;
			/*
			 * Displaced-external shape: the descent broke on an
			 * external d.nf, so the store re-parents that external
			 * under a fresh branch published at d.pnf's CHILD slot
			 * (ft_store_at_graft_point_prepare's displaced arm) -- it
			 * never targets d.pnf's byte-slots.  A compressed d.pnf is
			 * then the STABLE, expected structure (the graft key runs
			 * through a compressed path to an external leaf), NOT a
			 * transient race, so the store handles it as-is.
			 */
			bool displaced_shape = d.nf && ft_node_external(d.nf);

			/*
			 * A compressed d.pnf at a BYTE-SLOT store (depth==key_len,
			 * or a non-displaced diverge) means the descent raced a
			 * concurrent chain-compress / relocation -- ft_node_set_nth_rec
			 * cannot target a compressed node -- so re-descend.  The
			 * displaced shape above is EXEMPT: guarding it spun forever in
			 * single-writer (no peer to un-compress a stable compressed
			 * parent, so the re-descend never made progress).
			 */
			if ((ft_node_compressed(d.pnf)
					|| ft_node_skip_compressed(d.pnf))
					&& !displaced_shape) {
				pstatus = CDS_FT_STATUS_MEMORY_ERROR;
			} else {
				if (self_secured)
					cds_ft_alloc_reserve_activate(dst_ft,
						&graft_reserve);
				pstatus = ft_store_at_graft_point_prepare(dst_ft,
					key, key_len, &d, graft_payload,
					src_count, &glue, /*outer*/ NULL, &st);
				if (self_secured)
					cds_ft_alloc_reserve_deactivate(dst_ft);
			}
			if (self_secured) {
				cds_ft_alloc_reserve_drain(dst_ft, &graft_reserve);
				self_secured = false;
			}
			if (pstatus != CDS_FT_STATUS_OK) {
				/*
				 * Prepare failed BEFORE the swap: src is pristine,
				 * only the invisible dst build + reserved txns need
				 * unwinding.  Re-descend -- {p}'s node lock
				 * guarantees a winner each contention round.
				 */
				ft_glue_abort(dst_ft, &glue);
				if (glue.txn)
					ft_flip_txn_destroy(glue.txn);
				if (src_retire_txn) {
					ft_flip_txn_destroy(src_retire_txn);
					src_retire_txn = NULL;
				}
				if (run_splice_txn) {
					ft_flip_txn_destroy(run_splice_txn);
					run_splice_txn = NULL;
				}
				if (!already_swapped)
					free_cds_ft_node_unpublished(src_ft, fresh_node);
				ft_txn_attempt_bail(&optxn, ra_txn);
				goto retry_attach;
			}
			nosplit_prepared = true;
		}

		/*
		 * MW LOCK_FINE drop (§11, cross-trie): a forward publish into the
		 * captured dst spine parent @fence_parent (== d.pnf, a LIVE node from
		 * the descent) happens at the commit -- AFTER the src-root swap below.
		 * Under the FT-wide-lock drop a concurrent peer (a sibling graft growing
		 * that spine node, a point-remove recompacting it) can RETIRE it between
		 * this graft's descent and its commit; the commit's value-swap guard
		 * passes on a dead-but-stable state word, so the forward publish stores
		 * into a tombstoned node -- a wild store that corrupts the arena, and
		 * (worse) if the guard instead MISMATCHES the tombstoned word the commit
		 * ABORTS past the point of no return, silently dropping the whole
		 * subtree.  Lock @fence_parent's node lock HERE, before the swap, so
		 * no peer can retire it through the commit (which records the held
		 * {LOCK|s -> s} release via @publish_parent_holder, so the commit is
		 * truly unfailable).  A miss (already retired / proxied / peer-locked)
		 * is a clean re-descend with src pristine -- ft_meta_lock_acquire did
		 * NOT set the fence, so nothing to unwind but the invisible dst build.
		 *
		 * TWO shapes reach an unfenced forward publish into d.pnf: the GLUE
		 * diverge (@glue.publish_parent set at build) and the NOSPLIT
		 * DISPLACED-external attach (its commit publishes the fresh branch into
		 * d.pnf's child slot, st.pnf == d.pnf; @st is valid once
		 * @nosplit_prepared).  The in-place NOSPLIT store needs no fence here --
		 * its ft_node_set_nth_rec recompacts + node locks {p} inside prepare
		 * and holds it through commit.  Gated on lock_fine so the FT-wide-lock
		 * build is byte-identical (the mutex already serialises retires).
		 */
		{
			struct cds_ft_inode_flag *fence_parent = NULL;

			if (dst_ft->lock_fine) {
				if (prep == FT_GRAFT_PREP_GLUE)
					fence_parent = glue.publish_parent;
				else if (nosplit_prepared && st.displaced_shape)
					fence_parent = st.pnf;
			}
			/*
			 * The GLUE build already fenced the compressed divergence node
			 * @cn (== d.nf) it splits + replaces -- ft_split_compressed_graft_build
			 * marks it BEFORE reading its plan, so the whole build runs under
			 * the fence and glue.split_cn_holder is already set.  Here we fence
			 * only @publish_parent (the LIVE node the forward publish stores
			 * INTO), which @cn's fence does not cover.
			 */
			if (fence_parent) {
				struct cds_ft_metadata *pp_meta =
					ft_flag_to_metadata(dst_ft, fence_parent);
				struct ft_lock_ctx fctx;
				struct ft_held_anchor fh;
				unsigned int fdep;

				/*
				 * ROUTED, not raw: this fence is what collided
				 * with the glue split's own anchor under a
				 * coarse spacing -- the op missed against its
				 * OWN hold, aborted, and retried into the
				 * identical shape.  A miss is a clean re-descend
				 * for a PEER's hold, never for the op's own, so
				 * the dedupe has to see this one.
				 *
				 * Which takes the GLUE's ctx, not a bare one:
				 * the glue's marks are part of this op's held
				 * set, and @fence_parent is routinely the very
				 * word the build fenced as @split_cn_holder.
				 * ft_lock_ctx_init carries no glue, so the
				 * dedupe cannot see that hold and the acquire
				 * refuses the op's own mark after all.  The
				 * descent stays this site's own.
				 */
				ft_glue_lock_ctx(&glue, &fctx);
				fctx.d = &d;
				if (!ft_lock_ctx_depth_of(dst_ft, &fctx,
							fence_parent, &fdep) ||
						ft_acquire_member(dst_ft, &fctx,
							fence_parent, pp_meta,
							fdep, &fh)) {
					/*
					 * A miss (@publish_parent already retired / proxied /
					 * peer-locked) is a clean re-descend, src pristine.
					 * ft_glue_abort below releases the @cn split-retire
					 * fence held by the GLUE build (if any), so no fence
					 * leaks (the commit never registered it).
					 */
					ft_glue_abort(dst_ft, &glue);
					if (glue.txn)
						ft_flip_txn_destroy(glue.txn);
					if (src_retire_txn) {
						ft_flip_txn_destroy(src_retire_txn);
						src_retire_txn = NULL;
					}
					if (run_splice_txn) {
						ft_flip_txn_destroy(run_splice_txn);
						run_splice_txn = NULL;
					}
					if (!already_swapped)
						free_cds_ft_node_unpublished(src_ft, fresh_node);
					ft_txn_attempt_bail(&optxn, ra_txn);
					goto retry_attach;
				}
				/*
				 * SHARED is the dedupe SUCCEEDING, not a miss:
				 * the op already holds the word protecting
				 * @fence_parent, so the exclusion this fence
				 * exists for is in force, and the member owes no
				 * release and no terminal -- the acquire that
				 * first took the word recorded both.  Leave the
				 * holder unset and the commit's
				 * ft_flip_txn_hold_or_lock_parent takes its
				 * ordinary acquire-or-guard route, whose own
				 * shared arm records nothing for that same
				 * reason.  Bailing here refuses the op's OWN
				 * hold, which no re-descend can clear.
				 */
				if (!fh.shared) {
					glue.publish_parent_holder = fh.lock;
					glue.publish_parent_snap = fh.lock_snap;
				}
			}
		}

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
		/*
		 * Point of no return -- ONE-SHOT (doc post-store retry): capture
		 * @old_src_root + the src ordered-list run, then unlink the src root
		 * (publish a fresh empty root).  A post-swap commit-abort retry keeps
		 * src empty and re-runs only the dst-side attach, so this whole block
		 * runs exactly once: @already_swapped guards it, @fresh_node is nulled
		 * (it is now src's LIVE root -- every src-side free above is skipped),
		 * and @src_retire_txn is consumed here + nulled so a retry's earlier
		 * exits never re-destroy it.  @graft_run_first / _last persist across
		 * retries (the run moved with the payload; src is now empty so the run
		 * cannot be re-read from it).
		 */
		if (!already_swapped) {
			old_src_root = src_ft->root;

			/*
			 * Freeze-on-free (doc §4.B): a NIL-key graft frees the orphaned
			 * src-root wrapper @old_src_root below (its external chain was
			 * grafted into dst, leaving the wrapper empty); its tombstone rides
			 * the SAME flip as the src-root retire that unlinks it (atomic
			 * detach), recorded into the retire txn in each arm below.  Non-NIL:
			 * @old_src_root IS the payload, moved LIVE into dst -- it must NOT be
			 * marked.  No fallible step between here and the retire (all returned
			 * above).
			 *
			 * Ordered list: capture src's whole list (the run to graft) and,
			 * paired with the structural src-root retire, unlink it from src
			 * -- FUSED into ONE flip so a src reader never sees src
			 * structurally empty while its ordered list still shows the run
			 * (or vice versa).  The synchronize_rcu below then drains src
			 * readers of the old content; the run is spliced into dst after
			 * the structural publish (same commit point).  No rollback: every
			 * failure mode (OOM / populated) returned above, before this
			 * retire.
			 */
			if (src_ft->exclusive && !dst_ft->group->ordered_list_set) {
				/*
				 * FUSED cross-trie move (exclusive src, list off): record the
				 * src-root retire INTO @glue.txn instead of a separate flip, so
				 * the src unlink flips ATOMICALLY with the dst attach below --
				 * no orphan window, no drain (no readers).  @already_swapped
				 * stays FALSE until that commit succeeds (set post-commit), so a
				 * commit abort rolls this src edge back too and the retry
				 * re-descends with src still full (clean, like a pre-swap
				 * failure).
				 */
				ft_flip_txn_record_root(glue.txn,
					(void **) &src_ft->root,
					(void *) old_src_root,
					(void *) ft_node_flag(fresh_node, 0));
				/*
				 * NIL-key src: @old_src_root is the emptied wrapper (its
				 * external chain moved LIVE into dst); its freeze-on-free
				 * tombstone rides the SAME fused flip (atomic detach §4.B) so
				 * the unlink + freeze commit atomically with the attach -- an
				 * abort rolls the tombstone back too and the wrapper is
				 * reclaimed post-drain at the nil-key free below.  Non-nil:
				 * @old_src_root IS the payload, moved LIVE -- no tombstone.
				 */
				if (nil_key_root)
					ft_flip_txn_record_tombstone(glue.txn,
						cds_ft_item_to_metadata(
							ft_node_ptr(old_src_root)));
				src_swap_fused = true;
			} else if (dst_ft->group->ordered_list_set) {
				graft_run_first = ft_ord_first(src_ft);
				graft_run_last = ft_ord_last(src_ft);
				/*
				 * Just empty src's sentinel here (relink_dest NULL): the run's
				 * cells are re-homed into dst by the run-splice below, which
				 * repoints their outer links to dst's neighbours / sentinel.
				 */
				if (nil_key_root)
					ft_flip_txn_record_tombstone(src_retire_txn,
						cds_ft_item_to_metadata(
							ft_node_ptr(old_src_root)));
				ft_root_list_swap_publish(src_ft, src_retire_txn,
					&src_ft->root,
					old_src_root, ft_node_flag(fresh_node, 0),
					graft_run_first, NULL, graft_run_last, NULL,
					NULL, false);
			} else if (nil_key_root) {
				/*
				 * List off + nil-key: the lone root edge plus the orphaned
				 * wrapper's tombstone commit as ONE 2-edge flip through the
				 * pre-reserved txn (readers resolve the transient root proxy
				 * exactly as on the list-on path).
				 */
				ft_flip_txn_record_root(src_retire_txn,
					(void **) &src_ft->root,
					(void *) old_src_root,
					(void *) ft_node_flag(fresh_node, 0));
				ft_flip_txn_record_tombstone(src_retire_txn,
					cds_ft_item_to_metadata(ft_node_ptr(old_src_root)));
				ft_flip_txn_commit(src_ft, src_retire_txn);
			} else {
				ft_root_edge_flip(src_ft, &src_ft->root,
					old_src_root, ft_node_flag(fresh_node, 0));
			}
			if (!src_swap_fused) {
				/*
				 * Separate swap committed: src is now empty + the payload
				 * owned.  The FUSED arm defers all of this to its atomic
				 * commit (below) -- src->root is unchanged here, so it keeps
				 * @already_swapped false and @fresh_node live until then.
				 */
				FT_TP(root_publish, (const void *) src_ft,
					(const void *) src_ft->root);
				already_swapped = true;
				fresh_node = NULL;	/* now src's live root: never freed below */
				src_retire_txn = NULL;	/* consumed by the swap above */
			}
		}

		/*
		 * Ordered list: arm the run-splice fusion so the NOSPLIT store
		 * commits the structural attach and the ordered-list splice in ONE
		 * flip (closing the appear-side cross-view window).  The GLUE path
		 * and the displaced-external store shape leave @armed false, falling
		 * back to the standalone two-commit splice below.
		 */
		if (graft_run_first) {
			/*
			 * NIL-key graft: stamp the spliced cell's key-rebuild byte
			 * BEFORE it is fused into dst's ordered list -- once the
			 * fused splice publishes the cell, a reader iterating to it
			 * rematerializes the key via incoming_byte.  (ft_set_parent
			 * does not maintain it for externals; harmless when the
			 * parent is compressed, where the up-walk ignores it.)
			 */
			if (nil_key_root)
				cds_ft_item_to_metadata(graft_run_first)->incoming_byte =
					key[key_len - 1];
			graft_run.run_first = graft_run_first;
			graft_run.run_last = graft_run_last;
			graft_run.pred = graft_pred;
			graft_run.succ = graft_succ;
			graft_run.armed = false;
			run_arg = &graft_run;
		}

		if (!src_ft->exclusive)
			ft_writer_lock_gp_wait(src_ft);

		if (prep == FT_GRAFT_PREP_GLUE) {
			/*
			 * Failure-free commit of the build-invisible diverge
			 * cluster: wire the deferred live back-pointers (the
			 * displaced old child, the payload, and the cluster
			 * top), splice the cluster into dst with a single
			 * forward publish -- then reclaim the old compressed
			 * node and the source's old root.  Nothing can fail.
			 *
			 * glue.txn: commit the back-pointers, the forward
			 * publish, AND (list on) the <=4 ordered-list run-splice
			 * cell edges as ONE atomic flip-txn, so the whole attach
			 * -- structure and ordered list -- is observed old XOR new
			 * with no deferred-edge ordering window.  ft_glue_txn_commit
			 * arms @run_arg, so the standalone splice below is skipped.
			 *
			 * Order-statistics fold (BULK): the diverge cluster raises
			 * @glue.publish_parent's subtree by +src_count; record that
			 * walk into the same commit (a no-op when rank stats off).
			 */
			glue.count_delta = (long) src_count;
			store_cst = ft_glue_txn_commit(dst_ft, &glue, run_arg);
			attached_nf = glue.attached_nf;
			/*
			 * Defect D / abort-safety: the diverge cluster's OLD nodes
			 * (compressed graft-point node, gathered on glue's free list)
			 * are only reclaimable when the commit committed.  Under the
			 * drop ft_glue_txn_commit CAN abort (a peer conflicts on the
			 * forward-publish footprint the pre-swap fence does not cover);
			 * an aborted commit left those nodes LIVE, so freeing them here
			 * would double-free them against the peer.  Gate on @store_cst;
			 * the post-swap retry below re-descends and re-commits.
			 */
			if (store_cst == URCU_TXN_STATUS_OK)
				ft_glue_free_old(dst_ft, &glue);
		} else {
			/*
			 * Non-split attach: the payload subtrie is built into
			 * @glue with its back-pointers deferred, recompacted into
			 * the live graft-point node, and published -- all inside
			 * ft_store_at_graft_point.  Every node draws from the
			 * reserve (self-secured above, or the caller's) and the
			 * slot proxy + run-splice edges draw from the pre-reserved
			 * @glue.txn, so the store has no fallible step left: it
			 * cannot fail, and there is NO source-root rollback (which
			 * would have flickered the source empty-then-full).
			 */
			unsigned int attached_depth = 0;

			/*
			 * NOSPLIT commit: the fallible half (the recompact of the
			 * graft-point node {p}) already ran build-invisibly BEFORE
			 * the swap (@st / @nosplit_prepared).  The commit only
			 * records the forward slot edge, the relocation edges, and
			 * the ordered-list run-splice into the pre-reserved
			 * @glue.txn and flips them -- no allocation, no fallible
			 * step, no src rollback.
			 */
			assert(nosplit_prepared);
			store_cst = ft_store_at_graft_point_commit(dst_ft,
				&attached_nf, &attached_depth, run_arg, &st,
				(long) src_count);
		}

		/*
		 * FUSED cross-trie move: the src-root retire rode @glue.txn, so a
		 * SUCCESSFUL commit atomically emptied src (its new root is @fresh_node)
		 * AND published the payload into dst -- no orphan window.  Only NOW is
		 * the swap done: mark it so the abort handling and any teardown treat
		 * src as consumed, and stop guarding @fresh_node (it is src's LIVE
		 * root).  A FAILED commit rolled the src-root edge back too, so
		 * @already_swapped stays false and the store-abort retry below
		 * re-descends with src still full -- a clean pre-swap-style re-attempt.
		 */
		if (store_cst == URCU_TXN_STATUS_OK && src_swap_fused) {
			already_swapped = true;
			fresh_node = NULL;
			FT_TP(root_publish, (const void *) src_ft,
				(const void *) src_ft->root);
		}

		/*
		 * Post-swap store ABORT (§11 drop, doc ft-graft-abort-safe-poststore-
		 * retry): under the FT-wide-lock drop the attach commit can conflict
		 * with a peer beyond the single word the pre-swap fence covers and
		 * ABORT.  The rolled-back flip left dst byte-for-byte unchanged, every
		 * registered node lock auto-cleared (the recompact's {p} retire and
		 * the Fix-A publish fence both back to LIVE), and glue.txn consumed.
		 * The payload is still detached from the emptied EXCLUSIVE (reader-free)
		 * src and OWNED by us, so RE-ATTEMPT the dst-side attach: free this
		 * attempt's UNPUBLISHED products (the aborted txn rolled its edges back
		 * but freed no nodes) and re-descend.  A peer committed a conflicting
		 * flip => it made progress; {p}'s LOCK try-lock guarantees a winner
		 * each round, so the retry terminates (the whole-op RCU pin spans it).
		 */
		if (store_cst != URCU_TXN_STATUS_OK) {
			/*
			 * Free the failed attempt's unpublished new nodes.  GLUE: the
			 * whole diverge cluster (every fresh node ft_glue_track'd);
			 * glue.txn is already NULL (the commit consumed it).  NOSPLIT:
			 * the displaced branch + any glue cluster, PLUS -- if the reserve
			 * RELOCATED {p} -- the unpublished relocated {p}' (@st.dest,
			 * allocated directly by ft_node_recompact, NOT glue-tracked).  An
			 * IN-PLACE recompact (old_recompacted_node == NULL) leaves
			 * @st.dest == the LIVE d->pnf -- do NOT free it.  Never
			 * ft_glue_free_old here: those OLD nodes stay LIVE (the abort
			 * rolled their retire back), reclaimed by whoever finally commits.
			 */
			ft_glue_abort(dst_ft, &glue);
			if (prep != FT_GRAFT_PREP_GLUE && st.old_recompacted_node)
				free_cds_ft_node_unpublished(dst_ft,
					ft_node_ptr(st.dest));
			/*
			 * Per-attempt state reset before re-descending: the NOSPLIT
			 * prepare re-fills @st (and re-sets @nosplit_prepared) from
			 * scratch; the run-splice fallback txn (list on) was created
			 * one-shot but never consumed (the run fused into glue.txn), so
			 * drop it -- a retry stays fused and needs none.  For the SEPARATE
			 * swap (@already_swapped) @fresh_node / @src_retire_txn are the
			 * one-shot src state, held across the retry by the swap guard.
			 * For the FUSED move the abort rolled the src-root edge back too,
			 * so @already_swapped stayed FALSE and @fresh_node is this
			 * attempt's UNPUBLISHED empty root -- free it (the retry re-allocs
			 * a fresh one at the loop top), exactly as the fallible-prefix
			 * retries do; otherwise it leaks one arena node per aborted round.
			 */
			nosplit_prepared = false;
			if (run_splice_txn) {
				ft_flip_txn_destroy(run_splice_txn);
				run_splice_txn = NULL;
			}
			if (!already_swapped)
				free_cds_ft_node_unpublished(src_ft, fresh_node);
			ft_txn_attempt_bail(&optxn, ra_txn);
			goto retry_attach;
		}

		/*
		 * Order-statistics: the +src_count ancestor walk is now FOLDED
		 * onto the attach commit above (glue.count_delta for the GLUE
		 * diverge and displaced-external shapes; ft_store_at_graft_point's
		 * count_delta for the in-place / recompact-relocate slot shapes),
		 * so it flips ATOMICALLY with the structural publish -- exact
		 * under concurrent writers.  No post-commit propagate walk.
		 */

		/*
		 * Ordered list: src is now structurally empty + drained; the payload
		 * is published under @key in dst.  Every ordered graft shape (GLUE,
		 * displaced-external, in-place slot) FUSES the run-splice into its
		 * structural flip (@armed), so a reader never sees the run in one
		 * index but not the other.  The standalone two-commit splice remains
		 * as a defensive fallback for any not-yet-fused shape (none today);
		 * without it an unfused shape would strand the run out of the list.
		 * It commits through the pre-reserved @run_splice_txn (un-abortable
		 * post-drain); the fused common case frees that txn unused.
		 */
		if (graft_run_first && !graft_run.armed) {
			ft_ord_cell_run_splice(dst_ft, run_splice_txn,
				graft_run_first, graft_run_last, graft_pred,
				graft_succ);
			run_splice_txn = NULL;	/* consumed */
		}
		if (run_splice_txn)
			ft_flip_txn_destroy(run_splice_txn);	/* fused: unused */

		/*
		 * NIL-key graft succeeded: the external chain head was placed
		 * directly, so the orphaned wrapper internal (the old source root)
		 * is reclaimed.  Deferred, as readers may have been inside it before
		 * the root swap + drain above.
		 */
		if (nil_key_root)
			free_cds_ft_node(src_ft, ft_node_ptr(old_src_root));
		/*
		 * The attach SUCCEEDED and nothing below re-attempts: close the
		 * attempt without aging it.  This is the loop's only fall-out
		 * edge; every other exit closes at its own return / goto.
		 */
		ft_txn_attempt_end(&optxn, ra_txn);
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

	if (dst_ft->lock_fine) {
		const struct rcu_flavor_struct *flavor = dst_ft->group->flavor;

		/*
		 * FT-wide-lock drop RCU-pinning (§11 cross-trie).  A FINE
		 * cross-trie graft descends the SHARED dst WITHOUT the FT-wide
		 * mutex and captures spine nodes (d->pnf / ppnf / pppnf) that its
		 * recompact / Fix-A fence later node lock.  A node lock
		 * rejects a relocated-but-LIVE (tombstoned) node, but NOT a
		 * reclaimed-and-recycled one -- the arena re-zeroes a slot's
		 * metadata on reallocation, so ft_lock_member false-
		 * succeeds on a recycled node -> wild store into an unrelated live
		 * node.  Nothing else pins the captured nodes: ft_graft_keylen
		 * builds its txn with ft_flip_txn_create() (flavor NULL, "the
		 * caller brackets the RCU read side") and ft_writer_lock_scope
		 * no-ops under the drop.  Provide that bracket here so no grace
		 * period can reclaim a captured node across descent->lock->commit
		 * (the same read section insert gets from urcu_txn_begin).  Safe
		 * to hold across the whole op: a FINE cross-trie graft REQUIRES an
		 * exclusive src (BUSY_ERROR otherwise, ft_graft_keylen) and every
		 * grace period in the body (ft_writer_lock_gp_wait) is
		 * !exclusive-gated, so nothing synchronizes under the read lock.
		 */
		flavor->read_lock();
		status = ft_graft_keylen(dst_ft, _key, key_len, src_ft, NULL);
		flavor->read_unlock();
	} else
		status = ft_graft_keylen(dst_ft, _key, key_len, src_ft, NULL);
	FT_TP(graft_exit, (int) status);
	return status;
}

#ifdef FEATURE_FT_PROBE_GRAFT_SWAP
extern unsigned long cds_ft_probe_gs_commit_ok;
extern unsigned long cds_ft_probe_gs_reoccupy;
extern unsigned long cds_ft_probe_gs_slot_moved;
extern unsigned long cds_ft_probe_gs_retry;
extern unsigned long cds_ft_probe_gs_exact;
extern unsigned long cds_ft_probe_gs_kshort;
extern unsigned long cds_ft_probe_gs_delegate;
extern unsigned long cds_ft_probe_gs_fused;
extern unsigned long cds_ft_probe_gs_ext_child;
extern unsigned long cds_ft_probe_gs_torn;
extern unsigned long cds_ft_probe_gs_alias;
extern unsigned long cds_ft_probe_gs_canon_alias;
extern unsigned long cds_ft_probe_gs_fuse_pcn;
extern unsigned long cds_ft_probe_gs_fuse_ccn;
extern unsigned long cds_ft_probe_gs_fuse_len;
extern unsigned long cds_ft_probe_gs_fuse_incoh;
extern unsigned long cds_ft_probe_gs_fuse_incoh_committed;
#endif	/* FT_GS_PROBE_INC comes from ft-mutation-helpers.h */

/*
 * Outcome of ft_graft_swap_descend's read-only descent toward the swap key.
 */
enum ft_graft_swap_case {
	FT_GRAFT_SWAP_EXACT,		/* reached key_len at a live subtree (d->nf) */
	FT_GRAFT_SWAP_KEY_SHORTER,	/* key ends strictly inside compressed d->nf */
	FT_GRAFT_SWAP_DELEGATE,		/* diverge / dead-end: no content at key */
};

/*
 * Settle the graft point's PAIR -- the RAW slot value the commit quotes as its
 * expected-old (@raw_ret) and the RESOLVED occupant the extract side re-roots
 * into @swap_ft (@d->nf) -- from ONE load of the slot.
 *
 * ★ ONE LOAD, NOT TWO.  Deriving @d->nf and @raw_ret from separate loads of
 * the slot lets a peer graft_swap land between them, so the pair names
 * DIFFERENT objects: the commit ratifies displacing the node the peer just
 * installed while the extract side re-roots the one the peer just took.  Both
 * swap tries then own that subtree, and once the aliased node comes back round
 * to the graft point it is BOTH what this op publishes and what it quotes as
 * expected-old -- a no-op replace reporting OK, after which the extract side
 * NULLs the parent of a node still wired into dst.  Note the window is wide:
 * the two loads are observed to disagree hundreds of times per run of
 * inv_graft_swap_shared_dst_nolist, so this is a routine interleaving, not a
 * corner (FEATURE_FT_PROBE_GRAFT_SWAP counts it).
 *
 * Re-derive through the SAME primitive ft_node_get_nth_reanchor_slot uses
 * (resolve flip proxy, then reanchor) so the comparison is exact rather than a
 * skip-encoding artifact.  That it IS exact rests on @d->nf always holding the
 * REANCHORED form at both exits: every producer (ft_descent_init's root,
 * ft_descent_step, ft_descent_traverse_compressed) yields a value that is
 * "never skip-encoded", which is also why the descent loop's defensive
 * ft_resolve_skip_compressed(@d->nf) is a no-op and does NOT leave the
 * KEY_SHORTER exit comparing a one-hop resolve against a reanchor.  A future
 * producer that skips the reanchor would turn this guard into a permanent
 * mismatch -- i.e. an unbounded re-descend -- so keep that invariant.
 * On disagreement -- or on a reanchor level-move --
 * raise @d->skip_conflict, the existing "a mutating caller must re-descend"
 * flag (ft-mutation-helpers.h), and leave @raw_ret NULL so a caller that
 * somehow skipped the check still gets an expected-old that cannot match a
 * populated slot, i.e. an aborted commit rather than a corrupted trie.
 *
 * Returns true when the pair is coherent.
 */
static
bool ft_graft_swap_settle(struct cds_ft *ft, struct ft_descent *d,
		struct cds_ft_inode_flag **raw_ret)
{
	struct cds_ft_inode_flag *raw = *d->nfp;	/* THE one load */
	/*
	 * Resolve ONCE, and hand the SETTLED form out.  The validation below
	 * already resolves -- what @raw_ret feeds is the forward publish's
	 * expected-old, and an engine proxy is not a value the slot HAS: it is a
	 * parking marker for a record whose owner writes the real value at
	 * settle.  Handing the marker out records an expected-old that either
	 * never matches, or (worse, on a validate edge) matches while the parker
	 * is still parked and republishes a pointer into a descriptor nobody will
	 * ever settle again.  Resolving keeps the "one load" coherence -- both
	 * uses come from @raw -- and yields the FT-form value (SKIP_X included)
	 * the CAS must compare against.
	 */
	struct cds_ft_inode_flag *settled = ft_resolve_flip_proxy(raw);
	unsigned int rewind = 0;

	if (caa_unlikely(ft_reanchor_flag(ft, settled,
			&rewind) != d->nf || rewind != 0)) {
		FT_GS_PROBE_INC(cds_ft_probe_gs_torn);
		d->skip_conflict = true;
		return false;
	}
	*raw_ret = settled;
	return true;
}

/*
 * Read-only descent to the graft point for cds_ft_graft_swap.  Unlike
 * ft_descend_to_graft_point it publishes nothing: a key-shorter or diverging
 * key is reported, never split in place, so the whole swap can be assembled as
 * a build-invisible transaction.
 *
 *   FT_GRAFT_SWAP_EXACT:       d->depth == key_len and d->nf is the existing
 *                              subtree at @key (the displaced old-child).
 *   FT_GRAFT_SWAP_KEY_SHORTER: @key ends inside the compressed node d->nf
 *                              (d->depth is the node's start depth, d->pnf /
 *                              d->nfp hold it).
 *   FT_GRAFT_SWAP_DELEGATE:    the path diverges, dead-ends, or the slot at
 *                              @key is empty -- there is nothing to extract, so
 *                              the swap reduces to an insert (the caller routes
 *                              to the now-atomic cds_ft_graft).
 *
 * @raw_ret (EXACT / KEY_SHORTER): the graft-point slot's RAW content, read here
 * at the descent's own cursor.  The caller quotes it as the forward publish's
 * expected-old, so the edge that DISPLACES the occupant is decided by the same
 * descent that told the extract side which occupant to RE-ROOT.  Re-reading the
 * slot later -- at the caller's plan, or (worse) at record time inside the glue
 * committer -- widens that gap: a peer that swaps the graft point in between
 * makes the late read report the PEER's node, so the commit ratifies displacing
 * content this op never planned to take while the extract side still moves the
 * stale occupant into @swap_ft.  Both then own the same subtree.
 *
 * ★ IT MUST BE THE SLOT, NOT @d->nf.  ft_descent_step stores the REANCHORED
 * resolution of the slot (ft_node_get_nth_reanchor_slot) while @d->nfp names
 * the raw slot, so for a SKIP_X occupant the two differ -- and an expected-old
 * that can never match turns the KEY_SHORTER legacy publish, whose commit
 * status is dropped, into a silent no-op that still frees the replaced node.
 *
 * ★ AND IT MUST BE THE SAME LOAD THAT PRODUCED @d->nf.  Reading the slot a
 * second time here is what the ft_graft_swap_settle exit closes: the pair is
 * settled from ONE load, and a disagreement raises @d->skip_conflict for the
 * caller to re-descend (@raw_ret then stays NULL).
 *
 * A CALLER MUST TEST @d->skip_conflict before trusting @d or @raw_ret.
 * cds_ft_graft_swap does (retry_swap).  Of ft_merge_descend's three callers
 * only the dst descent does (fractal-trie.c:1016); the src descent
 * (ft-merge.h:2794) and the merge-point re-descend (:2883) still ignore it,
 * as they did before this exit existed -- the flag simply raises slightly
 * more often now.  Those two are unaudited, not known-safe.
 */
static
enum ft_graft_swap_case ft_graft_swap_descend(struct cds_ft *ft,
		const uint8_t *key, size_t key_len, struct ft_descent *d,
		struct cds_ft_inode_flag **raw_ret)
{
	const uint8_t *ik = key;

	*raw_ret = NULL;
	ft_descent_init(d, ft);
	for (; d->depth < key_len; ) {
		if (ft_node_external(d->nf))
			return FT_GRAFT_SWAP_DELEGATE;
		d->nf = ft_resolve_skip_compressed(ft, d->nf);
		if (ft_node_compressed(d->nf)) {
			struct cds_ft_compressed_node *cn =
				ft_compressed_node_ptr(d->nf);
			int remaining = (int) (key_len - d->depth);
			int cmp = cn->len < remaining ? cn->len : remaining;
			int j = ft_match_compressed_key(ik, cn, cmp);

			if (j < cmp)
				return FT_GRAFT_SWAP_DELEGATE;	/* diverge */
			if (cn->len <= remaining) {
				ft_descent_traverse_compressed(ft, d, cn, &ik);
				continue;
			}
			/*
			 * j == cmp == remaining < cn->len: key ends inside cn.
			 * @cn came from the earlier load, so settle the pair (and
			 * with it this classification) against ONE load of the slot.
			 */
			(void) ft_graft_swap_settle(ft, d, raw_ret);
			return FT_GRAFT_SWAP_KEY_SHORTER;
		}
		if (!ft_descent_step(ft, d, *(ik++)))
			return FT_GRAFT_SWAP_DELEGATE;	/* dead-end */
	}
	if (!d->nf)
		return FT_GRAFT_SWAP_DELEGATE;	/* empty slot at key */
	(void) ft_graft_swap_settle(ft, d, raw_ret);
	return FT_GRAFT_SWAP_EXACT;
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

	/*
	 * MW LOCK_FINE (step 6, §9.5): @swap_ft is the consumed source of the
	 * exchange, so like cds_ft_graft it must be EXCLUSIVE -- an exclusive
	 * swap skips its FT-wide lock, so only dst's lock is taken (one lock, no
	 * cross-trie deadlock), and the dual root/subtree swap runs directly and
	 * is build-invisible (a rejected swap leaves both tries pristine).  A
	 * LIVE (lock-mode, non-exclusive) @swap_ft is REJECTED with BUSY before
	 * anything is touched; the caller makes it exclusive first
	 * (cds_ft_make_exclusive).  @dst_ft may be a live concurrent trie.  Inert
	 * outside lock-mode.  (On success @swap_ft inherits @dst_ft's access
	 * discipline -- see the header -- so it may end up concurrent again.)
	 */
	if (!swap_ft->exclusive) {
		FT_TP(graft_swap_exit, (int) CDS_FT_STATUS_BUSY_ERROR);
		return CDS_FT_STATUS_BUSY_ERROR;
	}

	ft_crosstrie_lock_mode_guard(dst_ft, swap_ft);
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
		struct ft_flip_txn *dual_txn;

		/*
		 * PRE-RESERVE the cross-trie dual root-swap txn before the
		 * failure-free swap below: it commits through this txn
		 * (ft_ord_cell_flip_into).  OOM here aborts cleanly -- the
		 * whole-trie swap has no prior fallible state, both tries pristine.
		 * The dual always flips both roots, so it is multi-edge even list
		 * off (never a lone store).
		 */
		/*
		 * NAMED FOR @dst_ft, WRITES BOTH ROOTS -- here &dst_ft->root and
		 * &swap_ft->root.  Both record MW by construction, exactly as in
		 * the graft dual above.
		 */
		dual_txn = ft_flip_txn_create_bounded(dst_ft,
			FT_ROOT_LIST_SWAP_DUAL_MAX_EDGES);
		if (!dual_txn) {
			FT_TP(graft_swap_exit,
				(int) CDS_FT_STATUS_MEMORY_ERROR);
			return CDS_FT_STATUS_MEMORY_ERROR;
		}

		/*
		 * Swap both roots and (mirroring them) both ordered lists, fused
		 * into ONE cross-trie flip (dst and swap share a group, so a
		 * single epoch flip settles every root/endpoint proxy at once).
		 * A reader resolves all slots to ONE phase, so it never sees a
		 * side's structure showing NEW content while its ordered-list
		 * front is still OLD (the intra-trie window), nor a key reachable
		 * in both tries or neither (the cross-trie window).  The moved
		 * runs' outer links are NULL-terminated by the dual (universal
		 * end) rather than relinked to the foreign sentinel: both sides
		 * are live, so detach and attach cannot be split by a drain here
		 * -- a straddler of either run must see a universal end, not the
		 * other trie's sentinel.  Capture the swap root and all four
		 * endpoints up front: the two sides reference each other's pre-swap
		 * values.  The drain + finalize below (AFTER the flip) retires the
		 * straddlers and restores each side's run to circular.  (List off:
		 * head/tail are unused; the swap reduces to the two lone root
		 * edges, MCAS-expressible like the list-on path.)
		 */
		{
			struct cds_ft_inode_flag *swap_root = swap_ft->root;
			struct ft_ord_cell *dh = ft_ord_first(dst_ft);
			struct ft_ord_cell *dt = ft_ord_last(dst_ft);
			struct ft_ord_cell *sh = ft_ord_first(swap_ft);
			struct ft_ord_cell *st = ft_ord_last(swap_ft);
			struct ft_root_swap_side dst_side = {
				.ft = dst_ft, .slot = &dst_ft->root,
				.old_root = tmp, .new_root = swap_root,
				.head_old = dh, .head_new = sh,
				.tail_old = dt, .tail_new = st,
			};
			struct ft_root_swap_side swap_side = {
				.ft = swap_ft, .slot = &swap_ft->root,
				.old_root = swap_root, .new_root = tmp,
				.head_old = sh, .head_new = dh,
				.tail_old = st, .tail_new = dt,
			};

			ft_root_list_swap_publish_dual(dual_txn, &dst_side,
				&swap_side);
		}
		FT_TP(root_publish, (const void *) dst_ft,
			(const void *) dst_ft->root);
		FT_TP(root_publish, (const void *) swap_ft,
			(const void *) swap_ft->root);

		/*
		 * Drain readers of either side -- both the pre-swap readers (this
		 * subsumes the old pre-flip drain: a whole-trie root swap changes
		 * no parent pointer, so nothing between the flip and here depends
		 * on readers being already gone) AND the straddlers of the two
		 * moved runs.  Then finalize each side's adopted run from its
		 * NULL-terminated transient back to circular form: dst now owns
		 * swap's old run (point its outer links at dst's sentinel), swap
		 * now owns dst's old run (point at swap's sentinel).  Reading the
		 * exclusive flags here, before the swap_ft->exclusive update below,
		 * preserves the original drain condition.
		 */
		if (!swap_ft->exclusive || !dst_ft->exclusive)
			ft_writer_lock_gp_wait(dst_ft);
		if (dst_ft->group->ordered_list_set) {
			ft_ord_finalize_circular(dst_ft);
			ft_ord_finalize_circular(swap_ft);
		}

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
		enum ft_graft_swap_case kase;
		struct cds_ft_metadata *pmeta, *swap_rmeta;
		struct cds_ft_inode_flag *old_child, *old_swap_root;
		struct cds_ft_inode *fresh = NULL;
		struct cds_ft_metadata *fresh_meta = NULL;
		struct ft_flip_txn *swap_retire_txn = NULL;
		struct ft_flip_txn *extract_txn = NULL;
		struct ft_flip_txn *glue_publish_txn = NULL;
		struct ft_flip_txn *run_replace_txn = NULL;
		struct ft_glue glue_insert, glue_extract;
		struct cds_ft_inode_flag *canon = NULL;
		struct cds_ft_inode_flag *top_B = NULL;	/* extracted swap root, NULL = external/none */
		struct cds_ft_compressed_node *ks_cn = NULL;	/* key-shorter original cn */
		/*
		 * The RAW graft-point slot value this attempt PLANNED against, as
		 * the DESCENT read it (ft_graft_swap_descend's @raw_ret) -- the
		 * displaced occupant the extract side is about to re-root into
		 * @swap_ft.  It becomes the forward publish's expected-old
		 * (ft_glue_set_publish_old), so a peer that swaps the graft point out
		 * from under this attempt makes the commit ABORT instead of ratifying
		 * a world this op never saw.  Raw, not @d.nf: the descent RESOLVES a
		 * skip-compressed occupant, and the expected-old is compared against
		 * the slot.
		 */
		struct cds_ft_inode_flag *gs_pub_old = NULL;
		bool swap_empty;
		bool old_child_external = false;
		bool have_insert = false;
		unsigned long old_count = 0, swap_count;
		/* run_D = dst's subtree-at-key heads; run_S = swap's whole list. */
		struct ft_ord_cell *gs_d_first = NULL, *gs_d_last = NULL;
		struct ft_ord_cell *gs_s_first = NULL, *gs_s_last = NULL;
		bool gs_ord = dst_ft->group->ordered_list_set;
		/*
		 * Empty-swap (remove) where the graft point is its parent's SOLE
		 * child: publishing NULL would leave the parent a childless (invalid)
		 * node -- a compressed node, or a single-child internal.  Prune it via
		 * ft_detach_node (move-style: preserves the displaced subtree for the
		 * extract side, frees the parent + the single-child chain).  The prune
		 * can recompact a surviving ancestor, so a reserve is filled in the
		 * fallible prep and the detach draws from it -- keeping the commit
		 * failure-free.
		 */
		struct cds_ft_alloc_reserve gs_reserve;
		bool gs_reserved = false;
		bool empty_pruned = false;
		bool gs_rlock = false;
#ifdef FEATURE_FT_PROBE_GRAFT_SWAP
		/* What this attempt published into the graft-point slot. */
		struct cds_ft_inode_flag *gs_top = NULL;
		bool gs_fuse_incoh = false;
#endif

		/*
		 * Read-only descent: nothing is published, so the whole swap can be
		 * assembled as a build-invisible transaction and an allocation failure
		 * leaves both tries pristine.
		 */
		/*
		 * §11 cross-trie RCU-pinning: this descent captures live-dst spine
		 * nodes (d.pnf ...) that the extract-side detach and the insert-side
		 * publish-replace node lock below.  A node lock rejects a
		 * relocated-but-live node but NOT a reclaimed+recycled one (the arena
		 * re-zeroes metadata on realloc -> false-success -> wild store), so
		 * pin the captured nodes with the flavor read side across
		 * descent->lock, exactly as cds_ft_graft does.  SCOPED, not whole-op:
		 * graft_swap drains dst readers with ft_writer_lock_gp_wait(dst_ft)
		 * before its extract-side root install, and a grace period under a
		 * read section self-deadlocks -- so the section is RELEASED just
		 * before that dst drain (every descent-captured dst node is LOCK-
		 * locked by then).  The consumed @swap_ft is exclusive (BUSY_ERROR
		 * otherwise), so its own ft_writer_lock_gp_wait is !exclusive-gated
		 * and skipped, and nothing else synchronizes inside the section.
		 */
		if (dst_ft->lock_fine && swap_ft->exclusive) {
			dst_ft->group->flavor->read_lock();
			gs_rlock = true;
		}
	unsigned long rs_depth __attribute__((unused)) = 0;

	/*
	 * ESCALATION LANE for retry_swap.  Without a persistent handle this loop
	 * ages nothing: every attempt commits through its own ft_flip_txn, so
	 * urcu_txn_conflict() is never called, txn->retry never advances and
	 * urcu_txn__self_qualifies() is never reached -- and a standalone handle
	 * carries no domain anyway.  A contended writer then spins with no
	 * termination argument instead of taking its FIFO turn.
	 *
	 * SCOPED to the contended region, not the whole op: all four retry edges
	 * are above the dst drain, and the bracket is closed before it -- see the
	 * two close sites.
	 *
	 * @optxn is NEVER bound into a commit (they all use standalone
	 * ft_flip_txn_create_bounded), so optxn->desc stays NULL and
	 * urcu_txn_end() never reaches urcu_txn_destroy(): the double free that
	 * reverted an earlier attempt at this pattern is designed out, and the
	 * grep "optxn appears only in init/begin/end/bail" is the invariant.
	 *
	 * @gs_open is a VARIABLE, not a re-test of @gs_bracket at each exit: a
	 * re-evaluated condition is a second chance to disagree with the entry.
	 */
	const bool gs_bracket = dst_ft->lock_fine && swap_ft->exclusive;
	struct urcu_txn optxn;
	bool gs_open = false;

	if (gs_bracket)
		ft_txn_op_init(dst_ft, &optxn);
retry_swap:
	if (gs_bracket) {
		urcu_txn_begin(&optxn);
		gs_open = true;
	}
	/*
	 * ASK @gs_bracket, not the expression it was built from.  The bracket
	 * below keys on the const bool precisely so the condition is evaluated
	 * ONCE (see @gs_open's comment above); a probe that re-derives it is a
	 * second chance to disagree with the arm it claims to be measuring.
	 */
	RSPIN_ENTER_X(4, rs_depth, 2, gs_bracket);
	RSPIN_SITE_ENTER(2, rs_depth, gs_bracket);
		/*
		 * MW LOCK_FINE drop: the re-descend point.  graft_swap's commit is
		 * failure-free under the FT-wide lock, but with the lock dropped a
		 * peer can relocate the contended dst spine between this op's descent
		 * and its commit -- the empty-swap prune's ft_detach_node then cannot
		 * lock the recompacting ancestor and returns -EAGAIN (build-invisible:
		 * nothing published, dst byte-for-byte as before).  Re-plan against the
		 * settled tree, mirroring ft_graft_keylen's retry_attach and
		 * ft_merge_graft_subpos_inplace's retry_merge.  Reset every per-attempt
		 * build-product handle: the declaration initializers ran once, and a
		 * goto here does not re-run them.  The §11 RCU pin stays held across
		 * attempts (a re-descend under the same read section re-pins).
		 */
		fresh = NULL;
		fresh_meta = NULL;
		swap_retire_txn = NULL;
		extract_txn = NULL;
		glue_publish_txn = NULL;
		run_replace_txn = NULL;
		canon = NULL;
		top_B = NULL;
		ks_cn = NULL;
		gs_pub_old = NULL;
		old_child_external = false;
		have_insert = false;
		empty_pruned = false;
		gs_reserved = false;
		gs_d_first = gs_d_last = gs_s_first = gs_s_last = NULL;
		kase = ft_graft_swap_descend(dst_ft, key, key_len, &d,
				&gs_pub_old);
#ifdef FEATURE_FT_PROBE_GRAFT_SWAP
		gs_top = NULL;
		gs_fuse_incoh = false;
		if (kase == FT_GRAFT_SWAP_EXACT)
			FT_GS_PROBE_INC(cds_ft_probe_gs_exact);
		else if (kase == FT_GRAFT_SWAP_KEY_SHORTER)
			FT_GS_PROBE_INC(cds_ft_probe_gs_kshort);
		else
			FT_GS_PROBE_INC(cds_ft_probe_gs_delegate);
#endif
		/*
		 * The descent could not settle a coherent (raw slot, resolved
		 * occupant) pair -- a peer moved the graft point under it, or a
		 * chain-merge reanchored the captured slot to another level
		 * (ft_graft_swap_settle / ft_descent_step both raise this).  A
		 * pure read-only descent has published and allocated NOTHING at
		 * this point, so re-plan against the settled tree: the same bail
		 * cds_ft_graft takes (retry_attach, :1599) and insert takes
		 * (ft-insert.h skip_conflict), which cds_ft_graft_swap had never
		 * honoured.
		 */
		if (caa_unlikely(d.skip_conflict)) {
			FT_GS_PROBE_INC(cds_ft_probe_gs_retry);
			ft_txn_attempt_bail(&optxn, gs_open);
			gs_open = false;
			goto retry_swap;
		}
		if (kase == FT_GRAFT_SWAP_DELEGATE) {
			/*
			 * No content at @key: the swap reduces to inserting swap_ft's
			 * content at @key, which empties swap_ft.  cds_ft_graft is itself
			 * a build-invisible transaction and empties the source.
			 * Pass the ORIGINAL application key: cds_ft_graft applies
			 * the key map itself, and the already-remapped @key would
			 * be remapped twice on a non-identity group (wrong graft
			 * point, wrong splice position).
			 */
			enum cds_ft_status s = cds_ft_graft(dst_ft, _key, _key_len,
					swap_ft);

			if (gs_rlock)
				dst_ft->group->flavor->read_unlock();
			FT_TP(graft_swap_exit, (int) s);
			ft_txn_attempt_end(&optxn, gs_open);
			gs_open = false;
			return s;
		}

		old_swap_root = swap_ft->root;
#ifdef FEATURE_FT_PROBE_GRAFT_SWAP
		if (old_swap_root && old_swap_root == gs_pub_old)
			FT_GS_PROBE_INC(cds_ft_probe_gs_alias);
#endif
		swap_rmeta = ft_root_metadata(swap_ft);
		swap_empty = (ft_meta_nr_child(swap_rmeta) == 0 && !swap_rmeta->external_nodes);
		swap_count = swap_empty ? 0 : ft_nr_keys_get(swap_rmeta);

		/*
		 * Identify the displaced old-child and its key count.  KEY_SHORTER: the
		 * extracted subtree is everything below the prefix, i.e. the suffix of
		 * the compressed node d.nf (its whole subtree count).  EXACT: d.nf is
		 * the displaced node.
		 */
		if (kase == FT_GRAFT_SWAP_KEY_SHORTER) {
			ks_cn = ft_compressed_node_ptr(d.nf);
			old_count = ft_nr_keys_get(
				cds_ft_item_to_metadata((struct cds_ft_inode *) ks_cn));
			old_child = ks_cn->child;
		} else {	/* FT_GRAFT_SWAP_EXACT */
			old_child = d.nf;
			if (!ft_node_external(old_child))
				old_count = ft_nr_keys_get(
					cds_ft_item_to_metadata(ft_node_ptr(old_child)));
			else
				old_count = 1;	/* one key (possibly a dup chain) */
			/*
			 * Skip-encoded externals carry a compressed prefix; treat them as
			 * non-external so the prefix is materialized into swap_ft's root.
			 */
			old_child_external = ft_node_external(old_child) &&
				!ft_node_skip_compressed(old_child);
			if (old_child_external)
				FT_GS_PROBE_INC(cds_ft_probe_gs_ext_child);
		}

		ft_glue_init(&glue_insert);
		glue_insert.op = &optxn;
		ft_glue_init(&glue_extract);
		glue_extract.op = &optxn;
		/*
		 * @d is the DST graft-point descent, so it dates the insert glue's
		 * publish parent (that glue's whole cluster hangs off the graft
		 * point).  The extract glue works the SWAP trie, which this descent
		 * does not describe, so it gets none -- see its own commit.
		 */
		glue_insert.lock_d = &d;

		/* ===== PREP: build clusters A and B (both tries pristine) ===== */

		/*
		 * Empty-swap remove: the COMMIT routes it through ft_detach_node
		 * (correct parent bookkeeping for every shape -- sole-child prune,
		 * multi-child in-place delete, recompaction).  Secure a node reserve
		 * for any recompaction the detach may do, build-invisibly (an OOM
		 * here leaves both tries pristine), so the commit detach cannot fail.
		 */
		if (swap_empty) {
			memset(&gs_reserve, 0, sizeof(gs_reserve));
			if (ft_bulk_node_reserve_fill(dst_ft, &gs_reserve)) {
				cds_ft_alloc_reserve_drain(dst_ft, &gs_reserve);
				goto prep_oom;
			}
			gs_reserved = true;
		}

		/*
		 * Insert side (cluster A): canonicalized swap content, placed at the
		 * graft point in dst.  Empty swap inserts nothing (a remove).
		 */
		if (!swap_empty) {
			canon = ft_compress_single_child_if_needed(dst_ft,
				old_swap_root, &glue_insert);
			if (canon == (struct cds_ft_inode_flag *) (long) -ENOMEM)
				goto prep_oom;
		}

#ifdef FEATURE_FT_SKIP_COMPRESSED
		{
			/*
			 * EXACT + compressed parent + compressed canon: the slot already
			 * sits under a compressed node, so placing another compressed
			 * there would violate "no two adjacent compresseds".  Fuse them
			 * into one compressed at the grandparent slot.  Build-invisible:
			 * the merged cn's live child (canon's grandchild) is deferred,
			 * @canon (a fresh absorbed wrapper) is freed now, and the live
			 * parent cn is reclaimed at commit.
			 */
			struct cds_ft_compressed_node *pcn = NULL, *ccn = NULL;

			if (!swap_empty && kase == FT_GRAFT_SWAP_EXACT) {
				if (ft_node_skip_compressed(d.pnf))
					pcn = ft_skip_to_compressed(dst_ft, d.pnf);
				else if (ft_node_compressed(d.pnf))
					pcn = ft_compressed_node_ptr(d.pnf);
				if (ft_node_compressed(canon))
					ccn = ft_compressed_node_ptr(canon);
			}
#ifdef FEATURE_FT_PROBE_GRAFT_SWAP
			if (!swap_empty && kase == FT_GRAFT_SWAP_EXACT) {
				if (pcn) FT_GS_PROBE_INC(cds_ft_probe_gs_fuse_pcn);
				if (ccn) FT_GS_PROBE_INC(cds_ft_probe_gs_fuse_ccn);
				if (pcn && ccn && (unsigned int) pcn->len + ccn->len
						<= FT_SKIP_LEN_MAX)
					FT_GS_PROBE_INC(cds_ft_probe_gs_fuse_len);
			}
#endif
			if (pcn && ccn &&
			    (unsigned int) pcn->len + ccn->len <= FT_SKIP_LEN_MAX) {
				struct cds_ft_metadata *pcn_meta =
					cds_ft_item_to_metadata((struct cds_ft_inode *) pcn);
				unsigned int merged_len = pcn->len + ccn->len;
				struct cds_ft_compressed_node *merged;
				struct cds_ft_metadata *merged_meta;
				struct cds_ft_inode_flag *merged_flag, *merged_skip;
				struct cds_ft_inode_flag **pub_slot;
				struct cds_ft_inode_flag *pub_parent, *pub_old;

				merged = alloc_compressed_node(dst_ft, merged_len,
						&merged_meta);
				if (!merged)
					goto prep_oom;
				memcpy(merged->key_bytes, pcn->key_bytes, pcn->len);
				memcpy(&merged->key_bytes[pcn->len], ccn->key_bytes,
					ccn->len);
				merged->len = (uint8_t) merged_len;
				merged->child = ccn->child;	/* live swap grandchild */
				ft_meta_nr_child_set(merged_meta, 1);
				/*
				 * @merged now roots the swap content (child == ccn->child), so
				 * it carries @swap_count -- baked build-invisible.  The dst NET
				 * delta (swap_count - old_count) folds onto @glue_insert's
				 * publish txn from @pub_parent (the grandparent) below.
				 */
				ft_nr_keys_store(swap_ft, merged_meta,
					swap_count, CMM_RELAXED);
				/*
				 * Resolve pcn's grandparent slot COHERENTLY (parent +
				 * offset from one snapshot) rather than reading
				 * pcn_meta->parent raw and fetching the slot separately: a
				 * peer that recompacts pcn's grandparent to a larger node
				 * type (or Phase-4.3 re-homes pcn) between the two reads
				 * would leave pub_parent naming the OLD grandparent while
				 * pub_slot indexes the NEW body, so ft_set_parent_slot ->
				 * ft_slot_to_byte indexes the wrong body out of bounds and
				 * bakes a wild offset into the build-invisible @merged.
				 * The forward publish below (ft_glue_set_publish) reuses
				 * the same coherent (pub_parent, pub_slot) and the commit
				 * guard_parent ratifies pub_parent at the flip.
				 */
				pub_slot = ft_resolve_parent_slot(pcn_meta, dst_ft,
					&pub_parent);
				/*
				 * The plan value for THIS slot (the grandparent's,
				 * not @d.nfp's): @merged was built from @pcn's bytes
				 * read a few lines up, so the occupant read here is
				 * the world the fuse planned against.
				 */
				/* SETTLED, not raw: a parked engine proxy is not a
				 * value this slot has (see ft_graft_swap_settle). */
				pub_old = ft_resolve_flip_proxy(*pub_slot);
				/*
				 * ★ THE EXPECTED-OLD CANNOT ARBITRATE THIS ON ITS OWN.
				 * @merged encodes @pcn's bytes and length, read above;
				 * @pub_old is a LATER, independent read of the
				 * grandparent slot.  A peer that replaces @pcn between
				 * the two leaves them describing different worlds -- and
				 * the commit still SUCCEEDS, because @pub_old was read
				 * from the peer's world and therefore matches the slot.
				 * What lands is @merged, encoding bytes that are already
				 * stale, over the peer's node.
				 *
				 * So settle the pair here: the slot must still denote the
				 * @pcn the fuse planned against.  If it does not, nothing
				 * is published yet (@merged is untracked and unpublished,
				 * both glues hold only build-invisible state), so drop
				 * this attempt and re-descend -- the same bail
				 * ft_graft_swap_settle takes for the graft point itself.
				 */
				if (caa_unlikely(ft_resolve_skip_compressed(dst_ft,
						pub_old)
						!= ft_compressed_node_flag(pcn))) {
					FT_GS_PROBE_INC(cds_ft_probe_gs_fuse_incoh);
#ifdef FEATURE_FT_PROBE_GRAFT_SWAP
					gs_fuse_incoh = true;
#endif
					free_compressed_node_unpublished(dst_ft, merged);
					ft_glue_abort(dst_ft, &glue_insert);
					ft_glue_abort(swap_ft, &glue_extract);
					FT_GS_PROBE_INC(cds_ft_probe_gs_retry);
					ft_txn_attempt_bail(&optxn, gs_open);
					gs_open = false;
					goto retry_swap;
				}
				merged_meta->parent_word = ft_parent_word(dst_ft, pub_parent);
				ft_set_parent_slot(merged_meta, pub_parent, pub_slot);
				merged_flag = ft_compressed_node_flag(merged);
				ft_glue_track(&glue_insert, merged_flag);
				ft_glue_defer_edge(dst_ft, &glue_insert, ccn->child,
					merged_flag, &merged->child);
				/*
				 * @canon is the fresh wrapper just absorbed: drop it from
				 * tracking and free it (its deferred child edge is superseded
				 * by the one above via the defer-edge de-dup on @child).
				 */
				ft_glue_untrack(dst_ft, &glue_insert, ccn);
				free_compressed_node_unpublished(dst_ft, ccn);
				merged_skip = ft_publish_compressed(dst_ft, merged,
						merged_flag);
				ft_glue_set_publish_old(dst_ft, &glue_insert,
					pub_parent, pub_slot, merged_skip, pub_old);
				ft_glue_defer_free(&glue_insert, pcn, true);
				d.pnf = merged_flag;	/* structural edits target @merged */
				have_insert = true;
#ifdef FEATURE_FT_PROBE_GRAFT_SWAP
				gs_top = merged_skip;
				FT_GS_PROBE_INC(cds_ft_probe_gs_fused);
#endif
			}
		}
#endif /* FEATURE_FT_SKIP_COMPRESSED */

		if (!swap_empty && !have_insert) {
			struct cds_ft_inode_flag *top_A;

			if (kase == FT_GRAFT_SWAP_KEY_SHORTER) {
				/*
				 * Replace the whole compressed node with a fresh prefix
				 * [d.depth, key_len) wrapping @canon (the chain-merge folds the
				 * prefix bytes into @canon when it is compressed).  d.pnf is the
				 * cn's parent (never compressed), so no grandparent fuse.
				 */
				top_A = ft_build_branch(dst_ft, key, d.depth, key_len,
						canon, swap_count, false, &glue_insert);
				if (!top_A)
					goto prep_oom;
				ft_glue_defer_free(&glue_insert, ks_cn, true);
			} else {
				/* EXACT, simple replace of d.nf at d.nfp by @canon. */
				top_A = canon;
			}
			/*
			 * A compressed cluster top installs as the SKIP form in the live
			 * parent slot; its child's back-pointer is deferred, so the skip
			 * only resolves once ft_glue_apply_deferred has run -- which
			 * it does (before the forward publish) at commit.  The set_publish
			 * deferred edge (top -> d.pnf) is recorded LAST, so by the time it
			 * is applied the child back-pointer is already in place.
			 */
			if (ft_node_compressed(top_A))
				top_A = ft_publish_compressed(dst_ft,
					ft_compressed_node_ptr(top_A), top_A);
			ft_glue_set_publish_old(dst_ft, &glue_insert, d.pnf, d.nfp,
				top_A, gs_pub_old);
			have_insert = true;
#ifdef FEATURE_FT_PROBE_GRAFT_SWAP
			gs_top = top_A;
			if (top_A == gs_pub_old)
				FT_GS_PROBE_INC(cds_ft_probe_gs_canon_alias);
#endif
		}

		/*
		 * Extract side (cluster B): materialize the displaced subtree as
		 * swap_ft's new root.  KEY_SHORTER builds the root from the suffix path
		 * + live grandchild; EXACT runs the build-invisible make_root_internal
		 * on the displaced node.  External (or absent) content attaches as
		 * external_nodes at commit instead (no build).
		 */
		if (kase == FT_GRAFT_SWAP_KEY_SHORTER) {
			unsigned int prefix_len = (unsigned int) (key_len - d.depth);

			top_B = ft_build_extracted_root_glue(swap_ft, &glue_extract,
				ks_cn->key_bytes[prefix_len],
				&ks_cn->key_bytes[prefix_len + 1],
				ks_cn->len - prefix_len - 1,
				ks_cn->child, old_count);
			if (top_B == (struct cds_ft_inode_flag *) (long) -ENOMEM)
				goto prep_oom;
		} else if (!old_child_external && old_child) {
			top_B = ft_make_root_internal_glue(swap_ft, &glue_extract,
					old_child);
			if (top_B == (struct cds_ft_inode_flag *) (long) -ENOMEM)
				goto prep_oom;
		}

		/*
		 * Insert side: commit the replace through a flip-txn following the
		 * bulk-op rule -- a pointer NOT reader-observable during the commit
		 * window is set immediately with a plain store, only a LIVE publish
		 * rides the txn.  Here the whole inserted cluster is the swap content,
		 * unlinked + drained below, so all its back-pointers are hidden:
		 * ft_glue_txn_commit_replace sets them immediately (ft_glue_apply_
		 * deferred) and rides only the forward replace edge into dst + the <=4
		 * run-replace cell edges on the txn, so structure and ordered list flip
		 * together in ONE selector flip (closing the cross-view window).
		 * Reserve generous headroom (forward 1-2 stores + <=4 cell edges, plus
		 * the deferred floor) up front so records can't fail mid-commit;
		 * created before the failure-free section so an OOM here is still a
		 * clean prep_oom.  The extract side stays on plain rcu_assign: it
		 * publishes into the drained swap_ft where no reader is present.
		 *
		 * NOT for KEY_SHORTER (the swap key ends INSIDE a compressed node):
		 * that shape wraps a LIVE dst compressed node whose re-parent is a live
		 * pointer not yet classified for the txn, so its EDGE commit stays on
		 * the legacy apply-deferred + publish-replace path (glue_insert.txn ==
		 * NULL selects it) until that wrap re-parent is folded in.  Its
		 * free-list FREEZE still fuses -- into glue_publish_txn, wired up in
		 * the commit below (doc §4.B).
		 */
		if (have_insert && kase != FT_GRAFT_SWAP_KEY_SHORTER) {
			glue_insert.txn = ft_flip_txn_create(dst_ft);
			if (!glue_insert.txn || !ft_flip_txn_reserve(glue_insert.txn,
					/* +1: fused recompact-relocate tombstone (§4.B);
					 * + FLOOR_FREE: fused free-list tombstones */
					FT_GLUE_FLOOR_DEFERRED + 7 + 1 /* +1 §4.B parent guard */ + FT_GLUE_FLOOR_FREE
					+ (dst_ft->rank_stats ? (int) key_len + 1 : 0) /* nr_keys fold walk */))
				goto prep_oom;
			glue_insert.fuse_free_list = true;
		} else if (have_insert) {
			/*
			 * KEY_SHORTER legacy publish (glue_insert.txn stays NULL here ->
			 * ft_glue_apply_deferred + ft_glue_publish_replace below): the
			 * publish-replace runs in the failure-free section, so pre-reserve
			 * its forward(+SKIP_X dual) + run-replace commit txn here, where an
			 * OOM is still a clean prep_oom.  It is always consumed on the
			 * legacy path (the reservation condition mirrors the consume
			 * condition); a later prep_oom frees it.  The +FT_GLUE_FLOOR_FREE
			 * headroom lets the free-list FREEZE fuse into this same flip
			 * (doc §4.B atomic detach): glue_insert's retired-node tombstones
			 * ride the publish-replace flip that unlinks them (wired up in the
			 * commit below) instead of a standalone lone-edge flip.  Only the
			 * free-list freeze fuses; the live wrap re-parent stays on the
			 * immediate apply_deferred path (not yet txn-classified).
			 */
			glue_publish_txn = ft_flip_txn_create_bounded(dst_ft,
				FT_GLUE_PUBLISH_REPLACE_MAX_EDGES + FT_GLUE_FLOOR_FREE
				+ 1 /* +1 §4.B parent guard */
				+ (dst_ft->rank_stats ? (int) key_len + 1 : 0) /* nr_keys fold walk */);
			if (!glue_publish_txn)
				goto prep_oom;
		}

		/* Transient empty swap root for the unlink window (fallible). */
		if (!swap_empty) {
			fresh = alloc_cds_ft_node(swap_ft, &ft_types[0], &fresh_meta);
			if (!fresh)
				goto prep_oom;
			/* Fresh root for @swap_ft: named while still invisible. */
			fresh_meta->parent_word = ft_trie_parent(swap_ft);
			/*
			 * Pre-reserve the swap-root retire's txn -- a fallible step
			 * before the failure-free commit.  The retire below runs past
			 * the COMMIT marker and cannot abort, so it commits through this
			 * pre-reserved txn (ft_ord_cell_flip_into).  Reserved only when
			 * the list is on (list off retires via the lone-edge
			 * ft_root_edge_flip).  OOM here is a clean pre-commit abort; a
			 * later prep_oom (e.g. the extract-txn reserve below failing)
			 * frees it.
			 */
			if (gs_ord) {
				swap_retire_txn = ft_flip_txn_create_bounded(swap_ft,
					FT_ROOT_LIST_SWAP_MAX_EDGES);
				if (!swap_retire_txn)
					goto prep_oom;
			}
		}

		/*
		 * Pre-reserve the extract-side run_D install txn (the post-drain
		 * fused root-install + run_D head/tail flip far below): it commits
		 * after both syncs, in the failure-free section, so it cannot abort.
		 * Reserved only when the list is on -- list off makes that flip at
		 * most the lone top_B root-install edge (an infallible on-stack
		 * store).  Plus the standalone dst run-replace fallback txn (the
		 * gs_reserved sole-child detach-prune path, where the publish leaves
		 * swap_run unarmed; every other shape fuses the replace and frees it
		 * unused).  These are the last fallible steps before the COMMIT marker.
		 */
		/*
		 * extract_txn carries the swap-root install: list-on rides the full
		 * pre-reserved txn (root install + run_D head/tail + the fused top_B
		 * transient-root tombstone + the fused free-list tombstone); list-off
		 * with a top_B retire needs a 3-edge txn (root install + top_B
		 * tombstone + free-list tombstone, atomic detach §4.B) rather than a
		 * lone store.  !gs_ord && !top_B keeps its lone external attach store.
		 * One shared goto keeps this block's abort footprint unchanged.
		 *
		 * Fuse the extract-side glue free-list retire into extract_txn: the
		 * one node ft_make_root_internal_glue defers (the peeled compressed
		 * old_child, <= 1) freezes dead atomically with the root install that
		 * unlinks it (atomic detach, §4.B), instead of a standalone flip in
		 * ft_glue_apply_deferred below.  The +1 headroom in each arm covers it.
		 * A non-empty free_list means a compressed old_child, hence top_B is
		 * non-NULL, hence extract_txn is this reserved txn -- never the
		 * !gs_ord && !top_B NULL case, where the free_list is empty and
		 * ft_glue_tombstone_free_list's loop is skipped (no NULL-txn deref).
		 */
		if (gs_ord || top_B) {
			extract_txn = ft_flip_txn_create_bounded(swap_ft, gs_ord ?
				FT_ROOT_LIST_SWAP_MAX_EDGES + 2 : 3);
			if (!extract_txn)
				goto prep_oom;
			glue_extract.txn = extract_txn;
			glue_extract.fuse_free_list = true;
			if (gs_ord) {
				run_replace_txn = ft_flip_txn_create_bounded(dst_ft,
					FT_ORD_CELL_RUN_REPLACE_MAX_EDGES);
				if (!run_replace_txn)
					goto prep_oom;
			}
		}

		/* ===== COMMIT (failure-free) ===== */

		/*
		 * Ordered list: capture both runs while both lists are intact.
		 * run_D = dst's subtree-at-key (old_child's heads), which becomes
		 * swap_ft's whole list; run_S = swap_ft's whole list, which replaces
		 * run_D in dst.  The mutations land at the matching structural
		 * sub-points below so the existing per-side syncs drain each side.
		 */
		if (gs_ord) {
			gs_d_first = ft_ord_cell_ptr(rcu_dereference(
				ft_subtree_minmax_head(dst_ft, old_child, false)->prev));
			gs_d_last = ft_ord_cell_ptr(rcu_dereference(
				ft_subtree_minmax_head(dst_ft, old_child, true)->prev));
			gs_s_first = ft_ord_first(swap_ft);	/* NULL if swap empty */
			gs_s_last = ft_ord_last(swap_ft);
		}
		/*
		 * Fuse the run_D->run_S ordered-list replace into the SAME flip as
		 * the structural publish below, so a reader never sees run_S present
		 * in the structure at @key but the ordered list still showing run_D
		 * (or run_D the reverse).  Armed by the publish shapes that record
		 * their forward edge (have_insert glue / empty-swap publish-NULL);
		 * the sole-child detach-prune (gs_reserved) leaves it unarmed and
		 * falls back to the standalone two-commit run-replace below.
		 */
		struct ft_graft_swap_run swap_run = {
			gs_d_first, gs_d_last, gs_s_first, gs_s_last, false
		};
		struct ft_graft_swap_run *swap_run_arg = gs_ord ? &swap_run : NULL;

		/*
		 * Insert side: wire the deferred live back-pointers, then the single
		 * forward publish that splices cluster A into dst (detaching the old
		 * content).  Empty swap publishes NULL (a remove).
		 *
		 * ★ THE DST ATTACH IS THE POINT OF NO RETURN, AND IT RUNS FIRST.  The
		 * swap-root retire used to precede it, on a "jump out" argument that
		 * required draining swap's readers between the unlink and the
		 * re-parent.  That drain has been !exclusive-gated ever since a LIVE
		 * @swap_ft became a BUSY rejection at entry, i.e. it is dead code: the
		 * consumed source is ALWAYS exclusive here, so it has no readers to
		 * jump out and no ordering to honour.  What the old order did cost was
		 * the only thing that mattered -- a commit that ABORTS (a peer touched
		 * the contended dst spine) left swap already emptied, so the abort was
		 * unrecoverable and its status was DROPPED.  Retiring AFTER the attach
		 * makes every abort build-invisible on both sides, which is what lets
		 * the status be honoured with a plain re-descend.
		 */
		if (have_insert) {
			enum urcu_txn_status ins_cst = URCU_TXN_STATUS_OK;

			/*
			 * Order-statistics (BULK): the replace swaps @old_count keys for
			 * @swap_count, so the dst NET delta rides the replace publish
			 * (ft_glue_txn_commit_edges / ft_glue_publish_replace record it
			 * from @glue_insert.publish_parent) instead of a post-commit walk.
			 */
			glue_insert.count_delta = (long) swap_count - (long) old_count;
			if (glue_insert.txn)
				ins_cst = ft_glue_txn_commit_replace(dst_ft,
					&glue_insert, swap_run_arg);
			else {
				/*
				 * Fuse glue_insert's free-list FREEZE into the publish-replace
				 * flip (doc §4.B atomic detach).  Point glue_insert at
				 * glue_publish_txn HERE -- AFTER the glue_insert.txn dispatch
				 * above already chose this legacy branch -- so
				 * ft_glue_apply_deferred records the retired-node tombstones
				 * into it and ft_glue_publish_replace commits them with the
				 * forward replace that unlinks them.  The live wrap re-parent
				 * is still applied immediately by apply_deferred (a hidden,
				 * drained back-edge); only the freeze rides the txn.
				 */
				glue_insert.txn = glue_publish_txn;
				glue_insert.fuse_free_list = true;
				ft_glue_apply_deferred(dst_ft, &glue_insert);
				/*
				 * ★ HONOUR THIS STATUS.  This is the same point of no
				 * return as the fused arm above, and it aborts for the
				 * same reason -- a peer took the graft point.  Ignoring
				 * it would retire @swap_ft's root, re-root the
				 * displaced subtree into it and FREE the dst node the
				 * publish never replaced, leaving a live dst grandchild
				 * whose parent points into the OTHER trie.  Route it
				 * into the same unwind: the abort is build-invisible on
				 * dst (nothing published) and on @swap_ft (still holds
				 * its content; apply_deferred rewrote only its own
				 * exclusive interior), so a plain re-descend is clean.
				 */
				ins_cst = ft_glue_publish_replace(dst_ft,
					glue_publish_txn, &glue_insert,
					swap_run_arg);
				glue_publish_txn = NULL;	/* consumed */
				glue_insert.txn = NULL;	/* the commit reclaimed it */
			}
			/*
			 * MW LOCK_FINE drop: a peer touched the contended dst spine between
			 * this op's descent and its replace commit -- it relocated
			 * @publish_parent (ft_flip_txn_guard_parent's clean-live expectation
			 * fails against the retired/re-homed parent word) or it swapped the
			 * graft point out from under us (the forward record's expected-old
			 * mismatches).  A flip-txn commit is all-or-none, and with the
			 * swap-root retire now BELOW this point NOTHING has been published on
			 * either side: dst is byte-for-byte as before and @swap_ft still holds
			 * its full content.  The replace's apply_deferred did already rewrite
			 * swap's interior parent back-pointers (immediate stores, not rolled
			 * back), which is invisible -- swap is exclusive, the content nodes are
			 * never freed, and the rebuild reads forward-only and overwrites them.
			 * Free this attempt's build-invisible clusters + reserve + txns, then
			 * re-descend (mirrors ft_graft_keylen's store-abort retry_attach).
			 *
			 * ★ EVERY SHAPE MUST HONOUR @ins_cst, not just the fused arm.
			 * Two graft_swaps at ONE dst position can both reach here with
			 * only one attach landed; a loser that ignores the status still
			 * runs its extract side, re-rooting into @swap_ft a subtree the
			 * abort left wired into dst and NULLing that subtree's parent
			 * back-pointer where it still hangs at depth 1 of the
			 * destination -- inv_graft_swap_shared_dst's "parent mismatch:
			 * got (nil)".
			 */
			if (ins_cst != URCU_TXN_STATUS_OK) {
				ft_glue_abort(dst_ft, &glue_insert);
				ft_glue_abort(swap_ft, &glue_extract);
				if (fresh) {
					/*
					 * UNPUBLISHED: the retire that would have
					 * installed @fresh as swap's root is below
					 * this bail, so it never became reader-visible
					 * and carries no tombstone -- the audit build's
					 * freeze-on-free assert is exactly right to
					 * refuse it through the published path.
					 */
					free_cds_ft_node_unpublished(swap_ft, fresh);
					fresh = NULL;
				}
				if (swap_retire_txn) {
					ft_flip_txn_destroy(swap_retire_txn);
					swap_retire_txn = NULL;
				}
				if (extract_txn) {
					ft_flip_txn_destroy(extract_txn);
					extract_txn = NULL;
				}
				if (run_replace_txn) {
					ft_flip_txn_destroy(run_replace_txn);
					run_replace_txn = NULL;
				}
				if (gs_reserved) {
					cds_ft_alloc_reserve_drain(dst_ft, &gs_reserve);
					gs_reserved = false;
				}
				FT_GS_PROBE_INC(cds_ft_probe_gs_retry);
				ft_txn_attempt_bail(&optxn, gs_open);
				gs_open = false;
				goto retry_swap;
			}

#ifdef FEATURE_FT_PROBE_GRAFT_SWAP
			/*
			 * B4 residual probe.  Our commit reported OK, so the graft
			 * point must no longer hold @gs_pub_old -- the occupant the
			 * extract side below is about to re-root into @swap_ft (and
			 * whose parent back-pointer it NULLs).  Re-descend and check.
			 * Still inside the §11 read section, so nothing read here can
			 * have been reclaimed out from under the comparison.
			 */
			{
				struct ft_descent d2;
				struct cds_ft_inode_flag *raw2 = NULL;
				enum ft_graft_swap_case k2;

				FT_GS_PROBE_INC(cds_ft_probe_gs_commit_ok);
				if (gs_fuse_incoh)
					FT_GS_PROBE_INC(
						cds_ft_probe_gs_fuse_incoh_committed);
				k2 = ft_graft_swap_descend(dst_ft, key, key_len,
					&d2, &raw2);
				if (raw2 && raw2 == gs_pub_old) {
					FT_GS_PROBE_INC(cds_ft_probe_gs_reoccupy);
					fprintf(stderr, "GSPROBE reoccupy: kase=%d k2=%d "
						"pub_old=%p published=%p old_child=%p "
						"nfp=%p->%p nfp2=%p->%p pnf=%p pnf2=%p\n",
						(int) kase, (int) k2,
						(void *) gs_pub_old, (void *) gs_top,
						(void *) old_child,
						(void *) d.nfp,
						(void *) (d.nfp ? *d.nfp : NULL),
						(void *) d2.nfp,
						(void *) (d2.nfp ? *d2.nfp : NULL),
						(void *) d.pnf, (void *) d2.pnf);
				} else if (d2.nfp != d.nfp) {
					FT_GS_PROBE_INC(cds_ft_probe_gs_slot_moved);
				}
			}
#endif

			/*
			 * Retire swap's root to an empty node AND (paired) unlink run_S from
			 * swap's ordered list, FUSED in ONE flip.  Past the dst attach's
			 * commit, so this is the failure-free section: the pre-reserved
			 * @swap_retire_txn cannot abort, and @swap_ft is exclusive so the
			 * cross-view window the fusion closes has no observer left anyway --
			 * it is kept because the pairing is the invariant, not the audience.
			 * run_D is installed as swap's list after the extract publish below.
			 * (List off: just the lone root edge.)
			 *
			 * @swap_ft's content is momentarily reachable from BOTH tries here
			 * (dst published it above, swap has not yet let go).  Only a reader of
			 * @swap_ft could see that, and an exclusive trie has none -- the same
			 * premise that lets the replace's apply_deferred re-parent swap's
			 * interior into dst before this point.
			 */
			if (!swap_empty) {
				struct cds_ft_inode_flag *empty =
					ft_node_flag(fresh, 0);

				assert(swap_ft->exclusive);
				if (gs_ord)
					/*
					 * Empty swap's sentinel (relink_dest NULL):
					 * run_S's cells were re-homed into dst by the
					 * run-replace above; run_D is installed as
					 * swap's list after the extract publish.
					 */
					ft_root_list_swap_publish(swap_ft,
						swap_retire_txn, &swap_ft->root,
						swap_ft->root, empty,
						ft_ord_first(swap_ft), NULL,
						ft_ord_last(swap_ft), NULL,
						NULL, false);
				else
					ft_root_edge_flip(swap_ft, &swap_ft->root,
						swap_ft->root, empty);
				swap_retire_txn = NULL;	/* consumed */
				FT_TP(root_publish, (const void *) swap_ft,
					(const void *) swap_ft->root);
			}
		} else {
			/*
			 * Empty-swap remove (a REMOVE of @old_child at @key): route it
			 * through a move-style ft_detach_node so the parent's bookkeeping
			 * is correct for EVERY graft-point shape -- not just a sole-child
			 * prune (compressed / single-child internal: frees @d.pnf and the
			 * chain to a surviving ancestor) but also a multi-child in-place
			 * delete (clears the parent's pigeon bitmap bit / popcount slot,
			 * which a raw ft_publish_to_parent(NULL) does NOT) and any
			 * recompaction a shrinking parent needs.  free_detached_subtree ==
			 * false preserves @old_child for the extract side below.  Propagate
			 * -@old_count FIRST (undercount ordering, while @d.pnf is still
			 * live), then detach from the reserve secured in PREP so it cannot
			 * fail.  The detach owns the parent nr_child + any prune, so the
			 * post-publish nr_child-- / propagate are skipped (empty_pruned).
			 *
			 * Fuse run_D's ordered-list removal into the detach's structural
			 * flip (EXCISE-ONLY ft_detach_run, into == NULL -- run_D is
			 * re-homed to swap_ft below), closing the disappear-side cross-view
			 * window: ft_detach_node arms @drun on whichever commit path it
			 * takes (in-place / recompaction / compressed-external-promote),
			 * @dpub is the gating/arm flag, and swap_run.armed then suppresses
			 * the standalone run-replace.  run_D's heads are read while
			 * @old_child is still intact (before the detach).
			 */
			int dret;
			struct ft_remove_pub dpub = { .armed = false };
			struct ft_detach_run drun = { .into = NULL };

			if (gs_ord) {
				drun.rfirst = ft_subtree_minmax_head(dst_ft, old_child,
						false);
				drun.rlast = ft_subtree_minmax_head(dst_ft, old_child,
						true);
			}
			cds_ft_alloc_reserve_activate(dst_ft, &gs_reserve);
			struct ft_lock_ctx lctx;

			ft_lock_ctx_init(&lctx, &d, NULL, &optxn);
			dret = ft_detach_node(dst_ft, &lctx, d.nfp, d.pnfp, d.depth,
					false, NULL, gs_ord ? &dpub : NULL,
					gs_ord ? &drun : NULL, NULL, NULL,
					-(long) old_count /* fold -old_count onto the detach commit */,
					NULL, false, NULL, NULL);
			cds_ft_alloc_reserve_deactivate(dst_ft);
			/*
			 * MW LOCK_FINE drop: under the FT-wide lock this detach is
			 * failure-free, but with the lock dropped it can fail two ways,
			 * BOTH build-invisible in ft_detach_node (the failure aborts before
			 * any reader-visible store) and this is the empty-swap REMOVE (no
			 * swap content consumed), so both tries stay pristine:
			 *
			 *  - -EAGAIN: a peer relocated the contended spine and the detach's
			 *    recompaction cannot lock its ancestor.  Free this attempt's
			 *    build-invisible extract cluster + reserve + txns and re-descend
			 *    (mirrors retry_attach / retry_merge).
			 *  - -ENOMEM: the detach's commit flip-txn is a malloc NOT backed by
			 *    the node reserve, so it can still fail under memory pressure
			 *    (ft-remove.h, "aborts before any side-effect").  Terminal: fall
			 *    into prep_oom (releases the §11 pin, frees every handle, returns
			 *    MEMORY_ERROR) rather than spinning a hopeless re-descend.
			 */
			if (dret == -EAGAIN) {
				ft_glue_abort(swap_ft, &glue_extract);
				ft_glue_abort(dst_ft, &glue_insert);
				if (extract_txn) {
					ft_flip_txn_destroy(extract_txn);
					extract_txn = NULL;
				}
				if (run_replace_txn) {
					ft_flip_txn_destroy(run_replace_txn);
					run_replace_txn = NULL;
				}
				if (gs_reserved) {
					cds_ft_alloc_reserve_drain(dst_ft, &gs_reserve);
					gs_reserved = false;
				}
				FT_GS_PROBE_INC(cds_ft_probe_gs_retry);
				ft_txn_attempt_bail(&optxn, gs_open);
				gs_open = false;
				goto retry_swap;
			}
			if (dret != 0) {
				assert(dret == -ENOMEM);
				goto prep_oom;
			}
			(void) dret;
			swap_run.armed = drun.armed;
			empty_pruned = true;
		}

		/*
		 * graft_swap edits the subtree at @key via ft_publish_to_parent
		 * directly (no ft_node_set_nth), so emit the structural edge for
		 * consumers.  The pruned case freed @d.pnf and emits its own
		 * detach tracepoints.
		 */
		if (d.depth >= 1 && !empty_pruned)
			FT_TP(tree_edge_set, (const void *) dst_ft,
				(const void *) d.pnf,
				(unsigned int) (d.depth - 1),
				(uint8_t) _key[d.depth - 1],
				(const void *) (have_insert ? glue_insert.top : NULL));

		/*
		 * Parent nr_child on the non-NULL -> NULL transition (remove) +
		 * external-count propagation.  Skipped for @empty_pruned: the detach
		 * above owns nr_child and propagated the count first (@d.pnf is freed).
		 */
		if (!empty_pruned) {
			pmeta = cds_ft_item_to_metadata(ft_node_ptr(d.pnf));
			if (!have_insert)
				ft_meta_nr_child_dec_flip(pmeta);
			/* NET count delta folded onto the replace publish above. */
		}

		/*
		 * Replace run_D with run_S in dst's ordered list (run_S now lives at
		 * @key structurally; run_S NULL for an empty swap -> run_D just
		 * leaves).  Paired with the dst-side drain below, which removes any
		 * reader still holding run_D in dst.  Skipped when the publish above
		 * already FUSED it into the structural flip (swap_run.armed): only the
		 * gs_reserved sole-child detach-prune path still uses this standalone
		 * form (its prune commits separately via ft_detach_node).  It commits
		 * through the pre-reserved @run_replace_txn (un-abortable post-drain);
		 * every fused shape frees that txn unused.
		 */
		if (gs_ord && !swap_run.armed) {
			ft_ord_cell_run_replace(dst_ft, run_replace_txn,
				gs_d_first, gs_d_last, gs_s_first, gs_s_last);
			run_replace_txn = NULL;	/* consumed */
		}
		if (run_replace_txn)
			ft_flip_txn_destroy(run_replace_txn);	/* fused: unused */

		/*
		 * Drain dst-side readers that may still hold the displaced
		 * subtree (or any node within it) in their RCU snapshot with
		 * its OLD parent pointing into dst.  Without this sync, the
		 * extract apply_deferred below rewires that parent to point
		 * into cluster B (in swap_ft), and a reader walking up via
		 * the rewired pointer would CROSS-TRIE-ESCAPE from dst into
		 * swap_ft -- observing top_B's NULL parent at non-root depth.
		 * The earlier sync at the swap unlink only drains swap_ft
		 * readers; this one drains dst_ft readers that captured the
		 * displaced data before it was detached from dst by the
		 * insert-side publish above.
		 *
		 * Exclusive dst carries no RCU readers, so the sync is
		 * skipped in that case.
		 */
		/*
		 * Release the §11 RCU-pin BEFORE this dst grace period: every
		 * descent-captured dst node has been LOCK-locked by the
		 * extract/insert commits above (so it can no longer be reclaimed
		 * out from under us), and a grace period inside a read section
		 * would self-deadlock.  The extract-side root install below
		 * re-parents only the already-detached displaced subtree, which
		 * needs no descent-capture pin.
		 */
		/*
		 * Close the escalation bracket BEFORE the dst drain below:
		 * ft_writer_lock_gp_wait() waits a grace period, and a writer parked
		 * on the domain's FIFO lane is an ONLINE, non-quiescent reader holding
		 * that grace period open -- which is why that function asserts
		 * !urcu_txn_in_fallback().  Same reason the read section is released
		 * immediately below.
		 */
		ft_txn_attempt_end(&optxn, gs_open);
		gs_open = false;
		if (gs_rlock) {
			dst_ft->group->flavor->read_unlock();
			gs_rlock = false;
		}
		if (!dst_ft->exclusive)
			ft_writer_lock_gp_wait(dst_ft);

		/*
		 * Extract side: wire cluster B's deferred back-pointer, then install
		 * swap_ft's new root.  This re-parents the displaced subtree AFTER it
		 * has been detached from dst by the publish above and after the
		 * dst-side drain above.
		 */
		ft_glue_apply_deferred(dst_ft, &glue_extract);
		{
			struct ft_ord_cell_edge edges[3] = { 0 };	/* root + head + tail */
			unsigned int n = 0;

			if (top_B) {
				struct cds_ft_metadata *bm =
					cds_ft_item_to_metadata(ft_node_ptr(top_B));

				/* @top_B is swap_ft's new root: name that trie. */
				bm->parent_word = ft_trie_parent(swap_ft);
				ft_meta_parent_slot_offset_set(bm, 0);
				/*
				 * Structural root install, deferred into the fused flip
				 * below so it commits atomically with run_D's head/tail --
				 * the appear-side cross-view window (structure-present /
				 * list-empty).
				 */
				edges[n].slot = (struct ft_ord_cell **) &swap_ft->root;
				edges[n].old_target =
					(struct ft_ord_cell *) swap_ft->root;
				edges[n].new_target = (struct ft_ord_cell *) top_B;
				/* A root records MW: no node owns it. */
				edges[n].root = true;
				n++;
			} else {
				/*
				 * External (or absent) displaced content: attach it as
				 * external_nodes on swap_ft's root (the transient @fresh for a
				 * non-empty swap, or old_swap_root's empty root for an empty swap).
				 * The preceding dst-side drain makes this displaced
				 * (pre-existing) content's plain forward store safe, and the
				 * target is a NULL-parent root (DRAIN_EXEMPT).
				 */
				struct cds_ft_inode_flag *root_nf = swap_empty ?
					old_swap_root : ft_node_flag(fresh, 0);
				struct cds_ft_metadata *rm = swap_empty ?
					swap_rmeta : fresh_meta;

				if (old_child) {
					ft_metadata_set_external_nodes(root_nf, rm,
						(struct cds_ft_node *) ft_node_ptr(old_child));
					/*
					 * Root: parent is legitimately NULL.  Publishing
					 * prev here is safe (no fresh non-root cluster
					 * node in this back-pointer chain); kept paired
					 * with the metadata write for consistency with
					 * the other attach sites.
					 */
					ft_publish_external_nodes_prev(dst_ft, root_nf,
						(struct cds_ft_node *) ft_node_ptr(old_child));
					ft_nr_keys_store(swap_ft, rm, old_count, CMM_RELEASE);
				}
			}

			/*
			 * Install run_D (the extracted subtree's heads) as swap_ft's
			 * whole ordered list, FUSED with the structural root install
			 * above (top_B case) into ONE flip so a reader never sees swap
			 * structure-present / list-empty.  swap_ft was drained at the
			 * unlink sync, so clearing run_D's boundary links is a plain store.
			 */
			if (gs_ord) {
				/*
				 * swap_ft was emptied (swap_retire / already empty), so its
				 * sentinel currently points at itself.  Point the run's outer
				 * links at swap's sentinel (plain: swap was drained) and fuse
				 * the sentinel endpoint flips (self -> run_D first/last) with
				 * the structural root install above.
				 */
				gs_d_first->lnode.prev = &swap_ft->ord_sentinel.node;
				gs_d_last->lnode.next = &swap_ft->ord_sentinel.node;
				edges[n].tag = URCU_TXN_TAG;	/* ordered-cell edge */
				edges[n].slot = (struct ft_ord_cell **)
					&swap_ft->ord_sentinel.node.next;
				edges[n].old_target = ft_ord_sentinel_cell(swap_ft);
				edges[n].new_target = gs_d_first;
				n++;
				edges[n].tag = URCU_TXN_TAG;	/* ordered-cell edge */
				edges[n].slot = (struct ft_ord_cell **)
					&swap_ft->ord_sentinel.node.prev;
				edges[n].old_target = ft_ord_sentinel_cell(swap_ft);
				edges[n].new_target = gs_d_last;
				n++;
			}
			/*
			 * Freeze-on-free (doc §4.B): when @top_B replaces swap_ft's
			 * root, the transient root this install retires -- @fresh for a
			 * non-empty swap, the old empty @old_swap_root for an empty swap,
			 * both reader-visible as swap_ft->root during the unlink window --
			 * gets its tombstone recorded INTO extract_txn, so the mark and
			 * the root install unlink flip atomically (atomic detach).  Only
			 * on the top_B path (the external/absent case keeps the root live);
			 * top_B always installed the root edge into @edges, so extract_txn
			 * is reserved (list-on) or the 3-edge list-off txn just reserved.
			 * (This transient-root tombstone is distinct from the fused
			 * free-list tombstone already recorded by ft_glue_apply_deferred.)
			 */
			if (top_B)
				ft_flip_txn_record_tombstone(extract_txn,
					cds_ft_item_to_metadata(swap_empty ?
						ft_node_ptr(old_swap_root) : fresh));
			/*
			 * Post-drain commit (un-abortable): a reserved extract_txn --
			 * list on (root install + run_D head/tail + the fused top_B and
			 * free-list tombstones) or the list-off 3-edge top_B retire (root
			 * install + top_B tombstone + free-list tombstone) -- commits every
			 * recorded edge; freed unused if none changed.  When extract_txn is
			 * NULL (list off, no top_B retire) @n is 0 and the free_list is
			 * empty, so the lone-edge arm is unreached.
			 */
			if (extract_txn) {
				if (n)
					/* Bulk op, not yet MW-hardened: ABORT
					 * unreachable under its exclusion. */
					(void) ft_ord_cell_flip_into(swap_ft,
						extract_txn, edges, n);
				else
					ft_flip_txn_destroy(extract_txn);
				extract_txn = NULL;	/* consumed / freed */
			} else if (n) {
				ft_ord_cell_flip_one(&edges[0]);
			}
			FT_TP(root_publish, (const void *) swap_ft,
				(const void *) swap_ft->root);
			if (top_B) {
				if (swap_empty)
					free_cds_ft_node(swap_ft,
						ft_node_ptr(old_swap_root));
				else
					free_cds_ft_node(swap_ft, fresh);
			}
		}

		/*
		 * Reclaim the old (replaced) live nodes after the publishes.
		 *
		 * ★ BOTH free lists hold DST nodes, so BOTH reclaim through @dst_ft.
		 * @glue_extract's list is the compressed @old_child that
		 * ft_make_root_internal_glue PEELED to materialize swap's new root --
		 * a node that lived in the DESTINATION, was reader-visible there, and
		 * whose readers only this trie's grace period drains.  Reclaiming it
		 * through @swap_ft instead took the trie's access discipline from the
		 * WRONG side: cds_ft_free_item frees IMMEDIATELY on an exclusive trie
		 * (no grace period, straight onto the arena free list), and @swap_ft
		 * is exclusive by contract right up to the inherit below.  The node
		 * was therefore recycled while dst readers -- and a peer writer's
		 * in-flight descent -- still held it, and the arena handed the same
		 * address back as the next allocation: a second trie's root wearing a
		 * live destination node's address.  That is what
		 * inv_graft_swap_shared_dst_nolist saw as a depth-1 node whose parent
		 * had gone NULL.  ft_glue_apply_deferred above was already passed
		 * @dst_ft for this same glue; this call was the odd one out.
		 */
		ft_glue_free_old(dst_ft, &glue_insert);
		ft_glue_free_old(dst_ft, &glue_extract);

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
		 * swap_ft now holds content displaced from dst_ft; inherit dst_ft's
		 * access discipline for that content.  dst_ft keeps its own.
		 */
		swap_ft->exclusive = dst_ft->exclusive;
		/* Free any unused empty-swap prune reserve. */
		if (gs_reserved)
			cds_ft_alloc_reserve_drain(dst_ft, &gs_reserve);
		FT_TP(graft_swap_exit, (int) CDS_FT_STATUS_OK);
		return CDS_FT_STATUS_OK;

	prep_oom:
		/*
		 * Allocation failed during the build: free every fresh glue node (both
		 * clusters), drop the transient swap root, and surface MEMORY_ERROR.
		 * No deferred edge was applied and nothing was published, so dst_ft and
		 * swap_ft are both pristine -- there is nothing to roll back.
		 */
		/* Every build error jumps here before the dst-drain release above. */
		/*
		 * Close the escalation bracket BEFORE the build-error unwind:
		 * ft_writer_lock_gp_wait() waits a grace period, and a writer parked
		 * on the domain's FIFO lane is an ONLINE, non-quiescent reader holding
		 * that grace period open -- which is why that function asserts
		 * !urcu_txn_in_fallback().  Same reason the read section is released
		 * immediately below.
		 */
		ft_txn_attempt_end(&optxn, gs_open);
		gs_open = false;
		if (gs_rlock) {
			dst_ft->group->flavor->read_unlock();
			gs_rlock = false;
		}
		ft_glue_abort(dst_ft, &glue_insert);
		ft_glue_abort(swap_ft, &glue_extract);
		if (glue_insert.txn)
			ft_flip_txn_destroy(glue_insert.txn);
		if (glue_publish_txn)
			ft_flip_txn_destroy(glue_publish_txn);
		if (swap_retire_txn)
			ft_flip_txn_destroy(swap_retire_txn);
		if (extract_txn)
			ft_flip_txn_destroy(extract_txn);
		if (run_replace_txn)
			ft_flip_txn_destroy(run_replace_txn);
		if (fresh)
			free_cds_ft_node_unpublished(swap_ft, fresh);	/* never published */
		if (gs_reserved)
			cds_ft_alloc_reserve_drain(dst_ft, &gs_reserve);
		FT_TP(graft_swap_exit, (int) CDS_FT_STATUS_MEMORY_ERROR);
		return CDS_FT_STATUS_MEMORY_ERROR;
	}
}

