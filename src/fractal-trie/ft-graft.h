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
		struct ft_glue *glue)
{
	struct cds_ft_compressed_node *cn = ft_compressed_node_ptr(d->nf);
	struct cds_ft_metadata *cn_meta =
		cds_ft_item_to_metadata((struct cds_ft_inode *) cn);

	/* Compressed metadata never carries external_nodes (see
	 * ft_split_compressed_insert). */
	assert(!cn_meta->external_nodes);
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

	(void) branch_cluster_leaf;	/* documents intent; both set_nth defer */

	/* Compute old child's nr_keys. */
	if (!ft_node_external(cn->child)) {
		struct cds_ft_metadata *cm =
			cds_ft_item_to_metadata(ft_node_ptr(cn->child));
		old_child_nr_keys = ft_nr_keys_get(cm);
	} else if (cn->child) {
		old_child_nr_keys = 1;
	} else {
		old_child_nr_keys = 0;
	}

	/*
	 * 1. Build the OLD-direction suffix -> old child (mirrors the legacy
	 * split).  cn->child (live) is deferred into @glue.
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
		sfx->child = cn->child;
		sfx->len = suffix_len;
		memcpy(sfx->key_bytes, &cn->key_bytes[diverge_pos + 1],
			suffix_len);
		ft_meta_nr_child_set(sfx_meta, 1);
		ft_nr_keys_store(ft, sfx_meta, old_child_nr_keys, CMM_RELAXED);
		old_suffix_flag = ft_compressed_node_flag(sfx);
		sfx_skip_flag = ft_publish_compressed(ft, sfx, old_suffix_flag);
		ft_glue_track(glue, old_suffix_flag);
		/*
		 * The displaced child @cn->child is LIVE: a reader can still descend
		 * to it through @cn (the compressed node being split, untouched until
		 * the forward publish replaces it).  So its re-parent onto the fresh
		 * @sfx is a reader-observable pointer -- ride it on the flip-txn
		 * (dst_origin) so it flips atomically with the forward edge.  Holds for
		 * an external @cn->child too: the up-walk readers (ft_get_parent_rcu /
		 * ft_skip_to_compressed / ft_skip_reanchor) resolve a flip proxy parked
		 * on an external's parent.  The merge-rekey path has no txn (glue->txn
		 * NULL); there it stays on the legacy fresh-before-live immediate store.
		 */
		ft_glue_defer_edge_origin(ft, glue, cn->child, old_suffix_flag,
			&sfx->child, glue->txn != NULL);
	} else if (suffix_len == 1) {
		struct cds_ft_inode_flag *dest = NULL;

		/* 1-child internal suffix (non-SC): cluster-leaf. */
		ret = ft_node_set_nth(ft, &dest,
				cn->key_bytes[diverge_pos + 1],
				cn->child, NULL, NULL,
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
		ft_glue_defer_edge_origin(ft, glue, cn->child, dest, slot,
			glue->txn != NULL);
	} else {
		old_suffix_flag = cn->child;	/* suffix_len == 0 */
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

		bm->parent = NULL;
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
		/* Reallocated: drop the order-1 copy from tracking + free it. */
		ft_glue_untrack(ft, glue, old_branch);
		free_cds_ft_node_unpublished(ft, old_branch);
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
	 * suffix_len == 0: @old_suffix_flag IS the live @cn->child wired directly
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
		struct ft_graft_store_state *st)
{
	memset(st, 0, sizeof(*st));
	st->glue = glue;

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
				.gp = d->pppnf, .gp_slot = d->ppnfp });
		if (ret)
			return CDS_FT_STATUS_MEMORY_ERROR;

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
					.gp_slot = d->ppnfp });
			if (ret)
				return CDS_FT_STATUS_MEMORY_ERROR;

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
		ft_glue_txn_commit(ft, st->glue, run);
		if (st->tp_i >= 1)
			FT_TP(tree_edge_set, (const void *) ft,
				(const void *) st->pnf,
				(unsigned int) (st->tp_i - 1),
				(uint8_t) st->tp_key[st->tp_i - 1],
				(const void *) st->attached);
	} else {
		struct cds_ft_inode_flag **slot = NULL;
		struct ft_ord_cell_edge redges[4];
		struct cds_ft_inode_flag *count_base = NULL;
		unsigned int rn = 0, i;

		ft_node_get_nth_skip(st->dest, &slot, st->slot_byte, FT_PF_NONE);
		assert(slot);
		ft_set_parent(ft, st->attached, st->dest, slot);
		ft_glue_apply_deferred(ft, st->glue);
		if (st->old_recompacted_node) {
			unsigned int k;
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
			 * ft_flip_txn_record_release_copying).  Non-lock_fine keeps the guard.
			 */
			if (!ft->lock_fine)
				ft_flip_txn_guard_parent(ft, st->glue->txn,
					pub_parent);
			_ft_publish_to_parent(ft, st->dest,
				pub_slot, st->dest,
				*pub_slot /* SW graft: old dst node */,
				&st->reserve_rec);
			for (k = 0; k < st->reserve_rec.n; k++)
				ft_flip_txn_record_reserved(st->glue->txn,
					(void **) st->reserve_rec.slot[k],
					(void *) st->reserve_rec.old_val[k],
					(void *) st->reserve_rec.new_val[k]);
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
		ft_flip_txn_record_reserved(st->glue->txn, (void **) slot,
			NULL, (void *) st->slot_value);
		if (run) {
			rn = ft_ord_cell_run_splice_edges(ft, run->run_first,
				run->run_last, run->pred, run->succ, redges, 0);
			for (i = 0; i < rn; i++)
				ft_flip_txn_record_tag(st->glue->txn,
					(void **) redges[i].slot,
					redges[i].old_target,
					redges[i].new_target,
					ft_edge_tag(&redges[i]));
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
		 * as its C-half {COPYING|s -> TOMBSTONE|s} terminal, registered on
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
		ft_flip_txn_commit(ft, st->glue->txn);
		st->glue->txn = NULL;
		if (run)
			run->armed = true;

		if (st->old_recompacted_node)
			free_cds_ft_node(ft, st->old_recompacted_node);
	}
	ft_glue_free_old(ft, st->glue);
	*attached_nf = st->attached;
	*attached_depth = st->attached_depth;
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

	status = ft_store_at_graft_point_prepare(ft, key, key_len, d,
			graft_payload, graft_external_count, glue, &st);
	if (status != CDS_FT_STATUS_OK)
		return status;
	ft_store_at_graft_point_commit(ft, attached_nf, attached_depth, run,
			&st, count_delta);
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
		struct ft_descent *d, struct ft_glue *glue)
{
	const uint8_t *ik = key;

	ft_descent_init(d, ft);
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
				if (ft_split_compressed_graft_build(ft, d, key,
						key_len, j, payload, src_count,
						glue))
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
	if (src_ft != dst_ft && src_ft->lock_mode && !src_ft->exclusive)
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
		struct cds_ft_metadata *dst_rmeta = ft_root_metadata(dst_ft);
		struct cds_ft_inode *fresh_root;
		struct cds_ft_metadata *fresh_meta;
		struct cds_ft_inode *old_dst_root;

		/* Destination must be empty for a root-level graft. */
		if (ft_meta_nr_child(dst_rmeta) != 0 || dst_rmeta->external_nodes)
			return CDS_FT_STATUS_POPULATED_ERROR;

		/*
		 * Allocate a fresh empty root for the source before
		 * swapping, so the source remains a valid trie.
		 */
		fresh_root = alloc_cds_ft_node(dst_ft, &ft_types[0], &fresh_meta);
		if (!fresh_root)
			return CDS_FT_STATUS_MEMORY_ERROR;

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
			dual_txn = ft_flip_txn_create_bounded(
				FT_ROOT_LIST_SWAP_DUAL_MAX_EDGES + 1);
			if (!dual_txn) {
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
		old_dst_root = ft_node_ptr(dst_ft->root);
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
			struct cds_ft_inode_flag *dst_old = dst_ft->root;
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
			 */
			ft_flip_txn_record_tombstone(dual_txn,
				cds_ft_item_to_metadata(ft_node_ptr(dst_old)));
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
		 * retry path.  Progress: {p}'s COPYING lock guarantees a winner
		 * each contention round.
		 */
retry_attach:
		/*
		 * Preallocate a fresh empty root for the source trie
		 * before the point of no return, so we can fail cleanly
		 * on memory shortage instead of calling abort().
		 */
		fresh_node = alloc_cds_ft_node(src_ft, &ft_types[0], &fresh_meta);
		if (!fresh_node)
			return CDS_FT_STATUS_MEMORY_ERROR;

		/*
		 * PREP (dst + source pristine): build the dst-side attach.
		 * A diverge split builds its whole cluster invisibly into
		 * @glue; otherwise just locate the graft point in @d.
		 */
		ft_glue_init(&glue);
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
			glue.txn = ft_flip_txn_create();
			if (!glue.txn || !ft_flip_txn_reserve(glue.txn,
					/* + FLOOR_FREE: fused free-list tombstones (§4.B) */
					FT_GLUE_FLOOR_DEFERRED + 7 + 1 /* +1 §4.B parent guard */ + FT_GLUE_FLOOR_FREE
					/* + count walk: the +src_count nr_keys ancestor edges (BULK fold) */
					+ (dst_ft->rank_stats ? (int) key_len + 1 : 0))) {
				if (glue.txn)
					ft_flip_txn_destroy(glue.txn);
				free_cds_ft_node_unpublished(src_ft, fresh_node);
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
				src_count, &d, &glue);
		if (prep == FT_GRAFT_PREP_OOM) {
			ft_glue_abort(dst_ft, &glue);
			if (glue.txn)
				ft_flip_txn_destroy(glue.txn);
			free_cds_ft_node_unpublished(src_ft, fresh_node);
			return CDS_FT_STATUS_MEMORY_ERROR;
		}
		if (prep == FT_GRAFT_PREP_POPULATED) {
			if (glue.txn)
				ft_flip_txn_destroy(glue.txn);
			free_cds_ft_node_unpublished(src_ft, fresh_node);
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
			free_cds_ft_node_unpublished(src_ft, fresh_node);
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
			ft_flip_txn_destroy(glue.txn);
			free_cds_ft_node_unpublished(src_ft, fresh_node);
			return CDS_FT_STATUS_POPULATED_ERROR;
		}

		/*
		 * Pre-reserve the failure-free src-root retire's txn now -- after
		 * the last POPULATED / OOM exit, before the only remaining fallible
		 * step (the self-secure reserve below).  The retire is past the
		 * point of no return and cannot abort, so it commits through this
		 * pre-reserved txn (ft_ord_cell_flip_into).  List off retires via a
		 * lone-edge root store, so reserve only when the list is on -- EXCEPT
		 * a nil-key root retire fuses the orphaned wrapper's tombstone into
		 * the retire (atomic detach, §4.B), needing a 2-edge txn even list-
		 * off; list-on grows by +1 for that same fused tombstone.
		 */
		if (dst_ft->group->ordered_list_set) {
			src_retire_txn = ft_flip_txn_create_bounded(
				FT_ROOT_LIST_SWAP_MAX_EDGES + 1);
			run_splice_txn = ft_flip_txn_create_bounded(
				FT_ORD_CELL_RUN_SPLICE_MAX_EDGES);
			if (!src_retire_txn || !run_splice_txn) {
				if (src_retire_txn)
					ft_flip_txn_destroy(src_retire_txn);
				if (run_splice_txn)
					ft_flip_txn_destroy(run_splice_txn);
				ft_flip_txn_destroy(glue.txn);
				free_cds_ft_node_unpublished(src_ft, fresh_node);
				return CDS_FT_STATUS_MEMORY_ERROR;
			}
		} else if (nil_key_root) {
			src_retire_txn = ft_flip_txn_create_bounded(2);
			if (!src_retire_txn) {
				ft_flip_txn_destroy(glue.txn);
				free_cds_ft_node_unpublished(src_ft, fresh_node);
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
				&graft_pred, &graft_succ);

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
				free_cds_ft_node_unpublished(src_ft, fresh_node);
				return CDS_FT_STATUS_MEMORY_ERROR;
			}
			self_secured = true;
		}

		/*
		 * MW LOCK_FINE drop (§11, cross-trie): run the NOSPLIT store's
		 * FALLIBLE half -- ft_store_at_graft_point_prepare, i.e. the
		 * recompact of the graft-point node {p} -- BEFORE the point-of-no-
		 * return src-root swap.  A concurrent relocation of {p}, a peer that
		 * holds {p}'s (or the grandparent's) COPYING lock, or a stale
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
		 * locked {p} is caught INSIDE the recompact's COPYING mark
		 * (ft_meta_copying_mark -> -EAGAIN -> prepare MEMORY_ERROR).
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
					src_count, &glue, &st);
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
				 * unwinding.  Re-descend -- {p}'s COPYING lock
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
				free_cds_ft_node_unpublished(src_ft, fresh_node);
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
		 * subtree.  Lock @fence_parent's COPYING fence HERE, before the swap, so
		 * no peer can retire it through the commit (which records the held
		 * {COPYING|s -> s} release via @publish_parent_holder, so the commit is
		 * truly unfailable).  A miss (already retired / proxied / peer-locked)
		 * is a clean re-descend with src pristine -- ft_meta_copying_mark did
		 * NOT set the fence, so nothing to unwind but the invisible dst build.
		 *
		 * TWO shapes reach an unfenced forward publish into d.pnf: the GLUE
		 * diverge (@glue.publish_parent set at build) and the NOSPLIT
		 * DISPLACED-external attach (its commit publishes the fresh branch into
		 * d.pnf's child slot, st.pnf == d.pnf; @st is valid once
		 * @nosplit_prepared).  The in-place NOSPLIT store needs no fence here --
		 * its ft_node_set_nth_rec recompacts + COPYING-locks {p} inside prepare
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
			if (fence_parent) {
				struct cds_ft_metadata *pp_meta =
					ft_flag_to_metadata(dst_ft, fence_parent);
				uintptr_t pp_snap = 0;

				if (ft_meta_copying_mark(pp_meta, &pp_snap)) {
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
					free_cds_ft_node_unpublished(src_ft, fresh_node);
					goto retry_attach;
				}
				glue.publish_parent_holder = pp_meta;
				glue.publish_parent_snap = pp_snap;
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
		if (dst_ft->group->ordered_list_set) {
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
			ft_flip_txn_record_reserved(src_retire_txn,
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
		FT_TP(root_publish, (const void *) src_ft,
			(const void *) src_ft->root);

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
			ft_glue_txn_commit(dst_ft, &glue, run_arg);
			attached_nf = glue.attached_nf;
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
			ft_store_at_graft_point_commit(dst_ft, &attached_nf,
				&attached_depth, run_arg, &st,
				(long) src_count);
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

#ifdef FEATURE_FT_MW_LOCK_FINE_DROP
	if (dst_ft->lock_fine) {
		const struct rcu_flavor_struct *flavor = dst_ft->group->flavor;

		/*
		 * FT-wide-lock drop RCU-pinning (§11 cross-trie).  A FINE
		 * cross-trie graft descends the SHARED dst WITHOUT the FT-wide
		 * mutex and captures spine nodes (d->pnf / ppnf / pppnf) that its
		 * recompact / Fix-A fence later COPYING-lock.  A COPYING lock
		 * rejects a relocated-but-LIVE (tombstoned) node, but NOT a
		 * reclaimed-and-recycled one -- the arena re-zeroes a slot's
		 * metadata on reallocation, so ft_copying_lock_member false-
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
#endif
		status = ft_graft_keylen(dst_ft, _key, key_len, src_ft, NULL);
	FT_TP(graft_exit, (int) status);
	return status;
}

/*
 * Outcome of ft_graft_swap_descend's read-only descent toward the swap key.
 */
enum ft_graft_swap_case {
	FT_GRAFT_SWAP_EXACT,		/* reached key_len at a live subtree (d->nf) */
	FT_GRAFT_SWAP_KEY_SHORTER,	/* key ends strictly inside compressed d->nf */
	FT_GRAFT_SWAP_DELEGATE,		/* diverge / dead-end: no content at key */
};

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
 */
static
enum ft_graft_swap_case ft_graft_swap_descend(struct cds_ft *ft,
		const uint8_t *key, size_t key_len, struct ft_descent *d)
{
	const uint8_t *ik = key;

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
			/* j == cmp == remaining < cn->len: key ends inside cn. */
			return FT_GRAFT_SWAP_KEY_SHORTER;
		}
		if (!ft_descent_step(ft, d, *(ik++)))
			return FT_GRAFT_SWAP_DELEGATE;	/* dead-end */
	}
	if (!d->nf)
		return FT_GRAFT_SWAP_DELEGATE;	/* empty slot at key */
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
	if (swap_ft->lock_mode && !swap_ft->exclusive) {
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
		dual_txn = ft_flip_txn_create_bounded(
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
#ifdef FEATURE_FT_MW_LOCK_FINE_DROP
		bool gs_rlock = false;
#endif

		/*
		 * Read-only descent: nothing is published, so the whole swap can be
		 * assembled as a build-invisible transaction and an allocation failure
		 * leaves both tries pristine.
		 */
#ifdef FEATURE_FT_MW_LOCK_FINE_DROP
		/*
		 * §11 cross-trie RCU-pinning: this descent captures live-dst spine
		 * nodes (d.pnf ...) that the extract-side detach and the insert-side
		 * publish-replace COPYING-lock below.  A COPYING lock rejects a
		 * relocated-but-live node but NOT a reclaimed+recycled one (the arena
		 * re-zeroes metadata on realloc -> false-success -> wild store), so
		 * pin the captured nodes with the flavor read side across
		 * descent->lock, exactly as cds_ft_graft does.  SCOPED, not whole-op:
		 * graft_swap drains dst readers with ft_writer_lock_gp_wait(dst_ft)
		 * before its extract-side root install, and a grace period under a
		 * read section self-deadlocks -- so the section is RELEASED just
		 * before that dst drain (every descent-captured dst node is COPYING-
		 * locked by then).  The consumed @swap_ft is exclusive (BUSY_ERROR
		 * otherwise), so its own ft_writer_lock_gp_wait is !exclusive-gated
		 * and skipped, and nothing else synchronizes inside the section.
		 */
		if (dst_ft->lock_fine && swap_ft->exclusive) {
			dst_ft->group->flavor->read_lock();
			gs_rlock = true;
		}
#endif
		kase = ft_graft_swap_descend(dst_ft, key, key_len, &d);
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

#ifdef FEATURE_FT_MW_LOCK_FINE_DROP
			if (gs_rlock)
				dst_ft->group->flavor->read_unlock();
#endif
			FT_TP(graft_swap_exit, (int) s);
			return s;
		}

		old_swap_root = swap_ft->root;
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
		}

		ft_glue_init(&glue_insert);
		ft_glue_init(&glue_extract);

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
			if (pcn && ccn &&
			    (unsigned int) pcn->len + ccn->len <= FT_SKIP_LEN_MAX) {
				struct cds_ft_metadata *pcn_meta =
					cds_ft_item_to_metadata((struct cds_ft_inode *) pcn);
				unsigned int merged_len = pcn->len + ccn->len;
				struct cds_ft_compressed_node *merged;
				struct cds_ft_metadata *merged_meta;
				struct cds_ft_inode_flag *merged_flag, *merged_skip;
				struct cds_ft_inode_flag **pub_slot;
				struct cds_ft_inode_flag *pub_parent;

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
				merged_meta->parent = pub_parent;
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
				ft_glue_set_publish(dst_ft, &glue_insert, pub_parent,
					pub_slot, merged_skip);
				ft_glue_defer_free(&glue_insert, pcn, true);
				d.pnf = merged_flag;	/* structural edits target @merged */
				have_insert = true;
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
			ft_glue_set_publish(dst_ft, &glue_insert, d.pnf, d.nfp, top_A);
			have_insert = true;
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
			glue_insert.txn = ft_flip_txn_create();
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
			glue_publish_txn = ft_flip_txn_create_bounded(
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
				swap_retire_txn = ft_flip_txn_create_bounded(
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
			extract_txn = ft_flip_txn_create_bounded(gs_ord ?
				FT_ROOT_LIST_SWAP_MAX_EDGES + 2 : 3);
			if (!extract_txn)
				goto prep_oom;
			glue_extract.txn = extract_txn;
			glue_extract.fuse_free_list = true;
			if (gs_ord) {
				run_replace_txn = ft_flip_txn_create_bounded(
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
		 * "Jump out" prevention: unlink old_swap_root from swap_ft (install
		 * @fresh) and drain its readers BEFORE its parent pointer is flipped
		 * into dst_ft.  Readers see an empty swap_ft between here and the final
		 * root install below.
		 */
		if (!swap_empty) {
			struct cds_ft_inode_flag *empty = ft_node_flag(fresh, 0);

			/*
			 * Retire swap's root to an empty node AND (paired) unlink run_S
			 * from swap's ordered list, FUSED in ONE flip so a reader never
			 * sees swap structurally empty while its ordered list still shows
			 * run_S -- the disappear-side cross-view window.  The following
			 * sync then drains swap's readers of the old content; run_D is
			 * installed as swap's list after the extract publish below.
			 * (List off: just the lone root edge.)
			 */
			if (gs_ord)
				/*
				 * Empty swap's sentinel (relink_dest NULL): run_S's cells
				 * are re-homed into dst by the run-replace below; run_D is
				 * installed as swap's list after the extract publish.
				 */
				ft_root_list_swap_publish(swap_ft, swap_retire_txn,
					&swap_ft->root,
					swap_ft->root, empty,
					ft_ord_first(swap_ft), NULL,
					ft_ord_last(swap_ft), NULL,
					NULL, false);
			else
				ft_root_edge_flip(swap_ft, &swap_ft->root,
					swap_ft->root, empty);
			FT_TP(root_publish, (const void *) swap_ft,
				(const void *) swap_ft->root);
			if (!swap_ft->exclusive)
				ft_writer_lock_gp_wait(swap_ft);
		}

		/*
		 * Insert side: wire the deferred live back-pointers, then the single
		 * forward publish that splices cluster A into dst (detaching the old
		 * content).  Empty swap publishes NULL (a remove).
		 */
		if (have_insert) {
			/*
			 * Order-statistics (BULK): the replace swaps @old_count keys for
			 * @swap_count, so the dst NET delta rides the replace publish
			 * (ft_glue_txn_commit_edges / ft_glue_publish_replace record it
			 * from @glue_insert.publish_parent) instead of a post-commit walk.
			 */
			glue_insert.count_delta = (long) swap_count - (long) old_count;
			if (glue_insert.txn)
				ft_glue_txn_commit_replace(dst_ft, &glue_insert,
					swap_run_arg);
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
				ft_glue_publish_replace(dst_ft, glue_publish_txn,
					&glue_insert, swap_run_arg);
				glue_publish_txn = NULL;	/* consumed */
				glue_insert.txn = NULL;	/* the commit reclaimed it */
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
			dret = ft_detach_node(dst_ft, d.nfp, d.pnfp, d.depth,
					false, NULL, gs_ord ? &dpub : NULL,
					gs_ord ? &drun : NULL, NULL, NULL,
					-(long) old_count /* fold -old_count onto the detach commit */);
			cds_ft_alloc_reserve_deactivate(dst_ft);
			assert(dret == 0);	/* reserve guarantees no -ENOMEM */
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
#ifdef FEATURE_FT_MW_LOCK_FINE_DROP
		/*
		 * Release the §11 RCU-pin BEFORE this dst grace period: every
		 * descent-captured dst node has been COPYING-locked by the
		 * extract/insert commits above (so it can no longer be reclaimed
		 * out from under us), and a grace period inside a read section
		 * would self-deadlock.  The extract-side root install below
		 * re-parents only the already-detached displaced subtree, which
		 * needs no descent-capture pin.
		 */
		if (gs_rlock) {
			dst_ft->group->flavor->read_unlock();
			gs_rlock = false;
		}
#endif
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

				/* top_B is freshly built (invisible); wire its root parent. */
				bm->parent = NULL;
#ifdef FEATURE_FT_SKIP_COMPRESSED
				ft_meta_parent_slot_offset_set(bm, 0);
#endif
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
				edges[n].tag = URCU_MCAS_TAG;	/* ordered-cell edge */
				edges[n].slot = (struct ft_ord_cell **)
					&swap_ft->ord_sentinel.node.next;
				edges[n].old_target = ft_ord_sentinel_cell(swap_ft);
				edges[n].new_target = gs_d_first;
				n++;
				edges[n].tag = URCU_MCAS_TAG;	/* ordered-cell edge */
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

		/* Reclaim the old (replaced) live nodes after the publishes. */
		ft_glue_free_old(dst_ft, &glue_insert);
		ft_glue_free_old(swap_ft, &glue_extract);

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
#ifdef FEATURE_FT_MW_LOCK_FINE_DROP
		/* Every build error jumps here before the dst-drain release above. */
		if (gs_rlock) {
			dst_ft->group->flavor->read_unlock();
			gs_rlock = false;
		}
#endif
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
			free_cds_ft_node(swap_ft, fresh);
		if (gs_reserved)
			cds_ft_alloc_reserve_drain(dst_ft, &gs_reserve);
		FT_TP(graft_swap_exit, (int) CDS_FT_STATUS_MEMORY_ERROR);
		return CDS_FT_STATUS_MEMORY_ERROR;
	}
}

