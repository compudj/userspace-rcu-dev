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
		sfx_meta->nr_child = 1;
		ft_nr_keys_store(sfx_meta, old_child_nr_keys, CMM_RELAXED);
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
		ft_nr_keys_store(cds_ft_item_to_metadata(ft_node_ptr(dest)),
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
		bm->parent_slot_offset = 0;
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
	ft_nr_keys_store(cds_ft_item_to_metadata(ft_node_ptr(branch_flag)),
		old_child_nr_keys, CMM_RELAXED);

	/* Wire the OLD direction (re-encode compressed slot to skip form). */
	ft_node_get_nth_skip(branch_flag, &slot, old_ordinal, FT_PF_NONE);
	if (sfx_skip_flag && sfx_skip_flag != old_suffix_flag && slot)
		rcu_assign_pointer(*slot, sfx_skip_flag);
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
			rcu_assign_pointer(*slot, skip);
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
		pfx_meta->nr_child = 1;
		ft_nr_keys_store(pfx_meta, ft_nr_keys_get(cn_meta), CMM_RELAXED);
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
			pfx_meta->nr_child = 1;
			ft_nr_keys_store(pfx_meta, ft_nr_keys_get(cn_meta),
				CMM_RELAXED);
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
		ft_nr_keys_store(pfx_meta, ft_nr_keys_get(cn_meta), CMM_RELAXED);
		top_flag = dest;
		ft_glue_track(glue, dest);
		}
#ifdef FEATURE_FT_SKIP_COMPRESSED
	after_prefix:
		(void) 0;
#endif
	} else {
		/* diverge_pos == 0: branch IS the top. */
		ft_nr_keys_store(cds_ft_item_to_metadata(ft_node_ptr(branch_flag)),
			ft_nr_keys_get(cn_meta), CMM_RELAXED);
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
		ret = ft_node_set_nth(ft, &dest, key[key_len - 1], NULL,
			&st->old_recompacted_node, pmeta, d->depth - 1, false);
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
			ft_nr_keys_store(bm, ft_nr_keys_get(bm) + 1,
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
			ret = ft_node_set_nth(ft, &dest, key[i - 1], NULL,
				&st->old_recompacted_node, pmeta,
				d->depth - 1, false);
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
void ft_store_at_graft_point_commit(struct cds_ft *ft,
		struct cds_ft_inode_flag **attached_nf,
		unsigned int *attached_depth,
		struct ft_graft_run *run,
		struct ft_graft_store_state *st)
{
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
		unsigned int rn = 0, i;
		bool gp;

		ft_node_get_nth_skip(st->dest, &slot, st->slot_byte, FT_PF_NONE);
		assert(slot);
		ft_set_parent(ft, st->attached, st->dest, slot);
		ft_glue_apply_deferred(ft, st->glue);
		if (st->old_recompacted_node) {
			struct ft_pub_rec rec = { .n = 0 };
			unsigned int k;

			/*
			 * The reserve recompacted (relocated) the dst attach node:
			 * fold its grandparent re-point (and a compressed
			 * grandparent's SKIP_X dual) into glue->txn -- pre-reserved
			 * before the build, so no allocation here -- so the
			 * relocation flips ATOMICALLY with the grafted slot edge and
			 * the run-splice in the single commit below.  The src drain
			 * does not cover dst, and the old dst node stays resolved-to
			 * via the parked grandparent proxy until the commit (freed
			 * below, after it).
			 */
			_ft_publish_to_parent(ft, st->publish_pmeta->parent,
				st->pnfp, st->dest, &rec);
			for (k = 0; k < rec.n; k++)
				ft_flip_txn_record_reserved(st->glue->txn,
					(void **) rec.slot[k],
					(void *) rec.old_val[k],
					(void *) rec.new_val[k]);
		} else {
			/* In-place reserve: a redundant same-value republish. */
			ft_publish_to_parent(ft, st->publish_pmeta->parent,
				st->pnfp, st->dest);
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
				ft_flip_txn_record_reserved(st->glue->txn,
					(void **) redges[i].slot,
					redges[i].old_target,
					redges[i].new_target);
		}
		gp = urcu_flip_txn_commit(st->glue->txn);
		ft_flip_txn_reclaim(ft, st->glue->txn, gp);
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
		struct ft_graft_run *run)
{
	struct ft_graft_store_state st;
	enum cds_ft_status status;

	status = ft_store_at_graft_point_prepare(ft, key, key_len, d,
			graft_payload, graft_external_count, glue, &st);
	if (status != CDS_FT_STATUS_OK)
		return status;
	ft_store_at_graft_point_commit(ft, attached_nf, attached_depth, run, &st);
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
				ft_descent_traverse_compressed(d, cn, &ik);
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
		struct urcu_flip_txn **pre_txn)
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
		struct cds_ft_inode *old_dst_root;

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
		 * unchanged; only the head/tail endpoints transfer.  Fuse each
		 * side's structural root swap with its list-endpoint transfer in
		 * ONE flip (ft_root_list_swap_publish), so a reader never sees
		 * the keys present in one index but absent from the other:
		 *  - dst (appear): publish src->root AND src's head/tail at once
		 *    (dst's head/tail were NULL), closing the structure-present /
		 *    list-empty window.
		 *  - src (disappear): retire src->root to a fresh empty root AND
		 *    clear src's head/tail at once, closing the symmetric
		 *    structure-empty / list-present window on the drained source.
		 * The dst side runs first so it captures src's still-live root
		 * and head/tail before the src side retires them.
		 */
		if (dst_ft->group->ordered_list_set) {
			ft_root_list_swap_publish(dst_ft, &dst_ft->root,
				dst_ft->root, src_ft->root,
				NULL, src_ft->ord_cell_head,
				NULL, src_ft->ord_cell_tail);
			ft_root_list_swap_publish(src_ft, &src_ft->root,
				src_ft->root, ft_node_flag(fresh_root, 0),
				src_ft->ord_cell_head, NULL,
				src_ft->ord_cell_tail, NULL);
			/* the src flip above cleared src's head/tail to NULL */
		} else {
			/*
			 * No ordered list: the root pointer is the only
			 * reader-visible slot, so a single rcu_assign_pointer is
			 * already atomic -- no flip needed (and no synchronize_rcu,
			 * both being root positions with parent == NULL).
			 */
			rcu_assign_pointer(dst_ft->root, src_ft->root);
			rcu_assign_pointer(src_ft->root,
				ft_node_flag(fresh_root, 0));
		}
		FT_TP(root_publish, (const void *) dst_ft,
			(const void *) dst_ft->root);
		FT_TP(root_publish, (const void *) src_ft,
			(const void *) src_ft->root);
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
		 * NIL-key-only source: the whole source is a single prefix key,
		 * stored as the root's external_nodes (a childless internal -- valid
		 * only AT a root).  Grafting that wrapper internal to a non-root
		 * position would leave a non-canonical childless internal there
		 * (cds_ft_remove_all's invariant).  Graft the external chain head
		 * DIRECTLY instead, so the placed node is a plain external, and free
		 * the orphaned wrapper on success.  (Cross-trie graft never hits this:
		 * a real source root always has children.)
		 */
		bool nil_key_root = (src_rmeta->nr_child == 0
				&& src_rmeta->external_nodes != NULL);
		struct cds_ft_inode_flag *graft_payload = nil_key_root ?
			(struct cds_ft_inode_flag *) ft_dereference_external(
				src_rmeta->external_nodes) : src_ft->root;

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
		 * mid-build, like the glue floor arrays); the + 6 headroom covers the
		 * forward edge's 1-2 stores plus the <=4 run-splice cell edges (also
		 * the in-place slot proxy + its run edges, FT_GRAFT_RUN_FLIP_CAP).
		 * Created before the build only so its lifecycle is co-located here;
		 * the build records nothing into it -- the commit replays g->deferred
		 * (and the store reserves its slot proxy) through it post-drain.
		 */
		glue.txn = ft_flip_txn_take(pre_txn);
		if (!glue.txn) {
			glue.txn = ft_flip_txn_create();
			if (!glue.txn || !urcu_flip_txn_reserve(glue.txn,
					FT_GLUE_FLOOR_DEFERRED + 6)) {
				if (glue.txn)
					urcu_flip_txn_destroy(glue.txn);
				free_cds_ft_node(src_ft, fresh_node);
				return CDS_FT_STATUS_MEMORY_ERROR;
			}
		}
		prep = ft_graft_build(dst_ft, key, key_len, graft_payload,
				src_count, &d, &glue);
		if (prep == FT_GRAFT_PREP_OOM) {
			ft_glue_abort(dst_ft, &glue);
			if (glue.txn)
				urcu_flip_txn_destroy(glue.txn);
			free_cds_ft_node(src_ft, fresh_node);
			return CDS_FT_STATUS_MEMORY_ERROR;
		}
		if (prep == FT_GRAFT_PREP_POPULATED) {
			if (glue.txn)
				urcu_flip_txn_destroy(glue.txn);
			free_cds_ft_node(src_ft, fresh_node);
			return CDS_FT_STATUS_POPULATED_ERROR;
		}
		/*
		 * NOSPLIT graft point already occupied (the store would report this
		 * post-swap): surface POPULATED here, BEFORE the source-root swap, so
		 * the source stays pristine -- no rollback, no reader-observable
		 * empty-then-full flicker.  Same condition the store checks at
		 * d->depth == key_len.
		 */
		if (prep == FT_GRAFT_PREP_NOSPLIT && d.depth == key_len && d.nf) {
			urcu_flip_txn_destroy(glue.txn);
			free_cds_ft_node(src_ft, fresh_node);
			return CDS_FT_STATUS_POPULATED_ERROR;
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
		if (prep == FT_GRAFT_PREP_NOSPLIT && !dst_ft->active_reserve) {
			memset(&graft_reserve, 0, sizeof(graft_reserve));
			if (ft_bulk_node_reserve_fill(dst_ft, &graft_reserve)) {
				cds_ft_alloc_reserve_drain(dst_ft, &graft_reserve);
				ft_glue_abort(dst_ft, &glue);
				urcu_flip_txn_destroy(glue.txn);
				free_cds_ft_node(src_ft, fresh_node);
				return CDS_FT_STATUS_MEMORY_ERROR;
			}
			self_secured = true;
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
		rcu_assign_pointer(src_ft->root, ft_node_flag(fresh_node, 0));
		FT_TP(root_publish, (const void *) src_ft,
			(const void *) src_ft->root);

		/*
		 * Ordered list: capture src's whole list (the run to graft) and
		 * unlink it from src here, paired with the structural src-root
		 * unlink, so the synchronize_rcu below drains src ord-readers too.
		 * The run is spliced into dst after the structural publish (same
		 * commit point).  Restored on the OOM rollback below.
		 */
		if (dst_ft->group->ordered_list_set) {
			graft_run_first = src_ft->ord_cell_head;
			graft_run_last = src_ft->ord_cell_tail;
			src_ft->ord_cell_head = NULL;
			src_ft->ord_cell_tail = NULL;
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
			src_ft->group->flavor->update_synchronize_rcu();

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
			 */
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

			if (self_secured)
				cds_ft_alloc_reserve_activate(dst_ft,
					&graft_reserve);
			status = ft_store_at_graft_point(dst_ft, key, key_len,
							  &d, graft_payload,
							  src_count,
							  &attached_nf,
							  &attached_depth,
							  &glue, run_arg);
			if (self_secured) {
				cds_ft_alloc_reserve_deactivate(dst_ft);
				cds_ft_alloc_reserve_drain(dst_ft, &graft_reserve);
			}
			assert(status == CDS_FT_STATUS_OK);
			(void) status;
		}

		/*
		 * Propagate src_count up the ancestor chain, starting
		 * from @attached_nf's parent (skipping @attached_nf
		 * itself, whose nr_keys is already the payload count).
		 *
		 * Note: *d.pnfp can't be used as the start because under
		 * SKIP_COMPRESSED, ft_publish_to_parent may have updated
		 * the grandparent slot (via cn's skip_slot mechanism) to
		 * point directly at @attached_nf -- starting propagation
		 * there would double-count @attached_nf's subtree.
		 */
		{
			/*
			 * ft_get_parent_rcu (not ft_flag_to_metadata) so the start
			 * point is correct even when @attached_nf is the placed
			 * EXTERNAL of a NIL-key graft.
			 */
			struct cds_ft_inode_flag *ap = ft_get_parent_rcu(dst_ft,
				ft_resolve_skip_compressed(dst_ft, attached_nf));
			if (ap)
				ft_propagate_external_count_parent(dst_ft, ap,
					(long) src_count);
		}

		/*
		 * Ordered list: src is now structurally empty + drained; the payload
		 * is published under @key in dst.  Every ordered graft shape (GLUE,
		 * displaced-external, in-place slot) FUSES the run-splice into its
		 * structural flip (@armed), so a reader never sees the run in one
		 * index but not the other.  The standalone two-commit splice remains
		 * as a defensive fallback for any not-yet-fused shape (none today);
		 * without it an unfused shape would strand the run out of the list.
		 */
		if (graft_run_first && !graft_run.armed)
			ft_ord_cell_run_splice(dst_ft, graft_run_first,
				graft_run_last, graft_pred, graft_succ);

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
				ft_descent_traverse_compressed(d, cn, &ik);
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

		/*
		 * Swap both roots and (mirroring them) both ordered lists.  Fuse
		 * EACH side's root swap with its ordered-list head/tail swap into
		 * ONE flip (ft_root_list_swap_publish), so a reader never sees a
		 * side's structure already showing the NEW content while its
		 * ordered-list front is still the OLD -- the root graft_swap
		 * cross-view window.  Capture the swap root and all four endpoints
		 * up front: the two flips reference each other's pre-swap values.
		 */
		if (dst_ft->group->ordered_list_set) {
			struct cds_ft_inode_flag *swap_root = swap_ft->root;
			struct ft_ord_cell *dh = dst_ft->ord_cell_head;
			struct ft_ord_cell *dt = dst_ft->ord_cell_tail;
			struct ft_ord_cell *sh = swap_ft->ord_cell_head;
			struct ft_ord_cell *st = swap_ft->ord_cell_tail;

			ft_root_list_swap_publish(dst_ft, &dst_ft->root,
				tmp, swap_root, dh, sh, dt, st);
			ft_root_list_swap_publish(swap_ft, &swap_ft->root,
				swap_root, tmp, sh, dh, st, dt);
		} else {
			rcu_assign_pointer(dst_ft->root, swap_ft->root);
			rcu_assign_pointer(swap_ft->root, tmp);
		}
		FT_TP(root_publish, (const void *) dst_ft,
			(const void *) dst_ft->root);
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
		enum ft_graft_swap_case kase;
		struct cds_ft_metadata *pmeta, *swap_rmeta;
		struct cds_ft_inode_flag *old_child, *old_swap_root;
		struct cds_ft_inode *fresh = NULL;
		struct cds_ft_metadata *fresh_meta = NULL;
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

		/*
		 * Read-only descent: nothing is published, so the whole swap can be
		 * assembled as a build-invisible transaction and an allocation failure
		 * leaves both tries pristine.
		 */
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

			FT_TP(graft_swap_exit, (int) s);
			return s;
		}

		old_swap_root = swap_ft->root;
		swap_rmeta = ft_root_metadata(swap_ft);
		swap_empty = (swap_rmeta->nr_child == 0 && !swap_rmeta->external_nodes);
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
				merged_meta->nr_child = 1;
				ft_nr_keys_store(merged_meta,
					ft_nr_keys_get(pcn_meta), CMM_RELAXED);
				merged_meta->parent = pcn_meta->parent;
				pub_parent = pcn_meta->parent;
				pub_slot = ft_get_parent_slot(pcn_meta, dst_ft);
				ft_set_parent_slot(merged_meta,
					merged_meta->parent, pub_slot);
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
				d.pnf = merged_flag;	/* count updates land on merged */
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
		 * pointer not yet classified for the txn; keep it on the legacy
		 * apply-deferred + publish-replace path (glue_insert.txn == NULL
		 * selects it) until that wrap re-parent is folded in.
		 */
		if (have_insert && kase != FT_GRAFT_SWAP_KEY_SHORTER) {
			glue_insert.txn = ft_flip_txn_create();
			if (!glue_insert.txn || !urcu_flip_txn_reserve(glue_insert.txn,
					FT_GLUE_FLOOR_DEFERRED + 6))
				goto prep_oom;
		}

		/* Transient empty swap root for the unlink window (fallible). */
		if (!swap_empty) {
			fresh = alloc_cds_ft_node(swap_ft, &ft_types[0], &fresh_meta);
			if (!fresh)
				goto prep_oom;
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
			gs_s_first = swap_ft->ord_cell_head;	/* NULL if swap empty */
			gs_s_last = swap_ft->ord_cell_tail;
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
			rcu_assign_pointer(swap_ft->root, ft_node_flag(fresh, 0));
			FT_TP(root_publish, (const void *) swap_ft,
				(const void *) swap_ft->root);
			/*
			 * run_S is captured; unlink it from swap's ordered list here
			 * (paired with the structural root unlink) so this sync drains
			 * swap ord-readers of run_S too.  run_D is installed as swap's
			 * list after the extract publish below.
			 */
			if (gs_ord) {
				swap_ft->ord_cell_head = NULL;
				swap_ft->ord_cell_tail = NULL;
			}
			if (!swap_ft->exclusive)
				swap_ft->group->flavor->update_synchronize_rcu();
		}

		/*
		 * Insert side: wire the deferred live back-pointers, then the single
		 * forward publish that splices cluster A into dst (detaching the old
		 * content).  Empty swap publishes NULL (a remove).
		 */
		if (have_insert) {
			if (glue_insert.txn)
				ft_glue_txn_commit_replace(dst_ft, &glue_insert,
					swap_run_arg);
			else {
				ft_glue_apply_deferred(dst_ft, &glue_insert);
				ft_glue_publish_replace(dst_ft, &glue_insert,
					swap_run_arg);
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
			ft_propagate_external_count_parent(dst_ft, d.pnf,
					-(long) old_count);
			cds_ft_alloc_reserve_activate(dst_ft, &gs_reserve);
			dret = ft_detach_node(dst_ft, d.nfp, d.pnfp, d.depth,
					false, NULL, gs_ord ? &dpub : NULL,
					gs_ord ? &drun : NULL);
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
				pmeta->nr_child--;
			if (swap_count != old_count)
				ft_propagate_external_count_parent(dst_ft, d.pnf,
						(long) swap_count - (long) old_count);
		}

		/*
		 * Replace run_D with run_S in dst's ordered list (run_S now lives at
		 * @key structurally; run_S NULL for an empty swap -> run_D just
		 * leaves).  Paired with the dst-side drain below, which removes any
		 * reader still holding run_D in dst.  Skipped when the publish above
		 * already FUSED it into the structural flip (swap_run.armed): only the
		 * gs_reserved sole-child detach-prune path still uses this standalone
		 * form (its prune commits separately via ft_detach_node).
		 */
		if (gs_ord && !swap_run.armed)
			ft_ord_cell_run_replace(dst_ft, gs_d_first, gs_d_last,
				gs_s_first, gs_s_last);

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
		if (!dst_ft->exclusive)
			dst_ft->group->flavor->update_synchronize_rcu();

		/*
		 * Extract side: wire cluster B's deferred back-pointer, then install
		 * swap_ft's new root.  This re-parents the displaced subtree AFTER it
		 * has been detached from dst by the publish above and after the
		 * dst-side drain above.
		 */
		ft_glue_apply_deferred(dst_ft, &glue_extract);
		if (top_B) {
			struct cds_ft_metadata *bm =
				cds_ft_item_to_metadata(ft_node_ptr(top_B));

			rcu_assign_pointer(bm->parent, NULL);
#ifdef FEATURE_FT_SKIP_COMPRESSED
			bm->parent_slot_offset = 0;
#endif
			rcu_assign_pointer(swap_ft->root, top_B);
			FT_TP(root_publish, (const void *) swap_ft,
				(const void *) swap_ft->root);
			if (swap_empty)
				free_cds_ft_node(swap_ft, ft_node_ptr(old_swap_root));
			else
				free_cds_ft_node(swap_ft, fresh);
		} else {
			/*
			 * External (or absent) displaced content: attach it as
			 * external_nodes on swap_ft's root (the transient @fresh for a
			 * non-empty swap, or old_swap_root's empty root for an empty swap).
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
				ft_nr_keys_store(rm, old_count, CMM_RELEASE);
			}
		}

		/*
		 * Install run_D (the extracted subtree's heads) as swap_ft's whole
		 * ordered list, mirroring the extract root publish above.  swap_ft
		 * was drained at the unlink sync, so clearing run_D's boundary links
		 * is a plain store; the head/tail publish uses rcu_assign.
		 */
		if (gs_ord) {
			gs_d_first->ord_prev = NULL;
			gs_d_last->ord_next = NULL;
			rcu_assign_pointer(swap_ft->ord_cell_head, gs_d_first);
			rcu_assign_pointer(swap_ft->ord_cell_tail, gs_d_last);
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
		ft_glue_abort(dst_ft, &glue_insert);
		ft_glue_abort(swap_ft, &glue_extract);
		if (glue_insert.txn)
			urcu_flip_txn_destroy(glue_insert.txn);
		if (fresh)
			free_cds_ft_node(swap_ft, fresh);
		if (gs_reserved)
			cds_ft_alloc_reserve_drain(dst_ft, &gs_reserve);
		FT_TP(graft_swap_exit, (int) CDS_FT_STATUS_MEMORY_ERROR);
		return CDS_FT_STATUS_MEMORY_ERROR;
	}
}

