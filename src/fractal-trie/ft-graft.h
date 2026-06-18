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
 * Build (invisibly) a swap_ft root node holding the extracted subtree
 *   @first_byte ++ @rest[0 .. @rest_len-1]  ->  @child
 * i.e. an internal root whose @first_byte slot leads (via a fresh compressed
 * suffix when @rest_len >= 1) to the LIVE @child.  Used (via
 * ft_make_root_internal_glue) by the extract side of cds_ft_graft_swap and by
 * ft_detach_keylen's pre-publish root materialization.
 *
 * @child is LIVE data relocated into swap_ft: its back-pointer is recorded as
 * a deferred edge in @glue rather than flipped now, and every fresh node is
 * tracked so an OOM elsewhere in the swap build frees the cluster (both tries
 * pristine, nothing published).  Nothing is freed here.
 *
 * @child is always a plain (internal / external) node -- a compressed node's
 * child is plain by the chain-merge invariant, and the suffix path is held by
 * the fresh compressed @rest -- so no chain-merge is needed.
 *
 * Returns the new internal-tagged root flag, or (void *)(long)-ENOMEM.
 */
static
struct cds_ft_inode_flag *ft_build_extracted_root_glue(struct cds_ft *ft,
		struct ft_graft_glue *glue,
		uint8_t first_byte, const uint8_t *rest, unsigned int rest_len,
		struct cds_ft_inode_flag *child, unsigned long subtree_count)
{
	struct cds_ft_inode_flag *slot_value;
	struct cds_ft_inode_flag *skip_value = NULL;
	struct cds_ft_compressed_node *new_cn = NULL;
	struct cds_ft_inode *root_node;
	struct cds_ft_metadata *root_meta;
	struct cds_ft_inode_flag *dest;
	struct cds_ft_inode_flag **slot = NULL;
	int ret;

	if (rest_len == 0) {
		slot_value = child;	/* LIVE; back-pointer deferred below. */
	} else {
		struct cds_ft_metadata *new_cn_meta;

		new_cn = alloc_compressed_node(ft, rest_len, &new_cn_meta);
		if (!new_cn)
			return (struct cds_ft_inode_flag *) (long) -ENOMEM;
		new_cn->len = (uint8_t) rest_len;
		new_cn->child = child;
		memcpy(new_cn->key_bytes, rest, rest_len);
		new_cn_meta->nr_child = 1;
		ft_nr_keys_store(new_cn_meta, subtree_count, CMM_RELAXED);
		slot_value = ft_compressed_node_flag(new_cn);	/* PLAIN */
		ft_graft_glue_track(glue, slot_value);
		/* @child (live) -> new_cn, deferred to the post-sync commit. */
		ft_graft_glue_defer_edge(ft, glue, child, slot_value, &new_cn->child);
		/* Skip form for the root slot (resolves once the edge applies). */
		skip_value = ft_publish_compressed(ft, new_cn, slot_value);
	}

	root_node = alloc_cds_ft_node(ft, &ft_types[0], &root_meta);
	if (!root_node)
		/* new_cn (if any) is tracked in @glue; the caller's abort frees it. */
		return (struct cds_ft_inode_flag *) (long) -ENOMEM;
	dest = ft_node_flag(root_node, 0);
	/*
	 * rest_len == 0: @child is live, so defer its back-pointer (cluster_leaf).
	 * rest_len >= 1: the slot holds the fresh new_cn, whose own back-pointer
	 * into @dest is a fresh-to-fresh edge that is safe to set during the build.
	 */
	ret = ft_node_set_nth(ft, &dest, first_byte, slot_value,
			NULL, root_meta, 0, rest_len == 0 /* cluster_leaf */);
	if (ret)
		return (struct cds_ft_inode_flag *) (long) -ENOMEM;
	ft_graft_glue_track(glue, dest);
	ft_nr_keys_store(ft_flag_to_metadata(ft, dest), subtree_count, CMM_RELAXED);
	ft_node_get_nth_skip(dest, &slot, first_byte, FT_PF_NONE);
	if (rest_len == 0) {
		ft_graft_glue_defer_edge(ft, glue, child, dest, slot);
	} else {
		/* Re-encode the root slot to the skip form (new_cn is compressed). */
		if (skip_value && skip_value != slot_value && slot)
			rcu_assign_pointer(*slot, skip_value);
		ft_set_parent(ft, slot_value, dest, slot);
	}
	return dest;
}

/*
 * Build-invisible internal-root materialization, preserving the trie-wide
 * invariant that the root pointer always tags an internal node (never
 * compressed, never skip-compressed).  Used by cds_ft_graft_swap's extract
 * side and by ft_detach_keylen.  Materializes an internal-node root from the
 * LIVE displaced @old_child without publishing or mutating live data: fresh nodes are tracked in @glue, the moved grandchild's
 * back-pointer is deferred, and the peeled-away compressed node is recorded for
 * deferred free.  Nothing is freed here.
 *
 *   - @old_child internal:    returned unchanged (already a valid internal
 *                             root; the caller clears its parent at commit).
 *                             No glue node, no deferred edge.
 *   - @old_child compressed:  peel the first path byte into a fresh internal
 *                             root, the rest (if any) into a fresh compressed
 *                             node; defer the live grandchild's back-pointer;
 *                             record the old compressed node for deferred free.
 *
 * External @old_child must be filtered by the caller (externals attach as
 * external_nodes, not via this helper).
 *
 * Returns the new internal-tagged root flag, or (void *)(long)-ENOMEM.
 */
static
struct cds_ft_inode_flag *ft_make_root_internal_glue(struct cds_ft *ft,
		struct ft_graft_glue *glue, struct cds_ft_inode_flag *old_child)
{
	struct cds_ft_compressed_node *cn;
	struct cds_ft_metadata *cn_meta;
	struct cds_ft_inode_flag *root;

	old_child = ft_resolve_skip_compressed(ft, old_child);
	if (caa_likely(!ft_node_compressed(old_child)))
		return old_child;	/* already internal */
	cn = ft_compressed_node_ptr(old_child);
	cn_meta = cds_ft_item_to_metadata((struct cds_ft_inode *) cn);
	root = ft_build_extracted_root_glue(ft, glue,
			cn->key_bytes[0], &cn->key_bytes[1], cn->len - 1,
			cn->child, ft_nr_keys_get(cn_meta));
	if (root == (struct cds_ft_inode_flag *) (long) -ENOMEM)
		return root;
	/* Reclaim the peeled-away compressed node after the commit. */
	ft_graft_glue_defer_free(glue, cn, true);
	return root;
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

		rcu_assign_pointer(dst_ft->root, swap_ft->root);
		FT_TP(root_publish, (const void *) dst_ft,
			(const void *) dst_ft->root);
		rcu_assign_pointer(swap_ft->root, tmp);
		FT_TP(root_publish, (const void *) swap_ft,
			(const void *) swap_ft->root);

		/* Ordered list: swap whole lists (head/tail), mirroring the roots. */
		if (dst_ft->group->ordered_list_set) {
			struct ft_ord_cell *dh = dst_ft->ord_cell_head;
			struct ft_ord_cell *dt = dst_ft->ord_cell_tail;

			rcu_assign_pointer(dst_ft->ord_cell_head,
				swap_ft->ord_cell_head);
			rcu_assign_pointer(dst_ft->ord_cell_tail,
				swap_ft->ord_cell_tail);
			rcu_assign_pointer(swap_ft->ord_cell_head, dh);
			rcu_assign_pointer(swap_ft->ord_cell_tail, dt);
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
		struct ft_graft_glue glue_insert, glue_extract;
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

		ft_graft_glue_init(&glue_insert);
		ft_graft_glue_init(&glue_extract);

		/* ===== PREP: build clusters A and B (both tries pristine) ===== */

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
				ft_set_parent_slot(merged_meta, pub_slot);
				merged_flag = ft_compressed_node_flag(merged);
				ft_graft_glue_track(&glue_insert, merged_flag);
				ft_graft_glue_defer_edge(dst_ft, &glue_insert, ccn->child,
					merged_flag, &merged->child);
				/*
				 * @canon is the fresh wrapper just absorbed: drop it from
				 * tracking and free it (its deferred child edge is superseded
				 * by the one above via the defer-edge de-dup on @child).
				 */
				ft_graft_glue_untrack(dst_ft, &glue_insert, ccn);
				free_compressed_node_unpublished(dst_ft, ccn);
				merged_skip = ft_publish_compressed(dst_ft, merged,
						merged_flag);
				ft_graft_glue_set_publish(dst_ft, &glue_insert, pub_parent,
					pub_slot, merged_skip);
				ft_graft_glue_defer_free(&glue_insert, pcn, true);
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
				ft_graft_glue_defer_free(&glue_insert, ks_cn, true);
			} else {
				/* EXACT, simple replace of d.nf at d.nfp by @canon. */
				top_A = canon;
			}
			/*
			 * A compressed cluster top installs as the SKIP form in the live
			 * parent slot; its child's back-pointer is deferred, so the skip
			 * only resolves once ft_graft_glue_apply_deferred has run -- which
			 * it does (before the forward publish) at commit.  The set_publish
			 * deferred edge (top -> d.pnf) is recorded LAST, so by the time it
			 * is applied the child back-pointer is already in place.
			 */
			if (ft_node_compressed(top_A))
				top_A = ft_publish_compressed(dst_ft,
					ft_compressed_node_ptr(top_A), top_A);
			ft_graft_glue_set_publish(dst_ft, &glue_insert, d.pnf, d.nfp, top_A);
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
			ft_graft_glue_apply_deferred(dst_ft, &glue_insert);
			ft_graft_glue_publish(dst_ft, &glue_insert);
		} else {
			ft_publish_to_parent(dst_ft, d.pnf, d.nfp, NULL);
		}

		/*
		 * graft_swap edits the subtree at @key via ft_publish_to_parent
		 * directly (no ft_node_set_nth), so emit the structural edge for
		 * consumers.
		 */
		if (d.depth >= 1)
			FT_TP(tree_edge_set, (const void *) dst_ft,
				(const void *) d.pnf,
				(unsigned int) (d.depth - 1),
				(uint8_t) _key[d.depth - 1],
				(const void *) (have_insert ? glue_insert.top : NULL));

		/* Parent nr_child on the non-NULL -> NULL transition (remove). */
		pmeta = cds_ft_item_to_metadata(ft_node_ptr(d.pnf));
		if (!have_insert)
			pmeta->nr_child--;

		/* Propagate the external-count delta through the ancestors. */
		if (swap_count != old_count)
			ft_propagate_external_count_parent(dst_ft, d.pnf,
					(long) swap_count - (long) old_count);

		/*
		 * Replace run_D with run_S in dst's ordered list (run_S now lives at
		 * @key structurally; run_S NULL for an empty swap -> run_D just
		 * leaves).  Paired with the dst-side drain below, which removes any
		 * reader still holding run_D in dst.
		 */
		if (gs_ord)
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
		ft_graft_glue_apply_deferred(dst_ft, &glue_extract);
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
		ft_graft_glue_free_old(dst_ft, &glue_insert);
		ft_graft_glue_free_old(swap_ft, &glue_extract);

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
		FT_TP(graft_swap_exit, (int) CDS_FT_STATUS_OK);
		return CDS_FT_STATUS_OK;

	prep_oom:
		/*
		 * Allocation failed during the build: free every fresh glue node (both
		 * clusters), drop the transient swap root, and surface MEMORY_ERROR.
		 * No deferred edge was applied and nothing was published, so dst_ft and
		 * swap_ft are both pristine -- there is nothing to roll back.
		 */
		ft_graft_glue_abort(dst_ft, &glue_insert);
		ft_graft_glue_abort(swap_ft, &glue_extract);
		if (fresh)
			free_cds_ft_node(swap_ft, fresh);
		FT_TP(graft_swap_exit, (int) CDS_FT_STATUS_MEMORY_ERROR);
		return CDS_FT_STATUS_MEMORY_ERROR;
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
			m->parent_slot_offset = 0;
#endif
		}
		uatomic_store(&detached->max_used_key_len,
			      uatomic_load(&ft->max_used_key_len, CMM_RELAXED),
			      CMM_RELAXED);

		/* Give source a fresh empty root. */
		rcu_assign_pointer(ft->root, ft_node_flag(fresh_node, 0));
		FT_TP(root_publish, (const void *) ft, (const void *) ft->root);

		/*
		 * Ordered list: a root detach moves the WHOLE trie, so @ft's
		 * entire ordered cell list becomes @detached's.  The cells'
		 * internal links are unchanged; only the head/tail endpoints
		 * transfer.  Matches the root-swap above (a concurrent reader
		 * mid-iteration follows its RCU snapshot into @detached).
		 */
		if (ft->group->ordered_list_set) {
			detached->ord_cell_head = ft->ord_cell_head;
			detached->ord_cell_tail = ft->ord_cell_tail;
			ft->ord_cell_head = NULL;
			ft->ord_cell_tail = NULL;
		}

		/*
		 * Drain @ft's readers that entered before the root swap and may
		 * still be inside the moved subtree (or parked in the moved
		 * ordered run): the exclusivity promise on @detached -- which a
		 * subsequent graft relies on to skip ITS grace period, and
		 * which makes mutation frees on @detached SYNCHRONOUS -- must
		 * hold at return, not eventually.  Skip for exclusive sources,
		 * which carry no RCU readers by construction.
		 */
		if (!ft->exclusive)
			ft->group->flavor->update_synchronize_rcu();

		*result_ft = detached;
		return CDS_FT_STATUS_OK;
	}

	/*
	 * key_len > 0: a plain key-guided descent to the detach target
	 * @child.  No branch-point snapshot is tracked here: ft_detach_node
	 * is bootstrapped from @child's own slot and recovers the surviving
	 * ancestor by climbing parent pointers (the same upward walk used by
	 * cds_ft_remove's count==1 prune), so the descent only has to locate
	 * @child, its slot, and its parent.
	 */
	{
		struct ft_descent d;
		const uint8_t *ik = key;

		ft_descent_init(&d, ft);

		for (; d.depth < key_len; ) {
			uint8_t kv;

			if (!d.nf)
				return CDS_FT_STATUS_NOT_FOUND;
			if (ft_node_external(d.nf))
				return CDS_FT_STATUS_NOT_FOUND;
			if (ft_node_compressed(d.nf)) {
				struct cds_ft_compressed_node *cn =
					ft_compressed_node_ptr(d.nf);

				ft_descent_traverse_compressed(&d, cn, &ik);
				continue;
			}
			kv = *(ik++);
			ft_descent_step(ft, &d, kv);
		}

		child = d.nf;

		if (!child)
			return CDS_FT_STATUS_NOT_FOUND;

		/*
		 * Compute the external node count of the subtree
		 * being detached before it is removed from the trie.
		 */
		{
			unsigned long detached_count;
			struct ft_graft_glue glue;
			struct cds_ft_inode_flag *new_root = NULL;

			if (!ft_node_external(child)) {
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
			ft_graft_glue_init(&glue);
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
			 * Materialize the detached trie's internal root NOW,
			 * build-invisibly, while nothing has been published:
			 * the trie root invariant requires an internal node,
			 * but a compressed/skip-compressed @child needs fresh
			 * allocations to peel its first path byte.  This is
			 * the LAST fallible step -- doing it after the detach
			 * publish would have no rollback (the subtree would be
			 * unreachable from both tries: silent data loss).  The
			 * fresh nodes are tracked in @glue, the live
			 * grandchild's back-pointer flip is deferred to the
			 * post-drain commit below, and the peeled compressed
			 * node's free is deferred likewise; an abort leaves
			 * the source pristine.
			 */
			if (!ft_node_external(child)) {
				new_root = ft_make_root_internal_glue(detached,
						&glue, child);
				if (new_root ==
				    (struct cds_ft_inode_flag *) (long) -ENOMEM) {
					ft_graft_glue_abort(detached, &glue);
					cds_ft_destroy(detached);
					return CDS_FT_STATUS_MEMORY_ERROR;
				}
			}

			/*
			 * Propagate count removal through ancestors
			 * before detach to avoid writing freed metadata.
			 */
			ft_propagate_external_count_parent(ft, d.pnf,
				-(long) detached_count);

			/*
			 * Detach child from the source trie and prune
			 * empty branches above.  After this, child is
			 * no longer reachable from the live trie for
			 * new readers.
			 */
			{
				/*
				 * Subtree-move detach (free_detached_subtree
				 * == false): preserve @child as the root of
				 * the new @detached trie.  Bootstrapped from
				 * @child's own slot, ft_detach_node climbs
				 * parent pointers to the surviving ancestor,
				 * unlinks the branch there, and its free-walk
				 * phase 1 reclaims the intermediate single-
				 * child chain between that ancestor and
				 * @child (the @nr_clear elevated links) while
				 * phase 2 -- which would free @child and
				 * below -- is gated off for move-style.  No
				 * explicit chain reclaim is needed here.
				 */
				int ret = ft_detach_node(ft,
							 d.nfp,
							 d.pnfp,
							 d.depth,
							 false);
				assert(ret != -ENOENT);
				if (ret < 0) {
					/*
					 * Recompaction failed (-ENOMEM).
					 * Undo propagation and abort.  The
					 * glue cluster is still invisible:
					 * the abort leaves @ft pristine.
					 */
					ft_propagate_external_count_parent(ft,
						d.pnf,
						(long) detached_count);
					ft_graft_glue_abort(detached, &glue);
					cds_ft_destroy(detached);
					return CDS_FT_STATUS_MEMORY_ERROR;
				}
			}

			/*
			 * Ordered list: the detached subtree's keys form a
			 * contiguous run in @ft's ordered cell list.  Move that
			 * run out of @ft and install it as @detached's entire
			 * list.  @child's subtree is intact (move-style detach),
			 * so its structural min/max heads are the run endpoints
			 * (the detach-point external_nodes, if any, are the run
			 * minimum -- they become @detached's NIL-key entries).
			 * The flip is atomic for a concurrent ordered reader; the
			 * internal-child branch's synchronize_rcu below then drains
			 * any @ft reader parked in the run.
			 */
			if (ft->group->ordered_list_set) {
				struct cds_ft_node *rfirst =
					ft_subtree_minmax_head(ft, child, false);
				struct cds_ft_node *rlast =
					ft_subtree_minmax_head(ft, child, true);

				ft_ord_cell_run_detach(ft, detached, rfirst, rlast);
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
			if (!ft_node_external(child)) {
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
				/*
				 * COMMIT (failure-free): the internal root was
				 * materialized build-invisibly BEFORE the
				 * detach published anything (see the
				 * ft_make_root_internal_glue call above).
				 * Wire the deferred live back-pointer (the
				 * grandchild moved under the fresh cluster --
				 * safe now, the drain above guarantees no
				 * reader still up-walks from inside the
				 * subtree), install the root, then reclaim the
				 * peeled-away compressed node.
				 */
				ft_graft_glue_apply_deferred(detached, &glue);
				free_cds_ft_node(detached,
					ft_node_ptr(detached->root));
				/* No readers in detached root yet. */
				detached->root = new_root;
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
						ft_node_ptr(new_root));
					rcu_assign_pointer(m->parent, NULL);
#ifdef FEATURE_FT_SKIP_COMPRESSED
					m->parent_slot_offset = 0;
#endif
				}
				ft_graft_glue_free_old(detached, &glue);
				ft_graft_glue_fini(&glue);
			} else {
				struct cds_ft_metadata *dmeta =
					ft_root_metadata(detached);

				/*
				 * Drain source-trie readers here too: they may
				 * still be parked on the detached external
				 * chain (a lookup that returned the head, a
				 * duplicate-chain walk) or in the moved ordered
				 * run.  The exclusivity promise on @detached
				 * must hold AT RETURN -- a subsequent graft of
				 * @detached legitimately skips its own grace
				 * period, and exclusive-mode mutations free
				 * SYNCHRONOUSLY, so a parked reader would
				 * dereference freed memory.  The drain also
				 * precedes the prev re-point below, so no
				 * reader's up-walk from the chain head can
				 * escape into @detached's root.  Skip for
				 * exclusive sources (no RCU readers by
				 * construction).
				 */
				if (!ft->exclusive)
					ft->group->flavor->update_synchronize_rcu();
				ft_metadata_set_external_nodes(detached->root, dmeta,
					(struct cds_ft_node *)
					ft_node_ptr(child));
				/*
				 * detached->root: parent is legitimately NULL.
				 * Pair the prev publish for consistency.
				 */
				ft_publish_external_nodes_prev(ft, detached->root,
					(struct cds_ft_node *)
					ft_node_ptr(child));
				ft_nr_keys_store(dmeta, detached_count, CMM_RELAXED);
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

