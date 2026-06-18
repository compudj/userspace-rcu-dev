// SPDX-FileCopyrightText: 2012-2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-only

/*
 * src/fractal-trie/ft-remove.h
 *
 * Userspace RCU library - Fractal Trie: remove / remove_all and the detach-node recompaction.
 *
 * Implementation unit: #included once by fractal-trie.c, in dependency
 * order, into a single translation unit (preserves cross-module inlining).
 * Not a standalone header.
 */
#ifndef FRACTAL_TRIE_IMPL
#error "ft-remove.h is an implementation unit; #include it from fractal-trie.c only"
#endif

enum cds_ft_status cds_ft_remove(struct cds_ft *ft,
		struct cds_ft_iter *iter,
		struct cds_ft_node *node)
{
	struct cds_ft_inode_flag *holder_flag;
	struct cds_ft_metadata *holder_meta;
	struct cds_ft_inode_flag **head_slot = NULL;
	const uint8_t *iter_key;
	size_t key_len = ft_key_len(ft, ft_iter_resolve_key_len(iter));
	int ret;

	CDS_FT_SCOPED_WRITER(ft);
	FT_TP(remove_enter, (const void *) ft, (const void *) iter,
		iter_key(iter), key_len);

	/*
	 * If the iterator has a valid path, the RCU read-side lock must
	 * be held.
	 */
	if (iter->cache_valid)
		CDS_FT_ASSERT_RCU_READ_LOCKED(ft);
	iter_debug_path_check(iter);

	if (!valid_external_node(node) || !valid_key_len(ft, key_len)) {
		FT_TP(remove_exit, (int) CDS_FT_STATUS_INVALID_ARGUMENT_ERROR);
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}

	iter_key = ft_iter_read_key(iter);
	dbg_printf("cds_ft_remove attempt: node %p\n", node);

	/*
	 * No top-down descent.  @node is application-owned and, with the RCU
	 * read-side lock held continuously since it was obtained, stays
	 * alive; the writer mutex held here freezes the structure, so
	 * node->prev is a settled live pointer to the node's holder and the
	 * slot that holds @node can be derived directly:
	 *
	 *  - INTERNAL ancestors recover their parent slot from their own
	 *    metadata (parent + parent_slot_offset), used by ft_detach_node's
	 *    upward prune walk.
	 *  - the metadata-less EXTERNAL head's slot in its holder is
	 *    re-derived from the key here (a compressed holder's &cn->child,
	 *    or an internal holder's body slot for the key's last byte).
	 */

	/*
	 * A removed node carries the tombstone on node->next (set by a prior
	 * unchain/detach).  Re-removing it is an idempotent miss; this also
	 * guards against operating on a node already unlinked from the trie.
	 */
	if (ft_node_is_removed(node)) {
		dbg_printf("cds_ft_remove: node %p already removed\n", node);
		FT_TP(remove_exit, (int) CDS_FT_STATUS_NOT_FOUND);
		return CDS_FT_STATUS_NOT_FOUND;
	}

	/*
	 * Resolve @node's holder (its parent), transparently across the cell
	 * indirection: a head's prev is its cell (parent in cell->parent), a
	 * non-head duplicate's prev is its predecessor.  NULL => never inserted.
	 */
	holder_flag = ft_node_holder(ft, node);
	if (!holder_flag) {
		/* Never inserted (a freshly-initialized node). */
		dbg_printf("cds_ft_remove: node %p has no parent\n", node);
		FT_TP(remove_exit, (int) CDS_FT_STATUS_NOT_FOUND);
		return CDS_FT_STATUS_NOT_FOUND;
	}

	/*
	 * Cell-always: @node heads its chain iff its prev is the cell (not an
	 * external predecessor).  Capture the head's cell + successor BEFORE the
	 * unlink: a head promotion retargets the cell at the successor (done in
	 * ft_unchain_node), and a key disappearance (no successor) frees the
	 * cell after the removal commits (ret == 0).
	 */
	bool cell_was_head = ft->ordered_list &&
		!ft_node_external((struct cds_ft_inode_flag *) node->prev);
	struct ft_ord_cell *dead_cell = cell_was_head ?
		ft_ord_cell_ptr(node->prev) : NULL;
	struct cds_ft_node *cell_succ = cell_was_head ? ft_node_next(node) : NULL;

	if (ft_node_external(holder_flag)) {
		/*
		 * node->prev is a cds_ft_node: @node is a non-head duplicate.
		 * Unlink it from its chain (ft_unchain_node relinks the pinned
		 * predecessor/successor and tombstones @node).  The key count is
		 * unchanged (other duplicates remain) and the chain head -- and
		 * any grandparent skip pointer to it -- is untouched, so no head
		 * slot is needed.
		 */
		ft_unchain_node(ft, NULL, node);
		ret = 0;
	} else if (ft_node_compressed(holder_flag) ||
		   ft_node_skip_compressed(holder_flag)) {
		/*
		 * Compressed holder: @node is its single external child
		 * (cn->child).  Leaf key.
		 */
		struct cds_ft_compressed_node *cn =
			ft_node_skip_compressed(holder_flag) ?
				ft_skip_to_compressed(ft, holder_flag) :
				ft_compressed_node_ptr(holder_flag);

		holder_meta = cds_ft_item_to_metadata((struct cds_ft_inode *) cn);
		head_slot = &cn->child;
		if ((struct cds_ft_node *) ft_node_ptr(*head_slot) != node) {
			dbg_printf("cds_ft_remove: node %p not at compressed child slot\n", node);
			FT_TP(remove_exit, (int) CDS_FT_STATUS_NOT_FOUND);
			return CDS_FT_STATUS_NOT_FOUND;
		}
		if (!ft_node_next(node)) {
			/*
			 * Last/only entry: prune the now-empty branch.
			 * ft_detach_node bootstraps from the holder slot (recovered
			 * from the holder's own metadata offset) and walks up via
			 * metadata->parent.  Propagate -1 before detach, which may
			 * free internal nodes.
			 */
			ft_propagate_external_count_parent(ft, holder_flag, -1);
			ret = ft_detach_node(ft, head_slot,
				ft_get_parent_slot(holder_meta, ft),
				key_len, true);
			if (ret)
				ft_propagate_external_count_parent(ft, holder_flag, 1);
			else
				ft_node_mark_removed(node);
		} else {
			/* Removing the head, duplicates remain: key count unchanged. */
			ft_unchain_node(ft, (struct cds_ft_node **) head_slot, node);
#ifdef FEATURE_FT_SKIP_COMPRESSED
			/*
			 * Unchaining replaced cn->child with the next entry, but
			 * the grandparent's skip-compressed slot still encodes the
			 * OLD head, which the caller is about to call_rcu-free.  A
			 * candidate descent or ft_skip_reanchor up-walk following
			 * the stale skip pointer would dereference the freed node
			 * (the dangling-skip-slot UAF).  Re-encode the grandparent
			 * slot (recovered from cn's parent-slot offset) to the new
			 * cn->child; ordered before the caller's free.
			 * ft_update_skip_pointer no-ops when the slot holds a plain
			 * (non-skip) compressed pointer.
			 */
			ft_update_skip_pointer(ft_get_parent_slot(holder_meta, ft),
				cn);
#endif
			ret = 0;
		}
	} else if (ft_node_external_nodes(holder_flag) ==
			(struct cds_ft_node *) node) {
		/*
		 * Internal holder whose external_nodes chain head is @node:
		 * prefix key (the key terminates at an internal node that also
		 * has longer-key children).  The holder stays; unlink @node from
		 * its external_nodes chain.  Propagate -1 only when @node is the
		 * last entry (the chain becomes empty).
		 */
		holder_meta = cds_ft_item_to_metadata(ft_node_ptr(holder_flag));
		if (!ft_node_next(node))
			ft_propagate_external_count_parent(ft, holder_flag, -1);
		ft_unchain_node(ft, (struct cds_ft_node **) &holder_meta->external_nodes,
			node);
#ifdef FEATURE_FT_SKIP_COMPRESSED
		/*
		 * If the unchain emptied the external chain and the holder now
		 * has exactly one child + no external, canonicalize via
		 * chain-compress to restore the SKIP_COMPRESSED invariant (no
		 * non-root 1-child internal without external).  The holder's own
		 * slot in its parent is recovered from its metadata offset.
		 */
		if (ft_group_skip_compressed(ft->group) &&
		    !holder_meta->external_nodes &&
		    holder_meta->nr_child == 1 &&
		    holder_meta->parent != NULL) {
			ft_canonicalize_chain_compress(ft, holder_flag,
				holder_meta, ft_get_parent_slot(holder_meta, ft));
		}
#endif
		ret = 0;
	} else {
		/*
		 * Internal holder, @node is a body child: leaf key.  Recover the
		 * holder's body slot for @node from the key's last byte.
		 */
		struct cds_ft_inode_flag *child;

		holder_meta = cds_ft_item_to_metadata(ft_node_ptr(holder_flag));
		child = ft_node_get_nth_skip(holder_flag, &head_slot,
			iter_key[key_len - 1], FT_PF_NONE);
		if (!child ||
		    (struct cds_ft_node *) ft_node_ptr(child) != node) {
			dbg_printf("cds_ft_remove: node %p not at key slot\n", node);
			FT_TP(remove_exit, (int) CDS_FT_STATUS_NOT_FOUND);
			return CDS_FT_STATUS_NOT_FOUND;
		}
		if (!ft_node_next(node)) {
			/*
			 * Last/only entry: prune the now-empty branch.
			 * Propagate -1 before detach, which may free internal nodes.
			 */
			ft_propagate_external_count_parent(ft, holder_flag, -1);
			ret = ft_detach_node(ft, head_slot,
				ft_get_parent_slot(holder_meta, ft),
				key_len, true);
			if (ret)
				ft_propagate_external_count_parent(ft, holder_flag, 1);
			else
				ft_node_mark_removed(node);
		} else {
			/* Removing the head, duplicates remain: key count unchanged. */
			ft_unchain_node(ft, (struct cds_ft_node **) head_slot, node);
			ret = 0;
		}
	}

	/*
	 * Head with no successor: the key disappeared, so its cell is unspliced
	 * from the ordered list (when enabled) and freed (deferred, for parked
	 * up-walkers).  A promotion (cell_succ) keeps the cell in place -- same
	 * key, only cell->node retargeted in ft_unchain_node -- so no list op.
	 */
	if (ret == 0 && cell_was_head && !cell_succ) {
		/* cell_was_head implies ordered_list, so the list op always runs. */
		ft_ord_cell_unsplice(ft, dead_cell);
		ft_ord_cell_free(ft, dead_cell);
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
	iter->cache_valid = false;
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

/*
 * ft_locate_chain_head: derive an external chain head's position with no
 * descent, from the head itself (head->prev is its holder) and the key.
 *
 * Sets *holder_flag_p (the head's internal/compressed holder), *is_prefix_p
 * (true when the chain hangs off the holder's external_nodes -- a prefix key --
 * rather than a body/compressed child slot), and, for the non-prefix case,
 * *head_slot_p (the holder slot that holds @head: &cn->child for a compressed
 * holder, or the body slot for the key's last byte for an internal holder).
 *
 * Returns true iff @head is the live chain head at that position (so a cached
 * iter->node is still current under the writer mutex): the prefix case is
 * validated by holder->external_nodes == head; the leaf case by *head_slot ==
 * head.  Returns false when @head is a non-head duplicate, has no holder, or
 * the holder no longer points at it (stale cache) -- the caller then re-seeds
 * via a fresh lookup.
 */
static
bool ft_locate_chain_head(struct cds_ft *ft, struct cds_ft_node *head,
		const uint8_t *iter_key, size_t key_len,
		struct cds_ft_inode_flag **holder_flag_p,
		struct cds_ft_inode_flag ***head_slot_p,
		bool *is_prefix_p)
{
	struct cds_ft_inode_flag *holder_flag = ft_node_holder(ft, head);

	if (!holder_flag || ft_node_external(holder_flag))
		return false;	/* no holder, or @head is a non-head duplicate */
	*holder_flag_p = holder_flag;
	if (ft_node_compressed(holder_flag) ||
	    ft_node_skip_compressed(holder_flag)) {
		struct cds_ft_compressed_node *cn =
			ft_node_skip_compressed(holder_flag) ?
				ft_skip_to_compressed(ft, holder_flag) :
				ft_compressed_node_ptr(holder_flag);

		*is_prefix_p = false;
		*head_slot_p = &cn->child;
	} else if (ft_node_external_nodes(holder_flag) ==
			(struct cds_ft_node *) head) {
		*is_prefix_p = true;
		*head_slot_p = NULL;	/* external_nodes, not a body slot */
		return true;
	} else {
		struct cds_ft_inode_flag *child;

		*is_prefix_p = false;
		child = ft_node_get_nth_skip(holder_flag, head_slot_p,
			iter_key[key_len - 1], FT_PF_NONE);
		if (!child)
			return false;
	}
	return (struct cds_ft_node *) ft_node_ptr(**head_slot_p) == head;
}

enum cds_ft_status cds_ft_remove_all(struct cds_ft *ft,
		struct cds_ft_iter *iter,
		struct cds_ft_node **result_node)
{
	struct cds_ft_node *chain_head;
	struct cds_ft_inode_flag *holder_flag;
	struct cds_ft_metadata *holder_meta;
	struct cds_ft_inode_flag **head_slot;
	bool is_prefix;
	int ret;
	const uint8_t *iter_key;
	size_t key_len = ft_key_len(ft, ft_iter_resolve_key_len(iter));

	CDS_FT_SCOPED_WRITER(ft);
	/*
	 * If the iterator has a valid path, the RCU read-side lock must
	 * be held.
	 */
	if (iter->cache_valid)
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
		ft_nr_keys_store(metadata, ft_nr_keys_get(metadata) - 1,
			CMM_RELEASE);
		rcu_assign_pointer(metadata->external_nodes, NULL);
		/* Ordered list on: the head's cell leaves the trie (capture before
		 * mark_removed, though that only tombstones ->next).  List off: none. */
		if (ft->ordered_list) {
			struct ft_ord_cell *dead = ft_ord_cell_ptr(external_nodes->prev);

			ft_ord_cell_unsplice(ft, dead);
			ft_ord_cell_free(ft, dead);
		}
		/* The whole chain has left the trie: tombstone every node. */
		ft_chain_mark_removed(external_nodes);
		/* The mutation invalidates the cached position (general-path parity). */
		iter->cache_valid = false;
		iter_debug_path_clear(iter);
		iter->path_len = 0;
		return CDS_FT_STATUS_OK;
	}

	iter_key = ft_iter_read_key(iter);
	dbg_printf("cds_ft_remove_all attempt\n");

	/*
	 * No top-down descent: anchor on the cached chain head (iter->node)
	 * under the writer mutex.  If it is still the live head at iter->key
	 * (ft_locate_chain_head validates *head_slot == iter->node), use it;
	 * otherwise re-seed via a fresh exact lookup (the standard positioning
	 * descent, NOT a remove-specific re-descent) and locate from there.
	 */
	chain_head = NULL;
	if (iter->cache_valid && iter->node && !ft_node_is_removed(iter->node) &&
	    ft_locate_chain_head(ft, iter->node, iter_key, key_len,
		    &holder_flag, &head_slot, &is_prefix))
		chain_head = iter->node;
	if (!chain_head) {
		enum cds_ft_status s = (*ft->lookup_iter_fn)(ft, iter);

		if (s != CDS_FT_STATUS_OK || !iter->node ||
		    !ft_locate_chain_head(ft, iter->node, iter_key, key_len,
			    &holder_flag, &head_slot, &is_prefix)) {
			*result_node = NULL;
			return CDS_FT_STATUS_NOT_FOUND;
		}
		chain_head = iter->node;
	}
	holder_meta = cds_ft_item_to_metadata(ft_node_ptr(holder_flag));
	*result_node = chain_head;

	if (is_prefix) {
		/*
		 * Prefix key: the chain hangs off the internal holder's
		 * external_nodes.  Clear it (one key) and tombstone the chain.
		 * Propagate -1 before clearing (undercount ordering).
		 *
		 * The holder always KEEPS at least one child here.  An is_prefix
		 * holder is an internal node carrying external_nodes, and by
		 * construction such a node has nr_child >= 1: a node that holds only
		 * external keys with no children is never represented as an internal
		 * node -- it is stored as an external-chain pointer in the parent
		 * slot (ft_detach_node promotes a childless holder's external_nodes
		 * to the parent when its last child is removed), and ft_locate_chain
		 * _head would classify it as a LEAF, not a prefix.  So clearing
		 * external_nodes leaves a valid branch with children -- never a
		 * childless (dead-end) internal, which a trie WITHOUT the optional
		 * ordinal cell list would break on (its ordered traversal descends
		 * the structure and a dead-end branch has no entry to find).  The
		 * only follow-up is canonicalizing a now-single-child holder under
		 * skip builds.
		 */
		ft_propagate_external_count_parent(ft, holder_flag, -1);
		rcu_assign_pointer(holder_meta->external_nodes, NULL);
		ft_chain_mark_removed(chain_head);
		ret = 0;
		assert(holder_meta->nr_child > 0);
#ifdef FEATURE_FT_SKIP_COMPRESSED
		if (ft_group_skip_compressed(ft->group) &&
		    holder_meta->nr_child == 1 &&
		    holder_meta->parent != NULL) {
			ft_canonicalize_chain_compress(ft, holder_flag,
				holder_meta, ft_get_parent_slot(holder_meta, ft));
		}
#endif
	} else {
		/*
		 * Leaf key: the whole chain sits at head_slot (a body slot, or
		 * a compressed holder's &cn->child).  Removing it empties the
		 * slot, so prune the branch via ft_detach_node bootstrapped from
		 * the holder (it climbs via metadata->parent).  Propagate -1
		 * before detach (which may free internal nodes).
		 */
		ft_propagate_external_count_parent(ft, holder_flag, -1);
		ret = ft_detach_node(ft, head_slot,
			ft_get_parent_slot(holder_meta, ft), key_len, true);
		if (ret)
			ft_propagate_external_count_parent(ft, holder_flag, 1);
		else
			ft_chain_mark_removed(chain_head);
	}

	/*
	 * detach should not replace a NULL pointer because it has been
	 * found by a mutex-protected traversal within this function.
	 */
	assert(ret != -ENOENT);

	/* Ordered list on: the whole key left the trie: its head's cell is
	 * unspliced and freed (deferred).  chain_head->prev still carries the cell
	 * (detach reshapes ancestors and head_slot, not the head's prev).  List
	 * off: chain_head->prev is the flagged parent, no cell. */
	if (ret == 0 && ft->ordered_list) {
		struct ft_ord_cell *dead = ft_ord_cell_ptr(chain_head->prev);

		ft_ord_cell_unsplice(ft, dead);
		ft_ord_cell_free(ft, dead);
	}

	iter->cache_valid = false;
	iter_debug_path_clear(iter);
	iter->path_len = 0;

	if (ret) {
		/*
		 * Leaf-key detach ENOMEM: nothing was published (the chain is
		 * still live in the trie; the count undo above restored the
		 * ancestors).  Surface the real error with a NULL out-param --
		 * the header contract -- so the caller cannot reclaim the
		 * still-reachable chain.
		 */
		*result_node = NULL;
		return CDS_FT_STATUS_MEMORY_ERROR;
	}

	return CDS_FT_STATUS_OK;
}


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
 * caller's ft_graft_glue_abort with both tries pristine.
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
 * -ENOMEM (caller runs ft_graft_glue_abort).
 */
static
int ft_split_compressed_graft_build(struct cds_ft *ft,
		struct ft_descent *d,
		const uint8_t *key, size_t key_len,
		unsigned int diverge_pos,
		struct cds_ft_inode_flag *payload,
		unsigned long src_count,
		struct ft_graft_glue *glue)
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
		ft_graft_glue_track(glue, old_suffix_flag);
		ft_graft_glue_defer_edge(ft, glue, cn->child, old_suffix_flag,
			&sfx->child);
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
		ft_graft_glue_track(glue, dest);
		ft_node_get_nth_skip(dest, &slot,
			cn->key_bytes[diverge_pos + 1], FT_PF_NONE);
		ft_graft_glue_defer_edge(ft, glue, cn->child, dest, slot);
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
	ft_graft_glue_track(glue, branch_flag);
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
		ft_graft_glue_untrack(ft, glue, old_branch);
		free_cds_ft_node_unpublished(ft, old_branch);
		ft_graft_glue_track(glue, branch_flag);
	}
	ft_nr_keys_store(cds_ft_item_to_metadata(ft_node_ptr(branch_flag)),
		old_child_nr_keys, CMM_RELAXED);

	/* Wire the OLD direction (re-encode compressed slot to skip form). */
	ft_node_get_nth_skip(branch_flag, &slot, old_ordinal, FT_PF_NONE);
	if (sfx_skip_flag && sfx_skip_flag != old_suffix_flag && slot)
		rcu_assign_pointer(*slot, sfx_skip_flag);
	ft_graft_glue_defer_edge(ft, glue, old_suffix_flag, branch_flag, slot);
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
	ft_graft_glue_defer_edge(ft, glue, new_dir, branch_flag, slot);

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
		ft_graft_glue_track(glue, top_flag);
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
			ft_graft_glue_track(glue, top_flag);
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
		ft_graft_glue_track(glue, dest);
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
	ft_graft_glue_set_publish(ft, glue, d->pnf, d->nfp, top_flag);
	ft_graft_glue_defer_free(glue, cn, true);
	glue->attached_nf = new_dir;
	return 0;
}


/*
 * Graft-transaction glue helpers.  The struct and the rationale are
 * defined up near ft_build_branch (the type is referenced by the
 * build-invisible builders that precede this point).
 */
static
void ft_graft_glue_init(struct ft_graft_glue *g)
{
	g->deferred = g->deferred_floor;
	g->nr_deferred = 0;
	g->cap_deferred = FT_GRAFT_GLUE_FLOOR_DEFERRED;
	g->free_list = g->free_floor;
	g->nr_free = 0;
	g->cap_free = FT_GRAFT_GLUE_FLOOR_FREE;
	g->built = g->built_floor;
	g->nr_built = 0;
	g->cap_built = FT_GRAFT_GLUE_FLOOR_BUILT;
	g->splices = g->splices_floor;
	g->nr_splices = 0;
	g->cap_splices = FT_GRAFT_GLUE_FLOOR_SPLICE;
	g->publish_parent = NULL;
	g->publish_slot = NULL;
	g->top = NULL;
	g->attached_nf = NULL;
}

/*
 * Release a glue's malloc'd backing (when it grew past the inline floor) and
 * reset the arrays to the floor, so a second call is a no-op.  Both the abort
 * path (via ft_graft_glue_abort) and the success path call this; graft /
 * graft_swap stay on the floor, so it does nothing for them.
 */
static
void ft_graft_glue_fini(struct ft_graft_glue *g)
{
	if (g->deferred != g->deferred_floor) {
		free(g->deferred);
		g->deferred = g->deferred_floor;
		g->cap_deferred = FT_GRAFT_GLUE_FLOOR_DEFERRED;
	}
	if (g->free_list != g->free_floor) {
		free(g->free_list);
		g->free_list = g->free_floor;
		g->cap_free = FT_GRAFT_GLUE_FLOOR_FREE;
	}
	if (g->built != g->built_floor) {
		free(g->built);
		g->built = g->built_floor;
		g->cap_built = FT_GRAFT_GLUE_FLOOR_BUILT;
	}
	if (g->splices != g->splices_floor) {
		free(g->splices);
		g->splices = g->splices_floor;
		g->cap_splices = FT_GRAFT_GLUE_FLOOR_SPLICE;
	}
}

/*
 * Grow the three arrays onto a malloc'd backing so the build can hold a
 * cluster larger than the inline floor (cds_ft_merge_at's tree-shaped spine).
 * Sizes come from a read-only counting pre-pass; call once, right after init,
 * before any track / defer.  A request at or below a floor leaves that array
 * inline.  Returns 0, or -ENOMEM (whatever already grew is released by a
 * later abort / fini, so the caller need only surface the error).
 */
#ifdef FEATURE_FT_MERGE
static
int ft_graft_glue_reserve(struct ft_graft_glue *g,
		int nr_built, int nr_deferred, int nr_free, int nr_splices)
{
	if (nr_built > g->cap_built) {
		struct cds_ft_inode_flag **p =
			malloc((size_t) nr_built * sizeof(*p));

		if (!p)
			return -ENOMEM;
		memcpy(p, g->built, (size_t) g->nr_built * sizeof(*p));
		if (g->built != g->built_floor)
			free(g->built);
		g->built = p;
		g->cap_built = nr_built;
	}
	if (nr_deferred > g->cap_deferred) {
		struct ft_graft_deferred_edge *p =
			malloc((size_t) nr_deferred * sizeof(*p));

		if (!p)
			return -ENOMEM;
		memcpy(p, g->deferred, (size_t) g->nr_deferred * sizeof(*p));
		if (g->deferred != g->deferred_floor)
			free(g->deferred);
		g->deferred = p;
		g->cap_deferred = nr_deferred;
	}
	if (nr_free > g->cap_free) {
		struct ft_graft_free_item *p =
			malloc((size_t) nr_free * sizeof(*p));

		if (!p)
			return -ENOMEM;
		memcpy(p, g->free_list, (size_t) g->nr_free * sizeof(*p));
		if (g->free_list != g->free_floor)
			free(g->free_list);
		g->free_list = p;
		g->cap_free = nr_free;
	}
	if (nr_splices > g->cap_splices) {
		struct ft_graft_splice *p =
			malloc((size_t) nr_splices * sizeof(*p));

		if (!p)
			return -ENOMEM;
		memcpy(p, g->splices, (size_t) g->nr_splices * sizeof(*p));
		if (g->splices != g->splices_floor)
			free(g->splices);
		g->splices = p;
		g->cap_splices = nr_splices;
	}
	return 0;
}
#endif /* FEATURE_FT_MERGE */

/*
 * Record the single forward publish that splices the built cluster into
 * dst, and wire the cluster top's back-pointer into its (live)
 * publish_parent.  The builders call this once the cluster is fully
 * built.  top is fresh and unpublished, so ft_graft_glue_defer_edge
 * stores top->parent IMMEDIATELY (its fresh-child fast path); the store
 * lands before any other commit-time mutation, so by the time
 * apply_deferred flips any live back-pointer into the cluster, the
 * up-walk path from cluster nodes through top into publish_parent is
 * already wired.
 */
static
void ft_graft_glue_set_publish(struct cds_ft *ft, struct ft_graft_glue *g,
		struct cds_ft_inode_flag *parent_nf,
		struct cds_ft_inode_flag **parent_slot,
		struct cds_ft_inode_flag *top)
{
	g->publish_parent = parent_nf;
	g->publish_slot = parent_slot;
	g->top = top;
	ft_graft_glue_defer_edge(ft, g, top, parent_nf, parent_slot);
}

/* Record a fresh glue node so the abort path can free it. */
static
void ft_graft_glue_track(struct ft_graft_glue *g,
		struct cds_ft_inode_flag *nf)
{
	assert(g->nr_built < g->cap_built);
	/*
	 * PLAIN forms only: a SKIP pointer encodes the CHILD's address, so
	 * the abort path's kind dispatch would free the wrong node (the 2.1
	 * corruption shape) and the identity helpers would have to chase the
	 * child's back-pointer mid-build.  Callers track the plain compressed
	 * flag and re-encode separately for the slot publish.
	 */
	assert(!ft_node_skip_compressed(nf));
	g->built[g->nr_built++] = nf;
}

/*
 * Drop a tracked glue node that has been consumed (chain-merge absorbs a
 * freshly-built compressed wrapper and frees it during the build).  Match
 * by underlying node identity so a plain-flag tracking entry is found
 * even when the absorbed reference is skip-encoded.  No-op if absent.
 */
static
void ft_graft_glue_untrack(struct cds_ft *ft, struct ft_graft_glue *g, void *node_ptr)
{
	int i;

	for (i = 0; i < g->nr_built; i++) {
		struct cds_ft_inode_flag *nf = g->built[i];
		void *p;

		if (ft_node_compressed(nf))
			p = ft_compressed_node_ptr(nf);
		else if (ft_node_skip_compressed(nf))
			p = ft_skip_to_compressed(ft, nf);
		else
			p = ft_node_ptr(nf);
		if (p == node_ptr) {
			g->built[i] = g->built[--g->nr_built];
			return;
		}
	}
}

/*
 * Test whether @child names a node currently in the glue's tracked
 * (fresh, unpublished) set.  Match by underlying node identity so a
 * plain-flag tracking entry is found even when @child is the skip-
 * encoded form (or vice versa).
 */
static
bool ft_graft_glue_is_fresh(struct cds_ft *ft, struct ft_graft_glue *g,
		struct cds_ft_inode_flag *child)
{
	void *cp;
	int i;

	if (!child)
		return false;
	if (ft_node_compressed(child))
		cp = ft_compressed_node_ptr(child);
	else if (ft_node_skip_compressed(child))
		cp = ft_skip_to_compressed(ft, child);
	else if (ft_node_external(child))
		return false;	/* externals are never glue-tracked */
	else
		cp = ft_node_ptr(child);
	for (i = 0; i < g->nr_built; i++) {
		struct cds_ft_inode_flag *nf = g->built[i];
		void *bp;

		if (ft_node_compressed(nf))
			bp = ft_compressed_node_ptr(nf);
		else if (ft_node_skip_compressed(nf))
			bp = ft_skip_to_compressed(ft, nf);
		else
			bp = ft_node_ptr(nf);
		if (bp == cp)
			return true;
	}
	return false;
}

/*
 * Wire a back-pointer (@child->parent = @parent, slot @slot) within a
 * build-invisible graft cluster.
 *
 * Two regimes by @child kind, the rule that the user pinned down:
 *
 *   - Internal-to-glue (@child is a freshly-allocated, glue-tracked node):
 *     store IMMEDIATELY.  The child is unpublished, so the rcu_assign_pointer
 *     is invisible to readers; doing it now (rather than deferring) means
 *     the entire intra-cluster back-pointer chain is fully wired by the
 *     time apply_deferred flips any LIVE-data back-pointer into the
 *     cluster.  An up-walk from re-parented live data then sees a coherent
 *     chain from the live edge through the cluster up into publish_parent
 *     -- no transient NULL parents along the way.
 *
 *   - External-to-glue (@child is a LIVE node being re-parented INTO the
 *     glue cluster, e.g. the displaced old child of a diverge split or
 *     the source-payload leaf at the bottom of the new branch): record
 *     the (child, parent, slot) tuple and defer the ft_set_parent until
 *     ft_graft_glue_apply_deferred at commit time, after the source has
 *     been drained.  Flipping a live back-pointer during the build would
 *     be a publication-visible mutation before the cluster is observable.
 *
 *   Deferred entries are de-duplicated by @child so a canonicalization
 *   wrapper later absorbed by a chain-merge keeps only its final mapping.
 */
static
void ft_graft_glue_defer_edge_origin(struct cds_ft *ft, struct ft_graft_glue *g,
		struct cds_ft_inode_flag *child,
		struct cds_ft_inode_flag *parent,
		struct cds_ft_inode_flag **slot,
		bool dst_origin)
{
	int i;

	if (ft_graft_glue_is_fresh(ft, g, child)) {
		ft_set_parent(ft, child, parent, slot);
		return;
	}
	for (i = 0; i < g->nr_deferred; i++) {
		if (g->deferred[i].child == child) {
			g->deferred[i].parent = parent;
			g->deferred[i].slot = slot;
			g->deferred[i].dst_origin = dst_origin;
			return;
		}
	}
	assert(g->nr_deferred < g->cap_deferred);
	g->deferred[g->nr_deferred].child = child;
	g->deferred[g->nr_deferred].parent = parent;
	g->deferred[g->nr_deferred].slot = slot;
	g->deferred[g->nr_deferred].dst_origin = dst_origin;
	g->nr_deferred++;
}

static
void ft_graft_glue_defer_edge(struct cds_ft *ft, struct ft_graft_glue *g,
		struct cds_ft_inode_flag *child,
		struct cds_ft_inode_flag *parent,
		struct cds_ft_inode_flag **slot)
{
	ft_graft_glue_defer_edge_origin(ft, g, child, parent, slot,
		/*dst_origin*/ false);
}

/* Record an old (replaced) live node to reclaim deferred at commit. */
static
void ft_graft_glue_defer_free(struct ft_graft_glue *g,
		void *node, bool compressed)
{
	assert(g->nr_free < g->cap_free);
	g->free_list[g->nr_free].node = node;
	g->free_list[g->nr_free].compressed = compressed;
	g->nr_free++;
}

/*
 * Abort the build: free every freshly-built (never-observed) glue node, then
 * release the malloc'd backing.  Both tries are left pristine -- no deferred
 * edge was applied, so no live back-pointer references the glue.
 */
static
void ft_graft_glue_abort(struct cds_ft *ft, struct ft_graft_glue *g)
{
	int i;

	for (i = 0; i < g->nr_built; i++) {
		struct cds_ft_inode_flag *nf = g->built[i];

		if (ft_node_compressed(nf))
			free_compressed_node_unpublished(ft,
				ft_compressed_node_ptr(nf));
		else if (ft_node_skip_compressed(nf))
			free_compressed_node_unpublished(ft,
				ft_skip_to_compressed(ft, nf));
		else
			free_cds_ft_node_unpublished(ft, ft_node_ptr(nf));
	}
	ft_graft_glue_fini(g);
}

/*
 * Commit step 1: wire the deferred LIVE back-pointers.  Call after the
 * source unlink + grace period, before the forward publish of the
 * cluster top, so an up-walk from any re-parented live node enters the
 * new cluster before the old nodes are forward-detached and freed.
 *
 * Only edges whose CHILD is a live (observable) node are deferred --
 * setting a live node's parent is a publication-visible mutation that
 * must wait for sync_rcu.  Fresh-to-fresh edges inside the cluster are
 * set IMMEDIATELY during the build (the child is unpublished, the store
 * has no reader-visible effect, and by commit time the entire internal
 * chain from any live re-parent target up to publish_parent is already
 * in place).  Iteration order here therefore does not matter: each live
 * back-pointer flip lands on an already-fully-wired cluster.
 */
static
void ft_graft_glue_apply_deferred(struct cds_ft *ft, struct ft_graft_glue *g)
{
	int i;

	/*
	 * Apply only src-origin edges (dst_origin == false).  graft and
	 * graft_swap record every edge as src-origin (the default), so this
	 * wires all of theirs.  cds_ft_merge_at additionally records
	 * dst-origin edges, which it does NOT re-parent here: a dst child
	 * stays reachable via the old dst spine until the forward publish, so
	 * its back-pointer is switched atomically (with the merge-point
	 * forward slot) by the flip-latch, after this call.
	 */
	for (i = 0; i < g->nr_deferred; i++)
		if (!g->deferred[i].dst_origin)
			ft_set_parent(ft, g->deferred[i].child, g->deferred[i].parent,
				g->deferred[i].slot);
}

/*
 * Record a deferred duplicate-chain splice (cds_ft_merge_at, same full key in
 * both tries): the @src_head chain is to be appended to @dst_head's chain.
 * The @dst_head chain's forward owner and back-pointer are wired separately
 * (Phase-1 set + deferred edge), like any other re-parented external; this
 * records only the concatenation, applied at commit.
 */
#ifdef FEATURE_FT_MERGE
static
void ft_graft_glue_record_splice(struct ft_graft_glue *g,
		struct cds_ft_node *dst_head,
		struct cds_ft_node *src_head)
{
	assert(g->nr_splices < g->cap_splices);
	g->splices[g->nr_splices].dst_head = dst_head;
	g->splices[g->nr_splices].src_head = src_head;
	g->splices[g->nr_splices].src_cell = NULL;
	g->nr_splices++;
}

/*
 * Commit: apply the deferred duplicate-chain splices.  Call AFTER the source
 * has been detached + drained (so the appended @src_head chain has no second
 * owner traversing it from the source tree).
 *
 * Per splice: append the whole src chain to dst's tail, prev-before-next (the
 * ft_chain_node idiom, but preserving src_head->next so the rest of the src
 * chain rides along).  @dst_head stays the head, so in-flight dst snapshots
 * keep their ordering.
 */
static
void ft_graft_glue_apply_splices(struct cds_ft *ft __attribute__((unused)),
		struct ft_graft_glue *g)
{
	int i;

	for (i = 0; i < g->nr_splices; i++) {
		struct cds_ft_node *dst_head = g->splices[i].dst_head;
		struct cds_ft_node *src_head = g->splices[i].src_head;
		struct cds_ft_node *tail = dst_head;
		/*
		 * Ordered list on: @src_head was a head in src (prev is its cell);
		 * it becomes a non-head duplicate of @dst_head, so its cell leaves
		 * the trie.  The cell is NOT unreachable yet: on the merge spine
		 * path this runs after the structural flip, and the surviving src
		 * heads' cells -- already reachable in dst -- still carry the old
		 * src-run ord_prev/ord_next, including links to THIS cell, until
		 * the post-publish interleave rewires them.  Capture it in the
		 * splice record; ft_graft_glue_free_collided_cells frees it after
		 * the interleave through the grace-period-deferred cell free.
		 * List off: src_head->prev is the flagged parent, no cell.
		 */
		g->splices[i].src_cell = ft->ordered_list ?
			ft_ord_cell_ptr(src_head->prev) : NULL;

		while (ft_node_next(tail))
			tail = ft_node_next(tail);
		src_head->prev = tail;	/* write-side only, plain store */
		rcu_assign_pointer(tail->next, src_head);
	}
}

/*
 * Free the collided (demoted) src heads' cells.  Call AFTER the ordered-list
 * interleave: only then has every surviving cell's stale src-run link been
 * rewired away from these cells, making them unreachable to NEW readers; the
 * grace-period defer inside ft_ord_cell_free then covers readers already
 * holding a stale link or parked on a demoted head.
 */
static
void ft_graft_glue_free_collided_cells(struct cds_ft *ft,
		struct ft_graft_glue *g)
{
	int i;

	for (i = 0; i < g->nr_splices; i++)
		if (g->splices[i].src_cell)
			ft_ord_cell_free(ft, g->splices[i].src_cell);
}
#endif /* FEATURE_FT_MERGE */

/*
 * Commit step 2: the single forward publish that makes the whole cluster
 * reachable in dst.  Call after ft_graft_glue_apply_deferred.  The
 * cluster top's parent is wired into publish_parent at set_publish time
 * (build phase, fresh-child store) and the rest of the cluster's
 * internal back-pointers are also already set, so by the time we publish
 * every back-pointer needed for an up-walk from any re-parented live
 * node up through the cluster to publish_parent is in place.
 */
static
void ft_graft_glue_publish(struct cds_ft *ft, struct ft_graft_glue *g)
{
	ft_publish_to_parent(ft, g->publish_parent, g->publish_slot, g->top);
}

/*
 * Commit step 3: reclaim the old (replaced) live nodes, deferred via the
 * normal grace-period free.  Call after the forward publish.
 */
static
void ft_graft_glue_free_old(struct cds_ft *ft, struct ft_graft_glue *g)
{
	int i;

	for (i = 0; i < g->nr_free; i++) {
		if (g->free_list[i].compressed)
			free_compressed_node(ft, g->free_list[i].node);
		else
			free_cds_ft_node(ft, g->free_list[i].node);
	}
}

/*
 * Build a branch for key[start .. end-1] with @leaf at the bottom.
 * When the path is 2+ bytes, a single compressed node is used instead
 * of a chain of single-child internal nodes.  Returns the topmost
 * flagged node, or NULL on allocation failure.
 *
 * @glue: when non-NULL (graft build-invisible mode), @leaf is LIVE
 * payload data: its back-pointer into the bottom branch node is deferred
 * to the post-sync commit, every fresh branch node is tracked in @glue,
 * and on OOM the function returns NULL WITHOUT freeing -- the caller's
 * ft_graft_glue_abort reclaims the tracked nodes.  When NULL, the legacy
 * immediate path runs (leaf back-pointer set now, self-free on OOM).
 */
static
struct cds_ft_inode_flag *ft_build_branch(struct cds_ft *ft,
		const uint8_t *key, unsigned int start, unsigned int end,
		struct cds_ft_inode_flag *leaf,
		unsigned long subtree_external_count,
		bool has_external_nodes,
		struct ft_graft_glue *glue)
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
	 * 1-child internal node.  In glue mode ft_try_compress_chain
	 * defers the live leaf's back-pointer (and absorbs a compressed
	 * canonicalization-wrapper @leaf into this compressed).
	 */
	if (end >= compress_start + 1) {
		struct cds_ft_inode_flag *compressed;

		compressed = ft_try_compress_chain(ft, key, end,
			compress_start, leaf, NULL, glue);
		if (compressed == (void *) (long) -ENOMEM)
			return NULL;
		if (compressed) {
			struct cds_ft_metadata *m =
				ft_flag_to_metadata(ft, compressed);

			ft_nr_keys_store(m,
				subtree_external_count, CMM_RELAXED);
			/*
			 * ft_try_compress_chain already tracked the compressed
			 * node by its PLAIN flag in glue mode (the @compressed
			 * return here is the skip form, unsafe to track).
			 */
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
		/*
		 * Bottom internal whose child is the live @leaf (no
		 * compression happened): defer @leaf's back-pointer
		 * (cluster_leaf) and record it for commit.  Higher internals
		 * and the compressed-wrapping case have only fresh children,
		 * whose back-pointers are safe to set during the build.
		 */
		bool leaf_edge = (glue != NULL) && (cur == leaf);
		int ret;

		ret = ft_node_set_nth(ft, &dest, key[i], cur,
			NULL, NULL, i, leaf_edge);
		if (ret) {
			if (glue)
				return NULL;	/* abort frees tracked nodes */
			/*
			 * Legacy: free the created internal chain and, if
			 * present, the compressed chunk at the bottom.
			 */
			while (cur != leaf) {
				if (ft_node_compressed(cur)) {
					free_compressed_node(ft,
						ft_compressed_node_ptr(cur));
					cur = leaf;
				} else {
					struct cds_ft_inode_flag *next;
					uint8_t kv = key[i + 1];

					next = ft_node_get_nth(ft, cur, NULL, kv, FT_PF_NONE);
					free_cds_ft_node(ft, ft_node_ptr(cur));
					cur = next;
					i++;
				}
			}
			return NULL;
		}
		ft_nr_keys_store(
			cds_ft_item_to_metadata(ft_node_ptr(dest)),
			subtree_external_count, CMM_RELAXED);
		if (glue) {
			ft_graft_glue_track(glue, dest);
			if (leaf_edge) {
				struct cds_ft_inode_flag **slot = NULL;

				ft_node_get_nth_skip(dest, &slot, key[i],
					FT_PF_NONE);
				ft_graft_glue_defer_edge(ft, glue, leaf, dest, slot);
			}
		}
		/*
		 * Initialize density: this node's child (cur) may be
		 * the graft payload with an existing subtree.
		 * Bottom-up order ensures child density is set before
		 * parent.
		 */
		cur = dest;
	}
	return cur;
}

/*
 * Free a FRESH, never-published single-path branch built by ft_build_branch
 * (legacy glue == NULL mode) after a LATER fallible step failed: walk the
 * single-child chain from @top down to -- but not including -- @leaf (the
 * caller's payload), freeing every fresh node.  Writer-private memory, so
 * immediate frees are safe.  A skip-encoded link resolves through the leaf's
 * back-pointer, which the build wired before returning.
 */
static
void ft_free_branch_unpublished(struct cds_ft *ft,
		struct cds_ft_inode_flag *top, struct cds_ft_inode_flag *leaf)
{
	while (top && top != leaf) {
		struct cds_ft_inode_flag *next;

		if (ft_node_skip_compressed(top)) {
			struct cds_ft_compressed_node *cn =
				ft_skip_to_compressed(ft, top);

			next = cn->child;
			free_compressed_node_unpublished(ft, cn);
		} else if (ft_node_compressed(top)) {
			struct cds_ft_compressed_node *cn =
				ft_compressed_node_ptr(top);

			next = cn->child;
			free_compressed_node_unpublished(ft, cn);
		} else if (ft_node_external(top)) {
			/* Only @leaf may be external on a fresh branch. */
			assert(top == leaf);
			break;
		} else {
			uint8_t v;

			next = ft_node_get_direction(ft, top, -1, &v,
				FT_RIGHT, false);
			free_cds_ft_node_unpublished(ft, ft_node_ptr(top));
		}
		top = next;
	}
}


/*
 * ft_compress_single_child_if_needed: convert a 1-child internal node
 * (no external_nodes) to a compressed/skip-encoded node when the trie
 * group has skip-compressed enabled.  Used at graft sites where a
 * moved root (which was an internal at trie-root position) is being
 * placed at a non-root position: under skip mode, non-root 1-child
 * internals without external_nodes must be a compressed (chain-
 * compress invariant).
 *
 * Two cases on the single child:
 *  - non-compressed:    build a 1-byte cn(byte) -> single_child.
 *  - compressed (or
 *    skip-encoded):     chain-merge -- build a cn(byte ++
 *                       single_cn.key_bytes) -> single_cn.child,
 *                       free the absorbed cn.  Bounded by
 *                       FT_SKIP_LEN_MAX; on overflow leave @child
 *                       unchanged.
 *
 * Returns the converted compressed/skip-encoded flag on success.
 * Returns @child unchanged when conversion isn't applicable (and the
 * unchanged @child is itself canonical at the target position):
 *   - skip-compressed mode is disabled,
 *   - @child is not an internal node,
 *   - @child has more than one child,
 *   - @child has external_nodes attached,
 *   - chain-merge length would overflow FT_SKIP_LEN_MAX.
 * Returns (void *)(long)-ENOMEM on allocation failure: conversion WAS
 * required (a non-root 1-child internal) but could not be done, so the
 * caller must NOT publish @child non-canonically -- it must fail the
 * mutation.  @child is left untouched in that case (nothing freed), so
 * the caller can still roll it back.
 *
 * On successful conversion the old internal is freed.  Write-side
 * only (mutex held); the caller is the sole owner of @child.
 *
 * @glue: when non-NULL, build-invisible mode for the graft transaction.
 * The wrapper @cn is built but its re-parent of the live @cn->child, and
 * the frees of the old internal and any absorbed sub-cn, are DEFERRED to
 * the post-sync commit (recorded in @glue) rather than performed here.
 * The wrapper is tracked in @glue so an OOM elsewhere in the build frees
 * it.  When NULL, the legacy immediate path runs (caller has already
 * drained the source).
 */
static
struct cds_ft_inode_flag *ft_compress_single_child_if_needed(struct cds_ft *ft,
		struct cds_ft_inode_flag *child,
		struct ft_graft_glue *glue)
{
#ifdef FEATURE_FT_SKIP_COMPRESSED
	struct cds_ft_inode *node;
	unsigned int type_index;
	const struct cds_ft_type *type;
	struct cds_ft_metadata *meta;
	uint8_t byte;
	struct cds_ft_inode_flag *single_child;
	struct cds_ft_compressed_node *single_cn = NULL;
	unsigned int single_len = 0;
	struct cds_ft_compressed_node *cn;
	struct cds_ft_metadata *cn_meta;
	struct cds_ft_inode_flag *cflag;
	unsigned int cn_len;

	if (!ft_group_skip_compressed(ft->group))
		return child;
	if (!ft_node_internal(child))
		return child;
	node = ft_node_ptr(child);
	type_index = ft_node_type(child);
	type = &ft_types[type_index];
	meta = cds_ft_item_to_metadata_fast(node, type->order);
	if (meta->nr_child != 1 || meta->external_nodes != NULL)
		return child;

	/*
	 * Find the single set byte.  ft_node_get_minmax returns the
	 * resolved child (skip-encoded -> compressed flag); but we
	 * need the raw slot value to preserve any skip encoding when
	 * placing it as cn->child.  Walk via the low-level get_ith.
	 */
	switch (type->type_class) {
	case FT_POPCOUNT:
		ft_popcount_node_get_ith_pos(type, node, 0, &byte, &single_child);
		break;
	case FT_PIGEON:
	{
		unsigned int i;

		single_child = NULL;
		byte = 0;
		for (i = 0; i < FT_ENTRY_PER_NODE; i++) {
			struct cds_ft_inode_flag *v =
				ft_pigeon_node_get_ith_pos(type, node, i);
			if (v) {
				byte = (uint8_t) i;
				single_child = v;
				break;
			}
		}
		break;
	}
	default:
		return child;
	}
	if (!single_child)
		return child;

	/*
	 * Chain-merge when the single child is itself a compressed
	 * (or skip-encoded) cn: absorb its path bytes so the result
	 * is one compressed spanning [byte ++ single_cn->key_bytes]
	 * -> single_cn->child.  Preserves the "no two adjacent
	 * compresseds" invariant.
	 */
	if (ft_node_skip_compressed(single_child))
		single_cn = ft_skip_to_compressed(ft, single_child);
	else if (ft_node_compressed(single_child))
		single_cn = ft_compressed_node_ptr(single_child);
	if (single_cn) {
		single_len = single_cn->len;
		if (1U + single_len > FT_SKIP_LEN_MAX) {
			/* Overflow: leave un-canonicalized. */
			return child;
		}
	}
	cn_len = 1U + single_len;

	cn = alloc_compressed_node(ft, cn_len, &cn_meta);
	if (!cn)
		return (struct cds_ft_inode_flag *) (long) -ENOMEM;
	cn->len = (uint8_t) cn_len;
	cn->key_bytes[0] = byte;
	if (single_cn) {
		memcpy(&cn->key_bytes[1], single_cn->key_bytes, single_len);
		cn->child = single_cn->child;
	} else {
		cn->child = single_child;
	}
	cn_meta->nr_child = 1;
	ft_nr_keys_store(cn_meta, ft_nr_keys_get(meta), CMM_RELAXED);
	cflag = ft_compressed_node_flag(cn);
	if (glue) {
		/*
		 * Build-invisible path (graft transaction).  @node and
		 * @single_cn are LIVE source data that must survive an OOM
		 * elsewhere in the build, and @cn->child's back-pointer must
		 * not be flipped until the post-sync commit.  Defer all three;
		 * track @cn so the abort path frees it.  The deferred edge
		 * records the PLAIN @cflag -- ft_set_parent recovers @cn
		 * directly and records its skip_slot at commit.
		 */
		ft_graft_glue_track(glue, cflag);
		ft_graft_glue_defer_edge(ft, glue, cn->child, cflag, &cn->child);
		ft_graft_glue_defer_free(glue, node, false);
		if (single_cn)
			ft_graft_glue_defer_free(glue, single_cn, true);
		/*
		 * Return the PLAIN flag, NOT the skip form: @cn->child's
		 * back-pointer is deferred, so a skip pointer would not yet
		 * resolve (ft_skip_to_compressed recovers @cn via that stale
		 * back-pointer).  The caller installs @cn directly (chain-merge
		 * recovers it via ft_compressed_node_ptr) and re-encodes the
		 * holding slot to the skip form before publish -- same dance as
		 * the split's suffix.
		 */
		return cflag;
	}
	ft_set_parent(ft, cn->child, cflag, &cn->child);
	/*
	 * Legacy immediate path: the caller has already unlinked @node
	 * from src and waited a grace period (cds_ft_graft / graft_swap
	 * protocol); @node has not been published to dst.  No reader can
	 * be inside it, so the immediate-free path is safe -- saves a grace
	 * period of deferred-free pressure.  Same applies to single_cn
	 * (the compressed sub-node we just absorbed): it was reachable
	 * only via @node, which is itself unpublished here.
	 */
	free_cds_ft_node_unpublished(ft, node);
	if (single_cn)
		free_compressed_node_unpublished(ft, single_cn);
	return ft_publish_compressed(ft, cn, cflag);
#else
	(void) ft;
	(void) glue;
	return child;
#endif
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
 * the payload and the destination pristine (the caller runs ft_graft_glue_abort).
 * commit applies the deferred edges, swings the forward publish and reclaims the
 * replaced source root (if canonicalized); it cannot fail.
 */
struct ft_graft_store_state {
	struct ft_graft_glue *glue;
	bool displaced_shape;
	struct cds_ft_inode_flag *attached;		/* payload (at-node) or branch */
	unsigned int attached_depth;
	/* flip publish (slot-at-node and built-branch-flip shapes): */
	struct ft_flip_batch *b;
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

static
enum cds_ft_status ft_store_at_graft_point_prepare(struct cds_ft *ft,
		const uint8_t *key, size_t key_len,
		struct ft_descent *d,
		struct cds_ft_inode_flag *graft_payload,
		unsigned long graft_external_count,
		struct ft_graft_glue *glue,
		struct ft_flip_batch **pre_flip,
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
		struct cds_ft_inode_flag *slot_value, *pf;
		struct ft_flip_batch *b;
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
		 * LAST fallible steps, so they run FIRST, parking a flip proxy
		 * that keeps resolving to the empty slot.  An ENOMEM here leaves
		 * both tries untouched (no live edge flipped); the failure-free
		 * wiring runs behind the parked proxy in commit.
		 */
		slot_value = graft_payload;
		if (ft_node_compressed(graft_payload))
			slot_value = ft_publish_compressed(ft,
				ft_compressed_node_ptr(graft_payload),
				graft_payload);
		b = ft_flip_batch_take(ft, 1, pre_flip);
		if (!b)
			return CDS_FT_STATUS_MEMORY_ERROR;
		pf = ft_flip_batch_add(b, NULL, slot_value);
		dest = d->pnf;
		ret = ft_node_set_nth(ft, &dest, key[key_len - 1], pf,
			&st->old_recompacted_node, pmeta, d->depth - 1, false);
		if (ret) {
			/* @b is owned here now (taken or freshly allocated). */
			ft_flip_batch_free_unpublished(b);
			return CDS_FT_STATUS_MEMORY_ERROR;
		}

		st->attached = graft_payload;
		st->attached_depth = (unsigned int) key_len;
		st->b = b;
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
			struct cds_ft_inode_flag *pf;
			struct ft_flip_batch *b;
			int ret;

			pmeta = cds_ft_item_to_metadata(ft_node_ptr(d->pnf));

			/*
			 * Same R8 + fresh-before-live discipline as the
			 * d->depth == key_len arm: park a flip proxy so the
			 * fallible slot store runs FIRST and the wiring
			 * completes invisibly in commit.
			 */
			b = ft_flip_batch_take(ft, 1, pre_flip);
			if (!b)
				return CDS_FT_STATUS_MEMORY_ERROR;
			pf = ft_flip_batch_add(b, NULL, branch);
			ret = ft_node_set_nth(ft, &dest, key[i - 1], pf,
				&st->old_recompacted_node, pmeta,
				d->depth - 1, false);
			if (ret) {
				ft_flip_batch_free_unpublished(b);
				return CDS_FT_STATUS_MEMORY_ERROR;
			}

			st->attached = branch;
			st->attached_depth = d->depth;
			st->b = b;
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
		struct ft_graft_store_state *st)
{
	if (st->displaced_shape) {
		/*
		 * Phase 2: glue deferred FIRST (the payload's live back-pointers
		 * must be wired before any dst-reachable live edge flips into the
		 * fresh cluster), then the back-channel, then the forward publish.
		 */
		ft_graft_glue_apply_deferred(ft, st->glue);
		ft_publish_external_nodes_prev(ft, st->attached, st->displaced);
		ft_publish_to_parent(ft, st->pnf, st->nfp, st->attached);
		if (st->tp_i >= 1)
			FT_TP(tree_edge_set, (const void *) ft,
				(const void *) st->pnf,
				(unsigned int) (st->tp_i - 1),
				(uint8_t) st->tp_key[st->tp_i - 1],
				(const void *) st->attached);
	} else {
		struct cds_ft_inode_flag **slot = NULL;

		ft_node_get_nth_skip(st->dest, &slot, st->slot_byte, FT_PF_NONE);
		assert(slot);
		ft_set_parent(ft, st->attached, st->dest, slot);
		ft_graft_glue_apply_deferred(ft, st->glue);
		ft_publish_to_parent(ft, st->publish_pmeta->parent, st->pnfp,
			st->dest);
		ft_flip_batch_commit(st->b);
		rcu_assign_pointer(*slot, st->slot_value);
		ft_flip_batch_reclaim(st->b);

		if (st->old_recompacted_node)
			free_cds_ft_node(ft, st->old_recompacted_node);
	}
	ft_graft_glue_free_old(ft, st->glue);
	*attached_nf = st->attached;
	*attached_depth = st->attached_depth;
}

/*
 * Combined NOSPLIT store: prepare + commit back-to-back -- cds_ft_graft's call
 * site, run after the source-root unlink + drain.  @pre_flip, when non-NULL, is
 * a caller-pre-allocated 1-entry flip batch the prepare uses instead of
 * allocating its own: a sub-position merge pre-allocates it BEFORE its source
 * unlink so this post-drain store has no fallible allocation left (the node
 * allocations draw from the reserve, the flip batch is pre-secured).
 */
static
enum cds_ft_status ft_store_at_graft_point(struct cds_ft *ft,
		const uint8_t *key, size_t key_len,
		struct ft_descent *d,
		struct cds_ft_inode_flag *graft_payload,
		unsigned long graft_external_count,
		struct cds_ft_inode_flag **attached_nf,
		unsigned int *attached_depth,
		struct ft_graft_glue *glue,
		struct ft_flip_batch **pre_flip)
{
	struct ft_graft_store_state st;
	enum cds_ft_status status;

	status = ft_store_at_graft_point_prepare(ft, key, key_len, d,
			graft_payload, graft_external_count, glue, pre_flip, &st);
	if (status != CDS_FT_STATUS_OK)
		return status;
	ft_store_at_graft_point_commit(ft, attached_nf, attached_depth, &st);
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
		struct ft_descent *d, struct ft_graft_glue *glue)
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
		struct ft_flip_batch **pre_flip)
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
		rcu_assign_pointer(dst_ft->root, src_ft->root);
		FT_TP(root_publish, (const void *) dst_ft,
			(const void *) dst_ft->root);
		rcu_assign_pointer(src_ft->root, ft_node_flag(fresh_root, 0));
		FT_TP(root_publish, (const void *) src_ft,
			(const void *) src_ft->root);
		free_cds_ft_node(dst_ft, old_dst_root);
		/*
		 * Ordered list: dst was empty (checked above), so src's WHOLE
		 * ordered list becomes dst's.  Cells' internal links unchanged;
		 * only the head/tail endpoints transfer (mirrors the root swap,
		 * which likewise needs no synchronize_rcu).
		 */
		if (dst_ft->group->ordered_list_set) {
			rcu_assign_pointer(dst_ft->ord_cell_head,
				src_ft->ord_cell_head);
			rcu_assign_pointer(dst_ft->ord_cell_tail,
				src_ft->ord_cell_tail);
			src_ft->ord_cell_head = NULL;
			src_ft->ord_cell_tail = NULL;
		}
		goto done;
	}

	{
		struct ft_descent d;
		struct cds_ft_inode *fresh_node;
		struct cds_ft_metadata *fresh_meta;
		struct ft_graft_glue glue;
		enum ft_graft_prep prep;
		unsigned long src_count = ft_nr_keys_get(src_rmeta);
		struct cds_ft_inode_flag *old_src_root;
		struct cds_ft_inode_flag *attached_nf = NULL;
		struct ft_ord_cell *graft_run_first = NULL, *graft_run_last = NULL;
		struct ft_ord_cell *graft_pred = NULL, *graft_succ = NULL;
		/*
		 * Self-secured NOSPLIT attach: when no caller reserve is active, this
		 * graft reserves its own commit nodes + flip batch before publishing
		 * the empty source root, so the post-publish store cannot fail and
		 * needs no reader-observable source-root rollback.  Empty (NULL @pre)
		 * under a caller reserve -- the rekey, whose reserve + @pre_flip
		 * already make the store unfailable.
		 */
		struct cds_ft_alloc_reserve graft_reserve;
		struct ft_flip_batch *graft_flip = NULL;
		struct ft_flip_batch **store_pre_flip = pre_flip;
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
		ft_graft_glue_init(&glue);
		prep = ft_graft_build(dst_ft, key, key_len, graft_payload,
				src_count, &d, &glue);
		if (prep == FT_GRAFT_PREP_OOM) {
			ft_graft_glue_abort(dst_ft, &glue);
			free_cds_ft_node(src_ft, fresh_node);
			return CDS_FT_STATUS_MEMORY_ERROR;
		}
		if (prep == FT_GRAFT_PREP_POPULATED) {
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
		 * source-root swap below).  A generous node reserve + a flip batch,
		 * drawn here where failure is clean (nothing published yet), make the
		 * post-swap ft_store_at_graft_point unfailable -- so the old rollback
		 * that re-published the source root on a store OOM (a reader-observable
		 * flicker of the source: empty, then full again) is gone.  Skipped when
		 * a caller reserve is already active (the rekey), which secures it via
		 * @pre_flip + that reserve.  The flip is freed below if the store's
		 * shape (a displaced external) did not consume it.
		 */
		if (prep == FT_GRAFT_PREP_NOSPLIT && !dst_ft->active_reserve) {
			memset(&graft_reserve, 0, sizeof(graft_reserve));
			if (ft_bulk_node_reserve_fill(dst_ft, &graft_reserve)) {
				cds_ft_alloc_reserve_drain(dst_ft, &graft_reserve);
				ft_graft_glue_abort(dst_ft, &glue);
				free_cds_ft_node(src_ft, fresh_node);
				return CDS_FT_STATUS_MEMORY_ERROR;
			}
			graft_flip = ft_flip_batch_alloc(dst_ft, 1);
			if (!graft_flip) {
				cds_ft_alloc_reserve_drain(dst_ft, &graft_reserve);
				ft_graft_glue_abort(dst_ft, &glue);
				free_cds_ft_node(src_ft, fresh_node);
				return CDS_FT_STATUS_MEMORY_ERROR;
			}
			store_pre_flip = &graft_flip;
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

		if (!src_ft->exclusive)
			src_ft->group->flavor->update_synchronize_rcu();

		if (prep == FT_GRAFT_PREP_GLUE) {
			/*
			 * Failure-free commit of the build-invisible diverge
			 * cluster: wire the deferred live back-pointers (the
			 * displaced old child, the payload, and the cluster
			 * top), splice the cluster into dst with a single
			 * forward publish, then reclaim the old compressed
			 * node and the source's old root.  Nothing can fail.
			 */
			ft_graft_glue_apply_deferred(dst_ft, &glue);
			ft_graft_glue_publish(dst_ft, &glue);
			attached_nf = glue.attached_nf;
			ft_graft_glue_free_old(dst_ft, &glue);
		} else {
			/*
			 * Non-split attach: the payload subtrie is built into
			 * @glue with its back-pointers deferred, recompacted into
			 * the live graft-point node, and published -- all inside
			 * ft_store_at_graft_point.  Every node draws from the
			 * reserve (self-secured above, or the caller's) and the
			 * proxy parks in the pre-secured flip batch, so the store
			 * has no fallible step left: it cannot fail, and there is
			 * NO source-root rollback (which would have flickered the
			 * source empty-then-full to a reader).
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
							  &glue, store_pre_flip);
			if (self_secured) {
				cds_ft_alloc_reserve_deactivate(dst_ft);
				cds_ft_alloc_reserve_drain(dst_ft, &graft_reserve);
			}
			assert(status == CDS_FT_STATUS_OK);
			(void) status;
			/* Free the flip batch if a displaced-external shape skipped it. */
			if (graft_flip)
				ft_flip_batch_free_unpublished(graft_flip);
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
		 * NIL-key graft: the placed external sits directly under an internal
		 * slot, so refresh its CELL edge byte for the ordered key rebuild
		 * (ft_set_parent does not maintain it for externals; harmless when
		 * the parent is compressed, where the up-walk ignores it).
		 */
		if (nil_key_root && graft_run_first)
			cds_ft_item_to_metadata(graft_run_first)->incoming_byte =
				key[key_len - 1];

		/*
		 * Ordered list: src is now structurally empty + drained; the
		 * payload is published under @key in dst.  Splice the captured run
		 * (src's whole former list) into dst's ordered cell list at the
		 * @key position (an empty range in dst -> no interleave).  Same
		 * commit point as the structural publish above.
		 */
		if (graft_run_first)
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

