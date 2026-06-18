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

