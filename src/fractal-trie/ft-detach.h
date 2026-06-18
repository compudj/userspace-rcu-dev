// SPDX-FileCopyrightText: 2012-2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-only

/*
 * src/fractal-trie/ft-detach.h
 *
 * Userspace RCU library - Fractal Trie: detach a sub-trie into a transient trie.
 *
 * Implementation unit: #included once by fractal-trie.c, in dependency
 * order, into a single translation unit (preserves cross-module inlining).
 * Not a standalone header.
 */
#ifndef FRACTAL_TRIE_IMPL
#error "ft-detach.h is an implementation unit; #include it from fractal-trie.c only"
#endif

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

enum cds_ft_status cds_ft_detach(struct cds_ft *ft,
		const uint8_t *_key, size_t _key_len,
		struct cds_ft **result_ft)
{
	size_t key_len;
	enum cds_ft_status status;

	FT_TP_KEY(detach_enter, ft, _key, _key_len);

	*result_ft = NULL;

	if (!ft) {
		FT_TP(detach_exit, (int) CDS_FT_STATUS_INVALID_ARGUMENT_ERROR);
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}

	/*
	 * Root-level detach (key_len == 0) is valid for both
	 * variable-length and fixed-length groups.  See cds_ft_graft.
	 */
	if (_key_len == 0) {
		key_len = 0;
	} else {
		key_len = ft_key_len(ft, _key_len);
		if (!valid_key_len(ft, key_len) ||
				ft->group->key_len != CDS_FT_LEN_VARIABLE) {
			FT_TP(detach_exit, (int) CDS_FT_STATUS_INVALID_ARGUMENT_ERROR);
			return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
		}
	}

	status = ft_detach_keylen(ft, _key, key_len, result_ft);
	FT_TP(detach_exit, (int) status);
	return status;
}

/*
 * One-commit insert tail (see struct ft_insert_commit): the structural slot
 * already holds a parked flip proxy (the fresh head is invisible -- the proxy
 * resolves to the old slot value), and the head's parent chain is fully wired,
 * so the splice-position search runs exactly as the post-publish splice did
 * (the from-head seed walks the parent chain, never the parked slot).  Park
 * the <= 4 ordered-list neighbour edges into the SAME batch, commit once --
 * the head becomes reachable in the structural index AND spliced into the
 * cell list atomically for every reader -- then settle all slots to their
 * direct values and finalize the real top's parent bookkeeping (skip_slot,
 * incoming_byte).
 */
static
void ft_insert_one_commit(struct cds_ft *ft, const uint8_t *key,
		size_t key_len, struct ft_ord_cell *cell,
		struct ft_insert_commit *ic)
{
	struct ft_ord_cell *pred, *succ;
	struct ft_ord_cell_edge edges[4];
	unsigned int i, n = 0;

	pred = ft_ord_cell_find_pred_from_head(ft, key, key_len, cell);
	if (pred)
		succ = ft_ord_cell_resolve_ord(&pred->ord_next);
	else
		/* New minimum: successor is the old list head (O(1), no descent). */
		succ = ft_ord_cell_resolve_ord(&ft->ord_cell_head);
	/* Pre-set @cell's own links; not yet reachable via the list. */
	cell->ord_prev = pred;
	cell->ord_next = succ;
	if (pred) {
		edges[n].slot = &pred->ord_next;
		edges[n].old_target = succ;
		edges[n].new_target = cell;
		n++;
	}
	if (succ) {
		edges[n].slot = &succ->ord_prev;
		edges[n].old_target = pred;
		edges[n].new_target = cell;
		n++;
	}
	/* New min (!pred) => head was @succ; new max (!succ) => tail was @pred. */
	n = ft_ord_cell_endpoint_edge(&ft->ord_cell_head, succ, cell, edges, n);
	n = ft_ord_cell_endpoint_edge(&ft->ord_cell_tail, pred, cell, edges, n);
	/* Park the ordered-list edges (each resolves to OLD until the commit). */
	for (i = 0; i < n; i++)
		rcu_assign_pointer(*edges[i].slot,
			(struct ft_ord_cell *) ft_flip_batch_add(ic->batch,
				(struct cds_ft_inode_flag *) edges[i].old_target,
				(struct cds_ft_inode_flag *) edges[i].new_target));

	/* THE commit: structural slot + ordered-list edges, atomically. */
	urcu_flip_commit(&ic->batch->group);

	/*
	 * Settle: direct values in every parked slot (idempotent for readers,
	 * the proxies already resolve to the new targets).  The real top's
	 * full wiring (parent, slot offset, incoming_byte) was done at park
	 * time, while still invisible.  A split-shape park settles through
	 * ft_publish_to_parent for its dual skip-slot maintenance; the attach
	 * shape's set_nth already did its own bookkeeping, so a direct store
	 * of the same canonical value suffices.
	 */
	if (ic->publish_to_parent)
		ft_publish_to_parent(ft, ic->parent_nf, ic->slot,
			ic->slot_value);
	else
		rcu_assign_pointer(*ic->slot, ic->slot_value);
	for (i = 0; i < n; i++)
		rcu_assign_pointer(*edges[i].slot, edges[i].new_target);
	ft_flip_batch_reclaim(ic->batch);
	ic->batch = NULL;
	/*
	 * The old compressed node a split replaced: readers resolved the
	 * proxy to it until the commit above, so only now may its grace-
	 * period-deferred free be queued.
	 */
	if (ic->free_old_cn)
		free_compressed_node(ft, ic->free_old_cn);
}

