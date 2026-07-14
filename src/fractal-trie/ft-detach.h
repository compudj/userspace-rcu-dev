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
		if (ft_meta_nr_child(rmeta) == 0 && !rmeta->external_nodes)
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
		 * Pre-reserve the root-swap flip-txn (root + head/tail) BEFORE the
		 * swap, while @ft is still pristine and @detached owns only its own
		 * empty root: an OOM here aborts cleanly (tear down @detached, @ft
		 * untouched).  List off uses the lone-edge ft_root_edge_flip below
		 * (no txn).  This is the whole-trie root detach, not a hot path.
		 */
		struct ft_flip_txn *root_txn = NULL;

		if (ft->group->ordered_list_set) {
			root_txn = ft_flip_txn_create_bounded(
				FT_ROOT_LIST_SWAP_MAX_EDGES);
			if (!root_txn) {
				cds_ft_destroy(detached);
				return CDS_FT_STATUS_MEMORY_ERROR;
			}
		}

		/*
		 * Allocate a fresh empty root for the source trie
		 * before swapping.
		 */
		fresh_node = alloc_cds_ft_node(ft, &ft_types[0], &fresh_meta);
		if (!fresh_node) {
			if (root_txn)
				ft_flip_txn_destroy(root_txn);
			cds_ft_destroy(detached);
			return CDS_FT_STATUS_MEMORY_ERROR;
		}

		/*
		 * Move the source root into the detached trie.
		 * Free the empty root that cds_ft_create allocated for
		 * the detached trie, and replace it with the source root.
		 * @detached is a fresh handle not yet returned to the caller, so
		 * its empty root was never reader-visible: free it UNPUBLISHED
		 * (immediate, no grace period, no freeze-on-free tombstone -- it
		 * was never live in any trie).
		 */
		free_cds_ft_node_unpublished(detached, ft_node_ptr(detached->root));
		/* No readers in detached root yet. */
		detached->root = ft->root;
		FT_TP(root_publish, (const void *) detached,
			(const void *) detached->root);
		/*
		 * This node was already @ft's root, so its parent is already
		 * NULL; only the now-stale parent_slot_offset needs clearing.
		 */
#ifdef FEATURE_FT_SKIP_COMPRESSED
		{
			struct cds_ft_metadata *m = cds_ft_item_to_metadata(
				ft_node_ptr(detached->root));
			ft_meta_parent_slot_offset_set(m, 0);
		}
#endif
		uatomic_store(&detached->max_used_key_len,
			      uatomic_load(&ft->max_used_key_len, CMM_RELAXED),
			      CMM_RELAXED);

		/*
		 * Give source a fresh empty root.  A root detach moves the
		 * WHOLE trie, so when the ordered list is enabled @ft's entire
		 * ordered cell run becomes @detached's: the cells' internal
		 * links are unchanged, and @ft's root swap is fused with its
		 * sentinel clear into ONE flip (ft_root_list_swap_publish, src/
		 * disappear side) so a concurrent reader never observes @ft with
		 * its structure emptied but its ordered list still populated (or
		 * vice versa) -- a reader mid-iteration follows its RCU snapshot
		 * into @detached.  The run's OUTER links (first->prev, last->next)
		 * are NOT relinked in that flip (relink_dest NULL): they stay at
		 * @ft's sentinel, straddler-safe, and re-home to @detached's
		 * sentinel only after the drain (ft_ord_finalize_circular).
		 * @detached has no readers yet, so point its sentinel at the run
		 * with plain stores.
		 */
		if (ft->group->ordered_list_set) {
			struct ft_ord_cell *first = ft_ord_first(ft);
			struct ft_ord_cell *last = ft_ord_last(ft);

			/*
			 * relink_dest NULL: clear @ft's sentinel + swap the root, but do
			 * NOT relink the moved run's outer links in the flip -- they stay
			 * pointing at @ft's (cleared) sentinel, which an @ft reader
			 * straddling the move recognises as its own end (a foreign
			 * @detached sentinel would be dereferenced as a cell).  @detached
			 * has no readers, so point its sentinel at the run by plain store;
			 * the run's outer links are finalized to @detached's sentinel after
			 * the drain below (ft_ord_finalize_circular).
			 */
			ft_root_list_swap_publish(ft, root_txn, &ft->root,
				ft->root, ft_node_flag(fresh_node, 0),
				first, NULL, last, NULL, NULL, false);
			if (first) {
				detached->ord_sentinel.node.next =
					ft_ord_cell_lnode(first);
				detached->ord_sentinel.node.prev =
					ft_ord_cell_lnode(last);
			}
		} else {
			/*
			 * No ordered list: the root pointer is the only
			 * reader-visible slot.  Express it as a single-edge flip
			 * descriptor anyway (commits as one release store, like a
			 * bare rcu_assign_pointer) so the root swap is
			 * MCAS-expressible like every other structural publish.
			 */
			ft_root_edge_flip(ft, &ft->root, ft->root,
				ft_node_flag(fresh_node, 0));
		}
		FT_TP(root_publish, (const void *) ft, (const void *) ft->root);

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

		/*
		 * Drain done: restore @detached's list to circular form (the moved run's
		 * outer links were left at @ft's sentinel for straddler safety; point
		 * them at @detached's sentinel now for its own removes / reverse walks).
		 */
		if (ft->group->ordered_list_set)
			ft_ord_finalize_circular(detached);

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

				ft_descent_traverse_compressed(ft, &d, cn, &ik);
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
			struct ft_glue glue;
			struct cds_ft_inode_flag *new_root = NULL;

			if (!ft_node_external(child)) {
				struct cds_ft_metadata *child_meta =
					cds_ft_item_to_metadata(
						ft_node_ptr(child));
				detached_count = ft_nr_keys_get(child_meta);
			} else {
				detached_count = 1;	/* One key (possibly with duplicates). */
			}

			/*
			 * Non-root detach STRIPS the @key prefix from every
			 * moved leaf, so the detached leaves carry a key that no
			 * longer matches their (now-shallower) position.  Create
			 * the detached trie with speculative keys OFF: its
			 * lookups reconstruct the key from the structure (EAGER)
			 * and never read the stale stored field.  The app may
			 * re-stamp the leaves and create a speculative trie if it
			 * needs speculative lookups on the detached data.  (Root
			 * detach, above, preserves every key and keeps the
			 * group's speculative mode.)
			 */
			struct cds_ft_attr detached_attr = {
				.speculative_keys_disabled = true,
			};

			status = cds_ft_create(ft->group, &detached_attr,
					&detached);
			if (status != CDS_FT_STATUS_OK)
				return status;
			ft_glue_init(&glue);
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
					ft_glue_abort(detached, &glue);
					cds_ft_destroy(detached);
					return CDS_FT_STATUS_MEMORY_ERROR;
				}
			}

			/*
			 * Detach child from the source trie and prune
			 * empty branches above.  After this, child is
			 * no longer reachable from the live trie for
			 * new readers.
			 */
			{
				/*
				 * Subtree-move detach (free_detached_subtree == false): preserve
				 * @child as the root of the new @detached trie.  Bootstrapped from
				 * @child's own slot, ft_detach_node climbs parent pointers to the
				 * surviving ancestor, unlinks the branch there, and its free-walk
				 * phase 1 reclaims the intermediate single-child chain (the @nr_clear
				 * elevated links) while phase 2 -- which would free @child and below
				 * -- is gated off for move-style.
				 *
				 * Ordered list on: the detached subtree's keys form a contiguous run
				 * in @ft's ordered cell list.  Capture that run's endpoints (the
				 * structural min/max heads of the intact @child subtree -- a prefix
				 * key at the detach point, if any, is the run minimum) and hand them,
				 * with a deferred-publish @pub, to ft_detach_node so the structural
				 * unlink and the ordered-cell run-detach commit in ONE flip: a reader
				 * never sees the run gone from the structure but still present in the
				 * ordered list (or vice versa).  @run.armed reports whether a path
				 * fused it; the rare unfused shape (compressed fresh-internal root)
				 * falls back to the standalone two-commit run-detach.  The
				 * internal-child branch's synchronize_rcu below then drains any @ft
				 * reader parked in the run.
				 */
				struct ft_remove_pub pub = { .armed = false };
				struct ft_detach_run run = { .armed = false };
				struct ft_remove_pub *pubp = NULL;
				struct ft_detach_run *runp = NULL;
				struct ft_flip_txn *run_txn = NULL;
				int ret;

				if (ft->group->ordered_list_set) {
					run.into = detached;
					run.rfirst = ft_subtree_minmax_head(ft, child, false);
					run.rlast = ft_subtree_minmax_head(ft, child, true);
					pubp = &pub;
					runp = &run;
					/*
					 * Pre-reserve the standalone run-detach txn before the
					 * structural unlink: the rare unfused shape excises the
					 * run AFTER ft_detach_node is public -- un-abortable.
					 * OOM here aborts cleanly (undo propagation, abort the
					 * build, destroy @detached), leaving @ft pristine.
					 */
					run_txn = ft_flip_txn_create_bounded(
						FT_ORD_CELL_RUN_DETACH_MAX_EDGES);
					if (!run_txn) {
						ft_glue_abort(detached, &glue);
						cds_ft_destroy(detached);
						return CDS_FT_STATUS_MEMORY_ERROR;
					}
				}
				/*
				 * Fold the whole-subtree count removal onto the structural
				 * unlink: @count_delta -detached_count rides ft_detach_node's
				 * own commit, so the surviving ancestor's -detached_count walk
				 * flips ATOMICALLY with the detach (exact under concurrent
				 * writers -- the same magnitude-agnostic machinery as the leaf
				 * -1 fold).  No pre-decrement, no post-detach propagate walk
				 * over the now-freed intermediate chain.
				 */
				ret = ft_detach_node(ft, d.nfp, d.pnfp, d.depth,
						false, NULL, pubp, runp, NULL, NULL,
						-(long) detached_count);
				assert(ret != -ENOENT);
				if (ret < 0) {
					/*
					 * Recompaction -ENOMEM: nothing was published (the
					 * deferred flip never ran, run.armed stays false); the
					 * folded count edges rode the uncommitted txn (an abort
					 * applies nothing), so aborting leaves @ft pristine.
					 */
					if (run_txn)
						ft_flip_txn_destroy(run_txn);
					ft_glue_abort(detached, &glue);
					cds_ft_destroy(detached);
					return CDS_FT_STATUS_MEMORY_ERROR;
				}
				if (ft->group->ordered_list_set) {
					/* Fused into the structural flip (run.armed), or commit
					 * the standalone run-detach through the pre-reserved txn;
					 * release it unused when the structural flip fused it. */
					if (!run.armed)
						ft_ord_cell_run_detach(ft, run_txn, detached,
							run.rfirst, run.rlast);
					else
						ft_flip_txn_destroy(run_txn);
				}
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
				ft_glue_apply_deferred(detached, &glue);
				/*
				 * @detached is a fresh handle with no readers
				 * yet; its placeholder empty root was never
				 * reader-visible, so free it UNPUBLISHED
				 * (immediate, no tombstone -- never live).
				 */
				free_cds_ft_node_unpublished(detached,
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
					m->parent = NULL;
#ifdef FEATURE_FT_SKIP_COMPRESSED
					ft_meta_parent_slot_offset_set(m, 0);
#endif
				}
				ft_glue_free_old(detached, &glue);
				ft_glue_fini(&glue);
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
				ft_nr_keys_store(detached, dmeta, detached_count, CMM_RELAXED);
			}
		}
		{
			size_t fm = uatomic_load(&ft->max_used_key_len,
						 CMM_RELAXED);
			uatomic_store(&detached->max_used_key_len,
				      fm > key_len ? fm - key_len : 0,
				      CMM_RELAXED);
		}

		/*
		 * Restore @detached's ordinal-cell list to its circular form now that
		 * the drains above have retired any @ft reader straddling the moved run:
		 * the run-install left the run's outer links at @ft's former neighbours
		 * (straddler-safe), so point them at @detached's sentinel for the
		 * exclusive trie's own removes / reverse walks.
		 */
		if (ft->group->ordered_list_set)
			ft_ord_finalize_circular(detached);

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

