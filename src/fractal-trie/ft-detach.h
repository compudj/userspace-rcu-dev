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
 * Flip-latch batch for cds_ft_merge_at: a urcu_flip_group plus its
 * proxies, allocated as one block and reclaimed together via call_rcu
 * once the proxied slots have been settled to their direct new targets.
 * ft_flip_proxy is 16-byte aligned so each proxy's address carries the
 * type-7 proxy tag (ft_flip_proxy_flag); malloc returns 16-byte-aligned
 * blocks at userspace addresses with the skip-len high bits clear.
 */
struct ft_flip_proxy {
	struct urcu_flip_proxy proxy;
} __attribute__((aligned(16)));

struct ft_flip_batch {
	struct rcu_head rcu_head;
	struct cds_ft *ft;
	struct urcu_flip_group group;
	unsigned int nr;
	unsigned int cap;
	struct ft_flip_proxy proxies[];
};

static
void ft_flip_batch_free_rcu(struct rcu_head *head)
{
	free(caa_container_of(head, struct ft_flip_batch, rcu_head));
}

static
struct ft_flip_batch *ft_flip_batch_alloc(struct cds_ft *ft, unsigned int cap)
{
	struct ft_flip_batch *b;

	b = malloc(sizeof(*b) + (size_t) cap * sizeof(struct ft_flip_proxy));
	if (!b)
		return NULL;
	b->ft = ft;
	urcu_flip_group_init(&b->group);
	b->nr = 0;
	b->cap = cap;
	return b;
}

/*
 * Take a caller-reserved flip batch when @pre supplies one, NULLing the
 * caller's slot to transfer ownership: from here on the consuming op frees the
 * batch (reclaim on commit, free_unpublished on its abort), and the caller
 * frees only the slots it still holds.  Falls back to a fresh allocation when
 * no batch was reserved (the standalone-op path) -- so a reserved op draws a
 * pre-allocated, unfailable batch while a normal op allocates as before.  @cap
 * is the fallback capacity; a reserved batch is already sized >= @cap by the
 * caller's read-only count pass.
 */
static
struct ft_flip_batch *ft_flip_batch_take(struct cds_ft *ft, unsigned int cap,
		struct ft_flip_batch **pre)
{
	if (pre && *pre) {
		struct ft_flip_batch *b = *pre;

		*pre = NULL;
		return b;
	}
	return ft_flip_batch_alloc(ft, cap);
}

/*
 * Free a flip batch that was allocated but never installed (no proxy stored
 * in any live slot, group never committed): a plain free, no grace period,
 * since no reader can reference it.  Used by the merge's last-fallible src
 * unlink abort path.
 */
static
void ft_flip_batch_free_unpublished(struct ft_flip_batch *b)
{
	free(b);
}

/* One release store: every proxy in the batch flips old -> new atomically. */
static
void ft_flip_batch_commit(struct ft_flip_batch *b)
{
	urcu_flip_commit(&b->group);
}

/*
 * Record a proxy {old_nf, new_nf} and return its tagged flag, to be stored
 * (sel == 0 -> old_nf, transparent) into the slot being flipped.
 */
static
struct cds_ft_inode_flag *ft_flip_batch_add(struct ft_flip_batch *b,
		struct cds_ft_inode_flag *old_nf,
		struct cds_ft_inode_flag *new_nf)
{
	struct urcu_flip_proxy *p;

	assert(b->nr < b->cap);
	p = &b->proxies[b->nr++].proxy;
	urcu_flip_proxy_init(p, &b->group, old_nf, new_nf);
	return ft_flip_proxy_flag(p);
}

/* Reclaim the batch after settle (deferred for in-flight readers). */
static
void ft_flip_batch_reclaim(struct ft_flip_batch *b)
{
	if (b->ft->exclusive)
		free(b);
	else
		b->ft->group->flavor->update_call_rcu(&b->rcu_head,
			ft_flip_batch_free_rcu);
}


/*
 * Ordinal-cell list maintenance.
 *
 * Mirrors the ORD_CHAIN chain maintenance, but the key-ordered doubly-linked
 * list threads the library-owned cells (one per distinct-key head) via
 * ft_ord_cell.ord_next / ord_prev instead of in-leaf fields.  Each point op
 * flips the (<=2) live neighbour edges through one flip-batch so a
 * bidirectional ordered reader sees the splice atomically; the spliced-in /
 * replacement cell pre-sets its own links with plain stores (not yet ord-
 * reachable), while an unspliced cell keeps its links for parked readers
 * until its deferred free.  Runtime-gated by group->ordered_list_set: a
 * point op consults these only when the list is enabled.
 *
 * RUNS UNDER WRITER EXCLUSION; no concurrent writer races, no proxy at rest.
 * Promotion (ft_unchain_node) and replace (cds_ft_replace) need NO list op:
 * the cell stays put and only cell->node is retargeted.  Bulk ops (merge /
 * graft / graft_swap / detach) maintain the list through the run helpers
 * below (run_detach / run_splice / run_replace / run_unlink) and the merge
 * interleave.
 */

struct ft_ord_cell_edge {
	struct ft_ord_cell **slot;	/* a neighbour's ord_next / ord_prev slot */
	struct ft_ord_cell *old_target;
	struct ft_ord_cell *new_target;
};

static void ft_ord_cell_flip(struct cds_ft *ft, struct ft_ord_cell_edge *edges,
		unsigned int n);

/*
 * Append an endpoint (ord_cell_head / ord_cell_tail) update to a flip-edge batch
 * when @slot currently holds @match, so the endpoint transitions ATOMICALLY with
 * the neighbour edges in the same flip: a reader resolving ord_cell_head/tail via
 * ft_ord_cell_resolve_ord then sees a consistent old-XOR-new view (it never
 * observes the old head pointer together with an already-flipped back-edge).
 * @match/@newval may be NULL (empty-list transitions); the proxy mechanism and
 * the *slot == @match guard both handle NULL.  Returns the new edge count.
 */
static
unsigned int ft_ord_cell_endpoint_edge(struct ft_ord_cell **slot,
		struct ft_ord_cell *match, struct ft_ord_cell *newval,
		struct ft_ord_cell_edge *edges, unsigned int n)
{
	if (*slot == match) {
		edges[n].slot = slot;
		edges[n].old_target = match;
		edges[n].new_target = newval;
		n++;
	}
	return n;
}

/*
 * Structural min/max dup-chain HEAD of the subtree rooted at @nf, under WRITER
 * EXCLUSION (no concurrent mutation -> no skip re-anchor / flip-proxy / transient
 * empty states to handle, unlike the reader-side minmax descent).  Mirrors the
 * key ordering the ordered cell list uses: a key that ends at an internal node
 * (metadata->external_nodes, a prefix key) sorts BEFORE every longer key under
 * it, so it is the subtree minimum.  Used to locate the endpoints of the
 * contiguous ordered-list run a bulk op relocates.
 */
static
struct cds_ft_node *ft_subtree_minmax_head(struct cds_ft *ft, struct cds_ft_inode_flag *nf,
		bool want_max)
{
	enum ft_direction dir = want_max ? FT_RIGHTMOST : FT_LEFTMOST;
	uint8_t scratch;

	for (;;) {
		nf = ft_resolve_skip_compressed(ft, nf);
		if (ft_node_external(nf))
			return (struct cds_ft_node *) ft_node_ptr(nf);
		if (ft_node_compressed(nf)) {
			struct cds_ft_compressed_node *cn =
				ft_compressed_node_ptr(nf);

			nf = rcu_dereference(cn->child);
			continue;
		}
		/* Internal node. */
		{
			struct cds_ft_metadata *m =
				cds_ft_item_to_metadata(ft_node_ptr(nf));
			struct cds_ft_node *ext =
				ft_dereference_external(m->external_nodes);
			struct cds_ft_inode_flag *child;

			if (!want_max && ext)
				return ext;	/* prefix key: subtree minimum */
			child = ft_node_get_minmax(ft, nf, &scratch, dir, false);
			if (!child) {
				/*
				 * No children: a NIL-key-only internal whose
				 * external_nodes is the sole key, so it is also the
				 * subtree MAXIMUM (a single-prefix-key trie root, e.g.
				 * a detached external).  want_min returned it above.
				 */
				assert(ext != NULL);
				return ext;
			}
			nf = child;
		}
	}
}

/*
 * Move the contiguous ordered-list run whose endpoints are the cells of
 * @first_head .. @last_head (heads, in key order) OUT of @ft's ordered cell
 * list and install it as the ENTIRE ordered list of @into -- the cds_ft_detach
 * shape, where @into is a fresh EXCLUSIVE trie receiving exactly that subtree.
 * The run's internal ord links are preserved; only its two boundary edges in
 * @ft are flipped (atomic for a concurrent ordered reader, per the flip-latch),
 * @ft's head/tail are repaired, and @into's head/tail are set.  @into being
 * exclusive, clearing the run's new boundary links is a plain store.  Caller
 * gates on ordered_list_set.
 */
static
void ft_ord_cell_run_detach(struct cds_ft *ft, struct cds_ft *into,
		struct cds_ft_node *first_head, struct cds_ft_node *last_head)
{
	struct ft_ord_cell *first =
		ft_ord_cell_ptr(rcu_dereference(first_head->prev));
	struct ft_ord_cell *last =
		ft_ord_cell_ptr(rcu_dereference(last_head->prev));
	struct ft_ord_cell *pred = ft_ord_cell_resolve_ord(&first->ord_prev);
	struct ft_ord_cell *succ = ft_ord_cell_resolve_ord(&last->ord_next);
	struct ft_ord_cell_edge edges[4];
	unsigned int n = 0;

	if (pred) {
		edges[n].slot = &pred->ord_next;
		edges[n].old_target = first;
		edges[n].new_target = succ;
		n++;
	}
	if (succ) {
		edges[n].slot = &succ->ord_prev;
		edges[n].old_target = last;
		edges[n].new_target = pred;
		n++;
	}
	n = ft_ord_cell_endpoint_edge(&ft->ord_cell_head, first, succ, edges, n);
	n = ft_ord_cell_endpoint_edge(&ft->ord_cell_tail, last, pred, edges, n);
	ft_ord_cell_flip(ft, edges, n);
	/* @into is exclusive: no readers, plain stores. */
	first->ord_prev = NULL;
	last->ord_next = NULL;
	into->ord_cell_head = first;
	into->ord_cell_tail = last;
}

static
void ft_ord_cell_flip_prealloc(struct cds_ft *ft __attribute__((unused)),
		struct ft_ord_cell_edge *edges, unsigned int n,
		struct ft_flip_batch *b)
{
	unsigned int i;

	if (n == 0) {
		ft_flip_batch_free_unpublished(b);
		return;
	}
	assert(n <= b->cap);
	for (i = 0; i < n; i++)
		rcu_assign_pointer(*edges[i].slot,
			(struct ft_ord_cell *) ft_flip_batch_add(b,
				(struct cds_ft_inode_flag *) edges[i].old_target,
				(struct cds_ft_inode_flag *) edges[i].new_target));
	urcu_flip_commit(&b->group);
	for (i = 0; i < n; i++)
		rcu_assign_pointer(*edges[i].slot, edges[i].new_target);
	ft_flip_batch_reclaim(b);
}

static
void ft_ord_cell_flip(struct cds_ft *ft, struct ft_ord_cell_edge *edges,
		unsigned int n)
{
	struct ft_flip_batch *b;
	unsigned int i;

	if (n == 0)
		return;
	b = ft_flip_batch_alloc(ft, n);
	if (caa_unlikely(!b)) {
		/*
		 * Degraded fallback (point-op splices only, <= 3 edges; the
		 * merge interleave pre-allocates its batch in the fallible
		 * build phase and never lands here): sequential edge stores.
		 * A bidirectional reader between two stores can observe one
		 * neighbour's edge updated and the mirrored one not yet --
		 * transient and self-healing, never a dangling pointer.
		 */
		for (i = 0; i < n; i++)
			rcu_assign_pointer(*edges[i].slot, edges[i].new_target);
		return;
	}
	ft_ord_cell_flip_prealloc(ft, edges, n, b);
}

/*
 * Find the cell of the in-order predecessor (mode LT) / successor (mode GT)
 * of @key via the eager relational descent on the writer's cell scratch
 * iterator.  Returns NULL when none exists (@key is the new minimum/maximum).
 */
static
struct ft_ord_cell *ft_ord_cell_find_rel(struct cds_ft *ft, const uint8_t *key,
		size_t key_len, enum ft_lookup_inequality mode)
{
	struct cds_ft_iter *it = ft->ord_cell_scratch_iter;
	struct cds_ft_node *head;

	if (cds_ft_iter_set_key(it, key, key_len) != CDS_FT_STATUS_OK)
		return NULL;
	it->prefix_len = 0;
	it->node = NULL;
	if (cds_ft_lookup_inequality_impl(ft, it, mode, FT_LOOKUP_LIMIT_NONE,
			false, false) != CDS_FT_STATUS_OK)
		return NULL;
	head = cds_ft_iter_node(it);
	if (!head)
		return NULL;
	return ft_ord_cell_ptr(rcu_dereference(head->prev));
}

/*
 * Find the cell of the in-order predecessor of the freshly-inserted head
 * carried by @cell, WITHOUT re-descending from the root.  The insert just
 * walked root->leaf to attach the head, so its deepest node is the going-up
 * seed: position the writer's scratch iterator AT the new head (a live cursor)
 * and re-enter the relational lookup, which recovers the deepest node from
 * iter->node via the parent chain (its cross-call fast path) and runs the SAME
 * structural backtrack the key-based descent would -- but starting at the
 * divergence point instead of the root.  @seed_from_node suppresses the
 * ordinal-cell fast path (the new head's cell is not yet spliced).
 *
 * @key / @key_len are the head's APPLICATION-form key (set_key remaps): the
 * search key is written straight into iter_key (a memcpy, no up-walk), and the
 * cursor fields are seeded on top so read_key returns that buffer.
 *
 * Returns the predecessor cell, or NULL when the head's key is the new minimum.
 * A compressed/skip-compressed holder makes the impl fall back to a root
 * re-descent internally (correctness preserved, no descent saved for that key).
 */
static
struct ft_ord_cell *ft_ord_cell_find_pred_from_head(struct cds_ft *ft,
		const uint8_t *key, size_t key_len, struct ft_ord_cell *cell)
{
	struct cds_ft_iter *it = ft->ord_cell_scratch_iter;
	struct cds_ft_node *pred_head;

	/*
	 * Write the search key into iter_key (set_key clears cache_valid/node and
	 * sets key_len + key_off=0, path_len=0), then seed a live cursor AT the new
	 * head on top: cache_valid + node + path_len==key_depth drive the
	 * cross-call node-recovery fast path; prefix 0 = unscoped; ord_cell_node
	 * cleared so a fall-back cell cursor re-resolves from node->prev.
	 */
	it->node = NULL;
	if (cds_ft_iter_set_key(it, key, key_len) != CDS_FT_STATUS_OK)
		return NULL;
	it->node = cell->node;
	it->cache_valid = true;
	it->ord_cell_node = NULL;
	it->prefix_len = 0;
	it->path_len = it->key_len + 1;
	if (cds_ft_lookup_inequality_impl(ft, it, FT_LOOKUP_LT,
			FT_LOOKUP_LIMIT_NONE, false, true) != CDS_FT_STATUS_OK)
		return NULL;
	pred_head = cds_ft_iter_node(it);
	if (!pred_head)
		return NULL;
	return ft_ord_cell_ptr(rcu_dereference(pred_head->prev));
}

/*
 * Locate @cell's splice neighbours from its (parent-chain-wired) head and
 * pre-set the cell's own links -- invisible until the neighbour edges flip.
 * Shared by the legacy post-publish splice, the one-commit park and the
 * B-lite (external_nodes shape) pre-publish fill.
 */
static
void ft_ord_cell_prefill(struct cds_ft *ft, const uint8_t *key, size_t key_len,
		struct ft_ord_cell *cell, struct ft_ord_cell **pred_out,
		struct ft_ord_cell **succ_out)
{
	struct ft_ord_cell *pred, *succ;

	pred = ft_ord_cell_find_pred_from_head(ft, key, key_len, cell);
	if (pred)
		succ = ft_ord_cell_resolve_ord(&pred->ord_next);
	else
		/* New minimum: successor is the old list head (O(1), no descent). */
		succ = ft_ord_cell_resolve_ord(&ft->ord_cell_head);
	/* Pre-set @cell's own links; not yet reachable via the list. */
	cell->ord_prev = pred;
	cell->ord_next = succ;
	*pred_out = pred;
	*succ_out = succ;
}

/* Build the <= 4 visible neighbour edges for splicing @cell between
 * @pred / @succ.  Returns the edge count. */
static
unsigned int ft_ord_cell_splice_edges(struct cds_ft *ft,
		struct ft_ord_cell *cell, struct ft_ord_cell *pred,
		struct ft_ord_cell *succ, struct ft_ord_cell_edge *edges)
{
	unsigned int n = 0;

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
	return n;
}

/*
 * As ft_ord_cell_prefill, but locates the neighbours by a relational descent
 * on the key (the new key is still absent).  For the shapes whose fresh head
 * attaches as a holder's external_nodes (prefix / NIL keys): the from-head
 * seed needs the holder relation, which is only wired by the attach itself.
 */
static
void ft_ord_cell_prefill_by_key(struct cds_ft *ft, const uint8_t *key,
		size_t key_len, struct ft_ord_cell *cell,
		struct ft_ord_cell **pred_out, struct ft_ord_cell **succ_out)
{
	struct ft_ord_cell *pred, *succ;

	pred = ft_ord_cell_find_rel(ft, key, key_len, FT_LOOKUP_LT);
	if (pred)
		succ = ft_ord_cell_resolve_ord(&pred->ord_next);
	else
		succ = ft_ord_cell_resolve_ord(&ft->ord_cell_head);
	cell->ord_prev = pred;
	cell->ord_next = succ;
	*pred_out = pred;
	*succ_out = succ;
}

static
void ft_ord_cell_splice_at(struct cds_ft *ft, struct ft_ord_cell *cell,
		struct ft_ord_cell *pred, struct ft_ord_cell *succ)
{
	struct ft_ord_cell_edge edges[4];
	unsigned int n;

	n = ft_ord_cell_splice_edges(ft, cell, pred, succ, edges);
	ft_ord_cell_flip(ft, edges, n);
}

static
void ft_ord_cell_splice(struct cds_ft *ft, const uint8_t *key, size_t key_len,
		struct ft_ord_cell *cell)
{
	struct ft_ord_cell *pred, *succ;

	ft_ord_cell_prefill(ft, key, key_len, cell, &pred, &succ);
	ft_ord_cell_splice_at(ft, cell, pred, succ);
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

/* Remove @cell from the ordered cell list (its key disappeared). */
static
void ft_ord_cell_unsplice(struct cds_ft *ft, struct ft_ord_cell *cell)
{
	struct ft_ord_cell *pred = ft_ord_cell_resolve_ord(&cell->ord_prev);
	struct ft_ord_cell *succ = ft_ord_cell_resolve_ord(&cell->ord_next);
	struct ft_ord_cell_edge edges[4];
	unsigned int n = 0;

	if (pred) {
		edges[n].slot = &pred->ord_next;
		edges[n].old_target = cell;
		edges[n].new_target = succ;
		n++;
	}
	if (succ) {
		edges[n].slot = &succ->ord_prev;
		edges[n].old_target = cell;
		edges[n].new_target = pred;
		n++;
	}
	n = ft_ord_cell_endpoint_edge(&ft->ord_cell_head, cell, succ, edges, n);
	n = ft_ord_cell_endpoint_edge(&ft->ord_cell_tail, cell, pred, edges, n);
	/* @cell keeps its links for parked readers until its deferred free. */
	ft_ord_cell_flip(ft, edges, n);
}

/*
 * Replace @old_cell with @new_cell at the same list position (insert_replace:
 * a fresh head's cell takes the replaced head's cell slot).  @new_cell
 * inherits @old_cell's neighbours; @old_cell keeps its links for parked
 * readers until its deferred free.  O(1): reuses @old_cell's neighbours, no
 * relational descent.
 */
static
void ft_ord_cell_swap(struct cds_ft *ft, struct ft_ord_cell *old_cell,
		struct ft_ord_cell *new_cell)
{
	struct ft_ord_cell *pred = ft_ord_cell_resolve_ord(&old_cell->ord_prev);
	struct ft_ord_cell *succ = ft_ord_cell_resolve_ord(&old_cell->ord_next);
	struct ft_ord_cell_edge edges[4];
	unsigned int n = 0;

	new_cell->ord_prev = pred;
	new_cell->ord_next = succ;
	if (pred) {
		edges[n].slot = &pred->ord_next;
		edges[n].old_target = old_cell;
		edges[n].new_target = new_cell;
		n++;
	}
	if (succ) {
		edges[n].slot = &succ->ord_prev;
		edges[n].old_target = old_cell;
		edges[n].new_target = new_cell;
		n++;
	}
	n = ft_ord_cell_endpoint_edge(&ft->ord_cell_head, old_cell, new_cell, edges, n);
	n = ft_ord_cell_endpoint_edge(&ft->ord_cell_tail, old_cell, new_cell, edges, n);
	ft_ord_cell_flip(ft, edges, n);
}

/*
 * Locate the ordered-list neighbours (@pred, @succ) that a run grafted at @key
 * will splice between.  MUST be called while @dst is still payload-free (before
 * the structural attach publishes the grafted subtree), else the relational
 * descent would return a payload head as the boundary.  @key is APPLICATION form
 * (find_rel remaps).  Since the attach point is empty, @pred = last @dst key <
 * @key and @succ = first @dst key > @key (nothing of @dst's lies in the run's
 * range in between).
 */
static
void ft_ord_cell_find_splice_pos(struct cds_ft *dst, const uint8_t *key,
		size_t key_len, struct ft_ord_cell **pred_out,
		struct ft_ord_cell **succ_out)
{
	struct ft_ord_cell *pred, *succ;
	size_t flen = dst->group->key_len;

	if (flen != CDS_FT_LEN_VARIABLE && key_len != flen) {
		/*
		 * Internal graft on a FIXED-length group (ft_graft_keylen, e.g.
		 * cds_ft_merge_at's detach+graft path): @key is the graft
		 * PREFIX, shorter than the group's key length, which the public
		 * relational lookups reject -- probing with it verbatim came
		 * back empty and the grafted run was silently never spliced
		 * (merged keys invisible to ordered iteration).  Probe with the
		 * prefix PADDED to the fixed length instead: the attach point
		 * is empty (graft returns POPULATED_ERROR otherwise), so no
		 * @dst key starts with @key, and
		 *   LT(key . min..min) = last @dst key below the prefix range,
		 *   GT(key . max..max) = first @dst key above it.
		 * Pad bytes are the ordinal-space extremes mapped back to
		 * application form (find_rel remaps app -> ordinal).
		 */
		const struct cds_ft_key_map *km = &dst->group->key_map;
		uint8_t pad_min = km->identity ? 0x00 : km->ordinal_to_key[0x00];
		uint8_t pad_max = km->identity ? 0xff : km->ordinal_to_key[0xff];
		uint8_t pad[FT_MAX_KEY_LEN];

		assert(key_len < flen && flen <= FT_MAX_KEY_LEN);
		memcpy(pad, key, key_len);
		memset(pad + key_len, pad_min, flen - key_len);
		pred = ft_ord_cell_find_rel(dst, pad, flen, FT_LOOKUP_LT);
		if (pred) {
			succ = ft_ord_cell_resolve_ord(&pred->ord_next);
		} else {
			memset(pad + key_len, pad_max, flen - key_len);
			succ = ft_ord_cell_find_rel(dst, pad, flen,
					FT_LOOKUP_GT);
		}
		*pred_out = pred;
		*succ_out = succ;
		return;
	}

	pred = ft_ord_cell_find_rel(dst, key, key_len, FT_LOOKUP_LT);
	if (pred)
		succ = ft_ord_cell_resolve_ord(&pred->ord_next);
	else
		succ = ft_ord_cell_find_rel(dst, key, key_len, FT_LOOKUP_GT);
	*pred_out = pred;
	*succ_out = succ;
}

/*
 * Splice the contiguous ordered-list run [@run_first .. @run_last] (already
 * linked internally, in key order) into @dst's ordered cell list BETWEEN the
 * given neighbours @pred and @succ -- the cds_ft_graft shape, where the run is
 * the source trie's whole list attached at an EMPTY point in @dst (graft returns
 * POPULATED_ERROR otherwise, so no @dst key interleaves the run's range).
 *
 * @pred / @succ MUST be located BEFORE the structural attach publishes the
 * payload into @dst (see ft_ord_cell_find_splice_pos): a relational descent run
 * after the payload is live would return a PAYLOAD head (part of the run itself)
 * as the boundary.  @pred / @succ are @dst-original cells, which graft never
 * moves, so they stay valid until this splice.
 *
 * Pre-sets the run's outer links (run not yet reachable in @dst), flips the
 * <=2 boundary edges atomically (for @dst's live readers), and repairs @dst
 * head/tail.  The run's source trie must already have released it (head/tail
 * cleared + a grace period) so no source reader is mid-run.
 */
static
void ft_ord_cell_run_splice(struct cds_ft *dst, struct ft_ord_cell *run_first,
		struct ft_ord_cell *run_last, struct ft_ord_cell *pred,
		struct ft_ord_cell *succ)
{
	struct ft_ord_cell_edge edges[4];
	unsigned int n = 0;

	/* Pre-set the run's outer links; not yet reachable via @dst's list. */
	run_first->ord_prev = pred;
	run_last->ord_next = succ;
	if (pred) {
		edges[n].slot = &pred->ord_next;
		edges[n].old_target = succ;
		edges[n].new_target = run_first;
		n++;
	}
	if (succ) {
		edges[n].slot = &succ->ord_prev;
		edges[n].old_target = pred;
		edges[n].new_target = run_last;
		n++;
	}
	n = ft_ord_cell_endpoint_edge(&dst->ord_cell_head, succ, run_first, edges, n);
	n = ft_ord_cell_endpoint_edge(&dst->ord_cell_tail, pred, run_last, edges, n);
	ft_ord_cell_flip(dst, edges, n);
}

/*
 * Replace the run [@d_first .. @d_last] currently in @dst's ordered list with
 * the run [@s_first .. @s_last] at the SAME position -- the cds_ft_graft_swap
 * shape, where @dst's subtree-at-key (run_D) is swapped out for the swap trie's
 * content (run_S).  @s_first may be NULL (empty swap -> run_D just leaves and the
 * gap closes).  The position is taken from run_D's own neighbours (no relational
 * descent: the swap exchanges two subtrees at the same key, so run_S lands
 * exactly where run_D was).  Atomic for @dst's live readers via one flip of the
 * <=2 boundary edges.  run_D keeps its links for parked readers; the caller
 * re-homes run_D into the swap trie afterwards.
 */
static
void ft_ord_cell_run_replace(struct cds_ft *dst,
		struct ft_ord_cell *d_first, struct ft_ord_cell *d_last,
		struct ft_ord_cell *s_first, struct ft_ord_cell *s_last)
{
	struct ft_ord_cell *pred = ft_ord_cell_resolve_ord(&d_first->ord_prev);
	struct ft_ord_cell *succ = ft_ord_cell_resolve_ord(&d_last->ord_next);
	struct ft_ord_cell *new_first = s_first ? s_first : succ;
	struct ft_ord_cell *new_last = s_last ? s_last : pred;
	struct ft_ord_cell_edge edges[4];
	unsigned int n = 0;

	if (s_first) {
		/* Pre-set run_S's outer links; not yet reachable via @dst. */
		s_first->ord_prev = pred;
		s_last->ord_next = succ;
	}
	if (pred) {
		edges[n].slot = &pred->ord_next;
		edges[n].old_target = d_first;
		edges[n].new_target = new_first;
		n++;
	}
	if (succ) {
		edges[n].slot = &succ->ord_prev;
		edges[n].old_target = d_last;
		edges[n].new_target = new_last;
		n++;
	}
	n = ft_ord_cell_endpoint_edge(&dst->ord_cell_head, d_first, new_first, edges, n);
	n = ft_ord_cell_endpoint_edge(&dst->ord_cell_tail, d_last, new_last, edges, n);
	ft_ord_cell_flip(dst, edges, n);
}

/*
 * Remove the contiguous run [@first_head .. @last_head] from @ft's ordered list
 * WITHOUT re-homing it -- the cds_ft_merge source side, where the run's cells
 * disperse (survivors are spliced into dst, collided heads are freed).  Relink
 * the two boundary edges (atomic for @ft's live readers) and repair head/tail;
 * the run cells keep their stale links (caller no longer references them as a
 * run).  Whole-list removal (pred == succ == NULL) clears head/tail.
 */
#ifdef FEATURE_FT_MERGE
static
void ft_ord_cell_run_unlink(struct cds_ft *ft, struct cds_ft_node *first_head,
		struct cds_ft_node *last_head)
{
	struct ft_ord_cell *first =
		ft_ord_cell_ptr(rcu_dereference(first_head->prev));
	struct ft_ord_cell *last =
		ft_ord_cell_ptr(rcu_dereference(last_head->prev));
	struct ft_ord_cell *pred = ft_ord_cell_resolve_ord(&first->ord_prev);
	struct ft_ord_cell *succ = ft_ord_cell_resolve_ord(&last->ord_next);
	struct ft_ord_cell_edge edges[4];
	unsigned int n = 0;

	if (pred) {
		edges[n].slot = &pred->ord_next;
		edges[n].old_target = first;
		edges[n].new_target = succ;
		n++;
	}
	if (succ) {
		edges[n].slot = &succ->ord_prev;
		edges[n].old_target = last;
		edges[n].new_target = pred;
		n++;
	}
	n = ft_ord_cell_endpoint_edge(&ft->ord_cell_head, first, succ, edges, n);
	n = ft_ord_cell_endpoint_edge(&ft->ord_cell_tail, last, pred, edges, n);
	ft_ord_cell_flip(ft, edges, n);
}
#endif /* FEATURE_FT_MERGE */

/*
 * Set @child's parent back-pointer to a raw flag @value (a flip-proxy)
 * verbatim, WITHOUT touching parent_slot_offset.  Mirrors ft_set_parent's
 * child-kind dispatch; the settle step later replaces @value with the
 * real parent via ft_set_parent (which does maintain skip_slot).
 */
static
void ft_set_parent_raw(struct cds_ft *ft, struct cds_ft_inode_flag *child,
		struct cds_ft_inode_flag *value)
{
	(void) ft;
	if (!child)
		return;
	/* Flip-proxy child: transient slot value, not a node (see
	 * ft_set_parent); the parking mutator wires the real child. */
	if (caa_unlikely(ft_node_flip_proxy(child)))
		return;
#ifdef FEATURE_FT_SKIP_COMPRESSED
	if (ft_node_skip_compressed(child)) {
		struct cds_ft_compressed_node *cn = ft_skip_to_compressed(ft, child);

		rcu_assign_pointer(cds_ft_item_to_metadata(
			(struct cds_ft_inode *) cn)->parent, value);
		return;
	}
	if (ft_node_compressed(child)) {
		struct cds_ft_compressed_node *cn = ft_compressed_node_ptr(child);

		rcu_assign_pointer(cds_ft_item_to_metadata(
			(struct cds_ft_inode *) cn)->parent, value);
		return;
	}
#endif
	if (ft_node_external(child)) {
		/*
		 * Ordered list on: store the raw flag (a flip-proxy) into the head's
		 * cell->parent, not over its prev (which is the cell pointer).  The
		 * read path resolves cell->parent THEN the flip-proxy, so a proxy
		 * parked in cell->parent settles correctly; the later ft_set_parent
		 * replaces it with the real parent.  List off / non-cell: the head's
		 * prev IS the flagged parent, so store the proxy directly there.
		 */
		if (ft->ordered_list)
			ft_ord_cell_set_parent((struct cds_ft_node *) child, value);
		else
			rcu_assign_pointer(((struct cds_ft_node *) child)->prev, value);
		return;
	}
	rcu_assign_pointer(cds_ft_item_to_metadata(ft_node_ptr(child))->parent,
		value);
}

/*
 * Fill @r with a generous SUPERSET of the nodes a bulk op's commit can allocate
 * -- CDS_FT_ALLOC_RESERVE_CAP of every internal node type (its own order +
 * bitmap), the minimal order, and (speculative groups) the compressed-node
 * orders.  Drawn before the op's last fallible step, this lets the commit draw
 * and never fail on an arena allocation, so nothing after that step needs a
 * reader-observable rollback.  Used by the same-trie rekey (before its detach)
 * and by ft_graft_keylen's NOSPLIT attach (before it publishes the empty source
 * root).  A generous superset avoids predicting the exact manifest; a bulk op
 * already pays an RCU grace period, so the handful of throwaway arena
 * pops/pushes is negligible.  Returns 0, or -ENOMEM (caller drains).
 */
static
int ft_bulk_node_reserve_fill(struct cds_ft *ft, struct cds_ft_alloc_reserve *r)
{
	unsigned int ntypes = (unsigned int) (sizeof(ft_types) / sizeof(ft_types[0]));
	unsigned int i;
	int ret = 0;

	for (i = 0; i < ntypes && !ret; i++) {
		if (ft_types[i].type_class == FT_NULL)
			continue;
		ret = cds_ft_alloc_reserve_add(ft, r, CDS_FT_ALLOC_KIND_NODE,
			ft_types[i].order, ft_types[i].bitmap,
			CDS_FT_ALLOC_RESERVE_CAP);
	}
	/* Minimal order, below ft_types[0] (small / compressed nodes). */
	if (!ret && FT_ALLOC_ORDER_MIN < ft_types[0].order)
		ret = cds_ft_alloc_reserve_add(ft, r, CDS_FT_ALLOC_KIND_NODE,
			FT_ALLOC_ORDER_MIN, FT_NO_BITMAP,
			CDS_FT_ALLOC_RESERVE_CAP);
	/* Compressed-node arena (speculative groups; else compressed == NODE). */
	if (ft->group->speculative) {
		unsigned int order;
		unsigned int cmax = ft_compressed_order(FT_SKIP_LEN_MAX);

		if (cmax > FT_ALLOC_ORDER_MAX)
			cmax = FT_ALLOC_ORDER_MAX;
		for (order = FT_ALLOC_ORDER_MIN; order <= cmax && !ret; order++)
			ret = cds_ft_alloc_reserve_add(ft, r,
				CDS_FT_ALLOC_KIND_COMPRESSED, order,
				FT_NO_BITMAP, CDS_FT_ALLOC_RESERVE_CAP);
	}
	return ret;
}

