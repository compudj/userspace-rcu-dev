// SPDX-FileCopyrightText: 2012-2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-only

/*
 * src/fractal-trie/ft-insert.h
 *
 * Userspace RCU library - Fractal Trie: insert + one-commit publish + compressed-path split; shared flip-batch and node-reserve helpers live here.
 *
 * Implementation unit: #included once by fractal-trie.c, in dependency
 * order, into a single translation unit (preserves cross-module inlining).
 * Not a standalone header.
 */
#ifndef FRACTAL_TRIE_IMPL
#error "ft-insert.h is an implementation unit; #include it from fractal-trie.c only"
#endif

/*
 * One-commit insert state (ordered-list fresh-head insert): the attach
 * machinery parks a flip proxy in the structural slot (resolving to the OLD
 * value, so the key stays invisible) and records the settle information here;
 * insert_done then adds the ordered-list neighbour edges to the SAME batch and
 * makes the key reachable in the structural index AND spliced into the cell
 * list with one urcu_flip_commit -- a reader can never observe the fresh head
 * without its cell in the list (2026-06 review, 2.13).  @batch == NULL: direct
 * publish (ordered list off, or a shape not yet converted).
 */
struct ft_insert_commit {
	struct ft_flip_batch *batch;		/* armed at the publish site */
	struct cds_ft_inode_flag **slot;	/* parked slot (settle target) */
	struct cds_ft_inode_flag *slot_value;	/* canonical value to settle */
	struct cds_ft_inode_flag *parent_nf;	/* @slot's owner (publish settle) */
	/*
	 * Count-propagation base for the post-commit +1 (the key only counts
	 * once reachable): the deepest node whose nr_keys must reflect the
	 * fresh key.  NULL = the caller's *d.pnfp.
	 */
	struct cds_ft_inode_flag *count_from;
	/*
	 * Old compressed node replaced by the parked publish: readers keep
	 * resolving the proxy to it until the commit, so its (grace-period-
	 * deferred) free must be queued only AFTER the commit -- a free queued
	 * pre-commit would not cover readers that pick the proxy up later.
	 */
	struct cds_ft_compressed_node *free_old_cn;
	bool publish_to_parent;			/* settle via ft_publish_to_parent */
	bool spliced;				/* cell already spliced (B-lite shape) */
};

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

/* Flip-batch helpers (defined with the flip machinery, after the readers). */
static struct ft_flip_batch *ft_flip_batch_alloc(struct cds_ft *ft,
		unsigned int cap);
static struct ft_flip_batch *ft_flip_batch_take(struct cds_ft *ft,
		unsigned int cap, struct ft_flip_batch **pre);
static struct cds_ft_inode_flag *ft_flip_batch_add(struct ft_flip_batch *b,
		struct cds_ft_inode_flag *old_nf,
		struct cds_ft_inode_flag *new_nf);

/*
 * Publish @new_top into @slot (owned by @parent_nf): direct via
 * ft_publish_to_parent, or -- one-commit insert, @ic armed -- park a flip
 * proxy that keeps resolving to the old slot value until insert_done's single
 * commit, recording the settle (which then runs the real ft_publish_to_parent,
 * including its dual skip-slot maintenance).  The caller must have wired
 * @new_top's parent back-pointer already (parent-before-publish; with a parked
 * proxy the cluster only becomes reachable at the commit, by which time the
 * wiring is complete either way).
 */
static
void ft_insert_publish_or_park(struct cds_ft *ft,
		struct cds_ft_inode_flag *parent_nf,
		struct cds_ft_inode_flag **slot,
		struct cds_ft_inode_flag *new_top,
		struct ft_insert_commit *ic)
{
	if (ic && ic->batch) {
		rcu_assign_pointer(*slot,
			ft_flip_batch_add(ic->batch, *slot, new_top));
		ic->slot = slot;
		ic->slot_value = new_top;
		ic->parent_nf = parent_nf;
		ic->publish_to_parent = true;
		return;
	}
	ft_publish_to_parent(ft, parent_nf, slot, new_top);
}

/*
 * Arm the one-commit batch just before a fresh-head publish (fallible; the
 * caller's error unwind runs with nothing published).  No-op when the ordered
 * list is off or no @ic is threaded (insert_replace, bulk builders).  Returns
 * 0, or -ENOMEM.
 */
static
int ft_insert_commit_arm(struct cds_ft *ft, struct ft_insert_commit *ic)
{
	if (!ic || !ft->ordered_list)
		return 0;
	ic->batch = ft_flip_batch_alloc(ft, 5);
	if (!ic->batch)
		return -ENOMEM;
	return 0;
}

/*
 * Split a compressed node during insert when the new key diverges
 * from the compressed path at position @diverge_pos.
 *
 * Builds the following structure bottom-up:
 *
 *   [prefix compressed/internal] -> [branch internal]
 *                                    +- old_ordinal -> [suffix compressed/internal] -> old_child
 *                                    `- new_ordinal -> [new branch compressed/internal] -> new_leaf
 *
 * If diverge_pos == 0, no prefix is needed.  If the suffix or new
 * branch is 0 bytes, the child is placed directly.  If 1 byte, a
 * single-child internal node is used.  If >= 2 bytes, a compressed
 * node is created.
 *
 * The old compressed node's external_nodes (if any) are preserved
 * at the prefix level (or the branch if no prefix).
 *
 * Publishes the result at @parent_slot via rcu_assign_pointer and
 * frees the old compressed node.  Returns 0 on success, -ENOMEM
 * on allocation failure (compressed node left in place).
 */
static
int ft_split_compressed_insert(struct cds_ft *ft,
		struct cds_ft_inode_flag **parent_slot,
		struct cds_ft_inode_flag *compressed_flag,
		const uint8_t *iter_key,	/* key bytes at compressed node's depth */
		unsigned int remaining_key,	/* key bytes remaining from compressed depth */
		unsigned int diverge_pos,	/* position within compressed path */
		struct cds_ft_node *child_node,	/* new external node to insert */
		unsigned int node_depth,	/* depth of the compressed node */
		struct ft_insert_commit *ic)	/* one-commit insert, may be NULL */
{
	struct cds_ft_compressed_node *cn = ft_compressed_node_ptr(compressed_flag);
	struct cds_ft_metadata *cn_meta =
		cds_ft_item_to_metadata((struct cds_ft_inode *) cn);

	/*
	 * Compressed metadata never carries external_nodes
	 * (ft_metadata_set_external_nodes aborts on a compressed target).
	 * The prefix builders below rely on it.
	 */
	assert(!cn_meta->external_nodes);
	FT_TP(split_compressed_insert_enter, (const void *) cn,
		cn->len, diverge_pos);
	struct cds_ft_inode_flag *old_suffix_flag, *new_branch_flag;
	struct cds_ft_inode_flag *branch_flag, *top_flag;
	struct cds_ft_inode_flag *created[FT_MAX_DEPTH];
	int nr_created = 0;
	unsigned int suffix_len = cn->len - diverge_pos - 1;
	unsigned int new_len = remaining_key - diverge_pos - 1;
	uint8_t old_ordinal = cn->key_bytes[diverge_pos];
	uint8_t new_ordinal = iter_key[diverge_pos];
	unsigned long old_child_nr_keys;
	int ret;
	/*
	 * Two-phase publish: build the cluster invisibly, then wire the
	 * deferred back-pointers and swing the parent slot, at the end.
	 * See the rcu-mutation build-invisible pattern.
	 *
	 * suffix_len >= 1: the branch's children (suffix, new) are new cluster
	 * nodes -- set their back-pointers normally; only the live old child
	 * into the new suffix (cn->child -> sfx) is deferred (deferred edge 1).
	 * suffix_len == 0: the branch is a cluster-leaf (old direction is the
	 * live cn->child, new direction the new subtree); set_nth defers BOTH
	 * its children, wired here at publish to the final branch_flag (deferred
	 * edges 1 and 2).
	 */
	struct cds_ft_inode_flag *sfx_skip_flag = NULL;
	struct cds_ft_inode_flag *deferred_child = NULL;
	struct cds_ft_inode_flag *deferred_parent = NULL;
	struct cds_ft_inode_flag **deferred_slot = NULL;
	struct cds_ft_inode_flag *deferred_child2 = NULL;
	struct cds_ft_inode_flag **deferred_slot2 = NULL;
	bool branch_cluster_leaf = (suffix_len == 0);

	unsigned int junction_depth = node_depth + diverge_pos;

	/* Compute old child's nr_keys for the new nodes. */
	if (!ft_node_external(cn->child)) {
		struct cds_ft_metadata *cm =
			cds_ft_item_to_metadata(ft_node_ptr(cn->child));
		old_child_nr_keys = ft_nr_keys_get(cm);
	} else if (cn->child) {
		old_child_nr_keys = 1;	/* external leaf */
	} else {
		old_child_nr_keys = 0;
	}

	/* 1. Build old suffix -> old child. */
	if (suffix_len >= 1) {
		struct cds_ft_compressed_node *sfx;
		struct cds_ft_metadata *sfx_meta;

		sfx = alloc_compressed_node(ft, suffix_len, &sfx_meta);
		if (!sfx) goto error;
		sfx->child = cn->child;
		sfx->len = suffix_len;
		memcpy(sfx->key_bytes, &cn->key_bytes[diverge_pos + 1], suffix_len);
		sfx_meta->nr_child = 1;
		ft_nr_keys_store(sfx_meta, old_child_nr_keys, CMM_RELAXED);
		old_suffix_flag = ft_compressed_node_flag(sfx);	/* PLAIN: install + recover sfx directly */
		sfx_skip_flag = ft_publish_compressed(ft, sfx, old_suffix_flag);	/* skip form for the slot */
		created[nr_created++] = old_suffix_flag;	/* track PLAIN so the error path frees sfx directly */
		/*
		 * Defer re-parenting the live old child to publish: writing
		 * cn->child's back-pointer now would make this unpublished
		 * cluster observable from the bottom (and the branch install
		 * below would otherwise recover sfx through it).  sfx->child
		 * already points at cn->child (a write into the new sfx only).
		 */
		deferred_child = cn->child;
		deferred_parent = old_suffix_flag;
		deferred_slot = &sfx->child;
	} else {
		/* suffix_len == 0: old child directly. */
		old_suffix_flag = cn->child;
	}

	/* 2. Build new branch -> new leaf. */
	if (new_len >= 1) {
		struct cds_ft_compressed_node *nb;
		struct cds_ft_metadata *nb_meta;

		nb = alloc_compressed_node(ft, new_len, &nb_meta);
		if (!nb) goto error;
		nb->child = (struct cds_ft_inode_flag *) child_node;
		nb->len = new_len;
		{
			unsigned int k;

			for (k = 0; k < new_len; k++)
				nb->key_bytes[k] = iter_key[diverge_pos + 1 + k];
		}
		nb_meta->nr_child = 1;
		ft_nr_keys_store(nb_meta, 1, CMM_RELAXED);
		new_branch_flag = ft_compressed_node_flag(nb);
		ft_set_parent(ft, nb->child, new_branch_flag, NULL);
		new_branch_flag = ft_publish_compressed(ft, nb, new_branch_flag);
		created[nr_created++] = new_branch_flag;
	} else {
		/* new_len == 0: child_node directly. */
		new_branch_flag = (struct cds_ft_inode_flag *) child_node;
	}

	/* 3. Build branch node with both children. */
	{
		struct cds_ft_inode_flag *dest = NULL;
		struct cds_ft_metadata *branch_meta;

		/* First child: old direction. */
		ret = ft_node_set_nth(ft, &dest, old_ordinal, old_suffix_flag, NULL, NULL,
				junction_depth, branch_cluster_leaf);
		if (ret) goto error;
		created[nr_created++] = dest;
		branch_flag = dest;

		/* Second child: new direction. */
		{
			struct cds_ft_inode *old_recompacted = NULL;

			branch_meta = cds_ft_item_to_metadata(ft_node_ptr(dest));
			ret = ft_node_set_nth(ft, &dest, new_ordinal, new_branch_flag,
					&old_recompacted, branch_meta, junction_depth,
					branch_cluster_leaf);
			if (ret) goto error;
			if (old_recompacted) {
				free_cds_ft_node(ft, old_recompacted);
				/* Update created entry to the recompacted node. */
				created[nr_created - 1] = dest;
			}
		}

		branch_meta = cds_ft_item_to_metadata(ft_node_ptr(dest));
		ft_nr_keys_store(branch_meta, old_child_nr_keys + 1,
				CMM_RELAXED);
		branch_flag = dest;

		/*
		 * sfx was installed via its PLAIN flag so ft_set_parent could
		 * recover it directly (without reading cn->child's back-pointer,
		 * which still points at the old cn).  Re-encode the old-direction
		 * slot to sfx's skip form now -- a value write into the still-
		 * unpublished branch; it becomes recoverable once cn->child's
		 * back-pointer is set at publish.
		 */
		if (suffix_len >= 1 && sfx_skip_flag != old_suffix_flag) {
			struct cds_ft_inode_flag **oslot = NULL;

			ft_node_get_nth_skip(branch_flag, &oslot, old_ordinal,
					FT_PF_NONE);
			if (oslot)
				rcu_assign_pointer(*oslot, sfx_skip_flag);
		}

		/*
		 * suffix_len == 0: the branch is a cluster-leaf, so set_nth left
		 * both of its children unparented.  Record both deferred edges
		 * (old = live cn->child, new = the new subtree) against the now-
		 * final branch_flag; phase 2 wires them.  The forward slots are
		 * already correct (value-copied by set_nth / recompaction).
		 */
		if (branch_cluster_leaf) {
			ft_node_get_nth_skip(branch_flag, &deferred_slot,
					old_ordinal, FT_PF_NONE);
			ft_node_get_nth_skip(branch_flag, &deferred_slot2,
					new_ordinal, FT_PF_NONE);
			deferred_child = cn->child;
			deferred_parent = branch_flag;
			deferred_child2 = new_branch_flag;
		}
	}

	/*
	 * 4. Build prefix -> branch (if needed).  @cn carries no
	 * external_nodes (asserted at entry), so the prefix is always a
	 * plain compressed run over key_bytes[0 .. diverge_pos).
	 */
	if (diverge_pos >= 1) {
		struct cds_ft_compressed_node *pfx;
		struct cds_ft_metadata *pfx_meta;
		struct cds_ft_inode_flag *pfx_child;

		pfx = alloc_compressed_node(ft, diverge_pos, &pfx_meta);
		if (!pfx) goto error;
		pfx->child = branch_flag;
		pfx->len = diverge_pos;
		memcpy(pfx->key_bytes, cn->key_bytes, diverge_pos);
		pfx_meta->nr_child = 1;
		ft_nr_keys_store(pfx_meta, ft_nr_keys_get(cn_meta) + 1, CMM_RELAXED);
		pfx_child = ft_compressed_node_flag(pfx);
		ft_set_parent(ft, branch_flag, pfx_child, &pfx->child);
		pfx_child = ft_publish_compressed(ft, pfx, pfx_child);
		created[nr_created++] = pfx_child;
		top_flag = pfx_child;
	} else {
		/* diverge_pos == 0: branch IS the top. */
		struct cds_ft_metadata *branch_meta =
			cds_ft_item_to_metadata(ft_node_ptr(branch_flag));
		ft_nr_keys_store(branch_meta, ft_nr_keys_get(cn_meta) + 1,
				CMM_RELAXED);
		top_flag = branch_flag;
	}

	/* 5. Publish the split structure, replacing the compressed node. */
	FT_TP(compressed_split, "insert", (const void *) cn, cn->len,
		(const void *) top_flag, diverge_pos);
	/*
	 * Compressed-split replaces the compressed node in its parent's
	 * slot via a direct ft_publish_to_parent call, bypassing
	 * ft_node_set_nth.  Emit tree_edge_set explicitly so consumers
	 * see the (parent, key_byte, top_flag) structural edge.  The
	 * compressed node sits at node_depth; iter_key points at the
	 * key bytes starting at that depth, so iter_key[-1] is the
	 * parent's key_byte that led to the compressed node (safe for
	 * node_depth >= 1, which always holds since compressed nodes
	 * are never at the root).
	 */
	FT_TP(tree_edge_set, (const void *) ft,
		(const void *) cn_meta->parent,
		(unsigned int) (node_depth - 1),
		(uint8_t) iter_key[-1],
		(const void *) top_flag);
	/*
	 * Phase 2 (publish) -- no failures past here.  Wire every back-pointer
	 * before swinging the parent's forward slot, so an up-walk that lands
	 * on the new cluster from either direction sees the back-pointers wired
	 * before the cluster becomes reader-reachable.
	 *
	 * ORDER among the deferred edges matters.  deferred_child is always the
	 * LIVE old child (cn->child) re-parented into the cluster; setting its
	 * back-pointer is itself a back-channel publish -- a reader up-walking
	 * from cn->child immediately enters the new cluster and can then scan
	 * the cluster's other (sibling) slots.  deferred_child2 is the FRESH
	 * new subtree, observable only through the cluster.  So wire the cluster
	 * top's own back-pointer and the fresh edge FIRST, and the live edge
	 * LAST: otherwise a reader entering via the live child reads a sibling
	 * slot pointing at the fresh subtree whose parent is not yet set, and
	 * its consume chain -- anchored at the live back-pointer store -- has no
	 * happens-before edge to the later fresh-parent store, so it observes a
	 * stale NULL parent (ft_skip_reanchor holder == NULL).
	 *
	 * suffix_len >= 1: only the live edge exists (cn->child -> sfx).
	 * suffix_len == 0: cluster-leaf branch's two children (live cn->child
	 * and fresh new subtree), both -> branch_flag.
	 */
	ret = ft_insert_commit_arm(ft, ic);
	if (ret)
		goto error;
	ft_set_parent(ft, top_flag, cn_meta->parent, parent_slot);
	if (deferred_child2)
		ft_set_parent(ft, deferred_child2, deferred_parent, deferred_slot2);
	if (deferred_child)
		ft_set_parent(ft, deferred_child, deferred_parent, deferred_slot);
	ft_insert_publish_or_park(ft, cn_meta->parent, parent_slot, top_flag, ic);

	/*
	 * 7. Free the old compressed node.  Parked publish: readers resolve
	 * the proxy to @cn until the commit, so defer the free past it.
	 */
	if (ic && ic->slot)
		ic->free_old_cn = cn;
	else
		free_compressed_node(ft, cn);

	return 0;

error:
	{
		int i;

		for (i = 0; i < nr_created; i++) {
			if (ft_node_compressed(created[i]))
				free_compressed_node_unpublished(ft,
					ft_compressed_node_ptr(created[i]));
			else if (ft_node_skip_compressed(created[i]))
				free_compressed_node_unpublished(ft,
					ft_skip_to_compressed(ft, created[i]));
			else
				free_cds_ft_node_unpublished(ft, ft_node_ptr(created[i]));
		}
	}
	return -ENOMEM;
}


/*
 * Split a compressed node when the insert key is shorter than the
 * compressed path (key terminates within the path).
 *
 * Builds: [prefix] -> [junction] -> [suffix] -> old_child
 *
 * The junction is an internal node at the key endpoint depth with
 * one child (the suffix direction).  The caller stores the new
 * node as external_nodes on the junction and handles publication,
 * propagation, and freeing of the old compressed node.
 *
 * @parent_slot: address of the slot in cn's parent that holds cn.  Used
 * to wire top_flag's own back-pointer into the live parent BEFORE the
 * deferred (back-channel) re-parent of cn->child into the new suffix.
 * Otherwise an up-walk from cn->child enters the new cluster and walks
 * up to top_flag, which would have parent == NULL.
 *
 * On success, sets *top_ret to the topmost node (prefix or junction)
 * and *jct_ret to the junction node.  Returns 0.
 * On failure, frees any partially created nodes and returns -ENOMEM.
 */
static
int ft_split_compressed_key_shorter(struct cds_ft *ft,
		struct cds_ft_inode_flag *compressed_flag,
		struct cds_ft_inode_flag **parent_slot,
		unsigned int remaining,
		struct cds_ft_inode_flag **top_ret,
		struct cds_ft_inode_flag **jct_ret,
		unsigned int node_depth)
{
	struct cds_ft_compressed_node *cn = ft_compressed_node_ptr(compressed_flag);
	struct cds_ft_metadata *cn_meta =
		cds_ft_item_to_metadata((struct cds_ft_inode *) cn);
	unsigned int suffix_len = cn->len - remaining - 1;

	/* Compressed metadata never carries external_nodes (see
	 * ft_split_compressed_insert); the prefix builders rely on it. */
	assert(!cn_meta->external_nodes);
	struct cds_ft_inode_flag *suffix_flag;
	struct cds_ft_inode_flag *jct_flag;
	struct cds_ft_inode_flag *top_flag;
	struct cds_ft_inode_flag *created[FT_MAX_DEPTH];
	int nr_created = 0;
	unsigned long child_nr_keys;
	int ret;
	/*
	 * Build-invisible / publish / reclaim (see rcu-mutation pattern).
	 * The live old child (cn->child) is re-parented into the new suffix or,
	 * for suffix_len == 0, straight into the junction.  Defer that single
	 * back-pointer to the failure-free tail so a later allocation failure
	 * frees the never-observed cluster with cn->child untouched.  The caller
	 * publishes the top right after we return, so this deferred (bottom)
	 * publish precedes the top forward publish.
	 */
	uint8_t jct_ordinal = cn->key_bytes[remaining];
	bool jct_cluster_leaf = (suffix_len == 0);
	struct cds_ft_inode_flag *sfx_skip_flag = NULL;
	struct cds_ft_inode_flag *deferred_child = NULL;
	struct cds_ft_inode_flag *deferred_parent = NULL;
	struct cds_ft_inode_flag **deferred_slot = NULL;

	if (!ft_node_external(cn->child)) {
		struct cds_ft_metadata *cm =
			cds_ft_item_to_metadata(ft_node_ptr(cn->child));
		child_nr_keys = ft_nr_keys_get(cm);
	} else if (cn->child) {
		child_nr_keys = 1;
	} else {
		child_nr_keys = 0;
	}

	/*
	 * Build suffix -> old child.
	 *
	 * Under SKIP_COMPRESSED, suffix_len >= 1 must produce a
	 * compressed (skip-encoded) node; a 1-child internal at this
	 * level would violate the chain-compress invariant (verify
	 * rejects it).  Under non-SC mode, suffix_len == 1 historically
	 * produced an internal node -- that's still acceptable since the
	 * invariant only applies in skip mode.
	 */
	if (suffix_len >= 2
#ifdef FEATURE_FT_SKIP_COMPRESSED
			|| (suffix_len == 1 && ft_group_skip_compressed(ft->group))
#endif
	   ) {
		struct cds_ft_compressed_node *sfx;
		struct cds_ft_metadata *sfx_meta;

		sfx = alloc_compressed_node(ft, suffix_len, &sfx_meta);
		if (!sfx) goto error;
		sfx->child = cn->child;
		sfx->len = suffix_len;
		memcpy(sfx->key_bytes, &cn->key_bytes[remaining + 1],
			suffix_len);
		sfx_meta->nr_child = 1;
		ft_nr_keys_store(sfx_meta, child_nr_keys,
			CMM_RELAXED);
		suffix_flag = ft_compressed_node_flag(sfx);	/* PLAIN: install + recover sfx directly */
		sfx_skip_flag = ft_publish_compressed(ft, sfx, suffix_flag);	/* skip form for the slot */
		created[nr_created++] = suffix_flag;	/* track PLAIN so the error path frees sfx directly */
		/*
		 * Defer re-parenting the live old child into the new suffix:
		 * writing cn->child's back-pointer now would expose the
		 * unpublished cluster from below, and the junction install
		 * below would recover sfx through it.  sfx->child already
		 * points at cn->child (a write into the new sfx only).
		 */
		deferred_child = cn->child;
		deferred_parent = suffix_flag;	/* PLAIN sfx flag */
		deferred_slot = &sfx->child;
	} else if (suffix_len == 1) {
		struct cds_ft_inode_flag *dest = NULL;

		/*
		 * 1-child internal suffix (non-SC): the live cn->child is its
		 * only child, so this node is a cluster-leaf -- defer cn->child's
		 * back-pointer.
		 */
		ret = ft_node_set_nth(ft, &dest,
			cn->key_bytes[remaining + 1],
			cn->child, NULL, NULL,
			node_depth + remaining + 1, true);
		if (ret) goto error;
		{
			struct cds_ft_metadata *m =
				cds_ft_item_to_metadata(ft_node_ptr(dest));
			ft_nr_keys_store(m, child_nr_keys,
				CMM_RELAXED);
		}
		suffix_flag = dest;
		created[nr_created++] = dest;
		deferred_child = cn->child;
		deferred_parent = dest;
		ft_node_get_nth_skip(dest, &deferred_slot,
			cn->key_bytes[remaining + 1], FT_PF_NONE);
	} else {
		suffix_flag = cn->child;
	}

	/* Junction: internal node with suffix child. */
	{
		struct cds_ft_inode_flag *dest = NULL;
		struct cds_ft_metadata *jct_meta;

		ret = ft_node_set_nth(ft, &dest,
			jct_ordinal,
			suffix_flag, NULL, NULL,
			node_depth + remaining, jct_cluster_leaf);
		if (ret) goto error;
		jct_meta = cds_ft_item_to_metadata(ft_node_ptr(dest));
		ft_nr_keys_store(jct_meta, child_nr_keys,
			CMM_RELAXED);
		jct_flag = dest;
		created[nr_created++] = dest;

		if (jct_cluster_leaf) {
			/*
			 * suffix_len == 0: the junction is the cluster-leaf and
			 * its suffix-direction child is the live cn->child.
			 * Record the deferred edge against the final junction.
			 */
			deferred_child = cn->child;
			deferred_parent = jct_flag;
			ft_node_get_nth_skip(jct_flag, &deferred_slot,
				jct_ordinal, FT_PF_NONE);
		} else if (sfx_skip_flag && sfx_skip_flag != suffix_flag) {
			/*
			 * suffix_len >= 1 compressed: sfx was installed via its
			 * PLAIN flag so ft_set_parent recovered it directly.
			 * Re-encode the junction's slot to sfx's skip form -- a
			 * value write into the still-unpublished junction; it
			 * resolves once cn->child's back-pointer is set at the tail.
			 */
			struct cds_ft_inode_flag **oslot = NULL;

			ft_node_get_nth_skip(jct_flag, &oslot, jct_ordinal,
				FT_PF_NONE);
			if (oslot)
				rcu_assign_pointer(*oslot, sfx_skip_flag);
		}
	}

	/* Prefix -> junction (no external_nodes on @cn: asserted at entry). */
	if (remaining >= 2) {
		struct cds_ft_compressed_node *pfx;
		struct cds_ft_metadata *pfx_meta;

		pfx = alloc_compressed_node(ft, remaining, &pfx_meta);
		if (!pfx) goto error;
		pfx->child = jct_flag;
		pfx->len = remaining;
		memcpy(pfx->key_bytes, cn->key_bytes, remaining);
		pfx_meta->nr_child = 1;
		ft_nr_keys_store(pfx_meta, ft_nr_keys_get(cn_meta),
			CMM_RELAXED);
		top_flag = ft_compressed_node_flag(pfx);
		ft_set_parent(ft, jct_flag, top_flag, NULL);
		top_flag = ft_publish_compressed(ft, pfx, top_flag);
		created[nr_created++] = top_flag;
	} else if (remaining == 1) {
#ifdef FEATURE_FT_SKIP_COMPRESSED
		if (ft_group_skip_compressed(ft->group)) {
			/*
			 * 1-byte prefix: emit a 1-byte compressed instead of a
			 * 1-child internal (canonical form under
			 * SKIP_COMPRESSED).
			 */
			struct cds_ft_compressed_node *pfx;
			struct cds_ft_metadata *pfx_meta;

			pfx = alloc_compressed_node(ft, 1, &pfx_meta);
			if (!pfx) goto error;
			pfx->child = jct_flag;
			pfx->len = 1;
			pfx->key_bytes[0] = cn->key_bytes[0];
			pfx_meta->nr_child = 1;
			ft_nr_keys_store(pfx_meta, ft_nr_keys_get(cn_meta),
				CMM_RELAXED);
			top_flag = ft_compressed_node_flag(pfx);
			ft_set_parent(ft, jct_flag, top_flag, &pfx->child);
			top_flag = ft_publish_compressed(ft, pfx, top_flag);
			created[nr_created++] = top_flag;
		} else
#endif
		{
			struct cds_ft_inode_flag *dest = NULL;
			struct cds_ft_metadata *pfx_meta;

			ret = ft_node_set_nth(ft, &dest, cn->key_bytes[0],
				jct_flag, NULL, NULL, node_depth, false);
			if (ret) goto error;
			pfx_meta = cds_ft_item_to_metadata(ft_node_ptr(dest));
			ft_nr_keys_store(pfx_meta, ft_nr_keys_get(cn_meta),
				CMM_RELAXED);
			top_flag = dest;
			created[nr_created++] = dest;
		}
	} else {
		/* remaining == 0: no prefix, junction IS the top. */
		top_flag = jct_flag;
	}

	/*
	 * Failure-free tail.  First wire top_flag's own back-pointer into
	 * cn's live parent (so an up-walk that enters the cluster via the
	 * deferred back-channel below finds a parent-wired top), THEN wire
	 * the single deferred back-pointer (the live old child into the new
	 * suffix / junction).  No allocation happens past here; the caller
	 * publishes the top forward immediately after we return.
	 */
	ft_set_parent(ft, top_flag, cn_meta->parent, parent_slot);
	if (deferred_child)
		ft_set_parent(ft, deferred_child, deferred_parent, deferred_slot);

	FT_TP(compressed_split, "key_shorter", (const void *) cn, cn->len,
		(const void *) top_flag, remaining);
	*top_ret = top_flag;
	*jct_ret = jct_flag;
	return 0;

error:
	{
		int i;

		for (i = 0; i < nr_created; i++) {
			if (ft_node_compressed(created[i]))
				free_compressed_node_unpublished(ft,
					ft_compressed_node_ptr(created[i]));
			else if (ft_node_skip_compressed(created[i]))
				free_compressed_node_unpublished(ft,
					ft_skip_to_compressed(ft, created[i]));
			else
				free_cds_ft_node_unpublished(ft,
					ft_node_ptr(created[i]));
		}
	}
	return -ENOMEM;
}

/*
 * We reached an unpopulated node. Create it and the children we need,
 * and then attach the entire branch to the current node. This may
 * trigger recompaction of the current node.
 *
 * ft_attach_node() ensures that a lookup will _never_ see a branch that
 * leads to a dead-end: before attaching a branch, the entire content of
 * the new branch is populated, thus creating a cluster, before
 * attaching the cluster to the rest of the trie, thus making it visible
 * to lookups.
 *
 * @external_node argument is either NULL or a pointer to the external
 * node we are replacing at the attachment location. We need to chain
 * this external node in the topmost internal node external node list in
 * that case.
 */
/*
 * Graft-transaction glue (build-invisible / publish / reclaim -- see the
 * rcu-mutation discipline).  A graft attaches a payload subtrie at a
 * non-root key.  The attach cluster ("glue") is built entirely from
 * fresh, unobservable nodes BEFORE the source root is unlinked, so an
 * allocation failure frees the glue with both tries pristine -- there is
 * nothing to roll back, and no past-sync abort().
 *
 * Every edge from the glue into LIVE data is a back-pointer re-parent
 * that must be deferred to the failure-free commit and applied only
 * after the source is unlinked and a grace period has drained its
 * readers.  Two flavours of live data:
 *   - the displaced dst old-child of a split compressed node, and
 *   - the live payload nodes pulled from the source (its old root, and
 *     any sub-compressed absorbed during canonicalization).
 * Forward edges into live data (a fresh node's child slot pointing at a
 * live node) ARE set during the build: they live in unobservable glue
 * nodes, so no reader follows them until the single commit-time publish.
 *
 * @built tracks every fresh glue node so the abort path can free them
 * (immediate free -- never observed).  @deferred records the live
 * back-pointers to wire at commit.  @free_list records old (replaced)
 * live nodes to reclaim deferred after the publish.
 *
 * The struct is instantiable more than once: cds_ft_graft uses a single
 * glue for the dst-side attach; cds_ft_graft_swap commits two (the
 * dst-side insert glue + the swap-side extracted-root glue) together.
 *
 * Defined up here (rather than with its helper bodies further down)
 * because ft_try_compress_chain, ft_build_branch and the build-only
 * graft split all reference the complete type.
 */
struct ft_graft_deferred_edge {
	struct cds_ft_inode_flag *child;	/* live node to re-parent */
	struct cds_ft_inode_flag *parent;	/* glue node it will point to */
	struct cds_ft_inode_flag **slot;	/* slot in parent holding child */
	/*
	 * cds_ft_merge_at references live subtrees from BOTH tries.  A
	 * src-origin child is drained by the early src unlink (applied at
	 * apply_deferred); a dst-origin child stays reachable via the old dst
	 * spine until the forward publish + dst drain, so its back-pointer flip
	 * must wait (applied at apply_deferred_dst).  graft / graft_swap only
	 * ever re-parent src-origin nodes, so this defaults to false and their
	 * single apply_deferred call still wires every edge.
	 */
	bool dst_origin;
};

struct ft_graft_free_item {
	void *node;		/* cds_ft_inode * or cds_ft_compressed_node * */
	bool compressed;
};

/*
 * A deferred duplicate-chain splice, used only by cds_ft_merge_at when the
 * SAME full key exists in both tries: the two LIVE external chains must be
 * concatenated under the fresh merged node @owner.  graft / graft_swap never
 * concatenate two live chains, so this is merge-only.
 *
 * @dst_head is kept as the surviving chain head: its forward owner (a fresh
 * merged node's metadata->external_nodes, or a fresh merged node's child slot)
 * and its back-pointer (@dst_head->prev = that node) are wired by the ordinary
 * Phase-1 set + deferred edge, exactly like any other re-parented external.
 * This struct carries ONLY the concatenation, which is publication-visible on
 * two live chains and is applied at commit by ft_graft_glue_apply_splices,
 * AFTER the source has been detached + drained: the @src_head chain is appended
 * to @dst_head's tail (prev-before-next, the ft_chain_node idiom, but preserving
 * src_head->next so the rest of the src chain rides along).
 */
struct ft_graft_splice {
	struct cds_ft_node *dst_head;		/* surviving head (kept first) */
	struct cds_ft_node *src_head;		/* appended to dst_head's tail */
	/*
	 * The demoted @src_head's ordered-list cell, captured by
	 * ft_graft_glue_apply_splices.  It stays REACHABLE through its src-run
	 * neighbours' stale ord_prev/ord_next until the post-publish interleave
	 * rewires them, so it is freed only by
	 * ft_graft_glue_free_collided_cells, called after the interleave, via
	 * the grace-period-deferred cell free.  NULL when the list is off.
	 */
	struct ft_ord_cell *src_cell;
};

/*
 * Inline floor sizing: a graft / graft_swap attach cluster spans at most a
 * compressed prefix + branch + suffix + a payload path of up to FT_MAX_DEPTH
 * nodes + a canonicalization wrapper, with few deferred edges and freed nodes
 * (old-child; payload top / grandchild; old cn, old src root, absorbed
 * sub-cn).  These fit the inline arrays, so graft / graft_swap never allocate
 * a backing buffer and never grow past the floor.
 *
 * cds_ft_merge_at instead builds a TREE-shaped spine (one deferred edge per
 * disjoint subtree, one free per copied node), which can far exceed the floor.
 * It calls ft_graft_glue_reserve() to move the three arrays onto a malloc'd
 * backing sized by a read-only counting pre-pass; ft_graft_glue_abort() and
 * ft_graft_glue_fini() release it.  Every accessor indexes through the
 * pointers, so the growth is invisible to the helpers.
 */
#define FT_GRAFT_GLUE_FLOOR_BUILT	(2 * FT_MAX_DEPTH + 8)
#define FT_GRAFT_GLUE_FLOOR_DEFERRED	8
#define FT_GRAFT_GLUE_FLOOR_FREE	8
#define FT_GRAFT_GLUE_FLOOR_SPLICE	8

struct ft_graft_glue {
	struct ft_graft_deferred_edge *deferred;
	int nr_deferred;
	int cap_deferred;
	struct ft_graft_free_item *free_list;
	int nr_free;
	int cap_free;
	struct cds_ft_inode_flag **built;
	int nr_built;
	int cap_built;
	struct ft_graft_splice *splices;
	int nr_splices;
	int cap_splices;
	/*
	 * The single forward store that splices the cluster into dst at
	 * commit: parent_slot is swung to top.  publish_parent is the
	 * node flag owning the slot (for compressed skip bookkeeping;
	 * NULL at the root).  The cluster top's own back-pointer into
	 * publish_parent is recorded as an ordinary deferred edge.
	 */
	struct cds_ft_inode_flag *publish_parent;
	struct cds_ft_inode_flag **publish_slot;
	struct cds_ft_inode_flag *top;
	/*
	 * Node whose nr_keys == the grafted payload's key count, and from
	 * whose parent the external-count propagation starts at commit.
	 */
	struct cds_ft_inode_flag *attached_nf;
	/*
	 * Inline floor backing.  ft_graft_glue_init points the three arrays
	 * here; graft / graft_swap never outgrow it.  ft_graft_glue_reserve
	 * repoints to a malloc'd buffer when a count would exceed its floor.
	 */
	struct ft_graft_deferred_edge deferred_floor[FT_GRAFT_GLUE_FLOOR_DEFERRED];
	struct ft_graft_free_item free_floor[FT_GRAFT_GLUE_FLOOR_FREE];
	struct cds_ft_inode_flag *built_floor[FT_GRAFT_GLUE_FLOOR_BUILT];
	struct ft_graft_splice splices_floor[FT_GRAFT_GLUE_FLOOR_SPLICE];
};

static void ft_graft_glue_track(struct ft_graft_glue *g,
		struct cds_ft_inode_flag *nf);
static void ft_graft_glue_untrack(struct cds_ft *ft, struct ft_graft_glue *g, void *node_ptr);
static bool ft_graft_glue_is_fresh(struct cds_ft *ft, struct ft_graft_glue *g,
		struct cds_ft_inode_flag *child);
static void ft_graft_glue_defer_edge(struct cds_ft *ft, struct ft_graft_glue *g,
		struct cds_ft_inode_flag *child,
		struct cds_ft_inode_flag *parent,
		struct cds_ft_inode_flag **slot);
static void ft_graft_glue_defer_free(struct ft_graft_glue *g,
		void *node, bool compressed);
static void ft_graft_glue_set_publish(struct cds_ft *ft, struct ft_graft_glue *g,
		struct cds_ft_inode_flag *parent_nf,
		struct cds_ft_inode_flag **parent_slot,
		struct cds_ft_inode_flag *top);

static struct cds_ft_inode_flag *ft_build_branch(struct cds_ft *ft,
		const uint8_t *key, unsigned int start, unsigned int end,
		struct cds_ft_inode_flag *leaf,
		unsigned long subtree_external_count,
		bool has_external_nodes,
		struct ft_graft_glue *glue);
static void ft_free_branch_unpublished(struct cds_ft *ft,
		struct cds_ft_inode_flag *top, struct cds_ft_inode_flag *leaf);

static struct cds_ft_inode_flag *ft_compress_single_child_if_needed(
		struct cds_ft *ft, struct cds_ft_inode_flag *child,
		struct ft_graft_glue *glue);

/*
 * Try to create a compressed path for a chain of single-child nodes.
 * Returns the compressed node flag on success, NULL if compression
 * is not applicable (path too short) or disabled, -ENOMEM cast to
 * pointer on allocation failure.
 *
 * @glue: when non-NULL (graft build-invisible mode), the live child's
 * back-pointer is recorded as a deferred edge rather than set now, and an
 * absorbed compressed @child (a fresh canonicalization wrapper) is
 * untracked from the glue as it is freed during the merge.
 */
#ifdef FEATURE_FT_COMPRESS
static
struct cds_ft_inode_flag *ft_try_compress_chain(struct cds_ft *ft,
		const uint8_t *key, size_t key_len, unsigned int level,
		struct cds_ft_inode_flag *child,
		struct cds_ft_node *external_nodes __attribute__((unused)),
		struct ft_graft_glue *glue)
{
	uint8_t path_len = (uint8_t)(key_len - level);
	struct cds_ft_compressed_node *cn;
	struct cds_ft_metadata *cn_meta;
	struct cds_ft_compressed_node *child_cn = NULL;
	unsigned int child_len = 0;
	uint8_t merged_len;
	int j;

	/*
	 * Length-1 compressed nodes are canonical under
	 * FEATURE_FT_SKIP_COMPRESSED: the publish wraps cn into a
	 * SKIP_X-tagged slot pointer, dispatching for free relative to
	 * the 1-child internal node it replaces.
	 */
	if (path_len < 1)
		return NULL;

	/*
	 * Chain-merge: if @child is already a compressed (or skip-
	 * compressed) node, wrapping it in another compressed prefix
	 * would violate the "no two adjacent compresseds" invariant.
	 * Absorb the child's path bytes into the outer cn so the
	 * result is a single compressed spanning
	 * (key[level..key_len-1] ++ child_cn->key_bytes) ->
	 * child_cn->child.  Bounded by FT_SKIP_LEN_MAX; on overflow,
	 * fall back to the un-merged form (rare; the residue may be
	 * cleaned up by a subsequent mutation).
	 */
#ifdef FEATURE_FT_SKIP_COMPRESSED
	if (ft_node_skip_compressed(child))
		child_cn = ft_skip_to_compressed(ft, child);
	else
#endif
	if (ft_node_compressed(child))
		child_cn = ft_compressed_node_ptr(child);
	if (child_cn) {
		child_len = child_cn->len;
		/*
		 * Cap the fused path at what one compressed node can hold.  Under
		 * skip-compressed the merged node must also stay skip-encodable, so
		 * the cap is FT_SKIP_LEN_MAX.  Without skip-compression (notably
		 * 32-bit, where FT_SKIP_LEN_MAX is 0) the node is a plain compressed
		 * and the only limit is its uint8_t len field -- cap at UINT8_MAX.
		 * Using FT_SKIP_LEN_MAX unconditionally would never fuse there and
		 * leave two adjacent compresseds, violating the invariant.
		 */
#ifdef FEATURE_FT_SKIP_COMPRESSED
		if ((unsigned int) path_len + child_len > FT_SKIP_LEN_MAX) {
#else
		if ((unsigned int) path_len + child_len > UINT8_MAX) {
#endif
			/* Overflow: leave adjacency in place. */
			child_cn = NULL;
			child_len = 0;
		}
	}
	merged_len = (uint8_t)(path_len + child_len);

	cn = alloc_compressed_node(ft, merged_len, &cn_meta);
	if (!cn)
		return (struct cds_ft_inode_flag *) (long) -ENOMEM;
	if (child_cn)
		cn->child = child_cn->child;
	else
		cn->child = child;
	cn->len = merged_len;
	for (j = 0; j < path_len; j++)
		cn->key_bytes[j] = key[level + j];
	if (child_cn)
		memcpy(&cn->key_bytes[path_len],
			child_cn->key_bytes, child_len);
	cn_meta->nr_child = 1;
	ft_nr_keys_store(cn_meta, 1, CMM_RELAXED);
	/* Compressed nodes must not carry external_nodes. */
	assert(!external_nodes);
	{
		struct cds_ft_inode_flag *cflag = ft_compressed_node_flag(cn);

		if (glue) {
			/*
			 * Build-invisible (graft): cn->child is LIVE -- either
			 * the merged-away child_cn's grandchild or the leaf
			 * itself.  Record its back-pointer for the post-sync
			 * commit instead of flipping it now.
			 *
			 * The absorbed child_cn is freed one of two ways: a fresh
			 * canonicalization wrapper (tracked in @glue) is dropped
			 * from tracking and freed here, so the abort path cannot
			 * double-free it; a LIVE compressed leaf (a re-rooted-in-
			 * place merge source absorbed into the branch run, never
			 * tracked) is still reader-reachable until the commit, so
			 * its free is DEFERRED past the grace period instead.
			 *
			 * Track the PLAIN @cflag (not the skip form returned to
			 * the caller): the abort path resolves a tracked node
			 * via ft_compressed_node_ptr, so it must not depend on
			 * cn->child's still-deferred back-pointer (which is how
			 * a skip pointer recovers its compressed node).
			 */
			ft_graft_glue_defer_edge(ft, glue, cn->child, cflag,
				&cn->child);
			if (child_cn) {
				if (ft_graft_glue_is_fresh(ft, glue,
						ft_compressed_node_flag(child_cn))) {
					ft_graft_glue_untrack(ft, glue, child_cn);
					free_compressed_node_unpublished(ft,
						child_cn);
				} else {
					ft_graft_glue_defer_free(glue, child_cn,
						true);
				}
			}
			ft_graft_glue_track(glue, cflag);
			/*
			 * Emit the creation trace but return the PLAIN flag:
			 * the caller installs @cn directly and resolves it via
			 * ft_compressed_node_ptr (cn->child's back-pointer is
			 * deferred, so the skip form would not yet resolve).
			 * The caller re-encodes the holding slot to skip before
			 * publish.
			 */
			(void) ft_publish_compressed(ft, cn, cflag);
			return cflag;
		}
		ft_set_parent(ft, cn->child, cflag, &cn->child);
		if (child_cn)
			free_compressed_node_unpublished(ft, child_cn);
		/* compressed_publish emitted by ft_publish_compressed. */
		return ft_publish_compressed(ft, cn, cflag);
	}
}
#else
static inline
struct cds_ft_inode_flag *ft_try_compress_chain(
		struct cds_ft *ft __attribute__((unused)),
		const uint8_t *key __attribute__((unused)),
		size_t key_len __attribute__((unused)),
		unsigned int level __attribute__((unused)),
		struct cds_ft_inode_flag *child __attribute__((unused)),
		struct cds_ft_node *external_nodes __attribute__((unused)),
		struct ft_graft_glue *glue __attribute__((unused)))
{
	return NULL;
}
#endif


static
int ft_attach_node(struct cds_ft *ft,
		struct cds_ft_inode_flag **attach_node_flag_ptr,
		struct cds_ft_inode_flag *attach_node_flag,
		struct cds_ft_inode_flag **old_node_flag_ptr,
		struct cds_ft_inode_flag *old_node_flag,
		const uint8_t *key,
		size_t key_len,
		unsigned int level,
		struct cds_ft_node *child_node,
		struct cds_ft_node *external_nodes,
		struct ft_insert_commit *ic)
{
	struct cds_ft_metadata *metadata = NULL;
	struct cds_ft_inode_flag *iter_node_flag, *iter_dest_node_flag,
				*created_nodes[FT_MAX_DEPTH];
	struct cds_ft_inode *old_recompacted_node = NULL;
	int ret, i, nr_created_nodes = 0;
	const uint8_t *iter_key = key + key_len;

	FT_TP(attach_node_enter, (const void *) attach_node_flag,
		(const void *) old_node_flag, level);

	dbg_printf("Attach node at level %u (old_node_flag %p, attach_node_flag_ptr %p attach_node_flag %p)\n",
		level, old_node_flag, attach_node_flag_ptr, attach_node_flag);

	assert(!old_node_flag || external_nodes);
	assert(level > 0);	/* Root is always internal; level 0 is handled directly. */
	if (attach_node_flag)
		metadata = cds_ft_item_to_metadata(ft_node_ptr(attach_node_flag));

	/* Concurrent update prevented by mutual exclusion. */
	assert(!(old_node_flag_ptr && (ft_node_ptr(*old_node_flag_ptr) && !external_nodes)));
	assert(!(attach_node_flag_ptr && ft_node_ptr(*attach_node_flag_ptr) !=
			ft_node_ptr(attach_node_flag)));
	(void) old_node_flag_ptr;	/* Used by assert above; silence -DNDEBUG. */

	/* Create new branch, starting from bottom */
	iter_node_flag = (struct cds_ft_inode_flag *) child_node;

	{
		struct cds_ft_inode_flag *compressed;
		/*
		 * Compressed nodes must not carry metadata->external_nodes.
		 * When external_nodes exist at @level, compress from
		 * level+1 (one byte shorter) and let the loop below
		 * create an internal node at @level that holds the
		 * external_nodes.
		 */
		unsigned int compress_level = external_nodes ? level + 1 : level;

		compressed = ft_try_compress_chain(ft, key, key_len,
			compress_level, iter_node_flag, NULL, NULL);
		if (compressed == (void *) (long) -ENOMEM) {
			ret = -ENOMEM;
			goto check_error;
		}
		if (compressed) {
			iter_node_flag = compressed;
			/*
			 * Track the PLAIN compressed flag, never the skip form
			 * ft_try_compress_chain returns in skip-compressed
			 * groups: a SKIP pointer encodes the CHILD's address
			 * (the application's external node for a leaf attach),
			 * so the kind dispatch in check_error's unwind would
			 * misread it as a plain node and run arena arithmetic
			 * on the application's pointer.  Same convention as the
			 * glue builders (see ft_try_compress_chain's glue arm).
			 */
			created_nodes[nr_created_nodes++] =
				ft_node_skip_compressed(compressed) ?
				ft_compressed_node_flag(
					ft_skip_to_compressed(ft, compressed)) :
				compressed;
			iter_key = key + compress_level;
			/*
			 * When external_nodes exist, compress_level = level + 1.
			 * iter_key points to key + level + 1.  The loop below
			 * runs one iteration to create an internal node at
			 * @level that dispatches on key[level] with the
			 * compressed node as child.  The loop then places
			 * external_nodes on this internal node.
			 */
		}
	}
	if ((!ft_node_compressed(iter_node_flag) &&
	     !ft_node_skip_compressed(iter_node_flag)) ||
	    external_nodes) {
		for (i = (ft_node_compressed(iter_node_flag) ||
			  ft_node_skip_compressed(iter_node_flag)) ?
				(int) level + 1 : (int) key_len;
		     i > (int) level; i--) {
			uint8_t key_value;

			key_value = *(--iter_key);
			dbg_printf("branch creation level %d, key %u\n",
					i, (unsigned int) key_value);
			iter_dest_node_flag = NULL;
			ret = ft_node_set_nth(ft, &iter_dest_node_flag, key_value, iter_node_flag, NULL, NULL,
					i - 1, false);
			if (ret) {
				dbg_printf("branch creation error %d\n", ret);
				goto check_error;
			}
			{
				struct cds_ft_metadata *branch_meta =
					cds_ft_item_to_metadata(ft_node_ptr(iter_dest_node_flag));
				ft_nr_keys_store(branch_meta, 1, CMM_RELAXED);
			}
			created_nodes[nr_created_nodes++] = iter_dest_node_flag;
			iter_node_flag = iter_dest_node_flag;
		}

		if (external_nodes) {
			struct cds_ft_metadata *iter_node_metadata;

			iter_node_metadata = cds_ft_item_to_metadata(ft_node_ptr(iter_node_flag));
			/*
			 * Phase 1 (build-invisible): write the cluster top's
			 * cluster-internal external_nodes pointer.  The
			 * back-channel publish (external_nodes->prev =
			 * iter_node_flag) is deferred to Phase 2 below, after
			 * set_nth wires iter_node_flag's parent -- otherwise an
			 * up-walk from external_nodes (still reachable through
			 * the old slot at attach_node_flag_ptr) lands on
			 * iter_node_flag with parent == NULL.
			 */
			ft_metadata_set_external_nodes(iter_node_flag,
				iter_node_metadata, external_nodes);
			ft_nr_keys_store(iter_node_metadata,
				ft_nr_keys_get(iter_node_metadata) + 1, CMM_RELAXED);
		}
	}

	/* Publish branch. */
	{
		uint8_t key_value;
		struct cds_ft_inode_flag *slot_child = iter_node_flag;

		key_value = *(--iter_key);
		dbg_printf("publish branch at level %d, key %u\n", level - 1, (unsigned int) key_value);

		ret = ft_insert_commit_arm(ft, ic);
		if (ret)
			goto check_error;
		if (ic && ic->batch) {
			/*
			 * One-commit insert: park a flip proxy in the slot
			 * instead of the cluster top.  It resolves to the OLD
			 * slot value (the displaced external, or NULL) until
			 * insert_done commits it together with the ordered-
			 * list edges, so the fresh key stays invisible here.
			 * The set_nth machinery skips proxy children (see
			 * ft_set_parent); the real top's wiring follows below,
			 * once the final slot is known.
			 */
			slot_child = ft_flip_batch_add(ic->batch,
				old_node_flag, iter_node_flag);
			ic->slot_value = iter_node_flag;
		}

		/* We need to use set_nth on the previous level. */
		iter_dest_node_flag = attach_node_flag;
		ret = ft_node_set_nth(ft, &iter_dest_node_flag, key_value, slot_child,
				&old_recompacted_node, metadata, level - 1, false);
		if (ret) {
			dbg_printf("branch publish error %d\n", ret);
			goto check_error;
		}
		if (ic && ic->batch) {
			struct cds_ft_inode_flag **slot_ptr = NULL;

			/*
			 * Fully wire the real top NOW (parent, slot offset,
			 * incoming_byte) -- every write lands in the FRESH
			 * top's own metadata/cell, invisible while the slot
			 * holds the proxy, and the splice-position search at
			 * insert_done depends on it (the structural up-walk
			 * rebuilds the new key through these fields).  A
			 * recompact replaced the target with a fresh copy (the
			 * proxy slot rode along; the copied LIVE children were
			 * re-parented by the recompact sweep, which skips the
			 * proxy) -- wire against the copy.
			 */
			ft_node_get_nth_skip(iter_dest_node_flag, &slot_ptr,
				key_value, FT_PF_NONE);
			assert(slot_ptr);
			ft_set_parent(ft, iter_node_flag, iter_dest_node_flag,
				slot_ptr);
			ic->slot = slot_ptr;
		}
		/*
		 * Phase 2: iter_node_flag's parent is now wired (by
		 * ft_node_set_nth above, either in-place or via recompact's
		 * reparent loop; by the explicit raw store for a one-commit
		 * insert).  Wire the back-channel from the live displaced
		 * external before the outer forward publish.
		 */
		ft_publish_external_nodes_prev(ft, iter_node_flag, external_nodes);
		/* Attach branch (unlink the old node from the trie).
		 * ft_publish_to_parent handles skip pointer update
		 * if the attach target is a compressed node's child.
		 */
		ft_publish_to_parent(ft, attach_node_flag,
			attach_node_flag_ptr, iter_dest_node_flag);

		/* Reclaim safely after unlink. */
		if (old_recompacted_node)
			free_cds_ft_node(ft, old_recompacted_node);
	}

	/* Success */
	ret = 0;

check_error:
	if (ret) {
		/*
		 * All goto-check_error paths in this function are before
		 * ft_publish_to_parent, so created_nodes[] never escaped
		 * the writer's stack -- immediate-free is safe.  An armed
		 * one-commit batch was never parked anywhere visible (the
		 * final set_nth failed before storing): release it.
		 */
		if (ic && ic->batch) {
			ft_flip_batch_free_unpublished(ic->batch);
			ic->batch = NULL;
		}
		for (i = 0; i < nr_created_nodes; i++) {
			if (ft_node_compressed(created_nodes[i]))
				free_compressed_node_unpublished(ft,
					ft_compressed_node_ptr(created_nodes[i]));
			else
				free_cds_ft_node_unpublished(ft, ft_node_ptr(created_nodes[i]));
		}
	}
	FT_TP(attach_node_exit, (int) ret);
	return ret;
}

static
void ft_chain_node(struct cds_ft_node *last_node, struct cds_ft_node *node)
{
	FT_TP(chain_node, (const void *) last_node, (const void *) node);
	/*
	 * Add node to tail of list to ensure that RCU traversals will
	 * always see either the prior node or the newly added if
	 * executed concurrently with a sequence of add followed by del
	 * on the same key. Safe against concurrent RCU read traversals.
	 *
	 * The prev pointer is write-side only (mutex-held), so a plain
	 * store is sufficient.
	 */
	node->prev = last_node;
	node->next = NULL;
	rcu_assign_pointer(last_node->next, node);
}

/*
 * There are a few cases to cover for add:
 *
 * 1) There is already an external node at that key. Chain this new node
 *    with the existing node (duplicate).
 * 2) There is already an internal node with associated external node at
 *    that key. Chain this new node with the existing node (duplicate).
 * 3) The traversal ends before reaching the end of the lookup key:
 *    3.1) The last node encountered during traversal is an internal
 *         node. Attach a new cluster as child of this internal node.
 *    3.2) The last node encountered during traversal is an external
 *         node. Need to transform this external node into an internal
 *         node with associated external node, attach a new cluster as
 *         child of this internal node, and populate this new internal
 *         node into the trie to replace the prior external node.
 */

/*
 * ft_insert_compressed_past_child: key continues past a compressed
 * node's external child.  Build a branch below the child and propagate
 * density / external count through the snapshot.
 *
 * Returns 0 on success, -ENOMEM on allocation failure.
 */
static
int ft_insert_compressed_past_child(struct cds_ft *ft,
		struct ft_descent *d,
		const uint8_t *key, size_t key_len,
		struct cds_ft_compressed_node *cn,
		struct cds_ft_node *node,
		struct ft_insert_commit *ic)
{
	struct cds_ft_inode_flag *branch;
	struct cds_ft_metadata *br_meta;
	int ret;

	ret = ft_insert_commit_arm(ft, ic);
	if (ret)
		return ret;	/* nothing built yet */

	/*
	 * Case 1 (external at END of compressed path): build a
	 * branch for the continuing key, with an internal node at
	 * d->depth + cn->len that holds the old external child as
	 * external_nodes and dispatches the next key byte.
	 */
	{
		unsigned int br_start = d->depth + cn->len;
		struct cds_ft_inode_flag *inner;
		struct cds_ft_inode_flag *dest = NULL;

		inner = ft_build_branch(ft, key,
			br_start + 1, key_len,
			(struct cds_ft_inode_flag *) node, 1, false, NULL);
		if (!inner) {
			ret = -ENOMEM;
			goto arm_unwind;
		}
		ret = ft_node_set_nth(ft, &dest, key[br_start],
			inner, NULL, NULL, br_start, false);
		if (ret) {
			/*
			 * Free the built branch (it was leaked before): the
			 * cluster is writer-private, nothing was published.
			 * insert_done resets node->prev for the retry.
			 */
			ft_free_branch_unpublished(ft, inner,
				(struct cds_ft_inode_flag *) node);
			ret = -ENOMEM;
			goto arm_unwind;
		}
		branch = dest;
		br_meta = cds_ft_item_to_metadata(ft_node_ptr(branch));
		/*
		 * Phase 1 (build-invisible): wire branch's own parent and its
		 * cluster-internal external_nodes pointer.  The back-channel
		 * publish (cn->child->prev = branch) is deferred to Phase 2
		 * below -- otherwise an up-walk from cn->child (still reachable
		 * through the unmodified cn) lands on branch with parent NULL.
		 */
		ft_set_parent(ft, branch, d->nf, &cn->child);
		ft_metadata_set_external_nodes(branch, br_meta,
			(struct cds_ft_node *) cn->child);
		/*
		 * Count only the pre-existing key (old external from
		 * the compressed child).  The new key's +1 is added
		 * by ft_propagate_external_count_parent below.
		 */
		ft_nr_keys_store(br_meta, 1, CMM_RELAXED);
	}
	/* Phase 2: back-channel + forward publish. */
	ft_publish_external_nodes_prev(ft, branch, (struct cds_ft_node *) cn->child);
	ft_insert_publish_or_park(ft, d->nf, &cn->child, branch, ic);
	/* One-commit (parked): the +1 follows the commit at insert_done. */
	if (ic && ic->slot)
		ic->count_from = branch;
	else
		ft_propagate_external_count_parent(ft, branch, 1);
	return 0;
arm_unwind:
	if (ic && ic->batch) {
		ft_flip_batch_free_unpublished(ic->batch);
		ic->batch = NULL;
	}
	return ret;
}

/*
 * ft_insert_compressed_diverge: key diverges from the compressed path
 * at position @j.  Split the compressed node and insert the new key.
 *
 * Returns 0 on success, negative errno on failure.
 */
static
int ft_insert_compressed_diverge(struct cds_ft *ft,
		struct ft_descent *d,
		const uint8_t *iter_key,
		unsigned int remaining, unsigned int j,
		struct cds_ft_node *node,
		struct ft_insert_commit *ic)
{
	int dret;

	dret = ft_split_compressed_insert(ft,
		d->nfp, d->nf, iter_key, remaining,
		j, node, d->depth, ic);
	if (dret)
		return dret;
	/*
	 * ft_split_compressed_insert wires top_flag's back-pointer into the
	 * live parent before publishing the cluster's forward slot, so the
	 * caller does not need to set the parent here.
	 *
	 * One-commit (parked): the +1 follows the commit at insert_done.
	 */
	if (ic && ic->slot)
		ic->count_from = d->pnf;
	else
		ft_propagate_external_count_parent(ft, d->pnf, 1);
	return 0;
}

/*
 * ft_insert_compressed_key_shorter: key ends before the compressed
 * path.  Split the compressed node into prefix -> junction -> suffix,
 * then attach the new external node at the junction.
 *
 * Returns 0 on success, -EEXIST if duplicate detected (with
 * *unique_node_ret set), or negative errno on failure.
 */
static
int ft_insert_compressed_key_shorter(struct cds_ft *ft,
		struct ft_descent *d,
		unsigned int remaining,
		struct cds_ft_node *node,
		struct cds_ft_node **unique_node_ret,
		struct ft_insert_commit *ic)
{
	struct cds_ft_inode_flag *top_flag, *jct_flag;
	struct cds_ft_metadata *jct_meta;
	bool fresh_head;
	int sret;

	sret = ft_split_compressed_key_shorter(ft,
		d->nf, d->nfp, remaining, &top_flag, &jct_flag, d->depth);
	if (sret)
		return sret;
	jct_meta = cds_ft_item_to_metadata(ft_node_ptr(jct_flag));
	assert(!ft_node_compressed(jct_flag));
	/*
	 * Fresh head iff the junction carries no external_nodes (a non-empty
	 * set was transferred from the old compressed node for remaining == 0:
	 * the key already exists).  Attach a fresh head to the junction NOW,
	 * while the whole cluster is still invisible, so the parked one-commit
	 * publish below makes the structural attach and the ordered-list
	 * splice atomic; duplicates publish directly (no splice).
	 */
	fresh_head = (jct_meta->external_nodes == NULL);
	if (fresh_head) {
		ft_external_head_set_parent(ft, node, jct_flag);
		node->next = NULL;
		/* Cluster-internal store: the junction is unpublished. */
		jct_meta->external_nodes = node;
		sret = ft_insert_commit_arm(ft, ic);
		if (sret) {
			/*
			 * Arm failed: roll the head attach back (the cluster
			 * is still invisible) and publish the split WITHOUT
			 * the new key -- a key-neutral restructure of the same
			 * content -- then surface the failure (the caller's
			 * insert_done resets node->prev for a retry).
			 */
			jct_meta->external_nodes = NULL;
			node->next = NULL;
			ft_publish_to_parent(ft, d->pnf, d->nfp, top_flag);
			free_compressed_node(ft,
				ft_compressed_node_ptr(d->nf));
			return sret;
		}
	}
	/*
	 * ft_split_compressed_key_shorter wires top_flag's back-pointer into
	 * the live parent before its deferred back-channel re-parent of
	 * cn->child, so the caller does not need to set the parent here.
	 */
	ft_insert_publish_or_park(ft, d->pnf, d->nfp, top_flag,
		fresh_head ? ic : NULL);
	if (!fresh_head) {
		if (unique_node_ret) {
			*unique_node_ret = jct_meta->external_nodes;
			free_compressed_node(ft,
				ft_compressed_node_ptr(d->nf));
			return -EEXIST;
		}
		{
			/*
			 * Junction already has external_nodes (transferred
			 * from old compressed node for remaining == 0).
			 * Chain new node as duplicate; no key count change.
			 */
			struct cds_ft_node *last = jct_meta->external_nodes;

			while (ft_node_next(last))
				last = ft_node_next(last);
			ft_chain_node(last, node);
		}
		free_compressed_node(ft, ft_compressed_node_ptr(d->nf));
		return 0;
	}
	/* One-commit (parked): the +1 follows the commit at insert_done. */
	if (ic && ic->slot) {
		ic->count_from = jct_flag;
		ic->free_old_cn = ft_compressed_node_ptr(d->nf);
	} else {
		ft_propagate_external_count_parent(ft, jct_flag, 1);
		free_compressed_node(ft, ft_compressed_node_ptr(d->nf));
	}
	return 0;
}

/*
 * Handle a compressed node during insert descent.
 *
 * Full match + internal/compressed child: traverse through.
 * Full match + external child at end of key: break for duplicate handling.
 * Full match + external child, key continues: build branch inline.
 * Key diverges: split via ft_split_compressed_insert.
 * Key shorter: split via ft_split_compressed_key_shorter.
 *
 * Returns CONTINUE, BREAK, or END (with ret set via *ret_p).
 * On END, the caller should goto insert_done.
 * On error, returns END with *ret_p < 0.
 */
static
enum ft_descent_action ft_insert_compressed(struct cds_ft *ft,
		struct ft_descent *d, const uint8_t **iter_key_p,
		const uint8_t *key, size_t key_len,
		unsigned int key_depth,
		struct cds_ft_node *node,
		struct cds_ft_node **unique_node_ret,
		struct cds_ft_inode_flag **snapshot,
		unsigned int *snapshot_depth,
		int *nr_snapshot_p,
		int *ret_p,
		struct ft_insert_commit *ic)
{
	struct cds_ft_compressed_node *cn = ft_compressed_node_ptr(d->nf);
	unsigned int remaining = key_depth - 1 - d->depth;
	unsigned int cmp = cn->len < remaining ? cn->len : remaining;
	unsigned int j;

	j = ft_match_compressed_key(*iter_key_p, cn, cmp);
	if (j == cmp && cn->len <= remaining) {
		/* Full match: traverse through if child is internal,
		 * compressed, or skip-compressed (which encodes another
		 * compressed node deeper in the chain). */
		if (cn->child &&
		    (ft_node_skip_compressed(cn->child) ||
		     ft_node_internal(cn->child) ||
		     ft_node_compressed(cn->child))) {
			ft_snapshot_push(snapshot, snapshot_depth,
				*nr_snapshot_p, d->nf, d->depth);
			ft_descent_traverse_compressed(d, cn, iter_key_p);
			return FT_DESCENT_CONTINUE;
		}
		if (!cn->child) {
			struct cds_ft_metadata *cn_meta =
				cds_ft_item_to_metadata((struct cds_ft_inode *) cn);
			fprintf(stderr, "BUG: cn->child NULL, cn=%p cn->len=%u depth=%u external_nodes=%p nr_child=%u\n",
				cn, cn->len, d->depth, cn_meta->external_nodes, (unsigned)cn_meta->nr_child);
			abort();
		}
		if (cn->len == remaining) {
			/* Key ends at external child: duplicate. */
			ft_snapshot_push(snapshot, snapshot_depth,
				*nr_snapshot_p, d->nf, d->depth);
			ft_descent_traverse_compressed(d, cn, iter_key_p);
			return FT_DESCENT_BREAK;
		}
		/* Key continues past external child: build branch. */
		*ret_p = ft_insert_compressed_past_child(ft, d, key,
			key_len, cn, node, ic);
		return FT_DESCENT_END;
	}
	if (j < cmp) {
		/* Key diverges: split at position j. */
		*ret_p = ft_insert_compressed_diverge(ft, d,
			*iter_key_p, remaining, j, node, ic);
		return FT_DESCENT_END;
	}
	/* Key shorter: split into prefix -> junction -> suffix. */
	*ret_p = ft_insert_compressed_key_shorter(ft, d, remaining,
		node, unique_node_ret, ic);
	return FT_DESCENT_END;
}

static
int _cds_ft_insert(struct cds_ft *ft,
		const uint8_t *_key, size_t _key_len,
		struct cds_ft_node *node,
		struct cds_ft_node **unique_node_ret)
{
	unsigned int key_depth;
	struct ft_descent d;
	size_t key_len = ft_key_len(ft, _key_len);
	const struct cds_ft_key_map *km = &ft->group->key_map;
	uint8_t ordinal_buf[FT_MAX_KEY_LEN];
	const uint8_t *key;
	const uint8_t *iter_key;
	struct cds_ft_inode_flag *snapshot[FT_MAX_DEPTH];
	unsigned int snapshot_depth[FT_MAX_DEPTH]; /* parallel depth tracking */
	int nr_snapshot = 0;
	int ret;
	struct ft_ord_cell *precell;
	struct ft_insert_commit ic = { NULL, NULL, NULL, NULL, NULL, NULL, false, false };

	if (!valid_external_node(node) || !valid_key_len(ft, key_len))
		return -EINVAL;
	if (caa_likely(km->identity)) {
		key = _key;
	} else {
		ft_key_to_ordinals(ordinal_buf, _key, key_len, km);
		key = ordinal_buf;
	}
	iter_key = key;
	/* Expect zeroed prev/next pointers. This catches some double-insert misuses. */
	if (node->prev || ft_node_next(node))
		return -EINVAL;

	/*
	 * Ordered-list trie: pre-wire @node's ordinal cell before any structural
	 * mutation, so the only failure-prone allocation happens up front (a
	 * clean -ENOMEM, nothing to roll back) and every fresh-head wiring site
	 * downstream just records the flagged parent into the cell (node->prev
	 * already carries it).  If @node ends up a duplicate (chained, not a
	 * head) or the insert fails, the unused @precell is freed at insert_done
	 * (a chained @node has its prev repointed at the predecessor, losing the
	 * cell from node->prev, so the handle is kept here).
	 *
	 * List off: no cell -- @node behaves like a non-cell build (its prev is
	 * wired to the flagged parent directly by the fresh-head sites), saving
	 * the per-key cell.  @precell stays NULL and the cell paths below no-op.
	 */
	precell = NULL;
	if (ft->ordered_list) {
		void *cell = ft_ord_cell_alloc(ft, node, NULL);

		if (!cell)
			return -ENOMEM;
		node->prev = cell;
		precell = ft_ord_cell_ptr(cell);
		/*
		 * Head's last edge byte for the up-walk key rebuild: the cell is
		 * the head's metadata record and @key is ordinal here, so
		 * key[key_len - 1] is the byte the head hangs under (ignored by the
		 * up-walk when the head's parent is a compressed node, whose
		 * key_bytes already span the head's position).
		 */
		if (key_len)
			cds_ft_item_to_metadata(precell)->incoming_byte =
				(uint8_t) key[key_len - 1];
	}

	key_depth = key_len + 1;

	dbg_printf("cds_ft_insert attempt: node %p\n", node);
	ft_descent_init(&d, ft);

	for (; d.depth < key_depth - 1; ) {
		uint8_t key_value;

		if (!d.nf)
			break;
		/*
		 * Resolve skip-compressed pointer.  Convert to the
		 * underlying compressed flag so the compressed handler
		 * below processes it correctly.
		 */
		d.nf = ft_resolve_skip_compressed(ft, d.nf);
		/* Found external node. */
		if (ft_node_external(d.nf))
			break;
		/* Decompress compressed node before continuing descent. */
		/*
		 * Compressed node: compare remaining key bytes with
		 * the compressed path.  If they match, traverse
		 * through to the child.  If they diverge, decompress
		 * at this point and restart.
		 */
		if (ft_node_compressed(d.nf)) {
			enum ft_descent_action act;

			act = ft_insert_compressed(ft, &d, &iter_key,
				key, key_len, key_depth, node,
				unique_node_ret, snapshot, snapshot_depth,
				&nr_snapshot, &ret, &ic);
			if (act == FT_DESCENT_END)
				goto insert_done;
			if (act == FT_DESCENT_BREAK)
				break;
			continue;
		}
		dbg_printf("cds_ft_insert iter ppnf %p pnf %p nfp %p nf %p\n",
				d.ppnf, d.pnf, d.nfp, d.nf);
		ft_snapshot_push(snapshot, snapshot_depth,
			nr_snapshot, d.nf, d.depth);
		key_value = *(iter_key++);
		ft_descent_step(ft, &d, key_value);
	}

	/*
	 * Resolve any skip-compressed pointer left in d.nf by the descent
	 * loop's final step (e.g., ft_descent_traverse_compressed sets d.nf
	 * to cn->child raw, which may be skip-compressed).  The loop body's
	 * resolve at the top of each iteration only fires when the loop
	 * iterates again; a traverse that pushes d.depth to key_depth - 1
	 * exits the loop without re-entering.
	 */
	d.nf = ft_resolve_skip_compressed(ft, d.nf);

	if (d.depth == key_depth - 1) {
		/* Found either an internal, external node or NULL at end of key. */
		if (!d.nf) {
			dbg_printf("cds_ft_insert NULL ppnf %p pnf %p nfp %p nf %p\n",
					d.ppnf, d.pnf, d.nfp, d.nf);

			ret = ft_attach_node(ft, d.pnfp, d.pnf,
					d.nfp, d.nf, key, key_len, d.depth, node,
					NULL, &ic);
			if (ret == 0) {
				/*
				 * One-commit insert (ic.slot parked): the key is
				 * not reachable until insert_done's commit, so
				 * the count propagation moves there (an early +1
				 * would overcount -- the inverse of the nr_keys
				 * undercount discipline).
				 */
				if (!ic.slot)
					ft_propagate_external_count_parent(ft,
						*d.pnfp, 1);
				if (d.depth >= 2)
					FT_TP(tree_edge_set, (const void *) ft,
						(const void *) d.ppnf,
						(unsigned int) (d.depth - 2),
						(uint8_t) key[d.depth - 2],
						(const void *) *d.pnfp);
			}

		} else if (ft_node_compressed(d.nf)) {
			/*
			 * Key ends at a compressed node's depth.
			 * Split: internal(external_nodes) + compressed(len-1).
			 */
			ret = ft_insert_compressed_key_shorter(ft, &d, 0,
				node, unique_node_ret, &ic);
		} else if (!ft_node_external(d.nf)) {
			struct cds_ft_node *external_nodes;
			struct cds_ft_metadata *metadata;

			assert(!ft_node_compressed(d.nf));
			metadata = cds_ft_item_to_metadata(ft_node_ptr(d.nf));
			external_nodes = metadata->external_nodes;
			if (external_nodes) {
				struct cds_ft_node *iter_node, *last_node = NULL;

				if (unique_node_ret) {
					*unique_node_ret = external_nodes;
					ret = -EEXIST;
					goto insert_done;
				}
				/* Find last duplicate */
				iter_node = external_nodes;
				cds_ft_for_each_duplicate(iter_node)
					last_node = iter_node;

				dbg_printf("cds_ft_insert duplicate internal ppnf %p pnf %p nfp %p nf %p\n",
						d.ppnf, d.pnf, d.nfp, d.nf);

				/* Adding duplicate at existing key: no key count change. */
				ft_chain_node(last_node, node);
				ret = 0;
			} else {
				/* New key at this internal node. */
				ft_external_head_set_parent(ft, node, d.nf);
				node->next = NULL;
				if (ft->ordered_list) {
					/*
					 * Readers do not resolve flip proxies
					 * on external_nodes loads, so this
					 * shape cannot park a one-commit
					 * proxy.  B-lite instead: locate the
					 * splice position and pre-fill the
					 * cell's links BEFORE the head becomes
					 * reachable -- a reader landing on the
					 * fresh head always sees valid links
					 * -- and flip the neighbour edges
					 * right after the store.
					 */
					struct ft_ord_cell *pred, *succ;

					ft_ord_cell_prefill_by_key(ft, _key,
						_key_len, precell, &pred, &succ);
					rcu_assign_pointer(
						metadata->external_nodes, node);
					ft_ord_cell_splice_at(ft, precell,
						pred, succ);
					ic.spliced = true;
				} else {
					rcu_assign_pointer(
						metadata->external_nodes, node);
				}
				ret = 0;
				ft_propagate_external_count_parent(ft, d.nf, 1);
			}
		} else {
			struct cds_ft_node *iter_node, *last_node = NULL;

			if (unique_node_ret) {
				*unique_node_ret = (struct cds_ft_node *) ft_node_ptr(d.nf);
				ret = -EEXIST;
				goto insert_done;
			}
			/* Find last duplicate */
			iter_node = (struct cds_ft_node *) ft_node_ptr(d.nf);
			cds_ft_for_each_duplicate(iter_node)
				last_node = iter_node;

			dbg_printf("cds_ft_insert duplicate external ppnf %p pnf %p nfp %p nf %p\n",
					d.ppnf, d.pnf, d.nfp, d.nf);

			/* Adding duplicate at existing key: no key count change. */
			ft_chain_node(last_node, node);
			ret = 0;
		}
	} else {
		/* Found NULL node or external node before end of key. */

		/*
		 * If the last node encountered during traversal is an external node,
		 * transform this external node into an internal node with associated
		 * external node, attach a new cluster as child of this internal node, and
		 * populate this new internal node into the trie to replace the prior
		 * external node.
		 * It's the same for NULL node, only that there is no need to chain any
		 * external node.
		 */

		dbg_printf("cds_ft_insert NULL or external ppnf %p pnf %p nfp %p nf %p\n",
				d.ppnf, d.pnf, d.nfp, d.nf);

		ret = ft_attach_node(ft, d.pnfp, d.pnf,
				d.nfp, d.nf, key, key_len, d.depth, node,
				(struct cds_ft_node *) ft_node_ptr(d.nf), &ic);
		if (ret == 0) {
			/* One-commit: count propagation deferred (see above). */
			if (!ic.slot)
				ft_propagate_external_count_parent(ft, *d.pnfp, 1);
			if (d.depth >= 2)
				FT_TP(tree_edge_set, (const void *) ft,
					(const void *) d.ppnf,
					(unsigned int) (d.depth - 2),
					(uint8_t) key[d.depth - 2],
					(const void *) *d.pnfp);
		}
	}

insert_done:
	/*
	 * @node became a fresh head iff node->prev is still its (cell) carrier
	 * -- i.e. not external.  A duplicate append (ft_chain_node repointed
	 * node->prev at the predecessor) or a failed insert leaves @precell
	 * orphaned: free it, and on failure restore node->prev to its zeroed
	 * state so the application may retry, then splice the kept cell into the
	 * ordered list.  List off: no cell was allocated -- @node->prev is the
	 * flagged parent (fresh head) or the predecessor (dup), so there is
	 * nothing to free or splice; a FAILED insert may still have wired
	 * node->prev early (the build paths set the raw parent before their
	 * fallible publish, e.g. ft_try_compress_chain -> ft_set_parent, the
	 * split branch builders), and the failure unwind frees that cluster:
	 * reset it so the dangling pointer cannot leak into a retry -- the
	 * zeroed-prev check at entry would otherwise reject the node with
	 * -EINVAL forever.
	 */
	if (!ft->ordered_list) {
		if (ret != 0)
			node->prev = NULL;
	} else {
		if (ret != 0) {
			node->prev = NULL;
			ft_ord_cell_free_unpublished(ft, precell);
			if (ic.batch)
				ft_flip_batch_free_unpublished(ic.batch);
		} else if (ft_node_external((struct cds_ft_inode_flag *) node->prev)) {
			ft_ord_cell_free_unpublished(ft, precell);
			if (ic.batch)
				ft_flip_batch_free_unpublished(ic.batch);
		} else if (ic.slot) {
			/*
			 * One-commit: the structural slot is parked as a flip
			 * proxy (the key is still invisible).  Splice position
			 * + cell links first, then ONE commit flips the
			 * structural slot AND the ordered-list edges -- a
			 * reader never sees the head without its cell in the
			 * list.  Count propagation follows the commit (the key
			 * only now counts), from the shape's recorded base.
			 */
			ft_insert_one_commit(ft, _key, _key_len, precell, &ic);
			ft_propagate_external_count_parent(ft,
				ic.count_from ? ic.count_from : *d.pnfp, 1);
		} else if (ic.spliced) {
			/* B-lite shape: spliced at the attach site. */
		} else {
			/*
			 * @node became a fresh head through a shape that
			 * publishes without a parkable slot (insert_replace's
			 * paths): post-publish splice.
			 */
			ft_ord_cell_splice(ft, _key, _key_len, precell);
			if (ic.batch)
				ft_flip_batch_free_unpublished(ic.batch);
		}
	}
	if (ret == 0) {
		if (key_len > uatomic_load(&ft->max_used_key_len, CMM_RELAXED))
			uatomic_store(&ft->max_used_key_len, key_len, CMM_RELAXED);
	}

	return ret;
}

enum cds_ft_status cds_ft_insert(struct cds_ft *ft,
		const uint8_t *key, size_t key_len,
		struct cds_ft_node *node)
{
	int ret;

	CDS_FT_SCOPED_WRITER(ft);
	FT_TP_KEY(insert_enter, ft, key, key_len);
	ret = _cds_ft_insert(ft, key, key_len, node, NULL);

	if (ret == 0) {
		FT_TP(insert_exit, (int) CDS_FT_STATUS_OK);
		return CDS_FT_STATUS_OK;
	}
	if (ret == -EINVAL) {
		FT_TP(insert_exit, (int) CDS_FT_STATUS_INVALID_ARGUMENT_ERROR);
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}
	FT_TP(insert_exit, (int) CDS_FT_STATUS_MEMORY_ERROR);
	return CDS_FT_STATUS_MEMORY_ERROR;
}

enum cds_ft_status cds_ft_insert_unique(struct cds_ft *ft,
		const uint8_t *key, size_t key_len,
		struct cds_ft_node *node,
		struct cds_ft_node **result_node)
{
	int ret;
	struct cds_ft_node *ret_node = NULL;

	CDS_FT_SCOPED_WRITER(ft);
	FT_TP_KEY(insert_unique_enter, ft, key, key_len);
	ret = _cds_ft_insert(ft, key, key_len, node, &ret_node);
	if (ret == -EEXIST) {
		*result_node = ret_node;
		FT_TP(insert_unique_exit, (int) CDS_FT_STATUS_DUPLICATE_FOUND);
		return CDS_FT_STATUS_DUPLICATE_FOUND;
	}
	if (ret == -EINVAL) {
		*result_node = NULL;
		FT_TP(insert_unique_exit, (int) CDS_FT_STATUS_INVALID_ARGUMENT_ERROR);
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}
	if (ret) {
		*result_node = NULL;
		FT_TP(insert_unique_exit, (int) CDS_FT_STATUS_MEMORY_ERROR);
		return CDS_FT_STATUS_MEMORY_ERROR;
	}
	*result_node = node;
	FT_TP(insert_unique_exit, (int) CDS_FT_STATUS_OK);
	return CDS_FT_STATUS_OK;
}

/*
 * Insert a node, replacing the entire existing duplicate chain at the
 * same key if one exists.
 *
 * On success, *@old_node_ret is set to the head of the replaced chain
 * (or NULL if no prior node existed). The caller must wait for a grace
 * period before reclaiming the old chain.
 *
 * Returns 0 on success, -EINVAL on bad arguments, or a negative errno
 * on memory allocation failure.
 */
static
int _cds_ft_insert_replace(struct cds_ft *ft,
		const uint8_t *_key, size_t _key_len,
		struct cds_ft_node *node,
		struct cds_ft_node **old_node_ret)
{
	unsigned int key_depth;
	struct ft_descent d;
	const uint8_t *iter_key;
	size_t key_len = ft_key_len(ft, _key_len);
	const struct cds_ft_key_map *km = &ft->group->key_map;
	uint8_t ordinal_buf[FT_MAX_KEY_LEN];
	const uint8_t *key;
	struct cds_ft_inode_flag *snapshot[FT_MAX_DEPTH];
	unsigned int snapshot_depth[FT_MAX_DEPTH];
	int nr_snapshot = 0;
	int ret;
	struct ft_ord_cell *precell;

	*old_node_ret = NULL;

	if (!valid_external_node(node) || !valid_key_len(ft, key_len))
		return -EINVAL;
	if (caa_likely(km->identity)) {
		key = _key;
	} else {
		ft_key_to_ordinals(ordinal_buf, _key, key_len, km);
		key = ordinal_buf;
	}
	/* Expect zeroed prev/next pointers. */
	if (node->prev || ft_node_next(node))
		return -EINVAL;

	/* Ordered-list trie: pre-wire @node's cell (see _cds_ft_insert).  A replace
	 * always lands @node as the sole head on success, so the cell is kept
	 * unless the insert fails (or the key_shorter path finds the key and
	 * leaves @node uninstalled -- both freed below).  List off: no cell. */
	precell = NULL;
	if (ft->ordered_list) {
		void *cell = ft_ord_cell_alloc(ft, node, NULL);

		if (!cell)
			return -ENOMEM;
		node->prev = cell;
		precell = ft_ord_cell_ptr(cell);
		/*
		 * Head's last edge byte for the up-walk key rebuild: the cell is
		 * the head's metadata record and @key is ordinal here, so
		 * key[key_len - 1] is the byte the head hangs under (ignored by the
		 * up-walk when the head's parent is a compressed node, whose
		 * key_bytes already span the head's position).
		 */
		if (key_len)
			cds_ft_item_to_metadata(precell)->incoming_byte =
				(uint8_t) key[key_len - 1];
	}

	key_depth = key_len + 1;

	dbg_printf("_cds_ft_insert_replace attempt: node %p\n", node);
	iter_key = key;
	ft_descent_init(&d, ft);

	for (; d.depth < key_depth - 1; ) {
		uint8_t key_value;

		if (!d.nf)
			break;
		/* Resolve skip-compressed pointer. */
		d.nf = ft_resolve_skip_compressed(ft, d.nf);
		if (ft_node_external(d.nf))
			break;
		if (ft_node_compressed(d.nf)) {
			enum ft_descent_action act;

			act = ft_insert_compressed(ft, &d, &iter_key,
				key, key_len, key_depth, node,
				NULL, snapshot, snapshot_depth,
				&nr_snapshot, &ret, NULL);
			if (act == FT_DESCENT_END)
				goto insert_replace_done;
			if (act == FT_DESCENT_BREAK)
				break;
			continue;
		}
		dbg_printf("_cds_ft_insert_replace iter ppnf %p pnf %p nfp %p nf %p\n",
				d.ppnf, d.pnf, d.nfp, d.nf);
		ft_snapshot_push(snapshot, snapshot_depth,
			nr_snapshot, d.nf, d.depth);
		key_value = *(iter_key++);
		ft_descent_step(ft, &d, key_value);
	}

	/*
	 * Resolve any skip-compressed pointer left in d.nf by a final
	 * traverse that exited the loop without re-entering the loop's
	 * resolve step.
	 */
	d.nf = ft_resolve_skip_compressed(ft, d.nf);

	if (d.depth == key_depth - 1) {
		/* Found either an internal, external node or NULL at end of key. */
		if (!d.nf) {
			/* No existing node. Regular attach. */
			dbg_printf("_cds_ft_insert_replace NULL at end of key\n");

			ret = ft_attach_node(ft, d.pnfp, d.pnf,
					d.nfp, d.nf, key, key_len, d.depth, node,
					NULL, NULL);
			if (ret == 0) {
				ft_propagate_external_count_parent(ft, *d.pnfp, 1);
				if (d.depth >= 2)
					FT_TP(tree_edge_set, (const void *) ft,
						(const void *) d.ppnf,
						(unsigned int) (d.depth - 2),
						(uint8_t) key[d.depth - 2],
						(const void *) *d.pnfp);
			}
		} else if (ft_node_compressed(d.nf)) {
			/*
			 * Key ends at a compressed node's depth.
			 * Split: internal(external_nodes) + compressed(len-1).
			 */
			ret = ft_insert_compressed_key_shorter(ft, &d, 0,
				node, old_node_ret, NULL);
			if (ret == -EEXIST) {
				ret = 0;	/* Replace handled by key_shorter. */
				/*
				 * key_shorter found the key already present and
				 * left @node uninstalled (*old_node_ret names the
				 * existing chain, which keeps its own cell).  Drop
				 * the pre-wired cell from node->prev; insert_replace_done
				 * frees the now-orphaned @precell.
				 */
				node->prev = NULL;
			}
		} else if (!ft_node_external(d.nf)) {
			struct cds_ft_node *external_nodes;
			struct cds_ft_metadata *metadata;

			assert(!ft_node_compressed(d.nf));
			metadata = cds_ft_item_to_metadata(ft_node_ptr(d.nf));
			external_nodes = metadata->external_nodes;
			if (external_nodes) {
				dbg_printf("_cds_ft_insert_replace: replacing internal metadata chain %p\n",
						external_nodes);
				/* Replace existing chain: key count unchanged. */
				*old_node_ret = external_nodes;
				ft_external_head_set_parent(ft, node, d.nf);
				node->next = NULL;
				rcu_assign_pointer(metadata->external_nodes, node);
				/*
				 * Ordered list on: the replaced head's cell leaves the
				 * trie; @node's pre-wired cell takes its list slot (O(1)
				 * swap, no re-descent), then the old cell is freed.  List
				 * off: no cells, nothing to swap or free.
				 */
				if (ft->ordered_list) {
					struct ft_ord_cell *old_cell =
						ft_ord_cell_ptr(external_nodes->prev);

					ft_ord_cell_swap(ft, old_cell, precell);
					ft_ord_cell_free(ft, old_cell);
				}
			} else {
				/* No external nodes yet. New key. */
				ft_external_head_set_parent(ft, node, d.nf);
				node->next = NULL;
				rcu_assign_pointer(metadata->external_nodes, node);
				ft_propagate_external_count_parent(ft, d.nf, 1);
			}
			ret = 0;
		} else {
			dbg_printf("_cds_ft_insert_replace: replacing external chain %p\n",
					ft_node_ptr(d.nf));
			/* External node at end of key. Replace chain: key count unchanged. */
			*old_node_ret = (struct cds_ft_node *) ft_node_ptr(d.nf);
			ft_external_head_set_parent(ft, node, d.pnf);
			node->next = NULL;
			ft_publish_to_parent(ft, d.pnf, d.nfp,
				(struct cds_ft_inode_flag *) node);
			/* Ordered list on: replaced head's cell leaves; @node's cell
			 * takes its slot, then the old cell is freed.  List off: none. */
			if (ft->ordered_list) {
				struct ft_ord_cell *old_cell =
					ft_ord_cell_ptr((*old_node_ret)->prev);

				ft_ord_cell_swap(ft, old_cell, precell);
				ft_ord_cell_free(ft, old_cell);
			}
			ret = 0;
		}
	} else {
		/*
		 * Found NULL node or external node before end of key.
		 * Attach a new branch, displacing any shorter-key
		 * external node into the new branch's metadata.
		 */
		dbg_printf("_cds_ft_insert_replace: attach before end of key\n");

		ret = ft_attach_node(ft, d.pnfp, d.pnf,
				d.nfp, d.nf, key, key_len, d.depth, node,
				(struct cds_ft_node *) ft_node_ptr(d.nf), NULL);
		if (ret == 0) {
			ft_propagate_external_count_parent(ft, *d.pnfp, 1);
			if (d.depth >= 2)
				FT_TP(tree_edge_set, (const void *) ft,
					(const void *) d.ppnf,
					(unsigned int) (d.depth - 2),
					(uint8_t) key[d.depth - 2],
					(const void *) *d.pnfp);
		}
	}

insert_replace_done:
	/*
	 * @node became the installed head iff node->prev still carries its
	 * pre-wired cell (not external).  A duplicate append (descent through a
	 * compressed node chained @node), the uninstalled key_shorter -EEXIST
	 * path (prev NULLed above), or a failed insert leaves @precell orphaned
	 * -- free it, and on failure restore node->prev to its zeroed state.
	 * List off: no cell, nothing to free or splice (the swap sites above are
	 * gated too) -- but a FAILED insert may have wired node->prev early in
	 * a build path whose cluster the unwind then freed: reset it so the
	 * retry does not hit the zeroed-prev entry check (see _cds_ft_insert).
	 */
	if (!ft->ordered_list) {
		if (ret != 0)
			node->prev = NULL;
	} else {
		if (ret != 0) {
			node->prev = NULL;
			ft_ord_cell_free_unpublished(ft, precell);
		} else if (ft_node_external((struct cds_ft_inode_flag *) node->prev)) {
			ft_ord_cell_free_unpublished(ft, precell);
		} else if (*old_node_ret == NULL) {
			/*
			 * Fresh head (no chain replaced): splice its kept cell.  A replace
			 * (*old_node_ret set) already swapped @precell into the replaced
			 * head's list slot at the replace site, so it must NOT splice again.
			 */
			ft_ord_cell_splice(ft, _key, _key_len, precell);
		}
	}
	if (ret == 0) {
		if (key_len > uatomic_load(&ft->max_used_key_len, CMM_RELAXED))
			uatomic_store(&ft->max_used_key_len, key_len, CMM_RELAXED);
	}

	return ret;
}

enum cds_ft_status cds_ft_insert_replace(struct cds_ft *ft,
		const uint8_t *key, size_t key_len,
		struct cds_ft_node *node,
		struct cds_ft_node **result_node)
{
	struct cds_ft_node *old_node = NULL;
	int ret;

	CDS_FT_SCOPED_WRITER(ft);
	FT_TP_KEY(insert_replace_enter, ft, key, key_len);
	ret = _cds_ft_insert_replace(ft, key, key_len, node, &old_node);
	if (ret == -EINVAL) {
		*result_node = NULL;
		FT_TP(insert_replace_exit, (int) CDS_FT_STATUS_INVALID_ARGUMENT_ERROR);
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}
	if (ret) {
		*result_node = NULL;
		FT_TP(insert_replace_exit, (int) CDS_FT_STATUS_MEMORY_ERROR);
		return CDS_FT_STATUS_MEMORY_ERROR;
	}
	*result_node = old_node;
	if (old_node) {
		FT_TP(insert_replace_exit, (int) CDS_FT_STATUS_DUPLICATE_FOUND);
		return CDS_FT_STATUS_DUPLICATE_FOUND;
	}
	FT_TP(insert_replace_exit, (int) CDS_FT_STATUS_OK);
	return CDS_FT_STATUS_OK;
}

enum cds_ft_status cds_ft_replace(struct cds_ft *ft,
		struct cds_ft_iter *iter,
		struct cds_ft_node *old_node,
		struct cds_ft_node *new_node)
{
	struct cds_ft_inode_flag *holder_flag;
	struct cds_ft_inode_flag **pub_slot;
	struct cds_ft_compressed_node *cn = NULL;
	const uint8_t *iter_key;
	size_t key_len = ft_key_len(ft, ft_iter_resolve_key_len(iter));
	enum cds_ft_status s;

	CDS_FT_SCOPED_WRITER(ft);
	FT_TP_ITER_KEY(replace_enter, iter);

	/*
	 * If the iterator has a valid path, the RCU read-side lock must
	 * be held.
	 */
	if (iter->cache_valid)
		CDS_FT_ASSERT_RCU_READ_LOCKED(ft);
	iter_debug_path_check(iter);

	if (!valid_external_node(old_node) || !valid_external_node(new_node)
			|| !valid_key_len(ft, key_len)) {
		s = CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
		FT_TP(replace_exit, (int) s);
		return s;
	}
	/* Expect zeroed next and prev pointers on new_node. */
	if (ft_node_next(new_node) || new_node->prev) {
		s = CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
		FT_TP(replace_exit, (int) s);
		return s;
	}

	iter_key = ft_iter_read_key(iter);

	dbg_printf("cds_ft_replace: old_node %p new_node %p\n", old_node, new_node);

	/*
	 * No top-down descent.  As in cds_ft_remove, @old_node is
	 * application-owned and -- with the RCU read-side lock held
	 * continuously since it was obtained -- alive; the writer mutex held
	 * here freezes the structure, so @old_node->prev is a settled live
	 * pointer to its holder.  @new_node takes @old_node's exact place in
	 * the duplicate chain, so the key count and trie shape are unchanged:
	 * only the chain link (or head slot) that points at @old_node is
	 * repointed at @new_node.  That slot is derived from the holder -- the
	 * predecessor's next for a non-head duplicate, else the head slot
	 * cds_ft_remove recovers (a compressed holder's &cn->child, an
	 * internal holder's external_nodes, or an internal body slot keyed by
	 * the last key byte).
	 */
	if (ft_node_is_removed(old_node)) {
		/* Already unlinked from the trie. */
		s = CDS_FT_STATUS_NOT_FOUND;
		FT_TP(replace_exit, (int) s);
		return s;
	}
	holder_flag = ft_node_holder(ft, old_node);
	if (!holder_flag) {
		/* Never inserted (a freshly-initialized node). */
		s = CDS_FT_STATUS_NOT_FOUND;
		FT_TP(replace_exit, (int) s);
		return s;
	}

	if (ft_node_external(holder_flag)) {
		/* Non-head duplicate: repoint the predecessor's next. */
		pub_slot = (struct cds_ft_inode_flag **)
			&((struct cds_ft_node *) holder_flag)->next;
	} else if (ft_node_compressed(holder_flag) ||
		   ft_node_skip_compressed(holder_flag)) {
		/* Compressed holder: @old_node is its single external child. */
		cn = ft_node_skip_compressed(holder_flag) ?
			ft_skip_to_compressed(ft, holder_flag) :
			ft_compressed_node_ptr(holder_flag);
		if ((struct cds_ft_node *) ft_node_ptr(cn->child) != old_node) {
			s = CDS_FT_STATUS_NOT_FOUND;
			FT_TP(replace_exit, (int) s);
			return s;
		}
		pub_slot = &cn->child;
	} else if (ft_node_external_nodes(holder_flag) ==
			(struct cds_ft_node *) old_node) {
		/* Internal holder: @old_node heads its external_nodes chain. */
		pub_slot = (struct cds_ft_inode_flag **)
			&cds_ft_item_to_metadata(ft_node_ptr(holder_flag))->external_nodes;
	} else {
		/* Internal holder: @old_node is a body child (leaf key). */
		struct cds_ft_inode_flag *child;

		child = ft_node_get_nth_skip(holder_flag, &pub_slot,
			iter_key[key_len - 1], FT_PF_NONE);
		if (!child ||
		    (struct cds_ft_node *) ft_node_ptr(child) != old_node) {
			s = CDS_FT_STATUS_NOT_FOUND;
			FT_TP(replace_exit, (int) s);
			return s;
		}
	}

	/*
	 * Splice @new_node into the chain in place of @old_node: it inherits
	 * @old_node's prev and next, the successor (if any) is repointed back
	 * at it, and rcu_assign_pointer publishes it into the slot, ordering
	 * those stores before it becomes reachable.
	 */
	new_node->prev = old_node->prev;
	new_node->next = ft_node_next(old_node);
	if (new_node->next)
		new_node->next->prev = new_node;
	rcu_assign_pointer(*pub_slot, (struct cds_ft_inode_flag *) new_node);

	/*
	 * Cell transfer: @new_node inherited @old_node's prev (its cell, when a
	 * head) via the copy above, so it shares the same cell -- now fully
	 * assembled and published.  Retarget the cell at @new_node so up-walks
	 * and ordered iteration resolve to the live node; the cell's parent and
	 * ord-list position are preserved (no list surgery, no free).  A
	 * non-head duplicate replace copied an external prev -- nothing to do.
	 * List off: @new_node->prev is the flagged parent directly (inherited),
	 * no cell to retarget.
	 */
	if (ft->ordered_list &&
	    !ft_node_external((struct cds_ft_inode_flag *) new_node->prev))
		rcu_assign_pointer(ft_ord_cell_ptr(new_node->prev)->node, new_node);

#ifdef FEATURE_FT_SKIP_COMPRESSED
	/*
	 * A compressed-head replace changed cn->child; re-encode the
	 * grandparent's skip-compressed slot to the new child so a skip
	 * descent or ft_skip_reanchor up-walk does not follow the stale
	 * pointer into the about-to-be-freed @old_node (the dangling-skip
	 * UAF that cds_ft_remove guards against on its compressed-head
	 * unchain).  No-ops for a plain (non-skip) compressed slot.
	 */
	if (cn)
		ft_update_skip_pointer(ft_get_parent_slot(
			cds_ft_item_to_metadata((struct cds_ft_inode *) cn), ft), cn);
#endif

	/*
	 * @old_node has left the trie (replaced by @new_node): tombstone it.
	 * Its next pointer is preserved so a concurrent reader positioned on
	 * @old_node still follows the chain.
	 */
	ft_node_mark_removed(old_node);


	/*
	 * The trie structure is unchanged (no recompaction), so the iterator
	 * path remains valid in cached mode.
	 */
	iter_auto_invalidate_cache(iter);
	s = CDS_FT_STATUS_OK;
	FT_TP(replace_exit, (int) s);
	return s;
}

/*
 * Called with RCU read lock held.
 *
 * There are a few cases to cover for delete:
 *
 * 1) The node belongs to a list of external nodes duplicates with two
 *    or more items. Remove the node by unlinking it from its list.
 * 2) There is only one external node within this node's list.
 *    2.1) The node is within an external nodes list for which the list
 *         head is an standalone external nodes pointer. The external
 *         nodes list for this key should be removed. Removing an
 *         external nodes list should prune the entire branch leading to
 *         that list so no lookup observe empty internal nodes. This is
 *         done by ft_detach_node(). Internal nodes are considered empty
 *         if they have no internal and no external node children, *and*
 *         their associated list of external nodes is empty. When
 *         detaching an internal node which has no children, but has
 *         an associated list of external nodes, it is replaced by a
 *         pointer to the external nodes.
 *    2.2) The node is within an external nodes list which is associated
 *         with an internal node. Unlink the node from its list, leaving
 *         the external nodes list empty.
 */
