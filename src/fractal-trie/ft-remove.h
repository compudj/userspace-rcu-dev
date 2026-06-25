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

/*
 * ft_detach_node: detach a node from the trie and prune empty
 * single-child ancestors above it.
 *
 * Walks upward from the parent of the detached node via
 * metadata->parent pointers.  Prunes single-child ancestors until
 * reaching a node with multiple children, external nodes, or the
 * root.  The pruned branch is replaced by the topmost external
 * nodes found during the walk (or NULL).
 *
 * @detach_node_flag_ptr: slot in parent pointing to the detached node.
 * @detach_parent_flag_ptr: slot in grandparent pointing to the parent.
 * @detach_depth: trie depth of the detached node.
 * @fuse_cell: when a key-disappearing remove has the ordered list on, the
 *   dead head's ord cell to unsplice ATOMICALLY with the structural unlink;
 *   NULL otherwise (move/merge callers, or list off).
 * @pub: armed by the in-place (no-recompaction) leaf delete so the forward
 *   NULL store is deferred and committed in one flip with @fuse_cell's
 *   unsplice (ft_remove_one_commit).  On return, @pub->armed tells the caller
 *   whether the unsplice was fused here (true) or must be done separately
 *   (false: recompaction or compressed-parent shape, still two-commit).  Both
 *   NULL for non-fusing callers.
 */

/*
 * Replace a compressed (or skip-compressed) parent in
 * ft_detach_node's structural-change phase.
 *
 * Two sub-cases:
 *
 *   - The detached child carried external_nodes that must be
 *     promoted to the compressed node's child slot
 *     (@topmost_external_nodes != NULL): keep the compressed node
 *     (its path is needed for lookups) and replace cn->child with
 *     the external chain head.  Reset *nr_clear so the
 *     free-intermediate walk does not run later.
 *
 *   - Otherwise: the compressed parent is no longer needed.
 *     Allocate a fresh empty internal, inherit parent + skip slot
 *     metadata, publish it in place of the compressed node, and
 *     free the compressed.  Returns -ENOMEM if the fresh
 *     allocation failed.
 */
static
int ft_detach_node_replace_compressed_parent(struct cds_ft *ft,
		struct cds_ft_inode_flag *iter_node_flag,
		struct cds_ft_inode_flag **detach_parent_flag_ptr,
		struct cds_ft_node *topmost_external_nodes,
		int *nr_clear,
		struct ft_ord_cell *fuse_cell,
		struct ft_remove_pub *pub,
		struct ft_detach_run *run)
{
	if (topmost_external_nodes) {
		/*
		 * Keep the compressed node -- its path is needed for
		 * lookups to reach the correct depth.  Replace
		 * cn->child with the external node.
		 *
		 * Compressed nodes can have an external child
		 * (cn->child pointing to an external node) but must
		 * NOT have metadata->external_nodes set.
		 */
		struct cds_ft_compressed_node *cn;

		if (ft_node_skip_compressed(iter_node_flag))
			cn = ft_skip_to_compressed(ft, iter_node_flag);
		else
			cn = ft_compressed_node_ptr(iter_node_flag);
		/*
		 * Wire the external's prev to cn BEFORE publishing cn->child:
		 * after the publish, a skip pointer at cn's grandparent slot
		 * resolves through cn->child = topmost_external_nodes, and
		 * ft_skip_to_compressed walks topmost->prev to recover cn.
		 * Setting prev after the publish leaves a window where prev
		 * still points at the about-to-be-detached old holder, so
		 * skip-recovery returns the wrong compressed node.
		 */
		ft_set_parent(ft,
			(struct cds_ft_inode_flag *) topmost_external_nodes,
			ft_compressed_node_flag(cn), &cn->child);
		/*
		 * External-promote publish into cn->child (the moment the
		 * removed leaf key disappears for an exact reader, which
		 * descends through cn->child).  When fusion is requested, record
		 * the publish's reader-visible edges (cn->child, plus cn's
		 * grandparent SKIP_X dual) and commit them in ONE flip with the
		 * dead head cell's unsplice -- every cn->child reader resolves a
		 * flip proxy now -- and signal it via pub->armed.
		 */
		if ((fuse_cell || run) && pub && !pub->armed) {
			struct ft_pub_rec rec = { .n = 0 };

			_ft_publish_to_parent(ft, ft_compressed_node_flag(cn),
				&cn->child,
				(struct cds_ft_inode_flag *) topmost_external_nodes,
				&rec);
			ft_remove_commit_rec(ft, &rec, fuse_cell, run);
			pub->armed = true;
		} else {
			struct ft_pub_rec rec = { .n = 0 };

			/*
			 * Non-fused external-promote (no cell to unsplice -- list
			 * off, or fusion not requested / already armed): still route
			 * the publish through the op flip-txn so the forward slot AND
			 * a compressed grandparent's SKIP_X dual flip atomically (no
			 * torn forward/skip window).  A lone edge reduces to a single
			 * release store in ft_ord_cell_flip, so this stays allocation-
			 * free and infallible for the common plain-parent case.
			 */
			_ft_publish_to_parent(ft, ft_compressed_node_flag(cn),
				&cn->child,
				(struct cds_ft_inode_flag *) topmost_external_nodes,
				&rec);
			ft_remove_commit_rec(ft, &rec, NULL, NULL);
		}
		*nr_clear = 0;
		return 0;
	}
	{
		struct cds_ft_inode *fresh;
		struct cds_ft_metadata *fresh_meta;
		struct cds_ft_metadata *src_meta;

		fresh = alloc_cds_ft_node(ft, &ft_types[0], &fresh_meta);
		if (!fresh)
			return -ENOMEM;
		src_meta = cds_ft_item_to_metadata(
			(struct cds_ft_inode *) ft_compressed_node_ptr(
				iter_node_flag));
		fresh_meta->parent = src_meta->parent;
#ifdef FEATURE_FT_SKIP_COMPRESSED
		fresh_meta->parent_slot_offset = src_meta->parent_slot_offset;
#endif
		{
			struct ft_pub_rec rec = { .n = 0 };

			/*
			 * Compressed -> fresh-internal recompaction publish: route
			 * the forward slot (+ a compressed grandparent's SKIP_X dual)
			 * through the op flip-txn so they flip atomically; lone edge
			 * stays a single release store.
			 */
			_ft_publish_to_parent(ft, src_meta->parent,
				detach_parent_flag_ptr,
				ft_node_flag(fresh, 0), &rec);
			ft_remove_commit_rec(ft, &rec, NULL, NULL);
		}
		free_compressed_node(ft,
			ft_compressed_node_ptr(iter_node_flag));
	}
	return 0;
}

#ifdef FEATURE_FT_SKIP_COMPRESSED
/*
 * ft_canonicalize_chain_compress: collapse a non-root 1-child internal
 * node with no external_nodes into a single compressed node, merging
 * with adjacent compressed parent/child via the 4-case chain-merge:
 *
 *  - parent non-compressed, child non-compressed:
 *      [iter_internal] -> [new_cn(1 byte)]; child preserved.
 *  - parent non-compressed, child compressed:
 *      [iter_internal, child_cn] -> [new_cn(1 + child_cn.len bytes)];
 *      new_cn.child = child_cn.child.
 *  - parent compressed, child non-compressed:
 *      [parent_cn, iter_internal] -> [new_cn(parent_cn.len + 1 bytes)];
 *      new_cn.child = surviving_child.
 *  - parent compressed, child compressed:
 *      [parent_cn, iter_internal, child_cn] ->
 *      [new_cn(parent_cn.len + 1 + child_cn.len bytes)];
 *      new_cn.child = child_cn.child.
 *
 * Preserves the "no two adjacent compresseds" invariant by absorbing
 * adjacent compressed neighbours into the new node.  Bounded by
 * FT_SKIP_LEN_MAX (uint8_t cn->len): when the merged length would
 * exceed the bound, leaves non-canonical residue (subsequent inserts
 * may rebuild canonical form).  Allocation failure: same fallback.
 *
 * Preconditions (caller asserts):
 *   - @iter_meta->nr_child == 1
 *   - @iter_meta->external_nodes == NULL
 *   - @iter_meta->parent != NULL  (non-root)
 *
 * @iter_node_flag: the 1-child internal being collapsed.
 * @iter_meta:      its metadata.
 * @slot_ptr:       parent slot pointing to @iter_node_flag.  Used only
 *                  when the parent is non-compressed; compressed parents
 *                  resolve their own slot via parent_slot_offset.
 */
static
void ft_canonicalize_chain_compress(struct cds_ft *ft,
		struct cds_ft_inode_flag *iter_node_flag,
		struct cds_ft_metadata *iter_meta,
		struct cds_ft_inode_flag **slot_ptr)
{
	uint8_t surviving_byte = 0;
	struct cds_ft_inode_flag *surviving_child;
	bool parent_compressed, child_compressed;
	struct cds_ft_compressed_node *parent_cn, *child_cn;
	struct cds_ft_metadata *parent_cn_meta;
	unsigned int parent_len, child_len, merged_len;
	struct cds_ft_metadata *new_cn_meta;
	struct cds_ft_compressed_node *new_cn;
	struct cds_ft_inode_flag *new_cn_flag;
	struct cds_ft_inode_flag **publish_slot;
	struct cds_ft_inode_flag *publish_parent;

	surviving_child = ft_node_get_minmax(ft, iter_node_flag,
		&surviving_byte, FT_LEFTMOST,
		false /* writer; no validation */);
	if (!surviving_child)
		return;
	parent_compressed = ft_node_compressed(iter_meta->parent);
	child_compressed = ft_node_compressed(surviving_child);
	parent_cn = parent_compressed
		? ft_compressed_node_ptr(iter_meta->parent)
		: NULL;
	parent_cn_meta = parent_cn
		? cds_ft_item_to_metadata((struct cds_ft_inode *) parent_cn)
		: NULL;
	child_cn = child_compressed
		? ft_compressed_node_ptr(surviving_child)
		: NULL;
	parent_len = parent_cn ? parent_cn->len : 0;
	child_len = child_cn ? child_cn->len : 0;
	merged_len = parent_len + 1 + child_len;

	if (merged_len > FT_SKIP_LEN_MAX)
		return;
	new_cn = alloc_compressed_node(ft, merged_len, &new_cn_meta);
	if (!new_cn)
		return;

	/* Compose merged path bytes. */
	if (parent_cn)
		memcpy(new_cn->key_bytes, parent_cn->key_bytes, parent_len);
	new_cn->key_bytes[parent_len] = surviving_byte;
	if (child_cn)
		memcpy(&new_cn->key_bytes[parent_len + 1],
			child_cn->key_bytes, child_len);
	new_cn->len = (uint8_t) merged_len;
	new_cn->child = child_cn ? child_cn->child : surviving_child;
	new_cn_meta->nr_child = 1;
	ft_nr_keys_store(new_cn_meta,
		ft_nr_keys_get(parent_cn_meta
			? parent_cn_meta
			: iter_meta),
		CMM_RELAXED);

	if (parent_cn) {
		/*
		 * Replace parent_cn at its own slot in the grandparent.
		 * Inherit grandparent context from parent_cn.
		 */
		new_cn_meta->parent = parent_cn_meta->parent;
		publish_parent = parent_cn_meta->parent;
		publish_slot = ft_get_parent_slot(parent_cn_meta, ft);
	} else {
		new_cn_meta->parent = iter_meta->parent;
		publish_parent = iter_meta->parent;
		publish_slot = slot_ptr;
	}
	ft_set_parent_slot(new_cn_meta, new_cn_meta->parent, publish_slot);

	new_cn_flag = ft_compressed_node_flag(new_cn);
	{
		struct ft_pub_rec rec = { .n = 0 };
		struct cds_ft_inode_flag *new_cn_pub;

		/*
		 * Chain-compress canonicalization publish: the merged compressed
		 * node replaces the collapsed chain at publish_slot.  Fuse the
		 * LIVE child re-parent (new_cn->child -> new_cn) with the forward
		 * slot (+ a compressed grandparent's SKIP_X dual) into ONE flip,
		 * so a reader never sees new_cn->child re-parented onto new_cn
		 * while the grandparent slot still points at the collapsed chain
		 * (or vice versa).  The back-pointer takes the PLAIN compressed
		 * flag (as the prior bare ft_set_parent did); the forward slot
		 * takes the published (possibly skip-encoded) flag.  Because the
		 * back-edge is deferred, the publish takes new_cn_meta explicitly
		 * (ft_skip_to_compressed would otherwise read the not-yet-stored
		 * back-edge to resolve a SKIP_X forward flag).  A lone edge stays
		 * a single release store.
		 */
		ft_pub_rec_add_back_edge(ft, &rec, new_cn->child, new_cn_flag,
			&new_cn->child);
		new_cn_pub = ft_publish_compressed(ft, new_cn, new_cn_flag);
		_ft_publish_to_parent_meta(ft, publish_parent, publish_slot,
			new_cn_pub, new_cn_meta, &rec);
		ft_remove_commit_rec(ft, &rec, NULL, NULL);
	}

	free_cds_ft_node(ft, ft_node_ptr(iter_node_flag));
	if (parent_cn)
		free_compressed_node(ft, parent_cn);
	if (child_cn)
		free_compressed_node(ft, child_cn);
}
#endif

static
int ft_detach_node(struct cds_ft *ft,
		struct cds_ft_inode_flag **detach_node_flag_ptr,
		struct cds_ft_inode_flag **detach_parent_flag_ptr,
		unsigned int detach_depth,
		bool free_detached_subtree,
		struct ft_ord_cell *fuse_cell,
		struct ft_remove_pub *pub,
		struct ft_detach_run *run)
{
	struct cds_ft_metadata *metadata_stack[FT_MAX_DEPTH];
	struct cds_ft_inode_flag *iter_node_flag;
	struct cds_ft_inode *old_recompacted_node = NULL;
	int ret, nr_metadata = 0, nr_clear = 0, nr_branch = 0;
	uint8_t n = 0;
	struct cds_ft_node *topmost_external_nodes = NULL;
	bool prev_external_nodes_found = false;
	struct cds_ft_inode_flag *cur;
	unsigned int cur_depth;
	/*
	 * Snapshot of the slot value at the detach point.  Set once the
	 * upward walk finishes elevating @detach_node_flag_ptr, before
	 * ft_node_replace_ptr overwrites the slot.  The free-walk below
	 * starts from this value so it covers BOTH the elevated single-
	 * child ancestors and the original detach target (the elevation
	 * walk only crosses nodes with nr_child==1, so the underlying
	 * chain reaches the original detach child).
	 */
	struct cds_ft_inode_flag *elevated_old_child;

	FT_TP(detach_node_enter, (const void *) *detach_node_flag_ptr, detach_depth);

	/*
	 * Check the node being replaced (the child at detach_node_flag_ptr)
	 * for external_nodes.  After the child's last internal child was
	 * removed (triggering this detach), the child may still hold
	 * variable-length key entries that must be preserved by promoting
	 * them to the replacement slot.
	 *
	 * Only for a destroy-style detach (free_detached_subtree): a
	 * move-style detach PRESERVES the target subtree (it becomes another
	 * trie's content), so the target keeps its own external_nodes and
	 * must NOT have them promoted into the source.  This matters once a
	 * move-style caller bootstraps from the target itself (the climb
	 * starts at *detach_node_flag_ptr == the target); the descent-based
	 * callers passed the single-child chain head here, which never
	 * carries external_nodes, so this gate is a no-op for them.
	 */
	if (free_detached_subtree) {
		struct cds_ft_inode_flag *detach_child = *detach_node_flag_ptr;

		/*
		 * Resolve skip-compressed before type checks: a skip
		 * pointer with an external child has low bits == 0,
		 * falsely matching ft_node_external and skipping the
		 * external_nodes preservation entirely.
		 */
		detach_child = ft_resolve_skip_compressed(ft, detach_child);

		if (detach_child && !ft_node_external(detach_child)) {
			struct cds_ft_metadata *child_meta =
				ft_flag_to_metadata(ft, detach_child);
			if (child_meta && child_meta->external_nodes)
				topmost_external_nodes = child_meta->external_nodes;
		}
	}

	/*
	 * Walk upward from the parent of the detached node via
	 * metadata->parent.  At each ancestor, check if it has only
	 * one child left.  If so, mark it for pruning and continue.
	 * Stop when reaching a multi-child node, a node with
	 * external_nodes, or the root (parent == NULL).
	 */
	/*
	 * Resolve a skip-compressed initial parent slot.  When the detach is
	 * bootstrapped from a leaf whose holder is a compressed node (e.g.
	 * cds_ft_remove deriving the position from node->prev), the slot that
	 * holds the holder is skip-encoded (SKIP_X(cn->child)); ft_node_ptr
	 * does not strip the skip high bits, so resolve to the plain
	 * compressed flag here.  No-op for a plain (descent-supplied) slot.
	 */
	cur = ft_resolve_skip_compressed(ft, *detach_parent_flag_ptr);
	cur_depth = detach_depth - ft_parent_depth_span(cur, *detach_node_flag_ptr);

	while (cur) {
		struct cds_ft_metadata *metadata;
		bool is_root;

		metadata = cds_ft_item_to_metadata(ft_node_ptr(cur));
		metadata_stack[nr_metadata++] = metadata;
		is_root = (metadata->parent == NULL);

		assert(metadata->nr_child > 0);
		if (!prev_external_nodes_found && (metadata->nr_child == 1 && !metadata->external_nodes && !is_root)) {
			nr_clear++;
		}
		nr_branch++;
		/*
		 * Stop the upward prune at a surviving boundary: a multi-child
		 * node, the root, a level past one whose external_nodes were
		 * promoted (prev_external_nodes_found), or a node that keeps its
		 * own key (external_nodes) once a deeper promotion is already in
		 * flight (topmost_external_nodes set).  A SINGLE-child node that
		 * carries external_nodes does NOT stop here: pruning its only
		 * (on-path) child empties it, so it cannot stay -- its external
		 * chain is promoted into its own parent slot and the climb
		 * continues past it (handled just below).  When the detach was
		 * bootstrapped from a node that already carried external_nodes
		 * (descent callers, topmost set at start), that node is the one
		 * being promoted and a further external ancestor is a genuine
		 * boundary -- hence the `&& topmost_external_nodes` guard.
		 */
		if (prev_external_nodes_found || metadata->nr_child > 1 ||
		    (metadata->external_nodes && topmost_external_nodes) ||
		    is_root) {
			if (!is_root) {
				struct cds_ft_metadata *parent_meta =
					cds_ft_item_to_metadata(
						ft_node_ptr(metadata->parent));
				metadata_stack[nr_metadata++] = parent_meta;
			}
			/*
			 * Find the key byte for replace_ptr.  Only needed
			 * for internal parents (compressed handled
			 * separately below).
			 */
			if (!ft_node_compressed(cur))
				ft_node_find_child(ft, cur, *detach_node_flag_ptr,
					&n, NULL);
			break;
		}
		/*
		 * Single-child node made childless by the prune that carries a
		 * shorter key: promote its external chain (the deepest such node
		 * on the path is the one promoted; prev_external_nodes_found then
		 * stops the climb at the next, surviving level).
		 */
		if (metadata->external_nodes && !topmost_external_nodes)
			topmost_external_nodes = metadata->external_nodes;
		if (topmost_external_nodes)
			prev_external_nodes_found = true;

		/*
		 * Walk up: the current node becomes the child,
		 * update detach pointers to prune at this level.
		 */
		{
			struct cds_ft_inode_flag *parent_nf = metadata->parent;

			if (!parent_nf)
				break;
#ifdef FT_IMMEDIATE_FREE
			{
				unsigned char *_p = (unsigned char *) ft_node_ptr(parent_nf);
				if (*_p == 0xfe) {
					fprintf(stderr, "ft_detach_node: stale parent detected! "
						"cur=%p cur_depth=%u parent_nf=%p (poisoned) "
						"detach_depth=%u nr_clear=%d\n",
						cur, cur_depth, parent_nf,
						detach_depth, nr_clear);
					abort();
				}
			}
#endif
			/*
			 * Find the slot in the grandparent pointing to
			 * cur, which becomes the new detach_parent_flag_ptr.
			 * Find the slot in cur pointing to its child (the
			 * previous level), which becomes detach_node_flag_ptr.
			 */
			{
				/*
				 * Climb one level: cur (and its child-on-path,
				 * already at detach_node_flag_ptr's level) is pruned,
				 * so the detach target becomes cur and its parent
				 * becomes parent_nf.  The new detach_parent_flag_ptr
				 * must therefore point to PARENT_NF's slot in its own
				 * parent (so *detach_parent_flag_ptr == parent_nf ==
				 * the new cur), recovered in O(1) from parent_nf's
				 * metadata parent-slot offset.  ft_get_parent_slot
				 * handles all kinds: root (parent == NULL) -> &ft->root,
				 * compressed -> its grandparent slot, plain internal ->
				 * the body slot.  (The previous code recovered cur's own
				 * slot here, which aliases detach_parent_flag_ptr and
				 * left *detach_parent_flag_ptr != cur after the climb -- a
				 * latent bug, never reached because descent callers pass
				 * the surviving-ancestor detach point and break above.)
				 */
				struct cds_ft_metadata *parent_nf_meta =
					cds_ft_item_to_metadata(ft_node_ptr(parent_nf));
				struct cds_ft_inode_flag **new_parent_flag_ptr =
					ft_get_parent_slot(parent_nf_meta, ft);
				/*
				 * Defensive: if the recovered slot aliases the current
				 * detach_parent_flag_ptr, advancing would put both
				 * pointers at the same slot, breaking the replace which
				 * assumes detach_node_flag_ptr is WITHIN
				 * iter_node_flag's child array.
				 */
				if (new_parent_flag_ptr == detach_parent_flag_ptr)
					break;
				detach_node_flag_ptr = detach_parent_flag_ptr;
				detach_parent_flag_ptr = new_parent_flag_ptr;
			}
			cur_depth -= ft_parent_depth_span(parent_nf, cur);
			cur = parent_nf;
		}
	}

	iter_node_flag = *detach_parent_flag_ptr;
	elevated_old_child = *detach_node_flag_ptr;

	/*
	 * Replace within parent.  If the parent is a compressed node:
	 *
	 * If topmost_external_nodes is set, the child below the
	 * compressed node has variable-length key entries.  Keep the
	 * compressed node (its path is needed for lookups) and replace
	 * cn->child with the external node directly.
	 *
	 * Otherwise, replace the compressed node with a fresh internal
	 * node (e.g. compressed root from detach/graft_swap must remain
	 * internal).
	 */
	if (ft_node_compressed(iter_node_flag) ||
	    ft_node_skip_compressed(iter_node_flag)) {
		ret = ft_detach_node_replace_compressed_parent(ft,
			iter_node_flag, detach_parent_flag_ptr,
			topmost_external_nodes, &nr_clear, fuse_cell, pub, run);
		if (ret)
			goto end;
		/*
		 * Phase 2-style free walk for the orphaned chain below
		 * the compressed parent.  When a compressed cn's child
		 * was an internal node N with N->external_nodes
		 * promoted (or no external_nodes), the chain
		 * [N -> ... -> external_target] is now unreachable.
		 * Walk from @elevated_old_child (the original cn->child)
		 * down the single-child no-ext chain, collecting nodes
		 * to free.  The first iteration is special: the target
		 * itself may carry residual content (external_nodes)
		 * that was promoted as topmost_external_nodes.
		 */
		if (free_detached_subtree) {
			struct cds_ft_inode_flag *to_free[FT_MAX_DEPTH];
			int nr_to_free = 0, fi;
			struct cds_ft_inode_flag *walk_nf = elevated_old_child;
			bool phase2_first = true;

			while (walk_nf &&
			       !ft_node_external(walk_nf) &&
			       nr_to_free < FT_MAX_DEPTH) {
				struct cds_ft_inode_flag *next = NULL;
				unsigned int nr_child;
				struct cds_ft_node *ext_nodes;

				if (ft_node_compressed(walk_nf)) {
					struct cds_ft_compressed_node *cn;
					struct cds_ft_metadata *cm;

					cn = ft_compressed_node_ptr(
						ft_skip_child_ptr(walk_nf));
					cm = cds_ft_item_to_metadata(
						(struct cds_ft_inode *) cn);
					nr_child = cm->nr_child;
					ext_nodes = cm->external_nodes;
					next = cn->child;
				} else {
					struct cds_ft_metadata *m =
						cds_ft_item_to_metadata(
							ft_node_ptr(walk_nf));

					nr_child = m->nr_child;
					ext_nodes = m->external_nodes;
					if (nr_child == 1) {
						unsigned int key;

						for (key = 0; key < 256; key++) {
							next = ft_node_get_nth(ft,
								walk_nf, NULL,
								(uint8_t) key,
								FT_PF_NONE);
							if (next)
								break;
						}
					}
				}

				if (!phase2_first &&
				    (nr_child > 1 || ext_nodes))
					break;
				phase2_first = false;
				to_free[nr_to_free++] = walk_nf;
				walk_nf = next;
			}
			/*
			 * As in the replace-ptr free-walk below: a skip-compressed
			 * external leaf at the chain end keeps its path bytes in a
			 * separate, now-orphaned skip-target compressed node that the
			 * walk stops short of.  Free it (the external leaf stays
			 * caller-owned).
			 */
			if (walk_nf && ft_node_skip_compressed(walk_nf))
				free_compressed_node(ft,
					ft_skip_to_compressed(ft, walk_nf));
			for (fi = 0; fi < nr_to_free; fi++) {
				if (ft_node_compressed(to_free[fi]))
					free_compressed_node(ft,
						ft_compressed_node_ptr(
							ft_skip_child_ptr(to_free[fi])));
				else
					free_cds_ft_node(ft,
						ft_node_ptr(to_free[fi]));
			}
		}
	} else {
		/*
		 * Density was already propagated above (before
		 * structural changes).
		 */

		ret = ft_node_replace_ptr(ft,
			detach_node_flag_ptr,
			&iter_node_flag,
			&old_recompacted_node,
			metadata_stack[nr_branch - 1],
			n, (struct cds_ft_inode_flag *) topmost_external_nodes,
			detach_parent_flag_ptr == &ft->root,
			cur_depth, pub);
		if (!ret) {
			/*
			 * In-place key-disappearing remove (the holder stayed
			 * above min_child, so replace_ptr deferred its single
			 * reader-visible forward store into @pub instead of doing
			 * it): commit that store -- @old_val -> @new_val (NULL for a
			 * leaf delete, the promoted external chain head for an
			 * external promote) -- fused with @fuse_cell's ordered-list
			 * unsplice in ONE flip, then settle the pigeon bitmap bit (a
			 * reader channel, delete only) after.  Recompaction (pub
			 * unarmed) published its rebuilt node itself and stays
			 * two-commit -- the caller unsplices.
			 */
			if (pub && pub->armed) {
				ft_remove_one_commit(ft, pub->slot, pub->old_val,
					pub->new_val, fuse_cell, run);
				if (pub->pigeon_bitmap)
					cds_clear_bit_relaxed(
						pub->pigeon_bitmap->bitmap,
						pub->pigeon_bit);
			}
			/*
			 * Free the old detach subtree.  After
			 * ft_node_replace_ptr replaced it, the entire
			 * single-child chain from @elevated_old_child
			 * down is unreachable.  When the upward walk
			 * elevated the detach point, @elevated_old_child
			 * is the topmost ancestor that became orphaned;
			 * walking through its single-child chain reaches
			 * the original detach target (each elevated node
			 * has nr_child==1 by the walk's own invariant).
			 * Collect nodes first, then free after the walk
			 * completes (avoids use-after-free during
			 * traversal).  topmost_external_nodes (if any)
			 * was already saved and published at the
			 * replacement point.
			 */
			{
				struct cds_ft_inode_flag *to_free[FT_MAX_DEPTH];
				int nr_to_free = 0, fi;
				struct cds_ft_inode_flag *walk_nf = elevated_old_child;

				/*
				 * Free walk semantics:
				 *
				 *   Phase 1 -- elevated ancestors (always
				 *   single-child no-external by the upward
				 *   walk's own invariant).  Free @nr_clear
				 *   nodes unconditionally.
				 *
				 *   Phase 2 -- target and chain below.  For
				 *   destroy-style detach, walk the target's
				 *   single-child no-external chain (descent
				 *   tracking guarantees this) until we hit
				 *   an external, a multi-child node, a node
				 *   with external_nodes (its content was
				 *   either preserved as topmost_external_nodes
				 *   or still referenced), or nr_child == 0.
				 *   For move-style, stop -- the target is the
				 *   new trie's root and must be preserved.
				 */
				/*
				 * Pointer classification used by the walks
				 * below:
				 *
				 *   ft_node_external() returns true for
				 *   genuine external leaves AND for
				 *   external skip-target encodings
				 *   (FT_SKIP_MASK on an external pointer):
				 *   in both cases the underlying node is an
				 *   external leaf that the caller (not the
				 *   free walk) is responsible for reclaiming.
				 *
				 *   ft_node_compressed() -- checks
				 *   FT_COMPRESSED_MASK only -- covers both
				 *   plain compressed and legacy high-bit
				 *   skip-compressed: both encodings keep
				 *   FT_COMPRESSED_MASK on the parent slot,
				 *   and ft_compressed_node_ptr() strips
				 *   FT_TAG_MASK to recover the cn pointer.
				 *
				 *   Otherwise (neither external nor
				 *   compressed) the pointer is an internal
				 *   node (possibly with FT_SKIP_MASK from an
				 *   internal skip-target; dormant
				 *   today).
				 */
				/* Phase 1: elevated ancestors. */
				while (nr_to_free < nr_clear &&
				       walk_nf &&
				       !ft_node_external(walk_nf) &&
				       nr_to_free < FT_MAX_DEPTH) {
					struct cds_ft_inode_flag *next = NULL;

					if (ft_node_skip_compressed(walk_nf)) {
						/*
						 * Skip-encoded elevated link: the slot
						 * value encodes the child BELOW the
						 * elided skip-target compressed node
						 * (which carries the path bytes).  The
						 * orphaned node is that skip-target; the
						 * child is the next link down.  Queue the
						 * target's plain compressed flag and
						 * advance to the child.  ft_node_ptr /
						 * ft_compressed_node_ptr on the raw skip
						 * value would mis-free the child (taken
						 * from its low tag) and leak the target --
						 * this is the move-style detach's single-
						 * byte-prefix case, where the child below
						 * is the preserved move target.
						 */
						struct cds_ft_compressed_node *cn =
							ft_skip_to_compressed(ft, walk_nf);

						to_free[nr_to_free++] =
							ft_compressed_node_flag(cn);
						walk_nf = ft_skip_child_ptr(walk_nf);
						continue;
					}
					if (ft_node_compressed(walk_nf)) {
						struct cds_ft_compressed_node *cn;

						cn = ft_compressed_node_ptr(walk_nf);
						next = cn->child;
					} else {
						unsigned int key;

						for (key = 0; key < 256; key++) {
							next = ft_node_get_nth(ft,
								walk_nf, NULL,
								(uint8_t) key,
								FT_PF_NONE);
							if (next)
								break;
						}
					}
					to_free[nr_to_free++] = walk_nf;
					walk_nf = next;
				}

				/* Phase 2: target and chain below. */
				if (free_detached_subtree) {
					bool phase2_first = true;

					while (walk_nf &&
					       !ft_node_external(walk_nf) &&
					       nr_to_free < FT_MAX_DEPTH) {
						struct cds_ft_inode_flag *next = NULL;
						unsigned int nr_child;
						struct cds_ft_node *ext_nodes;

						if (ft_node_compressed(walk_nf)) {
							struct cds_ft_compressed_node *cn;
							struct cds_ft_metadata *cm;

							cn = ft_compressed_node_ptr(walk_nf);
							cm = cds_ft_item_to_metadata(
								(struct cds_ft_inode *) cn);
							nr_child = cm->nr_child;
							ext_nodes = cm->external_nodes;
							next = cn->child;
						} else {
							struct cds_ft_metadata *m =
								cds_ft_item_to_metadata(
									ft_node_ptr(walk_nf));

							nr_child = m->nr_child;
							ext_nodes = m->external_nodes;
							if (nr_child == 1) {
								unsigned int key;

								for (key = 0; key < 256; key++) {
									next = ft_node_get_nth(ft,
										walk_nf, NULL,
										(uint8_t) key,
										FT_PF_NONE);
									if (next)
										break;
								}
							}
						}

						/*
						 * Stop at content (multi-child
						 * or external_nodes), except
						 * on the very first phase-2
						 * iteration: that node is the
						 * detach target itself, which
						 * may carry residual content
						 * that was either promoted
						 * (topmost_external_nodes) or
						 * just cleared by the caller.
						 */
						if (!phase2_first &&
						    (nr_child > 1 || ext_nodes))
							break;
						phase2_first = false;
						to_free[nr_to_free++] = walk_nf;
						walk_nf = next;
					}
					/*
					 * The chain stops at @walk_nf.  If that is a
					 * skip-compressed external leaf, its path bytes
					 * live in a separate skip-target compressed node:
					 * the external leaf is caller-owned, but that
					 * skip-target is trie-owned and now orphaned, and
					 * the walk above stopped at the external pointer
					 * without seeing it.  Free it here.
					 */
					if (walk_nf && ft_node_skip_compressed(walk_nf))
						free_compressed_node(ft,
							ft_skip_to_compressed(ft, walk_nf));
				}
				for (fi = 0; fi < nr_to_free; fi++) {
					if (ft_node_compressed(to_free[fi]))
						free_compressed_node(ft,
							ft_compressed_node_ptr(
								to_free[fi]));
					else
						free_cds_ft_node(ft,
							ft_node_ptr(to_free[fi]));
				}
			}
		}
	}
	if (ret)
		goto end;

	/*
	 * Update address of parent ptr in its parent.
	 * Skip for compressed parents: the replacement was already
	 * published inline above.
	 */
	if (!ft_node_compressed(iter_node_flag) &&
	    !ft_node_skip_compressed(iter_node_flag)) {
		struct cds_ft_metadata *iter_meta =
			cds_ft_item_to_metadata(ft_node_ptr(iter_node_flag));

		dbg_printf("ft_detach_node: publish %p instead of %p\n",
			iter_node_flag, *detach_parent_flag_ptr);
		/*
		 * Recompaction publish (old_recompacted_node set => the holder
		 * was rebuilt smaller; a NULL topmost_external_nodes => a pure
		 * delete, no external-promote).  When fusion is requested, record
		 * the publish's 1-2 reader-visible edges (forward slot + a
		 * compressed grandparent's SKIP_X dual) and commit them in ONE
		 * flip with @fuse_cell's unsplice -- the recompaction dual of the
		 * in-place fusion above -- and signal it via pub->armed so the
		 * caller skips the standalone unsplice.  A compressed grandparent
		 * publishes into cn->child; every reader that descends a compressed
		 * child now resolves a flip proxy there
		 * (ft_cn_child_dereference_acquire_prefetch), so the parked forward
		 * edge is safe.  Direct publish for an in-place redundant
		 * republish, external-promote, or list off.
		 */
		if ((fuse_cell || run) && pub && !pub->armed &&
		    old_recompacted_node && !topmost_external_nodes) {
			struct ft_pub_rec rec = { .n = 0 };

			_ft_publish_to_parent(ft, iter_meta->parent,
				detach_parent_flag_ptr, iter_node_flag, &rec);
			ft_remove_commit_rec(ft, &rec, fuse_cell, run);
			pub->armed = true;
		} else {
			struct ft_pub_rec rec = { .n = 0 };

			/*
			 * Non-fused recompaction / external-promote / in-place
			 * redundant republish: route through the op flip-txn so the
			 * forward slot AND a compressed grandparent's SKIP_X dual flip
			 * atomically.  A lone edge reduces to a single release store
			 * in ft_ord_cell_flip (allocation-free, infallible).
			 */
			_ft_publish_to_parent(ft, iter_meta->parent,
				detach_parent_flag_ptr, iter_node_flag, &rec);
			ft_remove_commit_rec(ft, &rec, NULL, NULL);
		}

#ifdef FEATURE_FT_SKIP_COMPRESSED
		/*
		 * Post-detach canonicalization: if the surviving ancestor
		 * is now a non-root internal with exactly 1 live child and
		 * no external_nodes attached, fold it via the chain-compress
		 * 4-case merge (canonical form under SKIP_COMPRESSED).
		 */
		if (iter_meta->nr_child == 1 &&
		    !iter_meta->external_nodes &&
		    iter_meta->parent != NULL) {
			ft_canonicalize_chain_compress(ft, iter_node_flag,
				iter_meta, detach_parent_flag_ptr);
		}
#endif
	}
end:
	/* Reclaim safely after replacement. */
	if (old_recompacted_node)
		free_cds_ft_node(ft, old_recompacted_node);

	/*
	 * Density was already propagated before structural changes
	 * (above), while parent pointers were still valid.
	 */
	FT_TP(detach_node_exit, (int) ret);
	return ret;
}

/*
 * ft_unchain_node: remove @node from its duplicate chain using prev/next.
 *
 * @head_slot: address of the pointer that holds the head of the chain
 *             (e.g. &metadata->external_nodes or the parent's child slot).
 *             Only used when @node is the head of the chain.
 * @node:      the node to remove.
 *
 * For head nodes (node->prev is a flagged internal pointer): updates
 * *head_slot to point to node->next.
 * For non-head nodes (node->prev is a cds_ft_node): updates prev->next
 * to skip over node.
 * In both cases, if node->next exists, its prev pointer inherits
 * node->prev (either the parent pointer or the predecessor node).
 *
 * Ordering: next_node->prev is updated BEFORE the pointer publication
 * (*head_slot or prev_node->next).  If we published first, a concurrent
 * reader following the new head via a skip-compressed pointer could call
 * ft_skip_to_compressed and read the stale prev pointing to @node (the
 * node being removed, a cds_ft_node rather than the flagged parent),
 * returning a garbage compressed-node pointer.  rcu_assign_pointer on
 * the publication provides release semantics pairing with the reader's
 * rcu_dereference of child->prev.
 */
/*
 * Head promotion: @node, the head of a duplicate chain, leaves the trie and
 * @next_node (its next duplicate, non-NULL) takes its place at the same key.
 *
 * Publish a FRESH cell for @next_node -- its node field set while the cell is
 * still hidden, keeping cell->node WRITE-ONCE (the public cds_ft_cell_node and
 * the internal cell readers load it as a plain pointer, never a flip proxy) --
 * and swap it in for @node's cell, FUSED with the structural forward publish
 * (@next_node into *@head_slot) and, for a compressed holder, the grandparent
 * SKIP_X dual, in ONE flip (ft_ord_cell_swap_publish_multi).  A reader thus
 * never observes the promoted head at one index but the old head at another,
 * nor a torn cell->node-vs-external_nodes prefix comparison (ft_rebuild_key_upwalk).
 *
 * @parent_nf is the holder flag used by _ft_publish_to_parent to locate the
 * SKIP_X dual: a compressed node's PLAIN flag for a compressed holder, else the
 * internal holder flag (no dual).  List off (no cell), or a cell-alloc OOM,
 * degrade to the in-place publish (the prior behavior): the structural forward
 * and SKIP_X dual still flip atomically, only cell->node is retargeted in place
 * (transiently breaking write-once under memory pressure -- self-healing, and no
 * proxy is ever parked there).
 */
static
void ft_promote_head(struct cds_ft *ft, struct cds_ft_inode_flag *parent_nf,
		struct cds_ft_node **head_slot, struct cds_ft_node *node,
		struct cds_ft_node *next_node)
{
	struct ft_ord_cell *old_cell = ft->ordered_list ?
		ft_ord_cell_ptr(node->prev) : NULL;
	void *new_cell_flag = old_cell ?
		ft_ord_cell_alloc(ft, next_node, old_cell->parent) : NULL;
	struct ft_pub_rec rec = { .n = 0 };
	struct ft_ord_cell_edge sedges[2];
	unsigned int n_s;

	assert(next_node != NULL);
	if (old_cell && new_cell_flag) {
		/* Fresh-cell swap: cell->node stays write-once. */
		struct ft_ord_cell *new_cell = ft_ord_cell_ptr(new_cell_flag);

		cds_ft_item_to_metadata(new_cell)->incoming_byte =
			cds_ft_item_to_metadata(old_cell)->incoming_byte;
		/* Back-pointer wired before the forward publish (parent-first). */
		next_node->prev = new_cell_flag;
		_ft_publish_to_parent(ft, parent_nf,
			(struct cds_ft_inode_flag **) head_slot,
			(struct cds_ft_inode_flag *) next_node, &rec);
		n_s = ft_pub_rec_sedges(&rec, sedges);
		ft_ord_cell_swap_publish_multi(ft, old_cell, new_cell, sedges, n_s);
		ft_ord_cell_free(ft, old_cell);
	} else {
		/* List off, or cell-alloc OOM: in-place publish (degraded). */
		next_node->prev = node->prev;	/* inherit old cell / parent */
		if (old_cell)			/* OOM: retarget in place */
			rcu_assign_pointer(old_cell->node, next_node);
		_ft_publish_to_parent(ft, parent_nf,
			(struct cds_ft_inode_flag **) head_slot,
			(struct cds_ft_inode_flag *) next_node, &rec);
		n_s = ft_pub_rec_sedges(&rec, sedges);
		ft_ord_cell_flip(ft, sedges, n_s);
	}
}

static
void ft_unchain_node(struct cds_ft *ft, struct cds_ft_inode_flag *parent_nf,
		struct cds_ft_node **head_slot, struct cds_ft_node *node)
{
	struct cds_ft_node *next_node = ft_node_next(node);

	FT_TP(unchain_node, (const void *) head_slot, (const void *) node,
		!ft_node_external((struct cds_ft_inode_flag *) node->prev));
	if (ft_node_external((struct cds_ft_inode_flag *) node->prev)) {
		/* Non-head: prev is a cds_ft_node. */
		struct cds_ft_node *prev_node =
			(struct cds_ft_node *) node->prev;

		if (next_node)
			next_node->prev = node->prev;
		rcu_assign_pointer(prev_node->next, next_node);
	} else if (next_node) {
		/*
		 * Head with a successor: prev is the cell flag (list on) or the
		 * flagged parent (list off).  Promote @next_node via a fresh
		 * cell swap fused with the structural publish.
		 */
		ft_promote_head(ft, parent_nf, head_slot, node, next_node);
	} else {
		/*
		 * Head with no successor: the key disappears (only reached for a
		 * list-off internal external_nodes chain that empties -- a list-on
		 * key disappearance routes through the fused ft_remove_one_commit,
		 * and a compressed / body-slot leaf through ft_detach_node).  Clear
		 * the head slot (+ a compressed holder's SKIP_X dual) through the
		 * op flip-txn -- a lone edge is a bare release store.
		 */
		struct ft_pub_rec rec = { .n = 0 };
		struct ft_ord_cell_edge sedges[2];
		unsigned int n_s;

		_ft_publish_to_parent(ft, parent_nf,
			(struct cds_ft_inode_flag **) head_slot, NULL, &rec);
		n_s = ft_pub_rec_sedges(&rec, sedges);
		ft_ord_cell_flip(ft, sedges, n_s);
	}
	/*
	 * @node has left the trie: tombstone it.  Its next pointer is
	 * preserved (still == next_node) so a concurrent reader positioned
	 * on @node still follows the chain; the bit only marks removal for a
	 * later position-based remove.
	 */
	ft_node_mark_removed(node);
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
	/*
	 * Key-disappearing detach (head with no successor) + ordered list on:
	 * fuse the structural unlink with @dead_cell's unsplice in one flip.
	 * @pub.armed reports whether ft_detach_node fused it (in-place leaf
	 * delete) or left it for the two-commit fallback below.
	 */
	bool fuse_remove = cell_was_head && !cell_succ;
	struct ft_remove_pub pub = { .armed = false };
	struct ft_remove_pub *pubp = fuse_remove ? &pub : NULL;
	struct ft_ord_cell *fuse_cell = fuse_remove ? dead_cell : NULL;

	if (ft_node_external(holder_flag)) {
		/*
		 * node->prev is a cds_ft_node: @node is a non-head duplicate.
		 * Unlink it from its chain (ft_unchain_node relinks the pinned
		 * predecessor/successor and tombstones @node).  The key count is
		 * unchanged (other duplicates remain) and the chain head -- and
		 * any grandparent skip pointer to it -- is untouched, so no head
		 * slot is needed.
		 */
		ft_unchain_node(ft, NULL, NULL, node);
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
				key_len, true, fuse_cell, pubp, NULL);
			if (ret)
				ft_propagate_external_count_parent(ft, holder_flag, 1);
			else
				ft_node_mark_removed(node);
		} else {
			/*
			 * Removing the head, duplicates remain: key count unchanged.
			 * ft_promote_head fuses the cn->child republish with the
			 * grandparent SKIP_X dual (via _ft_publish_to_parent on cn's
			 * plain flag) and the cell swap in ONE flip -- so a candidate
			 * descent or ft_skip_reanchor up-walk never follows the stale
			 * skip pointer into the about-to-be-freed old head (the
			 * dangling-skip-slot UAF the standalone ft_update_skip_pointer
			 * re-encode used to close).
			 */
			ft_unchain_node(ft, ft_compressed_node_flag(cn),
				(struct cds_ft_node **) head_slot, node);
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
		if (!ft_node_next(node)) {
			/*
			 * Last entry: the external chain empties, so the prefix
			 * key disappears (the holder KEEPS its longer-key children
			 * -- prefix-with-siblings).  When the ordered list is on
			 * (fuse_remove), commit the external_nodes -> NULL store
			 * together with @dead_cell's ordered-list unsplice in ONE
			 * flip (the dual of the in-place leaf delete): a reader
			 * never sees the key gone from the structural index but
			 * present in the ordered list.  Readers resolve a parked
			 * proxy on external_nodes via ft_dereference_external.
			 * @pub.armed then tells the deferred-free block below the
			 * unsplice already happened.
			 */
			ft_propagate_external_count_parent(ft, holder_flag, -1);
			if (fuse_remove) {
				ft_remove_one_commit(ft,
					(struct cds_ft_inode_flag **) &holder_meta->external_nodes,
					(struct cds_ft_inode_flag *) node, NULL,
					dead_cell, NULL);
				ft_node_mark_removed(node);
				pub.armed = true;
			} else {
				/* List off: the single store is atomic alone. */
				ft_unchain_node(ft, holder_flag,
					(struct cds_ft_node **) &holder_meta->external_nodes,
					node);
			}
		} else {
			/* Duplicates remain: head promotion (fresh-cell swap). */
			ft_unchain_node(ft, holder_flag,
				(struct cds_ft_node **) &holder_meta->external_nodes,
				node);
		}
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
				key_len, true, fuse_cell, pubp, NULL);
			if (ret)
				ft_propagate_external_count_parent(ft, holder_flag, 1);
			else
				ft_node_mark_removed(node);
		} else {
			/* Removing the head, duplicates remain: key count unchanged. */
			ft_unchain_node(ft, holder_flag,
				(struct cds_ft_node **) head_slot, node);
			ret = 0;
		}
	}

	/*
	 * Head with no successor: the key disappeared, so its cell is unspliced
	 * from the ordered list (when enabled) and freed (deferred, for parked
	 * up-walkers).  A promotion (cell_succ) keeps the cell in place -- same
	 * key, only cell->node retargeted in ft_unchain_node -- so no list op.
	 * An in-place leaf detach already fused the unsplice into its structural
	 * flip (pub.armed); only the deferred cell free remains here.
	 */
	if (ret == 0 && fuse_remove) {
		/* cell_was_head implies ordered_list, so the list op always runs. */
		if (!pub.armed)
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
		/*
		 * Ordered list on: the NIL key is the global minimum (a prefix of
		 * every key), so its removal is the prefix-with-siblings clear at
		 * the root.  Fuse the root external_nodes -> NULL store with the
		 * head cell's unsplice in ONE flip so a reader never sees the key
		 * gone from one index but present in the other; readers resolve a
		 * parked proxy on external_nodes via ft_dereference_external.  List
		 * off: a plain NULL store, atomic alone.
		 */
		if (ft->ordered_list) {
			struct ft_ord_cell *dead = ft_ord_cell_ptr(external_nodes->prev);

			ft_remove_one_commit(ft,
				(struct cds_ft_inode_flag **) &metadata->external_nodes,
				(struct cds_ft_inode_flag *) external_nodes, NULL,
				dead, NULL);
			ft_ord_cell_free(ft, dead);
		} else {
			rcu_assign_pointer(metadata->external_nodes, NULL);
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

	/*
	 * Ordered list on: the whole key leaves the trie, so its head's cell is
	 * unspliced + freed below.  An in-place leaf detach fuses that unsplice
	 * into its structural flip (pub.armed); the prefix and recompaction /
	 * compressed-parent shapes stay two-commit (pub unarmed).
	 */
	struct ft_remove_pub pub = { .armed = false };
	struct ft_ord_cell *dead_cell = ft->ordered_list ?
		ft_ord_cell_ptr(chain_head->prev) : NULL;

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
		/*
		 * Fuse the external_nodes -> NULL store with the head cell's
		 * ordered-list unsplice in ONE flip (the ordered list on): a
		 * reader never sees the prefix key gone from the structural index
		 * but present in the ordered list.  Readers resolve a parked proxy
		 * on external_nodes via ft_dereference_external; @pub.armed tells
		 * the deferred-free block below the unsplice already happened.
		 * List off: a plain NULL store, atomic alone.
		 */
		if (ft->ordered_list) {
			ft_remove_one_commit(ft,
				(struct cds_ft_inode_flag **) &holder_meta->external_nodes,
				(struct cds_ft_inode_flag *) chain_head, NULL,
				dead_cell, NULL);
			pub.armed = true;
		} else {
			rcu_assign_pointer(holder_meta->external_nodes, NULL);
		}
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
			ft_get_parent_slot(holder_meta, ft), key_len, true,
			dead_cell, ft->ordered_list ? &pub : NULL, NULL);
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
	 * (detach reshapes ancestors and head_slot, not the head's prev).  An
	 * in-place leaf detach already fused the unsplice (pub.armed); only the
	 * deferred free remains.  List off: chain_head->prev is the flagged
	 * parent, no cell. */
	if (ret == 0 && ft->ordered_list) {
		if (!pub.armed)
			ft_ord_cell_unsplice(ft, dead_cell);
		ft_ord_cell_free(ft, dead_cell);
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

