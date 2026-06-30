// SPDX-FileCopyrightText: 2012-2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-only

/*
 * src/fractal-trie/ft-insert.h
 *
 * Userspace RCU library - Fractal Trie: the key-insert path -- descent to the
 * attach point and ft_attach_node, the one-commit ordered-list publish
 * (struct ft_insert_commit / ft_insert_one_commit) and the compressed-path
 * split.  The flip-batch, node-reserve and node-cluster builders it uses now
 * live in their own shared modules (ft-mutation-helpers.h, ft-cluster-build.h).
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
 * list with one urcu_txn_sw_group_commit -- a reader can never observe the fresh head
 * without its cell in the list (2026-06 review, 2.13).  @batch == NULL: direct
 * publish (ordered list off, or a shape not yet converted).
 */
struct ft_insert_commit {
	struct urcu_txn_sw_txn *txn;		/* armed at the publish site */
	struct cds_ft_inode_flag **slot;	/* forward-publish sentinel (one-commit
						 * parked) -- the txn settles the edges */
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
	/*
	 * Old internal node replaced by a recompact-relocation forward publish
	 * folded into the one-commit (ft_attach_node): the parked grandparent
	 * proxy resolves to it until the commit flips, so its grace-period-
	 * deferred free must be queued only AFTER the commit, like free_old_cn.
	 * NULL = no relocation (in-place reserve).
	 */
	struct cds_ft_inode *free_old_node;
	/*
	 * Deferred LIVE re-parent edge (split-compressed one-commit).  The live
	 * old child's back-pointer is the back-channel publish that exposes the
	 * fresh cluster to reanchor up-walkers; for a parked commit the forward
	 * publish IS the single commit, so this edge must be wired THERE (after
	 * the cell links are set), not during the build -- otherwise a reader
	 * reanchoring through the re-parented child reaches the fresh head before
	 * its cell is spliced (the inv_insert_splice_window race).  NULL = none.
	 */
	struct cds_ft_inode_flag *live_child;
	struct cds_ft_inode_flag *live_parent;
	struct cds_ft_inode_flag **live_slot;
	bool publish_to_parent;			/* settle via ft_publish_to_parent */
};

/*
 * Park the deferred LIVE re-parent edge (split-compressed one-commit) into
 * @batch, so the live old child's back-pointer flips to its new cluster parent
 * ATOMICALLY with the forward structural edge and the ordered-list neighbour
 * edges at the single urcu_txn_sw_group_commit.  That back-pointer is the only field a
 * reanchor up-walk reads to recover a skip-compressed node (ft_skip_reanchor,
 * which resolves flip proxies on the parent read), so until the commit a reader
 * resolves it to the OLD parent and never enters the fresh cluster -- closing
 * the window where the head was tree-reachable (via the re-parented child) but
 * not yet in the ordered list.
 *
 * The slot-offset / incoming-byte bookkeeping is set NOW, for the new parent: a
 * reader gated at the OLD parent -- always the compressed node being split, so
 * compressed -- skips incoming_byte, hence the early write is unobservable
 * until the commit makes the (possibly internal) new parent current.  Records
 * the parent-field edge (old -> @new_parent) into @txn; the commit settles it.
 */
static
void ft_park_live_parent_edge(struct cds_ft *ft,
		struct cds_ft_inode_flag *child,
		struct cds_ft_inode_flag *new_parent,
		struct cds_ft_inode_flag **slot,
		struct urcu_txn_sw_txn *txn)
{
	struct cds_ft_metadata *meta = NULL;
	struct cds_ft_inode_flag **field;

#ifdef FEATURE_FT_SKIP_COMPRESSED
	if (ft_node_skip_compressed(child))
		meta = cds_ft_item_to_metadata((struct cds_ft_inode *)
			ft_skip_to_compressed(ft, child));
	else
#endif
	if (ft_node_compressed(child))
		meta = cds_ft_item_to_metadata((struct cds_ft_inode *)
			ft_compressed_node_ptr(child));
	else if (!ft_node_external(child))
		meta = cds_ft_item_to_metadata(ft_node_ptr(child));

	if (meta) {
		ft_set_parent_slot(meta, new_parent, slot);
		field = &meta->parent;
	} else if (ft->ordered_list) {
		/*
		 * External head, ordered list ON: its parent lives in the cell
		 * carried by node->prev.
		 */
		field = &ft_ord_cell_ptr(
			((struct cds_ft_node *) child)->prev)->parent;
	} else {
		/*
		 * External head, ordered list OFF: there is no cell -- the
		 * parent is stored directly in node->prev (see ft_set_parent's
		 * external branch, which rcu_assigns prev = parent).  Re-parent
		 * that field, matching the immediate ft_set_parent the non-parked
		 * path would have done.
		 */
		field = (struct cds_ft_inode_flag **)
			&((struct cds_ft_node *) child)->prev;
	}
	ft_flip_txn_record_reserved(txn, (void **) field, *field, new_parent);
}

/*
 * One-commit insert tail (see struct ft_insert_commit): the structural slot is
 * recorded as a txn edge (the fresh head is invisible -- the slot still reads
 * its old value), and the head's parent chain is fully wired, so the
 * splice-position search runs exactly as the post-publish splice did (the
 * from-head seed walks the parent chain, never the recorded slot).
 *
 * Ordered list ON (@cell != NULL): park the <= 4 ordered-list neighbour edges
 * into the SAME txn, commit once -- the head becomes reachable in the structural
 * index AND spliced into the cell list atomically for every reader.  Ordered
 * list OFF (@cell == NULL): there is no cell to splice; the txn carries only the
 * structural edges (the slot publish recorded at the publish site, plus any
 * split-compressed live re-parent), and the single commit below makes that
 * structural publish atomic and freeze-before-install all the same.  Either way
 * it then settles all slots to their direct values; the real top's writer-only
 * wiring (parent, slot offset, incoming_byte) was done at record time.
 */
static
void ft_insert_one_commit(struct cds_ft *ft, const uint8_t *key,
		size_t key_len, struct ft_ord_cell *cell,
		struct ft_insert_commit *ic)
{
	if (cell) {
		struct ft_ord_cell *pred;
		struct urcu_txn_sw_list_node *pred_lnode;

		/*
		 * Locate the predecessor.  From-HEAD (seed at the fresh head, walk the
		 * live parent chain up) is the fast default and the only safe choice for
		 * the attach shapes: a from-root LT would descend through the attach's own
		 * mid-build re-parent edge and loop in the reanchor.  Two shapes need
		 * from-ROOT instead, and neither loops there:
		 *   - split-compressed (ic->live_child): the deferred live edge is parked,
		 *     so a from-head LT would reanchor through the not-yet-wired edge; the
		 *     old compressed node is intact, so from-root descends it cleanly.
		 *   - external_nodes prefix key (!ic->publish_to_parent): the head sits at
		 *     an internal node's external_nodes and sorts BEFORE its extensions, so
		 *     a from-head seed mis-locates it; from-root descends only live nodes
		 *     (the parked external_nodes resolves to "no head"), no live re-parent.
		 * TODO(perf): the from-root cases re-descend; revisit if they show up hot.
		 */
		pred = ft_ord_cell_find_pred_from_head(ft, key, key_len, cell,
			ic->live_child != NULL || !ic->publish_to_parent);
		/*
		 * Splice @cell after @pred via the public composable op, recorded
		 * straight into the structural commit txn (FT's type-7 proxy tag
		 * applies, so the splice is atomic with the structural publish for a
		 * bidirectional ordered reader).  Sentinel topology: a new MINIMUM (no
		 * predecessor) splices after the sentinel node -- add_after(sentinel) IS
		 * the old "head was @succ" endpoint flip; a new maximum lands before the
		 * sentinel naturally (pred->next was the sentinel).  add_after_prepare
		 * records pred->next: succ -> cell and succ->prev: pred -> cell, where
		 * either neighbour may be the sentinel -- the old <=4 hand-built edges
		 * (incl. head/tail) collapse to its 2.
		 */
		pred_lnode = pred ? ft_ord_cell_lnode(pred) : &ft->ord_sentinel.node;
		(void) urcu_txn_sw_list_add_after_prepare(ic->txn,
			ft_ord_cell_lnode(cell), pred_lnode);
	}

	/*
	 * Record the deferred LIVE re-parent edge (split-compressed shapes) into
	 * the SAME txn: the live old child's back-pointer is the back-channel a
	 * reanchor up-walk follows into the fresh cluster, so flipping it in the
	 * one commit -- together with the forward edge (recorded at the publish
	 * site) and the neighbour edges -- makes structural reachability and the
	 * ordered-list splice a single atomic publication for every reader.
	 */
	if (ic->live_child)
		ft_park_live_parent_edge(ft, ic->live_child,
			ic->live_parent, ic->live_slot, ic->txn);

	/*
	 * Freeze-on-free (doc §4.B): the old compressed/internal node this
	 * commit retires gets its one-way LIVE->DEAD tombstone BEFORE the commit
	 * unlinks it (a no-op under one writer; under MCAS a concurrent writer
	 * targeting it then fails its validate-live CAS).  Marked here, after the
	 * last abort point (the recording above is infallible into the
	 * pre-reserved txn).
	 */
	if (ic->free_old_cn)
		ft_meta_tombstone_set_flip(cds_ft_item_to_metadata(
			(struct cds_ft_inode *) ic->free_old_cn));
	if (ic->free_old_node)
		ft_meta_tombstone_set_flip(
			cds_ft_item_to_metadata(ic->free_old_node));
	/*
	 * THE commit: install every recorded edge -- the forward structural
	 * publish (forward slot + any compressed-parent skip-slot dual, or the
	 * set_nth slot proxy), the ordered-list neighbour edges and the live
	 * re-parent edge -- then flip the group and settle each slot to its
	 * direct value, all atomically.  The real top's writer-only wiring
	 * (parent, slot offset, incoming_byte) was done at record time, while
	 * still invisible.
	 */
	ft_flip_txn_commit(ft, ic->txn);
	ic->txn = NULL;
	/*
	 * The old compressed node a split replaced, or the old internal node a
	 * recompact-relocation publish replaced: readers resolved the parked
	 * proxy to it until the commit above, so only now may its grace-period-
	 * deferred free be queued.
	 */
	if (ic->free_old_cn)
		free_compressed_node(ft, ic->free_old_cn);
	if (ic->free_old_node)
		free_cds_ft_node(ft, ic->free_old_node);
}

/*
 * Publish @new_top into @slot (owned by @parent_nf): direct via
 * ft_publish_to_parent, or -- one-commit insert, @ic armed -- RECORD the
 * forward publish (the slot store, plus a compressed parent's dual skip-slot
 * store; both captured via _ft_publish_to_parent, which also runs @new_top's
 * writer-only parent-slot bookkeeping) into @ic->txn, so the forward edge
 * commits atomically with the ordered-list splice at insert_done's single flip.
 * The caller must have wired @new_top's parent back-pointer already (parent-
 * before-publish; with a recorded forward edge the cluster only becomes
 * reachable at the commit, by which time the wiring is complete either way).
 */
static
void ft_insert_publish_or_park(struct cds_ft *ft,
		struct cds_ft_inode_flag *parent_nf,
		struct cds_ft_inode_flag **slot,
		struct cds_ft_inode_flag *new_top,
		struct ft_insert_commit *ic)
{
	struct ft_pub_rec rec = { .n = 0 };
	unsigned int k;

	/*
	 * @ic is mandatory and armed by the caller (a bulk insert is a graft /
	 * merge_at, not this path), so the forward edge always rides the txn.
	 */
	assert(ic && ic->txn);
	_ft_publish_to_parent(ft, parent_nf, slot, new_top, &rec);
	for (k = 0; k < rec.n; k++)
		ft_flip_txn_record_reserved(ic->txn,
			(void **) rec.slot[k],
			(void *) rec.old_val[k],
			(void *) rec.new_val[k]);
	ic->slot = slot;	/* sentinel: one-commit forward recorded */
	ic->publish_to_parent = true;
}

/*
 * Arm the one-commit txn just before a fresh-head publish (fallible; the
 * caller's error unwind runs with nothing published).  @ic is mandatory -- a
 * bulk insert is a graft / merge_at, not this path.  Armed for BOTH ordered-list
 * states: the structural
 * slot publish commits through the txn either way (freeze-before-install, MCAS
 * Invariant-1); with the list on it additionally carries the cell-splice edges,
 * with the list off it carries only the structural edges (and insert_done
 * commits it with @cell == NULL).  Returns 0, or -ENOMEM.
 */
static
int ft_insert_commit_arm(struct cds_ft *ft, struct ft_insert_commit *ic)
{
	(void) ft;
	assert(ic);
	/*
	 * Forward publish (<=2: the slot store + a compressed parent's skip-slot
	 * dual) + <=4 cell neighbour edges (list-on) + the live re-parent edge.
	 */
	/*
	 * Edge budget: the forward publish (<=2: slot + a compressed parent's
	 * skip-slot dual) OR -- attach path -- the new key's reserved slot edge
	 * (1) plus the recompact-relocation grandparent publish (<=2) folded in;
	 * + <=4 cell neighbour edges (list-on) + the live re-parent edge.
	 */
	ic->txn = ft_flip_txn_create_bounded(9);
	if (!ic->txn)
		return -ENOMEM;
	return 0;
}

/*
 * Park a "new key at an existing internal node" publish into the one-commit
 * batch.  The head is published by storing it into @metadata->external_nodes
 * (not a child slot), so park a flip proxy THERE -- readers resolve it via
 * ft_dereference_external -- and the structural publish then commits atomically
 * with the ordinal-cell splice at the single urcu_txn_sw_group_commit.  No transient
 * half-spliced list state.  Settles direct (publish_to_parent false: there is
 * no parent child-slot to re-encode).  @ic must be armed.  The fresh-key case
 * parks old == NULL, so a reader resolves the proxy to "no head" until the
 * commit.
 */
static
void ft_insert_park_external_nodes(struct cds_ft *ft,
		struct cds_ft_metadata *metadata, struct cds_ft_node *node,
		struct ft_insert_commit *ic)
{
	(void) ft;
	ft_flip_txn_record_reserved(ic->txn,
		(void **) &metadata->external_nodes,
		(void *) metadata->external_nodes, (void *) node);
	ic->slot = (struct cds_ft_inode_flag **) &metadata->external_nodes;
	ic->publish_to_parent = false;
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

/*
 * Free a freshly-built, never-published split cluster -- the nodes tracked in
 * @created[0 .. @nr_created).  Shared by both compressed-split helpers'
 * -ENOMEM error paths AND the abort-fully path in
 * ft_insert_compressed_key_shorter (when the one-commit arm fails after a
 * successful build).  The live old child (cn->child) is never tracked in
 * @created, and free_*_unpublished free only the node (not its children), so
 * the live subtree is never touched.
 *
 * Freed PARENT-FIRST (reverse of the bottom-up creation order): a skip-encoded
 * top is resolved to its compressed node via its child's back-pointer
 * (ft_skip_to_compressed reads child->parent), so the child must still be live
 * when the parent is freed.  (A partial-build error path never has both, but
 * the post-build abort frees the whole cluster, so the order matters.)
 */
static
void ft_free_unpublished_split_cluster(struct cds_ft *ft,
		struct cds_ft_inode_flag *const *created, int nr_created)
{
	int i;

	for (i = nr_created - 1; i >= 0; i--) {
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

static
int ft_split_compressed_insert(struct cds_ft *ft,
		struct cds_ft_inode_flag **parent_slot,
		struct cds_ft_inode_flag *compressed_flag,
		const uint8_t *iter_key,	/* key bytes at compressed node's depth */
		unsigned int remaining_key,	/* key bytes remaining from compressed depth */
		unsigned int diverge_pos,	/* position within compressed path */
		struct cds_ft_node *child_node,	/* new external node to insert */
		unsigned int node_depth,	/* depth of the compressed node */
		struct ft_insert_commit *ic)	/* one-commit insert (mandatory) */
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
		ft_meta_nr_child_set(sfx_meta, 1);
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
		ft_meta_nr_child_set(nb_meta, 1);
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
				*oslot = sfx_skip_flag;
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
		ft_meta_nr_child_set(pfx_meta, 1);
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
	if (deferred_child) {
		/*
		 * deferred_child is the LIVE old child (cn->child).  Setting its
		 * back-pointer is the back-channel publish that exposes this fresh
		 * cluster to a reanchor up-walker, so defer it to the parked
		 * one-commit: ft_insert_one_commit parks it into the SAME flip
		 * batch (structure + cell + this edge flip atomically; the splice
		 * search runs from the root so it does not need it wired early).
		 */
		ic->live_child = deferred_child;
		ic->live_parent = deferred_parent;
		ic->live_slot = deferred_slot;
	}
	ft_insert_publish_or_park(ft, cn_meta->parent, parent_slot, top_flag, ic);

	/*
	 * 7. Free the old compressed node.  Parked publish: readers resolve
	 * the proxy to @cn until the commit, so defer the free past it.
	 */
	ic->free_old_cn = cn;

	return 0;

error:
	ft_free_unpublished_split_cluster(ft, created, nr_created);
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
		unsigned int node_depth,
		/*
		 * The LIVE old-child re-parent edge (cn->child into the new
		 * suffix/junction) is RETURNED, not wired: the caller defers it to
		 * the parked one-commit (so it flips atomically with the cell) or
		 * wires it directly (list-off / duplicate).  See the same shape in
		 * ft_split_compressed_insert.  NULL out = no live edge.
		 */
		struct cds_ft_inode_flag **live_child_ret,
		struct cds_ft_inode_flag **live_parent_ret,
		struct cds_ft_inode_flag ***live_slot_ret,
		/*
		 * Out: the freshly-built (still-unpublished) cluster nodes, so the
		 * caller can tear it down on a post-return abort.  Optional (NULL).
		 */
		struct cds_ft_inode_flag **created_ret,
		int *nr_created_ret)
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
		ft_meta_nr_child_set(sfx_meta, 1);
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
				*oslot = sfx_skip_flag;
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
		ft_meta_nr_child_set(pfx_meta, 1);
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
			ft_meta_nr_child_set(pfx_meta, 1);
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
	/* Return the live edge; the caller defers (parked) or wires (direct). */
	*live_child_ret = deferred_child;
	*live_parent_ret = deferred_parent;
	*live_slot_ret = deferred_slot;
	/*
	 * Hand the built (still-unpublished) cluster back so the caller can tear
	 * it down if it must abort after we return (a one-commit arm -ENOMEM):
	 * created[] holds exactly the fresh cluster nodes, never the live
	 * cn->child.
	 */
	if (created_ret) {
		memcpy(created_ret, created,
			(size_t) nr_created * sizeof(created[0]));
		*nr_created_ret = nr_created;
	}

	FT_TP(compressed_split, "key_shorter", (const void *) cn, cn->len,
		(const void *) top_flag, remaining);
	*top_ret = top_flag;
	*jct_ret = jct_flag;
	return 0;

error:
	ft_free_unpublished_split_cluster(ft, created, nr_created);
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
		/*
		 * Accumulates the one-commit forward edges for a recompact-
		 * relocating reserve: the reserve set_nth records a compressed
		 * parent's SKIP_X dual here, and the forward fold below adds the
		 * grandparent slot edge, so both flip ATOMICALLY in ic->txn.
		 */
		struct ft_pub_rec rec = { .n = 0 };

		key_value = *(--iter_key);
		dbg_printf("publish branch at level %d, key %u\n", level - 1, (unsigned int) key_value);

		ret = ft_insert_commit_arm(ft, ic);
		if (ret)
			goto check_error;

		/* We need to use set_nth on the previous level. */
		iter_dest_node_flag = attach_node_flag;
		/*
		 * Every ft_attach_node caller passes a non-NULL @ic and
		 * ft_insert_commit_arm has armed ic->txn above (an arm failure
		 * jumps to check_error), so the one-commit txn is always present
		 * here -- the reserved-byte publish below is unconditional.
		 */
		assert(ic && ic->txn);
		{
			struct cds_ft_inode_flag **slot_ptr = NULL;

			/*
			 * One-commit insert (reserved-byte model): occupy
			 * key_value's slot but leave it resolving to its OLD
			 * value, then RECORD the slot edge (old -> the fresh
			 * top) into ic->txn so it settles atomically with the
			 * ordered-list edges at insert_done.  Nothing foreign is
			 * stored in the slot during the build -- the fresh key
			 * stays invisible because the slot still reads its old
			 * value (NULL for a new byte, or the displaced external).
			 *
			 *  - new byte (old_node_flag == NULL): RESERVE it via a
			 *    set_nth with a NULL child -- sets the bitmap bit and
			 *    bumps nr_child, leaving a bit-set+NULL slot that
			 *    reads as not-present, and may recompact + relocate
			 *    the node (the reserved byte rides along; the edge is
			 *    recorded against the FINAL slot below).
			 *  - displaced external (old_node_flag != NULL): the slot
			 *    already holds it; no set_nth (a NULL store would drop
			 *    the live external before the commit).
			 *
			 * Then fully wire the fresh top NOW (parent, slot offset,
			 * incoming_byte) -- invisible while the slot reads old --
			 * so the splice-position search at insert_done can rebuild
			 * the new key through the parent chain.
			 */
			if (!old_node_flag) {
				ret = ft_node_set_nth_rec(ft, &iter_dest_node_flag,
					key_value, NULL, &old_recompacted_node,
					metadata, level - 1, false, &rec);
				if (ret) {
					dbg_printf("branch publish error %d\n", ret);
					goto check_error;
				}
			}
			ft_node_get_nth_skip(iter_dest_node_flag, &slot_ptr,
				key_value, FT_PF_NONE);
			assert(slot_ptr);
			ft_set_parent(ft, iter_node_flag, iter_dest_node_flag,
				slot_ptr);
			ft_flip_txn_record_reserved(ic->txn, (void **) slot_ptr,
				(void *) old_node_flag,
				(void *) iter_node_flag);
			ic->slot = slot_ptr;
			FT_TP(tree_edge_set, (const void *) ft,
				(const void *) iter_dest_node_flag,
				(unsigned int) (level - 1), (uint8_t) key_value,
				(const void *) iter_node_flag);
		}
		/*
		 * Phase 2: iter_node_flag's parent is now wired (by
		 * ft_node_set_nth above, either in-place or via recompact's
		 * reparent loop; by the explicit raw store for a one-commit
		 * insert).  Re-parent the LIVE displaced external head onto the
		 * fresh cluster top iter_node_flag.  external_nodes is
		 * reader-reachable through its old slot until the forward publish,
		 * so an up-walk would follow its prev / cell->parent INTO the
		 * not-yet-published cluster (and from there its build-invisible
		 * internals).  So this re-parent must flip ATOMICALLY with the
		 * forward publish below, not before it: park it into the
		 * one-commit (ic->live_child), which ft_insert_one_commit replays
		 * via ft_park_live_parent_edge (resolving the external head's
		 * cell->parent / prev).  The cluster then becomes reachable via
		 * BOTH its forward slot and this back-pointer in one flip.
		 */
		if (external_nodes) {
			/* ic->txn is always armed here (see the assert above). */
			ic->live_child =
				(struct cds_ft_inode_flag *) external_nodes;
			ic->live_parent = iter_node_flag;
			ic->live_slot = NULL;
		}
		/* Attach branch (unlink the old node from the trie).
		 * ft_publish_to_parent handles skip pointer update
		 * if the attach target is a compressed node's child.
		 */
		if (iter_dest_node_flag != attach_node_flag) {
			unsigned int k;

			/*
			 * One-commit AND the reserve recompacted the attach node: the
			 * relocation swaps the OLD attach node for the fresh copy at its
			 * grandparent slot.  @rec already holds a compressed parent's
			 * SKIP_X dual (recorded by the reserve ft_node_set_nth_rec
			 * above, instead of a premature bare store); add the forward
			 * grandparent slot edge here so the relocation flips ATOMICALLY
			 * with the new key's slot edge in ic->txn -- "relocate + new
			 * key" is one publication.  ic->txn was armed before the build,
			 * so no allocation (hence no failure) here.  The old node stays
			 * resolved-to via the parked grandparent proxy until the commit,
			 * so defer its free past insert_done.
			 */
			_ft_publish_to_parent(ft, attach_node_flag,
				attach_node_flag_ptr, iter_dest_node_flag, &rec);
			for (k = 0; k < rec.n; k++)
				ft_flip_txn_record_reserved(ic->txn,
					(void **) rec.slot[k],
					(void *) rec.old_val[k],
					(void *) rec.new_val[k]);
			ic->free_old_node = old_recompacted_node;
			old_recompacted_node = NULL;
		} else {
			/*
			 * In-place reserve (dest == attach node): a redundant
			 * same-value republish; the direct publish handles the
			 * skip-slot bookkeeping.
			 */
			ft_publish_to_parent(ft, attach_node_flag,
				attach_node_flag_ptr, iter_dest_node_flag);
		}

		/* Reclaim safely after unlink (deferred to the commit when the
		 * relocation was folded into ic->txn above). */
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
		 * one-commit txn has no recorded edges yet (the slot edge is
		 * recorded only after the last fallible step, the reserve
		 * set_nth, succeeds; nothing is ever stored in a live slot
		 * during the build): destroy it.
		 */
		if (ic->txn) {
			ft_flip_txn_destroy(ic->txn);
			ic->txn = NULL;
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
void ft_chain_node(struct cds_ft *ft, struct cds_ft_node *last_node,
		struct cds_ft_node *node)
{
	FT_TP(chain_node, (const void *) last_node, (const void *) node);
	/*
	 * Add node to tail of list to ensure that RCU traversals will
	 * always see either the prior node or the newly added if
	 * executed concurrently with a sequence of add followed by del
	 * on the same key. Safe against concurrent RCU read traversals.
	 *
	 * The prev pointer is write-side only (mutex-held), so a plain
	 * store is sufficient.  The forward link is the reader-visible
	 * publish: express it as a single-edge flip descriptor (a lone
	 * release store, like rcu_assign_pointer) so it is MCAS-expressible.
	 */
	node->prev = last_node;
	node->next = NULL;
	ft_chain_next_flip(ft, &last_node->next, NULL, node);
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
	/*
	 * Phase 2: park the LIVE displaced-external-head re-parent (the old
	 * external cn->child's cell->parent / prev -> branch) into the
	 * one-commit so it flips ATOMICALLY with the forward publish below: a
	 * reader never sees cn->child re-parented onto branch while the
	 * grandparent slot still points at cn (or vice versa).  ft_insert_one_
	 * commit replays it via ft_park_live_parent_edge (which resolves the
	 * external head's cell->parent / prev).
	 */
	ic->live_child = (struct cds_ft_inode_flag *) cn->child;
	ic->live_parent = branch;
	ic->live_slot = NULL;
	ft_insert_publish_or_park(ft, d->nf, &cn->child, branch, ic);
	/* One-commit (parked): the +1 follows the commit at insert_done. */
	ic->count_from = branch;
	return 0;
arm_unwind:
	if (ic->txn) {
		ft_flip_txn_destroy(ic->txn);
		ic->txn = NULL;
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
	ic->count_from = d->pnf;
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
	struct cds_ft_inode_flag *live_child, *live_parent;
	struct cds_ft_inode_flag **live_slot;
	struct cds_ft_inode_flag *split_created[FT_MAX_DEPTH];
	int split_nr_created = 0;
	int sret;

	/*
	 * A key-shorter insert is always a NEW key: a key ending inside a
	 * compressed path has no node at that depth that could already hold it
	 * (a real duplicate is caught on descent, before this split), so the
	 * freshly-built junction always takes a fresh head -- there is no
	 * duplicate case here.
	 */
	(void) unique_node_ret;

	sret = ft_split_compressed_key_shorter(ft,
		d->nf, d->nfp, remaining, &top_flag, &jct_flag, d->depth,
		&live_child, &live_parent, &live_slot,
		split_created, &split_nr_created);
	if (sret)
		return sret;
	jct_meta = cds_ft_item_to_metadata(ft_node_ptr(jct_flag));
	assert(!ft_node_compressed(jct_flag));
	assert(jct_meta->external_nodes == NULL);	/* always a fresh head */
	/*
	 * Attach the fresh head to the junction NOW, while the whole cluster is
	 * still invisible, so the parked one-commit publish below makes the
	 * structural attach and the ordered-list splice atomic.
	 */
	ft_external_head_set_parent(ft, node, jct_flag);
	node->next = NULL;
	/* Cluster-internal store: the junction is unpublished. */
	jct_meta->external_nodes = node;
	sret = ft_insert_commit_arm(ft, ic);
	if (sret) {
		/*
		 * Arm failed (-ENOMEM): abort the whole insert.  The split
		 * cluster is still build-invisible (nothing published, cn->child
		 * untouched, @d->nf still reader-reachable at @d->nfp), so roll
		 * the head attach back and TEAR THE CLUSTER DOWN -- the structure
		 * is left byte-for-byte unchanged.
		 *
		 * Do NOT publish a key-neutral restructure here: the forward
		 * publish and the live old-child re-parent must flip ATOMICALLY
		 * (the re-parent's source is the live cn->child, so wiring it
		 * before the forward exposes the unpublished cluster to an
		 * up-walk), which needs the one-commit txn we just failed to
		 * allocate.  There is no MCAS-expressible publish on this path --
		 * a bare forward store would sit outside the descriptor set --
		 * only a clean abort.  The caller surfaces MEMORY_ERROR;
		 * insert_done resets node->prev for a retry.
		 */
		jct_meta->external_nodes = NULL;
		node->next = NULL;
		ft_free_unpublished_split_cluster(ft, split_created,
			split_nr_created);
		return sret;
	}
	/*
	 * top_flag's own back-pointer was wired in the split (it is a fresh
	 * cluster node).  The LIVE old-child re-parent is the back-channel that
	 * exposes the cluster to a reanchor up-walk, so defer it to the parked
	 * one-commit: it flips atomically with the forward publish and the cell
	 * (fresh head + ordered list).
	 */
	ic->live_child = live_child;
	ic->live_parent = live_parent;
	ic->live_slot = live_slot;
	ft_insert_publish_or_park(ft, d->pnf, d->nfp, top_flag, ic);
	/*
	 * Parked one-commit: the new key's +1 count and the old compressed
	 * node's free both follow the commit at insert_done (a reader resolves
	 * the parked proxy to the old node until then).
	 */
	ic->count_from = jct_flag;
	ic->free_old_cn = ft_compressed_node_ptr(d->nf);
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
				cn, cn->len, d->depth, cn_meta->external_nodes, (unsigned)ft_meta_nr_child(cn_meta));
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
	struct ft_insert_commit ic = { 0 };

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
				ft_chain_node(ft, last_node, node);
				ret = 0;
			} else {
				/* New key at this internal node. */
				ft_external_head_set_parent(ft, node, d.nf);
				node->next = NULL;
				if (ft->ordered_list) {
					/*
					 * Park the external_nodes publish into the
					 * one-commit batch -- readers resolve the
					 * proxy via ft_dereference_external -- so the
					 * structural publish commits atomically with
					 * the ordinal-cell splice at insert_done's
					 * single flip (no transient half-spliced
					 * list).  Count follows the commit
					 * (ic.count_from).
					 */
					ret = ft_insert_commit_arm(ft, &ic);
					if (ret)
						goto insert_done;
					ft_insert_park_external_nodes(ft,
						metadata, node, &ic);
					ic.count_from = d.nf;
				} else {
					/*
					 * List off: external_nodes is the single
					 * reader-visible slot (readers resolve via
					 * ft_dereference_external).  Commit the NEW-key
					 * publish as a 1-edge flip -- a lone release
					 * store, infallible and MCAS-expressible --
					 * instead of a bare store.
					 */
					struct ft_ord_cell_edge edge = {
						.slot = (struct ft_ord_cell **)
							&metadata->external_nodes,
						.old_target = NULL,
						.new_target = (struct ft_ord_cell *)
							node,
					};

					ft_ord_cell_flip_one(&edge);
					ft_propagate_external_count_parent(ft,
						d.nf, 1);
				}
				ret = 0;
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
			ft_chain_node(ft, last_node, node);
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
		if (ret != 0) {
			node->prev = NULL;
			if (ic.txn)
				ft_flip_txn_destroy(ic.txn);
		} else if (ic.slot) {
			/*
			 * One-commit: the structural slot publish was recorded
			 * into ic.txn (the key is invisible -- the slot reads
			 * its old value).  No cell to splice (list off), so
			 * commit the structural edges alone (@cell == NULL); a
			 * reader flips from not-present to the new key
			 * atomically.  Count propagation follows the commit.
			 */
			ft_insert_one_commit(ft, _key, _key_len, NULL, &ic);
			ft_propagate_external_count_parent(ft,
				ic.count_from ? ic.count_from : *d.pnfp, 1);
		} else if (ic.txn) {
			/* Armed but nothing parked (duplicate append): no
			 * structural publish to commit. */
			ft_flip_txn_destroy(ic.txn);
			ic.txn = NULL;
		}
	} else {
		if (ret != 0) {
			node->prev = NULL;
			ft_ord_cell_free_unpublished(ft, precell);
			if (ic.txn)
				ft_flip_txn_destroy(ic.txn);
		} else if (ft_node_external((struct cds_ft_inode_flag *) node->prev)) {
			ft_ord_cell_free_unpublished(ft, precell);
			if (ic.txn)
				ft_flip_txn_destroy(ic.txn);
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
		}
		/*
		 * No final else: after the one-commit conversion every
		 * published fresh head here parks a structural slot (ic.slot),
		 * so the old B-lite (ic.spliced) and post-publish-splice
		 * fallbacks are unreachable -- the cell is always spliced
		 * atomically by ft_insert_one_commit above.
		 */
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
	struct ft_insert_commit ic = { 0 };

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
				&nr_snapshot, &ret, &ic);
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
					NULL, &ic);
			if (ret == 0) {
				/* Parked one-commit: +1 follows the commit. */
				if (!ic.slot)
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
				node, old_node_ret, &ic);
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
				/*
				 * Ordered list on: publish the new head into
				 * external_nodes AND swap its cell in ONE atomic flip
				 * -- external_nodes is a plain pointer slot, readers
				 * resolve a parked proxy via ft_dereference_external.
				 * List off: a plain publish, no cell.
				 */
				if (ft->ordered_list) {
					struct ft_ord_cell *old_cell =
						ft_ord_cell_ptr(external_nodes->prev);

					/*
					 * The flip is the op's sole side-effect (the
					 * new head is fresh); on OOM nothing is applied,
					 * the old chain is intact (do NOT free its cell)
					 * and the replace aborts retriably.
					 */
					if (ft_ord_cell_swap_publish(ft, old_cell,
							precell,
							(struct cds_ft_inode_flag **)
								&metadata->external_nodes,
							(struct cds_ft_inode_flag *)
								external_nodes,
							(struct cds_ft_inode_flag *)
								node) != 0) {
						ret = -ENOMEM;
						goto insert_replace_done;
					}
					ft_ord_cell_free(ft, old_cell);
				} else {
					/*
					 * List off: external_nodes is the single
					 * reader-visible slot (readers resolve via
					 * ft_dereference_external).  Commit the swap as a
					 * 1-edge flip -- a lone release store, infallible
					 * and MCAS-expressible -- instead of a bare store.
					 */
					struct ft_ord_cell_edge edge = {
						.slot = (struct ft_ord_cell **)
							&metadata->external_nodes,
						.old_target = (struct ft_ord_cell *)
							external_nodes,
						.new_target = (struct ft_ord_cell *) node,
					};

					ft_ord_cell_flip_one(&edge);
				}
			} else {
				/* No external nodes yet. New key at this node. */
				ft_external_head_set_parent(ft, node, d.nf);
				node->next = NULL;
				if (ft->ordered_list) {
					/*
					 * Park external_nodes -- readers resolve the
					 * proxy via ft_dereference_external -- so it
					 * commits atomically with the ordinal-cell
					 * splice (no transient half-spliced list).
					 */
					ret = ft_insert_commit_arm(ft, &ic);
					if (ret)
						goto insert_replace_done;
					ft_insert_park_external_nodes(ft,
						metadata, node, &ic);
					ic.count_from = d.nf;
				} else {
					/*
					 * List off: external_nodes is the single
					 * reader-visible slot (readers resolve via
					 * ft_dereference_external).  Commit the NEW-key
					 * publish as a 1-edge flip -- a lone release
					 * store, infallible and MCAS-expressible --
					 * instead of a bare store.
					 */
					struct ft_ord_cell_edge edge = {
						.slot = (struct ft_ord_cell **)
							&metadata->external_nodes,
						.old_target = NULL,
						.new_target = (struct ft_ord_cell *)
							node,
					};

					ft_ord_cell_flip_one(&edge);
					ft_propagate_external_count_parent(ft,
						d.nf, 1);
				}
			}
			ret = 0;
		} else {
			dbg_printf("_cds_ft_insert_replace: replacing external chain %p\n",
					ft_node_ptr(d.nf));
			/* External node at end of key. Replace chain: key count unchanged. */
			*old_node_ret = (struct cds_ft_node *) ft_node_ptr(d.nf);
			ft_external_head_set_parent(ft, node, d.pnf);
			node->next = NULL;
			{
				/*
				 * Atomic replace: publish the new head into every
				 * reader-visible slot in ONE flip.  The plain case is a
				 * single forward edge at d.nfp (a plain external child, or
				 * a plainly-reached compressed parent's cn->child).  A leaf
				 * reached through a SKIP_X suffix is a DUAL: the exact
				 * descent reads cn->child while the candidate descent reads
				 * the grandparent skip slot, and BOTH name the leaf -- so
				 * the new head must appear at both atomically or a reader
				 * sees them disagree (and the old leaf is freed after this).
				 * Recording cn->child as a second flip edge (not a separate
				 * bare store) fuses the pair, mirroring the remove
				 * external-promote dual (ft_remove_commit_rec).  This sedge
				 * model is shared by both ordered-list states; the cell swap
				 * rides the same flip when the list is on.
				 */
				struct ft_ord_cell_edge sedges[2];
				unsigned int n_sedge = 0;

				sedges[0].slot = (struct ft_ord_cell **) d.nfp;
				sedges[0].old_target = (struct ft_ord_cell *) d.nf;
				sedges[0].new_target = (struct ft_ord_cell *) node;
				n_sedge = 1;

#ifdef FEATURE_FT_SKIP_COMPRESSED
				if (ft_node_compressed(d.pnf)) {
					struct cds_ft_compressed_node *cn =
						ft_compressed_node_ptr(d.pnf);
					struct cds_ft_metadata *cn_meta =
						cds_ft_item_to_metadata(
							(struct cds_ft_inode *) cn);
					struct cds_ft_inode_flag **sslot =
						ft_get_parent_slot(cn_meta, ft);

					if (sslot && ft_node_skip_compressed(*sslot)) {
						/* edge 0: grandparent SKIP_X dual. */
						sedges[0].slot =
							(struct ft_ord_cell **) sslot;
						sedges[0].old_target =
							(struct ft_ord_cell *) *sslot;
						sedges[0].new_target =
							(struct ft_ord_cell *)
							ft_skip_compressed_flag(
								(struct cds_ft_inode_flag *)
									node, cn->len);
						/* edge 1: cn->child forward (exact descent). */
						sedges[1].slot =
							(struct ft_ord_cell **) &cn->child;
						sedges[1].old_target =
							(struct ft_ord_cell *) cn->child;
						sedges[1].new_target =
							(struct ft_ord_cell *) node;
						n_sedge = 2;
					}
				}
#endif
				if (ft->ordered_list) {
					/* Swap the structural slot(s) AND the head's
					 * cell in one flip.  The new head is fresh, so
					 * the flip is the op's sole side-effect: self-
					 * allocate (NULL txn); on OOM nothing is applied,
					 * the old head's cell is NOT freed and the replace
					 * aborts retriably. */
					struct ft_ord_cell *old_cell =
						ft_ord_cell_ptr((*old_node_ret)->prev);

					if (ft_ord_cell_swap_publish_multi(ft, old_cell,
							precell, sedges, n_sedge,
							NULL) != 0) {
						ret = -ENOMEM;
						goto insert_replace_done;
					}
					ft_ord_cell_free(ft, old_cell);
				} else {
					/*
					 * List off: no cell.  The new head is fresh, so
					 * the flip is the op's sole side-effect with no
					 * pre-flip live store -- abortable: self-allocate
					 * (ft_ord_cell_flip_try); on a multi-edge OOM
					 * nothing is applied, the old head is NOT freed and
					 * the replace aborts retriably.  A lone edge takes
					 * the infallible on-stack store.
					 */
					if (ft_ord_cell_flip_try(ft, sedges,
							n_sedge) != 0) {
						ret = -ENOMEM;
						goto insert_replace_done;
					}
				}
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
				(struct cds_ft_node *) ft_node_ptr(d.nf), &ic);
		if (ret == 0) {
			/* Parked one-commit: +1 follows the commit. */
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
		if (ret != 0) {
			node->prev = NULL;
			if (ic.txn)
				ft_flip_txn_destroy(ic.txn);
		} else if (ic.slot) {
			/*
			 * One-commit (list off): commit the recorded structural
			 * publish alone (@cell == NULL), then propagate the
			 * count.  A pure replace does not park (ic.slot unset)
			 * and falls through untouched.
			 */
			ft_insert_one_commit(ft, _key, _key_len, NULL, &ic);
			ft_propagate_external_count_parent(ft,
				ic.count_from ? ic.count_from : *d.pnfp, 1);
		} else if (ic.txn) {
			/* Armed but nothing parked (duplicate append): no
			 * structural publish to commit. */
			ft_flip_txn_destroy(ic.txn);
			ic.txn = NULL;
		}
	} else {
		if (ret != 0) {
			node->prev = NULL;
			ft_ord_cell_free_unpublished(ft, precell);
			if (ic.txn)
				ft_flip_txn_destroy(ic.txn);
		} else if (ft_node_external((struct cds_ft_inode_flag *) node->prev)) {
			/* Duplicate append / -EEXIST (prev NULL): cell orphaned. */
			ft_ord_cell_free_unpublished(ft, precell);
			if (ic.txn)
				ft_flip_txn_destroy(ic.txn);
		} else if (ic.slot) {
			/*
			 * Fresh head, parked one-commit: structural publish +
			 * ordinal-cell splice flip atomically.  A replace
			 * (*old_node_ret set) does not park -- it swapped @precell
			 * into the replaced head's list slot at the replace site,
			 * so it falls through here untouched.
			 */
			ft_insert_one_commit(ft, _key, _key_len, precell, &ic);
			ft_propagate_external_count_parent(ft,
				ic.count_from ? ic.count_from : *d.pnfp, 1);
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
	 * Splice @new_node into @old_node's chain position.  When @old_node is a
	 * chain HEAD with the ordered list on, publish a FRESH cell for @new_node
	 * (its node field set while the cell is hidden -- cell->node stays
	 * WRITE-ONCE, the way the public cds_ft_cell_node reads it as a plain
	 * pointer) and swap it in for @old_node's cell FUSED with the structural
	 * forward publish and -- for a compressed holder -- the grandparent SKIP_X
	 * dual, in ONE flip: a reader never sees @new_node at one index but
	 * @old_node (about to be freed) at another, nor a stale skip target into
	 * @old_node (the dangling-skip UAF).  A non-head duplicate (no cell) is a
	 * single predecessor->next store; a list-off head inherits the flagged
	 * parent and flips the structural + SKIP_X dual.  Mirrors ft_promote_head.
	 */
	{
		bool is_head = !ft_node_external(
			(struct cds_ft_inode_flag *) old_node->prev);
		struct cds_ft_inode_flag *parent_nf = cn ?
			ft_compressed_node_flag(cn) : holder_flag;
		struct ft_ord_cell *old_cell = (ft->ordered_list && is_head) ?
			ft_ord_cell_ptr(old_node->prev) : NULL;
		void *new_cell_flag = NULL;
		struct ft_pub_rec rec = { .n = 0 };
		struct ft_ord_cell_edge sedges[2];
		unsigned int n_s;

		/*
		 * Alloc the fresh cell BEFORE any mutation so OOM aborts cleanly
		 * (@new_node still carries its zeroed links, @old_node is intact).
		 */
		if (old_cell) {
			new_cell_flag = ft_ord_cell_alloc(ft, new_node,
				old_cell->parent);
			if (!new_cell_flag) {
				s = CDS_FT_STATUS_MEMORY_ERROR;
				FT_TP(replace_exit, (int) s);
				return s;
			}
		}
		/* @new_node's successor link is build-invisible (it is fresh). */
		new_node->next = ft_node_next(old_node);
		if (!is_head) {
			/*
			 * Non-head duplicate: the predecessor->next forward link
			 * (old_node -> new_node) is the single reader-visible
			 * publish -> express it as a single-edge flip descriptor.
			 * The successor's prev back-edge is a plain store ordered
			 * before that infallible lone-edge flip.
			 */
			if (new_node->next)
				new_node->next->prev = new_node;
			new_node->prev = old_node->prev;
			ft_chain_next_flip(ft, (struct cds_ft_node **) pub_slot,
				old_node, new_node);
		} else if (old_cell) {
			/*
			 * Head, list on: fresh-cell swap fused with the publish.
			 * The successor's prev is a LIVE settled store (a skip
			 * resolution reads a head's prev raw, so it cannot ride the
			 * flip), so make the flip infallible by pre-reserving its
			 * txn BEFORE that store; on OOM abort with the successor
			 * untouched and the replace retriable.
			 */
			struct ft_ord_cell *new_cell =
				ft_ord_cell_ptr(new_cell_flag);
			struct urcu_txn_sw_txn *txn =
				ft_flip_txn_create_bounded(
					FT_ORD_CELL_SWAP_PUBLISH_MAX_EDGES);

			if (!txn) {
				new_node->next = NULL;
				ft_ord_cell_free_unpublished(ft, new_cell);
				s = CDS_FT_STATUS_MEMORY_ERROR;
				FT_TP(replace_exit, (int) s);
				return s;
			}
			if (new_node->next)
				new_node->next->prev = new_node;
			cds_ft_item_to_metadata(new_cell)->incoming_byte =
				cds_ft_item_to_metadata(old_cell)->incoming_byte;
			/* Fresh @new_node -> cell: build-invisible. */
			new_node->prev = new_cell_flag;
			_ft_publish_to_parent(ft, parent_nf, pub_slot,
				(struct cds_ft_inode_flag *) new_node, &rec);
			n_s = ft_pub_rec_sedges(&rec, sedges);
			ft_ord_cell_swap_publish_multi(ft, old_cell, new_cell,
				sedges, n_s, txn);
			ft_ord_cell_free(ft, old_cell);
		} else {
			/*
			 * Head, list off: no cell; flip the structural + SKIP_X
			 * dual.  The successor's prev back-edge is a LIVE settled
			 * store (uniform pre-reserve: no flip proxy on any prev
			 * slot, matching the list-on head), so make the flip
			 * infallible by PRE-RESERVING its bounded txn BEFORE that
			 * store; on OOM abort with the successor untouched and the
			 * replace retriable (@new_node restored to its fresh state).
			 */
			struct urcu_txn_sw_txn *txn =
				ft_flip_txn_create_bounded(FT_PUB_SEDGE_MAX_EDGES);

			if (!txn) {
				new_node->next = NULL;
				s = CDS_FT_STATUS_MEMORY_ERROR;
				FT_TP(replace_exit, (int) s);
				return s;
			}
			if (new_node->next)
				new_node->next->prev = new_node;
			new_node->prev = old_node->prev;
			_ft_publish_to_parent(ft, parent_nf, pub_slot,
				(struct cds_ft_inode_flag *) new_node, &rec);
			n_s = ft_pub_rec_sedges(&rec, sedges);
			ft_ord_cell_flip_into(ft, txn, sedges, n_s);
		}
	}

	/*
	 * @old_node has left the trie (replaced by @new_node): tombstone it.
	 * Its next pointer is preserved so a concurrent reader positioned on
	 * @old_node still follows the chain.
	 */
	ft_node_mark_removed_flip(ft, old_node);


	/*
	 * The trie structure is unchanged (no recompaction), so the iterator
	 * path remains valid in cached mode.
	 */
	iter_auto_invalidate_cache(iter);
	s = CDS_FT_STATUS_OK;
	FT_TP(replace_exit, (int) s);
	return s;
}
