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
		struct ft_detach_run *run,
		struct ft_flip_txn *txn,
		long count_delta)
{
	/*
	 * @txn is created and reserved by the caller (ft_detach_node), sized for
	 * this publish's structural edges PLUS one freeze-on-free tombstone per
	 * node in the orphaned chain the caller collected below -- so the
	 * sub-case-2 retire here and the caller's entire orphan set freeze
	 * ATOMICALLY with the publish that unlinks them (atomic detach, §4.B).
	 * The caller reserves it before any side-effect (its orphan walk is
	 * read-only), so the pre-flip ft_set_parent here still reaches an
	 * allocation-free point of no return, committing through
	 * ft_ord_cell_flip_into (infallible).  On any failure below the caller
	 * destroys @txn.
	 */
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
		 * Fold the external head's back-edge -- cell->parent (list on) or
		 * its prev (list off) -- INTO @txn so it commits ATOMICALLY with
		 * cn->child below, rather than as a bare store racing ahead of the
		 * publish.  A skip pointer at cn's grandparent slot resolves through
		 * cn->child = topmost_external_nodes, and ft_skip_to_compressed walks
		 * topmost->prev to recover cn; the atomic commit means skip-recovery
		 * never observes cn->child = topmost while topmost's back-edge still
		 * names the about-to-be-detached old holder.  Both slots resolve a
		 * flip-proxy on the read side (cell->parent via ft_resolve_head_prev,
		 * prev via ft_dereference_prev_resolved), so the folded descriptor is
		 * concurrent-reader-safe.  The +1 edge was reserved by the caller
		 * (ft_detach_node) when topmost_external_nodes is set.
		 */
		{
			struct cds_ft_inode_flag *cn_flag =
				ft_compressed_node_flag(cn);

			if (ft->ordered_list) {
				struct ft_ord_cell *cell = ft_ord_cell_ptr(
					topmost_external_nodes->prev);

				ft_flip_txn_record_reserved(txn,
					(void **) &cell->parent,
					cell->parent, cn_flag);
			} else {
				ft_flip_txn_record_reserved(txn,
					(void **) &topmost_external_nodes->prev,
					topmost_external_nodes->prev, cn_flag);
			}
		}
		/*
		 * nr_keys fold (LEAF Increment 2): the removed leaf's -1 walk from
		 * the kept compressed node @cn (which STAYS in place; its subtree
		 * loses the disappearing key) up to root rides THIS promote commit
		 * atomically -- recorded before the publish dispatch below so both
		 * commit arms carry it.  Reserved by the caller's count_reserve; a
		 * no-op (and skipped) for a count-neutral move detach / !rank_stats.
		 */
		if (count_delta)
			ft_flip_txn_record_count_parent(ft, txn,
				ft_compressed_node_flag(cn), count_delta);
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

			/* VALIDATE (§4.B): guard the LIVE kept compressed node cn. */
			ft_flip_txn_guard_parent(ft, txn, ft_compressed_node_flag(cn));
			_ft_publish_to_parent(ft, ft_compressed_node_flag(cn),
				&cn->child,
				(struct cds_ft_inode_flag *) topmost_external_nodes,
				&rec);
			if (ft_remove_commit_rec(ft, &rec, fuse_cell, run,
					txn) > 0)
				/* Peer won: nothing installed (txn consumed). */
				return -EAGAIN;
			pub->armed = true;
		} else {
			struct ft_pub_rec rec = { .n = 0 };

			/*
			 * Non-fused external-promote (no cell to unsplice -- list
			 * off, or fusion not requested / already armed): still route
			 * the publish through the op flip-txn so the forward slot AND
			 * a compressed grandparent's SKIP_X dual flip atomically (no
			 * torn forward/skip window).  A lone edge reduces to a single
			 * release store (ft_ord_cell_flip_one), so this stays allocation-
			 * free and infallible for the common plain-parent case.
			 */
			/* VALIDATE (§4.B): guard the LIVE kept compressed node cn. */
			ft_flip_txn_guard_parent(ft, txn, ft_compressed_node_flag(cn));
			_ft_publish_to_parent(ft, ft_compressed_node_flag(cn),
				&cn->child,
				(struct cds_ft_inode_flag *) topmost_external_nodes,
				&rec);
			if (ft_remove_commit_rec(ft, &rec, NULL, NULL, txn) > 0)
				/* Peer won: nothing installed (txn consumed). */
				return -EAGAIN;
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
			return -ENOMEM;	/* nothing published: caller destroys @txn */
		src_meta = cds_ft_item_to_metadata(
			(struct cds_ft_inode *) ft_compressed_node_ptr(
				iter_node_flag));
		fresh_meta->parent = src_meta->parent;
		/*
		 * nr_keys fold (LEAF Increment 2): the fresh internal REPLACES the
		 * retired compressed node, so it carries the retired node's
		 * post-removal count (get(src) + count_delta, i.e. src - 1 when this
		 * detach retires a key -- typically 0, a compressed root emptied of
		 * its lone leaf).  A build-invisible plain store on the not-yet-
		 * published fresh node; a no-op when !rank_stats.  The -1 walk from
		 * the fresh node's stable parent rides the publish below.
		 */
		if (count_delta)
			ft_nr_keys_store(ft, fresh_meta,
				ft_nr_keys_get(src_meta) + count_delta, CMM_RELAXED);
#ifdef FEATURE_FT_SKIP_COMPRESSED
		ft_meta_parent_slot_offset_set(fresh_meta,
			ft_meta_parent_slot_offset(src_meta));
#endif
		{
			struct ft_pub_rec rec = { .n = 0 };

			/*
			 * Compressed -> fresh-internal recompaction publish: route
			 * the forward slot (+ a compressed grandparent's SKIP_X dual)
			 * through the op flip-txn so they flip atomically; lone edge
			 * stays a single release store.
			 */
			/* VALIDATE (§4.B): guard the LIVE grandparent src_meta->parent. */
			ft_flip_txn_guard_parent(ft, txn, src_meta->parent);
			_ft_publish_to_parent(ft, src_meta->parent,
				detach_parent_flag_ptr,
				ft_node_flag(fresh, 0), &rec);
			/*
			 * Freeze the retired compressed node dead (§4.B freeze-on-
			 * free): this compressed->fresh-internal recompaction retires
			 * the old compressed node like every other retire.  Record the
			 * tombstone INTO @txn so it flips ATOMICALLY with the publish
			 * that unlinks it -- an aborted publish leaves it live (atomic
			 * detach).  This detach sub-case is not reached by the current
			 * test suite (verified: zero hits across ft_unit + ft_inv both
			 * list modes), so the freeze-on-free audit never exercises it.
			 */
			ft_flip_txn_record_tombstone(txn, cds_ft_item_to_metadata(
				(struct cds_ft_inode *) ft_compressed_node_ptr(
					iter_node_flag)));
			/*
			 * nr_keys fold (LEAF Increment 2): the removed leaf's -1
			 * walk from the fresh node's stable parent up to root rides
			 * this publish (the fresh node itself was baked above).  A
			 * compressed ROOT retire leaves src_meta->parent == NULL, so
			 * the walk records no ancestor edge and only the baked root
			 * count changes.
			 */
			if (count_delta)
				ft_flip_txn_record_count_parent(ft, txn,
					src_meta->parent, count_delta);
			if (ft_remove_commit_rec(ft, &rec, NULL, NULL, txn) > 0) {
				/*
				 * Peer won: the fresh internal never published;
				 * the retired compressed node stays live and
				 * linked (its tombstone was discarded with the
				 * aborted commit).
				 */
				free_cds_ft_node_unpublished(ft, fresh);
				return -EAGAIN;
			}
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
 *   - iter_meta nr_child == 1
 *   - @iter_meta->external_nodes == NULL
 *   - @iter_meta->parent != NULL  (non-root)
 *
 * @iter_node_flag: the 1-child internal being collapsed.
 * @iter_meta:      its metadata.
 * @slot_ptr:       parent slot pointing to @iter_node_flag.  Used only
 *                  when the parent is non-compressed; compressed parents
 *                  resolve their own slot via parent_slot_offset.
 */
#endif	/* FEATURE_FT_SKIP_COMPRESSED */
/*
 * Freeze-on-free (doc §4.B) for the ft_detach_node branch-2 orphan chain: mark
 * every collected orphan (+ the trailing skip-target) DEAD.  This helper is
 * config-agnostic (the branch-2 orphan chain forms with or without skip
 * compression; ft_skip_to_compressed has an unconditional non-skip stub), and
 * it is called from a path (ft_detach_node) that is reachable outside the
 * skip block, so it lives OUTSIDE the FEATURE_FT_SKIP_COMPRESSED guard.  @txn
 * non-NULL
 * records each tombstone INTO the op's commit txn so the freeze flips
 * ATOMICALLY with the flip that unlinks the chain (atomic detach); @txn NULL
 * falls back to a standalone lone-edge flip (the list-off pub-less lone-store
 * path, whose unlink is not a txn commit -- a no-op under one writer).  Node
 * resolution mirrors the branch-2 free loop.
 */
static
void ft_detach_freeze_orphans(struct cds_ft *ft, struct ft_flip_txn *txn,
		struct cds_ft_inode_flag **orphans, int nr_orphans,
		struct cds_ft_inode_flag *trailing_skip_cn_flag)
{
	int i;

	for (i = 0; i < nr_orphans; i++) {
		struct cds_ft_metadata *m = ft_node_compressed(orphans[i])
			? cds_ft_item_to_metadata((struct cds_ft_inode *)
				ft_compressed_node_ptr(orphans[i]))
			: cds_ft_item_to_metadata(ft_node_ptr(orphans[i]));

		if (txn)
			ft_flip_txn_record_tombstone(txn, m);
		else
			ft_meta_tombstone_set_flip(m);
	}
	if (trailing_skip_cn_flag) {
		struct cds_ft_metadata *m = cds_ft_item_to_metadata(
			(struct cds_ft_inode *) ft_skip_to_compressed(ft,
				trailing_skip_cn_flag));

		if (txn)
			ft_flip_txn_record_tombstone(txn, m);
		else
			ft_meta_tombstone_set_flip(m);
	}
}
#ifdef FEATURE_FT_SKIP_COMPRESSED
/*
 * ft_chain_compress_fused: the fused-merge primitive behind the
 * chain-compress canonicalization.  Builds the merged compressed @new_cn
 * INVISIBLY (rcu-mutation build phase) from the caller-supplied surviving
 * child, then commits the whole transition -- forward slot publish + the
 * surviving child's LIVE re-parent back-edge + a compressed grandparent's
 * SKIP_X dual + @dead_cell / @run's ordered-cell unsplice -- in ONE flip.
 *
 * Because the merge IS the build-invisible commit, the key-disappearing
 * removal that triggers it can publish through this single flip directly
 * (no intermediate recompacted / in-place node, no transient non-canonical
 * state ever): the merged @new_cn REPLACES the boundary node, subsuming the
 * boundary's child-slot clear (shape D) or external_nodes clear (shape P).
 *
 * @surviving_child / @surviving_byte: the boundary's sole post-removal child
 * and its incoming byte, computed by the caller from PRE-commit state (so
 * this runs as the removal's commit, not a second flip after it).  Both are
 * stable across the removal: the merge never rebuilds the parent or the
 * surviving child.
 * @plan_nr_child: the boundary's child count the caller's plan assumed (2 for
 * the shape-D fold that retires one of the two, 1 for the post-removal
 * collapses) -- re-validated against the boundary's COPYING-fence snapshot
 * together with the @surviving_byte -> @surviving_child mapping, so a peer
 * commit between the caller's derivation and the fence mark surfaces as
 * -EAGAIN instead of a merge built from a stale plan.
 * @dead_cell / @run: the dead key's ordered-list unsplice, folded into the
 * merge flip (NULL when the ordered list is off / no fusion).
 * @orphans / @nr_orphans / @trailing_orphan: an optional orphan chain the SAME
 * unlink strands (ft_detach_node's branch-2 prune); each node's freeze-on-free
 * tombstone is recorded INTO the merge flip so it freezes atomically with the
 * collapse.  Pass NULL/0/NULL for the standalone canonicalize / chain-leaf
 * callers (no extra chain retired by their flip); the txn reservation grows to
 * cover them.
 *
 * Returns 0 when the merge committed; -ENOMEM when the txn or @new_cn could
 * not be allocated (NOTHING was published -- the caller aborts the whole
 * removal, structure untouched); -EAGAIN on a peer conflict (a dirty COPYING
 * mark, a failed plan re-validation, a parked latch in a captured slot, or a
 * commit ABORT -- nothing installed, every fence cleared, the caller's retry
 * re-descends); a positive value when the merge does not apply because the
 * merged length would exceed the compressed-node bound (caller falls back to
 * the non-fused removal, leaving the boundary 1-child internal as before).
 * The replaced nodes (@boundary, @parent_cn, @child_cn) are enumerated
 * explicitly at the commit so a future MCAS can fold each one's sequence
 * counter into the same transaction.
 */
static
int ft_chain_compress_fused(struct cds_ft *ft,
		struct cds_ft_inode_flag *iter_node_flag,
		struct cds_ft_metadata *iter_meta,
		struct cds_ft_inode_flag **slot_ptr,
		struct cds_ft_inode_flag *surviving_child,
		uint8_t surviving_byte,
		unsigned int plan_nr_child,
		struct ft_ord_cell *dead_cell,
		struct ft_detach_run *run,
		struct cds_ft_inode_flag **orphans,
		int nr_orphans,
		struct cds_ft_inode_flag *trailing_orphan,
		struct cds_ft_node *freeze_leaf,
		long count_delta,
		unsigned int count_reserve)
{
	struct cds_ft_compressed_node *parent_cn, *child_cn;
	struct cds_ft_metadata *parent_cn_meta;
	unsigned int parent_len, child_len, merged_len;
	struct cds_ft_metadata *new_cn_meta;
	struct cds_ft_compressed_node *new_cn;
	struct cds_ft_inode_flag *new_cn_flag;
	struct cds_ft_inode_flag **publish_slot;
	struct cds_ft_inode_flag *publish_parent;
	struct cds_ft_inode_flag *iter_parent;
	struct ft_flip_txn *txn;
	uintptr_t s_iter, s_pcn = 0, s_ccn = 0;

	assert(surviving_child);
	/*
	 * Pre-reserve the commit flip-txn BEFORE any pre-flip side-effect.  The
	 * surviving child's (parent, parent-slot-offset) pair is RECORDED into
	 * @txn by ft_pub_rec_add_back_edge below (ft_reparent_record_meta: both
	 * edges co-committed), so an aborted flip discards the pair coherently
	 * -- no settled offset survives against the still-old parent.  With the
	 * txn reserved the publish commits through ft_ord_cell_flip_into without
	 * allocating, so the only OOM points are this reservation and the new_cn
	 * allocation, both BEFORE the build's first side-effect (a concurrent-
	 * writer ABORT surfaces as -EAGAIN with new_cn reclaimed, see the commit
	 * site below).  Created FIRST (its sizing needs only the caller's
	 * arguments) so the COPYING fences below can register with it: every
	 * bail from here on is a ft_flip_txn_destroy or the commit itself, and
	 * both terminal paths drain the fence registry -- no unwind can leak a
	 * fence.
	 */
	txn = ft_flip_txn_create_bounded(FT_REMOVE_COMMIT_REC_MAX_EDGES + 3
			+ 1 /* §4.B parent guard */
			+ 1 /* back-edge (parent, offset) pair: the state-word edge */
			+ nr_orphans + (trailing_orphan ? 1 : 0)
			+ (freeze_leaf ? FT_HLIST_FREEZE_MAX_EDGES : 0)
			+ count_reserve /* nr_keys walk from publish_parent (R3 fold) */);
	if (!txn)
		return -ENOMEM;	/* nothing touched: caller aborts */
	/*
	 * F2 COPYING fence (doc at ft_meta_copying_mark): fence the whole
	 * collapsed chain -- the boundary and the two compressed nodes whose
	 * bodies the merged node subsumes -- BEFORE trusting any of their
	 * mutable state.  Each mark's clean snapshot feeds that node's
	 * tombstone expected-old at the commit, so a peer state change under
	 * any fence aborts exactly one side.  A dirty mark (peer proxy, a
	 * concurrent copier, a real retire -- e.g. a peer already re-published
	 * the surviving child and tombstoned the old copy this plan captured)
	 * bails to the caller's retry.
	 */
	if (ft_meta_copying_mark(iter_meta, &s_iter)) {
		ft_flip_txn_destroy(txn);
		return -EAGAIN;
	}
	ft_flip_txn_copying_register(txn, iter_meta);
	/*
	 * Re-validate the caller's PRE-fence plan under the fence: the
	 * boundary must still have the child population the plan was derived
	 * from (@plan_nr_child: 2 for the shape-D fold that retires one of the
	 * two, 1 for the post-removal collapses), and @surviving_byte must
	 * still map to @surviving_child -- a peer commit between the caller's
	 * derivation and the mark (an insert into the boundary, a child
	 * republish) is exactly what the mark snapshot cannot vouch for.
	 * Never fires single-writer.
	 */
	if (caa_unlikely(ft_state_nr_child(s_iter) != plan_nr_child ||
			ft_node_get_nth(ft, iter_node_flag, NULL,
				surviving_byte, FT_PF_NONE)
					!= surviving_child)) {
		ft_flip_txn_destroy(txn);
		return -EAGAIN;
	}
	/*
	 * ONE snapshot of the boundary's parent (latch-checked): the F1
	 * discipline -- a peer's parked flip proxy must be neither classified
	 * nor embedded.
	 */
	iter_parent = (struct cds_ft_inode_flag *)
		rcu_dereference(iter_meta->parent);
	if (caa_unlikely(ft_node_flip_proxy(iter_parent))) {
		ft_flip_txn_destroy(txn);
		return -EAGAIN;
	}
	parent_cn = ft_node_compressed(iter_parent)
		? ft_compressed_node_ptr(iter_parent)
		: NULL;
	parent_cn_meta = parent_cn
		? cds_ft_item_to_metadata((struct cds_ft_inode *) parent_cn)
		: NULL;
	if (parent_cn_meta && ft_meta_copying_mark(parent_cn_meta, &s_pcn)) {
		ft_flip_txn_destroy(txn);
		return -EAGAIN;
	}
	if (parent_cn_meta)
		ft_flip_txn_copying_register(txn, parent_cn_meta);
	child_cn = ft_node_compressed(surviving_child)
		? ft_compressed_node_ptr(surviving_child)
		: NULL;
	if (child_cn && ft_meta_copying_mark(
			cds_ft_item_to_metadata((struct cds_ft_inode *) child_cn),
			&s_ccn)) {
		ft_flip_txn_destroy(txn);
		return -EAGAIN;
	}
	if (child_cn)
		ft_flip_txn_copying_register(txn,
			cds_ft_item_to_metadata((struct cds_ft_inode *) child_cn));
	parent_len = parent_cn ? parent_cn->len : 0;
	child_len = child_cn ? child_cn->len : 0;
	merged_len = parent_len + 1 + child_len;

	if (merged_len > FT_SKIP_LEN_MAX) {
		/* Merge does not apply: caller falls back (fences cleared). */
		ft_flip_txn_destroy(txn);
		return 1;
	}
	new_cn = alloc_compressed_node(ft, merged_len, &new_cn_meta);
	if (!new_cn) {
		ft_flip_txn_destroy(txn);	/* PREPARE state: no grace period */
		return -ENOMEM;	/* nothing published: caller aborts */
	}

	/* Compose merged path bytes. */
	if (parent_cn)
		memcpy(new_cn->key_bytes, parent_cn->key_bytes, parent_len);
	new_cn->key_bytes[parent_len] = surviving_byte;
	if (child_cn)
		memcpy(&new_cn->key_bytes[parent_len + 1],
			child_cn->key_bytes, child_len);
	new_cn->len = (uint8_t) merged_len;
	if (child_cn) {
		struct cds_ft_inode_flag *child_child =
			rcu_dereference(child_cn->child);

		/*
		 * A peer latch parked on the boundary cn's child must not be
		 * embedded in the merged cn (the peer settles only the
		 * ORIGINAL slot; an embedded copy dangles into its reclaimed
		 * descriptor -- see the recompact copy-loop bail).  Nothing
		 * recorded into @txn yet and new_cn never published: reclaim
		 * both and let the caller retry after the peer settles.
		 */
		if (caa_unlikely(ft_node_flip_proxy(child_child))) {
			free_compressed_node_unpublished(ft, new_cn);
			ft_flip_txn_destroy(txn);
			return -EAGAIN;
		}
		new_cn->child = child_child;
	} else {
		new_cn->child = surviving_child;
	}
	ft_meta_nr_child_set(new_cn_meta, 1);
	/*
	 * Build the merged node with its POST-removal count (top + @count_delta,
	 * i.e. top - 1 when this merge retires a key -- R3 fold): the collapsed
	 * top node's count minus the disappearing key.  @count_delta 0 (a
	 * count-neutral canonicalize, or a leaf-detach whose -1 the caller still
	 * pre-decrements) copies it verbatim, as before.  No-op when !rank_stats.
	 */
	ft_nr_keys_store(ft,new_cn_meta,
		ft_nr_keys_get(parent_cn_meta
			? parent_cn_meta
			: iter_meta) + count_delta,
		CMM_RELAXED);

	if (parent_cn) {
		/*
		 * Replace parent_cn at its own slot in the grandparent.
		 * Inherit grandparent context from parent_cn as ONE consistent
		 * (parent, slot) snapshot (Phase 4.3 atomic re-home): a peer
		 * re-homing parent_cn commits its parent and state-word offset
		 * atomically, so two raw reads could tear across that commit.
		 */
		publish_slot = ft_resolve_parent_slot(parent_cn_meta, ft,
			&publish_parent);
		new_cn_meta->parent = publish_parent;
	} else {
		/* The boundary's own latch-checked parent snapshot above. */
		new_cn_meta->parent = iter_parent;
		publish_parent = iter_parent;
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
		 * back-edge to resolve a SKIP_X forward flag).  The commit rides
		 * the pre-reserved @txn (ft_ord_cell_flip_into), so it is
		 * allocation-free past this point and cannot fail.
		 */
		ft_pub_rec_add_back_edge(ft, &rec, txn, new_cn->child,
			new_cn_flag, &new_cn->child);
		new_cn_pub = ft_publish_compressed(ft, new_cn, new_cn_flag);
		/* VALIDATE (§4.B): guard the LIVE (great-)grandparent publish_parent. */
		ft_flip_txn_guard_parent(ft, txn, publish_parent);
		_ft_publish_to_parent_meta(ft, publish_parent, publish_slot,
			new_cn_pub, new_cn_meta, NULL, &rec);
		/*
		 * Freeze-on-free (doc §4.B, atomic detach): the collapsed chain
		 * this commit retires -- the 1-child boundary @iter_node_flag and
		 * the old parent/child compressed nodes new_cn merges (<=3) --
		 * gets its one-way LIVE->DEAD tombstone RECORDED INTO @txn (the +3
		 * reserved above), so the freeze flips ATOMICALLY with the commit
		 * that unlinks it: an aborted commit leaves every node live.  A
		 * no-op under one writer.  All three are FENCED (marked COPYING
		 * above), so each expected old is its mark's clean snapshot: the
		 * commit ratifies exactly the chain state this merge was built
		 * from, and {COPYING|s -> TOMBSTONE|s} consumes the fence.
		 */
		ft_flip_txn_record_tombstone_copying(txn, iter_meta, s_iter);
		if (parent_cn)
			ft_flip_txn_record_tombstone_copying(txn, parent_cn_meta,
				s_pcn);
		if (child_cn)
			ft_flip_txn_record_tombstone_copying(txn,
				cds_ft_item_to_metadata(
					(struct cds_ft_inode *) child_cn),
				s_ccn);
		ft_detach_freeze_orphans(ft, txn, orphans, nr_orphans,
			trailing_orphan);
		/*
		 * The removed external leaf (a single-entry chain, so
		 * freeze_leaf->next == NULL) freezes atomically with this same
		 * commit that retires its holder chain (doc §4.B): one MARK(NULL)
		 * edge, the +1 reserved above.  NULL when the caller is not
		 * retiring a leaf through this merge.
		 */
		if (freeze_leaf)
			ft_hlist_freeze_prepare(ft_flip_txn_handle(txn), freeze_leaf);
		/*
		 * R3 fold: the retired key's -1 walk from the merged node's stable
		 * parent (publish_parent) up to root rides THIS commit atomically
		 * with the collapse (the merged node itself was built -1 above).
		 * Reserved by @count_reserve; skipped (and a no-op) when there is no
		 * count change.
		 */
		if (count_delta)
			ft_flip_txn_record_count_parent(ft, txn, publish_parent,
				count_delta);
		if (ft_remove_commit_rec(ft, &rec, dead_cell, run, txn) > 0) {
			/*
			 * Peer won: NOTHING installed -- the collapsed chain
			 * (boundary + parent_cn/child_cn) is still live and
			 * linked (its tombstones, freezes, and count edges were
			 * discarded with the aborted commit).  Only the
			 * never-published merged node is ours to reclaim; the
			 * caller retries and re-plans against the current tree.
			 */
			free_compressed_node_unpublished(ft, new_cn);
			return -EAGAIN;
		}
	}

	free_cds_ft_node(ft, ft_node_ptr(iter_node_flag));
	if (parent_cn)
		free_compressed_node(ft, parent_cn);
	if (child_cn)
		free_compressed_node(ft, child_cn);
	return 0;
}

/*
 * Post-removal chain-compress prune as a standalone second flip: compute the
 * surviving child from the (already-committed) 1-child boundary and fuse no
 * ordered-cell edge.  The fused-merge primitive above is the preferred path
 * (the prune rides the removal's own flip); this thin wrapper remains for the
 * out-of-bound (>FT_SKIP_LEN_MAX) residue case, where the boundary stays a
 * 1-child internal and an allocation failure is silently skipped (the prune
 * is best-effort once the removal has already published).
 */
static
void ft_canonicalize_chain_compress(struct cds_ft *ft,
		struct cds_ft_inode_flag *iter_node_flag,
		struct cds_ft_metadata *iter_meta,
		struct cds_ft_inode_flag **slot_ptr)
{
	uint8_t surviving_byte = 0;
	struct cds_ft_inode_flag *surviving_child;

	surviving_child = ft_node_get_minmax(ft, iter_node_flag,
		&surviving_byte, FT_LEFTMOST,
		false /* writer; no validation */);
	if (!surviving_child)
		return;
	(void) ft_chain_compress_fused(ft, iter_node_flag, iter_meta,
		slot_ptr, surviving_child, surviving_byte,
		1 /* already-committed 1-child boundary */, NULL, NULL,
		NULL, 0, NULL, NULL, 0 /* count-neutral canonicalize */, 0);
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
		struct ft_detach_run *run,
		struct ft_glue *retire_glue,
		struct cds_ft_node *freeze_leaf,
		long count_delta)
{
	struct cds_ft_metadata *metadata_stack[FT_MAX_DEPTH];
	struct cds_ft_inode_flag *iter_node_flag;
	struct cds_ft_inode *old_recompacted_node = NULL;
	int ret, nr_metadata = 0, nr_clear = 0, nr_branch = 0;
	uint8_t n = 0;
	/*
	 * Pre-reserved commit flip-txn for the plain-branch key-removal commit
	 * (in-place delete OR recompaction publish), reserved before
	 * ft_node_replace_ptr's pre-flip side-effects (nr_child-- / eager child
	 * re-parent) when that commit will be multi-edge (a cell unsplice, or a
	 * compressed-parent SKIP_X dual) -- a lone edge stays the infallible
	 * on-stack release store.  Consumed by ft_ord_cell_flip_into at whichever
	 * commit fires; freed at @end if reserved but unused.
	 */
	struct ft_flip_txn *commit_txn = NULL;
	bool commit_txn_used = false;
	/*
	 * Branch-2 (non-compressed parent) orphan chain, freed AFTER the commit
	 * that unlinks it -- below the recompaction republish, not in the
	 * pre-republish !ret block.  A recompaction defers its forward commit
	 * (@commit_txn) to the parent-slot block far below, so deferring the free
	 * to there lets every freed node's freeze-on-free tombstone (recorded INTO
	 * @commit_txn) flip ATOMICALLY with that unlink -- and frees only after the
	 * structural unlink, never before it.  The set is copied out of the
	 * branch-2 to_free[] on a successful detach; @free_orphans_pending gates
	 * the deferred walk.
	 */
	struct cds_ft_inode_flag *orphan_free[FT_MAX_DEPTH];
	int nr_orphan_free = 0;
	struct cds_ft_inode_flag *orphan_trailing = NULL;
	bool free_orphans_pending = false;
	bool retire_glue_fused = false;
	bool freeze_leaf_fused = false;
	struct cds_ft_node *topmost_external_nodes = NULL;
	bool prev_external_nodes_found = false;
	/*
	 * Set when the pure-delete leaves the surviving boundary a non-root
	 * 1-child no-external internal whose SKIP_X canonical form is a merged
	 * compressed node: the chain-compress prune is then FUSED into the
	 * key-removal commit (build the merged node, replace the boundary in ONE
	 * flip), bypassing ft_node_replace_ptr and the standalone post-detach
	 * canonicalize -- no intermediate recompacted/in-place node, no transient
	 * non-canonical state.  See ft_chain_compress_fused.
	 */
	bool boundary_fused = false;
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
	/*
	 * nr_keys count fold (LEAF Increment 2): the removed leaf's @count_delta
	 * (-1, or 0 for a count-neutral move detach) rides the SAME commit that
	 * unlinks the leaf, rather than a standalone pre-decrement walk.  Each
	 * per-outcome fold below records the -1 walk from that outcome's stable
	 * base into its commit txn (in-place / recompaction: @commit_txn;
	 * shape-D: ft_chain_compress_fused's own txn; compressed parent:
	 * @orphan_txn) and sets @count_folded; the residual for a lone-store
	 * (list-off pub-less) outcome that reaches no txn falls back to the
	 * standalone walk at the end.  All no-ops when !rank_stats.
	 */
	bool count_folded = false;
	/*
	 * nr_keys fold (list-off residual close): force the list-off lone forward
	 * store through a commit txn so the count folds ATOMICALLY with the unlink.
	 * When the caller supplies no @pub (list-off: there is no ordered-list cell
	 * to fuse, so the in-place delete / external promote would store bare via
	 * ft_node_replace_ptr and leave its -@count_delta walk to a standalone
	 * post-commit root-ward RMW), but order-statistics are
	 * on, defer that store into a LOCAL pub instead: the pub-armed arm below
	 * commits it via ft_remove_one_commit and records the -@count_delta walk
	 * from the surviving holder @iter_node_flag into the SAME commit (the fold
	 * at the `pub->armed` block), exactly as the list-on fused path already
	 * does.  This is the single choke point for every ft_detach_node caller
	 * (leaf remove, remove_all, whole-subtree detach, merge src-unlink,
	 * graft_swap extract) -- each passes pub == NULL in list-off with a signed
	 * @count_delta, and each folds here uniformly.  A no-op when !rank_stats:
	 * @pub stays NULL and the bare lone-edge store is byte-identical (the
	 * count walk is irrelevant without order-statistics).  @local_pub outlives
	 * every use (whole-function scope); ft_detach_node never returns @pub.
	 */
	struct ft_remove_pub local_pub = { .armed = false };

	if (!pub && ft->rank_stats)
		pub = &local_pub;

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

	/*
	 * nr_keys count fold (LEAF Increment 2): no standalone pre-decrement
	 * here anymore -- each per-outcome commit below folds the @count_delta
	 * walk onto its own flip so the count flips ATOMICALLY with the unlink
	 * (exact under concurrent writers).  See @count_folded.
	 */

	while (cur) {
		struct cds_ft_metadata *metadata;
		bool is_root;

		metadata = cds_ft_item_to_metadata(ft_node_ptr(cur));
		metadata_stack[nr_metadata++] = metadata;
		is_root = (metadata->parent == NULL);

		assert(ft_meta_nr_child(metadata) > 0);
		if (!prev_external_nodes_found && (ft_meta_nr_child(metadata) == 1 && !metadata->external_nodes && !is_root)) {
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
		if (prev_external_nodes_found || ft_meta_nr_child(metadata) > 1 ||
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
		struct cds_ft_inode_flag *to_free[FT_MAX_DEPTH];
		int nr_to_free = 0, fi;
		struct cds_ft_compressed_node *trailing_skip_cn = NULL;

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
		 *
		 * Freeze-on-free (doc §4.B, atomic detach): COLLECT the orphaned
		 * chain HERE; the per-node freeze-on-free tombstones are RECORDED
		 * into the replace's publish txn just below and flip ATOMICALLY
		 * with the commit that unlinks the chain, so a concurrent MCAS
		 * writer targeting any of these nodes validates the mark (expected
		 * = live) and fails -- and an aborted replace leaves them live and
		 * unmarked (no stranded dead-but-linked node).  The walk is
		 * read-only on the orphan subtree, whose internal pointers the
		 * replace does NOT touch (it rewrites the parent slot + promotes
		 * the target's external_nodes, tolerated by the phase2_first
		 * special-case), so the set it builds is identical pre/post commit;
		 * the actual free is deferred to after the commit.
		 */
		if (free_detached_subtree) {
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
					nr_child = ft_meta_nr_child(cm);
					ext_nodes = cm->external_nodes;
					next = cn->child;
				} else {
					struct cds_ft_metadata *m =
						cds_ft_item_to_metadata(
							ft_node_ptr(walk_nf));

					nr_child = ft_meta_nr_child(m);
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
				trailing_skip_cn =
					ft_skip_to_compressed(ft, walk_nf);
		}
		/*
		 * Atomic detach (§4.B): create the publish txn sized for the
		 * replace's structural edges PLUS one freeze-on-free tombstone per
		 * collected orphan (+ the trailing skip-target), and record each so
		 * the whole retired chain freezes ATOMICALLY with the replace
		 * commit that unlinks it.  create_bounded mallocs the exact cap;
		 * OOM aborts before any reader-visible store (the orphan walk above
		 * is read-only), and on the replace failing we destroy it here.
		 */
		{
			struct ft_flip_txn *orphan_txn =
				ft_flip_txn_create_bounded(
					FT_REMOVE_COMMIT_REC_MAX_EDGES
					+ 1 /* §4.B parent guard (Sites 3+4 excl.) */
					+ nr_to_free
					+ (trailing_skip_cn ? 1 : 0)
					+ (topmost_external_nodes ? 1 : 0) /* folded external back-edge */
					+ (ft->rank_stats ? detach_depth + 1 : 0) /* nr_keys fold walk */
					+ (freeze_leaf ? FT_HLIST_FREEZE_MAX_EDGES : 0));

			if (!orphan_txn) {
				ret = -ENOMEM;
				goto end;
			}
			for (fi = 0; fi < nr_to_free; fi++)
				ft_flip_txn_record_tombstone(orphan_txn,
					cds_ft_item_to_metadata(ft_node_compressed(
						to_free[fi])
						? (struct cds_ft_inode *)
						  ft_compressed_node_ptr(
							ft_skip_child_ptr(to_free[fi]))
						: ft_node_ptr(to_free[fi])));
			if (trailing_skip_cn)
				ft_flip_txn_record_tombstone(orphan_txn,
					cds_ft_item_to_metadata(
						(struct cds_ft_inode *)
						trailing_skip_cn));
			/*
			 * The removed external leaf freezes atomically with this
			 * compressed-parent replace that unlinks its chain (doc §4.B):
			 * one MARK edge into @orphan_txn, the +1 reserved above.
			 */
			if (freeze_leaf) {
				ft_hlist_freeze_prepare(ft_flip_txn_handle(orphan_txn),
					freeze_leaf);
				freeze_leaf_fused = true;
			}
			ret = ft_detach_node_replace_compressed_parent(ft,
				iter_node_flag, detach_parent_flag_ptr,
				topmost_external_nodes, &nr_clear, fuse_cell,
				pub, run, orphan_txn, count_delta);
			if (ret) {
				/*
				 * -EAGAIN: the commit CONSUMED @orphan_txn (a
				 * peer won mid-flip); only a pre-commit failure
				 * (-ENOMEM before the commit) leaves it live.
				 */
				if (ret != -EAGAIN)
					ft_flip_txn_destroy(orphan_txn);
				goto end;
			}
		}
		/* Orphan chain unlinked: free the set collected above. */
		if (free_detached_subtree) {
			if (trailing_skip_cn)
				free_compressed_node(ft, trailing_skip_cn);
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
		struct cds_ft_inode_flag *to_free[FT_MAX_DEPTH];
		int nr_to_free = 0, fi;
		struct cds_ft_inode_flag *trailing_skip_cn_flag = NULL;

		/*
		 * Density was already propagated above (before
		 * structural changes).
		 */
		/*
		 * Freeze-on-free (doc §4.B): COLLECT the orphaned detach subtree
		 * and tombstone every node HERE, before ANY of this branch's
		 * commits unlinks it -- the shape-D ft_chain_compress_fused below,
		 * or ft_node_replace_ptr / ft_remove_one_commit further down.  The
		 * walk is read-only on the orphan subtree, whose internal pointers
		 * none of those commits touch (they rewrite the surviving boundary
		 * + its parent slot; the detach target's own external_nodes clear /
		 * topmost promote is tolerated by the phase2_first special-case), so
		 * the set it builds is identical pre/post commit.  The free is
		 * deferred to after the commit (the loop in the !ret block below
		 * consumes @to_free / @trailing_skip_cn_flag).  (On an OOM abort
		 * these still-live nodes are left marked dead -- a no-op under one
		 * writer, corrected by a later successful detach.)
		 *
		 * Free walk semantics:
		 *
		 *   Phase 1 -- elevated ancestors (always single-child no-external
		 *   by the upward walk's own invariant).  Free @nr_clear nodes
		 *   unconditionally.
		 *
		 *   Phase 2 -- target and chain below.  For destroy-style detach,
		 *   walk the target's single-child no-external chain (descent
		 *   tracking guarantees this) until we hit an external, a
		 *   multi-child node, a node with external_nodes (its content was
		 *   either preserved as topmost_external_nodes or still referenced),
		 *   or nr_child == 0.  For move-style, stop -- the target is the new
		 *   trie's root and must be preserved.
		 *
		 * Pointer classification: ft_node_external() is an external leaf
		 * (caller-reclaimed); ft_node_compressed() covers plain + high-bit
		 * skip-compressed; otherwise an internal node.
		 */
		{
			struct cds_ft_inode_flag *walk_nf = elevated_old_child;

			/* Phase 1: elevated ancestors. */
			while (nr_to_free < nr_clear &&
			       walk_nf &&
			       !ft_node_external(walk_nf) &&
			       nr_to_free < FT_MAX_DEPTH) {
				struct cds_ft_inode_flag *next = NULL;

				if (ft_node_skip_compressed(walk_nf)) {
					/*
					 * Skip-encoded elevated link: the slot value
					 * encodes the child BELOW the elided skip-target
					 * compressed node (which carries the path bytes).
					 * The orphaned node is that skip-target; the child
					 * is the next link down.  Queue the target's plain
					 * compressed flag and advance to the child.
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
						nr_child = ft_meta_nr_child(cm);
						ext_nodes = cm->external_nodes;
						next = cn->child;
					} else {
						struct cds_ft_metadata *m =
							cds_ft_item_to_metadata(
								ft_node_ptr(walk_nf));

						nr_child = ft_meta_nr_child(m);
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
				 * The chain stops at @walk_nf.  A skip-compressed
				 * external leaf keeps its path bytes in a separate
				 * skip-target compressed node the walk stops short of:
				 * the external leaf is caller-owned, but that
				 * skip-target is trie-owned and now orphaned.  Queue it.
				 */
				if (walk_nf && ft_node_skip_compressed(walk_nf))
					trailing_skip_cn_flag = walk_nf;
			}
			/*
			 * The collected set (+ trailing skip-target) is tombstoned at
			 * the commit that unlinks it, not here -- see the freeze below
			 * this branch's commit dispatch (shape-D records into its own
			 * merge flip; an in-place delete records into @commit_txn;
			 * recompaction / list-off keep a standalone freeze before the
			 * free).  Deferred to the free walk in the !ret block below.
			 */
		}
#ifdef FEATURE_FT_SKIP_COMPRESSED
		/*
		 * Shape-D fusion: a pure delete (no external promote) that leaves
		 * the surviving boundary a non-root 1-child no-external internal
		 * folds the chain-compress prune INTO the key-removal commit -- build
		 * the merged compressed node and let it REPLACE the boundary in ONE
		 * flip (forward + surviving-child re-parent + a SKIP_X dual + the
		 * dead cell's unsplice), bypassing ft_node_replace_ptr entirely (no
		 * intermediate 1-child node is ever published) AND the standalone
		 * post-detach canonicalize below.  The boundary has exactly 2 live
		 * children here (it drops to 1); the surviving child is the one NOT
		 * being detached, read from pre-commit state.  On the merged-node
		 * allocation failing the whole detach aborts before any reader-
		 * visible store (the caller rolls back the count).  Out of bound
		 * (merged_len > FT_SKIP_LEN_MAX) falls through to replace_ptr.
		 */
		{
			struct cds_ft_metadata *bmeta =
				cds_ft_item_to_metadata(ft_node_ptr(iter_node_flag));

			if (ft_group_skip_compressed(ft->group) &&
			    !topmost_external_nodes &&
			    ft_meta_nr_child(bmeta) == 2 &&
			    !bmeta->external_nodes &&
			    bmeta->parent != NULL) {
				struct cds_ft_inode_flag *s_child = NULL;
				uint8_t s_byte = 0;
				unsigned int b;

				for (b = 0; b < 256; b++) {
					struct cds_ft_inode_flag *c;

					if ((uint8_t) b == n)
						continue;
					c = ft_node_get_nth(ft, iter_node_flag,
						NULL, (uint8_t) b, FT_PF_NONE);
					if (c) {
						s_child = c;
						s_byte = (uint8_t) b;
						break;
					}
				}
				if (s_child) {
					/*
					 * nr_keys fold (LEAF Increment 2): shape-D is the R3
					 * chain-compress fuse -- the merged node is built with
					 * its post-removal count and the -1 walk from its stable
					 * parent (publish_parent) rides the merge flip.  Reserve
					 * the walk (bounded by the removed leaf's depth); a no-op
					 * for a count-neutral move / !rank_stats.
					 */
					int cret = ft_chain_compress_fused(ft,
						iter_node_flag, bmeta,
						detach_parent_flag_ptr,
						s_child, s_byte,
						2 /* shape-D: survivor + the child this commit detaches */,
						fuse_cell, run,
						to_free, nr_to_free,
						trailing_skip_cn_flag, freeze_leaf,
						count_delta,
						ft->rank_stats ? detach_depth + 1 : 0);

					if (cret == 0) {
						ret = 0;
						boundary_fused = true;
						count_folded = true;
						if (freeze_leaf)
							freeze_leaf_fused = true;
						/*
						 * Shape-D already fused @fuse_cell's unsplice
						 * (ft_chain_compress_fused -> ft_remove_commit_rec)
						 * INTO this commit, so signal the caller: a
						 * standalone two-commit unsplice would re-record the
						 * (now stale) head/tail neighbour edge and reopen a
						 * cross-view window where a reader sees the removed
						 * key's cell as the ordered min after its structural
						 * removal.  @pub is non-NULL only on the fuse_cell
						 * point-remove path; the bulk @run is armed by
						 * ft_remove_commit_rec itself.
						 */
						if (pub)
							pub->armed = true;
					} else if (cret < 0) {
						ret = cret;
						goto end;
					}
					/* cret > 0: out of bound -- replace_ptr below. */
				}
			}
		}
#endif
		if (!boundary_fused) {
			struct cds_ft_inode_flag *bparent =
				metadata_stack[nr_branch - 1]->parent;
			/*
			 * Pre-reserve the commit txn BEFORE ft_node_replace_ptr's pre-flip
			 * side-effects -- the in-place delete's nr_child decrement (it
			 * commits the deferred slot store fused with the cell unsplice via
			 * ft_remove_one_commit below) OR the recompaction's eager re-parent
			 * of the rebuilt node's children (it publishes via the tail
			 * ft_remove_commit_rec).  Reserve only when that commit will be
			 * multi-edge: a cell unsplice / run is present, or the boundary's
			 * parent is compressed so the forward publish carries a SKIP_X dual.
			 * A lone edge (non-compressed parent, no cell) stays the infallible
			 * on-stack release store, so no txn is needed -- and an in-place
			 * delete with neither is a direct lone store inside
			 * ft_node_replace_ptr that never reaches ft_remove_one_commit.
			 * Reservation failure aborts before any side-effect (the upward
			 * walk is read-only).  Whichever commit fires consumes the txn; if
			 * none does (e.g. an n==1 publish, or replace_ptr failed) it is
			 * freed unused at @end.
			 *
			 * @commit_txn is also the retire txn for a DEL recompaction: when the
			 * boundary shrinks, ft_node_replace_ptr's recompact records the old
			 * copy's freeze-on-free tombstone INTO @commit_txn, so it flips
			 * atomically with the forward republish this same txn commits below
			 * (atomic detach, §4.B) rather than as an early standalone flip.  A
			 * key-disappearing remove on an ordered-list trie always unsplices a
			 * cell (fuse_cell), so @commit_txn is always reserved and every DEL
			 * recompaction retire fuses; only a list-off shrink under a
			 * non-compressed parent with no run leaves @commit_txn NULL, where the
			 * recompact falls back to the standalone tombstone flip (still safe:
			 * one writer, and the freeze is a no-op until the multi-writer engine).
			 */
			if (fuse_cell || run ||
			    (bparent && (ft_node_compressed(bparent) ||
					 ft_node_skip_compressed(bparent))) ||
			    (pub && (nr_to_free > 0 || trailing_skip_cn_flag)) ||
			    ft->rank_stats /* carry the nr_keys fold walk */) {
				commit_txn = ft_flip_txn_create_bounded(
					FT_REMOVE_COMMIT_REC_MAX_EDGES
					+ 1 /* §4.B parent guard (Site 1 arms excl.) */
					+ nr_to_free
					+ (trailing_skip_cn_flag ? 1 : 0)
					+ (retire_glue ? retire_glue->cap_free : 0)
					+ (ft->rank_stats ? detach_depth + 1 : 0) /* nr_keys fold walk */
					+ (freeze_leaf ? FT_HLIST_FREEZE_MAX_EDGES : 0));
				if (!commit_txn) {
					ret = -ENOMEM;
					goto end;
				}
			}
			ret = ft_node_replace_ptr(ft,
				detach_node_flag_ptr,
				&iter_node_flag,
				&old_recompacted_node,
				metadata_stack[nr_branch - 1],
				n, (struct cds_ft_inode_flag *) topmost_external_nodes,
				detach_parent_flag_ptr == &ft->root,
				cur_depth, pub, commit_txn);
		}
		if (!ret) {
			/*
			 * Freeze the collected orphan chain (+ trailing skip-target)
			 * before the free below (doc §4.B, atomic detach).  Now that
			 * replace_ptr has run, record the tombstones INTO the commit
			 * that unlinks the chain, since the free is now deferred past ALL
			 * of them (below the republish block).  pub != NULL always reserves
			 * and consumes @commit_txn -- via ft_remove_one_commit for an
			 * in-place delete, or the recompaction republish in the parent-slot
			 * block -- so the freeze flips ATOMICALLY with that unlink and an
			 * aborted commit discards it.  The list-off pub-less path does a
			 * direct lone store that never touches @commit_txn, so it keeps a
			 * standalone freeze (NULL txn) -- a no-op under one writer.  Shape-D
			 * already recorded into its own merge flip.
			 */
			if (!boundary_fused && (nr_to_free > 0 || trailing_skip_cn_flag))
				ft_detach_freeze_orphans(ft,
					(pub && commit_txn) ? commit_txn : NULL,
					to_free, nr_to_free, trailing_skip_cn_flag);
			/*
			 * A caller-supplied external retire set (@retire_glue: the
			 * merge src-side glue's overlap-spine free-list, freed by the
			 * caller AFTER this detach returns) freezes atomically with the
			 * SAME unlink that makes it unreachable.  A LIST-ON src detach
			 * rides @commit_txn here on the branch-2 commit_txn path --
			 * empirically 100% of merge stress: branch 2, pub != NULL,
			 * commit_txn present.  Recorded AFTER replace_ptr so a
			 * recompaction's own DEL tombstone is already in @commit_txn;
			 * order among tombstones is irrelevant (all commit together).
			 * A LIST-OFF src detach (pub == NULL) leaves it for the
			 * standalone fallback at @end -- as do shape-D / a compressed
			 * parent (a no-op under one writer).
			 */
			if (!boundary_fused && retire_glue && pub && commit_txn) {
				retire_glue->txn = commit_txn;
				retire_glue->fuse_free_list = true;
				ft_glue_tombstone_free_list(retire_glue);
				retire_glue_fused = true;
			}
			/*
			 * The removed external leaf (single-entry chain, node->next
			 * == NULL) freezes atomically with the SAME unlink: recorded
			 * into @commit_txn beside the orphan tombstones (the +1
			 * reserved above) when the in-place / recompaction commit
			 * consumes it (pub && commit_txn).  The list-off pub-less
			 * direct-store path (commit_txn NULL) keeps the standalone
			 * fallback at @end -- a no-op under one writer; shape-D already
			 * froze it in its merge flip.
			 */
			if (!boundary_fused && freeze_leaf && pub && commit_txn) {
				ft_hlist_freeze_prepare(ft_flip_txn_handle(commit_txn),
					freeze_leaf);
				freeze_leaf_fused = true;
			}
			/*
			 * In-place key-disappearing remove (the holder stayed
			 * above min_child, so replace_ptr deferred its single
			 * reader-visible forward store into @pub instead of doing
			 * it): commit that store -- @old_val -> @new_val (NULL for a
			 * leaf delete, the promoted external chain head for an
			 * external promote) -- fused with @fuse_cell's ordered-list
			 * unsplice in ONE flip.  A pigeon delete leaves its occupancy
			 * bit set (sticky soft-delete hint; recompact rebuilds it
			 * clean), so nothing settles after the flip.  Recompaction
			 * (pub unarmed) published its rebuilt node itself and stays
			 * two-commit -- the caller unsplices.
			 */
			if (!boundary_fused && pub && pub->armed) {
				/*
				 * nr_keys fold (LEAF Increment 2): the in-place delete /
				 * external promote leaves the holder @iter_node_flag in
				 * place (its subtree loses the disappearing key), so the
				 * -1 walk from @iter_node_flag up to root rides THIS
				 * commit atomically -- recorded before ft_remove_one_commit
				 * consumes @commit_txn.  A no-op / skipped for a count-
				 * neutral move / !rank_stats.
				 */
				if (count_delta && commit_txn) {
					ft_flip_txn_record_count_parent(ft, commit_txn,
						iter_node_flag, count_delta);
					count_folded = true;
				}
				/*
				 * External promote: the promoted head's back-channel
				 * re-parent (captured by replace_ptr into @pub) rides
				 * THIS commit so an ABORT discards it with the forward
				 * flip.  The @commit_txn-NULL lone-edge boundary keeps
				 * the pre-fusion timing: apply it just before the lone
				 * forward store (single writer, cannot abort).
				 */
				if (pub->head_parent_field) {
					if (commit_txn)
						ft_flip_txn_record_reserved(commit_txn,
							(void **) pub->head_parent_field,
							pub->head_parent_old,
							pub->head_parent_new);
					else
						rcu_assign_pointer(*pub->head_parent_field,
							pub->head_parent_new);
				}
				ret = ft_remove_one_commit(ft, pub->slot,
					pub->old_val, pub->new_val,
					pub->state_meta,
					fuse_cell, run, commit_txn, NULL);
				commit_txn_used = (commit_txn != NULL);
			}
			/*
			 * Hand the collected orphan chain off to the deferred free
			 * below the republish block (@orphan_free): a recompaction's
			 * forward unlink commits down there, so the reclaim must wait
			 * for it (a freed node's freeze-on-free tombstone commits with
			 * that unlink, and the free must follow the structural unlink,
			 * not precede it).  The tombstones are already recorded above --
			 * into @commit_txn (pub) or standalone (list-off).
			 */
			if (!ret) {
				for (fi = 0; fi < nr_to_free; fi++)
					orphan_free[fi] = to_free[fi];
				nr_orphan_free = nr_to_free;
				orphan_trailing = trailing_skip_cn_flag;
				free_orphans_pending = true;
			}
		}
	}
	if (ret)
		goto end;

	/*
	 * Update address of parent ptr in its parent.
	 * Skip for compressed parents: the replacement was already
	 * published inline above.  Skip entirely when shape-D fusion already
	 * committed the merged compressed node in place of the boundary (the
	 * boundary is freed, there is no forward republish or separate prune).
	 */
	if (!boundary_fused &&
	    !ft_node_compressed(iter_node_flag) &&
	    !ft_node_skip_compressed(iter_node_flag)) {
		struct cds_ft_metadata *iter_meta =
			cds_ft_item_to_metadata(ft_node_ptr(iter_node_flag));

		/*
		 * nr_keys fold (LEAF Increment 2): a recompaction rebuilds the
		 * holder smaller; the fresh copy (@iter_node_flag) carried the OLD
		 * node's count out of ft_node_recompact, so bake in its post-removal
		 * count here -- a build-invisible plain store on the not-yet-
		 * published fresh copy.  Reader-safety: the fresh copy is reachable
		 * via up-walk pre-commit (recompact re-parents children to it), but
		 * a node's own nr_keys is only ever read at a root-DESCENDED position
		 * -- which lands on the OLD node via the not-yet-flipped parent slot
		 * -- so its transiently-shifted count is never observed (same
		 * invariant as the insert I7 relocation fold).  The -1 walk from the
		 * STABLE grandparent iter_meta->parent rides the republish below.
		 */
		if (old_recompacted_node && count_delta)
			ft_nr_keys_store(ft, iter_meta,
				ft_nr_keys_get(iter_meta) + count_delta,
				CMM_RELAXED);

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

			/* VALIDATE (§4.B): guard the LIVE grandparent iter_meta->parent. */
			ft_flip_txn_guard_parent(ft, commit_txn, iter_meta->parent);
			_ft_publish_to_parent(ft, iter_meta->parent,
				detach_parent_flag_ptr, iter_node_flag, &rec);
			/*
			 * nr_keys fold (LEAF Increment 2): the recompaction's -1
			 * walk from the STABLE grandparent iter_meta->parent (the
			 * rebuilt copy itself was baked above) rides this same commit.
			 */
			if (count_delta) {
				ft_flip_txn_record_count_parent(ft, commit_txn,
					iter_meta->parent, count_delta);
				count_folded = true;
			}
			/*
			 * Recompaction (its eager child re-parent already ran in
			 * ft_node_replace_ptr) so the publish commits through the
			 * pre-reserved txn (reserved above; this arm requires
			 * fuse_cell/run) and cannot fail.
			 */
			if (ft_remove_commit_rec(ft, &rec, fuse_cell, run,
					commit_txn_used ? NULL : commit_txn) > 0) {
				/* Peer won: nothing installed (txn consumed). */
				commit_txn_used = (commit_txn != NULL);
				ret = -EAGAIN;
				goto end;
			}
			commit_txn_used = (commit_txn != NULL);
			pub->armed = true;
		} else if (!(pub && pub->armed) &&
		    (old_recompacted_node || topmost_external_nodes)) {
			struct ft_pub_rec rec = { .n = 0 };

			/*
			 * Real forward-slot change not yet committed: a
			 * non-fused recompaction (rebuilt node) or a
			 * non-in-place external promote.  Commit the
			 * forward slot + any compressed-grandparent
			 * SKIP_X dual atomically through @commit_txn
			 * (recompaction's child re-parent already ran);
			 * a lone-edge promote stays one release store.
			 *
			 * The !pub->armed guard excludes the IN-PLACE
			 * external promote: there the holder never moves
			 * (the promoted head goes into a slot below it,
			 * already committed by ft_remove_one_commit and
			 * fused with the cell unsplice, consuming
			 * @commit_txn).  Re-emitting the unchanged
			 * parent->holder edge here is an old==new no-op
			 * and, with @commit_txn spent, would trip the
			 * NULL-txn assert(n<=1) once a compressed
			 * grandparent makes it multi-edge; it falls
			 * through to the no-op else.  (pub->armed and
			 * old_recompacted_node are mutually exclusive.)
			 */
			/* VALIDATE (§4.B): guard the LIVE grandparent iter_meta->parent. */
			ft_flip_txn_guard_parent(ft, commit_txn, iter_meta->parent);
			_ft_publish_to_parent(ft, iter_meta->parent,
				detach_parent_flag_ptr, iter_node_flag, &rec);
			/*
			 * nr_keys fold (LEAF Increment 2): a non-fused RECOMPACTION
			 * folds the -1 walk from the stable grandparent onto this
			 * republish (the rebuilt copy was baked above).  A non-in-place
			 * external PROMOTE (old_recompacted_node NULL) does NOT -- the
			 * holder never moved, this republish re-emits an old==new no-op,
			 * and the promote's count falls to the standalone residual
			 * below (base @iter_node_flag).
			 */
			if (old_recompacted_node && count_delta) {
				ft_flip_txn_record_count_parent(ft, commit_txn,
					iter_meta->parent, count_delta);
				count_folded = true;
			}
			if (ft_remove_commit_rec(ft, &rec, NULL, NULL,
					commit_txn_used ? NULL : commit_txn) > 0) {
				/* Peer won: nothing installed (txn consumed). */
				commit_txn_used = (commit_txn != NULL);
				ret = -EAGAIN;
				goto end;
			}
			commit_txn_used = (commit_txn != NULL);
		}
		/*
		 * else: in-place redundant republish.  The holder
		 * stayed at its parent slot (a plain leaf delete or
		 * an in-place external promote), so the forward slot
		 * -- and any compressed-grandparent SKIP_X dual --
		 * already hold @iter_node_flag.  The in-place forward
		 * store committed via ft_remove_one_commit above;
		 * re-emitting the unchanged slot(s) is a pure no-op
		 * (old == new), so skip it entirely.  Any commit_txn
		 * left unconsumed here is freed at @end.
		 */

		/*
		 * nr_keys: every outcome now folds its -@count_delta walk onto the
		 * op's own commit -- in-place / recompaction into @commit_txn, shape-D
		 * into ft_chain_compress_fused's txn, compressed parent into
		 * @orphan_txn, and the former list-off pub-less lone store via the
		 * @local_pub substitution above (which arms @pub, so the pub-armed
		 * fold fires).  There is no standalone post-commit residual left.
		 */
		assert(count_folded || !count_delta || !ft->rank_stats);

#ifdef FEATURE_FT_SKIP_COMPRESSED
		/*
		 * Post-detach canonicalization: if the surviving ancestor
		 * is now a non-root internal with exactly 1 live child and
		 * no external_nodes attached, fold it via the chain-compress
		 * 4-case merge (canonical form under SKIP_COMPRESSED).
		 */
		if (ft_meta_nr_child(iter_meta) == 1 &&
		    !iter_meta->external_nodes &&
		    iter_meta->parent != NULL) {
			ft_canonicalize_chain_compress(ft, iter_node_flag,
				iter_meta, detach_parent_flag_ptr);
		}
#endif
	}
	/*
	 * Deferred branch-2 orphan free (see @orphan_free): runs AFTER the commit
	 * that unlinked the chain -- the in-place ft_remove_one_commit or the
	 * recompaction republish above -- so every freed node's freeze-on-free
	 * tombstone has already committed and the reclaim follows the structural
	 * unlink.  An abort takes `goto end` above and bypasses this; the flag is
	 * set only on a branch-2 success.
	 */
	if (free_orphans_pending) {
		int fj;

		if (orphan_trailing)
			free_compressed_node(ft, ft_skip_to_compressed(ft,
				orphan_trailing));
		for (fj = 0; fj < nr_orphan_free; fj++) {
			if (ft_node_compressed(orphan_free[fj]))
				free_compressed_node(ft,
					ft_compressed_node_ptr(orphan_free[fj]));
			else
				free_cds_ft_node(ft, ft_node_ptr(orphan_free[fj]));
		}
	}
end:
	/*
	 * nr_keys fold (LEAF Increment 2): no abort rollback needed.  Every
	 * per-outcome count fold rides an UNcommitted txn on the abort path (the
	 * commit is what would apply it), so an abort applies nothing; the
	 * standalone residual runs only on success (ret == 0, inside the parent-
	 * slot block).  So a failed detach leaves nr_keys untouched, and there is
	 * no pre-decrement left to undo.
	 */
	/*
	 * Free a pre-reserved commit txn that no commit consumed (reservation
	 * succeeded but ft_node_replace_ptr recompacted / failed, or shape-D
	 * fused with its own txn).  PREPARE state -> no grace period.
	 */
	if (commit_txn && !commit_txn_used)
		ft_flip_txn_destroy(commit_txn);
	/* Reclaim safely after replacement. */
	if (old_recompacted_node) {
		if (ret == -EAGAIN)
			/*
			 * ABORTED republish: the rebuilt copy never published
			 * (its child re-parent edges and the old copy's
			 * tombstone were recorded into the aborted commit and
			 * discarded with it).  The OLD node stays live and
			 * linked; reclaim the never-visible FRESH copy instead.
			 */
			free_cds_ft_node_unpublished(ft,
				ft_node_ptr(iter_node_flag));
		else
			free_cds_ft_node(ft, old_recompacted_node);
	}

	/*
	 * Standalone fallback for a caller-supplied @retire_glue that no commit
	 * txn absorbed.  A LIST-ON src detach passes pub != NULL and fuses above
	 * (probed: 100% branch-2 commit_txn); a LIST-OFF src detach passes pub ==
	 * NULL (no cell to unsplice, run == NULL), so the freeze block's `pub`
	 * gate fails and the retire set lands here -- the same lone-store residual
	 * the list-off orphan chain leaves, to be closed once the lone edge is
	 * forced through a txn.  A theoretical shape-D / compressed-parent detach
	 * lands here too.  On success (the unlink that stranded the retire set
	 * committed) freeze it standalone before the caller frees it; on an abort
	 * (ret != 0) the caller rolls the whole op back and does NOT free it, so
	 * leave it untouched.  A no-op under one writer either way.
	 */
	if (!ret && retire_glue && !retire_glue_fused) {
		retire_glue->fuse_free_list = false;
		ft_glue_tombstone_free_list(retire_glue);
	}
	/*
	 * Standalone fallback for the removed external leaf when no commit txn
	 * absorbed its freeze: the list-off pub-less direct-store detach (pub ==
	 * NULL leaves @commit_txn NULL, so the freeze block's `pub` gate fails) --
	 * the same lone-store residual as the list-off orphan chain, to be closed
	 * once the lone edge is forced through a txn.  On success (the unlink that
	 * stranded it committed) freeze @node before the caller reclaims it; on an
	 * abort (ret != 0) leave it chained.  Behaviour-identical to the old
	 * caller-side mark; a no-op under one writer.
	 */
	if (!ret && freeze_leaf && !freeze_leaf_fused)
		ft_node_mark_removed_flip(ft, freeze_leaf);
	/*
	 * A fused @retire_glue->txn now points at the commit_txn its commit
	 * reclaimed above -- clear it so a future free-path reader of the glue
	 * cannot dereference freed memory (nothing reads it on the caller's free
	 * path today; this is defensive).
	 */
	if (retire_glue)
		retire_glue->txn = NULL;

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
 * internal holder flag (no dual).  The list-on promotion is fully abortable: the
 * fresh-cell alloc AND the flip-txn pre-reservation both fail cleanly on OOM
 * (-ENOMEM, nothing published, the chain intact and retriable).  @next_node's
 * prev (its node->cell link) FOLDS into the swap commit as a proxied edge, so it
 * flips atomically with the forward publish; a concurrent skip resolution strips
 * the transient proxy at the load (ft_dereference_prev_resolved) before
 * ft_resolve_head_prev interprets it.  The flip is still pre-reserved so the
 * commit cannot OOM, never a bare in-place cell->node retarget under memory
 * pressure.  List off (no cell) inherits the flagged parent on the successor --
 * still a SETTLED store there (read raw), see the list-off branch.
 */
static
int ft_promote_head(struct cds_ft *ft, struct cds_ft_inode_flag *parent_nf,
		struct cds_ft_node **head_slot, struct cds_ft_node *node,
		struct cds_ft_node *next_node)
{
	struct ft_ord_cell *old_cell = ft->ordered_list ?
		ft_ord_cell_ptr(node->prev) : NULL;
	struct ft_pub_rec rec = { .n = 0 };
	struct ft_ord_cell_edge sedges[2] = { 0 };
	unsigned int n_s;

	assert(next_node != NULL);
	if (old_cell) {
		/*
		 * Ordered list on: cell->node is write-once, so head promotion
		 * publishes a FRESH cell and swaps it in.  Allocate the cell AND
		 * pre-reserve the flip-txn FIRST, before any reader-visible change;
		 * on either OOM abort here -- nothing is published and the chain is
		 * untouched, so the caller returns CDS_FT_STATUS_MEMORY_ERROR (the
		 * promotion may be retried).  No bare in-place cell->node retarget
		 * on OOM: that would commit a reader-visible store outside the
		 * descriptor protocol.
		 */
		void *new_cell_flag = ft_ord_cell_alloc(ft, next_node,
			old_cell->parent);
		struct ft_ord_cell *new_cell;
		struct ft_flip_txn *txn;

		if (!new_cell_flag)
			return -ENOMEM;
		txn = ft_flip_txn_create_bounded(
			FT_ORD_CELL_SWAP_PUBLISH_MAX_EDGES +
			FT_HLIST_FREEZE_MAX_EDGES + 2);	/* +1 §4.B parent guard, +1 next_node->prev fold */
		if (!txn) {
			ft_ord_cell_free_unpublished(ft,
				ft_ord_cell_ptr(new_cell_flag));
			return -ENOMEM;
		}
		new_cell = ft_ord_cell_ptr(new_cell_flag);
		cds_ft_item_to_metadata(new_cell)->incoming_byte =
			cds_ft_item_to_metadata(old_cell)->incoming_byte;
		/*
		 * Back-pointer (next_node->prev: the promoted successor's node->cell
		 * link) FOLDED into the swap commit as a proxied edge -- it flips
		 * ATOMICALLY with the forward head publish and @node's freeze, never a
		 * pre-commit reader-visible store.  A concurrent skip resolution strips
		 * the transient proxy at the load (ft_dereference_prev_resolved) before
		 * ft_resolve_head_prev interprets it, so the slot may carry the proxy.
		 * The forward publish's forward-before-parent check reads the folded
		 * prev's intended value (new_cell_flag), not the not-yet-stored slot.
		 * The reservation above carries this edge.
		 */
		ft_flip_txn_record_reserved(txn, (void **) &next_node->prev,
			next_node->prev, new_cell_flag);
		/* VALIDATE (§4.B): guard the LIVE holder this head-promote publishes into. */
		ft_flip_txn_guard_parent(ft, txn, parent_nf);
		_ft_publish_to_parent_meta(ft, parent_nf,
			(struct cds_ft_inode_flag **) head_slot,
			(struct cds_ft_inode_flag *) next_node, NULL,
			new_cell_flag, &rec);
		n_s = ft_pub_rec_sedges(&rec, sedges);
		/*
		 * Fuse @node's freeze (mark node->next, target preserved) into the
		 * swap commit: a reader never sees @node's head anchor promoted away
		 * while @node is still unmarked (doc §4.B).  The reservation above
		 * carries the extra edge.
		 */
		ft_hlist_freeze_prepare(ft_flip_txn_handle(txn), node);
		if (ft_ord_cell_swap_publish_multi(ft, old_cell, new_cell,
				sedges, n_s, txn)) {
			/*
			 * Peer won the commit: NOTHING installed -- @old_cell is
			 * still the linked head cell (must NOT be freed) and the
			 * fresh @new_cell never published.  Reclaim only ours
			 * and retry from a fresh derivation.
			 */
			ft_ord_cell_free_unpublished(ft, new_cell);
			return -EAGAIN;
		}
		ft_ord_cell_free(ft, old_cell);
	} else {
		/*
		 * List off: the head's prev IS the flagged parent, inherited on
		 * the promoted successor as a FOLDED edge riding the swap commit
		 * -- it flips ATOMICALLY with the forward head publish and
		 * @node's freeze, never a pre-commit reader-visible store (and
		 * never a hand-rolled restore on a peer-won commit, which the
		 * remove-retry skeptic flagged as assuming one-remover-per-node).
		 * The prev readers resolve a parked proxy at the load
		 * (ft_dereference_prev_resolved / ft_node_holder), mirroring the
		 * cell arm above; the forward publish's forward-before-parent
		 * check reads the folded INTENDED value, not the not-yet-stored
		 * slot.  A peer latch already parked on either word aborts the
		 * attempt before anything is recorded.
		 */
		struct ft_flip_txn *txn =
			ft_flip_txn_create_bounded(FT_PUB_SEDGE_MAX_EDGES +
				FT_HLIST_FREEZE_MAX_EDGES + 2);	/* +1 §4.B parent guard, +1 prev fold */

		void *prev_save;
		void *inherit;

		if (!txn)
			return -ENOMEM;
		prev_save = rcu_dereference(next_node->prev);
		inherit = rcu_dereference(node->prev);
		if (caa_unlikely(ft_node_flip_proxy(
					(struct cds_ft_inode_flag *) prev_save) ||
				ft_node_flip_proxy(
					(struct cds_ft_inode_flag *) inherit))) {
			ft_flip_txn_destroy(txn);	/* PREPARE state: nothing recorded */
			return -EAGAIN;
		}
		ft_flip_txn_record_reserved(txn, (void **) &next_node->prev,
			prev_save, inherit);
		/* VALIDATE (§4.B): guard the LIVE holder this head-promote publishes into. */
		ft_flip_txn_guard_parent(ft, txn, parent_nf);
		_ft_publish_to_parent_meta(ft, parent_nf,
			(struct cds_ft_inode_flag **) head_slot,
			(struct cds_ft_inode_flag *) next_node, NULL,
			inherit /* folded prev: intended parent value */, &rec);
		n_s = ft_pub_rec_sedges(&rec, sedges);
		/* Fuse @node's freeze into the structural publish (doc §4.B). */
		ft_hlist_freeze_prepare(ft_flip_txn_handle(txn), node);
		if (ft_ord_cell_flip_into(ft, txn, sedges, n_s) > 0) {
			/*
			 * Peer won: NOTHING installed -- the folded prev edge
			 * was discarded with the aborted commit, @next_node's
			 * prev still names its predecessor @node.  Retry from a
			 * fresh derivation; nothing to undo.
			 */
			return -EAGAIN;
		}
	}
	return 0;
}

static
int ft_unchain_node(struct cds_ft *ft, struct cds_ft_inode_flag *parent_nf,
		struct cds_ft_node **head_slot, struct cds_ft_node *node)
{
	struct cds_ft_node *next_node = ft_node_next(node);

	FT_TP(unchain_node, (const void *) head_slot, (const void *) node,
		!ft_node_external((struct cds_ft_inode_flag *) node->prev));
	if (ft_node_external((struct cds_ft_inode_flag *) node->prev)) {
		/*
		 * Non-head interior duplicate (prev is a cds_ft_node): atomic
		 * detach.  ft_hlist_del_prepare freezes @node (marks node->next),
		 * relinks the chain past it (prev_node->next: node -> next_node) and
		 * fixes the back-link (next_node->prev: node -> prev_node) in ONE
		 * commit -- freeze and unlink land together (doc §4.B), so once this is
		 * a concurrent engine a racing del(node)/insert_after(node) fails its
		 * old-value check.  Multi-edge: a reader resolves the transient
		 * interior-next proxies via cds_ft_node_next_rcu.  This subsumes the
		 * common ft_node_mark_removed freeze below, so it returns directly.
		 * Abortable cleanly: on a txn-alloc OOM nothing is recorded or
		 * published and @node stays fully chained.
		 */
		struct ft_flip_txn *txn =
			ft_flip_txn_create_bounded(FT_HLIST_DEL_MAX_EDGES);
		enum urcu_txn_status st;

		if (!txn)
			return -ENOMEM;
		if (ft_hlist_del_prepare(ft_flip_txn_handle(txn), node)) {
			/*
			 * Peer conflict observed at prepare time (@node or a
			 * neighbour mid-deletion): nothing was installed
			 * (records never install without a commit) -- drop the
			 * txn and retry from a fresh position derivation.
			 */
			ft_flip_txn_destroy(txn);
			return -EAGAIN;
		}
		st = ft_flip_txn_commit(ft, txn);
		if (st < 0)
			return -ENOMEM;
		/* ABORT: peer won, nothing installed, @node fully chained. */
		return st > 0 ? -EAGAIN : 0;
	} else if (next_node) {
		/*
		 * Head with a successor: prev is the cell flag (list on) or the
		 * flagged parent (list off).  Promote @next_node via a fresh cell
		 * swap fused with the structural publish, @node's freeze fused into
		 * that same commit (doc §4.B).  A list-on promotion allocates the
		 * fresh cell; on OOM it aborts before any reader-visible change
		 * (@node stays chained, not tombstoned) and the caller maps the
		 * failure to CDS_FT_STATUS_MEMORY_ERROR.
		 */
		return ft_promote_head(ft, parent_nf, head_slot, node, next_node);
	} else {
		/*
		 * Head with no successor: the key disappears (only reached for a
		 * list-off internal external_nodes chain that empties -- a list-on
		 * key disappearance routes through the fused ft_remove_one_commit,
		 * and a compressed / body-slot leaf through ft_detach_node).  Clear
		 * the head slot (+ a compressed holder's SKIP_X dual) and fuse
		 * @node's freeze (mark node->next: NULL -> MARK(NULL), preserving the
		 * end-of-chain target) into that same flip-txn (doc §4.B).
		 * Abortable: the flip IS the op's commit (no pre-flip reader-visible
		 * side-effect), so on a txn-alloc OOM the unchain aborts cleanly --
		 * nothing published, @node still chained, caller rolls back the -1
		 * count and returns MEMORY_ERROR.  Folding the freeze makes this a
		 * multi-edge commit (freeze + >=1 structural), so it always allocates
		 * a txn (no lone-edge on-stack path) -- one extra alloc per head
		 * clear, the single-writer cost of the atomic detach.
		 */
		struct ft_pub_rec rec = { .n = 0 };
		struct ft_ord_cell_edge sedges[2] = { 0 };
		struct ft_flip_txn *txn;
		unsigned int n_s;

		txn = ft_flip_txn_create_bounded(FT_PUB_SEDGE_MAX_EDGES +
			FT_HLIST_FREEZE_MAX_EDGES + 1);
		if (!txn)
			return -ENOMEM;
		/* VALIDATE (§4.B): guard the LIVE holder this head-clear publishes into. */
		ft_flip_txn_guard_parent(ft, txn, parent_nf);
		_ft_publish_to_parent(ft, parent_nf,
			(struct cds_ft_inode_flag **) head_slot, NULL, &rec);
		n_s = ft_pub_rec_sedges(&rec, sedges);
		ft_hlist_freeze_prepare(ft_flip_txn_handle(txn), node);
		if (ft_ord_cell_flip_into(ft, txn, sedges, n_s) > 0)
			/* Peer won: nothing installed, @node still chained. */
			return -EAGAIN;
	}
	return 0;
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
static
enum cds_ft_status _cds_ft_remove_locked(struct cds_ft *ft,
		struct cds_ft_iter *iter,
		struct cds_ft_node *node,
		bool *need_retry)
{
	struct cds_ft_inode_flag *holder_flag;
	struct cds_ft_metadata *holder_meta;
	struct cds_ft_inode_flag **head_slot = NULL;
	const uint8_t *iter_key;
	size_t key_len = ft_key_len(ft, ft_iter_resolve_key_len(iter));
	int ret;

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
	/*
	 * PRE-RESERVE the dead cell's standalone-unsplice txn here, in the
	 * fallible prefix BEFORE any structural change: when the structural
	 * commit cannot fuse the unsplice (a recompaction / external-promote
	 * shape leaves pub unarmed), the unsplice runs AFTER that commit is
	 * public -- un-abortable -- so its flip-txn must already be reserved.
	 * On OOM here the removal aborts cleanly (key untouched).  The fused
	 * common case frees it unused at the tail.
	 */
	struct ft_flip_txn *unsplice_txn = NULL;

	if (fuse_remove) {
		unsplice_txn = ft_flip_txn_create_bounded(
			FT_ORD_CELL_UNSPLICE_MAX_EDGES);
		if (!unsplice_txn) {
			FT_TP(remove_exit, (int) CDS_FT_STATUS_MEMORY_ERROR);
			return CDS_FT_STATUS_MEMORY_ERROR;
		}
	}

	if (ft_node_external(holder_flag)) {
		/*
		 * node->prev is a cds_ft_node: @node is a non-head duplicate.
		 * Unlink it from its chain (ft_unchain_node relinks the pinned
		 * predecessor/successor and tombstones @node).  The key count is
		 * unchanged (other duplicates remain) and the chain head -- and
		 * any grandparent skip pointer to it -- is untouched, so no head
		 * slot is needed.
		 */
		ret = ft_unchain_node(ft, NULL, NULL, node);
	} else if (ft_node_compressed(holder_flag) ||
		   ft_node_skip_compressed(holder_flag)) {
		/*
		 * Compressed holder: @node is its single external child
		 * (cn->child).  Leaf key.
		 */
		struct cds_ft_compressed_node *cn;

		/* Flight-recorder mis-wire detector (no-op without FT_ENABLE_TRACING). */
		FT_TRACE_MISWIRE(ft, holder_flag, 2);
		cn = ft_node_skip_compressed(holder_flag) ?
			ft_skip_to_compressed(ft, holder_flag) :
			ft_compressed_node_ptr(holder_flag);

		holder_meta = cds_ft_item_to_metadata((struct cds_ft_inode *) cn);
		head_slot = &cn->child;
		/*
		 * Resolve a peer's mid-commit flip proxy before the identity
		 * compare (§9): a raw proxy value would spuriously mismatch a
		 * slot that still resolves to @node.  A genuine post-resolve
		 * mismatch (a peer republished the holder) is caught by the
		 * commit's expected-value CAS / §4.B guard as ABORT -> retry.
		 */
		if ((struct cds_ft_node *) ft_node_ptr(ft_resolve_flip_proxy(
				rcu_dereference(*head_slot))) != node) {
			dbg_printf("cds_ft_remove: node %p not at compressed child slot\n", node);
			/* Drop the pre-reserved unsplice txn (nothing published yet). */
			if (unsplice_txn)
				ft_flip_txn_destroy(unsplice_txn);
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
			ret = ft_detach_node(ft, head_slot,
				ft_get_parent_slot(holder_meta, ft),
				key_len, true, fuse_cell, pubp, NULL, NULL, node,
				-1 /* leaf key removed: detach owns the -1 */);
			/* @node's freeze rode the detach commit (freeze_leaf). */
		} else {
			/*
			 * Removing the head, duplicates remain: key count unchanged.
			 * ft_promote_head fuses the cn->child republish with the
			 * grandparent SKIP_X dual (via _ft_publish_to_parent on cn's
			 * plain flag) and the cell swap in ONE flip -- so a candidate
			 * descent or ft_skip_reanchor up-walk never follows the stale
			 * skip pointer into the about-to-be-freed old head.
			 */
			ret = ft_unchain_node(ft, ft_compressed_node_flag(cn),
				(struct cds_ft_node **) head_slot, node);
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
			 * -- prefix-with-siblings).
			 */
			bool last_fused = false;
#ifdef FEATURE_FT_SKIP_COMPRESSED
			/*
			 * Fuse the chain-compress prune INTO the key-removal
			 * commit: when the holder is left a non-root 1-child
			 * no-external internal, its SKIP_COMPRESSED canonical form
			 * is a merged compressed node, so build that node and let
			 * it REPLACE the holder -- subsuming the external_nodes ->
			 * NULL clear -- carrying @dead_cell's unsplice in the SAME
			 * flip.  No separate clear, no transient non-canonical
			 * state.  Allocation failure aborts the whole removal (key
			 * not removed, count rolled back, retriable MEMORY_ERROR).
			 * The surviving body child is the holder's sole non-NIL
			 * branch (the external entry being removed is not a body
			 * child), stable across the removal.
			 */
			if (ft_group_skip_compressed(ft->group) &&
			    ft_meta_nr_child(holder_meta) == 1 &&
			    holder_meta->parent != NULL) {
				uint8_t s_byte = 0;
				struct cds_ft_inode_flag *s_child =
					ft_node_get_minmax(ft, holder_flag,
						&s_byte, FT_LEFTMOST, false);
				int cret;

				/*
				 * R3 chain-compress fold: the merged compressed node
				 * replacing the collapsed 1-child holder is built with the
				 * retired key's -1 and records the ancestor -1 walk into its
				 * OWN commit (count_delta -1), so no pre-decrement here.
				 */
				cret = ft_chain_compress_fused(ft,
					holder_flag, holder_meta,
					ft_get_parent_slot(holder_meta, ft),
					s_child, s_byte,
					1 /* sole body child; the removed entry is external */,
					fuse_cell, NULL,
					NULL, 0, NULL, node,
					-1, ft->rank_stats ? key_len + 1 : 0);

				if (cret == 0) {
					/*
					 * @node's freeze rode the fused merge commit
					 * above (freeze_leaf), atomic with the chain
					 * retire -- no separate mark_removed flip.
					 */
					if (fuse_remove)
						pub.armed = true;
					ret = 0;
					last_fused = true;
				} else if (cret < 0) {
					ret = cret;
					last_fused = true;
				}
				/* cret > 0: merge out of bound -- fall back to plain clear. */
			}
#endif
			if (!last_fused) {
				/*
				 * Non-fused: clear external_nodes -> NULL on its own.
				 * When the ordered list is on (fuse_remove), commit
				 * that store together with @dead_cell's unsplice in
				 * ONE flip (a reader never sees the key gone from the
				 * structural index but present in the ordered list;
				 * readers resolve a parked proxy on external_nodes via
				 * ft_dereference_external).  @pub.armed tells the
				 * deferred-free block below the unsplice happened.
				 */
				/*
				 * Fold the prefix key's -1 onto the clear: the holder
				 * stays in place (keeps its children), so its -1 is the
				 * holder->root walk -- a multi-edge R1.  Take a caller-
				 * reserved bounded txn whenever the count must ride
				 * (ordered_list || rank_stats), record the walk from
				 * holder_flag, and let ft_remove_one_commit flip the
				 * external_nodes -> NULL clear, @dead_cell's unsplice
				 * (list on) and @node's freeze (freeze_leaf) and the
				 * count edges TOGETHER (ft_ord_cell_flip_into,
				 * infallible): a reader never sees the key gone from one
				 * index but present in another, nor a count out of step
				 * with the structure (parked proxies resolve via
				 * ft_dereference_external / nr_keys via ft_nr_keys_load).
				 * No pre-decrement, no OOM rollback -- the pre-reserved
				 * txn commits infallibly, the only abort is the arm
				 * (which frees the pre-reserved unsplice txn and leaves
				 * the key in place).  List off + rank stats off: the
				 * infallible lone ft_unchain_node store, unchanged.
				 */
				if (ft->ordered_list || ft->rank_stats) {
					struct ft_flip_txn *txn = ft_flip_txn_create_bounded(
						FT_REMOVE_COMMIT_REC_MAX_EDGES + 1 +
						(ft->rank_stats ? key_len + 1 : 0));

					if (!txn) {
						if (unsplice_txn)
							ft_flip_txn_destroy(unsplice_txn);
						FT_TP(remove_exit, (int) CDS_FT_STATUS_MEMORY_ERROR);
						return CDS_FT_STATUS_MEMORY_ERROR;
					}
					/* VALIDATE (§4.B): guard the LIVE holder whose
					 * external_nodes this single-node clear empties. */
					ft_flip_txn_guard_parent(ft, txn, holder_flag);
					ft_flip_txn_record_count_parent(ft, txn,
						holder_flag, -1);
					ret = ft_remove_one_commit(ft,
						(struct cds_ft_inode_flag **) &holder_meta->external_nodes,
						(struct cds_ft_inode_flag *) node, NULL,
						NULL, dead_cell, NULL, txn, node);
					if (ret == 0 && fuse_remove)
						pub.armed = true;
				} else {
					/* List off + rank stats off: unchanged; no count. */
					ret = ft_unchain_node(ft, holder_flag,
						(struct cds_ft_node **) &holder_meta->external_nodes,
						node);
				}
#ifdef FEATURE_FT_SKIP_COMPRESSED
				/*
				 * Out-of-bound residue (the merge above did not apply
				 * because merged_len exceeds the compressed bound):
				 * best-effort post-prune via the standalone second
				 * flip, leaving a 1-child internal if it still cannot
				 * merge.  Only on a successful unlink.
				 */
				if (ret == 0 && ft_group_skip_compressed(ft->group) &&
				    !holder_meta->external_nodes &&
				    ft_meta_nr_child(holder_meta) == 1 &&
				    holder_meta->parent != NULL) {
					ft_canonicalize_chain_compress(ft, holder_flag,
						holder_meta, ft_get_parent_slot(holder_meta, ft));
				}
#endif
			}
		} else {
			/* Duplicates remain: head promotion (fresh-cell swap). */
			ret = ft_unchain_node(ft, holder_flag,
				(struct cds_ft_node **) &holder_meta->external_nodes,
				node);
		}
	} else {
		/*
		 * Internal holder, @node is a body child: leaf key.  Recover the
		 * holder's body slot for @node from the key's last byte.
		 */
		struct cds_ft_inode_flag *child;

		holder_meta = cds_ft_item_to_metadata(ft_node_ptr(holder_flag));
		child = ft_node_get_nth_skip(holder_flag, &head_slot,
			iter_key[key_len - 1], FT_PF_NONE);
		/* Resolve a peer's mid-commit proxy before the identity compare (§9). */
		child = ft_resolve_flip_proxy(child);
		if (!child ||
		    (struct cds_ft_node *) ft_node_ptr(child) != node) {
			dbg_printf("cds_ft_remove: node %p not at key slot\n", node);
			/* Drop the pre-reserved unsplice txn (nothing published yet). */
			if (unsplice_txn)
				ft_flip_txn_destroy(unsplice_txn);
			FT_TP(remove_exit, (int) CDS_FT_STATUS_NOT_FOUND);
			return CDS_FT_STATUS_NOT_FOUND;
		}
		if (!ft_node_next(node)) {
			/*
			 * Last/only entry: prune the now-empty branch.
			 * Propagate -1 before detach, which may free internal nodes.
			 */
			ret = ft_detach_node(ft, head_slot,
				ft_get_parent_slot(holder_meta, ft),
				key_len, true, fuse_cell, pubp, NULL, NULL, node,
				-1 /* leaf key removed: detach owns the -1 */);
			/* @node's freeze rode the detach commit (freeze_leaf). */
		} else {
			/* Removing the head, duplicates remain: key count unchanged. */
			ret = ft_unchain_node(ft, holder_flag,
				(struct cds_ft_node **) head_slot, node);
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
	if (fuse_remove) {
		/* cell_was_head implies ordered_list, so the list op always runs. */
		if (ret == 0) {
			/* An in-place / fused structural commit already carried the
			 * unsplice (pub.armed); otherwise commit it now through the
			 * txn pre-reserved before the structural change. */
			if (!pub.armed) {
				/*
				 * Two-commit fallback: the structural commit is
				 * PUBLIC, so this unsplice must complete -- on a
				 * peer conflict DRIVE IT FORWARD (each ABORT
				 * means a peer committed: obstruction-free), and
				 * retry the small bounded re-reservation too
				 * (the aborted commit consumed the txn); backing
				 * out would leave the key gone from the
				 * structural index but present in the ordered
				 * list.
				 */
				while (ft_ord_cell_unsplice(ft, unsplice_txn,
						dead_cell) > 0) {
					do {
						unsplice_txn =
							ft_flip_txn_create_bounded(
							FT_ORD_CELL_UNSPLICE_MAX_EDGES);
					} while (caa_unlikely(!unsplice_txn));
				}
			} else
				ft_flip_txn_destroy(unsplice_txn);
			ft_ord_cell_free(ft, dead_cell);
		} else {
			/* Removal aborted (MEMORY_ERROR or a peer conflict):
			 * nothing left the trie, the cell stays in the list --
			 * release the unused reservation. */
			ft_flip_txn_destroy(unsplice_txn);
		}
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
	case -EAGAIN:
		/*
		 * A peer writer won a commit this attempt (ABORT) or moved the
		 * position pre-commit: NOTHING was published and every fused
		 * edge (freeze, count, tombstone) was discarded with it.
		 * Signal the wrapper's retry loop to re-derive and re-attempt.
		 */
		*need_retry = true;
		return CDS_FT_STATUS_OK;	/* value unused: wrapper retries */
	default:
		abort();
	}
}

enum cds_ft_status cds_ft_remove(struct cds_ft *ft,
		struct cds_ft_iter *iter,
		struct cds_ft_node *node)
{
	struct urcu_mcas_txn optxn;
	enum cds_ft_status s;
	bool need_retry;

	CDS_FT_SCOPED_WRITER(ft);
	/*
	 * FT-owned per-op read-side bracket + retry identity (doc §11): on a
	 * concurrent trie the body's position derivation, parked records, and
	 * internal commits must run inside a read-side section so a peer
	 * writer's call_rcu-deferred frees cannot reclaim under them; a
	 * caller-held section merely nests.  @node's liveness AT ENTRY remains
	 * the caller's obligation (a lookup reference, valid only under the
	 * caller's own section -- the §11 reference-lifetime contract).
	 *
	 * RETRY: a peer-conflict attempt (a commit ABORT, or a pre-commit
	 * position conflict) publishes nothing and signals @need_retry; the
	 * loop re-derives the position from node->prev against the current
	 * tree and re-attempts.  The op's internal txns are still standalone
	 * (the pre-reserved unsplice txn coexists with the main commit txn, so
	 * they cannot share one handle), so contention aging is carried
	 * manually on the PERSISTENT @optxn via urcu_txn_conflict: after
	 * URCU_TXN_FALLBACK conflicts the domain escalates this writer into
	 * the per-trie FIFO fair-mutex lane -- every writer's begin() honors
	 * domain->active, so the lane drains the contention and the retry
	 * terminates (no livelock).  Exclusive trie: the bracket opens nothing
	 * and no conflict ever fires.
	 */
	ft_txn_op_init(ft, &optxn);
	for (;;) {
		need_retry = false;
		urcu_txn_begin(&optxn);
		s = _cds_ft_remove_locked(ft, iter, node, &need_retry);
		if (!need_retry)
			break;
		/* Age the conflict, keep the FIFO turn, close the attempt. */
		urcu_txn_conflict(&optxn);
		urcu_txn_end(&optxn);
	}
	urcu_txn_end(&optxn);
	return s;
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

static
enum cds_ft_status _cds_ft_remove_all_locked(struct cds_ft *ft,
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
		/*
		 * Ordered list on: the NIL key is the global minimum (a prefix of
		 * every key), so its removal is the prefix-with-siblings clear at
		 * the root.  Fuse the root external_nodes -> NULL store with the head
		 * cell's unsplice AND the root nr_keys -1 (rank stats) in ONE flip so
		 * a reader never sees the key gone from one index but present in the
		 * other, nor a count out of step with the structure; readers resolve
		 * a parked proxy on external_nodes via ft_dereference_external and on
		 * nr_keys via ft_nr_keys_load.  List off, no rank stats: express the
		 * node -> NULL clear as a lone 1-edge flip (an infallible release
		 * store) rather than a bare store.
		 */
		if (ft->ordered_list || ft->rank_stats) {
			struct ft_ord_cell *dead = ft->ordered_list ?
				ft_ord_cell_ptr(external_nodes->prev) : NULL;
			struct ft_flip_txn *txn;

			/*
			 * R1 count fold: the NIL key lives at the root, so its -1 is a
			 * single edge on the root's own nr_keys (no ancestors).  Reserve
			 * it plus the structural + cell edges, record the count walk from
			 * the root, and let ft_remove_one_commit flip them together --
			 * exact and atomic (the decrement goes live WITH the detach, not
			 * before it), so no pre-decrement and no OOM rollback: the pre-
			 * reserved txn commits infallibly and the arm is the only abort.
			 * A no-op count record when rank stats are off (list-on path).
			 */
			txn = ft_flip_txn_create_bounded(
				FT_REMOVE_COMMIT_REC_MAX_EDGES +
				(ft->rank_stats ? 1 : 0));
			if (!txn) {
				*result_node = NULL;
				return CDS_FT_STATUS_MEMORY_ERROR;
			}
			ft_flip_txn_record_count_parent(ft, txn, ft->root, -1);
			ft_remove_one_commit(ft,
				(struct cds_ft_inode_flag **) &metadata->external_nodes,
				(struct cds_ft_inode_flag *) external_nodes, NULL,
				NULL, dead, NULL, txn, NULL);
			if (dead)
				ft_ord_cell_free(ft, dead);
		} else {
			struct ft_ord_cell_edge edge = {
				.slot = (struct ft_ord_cell **)
					&metadata->external_nodes,
				.old_target = (struct ft_ord_cell *)
					external_nodes,
				.new_target = NULL,
			};

			ft_ord_cell_flip_one(&edge);
		}
		/* The whole chain has left the trie: tombstone every node. */
		ft_chain_mark_removed_flip(ft, external_nodes);
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
	/*
	 * PRE-RESERVE the dead cell's standalone-unsplice txn before any
	 * structural change (see cds_ft_remove): the prefix / recompaction /
	 * compressed-parent shapes stay two-commit (pub unarmed) and unsplice
	 * AFTER the structural commit is public -- un-abortable.  OOM here
	 * aborts the removal cleanly; the fused case frees it unused.
	 */
	struct ft_flip_txn *unsplice_txn = NULL;

	if (dead_cell) {
		unsplice_txn = ft_flip_txn_create_bounded(
			FT_ORD_CELL_UNSPLICE_MAX_EDGES);
		if (!unsplice_txn) {
			*result_node = NULL;
			return CDS_FT_STATUS_MEMORY_ERROR;
		}
	}

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
		{
			bool prefix_fused = false;
#ifdef FEATURE_FT_SKIP_COMPRESSED
			/*
			 * Fuse the chain-compress prune INTO the key-removal
			 * commit (see cds_ft_remove): when the holder is left a
			 * non-root 1-child no-external internal, the merged
			 * compressed node REPLACES the holder -- subsuming the
			 * external_nodes -> NULL clear -- and carries @dead_cell's
			 * unsplice in the SAME flip.  No separate clear, no
			 * transient non-canonical state.  Allocation failure
			 * aborts the whole removal (key not removed, count rolled
			 * back, retriable MEMORY_ERROR with a NULL out-param).
			 */
			if (ft_group_skip_compressed(ft->group) &&
			    ft_meta_nr_child(holder_meta) == 1 &&
			    holder_meta->parent != NULL) {
				uint8_t s_byte = 0;
				struct cds_ft_inode_flag *s_child =
					ft_node_get_minmax(ft, holder_flag,
						&s_byte, FT_LEFTMOST, false);
				int cret;

				/*
				 * R3 chain-compress fold: the merged compressed node
				 * replacing the collapsed 1-child holder is built with the
				 * retired key's -1 and records the ancestor -1 walk into its
				 * OWN commit (count_delta -1), so no pre-decrement here.
				 */
				cret = ft_chain_compress_fused(ft,
					holder_flag, holder_meta,
					ft_get_parent_slot(holder_meta, ft),
					s_child, s_byte,
					1 /* sole body child; the removed entry is external */,
					ft->ordered_list ? dead_cell : NULL,
					NULL, NULL, 0, NULL, NULL,
					-1, ft->rank_stats ? key_len + 1 : 0);

				if (cret == 0) {
					ft_chain_mark_removed_flip(ft, chain_head);
					if (ft->ordered_list)
						pub.armed = true;
					ret = 0;
					prefix_fused = true;
				} else if (cret < 0) {
					ret = cret;
					prefix_fused = true;
				}
				/* cret > 0: merge out of bound -- fall back to plain clear. */
			}
#endif
			if (!prefix_fused) {
				/*
				 * Non-fused external_nodes -> NULL clear, always on a
				 * pre-reserved bounded txn (no bare lone clear even with
				 * both flags off).  ft_remove_one_commit flips the
				 * external_nodes -> NULL clear, the head cell's unsplice
				 * (ordered list on; @pub.armed tells the deferred-free
				 * block below it happened), the prefix key's -1 count
				 * walk from holder_flag (rank stats on) AND the §4.B
				 * VALIDATE guard on the LIVE holder TOGETHER in ONE flip
				 * (ft_ord_cell_flip_into, infallible commit): a reader
				 * never sees the key gone from one index but present in
				 * the other, nor a count out of step with the structure
				 * (parked proxies resolve via ft_dereference_external /
				 * nr_keys via ft_nr_keys_load), and a concurrent remove
				 * that froze the holder aborts this clear.  The holder
				 * stays in place (keeps its children), so its -1 is the
				 * holder->root walk -- a multi-edge R1.  Pre-reserved =>
				 * infallible commit: no pre-decrement, no OOM rollback;
				 * the only abort is the arm, which leaves the key in
				 * place.  List off + rank off: the forward clear + the
				 * guard, a 2-record slab-allocated MCAS commit.
				 */
				struct ft_flip_txn *txn = ft_flip_txn_create_bounded(
					FT_REMOVE_COMMIT_REC_MAX_EDGES + 1 +
					(ft->rank_stats ? key_len + 1 : 0));

				if (!txn) {
					if (unsplice_txn)
						ft_flip_txn_destroy(unsplice_txn);
					*result_node = NULL;
					return CDS_FT_STATUS_MEMORY_ERROR;
				}
				/* VALIDATE (§4.B): guard the LIVE holder whose
				 * external_nodes this clear empties. */
				ft_flip_txn_guard_parent(ft, txn, holder_flag);
				ft_flip_txn_record_count_parent(ft, txn,
					holder_flag, -1);
				ft_remove_one_commit(ft,
					(struct cds_ft_inode_flag **) &holder_meta->external_nodes,
					(struct cds_ft_inode_flag *) chain_head, NULL,
					NULL, dead_cell, NULL, txn, NULL);
				if (ft->ordered_list)
					pub.armed = true;
				ret = 0;
				if (ret == 0) {
					ft_chain_mark_removed_flip(ft, chain_head);
					assert(ft_meta_nr_child(holder_meta) > 0);
#ifdef FEATURE_FT_SKIP_COMPRESSED
					/*
					 * Out-of-bound residue: best-effort post-prune
					 * via the standalone second flip (leaves a
					 * 1-child internal if it still cannot merge).
					 */
					if (ft_group_skip_compressed(ft->group) &&
					    ft_meta_nr_child(holder_meta) == 1 &&
					    holder_meta->parent != NULL) {
						ft_canonicalize_chain_compress(ft, holder_flag,
							holder_meta, ft_get_parent_slot(holder_meta, ft));
					}
#endif
				}
			}
		}
	} else {
		/*
		 * Leaf key: the whole chain sits at head_slot (a body slot, or
		 * a compressed holder's &cn->child).  Removing it empties the
		 * slot, so prune the branch via ft_detach_node bootstrapped from
		 * the holder (it climbs via metadata->parent).  Propagate -1
		 * before detach (which may free internal nodes).
		 */
		ret = ft_detach_node(ft, head_slot,
			ft_get_parent_slot(holder_meta, ft), key_len, true,
			dead_cell, ft->ordered_list ? &pub : NULL, NULL, NULL,
			NULL, -1 /* leaf key removed: detach owns the -1 */);
		if (!ret)
			ft_chain_mark_removed_flip(ft, chain_head);
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
	if (ft->ordered_list) {
		if (ret == 0) {
			/* An in-place / fused structural commit already carried the
			 * unsplice (pub.armed); otherwise commit it now through the
			 * txn pre-reserved before the structural change. */
			if (!pub.armed)
				/* remove_all: not yet retry-enabled (whole-chain
				 * standalone marks block it); ABORT unreachable
				 * under its current exclusion. */
				(void) ft_ord_cell_unsplice(ft, unsplice_txn,
					dead_cell);
			else
				ft_flip_txn_destroy(unsplice_txn);
			ft_ord_cell_free(ft, dead_cell);
		} else {
			/* Removal aborted: the cell stays in the list -- release the
			 * unused reservation. */
			ft_flip_txn_destroy(unsplice_txn);
		}
	}

	iter->cache_valid = false;
	iter_debug_path_clear(iter);
	iter->path_len = 0;

	if (ret) {
		/*
		 * Leaf-key detach ENOMEM: nothing was published -- the chain is
		 * still live in the trie, and the -1 count rides the commit
		 * (ft_flip_txn_record_count_parent), so a failed/aborted commit
		 * discarded it with the rest: no standalone pre-decrement
		 * remains to restore (the old "count undo" is gone with the
		 * R-fold).  Surface the real error with a NULL out-param --
		 * the header contract -- so the caller cannot reclaim the
		 * still-reachable chain.
		 *
		 * KNOWN MW GAP: a peer-conflict -EAGAIN (copied-slot latch
		 * bail, chain-compress abort) also lands here as MEMORY_ERROR
		 * -- remove_all has no retry loop yet ("not yet retry-
		 * enabled" above).  Nothing is published either way; the
		 * error class is wrong, not the structure.
		 */
		*result_node = NULL;
		return CDS_FT_STATUS_MEMORY_ERROR;
	}

	return CDS_FT_STATUS_OK;
}

enum cds_ft_status cds_ft_remove_all(struct cds_ft *ft,
		struct cds_ft_iter *iter,
		struct cds_ft_node **result_node)
{
	struct urcu_mcas_txn optxn;
	enum cds_ft_status s;

	CDS_FT_SCOPED_WRITER(ft);
	/*
	 * FT-owned per-op read-side bracket (doc §11 Phase A) -- see
	 * cds_ft_remove.  Internal txns are still standalone (no retry loop
	 * to carry aging across yet); exclusive trie: the bracket opens
	 * nothing.
	 */
	ft_txn_op_init(ft, &optxn);
	urcu_txn_begin(&optxn);
	s = _cds_ft_remove_all_locked(ft, iter, result_node);
	urcu_txn_end(&optxn);
	return s;
}

