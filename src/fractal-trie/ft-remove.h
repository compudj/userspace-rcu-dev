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

#ifdef FT_B2_ARM_PROBE
/*
 * -DFT_B2_ARM_PROBE: DID THE PATH EVEN RUN?  Per publish path of
 * ft_detach_node, how often the path is REACHED carrying a commit txn, and
 * what ft_flip_txn_arm_per_op's own predicate answers there -- an empty
 * registry, a non-FINE trie, a spacing the arm refuses, or an arm already in
 * force.  CALL counters, never failure counters.
 *
 * ★ WHY IT IS NOT ANSWERED BY THE armSW COLUMN.  ft-txn-kind-stats prices a
 * site by TXN CREATION, so an arm that never runs and an arm that runs and is
 * REFUSED report the same zero -- and a third site hand-arming the same txn
 * (the rekey fold) reports as this site's armSW.  Splitting reach from refusal
 * is the only way to read an arm's yield as a property of the arm.
 *
 * ☠ MEASURED, and it is the finding: ft_inv FT_INV_MW=1 reaches the IN-PLACE
 * publish path TEN times in a whole run.  An in-place delete needs
 * ft_in_place_ok(), which needs an EXCLUSIVE trie -- and an exclusive trie is
 * one the per-op arm refuses outright -- so on a shared trie every delete
 * recompacts and only the external PROMOTE sub-case reaches it at all.  The
 * remove surface commits through the recompaction republish below.
 */
static unsigned long ft_b2p_inplace_reach, ft_b2p_inplace_nolocks,
	ft_b2p_inplace_armed,
	ft_b2p_pubA_reach, ft_b2p_pubA_nolocks, ft_b2p_pubA_armed,
	ft_b2p_pubB_reach, ft_b2p_pubB_nolocks, ft_b2p_pubB_armed,
	ft_b2p_created, ft_b2p_notfine, ft_b2p_nospacing;

static __attribute__((destructor))
void ft_b2_arm_probe_report(void)
{
	fprintf(stderr,
"# FT_B2_ARM_PROBE (ft_detach_node commit_txn, per publish path)\n"
"#   created           %lu   (!lock_fine %lu, bad spacing %lu)\n"
"#   in-place  reach   %lu   nr_locks==0 %lu   armed %lu\n"
"#   republish A reach %lu   nr_locks==0 %lu   armed %lu\n"
"#   republish B reach %lu   nr_locks==0 %lu   armed %lu\n",
		uatomic_load(&ft_b2p_created, CMM_RELAXED),
		uatomic_load(&ft_b2p_notfine, CMM_RELAXED),
		uatomic_load(&ft_b2p_nospacing, CMM_RELAXED),
		uatomic_load(&ft_b2p_inplace_reach, CMM_RELAXED),
		uatomic_load(&ft_b2p_inplace_nolocks, CMM_RELAXED),
		uatomic_load(&ft_b2p_inplace_armed, CMM_RELAXED),
		uatomic_load(&ft_b2p_pubA_reach, CMM_RELAXED),
		uatomic_load(&ft_b2p_pubA_nolocks, CMM_RELAXED),
		uatomic_load(&ft_b2p_pubA_armed, CMM_RELAXED),
		uatomic_load(&ft_b2p_pubB_reach, CMM_RELAXED),
		uatomic_load(&ft_b2p_pubB_nolocks, CMM_RELAXED),
		uatomic_load(&ft_b2p_pubB_armed, CMM_RELAXED));
}

#define FT_B2P_PATH(name, ft, txn)					\
	do {								\
		if (txn) {						\
			uatomic_inc(&ft_b2p_##name##_reach);		\
			if (!(txn)->nr_locks)				\
				uatomic_inc(&ft_b2p_##name##_nolocks);	\
			if ((txn)->structural_sw)			\
				uatomic_inc(&ft_b2p_##name##_armed);	\
		}							\
	} while (0)
#define FT_B2P_CREATED(ft, txn)						\
	do {								\
		if (txn) {						\
			uatomic_inc(&ft_b2p_created);			\
			if (!(ft)->lock_fine)				\
				uatomic_inc(&ft_b2p_notfine);		\
			else if ((ft)->lock_spacing !=			\
					CDS_FT_LOCK_SPACING_PER_NODE)	\
				uatomic_inc(&ft_b2p_nospacing);		\
		}							\
	} while (0)
#else
#define FT_B2P_PATH(name, ft, txn)	do { } while (0)
#define FT_B2P_CREATED(ft, txn)		do { } while (0)
#endif	/* FT_B2_ARM_PROBE */

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
/*
 * @iter_depth is the byte-depth of @iter_node_flag, and @ctx the op's lock
 * context: both acquires below are lock-set members, and a member's acquire
 * goes to its ANCHOR (doc/design/ft-dlm-lock-coarseness.md §2).
 */
static
int ft_detach_node_replace_compressed_parent(struct cds_ft *ft,
		struct cds_ft_inode_flag *iter_node_flag,
		unsigned int iter_depth,
		const struct ft_lock_ctx *ctx,
		struct cds_ft_inode_flag **detach_parent_flag_ptr,
		struct cds_ft_node *topmost_external_nodes,
		struct cds_ft_inode_flag *elevated_old_child,
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
	/*
	 * Resolve the retired compressed node ONCE via the shared read-side
	 * reanchor (MW convergence): @iter_node_flag is a raw slot value that may
	 * be SKIP-compressed, and ft_compressed_node_ptr does NOT strip the skip
	 * high bits -- a raw ft_compressed_node_ptr(iter_node_flag) fed to
	 * cds_ft_item_to_metadata() faults on the skip_len bits (the ft-remove.h
	 * detach segv).  @src_cn is the LIVE compressed node the deref sites
	 * (src_meta, the tombstone freeze, the free) must use; the raw
	 * @iter_node_flag is kept only for the forward CAS expected-old (it must
	 * match the SKIP_X stored in the grandparent slot).  A rewind > 0 (a peer
	 * chain-merge moved the encoded position shallower) means the detach
	 * premise is stale -- bail before any build so the caller re-descends.
	 */
	unsigned int iter_rewind;
	struct cds_ft_compressed_node *src_cn = ft_compressed_node_ptr(
		ft_reanchor_flag(ft, iter_node_flag, &iter_rewind));

	if (caa_unlikely(iter_rewind)) {
		/*
		 * @txn is caller-owned and already holds the orphan freeze-on-free
		 * tombstones; the caller's cleanup does NOT destroy on -EAGAIN (it
		 * assumes a commit consumed it), so a PRE-commit -EAGAIN must destroy
		 * it here or leak the descriptor.  (Pre-existing on this rewind bail;
		 * the DLM acquire-miss bails below share the same contract.)
		 */
		ft_flip_txn_destroy(txn);
		return -EAGAIN;
	}
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
		struct cds_ft_compressed_node *cn = src_cn;
		/*
		 * DLM Step 1: @cn is the value-swap RELEASE target this external-
		 * promote publishes into (its child slot); acquire it (hard, no guard-
		 * fallback) up front and record its {LOCK|s -> s} release, so both
		 * publish arms below skip their lock_or_guard.  -EAGAIN before any edge
		 * is recorded -> caller destroys @txn and re-descends.
		 */
		if (ft->lock_fine) {
			struct ft_held_anchor cn_held;

			if (ft_acquire_member(ft, ctx,
					ft_compressed_node_flag(cn),
					cds_ft_item_to_metadata(
						(struct cds_ft_inode *) cn),
					iter_depth, &cn_held)) {
				/* Pre-commit -EAGAIN: destroy the caller-owned txn
				 * (its cleanup skips destroy on -EAGAIN). */
				ft_flip_txn_destroy(txn);
				return -EAGAIN;
			}
			/*
			 * A member the op ALREADY held rode an earlier acquire,
			 * which recorded its release; a second record would settle
			 * the single word twice.
			 */
			if (!cn_held.shared) {
				ft_flip_txn_lock_register(txn, cn_held.lock,
					cn_held.lock_snap);
				ft_flip_txn_record_release_lock(txn, cn_held.lock,
					cn_held.lock_snap);
			}
		}
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

				ft_flip_txn_record_head_back_edge(txn,
					(void **) &cell->parent,
					cell->parent, cn_flag);
			} else {
				ft_flip_txn_record_head_back_edge(txn,
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

			/* VALIDATE (§4.B): guard the LIVE kept compressed node cn.
			 * DLM: cn's RELEASE was acquired + recorded up front under
			 * lock_fine, so skip the incremental lock here. */
			if (!ft->lock_fine)
				ft_flip_txn_lock_or_guard_parent(ft, txn, ctx,
					ft_compressed_node_flag(cn), iter_depth);
			_ft_publish_to_parent(ft, ft_compressed_node_flag(cn),
				&cn->child,
				(struct cds_ft_inode_flag *) topmost_external_nodes,
				elevated_old_child, &rec);
			if (ft_remove_commit_rec(ft, &rec, fuse_cell, run,
					txn, false) > 0)
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
			/* VALIDATE (§4.B): guard the LIVE kept compressed node cn.
			 * DLM: cn's RELEASE was acquired + recorded up front under
			 * lock_fine, so skip the incremental lock here. */
			if (!ft->lock_fine)
				ft_flip_txn_lock_or_guard_parent(ft, txn, ctx,
					ft_compressed_node_flag(cn), iter_depth);
			_ft_publish_to_parent(ft, ft_compressed_node_flag(cn),
				&cn->child,
				(struct cds_ft_inode_flag *) topmost_external_nodes,
				elevated_old_child, &rec);
			if (ft_remove_commit_rec(ft, &rec, NULL, NULL, txn, false) > 0)
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
			(struct cds_ft_inode *) src_cn);
		/*
		 * Recover the grandparent (parent, slot) as ONE coherent snapshot
		 * from src_meta, exactly as the chain-compress publishes do -- never
		 * pair the descent-captured @detach_parent_flag_ptr with a fresh
		 * src_meta->parent.  A peer that recompacts the grandparent to a larger
		 * node type (or re-homes it, Phase 4.3) between the descent and here
		 * leaves the captured slot pointing into the OLD, now-replaced parent
		 * body while src_meta->parent already names the NEW one; pairing the
		 * stale slot with the fresh parent indexes the new parent's bitmap out
		 * of bounds (ft_slot_to_byte OOB assert).  ft_resolve_parent_slot
		 * computes the slot as parent_body + offset, so it is in-bounds by
		 * construction; the txn guard + forward CAS + tombstone freeze below
		 * abort a commit if the grandparent or src_cn raced after this snapshot.
		 */
		struct cds_ft_inode_flag *pub_parent;
		struct cds_ft_inode_flag **pub_slot =
			ft_resolve_parent_slot(src_meta, ft, &pub_parent);

		/*
		 * The fresh replacement is EMPTY -- nothing below sets a child on
		 * it -- and this sub-case exists for the ROOT: "a compressed root
		 * from detach/graft_swap must remain internal" (the header above).
		 * An empty internal AT THE ROOT is the empty trie, which is legal.
		 *
		 * Reached with a non-NULL parent, the same publish wires a
		 * childless internal into a live parent slot, and that node stays
		 * there: correctly counted (0), correctly back-pointered, of the
		 * smallest type, holding nothing.  It is the "dead interior node"
		 * cds_ft_verify names, and it breaks ordered navigation -- the
		 * empty-subtree arm of the inequality descent is justified by the
		 * premise that a slot-emptied internal is never left in place, so
		 * a walk that reaches it either re-enters the branch forever or
		 * climbs out and skips the remaining subtree.  Exact-key lookup
		 * still finds every stranded key, which is why nothing else
		 * reported it.
		 *
		 * The plan that led here is stale rather than the tree being
		 * broken, so bail and let the caller re-descend, exactly as the
		 * rewind and DLM acquire-miss bails above do.  PRE-commit: free
		 * the unpublished node and destroy the caller-owned txn (the
		 * caller skips the destroy on -EAGAIN, assuming a commit consumed
		 * it), leaving the structure byte-for-byte as it was.
		 */
		if (pub_parent != NULL) {
			free_cds_ft_node_unpublished(ft, fresh);
			ft_flip_txn_destroy(txn);
			return -EAGAIN;
		}

		fresh_meta->parent_word = ft_parent_word(ft, pub_parent);
		/*
		 * DLM Step 1: acquire {src_cn (RETIRE), pub_parent (RELEASE)} in ONE
		 * MCAS up front (guard src_cn.parent==pub_parent), replacing the
		 * pub_parent lock_or_guard + the PLAIN src_cn tombstone (a peer state
		 * change under the fence now aborts).  This compressed->fresh-internal
		 * sub-case is test-under-covered, so it mirrors the chain-compress
		 * pattern exactly.  pub_parent NULL (compressed ROOT retire) => the
		 * set is {src_cn} only.
		 */
		struct cds_ft_metadata *src_cn_meta_a =
			cds_ft_item_to_metadata((struct cds_ft_inode *) src_cn);
		struct ft_held_anchor src_held;
		unsigned int pp_depth = 0;
		bool dlm_a2 = false;

		/*
		 * @pub_parent came from src_cn's back-pointer, which carries no
		 * depth; the descent's window is what dates it.  A parent this
		 * descent never passed cannot be anchored here at all -- re-plan
		 * rather than anchor it by another node's depth.
		 */
		if (pub_parent && !ft_lock_ctx_depth_of(ft, ctx, pub_parent,
				&pp_depth)) {
			free_cds_ft_node_unpublished(ft, fresh);
			ft_flip_txn_destroy(txn);
			return -EAGAIN;
		}
		if (ft->lock_fine) {
			struct ft_dlm_member set[2];
			int dret;

			set[0] = (struct ft_dlm_member){
				.nf = ft_compressed_node_flag(src_cn),
				.node = src_cn_meta_a,
				.depth = iter_depth,
				.guard_child = src_cn_meta_a,
				.guard_pf = pub_parent };
			set[1] = (struct ft_dlm_member){ .nf = pub_parent,
				.node = pub_parent ?
					ft_flag_to_metadata(ft, pub_parent) : NULL,
				.depth = pp_depth };
			dret = ft_dlm_acquire_set(ft, ctx, set, 2);
			if (dret) {
				free_cds_ft_node_unpublished(ft, fresh);
				/*
				 * Pre-commit bail: on -EAGAIN the caller-owned txn is
				 * NOT destroyed by the caller, so destroy it here; on
				 * -ENOMEM the caller destroys it (its != -EAGAIN arm).
				 */
				if (dret == -EAGAIN)
					ft_flip_txn_destroy(txn);
				return dret;	/* nothing acquired (all-or-none) */
			}
			src_held = set[0].held;
			/*
			 * Coarsening put src_cn's lock on an ANCESTOR that
			 * SURVIVES this retire, so the fused
			 * {LOCK|s -> TOMBSTONE|s} splits: the ancestor is
			 * released here (before any guard lands on it -- the
			 * ordering rule), the node tombstoned below.  A no-op
			 * where the two words coincide.
			 */
			ft_flip_txn_lock_register(txn, src_held.lock,
				src_held.lock_snap);
			ft_flip_txn_record_anchor_release(txn, &src_held,
				src_cn_meta_a);
			if (pub_parent && !set[1].held.shared) {
				ft_flip_txn_lock_register(txn, set[1].held.lock,
					set[1].held.lock_snap);
				ft_flip_txn_record_release_lock(txn,
					set[1].held.lock, set[1].held.lock_snap);
			}
			dlm_a2 = true;
		}
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
			/* VALIDATE (§4.B): lock (or guard-fallback) the
			 * coherently-resolved grandparent -- value-swap target (§10.5).
			 * DLM: pub_parent's RELEASE was acquired up front under lock_fine. */
			if (!ft->lock_fine)
				ft_flip_txn_lock_or_guard_parent(ft, txn, ctx,
					pub_parent, pp_depth);
			_ft_publish_to_parent(ft, pub_parent,
				pub_slot,
				ft_node_flag(fresh, 0),
				*pub_slot, &rec);
			/*
			 * Freeze the retired compressed node dead (§4.B freeze-on-
			 * free): this compressed->fresh-internal recompaction retires
			 * the old compressed node like every other retire.  Record the
			 * tombstone INTO @txn so it flips ATOMICALLY with the publish
			 * that unlinks it -- an aborted publish leaves it live (atomic
			 * detach).  This detach sub-case is not reached by the current
			 * test suite (verified: zero hits across ft_unit + ft_inv both
			 * list modes), so the freeze-on-free audit never exercises it.
			 * DLM: src_cn was LOCK-acquired up front, so record the FENCED
			 * terminal (a peer state change aborts) -- fused with the
			 * release where the acquire took src_cn's own word, plain
			 * against its clean word where coarsening sent the lock to a
			 * surviving ancestor.
			 */
			if (dlm_a2)
				ft_flip_txn_record_retire_anchored(txn, ctx,
					&src_held, src_cn_meta_a);
			else
				ft_flip_txn_record_tombstone(txn, cds_ft_item_to_metadata(
					(struct cds_ft_inode *) src_cn));
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
					pub_parent, count_delta);
			if (ft_remove_commit_rec(ft, &rec, NULL, NULL, txn, false) > 0) {
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
		free_compressed_node(ft, src_cn);
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
 *
 * DLM plan-lock (lock_fine): @snaps non-NULL means
 * the caller lock-acquired each orphan at collection (parallel to @orphans, and
 * @trailing_snap for the trailing skip-target) so the collapse decision was made
 * on a frozen state word.  Record the FENCED {LOCK|s -> TOMBSTONE|s} terminal
 * from that snapshot instead of the plain RYW tombstone: a peer that grew the
 * orphan tears the fenced expected-old and ABORTS this commit (§9.2
 * orphan-chain coherence).  In practice a peer cannot even get that far:
 * ft_meta_nr_child_inc SPINS on FT_STATE_INPLACE_WAIT_MASK, which includes
 * FT_STATE_LOCK unconditionally, so it WAITS for the mark and the coherence
 * comes from EXCLUSION rather than from detect-and-abort.
 * @held NULL keeps the plain path
 * byte-identical.  Under DLM @txn is always non-NULL (commit_txn is FORCE-TXN and
 * pub is never NULL), so the fenced arm never needs the standalone fallback.
 */
/*
 * Freeze ONE orphan from the acquire that protects it: lift the lock off the
 * ancestor that survives the retire (a no-op where the orphan carries its own),
 * then tombstone the orphan against ITS clean word.  A member that deduped onto
 * a word an earlier orphan took owes no release -- one word, one terminal.
 *
 * A SURVIVING anchor is handed to @txn outright -- release record AND registry
 * entry -- so the caller's sweep stops owning it (struct ft_held_anchor's
 * @txn_owned).  The two owners cannot be reconciled after the fact: the release
 * leaves that word clean and LIVE, a peer takes it immediately (under a coarse
 * spacing every op wants the same ancestor), and a later "release it if it is
 * still locked" then strips the PEER's mark.  Where the anchor IS the retired
 * node the fused tombstone is its terminal and the word is unlockable
 * afterwards, so that mark stays with the caller and costs no registry slot --
 * which is what keeps a FT_MAX_DEPTH orphan chain inside FT_FLIP_TXN_MAX_LOCKS,
 * and the per-node granularity byte-identical.
 */
static inline
void ft_detach_freeze_one(struct ft_flip_txn *txn,
		const struct ft_lock_ctx *ctx,
		struct ft_held_anchor *h, struct cds_ft_metadata *m)
{
	if (!h->shared) {
		/* Register BEFORE recording: the record asks who owns the word. */
		if (h->lock != m) {
			ft_flip_txn_lock_register(txn, h->lock, h->lock_snap);
			h->txn_owned = true;
		}
		ft_flip_txn_record_anchor_release(txn, h, m);
	}
	ft_flip_txn_record_retire_anchored(txn, ctx, h, m);
}

static
void ft_detach_freeze_orphans(struct cds_ft *ft, struct ft_flip_txn *txn,
		const struct ft_lock_ctx *ctx,
		struct cds_ft_inode_flag **orphans, int nr_orphans,
		struct cds_ft_inode_flag *trailing_skip_cn_flag,
		struct ft_held_anchor *held,
		struct ft_held_anchor *trailing_held)
{
	int i;

	for (i = 0; i < nr_orphans; i++) {
		struct cds_ft_metadata *m = ft_node_compressed(orphans[i])
			? cds_ft_item_to_metadata((struct cds_ft_inode *)
				ft_compressed_node_ptr(orphans[i]))
			: cds_ft_item_to_metadata(ft_node_ptr(orphans[i]));

		if (held)
			ft_detach_freeze_one(txn, ctx, &held[i], m);
		else if (txn)
			ft_flip_txn_record_tombstone(txn, m);
		else
			ft_meta_tombstone_set_flip(m);
	}
	if (trailing_skip_cn_flag) {
		struct cds_ft_metadata *m = cds_ft_item_to_metadata(
			(struct cds_ft_inode *) ft_skip_to_compressed(ft,
				trailing_skip_cn_flag));

		if (held)
			ft_detach_freeze_one(txn, ctx, trailing_held, m);
		else if (txn)
			ft_flip_txn_record_tombstone(txn, m);
		else
			ft_meta_tombstone_set_flip(m);
	}
}
/*
 * DLM orphan plan-lock (§9.2, lock_fine): lock acquire orphan @m at collection
 * and record it into ft_detach_node's fn-scope lock arrays (parallel to its
 * to_free[]).  @require_single_child validates the marked snapshot's
 * nr_child == 1 -- the elevated-ancestor invariant the collapse rests on -- so a
 * peer that GREW the orphan between the plan climb and this mark forces a
 * re-descend instead of a silent retire (a plain compressed node is structurally
 * single-child and skips the check; a concurrent restructure of it dirties its
 * word and fails the mark).  Returns -EAGAIN on a dirty word or a grown orphan
 * (caller: `goto end`, whose sweep releases the marks already held); 0 stores
 * {m, snap} at *n and advances it.  The snapshot feeds the fenced
 * {LOCK|s -> TOMBSTONE|s} tombstone at freeze.
 */
/*
 * Byte-depth of @parent_nf's immediate child: one key byte past a bitmap node,
 * the whole run past a compressed one.
 */
static inline
unsigned int ft_child_depth_of(const struct cds_ft *ft,
		struct cds_ft_inode_flag *parent_nf, unsigned int parent_depth)
{
	return parent_depth + ft_node_span(ft, parent_nf);
}

/*
 * EXTEND the descent over one step of a writer walk: position the cursor on
 * @nf at @depth, then enter it so the anchor table covers the levels its span
 * crosses.  The next node down is then anchored by plain ft_descent_anchor,
 * exactly as if the original descent had walked here.  Returns the next node's
 * byte-depth.
 *
 * A no-op under per-node granularity (ft_descent_enter_node returns at once)
 * and where no descent ran.
 */
static inline
unsigned int ft_walk_extend(struct ft_descent *d, bool valid,
		struct cds_ft_inode_flag *nf, unsigned int depth,
		unsigned int span)
{
	if (valid) {
		d->nf = nf;
		d->depth = depth;
		ft_descent_enter_node(d, nf, depth, span);
		d->depth = depth + span;
	}
	return depth + span;
}

/*
 * Acquire an orphan the detach's DOWNWARD walk collected.
 *
 * The walk moves BELOW the descent's cursor, so @depth comes from the walk
 * itself -- ft_walk_extend has entered every node above this one, so the anchor
 * table covers it.  Refusing here instead would be fatal rather than merely
 * costly: remove_all has NO retry loop, so its -EAGAIN surfaces as a hard
 * MEMORY_ERROR.
 */
static inline
int ft_detach_orphan_acquire_at(const char *fn, int line,
		const struct cds_ft *ft,
		const struct ft_lock_ctx *ctx, struct cds_ft_inode_flag *nf,
		unsigned int depth, struct cds_ft_metadata *m,
		struct ft_held_anchor *held)
{
	return ft_acquire_member_at(fn, line, ft, ctx, nf, m, depth, held);
}

#define ft_detach_orphan_acquire(ft, ctx, nf, depth, m, held)		\
	ft_detach_orphan_acquire_at(__func__, __LINE__, (ft), (ctx),	\
		(nf), (depth), (m), (held))

static inline
int ft_detach_orphan_planlock(const struct cds_ft *ft,
		const struct ft_lock_ctx *ctx, struct cds_ft_inode_flag *nf,
		unsigned int depth, struct cds_ft_metadata *m,
		bool require_single_child,
		struct ft_held_anchor *locked, int *n)
{
	struct ft_held_anchor h;

	if (ft_detach_orphan_acquire(ft, ctx, nf, depth, m, &h))
		return -EAGAIN;
	if (require_single_child && ft_state_nr_child(h.node_snap) != 1) {
		if (!h.shared)
			ft_meta_lock_release(h.lock);
		return -EAGAIN;
	}
	locked[(*n)++] = h;
	return 0;
}
/*
 * WHAT A FOLDED COLLAPSE OWES ITS CALLER.  Under @record_only the collapse is
 * RECORDED and not committed, so the chain it retires is still live and linked
 * when it returns.  The nodes are handed back for the caller to reclaim after
 * ITS commit lands.
 *
 * ☠ DEFINED OUTSIDE the FEATURE_FT_SKIP_COMPRESSED block that holds the
 * collapse itself, because struct ft_detach_recompact_out EMBEDS it BY VALUE
 * and that struct exists in every build.  A build with skip-compression off has
 * no collapse to run, but it still needs the type to be complete.
 */
struct ft_chain_compress_reclaim {
	/*
	 * TWO LIFETIMES, and mixing them frees a live node.  The first three are
	 * the RETIRED chain: the caller's commit unlinks them, so they are freed
	 * when it SUCCEEDS.  @new_cn is the merged node this collapse built and
	 * recorded but never published, so it is freed when the caller's commit
	 * ABORTS -- the same split ft_detach_recompact_out draws between
	 * @old_node and @new_flag.
	 */
	struct cds_ft_inode *boundary;		/* the 1-child boundary node */
	struct cds_ft_compressed_node *parent_cn;
	struct cds_ft_compressed_node *child_cn;
	struct cds_ft_compressed_node *new_cn;	/* unpublished: free on ABORT */
};

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
 * collapses) -- re-validated against the boundary's node lock snapshot
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
 * removal, structure untouched); -EAGAIN on a peer conflict (a dirty LOCK
 * mark, a failed plan re-validation, a parked latch in a captured slot, or a
 * commit ABORT -- nothing installed, every fence cleared, the caller's retry
 * re-descends); a positive value when the merge does not apply because the
 * merged length would exceed the compressed-node bound (caller falls back to
 * the non-fused removal, leaving the boundary 1-child internal as before).
 * The replaced nodes (@boundary, @parent_cn, @child_cn) are enumerated
 * explicitly at the commit so a future MCAS can fold each one's sequence
 * counter into the same transaction.
 */
/*
 * Hand a chain-compress RETIRE member's acquire to the commit txn: lift the lock
 * off the ancestor that survives (a no-op where the member's own word carries
 * it) and register the word so every unwind path drains it.  A member that
 * deduped onto a word the set already took owes both to the acquire that first
 * took it.  The tombstone itself is recorded later, at the commit.
 */
static inline
void ft_chain_compress_register_retire(struct ft_flip_txn *txn,
		const struct ft_held_anchor *h, struct cds_ft_metadata *node)
{
	if (h->shared)
		return;
	ft_flip_txn_lock_register(txn, h->lock, h->lock_snap);
	ft_flip_txn_record_anchor_release(txn, h, node);
}

/*
 * @iter_depth is the boundary's byte-depth and @ctx the op's lock context: the
 * whole collapsed chain is a lock-set, and its members anchor by depth.  The set
 * straddles the boundary -- parent_CN and pp lie ABOVE it, the surviving child
 * ONE hop below -- which is exactly the shape §7.1 describes.
 */
/*
 * @shared_txn / @record_only (FOLD, for the rekey's one-decide writer): record
 * this collapse into the caller's txn instead of committing a second time.  Two
 * commits cannot be one decide, and this collapse owning its own txn is exactly
 * what confined ft_detach_node's fold to the SIMPLE shape (BP above min_child)
 * -- which is what the rekey's `nr_child < 3` gate restated.
 * @reclaim is REQUIRED when @record_only.
 */
static
int ft_chain_compress_fused(struct cds_ft *ft,
		struct cds_ft_inode_flag *iter_node_flag,
		unsigned int iter_depth,
		const struct ft_lock_ctx *ctx,
		struct cds_ft_metadata *iter_meta,
		struct cds_ft_inode_flag *surviving_child,
		uint8_t surviving_byte,
		unsigned int plan_nr_child,
		struct ft_ord_cell *dead_cell,
		struct ft_detach_run *run,
		struct cds_ft_inode_flag **orphans,
		int nr_orphans,
		struct cds_ft_inode_flag *trailing_orphan,
		struct ft_held_anchor *orphan_held,
		struct ft_held_anchor *trailing_orphan_held,
		struct cds_ft_node *freeze_leaf,
		long count_delta,
		unsigned int count_reserve,
		struct ft_flip_txn *shared_txn,
		bool record_only,
		struct ft_chain_compress_reclaim *reclaim)
{
	struct cds_ft_compressed_node *parent_cn, *child_cn;
	struct cds_ft_metadata *parent_cn_meta;
	unsigned int parent_len, child_len, merged_len;
	struct cds_ft_metadata *new_cn_meta;
	struct cds_ft_compressed_node *new_cn;
	struct cds_ft_inode_flag *new_cn_flag;
	struct cds_ft_inode_flag **publish_slot;
	struct cds_ft_inode_flag *publish_parent;
	unsigned int pub_depth = 0;
	/*
	 * The boundary's parent and ITS parent, dated once: the acquire arm needs
	 * both as lock-set members, and the publish below needs whichever of them
	 * it republishes into.
	 */
	unsigned int parent_depth = 0, pp_depth = 0;
	struct cds_ft_inode_flag *iter_parent;
	struct ft_flip_txn *txn;
	/*
	 * The three RETIRE members of the collapsed chain.  Each names TWO words
	 * once a coarse spacing splits them: the one its acquire took (released
	 * at registration, since it is an ancestor that SURVIVES) and the node's
	 * own (tombstoned at the commit, and the word every plan re-validation
	 * below must read).
	 */
	struct ft_held_anchor iter_held, pcn_held, ccn_held;

	assert(surviving_child);
	/*
	 * Pre-reserve the commit flip-txn BEFORE any pre-flip side-effect.  The
	 * surviving child's (parent, parent-slot-offset) pair is RECORDED into
	 * @txn by ft_record_child_back_edge below (ft_reparent_record_meta: both
	 * edges co-committed), so an aborted flip discards the pair coherently
	 * -- no settled offset survives against the still-old parent.  With the
	 * txn reserved the publish commits through ft_ord_cell_flip_into without
	 * allocating, so the only OOM points are this reservation and the new_cn
	 * allocation, both BEFORE the build's first side-effect (a concurrent-
	 * writer ABORT surfaces as -EAGAIN with new_cn reclaimed, see the commit
	 * site below).  Created FIRST (its sizing needs only the caller's
	 * arguments) so the LOCK fences below can register with it: every
	 * bail from here on is a ft_flip_txn_destroy or the commit itself, and
	 * both terminal paths drain the fence registry -- no unwind can leak a
	 * fence.
	 */
	if (record_only) {
		/*
		 * FOLD: the caller's txn, which the caller pre-reserved for this
		 * collapse's edges too.  NEVER destroyed on a bail below -- the
		 * caller owns it and may still commit the rest of its plan.
		 */
		txn = shared_txn;
	} else {
		txn = ft_flip_txn_create_bounded(ft, FT_REMOVE_COMMIT_REC_MAX_EDGES + 3
				+ 1 /* §4.B parent guard */
				+ 1 /* back-edge (parent, offset) pair: the state-word edge */
				+ ft_freeze_reserve(ft, (unsigned int) nr_orphans
					+ (trailing_orphan ? 1 : 0))
				+ (freeze_leaf ? FT_HLIST_FREEZE_MAX_EDGES : 0)
				+ count_reserve /* nr_keys walk from publish_parent (R3 fold) */);
	}
	if (!txn)
		return -ENOMEM;	/* nothing touched: caller aborts */
	/*
	 * F2 node lock (doc at ft_meta_lock_acquire): fence the whole
	 * collapsed chain -- the boundary and the two compressed nodes whose
	 * bodies the merged node subsumes -- BEFORE trusting any of their
	 * mutable state.  Each mark's clean snapshot feeds that node's
	 * tombstone expected-old at the commit, so a peer state change under
	 * any fence aborts exactly one side.  A dirty mark (peer proxy, a
	 * concurrent copier, a real retire -- e.g. a peer already re-published
	 * the surviving child and tombstoned the old copy this plan captured)
	 * bails to the caller's retry.
	 */
	if (ft->lock_fine) {
		/*
		 * Acquire the WHOLE chain-compress lock-set
		 * {B, parent_CN, child_CN, publish_parent} in ONE all-or-none MCAS,
		 * replacing the three incremental retire-marks + the publish_parent
		 * lock_or_guard.  B / parent_CN / child_CN are RETIRE members (their
		 * bodies subsume into new_cn); publish_parent is the value-swap RELEASE
		 * target.  Guards B.parent==iter_parent and parent_CN.parent==pp ride
		 * the same commit; child_CN needs no guard (the post-acquire
		 * surviving_byte->surviving_child re-validation covers its position,
		 * and a peer split retires it -> ft_dlm_lock -EAGAIN).
		 */
		struct cds_ft_metadata *parent_cn_meta_l, *child_cn_meta_l = NULL;
		struct cds_ft_inode_flag *pp_flag = NULL;
		struct ft_dlm_member set[4];
		int nr_set = 0, si = 0, dret;

		iter_parent = (struct cds_ft_inode_flag *)
			ft_parent_node(rcu_dereference(iter_meta->parent_word));
		if (caa_unlikely(ft_node_flip_proxy(iter_parent))) {
			if (!record_only)
				ft_flip_txn_destroy(txn);
			return -EAGAIN;
		}
		parent_cn = ft_node_compressed(iter_parent)
			? ft_compressed_node_ptr(iter_parent) : NULL;
		parent_cn_meta_l = parent_cn
			? cds_ft_item_to_metadata((struct cds_ft_inode *) parent_cn)
			: NULL;
		child_cn = ft_node_compressed(surviving_child)
			? ft_compressed_node_ptr(surviving_child) : NULL;
		if (child_cn)
			child_cn_meta_l = cds_ft_item_to_metadata(
				(struct cds_ft_inode *) child_cn);
		/* Publish grandparent = parent_CN's parent, else B's parent. */
		if (parent_cn_meta_l)
			(void) ft_resolve_parent_slot(parent_cn_meta_l, ft, &pp_flag);
		else
			pp_flag = iter_parent;

		/*
		 * Both members above the boundary were reached by back-pointer, so
		 * neither carries a depth: date each from the node BELOW it -- the
		 * boundary is at @iter_depth, and where @pp_flag is the compressed
		 * parent's own parent it is one hop above that.  (Where there is no
		 * compressed parent, @pp_flag IS @iter_parent and shares its depth.)
		 * A member neither the window nor the hop can date voids the plan.
		 */
		if (!ft_lock_ctx_depth_of_parent(ft, ctx, iter_parent, iter_depth,
					&parent_depth)) {
			if (!record_only)
				ft_flip_txn_destroy(txn);
			return -EAGAIN;
		}
		if (pp_flag && !(parent_cn_meta_l ?
				ft_lock_ctx_depth_of_parent(ft, ctx, pp_flag,
					parent_depth, &pp_depth) :
				ft_lock_ctx_depth_of(ft, ctx, pp_flag, &pp_depth))) {
			if (!record_only)
				ft_flip_txn_destroy(txn);
			return -EAGAIN;
		}

		set[nr_set++] = (struct ft_dlm_member){ .nf = iter_node_flag,
			.node = iter_meta, .depth = iter_depth,
			.guard_child = iter_meta, .guard_pf = iter_parent };
		if (parent_cn_meta_l)
			set[nr_set++] = (struct ft_dlm_member){ .nf = iter_parent,
				.node = parent_cn_meta_l, .depth = parent_depth,
				.guard_child = parent_cn_meta_l, .guard_pf = pp_flag };
		if (child_cn_meta_l)
			/*
			 * The one member BELOW the pivot (§7.1): an immediate child
			 * of the boundary, so it starts one internal-node hop past
			 * it.
			 */
			set[nr_set++] = (struct ft_dlm_member){ .nf = surviving_child,
				.node = child_cn_meta_l,
				.depth = iter_depth + 1 };
		if (pp_flag)
			set[nr_set++] = (struct ft_dlm_member){ .nf = pp_flag,
				.node = ft_flag_to_metadata(ft, pp_flag),
				.depth = pp_depth };

		dret = ft_dlm_acquire_set(ft, ctx, set, nr_set);
		if (dret) {
			if (!record_only)
				ft_flip_txn_destroy(txn);
			return dret;	/* -EAGAIN / -ENOMEM; nothing acquired */
		}
		parent_cn_meta = parent_cn_meta_l;
		iter_held = set[si++].held;
		ft_chain_compress_register_retire(txn, &iter_held, iter_meta);
		if (parent_cn_meta_l) {
			pcn_held = set[si++].held;
			ft_chain_compress_register_retire(txn, &pcn_held,
				parent_cn_meta_l);
		}
		if (child_cn_meta_l) {
			ccn_held = set[si++].held;
			ft_chain_compress_register_retire(txn, &ccn_held,
				child_cn_meta_l);
		}
		if (pp_flag) {
			/*
			 * publish_parent is the value-swap RELEASE target (survives).
			 * Record its {LOCK|s -> s} release + register it up front with
			 * the acquire, so the publish below skips its lock_or_guard and
			 * every pre-publish bail's ft_flip_txn_destroy drains it -- no
			 * separate publish_parent unwind path.  A member that deduped
			 * onto a word the set already took owes neither.
			 */
			if (!set[si].held.shared) {
				ft_flip_txn_lock_register(txn, set[si].held.lock,
					set[si].held.lock_snap);
				ft_flip_txn_record_release_lock(txn,
					set[si].held.lock, set[si].held.lock_snap);
			}
			si++;
		}
	} else
	{
		if (ft_acquire_member(ft, ctx, iter_node_flag, iter_meta,
				iter_depth, &iter_held)) {
			if (!record_only)
				ft_flip_txn_destroy(txn);
			return -EAGAIN;
		}
		ft_chain_compress_register_retire(txn, &iter_held, iter_meta);
		/*
		 * ONE snapshot of the boundary's parent (latch-checked): the F1
		 * discipline -- a peer's parked flip proxy must be neither classified
		 * nor embedded.
		 */
		iter_parent = (struct cds_ft_inode_flag *)
			ft_parent_node(rcu_dereference(iter_meta->parent_word));
		if (caa_unlikely(ft_node_flip_proxy(iter_parent))) {
			if (!record_only)
				ft_flip_txn_destroy(txn);
			return -EAGAIN;
		}
		parent_cn = ft_node_compressed(iter_parent)
			? ft_compressed_node_ptr(iter_parent)
			: NULL;
		parent_cn_meta = parent_cn
			? cds_ft_item_to_metadata((struct cds_ft_inode *) parent_cn)
			: NULL;
		if (parent_cn_meta) {
			if (!ft_lock_ctx_depth_of_parent(ft, ctx, iter_parent,
						iter_depth, &parent_depth) ||
					ft_acquire_member(ft, ctx, iter_parent,
						parent_cn_meta, parent_depth,
						&pcn_held)) {
				if (!record_only)
					ft_flip_txn_destroy(txn);
				return -EAGAIN;
			}
			ft_chain_compress_register_retire(txn, &pcn_held,
				parent_cn_meta);
		}
		child_cn = ft_node_compressed(surviving_child)
			? ft_compressed_node_ptr(surviving_child)
			: NULL;
		if (child_cn) {
			/* The §7.1 below-pivot member: one hop past the boundary. */
			if (ft_acquire_member(ft, ctx, surviving_child,
					cds_ft_item_to_metadata(
						(struct cds_ft_inode *) child_cn),
					iter_depth + 1, &ccn_held)) {
				if (!record_only)
					ft_flip_txn_destroy(txn);
				return -EAGAIN;
			}
			ft_chain_compress_register_retire(txn, &ccn_held,
				cds_ft_item_to_metadata(
					(struct cds_ft_inode *) child_cn));
		}
	}
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
	if (caa_unlikely(ft_state_nr_child(iter_held.node_snap) != plan_nr_child ||
			ft_node_get_nth(ft, iter_node_flag, NULL,
				surviving_byte, FT_PF_NONE)
					!= surviving_child)) {
		if (!record_only)
			ft_flip_txn_destroy(txn);
		return -EAGAIN;
	}
	parent_len = parent_cn ? parent_cn->len : 0;
	child_len = child_cn ? child_cn->len : 0;
	merged_len = parent_len + 1 + child_len;

	if (merged_len > FT_SKIP_LEN_MAX) {
		/* Merge does not apply: caller falls back (fences cleared). */
		if (!record_only)
			ft_flip_txn_destroy(txn);
		return 1;
	}
	new_cn = alloc_compressed_node(ft, merged_len, &new_cn_meta);
	if (!new_cn) {
		if (!record_only)
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
			if (!record_only)
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
		new_cn_meta->parent_word = ft_parent_word(ft, publish_parent);
	} else {
		/*
		 * Replace the boundary at its own slot in iter_parent.  Resolve
		 * that slot COHERENTLY from iter_meta (Phase 4.3 atomic re-home),
		 * exactly as the parent_cn arm resolves parent_cn_meta -- never
		 * reuse a slot the caller precomputed with ft_get_parent_slot
		 * before this op's fence.  A peer that recompacts the boundary's
		 * parent (growing it to a larger node type) or re-homes it between
		 * that snapshot and here leaves the precomputed slot pointing into
		 * the OLD, now-replaced parent body, while iter_meta->parent
		 * already names the NEW parent: pairing the stale slot with the
		 * fresh parent indexes the new parent's bitmap with an
		 * out-of-bounds rank (ft_slot_to_byte OOB).  Require the resolved
		 * parent to still be the latch-checked iter_parent the plan above
		 * was validated against; a re-home that changed it lands us on an
		 * unvalidated parent -- reclaim the unpublished merged node and
		 * retry.
		 */
		publish_slot = ft_resolve_parent_slot(iter_meta, ft,
			&publish_parent);
		if (caa_unlikely(publish_parent != iter_parent)) {
			free_compressed_node_unpublished(ft, new_cn);
			if (!record_only)
				ft_flip_txn_destroy(txn);
			return -EAGAIN;
		}
		new_cn_meta->parent_word = ft_parent_word(ft, iter_parent);
	}
	ft_set_parent_slot(new_cn_meta,
			ft_parent_node(new_cn_meta->parent_word), publish_slot);

	/*
	 * The publish target is a lock-set member (the value-swap RELEASE half),
	 * and it was resolved through a back-pointer, so the descent's window is
	 * what dates it.  Checked here rather than at the acquire below: nothing
	 * is reader-visible yet, so a target this descent cannot date is a clean
	 * re-plan.  It is the parent of whichever node this publish REPLACES --
	 * the compressed parent, else the boundary -- so the one-hop derivation
	 * dates it where the window cannot.  Under the DLM arm the same node was
	 * already anchored as @pp.
	 */
	if (!ft_lock_ctx_depth_of_parent(ft, ctx, publish_parent,
			parent_cn ? parent_depth : iter_depth, &pub_depth)) {
		free_compressed_node_unpublished(ft, new_cn);
		if (!record_only)
			ft_flip_txn_destroy(txn);
		return -EAGAIN;
	}

	new_cn_flag = ft_compressed_node_flag(new_cn);
	{
		struct ft_pub_rec rec = { .n = 0 };
		struct cds_ft_inode_flag *new_cn_pub;
		/*
		 * Plan snapshot of publish_slot's old value: it still holds the
		 * collapsed chain's reference (parent_cn's grandparent flag, or
		 * @iter_node_flag) resolved just above -- nothing has been stored
		 * yet.  The commit CAS rejects a peer that raced it (see
		 * ft_pub_rec_add).
		 *
		 * WAITING load, not a raw one: @publish_slot is recorded as an
		 * edge of THIS txn a few lines down, so it enters the txn's own
		 * write set and the read-policy rule is to wait out an undecided
		 * parker.  "The commit CAS rejects a peer that raced it" holds
		 * for a stale PLAIN value and not for a parked flip proxy: that
		 * is a descriptor-record POINTER, and handing it to the engine as
		 * an expected-old trips urcu_txn_add's !urcu_txn_is_proxy check
		 * (--enable-rcu-debug; a release build POISONS the descriptor and
		 * the retry loop absorbs it, so the arm is green and wrong).
		 * Only a coarse spacing exposes it -- under per-node this op
		 * holds @publish_parent's own word, so no peer can park here.
		 */
		struct cds_ft_inode_flag *pub_expected_old =
			urcu_txn_load(txn->mtxn, (void **) publish_slot,
				FT_FLIP_PROXY_TAG);

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
		ft_record_child_back_edge(ft, txn, new_cn->child,
			new_cn_flag, &new_cn->child);
		new_cn_pub = ft_publish_compressed(ft, new_cn, new_cn_flag);
		/* VALIDATE (§4.B): lock (or guard-fallback) the LIVE
		 * (great-)grandparent publish_parent -- value-swap target (§10.5).
		 * DLM: under lock_fine the whole lock-set (incl. publish_parent's
		 * RELEASE) was acquired up front, so skip the incremental lock here. */
		if (!ft->lock_fine)
			ft_flip_txn_lock_or_guard_parent(ft, txn, ctx,
				publish_parent, pub_depth);
		_ft_publish_to_parent_meta(ft, publish_parent, publish_slot,
			new_cn_pub, pub_expected_old, new_cn_meta, NULL, &rec,
			/*slot_owner_nf=*/ publish_parent);
		/*
		 * Freeze-on-free (doc §4.B, atomic detach): the collapsed chain
		 * this commit retires -- the 1-child boundary @iter_node_flag and
		 * the old parent/child compressed nodes new_cn merges (<=3) --
		 * gets its one-way LIVE->DEAD tombstone RECORDED INTO @txn (the +3
		 * reserved above), so the freeze flips ATOMICALLY with the commit
		 * that unlinks it: an aborted commit leaves every node live.  A
		 * no-op under one writer.  All three are FENCED (marked LOCK
		 * above), so each expected old is its mark's clean snapshot: the
		 * commit ratifies exactly the chain state this merge was built
		 * from.  The fence is consumed by the fused
		 * {LOCK|s -> TOMBSTONE|s} where the member carries its own lock,
		 * and by the release recorded at registration where coarsening
		 * left the lock on a surviving ancestor.
		 */
		ft_flip_txn_record_retire_anchored(txn, ctx, &iter_held,
			iter_meta);
		if (parent_cn)
			ft_flip_txn_record_retire_anchored(txn, ctx, &pcn_held,
				parent_cn_meta);
		if (child_cn)
			ft_flip_txn_record_retire_anchored(txn, ctx, &ccn_held,
				cds_ft_item_to_metadata(
					(struct cds_ft_inode *) child_cn));
		ft_detach_freeze_orphans(ft, txn, ctx, orphans, nr_orphans,
			trailing_orphan, orphan_held, trailing_orphan_held);
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
		if (ft_remove_commit_rec(ft, &rec, dead_cell, run, txn,
				record_only) > 0) {
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

	if (record_only) {
		/*
		 * ☠ NOT FREED HERE.  Under the fold nothing has committed yet, so
		 * this chain is still LIVE and LINKED: these frees would reclaim
		 * nodes the caller's pending commit has not unlinked, and a reader
		 * is entitled to be walking them.  Hand them back instead -- the
		 * free path is the CALLER's choice (@913cf0ca).
		 */
		reclaim->boundary = ft_node_ptr(iter_node_flag);
		reclaim->parent_cn = parent_cn;
		reclaim->child_cn = child_cn;
		/*
		 * The merged node is RECORDED, not published: the caller's commit
		 * is what publishes it, so an abort leaves it ours to reclaim.
		 * (The non-folded path cannot reach here with it unpublished --
		 * every bail above frees it and returns.)
		 */
		reclaim->new_cn = new_cn;
		return 0;
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
		unsigned int iter_depth,
		const struct ft_lock_ctx *ctx,
		struct cds_ft_metadata *iter_meta)
{
	uint8_t surviving_byte = 0;
	struct cds_ft_inode_flag *surviving_child;

	surviving_child = ft_node_get_minmax(ft, iter_node_flag,
		&surviving_byte, FT_LEFTMOST,
		false /* writer; no validation */);
	if (!surviving_child)
		return;
	(void) ft_chain_compress_fused(ft, iter_node_flag, iter_depth, ctx,
		iter_meta,
		surviving_child, surviving_byte,
		1 /* already-committed 1-child boundary */, NULL, NULL,
		NULL, 0, NULL, NULL, 0 /* no orphan chain */,
		NULL, 0 /* count-neutral canonicalize */, 0,
				NULL, false, NULL);
}
#endif

/*
 * FOLD (coherent rekey one-decide writer): the record_only detach's src-junction
 * recompaction produces two copies the CALLER must reclaim after its own commit --
 * the OLD copy (retired by the flip, freed on commit OK) and the fresh NEW copy
 * (republished by the flip, freed UNPUBLISHED if the caller's commit ABORTS).
 * ft_detach_node surfaces both here (zeroed = no recompaction happened / not
 * record_only); NULL @recompact_out means the caller does not run a record_only
 * detach.  An ELEVATING detach adds a third group with the OLD copy's lifetime:
 * the orphan chain the upward walk cleared.
 */
struct ft_detach_recompact_out {
	struct cds_ft_inode *old_node;		/* retired copy: free on commit OK */
	struct cds_ft_inode_flag *new_flag;	/* fresh copy: free unpublished on abort */
	/*
	 * A FOLDED COLLAPSE's nodes, when the detach's boundary parent fell to
	 * one child and the collapse was recorded into the caller's txn rather
	 * than committed.  Same two lifetimes as the pair above; @boundary is
	 * NULL when no collapse happened.
	 */
	struct ft_chain_compress_reclaim collapse;
	/*
	 * An ELEVATING detach's orphan chain: the ancestors the upward walk
	 * cleared plus the chain below the detach target, and the trailing
	 * skip-compressed target if there is one.  Same lifetime as @old_node --
	 * the caller's commit is what UNLINKS them, so they are freed when it
	 * SUCCEEDS and left alone when it aborts (nothing unlinked them; a
	 * reader is still entitled to be walking the chain).  Their
	 * freeze-on-free tombstones were recorded into the caller's shared txn,
	 * so they flip with that same unlink.
	 */
	struct cds_ft_inode_flag *orphans[FT_MAX_DEPTH];
	int nr_orphans;
	struct cds_ft_inode_flag *orphan_trailing;
};

static
int ft_detach_node(struct cds_ft *ft,
		const struct ft_lock_ctx *op_ctx,
		struct cds_ft_inode_flag **detach_node_flag_ptr,
		struct cds_ft_inode_flag **detach_parent_flag_ptr,
		unsigned int detach_depth,
		bool free_detached_subtree,
		struct ft_ord_cell *fuse_cell,
		struct ft_remove_pub *pub,
		struct ft_detach_run *run,
		struct ft_glue *retire_glue,
		struct cds_ft_node *freeze_leaf,
		long count_delta,
		struct ft_flip_txn *shared_txn,
		bool record_only,
		const struct ft_parent_hint *src_held_hint,
		struct ft_detach_recompact_out *recompact_out)
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
	/*
	 * THE BOUNDARY THE CLIMB ACTUALLY LANDED ON, and its slot.  A caller's
	 * @src_held_hint describes the DETACH TARGET's junction, which stops
	 * being the recompacted node the moment the climb ELEVATES: it then
	 * recompacts an ANCESTOR, whose parent is one level higher again.
	 * Forwarding the caller's hint there points the republish -- and the
	 * read-set guard that validates it -- at the wrong node.  These name
	 * the real pair, re-derived by the climb that moved.
	 */
	struct cds_ft_inode_flag *boundary_parent_nf = NULL;
	bool climbed = false;
	struct ft_parent_hint elevated_hint;
	const struct ft_parent_hint *replace_hint = src_held_hint;
	/*
	 * DLM orphan plan-lock (§9.2): under
	 * ft->lock_fine every collected orphan (Block A or Block B, both mutually
	 * exclusive) is lock-acquired at collection so the nr_child==1 collapse
	 * decision is made on a FROZEN state word; its clean snapshot feeds the
	 * FENCED {LOCK|s -> TOMBSTONE|s} tombstone at freeze (a peer that grows
	 * the orphan tears the expected-old and aborts our commit).  The marks live
	 * in these fn-scope arrays (not the txn locks[] registry -- the chain
	 * reaches FT_MAX_DEPTH, past FT_FLIP_TXN_MAX_LOCKS), so the op OWNS their
	 * clearing: one unconditional ft_meta_lock_release_if_held sweep at @end
	 * releases every held mark and no-ops on the ones a successful commit already
	 * consumed (TOMBSTONE), needing no per-commit "consumed" bookkeeping.
	 *
	 * Sized for the chain PLUS the trailing skip-target, which is one more
	 * orphan and belongs in the same array: a mark kept in a variable of its
	 * own is a mark no later acquire can see, and a coarse spacing then lands
	 * the next member's anchor on it (the op waits on itself; remove_all has
	 * no retry loop, so that surfaces as MEMORY_ERROR).
	 */
	struct ft_held_anchor orphan_held[FT_MAX_DEPTH + 1];
	int nr_orphan_locked = 0;
	/*
	 * This op's lock context: the caller's anchor source, plus the words
	 * THIS function holds.  The orphan marks above are part of the held set
	 * even though they sit outside any txn registry -- an acquire that
	 * cannot see them re-takes one and waits on the op itself.  @nr_extra is
	 * refreshed at each acquire because the orphan walk is still growing it.
	 */
	struct ft_lock_ctx lctx;
	/*
	 * The op's descent, COPIED so the orphan walks below may EXTEND it.
	 * Those walks move DOWN a chain past the descent's cursor, and their
	 * nodes' depths exist nowhere else: feeding each step through
	 * ft_descent_enter_node keeps ONE depth-tracking mechanism and ONE
	 * anchor table for the whole op, which is what agreement wants
	 * (doc/design/ft-dlm-lock-coarseness.md §9).  A copy, not the caller's,
	 * because the extension is this op's business alone.
	 */
	struct ft_descent wd;
	bool wd_valid = false;
	/*
	 * The trailing skip-target's mark, IN @orphan_held (never a copy): the
	 * held set is what the next acquire consults, and the freeze needs the
	 * same entry the release sweep will clear.  NULL until it is taken.
	 */
	struct ft_held_anchor *orphan_trailing_held = NULL;
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
	unsigned int cur_span;
	/*
	 * Snapshot of the slot value at the detach point.  Set once the
	 * upward walk finishes elevating @detach_node_flag_ptr, before
	 * ft_node_replace_ptr overwrites the slot.  The free-walk below
	 * starts from this value so it covers BOTH the elevated single-
	 * child ancestors and the original detach target (the elevation
	 * walk only crosses nodes with nr_child==1, so the underlying
	 * chain reaches the original detach child).
	 *
	 * It is equally the PLAN EXPECTED-OLD handed to ft_node_replace_ptr:
	 * the orphan set, the count fold and the drop all name THIS subtree,
	 * so a DEL recompaction that finds another one at the slot is working
	 * from a plan a peer has already invalidated (-EAGAIN, re-descend).
	 */
	struct cds_ft_inode_flag *elevated_old_child;
	/*
	 * The same slot's value as the CLIMB read it -- the plan's expected-old.
	 *
	 * The climb decides "every level from the holder up to here holds only the
	 * branch we are removing, so prune at this slot" from the nr_child of the
	 * nodes it walks.  That verdict is about the SUBTREE the slot held THEN.  A
	 * peer that republishes the slot mid-climb (an insert splitting the chain
	 * below, whose fresh junction carries the removed key AND the peer's own)
	 * makes the verdict false, and re-reading the slot after the climb adopts
	 * the peer's subtree into the plan -- the prune then unlinks a live key,
	 * silently, with a byte and a child count that are both innocent.  So the
	 * value is captured WHERE THE CLIMB USES IT: at entry for a plan that never
	 * elevates, and at each elevation for the level it just decided to prune.
	 *
	 * The FIRST elevation is the load-bearing one and gets its value from the
	 * SAME load that produced @cur (@entry_holder_raw below), not from a second
	 * read: the two are ~300 ns apart and a peer publish lands between them.
	 */
	struct cds_ft_inode_flag *plan_old_child;
	/*
	 * The holder slot the climb starts from, and the ONE raw value it read
	 * there -- the value @cur was resolved from.  When the climb elevates, that
	 * slot becomes the drop target, so this pair IS its plan expected-old.
	 */
	struct cds_ft_inode_flag **entry_holder_slot;
	struct cds_ft_inode_flag *entry_holder_raw;
	/*
	 * Snapshot of the holder slot (@detach_parent_flag_ptr) taken BEFORE
	 * ft_node_replace_ptr overwrites @iter_node_flag with the fresh
	 * recompacted copy: the plan-snapshot expected-old for the forward
	 * republish that flips the holder slot to that copy (defect #1).  In the
	 * external-promote sub-case (no recompaction) it equals the unchanged
	 * @iter_node_flag, so the same-value republish still holds.
	 */
	struct cds_ft_inode_flag *holder_old_flag;
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
	 * Force the list-off lone forward store through a commit txn (MW safety +
	 * nr_keys fold).  When the caller supplies no @pub (list-off: there is no
	 * ordered-list cell to fuse), an in-place external promote would otherwise
	 * store bare into the LIVE holder's child slot via ft_node_replace_ptr's
	 * lone-edge arm (ft_node_child_edge_flip) -- a blind rcu_assign that under
	 * MW resurrects a peer's relocated node / smashes a parked latch and is NOT
	 * §4.B-guarded (the concurrent-recompaction premise requires EVERY live
	 * child-slot mutation to be a proxy'd, holder-serialized txn).  Defer that
	 * store into a LOCAL pub instead: the pub-armed arm below commits it via
	 * ft_remove_one_commit through @commit_txn (holder guard + structural edge)
	 * and -- when order statistics are on -- records the -@count_delta walk from
	 * the surviving holder into the SAME commit, exactly as the list-on fused
	 * path does.  This is the single choke point for every ft_detach_node caller
	 * (leaf remove, remove_all, whole-subtree detach, merge src-unlink,
	 * graft_swap extract): each passes pub == NULL in list-off, and each is now
	 * routed uniformly through the txn.  A recompaction shape never arms @pub,
	 * so it is unaffected (its rebuilt copy is published by the recompaction
	 * path below); when !rank_stats @count_delta is 0 so the fold is a no-op and
	 * only the guarded structural edge commits.  @local_pub outlives every use
	 * (whole-function scope); ft_detach_node never returns @pub.
	 */
	struct ft_remove_pub local_pub = { .armed = false };

	if (!pub)
		pub = &local_pub;

	if (ft_lock_ctx_descent(op_ctx)) {
		wd = *ft_lock_ctx_descent(op_ctx);
		wd_valid = true;
	}
	ft_lock_ctx_init(&lctx, wd_valid ? &wd : NULL, NULL,
		op_ctx ? op_ctx->op : NULL);
	lctx.held.extra = orphan_held;
	/*
	 * This frame keeps its OWN out-of-registry array (@orphan_held), so the
	 * caller's would be lost: CHAIN to it.  The rekey fold arrives here
	 * holding ft_rekey_cow_stop's marks, and under a coarse spacing S_top's
	 * fence lands on the very node this detach recompacts.
	 */
	lctx.held.outer = op_ctx ? &op_ctx->held : NULL;

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
			/*
			 * Resolve a parked flip proxy before capture: a concurrent
			 * one-commit splice (ft_insert_park_external_nodes)
			 * transiently installs its descriptor in external_nodes, and
			 * this value is republished RAW into cn->child + the SKIP_X
			 * dual by the external-promote below.  Snapshotting a peer's
			 * latch into a structural slot leaves a dangling descriptor
			 * once that txn settles and reclaims -- and the SKIP_X form
			 * dispatches the 0xF flag as a node (ft_node_external fails ->
			 * item_to_metadata faults).  Mirror every ft_dereference_external
			 * descent reader; the common no-splice case is one masked test.
			 */
			if (child_meta && child_meta->external_nodes)
				topmost_external_nodes = ft_dereference_external(
					child_meta->external_nodes);
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
	 * Resolve the initial parent slot.  Two layers:
	 *
	 *  - Flip-proxy: @detach_parent_flag_ptr is a forward child slot (the
	 *    grandparent's slot to the holder), which a concurrent recompaction
	 *    re-home transiently parks a type-7 MCAS proxy in.  A raw load feeds
	 *    that proxy to cds_ft_item_to_metadata() at the top of the up-walk
	 *    below and faults (the SAME crash the loop-body parent read guards
	 *    against, but on iteration 0).  Resolve it exactly as cds_ft_remove
	 *    does at its compressed-child identity compare.  @cur is only ever
	 *    dereferenced here (never a CAS expected-old), so resolving is sound.
	 *
	 *  - Skip-compressed: when the detach is bootstrapped from a leaf whose
	 *    holder is a compressed node (e.g. cds_ft_remove deriving the
	 *    position from node->prev), the slot is skip-encoded (SKIP_X(cn->
	 *    child)); ft_node_ptr does not strip the skip high bits, so resolve
	 *    to the plain compressed flag.  No-op for a plain (descent) slot.
	 */
	{
		/*
		 * Reanchor a skip-compressed initial slot through the shared
		 * read-side primitive (MW convergence) rather than the unvalidated
		 * one-hop ft_resolve_skip_compressed: a peer split can tear the
		 * one-hop recovery to a node whose ft_meta_nr_child reads a bitmap
		 * byte (the skip-gated ft-remove.h prune-climb 0-child assert).  @cur
		 * is only ever dereferenced (never a CAS expected-old), so using the
		 * reanchored LIVE node is sound; a rewind is tolerated (the climb
		 * self-corrects / the commit guards it).
		 */
		unsigned int cur_rewind;

		/*
		 * ONE load feeds both @cur and @entry_holder_raw: the climb's
		 * whole verdict about this level is derived from @cur, so the
		 * expected-old that says "the slot still holds what I walked"
		 * must be the SAME read that produced it.  A second load a few
		 * hundred nanoseconds later is a coherent-pair violation, and
		 * MEASURED to be one -- a peer's publish landing between the two
		 * is captured as the plan's own expected value and then pruned.
		 */
		entry_holder_raw = (struct cds_ft_inode_flag *)
			rcu_dereference(*detach_parent_flag_ptr);
		cur = ft_reanchor_flag(ft,
			ft_resolve_flip_proxy(entry_holder_raw),
			&cur_rewind);
	}
	entry_holder_slot = detach_parent_flag_ptr;
	/*
	 * HOLDER IDENTITY: @cur must be the very node @detach_node_flag_ptr
	 * addresses a slot inside.  The caller reached that slot by descending
	 * through the holder; @cur is a SEPARATE, later load of the slot that
	 * holds the holder.  A peer that republishes the holder between the two
	 * -- an insert splitting the compressed chain here, whose fresh copy
	 * carries the removed key AND its own -- leaves the pair disagreeing:
	 * the slot addresses the retired body, @cur names the fresh copy.
	 *
	 * The climb then walks the WRONG node.  It is fatal precisely because
	 * the fresh copy looks prunable: a compressed node structurally holds
	 * exactly one child, so the climb scores it a single-child ancestor,
	 * elevates past it, and drops the branch whole -- the multi-child
	 * junction the peer published UNDER it is never looked at, and the peer's
	 * key goes with the prune.  No expected-old on the holder slot can see
	 * this: that slot's value agrees with itself at every load (the climb
	 * elevates ONTO it and both reads return the fresh copy).  What is stale
	 * is the DESCENT's premise -- "the chain below this holder holds only the
	 * key I am removing" -- and this is where that premise is checkable.
	 *
	 * Nothing is built, locked or reserved yet: re-descend against the
	 * settled tree.  The invariant holds by construction across an elevation
	 * below (the new @detach_node_flag_ptr is the slot ft_get_parent_slot
	 * recovered INSIDE the new @cur), so it is tested once, here.
	 */
	if (caa_unlikely(!ft_slot_in_node(cur, detach_node_flag_ptr)))
		return -EAGAIN;
	/* Plan expected-old for a detach that never elevates (see @plan_old_child). */
	plan_old_child = (struct cds_ft_inode_flag *)
		rcu_dereference(*detach_node_flag_ptr);
	/*
	 * @cur HOLDS the detached child's slot, so it starts a full SPAN above
	 * it -- one key byte for a bitmap node, the whole run for a compressed
	 * one, which is what ft_node_span answers.  A span wider than the
	 * child's own depth means @cur is not the parent of anything at that
	 * depth: the plan is stale, so re-descend (nothing is built, locked or
	 * reserved yet, exactly as the bail above).
	 */
	cur_span = ft_node_span(ft, cur);
	if (caa_unlikely(cur_span > detach_depth))
		return -EAGAIN;
	cur_depth = detach_depth - cur_span;

	/*
	 * nr_keys count fold (LEAF Increment 2): no standalone pre-decrement
	 * here anymore -- each per-outcome commit below folds the @count_delta
	 * walk onto its own flip so the count flips ATOMICALLY with the unlink
	 * (exact under concurrent writers).  See @count_folded.
	 */

	while (cur) {
		struct cds_ft_metadata *metadata;
		struct cds_ft_inode_flag *resolved_parent;
		unsigned int nr_child;
		bool is_root;

		metadata = cds_ft_item_to_metadata(ft_node_ptr(cur));
		metadata_stack[nr_metadata++] = metadata;
		/*
		 * ONE proxy-resolved snapshot of this ancestor's parent
		 * back-pointer, consumed by every use below (the is_root
		 * boundary test, the boundary parent_meta classify, and the
		 * climb).  &metadata->parent is a flip-txn parked slot
		 * (ft_reparent_record_meta), so a concurrent recompaction
		 * re-home transiently parks a type-7 proxy here; a raw load fed
		 * to cds_ft_item_to_metadata() computes metadata off the
		 * latch/record memory and faults (the dominant FT_INV_MW
		 * up-walk crash), and a raw non-NULL proxy also mis-answers
		 * is_root.  Resolve once (a predicted-not-taken mask-compare
		 * when no re-home is in flight -- single-writer unchanged),
		 * exactly as the reader up-walk ft_skip_reanchor does.
		 */
		resolved_parent = ft_resolve_flip_proxy(
			(struct cds_ft_inode_flag *) ft_parent_node(
				rcu_dereference(metadata->parent_word)));
		is_root = (resolved_parent == NULL);
		boundary_parent_nf = resolved_parent;	/* always names @cur */
		/*
		 * ONE proxy-resolved snapshot of this ancestor's child count, for
		 * exactly the same two reasons as @resolved_parent just above --
		 * and &metadata->state is the SAME kind of parked slot.
		 *
		 * RESOLVED: a peer's in-flight commit parks an engine record
		 * POINTER in the state word (FT_STATE_PROXY, bit 0).  A raw
		 * ft_meta_nr_child() then decodes bits 2-10 of that pointer as the
		 * count -- an arbitrary value, almost always > 1, which answers
		 * "this ancestor is a surviving multi-child boundary" YES for a
		 * node that in truth holds only the child we came up from.  The
		 * prune then stops one level too low and ft_node_replace_ptr
		 * DELs the boundary's last child, the case ft_node_recompact's
		 * NODE_INDEX_NULL assert catches at the dereference site.
		 *
		 * ONCE: the three tests below (the emptied-ancestor bail, the
		 * nr_clear tally and the boundary stop) must agree on one value.
		 * Three separate loads of a word peers mutate can disagree, and a
		 * plan built from two different counts is incoherent even when
		 * each load is individually resolved.
		 *
		 * Off the commit window urcu_txn_read short-circuits to a plain
		 * load, so single-writer is unchanged.
		 */
		nr_child = ft_meta_nr_child_load(metadata);

		/*
		 * A climbed ancestor with nr_child == 0 means a peer is concurrently
		 * recompacting / emptying a shared spine node (skip-compression drives
		 * this churn, hence skip-gated): our prune plan is racing that peer.
		 * Bail before any build and re-descend against the settled tree,
		 * exactly as the flip-proxy holder bail below.  Under mutual exclusion
		 * this is impossible -- a climbed ancestor always still holds the child
		 * we came up from (nr_child >= 1) -- so the pre-MW invariant assert
		 * becomes an MW retry point, not a fatal abort.
		 */
		if (caa_unlikely(nr_child == 0))
			return -EAGAIN;
		if (!prev_external_nodes_found && (nr_child == 1 && !metadata->external_nodes && !is_root)) {
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
		if (prev_external_nodes_found || nr_child > 1 ||
		    (metadata->external_nodes && topmost_external_nodes) ||
		    is_root) {
			if (!is_root) {
				struct cds_ft_metadata *parent_meta;

#ifdef FT_ENABLE_TRACING
				/*
				 * THE CLIMB'S DEREFERENCE, checked before it
				 * happens.  This is the frame the residual SEGV
				 * faults in, and it is the signature that
				 * SURVIVES tracing (the reader-side one does
				 * not: 0/2100 traced against ~1%/run untraced).
				 *
				 * A freelist link is 8-mod-16, so it clears the
				 * EXTERNAL tag -- a reclaimed parent reads as an
				 * external node here, which is exactly the
				 * invariant ft_get_parent_rcu asserts and this
				 * climb never checked.  Round-trip it so the
				 * event alone separates recycled memory from a
				 * live object with a bad link.
				 */
				if (caa_unlikely(resolved_parent &&
						ft_node_external(resolved_parent))) {
					const void *pp = ft_node_ptr(resolved_parent);

					FT_TP(parent_external_violation,
						(const void *) cur, pp,
						(const void *) cds_ft_metadata_to_item(
							cds_ft_item_to_metadata(
							(struct cds_ft_inode *) pp)),
						(unsigned long) metadata->parent_word);
					ft_trace_capture();
					abort();
				}
#endif
				parent_meta = cds_ft_item_to_metadata(
						ft_node_ptr(resolved_parent));
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
			/*
			 * PLAN -> COMMIT window (-DFT_DELAY_INJECT only, no-op
			 * otherwise).  The decision "this ancestor keeps a child,
			 * so it stays wired" has just been made from a count read
			 * a few lines up; everything that empties it happens after
			 * this point.  Widening the gap here is what lets a peer's
			 * removal of the OTHER child land inside the window, which
			 * is the interleaving the residual needs and which is
			 * otherwise ~1 run in 100.
			 */
#ifndef FT_DELAY_SITE_B_ONLY
			ft_delay_writer();
#endif
			break;
		}
		/*
		 * Single-child node made childless by the prune that carries a
		 * shorter key: promote its external chain (the deepest such node
		 * on the path is the one promoted; prev_external_nodes_found then
		 * stops the climb at the next, surviving level).
		 */
		/* Resolve a parked splice proxy before republish (see above). */
		if (metadata->external_nodes && !topmost_external_nodes)
			topmost_external_nodes = ft_dereference_external(
				metadata->external_nodes);
		if (topmost_external_nodes)
			prev_external_nodes_found = true;

		/*
		 * Walk up: the current node becomes the child,
		 * update detach pointers to prune at this level.
		 */
		{
			struct cds_ft_inode_flag *parent_nf = resolved_parent;

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
				/*
				 * Re-anchor the plan's expected-old onto the level
				 * this iteration just decided to prune: @cur held
				 * only the branch below, so the slot that holds @cur
				 * is the one the drop targets, and the value that
				 * verdict rests on is the one @cur was resolved from
				 * (see @plan_old_child).  For the entry holder that
				 * value is already in hand -- reusing it is what keeps
				 * the pair coherent; deeper levels reach @cur through
				 * the back-pointer chain and have no earlier read.
				 */
				plan_old_child = detach_node_flag_ptr ==
						entry_holder_slot ?
					entry_holder_raw :
					(struct cds_ft_inode_flag *)
						rcu_dereference(*detach_node_flag_ptr);
			}
			/*
			 * One hop up moves the byte-depth by the PARENT's span,
			 * never by one: a compressed parent consumes its whole
			 * run.  Dating the climb by one per ancestor puts every
			 * node it reports at a depth the descent disagrees with,
			 * and a lock-set member is then anchored at a level that
			 * is not its own.
			 */
			cur_span = ft_node_span(ft, parent_nf);
			if (caa_unlikely(cur_span > cur_depth))
				return -EAGAIN;	/* stale plan: re-descend */
			cur_depth -= cur_span;
			cur = parent_nf;
			climbed = true;
		}
	}

	iter_node_flag = *detach_parent_flag_ptr;
	elevated_old_child = *detach_node_flag_ptr;
	/*
	 * F1 / RESOLVED-POINTER CONTRACT (ft-helpers.h): a peer mid-commit
	 * recompacting or re-homing the boundary (or the detach target) parks a
	 * flip-proxy on its slot.  Left raw, that proxy is silently misclassified
	 * as a type-7 internal node below (ft_node_compressed / the phase-2 free
	 * walk / the shape-D ft_node_ptr) and its tag stripped into a wild
	 * pointer, faulting frames later in cds_ft_item_to_metadata.  Do NOT
	 * classify or embed a parked proxy: bail so the wrapper re-descends and
	 * re-derives the slot once the peer's commit settles.  Resolving in place
	 * would be wrong -- a recompaction relocates the boundary and retires its
	 * old parent body, so detach_parent_flag_ptr can point into freed memory.
	 */
	if (caa_unlikely(ft_node_flip_proxy(iter_node_flag) ||
			ft_node_flip_proxy(elevated_old_child)))
		return -EAGAIN;
	/*
	 * PLAN EXPECTED-OLD, enforced (see @plan_old_child): everything below --
	 * the orphan set walked from @elevated_old_child, the count fold, and the
	 * drop itself -- names the subtree the climb condemned.  A peer that
	 * republished this slot since has put a DIFFERENT subtree here, one no
	 * level of the climb ever counted, so the whole plan is void.  Nothing is
	 * built, locked or reserved yet: re-descend against the settled tree.
	 */
	if (caa_unlikely(elevated_old_child != plan_old_child))
		return -EAGAIN;
	/* Plan-snapshot the holder slot before any recompaction overwrite. */
	holder_old_flag = iter_node_flag;

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
			unsigned int walk_depth = ft_child_depth_of(ft,
				iter_node_flag, cur_depth);
			bool phase2_first = true;

			while (walk_nf &&
			       !ft_node_external(walk_nf) &&
			       nr_to_free < FT_MAX_DEPTH) {
				struct cds_ft_inode_flag *next = NULL;
				unsigned int nr_child;
				struct cds_ft_node *ext_nodes;
				struct cds_ft_metadata *ometa;
				struct cds_ft_compressed_node *ocn = NULL;
				struct ft_held_anchor owalk = { 0 };
				uintptr_t osnap = 0;

				if (ft_node_compressed(walk_nf))
					ocn = ft_compressed_node_ptr(
						ft_skip_child_ptr(walk_nf));
				ometa = ocn
					? cds_ft_item_to_metadata(
						(struct cds_ft_inode *) ocn)
					: cds_ft_item_to_metadata(
						ft_node_ptr(walk_nf));
				/*
				 * §9.2 plan-lock: lock acquire the orphan BEFORE
				 * reading its nr_child for the collapse decision, so
				 * the "retire it" verdict is derived from a FROZEN
				 * word and the fenced tombstone's expected-old (this
				 * snap) is exactly that state.  A peer that grows the
				 * orphan AFTER the mark tears the fenced expected-old
				 * and aborts our commit (detect) -- and in practice
				 * cannot even reach that: ft_meta_nr_child_inc spins on
				 * FT_STATE_INPLACE_WAIT_MASK, which includes LOCK
				 * unconditionally, so it WAITS for the mark.  One that
				 * grew it BEFORE is reflected
				 * in @osnap and stops the walk (nr_child > 1).  A dirty
				 * mark (peer proxy / concurrent copier / real retire)
				 * bails to the caller's re-descend.
				 */
				if (ft->lock_fine) {
					lctx.held.nr_extra =
						(unsigned int) nr_orphan_locked;
					if (ft_detach_orphan_acquire(ft, &lctx,
								walk_nf, walk_depth,
								ometa, &owalk)) {
						ret = -EAGAIN;
						goto end;
					}
					osnap = owalk.node_snap;
					nr_child = ft_state_nr_child(osnap);
				} else
					nr_child = ft_meta_nr_child(ometa);
				ext_nodes = ometa->external_nodes;
				if (ocn) {
					next = ocn->child;
				} else if (nr_child == 1) {
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

				if (!phase2_first &&
				    (nr_child > 1 || ext_nodes)) {
					if (ft->lock_fine && !owalk.shared)
						ft_meta_lock_release(owalk.lock);
					break;
				}
				phase2_first = false;
				to_free[nr_to_free++] = walk_nf;
				if (ft->lock_fine)
					orphan_held[nr_orphan_locked++] = owalk;
				walk_nf = next;
			}
			/*
			 * As in the replace-ptr free-walk below: a skip-compressed
			 * external leaf at the chain end keeps its path bytes in a
			 * separate, now-orphaned skip-target compressed node that the
			 * walk stops short of.  Free it (the external leaf stays
			 * caller-owned).
			 */
			if (walk_nf && ft_node_skip_compressed(walk_nf)) {
				trailing_skip_cn =
					ft_skip_to_compressed(ft, walk_nf);
				if (ft->lock_fine) {
					struct cds_ft_metadata *tm =
						cds_ft_item_to_metadata(
							(struct cds_ft_inode *)
							trailing_skip_cn);

			/*
			 * The one-hop ft_skip_to_compressed is NOT MW-safe: a peer
			 * split/merge can tear the skip back-pointer so it recovers
			 * a node that is not this skip's target at all (type
			 * confusion -- ft_reanchor_flag's header states the same
			 * hazard for the descent side).  Every OTHER orphan is
			 * guarded before it is retired (the walk breaks on
			 * nr_child > 1, ft_detach_orphan_planlock refuses a grown
			 * one); the trailing skip-target was the one that was not,
			 * and it is retired unconditionally below.
			 *
			 * A compressed node holds exactly ONE child, so a resolved
			 * target whose count says otherwise is not the node this
			 * plan is about.  Measured: retiring it dropped a
			 * concurrently published 2-child subtree, i.e. SILENT KEY
			 * LOSS (an insert reported OK for a key no root descent
			 * could then find).  Bail to the op's re-descend, the same
			 * answer a dirty mark gets one line below.
			 */
					if (ft_meta_nr_child_load(tm) != 1) {
						ret = -EAGAIN;
						goto end;
					}
					lctx.held.nr_extra =
						(unsigned int) nr_orphan_locked;
					if (ft_detach_orphan_acquire(ft, &lctx,
							ft_compressed_node_flag(
								trailing_skip_cn),
								walk_depth, tm,
							&orphan_held[nr_orphan_locked])) {
						ret = -EAGAIN;
						goto end;
					}
					orphan_trailing_held =
						&orphan_held[nr_orphan_locked++];
					lctx.held.nr_extra =
						(unsigned int) nr_orphan_locked;
				}
			}
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
				ft_flip_txn_create_bounded(ft,
					FT_REMOVE_COMMIT_REC_MAX_EDGES
					+ 1 /* §4.B parent guard (Sites 3+4 excl.) */
					+ ft_freeze_reserve(ft, (unsigned int) nr_to_free
						+ (trailing_skip_cn ? 1 : 0))
					+ (topmost_external_nodes ? 1 : 0) /* folded external back-edge */
					+ (ft->rank_stats ? detach_depth + 1 : 0) /* nr_keys fold walk */
					+ (freeze_leaf ? FT_HLIST_FREEZE_MAX_EDGES : 0));

			if (!orphan_txn) {
				ret = -ENOMEM;
				goto end;
			}
			for (fi = 0; fi < nr_to_free; fi++) {
				struct cds_ft_metadata *m = cds_ft_item_to_metadata(
					ft_node_compressed(to_free[fi])
						? (struct cds_ft_inode *)
						  ft_compressed_node_ptr(
							ft_skip_child_ptr(to_free[fi]))
						: ft_node_ptr(to_free[fi]));
				if (ft->lock_fine)
					ft_detach_freeze_one(orphan_txn, &lctx,
						&orphan_held[fi], m);
				else
					ft_flip_txn_record_tombstone(orphan_txn, m);
			}
			if (trailing_skip_cn) {
				struct cds_ft_metadata *m = cds_ft_item_to_metadata(
					(struct cds_ft_inode *) trailing_skip_cn);
				if (ft->lock_fine)
					ft_detach_freeze_one(orphan_txn, &lctx,
						orphan_trailing_held, m);
				else
					ft_flip_txn_record_tombstone(orphan_txn, m);
			}
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
			/*
			 * @cur_depth dates @iter_node_flag: the climb tracks the
			 * holder's byte-depth alongside the node itself, and
			 * @iter_node_flag is that same holder re-read from its
			 * slot.
			 */
			lctx.held.txn = orphan_txn;
			lctx.held.nr_extra = (unsigned int) nr_orphan_locked;
			ret = ft_detach_node_replace_compressed_parent(ft,
				iter_node_flag, cur_depth, &lctx,
				detach_parent_flag_ptr,
				topmost_external_nodes, elevated_old_child,
				&nr_clear, fuse_cell,
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
			unsigned int walk_depth = ft_child_depth_of(ft,
				iter_node_flag, cur_depth);
			/*
			 * The walk leaves the key path -- it descends the
			 * elevated chain, then the target's own chain -- so it
			 * extends a descent OF ITS OWN.  Extending the op's
			 * would re-answer the anchor query for every lock set
			 * that resolves after it (the chain-compress fuse and
			 * both publish guards below): those members sit on the
			 * KEY path, and a level whose boundary node is still
			 * pending resolves to the descent's CURSOR, which the
			 * walk would have moved onto its own branch.
			 */
			struct ft_descent wwd;
			struct ft_lock_ctx wlctx;

			if (wd_valid)
				wwd = wd;
			ft_lock_ctx_init(&wlctx, wd_valid ? &wwd : NULL, NULL,
				op_ctx ? op_ctx->op : NULL);
			wlctx.held.extra = orphan_held;
			/*
			 * CHAIN to the caller's held set, for the same reason the
			 * detach's own @lctx does: ft_lock_ctx_init NULLs .outer, so a
			 * walk that does not restore it makes every orphan acquire BLIND
			 * to the holds this op arrived with -- and reads its OWN mark as
			 * a peer's.  ft_held_set_snap recurses through .outer and tests
			 * each level's .glue, so the one link reaches the caller's txn
			 * registry, its extra array and its glue alike.  The rekey fold
			 * is the ctx that carries a glue, and it is exactly the caller
			 * whose detach ELEVATES into this walk.
			 */
			wlctx.held.outer = op_ctx ? &op_ctx->held : NULL;

			/* Phase 1: elevated ancestors. */
			while (nr_to_free < nr_clear &&
			       walk_nf &&
			       !ft_node_external(walk_nf) &&
			       nr_to_free < FT_MAX_DEPTH) {
				struct cds_ft_inode_flag *next = NULL;
				struct cds_ft_metadata *ometa;
				bool require_sc;

				/*
				 * F1 / RESOLVED-POINTER CONTRACT, at every link.
				 * The walk steps through raw child slots
				 * (@cn->child below, and phase 2's @ocn->child):
				 * a peer mid-commit on the chain parks a type-7
				 * flip proxy there, and a proxy's low nibble reads
				 * as an INTERNAL node.  Left unchecked it is
				 * classified as one, its tag stripped into the
				 * latch address, and ft_node_get_nth dispatches on
				 * ft_types[7] -- the garbage jump named in
				 * ft_node_get_nth's own header.
				 *
				 * Bail rather than resolve, exactly as the three
				 * plan-lock bails in these loops do: a parked proxy
				 * means a peer commit is in flight on the very
				 * chain this free set is being derived from, so the
				 * set is stale.  Nothing is published yet -- the
				 * to_free[] walk runs BEFORE the commit and its
				 * nodes are freed only in the !ret block -- so the
				 * op re-descends and re-derives it.
				 */
				if (caa_unlikely(ft_node_flip_proxy(walk_nf))) {
					ret = -EAGAIN;
					goto end;
				}

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
					/* Skip-target: structurally single-child. */
					wlctx.held.txn = lctx.held.txn;
					wlctx.held.nr_extra =
						(unsigned int) nr_orphan_locked;
					if (ft->lock_fine && ft_detach_orphan_planlock(
							ft, &wlctx,
							ft_compressed_node_flag(cn),
							walk_depth,
							cds_ft_item_to_metadata(
								(struct cds_ft_inode *) cn),
							false, orphan_held,
							&nr_orphan_locked)) {
						ret = -EAGAIN;
						goto end;
					}
					to_free[nr_to_free++] =
						ft_compressed_node_flag(cn);
					walk_depth = ft_walk_extend(&wwd, wd_valid,
						ft_compressed_node_flag(cn),
						walk_depth, cn->len);
					walk_nf = ft_skip_child_ptr(walk_nf);
					continue;
				}
				if (ft_node_compressed(walk_nf)) {
					struct cds_ft_compressed_node *cn;

					cn = ft_compressed_node_ptr(walk_nf);
					next = cn->child;
					ometa = cds_ft_item_to_metadata(
						(struct cds_ft_inode *) cn);
					require_sc = false;	/* compressed: single-child */
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
					ometa = cds_ft_item_to_metadata(
						ft_node_ptr(walk_nf));
					require_sc = true;	/* elevated internal: nr_child==1 */
				}
				wlctx.held.txn = lctx.held.txn;
				wlctx.held.nr_extra = (unsigned int) nr_orphan_locked;
				if (ft->lock_fine && ft_detach_orphan_planlock(ft,
						&wlctx, walk_nf, walk_depth, ometa,
						require_sc, orphan_held,
						&nr_orphan_locked)) {
					ret = -EAGAIN;
					goto end;
				}
				to_free[nr_to_free++] = walk_nf;
				walk_depth = ft_walk_extend(&wwd, wd_valid, walk_nf,
					walk_depth,
					ft_node_compressed(walk_nf) ?
						ft_compressed_node_ptr(walk_nf)->len :
						1);
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
					struct cds_ft_metadata *ometa;
					struct cds_ft_compressed_node *ocn = NULL;
					struct ft_held_anchor owalk = { 0 };
					uintptr_t osnap = 0;

					/* Same contract as phase 1 above. */
					if (caa_unlikely(ft_node_flip_proxy(walk_nf))) {
						ret = -EAGAIN;
						goto end;
					}

					if (ft_node_compressed(walk_nf))
						ocn = ft_compressed_node_ptr(walk_nf);
					ometa = ocn
						? cds_ft_item_to_metadata(
							(struct cds_ft_inode *) ocn)
						: cds_ft_item_to_metadata(
							ft_node_ptr(walk_nf));
					/*
					 * §9.2 plan-lock (mirrors Block A): MARK before
					 * reading nr_child for the collapse decision so the
					 * fenced tombstone's expected-old is the exact word
					 * the "retire it" verdict rests on.
					 */
					if (ft->lock_fine) {
						wlctx.held.txn = lctx.held.txn;
						wlctx.held.nr_extra = (unsigned int)
							nr_orphan_locked;
						if (ft_detach_orphan_acquire(ft,
								&wlctx, walk_nf,
								walk_depth, ometa,
								&owalk)) {
							ret = -EAGAIN;
							goto end;
						}
						osnap = owalk.node_snap;
						nr_child = ft_state_nr_child(osnap);
					} else
						nr_child = ft_meta_nr_child(ometa);
					ext_nodes = ometa->external_nodes;
					if (ocn) {
						next = ocn->child;
					} else if (nr_child == 1) {
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

					if (!phase2_first &&
					    (nr_child > 1 || ext_nodes)) {
						if (ft->lock_fine && !owalk.shared)
							ft_meta_lock_release(
								owalk.lock);
						break;
					}
					phase2_first = false;
					to_free[nr_to_free++] = walk_nf;
					if (ft->lock_fine)
						orphan_held[nr_orphan_locked++] =
							owalk;
					walk_depth = ft_walk_extend(&wwd, wd_valid,
						walk_nf, walk_depth,
						ocn ? ocn->len : 1);
					walk_nf = next;
				}
				/*
				 * The chain stops at @walk_nf.  A skip-compressed
				 * external leaf keeps its path bytes in a separate
				 * skip-target compressed node the walk stops short of:
				 * the external leaf is caller-owned, but that
				 * skip-target is trie-owned and now orphaned.  Queue it.
				 */
				if (walk_nf && ft_node_skip_compressed(walk_nf)) {
					trailing_skip_cn_flag = walk_nf;
					if (ft->lock_fine) {
						struct cds_ft_metadata *tm =
							cds_ft_item_to_metadata(
								(struct cds_ft_inode *)
								ft_skip_to_compressed(ft,
									walk_nf));

						/* See the trailing-target guard above:
						 * the one-hop skip resolver is not
						 * MW-safe, and this target is retired
						 * unguarded otherwise. */
						if (ft_meta_nr_child_load(tm) != 1) {
							ret = -EAGAIN;
							goto end;
						}
						wlctx.held.txn = lctx.held.txn;
						wlctx.held.nr_extra = (unsigned int)
							nr_orphan_locked;
						if (ft_detach_orphan_acquire(ft,
								&wlctx, walk_nf,
								walk_depth, tm,
								&orphan_held[nr_orphan_locked])) {
							ret = -EAGAIN;
							goto end;
						}
						orphan_trailing_held =
							&orphan_held[nr_orphan_locked++];
						wlctx.held.nr_extra = (unsigned int)
							nr_orphan_locked;
					}
				}
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

			/*
			 * Proxy-resolved, for the reason spelled out at the
			 * up-walk's own snapshot: @bmeta IS that boundary node,
			 * so this is a SECOND raw read of the very word the climb
			 * just had to resolve.  Measured on the shared-destination
			 * merge oracle's plan: 711 of 749873 reads here land on a
			 * peer's parked state word.  The failure is worse than the
			 * climb's, because it is silent -- a garbage count that
			 * happens to decode as 2 enters the shape-D fusion, whose
			 * scan below takes the FIRST surviving child and builds a
			 * merged compressed node to REPLACE the boundary, dropping
			 * every other child the node really had.  No assert stands
			 * between that and a published trie.
			 */
			if (ft_group_skip_compressed(ft->group) &&
			    !topmost_external_nodes &&
			    ft_meta_nr_child_load(bmeta) == 2 &&
			    !bmeta->external_nodes &&
			    ft_parent_node(bmeta->parent_word) != NULL) {
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
						lctx.held.txn = NULL;
					lctx.held.nr_extra =
						(unsigned int) nr_orphan_locked;
					int cret = ft_chain_compress_fused(ft,
						iter_node_flag, cur_depth, &lctx,
						bmeta,
						s_child, s_byte,
						2 /* shape-D: survivor + the child this commit detaches */,
						fuse_cell, run,
						to_free, nr_to_free,
						trailing_skip_cn_flag,
						ft->lock_fine ? orphan_held : NULL,
						orphan_trailing_held,
						freeze_leaf,
						count_delta,
						ft->rank_stats ? detach_depth + 1 : 0,
				/*
				 * FOLD the collapse into the caller's txn when this
				 * detach is itself being folded, so the whole move is
				 * ONE decide.  Its retired chain and its unpublished
				 * merged node come back through @recompact_out for the
				 * caller to reclaim on the right side of its commit.
				 */
				record_only ? shared_txn : NULL, record_only,
				record_only && recompact_out ?
					&recompact_out->collapse : NULL);

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
			 * (atomic detach, §4.B) rather than as an early standalone flip.
			 *
			 * FORCE-TXN: created UNCONDITIONALLY.  The former residual gate
			 * (list-off shrink under a non-compressed parent with no run,
			 * rank-stats off) left @commit_txn NULL, sending the DEL
			 * recompaction down ft_node_recompact's retire_txn-NULL arm --
			 * an IMMEDIATE reparent sweep + external-head prev store onto
			 * the not-yet-published copy, reader-followable through the
			 * still-reachable children until the republish (the early
			 * live-wire class).  With the txn always present, every DEL
			 * retire records its sweep, tombstone and forward republish
			 * into ONE flip.  A txn no commit consumes is freed unused at
			 * @end; the new -ENOMEM aborts before any side-effect.
			 */
			if (record_only) {
				/*
				 * FOLD (coherent rekey one-decide writer): the caller's
				 * SHARED mixed txn absorbs this SIMPLE-case detach's forward-
				 * slot clear + nr_child-- (both MW -- BP survives unlocked, a
				 * peer conflict aborts the fold clean) so they flip atomically
				 * with the dst-attach + S_top COW the caller records into the
				 * same txn; the caller runs the ONE commit and owns @commit_txn
				 * (never freed here).  The caller pre-reserved it for the whole
				 * fold, an ELEVATING detach's orphan tombstones included.
				 *
				 * An ELEVATING detach IS expressible here: its orphan chain rides
				 * @recompact_out with the RETIRED copy's lifetime (the caller's
				 * commit is what unlinks it, so it is freed when that commit
				 * lands), and every tombstone freezes into this same shared txn --
				 * so the freeze flips with the unlink exactly as it does when this
				 * detach commits for itself.  What remains out of scope is a
				 * caller-supplied retire set or leaf freeze: the rekey fold passes
				 * neither, and each would need a reclaim owner of its own.
				 */
				assert(!retire_glue && !freeze_leaf);
				commit_txn = shared_txn;
			} else {
				commit_txn = ft_flip_txn_create_bounded(ft,
					FT_REMOVE_COMMIT_REC_MAX_EDGES
					+ 1 /* §4.B parent guard (Site 1 arms excl.) */
					+ ft_freeze_reserve(ft, (unsigned int) nr_to_free
						+ (trailing_skip_cn_flag ? 1 : 0))
					+ (retire_glue ? retire_glue->cap_free : 0)
					+ (ft->rank_stats ? detach_depth + 1 : 0) /* nr_keys fold walk */
					+ (freeze_leaf ? FT_HLIST_FREEZE_MAX_EDGES : 0));
				if (!commit_txn) {
					ret = -ENOMEM;
					goto end;
				}
			}
			FT_B2P_CREATED(ft, commit_txn);
#ifdef FT_REMOVE_CLAIM
			/*
			 * §4 STEP B2's DRY RUN, the tool 658989ef was for the rekey
			 * writer: point B0's owner assert at the remove commit_rec so
			 * every record it plants without owning is named, at an abort,
			 * on a build otherwise byte-identical to the unarmed one.
			 *
			 * ☞ CLAIMED HERE, WHICH IS EARLIER THAN THE ARM WOULD BE.  A
			 * dry run has to claim BEFORE the records it wants checked, and
			 * for this site that is before its acquires finish -- so read a
			 * miss as "this record is planted before its owner is
			 * registered" FIRST.  That ORDERING class has been the answer
			 * more often than a missing acquire.
			 *
			 * ☠ And read a miss as "the registry cannot SEE this hold"
			 * before "the op does not HOLD it": where a caller's SWEEP owns
			 * the mark's clearing, the absence is by design (ft_flip_txn_owns).
			 */
			ft_flip_txn_claim_per_op_armable(ft, commit_txn);
#endif
#ifdef FEATURE_FT_PROBE_PROMOTE
			/*
			 * §4.B unguarded-promote probe.  At the CALL SITE, not inside
			 * ft_popcount_node_replace_ptr: that function returns from its
			 * `if (pub)` branch before any counter placed within it, so an
			 * inside counter reads 0 for BOTH variants and looks like the
			 * arm is dead when only the unguarded one is.
			 */
			if (topmost_external_nodes) {
				if (pub)
					FT_PROMOTE_PROBE_INC(cds_ft_probe_promote_deferred);
				else
					FT_PROMOTE_PROBE_INC(cds_ft_probe_promote_immediate);
			}
#endif
			lctx.held.txn = commit_txn;
			lctx.held.nr_extra = (unsigned int) nr_orphan_locked;
			/*
			 * RE-DERIVE the hint when the climb MOVED.  The caller's
			 * names the detach target's junction; after an elevation
			 * the recompacted node is an ANCESTOR of that, and its own
			 * parent is @boundary_parent_nf with @detach_parent_flag_ptr
			 * as its slot -- both re-derived by the same walk that
			 * moved, which is the only frame that can know them.  Left
			 * stale, the @parent_guard read-set term validates the
			 * recompacted node against a parent that is not its own, so
			 * the acquire's commit ABORTS on every attempt and the
			 * caller's -EAGAIN loop re-derives the identical plan
			 * forever.
			 *
			 * @parent_held is FALSE and @gp / @gp_slot are NULL: the
			 * caller's answers were about ITS junction and say nothing
			 * about this one.  A word this op does happen to hold is
			 * still handled -- the acquire DEDUPES against the held set
			 * rather than refusing -- and a COMPRESSED parent, whose
			 * SKIP_X dual would need a great-grandparent this frame has
			 * not derived, is refused above.
			 */
			if (climbed && src_held_hint &&
					boundary_parent_nf &&
					ft_node_compressed(boundary_parent_nf)) {
				/*
				 * A COMPRESSED landing parent carries a SKIP_X dual the
				 * recompact re-encodes into its OWN parent's slot, and
				 * this walk has not derived that great-grandparent pair.
				 * Deriving it from a back-pointer has exactly the
				 * staleness the hint exists to avoid.  A shape this frame
				 * cannot express, not a peer: -EDOM, so the caller
				 * reports it uncovered instead of retrying forever.
				 */
				ret = -EDOM;
				goto end;
			}
			if (climbed && src_held_hint) {
				elevated_hint = (struct ft_parent_hint){
					.parent = boundary_parent_nf,
					.slot = detach_parent_flag_ptr,
					.gp = NULL, .gp_slot = NULL,
					.parent_held = false,
					.parent_guard = src_held_hint->parent_guard };
				replace_hint = &elevated_hint;
			}
			ret = ft_node_replace_ptr(ft,
				detach_node_flag_ptr,
				elevated_old_child,
				&iter_node_flag,
				&old_recompacted_node,
				metadata_stack[nr_branch - 1],
				n, (struct cds_ft_inode_flag *) topmost_external_nodes,
				detach_parent_flag_ptr == &ft->root,
				cur_depth, pub, commit_txn, replace_hint,
				&lctx);
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
					&lctx,
					to_free, nr_to_free, trailing_skip_cn_flag,
					ft->lock_fine ? orphan_held : NULL,
					orphan_trailing_held);
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
#ifdef FEATURE_FT_PROBE_PROMOTE
				/*
				 * Did the §4.B acquire actually run for a PROMOTE?
				 * `pub != NULL` is only a proxy: the guard needs
				 * pub->armed too, so a promote that left @pub unarmed
				 * would store without the acquire just as the pub-less
				 * path would.  Count the consequence, not the proxy.
				 */
				if (topmost_external_nodes)
					FT_PROMOTE_PROBE_INC(cds_ft_probe_promote_guarded);
#endif
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
						ft_flip_txn_record_head_back_edge(commit_txn,
							(void **) pub->head_parent_field,
							pub->head_parent_old,
							pub->head_parent_new);
					else
						rcu_assign_pointer(*pub->head_parent_field,
							pub->head_parent_new);
				}
				/*
				 * §4.B VALIDATE (Phase 4.3, MW): the EXTERNAL-PROMOTE
				 * in-place commit stores pub->slot INSIDE the live holder
				 * @iter_node_flag but records NO state edge on it --
				 * pub->state_meta is NULL for a promote (set only for a
				 * pure leaf delete, which fuses a real nr_child-- that
				 * self-guards the holder).  The forward CAS below validates
				 * only the slot VALUE (the displaced old child), so a peer
				 * that recompacts/freezes the holder through its grandparent
				 * slot leaves that value intact in the retired copy: the CAS
				 * still matches and the promoted head is published into a
				 * reclaimed node (UAF).  Guard the holder's state word so
				 * such a peer's freeze-on-free ABORTs this commit --
				 * symmetric with the external-CLEAR guards (2547, 3071).
				 * Reuses the "+1 §4.B parent guard" reservation on
				 * @commit_txn (mutually exclusive with the recompaction
				 * republish guard, which fires only on the pub-UNARMED path).
				 */
				if (commit_txn && !pub->state_meta) {
					lctx.held.txn = commit_txn;
					lctx.held.nr_extra =
						(unsigned int) nr_orphan_locked;
					ft_flip_txn_lock_or_guard_parent(ft,
						commit_txn, &lctx,
						iter_node_flag, cur_depth);
				}
				/*
				 * PHASE B, STEP B2 -- THE ARM.  The guard/release
				 * above is this path's LAST ft_flip_txn_lock_register,
				 * and ft_remove_one_commit below plants the first
				 * record after it, so this is where
				 * ft_flip_txn_arm_per_op's "after the op's last
				 * register" contract puts it.  The helper applies the
				 * lock_fine, per-node-spacing and empty-registry
				 * refusals itself.
				 *
				 * ☞ WHAT IT CONVERTS IS THE TAIL, and that is a
				 * property of this op rather than a gap.  ft_detach_node
				 * interleaves registers and records -- the orphan
				 * freezes and the recompact/collapse edges are planted
				 * EARLIER on this same txn and stay MW, which is
				 * stricter and always sound.  Converting them needs
				 * their own arm points, each with its own last-register
				 * to sit after; that is a later step, not something
				 * this one leaves half-done.
				 *
				 * ☞ The dry run (-DFT_REMOVE_CLAIM) claims at txn
				 * CREATION -- strictly earlier than here -- and is clean
				 * on both suites, so every record on this txn already
				 * passes the owner check, not merely the ones this arm
				 * converts.
				 */
				FT_B2P_PATH(inplace, ft, commit_txn);
				if (commit_txn)
					ft_flip_txn_arm_per_op(ft, commit_txn);
				ret = ft_remove_one_commit(ft, pub->slot,
					pub->slot_owner,
					pub->old_val, pub->new_val,
					pub->state_meta,
					fuse_cell, run, commit_txn, NULL, record_only);
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
				/*
				 * FOLD: this detach commits NOTHING, so the chain is
				 * still live and linked when it returns -- the caller's
				 * commit is the unlink.  Hand it out with the retired
				 * copy's lifetime instead of arming the local deferred
				 * free below, which would reclaim a node a reader is
				 * entitled to be walking (and would free it even when
				 * the caller's commit ABORTS and nothing unlinked it).
				 */
				if (record_only) {
					if (recompact_out) {
						for (fi = 0; fi < nr_to_free; fi++)
							recompact_out->orphans[fi] =
								to_free[fi];
						recompact_out->nr_orphans = nr_to_free;
						recompact_out->orphan_trailing =
							trailing_skip_cn_flag;
					}
				} else {
					for (fi = 0; fi < nr_to_free; fi++)
						orphan_free[fi] = to_free[fi];
					nr_orphan_free = nr_to_free;
					orphan_trailing = trailing_skip_cn_flag;
					free_orphans_pending = true;
				}
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

			/*
			 * MW consistency: @detach_parent_flag_ptr was recovered by the
			 * up-prune walk, while @iter_meta->parent and its state-word slot
			 * offset were inherited by the recompact from a later snapshot.  A
			 * peer re-home of the grandparent between the two tears the pair --
			 * _ft_publish_to_parent_meta would then compute the fresh copy's
			 * offset (ft_set_parent_slot -> ft_slot_to_byte) off the wrong
			 * parent body and fault out of range against its bitmap.  Bail to a
			 * re-descend when iter_meta's OWN resolved slot no longer IS
			 * @detach_parent_flag_ptr: nothing is published yet (this fresh
			 * copy is build-invisible), so the end: unwind discards the copy and
			 * the unconsumed txn, exactly as the peer-won -EAGAIN below.
			 */
			if (ft_get_parent_slot(iter_meta, ft) !=
					detach_parent_flag_ptr) {
				ret = -EAGAIN;
				goto end;
			}
			/*
			 * VALIDATE (§4.B): guard the LIVE grandparent iter_meta->parent.
			 *
			 * LOCK_FINE (§9.3): a recompact ran (@old_recompacted_node), so
			 * this grandparent is its P -- already LOCKED, with a
			 * {LOCK|s -> s} release recorded on this very word.  The release
			 * record IS the guard (same word, same abort on a peer state
			 * change) and is strictly stronger, so the conversion REPLACES it.
			 * (Leaving the guard would be vacuous, not wrong: release-then-guard
			 * chains as a read-your-writes no-op -- see the ordering rule at
			 * ft_flip_txn_record_release_lock.)
			 */
			if (!(ft->lock_fine && old_recompacted_node))
				ft_flip_txn_guard_parent(ft, commit_txn,
					ft_parent_node(iter_meta->parent_word));
			_ft_publish_to_parent(ft, ft_parent_node(iter_meta->parent_word),
				detach_parent_flag_ptr, iter_node_flag,
				holder_old_flag, &rec);
			/*
			 * nr_keys fold (LEAF Increment 2): the recompaction's -1
			 * walk from the STABLE grandparent iter_meta->parent (the
			 * rebuilt copy itself was baked above) rides this same commit.
			 */
			if (count_delta) {
				ft_flip_txn_record_count_parent(ft, commit_txn,
					ft_parent_node(iter_meta->parent_word),
					count_delta);
				count_folded = true;
			}
			/*
			 * Recompaction (its eager child re-parent already ran in
			 * ft_node_replace_ptr) so the publish commits through the
			 * pre-reserved txn (reserved above; this arm requires
			 * fuse_cell/run) and cannot fail.
			 */
			FT_B2P_PATH(pubA, ft, commit_txn_used ? NULL : commit_txn);
			if (ft_remove_commit_rec(ft, &rec, fuse_cell, run,
					commit_txn_used ? NULL : commit_txn,
					record_only) > 0) {
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
			/*
			 * MW consistency, RECOMPACTION sub-case ONLY (old_recompacted_node
			 * set).  A non-fused recompaction publishes a build-invisible fresh
			 * copy whose (parent, offset) the recompact inherited as ONE plain
			 * snapshot, so ft_get_parent_slot(iter_meta) is a STABLE value no
			 * peer can write; a grandparent re-home that tears it vs
			 * @detach_parent_flag_ptr would fault ft_slot_to_byte in
			 * _ft_publish_to_parent_meta, so bail to a re-descend -- the fresh
			 * copy is freed and the recorded (unconsumed) txn destroyed at end:,
			 * nothing published yet (as the fused site above).
			 *
			 * Do NOT guard the external-promote sub-case (old_recompacted_node
			 * == NULL, reachable list-off with pub == NULL): there
			 * @iter_node_flag is the LIVE holder whose promoted head was already
			 * published reader-visibly above, so a -EAGAIN here would STRAND
			 * that mutation (the caller retries as "nothing published"), and its
			 * parent is peer-writable -- a check-then-use cannot make the raw
			 * re-read in _ft_publish_to_parent_meta atomic anyway.  That torn
			 * pair is a separate, pre-existing list-off gap.
			 */
			if (old_recompacted_node &&
			    ft_get_parent_slot(iter_meta, ft) !=
					detach_parent_flag_ptr) {
				ret = -EAGAIN;
				goto end;
			}
			/*
			 * VALIDATE (§4.B): lock (or guard-fallback) the LIVE
			 * grandparent iter_meta->parent -- value-swap target (§10.5).
			 *
			 * LOCK_FINE (§9.3): skip entirely ONLY when a recompact actually
			 * ran -- then this grandparent is its P, already locked, and its
			 * recorded {LOCK|s -> s} release IS the guard, strictly stronger
			 * (see the ordering rule at ft_flip_txn_record_release_lock).
			 * The external-promote sub-case below reaches here with
			 * @old_recompacted_node == NULL and NO recompact, hence no lock on
			 * this parent -- lock_or_guard acquires it (or guard-falls-back).
			 */
			if (ft->lock_fine && old_recompacted_node)
				; /* recompact's P already locked; release IS the guard */
			else
				ft_flip_txn_lock_or_guard_parent(ft, commit_txn,
					&lctx,
					ft_parent_node(iter_meta->parent_word),
					FT_DEPTH_FROM_DESCENT);
			_ft_publish_to_parent(ft, ft_parent_node(iter_meta->parent_word),
				detach_parent_flag_ptr, iter_node_flag,
				holder_old_flag, &rec);
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
					ft_parent_node(iter_meta->parent_word),
					count_delta);
				count_folded = true;
			}
			FT_B2P_PATH(pubB, ft, commit_txn_used ? NULL : commit_txn);
			if (ft_remove_commit_rec(ft, &rec, NULL, NULL,
					commit_txn_used ? NULL : commit_txn,
					record_only) > 0) {
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
		    ft_parent_node(iter_meta->parent_word) != NULL) {
			/*
			 * Post-commit: the structural commit consumed its own
			 * registry, so only the orphan marks this op still owns
			 * are held.
			 */
			lctx.held.txn = NULL;
			lctx.held.nr_extra = (unsigned int) nr_orphan_locked;
			ft_canonicalize_chain_compress(ft, iter_node_flag,
				cur_depth, &lctx, iter_meta);
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
	 * DLM orphan plan-lock cleanup (§9.2): release every orphan lock acquire the
	 * op still holds AND still owns.  What is left here marks the orphan's OWN
	 * word, so its terminal is the fenced {LOCK|s -> TOMBSTONE|s} tombstone: a
	 * successful detach committed it (LOCK dropped, and an acquire refuses a
	 * TOMBSTONE, so the bit cannot come back) and clear_if_held no-ops; on ANY
	 * abort / pre-commit bail (including a mark-miss mid-collection, which
	 * jumps here) the tombstone never applied, so the mark is still {LOCK|s}
	 * and is released here -- the structure returns byte-for-byte to its
	 * pre-op state.  One sweep covers both Block A and Block B (mutually
	 * exclusive), the trailing skip-target (an entry of @orphan_held like any
	 * other) and every commit outcome (in-place / recompaction / shape-D), no
	 * "consumed" tracking.
	 *
	 * A mark whose anchor SURVIVES the retire is NOT here: ft_detach_freeze_one
	 * handed it to the freezing txn (@txn_owned), which is the only owner that
	 * can tell a consumed mark from a peer's fresh one on that still-lockable
	 * word.
	 */
	{
		int oi;

		for (oi = 0; oi < nr_orphan_locked; oi++)
			if (!orphan_held[oi].shared &&
					!orphan_held[oi].txn_owned)
				ft_meta_lock_release_if_held(
					orphan_held[oi].lock);
	}
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
	 * fused with its own txn).  PREPARE state -> no grace period.  Under
	 * record_only @commit_txn IS the caller's SHARED txn -- never freed here
	 * (the caller owns it and, on any bail, destroys it after re-descending).
	 */
	if (commit_txn && !commit_txn_used && !record_only)
		ft_flip_txn_destroy(commit_txn);
	/*
	 * Reclaim safely after replacement.  Under record_only the src-junction
	 * recompaction's OLD copy stays LIVE (resolved through the parked grandparent
	 * proxy) until the CALLER's commit publishes the fresh copy, so its free is
	 * DEFERRED to the caller: hand it out through @old_recompacted_out on a
	 * recorded-OK (ret == 0) path; the caller frees it (call_rcu) after its commit
	 * succeeds.  A record_only bail (ret != 0) frees the unpublished fresh copy
	 * here, exactly as the self-committing abort arm.
	 */
	if (record_only && old_recompacted_node && !ret) {
		/*
		 * Hand BOTH copies to the caller: the OLD copy stays LIVE (resolved
		 * through the parked grandparent proxy) until the caller's commit
		 * retires it -> free on commit OK; the fresh NEW copy (@iter_node_flag,
		 * republished into the shared txn) is UNPUBLISHED until that commit ->
		 * free unpublished if the caller's commit ABORTS.
		 */
		if (recompact_out) {
			recompact_out->old_node = old_recompacted_node;
			recompact_out->new_flag = iter_node_flag;
		}
	} else if (old_recompacted_node) {
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
int ft_promote_head(struct cds_ft *ft, const struct ft_lock_ctx *ctx,
		struct cds_ft_inode_flag *parent_nf, unsigned int parent_depth,
		struct cds_ft_node **head_slot, struct cds_ft_node *node,
		struct cds_ft_node *next_node,
		struct cds_ft_metadata *held_holder, uintptr_t held_snap)
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

		if (!new_cell_flag) {
			if (held_holder)
				ft_meta_lock_release(held_holder);
			return -ENOMEM;
		}
		txn = ft_flip_txn_create_bounded(ft,
			FT_ORD_CELL_SWAP_PUBLISH_MAX_EDGES +
			FT_HLIST_FREEZE_MAX_EDGES + 2);	/* +1 §4.B parent guard, +1 next_node->prev fold */
		if (!txn) {
			ft_ord_cell_free_unpublished(ft,
				ft_ord_cell_ptr(new_cell_flag));
			if (held_holder)
				ft_meta_lock_release(held_holder);
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
		/*
		 * THE HOLDER'S FENCE IS TAKEN BEFORE THE BACK EDGE, NOT AFTER IT.
		 *
		 * @next_node->prev is a back-channel word of an EXTERNAL head, and
		 * an external carries no state word of its own -- so the word that
		 * excludes a peer here is the HOLDER's, the internal node whose
		 * @head_slot chains this list.  Recording the edge first named no
		 * owner at all and left the whole promote lane reading as an
		 * exclusion gap; taking the fence first makes the holder a
		 * REGISTERED lock on this very txn, which is what
		 * ft_flip_txn_owns then answers with.
		 *
		 * Safe to move: ft_flip_txn_hold_or_lock_parent returns void -- the
		 * held arm records a RELEASE terminal and registers it, the unheld
		 * arm acquires-or-guards -- so no bail is introduced, and every bail
		 * that releases @held_holder explicitly is still ABOVE this point.
		 *
		 * VALIDATE (§4.B): it also guards the LIVE holder this head-promote
		 * publishes into.
		 */
		ft_flip_txn_hold_or_lock_parent(ft, txn, ctx, parent_nf,
			parent_depth, held_holder, held_snap);
		ft_flip_txn_record_reserved(txn,
			ft_flag_to_metadata(ft, parent_nf),
			(void **) &next_node->prev,
			next_node->prev, new_cell_flag);
		_ft_publish_to_parent_meta(ft, parent_nf,
			(struct cds_ft_inode_flag **) head_slot,
			(struct cds_ft_inode_flag *) next_node,
			(struct cds_ft_inode_flag *) node,
			NULL, new_cell_flag, &rec, /*slot_owner_nf=*/ parent_nf);
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
			ft_flip_txn_create_bounded(ft, FT_PUB_SEDGE_MAX_EDGES +
				FT_HLIST_FREEZE_MAX_EDGES + 2);	/* +1 §4.B parent guard, +1 prev fold */

		void *prev_save;
		void *inherit;

		if (!txn) {
			if (held_holder)
				ft_meta_lock_release(held_holder);
			return -ENOMEM;
		}
		prev_save = rcu_dereference(next_node->prev);
		inherit = rcu_dereference(node->prev);
		if (caa_unlikely(ft_node_flip_proxy(
					(struct cds_ft_inode_flag *) prev_save) ||
				ft_node_flip_proxy(
					(struct cds_ft_inode_flag *) inherit))) {
			ft_flip_txn_destroy(txn);	/* PREPARE state: nothing recorded */
			if (held_holder)
				ft_meta_lock_release(held_holder);
			return -EAGAIN;
		}
		/* The holder's fence FIRST -- see the cell arm above for why. */
		ft_flip_txn_hold_or_lock_parent(ft, txn, ctx, parent_nf,
			parent_depth, held_holder, held_snap);
		ft_flip_txn_record_reserved(txn,
			ft_flag_to_metadata(ft, parent_nf),
			(void **) &next_node->prev, prev_save, inherit);
		_ft_publish_to_parent_meta(ft, parent_nf,
			(struct cds_ft_inode_flag **) head_slot,
			(struct cds_ft_inode_flag *) next_node,
			(struct cds_ft_inode_flag *) node,
			NULL, inherit /* folded prev: intended parent value */, &rec,
			/*slot_owner_nf=*/ parent_nf);
		n_s = ft_pub_rec_sedges(&rec, sedges);
		/* Fuse @node's freeze into the structural publish (doc §4.B). */
		ft_hlist_freeze_prepare(ft_flip_txn_handle(txn), node);
		int cret = ft_flip_status_to_errno(
			ft_ord_cell_flip_into(ft, txn, sedges, n_s));

		if (cret) {
			/*
			 * NOTHING installed -- the folded prev edge was
			 * discarded with the failed commit, @next_node's prev
			 * still names its predecessor @node.  Nothing to undo:
			 * -EAGAIN retries from a fresh derivation, -ENOMEM
			 * (an acquire that could not allocate) does not.
			 */
			return cret;
		}
	}
	return 0;
}

static
int ft_unchain_node(struct cds_ft *ft, const struct ft_lock_ctx *ctx,
		struct cds_ft_inode_flag *parent_nf, unsigned int parent_depth,
		struct cds_ft_node **head_slot, struct cds_ft_node *node)
{
	struct cds_ft_metadata *hmeta = NULL;	/* MW LOCK_FINE holder lock */
	uintptr_t hsnap = 0;
	struct cds_ft_node *next_node;

	/*
	 * MW LOCK_FINE (Step A, holder lock): serialise concurrent same-key chain
	 * mutation on the head's IMMEDIATE PARENT -- the trie holder.  Acquire its
	 * node lock BEFORE the chain read below (a peer mid-flip parks a proxy
	 * on node->next / relinks a neighbour's prev that this read would deref)
	 * and hold it across the mutate+commit.  @parent_nf is the holder for a
	 * head op; an interior detach passes NULL, so derive the head's holder by
	 * walking prev (ft_chain_head_holder).  Released after the op: promote /
	 * interior post-commit-clear -- their commits leave holder->state untouched
	 * (promote is a slot swap + a LOCK-masking guard; interior relinks
	 * neighbours), so a held fence bit composes.  Head-no-successor records the
	 * {LOCK|s -> s} RELEASE + registers it, so it FUSES with the same-word
	 * nr_child-- (a masking guard would disagree on expected-old and poison)
	 * and the txn owns the clear (commit consumes it; abort/destroy auto-clears
	 * the registered fence).  Under the FT-wide writer_lock no peer contends
	 * -> the acquire never misses in soak; FEATURE_FT_FAULT_INJECT drives the
	 * -EAGAIN bail (shared cds_ft_fault_lock_countdown).
	 */
	if (ft->lock_fine) {
		struct cds_ft_inode_flag *lock_nf = parent_nf ? parent_nf :
			ft_chain_head_holder(ft, node);

		/*
		 * @node is a PUBLISHED chain member being unchained, so it HAS a
		 * holder -- ASSERT rather than fall through unlocked.  A NULL means
		 * a never-inserted node (prev NULL), produced only by ft-insert's
		 * unwind paths on UNPUBLISHED nodes, which never reach an unchain.
		 * The old tolerance mutated the chain with NO exclusion, which the
		 * MW store's expected-value CAS still arbitrated; once these become
		 * sw it is a LOST UPDATE, so the assumption must fail loudly now.
		 * Measured unreachable: 0 NULL in 491532 ft_chain_head_holder calls
		 * across ft_unit and ft_inv's three list modes.
		 */
		assert(lock_nf);
		{
			struct ft_held_anchor h;
			unsigned int lock_depth = parent_depth;

			/*
			 * A DERIVED holder (interior detach, @parent_nf NULL)
			 * carries no depth of its own -- the head's holder IS the
			 * descent's parent (§5.2), so let the window date it.
			 */
			if (!parent_nf)
				lock_depth = FT_DEPTH_FROM_DESCENT;
			if (lock_depth == FT_DEPTH_FROM_DESCENT &&
					!ft_lock_ctx_depth_of(ft, ctx, lock_nf,
						&lock_depth))
				return -EAGAIN;
			if (ft_acquire_member(ft, ctx, lock_nf,
					ft_flag_to_metadata(ft, lock_nf),
					lock_depth, &h))
				return -EAGAIN;
			/*
			 * A holder the op ALREADY held is protected without a
			 * second mark, and its release belongs to the acquire
			 * that took it: leave @hmeta NULL so the publish below
			 * routes to the ordinary guard rather than recording a
			 * second terminal on the one word.
			 */
			if (!h.shared) {
				hmeta = h.lock;
				hsnap = h.lock_snap;
			}
		}
	}

	next_node = ft_node_next(node);

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
			ft_flip_txn_create_bounded(ft, FT_HLIST_DEL_MAX_EDGES);
		enum urcu_txn_status st;

		if (!txn) {
			if (hmeta)
				ft_meta_lock_release(hmeta);
			return -ENOMEM;
		}
		if (ft_hlist_del_prepare(ft_flip_txn_handle(txn), node)) {
			/*
			 * Peer conflict observed at prepare time (@node or a
			 * neighbour mid-deletion): nothing was installed
			 * (records never install without a commit) -- drop the
			 * txn and retry from a fresh position derivation.
			 */
			ft_flip_txn_destroy(txn);
			if (hmeta)
				ft_meta_lock_release(hmeta);
			return -EAGAIN;
		}
		st = ft_flip_txn_commit(ft, txn);
		/*
		 * The interior relink never touches holder->state -- the held
		 * fence is independent of this commit, so drop it directly on
		 * every outcome.
		 */
		if (hmeta)
			ft_meta_lock_release(hmeta);
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
		/*
		 * Hand the held fence to ft_promote_head: it records the RELEASE
		 * (composing with its holder guard, which a still-held LOCK
		 * would otherwise self-abort) and OWNS the fence outcome -- commit
		 * consumes it, an abort/destroy auto-clears the registered fence,
		 * and every early-fail path clears it directly.  No post-commit
		 * clear here.
		 */
		return ft_promote_head(ft, ctx, parent_nf, parent_depth,
			head_slot, node,
				next_node, hmeta, hsnap);
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

		txn = ft_flip_txn_create_bounded(ft, FT_PUB_SEDGE_MAX_EDGES +
			FT_HLIST_FREEZE_MAX_EDGES + 1);
		if (!txn) {
			/* Early fence held but not yet handed to the txn. */
			if (hmeta)
				ft_meta_lock_release(hmeta);
			return -ENOMEM;
		}
		/*
		 * VALIDATE (§4.B): guard the LIVE holder this head-clear publishes
		 * into.  Holding its lock (hmeta): record the {LOCK|s -> s}
		 * RELEASE + register so it FUSES with the fused nr_child-- on the
		 * same word (a masking guard would disagree on expected-old and
		 * poison the descriptor) and the txn owns the fence outcome --
		 * commit consumes it, an aborted/destroyed commit auto-clears the
		 * registered fence (so the flip_into abort below needs no manual
		 * clear).
		 */
		ft_flip_txn_hold_or_lock_parent(ft, txn, ctx, parent_nf,
			parent_depth, hmeta, hsnap);
		_ft_publish_to_parent(ft, parent_nf,
			(struct cds_ft_inode_flag **) head_slot, NULL,
			(struct cds_ft_inode_flag *) node, &rec);
		n_s = ft_pub_rec_sedges(&rec, sedges);
		ft_hlist_freeze_prepare(ft_flip_txn_handle(txn), node);
		int cret = ft_flip_status_to_errno(
			ft_ord_cell_flip_into(ft, txn, sedges, n_s));

		if (cret)
			/* Nothing installed, @node still chained. */
			return cret;
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
		bool *need_retry,
		struct urcu_txn *op)
{
	/*
	 * Anchor source for the op's lock-sets, populated only where a descent
	 * ran (@have_descent).  Under per-node granularity none does: every
	 * member anchors on itself, so no depth is needed.
	 */
	struct ft_descent d;
	bool have_descent = false;
	unsigned int holder_depth = 0;
	struct cds_ft_inode_flag *holder_flag;
	struct cds_ft_metadata *holder_meta;
	struct cds_ft_inode_flag **head_slot = NULL;
	const uint8_t *iter_key;
	size_t key_len = ft_key_len(ft, ft_iter_resolve_key_len(iter));
	/*
	 * POISONED, not zeroed.  Every path assigns @ret today (gcc's
	 * -Wmaybe-uninitialized agrees), but the terminal switch below ends in
	 * `default: abort()`, so a path added later that forgets to assign
	 * decides the op's whole outcome from stack garbage: garbage that lands
	 * on 0 reports CDS_FT_STATUS_OK for a removal that may not have
	 * happened -- the defect @2d3b92ea fixed in remove_all -- and any other
	 * value aborts or not depending on the frame.
	 *
	 * -EINVAL is deliberately NOT a case in that switch, so an unassigned
	 * @ret reaches `default:` DETERMINISTICALLY instead of by luck.  The
	 * unknown is outside the domain and the consumer already refuses it.
	 */
	int ret = -EINVAL;

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
	 * STALE BACK-EDGE (measured: 78% of the cases where this fires).  The
	 * derivation above trusts @node->prev, and a back-pointer is updated
	 * LAZILY: a peer that replaced the holder can leave @node->prev naming
	 * the RETIRED old one while the FORWARD path from the root still
	 * resolves @node correctly.  The op is then derived against a dead
	 * node -- ft_meta_lock_acquire refuses a TOMBSTONE word, the caller
	 * gets -EAGAIN, and the wrapper's retry re-derives the SAME dead
	 * holder forever, holding the per-trie FIFO fair-mutex turn and
	 * denying every other writer (measured: 2,000,000+ consecutive
	 * attempts, 11 of 12 writer threads parked).  Escalation cannot break
	 * it: a tombstone is permanent, not contention.
	 *
	 * The FORWARD path is authoritative, so re-derive the holder by a
	 * key-guided descent -- the same walk ft_detach_at uses -- and carry
	 * on with the live parent.  A descent that no longer reaches @node
	 * (the minority shape: the key really is gone) is an idempotent miss,
	 * reported like the ft_node_is_removed() early-out above rather than
	 * spun on.
	 */
	if (caa_unlikely(ft_flag_tombstoned(ft, holder_flag))) {
		const uint8_t *ik = iter_key;

		ft_anchor_descend(ft, &d, iter_key, key_len, &ik);
		if (!d.nf || d.pnf == NULL ||
				ft_flag_tombstoned(ft, d.pnf)) {
			/* The key is not reachable either: idempotent miss. */
			FT_TP(remove_exit, (int) CDS_FT_STATUS_NOT_FOUND);
			return CDS_FT_STATUS_NOT_FOUND;
		}
		holder_flag = d.pnf;
		holder_depth = d.pdepth;
		have_descent = true;
	} else if (ft->lock_spacing != CDS_FT_LOCK_SPACING_PER_NODE) {
		/*
		 * ANCHORED LOCK-SETS need a byte-depth per member, and this path has
		 * none: it derives the holder from @node's back-pointer and never
		 * walks.  Only the leaf's depth is free (== @key_len, what
		 * ft_detach_node already takes as its detach_depth); the holder and
		 * everything above it have none, and a climb cannot recover them --
		 * absolute depth is unknown to it until the root, so it can neither
		 * stop early nor hand back the node it walked past
		 * (doc/design/ft-dlm-lock-coarseness.md §5.3).
		 *
		 * Descend for them.  The walk is the recovery arm's, and the cost is
		 * OPT-IN WITH THE COARSENESS: per-node granularity anchors every member
		 * on itself, needs no depth, and keeps this path handle-derived.
		 */
		const uint8_t *ik = iter_key;

		ft_anchor_descend(ft, &d, iter_key, key_len, &ik);
		/*
		 * Locate the holder ON the descent and take ITS byte-depth -- that
		 * depth, not the leaf's, is what selects the holder's anchor.  The
		 * descent stops in one of two places:
		 *
		 *  - UNDER the leaf: it broke on an external @d.nf, so the holder is
		 *    @d.pnf at @d.pdepth.
		 *  - ON the holder: the key ended at an internal node and the leaf
		 *    hangs off its external_nodes (a prefix key), so the holder is
		 *    @d.nf at @d.depth.
		 *
		 * Neither matches for an EXTERNAL holder: ft_node_holder resolves a
		 * non-head duplicate's prev to its PREDECESSOR rather than to the trie
		 * parent (the distinction the ft_node_external(holder_flag) arm below
		 * turns on), so that chain's trie holder comes from
		 * ft_chain_head_holder and is anchored with it, not from here.
		 */
		have_descent = true;
		/*
		 * A holder the descent does NOT pass is one @node->prev names
		 * STALELY: a peer republished the holder and the back-edge still
		 * carries the old copy, which the tombstone test above misses
		 * whenever the peer's retire has not landed yet.  Take the
		 * FORWARD path's holder, the same authority the arm above
		 * applies -- the two positions are the two bullets listed there.
		 *
		 * Keeping the back-pointer's node instead leaves it UNDATED, and
		 * a byte-depth of 0 does not READ as undated: it is THE ROOT, so
		 * ft_anchor_meta anchors that holder on ITSELF while every op
		 * that dates it anchors on an ancestor.  The two then exclude
		 * nothing, which is the §1 disagreement -- measured as a LOST
		 * UPDATE, a chain-head promote publishing in place into a holder
		 * a peer was COW-recompacting (inv_writer_progress_chainmerge).
		 */
		if (d.nf == holder_flag) {
			holder_depth = d.depth;
		} else if (d.pnf == holder_flag) {
			holder_depth = d.pdepth;
		} else if (!ft_node_external(holder_flag)) {
			bool prefix = d.nf && !ft_node_external(d.nf) &&
				d.depth == key_len;
			struct cds_ft_inode_flag *fwd = prefix ? d.nf : d.pnf;

			if (!fwd || ft_flag_tombstoned(ft, fwd)) {
				/*
				 * The forward holder is dead too: nothing is
				 * reserved or published yet, so re-derive the
				 * whole position against the settled tree --
				 * the retry the tail takes for a peer-won
				 * commit, reached before any of the work.
				 */
				*need_retry = true;
				return CDS_FT_STATUS_OK;
			}
			holder_flag = fwd;
			holder_depth = prefix ? d.depth : d.pdepth;
		}
	}

	/*
	 * The op's lock context.  @d is an anchor source wherever the walk above
	 * actually RAN -- which is every coarse spacing, since the arm that runs
	 * it is gated on exactly that -- and NULL under per-node, where it never
	 * runs and every member anchors on itself anyway.
	 *
	 * ☠ "The walk RAN" is not "the walk LOCATED THE HOLDER".  This flag used
	 * to mean the second, so a holder the two arms above do not match threw a
	 * perfectly good anchor table away -- and a member dated by the ONE-HOP
	 * rule (ft_lock_ctx_depth_of_parent needs no descent to answer a DEPTH)
	 * then reached ft_anchor_meta with no descent to answer its ANCHOR.  That
	 * is the assert, in ft_chain_compress_fused under
	 * CDS_FT_LOCK_SPACING=exponential.
	 *
	 * A descent that does not describe a member reports that PER MEMBER and
	 * the caller re-plans; withholding it turns "this member" into "every
	 * member".
	 */
	struct ft_lock_ctx lctx;

	ft_lock_ctx_init(&lctx, have_descent ? &d : NULL, NULL, op);

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
		unsplice_txn = ft_flip_txn_create_bounded(ft,
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
		ret = ft_unchain_node(ft, &lctx, NULL, 0, NULL, node);
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
			ret = ft_detach_node(ft, &lctx, head_slot,
				ft_get_parent_slot(holder_meta, ft),
				key_len, true, fuse_cell, pubp, NULL, NULL, node,
				-1 /* leaf key removed: detach owns the -1 */,
				NULL, false, NULL, NULL);
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
			ret = ft_unchain_node(ft, &lctx,
				ft_compressed_node_flag(cn), holder_depth,
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
			    ft_parent_node(holder_meta->parent_word) != NULL) {
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
					holder_flag, holder_depth, &lctx,
					holder_meta,
					s_child, s_byte,
					1 /* sole body child; the removed entry is external */,
					fuse_cell, NULL,
					NULL, 0, NULL, NULL, 0 /* no orphan chain */, node,
					-1, ft->rank_stats ? key_len + 1 : 0,
					NULL, false, NULL);

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
					struct ft_flip_txn *txn = ft_flip_txn_create_bounded(ft,
						FT_REMOVE_COMMIT_REC_MAX_EDGES + 1 +
						(ft->rank_stats ? key_len + 1 : 0));

					if (!txn) {
						if (unsplice_txn)
							ft_flip_txn_destroy(unsplice_txn);
						FT_TP(remove_exit, (int) CDS_FT_STATUS_MEMORY_ERROR);
						return CDS_FT_STATUS_MEMORY_ERROR;
					}
					/* VALIDATE (§4.B): lock (or guard-fallback) the
					 * LIVE holder whose external_nodes this single-node
					 * clear empties -- value-swap target (§10.5). */
					ft_flip_txn_lock_or_guard_parent(ft, txn, &lctx,
					holder_flag, holder_depth);
					ft_flip_txn_record_count_parent(ft, txn,
						holder_flag, -1);
					ret = ft_remove_one_commit(ft,
						(struct cds_ft_inode_flag **) &holder_meta->external_nodes,
						holder_meta,
						(struct cds_ft_inode_flag *) node, NULL,
						NULL, dead_cell, NULL, txn, node, false);
					if (ret == 0 && fuse_remove)
						pub.armed = true;
				} else {
					/* List off + rank stats off: unchanged; no count. */
					ret = ft_unchain_node(ft, &lctx, holder_flag,
						holder_depth,
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
				    ft_parent_node(holder_meta->parent_word) != NULL) {
					ft_canonicalize_chain_compress(ft, holder_flag,
						holder_depth, &lctx, holder_meta);
				}
#endif
			}
		} else {
			/* Duplicates remain: head promotion (fresh-cell swap). */
			ret = ft_unchain_node(ft, &lctx, holder_flag, holder_depth,
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
			ret = ft_detach_node(ft, &lctx, head_slot,
				ft_get_parent_slot(holder_meta, ft),
				key_len, true, fuse_cell, pubp, NULL, NULL, node,
				-1 /* leaf key removed: detach owns the -1 */,
				NULL, false, NULL, NULL);
			/* @node's freeze rode the detach commit (freeze_leaf). */
		} else {
			/* Removing the head, duplicates remain: key count unchanged. */
			ret = ft_unchain_node(ft, &lctx, holder_flag, holder_depth,
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
							ft_flip_txn_create_bounded(ft,
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
	 * -ENOENT (detach's replace reached an emptied FT_NULL slot, or the
	 * unchain found the node already gone) is NOT a bug under Phase 4.3
	 * concurrent writers: the position was derived by a NON-exclusive
	 * traversal, so a peer's recompaction can retype/empty the cached parent
	 * slot between the derivation and the replace.  That path publishes
	 * nothing and destroys every pre-reserved txn (see end:), so it is a
	 * clean abort -- routed to the retry loop below exactly like -EAGAIN
	 * (re-derive the position from node->prev against the current tree).
	 * (Under single-writer exclusion the found-implies-replaceable invariant
	 * still holds, so -ENOENT does not arise there.)
	 */

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
	case -ENOENT:
		/*
		 * A peer writer won a commit this attempt (ABORT), or moved /
		 * recompaction-retyped the position pre-commit (-ENOENT: the
		 * cached slot emptied to FT_NULL, or the node was already
		 * unchained): NOTHING was published and every fused edge
		 * (freeze, count, tombstone) was discarded with it.  Signal the
		 * wrapper's retry loop to re-derive and re-attempt.
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
	struct urcu_txn optxn;
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
		s = _cds_ft_remove_locked(ft, iter, node, &need_retry, &optxn);
		if (!need_retry)
			break;
		/* Age the conflict, forfeit the turn, close the attempt. */
		ft_txn_attempt_bail(&optxn, true);
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
		struct cds_ft_node **result_node,
		struct urcu_txn *op)
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
			txn = ft_flip_txn_create_bounded(ft,
				FT_REMOVE_COMMIT_REC_MAX_EDGES +
				(ft->rank_stats ? 1 : 0));
			if (!txn) {
				*result_node = NULL;
				return CDS_FT_STATUS_MEMORY_ERROR;
			}
			ft_flip_txn_record_count_parent(ft, txn, ft->root, -1);
			/*
			 * Same rule as the prefix clear below: the pre-reserved
			 * txn cannot fail to ALLOCATE, but the flip can still
			 * ABORT, and an aborted flip left the NIL key in the
			 * trie.  Returning OK there hands the caller a chain it
			 * may reclaim while the root still points at it -- and
			 * frees @dead while it is still spliced into the list.
			 */
			if (ft_remove_one_commit(ft,
					(struct cds_ft_inode_flag **) &metadata->external_nodes,
					metadata,
					(struct cds_ft_inode_flag *) external_nodes, NULL,
					NULL, dead, NULL, txn, NULL, false)) {
				*result_node = NULL;
				return CDS_FT_STATUS_MEMORY_ERROR;
			}
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
	 * ANCHORED LOCK-SETS need a byte-depth per member, and locating the chain
	 * head gives none: the holder comes from the cached node's back-pointer.
	 * Descend for the depths, exactly as _cds_ft_remove_locked does and under
	 * the same opt-in -- per-node granularity anchors every member on itself,
	 * needs no depth, and leaves this path handle-derived
	 * (doc/design/ft-dlm-lock-coarseness.md §5.3).
	 */
	struct ft_descent d;
	struct ft_lock_ctx lctx;
	unsigned int holder_depth = 0;
	bool have_descent = false;

	if (ft->lock_spacing != CDS_FT_LOCK_SPACING_PER_NODE) {
		const uint8_t *ik = iter_key;

		ft_anchor_descend(ft, &d, iter_key, key_len, &ik);
		/*
		 * The holder is where the walk stopped: ON it for a prefix key
		 * (the key ended at an internal node carrying external_nodes),
		 * one level UP where the walk broke on the external leaf.
		 */
		if (d.nf == holder_flag) {
			holder_depth = d.depth;
			have_descent = true;
		} else if (d.pnf == holder_flag) {
			holder_depth = d.pdepth;
			have_descent = true;
		}
	}
	ft_lock_ctx_init(&lctx, have_descent ? &d : NULL, NULL, op);

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
		unsplice_txn = ft_flip_txn_create_bounded(ft,
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
			    ft_parent_node(holder_meta->parent_word) != NULL) {
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
					holder_flag, holder_depth, &lctx,
					holder_meta,
					s_child, s_byte,
					1 /* sole body child; the removed entry is external */,
					ft->ordered_list ? dead_cell : NULL,
					NULL, NULL, 0, NULL, NULL, 0 /* no orphan chain */, NULL,
					-1, ft->rank_stats ? key_len + 1 : 0,
					NULL, false, NULL);

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
				struct ft_flip_txn *txn = ft_flip_txn_create_bounded(ft,
					FT_REMOVE_COMMIT_REC_MAX_EDGES + 1 +
					(ft->rank_stats ? key_len + 1 : 0));

				if (!txn) {
					if (unsplice_txn)
						ft_flip_txn_destroy(unsplice_txn);
					*result_node = NULL;
					return CDS_FT_STATUS_MEMORY_ERROR;
				}
				/* VALIDATE (§4.B): lock (or guard-fallback) the LIVE
				 * holder whose external_nodes this clear empties --
				 * value-swap target (§10.5). */
				ft_flip_txn_lock_or_guard_parent(ft, txn, &lctx,
					holder_flag, holder_depth);
				ft_flip_txn_record_count_parent(ft, txn,
					holder_flag, -1);
				/*
				 * The commit's status is the ANSWER, not a
				 * formality: "pre-reserved => infallible" covers
				 * the ALLOCATION, and the §4.B VALIDATE arm above
				 * can still abort -- which, as that comment says,
				 * LEAVES THE KEY IN PLACE.  Discarding it and
				 * setting ret = 0 reported CDS_FT_STATUS_OK for a
				 * removal that did not happen, and a caller that
				 * believes OK reclaims a node still linked in the
				 * trie.  (The `ret = 0; if (ret == 0)` this
				 * replaces was the scar of the dropped status.)
				 */
				ret = ft_remove_one_commit(ft,
					(struct cds_ft_inode_flag **) &holder_meta->external_nodes,
					holder_meta,
					(struct cds_ft_inode_flag *) chain_head, NULL,
					NULL, dead_cell, NULL, txn, NULL, false);
				if (ret == 0) {
					/*
					 * Only a COMMITTED flip carried the
					 * unsplice; an aborted one leaves the cell
					 * spliced, and the tail's abort arm
					 * releases the unused reservation.
					 */
					if (ft->ordered_list)
						pub.armed = true;
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
					    ft_parent_node(holder_meta->parent_word) != NULL) {
						ft_canonicalize_chain_compress(ft, holder_flag,
							holder_depth, &lctx, holder_meta);
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
		ft_removeall_fault_scope_enter();
		ret = ft_detach_node(ft, &lctx, head_slot,
			ft_get_parent_slot(holder_meta, ft), key_len, true,
			dead_cell, ft->ordered_list ? &pub : NULL, NULL, NULL,
			NULL, -1 /* leaf key removed: detach owns the -1 */,
			NULL, false, NULL, NULL);
		ft_removeall_fault_scope_exit();
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
		 * -EAGAIN is a peer, -ENOMEM is memory, and the two report
		 * differently: BUSY_ERROR sends the caller back to retry,
		 * MEMORY_ERROR tells it to free something first.  The
		 * distinction is only as good as the sources, which is what
		 * blocked this mapping before -- an acquire that could not
		 * allocate its own lock txn used to reach the commit as
		 * @acquire_miss and abort, arriving here as -EAGAIN with no
		 * allocation failure anywhere in the errno.  That path now
		 * carries @acquire_enomem and commits MEMORY_ERROR instead, so
		 * every -EAGAIN landing here is a peer.
		 */
		*result_node = NULL;
		return ret == -EAGAIN ? CDS_FT_STATUS_BUSY_ERROR :
			CDS_FT_STATUS_MEMORY_ERROR;
	}

	return CDS_FT_STATUS_OK;
}

enum cds_ft_status cds_ft_remove_all(struct cds_ft *ft,
		struct cds_ft_iter *iter,
		struct cds_ft_node **result_node)
{
	struct urcu_txn optxn;
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
	s = _cds_ft_remove_all_locked(ft, iter, result_node, &optxn);
	urcu_txn_end(&optxn);
	return s;
}

