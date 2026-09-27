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
 *     the external chain head.
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
 *
 * @freeze_leaf (@freeze_len entries), when set, is the removed external leaf
 * whose chain freezes in @txn.  It is recorded HERE, after each arm's acquire,
 * because its chain's holder can be @iter_node_flag itself -- a leaf straight
 * under the compressed parent -- which only these acquires take.  The caller
 * used to record it first: -DFT_DEBUG_CHAIN_HOLD read it holding nothing
 * 4 / 19 / 1 times per ft_inv leg (per-node / exponential / root-only), all
 * in the retiring arm below.
 */
static
int ft_detach_node_replace_compressed_parent(struct cds_ft *ft,
		struct cds_ft_inode_flag *iter_node_flag,
		unsigned int iter_depth,
		const struct ft_lock_ctx *ctx,
		struct cds_ft_inode_flag **detach_parent_flag_ptr,
		struct cds_ft_node *topmost_external_nodes,
		struct cds_ft_inode_flag *elevated_old_child,
		struct ft_ord_cell *fuse_cell,
		struct ft_remove_pub *pub,
		struct ft_detach_run *run,
		struct ft_flip_txn *txn,
		bool record_only,
		long count_delta,
		struct cds_ft_node *freeze_leaf,
		unsigned int freeze_len)
{
	/*
	 * Does the op hold the SKIP_X dual's derived grandparent?  ANSWERED BY
	 * THE ACQUIRE at each of the three publishes below -- the return of
	 * ft_lock_skip_dual_gp, which is REGISTERED-or-SHARED, i.e. the
	 * assert's own predicate.
	 *
	 * ☠ NOT by ft_skip_dual_gp_held.  That helper asks @ctx, and a
	 * ft_owner_ctx_holds() miss means "the registry cannot see this hold",
	 * never "the op does not hold it": an acquire taken through
	 * ft_dlm_acquire_set_at registers into NO ctx at all (it receives the
	 * ctx as a const pointer), so its marks are invisible to the ask until
	 * the owner's commit hands them to a txn.  That is exactly how
	 * ft_node_recompact's dual site came to record "0 held of 805202" about
	 * a word it holds on every fine-trie publish.  The ask is kept below as
	 * a MEASUREMENT of that registry, and its answer must not be wired to a
	 * record kind.
	 */
	bool dual_gp_held = false;

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
		if (!record_only)
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
		const struct ft_glue *split_g = record_only ?
			ft_glue_that_split(ctx, cn) : NULL;

		/*
		 * ☠ @cn IS A NODE THIS VERY COMMIT DESTROYS -- so do not write it.
		 *
		 * A same-trie rekey whose graft SPLIT @cn arrives here with @cn on
		 * the glue's retire list and @cn's one live child @cn_child already
		 * re-homed, by a deferred edge, into the fresh cluster the forward
		 * publish installs.  The promote below would then write @cn twice
		 * over: into @cn->child, a body nothing will read after the flip;
		 * and, through _ft_publish_to_parent's SKIP_X arm, into @cn's own
		 * home slot -- which is the WORD THE GLUE'S PUBLISH REPOINTS.  Two
		 * records on one word whose expected-olds disagree: the dual chains
		 * {skip(cn_child) -> skip(head)} and the glue then chains
		 * {skip(cn_child) -> top}, urcu_txn_record_chain finds
		 * r->new_ptr != old_ptr, POISONS the descriptor, the commit aborts
		 * and the op re-plans the identical shape.  MEASURED on the
		 * shipping build: 310,024 poisons in 40 s on one such shape, a
		 * fresh cluster allocated per attempt.
		 *
		 * ☠ AND CURING ONLY THAT COLLISION COMMITS A WRONG TRIE.  MEASURED
		 * with the collision cured (a read-your-own-writes expected-old on
		 * the glue's publish): the call returns, cds_ft_verify is RED --
		 * "head cell {parent <cn>} != expected {owner <branch>}" -- and the
		 * next insert wedges.  The glue's build wired the branch's old
		 * direction to @cn_child, believing it survives; the promote
		 * DISSOLVES @cn_child and puts the external head in its place.  So
		 * the two steps disagree about what lives under that branch, and
		 * the collision is the first SYMPTOM of that disagreement, not the
		 * defect.
		 *
		 * RE-HOME instead of publishing: hand the promoted head to the
		 * deferred edge that was going to move @cn_child, and store it into
		 * the fresh cluster's own slot.  The edge is where BOTH of the
		 * promote's writes belong once @cn is retired -- the forward one
		 * because the fresh slot is @cn->child's successor, and the head's
		 * back-edge because ft_reparent_record records it against the fresh
		 * owner with a read-your-own-writes expected-old, atomically with
		 * the publish.  @cn's home slot is then written ONCE, by the glue,
		 * and its raw expected-old is correct because nothing else touched
		 * the word.
		 *
		 * The store into the fresh slot is a PLAIN one: that cluster is
		 * build-invisible until the forward publish, and the txn publishes
		 * reachability, not interiors.
		 */
		if (split_g) {
			struct ft_glue *g = (struct ft_glue *) split_g;
			int idx = g->old_dir_deferred;

			/*
			 * The drop-old-direction arm leaves the caller no detach
			 * to run (@old_dir_dropped), so reaching here with it set
			 * would mean two steps both claiming the old direction.
			 */
			urcu_assert_debug(!g->old_dir_dropped);
			/*
			 * ...AND THE SAME FOR A SUBSTITUTED OLD DIRECTION.  Two of
			 * the three substitutions leave @old_dir_deferred at -1 and
			 * are caught by the refusal below; the PROMOTE arm defers a
			 * LIVE head and so hands back a VALID index, which this
			 * frame would clobber with its own promoted chain.  The
			 * caller that arms @old_dir_replace runs no detach (the
			 * `!done` guard at its step 3), so this is unreachable --
			 * asserted rather than assumed, because the refusal below
			 * cannot see it.
			 */
			urcu_assert_debug(!g->old_dir_replace.done);
			/*
			 * REFUSE, do not guess, on every shape the re-home is not
			 * written for: a re-home through a fresh suffix node
			 * (whose parent slot holds a SKIP form this frame cannot
			 * re-encode), a build that re-homed nothing, a rank-stats
			 * trie (the fresh cluster was sized with @cn_child's key
			 * count, which the promote changes), and the fused arm
			 * (its cell unsplice rides the publish we are removing).
			 * Terminal -- the shape is deterministic, so -EAGAIN would
			 * spin -- and BEFORE ANY SIDE-EFFECT: nothing is acquired,
			 * recorded or stored above this point.
			 */
			if (g->old_dir_via_suffix || idx < 0 || ft->rank_stats ||
					count_delta || fuse_cell || run)
				return -EDOM;
			/* A bulk fold: the FT-wide writer lock is the exclusion. */
			if (freeze_leaf) {
				ft_ch_audit_ctx(ft, txn, ctx, freeze_leaf);
				FT_HLIST_PLAN_OBSERVE(ft,
					ft_flip_txn_handle(txn), freeze_leaf,
					freeze_len, 0);
				ft_hlist_freeze_chain_prepare(ft,
					ft_flip_txn_handle(txn), freeze_leaf,
					freeze_len);
			}
			g->deferred[idx].child =
				(struct cds_ft_inode_flag *) topmost_external_nodes;
			CMM_STORE_SHARED(*g->deferred[idx].slot,
				(struct cds_ft_inode_flag *) topmost_external_nodes);
			return 0;
		}
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
				if (!record_only)
					ft_flip_txn_destroy(txn);
				return -EAGAIN;
			}
			/*
			 * A member the op ALREADY held rode an earlier acquire,
			 * which recorded its release; a second record would settle
			 * the single word twice.
			 */
			if (!cn_held.shared) {
				ft_flip_txn_lock_register_held(txn, &cn_held);
				ft_flip_txn_record_release_lock(txn, cn_held.lock,
					cn_held.lock_snap);
			}
			/*
			 * ☠ @elevated_old_child IS A PLAN READ of cn->child,
			 * taken before this lock, and the publish below records
			 * it as the expected-old of a word the op now HOLDS --
			 * a blind SW store (ft_flip_txn_record_tag's held_sw)
			 * that pins nothing.  A peer insert of a longer key
			 * republishes cn->child IN PLACE under this very word
			 * (ft_insert_compressed_past_child) without moving
			 * cn's state: re-read it here, under the lock.
			 */
#ifndef FT_DEBUG_NO_CN_CHILD_RECHECK
			if (!record_only &&
					ft_resolve_flip_proxy(rcu_dereference(
						cn->child)) != elevated_old_child) {
				FT_DBG_RETRY_SITE();
				ft_flip_txn_destroy(txn);
				return -EAGAIN;
			}
#endif
		}
		/* After the acquire: see the function header. */
		if (freeze_leaf) {
			ft_ch_audit_ctx(ft, txn, ctx, freeze_leaf);
			/*
			 * ★ VALIDATE THE PLAN UNDER THE LOCK -- see
			 * ft_chain_compress_fused's twin.  @freeze_leaf's holder
			 * is held: @cn itself when no climb happened, else an
			 * orphan below it that ft_detach_node's orphan walk locked
			 * and holds until its `end:` sweep.  @freeze_len was derived before the acquire, and
			 * the chain parks SW: a duplicate appended in between
			 * would be frozen over BLIND and lost.  The note that
			 * stood here ("MEASURED ... never stale, 0 of 65,434")
			 * was a fact about unpreempted load; the chain_compress
			 * twin went stale 9 times in ~6M once the remover was
			 * preempted.  Nothing recorded or stored yet on this
			 * path; same unwind as the acquire's bail above.
			 */
			if (ft->lock_fine && !record_only &&
					!ft_hlist_chain_plan_ok(ft,
						ft_flip_txn_handle(txn),
						freeze_leaf, freeze_len)) {
				FT_HLIST_PLAN_BAIL();
				FT_DBG_RETRY_SITE();
				ft_flip_txn_destroy(txn);
				return -EAGAIN;
			}
			FT_HLIST_PLAN_OBSERVE(ft, ft_flip_txn_handle(txn),
				freeze_leaf, freeze_len, 1);
			ft_hlist_freeze_chain_prepare_checked(ft,
				ft_flip_txn_handle(txn),
				freeze_leaf, freeze_len);
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

				ft_flip_txn_record_head_back_edge_owned(txn,
					(void **) &cell->parent,
					cell->parent, cn_flag,
					ft_back_edge_owner(cell->parent)
					FT_BE_SITE(FT_BE_DETACH_CN_PARENT, ctx));
			} else {
				ft_flip_txn_record_head_back_edge_owned(txn,
					(void **) &topmost_external_nodes->prev,
					topmost_external_nodes->prev, cn_flag,
					ft_back_edge_owner(
						topmost_external_nodes->prev)
					FT_BE_SITE(FT_BE_DETACH_CN_PARENT, ctx));
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
			struct ft_pub_rec rec = { .ctx = ctx, .n = 0,
				.mtxn = txn ? txn->mtxn : NULL };

			/* The caller folds only with neither. */
			urcu_assert_debug(!record_only);

			/* VALIDATE (§4.B): guard the LIVE kept compressed node cn.
			 * DLM: cn's RELEASE was acquired + recorded up front under
			 * lock_fine, so skip the incremental lock here. */
			if (!ft->lock_fine)
				ft_flip_txn_lock_or_guard_parent(ft, txn, ctx,
					ft_compressed_node_flag(cn), iter_depth);
			/*
			 * ☠ §9.3's THIRD MEMBER IS **NOT** TAKEN HERE, and the
			 * attempt HANGS.  This publish parent IS a compressed
			 * node, so the republish does emit a SKIP_X dual into
			 * cn's own parent -- exactly the shape the other four
			 * producers now acquire.  Adding
			 * ft_lock_skip_dual_gp here wedges
			 * test_rekey_same_path_atomic_or_refused: "child killed
			 * by signal 14 (HANG -- the same-path move did not
			 * return)", 2 of 2, standalone, where HEAD is 2 of 2
			 * green.  Bisected to THESE TWO CALLS: the other three
			 * acquires are green with this one removed, and red with
			 * it alone.
			 *
			 * The mechanism is the one the choke-point note at
			 * ft_flip_txn_lock_or_guard_parent already names -- "an
			 * op can arrive here holding this publish target's lock
			 * already, taken for a DIFFERENT member of its own set.
			 * Re-acquiring then misses against the op's OWN hold,
			 * which sets @acquire_miss and aborts the commit, and the
			 * caller retries into the identical shape: the op waits
			 * on itself, forever."  The line just above says cn's
			 * lock was "acquired + recorded up front under
			 * lock_fine", so this frame is precisely an op that
			 * arrives holding part of the set.
			 *
			 * ☠ AND "THE OP ALREADY HOLDS IT" MEASURED 0 held of
			 * 40894 across all 153 inv rows (ft_skip_dual_gp_held) --
			 * but READ THAT NUMBER AS A REGISTRY ANSWER, NOT AS A
			 * LOCK ANSWER.  The ask consults @ctx, and an acquire
			 * taken through ft_dlm_acquire_set_at registers into no
			 * ctx at all, so a hold that lives in a caller's local
			 * anchor array is invisible to it.  The identical 100%/0%
			 * split at ft_node_recompact's dual site was exactly that
			 * -- 0 of 805202 through @ctx, 490411 of 490411 asked of
			 * the set the function actually took.  The reading here
			 * (that ft_detach_node's {C, P, GP} names the grandparent
			 * of the op's ITERATION parent, a different node from the
			 * dual's back-pointer-derived owner) may still be right;
			 * it is simply not what this instrument measured.
			 *
			 * It does not matter for the KIND, because the ACQUIRE
			 * below answers the question properly -- REGISTERED or
			 * SHARED, the assert's own predicate -- and that is what
			 * @dual_gp_held now carries.  The ask stays as a
			 * measurement of the registry's reach.
			 */
			(void) ft_skip_dual_gp_held(ft, ctx,
				ft_compressed_node_flag(cn),
				txn ? txn->mtxn : NULL);	/* measurement */
			/*
			 * §9.3's THIRD MEMBER, taken here at last.  It used to
			 * HANG -- and the cause was never this site: on a COARSE
			 * trie ft_lock_skip_dual_gp acquired nothing yet still
			 * fell through to ft_flip_txn_lock_or_guard_parent's
			 * `guard:` tail and planted a §4.B guard the commit could
			 * not satisfy, so every attempt aborted and the caller
			 * retried forever (measured: 8,427,520 calls, ALL
			 * FT_LOG_EXIT_NOT_FINE, while the row never returned).
			 * With the helper's coarse early-out the same call is
			 * green.
			 */
			dual_gp_held = ft_lock_skip_dual_gp(ft, ctx, txn,
				ft_compressed_node_flag(cn),
				txn ? txn->mtxn : NULL);
			_ft_publish_to_parent(ft, ft_compressed_node_flag(cn),
				&cn->child,
				(struct cds_ft_inode_flag *) topmost_external_nodes,
				elevated_old_child, &rec, dual_gp_held);
			if (ft_remove_commit_rec(ft, &rec, fuse_cell, run,
					txn, false) > 0)
				/* Peer won: nothing installed (txn consumed). */
				return -EAGAIN;
			pub->armed = true;
		} else {
			struct ft_pub_rec rec = { .ctx = ctx, .n = 0,
				.mtxn = txn ? txn->mtxn : NULL };

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
			/*
			 * ☠ §9.3's THIRD MEMBER IS **NOT** TAKEN HERE, and the
			 * attempt HANGS.  This publish parent IS a compressed
			 * node, so the republish does emit a SKIP_X dual into
			 * cn's own parent -- exactly the shape the other four
			 * producers now acquire.  Adding
			 * ft_lock_skip_dual_gp here wedges
			 * test_rekey_same_path_atomic_or_refused: "child killed
			 * by signal 14 (HANG -- the same-path move did not
			 * return)", 2 of 2, standalone, where HEAD is 2 of 2
			 * green.  Bisected to THESE TWO CALLS: the other three
			 * acquires are green with this one removed, and red with
			 * it alone.
			 *
			 * The mechanism is the one the choke-point note at
			 * ft_flip_txn_lock_or_guard_parent already names -- "an
			 * op can arrive here holding this publish target's lock
			 * already, taken for a DIFFERENT member of its own set.
			 * Re-acquiring then misses against the op's OWN hold,
			 * which sets @acquire_miss and aborts the commit, and the
			 * caller retries into the identical shape: the op waits
			 * on itself, forever."  The line just above says cn's
			 * lock was "acquired + recorded up front under
			 * lock_fine", so this frame is precisely an op that
			 * arrives holding part of the set.
			 *
			 * ☠ AND "THE OP ALREADY HOLDS IT" MEASURED 0 held of
			 * 40894 across all 153 inv rows (ft_skip_dual_gp_held) --
			 * but READ THAT NUMBER AS A REGISTRY ANSWER, NOT AS A
			 * LOCK ANSWER.  The ask consults @ctx, and an acquire
			 * taken through ft_dlm_acquire_set_at registers into no
			 * ctx at all, so a hold that lives in a caller's local
			 * anchor array is invisible to it.  The identical 100%/0%
			 * split at ft_node_recompact's dual site was exactly that
			 * -- 0 of 805202 through @ctx, 490411 of 490411 asked of
			 * the set the function actually took.  The reading here
			 * (that ft_detach_node's {C, P, GP} names the grandparent
			 * of the op's ITERATION parent, a different node from the
			 * dual's back-pointer-derived owner) may still be right;
			 * it is simply not what this instrument measured.
			 *
			 * It does not matter for the KIND, because the ACQUIRE
			 * below answers the question properly -- REGISTERED or
			 * SHARED, the assert's own predicate -- and that is what
			 * @dual_gp_held now carries.  The ask stays as a
			 * measurement of the registry's reach.
			 */
			(void) ft_skip_dual_gp_held(ft, ctx,
				ft_compressed_node_flag(cn),
				txn ? txn->mtxn : NULL);	/* measurement */
			/*
			 * §9.3's THIRD MEMBER, taken here at last.  It used to
			 * HANG -- and the cause was never this site: on a COARSE
			 * trie ft_lock_skip_dual_gp acquired nothing yet still
			 * fell through to ft_flip_txn_lock_or_guard_parent's
			 * `guard:` tail and planted a §4.B guard the commit could
			 * not satisfy, so every attempt aborted and the caller
			 * retried forever (measured: 8,427,520 calls, ALL
			 * FT_LOG_EXIT_NOT_FINE, while the row never returned).
			 * With the helper's coarse early-out the same call is
			 * green.
			 */
			dual_gp_held = ft_lock_skip_dual_gp(ft, ctx, txn,
				ft_compressed_node_flag(cn),
				txn ? txn->mtxn : NULL);
			_ft_publish_to_parent(ft, ft_compressed_node_flag(cn),
				&cn->child,
				(struct cds_ft_inode_flag *) topmost_external_nodes,
				elevated_old_child, &rec, dual_gp_held);
			if (ft_remove_commit_rec(ft, &rec, NULL, NULL, txn,
					record_only) > 0)
				/* Peer won: nothing installed (txn consumed). */
				return -EAGAIN;
		}
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
			if (!record_only)
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
			if (!record_only)
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
					if (!record_only)
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
			/*
			 * ☠ NOT FOR A SHARED HOLD.  set[0] dedupes onto a word
			 * this op already holds -- at root-only, an orphan whose
			 * anchor is the root, which on this arm IS src_cn -- and
			 * ft_detach_freeze_one has then already registered that
			 * word in the orphan txn.  Registering it again here had
			 * every pre-commit terminal release ONE word TWICE
			 * (ft_flip_txn_lock_release_all does not dedupe): the
			 * second release clears a peer's fence if the root was
			 * re-taken in between.  The helper's own contract says
			 * callers gate on @shared (its assert), exactly as set[1]
			 * below does; the retire below handles @shared itself.
			 * Found by the adversarial review of an under-lock plan
			 * check for this arm's freeze, whose -EAGAIN bail reaches
			 * this terminal on every stale plan.
			 */
			if (!src_held.shared) {
				ft_flip_txn_lock_register_held(txn, &src_held);
				ft_flip_txn_record_anchor_release(txn, &src_held,
					src_cn_meta_a);
			}
			if (pub_parent && !set[1].held.shared) {
				ft_flip_txn_lock_register_held(txn, &set[1].held);
				ft_flip_txn_record_release_lock(txn,
					set[1].held.lock, set[1].held.lock_snap);
			}
			dlm_a2 = true;
			/*
			 * The same plan read as the topmost arm's, and nothing
			 * here records cn->child at all: retiring src_cn for an
			 * EMPTY fresh node is only right while its child is
			 * still the one the climb condemned.
			 */
#ifndef FT_DEBUG_NO_CN_CHILD_RECHECK
			if (!record_only &&
					ft_resolve_flip_proxy(rcu_dereference(
						src_cn->child)) != elevated_old_child) {
				FT_DBG_RETRY_SITE();
				free_cds_ft_node_unpublished(ft, fresh);
				ft_flip_txn_destroy(txn);
				return -EAGAIN;
			}
#endif
		}
		/*
		 * After the acquire: the leaf sits straight under @src_cn, which
		 * this arm retires and has just locked.  See the function header.
		 */
		if (freeze_leaf) {
			ft_ch_audit_ctx(ft, txn, ctx, freeze_leaf);
			/*
			 * ★ VALIDATE THE PLAN UNDER THE LOCK -- same reason,
			 * same holder argument and same disposition as the
			 * promote arm above.  Only the
			 * unpublished @fresh exists; same unwind as the acquire's
			 * bail above.
			 */
			if (ft->lock_fine && !record_only &&
					!ft_hlist_chain_plan_ok(ft,
						ft_flip_txn_handle(txn),
						freeze_leaf, freeze_len)) {
				FT_HLIST_PLAN_BAIL();
				FT_DBG_RETRY_SITE();
				free_cds_ft_node_unpublished(ft, fresh);
				ft_flip_txn_destroy(txn);
				return -EAGAIN;
			}
			FT_HLIST_PLAN_OBSERVE(ft, ft_flip_txn_handle(txn),
				freeze_leaf, freeze_len, 2);
			ft_hlist_freeze_chain_prepare(ft, ft_flip_txn_handle(txn),
				freeze_leaf, freeze_len);
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
			struct ft_pub_rec rec = { .ctx = ctx, .n = 0,
				.mtxn = txn ? txn->mtxn : NULL };

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
			/*
			 * §9.3's THIRD MEMBER: @pub_parent may be compressed, in
			 * which case this publish also emits its SKIP_X dual into
			 * a grandparent the op holds nothing of.  Acquire it from
			 * the same derivation the record names.  (Kind still
			 * false -- the eight producers flip TOGETHER; see
			 * ft_node_recompact's dual site, where the last of them
			 * was shown to hold its owner after all.)
			 */
			dual_gp_held = ft_lock_skip_dual_gp(ft, ctx, txn,
				pub_parent, txn ? txn->mtxn : NULL);
			_ft_publish_to_parent(ft, pub_parent,
				pub_slot,
				ft_node_flag(fresh, 0),
				*pub_slot, &rec, dual_gp_held);
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
			if (ft_remove_commit_rec(ft, &rec, NULL, NULL, txn,
					record_only) > 0) {
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
 *
 * @h_dies_first: @h is in a frame that returns before @txn's terminal runs --
 * ft_detach_node's @orphan_held[] under a DEFERRED commit (@record_only: the
 * rekey commits @shared_txn after the detach has returned).  The hand-off then
 * links no silencer (ft_flip_txn_lock_own_frame): measured with
 * -DFT_DEBUG_STACK_SILENCER, the terminal otherwise wrote through a pointer
 * into the dead frame, once per ft_unit run at exponential spacing.
 */
static inline
void ft_detach_freeze_one(struct ft_flip_txn *txn,
		const struct ft_lock_ctx *ctx,
		struct ft_held_anchor *h, struct cds_ft_metadata *m,
		bool h_dies_first)
{
#ifdef FT_DEBUG_FREEZE_SILENCER_ON_STACK
	h_dies_first = false;	/* red control: link it anyway */
#endif
	if (!h->shared) {
		unsigned int slot = 0;
		bool owned = false;
		bool retires;

		/* Register BEFORE recording: the record asks who owns the word. */
		if (h->lock != m) {
			slot = ft_flip_txn_lock_own_frame(txn, h, h_dies_first);
			owned = true;
		}
		/*
		 * The record's own early-out is the ONLY place that knows
		 * whether this word ends RELEASED or RETIRED, and the terminal
		 * scrub needs that answer (it cannot re-derive it from the word
		 * afterwards without racing a peer's retire).
		 */
		retires = ft_flip_txn_record_anchor_release(txn, h, m);
		if (owned && retires)
			txn->locks[slot].tombstone_terminal = true;
#ifndef FT_DEBUG_FREEZE_TOMBSTONE_MW
		/*
		 * ...AND WHERE THE ANCHOR IS THE ORPHAN, SAY THAT THIS TXN HOLDS
		 * IT.  That mark stays with the caller (no registry slot, see
		 * above), so the retire's record-time ownership question
		 * answered NO and the fused {LOCK|s -> TOMBSTONE|s} went MW --
		 * 1.33M records per ft_inv FT_INV_MW=1 run at per-node, every
		 * one on a word this op itself took.  Its expected-old check was
		 * the MW-CAS era's detect-and-abort for a peer that grew the
		 * orphan; under the lock that peer cannot even start
		 * (ft_meta_nr_child_inc waits on FT_STATE_LOCK), and a peer's
		 * {live -> live} validate cannot land on a LOCKED word either.
		 * @covered answers ft_flip_txn_owns without taking on a release
		 * (the caller's sweep keeps that), so the terminal parks SW.  It
		 * settles outside the registry's late pass, which is harmless for
		 * a TOMBSTONE: the word is unlockable from the moment it lands.
		 * A full @covered drops the entry and the record stays MW.
		 */
		if (h->lock == m)
			ft_flip_txn_cover_member(txn, m);
#endif
	}
	ft_flip_txn_record_retire_anchored(txn, ctx, h, m);
}

static
void ft_detach_freeze_orphans(struct cds_ft *ft, struct ft_flip_txn *txn,
		const struct ft_lock_ctx *ctx,
		struct cds_ft_inode_flag **orphans, int nr_orphans,
		struct cds_ft_inode_flag *trailing_skip_cn_flag,
		struct ft_held_anchor *held,
		struct ft_held_anchor *trailing_held,
		bool held_dies_first)
{
	int i;

	for (i = 0; i < nr_orphans; i++) {
		struct cds_ft_metadata *m = ft_node_compressed(orphans[i])
			? cds_ft_item_to_metadata((struct cds_ft_inode *)
				ft_compressed_node_ptr(orphans[i]))
			: cds_ft_item_to_metadata(ft_node_ptr(orphans[i]));
		/*
		 * An orphan is a LINK of the condemned chain -- keyless and
		 * single-child, or the one junction whose head the climb
		 * promoted -- and it is held here: a second child means the
		 * walk that collected it left the plan, and retiring it
		 * takes a live key with it (the phase-2 target check in
		 * ft_detach_node is the measured instance).
		 */
		urcu_assert_debug(ft_node_compressed(orphans[i]) ||
			ft_meta_nr_child_load(m) <= 1);

		if (held)
			ft_detach_freeze_one(txn, ctx, &held[i], m,
				held_dies_first);
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
			ft_detach_freeze_one(txn, ctx, trailing_held, m,
				held_dies_first);
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
 * A no-op where no descent ran.  ☠ NOT a no-op under per-node granularity any
 * more: ft_descent_enter_node returns early there for the ANCHOR TABLE, but the
 * ancestor-ledger push sits BEFORE that return.  So this extends the LEDGER at
 * every spacing, which is why the descent it is handed must be one that is
 * allowed to write (see ft_detach_node's copy, which is not).
 */
static inline
unsigned int ft_walk_extend(struct ft_descent *d, bool valid,
		struct cds_ft_inode_flag *nf, unsigned int depth,
		unsigned int span)
{
	if (valid) {
		d->nf = nf;
		d->depth = depth;
#ifdef FT_DEBUG_ANC_LEDGER
		if (d->anc_rec)
			uatomic_inc(&ft_anc_rec_copy_push);
#endif
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
 * table covers it.
 *
 * ☞ AN -EAGAIN FROM HERE IS NO LONGER THE WHOLE CALL FAILING, and this comment
 * used to say it was ("remove_all has NO retry loop ... so an -EAGAIN from here
 * is the whole CALL failing and the caller's to retry").  cds_ft_remove_all now
 * loops: the refusal ages a conflict on the op's persistent txn and the attempt
 * is re-derived, so refusing here costs an ATTEMPT rather than the CALL.  It is
 * not a MEMORY_ERROR either -- the tail maps `-EAGAIN` to that retry and
 * `-ENOMEM` to MEMORY_ERROR -- which only became trustworthy once
 * @acquire_enomem gave an allocation failure inside the acquire its own channel,
 * instead of arriving as an -EAGAIN no peer produced.  Both sentences are
 * corrected rather than deleted: each read as an open bug, and sends the next
 * reader after something already fixed.
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

/*
 * The link below orphan @nf on a detach's downward walk: a compressed node's
 * child, or an internal node's first (single) child.  Read it only once @nf is
 * marked: the value is what the walk's verdict on the NEXT link rests on.
 */
static inline
struct cds_ft_inode_flag *ft_detach_walk_next(const struct cds_ft *ft,
		struct cds_ft_inode_flag *nf)
{
	struct cds_ft_inode_flag *next = NULL;
	unsigned int key;

	if (ft_node_compressed(nf))
		return ft_compressed_node_ptr(nf)->child;
	for (key = 0; key < 256; key++) {
		next = ft_node_get_nth(ft, nf, NULL, (uint8_t) key,
			FT_PF_NONE);
		if (next)
			break;
	}
	return next;
}

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
#ifdef FT_DBG_RA_UNSPLICE
/*
 * ARM and YIELD for cds_ft_remove_all's TWO-COMMIT ordered-list tail, and it
 * was built because a RED CONTROL LIED.  Restoring the discarded-status form of
 * that unsplice left inv_concurrent_remove_all_list GREEN, which reads as "the
 * drive-forward buys nothing" -- and is instead "the branch never ran": every
 * removal in that row's LEAF shapes fuses the unsplice into its structural
 * flip, so @pub is always armed and the two-commit tail is dead code there.
 * Measured with this counter: two_commit_arm=0 over 150 rounds x 64 keys x 3
 * lanes.  Arm presence and arm YIELD are different facts, and only the counter
 * tells a green control apart from an unexecuted one.
 */
unsigned long ft_dbg_ra_unsplice_arm, ft_dbg_ra_unsplice_abort;
unsigned long ft_dbg_ra_unsplice_fused;
/* ...and the back-pointer/descent disagreement the exponential bail refuses. */
unsigned long ft_dbg_ra_stale_descent;

__attribute__((destructor))
static void ft_dbg_ra_unsplice_report(void)
{
	fprintf(stderr, "FT RA UNSPLICE: two_commit_arm=%lu abort=%lu fused=%lu\n",
		ft_dbg_ra_unsplice_arm, ft_dbg_ra_unsplice_abort,
		ft_dbg_ra_unsplice_fused);
	fprintf(stderr, "FT RA STALE DESCENT: exponential_bail=%lu\n",
		ft_dbg_ra_stale_descent);
}
# define FT_DBG_RA_UNSPLICE_ARM()	uatomic_inc(&ft_dbg_ra_unsplice_arm)
# define FT_DBG_RA_UNSPLICE_ABORT()	uatomic_inc(&ft_dbg_ra_unsplice_abort)
# define FT_DBG_RA_UNSPLICE_FUSED()	uatomic_inc(&ft_dbg_ra_unsplice_fused)
# define FT_DBG_RA_STALE_DESCENT()	uatomic_inc(&ft_dbg_ra_stale_descent)
#else
# define FT_DBG_RA_UNSPLICE_ARM()	do { } while (0)
# define FT_DBG_RA_UNSPLICE_ABORT()	do { } while (0)
# define FT_DBG_RA_UNSPLICE_FUSED()	do { } while (0)
# define FT_DBG_RA_STALE_DESCENT()	do { } while (0)
#endif

#ifdef FT_ENABLE_TRACING
/*
 * ARM YIELD, not arm presence.  "The check compiled in" and "the check ever
 * REFUSED anything" are different facts, and only the second says whether a
 * guard is doing work: the first attempt at the detach arm was armed ~100k
 * times per run and refused NOTHING, because its target had been re-read so
 * late that it was comparing a value against itself.  A guard without a yield
 * counter cannot tell that apart from a race that simply did not happen.
 */
unsigned long ft_dbg_plan_stale_ext, ft_dbg_plan_stale_detach;
/* ...and the orphan walk's own stale-plan refusal. */
unsigned long ft_dbg_orphan_walk_stale;
/* ...split by arm: the record_only FOLD path is the one whose abort
 * safety rests on the CALLER destroying the shared txn. */
unsigned long ft_dbg_plan_stale_fold, ft_dbg_plan_stale_alone;
#endif

/*
 * What the OP INTENDS, as opposed to what the TRIE currently holds.
 *
 * ☠ THE DISTINCTION IS THE WHOLE POINT.  A collapse rests on a picture of the
 * boundary -- how many children it has, which one survives, whether it carries
 * an external head -- and that picture used to be read by the CALLER, before
 * the callee took the boundary's lock.  Under fine locking the mark IS the
 * exclusion, so a fact read before it is worth nothing at commit time: a peer
 * that legally holds the same lock changes the node in between, and the
 * collapse ratifies a picture that is already false.  Measured: a peer
 * published a fresh junction carrying a live key into a body slot of the
 * boundary and committed; this collapse re-entered, retired the boundary, and
 * freed the key with it.
 *
 * So the callee DERIVES every trie fact under its own mark and trusts none from
 * the caller.  What it cannot derive is the op's TARGET -- you can hoist a
 * QUERY, you cannot hoist a TARGET: only the op knows which child it came to
 * remove.  That, and only that, is what this carries.
 *
 * @detach_child / @detach_byte: the body child this commit removes from the
 *   boundary, and its byte.  NULL when the op removes no body child (the
 *   post-removal collapses, whose entry is the EXTERNAL one).
 *   ★ At the SHAPE-D fold it is the climb's own @elevated_old_child with its
 *   byte @n -- the pair the climb condemned, both fixed BEFORE the boundary is
 *   acquired.  @n is set once (ft_node_find_child on @cur) after the last
 *   elevation, and @detach_node_flag_ptr is a slot INSIDE @cur throughout, so
 *   the pair names the boundary's own edge.  ☞ ft-remove.h already enforces it
 *   PRE-fence (@plan_old_child); carrying it here is that same predicate moved
 *   past the acquire, which is the only place it can speak for the commit.
 * @expect_ext: the external head the op's shape says the boundary carries --
 *   the head it is itself removing, or NULL for the callers whose gate asserts
 *   there is none.  Never a value read speculatively from the trie: each caller
 *   names the head it came to unlink.
 */
struct ft_chain_compress_intent {
	struct cds_ft_inode_flag *detach_child;
	struct cds_ft_node *expect_ext;
	uint8_t detach_byte;
};

/*
 * The fact the collapse rests on that the boundary's own mark did not use to
 * cover: its EXTERNAL HEAD.  Read HERE, under that mark, and compared against
 * the head the op named -- never against a picture the caller took before it.
 * True == the plan no longer describes the trie, so the caller must bail.
 *
 * ...and the second: the child this commit DETACHES.  A peer's publish into
 * that slot is a SAME-SLOT VALUE SWAP -- it moves neither the count nor the
 * survivor -- so this is the only word that can see it.
 */
static inline
bool ft_chain_compress_plan_stale(struct cds_ft_inode_flag *iter_node_flag,
		const struct ft_chain_compress_intent *intent)
{
	if (ft_node_external_nodes(iter_node_flag) != intent->expect_ext) {
#ifdef FT_ENABLE_TRACING
		uatomic_inc(&ft_dbg_plan_stale_ext);
#endif
		return true;
	}
	/*
	 * ☠ RAW, like for like.  @detach_child is the climb's own slot WORD
	 * (@elevated_old_child), so it must be compared against the slot word --
	 * ft_node_get_nth_skip, "the slot value as-is, including skip-compressed
	 * pointers".  ft_node_get_nth RESOLVES a skip-compressed word to the
	 * compressed node reached through its child's back-pointer, which is a
	 * DIFFERENT address, so comparing the two forms is unequal for every
	 * chain-shaped branch and equal for every plain one: measured 347 vs 441
	 * per run, a shape-determined constant, i.e. a PERMANENT -EAGAIN.  That
	 * is what wedged ft_unit at test 2 on the first attempt.
	 */
	if (intent->detach_child &&
			ft_node_get_nth_skip(iter_node_flag, NULL,
				intent->detach_byte, FT_PF_NONE)
					!= intent->detach_child) {
#ifdef FT_ENABLE_TRACING
		uatomic_inc(&ft_dbg_plan_stale_detach);
#endif
		return true;
	}
	return false;
}

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
	if (h->shared) {
		/*
		 * Its word was taken by an EARLIER member of this op, so it owes
		 * no release and no terminal -- but the op DOES hold it, and the
		 * record-time ownership question is asked against @txn.  Say so
		 * (ft_flip_txn's @covered).
		 */
		ft_flip_txn_cover_member(txn, h->member);
		return;
	}
	ft_flip_txn_lock_register_held(txn, h);
	ft_flip_txn_record_anchor_release(txn, h, node);
}

/*
 * The DEEP pending-publish shape, asked identically by the caller's shape-D gate
 * and by the collapse itself so the two can never drift apart.
 *
 * The shallow shape @pending_child covers is "the BOUNDARY's own survivor slot
 * IS the caller txn's pending forward publish target".  This is the same
 * collision ONE COMPRESSED LEVEL DEEPER: the survivor is a compressed node whose
 * OWN child slot is that target.  The collapse absorbs that node's run into the
 * merged node, so the merged node's child is taken from @child_cn->child -- and
 * that is precisely the slot the caller's commit is about to overwrite with its
 * fresh cluster top.  Fusing around the COMMITTED occupant there builds the
 * merged run on a node the same commit retires and publishes the moved cluster
 * into a slot of the node this collapse replaces, stranding the whole moved
 * subtree (measured: rekey_merge over a multi-level survivor chain loses both
 * moved keys, and the committed occupant's re-parent collides with the glue's
 * free-list tombstone on one state word -- MW here, SW there).
 *
 * Deliberately asked on the RESOLVED compressed pointer rather than on the slot
 * the boundary holds: the boundary's flag may be the skip-encoded spelling of
 * the same node, and it is the node's own child WORD ADDRESS that the caller
 * armed.  A build without FEATURE_FT_COMPRESS has no compressed survivor and so
 * never takes this arm -- which is why the ablated builds never showed the loss.
 */
static inline
bool ft_chain_compress_deep_pending(struct cds_ft_inode_flag *survivor,
		const struct ft_flip_txn *txn)
{
	struct cds_ft_compressed_node *cn;

	if (!survivor || !txn || !txn->pending_pub_slot)
		return false;
	if (!ft_node_compressed(survivor))
		return false;
	cn = ft_compressed_node_ptr(survivor);
	return &cn->child == txn->pending_pub_slot;
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
 *
 * @pending_child (FOLD, pending-publish): non-NULL ONLY on the record_only fold
 * when the boundary slot @surviving_byte names is the caller txn's OWN pending
 * forward publish target (@pending_pub_slot -- armed before this fold runs by
 * ft_glue_set_publish on the GLUE and merge lanes, or directly by the NOSPLIT
 * store commit in ft-graft.h).  It is the FRESH, UNPUBLISHED cluster top that same
 * commit installs there, and it -- not the committed occupant -- is the child
 * the merged node must be built around: fusing around the committed value
 * builds new_cn on a node the same commit retires and lets the commit publish
 * the cluster into a slot of the boundary this collapse replaces, stranding
 * the whole moved subtree (the shape the sibling two-child-BP pin measures).
 * The COMMITTED value stays in @surviving_child: the plan re-validation and
 * the lock-set are about what the trie holds NOW, and the committed occupant's
 * retire belongs to the GLUE (its free list), never to this collapse -- which
 * is also what keeps the two from colliding on one state word (the measured
 * MW-PSO-vs-SW-tombstone chain conflict).  ☞ Except that @pending_child may BE
 * the committed occupant: a duplicate-chain splice publishes the dst's own
 * app-owned head back into its slot, so the pending value is live, not fresh,
 * and its back edge must be RECORDED (see @pending_top_live at the back-edge
 * arm), never stored.  The collapse then reports the fold
 * (@pending_pub_folded / @pending_pub_node) so the glue commit skips its
 * forward publish and re-bases its count delta onto the merged node.
 * ☑ A COMPRESSED @pending_child IS ABSORBED, not refused.  The caller used to
 * turn the whole move away here on the grounds that "absorbing a fresh
 * unpublished compressed top would retire a node that was never published and
 * re-aim its cluster's deferred edges".  The first half is the answer, not the
 * obstacle -- a node that was NEVER PUBLISHED is not retired at all: it is
 * untracked from the glue and freed outright, with no grace period, because no
 * reader ever had a path to it.  The second half holds for ONE edge: when the
 * pending top's child is the FRESH cluster node below it, its back-pointer is a
 * plain store into a private body and nothing is queued.  When that child is
 * LIVE -- the dst's own head, handed back by a duplicate-chain merge at the
 * run's end -- its back edge IS a queued glue entry naming the pending top,
 * and the back-edge arm re-aims it at the merged node (ft_glue_reaim_edge).
 *
 * So the run simply grows by the pending top's bytes -- @parent_cn ++
 * @surviving_byte ++ pending->key_bytes -- and takes the pending top's CHILD.
 * That is the same absorption the DEEP fold already performs one level down,
 * and it is what keeps the canonical form: fusing AROUND a compressed pending
 * top would put two adjacent compressed nodes in the trie, which cds_ft_verify
 * rejects, and falling back to ft_node_replace_ptr would publish a one-child
 * internal, which skip mode does not allow to persist.  Both alternatives are
 * illegal; this one is the only legal product.
 *
 * The ONE thing that is still refused is a run that cannot be SPELLED:
 * @merged_len past FT_SKIP_LEN_MAX takes the same `return 1` every other
 * over-long merge takes, and the caller falls back.  That is a property of the
 * trie, not of this writer.
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
		unsigned int freeze_len,
		long count_delta,
		unsigned int count_reserve,
		struct ft_flip_txn *shared_txn,
		bool record_only,
		struct cds_ft_inode_flag *pending_child,
		const struct ft_chain_compress_intent *intent,
		struct ft_chain_compress_reclaim *reclaim)
{
	struct cds_ft_compressed_node *parent_cn, *child_cn;
	struct cds_ft_metadata *parent_cn_meta;
	unsigned int parent_len, child_len, merged_len;
	struct cds_ft_metadata *new_cn_meta;
	struct cds_ft_compressed_node *new_cn;
	struct cds_ft_inode_flag *new_cn_flag;
	struct cds_ft_inode_flag **publish_slot;
	/*
	 * The publish target is the CALLER's fresh, unpublished body (see the
	 * arm below): the forward edge is a plain interior store, not a txn
	 * record, and neither the §4.B guard nor the dating applies to it.
	 */
	bool pub_home_private = false;
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
	/*
	 * The child the merged node is BUILT around: the pending cluster top
	 * under the fold's substitution (see @pending_child at the header),
	 * else the committed survivor.  Every plan-facing read (the lock-set,
	 * the re-validation) keeps @surviving_child.
	 */
	struct cds_ft_inode_flag *build_child =
		pending_child ? pending_child : surviving_child;
	/*
	 * The DEEP spelling of the same substitution (see
	 * ft_chain_compress_deep_pending): set once the lock-set arm has derived
	 * @child_cn and the plan re-validation has ratified the survivor, and
	 * from there it steers three things -- the merged node's child value, the
	 * PLAIN back edge that value takes, and the fold report to the glue.
	 */
	bool deep_fold = false;
	/*
	 * The pending top when it is a RUN: its bytes join the merged run and
	 * its child becomes the merged node's.  It is FRESH and UNPUBLISHED, so
	 * it is untracked from the glue and freed outright below -- never
	 * retired, and never on the free list the commit drains.
	 */
	struct cds_ft_compressed_node *pending_cn = NULL;
	/* ...and the glue that BUILT it, which is the only owner that may free it. */
	struct ft_glue *pending_glue = NULL;
	/*
	 * The absorbed run's CHILD is LIVE (not built by @pending_glue): its back
	 * edge is the glue's queued entry, which names the run this fold frees.
	 * See the back-edge arm.
	 */
	bool absorbed_child_live = false;

	assert(surviving_child);
	/* The substitution exists only on the fold; the caller gates both. */
	assert(!pending_child || record_only);
	/*
	 * A COMPRESSED pending top is ABSORBED into the merged run (see the
	 * header).  Recover its node from either spelling: ft_glue_set_publish
	 * announces whatever the builder handed it, which is the PLAIN flag for
	 * a fresh run whose child edge is still deferred, and the SKIP form once
	 * the split re-encoded it.
	 */
	/*
	 * ☠ NEVER THE COMMITTED OCCUPANT.  This absorption FREES the pending
	 * top unpublished, with no grace period, on the premise that it is
	 * fresh.  A pending value that IS the slot's committed occupant (see
	 * @pending_top_live below) is live and published; absorbing it would
	 * free a node readers hold.  No armer hands back a live compressed top
	 * today (the one live-top armer requires an external head); the
	 * identity test keeps that a property of this code, not of the callers.
	 */
	if (pending_child && pending_child != surviving_child &&
			(ft_node_compressed(pending_child) ||
			ft_node_skip_compressed(pending_child))) {
		/*
		 * ☠ THE OWNER MUST BE IN HAND BEFORE THE ABSORPTION, not looked
		 * for afterwards.  Freeing the run without untracking it leaves it
		 * in @built for ft_glue_abort to free a SECOND time on any later
		 * bail.  The glue is what armed @pending_pub_slot in the first
		 * place (ft_glue_set_publish), so it is always there for this
		 * shape -- and if it somehow is not, the absorption does not
		 * happen and the caller's refusal stands, which is the only answer
		 * this frame can give without an owner.
		 */
		pending_glue = ft_glue_of_ctx(ctx);
		if (pending_glue)
			pending_cn = ft_node_skip_compressed(pending_child) ?
				ft_skip_to_compressed(ft, pending_child) :
				ft_compressed_node_ptr(pending_child);
	}
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
				+ FT_CELL_LOCKSET_MAX_RECORDS
				+ 1 /* §4.B parent guard */
				+ 1 /* back-edge (parent, offset) pair: the state-word edge */
				+ ft_freeze_reserve(ft, (unsigned int) nr_orphans
					+ (trailing_orphan ? 1 : 0))
				+ freeze_len * FT_HLIST_FREEZE_MAX_EDGES
				+ count_reserve /* nr_keys walk from publish_parent (R3 fold) */
				+ 1 /* @new_cn's born-lock release */);
	}
	if (!txn)
		return -ENOMEM;	/* nothing touched: caller aborts */
#ifdef FT_COLLAPSE_CLAIM
	/*
	 * §4 STEP B3's DRY RUN, the a3c75659 tool aimed at the CHAIN-COMPRESS
	 * collapse: point B0's owner assert at this txn so every record it plants
	 * without owning is named, at an abort, on a build otherwise
	 * byte-identical to the unarmed one.
	 *
	 * ☞ CLAIMED AT CREATION, EARLIER THAN THE ARM WOULD BE.  A dry run has to
	 * claim BEFORE the records it wants checked, and for this site that is
	 * before the lock-set acquire below -- so read a miss as "this record is
	 * planted before its owner is registered" FIRST.  That ORDERING class has
	 * been the answer more often than a missing acquire.
	 *
	 * ☠ And read a miss as "the registry cannot SEE this hold" before "the op
	 * does not HOLD it": where a caller's SWEEP owns the mark's clearing, the
	 * absence is by design (ft_flip_txn_owns).
	 *
	 * ☠ @record_only hands us the CALLER's txn.  Claim it anyway -- the claim
	 * sets no record kind, so it is behaviour-neutral, and the fold's records
	 * are exactly the ones this step has to judge.
	 */
	ft_flip_txn_claim_per_op_armable(ft, txn);
#endif
	FT_TP(chain_compress_enter, (const void *) iter_node_flag,
		(const void *) iter_meta,
		(const void *) CMM_LOAD_SHARED(iter_meta->external_nodes),
		(const void *) freeze_leaf, (const void *) dead_cell,
		plan_nr_child, (int) record_only);
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
		/*
		 * Derived from the BUILD child: absorption is about the run the
		 * merged node will actually hold.  The committed occupant's fate
		 * (retire) belongs to the glue, which holds its own lock on it.
		 */
		/*
		 * ☠ NEVER THE PENDING TOP.  @child_cn is the LIVE run below the
		 * boundary, and everything downstream treats it as one: a §7.1
		 * below-pivot lock member, a retire registration, and a
		 * @reclaim->child_cn entry the commit frees through a grace
		 * period.  The pending top is FRESH and UNPUBLISHED -- no reader
		 * can reach it, so there is nothing to lock, nothing to retire and
		 * no grace period to wait; it is untracked from the glue and freed
		 * outright where the run absorbs it.  Routing it here produces the
		 * right STRUCTURE and the wrong OWNERSHIP.
		 */
		child_cn = (ft_node_compressed(build_child) &&
				(!pending_cn ||
					ft_compressed_node_ptr(build_child) != pending_cn))
			? ft_compressed_node_ptr(build_child) : NULL;
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
				ft_flip_txn_lock_register_held(txn, &set[si].held);
				ft_flip_txn_record_release_lock(txn,
					set[si].held.lock, set[si].held.lock_snap);
			} else {
				/* Held via an earlier member; owes no release. */
				ft_flip_txn_cover_member(txn,
					set[si].held.member);
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
		/* BUILD child, as in the DLM arm above. */
		/* BUILD child, and never the pending top -- see the DLM arm. */
		child_cn = (ft_node_compressed(build_child) &&
				(!pending_cn ||
					ft_compressed_node_ptr(build_child) != pending_cn))
			? ft_compressed_node_ptr(build_child)
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
	 * ★ THE PLAN, UNDER THE FENCE.  Everything the collapse rests on is read
	 * HERE, on the far side of the boundary's own mark, and compared against
	 * what the OP said it came to do (@intent) -- never against a picture the
	 * caller took before the mark, which a peer holding the same lock is
	 * entitled to invalidate.  Four things, and the last two are why keys
	 * were being lost:
	 *
	 *  1. the child COUNT (@plan_nr_child: 2 for the shape-D fold that
	 *     retires one of the two, 1 for the post-removal collapses).  Free:
	 *     nr_child lives inside the state word, so the acquire's own snapshot
	 *     carries it.
	 *  2. the SURVIVOR still at @surviving_byte.  ☞ This one cannot be
	 *     derived instead of compared: when the survivor is compressed it is
	 *     a member of the lock-set above, so it has to be named to be locked.
	 *     A mismatch bails, which is equivalent.
	 *  3. the EXTERNAL HEAD.  ☠ Never checked before.  A peer's
	 *     ft_insert_park_external_nodes writes exactly this word, under this
	 *     same lock, and moves neither nr_child nor any slot above -- so
	 *     (1) and (2) both pass and the collapse retires a boundary that is
	 *     holding a live key.
	 *  4. the child THIS COMMIT DETACHES.  ☠ Never checked before.  A peer's
	 *     publish into that slot is a SAME-SLOT VALUE SWAP: the count does
	 *     not move, the survivor does not move, and the collapse drops a
	 *     freshly published subtree.  MEASURED as the loss, in the act.
	 *
	 * Never fires single-writer.
	 */
	if (caa_unlikely(ft_state_nr_child(iter_held.node_snap) != plan_nr_child ||
			ft_node_get_nth(ft, iter_node_flag, NULL,
				surviving_byte, FT_PF_NONE)
					!= surviving_child ||
			ft_chain_compress_plan_stale(iter_node_flag, intent))) {
#ifdef FT_ENABLE_TRACING
		if (record_only)
			uatomic_inc(&ft_dbg_plan_stale_fold);
		else
			uatomic_inc(&ft_dbg_plan_stale_alone);
#endif
		if (!record_only)
			ft_flip_txn_destroy(txn);
		return -EAGAIN;
	}
	/*
	 * Ask the DEEP shape only now: @child_cn is derived by whichever lock-set
	 * arm ran, and the re-validation immediately above is what makes
	 * @surviving_child (hence @build_child) the value the trie still holds.
	 * The two substitutions are mutually exclusive by construction -- one
	 * pending slot cannot be both the boundary's survivor slot and the
	 * survivor's own child slot -- and the caller gates them as one if/else.
	 */
	deep_fold = record_only && child_cn &&
		ft_chain_compress_deep_pending(build_child, shared_txn);
	assert(!(deep_fold && pending_child));

	parent_len = parent_cn ? parent_cn->len : 0;
	child_len = child_cn ? child_cn->len : pending_cn ? pending_cn->len : 0;
	merged_len = parent_len + 1 + child_len;

	if (merged_len > FT_SKIP_LEN_MAX) {
		/*
		 * ☠ AN OVER-LONG ABSORPTION HAS NO FALLBACK TO FALL BACK TO.
		 * `return 1` means "merge does not apply, caller falls back",
		 * and the caller's fallback is ft_node_replace_ptr -- which is
		 * exactly the product the @pending_child contract above calls
		 * ILLEGAL for this shape: it publishes a one-child internal that
		 * skip mode does not allow to persist, and the run it should have
		 * absorbed is still compressed beneath it.
		 *
		 * MEASURED on a release build with a 126-byte run (the boundary
		 * is exact: merged_len 127 clean, 128 red): the op returns
		 * SUCCESS and cds_ft_verify answers `compressed node ... node
		 * lock set at rest (leaked lock)`.  The leaked lock is a
		 * PRE-EXISTING defect of that fallback -- the same shape with the
		 * dst placed elsewhere is red at the parent commit too -- but
		 * REACHING it from here is not: before the absorption existed,
		 * this shape took the shape-D gate's terminal refusal before any
		 * side-effect.  A pre-existing root cause is not a
		 * non-regression; what changed for the caller is a clean
		 * FT_REKEY_UNCOVERED becoming SUCCESS on a corrupt trie.
		 *
		 * So the absorption refuses TERMINALLY instead, which is what a
		 * run that cannot be spelled deserves: it is a property of the
		 * trie, not of this writer.  Only the absorbing shape takes this
		 * exit.
		 *
		 * ☠ AND THE DEEP FOLD TAKES THE SAME EXIT, for the same reason.  It
		 * was left on the fallback because its live-top case was
		 * unreachable; the through-a-run merge arm made it reachable
		 * ({ab, a+128*'c'}, dst the long key, src "ab": the src junction
		 * collapses into the dst run, the merged run cannot be spelled),
		 * and the fallback then published the one-child internal, with
		 * the caller's subsumption reporting a fold that never happened.
		 * MEASURED: verify RED at 128 bytes, clean at 127; refused clean
		 * with this line.
		 */
		if (pending_cn || deep_fold)
			return -EDOM;
#ifndef FT_DEBUG_OVERLONG_COLLAPSE_FALLBACK
		/*
		 * ☑ AN ORDINARY OVER-LONG MERGE IS SPELLED, NOT REFUSED.  Its
		 * fallback was never a legal product: ft_node_replace_ptr leaves
		 * the boundary a one-child keyless internal, which cds_ft_verify
		 * refuses in skip mode -- single-threaded, on the default build:
		 * {ab, a+128*'c'}, remove "ab" (127 bytes clean).  But a run
		 * longer than a skip pointer can encode is not unspellable:
		 * cds_ft_insert builds exactly that shape for a long unique
		 * suffix -- ONE compressed node, published through a PLAIN
		 * compressed flag (ft_publish_compressed picks the encoding by
		 * length), and verify accepts it.  So build the same node here
		 * and let the publish below take the plain flag.  The only bound
		 * left is the node's own 8-bit length, which a key of
		 * FT_MAX_KEY_LEN bytes below a non-root boundary cannot reach.
		 */
		if (merged_len > UINT8_MAX) {
			if (!record_only)
				ft_flip_txn_destroy(txn);
			return 1;
		}
#else
		/* Merge does not apply: caller falls back (fences cleared). */
		if (!record_only)
			ft_flip_txn_destroy(txn);
		return 1;
#endif
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
	else if (pending_cn)
		memcpy(&new_cn->key_bytes[parent_len + 1],
			pending_cn->key_bytes, child_len);
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
		/*
		 * DEEP FOLD: @child_cn->child IS the caller's pending forward
		 * publish target, so the value the merged node must carry is the
		 * cluster top that commit installs there -- never @child_child,
		 * the occupant the same commit retires.  The proxy check above is
		 * kept unconditionally: a peer parked on this word is contention
		 * whichever value we are about to take.
		 */
		new_cn->child = deep_fold ? shared_txn->pending_pub_val
					  : child_child;
	} else if (pending_cn) {
		/*
		 * ABSORBED: the merged run carries the pending top's bytes, so
		 * the child it must hold is the pending top's OWN child -- most
		 * often the fresh cluster node below it, which no reader can
		 * reach yet.
		 *
		 * A fresh child needs no special back edge: @pending_child
		 * non-NULL already routes the wiring to the PLAIN ft_set_parent
		 * arm below, which is the right one for a fresh, unpublished
		 * child.
		 *
		 * ☠ AND THE ABSORBED RUN LEAVES THE GLUE'S BUILT SET WITH IT.
		 * It was never published, so it is not retired and owes no grace
		 * period -- but the caller's ft_glue_abort frees everything in
		 * @built on a bail, and this node is about to stop existing, so
		 * it must be untracked FIRST or that abort double-frees it.  The
		 * pair is the one ft_glue_untrack's own header names ("chain-merge
		 * absorbs a freshly-built compressed wrapper").
		 *
		 * The order matters: TAKE THE CHILD FIRST, then untrack, then
		 * free -- and all three before any later bail can run, so no path
		 * sees the node in a half-owned state.
		 */
		new_cn->child = pending_cn->child;
		/*
		 * ☠ ...EXCEPT WHEN IT IS NOT FRESH.  A duplicate-chain merge at
		 * the run's end (a moved key equal to a dst key) hands back the
		 * dst's own APP-OWNED HEAD as the run's child: ft_merge_build_run
		 * queues its back edge in the glue, aimed at the run, because it
		 * is live.  Read here, before the run is freed.
		 */
#ifndef FT_DEBUG_ABSORB_LIVE_CHILD_UNAIMED
		absorbed_child_live = !ft_glue_is_fresh(ft, pending_glue,
			new_cn->child);
#endif
		/*
		 * ☠ @built IS NOT THE ONLY NAME THE GLUE KEPT.  ft_glue_set_publish
		 * stored the same flag in @top, and the txn in @pending_pub_val;
		 * untracking only the @built entry leaves both naming freed
		 * memory.  Measured: on the PLAIN spelling the glue reaches
		 * ft_glue_apply_deferred with @top dangling.  It is LATENT today
		 * -- the forward-publish consumer is skipped by
		 * @pending_pub_folded, and the other two consumers are reachable
		 * only from the legacy KEY_SHORTER graft_swap -- but the fix
		 * belongs here, beside the free, not in a caller two files away
		 * that happens to skip the read.
		 */
		ft_glue_untrack(ft, pending_glue, pending_cn);
		if (pending_glue->top == pending_child)
			pending_glue->top = NULL;
		if (shared_txn && shared_txn->pending_pub_val == pending_child) {
			shared_txn->pending_pub_val = NULL;
			shared_txn->pending_pub_plain = NULL;
		}
		free_compressed_node_unpublished(ft, pending_cn);
	} else {
		new_cn->child = build_child;
	}
	ft_meta_nr_child_set(new_cn_meta, 1);
	/*
	 * Build the merged node with its POST-removal count (top + @count_delta,
	 * i.e. top - 1 when this merge retires a key -- R3 fold): the collapsed
	 * top node's count minus the disappearing key.  @count_delta 0 (a
	 * count-neutral canonicalize, or a leaf-detach whose -1 the caller still
	 * pre-decrements) copies it verbatim, as before.  No-op when !rank_stats.
	 *
	 * ONE OWNER EACH under the @pending_child fold: this store owns ONLY the
	 * DETACH's side (the boundary's committed count + @count_delta, i.e.
	 * minus the moved-out subtree), and the GLUE's count arm owns the
	 * ATTACH's side -- it adds g->count_delta to @pending_pub_node (this
	 * merged node) at commit-record time and walks the shared stable parent
	 * (ft_glue_txn_commit_edges' folded-publish arm).  Baking the attach's
	 * delta here too would double-count it.
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
		/*
		 * ☠ READ-YOUR-OWN-WRITES, and this is the whole defect when the
		 * collapse is FOLDED into a caller's commit.  A same-trie rekey
		 * grafts first: its ADD-recompaction rebuilds the node that holds
		 * @parent_cn and RECORDS @parent_cn's re-home into the very txn
		 * this collapse now publishes through.  A recorded edge is not
		 * parked yet -- it lives only in the descriptor -- so a raw
		 * derivation answers with the PRE-OP slot, which sits inside the
		 * copy THIS COMMIT RETIRES.  ft_txn_parent_slot_at exists for
		 * exactly this and its header says so.
		 *
		 * MEASURED, single-threaded, default build: the two answers
		 * disagree ({old z, slot in old z} vs {fresh z, slot in fresh z})
		 * and the collapse's publish landed in the retired copy -- the
		 * live trie kept BOTH the merged node's new home and the
		 * uncollapsed chain, one node under two parents, three tombstoned
		 * nodes still reachable and the merged node leaked.  The op
		 * returned OK on it.
		 */
		publish_slot = ft_txn_parent_slot_at(parent_cn_meta, ft,
			txn ? txn->mtxn : NULL, &publish_parent);
		if (record_only && shared_txn) {
			struct cds_ft_inode_flag *raw_parent = NULL;

			(void) ft_txn_parent_slot_at(parent_cn_meta, ft, NULL,
				&raw_parent);
			/*
			 * ☠ THE HOME THIS COMMIT WILL LEAVE IS A NODE NOBODY
			 * HAS PUBLISHED YET, AND THIS COLLAPSE CANNOT PUBLISH
			 * INTO IT.  The two answers disagree only when the
			 * CALLER re-homed @parent_cn in this very txn -- a
			 * same-trie rekey whose graft recompacted the node that
			 * holds it -- so @publish_parent is the caller's FRESH,
			 * BUILD-INVISIBLE copy.  Its slot is not a word this op
			 * holds, and the forward edge's owner assert says so
			 * (measured: __ft_flip_txn_record_tag_ctx's
			 * ft_flip_txn_owns arm fires on the first attempt).  A
			 * private home wants the PLAIN store the SKIP_X dual
			 * arm makes for exactly this case
			 * (ft_dual_home_is_private) -- with the §4.B guard and
			 * the publish-parent dating dropped with it, since
			 * neither has anything to say about a node no reader
			 * can reach.  That publish is NOT WRITTEN YET.
			 *
			 * Until it is, REFUSE THE MOVE TERMINALLY, with the
			 * rekey's carve-out code and for its stated meaning
			 * ("this writer cannot express this shape") -- never
			 * -EAGAIN, which would spin on a deterministic shape.
			 *
			 * ☠ THIS IS NOT THE "BEFORE ANY SIDE-EFFECT" REFUSAL
			 * THE TWO -EDOM ARMS IN ft_detach_node MAKE, and the
			 * difference is worth stating rather than inheriting
			 * their wording.  By here this collapse has ALREADY
			 * acquired its lock set and recorded the releases into
			 * the caller's SHARED txn -- MEASURED at this return:
			 * 5 registered locks (every state word reading
			 * LOCK-set) and 8 records in the descriptor.  What
			 * makes the refusal clean is not that nothing happened
			 * but that nothing is READER-VISIBLE and nothing is
			 * ORPHANED: @new_cn is freed unpublished here, the
			 * caller's ft_rekey_graft_simple_attempt detach_bail
			 * runs ft_flip_txn_destroy on the shared txn, and that
			 * drains the registry (ft_flip_txn_lock_release_all,
			 * nr_locks=5) leaving every one of those words live and
			 * unlocked -- the contract stated at
			 * ft_flip_txn_record_release_lock.  MEASURED end to
			 * end: 40,000 refused moves in one process move RSS by
			 * 0 bytes, the key count is unchanged, and cds_ft_verify
			 * is clean after each.
			 *
			 * ☠ WITHOUT THIS THE OP RETURNS OK ON A CORRUPT TRIE.
			 * The raw derivation aims the publish at the copy the
			 * same commit RETIRES, so the collapse lands in a dead
			 * node: the live trie keeps the uncollapsed chain AND
			 * the merged node's new home -- one node under two
			 * parents, three tombstoned nodes still reachable, the
			 * merged node leaked, and the next insert under that
			 * prefix never returns.  MEASURED on 58 of 4328
			 * enumerated shapes of this family.
			 */
			/*
			 * ☑ SERVED: PUBLISH INTO THE PRIVATE HOME WITH A PLAIN
			 * STORE.  The two answers disagree exactly when the
			 * CALLER re-homed @parent_cn in this very txn, so
			 * @publish_parent is its FRESH, BUILD-INVISIBLE copy --
			 * a node NO READER CAN REACH until the caller's one
			 * commit publishes it.  The txn publishes REACHABILITY,
			 * NOT INTERIORS: a word inside a private body is an
			 * interior write, so it wants the bare store, and with
			 * it the §4.B guard and the publish-parent DATING both
			 * fall away -- neither has anything to say about a node
			 * no reader can reach, and the dating would refuse
			 * outright because the descent never saw it.
			 *
			 * ☞ THIS IS THE ANSWER THE SKIP_X DUAL ALREADY GIVES
			 * ONE ARM OVER, and passing @rec = NULL is how it is
			 * said: _ft_publish_to_parent_meta's forward edge and
			 * its dual BOTH fall to their `else` store, which is
			 * the same dispatch ft_dual_home_is_private makes for
			 * the dual alone.  The refusal here was the last piece
			 * of that argument left unwritten.
			 *
			 * On ABORT the caller frees the fresh copy and the
			 * store dies with it; a RECORDED edge could not say
			 * that -- it would aim a CAS at a word the op does not
			 * hold, which is the owner assert this refusal was
			 * standing in front of.
			 */
			if (caa_unlikely(publish_parent != raw_parent)) {
				pub_home_private = true;
			}
			/*
			 * ☠ THE SAME DISAGREEMENT AS THE PROMOTE ARM'S, ONE ARM
			 * OVER.  When the caller's graft SPLIT @parent_cn, this
			 * collapse absorbs a node that commit RETIRES and
			 * republishes it at @parent_cn's home -- the word the
			 * glue's forward publish repoints -- while the split has
			 * already re-homed @parent_cn's child under its fresh
			 * cluster.  Two steps, one word, incompatible plans.
			 *
			 * ☠ AND --enable-rcu-debug CANNOT SEE IT HERE.  MEASURED
			 * on a release probe build: this shape poisons the
			 * descriptor THREE times per attempt and every one is
			 * SW-against-SW -- the kind assert never fires, @cn's
			 * state word is retired twice, and a {live -> live}
			 * validate chains after a tombstone.  The livelock is the
			 * only detector, which is why the refusal is what makes
			 * the shape reportable at all.
			 *
			 * Serving it needs the merged node re-based BELOW the
			 * split point -- a different product from the one this
			 * function builds -- so refuse terminally with the
			 * rekey's carve-out code, exactly as the arm above does,
			 * and for the same stated meaning.
			 */
			if (caa_unlikely(ft_glue_that_split(ctx, parent_cn) !=
					NULL)) {
				free_compressed_node_unpublished(ft, new_cn);
				return -EDOM;
			}
		}
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
		/*
		 * ☐ UNAUDITED, and named here rather than left silent: this arm
		 * still derives its slot RAW, and its latch check compares
		 * PARENTS, so a same-txn re-home to the same parent at a changed
		 * OFFSET would pass it.  No shape reaching it through a folded
		 * rekey has been constructed -- a skeptic built six candidates
		 * (no run above the junction, a root-level junction) and every
		 * one came back clean or refused-intact -- but "not reproduced"
		 * is not "unreachable".  Same class as the parent_cn arm above.
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
	if (!pub_home_private &&
			!ft_lock_ctx_depth_of_parent(ft, ctx, publish_parent,
			parent_cn ? parent_depth : iter_depth, &pub_depth)) {
		free_compressed_node_unpublished(ft, new_cn);
		if (!record_only)
			ft_flip_txn_destroy(txn);
		return -EAGAIN;
	}
	/*
	 * ★ VALIDATE THE FREEZE'S PLAN UNDER THE LOCK -- the fused detach's
	 * check (ft_detach_node), owed here since the chain parked SW.
	 *
	 * @freeze_len reached this collapse as cds_ft_remove's literal "1 =
	 * this key's SOLE entry", derived from an UNHELD read of @succ_node.
	 * A same-key insert can append a duplicate between that read and the
	 * acquire above -- a remover preempted there is all it takes -- and
	 * the freeze below would then record {NULL -> MARK(NULL)} over the
	 * live @freeze_leaf->next.  Under MW the install CAS absorbed that
	 * stale plan; under SW the record parks and stores BLIND, so the
	 * appended node is dropped with the key, UNMARKED: a lost insert.
	 * MEASURED under preemption (2 cpus, 2 competing spinners): 9 stale
	 * plans here of ~5.9M, one per lost node, per-node and root-only;
	 * 0 with -DNO_FEATURE_FT_CHAIN_SW.  The comment that stood at the
	 * freeze, "MEASURED 0 of 279,029", was a fact about unpreempted load.
	 *
	 * @freeze_leaf's chain holder is HELD here, so the word can be asked:
	 * on the prefix arms (cds_ft_remove, cds_ft_remove_all) the leaf heads
	 * the boundary's external chain and the holder is the boundary itself
	 * (@iter_held, acquired above); on ft_detach_node's shape-D call the
	 * holder is an orphan below the boundary or the trailing skip target,
	 * which ft_detach_node locked in its orphan walk and holds until its
	 * `end:` sweep.  Refused BEFORE any record or store: only the
	 * unpublished @new_cn exists, and this is the same unwind as the bail
	 * just above.  Retriable, and it terminates: the caller re-derives,
	 * sees the successor, and takes the promote / unchain lane.
	 *
	 * @record_only is excluded for the txn's ownership (a caller-owned txn
	 * must not be destroyed here), not for exclusion: no record_only caller
	 * passes a @freeze_leaf today (the rekey fold passes NULL).
	 */
	if (freeze_leaf && !record_only &&
#ifdef FT_RED_NO_CCF_PLAN_CHECK
			/* RED CONTROL: the pre-fix code, for the audit's positive. */
			0 &&
#endif
			!ft_hlist_chain_plan_ok(ft, ft_flip_txn_handle(txn),
				freeze_leaf, freeze_len)) {
		FT_HLIST_PLAN_BAIL();
		FT_DBG_RETRY_SITE();
		free_compressed_node_unpublished(ft, new_cn);
		ft_flip_txn_destroy(txn);
		return -EAGAIN;
	}

	new_cn_flag = ft_compressed_node_flag(new_cn);
	{
		struct ft_pub_rec rec = { .ctx = ctx, .n = 0 };
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
			ft_txn_load(txn->mtxn, (void **) publish_slot,
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
		/*
		 * ☠ THE PENDING TOP IS NOT ALWAYS FRESH.  A duplicate-chain
		 * SPLICE -- one key on each side of a same-parent rekey_merge,
		 * {ab, ac} + rekey_merge(dst "ac", src "ab") -- hands
		 * ft_glue_set_publish the dst's own APP-OWNED HEAD, i.e. the
		 * COMMITTED OCCUPANT of the slot (ft_merge_build appended the src
		 * leaf to its dup chain and handed it back).  The fold's
		 * substitution is then BY IDENTITY, and the node this arm wires is
		 * LIVE: reader-reachable now, its parent word read by every
		 * ordered up-walk.  Two things go wrong with the plain store on
		 * it.  It is a reader-visible mutation BEFORE the commit that no
		 * abort rolls back; and the glue could not take its fresh-child
		 * fast path for a live top, so it QUEUED the same edge -- aimed at
		 * the boundary this collapse retires -- and would re-apply the
		 * dead parent over this store at the flip.  MEASURED, all four
		 * rank/list modes: `cn->len 16` decoded off the freed boundary
		 * through the head's back-pointer, and `head prev != owner`.
		 *
		 * A live pending top takes the RECORD arm below like every other
		 * live child, and ft_glue_apply_deferred drops the glue's queued
		 * entry for it on the strength of @pending_pub_folded.  The
		 * identity test is exact: a committed occupant is reader-reachable
		 * and a fresh top is not, so nothing fresh can compare equal.
		 *
		 * ☞ ONE PRE-COMMIT STORE REMAINS, AND IT IS A NO-OP HERE.  The
		 * record arm's external branch stamps the head's incoming byte
		 * (ft_head_stamp_incoming_byte) before it records, and for a bare
		 * head that store is reader-visible ahead of the publish and not
		 * undone by an abort -- its header says so.  On this shape the
		 * head does not move: it hangs off the same byte of the fresh run
		 * as it did off the boundary, so the stamped value is the value
		 * already there.  That stops being true the day this arm serves
		 * a head that changes slot; it does not today.
		 */
		bool pending_top_live =
			(pending_child && pending_child == surviving_child) ||
			(deep_fold && child_cn &&
				new_cn->child == child_cn->child);

		if (absorbed_child_live && ft_glue_reaim_edge(pending_glue,
				new_cn->child, new_cn_flag, &new_cn->child)) {
			/*
			 * ABSORBED RUN, LIVE CHILD: ONE OWNER, THE GLUE.  The run
			 * this fold freed is still the PARENT of its child's
			 * queued glue entry, and ft_glue_txn_commit_edges records
			 * that entry into this same txn after the fold -- so left
			 * alone it overwrote this arm's plain store and aimed a
			 * live head's parent word at freed memory: cds_ft_verify
			 * "skip-encoded slot slen 2 != cn->len 1" (the length read
			 * off the freed run) and both copies of the key lost to
			 * lookup, on the default build, single-threaded -- {cacb,
			 * baca, ccb} + rekey_merge(dst "cc", src "cac").  The
			 * plain store was wrong on its own too: the head is
			 * reader-reachable until the commit, which no abort would
			 * have rolled back.  Re-aimed at @new_cn, the entry
			 * records the edge atomically with this publish, like
			 * every other live dst-origin child.  (Live with no
			 * queued entry takes the RECORD arm below.)
			 */
		} else if ((pending_child || deep_fold) && !pending_top_live &&
				!absorbed_child_live) {
			/*
			 * FOLD substitution: the child is the FRESH, UNPUBLISHED
			 * cluster top -- build-invisible until the caller's one
			 * commit -- so its back edge is a PLAIN STORE, exactly
			 * the store ft_glue_defer_edge's fresh-child fast path
			 * made when it aimed it at the boundary this collapse
			 * replaces (that fast path stores and does NOT queue, so
			 * nothing re-applies the old parent over this).  A
			 * RECORDED re-parent here would plant an MW parent+PSO
			 * pair on a node the glue's free-list retire also
			 * settles -- the measured record_chain collision that
			 * poisons (release) or asserts (rcu-debug).
			 */
			ft_set_parent(ft, new_cn->child, new_cn_flag,
				&new_cn->child);
		} else {
			/*
			 * @ctx is the op's held set, and handing it over is what
			 * keeps this site from recording a §4.B MW validate on a
			 * word the op already holds SW -- see the call in
			 * ft_record_child_back_edge for the three records that
			 * collide without it.
			 */
			ft_record_child_back_edge(ft, txn, new_cn->child,
				new_cn_flag, &new_cn->child, ctx);
			/*
			 * ...so @new_cn, the fresh parent that edge now names, is
			 * published LOCKED (ft_flip_txn_lock_born_spine): the
			 * edge settles after the decide.  It starts where the
			 * node it replaces started.  Not on the FOLD
			 * (@record_only): that is the rekey's bulk writer, whose
			 * FT-wide lock keeps every point writer out of the commit.
			 */
			if (!record_only && ft->lock_fine && !ft->exclusive) {
				struct ft_born_spine_node sp = {
					.meta = new_cn_meta,
					.start = parent_cn ? parent_depth :
						iter_depth,
					.span = merged_len,
				};

				ft_flip_txn_lock_born_spine(ft, txn, &sp, 1);
			}
		}
		new_cn_pub = ft_publish_compressed(ft, new_cn, new_cn_flag);
		/* VALIDATE (§4.B): lock (or guard-fallback) the LIVE
		 * (great-)grandparent publish_parent -- value-swap target (§10.5).
		 * DLM: under lock_fine the whole lock-set (incl. publish_parent's
		 * RELEASE) was acquired up front, so skip the incremental lock here. */
		if (!ft->lock_fine && !pub_home_private)
			ft_flip_txn_lock_or_guard_parent(ft, txn, ctx,
				publish_parent, pub_depth);
		_ft_publish_to_parent_meta(ft, publish_parent, publish_slot,
			new_cn_pub, pub_expected_old, new_cn_meta, NULL,
			pub_home_private ? NULL : &rec,
			/*slot_owner_nf=*/ publish_parent, false);
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
		/*
		 * @orphan_held is only ever ft_detach_node's own array, and
		 * under @record_only @txn is its caller's: the array's frame
		 * returns before this txn's terminal.
		 */
		ft_detach_freeze_orphans(ft, txn, ctx, orphans, nr_orphans,
			trailing_orphan, orphan_held, trailing_orphan_held,
			record_only);
		/*
		 * PHASE B, STEP B3 -- THE ARM.  ft_detach_freeze_orphans above
		 * holds this op's LAST ft_flip_txn_lock_register (an orphan
		 * chain anchor, where the anchor is not the retired node
		 * itself), and under lock_fine everything else was taken in the
		 * ONE all-or-none acquire far above -- the incremental
		 * lock_or_guard beside the publish is the !lock_fine arm, which
		 * this helper refuses anyway.  So this is where
		 * ft_flip_txn_arm_per_op's "after the op's last register"
		 * contract puts it, and ft_remove_commit_rec below plants the
		 * first record after it.
		 *
		 * ☞ READINESS: the dry run (-DFT_COLLAPSE_CLAIM) claims at txn
		 * CREATION -- strictly earlier -- and is clean on ft_unit and
		 * ft_inv FT_INV_MW=1 both, so every record on this txn already
		 * passes the owner check.
		 *
		 * ☞ WHAT IT CONVERTS is the collapse's forward publish, whose
		 * owner is @publish_parent -- a RELEASE member of the same
		 * all-or-none lock-set.  The retires, the orphan freezes and
		 * the child back-edge are planted ABOVE it and stay MW, which
		 * is stricter and always sound; converting those would mean
		 * arming before this op's own bookkeeping registers, which the
		 * contract does not allow from here.
		 *
		 * ☠ NOT on the FOLD path (@record_only): @txn is the CALLER's
		 * and the completeness of its registry is the caller's
		 * judgement, not this frame's.
		 */
		if (!record_only)
			ft_flip_txn_arm_per_op(ft, txn);
		/*
		 * The removed external leaf (a single-entry chain, so
		 * freeze_leaf->next == NULL) freezes atomically with this same
		 * commit that retires its holder chain (doc §4.B): one MARK(NULL)
		 * edge, the +1 reserved above.  NULL when the caller is not
		 * retiring a leaf through this merge.
		 */
		if (freeze_leaf) {
			ft_ch_audit_ctx(ft, txn, ctx, freeze_leaf);
			/*
			 * The plan was validated under the holder before the
			 * publish (see the bail above @new_cn_flag), which is
			 * what licenses the _checked form.  The observe stays,
			 * so a stale plan past that check still gets counted.
			 */
			FT_HLIST_PLAN_OBSERVE(ft, ft_flip_txn_handle(txn),
				freeze_leaf, freeze_len, 3);
#ifdef FT_DEBUG_FREEZE_STALE_ABORT
			/*
			 * PROBE: the plan was checked under the lock above and
			 * is stale NOW -- so a peer changed the chain while this
			 * op holds what it believes is the holder.  Name the word
			 * we hold and the chain word's raw value (a parked proxy
			 * names the peer's descriptor), end the flight recorder
			 * here, and die: the peer's lock take and chain store are
			 * the last events on its cpu.
			 */
			if (!record_only &&
					!ft_hlist_chain_plan_ok(ft,
						ft_flip_txn_handle(txn),
						freeze_leaf, freeze_len)) {
				void *raw = CMM_LOAD_SHARED(freeze_leaf->next);

				FT_TP(lock_stolen, (const void *) freeze_leaf,
					0u, (const void *) raw, 0u,
					(const void *) iter_held.lock);
				fprintf(stderr, "FT FREEZE STALE AFTER CHECK: leaf %p next raw %p (%s) len %u; this op holds %p (boundary %p meta %p)\n",
					(void *) freeze_leaf, raw,
					ft_txn_is_proxy(raw, FT_HLIST_TAG) ?
						"PARKED PROXY" : "plain",
					freeze_len, (void *) iter_held.lock,
					(void *) iter_node_flag,
					(void *) iter_meta);
				ft_trace_capture();
				abort();
			}
#endif
			ft_hlist_freeze_chain_prepare_checked(ft,
				ft_flip_txn_handle(txn),
				freeze_leaf, freeze_len);
		}
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
		/*
		 * The predicate's word, re-read at the LAST instant before the
		 * collapse commits.  A pair (chain_compress_enter.ext,
		 * chain_compress_exit.ext_now) that DIFFERS is the stale
		 * predicate caught in the act, in one event pair, without
		 * having to reconstruct it from edge_records.
		 */
		FT_TP(chain_compress_exit, (const void *) iter_node_flag,
			(const void *) iter_meta,
			(const void *) CMM_LOAD_SHARED(iter_meta->external_nodes),
			-1);
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
		/*
		 * REPORT THE FOLD, at the tail where nothing can bail any more:
		 * the caller's pending forward publish is now live in the merged
		 * node, so the glue commit must SKIP its own publish (it would
		 * install into the boundary this collapse just recorded retired)
		 * and must re-base its order-statistics delta onto the merged
		 * node (@pending_pub_node; the count arm decodes it
		 * compressed-aware).  Flags live on the caller's txn and die
		 * with it: every abort path destroys the txn before (or with)
		 * reclaiming @new_cn, and each retry creates a fresh txn with
		 * these fields cleared.
		 */
		if (pending_child || deep_fold) {
			txn->pending_pub_folded = true;
			txn->pending_pub_node = new_cn_flag;
		}
		return 0;
	}
	FT_TP(chain_compress_exit, (const void *) iter_node_flag,
		(const void *) iter_meta,
		(const void *) CMM_LOAD_SHARED(iter_meta->external_nodes), 0);
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
	/*
	 * INTENT: this is a pure canonicalize.  It removes nothing -- the
	 * caller's gate (ft-remove.h, the post-detach fold) has already asserted
	 * the boundary carries NO external head, so that is what the callee must
	 * still find under the mark, and there is no body child to detach.
	 */
	{
		const struct ft_chain_compress_intent intent = {
			.detach_child = NULL,
			.expect_ext = NULL,
			.detach_byte = 0,
		};

		(void) ft_chain_compress_fused(ft, iter_node_flag, iter_depth, ctx,
			iter_meta,
			surviving_child, surviving_byte,
			1 /* already-committed 1-child boundary */, NULL, NULL,
			NULL, 0, NULL, NULL, 0 /* no orphan chain */,
			NULL, 0 /* no freeze_leaf */,
			0 /* count-neutral canonicalize */, 0,
			NULL, false, NULL /* no pending publish */,
			&intent, NULL);
	}
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

#ifdef FT_DEBUG_STACK_SILENCER
static unsigned long ft_stack_silencer_hits, ft_stack_silencer_scans,
	ft_stack_silencer_locks;
static __attribute__((destructor)) void ft_stack_silencer_report(void)
{
	fprintf(stderr, "FT STACK SILENCER: %lu deferred returns scanned, %lu registered locks examined, %lu still pointing into a dead frame\n",
		uatomic_read(&ft_stack_silencer_scans),
		uatomic_read(&ft_stack_silencer_locks),
		uatomic_read(&ft_stack_silencer_hits));
}
#endif

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
		unsigned int freeze_len,
		long count_delta,
		struct ft_flip_txn *shared_txn,
		bool record_only,
		const struct ft_parent_hint *src_held_hint,
		struct ft_detach_recompact_out *recompact_out,
		bool in_place)
{
	struct cds_ft_metadata *metadata_stack[FT_MAX_DEPTH];
	struct cds_ft_inode_flag *iter_node_flag;
	struct cds_ft_inode *old_recompacted_node = NULL;
	int ret, nr_metadata = 0, nr_clear = 0, nr_branch = 0;
	uint8_t n = 0;
	/*
	 * THE ELEVATED CHAIN: what the climb prunes between the boundary and the
	 * original detach target.  @nr_clear counts its KEYLESS single-child
	 * links; @climb_promoted says whether the climb also emptied a KEYED
	 * single-child junction and lifted its head into the boundary's slot.
	 * That junction is unlinked by the very same publish, so it is an orphan
	 * too, and @nr_elevated is the chain's full length -- the budget both
	 * orphan walks below are bounded on.
	 */
	bool climb_promoted = false;
	int nr_elevated;
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
	 * the next member's anchor on it (the op waits on itself, and a self-refusal
	 * is the one -EAGAIN no peer will ever clear -- so under remove_all's retry
	 * loop it does not surface as an error at all, it SPINS).
	 */
	struct ft_held_anchor orphan_held[FT_MAX_DEPTH + 1];
	int nr_orphan_locked = 0;
	/*
	 * Did §9.3's third member -- the SKIP_X dual's derived grandparent --
	 * actually get ACQUIRED at publish?  The republish below owes that
	 * answer to ft_pub_rec_add, and it may only be answered by the acquire
	 * itself (see the note at its use).
	 */
	bool dual_gp_held = false;
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
	/* The node @topmost_external_nodes was lifted off (see the orphan walk). */
	struct cds_ft_metadata *topmost_src_meta = NULL;
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
	/*
	 * The in-place leaf delete took its holder ahead of the plan (the hoist
	 * above ft_node_replace_ptr).  Read at the B2 arm, which stays off for it.
	 */
	bool leaf_hoisted = false;
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
	 *
	 * ☠ AND SO DOES EVERY LATER ONE (@pair_held).  Each level's pair check
	 * has just proved that the slot holding @cur resolves to @cur, and that
	 * load -- not a re-read at the elevation -- is the level's plan.  The
	 * re-read let a peer's commit land between the two: a remove of the
	 * junction's prefix key collapsed the junction into a skip pointer to
	 * its surviving leaf, the climb read the junction's external (the key
	 * that remove had just frozen) as @topmost_external_nodes, and the
	 * re-read then named the peer's fresh skip node -- so the orphan walk
	 * locked the NEW chain, never the junction, and the promote published
	 * the frozen key back into the slot (the two-writer harness:
	 * STALE-AFTER-RM, LTTng-traced).
	 */
	struct cds_ft_inode_flag *plan_old_child;
	/* This level's pair-checked load of the slot holding @cur. */
	struct cds_ft_inode_flag *pair_held = NULL;
	/*
	 * The shape-D collapse predicate as the PLAN evaluated it (unlocked):
	 * see the residue re-check after ft_node_replace_ptr.
	 */
	bool shape_d_plan = false;
	/*
	 * The detach TARGET as the entry read it (@plan_old_child before the
	 * climb re-anchors it): the node the orphan walk must arrive at once it
	 * has walked the @nr_elevated links the climb counted.
	 */
	struct cds_ft_inode_flag *entry_target;
	/*
	 * The holder slot the climb starts from, and the ONE raw value it read
	 * there -- the value @cur was resolved from.  When the climb elevates, that
	 * slot becomes the drop target, so this pair IS its plan expected-old.
	 */
	struct cds_ft_inode_flag **entry_holder_slot;
	struct cds_ft_inode_flag *entry_holder_raw;
#ifdef FT_DEBUG_DEL_TOMB
	struct cds_ft_inode_flag **dbg_arg_detach_slot = detach_node_flag_ptr;
	struct cds_ft_inode_flag **dbg_arg_parent_slot = detach_parent_flag_ptr;
#endif
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
		/*
		 * ☠ A COPY MAY READ THE LEDGER, NEVER APPEND TO IT.  The orphan
		 * walk below extends this copy (ft_walk_extend), and
		 * ft_descent_enter_node pushes BEFORE its per-node early return
		 * -- so a copy that kept @anc_rec would file the ORPHANED
		 * SUBTREE onto the key path under the original generation, with
		 * ft_anc_ledger_valid still vouching for the result.  @anc_gen
		 * is deliberately kept: reading is exactly what a copy is for.
		 */
#ifdef FT_ANC_LEDGER
		wd.anc_rec = false;
#endif
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
		 * ☠ THE SLOT MUST STILL HOLD THE CALLER'S LEAF.  Every destroy-
		 * style caller is a remove that names the chain head it came to
		 * clear (@freeze_leaf) and checked it at @detach_node_flag_ptr --
		 * at an EARLIER instant.  A peer insert of a LONGER key through
		 * that head (past-child: "zone46" under "zone4") republishes the
		 * slot with a fresh junction that carries the head as its
		 * external_nodes AND the peer's key as a child.  Read here, that
		 * junction is "a detached internal node whose prefix head must be
		 * promoted": the plan below drops the junction -- the peer's key
		 * with it -- and republishes the head this op is REMOVING
		 * (MEASURED: key loss within seconds with two writers, one key
		 * each).  Nothing is built, locked or reserved yet: re-descend.
		 */
#ifndef FT_DEBUG_NO_LEAF_IDENTITY
		if (freeze_leaf && caa_unlikely((struct cds_ft_node *)
				ft_node_ptr(ft_resolve_flip_proxy(detach_child)) !=
					freeze_leaf))
			return -EAGAIN;
#endif
		/*
		 * RESOLVE-THEN-SKIP, in that order (ft-helpers.h:1870, and the
		 * entry-holder read below at :2373 does exactly this:
		 * ft_reanchor_flag(ft, ft_resolve_flip_proxy(raw), ...)).
		 *
		 * ☠ THE FLIP-PROXY HALF WAS MISSING HERE, AND IT FAULTS.  This
		 * slot is loaded RAW above, and a concurrent one-commit splice
		 * parks a type-7 MCAS descriptor in it; the low nibble 0xF then
		 * fails ft_node_external, so the type check below admits it and
		 * ft_flag_to_metadata computes a metadata address from a
		 * DESCRIPTOR pointer.  The comment on the very next statement
		 * already names that outcome -- "the SKIP_X form dispatches the
		 * 0xF flag as a node (ft_node_external fails -> item_to_metadata
		 * faults)" -- and the sibling parent-slot read twenty lines down
		 * says a raw load "feeds that proxy to cds_ft_item_to_metadata()
		 * ... and faults".  Both guard their slot; this one did not, and
		 * it is a measured SEGV site: 2 of 5 sampled rc=139 backtraces
		 * crash at the ft_flag_to_metadata below
		 * (doc/design/ft-stale-disposal-predicate.md §5.17).
		 *
		 * Skip-compressed must still be resolved before the type checks:
		 * a skip pointer with an external child has low bits == 0,
		 * falsely matching ft_node_external and skipping the
		 * external_nodes preservation entirely.
		 *
		 * Sound to resolve: @detach_child is only ever DEREFERENCED here
		 * (its metadata read for external_nodes preservation), never used
		 * as a CAS expected-old -- the same argument the entry-holder
		 * read below makes for @cur.
		 */
		detach_child = ft_resolve_skip_compressed(ft,
			ft_resolve_flip_proxy(detach_child));

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
			if (child_meta && child_meta->external_nodes) {
				topmost_external_nodes = ft_dereference_external(
					child_meta->external_nodes);
				topmost_src_meta = child_meta;
			}
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
	 * settled tree.
	 *
	 * ☠ THIS TESTS THE SLOT THE OP READS, NOT THE ONE IT WRITES.  The slot
	 * an elevation clears -- and a DEL recompaction publishes into -- is
	 * @detach_parent_flag_ptr, and "it is the slot ft_get_parent_slot
	 * recovered INSIDE the new @cur" was an assumption: the address was
	 * derived at an EARLIER instant, and a peer that republishes @cur's
	 * parent since leaves it pointing into the RETIRED body while @cur's
	 * word names the fresh copy.  That slot is validated per level inside
	 * the climb below, against the parent @cur's own word names (MEASURED:
	 * the elevated clear landed 0x30 past a 32-byte parent, in the
	 * neighbouring item; and re-resolving the pair at the refusal finds it
	 * self-coherent EVERY time -- it is a stale plan, not a torn read).
	 */
	if (caa_unlikely(!ft_slot_in_node(cur, detach_node_flag_ptr)))
		return -EAGAIN;
	/* Plan expected-old for a detach that never elevates (see @plan_old_child). */
	plan_old_child = (struct cds_ft_inode_flag *)
		rcu_dereference(*detach_node_flag_ptr);
	entry_target = ft_resolve_flip_proxy(plan_old_child);
	FT_DT_INC(ft_dt_cap0_reach);
	if (!plan_old_child)
		FT_DT_INC(ft_dt_cap0_null);
	/*
	 * ☠ A NULL PLAN EXPECTED-OLD IS A STALE PLAN, NOT A VALUE.  The slot
	 * this op came to clear is ALREADY EMPTY: a peer's in-place delete
	 * landed between the caller's position derivation (ft_locate_chain_head
	 * / node->prev, both validated against a slot that then held the chain)
	 * and this capture.  The recompacting tier never produced this state --
	 * it retired the holder with the delete, so the {C,P,GP} acquire refused
	 * the tombstone -- and the in-place tier leaves a live holder with a
	 * NULL hole instead, which is exactly what the equality guard below
	 * cannot see: it compares the re-read against the plan, and NULL equals
	 * NULL.  Left through, the plan reaches ft_node_replace_ptr with a NULL
	 * expected-old (ft_popcount_node_replace_ptr's `*node_flag_ptr != NULL`,
	 * MEASURED on inv_concurrent_remove_all_nolist: both sides NULL at the
	 * guard, 1 of ~1000 plans), or -- past min_child -- a DEL recompaction
	 * that drops a child that is not there and folds -1 a second time.
	 *
	 * Nothing is built, locked or reserved yet: re-descend.  The wrapper
	 * re-seeds through a fresh lookup, which cannot return a chain head
	 * from an empty slot, so this -EAGAIN clears on the next attempt
	 * (NOT_FOUND, or a fresh plan against whatever a re-insert put here).
	 */
#ifndef FT_DEBUG_NO_NULLPLAN
	if (caa_unlikely(!plan_old_child))
		return -EAGAIN;
#endif
	/*
	 * The same identity, on the read the PLAN rests on.  The check above
	 * is a separate, earlier load; the peer's publish can land between
	 * the two, and then everything below -- climb, orphan walk, replace --
	 * is coherent with the peer's junction and every expected-old matches.
	 * Only the caller's leaf says the plan is about the wrong node.
	 */
#ifndef FT_DEBUG_NO_LEAF_IDENTITY
	if (freeze_leaf && caa_unlikely((struct cds_ft_node *)
			ft_node_ptr(ft_resolve_flip_proxy(plan_old_child)) !=
				freeze_leaf))
		return -EAGAIN;
#endif
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
		pair_held = NULL;
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
		resolved_parent = ft_parent_node_resolved(
				rcu_dereference(metadata->parent_word));
		is_root = (resolved_parent == NULL);
		boundary_parent_nf = resolved_parent;	/* always names @cur */
		/*
		 * THE PAIR THIS LEVEL WRITES THROUGH MUST BE COHERENT.
		 *
		 * @detach_parent_flag_ptr is the slot this plan may CLEAR (an
		 * elevation) or PUBLISH INTO (a DEL recompaction of @cur).  The
		 * CALLER derived it from @cur's (parent_word, parent_slot_offset)
		 * at an earlier instant; by the time this level consumes it, a
		 * peer may have REPUBLISHED @cur's parent (a recompaction copies
		 * the parent body and retires the original).  The caller's
		 * address then points into the RETIRED body while @cur's word
		 * names the fresh copy -- individually coherent, not a pair.
		 * ft_resolve_parent_slot cannot see it (its coherence re-read
		 * only catches a re-home landing on the parent WORD, and that
		 * word is stable across both loads), and the entry HOLDER
		 * IDENTITY test cannot either: it checks the slot this op READS
		 * (@detach_node_flag_ptr inside @cur), not the slot it WRITES.
		 *
		 * MEASURED on inv_concurrent_remove_all_nolist, per-node AND
		 * root-only: the parent word resolved to a 32-byte node while
		 * the offset read 6 (an order-6 body's), so the elevated clear
		 * NULLed a word of the NEIGHBOURING item -- which held a live
		 * external head -- and fused the nr_child-- onto the real parent.
		 * That parent then carries one non-NULL child its count does not
		 * cover: every plan built from the count is wrong, every
		 * validation against the structure fails, and the node WEDGES
		 * (the memcg-killed legs).
		 *
		 * ☞ That signature was FIRST read as a torn two-store re-parent,
		 * and that reading is REFUTED.  A parent republished between the
		 * caller's derivation and this use produces the identical
		 * order-5-word / offset-6 disagreement with no torn read at all,
		 * and four independent probes agree it is the republish:
		 * re-resolving the pair AT the refusal finds it self-coherent in
		 * 100% of why2 hits (28/28 per-node, 119/119 root-only), while
		 * the three writer-side probes -- an offset store onto a word
		 * naming a different parent, a parent-word-only re-parent, and a
		 * plain offset store onto a child that is still REACHABLE
		 * through its own pair -- all read ZERO on this row.  Nobody
		 * tears the pair; the plan simply goes stale.  So this block is
		 * a STALE-PLAN refusal, and no re-parent producer is owed a fix
		 * on its account.
		 *
		 * The witness that is independent of the torn pair is the parent
		 * @cur's OWN word names, resolved just above: the slot must lie
		 * INSIDE that parent's body (or be the trie root slot), and what
		 * it holds must be @cur.  Both are address / identity tests, no
		 * value this op adopts.  Nothing is built, locked or reserved:
		 * re-descend, and the retry reads the pair after the re-parent's
		 * second store has landed.
		 */
		{
			int pair_bad = 0;
			struct cds_ft_inode_flag *held = NULL, *held_res = NULL;

			if (is_root) {
				if (caa_unlikely(detach_parent_flag_ptr != &ft->root))
					pair_bad = 1;
			} else if (caa_unlikely(!ft_slot_in_node(resolved_parent,
					detach_parent_flag_ptr))) {
				pair_bad = 2;
#ifdef FT_DEBUG_DEL_TOMB
				/*
				 * DISCRIMINATOR for why2.  Two different faults
				 * produce the identical signature "the slot lies
				 * outside the parent the word names":
				 *
				 *  (a) a TORN PAIR -- @cur's own (parent_word,
				 *      parent_slot_offset) were read one after the
				 *      other across a two-store re-parent, so the
				 *      pair NEVER described one real edge; or
				 *  (b) a STALE PLAN -- the pair is perfectly
				 *      coherent, but a peer REPUBLISHED the parent
				 *      since the CALLER derived
				 *      @detach_parent_flag_ptr, so the caller's
				 *      address points into the retired body while
				 *      the word names the fresh copy.
				 *
				 * Re-resolving the pair NOW separates them: if the
				 * fresh slot lands inside the fresh parent, the pair
				 * is self-coherent and (b) is what happened.  The
				 * refusal is right either way -- this only says WHICH
				 * defect the -EAGAIN is absorbing, i.e. whether a
				 * producer is still owed a fix.
				 */
				{
					struct cds_ft_inode_flag *fresh_parent = NULL;
					struct cds_ft_inode_flag **fresh_slot =
						ft_resolve_parent_slot(metadata, ft,
							&fresh_parent);

					if (fresh_parent && ft_slot_in_node(
							ft_resolve_flip_proxy(fresh_parent),
							fresh_slot))
						FT_DT_INC(ft_dt_pair_fresh_ok);
					else
						FT_DT_INC(ft_dt_pair_fresh_bad);
				}
#endif
			} else {
				held = ft_resolve_flip_proxy(
					(struct cds_ft_inode_flag *)
					rcu_dereference(*detach_parent_flag_ptr));
				/*
				 * A compressed @cur is named by its parent through
				 * the SKIP_X form of ITS child (an external head's
				 * address with the skip bits set), so resolve the
				 * skip encoding BEFORE asking whether the value is a
				 * bare external: only a bare one is a foreign word.
				 */
				held_res = held ?
					ft_resolve_skip_compressed(ft, held) : NULL;
				if (caa_unlikely(!held_res ||
						ft_node_external(held_res)))
					pair_bad = 3;
				else if (caa_unlikely(ft_node_ptr(held_res) !=
						ft_node_ptr(cur)))
					pair_bad = 4;
				if (!pair_bad)
					pair_held = held;
			}
			if (caa_unlikely(pair_bad)) {
#ifdef FT_DEBUG_DEL_TOMB
				uatomic_inc(&ft_dt_pair_why[pair_bad]);
				if (uatomic_add_return(&ft_dt_pair_refused, 1) <= 8)
					fprintf(stderr, "FT DT-PAIR-REFUSE why=%d iter=%d cur=%p cur_meta=%p resolved_parent=%p is_root=%d slot=%p slot_val=%p held=%p held_res=%p entry_slot=%p arg_parent_slot=%p climbed=%d\n",
						pair_bad, nr_branch, (void *) cur,
						(void *) metadata, (void *) resolved_parent,
						(int) is_root, (void *) detach_parent_flag_ptr,
						(void *) CMM_LOAD_SHARED(*detach_parent_flag_ptr),
						(void *) held, (void *) held_res,
						(void *) entry_holder_slot,
						(void *) dbg_arg_parent_slot, (int) climbed);
#endif
				return -EAGAIN;	/* torn plan: re-descend */
			}
		}
		FT_DT_INC(ft_dt_pair_checked);
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
		/*
		 * ☠ AND A RETIRED ANCESTOR VOIDS THE WHOLE PLAN, at EVERY level
		 * of the climb -- the boundary included, since the stop test
		 * below runs on this same @cur.
		 *
		 * The climb reads every ancestor with NOTHING held, and from
		 * those reads it decides which slot is cleared, which chain is
		 * orphaned and where the prune stops.  A peer that RETIRED any
		 * node on this path has already republished a fresh copy
		 * elsewhere, so every one of those decisions names a node that
		 * is no longer in the trie -- and the plan then condemns, frees
		 * and re-parents around a dead path.  nr_child cannot see it: a
		 * retired node keeps its body readable (RCU) and its count
		 * intact; only the TOMBSTONE bit says the node has left.
		 *
		 * The recompacting delete never needed this either -- the
		 * {C,P,GP} acquire refuses PROXY|TOMBSTONE|LOCK outright, so a
		 * dead ancestor was rejected at the lock.  The in-place tier is
		 * what removes that acquire from the common path, which is why
		 * the check has to be stated here instead.
		 *
		 * Bail before anything is built, locked or reserved, exactly as
		 * the nr_child == 0 arm above.
		 */
		FT_DT_INC(ft_dt_climb_reach);
		if (caa_unlikely(ft_meta_tombstone(metadata))) {
			FT_DT_INC(ft_dt_climb_tomb);
			return -EAGAIN;
		}
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
			 *
			 * ☠ A MISS IS A STALE PLAN, NOT A CAN'T-HAPPEN.  The
			 * reverse lookup compares the slot's live value against
			 * @*detach_node_flag_ptr by RAW IDENTITY across two
			 * separate loads of the same word, and this climb runs
			 * PRE-FENCE: a peer commit on that slot lands between
			 * them and every slot mismatches, so the lookup returns
			 * false.  Its header calls that "should not happen on a
			 * well-formed trie" -- true single-writer, and why the
			 * result went unchecked.
			 *
			 * Left unchecked it is SILENT AND FATAL: @n keeps its
			 * initialiser 0, and the shape-D survivor scan below
			 * skips byte @n to exclude the child being detached.
			 * With @n == 0 it excludes nothing and takes the FIRST
			 * live child -- the detached branch itself.  The
			 * post-fence re-validation cannot catch it (it only asks
			 * that @surviving_byte still maps to @surviving_child,
			 * which is trivially true of the detached child), so the
			 * collapse fuses around the very head this commit
			 * freezes: the real sibling's key is ORPHANED and a
			 * REMOVED node stays reachable.  Readers never consult
			 * the removed mark, so cds_ft_lookup returns that
			 * tombstone as live; the next insert onto it records
			 * slot (void **) 2 from a marked `next`, and the MW
			 * install dereferences it.
			 *
			 * Do NOT retry the lookup in place: a retried find_child
			 * succeeds (measured 50,590 of 50,590 misses are
			 * transient) but then names the PEER'S NEW child, a
			 * subtree this climb never counted.  Abandoning the plan
			 * is the only correct response.
			 *
			 * MEASURED on a build that widens the miss-to-scan window
			 * (a 10,000-iteration cpu_relax on the miss path, nothing
			 * else), exponential spacing, two arms differing by this
			 * bail alone: 64 of 96 runs SEGV without it, 0 of 96
			 * with it.
			 */
			if (!ft_node_compressed(cur) &&
					caa_unlikely(!ft_node_find_child(ft, cur,
						*detach_node_flag_ptr, &n, NULL)))
				return -EAGAIN;	/* stale plan: re-descend */
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
		if (metadata->external_nodes && !topmost_external_nodes) {
			topmost_external_nodes = ft_dereference_external(
				metadata->external_nodes);
			topmost_src_meta = metadata;
			/*
			 * The junction the prune empties.  Its head takes its
			 * place in the boundary's slot, so the node itself is
			 * unlinked -- an orphan the walks below must budget for.
			 * ☠ It is NOT in @nr_clear (that tally is for keyless
			 * links), and a walk budgeted on @nr_clear alone stops one
			 * node short.  The plain REMOVE hid that: its target-chain
			 * walk picks up whatever the elevated budget left behind.
			 * A MOVE-style detach runs no such walk, and leaked the last
			 * link of every promote -- the junction under a compressed
			 * boundary, the run under the junction otherwise.
			 */
			climb_promoted = topmost_external_nodes != NULL;
		}
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
				 *
				 * ☠ A STALE PLAN, NOT A BOUNDARY.  The slot recovered
				 * from @parent_nf's metadata lives in ITS parent's body;
				 * the slot that holds @cur lives in @parent_nf's own.
				 * They alias only when the chain this climb read is not
				 * a tree any more -- a peer's relocation seen through
				 * reclaimed memory.  Breaking here used to keep the
				 * level's verdict (its @nr_clear tally, and now its
				 * promote) while NOT elevating past it, so both orphan
				 * walks would then reach one link too far: the detach
				 * target itself, freed while live.  Nothing is built,
				 * locked or reserved yet: re-descend, as every other
				 * stale-plan bail in this climb does.
				 */
				if (caa_unlikely(new_parent_flag_ptr ==
						detach_parent_flag_ptr))
					return -EAGAIN;
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
#ifndef FT_DEBUG_CLIMB_PLAN_REREAD
					pair_held;
#else
					(struct cds_ft_inode_flag *)
						rcu_dereference(*detach_node_flag_ptr);
#endif
				FT_DT_INC(ft_dt_cap1_reach);
				if (!plan_old_child)
					FT_DT_INC(ft_dt_cap1_null);
				/*
				 * The level just decided to prune is already
				 * empty: same stale plan as the entry capture,
				 * same answer (see there).
				 */
#ifndef FT_DEBUG_NO_NULLPLAN
				if (caa_unlikely(!plan_old_child))
					return -EAGAIN;	/* stale plan: re-descend */
#endif
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
	 * ☠ AN EMPTY HOLDER SLOT IS A STALE PLAN, NOT A NODE.  The climb
	 * validated @detach_parent_flag_ptr against its level's @cur (THE PAIR
	 * THIS LEVEL WRITES THROUGH, above), but that check and this READ are
	 * two separate instants: the loop exited, and a peer's in-place delete
	 * can clear the slot in between.  No pair check can close that -- the
	 * validation cannot be held across the read -- so the empty value has to
	 * be caught where it is actually observed, exactly as the plan
	 * expected-old below catches a republished child.
	 *
	 * Left through, NULL is not rejected by anything downstream: every
	 * classifier here asks the node's FLAGS, and ft_node_compressed(NULL) /
	 * ft_node_skip_compressed(NULL) are both false, so a NULL holder walks
	 * straight into the internal-node arm and reaches
	 * cds_ft_item_to_metadata(NULL), which indexes the metadata array of the
	 * range that would contain address 0 and FAULTS (MEASURED: release
	 * root-only, inv_concurrent_remove_all_list, SIGSEGV at 0x200010 -- the
	 * NULL page base plus the range's metadata offset -- with
	 * iter_node_flag = 0x0 in the frame).  This is the same failure the
	 * flip-proxy bail above prevents, reached by the other bad value.
	 *
	 * Nothing is built, locked or reserved yet: re-descend, exactly as the
	 * two guards either side of this one do.
	 */
	FT_DT_INC(ft_dt_holder_reach);
	if (caa_unlikely(!iter_node_flag)) {
		FT_DT_INC(ft_dt_holder_null_refused);
		return -EAGAIN;	/* holder gone: re-descend */
	}
	/*
	 * ☠ THE HOLDER IS THE NODE THE CLIMB ENDED AT, OR THE PLAN IS VOID.
	 * @detach_node_flag_ptr is a slot INSIDE the climb's boundary (its last
	 * @cur, metadata_stack[nr_branch - 1]), and every write below goes
	 * through it -- but @iter_node_flag is a FRESH read of the slot above
	 * it, and a peer that republished the boundary since (a compressed
	 * split's fresh junction, a recompaction's copy) hands back a node the
	 * slot does not live in.  Nothing downstream re-asks: the op locks
	 * THAT node, which is live, and its record lands on the RETIRED
	 * boundary's slot, whose body still holds the plan's value, so even an
	 * MW expected-old matches.  The live copy keeps naming the child this
	 * commit then retires.
	 *
	 * MEASURED (LTTng, inv_prefix_shape_zoo insert/remove_all): the climb
	 * condemned H, a junction whose only content left was its prefix head
	 * N, under the compressed boundary CN.  A peer split CN into a fresh
	 * junction J still carrying H, and committed; this op read J here,
	 * locked it, recorded CN->child: H -> N, and retired H.  J still named
	 * H: a tombstoned node reachable from a live one, and every remove of
	 * N's key then spun on it (50,000 consecutive retries).
	 *
	 * The leaf hoist below states this same test for its own arm; the
	 * promote arm had none.  With identity established here, the holder's
	 * acquire (which refuses a tombstone) is what keeps it: under that lock
	 * the boundary is live, so its slot is too.  Nothing is built, locked
	 * or reserved yet: re-descend.
	 *
	 * An EXTERNAL word is the same stale plan -- a peer collapsed the
	 * boundary into the bare leaf it carried -- and has no metadata to
	 * compare: refuse it first (MEASURED: 2 of 12 runs SEGV'd resolving
	 * one).  SKIP before EXTERNAL: a SKIP_X word carries its child's tag.
	 */
#ifndef FT_DEBUG_NO_HOLDER_IDENTITY
	if (caa_unlikely((!ft_node_skip_compressed(iter_node_flag) &&
				ft_node_external(iter_node_flag)) ||
			ft_flag_to_metadata(ft, iter_node_flag) !=
				metadata_stack[nr_branch - 1]))
		return -EAGAIN;	/* boundary republished: re-descend */
#endif
	/*
	 * PLAN EXPECTED-OLD, enforced (see @plan_old_child): everything below --
	 * the orphan set walked from @elevated_old_child, the count fold, and the
	 * drop itself -- names the subtree the climb condemned.  A peer that
	 * republished this slot since has put a DIFFERENT subtree here, one no
	 * level of the climb ever counted, so the whole plan is void.  Nothing is
	 * built, locked or reserved yet: re-descend against the settled tree.
	 */
	FT_DT_INC(ft_dt_guard_reach);
	if (!elevated_old_child && !plan_old_child)
		FT_DT_INC(ft_dt_guard_both_null);
	if (elevated_old_child != plan_old_child) {
		if (!elevated_old_child)
			FT_DT_INC(ft_dt_guard_elev_null_refused);
		if (!plan_old_child)
			FT_DT_INC(ft_dt_guard_plan_null_refused);
	}
	if (caa_unlikely(elevated_old_child != plan_old_child))
		return -EAGAIN;
	/*
	 * The elevated chain from @elevated_old_child down to (excluding) the
	 * original target: the keyless links the climb counted, plus the keyed
	 * junction it promoted past -- which always sits at the chain's TOP, the
	 * climb stopping at the very next level (prev_external_nodes_found).
	 * Both orphan walks below are budgeted on this, so a promote's junction
	 * is retired with the rest whether the target's subtree is destroyed
	 * (remove) or moved (rekey).
	 */
	nr_elevated = nr_clear + (climb_promoted ? 1 : 0);
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
		 * FOLD: ride the CALLER's commit (see the txn block below); it is
		 * also what decides who FREES the orphan set, at the bottom.
		 */
		bool fold_replace = record_only && shared_txn &&
				!fuse_cell && !run;

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
		 *
		 * A MOVE-style detach (@free_detached_subtree false: the target's
		 * subtree is re-attached elsewhere, not destroyed) still owns the
		 * ELEVATED chain above the target -- the keyed junction whose head
		 * the replace below promotes into cn->child, and any keyless link
		 * under it.  Walk exactly @nr_elevated links for it and stop short
		 * of the target; the plain remove walks on to the leaf.  A SKIP
		 * word stops the move's walk (the body reads it as its child): the
		 * elided run is the chain's last link, claimed by the trailing arm
		 * below while the budget is unspent.
		 */
		if (free_detached_subtree || nr_elevated > 0) {
			struct cds_ft_inode_flag *walk_nf = elevated_old_child;
			unsigned int walk_depth = ft_child_depth_of(ft,
				iter_node_flag, cur_depth);
			bool phase2_first = true;
			/*
			 * ☞ ITS OWN DESCENT, EXTENDED PER ORPHAN, and the depth
			 * ADVANCED with it (doc/design/ft-lockset-inventory.md §2
			 * row 6).  This walk used to date every orphan at the
			 * FIRST one's depth and anchor it from the op's descent,
			 * whose table ends above the chain -- so an orphan past
			 * the first anchored on ITSELF by ft_anchor_meta's
			 * on-a-level early return while every other op anchored
			 * it on an ancestor (88k of 200k acquires at exponential).
			 * The phase-2 walk below already extends a copy of its
			 * own; this one does the same, for the same reason: the
			 * op's descent stays on the KEY path for the acquires that
			 * resolve after it.
			 */
			struct ft_descent fwd;
			struct ft_lock_ctx flctx = lctx;

			if (wd_valid)
				fwd = wd;
			flctx.d = wd_valid ? &fwd : NULL;

			while (walk_nf &&
			       !ft_node_external(walk_nf) &&
			       nr_to_free < FT_MAX_DEPTH &&
			       (free_detached_subtree ||
				(nr_to_free < nr_elevated &&
				 !ft_node_skip_compressed(walk_nf)))) {
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
					flctx.held.txn = lctx.held.txn;
					flctx.held.nr_extra = lctx.held.nr_extra;
					/*
					 * The copy anchors the walk only because the
					 * op's descent already ENTERED the node the walk
					 * starts under; a cursor still ON it would leave
					 * its levels uncrossed (skeptic, 2026-09-17).
					 *
					 * ☠ THAT IS A PLAN, NOT AN INVARIANT.  @walk_depth
					 * comes from the climb (key_len minus the spans it
					 * reads LIVE), the anchors from the descent, and a
					 * peer split or merge between the two lets the climb
					 * date the walk deeper than the descent ever went.
					 * ft_anchor_meta then answers from the wrong path:
					 * the assert fired under rcu-debug, and a release
					 * build anchored a garbage item and faulted in
					 * ft_lock_set_order_by_anchor (two-writer owned-key
					 * harness, exponential spacing).  Re-descend, as the
					 * acquire's own refusal below does.
					 */
#ifndef FT_DEBUG_ORPHAN_WALK_UNDATED
					if (caa_unlikely(wd_valid &&
							fwd.depth < walk_depth)) {
						ret = -EAGAIN;
						goto end;
					}
#endif
					assert(!wd_valid || fwd.depth >= walk_depth);
					if (ft_detach_orphan_acquire(ft, &flctx,
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

				/*
				 * ☠☠ A `break` HERE IS NOT A REFUSAL.  The walk
				 * stops, but everything after it runs: the replace
				 * unlinks this whole chain from its parent while
				 * @to_free holds only [0, k), so orphan @k is
				 * unlinked and NEVER FREED -- leaked, together with
				 * whatever a peer put on it.  Single-writer the walk
				 * always reaches the leaf (every chain node passed
				 * the climb's 1-child/no-ext tests), so this arm
				 * fires ONLY on a plan the peer has invalidated --
				 * and a stale plan must ABORT the op, not truncate
				 * its free list.
				 *
				 * Nothing is built, recorded or reserved yet (the
				 * walk is read-only and @orphan_txn does not exist),
				 * so `goto end` is byte-for-byte clean and the sweep
				 * there releases every mark this op still holds.
				 * The mismatch is a COMMITTED peer write sitting in
				 * our own locked snapshot, so the re-descend sees a
				 * settled tree: one retry per peer event, never a
				 * conflict the op manufactures for itself.
				 *
				 *
				 * ★ AND THE EXEMPTION IS NOW KEYED ON IDENTITY, not
				 * position -- but NOT on the identity the first
				 * design assumed.  MEASURED (ft_unit, single-writer,
				 * probe at this line): the first orphan's own head is
				 * @ext_nodes == NULL while @topmost_external_nodes is
				 * NON-NULL, in every one of 24 samples, all in branch
				 * 2.  @topmost was promoted from an ancestor ABOVE
				 * this node (@prev_external_nodes_found is already
				 * set), so it is simply not THIS node's head.
				 * Requiring @ext_nodes == @topmost therefore refuses
				 * the NORMAL shape and wedges ft_unit entering
				 * test_density_stress.
				 *
				 * What the climb actually believes about the first
				 * orphan is weaker: it carries NO head of its own, OR
				 * the one head the climb lifted off it.  Anything
				 * else is a head that arrived after the plan -- a
				 * peer's park -- and freeing this node would take the
				 * key with it.
				 */
				/*
				 * SKEPTIC'S DISCRIMINATOR (probe only, no behaviour
				 * change): the FIRST orphan is exempted from the
				 * nr_child > 1 test, and its IDENTITY is never
				 * compared with the node the climb condemned
				 * (@elevated_old_child is RE-READ after the climb).
				 * Count both, on the oracle that loses keys.
				 */
				if (phase2_first) {
					FT_DT_INC(ft_dt_walk_reach);
					if (nr_child > 1)
						FT_DT_INC(ft_dt_walk_first_multi);
					if (nr_branch >= 2 && ometa !=
							metadata_stack[nr_branch - 2])
						FT_DT_INC(ft_dt_walk_first_ident);
				}
				/*
				 * ☠ AND A HEAD THAT DEPARTED.  The rule above
				 * accepts a first orphan with NO head, because the
				 * promoted head usually came from an ancestor above
				 * it -- but when it came off THIS node, "no head now"
				 * means a peer REMOVED it after the climb read it,
				 * pre-lock.  Promoting it anyway republishes a
				 * removed node into the boundary slot (MEASURED: the
				 * remove of "...zone46" resurrected a concurrently
				 * removed "...zone4", next == REMOVED, both writers
				 * then livelocked on it).  Under this orphan's lock
				 * the node it was lifted off must still carry it.
				 */
				if ((!phase2_first && (nr_child > 1 || ext_nodes)) ||
				    (phase2_first && ext_nodes &&
					    ext_nodes != topmost_external_nodes)
#ifndef FT_DEBUG_NO_DEPARTED_HEAD
				    || (ometa == topmost_src_meta &&
					    ext_nodes != topmost_external_nodes)
#endif
#ifndef FT_DEBUG_NO_FIRST_ORPHAN_COUNT
				    /*
				     * ☠ THE FIRST ORPHAN IS THE TARGET ONLY WHEN
				     * NOTHING WAS ELEVATED.  After a climb it is
				     * the chain's top -- a link the climb scored
				     * single-child from an UNHELD read -- and an
				     * in-place insert (FEATURE_FT_INSERT_IN_PLACE)
				     * grows it under this very lock without
				     * retiring it: the walk then stops (no scan
				     * past one child) and the drop retires the
				     * link with the peer's new child inside
				     * (MEASURED: inv_prefix_siblings_compressed_
				     * holder, in-place legs, "ord-cell list longer
				     * than trie").  Branch 2's phase 1 enforces
				     * the same count under its planlock.
				     */
				    || (phase2_first && nr_elevated > 0 &&
					    nr_child > 1)
#endif
				    ) {
					if (ft->lock_fine && !owalk.shared)
						ft_meta_lock_release(owalk.lock);
#ifdef FT_ENABLE_TRACING
					uatomic_inc(&ft_dbg_orphan_walk_stale);
#endif
					ret = -EAGAIN;
					goto end;
				}
				phase2_first = false;
				to_free[nr_to_free++] = walk_nf;
				if (ft->lock_fine)
					orphan_held[nr_orphan_locked++] = owalk;
				walk_depth = ft_walk_extend(&fwd, wd_valid, walk_nf,
					walk_depth, ocn ? ocn->len : 1);
				walk_nf = next;
			}
			/*
			 * As in the replace-ptr free-walk below: a skip-compressed
			 * external leaf at the chain end keeps its path bytes in a
			 * separate, now-orphaned skip-target compressed node that the
			 * walk stops short of.  Free it (the external leaf stays
			 * caller-owned).  For a MOVE the skip-target is an orphan
			 * only while the elevated budget is unspent: at the budget
			 * the skip word encodes the target's own run, which moves
			 * with it.
			 */
			if (walk_nf && ft_node_skip_compressed(walk_nf) &&
					(free_detached_subtree ||
					 nr_to_free < nr_elevated)) {
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
					flctx.held.txn = lctx.held.txn;
					flctx.held.nr_extra = lctx.held.nr_extra;
#ifndef FT_DEBUG_ORPHAN_WALK_UNDATED
					/* A walk the descent never dated: see above. */
					if (caa_unlikely(wd_valid &&
							fwd.depth < walk_depth)) {
						ret = -EAGAIN;
						goto end;
					}
#endif
					assert(!wd_valid || fwd.depth >= walk_depth);
					if (ft_detach_orphan_acquire(ft, &flctx,
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
			/*
			 * FOLD: ride the CALLER's commit.  Its own txn commits
			 * strictly earlier, and this arm publishes a SKIP_X dual
			 * whose home the publish resolves against the descriptor
			 * it is handed -- a private one cannot see the caller's
			 * pending re-parent of the compressed node, so the dual
			 * was written into the parent the caller's commit then
			 * retired.  Refused when the caller brought a cell or a
			 * run: ft_remove_commit_rec's record-only arm asserts
			 * !run && !dead_cell.
			 */
			struct ft_flip_txn *orphan_txn = fold_replace ? shared_txn :
				ft_flip_txn_create_bounded(ft,
					FT_REMOVE_COMMIT_REC_MAX_EDGES
					+ 1 /* §4.B parent guard (Sites 3+4 excl.) */
					+ ft_freeze_reserve(ft, (unsigned int) nr_to_free
						+ (trailing_skip_cn ? 1 : 0))
					+ (topmost_external_nodes ? 1 : 0) /* folded external back-edge */
					+ (ft->rank_stats ? detach_depth + 1 : 0) /* nr_keys fold walk */
					+ freeze_len * FT_HLIST_FREEZE_MAX_EDGES);

			if (!orphan_txn) {
				ret = -ENOMEM;
				goto end;
			}
			/*
			 * PUBLISH THE WALK'S MARKS BEFORE THE FREEZE READS THEM.
			 * An orphan whose anchor is its OWN word is never registered
			 * in the txn (its fused tombstone is the terminal, see
			 * ft_detach_freeze_one), so the record's ownership witness
			 * is the held set -- and the walk above left @nr_extra one
			 * short of the last acquire.  Under a per-op-armed CALLER's
			 * txn (the fold) an unwitnessed tombstone is an abort at the
			 * record; the plain-parent branch publishes before its
			 * freeze for the same reason.
			 */
			lctx.held.txn = orphan_txn;
			lctx.held.nr_extra = (unsigned int) nr_orphan_locked;
			for (fi = 0; fi < nr_to_free; fi++) {
				struct cds_ft_metadata *m = cds_ft_item_to_metadata(
					ft_node_compressed(to_free[fi])
						? (struct cds_ft_inode *)
						  ft_compressed_node_ptr(
							ft_skip_child_ptr(to_free[fi]))
						: ft_node_ptr(to_free[fi]));
				if (ft->lock_fine)
					ft_detach_freeze_one(orphan_txn, &lctx,
						&orphan_held[fi], m, fold_replace);
				else
					ft_flip_txn_record_tombstone(orphan_txn, m);
			}
			if (trailing_skip_cn) {
				struct cds_ft_metadata *m = cds_ft_item_to_metadata(
					(struct cds_ft_inode *) trailing_skip_cn);
				if (ft->lock_fine)
					ft_detach_freeze_one(orphan_txn, &lctx,
						orphan_trailing_held, m, fold_replace);
				else
					ft_flip_txn_record_tombstone(orphan_txn, m);
			}
			/*
			 * The removed external leaf freezes atomically with this
			 * compressed-parent replace that unlinks its chain (doc §4.B):
			 * one MARK edge into @orphan_txn, the +1 reserved above.  The
			 * replace records it, after the acquire that holds its chain.
			 */
			if (freeze_leaf)
				freeze_leaf_fused = true;
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
				fuse_cell,
				pub, run, orphan_txn, fold_replace,
				count_delta, freeze_leaf, freeze_len);
			if (ret) {
				/*
				 * -EAGAIN: the commit CONSUMED @orphan_txn (a
				 * peer won mid-flip); only a pre-commit failure
				 * (-ENOMEM before the commit) leaves it live.
				 */
				if (ret != -EAGAIN && !fold_replace)
					ft_flip_txn_destroy(orphan_txn);
				goto end;
			}
		}
		/*
		 * Orphan chain unlinked: free the set collected above -- unless
		 * this detach committed NOTHING (the FOLD), in which case the chain
		 * is still live and linked and the CALLER's commit is the unlink.
		 * Hand it out with the retired copy's lifetime then, exactly as the
		 * plain-parent branch does: the caller reclaims it after its commit
		 * lands and leaves it alone when that commit aborts.  The trailing
		 * skip-target goes out as its PLAIN compressed flag, decoded above
		 * under the plan -- a post-commit one-hop decode of the skip word
		 * would run through a back-pointer this very commit re-points (the
		 * promoted head's).
		 */
		if (fold_replace) {
			if (recompact_out) {
				for (fi = 0; fi < nr_to_free; fi++)
					recompact_out->orphans[fi] = to_free[fi];
				recompact_out->nr_orphans = nr_to_free;
				if (trailing_skip_cn &&
						recompact_out->nr_orphans <
							FT_MAX_DEPTH)
					recompact_out->orphans[
						recompact_out->nr_orphans++] =
						ft_compressed_node_flag(
							trailing_skip_cn);
			}
		} else {
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
		 *   Phase 1 -- elevated ancestors (single-child by the upward
		 *   walk's own invariant: keyless, or the ONE keyed junction the
		 *   climb promoted past, at the chain's top).  Free @nr_elevated
		 *   nodes unconditionally.
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

			/*
			 * Phase 1: elevated ancestors.
			 *
			 * ☠ SKIP BEFORE EXTERNAL, and the body two lines down has
			 * always said so.  ft_node_external tests the LOW TAG BITS
			 * ONLY, and a SKIP_X word carries its child's tag -- so a
			 * skip pointer ONTO AN EXTERNAL HEAD reads as external and
			 * this loop never starts.  The body's own first arm is the
			 * skip arm; the condition disagreed with it.
			 *
			 * What that costs is the skip-TARGET run: the node holding
			 * the path bytes between the boundary and a bare external
			 * head.  A move-style detach (@free_detached_subtree false,
			 * which is what the rekey passes) has no other collector --
			 * the trailing skip-target arm is nested inside
			 * `if (free_detached_subtree)` -- and ft_rekey_cow_stop owns
			 * nothing for a bare head (@s_top_meta is NULL).  So nobody
			 * frees it.
			 *
			 * MEASURED with the drain oracle over the 3000-shape corpus
			 * on a -DDEBUG_COUNTERS build: 310 of 310 seeds whose src is
			 * a bare head under a SKIP-encoded boundary slot leak exactly
			 * one compressed node, and 0 of the 197 bare-head seeds whose
			 * boundary slot is a plain internal do.  100 per cent
			 * penetrant, and invisible to every other oracle -- keys
			 * right, cds_ft_verify clean on all of them.  A point REMOVE
			 * of the same key is fine: it passes @free_detached_subtree
			 * true and the trailing arm catches it.
			 *
			 * ☞ This is the dispatch-order trap ft-rekey.h's own descent
			 * comment names ("a skip word's low tag bits are 0, so the
			 * external and internal predicates both read it wrong").
			 */
			while (nr_to_free < nr_elevated &&
			       walk_nf &&
			       (ft_node_skip_compressed(walk_nf) ||
				!ft_node_external(walk_nf)) &&
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
					/*
					 * ☠ THE ONE-HOP RESOLVE IS NOT MW-SAFE, and
					 * "structurally single-child" was an ASSUMPTION about
					 * the node this plan MEANS, never a fact about the
					 * node it just GOT.  ft_skip_to_compressed follows the
					 * skip child's parent back-pointer, and its own header
					 * says callers "get whatever the back-pointer
					 * currently says": a concurrent split re-parents that
					 * child onto the junction it is publishing, so this
					 * recovers the PEER'S FRESH JUNCTION instead of the
					 * elided skip-target.
					 *
					 * Queueing it puts a LIVE node into the orphan set,
					 * which TOMBSTONES it -- killing every key beneath it
					 * -- and then frees it as a compressed node.  MEASURED
					 * on inv_sibling_split_compress_unpinned: the collect
					 * lands 1.7 us AFTER the peer's publish_to_parent,
					 * the freeze and the free follow, the parent slot is
					 * never republished, and the key is gone for good.
					 * ☠ Guarding the FREE does not help -- two predicates
					 * were measured and both left the oracle red, because
					 * the TOMBSTONE has already killed the key.
					 *
					 * A compressed node holds EXACTLY ONE child, so a
					 * resolved target whose count says otherwise is not
					 * the node this plan is about.  This is the trailing
					 * skip-target's own rule (the guard above, added on
					 * the same measured key loss) applied at the one site
					 * that resolves the same way and never checked it.
					 * Bail to the op's re-descend.
					 */
					if (ft_meta_nr_child_load(
							cds_ft_item_to_metadata(
							(struct cds_ft_inode *) cn)) != 1) {
						ret = -EAGAIN;
						goto end;
					}
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
					ometa = cds_ft_item_to_metadata(
						(struct cds_ft_inode *)
						ft_compressed_node_ptr(walk_nf));
					require_sc = false;	/* compressed: single-child */
				} else {
					ometa = cds_ft_item_to_metadata(
						ft_node_ptr(walk_nf));
					require_sc = true;	/* elevated internal: nr_child==1 */
				}
#ifdef FT_DEBUG_ORPHAN_NEXT_BEFORE_LOCK
				next = ft_detach_walk_next(ft, walk_nf);
#endif
				wlctx.held.txn = lctx.held.txn;
				wlctx.held.nr_extra = (unsigned int) nr_orphan_locked;
				if (ft->lock_fine && ft_detach_orphan_planlock(ft,
						&wlctx, walk_nf, walk_depth, ometa,
						require_sc, orphan_held,
						&nr_orphan_locked)) {
					ret = -EAGAIN;
					goto end;
				}
				/*
				 * ☠ THE NEXT LINK IS READ UNDER THE MARK, NOT
				 * BEFORE IT.  A compressed link's count never moves
				 * -- it holds exactly one child -- so the plan-lock
				 * above validates nothing about WHICH child: a peer
				 * insert of a longer key republishes @cn->child in
				 * place, under this very word, from the head this op
				 * is removing to a fresh junction carrying that head
				 * AND the peer's key.  Read before the mark, the walk
				 * followed the old head, found it external, and
				 * stopped with the junction never looked at; the drop
				 * then took the branch whole (MEASURED: the peer's key
				 * lost within seconds, two writers, "...runs" vs
				 * "...runs6" under one compressed node).  Read after
				 * it, the walk reaches the junction and phase 2
				 * refuses its head.  Phase 2 and the compressed-parent
				 * walk already take the mark first.
				 */
#ifndef FT_DEBUG_ORPHAN_NEXT_BEFORE_LOCK
				next = ft_detach_walk_next(ft, walk_nf);
#endif
				/*
				 * A HEAD ON A CHAIN LINK THAT ARRIVED AFTER THE PLAN.
				 * The keyless links carry none, and the chain's top may
				 * carry exactly the one the climb lifted off it (the
				 * promoted junction).  Anything else is a peer's park,
				 * and retiring the link would take that key with it --
				 * the refusal phase 2 makes on its first orphan, read
				 * here under the mark taken just above (the plan-lock
				 * validates the child count alone).
				 */
				if ((ometa->external_nodes &&
						(nr_to_free != 0 ||
						 ometa->external_nodes !=
							topmost_external_nodes))
#ifndef FT_DEBUG_NO_DEPARTED_HEAD
				    /*
				     * ...and a head that DEPARTED from the link
				     * the climb lifted it off: the compressed
				     * arm's rule, which this walk lacked (the
				     * same resurrection, reached through a plain
				     * internal boundary).
				     */
				    || (ometa == topmost_src_meta &&
					ometa->external_nodes !=
						topmost_external_nodes)
#endif
				    ) {
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

				/*
				 * ☠ PHASE 2 STARTS AT THE TARGET, OR NOT AT ALL.
				 * Its first node is exempt from the nr_child > 1
				 * test below -- it is the target, whose count the
				 * detach itself is changing -- but nothing asked
				 * whether it IS the target.  Phase 1 follows each
				 * link's child as read UNDER that link's lock, so it
				 * walks the tree as it is NOW, not as the climb
				 * counted it: a peer insert that split a compressed
				 * link of the chain (a fresh run over a fresh
				 * junction carrying the old continuation AND the
				 * peer's key) is walked through, and the fresh
				 * junction lands here -- exempt -- and is retired
				 * with both keys inside.  MEASURED (LTTng + an
				 * orphan-count abort, inv_prefix_shape_zoo
				 * insert/remove_all): 6 of 6 runs, orphan[3] of 4,
				 * nr_child 2, cds_ft_verify then "ord-cell list
				 * longer than trie" with a never-removed key gone.
				 *
				 * Every destroy-style caller is a remove whose
				 * target is the chain head the entry pinned
				 * (@freeze_leaf): an EXTERNAL node, where this loop
				 * does not run.  An internal node here that is not
				 * the entry's target means the chain moved under the
				 * walk: re-descend.  Nothing of this walk's is
				 * recorded yet; its held links release at @end.
				 */
#ifndef FT_DEBUG_NO_PHASE2_TARGET
				if (walk_nf && !ft_node_external(walk_nf) &&
						walk_nf != entry_target) {
					ret = -EAGAIN;
					goto end;
				}
#endif
#ifndef FT_DEBUG_NO_ARRIVAL_EXTERNAL
				/*
				 * ☠ AND AN EXTERNAL ARRIVAL IS NOT EXEMPT: it has to
				 * be the target too.  The exemption above assumed a
				 * walk that reaches an external has reached THE chain
				 * head, because every link it took was a single
				 * child.  The in-place delete tier breaks that: a
				 * peer's in-place delete leaves a link LIVE with one
				 * child fewer, so a link this climb counted as
				 * "single child = our path" is single again with a
				 * DIFFERENT child -- a sibling key's -- and phase 1,
				 * taking the first child under the lock, walks into
				 * it.  MEASURED (delete tier, skip-compress off,
				 * inv_concurrent_remove_all_nolist, LTTng): lane A
				 * deleted key 62 in place from a two-child holder;
				 * lane B, planned with key 62 there, climbed through
				 * the holder, walked to key 63's head, retired it
				 * (LOST) and handed key 62's chain back a second time
				 * (DOUBLE-OWNED).  With skip-compression on, the
				 * in-place delete that would leave such a holder is
				 * re-planned instead (the shape-D hoist below), which
				 * is why only a skip-off build saw it.  Identity by
				 * address: a parked proxy retries too.
				 */
				if (walk_nf && ft_node_external(walk_nf) &&
						ft_node_ptr_raw(walk_nf) !=
						ft_node_ptr_raw(entry_target)) {
					ret = -EAGAIN;
					goto end;
				}
#endif

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

					/* Branch 2's twin of the walk above: same
					 * belief, same identity key, same refusal.
					 */
					if ((!phase2_first &&
					     (nr_child > 1 || ext_nodes)) ||
					    (phase2_first && ext_nodes &&
						ext_nodes != topmost_external_nodes)
#ifndef FT_DEBUG_NO_DEPARTED_HEAD
					    || (ometa == topmost_src_meta &&
						ext_nodes != topmost_external_nodes)
#endif
					    ) {
						if (ft->lock_fine && !owalk.shared)
							ft_meta_lock_release(
								owalk.lock);
#ifdef FT_ENABLE_TRACING
						uatomic_inc(&ft_dbg_orphan_walk_stale);
#endif
						ret = -EAGAIN;
						goto end;
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
			shape_d_plan = ft_group_skip_compressed(ft->group) &&
			    !topmost_external_nodes &&
			    ft_meta_nr_child_load(bmeta) == 2 &&
			    !bmeta->external_nodes &&
			    ft_parent_node(bmeta->parent_word) != NULL;
			if (shape_d_plan) {
				struct cds_ft_inode_flag *s_child = NULL;
				struct cds_ft_inode_flag **s_slot = NULL;
				struct cds_ft_inode_flag *fold_pending = NULL;
				uint8_t s_byte = 0;
				unsigned int b;

				for (b = 0; b < 256; b++) {
					struct cds_ft_inode_flag *c;

					if ((uint8_t) b == n)
						continue;
					c = ft_node_get_nth(ft, iter_node_flag,
						&s_slot, (uint8_t) b, FT_PF_NONE);
					if (c) {
						s_child = c;
						s_byte = (uint8_t) b;
						break;
					}
				}
				/*
				 * ☠ THE SURVIVOR SLOT MAY BE THIS TXN'S OWN
				 * PENDING FORWARD PUBLISH (@pending_pub_slot,
				 * armed before the fold ran -- by
				 * ft_glue_set_publish on the GLUE and merge
				 * lanes, or directly by the NOSPLIT store
				 * commit, ft-graft.h): a same-trie SIBLING
				 * move's shared parent
				 * holds exactly the detached child and the
				 * destination.  @s_child is then the COMMITTED
				 * occupant -- a node the SAME commit retires and
				 * replaces with the merged cluster -- and fusing
				 * around it strands the whole moved subtree
				 * (measured: the sibling two-child-BP pin, all
				 * four faces).  Build around the PENDING value
				 * instead (@pending_child at the collapse), the
				 * exact substitution ft_node_recompact's copy
				 * loops make by identity on this same slot.
				 */
				if (s_child && record_only && shared_txn &&
						shared_txn->pending_pub_slot &&
						s_slot == shared_txn->pending_pub_slot) {
					fold_pending = shared_txn->pending_pub_val;
					/*
					 * A COMPRESSED cluster top cannot be
					 * fused: absorbing it would retire a
					 * fresh node that was never published
					 * and re-aim its cluster's deferred
					 * edges (an ownership question the
					 * collapse does not take on), while
					 * falling back to the recompaction
					 * would publish a 1-child internal that
					 * skip mode's canonical form -- enforced
					 * at every writer-scope exit -- does not
					 * allow to persist.  So the SHAPE is
					 * uncovered: refuse the whole move,
					 * before any side-effect, with the
					 * rekey's own carve-out code
					 * (FT_REKEY_UNCOVERED == -EDOM).  Only
					 * the rekey fold can reach this: BOTH
					 * armers are gated on @record_only, and
					 * ft-rekey.h is its only setter.
					 */
					/*
					 * ☑ ABSORBED, not refused.  ☞ the
					 * @pending_child contract at
					 * ft_chain_compress_fused: a compressed
					 * pending top joins the merged run rather
					 * than sitting under it, which is the only
					 * legal product -- fusing AROUND it makes
					 * two adjacent compressed nodes and the
					 * replace_ptr fallback publishes a
					 * one-child internal.  An over-long run is
					 * still refused, by the merge's own
					 * FT_SKIP_LEN_MAX arm.
					 *
					 * ☠ ...BUT ONLY WHERE THE RUN HAS AN OWNER
					 * TO LEAVE.  The absorption untracks the
					 * pending top from the glue that BUILT it
					 * and frees it unpublished; with no glue in
					 * this frame's chain there is nobody to
					 * untrack it from, and freeing it anyway
					 * would leave it in some other owner's
					 * hands.  The glue is what armed
					 * @pending_pub_slot (ft_glue_set_publish),
					 * so it is always present for this shape --
					 * and where it is not, the old refusal is
					 * still the only answer available.
					 */
					if ((ft_node_compressed(fold_pending) ||
							ft_node_skip_compressed(
								fold_pending)) &&
							!ft_glue_of_ctx(&lctx)) {
						ret = -EDOM;
						goto end;
					}
				} else if (s_child && record_only && shared_txn &&
						ft_chain_compress_deep_pending(
							s_child, shared_txn)) {
					/*
					 * The SAME collision ONE COMPRESSED LEVEL
					 * DEEPER: the survivor is a compressed node
					 * whose own child slot is this txn's pending
					 * publish target, so the merged node's child
					 * -- normally taken from that very slot --
					 * must come from the pending value instead.
					 * The collapse does that substitution itself
					 * (it is the only frame that has @child_cn);
					 * what MUST happen here, before any
					 * side-effect, is the same refusal the
					 * shallow arm makes.
					 *
					 * ☠ A COMPRESSED pending value cannot be the
					 * merged node's child: the merged node IS
					 * compressed, and two adjacent compressed
					 * nodes are not a canonical form (cds_ft_verify
					 * rejects it).  The committed occupant this
					 * substitutes for can never be compressed for
					 * exactly that reason, so the substitution
					 * would be the only way to produce the shape.
					 * Refuse the whole move terminally with the
					 * rekey's carve-out code, as the shallow arm
					 * does -- never publish it.
					 */
					/*
					 * ☠ THE ARM IS LIVE; ONLY THIS SUB-CASE IS
					 * UNOBSERVED.  A counter at the arm fires
					 * 1 per 3000 shapes (with a PLAIN pending
					 * value); a counter at the COMPRESSED
					 * sub-case counts zero over 6000 runs, which
					 * is evidence for the canonical-form reading
					 * -- reaching it needs the pending slot to be
					 * a RUN'S OWN CHILD SLOT, i.e. the dst
					 * descent ending exactly where a run ends,
					 * and the graft's product there is an
					 * INTERNAL node (a displaced head, or a
					 * recompacted branch), never a run.
					 *
					 * ☞ SO IT STAYS A GUARD, not an assert.  An
					 * assert one branch off a LIVE path buys
					 * nothing on the build that ships: falling
					 * through lands on
					 * `new_cn->child = pending_pub_val`, two
					 * adjacent compressed nodes, which is the
					 * form the absorption exists to avoid.  The
					 * shallow arm absorbs its pending top; this
					 * one cannot, because the value is the
					 * merged node's CHILD rather than a run to
					 * concatenate.
					 */
					if (ft_node_compressed(
							shared_txn->pending_pub_val) ||
							ft_node_skip_compressed(
							shared_txn->pending_pub_val)) {
						ret = -EDOM;
						goto end;
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
					/*
					 * INTENT: the boundary carries no external
					 * head (the gate above asserts it), and this
					 * commit detaches exactly the child at @n --
					 * the one the climb is pruning.  @n is the
					 * boundary's own byte for it: the survivor scan
					 * just above skips @n while scanning
					 * @iter_node_flag, which is what makes them the
					 * same node.  Naming the target is the one
					 * thing the callee cannot derive for itself.
					 *
					 * ☠ LIKE FOR LIKE.  Name it the way the callee
					 * will READ it -- through ft_node_get_nth on
					 * the boundary, the accessor the survivor scan
					 * uses -- not by dereferencing the climb's slot
					 * pointer.  A raw slot value and a get_nth
					 * result are not the same encoding, so one
					 * never compares equal to the other: measured
					 * as a PERMANENT -EAGAIN, ft_unit wedged at
					 * test 2 and the memcg SIGKILLing the leak.
					 */
					/*
					 * INTENT: the boundary carries no external
					 * head (the gate above asserts it), and this
					 * commit detaches the pair the climb
					 * condemned -- @elevated_old_child at byte
					 * @n, both fixed before the boundary is
					 * acquired, and already enforced against the
					 * plan PRE-fence (@plan_old_child).  Carrying
					 * them here is that predicate moved past the
					 * acquire, where it can speak for the commit.
					 */
					const struct ft_chain_compress_intent intent = {
						.detach_child = elevated_old_child,
						.expect_ext = NULL,
						.detach_byte = n,
					};
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
						freeze_leaf, freeze_len,
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
				fold_pending, &intent,
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
					+ FT_CELL_LOCKSET_MAX_RECORDS
					+ 1 /* §4.B parent guard (Site 1 arms excl.) */
					+ ft_freeze_reserve(ft, (unsigned int) nr_to_free
						+ (trailing_skip_cn_flag ? 1 : 0))
					+ (retire_glue ? retire_glue->cap_free : 0)
					+ (ft->rank_stats ? detach_depth + 1 : 0) /* nr_keys fold walk */
					+ freeze_len * FT_HLIST_FREEZE_MAX_EDGES);
				if (!commit_txn) {
					ret = -ENOMEM;
					goto end;
				}
			}
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
			/*
			 * ☑ A COMPRESSED LANDING PARENT IS NOT, BY ITSELF, A SHAPE
			 * THIS FRAME CANNOT EXPRESS -- and the refusal that said so
			 * was VACUOUS for two thirds of what it caught.
			 *
			 * It read: "a COMPRESSED landing parent carries a SKIP_X dual
			 * the recompact re-encodes into its OWN parent's slot, and
			 * this walk has not derived that great-grandparent pair."
			 * Every clause is true and none of them bites.  The recompact
			 * takes GP only when the hint's @gp is non-NULL; a DEL never
			 * re-encodes the dual itself; and the detach's own republish
			 * records the dual from @cn_meta as MW (@dual_owner_held
			 * false, the arm below).  ☞ THE UN-CLIMBED REKEY PATH IS THE
			 * SHIPPED PRECEDENT: ft-rekey.h's own elevated hint passes
			 * `.gp = NULL, .gp_slot = NULL` under a compressed parent and
			 * has done all along.
			 *
			 * MEASURED by ablating it: 19 of the 26 shapes it refused on
			 * -DNO_FEATURE_FT_SKIP_COMPRESSED (4 of 5 on the default
			 * build) are then SERVED, cds_ft_verify CLEAN, keys right, on
			 * every rank x list arm and on --enable-rcu-debug and
			 * -DFT_REKEY_CLAIM -- including the shapes where the
			 * great-grandparent is an INTERNAL node, which the 3000-shape
			 * corpus cannot even produce.
			 *
			 * ☠ WHAT IT WAS REALLY CATCHING is one sub-family and nothing
			 * else: the resting node's run parent is the very node the
			 * CALLER's graft SPLIT.  That is the collapse-vs-split
			 * collision the arm at the top of this function refuses, wearing
			 * a second exit -- ablate this one on that sub-family and the
			 * engine aborts on `r->kind == kind` (rcu-debug) or the op
			 * livelocks (noskip).  So the predicate is narrowed to the
			 * thing that is actually red, and stays terminal: the shape is
			 * deterministic, so -EAGAIN would spin.
			 */
			/*
			 * ☠ @split_cn IS THE COMPRESSED NODE POINTER, NOT THE FLAG --
			 * the arm at the top of this function passes @parent_cn for
			 * the same lookup.  Handing it the flag compares a tagged word
			 * against an untagged one, matches nothing, and lets the one
			 * shape this guard exists for straight through: MEASURED, the
			 * engine then aborts on `r->kind == kind` at seed 2919.
			 */
			if (climbed && src_held_hint && boundary_parent_nf &&
					!topmost_external_nodes &&
					ft_node_compressed(boundary_parent_nf) &&
					ft_glue_that_split(&lctx,
						ft_compressed_node_ptr(
							boundary_parent_nf)) != NULL) {
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
			/*
			 * THE LEAF-DELETE HOIST: LOCK THE HOLDER BEFORE THE PLAN IS
			 * READ (doc/design/ft-lockset-inventory.md §1, row 1).
			 *
			 * ft_node_replace_ptr's in-place arm decides "stay in place"
			 * from the holder's nr_child and defers the slot clear plus
			 * the fused nr_child-- into @pub -- and until now this op
			 * held NOTHING when it read that count, when it recorded
			 * those two words, and when it committed them: 2.6M commits
			 * per ft_inv run, arbitrated only by the fused CAS.  The
			 * promote arm below acquires the holder, but AFTER the plan;
			 * the leaf arm never did.
			 *
			 * So take the holder -- its own word at per-node, its ANCHOR
			 * above -- here, ahead of the decision, exactly as the insert
			 * tier's hoist takes the attach node ahead of its reserve.
			 * The count replace_ptr reads is then the count under the
			 * lock, and the freeze_leaf chain word recorded below is
			 * under its nearest ancestor lock.  Registered in @commit_txn
			 * with its {LOCK|s -> s} release, so every exit reaches a
			 * terminal: the commit consumes it, and @end's destroy of an
			 * unused txn releases it.  If replace_ptr routes to a DEL
			 * recompaction instead, its {C,P,GP} acquire finds C held
			 * through @lctx and dedupes rather than refusing its own mark.
			 *
			 * Scope: the point removes on a SHARED trie.  An exclusive
			 * trie has no peer to exclude, the bulk callers pass the
			 * exclusive-only tier, and the FOLD's @shared_txn belongs to
			 * its caller.  A promote keeps its own acquire below.
			 *
			 * ☞ LOCKS FIRST, SW LATER.  @leaf_hoisted keeps the B2 arm
			 * below from converting this commit's records to SW: the
			 * transition flips MW to SW once, when every writer of these
			 * words holds its lock, not one producer at a time.
			 */
			if (ft->lock_fine && !ft->exclusive && in_place &&
					!record_only && commit_txn &&
					!topmost_external_nodes) {
				enum ft_lock_or_guard_exit hex;

				/*
				 * LOCK THE HOLDER THE PLAN NAMES.  @iter_node_flag is
				 * RE-READ from the parent slot after the climb, and
				 * @metadata_stack is what the climb derived: a peer
				 * that republished that slot in between (a
				 * recompaction of the holder into a copy) leaves the
				 * two naming DIFFERENT nodes.  MEASURED 1,819 times
				 * per ft_inv run at per-node, climb or no climb.  The
				 * pre-hoist code carried that torn plan to its commit
				 * and aborted there; locking @iter_node_flag would
				 * instead hold the COPY while the records name the
				 * planned node.  So refuse the torn plan before the
				 * acquire, and re-check the slot under the lock.
				 */
				if (ft_flag_to_metadata(ft, iter_node_flag) !=
						metadata_stack[nr_branch - 1]) {
					ret = -EAGAIN;
					goto end;
				}
				FT_INTERLEAVE(FT_IL_LEAF_HOIST_PRE);
				lctx.held.txn = commit_txn;
				lctx.held.nr_extra = (unsigned int) nr_orphan_locked;
				ft_flip_txn_lock_or_guard_parent_ex(__func__,
					__LINE__, ft, commit_txn, &lctx,
					iter_node_flag, cur_depth, &hex);
#ifdef FT_DEBUG_STRUCT_ANCHOR
				ft_sa_hoist_exit_count(1, hex);
#endif
				if (hex == FT_LOG_EXIT_MISS) {
					ret = -EAGAIN;
					goto end;
				}
				leaf_hoisted = true;
				/*
				 * Under the lock the holder can no longer be retired
				 * or relocated, so a slot that still names it names a
				 * LIVE, LINKED holder.  A registered lock is released
				 * by @end's destroy of the unused @commit_txn.
				 */
				if (rcu_dereference(*detach_parent_flag_ptr) !=
						iter_node_flag) {
					ret = -EAGAIN;
					goto end;
				}
#if defined(FEATURE_FT_SKIP_COMPRESSED) && !defined(FT_DEBUG_INPLACE_DELETE_RESIDUE)
				/*
				 * THE IN-PLACE TWIN OF THE RECOMPACTION RE-PLAN below.
				 * This hoist only runs for a pure delete, and an
				 * in-place delete drops the boundary's child count at
				 * its commit with no copy for that check to inspect.
				 * The shape-D test above is a PLAN read: a peer that
				 * removed the boundary's prefix key (or a third child)
				 * since leaves, under the lock we now hold, a keyless
				 * 2-child node this delete turns into the 1-child
				 * keyless internal skip mode refuses.  Measured on the
				 * in-place build's owned-key row: 5 of 96 runs left
				 * one at root-only and exponential spacing, 0 of 96
				 * with this re-plan.  Nothing is
				 * recorded yet and the lock rides @commit_txn: re-plan;
				 * the next plan reads the node keyless and collapses
				 * it.  Only where this plan did NOT choose the
				 * collapse, as below, so an out-of-bound shape-D
				 * cannot send it round forever.
				 */
				if (!shape_d_plan &&
				    ft_group_skip_compressed(ft->group)) {
					struct cds_ft_metadata *bm =
						metadata_stack[nr_branch - 1];

					if (ft_meta_nr_child_load(bm) == 2 &&
					    !bm->external_nodes &&
					    ft_parent_node(bm->parent_word) != NULL) {
						ret = -EAGAIN;
						goto end;
					}
				}
#endif
				FT_INTERLEAVE(FT_IL_LEAF_HOIST_POST);
			}
			ret = ft_node_replace_ptr(ft,
				detach_node_flag_ptr,
				elevated_old_child,
				&iter_node_flag,
				&old_recompacted_node,
				metadata_stack[nr_branch - 1],
				n, (struct cds_ft_inode_flag *) topmost_external_nodes,
				in_place,
				detach_parent_flag_ptr == &ft->root,
				cur_depth, pub, commit_txn, replace_hint,
				&lctx);
		}
#if defined(FEATURE_FT_SKIP_COMPRESSED) && !defined(FT_DEBUG_RECOMPACT_RESIDUE)
		/*
		 * A COPY THAT SHOULD HAVE BEEN A COLLAPSE.  The shape-D test above
		 * is a PLAN read: it saw @iter_node_flag keep its prefix key (or a
		 * third child), so removing one child left a legal keyed node and
		 * no collapse was owed.  A peer removing that prefix key between the
		 * plan and the recompaction's lock changes the answer: the copy the
		 * recompaction just built from the LOCKED state has one child and no
		 * key -- the 1-child keyless internal cds_ft_verify refuses in skip
		 * mode, and which a later remove below it cannot date its plan on
		 * (it livelocks at exponential spacing).  Measured: 143 such copies
		 * in 24 owned-key runs, every one planned with nr_child 2 and a key.
		 * Nothing is committed yet -- the copy is unpublished and its
		 * re-parent sweep and retire ride @commit_txn -- so re-plan; the
		 * next attempt sees the node without its key and collapses it.
		 * Only where this plan did NOT choose the collapse, so a shape-D that
		 * refuses as out of bound cannot send it round forever; not for a
		 * bulk fold (@record_only), which no peer can race.
		 */
		if (!ret && !boundary_fused && !shape_d_plan && !record_only &&
		    old_recompacted_node && !topmost_external_nodes &&
		    ft_group_skip_compressed(ft->group) &&
		    !ft_node_external(iter_node_flag) &&
		    !ft_node_compressed(iter_node_flag) &&
		    !ft_node_skip_compressed(iter_node_flag)) {
			struct cds_ft_metadata *nm = cds_ft_item_to_metadata(
				ft_node_ptr(iter_node_flag));

			if (ft_meta_nr_child_load(nm) == 1 &&
			    !nm->external_nodes &&
			    ft_parent_node(nm->parent_word) != NULL)
				ret = -EAGAIN;
		}
#endif
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
					orphan_trailing_held,
					/* @commit_txn IS @shared_txn then */
					record_only);
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
#ifdef FT_DEBUG_CHAIN_HOLD
				/*
				 * THE HEAD-WORD CONTROL: the removed leaf is a LIVE
				 * head until this commit, so ft_ch_head_reachable must
				 * read it reachable -- else its HIDDEN zeros are blind.
				 */
				if (ft->lock_fine && !ft->exclusive &&
						ft_wlock_held != ft)
					ft_ch_head_ctl_count(ft_ch_head_reachable(ft,
						freeze_leaf));
#endif
				ft_ch_audit_ctx(ft, commit_txn, &lctx, freeze_leaf);
				/*
				 * ★ VALIDATE THE PLAN UNDER THE LOCK (Mathieu,
				 * 2026-09-19).  @freeze_len reached this op as a
				 * LITERAL -- cds_ft_remove's "1 = this key's SOLE
				 * entry" -- derived from a read of @succ_node taken
				 * with NOTHING HELD, while the chain's holder is not
				 * acquired until ft_node_replace_ptr above.  So a
				 * same-key insert can append a duplicate in that
				 * window, and the freeze's derived NULL has been
				 * standing in for the exclusion the count did not
				 * have: MEASURED 10,157 (per-node) / 16,219
				 * (exponential) disagreements per ft_inv leg, with
				 * the txn registry owning the holder at 100% of them.
				 *
				 * We hold it HERE, so ask the word.  A stale plan is
				 * refused before any record is filed, which is the
				 * SAME decision the commit's install CAS makes today
				 * -- taken earlier, and mostly without needing the
				 * CAS to make it.  ☠ NOT "instead of" the CAS: see
				 * ft_hlist_chain_plan_ok's header -- an UNDECIDED
				 * peer proxy that outlasts urcu_txn_read's patience
				 * window still reads as NULL here, so the install CAS
				 * remains the backstop until the lock set is
				 * complete.  This shrinks the window the CAS
				 * arbitrates; it does not on its own make the record
				 * parkable.
				 *
				 * ☠ RETRIABLE, AND IT TERMINATES: the caller
				 * re-derives and finds the longer chain, so the next
				 * attempt takes the promote/unchain lane instead of
				 * this prune -- it does not re-enter the same refusal.
				 *
				 * ☠ AND NOTHING IS READER-VISIBLE YET.  On this arm
				 * (@pub and @commit_txn both present) the forward
				 * store, the head back edge and the recompaction
				 * republish are all RECORDED into @commit_txn, which
				 * ft_remove_one_commit does not reach until below; the
				 * lone bare rcu_assign_pointer of the back edge is the
				 * @commit_txn-NULL arm, which cannot be this one.  So
				 * @end's sweep returns the structure byte-for-byte to
				 * its pre-op state, exactly as for the leaf hoist's
				 * two -EAGAIN exits above.
				 */
				if (!ft_hlist_chain_plan_ok(ft,
						ft_flip_txn_handle(commit_txn),
						freeze_leaf, freeze_len)) {
					FT_HLIST_PLAN_BAIL();
					FT_DBG_RETRY_SITE();
					/*
					 * ☠ THE ERRNO IS LOAD-BEARING, not just a
					 * status.  @end's recompaction arm keys on it
					 * EXACTLY: `ret == -EAGAIN` frees the
					 * UNPUBLISHED fresh copy, anything else
					 * call_rcu-frees @old_recompacted_node -- the
					 * OLD, still-linked node.  -ENOENT is already a
					 * legal detach return elsewhere, so a later
					 * edit of this bail to any other negative value
					 * is a use-after-free one token away.
					 */
					ret = -EAGAIN;
					goto end;
				}
				ft_hlist_freeze_chain_prepare_checked(ft,
					ft_flip_txn_handle(commit_txn),
					freeze_leaf, freeze_len);
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
				/*
				 * ☞ THE BACK-EDGE CLAIM READS ZERO ON EVERY OWNER
				 * COLUMN HERE, AND THE OP DOES HOLD THE OWNER.
				 *
				 * FT_BE_DETACH_UNCHAIN is the one site where the txn
				 * registry, the per-thread hold ledger AND the CALLER's
				 * @op_ctx all report "holds nothing".  MEASURED: the
				 * owner's state word carries FT_STATE_LOCK (0x80004,
				 * tombstone clear) in 5/5 samples of a SINGLE-writer
				 * run, and @lctx -- THIS function's own lock context --
				 * answers held in 5/5.
				 *
				 * The marks live in @orphan_held, a fn-scope array
				 * rather than the txn locks[] registry (the orphan chain
				 * reaches FT_MAX_DEPTH, past FT_FLIP_TXN_MAX_LOCKS; see
				 * the DLM orphan plan-lock note at the top), so they are
				 * the `extra[]` arm of the held set: reachable from
				 * @lctx, invisible to @op_ctx and to the registry by
				 * construction.  Do not read the claim's zeros here as
				 * an unowned write -- ask @lctx.
				 */
				if (pub->head_parent_field) {
					if (commit_txn) {
						ft_flip_txn_record_head_back_edge_owned(
							commit_txn,
							(void **) pub->head_parent_field,
							pub->head_parent_old,
							pub->head_parent_new,
							ft_back_edge_owner(
								pub->head_parent_old)
							FT_BE_SITE(FT_BE_DETACH_UNCHAIN, &lctx));
					} else {
						rcu_assign_pointer(*pub->head_parent_field,
							pub->head_parent_new);
						FT_WIN_NOTE_RAW(pub->head_parent_field,
							pub->head_parent_new, 1);
					}
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
					/*
					 * ☠ A MISSED HOLDER ENDS THE ATTEMPT HERE.  A
					 * peer holds @iter_node_flag's word (a parked
					 * proxy on it counts), so the take latched
					 * @acquire_miss and @commit_txn is doomed: its
					 * commit would discard everything.  Carrying on
					 * arms the per-op claim and records the promote
					 * on @pub->slot -- a word this op does not own --
					 * which the rcu-debug owner assert catches
					 * (MEASURED: inv_prefix_shape_zoo, in-place
					 * rcu-debug at exponential, remove_all's detach,
					 * 2 of 128 pinned runs; the dump showed the
					 * holder's state word carrying a peer's proxy).
					 * Nothing is committed: @end destroys the unused
					 * txn, releasing what it registered, as
					 * insert_replace's and remove_all's arms do
					 * (dbc41961, da4ac288).
					 */
#ifndef FT_DEBUG_DETACH_MISS_CARRIES_ON
					if (caa_unlikely(commit_txn->acquire_miss)) {
						ret = -EAGAIN;
						goto end;
					}
#endif
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
				if (commit_txn && !leaf_hoisted)
					ft_flip_txn_arm_per_op(ft, commit_txn);
#ifdef FT_DEBUG_DEL_TOMB
				/*
				 * PROBE: name the two derivations when the slot this
				 * delete clears is not inside the node whose count it
				 * decrements.
				 */
				if (pub->state_meta) {
					char *sn = (char *) cds_ft_metadata_to_item(
						pub->state_meta);
					char *sl = (char *) pub->slot;
					static unsigned long ft_dt_split_ctx;

					if ((sl < sn || sl >= sn + ((size_t) 1 <<
							cds_ft_item_order(sn))) &&
							uatomic_add_return(&ft_dt_split_ctx, 1) <= 8) {
						unsigned int di;

						fprintf(stderr, "FT DEL-SPLIT-CTX slot=%p state_node=%p nr_branch=%d nr_clear=%d nr_metadata=%d climbed=%d climb_promoted=%d entry_holder_slot=%p entry_holder_raw=%p entry_slot_now=%p holder_old_flag=%p detach_node_flag_ptr=%p detach_parent_flag_ptr=%p iter_node_flag=%p elevated_old_child=%p plan_old_child=%p topmost_ext=%p arg_detach_slot=%p arg_parent_slot=%p\n",
							(void *) pub->slot, (void *) sn,
							nr_branch, nr_clear, nr_metadata,
							(int) climbed, (int) climb_promoted,
							(void *) entry_holder_slot,
							(void *) entry_holder_raw,
							(void *) CMM_LOAD_SHARED(*entry_holder_slot),
							(void *) holder_old_flag,
							(void *) detach_node_flag_ptr,
							(void *) detach_parent_flag_ptr,
							(void *) iter_node_flag,
							(void *) elevated_old_child,
							(void *) plan_old_child,
							(void *) topmost_external_nodes,
							(void *) dbg_arg_detach_slot,
							(void *) dbg_arg_parent_slot);
						for (di = 0; di < nr_metadata; di++)
							fprintf(stderr, "FT DEL-SPLIT-CTX   metadata_stack[%u]=%p item=%p state=%#lx\n",
								di, (void *) metadata_stack[di],
								cds_ft_metadata_to_item(metadata_stack[di]),
								(unsigned long) CMM_LOAD_SHARED(metadata_stack[di]->state));
					}
				}
#endif
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
			/*
			 * @mtxn: this publish can carry a compressed
			 * grandparent's SKIP_X dual, and the dual's HOME is
			 * resolved from a parent_word THIS op may already have
			 * a pending re-parent for (a same-trie rekey folds its
			 * detach into the graft's commit).  Without the handle
			 * ft_dual_home_is_private cannot see that re-parent and
			 * answers with the PRE-OP slot -- a word inside the node
			 * this very commit retires.  Same defect as the
			 * external-promote arm above, two sites deeper.
			 */
			struct ft_pub_rec rec = { .ctx = &lctx, .n = 0,
				.mtxn = commit_txn ? commit_txn->mtxn : NULL };

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
				ft_flip_txn_guard_parent_ctx(ft, commit_txn,
					ft_parent_node(iter_meta->parent_word),
					op_ctx);
			/*
			 * PHASE B, STEP B2 -- THE ARM, RECOMPACTION PUBLISH
			 * (fused).  This is where the remove surface commits
			 * whenever the delete RECOMPACTED: every delete does on
			 * a build without FEATURE_FT_INSERT_IN_PLACE, and one
			 * that would shrink the holder below min_child does on
			 * any build (@in_place, ft_node_replace_ptr); an
			 * in-place delete commits through the pub-armed arm
			 * above instead.
			 *
			 * PLACEMENT.  The op's last ft_flip_txn_lock_register
			 * on @commit_txn is ft_node_recompact's RELEASE-half
			 * loop ({P}, plus {GP} when P is compressed), which ran
			 * inside ft_node_replace_ptr above; the line beside
			 * this one plants a read-set guard and registers
			 * nothing, and ft_remove_commit_rec below plants the
			 * first record after it.  Measured (-DFT_ARM_REACH):
			 * @nr_locks is NEVER zero here, so the helper's
			 * empty-registry refusal is not what decides this site.
			 *
			 * ☞ WHAT IT CONVERTS is the forward publish (+ a
			 * compressed grandparent's SKIP_X dual), whose owner is
			 * the grandparent this recompact holds and released
			 * through this very txn.  The recompaction's OWN edges
			 * -- the reparent sweep, the child state guards -- are
			 * not converted and must not be: they dispatch MW
			 * themselves (@child_held false in
			 * ft_flip_txn_record_parent_word /
			 * ft_reparent_record_meta) because C's CHILDREN are
			 * never in the DLM set.  The cell / run edges stay
			 * MW_ALWAYS by their own helpers.
			 *
			 * ☠ NOT ON THE FOLD PATH.  @record_only means
			 * @commit_txn is the CALLER's shared txn, and whether
			 * its registry is complete is the caller's judgement,
			 * not this frame's -- the rekey writer arms it itself,
			 * so refusing here costs nothing.
			 */
			/*
			 * §9.3's THIRD MEMBER, as on the non-fused twin below:
			 * the dual's grandparent is DERIVED at publish, so the
			 * only sound way to hold it is to acquire it from that
			 * same derivation.  ABOVE the arm -- it is a
			 * ft_flip_txn_lock_register.
			 */
			dual_gp_held = ft_lock_skip_dual_gp(ft, &lctx, commit_txn,
				ft_parent_node(iter_meta->parent_word),
				commit_txn ? commit_txn->mtxn : NULL);
			if (commit_txn && !commit_txn_used && !record_only)
				ft_flip_txn_arm_per_op(ft, commit_txn);
			_ft_publish_to_parent(ft, ft_parent_node(iter_meta->parent_word),
				detach_parent_flag_ptr, iter_node_flag,
				holder_old_flag, &rec,
				/*
				 * ☠ FALSE, AND `old_recompacted_node != NULL`
				 * WAS NOT SOUND.  It said "the recompact took
				 * {C,P,GP}, so we hold the dual's owner", and
				 * that is a PLAN-TIME fact: ft_node_recompact
				 * resolves @pf_gp before the acquire and takes
				 * GP only `if (pf_gp)`, while the empty member
				 * is skipped by ft_dlm_acquire_set's
				 * `if (!set[i].nf) continue` -- guard included.
				 * The dual's owner, meanwhile, is DERIVED FRESH
				 * at publish from cn_meta's back-pointer.  So a
				 * compressed P that was ROOT-ATTACHED at plan
				 * time yields no GP and no guard, and a peer
				 * re-home landing in that window makes the
				 * publish derive a grandparent this op never
				 * acquired -- an SW park on an unowned word, at
				 * an armed site, silent without rcu-debug.
				 *
				 * No current op live-re-homes a root-attached
				 * compressed node (root restructures retire and
				 * rebuild), so the window is unproven-reachable
				 * -- which is a reason to keep looking, not a
				 * reason to park on it.  The dual costs one
				 * record on the minority of republishes whose
				 * parent is compressed; MW is stricter and
				 * always sound.
				 *
				 * ☞ To make this true again, the ANSWER must be
				 * publish-time: the acquired GP compared against
				 * the derived dual owner, not the plan's intent.
				 */
				dual_gp_held);
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
			/* @mtxn: as the fused arm above -- the dual's home must
			 * be resolved against this op's own pending re-parent. */
			struct ft_pub_rec rec = { .ctx = &lctx, .n = 0,
				.mtxn = commit_txn ? commit_txn->mtxn : NULL };

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
			/*
			 * §9.3's THIRD MEMBER, and the answer this producer owes
			 * the record below.  The paragraph that used to sit at
			 * the @dual_owner_held argument said exactly how to earn
			 * it -- "the ANSWER must be publish-time: the acquired GP
			 * compared against the derived dual owner, not the plan's
			 * intent" -- and ft_lock_skip_dual_gp IS that: it derives
			 * the GP the same way the record will and acquires THAT,
			 * so its return is the honest held answer rather than an
			 * inference from `old_recompacted_node != NULL`.
			 *
			 * A no-op unless a dual is actually recorded; @mtxn NULL
			 * mirrors this producer's @rec, which carries none.
			 * ABOVE the arm, because it is a ft_flip_txn_lock_register
			 * and the arm's contract is "after the op's LAST
			 * register" -- the same placement the promote arms use.
			 */
			dual_gp_held = ft_lock_skip_dual_gp(ft, &lctx, commit_txn,
				ft_parent_node(iter_meta->parent_word),
				commit_txn ? commit_txn->mtxn : NULL);
			/*
			 * PHASE B, STEP B2 -- THE ARM, RECOMPACTION PUBLISH
			 * (non-fused) and the non-in-place external promote.
			 * The argument is the fused arm's above, with the one
			 * difference that decides the placement: on the else
			 * arm the lock_or_guard beside this line IS this path's
			 * last register (the recompact arm's was
			 * ft_node_recompact's release loop), so the arm sits
			 * below BOTH.
			 */
			if (commit_txn && !commit_txn_used && !record_only)
				ft_flip_txn_arm_per_op(ft, commit_txn);
			_ft_publish_to_parent(ft, ft_parent_node(iter_meta->parent_word),
				detach_parent_flag_ptr, iter_node_flag,
				holder_old_flag, &rec,
				/*
				 * ☐ STILL false -- but for a DIFFERENT reason
				 * than before, and the difference is the whole
				 * finding.  The note that stood here explained why
				 * `old_recompacted_node != NULL` was NOT sound:
				 * it is a PLAN-TIME fact (ft_node_recompact
				 * resolves @pf_gp before the acquire and takes
				 * GP only `if (pf_gp)`, and an empty member is
				 * skipped by ft_dlm_acquire_set), while the
				 * dual's owner is DERIVED FRESH at publish from
				 * cn_meta's back-pointer -- so a compressed P
				 * that was ROOT-ATTACHED at plan time yields no
				 * GP, and a peer re-home landing in that window
				 * would have parked SW on a word this op never
				 * acquired.
				 *
				 * ☑ THE ACQUIRE IS NOW TAKEN ANYWAY, above --
				 * and it is worth taking for its own sake, kind
				 * aside: struct ft_pub_rec says "holding GP is
				 * what EXCLUDES a peer recompaction from copying
				 * that body out from under the record, and that
				 * is ft_lock_skip_dual_gp's job, not this word's".
				 * So this site now HOLDS the word it writes.
				 *
				 * ☑ AND THE KIND NOW FOLLOWS @dual_gp_held.  The
				 * note that stood here said the flip waited on
				 * ft_node_recompact ("it holds the DERIVED
				 * grandparent 0 times in 9721"), then corrected
				 * itself with "the kind is a per-op claim, not a
				 * global one -- the LOCK serialises an SW park
				 * against an MW CAS, so producers convert
				 * INDIVIDUALLY".
				 *
				 * ☠ THAT SECOND SENTENCE IS TRUE ONLY OF AN MW
				 * THAT HOLDS THE WORD, and stating it unqualified
				 * is how a body word gets read as lock-bearing.
				 * An MW record whose op holds nothing arbitrates
				 * against nothing, so a peer's SW park -- a plain
				 * store -- genuinely races it.  What makes the
				 * per-site conversion safe is not that "MW and SW
				 * serialise": it is that a site which cannot
				 * vouch answers FALSE and stays MW.  The
				 * lock-bearing carve-out (an MW record TAKES the
				 * lock, SW writes follow it) belongs to
				 * @metadata.state, NOT to a grandparent body
				 * word, which has no lock in its bits.
				 *
				 * The 9721 reading was itself a blind query: that
				 * site HELD its word and was asked through a
				 * registry ft_dlm_acquire_set can never populate.
				 * ☞ THE TRANSACTED-SLOT REGISTER.
				 */
				dual_gp_held);
			(void) dual_gp_held;
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
		/*
		 * ☠ NEVER ON THE FOLD (@record_only): this is a STANDALONE
		 * SECOND FLIP, and under the fold the caller's one-decide txn
		 * has NOT COMMITTED yet -- canonicalizing here publishes over a
		 * plan the caller still holds (measured: ft_skip_reanchor's
		 * assert(0), the up-walk landing on a mid-plan parent chain).
		 * Reachable on the fold only through the out-of-bound residue
		 * (merged_len > FT_SKIP_LEN_MAX falls back to the recompaction
		 * above; the pending-publish sibling shape is either FUSED with
		 * the @pending_child substitution or refused whole): that rare
		 * geometry -- a near-max compressed parent run over a 2-child
		 * boundary -- then keeps its 1-child internal past the op, which
		 * skip mode's verifier flags at the writer-scope exit.  KNOWN
		 * INCOMPLETE: closing it needs the caller to own a post-commit
		 * canonicalize, which is a rekey-side change, not this frame's.
		 */
		if (!record_only &&
		    ft_meta_nr_child(iter_meta) == 1 &&
		    !iter_meta->external_nodes &&
		    ft_parent_node(iter_meta->parent_word) != NULL) {
			/*
			 * Post-commit: the structural commit consumed its own
			 * registry, so only the orphan marks this op still owns
			 * are held.
			 */
			lctx.held.txn = NULL;
			lctx.held.nr_extra = (unsigned int) nr_orphan_locked;
#ifndef FT_DEBUG_NO_FOLD_RENAME
			if (wd_valid && old_recompacted_node)
				ft_descent_rename(&wd, old_recompacted_node,
					iter_node_flag);
#endif
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
					!orphan_held[oi].txn_owned) {
				/*
				 * ☠ ONLY A TXN THIS DETACH DOES NOT COMMIT is
				 * safe to ask.  A commit CONSUMES the flip-txn
				 * wrapper, so on every other path @commit_txn /
				 * @shared_txn may already be freed here and
				 * reading ->mtxn->desc is a use-after-free
				 * (measured: SEGV at both fine spacings, test 9,
				 * inside urcu_txn_find on a dead descriptor).
				 * @record_only is exactly the deferred contract:
				 * the collapse is RECORDED into the caller's txn
				 * and the caller commits it, so it is open here.
				 */
				struct ft_flip_txn *pend_txn = record_only ?
					shared_txn : NULL;
				struct urcu_txn_record *pend =
					ft_lock_terminal_pending(pend_txn,
						orphan_held[oi].lock);
				/*
				 * ☠ THE TERMINAL HAS NOT RUN YET.  The sweep's
				 * premise -- a successful detach COMMITTED this
				 * word's fused tombstone, an aborted one recorded
				 * none -- holds only when the detach commits its
				 * own txn.  Under a DEFERRED commit (@record_only
				 * / @fold_replace: the rekey's one-decide writer
				 * commits @shared_txn later) the record exists
				 * and is UNDECIDED right here, and its expected
				 * old is {LOCK|s}: a plain-CAS release now drops
				 * a bit the record still counts on, the word
				 * becomes lockable by a peer, and the SW park
				 * that follows writes over the peer blind.
				 *
				 * So hand the word to the txn that carries the
				 * terminal and let it own the clear on BOTH
				 * outcomes: its commit consumes the bit into the
				 * tombstone, its destroy releases it.  Measured
				 * (-DURCU_TXN_DEBUG_PARK_CLOBBER): the parks that
				 * clobbered {old 0x80004, found 0x4} were all
				 * this, worst at per-node spacing, where the
				 * orphan's lock IS its own word and the existing
				 * hand-off (h->lock != m) never fires.
				 */
#ifdef FT_RED_ORPHAN_RELEASE_EARLY
				pend = NULL;	/* red control: release anyway */
#endif
				if (pend) {
					FT_ORPHAN_HANDED();
					if (!ft_flip_txn_holds(pend_txn,
							orphan_held[oi].lock)) {
						unsigned int ls;

						/*
						 * ☠ REGISTER, BUT WITHOUT THE
						 * SILENCER.  ft_flip_txn_lock_own
						 * links &h->shared so the commit's
						 * terminal can scrub the handing
						 * entry -- and @orphan_held is THIS
						 * FRAME'S STACK.  A deferred commit
						 * runs after ft_detach_node has
						 * returned, so that pointer would be
						 * dangling: the terminal writes
						 * through it (*src_shared = true)
						 * and the run SEGVs (measured, both
						 * fine spacings).  Nothing reads
						 * @orphan_held past this sweep --
						 * it is the frame's last use -- so
						 * the silencer has no job here.
						 */
						ft_flip_txn_lock_register_member(
							pend_txn,
							orphan_held[oi].lock,
							orphan_held[oi].lock_snap,
							orphan_held[oi].member);
						ls = pend_txn->nr_locks - 1;
						if ((uintptr_t) pend->new_ptr &
								FT_STATE_TOMBSTONE)
							pend_txn->locks[ls].
								tombstone_terminal =
								true;
					}
					orphan_held[oi].txn_owned = true;
					continue;
				}
				/*
				 * ☠ THE FENCE MAY ALREADY BE GONE, AND THE WORD
				 * CANNOT SAY SO.  Our own committed terminal drops
				 * the LOCK; from that instant the word is a peer's
				 * to take, and a set bit means THEIRS, not ours.
				 * The sweep used to ask the word (clear_if_held)
				 * on the premise quoted above -- "an acquire
				 * refuses a TOMBSTONE, so the bit cannot come
				 * back" -- which holds only for a terminal that
				 * actually leaves the word TOMBSTONEd.  Where it
				 * drops LOCK without one, the word is re-lockable
				 * and the sweep clears a peer's fence.
				 *
				 * So ask what THIS op did.  Under @record_only the
				 * descriptor is still alive and answers exactly
				 * (above).  Otherwise it is already freed -- the
				 * UAF this block's first comment records -- and
				 * the op's own outcome is the witness: @ret == 0
				 * means the detach committed, so its fused
				 * tombstone consumed the fence; any bail or abort
				 * applied nothing and the mark is still {LOCK|s}.
				 */
				if (!pend_txn && !ret) {
					FT_ORPHAN_CONSUMED();
					/*
					 * PREFER A LEAK TO A THEFT, AND MEASURE
					 * IT.  If the premise is ever false the
					 * fence stays set and the node becomes
					 * unmutable -- bad, but bounded and
					 * local, where a theft corrupts a peer's
					 * exclusion.  Record it rather than
					 * refuse: a still-LOCKED, un-TOMBSTONEd
					 * word here is the premise failing, and
					 * the count is the thing to look at next.
					 */
					FT_ORPHAN_PREMISE_CHECK(
						orphan_held[oi].lock);
					continue;
				}
				FT_ORPHAN_RELEASED();
				ft_meta_lock_release(orphan_held[oi].lock);
				/*
				 * SCRUB only RELEASED-LIVE (finding A); a
				 * TOMBSTONED word is a CONSUMED fence and must
				 * keep answering holds() -- dedupe-on-dead is
				 * the designed flow, and taking a dead word
				 * hard-refuses forever (the exp-MW storm).
				 */
				if (!(CMM_LOAD_SHARED(
						orphan_held[oi].lock->state) &
						FT_STATE_TOMBSTONE))
					orphan_held[oi].shared = true;
			}
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
#ifdef FT_DEBUG_STACK_SILENCER
	/*
	 * ☠ DOES A TXN THAT OUTLIVES THIS FRAME STILL POINT INTO IT?
	 *
	 * ft_flip_txn_lock_own links &h->shared as the terminal's silencer, and
	 * @orphan_held is THIS frame's stack.  When the txn is the caller's
	 * deferred one (@record_only), its terminal runs after this function has
	 * returned and writes through that pointer -- the same dangling write
	 * that SEGV'd the orphan sweep before @c5ab837d.  The bounds are known
	 * exactly here, so this is a measurement, not a heuristic: no address
	 * arithmetic guess, just "is the registered silencer inside the array I
	 * am about to destroy".
	 */
	if (record_only && shared_txn) {
		unsigned int li;

		/* ARM: a zero above is only evidence if this ran at all. */
		uatomic_inc(&ft_stack_silencer_scans);
		uatomic_add(&ft_stack_silencer_locks, shared_txn->nr_locks);
		for (li = 0; li < shared_txn->nr_locks; li++) {
			const void *sil = (const void *)
				shared_txn->locks[li].src_shared;

			if (sil >= (const void *) &orphan_held[0] &&
					sil < (const void *) &orphan_held[
						FT_MAX_DEPTH + 1]) {
				if (uatomic_add_return(&ft_stack_silencer_hits,
						1) <= 4)
					fprintf(stderr, "FT STACK SILENCER: txn %p lock slot %u points at %p, inside this frame's orphan_held[] -- it outlives the frame\n",
						(void *) shared_txn, li, sil);
			}
		}
	}
#endif
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
	if (!ret && freeze_leaf && !freeze_leaf_fused) {
		unsigned int fi;
		struct cds_ft_node *fn = freeze_leaf;

		/*
		 * Bounded by @freeze_len for the same reason the txn arm is: a
		 * duplicate appended since the caller's derivation is NOT this
		 * op's to retire (ft_hlist_freeze_chain_prepare).  This arm has
		 * no commit to abort, so the bound is the only thing holding
		 * that line.
		 */
		/*
		 * ☠☠ THE LAST RAW PRODUCER ON A CHAIN WORD, AND THE CHAIN NOW
		 * PARKS.  ft_node_mark_removed_flip CASes &fn->next outside the
		 * record layer, so it cannot be SW-parked; since @80e855f5 every
		 * other writer of that word parks SW by default.  Its loop spins
		 * out a parked FT_HLIST_TAG before its CAS, which covers
		 * park-then-CAS -- but NOT the reverse: a CAS that lands first
		 * on a clean word is overwritten by the parker's blind settle,
		 * republishing the successor with the TOMBSTONE ERASED (see that
		 * function's header for the full ordering).
		 *
		 * It is safe today only because this arm is not reached:
		 * -DFT_DEBUG_CHAIN_CANARY scores the producer at raw = 0 in
		 * ft_inv AND ft_unit at all three spacings, and the chain hold
		 * audit gives this fallback no row at all.
		 *
		 * ☠ THAT IS A COVERAGE STATEMENT ABOUT TWO SUITES, NOT A PROOF,
		 * and a stale one would be silent.  So count every entry and,
		 * where the engine's own self-checks are armed, ABORT: if this
		 * path ever becomes reachable it must be converted to a RECORDED
		 * store before it runs, never re-argued.
		 *
		 * ☞ ASSERT ONLY, no counter: a counter on a path that never
		 * executes is write-only in a release build (nothing prints
		 * it), so it would be decoration.  The measurement above is the
		 * record; this is what stops a silent regression.
		 */
		urcu_assert_debug(!"ft_detach_node standalone freeze reached: "
			"raw CAS on a chain word that now parks SW -- convert "
			"it to a recorded store");
		for (fi = 0; fi < freeze_len && fn; fi++) {
			struct cds_ft_node *fnext;

			ft_ch_audit(ft, NULL, fn);
			fnext = ft_node_mark_removed_flip(ft, fn);
			fn = fnext;
		}
	}
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
	struct ft_pub_rec rec = { .ctx = ctx, .n = 0 };
	struct ft_ord_cell_edge sedges[2] = { 0 };
	unsigned int n_s;
	/*
	 * Did this attempt actually ACQUIRE the SKIP_X dual's grandparent?  The
	 * publish must say so per ATTEMPT, not per call site:
	 * ft_lock_skip_dual_gp returns false on the shapes where it acquires
	 * nothing (no compressed parent, no skip-encoded slot, or a ROOT dual
	 * whose record names no owner at all), and claiming ownership there
	 * hands FT_OWNER_ASSERT_OWNED a NULL owner.
	 */
	bool dual_gp_held;

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
		/*
		 * ☠ RESOLVE THE PARENT BEFORE COPYING IT.  The holder's lock is
		 * held, but a re-home INTO that holder can still be settling:
		 * the peer that published the holder fresh (its born lock) has
		 * already released it while its record on this very cell's
		 * parent still parks a proxy, the descriptor SUCCEEDED.  Copied
		 * raw, the proxy names a record of ANOTHER slot that no settle
		 * rewrites, and the new cell's parent stays a proxy for ever
		 * (MEASURED: inv_prefix_dup_promote_vs_extension at exponential
		 * spacing, the rcu-debug "FT CELL PARENT COPY" abort in 2 of 4
		 * full-suite runs; a probe at this line saw the holder live and
		 * held, the head slot naming @node, and the record re-homing the
		 * head from the compressed node to that holder).  Decided and
		 * holder-held, the resolved value is the parent.
		 */
#ifndef FT_DEBUG_NO_CELL_PARENT_RESOLVE
		void *new_cell_flag = ft_ord_cell_alloc(ft, next_node,
			ft_resolve_flip_proxy(rcu_dereference(old_cell->parent)));
#else
		void *new_cell_flag = ft_ord_cell_alloc(ft, next_node,
			old_cell->parent);
#endif
		struct ft_ord_cell *new_cell;
		struct ft_flip_txn *txn;

		if (!new_cell_flag) {
			if (held_holder)
				ft_meta_lock_release(held_holder);
			return -ENOMEM;
		}
		txn = ft_flip_txn_create_bounded(ft,
			FT_ORD_CELL_SWAP_PUBLISH_MAX_EDGES +
			FT_HLIST_FREEZE_MAX_EDGES + 3 +
			FT_CELL_LOCKSET_MAX_RECORDS);	/* +1 §4.B parent guard, +1 next_node->prev fold,
							 * +1 the SKIP_X dual GP's release-or-guard */
		if (!txn) {
			ft_ord_cell_free_unpublished(ft,
				ft_ord_cell_ptr(new_cell_flag));
			if (held_holder)
				ft_meta_lock_release(held_holder);
			return -ENOMEM;
		}
#ifdef FT_HLIST_CLAIM_PROMOTE
		/*
		 * §4 STEP B4's DRY RUN, the a3c75659 tool aimed at the
		 * EXTERNAL-HEAD lane: point B0's owner assert at this txn so
		 * every record it plants without owning is named at an abort,
		 * on a build otherwise byte-identical to the unarmed one.
		 *
		 * ☞ CLAIMED AT CREATION, EARLIER THAN THE ARM WOULD BE -- so
		 * read a miss as "this record is planted before its owner is
		 * registered" FIRST.  That ORDERING class is what e9268e13
		 * closed for this very lane by hoisting the holder's fence
		 * ABOVE the back edge.
		 *
		 * ☠ And read a miss as "the registry cannot SEE this hold"
		 * before "the op does not HOLD it" (ft_flip_txn_owns).
		 *
		 * ☠☠ THE PROMOTE KNOB IS SEPARATE FROM THE HEAD-CLEAR ONE,
		 * because BOTH promote arms carry a KNOWN EXCLUSION GAP and the
		 * head clear provably does not -- one knob would mask the
		 * armable lane behind the blocked ones.
		 *
		 * THE GAP, measured (ft_unit test_dup_chain_head_promotion, the
		 * 15th test, --enable-rcu-debug): _ft_publish_to_parent_meta
		 * emits TWO structural edges when the holder is skip-encoded --
		 * the forward @head_slot, owned by @parent_nf and REGISTERED by
		 * the fence above, and a SKIP_X DUAL into the GRANDPARENT's
		 * body, owned by a node this op never acquires (in the repro it
		 * is the ROOT node's slot at item+8).  ft_ord_cell_flip_into
		 * gives both edges the dispatching recorder, so an armed txn
		 * would SW-PARK a word the op does not exclude.
		 *
		 * ☠ IT IS NOT SPECIFIC TO THIS ARM -- it is SLOT-SHAPED
		 * ([[feedback_a_site_inventory_cannot_cover_a_dynamic_slot]]).
		 * The LIST-OFF twin below carries it too, and its whole-suite dry
		 * run was CLEAN: a constructed public-API repro aborts it.  A
		 * clean dry run is evidence about the SUITE, not about the lane.
		 * ☞ ft_unchain_node's head clear is the exception, and by
		 * ROUTING rather than by luck -- see the detector there.
		 *
		 * ☞ The fix is per EDGE, not per site: ft_ord_cell_edge already
		 * carries @owner and @root, and the missing third answer is
		 * whether the OP HOLDS that owner -- the same shape
		 * ft_flip_txn_record_parent_word already takes as @child_held.
		 * Where the op does hold it (ft_detach_node's republish holds
		 * the recompact's {P,GP}) the dual is a real conversion, so a
		 * blanket always-MW would give that back.  Which of the two --
		 * carry the flag, or extend these ops' lock-set to the
		 * grandparent -- is a DESIGN call, not this step's to assume.
		 */
		ft_flip_txn_claim_per_op_armable(ft, txn);
#endif
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
		/*
		 * §9.3's THIRD MEMBER: this publish's SKIP_X dual lands in the
		 * GRANDPARENT's body, so acquire GP -- the promote lane never
		 * took it, and a §4.B guard on GP's OWN word does not stand in
		 * for the acquire above per-node spacing.  ☞ ft_lock_skip_dual_gp.
		 * A no-op unless a dual will actually be recorded; @mtxn NULL
		 * mirrors this producer's @rec, which carries none.  Placed with
		 * the holder's fence, ABOVE the arm, because it is a
		 * ft_flip_txn_lock_register and the arm's contract is "after the
		 * op's LAST register".
		 */
		dual_gp_held = ft_lock_skip_dual_gp(ft, ctx, txn, parent_nf,
			txn ? txn->mtxn : NULL);
		/*
		 * ☠ A PRODUCER THAT CANNOT VOUCH MUST NOT REACH THE PUBLISH --
		 * AND THE BAIL MUST DO THE COMMIT'S CLEANUP.
		 *
		 * On a MISS the arm used to build the publish with @dual_gp_held
		 * false -- an MW record on a slot every other producer now parks
		 * SW -- and CALL the commit, relying on @acquire_miss to make it
		 * fail.  That puts this slot's kind at the mercy of a distant
		 * function's ordering, and "the abort makes it harmless" is not
		 * safe to say in any case: an aborted MW txn is not
		 * side-effect-free, its records being installed and then settled
		 * back.
		 *
		 * ☠ A BARE `return -EAGAIN` HERE WEDGES THE TRIE.  The commit is
		 * also this arm's CLEANUP -- ft_flip_txn_commit's miss path runs
		 * ft_flip_txn_destroy, hence ft_flip_txn_lock_release_all -- so
		 * returning without it keeps the op's OWN locks held and the
		 * retry then misses against them for ever.  ABLATION-MEASURED on
		 * inv_concurrent_same_key_removes at per-node spacing: bare bail
		 * = timeout at 100% CPU, control = 152/152 rc=0.  That is why
		 * every other -EAGAIN in this function sits AFTER the commit.
		 *
		 * So release exactly what the commit would.  @held_holder is NOT
		 * released here: ft_flip_txn_hold_or_lock_parent above REGISTERED
		 * it on @txn, so the destroy drops it -- which is why this file
		 * says every bail releasing it explicitly stays ABOVE that call.
		 */
		if (txn->acquire_miss) {
			ft_flip_txn_destroy(txn);
			ft_ord_cell_free_unpublished(ft, new_cell);
			return -EAGAIN;
		}
		ft_flip_txn_record_reserved(txn,
			ft_flag_to_metadata(ft, parent_nf),
			(void **) &next_node->prev,
			next_node->prev, new_cell_flag);
		/*
		 * PHASE B, STEP B4 -- THE ARM.  ft_flip_txn_hold_or_lock_parent
		 * above is this arm's LAST ft_flip_txn_lock_register, and the
		 * publish below plants the first record after it.
		 *
		 * ☞ WHAT UNBLOCKED IT is the per-edge @owner_held: this op holds
		 * the HOLDER (the forward @head_slot's owner) and NOT the
		 * grandparent, so the SKIP_X dual now records MW on its own
		 * account instead of riding the txn's armed mode.  Before that,
		 * arming here would have SW-parked a word the op never acquired
		 * -- reproduced through the PUBLIC API, on a build whose
		 * whole-suite dry run was clean.
		 */
		ft_flip_txn_arm_per_op(ft, txn);
		_ft_publish_to_parent_meta(ft, parent_nf,
			(struct cds_ft_inode_flag **) head_slot,
			(struct cds_ft_inode_flag *) next_node,
			(struct cds_ft_inode_flag *) node,
			/*
			 * ☑ THE HONEST ANSWER, AND IT IS NOW THE ONLY ONE LEFT
			 * THAT CAN SAY NO.  @dual_gp_held is REGISTERED-or-SHARED,
			 * both of which mean the op holds the word.  This used to
			 * add "but every dual producer passes false until the LAST
			 * one acquires, and then they flip TOGETHER", naming
			 * ft_node_recompact as the holdout -- that is discharged:
			 * its relocation site always HELD the word and merely
			 * could not be asked (it answers from its own acquire now),
			 * and the glue publishes are excluded by the FT-wide lock.
			 *
			 * ☞ THESE TWO ARMS -- ft_promote_head's ordered-list-ON
			 * publish (here) and its list-OFF twin below -- WERE the
			 * last producers that could answer no, and they did so
			 * exactly when the acquire MISSED: their MW count equalled
			 * FT DUAL-GP EXIT MISS to the record.  The bail at the
			 * acquire above now takes that case, so this publish is
			 * reached only by an op that HOLDS the word.
			 *
			 * MEASURED AFTER IT (-DFT_DEBUG_DUAL_SITE, ft_inv,
			 * per-node AND exponential): named_unheld = 0 for EVERY
			 * producer, with 6304 misses per per-node leg taking the
			 * bail instead of entering the engine.  ⇒ every record on
			 * this slot class is made by an op that holds it, and that
			 * is provable HERE rather than from what a later commit
			 * does with it.
			 */
			NULL, new_cell_flag, &rec, /*slot_owner_nf=*/ parent_nf,
			/* see above */ dual_gp_held);
		(void) dual_gp_held;
		n_s = ft_pub_rec_sedges(&rec, sedges);
		/*
		 * Fuse @node's freeze (mark node->next, target preserved) into the
		 * swap commit: a reader never sees @node's head anchor promoted away
		 * while @node is still unmarked (doc §4.B).  The reservation above
		 * carries the extra edge.
		 */
		/*
		 * The swap's three cell words are taken inside
		 * ft_ord_cell_swap_publish_multi, which every producer of this
		 * shape reaches -- see its header.
		 */
		ft_ch_audit(ft, txn, node);
		ft_hlist_freeze_prepare(ft, ft_flip_txn_handle(txn), node);
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
				FT_HLIST_FREEZE_MAX_EDGES + 3);	/* +1 §4.B parent guard, +1 prev fold,
								 * +1 the SKIP_X dual GP's release-or-guard */

		void *prev_save;
		void *inherit;

		if (!txn) {
			if (held_holder)
				ft_meta_lock_release(held_holder);
			return -ENOMEM;
		}
#ifdef FT_HLIST_CLAIM_PROMOTE
		/*
		 * §4 STEP B4's DRY RUN.  The SAME knob as the list-on arm above,
		 * because this arm carries the SAME gap -- PROVEN by construction,
		 * not inferred: a 60-line public-API program (list-off trie, four
		 * duplicates of one key, remove the head) aborts at
		 * ft-mutation-helpers.h:3108 on a build whose whole-suite dry run
		 * was CLEAN.  The suite never builds a skip-encoded holder here;
		 * that is a COVERAGE ARTIFACT, not an invariant.
		 */
		ft_flip_txn_claim_per_op_armable(ft, txn);
#endif
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
		/*
		 * §9.3's THIRD MEMBER: this publish's SKIP_X dual lands in the
		 * GRANDPARENT's body, so acquire GP -- the promote lane never
		 * took it, and a §4.B guard on GP's OWN word does not stand in
		 * for the acquire above per-node spacing.  ☞ ft_lock_skip_dual_gp.
		 * A no-op unless a dual will actually be recorded; @mtxn NULL
		 * mirrors this producer's @rec, which carries none.  Placed with
		 * the holder's fence, ABOVE the arm, because it is a
		 * ft_flip_txn_lock_register and the arm's contract is "after the
		 * op's LAST register".
		 */
		dual_gp_held = ft_lock_skip_dual_gp(ft, ctx, txn, parent_nf,
			txn ? txn->mtxn : NULL);
		/*
		 * ☠ A PRODUCER THAT CANNOT VOUCH MUST NOT REACH THE PUBLISH --
		 * AND THE BAIL MUST DO THE COMMIT'S CLEANUP.
		 *
		 * On a MISS the arm used to build the publish with @dual_gp_held
		 * false -- an MW record on a slot every other producer now parks
		 * SW -- and CALL the commit, relying on @acquire_miss to make it
		 * fail.  That puts this slot's kind at the mercy of a distant
		 * function's ordering, and "the abort makes it harmless" is not
		 * safe to say in any case: an aborted MW txn is not
		 * side-effect-free, its records being installed and then settled
		 * back.
		 *
		 * ☠ A BARE `return -EAGAIN` HERE WEDGES THE TRIE.  The commit is
		 * also this arm's CLEANUP -- ft_flip_txn_commit's miss path runs
		 * ft_flip_txn_destroy, hence ft_flip_txn_lock_release_all -- so
		 * returning without it keeps the op's OWN locks held and the
		 * retry then misses against them for ever.  ABLATION-MEASURED on
		 * inv_concurrent_same_key_removes at per-node spacing: bare bail
		 * = timeout at 100% CPU, control = 152/152 rc=0.  That is why
		 * every other -EAGAIN in this function sits AFTER the commit.
		 *
		 * So release exactly what the commit would.  @held_holder is NOT
		 * released here: ft_flip_txn_hold_or_lock_parent above REGISTERED
		 * it on @txn, so the destroy drops it -- which is why this file
		 * says every bail releasing it explicitly stays ABOVE that call.
		 */
		if (txn->acquire_miss) {
			ft_flip_txn_destroy(txn);
			return -EAGAIN;
		}
		ft_flip_txn_record_reserved(txn,
			ft_flag_to_metadata(ft, parent_nf),
			(void **) &next_node->prev, prev_save, inherit);
		/* PHASE B, STEP B4 -- THE ARM.  See the list-on arm above; the
		 * per-edge @owner_held is what makes it legal here too, and this
		 * is the arm whose absence a constructed repro exposed. */
		ft_flip_txn_arm_per_op(ft, txn);
		_ft_publish_to_parent_meta(ft, parent_nf,
			(struct cds_ft_inode_flag **) head_slot,
			(struct cds_ft_inode_flag *) next_node,
			(struct cds_ft_inode_flag *) node,
			NULL, inherit /* folded prev: intended parent value */, &rec,
			/*
			 * ☑ THE HONEST ANSWER, AND IT IS NOW THE ONLY ONE LEFT
			 * THAT CAN SAY NO.  @dual_gp_held is REGISTERED-or-SHARED,
			 * both of which mean the op holds the word.  This used to
			 * add "but every dual producer passes false until the LAST
			 * one acquires, and then they flip TOGETHER", naming
			 * ft_node_recompact as the holdout -- that is discharged:
			 * its relocation site always HELD the word and merely
			 * could not be asked (it answers from its own acquire now),
			 * and the glue publishes are excluded by the FT-wide lock.
			 *
			 * ☞ THESE TWO ARMS -- ft_promote_head's ordered-list-ON
			 * publish (here) and its list-OFF twin below -- WERE the
			 * last producers that could answer no, and they did so
			 * exactly when the acquire MISSED: their MW count equalled
			 * FT DUAL-GP EXIT MISS to the record.  The bail at the
			 * acquire above now takes that case, so this publish is
			 * reached only by an op that HOLDS the word.
			 *
			 * MEASURED AFTER IT (-DFT_DEBUG_DUAL_SITE, ft_inv,
			 * per-node AND exponential): named_unheld = 0 for EVERY
			 * producer, with 6304 misses per per-node leg taking the
			 * bail instead of entering the engine.  ⇒ every record on
			 * this slot class is made by an op that holds it, and that
			 * is provable HERE rather than from what a later commit
			 * does with it.
			 */
			/*slot_owner_nf=*/ parent_nf, /* see above */ dual_gp_held);
		(void) dual_gp_held;
		n_s = ft_pub_rec_sedges(&rec, sedges);
		/* Fuse @node's freeze into the structural publish (doc §4.B). */
		ft_ch_audit(ft, txn, node);
		ft_hlist_freeze_prepare(ft, ft_flip_txn_handle(txn), node);
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

#ifdef FT_ENABLE_TRACING
/*
 * How often the chain arm's DERIVED holder had been re-homed between the
 * unlocked ft_chain_head_holder walk and the acquire -- i.e. how often the
 * re-derive below actually saves a LOST UPDATE.  A fix that never fires is not
 * a fix; this counter is how that claim is checked.
 */
unsigned long ft_dbg_unchain_rehomed;
/*
 * How often the caller's ROUTING (member / promote / clear) had been overtaken
 * by a peer remove on the same chain between the unlocked derivation and the
 * acquire -- the ft_unchain_kind re-validation's firing count.
 */
unsigned long ft_dbg_unchain_rerouted;
#endif

/*
 * The ROUTING DECISION _cds_ft_remove_locked took for @node, derived from
 * node->prev / node->next with NOTHING held, and re-validated by
 * ft_unchain_node under the holder lock before any arm trusts it:
 *   INTERIOR  @node is a non-head duplicate (prev is a node): relink past it.
 *   PROMOTE   @node heads its chain and has a successor: promote the successor.
 *   CLEAR     @node heads its chain with no successor, the holder is an
 *             INTERNAL node and neither the ordered list nor rank stats are on:
 *             clear the head slot.  (A compressed holder's emptied chain is a
 *             DETACH; a list-on / rank-on disappearance is the fused
 *             ft_remove_one_commit -- neither routes here.)
 */
enum ft_unchain_kind {
	FT_UNCHAIN_INTERIOR,
	FT_UNCHAIN_PROMOTE,
	FT_UNCHAIN_CLEAR,
	/*
	 * A CLEAR after the fused collapse DECLINED (a run too long to spell):
	 * the 1-child internal it leaves is the designed product, so the
	 * residue re-route must not send the op round forever.
	 */
	FT_UNCHAIN_CLEAR_KEEP,
};

/*
 * THE COLLAPSE THE PLAN DID NOT SEE, list-off arm.  The caller picked this plain
 * clear over the fused collapse from UNHELD reads of the holder (its nr_child,
 * its parent); a peer that removed the holder's second child -- or was
 * re-parenting it while the plan read -- makes clearing its last external leave
 * a 1-child keyless internal, which cds_ft_verify refuses in skip mode and a
 * later remove beneath it livelocks on (exponential spacing, owned-key row;
 * LTTng: five fused attempts refused on a peer's lock, the sixth took this arm).
 * Under the holder's lock nr_child is exact: re-route, and the retry's plan
 * takes the fused collapse.
 */
static inline
bool ft_unchain_clear_leaves_residue(struct cds_ft *ft,
		enum ft_unchain_kind kind, struct cds_ft_inode_flag *holder)
{
#if defined(FEATURE_FT_SKIP_COMPRESSED) && !defined(FT_DEBUG_PREFIX_CLEAR_RESIDUE)
	struct cds_ft_metadata *hm;

	if (kind != FT_UNCHAIN_CLEAR || !holder ||
	    !ft_group_skip_compressed(ft->group) ||
	    ft_node_external(holder) || ft_node_compressed(holder) ||
	    ft_node_skip_compressed(holder))
		return false;
	hm = ft_flag_to_metadata(ft, holder);
	return ft_meta_nr_child_load(hm) == 1 &&
		ft_parent_node(hm->parent_word) != NULL;
#else
	(void) ft; (void) kind; (void) holder;
	return false;
#endif
}

static
int ft_unchain_node(struct cds_ft *ft, const struct ft_lock_ctx *ctx,
		struct cds_ft_inode_flag *parent_nf, unsigned int parent_depth,
		struct cds_ft_node **head_slot, struct cds_ft_node *node,
		enum ft_unchain_kind kind)
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
		 * ☠ A NULL HOLDER IS A STALE DERIVATION HERE TOO -- the exact twin
		 * of the duplicate-append arm in ft-insert.h, down to the refuted
		 * sentence.  The text that stood here said @node "HAS a holder",
		 * that NULL came "only by ft-insert's unwind paths on UNPUBLISHED
		 * nodes, which never reach an unchain", and cited the SAME count:
		 * "0 NULL in 491532 ft_chain_head_holder calls across ft_unit and
		 * ft_inv's three list modes".  One wrong zero was used to justify
		 * both asserts, and it is wrong for one reason: every oracle behind
		 * that count gives each writer a DISJOINT key range, so no two
		 * writers ever met on one duplicate chain.
		 *
		 * Under LOCK_FINE a same-key cds_ft_insert -- concurrent by
		 * contract -- retires or re-heads this chain between the descent
		 * that produced @node and this derivation, and then
		 * ft_chain_head_holder answers NULL.  MEASURED:
		 * inv_concurrent_same_key_append_nolist trips this assert on an
		 * UNPINNED run (full parallelism is the amplifier here -- pinning
		 * to two cpus HID it).
		 *
		 * So bail retriably, as the two acquire failures just below
		 * already do: the wrapper re-derives against the current tree.
		 * Asserting was not merely loud -- a release build fell through to
		 * ft_flag_to_metadata(ft, NULL) and acquired on the result.
		 */
		if (caa_unlikely(!lock_nf))
			return -EAGAIN;
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
			/*
			 * ★ RE-DERIVE AFTER THE ACQUIRE, for a DERIVED holder.
			 *
			 * @lock_nf came from ft_chain_head_holder (ft-helpers.h),
			 * a walk of @node->prev, taken with NO exclusion held --
			 * and a peer that inserts a key EXTENDING this chain's key
			 * re-homes the head onto a fresh junction in one commit,
			 * retiring nothing (ft-insert.h:2099-2141; the same edit
			 * doc/design/ft-stale-disposal-predicate.md §5.10 roots the
			 * bogus NOT_FOUND in).  Nothing below re-reads the holder,
			 * and the commit "never touches holder->state", so a stale
			 * @lock_nf is not arbitrated anywhere: this op and the next
			 * chain op would hold DIFFERENT words and relink one chain.
			 * The comment above already names the cost -- "once these
			 * become sw it is a LOST UPDATE".
			 *
			 * The engine's own splice-holder acquire does exactly this
			 * check (ft_glue_acquire_splice_holders,
			 * ft-mutation-helpers.h:14748: "re-parented under us: stale
			 * lock" -> -EAGAIN).  Mirrored here, INCLUDING its skip on a
			 * @shared hold: a word the op already held was validated by
			 * the acquire that took it, and re-testing it here would
			 * refuse on an outer frame's still-correct derivation.
			 *
			 * Release only what THIS acquire took (@shared / @txn_owned
			 * belong to another owner -- the predicate ft_unchain_node
			 * already uses for @hmeta above and the remove wrapper uses
			 * on its own exits).
			 */
			if (!parent_nf && !h.shared &&
			    ft_chain_head_holder(ft, node) != lock_nf) {
#ifdef FT_ENABLE_TRACING
				uatomic_inc(&ft_dbg_unchain_rehomed);
#endif
				if (!h.txn_owned)
					ft_meta_lock_release(h.lock);
				return -EAGAIN;
			}
			/*
			 * ★ RE-VALIDATE THE CALLER'S ROUTING, under the lock.
			 * @kind was chosen from node->prev / node->next read with
			 * NOTHING held (see ft_unchain_kind), and the arms below
			 * trust it: the promote arm has no head slot for a member,
			 * and the clear arm is legal only for an internal holder
			 * with the list and rank stats off.  A peer REMOVE on the
			 * SAME chain -- in contract under LOCK_FINE -- moves the op
			 * across those lines between the derivation and this
			 * acquire: it removes the head and @node, a member, is now
			 * the head; or it removes the only other duplicate and a
			 * promote finds no successor.  MEASURED: two writers
			 * removing adjacent duplicates of one key aborted in under
			 * a second in both list modes (the held arm's
			 * assert(parent_nf) for the first; the clear lane's no-dual
			 * detector on a compressed holder for the second, which a
			 * release build would have turned into a compressed node
			 * with a NULL child).
			 * Both words are FROZEN here -- every chain mutation holds
			 * this word, and a commit's lock release settles AFTER its
			 * structural words (ft_flip_txn_commit's late tag) -- so a
			 * mismatch is a fact about the trie: hand the whole op back
			 * and let the wrapper derive once, cleanly, exactly as the
			 * holder re-derive above does.  A hold this op already had
			 * (@h.shared) was validated by the acquire that took it and
			 * is not released here.
			 */
			{
				bool member = ft_node_external(
					(struct cds_ft_inode_flag *)
					rcu_dereference(node->prev));
				bool succ = ft_node_next(node) != NULL;

				/*
				 * A peer removed @node itself since the derivation
				 * (its MARK is on node->next): the interior lane
				 * would otherwise unlink a ghost -- its prev still
				 * names a member, and ft_hlist_del_prepare would
				 * read a marked next as a neighbour.  Hand it back
				 * as gone; the wrapper's re-derivation answers
				 * NOT_FOUND from the top-of-op tombstone test.
				 */
				if (ft_node_is_removed(node)) {
					if (!h.shared && !h.txn_owned)
						ft_meta_lock_release(h.lock);
					return -ENOENT;
				}
				if ((kind == FT_UNCHAIN_INTERIOR) != member ||
				    (kind == FT_UNCHAIN_PROMOTE && !succ) ||
				    ((kind == FT_UNCHAIN_CLEAR ||
				      kind == FT_UNCHAIN_CLEAR_KEEP) && succ) ||
				    ft_unchain_clear_leaves_residue(ft, kind,
						parent_nf)) {
#ifdef FT_ENABLE_TRACING
					uatomic_inc(&ft_dbg_unchain_rerouted);
#endif
					if (!h.shared && !h.txn_owned)
						ft_meta_lock_release(h.lock);
					return -EAGAIN;
				}
				/*
				 * ☠ THE HEAD SLOT IS A PLAN READ TOO.  The caller
				 * found @node in @head_slot before this lock, and
				 * the promote / clear below records that slot with
				 * @node as its expected-old -- a blind SW store now
				 * that the holder is HELD.  A peer insert of a key
				 * extending @node's republishes the slot IN PLACE
				 * under this very lock (a compressed holder's child
				 * becomes a junction carrying @node's chain as its
				 * prefix head), @node itself untouched: the checks
				 * above all pass, and the promote overwrote the
				 * junction, the peer's key with it (MEASURED:
				 * inv_prefix_dup_promote_vs_extension, 16 of 16
				 * runs).  Re-read it here.
				 */
#ifndef FT_DEBUG_NO_HEAD_SLOT_RECHECK
				if (head_slot && kind != FT_UNCHAIN_INTERIOR &&
				    (struct cds_ft_node *) ft_node_ptr(
					ft_resolve_flip_proxy(
					(struct cds_ft_inode_flag *)
					rcu_dereference(*head_slot))) != node) {
					if (!h.shared && !h.txn_owned)
						ft_meta_lock_release(h.lock);
					return -EAGAIN;
				}
#endif
			}
		}
	}

	/*
	 * ☑ IS @node's CHAIN RETIRED, though @node itself is not?  Asked HERE --
	 * after the lock_fine acquire block and before any edge is derived --
	 * because this is the one point where the prev walk is stable in BOTH
	 * writer modes: FINE holds the chain holder by now, COARSE holds the
	 * FT-wide writer lock throughout.
	 *
	 * ☠ INSIDE the `if (ft->lock_fine)` block it does not run under COARSE at
	 * all, and COARSE then has NO protection: a whole-chain displacement
	 * freezes only the HEAD (a txn is bounded, a chain is not), so a member
	 * of a displaced chain is retired and UNMARKED, the interior lane derives
	 * @pred from a stale prev, and the edge it records has an expected-old
	 * the slot can never hold -- 50001 laps and a memcg kill.  Measured:
	 * inv_concurrent_insert_replace_coarse died at ft_inv test 152.
	 *
	 * ☠ AND AT THE ENTRY ARM IT IS UNHELD, which is worse than useless: the
	 * walk then catches the window where a head remove has marked H but not
	 * yet relinked the members onto the promoted successor, and calls a chain
	 * that is merely losing its head "retired" -- a FALSE NOT_FOUND that
	 * LOSES A KEY (inv_concurrent_same_key_removes, count_keys 65 != 64).
	 * ft_chain_head_is_removed says HOLDER-LOCK CALLERS ONLY for this reason.
	 *
	 * -ESTALE, not -ENOENT: the wrapper routes -EAGAIN/-ENOENT to RETRY, and
	 * a retry only terminates if the ENTRY arm can answer the same question,
	 * which it cannot.  -ESTALE is definitive and answers NOT_FOUND.
	 */
	if (ft_chain_head_is_removed(node)) {
		if (hmeta)
			ft_meta_lock_release(hmeta);
		return -ESTALE;
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
		ft_ch_audit(ft, txn, node);
		if (ft_hlist_del_prepare(ft, ft_flip_txn_handle(txn), node)) {
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
		struct ft_pub_rec rec = { .ctx = ctx, .n = 0 };
		struct ft_ord_cell_edge sedges[2] = { 0 };
		struct ft_flip_txn *txn;
		unsigned int n_s;

		txn = ft_flip_txn_create_bounded(ft, FT_PUB_SEDGE_MAX_EDGES +
			FT_HLIST_FREEZE_MAX_EDGES +
			1 /* §4.B parent guard */ +
			1 /* the SKIP_X dual GP's release-or-guard */);
		if (!txn) {
			/* Early fence held but not yet handed to the txn. */
			if (hmeta)
				ft_meta_lock_release(hmeta);
			return -ENOMEM;
		}
#ifdef FT_HLIST_CLAIM
		/*
		 * §4 STEP B5's DRY RUN.  Its OWN knob, apart from the two
		 * promote arms, because this lane does NOT carry their SKIP_X
		 * dual gap and a shared knob would hide that behind their abort.
		 */
		ft_flip_txn_claim_per_op_armable(ft, txn);
#endif
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
		/*
		 * §9.3's third member, for the same reason as the promote arms.
		 * A NO-OP today by the routing the detector below asserts -- a
		 * compressed holder never reaches the head clear, so no dual is
		 * recorded and the helper's own condition returns early -- and
		 * wired anyway, because that detector exists precisely because
		 * "a ROUTING INVARIANT IS A CODE FACT, AND CODE MOVES".
		 * ☞ ft_lock_skip_dual_gp.
		 */
		/*
		 * ☞ KEEP THE ANSWER even though this lane emits no dual today.
		 * It costs nothing, and the day the routing above changes the
		 * publish will already be spelling its own exclusion instead of
		 * a constant that nobody re-examines.  Measured: ft_unchain_node
		 * has NO row in the FT_DEBUG_DUAL_SITE census, which is the
		 * detector's claim restated as a number.
		 */
		{
			bool dual_gp_held = ft_lock_skip_dual_gp(ft, ctx, txn,
				parent_nf, txn ? txn->mtxn : NULL);

			_ft_publish_to_parent(ft, parent_nf,
				(struct cds_ft_inode_flag **) head_slot, NULL,
				(struct cds_ft_inode_flag *) node, &rec,
				dual_gp_held);
		}
		n_s = ft_pub_rec_sedges(&rec, sedges);
		/*
		 * THIS LANE EMITS NO SKIP_X DUAL, and the arm below depends on
		 * it, so a DETECTOR says so rather than a comment.
		 *
		 * The dual arm of _ft_publish_to_parent is gated on
		 * ft_node_compressed(@parent_nf), and this branch is only
		 * reachable with @parent_nf NULL (interior relink) or INTERNAL:
		 * _cds_ft_remove_locked routes a COMPRESSED holder by whether
		 * @node has a successor -- none goes to ft_detach_node, one or
		 * more goes to the PROMOTE arm above -- so the head clear never
		 * sees a compressed holder, and the remaining call sites pass an
		 * internal holder_flag.  (The branch comment naming "a compressed
		 * holder's SKIP_X dual" is stale.)
		 *
		 * ☠ A ROUTING INVARIANT IS A CODE FACT, AND CODE MOVES.  The two
		 * promote arms sit one page away and DO emit the dual into a
		 * grandparent they never acquire; a clean dry run did not catch
		 * that there, so this lane's arm is not resting on one either.
		 * Armed under --enable-rcu-debug, which the gate runs at three
		 * spacings across four configs.
		 */
		urcu_assert_debug(n_s < 2);
		/*
		 * PHASE B, STEP B5 -- THE ARM.  ft_flip_txn_hold_or_lock_parent
		 * above is this lane's LAST ft_flip_txn_lock_register (its held
		 * arm registers @hmeta and records the release; its unheld arm
		 * acquires-or-guards @parent_nf), and the publish below plants
		 * the first record after it.
		 *
		 * ☞ WHAT IT CONVERTS is the head-slot clear, owned by
		 * @parent_nf -- the word the fence above holds.  The detector
		 * beside it is what makes that the WHOLE of what it converts:
		 * with no dual, the publish is a lone structural edge and there
		 * is no second owner to answer for.
		 *
		 * ☞ READINESS: -DFT_HLIST_CLAIM claims at txn CREATION,
		 * strictly earlier, and is clean on ft_unit and ft_inv
		 * FT_INV_MW=1 both.
		 */
		ft_flip_txn_arm_per_op(ft, txn);
		ft_ch_audit(ft, txn, node);
		ft_hlist_freeze_prepare(ft, ft_flip_txn_handle(txn), node);
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

#ifdef FT_ENABLE_TRACING
/* How often a NOT_FOUND was withdrawn because the holder had moved. */
unsigned long ft_dbg_rm_stale_holder;
#endif

/*
 * ☠ A REFUSAL IS A CLAIM ABOUT THE TRIE, NOT A RETURN CODE.
 *
 * The four-way branch below picks its arm from @holder_flag, derived from
 * @node->prev far above any mark.  When a peer re-homes the head in that
 * window the branch tests the WRONG node's words, the arm's identity compare
 * fails, and the op answers CDS_FT_STATUS_NOT_FOUND -- a PERMANENT, key-losing
 * answer to a TRANSIENT race.  MEASURED: at that exact return a fresh
 * ft_node_holder(@node) says the node IS its holder's head and the holder is
 * alive and untombstoned.
 *
 * So before conceding, ask the question again.  If the holder moved, the
 * refusal was about a node this op no longer has any business reading: hand it
 * back to the op's own retry (@need_retry, the escape the dead-forward-holder
 * arm already uses) instead of reporting the key gone.
 *
 * ☞ HISTORICAL, AND STILL LOAD-BEARING -- READ IT WITH ITS RESOLUTION.  What
 * follows is what this disposition measured WITH THE OLD PREDICATE (a bare
 * pointer double-read), which is why it was opt-in (-DFT_RM_HOLDER_RECHECK)
 * and DEFAULT OFF.  It is now DEFAULT ON, because the predicate -- not the
 * retry -- was the defect; the resolution is at ft_rm_holder_rehomed() below,
 * and each objection recorded here is answered inline.  A/B over 20 seeds of
 * the two-writer reproducer, identical source, only this arm differing:
 *
 * ☠☠☠ AND MULTI-WRITER STRESS SAYS IT IS NOT A FIX.  MEASURED 2026-09-06:
 *
 *   - ft_unit 331/331 and ft_inv LIST-OFF 128/128 pass, but ft_inv LIST-ON and
 *     the FT_INV_MW concurrent-writer leg BOTH HANG at test 44
 *     (inv_insert_replace_splice_window, rc=124 after `ok 43`) -- the very test
 *     FT_RM_ACQUIRE_FIRST hangs, by the same alternation moved up to the
 *     wrapper: the tombstone arm runs a DESCENT precisely when @node->prev is
 *     lazily stale, so @holder_flag is descent-derived while @fresh here is
 *     prev-derived, the two disagree PERMANENTLY, every attempt reports
 *     "moved", and the op never converges.  Test 44 serialises its writers, so
 *     that hang is never contention.
 *
 *     ☑ RESOLVED, and this paragraph is its own proof: the permanently
 *     disagreeing population is EXACTLY the one where @node->prev is lazily
 *     stale -- which means the holder it names is TOMBSTONED, which is WHY the
 *     tombstone arm ran at all.  ft_rm_holder_rehomed() returns false on a
 *     tombstoned @fresh, so that population never retries and the alternation
 *     cannot form.  Gate with it default ON: 17 configs, ft_inv on/off/mw
 *     128/128 everywhere, test 44 included, plus five configs at all three
 *     lock spacings.
 *
 *   - ☠ AND THE OBVIOUS GATE TRADES THE CURE FOR THE HANG.  Adding
 *     `!have_descent &&` here (ask @prev only about a holder that came from
 *     @prev) clears BOTH hangs -- ft_unit 331/331, ft_inv on/off/mw 128/128,
 *     all rc=0 -- and RM-FAIL then reads 21/30 on the two-key reproducer
 *     against 21/30 for the control.  THE CURE IS GONE.  So the cure and the
 *     hang are ONE code path: this arm withdraws the refusal only by retrying
 *     on exactly the condition that does not always terminate.
 *
 *     ☞ Which is why a bare double-read cannot be the fix.  It re-reads
 *     @node->prev with NO LOCK HELD, so it stabilises nothing and compares
 *     across two derivations.  A sound version must take the candidate's lock
 *     FIRST and re-read @prev under it -- prev against prev, never prev
 *     against a descent -- so that "unchanged" means "cannot change".
 *
 *     ☠ THAT PRESCRIPTION WAS REFUTED BY MEASUREMENT (§5.20c): a load -> lock
 *     -> re-load of ft_node_holder() fires ZERO times in 9.7 M evaluations
 *     (ft_unit + ft_inv on + mw + the reproducer), positive-controlled to the
 *     unit.  For the !have_descent population the back edge simply never
 *     moves; for the descent population the compare is unequal BY
 *     CONSTRUCTION.  A lock cannot stabilise what was never moving.  What the
 *     predicate needed was not stability but the LIVENESS of @fresh.
 *
 *     RM-FAIL (the shape it targets)   11/20  ->   1/20
 *     SEGV                              2/20  ->   2/20   (unchanged: it does
 *                                                          NOT create these)
 *     timeout or memcg kill             0/20  ->   5/20   ☠
 *
 * So the diagnosis is confirmed -- withdrawing the refusal all but eliminates
 * the shape -- and handing the op back to @need_retry SPINS: the caller's loop
 * is an unbounded for(;;), and under sustained churn @node->prev keeps moving,
 * so each retry re-derives a holder that has moved again.  -DFT_DEBUG_REMOVE_
 * RETRY_CAP does NOT catch it (it reports at 100/1000/10000 attempts and the
 * arm never passes 100); the failure is wall-clock and RSS, not attempt count.
 *
 * ☞ WHAT IT NEEDS: a BOUNDED disposition -- re-derive and re-run the arm
 * selection IN PLACE a bounded number of times, rather than restarting the op
 * -- or a bound on the outer loop.  Conceding NOT_FOUND after a bound is still
 * strictly better than today, where the wrong answer is unconditional.  That is
 * a contract question (may a remove refuse?) and is not settled here.
 */
/*
 * ★ WHAT THE BARE DOUBLE-READ LACKED WAS A PREDICATE, NOT A BOUND.
 *
 * Everything measured above stands, and it indicts the PREDICATE.
 * `fresh != holder_flag` folds TWO causes together and only one is transient:
 *
 *  - @fresh is TOMBSTONED.  The back edge is merely STALE: it names a RETIRED
 *    holder the forward descent above already replaced.  That is PERMANENT --
 *    no retry changes it -- and it is UNCONDITIONAL for the whole
 *    tombstone-arm population, because that arm deliberately replaces
 *    @holder_flag with a descent result while @node->prev goes on naming the
 *    retired one.  ☠ THAT is the 5/20 wall-clock/RSS spin recorded above: the
 *    old predicate fired on every single lap.  So no bound was ever needed --
 *    testing liveness removes the spinning population outright.
 *
 *  - @fresh is LIVE.  A peer RE-HOMED @node under a different, live holder: an
 *    insert of a key that EXTENDS @node's converted its leaf into a PREFIX HEAD
 *    in place, chaining @node into the new head's external_nodes (ft-insert.h,
 *    ft_attach_node -- "the external node we are replacing at the attachment
 *    location ... chain this external node in the topmost internal node
 *    external node list").  @node is LIVE and REACHABLE one hop lower, so
 *    refusing is the §1 key loss.  This IS transient, and it is charged to that
 *    peer's COMMITTED conversion: the retry re-derives against a tree that has
 *    already moved on, reads @node->prev = the head (live, so the tombstone arm
 *    does not fire), and the branch reaches @node by its own external-member
 *    condition.
 *
 * MEASURED with the liveness test in (doc/design/ft-stale-disposal-predicate.md
 * §5.24): RM-FAIL 2/30 -> 0/30, and 0 across 90 further seeds -- 120 seeds,
 * ZERO.  The spin did NOT return: timeouts flat at 9/30 against a baseline of
 * 8/30, suite walls unchanged (unit 45 s, ion 86, ioff 86, imw 103).
 *
 * ☞ A genuinely unlinked node is a REAL miss and must NOT retry: tested first.
 */
static inline
bool ft_rm_holder_rehomed(struct cds_ft *ft, const struct cds_ft_node *node,
		struct cds_ft_inode_flag *holder_flag)
{
	struct cds_ft_inode_flag *fresh;

	if (ft_node_is_removed((struct cds_ft_node *) node))
		return false;		/* genuinely gone: the miss is real */
	fresh = ft_node_holder(ft, (struct cds_ft_node *) node);
	if (!fresh || fresh == holder_flag)
		return false;
	if (ft_flag_tombstoned(ft, fresh))
		return false;		/* stale back edge, not a re-home */
#ifdef FT_ENABLE_TRACING
	uatomic_inc(&ft_dbg_rm_stale_holder);
#endif
	return true;
}

/*
 * The other transient miss: the slot came BACK.  @fresh == @holder_flag says
 * "not re-homed", but it cannot tell a miss from an ABA on the slot itself.
 * A peer insert of a key EXTENDING @node republishes @head_slot with a junction
 * carrying @node as its prefix head; that peer's remove then collapses the
 * junction and puts @node back into the SAME holder's slot.  An identity read
 * that landed in between saw the junction, and the back edge -- re-pointed and
 * restored by those two commits -- names @holder_flag again.  @node never left
 * the trie.  MEASURED: two writers toggling "...zone46" / "...zone4" through a
 * compressed holder, 3 of 5 runs refused the live "...zone4" here within 10 s.
 *
 * A live @node back in @head_slot is a retry, charged to the peer's two
 * committed writes; a removed one is still the real miss.  A NULL @head_slot
 * -- the body arm's byte had no slot at all -- has nothing to come back to.
 *
 * ☠ AND ONLY IN A LIVE HOLDER.  A RETIRED holder's slots keep whatever they
 * held when it was retired, so its slot "holds @node" forever: a remove
 * bootstrapped from a stale back edge naming that corpse would retry on it
 * without end (MEASURED: a lost "...zone46" whose remove spun alone at 100%
 * CPU after its peer exited).  ft_rm_holder_rehomed already calls a
 * tombstoned holder the real miss; this must agree with it.
 */
static inline
bool ft_rm_slot_regained(const struct cds_ft *ft, const struct cds_ft_node *node,
		struct cds_ft_inode_flag *holder_flag,
		struct cds_ft_inode_flag **head_slot)
{
#ifdef FT_DEBUG_NO_SLOT_REGAINED
	(void) ft; (void) node; (void) holder_flag; (void) head_slot;
	return false;
#else
	if (!head_slot || ft_node_is_removed((struct cds_ft_node *) node) ||
			ft_flag_tombstoned(ft, holder_flag))
		return false;
	return (const struct cds_ft_node *) ft_node_ptr(ft_resolve_flip_proxy(
			rcu_dereference(*head_slot))) == node;
#endif
}

/*
 * ☠ A MISS IS CONFIRMED UNDER THE HOLDER'S LOCK.  The identity compare and the
 * two predicates above are UNLOCKED reads, each answered at its own instant,
 * and a peer toggling a key that EXTENDS @node can answer each one from a
 * different tree: the compare sees the peer's junction in the slot, the back
 * edge (read next) sees that junction already collapsed and @node back under
 * @holder_flag, and the slot (read last) sees the peer's NEXT junction.  Three
 * self-consistent reads, one wrong verdict: NOT_FOUND for a key that never left
 * the trie (MEASURED: 5 of 32 two-writer runs, "...runs" vs "...runs6" through
 * one compressed holder, refused the live "...runs" at the compressed arm; a
 * lookup found it both before the remove and after the refusal).
 *
 * THE AUTHORITY IS THE KEY'S SLOT IN THE HOLDER, READ UNDER ITS LOCK -- not
 * @node's back edge, which is updated LAZILY and can name a RETIRED holder
 * while the forward path still reaches @node (the STALE BACK-EDGE arm in
 * _cds_ft_remove_locked).  Holding @lock_flag's lock, the slot cannot move:
 *
 *  - it holds @node again                     -> retry (the slot came back);
 *  - it holds an INTERNAL node (skip-encoded  -> retry: the key now goes
 *    included: a skip word's low bits read       DEEPER, through a junction
 *    as external, so ask that first)             a peer published here, and
 *                                                 @node may hang below it;
 *  - it is empty, or holds a DIFFERENT leaf   -> @node is not at its key's
 *                                                 position: the miss is real,
 *    unless the back edge names a LIVE other holder (a re-home this compare
 *    cannot see), which is a retry.
 *
 * The slot is re-derived under the lock (@key_byte, or the compressed
 * @lock_flag's child when negative): a packed body shifts its slots in place,
 * so a pre-lock slot address is itself a plan read.  Taken and dropped here,
 * with nothing built or reserved: this is not the op-wide hold measured and
 * refuted above.
 *
 * Returns true when the op must retry.
 */
static inline
bool ft_rm_miss_unconfirmed(struct cds_ft *ft, const struct ft_lock_ctx *lctx,
		const struct cds_ft_node *node,
		struct cds_ft_inode_flag *holder_flag,
		struct cds_ft_inode_flag *lock_flag,
		struct cds_ft_metadata *lock_meta, unsigned int holder_depth,
		int key_byte)
{
#ifdef FT_DEBUG_NO_MISS_UNDER_LOCK
	(void) ft; (void) lctx; (void) node; (void) holder_flag;
	(void) lock_flag; (void) lock_meta; (void) holder_depth; (void) key_byte;
	return false;
#else
	struct ft_held_anchor h = { 0 };
	struct cds_ft_inode_flag *cur, **slot = NULL, *fresh;
	bool retry;

	if (!ft->lock_fine ||
			ft_node_is_removed((struct cds_ft_node *) node))
		return false;
	/*
	 * A RETIRED holder decides nothing: its slots keep what they held, and
	 * @node may live on in its replacement behind a back edge that is only
	 * LAZILY updated.  Re-derive: the retry's STALE BACK-EDGE arm descends
	 * for the live holder by key, and a miss is then confirmed under THAT
	 * holder's lock -- a live word, so the retry terminates.
	 */
	if (ft_flag_tombstoned(ft, lock_flag))
		return true;
	/*
	 * Refused: a peer holds the word, or retired it after the test above.
	 * Either way the tree is moving under the verdict: re-derive.
	 */
	if (ft_acquire_member(ft, lctx, lock_flag, lock_meta, holder_depth, &h))
		return true;
	if (ft_node_is_removed((struct cds_ft_node *) node)) {
		retry = false;
	} else {
		if (key_byte < 0)
			cur = ft_compressed_node_ptr(lock_flag)->child;
		else
			cur = ft_node_get_nth_skip(holder_flag, &slot,
				(uint8_t) key_byte, FT_PF_NONE);
		cur = ft_resolve_flip_proxy(cur);
		if (cur && (ft_node_skip_compressed(cur) ||
				!ft_node_external(cur) ||
				(const struct cds_ft_node *) ft_node_ptr(cur) ==
					node)) {
			retry = true;
		} else {
			fresh = ft_node_holder(ft, (struct cds_ft_node *) node);
			retry = fresh && fresh != holder_flag &&
				!ft_flag_tombstoned(ft, fresh);
		}
	}
	if (!h.shared && !h.txn_owned)
		ft_meta_lock_release(h.lock);
	return retry;
#endif
}

/*
 * Drop the hoisted holder mark on every exit of _cds_ft_remove_locked.
 * ☞ The arms DEDUPE against it (ft_dlm_acquire_set_at -> ft_lock_ctx_holds)
 * rather than taking their own, so no arm's commit terminal releases it -- this
 * is the only release, and it must therefore run on EVERY path.  Guarded the
 * way ft_detach_node's orphan sweep is: never touch a mark this op no longer
 * owns (@shared) or has handed to a txn (@txn_owned).
 */
#if defined(FT_RM_ACQUIRE_FIRST) && defined(FT_RM_REVALIDATE)
# error "FT_RM_ACQUIRE_FIRST and FT_RM_REVALIDATE are alternative dispositions of the same defect; enable at most one"
#endif

#if defined(FT_RM_ACQUIRE_FIRST) || defined(FT_RM_REVALIDATE)
/*
 * @committed: did THIS op's commit run?  Most uses are bails, retries and
 * re-aims where nothing committed and the fence is provably still ours; the
 * success arm of the exit switch is the one that is not, and it must not ask
 * the word -- a committed terminal may have dropped our LOCK, after which a
 * set bit means a PEER's fence.
 */
#define FT_RM_RELEASE_OUTCOME(committed_)	do {			\
		if (rm_hold && !rm_held.shared && !rm_held.txn_owned) {	\
			if (committed_) {				\
				FT_ORPHAN_CONSUMED();			\
				FT_ORPHAN_PREMISE_CHECK(rm_held.lock);	\
			} else {					\
				FT_ORPHAN_RELEASED();			\
				ft_meta_lock_release(rm_held.lock);	\
			}						\
		}							\
		rm_hold = false;					\
	} while (0)
#define FT_RM_RELEASE()	FT_RM_RELEASE_OUTCOME(false)
#else
#define FT_RM_RELEASE()			do { } while (0)
#define FT_RM_RELEASE_OUTCOME(c_)	do { } while (0)
#endif

static
enum cds_ft_status _cds_ft_remove_locked(struct cds_ft *ft,
		struct cds_ft_iter *iter,
		struct cds_ft_node *node,
		bool *need_retry,
		struct ft_op *op)
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
		CDS_FT_ASSERT_CALLER_RCU_READ_LOCKED(ft);
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
	 * alive; node->prev names the node's holder and the slot that holds
	 * @node can be derived from it directly:
	 *
	 * ☠ BUT NOT BECAUSE A WRITER MUTEX FREEZES ANYTHING, and this comment
	 * used to say it did -- "the writer mutex held here freezes the
	 * structure, so node->prev is a settled live pointer".  That was true
	 * under CDS_FT_WRITER_LOCK_COARSE and is FALSE under the fine-grained
	 * default, where cds_ft_remove's own header promises it may run
	 * concurrently with another remove ON THE SAME KEY: there is no mutex
	 * here, and a back-pointer is stale at rest, unbounded.  A false
	 * premise on a CONVERTED path is worse than an unconverted op that
	 * says so, because nothing marks it as owed.
	 *
	 * ☞ WHAT ACTUALLY CARRIES IT is the derivation being RE-VALIDATED under
	 * the holder's lock once taken, with a RETRIABLE bail when the holder
	 * moved -- ft_chain_head_holder's re-check and ft_unchain_kind's
	 * under-lock routing, plus the txn's own expected-old.  The derivation
	 * here is a PLAN, not a conclusion.
	 *
	 * ☐ AND THE ALTERNATIVE DISPOSITIONS OF THIS SAME DEFECT ARE DEAD CODE:
	 * the FT_RM_ACQUIRE_FIRST and FT_RM_REVALIDATE arms below are reachable
	 * from no build -- neither macro is defined anywhere in the tree, so
	 * their measurements ("48/160 seeds") describe a path nothing compiles.
	 * Decide them or delete them; left as-is they read like live coverage.
	 */
	/*
	 * The slot derivation itself:
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
#if defined(FT_ENABLE_TRACING) || defined(FT_DEBUG_RM_SITE)
		ft_dbg_rm_site = __LINE__;
#endif
		return CDS_FT_STATUS_NOT_FOUND;
	}
	/*
	 * ...AND A MEMBER WHOSE PREDECESSOR NO LONGER OWNS IT IS EQUALLY GONE,
	 * with no tombstone anywhere to say so.
	 *
	 * The mark above is a PER-NODE fact, and a whole-chain displacement
	 * cannot write it on every member: cds_ft_insert_replace swings the
	 * anchor slot to a fresh head and freezes only the OLD HEAD in that
	 * commit, because a txn is bounded and a chain is not.  Asking the head
	 * instead does not work either -- the head is a CALLER-OWNED node, and
	 * re-arming it for reuse (cds_ft_node_init, which the API permits the
	 * moment the chain is handed back) erases the very mark the question
	 * reads.
	 *
	 * So ask the structure, not a flag: an interior member is in a live
	 * chain iff its predecessor still points AT it.  A displaced member's
	 * @prev names a node that has been recycled or relinked, whose next is
	 * NULL or someone else -- exactly the condition under which the interior
	 * lane would record `pred->next: elem -> next` against a slot that can
	 * never hold @elem, and spin forever (50001 attempts, then a memcg kill;
	 * five single-threaded calls reproduce it).
	 *
	 * Both loads are proxy-resolved and mark-masked, so a peer mid-commit
	 * reads as the word before or after it, never as a descriptor: a peer
	 * unlinking @pred has already relinked ITS predecessor at @node, and a
	 * peer removing @node itself is caught by the mark above.  Unheld, like
	 * the mark test it follows -- both are statements about the node the
	 * CALLER handed us, and the answer is NOT_FOUND either way.
	 */
	{
		void *nprev = ft_dereference_prev_resolved(node);

		/*
		 * ☠ AND THE TWO LOADS ARE NOT MUTUALLY CONSISTENT, so the
		 * predicate needs a THIRD one.  @node->prev and @pred->next are
		 * read unheld and a PROMOTION writes both: a same-key
		 * cds_ft_remove that retires the head promotes @node in its
		 * place, clearing the old head's next and re-homing @node->prev
		 * onto the flagged parent.  Read @node->prev BEFORE that
		 * re-home and @pred->next AFTER the clear and the structure
		 * says "your predecessor does not point at you" about a node
		 * that is not displaced at all -- it is the new HEAD.
		 *
		 * MEASURED, 3/3 runs of inv_concurrent_same_key_append_nolist:
		 * at the refusal @pred->next was NULL while an immediate
		 * re-read of @node->prev already carried the flagged parent
		 * (tag != external), with no tombstone on @node.  The op then
		 * answered NOT_FOUND -- a TERMINAL claim -- for a LIVE node,
		 * sometimes one that still had successors of its own.
		 *
		 * ☠ WHY A FALSE NOT_FOUND IS NOT A HARMLESS REFUSAL: the caller
		 * is entitled to conclude the node left the trie and to re-arm
		 * it (cds_ft_node_init, which the API permits the moment the
		 * chain is handed back).  That wipes prev/next on a node STILL
		 * LINKED, and the next cds_ft_insert of it descends onto
		 * ITSELF: ft_chain_head_holder walks a NULL prev, the append
		 * bails -EAGAIN and re-descends onto the same node forever
		 * (measured: 230,977,777 retries in ONE call, inside the
		 * escalation fallback, which then stalls every grace period).
		 *
		 * So RE-VALIDATE @node->prev after the @pred->next load and
		 * refuse only when it is UNCHANGED: a genuine displacement is
		 * STABLE (nobody relinks a displaced member), while a promotion
		 * moves @node->prev off the predecessor exactly when it clears
		 * the predecessor's next.  A prev that moved under us means the
		 * node was relinked, not dropped -- fall through and let
		 * ft_node_holder re-derive it from the word it now carries.
		 */
		if (nprev &&
		    ft_node_external((struct cds_ft_inode_flag *) nprev) &&
		    ft_hlist_next_rcu((struct cds_ft_node *) nprev) != node &&
		    ft_dereference_prev_resolved(node) == nprev) {
			dbg_printf("cds_ft_remove: node %p is not in its predecessor's chain\n",
				node);
			FT_TP(remove_exit, (int) CDS_FT_STATUS_NOT_FOUND);
#if defined(FT_ENABLE_TRACING) || defined(FT_DEBUG_RM_SITE)
			ft_dbg_rm_site = __LINE__;
#endif
			return CDS_FT_STATUS_NOT_FOUND;
		}
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
#if defined(FT_ENABLE_TRACING) || defined(FT_DEBUG_RM_SITE)
		ft_dbg_rm_site = __LINE__;
#endif
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
		struct cds_ft_inode_flag *fwd;
		unsigned int fwd_depth;
		bool prefix;

		ft_anchor_descend(ft, &d, iter_key, key_len, &ik);
		/*
		 * ★ THE DESCENT STOPS IN ONE OF TWO PLACES, and this arm knew
		 * only one of them.  These are the SAME two bullets the coarse
		 * arm below states and applies (its `prefix` test): the key can
		 * end AT an internal node whose external_nodes carry the leaf --
		 * a PREFIX KEY -- and then the holder is @d.nf at @d.depth, not
		 * @d.pnf.
		 *
		 * ☠ TAKING @d.pnf UNCONDITIONALLY IS A KEY LOSS.  A peer
		 * inserting a key that EXTENDS @node's converts @node's leaf
		 * into a PREFIX HEAD in place, retiring nothing, and chains
		 * @node into that head's external_nodes (ft-insert.h,
		 * ft_attach_node).  @node stays LIVE, one hop lower.  Aiming at
		 * the head's PARENT sends the four-way branch into the
		 * body-slot arm, whose slot then holds the HEAD and not @node,
		 * and it answers a PERMANENT NOT_FOUND for a key that exists.
		 *
		 * MEASURED (doc/design/ft-stale-disposal-predicate.md §5.20,
		 * §5.21): at that refusal @d.nf IS the head in 12,450 of 12,451
		 * samples (0 were @node), and this `prefix` test fires on every
		 * failing sample, 2,226/2,226.  Selecting @d.nf also makes the
		 * op acquire the word its writes actually touch -- the head's,
		 * not its parent's.
		 */
		prefix = d.nf && !ft_node_external(d.nf) &&
			d.depth == key_len;
		fwd = prefix ? d.nf : d.pnf;
		fwd_depth = prefix ? d.depth : d.pdepth;
		/*
		 * ☞ A DEAD FORWARD HOLDER IS A RETRY, NOT A MISS -- the second
		 * half of the same two-arm inconsistency.  The coarse arm below
		 * answers this very predicate with *need_retry ("nothing is
		 * reserved or published yet, so re-derive the whole position
		 * against the settled tree"); this arm answered it with a
		 * PERMANENT NOT_FOUND.  A tombstone on the freshly descended
		 * holder means a peer is retiring it RIGHT NOW: transient, and
		 * the contract forbids refusing for a transient reason.
		 * MEASURED: 2 of the residual refusals left by this guard and
		 * BOTH were tombstoned(fwd); neither was !d.nf (§5.23).
		 */
		if (fwd != NULL && ft_flag_tombstoned(ft, fwd)) {
			FT_DBG_RETRY_SITE();
			*need_retry = true;
			return CDS_FT_STATUS_OK;	/* wrapper retries */
		}
		/* Only an unreachable key is an idempotent miss. */
		if (!d.nf || fwd == NULL) {
			FT_TP(remove_exit, (int) CDS_FT_STATUS_NOT_FOUND);
#if defined(FT_ENABLE_TRACING) || defined(FT_DEBUG_RM_SITE)
			ft_dbg_rm_site = __LINE__;
#endif
			return CDS_FT_STATUS_NOT_FOUND;
		}
		holder_flag = fwd;
		holder_depth = fwd_depth;
		have_descent = true;
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

#ifdef FT_DEBUG_WIDEN_OWNER
	if (!have_descent && ft_bulk_active(ft))
		uatomic_inc(&ft_wo_nod_remove_locked);
#endif
	ft_lock_ctx_init(&lctx, have_descent ? &d : NULL, NULL, op);

#ifdef FT_RM_ACQUIRE_FIRST
	/*
	 * ★ ACQUIRE, THEN SELECT.  Everything below -- the cell capture, the
	 * four-way branch and its two identity compares -- reads words that
	 * belong to @holder_flag, which was derived from @node->prev with no
	 * exclusion held.  A peer that re-homes the head in that window makes
	 * the branch test the WRONG node and answer NOT_FOUND for a key that
	 * exists, and the contract forbids refusing for a transient reason.
	 *
	 * ☠ RETRYING DOES NOT FIX IT, MEASURED: handing the op back to
	 * @need_retry is a PRE-COMMIT BAIL, which forfeits the op's lane turn,
	 * so each restart re-derives against a still-moving tree -- 5 runs in
	 * 20 died on wall-clock or the memcg.  The exclusion has to be HELD
	 * ACROSS derive -> select -> compare, which is what this does.
	 *
	 * WHY IT CONVERGES.  The peer that re-homes @node is a holder copier:
	 * it holds H's FT_STATE_LOCK from mark to commit and retires H in that
	 * same commit.  So once the acquire succeeds (it refuses a TOMBSTONE)
	 * AND ft_node_holder(@node) still answers H when re-read AFTER the CAS,
	 * nothing can move H: a publish into it fails the §4.B guard, and a
	 * copy of it needs the lock we hold.  The branch then reads FROZEN
	 * words, so a mismatch below is a fact about the TRIE -- which is a
	 * refusal the contract allows.
	 *
	 * The arms re-acquire the same holder; ft_dlm_acquire_set_at DEDUPES
	 * against the op's held set (ft_lock_ctx_holds), which is why this is
	 * published into @lctx and why the hoist costs NO extra CAS -- every
	 * arm already took this lock, one step later.
	 *
	 * ☠☠☠ BROKEN — DO NOT ENABLE.  MEASURED: ft_inv HANGS deterministically
	 * in test 44 (inv_insert_replace_splice_window); gdb on the hung process
	 * puts the spinning thread in the ft_anchor_descend below.  THE LOOP
	 * ALTERNATES: the descent finds the LIVE holder, the acquire takes it,
	 * and then the validation asks @node->prev who the holder is -- which
	 * still names the RETIRED one, because a back-pointer is updated LAZILY
	 * -- so it mismatches, releases, re-aims at the DEAD holder, refuses it
	 * as a TOMBSTONE, descends again, forever.
	 *
	 * ★ THE FIX DIRECTION, and it is the same lesson the STALE BACK-EDGE
	 * note above already teaches: THE FORWARD PATH IS AUTHORITATIVE.  The
	 * validation must ask whether @holder_flag is still the node's holder
	 * BY DESCENT, not by ft_node_holder(@node->prev).  Every use of the
	 * back-pointer as an authority in this function has now been wrong
	 * twice.
	 *
	 * ☐ PER-NODE SPACING ONLY.  A coarse anchor needs a byte-depth for the
	 * re-derived holder and this loop has none; coarse keeps today's
	 * behaviour.  Per-node is the default and where the defect is measured.
	 */
	struct ft_held_anchor rm_held = { 0 };
	bool rm_hold = false;

	if (ft->lock_fine &&
	    holder_flag && !ft_node_external(holder_flag)) {
		for (;;) {
			struct cds_ft_inode_flag *fresh;

			if (ft_acquire_member(ft, &lctx, holder_flag,
					ft_flag_to_metadata(ft, holder_flag),
					holder_depth, &rm_held)) {
				/*
				 * ☠☠ -EAGAIN FOLDS THREE FACTS INTO ONE CODE:
				 * a peer LOCK, a mid-flip PROXY, or a TOMBSTONE.
				 * Only the first two are transient.  "A TOMBSTONE
				 * IS PERMANENT, NOT CONTENTION" -- and this file
				 * already MEASURED what treating it as contention
				 * costs: 2,000,000+ consecutive attempts with 11
				 * of 12 writers parked (the STALE BACK-EDGE note
				 * above, which is the SAME bug at the SAME
				 * derivation).  Re-deriving from @node->prev
				 * cannot escape it -- a back-pointer is updated
				 * LAZILY and goes on naming the retired holder --
				 * so the loop would refuse the same dead word
				 * forever.
				 *
				 * THE FORWARD PATH IS AUTHORITATIVE.  On a dead
				 * holder re-derive by the key-guided descent, the
				 * cure that arm already applies, and carry on with
				 * the live parent.  A descent that no longer
				 * reaches @node means the key really is gone --
				 * a fact about the TRIE -- so the branch's own
				 * compare may then legally answer NOT_FOUND.
				 */
				if (ft_flag_tombstoned(ft, holder_flag)) {
					const uint8_t *ik2 = iter_key;

					ft_anchor_descend(ft, &d, iter_key,
							key_len, &ik2);
					if (!d.nf || !d.pnf ||
					    ft_flag_tombstoned(ft, d.pnf))
						break;	/* gone: the branch refuses */
					holder_flag = d.pnf;
					holder_depth = d.pdepth;
					have_descent = true;
					continue;
				}
				/* A peer LOCK/PROXY: genuinely in progress. */
				FT_DBG_RETRY_SITE();
				*need_retry = true;
				FT_RM_RELEASE();
				return CDS_FT_STATUS_OK;
			}
			fresh = ft_node_holder(ft, node);
			if (fresh == holder_flag) {
				rm_hold = true;
				lctx.held.extra = &rm_held;
				lctx.held.nr_extra = 1;
				break;
			}
			/* It moved while we were taking it: drop and re-aim. */
			if (!rm_held.shared && !rm_held.txn_owned)
				ft_meta_lock_release(rm_held.lock);
			rm_held = (struct ft_held_anchor){ 0 };
			if (!fresh || ft_node_external(fresh))
				break;	/* no state word to hold: as before */
			/*
			 * ☞ @holder_depth is NOT re-derived here.  Harmless
			 * under the PER_NODE gate above (every member anchors
			 * on itself, the depth unused); it becomes a MIS-ANCHOR
			 * the moment that gate widens, so widening it means
			 * routing this through the descent too.
			 */
			holder_flag = fresh;
		}
	}
#endif

#ifdef FT_RM_REVALIDATE
	/*
	 * ★ REVALIDATE UNDER THE LOCK, AND RETRY THE WHOLE OP IF IT MOVED.
	 *
	 * THE DEFECT (root-caused, doc/design/ft-stale-disposal-predicate.md
	 * §5.10): everything below -- the cell capture, the four-way branch and
	 * its two identity compares -- reads words belonging to @holder_flag,
	 * which was derived from @node->prev at the top of this function with NO
	 * exclusion held.  The comment there still says "the writer mutex held
	 * here freezes the structure", which was true under
	 * CDS_FT_WRITER_LOCK_COARSE and is FALSE under the fine-grained default.
	 * So the arm is selected from TWO UNLOCKED READS TAKEN IN DIFFERENT
	 * EPOCHS, and a peer that inserts a key EXTENDING @node's converts
	 * @node's leaf into a PREFIX HEAD in place on the live holder -- the slot
	 * edge and @node's back-edge flipping in ONE commit (ft-insert.h:2099-2141)
	 * -- so the compare below tests the fresh internal node against @node and
	 * answers NOT_FOUND for a key that is present throughout.
	 *
	 * MEASURED: 48/160 seeds of the rig, 0/160 under
	 * CDS_FT_WRITER_LOCK_COARSE (which restores exactly the premise above).
	 *
	 * WHY THIS CONVERGES, and why it is NOT the refuted blind retry.  A lap
	 * happens only when a peer's commit is OBSERVED to have moved the holder,
	 * so every lap is charged to a distinct peer commit rather than to an
	 * attempt count -- the refusal-without-a-fact the contract forbids.  And
	 * the re-derivation is CORRECT rather than lazily stale: the conversion
	 * parks @node's back-edge into the SAME one-commit as the forward publish
	 * ("must flip ATOMICALLY with the forward publish", ft-insert.h:2129-2135),
	 * so once that commit lands, @node->prev names the NEW holder and the next
	 * attempt selects the right arm.
	 *
	 * ☠ WHY IT IS NOT FT_RM_ACQUIRE_FIRST, which hangs test 44
	 * DETERMINISTICALLY.  That disposition RE-AIMS IN-LOOP: on a mismatch it
	 * assigns @holder_flag = fresh and continues, while its tombstone arm
	 * re-aims from a DESCENT.  The two derivations disagree whenever a
	 * back-pointer is lazily stale, so the loop alternates -- descent finds
	 * the live holder, the validation re-aims at the dead one, the acquire
	 * refuses it as a TOMBSTONE, descend again, forever.  THERE IS NO LOOP
	 * HERE.  Derivation and validation both read @node->prev, so they can
	 * only disagree because a peer committed in between, and the answer to
	 * that is to hand the whole op back and derive once, cleanly.
	 *
	 * A TOMBSTONED holder is NOT handled here: the re-derivation runs the
	 * ft_node_is_removed / tombstone recovery arm at the top of this
	 * function, which descends for a live holder.  That is why a retry
	 * terminates on a dead word instead of refusing it forever.
	 *
	 * ☐ PER-NODE SPACING ONLY.  A coarser anchor needs a byte-depth for the
	 * holder and this path derives none (ft-remove.h:5653-5657: it becomes a
	 * MIS-ANCHOR the moment that gate widens).  Per-node is the default and
	 * is where the defect is measured.
	 *
	 * ☠☠☠ MEASURED AND REFUTED AS WRITTEN — DO NOT ENABLE.  41 seeds of the
	 * rig (same workload as §5.9/§5.10) against the flag OFF control:
	 *
	 *     rc=124 (HANG)   20 / 41      control ~10 / 160
	 *     rc=137 (memcg)   7 / 41
	 *     rc=134 (abort)   8 / 41  -- RM-FAIL 4, STALE-FOUND 5: NOT cured
	 *     rc=20 / rc=21    2 / 2   -- READER-VISIBLE regressions the control
	 *                                 does not produce at this rate: a HARD
	 *                                 miss (a key LOST) and a wrong identity
	 *     runs reaching their final line: 0 / 41
	 *
	 * ☞ WHY, and it is a fact about the DISPOSITION, not a coding slip.  The
	 * hold is published into @lctx and therefore spans the WHOLE op, through
	 * the arms and the detach.  Under -DFEATURE_FT_HOLD_TRACE the ledger says
	 * so directly: peers are refused the word repeatedly --
	 * "FT REFUSED (LOCK, unknown holder): ft_insert_dlm_acquire_split:778"
	 * three times on ONE word in each of three seeds, plus refusals at
	 * ft_node_recompact:1405, ft_chain_compress_fused:1315 and at
	 * _cds_ft_remove_locked:5726 itself.  Every peer INSERT that needs the
	 * holder's word is blocked for the duration of a remove -- which is the
	 * FT-wide writer lock re-created one node at a time, with none of its
	 * ordering.  ☞ This is also the seam rule's territory: no NODE lock may
	 * be held across a grace period (ft_seam_check, ft-mutation-helpers.h:2592,
	 * called from ft_writer_lock_gp_wait).
	 *
	 * ★ WHAT THIS DOES NOT REFUTE: revalidating under the lock.  The
	 * measurement indicts the SCOPE of the hold, not the revalidation.  The
	 * untried refinement is to hold ONLY across derive -> select -> compare
	 * and release immediately after the compare, WITHOUT publishing into
	 * @lctx -- letting each arm take its own word as it does today, and
	 * letting the txn's expected-old catch a move that happens after the
	 * decision.  That is a different disposition and is NOT measured.
	 */
	struct ft_held_anchor rm_held = { 0 };
	bool rm_hold = false;

	if (ft->lock_fine &&
	    holder_flag && !ft_node_external(holder_flag)) {
		if (ft_acquire_member(ft, &lctx, holder_flag,
				ft_flag_to_metadata(ft, holder_flag),
				holder_depth, &rm_held)) {
			/*
			 * -EAGAIN folds LOCK / PROXY / TOMBSTONE.  All three are
			 * answered the same way: nothing was acquired, so nothing
			 * is released, and the re-derivation above sorts a dead
			 * holder out through its own recovery arm.
			 */
			FT_DBG_RETRY_SITE();
			*need_retry = true;
			return CDS_FT_STATUS_OK;	/* value unused: wrapper retries */
		}
		if (ft_node_holder(ft, node) != holder_flag) {
			/*
			 * It moved between the derive and the lock.  Retry the
			 * WHOLE op rather than re-aim: see the alternation above.
			 */
			if (!rm_held.shared && !rm_held.txn_owned)
				ft_meta_lock_release(rm_held.lock);
			FT_DBG_RETRY_SITE();
			*need_retry = true;
			return CDS_FT_STATUS_OK;	/* value unused: wrapper retries */
		}
		/*
		 * Held across derive -> select -> compare.  Published into @lctx so
		 * the arms DEDUPE against it (ft_dlm_acquire_set_at ->
		 * ft_lock_ctx_holds) instead of taking their own: no arm's commit
		 * terminal releases it, so FT_RM_RELEASE is the only release and
		 * must run on every exit below.
		 */
		rm_hold = true;
		lctx.held.extra = &rm_held;
		lctx.held.nr_extra = 1;
	}
#endif

	/*
	 * ★ DERIVE -> SELECT -> COMPARE window (-DFT_DELAY_INJECT only; a no-op
	 * in every shipping build, fractal-trie-internal.h:4236).
	 *
	 * @holder_flag is now fully derived and NOTHING is held.  Every word the
	 * four arms below read -- the cell capture, the kind dispatch and the two
	 * identity compares -- belongs to that holder, and a peer INSERT of a key
	 * that EXTENDS @node's converts @node's leaf into a PREFIX HEAD in place
	 * on it, flipping the holder's slot and @node's back-edge in one commit
	 * while retiring nothing (doc/design/ft-stale-disposal-predicate.md
	 * §5.10).  Measured on the wire, that window is TEN NANOSECONDS wide
	 * (§5.12), which is why the defect needs 80 seeds of a concurrent rig to
	 * show up 53 times.
	 *
	 * Placed AFTER the whole derivation and BEFORE the first read of a
	 * holder-owned word, i.e. exactly at the epoch boundary the defect
	 * straddles.
	 *
	 * ☠ MEASURED NOT TO HELP FOR THIS DEFECT, recorded so the next reader
	 * does not repeat it.  Against the rig's DETERM=1 case, 10 seeds each:
	 *
	 *     no delay        13/20 fired, mean 253 ms
	 *     FT_DELAY_US=1    6/10
	 *     FT_DELAY_US=5    5/10
	 *     FT_DELAY_US=20   5/10
	 *     FT_DELAY_US=200  3/5, two runs clean for a full 10 s
	 *
	 * Widening the window makes it WORSE, and the reason is instructive: the
	 * peer's insert of the extension and its own removal both fit inside a
	 * sleep, so the holder's slot is back to @node by the time the compare
	 * runs.  The race is self-healing under a long enough delay.  What buys
	 * the speed is the KEY SHAPE (DETERM=1), not the timing.
	 */
	ft_delay_writer();

	/*
	 * Cell-always: @node heads its chain iff its prev is the cell (not an
	 * external predecessor).  Capture the head's cell + successor BEFORE the
	 * unlink: a head promotion retargets the cell at the successor (done in
	 * ft_unchain_node), and a key disappearance (no successor) frees the
	 * cell after the removal commits (ret == 0).
	 */
	/*
	 * ONE load of node->prev, RESOLVED, and the successor through the hlist
	 * resolver: nothing is held here, and a peer remove on the same chain
	 * can have a flip proxy parked on either word (its promote records the
	 * successor's prev; its member unlink records the successor's next and
	 * the predecessor's next).  Two raw loads of prev could disagree with
	 * each other and with the resolved holder derivation above -- and a raw
	 * ft_ord_cell_ptr() of a parked proxy is a wild cell.  Resolved, each
	 * value is the word before or after that peer's commit, never a
	 * descriptor; a STALE-but-valid routing is what ft_unchain_node's
	 * under-lock re-validation (ft_unchain_kind) catches and retries.
	 */
	void *node_prev = ft_dereference_prev_resolved(node);
	/*
	 * ONE read of the successor, for BOTH decisions that depend on it:
	 * the lane dispatch below (a head with a successor is promoted, one
	 * without is pruned / cleared) and the fuse decision here (a key that
	 * disappears takes its cell with it).  Read twice, at two instants,
	 * the two disagreed under a same-key peer: a successor appended
	 * between the reads sent a "sole entry" (@fuse_remove, unsplice txn
	 * reserved) down the PROMOTE lane, which swapped the head's cell for
	 * a fresh one -- and the two-commit tail then drove the unsplice of
	 * the swapped-out cell forward forever (its expected-olds can never
	 * match; measured as a memcg kill within seconds).  A successor
	 * removed between the reads did the mirror image: a key pruned with
	 * its cell left in the list (ft_verify "ord-cell list longer than
	 * trie").  Staleness of this ONE read is what the lanes then catch:
	 * the promote's under-lock re-validation (ft_unchain_kind) and the
	 * sole-entry freeze's derived expected-old both retry the op.
	 */
	struct cds_ft_node *succ_node = ft_hlist_next_rcu(node);
	bool cell_was_head = ft->ordered_list &&
		!ft_node_external((struct cds_ft_inode_flag *) node_prev);
	struct ft_ord_cell *dead_cell = cell_was_head ?
		ft_ord_cell_ptr(node_prev) : NULL;
	struct cds_ft_node *cell_succ = cell_was_head ? succ_node : NULL;
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
			FT_RM_RELEASE();
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
		ret = ft_unchain_node(ft, &lctx, NULL, 0, NULL, node,
			FT_UNCHAIN_INTERIOR);
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
			/*
			 * RE-HOMED UNDER A LIVE HOLDER -> RETRY THE WHOLE OP.
			 * See ft_rm_holder_rehomed: the liveness test is what
			 * makes this terminate where the bare double-read spun.
			 */
			if (ft_rm_holder_rehomed(ft, node, holder_flag) ||
					ft_rm_slot_regained(ft, node, holder_flag,
						head_slot) ||
					ft_rm_miss_unconfirmed(ft, &lctx, node,
						holder_flag,
						ft_compressed_node_flag(cn),
						holder_meta, holder_depth, -1)) {
				FT_DBG_RETRY_SITE();
				*need_retry = true;
				FT_RM_RELEASE();
				return CDS_FT_STATUS_OK;
			}
			FT_TP(remove_exit, (int) CDS_FT_STATUS_NOT_FOUND);
#if defined(FT_ENABLE_TRACING) || defined(FT_DEBUG_RM_SITE)
			ft_dbg_rm_site = __LINE__;
#endif
			FT_RM_RELEASE();
			return CDS_FT_STATUS_NOT_FOUND;
		}
		if (!succ_node) {
			/*
			 * Last/only entry: prune the now-empty branch.
			 * ft_detach_node bootstraps from the holder slot (recovered
			 * from the holder's own metadata offset) and walks up via
			 * metadata->parent.  Propagate -1 before detach, which may
			 * free internal nodes.
			 */
			/*
			 * NAME THE DERIVATION.  The freeze this detach fuses is
			 * given a LITERAL length, so it never passes through
			 * ft_hlist_chain_len and the tail-why classifier filed it
			 * as "(unstamped)" -- a bucket that has to be reasoned
			 * into a site instead of read off the table.  The literal
			 * is derived HERE, from the @succ_node read above, so
			 * stamp it here.
			 */
			FT_HLIST_WHY_STAMP();
			ret = ft_detach_node(ft, &lctx, head_slot,
				ft_get_parent_slot(holder_meta, ft),
				key_len, true, fuse_cell, pubp, NULL, NULL, node,
				1 /* @node is this key's SOLE entry */,
				-1 /* leaf key removed: detach owns the -1 */,
				NULL, false, NULL, NULL,
				/* EXPERIMENT: delete tier open, for the trace. */
				ft_in_place_delete_ok(ft));
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
				(struct cds_ft_node **) head_slot, node,
				FT_UNCHAIN_PROMOTE);
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
		if (!succ_node) {
			/*
			 * Last entry: the external chain empties, so the prefix
			 * key disappears (the holder KEEPS its longer-key children
			 * -- prefix-with-siblings).
			 */
			bool last_fused = false, fused_declined = false;
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
				/*
				 * INTENT: no body child is detached -- the entry
				 * being removed is the EXTERNAL one, and this arm
				 * is the one whose branch condition established
				 * that the head IS @node.  So @node is what the
				 * callee must still find under the mark.
				 */
				const struct ft_chain_compress_intent intent = {
					.detach_child = NULL,
					.expect_ext = node,
					.detach_byte = 0,
				};

				cret = ft_chain_compress_fused(ft,
					holder_flag, holder_depth, &lctx,
					holder_meta,
					s_child, s_byte,
					1 /* sole body child; the removed entry is external */,
					fuse_cell, NULL,
					NULL, 0, NULL, NULL, 0 /* no orphan chain */, node,
					1 /* @node is this key's SOLE entry */,
					-1, ft->rank_stats ? key_len + 1 : 0,
					NULL, false,
					NULL /* no pending publish */, &intent, NULL);

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
				if (cret > 0)
					fused_declined = true;
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
						FT_RM_RELEASE();
						return CDS_FT_STATUS_MEMORY_ERROR;
					}
					/* VALIDATE (§4.B): lock (or guard-fallback) the
					 * LIVE holder whose external_nodes this single-node
					 * clear empties -- value-swap target (§10.5). */
					ft_flip_txn_lock_or_guard_parent(ft, txn, &lctx,
					holder_flag, holder_depth);
#ifndef FT_DEBUG_RM_PREFIX_MISS_CARRIES_ON
					/*
					 * A MISSED TAKE ENDS THE ATTEMPT HERE.  Carrying on
					 * recorded the count walk, took @dead_cell's lock-set
					 * and built the unsplice and freeze edges into a txn
					 * ft_flip_txn_commit can only discard -- 9 records per
					 * miss, 21-43k misses per ft_inv leg.  Run that discard
					 * now: the same call ft_ord_cell_flip_into ends in, so
					 * the same aging, the same errno and the same exit.
					 */
					if (caa_unlikely(txn->acquire_miss)) {
						ret = ft_flip_status_to_errno(
							ft_flip_txn_commit(ft, txn));
					} else
#endif
#if defined(FEATURE_FT_SKIP_COMPRESSED) && !defined(FT_DEBUG_PREFIX_CLEAR_RESIDUE)
					/*
					 * THE COLLAPSE THE PLAN DID NOT SEE.  The fused arm
					 * above is chosen from an UNHELD nr_child: this plan
					 * saw a second child, so a plain clear would leave a
					 * legal keyless node with two.  A peer that removed that
					 * second child before our take changes the answer:
					 * clearing the last external of a 1-child node leaves
					 * the 1-child keyless internal cds_ft_verify refuses in
					 * skip mode, and the post-prune below is a separate
					 * flip that loses under contention -- the
					 * residue a later remove beneath it livelocks on
					 * (exponential spacing, owned-key row).  Under the
					 * holder's lock nr_child is exact: re-plan, and the
					 * next attempt takes the fused collapse.
					 */
					if (!fused_declined &&
					    ft_group_skip_compressed(ft->group) &&
					    ft_meta_nr_child_load(holder_meta) == 1 &&
					    ft_parent_node(holder_meta->parent_word) !=
							NULL) {
						ft_flip_txn_destroy(txn);
						ret = -EAGAIN;
					} else
#endif
					{
						ft_flip_txn_record_count_parent(ft, txn,
							holder_flag, -1);
						ret = ft_remove_one_commit(ft,
							(struct cds_ft_inode_flag **) &holder_meta->external_nodes,
							holder_meta,
							(struct cds_ft_inode_flag *) node, NULL,
							NULL, dead_cell, NULL, txn, node, false);
					}
					if (ret == 0 && fuse_remove)
						pub.armed = true;
				} else {
					/* List off + rank stats off: unchanged; no count. */
					ret = ft_unchain_node(ft, &lctx, holder_flag,
						holder_depth,
						(struct cds_ft_node **) &holder_meta->external_nodes,
						node, fused_declined ?
							FT_UNCHAIN_CLEAR_KEEP :
							FT_UNCHAIN_CLEAR);
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
				node, FT_UNCHAIN_PROMOTE);
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
			/*
			 * RE-HOMED UNDER A LIVE HOLDER -> RETRY THE WHOLE OP.
			 * See ft_rm_holder_rehomed: the liveness test is what
			 * makes this terminate where the bare double-read spun.
			 */
			if (ft_rm_holder_rehomed(ft, node, holder_flag) ||
					ft_rm_slot_regained(ft, node, holder_flag,
						head_slot) ||
					ft_rm_miss_unconfirmed(ft, &lctx, node,
						holder_flag, holder_flag,
						holder_meta, holder_depth,
						iter_key[key_len - 1])) {
				FT_DBG_RETRY_SITE();
				*need_retry = true;
				FT_RM_RELEASE();
				return CDS_FT_STATUS_OK;
			}
			FT_TP(remove_exit, (int) CDS_FT_STATUS_NOT_FOUND);
#if defined(FT_ENABLE_TRACING) || defined(FT_DEBUG_RM_SITE)
			ft_dbg_rm_site = __LINE__;
#endif
			FT_RM_RELEASE();
			return CDS_FT_STATUS_NOT_FOUND;
		}
		if (!succ_node) {
			/*
			 * Last/only entry: prune the now-empty branch.
			 * Propagate -1 before detach, which may free internal nodes.
			 */
			/* Name the derivation: see the compressed-holder arm above. */
			FT_HLIST_WHY_STAMP();
			ret = ft_detach_node(ft, &lctx, head_slot,
				ft_get_parent_slot(holder_meta, ft),
				key_len, true, fuse_cell, pubp, NULL, NULL, node,
				1 /* @node is this key's SOLE entry */,
				-1 /* leaf key removed: detach owns the -1 */,
				NULL, false, NULL, NULL,
				/* EXPERIMENT: delete tier open, for the trace. */
				ft_in_place_delete_ok(ft));
			/* @node's freeze rode the detach commit (freeze_leaf). */
		} else {
			/* Removing the head, duplicates remain: key count unchanged. */
			ret = ft_unchain_node(ft, &lctx, holder_flag, holder_depth,
				(struct cds_ft_node **) head_slot, node,
				FT_UNCHAIN_PROMOTE);
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
		FT_RM_RELEASE_OUTCOME(true);
		return CDS_FT_STATUS_OK;
	case -ENOMEM:
		FT_TP(remove_exit, (int) CDS_FT_STATUS_MEMORY_ERROR);
		FT_RM_RELEASE();
		return CDS_FT_STATUS_MEMORY_ERROR;
	case -ESTALE:
		/*
		 * @node's CHAIN left the trie (a whole-chain displacement
		 * freezes only its head), established UNDER THE HOLDER by
		 * ft_unchain_node.  Definitive, so it answers NOT_FOUND rather
		 * than joining the retry cases below: re-deriving would find the
		 * same retired chain every lap.
		 */
		FT_TP(remove_exit, (int) CDS_FT_STATUS_NOT_FOUND);
#if defined(FT_ENABLE_TRACING) || defined(FT_DEBUG_RM_SITE)
		ft_dbg_rm_site = __LINE__;
#endif
		FT_RM_RELEASE();
		return CDS_FT_STATUS_NOT_FOUND;
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
		FT_DBG_RETRY_SITE();
#ifdef FT_DEBUG_REMOVE_RETRY_CAP
		/*
		 * ★ WHICH of the two.  The exit fuses -EAGAIN (a peer won a
		 * commit, or a lock acquire refused) with -ENOENT (the derived
		 * position emptied / retyped pre-commit).  They have different
		 * cures -- one is contention, the other a derivation that can be
		 * STABLY dead -- so a livelock here is undiagnosable until they
		 * are counted apart.
		 */
		if (ret == -EAGAIN)
			ft_dbg_rm_eagain++;
		else
			ft_dbg_rm_enoent++;
#endif
		*need_retry = true;
		FT_RM_RELEASE();
		return CDS_FT_STATUS_OK;	/* value unused: wrapper retries */
	default:
		abort();
	}
}

#ifdef FT_DEBUG_REMOVE_RETRY_CAP
# include <stdio.h>
# include <stdlib.h>
# ifndef FT_REMOVE_RETRY_CAP
#  define FT_REMOVE_RETRY_CAP	50000
# endif
/*
 * The wall-clock threshold for the per-op "FT REMOVE SLOW" diagnostics below,
 * in nanoseconds.  0 compiles them out entirely, leaving only the RETRY CAP's
 * livelock abort.
 *
 * This knob exists because the two existing modes cannot express what a GATE
 * LEG needs.  The gate arms this detector for ONE reason: a remove LIVELOCK
 * must abort loudly instead of expiring as a timeout, because rc=124 cannot be
 * told apart from CPU contention on a box running 25 legs at once.  It does not
 * want the per-op tail data.  And -DFT_REMOVE_TAIL_QUIET cannot silence that
 * data: it deliberately keeps the >1ms metric, which is the metric D.3's A/B
 * measured (doc/design/mw-to-fine-locking-remainder.md §D.3 -- the milestone
 * dump is what biases a control arm, not this).  MEASURED on the imw leg of
 * this very config: 9,799 SLOW pairs and 1.9 MB of stderr in the first 30 s,
 * which drowns the TAP stream the gate greps for its verdict.
 *
 * Default 1 ms, i.e. byte-identical to what every existing user compiled.
 */
# ifndef FT_REMOVE_BUSY_ATTEMPTS
/* 0 disables; a remove retrying more than this reports its breakdown. */
#  define FT_REMOVE_BUSY_ATTEMPTS	200
# endif
# ifndef FT_REMOVE_SLOW_NS
#  define FT_REMOVE_SLOW_NS	1000000
# endif
#endif

enum cds_ft_status cds_ft_remove(struct cds_ft *ft,
		struct cds_ft_iter *iter,
		struct cds_ft_node *node)
{
	struct ft_op optxn;
	struct ft_op_retry op_retry;
	enum cds_ft_status s;
	bool need_retry;
#ifdef FT_DEBUG_REMOVE_RETRY_CAP
	unsigned int ft_remove_attempts = 0;
	uint64_t ft_remove_t0 = ft_dbg_now_ns();
	struct rusage ft_remove_ru0;

	getrusage(RUSAGE_THREAD, &ft_remove_ru0);
	uint64_t ft_acc_begin = 0, ft_acc_body = 0, ft_acc_bail = 0;
	uint64_t ft_tA, ft_tB, ft_tC;
#ifdef CDS_FAIR_MUTEX_DBG_POLL
	unsigned long ft_fmtx0 = cds_fmtx_dbg_polls;
	unsigned long ft_wfcq0 = cds_wfcq_dbg_polls;
#endif

	ft_dbg_acq_dirty_lock = 0;
	ft_dbg_acq_dirty_other = 0;
	ft_dbg_acq_cabort = 0;
	ft_dbg_gp_ns = 0;
	ft_dbg_gp_calls = 0;
	ft_dbg_arena_ns = 0;
	ft_dbg_arena_waits = 0;
#endif

	CDS_FT_SCOPED_WRITER(ft);
	if (caa_unlikely(ft_bulk_active(ft))) {
		/* See ft_iter_redescend_node: the gate is set, so re-derive. */
		s = ft_iter_redescend_node(ft, iter, node);
		if (s != CDS_FT_STATUS_OK)
			return s;
	}
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
	 * manually on the PERSISTENT @optxn via ft_op_conflict: after
	 * its lane budget of conflicts this writer escalates into
	 * the per-trie FIFO fair-mutex lane -- every writer's begin() honors
	 * domain->active, so the lane drains the contention and the retry
	 * terminates (no livelock).  Exclusive trie: the bracket opens nothing
	 * and no conflict ever fires.
	 */
	FT_SH_STALL_DECL;

	ft_op_retry_init(&op_retry, FT_OP_REMOVE, NULL, 0);
	ft_txn_op_init(ft, &optxn);
	for (;;) {
		need_retry = false;
		ft_op_retry_tick(ft, &op_retry, 0);
		FT_SH_STALL_TICK(ft, "cds_ft_remove");
#ifdef FT_DEBUG_REMOVE_RETRY_CAP
		ft_tA = ft_dbg_now_ns();
#endif
		ft_op_begin(&optxn);
#ifdef FT_DEBUG_REMOVE_RETRY_CAP
		ft_tB = ft_dbg_now_ns();
		ft_acc_begin += ft_tB - ft_tA;
#endif
#ifdef FT_DLM_LINGER
		ft_linger_word = NULL;	/* only THIS attempt's refusal counts */
#endif
		s = _cds_ft_remove_locked(ft, iter, node, &need_retry, &optxn);
#ifdef FT_DEBUG_REMOVE_RETRY_CAP
		ft_tC = ft_dbg_now_ns();
		ft_acc_body += ft_tC - ft_tB;
#endif
		if (!need_retry)
			break;
#ifdef FT_DEBUG_REMOVE_RETRY_CAP
		++ft_remove_attempts;
#ifndef FT_REMOVE_TAIL_QUIET
		if (caa_unlikely(ft_remove_attempts == 100 ||
				ft_remove_attempts == 1000 ||
				ft_remove_attempts == 10000)) {
			fprintf(stderr, "FT REMOVE RETRY TAIL: attempts=%u "
				"retry=%lu in_lane=%d(th %d) "
				"lane_published=%d active=%d "
				"dirtyLOCK=%u dirtyOTHER=%u cabort=%u\n",
				ft_remove_attempts, optxn.retry,
				optxn.in_lane, ft_op_in_lane(),
				optxn.lane_published,
				optxn.lane ? (int) uatomic_load(
					&optxn.lane->active, CMM_RELAXED)
					: -1,
				ft_dbg_acq_dirty_lock, ft_dbg_acq_dirty_other,
				ft_dbg_acq_cabort);
			fprintf(stderr, "FT REMOVE RETRY WHY: eagain=%lu "
				"enoent=%lu\n", ft_dbg_rm_eagain,
				ft_dbg_rm_enoent);
#ifdef FT_ENABLE_TRACING
			/*
			 * The traced build's milestone IS the violation: emit
			 * the self-diagnosing event, persist the ring, crash
			 * before the window scrolls out.
			 */
			if (ft_remove_attempts == 100 && ft_dbg_last_refused) {
				const struct cds_ft_metadata *m =
					ft_dbg_last_refused;
				const struct ft_dbg_take_slot *sl =
					ft_dbg_take_slot_of(m);
				int match = (sl->meta == m);

				FT_TP(remove_anchor_starved, (const void *) m,
					(unsigned long) CMM_LOAD_SHARED(
						m->state),
					ft_dbg_refused_streak,
					ft_remove_attempts,
					match ? sl->fn : "?",
					match ? sl->line : 0,
					match ? sl->op_bound : -1,
					match ? (unsigned long)
						((ft_dbg_now_ns() - sl->ts_ns)
							/ 1000) : 0);
				(void) system("lttng snapshot record 1>&2");
				abort();
			}
#else /* !FT_ENABLE_TRACING */
			/*
			 * Sample the refused word for 200us: is this ONE hold
			 * spanning the victim's episode, or a churn of takes?
			 * The registry names the taker; arena-backed metadata
			 * stays mapped, so a racy read cannot fault.
			 */
			if (ft_dbg_last_refused) {
				const struct cds_ft_metadata *m =
					ft_dbg_last_refused;
				const struct ft_dbg_take_slot *sl =
					ft_dbg_take_slot_of(m);
				uint64_t t0 = ft_dbg_now_ns(), tnow;
				unsigned int held = 0, total = 0, trans = 0;
				int prev_locked = -1;

				do {
					uintptr_t sw = CMM_LOAD_SHARED(
						m->state);
					int locked = !!(sw & FT_STATE_LOCK);

					total++;
					held += locked;
					if (prev_locked >= 0 &&
							locked != prev_locked)
						trans++;
					prev_locked = locked;
					caa_cpu_relax();
					tnow = ft_dbg_now_ns();
				} while (tnow - t0 < 200000);
				if (sl->meta == m) {
					fprintf(stderr, "FT REMOVE RETRY "
						"HOLDER: streak=%u "
						"held=%u/%u trans=%u "
						"taker=%s:%d op_bound=%d "
						"tid=%lx take_age_us=%llu "
						"phase=%d phase_age_us=%llu\n",
						ft_dbg_refused_streak,
						held, total, trans,
						sl->fn, sl->line,
						sl->op_bound, sl->tid,
						(unsigned long long)
						(t0 - sl->ts_ns) / 1000,
						sl->phase_line,
						sl->phase_ns ?
						(unsigned long long)
						(t0 - sl->phase_ns) / 1000 : 0);
				} else {
					fprintf(stderr, "FT REMOVE RETRY "
						"HOLDER: streak=%u "
						"held=%u/%u trans=%u "
						"taker=UNKNOWN (slot %p vs "
						"%p)\n",
						ft_dbg_refused_streak,
						held, total, trans,
						(void *) sl->meta,
						(void *) m);
				}
			}
#endif /* !FT_ENABLE_TRACING */
		}
#endif /* !FT_REMOVE_TAIL_QUIET */
		if (caa_unlikely(ft_remove_attempts > FT_REMOVE_RETRY_CAP)) {
			fprintf(stderr, "FT REMOVE LIVELOCK: %u attempts on "
				"one remove\n", ft_remove_attempts);
			abort();
		}
#endif
		/* Age the conflict, forfeit the turn, close the attempt. */
		ft_txn_attempt_bail(&optxn, true);
#ifdef FT_DLM_LINGER
		ft_dlm_linger(&optxn);	/* nothing held here; see the helper */
#endif
#ifdef FT_DEBUG_REMOVE_RETRY_CAP
		ft_acc_bail += ft_dbg_now_ns() - ft_tC;
#endif
	}
	ft_op_end(&optxn);
#ifdef FT_DEBUG_REMOVE_RETRY_CAP
	{
		uint64_t wall = ft_dbg_now_ns() - ft_remove_t0;

		/*
		 * ☞ BUSY IS NOT SLOW, and only one of them was reported.
		 * This block fires on WALL TIME, so a remove that retried two
		 * thousand times CHEAPLY never printed: the run that measured
		 * max-retry 38,244 produced exactly ONE "SLOW" line, and it had
		 * attempts=0 -- a grace-period wait, not a retry storm.  Fire on
		 * the attempt count too, which is the axis the retry question is
		 * actually asked on.
		 */
		if (FT_REMOVE_BUSY_ATTEMPTS &&
				caa_unlikely(ft_remove_attempts >
					FT_REMOVE_BUSY_ATTEMPTS))
			fprintf(stderr, "FT REMOVE BUSY: attempts=%u "
				"dirtyLOCK=%u dirtyOTHER=%u cabort=%u "
				"wall_us=%llu\n",
				ft_remove_attempts, ft_dbg_acq_dirty_lock,
				ft_dbg_acq_dirty_other, ft_dbg_acq_cabort,
				(unsigned long long) (wall / 1000));
		if (FT_REMOVE_SLOW_NS && caa_unlikely(wall > FT_REMOVE_SLOW_NS))
			fprintf(stderr, "FT REMOVE SLOW: wall_us=%llu "
				"attempts=%u dirtyLOCK=%u dirtyOTHER=%u "
				"cabort=%u begin_us=%llu body_us=%llu "
				"bail_us=%llu other_us=%llu\n",
				(unsigned long long) (wall / 1000),
				ft_remove_attempts, ft_dbg_acq_dirty_lock,
				ft_dbg_acq_dirty_other, ft_dbg_acq_cabort,
				(unsigned long long) (ft_acc_begin / 1000),
				(unsigned long long) (ft_acc_body / 1000),
				(unsigned long long) (ft_acc_bail / 1000),
				(unsigned long long) ((wall - ft_acc_begin -
					ft_acc_body - ft_acc_bail) / 1000));
		if (FT_REMOVE_SLOW_NS && caa_unlikely(wall > FT_REMOVE_SLOW_NS)) {
			struct rusage ru;

			getrusage(RUSAGE_THREAD, &ru);
			fprintf(stderr, "FT REMOVE SLOW GP: gp_us=%llu "
				"gp_calls=%u arena_us=%llu arena_waits=%u "
				"nvcsw=%ld nivcsw=%ld\n",
				(unsigned long long) (ft_dbg_gp_ns / 1000),
				ft_dbg_gp_calls,
				(unsigned long long) (ft_dbg_arena_ns / 1000),
				ft_dbg_arena_waits,
				(long) (ru.ru_nvcsw - ft_remove_ru0.ru_nvcsw),
				(long) (ru.ru_nivcsw -
					ft_remove_ru0.ru_nivcsw));
		}
#ifdef CDS_FAIR_MUTEX_DBG_POLL
		if (FT_REMOVE_SLOW_NS && caa_unlikely(wall > FT_REMOVE_SLOW_NS))
			fprintf(stderr, "FT REMOVE SLOW POLLS: fmtx=%lu "
				"wfcq=%lu\n",
				cds_fmtx_dbg_polls - ft_fmtx0,
				cds_wfcq_dbg_polls - ft_wfcq0);
#endif
	}
#endif
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
	/*
	 * /!\ RESOLVE BEFORE COMPARING.  A peer writer's committed-but-unsettled
	 * txn parks a flip proxy in this very slot, and ft_node_ptr's contract is
	 * a RESOLVED pointer -- it asserts on a parked one under
	 * -DFT_DEBUG_PROXY_ASSERT, which is how this was found (9 legs of a gate
	 * run, deterministically at the same test).
	 *
	 * Read RAW, the mask lands on a DESCRIPTOR, the compare fails, and this
	 * answers "not the live head" for a head that IS live.  The first caller
	 * absorbs that as a stale cache and re-seeds, but the second asks again
	 * AFTER a fresh lookup has already found the node, and turns a false here
	 * into CDS_FT_STATUS_NOT_FOUND -- a wrong answer for a key that is
	 * present, and one the oracles cannot see, because a concurrent
	 * remove_all lane legitimately reports NOT_FOUND too.
	 *
	 * A refusal is a claim about the trie; it must not be decided by a word
	 * that merely happened to be in flight.
	 */
	return (struct cds_ft_node *) ft_node_ptr(
			ft_resolve_flip_proxy(**head_slot_p)) == head;
}



static
enum cds_ft_status _cds_ft_remove_all_locked(struct cds_ft *ft,
		struct cds_ft_iter *iter,
		struct cds_ft_node **result_node,
		bool *need_retry,
		struct ft_op *op)
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
		CDS_FT_ASSERT_CALLER_RCU_READ_LOCKED(ft);
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
		struct cds_ft_inode_flag *root_nf;
		struct ft_flip_txn *root_txn = NULL;
		unsigned int nr_frozen;

		root_nf = ft_resolve_flip_proxy(rcu_dereference(ft->root));
		metadata = cds_ft_item_to_metadata(ft_node_ptr(root_nf));
		external_nodes = metadata->external_nodes;
		if (!external_nodes) {
			*result_node = NULL;
#if defined(FT_ENABLE_TRACING) || defined(FT_DEBUG_RM_SITE)
			ft_dbg_rm_site = __LINE__;
#endif
			return CDS_FT_STATUS_NOT_FOUND;
		}
		/*
		 * The DERIVATION the freeze below is bounded by; see
		 * ft_hlist_freeze_chain_prepare.  Taken before either commit
		 * path so both reserve the same number of edges.
		 */
		nr_frozen = ft_hlist_chain_len(external_nodes);
		/*
		 * LOCK THE ROOT, THEN RE-READ THE PLAN UNDER IT
		 * (doc/design/ft-lockset-inventory.md §1, row 2).
		 *
		 * Both commits below clear the ROOT's @external_nodes and freeze
		 * the chain it heads, and until now they did it holding nothing
		 * -- ~58k commits per ft_inv run, at every spacing.  The root is
		 * the word's owner and, at depth 0, its own anchor under every
		 * spacing, so this is one acquire with no descent.  It is
		 * registered in @root_txn with its release, which the commit
		 * consumes and an early destroy releases.
		 *
		 * Derive, acquire, RE-VALIDATE, bail retriably: a root replaced
		 * in between (graft_swap, a root recompaction) refuses the
		 * acquire or fails the identity check, and a chain grown or
		 * cleared in between fails the head / length check.  Both are
		 * the retry the callers below already take on an aborted flip.
		 * An exclusive or COARSE trie has no per-node peer to exclude.
		 */
		root_txn = ft_flip_txn_create_bounded(ft,
			FT_REMOVE_COMMIT_REC_MAX_EDGES +
			(ft->rank_stats ? 1 : 0) +
			nr_frozen * FT_HLIST_FREEZE_MAX_EDGES +
			1 /* the root's {LOCK|s -> s} release */);
		if (!root_txn) {
			*result_node = NULL;
			return CDS_FT_STATUS_MEMORY_ERROR;
		}
		if (ft->lock_fine && !ft->exclusive) {
			struct ft_lock_ctx rctx;
			enum ft_lock_or_guard_exit rex;

			ft_lock_ctx_init(&rctx, NULL, root_txn, op);
			ft_flip_txn_lock_or_guard_parent_ex(__func__, __LINE__,
				ft, root_txn, &rctx, root_nf, 0, &rex);
			if (rex == FT_LOG_EXIT_MISS) {
				bool enomem = root_txn->acquire_enomem;

				ft_flip_txn_destroy(root_txn);
				*result_node = NULL;
				if (enomem)
					return CDS_FT_STATUS_MEMORY_ERROR;
				FT_DBG_RETRY_SITE();
				*need_retry = true;
				return CDS_FT_STATUS_OK;
			}
			if (ft_resolve_flip_proxy(rcu_dereference(ft->root)) !=
						root_nf ||
					metadata->external_nodes != external_nodes ||
					ft_hlist_chain_len(external_nodes) !=
						nr_frozen) {
				ft_flip_txn_destroy(root_txn);
				*result_node = NULL;
				FT_DBG_RETRY_SITE();
				*need_retry = true;
				return CDS_FT_STATUS_OK;
			}
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
			/*
			 * Read under the root's lock, after the re-check above,
			 * so this prev is settled (every producer of a parked
			 * head prev holds the chain holder); resolved anyway, as
			 * the plan-time read below must be.
			 */
			struct ft_ord_cell *dead = ft->ordered_list ?
				ft_ord_cell_ptr(ft_dereference_prev_resolved(
					external_nodes)) : NULL;
			struct ft_flip_txn *txn = root_txn;

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
			ft_flip_txn_record_count_parent(ft, txn, ft->root, -1);
			/*
			 * ☑ AND THE CHAIN'S FREEZE RIDES THIS FLIP, like the
			 * leaf and prefix arms.  It used to be a post-commit
			 * ft_ra_sweep_held, whose acquire is gated on
			 * `lock_spacing == PER_NODE` -- so above per-node it
			 * never fired and the marks were bare CASes holding
			 * NOTHING.  MEASURED: UNHELD=1 at exponential AND
			 * root-only on inv_remove_cross_view_prefix_siblings_all,
			 * 0 at per-node.  Folding it in is spacing-INDEPENDENT,
			 * which is why it is the cure rather than widening the
			 * acquire.
			 */
			ft_ch_audit(ft, txn, external_nodes);
			/*
			 * The plan IS validated under the lock here already --
			 * the root acquire above re-reads it and bails retriably
			 * (doc §1 row 2), which is this class's prescribed shape.
			 */
			ft_hlist_freeze_chain_prepare_checked(ft,
				ft_flip_txn_handle(txn),
				external_nodes, nr_frozen);
			/*
			 * Same rule as the prefix clear below: the pre-reserved
			 * txn cannot fail to ALLOCATE, but the flip can still
			 * ABORT, and an aborted flip left the NIL key in the
			 * trie.  Returning OK there hands the caller a chain it
			 * may reclaim while the root still points at it -- and
			 * frees @dead while it is still spliced into the list.
			 */
			/*
			 * PRE-RESERVED => the commit cannot fail to allocate,
			 * so its only non-zero is -EAGAIN: a peer won this
			 * flip and NOTHING was installed (the fused cell and
			 * count edges were discarded with it).  Reporting that
			 * as MEMORY_ERROR sent the caller freeing memory over a
			 * lock it merely lost; it is the NIL key's copy of the
			 * mislabel the tail below fixed for every other shape.
			 * Retriable: re-read external_nodes and re-attempt.
			 */
			if (ft_remove_one_commit(ft,
					(struct cds_ft_inode_flag **) &metadata->external_nodes,
					metadata,
					(struct cds_ft_inode_flag *) external_nodes, NULL,
					NULL, dead, NULL, txn, NULL, false)) {
				*result_node = NULL;
				FT_DBG_RETRY_SITE();
				*need_retry = true;
				return CDS_FT_STATUS_OK;	/* discarded by the retry loop */
			}
			if (dead)
				ft_ord_cell_free(ft, dead);
		} else {
			/*
			 * List off AND rank off.  This was a bare lone flip with
			 * the tombstones swept afterwards; the sweep could not
			 * hold anything above per-node (above), so take a
			 * pre-reserved bounded txn here too and commit the clear
			 * WITH the chain's marks -- the same shape the prefix
			 * clear uses for exactly this reason ("no bare lone
			 * clear even with both flags off").
			 */
			struct ft_flip_txn *txn = root_txn;

			ft_ch_audit(ft, txn, external_nodes);
			/*
			 * The plan IS validated under the lock here already --
			 * the root acquire above re-reads it and bails retriably
			 * (doc §1 row 2), which is this class's prescribed shape.
			 */
			ft_hlist_freeze_chain_prepare_checked(ft,
				ft_flip_txn_handle(txn),
				external_nodes, nr_frozen);
			if (ft_remove_one_commit(ft,
					(struct cds_ft_inode_flag **) &metadata->external_nodes,
					metadata,
					(struct cds_ft_inode_flag *) external_nodes, NULL,
					NULL, NULL, NULL, txn, NULL, false)) {
				*result_node = NULL;
				FT_DBG_RETRY_SITE();
				*need_retry = true;
				return CDS_FT_STATUS_OK;	/* discarded by the retry loop */
			}
		}
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
		bool found = s == CDS_FT_STATUS_OK && iter->node;

		if (!found || !ft_locate_chain_head(ft, iter->node, iter_key,
				key_len, &holder_flag, &head_slot, &is_prefix)) {
			/*
			 * ☠ FOUND, THEN NOT LOCATED, IS NOT A MISS.  The lookup
			 * just reached a live head; ft_locate_chain_head then
			 * re-derives its position from two more UNHELD reads
			 * (the back edge, then the holder's slot), and a peer
			 * re-home between them -- an insert of a key extending
			 * this one republishing the slot with a junction that
			 * carries the head -- makes them disagree.  That is the
			 * identity-bail shape _cds_ft_remove_locked confirms
			 * under the holder's lock (ft_rm_miss_unconfirmed);
			 * here nothing is held yet, so re-derive the attempt.
			 */
#ifndef FT_DEBUG_RA_LOCATE_MISS
			if (found && !ft_node_is_removed(iter->node)) {
				*result_node = NULL;
				FT_DBG_RETRY_SITE();
				*need_retry = true;
				return CDS_FT_STATUS_OK;	/* discarded by the retry loop */
			}
#endif
			*result_node = NULL;
#if defined(FT_ENABLE_TRACING) || defined(FT_DEBUG_RM_SITE)
			ft_dbg_rm_site = __LINE__;
#endif
			return CDS_FT_STATUS_NOT_FOUND;
		}
		chain_head = iter->node;
	}
	holder_meta = cds_ft_item_to_metadata(ft_node_ptr(holder_flag));
	*result_node = chain_head;

	/*
	 * The holder comes from the cached node's back-pointer; per-node locking
	 * anchors every member on itself, so no descent (and no depth) is needed.
	 */
	struct ft_lock_ctx lctx;
	unsigned int holder_depth = 0;

	ft_lock_ctx_init(&lctx, NULL, NULL, op);

	/*
	 * Ordered list on: the whole key leaves the trie, so its head's cell is
	 * unspliced + freed below.  An in-place leaf detach fuses that unsplice
	 * into its structural flip (pub.armed); the prefix and recompaction /
	 * compressed-parent shapes stay two-commit (pub unarmed).
	 */
	struct ft_remove_pub pub = { .armed = false };
	/*
	 * THE HEAD'S CELL, FROM ONE RESOLVED LOAD -- cds_ft_remove's discipline.
	 * Nothing is held here, and a peer that removes the head before
	 * @chain_head and PROMOTES it parks a flip proxy on @chain_head->prev
	 * for the length of its commit.  Read raw, ft_ord_cell_ptr() masked the
	 * proxy's tag into an address INSIDE the peer's descriptor (...0xe), the
	 * under-lock re-checks passed once the peer had committed, and the cell
	 * lock take then followed that "cell" into unmapped memory: the zoo's
	 * SEGV in ft_cell_word_lock (probe: every wild cell ended in 0xe and
	 * resolved to a real one; the two that got past the re-checks reached
	 * ft_detach_node's take).  insert_replace had the same raw read
	 * (f8f6cd3e).
	 *
	 * Resolved, the value is the word before or after that peer's commit,
	 * never a descriptor, and a stale-but-real cell is what the under-lock
	 * re-validation (the chain plan, the cell plan) already catches.  A prev
	 * naming a PREDECESSOR says @chain_head is no longer a head: re-plan.
	 */
#ifdef FT_DEBUG_RA_PLAN_CELL_RAW
	struct ft_ord_cell *dead_cell = ft->ordered_list ?
		ft_ord_cell_ptr(chain_head->prev) : NULL;
#else
	struct ft_ord_cell *dead_cell = NULL;

	if (ft->ordered_list) {
		void *head_prev = ft_dereference_prev_resolved(chain_head);

		if (ft_node_external((struct cds_ft_inode_flag *) head_prev)) {
			*result_node = NULL;
			FT_DBG_RETRY_SITE();
			*need_retry = true;
			return CDS_FT_STATUS_OK;	/* discarded by the retry loop */
		}
		dead_cell = ft_ord_cell_ptr(head_prev);
	}
#endif
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
			bool prefix_fused = false, fused_declined = false;
			/*
			 * The DERIVATION both arms below freeze against, taken
			 * once: ft_hlist_freeze_chain_prepare treats it as a
			 * BOUND, so a duplicate appended since is not this op's
			 * to retire and instead tears the derived tail's NULL.
			 */
			unsigned int nr_frozen = ft_hlist_chain_len(chain_head);
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
				/*
				 * INTENT: as above -- no body child detached, and
				 * the external entry this op clears is
				 * @chain_head, the head the is_prefix arm derived
				 * and is about to unlink.
				 */
				const struct ft_chain_compress_intent intent = {
					.detach_child = NULL,
					.expect_ext = chain_head,
					.detach_byte = 0,
				};

				cret = ft_chain_compress_fused(ft,
					holder_flag, holder_depth, &lctx,
					holder_meta,
					s_child, s_byte,
					1 /* sole body child; the removed entry is external */,
					ft->ordered_list ? dead_cell : NULL,
					NULL, NULL, 0, NULL, NULL, 0 /* no orphan chain */,
					chain_head, nr_frozen,
					-1, ft->rank_stats ? key_len + 1 : 0,
					NULL, false,
					NULL /* no pending publish */, &intent, NULL);

				if (cret == 0) {
					/*
					 * ☑ THE CHAIN'S FREEZE RODE THE FUSED
					 * COMMIT (freeze_leaf + nr_frozen above),
					 * under the holder that commit owns.  It
					 * used to be a post-commit sweep whose own
					 * comment conceded the point: "the fuse may
					 * RETIRE @holder_flag ... an acquire that
					 * refuses a tombstone degrades to exactly
					 * today's unheld sweep".  A freeze that is
					 * IN the commit needs no acquire afterwards
					 * and cannot degrade -- and it is atomic
					 * with the unlink, so no reader sees the
					 * key gone with the chain still live.
					 */
					if (ft->ordered_list)
						pub.armed = true;
					ret = 0;
					prefix_fused = true;
				} else if (cret < 0) {
					ret = cret;
					prefix_fused = true;
				} else {
					/* Merge out of bound -- fall back to plain clear. */
					fused_declined = true;
				}
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
					(ft->rank_stats ? key_len + 1 : 0) +
					nr_frozen * FT_HLIST_FREEZE_MAX_EDGES);

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
				/*
				 * ☞ A MISSED TAKE ENDS THE ATTEMPT HERE, before the
				 * count walk and the chain freeze are recorded into a
				 * txn the commit is bound to discard: the freeze's
				 * chain records are SW whoever holds the word, so each
				 * is a stale plan against the holder's own in-flight
				 * freeze of the same chain (the SW stale-old audit on
				 * inv_prefix_shape_zoo, the peer's record on the same
				 * slot, SUCCEEDED).  Same exit as the plan bail below.
				 */
				if (caa_unlikely(txn->acquire_miss)) {
					ft_flip_txn_destroy(txn);
					if (unsplice_txn)
						ft_flip_txn_destroy(unsplice_txn);
					*result_node = NULL;
					FT_DBG_RETRY_SITE();
					*need_retry = true;
					return CDS_FT_STATUS_OK;
				}
#if defined(FEATURE_FT_SKIP_COMPRESSED) && !defined(FT_DEBUG_RA_CLEAR_RESIDUE)
				/*
				 * THE PLAIN CLEAR MUST NOT STRAND ITS HOLDER -- the
				 * remove twin of this check (_cds_ft_remove_locked).
				 * This arm was chosen from an UNHELD read: the holder
				 * had more than one child, so clearing its key left a
				 * legal node.  A peer that removed a child before the
				 * lock above changes the answer, and the clear would
				 * leave a 1-child keyless internal skip mode refuses;
				 * the post-prune below is a separate commit that can
				 * lose to a peer and leave it.  The holder is held and
				 * nothing is recorded yet: re-plan, and the retry
				 * reads one child and takes the fused collapse.  Not
				 * where the fused collapse itself declined (out of
				 * bound): the retry would decline again.
				 */
				if (!fused_declined &&
				    ft_group_skip_compressed(ft->group) &&
				    ft_meta_nr_child_load(holder_meta) == 1 &&
				    ft_parent_node(holder_meta->parent_word) != NULL) {
					ft_flip_txn_destroy(txn);
					if (unsplice_txn)
						ft_flip_txn_destroy(unsplice_txn);
					*result_node = NULL;
					FT_DBG_RETRY_SITE();
					*need_retry = true;
					return CDS_FT_STATUS_OK;
				}
#else
				(void) fused_declined;
#endif
				ft_flip_txn_record_count_parent(ft, txn,
					holder_flag, -1);
				/*
				 * ☑ AND THE CHAIN'S FREEZE RIDES THIS SAME FLIP.
				 * Recorded onto @txn -- which already carries the
				 * §4.B guard and the count walk -- so
				 * ft_remove_one_commit's ft_ord_cell_flip_into
				 * commits the external_nodes clear, the unsplice,
				 * the count and every {v -> MARK(v)} together.
				 * The holder is LOCKED by the guard above, so the
				 * whole chain is frozen under its owner; the
				 * post-commit ft_ra_sweep_held this replaces had
				 * to re-acquire that word a second time to say
				 * the same thing, and was not atomic with the
				 * unlink.  @nr_frozen is the DERIVATION and the
				 * walk is BOUND by it: a duplicate appended since
				 * tears the derived tail's NULL and aborts this
				 * commit, leaving the key in place for the retry.
				 */
				ft_ch_audit_ctx(ft, txn, &lctx, chain_head);
				/*
				 * ★ VALIDATE THE PLAN UNDER THE LOCK.  @nr_frozen was
				 * counted before the §4.B acquire just above, so the
				 * freeze's derived tail NULL has been standing in for
				 * an exclusion the count did not have.  We hold the
				 * holder here; ask the word.  (It shrinks the window
				 * the install CAS arbitrates -- it does not replace
				 * the CAS; see ft_hlist_chain_plan_ok's header.)
				 *
				 * ☠ PRE-COMMIT TERMINAL, BOTH HANDLES.  @txn already
				 * carries the §4.B guard, the count records and the
				 * registered holder lock, and @unsplice_txn is a
				 * reservation this return would otherwise strand -- the
				 * tail that frees them (and ft_remove_one_commit, which
				 * drains @txn on a failed commit) is below and is
				 * skipped by a retry return.  Leaving without both
				 * destroys is how a bail leaks FT_STATE_LOCK and
				 * refuses every peer the holder forever.
				 */
				if (!ft_hlist_chain_plan_ok(ft, ft_flip_txn_handle(txn),
						chain_head, nr_frozen)) {
					FT_HLIST_PLAN_BAIL();
					ft_flip_txn_destroy(txn);
					if (unsplice_txn)
						ft_flip_txn_destroy(unsplice_txn);
					*result_node = NULL;
					FT_DBG_RETRY_SITE();
					*need_retry = true;
					return CDS_FT_STATUS_OK;
				}
				ft_hlist_freeze_chain_prepare_checked(ft,
					ft_flip_txn_handle(txn), chain_head,
					nr_frozen);
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
		/*
		 * ☑ THE WHOLE CHAIN'S FREEZE RIDES THE DETACH COMMIT, under the
		 * holder that commit already owns.  It used to be a bare
		 * ft_chain_mark_removed_flip sweep AFTER the detach returned --
		 * one unlocked uatomic_cmpxchg per chain node, holding NOTHING
		 * (the hold audit's only surviving class-A violation: 100%
		 * NOLOCKS at every row that reached it).
		 *
		 * ☠ AND IT COULD NOT BE FIXED BY TAKING A LOCK, which is why it
		 * waited for the structural phase.  By sweep time the holder is
		 * GONE, not contended: the detach retires it IN the very commit
		 * that unlinks the chain, so the acquire found FT_STATE_TOMBSTONE
		 * (measured: ok=0, refused=22007, of which 22005 were -EAGAIN on
		 * a tombstoned word).  Nor could the acquire be hoisted above the
		 * detach -- unpublished it misses against the op's OWN hold
		 * (ft_detach_node climbs from holder_meta) and waits on itself
		 * forever; published into @lctx it becomes the whole-op scope
		 * -DFT_RM_REVALIDATE was measured and refuted for (20/41 hangs,
		 * reader-visible key loss).  The lock was never the answer.
		 *
		 * @nr_frozen is the DERIVATION, and ft_hlist_freeze_chain_prepare
		 * treats it as a BOUND, not a hint: a duplicate appended since is
		 * not this op's to retire, and tearing the derived tail's NULL
		 * aborts the commit so the retry re-derives the longer chain.
		 */
		unsigned int nr_frozen = ft_hlist_chain_len(chain_head);

		ft_removeall_fault_scope_enter();
		ret = ft_detach_node(ft, &lctx, head_slot,
			ft_get_parent_slot(holder_meta, ft), key_len, true,
			dead_cell, ft->ordered_list ? &pub : NULL, NULL, NULL,
			chain_head, nr_frozen,
			-1 /* leaf key removed: detach owns the -1 */,
			NULL, false, NULL, NULL,
			/* EXPERIMENT: delete tier open, for the trace. */
			ft_in_place_delete_ok(ft));
		ft_removeall_fault_scope_exit();
	}

	/*
	 * -ENOENT (detach's replace reached an emptied FT_NULL slot) is a PEER,
	 * not a bug, and this used to assert it away on the premise that the
	 * position "has been found by a mutex-protected traversal within this
	 * function".  No mutex exists here under LOCK_FINE: @head_slot is
	 * derived by ft_locate_chain_head with NOTHING HELD, so a peer's
	 * recompaction can retype or empty that slot between the derivation and
	 * the replace -- exactly the window cds_ft_remove documents at its own
	 * tail.  The path publishes nothing, so it joins -EAGAIN in the retry
	 * below instead of aborting a debug build (and, under NDEBUG, falling
	 * through to a MEMORY_ERROR no allocation produced).
	 */

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
			if (!pub.armed) {
				/*
				 * ☠ THIS DISCARDED THE STATUS on the premise
				 * that an "ABORT [is] unreachable under its
				 * current exclusion" -- a premise the fine
				 * locking conversion DELETES.  The op's whole-
				 * chain standalone marks are already public here,
				 * so the removal cannot be retried and the
				 * unsplice MUST complete: an aborted one left the
				 * key gone from the structural index but present
				 * in the ordered list, and then freed @dead_cell
				 * while it was still spliced into that list.
				 * DRIVE IT FORWARD instead -- each ABORT means a
				 * peer committed (obstruction-free), and the
				 * aborted commit consumed the txn, so the small
				 * bounded reservation is retried too.  Byte-for-
				 * byte the tail cds_ft_remove already runs.
				 */
				FT_DBG_RA_UNSPLICE_ARM();
				while (ft_ord_cell_unsplice(ft, unsplice_txn,
						dead_cell) > 0) {
					FT_DBG_RA_UNSPLICE_ABORT();
					do {
						unsplice_txn =
							ft_flip_txn_create_bounded(ft,
							FT_ORD_CELL_UNSPLICE_MAX_EDGES);
					} while (caa_unlikely(!unsplice_txn));
				}
			} else {
				FT_DBG_RA_UNSPLICE_FUSED();
				ft_flip_txn_destroy(unsplice_txn);
			}
			ft_ord_cell_free(ft, dead_cell);
		} else {
			/* Removal aborted: the cell stays in the list -- release the
			 * unused reservation. */
			ft_flip_txn_destroy(unsplice_txn);
		}
	}

	/*
	 * ☠ KEEP THE KEY.  The retriable exit below re-attempts on the strength
	 * of "@iter's cache is invalidated just above, so the next attempt
	 * re-seeds through a fresh lookup" -- and that re-seed reads @iter's
	 * KEY, which on a keycopy trie is reachable only WHILE @cache_valid
	 * holds (ft_iter_key_referenced -> @iter->node + speculative_key_offset).
	 * A bare clear here hands the retry a zeroed key, and the re-seed then
	 * reports NOT_FOUND for a key that is present -- MEASURED as
	 * inv_compact_keycopy_terminates' drain abort, key 248 of 2000.
	 */
	ft_iter_drop_position_keep_key(iter);

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
		 * differently.  The distinction is only as good as the sources,
		 * which is what blocked this mapping before -- an acquire that
		 * could not allocate its own lock txn used to reach the commit
		 * as @acquire_miss and abort, arriving here as -EAGAIN with no
		 * allocation failure anywhere in the errno.  That path now
		 * carries @acquire_enomem and commits MEMORY_ERROR instead, so
		 * every -EAGAIN landing here is a peer.
		 *
		 * ⇒ A PEER IS NOW THIS OP'S OWN PROBLEM, not the caller's.  It
		 * used to be reported as BUSY_ERROR and handed back, which is
		 * why 91% of remove_all calls FAILED under a same-key peer
		 * (measured: BUSY 25634 vs OK 2473 over 150 rounds) -- the
		 * caller's own retry cannot fix that, because a fresh urcu_txn
		 * per attempt ages NO conflict and so never escalates into the
		 * FIFO lane that drains the contention.  Signal the wrapper's
		 * retry loop instead: it re-derives the chain head against the
		 * current tree on the PERSISTENT @optxn.  Nothing was published
		 * on either errno, so re-attempting is sound (@iter's cache is
		 * invalidated just above, so the next attempt re-seeds through
		 * a fresh lookup).
		 */
		*result_node = NULL;
		if (ret == -EAGAIN || ret == -ENOENT) {
			FT_DBG_RETRY_SITE();
			*need_retry = true;
			return CDS_FT_STATUS_OK;	/* discarded by the retry loop */
		}
		return CDS_FT_STATUS_MEMORY_ERROR;
	}

	return CDS_FT_STATUS_OK;
}

/*
 * Public entry: FT-owned per-op read-side bracket + retry identity, mirroring
 * cds_ft_remove and cds_ft_replace.
 *
 * WHAT THIS OP WAS MISSING, and it is not what the other three were.  remove_all
 * already ACQUIRES and RE-VALIDATES everything it derives: the leaf arm goes
 * through ft_detach_node (the same primitive the converted point remove uses),
 * and the prefix arm through ft_flip_txn_lock_or_guard_parent plus a commit
 * whose expected-old IS the re-validation.  Measured against a same-key peer,
 * the structure was never corrupted -- 150 rounds x 64 keys x 3 duplicates lost
 * no node, double-owned no node and verified green every round.  What it lacked
 * was PROGRESS: the body ran EXACTLY ONCE, so every one of those correct,
 * published-nothing aborts was handed to the caller as BUSY_ERROR.  91% of the
 * calls failed that way.
 *
 * ⇒ the conversion is the LOOP, not new exclusion.  Aging is carried on the
 * PERSISTENT @optxn via ft_op_conflict (ft_txn_attempt_bail): after
 * its lane budget of conflicts this writer escalates into the per-trie
 * FIFO fair-mutex lane, which drains the contention so the retry TERMINATES.
 * That is also why a caller looping on BUSY_ERROR was never an adequate
 * substitute -- a fresh txn per attempt ages nothing and can starve.  An
 * exclusive trie opens nothing and never conflicts.
 *
 * @iter is re-seeded by the body itself: every retriable exit runs after the
 * cache invalidation, so the next attempt re-descends through a fresh lookup.
 */
enum cds_ft_status cds_ft_remove_all(struct cds_ft *ft,
		struct cds_ft_iter *iter,
		struct cds_ft_node **result_node)
{
	struct ft_op optxn;
	struct ft_op_retry op_retry;
	enum cds_ft_status s;
	bool need_retry;

	CDS_FT_SCOPED_WRITER(ft);
	/*
	 * Same rule as cds_ft_remove (ft_iter_redescend_node): with a bulk op live
	 * the cached position may predate its flip.  Dropping it is enough here
	 * -- the locked body already re-seeds by a fresh lookup from the root
	 * when the cache is not valid.
	 */
	if (caa_unlikely(ft_bulk_active(ft)))
		ft_iter_drop_position_keep_key(iter);
	ft_op_retry_init(&op_retry, FT_OP_REMOVE_ALL, NULL, 0);
	ft_txn_op_init(ft, &optxn);
	for (;;) {
		need_retry = false;
		ft_op_retry_tick(ft, &op_retry, 0);
		ft_op_begin(&optxn);
		s = _cds_ft_remove_all_locked(ft, iter, result_node,
				&need_retry, &optxn);
		if (!need_retry)
			break;
		/* Age the conflict, keep the FIFO turn, close the attempt. */
		ft_txn_attempt_bail(&optxn, true);
	}
	ft_op_end(&optxn);
	return s;
}

