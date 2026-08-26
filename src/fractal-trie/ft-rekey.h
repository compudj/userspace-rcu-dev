// SPDX-FileCopyrightText: 2012-2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-only

/*
 * src/fractal-trie/ft-rekey.h
 *
 * Userspace RCU library - Fractal Trie: the REKEY (same-trie move) writer.
 *
 * A rekey moves a subtree from one key to another WITHIN ONE TRIE, so its
 * source and its destination are the same live, concurrently-read structure.
 * That is what separates it from the cross-trie graft/merge writers in
 * ft-graft.h / ft-merge.h, whose entry gates REQUIRE an exclusive source
 * (src_ft != dst_ft && !src_ft->exclusive -> BUSY_ERROR) and which therefore
 * have no concurrent peer on the src side at all.
 *
 * The two used to share ft_merge_at_inner / ft_merge_spine_copy /
 * ft_merge_graft_subpos_inplace -- 599 lines of worker code carrying ZERO
 * tests on @rekey or on src_ft == dst_ft, so both callers were served by the
 * stricter contract, unconditionally and invisibly.  This unit holds the
 * rekey's own copies (ft_rekey_at_inner / ft_rekey_spine_copy /
 * ft_rekey_subpos_inplace), split off so each side can be specialised against
 * its single contract.
 *
 * Functions that only compute SHAPE -- ft_merge_build, ft_merge_count,
 * ft_merge_ord_interleave_collect and the small helpers -- carry no atomicity
 * contract and stay SHARED in ft-merge.h.  The boundary is exactly "does it
 * COMMIT, DRAIN, or CREATE A TXN", which is the only thing the two callers
 * disagree about.
 *
 * Implementation unit: #included once into the fractal-trie.c translation
 * unit, AFTER ft-merge.h and ft-mutation-node.h (it uses their shared
 * helpers).  Not a standalone header.
 */
#ifndef FRACTAL_TRIE_IMPL
#error "ft-rekey.h is an implementation unit; #include it from fractal-trie.c only"
#endif


/* ------------- moved from ft-mutation-node.h ------------- */

/*
 * ft_rekey_cow_stop: identity-preserving COW of a LIVE published node @stop_flag
 * (the "S_top" of a same-trie rekey) into a FRESH-address copy returned UNPUBLISHED
 * in *@stop_prime_ret, re-parenting @stop's direct children onto the copy and
 * RETIRING @stop -- all as records in the caller's mixed SW/MW commit @txn.  The
 * interior below @stop is SHARED (same child addresses), so only @stop's identity
 * moves: exactly what the coherent reader's two-descent address-witness needs to
 * detect a move.  Mirrors ft_node_recompact's RELOCATE copy/reparent core but
 * (a) returns the copy for the CALLER to place (a rekey publishes it at the dst
 * junction; the isolation test at @stop's own parent slot -- a same-position
 * in-place clone), and (b) drives the SW mixed engine: @txn has structural_sw
 * set, so the re-parents' parent-pointer + pso edges and the retire park SW
 * (locked, cannot fail).
 *
 * SW SAFETY (why the parks cannot be clobbered): the parent-pointer edge is
 * owned by @stop's lock (no peer re-homes a child of a LOCK-held parent).
 * The re-parent's §4.B guard edge and the retire both park the STATE word, which
 * ft_meta_nr_child_inc CASes IGNORING LOCK, so each such node is
 * LOCK-MARKED here (@stop for the retire, every metadata-bearing child for
 * its guard) and a peer's count CAS now HONORS the mark and spins
 * (FT_STATE_INPLACE_WAIT_MASK) until this commit settles the word.  The interior
 * stays SHARED, so children of children need no marks.
 *
 * ☠ RELEASE ATTRIBUTION -- do not re-derive.  Each child's mark is released by
 * ft_reparent_record_meta's {live_state -> live_state} STATE edge, whose
 * live_state has LOCK masked out and which is recorded UNCONDITIONALLY; the
 * retire consumes @stop's.  It is NOT the pso edge: since @118245b0 the pso is
 * its own word and its edge is recorded only when the slot index CHANGES, so
 * resting the release on it would leak a permanent LOCK on every child that
 * lands at the same index.  The pre-§8.3 story (pso rode the state word, the pso
 * edge did the masking) reads plausibly and is wrong.
 *
 * CALLER CONTRACT:
 *  - ft->lock_fine and ft_flip_txn_set_structural_sw(@txn, true) before calling.
 *  - @marks / @snaps: caller-owned scratch of >= FT_ENTRY_PER_NODE + 1 entries;
 *    *@nr_marks returns the count recorded (published incrementally so a bail is
 *    covered).  These marks are NOT txn-registered (count can exceed
 *    FT_FLIP_TXN_MAX_LOCKS), so the CALLER MUST, after committing OR aborting
 *    @txn, sweep ft_meta_lock_release_if_held over marks[0..*@nr_marks) -- a
 *    no-op for the LOCK a commit consumed via the retire / pso SW edges, a
 *    release for any still held on abort (the ft_detach_node orphan-chain
 *    pattern).
 *  - On success (0): *@stop_prime_ret is the fresh UNPUBLISHED copy.  The caller
 *    wires its parent back-edge and records the forward publish (and, for a real
 *    rekey, clears @stop's src slot) into @txn before commit, and frees the OLD
 *    @stop via call_rcu after the grace period once the commit succeeds.
 *  - On failure (<0: -EAGAIN re-descend / -ENOMEM): *@stop_prime_ret is NULL, the
 *    unpublished copy (if any) is already freed, nothing is published; the caller
 *    still sweeps @marks.
 *
 * SCOPE: @stop is an internal POPCOUNT / PIGEON node with no co-located external
 * list, or a PLAIN COMPRESSED one.  [TODO: an external-head S_top, and a
 * co-located external list, still need recompact's extra arms.]
 *
 * THE COMPRESSED ARM IS THE SAME SHAPE WITH ONE CHILD.  A compressed node carries
 * a key-byte run and a single child, so the copy is len + key_bytes + that child,
 * and the mark/re-parent sweep runs once instead of over a bitmap.
 *
 * @cut CUTS that run: the copy carries key_bytes[@cut .. len) instead of the whole
 * span, which is what a src key ENDING INSIDE the run needs -- the moved top is
 * the run's tail, manufactured here because no node exists at that depth.  The
 * child is unchanged (a run has exactly one, and everything below the cut is
 * everything the run held), so the mark and re-parent are the same single edge,
 * and @stop is retired WHOLE exactly as an uncut copy retires it.  @cut is 0 for
 * every other caller and must be < len; the bitmap arms reject a non-zero one,
 * having no run to cut.  Three things are particular to this arm:
 *  - the CHILD COUNT is metadata, so setting ->child does not maintain it the way
 *    the bitmap arms' set_nth calls do; cds_ft_verify cross-checks it.
 *  - the caller gets the PLAIN node flag back.  The skip form cannot be inverted
 *    before the commit (it names the child, and the node is recovered through that
 *    child's back-pointer, which still names the original), so a caller freeing
 *    the copy on a later bail would free the live original.
 *  - a SKIP-encoded @stop never arrives: the encoding is a group property that
 *    EAGER clears, and a rekey requires EAGER.  Asserted, not handled.
 */
/*
 * Lock one child of the COW'd stop node -- the §7.2 fan-out.  Every child of one
 * node sits at the SAME byte-depth, so under a coarse spacing they either all
 * share one anchor on the path above (which this op frequently already holds, so
 * the collapse is 256 -> 0) or each is its own, and no two can collide.  Either
 * way the dedupe in the choke point is what decides it -- no set is needed here.
 */
static inline
int ft_rekey_cow_lock_child(const struct cds_ft *ft,
		const struct ft_lock_ctx *ctx, struct cds_ft_inode_flag *child,
		struct cds_ft_metadata *cm, unsigned int child_depth,
		struct ft_held_anchor *held)
{
	return ft_acquire_member(ft, ctx, child, cm, child_depth, held);
}

static
int ft_rekey_cow_stop(struct cds_ft *ft, const struct ft_lock_ctx *ctx,
		struct ft_flip_txn *txn,
		struct cds_ft_inode_flag *stop_flag, unsigned int stop_depth,
		unsigned int cut,
		struct cds_ft_inode_flag **stop_prime_ret,
		struct ft_held_anchor *marks,
		unsigned int *nr_marks)
{
	bool compressed = ft_node_compressed(stop_flag);
	/*
	 * ft_compressed_node_ptr, NOT ft_skip_to_compressed: the two accessors are
	 * not interchangeable.  The skip one masks the length bits off and treats
	 * what is left as the run's CHILD, recovering the node through that child's
	 * back-pointer -- so handing it a PLAIN compressed flag makes it read a
	 * child pointer out of a flag that is not one (measured: a segfault in the
	 * first re-parent, on a garbage child).  A skip-encoded @stop_flag never
	 * arrives here; the caller's shape gate refuses it.
	 */
	struct cds_ft_compressed_node *stop_cn = compressed ?
		ft_compressed_node_ptr(stop_flag) : NULL;
	unsigned int ti = compressed ? 0 : ft_node_type(stop_flag);
	const struct cds_ft_type *type = &ft_types[ti];
	struct cds_ft_inode *stop_node = compressed ?
		(struct cds_ft_inode *) stop_cn : ft_node_ptr(stop_flag);
	struct cds_ft_metadata *stop_meta = cds_ft_item_to_metadata(stop_node);
	struct cds_ft_inode *new_node = NULL;
	struct cds_ft_metadata *new_meta;
	struct cds_ft_inode_flag *new_flag;
	bool new_init_done = false;
	struct ft_held_anchor stop_held;
	/*
	 * Every child of @stop starts at the SAME byte-depth: one hop past a
	 * bitmap node, past the whole run for a compressed one.  That single
	 * depth is what the §7.2 fan-out hoists on.
	 */
	unsigned int child_depth;
	unsigned int nm = 0, i;
	int ret;
	/*
	 * THIS function's held set.  @marks is where its acquires live -- @stop's
	 * fence and one per child -- and none of them reaches a txn registry until
	 * the caller's sweep, so a child acquire consulting the CALLER's context
	 * cannot see them.  Under a coarse spacing every child of @stop anchors on
	 * the path above, which is frequently @stop itself, and the op then refuses
	 * its own fence.  @nr_extra is refreshed at each acquire because the fan-out
	 * is still growing it.
	 */
	struct ft_lock_ctx cctx;

	*stop_prime_ret = NULL;
	*nr_marks = 0;
	/*
	 * @structural_sw is the claim that matters: this body parks SW.  The
	 * EXCLUSION behind it is mode-dependent -- the per-node DLM locks under
	 * lock_fine, the FT-wide mutex (CDS_FT_SCOPED_WRITER, taken by
	 * ft_rekey_graft_simple_locked) under coarse -- so asserting lock_fine
	 * here asserted one mode's mechanism, not the property.
	 */
	assert(txn->structural_sw);
	/*
	 * A cut names a byte offset INTO a run, so only the compressed arm can
	 * honour one, and it must leave at least one byte behind: a zero-length
	 * run is not a node.
	 */
	assert(!cut || (compressed && cut < (unsigned int) stop_cn->len));
	assert(compressed || type->type_class == FT_POPCOUNT ||
		type->type_class == FT_PIGEON);
#ifdef FEATURE_FT_SKIP_COMPRESSED
	assert(!ft_node_skip_compressed(stop_flag));	/* caller's gate */
#endif
	/*
	 * A CO-LOCATED EXTERNAL CHAIN is in scope: it is one forward pointer to
	 * copy plus one back edge to record, the same shape as a child.  What stays
	 * out of scope is an S_top that IS an external head -- see the caller.
	 */

	/* 1. Acquire @stop's retire lock -- the fence BEFORE any body read. */
	if (ft_acquire_member(ft, ctx, stop_flag, stop_meta, stop_depth,
			&stop_held))
		return -EAGAIN;
	marks[nm++] = stop_held;
	*nr_marks = nm;
	/*
	 * A held set carries ONE out-of-registry array, so a caller keeping marks
	 * of its own would lose them here.  No caller does; assert it rather than
	 * let a future one lose its exclusion silently.
	 */
	assert(!ctx || !ctx->held.nr_extra);
	ft_lock_ctx_init(&cctx, ft_lock_ctx_descent(ctx), txn, ctx ? ctx->op : NULL);
	cctx.held.extra = marks;

	/* <=2 edges/child (parent + pso) + 1 retire; caller reserves its publish. */
	if (!ft_flip_txn_reserve_extra(txn, compressed ? 2 + 1 :
			2 * ft_meta_nr_child_load(stop_meta) + 1)) {
		ret = -ENOMEM;
		goto out;
	}

	if (compressed) {
		/*
		 * 2'. + 3'. + 4'.  The one-child form: copy the run and its child,
		 *     then mark and re-parent that child.  The source child is read
		 *     RESOLVED through the txn for the same reason the bitmap arms
		 *     resolve theirs -- a peer's parked flip proxy must never be
		 *     embedded in the copy.
		 */
		struct cds_ft_compressed_node *new_cn;
		struct cds_ft_inode_flag *child;
		struct cds_ft_metadata *cm;
		void *resolved;

		unsigned int tail_len = (unsigned int) stop_cn->len - cut;

		new_cn = alloc_compressed_node(ft, tail_len, &new_meta);
		if (!new_cn) {
			ret = -ENOMEM;
			goto out;
		}
		new_node = (struct cds_ft_inode *) new_cn;
		new_cn->len = tail_len;
		memcpy(new_cn->key_bytes, stop_cn->key_bytes + cut, tail_len);
		if (!ft_flip_txn_resolve_prio(txn, (void **) &stop_cn->child,
				&resolved)) {
			ret = -EAGAIN;
			goto abandon;
		}
		child = (struct cds_ft_inode_flag *) resolved;
		new_cn->child = child;
		/*
		 * The child count is metadata, not a body field, so setting ->child
		 * does not maintain it the way the bitmap arms' set_nth calls do --
		 * and cds_ft_verify cross-checks it against cn->child's presence
		 * ("compressed node nr_child 0 does not match cn->child presence").
		 * Carry it over as ft_compact's clone does; it is 1 for a compressed
		 * node.
		 */
		ft_meta_nr_child_set(new_meta, ft_meta_nr_child(stop_meta));
		ft_nr_keys_store(ft, new_meta, ft_nr_keys_get(stop_meta),
			CMM_RELAXED);
		new_flag = ft_compressed_node_flag(new_cn);
		child_depth = stop_depth + stop_cn->len;
		cm = ft_child_state_meta(ft, child);
		if (cm) {
			uintptr_t csnap;

			cctx.held.nr_extra = nm;
			if (ft_rekey_cow_lock_child(ft, &cctx, child, cm,
					child_depth, &marks[nm])) {
				ret = -EAGAIN;
				goto abandon;
			}
			csnap = marks[nm].node_snap;
			nm++;
			*nr_marks = nm;
		}
		ft_reparent_record(ft, txn, child, new_flag, &new_cn->child,
			/*child_marked=*/ cm != NULL, /*hold_ctx=*/ NULL);
		/*
		 * 5'. Retire @stop and hand back the NODE flag, not the skip-encoded
		 *     slot form.
		 *
		 * ★ THE SKIP FORM IS NOT INVERTIBLE HERE, which is why the caller gets
		 * the plain one.  A skip value names the CHILD plus the run length, and
		 * ft_skip_to_compressed recovers the node through that child's
		 * back-pointer -- which, until this txn commits, still names the OLD
		 * node.  A caller that had to free the copy on a later bail would
		 * therefore resolve the skip flag to the LIVE ORIGINAL and free that.
		 * The plain flag inverts with ft_compressed_node_ptr and cannot.
		 */
		if (!stop_held.shared)
			ft_flip_txn_record_anchor_release(txn, &stop_held,
				stop_meta);
		ft_flip_txn_record_retire_anchored(txn, ctx, &stop_held,
			stop_meta);
		*stop_prime_ret = new_flag;
		return 0;
	}

	/* 2. Fresh same-type, same-capacity copy (zeroed; body rebuilt below). */
	new_node = alloc_cds_ft_node(ft, type, &new_meta);
	if (!new_node) {
		ret = -ENOMEM;
		goto out;
	}
	new_flag = ft_node_flag(new_node, ti);
	child_depth = stop_depth + 1;	/* a bitmap node spans ONE key byte */
	ft_nr_keys_store(ft, new_meta, ft_nr_keys_get(stop_meta), CMM_RELAXED);

	/*
	 * 3. Copy children @stop -> @stop', each source slot RESOLVED (frozen
	 *    under @stop's fence) so a peer's parked flip proxy is never embedded.
	 *    Verbatim positions -> identity-preserving; interior stays shared.
	 */
#define COW_IS_INIT(bv) ({						\
	bool __i = false;						\
	if (type->type_class == FT_POPCOUNT) {				\
		__i = !new_init_done;					\
		new_init_done = true;					\
	}								\
	__i;								\
})
	if (type->type_class == FT_POPCOUNT) {
		uint8_t nc = ft_popcount_node_get_nr_child(type, stop_node);

		for (i = 0; i < nc; i++) {
			struct cds_ft_inode_flag *iter, **src_slot;
			void *resolved;
			uint8_t v;

			ft_popcount_node_get_ith_pos(type, stop_node, i, &v, &iter);
			if (!iter)
				continue;
			ft_node_get_nth_skip(stop_flag, &src_slot, v, FT_PF_NONE);
			if (!ft_flip_txn_resolve_prio(txn, (void **) src_slot,
					&resolved)) {
				ret = -EAGAIN;
				goto abandon;
			}
			iter = (struct cds_ft_inode_flag *) resolved;
			if (!iter)
				continue;
			if (type->popcount_2l)
				ret = ft_popcount_2l_node_set_nth(type, new_node,
						new_meta, v, iter, COW_IS_INIT(v));
			else if (type->popcount_1l)
				ret = ft_popcount_1l_node_set_nth(type, new_node,
						new_meta, v, iter, COW_IS_INIT(v));
			else
				ret = _ft_node_set_nth(ft, type, new_node, new_flag,
						new_meta, v, iter, COW_IS_INIT(v),
						true, NULL);
			assert(!ret);
		}
	} else {	/* FT_PIGEON */
		for (i = 0; i < FT_ENTRY_PER_NODE; i++) {
			struct cds_ft_inode_flag *iter, **src_slot;
			void *resolved;

			iter = ft_pigeon_node_get_ith_pos(type, stop_node, i);
			if (!iter)
				continue;
			ft_node_get_nth_skip(stop_flag, &src_slot, (uint8_t) i,
					FT_PF_NONE);
			if (!ft_flip_txn_resolve_prio(txn, (void **) src_slot,
					&resolved)) {
				ret = -EAGAIN;
				goto abandon;
			}
			iter = (struct cds_ft_inode_flag *) resolved;
			if (!iter)
				continue;
			if (type->popcount_2l)
				ret = ft_popcount_2l_node_set_nth(type, new_node,
						new_meta, (uint8_t) i, iter,
						COW_IS_INIT((uint8_t) i));
			else if (type->popcount_1l)
				ret = ft_popcount_1l_node_set_nth(type, new_node,
						new_meta, (uint8_t) i, iter,
						COW_IS_INIT((uint8_t) i));
			else
				ret = _ft_node_set_nth(ft, type, new_node, new_flag,
						new_meta, i, iter,
						COW_IS_INIT((uint8_t) i), true, NULL);
			assert(!ret);
		}
	}
#undef COW_IS_INIT

	/*
	 * 3b. The CO-LOCATED EXTERNAL CHAIN, if any: the key that ends exactly at
	 *     @stop.  Its head is APP-OWNED and never copied -- only the forward
	 *     pointer moves to the copy, and the head's back edge is recorded like
	 *     any child's.  ft_reparent_record dispatches on the child kind, so it
	 *     writes whichever back-channel this group uses (the cell's parent with
	 *     the list on, the head's own prev with it off); there is no mark,
	 *     because an external head carries no state word (ft_child_state_meta
	 *     returns NULL for one).
	 */
	if (stop_meta->external_nodes) {
		new_meta->external_nodes = stop_meta->external_nodes;
		ft_reparent_record(ft, txn,
			(struct cds_ft_inode_flag *) ft_dereference_external(
				new_meta->external_nodes),
			new_flag,
			(struct cds_ft_inode_flag **) &new_meta->external_nodes,
			/*child_marked=*/ false, /*hold_ctx=*/ NULL);
	}

	/*
	 * 4. MARK each metadata-bearing child + record its SW re-parent onto
	 *    @stop'.  Iterate the FRESH node's children (post-copy).  The mark must
	 *    precede ft_reparent_record so its pso edge's new_state (LOCK masked
	 *    out) releases the mark at the commit flip.
	 */
	if (type->type_class == FT_POPCOUNT) {
		uint8_t nc = ft_popcount_node_get_nr_child(type, new_node);

		for (i = 0; i < nc; i++) {
			struct cds_ft_inode_flag *iter, **slot = NULL;
			struct cds_ft_metadata *cm;
			uint8_t v;

			ft_popcount_node_get_ith_pos(type, new_node, i, &v, &iter);
			if (!iter)
				continue;
			ft_node_get_nth_skip(new_flag, &slot, v, FT_PF_NONE);
			cm = ft_child_state_meta(ft, iter);
			if (cm) {
				uintptr_t csnap;

				cctx.held.nr_extra = nm;
				if (ft_rekey_cow_lock_child(ft, &cctx, iter, cm,
						child_depth, &marks[nm])) {
					ret = -EAGAIN;
					goto abandon;
				}
				csnap = marks[nm].node_snap;
				nm++;
				*nr_marks = nm;
			}
			ft_reparent_record(ft, txn, iter, new_flag, slot,
				/*child_marked=*/ true,		/* marked above */
				/*hold_ctx=*/ NULL);
		}
	} else {	/* FT_PIGEON */
		for (i = 0; i < FT_ENTRY_PER_NODE; i++) {
			struct cds_ft_inode_flag *iter, **slot = NULL;
			struct cds_ft_metadata *cm;

			iter = ft_pigeon_node_get_ith_pos(type, new_node, i);
			if (!iter)
				continue;
			ft_node_get_nth_skip(new_flag, &slot, (uint8_t) i, FT_PF_NONE);
			cm = ft_child_state_meta(ft, iter);
			if (cm) {
				uintptr_t csnap;

				cctx.held.nr_extra = nm;
				if (ft_rekey_cow_lock_child(ft, &cctx, iter, cm,
						child_depth, &marks[nm])) {
					ret = -EAGAIN;
					goto abandon;
				}
				csnap = marks[nm].node_snap;
				nm++;
				*nr_marks = nm;
			}
			ft_reparent_record(ft, txn, iter, new_flag, slot,
				/*child_marked=*/ true,		/* marked above */
				/*hold_ctx=*/ NULL);
		}
	}

	/*
	 * 5. Retire @stop (SW fenced tombstone {LOCK|snap -> TOMBSTONE|snap}).
	 *    A peer's ft_meta_nr_child_inc(@stop) now honors the mark and spins, so
	 *    the SW park is not clobbered.  Not registered -- the caller's sweep
	 *    owns clearing (marks[0]).
	 */
	if (!stop_held.shared)
		ft_flip_txn_record_anchor_release(txn, &stop_held, stop_meta);
	ft_flip_txn_record_retire_anchored(txn, ctx, &stop_held, stop_meta);

	*stop_prime_ret = new_flag;
	return 0;

abandon:
	/* Each kind to its own arena: a compressed copy came from the other one. */
	if (compressed)
		free_compressed_node_unpublished(ft,
			(struct cds_ft_compressed_node *) new_node);
	else
		free_cds_ft_node_unpublished(ft, new_node);
out:
	return ret;	/* marks[0..*nr_marks) swept by the caller */
}

/* ------------- moved from fractal-trie.c ------------- */

/*
 * Does the located ordered-list splice pair (@pred, @succ) really BRACKET the dst
 * key range in key order?  ft_ord_cell_find_splice_pos derives the pair from a
 * RELATIONAL (inequality) descent, and relational reads are NOT coherent under
 * concurrent structural mutation -- a peer rekey in flight can make that descent
 * return a pair that is genuinely ADJACENT in the list but sits at the WRONG key
 * position.  Every edge of the splice then validates (the pair IS adjacent) and
 * the commit installs the run in the wrong place: an ordered list that is a
 * well-formed doubly-linked list yet no longer key-ordered.
 *
 * So re-derive each neighbour's key from the STRUCTURE (the same up-walk the
 * rekey-coherent reader uses, ordinal space, right-aligned in @scratch) and
 * require pred < the dst range < succ.  @dst_key is a PREFIX, so the range is
 * every key that extends it, and ft_rekey_prefix_range_cmp is what places a
 * neighbour against that range in either group flavour.
 *
 * A sentinel / absent neighbour has no key, but it is NOT exempt: it ASSERTS that
 * the run belongs at the list end, so the assertion itself is what gets checked --
 * an absent pred means succ must really be the list MINIMUM, an absent succ means
 * pred must really be the list MAXIMUM.  Skipping that (a first cut did) leaves the
 * incoherent derivation a way through: a spurious "no key below the dst range"
 * (find_splice_pos returns pred == NULL and then probes GT) paired with a succ that
 * legitimately sorts above the range passes a key-only check, and the pair reaches
 * the endpoint-adjacency guard as a bogus PERMANENT -EINVAL.  That was the last
 * residual failure of the shared-junction oracle (~1 run in 70).
 */
/*
 * Where does the ordinal key @kb (@klen bytes) sit relative to the RANGE of keys
 * the prefix @dst_ord (@dst_len bytes) covers?  -1 below it, 0 inside it, 1 above.
 *
 * The trie's key order compares bytes over the COMMON length and, on a tie, puts
 * the SHORTER key first (measured: 'aa' < 'ab' < 'abc' < 'abd' < 'b').  So the
 * range is [@dst_ord itself .. every key extending it]: a strict PREFIX of
 * @dst_ord sorts BELOW the range, and @dst_ord itself is INSIDE.  A fixed-length
 * group is the special case where no key is shorter than @dst_len, which is why
 * this used to be expressible as a plain compare against @dst_ord padded with the
 * ordinal extremes -- that padding is what tied the check to fixed lengths, and
 * comparing over the common length instead needs no padding at all.
 */
static
int ft_rekey_prefix_range_cmp(const uint8_t *kb, size_t klen,
		const uint8_t *dst_ord, size_t dst_len)
{
	size_t n = klen < dst_len ? klen : dst_len;
	int cmp = memcmp(kb, dst_ord, n);

	if (cmp != 0)
		return cmp < 0 ? -1 : 1;
	return klen < dst_len ? -1 : 0;
}

/*
 * Free the UNPUBLISHED S_top copy ft_rekey_cow_stop made, whichever kind it is.
 * The two kinds come from different arenas, and the compressed one is handed back
 * as a plain node flag precisely so this inversion is safe (see cow_stop).
 */
static
void ft_rekey_free_stop_prime(struct cds_ft *ft, struct cds_ft_inode_flag *nf)
{
	if (!nf)
		return;
	if (ft_node_compressed(nf))
		free_compressed_node_unpublished(ft, ft_compressed_node_ptr(nf));
	else
		free_cds_ft_node_unpublished(ft, ft_node_ptr(nf));
}

#ifdef FEATURE_FT_MERGE
/*
 * Build the ORDERED-CELL edge set for an IN-TRIE interleave: the moved run's keys
 * thread individually between the destination region's, and every relink is
 * RECORDED so the whole reorder lands in the move's one commit.
 *
 * This is the case a run splice cannot express.  When the moved suffixes do not
 * disjointly precede or follow the region's, the moved cells do not stay
 * contiguous, so there is no single pair of boundary edges to write -- each
 * survivor lands between two region cells.  ft_merge_ord_interleave_collect
 * computes exactly that order; what it needs from an in-trie caller is
 * @record_all, because its survivors are live in the list being rebuilt rather
 * than arriving from a consumed source list.
 *
 * ON TOP OF THE COLLECT, two things the cross-trie merge never needs:
 *  - the SRC GAP.  A cross-trie merge throws its source list away; here the run
 *    vacates a position in the same list, so its old neighbours must be stitched
 *    to each other (ft_ord_cell_run_detach_edges, 2 edges).
 *  - an ADJACENCY refusal.  The run and the region are each contiguous and
 *    disjoint in the CURRENT list (their key prefixes are disjoint; they only
 *    interleave AFTER the move), so the ONLY way an edge slot can repeat is the
 *    two runs abutting -- then the gap closure and the collect's boundary edges
 *    name the same links, and one of the collect's seeds would be a cell that is
 *    itself moving.  Refused rather than special-cased.
 *
 * A COLLISION is refused too, and for a reason the collect's own contract states:
 * it drops a colliding src head on the premise that the cell becomes an
 * unreachable floating duplicate, which holds only when the src list is consumed.
 * In-trie that cell stays linked where it is and would keep answering as a
 * distinct key, so it would have to be unlinked as well -- a further step this
 * does not take.  With @record_all the collect stores nothing, so running it and
 * discarding the result is how the check is made.
 *
 * Returns 0 with *@edges_ret (caller frees) and *@n_ret, or -EINVAL (a shape
 * above), -EAGAIN (a torn read) or -ENOMEM.  Records nothing itself.
 */
static
int ft_rekey_ord_interleave(struct cds_ft *ft, struct cds_ft_inode_flag *D,
		size_t dst_len, size_t src_len,
		struct cds_ft_node *run_rfirst, struct cds_ft_node *run_rlast,
		unsigned long merged_keys,
		struct ft_ord_cell_edge **edges_ret, unsigned int *n_ret)
{
	size_t max_len = ft->group->max_key_len;
	struct ft_ord_cell *rfc, *rlc, *run_pred, *run_succ;
	struct ft_ord_cell *dfirst, *dlast, *reg_pred, *reg_succ;
	struct ft_merge_src_cap *caps = NULL;
	uint8_t *pool = NULL;
	size_t pool_cap = 0, pool_len = 0;
	struct ft_ord_cell_edge *edges = NULL;
	unsigned long nsrc = 0, ncollide = 0, cap_n;
	unsigned int n;
	struct ft_ord_cell *sc, *slast;
	int ret;

	*edges_ret = NULL;
	*n_ret = 0;
	rfc = ft_ord_cell_ptr(rcu_dereference(run_rfirst->prev));
	rlc = ft_ord_cell_ptr(rcu_dereference(run_rlast->prev));
	dfirst = ft_ord_cell_ptr(rcu_dereference(
		ft_subtree_minmax_head(ft, D, false)->prev));
	dlast = ft_ord_cell_ptr(rcu_dereference(
		ft_subtree_minmax_head(ft, D, true)->prev));
	if (!rfc || !rlc || !dfirst || !dlast)
		return -EAGAIN;
	run_pred = ft_ord_cell_resolve_ord(&rfc->lnode.prev);
	run_succ = ft_ord_cell_resolve_ord(&rlc->lnode.next);
	reg_pred = ft_ord_cell_resolve_ord(&dfirst->lnode.prev);
	reg_succ = ft_ord_cell_resolve_ord(&dlast->lnode.next);
	/* The two runs must not abut, in either order (see the header). */
	if (run_succ == dfirst || run_pred == dlast ||
			reg_pred == rlc || reg_succ == rfc)
		return -EINVAL;

	/*
	 * Capture the run's key SUFFIXES while it is still attached and
	 * up-walkable, exactly as ft_merge_spine_copy does: the merge order is
	 * rebuilt from the two live runs, never from the about-to-be-published
	 * structure, so nothing needs a proxy installed first.
	 */
	caps = (struct ft_merge_src_cap *) malloc((merged_keys + 8) *
			sizeof(*caps));
	if (!caps)
		return -ENOMEM;
	sc = rfc;
	slast = rlc;
	for (;;) {
		uint8_t sbuf[FT_MAX_KEY_LEN];
		size_t sfl = ft_rebuild_key_upwalk(ft, sc, sbuf, max_len);
		size_t suf_len;

		if (sfl < src_len || nsrc >= merged_keys + 8) {
			ret = -EAGAIN;		/* torn up-walk, or the run grew */
			goto out;
		}
		suf_len = sfl - src_len;
		if (pool_len + suf_len > pool_cap) {
			size_t ncap = pool_cap ? pool_cap * 2 : 256;
			uint8_t *np;

			while (ncap < pool_len + suf_len)
				ncap *= 2;
			np = (uint8_t *) realloc(pool, ncap);
			if (!np) {
				ret = -ENOMEM;
				goto out;
			}
			pool = np;
			pool_cap = ncap;
		}
		memcpy(pool + pool_len, sbuf + (max_len - sfl) + src_len,
			suf_len);
		caps[nsrc].cell = sc;
		caps[nsrc].suffix_off = pool_len;
		caps[nsrc].suffix_len = suf_len;
		pool_len += suf_len;
		nsrc++;
		if (sc == slast)
			break;
		sc = ft_ord_cell_resolve_ord(&sc->lnode.next);
		if (!sc) {
			ret = -EAGAIN;
			goto out;
		}
	}

	/*
	 * <= 2 visible edges per survivor run + 2 boundary, plus (record_all) up to
	 * 2 per survivor for its own links, plus the 2 src-gap edges.
	 */
	cap_n = 2 * merged_keys + 2 + 2 * nsrc + FT_ORD_CELL_RUN_DETACH_MAX_EDGES;
	edges = (struct ft_ord_cell_edge *) calloc(cap_n, sizeof(*edges));
	if (!edges) {
		ret = -ENOMEM;
		goto out;
	}
	n = ft_merge_ord_interleave_collect(ft, dst_len, dfirst, reg_succ,
			reg_pred, caps, nsrc, pool, edges, /*record_all=*/ true,
			&ncollide);
	if (ncollide) {
		ret = -EINVAL;			/* see the header */
		goto out;
	}
	/* Close the gap the run vacates. */
	n = ft_ord_cell_run_detach_edges(ft, run_rfirst, run_rlast, &rfc, &rlc,
			edges, n);
	assert(n <= cap_n);
	*edges_ret = edges;
	*n_ret = n;
	edges = NULL;
	ret = 0;
out:
	free(edges);
	free(pool);
	free(caps);
	return ret;
}
#endif /* FEATURE_FT_MERGE: the only caller is merge-only */

/*
 * Order two key SUFFIXES the way the trie orders keys: bytes over the common
 * length, and on a tie the shorter one first.
 */
static
int ft_rekey_suffix_cmp(const uint8_t *a, size_t alen, const uint8_t *b,
		size_t blen)
{
	size_t n = alen < blen ? alen : blen;
	int cmp = memcmp(a, b, n);

	if (cmp != 0)
		return cmp < 0 ? -1 : 1;
	if (alen == blen)
		return 0;
	return alen < blen ? -1 : 1;
}

/*
 * An OCCUPIED destination: does the moved run land entirely BELOW the merge
 * region already there (-1), entirely ABOVE it (1), or INTERLEAVED with it (0)?
 *
 * Both sides are compared by the suffix BELOW their own merge point, which is what
 * the merged order is decided on -- the moved keys become @dst_key ++ suffix, and
 * the region's are @dst_key ++ their own, so the shared prefix cancels.
 *
 * ★ WHY THE ANSWER MATTERS SO MUCH.  Entirely below or entirely above, the moved
 * cells stay ONE CONTIGUOUS list range and the move is the same run splice an empty
 * destination gets -- at the region's front or back rather than into a gap.
 * INTERLEAVED, it is not a run move at all: the cells have to be threaded
 * individually between the region's, which is ft_merge_ord_interleave_collect's
 * job, and that helper PLAIN-STORES each surviving cell's links on the premise
 * that the cell "is not ord-reachable in @dst -- never was".  True for the
 * cross-trie merge it was written for, FALSE here: an in-trie rekey's source cells
 * are live in the very list being rebuilt, so those stores would be
 * reader-visible and non-atomic.  Interleaving in-trie needs a mode that RECORDS
 * every relink instead, so it is refused rather than approximated.
 *
 * A COLLISION cannot occur in the two cases this admits: identical full keys mean
 * identical suffixes, which strict disjointness excludes.  That is what keeps the
 * duplicate-chain absorption out of the picture here.
 */
static
int ft_rekey_run_vs_region(struct cds_ft *ft,
		struct ft_ord_cell *rfc, struct ft_ord_cell *rlc, size_t src_len,
		struct ft_ord_cell *dfirst, struct ft_ord_cell *dlast,
		size_t dst_len)
{
	size_t max_len = ft->group->max_key_len;
	uint8_t rb[FT_MAX_KEY_LEN], db[FT_MAX_KEY_LEN];
	size_t rl, dl;

	/* run MAX vs region MIN: below iff strictly less. */
	rl = ft_rebuild_key_upwalk(ft, rlc, rb, max_len);
	dl = ft_rebuild_key_upwalk(ft, dfirst, db, max_len);
	if (!rl || !dl || rl < src_len || dl < dst_len)
		return 0;			/* unreadable: treat as interleaved */
	if (ft_rekey_suffix_cmp(rb + (max_len - rl) + src_len, rl - src_len,
			db + (max_len - dl) + dst_len, dl - dst_len) < 0)
		return -1;
	/* run MIN vs region MAX: above iff strictly greater. */
	rl = ft_rebuild_key_upwalk(ft, rfc, rb, max_len);
	dl = ft_rebuild_key_upwalk(ft, dlast, db, max_len);
	if (!rl || !dl || rl < src_len || dl < dst_len)
		return 0;
	if (ft_rekey_suffix_cmp(rb + (max_len - rl) + src_len, rl - src_len,
			db + (max_len - dl) + dst_len, dl - dst_len) > 0)
		return 1;
	return 0;
}

/*
 * TRI-STATE, and the third value is the whole point: 1 the pair brackets the
 * dst range, 0 the derivation was TORN (transient -- re-derive), -1 a
 * neighbour is STRUCTURALLY inside the range.
 *
 * ☠ THE TWO FAILURES ARE NOT THE SAME FAILURE, and answering both with a retry
 * is an infinite loop.  A torn read is a peer restructuring this neighbourhood
 * and clears on its own.  A neighbour inside the range means a key legitimately
 * EXTENDS the dst prefix -- the destination is occupied -- and no retry can
 * change that: the op re-descends, re-derives the identical pair, and refuses
 * again forever.
 *
 * That shape is reachable whenever the caller's @merge_dst probe answered NO
 * for a destination that is in fact occupied, which it does by construction: the
 * probe stops at a COMPRESSED (or external / skip) node rather than decode it,
 * so an occupied destination reached through a compressed run reads as an empty
 * one.  The premise this check was written under -- "the dst prefix is empty
 * here" -- therefore does not hold, and the caller must answer the miss with a
 * SHAPE refusal the dispatcher can fall back on.
 */
static
int ft_rekey_splice_pos_brackets(struct cds_ft *ft, const uint8_t *dst_ord,
		size_t dst_len, struct ft_ord_cell *pred,
		struct ft_ord_cell *succ)
{
	size_t max_len = ft->group->max_key_len;
	uint8_t scratch[FT_MAX_KEY_LEN];
	struct ft_ord_cell *sentinel = ft_ord_sentinel_cell(ft);
	bool pred_end = !pred || pred == sentinel;
	bool succ_end = !succ || succ == sentinel;
	size_t klen;

	if (pred_end && succ_end)
		return 0;			/* "empty list" -- the run is IN it */
	if (pred_end && ft_ord_first(ft) != succ)
		return 0;			/* head insert, but succ is not the min */
	if (succ_end && ft_ord_last(ft) != pred)
		return 0;			/* tail insert, but pred is not the max */
	if (pred && pred != sentinel) {
		klen = ft_rebuild_key_upwalk(ft, pred, scratch, max_len);
		if (!klen)
			return 0;		/* unreadable: torn */
		if (ft_rekey_prefix_range_cmp(scratch + (max_len - klen),
				klen, dst_ord, dst_len) >= 0)
			return -1;		/* a key extends the dst prefix */
	}
	if (succ && succ != sentinel) {
		klen = ft_rebuild_key_upwalk(ft, succ, scratch, max_len);
		if (!klen)
			return 0;		/* unreadable: torn */
		if (ft_rekey_prefix_range_cmp(scratch + (max_len - klen),
				klen, dst_ord, dst_len) <= 0)
			return -1;		/* a key extends the dst prefix */
	}
	return 1;
}

/*
 * THREE KINDS OF REFUSAL, deliberately distinct return codes, because they have
 * different futures and only one of them is the caller's fault:
 *
 *   -EINVAL              ARGUMENT.  The caller got it wrong and no state of the
 *                        trie would make the call legal (zero/over-long key,
 *                        unequal lengths, src and dst overlapping).  Terminal;
 *                        surfaces as CDS_FT_STATUS_INVALID_ARGUMENT_ERROR.
 *
 *   FT_REKEY_UNCOVERED   SHAPE or MODE.  Nothing is wrong with the arguments or
 *                        the trie -- THIS writer does not cover it (a coarse
 *                        trie, a skip-compressed source, a boundary parent at
 *                        its min_child floor, ...).  A gap in the one-decide
 *                        cut that widening it closes, so it is the only code
 *                        the dispatcher may answer with a fallback.
 *
 *   -ENOTSUP             BUILD.  The feature is compiled out
 *                        (-DNO_FEATURE_FT_MERGE).  Terminal, and no runtime
 *                        state or wider cut can change it.
 *
 * They were ALL -EINVAL, which made "the caller passed nonsense" and "this
 * writer cannot express this shape" indistinguishable at the dispatch -- so the
 * fallback ran for both, and an argument error was answered by staging a move
 * that then failed differently, or worse, succeeded.
 */
#define FT_REKEY_UNCOVERED	(-EDOM)


/*
 * A FOLDED COLLAPSE'S TWO LIFETIMES (struct ft_chain_compress_reclaim).  The
 * collapse records into this op's txn and commits nothing, so it hands its
 * nodes back rather than freeing them: at the point it returns, the chain it
 * retires is still LIVE AND LINKED and a reader is entitled to be walking it.
 * Which side of the commit each node belongs to is the same split
 * ft_detach_recompact_out draws between @old_node and @new_flag.
 *
 * RETIRED CHAIN -- freed when the commit SUCCEEDS, because that commit is what
 * unlinks them.  Deferred past readers by the free helpers themselves.
 */
static inline
void ft_rekey_collapse_free_retired(struct cds_ft *ft,
		struct ft_chain_compress_reclaim *rc)
{
	if (rc->boundary)
		free_cds_ft_node(ft, rc->boundary);
	if (rc->parent_cn)
		free_compressed_node(ft, rc->parent_cn);
	if (rc->child_cn)
		free_compressed_node(ft, rc->child_cn);
	memset(rc, 0, sizeof(*rc));
}

/*
 * An ELEVATING detach's ORPHAN CHAIN -- the ancestors its upward walk cleared,
 * plus the trailing skip-compressed target.  Same lifetime as the retired chain
 * above and for the same reason: THIS commit is what unlinks them, and their
 * freeze-on-free tombstones rode it, so they are reclaimed here and nowhere
 * else.  An ABORT leaves them live and linked, so no abort path frees them.
 */
static inline
void ft_rekey_detach_free_orphans(struct cds_ft *ft,
	struct ft_detach_recompact_out *rc)
{
	int i;

	if (rc->orphan_trailing)
		free_compressed_node(ft, ft_skip_to_compressed(ft,
			rc->orphan_trailing));
	for (i = 0; i < rc->nr_orphans; i++) {
		if (ft_node_compressed(rc->orphans[i]))
			free_compressed_node(ft,
				ft_compressed_node_ptr(rc->orphans[i]));
		else
			free_cds_ft_node(ft, ft_node_ptr(rc->orphans[i]));
	}
	rc->nr_orphans = 0;
	rc->orphan_trailing = NULL;
}

/*
 * MERGED NODE -- recorded but never published, so it is ours to reclaim when
 * the commit ABORTS or the op bails before it.  The retired chain is NOT freed
 * on these paths: nothing unlinked it, so it is still live.
 */
static inline
void ft_rekey_collapse_free_unpublished(struct cds_ft *ft,
		struct ft_chain_compress_reclaim *rc)
{
	if (rc->new_cn)
		free_compressed_node_unpublished(ft, rc->new_cn);
	memset(rc, 0, sizeof(*rc));
}

/*
 * Attempt age past which a refusal that COULD be a peer is treated as the shape
 * it also could be.  See the -1 arm of the bracket check below for why the two
 * are indistinguishable from one attempt.
 */
#define FT_REKEY_UNCOVERED_AFTER	4096

/*
 * HAND THE FOLD'S OUT-OF-REGISTRY MARKS TO @txn, at the TAKE.
 *
 * ft_rekey_cow_stop's fences live in a fn-scope array rather than a lock-SET --
 * it reaches 17 distinct anchors on the unit fixture, and a set is the unit the
 * MCAS install SORTS for deadlock-free acquisition, so the array is the right
 * home for them.  But "not a set" was read as "not in the registry either", and
 * the registry is a different question: it is what a RECORD-time owner check can
 * see (ft_flip_txn_owns reads locks[] and nothing else).
 *
 * Every later acquire of one of these words DEDUPES against the array through
 * @held.extra and correctly registers nothing of its own (ft_glue_acquire_
 * reparent_marks' @h.shared arm), so without this the registry never learns the
 * word by ANY route -- and the re-parent records those marks exist to license
 * name it as their owner.
 *
 * Registration TRANSFERS the clear (ft_unlock_held's header), so @txn_owned
 * follows it and the post-abort sweep must skip those entries -- one owner per
 * fence.  A SHARED mark transfers nothing: the acquire that FIRST took the word
 * owns both.  Idempotent, so a caller may re-run it as the array grows.
 *
 * The release records stay where they are (the marks loop before the commit):
 * ft_flip_txn_record_anchor_release_held reads the word through the txn and
 * chains onto whatever is already recorded there, so it is order-independent by
 * construction -- what was NOT order-independent is the registration, which
 * every record between here and there needs.
 */
static
void ft_rekey_marks_to_txn(struct ft_flip_txn *txn,
		struct ft_held_anchor *marks, unsigned int nr_marks)
{
	unsigned int i;

	for (i = 0; i < nr_marks; i++) {
		if (marks[i].shared || marks[i].txn_owned)
			continue;
		ft_flip_txn_lock_register(txn, marks[i].lock,
			marks[i].lock_snap);
		marks[i].txn_owned = true;
	}
}

static
int ft_rekey_graft_simple_attempt(struct cds_ft *ft,
		const uint8_t *src_key, size_t src_len,
		const uint8_t *dst_key, size_t dst_len,
		bool require_empty, struct urcu_txn *optxn)
{
	uint8_t src_ord[FT_MAX_KEY_LEN], dst_ord[FT_MAX_KEY_LEN];
	struct cds_ft_inode_flag *s_top, *s_top_prime = NULL, *attached_nf = NULL;
	struct cds_ft_node *run_rfirst = NULL, *run_rlast = NULL;
	struct ft_ord_cell *run_dpred = NULL, *run_dsucc = NULL;
	struct cds_ft_metadata *s_top_meta, *bp_meta;
	struct ft_detach_recompact_out detach_rc = { 0 };
	struct ft_flip_txn *txn;
	struct ft_glue glue;
	struct ft_graft_store_state gst_st = { 0 };	/* GLUE never runs prepare */
	struct ft_descent d_src, d_dst;
	/*
	 * The src descent is the anchor source for every acquire this attempt
	 * makes; @d_src is only valid once the walk below has run, so the
	 * context is (re)bound where it is used.
	 */
	struct ft_lock_ctx lctx_src;
	struct cds_ft_alloc_reserve reserve;
	struct ft_remove_pub pub = { .armed = false };
	/*
	 * The two nodes the graft's own step locks, whichever dst shape it took:
	 * NOSPLIT {dst parent, its parent}, GLUE {split compressed node, publish
	 * parent}.  @graft_c is the one it retires, @graft_p the one it releases.
	 */
	struct cds_ft_inode_flag *graft_c = NULL, *graft_p = NULL, *cn_flag = NULL;
	struct cds_ft_metadata *pp_meta = NULL;	/* GLUE publish-parent fence WE own */
	uintptr_t pp_snap = 0;
	/* ...unless it DEDUPED onto a word an earlier step of this op took. */
	bool pp_shared = false;
	/*
	 * INCREMENT 3: the dst point is OCCUPIED, so step 2 UNIONS into it with
	 * ft_merge_build instead of grafting a COW'd S_top' into a spare slot.  The
	 * merged cluster reuses @glue as its dst side (so every abort / free_old /
	 * fini path below applies unchanged); @src_glue is the extra src side, which
	 * carries the retire of S_top itself -- the merge does what cow_stop would.
	 */
	struct ft_glue src_glue;
#ifdef FEATURE_FT_MERGE
	struct ft_merge_ctx mctx;
	struct ft_merge_counts mcnt = { 0, 0, 0, 0, 0 };
#endif
	struct cds_ft_inode_flag *merged_nf = NULL;
	struct cds_ft_inode_flag *probe_D = NULL;	/* occupied dst merge point */
	unsigned long merged_keys = 0;
	bool merge_dst = false, src_glue_live = false;
	/*
	 * The dst position abuts the moved run, so the move leaves the run's ordered
	 * position alone -- see the splice-position derivation.
	 */
	bool run_keeps_pos = false;
	bool s_top_compressed = false;	/* the moved top is a compressed run */
	/*
	 * The src key ends INSIDE a run, @src_cut bytes into it.  Then S_top is
	 * not a node: it is the run's TAIL, and ft_rekey_cow_stop manufactures it
	 * by copying @key_bytes from this offset instead of from zero.  The
	 * junction is the run's OWN parent slot and the whole run is retired --
	 * a run has one child, so cutting it moves everything it held.  Zero for
	 * the ordinary shape, where S_top is a node the descent landed on.
	 */
	unsigned int src_cut = 0;
#ifdef FEATURE_FT_MERGE
	/* Occupied dst: thread the cells individually, do not splice a run. */
	bool run_interleaves = false;
#endif
	/*
	 * +2, not +1: cow_stop can fill S_top plus all FT_ENTRY_PER_NODE of its
	 * children, and the GLUE shape adds the split cluster's one displaced child.
	 */
	struct ft_held_anchor marks[FT_ENTRY_PER_NODE + 2];
	const uint8_t *ik;
	unsigned int nr_marks = 0, adepth = 0, i, ti;
	bool marks_consumed = false;
	bool src_parent_held;
	enum ft_graft_prep prep;
	enum cds_ft_status gst;
	enum urcu_txn_status gcst = URCU_TXN_STATUS_OK, st;
	unsigned long cnt;
	int ret;

	/*
	 * ARGUMENT refusals: the CALLER got it wrong, and no state of the trie
	 * would make this call legal.  Terminal -EINVAL, surfaced as
	 * CDS_FT_STATUS_INVALID_ARGUMENT_ERROR.
	 */
	if (src_len == 0 || dst_len == 0 ||
			src_len > FT_MAX_KEY_LEN || dst_len > FT_MAX_KEY_LEN)
		return -EINVAL;
	/*
	 * CAPABILITY refusal: nothing is wrong with the arguments or the trie --
	 * this WRITER does not cover the mode.  -ENOTSUP, distinct from -EINVAL,
	 * because the two have opposite futures: an argument error is permanent
	 * for every caller, while an uncovered mode is a gap in THIS writer that
	 * widening its cut closes.
	 */
	/*
	 * COARSE IS IN THIS WRITER'S CUT.  It was excluded for two reasons and
	 * both have since gone, so the gate that stood here is removed rather
	 * than re-argued.
	 *
	 * TERMINATION was the stated one: "with this gate lifted the retry loop
	 * NEVER TERMINATES on a coarse trie".  It does not reproduce.  Measured
	 * twice on inv_rekey_coarse_mixed_writers with both halves of the gate
	 * lifted -- 400 runs (2026-08-16) and again at 16,711 moves with 0
	 * retries -- and the refusal histogram inverts the story: coarse refuses
	 * ~0.47% per move and its dominant refusal is the final MCAS commit
	 * aborting, which is exactly what the retry lane absorbs.  FINE, the mode
	 * that ships, refuses ~390% per move and terminates fine.
	 *
	 * EXCLUSION was the real one, and it was fixed rather than refuted: this
	 * writer parks its structural edges SW, an SW park cannot fail, and on a
	 * coarse trie it used to take no writer scope at all -- so those parks
	 * arbitrated against nobody.  ft_rekey_graft_simple_locked now takes
	 * CDS_FT_SCOPED_WRITER around its whole retry loop, which IS the protocol
	 * every other coarse writer speaks (and is inert under lock_fine, where
	 * the per-node DLM locks are).
	 *
	 * ★ AND THAT IS CHECKED, not asserted: -DURCU_TXN_DEBUG_SETTLE reports
	 * every word a peer wrote between our park and our settle.  With the
	 * FT-wide scope deliberately removed (-DFT_RED_REKEY_NOLOCK, 68,655
	 * scopes skipped) it found 0 foreign writes in 1.37M settle records, and
	 * the detector itself is proven able to see that class
	 * (tests/unit/test_rcu_txn_settle_premise.c).
	 */
	ft_key_to_ordinals(src_ord, src_key, src_len, &ft->group->key_map);
	ft_key_to_ordinals(dst_ord, dst_key, dst_len, &ft->group->key_map);
	/*
	 * DISJOINT keys: neither may be a prefix of the other (equal keys being
	 * the degenerate case).  A prefix relationship puts one key inside the
	 * other's subtree, so the move is circular -- and it is ALSO what keeps
	 * the ordered-list splice sound now that the shape gate no longer forces
	 * src_len == dst_len by construction.  The run this move re-splices is
	 * exactly the keys under @src_key, an ordinally CONTIGUOUS range, so a
	 * splice neighbour strictly INTERIOR to that run would have to be
	 * bracketed by two run keys -- which puts @dst_key inside the run's own
	 * range, i.e. makes @src_key a prefix of it.  Rejecting that here leaves
	 * only the two ENDPOINT-adjacency shapes for the guard below to catch, so
	 * "no cell of the moved run is a splice neighbour" holds by construction.
	 * (A neighbour interior to a PEER's concurrently-moving run is a
	 * different, still-open question -- see the splice validation below.)
	 */
	if (memcmp(src_ord, dst_ord, src_len < dst_len ? src_len : dst_len) == 0)
		return -EINVAL;

	/*
	 * UNEQUAL LENGTHS ARE IN SCOPE, and what let them in was the destination
	 * probe below: while it stopped at a compressed node, every unequal-length
	 * shape refused anyway -- at a LATER gate -- because a longer @dst_key puts
	 * the destination behind exactly the path-compressed run the probe declined
	 * to decode.  Measured then, and it is why relaxing the length rule alone
	 * was reverted rather than landed.
	 *
	 * Nothing STORED needs rewriting: a node carries neither a depth nor a key
	 * length (cds_ft_metadata is parent_word / external_nodes /
	 * parent_slot_offset; cds_ft_node is prev / next), so a moved subtree's new
	 * key lengths are entirely a property of where its top hangs.
	 *
	 * Two things do follow from the length change, and both are handled the way
	 * ft_rekey_at_inner already handles them for the staged path:
	 *
	 *  - OVERFLOW.  A moved key K becomes @dst_key || (K minus the @src_key
	 *    prefix), so a longer destination can push it past the group's
	 *    max_key_len and overflow the fixed-size key buffers downstream (the
	 *    iterator's, the up-walk's).  Bound len(K) by the trie's own
	 *    max_used_key_len and REFUSE THE SHAPE rather than answering the
	 *    argument here: ft_rekey_at_inner owns that contract and returns
	 *    OVERFLOW_ERROR for it, and FT_REKEY_UNCOVERED is the one code that
	 *    falls through to it (see the dispatcher).  One place defines the
	 *    answer.
	 *
	 *  - max_used_key_len itself, raised by the caller on a committed move.
	 *
	 * ☠ AND THE FIXED-LENGTH FLAVOUR IS EXCLUDED HERE, EXPLICITLY.  It does
	 * reach this writer -- test_rekey_fixed_len_atomic_or_refused drives it --
	 * and the equal-length rule used to be what kept it safe: with the lengths
	 * free, moving a 2-byte prefix onto a 1-byte one SHORTENS every key it
	 * carries, and a group whose keys all have ONE length cannot express that.
	 * The refusal is a shape code, not an argument answer, for the same reason
	 * the overflow one is: ft_rekey_at_inner owns the "a rekey needs a
	 * variable-length group" contract and returns INVALID_ARGUMENT for it.
	 *
	 * ☠ AND THE JUNCTION GATE BELOW IS WHAT KEEPS THIS HONEST: with the lengths
	 * free the src and dst junctions no longer sit on the same level, so the
	 * cross-depth aliases it rejects are now REACHABLE rather than vacuous.
	 * They were written as rejections, not asserts, for exactly this day.
	 */
	if (src_len != dst_len) {
		size_t src_max;

		if (ft->group->key_len != CDS_FT_LEN_VARIABLE)
			return FT_REKEY_UNCOVERED;
		src_max = uatomic_load(&ft->max_used_key_len, CMM_RELAXED);
		if (src_max > src_len &&
				src_max - src_len >
				ft->group->max_key_len - dst_len)
			return FT_REKEY_UNCOVERED;
	}

	/*
	 * Descend src to S_top through plain internal nodes, capturing (nfp, pnfp,
	 * depth) so ft_detach_node can bootstrap the src-slot clear + BP nr_child--.
	 */
	ft_descent_init(&d_src, ft);
	ik = src_ord;
	while (d_src.depth < src_len) {
		if (!d_src.nf || ft_node_external(d_src.nf))
			return FT_REKEY_UNCOVERED;
#ifdef FEATURE_FT_SKIP_COMPRESSED
		if (ft_node_skip_compressed(d_src.nf))
			return FT_REKEY_UNCOVERED;
#endif
		/*
		 * A compressed run on the way DOWN to S_top is crossed whole, the
		 * way the dst probe crosses one: step() advances a single byte, so
		 * walking a run with it counts the bytes but never resolves into
		 * cn->child, landing depth-correct on NULL.
		 *
		 * A run that OVERSHOOTS @src_len is not out of scope either -- the
		 * src key simply ends inside it.  There is no node at that depth to
		 * be S_top and no slot in the run to clear, so the move CUTS the
		 * run instead: the moved top is its TAIL (@src_cut bytes in), and
		 * the junction is the run's own parent slot.  Nothing is stranded
		 * by that -- a run holds exactly one child, so everything below the
		 * cut is everything the run held, and the run itself is retired
		 * whole.  The bytes BEFORE the cut must still match the key; the
		 * bytes after it are what the tail carries to the destination.
		 */
		if (ft_node_compressed(d_src.nf)) {
			struct cds_ft_compressed_node *cn =
				ft_compressed_node_ptr(d_src.nf);
			unsigned int remaining =
				(unsigned int) (src_len - d_src.depth);

			if ((unsigned int) cn->len > remaining) {
				if (ft_match_compressed_key(ik, cn, remaining)
						!= remaining)
					return FT_REKEY_UNCOVERED;
				src_cut = remaining;
				break;		/* S_top is this run's tail */
			}
			if (ft_match_compressed_key(ik, cn,
					(unsigned int) cn->len)
					!= (unsigned int) cn->len)
				return FT_REKEY_UNCOVERED;
			ft_descent_traverse_compressed(ft, &d_src, cn, &ik);
			continue;
		}
		ft_descent_step(ft, &d_src, *(ik++));
	}
	s_top = d_src.nf;
	/*
	 * @src_cut lands the descent ON the run rather than past it, so the
	 * depth it stopped at is the RUN'S OWN and the key ends @src_cut bytes
	 * further in.  Both readings say the same thing: the src prefix is
	 * exhausted exactly here.
	 */
	if (!s_top || d_src.depth + src_cut != src_len ||
			ft_node_flip_proxy(s_top) || ft_node_external(s_top))
		return FT_REKEY_UNCOVERED;
	/*
	 * A COMPRESSED S_top is in scope: ft_rekey_cow_stop copies the run and its
	 * one child, and the one re-parent it records is also what re-points the
	 * skip back-channel, so nothing about the encoding needs recomputing.
	 *
	 * The SKIP-ENCODED slot form is refused, and that refusal guards a shape a
	 * REKEY CANNOT REACH rather than narrowing this cut.  The encoding is a GROUP
	 * property, and cds_ft_group_attr_set_lookup_optimization CLEARS
	 * CDS_FT_FLAG_SKIP_COMPRESSED for EAGER; a rekey requires EAGER, because it
	 * re-parents a leaf without being able to rewrite an app-owned stored key.
	 * So every compressed run a rekey meets was published PLAIN.  Keep the check:
	 * it is cheap, and it is what stops the accessor mismatch below from becoming
	 * a wild read if that coupling ever changes.
	 */
	if (ft_node_compressed(s_top)) {
		s_top_compressed = true;
	} else {
#ifdef FEATURE_FT_SKIP_COMPRESSED
		if (ft_node_skip_compressed(s_top))
			return FT_REKEY_UNCOVERED;
#endif
		ti = ft_node_type(s_top);
		if (ft_types[ti].type_class != FT_POPCOUNT &&
				ft_types[ti].type_class != FT_PIGEON)
			return FT_REKEY_UNCOVERED;
	}
	/*
	 * A key ending exactly AT @src_key -- @s_top's co-located external chain --
	 * moves with the subtree: ft_rekey_cow_stop carries the forward pointer and
	 * records the head's back edge.  The head itself is app-owned and is never
	 * copied, which is also why an S_top that IS an external head stays out:
	 * there would be no library node to give a fresh address to, and the
	 * coherent reader's witness is built on that freshness.
	 */
	s_top_meta = s_top_compressed ?
		cds_ft_item_to_metadata((struct cds_ft_inode *)
			ft_compressed_node_ptr(s_top)) :
		cds_ft_item_to_metadata(ft_node_ptr(s_top));

	/*
	 * BP (= S_top's parent).  A COMPRESSED BP is in scope: a run is a node
	 * like any other to the detach -- it just has exactly one child, so
	 * dropping S_top empties it and the detach's upward walk ELEVATES past
	 * it.  That shape is carried by the fold (the orphan chain rides
	 * @detach_rc and is reclaimed on the far side of the one commit), so
	 * the only thing the run changes here is where its metadata lives.
	 *
	 * A SKIP-COMPRESSED BP stays out: its metadata is the skip target's,
	 * and the detach's trailing-skip arm is a second retire the fold has
	 * no owner for.
	 */
	if (!d_src.pnf || ft_node_flip_proxy(d_src.pnf) ||
			ft_node_external(d_src.pnf))
		return FT_REKEY_UNCOVERED;
#ifdef FEATURE_FT_SKIP_COMPRESSED
	if (ft_node_skip_compressed(d_src.pnf))
		return FT_REKEY_UNCOVERED;
#endif
	bp_meta = ft_node_compressed(d_src.pnf) ?
		cds_ft_item_to_metadata((struct cds_ft_inode *)
			ft_compressed_node_ptr(d_src.pnf)) :
		cds_ft_item_to_metadata(ft_node_ptr(d_src.pnf));
	/*
	 * RACE-WINDOW INJECTION for the descent->detach ABA hypothesis
	 * (-DFT_DELAY_INJECT, FT_DELAY_MODE=writer, FT_DELAY_US=N).
	 *
	 * @d_src.pnfp is an ADDRESS INTO A NODE BODY, captured by the descent
	 * above and not dereferenced until ft_detach_node's climb loads
	 * *detach_parent_flag_ptr far below.  If the node OWNING that slot is
	 * retired, freed and REALLOCATED in between, the address still points at
	 * valid memory belonging to a different node, and the climb reads a value
	 * that was never in the slot it thinks it read.
	 *
	 * HERE is the right place to widen it: the descent has captured the slot
	 * and the shape gates have run, but the op holds NO lock and NO reserve
	 * yet -- so a delay lets peers actually retire and recycle it.  Delaying
	 * later, once this op holds the graft's nodes, would block the very peers
	 * whose recycling the hypothesis needs.
	 *
	 * Paired with the ABA generation witness in ft_detach_node: delay OFF must
	 * read 0 and delay ON must read non-zero for the hypothesis to stand.
	 */
	/*
	 * PROBABILISTIC (FT_DELAY_ONE_IN=N), and that is not a refinement -- a
	 * flat delay here BUYS NOTHING.  Measured: 50us per attempt widened the
	 * window ~18x and dropped throughput from 8616 moves to 464 in the same
	 * wall time, so (window x opportunities) came out flat.  Delaying only
	 * 1 attempt in N keeps the other N-1 at full speed, so wide windows per
	 * unit time actually rises.
	 */
	{
		static long one_in = -1;

		if (caa_unlikely(one_in < 0)) {
			const char *e = getenv("FT_DELAY_ONE_IN");

			one_in = e ? atol(e) : 1;
		}
		if (one_in <= 1) {
			ft_delay_writer();
		} else {
			static __thread unsigned long tick;

			if (++tick % (unsigned long) one_in == 0)
				ft_delay_writer();
		}
	}
	/*
	 * PROXY-SAFE count read: the raw ft_meta_nr_child() would decode a peer's
	 * parked FT_STATE_PROXY as a garbage child count and reject a perfectly good
	 * shape as PERMANENTLY invalid (measured: 10-12 spurious -EINVAL per stress
	 * run).  ft_meta_nr_child_load resolves the proxy to the committed value.
	 */
	/*
	 * A BINARY BP IS COVERED.  An internal node has at least two children (a
	 * one-child one is path-compressed away), so two is the MINIMUM ARITY and
	 * refusing it would refuse the ordinary case rather than a corner one.
	 * Dropping S_top from a binary BP leaves one child, so the move owes a
	 * chain-compress collapse -- and ft_chain_compress_fused RECORDS that
	 * collapse into this op's txn rather than committing one of its own (two
	 * commits cannot be one decide), hands its chain back for reclaim on the
	 * right side of the commit, and the reservation below is sized for its
	 * edges.  Two children therefore ride the same single decide as three.
	 *
	 * A ONE-CHILD BP is covered too, and it is the compressed-run shape: a
	 * run holds exactly one child, so dropping S_top empties it and the
	 * detach's upward walk ELEVATES -- it stops only at a boundary, and
	 * nr_child > 1 is what makes one.  The orphan chain that walk clears
	 * rides @detach_rc back to this frame and is freed on the far side of
	 * the one commit, which is the unlink it must follow.
	 *
	 * ZERO is not a shape: BP is S_top's parent, so it has at least the one
	 * child this move removes.  A count that reads below one is a torn or
	 * superseded read, and re-descending is the answer.
	 */
	if (ft_meta_nr_child_load(bp_meta) < 1)
		return -EAGAIN;

	/*
	 * INCREMENT 3: an OCCUPIED dst is a MERGE, not an error -- the moved subtree
	 * unions into whatever already sits at @dst_key instead of being grafted into
	 * a spare slot.  Probe read-only and conservatively (a descent that cannot
	 * reach the dst depth plainly proves nothing, so those shapes are left to the
	 * later gate) and record the answer for step 2.
	 *
	 * ★ WHY THE MERGE PATH DOES NOT COW S_top, and it is not an oversight.
	 * ft_rekey_cow_stop exists to give the moved subtree's top a FRESH ADDRESS, so
	 * the coherent reader's two-descent witness sees the move as a changed visited
	 * -node set.  A merge already does that BY CONSTRUCTION: entered with an
	 * internal, non-compressed S_top, ft_merge_build cannot take either of the two
	 * exits that return a live node (the shared-run collapse needs both sides
	 * compressed; the leaf-splice needs both external), so it falls to its tail and
	 * returns a freshly allocated M -- and it retires S_top outright on the way.
	 * Interposing a COW would allocate a copy for the merge to consume and free.
	 * cow_stop's OTHER job -- marking the children whose state words the commit
	 * parks into -- is NOT redundant, and is done by the glue's own acquire.
	 *
	 * SCOPE OF THIS FIRST CUT (each a PERMANENT -EINVAL, checked at the gate below,
	 * not silently degraded):
	 *  - with the list ON, a moved run whose suffixes DISJOINTLY precede or follow
	 *    the destination region's.  Then the moved cells stay one contiguous range
	 *    and splice at a region boundary, which is the same run move an empty
	 *    destination gets.  An INTERLEAVED range is refused: threading the cells
	 *    individually is ft_merge_ord_interleave_collect's job, and that helper
	 *    plain-stores each surviving cell's links because they are not
	 *    ord-reachable in a cross-trie merge -- which is false here, where the
	 *    source cells live in the very list being rebuilt (ft_rekey_run_vs_region).
	 *  - a PLAIN INTERNAL node at the dst POINT.  A compressed / skip /
	 *    external node AT @dst_len brings the KEY_SHORTER wrap and Edge-D
	 *    shapes, which are ft_merge_spine_copy's job.  A compressed run
	 *    strictly ABOVE the point is in scope -- see the probe below.
	 */
	{
		struct ft_descent d_probe;
		const uint8_t *pk = dst_ord;

		/*
		 * DECODE a compressed run the destination path crosses, instead of
		 * stopping at it.  A run the dst key consumes WHOLE is a plain
		 * matter of ROUTING only: the node at @dst_len below it is whatever it
		 * is, and the arm is chosen by THAT node, not by how the descent
		 * reached it.  Stopping short instead reported an OCCUPIED "az"
		 * (reached through the path-compressed run under 'a') as an EMPTY
		 * destination, which sent an occupied-destination move to the
		 * empty-dst splice arm -- whose bracket check then found the dst's
		 * own keys inside its range and refused, forever
		 * (project_ft_rekey_occupied_dst_behind_compressed_livelock).
		 *
		 * Two readings still stop the walk, and both leave @merge_dst false
		 * exactly as before:
		 *  - the run DIVERGES from @dst_ord: no key carries the dst prefix,
		 *    so the destination really is empty and the splice arm is right;
		 *  - the run OVERSHOOTS @dst_len (the dst key ends INSIDE it): the
		 *    KEY_SHORTER dst, still outside this cut.  Deliberately NOT
		 *    hardened into an immediate shape refusal -- a peer split can
		 *    make this reading transient, so it keeps the BOUNDED path
		 *    (feedback_structural_single_threaded_can_be_transient_under_peers).
		 */
		ft_descent_init(&d_probe, ft);
		while (d_probe.depth < dst_len && d_probe.nf) {
			/*
			 * SKIP, then COMPRESSED, then EXTERNAL -- the kind
			 * dispatch order the rest of the unit uses, and it is
			 * not cosmetic: a skip pointer ONTO AN EXTERNAL leaf has
			 * low tag bits 0, so ft_node_external() matches it on the
			 * raw value (feedback_skip_before_external_tag_order).
			 * The old AND-chain was order-blind because every arm
			 * meant the same thing; a dispatch is not.
			 */
#ifdef FEATURE_FT_SKIP_COMPRESSED
			if (ft_node_skip_compressed(d_probe.nf))
				break;
#endif
			if (ft_node_external(d_probe.nf))
				break;
			if (ft_node_compressed(d_probe.nf)) {
				struct cds_ft_compressed_node *cn =
					ft_compressed_node_ptr(d_probe.nf);
				unsigned int remaining =
					(unsigned int) (dst_len - d_probe.depth);

				if ((unsigned int) cn->len > remaining ||
						ft_match_compressed_key(pk, cn,
							(unsigned int) cn->len)
						!= (unsigned int) cn->len)
					break;
				ft_descent_traverse_compressed(ft, &d_probe, cn, &pk);
				continue;
			}
			ft_descent_step(ft, &d_probe, *(pk++));
		}
		merge_dst = (d_probe.depth == dst_len && d_probe.nf != NULL);
		probe_D = merge_dst ? d_probe.nf : NULL;
		/*
		 * @require_empty is the GRAFT caller's semantics (cds_ft_rekey_graft
		 * refuses an occupied destination rather than unioning into it), and
		 * -EEXIST is how it travels back: a distinct code, because the
		 * dispatcher must tell "the destination holds content" (terminal,
		 * POPULATED_ERROR) apart from "this shape is outside the one-decide
		 * cut" (-EINVAL, try the staged writer).  Read-only so far, so this is
		 * a clean no-op bail.
		 */
		if (merge_dst && require_empty)
			return -EEXIST;
		/*
		 * A COMPRESSED S_top is in scope for the GRAFT arm only.  The merge arm
		 * skips ft_rekey_cow_stop on the argument that ft_merge_build gives the
		 * moved top a fresh address BY CONSTRUCTION -- and that argument names
		 * its premise: "entered with an internal, non-compressed S_top,
		 * ft_merge_build cannot take either of the two exits that return a live
		 * node (the shared-run collapse needs BOTH SIDES COMPRESSED...)".  A
		 * compressed S_top is exactly what unlocks that exit, so the freshness
		 * the coherent reader's witness depends on would be gone.
		 */
		if (merge_dst && s_top_compressed)
			return FT_REKEY_UNCOVERED;
		/*
		 * A co-located external chain is carried by ft_rekey_cow_stop, which
		 * the MERGE arm skips -- ft_merge_build would have to union that key
		 * into the destination's own chain, and nothing here has tested it.
		 * The graft arm takes it.
		 */
		if (merge_dst && s_top_meta->external_nodes)
			return FT_REKEY_UNCOVERED;
#ifndef FEATURE_FT_MERGE
		/*
		 * An OCCUPIED destination IS a merge (INCREMENT 3 unions S_top
		 * into it with ft_merge_build), so -DNO_FEATURE_FT_MERGE compiles
		 * that fold out along with the rest of the subsystem.  Report it
		 * the way the merge API itself does rather than building a fold
		 * whose machinery is not there.  The EMPTY-dst rekey below is a
		 * graft and stays available.
		 */
		if (merge_dst)
			return -ENOTSUP;
#endif
	}

	/*
	 * List on: capture the moved subtree's contiguous ordered-cell run endpoints
	 * (the structural min/max external heads under S_top) from the still-pristine
	 * list, so the ONE commit can unsplice the run from the src ordered position
	 * and re-splice it at the dst position (six MW boundary edges) atomically with
	 * the structural move -- a coherent reader never sees a moved key gone from the
	 * structure but still in the list (or vice versa).  cow_stop SHARES S_top's
	 * leaves (only S_top's own node relocates), so these heads stay valid across it.
	 *
	 * Locate the dst splice neighbours NOW, on the pristine list (find_splice_pos
	 * needs the dst attach point empty, which it still is -- nothing is published
	 * until the final commit), and REJECT an adjacency shape up front (before any
	 * txn / lock acquire / record, so the bail is a clean no-op -EINVAL): because
	 * find_splice_pos runs while the run is STILL at src, a dst gap that abuts the
	 * run resolves the run's own ENDPOINT cell as a splice neighbour (dsucc == run
	 * first, or dpred == run last), which would record a duplicate-slot MW edge and
	 * plain-store a self-cyclic run link.  This hook only supports a dst position
	 * clear of the run's current ordered neighbourhood; a general rekey would locate
	 * the splice against the run-removed list instead.
	 *
	 * Completeness of the "no run cell is a splice neighbour" guarantee is JOINT:
	 * this guard rejects the two ENDPOINT-adjacency shapes, while the INTERIOR case
	 * (a dst gap whose neighbour is a run cell strictly between rfc and rlc) is
	 * excluded by the DISJOINT-key rule at the top of this function -- an interior
	 * neighbour would have to be bracketed by two run keys, which puts @dst_key
	 * inside the run's own contiguous ordinal range and so makes @src_key a prefix
	 * of it.  The rule is stated here rather than left implicit in a shape
	 * gate, because no gate forces src_len == dst_len.
	 *
	 * VALIDATE the located pair FIRST, and bail -EAGAIN (transient, re-derive) when
	 * it does not bracket the dst key range: find_splice_pos derives the pair from a
	 * RELATIONAL descent, which is NOT coherent under concurrent structural
	 * mutation, so a peer rekey in flight can return a pair that is adjacent in the
	 * list but sits at the WRONG key position.  ADJACENT + BRACKETING is the full
	 * correctness condition for a splice, and only adjacency was checked: the
	 * splice's own boundary edge (pred->next expect succ) validates adjacency at
	 * commit, and ft_rekey_splice_pos_brackets validates the other half here.  The
	 * bracket check must precede the endpoint-adjacency check so that a racy pair
	 * involving a run endpoint surfaces as the transient -EAGAIN it is rather than
	 * the permanent -EINVAL of the genuine (correctly-derived, abutting) shape --
	 * a racy pred == rlc drags succ = resolve(rlc->next) along, which then sorts
	 * BELOW the dst range and fails the bracket.
	 *
	 * WHAT IS STILL OPEN (general rekey, not this hook's tested shapes): a pair that
	 * brackets and is adjacent when validated can still be invalidated afterwards if
	 * a neighbour is INTERIOR to a peer's moving run -- the peer's move leaves an
	 * interior cell's own links untouched, so no edge of this commit detects it, and
	 * the run lands inside the peer's run.  Every neighbour reachable in the tested
	 * layouts is either a never-moving key or a peer run's ENDPOINT (whose outer link
	 * IS one of these edges, hence detected).  Closing it in general needs the
	 * per-FT move seqcount / relational coherence (doc: in-trie-move-seqcount.md),
	 * which would also let a rekey validate the derivation itself.
	 */
	if (ft->ordered_list) {
		struct ft_ord_cell *rfc, *rlc;

		run_rfirst = ft_subtree_minmax_head(ft, s_top, false);
		run_rlast = ft_subtree_minmax_head(ft, s_top, true);
		rfc = ft_ord_cell_ptr(rcu_dereference(run_rfirst->prev));
		rlc = ft_ord_cell_ptr(rcu_dereference(run_rlast->prev));
		if (merge_dst) {
			/*
			 * OCCUPIED destination: the splice position is not a gap to
			 * search for, it is a BOUNDARY of the merge region already there,
			 * so derive it from that region's own endpoint cells instead of
			 * relationally.  That is strictly better than find_splice_pos --
			 * a structural read of the region needs no two-pass agreement and
			 * no bracket check to be trusted -- and it is available only here,
			 * where the region exists.
			 *
			 * Which boundary depends on where the moved run sorts relative to
			 * the region, and an INTERLEAVE is refused: see
			 * ft_rekey_run_vs_region for why that one needs machinery this
			 * writer does not have.
			 *
			 * THE DISJOINTNESS SURVIVES TO THE COMMIT, and the splice's own
			 * edges are what make it.  The only peer insert that can break it
			 * is one landing BELOW the region's minimum (for a run spliced in
			 * front) -- anything inside the region still sorts above the whole
			 * run, so the order holds -- and such an insert splices between
			 * @run_dpred and @run_dsucc, which is the very pair
			 * ft_ord_cell_run_resplice_edges records as pred->next == succ and
			 * succ->prev == pred.  It therefore ABORTS this commit rather than
			 * mis-ordering the list.  Symmetrically for a run spliced behind.
			 */
			struct ft_ord_cell *dfirst, *dlast;

			dfirst = ft_ord_cell_ptr(rcu_dereference(
				ft_subtree_minmax_head(ft, probe_D, false)->prev));
			dlast = ft_ord_cell_ptr(rcu_dereference(
				ft_subtree_minmax_head(ft, probe_D, true)->prev));
			if (!dfirst || !dlast)
				return -EAGAIN;		/* region read torn */
			switch (ft_rekey_run_vs_region(ft, rfc, rlc, src_len,
					dfirst, dlast, dst_len)) {
			case -1:			/* run below: splice in front */
				run_dpred = ft_ord_cell_resolve_ord(
						&dfirst->lnode.prev);
				run_dsucc = dfirst;
				break;
			case 1:				/* run above: splice behind */
				run_dpred = dlast;
				run_dsucc = ft_ord_cell_resolve_ord(
						&dlast->lnode.next);
				break;
			default:
				/*
				 * INTERLEAVED: not a run move.  The cells thread
				 * individually between the region's, which
				 * ft_rekey_ord_interleave builds at the cell step
				 * below (and which refuses the shapes it cannot
				 * express).  No splice pair applies.
				 */
#ifdef FEATURE_FT_MERGE
				run_interleaves = true;
				break;
#else
				return -EINVAL;		/* no merge: no interleave */
#endif
			}
		} else {
			/*
			 * COHERENT derivation: two from-root traversals, compared by their
			 * visited-node witness (ft_ord_cell_find_splice_pos_coherent).
			 * This is what makes the splice position trustworthy -- the
			 * single-pass relational answer is not, and no amount of checking
			 * the ANSWER repairs that (measured: the key-bracket check below,
			 * which is a relational-era read itself, was fooled too).
			 * Disagreement means a peer move is restructuring this
			 * neighbourhood: bail and re-derive.
			 */
			if (!ft_ord_cell_find_splice_pos_coherent(ft, dst_key,
					dst_len, &run_dpred, &run_dsucc))
				return -EAGAIN;	/* torn derivation: re-descend */
			/*
			 * Belt and braces, and cheap: the pair must also BRACKET the dst
			 * key range.  Adjacent + bracketing is the full correctness
			 * condition for a splice; the two-pass agreement establishes the
			 * pair was not read torn, this establishes it is the RIGHT pair.
			 */
			switch (ft_rekey_splice_pos_brackets(ft, dst_ord,
					dst_len, run_dpred, run_dsucc)) {
			case 1:
				break;			/* bracketed: proceed */
			case -1:
				/*
				 * A key extends the dst prefix.  ☠ THAT IS NOT
				 * ALWAYS STRUCTURAL, and assuming it was is what
				 * this bound exists to correct: single-threaded
				 * it means the destination is OCCUPIED behind a
				 * node the @merge_dst probe would not decode, and
				 * no retry can clear it; under PEERS the very
				 * same reading is produced transiently by another
				 * move mid-splice, and retrying is exactly right.
				 * Measured: inv_rekey_graft_shared (16 writers
				 * over 2 shared junctions) refuses moves that
				 * used to succeed if this answers UNCOVERED at
				 * the first sight of it.
				 *
				 * So retry it like the contention it may be, and
				 * give up only once the attempt age says no peer
				 * is plausibly still responsible.  The bound is
				 * what converts the permanent case from an
				 * unbounded spin into a refusal; its exact value
				 * only has to sit far above real contention (the
				 * rekey oracles complete their moves with single
				 * -digit retries) and far below a livelock (the
				 * measured one ran 193k attempts).
				 */
				if (optxn->retry < FT_REKEY_UNCOVERED_AFTER)
					return -EAGAIN;
				/*
				 * Aged out, so the reading is STRUCTURAL: the
				 * destination really is occupied.  For a GRAFT that
				 * is the CALLER'S ANSWER -- the same -EEXIST the
				 * decoded @merge_dst probe gives a few lines above --
				 * and no wider cut changes it, so reporting UNCOVERED
				 * would send a settled refusal to a fallback.  Only a
				 * MERGE, which would UNION into that occupant, is the
				 * shape this writer leaves uncovered here.
				 */
				if (require_empty)
					return -EEXIST;
				return FT_REKEY_UNCOVERED;
			default:
				return -EAGAIN;		/* torn: re-derive */
			}
		}
		/*
		 * THE DST POSITION ABUTS THE RUN, and that is not a shape to refuse: it
		 * means the moved keys sort into the SAME list slot the run already
		 * occupies, so the unsplice and the re-splice CANCEL and the run STAYS
		 * PUT.  Recorded as @run_keeps_pos and honoured at the splice below.
		 *
		 * Why it cancels, for the two ways it arises.  The located pair is
		 * ADJACENT in the list, so @run_dsucc == the run's first cell forces
		 * @run_dpred to be the run's own predecessor A, and @run_dpred == its
		 * last cell forces @run_dsucc to be its successor B.  Either way the dst
		 * key range lies strictly inside (A, B) -- and the run is the ONLY thing
		 * in (A, B) -- so the run's NEW keys sort between A and B exactly where
		 * its old ones did.  Nothing about the list has to change.
		 *
		 * Emitting the six edges anyway is what made this look unsupportable:
		 * the unsplice's A->next = B and the re-splice's A->next = rfc are TWO
		 * RECORDS ON ONE SLOT, which breaks the engine's distinct-slot rule and
		 * would plain-store a self-cyclic run link.  Locating the pair against
		 * the run-REMOVED list -- the other repair this comment used to propose
		 * -- yields (A, B) and therefore those same cancelling edges; the
		 * cancellation is the answer, not a different derivation.
		 */
		if (run_dsucc == rfc || run_dpred == rlc)
			run_keeps_pos = true;
	}

	cnt = ft_nr_keys_get(s_top_meta);	/* subtree key count (count edges no-op if rank off) */

	/*
	 * ON the op's persistent handle, not a standalone one: retry aging and
	 * the FIFO escalation turn live in @optxn and must survive this attempt.
	 * ft_flip_txn_create() inits a handle with NO domain, so every attempt
	 * started fresh, never qualified for a turn, and simply spun.
	 */
	txn = ft_flip_txn_create_on(ft, optxn);
	if (!txn)
		return -ENOMEM;

	/* 1. COW S_top -> S_top' (SW; records re-parents + retire, LOCKED). */
	/*
	 * The mode says nothing about &ft->root: a root records MW by
	 * construction (ft_flip_txn_record_root), whichever txn writes it.
	 *
	 * ☠ NO SHAPE HERE REACHES ONE TODAY, and saying so is the point.  A
	 * depth-1 source junction is the only shape that would make the detach
	 * recompact the ROOT and republish it at that slot, and the junction
	 * gate below refuses exactly that shape, FT_REKEY_UNCOVERED -- giving
	 * as its reason that &ft->root has "no node word to park the republish
	 * under".
	 *
	 * The record helper is what ANSWERS that reason, and it answers it for
	 * every caller at once rather than per txn: the gate can only decline
	 * the whole move, whereas the record simply plants that one slot the
	 * way every other writer of it already does.  Lifting the gate is a
	 * separate question -- other refusals sit behind it -- but arming is
	 * no longer part of it.
	 */
	ft_flip_txn_set_structural_sw(txn, true);
#ifdef FT_REKEY_CLAIM
	/*
	 * 9.1 AUDIT: this writer is HAND-armed, so B0's owner assert never runs on
	 * it -- @dbg_arm_per_op is set only by ft_flip_txn_arm_per_op and the claim.
	 * Point the claim at the one already-armed SW content site and the assert
	 * names every slot it parks without owning, which is what §9.1 calls its
	 * missing fine-lock conversions.
	 *
	 * ☠ THE _ARMABLE FORM, NOT THE RAW ONE.  This driver runs under COARSE and
	 * exclusive too (ft_rekey_graft_simple_locked takes CDS_FT_SCOPED_WRITER),
	 * and there the SW parks are legal on the TRIE-WIDE exclusion -- the
	 * per-node locks the assert looks for are deliberately never taken, so the
	 * raw claim reports the wide mutex's own soundness as a violation.  That is
	 * the false positive ft_flip_txn_claim_per_op_armable exists to refuse, and
	 * skipping it cost a full misdiagnosis: ft_inv's coarse rekey arms aborted
	 * at ft_store_at_graft_point_commit's republish with ft->lock_fine == false,
	 * which reads exactly like a missing acquire and is not one.
	 */
	ft_flip_txn_claim_per_op_armable(ft, txn);
#endif
	if (!merge_dst) {
		ft_lock_ctx_init(&lctx_src, &d_src, txn, optxn);
		/*
		 * NOT bound to @optxn.  ft_flip_txn_create_*_on sets t->mtxn =
		 * op -- an "_on" txn SHARES the handle rather than making its own
		 * -- and this driver's CONTENT txn is already
		 * ft_flip_txn_create_on(optxn).  Binding here would put the
		 * DEDICATED acquire txn on that same live handle and commit it
		 * mid-op (measured: SIGSEGV in test_rekey_graft_liston, every
		 * spacing).  The acquire's escalation aging and a content txn on
		 * one handle are mutually exclusive; giving this path both needs
		 * a SECOND handle, not a binding.
		 */
		ret = ft_rekey_cow_stop(ft, &lctx_src, txn, s_top, d_src.depth,
			src_cut,
				&s_top_prime, marks, &nr_marks);
		if (ret) {
			ft_flip_txn_destroy(txn);	/* pre-commit bail: destroy caller-owned */
			goto sweep;
		}
		ft_rekey_marks_to_txn(txn, marks, nr_marks);
	}

	/*
	 * 2. + 3.  structural_sw STAYS TRUE for the rest: the graft ALWAYS relocates
	 * the dst attach node (a reserve recompaction), which ACQUIRES node locks
	 * over the dst parent + the republish grandparent and records its re-parents /
	 * release / retire as SW under those locks -- so the graft forward publish and
	 * recompact edges are correctly SW.  The detach's src-junction (BP) edges are
	 * UNLOCKED, and no toggle is needed (toggling OFF would wrongly demote the
	 * recompact's lock-expecting edges to MW -> expected-old mismatch -> abort).
	 *
	 * ★ NOT because "ft_ord_cell_record_into forces them MW regardless of
	 * structural_sw", which this said and which is false for exactly these
	 * edges: ft_ord_cell_record_into dispatches on ft_edge_tag(), and
	 * ft_edge_tag maps tag 0 -> FT_FLIP_PROXY_TAG, taking the
	 * ft_flip_txn_record_tag (structural_sw-honouring) branch, NOT
	 * ft_flip_txn_record_tag_mw.  Only a non-zero (ordered-cell) tag is forced
	 * MW.  What actually makes the src-junction edges safe here is the
	 * exclusive-source gate, not the record path -- so do not weaken that gate
	 * on the strength of the old sentence.
	 * The mixed commit installs the MW (detach) edges first, then parks the SW
	 * (graft + cow_stop) edges before the flip.
	 */

	/* 2. Graft-fold: record-only attach of S_top' at @dst_key. */
	ft_glue_init(&glue);
	glue.op = optxn;
	glue.txn = txn;
	glue.record_only = true;
	/*
	 * GLUE (compressed-divergence) shape: have the build FENCE the compressed
	 * node it splits before it reads its plan, so the whole build runs under
	 * that fence and its retire rides our commit (ft_split_compressed_graft_build).
	 * The fence is @glue's until the commit registers it; ft_glue_abort is the
	 * single release point, and every bail below routes through it.
	 */
	glue.fence_split_cn = true;
	memset(&reserve, 0, sizeof(reserve));
	/*
	 * The node reserve exists for the GRAFT arm, whose recompaction allocates
	 * under a no-fail contract and so activates it.  The merge arm never
	 * activates it -- ft_merge_build allocates its cluster from the arena and is
	 * allowed to fail, which is the whole reason the fold can still bail there --
	 * so filling it would allocate a batch per move only to drain it untouched.
	 */
	if (!merge_dst && ft_bulk_node_reserve_fill(ft, &reserve)) {
		ft_flip_txn_destroy(txn);
		ret = -ENOMEM;
		goto sweep;
	}
	if (merge_dst) {
#ifdef FEATURE_FT_MERGE
		/*
		 * INCREMENT 3, step 2': UNION S_top into the occupied dst.
		 *
		 * Descend the dst ourselves -- ft_graft_build would report POPULATED and
		 * hand back nothing to publish into.  MIRRORS the @merge_dst probe
		 * above byte for byte, which is what makes the two agree on a quiet
		 * tree: a wholly-consumed compressed run is decoded, and every reading
		 * the probe stops on (divergence, an overshooting run, a skip or
		 * external node) bails here too.  A disagreement is therefore a PEER
		 * changing the shape between the probe and this descent; it keeps the
		 * pre-existing terminal answer rather than a retry, because a probe
		 * and a descent that disagree on a QUIET tree would be a defect this
		 * op cannot retry its way out of.
		 *
		 * @d_dst then names {D, publish parent, publish slot, grandparent},
		 * where the publish parent may now be the compressed node itself and
		 * the publish slot its ->child -- a slot with metadata, a state word
		 * and a lock level of its own (ft_descent_traverse_compressed enters
		 * it as one), so every fence and acquire below reads it the same way
		 * it reads a plain parent.
		 */
		const uint8_t *dk = dst_ord;

		ft_descent_init(&d_dst, ft);
		while (d_dst.depth < dst_len) {
			/* Kind dispatch, SKIP first: see the probe's note. */
			if (!d_dst.nf
#ifdef FEATURE_FT_SKIP_COMPRESSED
					|| ft_node_skip_compressed(d_dst.nf)
#endif
					|| ft_node_external(d_dst.nf)) {
				ret = -EINVAL;
				goto bail_build;
			}
			if (ft_node_compressed(d_dst.nf)) {
				struct cds_ft_compressed_node *cn =
					ft_compressed_node_ptr(d_dst.nf);
				unsigned int remaining =
					(unsigned int) (dst_len - d_dst.depth);

				if ((unsigned int) cn->len > remaining ||
						ft_match_compressed_key(dk, cn,
							(unsigned int) cn->len)
						!= (unsigned int) cn->len) {
					ret = -EINVAL;
					goto bail_build;
				}
				ft_descent_traverse_compressed(ft, &d_dst, cn, &dk);
				continue;
			}
			ft_descent_step(ft, &d_dst, *(dk++));
		}
		/*
		 * The merge point itself must be a PLAIN INTERNAL node: a compressed,
		 * skip or external D is the KEY_SHORTER / Edge-D / leaf-splice family,
		 * which belongs to ft_merge_spine_copy.  d_dst.nf is non-NULL by the
		 * probe, but re-checked because the probe ran outside this txn.
		 */
		if (d_dst.depth != dst_len || !d_dst.nf ||
				ft_node_flip_proxy(d_dst.nf) ||
				ft_node_external(d_dst.nf) ||
				ft_node_compressed(d_dst.nf) ||
#ifdef FEATURE_FT_SKIP_COMPRESSED
				ft_node_skip_compressed(d_dst.nf) ||
#endif
				!d_dst.pnf || !d_dst.nfp) {
			ret = -EINVAL;
			goto bail_build;
		}
		{
			unsigned int dti = ft_node_type(d_dst.nf);

			if (ft_types[dti].type_class != FT_POPCOUNT &&
					ft_types[dti].type_class != FT_PIGEON) {
				ret = -EINVAL;
				goto bail_build;
			}
		}
		/*
		 * FENCE the publish parent OURSELVES, exactly as the GLUE arm does and
		 * for the same reason: this txn is structural_sw, so the forward publish
		 * PARKS a plain store into that node's slot, and the fold's rule is SW
		 * iff the op holds the slot's lock.  ft_glue_txn_commit_edges would
		 * otherwise degrade an acquire miss to a §4.B guard.
		 */
		/*
		 * Hand the fold's glue its anchor source, here, the single place
		 * holding both (ft_merge_spine_copy does the same at its own
		 * gd.lock_d).  The fold's members are reached from the BUILD --
		 * a deferred re-parent child above all -- so nothing else can
		 * date them, and a glue with no descent leaves every one of them
		 * undatable: the acquire bails, the caller re-descends, and the
		 * next attempt is identical.
		 */
		glue.lock_d = &d_dst;
		/*
		 * And the SRC path for the members still on it.  The fold's
		 * deferred re-parents come from BOTH sides -- the src children
		 * being moved and the dst children being absorbed -- and each
		 * must anchor where it is NOW, which is the only position a peer
		 * can descend to before this commit lands (§3).
		 */
		glue.lock_d_src = &d_src;
		pp_meta = ft_flag_to_metadata(ft, d_dst.pnf);
		{
			struct ft_lock_ctx dctx;
			struct ft_held_anchor pph;

			/* The DST descent dates this one: it IS its parent slot. */
			ft_lock_ctx_init(&dctx, &d_dst, txn, optxn);
			if (!pp_meta || ft_acquire_member(ft, &dctx, d_dst.pnf,
					pp_meta, d_dst.pdepth, &pph)) {
				pp_meta = NULL;
				ret = -EAGAIN;
				goto bail_build;
			}
			pp_meta = pph.lock;
			pp_snap = pph.lock_snap;
			/*
			 * A SHARED acquire deduped onto a word this op already
			 * holds: the fence is in force -- so the forward publish
			 * may still PARK its SW store -- but the FIRST acquire
			 * owns both the release and the registry entry, which is
			 * what @publish_parent_shared tells the commit.  The
			 * split arm below carries the same flag; refusing it here
			 * instead would refuse the op's own mark forever.
			 */
			pp_shared = pph.shared;
		}
		ft_glue_take_publish_parent(&glue, pp_meta, pp_snap, pp_shared);
#ifdef FEATURE_FT_SKIP_COMPRESSED
		/*
		 * A COMPRESSED publish parent -- the shape a destination behind a
		 * path-compressed run presents -- makes the forward publish TWO
		 * stores, not one: _ft_publish_to_parent also re-encodes the SKIP_X
		 * pointer that lets a candidate reader BYPASS the compressed node,
		 * and that pointer sits in a slot of the compressed node's own
		 * parent.  This txn is structural_sw, so both stores PARK, and a
		 * park does not arbitrate -- the op must hold the word behind each.
		 * @publish_parent_holder covers the first; this covers the second,
		 * the same {GP} third lock-set member ft_node_recompact takes
		 * whenever its own parent is compressed (§9.3).
		 */
		if (ft_node_compressed(d_dst.pnf)) {
			struct cds_ft_compressed_node *pcn =
				ft_compressed_node_ptr(d_dst.pnf);
			struct cds_ft_metadata *pcn_meta =
				cds_ft_item_to_metadata(
					(struct cds_ft_inode *) pcn);
			struct cds_ft_inode_flag **dual =
				ft_get_parent_slot(pcn_meta, ft);

			/*
			 * ☠ THE PUBLISH AND THE LOCK MUST NAME ONE SLOT.  The
			 * publish derives the dual from @pcn's OWN back-pointer;
			 * the lock below is taken on the parent the DESCENT
			 * reached.  A disagreement means a peer re-homed @pcn
			 * since the descent, so the lock would fence a node the
			 * publish never writes -- and the store it does make would
			 * be an unguarded park.  Answer the peer, not the shape:
			 * this is precisely a condition a re-descent clears.
			 *
			 * (@pcn's own lock is taken ABOVE, and holding it is what
			 * makes the reading stable from here to the commit: a peer
			 * re-home of @pcn parks @pcn's state word, which it cannot
			 * do behind this fence.)
			 */
			if (dual != d_dst.pnfp) {
				ret = -EAGAIN;
				goto bail_build;
			}
			if (dual && ft_node_skip_compressed(*dual)) {
				struct ft_lock_ctx gctx;
				struct ft_held_anchor gph;
				struct cds_ft_metadata *gp_meta;

				/*
				 * The dual lives in &ft->root: a slot with no
				 * node word to park under.  Same refusal the
				 * junction gate below makes for a root-level
				 * junction, and for the same reason.
				 */
				if (!d_dst.ppnf) {
					ret = FT_REKEY_UNCOVERED;
					goto bail_build;
				}
				gp_meta = ft_flag_to_metadata(ft, d_dst.ppnf);
				ft_lock_ctx_init(&gctx, &d_dst, txn, optxn);
				/*
				 * ☠ CHAIN THE GLUE, or a COARSE spacing
				 * livelocks the move.  @publish_parent's mark
				 * is set a few lines up and reaches NO txn
				 * registry until ft_glue_txn_commit_edges runs
				 * -- the one decide has not happened yet -- so
				 * the glue is the only witness that this op
				 * holds it.  Under a spacing that collapses the
				 * compressed parent and its own parent onto ONE
				 * word, an acquire that cannot see that witness
				 * reads the op's OWN fence as contention, bails
				 * -EAGAIN, and every retry re-derives the
				 * identical plan.  Single-threaded, so no peer
				 * can ever clear it.
				 */
				gctx.held.glue = &glue;
				if (!gp_meta || ft_acquire_member(ft, &gctx,
						d_dst.ppnf, gp_meta,
						d_dst.ppdepth, &gph)) {
					ret = -EAGAIN;
					goto bail_build;
				}
				glue.publish_gp_holder = gph.lock;
				glue.publish_gp_snap = gph.lock_snap;
				/*
				 * Coarsening collapses GP onto the publish
				 * parent's word routinely (they are adjacent
				 * levels): the fence is in force either way, but
				 * only the FIRST acquire owes the release.
				 */
				glue.publish_gp_shared = gph.shared;
				/*
				 * HAND THE FENCE TO THE TXN HERE, not at
				 * ft_glue_txn_commit_edges: the two lines are the
				 * same two ft_flip_txn_hold_or_lock_parent's held
				 * arm runs, and this is the moment they become
				 * TRUE.  Between here and commit_edges the fold's
				 * detach RECOMPACTS under this very word and
				 * republishes into its body (ft_chain_compress_fused
				 * -> the forward slot), and a record-time owner
				 * check has only the txn's locks[]: a fence that
				 * arrives afterwards reads as UNOWNED at every
				 * record in between, which is what -DFT_REKEY_CLAIM
				 * aborts on.
				 *
				 * The detach's own acquire of that word DEDUPES
				 * (ft_chain_compress_fused registers only
				 * !held.shared) and so registers nothing -- rightly,
				 * since this acquire owes the release.  After the
				 * transfer that dedupe answers from the REGISTRY,
				 * which ft_held_set_snap reads FIRST, so the coarse
				 * spacing the glue-chaining above protects is
				 * unaffected.
				 *
				 * The release is recorded, not deferred, and the
				 * early placement is the one its own contract asks
				 * for: ft_flip_txn_record_anchor_release_held is
				 * placed by COVERAGE rather than order, a §4.B guard
				 * planted before a release would poison the txn, and
				 * a retire this op decides LATER chains onto the
				 * release's clean pending value.
				 */
				if (!gph.shared) {
					ft_flip_txn_lock_register(txn, gph.lock,
						gph.lock_snap);
					ft_flip_txn_record_anchor_release_held(
						txn, gph.lock);
					glue.publish_gp_txn_owned = true;
				}
			}
		}
#endif /* FEATURE_FT_SKIP_COMPRESSED */

		/* Size both glues from the read-only pre-pass, with headroom. */
		ft_glue_init(&src_glue);
		src_glue.op = optxn;
		/*
		 * The merge's SRC glue anchors from the src descent, as the dst
		 * glue does from @d_dst: its overlap fences land on nodes still
		 * sitting under @s_top, and only @d_src dates those.
		 */
		src_glue.lock_d = &d_src;
		src_glue_live = true;
		/*
		 * ONE op, ONE held set.  The two glues commit together, so a mark
		 * either takes is a mark the other's dedupe must see -- and under
		 * coarsening they DO meet: a src child anchors on the src node the
		 * merge build fenced as an overlap, so the re-parent mark refuses
		 * against this op's own fence and every retry rebuilds it.
		 */
		glue.peer = &src_glue;
		src_glue.peer = &glue;
		ft_merge_count(ft, s_top, 0, d_dst.nf, 0, &mcnt);
		if (ft_glue_reserve(&glue, mcnt.nb + 8, mcnt.nd + 8,
					mcnt.nf_dst + 8, mcnt.ns + 8) ||
		    ft_glue_reserve(&src_glue, 0, 0, mcnt.nf_src + 8, 0)) {
			ret = -ENOMEM;
			goto bail_build;
		}
		/*
		 * FUSE both free lists into the shared txn.  The dst side MUST (a fenced
		 * overlap retire records its {LOCK|s -> TOMBSTONE|s} terminal into
		 * g->txn and asserts on this flag), and the src side must for the reason
		 * the whole fold exists: its free list carries S_top, whose retire has to
		 * flip WITH the publish rather than as a standalone lone-edge store the
		 * commit could not roll back.  The reserve above sized both.
		 */
		glue.fuse_free_list = true;
		src_glue.txn = txn;
		src_glue.fuse_free_list = true;
		mctx.dst_ft = ft;
		mctx.gd = &glue;
		mctx.gs = &src_glue;
		/*
		 * FENCE the dst overlap spine.  Unlike the STAGED rekey -- which reaches
		 * ft_merge_spine_copy already past its point of no return and so must
		 * skip this -- the one-decide fold's single commit is still ahead of it,
		 * so a missed fence is a clean re-descend and the fence is affordable.
		 */
		mctx.fence_overlap = ft->lock_fine;
		/*
		 * ★ And the SRC spine too, which the cross-trie merge does not need.
		 * cds_ft_merge_at owns its source exclusively; THIS source is the live
		 * in-trie subtree -- the detach is only RECORDED into the same txn, so
		 * S_top and everything under it stays reachable to peers for the whole
		 * build window.  Unfenced, a peer inserting below S_top during the copy
		 * is retired along with the node it was inserted into: silent key loss
		 * (ft_merge_lock_overlap's header states the mechanism for the dst side;
		 * it is the same mechanism).
		 */
		mctx.fence_src = ft->lock_fine;
		mctx.overlap_contended = false;
		/*
		 * Where relative depth 0 IS on each side, so both spines' fences
		 * date their nodes ABSOLUTELY (struct ft_merge_ctx).  The fold
		 * enters both sides at offset 0, so each base is its descent's
		 * own cursor depth.
		 */
		mctx.dst_base_depth = d_dst.depth;
		mctx.src_base_depth = d_src.depth;
		merged_nf = ft_merge_build(&mctx, s_top, 0, d_dst.nf, 0, 0,
				&merged_keys);
		if (merged_nf == FT_MERGE_OOM) {
			merged_nf = NULL;
			ret = mctx.overlap_contended ? -EAGAIN : -ENOMEM;
			goto bail_build;
		}
		/*
		 * The merged top replaces D in the publish parent's slot.  Recorded, not
		 * stored: ft_glue_txn_commit_edges runs at step 3c below, after the
		 * detach, like the GLUE arm.
		 */
		ft_glue_set_publish(ft, &glue, d_dst.pnf, d_dst.nfp, merged_nf);
		glue.attached_nf = merged_nf;
		glue.count_delta = (long) cnt;
		/*
		 * COLLIDED KEYS: a full key present on BOTH sides makes ft_merge_build
		 * splice the src leaf onto the dst head's DUPLICATE CHAIN.  That append
		 * walks a LIVE chain, so it runs under the chain holder's node lock --
		 * the exclusion @cc91bd8b added when it closed the last unlocked chain
		 * mutation.  ft_merge_spine_copy takes it before its own point of no
		 * return and SKIPS it for a pre-reserved caller, whose placement cannot
		 * fail; this fold is the case its comment anticipated -- a single commit
		 * still ahead of us, so failing here is free and the lock is ours to take.
		 * No-op on a collision-free merge, which is every disjoint union.
		 */
		if (ft_glue_acquire_splice_holders(ft, &glue)) {
			ret = -EAGAIN;
			goto bail_build;
		}
		attached_nf = merged_nf;
		adepth = (unsigned int) dst_len;
		prep = FT_GRAFT_PREP_NOSPLIT;	/* not a graft; keeps the arms below off */
#endif /* FEATURE_FT_MERGE */
	} else
	{
		/*
		 * The op's outstanding marks -- ft_rekey_cow_stop's @stop fence
		 * and one per COW'd child -- reach no registry until the sweep
		 * below, so the build's own acquires can only see them through
		 * this frame (the same frame the store-prepare arm passes).  This
		 * is an IN-TRIE move: src and dst share a root, so under a coarse
		 * spacing those marks and the split-CN fence are ONE word, and
		 * without the frame the build refuses the op's own fence and the
		 * caller re-descends onto the identical shape forever.
		 */
		struct ft_lock_ctx bctx;

		ft_lock_ctx_init(&bctx, &d_src, txn, optxn);
		bctx.held.extra = marks;
		bctx.held.nr_extra = nr_marks;
		/*
		 * ARM THE OLD-DIRECTION DROP.  When the dst key diverges inside
		 * the very run S_top hangs off, that run's one child IS S_top --
		 * the subtree @s_top_prime is the copy of -- so the split has no
		 * old half to keep and the move owes no detach.  Naming the node
		 * is the whole condition: a run has exactly one child, so
		 * BP == the split node already says which edge goes.
		 *
		 * Set unconditionally -- the build only ever splits a compressed
		 * node, so a plain BP can never match and needs no test here.
		 */
		glue.drop_old_dir_of = d_src.pnf;
		prep = ft_graft_build(ft, dst_ord, dst_len, s_top_prime, cnt,
			&d_dst, &glue, &bctx.held);
	}
	/*
	 * Non-buildable outcomes, mapped to the driver's contract.  A build that
	 * reports OOM or a lost split-fence race has published NOTHING (and, on the
	 * RETRY arm, has not even marked the compressed node), so each is the clean
	 * transient the caller re-descends on; an occupied graft point is the
	 * permanent shape error the up-front dst probe already rejects for the
	 * shapes it can prove, and this is the same answer for the rest.
	 */
	if (prep == FT_GRAFT_PREP_OOM) {
		ret = -ENOMEM;
		goto bail_build;
	}
	if (prep == FT_GRAFT_PREP_RETRY) {
		ret = -EAGAIN;
		goto bail_build;
	}
	if (prep == FT_GRAFT_PREP_POPULATED) {
		/*
		 * Content at or below the graft point that the up-front probe could
		 * not prove (a compressed or skip-encoded occupant it declines to
		 * walk).  For a GRAFT that is the destination-occupied answer its
		 * caller documents; for a MERGE it is a dst shape this cut leaves to
		 * ft_merge_spine_copy.
		 */
		ret = require_empty ? -EEXIST : -EINVAL;
		goto bail_build;
	}
	/*
	 * A reanchoring descent step landed a live node SHALLOWER than the
	 * dispatched child, so the captured publish chain names the wrong level --
	 * the same bail the graft and insert take.  Nothing is reader-visible yet.
	 */
	if (caa_unlikely(d_dst.skip_conflict)) {
		ret = -EAGAIN;
		goto bail_build;
	}
	/*
	 * SHAPE GATE.  Two dst shapes are supported, and what distinguishes them is
	 * only WHICH TWO NODES the graft locks:
	 *
	 *  - NOSPLIT: an absent, append-in-place dst point.  The reserve
	 *    recompaction retires and RELOCATES the dst parent d_dst.pnf, holding
	 *    its parent d_dst.ppnf and releasing it at the flip.  (Its optional
	 *    third member, the SKIP_X great-grandparent, is excluded by requiring a
	 *    PLAIN d_dst.ppnf -- which the pre-lift gate got for free from the src
	 *    descent's own plainness checks, ppnf being shared.)
	 *  - GLUE: the dst key diverges INSIDE a compressed node, so the build
	 *    assembled the whole split cluster invisibly.  It holds the split node
	 *    @cn (fenced by the build, retired by our commit) and -- fenced just
	 *    below -- @glue.publish_parent, the live node whose slot the forward
	 *    publish replaces.  A COMPRESSED publish_parent is refused: the publish
	 *    would then also rewrite the SKIP_X dual, a slot in a THIRD node this
	 *    arm does not hold.  ★ The MERGE arm DOES hold it (@publish_gp_holder,
	 *    taken with the publish parent above), so a compressed publish parent
	 *    is in scope there and only there -- which is what routes an occupied
	 *    destination reached through a path-compressed run to this arm instead
	 *    of to the empty-dst splice.
	 *
	 * Against that pair, the src junction BP (= d_src.pnf) and its parent:
	 *   - BP's parent IS the graft-held node: REUSE the held lock
	 *     (@src_parent_held).  This is the shape the hook started with.
	 *   - BP's parent is any other node: the detach ACQUIRES it itself, guarded
	 *     (@parent_guard).  Deadlock-free -- both acquires are try-locks that
	 *     abort rather than block.
	 *   - BP IS the graft's PUBLISH PARENT: supported.  The detach
	 *     DEL-recompacts that node, which would strand the forward publish in
	 *     the superseded copy; the op's pending publish rides the recompaction
	 *     instead (ft_flip_txn @pending_pub_slot), so the replaced child and
	 *     the shrunk parent go live in ONE flip.
	 *   - BP, or BP's parent, IS the node the graft RETIRES: rejected here,
	 *     PERMANENTLY -- the detach would edit the copy the flip retires, and
	 *     no fallback writer expresses that shape.  (Cross-depth aliases are
	 *     rejected rather than asserted so that a relaxation of the length
	 *     rule cannot silently reach them.)
	 *
	 * Plus, for both: the junctions must be BELOW the root (d_src.ppnf and the
	 * graft's own publish target non-NULL) -- a root-level junction republishes
	 * into &ft->root, a slot with no node word to lock, so its SW park would be
	 * an unguarded plain store.
	 */
	if (merge_dst) {
		/*
		 * The merge's two nodes, in the same roles the graft's pair plays: @D is
		 * the one it RETIRES (through the fenced overlap terminal) and the
		 * publish parent is the one it RELEASES at the flip.  Both are already
		 * held -- D by the overlap fence, the parent by the mark above -- so the
		 * gate below only has to keep the detach's junctions clear of them.
		 */
		graft_c = d_dst.nf;
		graft_p = d_dst.pnf;
	} else if (prep == FT_GRAFT_PREP_GLUE) {
		cn_flag = d_dst.nf;		/* the compressed node the build split */
		graft_c = cn_flag;
		graft_p = glue.publish_parent;
	} else {
		graft_c = d_dst.pnf;
		graft_p = d_dst.ppnf;
		/*
		 * A SHORT landing is IN SCOPE: the descent stopped above @dst_len
		 * on an empty slot, and ft_store_at_graft_point_prepare's other
		 * arm already builds the intermediate path down to the key and
		 * reserves the slot for it -- with the same recompaction, the same
		 * parent hint and the same fold guard as the exact arm.  The two
		 * differ only in WHAT goes in the slot: the payload itself, or a
		 * fresh branch carrying it.  Neither changes which nodes this move
		 * locks, so @graft_c / @graft_p keep their meaning.
		 *
		 * @d_dst.nf still refuses, and it means two different things at the
		 * two depths.  AT @dst_len the point is OCCUPIED -- an argument
		 * error, terminal.  ABOVE it the slot holds an EXTERNAL leaf, a key
		 * ending on the path, which the attach would DISPLACE into the
		 * fresh branch's metadata; that is a legal move this cut does not
		 * express (the displaced shape carries a second publish the fold
		 * has no owner for), so it owes UNCOVERED, not -EINVAL.
		 */
		if (d_dst.nf) {
			ret = d_dst.depth == dst_len ?
				-EINVAL : FT_REKEY_UNCOVERED;
			goto bail_build;
		}
	}
	src_parent_held = d_src.ppnf == graft_p;
	/*
	 * A ROOT-LEVEL JUNCTION is a SHAPE this cut declines, and it owes
	 * FT_REKEY_UNCOVERED rather than -EINVAL.  The two are not
	 * interchangeable: -EINVAL is TERMINAL ("no state of the trie would make
	 * this call legal") while UNCOVERED falls through to the worker that owns
	 * the entry contract -- which is what turns a declined shape into the
	 * NOT_SUPPORTED the public entry documents.  The caller's arguments here
	 * are perfectly valid; what is missing is a node word to park the
	 * republish under (&ft->root has none).
	 *
	 * ☠ AND IT IS NOW AN ORDINARY REQUEST.  While the writer required equal
	 * key lengths, a one-byte source could only ever pair with a one-byte
	 * destination, so this arm was a corner; with the lengths free, "move the
	 * subtree at a depth-1 key" is a normal call, and answering it INVALID
	 * ARGUMENT would be a wrong answer rather than a narrow one.
	 */
	if (!graft_p || !graft_c) {
		ret = FT_REKEY_UNCOVERED;
		goto bail_build;
	}
	/*
	 * The ALIASING terms are -EINVAL: they say the src junction IS the node
	 * the graft retires, which no fallback writer expresses -- and two DLM
	 * tests pin that code as the "refused cleanly, permanently, before any
	 * mutation" answer.  BP == the graft's publish PARENT is NOT among them:
	 * the recompaction fold carries that shape (see @pending_pub_slot).
	 *
	 * NEITHER IS BP == the graft CHILD.  Under a NOSPLIT prep the graft
	 * ADD-recompacts BP itself, and the detach's slot drop rides that same
	 * copy via @pending_del_slot, so BP is superseded ONCE and its child
	 * count is never transiently short.  Under a GLUE prep BP IS the run
	 * the split retires, and there the build DROPS ITS OLD DIRECTION
	 * (@drop_old_dir_of) -- so there is no second edit against that node
	 * left to order.  The term therefore asks whether the build actually
	 * took that path, not which prep it was: a GLUE split that KEPT both
	 * arms still owes a detach into a node this flip retires, and that is
	 * the shape no writer expresses.
	 */
	/*
	 * AN ELEVATING DETACH IS EXPRESSIBLE.  A one-child BP is emptied by
	 * the slot drop, so the detach's climb does not stop on BP -- it
	 * ELEVATES and recompacts BP's PARENT.  The hint below still names
	 * BP's own junction, which is right only while the boundary IS BP;
	 * past a climb ft_detach_node RE-DERIVES it from the walk that
	 * moved, which is the only frame that can.  The orphan chain that
	 * climb clears rides @detach_rc and is freed on the far side of the
	 * one commit.
	 */
	if ((ft_node_compressed(graft_p) && !merge_dst) ||
			ft_node_skip_compressed(graft_p) ||
			(d_src.pnf == graft_c &&
				prep != FT_GRAFT_PREP_NOSPLIT &&
				!glue.old_dir_dropped) ||
			d_src.ppnf == graft_c) {
		ret = -EINVAL;
		goto bail_build;
	}
	/*
	 * Reserve the graft slot edge + the detach struct/state edges, plus (list on)
	 * the six ordered-cell run boundary edges (src unsplice + dst splice), plus
	 * -- for the GLUE shape -- the split cluster's own bound: its deferred
	 * back-edges, forward publish, split retire and free-list tombstones, the
	 * same floor ft_graft_keylen reserves its standalone txn to, plus the
	 * edges a folded chain-compress collapse adds when the detach takes BP
	 * down to one child.  The count walk is depth-bounded and only exists when
	 * the trie keeps rank stats.
	 */
	if (!ft_flip_txn_reserve_extra(txn, FT_REMOVE_COMMIT_REC_MAX_EDGES + 4 +
			(ft->ordered_list ? FT_ORD_CELL_RUN_DETACH_MAX_EDGES +
				FT_ORD_CELL_RUN_RESPLICE_MAX_EDGES : 0) +
			/*
			 * Merged cluster: every deferred re-parent costs up to THREE
			 * records (parent, state guard, offset) now that the fold routes
			 * them through ft_reparent_record, plus one tombstone per retired
			 * overlap node on either side, the forward publish, and the
			 * depth-bounded count walk.  Sized from the read-only pre-pass,
			 * with the same +8 headroom the glue arrays get.
			 */
#ifdef FEATURE_FT_MERGE
			(merge_dst ? 3 * (unsigned int) (mcnt.nd + 8) +
				(unsigned int) (mcnt.nf_dst + mcnt.nf_src + 16) + 8 +
				(unsigned int) (mcnt.ns + 8) +	/* dup-chain splices */
				/*
				 * A COMPRESSED publish parent -- the destination
				 * behind a path-compressed run -- costs TWO more
				 * records than a plain one: the SKIP_X dual the
				 * forward publish re-encodes, and the {LOCK|s -> s}
				 * release of the grandparent that owns it.  Counted
				 * unconditionally rather than off the shape, on the
				 * same reasoning the folded collapse's term states:
				 * a reservation is where OOM gets ANSWERED, and by
				 * the time the shortfall shows up the op is past
				 * the point where there is an answer.
				 */
				2 +
				(ft->rank_stats ? (unsigned int) dst_len + 1 : 0) : 0) +
#endif
			(prep == FT_GRAFT_PREP_GLUE ?
				FT_GLUE_FLOOR_DEFERRED + 7 + 1 + FT_GLUE_FLOOR_FREE +
				(ft->rank_stats ? (unsigned int) dst_len + 1 : 0) : 0) +
#ifdef FEATURE_FT_SKIP_COMPRESSED
			/*
			 * A FOLDED COLLAPSE.  Dropping S_top can leave BP with a single
			 * child, and ft_chain_compress_fused then records the whole
			 * chain-compress -- the merged forward publish, the surviving
			 * child's (parent, offset) back-edge pair, the §4.B parent guard
			 * and the collapsed chain's freeze tombstones -- into THIS txn
			 * instead of committing a second one.  This is its own standalone
			 * bound MINUS the FT_REMOVE_COMMIT_REC_MAX_EDGES already counted
			 * above: the two shapes SHARE that budget, because exactly one of
			 * them runs (@boundary_fused).
			 *
			 * ☠ RESERVED UNCONDITIONALLY, never off the shape gate's nr_child
			 * read.  That read is a plan-time snapshot of a word peers commit
			 * into, so BP can fall to the collapsing arity between the gate and
			 * the detach's own count and make the collapse fire under a
			 * reservation sized for its absence.  A reservation is where OOM
			 * gets answered -- once the detach has cleared BP's slot the op is
			 * past the point where there is an answer, and the shortfall
			 * surfaces as a sticky -ENOMEM at the commit instead.
			 *
			 */
			(ft_group_skip_compressed(ft->group) ?
				3 + 1 /* §4.B parent guard */
				+ 1 /* back-edge (parent, offset) pair */
				+ (ft->rank_stats ? (unsigned int) d_src.depth + 1 : 0) : 0) +
#endif
			/*
			 * AN ELEVATING DETACH's orphan chain.  A one-child BP -- and a
			 * compressed run is exactly that -- is EMPTIED by the slot drop,
			 * so the detach's upward walk clears it and every one-child
			 * ancestor above it, and each cleared node freezes into THIS txn.
			 * The climb stops at the first boundary and can climb no higher
			 * than the src depth, which is what bounds the term.
			 *
			 * Not gated on BP's arity, on the same reasoning the collapse term
			 * above states: that read is a plan-time snapshot of a word peers
			 * commit into, and a reservation is where OOM gets ANSWERED.  Nor
			 * on skip-compression -- the walk elevates through plain nodes
			 * just the same.
			 */
			ft_freeze_reserve(ft, (unsigned int) d_src.depth + 1) +
			0)) {
		ret = -ENOMEM;
		goto bail_build;
	}
	if (merge_dst) {
		/*
		 * Nothing more to prepare: the merged cluster is built, its publish is
		 * set, and its publish parent was fenced with the descent (both above).
		 * The single reserve is already drained by ft_merge_build's own
		 * allocations, so there is no graft reserve to activate here.
		 */
		cds_ft_alloc_reserve_drain(ft, &reserve);
	} else if (prep == FT_GRAFT_PREP_GLUE) {
		/*
		 * FENCE the publish parent OURSELVES, and fail the move on a miss.
		 * ft_glue_txn_commit_edges would otherwise route an unheld parent
		 * through ft_flip_txn_lock_or_guard_parent, which DEGRADES an acquire
		 * miss to a §4.B guard -- correct for the cross-trie graft, wrong here:
		 * this txn is structural_sw, so the forward publish PARKS a plain store
		 * into that node's slot, and the fold's rule is SW iff the op holds the
		 * slot's lock.  A miss (retired / proxied / peer-locked) is the clean
		 * transient: nothing is published, and ft_meta_lock_acquire did not set
		 * the fence.  Mirrors ft_graft_keylen's own pre-swap fence.
		 */
		{
			struct ft_held_anchor pph;

			ft_lock_ctx_init(&lctx_src, &d_src, txn, optxn);
			/*
			 * The REST of the op's held set, exactly as the
			 * store-prepare and detach arms name it:
			 * ft_rekey_cow_stop's marks are handed to the registry by
			 * ft_rekey_marks_to_txn right after the stop (and again
			 * after the dst take), and the glue holds the split-CN
			 * fence.  Under a
			 * coarse spacing this publish parent anchors onto one of
			 * them -- the trie root, for an in-trie move -- and a
			 * frame naming neither refuses the op's own fence.
			 */
			lctx_src.held.extra = marks;
			lctx_src.held.nr_extra = nr_marks;
			lctx_src.held.glue = &glue;
			pp_meta = ft_flag_to_metadata(ft, glue.publish_parent);
			unsigned int ppd;

			/*
			 * The publish parent came from the glue, not from a
			 * descent step, so the window is what dates it; a node
			 * this descent never passed voids the attempt.
			 */
			if (!ft_lock_ctx_depth_of(ft, &lctx_src,
						glue.publish_parent, &ppd) ||
					ft_acquire_member(ft, &lctx_src,
						glue.publish_parent, pp_meta,
						ppd, &pph)) {
				pp_meta = NULL;
				ret = -EAGAIN;
				goto bail_build;
			}
			/*
			 * A SHARED hit is the dedupe WORKING: the fence is in
			 * force from an earlier acquire of this same op, so the
			 * publish may park its SW store -- the fold's rule is SW
			 * iff the op holds the slot's word, and it does.  What it
			 * must NOT do is settle the word twice, which is what
			 * @publish_parent_shared tells the commit.
			 */
			pp_meta = pph.lock;
			pp_snap = pph.lock_snap;
			pp_shared = pph.shared;
		}
		ft_glue_take_publish_parent(&glue, pp_meta, pp_snap, pp_shared);
		/*
		 * The one LIVE node the split cluster re-parents: @cn's displaced
		 * child, which moves onto the fresh suffix (or straight under the fresh
		 * branch when there is no suffix).  Its (parent, offset) pair parks SW,
		 * and the offset lives in the state word ft_meta_nr_child_inc CASes
		 * from an insert BELOW it -- which @cn's fence does not exclude -- so
		 * MARK it, exactly as ft_rekey_cow_stop marks the children whose state
		 * words it parks into.  Released by the re-parent's {live_state ->
		 * live_state} STATE edge, which is recorded unconditionally -- NOT by
		 * the pso edge, which since @118245b0 is its own word and is recorded
		 * only when the slot index CHANGES (resting the release on it leaks a
		 * permanent LOCK on every child that lands at the same index; see
		 * ft_rekey_cow_stop's release-attribution note).  Every bail path
		 * releases it through the @marks sweep.  An external child has no state word and
		 * no metadata, so there is nothing to mark and nothing to clobber.
		 */
		{
			struct cds_ft_compressed_node *cn =
				ft_compressed_node_ptr(cn_flag);
			struct cds_ft_metadata *cm =
				ft_child_state_meta(ft, cn->child);

			if (cm) {
				/*
				 * The DST descent dates this one, and @cn IS its
				 * cursor -- so the displaced child is the cursor's
				 * immediate child, the below-cursor case the window
				 * cannot name (§7.1).  Dating it from the SRC
				 * descent asked the wrong path entirely; that was
				 * inert only because per-node reads no depth at all.
				 */
				struct ft_lock_ctx dctx;
				unsigned int cd;

				ft_lock_ctx_init(&dctx, &d_dst, txn, optxn);
					/*
				 * The REST of the op's held set: the cow_stop
				 * marks reach no registry until the sweep, and
				 * the glue holds the split-CN fence.  Under a
				 * coarse spacing this child anchors onto one of
				 * them, and a frame naming neither refuses a word
				 * the op took two steps earlier.  @nr_extra is
				 * read at the acquire because this same array is
				 * still growing (the mark lands at @nr_marks).
				 */
				dctx.held.extra = marks;
				dctx.held.nr_extra = nr_marks;
				dctx.held.glue = &glue;
				if (!ft_lock_ctx_depth_of(ft, &dctx, cn->child,
						&cd)) {
					struct cds_ft_inode_flag *lp = NULL;

					(void) ft_resolve_parent_slot(cm, ft,
						&lp);
					if (!ft_lock_ctx_depth_of_cursor_child(
							ft, &dctx, lp, &cd)) {
						ret = -EAGAIN;
						goto bail_build;
					}
				}
				if (ft_acquire_member(ft, &dctx, cn->child, cm,
						cd, &marks[nr_marks])) {
					ret = -EAGAIN;
					goto bail_build;
				}
				nr_marks++;
				/* Same handover, for the mark that lands late. */
				ft_rekey_marks_to_txn(txn, marks, nr_marks);
				/*
				 * Tell the glue we already hold this one.  The split
				 * build DEFERS this same child, and
				 * ft_glue_acquire_reparent_marks marks every deferred
				 * entry -- without this it would fail against our own
				 * fence and bail -EAGAIN on every attempt.  Read-only
				 * to the glue: @marks below stays its sole release.
				 */
				glue.caller_holder = cm;
			}
		}
		/*
		 * The NOSPLIT path passes the moved subtree's key count through
		 * ft_store_at_graft_point_commit; the GLUE publish takes it here, and
		 * ft_glue_txn_commit_edges records the +count walk from the stable
		 * publish parent into the same commit.
		 */
		/*
		 * NET ZERO when the old direction was dropped: the subtree LEAVES
		 * the run and RE-ENTERS under the same publish parent's slot, so no
		 * ancestor's key count moves -- the fresh path already carries the
		 * moved count from build, and there is no detach to walk it back.
		 */
		glue.count_delta = glue.old_dir_dropped ? 0 : (long) cnt;
		cds_ft_alloc_reserve_drain(ft, &reserve);	/* GLUE builds its own cluster */
	} else {
		/*
		 * Drive prepare + commit SEPARATELY (not the combined
		 * ft_store_at_graft_point wrapper) so the reserve recompaction's
		 * relocated old dst-parent copy (@gst_st.old_recompacted_node) is
		 * visible here: the graft ALWAYS relocates the attach node for its
		 * atomic publish, and under record_only its old copy stays LIVE until
		 * the caller's commit, so the caller frees it post-commit.
		 */
		/*
		 * ARM THE DETACH'S SLOT DROP ON THE GRAFT'S OWN COPY when BP is
		 * the node this graft recompacts.  Both halves of the decide then
		 * edit BP exactly once, in one copy: born holding the grafted
		 * child and no longer holding S_top.  Without it the detach below
		 * addresses -- and sizes its shape from -- the copy this prepare
		 * retires, whose committed child count is short by the child
		 * being added here.
		 *
		 * The expected-old is the LIVE flag the src descent resolved;
		 * ft_rekey_cow_stop retires S_top's own state word and builds
		 * S_top', but leaves BP's slot holding S_top until the commit.
		 */
		if (d_src.pnf == graft_c
#ifdef FT_RED_NO_DEL_FOLD
				&& 0	/* red control: see fractal-trie-internal.h */
#endif
		   ) {
			txn->pending_del_slot = d_src.nfp;
			txn->pending_del_expected = s_top;
		}
		cds_ft_alloc_reserve_activate(ft, &reserve);
		{
			/*
			 * The op's outstanding marks -- ft_rekey_cow_stop's @stop
			 * fence and one per COW'd child -- reach no registry until
			 * the sweep below, so the store's own recompactions can
			 * only see them through this frame.  Under a coarse spacing
			 * they collapse onto one word and the store refuses its own
			 * fence: the CDS_FT_STATUS_BUSY_ERROR that "is not expected
			 * single-threaded".
			 */
			struct ft_lock_ctx octx;

			ft_lock_ctx_init(&octx, &d_src, txn, optxn);
			octx.held.extra = marks;
			octx.held.nr_extra = nr_marks;
			gst = ft_store_at_graft_point_prepare(ft, dst_ord,
				dst_len, &d_dst, s_top_prime, cnt, &glue,
				&octx.held, &gst_st);
		}
		if (gst == CDS_FT_STATUS_OK)
			/*
			 * NET count.  With the drop folded into this same copy the
			 * subtree LEAVES and RE-ENTERS one node: BP' holds it under
			 * the grafted byte instead of the src byte, so BP's key
			 * count -- and every ancestor's -- is unchanged, and the
			 * detach below that would have walked -@cnt does not run.
			 */
			gcst = ft_store_at_graft_point_commit(ft, &attached_nf, &adepth,
					NULL /*run*/, &gst_st,
					txn->pending_del_folded ? 0 : (long) cnt);
		cds_ft_alloc_reserve_deactivate(ft);
		cds_ft_alloc_reserve_drain(ft, &reserve);
		if (gst != CDS_FT_STATUS_OK || gcst != URCU_TXN_STATUS_OK) {
			/*
			 * Not expected single-threaded with the reserve pre-filled.
			 * Record-only commit leaves the shared txn intact (terminal commit
			 * gated off), so the caller owns cleanup: free S_top', abort the
			 * glue build, destroy the txn.  (prepare failure freed its own
			 * invisible build + left glue clean.)
			 */
			/* NULL on the merge path: no COW */
			ft_rekey_free_stop_prime(ft, s_top_prime);
			ft_glue_abort(ft, &glue);
	if (src_glue_live) {		/* merged cluster's src side */
		ft_glue_abort(ft, &src_glue);
		src_glue_live = false;
	}
			ft_flip_txn_destroy(txn);
			ret = -EIO;
			goto sweep;
		}
	}

	/*
	 * 3. Detach-fold: remove S_top from BP (clear its slot + nr_child--).  In the
	 * default (concurrent-safe) build EVERY popcount delete recompacts BP, and
	 * that recompaction republishes into BP's parent.  Two shapes, decided by the
	 * gate above and carried by @src_parent_held:
	 *   - BP's parent IS the spine ancestor the graft's dst-parent recompaction
	 *     already holds the lock (both junctions are its children).  REUSE the held
	 *     lock; re-acquiring it would abort -EAGAIN.
	 *   - BP's parent is a node this op holds nothing on.  The recompaction
	 *     acquires and releases it itself, in its own up-front lock-set commit.
	 * Either way the identity is passed EXPLICITLY, and its slot with it, rather
	 * than letting the recompaction resolve BP's current parent: that resolve is
	 * racy, and a peer that re-homed BP since this descent would make "BP's
	 * parent" a node this op does NOT hold -- an SW park into a slot that no
	 * longer holds BP.  @parent_guard puts the BP.parent == @parent read-set
	 * guard on the acquire commit in BOTH shapes (the held arm guards
	 * unconditionally), so a re-home ABORTS it (-EAGAIN, trie pristine) and the
	 * caller re-descends.  BP's parent is never compressed (the src descent
	 * rejects compressed nodes at every level it walks), so no SKIP_X dual and no
	 * @gp member.
	 */
	/*
	 * ★ THE CONTENT TXN IS PART OF THIS OP'S HELD SET, and naming it is
	 * MANDATORY for a fold.  @txn's locks[] registry is where an acquire whose
	 * RELEASE has been RECORDED but not yet COMMITTED still lives, and under
	 * the fold that is every acquire this op has made -- the one decide has not
	 * run yet.  ft_flip_txn_record_release_lock drops the hold-trace entry as
	 * soon as it records the {LOCK|s -> s} edge ("the commit owns this release
	 * now") while the WORD keeps its LOCK bit until that commit lands, so the
	 * registry is the ONLY remaining witness that this op holds the word.
	 *
	 * ☠ WITHOUT IT THE MOVE LIVELOCKS.  The graft side's ft_node_recompact
	 * acquires the src junction and records its release into @txn; the collapse
	 * ft_chain_compress_fused runs inside the same decide and meets that LOCK
	 * bit still set.  A registry this frame does not name is a hold
	 * ft_dlm_acquire_set's dedupe cannot see, so ft_dlm_lock reads the op's OWN
	 * mark as contention and the -EAGAIN retry re-derives the identical plan
	 * forever -- single-threaded, so no peer can ever clear it.
	 *
	 * The op's marks so far -- ft_rekey_cow_stop's @stop fence and one per
	 * COW'd child -- reach no txn registry until the sweep below, so the
	 * detach's own acquires see THOSE through @extra on this frame instead.
	 * Under a coarse spacing S_top's fence lands on BP, which is exactly the
	 * node this detach recompacts.
	 */
	/*
	 * SKIP THE DETACH ENTIRELY when its slot drop already rode the graft's
	 * recompaction of BP (@pending_del_slot).  BP == graft_c means the src
	 * slot and the grafted slot are BOTH IN BP, so the move is a slot rename
	 * inside one node and that one copy is the whole structural edit: BP is
	 * superseded ONCE, its child set is right at birth, and there is no
	 * second recompaction left to size a shape from.
	 *
	 * Armed-but-not-folded falls through here and detaches normally -- the
	 * flag is set by the copy loop that actually consumed the drop, never by
	 * the arming.
	 */
	/*
	 * ...and skip it for the same reason when the GLUE split DROPPED the
	 * old direction: the slot that would be cleared lives in a run this
	 * flip retires WHOLE, and the fresh path published in its place never
	 * held the src edge at all.
	 */
	if (!txn->pending_del_folded && !glue.old_dir_dropped) {
		ft_lock_ctx_init(&lctx_src, &d_src, txn, optxn);
		lctx_src.held.extra = marks;
		lctx_src.held.nr_extra = nr_marks;
		/*
		 * And the GLUE, which holds the rest -- the publish-parent fence above
		 * all.  Under a coarse spacing that fence and BP's recompaction are ONE
		 * word, so without this the detach refuses a fence this op took three
		 * steps earlier.
		 */
		lctx_src.held.glue = &glue;
		ret = ft_detach_node(ft, &lctx_src, d_src.nfp, d_src.pnfp, d_src.depth,
				false /*free_detached_subtree: S_top is retired by cow_stop*/,
				NULL /*fuse_cell: list off*/, &pub, NULL /*run*/,
				NULL /*retire_glue*/, NULL /*freeze_leaf*/,
				-(long) cnt, txn /*shared_txn*/, true /*record_only*/,
				&(const struct ft_parent_hint){	/* BP's parent: held or acquired */
					.parent = d_src.ppnf, .slot = d_src.pnfp,
					.gp = NULL, .gp_slot = NULL,
					.parent_held = src_parent_held,
					.parent_guard = true },
				&detach_rc /*old + fresh BP copies, reclaimed post-commit*/);
		if (ret) {
			/*
			 * Pre-commit bail.  Reclaim EVERY unpublished fresh copy built so far --
			 * S_top' AND the graft's relocated dst-parent copy (@gst_st.dest, whose
			 * old counterpart is @gst_st.old_recompacted_node): the graft always
			 * relocates the attach node, and under record_only nothing it built is
			 * published until the caller's commit, so this arm owns the fresh copy
			 * exactly as the commit-abort arm below does.  (@glue's own build is
			 * covered by ft_glue_abort; nr_built is 0 for the NOSPLIT shape.)
			 * REACHABLE: a peer holding BP -- or, in the non-shared-parent shape,
			 * BP's own parent -- makes the detach's up-front lock-set acquire abort
			 * -EAGAIN right here, as does a peer that re-homed BP since this
			 * descent (the @parent_guard read-set validation).  The
			 * single-threaded route -- a same-junction move (BP == the graft's
			 * own attach node, hence already LOCK-held) -- is excluded: the shape
			 * gate rejects it up front, before any of this is built.
			 */
			pp_meta = NULL;		/* ft_glue_abort below is the single owner */
			/* NULL on the merge path: no COW */
			ft_rekey_free_stop_prime(ft, s_top_prime);
			ft_glue_abort(ft, &glue);
				if (src_glue_live) {	/* merged cluster's src side */
					ft_glue_abort(ft, &src_glue);
					src_glue_live = false;
				}
			ft_flip_txn_destroy(txn);
			/*
			 * ☠ FREE AFTER THE REGISTRY SWEEP, NEVER BEFORE IT.
			 * @gst_st.dest carries a fence this attempt registered on @txn
			 * (the born-locked relocation acquire), and ft_flip_txn_destroy's
			 * sweep RELEASES it through the node's own metadata.  Freeing
			 * first hands that word to the allocator, which can refund the
			 * item into the active reserve and re-zero its metadata on the
			 * next draw -- so the release lands on a cleared word (assert) or
			 * on a LIVE peer's word (corruption).
			 */
			if (gst_st.old_recompacted_node)
				free_cds_ft_node_unpublished(ft, ft_node_ptr(gst_st.dest));
			goto sweep;
		}
	}

	/*
	 * 3b. Cell-fold (list on): record the six ordered-cell run boundary edges into
	 * the SHARED txn so the run unsplices from src + re-splices at dst ATOMICALLY
	 * with the structural move.  Driver-managed (run == NULL to the structural folds
	 * above) rather than threaded through them, because the dst-splice PLAIN-STORES
	 * the run's outer links (rfc->prev, rlc->next) at record time, so the src unsplice
	 * -- which READS those links to find the src neighbours -- must record FIRST.  The
	 * structural folds' order (the graft acquires its lock set before the detach
	 * reuses or acquires BP's parent) can't provide that, so the cells are
	 * recorded here, in the required order, on the still-pristine live list (no
	 * structural fold above published anything).
	 *
	 * All SIX edges ride the ONE commit (plan Q3): the src unsplice's two neighbour
	 * back-edges, the dst splice's two neighbour back-edges, AND -- via the same-trie
	 * ft_ord_cell_run_resplice_edges instead of the cross-trie
	 * ft_ord_cell_run_splice_edges -- the run's own two OUTER links, which the
	 * cross-trie form would PLAIN-STORE.  A plain store is wrong for a LIVE run: it
	 * publishes the dst neighbours before the boundary flip and survives an abort,
	 * permanently breaking the back-edges.  Cell edges are always MW, so a peer's
	 * conflicting splice aborts this commit clean and the caller re-descends.
	 */
	if (ft->ordered_list) {
		struct ft_ord_cell_edge cedges[FT_ORD_CELL_RUN_DETACH_MAX_EDGES +
			FT_ORD_CELL_RUN_RESPLICE_MAX_EDGES];
		struct ft_ord_cell *rfc, *rlc, *src_pred, *src_succ;
		unsigned int cn;

		/*
		 * DISTINCT-SLOT precondition (the engine's, rcu-txn-mcas.h: a txn's records
		 * must target pairwise-distinct slots).  Two of the six edges coincide iff a
		 * dst neighbour IS a src neighbour: dst_pred == src_pred puts two records on
		 * &src_pred->lnode.next, dst_succ == src_succ two on &src_succ->lnode.prev.
		 * The endpoint-adjacency guard does NOT exclude that, because the dst pair was
		 * derived earlier than these src reads and a peer move can have shifted the
		 * neighbourhood in between -- MEASURED at 1.5-2% of commits before the move
		 * counter pinned the window shut.  ft_flip_txn_create's expect_conflict makes
		 * the engine RECONCILE rather than corrupt (it poisons -> clean abort), but
		 * relying on that is relying on a fallback: bail explicitly instead, so the
		 * distinctness the engine requires holds BY CONSTRUCTION.  Clean -EAGAIN: the
		 * txn has recorded structural edges but published nothing, so the bail below
		 * unwinds exactly like the other pre-commit bails.
		 */
		rfc = ft_ord_cell_ptr(rcu_dereference(run_rfirst->prev));
		rlc = ft_ord_cell_ptr(rcu_dereference(run_rlast->prev));
		src_pred = ft_ord_cell_resolve_ord(&rfc->lnode.prev);
		src_succ = ft_ord_cell_resolve_ord(&rlc->lnode.next);
		if (run_keeps_pos) {
			/*
			 * The run keeps its ordered position (the dst abuts it), so there
			 * is nothing to splice -- but "nothing changed" still has to be
			 * ASSERTED at the decide.  Between the derivation above and this
			 * commit a peer can splice a key into the run's own boundary, and
			 * the new keys would then sort on the wrong side of it: a list
			 * left well-formed and no longer key-ordered, with no edge of this
			 * commit noticing.  On the moving path the six edges are what
			 * notices; here two VALIDATE edges (old == new) on the run's OUTER
			 * links do it, so such a peer aborts this commit clean and the
			 * caller re-derives.
			 *
			 * The run's own links rather than A's and B's: a peer may REMOVE a
			 * neighbour, and an edge on a dying cell's slot is worse than one
			 * on a cell this move already owns.  Distinct by construction --
			 * ->prev and ->next are different fields even for a one-cell run.
			 */
			cedges[0].tag = URCU_TXN_TAG;
			cedges[0].slot = (struct ft_ord_cell **) &rfc->lnode.prev;
			cedges[0].old_target = src_pred;
			cedges[0].new_target = src_pred;
			cedges[1].tag = URCU_TXN_TAG;
			cedges[1].slot = (struct ft_ord_cell **) &rlc->lnode.next;
			cedges[1].old_target = src_succ;
			cedges[1].new_target = src_succ;
			ft_ord_cell_record_into_ft(ft, txn, cedges, 2);
			goto cells_done;
		}
#ifdef FEATURE_FT_MERGE
		if (run_interleaves) {
			/*
			 * INTERLEAVE: every relink is recorded, so the cells reorder
			 * with the structural publish in the one commit.  The edge set
			 * is unbounded in the run length, hence heap-allocated and
			 * freed here; a refusal is a clean pre-commit bail like the
			 * distinct-slot one below.
			 */
			struct ft_ord_cell_edge *iedges = NULL;
			unsigned int in = 0;
			int iret = ft_rekey_ord_interleave(ft, probe_D, dst_len,
					src_len, run_rfirst, run_rlast,
					merged_keys, &iedges, &in);

			if (!iret && !ft_flip_txn_reserve_extra(txn, in)) {
				free(iedges);
				iedges = NULL;
				iret = -ENOMEM;
			}
			if (iret) {
				pp_meta = NULL;	/* ft_glue_abort: single owner */
				ft_rekey_free_stop_prime(ft, s_top_prime);
				if (detach_rc.new_flag)
					free_cds_ft_node_unpublished(ft,
						ft_node_ptr(detach_rc.new_flag));
				ft_rekey_collapse_free_unpublished(ft,
					&detach_rc.collapse);
				ft_glue_abort(ft, &glue);
				if (src_glue_live) {
					ft_glue_abort(ft, &src_glue);
					src_glue_live = false;
				}
				ft_flip_txn_destroy(txn);
				/*
				 * ☠ FREE AFTER THE REGISTRY SWEEP, NEVER BEFORE IT.
				 * @gst_st.dest carries a fence this attempt registered on @txn
				 * (the born-locked relocation acquire), and ft_flip_txn_destroy's
				 * sweep RELEASES it through the node's own metadata.  Freeing
				 * first hands that word to the allocator, which can refund the
				 * item into the active reserve and re-zero its metadata on the
				 * next draw -- so the release lands on a cleared word (assert) or
				 * on a LIVE peer's word (corruption).
				 */
				if (gst_st.old_recompacted_node)
					free_cds_ft_node_unpublished(ft,
						ft_node_ptr(gst_st.dest));
				ret = iret;
				goto sweep;
			}
			ft_ord_cell_record_into_ft(ft, txn, iedges, in);
			free(iedges);
			goto cells_done;
		}
#endif /* FEATURE_FT_MERGE: an occupied dst is a merge */
		if (src_pred == ft_ord_or_sentinel(ft, run_dpred) ||
				src_succ == ft_ord_or_sentinel(ft, run_dsucc)) {
			pp_meta = NULL;		/* ft_glue_abort: single owner */
			/* NULL on the merge path: no COW */
			ft_rekey_free_stop_prime(ft, s_top_prime);
			if (detach_rc.new_flag)
				free_cds_ft_node_unpublished(ft,
					ft_node_ptr(detach_rc.new_flag));
			ft_rekey_collapse_free_unpublished(ft,
				&detach_rc.collapse);
			ft_glue_abort(ft, &glue);
			if (src_glue_live) {	/* merged cluster's src side */
				ft_glue_abort(ft, &src_glue);
				src_glue_live = false;
			}
			ft_flip_txn_destroy(txn);
			/*
			 * ☠ FREE AFTER THE REGISTRY SWEEP, NEVER BEFORE IT.
			 * @gst_st.dest carries a fence this attempt registered on @txn
			 * (the born-locked relocation acquire), and ft_flip_txn_destroy's
			 * sweep RELEASES it through the node's own metadata.  Freeing
			 * first hands that word to the allocator, which can refund the
			 * item into the active reserve and re-zero its metadata on the
			 * next draw -- so the release lands on a cleared word (assert) or
			 * on a LIVE peer's word (corruption).
			 */
			if (gst_st.old_recompacted_node)
				free_cds_ft_node_unpublished(ft,
					ft_node_ptr(gst_st.dest));
			ret = -EAGAIN;
			goto sweep;
		}

		/* src unsplice FIRST: reads the run's pristine outer links -> src neighbours. */
		cn = ft_ord_cell_run_detach_edges(ft, run_rfirst, run_rlast,
				&rfc, &rlc, cedges, 0);
		/*
		 * dst splice into the gap located up front on the pristine list.  Records the
		 * run's outer links rather than storing them, so the whole move is atomic and
		 * abort-clean; all six slots are distinct per the check above.
		 */
		cn = ft_ord_cell_run_resplice_edges(ft, rfc, rlc, run_dpred,
				run_dsucc, cedges, cn);
		ft_ord_cell_record_into_ft(ft, txn, cedges, cn);
cells_done:
		;
	}

	/*
	 * 3c. GLUE shape only: record the split cluster's publish -- its hidden
	 * back-pointers (plain stores on unpublished nodes), the displaced child's
	 * live re-parent, the forward edge into the fenced publish parent, @cn's
	 * retire and the count walk -- into the SAME txn.  It runs LAST, after the
	 * detach and cell folds, because it is the step that does live bookkeeping
	 * and is specified to be called where abort is impossible; every arm that
	 * can still bail is above it.  (The NOSPLIT shape is the other way round --
	 * its prepare/commit is where its recompaction ACQUIRES the lock set the
	 * detach then reuses, so it has to run first.)  Under record_only this
	 * records and returns OK without committing; the caller's one commit below
	 * publishes it, and from here the txn registry -- not this function -- owns
	 * the publish parent's fence.
	 */
	if (prep == FT_GRAFT_PREP_GLUE || merge_dst) {
		/*
		 * The status is CHECKED, not discarded: the fold's re-parent mark
		 * acquire runs inside this call, before it records anything, and CAN
		 * miss on a contended child.
		 * Discarding that would carry on to the commit below with children this
		 * op does not hold, SW-parking their state words unexcluded: precisely
		 * the silent-success shape (ignored commit status, op reports OK) that
		 * cost a whole source subtree once already.
		 *
		 * The miss is clean: it happens before the first record and before
		 * ft_glue_tombstone_free_list, so nothing of this cluster is in @txn and
		 * NOTHING is published -- the txn still carries only the detach and cell
		 * records, which this bail discards with it.  @pp_meta ownership has NOT
		 * transferred (that happens by recording), so we still owe its release
		 * and must not NULL it here.
		 */
		if (ft_glue_txn_commit_edges(ft, &glue, NULL, 0) !=
				URCU_TXN_STATUS_OK) {
			/*
			 * ☠ DO NOT release @pp_meta here.  An earlier version did,
			 * reasoning that a failure could only come from the mark
			 * acquire at the top of ft_glue_txn_commit_edges -- before
			 * any record, so before ownership passes to the txn.  That
			 * is an ARGUMENT about which failure happens, and under a
			 * SHARED destination it is false: commit_edges also fails
			 * AFTER routing the publish parent through the txn, which
			 * NULLs g->publish_parent_holder and takes the fence with
			 * it.  Clearing it then asserts on an unheld word.
			 *
			 * ft_glue_abort below is the choke point and already does
			 * the right thing either way -- clear_IF_HELD, guarded on
			 * the holder field the transfer NULLs.  Just disown it so
			 * bail_build's own clear cannot double up.
			 */
			pp_meta = NULL;
			/* NULL on the merge path: no COW */
			ft_rekey_free_stop_prime(ft, s_top_prime);
			if (detach_rc.new_flag)
				free_cds_ft_node_unpublished(ft,
					ft_node_ptr(detach_rc.new_flag));
			ft_rekey_collapse_free_unpublished(ft,
				&detach_rc.collapse);
			ft_glue_abort(ft, &glue);
			if (src_glue_live) {	/* merged cluster's src side */
				ft_glue_abort(ft, &src_glue);
				src_glue_live = false;
			}
			ft_flip_txn_destroy(txn);
			ret = -EAGAIN;
			goto sweep;
		}
		pp_meta = NULL;		/* ownership transferred to @txn */
		/*
		 * The merged cluster's SRC side has no publish of its own -- only
		 * retires, S_top's among them -- so it never goes through
		 * ft_glue_txn_commit_edges.  Record its freezes into the shared txn
		 * directly, here, on the committing path and after the last bail above,
		 * exactly where the dst side's own tombstone step just ran.
		 */
		if (src_glue_live)
			ft_glue_tombstone_free_list(&src_glue);
		/*
		 * The dup-chain appends, recorded (not stored) so the src duplicates
		 * become reachable ATOMICALLY with the merged structure -- a collided
		 * key never momentarily shows only its dst side.  After the cluster's
		 * edges, exactly as ft_merge_spine_copy orders it.
		 */
#ifdef FEATURE_FT_MERGE
		if (merge_dst)
			ft_glue_record_splices(ft, &glue, txn);
#endif
	}

	/* 4. ONE commit of the whole stitch (consumes txn). */
#ifdef FEATURE_FT_FAULT_INJECT
	/*
	 * Test-only: abort this commit exactly as a peer winning a raced MW slot
	 * would.  Without it the fold's abort branch below is DEAD -- and not for
	 * want of contention: the shared-destination merge oracle drove 62342
	 * publish-parent fence misses and 1079 commit_edges misses over 71218
	 * merges and still took this exit 0 times, because every acquire is AHEAD
	 * of the commit and turns the peer away first.  @acquire_miss is the
	 * engine's own discard-unpublished route, so what runs below is the real
	 * unwind, not a synthesised status.
	 */
	if (merge_dst && cds_ft_fault_commit_countdown >= 0) {
		if (cds_ft_fault_commit_countdown == 0) {
			cds_ft_fault_commit_countdown = -1;
			txn->acquire_miss = true;
		} else {
			cds_ft_fault_commit_countdown--;
		}
	}
#endif
	/*
	 * The marks' anchor releases.
	 *
	 * `marks_consumed` below claims the commit consumed every mark, and its
	 * two justifications -- a child's {live_state -> live_state} re-parent
	 * edge, @stop's retire -- are claims about each NODE's OWN word.  Under a
	 * coarse spacing the word the acquire TOOK is the node's ANCHOR, and no
	 * node terminal touches it: without this the LOCK survives the commit and
	 * leaks, so the next op to anchor there refuses it forever and a later
	 * release of it asserts.
	 *
	 * Placed here because this is past the LAST acquire -- @nr_marks is still
	 * growing up to the displaced-child mark above -- NOT for ordering:
	 * ft_flip_txn_record_anchor_release_held reads the word through the txn,
	 * so it chains onto whatever the op has already recorded there and is
	 * order-independent by construction (measured: recording it right after
	 * ft_rekey_cow_stop is equally green).  Self-guarding too, so an
	 * UNcoarsened mark -- whose node terminal IS its release -- records
	 * nothing and the default granularity stays byte-identical.
	 */
	for (i = 0; i < nr_marks; i++) {
		if (marks[i].shared)
			continue;	/* an earlier acquire owns its release */
		if (!ft_flip_txn_reserve_extra(txn, 1)) {
			ret = -ENOMEM;
			goto bail_build;
		}
		ft_flip_txn_record_anchor_release_held(txn, marks[i].lock);
	}
	st = ft_flip_txn_commit(ft, txn);
	if (st == URCU_TXN_STATUS_OK) {
		/*
		 * The commit CONSUMED every mark in @marks: each child's release is
		 * ft_reparent_record_meta's {live_state -> live_state} state edge
		 * (live_state has LOCK masked, and the edge is recorded
		 * unconditionally), and @stop's is its retire.  So the sweep below
		 * must NOT run -- see its own comment.
		 */
		marks_consumed = true;
		ft_glue_free_old(ft, &glue);		/* graft old copies */
		/*
		 * The merged cluster's SRC side: its free list holds S_top itself (and
		 * any src overlap node the build copied), retired by this commit.  That
		 * is what makes the cds_ft_free_item_deferred below a MERGE-path double
		 * free -- the merge already owns S_top's reclaim, so only the cow_stop
		 * path still owes it.
		 */
		if (src_glue_live)
			ft_glue_free_old(ft, &src_glue);
		/*
		 * Chain holders are NOT in the txn registry, so the commit does not
		 * consume them: release them here, the committed path's own point.
		 * Every non-committed path reaches ft_glue_abort, which owns the same
		 * release -- the choke point, not the exits that happen to be visible.
		 */
		ft_glue_release_splice_holders(&glue);
		/*
		 * The reserve recompaction's relocated old dst-parent copy: its retire
		 * committed with the flip (deferred past readers via the recompact's
		 * fenced tombstone), so reclaim it now -- mirrors ft_store_at_graft_
		 * point_commit's own post-commit free (ft-graft.h).
		 */
		if (gst_st.old_recompacted_node)
			free_cds_ft_node(ft, gst_st.old_recompacted_node);
		if (detach_rc.old_node)
			free_cds_ft_node(ft, detach_rc.old_node);	/* old BP copy */
		/* The folded collapse's retired chain: this commit unlinked it. */
		ft_rekey_collapse_free_retired(ft, &detach_rc.collapse);
		/* Likewise an ELEVATING detach's orphan chain. */
		ft_rekey_detach_free_orphans(ft, &detach_rc);
		/*
		 * Old S_top after the grace period -- but ONLY on the cow_stop path.
		 * The merge retires S_top through @src_glue's free list, so
		 * ft_glue_free_old above already owns that reclaim; doing it here too
		 * is a double free.
		 */
		if (!merge_dst)
			cds_ft_free_item_deferred(ft, s_top_meta);
		ret = 0;
	} else {
		/*
		 * Abort (a peer won a raced MW slot): NOTHING published, so reclaim every
		 * UNPUBLISHED fresh copy -- S_top', the graft's relocated dst-parent copy
		 * (@gst_st.dest), and the detach's relocated BP copy (@detach_rc.new_flag)
		 * -- and abort the glue build.  The retired old copies stay LIVE (their
		 * tombstones rolled back), so they are NOT freed here.  Unreachable under
		 * the single-writer contract, kept leak-free for future concurrent use.
		 */
		/* NULL on the merge path: no COW */
		ft_rekey_free_stop_prime(ft, s_top_prime);
		if (gst_st.old_recompacted_node)
			free_cds_ft_node_unpublished(ft, ft_node_ptr(gst_st.dest));
		if (detach_rc.new_flag)
			free_cds_ft_node_unpublished(ft, ft_node_ptr(detach_rc.new_flag));
		ft_rekey_collapse_free_unpublished(ft, &detach_rc.collapse);
		ft_glue_abort(ft, &glue);
			if (src_glue_live) {	/* merged cluster's src side */
				ft_glue_abort(ft, &src_glue);
				src_glue_live = false;
			}
		ret = -EAGAIN;
	}
	ft_glue_fini(&glue);
	if (src_glue_live)
		ft_glue_fini(&src_glue);
	goto sweep;

bail_build:
	/*
	 * Common unwind for every bail between the graft build and the point where
	 * the graft's own step has committed its records: the build published
	 * nothing, so drop the (possibly filled) node reserve, release the publish
	 * parent's fence if this function still owns it -- once
	 * ft_glue_txn_commit_edges has run, the txn registry owns it and clearing
	 * here would race a peer's re-mark -- free the unpublished S_top' copy, and
	 * let ft_glue_abort free the invisible cluster and release the split node's
	 * fence (its single clear point).  @marks is swept below, as on every path.
	 */
	cds_ft_alloc_reserve_drain(ft, &reserve);
	/*
	 * The publish-parent fence has EXACTLY ONE owner on this path, and it is
	 * ft_glue_abort below: @glue.publish_parent_holder is set in the same breath
	 * as @pp_meta, ft_glue_abort clears it IF HELD, and the txn NULLs that field
	 * when a record transfers ownership.  A second clear here was right only
	 * while every bail above was known to still hold the fence -- under a SHARED
	 * destination that stopped being true, and it asserted on an unheld word.
	 * Disown and let the choke point do it.
	 */
	pp_meta = NULL;
	/* NULL on the merge path: no COW */
	ft_rekey_free_stop_prime(ft, s_top_prime);
	/*
	 * REACHED AFTER THE DETACH TOO -- the mark-release reserve loop bails here
	 * -- so this label owes the same unpublished copies the post-detach @sweep
	 * bails free: the detach's relocated BP copy and a folded collapse's merged
	 * node.  Both are zeroed on every path that reaches here BEFORE the detach,
	 * which is what makes one unconditional unwind correct for both halves; and
	 * the post-detach paths that free them themselves @goto sweep, skipping
	 * this label, so neither can be freed twice.
	 */
	if (detach_rc.new_flag)
		free_cds_ft_node_unpublished(ft, ft_node_ptr(detach_rc.new_flag));
	ft_rekey_collapse_free_unpublished(ft, &detach_rc.collapse);
	ft_glue_abort(ft, &glue);
			if (src_glue_live) {	/* merged cluster's src side */
				ft_glue_abort(ft, &src_glue);
				src_glue_live = false;
			}
	ft_flip_txn_destroy(txn);
	/*
	 * @gst_st.dest was LEAKED here: this bail is reachable with a relocated
	 * dest already built (the marks-reserve ENOMEM above).  @gst_st is
	 * zero-initialised, so the guard is safe on paths that never grafted.
	 * After the sweep, for the reason spelled out at the bails above.
	 */
	if (gst_st.old_recompacted_node)
		free_cds_ft_node_unpublished(ft, ft_node_ptr(gst_st.dest));

sweep:
	/*
	 * Release every mark the commit did NOT consume.
	 *
	 * ONLY on paths that did not reach a successful commit.  This used to run
	 * unconditionally, on the reading that a consumed mark makes
	 * clear_if_held "a no-op" -- true only with no peers.  After a successful
	 * commit each of these nodes is LIVE and CLEAN, so a peer that re-marked
	 * one in the commit->sweep window has its fence CLEARED here: fence theft,
	 * the same shape as the publish-parent holder's, and the precise thing
	 * ft_glue_release_reparent_marks' own contract ("call ONLY on paths that
	 * did NOT reach a successful commit") exists to avoid.  The fold is not
	 * concurrent yet, which is why this was invisible; it has to be right
	 * before it is.
	 */
	/*
	 * ...and NEVER a mark the txn owns (@txn_owned): its clear rides
	 * ft_flip_txn_lock_release_all, whose release is the STRICT one, so a
	 * second clear here would leave that assert reading an already-clean word.
	 */
	if (!marks_consumed)
		for (i = 0; i < nr_marks; i++)
			if (!marks[i].shared && !marks[i].txn_owned)
				ft_meta_lock_release_if_held(marks[i].lock);
	return ret;
}

/*
 * Retry wrapper: the escalation lane the fold never had.
 *
 * Every bail in the attempt above is abort-clean (the trie is byte-for-byte as
 * before), so retrying is just calling again -- but calling again is not enough
 * on its own.  Progress under contention comes from AGING a PERSISTENT handle:
 * urcu_txn_conflict() advances @optxn->retry, and once it reaches the fallback
 * budget the writer takes its FIFO turn on the trie's escalation domain and
 * commits without competition.  A fresh handle per attempt -- which is what the
 * caller's external "just call again" loop produced -- resets that age to zero
 * every time, so the writer never qualifies and spins instead.
 *
 * SCOPE: this arbitrates COMMITS.  It deliberately does NOT wrap the per-node
 * LOCK acquires -- an escalated acquirer would hold its FIFO turn while
 * spinning for a holder that is itself funnelled behind that turn (the circular
 * wait documented at the FT-wide writer lock).  An acquire miss stays a clean
 * bail that re-descends.
 *
 * -EINVAL (shape) and -ENOMEM are terminal; the transient contention codes the
 * attempt documents (-EAGAIN, -EIO) are what this loop absorbs.
 */
#ifdef FT_DEBUG_REKEY_RETRY_CAP
# include <stdio.h>
# include <stdlib.h>
# ifndef FT_REKEY_RETRY_CAP
#  define FT_REKEY_RETRY_CAP	50000
# endif
static __thread unsigned int ft_rekey_attempts;
#endif

static
int ft_rekey_graft_simple_locked(struct cds_ft *ft,
		const uint8_t *src_key, size_t src_len,
		const uint8_t *dst_key, size_t dst_len, bool require_empty)
{
	const struct rcu_flavor_struct *flavor = ft->group->flavor;
	struct urcu_txn optxn;
	int ret;

	/*
	 * JOIN THE PEER EXCLUSION PROTOCOL.  This writer parks its structural
	 * edges SW (ft_flip_txn_set_structural_sw), and an SW park CANNOT FAIL:
	 * it does not arbitrate.  That is legal only where the op excludes every
	 * peer writer over those slots.  On a FINE trie the per-node DLM LOCK
	 * try-locks are that protocol and this scope is INERT
	 * (ft_writer_lock_scope_enter returns early on @lock_fine).  On a COARSE
	 * trie the protocol is the FT-WIDE MUTEX, which every other writer takes
	 * through exactly this scope and which this op used to take nowhere --
	 * so its unfailable parks arbitrated against nobody.
	 *
	 * PLACEMENT.  After the move gate, never before it: ft_move_gate_enter
	 * waits a GRACE PERIOD, and a peer parked on this mutex is an ONLINE,
	 * non-quiescent QSBR reader -- holding the lock across that gate would be
	 * a writer waiting on the very readers it is blocking.  Both entries
	 * (_cds_ft_debug_rekey_graft_simple and ft_rekey_one_decide's dispatch)
	 * enter the gate before calling here, so this placement satisfies it for
	 * both.  Around the WHOLE retry loop rather than each attempt, which is
	 * the shape cds_ft_remove already uses, and safe for the same reason:
	 * @optxn parks QUIESCENT (below), so an escalated wait inside this scope
	 * does not stop the grace periods its peers need.
	 *
	 * No ft_writer_lock_gp_wait exists anywhere under the attempt (the two in
	 * this unit are on the STAGED path), so nothing here drops and retakes the
	 * lock mid-scope.
	 */
	CDS_FT_SCOPED_WRITER(ft);

#ifdef FT_DEBUG_REKEY_RETRY_CAP
	ft_rekey_attempts = 0;		/* per MOVE, not per thread lifetime */
#endif
	ft_txn_op_init(ft, &optxn);
	/*
	 * PARK THE ESCALATION QUIESCENT, which this op may do and insert / remove
	 * may not.  urcu_txn_begin escalates -- and therefore blocks -- before it
	 * opens the txn's own read section, so the only thing that could be pinned
	 * across that park is an RCU section the CALLER holds.  This entry has none:
	 * cds_ft_rekey_graft already forbids being called from a read section (its
	 * move gate waits for a grace period), and the per-attempt pin below is
	 * taken AFTER begin.  A writer parked online is what stops every grace
	 * period in the process, so quiescing here is what keeps a contended move
	 * from wedging the peers that wait on one.
	 */
	urcu_txn_set_park_quiescent(&optxn, 1);
	for (;;) {
#ifdef FT_DEBUG_REKEY_RETRY_CAP
		/*
		 * A LIVELOCK DETECTOR, and it is cheap because of what -EAGAIN
		 * MEANS: the code promises that retrying can help, i.e. that some
		 * PEER is responsible for the refusal.  So an attempt count that
		 * runs away is a self-refusal -- a condition no re-descent can
		 * change -- and the op will spin on it forever.  Three of these
		 * have been found in this writer by hand, each presenting as a
		 * silent hang; this turns the whole class into a loud failure
		 * without needing to know which site is at fault.
		 *
		 * The cap is deliberately far above any real contention: a
		 * saturated multi-writer arm re-attempts single digits, so five
		 * figures cannot be a peer.
		 */
		if (caa_unlikely(++ft_rekey_attempts > FT_REKEY_RETRY_CAP)) {
			fprintf(stderr,
				"FT REKEY LIVELOCK: %u attempts on one move -- "
				"an -EAGAIN no re-descent can clear.  Single-"
				"threaded this is certain; under peers it is "
				"still far past any real contention.  Find the "
				"site by breaking on each -EAGAIN return in "
				"ft_rekey_graft_simple_attempt (full header path "
				"+ `set breakpoint pending on`); a refusal that "
				"is a property of the STRUCTURE owes "
				"FT_REKEY_UNCOVERED, not a retry.\n",
				ft_rekey_attempts);
			abort();
		}
#endif
		urcu_txn_begin(&optxn);
		/*
		 * PER ATTEMPT, not around the loop.  The pin exists to keep the
		 * nodes ONE attempt captures alive from descent through commit, and
		 * each attempt re-descends -- so per-attempt is both sufficient and
		 * what leaves begin's park unpinned.
		 */
		flavor->read_lock();
		ret = ft_rekey_graft_simple_attempt(ft, src_key, src_len,
			dst_key, dst_len, require_empty, &optxn);
		flavor->read_unlock();
		if (ret != -EAGAIN && ret != -EIO)
			break;
		/* Age the conflict, as cds_ft_replace does; the turn is forfeited. */
		ft_txn_attempt_bail(&optxn, true);
	}
	urcu_txn_end(&optxn);
	/*
	 * A COMMITTED move can LENGTHEN every key it carried (@dst_len > @src_len),
	 * so raise the trie's high-water hint the same way ft_rekey_at_inner does
	 * for the staged path and cds_ft_merge_at for the cross-trie one.  Bounded
	 * by the pre-move maximum, which is conservative in the only direction that
	 * matters: the hint may over-state, never under-state.  Read AFTER the
	 * commit and stored unconditionally-if-greater, so a peer that raised it
	 * further in between is not lowered.
	 *
	 * ★ WHY IT IS SAFE TO DO THIS AFTER THE ONE DECIDE rather than inside it:
	 * the field is a HINT -- measured, no lookup / inequality / iteration path
	 * consumes it, only the mutators that maintain it and the public accessor
	 * cds_ft_max_used_key_len -- so a reader between the flip and this store
	 * sees the moved keys and a stale hint, not an inconsistency it can act on.
	 * The keys' own lengths are a property of where the subtree hangs, and that
	 * moved atomically.
	 */
	if (ret == 0 && dst_len > src_len) {
		size_t cur = uatomic_load(&ft->max_used_key_len, CMM_RELAXED);
		size_t nm = cur > src_len ? dst_len + (cur - src_len) : dst_len;

		if (nm > cur)
			uatomic_store(&ft->max_used_key_len, nm, CMM_RELAXED);
	}
	return ret;
}

/*
 * The MOVE entry point: bracket the move in the mode gate, then run it under our
 * OWN read lock.
 *
 * Order matters and is the gate's whole purpose: ft_move_gate_enter publishes
 * "expect a move" to readers and waits a grace period, so every reader still in a
 * critical section has finished before the body below mutates anything -- readers
 * that start after it see the gate and switch to the coherent path.  A burst of
 * concurrent moves pays ~one grace period in total (they piggyback the first).
 *
 * CALLER CONTRACT (new, and inherent to the gate): a move BLOCKS on a grace
 * period, so it must NOT be called from inside an RCU read-side critical section
 * -- the GP would wait for the caller's own section.  This entry takes the read
 * lock the body needs itself, AFTER the gate.  The gate is entered before the
 * shape gates run, so a rejected move also pays the GP; the public entry
 * (ft_rekey_one_decide's caller) cheap-checks the shape first instead.
 *
 * An occupied destination MERGES here (@require_empty false), which is what the
 * oracles driving this entry expect; the graft-semantics refusal belongs to the
 * public entry that documents it.
 */
int _cds_ft_debug_rekey_graft_simple(struct cds_ft *ft,
		const uint8_t *src_key, size_t src_len,
		const uint8_t *dst_key, size_t dst_len)
{
	int ret;

	ft_move_gate_enter(ft);
	ret = ft_rekey_graft_simple_locked(ft, src_key, src_len, dst_key, dst_len,
			false);	/* pins per attempt: see the locked wrapper */
	ft_move_gate_exit(ft);
	return ret;
}

/*
 * The ATOMIC rekey, for the public entry points: move @src_key's subtree to
 * @dst_key as ONE decide, or report that this shape is not one it covers.
 *
 * Everything the caller needs to know is in the return code.  0 committed the
 * move atomically -- a reader sees the subtree at the source XOR the
 * destination, with no instant where it is at neither, which is the property
 * the staged writer (a committed detach, then a merge back) cannot provide.
 * -EINVAL means the shape is outside this writer's cut and the caller should
 * fall back; -EEXIST is a GRAFT caller's occupied destination; -ENOMEM and
 * -ENOTSUP are terminal.  The transient contention codes never surface: the
 * retry wrapper absorbs them.
 *
 * It takes the READ LOCK the body needs but NOT the move gate, so one gate
 * bracket in the caller covers this attempt and any fallback -- two brackets
 * would pay two grace periods for one move.  The caller must therefore already
 * hold the gate and, by the gate's own contract, not be inside a read section.
 *
 * Hidden-visibility here rather than static because the public entries live in
 * ft-merge.h, which is included ABOVE the composition this wraps (it needs
 * detach, graft and merge all in scope); the declaration in
 * fractal-trie-internal.h is what bridges that.
 */
__attribute__((visibility("hidden")))
int ft_rekey_one_decide(struct cds_ft *ft,
		const uint8_t *src_key, size_t src_len,
		const uint8_t *dst_key, size_t dst_len, bool require_empty)
{
	/*
	 * The caller contract this entry documents, now CHECKED: the move gate
	 * its caller holds waits a grace period, and the retry loop below parks
	 * the escalation QUIESCENT -- both are broken by a caller that is inside
	 * a read section.  Exact only for flavors whose read_ongoing() counts
	 * sections; see the macro.
	 */
	CDS_FT_ASSERT_RCU_NOT_READ_LOCKED(ft);
	/*
	 * JOIN THE WRITER-EXCLUSION PROTOCOL THE REST OF THE TRIE USES.
	 *
	 * This writer parks its structural edges SW (ft_flip_txn_set_structural_sw
	 * below), and an SW park CANNOT FAIL -- it does not arbitrate -- so it is
	 * legal only where the op excludes every peer writer over those slots.
	 *
	 * FINE mode: the per-node DLM LOCK try-locks this op takes ARE that
	 * protocol, and CDS_FT_SCOPED_WRITER skips the FT-wide mutex there (the
	 * §11 drop), so this costs nothing and changes nothing.
	 *
	 * COARSE mode: every OTHER writer (insert, remove, graft, merge) excludes
	 * through the FT-WIDE MUTEX this macro takes.  Without joining it, this op
	 * shared NO exclusion protocol with its peers -- its per-node locks
	 * arbitrate against nobody, because in coarse mode nobody else takes them
	 * -- and its unfailable SW parks would be lost updates against a
	 * concurrent insert or remove.  That is what the
	 * assert(ft->lock_fine && ...) in ft_rekey_cow_stop was protecting: not
	 * the mechanism, but the fact that only FINE mode had a protocol this
	 * writer was already speaking.
	 *
	 * ORDER matches the other writers -- FT-wide mutex first, escalation
	 * domain second (urcu_txn_begin, inside the retry loop below) -- so the
	 * two-lock cycle those ops could otherwise form does not arise.  Taken
	 * AFTER the caller's ft_move_gate_enter so its grace period is not waited
	 * for while holding the mutex.
	 */
	/*
	 * NO read lock here: ft_rekey_graft_simple_locked pins PER ATTEMPT, which
	 * is what lets its escalation park quiesce.  Wrapping the loop instead --
	 * which this did -- pinned the park and made a contended move able to wedge
	 * every peer waiting on a grace period.
	 */
	return ft_rekey_graft_simple_locked(ft, src_key, src_len, dst_key, dst_len,
			require_empty);
}

/* ------------- moved from ft-merge.h ------------- */

/*
 * MERGE-ONLY.  The spine copy UNIONS the source subtree into an OCCUPIED
 * destination, which is a merge by definition and is built out of the merge
 * subsystem's own machinery (ft_merge_ctx, ft_merge_build,
 * ft_merge_ord_interleave_collect).  With -DNO_FEATURE_FT_MERGE only the rekey
 * GRAFT -- the empty-dst shape -- remains available, so this whole arm compiles
 * out and its caller answers CDS_FT_STATUS_NOT_SUPPORTED, exactly as
 * ft_rekey_graft_simple_attempt already does for the same shape.
 */
#ifdef FEATURE_FT_MERGE
static
enum cds_ft_status ft_rekey_spine_copy(struct cds_ft *dst_ft,
		struct cds_ft *src_ft, struct ft_descent *d_src,
		const uint8_t *src_key, size_t src_key_len, unsigned long cnt_src,
		unsigned int off_src, struct ft_descent *d_dst,
		unsigned long cnt_dst, unsigned int off_dst,
		size_t dst_key_len,
		struct ft_flip_txn **pre_txn, bool *contended)
{
	struct ft_glue gd, gs;
	struct ft_merge_ctx ctx = { .dst_ft = dst_ft, .gd = &gd, .gs = &gs };
	struct ft_merge_counts cnt = { 0, 0, 0, 0, 0 };
	bool root_src = (src_key_len == 0);
	bool ks_dst = (off_dst > 0);
	bool ed;
	struct cds_ft_inode_flag *S = d_src->nf;
	struct cds_ft_inode_flag *D = d_dst->nf;
	struct cds_ft_inode_flag *M, *M_slot = NULL, *pub, *D_old;
	struct cds_ft_inode_flag *pub_parent = d_dst->pnf, **pub_slot = d_dst->nfp;
	struct cds_ft_inode *fresh_root = NULL;
	struct cds_ft_metadata *fresh_meta;
	struct ft_flip_txn *txn;
	unsigned long merged_keys = 0;
	bool ms_ord = dst_ft->group->ordered_list_set;
	struct ft_ord_cell *ms_cursor = NULL, *ms_prev = NULL, *ms_succ = NULL;
	struct cds_ft_node *ms_s_first = NULL, *ms_s_last = NULL;
	struct ft_ord_cell_edge *ms_edges = NULL;
	struct ft_merge_src_cap *ms_src_caps = NULL;	/* src survivor suffixes, pre-captured */
	uint8_t *ms_src_pool = NULL;			/* packed suffix byte pool */
	unsigned long ms_nsrc = 0;			/* src run cells captured */
	unsigned int ms_cap = 0;	/* interleave edge cap, set once merged_keys is known */
	unsigned int ms_n = 0;		/* interleave edges collected (staged pre-commit) */
	/*
	 * The caller pre-reserved this op's flip-txn, i.e. it is already PAST its
	 * own point of no return and pre-built everything so this placement cannot
	 * fail.  Gates the dup-chain acquire below -- the only step here that can
	 * decline.
	 */
	bool unfailable = (pre_txn && *pre_txn);

	*contended = false;

	/*
	 * Every dst merge-point shape is handled.  The flip proxies the publish
	 * slot @pub_slot, the read-side descent resolves the type-7 proxy at every
	 * child fetch (ft_resolve_flip_proxy, before the skip handler), and the
	 * published node's parent is wired to @pub_parent by
	 * ft_glue_set_publish, so a descent and an up-walk see a coherent
	 * old-XOR-merged view across the flip.
	 *
	 * Edge D: the merge point's PARENT is a COMPRESSED node cn_p reached via a
	 * grandparent skip slot.  cn_p's own parent cannot be compressed ("no two
	 * adjacent compressed" invariant), so the grandparent slot d_dst->pnfp is a
	 * plain internal slot that merely *holds* skip(cn_p).  Handle it one level
	 * up: the merge is EXACT at d_dst->nf (off_dst == 0), M is wrapped under a
	 * fresh copy of the WHOLE cn_p, and that copy is published into d_dst->pnfp
	 * in place of skip(cn_p) -- identical machinery to a KEY_SHORTER dst wrap,
	 * just with cn_p as the wrapped node and the grandparent as the publish
	 * point.  ks_dst and ed are mutually exclusive (a KEY_SHORTER merge point
	 * sits inside cn_d, whose parent is internal).
	 */
	ed = (d_dst->pnf &&
	      ft_node_compressed(ft_resolve_skip_compressed(dst_ft, d_dst->pnf)));

	/* Size both glues from a read-only pre-pass (with headroom). */
	ft_merge_count(dst_ft, S, off_src, D, off_dst, &cnt);
	ft_glue_init(&gd);
	ft_glue_init(&gs);
	/*
	 * The dst glue's own acquires -- @pub_parent, above all -- fire from
	 * commit helpers that never see a descent, so hand them the destination
	 * one here, the single place holding both (the graft does the same at its
	 * ft_glue_set_publish).  @pub_parent is @d_dst's own parent or
	 * grandparent, so the window dates it; without this the acquire has no
	 * depth under a coarse spacing and its miss aborts a commit the unfailable
	 * arm cannot retry -- which is a LIVELOCK, not a failure.
	 */
	gd.lock_d = d_dst;
	/*
	 * And the SOURCE glue's, for the same reason: a src overlap fence is
	 * dated on the path the node is on NOW, which only @d_src describes.
	 * Inert for a cross-trie merge (@fence_src stays false -- the source is
	 * exclusive), set so the two glues answer from the same rule.
	 */
	gs.lock_d = d_src;
	/*
	 * Where relative depth 0 IS, on each side (see @dst_base_depth).
	 */
	ctx.dst_base_depth = (unsigned int) d_dst->depth + off_dst;
	ctx.src_base_depth = (unsigned int) d_src->depth + off_src;
	if (ft_glue_reserve(&gd, cnt.nb + 8, cnt.nd + 8,
				cnt.nf_dst + 8, cnt.ns + 8) ||
	    ft_glue_reserve(&gs, 0, 0, cnt.nf_src + 8, 0)) {
		ft_glue_fini(&gd);
		ft_glue_fini(&gs);
		return CDS_FT_STATUS_MEMORY_ERROR;
	}

	/* Root src: pre-allocate the fresh empty root for the commit swap. */
	if (root_src) {
		fresh_root = alloc_cds_ft_node(src_ft, &ft_types[0], &fresh_meta);
		if (!fresh_root) {
			ft_glue_fini(&gd);
			ft_glue_fini(&gs);
			return CDS_FT_STATUS_MEMORY_ERROR;
		}
		fresh_meta->parent_word = ft_trie_parent(src_ft);
		ft_nr_keys_store(src_ft, fresh_meta, 0, CMM_RELAXED);
	}

	/* Build the merged cluster invisibly (the only build-phase fallible step). */
	/*
	 * DLM overlap-spine plan-lock (§9.4 M-2): fence each dst overlap node before
	 * the build reads it, and retire it through the fenced terminal, so a peer
	 * growing a node this merge is copying cannot be silently retired with it.
	 * Skipped for the unfailable caller, which has no bail left to take.
	 */
	ctx.fence_overlap = dst_ft->lock_fine && !unfailable;
	ctx.fence_src = false;		/* cross-trie: the source is exclusive */
	M = ft_merge_build(&ctx, S, off_src, D, off_dst, 0, &merged_keys);
	if (M == FT_MERGE_OOM) {
		if (fresh_root)
			free_cds_ft_node_unpublished(src_ft, fresh_root);
		ft_glue_abort(dst_ft, &gd);
		ft_glue_abort(src_ft, &gs);
		/*
		 * A missed overlap fence shares FT_MERGE_OOM's unwind but is
		 * CONTENTION, not memory: both tries are pristine (the build published
		 * nothing and ft_glue_abort released every fence it took), so report it
		 * on the retry channel and let the caller re-descend.
		 */
		if (ctx.overlap_contended)
			*contended = true;
		return CDS_FT_STATUS_MEMORY_ERROR;
	}
	/*
	 * The ordered-list interleave is folded into the structural flip (one
	 * commit for structure + ordered list), so its <= 2*merged_keys+2 cell
	 * edges share the structural flip batch.  Each survivor run costs at most
	 * two visible edges; bound it now that @merged_keys is known.
	 */
	if (ms_ord)
		ms_cap = 2u * (unsigned int) merged_keys + 2u;

	/*
	 * Compute @pub (the value to publish), @pub_parent / @pub_slot (where) and,
	 * for the compressed shapes, reclaim the replaced node.
	 *
	 * KEY_SHORTER dst (off_dst > 0): the merge point sits off_dst bytes inside
	 * the compressed node cn_d = D, whose prefix bytes lie above it; wrap M
	 * under cn_d->key_bytes[0..off_dst) and replace cn_d at its own slot
	 * (d_dst->nfp).  Edge D (ed): the merge point's parent is the compressed
	 * node cn_p; wrap M under the WHOLE cn_p and replace cn_p at the grandparent
	 * slot d_dst->pnfp (pub_parent = d_dst->ppnf).  Either way reclaim the
	 * replaced compressed node -- ft_merge_build entered it (or, for Edge D, did
	 * not touch it) without recording the free.  ft_merge_wrap_prefix
	 * canonicalizes a single-child M and preserves the dst_origin of a
	 * compressed M's child.
	 *
	 * EXACT dst with an internal/absent parent (off_dst == 0, !ed): M itself
	 * replaces the subtree.  Canonicalize a non-root single-child internal merge
	 * top: when S and D contribute exactly one shared byte (a churned "ab"-prefix
	 * shape), ft_merge_build returns M as a 1-child internal with no
	 * external_nodes -- forbidden at a non-root position under skip mode
	 * (chain-compress invariant; cds_ft_verify catches it at the merge depth).
	 * Collapse it to a 1-byte compressed via the glue.  Only the TOP M can hit
	 * this; its lone child is always a FRESH recursion result, so the deferred
	 * child edge carries dst_origin=false correctly and disturbs no flip edge.
	 * A root dst merge point (d_dst->pnf == NULL) is exempt: a 1-child internal
	 * is canonical at the root.
	 */
	if (ks_dst || ed) {
		struct cds_ft_compressed_node *wrap_cn;
		uint8_t kbuf[FT_MAX_KEY_LEN];
		unsigned int wrap_depth, wrap_len;

		if (ed) {
			wrap_cn = ft_compressed_node_ptr(
				ft_resolve_skip_compressed(dst_ft, d_dst->pnf));
			wrap_len = wrap_cn->len;
			wrap_depth = (unsigned int) d_dst->depth - wrap_len;
			pub_parent = d_dst->ppnf;
			pub_slot = d_dst->pnfp;
		} else {	/* ks_dst */
			wrap_cn = ft_compressed_node_ptr(D);
			wrap_len = off_dst;
			wrap_depth = (unsigned int) d_dst->depth;
		}
		ft_glue_defer_free(&gd, wrap_cn, true);
		memcpy(&kbuf[wrap_depth], wrap_cn->key_bytes, wrap_len);
		pub = ft_merge_wrap_prefix(&ctx, kbuf, wrap_depth,
				wrap_depth + wrap_len, M, merged_keys);
		if (pub == FT_MERGE_OOM) {
			if (fresh_root)
				free_cds_ft_node_unpublished(src_ft, fresh_root);
			ft_glue_abort(dst_ft, &gd);
			ft_glue_abort(src_ft, &gs);
			return CDS_FT_STATUS_MEMORY_ERROR;
		}
	} else if (pub_parent) {
		pub = ft_compress_single_child_if_needed(dst_ft, M, &gd);
		if (pub == (struct cds_ft_inode_flag *) (long) -ENOMEM) {
			if (fresh_root)
				free_cds_ft_node_unpublished(src_ft, fresh_root);
			ft_glue_abort(dst_ft, &gd);
			ft_glue_abort(src_ft, &gs);
			return CDS_FT_STATUS_MEMORY_ERROR;
		}
	} else {
		pub = M;
	}

	/*
	 * Take or create the flip-txn: one latch per dst-origin re-parent edge,
	 * one for the merge-point forward slot, @ms_cap for the ordered-list
	 * interleave's boundary edges, and one per collided duplicate-chain splice
	 * (the src run tail-append, folded in below so the concatenation flips with
	 * the structure) -- structure, ordered list AND duplicate chains commit in
	 * ONE flip, so they share this txn.  Reserve it up front to that bound so
	 * every post-drain record (the structural edges, the INSTALLED-state cell
	 * edges and the splice edges, which append into the reserved head chunk) is
	 * allocation-free.
	 * The rekey hands in a pre-reserved txn (cannot fail); otherwise create one
	 * here, where failure aborts the still-invisible build (both tries pristine).
	 */
	{
		unsigned int nr_dst = 0;
		int j;

		for (j = 0; j < gd.nr_deferred; j++)
			if (gd.deferred[j].dst_origin)
				nr_dst++;
		txn = ft_flip_txn_take(pre_txn);
		if (!txn) {
			txn = ft_flip_txn_create(dst_ft);
			if (txn && !ft_flip_txn_reserve(txn,
					nr_dst + 1 + ms_cap + gd.cap_free
						+ gd.nr_splices
						+ 1 /* §4.B parent guard */
						/* + count walk: the (merged_keys - cnt_dst) nr_keys ancestor
						 * edges (BULK fold), bounded by the merge-point depth */
						+ (dst_ft->rank_stats ? (int) dst_key_len + 1 : 0)
				/* the split-retire terminal's second word, if any */
				+ ft_glue_split_cn_reserve(dst_ft))) {
				ft_flip_txn_destroy(txn);
				txn = NULL;
			} else if (txn) {
				/*
				 * Created + reserved gd.cap_free free-list headroom:
				 * fuse each dst-overlap retire's freeze into @txn (atomic
				 * detach, doc/design/mcas-multiwriter-readiness.md §4.B),
				 * so the whole retired dst spine freezes dead atomically
				 * with the forward publish that unlinks it.  The rekey
				 * take() path keeps the standalone flip -- its pre_txn is
				 * pre-sized by the rekey with no room to grow here.
				 */
				gd.fuse_free_list = true;
			}
		}
	}
	if (!txn) {
		if (fresh_root)
			free_cds_ft_node_unpublished(src_ft, fresh_root);
		ft_glue_abort(dst_ft, &gd);
		ft_glue_abort(src_ft, &gs);
		return CDS_FT_STATUS_MEMORY_ERROR;
	}
	ft_glue_set_publish(dst_ft, &gd, pub_parent, pub_slot, pub);
	/*
	 * The dst forward publish commits through @txn (step 4 below); point
	 * gd->txn at it so ft_glue_apply_deferred's tombstone_free_list records
	 * the fused dst retires into the same txn.  A no-op alias when
	 * fuse_free_list stayed false (the rekey take() path).
	 */
	gd.txn = txn;

	/*
	 * Slot-canonical form of @pub for the @pub_slot stores (both the flip
	 * proxy's new target and the settle): a compressed @pub is published
	 * SKIP-ENCODED, exactly as a direct slot write would store it, so a reader
	 * resolving the proxy gets a value identical in encoding to a normal slot
	 * read and runs the same skip handling.  set_publish wired @pub's parent +
	 * skip_slot via the plain flag already; an internal @pub (EXACT only -- a
	 * compressed-wrap @pub is always compressed) needs no re-encode.
	 *
	 * @D_old is the flip proxy's old target -- the value @pub_slot currently
	 * holds.  Read it from the live slot: for the compressed shapes the descent
	 * resolved d->nf / d->pnf to the PLAIN flag while the slot holds the SKIP
	 * form, and *pub_slot is untouched until the flip (the build is invisible
	 * and apply_deferred wires back-pointers, not this forward slot).
	 */
	if (ft_node_compressed(pub))
		M_slot = ft_publish_compressed(dst_ft,
				ft_compressed_node_ptr(pub), pub);
	else
		M_slot = pub;
	/*
	 * WAITING load, not a raw one: @pub_slot is this flip's forward edge, so
	 * it enters @txn's own write set.  "Untouched until the flip" is true of
	 * THIS op's stores and says nothing about a PEER parking a flip proxy
	 * there; that proxy is a descriptor-record POINTER, and as an
	 * expected-old it trips urcu_txn_add's !urcu_txn_is_proxy self-check.
	 */
	D_old = urcu_txn_load(txn->mtxn, (void **) pub_slot,
		FT_FLIP_PROXY_TAG);

	/*
	 * Ordered list: capture the dst merge subtree's min head (the cursor for
	 * the post-commit interleave walk) and the region predecessor, while D is
	 * still intact (the build only copied its spine; the flip below moves its
	 * leaves into M).  The surviving src cells are spliced in after the commit.
	 */
	if (ms_ord) {
		ms_cursor = ft_ord_cell_ptr(rcu_dereference(
			ft_subtree_minmax_head(dst_ft, D, false)->prev));
		ms_prev = ft_ord_cell_resolve_ord(&ms_cursor->lnode.prev);
		/*
		 * The dst region run is [@ms_cursor .. D's max head]; @ms_succ is
		 * the cell following it (NULL at the list tail), the stop boundary
		 * for the interleave walk and the trailing survivor's successor.
		 */
		ms_succ = ft_ord_cell_resolve_ord(&ft_ord_cell_ptr(rcu_dereference(
			ft_subtree_minmax_head(dst_ft, D, true)->prev))->lnode.next);
		/*
		 * Capture src's merged-subtree (S) run endpoints now, while S is
		 * still intact, but defer the actual run-unlink until AFTER the
		 * last fallible step (ft_merge_unlink_src_subtree).  The run
		 * unlink is an externally observable ordered-list mutation; doing
		 * it here would leave src's list inconsistent if the src subtree
		 * unlink below OOMs and we roll the whole merge back.  The unlink
		 * still happens before the drain, so sync drains src ord-readers
		 * of the run too.
		 */
		ms_s_first = ft_subtree_minmax_head(dst_ft, S, false);
		ms_s_last = ft_subtree_minmax_head(dst_ft, S, true);
		/*
		 * Pre-allocate the interleave's edge scratch NOW, while the build
		 * is still abortable.  The interleave is collected pre-commit and
		 * its proxies recorded into the (already-reserved) structural @txn, so
		 * the commit tail has no allocation left and cannot degrade to a
		 * non-atomic per-edge fallback.  Each survivor run costs at most
		 * two visible edges, so @ms_cap (2*merged_keys+2) bounds @ms_edges.
		 */
		ms_edges = malloc((size_t) ms_cap * sizeof(*ms_edges));
		if (!ms_edges) {
			ft_flip_txn_destroy(txn);
			if (fresh_root)
				free_cds_ft_node_unpublished(src_ft, fresh_root);
			ft_glue_abort(dst_ft, &gd);
			ft_glue_abort(src_ft, &gs);
			return CDS_FT_STATUS_MEMORY_ERROR;
		}
		/*
		 * Capture the src run's key SUFFIXES (below the src merge point)
		 * NOW, while S is still attached and up-walkable.  The post-commit
		 * interleave merges them against the LIVE dst suffixes to rebuild
		 * the merged order WITHOUT walking the about-to-be-published merged
		 * structure -- so no proxy need be installed before the collect.
		 * Capturing them here (not at the collect) is mandatory:
		 * ft_merge_unlink_src_subtree below leaves S-root's parent stale,
		 * so a src up-walk after the unlink could run into freed / relocated
		 * structure.  The run length is bounded by @cnt_src (the src
		 * subtree's key count >= its distinct heads); the variable-length
		 * suffix bytes pack into a realloc-growable pool addressed by offset
		 * (a byte pointer would dangle across the realloc).  Fallible
		 * pre-pass before the last fallible step -> any OOM aborts the
		 * still-invisible build, both tries pristine.
		 */
		ms_src_caps = malloc((size_t) (cnt_src + 8) * sizeof(*ms_src_caps));
		if (ms_src_caps) {
			struct ft_ord_cell *sc = ft_ord_cell_ptr(
				rcu_dereference(ms_s_first->prev));
			struct ft_ord_cell *slast = ft_ord_cell_ptr(
				rcu_dereference(ms_s_last->prev));
			size_t s_max_len = dst_ft->group->max_key_len;
			size_t pool_cap = 0, pool_len = 0;
			uint8_t sbuf[FT_MAX_KEY_LEN];
			bool oom = false;

			for (;;) {
				size_t sfl = ft_rebuild_key_upwalk(dst_ft, sc,
						sbuf, s_max_len);
				size_t suf_len;

				assert(sfl >= src_key_len &&
					ms_nsrc < cnt_src + 8);
				suf_len = sfl - src_key_len;
				if (pool_len + suf_len > pool_cap) {
					size_t ncap = pool_cap ? pool_cap * 2 : 256;
					uint8_t *np;

					while (ncap < pool_len + suf_len)
						ncap *= 2;
					np = realloc(ms_src_pool, ncap);
					if (!np) {
						oom = true;
						break;
					}
					ms_src_pool = np;
					pool_cap = ncap;
				}
				memcpy(ms_src_pool + pool_len,
					sbuf + (s_max_len - sfl) + src_key_len,
					suf_len);
				ms_src_caps[ms_nsrc].cell = sc;
				ms_src_caps[ms_nsrc].suffix_off = pool_len;
				ms_src_caps[ms_nsrc].suffix_len = suf_len;
				pool_len += suf_len;
				ms_nsrc++;
				if (sc == slast)
					break;
				sc = ft_ord_cell_resolve_ord(&sc->lnode.next);
			}
			if (oom) {
				free(ms_src_pool);
				ms_src_pool = NULL;
				free(ms_src_caps);
				ms_src_caps = NULL;
			}
		}
		if (!ms_src_caps) {
			free(ms_edges);
			ft_flip_txn_destroy(txn);
			if (fresh_root)
				free_cds_ft_node_unpublished(src_ft, fresh_root);
			ft_glue_abort(dst_ft, &gd);
			ft_glue_abort(src_ft, &gs);
			return CDS_FT_STATUS_MEMORY_ERROR;
		}
	}

	/*
	 * 1. Unlink the merge source from src.  Root src: swap in the pre-
	 *    allocated empty root.  Non-root src: detach its branch in place --
	 *    the LAST fallible step.  On its OOM both tries are pristine (the
	 *    detach self-undoes, the cluster is still unpublished), so abort the
	 *    build; this preserves the no-rollback property.  Then drain src
	 *    readers of the moved content.
	 *
	 *    ===== Everything from the drain onward is failure-free. =====
	 */
	struct ft_detach_run sdrun = { .into = NULL };

	if (ms_ord && !root_src) {
		/*
		 * Non-root src: fuse the src run-unlink into ft_merge_unlink_src_
		 * subtree's structural unlink flip (EXCISE-ONLY, into == NULL -- the
		 * run cells disperse to dst via the post-commit interleave), exactly
		 * as the subpos src side does, closing the src-side disappear window.
		 */
		sdrun.rfirst = ms_s_first;
		sdrun.rlast = ms_s_last;
	}
	/*
	 * Pre-reserve the src-side ordered-list commit txn before the (still
	 * fallible) src unlink.  Whichever fires -- the root-src root+endpoint
	 * swap or the rare unfused non-root run-unlink -- is the src side's
	 * commit, un-abortable once the structural unlink is public, so it commits
	 * through this pre-reserved txn (ft_ord_cell_flip_into).  Sized to the
	 * larger bound (4-edge run-unlink >= 3-edge root swap).  OOM here aborts
	 * the still-invisible build (both tries pristine).  The lone-edge list-off
	 * paths (ft_root_edge_flip) need no txn, so reserve only when ms_ord.
	 *
	 * ☠ WHY THE SRC SIDE IS TRANSACTED AT ALL, given cds_ft_merge_at and
	 * cds_ft_graft both REJECT a non-exclusive src (BUSY_ERROR), and an
	 * exclusive trie has no concurrent reader or writer to be atomic against:
	 *
	 * because that rejection is `src_ft != dst_ft` -- SAME-TRIE is excluded
	 * from it on purpose.  cds_ft_rekey_{graft,merge} reach this worker
	 * through ft_rekey_at_inner(ft, ..., ft, ...) with @rekey set, so on that
	 * path @src_ft IS @dst_ft: the LIVE, SHARED, concurrently-read trie.  Its
	 * "src side" is therefore exactly as contended as its dst side, and every
	 * edge here needs the same atomicity a cross-trie merge's src does not.
	 *
	 * So "the src is always exclusive now, drop the src-side txn" is a sound
	 * reading of the entry gates and a WRONG conclusion about this body.  The
	 * exclusivity that would justify it belongs to the CROSS-TRIE callers
	 * only; this code is shared with the one caller that has none.
	 */
	struct ft_flip_txn *src_side_txn = NULL;

	/*
	 * Size the src-side commit txn per shape.  A root src fuses the gs
	 * free-list freeze into its root swap / lone root flip (atomic detach,
	 * doc §4.B): + gs.cap_free tombstone edges (cnt.nf_src + 8, the exact
	 * upper bound ft_glue_defer_free asserts against).  A non-root src
	 * commits its run-unlink through a plain FT_ORD_CELL_RUN_UNLINK txn; its
	 * gs freeze stays standalone, applied below AFTER
	 * ft_merge_unlink_src_subtree's (still fallible) unlink so a rolled-back
	 * merge never freezes-then-strands (full non-root fusion -- into
	 * ft_detach_node's flip -- is a §4.B residual).  A list-off root with an
	 * EMPTY gs keeps its bare lone-edge store (no txn, no grace period).
	 * gs is fully populated by ft_merge_build above, so nr_free/cap_free are
	 * final here.  (gd is stamped in ft_glue_apply_deferred for the dst
	 * forward publish.)
	 */
	if (ms_ord) {
		src_side_txn = ft_flip_txn_create_bounded(src_ft, root_src ?
			FT_ROOT_LIST_SWAP_MAX_EDGES + (unsigned int) gs.cap_free :
			FT_ORD_CELL_RUN_UNLINK_MAX_EDGES);
		if (!src_side_txn) {
			free(ms_src_pool);
			free(ms_src_caps);
			free(ms_edges);
			ft_flip_txn_destroy(txn);
			if (fresh_root)
				free_cds_ft_node_unpublished(src_ft, fresh_root);
			ft_glue_abort(dst_ft, &gd);
			ft_glue_abort(src_ft, &gs);
			return CDS_FT_STATUS_MEMORY_ERROR;
		}
	} else if (root_src && gs.nr_free) {
		/* List-off root with retires: 1 root edge + gs.cap_free tombstones. */
		src_side_txn = ft_flip_txn_create_bounded(src_ft,
			1u + (unsigned int) gs.cap_free);
		if (!src_side_txn) {
			free(ms_src_pool);
			free(ms_src_caps);
			free(ms_edges);
			ft_flip_txn_destroy(txn);
			if (fresh_root)
				free_cds_ft_node_unpublished(src_ft, fresh_root);
			ft_glue_abort(dst_ft, &gd);
			ft_glue_abort(src_ft, &gs);
			return CDS_FT_STATUS_MEMORY_ERROR;
		}
	}

	/*
	 * Dup-chain lock-set (MW LOCK_FINE): take the node lock of every
	 * distinct holder whose chain step 3c appends a collided src run to, so
	 * the tail walk + append run under the same per-node lock insert, remove,
	 * promote and replace take on that chain.  HERE is the last point where a
	 * miss is free: every fallible allocation above has succeeded and the src
	 * unlink below is the point of no return, so an -EAGAIN unwinds a build
	 * that is still entirely invisible.  The locks are held across the unlink,
	 * the drain and the single commit that installs the appends, and released
	 * just after it.
	 *
	 * UNDOING THE ATTEMPT IS TRIVIAL, which is what makes the bail cheap:
	 * cds_ft_merge_at consumes an EXCLUSIVE source, so nothing is published and
	 * nothing has moved -- discarding the fresh cluster leaves both tries
	 * byte-for-byte as they were, and the caller just re-descends.  That is why
	 * contention needs no place in enum cds_ft_status (see @contended).
	 *
	 * SKIPPED when the CALLER pre-reserved @pre_txn.  That marks a caller which
	 * has ALREADY passed its own point of no return and pre-built everything so
	 * this placement cannot fail -- today the staged rekey (detach -> GP ->
	 * merge back), whose content is already out of the trie and has nowhere to
	 * go if we bail.  A failable acquire under an unfailable placement is a
	 * contradiction, and retrying there would re-draw a node reserve that a
	 * discarded attempt returns to the ARENA, not to the reserve.  Same-trie
	 * rekey gets its chain exclusion from the in-place one-decide writer's
	 * up-front DLM lock set instead -- a single commit with reader two-pass
	 * coherence, no detach and so no unfailable tail -- not from a retrofit
	 * here.  So this leaves the staged rekey's appends where they are today.
	 *
	 * No-op on a non-lock_fine trie or a collision-free merge, which is every
	 * merge of disjoint key sets -- the batch-staging workload pays nothing.
	 */
	if (!unfailable && ft_glue_acquire_splice_holders(dst_ft, &gd)) {
		free(ms_src_pool);
		free(ms_src_caps);
		free(ms_edges);
		ft_flip_txn_destroy(txn);
		if (src_side_txn)
			ft_flip_txn_destroy(src_side_txn);
		if (fresh_root)
			free_cds_ft_node_unpublished(src_ft, fresh_root);
		ft_glue_abort(dst_ft, &gd);
		ft_glue_abort(src_ft, &gs);
		*contended = true;
		return CDS_FT_STATUS_MEMORY_ERROR;
	}

	/*
	 * PRE-ACQUIRE THE PUBLISH TARGET, here, for the same reason: this is the
	 * last point at which failing to get it is free.
	 *
	 * The forward publish used to acquire @pub_parent at step 3 -- AFTER the src
	 * unlink -- through ft_flip_txn_lock_or_guard_parent, whose miss is not a
	 * failure it can report: it sets @acquire_miss, and the commit then ABORTS.
	 * By then the src subtree is unlinked and the merged cluster is unpublished,
	 * so the keys exist in NEITHER trie, and the ignored commit status let this
	 * function return CDS_FT_STATUS_OK on top of it.  Silent data loss with a
	 * success code, measured across most merge shapes.
	 *
	 * @pub_parent is above the overlap spine, so it is never in the fenced set;
	 * it CAN be a dup-chain holder we just locked (an external dst merge point),
	 * so take that fence over rather than re-marking our own word -- the
	 * self-deadlock this file has now hit twice.  Either way the holder rides
	 * @gd to the publish, which records the {LOCK|s -> s} release directly
	 * and so never consults lock_or_guard.  With no acquire left to miss, that
	 * abort cause is gone rather than merely less likely.
	 *
	 * A miss here bails exactly like the splice acquire above: both tries
	 * pristine, contention reported, caller re-descends.
	 */
	if (!unfailable && dst_ft->lock_fine && pub_parent) {
		struct cds_ft_metadata *pm = ft_flag_to_metadata(dst_ft, pub_parent);
		uintptr_t psnap = 0;

		struct ft_lock_ctx pctx;
		struct ft_held_anchor ph;
		unsigned int pdep;
		bool took = ft_glue_splice_holder_take(&gd, pm, &psnap);

		ft_glue_lock_ctx(&gd, &pctx);
		if (!took && (!ft_lock_ctx_depth_of(dst_ft, &pctx, pub_parent,
					&pdep) ||
				ft_acquire_member(dst_ft, &pctx, pub_parent, pm,
					pdep, &ph))) {
			free(ms_src_pool);
			free(ms_src_caps);
			free(ms_edges);
			ft_flip_txn_destroy(txn);
			if (src_side_txn)
				ft_flip_txn_destroy(src_side_txn);
			if (fresh_root)
				free_cds_ft_node_unpublished(src_ft, fresh_root);
			ft_glue_abort(dst_ft, &gd);
			ft_glue_abort(src_ft, &gs);
			*contended = true;
			return CDS_FT_STATUS_MEMORY_ERROR;
		}
		/*
		 * The holder is whichever word protects @pub_parent: the splice
		 * fence this op already took over, or the anchor just acquired.
		 */
		/*
		 * A SHARED acquire owes no release.  The claim above -- that
		 * @pub_parent is above the overlap spine and so never in the
		 * fenced set -- holds for the NODE and not for its ANCHOR:
		 * coarsening can put that anchor on an overlap node this op has
		 * already fenced, and root-only puts EVERY member on one word.
		 * Claiming it here would record a second terminal on a word whose
		 * fenced retire already owns one, which the engine poisons.
		 */
		if (!took && ph.shared) {
			gd.publish_parent_holder = NULL;
			gd.publish_parent_snap = 0;
		} else {
			gd.publish_parent_holder = took ? pm : ph.lock;
			gd.publish_parent_snap = took ? psnap : ph.lock_snap;
		}
	}

	if (root_src) {
		/*
		 * A root src moves the WHOLE source, so its run is the whole src
		 * ordered list: fuse the src->root swap with the head/tail clear into
		 * ONE flip (ft_root_list_swap_publish), so a src reader never sees src
		 * structurally empty but its ordered-list front still populated.  The
		 * run cells keep their internal links (only the head/tail endpoints
		 * flip), so the post-commit interleave still re-homes them to dst.
		 */
		if (ms_ord) {
			/*
			 * Empty src's sentinel (relink_dest NULL): the run cells are
			 * re-homed into dst by the post-commit interleave, which sets each
			 * survivor's links individually (collided heads are freed).
			 *
			 * Fuse the gs free-list freeze into this root-swap txn (atomic
			 * detach, doc §4.B): record each retired src-overlap node's
			 * tombstone into src_side_txn first, then the swap records the
			 * root + sentinel endpoint edges into the same txn and commits
			 * them all in ONE flip.  An empty gs records nothing.
			 */
			gs.txn = src_side_txn;
			gs.fuse_free_list = true;
			ft_glue_tombstone_free_list(&gs);
			ft_root_list_swap_publish(src_ft, src_side_txn,
				&src_ft->root,
				src_ft->root, ft_node_flag(fresh_root, 0),
				ft_ord_first(src_ft), NULL,
				ft_ord_last(src_ft), NULL, NULL, false);
			src_side_txn = NULL;	/* consumed */
		} else if (gs.nr_free) {
			/*
			 * List-off root with retires: fuse the gs free-list freeze into
			 * the root flip (atomic detach, doc §4.B).  Record the lone root
			 * edge + each retired node's tombstone into src_side_txn and
			 * commit them in ONE flip (a group flip, like the list-on path),
			 * rather than a bare store followed by standalone freezes.  The
			 * root edge normalizes to the same FT_FLIP_PROXY_TAG the swap uses.
			 */
			ft_flip_txn_record_root(src_side_txn,
				(void **) &src_ft->root,
				(void *) src_ft->root,
				(void *) ft_node_flag(fresh_root, 0));
			gs.txn = src_side_txn;
			gs.fuse_free_list = true;
			ft_glue_tombstone_free_list(&gs);
			ft_flip_txn_commit(src_ft, src_side_txn);
			src_side_txn = NULL;	/* consumed */
		} else {
			/*
			 * No ordered list, no retires: src->root is the only reader-
			 * visible slot.  Express the lone root edge as a single-edge flip
			 * descriptor (one release store, like a bare rcu_assign_pointer)
			 * so the swap is MCAS-expressible like the list-on path.
			 */
			ft_root_edge_flip(src_ft, &src_ft->root,
				src_ft->root, ft_node_flag(fresh_root, 0));
		}
		FT_TP(root_publish, (const void *) src_ft, (const void *) src_ft->root);
	} else if (ft_merge_unlink_src_subtree(src_ft, src_key, src_key_len,
				cnt_src, ms_ord ? &sdrun : NULL, &gs) < 0) {
		/*
		 * OOM in the last fallible step: @src_ft is left pristine (the run was
		 * not yet applied -- it commits in ft_detach_node's flip, past the
		 * fallible alloc), so abort the still-invisible build.
		 */
		free(ms_src_pool);
		free(ms_src_caps);
		free(ms_edges);
		ft_flip_txn_destroy(txn);
		if (src_side_txn)
			ft_flip_txn_destroy(src_side_txn);
		ft_glue_abort(dst_ft, &gd);
		ft_glue_abort(src_ft, &gs);
		return CDS_FT_STATUS_MEMORY_ERROR;
	}
	/*
	 * Non-root src: gs (the src overlap-spine free-list) is now frozen dead
	 * ATOMICALLY with the unlink -- ft_merge_unlink_src_subtree threaded gs
	 * into ft_detach_node as its @retire_glue, so each retired node's
	 * tombstone rode the SAME commit_txn flip that unlinked S (doc §4.B
	 * atomic detach).  On the OOM abort above ft_detach_node left gs unmarked
	 * (src stays pristine, gs still live).  A root src froze gs fused into its
	 * root swap / flip above.
	 */
	/*
	 * Now that the last fallible step has committed, remove src's merged
	 * subtree (S) run from src's ordered list: its cells disperse to dst
	 * (survivors) or are freed (collisions).  Deferred to here so an OOM in
	 * the src unlink above leaves src's list untouched on rollback; done
	 * before the drain so sync drains src ord-readers of the run too.  Skipped
	 * for a root src (fused into the root swap above) and for a non-root src
	 * whose unlink already fused the run (sdrun.armed).
	 */
	if (ms_ord && !root_src && !sdrun.armed) {
		ft_ord_cell_run_unlink(src_ft, src_side_txn, ms_s_first,
			ms_s_last);
		src_side_txn = NULL;	/* consumed */
	}
	/* Reserved but unused: a non-root run whose unlink already fused it. */
	if (src_side_txn)
		ft_flip_txn_destroy(src_side_txn);
	if (!src_ft->exclusive)
		ft_writer_lock_gp_wait(src_ft);

	/*
	 * 2. Re-parent the SRC-origin referenced subtrees directly: the src
	 *    drain above made them unreachable to readers, and the cluster is
	 *    not yet forward-published, so this is invisible.  dst-origin
	 *    edges are NOT applied here -- they go through the flip.
	 */
	ft_glue_apply_deferred(dst_ft, &gd);

	/*
	 * 3. Record the structural edges into @txn (still PREPARE): every
	 *    dst-origin child's parent re-parent and the merge-point forward slot.
	 *    ft_glue_record_back_edge mirrors ft_set_parent's child-kind dispatch
	 *    (reading the field's current value as the old target) and runs the
	 *    writer-only slot bookkeeping early; the commit's settle below writes
	 *    each field to its new parent, subsuming the old step-7 ft_set_parent.
	 *    Nothing is parked yet -- commit's auto-install stages every proxy
	 *    (selector 0, resolving to old) so up-walks and the root descent still
	 *    see the pre-merge dst until the flip.
	 */
	{
		int j;

		for (j = 0; j < gd.nr_deferred; j++) {
			if (!gd.deferred[j].dst_origin)
				continue;
			ft_glue_record_back_edge(dst_ft, txn,
				gd.deferred[j].child, gd.deferred[j].parent,
				gd.deferred[j].slot);
		}
		/* Forward publish: old dst subtree -> merged cluster. */
		/*
		 * VALIDATE (§4.B) / LOCK_FINE (step 6, §9.4 M-2): acquire the LIVE dst
		 * parent above the overlap spine as a RELEASE lock.  The forward publish
		 * is a same-slot REPLACE (nr_child invariant, §9.4 finding 1) so
		 * pub_parent is a value-swap survivor not recompacted -- guard-fallback
		 * on an acquire miss is correct; non-lock_fine falls to the §4.B guard.
		 *
		 * UNLESS WE ALREADY HOLD IT.  When the dst merge point is an EXTERNAL
		 * node, the collided head IS that node and its chain holder is its
		 * immediate parent -- @pub_parent.  Re-marking a word this op already
		 * fenced MISSES, and a miss now sets acquire_miss and ABORTS a commit
		 * with no bail path left (the src is already unlinked).  Hand the held
		 * fence to the txn instead: hold_or_lock records the {LOCK|s -> s}
		 * release, the guard's strictly stronger twin, and the txn owns the
		 * unlock from here (so the take clears our entry).
		 */
		/*
		 * The holder was acquired BEFORE the src unlink (see the pre-acquire
		 * above), whether by marking @pub_parent here or by taking over the
		 * dup-chain fence when the two coincide.  Recording its release
		 * directly is what keeps this publish off lock_or_guard, whose miss
		 * would abort a commit that has no way left to fail safely.  NULL
		 * holder = non-lock_fine or the unfailable caller, which routes to the
		 * ordinary acquire-or-guard exactly as before.
		 */
		{
			struct ft_lock_ctx mctx;

			ft_glue_lock_ctx(&gd, &mctx);
			ft_flip_txn_hold_or_lock_parent(dst_ft, txn, &mctx,
				pub_parent, FT_DEPTH_FROM_DESCENT,
				gd.publish_parent_holder,
				gd.publish_parent_snap);
		}
		/*
		 * OWNERSHIP TRANSFER (mirrors the graft): @txn's registry now owns
		 * this fence -- a commit consumes it through the recorded release, an
		 * abort clears it -- so drop the glue's claim.  There is no bail left
		 * between here and the commit, but leaving a stale holder behind is
		 * the foot-gun that makes the NEXT bail added here a double clear.
		 */
		gd.publish_parent_holder = NULL;
		gd.publish_parent_snap = 0;
		/*
		 * @pub_slot is d_dst->nfp: &dst_ft->root at depth 0 (whose
		 * record is always MW and ignores the owner), and a child slot
		 * inside @pub_parent below it -- the node the hold_or_lock
		 * above put in this txn's registry.
		 */
		ft_flip_txn_record_publish(txn, dst_ft,
			pub_parent ? ft_flag_to_metadata(dst_ft, pub_parent) :
				NULL,
			pub_slot, D_old, M_slot);
	}

	/*
	 * 3b. Ordered list: collect the interleave and record its cell edges into
	 *    the SAME txn, STILL IN PREPARE -- no install before the collect.
	 *    The merged order is reconstructed by a two-pointer merge of the two
	 *    LIVE cell runs (the dst region up-walked over the intact D, and the
	 *    src run's suffixes captured pre-unlink), so the collect does NOT need
	 *    the about-to-be-published merged structure -- the last
	 *    append-after-install is gone, and every edge is recorded before
	 *    install.  Structure + ordered list thus commit in ONE flip (commit
	 *    auto-installs every recorded proxy, then flips): a reader never sees
	 *    the merged structural minimum ahead of the ordered-list front.  The
	 *    collect pre-sets the surviving cells' own links invisibly (not yet
	 *    ord-reachable) and returns only the visible boundary edges.  It runs
	 *    before the splice fold (step 3c), but collisions are invariant: a
	 *    collided src head is a floating duplicate (only in the splice record),
	 *    never a distinct reachable head, so the merge enumerates the same heads
	 *    either way -- and 3c reads each demoted head's cell from its still-intact
	 *    src-run prev, which this collect leaves untouched.
	 */
	if (ms_ord) {
		unsigned int i;

		ms_n = ft_merge_ord_interleave_collect(dst_ft, dst_key_len,
			ms_cursor, ms_succ, ms_prev, ms_src_caps, ms_nsrc,
			ms_src_pool, ms_edges, /*record_all=*/ false,
			/*ncollide=*/ NULL);
		/*
		 * ☠ A RAW record_tag LOOP OVER ORD EDGES, which the sibling
		 * graft path deliberately does NOT do (see the comment at
		 * ft_ord_cell_record_into_ft's caller there): every edge here
		 * takes the structural_sw dispatch, so a CELL edge parks SW
		 * under an armed txn even though no cell carries a node lock.
		 * Sound today only because the modes that arm -- COARSE and
		 * exclusive -- exclude trie-wide.  The per-edge @owner is what
		 * stops it at the PHASE B arm: a cell's owner is NULL, so the
		 * record-time check refuses the park instead of taking it
		 * silently.  Routing this loop through the tag-dispatching
		 * recorder is the real fix and belongs with the site's arm.
		 */
		for (i = 0; i < ms_n; i++)
			ft_flip_txn_record_tag(txn, ms_edges[i].owner,
				(void **) ms_edges[i].slot,
				(void *) ms_edges[i].old_target,
				(void *) ms_edges[i].new_target,
				ft_edge_tag(&ms_edges[i]));
	}

	/*
	 * 3c. Duplicate-chain concatenation: fold each collided src run's
	 *    tail-append into the SAME txn, still in PREPARE.  Recorded AFTER the
	 *    interleave collect (3b) so each demoted src head's cell is captured
	 *    from its intact src-run prev before the append overwrites it, and
	 *    BEFORE the commit so the src duplicates become reachable ATOMICALLY
	 *    with the merged structure -- a collided key never momentarily shows
	 *    only its dst duplicates (the old post-commit apply's window).  Each
	 *    splice is one forward edge (dst tail -> src run), so only the tail
	 *    carries a proxy; src_head->next rides along.  The rekey take() path
	 *    reaches this too -- cds_ft_rekey_merge unions into an OCCUPIED
	 *    destination, so a full key present on both sides collides there like
	 *    any other merge, which is why that path's pre-reservation budgets one
	 *    splice edge per moved key.  (Only cds_ft_rekey_graft demands an empty
	 *    destination, and it never reaches the spine copy.)
	 *
	 *    Every append here runs under the chain holder's node lock, taken
	 *    before the point of no return by ft_glue_acquire_splice_holders and
	 *    released after the commit below -- so the walk to the tail cannot race
	 *    a peer's append/unchain/promote on the same chain.
	 */
	ft_glue_record_splices(dst_ft, &gd, txn);

	/*
	 * Order-statistics fold (BULK): record the dst net key-count delta
	 * (merged_keys - cnt_dst) walk from @pub_parent up into the SAME txn, so
	 * the aggregate flips ATOMICALLY with the merged spine's forward publish
	 * (step 4) -- exact under concurrent writers.  @pub is a fresh node
	 * already carrying merged_keys, so the walk begins one level up at the
	 * stable @pub_parent.  A no-op when rank stats off or the count is
	 * unchanged.
	 */
	if (pub_parent && merged_keys != cnt_dst)
		ft_flip_txn_record_count_parent(dst_ft, txn, pub_parent,
			(long) merged_keys - (long) cnt_dst);

	/*
	 * 4. Commit: one selector flip switches every dst-origin parent, the
	 *    forward slot, AND every interleave cell edge from old to merged,
	 *    atomically, then settles each slot to its direct merged target.  (List
	 *    off with no dst-origin re-parent and no collided splice reduces to a
	 *    single release store of the forward slot -- no proxy, no grace period.)
	 *    Because the forward
	 *    slot flips with the back-pointers, a reader (descend then up-walk) only
	 *    progresses old->merged; and the ordered-list front advances in the same
	 *    instant the merged minimum becomes reachable.
	 */
	{
		enum urcu_txn_status mst = ft_flip_txn_commit(dst_ft, txn);

		/*
		 * The flip did not happen, so no fenced retire took effect and this
		 * op owns none of those frees -- renounce them before the step-7
		 * reclaim.  The dominant reason a fenced terminal aborts is a PEER
		 * retiring the node under our fence (the retire primitives do not
		 * honour LOCK), and that peer owns the reclaim: freeing here
		 * would be a double free on top of an already-lost merge.
		 *
		 * The merge itself still cannot recover -- the src is unlinked by
		 * now, which is the pre-existing abort-after-point-of-no-return
		 * exposure this shares with the unlocked pub_parent publish -- but
		 * it must not corrupt the arena on the way out.
		 */
		if (mst != URCU_TXN_STATUS_OK)
			ft_glue_fenced_renounce_free(&gd);
	}

	/*
	 * 5. Drop the dup-chain holder locks: the appends are installed, so peers
	 *    may mutate those chains again.  BEFORE the step-7 reclaim below --
	 *    most holders are dst overlap-spine nodes this merge retires, and the
	 *    fence has to come off while the node is still there to clear.  (The
	 *    fence survives the commit either way: the free-list retire records a
	 *    plain {LOCK|s -> LOCK|s|TOMBSTONE} upgrade, and an aborted
	 *    commit leaves {LOCK|s}.  The one holder that IS the publish target
	 *    was handed to @txn above and is already released by its flip.)
	 */
	ft_glue_release_splice_holders(&gd);
	/*
	 *    Same for the overlap-spine plan-locks: a COMMITTED fenced retire
	 *    consumed each fence into TOMBSTONE (clear_if_held no-ops), while an
	 *    ABORTED commit left {LOCK|s} that must come off or every later peer
	 *    publish into that node fails forever.  One unconditional sweep covers
	 *    both, which is why no per-outcome bookkeeping is kept.  Before the
	 *    step-7 reclaim, while the nodes are still addressable.
	 */
	ft_glue_clear_fenced(&gd);

	/*
	 * 6. The dst net key-count delta (merged_keys - cnt_dst) is FOLDED into
	 *    the step-4 commit above (recorded from @pub_parent into @txn before
	 *    the flip), so it goes live ATOMICALLY with the merged spine -- no
	 *    post-commit propagate walk.
	 */

	/*
	 * 7. Reclaim the old overlap spines (src-side to src, dst-side to dst).
	 *    The flip-txn was already committed-and-reclaimed at step 4 (commit
	 *    settled every proxied slot -- the dst-origin parents, the forward slot
	 *    and the interleave cell edges -- to its direct merged target, so the
	 *    proxies are unreferenced; its parked block is deferred through the FT
	 *    flavor, or freed in place when the commit owed no grace period).  No dst
	 *    synchronize_rcu -- the flip subsumed the dst drain.
	 */
	ft_glue_free_old(src_ft, &gs);
	ft_glue_free_old(dst_ft, &gd);

	/*
	 * 8. Free the collided (demoted) src heads' cells.  The interleave (folded
	 * into the flip above) already rewired the surviving cells' stale src-run
	 * links away from these cells, so they are unreachable to new readers, and
	 * the grace-period defer inside the free covers readers already holding such
	 * a link or parked on a demoted head.
	 */
	if (ms_ord) {
		free(ms_edges);
		free(ms_src_caps);
		free(ms_src_pool);
	}
	ft_glue_free_collided_cells(dst_ft, &gd);

	ft_glue_fini(&gd);
	ft_glue_fini(&gs);
	return CDS_FT_STATUS_OK;
}
#endif /* FEATURE_FT_MERGE: an occupied dst is a merge */

static
enum cds_ft_status ft_rekey_subpos_inplace(struct cds_ft *dst_ft,
		struct cds_ft *src_ft,
		const uint8_t *okey_dst, const uint8_t *dst_key, size_t dst_key_len,
		const uint8_t *okey_src, size_t src_key_len,
		struct cds_ft_inode_flag *payload, unsigned long cnt_src,
		bool *handled)
{
	struct ft_glue glue;
	struct cds_ft_alloc_reserve reserve;
	struct ft_descent d;
	enum ft_graft_prep prep;
	struct cds_ft_inode_flag *attached_nf;
	bool ms_ord = dst_ft->group->ordered_list_set;
	struct ft_ord_cell *pred = NULL, *succ = NULL;
	struct ft_ord_cell *run_first = NULL, *run_last = NULL;
	struct cds_ft_node *s_first = NULL, *s_last = NULL;
	struct ft_graft_run mrun;
	struct ft_graft_run *run_arg = NULL;
	struct ft_detach_run srun = { .into = NULL, .armed = false };
	struct ft_detach_run *srunp = NULL;
	size_t src_max, nm, dm;
	/*
	 * MW LOCK_FINE drop: the "failure-free" commit below is only unfailable
	 * under the FT-wide lock -- under the drop its MCAS commit (GLUE) or its
	 * recompact prepare (NOSPLIT) can conflict with a peer and fail AFTER
	 * @ft_merge_unlink_src_subtree already excised the source subtree.  That
	 * unlink PRESERVES @payload (owned by this op, src pristine-empty), so
	 * the whole dst-side attach (re-descend -> build -> commit) is a
	 * retryable one-shot: @already_unlinked guards the src-side excise + the
	 * ordered-list run capture so a retry re-runs only the dst attach, and
	 * @glue.fence_split_cn fences the compressed divergence node (as
	 * cds_ft_graft) so a concurrent grow of it is arbitrated (fence miss ->
	 * FT_GRAFT_PREP_RETRY -> re-descend).  Mirrors cds_ft_graft's post-swap
	 * store retry.
	 */
	bool already_unlinked = false;
	struct ft_flip_txn *run_unlink_txn = NULL;
	struct ft_flip_txn *run_splice_txn = NULL;

	*handled = false;

	/*
	 * Classify the dst graft point with a build-invisible prep.  A GLUE
	 * diverge builds the whole split cluster into @glue (referencing @payload
	 * by deferred edge), published failure-free after the drain.  A NOSPLIT
	 * point is grafted post-drain by ft_store_at_graft_point drawing from a
	 * pre-filled reserve, so it likewise cannot fail on an arena allocation.
	 * Either way the source unlink is the single last fallible step, so an OOM
	 * leaves both tries pristine -- no rollback, no leak.
	 */
	unsigned long rm_depth __attribute__((unused)) = 0;

retry_merge:
	RSPIN_ENTER_X(0, rm_depth, 0, dst_ft->lock_fine && src_ft->exclusive);
	RSPIN_SITE_ENTER(1, rm_depth, dst_ft->lock_fine && src_ft->exclusive);
	ft_glue_init(&glue);
	/*
	 * Fence the compressed divergence node (like cds_ft_graft), so a
	 * concurrent grow of it is arbitrated and the build re-descends on a
	 * miss (FT_GRAFT_PREP_RETRY); ft_glue_init reset it, so set each attempt.
	 */
	glue.fence_split_cn = true;
	memset(&reserve, 0, sizeof(reserve));
	/*
	 * Every attach shape -- GLUE diverge and the NOSPLIT in-place store --
	 * commits through @glue.txn (exactly like cds_ft_graft): the build (below)
	 * tags its displaced old child dst_origin, and the NOSPLIT store reserves
	 * its slot proxy + run-splice edges, all out of this txn.  Create it BEFORE
	 * the build; destroyed on a POPULATED point (nothing to commit).
	 */
	glue.txn = ft_flip_txn_create(dst_ft);
	if (!glue.txn || !ft_flip_txn_reserve(glue.txn,
			/* +1: fused recompact-relocate tombstone (§4.B);
			 * + FLOOR_FREE: fused free-list tombstones;
			 * + count walk: the +cnt_src nr_keys ancestor edges (BULK fold) */
			FT_GLUE_FLOOR_DEFERRED + 7 + 1 /* +1 §4.B parent guard */ + FT_GLUE_FLOOR_FREE
				+ (dst_ft->rank_stats ? (int) dst_key_len + 1 : 0))) {
		if (glue.txn)
			ft_flip_txn_destroy(glue.txn);
		ft_glue_fini(&glue);
		if (already_unlinked)
			goto retry_merge;	/* src consumed: OOM is transient */
		return CDS_FT_STATUS_MEMORY_ERROR;
	}
	glue.fuse_free_list = true;	/* reserved free-list headroom above (§4.B) */
	prep = ft_graft_build(dst_ft, okey_dst, dst_key_len, payload, cnt_src,
			&d, &glue, /*outer*/ NULL);
	*handled = true;
	if (prep == FT_GRAFT_PREP_RETRY) {
		/*
		 * Compressed-divergence @cn fence miss: a peer owns @cn.  Nothing
		 * built; a clean re-descend (pre-unlink both tries pristine, post-
		 * unlink src is empty + @payload owned).
		 */
		ft_glue_abort(dst_ft, &glue);
		ft_flip_txn_destroy(glue.txn);
		goto retry_merge;
	}
	if (prep == FT_GRAFT_PREP_OOM) {
		ft_glue_abort(dst_ft, &glue);
		ft_flip_txn_destroy(glue.txn);
		if (already_unlinked)
			goto retry_merge;	/* src consumed: OOM is transient */
		return CDS_FT_STATUS_MEMORY_ERROR;	/* both tries pristine */
	}
	if (prep == FT_GRAFT_PREP_POPULATED) {
		/*
		 * Defensive: cnt_dst == 0 should never yield an occupied point.
		 * Impossible post-unlink (the payload is already excised + owned;
		 * no clean rc remains) -- assert rather than orphan it.
		 */
		assert(!already_unlinked);
		ft_flip_txn_destroy(glue.txn);
		ft_glue_fini(&glue);
		return CDS_FT_STATUS_POPULATED_ERROR;
	}
	if (prep == FT_GRAFT_PREP_NOSPLIT) {
		/*
		 * Pre-fill the store's node reserve while both tries are still
		 * pristine.  The store's slot proxy + run-splice edges draw from the
		 * already-reserved @glue.txn, so no separate flip batch is needed.
		 * Securing the reserve here -- before the source unlink -- makes the
		 * post-drain store wholly failure-free, so the unlink is the true last
		 * fallible step and nothing strands.
		 *
		 * Use the GENEROUS superset fill (as ft_graft_keylen's NOSPLIT attach
		 * does for the same ft_store_at_graft_point), not a hand-rolled exact
		 * manifest: predicting the store's node set is fragile (e.g. attaching
		 * a high byte forces a RANGE recompact of the attach node even within
		 * its child capacity -- a grow an exact count heuristic misses, which
		 * underflowed the reserve and aborted).  A bulk op already pays a grace
		 * period, so the throwaway pops/pushes are negligible.
		 */
		if (ft_bulk_node_reserve_fill(dst_ft, &reserve)) {
			cds_ft_alloc_reserve_drain(dst_ft, &reserve);
			ft_glue_abort(dst_ft, &glue);
			ft_flip_txn_destroy(glue.txn);
			if (already_unlinked)
				goto retry_merge;	/* src consumed: transient */
			return CDS_FT_STATUS_MEMORY_ERROR;
		}
	}
	/* GLUE: cluster built invisibly.  NOSPLIT: reserve secured (+ glue.txn). */

	/*
	 * Graft-parity (cds_ft_graft hoists the same test pre-swap): an exact-
	 * depth OCCUPIED NOSPLIT point is a PERMANENT POPULATED condition.  Catch
	 * it HERE, on the FIRST pass BEFORE the irreversible source unlink, so a
	 * misusing (non-disjoint) caller gets a clean POPULATED_ERROR with BOTH
	 * tries pristine -- rather than reaching the post-unlink store, asserting,
	 * and stranding @payload.  For the supported absent/diverged dst point
	 * (d.nf == NULL at key_len -- the only shape this helper is dispatched for)
	 * this is dead code.  A retry (post-unlink) cannot newly occupy the point
	 * under the disjoint exclusive-src contract, so the store's own POPULATED
	 * discrimination (below) stays a dead assert there too.
	 */
	if (!already_unlinked && prep == FT_GRAFT_PREP_NOSPLIT
			&& d.depth == dst_key_len && d.nf) {
		cds_ft_alloc_reserve_drain(dst_ft, &reserve);
		ft_glue_abort(dst_ft, &glue);
		ft_flip_txn_destroy(glue.txn);
		return CDS_FT_STATUS_POPULATED_ERROR;
	}

	/*
	 * Ordered list: locate the dst splice neighbours (RE-FOUND each attempt --
	 * dst may have grown across a retry) and, on the FIRST attempt only,
	 * capture the source subtree's run endpoints + cells while the subtree is
	 * still intact in src.  @run_first / @run_last / @s_first / @s_last persist
	 * across retries: the run is excised ONCE by the unlink below and then
	 * owned (out of both lists), so a retry re-splices the same owned run.
	 */
	if (ms_ord) {
		ft_ord_cell_find_splice_pos(dst_ft, dst_key, dst_key_len,
			&pred, &succ, NULL);
		if (!already_unlinked) {
			s_first = ft_subtree_minmax_head(src_ft, payload, false);
			s_last = ft_subtree_minmax_head(src_ft, payload, true);
			run_first = ft_ord_cell_ptr(rcu_dereference(s_first->prev));
			run_last = ft_ord_cell_ptr(rcu_dereference(s_last->prev));
			/* Src side: EXCISE-ONLY run so the structural unlink and the
			 * run's removal from src's ordered list commit in ONE flip
			 * (the disappear-side cross-view fix). */
			srun.rfirst = s_first;
			srun.rlast = s_last;
			srunp = &srun;
		}
		mrun.run_first = run_first;
		mrun.run_last = run_last;
		mrun.pred = pred;
		mrun.succ = succ;
		mrun.armed = false;
		run_arg = &mrun;
	}

	/*
	 * Source-side excise (ONE-SHOT, @already_unlinked): reserve the run txns,
	 * unlink the source subtree in place (preserving @payload), remove the run
	 * from src's list, drain src readers, and stamp an external payload's edge
	 * byte.  A retry after a post-unlink commit abort skips all of this -- src
	 * is already empty and @payload is owned -- and re-runs only the dst attach.
	 */
	if (!already_unlinked) {
		if (ms_ord) {
			/*
			 * Pre-reserve the standalone run-unlink + run-splice txns (the
			 * rare unfused shapes touch src's / dst's list AFTER the
			 * structural flip is public, un-abortable).  OOM here aborts the
			 * still-invisible build (both tries pristine).
			 */
			run_unlink_txn = ft_flip_txn_create_bounded(src_ft,
				FT_ORD_CELL_RUN_UNLINK_MAX_EDGES);
			run_splice_txn = ft_flip_txn_create_bounded(dst_ft,
				FT_ORD_CELL_RUN_SPLICE_MAX_EDGES);
			if (!run_unlink_txn || !run_splice_txn) {
				if (run_unlink_txn)
					ft_flip_txn_destroy(run_unlink_txn);
				if (run_splice_txn)
					ft_flip_txn_destroy(run_splice_txn);
				run_unlink_txn = run_splice_txn = NULL;
				cds_ft_alloc_reserve_drain(dst_ft, &reserve);
				ft_glue_abort(dst_ft, &glue);
				ft_flip_txn_destroy(glue.txn);
				return CDS_FT_STATUS_MEMORY_ERROR;
			}
		}

		/*
		 * Last fallible SRC step: unlink the source subtree in place,
		 * preserving @payload.  On OOM the unlink self-undoes (src pristine)
		 * and the still-invisible cluster / reserve is released -- no rollback.
		 */
		if (ft_merge_unlink_src_subtree(src_ft, okey_src, src_key_len,
				cnt_src, srunp, /*retire_glue=*/ NULL) < 0) {
			if (run_unlink_txn)
				ft_flip_txn_destroy(run_unlink_txn);
			if (run_splice_txn)
				ft_flip_txn_destroy(run_splice_txn);
			run_unlink_txn = run_splice_txn = NULL;
			cds_ft_alloc_reserve_drain(dst_ft, &reserve);
			ft_glue_abort(dst_ft, &glue);
			ft_flip_txn_destroy(glue.txn);
			return CDS_FT_STATUS_MEMORY_ERROR;
		}
		already_unlinked = true;	/* POINT OF NO RETURN: @payload owned */

		/*
		 * Remove the source run from src's ordered list (cells keep their
		 * internal links for the dst splice) -- BEFORE the drain, so sync
		 * drains src ord-readers of the run too.  The structural unlink FUSED
		 * this into its flip (@srun.armed); the standalone remains a fallback.
		 */
		if (ms_ord && !srun.armed) {
			ft_ord_cell_run_unlink(src_ft, run_unlink_txn, s_first,
				s_last);
			run_unlink_txn = NULL;	/* consumed */
		}
		if (run_unlink_txn) {
			ft_flip_txn_destroy(run_unlink_txn);
			run_unlink_txn = NULL;
		}

		if (!src_ft->exclusive)
			ft_writer_lock_gp_wait(src_ft);

		/*
		 * An EXTERNAL payload's edge byte changes (src_key's last byte ->
		 * dst_key's); stamp it in the head's CELL metadata now -- the cell is
		 * in NEITHER list and structurally invisible, so no reader rebuilds a
		 * key from it (stamping while still in src's list would let a src
		 * reader rematerialize an out-of-namespace key).  Internal/compressed
		 * payloads keep every leaf's edge byte (the subtree moves wholesale).
		 */
		if (ms_ord && ft_node_external(payload))
			cds_ft_item_to_metadata(run_first)->incoming_byte =
				okey_dst[dst_key_len - 1];
	}

	if (prep == FT_GRAFT_PREP_GLUE) {
		/*
		 * Failure-free commit of the build-invisible diverge cluster: the
		 * displaced old child is LIVE (still reachable through the compressed
		 * node being split until the forward publish), so via @glue.txn it
		 * flips atomically with the forward publish and the ordered-list
		 * run-splice -- the whole attach observed old XOR new.  The payload's
		 * hidden back-pointers are wired immediately inside the commit.  Then
		 * reclaim the replaced compressed node.
		 */
		/*
		 * Order-statistics fold (BULK): the diverge cluster raises
		 * @glue.publish_parent's subtree by +cnt_src; record that walk
		 * into the same commit (a no-op when rank stats off).
		 */
		enum urcu_txn_status cst;

		glue.count_delta = (long) cnt_src;
		cst = ft_glue_txn_commit(dst_ft, &glue, run_arg);
		if (cst != URCU_TXN_STATUS_OK) {
			/*
			 * MW LOCK_FINE drop: the GLUE commit's MCAS footprint (the
			 * forward publish + fused run-splice + fenced retires) can
			 * conflict with a peer and ABORT even though it allocates
			 * nothing.  The flip rolled back (dst byte-for-byte unchanged,
			 * the run-splice not applied); ft_glue_txn_commit consumed
			 * glue.txn.  Free the failed attempt's invisible cluster and
			 * re-attach the owned @payload via the retry.  @run_splice_txn
			 * / @run_first / @run_last persist (src already excised).
			 */
			ft_glue_abort(dst_ft, &glue);
			goto retry_merge;
		}
		attached_nf = glue.attached_nf;
		ft_glue_free_old(dst_ft, &glue);
		ft_glue_fini(&glue);
	} else {
		/*
		 * NOSPLIT: graft the in-place payload at @d.  Node allocations draw
		 * from the reserve and the slot proxy + run-splice edges from the
		 * pre-reserved @glue.txn.  Under the drop the store's recompact of
		 * the (contended) spine can still FAIL its prepare -- ft_store_at_
		 * graft_point returns -EAGAIN / MEMORY_ERROR (not the FT-wide-lock
		 * "cannot fail") -- after the source is already excised; re-attach
		 * the owned @payload via the retry.
		 */
		unsigned int adepth = 0;
		enum cds_ft_status st;

		cds_ft_alloc_reserve_activate(dst_ft, &reserve);
		st = ft_store_at_graft_point(dst_ft, okey_dst, dst_key_len, &d,
				payload, cnt_src, &attached_nf, &adepth, &glue,
				run_arg,
				(long) cnt_src);
		cds_ft_alloc_reserve_deactivate(dst_ft);
		cds_ft_alloc_reserve_drain(dst_ft, &reserve);
		if (st != CDS_FT_STATUS_OK) {
			/*
			 * ft_store_at_graft_point already freed its invisible build +
			 * txn (and, on a commit abort, the relocated copy) and cleaned
			 * up @glue.  Discriminate the failure class:
			 *
			 * - POPULATED is a PERMANENT occupied-point condition, NOT a
			 *   transient conflict -- retrying it would LIVELOCK and strand
			 *   the owned @payload.  Post-unlink it is impossible for the
			 *   supported disjoint exclusive-src contract (no peer creates
			 *   the moved key), so assert as the pre-unlink PREP_POPULATED
			 *   arm does.  A general (non-disjoint) caller -- a peer racing
			 *   the moved key into the target during the owned-but-unattached
			 *   window -- would still orphan @payload here: this retry-based
			 *   recovery excises the source BEFORE the dst attach, so the
			 *   window exists.  cds_ft_graft no longer has it: its exclusive
			 *   src-swap-fused arm lands the src retire ATOMICALLY with the
			 *   attach (only on commit success), so the same race MCAS-aborts,
			 *   retries, and re-detects POPULATED pre-commit with the source
			 *   never swapped.  Closing this gap for the sub-position move --
			 *   fusing ft_merge_unlink_src_subtree into the attach flip -- is a
			 *   deferred parity follow-up (not needed for any disjoint use).
			 * - Everything else is transient -- re-descend and re-attach
			 *   the owned @payload: BUSY_ERROR is contention (a peer
			 *   filled the reserve's byte, a recompact could not lock, or
			 *   the store's MCAS commit ABORTed), MEMORY_ERROR is a real
			 *   allocation failure.  Both retry; they are kept distinct so
			 *   a conflict never has to be read as an OOM.
			 */
			if (st == CDS_FT_STATUS_POPULATED_ERROR) {
				assert(!already_unlinked);
				return CDS_FT_STATUS_POPULATED_ERROR;
			}
			goto retry_merge;
		}
		ft_glue_fini(&glue);
	}

	/*
	 * Order-statistics: the +cnt_src ancestor walk is now FOLDED onto the
	 * attach commit above (glue.count_delta for the GLUE diverge and
	 * displaced-external shapes; ft_store_at_graft_point's count_delta for
	 * the in-place / recompact-relocate slot shapes), so it flips ATOMICALLY
	 * with the structural publish -- exact under concurrent writers.
	 */

	/*
	 * Ordered list: every attach shape (GLUE, displaced-external, in-place
	 * slot) FUSED the run-splice into its structural flip (@armed) -- the
	 * external edge-byte was stamped before that splice, above.  The standalone
	 * two-commit splice remains as a defensive fallback for any not-yet-fused
	 * shape (none today); without it an unfused shape would strand the moved
	 * run out of the ordered list.  It commits through the pre-reserved
	 * @run_splice_txn (un-abortable post-drain); the fused common case frees
	 * that txn unused.
	 */
	if (ms_ord && !mrun.armed) {
		ft_ord_cell_run_splice(dst_ft, run_splice_txn, run_first,
			run_last, pred, succ);
		run_splice_txn = NULL;	/* consumed */
	}
	if (run_splice_txn)
		ft_flip_txn_destroy(run_splice_txn);	/* fused: unused */

	/* Raise dst's max_used_key_len for the moved keys (dst_key || suffix). */
	src_max = uatomic_load(&src_ft->max_used_key_len, CMM_RELAXED);
	nm = src_max > src_key_len ? dst_key_len + (src_max - src_key_len) :
		dst_key_len;
	dm = uatomic_load(&dst_ft->max_used_key_len, CMM_RELAXED);
	if (nm > dm)
		uatomic_store(&dst_ft->max_used_key_len, nm, CMM_RELAXED);

	return CDS_FT_STATUS_OK;
}

/*
 * @pre_txn carries a flip-txn the caller reserved before its own last fallible
 * step, so the spine-copy / graft commit below draws an unfailable txn instead
 * of allocating one.  NULL on the public paths (cds_ft_merge_at and the outer
 * cds_ft_rekey_* call), set only by the same-trie rekey's post-detach re-merge,
 * which pre-reserves it (sized from the O(1) subtree key counts:
 * the structural re-parent + folded ordered-list interleave, or the graft
 * cluster floor) so its post-detach merge cannot fail -- no reader-observable
 * rollback.  The consume site NULLs the slot it takes, so the rekey frees the
 * txn only when a given merge shape left it unused.
 */
static enum cds_ft_status ft_rekey_at_inner(struct cds_ft *dst_ft,
		const uint8_t *dst_key, size_t dst_key_len,
		struct cds_ft *src_ft,
		const uint8_t *src_key, size_t src_key_len,
		struct ft_flip_txn **pre_txn, enum ft_rekey_mode rekey)
{
	struct cds_ft *subtree = NULL;
	enum cds_ft_status status;
	struct ft_descent d_src, d_dst;
	unsigned int off_src, off_dst;
	unsigned long cnt_src, cnt_dst;
	enum ft_graft_swap_case ks, kd;
	uint8_t okey_dst_buf[FT_MAX_KEY_LEN], okey_src_buf[FT_MAX_KEY_LEN];
	const uint8_t *okey_dst = dst_key, *okey_src = src_key;

	FT_TP(merge_enter, (const void *) dst_ft, (const void *) src_ft);

	if (!dst_ft || !src_ft) {
		FT_TP(merge_exit, (int) CDS_FT_STATUS_INVALID_ARGUMENT_ERROR);
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}
	if (dst_ft->group != src_ft->group) {
		FT_TP(merge_exit, (int) CDS_FT_STATUS_INVALID_ARGUMENT_ERROR);
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}
	if (dst_key_len > dst_ft->group->max_key_len ||
			src_key_len > dst_ft->group->max_key_len) {
		FT_TP(merge_exit, (int) CDS_FT_STATUS_INVALID_ARGUMENT_ERROR);
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}
	if ((dst_key_len > 0 && !dst_key) ||
			(src_key_len > 0 && !src_key)) {
		FT_TP(merge_exit, (int) CDS_FT_STATUS_INVALID_ARGUMENT_ERROR);
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}
	/*
	 * Fixed-length groups: each source key K has length fixed_len,
	 * the moved subtree's stripped keys have length
	 * (fixed_len - src_key_len), and the resulting destination key
	 * is dst_key || stripped, of length
	 * (dst_key_len + fixed_len - src_key_len).  For that result to
	 * equal fixed_len (the only key length the destination group
	 * accepts), src_key_len and dst_key_len must be equal.
	 */
	if (dst_ft->group->key_len != CDS_FT_LEN_VARIABLE
			&& dst_key_len != src_key_len) {
		FT_TP(merge_exit, (int) CDS_FT_STATUS_INVALID_ARGUMENT_ERROR);
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}
	/*
	 * merge_at is CROSS-TRIE only.  A same-trie move (src_ft == dst_ft) is a
	 * REKEY -- expressed by cds_ft_rekey_graft / cds_ft_rekey_merge, which reach
	 * this worker with @rekey set.  A same-trie cds_ft_merge_at (@rekey ==
	 * FT_REKEY_NONE) is rejected; use the dedicated rekey entry points instead.
	 */
	if (rekey == FT_REKEY_NONE && src_ft == dst_ft) {
		FT_TP(merge_exit, (int) CDS_FT_STATUS_INVALID_ARGUMENT_ERROR);
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}
	/*
	 * A rekey on a SPECULATIVE (leaf-stored-key) trie is refused: the move
	 * re-parents each leaf under @dst_key WITHOUT rewriting its app-owned stored
	 * key -- the library never writes that field -- so every moved leaf would
	 * carry a key that no longer matches its structural position, and a
	 * speculative lookup would then return the wrong key
	 * (ft_verify_speculative_key catches this only under
	 * FEATURE_FT_VERIFY_AT_MUTATION).  Only EAGER tries -- which reconstruct the
	 * key from structure and never read the stored field -- may rekey.
	 */
	if (rekey != FT_REKEY_NONE && dst_ft->speculative_key_offset_active) {
		FT_TP(merge_exit, (int) CDS_FT_STATUS_INVALID_ARGUMENT_ERROR);
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}
	/*
	 * The STAGED rekey needs a VARIABLE-length group, and the refusal belongs
	 * HERE, before anything is read or reserved.  The move is staged as a
	 * detach of @src_key's subtree into a transient trie followed by a merge of
	 * that trie back in at @dst_key, and a detached subtree carries keys
	 * STRIPPED of the prefix -- shorter than a fixed-length group's one key
	 * length, which is exactly why cds_ft_detach and cds_ft_graft take a
	 * non-root key on variable-length groups only.  The staged rekey is
	 * composed of those two operations, so it inherits the restriction.
	 *
	 * A fixed-length group is served by the ATOMIC writer instead
	 * (ft_rekey_one_decide, dispatched before this worker), which stages
	 * through no transient trie and so has no such restriction -- it is the
	 * only rekey a fixed-length group gets, and reaching here means its cut did
	 * not cover the shape.
	 *
	 * Enforcing it at the entry is what makes the refusal SAFE.  The placement
	 * is a merge of that transient at @dst_key, so it meets the fixed-length
	 * equal-prefix-length guard above with src_key_len == 0 != dst_key_len and
	 * refuses -- but only AFTER the detach has committed, leaving the caller an
	 * INVALID_ARGUMENT_ERROR (an "argument rejected, nothing happened" status)
	 * for a trie that has just lost every moved key to the destroyed transient.
	 */
	if (rekey != FT_REKEY_NONE &&
			dst_ft->group->key_len != CDS_FT_LEN_VARIABLE) {
		FT_TP(merge_exit, (int) CDS_FT_STATUS_INVALID_ARGUMENT_ERROR);
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}
	/*
	 * Combined-length overflow validation (mirrors cds_ft_graft): a moved
	 * key K becomes dst_key || (K - src_key prefix), of length
	 * dst_key_len + len(K) - src_key_len.  Bound len(K) by the source's
	 * max_used_key_len; without this check a variable-length merge with
	 * dst_key_len > src_key_len could create keys exceeding the group's
	 * max_key_len, overflowing the fixed-size key buffers downstream
	 * (the spine's compressed-wrap kbuf, the iterator buffers).
	 */
	{
		size_t src_max = uatomic_load(&src_ft->max_used_key_len,
				CMM_RELAXED);

		if (src_max > src_key_len &&
				src_max - src_key_len >
				dst_ft->group->max_key_len - dst_key_len) {
			FT_TP(merge_exit, (int) CDS_FT_STATUS_OVERFLOW_ERROR);
			return CDS_FT_STATUS_OVERFLOW_ERROR;
		}
	}

	/*
	 * MW LOCK_FINE (step 6, §9.5): a CROSS-trie merge consumes @src_ft, so it
	 * must be EXCLUSIVE -- an exclusive source skips its FT-wide lock, leaving
	 * only dst's lock (one lock, no cross-trie deadlock).  A live (lock-mode,
	 * non-exclusive) cross-trie source is REJECTED with BUSY before any lock is
	 * taken; the caller makes it exclusive first (cds_ft_make_exclusive).  A
	 * SAME-trie rekey (src == dst) is excluded -- it takes one lock reentrantly.
	 * Inert outside lock-mode.
	 */
	if (src_ft != dst_ft && !src_ft->exclusive) {
		FT_TP(merge_exit, (int) CDS_FT_STATUS_BUSY_ERROR);
		return CDS_FT_STATUS_BUSY_ERROR;
	}

	/*
	 * merge_at is a mutator; the application provides mutual exclusion
	 * between mutators.  Take the reentrant writer-validation scope (like
	 * cds_ft_graft_swap), NOT a blanket flavor read lock.
	 *
	 * The reason is the REKEY entries, not merge_at.  cds_ft_merge_at
	 * consumes an EXCLUSIVE source, so every ft_writer_lock_gp_wait on its
	 * path -- both here and both in ft_graft_keylen -- is
	 * !src_ft->exclusive-gated and never runs; that path syncs nowhere, and
	 * the spine copy pins it under a read lock for exactly that reason.
	 * cds_ft_rekey_{graft,merge} share this body with src_ft == dst_ft, a
	 * LIVE trie: there those waits DO run, and a read lock held across one
	 * would be a writer waiting on its own grace period.  One body, two
	 * source contracts -- so the pin is taken per path, not here.
	 */
	ft_crosstrie_lock_mode_guard(dst_ft, src_ft);
	CDS_FT_SCOPED_WRITER(dst_ft);
	CDS_FT_SCOPED_WRITER(src_ft);

	/*
	 * Convert both keys to ORDINAL form ONCE; every internal consumer
	 * (the merge-point descents, the spine build, the source-subtree
	 * unlink, the ordered-list interleave) takes the ordinal form.  The
	 * detach+graft fallback below instead receives the ORIGINAL
	 * application keys -- those entry points remap internally.
	 * Previously the descents consumed the application bytes raw while
	 * the unlink remapped: on a non-identity key map the build copied one
	 * subtree and the unlink targeted another.
	 */
	{
		const struct cds_ft_key_map *km = &dst_ft->group->key_map;

		if (caa_unlikely(!km->identity)) {
			ft_key_to_ordinals(okey_dst_buf, dst_key, dst_key_len,
				km);
			ft_key_to_ordinals(okey_src_buf, src_key, src_key_len,
				km);
			okey_dst = okey_dst_buf;
			okey_src = okey_src_buf;
		}
	}

	/*
	 * Same-trie "rekey" (src_ft == dst_ft): moving @src_key's subtree to
	 * @dst_key within one trie is allowed, but the two keys must be DISJOINT
	 * -- neither a prefix of the other.  A prefix relationship means one key
	 * lies inside the other's subtree, so the move would be circular (and
	 * @dst_key would not be absent), and equal keys are a degenerate self-
	 * move.  The check is byte-position-wise, identical in application and
	 * ordinal space (the key map is a per-position bijection).  Cross-trie
	 * subtrees are disjoint by construction, so the guard is same-trie only.
	 */
	if (rekey != FT_REKEY_NONE) {
		size_t m = src_key_len < dst_key_len ? src_key_len : dst_key_len;

		if (m == 0 || memcmp(okey_src, okey_dst, m) == 0) {
			FT_TP(merge_exit,
				(int) CDS_FT_STATUS_INVALID_ARGUMENT_ERROR);
			return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
		}
	}

	/*
	 * Locate both merge points read-only (a writer descends its own
	 * stable state).  No content under @src_key -> the merge is a no-op;
	 * cnt_src == 0 covers an empty @src_ft root (src_key_len == 0, where
	 * the descent still reports EXACT at the always-present root).
	 */
	ks = ft_merge_descend(src_ft, okey_src, src_key_len, &d_src,
			&off_src, &cnt_src);
	MRG_SKIPCONF_PROBE(0, d_src);
	if (ks == FT_GRAFT_SWAP_DELEGATE || cnt_src == 0) {
		FT_TP(merge_exit, (int) CDS_FT_STATUS_OK);
		return CDS_FT_STATUS_OK;
	}

	/*
	 * Same-trie "rekey": detach @src_key's subtree into a transient trie,
	 * then merge that whole trie back in at @dst_key.  The detach fully
	 * commits -- recompacting the shared common-prefix ancestor and draining
	 * -- BEFORE the placement reads it, so the move's two sides no longer
	 * alias the same node.  The in-place reorder CANNOT be used here: its
	 * source unlink would recompact that shared ancestor out from under the
	 * build-invisible destination placement (a stale-slot publish).  The
	 * cross-trie merge handles an occupied @dst_key by MERGING and an absent
	 * one by grafting; it is made UNFAILABLE (its nodes + flip batches are
	 * pre-reserved before the detach below), so it always commits the move --
	 * there is no failure path that would restore the content to @src_key, i.e.
	 * no reader-observable rollback.
	 *
	 * A KEY_SHORTER source (off_src > 0, @src_key ends inside the compressed
	 * node @d_src.nf) is reduced to the EXACT node boundary first, exactly as
	 * the cross-trie path does: detach @cn_s->child via the full-compressed
	 * key (src prefix ++ all of cn_s), and merge at @dst_key extended by the
	 * residual cn_s bytes -- ft_detach_keylen overshoots a compressed node, so
	 * it must be handed a boundary key, not an interior one.
	 */
	/*
	 * ☠ THE STAGED REKEY WRITER IS GONE.  It detached the subtree into a
	 * TEMPORARY TRIE and merged it back, so the moved keys were invisible to
	 * readers for a grace period per move -- measured by the suite itself as
	 * 1,163,089 absences over 18 moves.  An FT-wide writer lock does not
	 * redeem it: RCU readers run CONCURRENTLY with the writer whatever the
	 * writer-exclusion mode, so the absence window is a guarantee failure in
	 * every configuration, not a property of the fine-grained modes.
	 *
	 * A same-trie rekey is therefore served by the ATOMIC writer or not at
	 * all: ft_rekey_one_decide commits the src clear, the dst publish and the
	 * re-parents as ONE flip, and ft_rekey_dispatch routes every shape its cut
	 * covers there.  Reaching this worker with @rekey set means the cut did
	 * NOT cover the shape -- answer that, terminally, instead of staging a
	 * move through a window no probe can talk away.
	 *
	 * (@rekey == FT_REKEY_NONE, the CROSS-TRIE cds_ft_merge_at, continues
	 * below: it publishes build-invisibly into a separate destination trie
	 * and never hides a live key.)
	 */
	if (rekey != FT_REKEY_NONE)
		return CDS_FT_STATUS_NOT_SUPPORTED;

	bool md_rlock = false;
	bool md_contended;
	unsigned long md_spin = 0;	/* this op's merge_spine_retry depth (probe) */
	/*
	 * The op's PERSISTENT engine handle, spanning the whole merge_spine_retry
	 * loop the way ft_txn_op_init does for insert, remove and replace (doc
	 * §11).  Without it this loop ages nothing: every attempt started at
	 * retry 0, so the domain never escalated the writer into the per-trie
	 * FIFO fair-mutex lane and the loop had no termination argument at all --
	 * measured at 585250 declined lock sets over 235141 merges under an
	 * exponential spacing, one op re-descending 348 times.
	 *
	 * It is bound and bracketed ONLY on the arm that takes the read pin
	 * below, and that is not an optimization: urcu_txn_begin() enters the
	 * RCU read side, and the SAME-TRIE rekey shares this body with a LIVE
	 * src whose ft_writer_lock_gp_wait calls do run -- a read section held
	 * across one is a writer waiting on its own grace period.  The condition
	 * `dst_ft->lock_fine && src_ft->exclusive` is exactly the source
	 * contract that syncs nowhere, and it is where every one of those
	 * 585250 declines was measured (livesrc=0).
	 */
	struct urcu_txn optxn;

	MRG_SPIN_PROBE(0);
	ft_txn_op_init(dst_ft, &optxn);

	/*
	 * Re-entered when a spine-copy attempt could not take its dup-chain lock
	 * set (@md_contended).  The attempt moved nothing -- an EXCLUSIVE source
	 * means the merge is build-invisible until its one commit, so a declined
	 * attempt is trivially undone and both tries are byte-for-byte as they
	 * were.  Re-descend, because the whole reason the acquire declined is that
	 * a peer is reshaping the dst spine we planned against, and rebuild.  Same
	 * plan->commit retry shape as cds_ft_graft's retry_attach and
	 * cds_ft_graft_swap's retry_swap.
	 */
merge_spine_retry:
	/*
	 * §11 cross-trie RCU-pinning: the spine-copy path below descends the live
	 * dst here and node locks a descent-captured dst node (@d_dst->pnf /
	 * ->ppnf) inside ft_rekey_spine_copy.  A node lock false-succeeds on a
	 * reclaimed+recycled node (arena re-zeroes metadata), so pin the captured
	 * nodes with the flavor read side across descent -> lock, as cds_ft_graft
	 * does.  The dup-chain holders the splice lock set acquires are captured
	 * during that same window, so this pin covers them too.  With an EXCLUSIVE
	 * src the spine-copy's only grace period (its src drain) is
	 * !src_ft->exclusive-gated and skipped, so the whole ft_rekey_spine_copy
	 * runs GP-free under the read lock.  Released before the spine-copy return,
	 * on the fall-through to the detach/graft paths, and before each retry
	 * above (a retry re-descends, so it must re-pin what it re-reads).
	 */
	if (dst_ft->lock_fine && src_ft->exclusive) {
		/*
		 * Open the attempt on the persistent handle FIRST: a retry that
		 * has aged escalates here, and escalation blocks on the domain's
		 * fair mutex, which must not happen holding the pin below.  The
		 * explicit read_lock stays -- it is what pins the descent-captured
		 * dst nodes, and it must not become conditional on the handle
		 * having a flavour bound (ft_txn_op_init binds NULL on an
		 * exclusive dst).  RCU read sections nest; @md_rlock now marks
		 * both, and every site that releases it closes the txn too.
		 */
		urcu_txn_begin(&optxn);
		dst_ft->group->flavor->read_lock();
		md_rlock = true;
	}
	kd = ft_merge_descend(dst_ft, okey_dst, dst_key_len, &d_dst,
			&off_dst, &cnt_dst);
	MRG_SKIPCONF_PROBE(2, d_dst);
	/*
	 * Reanchor level-move: a peer chain-merge absorbed this slot's level
	 * into a longer compressed node while the descent walked it, so @d_dst
	 * names a position that has moved and every count and offset derived
	 * from it describes the old shape.  Insert and graft already bail on
	 * this; merge read it and continued.
	 *
	 * Re-descend down the SAME unwind the spine copy's contention path
	 * uses: nothing is built here (the label is above the read-lock pin,
	 * and the detach branch returned long before), so this is the state
	 * that path already returns to.  Release the pin first -- a retry
	 * re-reads, so it must re-pin.
	 */
	if (caa_unlikely(d_dst.skip_conflict)) {
		if (md_rlock) {
			dst_ft->group->flavor->read_unlock();
			/* Nothing moved: age the conflict, close the attempt. */
			ft_txn_attempt_bail(&optxn, true);
			md_rlock = false;
		}
		md_spin++;
		MRG_SPIN_PROBE(2);
		MRG_SPIN_MAX(md_spin);
		goto merge_spine_retry;
	}

	/*
	 * Atomic build-invisible spine-copy of @src_ft's subtree at @src_key into
	 * @dst_ft's non-empty subtree at @dst_key.  Each side is EXACT (@off == 0,
	 * the subtree at @d->nf) or KEY_SHORTER (@off > 0, the key ends inside a
	 * compressed node -- the src node is reclaimed by the commit's unlink, the
	 * dst node is wrapped under its prefix).  ft_rekey_spine_copy handles every
	 * such shape (internal, external, compressed and compressed-parent dst
	 * merge points); only a dst that is empty under @dst_key falls through to
	 * the detach-based graft below.
	 */
	if ((ks == FT_GRAFT_SWAP_EXACT || ks == FT_GRAFT_SWAP_KEY_SHORTER)
			&& cnt_dst > 0
			&& (kd == FT_GRAFT_SWAP_EXACT
				|| kd == FT_GRAFT_SWAP_KEY_SHORTER)) {
#ifndef FEATURE_FT_MERGE
		/*
		 * ONLY THE REKEY GRAFT IS AVAILABLE with merge compiled out.
		 * @cnt_dst > 0 is an OCCUPIED destination, i.e. a merge -- it
		 * must NOT fall through to the empty-dst graft below, which
		 * would attach into a subtree that already holds keys.  Refuse
		 * it terminally, the way ft_rekey_graft_simple_attempt answers
		 * the same shape (-ENOTSUP -> CDS_FT_STATUS_NOT_SUPPORTED).
		 */
		if (md_rlock) {
			dst_ft->group->flavor->read_unlock();
			urcu_txn_end(&optxn);
			md_rlock = false;
		}
		return CDS_FT_STATUS_NOT_SUPPORTED;
#else
		md_contended = false;
		status = ft_rekey_spine_copy(dst_ft, src_ft, &d_src,
				okey_src, src_key_len, cnt_src, off_src,
				&d_dst, cnt_dst, off_dst, dst_key_len,
				pre_txn, &md_contended);
		if (md_contended) {
			/* Contention, nothing moved: re-pin, re-descend, rebuild. */
			if (md_rlock) {
				dst_ft->group->flavor->read_unlock();
				/*
				 * AGE IT.  This is the arm that spun 348 deep with
				 * nothing to make it terminate: urcu_txn_conflict
				 * carries the retry count on the persistent handle, so
				 * the domain escalates this writer into the FIFO lane
				 * and the contention drains.  end() then FORFEITS the
				 * turn -- this is a pre-commit bail, and a bail that
				 * keeps its turn while the peer it waits on queues
				 * behind that same turn is the insert livelock.
				 */
				ft_txn_attempt_bail(&optxn, true);
				md_rlock = false;
			}
			md_spin++;
			MRG_SPIN_PROBE(1);
			if (src_ft->exclusive)
				MRG_SPIN_PROBE(4);
			MRG_SPIN_MAX(md_spin);
			goto merge_spine_retry;
		}
		if (status == CDS_FT_STATUS_OK) {
			/*
			 * Raise dst's max_used_key_len for the moved keys
			 * (dst_key_len + the longest stripped suffix), as the
			 * graft paths do -- later graft overflow validations
			 * feed off it.
			 */
			size_t src_max = uatomic_load(&src_ft->max_used_key_len,
					CMM_RELAXED);
			size_t nm = src_max > src_key_len ?
				dst_key_len + (src_max - src_key_len) :
				dst_key_len;
			size_t dm = uatomic_load(&dst_ft->max_used_key_len,
					CMM_RELAXED);

			if (nm > dm)
				uatomic_store(&dst_ft->max_used_key_len, nm,
					CMM_RELAXED);
		}
		if (md_rlock) {
			dst_ft->group->flavor->read_unlock();
			urcu_txn_end(&optxn);
			md_rlock = false;
		}
		FT_TP(merge_exit, (int) status);
		return status;
#endif /* FEATURE_FT_MERGE */
	}
	/*
	 * Fall-through: the spine-copy shape did not apply (whole-source move,
	 * or an empty dst under @dst_key -> the detach graft below).  Release the
	 * spine-copy descent pin here; the src_key_len==0 graft re-descends under
	 * its own bracket above, and the detach-graft path re-resolves its own
	 * (parent, slot).
	 */
	if (md_rlock) {
		dst_ft->group->flavor->read_unlock();
		urcu_txn_end(&optxn);
		md_rlock = false;
	}

	/*
	 * Whole-source move (src_key_len == 0): the entire @src_ft is the
	 * payload, so graft it directly -- no detach, no fallible rollback.
	 * ft_graft is itself leak-free: a diverge split is built invisibly and
	 * a NOSPLIT attach restores the saved old source root allocation-free
	 * on OOM, so an allocation failure leaves both tries pristine.  This
	 * covers an empty dst root and a diverged dst_key alike, and is the
	 * leak-free path for "rekey within a trie" (detach a sub-trie, then
	 * merge_at it at a new, currently-absent key in the same group).  A
	 * dst_key occupied by an empty-internal leftover yields POPULATED_ERROR
	 * here, exactly as the detach-then-graft path did.
	 */
	if (src_key_len == 0) {
		/*
		 * A live (non-exclusive) cross-trie source was already rejected with
		 * BUSY at merge_at's entry, so this graft sees an exclusive source (or
		 * a same-trie rekey); ft_graft_keylen runs its fused body directly.
		 */
		if (dst_ft->lock_fine && src_ft->exclusive) {
			const struct rcu_flavor_struct *flavor =
				dst_ft->group->flavor;

			/*
			 * §11 cross-trie RCU-pinning: this is the SAME live-dst
			 * descent + node lock as cds_ft_graft, reached through
			 * cds_ft_merge / cds_ft_merge_at, so it needs the same read-
			 * side bracket that pins descent-captured dst spine nodes
			 * against a peer relocate+free+recycle (see the bracket in
			 * cds_ft_graft).  With an EXCLUSIVE src the fused body takes
			 * no grace period (ft_writer_lock_gp_wait is !exclusive-
			 * gated), so the whole-op read lock cannot self-deadlock.
			 *
			 * That is the SAME condition, and the same reason, as the
			 * spine-copy pin above -- both branches read_lock() under
			 * exactly `dst_ft->lock_fine && src_ft->exclusive`.  What
			 * decides it is the SOURCE CONTRACT, not the branch: a
			 * cross-trie src is exclusive or the op was already refused
			 * with BUSY, and every grace period on that path is
			 * !exclusive-gated.  Only the same-trie rekey (src == dst,
			 * a LIVE src) actually runs those waits, and it is exactly
			 * the case this condition excludes from the read section.
			 */
			flavor->read_lock();
			status = ft_graft_keylen(dst_ft, dst_key, dst_key_len,
					src_ft, pre_txn);
			flavor->read_unlock();
		} else
			status = ft_graft_keylen(dst_ft, dst_key, dst_key_len,
					src_ft, pre_txn);
		FT_TP(merge_exit, (int) status);
		return status;
	}

	/*
	 * Empty dst ROOT (dst_key_len == 0, so the whole @dst_ft is empty at
	 * the merge point): move @src_ft@src_key in WITHOUT a fallible graft,
	 * so no rollback can strand the moved externals.  Pre-allocate the
	 * source's replacement root (fallible while @src_ft is still pristine),
	 * detach the source subtree (clean on its own failure), then SWAP it
	 * into @dst_ft's root.  The swap is a failure-free root-to-root move:
	 * no parent-pointer change and no "jump out" window (both ends are
	 * roots with parent NULL), and the detach already drained @src_ft's
	 * readers -- the same reasoning as ft_graft's root path.  Because
	 * nothing fallible follows the detach, this path has no rollback, so it
	 * cannot leak (contrast the diverged path below).
	 */
	if (dst_key_len == 0 && cnt_dst == 0) {
		struct cds_ft_inode *fresh_root;
		struct cds_ft_metadata *fresh_meta;
		struct cds_ft_metadata *dst_rmeta;
		struct cds_ft_inode_flag *dst_root_fenced;
		uintptr_t dst_root_snap;
		struct cds_ft_inode *old_dst_root;
		struct ft_flip_txn *appear_txn = NULL;
		size_t sm;
		int fence_ret;

		/*
		 * @cnt_dst was sampled far above, and an ENTIRE ft_detach_keylen of
		 * the source runs before the swap below -- the widest empty-dst
		 * window in the FT.  Re-decide emptiness UNDER the old root's
		 * node lock, so a contract-legal peer attach can no longer land
		 * inside that window and be freed with the root it landed on.  On a
		 * peer-populated dst, fall through to the DIVERGED path below (which
		 * merges into a populated destination) exactly as an up-front
		 * cnt_dst != 0 would have; on a held root, report BUSY.
		 */
		fence_ret = ft_root_attach_fence_empty(dst_ft, &dst_root_fenced,
			&dst_rmeta, &dst_root_snap, &optxn);
		if (fence_ret == -EEXIST)
			goto diverged;
		if (fence_ret) {
			status = CDS_FT_STATUS_BUSY_ERROR;
			goto out;
		}

		fresh_root = alloc_cds_ft_node(src_ft, &ft_types[0], &fresh_meta);
		if (!fresh_root) {
			ft_meta_lock_release(dst_rmeta);
			status = CDS_FT_STATUS_MEMORY_ERROR;
			goto out;
		}
		fresh_meta->parent_word = ft_trie_parent(src_ft);
		ft_nr_keys_store(src_ft, fresh_meta, 0, CMM_RELAXED);

		/*
		 * Pre-reserve the dst-appear root-swap txn BEFORE the detach: the
		 * swap is failure-free (post-detach) so it commits through this
		 * pre-reserved txn (ft_ord_cell_flip_into).  OOM here aborts while
		 * @src_ft is still pristine (no detach yet).  The dst old-root
		 * freeze-on-free tombstone rides the SAME flip (atomic detach,
		 * §4.B): +1 edge list-on; list-off is a 2-edge txn (root edge +
		 * tombstone) instead of the former lone ft_root_edge_flip store.
		 */
		if (dst_ft->group->ordered_list_set)
			appear_txn = ft_flip_txn_create_bounded(dst_ft,
				FT_ROOT_LIST_SWAP_MAX_EDGES + 1);
		else
			appear_txn = ft_flip_txn_create_bounded(dst_ft, 2);
		if (!appear_txn) {
			ft_meta_lock_release(dst_rmeta);
			free_cds_ft_node_unpublished(src_ft, fresh_root);
			status = CDS_FT_STATUS_MEMORY_ERROR;
			goto out;
		}

		status = ft_detach_keylen(src_ft, src_key, src_key_len, &subtree);
		if (status < 0) {
			/* NOT_FOUND impossible: @src_ft had content. */
			ft_meta_lock_release(dst_rmeta);
			if (appear_txn)
				ft_flip_txn_destroy(appear_txn);
			free_cds_ft_node_unpublished(src_ft, fresh_root);
			goto out;
		}

		/*
		 * Failure-free swap.  @subtree->root carries the moved content
		 * with parent already NULL (set by the detach), so it drops
		 * straight into @dst_ft's empty root slot.  @subtree keeps the
		 * pre-allocated empty root so its destroy below frees nothing of
		 * the moved content.  @dst_ft was empty, so it adopts @subtree's
		 * whole ordered cell list wholesale (head/tail endpoints only).
		 */
		/*
		 * The FENCED root, not a fresh read: under the fence the two are
		 * equal by construction, and using the fenced value keeps the
		 * retire, the swap's expected-old and the free naming ONE node.
		 */
		old_dst_root = ft_node_ptr(dst_root_fenced);
		/*
		 * Fuse the structural root swap with the ordered-list head/tail
		 * transfer into ONE flip (ft_root_list_swap_publish), so a reader
		 * never sees the moved keys reachable in @dst_ft's structure but
		 * its ordered list still empty -- the dst-appear cross-view window,
		 * closed the same way as the graft empty-dst path.  Only @dst_ft
		 * has concurrent readers here: @subtree is the fresh EXCLUSIVE trie
		 * the detach above just produced, so its root reset + head/tail
		 * clear are plain stores (no src-side disappear window, unlike
		 * ft_graft's cross-trie empty-dst).
		 *
		 * Freeze-on-free (doc §4.B): the dst old root this swap retires
		 * gets its tombstone recorded INTO the swap txn, so the mark and
		 * the unlink flip atomically (atomic detach).  Failure-free past
		 * the detach above, so this runs only on the committing path.
		 */
		/*
		 * @subtree->root becomes @dst_ft's root, so re-name its owner
		 * before either branch publishes it -- a cross-trie move leaves
		 * the back-edge naming the trie it came FROM, which is exactly
		 * the aliasing cds_ft_verify reports at depth 0.  @subtree is the
		 * fresh EXCLUSIVE detach product with no readers, so the store is
		 * safe ahead of the flip.
		 */
		cds_ft_item_to_metadata(ft_node_ptr(subtree->root))->parent_word =
			ft_trie_parent(dst_ft);
		if (dst_ft->group->ordered_list_set) {
			/*
			 * dst FILLS by adopting @subtree's whole list.  @subtree is the
			 * fresh EXCLUSIVE detach product (no readers), so its incoming run
			 * relinks to dst's sentinel in the SAME flip (relink_dest = subtree,
			 * relink_incoming = true); @subtree's sentinel resets to empty with
			 * a plain store.
			 */
			ft_flip_txn_lock_register(appear_txn, dst_rmeta,
				dst_root_snap);
			ft_flip_txn_record_tombstone_locked(appear_txn,
				dst_rmeta, dst_root_snap);
			ft_root_list_swap_publish(dst_ft, appear_txn, &dst_ft->root,
				dst_root_fenced, subtree->root,
				NULL, ft_ord_first(subtree),
				NULL, ft_ord_last(subtree), subtree, true);
			urcu_txn_list_init(&subtree->ord_sentinel);
		} else {
			/*
			 * No ordered list: dst's root is the only reader-visible
			 * structural slot.  Commit the appear root edge and the old-
			 * root tombstone as ONE 2-edge flip through the pre-reserved
			 * txn (a lone root edge would reduce to a release store, but
			 * the fused tombstone makes it multi-edge -- readers resolve
			 * the transient root proxy exactly as on the list-on path).
			 * (@subtree is the fresh EXCLUSIVE trie, so its root reset
			 * below stays a plain store.)
			 */
			ft_flip_txn_record_root(appear_txn,
				(void **) &dst_ft->root,
				(void *) dst_root_fenced, (void *) subtree->root);
			ft_flip_txn_lock_register(appear_txn, dst_rmeta,
				dst_root_snap);
			ft_flip_txn_record_tombstone_locked(appear_txn,
				dst_rmeta, dst_root_snap);
			ft_flip_txn_commit(dst_ft, appear_txn);
		}
		FT_TP(root_publish, (const void *) dst_ft,
			(const void *) dst_ft->root);
		subtree->root = ft_node_flag(fresh_root, 0);
		free_cds_ft_node(dst_ft, old_dst_root);

		/* dst_key_len == 0: dst keys equal the moved keys, same lengths. */
		sm = uatomic_load(&subtree->max_used_key_len, CMM_RELAXED);
		if (sm > uatomic_load(&dst_ft->max_used_key_len, CMM_RELAXED))
			uatomic_store(&dst_ft->max_used_key_len, sm, CMM_RELAXED);

		cds_ft_destroy(subtree);
		FT_TP(merge_exit, (int) CDS_FT_STATUS_OK);
		return CDS_FT_STATUS_OK;
	}

diverged:
	/*
	 * Sub-position source into a dst absent at @dst_key: graft the source
	 * subtree IN PLACE (no detach, no re-root) so the build-invisible cluster
	 * references @d_src.nf directly and the source unlink is the last fallible
	 * step -- the leak-free reorder.  Handles every source-root shape (plain
	 * multi-child internal, an internal carrying external_nodes, external,
	 * compressed, 1-child) at both a GLUE diverge and a NOSPLIT (at-node /
	 * build-branch) dst point.
	 *
	 * A KEY_SHORTER source (the src key ends @off_src bytes inside the
	 * compressed node @d_src.nf) reduces to the EXACT shape: the moved subtree
	 * is @cn_s->child, and its keys gain the residual bytes cn_s->key_bytes
	 * [off_src .. len) as a prefix, so graft @cn_s->child at @dst_key extended
	 * by that residual.  ft_merge_unlink_src_subtree overshoots @cn_s (preserves
	 * @cn_s->child) so the source side is identical; @cn_s itself is reclaimed
	 * by that unlink.
	 */
	if (ks == FT_GRAFT_SWAP_EXACT || ks == FT_GRAFT_SWAP_KEY_SHORTER) {
		bool handled;
		struct cds_ft_inode_flag *payload;
		const uint8_t *gokey = okey_dst, *gappkey = dst_key;
		size_t glen = dst_key_len;
		uint8_t ext_okey[FT_MAX_KEY_LEN], ext_app[FT_MAX_KEY_LEN];

		if (ks == FT_GRAFT_SWAP_EXACT) {
			payload = d_src.nf;	/* off_src == 0 by construction */
		} else {
			struct cds_ft_compressed_node *cn_s =
				ft_compressed_node_ptr(d_src.nf);
			unsigned int rlen = cn_s->len - off_src, j;
			const struct cds_ft_key_map *km =
				&dst_ft->group->key_map;

			payload = cn_s->child;
			memcpy(ext_okey, okey_dst, dst_key_len);
			memcpy(&ext_okey[dst_key_len], &cn_s->key_bytes[off_src],
				rlen);
			memcpy(ext_app, dst_key, dst_key_len);
			if (km->identity) {
				memcpy(&ext_app[dst_key_len],
					&cn_s->key_bytes[off_src], rlen);
			} else {
				for (j = 0; j < rlen; j++)
					ext_app[dst_key_len + j] =
						km->ordinal_to_key[
						cn_s->key_bytes[off_src + j]];
			}
			gokey = ext_okey;
			gappkey = ext_app;
			glen = dst_key_len + rlen;
		}

		status = ft_rekey_subpos_inplace(dst_ft, src_ft,
			gokey, gappkey, glen, okey_src, src_key_len,
			payload, cnt_src, &handled);
		if (handled) {
			FT_TP(merge_exit, (int) status);
			return status;
		}
	}

	/*
	 * Unreachable.  ft_merge_descend reports EXACT, KEY_SHORTER or DELEGATE;
	 * DELEGATE returned OK at the top (cnt_src == 0), and every EXACT /
	 * KEY_SHORTER source is handled above: a non-empty dst by the build-
	 * invisible spine copy, an empty dst root or whole-source move by their
	 * failure-free root swaps, and a diverged / absent dst point by
	 * ft_rekey_subpos_inplace (which always reports handled).  No shape
	 * falls through, so the old detach-then-graft fallback -- the last path
	 * that carried the double-OOM leak -- and its rollback are gone.
	 */
	assert(0 && "cds_ft_merge_at: unhandled merge shape");
	status = CDS_FT_STATUS_OK;
out:
	FT_TP(merge_exit, (int) status);
	return status;
}

static
enum cds_ft_status ft_rekey_dispatch(struct cds_ft *ft,
		const uint8_t *dst_key, size_t dst_key_len,
		const uint8_t *src_key, size_t src_key_len,
		enum ft_rekey_mode rekey)
{
	bool require_empty = (rekey == FT_REKEY_GRAFT);
	enum cds_ft_status status;

	if (!ft)
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	/*
	 * The atomic writer's shape-independent preconditions, checked BEFORE the
	 * gate so a trie that can never use it does not pay a grace period to be
	 * told so.  Everything else it decides for itself, from the structure.
	 *
	 * ★ THIS LIST ALSO CARRIES EVERY ARGUMENT CHECK THE ATOMIC WRITER DOES NOT
	 * REPEAT.  ft_merge_at_inner owns the entry contract -- NULL keys, a length
	 * past the group maximum, a SPECULATIVE trie (a move re-parents a leaf but
	 * cannot rewrite its app-owned stored key, so every moved key would lie) --
	 * and returns INVALID_ARGUMENT_ERROR for each.  Excluding them here rather
	 * than re-checking them keeps that contract in ONE place: a rejected
	 * argument falls through to the worker that defines the answer, and only a
	 * request that is VALID and merely outside the atomic cut is a fallback.
	 */
	/*
	 * ★ NO EQUAL-LENGTH TERM.  The one decide used to require it; the two
	 * lengths are now independent, and the combined-length OVERFLOW a longer
	 * destination can produce is answered where the rest of the entry contract
	 * lives -- the writer reports the shape as UNCOVERED and the fallback below
	 * returns OVERFLOW_ERROR.  Both lengths are still bounded here because a
	 * key longer than the group maximum is an ARGUMENT error, and this gate's
	 * job is only to keep the writer off requests it has no business deciding.
	 */
	bool one_decide =
		!ft->speculative_key_offset_active &&
		src_key && dst_key &&
		src_key_len != 0 && dst_key_len != 0 &&
		src_key_len <= ft->group->max_key_len &&
		dst_key_len <= ft->group->max_key_len;

	ft_move_gate_enter(ft);
#ifdef FT_RED_REKEY_NOLOCK
	ft_red_rekey_nolock = 1;	/* red control; see fractal-trie-internal.h */
#endif
	if (one_decide) {
		switch (ft_rekey_one_decide(ft, src_key, src_key_len, dst_key,
				dst_key_len, require_empty)) {
		case 0:
			status = CDS_FT_STATUS_OK;
			goto out;
		case -EEXIST:
			status = CDS_FT_STATUS_POPULATED_ERROR;
			goto out;
		case -ENOMEM:
			status = CDS_FT_STATUS_MEMORY_ERROR;
			goto out;
		case -ENOTSUP:
			/* Feature compiled out: no cut widens past that. */
			status = CDS_FT_STATUS_NOT_SUPPORTED;
			goto out;
		case -EINVAL:
			/*
			 * ARGUMENT error, now TERMINAL.  It used to fall into
			 * the fallback below together with the uncovered
			 * shapes, so a caller passing a zero-length or
			 * overlapping key had its mistake answered by STAGING
			 * a move instead of by a refusal.
			 */
			status = CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
			goto out;
		default:
			/*
			 * FT_REKEY_UNCOVERED only: the one code that may fall
			 * back, because it is the only one a wider cut closes.
			 */
			break;
		}
	}
	/*
	 * The staged fallback, now on the REKEY TWIN: src_ft == dst_ft here, so
	 * this path and the cross-trie ft_merge_at_inner no longer share a body
	 * whose src-side atomicity they disagree about.
	 */
	status = ft_rekey_at_inner(ft, dst_key, dst_key_len, ft,
			src_key, src_key_len, NULL, rekey);
out:
#ifdef FT_RED_REKEY_NOLOCK
	ft_red_rekey_nolock = 0;
#endif
	ft_move_gate_exit(ft);
	return status;
}

enum cds_ft_status cds_ft_rekey_graft(struct cds_ft *ft,
		const uint8_t *dst_key, size_t dst_key_len,
		const uint8_t *src_key, size_t src_key_len)
{
#ifdef FEATURE_FT_MERGE
	return ft_rekey_dispatch(ft, dst_key, dst_key_len, src_key, src_key_len,
			FT_REKEY_GRAFT);
#else
	(void) ft; (void) dst_key; (void) dst_key_len;
	(void) src_key; (void) src_key_len;
	return CDS_FT_STATUS_NOT_SUPPORTED;
#endif
}

enum cds_ft_status cds_ft_rekey_merge(struct cds_ft *ft,
		const uint8_t *dst_key, size_t dst_key_len,
		const uint8_t *src_key, size_t src_key_len)
{
#ifdef FEATURE_FT_MERGE
	return ft_rekey_dispatch(ft, dst_key, dst_key_len, src_key, src_key_len,
			FT_REKEY_MERGE);
#else
	(void) ft; (void) dst_key; (void) dst_key_len;
	(void) src_key; (void) src_key_len;
	return CDS_FT_STATUS_NOT_SUPPORTED;
#endif
}
