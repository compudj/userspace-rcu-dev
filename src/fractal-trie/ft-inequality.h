// SPDX-FileCopyrightText: 2012-2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-only

/*
 * src/fractal-trie/ft-inequality.h
 *
 * Userspace RCU library - Fractal Trie: inequality lookups: le / ge / lt / gt / first / last.
 *
 * Implementation unit: #included once by fractal-trie.c, in dependency
 * order, into a single translation unit (preserves cross-module inlining).
 * Not a standalone header.
 */
#ifndef FRACTAL_TRIE_IMPL
#error "ft-inequality.h is an implementation unit; #include it from fractal-trie.c only"
#endif

static inline_lookup
enum cds_ft_status cds_ft_lookup_inequality_impl(struct cds_ft *ft,
		struct cds_ft_iter *iter,
		enum ft_lookup_inequality mode,
		enum ft_lookup_limit limit,
		const bool use_keycopy,
		const bool seed_from_node)
{
	ssize_t key_depth, level;
	struct cds_ft_inode_flag *node_flag;
	struct cds_ft_node *ret_node;
	uint8_t ordinal_key[FT_MAX_KEY_LEN];
	/*
	 * @keep_ordinal: compile-time true for every instantiation EXCEPT
	 * (use_keycopy && limit == LIMIT_NONE).  In that one case the matched leaf
	 * is the sole result-key source (ft_speculative_keycopy_unconditional) and
	 * going-up dispatch reads input_key, so ordinal_key is dead: its memset,
	 * per-level fills (here and in the compressed helpers via fill_ordinal),
	 * byte-record output slots, and fallback copies all DCE.  @ord_scratch is
	 * the write-only sink for the going-up sibling / minmax byte-record on that
	 * path (ft_node_get_direction needs a valid output slot).
	 */
	const bool keep_ordinal = !(use_keycopy &&
			limit == FT_LOOKUP_LIMIT_NONE);
	uint8_t ord_scratch;
	enum ft_direction dir;
	const uint8_t *input_key = NULL;	/* set below; init for the hoisted cell-fastpath goto end */
	const uint8_t *iter_key;
	size_t key_len = 0;
	bool going_up = false, skip_eq_external_nodes;
	/*
	 * Parent-pointer going-up cursor (structural up_node form,
	 * mirroring the iter_skip walk-up).  @up_node is the deepest live
	 * node descent established; @up_node_lo is the shallowest depth it
	 * covers -- its TRUE shallow boundary (several levels below @up_node
	 * for a compressed run).  Tracked as descent proceeds so the going-up
	 * seed is live (taken from the descent, not read back from a
	 * recorded path), then climbed via ft_get_parent_rcu up to the
	 * going-up @level.  Seeding from the
	 * deepest node + true shallow boundary (rather than from @node_flag
	 * and the end-of-key-adjusted @level) avoids both the level
	 * adjustment desync and the compressed-span boundary ambiguity.
	 */
	struct cds_ft_inode_flag *up_node = NULL;
	ssize_t up_node_lo = 0;
	/*
	 * @use_keycopy is a compile-time literal at each instantiation (see the
	 * two cds_ft_lookup_inequality_impl callers in the dispatcher below), so
	 * always_inline constant-folds every per-call gate on it.  When set, a
	 * configured speculative skip-compressed group recovers the result key
	 * from the matched leaf and the min-descent follows skip pointers without
	 * reading the compressed node or filling ordinal_key; otherwise the
	 * descent rebuilds ordinal_key from the live compressed nodes (the fill +
	 * re-anchor path).
	 */

	CDS_FT_ASSERT_RCU_READ_LOCKED(ft);

	/*
	 * Ordinal-cell fast path: cds_ft_next / cds_ft_prev (GT/LT,
	 * LIMIT_NONE) on a cached head collapse to a single dependent load of the
	 * cell's ord_next / ord_prev -- no descent, no leaf touch for the step
	 * (cell->node + cell->ord_* co-reside in the 32B cell).  Hoisted ABOVE the
	 * key_len computation so the common walk does NOT resolve the (deferred,
	 * LAZY) length: the fast path advances via ord_next and never needs it.
	 * Only a fall-through to the descent (cache invalid / scoped) resolves the
	 * length where it is first used.
	 *
	 * @seed_from_node (a compile-time literal at every public instantiation, so
	 * the gate DCEs there) suppresses this fast path for the splice-time
	 * predecessor seed: that caller positions @iter at a FRESH head whose cell
	 * is not yet spliced (its ord_prev/ord_next are unset), so the cell cursor
	 * would resolve garbage.  It instead wants the cross-call node-recovery fast
	 * path below, which reconstructs the going-up seed from iter->node and runs
	 * the structural backtrack -- the predecessor among the ALREADY-linked keys.
	 */
	if (!seed_from_node &&
			(mode == FT_LOOKUP_GT || mode == FT_LOOKUP_LT) &&
			limit == FT_LOOKUP_LIMIT_NONE &&
			iter->cache_valid && iter->node &&
			ft_ord_cell_fastpath_ok(ft, iter)) {
		struct ft_ord_cell *cur = ft_ord_cell_cursor(iter);
		struct ft_ord_cell *nxt = (mode == FT_LOOKUP_GT) ?
			ft_ord_cell_resolve_ord(&cur->ord_next) :
			ft_ord_cell_resolve_ord(&cur->ord_prev);

		ft_ord_cell_iter_land(ft, iter, nxt);
#ifndef FT_NO_ORD_PREFETCH
		/* One-hop NTA prefetch of the cell the next call will land on. */
		if (nxt) {
			struct ft_ord_cell *nn = (mode == FT_LOOKUP_GT) ?
				ft_ord_cell_resolve_ord(&nxt->ord_next) :
				ft_ord_cell_resolve_ord(&nxt->ord_prev);
			if (nn)
				__builtin_prefetch((const void *) nn, 0, 0);
		}
#endif
		goto end;
	}

	switch (limit) {
	case FT_LOOKUP_LIMIT_NONE:
		/*
		 * Continuation from the current position: its length may be the
		 * deferred LAZY sentinel (a cell walk that fell to the descent on a
		 * scoped step) -- resolve it from the leaf.  No-op for fixed /
		 * descent / set_key positions.
		 */
		key_len = ft_key_len(ft, ft_iter_resolve_key_len(iter));
		if (!valid_key_len(ft, key_len)) {
			iter->node = NULL;
			iter->cache_valid = false;
			iter_debug_path_update(iter);
			iter->status = CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
			goto end;
		}
		break;
	case FT_LOOKUP_LIMIT_FIRST:
		key_len = iter->prefix_len;
		break;
	case FT_LOOKUP_LIMIT_LAST:
		key_len = ft->group->max_key_len;
		break;
	}

	key_depth = key_len + 1;

	switch (mode) {
	case FT_LOOKUP_GE:
	case FT_LOOKUP_LE:
	case FT_LOOKUP_GT:
	case FT_LOOKUP_LT:
		break;
	default:
		abort();	/* Internal library error. */
	}

	/*
	 * Read the input key in place from the iterator buffer -- no snapshot
	 * copy.  Every write to iter_key(iter) (the result) is terminal (goto
	 * end) or sourced from the separate ordinal_key / leaf-copy buffer, and
	 * all input reads (the descent and the going-up backtrack) precede any
	 * terminal write, so the input bytes are never overwritten while still
	 * needed.  The lone self-aliasing case -- the equal-match write below --
	 * is a no-op and is guarded.
	 *
	 * Input-key source: only a LIMIT_NONE continuation (cds_ft_next/prev, the
	 * relational lookups) iterates from the iterator's CURRENT key, which on a
	 * lazy-ref cell group is read IN PLACE from the live node (no prior copy).
	 * LIMIT_FIRST/LIMIT_LAST are absolute and key off the prefix in iter_key,
	 * NOT the current position, so they must NOT reference the node (a reused
	 * iterator's stale node would mis-seed the search).
	 */
	if (limit == FT_LOOKUP_LIMIT_NONE)
		input_key = ft_iter_read_key(iter);
	else
		input_key = iter_key(iter);
	iter_key = input_key;

	FT_TP(ineq_enter, (int) mode, input_key, key_len);

	if (keep_ordinal)
		memset(ordinal_key, 0, ft->group->max_key_len * sizeof(ordinal_key[0]));
	node_flag = ft_root_dereference_prefetch(ft);
	up_node = node_flag;		/* root covers depth 0 */
	up_node_lo = 0;

	/*
	 * Empty root short-circuit: when the root has no children,
	 * there is nothing to traverse and no inequality match is
	 * possible. An empty root is always a type-0 popcount_2l node
	 * with an all-zero bitmap header.  Compressed roots are never
	 * empty (recompaction replaces emptied compressed roots with
	 * internal nodes).
	 */
	if (!ft_node_compressed(node_flag)) {
		unsigned int type_idx = ft_node_type(node_flag);
		const struct cds_ft_type *type = &ft_types[type_idx];
		struct cds_ft_metadata *metadata = cds_ft_item_to_metadata_fast(
				ft_node_ptr(node_flag), type->order);

		/*
		 * Empty trie: no internal children and no NIL-key entries.
		 * Loading nr_child instead of probing the bitmap works for
		 * every internal node class.
		 */
		if (metadata->nr_child == 0 &&
				!uatomic_load(&metadata->external_nodes, CMM_RELAXED)) {
			iter->node = NULL;
			iter->cache_valid = true;
			iter_debug_path_snapshot(iter);
			iter->path_len = 1;
			iter->status = CDS_FT_STATUS_NOT_FOUND;
			goto end;
		}
	}

	/*
	 * Fast path: reuse the iterator's cached position from a prior
	 * lookup when it is still valid and covers the full key
	 * depth.  This avoids redundant per-level ft_node_get_nth(ft, )
	 * lookups (which are the expensive, cache-miss-prone part of
	 * the downward walk).  The caller must hold the RCU read-side
	 * lock continuously for the cached pointers to remain valid.
	 */
	iter_debug_path_check(iter);
	/*
	 * Continuation fast path: recover the position from iter->node by
	 * backtracking up the live parent chain.  Reuse it only when the
	 * cached position is EXACTLY at this key (path_len == key_depth and
	 * iter->node set) -- iter->node is then the deepest position for the
	 * key.  A deeper cached position (path_len > key_depth, e.g. an
	 * inequality result that landed on a longer key) is not the right
	 * cursor for this key and falls to slow_path.  The caller must hold
	 * the RCU read-side lock continuously for iter->node to stay valid.
	 */
	if (iter->cache_valid && iter->node &&
			(ssize_t)iter->path_len == key_depth &&
			key_depth > 1) {
		for (level = 1; level < key_depth; level++) {
			if (!keep_ordinal)
				continue;
			switch (limit) {
			case FT_LOOKUP_LIMIT_NONE:
				ordinal_key[level - 1] =
					input_key[level - 1];
				break;
			case FT_LOOKUP_LIMIT_FIRST:
				ordinal_key[level - 1] =
					input_key[level - 1];
				break;
			case FT_LOOKUP_LIMIT_LAST:
				if ((size_t) level <= iter->prefix_len)
					ordinal_key[level - 1] =
						input_key[level - 1];
				else
					ordinal_key[level - 1] = 0xff;
				break;
			}
		}
		{
			/*
			 * Cross-call continuation: recover the deepest trie node
			 * for iter->key from the cached position iter->node (the
			 * dup-chain head), not from a recorded descent path.  A
			 * prefix key sits at an internal/compressed holder whose
			 * external_nodes == iter->node; otherwise iter->node is a
			 * leaf child (a tag-0 external flag).  ft_get_parent_rcu
			 * on the head reaches the holder in O(1) (head->prev ==
			 * holder) and never walks the dup chain.
			 */
			struct cds_ft_inode_flag *cur =
				(struct cds_ft_inode_flag *) iter->node;
			struct cds_ft_inode_flag *holder =
				ft_get_parent_rcu(ft, cur);

			if (holder && !ft_node_external(holder) &&
			    ft_node_external_nodes(holder) ==
					(struct cds_ft_node *) iter->node)
				node_flag = holder;
			else
				node_flag = cur;
		}
		/*
		 * If the cached path entry is a compressed node, the
		 * fast path cannot determine the correct loop exit
		 * state (compressed nodes span multiple levels).
		 * Fall back to the slow path.
		 */
		if (ft_node_compressed(node_flag) ||
		    ft_node_skip_compressed(node_flag)) {
			node_flag = ft_root_dereference_prefetch(ft);
			iter_key = input_key;
			goto slow_path;
		}
		/*
		 * Reconstruct the loop exit value of @level to match
		 * what the traversal loop would have produced:
		 *  - key_depth     if last node is internal (loop ran
		 *                  to completion),
		 *  - key_depth - 1 if last node is external, NULL, or
		 *                  the path ended early (loop broke).
		 */
		if (!ft_node_external(node_flag))
			level = key_depth;
		else
			level = key_depth - 1;
		/*
		 * Cross-call fast path: the cached deepest node is the going-up
		 * seed.  It is never compressed/skip here (those fall back to
		 * slow_path above), so its shallow boundary is its own depth.
		 */
		up_node = node_flag;
		up_node_lo = key_depth - 1;
		FT_TP(fastpath_enter, (int) mode,
			(const void *) node_flag, (int) level);
		goto post_traversal;
	}

slow_path:
	FT_TP(slowpath_enter, (int) mode, (int) iter->cache_valid,
		(int) iter->path_len);
	for (level = 1; level < key_depth; level++) {
		uint8_t key_value;

		if (ft_node_compressed(node_flag)) {
			enum ft_descent_action act;
			ssize_t cmp_entry_level = level;

			act = ft_inequality_compressed(&node_flag,
				&level, key_depth, mode, limit,
				&iter_key, input_key, iter,
				ordinal_key, &skip_eq_external_nodes,
				keep_ordinal);
			if (act == FT_DESCENT_GOING_UP) {
				/*
				 * @node_flag is the compressed node; it occupies
				 * depths [cmp_entry_level-1, +cn->len).  Seed the
				 * cursor at its TRUE shallow boundary so the
				 * going-up climb reaches its internal parent.
				 */
				up_node = node_flag;
				up_node_lo = cmp_entry_level - 1;
				goto going_up;
			}
			/*
			 * Descend / break / continue: @node_flag is cn->child
			 * at the advanced @level (span 1 -- chain-merge forbids
			 * compressed-under-compressed).
			 */
			up_node = node_flag;
			up_node_lo = level;
			if (act == FT_DESCENT_DESCEND_CHILDREN)
				goto descend_children;
			if (act == FT_DESCENT_BREAK)
				break;
			if (level + 1 >= key_depth) {
				skip_eq_external_nodes = false;
				/*
				 * A compressed full-match consumed the whole key and
				 * landed on cn->child (which may hold this key as a
				 * prefix-key external).  For GE/GT (RIGHT) descend into
				 * that subtree: its leftmost is the smallest key >= the
				 * search key.  But for LE/LT (LEFT) every key in the
				 * subtree is >= the search key, so descending would
				 * return the subtree MAX, which is strictly greater --
				 * wrong.  Break to post_traversal instead, exactly as the
				 * non-compressed path does: it returns cn->child's own
				 * external_nodes for LE (the equal match) and climbs for
				 * LT's strict predecessor.
				 */
				if (mode == FT_LOOKUP_LE || mode == FT_LOOKUP_LT)
					break;
				goto descend_children;
			}
			continue;
		}

		switch (limit) {
		case FT_LOOKUP_LIMIT_NONE:
			key_value = *(iter_key++);
			break;
		case FT_LOOKUP_LIMIT_FIRST:
			key_value = input_key[level - 1];
			break;
		case FT_LOOKUP_LIMIT_LAST:
			if ((size_t) level <= iter->prefix_len)
				key_value = input_key[level - 1];
			else
				key_value = 0xff;
			break;
		}
		/*
		 * Record ordinal_key[level - 1] BEFORE the NULL-break so the
		 * going-up phase has the dispatch byte even for a dead-end
		 * descent.  The parent-pointer walk keys off ordinal_key
		 * (iter_key desyncs once level steps by span > 1), so this
		 * must be set regardless of whether descent continues.
		 */
		if (keep_ordinal)
			ordinal_key[level - 1] = key_value;
		{
			unsigned int rewind;

			/*
			 * Surgical re-anchor on a skip-compressed mismatch (no
			 * spin on a frozen slot).  rewind > 0: a concurrent
			 * chain-merge moved the encoded position shallower;
			 * re-descend the fixed key path from the root, exactly
			 * like the fast-path fallback above.
			 */
			node_flag = ft_node_get_nth_reanchor(ft, node_flag,
					key_value, &rewind);
			if (caa_unlikely(rewind)) {
				node_flag = ft_root_dereference_prefetch(ft);
				iter_key = input_key;
				goto slow_path;
			}
		}
		if (!node_flag) {
			FT_TP(slowpath_step, (int) level, key_value,
				(const void *) node_flag, 1);
			break;
		}
		up_node = node_flag;		/* child established at @level (span 1) */
		up_node_lo = level;
		FT_TP(slowpath_step, (int) level, key_value,
			(const void *) node_flag, 0);
		dbg_printf("cds_ft_lookup_inequality iter key lookup %u finds node_flag %p\n",
				(unsigned int) key_value, node_flag);
		if (ft_node_external(node_flag))
			break;
	}

	/*
	 * The slow-path traversal has freshly populated the iterator
	 * path.  Capture a grace-period snapshot so that subsequent
	 * check() calls can validate this new path.  The fast path
	 * (goto post_traversal above) skips this: its cached path was
	 * already validated by iter_debug_path_check().
	 */
	iter_debug_path_snapshot(iter);

post_traversal:
	FT_TP(post_traversal, (int) mode, (int) level,
		(const void *) node_flag);
	ft_delay_reader();
	/*
	 * If the descent consumed the whole key, normalize @level to the
	 * end-of-key depth (key_depth - 1) BEFORE the equal-match check below.
	 * The loop overshoots by one when the key terminates on an INTERNAL
	 * node (level reaches key_depth) rather than breaking on an external
	 * leaf (level == key_depth - 1): an internal node that holds the key as
	 * a prefix-key external (it also has a longer-key subtree) is the exact
	 * match, and the LE/GE arm's ft_node_internal branch returns its
	 * external_nodes.  Without this normalization that arm is skipped and
	 * the exact prefix-key match is missed (the non-compressed analogue of
	 * the compressed full-match handled in the slow-path loop above).
	 */
	if (level >= key_depth)
		level = key_depth - 1;
	switch (mode) {
	case FT_LOOKUP_LE:
	case FT_LOOKUP_GE:
		if (level == key_depth - 1) {
			struct cds_ft_node *external_nodes;

			if (ft_node_internal(node_flag)) {
				struct cds_ft_metadata *metadata;
				const struct cds_ft_type *type = &ft_types[ft_node_type(node_flag)];

				metadata = cds_ft_item_to_metadata_fast(ft_node_ptr(node_flag), type->order);
				external_nodes = ft_dereference_external(metadata->external_nodes);
			} else if (ft_node_compressed(node_flag)) {
				struct cds_ft_metadata *metadata =
					cds_ft_item_to_metadata(ft_node_ptr(node_flag));
				external_nodes = ft_dereference_external(metadata->external_nodes);
			} else {
				external_nodes = (struct cds_ft_node *) node_flag;
			}
			if (external_nodes) {
				/* End of key lookup succeded. We got an equal match.
				 * The result key equals the input, which is read in
				 * place from iter_key(iter), so the copy is a no-op
				 * self-copy unless a separate buffer is in use.
				 * memmove: @input_key may be the up-walk key at this
				 * same buffer's TAIL (iter_key + key_off), which the
				 * front write overlaps. */
				iter->key_len = key_len;
				iter->key_off = 0;
				if (!ft_speculative_keycopy(ft, external_nodes,
						iter_key(iter), (ssize_t) key_len) &&
						input_key != iter_key(iter))
					memmove(iter_key(iter), input_key, key_len);
				iter->node = external_nodes;
				iter->cache_valid = true;
				iter_debug_path_update(iter);
				iter->path_len = level + 1;
				iter->status = CDS_FT_STATUS_OK;
				goto end;
			}
		}
		break;
	case FT_LOOKUP_LT:
	case FT_LOOKUP_GT:
		break;
	default:
		assert(0);
	}

	/*
	 * LE/LT dead-ended at an external LEAF that is a PROPER PREFIX of the
	 * search key: the descent matched the leaf's whole key (depth @level)
	 * but the search key is longer, so leaf_key < search_key.  A leaf has
	 * no extensions, so no key lies strictly between it and the search key
	 * on this path -- it is the largest key <= the search key, larger than
	 * any left-sibling branch (which diverges at a smaller byte).  going_up
	 * skips external leaves (it only returns internal/compressed nodes'
	 * external_nodes), so return the leaf here.  The exact-length match
	 * (@level == key_depth - 1) is already handled by the LE/GE arm above
	 * (LE) or excluded by strictness (LT), so this is only the proper-prefix
	 * case.
	 *
	 * node_flag may be NULL here (the descent's empty-slot break), which
	 * ft_node_external() also matches: that is a mid-key dead-end, NOT a
	 * proper-prefix leaf -- fall through to going_up, which backtracks to
	 * the nearest lesser key.
	 */
	if (limit == FT_LOOKUP_LIMIT_NONE &&
			(mode == FT_LOOKUP_LE || mode == FT_LOOKUP_LT) &&
			node_flag && ft_node_external(node_flag) &&
			level < key_depth - 1) {
		int j;

		assert(level <= (int) ft->group->max_key_len);
		iter->key_len = level;
		iter->key_off = 0;
		if (!keep_ordinal)
			ft_speculative_keycopy_unconditional(ft,
				(const struct cds_ft_node *) ft_node_ptr(node_flag),
				iter_key(iter), level);
		else if (!ft_speculative_keycopy(ft,
				(const struct cds_ft_node *) ft_node_ptr(node_flag),
				iter_key(iter), level)) {
			for (j = 0; j < level; j++)
				iter_key(iter)[j] = ordinal_key[j];
		}
		iter->node = (struct cds_ft_node *) ft_node_ptr(node_flag);
		iter->cache_valid = true;
		iter_debug_path_update(iter);
		iter->path_len = level + 1;
		iter->status = CDS_FT_STATUS_OK;
		goto end;
	}

	/*
	 * For GE/GT: if the descent completed and the node at end-of-key
	 * is internal, any descendant key is strictly longer and therefore
	 * strictly greater. Skip backtracking and descend into children
	 * directly.
	 *
	 * For GE, the post-traversal above already returned if the node
	 * had external_nodes (the equal match). Reaching this point means
	 * no equal match exists, so descendant keys are the closest >=.
	 *
	 * For GT, the skip_eq_external_nodes flag (set below) will
	 * prevent the minmax descent from returning this node's own
	 * external_nodes (which are the equal match, not GT).
	 *
	 * LE/LT do not need this: their upward backtracking already
	 * checks external_nodes at each internal node going up, which is
	 * the correct direction to find shorter (lesser) prefix keys.
	 */
	if ((mode == FT_LOOKUP_GT || mode == FT_LOOKUP_GE) &&
			!ft_node_external(node_flag))
		goto descend_children;

	/*
	 * LE/LT, relational (LIMIT_NONE), with the search key exhausted at or
	 * above the prefix boundary: the empty key (level == prefix_len == 0), or
	 * a query equal to the scope prefix.  The going-up loop below would not
	 * run (no shorter key to back-track to) and control would fall through to
	 * descend_children, wrongly returning the subtree MAX.  No key is strictly
	 * less than the boundary; the only <= match is the boundary node's own
	 * external_nodes, which LE returns and LT excludes -> NOT_FOUND otherwise.
	 * (LIMIT_FIRST/LAST want the subtree min/max here, so they are excluded.)
	 */
	if (limit == FT_LOOKUP_LIMIT_NONE &&
			(mode == FT_LOOKUP_LE || mode == FT_LOOKUP_LT) &&
			level <= (ssize_t) iter->prefix_len && node_flag &&
			!ft_node_external(node_flag)) {
		struct cds_ft_node *ext = NULL;

		if (mode == FT_LOOKUP_LE) {
			struct cds_ft_metadata *m;

			if (ft_node_compressed(node_flag))
				m = cds_ft_item_to_metadata(ft_node_ptr(node_flag));
			else
				m = cds_ft_item_to_metadata_fast(ft_node_ptr(node_flag),
					ft_types[ft_node_type(node_flag)].order);
			ext = ft_dereference_external(m->external_nodes);
		}
		if (ext) {
			int j;

			iter->key_len = iter->prefix_len;
			iter->key_off = 0;
			if (!keep_ordinal)
				ft_speculative_keycopy_unconditional(ft, ext,
					iter_key(iter), (ssize_t) iter->prefix_len);
			else if (!ft_speculative_keycopy(ft, ext, iter_key(iter),
					(ssize_t) iter->prefix_len)) {
				for (j = 0; j < (int) iter->prefix_len; j++)
					iter_key(iter)[j] = ordinal_key[j];
			}
			iter->node = ext;
		} else {
			iter->node = NULL;
		}
		iter->cache_valid = true;
		iter_debug_path_update(iter);
		iter->path_len = iter->prefix_len + 1;
		iter->status = ext ? CDS_FT_STATUS_OK : CDS_FT_STATUS_NOT_FOUND;
		goto end;
	}

going_up:
	/* Ensure iter_key is exactly at the position matching the level we stopped at. */
	iter_key = input_key + level;

	/*
	 * Find highest value left/right of current node.
	 * Start at current level. If we cannot find any key left/right
	 * of ours, go one level up, seek highest value left/right of
	 * current (recursively), and when we find one, get the
	 * rightmost/leftmost child of its rightmost/leftmost child
	 * (recursively).
	 *
	 * Prefix-scoped traversal: backtracking stops at
	 * iter->prefix_len instead of 0, confining the search to the
	 * prefix subtree. When prefix_len == 0 this is identical to
	 * the original behavior.
	 */
	switch (mode) {
	case FT_LOOKUP_LE:
	case FT_LOOKUP_LT:
		dir = FT_LEFT;
		break;
	case FT_LOOKUP_GE:
	case FT_LOOKUP_GT:
		dir = FT_RIGHT;
		break;
	default:
		assert(0);
	}
	/*
	 * Parent-pointer going-up cursor.  The node at @level and the
	 * dispatcher at @level-1 (scanned for a sibling) are obtained via live
	 * back-pointer reads,
	 * which may observe a fresher version than the descent snapshot
	 * (intentional -- freshness -- and semantically valid).
	 *
	 * Anchor on @up_parent = node at depth @level-1: it is always valid
	 * for level > prefix_len (descent filled it), unlike node-at-level
	 * which is NULL on a dead-end descent.  @up_parent_lo is the shallowest
	 * depth @up_parent covers (> 1 for a compressed run).  As the loop
	 * decrements @level, climb @up_parent to its own parent via
	 * ft_get_parent_rcu once @level-1 drops below @up_parent_lo, accumulating
	 * span (compressed = cn->len, internal = 1).  @up_node (node at @level)
	 * is carried from the previous iteration's @up_parent -- the node we
	 * just climbed past -- so it is never derived by dereferencing a
	 * possibly-NULL node-at-level.  It is read only inside the LE/LT block
	 * below, which the first iteration (going_up == false) skips.
	 */
	{	/* Scope the going-up cursor locals so they fall out of scope before descend_children/end (avoids -Wjump-misses-init false positives). */
		struct cds_ft_inode_flag *up_parent;
		ssize_t up_parent_lo;
		bool up_first = true;

		/*
		 * Climb @up_node (the deepest live node descent established) up to
		 * the node covering the going-up @level: the loop @level may have
		 * been knocked back one by the end-of-key adjustment, and a
		 * compressed run spans several levels.  @up_node then is node-at-level
		 * (read by the LE/LT block, carried by the loop).
		 */
		while (level < up_node_lo) {
			struct cds_ft_inode_flag *gp = ft_get_parent_rcu(ft, up_node);

			up_node_lo -= (gp && ft_node_compressed(gp)) ?
				(ssize_t) ft_compressed_node_ptr(gp)->len : 1;
			up_node = gp;
		}
		/*
		 * Derive @up_parent = node at @level-1 (the dispatcher scanned for a
		 * sibling).  If @up_node already covers @level-1 it IS the dispatcher
		 * -- a compressed run (no within-span sibling, skipped by the
		 * !ft_node_internal test below) or a dead-end whose dispatcher we
		 * never descended past.  Otherwise it is @up_node's parent, one depth
		 * shallower (parenthood is by slot).
		 */
		if (up_node_lo <= level - 1) {
			up_parent = up_node;
			up_parent_lo = up_node_lo;
		} else {
			up_parent = ft_get_parent_rcu(ft, up_node);
			up_parent_lo = (up_parent && ft_node_compressed(up_parent)) ?
				level - (ssize_t) ft_compressed_node_ptr(up_parent)->len :
				level - 1;
		}
		for (; level > (ssize_t) iter->prefix_len; level--) {
			uint8_t key_value;

			ft_delay_reader();
			if (!up_first) {
				/*
				 * Climbed one level (level-- since last iter): the old
				 * dispatcher becomes the new node-at-level, and the new
				 * dispatcher is its parent once we cross below its span.
				 */
				up_node = up_parent;
				if (level - 1 < up_parent_lo) {
					struct cds_ft_inode_flag *gp =
						ft_get_parent_rcu(ft, up_parent);

					up_parent_lo -= (gp && ft_node_compressed(gp)) ?
						(ssize_t) ft_compressed_node_ptr(gp)->len : 1;
					up_parent = gp;
				}
			}
			up_first = false;
			/*
			 * Return external node if trying to find LE/LT
			 * inequality and encountering an external node when
			 * going upward.
			 */
			if (going_up && dir == FT_LEFT &&
			    !ft_node_external(up_node)) {
				struct cds_ft_metadata *metadata;

				if (ft_node_compressed(up_node))
					metadata = cds_ft_item_to_metadata(
						ft_node_ptr(up_node));
				else {
					const struct cds_ft_type *type = &ft_types[ft_node_type(up_node)];
					metadata = cds_ft_item_to_metadata_fast(
						ft_node_ptr(up_node),
						type->order);
				}
				{
				struct cds_ft_node *external_nodes = ft_dereference_external(metadata->external_nodes);

				if (external_nodes) {
					int j;

					assert(level <= (int) ft->group->max_key_len);
					iter->key_len = level;
					iter->key_off = 0;
					if (!keep_ordinal)
						ft_speculative_keycopy_unconditional(ft,
							external_nodes, iter_key(iter), level);
					else if (!ft_speculative_keycopy(ft, external_nodes,
							iter_key(iter), level)) {
						for (j = 0; j < level; j++)
							iter_key(iter)[j] = ordinal_key[j];
					}
					iter->node = external_nodes;
					iter->cache_valid = true;
					iter_debug_path_update(iter);
					iter->path_len = level + 1;
					iter->status = CDS_FT_STATUS_OK;
					goto end;
				}
				}
			}

			switch (limit) {
			case FT_LOOKUP_LIMIT_NONE:
				key_value = *(--iter_key);
				break;
			case FT_LOOKUP_LIMIT_FIRST:
				key_value = input_key[level - 1];
				break;
			case FT_LOOKUP_LIMIT_LAST:
				if ((size_t) level <= iter->prefix_len)
					key_value = input_key[level - 1];
				else
					key_value = 0xff;
				break;
			}
			/*
			 * Standard sibling lookup. Parent is level - 1. We are
			 * looking for sibling of the byte at ordinal_key[level - 1].
			 * Skip levels where the path entry is not an internal node
			 * (compressed or external entries from compressed path
			 * traversal have no siblings).
			 */
			if (!ft_node_internal(up_parent)) {
				FT_TP(ineq_going_up_step, level,
					(const void *) up_parent,
					0, (uint8_t) key_value);
				going_up = true;
				continue;
			}
			node_flag = ft_node_get_leftright(ft, up_parent, key_value,
					keep_ordinal ? &ordinal_key[level - 1] : &ord_scratch,
					dir, true /* validate_lookup */);
	#ifdef FEATURE_FT_SKIP_COMPRESSED
			/*
			 * Fill path only (!use_keycopy): a skip-encoded sibling must be
			 * resolved to fill ordinal_key.  Re-anchor the scanned parent on
			 * the live structure via the child's parent chain and re-scan it
			 * (the single skip concurrency mechanism), rather than spin.
			 * @rewind > 0 (merge) means the parent merged shallower: drop
			 * @level and recompute the dispatch byte at the new level.
			 * ft_skip_reanchor never returns NULL on a well-formed trie.
			 *
			 * Under use_keycopy the skip-encoded sibling is left raw and
			 * followed by the descend_children skip-follow (the leaf copy
			 * supplies the spanned bytes), so no re-anchor is needed here.
			 */
			while (!use_keycopy && node_flag &&
					caa_unlikely(ft_node_skip_compressed(node_flag))) {
				unsigned int rewind;
				struct cds_ft_inode_flag *at_pos;
				struct cds_ft_inode_flag *anchor =
					ft_skip_reanchor(ft, node_flag, &rewind, &at_pos);

				assert(anchor != NULL);
				if (caa_likely(rewind == 0)) {
					/*
					 * Surgical: @at_pos is the live resolved sibling at
					 * this depth that re-scanning @anchor for the
					 * unchanged dispatch byte ordinal_key[level - 1] would
					 * find -- the sibling byte was already recorded by the
					 * ft_node_get_leftright above, so use @at_pos directly
					 * instead of re-scanning.  Keep the path's parent entry
					 * live.
					 */
					node_flag = at_pos;
					break;
				}
				/*
				 * @rewind > 0 (merge): the parent merged shallower; drop
				 * @level and re-scan the live parent at the new level.
				 */
				level -= (ssize_t) rewind;
				/*
				 * The merge rewound @level and re-anchored the live
				 * holder at the new level-1.  Re-sync the parent cursor to
				 * @anchor (an internal node, span 1).  If the re-scan below
				 * finds no sibling, the for-loop's level-- then carries
				 * up_node = anchor = node at the new level, keeping the walk
				 * consistent with the rewound descent position.
				 */
				up_parent = anchor;
				up_parent_lo = level - 1;
				switch (limit) {
				case FT_LOOKUP_LIMIT_NONE:
					key_value = ordinal_key[level - 1];
					break;
				case FT_LOOKUP_LIMIT_FIRST:
					key_value = input_key[level - 1];
					break;
				case FT_LOOKUP_LIMIT_LAST:
					key_value = ((size_t) level <= iter->prefix_len) ?
						input_key[level - 1] : (uint8_t) 0xff;
					break;
				}
				node_flag = ft_node_get_leftright(ft, anchor, key_value,
						&ordinal_key[level - 1], dir,
						true /* validate_lookup */);
			}
	#endif
			if (keep_ordinal)
				dbg_printf("cds_ft_lookup_inequality find sibling from %u at %u finds node_flag %p\n",
						(unsigned int) key_value, (unsigned int) ordinal_key[level - 1],
						node_flag);
			else
				dbg_printf("cds_ft_lookup_inequality find sibling from %u finds node_flag %p\n",
						(unsigned int) key_value, node_flag);
			/* If found left/right sibling, find rightmost/leftmost child. */
			if (node_flag) {
				/* Record the sibling in the path. */
				/*
				 * Seed the cursor at the found sibling (depth @level, its
				 * shallow boundary) before descend_children -> minmax, so a
				 * transiently-empty first minmax step re-enters going-up
				 * with the cursor live.
				 */
				up_node = node_flag;
				up_node_lo = level;
				/*
				 * Reaching a sibling IS backtracking (up to the
				 * parent, then over to a strictly-greater/lesser
				 * sibling subtree), even when the sibling is found on
				 * the first iteration without climbing further.  Mark
				 * @going_up so the minmax descent below does NOT set
				 * skip_eq_external_nodes: the sibling subtree's first
				 * external_nodes is a different (strictly greater for
				 * GT) prefix key, not the search key's equal match, so
				 * it must not be skipped.  Without this, GT into an
				 * immediate sibling whose subtree root is a prefix key
				 * (e.g. next("y") with keys "z" < "zz1") skips that
				 * prefix key entirely.
				 */
				going_up = true;
				if (keep_ordinal)
					FT_TP(ineq_going_up_step, level,
						(const void *) up_parent,
						1, ordinal_key[level - 1]);
				else
					FT_TP(ineq_going_up_step_nokey, level,
						(const void *) up_parent, 1);
				break;
			}
			FT_TP(ineq_going_up_step, level,
				(const void *) up_parent,
				0, (uint8_t) key_value);
			going_up = true;
		}

		/*
		 * Prefix-scoped traversal: if backtracking exhausted the
		 * scope without finding a sibling, handle the prefix
		 * boundary.
		 *
		 * For LE/LT the prefix key itself (shorter than the search
		 * key) may be the closest match: return its external_nodes
		 * if present.
		 *
		 * For GE/GT no key within the scope satisfies the inequality.
		 *
		 * When going_up is false (e.g. LIMIT_FIRST/LIMIT_LAST
		 * reaching the prefix node without backtracking), we fall
		 * through to the downward min/max search below.
		 */
		if (going_up && level == (ssize_t) iter->prefix_len) {
			if (dir == FT_LEFT) {
				/*
				 * Node at prefix_len is the going-up cursor's @up_parent.
				 * This block is reached only via normal loop exit (no
				 * sibling found while backtracking down to prefix_len),
				 * whose last iteration ran at level == prefix_len + 1 with
				 * the invariant up_parent == node at level - 1 ==
				 * node at prefix_len.  Obtained via a live back-pointer
				 * read.
				 */
				struct cds_ft_inode_flag *pfx_flag = up_parent;

				if (!ft_node_external(pfx_flag)) {
					struct cds_ft_metadata *metadata;

					if (ft_node_compressed(pfx_flag))
						metadata = cds_ft_item_to_metadata(
							ft_node_ptr(pfx_flag));
					else {
						const struct cds_ft_type *type =
							&ft_types[ft_node_type(pfx_flag)];
						metadata = cds_ft_item_to_metadata_fast(
							ft_node_ptr(pfx_flag),
							type->order);
					}
					struct cds_ft_node *external_nodes =
						ft_dereference_external(metadata->external_nodes);

					if (external_nodes) {
						int j;

						iter->key_len = iter->prefix_len;
						iter->key_off = 0;
						if (!keep_ordinal)
							ft_speculative_keycopy_unconditional(ft,
								external_nodes, iter_key(iter),
								(ssize_t) iter->prefix_len);
						else if (!ft_speculative_keycopy(ft, external_nodes,
								iter_key(iter),
								(ssize_t) iter->prefix_len)) {
							for (j = 0; j < (int) iter->prefix_len; j++)
								iter_key(iter)[j] = ordinal_key[j];
						}
						iter->node = external_nodes;
						iter->cache_valid = true;
						iter_debug_path_update(iter);
						iter->path_len = iter->prefix_len + 1;
						iter->status = CDS_FT_STATUS_OK;
						goto end;
					}
				}
			}
			iter->node = NULL;
			iter->cache_valid = true;
			iter_debug_path_update(iter);
			iter->path_len = iter->prefix_len + 1;
			iter->status = CDS_FT_STATUS_NOT_FOUND;
			goto end;
		}
	}

	/*
	 * Fall-through dead-end within the scope prefix: the descent broke
	 * on an empty slot at level <= prefix_len (the prefix path itself
	 * does not exist), so the going-up loop above -- bounded at
	 * prefix_len -- never ran and the going_up NOT_FOUND arm did not
	 * trigger.  descend_children below would misread the NULL (tag 0)
	 * as an external-node match and return OK with iter->node == NULL.
	 * No key with the scope prefix exists: report NOT_FOUND.  Every
	 * `goto descend_children` jumps past this guard with a non-NULL
	 * @node_flag; only the fall-through path can carry NULL.
	 */
	if (!node_flag) {
		iter->node = NULL;
		iter->cache_valid = true;
		iter_debug_path_update(iter);
		iter->path_len = iter->prefix_len + 1;
		iter->status = CDS_FT_STATUS_NOT_FOUND;
		goto end;
	}

descend_children:
	/*
	 * A skip-encoded sibling carried over from going_up (use_keycopy) is
	 * resolved by the loop's skip-follow below, not tested as external here
	 * -- its tag bits would otherwise be misread as an external node.
	 */
	if (!(use_keycopy && ft_node_skip_compressed(node_flag))
			&& ft_node_external(node_flag)) {
		int j;

		assert(level <= (int) ft->group->max_key_len);
		iter->key_len = level;
		iter->key_off = 0;
		if (!keep_ordinal)
			ft_speculative_keycopy_unconditional(ft,
				(const struct cds_ft_node *) ft_node_ptr(node_flag),
				iter_key(iter), level);
		else if (!ft_speculative_keycopy(ft,
				(const struct cds_ft_node *) ft_node_ptr(node_flag),
				iter_key(iter), level)) {
			for (j = 0; j < level; j++)
				iter_key(iter)[j] = ordinal_key[j];
		}
		iter->node = (struct cds_ft_node *) ft_node_ptr(node_flag);
		iter->cache_valid = true;
		iter_debug_path_update(iter);
		iter->path_len = level + 1;
		iter->status = CDS_FT_STATUS_OK;
		goto end;
	}

	level++;

	/*
	 * From this point, we are guaranteed to be able to find a
	 * "lower than"/"greater than" match. ft_attach_node() and
	 * ft_detach_node() both guarantee that it is not possible for a
	 * lookup to reach a dead-end.
	 */

	/*
	 * Find rightmost/leftmost child of rightmost/leftmost child
	 * (recursively).
	 *
	 * skip_eq_external_nodes: when entering the minmax descent
	 * without backtracking (going_up == false) and the mode is
	 * strictly GT, the external_nodes at the first node are at the
	 * same position as the search key -- equal, not strictly
	 * greater. Skip them on the first iteration so the descent
	 * continues to a proper child.
	 */
	skip_eq_external_nodes = (!going_up && mode == FT_LOOKUP_GT);
	switch (mode) {
	case FT_LOOKUP_LE:
	case FT_LOOKUP_LT:
		dir = FT_RIGHTMOST;
		break;
	case FT_LOOKUP_GE:
	case FT_LOOKUP_GT:
		dir = FT_LEFTMOST;
		break;
	default:
		assert(0);
	}
	for (; level < (int) ft->group->max_tree_depth; level++) {
#ifdef FEATURE_FT_SKIP_COMPRESSED
		/*
		 * Skip-encoded compressed child: follow it directly without
		 * reading the compressed node.  The spanned length is in the
		 * pointer's upper bits (ft_skip_len == cn->len), so this
		 * reproduces ft_inequality_minmax_compressed's level advance
		 * exactly; the only thing dropped is the ordinal_key span fill,
		 * which the leaf-key copy supplies for the result.  A
		 * skip-compressed node cannot carry external_nodes (a terminating
		 * fork the encoding cannot express), so nothing terminates inside
		 * the span -- no min/max candidate is skipped.  Resolving here, at
		 * the loop top, also keeps the external / internal tests below
		 * from misreading the skip pointer's tag bits.
		 */
		if (use_keycopy && caa_unlikely(ft_node_skip_compressed(node_flag))) {
			level += (ssize_t) ft_skip_len(node_flag) - 1;
			node_flag = ft_skip_child_ptr(node_flag);
			skip_eq_external_nodes = false;
			up_node = node_flag;	/* live child below the span */
			up_node_lo = level;
			/*
			 * Mirror ft_inequality_minmax_compressed: an external child
			 * is the leaf at the span end -- break with @level already at
			 * the key length (no loop level++); an internal child
			 * continues the descent (the loop level++ lands on it).
			 */
			if (ft_node_external(node_flag))
				break;
			continue;
		}
#endif
		/*
		 * Return external node associated to internal node if
		 * trying to find GE/GT inequality and encountering an
		 * external node when going downward.
		 */
		if (dir == FT_LEFTMOST && ft_node_internal(node_flag)
				&& !skip_eq_external_nodes) {
			const struct cds_ft_type *type = &ft_types[ft_node_type(node_flag)];
			struct cds_ft_metadata *metadata = cds_ft_item_to_metadata_fast(
					ft_node_ptr(node_flag), type->order);
			struct cds_ft_node *external_nodes = ft_dereference_external(metadata->external_nodes);

			if (external_nodes) {
				ret_node = external_nodes;
				level--;
				goto found_minmax;
			}
		}
		/* Return external node. */
		if (ft_node_external(node_flag))
			break;
		/*
		 * Resolve a skip-encoded node to its compressed form.  No-op under
		 * use_keycopy (skip pointers were already followed at the loop
		 * top); on the fill path it converts a skip pointer so the
		 * compressed handler can read cn->key_bytes.
		 */
		node_flag = ft_resolve_skip_compressed(ft, node_flag);
		/*
		 * Compressed node: traverse the compressed path to reach the
		 * child, filling ordinal_key.  Under use_keycopy this is only the
		 * regular (non-skip-encoded) form for spans longer than
		 * FT_SKIP_LEN_MAX, and the fill is harmless (the result key comes
		 * from the leaf copy).
		 */
		if (ft_node_compressed(node_flag)) {
			enum ft_descent_action act;

			act = ft_inequality_minmax_compressed(
				&node_flag, &level, &ret_node,
				&skip_eq_external_nodes,
				ordinal_key, dir, keep_ordinal);
			if (act == FT_DESCENT_FOUND_MINMAX)
				goto found_minmax;
			if (act == FT_DESCENT_BREAK)
				break;
			assert(act == FT_DESCENT_CONTINUE);
			up_node = node_flag;	/* minmax cn->child at the advanced @level */
			up_node_lo = level;
			continue;
		}
		skip_eq_external_nodes = false;
		node_flag = ft_node_get_minmax(ft, node_flag,
				keep_ordinal ? &ordinal_key[level - 1] : &ord_scratch, dir,
				true /* validate_lookup */);
		/*
		 * Prefetch the min/max child's body for the next iteration's scan.
		 * ft_maybe_prefetch fires only on internal/compressed children
		 * (external/NULL/non-canonical-skip are dropped), so it never
		 * reintroduces the harmful external-leaf prefetch.  The lead comes
		 * from the GE/GT external-nodes metadata read at the loop top, which
		 * is enough to overlap the body miss: measured ~+3-4% on
		 * single-thread cds_ft_next over the 1M-key DNS set (the going-up
		 * sibling scan has no such lead and did NOT benefit -- not added).
		 */
		ft_maybe_prefetch(node_flag);
#ifdef FEATURE_FT_SKIP_COMPRESSED
		if (node_flag && caa_unlikely(ft_node_skip_compressed(node_flag))) {
			if (use_keycopy) {
				/*
				 * Leaf-copy path: a skip-encoded min/max child is returned
				 * raw and followed at the loop top next iteration -- the
				 * key drives the descent and the leaf copy supplies the
				 * spanned bytes, so no re-anchor.
				 */
				up_node = node_flag;
				up_node_lo = level;
				continue;
			}
			{
				/*
				 * Fill path (no leaf-key offset): ordinal_key must be
				 * filled from the live compressed node, so re-anchor the
				 * scanned node on the live structure via the child's parent
				 * chain and re-scan, rather than spin.  @rewind > 0 (merge)
				 * re-anchors shallower; @level -= rewind + 1 then the loop's
				 * level++ nets a -rewind step.  ft_skip_reanchor never
				 * returns NULL on a well-formed trie.
				 */
				unsigned int rewind;
				struct cds_ft_inode_flag *at_pos;
				struct cds_ft_inode_flag *anchor =
					ft_skip_reanchor(ft, node_flag, &rewind, &at_pos);

				assert(anchor != NULL);
				if (caa_likely(rewind == 0)) {
					node_flag = at_pos;
				} else {
					level -= (ssize_t) rewind + 1;
					node_flag = anchor;
					up_node = anchor;
					up_node_lo = level;
					continue;
				}
			}
		}
#endif
		/*
		 * Transiently empty internal node (popcount/pigeon): a reader may
		 * observe every slot of a reachable internal node as
		 * NULL in the narrow window between a writer's per-slot
		 * detach and the upward walk's slot-replace at a higher
		 * ancestor.  Quiescently, reachable internal nodes have
		 * nr_child >= 1 (the upward walk prunes single-child
		 * chains wholesale and replaces at the first multi-child
		 * ancestor, so a slot-emptied internal is never left in
		 * place).  Treat as "empty at this step" and let
		 * going_up find the next sibling at a higher level.
		 * Back level one step so the going-up cursor resumes at
		 * the parent.
		 */
		if (caa_unlikely(!node_flag)) {
			level--;
			going_up = true;
			goto going_up;
		}
		up_node = node_flag;		/* minmax child established at @level */
		up_node_lo = level;
		if (keep_ordinal)
			dbg_printf("cds_ft_lookup_inequality find minmax at %u finds node_flag %p\n",
					(unsigned int) ordinal_key[level - 1], node_flag);
		else
			dbg_printf("cds_ft_lookup_inequality find minmax finds node_flag %p\n",
					node_flag);
		if (ft_node_external(node_flag))
			break;
	}
	/*
	 * Every break path in the descent loop sets node_flag to a
	 * validated external (compressed-branch external child or
	 * minmax-branch external return).  Transiently-empty
	 * intermediate steps (non-root minmax == NULL above) do not
	 * reach this assert -- they jump to going_up and re-enter the
	 * search at a higher level.
	 */
	assert(ft_node_ptr(node_flag));
	ret_node = (struct cds_ft_node *) node_flag;
	/*
	 * The trie should always have external nodes at the
	 * very last level, so level should never grow large enough to overflow
	 * max_key_len.
	 */
	assert(level <= (int) ft->group->max_key_len);
	assert(level <= (int) ft->group->max_key_len);
found_minmax:
	{
		int j;

		iter->key_len = level;
		iter->key_off = 0;
		if (!keep_ordinal)
			ft_speculative_keycopy_unconditional(ft, ret_node,
				iter_key(iter), level);
		else if (!ft_speculative_keycopy(ft, ret_node, iter_key(iter),
				level)) {
			for (j = 0; j < level; j++)
				iter_key(iter)[j] = ordinal_key[j];
		}
		iter->node = ret_node;
		iter->cache_valid = true;
		iter_debug_path_update(iter);
		iter->path_len = level + 1;
		iter->status = ret_node ? CDS_FT_STATUS_OK : CDS_FT_STATUS_NOT_FOUND;
	}
end:
	FT_TP(ineq_result, (int) mode,
		input_key, key_len,
		iter->node ? iter_key(iter) : NULL,
		iter->node ? iter->key_len : 0,
		(int) iter->status);
	iter_auto_invalidate_cache(iter);
	return iter->status;
}

/*
 * Dispatcher: resolve the immutable @use_keycopy config (a speculative
 * skip-compressed group with a leaf-key offset) once, then tail-call the
 * matching always_inline instantiation so each variant's hot loop has the
 * per-call use_keycopy gates constant-folded away.  Mirrors the
 * do_cds_ft_lookup_inner (descend_cand, skip_compressed) specialization.
 * The config is immutable after group create, so this branch is perfectly
 * predicted and amortized over a full traversal.
 *
 * inline_lookup (force-inline) into each entry point: cds_ft_lookup_le/ge/lt/gt
 * pass compile-time-constant @mode and @limit (and cds_ft_next/prev resolve to
 * _gt/_lt), so inlining lets @mode AND @limit -- not just @use_keycopy -- DCE
 * each wrapper down to its single arm.  The six specialized bodies cost .so size
 * but the HOT footprint shrinks (a workload runs one DCE'd wrapper, smaller than
 * the shared generic): measured ~+10% ST and +4-22% MT on 1M-key DNS iterate.
 */
static inline_lookup
enum cds_ft_status cds_ft_lookup_inequality(struct cds_ft *ft,
		struct cds_ft_iter *iter,
		enum ft_lookup_inequality mode,
		enum ft_lookup_limit limit)
{
	if (ft->group->speculative_key_offset_set && ft->group->speculative &&
			(ft->group->flags & CDS_FT_FLAG_SKIP_COMPRESSED))
		return cds_ft_lookup_inequality_impl(ft, iter, mode, limit, true, false);
	return cds_ft_lookup_inequality_impl(ft, iter, mode, limit, false, false);
}

/*
 * Limit-none relational specializations, installed as fn-pointers by
 * ft_install_lookup_ops.  Each bakes its mode, LIMIT_NONE, and use_keycopy
 * into the force-inline cds_ft_lookup_inequality_impl so the body DCEs to a
 * single arm; referenced only via the installed pointer, so they stay real
 * out-of-line symbols and the public entry is a bare indirect tail-call.  The
 * RCU read lock is taken here (mirroring the lookup-API inners).
 */
#define FT_INEQ_SPEC(name, mode, kc)					\
	static enum cds_ft_status name(struct cds_ft *ft,		\
			struct cds_ft_iter *iter)			\
	{								\
		CDS_FT_SCOPED_READER(ft);				\
		return cds_ft_lookup_inequality_impl(ft, iter,		\
				(mode), FT_LOOKUP_LIMIT_NONE, (kc), false); \
	}
FT_INEQ_SPEC(ft_ineq_le_keycopy, FT_LOOKUP_LE, true)
FT_INEQ_SPEC(ft_ineq_le_eager,   FT_LOOKUP_LE, false)
FT_INEQ_SPEC(ft_ineq_ge_keycopy, FT_LOOKUP_GE, true)
FT_INEQ_SPEC(ft_ineq_ge_eager,   FT_LOOKUP_GE, false)
FT_INEQ_SPEC(ft_ineq_lt_keycopy, FT_LOOKUP_LT, true)
FT_INEQ_SPEC(ft_ineq_lt_eager,   FT_LOOKUP_LT, false)
FT_INEQ_SPEC(ft_ineq_gt_keycopy, FT_LOOKUP_GT, true)
FT_INEQ_SPEC(ft_ineq_gt_eager,   FT_LOOKUP_GT, false)
#undef FT_INEQ_SPEC

/*
 * Iterator-based inequality lookup public API.
 * The caller sets the key via cds_ft_iter_set_key() before calling.
 * On return the iterator holds the result key, key length, node, path,
 * and status.
 */
enum cds_ft_status cds_ft_lookup_le(struct cds_ft *ft,
		struct cds_ft_iter *iter)
{
	ft_iter_assert_bound(ft, iter);
	return (*ft->lookup_le_fn)(ft, iter);
}

enum cds_ft_status cds_ft_lookup_ge(struct cds_ft *ft,
		struct cds_ft_iter *iter)
{
	ft_iter_assert_bound(ft, iter);
	return (*ft->lookup_ge_fn)(ft, iter);
}

enum cds_ft_status cds_ft_lookup_lt(struct cds_ft *ft,
		struct cds_ft_iter *iter)
{
	ft_iter_assert_bound(ft, iter);
	return (*ft->lookup_lt_fn)(ft, iter);
}

enum cds_ft_status cds_ft_lookup_gt(struct cds_ft *ft,
		struct cds_ft_iter *iter)
{
	ft_iter_assert_bound(ft, iter);
	return (*ft->lookup_gt_fn)(ft, iter);
}

enum cds_ft_status cds_ft_lookup_first(struct cds_ft *ft,
		struct cds_ft_iter *iter)
{
	size_t saved_key_len = iter->key_len;
	enum cds_ft_status status;

	ft_iter_assert_bound(ft, iter);
	CDS_FT_SCOPED_READER(ft);
	dbg_printf("cds_ft_lookup_first\n");
	/* O(1) endpoint: the ordinal-cell list's minimum cell (unscoped only).
	 * Resolve a flip proxy: head/tail transition atomically with the
	 * neighbour edges during a concurrent splice/unsplice/run move.
	 * iter_auto_invalidate_cache: honour the UNCACHED contract like every
	 * other epilogue -- materialize the lazy leaf-referenced key and drop
	 * the cached position so it is not reused across a lock window. */
	if (ft_ord_cell_fastpath_ok(ft, iter)) {
		status = ft_ord_cell_iter_land(ft, iter,
			ft_ord_cell_resolve_ord(&ft->ord_cell_head));
		iter_auto_invalidate_cache(iter);
		return status;
	}
	/*
	 * LIMIT_FIRST sets key_len to prefix_len internally.
	 * When prefix_len == 0 this corresponds to a traversal of the
	 * entire trie.
	 * When prefix_len > 0 it descends through the prefix key
	 * bytes, then the GE post-traversal returns the prefix key
	 * itself if it has external_nodes, or the LEFTMOST minmax
	 * descent finds the smallest descendant.
	 */
	iter->key_len = iter->prefix_len;
	status = cds_ft_lookup_inequality(ft, iter,
			FT_LOOKUP_GE, FT_LOOKUP_LIMIT_FIRST);
	if (status != CDS_FT_STATUS_OK)
		iter->key_len = saved_key_len;
	return status;
}

enum cds_ft_status cds_ft_lookup_last(struct cds_ft *ft,
		struct cds_ft_iter *iter)
{
	size_t saved_key_len = iter->key_len;
	enum cds_ft_status status;

	ft_iter_assert_bound(ft, iter);
	CDS_FT_SCOPED_READER(ft);
	dbg_printf("cds_ft_lookup_last\n");
	/* O(1) endpoint: the ordinal-cell list's maximum cell (unscoped only).
	 * Resolve a flip proxy (see cds_ft_lookup_first, incl. the UNCACHED
	 * auto-invalidate rationale). */
	if (ft_ord_cell_fastpath_ok(ft, iter)) {
		status = ft_ord_cell_iter_land(ft, iter,
			ft_ord_cell_resolve_ord(&ft->ord_cell_tail));
		iter_auto_invalidate_cache(iter);
		return status;
	}
	/*
	 * LIMIT_LAST always uses key_len = max_key_len. When
	 * prefix_len > 0, the traversal uses actual prefix key bytes
	 * for levels 1..prefix_len then 0xFF for the remaining
	 * levels, descending as deep as possible along the rightmost
	 * path within the prefix subtree. LE backtracking (bounded
	 * at prefix_len) then finds the greatest actual key.
	 */
	iter->key_len = ft->group->max_key_len;
	status = cds_ft_lookup_inequality(ft, iter,
			FT_LOOKUP_LE, FT_LOOKUP_LIMIT_LAST);
	if (status != CDS_FT_STATUS_OK)
		iter->key_len = saved_key_len;
	return status;
}

/*
 * Propagate a signed delta to nr_keys through a snapshot
 * of ancestor internal nodes collected during descent.
 *
 * There are two types of concurrent readers:
 *
 *  - Pointer-based readers (iteration, key lookup) follow pointers
 *    with rcu_dereference.  They are not affected by nr_keys and
 *    always see a structurally consistent trie via RCU.
 *
 *  - Count-based readers (lookup_nth, lookup_nth_last, skip,
 *    count_keys, count_keys_prefix) read nr_keys to guide their
 *    descent.  They use ft_dereference_acquire (CMM_ACQUIRE load)
 *    for both nr_keys loads and child pointer loads, rather than
 *    rcu_dereference, to obtain the memory ordering described below.
 *
 * Undercount property:
 *
 * The update ordering is chosen so that nr_keys transiently
 * undercounts (nr_keys <= actual reachable keys) rather than
 * overcounts.  This is the conservative direction for count-based
 * readers: they may transiently miss a key at the boundary of a
 * concurrent mutation, but they will never enter a subtree expecting
 * a key that does not exist.  The alternative (overcount) would cause
 * count-based readers to descend into a subtree with fewer keys than
 * expected, potentially yielding NOT_FOUND for a key that should be
 * reachable at that rank.
 *
 * Update ordering:
 *
 *   Insert: publish pointer (rcu_assign_pointer), then increment
 *           nr_keys (uatomic_store CMM_RELEASE).
 *   Remove: decrement nr_keys (uatomic_store CMM_RELEASE), then
 *           detach pointer (rcu_assign_pointer).
 *
 * Read-side patterns:
 *
 * Count-based readers traverse the trie both downward and upward.
 * The read-side ordering of nr_keys vs pointer loads depends on
 * the traversal pattern, and each pattern interacts differently
 * with the insert and remove orderings.  Three distinct patterns
 * arise:
 *
 * Pattern 1 -- child pointer, then child's nr_keys
 *             (downward descent + upward walk):
 *
 *   Reader:
 *     R1: ft_dereference_acquire(child)        [load-acquire on parent's slot]
 *     R2: uatomic_load(child.nr_keys, CMM_ACQUIRE)
 *
 *   This is the standard message-passing order.  It occurs whenever
 *   the reader loads a child pointer from a parent node and then
 *   reads the child's own nr_keys (ft_child_key_count).  This
 *   happens in both the downward descent of lookup_nth and the
 *   upward walk of skip when iterating sibling subtrees.
 *
 *   Insert:  The new node's nr_keys is initialized before it is
 *     published via rcu_assign_pointer.  If R1 sees the new child
 *     (acquire pairs with the publish release), R2 sees the
 *     initial nr_keys.  If R1 sees NULL (not yet published), the
 *     reader skips -- undercount.
 *
 *   Remove:  The writer decrements the child's nr_keys before
 *     detaching a deeper pointer.  At this level the child pointer
 *     itself is unchanged, so R1 always sees the child.  R2 sees
 *     either old or decremented nr_keys -- both <= actual.
 *     Undercount holds trivially.
 *
 * Pattern 2 -- external_nodes, then child pointers
 *             (downward descent only):
 *
 *   Reader:
 *     R1: ft_dereference_acquire(metadata->external_nodes)
 *     R2: ft_dereference_acquire(child)
 *
 *   At each internal node during downward descent, the reader
 *   first checks external_nodes (keys at this depth), then
 *   iterates children.  Both fields belong to the same node.
 *
 *   Insert (setting external_nodes):  The writer does
 *     rcu_assign_pointer(external_nodes, node) then increments
 *     ancestor nr_keys.  R1 acquire pairs with the publish
 *     release -- if the reader sees the new external_nodes, the
 *     key is found.  If not, undercount.
 *
 *   Remove (clearing external_nodes):  The writer decrements
 *     nr_keys then rcu_assign_pointer(external_nodes, NULL).
 *     If R1 sees NULL, the acquire pairs with the release,
 *     making the nr_keys decrement visible to subsequent reads.
 *     If R1 sees the old external_nodes, the key is still
 *     reachable -- consistent pre-remove snapshot.
 *
 * Pattern 3 -- current node's nr_keys, then child pointers
 *             (skip_forward at_external_nodes case only):
 *
 *   Reader:
 *     R1: uatomic_load(node.nr_keys, CMM_ACQUIRE)
 *     R2: ft_dereference_acquire(child)
 *
 *   This inverted message-passing order occurs only in
 *   skip_forward when the current position is at an internal
 *   node's external_nodes: the reader reads the node's nr_keys
 *   to count remaining keys in the subtree, then iterates
 *   children.
 *
 *   Insert:
 *     Writer:
 *       W1: rcu_assign_pointer(child, new_node)  [store-release]
 *       W2: uatomic_store(node.nr_keys, ++, CMM_RELEASE)
 *     W2 release ensures W1 is visible when W2 becomes visible.
 *     If R1 sees the incremented nr_keys (acquire pairs with
 *     W2 release), all stores before W2 -- including W1 -- are
 *     visible.  R2 is ordered after R1 (by R1 acquire), so R2
 *     sees the published pointer.
 *     If R1 sees the old nr_keys, the reader does not know about
 *     the new key -- undercount.
 *
 *   Remove:
 *     Writer:
 *       W1: uatomic_store(node.nr_keys, --, CMM_RELEASE)
 *       W2: rcu_assign_pointer(child, NULL)      [store-release]
 *     W2 release ensures W1 is visible when W2 becomes visible.
 *     If R2 sees the detached pointer (acquire pairs with W2
 *     release), W1 is visible.  On multi-copy-atomic
 *     architectures (x86 TSO, ARMv8), R1 acquire orders R1
 *     before R2, and the coherence guarantee ensures R1 observes
 *     at least the state that was globally visible when R2's
 *     value was stored -- which includes W1.  So R1 sees the
 *     decremented nr_keys.
 *     If R2 sees the old pointer (child still present), the key
 *     is still reachable.  nr_keys may be old or decremented --
 *     either way <= actual (undercount).
 *     If R1 sees the decremented nr_keys but R2 sees the old
 *     pointer, nr_keys < actual -- undercount.
 *
 * In all three patterns, regardless of which combination of
 * old/new values the reader observes, nr_keys <= actual reachable
 * keys (undercount property).
 *
 * The ft_dereference_acquire macro (CMM_ACQUIRE rather than
 * rcu_dereference) is specifically needed for Pattern 3's remove
 * case, where the reader loads nr_keys before the pointer at the
 * same level.  Without acquire on the pointer load, a weakly-
 * ordered architecture could observe the detached pointer without
 * the preceding nr_keys decrement, violating the undercount
 * property.  Patterns 1 and 2 would be safe with rcu_dereference
 * alone, but all patterns use ft_dereference_acquire uniformly
 * for simplicity.
 */

/*
 * Shared, non-inlined copy of the inequality descent for the write path.  The
 * ordered-cell maintenance in ft-detach (ft_ord_cell_find_rel,
 * ft_ord_cell_find_pred_from_head, ft_merge_ord_interleave) reaches
 * cds_ft_lookup_inequality_impl at five sites.  The impl is force-inlined for
 * the read hot path -- the ft_ineq_* specializations DCE it on constant
 * mode/limit -- so without this each of those write-side sites inlined the full
 * ~20 KB descent, ~90 KB total.  The mutation modules are redirected to this one
 * shared copy via #define in fractal-trie.c; read-side callers keep the inlined
 * original.
 */
static enum cds_ft_status cds_ft_lookup_inequality_impl_shared(struct cds_ft *ft,
		struct cds_ft_iter *iter, enum ft_lookup_inequality mode,
		enum ft_lookup_limit limit, const bool use_keycopy,
		const bool seed_from_node)
{
	return cds_ft_lookup_inequality_impl(ft, iter, mode, limit, use_keycopy,
			seed_from_node);
}
