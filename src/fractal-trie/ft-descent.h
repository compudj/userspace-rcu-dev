// SPDX-FileCopyrightText: 2012-2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-only

/*
 * src/fractal-trie/ft-descent.h
 *
 * Userspace RCU library - Fractal Trie: the shared trie descent engine (ft_lookup_inner) and skip-pointer handling.
 *
 * Implementation unit: #included once by fractal-trie.c, in dependency
 * order, into a single translation unit (preserves cross-module inlining).
 * Not a standalone header.
 */
#ifndef FRACTAL_TRIE_IMPL
#error "ft-descent.h is an implementation unit; #include it from fractal-trie.c only"
#endif

enum ft_prefix_tracking {
	FT_PREFIX_TRACK_NONE,		/* No prefix tracking. */
	FT_PREFIX_TRACK_PARTIAL,	/* Track closest ancestor with external nodes. */
	FT_PREFIX_TRACK_LONGEST,	/* Track deepest match, even internal-only. */
};

/*
 * Sentinel value indicating that prefix tracking never recorded a
 * match. Used by FT_PREFIX_TRACK_LONGEST to distinguish "empty trie"
 * from "matched at root with no external nodes" (both have
 * match_node == NULL, but the latter sets match_len = 0).
 */
#define FT_MATCH_LEN_NONE	SIZE_MAX

/*
 * Handle a compressed node during exact lookup descent.
 *
 * Compares key bytes against the compressed path, tracks
 * partial/longest match if requested, advances key/index/node_flag
 * past the compressed path, and fills iter_path entries.
 *
 * Returns FT_DESCENT_CONTINUE to continue the loop,
 * FT_DESCENT_BREAK to break, or FT_DESCENT_END to jump to
 * the function's end label (with *status_ret and *found_ret set).
 */
static inline_lookup
enum ft_descent_action ft_lookup_compressed(struct cds_ft_inode_flag **node_flag_p,
		const uint8_t **key_p, const uint8_t *key_end,
		const uint8_t *key_safe_end,
		bool track, bool track_longest,
		const uint8_t **match_key_pos_p, struct cds_ft_node **match_node_p,
		struct cds_ft_node **found_ret,
		enum cds_ft_status *status_ret,
		bool candidate)
{
	struct cds_ft_inode_flag *node_flag = *node_flag_p;
	const uint8_t *key = *key_p;
	struct cds_ft_compressed_node *cn = ft_compressed_node_ptr(node_flag);
	int remaining_key = (int) (key_end - key);
	int remaining_safe = (int) (key_safe_end - key);
	int cmp_len = cn->len < remaining_key ? cn->len : remaining_key;

	/* Check external_nodes at the compressed node's depth. */
	if (track) {
		struct cds_ft_metadata *cn_meta =
			cds_ft_item_to_metadata((struct cds_ft_inode *) cn);
		struct cds_ft_node *ext =
			ft_dereference_external(cn_meta->external_nodes);

		if (ext || track_longest) {
			*match_key_pos_p = key;
			*match_node_p = ext;
		}
	}

	/*
	 * In candidate mode, skip key comparison -- just advance past
	 * the compressed path.  The caller verifies the key at the leaf.
	 */
	if (!candidate) {
		if (track_longest) {
			unsigned int mpos;
			int cmp = ft_key_cmp_ordinals(key, cn->key_bytes,
					cmp_len, remaining_safe, false, &mpos);

			if (cmp != 0) {
				*match_key_pos_p = key + mpos;
				*match_node_p = NULL;
				*status_ret = CDS_FT_STATUS_NOT_FOUND;
				return FT_DESCENT_END;
			}
			*match_key_pos_p = key + cmp_len;
			*match_node_p = NULL;
		} else {
			if (ft_key_cmp_ordinals(key, cn->key_bytes,
					cmp_len, remaining_safe, false, NULL) != 0) {
				*status_ret = CDS_FT_STATUS_NOT_FOUND;
				return FT_DESCENT_END;
			}
		}
	}
	if (cn->len > remaining_key) {
		struct cds_ft_metadata *cn_meta =
			cds_ft_item_to_metadata_fast(
				(struct cds_ft_inode *) cn,
				ft_compressed_order(cn->len));

		*found_ret = ft_dereference_external(cn_meta->external_nodes);
		*status_ret = *found_ret ? CDS_FT_STATUS_OK :
				CDS_FT_STATUS_NOT_FOUND;
		if (track && (*found_ret || track_longest)) {
			*match_key_pos_p = key;
			*match_node_p = *found_ret;
		}
		return FT_DESCENT_END;
	}

	/* Advance past the compressed path. */
	key += cn->len;
	node_flag = ft_dereference_acquire_prefetch(cn->child);
	assert(node_flag != NULL);	/* compressed node always has a live child (by construction) */

	*node_flag_p = node_flag;
	*key_p = key;

	if (key > key_end)
		return FT_DESCENT_BREAK;

	/*
	 * External child before end of key: record for partial
	 * tracking, set NOT_FOUND, and tell the caller to end.
	 */
	if (key < key_end && ft_node_external(node_flag)) {
		if (track) {
			*match_key_pos_p = key;
			*match_node_p = (struct cds_ft_node *) node_flag;
		}
		*status_ret = CDS_FT_STATUS_NOT_FOUND;
		return FT_DESCENT_END;
	}

	/*
	 * Track prefix match at the child node (the node after the
	 * compressed path) so callers that skip the normal tracking
	 * code via continue don't miss it.
	 */
	if (track && key < key_end && !ft_node_external(node_flag)) {
		struct cds_ft_metadata *metadata =
			cds_ft_item_to_metadata(ft_node_ptr(node_flag));
		struct cds_ft_node *ext =
			ft_dereference_external(metadata->external_nodes);

		if (ext || track_longest) {
			*match_key_pos_p = key;
			*match_node_p = ext;
		}
	}

	return FT_DESCENT_CONTINUE;
}

/*
 * do_cds_ft_lookup_inner: descent template.
 *
 * @descend_cand and @skip_compressed are compile-time constants at
 * every call site (the four specialization wrappers below pass true /
 * false literals).  always_inline + literal arguments lets the compiler
 * constant-fold the per-iter branches on these flags:
 *   - loop-top skip-compressed resolution
 *   - get_nth dispatch
 *   - post-get_nth skip-compressed resolution
 * Eliminates the per-iter `test %sil, %sil` hot spot identified via
 * perf annotate.
 */
static inline_lookup
enum cds_ft_status do_cds_ft_lookup_inner(struct cds_ft *ft,
		const uint8_t *key, size_t _key_len, size_t _key_readable_pad,
		struct cds_ft_node **result_node,
		struct cds_ft_iter *iter,
		enum ft_prefix_tracking tracking,
		size_t *tracking_match_len,
		struct cds_ft_node **tracking_match_node,
		bool descend_cand,
		bool skip_compressed)
{
	size_t key_len = ft_key_len(ft, _key_len);
	const uint8_t *orig_key = key;
	const uint8_t *key_end = orig_key + key_len;
	/*
	 * Caller-promised input over-read horizon: @_key_readable_pad
	 * bytes past @key + @key_len are safely loadable without faulting.
	 * Used as the input-side contract for ft_key_cmp_ordinals;
	 * descent through compressed nodes forwards the remaining-safe-
	 * bytes at each level.
	 */
	size_t key_readable_pad = _key_readable_pad;
	const uint8_t *key_safe_end = key_end + key_readable_pad;
	struct cds_ft_inode_flag *node_flag;
	struct cds_ft_node *found = NULL;
	enum cds_ft_status status;
	size_t iter_path_len = 0;
	bool track = (tracking != FT_PREFIX_TRACK_NONE);
	bool track_longest = (tracking == FT_PREFIX_TRACK_LONGEST);
	/*
	 * Pointer-form prefix-tracking state.  @match_key_pos == NULL is
	 * the "no match yet" sentinel (track_longest's initial state, was
	 * FT_MATCH_LEN_NONE in the size_t form).  Non-NULL points into the
	 * @orig_key buffer at the matched position; the size_t match_len
	 * reported to the caller is computed at end: as the difference.
	 */
	const uint8_t *match_key_pos = track_longest ? NULL : orig_key;
	struct cds_ft_node *match_node = NULL;

	CDS_FT_ASSERT_RCU_READ_LOCKED(ft);

	node_flag = ft_root_dereference_prefetch(ft);

	{
	/*
	 * Whether this iter caches its position for continuation reuse
	 * (CACHED mode under a continuously-held RCU read lock).  Captured
	 * before spilling @iter so the terminal path_len computation need
	 * not rehydrate @iter.  No path array is populated during descent:
	 * the going-up backtrack recovers per-level nodes from the live
	 * parent chain, and path_len is derived from the consumed key
	 * length at the terminal.
	 */
	const bool cache_path =
		iter && iter->cache_mode == CDS_FT_ITER_CACHED;

	if (iter)
		iter_debug_path_snapshot(iter);
	/*
	 * Spill @iter to its stack slot after the prologue's last
	 * in-register use of it.  The register holding @iter is then
	 * free for the rest of the function; the matching reload at
	 * end: rehydrates it for the epilogue writes.  Validations
	 * below this line goto-end through the spilled path.
	 */
	FT_SPILL_TO_STACK(iter);

	if (caa_unlikely(!valid_key_len(ft, key_len))) {
		status = CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
		goto end;
	}

	/*
	 * Root is always internal. For key_len == 0, return the root's
	 * metadata external_nodes (NIL-key entries).
	 */
	if (!key_len) {
		const struct cds_ft_type *type = &ft_types[ft_node_type(node_flag)];
		struct cds_ft_metadata *metadata = cds_ft_item_to_metadata_fast(ft_node_ptr(node_flag),
							type->order);
		found = ft_dereference_external(metadata->external_nodes);
		status = found ? CDS_FT_STATUS_OK : CDS_FT_STATUS_NOT_FOUND;
		if (track) {
			match_key_pos = orig_key;
			match_node = found;
		}
		goto end;
	}

	/*
	 * Consider external node in root metadata as possible match.
	 * For longest-match tracking, record the root position even
	 * when there are no external nodes.
	 */
	if (track) {
		const struct cds_ft_type *type = &ft_types[ft_node_type(node_flag)];
		struct cds_ft_metadata *metadata = cds_ft_item_to_metadata_fast(ft_node_ptr(node_flag),
							type->order);
		struct cds_ft_node *external_nodes = ft_dereference_external(metadata->external_nodes);

		if (external_nodes || track_longest) {
			match_key_pos = orig_key;
			match_node = external_nodes;
		}
	}

	/*
	 * Pre-loop non-internal root handler.  graft_swap can place a
	 * compressed node directly at ft->root when it splits inside a
	 * compressed prefix and the displaced subtree becomes the swap's
	 * root.  The hot loop assumes the dispatch parent is an internal
	 * node; resolve a non-internal root once here so the loop body
	 * stays lean (no tag-bit branch before each dispatch).
	 *
	 * Skip-encoded root is not currently produced by any mutator
	 * path, but resolve it defensively for completeness -- cost is
	 * one shr+jne, DCE'd when skip_compressed compile-time false.
	 */
	if (skip_compressed &&
	    caa_unlikely(ft_node_skip_compressed(node_flag))) {
		if (!descend_cand) {
#ifdef FEATURE_FT_SKIP_COMPRESSED
			/*
			 * Mutators do not produce a skip-encoded root.  Resolve
			 * it defensively for completeness via the non-validating
			 * resolver: a root has no concurrent re-parent of its
			 * skip child, so the back-pointer is a stable compressed
			 * node (no mid-split internal-node transient as in the
			 * interior dispatch, which re-anchors instead).
			 */
			node_flag = ft_resolve_skip_compressed(ft, node_flag);
#endif
		} else {
			unsigned int skip = ft_skip_len(node_flag);

			if ((int) skip > (int) (key_end - key)) {
				status = CDS_FT_STATUS_NOT_FOUND;
				goto end;
			}
			key += skip;
			node_flag = ft_skip_child_ptr(node_flag);
		}
	}
	if (caa_unlikely(!ft_node_internal(node_flag))) {
		if (ft_node_compressed(node_flag)) {
			enum ft_descent_action act;

			act = ft_lookup_compressed(&node_flag, &key, key_end,
				key_safe_end,
				track, track_longest,
				&match_key_pos, &match_node, &found, &status,
				descend_cand);
			if (act == FT_DESCENT_END)
				goto end;
			if (act == FT_DESCENT_BREAK)
				goto terminal;
			/* CONTINUE: node_flag is now plain, fall through to loop. */
		} else if (ft_node_external(node_flag)) {
			goto terminal;
		} else {
			status = CDS_FT_STATUS_NOT_FOUND;
			goto end;
		}
	}

#ifdef FEATURE_FT_SKIP_COMPRESSED
descend_loop:
#endif
	while (key < key_end) {
		uint8_t iter_key;

		/*
		 * Loop top is lean: node_flag is internal.  ft->root was
		 * normalized by the pre-loop check above; subsequent
		 * iterations land here only on the internal fall-through
		 * path of the post-step merged handler.  No tag-bit branch
		 * before dispatch.
		 */
		iter_key = *(key++);
		/*
		 * Dispatch returns the slot value as-is, including any
		 * skip-encoded high bits.  Skip resolution is handled
		 * uniformly by the post-step skip handler below (cand
		 * mode advances key + ft_skip_child_ptr; non-cand mode
		 * converts to compressed_flag).
		 *
		 * FT_PF_DATA: the prefetch fires on the raw slot value.
		 * For regular internal/external/compressed children (the
		 * dominant case -- ~97% on dns) the address is clean and
		 * the prefetch hits the right target.  For skip-encoded
		 * children (~3%) the high bits carry skip-length, the
		 * address is non-canonical, and __builtin_prefetch
		 * silently drops it (one cheap uop, no fault).
		 */
		{
			/*
			 * Eager-split: extract type_index from the tagged
			 * @node_flag once, then dispatch via the pretyped
			 * scanner.  The pre-loop normalization (or the
			 * internal-fall-through of the post-step merged
			 * handler) guarantees @node_flag is internal here,
			 * so the scanner doesn't need to re-check tag bit 0.
			 * Per-case FT_NODE_SUB_TAG_NOSKIP with a compile-
			 * time literal folds the SUB into the body load's
			 * displacement.
			 *
			 * Precise-descent skip-encoded resolution: validate the
			 * skip slot against the live compressed node.  On a
			 * mismatch (a concurrent writer split/merge reparented the
			 * skip child, possibly while @parent_node_flag was
			 * recompacted away) re-anchor on the live structure via
			 * ft_skip_reanchor instead of spinning -- a frozen slot may
			 * never republish.  Candidate descent doesn't need the cn
			 * (it advances via ft_skip_child_ptr below), so only the
			 * !descend_cand branch resolves/re-anchors here.
			 */
			struct cds_ft_inode_flag *parent_node_flag = node_flag;
			unsigned long _raw = (unsigned long) parent_node_flag;
			unsigned int _type =
				(unsigned int) ((_raw >> FT_INTERNAL_BITS) & 0x7);

			node_flag = ft_node_get_nth_skip_pretyped(parent_node_flag,
					_type, NULL, iter_key, FT_PF_DATA);
			if (!node_flag) {
				status = CDS_FT_STATUS_NOT_FOUND;
				goto end;
			}
			/*
			 * A cds_ft_merge_at flip transiently installs a type-7
			 * proxy in an interior merge-point slot; resolve it to the
			 * view-appropriate (old or merged) child before the skip /
			 * kind handlers classify it.  Predicted-not-taken when no
			 * merge is in flight (the proxy tag 0xF never matches an
			 * internal / external / compressed / skip child).
			 */
			node_flag = ft_resolve_flip_proxy(node_flag);
#ifdef FEATURE_FT_SKIP_COMPRESSED
			if (skip_compressed && !descend_cand
			    && caa_unlikely(ft_node_skip_compressed(node_flag))) {
				{
					/*
					 * Resolve the skip-compressed slot via the live
					 * skip-child parent chain (ft_skip_reanchor, the
					 * single skip concurrency mechanism).  Never assume
					 * the skip child's back-pointer is a compressed node
					 * and read its len/key_bytes directly: mid-split it
					 * can transiently be an INTERNAL node (e.g. a
					 * suffix_len==0 branch re-parenting the old external
					 * child before the slot's skip pointer is
					 * republished), which would resolve to a garbage key.
					 */
					unsigned int rewind;
					struct cds_ft_inode_flag *at_pos;
					struct cds_ft_inode_flag *anchor =
						ft_skip_reanchor(ft, node_flag, &rewind, &at_pos);

					assert(anchor != NULL);
					if (rewind == 0) {
						/*
						 * Split (or same-length replace): @at_pos is
						 * the live node at the failing slot's encoded
						 * depth -- exactly what the validate-success
						 * path resolves @cn to.  Descend INTO it by
						 * falling through to the post-step handler with
						 * @key unchanged (identical to the
						 * cn != NULL case, just sourced from the live
						 * parent-chain walk).  This bypasses the
						 * holder's stale slot, so we never re-read a
						 * slot that may mismatch again: each re-anchor
						 * advances the descent one level -- wait-free.
						 */
						node_flag = at_pos;
					} else {
						/*
						 * Merge overshoot: the failing level was
						 * absorbed into a longer compressed that begins
						 * ABOVE this depth, so @at_pos cannot be entered
						 * with the unchanged cursor.  Re-anchor at the
						 * holder and re-descend through the normal loop
						 * (path bookkeeping stays on the proven path;
						 * this rarer case remains lock-free).
						 */
						node_flag = anchor;
						/*
						 * The merged run begins within the
						 * span this descent already consumed:
						 * a rewind past @orig_key would mean
						 * the re-anchor walked above the
						 * search root -- impossible on a
						 * well-formed trie; catch a
						 * corruption-driven underflow here
						 * rather than reading before the
						 * caller's buffer.
						 */
						assert((size_t) (key - orig_key) >=
							(size_t) rewind + 1);
						key -= (size_t) rewind + 1;
						goto descend_loop;
					}
				}
			}
#endif
		}
		dbg_printf("cds_ft_lookup iter key lookup %u finds node_flag %p\n",
				(unsigned int) iter_key, node_flag);
#ifdef FEATURE_FT_SKIP_COMPRESSED
		/*
		 * Candidate-mode skip-advance for skip-encoded child slots.
		 * Precise-mode resolution / re-anchor happened in the
		 * dispatcher above.
		 *
		 * Tempting follow-up that does NOT work: adding a
		 * ft_maybe_prefetch(node_flag) here to prefetch the skip
		 * target's body (the 1-step-ahead FT_PF_DATA prefetch saw the
		 * raw skip pointer and dropped it as non-canonical; node_flag
		 * is cleared here so it would fire).  Measured a NET LOSS of
		 * ~10-13% on dns ft_specv at T1 AND T192 (interleaved A/B,
		 * 2026-05-24).  A prefetch on only ~3% of steps cannot cost
		 * that directly: adding the instruction perturbs the codegen /
		 * code layout of this always-inline descent template (which is
		 * iTLB/layout-sensitive) and regresses every step.  Besides,
		 * the lead time is tiny -- a skip target is consumed almost
		 * immediately (validated, for a leaf) -- so it could not hide
		 * the leaf's DRAM latency anyway.  Do not add a prefetch here.
		 */
		if (skip_compressed && descend_cand && caa_unlikely(ft_node_skip_compressed(node_flag))) {
			unsigned int skip = ft_skip_len(node_flag);
			int remaining = (int) (key_end - key);

			if ((int) skip > remaining) {
				status = CDS_FT_STATUS_NOT_FOUND;
				goto end;
			}
			key += skip;
			node_flag = ft_skip_child_ptr(node_flag);
		}
#endif
		/*
		 * Post-step handler for non-internal results.
		 *
		 * Compressed reachable in non-cand mode and also in
		 * cand mode for compressed paths longer than
		 * FT_SKIP_LEN_MAX: those keep the regular compressed
		 * pointer (skip-pointer length encoding wouldn't fit),
		 * so the slot does not carry a skip pointer and the
		 * pre-step skip handler above didn't resolve it.
		 * ft_lookup_compressed(candidate=true) advances past
		 * the compressed path without comparison.
		 */
		if (caa_unlikely(!ft_node_internal(node_flag))) {
			if (caa_unlikely(ft_node_compressed(node_flag))) {
				enum ft_descent_action act;

				act = ft_lookup_compressed(&node_flag, &key, key_end,
					key_safe_end,
					track, track_longest,
					&match_key_pos, &match_node, &found, &status,
					descend_cand);
				if (act == FT_DESCENT_END)
					goto end;
				if (act == FT_DESCENT_BREAK)
					break;
				continue;
			}
			if (ft_node_external(node_flag)) {
				if (key < key_end) {
					if (track) {
						match_key_pos = key;
						match_node = (struct cds_ft_node *) node_flag;
					}
					status = CDS_FT_STATUS_NOT_FOUND;
					goto end;
				}
				/* terminal external -- fall through. */
			} else {
				status = CDS_FT_STATUS_NOT_FOUND;
				goto end;
			}
		} else if (track && key < key_end) {
			/*
			 * Track prefix match on the internal child.  DCE'd
			 * when track=false (cds_ft_eager_lookup_key).
			 */
			const struct cds_ft_type *type = &ft_types[ft_node_type(node_flag)];
			struct cds_ft_metadata *metadata = cds_ft_item_to_metadata_fast(
					ft_node_ptr(node_flag), type->order);
			struct cds_ft_node *external_nodes = ft_dereference_external(metadata->external_nodes);

			if (external_nodes || track_longest) {
				match_key_pos = key;
				match_node = external_nodes;
			}
		}
	}

terminal:
	/*
	 * path_len is the number of path levels (root + one per consumed
	 * key byte) used by a CACHED iter's continuation fast path; derive
	 * it from the consumed key length.  Zero for UNCACHED / no-iter
	 * (unused there: UNCACHED clears cache_valid in the epilogue).
	 */
	if (cache_path)
		iter_path_len = (size_t) (key - orig_key) + 1;
	}

	/*
	 * Reached key_depth, check for terminal node: either external
	 * nodes or internal/compressed node associated with external nodes.
	 */
	if (ft_node_internal(node_flag)) {
		const struct cds_ft_type *type = &ft_types[ft_node_type(node_flag)];
		struct cds_ft_metadata *metadata = cds_ft_item_to_metadata_fast(ft_node_ptr(node_flag),
							type->order);
		found = ft_dereference_external(metadata->external_nodes);
		status = found ? CDS_FT_STATUS_OK : CDS_FT_STATUS_NOT_FOUND;
		if (track && (found || track_longest)) {
			match_key_pos = key_end;
			match_node = found;
		}
	} else if (ft_node_compressed(node_flag)) {
		struct cds_ft_compressed_node *cn =
			ft_compressed_node_ptr(node_flag);
		struct cds_ft_metadata *metadata =
			cds_ft_item_to_metadata_fast(
				(struct cds_ft_inode *) cn,
				ft_compressed_order(cn->len));
		found = ft_dereference_external(metadata->external_nodes);
		status = found ? CDS_FT_STATUS_OK : CDS_FT_STATUS_NOT_FOUND;
		if (track && (found || track_longest)) {
			match_key_pos = key_end;
			match_node = found;
		}
	} else {
		found = (struct cds_ft_node *) node_flag;
		/*
		 * NULL also matches the external tag: pathological re-anchor
		 * exhaustion can deliver it here.  Report NOT_FOUND rather
		 * than OK with a NULL result node (an NDEBUG build has no
		 * assert left to catch the contradiction).
		 */
		status = found ? CDS_FT_STATUS_OK : CDS_FT_STATUS_NOT_FOUND;
		if (track && (found || track_longest)) {
			match_key_pos = key_end;
			match_node = found;
		}
	}

end:
	/* Bring @iter back into a register for the epilogue writes. */
	FT_RELOAD_FROM_STACK(iter);
	if (result_node)
		*result_node = found;
	if (iter) {
		iter->node = found;
		iter->status = status;
		iter->path_len = iter_path_len;
		/*
		 * The path is valid for backtracking when we
		 * successfully descended into the trie, even if the
		 * exact key was not found.
		 */
		iter->cache_valid = (status == CDS_FT_STATUS_OK);
		iter_debug_path_update(iter);
		iter_auto_invalidate_cache(iter);
	}
	if (track) {
		*tracking_match_len = match_key_pos ?
			(size_t) (match_key_pos - orig_key) :
			FT_MATCH_LEN_NONE;
		*tracking_match_node = match_node;
	}
	return status;
}

/*
 * Four specialized instantiations of do_cds_ft_lookup_inner.  Each
 * wrapper passes a const (descend_cand, skip_compressed) pair so the
 * always_inline body collapses to a single specialized descent loop.
 * inline_lookup (always_inline) keeps them inlined into the dispatcher
 * and ultimately into the public entry points, so per-caller constants
 * (iter == NULL, candidate, tracking) also DCE the body.
 */
static inline_lookup
enum cds_ft_status do_cds_ft_lookup_dc_sc(struct cds_ft *ft,
		const uint8_t *key, size_t _key_len, size_t _key_readable_pad,
		struct cds_ft_node **result_node,
		struct cds_ft_iter *iter,
		enum ft_prefix_tracking tracking,
		size_t *tracking_match_len,
		struct cds_ft_node **tracking_match_node)
{
	return do_cds_ft_lookup_inner(ft, key, _key_len, _key_readable_pad,
			result_node, iter,
			tracking, tracking_match_len, tracking_match_node,
			true, true);
}

static inline_lookup
enum cds_ft_status do_cds_ft_lookup_dc_nosc(struct cds_ft *ft,
		const uint8_t *key, size_t _key_len, size_t _key_readable_pad,
		struct cds_ft_node **result_node,
		struct cds_ft_iter *iter,
		enum ft_prefix_tracking tracking,
		size_t *tracking_match_len,
		struct cds_ft_node **tracking_match_node)
{
	return do_cds_ft_lookup_inner(ft, key, _key_len, _key_readable_pad,
			result_node, iter,
			tracking, tracking_match_len, tracking_match_node,
			true, false);
}

static inline_lookup
enum cds_ft_status do_cds_ft_lookup_nodc_sc(struct cds_ft *ft,
		const uint8_t *key, size_t _key_len, size_t _key_readable_pad,
		struct cds_ft_node **result_node,
		struct cds_ft_iter *iter,
		enum ft_prefix_tracking tracking,
		size_t *tracking_match_len,
		struct cds_ft_node **tracking_match_node)
{
	return do_cds_ft_lookup_inner(ft, key, _key_len, _key_readable_pad,
			result_node, iter,
			tracking, tracking_match_len, tracking_match_node,
			false, true);
}

static inline_lookup
enum cds_ft_status do_cds_ft_lookup_nodc_nosc(struct cds_ft *ft,
		const uint8_t *key, size_t _key_len, size_t _key_readable_pad,
		struct cds_ft_node **result_node,
		struct cds_ft_iter *iter,
		enum ft_prefix_tracking tracking,
		size_t *tracking_match_len,
		struct cds_ft_node **tracking_match_node)
{
	return do_cds_ft_lookup_inner(ft, key, _key_len, _key_readable_pad,
			result_node, iter,
			tracking, tracking_match_len, tracking_match_node,
			false, false);
}

/*
 * @candidate: when true, skip key comparison at compressed nodes
 * during traversal (patricia-like mode).  The returned node is a
 * candidate that must be verified by the caller against their
 * stored key.  Constant-folded at each call site.
 *
 * 4-way dispatch on (descend_cand, skip_compressed).
 */
static inline_lookup
enum cds_ft_status do_cds_ft_lookup(struct cds_ft *ft,
		const uint8_t *key, size_t _key_len, size_t _key_readable_pad,
		struct cds_ft_node **result_node,
		struct cds_ft_iter *iter,
		enum ft_prefix_tracking tracking,
		size_t *tracking_match_len,
		struct cds_ft_node **tracking_match_node,
		bool candidate)
{
	bool skip_compressed = ft_group_skip_compressed(ft->group);

	if (candidate) {
		if (skip_compressed)
			return do_cds_ft_lookup_dc_sc(ft, key, _key_len, _key_readable_pad,
					result_node, iter, tracking,
					tracking_match_len, tracking_match_node);
		return do_cds_ft_lookup_dc_nosc(ft, key, _key_len, _key_readable_pad,
				result_node, iter, tracking,
				tracking_match_len, tracking_match_node);
	}
	if (skip_compressed)
		return do_cds_ft_lookup_nodc_sc(ft, key, _key_len, _key_readable_pad,
				result_node, iter, tracking,
				tracking_match_len, tracking_match_node);
	return do_cds_ft_lookup_nodc_nosc(ft, key, _key_len, _key_readable_pad,
			result_node, iter, tracking,
			tracking_match_len, tracking_match_node);
}

/*
 * Per-API cluster sub-sections.  Each public-API stub lives in
 * .text.hot.cds_ft_<api>.0_stub; its primary inner in
 * .text.hot.cds_ft_<api>.1_primary; secondary variants in
 * .text.hot.cds_ft_<api>.2_secondary.
 *
 * The linker script ft-lookup-layout.ld groups each cluster
 * (per public API) into one output section, page-aligned, with
 * sub-sections sorted alphabetically -- stub first, primary
 * second, secondary last.  This ensures the bench's hot path
 * (call -> stub -> primary inner) touches a single iTLB page on
 * the critical setup, and primary's tail spilling past 4 KiB
 * doesn't cost an extra iTLB miss on every lookup because the
 * descent's PC stays on page 1 during the first iterations.
 */
/*
 * Cluster layout: dispatch at page start, fast path next, slow paths
 * trailing.  The linker script (ft-lookup-layout.ld) gathers each
 * cluster's sub-sections in alphabetical name order, page-aligned
 * at the start:
 *
 *   ALIGN(4096) +
 *               |  .text.hot.cds_ft_<api>.0_dispatch  -- public stub
 *               |  .text.hot.cds_ft_<api>.1_fast      -- primary inner
 *               |                                      (spec_validated +
 *               |                                       skip_compressed)
 *               |  .text.hot.cds_ft_<api>.2_slow      -- slow paths:
 *               |                                      precise_*, *_nosc,
 *               |                                      *_nonidentity
 *
 * For the bench's hot path (call -> stub -> primary inner), the call
 * lands at the cluster's page boundary; the stub's indirect jmp
 * forwards into the primary on the same page.  Slow paths trail
 * later in the cluster (later addresses, possibly later pages).
 * The whole cluster shares iTLB/icache locality.
 *
 * Direction of the indirect jmp (forward to primary vs backward to
 * a slow path) is not a factor -- unconditional jmps don't use the
 * "backward predicted taken" heuristic (that applies only to
 * conditional branches with a cold BPB).  Layout experiments
 * 2026-05-22 confirmed stub-FIRST / stub-LAST / stub on different
 * page from primary all measure within ~1 ns on ft_specv_local
 * dns load-names ST.  Dispatch-FIRST was selected for clarity:
 * the bench's CALL lands at the page-aligned cluster start,
 * forward into the next-most-likely target.
 */
#define FT_LOOKUP_DISPATCH(name)	\
	__attribute__((section(".text.hot.cds_ft_" name ".0_dispatch")))
#define FT_LOOKUP_FAST_PATH(name)	\
	__attribute__((section(".text.hot.cds_ft_" name ".1_fast")))
#define FT_LOOKUP_SLOW_PATH(name)	\
	__attribute__((section(".text.hot.cds_ft_" name ".2_slow")))

/*
 * Specialized lookup_key/lookup_candidate_key inner functions.
 * Each bakes the (descend_cand, skip_compressed) pair into a
 * literal-arg call to the matching always_inline
 * wrapper -- the wrapper then inlines into the inner with all
 * per-iter branches on those constants folded out.  Each is
 * referenced via the function pointers installed on struct cds_ft
 * by ft_install_lookup_ops (see cds_ft_create), so gcc cannot
 * inline them into the caller -- they exist as real symbols and
 * the public entry point dispatches via an indirect tail call.
 *
 * Identity key_map is implied for the four hot-path inners
 * (FT_PREFIX_TRACK_NONE, key passed directly, no ordinals[]).
 * Non-identity callers route through ft_lookup_*_nonidentity
 * below, which carry a FT_MAX_KEY_LEN stack buffer.
 */
static FT_LOOKUP_FAST_PATH("lookup_key")
enum cds_ft_status ft_lookup_precise_sc(struct cds_ft *ft,
		const uint8_t *key, size_t key_len, size_t key_readable_pad,
		struct cds_ft_node **result_node)
{
	enum cds_ft_status status;
	key_len = ft_key_len(ft, key_len);
	if (!valid_key_len(ft, key_len))
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	CDS_FT_SCOPED_READER(ft);
	FT_TP_KEY(lookup_key_enter, ft, key, key_len);
	status = do_cds_ft_lookup_nodc_sc(ft, key, key_len, key_readable_pad,
			result_node, NULL,
			FT_PREFIX_TRACK_NONE, NULL, NULL);
	FT_TP(lookup_key_exit, (int) status);
	return status;
}

static FT_LOOKUP_SLOW_PATH("lookup_key")
enum cds_ft_status ft_lookup_precise_nosc(struct cds_ft *ft,
		const uint8_t *key, size_t key_len, size_t key_readable_pad,
		struct cds_ft_node **result_node)
{
	enum cds_ft_status status;
	key_len = ft_key_len(ft, key_len);
	if (!valid_key_len(ft, key_len))
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	CDS_FT_SCOPED_READER(ft);
	FT_TP_KEY(lookup_key_enter, ft, key, key_len);
	status = do_cds_ft_lookup_nodc_nosc(ft, key, key_len, key_readable_pad,
			result_node, NULL,
			FT_PREFIX_TRACK_NONE, NULL, NULL);
	FT_TP(lookup_key_exit, (int) status);
	return status;
}

static FT_LOOKUP_FAST_PATH("lookup_candidate_key")
enum cds_ft_status ft_lookup_cand_sc(struct cds_ft *ft,
		const uint8_t *key, size_t key_len, size_t key_readable_pad,
		struct cds_ft_node **result_node)
{
	key_len = ft_key_len(ft, key_len);
	if (!valid_key_len(ft, key_len))
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	CDS_FT_SCOPED_READER(ft);
	return do_cds_ft_lookup_dc_sc(ft, key, key_len, key_readable_pad,
			result_node, NULL,
			FT_PREFIX_TRACK_NONE, NULL, NULL);
}

static FT_LOOKUP_SLOW_PATH("lookup_candidate_key")
enum cds_ft_status ft_lookup_cand_nosc(struct cds_ft *ft,
		const uint8_t *key, size_t key_len, size_t key_readable_pad,
		struct cds_ft_node **result_node)
{
	key_len = ft_key_len(ft, key_len);
	if (!valid_key_len(ft, key_len))
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	CDS_FT_SCOPED_READER(ft);
	return do_cds_ft_lookup_dc_nosc(ft, key, key_len, key_readable_pad,
			result_node, NULL,
			FT_PREFIX_TRACK_NONE, NULL, NULL);
}

/*
 * Non-identity key_map fallback (key + candidate variants).
 * Allocates a FT_MAX_KEY_LEN stack buffer for the ordinals[]
 * remap and routes through do_cds_ft_lookup, which re-derives the
 * descend_cand path from group state at runtime.
 * Non-identity maps are rare (only set via
 * cds_ft_group_attr_set_key_map) so the extra dispatch hop is
 * not worth specializing further.
 */
#ifdef FEATURE_FT_KEY_MAP
static FT_LOOKUP_SLOW_PATH("lookup_key")
enum cds_ft_status ft_lookup_key_nonidentity(struct cds_ft *ft,
		const uint8_t *key, size_t key_len,
		size_t key_readable_pad,
		struct cds_ft_node **result_node)
{
	uint8_t ordinals[FT_MAX_KEY_LEN];
	enum cds_ft_status status;

	(void) key_readable_pad;	/* ordinals[] is a stack buffer of
					   fixed size; readable_pad doesn't
					   carry across the remap. */
	key_len = ft_key_len(ft, key_len);
	if (!valid_key_len(ft, key_len))
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	CDS_FT_SCOPED_READER(ft);
	ft_key_to_ordinals(ordinals, key, key_len, &ft->group->key_map);
	FT_TP_KEY(lookup_key_enter, ft, ordinals, key_len);
	status = do_cds_ft_lookup(ft, ordinals, key_len,
			FT_KEY_READABLE_PAD, result_node, NULL,
			FT_PREFIX_TRACK_NONE, NULL, NULL, false);
	FT_TP(lookup_key_exit, (int) status);
	return status;
}

static FT_LOOKUP_SLOW_PATH("lookup_candidate_key")
enum cds_ft_status ft_lookup_candidate_key_nonidentity(struct cds_ft *ft,
		const uint8_t *key, size_t key_len,
		size_t key_readable_pad,
		struct cds_ft_node **result_node)
{
	uint8_t ordinals[FT_MAX_KEY_LEN];

	(void) key_readable_pad;
	key_len = ft_key_len(ft, key_len);
	if (!valid_key_len(ft, key_len))
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	CDS_FT_SCOPED_READER(ft);
	ft_key_to_ordinals(ordinals, key, key_len, &ft->group->key_map);
	return do_cds_ft_lookup(ft, ordinals, key_len,
			FT_KEY_READABLE_PAD, result_node, NULL,
			FT_PREFIX_TRACK_NONE, NULL, NULL, true);
}
#endif /* FEATURE_FT_KEY_MAP */

/*
 * Public lookup_key / lookup_candidate_key entries.
 *
 * Reduced to a single indirect tail-call through the function
 * pointer installed on @ft at create time (see
 * ft_install_lookup_ops).  gcc -O2 emits a sibling call
 * (`mov 0x?(%rdi),%rax; jmp *%rax`) -- no stack frame, no
 * callee-save save/restore, no per-call branch on group shape.
 * The branch predictor stores the fn-ptr target once per trie
 * and predicts it for every subsequent lookup.
 */
FT_LOOKUP_DISPATCH("lookup_key")
