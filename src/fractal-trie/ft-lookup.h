// SPDX-FileCopyrightText: 2012-2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-only

/*
 * src/fractal-trie/ft-lookup.h
 *
 * Userspace RCU library - Fractal Trie: point lookups: exact / eager / candidate / partial / longest-match.
 *
 * Implementation unit: #included once by fractal-trie.c, in dependency
 * order, into a single translation unit (preserves cross-module inlining).
 * Not a standalone header.
 */
#ifndef FRACTAL_TRIE_IMPL
#error "ft-lookup.h is an implementation unit; #include it from fractal-trie.c only"
#endif

enum cds_ft_status cds_ft_eager_lookup_key(struct cds_ft *ft,
		const uint8_t *key, size_t _key_len, size_t _key_readable_pad,
		struct cds_ft_node **result_node)
{
	return (*ft->lookup_key_fn)(ft, key, _key_len,
			_key_readable_pad, result_node);
}

/*
 * cds_ft_lookup_candidate_key - Fast candidate lookup.
 *
 * Skips key comparison at compressed nodes during traversal,
 * returning a candidate node that may not be an exact match.
 * The caller MUST verify the returned node's key matches the
 * lookup key.  If it does not match, the key is not in the trie.
 *
 * This is faster than cds_ft_eager_lookup_key for workloads with long
 * compressed paths (e.g. reverse DNS, file paths) because it
 * eliminates per-node key comparisons, doing a single verification
 * at the end instead.
 */
FT_LOOKUP_DISPATCH("lookup_candidate_key")
enum cds_ft_status cds_ft_lookup_candidate_key(struct cds_ft *ft,
		const uint8_t *key, size_t _key_len, size_t _key_readable_pad,
		struct cds_ft_node **result_node)
{
	return (*ft->lookup_candidate_key_fn)(ft, key, _key_len,
			_key_readable_pad, result_node);
}

/*
 * Specialized iter-form lookup inners.  iter is always non-NULL on
 * this path (path tracking is the whole point of the iter API), so
 * each inner bakes that into the wrapper call along with the
 * (descend_cand, skip_compressed) pair.  iter already holds
 * ordinals-mapped key bytes (see cds_ft_iter_set_key: non-identity
 * key_map remapping happens at iter-set time, not at lookup time),
 * so there is no non-identity branch on this path.
 */
static FT_LOOKUP_FAST_PATH("lookup")
enum cds_ft_status ft_lookup_iter_precise_sc(struct cds_ft *ft,
		struct cds_ft_iter *iter)
{
	enum cds_ft_status status;
	CDS_FT_SCOPED_READER(ft);
	FT_TP_ITER_KEY(lookup_enter, iter);
	status = do_cds_ft_lookup_nodc_sc(ft, iter_key(iter), iter->key_len,
			FT_KEY_READABLE_PAD, NULL, iter,
			FT_PREFIX_TRACK_NONE, NULL, NULL);
	FT_TP(lookup_exit, (int) status);
	return status;
}

static FT_LOOKUP_SLOW_PATH("lookup")
enum cds_ft_status ft_lookup_iter_precise_nosc(struct cds_ft *ft,
		struct cds_ft_iter *iter)
{
	enum cds_ft_status status;
	CDS_FT_SCOPED_READER(ft);
	FT_TP_ITER_KEY(lookup_enter, iter);
	status = do_cds_ft_lookup_nodc_nosc(ft, iter_key(iter), iter->key_len,
			FT_KEY_READABLE_PAD, NULL, iter,
			FT_PREFIX_TRACK_NONE, NULL, NULL);
	FT_TP(lookup_exit, (int) status);
	return status;
}

FT_LOOKUP_DISPATCH("lookup")
enum cds_ft_status cds_ft_lookup(struct cds_ft *ft,
		struct cds_ft_iter *iter)
{
	ft_iter_assert_bound(ft, iter);
	return (*ft->lookup_iter_fn)(ft, iter);
}

/*
 * Specialized partial_key inners (tracking=PARTIAL, identity key_map).
 * Always precise descent (descend_cand=false) because partial-match
 * needs per-step compressed verification.
 */
static FT_LOOKUP_FAST_PATH("lookup_partial_key")
enum cds_ft_status ft_lookup_partial_key_sc(struct cds_ft *ft,
		const uint8_t *key, size_t _key_len, size_t *match_len,
		struct cds_ft_node **result_node)
{
	struct cds_ft_node *partial_node = NULL;
	size_t partial_len = 0;
	size_t key_len = ft_key_len(ft, _key_len);

	if (!valid_key_len(ft, key_len)) {
		*match_len = 0;
		*result_node = NULL;
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}
	CDS_FT_SCOPED_READER(ft);
	do_cds_ft_lookup_nodc_sc(ft, key, key_len, 0, NULL, NULL,
			FT_PREFIX_TRACK_PARTIAL, &partial_len, &partial_node);
	*match_len = partial_len;
	*result_node = partial_node;
	return partial_node ? CDS_FT_STATUS_OK : CDS_FT_STATUS_NOT_FOUND;
}

static FT_LOOKUP_SLOW_PATH("lookup_partial_key")
enum cds_ft_status ft_lookup_partial_key_nosc(struct cds_ft *ft,
		const uint8_t *key, size_t _key_len, size_t *match_len,
		struct cds_ft_node **result_node)
{
	struct cds_ft_node *partial_node = NULL;
	size_t partial_len = 0;
	size_t key_len = ft_key_len(ft, _key_len);

	if (!valid_key_len(ft, key_len)) {
		*match_len = 0;
		*result_node = NULL;
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}
	CDS_FT_SCOPED_READER(ft);
	do_cds_ft_lookup_nodc_nosc(ft, key, key_len, 0, NULL, NULL,
			FT_PREFIX_TRACK_PARTIAL, &partial_len, &partial_node);
	*match_len = partial_len;
	*result_node = partial_node;
	return partial_node ? CDS_FT_STATUS_OK : CDS_FT_STATUS_NOT_FOUND;
}

/*
 * Non-identity key_map fallback for partial_key.  Routes through
 * the generic do_cds_ft_lookup dispatcher because non-identity is
 * rare; ordinals[] is allocated on stack inside this fn so
 * call/return overhead is acceptable.
 */
#ifdef FEATURE_FT_KEY_MAP
static FT_LOOKUP_SLOW_PATH("lookup_partial_key")
enum cds_ft_status ft_lookup_partial_key_nonidentity(struct cds_ft *ft,
		const uint8_t *key, size_t _key_len, size_t *match_len,
		struct cds_ft_node **result_node)
{
	struct cds_ft_node *partial_node = NULL;
	size_t partial_len = 0;
	size_t key_len = ft_key_len(ft, _key_len);
	uint8_t ordinals[FT_MAX_KEY_LEN];

	if (!valid_key_len(ft, key_len)) {
		*match_len = 0;
		*result_node = NULL;
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}
	CDS_FT_SCOPED_READER(ft);
	ft_key_to_ordinals(ordinals, key, key_len, &ft->group->key_map);
	do_cds_ft_lookup(ft, ordinals, key_len, 0, NULL, NULL,
			FT_PREFIX_TRACK_PARTIAL, &partial_len, &partial_node,
			false);
	*match_len = partial_len;
	*result_node = partial_node;
	return partial_node ? CDS_FT_STATUS_OK : CDS_FT_STATUS_NOT_FOUND;
}
#endif /* FEATURE_FT_KEY_MAP */

FT_LOOKUP_DISPATCH("lookup_partial_key")
enum cds_ft_status cds_ft_lookup_partial_key(struct cds_ft *ft,
		const uint8_t *key, size_t _key_len, size_t *match_len,
		struct cds_ft_node **result_node)
{
	return (*ft->lookup_partial_key_fn)(ft, key, _key_len, match_len,
			result_node);
}

/*
 * Specialized partial (iter form) inners.  Same as lookup_iter_*
 * but with tracking=PARTIAL and the post-descent iter override.
 */
static FT_LOOKUP_FAST_PATH("lookup_partial")
enum cds_ft_status ft_lookup_partial_iter_sc(struct cds_ft *ft,
		struct cds_ft_iter *iter)
{
	struct cds_ft_node *partial_node = NULL;
	size_t partial_len = 0;

	CDS_FT_SCOPED_READER(ft);
	do_cds_ft_lookup_nodc_sc(ft, iter_key(iter), iter->key_len,
			FT_KEY_READABLE_PAD, NULL, iter,
			FT_PREFIX_TRACK_PARTIAL, &partial_len, &partial_node);
	iter->node = partial_node;
	iter->key_len = partial_len;
	iter->path_len = partial_len + 1;
	iter->status = partial_node ? CDS_FT_STATUS_OK : CDS_FT_STATUS_NOT_FOUND;
	return iter->status;
}

static FT_LOOKUP_SLOW_PATH("lookup_partial")
enum cds_ft_status ft_lookup_partial_iter_nosc(struct cds_ft *ft,
		struct cds_ft_iter *iter)
{
	struct cds_ft_node *partial_node = NULL;
	size_t partial_len = 0;

	CDS_FT_SCOPED_READER(ft);
	do_cds_ft_lookup_nodc_nosc(ft, iter_key(iter), iter->key_len,
			FT_KEY_READABLE_PAD, NULL, iter,
			FT_PREFIX_TRACK_PARTIAL, &partial_len, &partial_node);
	iter->node = partial_node;
	iter->key_len = partial_len;
	iter->path_len = partial_len + 1;
	iter->status = partial_node ? CDS_FT_STATUS_OK : CDS_FT_STATUS_NOT_FOUND;
	return iter->status;
}

FT_LOOKUP_DISPATCH("lookup_partial")
enum cds_ft_status cds_ft_lookup_partial(struct cds_ft *ft,
		struct cds_ft_iter *iter)
{
	ft_iter_assert_bound(ft, iter);
	return (*ft->lookup_partial_iter_fn)(ft, iter);
}

/*
 * Helper: derive the public-API return value for longest_match
 * from the descent result.  Sets *match_len / *result_node and
 * returns CDS_FT_STATUS_OK / NOT_FOUND / INTERNAL_MATCH or a
 * negative status when the descent itself failed.
 */
static inline
enum cds_ft_status ft_lookup_longest_match_key_finish(
		enum cds_ft_status ret,
		size_t longest_len, struct cds_ft_node *match_node,
		size_t *match_len, struct cds_ft_node **result_node)
{
	if (ret < 0) {
		*match_len = 0;
		*result_node = NULL;
		return ret;
	}
	if (longest_len == FT_MATCH_LEN_NONE) {
		*match_len = 0;
		*result_node = NULL;
		return CDS_FT_STATUS_NOT_FOUND;
	}
	*match_len = longest_len;
	*result_node = match_node;
	return match_node ? CDS_FT_STATUS_OK : CDS_FT_STATUS_INTERNAL_MATCH;
}

static FT_LOOKUP_FAST_PATH("lookup_longest_match_key")
enum cds_ft_status ft_lookup_longest_match_key_sc(struct cds_ft *ft,
		const uint8_t *key, size_t _key_len, size_t *match_len,
		struct cds_ft_node **result_node)
{
	struct cds_ft_node *match_node = NULL;
	size_t longest_len = 0;
	enum cds_ft_status ret;
	size_t key_len = ft_key_len(ft, _key_len);

	if (!valid_key_len(ft, key_len)) {
		*match_len = 0;
		*result_node = NULL;
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}
	CDS_FT_SCOPED_READER(ft);
	ret = do_cds_ft_lookup_nodc_sc(ft, key, key_len, 0, NULL, NULL,
			FT_PREFIX_TRACK_LONGEST, &longest_len, &match_node);
	return ft_lookup_longest_match_key_finish(ret, longest_len,
			match_node, match_len, result_node);
}

static FT_LOOKUP_SLOW_PATH("lookup_longest_match_key")
enum cds_ft_status ft_lookup_longest_match_key_nosc(struct cds_ft *ft,
		const uint8_t *key, size_t _key_len, size_t *match_len,
		struct cds_ft_node **result_node)
{
	struct cds_ft_node *match_node = NULL;
	size_t longest_len = 0;
	enum cds_ft_status ret;
	size_t key_len = ft_key_len(ft, _key_len);

	if (!valid_key_len(ft, key_len)) {
		*match_len = 0;
		*result_node = NULL;
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}
	CDS_FT_SCOPED_READER(ft);
	ret = do_cds_ft_lookup_nodc_nosc(ft, key, key_len, 0, NULL, NULL,
			FT_PREFIX_TRACK_LONGEST, &longest_len, &match_node);
	return ft_lookup_longest_match_key_finish(ret, longest_len,
			match_node, match_len, result_node);
}

#ifdef FEATURE_FT_KEY_MAP
static FT_LOOKUP_SLOW_PATH("lookup_longest_match_key")
enum cds_ft_status ft_lookup_longest_match_key_nonidentity(struct cds_ft *ft,
		const uint8_t *key, size_t _key_len, size_t *match_len,
		struct cds_ft_node **result_node)
{
	struct cds_ft_node *match_node = NULL;
	size_t longest_len = 0;
	enum cds_ft_status ret;
	size_t key_len = ft_key_len(ft, _key_len);
	uint8_t ordinals[FT_MAX_KEY_LEN];

	if (!valid_key_len(ft, key_len)) {
		*match_len = 0;
		*result_node = NULL;
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}
	CDS_FT_SCOPED_READER(ft);
	ft_key_to_ordinals(ordinals, key, key_len, &ft->group->key_map);
	ret = do_cds_ft_lookup(ft, ordinals, key_len, 0, NULL, NULL,
			FT_PREFIX_TRACK_LONGEST, &longest_len, &match_node, false);
	return ft_lookup_longest_match_key_finish(ret, longest_len,
			match_node, match_len, result_node);
}
#endif /* FEATURE_FT_KEY_MAP */

FT_LOOKUP_DISPATCH("lookup_longest_match_key")
enum cds_ft_status cds_ft_lookup_longest_match_key(struct cds_ft *ft,
		const uint8_t *key, size_t _key_len, size_t *match_len,
		struct cds_ft_node **result_node)
{
	return (*ft->lookup_longest_match_key_fn)(ft, key, _key_len,
			match_len, result_node);
}

/*
 * Helper: write descent result back into iter for longest_match iter.
 */
static inline
enum cds_ft_status ft_lookup_longest_match_iter_finish(
		enum cds_ft_status ret,
		size_t longest_len, struct cds_ft_node *match_node,
		struct cds_ft_iter *iter)
{
	if (ret < 0) {
		iter->node = NULL;
		iter->status = ret;
		goto end;
	}
	if (longest_len == FT_MATCH_LEN_NONE) {
		iter->node = NULL;
		iter->status = CDS_FT_STATUS_NOT_FOUND;
		goto end;
	}
	iter->node = match_node;
	iter->key_len = longest_len;
	iter->path_len = longest_len + 1;
	iter->status = match_node ? CDS_FT_STATUS_OK : CDS_FT_STATUS_INTERNAL_MATCH;
	iter->cache_valid = true;
	iter_debug_path_snapshot(iter);
end:
	iter_auto_invalidate_cache(iter);
	return iter->status;
}

static FT_LOOKUP_FAST_PATH("lookup_longest_match")
enum cds_ft_status ft_lookup_longest_match_iter_sc(struct cds_ft *ft,
		struct cds_ft_iter *iter)
{
	struct cds_ft_node *match_node = NULL;
	size_t longest_len = 0;
	enum cds_ft_status ret;

	CDS_FT_SCOPED_READER(ft);
	ret = do_cds_ft_lookup_nodc_sc(ft, iter_key(iter), iter->key_len,
			FT_KEY_READABLE_PAD, NULL, iter,
			FT_PREFIX_TRACK_LONGEST, &longest_len, &match_node);
	return ft_lookup_longest_match_iter_finish(ret, longest_len,
			match_node, iter);
}

static FT_LOOKUP_SLOW_PATH("lookup_longest_match")
enum cds_ft_status ft_lookup_longest_match_iter_nosc(struct cds_ft *ft,
		struct cds_ft_iter *iter)
{
	struct cds_ft_node *match_node = NULL;
	size_t longest_len = 0;
	enum cds_ft_status ret;

	CDS_FT_SCOPED_READER(ft);
	ret = do_cds_ft_lookup_nodc_nosc(ft, iter_key(iter), iter->key_len,
			FT_KEY_READABLE_PAD, NULL, iter,
			FT_PREFIX_TRACK_LONGEST, &longest_len, &match_node);
	return ft_lookup_longest_match_iter_finish(ret, longest_len,
			match_node, iter);
}

FT_LOOKUP_DISPATCH("lookup_longest_match")
enum cds_ft_status cds_ft_lookup_longest_match(struct cds_ft *ft,
		struct cds_ft_iter *iter)
{
	ft_iter_assert_bound(ft, iter);
	return (*ft->lookup_longest_match_iter_fn)(ft, iter);
}

/*
 * Return @node's external_nodes (the dup-chain head hanging off an
 * internal or compressed node).  @node must be internal or compressed.
 * Used to recover a cached iterator position's deepest trie node without
 * re-descending: a prefix key sits at an internal/compressed node
 * whose external_nodes == iter->node.
 */
static inline_lookup
struct cds_ft_node *ft_node_external_nodes(struct cds_ft_inode_flag *node)
{
	struct cds_ft_metadata *metadata;

	assert(!ft_node_external(node));
	if (ft_node_compressed(node))
		metadata = cds_ft_item_to_metadata(ft_node_ptr(node));
	else {
		const struct cds_ft_type *type = &ft_types[ft_node_type(node)];

		metadata = cds_ft_item_to_metadata_fast(ft_node_ptr(node),
				type->order);
	}
	return ft_dereference_external(metadata->external_nodes);
}

/*
 * Speculative inequality result-key capture: when the group is configured for
 * speculative skip-compressed lookup with a leaf-key offset, the matched leaf
 * @leaf stores the full result key -- in the byte order the application passed
 * to cds_ft_insert() -- at that offset.  Transform @level bytes of it to the
 * iterator's ordinal (trie) order into @dst and return true; the caller then
 * need not rebuild the key from the descent's compressed-node bytes.
 * ft_key_to_ordinals applies the group's key map, which is a plain copy for an
 * identity map and a per-byte remap otherwise, so the fast path covers
 * non-identity maps too (no identity restriction).  Returns false (copying
 * nothing) on groups without the offset / skip-compressed encoding, so the
 * caller falls back to the descent-built ordinal_key accumulation.
 *
 * This is the result-key source on a configured speculative group: the
 * min-descent intentionally leaves dispatch-irrelevant holes in ordinal_key
 * (it follows skip pointers without filling the spanned bytes), so the leaf
 * copy -- not ordinal_key -- carries the full result key.  Correctness is
 * validated end-to-end by the ordered-iteration / relational invariant tests.
 */
static inline_lookup
bool ft_speculative_keycopy(const struct cds_ft *ft,
		const struct cds_ft_node *leaf,
		uint8_t *dst, ssize_t level)
{
	const struct cds_ft_group *group = ft->group;
	const uint8_t *leaf_key;

	if (!leaf || level < 0)
		return false;
	if (!group->speculative_key_offset_set || !group->speculative ||
			!(group->flags & CDS_FT_FLAG_SKIP_COMPRESSED))
		return false;
	leaf_key = (const uint8_t *) leaf + group->speculative_key_offset;
	ft_key_to_ordinals(dst, leaf_key, (size_t) level, &group->key_map);
	return true;
}

/*
 * Unconditional variant of ft_speculative_keycopy for callers that have already
 * resolved use_keycopy at compile time: the leaf-copy config gate
 * (speculative_key_offset_set && speculative && SKIP_COMPRESSED) is then known
 * to hold, so the runtime re-check folds away.  Copies @level ordinal bytes of
 * @leaf's stored key into @dst; a NULL @leaf (NOT_FOUND) or @level < 0 copies
 * nothing (the result key is then unused).
 */
static inline_lookup
void ft_speculative_keycopy_unconditional(const struct cds_ft *ft,
		const struct cds_ft_node *leaf, uint8_t *dst, ssize_t level)
{
	const struct cds_ft_group *group = ft->group;
	const uint8_t *leaf_key;

	if (!leaf || level < 0)
		return;
	leaf_key = (const uint8_t *) leaf + group->speculative_key_offset;
	ft_key_to_ordinals(dst, leaf_key, (size_t) level, &group->key_map);
}

/*
 * Lazy-ref accessor model (scoped to the
 * ordinal-cell ordered list).  In a cell group with a leaf-key offset and an
 * identity key map, the ordered-iteration result key is held as a LIVE
 * REFERENCE into the matched leaf (iter->node + speculative_key_offset) instead
 * of being copied into iter_key(iter) on every cell-walk step.  That removes
 * the per-step key copy -- the cell walk never touches the leaf otherwise (its
 * node + ord_next co-reside in the 32B cell), so unlike the descent path the
 * leaf load is genuinely saved (the descent's going-up anchor would load it
 * regardless, which is why the by-reference key was a wash there).
 *
 * iter_key(iter) is therefore NOT the current key for such a position; every
 * reader of the current-position key MUST go through ft_iter_read_key().
 * Missing one silently corrupts (e.g. cds_ft_remove_all locating a wrong key).
 */
static inline
bool ft_iter_key_referenced(const struct cds_ft_iter *iter)
{
	const struct cds_ft_group *group = iter->ft->group;

	return group->ordered_list_set && group->speculative_key_offset_set &&
		group->key_map.identity && iter->cache_valid && iter->node;
}

/*
 * Read the iterator's CURRENT-POSITION key.  Returns the live leaf reference
 * when referenced, else the iter_key value.  Correct because every cell-walk /
 * descent result store sets iter->node to the matched leaf (whose stored key IS
 * the current key) and cds_ft_iter_set_key() clears cache_valid AND iter->node,
 * so iter->node + offset is authoritative exactly when cache_valid && node.
 * Valid only while the RCU lock that produced iter->node is held (cross-CS
 * callers cds_ft_iter_bind_key() first).
 */
static inline_lookup
struct ft_ord_cell *ft_ord_cell_cursor(const struct cds_ft_iter *iter);
static size_t ft_rebuild_key_upwalk(const struct cds_ft *ft,
		struct ft_ord_cell *cell, uint8_t *out, size_t max_len);

/*
 * Sentinel stored in iter->key_len by the ordinal-cell land for a VARIABLE-
 * length identity group: the length is DEFERRED and resolved on demand by
 * ft_iter_resolve_key_len() -- from the leaf (key_len_offset) when present, else
 * from the parent up-walk -- so a keyless cell walk reads neither key nor length.
 * SIZE_MAX is never a valid key length (bounded by max_key_len), so a consumer
 * that forgets to resolve hits an obvious overflow, not a silently-stale value.
 */
#define FT_ITER_KEY_LEN_LAZY	((size_t) -1)

/*
 * One-shot structural materialization for a VARIABLE-length EAGER ordered-list
 * iterator (no in-leaf key, no key_len_offset): the parent up-walk derives BOTH
 * the key bytes (into iter_key) AND the length in a SINGLE walk.  Caches the
 * length in iter->key_len (clearing the LAZY sentinel) so a later read_key /
 * resolve_key_len in the same step reuses it instead of walking again.  Returns
 * the length (0 on a NIL key / overflow / missing cell).
 */
static
size_t ft_iter_upwalk_into_buf(struct cds_ft_iter *iter)
{
	size_t max_len = iter->ft->group->max_key_len;
	struct ft_ord_cell *cell = ft_ord_cell_cursor(iter);
	size_t n = 0;

	if (cell)
		n = ft_rebuild_key_upwalk(iter->ft, cell, iter_key(iter), max_len);
	iter->key_len = n;
	iter->key_off = max_len - n;	/* key lives at iter_key[key_off ..) */
	iter->path_len = n + 1;
	return n;
}

static inline
const uint8_t *ft_iter_read_key(const struct cds_ft_iter *iter)
{
	const struct cds_ft_group *group = iter->ft->group;

	if (ft_iter_key_referenced(iter))
		return (const uint8_t *) iter->node + group->speculative_key_offset;
	/*
	 * EAGER ordered-list walk (no in-leaf key): rematerialize the current
	 * key STRUCTURALLY via the parent up-walk into iter_key.  Reached only
	 * when a key consumer asks for the key -- a keyless/count walk never
	 * calls this, so the O(depth) walk is paid strictly on demand.  The
	 * walk recovers ORDINAL bytes from the trie structure, which is what
	 * iter_key holds for ANY key map (consumers remap via
	 * ft_ordinals_to_key), so no identity requirement.  FIXED-length walks
	 * into the buffer (length is group->key_len).  VARIABLE-length derives
	 * the length from the SAME walk, cached via ft_iter_upwalk_into_buf and
	 * coordinated with ft_iter_resolve_key_len through the LAZY sentinel so
	 * one walk serves both.
	 */
	if (group->ordered_list_set && !group->speculative_key_offset_set &&
			iter->cache_valid && iter->node) {
		if (group->key_len == CDS_FT_LEN_VARIABLE) {
			/*
			 * The up-walk fills the key at the buffer TAIL and records
			 * iter->key_off; coordinated with ft_iter_resolve_key_len
			 * through the LAZY sentinel so one walk serves both.
			 */
			if (iter->key_len == FT_ITER_KEY_LEN_LAZY)
				ft_iter_upwalk_into_buf(
					(struct cds_ft_iter *) iter);
			return iter_key(iter) + iter->key_off;
		} else {
			struct ft_ord_cell *cell = ft_ord_cell_cursor(iter);
			size_t max_len = group->max_key_len;
			size_t n;

			if (cell && (n = ft_rebuild_key_upwalk(iter->ft, cell,
					iter_key(iter), max_len)) != 0) {
				((struct cds_ft_iter *) iter)->key_off =
					max_len - n;
				return iter_key(iter) + (max_len - n);
			}
		}
	}
	return iter_key(iter) + iter->key_off;
}

/*
 * Rebuild the ORDINAL key for a cell head @cell of length @key_len by walking
 * UP the parent chain, recovering each level's branch byte structurally:
 *   - external head: its last byte = the head's cell-metadata incoming_byte
 *     (unless its parent is a compressed node, whose key_bytes already span
 *     through the head's position).
 *   - internal node: metadata->incoming_byte (skip the root, which has none).
 *   - compressed node: its key_bytes[] span PLUS metadata->incoming_byte (the
 *     slot byte under which it hangs in its parent -- separate from key_bytes).
 * Fills @out[0..key_len) (caller-sized >= key_len) and returns true when the
 * walk accounts for exactly key_len bytes.
 *
 * This is the structural key source for the ordered-list walk that needs NO
 * speculative_key_offset (in-leaf key) and NO parent-bitmap inversion -- it
 * makes the ordered list usable on an EAGER / no-leaf-key trie.  Valid only
 * while the RCU lock that produced @cell is held continuously.
 */
static
size_t ft_rebuild_key_upwalk(const struct cds_ft *ft, struct ft_ord_cell *cell,
		uint8_t *out, size_t max_len)
{
	struct cds_ft_inode_flag *nf;
	size_t pos = max_len;	/* fill DEEPEST-byte-first leftward from the end */

	(void) ft;
	if (!cell)
		return 0;

	/*
	 * Resolve a cds_ft_merge / graft_swap flip proxy on every parent load: a
	 * concurrent bulk op re-parents nodes via a type-7 proxy installed BEFORE
	 * its drain, so an up-walk running under the reader's RCU lock would
	 * otherwise dereference the proxy as a node.  Gives the view-appropriate
	 * (old-or-merged) parent, consistent across the walk.
	 */
	nf = ft_resolve_flip_proxy(rcu_dereference(cell->parent));

	/*
	 * Fill the buffer FROM THE END: write the deepest (leaf-edge) byte at
	 * out[max_len-1] and grow leftward, so the key ends up in correct order
	 * occupying out[pos .. max_len) with NO reversal and the length DERIVED
	 * from the walk (pos drops by however many bytes the walk contributes --
	 * no key_len needed, so variable-length keys with no in-leaf length work).
	 * A compressed span lands as one contiguous forward memcpy (its key_bytes
	 * are already in key order); on underflow past out[0], fail (return 0).
	 * The key STARTS at out[max_len - returned]; the caller keeps that offset.
	 *
	 * Head's last byte: when it hangs off an internal node it sits in a slot
	 * whose byte is the head's cell-metadata incoming_byte; when its parent is
	 * a compressed node the head is that node's child and carries no separate
	 * edge byte (the compressed key_bytes run through the head's position).
	 */
	if (!nf) {
		/* Parentless head: its byte stands alone. */
		struct cds_ft_metadata *hmeta = cds_ft_item_to_metadata(cell);

		if (pos == 0)
			return 0;
		out[--pos] = (uint8_t) hmeta->incoming_byte;
	} else if (!ft_node_compressed(ft_resolve_skip_compressed(ft, nf))) {
		/*
		 * Parent is an internal (slot-array) node.  Write the head's edge
		 * byte ONLY when the head hangs off a SLOT.  A PREFIX key sits at the
		 * parent's external_nodes -- the key ENDS at the parent, so its last
		 * byte IS the parent's own incoming edge (written when the parent is
		 * processed below) and must not be double-counted here.  Fixed-length
		 * tries have no prefix keys, so this is always a slot head there.
		 */
		struct cds_ft_metadata *nmeta = ft_flag_to_metadata(ft, nf);

		if (cell->node !=
				ft_dereference_external(nmeta->external_nodes)) {
			struct cds_ft_metadata *hmeta =
				cds_ft_item_to_metadata(cell);

			if (pos == 0)
				return 0;
			out[--pos] = (uint8_t) hmeta->incoming_byte;
		}
	}
	/* else parent compressed: the head byte is covered by its key_bytes. */

	while (nf) {
		struct cds_ft_inode_flag *rnf = ft_resolve_skip_compressed(ft, nf);
		struct cds_ft_metadata *meta = ft_flag_to_metadata(ft, nf);

		if (ft_node_compressed(rnf)) {
			const struct cds_ft_compressed_node *cn =
				(const struct cds_ft_compressed_node *)
				ft_node_ptr(rnf);
			unsigned int len = cn->len;

			/*
			 * Compressed span in key order (key_bytes[0..len)): it sits
			 * immediately to the LEFT of what we've written so far, so a
			 * single forward memcpy places it correctly.
			 */
			if (pos < len)
				return 0;
			pos -= len;
			memcpy(&out[pos], cn->key_bytes, len);
		}
		/*
		 * This node's incoming edge byte (the slot byte in its parent).
		 * Read meta->parent ONCE (resolving a concurrent bulk op's flip
		 * proxy) and use it for both the compressed-parent test and the
		 * advance.  Contributed only when the PARENT is an internal
		 * (slot-array) node: a compressed parent's last key_byte already IS
		 * this edge, counted when that parent is processed -- so skip it here
		 * (mirrors the head's skip when its parent is compressed).  The root
		 * has no parent and contributes none.
		 */
		{
			struct cds_ft_inode_flag *parent =
				ft_resolve_flip_proxy(rcu_dereference(meta->parent));

			if (parent && !ft_node_compressed(
					ft_resolve_skip_compressed(ft, parent))) {
				if (pos == 0)
					return 0;
				out[--pos] = (uint8_t) meta->incoming_byte;
			}
			nf = parent;
		}
	}

	/* Key now occupies out[pos .. max_len) in key order; length = max_len - pos. */
	return max_len - pos;
}

/*
 * Resolve (and cache) the iterator's current-position key length.  For a
 * deferred-length cell position it reads node->key_len from the leaf once and
 * caches it into iter->key_len (and path_len); otherwise returns iter->key_len
 * unchanged (a no-op for fixed-length, non-identity, descent and non-cell
 * positions).  EVERY reader of the current-position length (cds_ft_iter_get_key,
 * cds_ft_remove*, the skip rebuilds, bind, the max-key-len scan) must call this
 * before reading iter->key_len.  The LAZY sentinel is only ever set with
 * cache_valid && node, so the leaf read is safe.
 */
static inline
size_t ft_iter_resolve_key_len(struct cds_ft_iter *iter)
{
	if (caa_unlikely(iter->key_len == FT_ITER_KEY_LEN_LAZY)) {
		/*
		 * VARIABLE-length EAGER ordered-list (no in-leaf KEY at
		 * speculative_key_offset): the key source is the iter buffer, and
		 * clearing the LAZY sentinel doubles as "buffer filled" for
		 * ft_iter_read_key -- so the length MUST come from the parent
		 * up-walk, which fills iter_key (+ key_off) in the same walk that
		 * derives the length, even when an in-leaf LENGTH (key_len_offset)
		 * is configured.  Only a leaf-referenced key (in-leaf key present)
		 * may take the leaf-length shortcut: its key reads never touch the
		 * buffer.
		 */
		if (!iter->ft->group->key_len_offset_set ||
				!iter->ft->group->speculative_key_offset_set) {
			ft_iter_upwalk_into_buf(iter);
			return iter->key_len;
		}
		iter->key_len = *(const size_t *) ((const char *) iter->node +
			iter->ft->group->key_len_offset);
		iter->path_len = iter->key_len + 1;
	}
	return iter->key_len;
}

/*
 * Copy a live leaf-referenced key into iter_key so it survives the position
 * being detached (UNCACHED auto-invalidate, bind, any cache_valid clear).  A
 * no-op self-copy when the key is already a value there.  Bytes are ordinal
 * (ft_iter_key_referenced requires an identity map).  Resolves the length first
 * so a deferred-length position materializes both before its node is dropped.
 */
static inline
void ft_iter_materialize_key(struct cds_ft_iter *iter)
{
	size_t klen = ft_iter_resolve_key_len(iter);
	const uint8_t *cur = ft_iter_read_key(iter);

	/*
	 * Pin the current key at the FRONT of iter_key (key_off = 0).  The hot
	 * cell-walk read keeps the up-walk key at the buffer TAIL (no move), but
	 * materialize is the bind / UNCACHED path: the saved key must then be
	 * re-descended from, and the relational descent reads its search key and
	 * writes its result into the SAME iter_key buffer -- which is only safe
	 * (result == search before divergence) when the key starts at offset 0.
	 * memmove because @cur (the up-walk tail, or a leaf reference) may overlap.
	 */
	if (cur != iter_key(iter)) {
		memmove(iter_key(iter), cur, klen);
		iter->key_off = 0;
	}
}


/*
 * Land an ordinal-cell walk result on @iter: materialize the head's key from
 * its leaf and cache the cell as the walk cursor.  @cell == NULL reports
 * NOT_FOUND (end of list).  Shared by the cell fast path and the O(1)
 * lookup_first / lookup_last endpoints.  Returns iter->status.
 */
static inline_lookup
enum cds_ft_status ft_ord_cell_iter_land(struct cds_ft *ft,
		struct cds_ft_iter *iter, struct ft_ord_cell *cell)
{
	struct cds_ft_node *node;
	size_t rlen;

	if (!cell) {
		iter->node = NULL;
		iter->ord_cell = NULL;
		iter->ord_cell_node = NULL;
		iter->cache_valid = false;
		iter_debug_path_update(iter);
		iter->status = CDS_FT_STATUS_NOT_FOUND;
		return iter->status;
	}
	node = cell->node;
	iter->node = node;
	iter->ord_cell = cell;
	iter->ord_cell_node = node;
	iter->cache_valid = true;
	/*
	 * Materialize as little as possible -- the cell walk's point is that a
	 * keyless / count traversal touches NO leaf:
	 *
	 *  - Identity map (key read in place from the leaf when an in-leaf key
	 *    is configured, see ft_iter_key_referenced) or EAGER group (no
	 *    in-leaf key; the up-walk in ft_iter_read_key recovers the ordinal
	 *    bytes structurally, identity or not): no key copy.  A VARIABLE-
	 *    length group also DEFERS the length: iter->key_len is the LAZY
	 *    sentinel, resolved on demand by ft_iter_resolve_key_len() only in
	 *    the key-consuming ops (get_key / remove / skip / bind).  So a
	 *    keyless variable-length walk reads neither the key nor the length
	 *    from the leaf.  A FIXED-length group takes its length from
	 *    group->key_len (no leaf touch either).
	 *  - Non-identity map WITH an in-leaf key: the leaf bytes are in
	 *    application order, so they must be remapped to ordinal order now,
	 *    which needs the length now -- read it (leaf, for variable) and copy
	 *    into iter_key (ft_iter_key_referenced is then false, consumers use
	 *    the buffer).
	 */
	if (caa_likely(ft->group->key_map.identity ||
			!ft->group->speculative_key_offset_set)) {
		if (ft->group->key_len != CDS_FT_LEN_VARIABLE) {
			iter->key_len = ft->group->key_len;
			iter->path_len = ft->group->key_len + 1;
		} else {
			iter->key_len = FT_ITER_KEY_LEN_LAZY;
			iter->path_len = 0;	/* materialized with the length */
		}
	} else {
		rlen = (ft->group->key_len != CDS_FT_LEN_VARIABLE) ?
			ft->group->key_len :
			*(const size_t *) ((const char *) node +
				ft->group->key_len_offset);
		ft_speculative_keycopy_unconditional(ft, node, iter_key(iter),
			(ssize_t) rlen);
		iter->key_len = rlen;
		iter->path_len = rlen + 1;
	}
	iter_debug_path_update(iter);
	iter->status = CDS_FT_STATUS_OK;
	return iter->status;
}

/*
 * True when the ordinal-cell O(1) endpoints / fast path can serve @iter: the
 * list is enabled, the result key + length are recoverable from the leaf, and
 * the traversal is unscoped.
 */
static inline_lookup
bool ft_ord_cell_fastpath_ok(const struct cds_ft *ft,
		const struct cds_ft_iter *iter)
{
	/*
	 * EAGER structural up-walk: an ordered-list group with NO in-leaf key
	 * (no speculative_key_offset) gets BOTH the result key AND its length
	 * from the parent up-walk (ft_iter_read_key / ft_iter_resolve_key_len),
	 * for fixed OR variable length -- no in-leaf key and no key_len_offset
	 * needed.  The walk recovers ORDINAL bytes, so it serves any key map
	 * (a non-identity map is applied on result-key copy-out).
	 */
	bool up_walk = ft->group->ordered_list_set &&
		!ft->group->speculative_key_offset_set;

	return ft->group->ordered_list_set &&
		(up_walk ||
		 /*
		  * In-leaf key (ft_ord_cell_iter_land reads/remaps it from the
		  * leaf): needs the length too -- fixed, or variable with an
		  * in-leaf length (key_len_offset).
		  */
		 (ft->group->key_len != CDS_FT_LEN_VARIABLE ||
			ft->group->key_len_offset_set)) &&
		iter->prefix_len == 0;
}

/*
 * Resolve the current head's cell for a cell-walk step: use the cached cursor
 * when it still refers to iter->node (no leaf touch), else re-enter the walk
 * via the head's prev (one leaf load -- the per-walk-entry cost).
 */
static inline_lookup
struct ft_ord_cell *ft_ord_cell_cursor(const struct cds_ft_iter *iter)
{
	if (iter->ord_cell_node == iter->node)
		return iter->ord_cell;
	return ft_ord_cell_ptr(rcu_dereference(iter->node->prev));
}
