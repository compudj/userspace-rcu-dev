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

/*
 * REKEY coherence second walk (cds_ft_attr_set_rekey_coherence): true when the
 * descent that landed on @found is coherent with respect to a concurrent in-trie
 * rekey -- i.e. @found's key, rematerialized from the trie STRUCTURE by the
 * parent-pointer up-walk, still equals the ordinal key bytes @ord_key[0..key_len)
 * the reader descended with.  A concurrent merge_at that moved @found's subtree
 * to a different prefix while this descent was in flight leaves it landed on a
 * leaf whose structural key now differs -> returns false, and the caller
 * re-descends from the root.  Deliberately reads the STRUCTURAL key (the up-walk,
 * as cds_ft_node_get_key's non-speculative branch does), never the leaf's stored
 * speculative key: the check is about the leaf's POSITION, not its stamped bytes.
 * Only reached when ft->rekey_coherence, which is ANDed with ft->ordered_list at
 * create, so the cell (the up-walk source) always exists.  Must run under the
 * same RCU read lock that produced @found (ft_rebuild_key_upwalk's contract).
 */
#ifdef FEATURE_FT_FAULT_INJECT
extern long cds_ft_fault_rekey_countdown;
#endif

static inline_lookup
bool ft_rekey_descent_coherent(const struct cds_ft *ft,
		const struct cds_ft_node *found,
		const uint8_t *ord_key, size_t key_len)
{
	const struct cds_ft_group *group = ft->group;
	struct ft_ord_cell *cell;
	uint8_t scratch[FT_MAX_KEY_LEN];
	size_t max_len = group->max_key_len;
	size_t klen;

#ifdef FEATURE_FT_FAULT_INJECT
	/* Force one coherence miss on demand to exercise the re-descend loop. */
	if (caa_unlikely(cds_ft_fault_rekey_countdown >= 0)) {
		if (cds_ft_fault_rekey_countdown == 0) {
			cds_ft_fault_rekey_countdown = -1;
			return false;
		}
		cds_ft_fault_rekey_countdown--;
	}
#endif
	cell = ft_ord_cell_ptr(ft_dereference_prev_resolved(
			(struct cds_ft_node *) found));
	klen = ft_rebuild_key_upwalk(ft, cell, scratch, max_len);
	return klen == key_len &&
		memcmp(scratch + (max_len - klen), ord_key, key_len) == 0;
}

/*
 * REKEY-coherent exact lookup specializations (installed on ft->lookup_key_fn in
 * place of ft_lookup_precise_{sc,nosc} when ft->rekey_coherence).  Same descent
 * as the plain variant, wrapped in the second-walk re-descend loop UNDER the one
 * read lock: on a coherence miss (a peer rekey restructured the path) the whole
 * descent is thrown away and retried from the root.  A hit whose structural key
 * matches, or any non-OK status, returns immediately.  Not in the hot .text
 * cluster (opt-in path); real symbols (fn-ptr targets), so not inlined.
 */
static
enum cds_ft_status ft_lookup_precise_coherent_sc(struct cds_ft *ft,
		const uint8_t *key, size_t key_len, size_t key_readable_pad,
		struct cds_ft_node **result_node)
{
	enum cds_ft_status status;

	key_len = ft_key_len(ft, key_len);
	if (!valid_key_len(ft, key_len))
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	CDS_FT_SCOPED_READER(ft);
	for (;;) {
		status = do_cds_ft_lookup_nodc_sc(ft, key, key_len,
				key_readable_pad, result_node, NULL,
				FT_PREFIX_TRACK_NONE, NULL, NULL);
		if (status != CDS_FT_STATUS_OK ||
				ft_rekey_descent_coherent(ft, *result_node,
					key, key_len))
			break;
	}
	return status;
}

static
enum cds_ft_status ft_lookup_precise_coherent_nosc(struct cds_ft *ft,
		const uint8_t *key, size_t key_len, size_t key_readable_pad,
		struct cds_ft_node **result_node)
{
	enum cds_ft_status status;

	key_len = ft_key_len(ft, key_len);
	if (!valid_key_len(ft, key_len))
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	CDS_FT_SCOPED_READER(ft);
	for (;;) {
		status = do_cds_ft_lookup_nodc_nosc(ft, key, key_len,
				key_readable_pad, result_node, NULL,
				FT_PREFIX_TRACK_NONE, NULL, NULL);
		if (status != CDS_FT_STATUS_OK ||
				ft_rekey_descent_coherent(ft, *result_node,
					key, key_len))
			break;
	}
	return status;
}

#ifdef FEATURE_FT_KEY_MAP
/*
 * Non-identity key_map REKEY-coherent exact lookup: remap once, then the same
 * second-walk re-descend loop.  The up-walk rematerializes ORDINAL bytes, so the
 * coherence compare is against @ordinals (ordinal space), not the caller key.
 */
static
enum cds_ft_status ft_lookup_key_coherent_nonidentity(struct cds_ft *ft,
		const uint8_t *key, size_t key_len, size_t key_readable_pad,
		struct cds_ft_node **result_node)
{
	uint8_t ordinals[FT_MAX_KEY_LEN];
	enum cds_ft_status status;

	(void) key_readable_pad;
	key_len = ft_key_len(ft, key_len);
	if (!valid_key_len(ft, key_len))
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	CDS_FT_SCOPED_READER(ft);
	ft_key_to_ordinals(ordinals, key, key_len, &ft->group->key_map);
	for (;;) {
		status = do_cds_ft_lookup(ft, ordinals, key_len,
				FT_KEY_READABLE_PAD, result_node, NULL,
				FT_PREFIX_TRACK_NONE, NULL, NULL, false);
		if (status != CDS_FT_STATUS_OK ||
				ft_rekey_descent_coherent(ft, *result_node,
					ordinals, key_len))
			break;
	}
	return status;
}
#endif /* FEATURE_FT_KEY_MAP */

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

/*
 * REKEY-coherent iterator EXACT lookup (installed on ft->lookup_iter_fn when
 * ft->rekey_coherence): the iter-form sibling of ft_lookup_precise_coherent_*.
 * Same second-walk re-descend loop under the one read lock, but the search key
 * is snapshotted first: do_cds_ft_lookup_inner's epilogue
 * (iter_auto_invalidate_cache, UNCACHED mode) may materialize the FOUND leaf's
 * key into iter_key(iter), so a naive retry would descend with the wrong key --
 * descend and compare against the stable local copy instead.  The exact iter
 * descent always top-descends from the root (no cross-call cache path, unlike
 * the inequality descent), so re-calling it is a clean fresh descent.
 */
static
enum cds_ft_status ft_lookup_iter_precise_coherent_sc(struct cds_ft *ft,
		struct cds_ft_iter *iter)
{
	enum cds_ft_status status;
	uint8_t search[FT_MAX_KEY_LEN];
	size_t klen = iter->key_len;

	CDS_FT_SCOPED_READER(ft);
	FT_TP_ITER_KEY(lookup_enter, iter);
	memcpy(search, iter_key(iter), klen);
	for (;;) {
		status = do_cds_ft_lookup_nodc_sc(ft, search, klen,
				FT_KEY_READABLE_PAD, NULL, iter,
				FT_PREFIX_TRACK_NONE, NULL, NULL);
		if (status != CDS_FT_STATUS_OK ||
				ft_rekey_descent_coherent(ft, iter->node,
					search, klen))
			break;
	}
	FT_TP(lookup_exit, (int) status);
	return status;
}

static
enum cds_ft_status ft_lookup_iter_precise_coherent_nosc(struct cds_ft *ft,
		struct cds_ft_iter *iter)
{
	enum cds_ft_status status;
	uint8_t search[FT_MAX_KEY_LEN];
	size_t klen = iter->key_len;

	CDS_FT_SCOPED_READER(ft);
	FT_TP_ITER_KEY(lookup_enter, iter);
	memcpy(search, iter_key(iter), klen);
	for (;;) {
		status = do_cds_ft_lookup_nodc_nosc(ft, search, klen,
				FT_KEY_READABLE_PAD, NULL, iter,
				FT_PREFIX_TRACK_NONE, NULL, NULL);
		if (status != CDS_FT_STATUS_OK ||
				ft_rekey_descent_coherent(ft, iter->node,
					search, klen))
			break;
	}
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
