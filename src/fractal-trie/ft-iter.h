// SPDX-FileCopyrightText: 2012-2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-only

/*
 * src/fractal-trie/ft-iter.h
 *
 * Userspace RCU library - Fractal Trie: the public iterator (create/destroy/get_key/set_key/bind/reset/copy) and batched cell next/prev iteration.
 *
 * Implementation unit: #included once into the fractal-trie.c translation
 * unit (preserves cross-module inlining).  Not a standalone header.
 */
#ifndef FRACTAL_TRIE_IMPL
#error "ft-iter.h is an implementation unit; #include it from fractal-trie.c only"
#endif

enum cds_ft_status cds_ft_iter_create(struct cds_ft *ft, struct cds_ft_iter **result_iter)
{
	size_t max_key_len = ft->group->max_key_len;
	/*
	 * Tail pad of FT_KEY_READABLE_PAD bytes after the key buffer so
	 * the descent can SIMD-load 32 bytes past key[key_len-1] without
	 * faulting.  The iter-based lookup paths pass FT_KEY_READABLE_PAD
	 * as @key_readable_pad (bytes safely loadable past key end).
	 */
	size_t key_size  = (max_key_len + FT_KEY_READABLE_PAD) * sizeof(uint8_t);
	struct cds_ft_iter *iter = calloc(1, sizeof(*iter) + key_size);

	CDS_FT_SCOPED_READER(ft);
	if (!iter) {
		*result_iter = NULL;
		return CDS_FT_STATUS_MEMORY_ERROR;
	}
	iter->ft = ft;
	iter->cache_mode = CDS_FT_ITER_CACHED;
	*result_iter = iter;
	FT_TP(iter_create, (const void *) ft, (const void *) *result_iter);
	return CDS_FT_STATUS_OK;
}

void cds_ft_iter_destroy(struct cds_ft_iter *iter)
{
	FT_TP(iter_destroy, (const void *) iter->ft, (const void *) iter);
	free(iter);
}

enum cds_ft_status cds_ft_iter_status(const struct cds_ft_iter *iter)
{
	return iter->status;
}

enum cds_ft_status cds_ft_iter_get_key(struct cds_ft_iter *iter,
		uint8_t *result_key, size_t result_key_max_len, size_t *result_key_len)
{
	size_t klen = ft_iter_resolve_key_len(iter);

	*result_key_len = klen;
	if (klen > result_key_max_len)
		return CDS_FT_STATUS_OVERFLOW_ERROR;
	/*
	 * Lazy-ref: when the current key is a live reference into iter->node,
	 * read it from the leaf; else from the iter_key value.  ft_iter_resolve_
	 * key_len above materialized a deferred (variable-length) length from the
	 * same leaf.  Same continuous-RCU-lock contract as reusing the cached
	 * position (cross-CS callers cds_ft_iter_bind_key() first).
	 */
	ft_ordinals_to_key(result_key, ft_iter_read_key(iter), klen,
			&iter->ft->group->key_map);
	return CDS_FT_STATUS_OK;
}

enum cds_ft_status cds_ft_node_get_key(const struct cds_ft *ft,
		const struct cds_ft_node *node, uint8_t *result_key,
		size_t result_key_max_len, size_t *result_key_len)
{
	const struct cds_ft_group *group = ft->group;
	const uint8_t *ordinals;
	size_t klen;
	uint8_t scratch[FT_MAX_KEY_LEN];

	if (!node)
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	if (group->speculative_key_offset_set) {
		/*
		 * In-leaf key: the head stores its ordinal key bytes (and, for a
		 * variable-length group, its length) directly.  Read them in place.
		 */
		ordinals = (const uint8_t *) node + group->speculative_key_offset;
		klen = (group->key_len != CDS_FT_LEN_VARIABLE) ? group->key_len :
			*(const size_t *) ((const char *) node +
				group->key_len_offset);
	} else {
		/*
		 * No in-leaf key: rebuild the ordinal key by the structural parent
		 * up-walk from the head's cell (the same EAGER source cds_ft_iter_
		 * get_key uses).  Requires the cell to exist -- i.e. the ordered list
		 * enabled, since a list-off trie allocates no cells (head->prev is the
		 * flagged parent directly, which is NOT an up-walk source).  The walk
		 * fills @scratch from the tail and returns the length; the key starts
		 * at @scratch[max-len].
		 */
		if (ft->ordered_list) {
			struct ft_ord_cell *cell = ft_ord_cell_ptr(
				rcu_dereference(((struct cds_ft_node *) node)->prev));
			size_t max_len = group->max_key_len;

			klen = ft_rebuild_key_upwalk(ft, cell, scratch, max_len);
			ordinals = scratch + (max_len - klen);
		} else {
			/* List off: no cell, no in-leaf key -> not materializable. */
			return CDS_FT_STATUS_NOT_FOUND;
		}
	}
	*result_key_len = klen;
	if (klen > result_key_max_len)
		return CDS_FT_STATUS_OVERFLOW_ERROR;
	ft_ordinals_to_key(result_key, ordinals, klen, &group->key_map);
	return CDS_FT_STATUS_OK;
}

enum cds_ft_status cds_ft_iter_get_prefix(struct cds_ft_iter *iter,
		uint8_t *result_key, size_t result_key_max_len, size_t *result_key_len)
{
	*result_key_len = iter->prefix_len;
	if (iter->prefix_len > result_key_max_len)
		return CDS_FT_STATUS_OVERFLOW_ERROR;
	ft_ordinals_to_key(result_key, iter_key(iter), iter->prefix_len,
			&iter->ft->group->key_map);
	return CDS_FT_STATUS_OK;
}

/*
 * Internal: set the iterator's search key from ALREADY-ORDINAL bytes (no key
 * map application; the public cds_ft_iter_set_key remaps and validates).  Used
 * by the bulk-op machinery, which converts the caller's key once at the public
 * entry and threads the ordinal form internally.
 */
static
void ft_iter_set_key_ordinals(struct cds_ft_iter *iter, const uint8_t *ordinals,
		size_t key_len)
{
	/*
	 * Setting a new key invalidates the cached position: the next
	 * operation re-descends from the root by the new key.
	 */
	memcpy(iter_key(iter), ordinals, key_len);
	iter->cache_valid = false;
	iter_debug_path_clear(iter);
	iter->path_len = 0;
	iter->key_len = key_len;
	iter->key_off = 0;	/* search key sits at the front of iter_key */
}

enum cds_ft_status cds_ft_iter_set_key(struct cds_ft_iter *iter, const uint8_t *key, size_t key_len)
{
	const struct cds_ft_key_map *km = &iter->ft->group->key_map;
	uint8_t ordinal_buf[FT_MAX_KEY_LEN];
	const uint8_t *key_ordinals;

	key_len = ft_key_len(iter->ft, key_len);
	FT_TP(iter_set_key_enter, (const void *) iter->ft, (const void *) iter,
		key, key_len == CDS_FT_LEN_ERROR ? 0 : key_len,
		(int) iter->key_len, (int) iter->path_len);
	if (key_len > iter->ft->group->max_key_len)
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	if (caa_likely(km->identity)) {
		key_ordinals = key;
	} else {
		ft_key_to_ordinals(ordinal_buf, key, key_len, km);
		key_ordinals = ordinal_buf;
	}
	ft_iter_set_key_ordinals(iter, key_ordinals, key_len);
	FT_TP(iter_set_key_exit, (const void *) iter->ft, (const void *) iter,
		(int) iter->path_len);
	return CDS_FT_STATUS_OK;
}

/*
 * The prefix is a subset of the current key. Set the key before setting
 * the prefix length.
 */
enum cds_ft_status cds_ft_iter_set_prefix_len(struct cds_ft_iter *iter, size_t prefix_len)
{
	if (prefix_len > ft_iter_resolve_key_len(iter)) {
		FT_TP(iter_set_prefix_len, (const void *) iter->ft,
			(const void *) iter, (int) prefix_len);
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}
	iter->prefix_len = prefix_len;
	FT_TP(iter_set_prefix_len, (const void *) iter->ft,
		(const void *) iter, (int) prefix_len);
	return CDS_FT_STATUS_OK;
}

void cds_ft_iter_reset(struct cds_ft_iter *iter)
{
	FT_TP(iter_reset, (const void *) iter->ft, (const void *) iter);
	iter->cache_valid = false;
	iter_debug_path_clear(iter);
	iter->status = CDS_FT_STATUS_OK;
	iter->path_len = 0;
	iter->key_len = 0;
	iter->prefix_len = 0;
	iter->node = NULL;
#ifdef DEBUG_CLEAR_ITER
	{
		const struct cds_ft_group *ft_group = iter->ft->group;

		/* Reset to 0 for debugging. */
		memset(iter_key(iter), 0, ft_group->max_key_len * sizeof(uint8_t));
	}
#endif
}

void cds_ft_iter_bind_key(struct cds_ft_iter *iter)
{
	FT_TP(iter_bind_key, (const void *) iter->ft, (const void *) iter);
	/*
	 * Materialize a live leaf-referenced key into the iterator's own buffer
	 * BEFORE detaching, so a cross-critical-section resume re-descends from
	 * the stable buffer rather than the (post-unlock, possibly reclaimed)
	 * leaf.  A no-op self-copy for groups whose key is already a value, in
	 * which case this is a plain invalidate.
	 */
	ft_iter_materialize_key(iter);
	iter->cache_valid = false;
	iter_debug_path_clear(iter);
	iter->path_len = 0;
	iter->node = NULL;
}

void cds_ft_iter_copy(struct cds_ft_iter *dst, const struct cds_ft_iter *src)
{
	dst->status = src->status;
	dst->cache_mode = src->cache_mode;
	dst->cache_valid = src->cache_valid;
	dst->path_len = src->path_len;
	dst->key_len = src->key_len;
	dst->key_off = src->key_off;
	dst->prefix_len = src->prefix_len;
	dst->node = src->node;
	dst->ord_cell = src->ord_cell;
	dst->ord_cell_node = src->ord_cell_node;
#ifdef URCU_FRACTAL_TRIE_DEBUG_PATH
	dst->gp_state = src->gp_state;
	dst->gp_state_valid = src->gp_state_valid;
#endif
	/*
	 * Copy the key buffer only when the source key is a VALUE there.  It is
	 * NOT one when:
	 *  - the key is a live leaf reference (identity cell position with an
	 *    in-leaf key): iter_key(src) is stale; @dst shares src->node +
	 *    cache_valid, so it reads the key from the same leaf;
	 *  - the length is the deferred LAZY sentinel (EAGER identity cell
	 *    position, no in-leaf key): nothing was materialized yet; @dst
	 *    shares the position and re-derives key + length from the up-walk
	 *    on demand.  Copying would be a SIZE_MAX memcpy.
	 * Copy at key_off: an up-walk-materialized key lives at the buffer
	 * TAIL, mirrored to the same offset in @dst (key_off copied above).
	 */
	if (!ft_iter_key_referenced(src) &&
			src->key_len != FT_ITER_KEY_LEN_LAZY)
		memcpy(iter_key(dst) + src->key_off,
			iter_key(src) + src->key_off, src->key_len);
}

struct cds_ft_node *cds_ft_iter_node(const struct cds_ft_iter *iter)
{
	return iter->node;
}

/*
 * Iterator-free ordered batched gather, shared by cds_ft_cell_next_batch /
 * cds_ft_cell_prev_batch.  Walks @ft's ordered cell list from @cursor (NULL =
 * the list minimum for forward, maximum for reverse), emits up to @cap opaque
 * CELL handles into @buf, and writes the resume cell -- the one after the last
 * emitted, NULL at the end -- to @next_cursor.  The cmm_ptr_eq physical-next
 * prediction (post-compaction the cells are contiguous in key order at @stride,
 * so a cell's ord_next/ord_prev usually resolves to cur +/- stride; cmm_ptr_eq
 * validates the arithmetic guess and lets the compiler address the NEXT load off
 * it, breaking the dependent-load chain so the gather pipelines) pays only on a
 * compacted arena and is a no-op otherwise.  @forward is a literal at both call
 * sites so always_inline folds the direction out.
 *
 * Cell-native: the cursor, the emitted handles, and the resume cursor are all
 * CELLS, so the walk never touches an external head node -- not to emit, not to
 * resume (no node<->cell round-trip, hence no resume cache is needed).  The
 * caller recovers the node from a cell via cds_ft_cell_node() (an inlined load
 * of the hot cell line, not a cold node->prev) and materializes keys lazily via
 * cds_ft_cell_get_key(); a keyless (no in-leaf key) ordered scan thus touches
 * ZERO external-head cachelines in the library.
 *
 * ORDERED-LIST ONLY: the step IS the cell ord_next / ord_prev walk.  A list-off
 * trie has no cell list, so it yields CDS_FT_STATUS_NOT_SUPPORTED (*count = 0,
 * *next_cursor = NULL): use the iterator (cds_ft_next / cds_ft_prev, structural
 * descent) there.
 *
 * RCU CONTRACT: @cursor and every emitted cell are valid only while the read
 * lock that produced @cursor is held continuously (see cds_ft_cell_get_key).
 */
static inline __attribute__((always_inline))
enum cds_ft_status ft_cell_batch_dir(struct cds_ft *ft,
		const struct cds_ft_cell *cursor, const struct cds_ft_cell **buf,
		size_t cap, size_t *count,
		const struct cds_ft_cell **next_cursor, const bool forward)
{
	size_t n = 0;
	const uintptr_t stride = (uintptr_t) 1 << FT_ORD_CELL_ALLOC_ORDER;
	struct ft_ord_cell *cur;

	CDS_FT_ASSERT_RCU_READ_LOCKED(ft);
	*next_cursor = NULL;
	*count = 0;
	if (caa_unlikely(!ft->ordered_list))
		return CDS_FT_STATUS_NOT_SUPPORTED;
	if (caa_unlikely(cap == 0))
		return CDS_FT_STATUS_OK;
	if (cursor)
		cur = (struct ft_ord_cell *) cursor;	/* resume AT the cell */
	else
		cur = ft_ord_cell_resolve_ord(forward ?
			&ft->ord_cell_head : &ft->ord_cell_tail);
	while (n < cap && cur) {
		struct ft_ord_cell *loaded, *guess;

		buf[n++] = (const struct cds_ft_cell *) cur;
		loaded = forward ?
			ft_ord_cell_resolve_ord(&cur->ord_next) :
			ft_ord_cell_resolve_ord(&cur->ord_prev);
		guess = (struct ft_ord_cell *) (forward ?
			(uintptr_t) cur + stride :
			(uintptr_t) cur - stride);
		cur = (loaded && cmm_ptr_eq(loaded, guess)) ? guess : loaded;
	}
	*next_cursor = (const struct cds_ft_cell *) cur;
	*count = n;
	return CDS_FT_STATUS_OK;
}

enum cds_ft_status cds_ft_cell_next_batch(struct cds_ft *ft,
		const struct cds_ft_cell *cursor, const struct cds_ft_cell **buf,
		size_t cap, size_t *count, const struct cds_ft_cell **next_cursor)
{
	return ft_cell_batch_dir(ft, cursor, buf, cap, count, next_cursor, true);
}

enum cds_ft_status cds_ft_cell_prev_batch(struct cds_ft *ft,
		const struct cds_ft_cell *cursor, const struct cds_ft_cell **buf,
		size_t cap, size_t *count, const struct cds_ft_cell **next_cursor)
{
	return ft_cell_batch_dir(ft, cursor, buf, cap, count, next_cursor, false);
}

/* Invariant offset of the head-node pointer within the opaque cell. */
size_t cds_ft_cell_node_offset(void)
{
	return offsetof(struct ft_ord_cell, node);
}

/*
 * Materialize a key from an opaque cell handle (the lazy companion to the cell
 * batch).  Same sources as cds_ft_node_get_key -- in-leaf key when configured,
 * else the parent up-walk -- but driven from the CELL, so the up-walk path never
 * touches the external head node.
 */
enum cds_ft_status cds_ft_cell_get_key(const struct cds_ft *ft,
		const struct cds_ft_cell *cell_opaque, uint8_t *result_key,
		size_t result_key_max_len, size_t *result_key_len)
{
	const struct cds_ft_group *group = ft->group;
	struct ft_ord_cell *cell = (struct ft_ord_cell *) cell_opaque;
	const uint8_t *ordinals;
	size_t klen;
	uint8_t scratch[FT_MAX_KEY_LEN];

	if (!cell)
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	if (group->speculative_key_offset_set) {
		const struct cds_ft_node *node = rcu_dereference(cell->node);

		ordinals = (const uint8_t *) node + group->speculative_key_offset;
		klen = (group->key_len != CDS_FT_LEN_VARIABLE) ? group->key_len :
			*(const size_t *) ((const char *) node +
				group->key_len_offset);
	} else {
		size_t max_len = group->max_key_len;

		klen = ft_rebuild_key_upwalk(ft, cell, scratch, max_len);
		ordinals = scratch + (max_len - klen);
	}
	*result_key_len = klen;
	if (klen > result_key_max_len)
		return CDS_FT_STATUS_OVERFLOW_ERROR;
	ft_ordinals_to_key(result_key, ordinals, klen, &group->key_map);
	return CDS_FT_STATUS_OK;
}

bool cds_ft_group_ordered_list(const struct cds_ft_group *group)
{
	return group->ordered_list_set;
}

enum cds_ft_status cds_ft_iter_set_cache_mode(struct cds_ft_iter *iter,
		enum cds_ft_iter_cache_mode mode)
{
	switch (mode) {
	case CDS_FT_ITER_CACHED:
		break;
	case CDS_FT_ITER_UNCACHED:
		/*
		 * Switching to uncached mode: any previously cached
		 * path may become stale if the caller drops the RCU
		 * read-side lock, so invalidate it now.  Materialize a
		 * reference-keycopy key first (same as the per-op uncached
		 * auto-invalidate), so the next re-descent reads the saved
		 * key rather than a stale leaf.
		 */
		if (iter->cache_mode == CDS_FT_ITER_CACHED) {
			ft_iter_materialize_key(iter);
			iter->cache_valid = false;
			iter_debug_path_clear(iter);
			iter->path_len = 0;
		}
		break;
	default:
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}
	iter->cache_mode = mode;
	return CDS_FT_STATUS_OK;
}

enum cds_ft_iter_cache_mode cds_ft_iter_get_cache_mode(
		const struct cds_ft_iter *iter)
{
	return iter->cache_mode;
}
