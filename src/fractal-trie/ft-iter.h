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
static inline_lookup
bool ft_iter_key_referenced(const struct cds_ft_iter *iter)
{
	const struct cds_ft_group *group = iter->ft->group;

	return group->ordered_list_set && iter->ft->speculative_key_offset_active &&
		group->key_map.identity && iter->cache_valid && iter->node;
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
static inline_lookup
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
 * One-shot structural materialization for a VARIABLE-length EAGER ordered-list
 * iterator (no in-leaf key, no key_len_offset): the parent up-walk derives BOTH
 * the key bytes (into iter_key) AND the length in a SINGLE walk.  Caches the
 * length in iter->key_len (clearing the LAZY sentinel) so a later read_key /
 * resolve_key_len in the same step reuses it instead of walking again.  Returns
 * the length (0 on a NIL key / overflow / missing cell).
 */
static inline_lookup
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

static inline_lookup
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
	if (group->ordered_list_set && !iter->ft->speculative_key_offset_active &&
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
 * Resolve (and cache) the iterator's current-position key length.  For a
 * deferred-length cell position it reads node->key_len from the leaf once and
 * caches it into iter->key_len (and path_len); otherwise returns iter->key_len
 * unchanged (a no-op for fixed-length, non-identity, descent and non-cell
 * positions).  EVERY reader of the current-position length (cds_ft_iter_get_key,
 * cds_ft_remove*, the skip rebuilds, bind, the max-key-len scan) must call this
 * before reading iter->key_len.  The LAZY sentinel is only ever set with
 * cache_valid && node, so the leaf read is safe.
 */
static inline_lookup
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
				!iter->ft->speculative_key_offset_active) {
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
static inline_lookup
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
			!ft->speculative_key_offset_active)) {
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
		!ft->speculative_key_offset_active;

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
	if (ft->speculative_key_offset_active) {
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
static inline_lookup
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
			&ft->ord_sentinel.node.next :
			&ft->ord_sentinel.node.prev);
	/*
	 * Sentinel topology: a link off the end resolves to the trie's own
	 * sentinel pseudo-cell, or -- transiently, while a cross-trie run move has
	 * NULL-terminated a moved run before its finalize -- to NULL.  Both are
	 * ft_ord_is_end, so terminate on that, not on "== sentinel" or "!= NULL".
	 * @loaded may thus be NULL here; the MLP guess/cmm_ptr_eq stays correct
	 * because @guess is never NULL, so cmm_ptr_eq(NULL, guess) is false and a
	 * NULL @loaded falls through to terminate the loop rather than mis-folding.
	 */
	while (n < cap && !ft_ord_is_end(ft, cur)) {
		struct ft_ord_cell *loaded, *guess;

		buf[n++] = (const struct cds_ft_cell *) cur;
		loaded = forward ?
			ft_ord_cell_resolve_ord(&cur->lnode.next) :
			ft_ord_cell_resolve_ord(&cur->lnode.prev);
		guess = (struct ft_ord_cell *) (forward ?
			(uintptr_t) cur + stride :
			(uintptr_t) cur - stride);
		cur = cmm_ptr_eq(loaded, guess) ? guess : loaded;
	}
	/* End of list (cur is the sentinel) => no resume cursor. */
	*next_cursor = ft_ord_is_end(ft, cur) ?
		NULL : (const struct cds_ft_cell *) cur;
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
	if (ft->speculative_key_offset_active) {
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

bool cds_ft_group_rank_stats(const struct cds_ft_group *group)
{
	return group->rank_stats_set;
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
