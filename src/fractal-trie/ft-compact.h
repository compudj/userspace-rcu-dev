// SPDX-FileCopyrightText: 2012-2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-only

/*
 * src/fractal-trie/ft-compact.h
 *
 * Userspace RCU library - Fractal Trie: cds_ft_compact: node and ordinal-cell relocation.
 *
 * Implementation unit: #included once by fractal-trie.c, in dependency
 * order, into a single translation unit (preserves cross-module inlining).
 * Not a standalone header.
 */
#ifndef FRACTAL_TRIE_IMPL
#error "ft-compact.h is an implementation unit; #include it from fractal-trie.c only"
#endif

/*
 * cds_ft_compact internals.
 *
 * Relocate every internal node of a trie into fresh, densely-packed
 * allocations in DFS descent order, so the node arenas defragment: the old
 * ranges drain as relocated nodes are freed, and the allocator reclaims the
 * ones that empty (see cds_ft_do_free_item).  The relative layout order is
 * not a measurable performance lever (DFS, BFS+DFS and density-DFS all tie);
 * the win is recovering locality lost to churn/graft, so we use plain DFS.
 *
 * Concurrency: the caller's writer exclusion keeps the trie quiescent w.r.t.
 * other writers for the whole walk; concurrent RCU readers are fine.  Each
 * node is republished and its old copy RCU-freed (ft_node_recompact +
 * cds_ft_free_item), so a reader observes the old or the new node, never a
 * freed one.  Compressed nodes are relocated too, via
 * ft_compact_relocate_compressed: a traditional compressed node through its
 * grandparent slot, a skip-compressed node recovered from its skip pointer
 * with ft_skip_to_compressed with no grandparent repoint (the slot holds a
 * skip pointer to the target, not to cn).  This covers a skip whose target is
 * an external leaf: cn is relocated, the leaf is not (leaves are
 * application-owned), and the descent ends at the loop's ft_node_external
 * check.  An internal skip target is then relocated through cn->child, where
 * ft_node_recompact's dual-pointer publish updates both cn->child and the skip
 * pointer.
 */
/* Relocate the internal node at *@holder into a fresh slot; RCU-free the old. */
static
void ft_compact_relocate_at(struct cds_ft *ft, struct cds_ft_inode_flag **holder)
{
	struct cds_ft_inode_flag *nf = *holder;
	unsigned int type_index = ft_node_type(nf);
	struct cds_ft_inode *node = ft_node_ptr(nf), *old_ret = NULL;
	struct cds_ft_metadata *meta = cds_ft_item_to_metadata(node);

	(void) ft_node_recompact(FT_RECOMPACT_RELOCATE, ft, type_index,
			&ft_types[type_index], node, meta, holder,
			0, NULL, NULL, &old_ret, holder == &ft->root, 0,
			false);
	/*
	 * The old node was just unpublished; concurrent readers may still
	 * hold it, so free it after a grace period.  Its range's nr_live
	 * decrements in the callback, so a fully-drained range self-reclaims.
	 * ALWAYS deferred, even on an exclusive trie: the step's walk keeps
	 * navigating relative to nodes it has just unpublished, and the
	 * exclusive-mode synchronous free threads the freelist link through
	 * the freed slot immediately.
	 */
	if (old_ret)
		cds_ft_free_item_deferred(ft, cds_ft_item_to_metadata(old_ret));
}

/*
 * Relocate a compressed node @cn into a fresh slot, keeping the same length.
 * Because the length is unchanged, the skip pointer's encoded length still
 * matches the relocated node (cn2->len == skip_len, whether a reader resolves
 * the old or the new node via the skip child's back-pointer), and the
 * parent-pointer flip is exactly the transient that ft_skip_reanchor already
 * tolerates; the old node stays alive until its grace period.  A compressed
 * node never carries external_nodes (that would fork a path that must remain
 * skippable), so the only reference to redirect is its child's back-pointer.
 * The grandparent slot is repointed only for a traditional (non-skip)
 * compressed node -- a skip pointer addresses the target, not @cn, so it needs
 * no change.
 *
 * @gp_slot: grandparent slot holding the cn flag (traditional), or NULL (skip).
 * Returns the new compressed node (or @cn unchanged on allocation failure).
 */
static
struct cds_ft_compressed_node *ft_compact_relocate_compressed(struct cds_ft *ft,
		struct cds_ft_compressed_node *cn,
		struct cds_ft_inode_flag **gp_slot)
{
	struct cds_ft_metadata *cn_meta =
		cds_ft_item_to_metadata((struct cds_ft_inode *) cn);
	struct cds_ft_metadata *cn2_meta;
	struct cds_ft_compressed_node *cn2;
	struct cds_ft_inode_flag *cn2_flag;
	uint8_t len = cn->len;

	cn2 = alloc_compressed_node(ft, len, &cn2_meta);
	if (!cn2)
		return cn;	/* OOM: leave in place (best-effort) */
	cn2->len = len;
	cn2->child = cn->child;
	memcpy(cn2->key_bytes, cn->key_bytes, len);
	cn2_meta->parent = cn_meta->parent;
	/*
	 * The relocated compressed node keeps the SAME slot in the SAME
	 * parent, so its parent-slot offset is identical.  Copy it on every
	 * build: the offset is no longer skip-specific (it backs the
	 * parent-pointer backtrack's O(1) slot recovery), and a position-
	 * based remove that climbs via ft_get_parent_slot would otherwise
	 * read a fresh-zeroed offset and resolve the wrong slot.
	 */
	cn2_meta->parent_slot_offset = cn_meta->parent_slot_offset;
	/* Same slot in the same parent => same incoming edge byte (up-walk source). */
	cn2_meta->incoming_byte = cn_meta->incoming_byte;
	cn2_meta->nr_child = cn_meta->nr_child;		/* == 1 for a compressed node */
	cn2_meta->external_nodes = NULL;		/* never set on a compressed node */
	ft_nr_keys_store(cn2_meta, ft_nr_keys_get(cn_meta), CMM_RELAXED);
	cn2_flag = ft_compressed_node_flag(cn2);
	/* Redirect the child's back-reference (internal: parent; external: prev). */
	ft_set_parent(ft, cn2->child, cn2_flag, &cn2->child);
	if (gp_slot)
		rcu_assign_pointer(*gp_slot, cn2_flag);
	/* Always-deferred free: see ft_compact_relocate_at. */
	FT_TP(compressed_free, (const void *) ft_compressed_node_flag(cn));
	cds_ft_free_item_deferred(ft, cn_meta);
	if (ft_debug_counters()) {
		uatomic_inc(&ft->group->nr_nodes_freed);
		uatomic_inc(&ft->group->nr_compressed_freed);
	}
	return cn2;
}

/*
 * Relocate one ordinal cell into a fresh slot from the dedicated cell arena.
 * The active recompaction context routes the allocation into a private cell
 * range, so cells relocated in key-traversal order pack densely there -- the
 * dense ord-walk stride that makes ordered iteration a sequential scan rather
 * than a random pointer chase.  Returns the new cell, or @old unchanged on
 * allocation failure (best-effort: leave it in place).
 *
 * Atomicity reuses ft_ord_cell_swap for the two ordered-list edges
 * (pred->ord_next / succ->ord_prev flip together via the flip-latch, so a
 * bidirectional ordered reader never sees a half-relocated list; ord_cell_head
 * /tail follow).  The head's UPWARD reference (head->prev) is then re-pointed
 * with a PLAIN RCU store: an up-walk reader resolves the old or the new cell,
 * both carrying an IDENTICAL parent (compaction runs under writer exclusion, so
 * @old->parent is settled), and @old stays live until its grace period -- so no
 * flip is needed, and head->prev must never hold a flip-proxy (ft_resolve_head_
 * prev does not resolve one).  @old keeps its own links for parked ordered
 * readers and is RCU-freed (its general-arena range drains for reclaim).
 */
static
struct ft_ord_cell *ft_compact_relocate_cell(struct cds_ft *ft,
		struct ft_ord_cell *old)
{
	struct cds_ft_metadata *meta = cds_ft_alloc_cell_item(ft);
	struct cds_ft_node *head = old->node;
	struct ft_ord_cell *new_cell;

	if (!meta)
		return old;		/* OOM: best-effort, leave in place */
	if (ft_debug_counters())
		uatomic_inc(&ft->group->nr_cells_allocated);
	new_cell = (struct ft_ord_cell *) cds_ft_metadata_to_item(meta);
	new_cell->node = head;
	new_cell->parent = old->parent;
	/* Carry the head's edge byte across the relocation (up-walk key source). */
	meta->incoming_byte = cds_ft_item_to_metadata(old)->incoming_byte;
	/* ord_prev / ord_next are set from @old's neighbours by the swap. */
	ft_ord_cell_swap(ft, old, new_cell);
	rcu_assign_pointer(head->prev, ft_ord_cell_flag(new_cell));
	/* Always-deferred free: see ft_compact_relocate_at. */
	if (ft_debug_counters())
		uatomic_inc(&ft->group->nr_cells_freed);
	cds_ft_free_item_deferred(ft, cds_ft_item_to_metadata(old));
	return new_cell;
}

/*
 * Forward relocate-descent: walk from the root to the leaf for @key
 * (the user key), relocating every internal node on the path that has
 * not already been relocated this pass (recompact_private).  Mirrors the
 * lookup descent's ordinal mapping and skip/compressed advancement, but
 * tracks the holder at each step (which the read descent does not) so it
 * can republish.  A skip target is relocated through the compressed
 * node's cn->child slot, so ft_node_recompact republishes both cn->child
 * and the skip pointer.  Increments *@relocated per node moved.
 */
static
void ft_compact_descend(struct cds_ft *ft, const uint8_t *key,
		size_t key_len, unsigned long *relocated)
{
	const struct cds_ft_key_map *km = &ft->group->key_map;
	struct cds_ft_inode_flag **holder = &ft->root;
	size_t depth = 0;

	for (;;) {
		struct cds_ft_inode_flag *nf = rcu_dereference(*holder);
		struct cds_ft_inode_flag **child_slot = NULL;
		struct cds_ft_inode_flag *raw;
		uint8_t ord;

		if (ft_node_external(nf))
			return;		/* reached a leaf */
		if (!cds_ft_metadata_in_recompact_private(
				cds_ft_item_to_metadata(ft_node_ptr(nf)))) {
			ft_compact_relocate_at(ft, holder);
			(*relocated)++;
			nf = rcu_dereference(*holder);	/* the relocated node */
		}
		if (depth >= key_len)
			return;		/* consumed the whole key */
		ord = key_to_ordinal(key[depth], km);
		raw = ft_node_get_nth_skip(nf, &child_slot, ord, FT_PF_NONE);
		if (!raw)
			return;		/* child absent (e.g. concurrent removal) */
		depth++;		/* child-index byte (matches iter_key = *key++) */
		if (ft_node_skip_compressed(raw)) {
			struct cds_ft_compressed_node *cn =
				ft_skip_to_compressed(ft, raw);

			depth += ft_skip_len(raw);
			/*
			 * Relocate the compressed node carrying the skip.
			 * ft_skip_to_compressed recovers it through the child's
			 * back-pointer, so a skip whose target is an external leaf is
			 * handled too (its parent is cell-indirect; compaction's
			 * writer exclusion keeps the back-pointer settled).  The
			 * grandparent slot is a skip pointer addressing the target,
			 * not cn, so it needs no repoint (NULL).  Continue at
			 * cn->child: an internal target is relocated next iteration,
			 * while an external leaf (not itself relocatable) ends the
			 * descent at the loop's ft_node_external check.
			 */
			if (!cds_ft_metadata_in_recompact_private(
					cds_ft_item_to_metadata((struct cds_ft_inode *) cn))) {
				cn = ft_compact_relocate_compressed(ft, cn, NULL);
				(*relocated)++;
			}
			holder = &cn->child;
		} else if (ft_node_compressed(raw)) {
			struct cds_ft_compressed_node *cn = ft_compressed_node_ptr(raw);

			depth += cn->len;
			/* Traditional: the grandparent slot (child_slot) holds the cn flag. */
			if (!cds_ft_metadata_in_recompact_private(
					cds_ft_item_to_metadata((struct cds_ft_inode *) cn))) {
				cn = ft_compact_relocate_compressed(ft, cn, child_slot);
				(*relocated)++;
			}
			holder = &cn->child;
		} else {
			holder = child_slot;	/* plain internal or external child */
		}
	}
}

/* Default per-step relocation budget when cds_ft_compact_step(batch == 0). */
#define FT_COMPACT_BATCH_DEFAULT	64

/*
 * Resumable compaction state.  Heap-allocated by cds_ft_compact_begin so the
 * caller treats it as opaque.  The navigation iterator is left in its default
 * CACHED mode: within a step's read-lock window it advances incrementally on
 * its cached path (the relocated-away nodes it navigates stay alive until the
 * read-unlock, and relocation preserves key order), and it retains the cursor
 * key itself.  Between steps the path is invalidated, so the iterator's key is
 * the only resume token; the private-range context persists, merged at end.
 */
struct cds_ft_compact_state {
	struct cds_ft *ft;
	struct cds_ft_iter *iter;
	struct ft_recompact_alloc_ctx ctx;
	bool started;
	bool done;
};

struct cds_ft_compact_state *cds_ft_compact_begin(struct cds_ft *ft)
{
	struct cds_ft_compact_state *st;

	if (caa_unlikely(ft->active_compact != NULL)) {
		/*
		 * A compaction is already in flight on this trie (a previous
		 * one was never ended, or two are being started).  Programmer
		 * error: assert in debug, and refuse in release rather than
		 * abandon the in-flight one.
		 */
		assert(!"cds_ft_compact_begin: a compaction is already in progress on this trie");
		return NULL;
	}
	st = calloc(1, sizeof(*st));
	if (!st)
		return NULL;
	if (cds_ft_iter_create(ft, &st->iter) != CDS_FT_STATUS_OK) {
		free(st);
		return NULL;
	}
	ft_recompact_alloc_init(&st->ctx);
	st->ft = ft;
	ft->active_compact = st;
	return st;
}

bool cds_ft_compact_step(struct cds_ft_compact_state *st, size_t batch)
{
	struct cds_ft *ft = st->ft;
	const struct rcu_flavor_struct *flavor = ft->group->flavor;
	unsigned long relocated = 0;

	if (st->done)
		return false;
	if (batch == 0)
		batch = FT_COMPACT_BATCH_DEFAULT;

	/*
	 * Route this step's relocations into private ranges, and hold the
	 * RCU read lock for the whole batch: it keeps the iterator's reads
	 * and our descents safe, and defers our own call_rcu node frees until
	 * the read-unlock between steps (where the drained ranges reclaim and
	 * concurrent mutations get their window).
	 */
	ft_recompact_alloc_set_active(&st->ctx);
	flavor->read_lock();
	while (relocated < batch) {
		uint8_t key[FT_MAX_KEY_LEN];
		size_t key_len;
		enum cds_ft_status s;

		/*
		 * Cached iter: lookup_gt advances incrementally on the cached
		 * path within this read-lock window.  The first lookup of each
		 * batch re-descends the current structure (the path was
		 * invalidated at the previous read-unlock) from the iterator's
		 * retained key.
		 */
		s = st->started ? cds_ft_lookup_gt(ft, st->iter)
				: cds_ft_lookup_first(ft, st->iter);
		st->started = true;
		if (s != CDS_FT_STATUS_OK) {	/* NOT_FOUND or error: finished */
			st->done = true;
			break;
		}
		if (cds_ft_iter_get_key(st->iter, key, sizeof(key),
				&key_len) != CDS_FT_STATUS_OK) {
			st->done = true;
			break;
		}
		ft_compact_descend(ft, key, key_len, &relocated);
		/*
		 * Relocate this key's cell into a dense private cell range, in the
		 * same key order the iterator visits -- so the ordered cell list
		 * becomes a near-sequential scan.  iter->node is the chain head;
		 * its cell is head->prev.  Skip a cell already moved this pass
		 * (its range is recompact_private), mirroring the node descent, so
		 * the pass is idempotent and re-visits do not re-allocate.
		 */
		if (ft->group->ordered_list_set && st->iter->node) {
			struct ft_ord_cell *cell = ft_ord_cell_ptr(
				rcu_dereference(st->iter->node->prev));

			if (!cds_ft_metadata_in_recompact_private(
					cds_ft_item_to_metadata(cell))) {
				ft_compact_relocate_cell(ft, cell);
				relocated++;
			}
		}
	}
	/*
	 * Drop the cached path before releasing the read lock: the nodes it
	 * references become eligible for the grace-period free once unlocked.
	 * Bind (not just invalidate) so the iterator's key is materialized into
	 * its own buffer -- on a reference-keycopy / ordinal-cell group the live
	 * key is a leaf reference that does NOT survive the unlock, and the next
	 * step re-descends from that key.
	 */
	cds_ft_iter_bind_key(st->iter);
	flavor->read_unlock();
	ft_recompact_alloc_set_active(NULL);
	return !st->done;
}

void cds_ft_compact_end(struct cds_ft_compact_state *st)
{
	st->ft->active_compact = NULL;
	ft_recompact_alloc_merge(&st->ctx);
	cds_ft_iter_destroy(st->iter);
	free(st);
}

/*
 * Close the shared-scanner redirect (opened in fractal-trie.c before the
 * mutation modules).  The incremental compaction above is a cold writer path
 * and shares the out-of-line scanners; cds_ft_compact, the debug/stats
 * helpers, and the ft-iter.h pulled in at the end of this unit want the
 * inlined scanner originals.
 */
#undef ft_node_get_nth
#undef ft_node_get_nth_skip
#undef ft_node_get_nth_reanchor
#undef ft_node_get_direction
#undef ft_node_get_minmax
#undef cds_ft_lookup_inequality_impl

void cds_ft_compact(struct cds_ft *ft)
{
	CDS_FT_SCOPED_WRITER(ft);
	struct cds_ft_compact_state *st = cds_ft_compact_begin(ft);

	if (!st)
		return;		/* OOM: best-effort, leave the trie as-is */
	while (cds_ft_compact_step(st, 0))
		;
	cds_ft_compact_end(st);
}

static
void print_indent(FILE *out, int level)
{
	int i;

	for (i = 0; i < level; i++)
		fprintf(out, "	");
}


static
void show_node_recursive(const struct cds_ft *ft, FILE *out, struct cds_ft_inode_flag *node_flag, int level)
{
	unsigned int key;

	print_indent(out, level);
	fprintf(out, "Level %d within node %p\n", level, node_flag);
	for (key = 0; key < 256; key++) {
		struct cds_ft_inode_flag *child_node_flag;

		child_node_flag = ft_node_get_nth(ft, node_flag, NULL, (uint8_t) key, FT_PF_NONE);
		if (!child_node_flag)
			continue;
		if (ft_node_internal(child_node_flag)) {
			struct cds_ft_metadata *metadata = cds_ft_item_to_metadata(ft_node_ptr(child_node_flag));
			struct cds_ft_node *external_nodes = rcu_dereference(metadata->external_nodes);

			print_indent(out, level);
			fprintf(out, "Level %d, key value: %u, internal node: %p, nr_children: %u\n",
				level, key, child_node_flag, metadata->nr_child);
			if (external_nodes) {
				print_indent(out, level);
				fprintf(out, "Level %d, key value: %u, (meta)external node list ptr: %p\n",
					level, key, external_nodes);
			}
			show_node_recursive(ft, out, child_node_flag, level + 1);
		} else if (ft_node_compressed(child_node_flag)) {
			struct cds_ft_compressed_node *cn =
				ft_compressed_node_ptr(child_node_flag);
			struct cds_ft_metadata *metadata =
				cds_ft_item_to_metadata((struct cds_ft_inode *) cn);
			struct cds_ft_node *external_nodes = rcu_dereference(metadata->external_nodes);

			print_indent(out, level);
			fprintf(out, "Level %d, key value: %u, compressed node: %p, path_len: %u, nr_keys: %lu\n",
				level, key, child_node_flag, (unsigned int) cn->len,
				ft_nr_keys_get(metadata));
			if (external_nodes) {
				print_indent(out, level);
				fprintf(out, "Level %d, key value: %u, (meta)external node list ptr: %p\n",
					level, key, external_nodes);
			}
			if (cn->child &&
			    !ft_node_external(cn->child))
				show_node_recursive(ft, out, cn->child, level + cn->len);
			else if (cn->child) {
				print_indent(out, level + cn->len);
				fprintf(out, "Level %d, compressed child: external node list ptr: %p\n",
					level + (int) cn->len, ft_node_ptr(cn->child));
			}
		} else {
			print_indent(out, level);
			fprintf(out, "Level %d, key value: %u, external node list ptr: %p\n",
				level, key, ft_node_ptr(child_node_flag));
		}
	}
}

static
void show_pretty(const struct cds_ft *ft, FILE *out)
{
	struct cds_ft_inode_flag *node_flag;
	int level = 0;

	fprintf(out, "Show Fractal Trie %p\n", ft);
	fprintf(out, "---------------------------------------------------\n");

	node_flag = rcu_dereference(ft->root);

	/* Root is always present and always internal. */
	{
		struct cds_ft_metadata *rm = cds_ft_item_to_metadata(ft_node_ptr(node_flag));

		print_indent(out, level);
		fprintf(out, "Level 0: root node %p\n", node_flag);
		(void) rm;
	}
	show_node_recursive(ft, out, node_flag, level + 1);
	fprintf(out, "---------------------------------------------------\n");
}

/*
 * JSON emitter: walks the same trie structure as show_pretty() and
 * produces a JSON document describing it.  The output has no trailing
 * newline, so it can be embedded into other JSON contexts if desired.
 *
 * Schema summary:
 *   Root:     { "ft": "0xPTR", "root": <node> }
 *   Internal: { "ptr", "kind", "level", "nr_child", "density",
 *               "external_nodes"?, "children": [ {"key_byte", "child"} ] }
 *   Compressed: { "ptr", "kind": "COMPRESSED", "level", "path_len",
 *                 "key_bytes", "external_nodes"?, "child" }
 *   External:   { "ptr", "kind": "EXTERNAL", "level" }
 */

static void json_emit_node(const struct cds_ft *ft, FILE *out,
		struct cds_ft_inode_flag *node_flag, int level);

/*
 * Return a symbolic name for an internal-node type index.  Mirrors
 * the ft_tp_node_kind enum labels but is always compiled in (not
 * gated on FT_ENABLE_TRACING) since the JSON output is a supported
 * interface independent of tracing.
 */
static
const char *internal_type_name(unsigned int type_index)
{
	if (type_index >= sizeof(ft_types) / sizeof(ft_types[0]))
		return "UNKNOWN";
	{
		unsigned int cls = ft_types[type_index].type_class;
		unsigned int order = ft_types[type_index].order;

		switch (cls) {
		case FT_POPCOUNT:
			/*
			 * Orders 5/6 are 2-level popcount_2l layouts (P2L);
			 * orders 7/8/9/10 are 1-level popcount_1l layouts
			 * (P1L: a single 256-bit bitmap + ptr table).
			 */
			switch (order) {
			case 5:  return "P2L_32";
			case 6:  return "P2L_64";
			case 7:  return "P1L_128";
			case 8:  return "P1L_256";
			case 9:  return "P1L_512";
			case 10: return "P1L_1024";
			}
			break;
		case FT_PIGEON:
			switch (order) {
			case 10: return "PIGEON_1024";
			case 11: return "PIGEON_2048";
			}
			break;
		}
	}
	return "UNKNOWN";
}

static
void json_emit_density(FILE *out, const struct cds_ft_metadata *m __attribute__((unused)))
{
	fprintf(out, "[]");
}

static
void json_emit_node(const struct cds_ft *ft, FILE *out,
		struct cds_ft_inode_flag *node_flag, int level)
{
	if (!node_flag) {
		fprintf(out, "null");
		return;
	}
	if (ft_node_external(node_flag)) {
		fprintf(out, "{\"ptr\":\"%p\",\"kind\":\"EXTERNAL\","
			"\"level\":%d}", node_flag, level);
		return;
	}
	if (ft_node_compressed(node_flag)) {
		struct cds_ft_compressed_node *cn =
			ft_compressed_node_ptr(node_flag);
		struct cds_ft_metadata *metadata =
			cds_ft_item_to_metadata((struct cds_ft_inode *) cn);
		struct cds_ft_node *external_nodes =
			rcu_dereference(metadata->external_nodes);
		unsigned int j;

		fprintf(out, "{\"ptr\":\"%p\",\"kind\":\"COMPRESSED\","
			"\"level\":%d,\"path_len\":%u,\"nr_keys\":%lu,"
			"\"density\":",
			node_flag, level, (unsigned int) cn->len,
			ft_nr_keys_get(metadata));
		json_emit_density(out, metadata);
		fprintf(out, ",\"key_bytes\":[");
		for (j = 0; j < cn->len; j++) {
			if (j) fprintf(out, ",");
			fprintf(out, "%u", cn->key_bytes[j]);
		}
		fprintf(out, "]");
		if (external_nodes)
			fprintf(out, ",\"external_nodes\":\"%p\"",
				(void *) external_nodes);
		fprintf(out, ",\"child\":");
		if (cn->child)
			json_emit_node(ft, out, cn->child, level + cn->len);
		else
			fprintf(out, "null");
		fprintf(out, "}");
		return;
	}
	/* Internal. */
	{
		struct cds_ft_metadata *metadata =
			cds_ft_item_to_metadata(ft_node_ptr(node_flag));
		struct cds_ft_node *external_nodes =
			rcu_dereference(metadata->external_nodes);
		unsigned int type_index = ft_node_type(node_flag);
		unsigned int key, printed = 0;

		fprintf(out, "{\"ptr\":\"%p\",\"kind\":\"%s\",\"level\":%d,"
			"\"nr_child\":%u,\"density\":",
			node_flag, internal_type_name(type_index), level,
			metadata->nr_child);
		json_emit_density(out, metadata);
		if (external_nodes)
			fprintf(out, ",\"external_nodes\":\"%p\"",
				(void *) external_nodes);
		fprintf(out, ",\"children\":[");
		for (key = 0; key < 256; key++) {
			struct cds_ft_inode_flag *child;

			child = ft_node_get_nth(ft, node_flag, NULL, (uint8_t) key, FT_PF_NONE);
			if (!child)
				continue;
			if (printed++) fprintf(out, ",");
			fprintf(out, "{\"key_byte\":%u,\"child\":", key);
			json_emit_node(ft, out, child, level + 1);
			fprintf(out, "}");
		}
		fprintf(out, "]}");
	}
}

static
void show_json(const struct cds_ft *ft, FILE *out)
{
	struct cds_ft_inode_flag *node_flag;

	node_flag = rcu_dereference(ft->root);
	fprintf(out, "{\"ft\":\"%p\",\"root\":", ft);
	json_emit_node(ft, out, node_flag, 0);
	fprintf(out, "}\n");
}

void cds_ft_show(const struct cds_ft *ft, FILE *out,
		enum cds_ft_show_format fmt)
{
	switch (fmt) {
	case CDS_FT_SHOW_JSON:
		show_json(ft, out);
		break;
	case CDS_FT_SHOW_PRETTY:
	default:
		show_pretty(ft, out);
		break;
	}
}

struct cds_ft_node_stats {
	uint64_t count;
	uint64_t distribution[257];
};

struct cds_ft_stats_level {
	uint64_t nr_external_nodes;
	uint64_t nr_metadata_external_nodes;
	uint64_t nr_duplicate_external_nodes;
	uint64_t nr_internal_nodes;
	uint64_t nr_compressed_nodes;
	struct cds_ft_node_stats node_stats[FT_TYPE_MAX_NR];
	bool has_nodes;
};

struct cds_ft_stats {
	struct cds_ft_stats_level level[FT_MAX_DEPTH];
};

enum cds_ft_status cds_ft_recompute_stats(struct cds_ft *ft)
{
	struct cds_ft_iter *iter;
	enum cds_ft_status status;
	size_t max_len = 0;

	CDS_FT_SCOPED_WRITER(ft);
	status = cds_ft_iter_create(ft, &iter);
	if (status != CDS_FT_STATUS_OK)
		return status;
	cds_ft_for_each_rcu(ft, iter) {
		size_t klen = ft_iter_resolve_key_len(iter);

		if (klen > max_len)
			max_len = klen;
	}
	status = cds_ft_iter_status(iter);
	cds_ft_iter_destroy(iter);
	if (status < 0)
		return status;
	uatomic_store(&ft->max_used_key_len, max_len, CMM_RELAXED);
	return CDS_FT_STATUS_OK;
}

static
void calc_stats_node(const struct cds_ft *ft __attribute__((unused)),
		struct cds_ft_inode_flag *node_flag, struct cds_ft_stats *stats, int level)
{
	unsigned long node_type = ft_node_type(node_flag);
	struct cds_ft_node_stats *node_stats = &stats->level[level].node_stats[node_type];
	const struct cds_ft_metadata *metadata;

	metadata = cds_ft_item_to_metadata(ft_node_ptr(node_flag));
	node_stats->count++;
	node_stats->distribution[metadata->nr_child]++;
	stats->level[level].nr_internal_nodes++;
	stats->level[level].has_nodes = true;
}

static
void calc_stats_node_recursive(const struct cds_ft *ft, struct cds_ft_inode_flag *node_flag,
		struct cds_ft_stats *stats, int level);

static
void calc_stats_node_recursive(const struct cds_ft *ft, struct cds_ft_inode_flag *node_flag,
		struct cds_ft_stats *stats, int level)
{
	unsigned int key;

	for (key = 0; key < 256; key++) {
		struct cds_ft_inode_flag *child_node_flag;

		child_node_flag = ft_node_get_nth(ft, node_flag, NULL, (uint8_t) key, FT_PF_NONE);
		if (!child_node_flag)
			continue;
		if (ft_node_internal(child_node_flag)) {
			struct cds_ft_metadata *metadata = cds_ft_item_to_metadata(ft_node_ptr(child_node_flag));
			struct cds_ft_node *external_nodes = rcu_dereference(metadata->external_nodes);

			calc_stats_node(ft, child_node_flag, stats, level);
			if (external_nodes) {
				struct cds_ft_node *iter_node;
				unsigned int count = 0;

				iter_node = external_nodes;
				cds_ft_for_each_duplicate(iter_node) {
					if (count++ == 0)
						stats->level[level].nr_metadata_external_nodes++;
					else
						stats->level[level].nr_duplicate_external_nodes++;
					stats->level[level].has_nodes = true;
				}
			}
			calc_stats_node_recursive(ft, child_node_flag, stats, level + 1);
		} else if (ft_node_compressed(child_node_flag)) {
			struct cds_ft_compressed_node *cn =
				ft_compressed_node_ptr(child_node_flag);
			struct cds_ft_metadata *metadata =
				cds_ft_item_to_metadata((struct cds_ft_inode *) cn);
			struct cds_ft_node *external_nodes = rcu_dereference(metadata->external_nodes);
			int j;

			stats->level[level].nr_internal_nodes++;
			stats->level[level].nr_compressed_nodes++;
			stats->level[level].has_nodes = true;
			if (external_nodes) {
				struct cds_ft_node *iter_node;
				unsigned int count = 0;

				iter_node = external_nodes;
				cds_ft_for_each_duplicate(iter_node) {
					if (count++ == 0)
						stats->level[level].nr_metadata_external_nodes++;
					else
						stats->level[level].nr_duplicate_external_nodes++;
					stats->level[level].has_nodes = true;
				}
			}
			for (j = 1; j < cn->len; j++) {
				stats->level[level + j].nr_internal_nodes++;
				stats->level[level + j].nr_compressed_nodes++;
				stats->level[level + j].has_nodes = true;
			}
			if (cn->child &&
			    !ft_node_external(cn->child))
				calc_stats_node_recursive(ft, cn->child, stats, level + cn->len);
			else if (cn->child) {
				struct cds_ft_node *iter_node;
				unsigned int count = 0;

				iter_node = (struct cds_ft_node *) ft_node_ptr(cn->child);
				cds_ft_for_each_duplicate(iter_node) {
					if (count++ == 0)
						stats->level[level + cn->len].nr_external_nodes++;
					else
						stats->level[level + cn->len].nr_duplicate_external_nodes++;
					stats->level[level + cn->len].has_nodes = true;
				}
			}
		} else {
			struct cds_ft_node *iter_node;
			unsigned int count = 0;

			iter_node = (struct cds_ft_node *) ft_node_ptr(child_node_flag);
			cds_ft_for_each_duplicate(iter_node) {
				if (count++ == 0)
					stats->level[level].nr_external_nodes++;
				else
					stats->level[level].nr_duplicate_external_nodes++;
				stats->level[level].has_nodes = true;
			}
		}
	}
}

static
void do_show_stats(const struct cds_ft *ft, FILE *out, const struct cds_ft_stats *stats)
{
	int level;

	fprintf(out, "Fractal Trie (%p) Statistics\n", ft);
	fprintf(out, "---------------------------------------------------\n");
	for (level = 0; level < FT_MAX_DEPTH; level++) {
		const struct cds_ft_stats_level *stats_level = &stats->level[level];
		unsigned long type;

		if (!stats_level->has_nodes)
			break;
		fprintf(out, "Level: %d\n", level);
		if (stats_level->nr_external_nodes) {
			print_indent(out, 1);
			fprintf(out, "External nodes: %" PRIu64 "\n", stats_level->nr_external_nodes);
		}
		if (stats_level->nr_metadata_external_nodes) {
			print_indent(out, 1);
			fprintf(out, "Metadata external nodes: %" PRIu64 "\n", stats_level->nr_metadata_external_nodes);
		}
		if (stats_level->nr_duplicate_external_nodes) {
			print_indent(out, 1);
			fprintf(out, "Duplicate external nodes: %" PRIu64 "\n", stats_level->nr_duplicate_external_nodes);
		}
		if (stats_level->nr_internal_nodes) {
			print_indent(out, 1);
			fprintf(out, "Internal nodes: %" PRIu64 "\n", stats_level->nr_internal_nodes);
		}
		if (stats_level->nr_compressed_nodes) {
			print_indent(out, 1);
			fprintf(out, "Compressed nodes: %" PRIu64 "\n", stats_level->nr_compressed_nodes);
		}
		for (type = 0; type < FT_TYPE_MAX_NR; type++) {
			const struct cds_ft_node_stats *node_stats = &stats->level[level].node_stats[type];
			uint64_t nr_nodes = node_stats->count;

			if (nr_nodes) {
				unsigned int i;
				bool first = true;

				print_indent(out, 2);
				fprintf(out, "Internal node type %lu: %" PRIu64 " (", type, nr_nodes);
				for (i = 0; i <= 256; i++) {
					if (node_stats->distribution[i]) {
						fprintf(out, "%s%u: %" PRIu64,
							(!first ? ", " : ""), i, node_stats->distribution[i]);
						first = false;
					}
				}
				fprintf(out, ")\n");
			}
		}
	}
	fprintf(out, "---------------------------------------------------\n");
}

void cds_ft_show_stats(const struct cds_ft *ft, FILE *out)
{
	struct cds_ft_inode_flag *node_flag;
	struct cds_ft_stats *stats;
	int level = 0;

	/*
	 * struct cds_ft_stats is several MiB: a per-level node-type
	 * distribution histogram (257 buckets) for each of FT_MAX_DEPTH
	 * levels.  That is far too large for the stack on threads with a
	 * small stack (e.g. musl's 128 KiB default), so heap-allocate it.
	 */
	stats = calloc(1, sizeof(*stats));
	if (!stats) {
		fprintf(out, "Fractal Trie (%p) Statistics: out of memory\n", ft);
		return;
	}

	node_flag = rcu_dereference(ft->root);

	/* Root is always present and always internal. */
	calc_stats_node(ft, node_flag, stats, level);
	calc_stats_node_recursive(ft, node_flag, stats, level + 1);
	do_show_stats(ft, out, stats);
	free(stats);
}

const char *cds_ft_status_to_string(enum cds_ft_status status)
{
	switch (status) {
	/* Success return codes (>= 0). */
	case CDS_FT_STATUS_OK:
		return "Operation completed successfully";
	case CDS_FT_STATUS_NOT_FOUND:
		return "No node found";
	case CDS_FT_STATUS_DUPLICATE_FOUND:
		return "Duplicate node exists";
	case CDS_FT_STATUS_INTERNAL_MATCH:
		return "Match ends at an internal node";

	/* Error return codes (< 0). */
	case CDS_FT_STATUS_INVALID_ARGUMENT_ERROR:
		return "Invalid argument";
	case CDS_FT_STATUS_MEMORY_ERROR:
		return "Memory allocation failure";
	case CDS_FT_STATUS_OVERFLOW_ERROR:
		return "Buffer too small for key length";
	case CDS_FT_STATUS_BUSY_ERROR:
		return "Resource busy";
	case CDS_FT_STATUS_POPULATED_ERROR:
		return "Destination already populated";
	case CDS_FT_STATUS_INTEGRITY_ERROR:
		return "Integrity verification failure";
	case CDS_FT_STATUS_NOT_SUPPORTED:
		return "Feature not compiled in";

	default:
		return "Unknown status value";
	}
}
