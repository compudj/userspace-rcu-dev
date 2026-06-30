// SPDX-FileCopyrightText: 2012-2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-only

/*
 * src/fractal-trie/ft-lookup-helpers.h
 *
 * Userspace RCU library - Fractal Trie: read-path (lookup / ordered-iteration)
 * helpers -- the ordinal-cell ACCESS primitives: the ordered sibling-list cell
 * struct, its flag / pointer / ord-resolution accessors, and alloc / free.
 * These are shared by the ordered read path (cds_ft_next, the iterator,
 * inequality lookups) and the write path, so they live early, beside the other
 * read helpers.
 *
 * NOTE: this is NOT every ord_cell function -- only the read-accessible
 * primitives.  The ordered-list MAINTENANCE operations (splice / unsplice /
 * swap / flip / run / find), which mutate the cell list and depend on the
 * relational descent and node scanners, live in ft-mutation-helpers.h on the
 * write side.  Batched cell iteration lives with the iterator (ft-iter.h).
 *
 * Implementation unit: #included once into the fractal-trie.c translation
 * unit (preserves cross-module inlining).  Not a standalone header.
 */
#ifndef FRACTAL_TRIE_IMPL
#error "ft-lookup-helpers.h is an implementation unit; #include it from fractal-trie.c only"
#endif

static inline_lookup
void *ft_ord_cell_flag(struct ft_ord_cell *cell)
{
	return (void *) ((unsigned long) cell | FT_ORD_CELL_TAG);
}

static inline_lookup
struct ft_ord_cell *ft_ord_cell_ptr(const void *prev)
{
	return (struct ft_ord_cell *)
		((unsigned long) prev & ~(unsigned long) FT_ORD_CELL_TAG);
}

/*
 * Resolve a head's flagged parent from its (already-rcu_dereference'd) prev.
 * When the group runs an ordinal-cell list, prev is a cell and the parent is
 * rcu_dereference(cell->parent); otherwise prev IS the flagged parent (no cell
 * interposed).  @ft selects the mode (the runtime cell-optional gate is added
 * later; for now the cell branch is unconditional in cell builds).  Valid only
 * for a head; a non-head dup's prev is the preceding node (callers gate on
 * ft_node_external like before).
 */
static inline_lookup
struct cds_ft_inode_flag *ft_resolve_head_prev(const struct cds_ft *ft, void *prev)
{
	if (ft->ordered_list)
		return rcu_dereference(ft_ord_cell_ptr(prev)->parent);
	return (struct cds_ft_inode_flag *) prev;
}

/*
 * Read an ordinal-cell ord_next / ord_prev slot, resolving an in-flight
 * flip-proxy.  The slots hold RAW (untagged) ft_ord_cell pointers, but a
 * point-op splice transiently installs a tagged flip-proxy (the same type-7
 * encoding as ft_resolve_flip_proxy) so the two directional edges flip
 * atomically for a bidirectional ordered reader.  Raw cells are >= 8-byte
 * aligned (bits 0-1 clear, like an external node) and proxies carry the
 * type-7 tag, so the proxy test is unambiguous.  Under writer exclusion no
 * proxy is installed at rest (a no-op on the write side).
 */
static inline_lookup
struct ft_ord_cell *ft_ord_cell_resolve_ord(struct urcu_txn_sw_list_node *const *slot)
{
	struct urcu_txn_sw_list_node *p = rcu_dereference(*slot);

	if (caa_unlikely(ft_node_flip_proxy((struct cds_ft_inode_flag *) p)))
		p = (struct urcu_txn_sw_list_node *) urcu_txn_sw_proxy_get(
			ft_flip_proxy_ptr((struct cds_ft_inode_flag *) p));
	/*
	 * Circular topology: a link "off the end" resolves to the per-trie
	 * sentinel node, NOT NULL.  The returned value is a pseudo-cell
	 * (ft_ord_cell_of(&sentinel.node)) whose ONLY valid use is the
	 * ft_ord_is_end() boundary test -- it is a bare urcu_txn_sw_list_node, not a
	 * full cell, so never deref its node/parent.  Use ft_ord_first / ft_ord_last
	 * when a NULL-if-empty endpoint is wanted.
	 */
	return ft_ord_cell_of(p);
}

/*
 * The ordinal-cell list's circular sentinel as a pseudo-cell: lnode is the
 * first field of struct ft_ord_cell (offset 0), so ft_ord_cell_of(&sentinel.node)
 * is bit-identical to &ft->ord_sentinel.node.  Valid ONLY as the ft_ord_is_end
 * comparison value and as a flip-edge old/new target denoting the list boundary.
 */
static inline_lookup
struct ft_ord_cell *ft_ord_sentinel_cell(const struct cds_ft *ft)
{
	return ft_ord_cell_of(&ft->ord_sentinel.node);
}

/*
 * True when @c is a resolved forward / backward link that points "off the end"
 * of @ft's ordinal-cell list: @ft's own circular sentinel, OR NULL.
 *
 * NULL is kept as a UNIVERSAL (trie-agnostic) end marker: a cross-trie bulk move
 * cannot point a moved run's outer link at the DESTINATION trie's sentinel
 * without that foreign sentinel becoming reachable to a SOURCE-trie reader
 * straddling the move (the source reader would not recognise another trie's
 * sentinel and would dereference it as a cell).  So a run handed to an exclusive
 * transient trie (ft_ord_cell_run_install) terminates at NULL, which every
 * trie's reader treats as the end -- exactly as the pre-sentinel NULL-terminated
 * list did.  The cells regain a real sentinel link when re-homed into a live
 * trie (run-splice / interleave overwrite their outer links), so the in-trie
 * remove folding (which needs first->prev == sentinel) is unaffected.
 */
static inline_lookup
bool ft_ord_is_end(const struct cds_ft *ft, const struct ft_ord_cell *c)
{
	return !c || &c->lnode == &ft->ord_sentinel.node;
}

/*
 * First cell of @ft's ordinal-cell list in key order, or NULL when the list is
 * empty -- the sentinel-model replacement for reading ord_cell_head.  Resolves a
 * flip proxy (a concurrent splice/run move flips the sentinel's next edge).
 */
static inline_lookup
struct ft_ord_cell *ft_ord_first(const struct cds_ft *ft)
{
	struct ft_ord_cell *c =
		ft_ord_cell_resolve_ord(&ft->ord_sentinel.node.next);

	return ft_ord_is_end(ft, c) ? NULL : c;
}

/* Last cell of @ft's ordinal-cell list, or NULL when empty (old ord_cell_tail). */
static inline_lookup
struct ft_ord_cell *ft_ord_last(const struct cds_ft *ft)
{
	struct ft_ord_cell *c =
		ft_ord_cell_resolve_ord(&ft->ord_sentinel.node.prev);

	return ft_ord_is_end(ft, c) ? NULL : c;
}

/* True when @ft's ordinal-cell list is empty (the sentinel points at itself). */
static inline_lookup
bool ft_ord_empty(const struct cds_ft *ft)
{
	return ft_ord_is_end(ft,
		ft_ord_cell_resolve_ord(&ft->ord_sentinel.node.next));
}

/*
 * Cell lifecycle (FT-allocator arena).
 *
 * The cell is an item of the group's dedicated cell arena (a uniform 32 B
 * item region, FT_ORD_CELL_ALLOC_ORDER), so cells pack contiguously for the
 * dense ord-walk and are RELOCATABLE by cds_ft_compact.  The paired metadata
 * slot is unused except its rcu_head, which cds_ft_free_item reuses to defer
 * the free past a grace period; cds_ft_metadata_to_item / _item_to_metadata
 * map between the cell and its metadata via the range header.
 */

/*
 * Allocate a head's cell and wire it to @node with parent @parent (which
 * may be NULL -- a root head -- or set later via ft_ord_cell_set_parent).
 * The ord_prev / ord_next list links start empty; the ordered-list splice
 * (runtime-gated by ordered_list_set) populates them later.  Returns the
 * cell-tagged pointer to store into node->prev, or NULL on allocation
 * failure (the caller fails the insert before mutating the trie).
 */
static
void *ft_ord_cell_alloc(struct cds_ft *ft, struct cds_ft_node *node,
		struct cds_ft_inode_flag *parent)
{
	struct cds_ft_metadata *meta = cds_ft_alloc_cell_item(ft);
	struct ft_ord_cell *cell;

	if (!meta)
		return NULL;
	cell = (struct ft_ord_cell *) cds_ft_metadata_to_item(meta);
	cell->lnode.prev = NULL;
	cell->lnode.next = NULL;
	cell->node = node;
	cell->parent = parent;
	if (ft_debug_counters())
		uatomic_inc(&ft->group->nr_cells_allocated);
	return ft_ord_cell_flag(cell);
}

/*
 * Release a head's cell after its key leaves the trie.  Routes through
 * cds_ft_free_item, which defers the free past a grace period (concurrent
 * mode) so an in-flight reader parked on a head it up-walks (head->prev ->
 * cell -> cell->parent) never dereferences freed memory, frees synchronously
 * in exclusive mode, and drains the cell range's nr_live for reclaim.
 */
static
void ft_ord_cell_free(struct cds_ft *ft, struct ft_ord_cell *cell)
{
	if (ft_debug_counters())
		uatomic_inc(&ft->group->nr_cells_freed);
	cds_ft_free_item(ft, cds_ft_item_to_metadata(cell));
}

/*
 * Immediate free for a cell that was never published (an insert that ended
 * a duplicate or failed before @node became reachable): no reader can hold a
 * reference, so the grace-period defer would only delay the free.
 */
static inline
void ft_ord_cell_free_unpublished(struct cds_ft *ft, struct ft_ord_cell *cell)
{
	if (ft_debug_counters())
		uatomic_inc(&ft->group->nr_cells_freed);
	cds_ft_free_item_unpublished(ft, cds_ft_item_to_metadata(cell));
}

/*
 * Set a head's relocated parent: in the cell-always model head->prev is the
 * cell (set once at alloc) and the flagged parent lives in cell->parent.
 * rcu_assign_pointer for the read-side up-walk (ft_resolve_head_prev does
 * rcu_dereference(cell->parent)); @head must already carry its cell.
 */
static inline
void ft_ord_cell_set_parent(struct cds_ft_node *head,
		struct cds_ft_inode_flag *parent)
{
	rcu_assign_pointer(ft_ord_cell_ptr(head->prev)->parent, parent);
}
