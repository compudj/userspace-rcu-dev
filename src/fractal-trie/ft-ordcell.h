// SPDX-FileCopyrightText: 2012-2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-only

/*
 * src/fractal-trie/ft-ordcell.h
 *
 * Userspace RCU library - Fractal Trie: ordinal-cell primitives: the ordered sibling-list cell, alloc/free, flag/resolve, batched cell iteration lives with the iterator.
 *
 * Implementation unit: #included once into the fractal-trie.c translation
 * unit (preserves cross-module inlining).  Not a standalone header.
 */
#ifndef FRACTAL_TRIE_IMPL
#error "ft-ordcell.h is an implementation unit; #include it from fractal-trie.c only"
#endif

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
struct ft_ord_cell *ft_ord_cell_resolve_ord(struct ft_ord_cell *const *slot)
{
	struct ft_ord_cell *p = rcu_dereference(*slot);

	if (caa_unlikely(ft_node_flip_proxy((struct cds_ft_inode_flag *) p)))
		p = (struct ft_ord_cell *) urcu_flip_proxy_get(
			ft_flip_proxy_ptr((struct cds_ft_inode_flag *) p));
	return p;
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
	cell->ord_prev = NULL;
	cell->ord_next = NULL;
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

/*
 * ft_node_holder: write-side resolution of a node's holder (the slot owner
 * "above" it), independent of the cell relocation.
 *
 *   - non-head duplicate: prev is the predecessor cds_ft_node (external).
 *   - head: prev is the flagged parent directly (non-cell build) or the
 *     cell whose ->parent holds the flagged parent (cell build).
 *   - never-inserted (prev NULL): returns NULL.
 *
 * Mutex-held callers (remove / replace / locate-chain-head) that previously
 * read node->prev as the holder route through this so the cell indirection
 * is transparent.  Identity in non-cell builds.
 */
static inline
