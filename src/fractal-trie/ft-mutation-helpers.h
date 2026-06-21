// SPDX-FileCopyrightText: 2012-2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-only

/*
 * src/fractal-trie/ft-mutation-helpers.h
 *
 * Userspace RCU library - Fractal Trie: shared write-path (mutation) helpers.
 *
 * The ordered-list maintenance subsystem: the flip-latch batch and the
 * ordinal-cell flip / splice / swap / run / find operations that keep the
 * key-ordered cell list consistent under the writer lock.  Used by insert,
 * remove, graft, detach and merge alike, so it lives in one module ahead of
 * them rather than parked in any single bulk-op file.
 *
 * Implementation unit: #included once by fractal-trie.c, in dependency order
 * into a single translation unit (preserves cross-module inlining).  It sits
 * after ft-inequality.h and the shared-scanner redirect, before ft-insert.h,
 * so its scanner / inequality calls route through the shared copies the rest
 * of the write path uses.  Not a standalone header.
 */
#ifndef FRACTAL_TRIE_IMPL
#error "ft-mutation-helpers.h is an implementation unit; #include it from fractal-trie.c only"
#endif

/*
 * Descent cursor -- tracks current, parent, and grandparent positions
 * during a key-guided traversal of the trie.
 *
 * Each level stores both the flagged-pointer value (nf / pnf / ppnf)
 * and the address of the slot that holds it (nfp / pnfp / ppnfp).
 * Callers that do not need every field may leave the unused ones
 * NULL; the struct carries the superset so that a single descent
 * helper can serve graft, insert, remove, and detach paths.
 */
struct ft_descent {
	unsigned int depth;			/* Levels traversed (0 .. key_len). */
	struct cds_ft_inode_flag *nf;		/* Current node-flag value. */
	struct cds_ft_inode_flag **nfp;		/* Slot that holds @nf. */
	struct cds_ft_inode_flag *pnf;		/* Parent node-flag value. */
	struct cds_ft_inode_flag **pnfp;	/* Slot that holds @pnf. */
	struct cds_ft_inode_flag *ppnf;		/* Grandparent node-flag value. */
	struct cds_ft_inode_flag **ppnfp;	/* Slot that holds @ppnf. */
};

static
void ft_descent_init(struct ft_descent *d, struct cds_ft *ft)
{
	d->depth = 0;
	d->nf = ft->root;
	d->nfp = &ft->root;
	d->pnf = NULL;
	d->pnfp = NULL;
	d->ppnf = NULL;
	d->ppnfp = NULL;
}

/*
 * Advance descent state through a compressed node on full key match.
 * Updates parent chain, current pointer, and depth.  The caller is
 * responsible for snapshot, snapshot_n, and detach tracking before
 * calling this helper.
 */
static inline_lookup
void ft_descent_traverse_compressed(struct ft_descent *d,
		struct cds_ft_compressed_node *cn,
		const uint8_t **iter_key)
{
	d->ppnf  = d->pnf;
	d->ppnfp = d->pnfp;
	d->pnf   = d->nf;
	d->pnfp  = d->nfp;
	d->nf    = cn->child;
	d->nfp   = &cn->child;
	d->depth += cn->len;
	*iter_key += cn->len;
}

/*
 * Advance the descent cursor one level down: rotate current -> parent ->
 * grandparent, then descend into child @key_value.
 *
 * Returns the new d->nf (the child's flagged pointer, possibly NULL).
 */
static inline
struct cds_ft_inode_flag *ft_descent_step(struct cds_ft *ft, struct ft_descent *d,
		uint8_t key_value)
{
	d->ppnf  = d->pnf;
	d->ppnfp = d->pnfp;
	d->pnf   = d->nf;
	d->pnfp  = d->nfp;
	d->nf    = ft_node_get_nth(ft, d->pnf, &d->nfp, key_value, FT_PF_NONE);
	d->depth++;
	return d->nf;
}

/*
 * Flip-latch batch for cds_ft_merge_at: a urcu_flip_group plus its
 * proxies, allocated as one block and reclaimed together via call_rcu
 * once the proxied slots have been settled to their direct new targets.
 * ft_flip_proxy is 16-byte aligned so each proxy's address carries the
 * type-7 proxy tag (ft_flip_proxy_flag); malloc returns 16-byte-aligned
 * blocks at userspace addresses with the skip-len high bits clear.
 */
struct ft_flip_proxy {
	struct urcu_flip_proxy proxy;
} __attribute__((aligned(16)));

struct ft_flip_batch {
	struct rcu_head rcu_head;
	struct cds_ft *ft;
	struct urcu_flip_group group;
	unsigned int nr;
	unsigned int cap;
	struct ft_flip_proxy proxies[];
};

static
void ft_flip_batch_free_rcu(struct rcu_head *head)
{
	free(caa_container_of(head, struct ft_flip_batch, rcu_head));
}

static
struct ft_flip_batch *ft_flip_batch_alloc(struct cds_ft *ft, unsigned int cap)
{
	struct ft_flip_batch *b;

	b = malloc(sizeof(*b) + (size_t) cap * sizeof(struct ft_flip_proxy));
	if (!b)
		return NULL;
	b->ft = ft;
	urcu_flip_group_init(&b->group);
	b->nr = 0;
	b->cap = cap;
	return b;
}

/*
 * Take a caller-reserved flip batch when @pre supplies one, NULLing the
 * caller's slot to transfer ownership: from here on the consuming op frees the
 * batch (reclaim on commit, free_unpublished on its abort), and the caller
 * frees only the slots it still holds.  Falls back to a fresh allocation when
 * no batch was reserved (the standalone-op path) -- so a reserved op draws a
 * pre-allocated, unfailable batch while a normal op allocates as before.  @cap
 * is the fallback capacity; a reserved batch is already sized >= @cap by the
 * caller's read-only count pass.
 */
static
struct ft_flip_batch *ft_flip_batch_take(struct cds_ft *ft, unsigned int cap,
		struct ft_flip_batch **pre)
{
	if (pre && *pre) {
		struct ft_flip_batch *b = *pre;

		*pre = NULL;
		return b;
	}
	return ft_flip_batch_alloc(ft, cap);
}

/*
 * Free a flip batch that was allocated but never installed (no proxy stored
 * in any live slot, group never committed): a plain free, no grace period,
 * since no reader can reference it.  Used by the merge's last-fallible src
 * unlink abort path.
 */
static
void ft_flip_batch_free_unpublished(struct ft_flip_batch *b)
{
	free(b);
}

/* One release store: every proxy in the batch flips old -> new atomically. */
static
void ft_flip_batch_commit(struct ft_flip_batch *b)
{
	urcu_flip_commit(&b->group);
}

/*
 * Record a proxy {old_nf, new_nf} and return its tagged flag, to be stored
 * (sel == 0 -> old_nf, transparent) into the slot being flipped.
 */
static
struct cds_ft_inode_flag *ft_flip_batch_add(struct ft_flip_batch *b,
		struct cds_ft_inode_flag *old_nf,
		struct cds_ft_inode_flag *new_nf)
{
	struct urcu_flip_proxy *p;

	assert(b->nr < b->cap);
	p = &b->proxies[b->nr++].proxy;
	urcu_flip_proxy_init(p, &b->group, old_nf, new_nf);
	return ft_flip_proxy_flag(p);
}

/* Reclaim the batch after settle (deferred for in-flight readers). */
static
void ft_flip_batch_reclaim(struct ft_flip_batch *b)
{
	if (b->ft->exclusive)
		free(b);
	else
		b->ft->group->flavor->update_call_rcu(&b->rcu_head,
			ft_flip_batch_free_rcu);
}


/*
 * Ordinal-cell list maintenance.
 *
 * Mirrors the ORD_CHAIN chain maintenance, but the key-ordered doubly-linked
 * list threads the library-owned cells (one per distinct-key head) via
 * ft_ord_cell.ord_next / ord_prev instead of in-leaf fields.  Each point op
 * flips the (<=2) live neighbour edges through one flip-batch so a
 * bidirectional ordered reader sees the splice atomically; the spliced-in /
 * replacement cell pre-sets its own links with plain stores (not yet ord-
 * reachable), while an unspliced cell keeps its links for parked readers
 * until its deferred free.  Runtime-gated by group->ordered_list_set: a
 * point op consults these only when the list is enabled.
 *
 * RUNS UNDER WRITER EXCLUSION; no concurrent writer races, no proxy at rest.
 * Promotion (ft_unchain_node) and replace (cds_ft_replace) need NO list op:
 * the cell stays put and only cell->node is retargeted.  Bulk ops (merge /
 * graft / graft_swap / detach) maintain the list through the run helpers
 * below (run_detach / run_splice / run_replace / run_unlink) and the merge
 * interleave.
 */

struct ft_ord_cell_edge {
	struct ft_ord_cell **slot;	/* a neighbour's ord_next / ord_prev slot */
	struct ft_ord_cell *old_target;
	struct ft_ord_cell *new_target;
};

/*
 * Deferred in-place leaf-delete publish (the remove dual of
 * ft_insert_commit).  When a leaf delete keeps the holder above min_child --
 * the common case, no recompaction -- its single reader-visible forward store
 * is the moment the key leaves the structural index.  The popcount/pigeon
 * replace_ptr RECORDS that store here (@slot transitions @old_val -> @new_val)
 * instead of doing it, so ft_detach_node can commit it in ONE flip together
 * with the dead head cell's ordered-list unsplice (ft_remove_one_commit): a
 * reader then never observes the key gone from one index but present in the
 * other.  Two store shapes share this:
 *   - leaf delete: @new_val == NULL (the child slot clears to empty).
 *   - external promote: a childless holder's external chain is promoted into
 *     the parent slot, so @new_val == the external chain head (the same value
 *     the immediate store would publish).  The promoted external's back-pointer
 *     is wired (ft_set_parent) BEFORE the deferral, parent-first.
 *
 * @armed is set only on the in-place path; the recompaction (-EFBIG) path
 * publishes its rebuilt node itself and leaves this untouched.  nr_child-- is
 * applied IN PLACE by the primitive for a delete (writers and canonicalize read
 * it; readers do not navigate by it); a promote replaces a child, so nr_child
 * is unchanged.  A pigeon DELETE also clears its child bitmap bit, a reader
 * channel that must settle AFTER the flip: the primitive records it in
 * @pigeon_bitmap / @pigeon_bit and ft_detach_node clears it post-commit (a
 * promote leaves the slot occupied, so no bitmap change).
 */
struct ft_remove_pub {
	struct cds_ft_inode_flag **slot;
	struct cds_ft_inode_flag *old_val;
	struct cds_ft_inode_flag *new_val;	/* NULL for delete; chain head for promote */
	struct cds_ft_bitmap *pigeon_bitmap;	/* non-NULL => clear bit post-flip */
	uint8_t pigeon_bit;
	bool armed;
};

static void ft_ord_cell_flip(struct cds_ft *ft, struct ft_ord_cell_edge *edges,
		unsigned int n);

/*
 * Append an endpoint (ord_cell_head / ord_cell_tail) update to a flip-edge batch
 * when @slot currently holds @match, so the endpoint transitions ATOMICALLY with
 * the neighbour edges in the same flip: a reader resolving ord_cell_head/tail via
 * ft_ord_cell_resolve_ord then sees a consistent old-XOR-new view (it never
 * observes the old head pointer together with an already-flipped back-edge).
 * @match/@newval may be NULL (empty-list transitions); the proxy mechanism and
 * the *slot == @match guard both handle NULL.  Returns the new edge count.
 */
static
unsigned int ft_ord_cell_endpoint_edge(struct ft_ord_cell **slot,
		struct ft_ord_cell *match, struct ft_ord_cell *newval,
		struct ft_ord_cell_edge *edges, unsigned int n)
{
	if (*slot == match) {
		edges[n].slot = slot;
		edges[n].old_target = match;
		edges[n].new_target = newval;
		n++;
	}
	return n;
}

/*
 * Structural min/max dup-chain HEAD of the subtree rooted at @nf, under WRITER
 * EXCLUSION (no concurrent mutation -> no skip re-anchor / flip-proxy / transient
 * empty states to handle, unlike the reader-side minmax descent).  Mirrors the
 * key ordering the ordered cell list uses: a key that ends at an internal node
 * (metadata->external_nodes, a prefix key) sorts BEFORE every longer key under
 * it, so it is the subtree minimum.  Used to locate the endpoints of the
 * contiguous ordered-list run a bulk op relocates.
 */
static
struct cds_ft_node *ft_subtree_minmax_head(struct cds_ft *ft, struct cds_ft_inode_flag *nf,
		bool want_max)
{
	enum ft_direction dir = want_max ? FT_RIGHTMOST : FT_LEFTMOST;
	uint8_t scratch;

	for (;;) {
		nf = ft_resolve_skip_compressed(ft, nf);
		if (ft_node_external(nf))
			return (struct cds_ft_node *) ft_node_ptr(nf);
		if (ft_node_compressed(nf)) {
			struct cds_ft_compressed_node *cn =
				ft_compressed_node_ptr(nf);

			nf = rcu_dereference(cn->child);
			continue;
		}
		/* Internal node. */
		{
			struct cds_ft_metadata *m =
				cds_ft_item_to_metadata(ft_node_ptr(nf));
			struct cds_ft_node *ext =
				ft_dereference_external(m->external_nodes);
			struct cds_ft_inode_flag *child;

			if (!want_max && ext)
				return ext;	/* prefix key: subtree minimum */
			child = ft_node_get_minmax(ft, nf, &scratch, dir, false);
			if (!child) {
				/*
				 * No children: a NIL-key-only internal whose
				 * external_nodes is the sole key, so it is also the
				 * subtree MAXIMUM (a single-prefix-key trie root, e.g.
				 * a detached external).  want_min returned it above.
				 */
				assert(ext != NULL);
				return ext;
			}
			nf = child;
		}
	}
}

/*
 * Move the contiguous ordered-list run whose endpoints are the cells of
 * @first_head .. @last_head (heads, in key order) OUT of @ft's ordered cell
 * list and install it as the ENTIRE ordered list of @into -- the cds_ft_detach
 * shape, where @into is a fresh EXCLUSIVE trie receiving exactly that subtree.
 * The run's internal ord links are preserved; only its two boundary edges in
 * @ft are flipped (atomic for a concurrent ordered reader, per the flip-latch),
 * @ft's head/tail are repaired, and @into's head/tail are set.  @into being
 * exclusive, clearing the run's new boundary links is a plain store.  Caller
 * gates on ordered_list_set.
 */
/*
 * Append the <=4 boundary edges that excise the contiguous run @first_head ..
 * @last_head from @ft's ordered cell list (the two neighbour back-edges plus any
 * head/tail repair); the run's internal links are preserved for re-homing.
 * Stashes the run-endpoint cells in *@first_out / *@last_out for the caller's
 * post-flip @into install.  Split out so a bulk detach can fuse these edges with
 * its structural subtree unlink in ONE flip (ft_remove_one_commit / _rec with a
 * struct ft_detach_run), exactly as ft_ord_cell_unsplice_edges does for a point
 * remove; ft_ord_cell_run_detach is the standalone (two-commit) wrapper.
 */
static
unsigned int ft_ord_cell_run_detach_edges(struct cds_ft *ft,
		struct cds_ft_node *first_head, struct cds_ft_node *last_head,
		struct ft_ord_cell **first_out, struct ft_ord_cell **last_out,
		struct ft_ord_cell_edge *edges, unsigned int n)
{
	struct ft_ord_cell *first =
		ft_ord_cell_ptr(rcu_dereference(first_head->prev));
	struct ft_ord_cell *last =
		ft_ord_cell_ptr(rcu_dereference(last_head->prev));
	struct ft_ord_cell *pred = ft_ord_cell_resolve_ord(&first->ord_prev);
	struct ft_ord_cell *succ = ft_ord_cell_resolve_ord(&last->ord_next);

	if (pred) {
		edges[n].slot = &pred->ord_next;
		edges[n].old_target = first;
		edges[n].new_target = succ;
		n++;
	}
	if (succ) {
		edges[n].slot = &succ->ord_prev;
		edges[n].old_target = last;
		edges[n].new_target = pred;
		n++;
	}
	n = ft_ord_cell_endpoint_edge(&ft->ord_cell_head, first, succ, edges, n);
	n = ft_ord_cell_endpoint_edge(&ft->ord_cell_tail, last, pred, edges, n);
	*first_out = first;
	*last_out = last;
	return n;
}

/*
 * Install the excised run (its endpoint cells, captured by
 * ft_ord_cell_run_detach_edges) as the ENTIRE ordered list of the exclusive
 * @into trie.  @into has no readers, so plain stores.  MUST run after the flip
 * that excised the run from @ft.
 */
static
void ft_ord_cell_run_install(struct cds_ft *into, struct ft_ord_cell *first,
		struct ft_ord_cell *last)
{
	first->ord_prev = NULL;
	last->ord_next = NULL;
	into->ord_cell_head = first;
	into->ord_cell_tail = last;
}

static
void ft_ord_cell_run_detach(struct cds_ft *ft, struct cds_ft *into,
		struct cds_ft_node *first_head, struct cds_ft_node *last_head)
{
	struct ft_ord_cell_edge edges[4];
	struct ft_ord_cell *first, *last;
	unsigned int n = ft_ord_cell_run_detach_edges(ft, first_head, last_head,
		&first, &last, edges, 0);

	ft_ord_cell_flip(ft, edges, n);
	ft_ord_cell_run_install(into, first, last);
}

/*
 * Cell-side of a key-disappearing structural unlink, fused into ONE flip.  A
 * point remove unsplices a single dead head cell (@cell); a bulk detach excises
 * a contiguous RUN (@rfirst .. @rlast heads) out of @ft and re-homes it as the
 * exclusive @into trie's whole list.  Exactly one of @cell / @into is set (the
 * other zero); both zero means "structural edges only" (ordered list off).
 * @armed is set once the fused commit runs (so a bulk caller knows the run was
 * fused, not left for the standalone two-commit fallback).
 */
struct ft_detach_run {
	struct cds_ft *into;			/* run re-home target (exclusive) */
	struct cds_ft_node *rfirst, *rlast;	/* run endpoint heads, in key order */
	struct ft_ord_cell *first, *last;	/* scratch: filled at flip time */
	bool armed;
};

static
void ft_ord_cell_flip_prealloc(struct cds_ft *ft __attribute__((unused)),
		struct ft_ord_cell_edge *edges, unsigned int n,
		struct ft_flip_batch *b)
{
	unsigned int i;

	if (n == 0) {
		ft_flip_batch_free_unpublished(b);
		return;
	}
	assert(n <= b->cap);
	for (i = 0; i < n; i++)
		rcu_assign_pointer(*edges[i].slot,
			(struct ft_ord_cell *) ft_flip_batch_add(b,
				(struct cds_ft_inode_flag *) edges[i].old_target,
				(struct cds_ft_inode_flag *) edges[i].new_target));
	urcu_flip_commit(&b->group);
	for (i = 0; i < n; i++)
		rcu_assign_pointer(*edges[i].slot, edges[i].new_target);
	ft_flip_batch_reclaim(b);
}

static
void ft_ord_cell_flip(struct cds_ft *ft, struct ft_ord_cell_edge *edges,
		unsigned int n)
{
	struct ft_flip_batch *b;
	unsigned int i;

	if (n == 0)
		return;
	b = ft_flip_batch_alloc(ft, n);
	if (caa_unlikely(!b)) {
		/*
		 * Degraded fallback (point-op splices only, <= 3 edges; the
		 * merge interleave pre-allocates its batch in the fallible
		 * build phase and never lands here): sequential edge stores.
		 * A bidirectional reader between two stores can observe one
		 * neighbour's edge updated and the mirrored one not yet --
		 * transient and self-healing, never a dangling pointer.
		 */
		for (i = 0; i < n; i++)
			rcu_assign_pointer(*edges[i].slot, edges[i].new_target);
		return;
	}
	ft_ord_cell_flip_prealloc(ft, edges, n, b);
}

/*
 * Find the cell of the in-order predecessor (mode LT) / successor (mode GT)
 * of @key via the eager relational descent on the writer's cell scratch
 * iterator.  Returns NULL when none exists (@key is the new minimum/maximum).
 */
static
struct ft_ord_cell *ft_ord_cell_find_rel(struct cds_ft *ft, const uint8_t *key,
		size_t key_len, enum ft_lookup_inequality mode)
{
	struct cds_ft_iter *it = ft->ord_cell_scratch_iter;
	struct cds_ft_node *head;

	if (cds_ft_iter_set_key(it, key, key_len) != CDS_FT_STATUS_OK)
		return NULL;
	it->prefix_len = 0;
	it->node = NULL;
	if (cds_ft_lookup_inequality_impl(ft, it, mode, FT_LOOKUP_LIMIT_NONE,
			false, false) != CDS_FT_STATUS_OK)
		return NULL;
	head = cds_ft_iter_node(it);
	if (!head)
		return NULL;
	return ft_ord_cell_ptr(rcu_dereference(head->prev));
}

/*
 * Find the cell of the in-order predecessor of the freshly-inserted head
 * carried by @cell, WITHOUT re-descending from the root.  The insert just
 * walked root->leaf to attach the head, so its deepest node is the going-up
 * seed: position the writer's scratch iterator AT the new head (a live cursor)
 * and re-enter the relational lookup, which recovers the deepest node from
 * iter->node via the parent chain (its cross-call fast path) and runs the SAME
 * structural backtrack the key-based descent would -- but starting at the
 * divergence point instead of the root.  @seed_from_node suppresses the
 * ordinal-cell fast path (the new head's cell is not yet spliced).
 *
 * @key / @key_len are the head's APPLICATION-form key (set_key remaps): the
 * search key is written straight into iter_key (a memcpy, no up-walk), and the
 * cursor fields are seeded on top so read_key returns that buffer.
 *
 * Returns the predecessor cell, or NULL when the head's key is the new minimum.
 * A compressed/skip-compressed holder makes the impl fall back to a root
 * re-descent internally (correctness preserved, no descent saved for that key).
 */
static
struct ft_ord_cell *ft_ord_cell_find_pred_from_head(struct cds_ft *ft,
		const uint8_t *key, size_t key_len, struct ft_ord_cell *cell,
		bool from_root)
{
	struct cds_ft_iter *it = ft->ord_cell_scratch_iter;
	struct cds_ft_node *pred_head;

	/*
	 * Write the search key into iter_key (set_key clears cache_valid/node and
	 * sets key_len + key_off=0, path_len=0).
	 *
	 * @from_root false (the common live-structure splice): seed a live cursor
	 * AT the new head on top: cache_valid + node + path_len==key_depth drive
	 * the cross-call node-recovery fast path; prefix 0 = unscoped;
	 * ord_cell_node cleared so a fall-back cell cursor re-resolves from
	 * node->prev.
	 *
	 * @from_root true (split-compressed one-commit, the fresh cluster is
	 * parked): the new head sits in an UNPUBLISHED cluster whose live old
	 * child (cn->child) is re-parented only at the commit, so a from-head LT
	 * would reanchor down through that not-yet-wired edge and mis-navigate.
	 * Descend from the root instead -- the parked forward proxy resolves to
	 * the OLD structure, where every existing key (hence the predecessor) is
	 * reachable and consistent.  Costs one extra descent, on the split path
	 * only.
	 */
	it->node = NULL;
	if (cds_ft_iter_set_key(it, key, key_len) != CDS_FT_STATUS_OK)
		return NULL;
	if (!from_root) {
		it->node = cell->node;
		it->cache_valid = true;
		it->ord_cell_node = NULL;
		it->prefix_len = 0;
		it->path_len = it->key_len + 1;
	}
	if (cds_ft_lookup_inequality_impl(ft, it, FT_LOOKUP_LT,
			FT_LOOKUP_LIMIT_NONE, false, !from_root) != CDS_FT_STATUS_OK)
		return NULL;
	pred_head = cds_ft_iter_node(it);
	if (!pred_head)
		return NULL;
	return ft_ord_cell_ptr(rcu_dereference(pred_head->prev));
}

/*
 * Append @cell's ordered-list unsplice edges (its <=2 neighbour back-edges plus
 * any head/tail endpoint repair) to @edges, returning the new count.  @cell
 * keeps its own links for parked readers until its deferred free.  Split out so
 * a key-disappearing remove can fuse these edges with its structural unlink in
 * a single flip (ft_remove_one_commit); ft_ord_cell_unsplice is the standalone
 * (two-commit) wrapper.
 */
static
unsigned int ft_ord_cell_unsplice_edges(struct cds_ft *ft,
		struct ft_ord_cell *cell, struct ft_ord_cell_edge *edges,
		unsigned int n)
{
	struct ft_ord_cell *pred = ft_ord_cell_resolve_ord(&cell->ord_prev);
	struct ft_ord_cell *succ = ft_ord_cell_resolve_ord(&cell->ord_next);

	if (pred) {
		edges[n].slot = &pred->ord_next;
		edges[n].old_target = cell;
		edges[n].new_target = succ;
		n++;
	}
	if (succ) {
		edges[n].slot = &succ->ord_prev;
		edges[n].old_target = cell;
		edges[n].new_target = pred;
		n++;
	}
	n = ft_ord_cell_endpoint_edge(&ft->ord_cell_head, cell, succ, edges, n);
	n = ft_ord_cell_endpoint_edge(&ft->ord_cell_tail, cell, pred, edges, n);
	return n;
}

/* Remove @cell from the ordered cell list (its key disappeared). */
static
void ft_ord_cell_unsplice(struct cds_ft *ft, struct ft_ord_cell *cell)
{
	struct ft_ord_cell_edge edges[4];
	unsigned int n = ft_ord_cell_unsplice_edges(ft, cell, edges, 0);

	ft_ord_cell_flip(ft, edges, n);
}

/*
 * Replace @old_cell with @new_cell at the same list position (insert_replace:
 * a fresh head's cell takes the replaced head's cell slot).  @new_cell
 * inherits @old_cell's neighbours; @old_cell keeps its links for parked
 * readers until its deferred free.  O(1): reuses @old_cell's neighbours, no
 * relational descent.
 */
static
void ft_ord_cell_swap(struct cds_ft *ft, struct ft_ord_cell *old_cell,
		struct ft_ord_cell *new_cell)
{
	struct ft_ord_cell *pred = ft_ord_cell_resolve_ord(&old_cell->ord_prev);
	struct ft_ord_cell *succ = ft_ord_cell_resolve_ord(&old_cell->ord_next);
	struct ft_ord_cell_edge edges[4];
	unsigned int n = 0;

	new_cell->ord_prev = pred;
	new_cell->ord_next = succ;
	if (pred) {
		edges[n].slot = &pred->ord_next;
		edges[n].old_target = old_cell;
		edges[n].new_target = new_cell;
		n++;
	}
	if (succ) {
		edges[n].slot = &succ->ord_prev;
		edges[n].old_target = old_cell;
		edges[n].new_target = new_cell;
		n++;
	}
	n = ft_ord_cell_endpoint_edge(&ft->ord_cell_head, old_cell, new_cell, edges, n);
	n = ft_ord_cell_endpoint_edge(&ft->ord_cell_tail, old_cell, new_cell, edges, n);
	ft_ord_cell_flip(ft, edges, n);
}

/*
 * Replace: swap @new_cell into @old_cell's list slot AND publish the new head
 * (@struct_new replaces @struct_old in @struct_slot) in ONE ft_ord_cell_flip, so
 * the tree-head swap and the ordinal-cell swap commit atomically -- a reader
 * observes the old head WITH its old cell, XOR the new head WITH its new cell,
 * never a published new head whose cell links are not yet swapped.
 *
 * @struct_slot is the SINGLE reader-visible slot the descent reads to reach the
 * head: external_nodes (internal chain), the child slot (plain external child),
 * or the grandparent SKIP_X slot (a leaf reached through a suffix compressed
 * node).  Readers resolve a parked proxy on all three (ft_dereference_external /
 * the descent's ft_resolve_flip_proxy).  Non-reader-visible bookkeeping for the
 * SKIP_X case (cn->child, node->parent) is set by the caller before the flip --
 * the reanchor up-walk never reads cn->child, and node->parent is wired first.
 */
static
void ft_ord_cell_swap_publish(struct cds_ft *ft, struct ft_ord_cell *old_cell,
		struct ft_ord_cell *new_cell,
		struct cds_ft_inode_flag **struct_slot,
		struct cds_ft_inode_flag *struct_old,
		struct cds_ft_inode_flag *struct_new)
{
	struct ft_ord_cell *pred = ft_ord_cell_resolve_ord(&old_cell->ord_prev);
	struct ft_ord_cell *succ = ft_ord_cell_resolve_ord(&old_cell->ord_next);
	struct ft_ord_cell_edge edges[5];
	unsigned int n = 0;

	new_cell->ord_prev = pred;
	new_cell->ord_next = succ;
	edges[n].slot = (struct ft_ord_cell **) struct_slot;
	edges[n].old_target = (struct ft_ord_cell *) struct_old;
	edges[n].new_target = (struct ft_ord_cell *) struct_new;
	n++;
	if (pred) {
		edges[n].slot = &pred->ord_next;
		edges[n].old_target = old_cell;
		edges[n].new_target = new_cell;
		n++;
	}
	if (succ) {
		edges[n].slot = &succ->ord_prev;
		edges[n].old_target = old_cell;
		edges[n].new_target = new_cell;
		n++;
	}
	n = ft_ord_cell_endpoint_edge(&ft->ord_cell_head, old_cell, new_cell, edges, n);
	n = ft_ord_cell_endpoint_edge(&ft->ord_cell_tail, old_cell, new_cell, edges, n);
	ft_ord_cell_flip(ft, edges, n);
}

/*
 * Key-disappearing remove, fused (the dual of ft_ord_cell_swap_publish): commit
 * a key's single reader-visible structural unlink (@struct_slot transitions from
 * @struct_old to @struct_new -- a leaf body slot or compressed cn->child cleared
 * to NULL, or an internal holder's external_nodes cleared) ATOMICALLY with the
 * dead head cell's ordered-list unsplice, in ONE flip.  A reader thus never
 * observes the key gone from the structural index but still present in the
 * ordered list (or vice versa).  @dead_cell may be NULL (ordered list off): then
 * only the structural edge flips.
 *
 * @struct_slot must be the SOLE reader-visible slot whose change removes the
 * key; shapes that touch a second reader-visible slot (a compressed parent's
 * SKIP_X dual pointer, a recompacted node's grandparent edge) do NOT use this.
 */
static
void ft_remove_one_commit(struct cds_ft *ft,
		struct cds_ft_inode_flag **struct_slot,
		struct cds_ft_inode_flag *struct_old,
		struct cds_ft_inode_flag *struct_new,
		struct ft_ord_cell *dead_cell,
		struct ft_detach_run *run)
{
	struct ft_ord_cell_edge edges[5];	/* 1 structural + <=4 cell/run */
	unsigned int n = 0;

	edges[n].slot = (struct ft_ord_cell **) struct_slot;
	edges[n].old_target = (struct ft_ord_cell *) struct_old;
	edges[n].new_target = (struct ft_ord_cell *) struct_new;
	n++;
	if (run)
		n = ft_ord_cell_run_detach_edges(ft, run->rfirst, run->rlast,
			&run->first, &run->last, edges, n);
	else if (dead_cell)
		n = ft_ord_cell_unsplice_edges(ft, dead_cell, edges, n);
	ft_ord_cell_flip(ft, edges, n);
	if (run) {
		ft_ord_cell_run_install(run->into, run->first, run->last);
		run->armed = true;
	}
}

/*
 * Key-disappearing remove via recompaction: commit the recompacted node's
 * 1-2 reader-visible structural stores -- recorded by ft_publish_to_parent
 * into @rec (the forward parent slot, plus a compressed parent's SKIP_X dual
 * pointer) -- ATOMICALLY with the dead head cell's ordered-list unsplice, in
 * ONE flip.  Fusing the forward and skip-dual edges in a single flip also
 * closes the candidate-before-exact ordering window the two-store publish
 * relied on: a reader sees the whole old-XOR-new transition at once.
 * @dead_cell may be NULL (list off): then only the recorded edges flip.
 */
static
void ft_remove_commit_rec(struct cds_ft *ft, struct ft_pub_rec *rec,
		struct ft_ord_cell *dead_cell, struct ft_detach_run *run)
{
	struct ft_ord_cell_edge edges[6];	/* <=2 structural + <=4 cell/run */
	unsigned int n = 0, i;

	for (i = 0; i < rec->n; i++) {
		edges[n].slot = (struct ft_ord_cell **) rec->slot[i];
		edges[n].old_target = (struct ft_ord_cell *) rec->old_val[i];
		edges[n].new_target = (struct ft_ord_cell *) rec->new_val[i];
		n++;
	}
	if (run)
		n = ft_ord_cell_run_detach_edges(ft, run->rfirst, run->rlast,
			&run->first, &run->last, edges, n);
	else if (dead_cell)
		n = ft_ord_cell_unsplice_edges(ft, dead_cell, edges, n);
	ft_ord_cell_flip(ft, edges, n);
	if (run) {
		ft_ord_cell_run_install(run->into, run->first, run->last);
		run->armed = true;
	}
}

/*
 * Locate the ordered-list neighbours (@pred, @succ) that a run grafted at @key
 * will splice between.  MUST be called while @dst is still payload-free (before
 * the structural attach publishes the grafted subtree), else the relational
 * descent would return a payload head as the boundary.  @key is APPLICATION form
 * (find_rel remaps).  Since the attach point is empty, @pred = last @dst key <
 * @key and @succ = first @dst key > @key (nothing of @dst's lies in the run's
 * range in between).
 */
static
void ft_ord_cell_find_splice_pos(struct cds_ft *dst, const uint8_t *key,
		size_t key_len, struct ft_ord_cell **pred_out,
		struct ft_ord_cell **succ_out)
{
	struct ft_ord_cell *pred, *succ;
	size_t flen = dst->group->key_len;

	if (flen != CDS_FT_LEN_VARIABLE && key_len != flen) {
		/*
		 * Internal graft on a FIXED-length group (ft_graft_keylen, e.g.
		 * cds_ft_merge_at's detach+graft path): @key is the graft
		 * PREFIX, shorter than the group's key length, which the public
		 * relational lookups reject -- probing with it verbatim came
		 * back empty and the grafted run was silently never spliced
		 * (merged keys invisible to ordered iteration).  Probe with the
		 * prefix PADDED to the fixed length instead: the attach point
		 * is empty (graft returns POPULATED_ERROR otherwise), so no
		 * @dst key starts with @key, and
		 *   LT(key . min..min) = last @dst key below the prefix range,
		 *   GT(key . max..max) = first @dst key above it.
		 * Pad bytes are the ordinal-space extremes mapped back to
		 * application form (find_rel remaps app -> ordinal).
		 */
		const struct cds_ft_key_map *km = &dst->group->key_map;
		uint8_t pad_min = km->identity ? 0x00 : km->ordinal_to_key[0x00];
		uint8_t pad_max = km->identity ? 0xff : km->ordinal_to_key[0xff];
		uint8_t pad[FT_MAX_KEY_LEN];

		assert(key_len < flen && flen <= FT_MAX_KEY_LEN);
		memcpy(pad, key, key_len);
		memset(pad + key_len, pad_min, flen - key_len);
		pred = ft_ord_cell_find_rel(dst, pad, flen, FT_LOOKUP_LT);
		if (pred) {
			succ = ft_ord_cell_resolve_ord(&pred->ord_next);
		} else {
			memset(pad + key_len, pad_max, flen - key_len);
			succ = ft_ord_cell_find_rel(dst, pad, flen,
					FT_LOOKUP_GT);
		}
		*pred_out = pred;
		*succ_out = succ;
		return;
	}

	pred = ft_ord_cell_find_rel(dst, key, key_len, FT_LOOKUP_LT);
	if (pred)
		succ = ft_ord_cell_resolve_ord(&pred->ord_next);
	else
		succ = ft_ord_cell_find_rel(dst, key, key_len, FT_LOOKUP_GT);
	*pred_out = pred;
	*succ_out = succ;
}

/*
 * Splice the contiguous ordered-list run [@run_first .. @run_last] (already
 * linked internally, in key order) into @dst's ordered cell list BETWEEN the
 * given neighbours @pred and @succ -- the cds_ft_graft shape, where the run is
 * the source trie's whole list attached at an EMPTY point in @dst (graft returns
 * POPULATED_ERROR otherwise, so no @dst key interleaves the run's range).
 *
 * @pred / @succ MUST be located BEFORE the structural attach publishes the
 * payload into @dst (see ft_ord_cell_find_splice_pos): a relational descent run
 * after the payload is live would return a PAYLOAD head (part of the run itself)
 * as the boundary.  @pred / @succ are @dst-original cells, which graft never
 * moves, so they stay valid until this splice.
 *
 * Pre-sets the run's outer links (run not yet reachable in @dst) and APPENDS
 * the <=2 neighbour edges plus any @dst head/tail repair to @edges, leaving the
 * caller to flip them.  Split out (the appear-side dual of
 * ft_ord_cell_run_detach_edges) so a bulk graft can FUSE these edges with its
 * structural attach publish in ONE flip (ft_store_at_graft_point's batch),
 * closing the appear-side cross-view window; ft_ord_cell_run_splice is the
 * standalone (two-commit) wrapper.
 */
static
unsigned int ft_ord_cell_run_splice_edges(struct cds_ft *dst,
		struct ft_ord_cell *run_first, struct ft_ord_cell *run_last,
		struct ft_ord_cell *pred, struct ft_ord_cell *succ,
		struct ft_ord_cell_edge *edges, unsigned int n)
{
	/* Pre-set the run's outer links; not yet reachable via @dst's list. */
	run_first->ord_prev = pred;
	run_last->ord_next = succ;
	if (pred) {
		edges[n].slot = &pred->ord_next;
		edges[n].old_target = succ;
		edges[n].new_target = run_first;
		n++;
	}
	if (succ) {
		edges[n].slot = &succ->ord_prev;
		edges[n].old_target = pred;
		edges[n].new_target = run_last;
		n++;
	}
	n = ft_ord_cell_endpoint_edge(&dst->ord_cell_head, succ, run_first, edges, n);
	n = ft_ord_cell_endpoint_edge(&dst->ord_cell_tail, pred, run_last, edges, n);
	return n;
}

/*
 * Pre-sets the run's outer links (run not yet reachable in @dst), flips the
 * <=2 boundary edges atomically (for @dst's live readers), and repairs @dst
 * head/tail.  The run's source trie must already have released it (head/tail
 * cleared + a grace period) so no source reader is mid-run.
 */
static
void ft_ord_cell_run_splice(struct cds_ft *dst, struct ft_ord_cell *run_first,
		struct ft_ord_cell *run_last, struct ft_ord_cell *pred,
		struct ft_ord_cell *succ)
{
	struct ft_ord_cell_edge edges[4];
	unsigned int n = ft_ord_cell_run_splice_edges(dst, run_first, run_last,
		pred, succ, edges, 0);

	ft_ord_cell_flip(dst, edges, n);
}

/*
 * Appear-side run-splice fusion descriptor (the dual of struct ft_detach_run):
 * a bulk graft attaches the source trie's whole former ordered-list run
 * [@run_first .. @run_last] between @dst's @pred / @succ neighbours.  Threaded
 * through ft_store_at_graft_point so the run-splice boundary edges join the
 * SAME flip as the structural attach publish -- a reader then never observes a
 * grafted key present in the structure but absent from the ordered list (or
 * vice versa).  @armed reports that the run was fused (so cds_ft_graft skips the
 * standalone two-commit ft_ord_cell_run_splice fallback).
 */
struct ft_graft_run {
	struct ft_ord_cell *run_first, *run_last;	/* src's captured former list */
	struct ft_ord_cell *pred, *succ;		/* dst splice neighbours */
	bool armed;
};

/*
 * Commit a graft's RECORDED structural publish edges (@rec: the forward parent
 * slot, plus a compressed parent's SKIP_X dual) ATOMICALLY with @run's
 * ordered-list run-splice, in ONE ft_ord_cell_flip -- the shared tail of the
 * GLUE and displaced-external graft shapes whose publish is a direct store (not
 * a parked flip-batch proxy like the in-place slot shape).  Arms @run.
 */
static
void ft_ord_cell_flip_rec_run(struct cds_ft *ft, struct ft_pub_rec *rec,
		struct ft_graft_run *run)
{
	struct ft_ord_cell_edge edges[6];	/* <=2 structural + <=4 cell */
	unsigned int n = 0, i;

	for (i = 0; i < rec->n; i++) {
		edges[n].slot = (struct ft_ord_cell **) rec->slot[i];
		edges[n].old_target = (struct ft_ord_cell *) rec->old_val[i];
		edges[n].new_target = (struct ft_ord_cell *) rec->new_val[i];
		n++;
	}
	n = ft_ord_cell_run_splice_edges(ft, run->run_first, run->run_last,
		run->pred, run->succ, edges, n);
	ft_ord_cell_flip(ft, edges, n);
	run->armed = true;
}

/*
 * Replace the run [@d_first .. @d_last] currently in @dst's ordered list with
 * the run [@s_first .. @s_last] at the SAME position -- the cds_ft_graft_swap
 * shape, where @dst's subtree-at-key (run_D) is swapped out for the swap trie's
 * content (run_S).  @s_first may be NULL (empty swap -> run_D just leaves and the
 * gap closes).  The position is taken from run_D's own neighbours (no relational
 * descent: the swap exchanges two subtrees at the same key, so run_S lands
 * exactly where run_D was).  Atomic for @dst's live readers via one flip of the
 * <=2 boundary edges.  run_D keeps its links for parked readers; the caller
 * re-homes run_D into the swap trie afterwards.
 */
static
void ft_ord_cell_run_replace(struct cds_ft *dst,
		struct ft_ord_cell *d_first, struct ft_ord_cell *d_last,
		struct ft_ord_cell *s_first, struct ft_ord_cell *s_last)
{
	struct ft_ord_cell *pred = ft_ord_cell_resolve_ord(&d_first->ord_prev);
	struct ft_ord_cell *succ = ft_ord_cell_resolve_ord(&d_last->ord_next);
	struct ft_ord_cell *new_first = s_first ? s_first : succ;
	struct ft_ord_cell *new_last = s_last ? s_last : pred;
	struct ft_ord_cell_edge edges[4];
	unsigned int n = 0;

	if (s_first) {
		/* Pre-set run_S's outer links; not yet reachable via @dst. */
		s_first->ord_prev = pred;
		s_last->ord_next = succ;
	}
	if (pred) {
		edges[n].slot = &pred->ord_next;
		edges[n].old_target = d_first;
		edges[n].new_target = new_first;
		n++;
	}
	if (succ) {
		edges[n].slot = &succ->ord_prev;
		edges[n].old_target = d_last;
		edges[n].new_target = new_last;
		n++;
	}
	n = ft_ord_cell_endpoint_edge(&dst->ord_cell_head, d_first, new_first, edges, n);
	n = ft_ord_cell_endpoint_edge(&dst->ord_cell_tail, d_last, new_last, edges, n);
	ft_ord_cell_flip(dst, edges, n);
}

/*
 * Remove the contiguous run [@first_head .. @last_head] from @ft's ordered list
 * WITHOUT re-homing it -- the cds_ft_merge source side, where the run's cells
 * disperse (survivors are spliced into dst, collided heads are freed).  Relink
 * the two boundary edges (atomic for @ft's live readers) and repair head/tail;
 * the run cells keep their stale links (caller no longer references them as a
 * run).  Whole-list removal (pred == succ == NULL) clears head/tail.
 */
#ifdef FEATURE_FT_MERGE
static
void ft_ord_cell_run_unlink(struct cds_ft *ft, struct cds_ft_node *first_head,
		struct cds_ft_node *last_head)
{
	struct ft_ord_cell *first =
		ft_ord_cell_ptr(rcu_dereference(first_head->prev));
	struct ft_ord_cell *last =
		ft_ord_cell_ptr(rcu_dereference(last_head->prev));
	struct ft_ord_cell *pred = ft_ord_cell_resolve_ord(&first->ord_prev);
	struct ft_ord_cell *succ = ft_ord_cell_resolve_ord(&last->ord_next);
	struct ft_ord_cell_edge edges[4];
	unsigned int n = 0;

	if (pred) {
		edges[n].slot = &pred->ord_next;
		edges[n].old_target = first;
		edges[n].new_target = succ;
		n++;
	}
	if (succ) {
		edges[n].slot = &succ->ord_prev;
		edges[n].old_target = last;
		edges[n].new_target = pred;
		n++;
	}
	n = ft_ord_cell_endpoint_edge(&ft->ord_cell_head, first, succ, edges, n);
	n = ft_ord_cell_endpoint_edge(&ft->ord_cell_tail, last, pred, edges, n);
	ft_ord_cell_flip(ft, edges, n);
}
#endif /* FEATURE_FT_MERGE */

/*
 * Set @child's parent back-pointer to a raw flag @value (a flip-proxy)
 * verbatim, WITHOUT touching parent_slot_offset.  Mirrors ft_set_parent's
 * child-kind dispatch; the settle step later replaces @value with the
 * real parent via ft_set_parent (which does maintain skip_slot).
 */
static
void ft_set_parent_raw(struct cds_ft *ft, struct cds_ft_inode_flag *child,
		struct cds_ft_inode_flag *value)
{
	(void) ft;
	if (!child)
		return;
	/* Flip-proxy child: transient slot value, not a node (see
	 * ft_set_parent); the parking mutator wires the real child. */
	if (caa_unlikely(ft_node_flip_proxy(child)))
		return;
#ifdef FEATURE_FT_SKIP_COMPRESSED
	if (ft_node_skip_compressed(child)) {
		struct cds_ft_compressed_node *cn = ft_skip_to_compressed(ft, child);

		rcu_assign_pointer(cds_ft_item_to_metadata(
			(struct cds_ft_inode *) cn)->parent, value);
		return;
	}
	if (ft_node_compressed(child)) {
		struct cds_ft_compressed_node *cn = ft_compressed_node_ptr(child);

		rcu_assign_pointer(cds_ft_item_to_metadata(
			(struct cds_ft_inode *) cn)->parent, value);
		return;
	}
#endif
	if (ft_node_external(child)) {
		/*
		 * Ordered list on: store the raw flag (a flip-proxy) into the head's
		 * cell->parent, not over its prev (which is the cell pointer).  The
		 * read path resolves cell->parent THEN the flip-proxy, so a proxy
		 * parked in cell->parent settles correctly; the later ft_set_parent
		 * replaces it with the real parent.  List off / non-cell: the head's
		 * prev IS the flagged parent, so store the proxy directly there.
		 */
		if (ft->ordered_list)
			ft_ord_cell_set_parent((struct cds_ft_node *) child, value);
		else
			rcu_assign_pointer(((struct cds_ft_node *) child)->prev, value);
		return;
	}
	rcu_assign_pointer(cds_ft_item_to_metadata(ft_node_ptr(child))->parent,
		value);
}

/*
 * ft_propagate_external_count_parent: propagate a signed nr_keys delta from
 * @start up to the root via metadata->parent pointers.
 *
 * @start: deepest internal/compressed node on the path (the node where the
 *         external was attached, or the deepest ancestor with metadata).
 *         Must not be an external node or NULL.
 * @delta: +1 for insert, -1 for remove.
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
static
void ft_propagate_external_count_parent(struct cds_ft *ft,
		struct cds_ft_inode_flag *start, long delta)
{
	struct cds_ft_inode_flag *cur = start;

	(void) ft;
	ft_delay_writer();

	while (cur) {
		struct cds_ft_metadata *m =
			cds_ft_item_to_metadata(ft_node_ptr(cur));
		ft_nr_keys_store(m, ft_nr_keys_get(m) + delta, CMM_RELEASE);
		ft_delay_writer();
		cur = m->parent;
	}
}

/*
 * Fill @r with a generous SUPERSET of the nodes a bulk op's commit can allocate
 * -- CDS_FT_ALLOC_RESERVE_CAP of every internal node type (its own order +
 * bitmap), the minimal order, and (speculative groups) the compressed-node
 * orders.  Drawn before the op's last fallible step, this lets the commit draw
 * and never fail on an arena allocation, so nothing after that step needs a
 * reader-observable rollback.  Used by the same-trie rekey (before its detach)
 * and by ft_graft_keylen's NOSPLIT attach (before it publishes the empty source
 * root).  A generous superset avoids predicting the exact manifest; a bulk op
 * already pays an RCU grace period, so the handful of throwaway arena
 * pops/pushes is negligible.  Returns 0, or -ENOMEM (caller drains).
 */
static
int ft_bulk_node_reserve_fill(struct cds_ft *ft, struct cds_ft_alloc_reserve *r)
{
	unsigned int ntypes = (unsigned int) (sizeof(ft_types) / sizeof(ft_types[0]));
	unsigned int i;
	int ret = 0;

	for (i = 0; i < ntypes && !ret; i++) {
		if (ft_types[i].type_class == FT_NULL)
			continue;
		ret = cds_ft_alloc_reserve_add(ft, r, CDS_FT_ALLOC_KIND_NODE,
			ft_types[i].order, ft_types[i].bitmap,
			CDS_FT_ALLOC_RESERVE_CAP);
	}
	/* Minimal order, below ft_types[0] (small / compressed nodes). */
	if (!ret && FT_ALLOC_ORDER_MIN < ft_types[0].order)
		ret = cds_ft_alloc_reserve_add(ft, r, CDS_FT_ALLOC_KIND_NODE,
			FT_ALLOC_ORDER_MIN, FT_NO_BITMAP,
			CDS_FT_ALLOC_RESERVE_CAP);
	/* Compressed-node arena (speculative groups; else compressed == NODE). */
	if (ft->group->speculative) {
		unsigned int order;
		unsigned int cmax = ft_compressed_order(FT_SKIP_LEN_MAX);

		if (cmax > FT_ALLOC_ORDER_MAX)
			cmax = FT_ALLOC_ORDER_MAX;
		for (order = FT_ALLOC_ORDER_MIN; order <= cmax && !ret; order++)
			ret = cds_ft_alloc_reserve_add(ft, r,
				CDS_FT_ALLOC_KIND_COMPRESSED, order,
				FT_NO_BITMAP, CDS_FT_ALLOC_RESERVE_CAP);
	}
	return ret;
}

/*
 * ft_glue: write-path attach-transaction subsystem.  Build a node cluster
 * entirely from fresh, unobservable nodes, then splice it into live data with a
 * single commit-time publish + deferred back-pointer wiring.  Shared by insert
 * (NOSPLIT store), remove, graft and merge -- relocated here (structs from
 * ft-insert.h, helper bodies from ft-graft.h) so the whole subsystem precedes
 * every user and needs no cross-module forward declarations.
 */
/*
 * Write-path attach-transaction glue (build-invisible / publish / reclaim --
 * see the rcu-mutation discipline).  The motivating case: a graft attaches a
 * payload subtrie at a non-root key.  The attach cluster ("glue") is built
 * entirely from fresh, unobservable nodes BEFORE the source root is unlinked,
 * so an allocation failure frees the glue with both tries pristine -- there
 * is nothing to roll back, and no past-sync abort().
 *
 * Every edge from the glue into LIVE data is a back-pointer re-parent
 * that must be deferred to the failure-free commit and applied only
 * after the source is unlinked and a grace period has drained its
 * readers.  Two flavours of live data:
 *   - the displaced dst old-child of a split compressed node, and
 *   - the live payload nodes pulled from the source (its old root, and
 *     any sub-compressed absorbed during canonicalization).
 * Forward edges into live data (a fresh node's child slot pointing at a
 * live node) ARE set during the build: they live in unobservable glue
 * nodes, so no reader follows them until the single commit-time publish.
 *
 * @built tracks every fresh glue node so the abort path can free them
 * (immediate free -- never observed).  @deferred records the live
 * back-pointers to wire at commit.  @free_list records old (replaced)
 * live nodes to reclaim deferred after the publish.
 *
 * The struct is instantiable more than once: cds_ft_graft uses a single
 * glue for the dst-side attach; cds_ft_graft_swap commits two (the
 * dst-side insert glue + the swap-side extracted-root glue) together.
 *
 * Defined up here (rather than with its helper bodies further down)
 * because ft_try_compress_chain, ft_build_branch and the build-only
 * graft split all reference the complete type.
 */
struct ft_glue_deferred_edge {
	struct cds_ft_inode_flag *child;	/* live node to re-parent */
	struct cds_ft_inode_flag *parent;	/* glue node it will point to */
	struct cds_ft_inode_flag **slot;	/* slot in parent holding child */
	/*
	 * cds_ft_merge_at references live subtrees from BOTH tries.  A
	 * src-origin child is drained by the early src unlink (applied at
	 * apply_deferred); a dst-origin child stays reachable via the old dst
	 * spine until the forward publish + dst drain, so its back-pointer flip
	 * must wait (applied at apply_deferred_dst).  graft / graft_swap only
	 * ever re-parent src-origin nodes, so this defaults to false and their
	 * single apply_deferred call still wires every edge.
	 */
	bool dst_origin;
};

struct ft_glue_free_item {
	void *node;		/* cds_ft_inode * or cds_ft_compressed_node * */
	bool compressed;
};

/*
 * A deferred duplicate-chain splice, used only by cds_ft_merge_at when the
 * SAME full key exists in both tries: the two LIVE external chains must be
 * concatenated under the fresh merged node @owner.  graft / graft_swap never
 * concatenate two live chains, so this is merge-only.
 *
 * @dst_head is kept as the surviving chain head: its forward owner (a fresh
 * merged node's metadata->external_nodes, or a fresh merged node's child slot)
 * and its back-pointer (@dst_head->prev = that node) are wired by the ordinary
 * Phase-1 set + deferred edge, exactly like any other re-parented external.
 * This struct carries ONLY the concatenation, which is publication-visible on
 * two live chains and is applied at commit by ft_glue_apply_splices,
 * AFTER the source has been detached + drained: the @src_head chain is appended
 * to @dst_head's tail (prev-before-next, the ft_chain_node idiom, but preserving
 * src_head->next so the rest of the src chain rides along).
 */
struct ft_glue_splice {
	struct cds_ft_node *dst_head;		/* surviving head (kept first) */
	struct cds_ft_node *src_head;		/* appended to dst_head's tail */
	/*
	 * The demoted @src_head's ordered-list cell, captured by
	 * ft_glue_apply_splices.  It stays REACHABLE through its src-run
	 * neighbours' stale ord_prev/ord_next until the post-publish interleave
	 * rewires them, so it is freed only by
	 * ft_glue_free_collided_cells, called after the interleave, via
	 * the grace-period-deferred cell free.  NULL when the list is off.
	 */
	struct ft_ord_cell *src_cell;
};

/*
 * Inline floor sizing: a graft / graft_swap attach cluster spans at most a
 * compressed prefix + branch + suffix + a payload path of up to FT_MAX_DEPTH
 * nodes + a canonicalization wrapper, with few deferred edges and freed nodes
 * (old-child; payload top / grandchild; old cn, old src root, absorbed
 * sub-cn).  These fit the inline arrays, so graft / graft_swap never allocate
 * a backing buffer and never grow past the floor.
 *
 * cds_ft_merge_at instead builds a TREE-shaped spine (one deferred edge per
 * disjoint subtree, one free per copied node), which can far exceed the floor.
 * It calls ft_glue_reserve() to move the three arrays onto a malloc'd
 * backing sized by a read-only counting pre-pass; ft_glue_abort() and
 * ft_glue_fini() release it.  Every accessor indexes through the
 * pointers, so the growth is invisible to the helpers.
 */
#define FT_GLUE_FLOOR_BUILT	(2 * FT_MAX_DEPTH + 8)
#define FT_GLUE_FLOOR_DEFERRED	8
#define FT_GLUE_FLOOR_FREE	8
#define FT_GLUE_FLOOR_SPLICE	8

struct ft_glue {
	struct ft_glue_deferred_edge *deferred;
	int nr_deferred;
	int cap_deferred;
	struct ft_glue_free_item *free_list;
	int nr_free;
	int cap_free;
	struct cds_ft_inode_flag **built;
	int nr_built;
	int cap_built;
	struct ft_glue_splice *splices;
	int nr_splices;
	int cap_splices;
	/*
	 * The single forward store that splices the cluster into dst at
	 * commit: parent_slot is swung to top.  publish_parent is the
	 * node flag owning the slot (for compressed skip bookkeeping;
	 * NULL at the root).  The cluster top's own back-pointer into
	 * publish_parent is recorded as an ordinary deferred edge.
	 */
	struct cds_ft_inode_flag *publish_parent;
	struct cds_ft_inode_flag **publish_slot;
	struct cds_ft_inode_flag *top;
	/*
	 * Node whose nr_keys == the grafted payload's key count, and from
	 * whose parent the external-count propagation starts at commit.
	 */
	struct cds_ft_inode_flag *attached_nf;
	/*
	 * Inline floor backing.  ft_glue_init points the three arrays
	 * here; graft / graft_swap never outgrow it.  ft_glue_reserve
	 * repoints to a malloc'd buffer when a count would exceed its floor.
	 */
	struct ft_glue_deferred_edge deferred_floor[FT_GLUE_FLOOR_DEFERRED];
	struct ft_glue_free_item free_floor[FT_GLUE_FLOOR_FREE];
	struct cds_ft_inode_flag *built_floor[FT_GLUE_FLOOR_BUILT];
	struct ft_glue_splice splices_floor[FT_GLUE_FLOOR_SPLICE];
};

/*
 * ft_glue helpers.  The struct and the rationale are defined just above;
 * the build-invisible builders that reference the type (ft_build_branch and
 * the graft / merge spine builders) follow in the later mutation modules.
 */
static
void ft_glue_init(struct ft_glue *g)
{
	g->deferred = g->deferred_floor;
	g->nr_deferred = 0;
	g->cap_deferred = FT_GLUE_FLOOR_DEFERRED;
	g->free_list = g->free_floor;
	g->nr_free = 0;
	g->cap_free = FT_GLUE_FLOOR_FREE;
	g->built = g->built_floor;
	g->nr_built = 0;
	g->cap_built = FT_GLUE_FLOOR_BUILT;
	g->splices = g->splices_floor;
	g->nr_splices = 0;
	g->cap_splices = FT_GLUE_FLOOR_SPLICE;
	g->publish_parent = NULL;
	g->publish_slot = NULL;
	g->top = NULL;
	g->attached_nf = NULL;
}

/*
 * Release a glue's malloc'd backing (when it grew past the inline floor) and
 * reset the arrays to the floor, so a second call is a no-op.  Both the abort
 * path (via ft_glue_abort) and the success path call this; graft /
 * graft_swap stay on the floor, so it does nothing for them.
 */
static
void ft_glue_fini(struct ft_glue *g)
{
	if (g->deferred != g->deferred_floor) {
		free(g->deferred);
		g->deferred = g->deferred_floor;
		g->cap_deferred = FT_GLUE_FLOOR_DEFERRED;
	}
	if (g->free_list != g->free_floor) {
		free(g->free_list);
		g->free_list = g->free_floor;
		g->cap_free = FT_GLUE_FLOOR_FREE;
	}
	if (g->built != g->built_floor) {
		free(g->built);
		g->built = g->built_floor;
		g->cap_built = FT_GLUE_FLOOR_BUILT;
	}
	if (g->splices != g->splices_floor) {
		free(g->splices);
		g->splices = g->splices_floor;
		g->cap_splices = FT_GLUE_FLOOR_SPLICE;
	}
}

/*
 * Grow the three arrays onto a malloc'd backing so the build can hold a
 * cluster larger than the inline floor (cds_ft_merge_at's tree-shaped spine).
 * Sizes come from a read-only counting pre-pass; call once, right after init,
 * before any track / defer.  A request at or below a floor leaves that array
 * inline.  Returns 0, or -ENOMEM (whatever already grew is released by a
 * later abort / fini, so the caller need only surface the error).
 */
#ifdef FEATURE_FT_MERGE
static
int ft_glue_reserve(struct ft_glue *g,
		int nr_built, int nr_deferred, int nr_free, int nr_splices)
{
	if (nr_built > g->cap_built) {
		struct cds_ft_inode_flag **p =
			malloc((size_t) nr_built * sizeof(*p));

		if (!p)
			return -ENOMEM;
		memcpy(p, g->built, (size_t) g->nr_built * sizeof(*p));
		if (g->built != g->built_floor)
			free(g->built);
		g->built = p;
		g->cap_built = nr_built;
	}
	if (nr_deferred > g->cap_deferred) {
		struct ft_glue_deferred_edge *p =
			malloc((size_t) nr_deferred * sizeof(*p));

		if (!p)
			return -ENOMEM;
		memcpy(p, g->deferred, (size_t) g->nr_deferred * sizeof(*p));
		if (g->deferred != g->deferred_floor)
			free(g->deferred);
		g->deferred = p;
		g->cap_deferred = nr_deferred;
	}
	if (nr_free > g->cap_free) {
		struct ft_glue_free_item *p =
			malloc((size_t) nr_free * sizeof(*p));

		if (!p)
			return -ENOMEM;
		memcpy(p, g->free_list, (size_t) g->nr_free * sizeof(*p));
		if (g->free_list != g->free_floor)
			free(g->free_list);
		g->free_list = p;
		g->cap_free = nr_free;
	}
	if (nr_splices > g->cap_splices) {
		struct ft_glue_splice *p =
			malloc((size_t) nr_splices * sizeof(*p));

		if (!p)
			return -ENOMEM;
		memcpy(p, g->splices, (size_t) g->nr_splices * sizeof(*p));
		if (g->splices != g->splices_floor)
			free(g->splices);
		g->splices = p;
		g->cap_splices = nr_splices;
	}
	return 0;
}
#endif /* FEATURE_FT_MERGE */

/* Record a fresh glue node so the abort path can free it. */
static
void ft_glue_track(struct ft_glue *g,
		struct cds_ft_inode_flag *nf)
{
	assert(g->nr_built < g->cap_built);
	/*
	 * PLAIN forms only: a SKIP pointer encodes the CHILD's address, so
	 * the abort path's kind dispatch would free the wrong node (the 2.1
	 * corruption shape) and the identity helpers would have to chase the
	 * child's back-pointer mid-build.  Callers track the plain compressed
	 * flag and re-encode separately for the slot publish.
	 */
	assert(!ft_node_skip_compressed(nf));
	g->built[g->nr_built++] = nf;
}

/*
 * Drop a tracked glue node that has been consumed (chain-merge absorbs a
 * freshly-built compressed wrapper and frees it during the build).  Match
 * by underlying node identity so a plain-flag tracking entry is found
 * even when the absorbed reference is skip-encoded.  No-op if absent.
 */
static
void ft_glue_untrack(struct cds_ft *ft, struct ft_glue *g, void *node_ptr)
{
	int i;

	for (i = 0; i < g->nr_built; i++) {
		struct cds_ft_inode_flag *nf = g->built[i];
		void *p;

		if (ft_node_compressed(nf))
			p = ft_compressed_node_ptr(nf);
		else if (ft_node_skip_compressed(nf))
			p = ft_skip_to_compressed(ft, nf);
		else
			p = ft_node_ptr(nf);
		if (p == node_ptr) {
			g->built[i] = g->built[--g->nr_built];
			return;
		}
	}
}

/*
 * Test whether @child names a node currently in the glue's tracked
 * (fresh, unpublished) set.  Match by underlying node identity so a
 * plain-flag tracking entry is found even when @child is the skip-
 * encoded form (or vice versa).
 */
static
bool ft_glue_is_fresh(struct cds_ft *ft, struct ft_glue *g,
		struct cds_ft_inode_flag *child)
{
	void *cp;
	int i;

	if (!child)
		return false;
	if (ft_node_compressed(child))
		cp = ft_compressed_node_ptr(child);
	else if (ft_node_skip_compressed(child))
		cp = ft_skip_to_compressed(ft, child);
	else if (ft_node_external(child))
		return false;	/* externals are never glue-tracked */
	else
		cp = ft_node_ptr(child);
	for (i = 0; i < g->nr_built; i++) {
		struct cds_ft_inode_flag *nf = g->built[i];
		void *bp;

		if (ft_node_compressed(nf))
			bp = ft_compressed_node_ptr(nf);
		else if (ft_node_skip_compressed(nf))
			bp = ft_skip_to_compressed(ft, nf);
		else
			bp = ft_node_ptr(nf);
		if (bp == cp)
			return true;
	}
	return false;
}

/*
 * Wire a back-pointer (@child->parent = @parent, slot @slot) within a
 * build-invisible graft cluster.
 *
 * Two regimes by @child kind, the rule that the user pinned down:
 *
 *   - Internal-to-glue (@child is a freshly-allocated, glue-tracked node):
 *     store IMMEDIATELY.  The child is unpublished, so the rcu_assign_pointer
 *     is invisible to readers; doing it now (rather than deferring) means
 *     the entire intra-cluster back-pointer chain is fully wired by the
 *     time apply_deferred flips any LIVE-data back-pointer into the
 *     cluster.  An up-walk from re-parented live data then sees a coherent
 *     chain from the live edge through the cluster up into publish_parent
 *     -- no transient NULL parents along the way.
 *
 *   - External-to-glue (@child is a LIVE node being re-parented INTO the
 *     glue cluster, e.g. the displaced old child of a diverge split or
 *     the source-payload leaf at the bottom of the new branch): record
 *     the (child, parent, slot) tuple and defer the ft_set_parent until
 *     ft_glue_apply_deferred at commit time, after the source has
 *     been drained.  Flipping a live back-pointer during the build would
 *     be a publication-visible mutation before the cluster is observable.
 *
 *   Deferred entries are de-duplicated by @child so a canonicalization
 *   wrapper later absorbed by a chain-merge keeps only its final mapping.
 */
static
void ft_glue_defer_edge_origin(struct cds_ft *ft, struct ft_glue *g,
		struct cds_ft_inode_flag *child,
		struct cds_ft_inode_flag *parent,
		struct cds_ft_inode_flag **slot,
		bool dst_origin)
{
	int i;

	if (ft_glue_is_fresh(ft, g, child)) {
		ft_set_parent(ft, child, parent, slot);
		return;
	}
	for (i = 0; i < g->nr_deferred; i++) {
		if (g->deferred[i].child == child) {
			g->deferred[i].parent = parent;
			g->deferred[i].slot = slot;
			g->deferred[i].dst_origin = dst_origin;
			return;
		}
	}
	assert(g->nr_deferred < g->cap_deferred);
	g->deferred[g->nr_deferred].child = child;
	g->deferred[g->nr_deferred].parent = parent;
	g->deferred[g->nr_deferred].slot = slot;
	g->deferred[g->nr_deferred].dst_origin = dst_origin;
	g->nr_deferred++;
}

static
void ft_glue_defer_edge(struct cds_ft *ft, struct ft_glue *g,
		struct cds_ft_inode_flag *child,
		struct cds_ft_inode_flag *parent,
		struct cds_ft_inode_flag **slot)
{
	ft_glue_defer_edge_origin(ft, g, child, parent, slot,
		/*dst_origin*/ false);
}

/*
 * Record the single forward publish that splices the built cluster into
 * dst, and wire the cluster top's back-pointer into its (live)
 * publish_parent.  The builders call this once the cluster is fully
 * built.  top is fresh and unpublished, so ft_glue_defer_edge
 * stores top->parent IMMEDIATELY (its fresh-child fast path); the store
 * lands before any other commit-time mutation, so by the time
 * apply_deferred flips any live back-pointer into the cluster, the
 * up-walk path from cluster nodes through top into publish_parent is
 * already wired.
 */
static
void ft_glue_set_publish(struct cds_ft *ft, struct ft_glue *g,
		struct cds_ft_inode_flag *parent_nf,
		struct cds_ft_inode_flag **parent_slot,
		struct cds_ft_inode_flag *top)
{
	g->publish_parent = parent_nf;
	g->publish_slot = parent_slot;
	g->top = top;
	ft_glue_defer_edge(ft, g, top, parent_nf, parent_slot);
}

/* Record an old (replaced) live node to reclaim deferred at commit. */
static
void ft_glue_defer_free(struct ft_glue *g,
		void *node, bool compressed)
{
	assert(g->nr_free < g->cap_free);
	g->free_list[g->nr_free].node = node;
	g->free_list[g->nr_free].compressed = compressed;
	g->nr_free++;
}

/*
 * Abort the build: free every freshly-built (never-observed) glue node, then
 * release the malloc'd backing.  Both tries are left pristine -- no deferred
 * edge was applied, so no live back-pointer references the glue.
 */
static
void ft_glue_abort(struct cds_ft *ft, struct ft_glue *g)
{
	int i;

	for (i = 0; i < g->nr_built; i++) {
		struct cds_ft_inode_flag *nf = g->built[i];

		if (ft_node_compressed(nf))
			free_compressed_node_unpublished(ft,
				ft_compressed_node_ptr(nf));
		else if (ft_node_skip_compressed(nf))
			free_compressed_node_unpublished(ft,
				ft_skip_to_compressed(ft, nf));
		else
			free_cds_ft_node_unpublished(ft, ft_node_ptr(nf));
	}
	ft_glue_fini(g);
}

/*
 * Commit step 1: wire the deferred LIVE back-pointers.  Call after the
 * source unlink + grace period, before the forward publish of the
 * cluster top, so an up-walk from any re-parented live node enters the
 * new cluster before the old nodes are forward-detached and freed.
 *
 * Only edges whose CHILD is a live (observable) node are deferred --
 * setting a live node's parent is a publication-visible mutation that
 * must wait for sync_rcu.  Fresh-to-fresh edges inside the cluster are
 * set IMMEDIATELY during the build (the child is unpublished, the store
 * has no reader-visible effect, and by commit time the entire internal
 * chain from any live re-parent target up to publish_parent is already
 * in place).  Iteration order here therefore does not matter: each live
 * back-pointer flip lands on an already-fully-wired cluster.
 */
static
void ft_glue_apply_deferred(struct cds_ft *ft, struct ft_glue *g)
{
	int i;

	/*
	 * Apply only src-origin edges (dst_origin == false).  graft and
	 * graft_swap record every edge as src-origin (the default), so this
	 * wires all of theirs.  cds_ft_merge_at additionally records
	 * dst-origin edges, which it does NOT re-parent here: a dst child
	 * stays reachable via the old dst spine until the forward publish, so
	 * its back-pointer is switched atomically (with the merge-point
	 * forward slot) by the flip-latch, after this call.
	 */
	for (i = 0; i < g->nr_deferred; i++)
		if (!g->deferred[i].dst_origin)
			ft_set_parent(ft, g->deferred[i].child, g->deferred[i].parent,
				g->deferred[i].slot);
}

/*
 * Record a deferred duplicate-chain splice (cds_ft_merge_at, same full key in
 * both tries): the @src_head chain is to be appended to @dst_head's chain.
 * The @dst_head chain's forward owner and back-pointer are wired separately
 * (Phase-1 set + deferred edge), like any other re-parented external; this
 * records only the concatenation, applied at commit.
 */
#ifdef FEATURE_FT_MERGE
static
void ft_glue_record_splice(struct ft_glue *g,
		struct cds_ft_node *dst_head,
		struct cds_ft_node *src_head)
{
	assert(g->nr_splices < g->cap_splices);
	g->splices[g->nr_splices].dst_head = dst_head;
	g->splices[g->nr_splices].src_head = src_head;
	g->splices[g->nr_splices].src_cell = NULL;
	g->nr_splices++;
}

/*
 * Commit: apply the deferred duplicate-chain splices.  Call AFTER the source
 * has been detached + drained (so the appended @src_head chain has no second
 * owner traversing it from the source tree).
 *
 * Per splice: append the whole src chain to dst's tail, prev-before-next (the
 * ft_chain_node idiom, but preserving src_head->next so the rest of the src
 * chain rides along).  @dst_head stays the head, so in-flight dst snapshots
 * keep their ordering.
 */
static
void ft_glue_apply_splices(struct cds_ft *ft __attribute__((unused)),
		struct ft_glue *g)
{
	int i;

	for (i = 0; i < g->nr_splices; i++) {
		struct cds_ft_node *dst_head = g->splices[i].dst_head;
		struct cds_ft_node *src_head = g->splices[i].src_head;
		struct cds_ft_node *tail = dst_head;
		/*
		 * Ordered list on: @src_head was a head in src (prev is its cell);
		 * it becomes a non-head duplicate of @dst_head, so its cell leaves
		 * the trie.  The cell is NOT unreachable yet: on the merge spine
		 * path this runs after the structural flip, and the surviving src
		 * heads' cells -- already reachable in dst -- still carry the old
		 * src-run ord_prev/ord_next, including links to THIS cell, until
		 * the post-publish interleave rewires them.  Capture it in the
		 * splice record; ft_glue_free_collided_cells frees it after
		 * the interleave through the grace-period-deferred cell free.
		 * List off: src_head->prev is the flagged parent, no cell.
		 */
		g->splices[i].src_cell = ft->ordered_list ?
			ft_ord_cell_ptr(src_head->prev) : NULL;

		while (ft_node_next(tail))
			tail = ft_node_next(tail);
		src_head->prev = tail;	/* write-side only, plain store */
		rcu_assign_pointer(tail->next, src_head);
	}
}

/*
 * Free the collided (demoted) src heads' cells.  Call AFTER the ordered-list
 * interleave: only then has every surviving cell's stale src-run link been
 * rewired away from these cells, making them unreachable to NEW readers; the
 * grace-period defer inside ft_ord_cell_free then covers readers already
 * holding a stale link or parked on a demoted head.
 */
static
void ft_glue_free_collided_cells(struct cds_ft *ft,
		struct ft_glue *g)
{
	int i;

	for (i = 0; i < g->nr_splices; i++)
		if (g->splices[i].src_cell)
			ft_ord_cell_free(ft, g->splices[i].src_cell);
}
#endif /* FEATURE_FT_MERGE */

/*
 * Commit step 2: the single forward publish that makes the whole cluster
 * reachable in dst.  Call after ft_glue_apply_deferred.  The
 * cluster top's parent is wired into publish_parent at set_publish time
 * (build phase, fresh-child store) and the rest of the cluster's
 * internal back-pointers are also already set, so by the time we publish
 * every back-pointer needed for an up-walk from any re-parented live
 * node up through the cluster to publish_parent is in place.
 */
static
void ft_glue_publish(struct cds_ft *ft, struct ft_glue *g)
{
	ft_publish_to_parent(ft, g->publish_parent, g->publish_slot, g->top);
}

/*
 * GLUE-path graft publish FUSED with an ordered-list run-splice -- the
 * appear-side dual of ft_remove_commit_rec.  When @run is set, RECORD the
 * cluster's forward publish edge (plus a compressed parent's SKIP_X dual) via a
 * ft_pub_rec instead of storing it, append the run's <=4 splice boundary edges,
 * and commit them all in ONE ft_ord_cell_flip, so a reader never sees the
 * grafted run reachable in the structure but absent from the ordered list (or
 * vice versa).  @run->armed is set so the caller skips the standalone splice.
 * @run NULL (ordered list off) falls back to the plain forward publish.
 */
static
void ft_glue_publish_run(struct cds_ft *ft, struct ft_glue *g,
		struct ft_graft_run *run)
{
	struct ft_pub_rec rec = { .n = 0 };

	if (!run) {
		ft_glue_publish(ft, g);
		return;
	}
	_ft_publish_to_parent(ft, g->publish_parent, g->publish_slot, g->top,
		&rec);
	ft_ord_cell_flip_rec_run(ft, &rec, run);
}

/*
 * Commit step 3: reclaim the old (replaced) live nodes, deferred via the
 * normal grace-period free.  Call after the forward publish.
 */
static
void ft_glue_free_old(struct cds_ft *ft, struct ft_glue *g)
{
	int i;

	for (i = 0; i < g->nr_free; i++) {
		if (g->free_list[i].compressed)
			free_compressed_node(ft, g->free_list[i].node);
		else
			free_cds_ft_node(ft, g->free_list[i].node);
	}
}
