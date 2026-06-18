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
static
void ft_ord_cell_run_detach(struct cds_ft *ft, struct cds_ft *into,
		struct cds_ft_node *first_head, struct cds_ft_node *last_head)
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
	/* @into is exclusive: no readers, plain stores. */
	first->ord_prev = NULL;
	last->ord_next = NULL;
	into->ord_cell_head = first;
	into->ord_cell_tail = last;
}

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
		const uint8_t *key, size_t key_len, struct ft_ord_cell *cell)
{
	struct cds_ft_iter *it = ft->ord_cell_scratch_iter;
	struct cds_ft_node *pred_head;

	/*
	 * Write the search key into iter_key (set_key clears cache_valid/node and
	 * sets key_len + key_off=0, path_len=0), then seed a live cursor AT the new
	 * head on top: cache_valid + node + path_len==key_depth drive the
	 * cross-call node-recovery fast path; prefix 0 = unscoped; ord_cell_node
	 * cleared so a fall-back cell cursor re-resolves from node->prev.
	 */
	it->node = NULL;
	if (cds_ft_iter_set_key(it, key, key_len) != CDS_FT_STATUS_OK)
		return NULL;
	it->node = cell->node;
	it->cache_valid = true;
	it->ord_cell_node = NULL;
	it->prefix_len = 0;
	it->path_len = it->key_len + 1;
	if (cds_ft_lookup_inequality_impl(ft, it, FT_LOOKUP_LT,
			FT_LOOKUP_LIMIT_NONE, false, true) != CDS_FT_STATUS_OK)
		return NULL;
	pred_head = cds_ft_iter_node(it);
	if (!pred_head)
		return NULL;
	return ft_ord_cell_ptr(rcu_dereference(pred_head->prev));
}

/*
 * Locate @cell's splice neighbours from its (parent-chain-wired) head and
 * pre-set the cell's own links -- invisible until the neighbour edges flip.
 * Shared by the legacy post-publish splice, the one-commit park and the
 * B-lite (external_nodes shape) pre-publish fill.
 */
static
void ft_ord_cell_prefill(struct cds_ft *ft, const uint8_t *key, size_t key_len,
		struct ft_ord_cell *cell, struct ft_ord_cell **pred_out,
		struct ft_ord_cell **succ_out)
{
	struct ft_ord_cell *pred, *succ;

	pred = ft_ord_cell_find_pred_from_head(ft, key, key_len, cell);
	if (pred)
		succ = ft_ord_cell_resolve_ord(&pred->ord_next);
	else
		/* New minimum: successor is the old list head (O(1), no descent). */
		succ = ft_ord_cell_resolve_ord(&ft->ord_cell_head);
	/* Pre-set @cell's own links; not yet reachable via the list. */
	cell->ord_prev = pred;
	cell->ord_next = succ;
	*pred_out = pred;
	*succ_out = succ;
}

/* Build the <= 4 visible neighbour edges for splicing @cell between
 * @pred / @succ.  Returns the edge count. */
static
unsigned int ft_ord_cell_splice_edges(struct cds_ft *ft,
		struct ft_ord_cell *cell, struct ft_ord_cell *pred,
		struct ft_ord_cell *succ, struct ft_ord_cell_edge *edges)
{
	unsigned int n = 0;

	if (pred) {
		edges[n].slot = &pred->ord_next;
		edges[n].old_target = succ;
		edges[n].new_target = cell;
		n++;
	}
	if (succ) {
		edges[n].slot = &succ->ord_prev;
		edges[n].old_target = pred;
		edges[n].new_target = cell;
		n++;
	}
	/* New min (!pred) => head was @succ; new max (!succ) => tail was @pred. */
	n = ft_ord_cell_endpoint_edge(&ft->ord_cell_head, succ, cell, edges, n);
	n = ft_ord_cell_endpoint_edge(&ft->ord_cell_tail, pred, cell, edges, n);
	return n;
}

/*
 * As ft_ord_cell_prefill, but locates the neighbours by a relational descent
 * on the key (the new key is still absent).  For the shapes whose fresh head
 * attaches as a holder's external_nodes (prefix / NIL keys): the from-head
 * seed needs the holder relation, which is only wired by the attach itself.
 */
static
void ft_ord_cell_prefill_by_key(struct cds_ft *ft, const uint8_t *key,
		size_t key_len, struct ft_ord_cell *cell,
		struct ft_ord_cell **pred_out, struct ft_ord_cell **succ_out)
{
	struct ft_ord_cell *pred, *succ;

	pred = ft_ord_cell_find_rel(ft, key, key_len, FT_LOOKUP_LT);
	if (pred)
		succ = ft_ord_cell_resolve_ord(&pred->ord_next);
	else
		succ = ft_ord_cell_resolve_ord(&ft->ord_cell_head);
	cell->ord_prev = pred;
	cell->ord_next = succ;
	*pred_out = pred;
	*succ_out = succ;
}

static
void ft_ord_cell_splice_at(struct cds_ft *ft, struct ft_ord_cell *cell,
		struct ft_ord_cell *pred, struct ft_ord_cell *succ)
{
	struct ft_ord_cell_edge edges[4];
	unsigned int n;

	n = ft_ord_cell_splice_edges(ft, cell, pred, succ, edges);
	ft_ord_cell_flip(ft, edges, n);
}

static
void ft_ord_cell_splice(struct cds_ft *ft, const uint8_t *key, size_t key_len,
		struct ft_ord_cell *cell)
{
	struct ft_ord_cell *pred, *succ;

	ft_ord_cell_prefill(ft, key, key_len, cell, &pred, &succ);
	ft_ord_cell_splice_at(ft, cell, pred, succ);
}

/* Remove @cell from the ordered cell list (its key disappeared). */
static
void ft_ord_cell_unsplice(struct cds_ft *ft, struct ft_ord_cell *cell)
{
	struct ft_ord_cell *pred = ft_ord_cell_resolve_ord(&cell->ord_prev);
	struct ft_ord_cell *succ = ft_ord_cell_resolve_ord(&cell->ord_next);
	struct ft_ord_cell_edge edges[4];
	unsigned int n = 0;

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
	/* @cell keeps its links for parked readers until its deferred free. */
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
	unsigned int n = 0;

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
	ft_ord_cell_flip(dst, edges, n);
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
