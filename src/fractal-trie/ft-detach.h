// SPDX-FileCopyrightText: 2012-2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-only

/*
 * src/fractal-trie/ft-detach.h
 *
 * Userspace RCU library - Fractal Trie: detach a sub-trie into a transient trie.
 *
 * Implementation unit: #included once by fractal-trie.c, in dependency
 * order, into a single translation unit (preserves cross-module inlining).
 * Not a standalone header.
 */
#ifndef FRACTAL_TRIE_IMPL
#error "ft-detach.h is an implementation unit; #include it from fractal-trie.c only"
#endif

enum cds_ft_status cds_ft_detach(struct cds_ft *ft,
		const uint8_t *_key, size_t _key_len,
		struct cds_ft **result_ft)
{
	size_t key_len;
	enum cds_ft_status status;

	FT_TP_KEY(detach_enter, ft, _key, _key_len);

	*result_ft = NULL;

	if (!ft) {
		FT_TP(detach_exit, (int) CDS_FT_STATUS_INVALID_ARGUMENT_ERROR);
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}

	/*
	 * Root-level detach (key_len == 0) is valid for both
	 * variable-length and fixed-length groups.  See cds_ft_graft.
	 */
	if (_key_len == 0) {
		key_len = 0;
	} else {
		key_len = ft_key_len(ft, _key_len);
		if (!valid_key_len(ft, key_len) ||
				ft->group->key_len != CDS_FT_LEN_VARIABLE) {
			FT_TP(detach_exit, (int) CDS_FT_STATUS_INVALID_ARGUMENT_ERROR);
			return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
		}
	}

	status = ft_detach_keylen(ft, _key, key_len, result_ft);
	FT_TP(detach_exit, (int) status);
	return status;
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

/*
 * One-commit insert tail (see struct ft_insert_commit): the structural slot
 * already holds a parked flip proxy (the fresh head is invisible -- the proxy
 * resolves to the old slot value), and the head's parent chain is fully wired,
 * so the splice-position search runs exactly as the post-publish splice did
 * (the from-head seed walks the parent chain, never the parked slot).  Park
 * the <= 4 ordered-list neighbour edges into the SAME batch, commit once --
 * the head becomes reachable in the structural index AND spliced into the
 * cell list atomically for every reader -- then settle all slots to their
 * direct values and finalize the real top's parent bookkeeping (skip_slot,
 * incoming_byte).
 */
static
void ft_insert_one_commit(struct cds_ft *ft, const uint8_t *key,
		size_t key_len, struct ft_ord_cell *cell,
		struct ft_insert_commit *ic)
{
	struct ft_ord_cell *pred, *succ;
	struct ft_ord_cell_edge edges[4];
	unsigned int i, n = 0;

	pred = ft_ord_cell_find_pred_from_head(ft, key, key_len, cell);
	if (pred)
		succ = ft_ord_cell_resolve_ord(&pred->ord_next);
	else
		/* New minimum: successor is the old list head (O(1), no descent). */
		succ = ft_ord_cell_resolve_ord(&ft->ord_cell_head);
	/* Pre-set @cell's own links; not yet reachable via the list. */
	cell->ord_prev = pred;
	cell->ord_next = succ;
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
	/* Park the ordered-list edges (each resolves to OLD until the commit). */
	for (i = 0; i < n; i++)
		rcu_assign_pointer(*edges[i].slot,
			(struct ft_ord_cell *) ft_flip_batch_add(ic->batch,
				(struct cds_ft_inode_flag *) edges[i].old_target,
				(struct cds_ft_inode_flag *) edges[i].new_target));

	/* THE commit: structural slot + ordered-list edges, atomically. */
	urcu_flip_commit(&ic->batch->group);

	/*
	 * Settle: direct values in every parked slot (idempotent for readers,
	 * the proxies already resolve to the new targets).  The real top's
	 * full wiring (parent, slot offset, incoming_byte) was done at park
	 * time, while still invisible.  A split-shape park settles through
	 * ft_publish_to_parent for its dual skip-slot maintenance; the attach
	 * shape's set_nth already did its own bookkeeping, so a direct store
	 * of the same canonical value suffices.
	 */
	if (ic->publish_to_parent)
		ft_publish_to_parent(ft, ic->parent_nf, ic->slot,
			ic->slot_value);
	else
		rcu_assign_pointer(*ic->slot, ic->slot_value);
	for (i = 0; i < n; i++)
		rcu_assign_pointer(*edges[i].slot, edges[i].new_target);
	ft_flip_batch_reclaim(ic->batch);
	ic->batch = NULL;
	/*
	 * The old compressed node a split replaced: readers resolved the
	 * proxy to it until the commit above, so only now may its grace-
	 * period-deferred free be queued.
	 */
	if (ic->free_old_cn)
		free_compressed_node(ft, ic->free_old_cn);
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

/*
 * Interleave the surviving source cells into @dst's ordered list after a
 * cds_ft_merge_at spine-copy commit.  Walks the merged subtree at @dst_key in
 * key order (@merged_keys distinct heads) via the structural inequality oracle,
 * two-pointering against @dst's ORIGINAL region cells: @ord_cursor steps through
 * those (captured before the commit, min head of the dst merge subtree), and any
 * walked head that is NOT the cursor cell is a surviving src head -> splice it
 * after the last placed cell.  Dst-original cells are left untouched (their
 * relative order is preserved by the merge); collided src heads were demoted to
 * duplicates + their cells freed, so the walk never sees them.  @prev_placed
 * starts at the region's predecessor (@ord_cursor's ord_prev).
 *
 * Identity key_map only (the seed uses @dst_key directly; matches the rest of
 * the ordered-list machinery).  Uses @dst's writer-exclusive scratch iterator.
 */
static
void ft_merge_ord_interleave(struct cds_ft *dst, const uint8_t *dst_key,
		size_t dst_key_len, unsigned long merged_keys,
		struct ft_ord_cell *ord_cursor, struct ft_ord_cell *prev_placed,
		struct ft_ord_cell_edge *edges, struct ft_flip_batch *flip_b)
{
	struct cds_ft_iter *it = dst->ord_cell_scratch_iter;
	unsigned long i;

	/*
	 * Seed at the merge region's minimum, via the descent oracle
	 * (cache_valid = false: the cell list is mid-update, so the cell fast
	 * path must not be used).  A root merge (@dst_key_len == 0) merges the
	 * whole trie -> seed at the GLOBAL minimum with LIMIT_FIRST, which ignores
	 * the search key (a zero-length key is invalid for cds_ft_iter_set_key /
	 * a relational GE on a FIXED-length group, so the relational path would
	 * bail and leave the src run unspliced).  A scoped merge (@dst_key_len >
	 * 0) needs the first key >= @dst_key, so it sets @dst_key and uses the
	 * relational GE (LIMIT_NONE), which honors @dst_key.
	 */
	it->node = NULL;
	it->cache_valid = false;
	if (dst_key_len == 0) {
		it->key_len = 0;
		it->prefix_len = 0;
		it->key_off = 0;
		if (cds_ft_lookup_inequality_impl(dst, it, FT_LOOKUP_GE,
				FT_LOOKUP_LIMIT_FIRST, false, false) != CDS_FT_STATUS_OK)
			goto unused;
	} else {
		const uint8_t *seed_key = dst_key;
		size_t seed_len = dst_key_len;
		size_t flen = dst->group->key_len;
		uint8_t padbuf[FT_MAX_KEY_LEN];

		/*
		 * FIXED-length group with a mid-key merge point: @dst_key is
		 * shorter than the group's key length, which the relational GE
		 * rejects -- pad it to the fixed length with the ordinal-space
		 * minimum so GE finds the merged region's first key (every key
		 * in the region extends @dst_key, hence sorts at or above the
		 * padded probe; everything below the region sorts under it).
		 * @dst_key is ALREADY ORDINAL here (cds_ft_merge_at converts
		 * once at entry), so pad with raw 0x00 and set the iterator key
		 * without the public set_key's key-map application.
		 */
		if (flen != CDS_FT_LEN_VARIABLE && dst_key_len != flen) {
			assert(dst_key_len < flen && flen <= FT_MAX_KEY_LEN);
			memcpy(padbuf, dst_key, dst_key_len);
			memset(padbuf + dst_key_len, 0x00,
				flen - dst_key_len);
			seed_key = padbuf;
			seed_len = flen;
		}
		ft_iter_set_key_ordinals(it, seed_key, seed_len);
		it->prefix_len = 0;
		it->node = NULL;
		it->cache_valid = false;
		if (cds_ft_lookup_inequality_impl(dst, it, FT_LOOKUP_GE,
				FT_LOOKUP_LIMIT_NONE, false, false) != CDS_FT_STATUS_OK)
			goto unused;
	}
	/*
	 * Single-flip re-weave.  Walk the merged region in key order; pre-set
	 * each surviving src cell's links with plain stores (the cell is not yet
	 * ord-reachable in @dst, so this is invisible) and accumulate ONLY the
	 * VISIBLE boundary edges -- a dst-original cell's ord_next / ord_prev, or
	 * @dst's head / tail -- into one batch.  A single flip then commits the
	 * entire interleave atomically, so a concurrent ordered reader never
	 * observes a partially re-woven list (the per-splice path was N
	 * independent flips).  Dst-original cells keep their relative order, so a
	 * dst<->dst step needs no edge; each survivor RUN costs at most two edges
	 * (one entering, one leaving), so 2 * merged_keys + 2 bounds the batch.
	 * @edges and @flip_b are pre-allocated by the caller in the merge's
	 * fallible BUILD phase (this runs in the failure-free commit tail, where
	 * an alloc failure would force a non-atomic fallback).
	 */
	{
		struct ft_ord_cell *prev = prev_placed;
		bool prev_is_dst = (prev_placed != NULL);
		unsigned int n = 0;

		for (i = 0; i < merged_keys; i++) {
			struct cds_ft_node *head = cds_ft_iter_node(it);
			struct ft_ord_cell *cell;

			if (!head)
				break;
			cell = ft_ord_cell_ptr(rcu_dereference(head->prev));
			if (cell == ord_cursor) {
				/*
				 * Dst-original cell: stays put, already linked in
				 * key order.  Its back edge changes only when a
				 * survivor run was just placed before it.
				 */
				if (prev && !prev_is_dst) {
					prev->ord_next = cell;	/* survivor: invisible */
					edges[n].slot = &cell->ord_prev;
					edges[n].old_target =
						ft_ord_cell_resolve_ord(&cell->ord_prev);
					edges[n].new_target = prev;
					n++;
				}
				prev = cell;
				prev_is_dst = true;
				ord_cursor = ft_ord_cell_resolve_ord(
					&ord_cursor->ord_next);
			} else {
				/* Surviving src cell: pre-set its back link. */
				cell->ord_prev = prev;	/* invisible */
				if (!prev) {
					/* new list minimum: flip @dst head. */
					edges[n].slot = &dst->ord_cell_head;
					edges[n].old_target =
						ft_ord_cell_resolve_ord(&dst->ord_cell_head);
					edges[n].new_target = cell;
					n++;
				} else if (prev_is_dst) {
					/* dst -> survivor: flip the dst cell's fwd edge. */
					edges[n].slot = &prev->ord_next;
					edges[n].old_target =
						ft_ord_cell_resolve_ord(&prev->ord_next);
					edges[n].new_target = cell;
					n++;
				} else {
					prev->ord_next = cell;	/* survivor: invisible */
				}
				prev = cell;
				prev_is_dst = false;
			}
			it->cache_valid = false;	/* force the descent oracle */
			if (cds_ft_lookup_inequality_impl(dst, it, FT_LOOKUP_GT,
					FT_LOOKUP_LIMIT_NONE, false, false) != CDS_FT_STATUS_OK)
				break;
		}
		/*
		 * Close the trailing edge: if the last placed cell is a survivor,
		 * link it to the region successor (@ord_cursor, advanced past the
		 * last dst-original; NULL at the list tail) and flip that
		 * neighbour's back edge -- or @dst's tail when there is none.
		 */
		if (prev && !prev_is_dst) {
			prev->ord_next = ord_cursor;	/* survivor: invisible */
			if (!ord_cursor) {
				edges[n].slot = &dst->ord_cell_tail;
				edges[n].old_target =
					ft_ord_cell_resolve_ord(&dst->ord_cell_tail);
				edges[n].new_target = prev;
				n++;
			} else {
				edges[n].slot = &ord_cursor->ord_prev;
				edges[n].old_target =
					ft_ord_cell_resolve_ord(&ord_cursor->ord_prev);
				edges[n].new_target = prev;
				n++;
			}
		}
		ft_ord_cell_flip_prealloc(dst, edges, n, flip_b);
	}
	return;
unused:
	/* Seed failed (empty region): the pre-allocated batch goes unused. */
	ft_flip_batch_free_unpublished(flip_b);
}

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
 * Unlink the EXACT subtree at @src_key from @src_ft IN PLACE, preserving the
 * subtree node so the spine-copy merge can keep referencing it (it is
 * re-parented into the merged cluster at commit).  This is the non-root-src
 * analogue of the root-src ft->root swap: it removes the merge source from
 * @src_ft and prunes the now-empty single-child branch above it.
 *
 * The descent + ft_detach_node + chain reclaim mirror ft_detach_keylen's
 * non-root path, but with @free_detached_subtree = false and WITHOUT wrapping
 * the subtree in a transient trie (the merge owns it via @gd's referenced
 * edges; wrapping it would double-own it).  @detached_count is the subtree's
 * unique-key count (from ft_merge_descend), propagated out of the ancestors.
 *
 * The caller invokes this as the LAST fallible commit step: on -ENOMEM
 * (ft_detach_node recompaction failed) @src_ft is left pristine, so the caller
 * aborts the still-invisible build with both tries intact -- no rollback.  On
 * success the caller drains @src_ft and runs the failure-free commit tail.
 */
static
int ft_merge_unlink_src_subtree(struct cds_ft *src_ft,
		const uint8_t *_src_key, size_t src_key_len,
		unsigned long detached_count)
{
	/*
	 * @src_key is ALREADY ORDINAL (cds_ft_merge_at converts once at its
	 * entry); a second key-map application here would descend a different
	 * subtree than the one the spine build copied.
	 */
	const uint8_t *key = _src_key, *ik;
	struct ft_descent d;
	int ret;

	ik = key;

	/*
	 * Plain key-guided descent to the merge source's subtree root.  As in
	 * ft_detach_keylen, no branch-point snapshot is tracked: ft_detach_node
	 * is bootstrapped from the target's own slot and climbs parent pointers
	 * to the surviving ancestor.  A KEY_SHORTER source (the key ends inside
	 * a compressed node) overshoots that node -- @d.nf becomes its child --
	 * and the climb's free walk reclaims the whole compressed node while
	 * preserving @d.nf, matching the old explicit chain reclaim.
	 */
	ft_descent_init(&d, src_ft);
	for (; d.depth < src_key_len; ) {
		uint8_t kv;

		/* Caller already established EXACT, so the path is present. */
		if (ft_node_compressed(d.nf)) {
			struct cds_ft_compressed_node *cn =
				ft_compressed_node_ptr(d.nf);

			ft_descent_traverse_compressed(&d, cn, &ik);
			continue;
		}
		kv = *(ik++);
		ft_descent_step(src_ft, &d, kv);
	}

	/* Propagate the removal through ancestors before touching freed slots. */
	ft_propagate_external_count_parent(src_ft, d.pnf,
			-(long) detached_count);

	/*
	 * Unlink the branch in place, preserving the move target (@d.nf, the
	 * subtree root the spine-copy merge keeps referencing).  Bootstrapped
	 * from the target's own slot, ft_detach_node climbs to the surviving
	 * ancestor and its free-walk phase 1 reclaims the intermediate single-
	 * child chain (compressed / skip-target nodes included) while phase 2 --
	 * which would free the target -- stays gated off for move-style.
	 */
	ret = ft_detach_node(src_ft, d.nfp, d.pnfp, d.depth,
			/*free_detached_subtree=*/ false);
	if (ret < 0) {
		/* Recompaction OOM: undo the propagation; src is pristine. */
		ft_propagate_external_count_parent(src_ft, d.pnf,
				(long) detached_count);
		return -ENOMEM;
	}
	return 0;
}

/*
 * KEY_SHORTER dst: the merge point sits @off bytes inside a compressed dst
 * node whose prefix bytes @key[@d_depth .. @key_len) lie ABOVE it.  Wrap the
 * freshly-built merged node @M (with @mk unique keys) under that prefix and
 * return the PLAIN flag to publish into the old compressed node's forward slot
 * (the caller skip-encodes it), or FT_MERGE_OOM.
 *
 * @M internal: ft_build_branch lays a compressed prefix over the fresh
 * internal M -- M's edge into the prefix applies immediately via defer_edge's
 * is_fresh fast path, so ft_build_branch's dst_origin=false leaf edge is
 * correct, and M's own children keep the dst_origin ft_merge_build gave them.
 * A 1-child-no-external M is first canonicalized to compressed (its lone child
 * is always a FRESH recursion result, so the fold is dst_origin-safe too).
 *
 * @M compressed: M is a fresh run produced by ft_merge_build_run (the sole
 * path returning a compressed top), whose child may be a LIVE dst splice head
 * carrying dst_origin=true (a duplicate-key merge through the compressed node).
 * ft_build_branch's chain-merge would re-defer that child as dst_origin=false
 * and demote a live re-parent out of the flip latch, so fuse the prefix into M
 * here by hand, carrying the run child's dst_origin (always true).  The fused
 * length @off + M->len <= cn_d->len, so it is a valid compressed length.
 */
static
struct cds_ft_inode_flag *ft_merge_wrap_prefix(struct ft_merge_ctx *c,
		const uint8_t *key, unsigned int d_depth, unsigned int key_len,
		struct cds_ft_inode_flag *M, unsigned long mk)
{
	struct cds_ft *ft = c->dst_ft;
	unsigned int prefix_len = key_len - d_depth;

	if (ft_node_compressed(M)) {
		struct cds_ft_compressed_node *mcn = ft_compressed_node_ptr(M);
		unsigned int merged_len = prefix_len + mcn->len;
		struct cds_ft_compressed_node *merged;
		struct cds_ft_metadata *merged_meta;
		struct cds_ft_inode_flag *mflag;

		merged = alloc_compressed_node(ft, (uint8_t) merged_len,
				&merged_meta);
		if (!merged)
			return FT_MERGE_OOM;
		memcpy(merged->key_bytes, &key[d_depth], prefix_len);
		memcpy(&merged->key_bytes[prefix_len], mcn->key_bytes, mcn->len);
		merged->len = (uint8_t) merged_len;
		merged->child = mcn->child;
		merged_meta->nr_child = 1;
		ft_nr_keys_store(merged_meta, mk, CMM_RELAXED);
		mflag = ft_compressed_node_flag(merged);
		ft_graft_glue_track(c->gd, mflag);
		/*
		 * Carry the run child's dst_origin (ft_merge_build_run records
		 * it true): a fresh child applies via is_fresh, a live splice
		 * head flips with the forward slot.  The defer de-dup supersedes
		 * M's stale &mcn->child edge with this one (same @child).
		 */
		ft_graft_glue_defer_edge_origin(ft, c->gd, mcn->child, mflag,
				&merged->child, /*dst_origin=*/ true);
		ft_graft_glue_untrack(ft, c->gd, mcn);
		free_compressed_node_unpublished(ft, mcn);
		return mflag;	/* PLAIN; caller skip-encodes before publish */
	} else {
		struct cds_ft_inode_flag *canon, *top;

		canon = ft_compress_single_child_if_needed(ft, M, c->gd);
		if (canon == (struct cds_ft_inode_flag *) (long) -ENOMEM)
			return FT_MERGE_OOM;
		top = ft_build_branch(ft, key, d_depth, key_len, canon, mk,
				/*has_external_nodes=*/ false, c->gd);
		if (!top)
			return FT_MERGE_OOM;
		return top;	/* PLAIN compressed; caller skip-encodes */
	}
}

/*
 * Keys in ORDINAL form (cds_ft_merge_at converts the caller's application
 * keys once at its entry; every consumer below -- the spine build, the source
 * unlink, the ordered-list interleave -- expects ordinal bytes).
 *
 * Build-invisible spine-copy merge of @src_ft's subtree at @src_key into
 * @dst_ft at the live EXACT subtree @d_dst.  All allocation is in the build
 * phase (plus the single fallible src unlink at commit for a non-root src), so
 * any OOM frees the fresh copies and leaves both tries pristine -- no rollback.
 *
 * @src_ft merged at @src_key (@d_src its subtree, @cnt_src its key count) into
 * @d_dst, a non-empty dst subtree.  A root src (src_key_len == 0) unlinks via
 * the ft->root swap; a non-root src unlinks its branch in place at commit (the
 * only post-build fallible step).  @off_src selects the src merge-point shape:
 * 0 is an EXACT subtree at @d_src->nf; > 0 is KEY_SHORTER (the src key ends
 * @off_src bytes inside the compressed node @d_src->nf -- the build enters that
 * node at the cursor, and ft_merge_unlink_src_subtree reclaims the whole node
 * via its chain-reclaim, the move-style detach preserving cn_s->child).  Both
 * root and non-root dst merge points are handled: a non-root point flips the
 * interior forward slot through a type-7 proxy that the read-side descent
 * resolves at every child fetch.  Compressed overlaps are handled.  @off_dst
 * selects the dst merge-point shape: 0 is an EXACT subtree at @d_dst->nf; > 0
 * is KEY_SHORTER (the dst key ends @off_dst bytes inside the compressed node
 * @d_dst->nf, whose prefix bytes are wrapped around the merged cluster by
 * ft_merge_wrap_prefix).
 *
 * Every (EXACT | KEY_SHORTER) src x (EXACT | KEY_SHORTER) dst shape is handled.
 *
 * Returns OK on a committed merge, or MEMORY_ERROR on OOM (both tries pristine).
 */
static
enum cds_ft_status ft_merge_spine_copy(struct cds_ft *dst_ft,
		struct cds_ft *src_ft, struct ft_descent *d_src,
		const uint8_t *src_key, size_t src_key_len, unsigned long cnt_src,
		unsigned int off_src, struct ft_descent *d_dst,
		unsigned long cnt_dst, unsigned int off_dst,
		const uint8_t *dst_key, size_t dst_key_len,
		struct ft_flip_batch **pre_flip,
		struct ft_flip_batch **pre_ms_flip)
{
	struct ft_graft_glue gd, gs;
	struct ft_merge_ctx ctx = { .dst_ft = dst_ft, .gd = &gd, .gs = &gs };
	struct ft_merge_counts cnt = { 0, 0, 0, 0, 0 };
	bool root_src = (src_key_len == 0);
	bool ks_dst = (off_dst > 0);
	bool ed;
	struct cds_ft_inode_flag *S = d_src->nf;
	struct cds_ft_inode_flag *D = d_dst->nf;
	struct cds_ft_inode_flag *M, *M_slot = NULL, *pub, *D_old;
	struct cds_ft_inode_flag *pub_parent = d_dst->pnf, **pub_slot = d_dst->nfp;
	struct cds_ft_inode *fresh_root = NULL;
	struct cds_ft_metadata *fresh_meta;
	struct ft_flip_batch *flip;
	unsigned long merged_keys = 0;
	bool ms_ord = dst_ft->group->ordered_list_set;
	struct ft_ord_cell *ms_cursor = NULL, *ms_prev = NULL;
	struct cds_ft_node *ms_s_first = NULL, *ms_s_last = NULL;
	struct ft_ord_cell_edge *ms_edges = NULL;
	struct ft_flip_batch *ms_flip = NULL;

	/*
	 * Every dst merge-point shape is handled.  The flip proxies the publish
	 * slot @pub_slot, the read-side descent resolves the type-7 proxy at every
	 * child fetch (ft_resolve_flip_proxy, before the skip handler), and the
	 * published node's parent is wired to @pub_parent by
	 * ft_graft_glue_set_publish, so a descent and an up-walk see a coherent
	 * old-XOR-merged view across the flip.
	 *
	 * Edge D: the merge point's PARENT is a COMPRESSED node cn_p reached via a
	 * grandparent skip slot.  cn_p's own parent cannot be compressed ("no two
	 * adjacent compressed" invariant), so the grandparent slot d_dst->pnfp is a
	 * plain internal slot that merely *holds* skip(cn_p).  Handle it one level
	 * up: the merge is EXACT at d_dst->nf (off_dst == 0), M is wrapped under a
	 * fresh copy of the WHOLE cn_p, and that copy is published into d_dst->pnfp
	 * in place of skip(cn_p) -- identical machinery to a KEY_SHORTER dst wrap,
	 * just with cn_p as the wrapped node and the grandparent as the publish
	 * point.  ks_dst and ed are mutually exclusive (a KEY_SHORTER merge point
	 * sits inside cn_d, whose parent is internal).
	 */
	ed = (d_dst->pnf &&
	      ft_node_compressed(ft_resolve_skip_compressed(dst_ft, d_dst->pnf)));

	/* Size both glues from a read-only pre-pass (with headroom). */
	ft_merge_count(dst_ft, S, off_src, D, off_dst, &cnt);
	ft_graft_glue_init(&gd);
	ft_graft_glue_init(&gs);
	if (ft_graft_glue_reserve(&gd, cnt.nb + 8, cnt.nd + 8,
				cnt.nf_dst + 8, cnt.ns + 8) ||
	    ft_graft_glue_reserve(&gs, 0, 0, cnt.nf_src + 8, 0)) {
		ft_graft_glue_fini(&gd);
		ft_graft_glue_fini(&gs);
		return CDS_FT_STATUS_MEMORY_ERROR;
	}

	/* Root src: pre-allocate the fresh empty root for the commit swap. */
	if (root_src) {
		fresh_root = alloc_cds_ft_node(src_ft, &ft_types[0], &fresh_meta);
		if (!fresh_root) {
			ft_graft_glue_fini(&gd);
			ft_graft_glue_fini(&gs);
			return CDS_FT_STATUS_MEMORY_ERROR;
		}
		rcu_assign_pointer(fresh_meta->parent, NULL);
		ft_nr_keys_store(fresh_meta, 0, CMM_RELAXED);
	}

	/* Build the merged cluster invisibly (the only build-phase fallible step). */
	M = ft_merge_build(&ctx, S, off_src, D, off_dst, 0, &merged_keys);
	if (M == FT_MERGE_OOM) {
		if (fresh_root)
			free_cds_ft_node_unpublished(src_ft, fresh_root);
		ft_graft_glue_abort(dst_ft, &gd);
		ft_graft_glue_abort(src_ft, &gs);
		return CDS_FT_STATUS_MEMORY_ERROR;
	}

	/*
	 * Compute @pub (the value to publish), @pub_parent / @pub_slot (where) and,
	 * for the compressed shapes, reclaim the replaced node.
	 *
	 * KEY_SHORTER dst (off_dst > 0): the merge point sits off_dst bytes inside
	 * the compressed node cn_d = D, whose prefix bytes lie above it; wrap M
	 * under cn_d->key_bytes[0..off_dst) and replace cn_d at its own slot
	 * (d_dst->nfp).  Edge D (ed): the merge point's parent is the compressed
	 * node cn_p; wrap M under the WHOLE cn_p and replace cn_p at the grandparent
	 * slot d_dst->pnfp (pub_parent = d_dst->ppnf).  Either way reclaim the
	 * replaced compressed node -- ft_merge_build entered it (or, for Edge D, did
	 * not touch it) without recording the free.  ft_merge_wrap_prefix
	 * canonicalizes a single-child M and preserves the dst_origin of a
	 * compressed M's child.
	 *
	 * EXACT dst with an internal/absent parent (off_dst == 0, !ed): M itself
	 * replaces the subtree.  Canonicalize a non-root single-child internal merge
	 * top: when S and D contribute exactly one shared byte (a churned "ab"-prefix
	 * shape), ft_merge_build returns M as a 1-child internal with no
	 * external_nodes -- forbidden at a non-root position under skip mode
	 * (chain-compress invariant; cds_ft_verify catches it at the merge depth).
	 * Collapse it to a 1-byte compressed via the glue.  Only the TOP M can hit
	 * this; its lone child is always a FRESH recursion result, so the deferred
	 * child edge carries dst_origin=false correctly and disturbs no flip edge.
	 * A root dst merge point (d_dst->pnf == NULL) is exempt: a 1-child internal
	 * is canonical at the root.
	 */
	if (ks_dst || ed) {
		struct cds_ft_compressed_node *wrap_cn;
		uint8_t kbuf[FT_MAX_KEY_LEN];
		unsigned int wrap_depth, wrap_len;

		if (ed) {
			wrap_cn = ft_compressed_node_ptr(
				ft_resolve_skip_compressed(dst_ft, d_dst->pnf));
			wrap_len = wrap_cn->len;
			wrap_depth = (unsigned int) d_dst->depth - wrap_len;
			pub_parent = d_dst->ppnf;
			pub_slot = d_dst->pnfp;
		} else {	/* ks_dst */
			wrap_cn = ft_compressed_node_ptr(D);
			wrap_len = off_dst;
			wrap_depth = (unsigned int) d_dst->depth;
		}
		ft_graft_glue_defer_free(&gd, wrap_cn, true);
		memcpy(&kbuf[wrap_depth], wrap_cn->key_bytes, wrap_len);
		pub = ft_merge_wrap_prefix(&ctx, kbuf, wrap_depth,
				wrap_depth + wrap_len, M, merged_keys);
		if (pub == FT_MERGE_OOM) {
			if (fresh_root)
				free_cds_ft_node_unpublished(src_ft, fresh_root);
			ft_graft_glue_abort(dst_ft, &gd);
			ft_graft_glue_abort(src_ft, &gs);
			return CDS_FT_STATUS_MEMORY_ERROR;
		}
	} else if (pub_parent) {
		pub = ft_compress_single_child_if_needed(dst_ft, M, &gd);
		if (pub == (struct cds_ft_inode_flag *) (long) -ENOMEM) {
			if (fresh_root)
				free_cds_ft_node_unpublished(src_ft, fresh_root);
			ft_graft_glue_abort(dst_ft, &gd);
			ft_graft_glue_abort(src_ft, &gs);
			return CDS_FT_STATUS_MEMORY_ERROR;
		}
	} else {
		pub = M;
	}

	/*
	 * Allocate the flip batch: one proxy per dst-origin re-parent edge,
	 * plus one for the merge-point forward slot.  Fallible -> abort the
	 * still-invisible build; both tries stay pristine.
	 */
	{
		unsigned int nr_dst = 0;
		int j;

		for (j = 0; j < gd.nr_deferred; j++)
			if (gd.deferred[j].dst_origin)
				nr_dst++;
		flip = ft_flip_batch_take(dst_ft, nr_dst + 1, pre_flip);
	}
	if (!flip) {
		if (fresh_root)
			free_cds_ft_node_unpublished(src_ft, fresh_root);
		ft_graft_glue_abort(dst_ft, &gd);
		ft_graft_glue_abort(src_ft, &gs);
		return CDS_FT_STATUS_MEMORY_ERROR;
	}
	ft_graft_glue_set_publish(dst_ft, &gd, pub_parent, pub_slot, pub);

	/*
	 * Slot-canonical form of @pub for the @pub_slot stores (both the flip
	 * proxy's new target and the settle): a compressed @pub is published
	 * SKIP-ENCODED, exactly as a direct slot write would store it, so a reader
	 * resolving the proxy gets a value identical in encoding to a normal slot
	 * read and runs the same skip handling.  set_publish wired @pub's parent +
	 * skip_slot via the plain flag already; an internal @pub (EXACT only -- a
	 * compressed-wrap @pub is always compressed) needs no re-encode.
	 *
	 * @D_old is the flip proxy's old target -- the value @pub_slot currently
	 * holds.  Read it from the live slot: for the compressed shapes the descent
	 * resolved d->nf / d->pnf to the PLAIN flag while the slot holds the SKIP
	 * form, and *pub_slot is untouched until the flip (the build is invisible
	 * and apply_deferred wires back-pointers, not this forward slot).
	 */
	if (ft_node_compressed(pub))
		M_slot = ft_publish_compressed(dst_ft,
				ft_compressed_node_ptr(pub), pub);
	else
		M_slot = pub;
	D_old = *pub_slot;

	/*
	 * Ordered list: capture the dst merge subtree's min head (the cursor for
	 * the post-commit interleave walk) and the region predecessor, while D is
	 * still intact (the build only copied its spine; the flip below moves its
	 * leaves into M).  The surviving src cells are spliced in after the commit.
	 */
	if (ms_ord) {
		ms_cursor = ft_ord_cell_ptr(rcu_dereference(
			ft_subtree_minmax_head(dst_ft, D, false)->prev));
		ms_prev = ft_ord_cell_resolve_ord(&ms_cursor->ord_prev);
		/*
		 * Capture src's merged-subtree (S) run endpoints now, while S is
		 * still intact, but defer the actual run-unlink until AFTER the
		 * last fallible step (ft_merge_unlink_src_subtree).  The run
		 * unlink is an externally observable ordered-list mutation; doing
		 * it here would leave src's list inconsistent if the src subtree
		 * unlink below OOMs and we roll the whole merge back.  The unlink
		 * still happens before the drain, so sync drains src ord-readers
		 * of the run too.
		 */
		ms_s_first = ft_subtree_minmax_head(dst_ft, S, false);
		ms_s_last = ft_subtree_minmax_head(dst_ft, S, true);
		/*
		 * Pre-allocate the interleave's edge batch + flip batch NOW,
		 * while the build is still abortable.  The interleave runs in
		 * the failure-free commit tail; an alloc failure there would
		 * have to degrade to a non-atomic per-edge fallback, exposing a
		 * half-re-woven list to bidirectional ordered readers under
		 * memory pressure.  Each survivor run costs at most two visible
		 * edges, so 2 * merged_keys + 2 bounds both.
		 */
		{
			unsigned int ms_cap =
				2u * (unsigned int) merged_keys + 2u;

			ms_edges = malloc((size_t) ms_cap * sizeof(*ms_edges));
			if (ms_edges)
				ms_flip = ft_flip_batch_take(dst_ft, ms_cap,
						pre_ms_flip);
			if (!ms_edges || !ms_flip) {
				free(ms_edges);
				ft_flip_batch_free_unpublished(flip);
				if (fresh_root)
					free_cds_ft_node_unpublished(src_ft,
						fresh_root);
				ft_graft_glue_abort(dst_ft, &gd);
				ft_graft_glue_abort(src_ft, &gs);
				return CDS_FT_STATUS_MEMORY_ERROR;
			}
		}
	}

	/*
	 * 1. Unlink the merge source from src.  Root src: swap in the pre-
	 *    allocated empty root.  Non-root src: detach its branch in place --
	 *    the LAST fallible step.  On its OOM both tries are pristine (the
	 *    detach self-undoes, the cluster is still unpublished), so abort the
	 *    build; this preserves the no-rollback property.  Then drain src
	 *    readers of the moved content.
	 *
	 *    ===== Everything from the drain onward is failure-free. =====
	 */
	if (root_src) {
		rcu_assign_pointer(src_ft->root, ft_node_flag(fresh_root, 0));
		FT_TP(root_publish, (const void *) src_ft, (const void *) src_ft->root);
	} else if (ft_merge_unlink_src_subtree(src_ft, src_key, src_key_len,
				cnt_src) < 0) {
		free(ms_edges);
		if (ms_flip)
			ft_flip_batch_free_unpublished(ms_flip);
		ft_flip_batch_free_unpublished(flip);
		ft_graft_glue_abort(dst_ft, &gd);
		ft_graft_glue_abort(src_ft, &gs);
		return CDS_FT_STATUS_MEMORY_ERROR;
	}
	/*
	 * Now that the last fallible step has committed, remove src's merged
	 * subtree (S) run from src's ordered list: its cells disperse to dst
	 * (survivors) or are freed (collisions).  Deferred to here so an OOM in
	 * the src unlink above leaves src's list untouched on rollback; done
	 * before the drain so sync drains src ord-readers of the run too.
	 */
	if (ms_ord)
		ft_ord_cell_run_unlink(src_ft, ms_s_first, ms_s_last);
	if (!src_ft->exclusive)
		src_ft->group->flavor->update_synchronize_rcu();

	/*
	 * 2. Re-parent the SRC-origin referenced subtrees directly: the src
	 *    drain above made them unreachable to readers, and the cluster is
	 *    not yet forward-published, so this is invisible.  dst-origin
	 *    edges are NOT applied here -- they go through the flip.
	 */
	ft_graft_glue_apply_deferred(dst_ft, &gd);

	/*
	 * 3. Stage the flip: point every dst-origin child's parent, and the
	 *    merge-point forward slot, at a proxy that still resolves to the
	 *    OLD target (selector 0).  Transparent to readers: up-walks and
	 *    the root descent still see the pre-merge dst.
	 */
	{
		int j;

		for (j = 0; j < gd.nr_deferred; j++) {
			struct cds_ft_inode_flag *child, *old_parent, *pf;

			if (!gd.deferred[j].dst_origin)
				continue;
			child = gd.deferred[j].child;
			/*
			 * Current parent = the old dst overlap node.  Resolve a
			 * skip-encoded picked child to its compressed node first:
			 * ft_get_parent_rcu reads node metadata and cannot take a
			 * skip pointer (ft_set_parent_raw below resolves itself).
			 */
			old_parent = ft_get_parent_rcu(dst_ft,
				ft_resolve_skip_compressed(dst_ft, child));
			pf = ft_flip_batch_add(flip, old_parent,
				gd.deferred[j].parent);
			ft_set_parent_raw(dst_ft, child, pf);
		}
		/* Publish slot: old dst subtree -> merged cluster. */
		rcu_assign_pointer(*pub_slot,
				ft_flip_batch_add(flip, D_old, M_slot));
	}

	/*
	 * 4. Flip: one release store switches every dst-origin parent AND the
	 *    forward slot from old to merged, atomically.  Because the forward
	 *    slot flips with the back-pointers, a reader (which descends then
	 *    walks up) only ever progresses old->merged, never regresses.
	 */
	urcu_flip_commit(&flip->group);

	/* 5. Concatenate same-key duplicate chains (dst now reachable via M). */
	ft_graft_glue_apply_splices(dst_ft, &gd);

	/*
	 * 6. Propagate the dst key-count delta through the ancestors, starting at
	 *    @pub_parent (the parent of the replaced node -- d_dst->pnf for an
	 *    EXACT / KEY_SHORTER point, the grandparent d_dst->ppnf for Edge D).
	 *    @pub is a fresh node already carrying merged_keys, so the walk must
	 *    begin one level up; @pub_parent is a plain internal flag (cn_p's
	 *    parent cannot be compressed), safe for ft_node_ptr.
	 */
	if (pub_parent && merged_keys != cnt_dst)
		ft_propagate_external_count_parent(dst_ft, pub_parent,
			(long) merged_keys - (long) cnt_dst);

	/*
	 * 7. Settle: rewrite each proxied slot to its direct merged target
	 *    (idempotent for readers -- the proxy already resolves to merged),
	 *    so the proxies become unreferenced and reclaimable.
	 */
	{
		int j;

		for (j = 0; j < gd.nr_deferred; j++)
			if (gd.deferred[j].dst_origin)
				ft_set_parent(dst_ft, gd.deferred[j].child,
					gd.deferred[j].parent,
					gd.deferred[j].slot);
		rcu_assign_pointer(*pub_slot, M_slot);
	}

	/*
	 * 8. Reclaim, all deferred: the flip batch (proxies now unreferenced)
	 *    and the old overlap spines (src-side to src, dst-side to dst).
	 *    No dst synchronize_rcu -- the flip subsumed the dst drain.
	 */
	ft_flip_batch_reclaim(flip);
	ft_graft_glue_free_old(src_ft, &gs);
	ft_graft_glue_free_old(dst_ft, &gd);

	/*
	 * 9. Ordered list: splice the surviving src cells into dst's ordered
	 * list at their merged positions.  Done AFTER the settle (step 7) so the
	 * merged structure carries direct pointers -- the interleave's GT
	 * continuation backtracks via parent pointers, which are flip proxies
	 * until settled.  Collided src cells were demoted to duplicates by
	 * apply_splices (step 5), so the merged-region walk never sees them;
	 * dst-original cells keep their links.  The edge + flip batches were
	 * pre-allocated in the build phase, so this cannot fail.
	 *
	 * 10. Only now free the collided cells: the interleave rewired the
	 * surviving cells' stale src-run links away from them, and the
	 * grace-period defer inside the free covers readers already holding
	 * such a link.
	 */
	if (ms_ord) {
		ft_merge_ord_interleave(dst_ft, dst_key, dst_key_len, merged_keys,
			ms_cursor, ms_prev, ms_edges, ms_flip);
		free(ms_edges);
	}
	ft_graft_glue_free_collided_cells(dst_ft, &gd);

	ft_graft_glue_fini(&gd);
	ft_graft_glue_fini(&gs);
	return CDS_FT_STATUS_OK;
}

/*
 * Reserve one node mirroring a freshly-built graft-cluster node @nf, so the
 * real graft (which rebuilds the same cluster) can draw it instead of
 * allocating.  Compressed node -- the compressed kind/order for its byte
 * length; internal node -- its type's order and bitmap.  Used to AUTO-LEARN a
 * GLUE diverge's node manifest from a build-and-abort learn pass, without hand-
 * coding the split's per-node arithmetic.
 */
static
int ft_merge_reserve_add_built(struct cds_ft *ft, struct cds_ft_alloc_reserve *r,
		struct cds_ft_inode_flag *nf)
{
	struct cds_ft_inode_flag *p = ft_resolve_skip_compressed(ft, nf);

	if (ft_node_compressed(p)) {
		struct cds_ft_compressed_node *cn = ft_compressed_node_ptr(p);
		enum cds_ft_alloc_kind kind = ft->group->speculative ?
			CDS_FT_ALLOC_KIND_COMPRESSED : CDS_FT_ALLOC_KIND_NODE;

		return cds_ft_alloc_reserve_add(ft, r, kind,
			ft_compressed_order(cn->len), false, 1);
	}
	{
		unsigned int ti = ft_node_type(p);

		return cds_ft_alloc_reserve_add(ft, r, CDS_FT_ALLOC_KIND_NODE,
			ft_types[ti].order, ft_types[ti].bitmap, 1);
	}
}

/*
 * Build-invisible graft of a SUB-position source subtree into a dst position
 * ABSENT at @dst_key (the diverged-dst case) WITHOUT re-rooting the payload --
 * the leak-free reorder for cds_ft_merge_at's residual shapes.  Modeled on
 * ft_graft_keylen's commit, but the source side is ft_merge_spine_copy's: the
 * payload stays @payload (= d_src->nf, referenced in place, not detached into a
 * fresh root), and ft_merge_unlink_src_subtree is the single last fallible step
 * (it self-undoes on OOM, preserving @payload).  Because the dst attach is built
 * invisibly BEFORE that unlink and published failure-free AFTER the drain,
 * nothing can strand the moved externals on an allocation failure -- no
 * rollback, hence no leak (contrast the legacy detach-then-graft fallback).
 *
 * Covers both diverged-dst shapes: a GLUE diverge (the dst key diverges inside a
 * compressed node, so ft_graft_build assembles the whole split cluster invisibly
 * -- no reserve) and a NOSPLIT point (an empty slot at @dst_key, or a built
 * branch / displaced external -- ft_store_at_graft_point grafts it post-drain
 * drawing from a pre-filled reserve so it cannot fail on the arena).  *handled is
 * always set true here, so the caller never falls through.
 *
 * @okey_dst / @okey_src are ORDINAL; @dst_key is the APPLICATION dst key (the
 * ordered-list splice-pos descent remaps it).  @payload is the source subtree to
 * move with @cnt_src unique keys: d_src->nf for an EXACT source, or, for a
 * KEY_SHORTER source, cn_s->child with @okey_dst / @dst_key / @dst_key_len
 * already EXTENDED by the residual cn_s bytes (the caller's reduction to EXACT).
 * @okey_src / @src_key_len remain the ORIGINAL source key (the unlink overshoots
 * cn_s on its own).
 */
/*
 * Reserve EXACTLY the nodes ft_store_at_graft_point will allocate for a NOSPLIT
 * in-place graft of @payload at the located point @d, so the post-drain store
 * draws them and cannot fail on an arena allocation.  @d is ft_graft_build's
 * NOSPLIT descent.  Two shapes:
 *  - at-node (@d->depth == @key_len, empty slot): the only commit node alloc is
 *    an optional grow-recompact of the attach node @d->pnf.
 *  - build-a-branch (@d->depth < @key_len): ft_build_branch's nodes, learned by
 *    a build-and-abort pass (it builds into a throwaway glue with no dst
 *    mutation), plus an optional grow when an empty slot ADDS a child (a
 *    displaced external is REPLACED at its slot, no grow).
 * The payload needs NO canonicalize allocation: ft_compress_single_child_if_
 * needed only allocates for a 1-child internal under skip mode, and a skip-mode
 * trie never holds a 1-child internal as a subtree root (it is canonicalized at
 * creation), so a re-rooted payload is never that shape.  The flip batch is a
 * malloc (not arena), so it is not reserved.  Returns 0, or -ENOMEM (learn build
 * or fill OOM; the caller drains the reserve).
 */
static
int ft_merge_nosplit_reserve(struct cds_ft *dst_ft, const uint8_t *okey_dst,
		size_t dst_key_len, struct cds_ft_inode_flag *payload,
		unsigned long cnt_src, struct ft_descent *d,
		struct cds_ft_alloc_reserve *reserve)
{
	struct cds_ft_inode_flag *displaced;
	struct ft_graft_glue lg;
	struct cds_ft_inode_flag *branch;
	bool grow = false;
	int rret = 0, bi;

	if (d->depth == dst_key_len && !d->nf) {
		grow = true;			/* at-node: only the grow, if any */
	} else {
		displaced = (d->nf && ft_node_external(d->nf)) ? d->nf : NULL;
		ft_graft_glue_init(&lg);
		branch = ft_build_branch(dst_ft, okey_dst, d->depth, dst_key_len,
			payload, cnt_src, displaced != NULL, &lg);
		if (!branch) {
			ft_graft_glue_fini(&lg);
			return -ENOMEM;
		}
		for (bi = 0; bi < lg.nr_built && !rret; bi++)
			rret = ft_merge_reserve_add_built(dst_ft, reserve,
				lg.built[bi]);
		ft_graft_glue_abort(dst_ft, &lg);	/* frees the learn nodes */
		if (rret)
			return rret;
		grow = (displaced == NULL);	/* an empty slot ADDS a child */
	}

	if (grow) {
		struct cds_ft_metadata *am =
			cds_ft_item_to_metadata(ft_node_ptr(d->pnf));
		unsigned int aidx = ft_node_type(d->pnf);

		if (am->nr_child + 1 > ft_types[aidx].max_child) {
			unsigned int gidx = find_nearest_type_index(aidx,
				am->nr_child + 1, am->parent == NULL);

			rret = cds_ft_alloc_reserve_add(dst_ft, reserve,
				CDS_FT_ALLOC_KIND_NODE, ft_types[gidx].order,
				ft_types[gidx].bitmap, 1);
		}
	}
	return rret;
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

static
enum cds_ft_status ft_merge_graft_subpos_inplace(struct cds_ft *dst_ft,
		struct cds_ft *src_ft,
		const uint8_t *okey_dst, const uint8_t *dst_key, size_t dst_key_len,
		const uint8_t *okey_src, size_t src_key_len,
		struct cds_ft_inode_flag *payload, unsigned long cnt_src,
		bool *handled)
{
	struct ft_graft_glue glue;
	struct cds_ft_alloc_reserve reserve;
	struct ft_flip_batch *pre_flip = NULL;
	struct ft_descent d;
	enum ft_graft_prep prep;
	struct cds_ft_inode_flag *attached_nf, *aparent;
	bool ms_ord = dst_ft->group->ordered_list_set;
	struct ft_ord_cell *pred = NULL, *succ = NULL;
	struct ft_ord_cell *run_first = NULL, *run_last = NULL;
	struct cds_ft_node *s_first = NULL, *s_last = NULL;
	size_t src_max, nm, dm;

	*handled = false;

	/*
	 * Classify the dst graft point with a build-invisible prep.  A GLUE
	 * diverge builds the whole split cluster into @glue (referencing @payload
	 * by deferred edge), published failure-free after the drain.  A NOSPLIT
	 * point is grafted post-drain by ft_store_at_graft_point drawing from a
	 * pre-filled reserve, so it likewise cannot fail on an arena allocation.
	 * Either way the source unlink is the single last fallible step, so an OOM
	 * leaves both tries pristine -- no rollback, no leak.
	 */
	ft_graft_glue_init(&glue);
	memset(&reserve, 0, sizeof(reserve));
	prep = ft_graft_build(dst_ft, okey_dst, dst_key_len, payload, cnt_src,
			&d, &glue);
	*handled = true;
	if (prep == FT_GRAFT_PREP_OOM) {
		ft_graft_glue_abort(dst_ft, &glue);
		return CDS_FT_STATUS_MEMORY_ERROR;	/* both tries pristine */
	}
	if (prep == FT_GRAFT_PREP_POPULATED) {
		/* Defensive: cnt_dst == 0 should never yield an occupied point. */
		ft_graft_glue_fini(&glue);
		return CDS_FT_STATUS_POPULATED_ERROR;
	}
	if (prep == FT_GRAFT_PREP_NOSPLIT) {
		/*
		 * Pre-fill the store's manifest while both tries are still pristine:
		 * the node allocations into the reserve, AND the one flip batch the
		 * store parks its proxy in (a malloc, so not reservable).  A displaced
		 * external is REPLACED via the back-channel with no flip batch; every
		 * other NOSPLIT shape (at-node empty slot, built non-displaced branch)
		 * needs exactly one.  Securing the flip batch here -- before the source
		 * unlink -- makes the post-drain store wholly failure-free, so the
		 * unlink is the true last fallible step and nothing strands.
		 */
		bool displaced = (d.depth < dst_key_len && d.nf
				&& ft_node_external(d.nf));

		if (ft_merge_nosplit_reserve(dst_ft, okey_dst, dst_key_len,
				payload, cnt_src, &d, &reserve)) {
			cds_ft_alloc_reserve_drain(dst_ft, &reserve);
			ft_graft_glue_fini(&glue);
			return CDS_FT_STATUS_MEMORY_ERROR;
		}
		if (!displaced) {
			pre_flip = ft_flip_batch_alloc(dst_ft, 1);
			if (!pre_flip) {
				cds_ft_alloc_reserve_drain(dst_ft, &reserve);
				ft_graft_glue_fini(&glue);
				return CDS_FT_STATUS_MEMORY_ERROR;
			}
		}
	}
	/* GLUE: cluster built invisibly.  NOSPLIT: reserve + flip batch secured. */

	/*
	 * Ordered list: locate the dst splice neighbours while dst is still
	 * payload-free, and capture the source subtree's run endpoints + cells
	 * while the subtree is still intact in src.
	 */
	if (ms_ord) {
		ft_ord_cell_find_splice_pos(dst_ft, dst_key, dst_key_len,
			&pred, &succ);
		s_first = ft_subtree_minmax_head(src_ft, payload, false);
		s_last = ft_subtree_minmax_head(src_ft, payload, true);
		run_first = ft_ord_cell_ptr(rcu_dereference(s_first->prev));
		run_last = ft_ord_cell_ptr(rcu_dereference(s_last->prev));
	}

	/*
	 * Last fallible step: unlink the source subtree in place, preserving
	 * @payload.  On OOM the unlink self-undoes (src pristine) and the still-
	 * invisible cluster / reserve is released (dst pristine) -- no rollback.
	 */
	if (ft_merge_unlink_src_subtree(src_ft, okey_src, src_key_len,
			cnt_src) < 0) {
		if (pre_flip)
			ft_flip_batch_free_unpublished(pre_flip);
		cds_ft_alloc_reserve_drain(dst_ft, &reserve);
		ft_graft_glue_abort(dst_ft, &glue);
		return CDS_FT_STATUS_MEMORY_ERROR;
	}

	/* ===== Everything from here on is failure-free. ===== */

	/*
	 * Remove the source run from src's ordered list now (the cells keep their
	 * internal links for the splice into dst below) -- before the drain, so
	 * sync drains src ord-readers of the run too.
	 */
	if (ms_ord)
		ft_ord_cell_run_unlink(src_ft, s_first, s_last);

	if (!src_ft->exclusive)
		src_ft->group->flavor->update_synchronize_rcu();

	if (prep == FT_GRAFT_PREP_GLUE) {
		/*
		 * Failure-free commit of the build-invisible diverge cluster: wire
		 * the deferred live back-pointers (the payload, the displaced old
		 * child, the cluster top), splice the cluster in with the single
		 * forward publish, then reclaim the replaced compressed node.
		 */
		ft_graft_glue_apply_deferred(dst_ft, &glue);
		ft_graft_glue_publish(dst_ft, &glue);
		attached_nf = glue.attached_nf;
		ft_graft_glue_free_old(dst_ft, &glue);
		ft_graft_glue_fini(&glue);
	} else {
		/*
		 * NOSPLIT: graft the in-place payload at @d.  Every node allocation
		 * draws from the reserve and the proxy parks in the pre-secured flip
		 * batch, so the store has no fallible step left -- it cannot fail.
		 */
		unsigned int adepth = 0;
		enum cds_ft_status st;

		cds_ft_alloc_reserve_activate(dst_ft, &reserve);
		st = ft_store_at_graft_point(dst_ft, okey_dst, dst_key_len, &d,
				payload, cnt_src, &attached_nf, &adepth, &glue,
				&pre_flip);
		cds_ft_alloc_reserve_deactivate(dst_ft);
		assert(st == CDS_FT_STATUS_OK);
		(void) st;
		cds_ft_alloc_reserve_drain(dst_ft, &reserve);
		ft_graft_glue_fini(&glue);
	}

	/*
	 * Propagate the moved key count up dst's ancestor chain, starting from
	 * @attached_nf's parent (its own nr_keys already carries @cnt_src).
	 * ft_get_parent_rcu handles every payload node type (external / compressed
	 * / internal) and resolves the merge-point flip proxy.
	 */
	aparent = ft_get_parent_rcu(dst_ft,
		ft_resolve_skip_compressed(dst_ft, attached_nf));
	if (aparent)
		ft_propagate_external_count_parent(dst_ft, aparent, (long) cnt_src);

	if (ms_ord) {
		/*
		 * An EXTERNAL payload is re-parented directly under a new internal
		 * slot, so its edge byte changes (src_key's last byte -> dst_key's).
		 * That byte lives in the head's CELL metadata (ft_rebuild_key_upwalk
		 * reads it for the ordered key rebuild) and ft_set_parent does NOT
		 * maintain it for externals, so refresh it here.  The moved external
		 * sits at exactly @dst_key (it was an EXACT leaf at @src_key, no
		 * suffix), so its new edge byte is the last byte of @dst_key.  An
		 * internal / compressed payload keeps every leaf's edge byte (the
		 * subtree moves wholesale), so no per-leaf fix-up is needed.
		 */
		if (ft_node_external(payload))
			cds_ft_item_to_metadata(run_first)->incoming_byte =
				okey_dst[dst_key_len - 1];
		/* Splice the moved run into dst at the located position. */
		ft_ord_cell_run_splice(dst_ft, run_first, run_last, pred, succ);
	}

	/* Raise dst's max_used_key_len for the moved keys (dst_key || suffix). */
	src_max = uatomic_load(&src_ft->max_used_key_len, CMM_RELAXED);
	nm = src_max > src_key_len ? dst_key_len + (src_max - src_key_len) :
		dst_key_len;
	dm = uatomic_load(&dst_ft->max_used_key_len, CMM_RELAXED);
	if (nm > dm)
		uatomic_store(&dst_ft->max_used_key_len, nm, CMM_RELAXED);

	return CDS_FT_STATUS_OK;
}

/*
 * @pre_flip / @pre_ms_flip carry flip batches the caller reserved before its
 * own last fallible step, so the spine-copy / graft commit below draws an
 * unfailable batch instead of allocating one.  Both NULL on the public path
 * (cds_ft_merge_at), set only by the same-trie rekey, which pre-allocates them
 * (sized from the O(1) subtree key counts) so its post-detach merge cannot fail
 * -- no reader-observable rollback.  The consume sites NULL the slot they take,
 * so the rekey frees exactly the batches a given merge shape left unused.
 */
static enum cds_ft_status ft_merge_at_inner(struct cds_ft *dst_ft,
		const uint8_t *dst_key, size_t dst_key_len,
		struct cds_ft *src_ft,
		const uint8_t *src_key, size_t src_key_len,
		struct ft_flip_batch **pre_flip,
		struct ft_flip_batch **pre_ms_flip)
{
	struct cds_ft *subtree = NULL;
	enum cds_ft_status status;
	struct ft_descent d_src, d_dst;
	unsigned int off_src, off_dst;
	unsigned long cnt_src, cnt_dst;
	enum ft_graft_swap_case ks, kd;
	uint8_t okey_dst_buf[FT_MAX_KEY_LEN], okey_src_buf[FT_MAX_KEY_LEN];
	const uint8_t *okey_dst = dst_key, *okey_src = src_key;

	FT_TP(merge_enter, (const void *) dst_ft, (const void *) src_ft);

	if (!dst_ft || !src_ft) {
		FT_TP(merge_exit, (int) CDS_FT_STATUS_INVALID_ARGUMENT_ERROR);
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}
	if (dst_ft->group != src_ft->group) {
		FT_TP(merge_exit, (int) CDS_FT_STATUS_INVALID_ARGUMENT_ERROR);
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}
	if (dst_key_len > dst_ft->group->max_key_len ||
			src_key_len > dst_ft->group->max_key_len) {
		FT_TP(merge_exit, (int) CDS_FT_STATUS_INVALID_ARGUMENT_ERROR);
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}
	if ((dst_key_len > 0 && !dst_key) ||
			(src_key_len > 0 && !src_key)) {
		FT_TP(merge_exit, (int) CDS_FT_STATUS_INVALID_ARGUMENT_ERROR);
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}
	/*
	 * Fixed-length groups: each source key K has length fixed_len,
	 * the moved subtree's stripped keys have length
	 * (fixed_len - src_key_len), and the resulting destination key
	 * is dst_key || stripped, of length
	 * (dst_key_len + fixed_len - src_key_len).  For that result to
	 * equal fixed_len (the only key length the destination group
	 * accepts), src_key_len and dst_key_len must be equal.
	 */
	if (dst_ft->group->key_len != CDS_FT_LEN_VARIABLE
			&& dst_key_len != src_key_len) {
		FT_TP(merge_exit, (int) CDS_FT_STATUS_INVALID_ARGUMENT_ERROR);
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}
	/*
	 * Combined-length overflow validation (mirrors cds_ft_graft): a moved
	 * key K becomes dst_key || (K - src_key prefix), of length
	 * dst_key_len + len(K) - src_key_len.  Bound len(K) by the source's
	 * max_used_key_len; without this check a variable-length merge with
	 * dst_key_len > src_key_len could create keys exceeding the group's
	 * max_key_len, overflowing the fixed-size key buffers downstream
	 * (the spine's compressed-wrap kbuf, the iterator buffers).
	 */
	{
		size_t src_max = uatomic_load(&src_ft->max_used_key_len,
				CMM_RELAXED);

		if (src_max > src_key_len &&
				src_max - src_key_len >
				dst_ft->group->max_key_len - dst_key_len) {
			FT_TP(merge_exit, (int) CDS_FT_STATUS_OVERFLOW_ERROR);
			return CDS_FT_STATUS_OVERFLOW_ERROR;
		}
	}

	/*
	 * merge_at is a mutator; the application provides mutual exclusion
	 * between mutators.  Take the reentrant writer-validation scope (like
	 * cds_ft_graft_swap), NOT a flavor read lock -- the spine-copy commit
	 * calls update_synchronize_rcu, which would deadlock inside a
	 * read-side critical section.
	 */
	CDS_FT_SCOPED_WRITER(dst_ft);
	CDS_FT_SCOPED_WRITER(src_ft);

	/*
	 * Convert both keys to ORDINAL form ONCE; every internal consumer
	 * (the merge-point descents, the spine build, the source-subtree
	 * unlink, the ordered-list interleave) takes the ordinal form.  The
	 * detach+graft fallback below instead receives the ORIGINAL
	 * application keys -- those entry points remap internally.
	 * Previously the descents consumed the application bytes raw while
	 * the unlink remapped: on a non-identity key map the build copied one
	 * subtree and the unlink targeted another.
	 */
	{
		const struct cds_ft_key_map *km = &dst_ft->group->key_map;

		if (caa_unlikely(!km->identity)) {
			ft_key_to_ordinals(okey_dst_buf, dst_key, dst_key_len,
				km);
			ft_key_to_ordinals(okey_src_buf, src_key, src_key_len,
				km);
			okey_dst = okey_dst_buf;
			okey_src = okey_src_buf;
		}
	}

	/*
	 * Same-trie "rekey" (src_ft == dst_ft): moving @src_key's subtree to
	 * @dst_key within one trie is allowed, but the two keys must be DISJOINT
	 * -- neither a prefix of the other.  A prefix relationship means one key
	 * lies inside the other's subtree, so the move would be circular (and
	 * @dst_key would not be absent), and equal keys are a degenerate self-
	 * move.  The check is byte-position-wise, identical in application and
	 * ordinal space (the key map is a per-position bijection).  Cross-trie
	 * subtrees are disjoint by construction, so the guard is same-trie only.
	 */
	if (src_ft == dst_ft) {
		size_t m = src_key_len < dst_key_len ? src_key_len : dst_key_len;

		if (m == 0 || memcmp(okey_src, okey_dst, m) == 0) {
			FT_TP(merge_exit,
				(int) CDS_FT_STATUS_INVALID_ARGUMENT_ERROR);
			return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
		}
	}

	/*
	 * Locate both merge points read-only (a writer descends its own
	 * stable state).  No content under @src_key -> the merge is a no-op;
	 * cnt_src == 0 covers an empty @src_ft root (src_key_len == 0, where
	 * the descent still reports EXACT at the always-present root).
	 */
	ks = ft_merge_descend(src_ft, okey_src, src_key_len, &d_src,
			&off_src, &cnt_src);
	if (ks == FT_GRAFT_SWAP_DELEGATE || cnt_src == 0) {
		FT_TP(merge_exit, (int) CDS_FT_STATUS_OK);
		return CDS_FT_STATUS_OK;
	}

	/*
	 * Same-trie "rekey": detach @src_key's subtree into a transient trie,
	 * then merge that whole trie back in at @dst_key.  The detach fully
	 * commits -- recompacting the shared common-prefix ancestor and draining
	 * -- BEFORE the placement reads it, so the move's two sides no longer
	 * alias the same node.  The in-place reorder CANNOT be used here: its
	 * source unlink would recompact that shared ancestor out from under the
	 * build-invisible destination placement (a stale-slot publish).  The
	 * cross-trie merge handles an occupied @dst_key by MERGING and an absent
	 * one by grafting; it is made UNFAILABLE (its nodes + flip batches are
	 * pre-reserved before the detach below), so it always commits the move --
	 * there is no failure path that would restore the content to @src_key, i.e.
	 * no reader-observable rollback.
	 *
	 * A KEY_SHORTER source (off_src > 0, @src_key ends inside the compressed
	 * node @d_src.nf) is reduced to the EXACT node boundary first, exactly as
	 * the cross-trie path does: detach @cn_s->child via the full-compressed
	 * key (src prefix ++ all of cn_s), and merge at @dst_key extended by the
	 * residual cn_s bytes -- ft_detach_keylen overshoots a compressed node, so
	 * it must be handed a boundary key, not an interior one.
	 */
	if (src_ft == dst_ft) {
		struct cds_ft *tmp = NULL;
		const uint8_t *det_key = src_key, *mrg_key = dst_key;
		size_t det_len = src_key_len, mrg_len = dst_key_len;
		uint8_t det_buf[FT_MAX_KEY_LEN], mrg_buf[FT_MAX_KEY_LEN];
		uint8_t omrg_buf[FT_MAX_KEY_LEN];
		const uint8_t *omrg = okey_dst;		/* ordinal mrg key */
		size_t omrg_len = dst_key_len;
		struct cds_ft_alloc_reserve reserve;
		struct ft_descent d_mrg;
		unsigned int off_mrg;
		unsigned long n = cnt_src, m;		/* moved / dst subtree counts */
		struct ft_flip_batch *pf_flip = NULL, *pf_ms = NULL;

		if (off_src > 0) {
			struct cds_ft_compressed_node *cn_s =
				ft_compressed_node_ptr(d_src.nf);
			const struct cds_ft_key_map *km =
				&dst_ft->group->key_map;
			unsigned int base = (unsigned int) d_src.depth, j;

			/* det_key = src prefix to cn_s start ++ ALL of cn_s. */
			memcpy(det_buf, src_key, base);
			/* mrg_key = dst_key ++ the residual cn_s bytes. */
			memcpy(mrg_buf, dst_key, dst_key_len);
			if (km->identity) {
				memcpy(&det_buf[base], cn_s->key_bytes, cn_s->len);
				memcpy(&mrg_buf[dst_key_len],
					&cn_s->key_bytes[off_src],
					cn_s->len - off_src);
			} else {
				for (j = 0; j < cn_s->len; j++)
					det_buf[base + j] = km->ordinal_to_key[
						cn_s->key_bytes[j]];
				for (j = off_src; j < cn_s->len; j++)
					mrg_buf[dst_key_len + j - off_src] =
						km->ordinal_to_key[
						cn_s->key_bytes[j]];
			}
			det_key = det_buf;
			det_len = base + cn_s->len;
			mrg_key = mrg_buf;
			mrg_len = dst_key_len + (cn_s->len - off_src);
			/* Ordinal mrg key: dst prefix ++ residual cn_s (already ordinal). */
			memcpy(omrg_buf, okey_dst, dst_key_len);
			memcpy(&omrg_buf[dst_key_len], &cn_s->key_bytes[off_src],
				cn_s->len - off_src);
			omrg = omrg_buf;
			omrg_len = mrg_len;
		}

		/*
		 * Read-only count pass at @mrg_key (writer lock held, so the counts
		 * stay valid for the post-detach merge): @m, the dst subtree key
		 * count, bounds the structural re-parent flips (nr_dst <= m); @n
		 * (== cnt_src, the moved subtree) sizes the ordered interleave flips
		 * (2n+2).  Both are O(1) reads off the subtree-root metadata.  The
		 * detach preserves @n into @tmp and only reshapes the PATH to
		 * @mrg_key (never its subtree), so nr_dst <= m still holds after.
		 * m == 0 means @mrg_key is absent (the merge grafts).
		 */
		(void) ft_merge_descend(dst_ft, omrg, omrg_len, &d_mrg, &off_mrg,
			&m);

		/*
		 * Pre-allocate the merge's flip batches HERE -- before the detach,
		 * where a malloc failure is harmless (nothing has moved).  The
		 * post-detach merge then has no fallible allocation left (every node
		 * draws from @reserve, every flip proxy from these batches), so it
		 * CANNOT fail and needs no reader-observable rollback.  @pf_flip
		 * (cap m+1) feeds the structural re-parent or the graft store;
		 * @pf_ms (cap 2n+2, only for an occupied ordered merge) feeds the
		 * ordered-list interleave.  Caps are upper bounds; the merge uses
		 * <= them and NULLs the slot it consumes, so we free the rest below.
		 */
		pf_flip = ft_flip_batch_alloc(dst_ft, (unsigned int) (m + 1));
		if (!pf_flip) {
			FT_TP(merge_exit, (int) CDS_FT_STATUS_MEMORY_ERROR);
			return CDS_FT_STATUS_MEMORY_ERROR;
		}
		if (m > 0 && dst_ft->group->ordered_list_set) {
			pf_ms = ft_flip_batch_alloc(dst_ft,
				(unsigned int) (2 * n + 2));
			if (!pf_ms) {
				ft_flip_batch_free_unpublished(pf_flip);
				FT_TP(merge_exit,
					(int) CDS_FT_STATUS_MEMORY_ERROR);
				return CDS_FT_STATUS_MEMORY_ERROR;
			}
		}

		/*
		 * Generous node reserve, also before the detach, so the merge's node
		 * allocations cannot fail either.  With both reserves in hand the
		 * detach is the LAST fallible step (clean on its own failure).
		 */
		memset(&reserve, 0, sizeof(reserve));
		if (ft_bulk_node_reserve_fill(dst_ft, &reserve)) {
			cds_ft_alloc_reserve_drain(dst_ft, &reserve);
			ft_flip_batch_free_unpublished(pf_flip);
			if (pf_ms)
				ft_flip_batch_free_unpublished(pf_ms);
			FT_TP(merge_exit, (int) CDS_FT_STATUS_MEMORY_ERROR);
			return CDS_FT_STATUS_MEMORY_ERROR;
		}
		status = ft_detach_keylen(dst_ft, det_key, det_len, &tmp);
		if (status < 0) {
			cds_ft_alloc_reserve_drain(dst_ft, &reserve);
			ft_flip_batch_free_unpublished(pf_flip);
			if (pf_ms)
				ft_flip_batch_free_unpublished(pf_ms);
			FT_TP(merge_exit, (int) (status == CDS_FT_STATUS_NOT_FOUND
				? CDS_FT_STATUS_OK : status));
			return status == CDS_FT_STATUS_NOT_FOUND
				? CDS_FT_STATUS_OK : status;	/* NOT_FOUND -> no-op */
		}
		cds_ft_alloc_reserve_activate(dst_ft, &reserve);
		cds_ft_alloc_reserve_activate(tmp, &reserve);
		/*
		 * Unfailable placement: every node draws from @reserve and every
		 * flip proxy from @pf_flip / @pf_ms, so the merge always commits the
		 * move.  No failure path can strand @tmp's content back at @det_key
		 * -- that re-graft was the reader-observable rollback we removed.
		 */
		status = ft_merge_at_inner(dst_ft, mrg_key, mrg_len, tmp, NULL, 0,
			&pf_flip, &pf_ms);
		cds_ft_alloc_reserve_deactivate(dst_ft);
		cds_ft_alloc_reserve_deactivate(tmp);
		assert(status == CDS_FT_STATUS_OK);
		/* Free the flip batches this merge shape did not consume. */
		if (pf_flip)
			ft_flip_batch_free_unpublished(pf_flip);
		if (pf_ms)
			ft_flip_batch_free_unpublished(pf_ms);
		cds_ft_alloc_reserve_drain(dst_ft, &reserve);
		cds_ft_destroy(tmp);
		FT_TP(merge_exit, (int) status);
		return status;
	}

	kd = ft_merge_descend(dst_ft, okey_dst, dst_key_len, &d_dst,
			&off_dst, &cnt_dst);

	/*
	 * Atomic build-invisible spine-copy of @src_ft's subtree at @src_key into
	 * @dst_ft's non-empty subtree at @dst_key.  Each side is EXACT (@off == 0,
	 * the subtree at @d->nf) or KEY_SHORTER (@off > 0, the key ends inside a
	 * compressed node -- the src node is reclaimed by the commit's unlink, the
	 * dst node is wrapped under its prefix).  ft_merge_spine_copy handles every
	 * such shape (internal, external, compressed and compressed-parent dst
	 * merge points); only a dst that is empty under @dst_key falls through to
	 * the detach-based graft below.
	 */
	if ((ks == FT_GRAFT_SWAP_EXACT || ks == FT_GRAFT_SWAP_KEY_SHORTER)
			&& cnt_dst > 0
			&& (kd == FT_GRAFT_SWAP_EXACT
				|| kd == FT_GRAFT_SWAP_KEY_SHORTER)) {
		status = ft_merge_spine_copy(dst_ft, src_ft, &d_src,
				okey_src, src_key_len, cnt_src, off_src,
				&d_dst, cnt_dst, off_dst, okey_dst, dst_key_len,
				pre_flip, pre_ms_flip);
		if (status == CDS_FT_STATUS_OK) {
			/*
			 * Raise dst's max_used_key_len for the moved keys
			 * (dst_key_len + the longest stripped suffix), as the
			 * graft paths do -- later graft overflow validations
			 * feed off it.
			 */
			size_t src_max = uatomic_load(&src_ft->max_used_key_len,
					CMM_RELAXED);
			size_t nm = src_max > src_key_len ?
				dst_key_len + (src_max - src_key_len) :
				dst_key_len;
			size_t dm = uatomic_load(&dst_ft->max_used_key_len,
					CMM_RELAXED);

			if (nm > dm)
				uatomic_store(&dst_ft->max_used_key_len, nm,
					CMM_RELAXED);
		}
		FT_TP(merge_exit, (int) status);
		return status;
	}

	/*
	 * Whole-source move (src_key_len == 0): the entire @src_ft is the
	 * payload, so graft it directly -- no detach, no fallible rollback.
	 * ft_graft is itself leak-free: a diverge split is built invisibly and
	 * a NOSPLIT attach restores the saved old source root allocation-free
	 * on OOM, so an allocation failure leaves both tries pristine.  This
	 * covers an empty dst root and a diverged dst_key alike, and is the
	 * leak-free path for "rekey within a trie" (detach a sub-trie, then
	 * merge_at it at a new, currently-absent key in the same group).  A
	 * dst_key occupied by an empty-internal leftover yields POPULATED_ERROR
	 * here, exactly as the detach-then-graft path did.
	 */
	if (src_key_len == 0) {
		status = ft_graft_keylen(dst_ft, dst_key, dst_key_len, src_ft,
				pre_flip);
		FT_TP(merge_exit, (int) status);
		return status;
	}

	/*
	 * Empty dst ROOT (dst_key_len == 0, so the whole @dst_ft is empty at
	 * the merge point): move @src_ft@src_key in WITHOUT a fallible graft,
	 * so no rollback can strand the moved externals.  Pre-allocate the
	 * source's replacement root (fallible while @src_ft is still pristine),
	 * detach the source subtree (clean on its own failure), then SWAP it
	 * into @dst_ft's root.  The swap is a failure-free root-to-root move:
	 * no parent-pointer change and no "jump out" window (both ends are
	 * roots with parent NULL), and the detach already drained @src_ft's
	 * readers -- the same reasoning as ft_graft's root path.  Because
	 * nothing fallible follows the detach, this path has no rollback, so it
	 * cannot leak (contrast the diverged path below).
	 */
	if (dst_key_len == 0 && cnt_dst == 0) {
		struct cds_ft_inode *fresh_root;
		struct cds_ft_metadata *fresh_meta;
		struct cds_ft_inode *old_dst_root;
		size_t sm;

		fresh_root = alloc_cds_ft_node(src_ft, &ft_types[0], &fresh_meta);
		if (!fresh_root) {
			status = CDS_FT_STATUS_MEMORY_ERROR;
			goto out;
		}
		rcu_assign_pointer(fresh_meta->parent, NULL);
		ft_nr_keys_store(fresh_meta, 0, CMM_RELAXED);

		status = ft_detach_keylen(src_ft, src_key, src_key_len, &subtree);
		if (status < 0) {
			/* NOT_FOUND impossible: @src_ft had content. */
			free_cds_ft_node_unpublished(src_ft, fresh_root);
			goto out;
		}

		/*
		 * Failure-free swap.  @subtree->root carries the moved content
		 * with parent already NULL (set by the detach), so it drops
		 * straight into @dst_ft's empty root slot.  @subtree keeps the
		 * pre-allocated empty root so its destroy below frees nothing of
		 * the moved content.  @dst_ft was empty, so it adopts @subtree's
		 * whole ordered cell list wholesale (head/tail endpoints only).
		 */
		old_dst_root = ft_node_ptr(dst_ft->root);
		rcu_assign_pointer(dst_ft->root, subtree->root);
		FT_TP(root_publish, (const void *) dst_ft,
			(const void *) dst_ft->root);
		if (dst_ft->group->ordered_list_set) {
			rcu_assign_pointer(dst_ft->ord_cell_head,
				subtree->ord_cell_head);
			rcu_assign_pointer(dst_ft->ord_cell_tail,
				subtree->ord_cell_tail);
			subtree->ord_cell_head = NULL;
			subtree->ord_cell_tail = NULL;
		}
		rcu_assign_pointer(subtree->root, ft_node_flag(fresh_root, 0));
		free_cds_ft_node(dst_ft, old_dst_root);

		/* dst_key_len == 0: dst keys equal the moved keys, same lengths. */
		sm = uatomic_load(&subtree->max_used_key_len, CMM_RELAXED);
		if (sm > uatomic_load(&dst_ft->max_used_key_len, CMM_RELAXED))
			uatomic_store(&dst_ft->max_used_key_len, sm, CMM_RELAXED);

		cds_ft_destroy(subtree);
		FT_TP(merge_exit, (int) CDS_FT_STATUS_OK);
		return CDS_FT_STATUS_OK;
	}

	/*
	 * Sub-position source into a dst absent at @dst_key: graft the source
	 * subtree IN PLACE (no detach, no re-root) so the build-invisible cluster
	 * references @d_src.nf directly and the source unlink is the last fallible
	 * step -- the leak-free reorder.  Handles every source-root shape (plain
	 * multi-child internal, an internal carrying external_nodes, external,
	 * compressed, 1-child) at both a GLUE diverge and a NOSPLIT (at-node /
	 * build-branch) dst point.
	 *
	 * A KEY_SHORTER source (the src key ends @off_src bytes inside the
	 * compressed node @d_src.nf) reduces to the EXACT shape: the moved subtree
	 * is @cn_s->child, and its keys gain the residual bytes cn_s->key_bytes
	 * [off_src .. len) as a prefix, so graft @cn_s->child at @dst_key extended
	 * by that residual.  ft_merge_unlink_src_subtree overshoots @cn_s (preserves
	 * @cn_s->child) so the source side is identical; @cn_s itself is reclaimed
	 * by that unlink.
	 */
	if (ks == FT_GRAFT_SWAP_EXACT || ks == FT_GRAFT_SWAP_KEY_SHORTER) {
		bool handled;
		struct cds_ft_inode_flag *payload;
		const uint8_t *gokey = okey_dst, *gappkey = dst_key;
		size_t glen = dst_key_len;
		uint8_t ext_okey[FT_MAX_KEY_LEN], ext_app[FT_MAX_KEY_LEN];

		if (ks == FT_GRAFT_SWAP_EXACT) {
			payload = d_src.nf;	/* off_src == 0 by construction */
		} else {
			struct cds_ft_compressed_node *cn_s =
				ft_compressed_node_ptr(d_src.nf);
			unsigned int rlen = cn_s->len - off_src, j;
			const struct cds_ft_key_map *km =
				&dst_ft->group->key_map;

			payload = cn_s->child;
			memcpy(ext_okey, okey_dst, dst_key_len);
			memcpy(&ext_okey[dst_key_len], &cn_s->key_bytes[off_src],
				rlen);
			memcpy(ext_app, dst_key, dst_key_len);
			if (km->identity) {
				memcpy(&ext_app[dst_key_len],
					&cn_s->key_bytes[off_src], rlen);
			} else {
				for (j = 0; j < rlen; j++)
					ext_app[dst_key_len + j] =
						km->ordinal_to_key[
						cn_s->key_bytes[off_src + j]];
			}
			gokey = ext_okey;
			gappkey = ext_app;
			glen = dst_key_len + rlen;
		}

		status = ft_merge_graft_subpos_inplace(dst_ft, src_ft,
			gokey, gappkey, glen, okey_src, src_key_len,
			payload, cnt_src, &handled);
		if (handled) {
			FT_TP(merge_exit, (int) status);
			return status;
		}
	}

	/*
	 * Unreachable.  ft_merge_descend reports EXACT, KEY_SHORTER or DELEGATE;
	 * DELEGATE returned OK at the top (cnt_src == 0), and every EXACT /
	 * KEY_SHORTER source is handled above: a non-empty dst by the build-
	 * invisible spine copy, an empty dst root or whole-source move by their
	 * failure-free root swaps, and a diverged / absent dst point by
	 * ft_merge_graft_subpos_inplace (which always reports handled).  No shape
	 * falls through, so the old detach-then-graft fallback -- the last path
	 * that carried the double-OOM leak -- and its rollback are gone.
	 */
	assert(0 && "cds_ft_merge_at: unhandled merge shape");
	status = CDS_FT_STATUS_OK;
out:
	FT_TP(merge_exit, (int) status);
	return status;
}

