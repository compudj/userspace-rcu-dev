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
/* A relocation publish is at most the forward edge + a compressed parent's SKIP_X dual. */
#define FT_RELOCATE_COMMIT_MAX_EDGES	2
/*
 * Relocate the internal node at *@holder into a fresh slot; RCU-free the old.
 * Sets *@oom on a best-effort leave-in-place (allocation failure) so the caller
 * can stop the pass rather than keep walking into doomed allocations.
 */
static
void ft_compact_relocate_at(struct cds_ft *ft, struct cds_ft_inode_flag **holder,
		bool *oom)
{
	struct cds_ft_inode_flag *nf = *holder;
	unsigned int type_index = ft_node_type(nf);
	struct cds_ft_inode *node = ft_node_ptr(nf), *old_ret = NULL;
	struct cds_ft_metadata *meta = cds_ft_item_to_metadata(node);
	struct cds_ft_inode_flag *parent = meta->parent;
	struct ft_pub_rec rec = { .n = 0 };
	struct ft_flip_txn *txn = NULL;
	int ret;

	/*
	 * The recompacted node's forward publish carries a SKIP_X dual (a second
	 * reader-visible edge) when its parent is compressed.  Either way the old
	 * copy this relocation retires gets its freeze-on-free tombstone recorded
	 * into the SAME commit (atomic detach, §4.B), so ALWAYS pre-reserve a
	 * bounded txn -- forward (+ SKIP_X dual for a compressed parent) + the
	 * tombstone -- BEFORE ft_node_recompact eagerly re-parents the rebuilt
	 * node's children: that re-parent is a point of no return (a clean _try
	 * abort would orphan the children onto an unpublished node), so the commit
	 * must be infallible.  A non-compressed parent, formerly a lone on-stack
	 * forward store, is now a 2-edge flip (forward + tombstone).  Reservation
	 * failure leaves the node in place -- the same best-effort contract as a
	 * node / cell allocation failure: nothing reader-visible has changed and
	 * nothing is reserved-but-leaked at the return.
	 */
	{
		unsigned int cap = (parent && (ft_node_compressed(parent) ||
				ft_node_skip_compressed(parent))) ?
			FT_RELOCATE_COMMIT_MAX_EDGES + 1 : 2;

		txn = ft_flip_txn_create_bounded(cap);
		if (!txn) {
			*oom = true;
			return;		/* OOM: best-effort, leave in place */
		}
	}
	ret = ft_node_recompact(FT_RECOMPACT_RELOCATE, ft, type_index,
			&ft_types[type_index], node, meta, holder,
			0, NULL, NULL, &old_ret, holder == &ft->root, 0,
			false, &rec, txn);
	if (ret != 0) {
		/*
		 * Node allocation failed inside the recompact before any
		 * reader-visible store, or the copy met a peer's parked flip
		 * proxy and abandoned itself (-EAGAIN, side effects undone).
		 * Either way nothing was recorded into @rec and *holder is
		 * unchanged: best-effort, leave the node in place (a bailed
		 * relocation is retried by a later compaction pass).
		 */
		if (txn)
			ft_flip_txn_destroy(txn);	/* reserved, unused */
		*oom = true;
		return;
	}
	/*
	 * Commit the recorded forward publish (and a compressed parent's SKIP_X
	 * dual) together with the old copy's freeze-on-free tombstone atomically
	 * through the pre-reserved @txn.  This is the moment the relocated node
	 * becomes reader-reachable at *holder and the old copy is frozen dead.
	 */
	/* Compaction, not yet MW-hardened: ABORT unreachable under its exclusion. */
	(void) ft_remove_commit_rec(ft, &rec, NULL, NULL, txn);
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
 * Returns the new compressed node (or @cn unchanged on allocation failure, with
 * *@oom set so the caller can stop the pass).
 */
static
struct cds_ft_compressed_node *ft_compact_relocate_compressed(struct cds_ft *ft,
		struct cds_ft_compressed_node *cn,
		struct cds_ft_inode_flag **gp_slot,
		bool *oom)
{
	struct cds_ft_metadata *cn_meta =
		cds_ft_item_to_metadata((struct cds_ft_inode *) cn);
	struct cds_ft_metadata *cn2_meta;
	struct cds_ft_compressed_node *cn2;
	struct cds_ft_inode_flag *cn2_flag;
	uint8_t len = cn->len;

	cn2 = alloc_compressed_node(ft, len, &cn2_meta);
	if (!cn2) {
		*oom = true;
		return cn;	/* OOM: leave in place (best-effort) */
	}
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
	ft_meta_parent_slot_offset_set(cn2_meta,
		ft_meta_parent_slot_offset(cn_meta));
	/* Same slot in the same parent => same incoming edge byte (up-walk source). */
	cn2_meta->incoming_byte = cn_meta->incoming_byte;
	ft_meta_nr_child_set(cn2_meta, ft_meta_nr_child(cn_meta));		/* == 1 for a compressed node */
	cn2_meta->external_nodes = NULL;		/* never set on a compressed node */
	ft_nr_keys_store(ft,cn2_meta, ft_nr_keys_get(cn_meta), CMM_RELAXED);
	cn2_flag = ft_compressed_node_flag(cn2);
	if (gp_slot) {
		/*
		 * Traditional compressed node: TWO reader-followable edges move
		 * -- the live child's back-reference (internal: parent+offset;
		 * external: prev) and the grandparent forward slot.  Commit them
		 * as ONE pre-reserved flip txn: the old two-step (bare back-ref
		 * store, then a lone forward flip) left a window where an
		 * up-walk from the still-reachable child entered the
		 * NOT-YET-PUBLISHED copy, and a peer mutation between the steps
		 * would be published over by a stale copy (the stale-plan
		 * class).  ft_reparent_record dispatches the child kind and
		 * co-commits the (parent, offset) pair; the forward edge is a
		 * plain structural record (no SKIP_X dual: the descent never
		 * chains two compressed nodes).  OOM: nothing recorded is
		 * installed -- destroy the txn, free the unpublished copy, leave
		 * @cn in place and stop the pass.
		 */
		struct ft_flip_txn *t = ft_flip_txn_create_bounded(3);

		if (!t) {
			free_compressed_node_unpublished(ft, cn2);
			*oom = true;
			return cn;
		}
		ft_reparent_record(ft, t, cn2->child, cn2_flag, &cn2->child);
		ft_flip_txn_record_reserved(t, (void **) gp_slot, *gp_slot,
			cn2_flag);
		if (ft_flip_txn_commit(ft, t) != URCU_TXN_STATUS_OK) {
			free_compressed_node_unpublished(ft, cn2);
			*oom = true;
			return cn;
		}
	} else {
		/*
		 * Skip form: the skip pointer addresses the TARGET, so the
		 * child's back-reference redirect IS the lone structural edge
		 * publishing the relocation -- a single atomic store, no
		 * two-step window.
		 */
		ft_set_parent(ft, cn2->child, cn2_flag, &cn2->child);
	}
	/*
	 * Freeze the retired compressed node dead (§4.B freeze-on-free) before it
	 * is handed to call_rcu.  Its structural unlink is the bare child
	 * back-reference redirect above (skip case, gp_slot == NULL) or the
	 * grandparent forward flip (traditional) -- neither is yet a flip-txn edge
	 * this mark can ride, so it is a lone-edge mark (the atomic mark+unlink
	 * fold, mirroring ft_compact_relocate_at, awaits the bidir-list weld:
	 * project_ft_bidir_list_integration).  Placed AFTER the unlink so a live,
	 * reader-reachable node is never marked dead: @cn is already detached
	 * (unreachable via its child's back-pointer / grandparent slot) and merely
	 * awaiting its grace period, exactly the transient ft_skip_reanchor already
	 * tolerates; the mark just adds the advisory tombstone bit a future
	 * concurrent writer's freeze-guard CAS will consult.
	 */
	ft_meta_tombstone_set_flip(cn_meta);
#ifdef FT_DEBUG_TOMBSTONE_AUDIT
	/*
	 * Freeze-on-free guard (doc §4.B): the compactor's always-deferred free
	 * bypasses free_compressed_node's assert, so enforce it here at the
	 * retire site -- every relocated-away compressed node must be tombstoned
	 * before it is handed to call_rcu.
	 */
	assert(ft_meta_tombstone(cn_meta));
#endif
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
 * allocation failure (best-effort: leave it in place, with *@oom set so the
 * caller can stop the pass).
 *
 * Atomicity reuses ft_ord_cell_swap for the two ordered-list edges
 * (pred->ord_next / succ->ord_prev flip together via the flip-latch, so a
 * bidirectional ordered reader never sees a half-relocated list; ord_cell_head
 * /tail follow).  That swap is the abortable commit boundary: on a flip-txn
 * OOM it installs nothing and returns -ENOMEM, and this relocation leaves @old
 * in place (best-effort), discarding the never-published @new cell.  The head's
 * UPWARD reference (head->prev) is then re-pointed
 * with a PLAIN RCU store: an up-walk reader resolves the old or the new cell,
 * both carrying an IDENTICAL parent (compaction runs under writer exclusion, so
 * @old->parent is settled), and @old stays live until its grace period -- so no
 * multi-edge flip is needed: head->prev is re-pointed with a lone-edge store
 * that installs no proxy.  (head-prev readers now resolve a parked proxy at the
 * load via ft_dereference_prev_resolved, so a proxy there would be legal -- but
 * this relocation never parks one.)  @old keeps its own links for parked ordered
 * readers and is RCU-freed (its general-arena range drains for reclaim).
 */
static
struct ft_ord_cell *ft_compact_relocate_cell(struct cds_ft *ft,
		struct ft_ord_cell *old, bool *oom)
{
	struct cds_ft_metadata *meta = cds_ft_alloc_cell_item(ft);
	struct cds_ft_node *head = old->node;
	struct ft_ord_cell *new_cell;

	if (!meta) {
		*oom = true;
		return old;		/* OOM: best-effort, leave in place */
	}
	if (ft_debug_counters())
		uatomic_inc(&ft->group->nr_cells_allocated);
	new_cell = (struct ft_ord_cell *) cds_ft_metadata_to_item(meta);
	new_cell->node = head;
	new_cell->parent = old->parent;
	/* Carry the head's edge byte across the relocation (up-walk key source). */
	meta->incoming_byte = cds_ft_item_to_metadata(old)->incoming_byte;
	/* ord_prev / ord_next are set from @old's neighbours by the swap. */
	if (ft_ord_cell_swap(ft, old, new_cell) != 0) {
		/*
		 * OOM reserving the swap flip-txn: the abortable commit
		 * installed nothing, so @old stays fully in the ordered list.
		 * Discard @new_cell (never published -- no reader can reach it)
		 * and leave @old in place, the same best-effort contract as the
		 * cell-allocation failure above.
		 */
		if (ft_debug_counters())
			uatomic_inc(&ft->group->nr_cells_freed);
		cds_ft_free_item_unpublished(ft, meta);
		*oom = true;
		return old;
	}
	/*
	 * Retarget the relocated head's node->cell back-link to @new_cell.  This
	 * is the SECOND store of the cell relocation -- ft_ord_cell_swap already
	 * moved the ordered-list neighbour links in its own flip -- so by shape it
	 * "should" fuse with that swap.  It need not (head->prev readers now resolve
	 * a parked proxy at the load via ft_dereference_prev_resolved, so a fused
	 * proxied edge would be legal -- but it is unnecessary here): @old and
	 * @new_cell are observationally IDENTICAL in every
	 * field a reader reaches through this back-link -- node, parent and
	 * incoming_byte are copied above, and ft_ord_cell_swap points new_cell's
	 * ord_prev/ord_next at @old's pred/succ while leaving @old's own links
	 * intact (both cells still point at the same pred/succ).  So a reader
	 * resolving the old-XOR-new back-link mid-relocation sees the same node,
	 * parent and predecessor/successor: the inter-store window is
	 * observationally empty.  Commit it as a lone-edge flip -- one release
	 * store, no proxy, byte-identical to the bare rcu_assign_pointer it
	 * replaces -- captured as a {slot, old, new} descriptor so a future
	 * multi-writer MCAS covers head->prev uniformly (it can be in a concurrent
	 * head-splice / head-promote writer's word-set).
	 */
	{
		struct ft_ord_cell_edge edge = {
			.slot = (struct ft_ord_cell **) &head->prev,
			.old_target = (struct ft_ord_cell *) head->prev,
			.new_target = (struct ft_ord_cell *)
				ft_ord_cell_flag(new_cell),
		};

		ft_ord_cell_flip_one(&edge);
	}
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
 * and the skip pointer.  Increments *@relocated per node moved.  Stops the
 * descent and sets *@oom if a relocation hits an allocation failure (the node
 * stays in place; further nodes on this path would likely fail the same way).
 */
static
void ft_compact_descend(struct cds_ft *ft, const uint8_t *key,
		size_t key_len, unsigned long *relocated, bool *oom)
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
			ft_compact_relocate_at(ft, holder, oom);
			(*relocated)++;
			if (*oom)
				return;		/* memory pressure: stop the descent */
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
				cn = ft_compact_relocate_compressed(ft, cn, NULL, oom);
				(*relocated)++;
				if (*oom)
					return;		/* memory pressure: stop the descent */
			}
			holder = &cn->child;
		} else if (ft_node_compressed(raw)) {
			struct cds_ft_compressed_node *cn = ft_compressed_node_ptr(raw);

			depth += cn->len;
			/* Traditional: the grandparent slot (child_slot) holds the cn flag. */
			if (!cds_ft_metadata_in_recompact_private(
					cds_ft_item_to_metadata((struct cds_ft_inode *) cn))) {
				cn = ft_compact_relocate_compressed(ft, cn, child_slot, oom);
				(*relocated)++;
				if (*oom)
					return;		/* memory pressure: stop the descent */
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
	/*
	 * Set when the last step stopped because a relocation hit an allocation
	 * failure (memory pressure).  The bound iterator cursor is left AT the
	 * interrupted key, so a subsequent step re-attempts it INCLUSIVELY
	 * (cds_ft_lookup_ge): leaving even one node of a key un-relocated pins
	 * its whole old arena range against reclaim, so resume must lose no key.
	 * Reset at each step entry (a fresh attempt clears it).
	 */
	bool oom;
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

enum cds_ft_compact_status cds_ft_compact_step(struct cds_ft_compact_state *st,
		size_t batch)
{
	struct cds_ft *ft = st->ft;
	const struct rcu_flavor_struct *flavor = ft->group->flavor;
	unsigned long relocated = 0;
	bool resume_inclusive;

	if (st->done)
		return CDS_FT_COMPACT_DONE;
	if (batch == 0)
		batch = FT_COMPACT_BATCH_DEFAULT;

	/*
	 * If the previous step stopped on OOM, its bound cursor is the
	 * interrupted (not-fully-relocated) key.  Re-attempt it INCLUSIVELY on
	 * this step's first lookup (lookup_ge, >=) instead of advancing past it
	 * (lookup_gt, >): completing that key is what lets its old range drain.
	 * The descent is idempotent (already-relocated nodes are recompact_
	 * private), so the re-attempt only finishes the un-relocated remainder.
	 */
	resume_inclusive = st->oom;
	st->oom = false;

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
		 * retained key -- inclusively on an OOM resume (see above).
		 */
		if (!st->started)
			s = cds_ft_lookup_first(ft, st->iter);
		else if (resume_inclusive)
			s = cds_ft_lookup_ge(ft, st->iter);
		else
			s = cds_ft_lookup_gt(ft, st->iter);
		resume_inclusive = false;	/* only the first lookup re-attempts */
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
		ft_compact_descend(ft, key, key_len, &relocated, &st->oom);
		/*
		 * Relocate this key's cell into a dense private cell range, in the
		 * same key order the iterator visits -- so the ordered cell list
		 * becomes a near-sequential scan.  iter->node is the chain head;
		 * its cell is head->prev.  Skip a cell already moved this pass
		 * (its range is recompact_private), mirroring the node descent, so
		 * the pass is idempotent and re-visits do not re-allocate.  Skip it
		 * too when the descent stopped on OOM: the head node may be un-
		 * relocated, and the resume re-attempts this whole key anyway.
		 */
		if (!st->oom && ft->group->ordered_list_set && st->iter->node) {
			struct ft_ord_cell *cell = ft_ord_cell_ptr(
				rcu_dereference(st->iter->node->prev));

			if (!cds_ft_metadata_in_recompact_private(
					cds_ft_item_to_metadata(cell))) {
				ft_compact_relocate_cell(ft, cell, &st->oom);
				relocated++;
			}
		}
		if (st->oom)
			break;		/* memory pressure: stop, resume re-attempts */
	}
	/*
	 * Drop the cached path before releasing the read lock: the nodes it
	 * references become eligible for the grace-period free once unlocked.
	 * Bind (not just invalidate) so the iterator's key is materialized into
	 * its own buffer -- on a reference-keycopy / ordinal-cell group the live
	 * key is a leaf reference that does NOT survive the unlock, and the next
	 * step re-descends from that key (the interrupted key on an OOM stop).
	 */
	cds_ft_iter_bind_key(st->iter);
	flavor->read_unlock();
	ft_recompact_alloc_set_active(NULL);
	/*
	 * OOM takes precedence (the caller frees memory and resumes from the
	 * interrupted key); otherwise report completion or that more remains.
	 */
	if (st->oom)
		return CDS_FT_COMPACT_OOM;
	return st->done ? CDS_FT_COMPACT_DONE : CDS_FT_COMPACT_MORE;
}

void cds_ft_compact_end(struct cds_ft_compact_state *st)
{
	st->ft->active_compact = NULL;
	ft_recompact_alloc_merge(&st->ctx);
	cds_ft_iter_destroy(st->iter);
	free(st);
}

enum cds_ft_compact_status cds_ft_compact(struct cds_ft *ft)
{
	CDS_FT_SCOPED_WRITER(ft);
	struct cds_ft_compact_state *st = cds_ft_compact_begin(ft);
	enum cds_ft_compact_status s;

	if (!st)
		return CDS_FT_COMPACT_OOM;	/* could not start the pass */
	/*
	 * One-shot: drive to completion, but STOP on the first OOM rather than
	 * walking the rest of the trie into doomed allocations.  The trie stays
	 * valid and partially compacted; a caller that wants to free memory and
	 * continue should drive cds_ft_compact_begin/step/end (which resumes).
	 */
	do {
		s = cds_ft_compact_step(st, 0);
	} while (s == CDS_FT_COMPACT_MORE);
	cds_ft_compact_end(st);
	return s;	/* CDS_FT_COMPACT_DONE or CDS_FT_COMPACT_OOM */
}
