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
 *
 * @node_depth is the node's absolute byte depth and @ctx carries the walk's
 * descent: together they let the recompaction's acquire DATE its {C, P, GP}
 * members and resolve each one's anchor under a coarse spacing.  Passing
 * neither refuses every member that is not the root -- the one-hop rule dates a
 * parent as @node_depth - span(parent), which underflows at depth 0 -- and the
 * refusal is a -EAGAIN this best-effort caller reports as *@oom, so the pass
 * makes no progress and its driver loops forever.
 */
static
void ft_compact_relocate_at(struct cds_ft *ft, struct cds_ft_inode_flag **holder,
		unsigned int node_depth, const struct ft_lock_ctx *ctx,
		int *bail)
{
	struct cds_ft_inode_flag *nf = *holder;
	unsigned int type_index = ft_node_type(nf);
	struct cds_ft_inode *node = ft_node_ptr(nf), *old_ret = NULL;
	struct cds_ft_metadata *meta = cds_ft_item_to_metadata(node);
	struct cds_ft_inode_flag *parent = ft_parent_node(meta->parent_word);
	struct ft_pub_rec rec = { .ctx = ctx, .n = 0 };
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

		txn = ft_flip_txn_create_bounded(ft, cap);
		if (!txn) {
			*bail = -ENOMEM;
			return;		/* OOM: best-effort, leave in place */
		}
	}
	ret = ft_node_recompact(FT_RECOMPACT_RELOCATE, ft, type_index,
			&ft_types[type_index], node, meta, holder,
			0, NULL, NULL, NULL, &old_ret, holder == &ft->root,
			node_depth, false, &rec, txn, NULL, ctx);
	if (ret != 0) {
		/*
		 * A node or reservation failure inside the recompact before any
		 * reader-visible store, the copy meeting a peer's parked flip
		 * proxy and abandoning itself, or -- once a coarse spacing puts
		 * this walk on the DLM path -- a member it cannot date or a
		 * lock-set acquire a peer already holds.  Every one of them
		 * returns before the child re-parent sweep, so nothing was
		 * recorded into @rec, no mark is still held and no copy is
		 * unfreed: best-effort, leave the node in place (a bailed
		 * relocation is retried by a later compaction pass).
		 *
		 * ☞ THE CONTENDED CASE IS LATENT, NOT LIVE, AND THIS COMMENT USED
		 * TO STATE OTHERWISE.  cds_ft_compact_step's contract requires
		 * the caller to hold its writer exclusion across the call, so no
		 * peer can park a proxy, hold a lock-set, or move a member under
		 * this walk: the -EAGAIN classes listed above cannot occur.
		 * MEASURED over both suites with fault injection armed -- 1,798
		 * relocations reaching this site, bails ENOMEM 6, -EAGAIN ZERO.
		 * So CDS_FT_COMPACT_OOM is an accurate report TODAY, not a
		 * mislabel.
		 *
		 * It becomes one the moment compact is converted to fine-grained
		 * locking, so both halves are in place AHEAD of that: @bail
		 * carries the errno rather than a bool, and cds_ft_compact_step
		 * reports -EAGAIN as CDS_FT_COMPACT_BUSY instead of _OOM.  A
		 * caller told OOM frees memory; a caller told BUSY just resumes.
		 * cds_ft_fault_compact_countdown is what executes this arm until
		 * a real peer can.
		 */
		if (txn)
			ft_flip_txn_destroy(txn);	/* reserved, unused */
		*bail = ret;		/* -EAGAIN (contended) or -ENOMEM, kept apart */
		return;
	}
	/*
	 * Commit the recorded forward publish (and a compressed parent's SKIP_X
	 * dual) together with the old copy's freeze-on-free tombstone atomically
	 * through the pre-reserved @txn.  This is the moment the relocated node
	 * becomes reader-reachable at *holder and the old copy is frozen dead.
	 */
	/*
	 * Compaction, not yet MW-hardened: ABORT unreachable under its
	 * exclusion -- CHECKED, because there is no recovery from it here and
	 * the silent form is a use-after-free.
	 *
	 * On an abort the forward publish never landed, so *@holder still names
	 * the OLD copy and the free below would reclaim a node that is still
	 * reader-reachable.  Nor can this bail instead: ft_node_recompact has
	 * already re-parented the children onto the fresh copy (the reservation
	 * comment above calls that the point of no return), so leaving the node
	 * in place strands them on a node no slot points at.  The commit MUST
	 * be infallible here, which is exactly what the exclusion buys -- so
	 * ASSERT the claim rather than discard the status, and the day
	 * compaction goes MW this fails loudly instead of freeing a live node.
	 */
	{
		enum urcu_txn_status st = ft_remove_commit_rec(ft, &rec, NULL,
				NULL, txn, false);

		assert(st == URCU_TXN_STATUS_OK);
		(void) st;	/* NDEBUG: behaviour-identical to the old discard */
	}
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
 * Hand a relocation's marks to its commit: @set[0] is RETIRED (its tombstone
 * chains onto the mark at per-node, sits beside the anchor's release above),
 * @set[1] is RELEASED.  The shape ft_node_recompact records for {C, P}.
 */
static inline
void ft_compact_record_terminals(struct ft_flip_txn *t,
		const struct ft_lock_ctx *ctx, const struct ft_dlm_member *set,
		struct cds_ft_metadata *retired)
{
	if (!set[0].held.shared) {
		ft_flip_txn_lock_register_held(t, &set[0].held);
		(void) ft_flip_txn_record_anchor_release(t, &set[0].held,
			retired);
	}
	ft_flip_txn_record_retire_anchored(t, ctx, &set[0].held, retired);
	if (set[1].nf && !set[1].held.shared) {
		ft_flip_txn_lock_register_held(t, &set[1].held);
		ft_flip_txn_record_release_lock(t, set[1].held.lock,
			set[1].held.lock_snap);
	}
}

/* Drop the marks a relocation's acquire took, before any txn owns them. */
static inline
void ft_compact_unlock_set(const struct ft_dlm_member *set, unsigned int nr)
{
	unsigned int i;

	for (i = 0; i < nr; i++)
		if (set[i].nf && !set[i].held.shared)
			ft_meta_lock_release(set[i].held.lock);
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
 * @gp_nf:   the node that CONTAINS @gp_slot -- the dual's owner.  Passed so the
 *           record can NAME it; see the record below for why that is worth
 *           plumbing and why it changes no behaviour.
 * Returns the new compressed node (or @cn unchanged on allocation failure, with
 * *@oom set so the caller can stop the pass).
 */
static
struct cds_ft_compressed_node *ft_compact_relocate_compressed(struct cds_ft *ft,
		struct cds_ft_compressed_node *cn,
		struct cds_ft_inode_flag **gp_slot,
		struct cds_ft_inode_flag *gp_nf,
		unsigned int cn_depth,
		const struct ft_lock_ctx *ctx,
		int *bail)
{
	struct cds_ft_metadata *cn_meta =
		cds_ft_item_to_metadata((struct cds_ft_inode *) cn);
	struct cds_ft_metadata *cn2_meta;
	struct cds_ft_compressed_node *cn2;
	struct cds_ft_inode_flag *cn2_flag;
	enum urcu_txn_status cst;
	/*
	 * THE RELOCATION'S LOCK SET (doc/design/ft-lockset-inventory.md §1,
	 * row 4).  This relocation RETIRES @cn and rewrites the words @cn owns:
	 * its child's back edge and, for a traditional compressed node, the
	 * grandparent slot that names it.  It used to do all of that holding
	 * NOTHING -- the exclusion was the caller's writer scope, a "caller's
	 * responsibility" TODO rather than a contract.  Under the FINE strategy
	 * it now takes the same shape ft_node_recompact takes for {C, P}: @cn
	 * with a RETIRE terminal, the grandparent (traditional only) with a
	 * RELEASE terminal, one all-or-none acquire, and the plan read UNDER the
	 * lock.  A COARSE or EXCLUSIVE trie keeps the unlocked shape below: no
	 * per-node peer exists there.
	 */
	bool fine = ft->lock_fine && !ft->exclusive;
	struct ft_dlm_member set[2];
	uint8_t len;

	memset(set, 0, sizeof(set));
	if (fine) {
		int dret;

		set[0] = (struct ft_dlm_member){
			.nf = ft_compressed_node_flag(cn), .node = cn_meta,
			.depth = cn_depth,
			/* validates @cn is still the grandparent's child */
			.guard_child = gp_slot ? cn_meta : NULL,
			.guard_pf = gp_slot ? gp_nf : NULL };
		if (gp_slot && gp_nf) {
			unsigned int gp_span = ft_node_span(ft, gp_nf);

			if (gp_span > cn_depth) {
				*bail = -EAGAIN;	/* not @cn's parent: stale */
				return cn;
			}
			set[1] = (struct ft_dlm_member){
				.nf = gp_nf,
				.node = ft_flag_to_metadata(ft, gp_nf),
				.depth = cn_depth - gp_span };
		}
		dret = ft_dlm_acquire_set(ft, ctx, set, 2);
		if (dret) {
			*bail = dret == -ENOMEM ? -ENOMEM : -EAGAIN;
			return cn;
		}
		/*
		 * RE-VALIDATE under the lock: the walk sampled the grandparent
		 * slot with nothing held.  @cn itself cannot be retired or split
		 * any more, and re-homing its child needs @cn's lock.
		 */
		if (gp_slot) {
			struct cds_ft_inode_flag *cur = rcu_dereference(*gp_slot);

			if (ft_node_flip_proxy(cur) || !ft_node_compressed(cur) ||
					ft_compressed_node_ptr(cur) != cn) {
				ft_compact_unlock_set(set, 2);
				*bail = -EAGAIN;
				return cn;
			}
		}
	}
	len = cn->len;
	cn2 = alloc_compressed_node(ft, len, &cn2_meta);
	if (!cn2) {
		ft_compact_unlock_set(set, 2);
		*bail = -ENOMEM;
		return cn;	/* OOM: leave in place (best-effort) */
	}
	cn2->len = len;
	cn2->child = cn->child;
	memcpy(cn2->key_bytes, cn->key_bytes, len);
	cn2_meta->parent_word = cn_meta->parent_word;
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
		/*
		 * + (fine) @cn's anchor release, its retire, and the
		 * grandparent's release.
		 */
		struct ft_flip_txn *t = ft_flip_txn_create_bounded(ft,
			fine ? 6 : 3);

		if (!t) {
			free_compressed_node_unpublished(ft, cn2);
			ft_compact_unlock_set(set, 2);
			*bail = -ENOMEM;
			return cn;
		}
		/*
		 * ☞ THE TERMINALS FIRST: they register @cn's mark, and the child
		 * re-parent just below writes @cn's own words (FT-SLOT-3), so its
		 * owner has to be in the registry when it is made -- the rule the
		 * slot record states further down.  Recorded after, every such
		 * back edge committed as a CAS (FT_DEBUG_MW_KEPT, HEAD_BACK /
		 * PARENT_WORD).  Order within one txn changes nothing else: the
		 * terminals write @cn's and the grandparent's state words, the
		 * re-parent the child's.
		 */
		if (fine)
			ft_compact_record_terminals(t, ctx, set, cn_meta);
		ft_reparent_record(ft, t, cn2->child, cn2_flag, &cn2->child,
			/*child_marked=*/ false, /*hold_ctx=*/ NULL,
			/*check_child=*/ false);	/* span-preserving */
		/*
		 * ☑ THE DUAL'S OWNER IS NAMED NOW.  It is the GRANDPARENT node
		 * that contains @gp_slot, and this helper used to be handed the
		 * bare slot, so the meta was not in scope and the record said
		 * FT_OWNER_UNPLUMBED -- "plumbing, not a missing lock", since
		 * the compaction pass runs under whole-trie exclusion.  The
		 * caller has that node in hand (it is the one it read the slot
		 * out of), so it passes it.
		 *
		 * ☠ AND THIS CHANGES NO BEHAVIOUR, deliberately.  The record's
		 * KIND is decided by @t->structural_sw, never by @owner, so
		 * naming the owner cannot park anything SW -- which matters,
		 * because the SKIP_X dual must stay MW until EVERY producer can
		 * vouch (one lane parking SW while another CASes the same slot
		 * is the cross-thread kind disagreement the engine cannot
		 * check).  FT_OWNER_ASSERT_OWNED is likewise inert here: it is
		 * guarded on dbg_arm_per_op && nr_locks, and this bounded txn is
		 * neither armed nor lock-carrying.  What the name buys is that
		 * the edge stops counting as UNPLUMBED -- one of the two
		 * producers note (1) of the word-kind table names as blocking
		 * the dual's conversion.
		 */
		/*
		 * Under the lock, hand the marks to @t BEFORE the slot record so
		 * the record's owner is in the registry when it is made, and read
		 * the slot's expected-old THROUGH @t (a raw re-read could land on
		 * a peer's parked proxy).  (The marks were handed over above,
		 * before the child re-parent.)
		 */
		ft_flip_txn_record_reserved(t,
			gp_nf ? ft_flag_to_metadata(ft, gp_nf) :
				FT_OWNER_UNPLUMBED,
			(void **) gp_slot,
			fine ? ft_txn_load(t->mtxn, (void **) gp_slot,
				FT_FLIP_PROXY_TAG) : *gp_slot,
			cn2_flag);
		cst = ft_flip_txn_commit(ft, t);
		if (cst != URCU_TXN_STATUS_OK) {
			free_compressed_node_unpublished(ft, cn2);
			/* An ABORT is a peer, not memory pressure. */
			*bail = cst == URCU_TXN_STATUS_ABORT ? -EAGAIN : -ENOMEM;
			return cn;
		}
	} else if (fine) {
		/*
		 * Skip form under the lock: the child's back-reference redirect
		 * is still the publish, but it rides a txn WITH @cn's retire, so
		 * the relocation and the old copy's tombstone flip together and
		 * the lock's release is the commit's.
		 */
		struct ft_flip_txn *t = ft_flip_txn_create_bounded(ft, 5);

		if (!t) {
			free_compressed_node_unpublished(ft, cn2);
			ft_compact_unlock_set(set, 2);
			*bail = -ENOMEM;
			return cn;
		}
		/* The terminals first, for the reason the arm above gives. */
		ft_compact_record_terminals(t, ctx, set, cn_meta);
		ft_reparent_record(ft, t, cn2->child, cn2_flag, &cn2->child,
			/*child_marked=*/ false, /*hold_ctx=*/ NULL,
			/*check_child=*/ false);	/* span-preserving */
		cst = ft_flip_txn_commit(ft, t);
		if (cst != URCU_TXN_STATUS_OK) {
			free_compressed_node_unpublished(ft, cn2);
			*bail = cst == URCU_TXN_STATUS_ABORT ? -EAGAIN : -ENOMEM;
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
	if (!fine)	/* fine: the commit's retire record already did */
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
	/* cds_ft_free_item_deferred owns the balance (fractal-trie-alloc.c). */
	cds_ft_free_item_deferred(ft, cn_meta);
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
		struct ft_ord_cell *old, struct cds_ft_inode_flag *holder_nf,
		unsigned int holder_depth, const struct ft_lock_ctx *ctx,
		int *bail)
{
	struct cds_ft_metadata *meta;
	struct cds_ft_node *head = old->node;
	struct ft_ord_cell *new_cell;
	int sret;
	/*
	 * THE CELL'S LOCK (doc/design/ft-lockset-inventory.md §1, row 4).  The
	 * relocation rewrites the head's back edge (@head->prev), whose owner is
	 * the chain's HOLDER -- the node whose slot or @external_nodes names
	 * @head -- and it used to do so holding nothing.  Under the FINE strategy
	 * take the holder first, re-read the plan under it, and commit the list
	 * replace and the back edge in ONE txn with the holder's release.  The
	 * list edges themselves stay the [DESIGN] MW lane.  A COARSE or EXCLUSIVE
	 * trie keeps the unlocked two-store shape below.
	 */
	bool fine = ft->lock_fine && !ft->exclusive;
	struct ft_held_anchor h = { 0 };
	void *prev_old = NULL;

	if (fine) {
		int aret;

		if (!holder_nf || !head) {
			*bail = -EAGAIN;
			return old;
		}
		aret = ft_acquire_member(ft, ctx, holder_nf,
			ft_flag_to_metadata(ft, holder_nf), holder_depth, &h);
		if (aret) {
			*bail = aret == -ENOMEM ? -ENOMEM : -EAGAIN;
			return old;
		}
		/*
		 * RE-VALIDATE under the lock: the cell still carries @head, @head
		 * still names the cell, and the cell still names @holder_nf.  A
		 * peer that re-parented or replaced the chain in between is a
		 * -EAGAIN the step reports as BUSY, never an OOM.
		 */
		/*
		 * @old->parent names the holder WITH the prefix-head bit when
		 * @head is its @external_nodes (ft_head_parent_word), so strip it
		 * before comparing -- ft_node_ptr_raw keeps bit 4 on a type-0
		 * internal flag, and an unstripped compare refuses the right
		 * holder on every attempt, a BUSY the step would resume forever.
		 * A parked proxy on it is a peer's re-home in flight: stale.
		 */
		prev_old = rcu_dereference(head->prev);
		if (old->node != head || ft_node_flip_proxy(prev_old) ||
				ft_ord_cell_ptr(prev_old) != old ||
				ft_node_flip_proxy(old->parent) ||
				ft_node_ptr_raw(ft_parent_prefix_strip(
					ft_parent_node(old->parent))) !=
				ft_node_ptr_raw(holder_nf)) {
			if (!h.shared)
				ft_meta_lock_release(h.lock);
			*bail = -EAGAIN;
			return old;
		}
	}
	meta = cds_ft_alloc_cell_item(ft);
	if (!meta) {
		if (fine && !h.shared)
			ft_meta_lock_release(h.lock);
		*bail = -ENOMEM;
		return old;		/* OOM: best-effort, leave in place */
	}
	if (ft_debug_counters())
		uatomic_inc(&ft->group->nr_cells_allocated);
	new_cell = (struct ft_ord_cell *) cds_ft_metadata_to_item(meta);
	new_cell->node = head;
	new_cell->parent = old->parent;
	/* Carry the head's edge byte across the relocation (up-walk key source). */
	meta->incoming_byte = cds_ft_item_to_metadata(old)->incoming_byte;
	if (fine) {
		/*
		 * ONE commit under the holder: the list replace, the back edge
		 * and the holder's release.  The two-store shape below is legal
		 * because its window is observationally empty (see there), but
		 * under the lock the back edge is a word of the locked holder and
		 * belongs in the lock's commit like every other one.  An ABORT
		 * installed nothing and released the lock with it.
		 */
		struct ft_flip_txn *t = ft_flip_txn_create_bounded(ft,
			FT_ORD_CELL_SWAP_REC_MAX_EDGES +
			1 /* @head->prev */ + 1 /* the holder's release */);
		enum urcu_txn_status st;

		if (!t) {
			if (ft_debug_counters())
				uatomic_inc(&ft->group->nr_cells_freed);
			cds_ft_free_item_unpublished(ft, meta);
			if (!h.shared)
				ft_meta_lock_release(h.lock);
			*bail = -ENOMEM;
			return old;
		}
		if (!h.shared) {
			ft_flip_txn_lock_register_held(t, &h);
			ft_flip_txn_record_release_lock(t, h.lock, h.lock_snap);
		}
		/*
		 * ord_prev / ord_next are set from @old's neighbours by the
		 * replace.  ☠ ITS STATUS IS NOT OPTIONAL: -ENOENT (@old already
		 * deleted) and -EAGAIN (a neighbour mid-delete) record NOTHING
		 * for the list, and committing the back edge alone would point
		 * @head->prev at a cell that is not in the list.
		 */
		/* Two cell words, recorded without asking who owns them. */
		/*
		 * ☞ STAGE 2: the FT's recorder, so the two neighbour edges get
		 * an owner (see ft_ord_cell_swap).  @old's own cell lock is
		 * already registered just above -- this take adds its pred and
		 * succ, whose words the same edges write.
		 */
		{
			struct ft_cell_plan plan = {
				.cell = old,
				/*
				 * ☠ @h IS THE HOLDER'S ANCHOR, NOT @old'S CELL
				 * LOCK.  The acquire above takes @holder_nf's
				 * metadata -- a trie node -- so @old's own cell
				 * word is NOT held by it and this plan must take
				 * it.  Reading @h as if it covered the cell left
				 * exactly one of the swap's three records unheld,
				 * measured as 310,164 against 620,328 held: the
				 * 2:1 that named which record it was.
				 */
				.cell_lock_held = false,
				.pred = ft_ord_cell_resolve_ord(
					&old->lnode.prev),
				.succ = ft_ord_cell_resolve_ord(
					&old->lnode.next),
			};
			struct ft_ord_cell_edge edges[4] = { 0 };
			unsigned int n;

			sret = ft_cell_lockset_take(ft, ctx, t, &plan);
			if (!sret) {
				n = ft_ord_cell_swap_edges(ft, old, new_cell,
					edges, 0);
				ft_ord_cell_record_into_ft(ft, t, edges, n);
			}
		}
		if (sret) {
			ft_flip_txn_destroy(t);	/* releases the registered lock */
			if (ft_debug_counters())
				uatomic_inc(&ft->group->nr_cells_freed);
			cds_ft_free_item_unpublished(ft, meta);
			*bail = -EAGAIN;
			return old;
		}
		/*
		 * ☞ NAMED BY ITS HOLDER.  The word belongs to the chain holder,
		 * which @h took above, and @prev_old was read and re-validated
		 * under that lock (it still names @old, no parked proxy).  So it
		 * is the owned record: the per-record gate parks it when this txn
		 * owns the holder, and a SHARED take (held through another frame,
		 * not registered here) keeps it MW as before.  Unowned, it was
		 * FT_DEBUG_MW_KEPT's largest ft_unit population (HEAD_BACK,
		 * ~620k per run).  -DFT_DEBUG_COMPACT_CELL_UNOWNED keeps it MW.
		 */
#ifndef FT_DEBUG_COMPACT_CELL_UNOWNED
		ft_flip_txn_record_head_back_edge_owned(t, (void **) &head->prev,
			prev_old, (void *) ft_ord_cell_flag(new_cell),
			ft_flag_to_metadata(ft, holder_nf)
			FT_BE_SITE(FT_BE_COMPACT_CELL, ctx));
#else
		ft_flip_txn_record_head_back_edge(t, (void **) &head->prev,
			prev_old, (void *) ft_ord_cell_flag(new_cell)
			FT_BE_SITE(FT_BE_COMPACT_CELL, ctx));
#endif
		st = ft_flip_txn_commit(ft, t);
		if (st != URCU_TXN_STATUS_OK) {
			if (ft_debug_counters())
				uatomic_inc(&ft->group->nr_cells_freed);
			cds_ft_free_item_unpublished(ft, meta);
			*bail = st == URCU_TXN_STATUS_ABORT ? -EAGAIN : -ENOMEM;
			return old;
		}
		goto relocated;
	}
	/* ord_prev / ord_next are set from @old's neighbours by the swap. */
	sret = ft_ord_cell_swap(ft, old, new_cell);
	if (sret != 0) {
		/*
		 * The swap installed nothing -- a reservation OOM (-ENOMEM), or
		 * a prepare refused / a commit aborted by a peer (-EAGAIN) --
		 * so @old stays fully in the ordered list.
		 * Discard @new_cell (never published -- no reader can reach it)
		 * and leave @old in place, the same best-effort contract as the
		 * cell-allocation failure above.
		 */
		if (ft_debug_counters())
			uatomic_inc(&ft->group->nr_cells_freed);
		cds_ft_free_item_unpublished(ft, meta);
		/* An aborted swap is a peer; a failed reservation is memory. */
		*bail = sret == -EAGAIN ? -EAGAIN : -ENOMEM;
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

		/*
		 * The [debt] HEAD-WORD class again, and this arm -- the
		 * COARSE / EXCLUSIVE lone-edge flip, which the FINE path above
		 * replaces with a recorded back edge under the holder -- had NO
		 * audit arm, so compaction could not appear in the class's zeros
		 * either way.  @holder_nf is the owner the fine path locks.
		 */
		ft_ch_audit_head(ft, head, holder_nf);
		ft_ord_cell_flip_one(&edge);
	}
relocated:
	/* Always-deferred free: see ft_compact_relocate_at. */
	/* cds_ft_free_item_deferred owns the balance (fractal-trie-alloc.c). */
	cds_ft_free_item_deferred(ft, cds_ft_item_to_metadata(old));
	return new_cell;
}

/*
 * The chain the step's lookup landed on, reached by the walk: relocate its
 * ordered cell unless this pass already moved it (its range is
 * recompact_private), mirroring the node descent so a re-visit does not
 * re-allocate.
 */
static inline
void ft_compact_cell_at(struct cds_ft *ft, struct cds_ft_node *cell_head,
		struct cds_ft_inode_flag *holder_nf, unsigned int holder_depth,
		const struct ft_lock_ctx *ctx, unsigned long *relocated,
		int *bail)
{
	void *prev = rcu_dereference(cell_head->prev);
	struct ft_ord_cell *cell;

	if (ft_node_flip_proxy(prev))
		return;		/* a peer's commit in flight on the head: skip */
	cell = ft_ord_cell_ptr(prev);
	if (cds_ft_metadata_in_recompact_private(
			cds_ft_item_to_metadata(cell)))
		return;
	ft_compact_relocate_cell(ft, cell, holder_nf, holder_depth, ctx, bail);
	(*relocated)++;
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
 *
 * The walk carries a real struct ft_descent, extended one node at a time with
 * ft_walk_extend: a relocation's acquire anchors its {C, P, GP} lock-set
 * through ft_anchor_meta, which under a coarse spacing resolves the anchor from
 * the descent's level table (doc/design/ft-dlm-lock-coarseness.md §9).  A byte
 * depth alone is not enough -- the one-hop rule DATES a member without a
 * descent, but only the table can name the boundary node its anchor IS -- so
 * this walk enters every node it passes, exactly as the read descent does, and
 * with the PLAIN flag of the node it enters (a skip-compressed node is entered
 * by its compressed node's own flag -- the relocated one -- not by the skip
 * word: a skip word in the table defeats ft_anchor_meta's `anchor == nf`
 * identity and names the elided node, the ft_node_recompact row of
 * doc/design/ft-lockset-inventory.md §2).
 *
 * ☞ THE CURSOR IS KEPT ON THE NODE AT @d.depth.  ft_walk_extend leaves @d.nf on
 * the node it ENTERED while @d.depth moves past it, and the table's pending-
 * level fallback answers with @d.nf -- the member's PARENT -- for a member that
 * starts exactly at a still-pending boundary (the ft_detach_orphan_planlock row
 * of the same section).  Every acquire this walk makes names the node at
 * @d.depth, so the walk points the cursor at it before asking.
 *
 * @cell_head, when set, is the chain the step's lookup landed on: its ordered
 * cell is relocated from HERE (ft_compact_cell_at), the one frame that knows the
 * chain's holder and the holder's depth, which is what the cell's lock is taken
 * on.
 */
static
void ft_compact_descend(struct cds_ft *ft, const uint8_t *key,
		size_t key_len, struct cds_ft_node *cell_head,
		unsigned long *relocated, int *bail, struct ft_op *op)
{
	const struct cds_ft_key_map *km = &ft->group->key_map;
	struct cds_ft_inode_flag **holder = &ft->root;
	/* The node containing *@holder (NULL at the root slot), and its start. */
	struct cds_ft_inode_flag *holder_owner = NULL;
	unsigned int holder_owner_depth = 0;
	struct ft_descent d;
	struct ft_lock_ctx ctx;
	size_t depth = 0;

	/*
	 * No outer held set: each relocation acquires, commits and releases its
	 * own lock-set, so the only dedupe that matters is the intra-set one
	 * ft_dlm_acquire_set already does.
	 */
	ft_descent_init(&d, ft);
	/*
	 * @op has no retry loop behind it -- a refused acquire here reaches the
	 * caller as -EAGAIN -- so it is carried for DEFERENCE, not aging: the
	 * caller's begin() honours domain->active, and a peer starving on one of
	 * these anchors is queued in front of this pass rather than barged past.
	 * Nothing under this descent waits for a grace period, so holding a lane
	 * turn across it cannot stall the readers it would then wait on.
	 */
	ft_lock_ctx_init(&ctx, &d, NULL, op);

	for (;;) {
		struct cds_ft_inode_flag *nf = rcu_dereference(*holder);
		struct cds_ft_inode_flag **child_slot = NULL;
		struct cds_ft_inode_flag *raw;
		uint8_t ord;

		if (ft_node_external(nf)) {
			/* reached a leaf: is it the chain the lookup landed on? */
			if (cell_head && holder_owner &&
					ft_node_ptr(nf) ==
						(struct cds_ft_inode *) cell_head)
				ft_compact_cell_at(ft, cell_head, holder_owner,
					holder_owner_depth, &ctx, relocated,
					bail);
			return;
		}
		d.nf = nf;	/* the cursor names the node at @d.depth (above) */
		if (!cds_ft_metadata_in_recompact_private(
				cds_ft_item_to_metadata(ft_node_ptr(nf)))) {
			ft_compact_relocate_at(ft, holder, (unsigned int) depth,
				&ctx, bail);
			(*relocated)++;
			if (*bail)
				return;		/* memory pressure: stop the descent */
			nf = rcu_dereference(*holder);	/* the relocated node */
		}
		/*
		 * Enter the node -- the RELOCATED one, so the table names a live
		 * boundary -- before descending past it, so its children anchor
		 * on the levels its span covers.  One key byte: *@holder is
		 * always a plain internal node (a compressed run arrives through
		 * the arms below, which enter it over its own length), which is
		 * the same one byte the depth++ below consumes.
		 */
		(void) ft_walk_extend(&d, true, nf, (unsigned int) depth, 1);
		if (depth >= key_len) {
			/* consumed the whole key: a PREFIX head, held by @nf */
			if (cell_head && cds_ft_item_to_metadata(
					ft_node_ptr(nf))->external_nodes ==
						cell_head)
				ft_compact_cell_at(ft, cell_head, nf,
					(unsigned int) depth, &ctx, relocated,
					bail);
			return;
		}
		ord = key_to_ordinal(key[depth], km);
		raw = ft_node_get_nth_skip(nf, &child_slot, ord, FT_PF_NONE);
		if (!raw)
			return;		/* child absent (e.g. concurrent removal) */
		holder_owner = nf;
		holder_owner_depth = (unsigned int) depth;
		depth++;		/* child-index byte (matches iter_key = *key++) */
		if (ft_node_skip_compressed(raw)) {
			struct cds_ft_compressed_node *cn =
				ft_skip_to_compressed(ft, raw);
			unsigned int span = ft_skip_len(raw);

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
			d.nf = ft_compressed_node_flag(cn);	/* the node at @d.depth */
			if (!cds_ft_metadata_in_recompact_private(
					cds_ft_item_to_metadata((struct cds_ft_inode *) cn))) {
				cn = ft_compact_relocate_compressed(ft, cn, NULL,
					NULL, (unsigned int) depth, &ctx, bail);
				(*relocated)++;
				if (*bail)
					return;		/* memory pressure: stop the descent */
			}
			/*
			 * Enter the LIVE compressed node by its own flag.  The skip
			 * word would also resolve to it (through the child's
			 * back-pointer), but a skip word in the table is what makes
			 * a later anchor query name the elided node instead of a
			 * member (see the header).
			 */
			holder_owner = ft_compressed_node_flag(cn);
			holder_owner_depth = (unsigned int) depth;
			depth = ft_walk_extend(&d, true, ft_compressed_node_flag(cn),
				(unsigned int) depth, span);
			holder = &cn->child;
		} else if (ft_node_compressed(raw)) {
			struct cds_ft_compressed_node *cn = ft_compressed_node_ptr(raw);
			unsigned int span = cn->len;

			d.nf = raw;	/* the node at @d.depth */
			/* Traditional: the grandparent slot (child_slot) holds the cn flag. */
			if (!cds_ft_metadata_in_recompact_private(
					cds_ft_item_to_metadata((struct cds_ft_inode *) cn))) {
				cn = ft_compact_relocate_compressed(ft, cn, child_slot,
					nf, (unsigned int) depth, &ctx, bail);
				(*relocated)++;
				if (*bail)
					return;		/* memory pressure: stop the descent */
			}
			holder_owner = ft_compressed_node_flag(cn);
			holder_owner_depth = (unsigned int) depth;
			/* The grandparent slot now holds the relocated node's flag. */
			depth = ft_walk_extend(&d, true, ft_compressed_node_flag(cn),
				(unsigned int) depth, span);
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
	int bail;		/* 0, or the errno that stopped the last step */
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
	struct ft_op optxn;

	ft_txn_op_init(ft, &optxn);
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
	resume_inclusive = st->bail != 0;
	st->bail = 0;

	/*
	 * Route this step's relocations into private ranges, and hold the
	 * RCU read lock for the whole batch: it keeps the iterator's reads
	 * and our descents safe, and defers our own call_rcu node frees until
	 * the read-unlock between steps (where the drained ranges reclaim and
	 * concurrent mutations get their window).
	 */
	ft_recompact_alloc_set_active(&st->ctx);
	flavor->read_lock();
	/*
	 * ☞ THE MODE FLIP, PER STEP.  A fine trie's point op samples the bulk
	 * gate once per op, from inside a read-side section, so a bulk op's
	 * publish-then-one-GP drains every op that saw it clear, and every later
	 * one queues on the FT-wide lock (ft_writer_lock_scope_enter, G5.25).
	 * Compaction follows the same protocol one STEP at a time: the scope is
	 * entered inside this step's read-side section, so the gate's grace
	 * period covers the batch, and a bulk op that starts between two steps
	 * makes the next one queue.  Sampling once for a whole cds_ft_compact
	 * pass would not: the pass drops the read lock between steps, the gate's
	 * grace period completes in that gap, and the bulk body would run beside
	 * a compaction that holds no FT-wide lock -- which is what lets bulk ops
	 * take no node lock of their own.
	 *
	 * The step's DLM locks (the relocations' lock sets) stay: in bulk mode a
	 * step takes BOTH, like a point op, because at the gate's two edges it
	 * meets ops that hold DLM locks only.
	 */
	{
	CDS_FT_SCOPED_WRITER(ft);

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
		ft_op_begin(&optxn);
		ft_compact_descend(ft, key, key_len,
				ft->group->ordered_list_set ? st->iter->node : NULL,
				&relocated, &st->bail, &optxn);
		ft_op_end(&optxn);
		/*
		 * ☞ THE CELL IS NOW RELOCATED INSIDE THE WALK (ft_compact_descend),
		 * the frame that has the chain's HOLDER and its depth, which is what
		 * the cell's lock is taken on.  The rest of this note describes that
		 * relocation.
		 *
		 * Relocate this key's cell into a dense private cell range, in the
		 * same key order the iterator visits -- so the ordered cell list
		 * becomes a near-sequential scan.  iter->node is the chain head;
		 * its cell is head->prev.  Skip a cell already moved this pass
		 * (its range is recompact_private), mirroring the node descent, so
		 * the pass is idempotent and re-visits do not re-allocate.  Skip it
		 * too when the descent stopped on OOM: the head node may be un-
		 * relocated, and the resume re-attempts this whole key anyway.
		 */
		if (st->bail)
			break;		/* stop the pass; the resume re-attempts */
	}
	}	/* the step's writer scope */
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
	 * A bail takes precedence (the caller resumes from the interrupted
	 * key); otherwise report completion or that more remains.  -EAGAIN and
	 * -ENOMEM are reported APART: a caller told OOM frees memory, which is
	 * the wrong remedy for a peer that merely held a lock-set.
	 */
	if (st->bail)
		return st->bail == -EAGAIN ? CDS_FT_COMPACT_BUSY :
			CDS_FT_COMPACT_OOM;
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
	return s;	/* DONE, or OOM / BUSY if a step stopped the pass */
}
