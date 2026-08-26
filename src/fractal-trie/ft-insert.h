// SPDX-FileCopyrightText: 2012-2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-only

/*
 * src/fractal-trie/ft-insert.h
 *
 * Userspace RCU library - Fractal Trie: the key-insert path -- descent to the
 * attach point and ft_attach_node, the one-commit ordered-list publish
 * (struct ft_insert_commit / ft_insert_one_commit) and the compressed-path
 * split.  The flip-batch, node-reserve and node-cluster builders it uses now
 * live in their own shared modules (ft-mutation-helpers.h, ft-cluster-build.h).
 *
 * Implementation unit: #included once by fractal-trie.c, in dependency
 * order, into a single translation unit (preserves cross-module inlining).
 * Not a standalone header.
 */
#ifndef FRACTAL_TRIE_IMPL
#error "ft-insert.h is an implementation unit; #include it from fractal-trie.c only"
#endif

/*
 * One-commit insert state (ordered-list fresh-head insert): the attach
 * machinery parks a flip proxy in the structural slot (resolving to the OLD
 * value, so the key stays invisible) and records the settle information here;
 * insert_done then adds the ordered-list neighbour edges to the SAME batch and
 * makes the key reachable in the structural index AND spliced into the cell
 * list with one MCAS flip commit -- a reader can never observe the fresh head
 * without its cell in the list (2026-06 review, 2.13).  @batch == NULL: direct
 * publish (ordered list off, or a shape not yet converted).
 */
struct ft_insert_commit {
	struct ft_flip_txn *txn;		/* armed at the publish site */
	/*
	 * The op's PERSISTENT engine handle (ft_txn_op_init'd before the op's
	 * restart_attempt loop, doc §11): the arm binds @txn to it, so retry
	 * aging, the FIFO escalation turn, and the learned descriptor size span
	 * the whole retry loop.  NULL for an op not yet migrated to the
	 * persistent handle (_cds_ft_insert_replace) -- the arm then creates a
	 * standalone per-attempt txn as before.
	 */
	struct urcu_txn *op;
	struct cds_ft_inode_flag **slot;	/* forward-publish sentinel (one-commit
						 * parked) -- the txn settles the edges */
	/*
	 * Count-propagation base for the post-commit +1 (the key only counts
	 * once reachable): the deepest node whose nr_keys must reflect the
	 * fresh key.  NULL = the caller's *d.pnfp.
	 */
	struct cds_ft_inode_flag *count_from;
	/*
	 * Order-statistics count fold (rank stats ON): when set, the +1 key-count
	 * propagation is recorded as nr_keys value-CAS edges on the STABLE
	 * ancestor chain from @count_from to the root INTO @txn (folded atomically
	 * with the structural publish), and insert_done skips the standalone
	 * post-commit root-ward count walk.  A shape opts in
	 * only once its @count_from is the stable base and every fresh node on the
	 * new key's path was built with its full post-commit count.  Requires the
	 * arm to have reserved the extra per-ancestor edges (actual descent depth).
	 */
	bool count_folded;
	/*
	 * Old compressed node replaced by the parked publish: readers keep
	 * resolving the proxy to it until the commit, so its (grace-period-
	 * deferred) free must be queued only AFTER the commit -- a free queued
	 * pre-commit would not cover readers that pick the proxy up later.
	 *
	 * @free_old_cn_held: the lock-set member the split builder acquired at
	 * ENTRY for this cn (F2 fence extended to the split-retire family) -- the
	 * word actually CAS'd plus the clean snapshots of it and of the cn.  The
	 * builder's whole plan -- diverge slicing, cn->child snapshot,
	 * deferred-edge captures -- derives from the fenced cn, and
	 * ft_insert_one_commit records the retire from this member
	 * (ft_flip_txn_record_retire_anchored), so a peer state change under the
	 * fence (a concurrent split/collapse/re-home of the SAME cn) aborts
	 * exactly one side instead of being erased by a ratified stale plan.
	 */
	struct ft_held_anchor free_old_cn_held;
	struct cds_ft_compressed_node *free_old_cn;
	/*
	 * Old internal node replaced by a recompact-relocation forward publish
	 * folded into the one-commit (ft_attach_node): the parked grandparent
	 * proxy resolves to it until the commit flips, so its grace-period-
	 * deferred free must be queued only AFTER the commit, like free_old_cn.
	 * NULL = no relocation (in-place reserve).
	 */
	struct cds_ft_inode *free_old_node;
	/*
	 * Deferred LIVE re-parent edge (split-compressed one-commit).  The live
	 * old child's back-pointer is the back-channel publish that exposes the
	 * fresh cluster to reanchor up-walkers; for a parked commit the forward
	 * publish IS the single commit, so this edge must be wired THERE (after
	 * the cell links are set), not during the build -- otherwise a reader
	 * reanchoring through the re-parented child reaches the fresh head before
	 * its cell is spliced (the inv_insert_splice_window race).  NULL = none.
	 */
	struct cds_ft_inode_flag *live_child;
	struct cds_ft_inode_flag *live_parent;
	struct cds_ft_inode_flag **live_slot;
	bool publish_to_parent;			/* settle via ft_publish_to_parent */
	/*
	 * Concurrent-writer commit-outcome scope (see the deferred-action model
	 * in <urcu/rcu-txn.h>).  @ft anchors the finalize/rollback callbacks;
	 * @created[0..nr_created) are this attempt's fresh, still-unpublished
	 * cluster nodes -- freed by the on-abort rollback when a peer writer wins
	 * the commit (or by the build's own error path pre-commit).  The
	 * on-commit finalize frees the retired free_old_* above.
	 */
	struct cds_ft *ft;
	struct cds_ft_inode_flag *created[FT_MAX_DEPTH];
	int nr_created;
	/*
	 * DLM Step 1 compressed-split acquire (see
	 * doc/design/mw-writer-lock-escalation-model.md): when a
	 * split builder pre-acquired CN's parent P (the value-swap forward-publish
	 * target) as part of its one-commit lock-set {CN, P}, @parent_locked_holder
	 * is P's metadata and @parent_locked_snap the clean state word captured at
	 * the acquire.  ft_insert_publish_or_park then records P's RELEASE terminal
	 * ({LOCK|snap -> snap}) from that held snap via
	 * ft_flip_txn_hold_or_lock_parent -- instead of re-marking P -- and registers
	 * it into @txn (whose commit/abort then owns P's fence).  NULL: no pre-
	 * acquire (past-child {CN}, non-split shapes) -> publish_or_park takes the
	 * incremental lock_or_guard.  A pre-publish bail releases it explicitly.
	 *
	 * @parent_lock_shared distinguishes the THIRD state a NULL holder would
	 * otherwise swallow: P was a lock-set member, but coarsening mapped it onto
	 * the SAME anchor as CN, so the op holds P's lock already and the release
	 * was recorded once, with CN's.  publish_or_park must then only guard --
	 * acquiring would fail against the op's OWN hold and self-abort, and a
	 * second release would double-record the one word.  Never set at per-node
	 * granularity, where two distinct nodes cannot share a word.
	 */
	struct cds_ft_metadata *parent_locked_holder;
	uintptr_t parent_locked_snap;
	bool parent_lock_shared;
};

static void ft_free_unpublished_split_cluster(struct cds_ft *ft,
		struct cds_ft_inode_flag *const *created, int nr_created);

/*
 * on-commit finalize: the commit installed, so the nodes this op retired are
 * now unreachable -- queue their grace-period-deferred free.  Registered into
 * the txn by the prepare; the engine runs it iff the commit succeeds.
 */
static
void ft_insert_finalize_cb(void *arg)
{
	struct ft_insert_commit *ic = arg;

	if (ic->free_old_cn)
		free_compressed_node(ic->ft, ic->free_old_cn);
	if (ic->free_old_node)
		free_cds_ft_node(ic->ft, ic->free_old_node);
}

/*
 * on-abort rollback: a peer writer won the commit (or it hit ENOMEM), so
 * NOTHING was installed -- the fresh cluster this attempt built was never
 * reader-reachable.  Free it (immediate, no grace period needed) so the
 * re-descend starts clean.  The retire targets stay live and untouched; the
 * caller resets @node's list linkage before retrying.
 */
static
void ft_insert_abort_cb(void *arg)
{
	struct ft_insert_commit *ic = arg;

	ft_free_unpublished_split_cluster(ic->ft, ic->created, ic->nr_created);
}

/*
 * Park the deferred LIVE re-parent edge (split-compressed one-commit) into
 * @batch, so the live old child's back-pointer flips to its new cluster parent
 * ATOMICALLY with the forward structural edge and the ordered-list neighbour
 * edges at the single MCAS flip commit.  That back-pointer is the only field a
 * reanchor up-walk reads to recover a skip-compressed node (ft_skip_reanchor,
 * which resolves flip proxies on the parent read), so until the commit a reader
 * resolves it to the OLD parent and never enters the fresh cluster -- closing
 * the window where the head was tree-reachable (via the re-parented child) but
 * not yet in the ordered list.
 *
 * For a metadata-bearing child the (parent, slot-offset) bookkeeping rides
 * @txn as a CO-COMMITTED pair (ft_reparent_record_meta): parent pointer via a
 * structural edge, offset via an FT_STATE_PROXY state edge -- so a commit
 * ABORT discards BOTH and the live child never carries a torn (old parent,
 * new-cluster offset) pair.  incoming_byte stays a plain same-value store
 * (key-invariant re-home).  External heads carry only the parent edge.
 */
static
void ft_park_live_parent_edge(struct cds_ft *ft,
		struct cds_ft_inode_flag *child,
		struct cds_ft_inode_flag *new_parent,
		struct cds_ft_inode_flag **slot,
		struct ft_flip_txn *txn)
{
	struct cds_ft_metadata *meta = NULL;
	struct cds_ft_inode_flag **field;

	/*
	 * @child was captured from a live slot (e.g. cn->child) a peer may be
	 * mid-flip on (Phase 4.3): resolve a transient type-7 flip proxy to its
	 * committed-or-old target before the kind dispatch below, else a 0xF-
	 * tagged proxy falls through to the internal branch and item_to_metadata
	 * faults on the proxy latch.  A conflicting peer commit is caught later
	 * by this txn's MCAS expected-old check.
	 */
	child = ft_resolve_flip_proxy(child);
#ifdef FEATURE_FT_SKIP_COMPRESSED
	if (ft_node_skip_compressed(child))
		meta = cds_ft_item_to_metadata((struct cds_ft_inode *)
			ft_skip_to_compressed(ft, child));
	else
#endif
	if (ft_node_compressed(child))
		meta = cds_ft_item_to_metadata((struct cds_ft_inode *)
			ft_compressed_node_ptr(child));
	else if (!ft_node_external(child))
		meta = cds_ft_item_to_metadata(ft_node_ptr(child));

	if (meta) {
		/*
		 * Metadata-bearing child: the (parent, slot-offset) pair rides
		 * @txn CO-COMMITTED (ft_reparent_record_meta) instead of an
		 * eager ft_set_parent_slot + parent-only record.  The eager
		 * offset store was unobservable in flight (readers gated at the
		 * OLD parent, always compressed here, skip incoming_byte) but
		 * SURVIVED a commit ABORT: the discarded parent record left the
		 * live child with (old parent, new-cluster offset) -- a torn
		 * pair feeding ft_resolve_parent_slot / the split (parent, slot)
		 * guards after the fresh cluster was freed, under the MW retry
		 * loop that makes ABORT routine.
		 */
		ft_reparent_record_meta(ft, txn, meta, new_parent, slot,
			/*child_marked=*/ false, /*hold_ctx=*/ NULL);
		return;
	}
	if (ft->ordered_list) {
		/*
		 * External head, ordered list ON: its parent lives in the cell
		 * carried by node->prev.
		 */
		field = &ft_ord_cell_ptr(
			((struct cds_ft_node *) child)->prev)->parent;
	} else {
		/*
		 * External head, ordered list OFF: there is no cell -- the
		 * parent is stored directly in node->prev (see ft_set_parent's
		 * external branch, which rcu_assigns prev = parent).  Re-parent
		 * that field, matching the immediate ft_set_parent the non-parked
		 * path would have done.
		 */
		field = (struct cds_ft_inode_flag **)
			&((struct cds_ft_node *) child)->prev;
	}
	/*
	 * External head: @field is cell->parent or node->prev -- the back channel
	 * ft_flip_txn_record_head_back_edge records always-MW, and its header
	 * carries the why.  The metadata-bearing children took the
	 * ft_reparent_record_meta arm above, which names its owner.
	 */
	/*
	 * ☠ THE EXPECTED-OLD IS READ THROUGH THE TXN, NEVER RAW.  @field is a
	 * TRANSACTED word: a concurrent op can have a transient proxy parked in
	 * it, and a raw `*field` hands that descriptor POINTER to the engine as
	 * the expected-old -- which urcu_txn_add traps under a debug build
	 * ("Debug builds still trap, to name the raw read") and which silently
	 * records a value no reader ever sees otherwise.  ft_record_child_back_edge
	 * reads the same class of word the same way.
	 */
	ft_flip_txn_record_head_back_edge(txn, (void **) field,
		urcu_txn_load(ft_flip_txn_handle(txn), (void **) field,
			FT_FLIP_PROXY_TAG),
		new_parent);
}

/*
 * One-commit insert tail (see struct ft_insert_commit): the structural slot is
 * recorded as a txn edge (the fresh head is invisible -- the slot still reads
 * its old value), and the head's parent chain is fully wired, so the
 * splice-position search runs exactly as the post-publish splice did (the
 * from-head seed walks the parent chain, never the recorded slot).
 *
 * Ordered list ON (@cell != NULL): park the <= 4 ordered-list neighbour edges
 * into the SAME txn, commit once -- the head becomes reachable in the structural
 * index AND spliced into the cell list atomically for every reader.  Ordered
 * list OFF (@cell == NULL): there is no cell to splice; the txn carries only the
 * structural edges (the slot publish recorded at the publish site, plus any
 * split-compressed live re-parent), and the single commit below makes that
 * structural publish atomic and freeze-before-install all the same.  Either way
 * it then settles all slots to their direct values; the real top's writer-only
 * wiring (parent, slot offset, incoming_byte) was done at record time.
 */
static
enum urcu_txn_status ft_insert_one_commit(struct cds_ft *ft, const uint8_t *key,
		size_t key_len, struct ft_ord_cell *cell,
		struct ft_insert_commit *ic)
{
	enum urcu_txn_status st;

	if (cell) {
		struct ft_ord_cell *pred, *pred2, *succ0;
		struct urcu_txn_list_node *pred_lnode;

		/*
		 * Locate the predecessor.  From-HEAD (seed at the fresh head, walk the
		 * live parent chain up) is the fast default and the only safe choice for
		 * the attach shapes: a from-root LT would descend through the attach's own
		 * mid-build re-parent edge and loop in the reanchor.  Two shapes need
		 * from-ROOT instead, and neither loops there:
		 *   - split-compressed (ic->live_child): the deferred live edge is parked,
		 *     so a from-head LT would reanchor through the not-yet-wired edge; the
		 *     old compressed node is intact, so from-root descends it cleanly.
		 *   - external_nodes prefix key (!ic->publish_to_parent): the head sits at
		 *     an internal node's external_nodes and sorts BEFORE its extensions, so
		 *     a from-head seed mis-locates it; from-root descends only live nodes
		 *     (the parked external_nodes resolves to "no head"), no live re-parent.
		 * TODO(perf): the from-root cases re-descend; revisit if they show up hot.
		 */
		if (ft_ord_cell_find_pred_from_head(ft, key, key_len, cell,
				ic->live_child != NULL || !ic->publish_to_parent,
				&pred) < 0)
			goto splice_conflict;
		/*
		 * ORDER-INTENT capture + confirm (concurrent writers): the search
		 * decided "@cell belongs between @pred and @pred's successor" by
		 * KEY order, but a peer key committed after the search can
		 * interpose between @pred and ours -- an insert-after-@pred would
		 * then land BEFORE the interposed key (internally consistent
		 * list, wrong key order; nothing dead, so neither the deletion
		 * mark nor any expected-old catches it).  Capture the successor,
		 * then RE-RUN the (seeded, cheap) predecessor search: it
		 * returning @pred again proves no key sat between @pred and ours
		 * at an instant AFTER the capture, and the insert_between prepare
		 * below records @succ0 as &pred->next's expected old -- so any
		 * interposition after that instant fails the commit's value CAS.
		 * A recycled-@pred coincidence (freed and re-bound in the window)
		 * re-initializes pred->next, which then mismatches @succ0 the
		 * same way.  Never diverges under a single writer.
		 */
		succ0 = ft_ord_cell_resolve_ord(pred ?
			&pred->lnode.next : &ft->ord_sentinel.node.next);
		if (ft_ord_cell_find_pred_from_head(ft, key, key_len, cell,
				ic->live_child != NULL || !ic->publish_to_parent,
				&pred2) < 0)
			goto splice_conflict;
		/*
		 * Splice @cell between @pred and @succ0 via the composable op,
		 * recorded straight into the structural commit txn (FT's type-7
		 * proxy tag applies, so the splice is atomic with the structural
		 * publish for a bidirectional ordered reader).  Sentinel topology:
		 * a new MINIMUM (no predecessor) splices after the sentinel node;
		 * a new maximum lands before the sentinel naturally (@succ0 is the
		 * sentinel pseudo-cell).  The prepare records pred->next:
		 * succ0 -> cell and succ0->prev: pred -> cell, where either
		 * neighbour may be the sentinel.
		 */
		pred_lnode = pred ? ft_ord_cell_lnode(pred) : &ft->ord_sentinel.node;
		/*
		 * A FAILING confirm or prepare is a peer conflict, not a soft
		 * no-op: pred2 != pred = a key interposed (or @pred's key
		 * vanished) around the capture; -ENOENT = @pred died (its next
		 * carries the fused unsplice's deletion mark); -EAGAIN = the
		 * order intent went stale or the successor is mid-deletion.
		 * Splicing anyway (the former void cast) produced the
		 * resurrected / out-of-order cell class the ord-verify catches
		 * at rest.  Unwind exactly as a commit ABORT would (nothing is
		 * installed -- records are discarded with the descriptor,
		 * registered fences cleared by the destroy, the fresh cluster
		 * freed as the on-abort rollback does) and let the caller's
		 * ABORT path re-descend; age the handle first, exactly as a real
		 * commit ABORT ages it.
		 */
		if (pred2 == pred &&
		    ft_txn_list_insert_between_prepare(ft_flip_txn_handle(ic->txn),
				ft_ord_cell_lnode(cell), pred_lnode,
				ft_ord_cell_lnode(succ0)) >= 0)
			goto spliced;
splice_conflict:
		ft_free_unpublished_split_cluster(ft, ic->created,
			ic->nr_created);
		urcu_txn_conflict(ft_flip_txn_handle(ic->txn));
		ft_flip_txn_destroy(ic->txn);
		ic->txn = NULL;
		return URCU_TXN_STATUS_ABORT;
spliced:;
	}

	/*
	 * Record the deferred LIVE re-parent edge (split-compressed shapes) into
	 * the SAME txn: the live old child's back-pointer is the back-channel a
	 * reanchor up-walk follows into the fresh cluster, so flipping it in the
	 * one commit -- together with the forward edge (recorded at the publish
	 * site) and the neighbour edges -- makes structural reachability and the
	 * ordered-list splice a single atomic publication for every reader.
	 */
	if (ic->live_child)
		ft_park_live_parent_edge(ft, ic->live_child,
			ic->live_parent, ic->live_slot, ic->txn);

	/*
	 * Freeze-on-free (doc §4.B): the old compressed/internal node this
	 * commit retires gets its one-way LIVE->DEAD tombstone recorded INTO the
	 * same txn as the structural unlink, so the mark and the unlink flip
	 * atomically (atomic detach) -- a concurrent writer targeting the node
	 * then fails its validate-live CAS, and never sees a torn "unlinked but
	 * still live" window.  A no-op under one writer.  Recorded here, after the
	 * last abort point (all recording is infallible into the pre-reserved txn).
	 */
	if (ic->free_old_cn)
		/*
		 * FENCED retire (F2 split extension): the expected old is the
		 * builder's entry-mark snapshot, so the commit ratifies exactly
		 * the cn state the split plan was derived from; the fence itself
		 * was registered with @txn at arm time (cleared on every
		 * non-commit outcome, consumed by this transition on commit).
		 * One record while the lock sits on the cn itself, two once
		 * coarsening moved it to a surviving ancestor.
		 */
		/*
		 * NULL ledger: the commit runs past the split's lock context, and
		 * @free_old_cn is the node this op RETIRES -- nothing of this op
		 * anchors on a word it is about to tombstone, so the arm that needs
		 * the ledger cannot apply.
		 */
		ft_flip_txn_record_retire_anchored(ic->txn, NULL,
			&ic->free_old_cn_held,
			cds_ft_item_to_metadata(
				(struct cds_ft_inode *) ic->free_old_cn));
	/*
	 * @free_old_node needs NO record here: its sole producer is the
	 * ft_attach_node relocation, whose recompact runs with @ic->txn as the
	 * retire txn and records the fenced {LOCK|s -> TOMBSTONE|s}
	 * transition itself.  A second raw {s -> s|TOMBSTONE} record on the
	 * same word would same-slot-UPGRADE the recompact's new value and leak
	 * the fence bit into the dead word.  The deferred free below still
	 * covers it.
	 */
	/*
	 * Order-statistics count fold (rank stats ON): record the +1 key-count
	 * propagation as nr_keys value-CAS edges on the STABLE ancestor chain from
	 * @count_from to the root, folded into THIS commit so the count flips
	 * atomically with the structural publish.  A no-op when rank stats are off
	 * or the shape did not opt in.  The arm reserved the per-ancestor edges.
	 */
	if (ic->count_folded)
		ft_flip_txn_record_count_parent(ft, ic->txn, ic->count_from, 1);
	/*
	 * THE commit: install every recorded edge -- the forward structural
	 * publish (forward slot + any compressed-parent skip-slot dual, or the
	 * set_nth slot proxy), the ordered-list neighbour edges and the live
	 * re-parent edge -- then flip the group and settle each slot to its
	 * direct value, all atomically.  The real top's writer-only wiring
	 * (parent, slot offset, incoming_byte) was done at record time, while
	 * still invisible.
	 */
	/*
	 * Register the commit-outcome cleanup, then commit.  The engine runs the
	 * finalize (grace-free the retired free_old_* nodes -- readers resolved the
	 * parked proxy to them until now) IFF the commit lands, or the rollback
	 * (free the fresh unpublished cluster) IFF a peer writer won the forward
	 * slot / froze a guarded node.  So both hazards -- "the retire target is
	 * still live on abort, don't free it" and "the fresh cluster leaked on
	 * abort, do free it" -- are dispatched by outcome, atomically with the
	 * publish decision.  @ic outlives the commit (the caller's retry frame).
	 */
	ic->ft = ft;
	st = ft_flip_txn_commit(ft, ic->txn);
	ic->txn = NULL;
	/*
	 * Dispatch the commit-outcome cleanup inline (the engine has no
	 * defer-on-commit/abort mechanism): on OK the retired free_old_* nodes
	 * are now unreachable, so finalize queues their grace-period-deferred
	 * free; on ABORT or MEMORY_ERROR nothing was installed, so the rollback
	 * frees the fresh unpublished cluster.  Both callbacks only touch memory
	 * whose reachability the commit outcome has already settled, so running
	 * them here rather than inside the commit is equivalent.
	 */
	if (st == URCU_TXN_STATUS_OK)
		ft_insert_finalize_cb(ic);
	else
		ft_insert_abort_cb(ic);
	return st;
}

#ifdef FEATURE_FT_PROBE_EMPTY_INSERT
/*
 * A/B CANDIDATE, insert side: refuse to publish an EMPTY non-root internal into
 * a live parent slot, and count every refusal.
 *
 * WHY HERE, AND WHY A BAIL RATHER THAN A PROBE.  The residual is a live
 * reachable internal with nr_child == 0, no external_nodes, a CLEAN state word,
 * the smallest internal type, and an incoming_byte that agrees with the parent
 * slot it hangs from -- i.e. correctly formed and correctly wired, with only its
 * CONTENT missing.  That fits a fresh node PUBLISHED empty far better than a
 * node emptied in place, and every removal-side arm has now been A/B-refuted by
 * a bail that never fired while the defect kept happening (the detach boundary
 * re-validation, the shape-D fusion gate, the FT_RECOMPACT_DEL sizing, and the
 * recompact copy loop's NULL-child skip).
 *
 * Six earlier instrumentation designs SUPPRESSED the defect -- including the
 * tracing build with every event disabled -- so a passive counter at a shared
 * chokepoint is not a legitimate experiment here.  What remains legitimate is an
 * A/B of a candidate fix that CARRIES ITS OWN BAIL COUNTER, placed on the
 * INSERT side rather than in the hot path that suppressed last time.  The rule
 * that settles it: a change that makes the defect vanish while its bail NEVER
 * FIRES has not fixed anything.
 *
 * Compiled out entirely by default, so the baseline arm of the A/B is the
 * ordinary build rather than a differently-shaped one.
 */
extern unsigned long cds_ft_probe_empty_publish_split;
extern unsigned long cds_ft_probe_empty_publish_attach;
/*
 * REACH controls.  "empty == 0" is only evidence if the site RUNS -- the same
 * trap that let the replace family's abort arms sit unexecutable.  These count
 * every visit to the two guarded publishes, so a zero empty-count can be read
 * as "this family is innocent" rather than "this instrument never fired".
 */
extern unsigned long cds_ft_probe_reach_split;
extern unsigned long cds_ft_probe_reach_attach;

static inline
bool ft_probe_internal_is_empty(struct cds_ft *ft,
		struct cds_ft_inode_flag *nf)
{
	struct cds_ft_metadata *meta;

	if (!nf || ft_node_flip_proxy(nf) || ft_node_external(nf))
		return false;
	if (ft_node_compressed(nf))
		return false;
#ifdef FEATURE_FT_SKIP_COMPRESSED
	if (ft_node_skip_compressed(nf))
		return false;
#endif
	(void) ft;
	meta = cds_ft_item_to_metadata(ft_node_ptr(nf));
	return ft_meta_nr_child(meta) == 0 && !meta->external_nodes;
}
#endif /* FEATURE_FT_PROBE_EMPTY_INSERT */

/*
 * Publish @new_top into @slot (owned by @parent_nf): direct via
 * ft_publish_to_parent, or -- one-commit insert, @ic armed -- RECORD the
 * forward publish (the slot store, plus a compressed parent's dual skip-slot
 * store; both captured via _ft_publish_to_parent, which also runs @new_top's
 * writer-only parent-slot bookkeeping) into @ic->txn, so the forward edge
 * commits atomically with the ordered-list splice at insert_done's single flip.
 * The caller must have wired @new_top's parent back-pointer already (parent-
 * before-publish; with a recorded forward edge the cluster only becomes
 * reachable at the commit, by which time the wiring is complete either way).
 */
/*
 * §9.3, the GP member: "Lock-set: {C, P} (+ {GP} iff P compressed)".
 *
 * A publish under a COMPRESSED parent writes TWO slots, because a skip pointer
 * is a shortcut that names the FAR END: besides @parent_nf's own child slot,
 * _ft_publish_to_parent re-encodes the SKIP_X dual -- and that slot lives in
 * GP's BODY, so §8.2 makes GP its owner.  The op must therefore HOLD GP, not
 * merely write through it.
 *
 * ☠ THE CONDITION MIRRORS THE PRODUCER EXACTLY, including the mtxn the slot is
 * resolved with (NULL, as ft_insert_publish_or_park's @rec carries none): an
 * acquire taken on a different derivation than the record locks the wrong word,
 * and one taken where no dual is recorded is pure contention on the hottest
 * insert path.
 *
 * A COMPRESSED ROOT's dual slot IS &ft->root, which has no owning node and
 * takes the always-MW ft_flip_txn_record_root route -- nothing to acquire.
 *
 * Registering hands the fence to @ic->txn, so every pre-publish bail that
 * destroys the txn clears it; a miss sets @acquire_miss and the commit aborts
 * all-or-none, exactly as the P acquire beside it does.
 */
static
void ft_insert_lock_skip_dual_gp(struct cds_ft *ft,
		const struct ft_lock_ctx *ctx,
		struct cds_ft_inode_flag *parent_nf,
		struct ft_insert_commit *ic)
{
#ifdef FEATURE_FT_SKIP_COMPRESSED
	struct cds_ft_compressed_node *cn;
	struct cds_ft_metadata *cn_meta;
	struct cds_ft_inode_flag *gp_nf = NULL;
	struct cds_ft_inode_flag **skip_slot;

	if (!parent_nf || !ft_node_compressed(parent_nf))
		return;
	cn = ft_compressed_node_ptr(parent_nf);
	cn_meta = cds_ft_item_to_metadata((struct cds_ft_inode *) cn);
	skip_slot = ft_txn_parent_slot_at(cn_meta, ft, NULL, &gp_nf);
	if (!skip_slot || !ft_node_skip_compressed(*skip_slot))
		return;			/* no dual edge will be recorded */
	if (skip_slot == &ft->root || !gp_nf)
		return;			/* root dual: no owning node */
	ft_flip_txn_lock_or_guard_parent(ft, ic->txn, ctx, gp_nf,
		FT_DEPTH_FROM_DESCENT);
#else
	(void) ft; (void) ctx; (void) parent_nf; (void) ic;
#endif
}

static
void ft_insert_publish_or_park(struct cds_ft *ft,
		const struct ft_lock_ctx *ctx,
		struct cds_ft_inode_flag *parent_nf,
		unsigned int parent_depth,
		struct cds_ft_inode_flag **slot,
		struct cds_ft_inode_flag *new_top,
		struct cds_ft_inode_flag *expected_old,
		struct ft_insert_commit *ic)
{
	struct ft_pub_rec rec = { .n = 0 };

	/*
	 * @expected_old: the plan-snapshot value of *slot (the old subtree this
	 * divergence replaces), captured by the caller BEFORE its build so the
	 * commit CAS rejects a peer that raced the slot -- see ft_pub_rec_add.
	 */

	/*
	 * @ic is mandatory and armed by the caller (a bulk insert is a graft /
	 * merge_at, not this path), so the forward edge always rides the txn.
	 */
	assert(ic && ic->txn);
	/*
	 * VALIDATE (§4.B) / LOCK (§9.1, LOCK_FINE): @parent_nf is the compressed-
	 * split lock-set member this publish writes into (I-4a diverge / key-shorter
	 * publish into CN's parent P; I-4b past-child publishes into CN itself).  In
	 * every shape @parent_nf SURVIVES the commit and its slot is a same-slot
	 * VALUE swap -- its own body is never copied under the lock -- so it is the
	 * value-swap publish target ft_flip_txn_lock_or_guard_parent handles: acquire
	 * it as a RELEASE lock under LOCK_FINE, or fall back to the §4.B guard on a
	 * miss.  All shapes are mutually exclusive with the ft_attach_node relocation
	 * guard, so the one reserved guard/release slot in ic->txn covers whichever
	 * fires.
	 *
	 * DLM Step 1: when a split builder pre-acquired @parent_nf as P in its one-
	 * commit lock-set (ic->parent_locked_holder set), record P's RELEASE from the
	 * held snap instead of re-marking it -- a re-mark would MISS on the op's OWN
	 * still-set fence and self-abort via the guard fallback.  A NULL holder
	 * (past-child {CN}, non-split shapes) routes to the incremental lock_or_guard,
	 * behaviour-identical to non-DLM.
	 *
	 * Coarsening adds the SHARED case: P's lock rode CN's anchor, so the op
	 * holds it and its release is already recorded (with CN's, at registration).
	 * Only the §4.B guard is left to plant -- re-acquiring would miss on the
	 * op's own hold exactly as a re-mark would, and a second release would
	 * double-record the one word.
	 */
	if (ic->parent_lock_shared)
		ft_flip_txn_guard_parent(ft, ic->txn, parent_nf);
	else
		ft_flip_txn_hold_or_lock_parent(ft, ic->txn, ctx, parent_nf,
			parent_depth, ic->parent_locked_holder,
			ic->parent_locked_snap);
	ft_insert_lock_skip_dual_gp(ft, ctx, parent_nf, ic);
	/*
	 * PHASE B, STEP B1 -- THE ARM, and this is the only point in the op where
	 * it is legal.  ft_flip_txn_arm_per_op's contract is "after the op's LAST
	 * ft_flip_txn_lock_register": a lock registered later still protects its
	 * word, but a record planted in between would be checked against a
	 * registry that does not yet name its owner.  The two lines above ARE the
	 * last two registers -- P's release (or its guard on the shared/miss arms)
	 * and, when P is compressed, the SKIP_X dual's GP (§9.3's third member).
	 *
	 * ☞ WHAT IT CONVERTS, AND WHAT IT DELIBERATELY DOES NOT.  Kind dispatch
	 * happens at RECORD time, so this arms the records planted from here on --
	 * the forward publish below, and the retire / count / re-parent edges
	 * ft_insert_commit adds later.  Records planted EARLIER on this same txn --
	 * the ft_attach_node relocation's recompact edges -- stay MW, which is
	 * STRICTER and always sound.  Converting those needs the arm moved above
	 * the recompact, which its own {C, P, GP} acquire has not finished at that
	 * point; that is a later step, not a gap this one leaves open.
	 *
	 * The helper refuses an empty registry, a non-FINE trie, and a trie whose
	 * constructor already armed it trie-wide -- so a shape that acquired
	 * nothing keeps today's all-MW behaviour rather than parking on an
	 * exclusion it never took.
	 */
	ft_flip_txn_arm_per_op(ft, ic->txn);
	_ft_publish_to_parent(ft, parent_nf, slot, new_top, expected_old, &rec);
	ft_flip_txn_record_pub_rec(ic->txn, &rec);
	ic->slot = slot;	/* sentinel: one-commit forward recorded */
	ic->publish_to_parent = true;
}

/*
 * DLM Step 1 (see doc/design/mw-writer-lock-escalation-model.md):
 * acquire the compressed-split lock-set
 * {CN, P} in ONE all-or-none MCAS up front, replacing the incremental CN
 * lock-acquire here plus the P lock_or_guard inside ft_insert_publish_or_park.
 * Mirrors the recompact hoist ({C,P}): P is CN's parent (the value-swap forward-
 * publish target), resolved racy and validated by the read-set guard
 * CN.parent==P in the SAME commit, so a peer re-homing CN between the plan read
 * and the acquire aborts it -> re-plan.
 *
 * On OK: @*held names the word the acquire actually CAS'd for CN -- CN's own
 * metadata under per-node granularity, its ANCHOR's under a coarser one -- plus
 * the clean snapshots the caller's release and retire terminals need
 * (struct ft_held_anchor).  The caller must name @held->lock, never @cn_meta,
 * on every bail and registration: under coarsening CN's own word was never
 * locked.  And ic->parent_locked_holder/_snap carry P's held release so
 * ft_insert_publish_or_park records it (ft_flip_txn_hold_or_lock_parent) instead
 * of re-marking.  P == NULL (CN at the root) => the lock-set is {CN} only, no
 * guard; publish_or_park's NULL @parent_nf then no-ops as today.
 *
 * Returns 0 (whole set acquired), -EAGAIN (a member is held / a re-home aborted
 * the commit -- NOTHING acquired, abort-and-regrow), or -ENOMEM.
 */
static inline
int ft_insert_dlm_acquire_split(struct cds_ft *ft,
		const struct ft_descent *d, struct cds_ft_inode_flag *cn_flag,
		unsigned int cn_depth,
		struct cds_ft_metadata *cn_meta, struct ft_held_anchor *held,
		struct ft_insert_commit *ic)
{
	struct cds_ft_inode_flag *pf_p = NULL;
	struct ft_dlm_member set[2];
	struct ft_lock_ctx lctx;
	unsigned int p_depth = 0;
	int dret;

	/* PLAN (read-only, racy): resolve CN's parent P. */
	(void) ft_resolve_parent_slot(cn_meta, ft, &pf_p);

	/*
	 * ANCHOR both members.  Under per-node granularity these are CN and P
	 * themselves and nothing below changes; under a coarser one they are the
	 * ancestors that carry their lock, and the two may COINCIDE -- CN and P
	 * share an anchor whenever they fall in one lock band.  Dedupe then, or
	 * the second ft_dlm_lock aborts -EAGAIN on the op's own hold (§7.3).
	 *
	 * P's depth comes from the DESCENT's window, not from @pf_p: the plan
	 * resolves P through CN's back-pointer, which yields a node with no depth
	 * at all.  Where the two disagree the descent is not describing this P,
	 * so there is no depth to anchor it by -- re-plan rather than anchor it
	 * with a depth that belongs to another node.
	 */
	if (pf_p && ft->lock_spacing != CDS_FT_LOCK_SPACING_PER_NODE) {
		if (!d || pf_p != d->pnf)
			return -EAGAIN;
		p_depth = d->pdepth;
	}
	set[0] = (struct ft_dlm_member){ .nf = cn_flag, .node = cn_meta,
		.depth = cn_depth,
		/*
		 * The guard is a read-set validation of CN's OWN back-edge (the
		 * plan resolved P through it), so it names @cn_meta even where
		 * the lock goes to an ancestor -- anchoring moves the exclusion,
		 * not the edge being validated.
		 */
		.guard_child = pf_p ? cn_meta : NULL, .guard_pf = pf_p };
	set[1] = (struct ft_dlm_member){ .nf = pf_p,
		.node = pf_p ? ft_flag_to_metadata(ft, pf_p) : NULL,
		.depth = p_depth };

	/*
	 * ACQUIRE {CN, P} + the read-set guard CN.parent==P in ONE MCAS.  The set
	 * primitive samples and guards CN's OWN word where coarsening left it
	 * unlocked -- the split still RETIRES CN against it -- and dedupes the two
	 * members where one anchor covers both.
	 *
	 * Bound to the op's PERSISTENT handle when there is one, so the acquire
	 * ages and takes its lane turn (struct ft_lock_ctx.op).  @ic->txn is not
	 * live at this point (the commit is armed later), so the op carries only
	 * this txn's records here.
	 */
	ft_lock_ctx_init(&lctx, d, NULL, ic ? ic->op : NULL);
	dret = ft_dlm_acquire_set(ft, &lctx, set, 2);
	if (dret)
		return dret == -ENOMEM ? -ENOMEM : -EAGAIN;

	*held = set[0].held;
	/*
	 * P's held release, threaded to ft_insert_publish_or_park.  A P that
	 * shares CN's anchor was acquired WITH it and its release is already
	 * recorded, so the publish is told it is held-and-accounted-for rather
	 * than absent.
	 */
	ic->parent_lock_shared = pf_p && set[1].held.shared;
	ic->parent_locked_holder = (pf_p && !set[1].held.shared) ?
		set[1].held.lock : NULL;
	ic->parent_locked_snap = pf_p ? set[1].held.lock_snap : 0;
	return 0;
}

/*
 * Release a compressed-split lock-set acquired by ft_insert_dlm_acquire_split on
 * a PRE-PUBLISH bail (CN's parent P is held but not yet registered into ic->txn
 * -- publish_or_park registers it).  The CN fence is cleared by the caller's own
 * ft_meta_lock_release(held.lock) / ic->txn drain, exactly as the incremental
 * scheme did; this only lifts the EXTRA word the DLM hoist acquired for P.
 * Where coarsening mapped P onto CN's anchor there is no extra word -- the
 * caller's own release lifts the one they share -- so the shared marker is
 * simply dropped.
 */
static inline
void ft_insert_dlm_release_parent(struct ft_insert_commit *ic)
{
	if (ic->parent_locked_holder) {
		ft_meta_lock_release(ic->parent_locked_holder);
		ic->parent_locked_holder = NULL;
	}
	ic->parent_lock_shared = false;
}

/*
 * Arm the one-commit txn just before a fresh-head publish (fallible; the
 * caller's error unwind runs with nothing published).  @ic is mandatory -- a
 * bulk insert is a graft / merge_at, not this path.  Armed for BOTH ordered-list
 * states: the structural
 * slot publish commits through the txn either way (freeze-before-install, MCAS
 * Invariant-1); with the list on it additionally carries the cell-splice edges,
 * with the list off it carries only the structural edges (and insert_done
 * commits it with @cell == NULL).  Returns 0, or -ENOMEM.
 */
static
int ft_insert_commit_arm(struct cds_ft *ft, struct ft_insert_commit *ic,
		unsigned int count_edges)
{
	assert(ic);
	/*
	 * Forward publish (<=2: the slot store + a compressed parent's skip-slot
	 * dual) + <=4 cell neighbour edges (list-on) + the live re-parent edge.
	 */
	/*
	 * Edge budget: the forward publish (<=2: slot + a compressed parent's
	 * skip-slot dual) OR -- attach path -- the new key's reserved slot edge
	 * (1) plus the recompact-relocation grandparent publish (<=2) folded in;
	 * + <=4 cell neighbour edges (list-on) + the live re-parent edge (<=7);
	 * + the <=2 freeze-on-free tombstone edges (an old compressed and/or old
	 * internal node this commit retires) now fused into the same flip as the
	 * unlink (atomic detach, doc §4.B); + 1 VALIDATE freeze guard (the
	 * relocation's live grandparent, or -- mutually exclusive insert shape --
	 * the in-place external-head park holder; §4.B validate); + 1 VALIDATE
	 * freeze guard on the IN-PLACE attach holder (the reserved-byte edge below
	 * stores a slot INSIDE the live attach node -- NOT mutually exclusive with
	 * the external-head park holder above, so it needs its own reservation);
	 * + @count_edges nr_keys count edges (rank-stats-ON count fold), one per
	 * STABLE ancestor from the count base to the root -- sized by the caller to
	 * the ACTUAL descent depth (0 when the shape does not fold its count);
	 * + 1 for the live re-parent's paired state edge (ft_reparent_record_meta
	 * records parent AND slot-offset, two records, when the parked live child
	 * bears metadata);
	 * + 1 once coarsening splits the split-retire terminal in two, the retired
	 * cn's tombstone no longer being the same word as the release of the lock
	 * that protected it (ft_flip_txn_record_retire_anchored);
	 * + 1 for the SKIP_X dual's GP release terminal (§9.3's GP member: a publish
	 * under a COMPRESSED parent re-encodes a dual slot living in GP's body, so
	 * the op acquires GP -- ft_insert_lock_skip_dual_gp -- and its release rides
	 * this commit).
	 */
	unsigned int anchored = ft->lock_spacing == CDS_FT_LOCK_SPACING_PER_NODE ?
			0 : 1;

	/*
	 * The content txn is STANDALONE.  Binding it to the op handle
	 * (ft_flip_txn_create_bounded_on) shares the op's descriptor and its
	 * install lane across attempts, and that sharing MANUFACTURES the very
	 * conflicts the retry loop then absorbs -- see the commit log.  The op
	 * still carries @ic->op for enrolment; what it must not do is commit
	 * through it.
	 */
	ic->txn = ft_flip_txn_create_bounded(ft, 15 + anchored + count_edges);
	if (!ic->txn)
		return -ENOMEM;
	return 0;
}

/*
 * Park a "new key at an existing internal node" publish into the one-commit
 * batch.  The head is published by storing it into @metadata->external_nodes
 * (not a child slot), so park a flip proxy THERE -- readers resolve it via
 * ft_dereference_external -- and the structural publish then commits atomically
 * with the ordinal-cell splice at the single MCAS flip commit.  No transient
 * half-spliced list state.  Settles direct (publish_to_parent false: there is
 * no parent child-slot to re-encode).  @ic must be armed.  Records old ==
 * @metadata->external_nodes read here, so it serves BOTH the fresh-key case
 * (old == NULL: a reader resolves the proxy to "no head" until the commit) AND
 * the chain-replace case (old == the chain being replaced -> @node).
 */
static
void ft_insert_park_external_nodes(struct cds_ft *ft,
		const struct ft_descent *d,
		struct cds_ft_metadata *metadata, struct cds_ft_node *node,
		struct ft_insert_commit *ic)
{
	struct ft_lock_ctx lctx;

	/*
	 * ☠ STEP 1 FIRST: TAKE the holder, do not merely OBSERVE it.
	 *
	 * §8.2 puts the entry list in the HOLDER's own field, so @metadata owns
	 * the word this parks into -- and owning it is not enough, the op has to
	 * HOLD it.  A §4.B VALIDATE on @metadata->state stood here instead, and
	 * it is sound only while every writer of that word is MW: an MW guard
	 * arbitrates against an MW peer through the expected-old, and against an
	 * SW park not at all.  "A slot is SW xor MW, globally"
	 * (urcu_txn_store_sw).  The moment ANY insert shape arms per-op, its
	 * lock releases and nr_child increments park @metadata->state SW -- and
	 * this shape, holding nothing, would be exactly the far-side op
	 * ft_flip_txn_record_state warns about: "an op that writes or validates
	 * this word having never performed step 1 at all... Ownership is TAKEN,
	 * never observed."
	 *
	 * So acquire and REGISTER the holder.  On a hit that replaces the guard
	 * with the stronger {LOCK|s -> s} release terminal -- a concurrent remove
	 * can no longer freeze the holder at all, rather than being detected
	 * after the fact.  On a miss ft_flip_txn_lock_or_guard_parent sets
	 * @acquire_miss and the commit ABORTS, all-or-none, and the op re-plans;
	 * it deliberately does not spin.
	 *
	 * ★ BEFORE the record, never after: the record asks ft_flip_txn_owns who
	 * owns the word it writes, so the registry has to name the holder
	 * already.
	 */
	ft_lock_ctx_init(&lctx, d, ic->txn, ic->op);
	ft_flip_txn_lock_or_guard_parent(ft, ic->txn, &lctx, d->nf,
		FT_DEPTH_FROM_DESCENT);
	ft_flip_txn_record_reserved(ic->txn, /*owner=*/ metadata,
		(void **) &metadata->external_nodes,
		(void *) metadata->external_nodes, (void *) node);
	ic->slot = (struct cds_ft_inode_flag **) &metadata->external_nodes;
	ic->publish_to_parent = false;
}

/*
 * Split a compressed node during insert when the new key diverges
 * from the compressed path at position @diverge_pos.
 *
 * Builds the following structure bottom-up:
 *
 *   [prefix compressed/internal] -> [branch internal]
 *                                    +- old_ordinal -> [suffix compressed/internal] -> old_child
 *                                    `- new_ordinal -> [new branch compressed/internal] -> new_leaf
 *
 * If diverge_pos == 0, no prefix is needed.  If the suffix or new
 * branch is 0 bytes, the child is placed directly.  If 1 byte, a
 * single-child internal node is used.  If >= 2 bytes, a compressed
 * node is created.
 *
 * The old compressed node's external_nodes (if any) are preserved
 * at the prefix level (or the branch if no prefix).
 *
 * Publishes the result at @parent_slot via rcu_assign_pointer and
 * frees the old compressed node.  Returns 0 on success, -ENOMEM
 * on allocation failure (compressed node left in place).
 */

/*
 * Free a freshly-built, never-published split cluster -- the nodes tracked in
 * @created[0 .. @nr_created).  Shared by both compressed-split helpers'
 * -ENOMEM error paths AND the abort-fully path in
 * ft_insert_compressed_key_shorter (when the one-commit arm fails after a
 * successful build).  The live old child (cn->child) is never tracked in
 * @created, and free_*_unpublished free only the node (not its children), so
 * the live subtree is never touched.
 *
 * Freed PARENT-FIRST (reverse of the bottom-up creation order): a skip-encoded
 * top is resolved to its compressed node via its child's back-pointer
 * (ft_skip_to_compressed reads child->parent), so the child must still be live
 * when the parent is freed.  (A partial-build error path never has both, but
 * the post-build abort frees the whole cluster, so the order matters.)
 */
static
void ft_free_unpublished_split_cluster(struct cds_ft *ft,
		struct cds_ft_inode_flag *const *created, int nr_created)
{
	int i;

	for (i = nr_created - 1; i >= 0; i--) {
		if (ft_node_compressed(created[i]))
			free_compressed_node_unpublished(ft,
				ft_compressed_node_ptr(created[i]));
		else if (ft_node_skip_compressed(created[i]))
			free_compressed_node_unpublished(ft,
				ft_skip_to_compressed(ft, created[i]));
		else
			free_cds_ft_node_unpublished(ft,
				ft_node_ptr(created[i]));
	}
}

static
int ft_split_compressed_insert(struct cds_ft *ft,
		const struct ft_descent *dsc,	/* anchor source for the lock-set */
		struct cds_ft_inode_flag **parent_slot,
		struct cds_ft_inode_flag *compressed_flag,
		const uint8_t *iter_key,	/* key bytes at compressed node's depth */
		unsigned int remaining_key,	/* key bytes remaining from compressed depth */
		unsigned int diverge_pos,	/* position within compressed path */
		struct cds_ft_node *child_node,	/* new external node to insert */
		unsigned int node_depth,	/* depth of the compressed node */
		struct ft_insert_commit *ic)	/* one-commit insert (mandatory) */
{
	struct cds_ft_compressed_node *cn = ft_compressed_node_ptr(compressed_flag);
	struct cds_ft_metadata *cn_meta =
		cds_ft_item_to_metadata((struct cds_ft_inode *) cn);
	/*
	 * F2 node lock, split-retire extension: this split retires @cn and
	 * derives its WHOLE plan from it -- the diverge slicing over
	 * cn->key_bytes, the cn->child snapshot, the deferred-edge captures --
	 * so fence @cn BEFORE the first read, exactly as a recompact fences the
	 * node it copies.  Single-splitter exclusion: a peer split / collapse /
	 * retire of the SAME cn either holds the fence (mark fails -> -EAGAIN,
	 * re-descend) or aborts at commit against the fenced tombstone's
	 * precise expected old (ft_insert_one_commit records
	 * the retire from @held).  Cleared locally on every
	 * bail until the arm registers it with @ic->txn (whose every terminal
	 * path then owns the outcome).
	 *
	 * @held names the word the acquire CAS'd, which is @cn_meta only at
	 * per-node granularity; every release and registration below goes
	 * through @held.lock so it lifts the lock this op actually took.
	 */
	struct ft_held_anchor held;
	int fret;
	struct cds_ft_inode_flag *fwd_expected_old;	/* set post-fence */

	/*
	 * DLM Step 1: under LOCK_FINE, acquire the whole split lock-set {CN, P} in
	 * one all-or-none MCAS up front (P = CN's parent, the value-swap publish
	 * target ft_insert_publish_or_park writes into below), replacing the
	 * incremental CN mark here + the P lock inside publish_or_park.  Non-DLM /
	 * non-lock_fine keeps the single CN lock-acquire (byte-identical).
	 */
	if (ft->lock_fine) {
		fret = ft_insert_dlm_acquire_split(ft, dsc, compressed_flag,
				node_depth, cn_meta, &held, ic);
	} else {
		struct ft_lock_ctx lctx;

		ft_lock_ctx_init(&lctx, dsc, ic ? ic->txn : NULL, ic ? ic->op : NULL);
		fret = ft_acquire_member(ft, &lctx, compressed_flag, cn_meta,
			node_depth, &held);
	}

	if (fret)
		return fret;	/* -EAGAIN: peer owns @cn/P; nothing built */

	/*
	 * Plan-snapshot the raw grandparent slot NOW (post-fence, pre-build):
	 * *parent_slot is the skip-PRESERVED, flip-proxy-resolved reference to
	 * @cn -- a SKIP_X flag under skip-compression, NOT the skip-resolved
	 * @compressed_flag -- which is exactly what the forward publish CAS must
	 * match.  Passing @compressed_flag would mismatch the stored SKIP_X
	 * forever (livelock); re-reading at publish time would ratify a peer
	 * that raced @cn's grandparent slot during the build.
	 */
	fwd_expected_old = ft_resolve_flip_proxy(*parent_slot);

	/*
	 * Compressed metadata never carries external_nodes
	 * (ft_metadata_set_external_nodes aborts on a compressed target).
	 * The prefix builders below rely on it.
	 */
	assert(!cn_meta->external_nodes);
	FT_TP(split_compressed_insert_enter, (const void *) cn,
		cn->len, diverge_pos);
	struct cds_ft_inode_flag *old_suffix_flag, *new_branch_flag;
	struct cds_ft_inode_flag *branch_flag, *top_flag;
	struct cds_ft_inode_flag *created[FT_MAX_DEPTH];
	int nr_created = 0;
	unsigned int suffix_len = cn->len - diverge_pos - 1;
	unsigned int new_len = remaining_key - diverge_pos - 1;
	uint8_t old_ordinal = cn->key_bytes[diverge_pos];
	uint8_t new_ordinal = iter_key[diverge_pos];
	unsigned long old_child_nr_keys;
	int ret;
	/*
	 * Two-phase publish: build the cluster invisibly, then wire the
	 * deferred back-pointers and swing the parent slot, at the end.
	 * See the rcu-mutation build-invisible pattern.
	 *
	 * suffix_len >= 1: the branch's children (suffix, new) are new cluster
	 * nodes -- set their back-pointers normally; only the live old child
	 * into the new suffix (cn->child -> sfx) is deferred (deferred edge 1).
	 * suffix_len == 0: the branch is a cluster-leaf (old direction is the
	 * live cn->child, new direction the new subtree); set_nth defers BOTH
	 * its children, wired here at publish to the final branch_flag (deferred
	 * edges 1 and 2).
	 */
	struct cds_ft_inode_flag *sfx_skip_flag = NULL;
	struct cds_ft_inode_flag *deferred_child = NULL;
	struct cds_ft_inode_flag *deferred_parent = NULL;
	struct cds_ft_inode_flag **deferred_slot = NULL;
	struct cds_ft_inode_flag *deferred_child2 = NULL;
	struct cds_ft_inode_flag **deferred_slot2 = NULL;
	struct cds_ft_inode_flag *cur_parent;
	bool branch_cluster_leaf = (suffix_len == 0);

	unsigned int junction_depth = node_depth + diverge_pos;

	/*
	 * ONE resolved snapshot of the live cn->child (§9): the slot may carry
	 * a peer's parked flip proxy -- kind-dispatching the raw latch faults
	 * (item_to_metadata on the latch memory), and multiple raw re-reads
	 * could tear across the peer's flip.  Every use below (sizing, the
	 * fresh suffix's forward wiring, the deferred-edge captures) consumes
	 * THIS value; a post-snapshot peer commit is caught by our forward
	 * CAS / §4.B guard at commit time (ABORT -> re-descend).
	 */
	struct cds_ft_inode_flag *old_child_flag =
		ft_cn_child_dereference_acquire_prefetch(cn);

	/* Compute old child's nr_keys for the new nodes. */
	if (!ft_node_external(old_child_flag)) {
		struct cds_ft_metadata *cm =
			cds_ft_item_to_metadata(ft_node_ptr(old_child_flag));
		old_child_nr_keys = ft_nr_keys_get(cm);
	} else if (old_child_flag) {
		old_child_nr_keys = 1;	/* external leaf */
	} else {
		old_child_nr_keys = 0;
	}

	/* 1. Build old suffix -> old child. */
	if (suffix_len >= 1) {
		struct cds_ft_compressed_node *sfx;
		struct cds_ft_metadata *sfx_meta;

		sfx = alloc_compressed_node(ft, suffix_len, &sfx_meta);
		if (!sfx) goto error;
		sfx->child = old_child_flag;
		sfx->len = suffix_len;
		memcpy(sfx->key_bytes, &cn->key_bytes[diverge_pos + 1], suffix_len);
		ft_meta_nr_child_set(sfx_meta, 1);
		ft_nr_keys_store(ft,sfx_meta, old_child_nr_keys, CMM_RELAXED);
		old_suffix_flag = ft_compressed_node_flag(sfx);	/* PLAIN: install + recover sfx directly */
		sfx_skip_flag = ft_publish_compressed(ft, sfx, old_suffix_flag);	/* skip form for the slot */
		created[nr_created++] = old_suffix_flag;	/* track PLAIN so the error path frees sfx directly */
		/*
		 * Defer re-parenting the live old child to publish: writing
		 * cn->child's back-pointer now would make this unpublished
		 * cluster observable from the bottom (and the branch install
		 * below would otherwise recover sfx through it).  sfx->child
		 * already points at cn->child (a write into the new sfx only).
		 */
		deferred_child = old_child_flag;
		deferred_parent = old_suffix_flag;
		deferred_slot = &sfx->child;
	} else {
		/* suffix_len == 0: old child directly. */
		old_suffix_flag = old_child_flag;
	}

	/* 2. Build new branch -> new leaf. */
	if (new_len >= 1) {
		struct cds_ft_compressed_node *nb;
		struct cds_ft_metadata *nb_meta;

		nb = alloc_compressed_node(ft, new_len, &nb_meta);
		if (!nb) goto error;
		nb->child = (struct cds_ft_inode_flag *) child_node;
		nb->len = new_len;
		{
			unsigned int k;

			for (k = 0; k < new_len; k++)
				nb->key_bytes[k] = iter_key[diverge_pos + 1 + k];
		}
		ft_meta_nr_child_set(nb_meta, 1);
		ft_nr_keys_store(ft,nb_meta, 1, CMM_RELAXED);
		new_branch_flag = ft_compressed_node_flag(nb);
		ft_set_parent(ft, nb->child, new_branch_flag, NULL);
		new_branch_flag = ft_publish_compressed(ft, nb, new_branch_flag);
		created[nr_created++] = new_branch_flag;
	} else {
		/* new_len == 0: child_node directly. */
		new_branch_flag = (struct cds_ft_inode_flag *) child_node;
	}

	/* 3. Build branch node with both children. */
	{
		struct cds_ft_inode_flag *dest = NULL;
		struct cds_ft_metadata *branch_meta;

		/* First child: old direction. */
		ret = ft_node_set_nth(ft, &dest, old_ordinal, old_suffix_flag, NULL, NULL,
				junction_depth, branch_cluster_leaf);
		if (ret) goto error;
		created[nr_created++] = dest;
		branch_flag = dest;

		/* Second child: new direction. */
		{
			struct cds_ft_inode *old_recompacted = NULL;

			branch_meta = cds_ft_item_to_metadata(ft_node_ptr(dest));
			ret = ft_node_set_nth(ft, &dest, new_ordinal, new_branch_flag,
					&old_recompacted, branch_meta, junction_depth,
					branch_cluster_leaf);
			if (ret) goto error;
			if (old_recompacted) {
				free_cds_ft_node(ft, old_recompacted);
				/* Update created entry to the recompacted node. */
				created[nr_created - 1] = dest;
			}
		}

		branch_meta = cds_ft_item_to_metadata(ft_node_ptr(dest));
		ft_nr_keys_store(ft,branch_meta, old_child_nr_keys + 1,
				CMM_RELAXED);
		branch_flag = dest;

		/*
		 * sfx was installed via its PLAIN flag so ft_set_parent could
		 * recover it directly (without reading cn->child's back-pointer,
		 * which still points at the old cn).  Re-encode the old-direction
		 * slot to sfx's skip form now -- a value write into the still-
		 * unpublished branch; it becomes recoverable once cn->child's
		 * back-pointer is set at publish.
		 */
		if (suffix_len >= 1 && sfx_skip_flag != old_suffix_flag) {
			struct cds_ft_inode_flag **oslot = NULL;

			ft_node_get_nth_skip(branch_flag, &oslot, old_ordinal,
					FT_PF_NONE);
			if (oslot)
				*oslot = sfx_skip_flag;
		}

		/*
		 * suffix_len == 0: the branch is a cluster-leaf, so set_nth left
		 * both of its children unparented.  Record both deferred edges
		 * (old = live cn->child, new = the new subtree) against the now-
		 * final branch_flag; phase 2 wires them.  The forward slots are
		 * already correct (value-copied by set_nth / recompaction).
		 */
		if (branch_cluster_leaf) {
			ft_node_get_nth_skip(branch_flag, &deferred_slot,
					old_ordinal, FT_PF_NONE);
			ft_node_get_nth_skip(branch_flag, &deferred_slot2,
					new_ordinal, FT_PF_NONE);
			deferred_child = old_child_flag;
			deferred_parent = branch_flag;
			deferred_child2 = new_branch_flag;
		}
	}

	/*
	 * 4. Build prefix -> branch (if needed).  @cn carries no
	 * external_nodes (asserted at entry), so the prefix is always a
	 * plain compressed run over key_bytes[0 .. diverge_pos).
	 */
	if (diverge_pos >= 1) {
		struct cds_ft_compressed_node *pfx;
		struct cds_ft_metadata *pfx_meta;
		struct cds_ft_inode_flag *pfx_child;

		pfx = alloc_compressed_node(ft, diverge_pos, &pfx_meta);
		if (!pfx) goto error;
		pfx->child = branch_flag;
		pfx->len = diverge_pos;
		memcpy(pfx->key_bytes, cn->key_bytes, diverge_pos);
		ft_meta_nr_child_set(pfx_meta, 1);
		ft_nr_keys_store(ft,pfx_meta, ft_nr_keys_get(cn_meta) + 1, CMM_RELAXED);
		pfx_child = ft_compressed_node_flag(pfx);
		ft_set_parent(ft, branch_flag, pfx_child, &pfx->child);
		pfx_child = ft_publish_compressed(ft, pfx, pfx_child);
		created[nr_created++] = pfx_child;
		top_flag = pfx_child;
	} else {
		/* diverge_pos == 0: branch IS the top. */
		struct cds_ft_metadata *branch_meta =
			cds_ft_item_to_metadata(ft_node_ptr(branch_flag));
		ft_nr_keys_store(ft,branch_meta, ft_nr_keys_get(cn_meta) + 1,
				CMM_RELAXED);
		top_flag = branch_flag;
	}

	/* 5. Publish the split structure, replacing the compressed node. */
	FT_TP(compressed_split, "insert", (const void *) cn, cn->len,
		(const void *) top_flag, diverge_pos);
	/*
	 * Compressed-split replaces the compressed node in its parent's
	 * slot via a direct ft_publish_to_parent call, bypassing
	 * ft_node_set_nth.  Emit tree_edge_set explicitly so consumers
	 * see the (parent, key_byte, top_flag) structural edge.  The
	 * compressed node sits at node_depth; iter_key points at the
	 * key bytes starting at that depth, so iter_key[-1] is the
	 * parent's key_byte that led to the compressed node (safe for
	 * node_depth >= 1, which always holds since compressed nodes
	 * are never at the root).
	 */
	FT_TP(tree_edge_set, (const void *) ft,
		(const void *) ft_parent_node(CMM_LOAD_SHARED(cn_meta->parent_word)),
		(unsigned int) (node_depth - 1),
		(uint8_t) iter_key[-1],
		(const void *) top_flag);
	/*
	 * Phase 2 (publish) -- no failures past here.  Wire every back-pointer
	 * before swinging the parent's forward slot, so an up-walk that lands
	 * on the new cluster from either direction sees the back-pointers wired
	 * before the cluster becomes reader-reachable.
	 *
	 * ORDER among the deferred edges matters.  deferred_child is always the
	 * LIVE old child (cn->child) re-parented into the cluster; setting its
	 * back-pointer is itself a back-channel publish -- a reader up-walking
	 * from cn->child immediately enters the new cluster and can then scan
	 * the cluster's other (sibling) slots.  deferred_child2 is the FRESH
	 * new subtree, observable only through the cluster.  So wire the cluster
	 * top's own back-pointer and the fresh edge FIRST, and the live edge
	 * LAST: otherwise a reader entering via the live child reads a sibling
	 * slot pointing at the fresh subtree whose parent is not yet set, and
	 * its consume chain -- anchored at the live back-pointer store -- has no
	 * happens-before edge to the later fresh-parent store, so it observes a
	 * stale NULL parent (ft_skip_reanchor holder == NULL).
	 *
	 * suffix_len >= 1: only the live edge exists (cn->child -> sfx).
	 * suffix_len == 0: cluster-leaf branch's two children (live cn->child
	 * and fresh new subtree), both -> branch_flag.
	 */
	/*
	 * I3 count fold reservation: the diverge caller records the +1 count
	 * walk from the STABLE parent of the split compressed node (d->pnf, whose
	 * slot @parent_slot is what this commit flips) up to the root -- at most
	 * @node_depth ancestor edges.  Every fresh split node already carries its
	 * full post-commit count, so the walk never touches one.  Reserve by the
	 * actual node depth (0 when rank stats are off).
	 */
	/*
	 * Concurrent-writer conflict check (Phase 4.3).  @parent_slot was recorded
	 * at descent as cn's slot in its parent; a peer writer may since have
	 * re-parented cn -- split cn's parent and re-homed cn under a fresh
	 * junction -- so cn_meta->parent and @parent_slot now disagree.
	 * ft_set_parent below would invert that (parent, slot) pair against the
	 * wrong node's bitmap and fault.  cn's CURRENT slot (recomputed from
	 * cn_meta) must still be @parent_slot; if not, signal a re-descend.  The
	 * fresh cluster is unreachable, so free it now.  (The residual commit-time
	 * window -- cn re-parented AFTER this check but before the forward install
	 * -- is caught by the forward slot's expected-value CAS and the §4.B freeze
	 * guard, both recorded by ft_insert_publish_or_park, surfacing as ABORT.)
	 *
	 * Capture @cur_parent from the SAME resolved snapshot that validates the
	 * slot: a peer re-home commits cn_meta->parent and the state-word slot
	 * offset as ONE co-committed MCAS pair, so a separate raw re-read of the
	 * parent below could return a peer's freshly re-homed parent P2 while
	 * @parent_slot still indexes the old parent P1 -- ft_set_parent would then
	 * invert (P2, @parent_slot) and fault ft_slot_to_byte with an out-of-range
	 * rank against P2's bitmap.  One snapshot keeps the pair consistent; a
	 * re-home landing AFTER it is caught by the commit CAS.
	 */
	if (ft_resolve_parent_slot(cn_meta, ft, &cur_parent) != parent_slot) {
		ft_free_unpublished_split_cluster(ft, created, nr_created);
		ft_meta_lock_release(held.lock);
		ft_insert_dlm_release_parent(ic);	/* release P held up front */
		return -EAGAIN;
	}
	/*
	 * Hand the fresh cluster to the op scope so the commit's on-abort rollback
	 * frees it if a peer writer wins the forward slot at commit time.
	 */
	memcpy(ic->created, created, (size_t) nr_created * sizeof(created[0]));
	ic->nr_created = nr_created;

	ret = ft_insert_commit_arm(ft, ic,
		ft->rank_stats ? node_depth + 2 : 0);
	if (ret)
		goto error;
	/*
	 * The armed txn now owns the fence outcome: registered for the clear on
	 * every non-commit terminal (destroy / ABORT / MEMORY_ERROR), consumed
	 * by the fenced tombstone transition on commit.  Past this point the
	 * local error path must NOT clear it (the caller's unwind destroys
	 * @ic->txn, which drains the registry).
	 */
	ft_flip_txn_lock_register(ic->txn, held.lock, held.lock_snap);
	ft_flip_txn_record_anchor_release(ic->txn, &held, cn_meta);
	ic->free_old_cn_held = held;
	/*
	 * @cur_parent came from the guard's ft_resolve_parent_slot snapshot,
	 * consistent with @parent_slot (a peer re-home commits the parent and its
	 * slot offset as one MCAS pair, resolved from a single status snapshot, so
	 * the pair cannot tear).  Do NOT re-read cn_meta->parent here: a raw reload
	 * could observe a peer's re-homed parent while @parent_slot still indexes
	 * the old one, faulting ft_slot_to_byte on the mismatched bitmap.  A flip
	 * proxy is already resolved by ft_resolve_parent_slot (never the type-7
	 * latch itself).  If the peer re-homes cn AFTER the guard, our forward CAS
	 * / §4.B guard aborts and this fresh cluster is discarded.
	 */
	ft_set_parent(ft, top_flag, cur_parent, parent_slot);
	if (deferred_child2)
		ft_set_parent(ft, deferred_child2, deferred_parent, deferred_slot2);
	if (deferred_child) {
		/*
		 * deferred_child is the LIVE old child (cn->child).  Setting its
		 * back-pointer is the back-channel publish that exposes this fresh
		 * cluster to a reanchor up-walker, so defer it to the parked
		 * one-commit: ft_insert_one_commit parks it into the SAME flip
		 * batch (structure + cell + this edge flip atomically; the splice
		 * search runs from the root so it does not need it wired early).
		 */
		ic->live_child = deferred_child;
		ic->live_parent = deferred_parent;
		ic->live_slot = deferred_slot;
	}
#ifdef FEATURE_FT_PROBE_EMPTY_INSERT
	/* A/B: never wire an empty internal under a live parent (see the note). */
	__atomic_fetch_add(&cds_ft_probe_reach_split, 1, __ATOMIC_RELAXED);
	if (cur_parent && ft_probe_internal_is_empty(ft, top_flag)) {
		__atomic_fetch_add(&cds_ft_probe_empty_publish_split, 1,
			__ATOMIC_RELAXED);
		ret = -EAGAIN;
		goto error;
	}
#endif
	{
		struct ft_lock_ctx pctx;

		/*
		 * @cur_parent came from CN's back-pointer, so the descent's
		 * window is what dates it.
		 */
		ft_lock_ctx_init(&pctx, dsc, ic ? ic->txn : NULL, ic ? ic->op : NULL);
		ft_insert_publish_or_park(ft, &pctx, cur_parent,
			FT_DEPTH_FROM_DESCENT, parent_slot, top_flag,
			fwd_expected_old, ic);
	}

	/*
	 * 7. Free the old compressed node.  Parked publish: readers resolve
	 * the proxy to @cn until the commit, so defer the free past it.
	 */
	ic->free_old_cn = cn;

	return 0;

error:
	/*
	 * Reached only BEFORE the post-arm registration (build allocation
	 * failures, or the arm itself failing with @ic->txn never created), so
	 * the fence is still locally owned: lift it with the cluster teardown.
	 * The DLM hoist also holds P up front (publish_or_park not reached here),
	 * so release it too.
	 */
	ft_free_unpublished_split_cluster(ft, created, nr_created);
	ft_meta_lock_release(held.lock);
	ft_insert_dlm_release_parent(ic);
	return -ENOMEM;
}


/*
 * Split a compressed node when the insert key is shorter than the
 * compressed path (key terminates within the path).
 *
 * Builds: [prefix] -> [junction] -> [suffix] -> old_child
 *
 * The junction is an internal node at the key endpoint depth with
 * one child (the suffix direction).  The caller stores the new
 * node as external_nodes on the junction and handles publication,
 * propagation, and freeing of the old compressed node.
 *
 * @parent_slot: address of the slot in cn's parent that holds cn.  Used
 * to wire top_flag's own back-pointer into the live parent BEFORE the
 * deferred (back-channel) re-parent of cn->child into the new suffix.
 * Otherwise an up-walk from cn->child enters the new cluster and walks
 * up to top_flag, which would have parent == NULL.
 *
 * On success, sets *top_ret to the topmost node (prefix or junction)
 * and *jct_ret to the junction node.  Returns 0.
 * On failure, frees any partially created nodes and returns -ENOMEM.
 */
static
int ft_split_compressed_key_shorter(struct cds_ft *ft,
		struct cds_ft_inode_flag *compressed_flag,
		struct cds_ft_inode_flag **parent_slot,
		unsigned int remaining,
		struct cds_ft_inode_flag **top_ret,
		struct cds_ft_inode_flag **jct_ret,
		unsigned int node_depth,
		/*
		 * The LIVE old-child re-parent edge (cn->child into the new
		 * suffix/junction) is RETURNED, not wired: the caller defers it to
		 * the parked one-commit (so it flips atomically with the cell) or
		 * wires it directly (list-off / duplicate).  See the same shape in
		 * ft_split_compressed_insert.  NULL out = no live edge.
		 */
		struct cds_ft_inode_flag **live_child_ret,
		struct cds_ft_inode_flag **live_parent_ret,
		struct cds_ft_inode_flag ***live_slot_ret,
		/*
		 * Out: the freshly-built (still-unpublished) cluster nodes, so the
		 * caller can tear it down on a post-return abort.  Optional (NULL).
		 */
		struct cds_ft_inode_flag **created_ret,
		int *nr_created_ret)
{
	struct cds_ft_compressed_node *cn = ft_compressed_node_ptr(compressed_flag);
	struct cds_ft_metadata *cn_meta =
		cds_ft_item_to_metadata((struct cds_ft_inode *) cn);
	unsigned int suffix_len = cn->len - remaining - 1;

	/* Compressed metadata never carries external_nodes (see
	 * ft_split_compressed_insert); the prefix builders rely on it. */
	assert(!cn_meta->external_nodes);
	struct cds_ft_inode_flag *suffix_flag;
	struct cds_ft_inode_flag *jct_flag;
	struct cds_ft_inode_flag *top_flag;
	struct cds_ft_inode_flag *created[FT_MAX_DEPTH];
	int nr_created = 0;
	unsigned long child_nr_keys;
	int ret;
	/*
	 * Build-invisible / publish / reclaim (see rcu-mutation pattern).
	 * The live old child (cn->child) is re-parented into the new suffix or,
	 * for suffix_len == 0, straight into the junction.  Defer that single
	 * back-pointer to the failure-free tail so a later allocation failure
	 * frees the never-observed cluster with cn->child untouched.  The caller
	 * publishes the top right after we return, so this deferred (bottom)
	 * publish precedes the top forward publish.
	 */
	uint8_t jct_ordinal = cn->key_bytes[remaining];
	bool jct_cluster_leaf = (suffix_len == 0);
	struct cds_ft_inode_flag *sfx_skip_flag = NULL;
	struct cds_ft_inode_flag *deferred_child = NULL;
	struct cds_ft_inode_flag *deferred_parent = NULL;
	struct cds_ft_inode_flag **deferred_slot = NULL;

	/*
	 * ONE resolved snapshot of the live cn->child (§9) -- see
	 * ft_split_compressed_insert: a raw read may hand a peer's parked
	 * flip proxy to the kind dispatch, and re-reads could tear across
	 * the peer's flip.  All uses below consume this value; a
	 * post-snapshot peer commit aborts ours at the forward CAS.
	 */
	struct cds_ft_inode_flag *old_child_flag =
		ft_cn_child_dereference_acquire_prefetch(cn);

	if (!ft_node_external(old_child_flag)) {
		struct cds_ft_metadata *cm =
			cds_ft_item_to_metadata(ft_node_ptr(old_child_flag));
		child_nr_keys = ft_nr_keys_get(cm);
	} else if (old_child_flag) {
		child_nr_keys = 1;
	} else {
		child_nr_keys = 0;
	}

	/*
	 * Build suffix -> old child.
	 *
	 * Under SKIP_COMPRESSED, suffix_len >= 1 must produce a
	 * compressed (skip-encoded) node; a 1-child internal at this
	 * level would violate the chain-compress invariant (verify
	 * rejects it).  Under non-SC mode, suffix_len == 1 historically
	 * produced an internal node -- that's still acceptable since the
	 * invariant only applies in skip mode.
	 */
	if (suffix_len >= 2
#ifdef FEATURE_FT_SKIP_COMPRESSED
			|| (suffix_len == 1 && ft_group_skip_compressed(ft->group))
#endif
	   ) {
		struct cds_ft_compressed_node *sfx;
		struct cds_ft_metadata *sfx_meta;

		sfx = alloc_compressed_node(ft, suffix_len, &sfx_meta);
		if (!sfx) goto error;
		sfx->child = old_child_flag;
		sfx->len = suffix_len;
		memcpy(sfx->key_bytes, &cn->key_bytes[remaining + 1],
			suffix_len);
		ft_meta_nr_child_set(sfx_meta, 1);
		ft_nr_keys_store(ft,sfx_meta, child_nr_keys,
			CMM_RELAXED);
		suffix_flag = ft_compressed_node_flag(sfx);	/* PLAIN: install + recover sfx directly */
		sfx_skip_flag = ft_publish_compressed(ft, sfx, suffix_flag);	/* skip form for the slot */
		created[nr_created++] = suffix_flag;	/* track PLAIN so the error path frees sfx directly */
		/*
		 * Defer re-parenting the live old child into the new suffix:
		 * writing cn->child's back-pointer now would expose the
		 * unpublished cluster from below, and the junction install
		 * below would recover sfx through it.  sfx->child already
		 * points at cn->child (a write into the new sfx only).
		 */
		deferred_child = old_child_flag;
		deferred_parent = suffix_flag;	/* PLAIN sfx flag */
		deferred_slot = &sfx->child;
	} else if (suffix_len == 1) {
		struct cds_ft_inode_flag *dest = NULL;

		/*
		 * 1-child internal suffix (non-SC): the live cn->child is its
		 * only child, so this node is a cluster-leaf -- defer cn->child's
		 * back-pointer.
		 */
		ret = ft_node_set_nth(ft, &dest,
			cn->key_bytes[remaining + 1],
			old_child_flag, NULL, NULL,
			node_depth + remaining + 1, true);
		if (ret) goto error;
		{
			struct cds_ft_metadata *m =
				cds_ft_item_to_metadata(ft_node_ptr(dest));
			ft_nr_keys_store(ft,m, child_nr_keys,
				CMM_RELAXED);
		}
		suffix_flag = dest;
		created[nr_created++] = dest;
		deferred_child = old_child_flag;
		deferred_parent = dest;
		ft_node_get_nth_skip(dest, &deferred_slot,
			cn->key_bytes[remaining + 1], FT_PF_NONE);
	} else {
		suffix_flag = old_child_flag;
	}

	/* Junction: internal node with suffix child. */
	{
		struct cds_ft_inode_flag *dest = NULL;
		struct cds_ft_metadata *jct_meta;

		ret = ft_node_set_nth(ft, &dest,
			jct_ordinal,
			suffix_flag, NULL, NULL,
			node_depth + remaining, jct_cluster_leaf);
		if (ret) goto error;
		jct_meta = cds_ft_item_to_metadata(ft_node_ptr(dest));
		/*
		 * I4 count fold: the junction holds the NEW key as its
		 * external_nodes (attached by the caller before publish), so
		 * build it with its FULL post-commit count -- the suffix
		 * subtree (child_nr_keys) PLUS that new key -- rather than the
		 * bare suffix count that relied on a post-commit +1 walk from
		 * the junction.  With the junction (and prefix below) built
		 * full, the fresh cluster is off the count walk and the +1 rides
		 * the commit as edges on the STABLE ancestors from d->pnf up.
		 */
		ft_nr_keys_store(ft,jct_meta, child_nr_keys + 1,
			CMM_RELAXED);
		jct_flag = dest;
		created[nr_created++] = dest;

		if (jct_cluster_leaf) {
			/*
			 * suffix_len == 0: the junction is the cluster-leaf and
			 * its suffix-direction child is the live cn->child.
			 * Record the deferred edge against the final junction.
			 */
			deferred_child = old_child_flag;
			deferred_parent = jct_flag;
			ft_node_get_nth_skip(jct_flag, &deferred_slot,
				jct_ordinal, FT_PF_NONE);
		} else if (sfx_skip_flag && sfx_skip_flag != suffix_flag) {
			/*
			 * suffix_len >= 1 compressed: sfx was installed via its
			 * PLAIN flag so ft_set_parent recovered it directly.
			 * Re-encode the junction's slot to sfx's skip form -- a
			 * value write into the still-unpublished junction; it
			 * resolves once cn->child's back-pointer is set at the tail.
			 */
			struct cds_ft_inode_flag **oslot = NULL;

			ft_node_get_nth_skip(jct_flag, &oslot, jct_ordinal,
				FT_PF_NONE);
			if (oslot)
				*oslot = sfx_skip_flag;
		}
	}

	/* Prefix -> junction (no external_nodes on @cn: asserted at entry). */
	if (remaining >= 2) {
		struct cds_ft_compressed_node *pfx;
		struct cds_ft_metadata *pfx_meta;

		pfx = alloc_compressed_node(ft, remaining, &pfx_meta);
		if (!pfx) goto error;
		pfx->child = jct_flag;
		pfx->len = remaining;
		memcpy(pfx->key_bytes, cn->key_bytes, remaining);
		ft_meta_nr_child_set(pfx_meta, 1);
		ft_nr_keys_store(ft,pfx_meta, ft_nr_keys_get(cn_meta) + 1,
			CMM_RELAXED);
		top_flag = ft_compressed_node_flag(pfx);
		ft_set_parent(ft, jct_flag, top_flag, NULL);
		top_flag = ft_publish_compressed(ft, pfx, top_flag);
		created[nr_created++] = top_flag;
	} else if (remaining == 1) {
#ifdef FEATURE_FT_SKIP_COMPRESSED
		if (ft_group_skip_compressed(ft->group)) {
			/*
			 * 1-byte prefix: emit a 1-byte compressed instead of a
			 * 1-child internal (canonical form under
			 * SKIP_COMPRESSED).
			 */
			struct cds_ft_compressed_node *pfx;
			struct cds_ft_metadata *pfx_meta;

			pfx = alloc_compressed_node(ft, 1, &pfx_meta);
			if (!pfx) goto error;
			pfx->child = jct_flag;
			pfx->len = 1;
			pfx->key_bytes[0] = cn->key_bytes[0];
			ft_meta_nr_child_set(pfx_meta, 1);
			ft_nr_keys_store(ft,pfx_meta, ft_nr_keys_get(cn_meta) + 1,
				CMM_RELAXED);
			top_flag = ft_compressed_node_flag(pfx);
			ft_set_parent(ft, jct_flag, top_flag, &pfx->child);
			top_flag = ft_publish_compressed(ft, pfx, top_flag);
			created[nr_created++] = top_flag;
		} else
#endif
		{
			struct cds_ft_inode_flag *dest = NULL;
			struct cds_ft_metadata *pfx_meta;

			ret = ft_node_set_nth(ft, &dest, cn->key_bytes[0],
				jct_flag, NULL, NULL, node_depth, false);
			if (ret) goto error;
			pfx_meta = cds_ft_item_to_metadata(ft_node_ptr(dest));
			ft_nr_keys_store(ft,pfx_meta, ft_nr_keys_get(cn_meta) + 1,
				CMM_RELAXED);
			top_flag = dest;
			created[nr_created++] = dest;
		}
	} else {
		/* remaining == 0: no prefix, junction IS the top. */
		top_flag = jct_flag;
	}

	/*
	 * Failure-free tail.  First wire top_flag's own back-pointer into
	 * cn's live parent (so an up-walk that enters the cluster via the
	 * deferred back-channel below finds a parent-wired top), THEN wire
	 * the single deferred back-pointer (the live old child into the new
	 * suffix / junction).  No allocation happens past here; the caller
	 * publishes the top forward immediately after we return.
	 *
	 * Wire top_flag's back-pointer from ONE resolved (parent, slot) snapshot
	 * (§9 / see ft_split_compressed_insert): a peer re-home commits
	 * cn_meta->parent and its slot offset as one MCAS pair, so a raw re-read
	 * of the parent could pair a re-homed parent P2 with @parent_slot (still
	 * indexing the old parent P1) and fault ft_slot_to_byte with an
	 * out-of-range rank against P2's bitmap.  Only wire when the resolved slot
	 * still IS @parent_slot (the consistent, non-re-homed pair, with any flip
	 * proxy already resolved); a re-home makes the pair disagree -> skip (the
	 * cluster stays build-invisible) and the caller's post-build (parent,
	 * slot) guard unwinds the whole insert with -EAGAIN before any publish.  A
	 * re-home AFTER that guard is caught by the forward CAS / §4.B as ABORT.
	 */
	{
		struct cds_ft_inode_flag *cur_parent;

		if (ft_resolve_parent_slot(cn_meta, ft, &cur_parent) == parent_slot)
			ft_set_parent(ft, top_flag, cur_parent, parent_slot);
	}
	/* Return the live edge; the caller defers (parked) or wires (direct). */
	*live_child_ret = deferred_child;
	*live_parent_ret = deferred_parent;
	*live_slot_ret = deferred_slot;
	/*
	 * Hand the built (still-unpublished) cluster back so the caller can tear
	 * it down if it must abort after we return (a one-commit arm -ENOMEM):
	 * created[] holds exactly the fresh cluster nodes, never the live
	 * cn->child.
	 */
	if (created_ret) {
		memcpy(created_ret, created,
			(size_t) nr_created * sizeof(created[0]));
		*nr_created_ret = nr_created;
	}

	FT_TP(compressed_split, "key_shorter", (const void *) cn, cn->len,
		(const void *) top_flag, remaining);
	*top_ret = top_flag;
	*jct_ret = jct_flag;
	return 0;

error:
	ft_free_unpublished_split_cluster(ft, created, nr_created);
	return -ENOMEM;
}

/*
 * We reached an unpopulated node. Create it and the children we need,
 * and then attach the entire branch to the current node. This may
 * trigger recompaction of the current node.
 *
 * ft_attach_node() ensures that a lookup will _never_ see a branch that
 * leads to a dead-end: before attaching a branch, the entire content of
 * the new branch is populated, thus creating a cluster, before
 * attaching the cluster to the rest of the trie, thus making it visible
 * to lookups.
 *
 * @external_node argument is either NULL or a pointer to the external
 * node we are replacing at the attachment location. We need to chain
 * this external node in the topmost internal node external node list in
 * that case.
 */
static
int ft_attach_node(struct cds_ft *ft,
		struct cds_ft_inode_flag **attach_node_flag_ptr,
		struct cds_ft_inode_flag *attach_node_flag,
		struct cds_ft_inode_flag **old_node_flag_ptr,
		struct cds_ft_inode_flag *old_node_flag,
		const uint8_t *key,
		size_t key_len,
		unsigned int level,
		struct cds_ft_node *child_node,
		struct cds_ft_node *external_nodes,
		struct ft_insert_commit *ic,
		const struct ft_lock_ctx *ctx)
{
	struct cds_ft_metadata *metadata = NULL;
	struct cds_ft_inode_flag *iter_node_flag, *iter_dest_node_flag,
				*created_nodes[FT_MAX_DEPTH];
	struct cds_ft_inode *old_recompacted_node = NULL;
	int ret, i, nr_created_nodes = 0;
	const uint8_t *iter_key = key + key_len;

	FT_TP(attach_node_enter, (const void *) attach_node_flag,
		(const void *) old_node_flag, level);

	dbg_printf("Attach node at level %u (old_node_flag %p, attach_node_flag_ptr %p attach_node_flag %p)\n",
		level, old_node_flag, attach_node_flag_ptr, attach_node_flag);

	assert(!old_node_flag || external_nodes);
	assert(level > 0);	/* Root is always internal; level 0 is handled directly. */
	if (attach_node_flag)
		metadata = cds_ft_item_to_metadata(ft_node_ptr(attach_node_flag));

	/*
	 * Concurrent-writer conflict detection (was a single-writer "concurrent
	 * update prevented by mutual exclusion" assert, false under MW): a peer
	 * mutated the old / attach slot between this writer's descent and here --
	 * the old slot now holds a live node with nowhere to chain it, or the
	 * attach slot no longer matches the descent-captured node (a peer's flip
	 * proxy also reads != the captured node here, so it too routes to retry).
	 * Bail to the from-root restart_attempt rather than build against a stale
	 * slot: nothing is armed or built yet (nr_created_nodes == 0), so
	 * check_error is a clean unwind.  The commit's forward-CAS + parent guard
	 * would surface the same conflict as ABORT; failing fast here just skips a
	 * doomed build.  No-op under retained exclusion.
	 */
	if ((old_node_flag_ptr && ft_node_ptr_raw(*old_node_flag_ptr) && !external_nodes) ||
			(attach_node_flag_ptr && ft_node_ptr_raw(*attach_node_flag_ptr) !=
				ft_node_ptr_raw(attach_node_flag))) {
		ret = -EAGAIN;
		goto check_error;
	}

	/* Create new branch, starting from bottom */
	iter_node_flag = (struct cds_ft_inode_flag *) child_node;

	{
		struct cds_ft_inode_flag *compressed;
		/*
		 * Compressed nodes must not carry metadata->external_nodes.
		 * When external_nodes exist at @level, compress from
		 * level+1 (one byte shorter) and let the loop below
		 * create an internal node at @level that holds the
		 * external_nodes.
		 */
		unsigned int compress_level = external_nodes ? level + 1 : level;

		compressed = ft_try_compress_chain(ft, key, key_len,
			compress_level, iter_node_flag, NULL, NULL);
		if (compressed == (void *) (long) -ENOMEM) {
			ret = -ENOMEM;
			goto check_error;
		}
		if (compressed) {
			iter_node_flag = compressed;
			/*
			 * Track the PLAIN compressed flag, never the skip form
			 * ft_try_compress_chain returns in skip-compressed
			 * groups: a SKIP pointer encodes the CHILD's address
			 * (the application's external node for a leaf attach),
			 * so the kind dispatch in check_error's unwind would
			 * misread it as a plain node and run arena arithmetic
			 * on the application's pointer.  Same convention as the
			 * glue builders (see ft_try_compress_chain's glue arm).
			 */
			created_nodes[nr_created_nodes++] =
				ft_node_skip_compressed(compressed) ?
				ft_compressed_node_flag(
					ft_skip_to_compressed(ft, compressed)) :
				compressed;
			iter_key = key + compress_level;
			/*
			 * When external_nodes exist, compress_level = level + 1.
			 * iter_key points to key + level + 1.  The loop below
			 * runs one iteration to create an internal node at
			 * @level that dispatches on key[level] with the
			 * compressed node as child.  The loop then places
			 * external_nodes on this internal node.
			 */
		}
	}
	if ((!ft_node_compressed(iter_node_flag) &&
	     !ft_node_skip_compressed(iter_node_flag)) ||
	    external_nodes) {
		for (i = (ft_node_compressed(iter_node_flag) ||
			  ft_node_skip_compressed(iter_node_flag)) ?
				(int) level + 1 : (int) key_len;
		     i > (int) level; i--) {
			uint8_t key_value;

			key_value = *(--iter_key);
			dbg_printf("branch creation level %d, key %u\n",
					i, (unsigned int) key_value);
			iter_dest_node_flag = NULL;
			ret = ft_node_set_nth(ft, &iter_dest_node_flag, key_value, iter_node_flag, NULL, NULL,
					i - 1, false);
			if (ret) {
				dbg_printf("branch creation error %d\n", ret);
				goto check_error;
			}
			{
				struct cds_ft_metadata *branch_meta =
					cds_ft_item_to_metadata(ft_node_ptr(iter_dest_node_flag));
				ft_nr_keys_store(ft,branch_meta, 1, CMM_RELAXED);
			}
			created_nodes[nr_created_nodes++] = iter_dest_node_flag;
			iter_node_flag = iter_dest_node_flag;
		}

		if (external_nodes) {
			struct cds_ft_metadata *iter_node_metadata;

			iter_node_metadata = cds_ft_item_to_metadata(ft_node_ptr(iter_node_flag));
			/*
			 * Phase 1 (build-invisible): write the cluster top's
			 * cluster-internal external_nodes pointer.  The
			 * back-channel publish (external_nodes->prev =
			 * iter_node_flag) is deferred to Phase 2 below, after
			 * set_nth wires iter_node_flag's parent -- otherwise an
			 * up-walk from external_nodes (still reachable through
			 * the old slot at attach_node_flag_ptr) lands on
			 * iter_node_flag with parent == NULL.
			 */
			ft_metadata_set_external_nodes(iter_node_flag,
				iter_node_metadata, external_nodes);
			ft_nr_keys_store(ft,iter_node_metadata,
				ft_nr_keys_get(iter_node_metadata) + 1, CMM_RELAXED);
		}
	}

	/* Publish branch. */
	{
		uint8_t key_value;
		/*
		 * Accumulates the one-commit forward edges for a recompact-
		 * relocating reserve: the reserve set_nth records a compressed
		 * parent's SKIP_X dual here, and the forward fold below adds the
		 * grandparent slot edge, so both flip ATOMICALLY in ic->txn.
		 */
		struct ft_pub_rec rec = { .n = 0 };

		key_value = *(--iter_key);
		dbg_printf("publish branch at level %d, key %u\n", level - 1, (unsigned int) key_value);

		/*
		 * Reserve the I6 count-fold edges optimistically: if this attach
		 * stays in place (the common case, iter_dest_node_flag ==
		 * attach_node_flag below), the +1 key-count walk from the stable
		 * attach node up to the root rides this txn.  Bounded by the ACTUAL
		 * descent depth (@level >= the attach node's depth, +2 slack for the
		 * root), never FT_MAX_DEPTH.  A relocation (I7) records no count
		 * edges here (count_folded stays false) so the slack is simply
		 * unused.  A no-op reserve (0) when order statistics are off.
		 */
		ret = ft_insert_commit_arm(ft, ic,
			ft->rank_stats ? level + 2 : 0);
		if (ret)
			goto check_error;

		/* We need to use set_nth on the previous level. */
		iter_dest_node_flag = attach_node_flag;
		/*
		 * Every ft_attach_node caller passes a non-NULL @ic and
		 * ft_insert_commit_arm has armed ic->txn above (an arm failure
		 * jumps to check_error), so the one-commit txn is always present
		 * here -- the reserved-byte publish below is unconditional.
		 */
		assert(ic && ic->txn);
		{
			struct cds_ft_inode_flag **slot_ptr = NULL;
			/*
			 * Set by the reserve below iff it added the byte's
			 * occupancy IN PLACE on the LIVE attach node, i.e. this
			 * op still owes that node an nr_child++.  A reserve that
			 * RELOCATED (recompact) built the count into its fresh
			 * copy instead and leaves this false.
			 */
			bool count_deferred = false;

			/*
			 * One-commit insert (reserved-byte model): occupy
			 * key_value's slot but leave it resolving to its OLD
			 * value, then RECORD the slot edge (old -> the fresh
			 * top) into ic->txn so it settles atomically with the
			 * ordered-list edges at insert_done.  Nothing foreign is
			 * stored in the slot during the build -- the fresh key
			 * stays invisible because the slot still reads its old
			 * value (NULL for a new byte, or the displaced external).
			 *
			 *  - new byte (old_node_flag == NULL): RESERVE it via a
			 *    set_nth with a NULL child -- sets the bitmap bit,
			 *    leaving a bit-set+NULL slot that reads as
			 *    not-present, and may recompact + relocate the node
			 *    (the reserved byte rides along; the edge is
			 *    recorded against the FINAL slot below).  Its
			 *    nr_child++ is NOT applied in place: an in-place
			 *    reserve hands it back via @count_deferred and it is
			 *    recorded into ic->txn below, so it goes live with
			 *    the publish and is discarded with an aborted
			 *    attempt (see ft_flip_txn_record_nr_child_inc).
			 *  - displaced external (old_node_flag != NULL): the slot
			 *    already holds it; no set_nth (a NULL store would drop
			 *    the live external before the commit).
			 *
			 * Then fully wire the fresh top NOW (parent, slot offset,
			 * incoming_byte) -- invisible while the slot reads old --
			 * so the splice-position search at insert_done can rebuild
			 * the new key through the parent chain.
			 */
			if (!old_node_flag) {
				ret = ft_node_set_nth_rec(ft, &iter_dest_node_flag,
					key_value, NULL, &old_recompacted_node,
					metadata, level - 1, false, &rec, ic->txn,
					NULL, ctx, &count_deferred);
				if (ret) {
					dbg_printf("branch publish error %d\n", ret);
					goto check_error;
				}
			}
			/*
			 * The in-place reserve owes @metadata (the LIVE attach
			 * node) its nr_child++: record it as an edge in ic->txn,
			 * immediately before -- and on the SAME state word as --
			 * the §4.B guard below, so the two chain into ONE record
			 * that the arm's guard reservation already covers.
			 * Deferring is what makes reserve-then-ABORT idempotent:
			 * the count is dropped with the attempt instead of
			 * leaking a phantom child that the retry double-counts.
			 */
			assert(!count_deferred ||
				iter_dest_node_flag == attach_node_flag);
			ft_node_get_nth_skip(iter_dest_node_flag, &slot_ptr,
				key_value, FT_PF_NONE);
			assert(slot_ptr);
			ft_set_parent(ft, iter_node_flag, iter_dest_node_flag,
				slot_ptr);
			/*
			 * §4.B VALIDATE (Phase 4.3, MW): the reserved-byte edge
			 * below stores @slot_ptr, a slot INSIDE @iter_dest_node_flag.
			 * When the reserve stayed IN PLACE (iter_dest == attach node)
			 * that slot lives in the LIVE, reader-reachable attach node,
			 * yet the forward CAS below validates only the slot VALUE
			 * (NULL for a new byte) -- a peer that recompacts/relocates
			 * the attach node through its grandparent slot leaves that
			 * value intact in the retired copy, so the CAS still matches
			 * and commits the new key into a reclaimed node (UAF + lost
			 * insert).  Guard the holder's state word so a peer's
			 * freeze-on-free (LOCK/TOMBSTONE) between this descent and
			 * the commit ABORTs instead -- symmetric with the split path
			 * (ft_insert_publish_or_park) and the external-clear guards in
			 * ft-remove.h.  A RELOCATION reserve publishes a build-
			 * invisible fresh copy, guarded at its grandparent slot below
			 * (iter_dest != attach), so it needs no guard here.
			 */
			/*
			 * I-1, closed: @slot_ptr lives INSIDE the attach node, so
			 * the attach node is this op's lock-set member and a plain
			 * guard left it merely validated, never held -- the gap the
			 * lock-set completeness audit quantified (3 slot writes, 0
			 * acquires).  ACQUIRE it: under lock_fine that records the
			 * {LOCK|s -> s} release, and since f7cc59f9 a miss is
			 * all-or-none (abort + re-descend) rather than a silent
			 * degrade to the guard.  Non-lock_fine still lands on the
			 * guard, unchanged.
			 *
			 * ORDER IS LOAD-BEARING: the acquire's release must be
			 * recorded BEFORE @count_deferred's nr_child++ edge, since
			 * both target this one state word.  release-then-count
			 * chains to a single {LOCK|s -> s+1} -- release AND
			 * increment in one record; count-then-release would arrive
			 * with expected old LOCK|s against a pending s+1 and
			 * POISON the descriptor (the ordering rule spelled out at
			 * ft_flip_txn_record_release_lock).  Hence the count
			 * edge moved down here from just after the reserve.
			 *
			 * The RELOCATION arm needs nothing: the reserve's recompact
			 * already locked the grandparent it republishes into and
			 * recorded its release (see the !ft->lock_fine guard below).
			 */
			if (iter_dest_node_flag == attach_node_flag)
				ft_flip_txn_lock_or_guard_parent(ft, ic->txn,
					ctx, iter_dest_node_flag,
					FT_DEPTH_FROM_DESCENT);
			if (count_deferred)
				ft_flip_txn_record_nr_child_inc(ic->txn, metadata);
			/*
			 * @slot_ptr is a child slot inside
			 * @iter_dest_node_flag, whose @metadata is the node the
			 * two lines above lock-or-guard and count against.
			 */
			ft_flip_txn_record_reserved(ic->txn,
				/*owner=*/ metadata, (void **) slot_ptr,
				(void *) old_node_flag,
				(void *) iter_node_flag);
			ic->slot = slot_ptr;
			FT_TP(tree_edge_set, (const void *) ft,
				(const void *) iter_dest_node_flag,
				(unsigned int) (level - 1), (uint8_t) key_value,
				(const void *) iter_node_flag);
		}
		/*
		 * Phase 2: iter_node_flag's parent is now wired (by
		 * ft_node_set_nth above, either in-place or via recompact's
		 * reparent loop; by the explicit raw store for a one-commit
		 * insert).  Re-parent the LIVE displaced external head onto the
		 * fresh cluster top iter_node_flag.  external_nodes is
		 * reader-reachable through its old slot until the forward publish,
		 * so an up-walk would follow its prev / cell->parent INTO the
		 * not-yet-published cluster (and from there its build-invisible
		 * internals).  So this re-parent must flip ATOMICALLY with the
		 * forward publish below, not before it: park it into the
		 * one-commit (ic->live_child), which ft_insert_one_commit replays
		 * via ft_park_live_parent_edge (resolving the external head's
		 * cell->parent / prev).  The cluster then becomes reachable via
		 * BOTH its forward slot and this back-pointer in one flip.
		 */
		if (external_nodes) {
			/* ic->txn is always armed here (see the assert above). */
			ic->live_child =
				(struct cds_ft_inode_flag *) external_nodes;
			ic->live_parent = iter_node_flag;
			ic->live_slot = NULL;
		}
		/* Attach branch (unlink the old node from the trie).
		 * ft_publish_to_parent handles skip pointer update
		 * if the attach target is a compressed node's child.
		 */
		if (iter_dest_node_flag != attach_node_flag) {
			struct cds_ft_metadata *idest_meta =
				cds_ft_item_to_metadata(
					ft_node_ptr(iter_dest_node_flag));

			/*
			 * Phase 4.3 atomic re-home: the fresh copy inherited the attach
			 * node's (parent, offset) as a CONSISTENT snapshot (ft_node_
			 * recompact).  If a peer re-homed the attach node since this op's
			 * descent -- recompacted its grandparent -- the inherited slot no
			 * longer matches the descent-captured @attach_node_flag_ptr, and
			 * publishing @iter_dest at the stale slot would invert it against
			 * the fresh copy's NEW parent and fault ft_slot_to_byte (the
			 * dominant FT_INV_MW crash).  Re-descend rather than publish a
			 * torn (parent, slot) pair.  ft_get_parent_slot is consistent now
			 * (A.1 pair-resolver + A.2 atomic reparent), so this can no longer
			 * pass spuriously on a torn back-pointer.  No-op under exclusion.
			 * @iter_dest is a successfully-built fresh copy NOT tracked in
			 * created_nodes[], so free it here before the shared unwind.
			 */
			if (ft_get_parent_slot(idest_meta, ft) != attach_node_flag_ptr) {
				free_cds_ft_node_unpublished(ft,
					ft_node_ptr(iter_dest_node_flag));
				ret = -EAGAIN;
				goto check_error;
			}

			/*
			 * One-commit AND the reserve recompacted the attach node: the
			 * relocation swaps the OLD attach node for the fresh copy at its
			 * grandparent slot.  @rec already holds a compressed parent's
			 * SKIP_X dual (recorded by the reserve ft_node_set_nth_rec
			 * above, instead of a premature bare store); add the forward
			 * grandparent slot edge here so the relocation flips ATOMICALLY
			 * with the new key's slot edge in ic->txn -- "relocate + new
			 * key" is one publication.  ic->txn was armed before the build,
			 * so no allocation (hence no failure) here.  The old node stays
			 * resolved-to via the parked grandparent proxy until the commit,
			 * so defer its free past insert_done.
			 */
			/*
			 * VALIDATE (§4.B): guard the LIVE grandparent this relocation
			 * republishes into -- a concurrent remover that froze it aborts
			 * this commit (Phase 4.3 load-bearing; no-op under exclusion).
			 * Use the fresh copy's inherited (resolved) parent -- the same
			 * consistent snapshot the publish slot below is verified against.
			 *
			 * LOCK_FINE (§9.3): this grandparent is recompact's P -- already
			 * LOCKED by the reserve's ft_node_recompact, which recorded its
			 * {LOCK|s -> s} release on this very word.  The release record
			 * IS the guard (same word, same abort on a peer state change) and
			 * is strictly stronger, so the conversion REPLACES the guard here.
			 * Leaving it would not corrupt anything -- release-then-guard
			 * chains as a read-your-writes no-op (see the ordering rule at
			 * ft_flip_txn_record_release_lock) -- it would just be a vacuous
			 * record re-validating the value the release already pins.
			 */
			if (!ft->lock_fine)
				ft_flip_txn_guard_parent(ft, ic->txn,
					ft_parent_node(idest_meta->parent_word));
#ifdef FEATURE_FT_PROBE_EMPTY_INSERT
			__atomic_fetch_add(&cds_ft_probe_reach_attach, 1,
				__ATOMIC_RELAXED);
			if (ft_probe_internal_is_empty(ft, iter_dest_node_flag)) {
				__atomic_fetch_add(
					&cds_ft_probe_empty_publish_attach, 1,
					__ATOMIC_RELAXED);
				ret = -EAGAIN;
				goto check_error;
			}
#endif
			_ft_publish_to_parent(ft, attach_node_flag,
				attach_node_flag_ptr, iter_dest_node_flag,
				attach_node_flag, &rec);
			ft_flip_txn_record_pub_rec(ic->txn, &rec);
			ic->free_old_node = old_recompacted_node;
			old_recompacted_node = NULL;
			/*
			 * The relocated fresh copy is unpublished until the commit
			 * flips the grandparent slot: track it with the cluster from
			 * here on (the earlier -EAGAIN guard freed it separately
			 * because it was NOT yet tracked), so the success handoff
			 * below covers it for the commit-ABORT rollback.
			 */
			created_nodes[nr_created_nodes++] = iter_dest_node_flag;
			/*
			 * I7 count fold (rank stats ON): the reserve recompacted
			 * the attach node, so the relocated copy iter_dest_node_flag
			 * (a fresh node published at the grandparent slot) carries
			 * the new key -- the fresh branch is wired into its reserved
			 * slot -- but the recompact copied only the OLD subtree count
			 * (ft_node_recompact, mutation-node.h).  Bump it +1 to its
			 * FULL post-commit count (a build-invisible plain store on the
			 * fresh copy, published atomically at the commit) so it is off
			 * the count walk, then record the +1 walk from
			 * metadata->parent -- the STABLE grandparent whose slot this
			 * relocation republishes into (VALIDATE-guarded live just
			 * above), commit-invariant -- up to the root.  The retired old
			 * attach node is NOT on the walk.  Single-level relocation:
			 * ft_node_set_nth_rec rebuilds only the attach node; the
			 * grandparent is not relocated.  Equivalent to the old
			 * post-commit walk from *d.pnfp, which post-commit is exactly
			 * this relocated copy (+1) then its stable ancestors.
			 *
			 * Reader-safety of this pre-commit +1: the fresh copy is
			 * reachable pre-commit ONLY as an ANCESTOR (recompact reparents
			 * its children's parent pointers to it), never through its own
			 * forward slot (the grandparent slot still resolves to the old
			 * node until the flip), and its new-key slot reads NULL until
			 * the commit -- so its own aggregate transiently overcounts what
			 * is reachable from it.  This is invisible because no count
			 * reader ever reads a node's OWN nr_keys except at a
			 * root-DESCENDED position (which lands on the old node via the
			 * proxy); the skip up-walk reads a node's CHILDREN's counts, not
			 * the node's own.  Do not add a reader that reads an up-walked
			 * ancestor's own nr_keys without revisiting this fold.
			 */
			{
				struct cds_ft_metadata *reloc_meta =
					cds_ft_item_to_metadata(
						ft_node_ptr(iter_dest_node_flag));

				ft_nr_keys_store(ft, reloc_meta,
					ft_nr_keys_get(reloc_meta) + 1,
					CMM_RELAXED);
			}
			ic->count_from = ft_parent_node(metadata->parent_word);
			ic->count_folded = true;
		} else {
			/*
			 * In-place reserve (dest == attach node): republish the
			 * attach node at its OWN grandparent slot.  The store is a
			 * same-value forward store whose real work is
			 * ft_set_parent_slot's parent_slot_offset / incoming_byte
			 * maintenance (and, for a compressed grandparent, the SKIP
			 * dual).  Under MW it must NOT be a bare direct store: if a
			 * peer re-homed the attach node since this op's descent
			 * (recompacted its grandparent), @attach_node_flag_ptr is a
			 * STALE slot in the retired grandparent, and a direct
			 * ft_publish_to_parent would (a) drive ft_set_parent_slot's
			 * offset / ft_slot_to_byte against the fresh grandparent from
			 * the stale slot -- garbage parent_slot_offset, the dominant
			 * FT_INV_MW fault -- and (b) fire its `*slot != new_child`
			 * store, resurrecting the retired attach node over the peer's
			 * copy, un-rolled-back by a commit ABORT.  Mirror the
			 * relocation branch above: re-descend if the attach node
			 * moved, guard the live grandparent, and record the
			 * (same-value) reader-visible edges into ic->txn so a peer
			 * freeze/relocate ABORTS this commit.  ft_set_parent_slot
			 * still runs immediately (it is not gated by @rec),
			 * preserving the in-place metadata bookkeeping the direct
			 * store performed (why dropping the call outright is wrong).
			 */
			struct cds_ft_metadata *attach_meta =
				cds_ft_item_to_metadata(
					ft_node_ptr(attach_node_flag));

			if (ft_get_parent_slot(attach_meta, ft) !=
					attach_node_flag_ptr) {
				ret = -EAGAIN;
				goto check_error;
			}
			/*
			 * The edges below write @attach_node_flag_ptr, a slot in
			 * the GRANDPARENT, so the grandparent is a lock-set member
			 * -- ACQUIRE it rather than merely guarding it (I-1's
			 * second half).  It is a value-swap target: the op does not
			 * change its nr_child and never copies its body under the
			 * lock, so the release terminal is clean and no count edge
			 * shares the word.  A miss aborts and re-descends
			 * (f7cc59f9), which is what the ft_get_parent_slot
			 * mismatch just above already does for the stale-slot case.
			 */
			ft_flip_txn_lock_or_guard_parent(ft, ic->txn, ctx,
				ft_parent_node(attach_meta->parent_word),
				FT_DEPTH_FROM_DESCENT);
#ifdef FEATURE_FT_PROBE_EMPTY_INSERT
			__atomic_fetch_add(&cds_ft_probe_reach_attach, 1,
				__ATOMIC_RELAXED);
			if (ft_probe_internal_is_empty(ft, iter_dest_node_flag)) {
				__atomic_fetch_add(
					&cds_ft_probe_empty_publish_attach, 1,
					__ATOMIC_RELAXED);
				ret = -EAGAIN;
				goto check_error;
			}
#endif
			_ft_publish_to_parent(ft, attach_node_flag,
				attach_node_flag_ptr, iter_dest_node_flag,
				attach_node_flag, &rec);
			ft_flip_txn_record_pub_rec(ic->txn, &rec);
			/*
			 * I6 count fold (rank stats ON): the attach node stays in
			 * place, so its metadata->parent chain up to the root is
			 * commit-invariant.  Record the +1 key-count walk from it
			 * (base = attach_node_flag, the stable node that owns the
			 * new key's reserved forward slot) into THIS txn, so the
			 * count flips ATOMICALLY with the structural publish, and
			 * insert_done skips the standalone post-commit root-ward
			 * count walk.  The fresh branch
			 * below carries its +1 from build (build-invisible plain
			 * stores), so it is never touched by the recorded walk.
			 * Behaviour-identical to the old post-commit walk from
			 * *d.pnfp (which, in place, is exactly attach_node_flag)
			 * under the retained writer exclusion.  The relocation
			 * branch above leaves count_folded false: there the base is
			 * the fresh relocated copy, correct only when read
			 * post-commit through *d.pnfp, so that shape (I7) stays a
			 * post-commit walk for now.
			 */
			ic->count_from = attach_node_flag;
			ic->count_folded = true;
		}

		/* Reclaim safely after unlink (deferred to the commit when the
		 * relocation was folded into ic->txn above). */
		if (old_recompacted_node)
			free_cds_ft_node(ft, old_recompacted_node);
	}

	/* Success */
	ret = 0;
	/*
	 * Hand the fresh cluster to the op scope: it stays unpublished until
	 * the one-commit flips the parked slots, so a commit ABORT's rollback
	 * (ft_insert_abort_cb) must free it -- without this handoff, every
	 * commit-time ABORT of the attach shapes leaked the whole fresh branch
	 * (only the split builders handed their clusters over).
	 */
	if (ic->txn) {
		memcpy(ic->created, created_nodes,
			(size_t) nr_created_nodes * sizeof(created_nodes[0]));
		ic->nr_created = nr_created_nodes;
	}

check_error:
	if (ret) {
		/*
		 * All goto-check_error paths in this function are before
		 * ft_publish_to_parent, so created_nodes[] never escaped
		 * the writer's stack -- immediate-free is safe.  The armed
		 * one-commit txn MAY already carry recorded edges (a reserve
		 * recompact records the external-head back-channel, reparent
		 * pairs and the retire tombstone before a later step bails
		 * -EAGAIN): records are PREPARE-state only -- nothing is
		 * installed in a live slot during the build -- and the
		 * UNCONDITIONAL destroy below discards them coherently.  Do
		 * not "optimize" the destroy behind a records-empty check.
		 */
		if (ic->txn) {
			ft_flip_txn_destroy(ic->txn);
			ic->txn = NULL;
		}
		for (i = 0; i < nr_created_nodes; i++) {
			if (ft_node_compressed(created_nodes[i]))
				free_compressed_node_unpublished(ft,
					ft_compressed_node_ptr(created_nodes[i]));
			else
				free_cds_ft_node_unpublished(ft, ft_node_ptr(created_nodes[i]));
		}
	}
	FT_TP(attach_node_exit, (int) ret);
	return ret;
}

static
int ft_chain_node(struct cds_ft *ft, struct cds_ft_node *last_node,
		struct cds_ft_node *node)
{
	struct ft_flip_txn *t;
	enum urcu_txn_status st;

	FT_TP(chain_node, (const void *) last_node, (const void *) node);
	/*
	 * Add node to the tail of the duplicate chain to ensure that RCU
	 * traversals always see either the prior node or the newly added one
	 * under a concurrent add-then-del on the same key.  Safe against
	 * concurrent RCU read traversals.
	 *
	 * The tail append is the sole reader-visible publish of a duplicate
	 * insert: record it as an hlist insert-after edge (last_node->next:
	 * NULL -> node, a recorded CAS a concurrent freeze of the tail would
	 * abort; @node is built invisibly, next = NULL, prev = last_node), folded
	 * into a single-edge flip-txn instead of a bare store.  Both next and
	 * prev ride the txn; the commit is the engine's single-edge fast path.
	 * Fallible: the txn allocation can OOM (the caller unwinds @node).
	 *
	 * CONCURRENT WRITERS (Phase 4.3): a peer freezing/removing @last_node
	 * conflicts here -- at prepare time (-ENOENT tail already marked /
	 * -EAGAIN neighbour mid-deletion; nothing recorded installs without a
	 * commit) or at commit time (ABORT: the expected-value CAS on
	 * last_node->next lost; nothing installed).  Both map to -EAGAIN: the
	 * callers route to insert_done, whose -EAGAIN handler ages the op's
	 * persistent handle and re-descends from root (restart_attempt resets
	 * @node's prev/next).  Previously ABORT was folded into 0 -- a silently
	 * LOST duplicate under contention.
	 */
	t = ft_flip_txn_create_bounded(ft, FT_HLIST_INSERT_AFTER_MAX_EDGES);
	if (!t)
		return -ENOMEM;
	if (ft_hlist_insert_after_prepare(ft_flip_txn_handle(t), node,
			last_node)) {
		ft_flip_txn_destroy(t);
		return -EAGAIN;
	}
	st = ft_flip_txn_commit(ft, t);
	if (st < 0)
		return -ENOMEM;
	return st > 0 ? -EAGAIN : 0;
}

/*
 * There are a few cases to cover for add:
 *
 * 1) There is already an external node at that key. Chain this new node
 *    with the existing node (duplicate).
 * 2) There is already an internal node with associated external node at
 *    that key. Chain this new node with the existing node (duplicate).
 * 3) The traversal ends before reaching the end of the lookup key:
 *    3.1) The last node encountered during traversal is an internal
 *         node. Attach a new cluster as child of this internal node.
 *    3.2) The last node encountered during traversal is an external
 *         node. Need to transform this external node into an internal
 *         node with associated external node, attach a new cluster as
 *         child of this internal node, and populate this new internal
 *         node into the trie to replace the prior external node.
 */

/*
 * ft_insert_compressed_past_child: key continues past a compressed
 * node's external child.  Build a branch below the child and propagate
 * density / external count through the snapshot.
 *
 * Returns 0 on success, -ENOMEM on allocation failure.
 */
static
int ft_insert_compressed_past_child(struct cds_ft *ft,
		struct ft_descent *d,
		const uint8_t *key, size_t key_len,
		struct cds_ft_compressed_node *cn,
		struct cds_ft_node *node,
		struct ft_insert_commit *ic)
{
	struct cds_ft_inode_flag *branch;
	struct cds_ft_metadata *br_meta;
	int ret;

	/*
	 * Reserve the I5 count-fold edges: the +1 key-count walk climbs from
	 * the STABLE compressed node @d->nf (whose child slot this commit flips
	 * to the fresh branch) up to the root -- at most @d->depth + 2 ancestor
	 * edges (actual descent depth, never FT_MAX_DEPTH).  No-op reserve (0)
	 * when order statistics are off.
	 */
	ret = ft_insert_commit_arm(ft, ic,
		ft->rank_stats ? d->depth + 2 : 0);
	if (ret)
		return ret;	/* nothing built yet */

	/*
	 * ONE resolved snapshot of the live external child, consumed by both
	 * the branch's external_nodes wiring and the parked live re-parent
	 * below.  The dispatcher classified cn->child external, but a peer may
	 * park a flip-proxy on it (re-parenting that external head) before these
	 * reads: a RAW read would publish the proxy as the branch's external
	 * head / @ic->live_child and ft_park_live_parent_edge would dereference
	 * the latch memory at commit.  Resolve once, like the sibling split
	 * builders (ft_split_compressed_diverge / _key_shorter); a post-snapshot
	 * peer commit is caught by the §4.B guard on @d->nf that
	 * ft_insert_publish_or_park records (ABORT -> re-descend).
	 */
	struct cds_ft_inode_flag *old_child_flag =
		ft_cn_child_dereference_acquire_prefetch(cn);

	/*
	 * Case 1 (external at END of compressed path): build a
	 * branch for the continuing key, with an internal node at
	 * d->depth + cn->len that holds the old external child as
	 * external_nodes and dispatches the next key byte.
	 */
	{
		unsigned int br_start = d->depth + cn->len;
		struct cds_ft_inode_flag *inner;
		struct cds_ft_inode_flag *dest = NULL;

		inner = ft_build_branch(ft, key,
			br_start + 1, key_len,
			(struct cds_ft_inode_flag *) node, 1, false, NULL);
		if (!inner) {
			ret = -ENOMEM;
			goto arm_unwind;
		}
		ret = ft_node_set_nth(ft, &dest, key[br_start],
			inner, NULL, NULL, br_start, false);
		if (ret) {
			/*
			 * Free the built branch (it was leaked before): the
			 * cluster is writer-private, nothing was published.
			 * insert_done resets node->prev for the retry.
			 */
			ft_free_branch_unpublished(ft, inner,
				(struct cds_ft_inode_flag *) node);
			ret = -ENOMEM;
			goto arm_unwind;
		}
		branch = dest;
		br_meta = cds_ft_item_to_metadata(ft_node_ptr(branch));
		/*
		 * Phase 1 (build-invisible): wire branch's own parent and its
		 * cluster-internal external_nodes pointer.  The back-channel
		 * publish (cn->child->prev = branch) is deferred to Phase 2
		 * below -- otherwise an up-walk from cn->child (still reachable
		 * through the unmodified cn) lands on branch with parent NULL.
		 */
		ft_set_parent(ft, branch, d->nf, &cn->child);
		ft_metadata_set_external_nodes(branch, br_meta,
			(struct cds_ft_node *) old_child_flag);
		/*
		 * I5 count fold: build the branch with its FULL post-commit
		 * count -- the pre-existing key (old external from the
		 * compressed child) PLUS the new key carried by @inner below it
		 * (built full by ft_build_branch) -- so this fresh node is off
		 * the count walk.  The +1 for the new key then rides the commit
		 * as edges on the STABLE ancestors from @d->nf up (recorded
		 * below), not a post-commit root-ward count walk.
		 */
		ft_nr_keys_store(ft,br_meta, 2, CMM_RELAXED);
	}
	/*
	 * Phase 2: park the LIVE displaced-external-head re-parent (the old
	 * external cn->child's cell->parent / prev -> branch) into the
	 * one-commit so it flips ATOMICALLY with the forward publish below: a
	 * reader never sees cn->child re-parented onto branch while the
	 * grandparent slot still points at cn (or vice versa).  ft_insert_one_
	 * commit replays it via ft_park_live_parent_edge (which resolves the
	 * external head's cell->parent / prev).
	 */
	ic->live_child = old_child_flag;
	ic->live_parent = branch;
	ic->live_slot = NULL;
	/* &cn->child's plan-snapshot old is the displaced child == ic->live_child. */
	{
		struct ft_lock_ctx pctx;

		ft_lock_ctx_init(&pctx, d, ic ? ic->txn : NULL, ic ? ic->op : NULL);
		ft_insert_publish_or_park(ft, &pctx, d->nf, d->depth,
			&cn->child, branch, ic->live_child, ic);
	}
	/*
	 * I5 count fold: @d->nf is the STABLE compressed node whose child slot
	 * (&cn->child) this commit flips to the fresh branch; the branch was
	 * built with its full post-commit count above, so the +1 walk starts at
	 * d->nf (its subtree grows from the old external's 1 key to the branch's
	 * 2) and touches only commit-invariant ancestors.  Record it into the
	 * commit txn (reserved at arm) and skip the post-commit propagate.
	 */
	ic->count_from = d->nf;
	ic->count_folded = true;
	return 0;
arm_unwind:
	if (ic->txn) {
		ft_flip_txn_destroy(ic->txn);
		ic->txn = NULL;
	}
	return ret;
}

/*
 * ft_insert_compressed_diverge: key diverges from the compressed path
 * at position @j.  Split the compressed node and insert the new key.
 *
 * Returns 0 on success, negative errno on failure.
 */
static
int ft_insert_compressed_diverge(struct cds_ft *ft,
		struct ft_descent *d,
		const uint8_t *iter_key,
		unsigned int remaining, unsigned int j,
		struct cds_ft_node *node,
		struct ft_insert_commit *ic)
{
	int dret;

	dret = ft_split_compressed_insert(ft, d,
		d->nfp, d->nf, iter_key, remaining,
		j, node, d->depth, ic);
	if (dret)
		return dret;
	/*
	 * ft_split_compressed_insert wires top_flag's back-pointer into the
	 * live parent before publishing the cluster's forward slot, so the
	 * caller does not need to set the parent here.
	 *
	 * I3 count fold: @d->pnf is the STABLE parent whose child slot (d->nfp)
	 * this commit flips to the fresh split cluster; every fresh split node was
	 * built with its full post-commit count, so the +1 count walk starts at
	 * d->pnf and touches only commit-invariant ancestors.  Record it into the
	 * commit txn (reserved at arm) and skip the post-commit propagate.
	 */
	ic->count_from = d->pnf;
	ic->count_folded = true;
	return 0;
}

/*
 * ft_insert_compressed_key_shorter: key ends before the compressed
 * path.  Split the compressed node into prefix -> junction -> suffix,
 * then attach the new external node at the junction.
 *
 * Returns 0 on success, -EEXIST if duplicate detected (with
 * *unique_node_ret set), or negative errno on failure.
 */
static
int ft_insert_compressed_key_shorter(struct cds_ft *ft,
		struct ft_descent *d,
		unsigned int remaining,
		struct cds_ft_node *node,
		struct cds_ft_node **unique_node_ret,
		struct ft_insert_commit *ic)
{
	struct cds_ft_inode_flag *top_flag, *jct_flag;
	struct cds_ft_metadata *jct_meta;
	struct cds_ft_inode_flag *live_child, *live_parent;
	struct cds_ft_inode_flag **live_slot;
	struct cds_ft_inode_flag *split_created[FT_MAX_DEPTH];
	int split_nr_created = 0;
	struct cds_ft_metadata *cn_meta = cds_ft_item_to_metadata(
		(struct cds_ft_inode *) ft_compressed_node_ptr(d->nf));
	struct ft_held_anchor held;
	int sret;
	struct cds_ft_inode_flag *fwd_expected_old;	/* set post-fence */

	/*
	 * A key-shorter insert is always a NEW key: a key ending inside a
	 * compressed path has no node at that depth that could already hold it
	 * (a real duplicate is caught on descent, before this split), so the
	 * freshly-built junction always takes a fresh head -- there is no
	 * duplicate case here.
	 */
	(void) unique_node_ret;

	/*
	 * F2 node lock, split-retire extension (see
	 * ft_split_compressed_insert): the key-shorter split retires @d->nf and
	 * plans from its body + child, so fence it before the builder's first
	 * read.  ft_insert_one_commit records the retire from @held.
	 */
	/*
	 * DLM Step 1: under LOCK_FINE, acquire the whole split lock-set {CN, P} in
	 * one all-or-none MCAS up front (P = CN's parent @d->pnf, the value-swap
	 * publish target below), mirroring the diverge builder.  Non-DLM /
	 * non-lock_fine keeps the single CN lock-acquire (byte-identical).
	 */
	if (ft->lock_fine) {
		sret = ft_insert_dlm_acquire_split(ft, d, d->nf, d->depth,
				cn_meta, &held, ic);
	} else {
		struct ft_lock_ctx lctx;

		ft_lock_ctx_init(&lctx, d, ic ? ic->txn : NULL, ic ? ic->op : NULL);
		sret = ft_acquire_member(ft, &lctx, d->nf, cn_meta, d->depth,
			&held);
	}
	if (sret)
		return sret;	/* -EAGAIN: peer owns the cn/P; nothing built */

	/*
	 * Plan-snapshot the raw parent slot NOW (post-fence, pre-build): the
	 * skip-PRESERVED, flip-proxy-resolved reference to @d->nf the forward
	 * publish CAS must match.  @d->nf is skip-RESOLVED, so under
	 * skip-compression it differs from the stored SKIP_X (livelock if
	 * passed); capturing pre-build also avoids ratifying a peer that raced
	 * the slot during the split builder.
	 */
	fwd_expected_old = ft_resolve_flip_proxy(*d->nfp);

	sret = ft_split_compressed_key_shorter(ft,
		d->nf, d->nfp, remaining, &top_flag, &jct_flag, d->depth,
		&live_child, &live_parent, &live_slot,
		split_created, &split_nr_created);
	if (sret) {
		ft_meta_lock_release(held.lock);
		ft_insert_dlm_release_parent(ic);	/* release P held up front */
		return sret;
	}
	jct_meta = cds_ft_item_to_metadata(ft_node_ptr(jct_flag));
	assert(!ft_node_compressed(jct_flag));
	assert(jct_meta->external_nodes == NULL);	/* always a fresh head */
	/*
	 * Attach the fresh head to the junction NOW, while the whole cluster is
	 * still invisible, so the parked one-commit publish below makes the
	 * structural attach and the ordered-list splice atomic.
	 */
	ft_external_head_set_parent(ft, node, jct_flag);
	node->next = NULL;
	/* Cluster-internal store: the junction is unpublished. */
	jct_meta->external_nodes = node;
	/*
	 * Reserve the I4 count-fold edges: the +1 key-count walk climbs from
	 * the STABLE parent d->pnf (whose child slot d->nfp this commit flips to
	 * the fresh split cluster) up to the root -- at most d->depth + 2
	 * ancestor edges (actual descent depth, never FT_MAX_DEPTH).  No-op
	 * reserve (0) when order statistics are off.
	 */
	sret = ft_insert_commit_arm(ft, ic,
		ft->rank_stats ? d->depth + 2 : 0);
	if (sret) {
		/*
		 * Arm failed (-ENOMEM): abort the whole insert.  The split
		 * cluster is still build-invisible (nothing published, cn->child
		 * untouched, @d->nf still reader-reachable at @d->nfp), so roll
		 * the head attach back and TEAR THE CLUSTER DOWN -- the structure
		 * is left byte-for-byte unchanged.
		 *
		 * Do NOT publish a key-neutral restructure here: the forward
		 * publish and the live old-child re-parent must flip ATOMICALLY
		 * (the re-parent's source is the live cn->child, so wiring it
		 * before the forward exposes the unpublished cluster to an
		 * up-walk), which needs the one-commit txn we just failed to
		 * allocate.  There is no MCAS-expressible publish on this path --
		 * a bare forward store would sit outside the descriptor set --
		 * only a clean abort.  The caller surfaces MEMORY_ERROR;
		 * insert_done resets node->prev for a retry.
		 */
		jct_meta->external_nodes = NULL;
		node->next = NULL;
		ft_free_unpublished_split_cluster(ft, split_created,
			split_nr_created);
		ft_meta_lock_release(held.lock);
		ft_insert_dlm_release_parent(ic);	/* release P held up front */
		return sret;
	}
	/*
	 * The armed txn now owns the fence outcome (registered clear on every
	 * non-commit terminal; consumed by the fenced tombstone on commit).
	 */
	ft_flip_txn_lock_register(ic->txn, held.lock, held.lock_snap);
	ft_flip_txn_record_anchor_release(ic->txn, &held, cn_meta);
	ic->free_old_cn_held = held;
	/*
	 * Concurrent-writer conflict check (mirror of the diverge builder's
	 * pre-arm guard): @d->nfp was recorded at descent as the cn's slot in
	 * its parent; a peer may since have re-homed the cn, so consuming the
	 * stale (parent, slot) pair below would invert it against the wrong
	 * node's body and fault ft_slot_to_byte mid-build.  The fence does not
	 * cover this (a re-home committed BEFORE the mark leaves the cn clean).
	 * Re-derive from the cn's own resolved (parent, offset) pair; disagree
	 * -> unwind exactly as the arm-failure above (nothing published) and
	 * re-descend.
	 */
	if (ft_get_parent_slot(cn_meta, ft) != d->nfp) {
		jct_meta->external_nodes = NULL;
		node->next = NULL;
		ft_free_unpublished_split_cluster(ft, split_created,
			split_nr_created);
		/* Registered above: the txn owns CN's fence; discard both. */
		ft_flip_txn_destroy(ic->txn);
		ic->txn = NULL;
		/* P was held up front, not yet registered (publish not reached). */
		ft_insert_dlm_release_parent(ic);
		return -EAGAIN;
	}
	/*
	 * Hand the fresh cluster to the op scope: the commit's on-abort
	 * rollback (ft_insert_abort_cb) frees it if a peer writer wins at
	 * commit time -- without this, every commit ABORT of the key-shorter
	 * shape leaked the unpublished split cluster.
	 */
	memcpy(ic->created, split_created,
		(size_t) split_nr_created * sizeof(split_created[0]));
	ic->nr_created = split_nr_created;
	/*
	 * top_flag's own back-pointer was wired in the split (it is a fresh
	 * cluster node).  The LIVE old-child re-parent is the back-channel that
	 * exposes the cluster to a reanchor up-walk, so defer it to the parked
	 * one-commit: it flips atomically with the forward publish and the cell
	 * (fresh head + ordered list).
	 */
	ic->live_child = live_child;
	ic->live_parent = live_parent;
	ic->live_slot = live_slot;
	{
		struct ft_lock_ctx pctx;

		ft_lock_ctx_init(&pctx, d, ic ? ic->txn : NULL, ic ? ic->op : NULL);
		ft_insert_publish_or_park(ft, &pctx, d->pnf, d->pdepth,
			d->nfp, top_flag, fwd_expected_old, ic);
	}
	/*
	 * I4 count fold: @d->pnf is the STABLE parent whose child slot (d->nfp)
	 * this commit flips from the old compressed node to the fresh split
	 * cluster; every fresh split node (junction + prefix) was built above
	 * with its full post-commit count, so the +1 walk starts at d->pnf and
	 * touches only commit-invariant ancestors.  Record it into the commit
	 * txn (reserved at arm) and skip the post-commit propagate.  The old
	 * compressed node's free follows the commit at insert_done (a reader
	 * resolves the parked proxy to it until then).
	 */
	ic->count_from = d->pnf;
	ic->count_folded = true;
	ic->free_old_cn = ft_compressed_node_ptr(d->nf);
	return 0;
}

/*
 * Handle a compressed node during insert descent.
 *
 * Full match + internal/compressed child: traverse through.
 * Full match + external child at end of key: break for duplicate handling.
 * Full match + external child, key continues: build branch inline.
 * Key diverges: split via ft_split_compressed_insert.
 * Key shorter: split via ft_split_compressed_key_shorter.
 *
 * Returns CONTINUE, BREAK, or END (with ret set via *ret_p).
 * On END, the caller should goto insert_done.
 * On error, returns END with *ret_p < 0.
 */
static
enum ft_descent_action ft_insert_compressed(struct cds_ft *ft,
		struct ft_descent *d, const uint8_t **iter_key_p,
		const uint8_t *key, size_t key_len,
		unsigned int key_depth,
		struct cds_ft_node *node,
		struct cds_ft_node **unique_node_ret,
		struct cds_ft_inode_flag **snapshot,
		unsigned int *snapshot_depth,
		int *nr_snapshot_p,
		int *ret_p,
		struct ft_insert_commit *ic)
{
	struct cds_ft_compressed_node *cn = ft_compressed_node_ptr(d->nf);
	unsigned int remaining = key_depth - 1 - d->depth;
	unsigned int cmp;
	unsigned int j;

	/* Flight-recorder mis-wire detector (no-op without FT_ENABLE_TRACING). */
	FT_TRACE_MISWIRE(ft, d->nf, 1);
	cmp = cn->len < remaining ? cn->len : remaining;
	j = ft_match_compressed_key(*iter_key_p, cn, cmp);
	if (j == cmp && cn->len <= remaining) {
		/* Full match: traverse through if child is internal,
		 * compressed, or skip-compressed (which encodes another
		 * compressed node deeper in the chain). */
		if (cn->child &&
		    (ft_node_skip_compressed(cn->child) ||
		     ft_node_internal(cn->child) ||
		     ft_node_compressed(cn->child))) {
			ft_snapshot_push(snapshot, snapshot_depth,
				*nr_snapshot_p, d->nf, d->depth);
			ft_descent_traverse_compressed(ft, d, cn, iter_key_p);
			return FT_DESCENT_CONTINUE;
		}
		if (!cn->child) {
			struct cds_ft_metadata *cn_meta =
				cds_ft_item_to_metadata((struct cds_ft_inode *) cn);
			fprintf(stderr, "BUG: cn->child NULL, cn=%p cn->len=%u depth=%u external_nodes=%p nr_child=%u\n",
				cn, cn->len, d->depth, cn_meta->external_nodes, (unsigned)ft_meta_nr_child(cn_meta));
			abort();
		}
		if (cn->len == remaining) {
			/* Key ends at external child: duplicate. */
			ft_snapshot_push(snapshot, snapshot_depth,
				*nr_snapshot_p, d->nf, d->depth);
			ft_descent_traverse_compressed(ft, d, cn, iter_key_p);
			return FT_DESCENT_BREAK;
		}
		/* Key continues past external child: build branch. */
		*ret_p = ft_insert_compressed_past_child(ft, d, key,
			key_len, cn, node, ic);
		return FT_DESCENT_END;
	}
	if (j < cmp) {
		/* Key diverges: split at position j. */
		*ret_p = ft_insert_compressed_diverge(ft, d,
			*iter_key_p, remaining, j, node, ic);
		return FT_DESCENT_END;
	}
	/* Key shorter: split into prefix -> junction -> suffix. */
	*ret_p = ft_insert_compressed_key_shorter(ft, d, remaining,
		node, unique_node_ret, ic);
	return FT_DESCENT_END;
}

static
int _cds_ft_insert(struct cds_ft *ft,
		const uint8_t *_key, size_t _key_len,
		struct cds_ft_node *node,
		struct cds_ft_node **unique_node_ret)
{
	unsigned int key_depth;
	struct ft_descent d;
	size_t key_len = ft_key_len(ft, _key_len);
	const struct cds_ft_key_map *km = &ft->group->key_map;
	uint8_t ordinal_buf[FT_MAX_KEY_LEN];
	const uint8_t *key;
	const uint8_t *iter_key;
	struct cds_ft_inode_flag *snapshot[FT_MAX_DEPTH];
	unsigned int snapshot_depth[FT_MAX_DEPTH]; /* parallel depth tracking */
	int nr_snapshot = 0;
	int ret;
	struct ft_ord_cell *precell;
	void *cell = NULL;			/* @precell's carrier; reused across retries */
	enum urcu_txn_status cst = URCU_TXN_STATUS_OK;	/* last commit outcome */
	struct ft_insert_commit ic = { 0 };
	/*
	 * The attach's recompactions lock {C, P, GP}; @d dates them and @ic.txn
	 * names what this op already holds.  Re-initialised at each attach,
	 * because the txn is armed part-way through.
	 */
	struct ft_lock_ctx actx;
	struct urcu_txn optxn;		/* persistent handle spanning the retry loop */

	if (!valid_external_node(node) || !valid_key_len(ft, key_len))
		return -EINVAL;
	if (caa_likely(km->identity)) {
		key = _key;
	} else {
		ft_key_to_ordinals(ordinal_buf, _key, key_len, km);
		key = ordinal_buf;
	}
	iter_key = key;
	/* Expect zeroed prev/next pointers. This catches some double-insert misuses. */
	if (node->prev || ft_node_next(node))
		return -EINVAL;

	/*
	 * Ordered-list trie: pre-wire @node's ordinal cell before any structural
	 * mutation, so the only failure-prone allocation happens up front (a
	 * clean -ENOMEM, nothing to roll back) and every fresh-head wiring site
	 * downstream just records the flagged parent into the cell (node->prev
	 * already carries it).  If @node ends up a duplicate (chained, not a
	 * head) or the insert fails, the unused @precell is freed at insert_done
	 * (a chained @node has its prev repointed at the predecessor, losing the
	 * cell from node->prev, so the handle is kept here).
	 *
	 * List off: no cell -- @node behaves like a non-cell build (its prev is
	 * wired to the flagged parent directly by the fresh-head sites), saving
	 * the per-key cell.  @precell stays NULL and the cell paths below no-op.
	 */
	precell = NULL;
	if (ft->ordered_list) {
		cell = ft_ord_cell_alloc(ft, node, NULL);

		if (!cell)
			return -ENOMEM;
		precell = ft_ord_cell_ptr(cell);
	}

	key_depth = key_len + 1;

	/*
	 * The op's persistent engine handle (doc §11): initialized ONCE, so
	 * contention aging (txn->retry), the FIFO fair-mutex escalation turn,
	 * and the learned descriptor size span every attempt of the loop below.
	 * Each attempt is bracketed by urcu_txn_begin()/urcu_txn_end(): the FT
	 * owns its read-side section -- on a concurrent trie the descent must
	 * run inside one so a peer writer's call_rcu-deferred frees cannot
	 * reclaim nodes under it; a caller-held section merely nests.  On an
	 * exclusive trie the bracket opens nothing (NULL flavor) and the domain
	 * is NULL (never escalates) -- behavior-identical to the pre-bracket
	 * path.
	 */
	ft_txn_op_init(ft, &optxn);

restart_attempt:
	urcu_txn_begin(&optxn);
	/*
	 * Per-attempt setup, re-entered on a concurrent-writer conflict (a
	 * pre-commit -EAGAIN or a commit ABORT): the failed attempt published
	 * nothing and its fresh cluster was already rolled back, so re-arm the
	 * caller's node linkage, clear the commit scope, and re-descend from the
	 * root against the now-current tree.  @precell is allocated once (above)
	 * and REUSED across attempts -- it is spliced only by a committing attempt.
	 *
	 * Head's last edge byte for the up-walk key rebuild: the cell is the head's
	 * metadata record and @key is ordinal here, so key[key_len - 1] is the byte
	 * the head hangs under (ignored by the up-walk when the head's parent is a
	 * compressed node, whose key_bytes already span the head's position).
	 */
	ic = (struct ft_insert_commit){ 0 };
	ic.ft = ft;
	ic.op = &optxn;
	cst = URCU_TXN_STATUS_OK;
	node->prev = cell;			/* NULL when the list is off */
	node->next = NULL;
	if (precell && key_len)
		cds_ft_item_to_metadata(precell)->incoming_byte =
			(uint8_t) key[key_len - 1];
	iter_key = key;
	nr_snapshot = 0;

	dbg_printf("cds_ft_insert attempt: node %p\n", node);
	ft_descent_init(&d, ft);

	for (; d.depth < key_depth - 1; ) {
		uint8_t key_value;

		/*
		 * The last reanchoring step landed shallower (rewind > 0: a peer
		 * chain-merge moved the encoded position up), so the captured
		 * publish slot d.nfp is at the wrong level -- re-descend against
		 * the now-current tree.  Shared read/write skip concurrency: the
		 * step detects the shift, the mutator reacts (a reader rewinds).
		 */
		if (caa_unlikely(d.skip_conflict)) {
			ret = -EAGAIN;
			goto insert_done;
		}
		if (!d.nf)
			break;
		/*
		 * Resolve skip-compressed pointer.  Convert to the
		 * underlying compressed flag so the compressed handler
		 * below processes it correctly.
		 */
		d.nf = ft_resolve_skip_compressed(ft, d.nf);
		/* Found external node. */
		if (ft_node_external(d.nf))
			break;
		/* Decompress compressed node before continuing descent. */
		/*
		 * Compressed node: compare remaining key bytes with
		 * the compressed path.  If they match, traverse
		 * through to the child.  If they diverge, decompress
		 * at this point and restart.
		 */
		if (ft_node_compressed(d.nf)) {
			enum ft_descent_action act;

			act = ft_insert_compressed(ft, &d, &iter_key,
				key, key_len, key_depth, node,
				unique_node_ret, snapshot, snapshot_depth,
				&nr_snapshot, &ret, &ic);
			if (act == FT_DESCENT_END)
				goto insert_done;
			if (act == FT_DESCENT_BREAK)
				break;
			continue;
		}
		dbg_printf("cds_ft_insert iter ppnf %p pnf %p nfp %p nf %p\n",
				d.ppnf, d.pnf, d.nfp, d.nf);
		ft_snapshot_push(snapshot, snapshot_depth,
			nr_snapshot, d.nf, d.depth);
		key_value = *(iter_key++);
		ft_descent_step(ft, &d, key_value);
	}

	/*
	 * A final reanchoring step that reached key_depth - 1 exited the loop
	 * without re-entering its top: honour a rewind > 0 conflict here too.
	 */
	if (caa_unlikely(d.skip_conflict)) {
		ret = -EAGAIN;
		goto insert_done;
	}
	/*
	 * Resolve any skip-compressed pointer left in d.nf by the descent
	 * loop's final step (e.g., ft_descent_traverse_compressed sets d.nf
	 * to cn->child raw, which may be skip-compressed).  The loop body's
	 * resolve at the top of each iteration only fires when the loop
	 * iterates again; a traverse that pushes d.depth to key_depth - 1
	 * exits the loop without re-entering.
	 */
	d.nf = ft_resolve_skip_compressed(ft, d.nf);

	if (d.depth == key_depth - 1) {
		/* Found either an internal, external node or NULL at end of key. */
		if (!d.nf) {
			dbg_printf("cds_ft_insert NULL ppnf %p pnf %p nfp %p nf %p\n",
					d.ppnf, d.pnf, d.nfp, d.nf);

			ft_lock_ctx_init(&actx, &d, ic.txn, ic.op);
			ret = ft_attach_node(ft, d.pnfp, d.pnf,
					d.nfp, d.nf, key, key_len, d.depth, node,
					NULL, &ic, &actx);
			if (ret == 0) {
				/*
				 * One-commit insert (ic.slot parked): the +1 count
				 * folds into insert_done's commit (an early +1 here
				 * would overcount -- the inverse of the nr_keys
				 * undercount discipline).  With order-statistics on
				 * the insert always parks, so no standalone walk
				 * remains.
				 */
				assert(ic.slot || !ft->rank_stats);
				if (d.depth >= 2)
					FT_TP(tree_edge_set, (const void *) ft,
						(const void *) d.ppnf,
						(unsigned int) (d.depth - 2),
						(uint8_t) key[d.depth - 2],
						(const void *) *d.pnfp);
			}

		} else if (ft_node_compressed(d.nf)) {
			/*
			 * Key ends at a compressed node's depth.
			 * Split: internal(external_nodes) + compressed(len-1).
			 */
			ret = ft_insert_compressed_key_shorter(ft, &d, 0,
				node, unique_node_ret, &ic);
		} else if (!ft_node_external(d.nf)) {
			struct cds_ft_node *external_nodes;
			struct cds_ft_metadata *metadata;

			assert(!ft_node_compressed(d.nf));
			metadata = cds_ft_item_to_metadata(ft_node_ptr(d.nf));
			external_nodes = metadata->external_nodes;
			if (external_nodes) {
				struct cds_ft_node *iter_node, *last_node = NULL;
				struct cds_ft_metadata *dup_hmeta = NULL;
				uintptr_t dup_hsnap = 0;

				if (unique_node_ret) {
					*unique_node_ret = external_nodes;
					ret = -EEXIST;
					goto insert_done;
				}
				/*
				 * MW LOCK_FINE (Step A, holder lock): the chain walk
				 * + tail append serialise on @metadata (the internal
				 * node whose external_nodes root this chain, the head's
				 * holder).  Acquire its node lock BEFORE the walk (a
				 * peer relinking the chain would send it into freed
				 * memory) and release after the append; ft_chain_node
				 * touches only last_node->next, never holder->state, so
				 * the held fence composes.  FT-wide lock makes the miss
				 * unreachable in soak (fault injection drives the bail).
				 */
				if (ft->lock_fine) {
#ifdef FEATURE_FT_FAULT_INJECT
					if (cds_ft_fault_lock_countdown >= 0) {
						if (cds_ft_fault_lock_countdown == 0) {
							cds_ft_fault_lock_countdown = -1;
							ret = -EAGAIN;
							goto insert_done;
						}
						cds_ft_fault_lock_countdown--;
					}
#endif
					{
						struct ft_lock_ctx hctx;
						struct ft_held_anchor hh;

						ft_lock_ctx_init(&hctx, &d,
							ic.txn, ic.op);
						if (ft_acquire_member(ft, &hctx,
								d.nf, metadata,
								d.depth, &hh)) {
							ret = -EAGAIN;
							goto insert_done;
						}
						/*
						 * SHARED is a SUCCESS: the word is
						 * already in this op's held set, so
						 * the chain walk is excluded and the
						 * FIRST acquire owns the release.
						 * Leaving @dup_hmeta NULL is exactly
						 * "held, owing no release".  Folding
						 * it into the miss above would refuse
						 * this op's OWN mark, which no retry
						 * can clear.
						 */
						if (!hh.shared) {
							dup_hmeta = hh.lock;
							dup_hsnap = hh.lock_snap;
						}
					}
				}
				/* Find last duplicate */
				iter_node = external_nodes;
				cds_ft_for_each_duplicate(iter_node)
					last_node = iter_node;

				dbg_printf("cds_ft_insert duplicate internal ppnf %p pnf %p nfp %p nf %p\n",
						d.ppnf, d.pnf, d.nfp, d.nf);

				/* Adding duplicate at existing key: no key count change. */
				ret = ft_chain_node(ft, last_node, node);
				if (dup_hmeta)
					ft_meta_lock_release(dup_hmeta);
				if (ret)
					goto insert_done;
			} else {
				/* New key at this internal node. */
				ft_external_head_set_parent(ft, node, d.nf);
				node->next = NULL;
				/*
				 * Park the external_nodes publish into the one-commit
				 * batch -- readers resolve the proxy via
				 * ft_dereference_external -- so the structural publish
				 * commits atomically with the ordinal-cell splice (list
				 * on) at insert_done's single flip, the +1 key count
				 * (rank stats) rides the SAME commit, AND the §4.B
				 * VALIDATE guard on the LIVE holder @metadata rides it
				 * too (ft_insert_park_external_nodes records it), so a
				 * concurrent remove that froze the holder aborts this
				 * publish.  List off + rank off carries only the forward
				 * external_nodes edge + that guard: a 2-record MCAS
				 * commit, slab-allocated and past the engine's nr==1
				 * fast path (no malloc) -- the multi-writer cost of
				 * guarding this in-place publish, no longer a bare lone
				 * flip.
				 */
				ret = ft_insert_commit_arm(ft, &ic,
					ft->rank_stats ? d.depth + 2 : 0);
				if (ret)
					goto insert_done;
				ft_insert_park_external_nodes(ft, &d,
					metadata, node, &ic);
				ic.count_from = d.nf;
				/*
				 * I1 (list on) / I2 (list off) count fold: @d.nf is
				 * the STABLE existing internal node the new key's
				 * external_nodes publish lands on, so its parent chain
				 * is commit-invariant.  Record the +1 count walk into
				 * the same txn (no fresh nodes on the path) and skip
				 * the post-commit propagate.  (Rank stats off: the
				 * record is a no-op and the flag skips a no-op walk.)
				 */
				ic.count_folded = true;
				ret = 0;
			}
		} else {
			struct cds_ft_node *iter_node, *last_node = NULL;
			struct cds_ft_node *dup_head =
				(struct cds_ft_node *) ft_node_ptr(d.nf);
			struct cds_ft_metadata *dup_hmeta = NULL;
			uintptr_t dup_hsnap = 0;

			if (unique_node_ret) {
				*unique_node_ret = dup_head;
				ret = -EEXIST;
				goto insert_done;
			}
			/*
			 * MW LOCK_FINE (Step A, holder lock): serialise the chain
			 * walk + tail append on the external head's holder (its
			 * immediate parent, resolved from the head itself via
			 * ft_chain_head_holder -- one hop, prev is the cell /
			 * flagged parent).  Acquire BEFORE the walk, release after
			 * the append (ft_chain_node touches only last_node->next).
			 * FT-wide lock makes the miss unreachable in soak (fault
			 * injection drives the bail).
			 *
			 * @dup_head is a PUBLISHED head the descent just reached, so
			 * it has a holder: ASSERT it rather than skipping the lock.
			 * A NULL means a never-inserted node (prev NULL), produced
			 * only by ft-insert's own unwind paths on UNPUBLISHED nodes,
			 * which cannot be here.  The old tolerance silently appended
			 * UNLOCKED, which the MW store's expected-value CAS still
			 * arbitrated -- but once these become sw it is a LOST UPDATE,
			 * so a wrong assumption must fail loudly now.  Measured
			 * unreachable: 0 NULL in 491532 ft_chain_head_holder calls
			 * across ft_unit and ft_inv's three list modes.
			 */
			if (ft->lock_fine) {
				struct cds_ft_inode_flag *holder_flag =
					ft_chain_head_holder(ft, dup_head);

				assert(holder_flag);
				{
					struct cds_ft_metadata *hm =
						ft_flag_to_metadata(ft, holder_flag);
#ifdef FEATURE_FT_FAULT_INJECT
					if (cds_ft_fault_lock_countdown >= 0) {
						if (cds_ft_fault_lock_countdown == 0) {
							cds_ft_fault_lock_countdown = -1;
							ret = -EAGAIN;
							goto insert_done;
						}
						cds_ft_fault_lock_countdown--;
					}
#endif
					{
						struct ft_lock_ctx hctx;
						struct ft_held_anchor hh;
						unsigned int hd;

						/*
						 * The head's holder IS the
						 * descent's parent (§5.2), so
						 * the window dates it.
						 */
						ft_lock_ctx_init(&hctx, &d,
							ic.txn, ic.op);
						if (!ft_lock_ctx_depth_of(ft,
								&hctx,
								holder_flag,
								&hd) ||
							ft_acquire_member(ft,
								&hctx,
								holder_flag,
								hm, hd, &hh)) {
							ret = -EAGAIN;
							goto insert_done;
						}
						/* Held already: no release owed. */
						if (!hh.shared) {
							dup_hmeta = hh.lock;
							dup_hsnap = hh.lock_snap;
						}
					}
				}
			}
			/* Find last duplicate */
			iter_node = dup_head;
			cds_ft_for_each_duplicate(iter_node)
				last_node = iter_node;

			dbg_printf("cds_ft_insert duplicate external ppnf %p pnf %p nfp %p nf %p\n",
					d.ppnf, d.pnf, d.nfp, d.nf);

			/* Adding duplicate at existing key: no key count change. */
			ret = ft_chain_node(ft, last_node, node);
			if (dup_hmeta)
				ft_meta_lock_release(dup_hmeta);
			if (ret)
				goto insert_done;
		}
	} else {
		/* Found NULL node or external node before end of key. */

		/*
		 * If the last node encountered during traversal is an external node,
		 * transform this external node into an internal node with associated
		 * external node, attach a new cluster as child of this internal node, and
		 * populate this new internal node into the trie to replace the prior
		 * external node.
		 * It's the same for NULL node, only that there is no need to chain any
		 * external node.
		 */

		dbg_printf("cds_ft_insert NULL or external ppnf %p pnf %p nfp %p nf %p\n",
				d.ppnf, d.pnf, d.nfp, d.nf);

		ft_lock_ctx_init(&actx, &d, ic.txn, ic.op);
		ret = ft_attach_node(ft, d.pnfp, d.pnf,
				d.nfp, d.nf, key, key_len, d.depth, node,
				(struct cds_ft_node *) ft_node_ptr(d.nf), &ic,
				&actx);
		if (ret == 0) {
			/* One-commit: +1 count folds into the commit (see above). */
			assert(ic.slot || !ft->rank_stats);
			if (d.depth >= 2)
				FT_TP(tree_edge_set, (const void *) ft,
					(const void *) d.ppnf,
					(unsigned int) (d.depth - 2),
					(uint8_t) key[d.depth - 2],
					(const void *) *d.pnfp);
		}
	}

insert_done:
	/*
	 * Concurrent-writer pre-commit conflict (-EAGAIN): the build found its
	 * descended position moved under a peer writer and bailed BEFORE arming or
	 * committing anything, freeing its own fresh cluster.  @precell is
	 * untouched (reused); re-descend against the now-current tree.
	 */
	if (ret == -EAGAIN) {
		if (ic.txn) {
			ft_flip_txn_destroy(ic.txn);
			ic.txn = NULL;
		}
		/*
		 * Age the pre-commit conflict (urcu_txn_conflict): without it a
		 * writer that keeps bailing on the same hot slot never advances
		 * txn->retry, never escalates, and can livelock.  Aging is all
		 * it does here -- the bail forfeits the FIFO turn, because the
		 * re-descend below asks a peer for the position it just lost.
		 */
		ft_txn_attempt_bail(&optxn, true);
		goto restart_attempt;
	}
	/*
	 * @node became a fresh head iff node->prev is still its (cell) carrier
	 * -- i.e. not external.  A duplicate append (ft_chain_node repointed
	 * node->prev at the predecessor) or a failed insert leaves @precell
	 * orphaned: free it, and on failure restore node->prev to its zeroed
	 * state so the application may retry, then splice the kept cell into the
	 * ordered list.  List off: no cell was allocated -- @node->prev is the
	 * flagged parent (fresh head) or the predecessor (dup), so there is
	 * nothing to free or splice; a FAILED insert may still have wired
	 * node->prev early (the build paths set the raw parent before their
	 * fallible publish, e.g. ft_try_compress_chain -> ft_set_parent, the
	 * split branch builders), and the failure unwind frees that cluster:
	 * reset it so the dangling pointer cannot leak into a retry -- the
	 * zeroed-prev check at entry would otherwise reject the node with
	 * -EINVAL forever.
	 */
	if (!ft->ordered_list) {
		if (ret != 0) {
			node->prev = NULL;
			if (ic.txn)
				ft_flip_txn_destroy(ic.txn);
		} else if (ic.slot) {
			/*
			 * One-commit: the structural slot publish was recorded
			 * into ic.txn (the key is invisible -- the slot reads
			 * its old value).  No cell to splice (list off), so
			 * commit the structural edges alone (@cell == NULL); a
			 * reader flips from not-present to the new key
			 * atomically.  The +1 count folds into that commit.
			 */
			cst = ft_insert_one_commit(ft, _key, _key_len, NULL, &ic);
			assert(ic.count_folded || !ft->rank_stats);
		} else if (ic.txn) {
			/* Armed but nothing parked (duplicate append): no
			 * structural publish to commit. */
			ft_flip_txn_destroy(ic.txn);
			ic.txn = NULL;
		}
	} else {
		if (ret != 0) {
			node->prev = NULL;
			ft_ord_cell_free_unpublished(ft, precell);
			if (ic.txn)
				ft_flip_txn_destroy(ic.txn);
		} else if (ft_node_external((struct cds_ft_inode_flag *) node->prev)) {
			ft_ord_cell_free_unpublished(ft, precell);
			if (ic.txn)
				ft_flip_txn_destroy(ic.txn);
		} else if (ic.slot) {
			/*
			 * One-commit: the structural slot is parked as a flip
			 * proxy (the key is still invisible).  Splice position
			 * + cell links first, then ONE commit flips the
			 * structural slot AND the ordered-list edges -- a
			 * reader never sees the head without its cell in the
			 * list.  The +1 count (the key only now counts) folds
			 * into that commit, from the shape's recorded base.
			 */
			cst = ft_insert_one_commit(ft, _key, _key_len, precell, &ic);
			assert(ic.count_folded || !ft->rank_stats);
		}
		/*
		 * No final else: after the one-commit conversion every
		 * published fresh head here parks a structural slot (ic.slot),
		 * so the old B-lite (ic.spliced) and post-publish-splice
		 * fallbacks are unreachable -- the cell is always spliced
		 * atomically by ft_insert_one_commit above.
		 */
	}
	/*
	 * Concurrent-writer commit conflict: a peer writer won the forward slot's
	 * expected-value CAS (or froze a §4.B-guarded node) at commit time, so
	 * nothing published and the fresh cluster was rolled back by the txn's
	 * on-abort action.  @precell was not spliced (the commit is atomic);
	 * re-descend and retry.
	 */
	if (cst == URCU_TXN_STATUS_ABORT) {
		/* The commit already aged the handle (retry++, keep the turn). */
		urcu_txn_end(&optxn);
		goto restart_attempt;
	}
	if (caa_unlikely(cst == URCU_TXN_STATUS_MEMORY_ERROR && ret == 0)) {
		/*
		 * Unreachable today: the one-commit txn is pre-reserved, so its
		 * commit is infallible ("reserved => infallible").  Defensive:
		 * should that invariant ever break, the failed commit published
		 * NOTHING (freeze-before-install; its on-abort action already
		 * freed the fresh cluster), so falling through with ret == 0
		 * would silently LOSE the insert.  Unwind like a failed attempt
		 * and surface the error instead.
		 */
		node->prev = NULL;
		if (ft->ordered_list)
			ft_ord_cell_free_unpublished(ft, precell);
		ret = -ENOMEM;
	}
	/*
	 * Terminal outcome (success or error): close this attempt's read-side
	 * section and release the escalation turn if held.
	 */
	urcu_txn_end(&optxn);
	if (ret == 0) {
		if (key_len > uatomic_load(&ft->max_used_key_len, CMM_RELAXED))
			uatomic_store(&ft->max_used_key_len, key_len, CMM_RELAXED);
	}

	return ret;
}

enum cds_ft_status cds_ft_insert(struct cds_ft *ft,
		const uint8_t *key, size_t key_len,
		struct cds_ft_node *node)
{
	int ret;

	CDS_FT_SCOPED_WRITER(ft);
	FT_TP_KEY(insert_enter, ft, key, key_len);
	ret = _cds_ft_insert(ft, key, key_len, node, NULL);

	if (ret == 0) {
		FT_TP(insert_exit, (int) CDS_FT_STATUS_OK);
		return CDS_FT_STATUS_OK;
	}
	if (ret == -EINVAL) {
		FT_TP(insert_exit, (int) CDS_FT_STATUS_INVALID_ARGUMENT_ERROR);
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}
	FT_TP(insert_exit, (int) CDS_FT_STATUS_MEMORY_ERROR);
	return CDS_FT_STATUS_MEMORY_ERROR;
}

enum cds_ft_status cds_ft_insert_unique(struct cds_ft *ft,
		const uint8_t *key, size_t key_len,
		struct cds_ft_node *node,
		struct cds_ft_node **result_node)
{
	int ret;
	struct cds_ft_node *ret_node = NULL;

	CDS_FT_SCOPED_WRITER(ft);
	FT_TP_KEY(insert_unique_enter, ft, key, key_len);
	ret = _cds_ft_insert(ft, key, key_len, node, &ret_node);
	if (ret == -EEXIST) {
		*result_node = ret_node;
		FT_TP(insert_unique_exit, (int) CDS_FT_STATUS_DUPLICATE_FOUND);
		return CDS_FT_STATUS_DUPLICATE_FOUND;
	}
	if (ret == -EINVAL) {
		*result_node = NULL;
		FT_TP(insert_unique_exit, (int) CDS_FT_STATUS_INVALID_ARGUMENT_ERROR);
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}
	if (ret) {
		*result_node = NULL;
		FT_TP(insert_unique_exit, (int) CDS_FT_STATUS_MEMORY_ERROR);
		return CDS_FT_STATUS_MEMORY_ERROR;
	}
	*result_node = node;
	FT_TP(insert_unique_exit, (int) CDS_FT_STATUS_OK);
	return CDS_FT_STATUS_OK;
}

/*
 * Insert a node, replacing the entire existing duplicate chain at the
 * same key if one exists.
 *
 * On success, *@old_node_ret is set to the head of the replaced chain
 * (or NULL if no prior node existed). The caller must wait for a grace
 * period before reclaiming the old chain.
 *
 * Returns 0 on success, -EINVAL on bad arguments, or a negative errno
 * on memory allocation failure.
 */
static
int _cds_ft_insert_replace(struct cds_ft *ft,
		const uint8_t *_key, size_t _key_len,
		struct cds_ft_node *node,
		struct cds_ft_node **old_node_ret)
{
	unsigned int key_depth;
	struct ft_descent d;
	const uint8_t *iter_key;
	size_t key_len = ft_key_len(ft, _key_len);
	const struct cds_ft_key_map *km = &ft->group->key_map;
	uint8_t ordinal_buf[FT_MAX_KEY_LEN];
	const uint8_t *key;
	struct cds_ft_inode_flag *snapshot[FT_MAX_DEPTH];
	unsigned int snapshot_depth[FT_MAX_DEPTH];
	int nr_snapshot = 0;
	int ret;
	struct ft_ord_cell *precell;
	void *cell = NULL;		/* @precell's carrier; reused across retries */
	struct urcu_txn optxn;
	struct ft_insert_commit ic = { 0 };
	/*
	 * The attach's recompactions lock {C, P, GP}; @d dates them and @ic.txn
	 * names what this op already holds.  Re-initialised at each attach,
	 * because the txn is armed part-way through.
	 */
	struct ft_lock_ctx actx;
	enum urcu_txn_status cst = URCU_TXN_STATUS_OK;	/* one-commit outcome */

	*old_node_ret = NULL;

	if (!valid_external_node(node) || !valid_key_len(ft, key_len))
		return -EINVAL;
	if (caa_likely(km->identity)) {
		key = _key;
	} else {
		ft_key_to_ordinals(ordinal_buf, _key, key_len, km);
		key = ordinal_buf;
	}
	/* Expect zeroed prev/next pointers. */
	if (node->prev || ft_node_next(node))
		return -EINVAL;

	/* Ordered-list trie: pre-wire @node's cell (see _cds_ft_insert).  A replace
	 * always lands @node as the sole head on success, so the cell is kept
	 * unless the insert fails (or the key_shorter path finds the key and
	 * leaves @node uninstalled -- both freed below).  List off: no cell. */
	precell = NULL;
	if (ft->ordered_list) {
		cell = ft_ord_cell_alloc(ft, node, NULL);

		if (!cell)
			return -ENOMEM;
		node->prev = cell;
		precell = ft_ord_cell_ptr(cell);
		/*
		 * Head's last edge byte for the up-walk key rebuild: the cell is
		 * the head's metadata record and @key is ordinal here, so
		 * key[key_len - 1] is the byte the head hangs under (ignored by the
		 * up-walk when the head's parent is a compressed node, whose
		 * key_bytes already span the head's position).
		 */
		if (key_len)
			cds_ft_item_to_metadata(precell)->incoming_byte =
				(uint8_t) key[key_len - 1];
	}

	/*
	 * The op's persistent engine handle, initialised ONCE so contention
	 * aging, the FIFO escalation turn and the learned descriptor size span
	 * every attempt (doc §11) -- exactly as _cds_ft_insert does.  @cell /
	 * @precell are allocated above and REUSED: only a committing attempt
	 * splices the cell, so a bailed one leaves it ours.
	 */
	ft_txn_op_init(ft, &optxn);

restart_replace_attempt:
	urcu_txn_begin(&optxn);
	/*
	 * Per-attempt state.  A bailed attempt published nothing and its fresh
	 * cluster was already rolled back, so re-arm @node's linkage and the
	 * commit scope and re-descend from the root against the current tree.
	 */
	ic = (struct ft_insert_commit){ 0 };
	ic.op = &optxn;
	cst = URCU_TXN_STATUS_OK;
	ret = 0;
	nr_snapshot = 0;
	*old_node_ret = NULL;
	node->prev = cell;		/* NULL when the list is off */
	node->next = NULL;
	if (precell && key_len)
		cds_ft_item_to_metadata(precell)->incoming_byte =
			(uint8_t) key[key_len - 1];

	key_depth = key_len + 1;

	dbg_printf("_cds_ft_insert_replace attempt: node %p\n", node);
	iter_key = key;
	ft_descent_init(&d, ft);

	for (; d.depth < key_depth - 1; ) {
		uint8_t key_value;

		if (!d.nf)
			break;
		/* Resolve skip-compressed pointer. */
		d.nf = ft_resolve_skip_compressed(ft, d.nf);
		if (ft_node_external(d.nf))
			break;
		if (ft_node_compressed(d.nf)) {
			enum ft_descent_action act;

			act = ft_insert_compressed(ft, &d, &iter_key,
				key, key_len, key_depth, node,
				NULL, snapshot, snapshot_depth,
				&nr_snapshot, &ret, &ic);
			if (act == FT_DESCENT_END)
				goto insert_replace_done;
			if (act == FT_DESCENT_BREAK)
				break;
			continue;
		}
		dbg_printf("_cds_ft_insert_replace iter ppnf %p pnf %p nfp %p nf %p\n",
				d.ppnf, d.pnf, d.nfp, d.nf);
		ft_snapshot_push(snapshot, snapshot_depth,
			nr_snapshot, d.nf, d.depth);
		key_value = *(iter_key++);
		ft_descent_step(ft, &d, key_value);
	}

	/*
	 * Resolve any skip-compressed pointer left in d.nf by a final
	 * traverse that exited the loop without re-entering the loop's
	 * resolve step.
	 */
	d.nf = ft_resolve_skip_compressed(ft, d.nf);

	if (d.depth == key_depth - 1) {
		/* Found either an internal, external node or NULL at end of key. */
		if (!d.nf) {
			/* No existing node. Regular attach. */
			dbg_printf("_cds_ft_insert_replace NULL at end of key\n");

			ft_lock_ctx_init(&actx, &d, ic.txn, ic.op);
			ret = ft_attach_node(ft, d.pnfp, d.pnf,
					d.nfp, d.nf, key, key_len, d.depth, node,
					NULL, &ic, &actx);
			if (ret == 0) {
				/* Parked one-commit: +1 folds into the commit. */
				assert(ic.slot || !ft->rank_stats);
				if (d.depth >= 2)
					FT_TP(tree_edge_set, (const void *) ft,
						(const void *) d.ppnf,
						(unsigned int) (d.depth - 2),
						(uint8_t) key[d.depth - 2],
						(const void *) *d.pnfp);
			}
		} else if (ft_node_compressed(d.nf)) {
			/*
			 * Key ends at a compressed node's depth.
			 * Split: internal(external_nodes) + compressed(len-1).
			 */
			ret = ft_insert_compressed_key_shorter(ft, &d, 0,
				node, old_node_ret, &ic);
			if (ret == -EEXIST) {
				ret = 0;	/* Replace handled by key_shorter. */
				/*
				 * key_shorter found the key already present and
				 * left @node uninstalled (*old_node_ret names the
				 * existing chain, which keeps its own cell).  Drop
				 * the pre-wired cell from node->prev; insert_replace_done
				 * frees the now-orphaned @precell.
				 */
				node->prev = NULL;
			}
		} else if (!ft_node_external(d.nf)) {
			struct cds_ft_node *external_nodes;
			struct cds_ft_metadata *metadata;

			assert(!ft_node_compressed(d.nf));
			metadata = cds_ft_item_to_metadata(ft_node_ptr(d.nf));
			external_nodes = metadata->external_nodes;
			if (external_nodes) {
				dbg_printf("_cds_ft_insert_replace: replacing internal metadata chain %p\n",
						external_nodes);
				/* Replace existing chain: key count unchanged. */
				*old_node_ret = external_nodes;
				ft_external_head_set_parent(ft, node, d.nf);
				node->next = NULL;
				/*
				 * Ordered list on: publish the new head into
				 * external_nodes AND swap its cell in ONE atomic flip
				 * -- external_nodes is a plain pointer slot, readers
				 * resolve a parked proxy via ft_dereference_external.
				 * List off: a plain publish, no cell.
				 */
				if (ft->ordered_list) {
					struct ft_ord_cell *old_cell =
						ft_ord_cell_ptr(external_nodes->prev);
					struct ft_ord_cell_edge sedge = {
						.slot = (struct ft_ord_cell **)
							&metadata->external_nodes,
						.old_target = (struct ft_ord_cell *)
							external_nodes,
						.new_target = (struct ft_ord_cell *)
							node,
					};
					struct ft_flip_txn *txn =
						ft_flip_txn_create_bounded(ft,
						FT_ORD_CELL_SWAP_PUBLISH_MAX_EDGES + 1);

					/*
					 * The flip is the op's sole side-effect (the
					 * new head is fresh); on OOM nothing is applied,
					 * the old chain is intact (do NOT free its cell)
					 * and the replace aborts retriably.  Arm a
					 * pre-reserved txn so the §4.B VALIDATE guard on
					 * the LIVE holder @d.nf rides the swap flip (a
					 * concurrent remove that froze the holder aborts
					 * this replace); ft_ord_cell_swap_publish_multi
					 * then fuses the external_nodes publish + the head
					 * cell's swap and commits it infallibly.
					 */
					if (!txn) {
						ret = -ENOMEM;
						goto insert_replace_done;
					}
					ft_flip_txn_guard_parent(ft, txn, d.nf);
					ft_replace_fault_arm_abort(txn);
					/*
					 * On a peer-conflict ABORT the commit installs
					 * NOTHING: the old chain and its cell stay LIVE.
					 * Dropping the status here reported the replace as
					 * OK and then freed @old_cell -- a live cell, still
					 * linked in the ordered list.  Surface -EAGAIN: the
					 * done handler's pre-commit test re-attempts, KEEPING
					 * @precell (nothing was published, so it is still
					 * ours), exactly as the head arms below do.
					 */
					if (ft_ord_cell_swap_publish_multi(ft, old_cell,
							precell, &sedge, 1, txn) != 0) {
						ret = -EAGAIN;
						goto insert_replace_done;
					}
					ft_ord_cell_free(ft, old_cell);
				} else {
					/*
					 * List off replace: external_nodes is the single
					 * reader-visible slot (readers resolve via
					 * ft_dereference_external).  Publish external_nodes:
					 * old chain -> @node on a guarded STANDALONE txn so
					 * the §4.B VALIDATE guard on the LIVE holder @d.nf
					 * rides the flip -- NOT the park: a replace changes
					 * no key count, so it must NOT enter the count-
					 * folding one-commit path (insert_one_commit asserts
					 * count_folded when rank stats are on).  Mirrors the
					 * list-on chain replace above, minus the cell.  Two
					 * edges: the forward external_nodes store + the guard.
					 */
					struct ft_ord_cell_edge sedge = {
						.slot = (struct ft_ord_cell **)
							&metadata->external_nodes,
						.old_target = (struct ft_ord_cell *)
							external_nodes,
						.new_target = (struct ft_ord_cell *)
							node,
					};
					struct ft_flip_txn *txn =
						ft_flip_txn_create_bounded(ft, 2);

					if (!txn) {
						ret = -ENOMEM;
						goto insert_replace_done;
					}
					ft_flip_txn_guard_parent(ft, txn, d.nf);
					ft_replace_fault_arm_abort(txn);
					/* Installs nothing on either failure, as the
					 * head arm: ABORT -> -EAGAIN (retry),
					 * MEMORY_ERROR -> -ENOMEM (do not). */
					ret = ft_flip_status_to_errno(
						ft_ord_cell_flip_into(ft, txn,
							&sedge, 1));
					if (ret)
						goto insert_replace_done;
				}
			} else {
				/* No external nodes yet. New key at this node. */
				ft_external_head_set_parent(ft, node, d.nf);
				node->next = NULL;
				/*
				 * Park external_nodes -- readers resolve the proxy via
				 * ft_dereference_external -- so it commits atomically
				 * with the ordinal-cell splice (list on) AND the §4.B
				 * VALIDATE guard on the LIVE holder @metadata rides the
				 * flip (ft_insert_park_external_nodes records it).  List
				 * off + rank off carries only the forward edge + that
				 * guard: a 2-record slab-allocated MCAS commit, not a
				 * bare lone flip -- the multi-writer cost of guarding
				 * this in-place publish.
				 */
				ret = ft_insert_commit_arm(ft, &ic,
					ft->rank_stats ? d.depth + 2 : 0);
				if (ret)
					goto insert_replace_done;
				ft_insert_park_external_nodes(ft, &d,
					metadata, node, &ic);
				ic.count_from = d.nf;
				/* I1 (list on) / I2 (list off) count fold: see cds_ft_insert. */
				ic.count_folded = true;
			}
			ret = 0;
		} else {
			dbg_printf("_cds_ft_insert_replace: replacing external chain %p\n",
					ft_node_ptr(d.nf));
			/* External node at end of key. Replace chain: key count unchanged. */
			*old_node_ret = (struct cds_ft_node *) ft_node_ptr(d.nf);
			ft_external_head_set_parent(ft, node, d.pnf);
			node->next = NULL;
			{
				/*
				 * Atomic replace: publish the new head into every
				 * reader-visible slot in ONE flip.  The plain case is a
				 * single forward edge at d.nfp (a plain external child, or
				 * a plainly-reached compressed parent's cn->child).  A leaf
				 * reached through a SKIP_X suffix is a DUAL: the exact
				 * descent reads cn->child while the candidate descent reads
				 * the grandparent skip slot, and BOTH name the leaf -- so
				 * the new head must appear at both atomically or a reader
				 * sees them disagree (and the old leaf is freed after this).
				 * Recording cn->child as a second flip edge (not a separate
				 * bare store) fuses the pair, mirroring the remove
				 * external-promote dual (ft_remove_commit_rec).  This sedge
				 * model is shared by both ordered-list states; the cell swap
				 * rides the same flip when the list is on.
				 */
				struct ft_ord_cell_edge sedges[2] = { 0 };
				unsigned int n_sedge = 0;

				sedges[0].slot = (struct ft_ord_cell **) d.nfp;
				sedges[0].old_target = (struct ft_ord_cell *) d.nf;
				sedges[0].new_target = (struct ft_ord_cell *) node;
				n_sedge = 1;

#ifdef FEATURE_FT_SKIP_COMPRESSED
				if (ft_node_compressed(d.pnf)) {
					struct cds_ft_compressed_node *cn =
						ft_compressed_node_ptr(d.pnf);
					struct cds_ft_metadata *cn_meta =
						cds_ft_item_to_metadata(
							(struct cds_ft_inode *) cn);
					struct cds_ft_inode_flag **sslot =
						ft_get_parent_slot(cn_meta, ft);

					if (sslot && ft_node_skip_compressed(*sslot)) {
						/* edge 0: grandparent SKIP_X dual. */
						sedges[0].slot =
							(struct ft_ord_cell **) sslot;
						sedges[0].old_target =
							(struct ft_ord_cell *) *sslot;
						sedges[0].new_target =
							(struct ft_ord_cell *)
							ft_skip_compressed_flag(
								(struct cds_ft_inode_flag *)
									node, cn->len);
						/* edge 1: cn->child forward (exact descent). */
						sedges[1].slot =
							(struct ft_ord_cell **) &cn->child;
						sedges[1].old_target =
							(struct ft_ord_cell *) cn->child;
						sedges[1].new_target =
							(struct ft_ord_cell *) node;
						n_sedge = 2;
					}
				}
#endif
				if (ft->ordered_list) {
					/*
					 * Swap the structural slot(s) AND the head's
					 * cell in one flip.  MW: the forward edge stores
					 * into the LIVE holder @d.pnf's child slot, so
					 * guard it §4.B -- a {live->live} VALIDATE on
					 * @d.pnf rides the same flip, so a peer that
					 * relocated/froze @d.pnf ABORTS this commit rather
					 * than the store landing in a stale/freed slot (the
					 * SKIP dual's grandparent slot resolves off @d.pnf,
					 * whose liveness the guard covers).  Pre-reserve the
					 * txn (guard + swap/publish edges) BEFORE any live
					 * store so the flip is OOM-infallible; on create
					 * failure nothing is applied, the old head's cell is
					 * NOT freed and the replace aborts retriably.
					 */
					struct ft_ord_cell *old_cell =
						ft_ord_cell_ptr((*old_node_ret)->prev);
					struct ft_flip_txn *txn =
						ft_flip_txn_create_bounded(ft,
							FT_ORD_CELL_SWAP_PUBLISH_MAX_EDGES +
							1 /* §4.B parent guard */);

					if (!txn) {
						ret = -ENOMEM;
						goto insert_replace_done;
					}
					/* VALIDATE (§4.B): guard the LIVE holder @d.pnf. */
					ft_flip_txn_guard_parent(ft, txn, d.pnf);
					ft_replace_fault_arm_abort(txn);
					/*
					 * On a peer-conflict ABORT the commit installs
					 * nothing (the old head and its cell stay live), so
					 * surface -EAGAIN -- the done handler's pre-commit
					 * test re-attempts and @precell is KEPT for the next
					 * attempt -- and do NOT free @old_cell (the swap did
					 * not happen).
					 */
					if (ft_ord_cell_swap_publish_multi(ft, old_cell,
							precell, sedges, n_sedge,
							txn) != 0) {
						ret = -EAGAIN;
						goto insert_replace_done;
					}
					ft_ord_cell_free(ft, old_cell);
				} else {
					/*
					 * List off: no cell.  The forward edge stores into
					 * the LIVE holder @d.pnf's child slot -- guard it
					 * §4.B and force the (possibly lone) edge through a
					 * PRE-RESERVED txn so the guard can ABORT a peer
					 * conflict: a bare lone store commits infallibly but
					 * cannot detect one (the not-yet-MW lone-store
					 * residue).  Pre-reserve BEFORE any live store; on
					 * create failure nothing is applied, the old head is
					 * NOT freed and the replace aborts retriably.
					 */
					struct ft_flip_txn *txn =
						ft_flip_txn_create_bounded(ft,
							FT_PUB_SEDGE_MAX_EDGES +
							1 /* §4.B parent guard */);

					if (!txn) {
						ret = -ENOMEM;
						goto insert_replace_done;
					}
					/* VALIDATE (§4.B): guard the LIVE holder @d.pnf. */
					ft_flip_txn_guard_parent(ft, txn, d.pnf);
					ft_replace_fault_arm_abort(txn);
					/* -EAGAIN on a peer-conflict ABORT (nothing
					 * installed); the op's own retry loop re-descends. */
					if (ft_ord_cell_flip_into(ft, txn, sedges,
							n_sedge) != 0) {
						ret = -EAGAIN;
						goto insert_replace_done;
					}
				}
			}
			ret = 0;
		}
	} else {
		/*
		 * Found NULL node or external node before end of key.
		 * Attach a new branch, displacing any shorter-key
		 * external node into the new branch's metadata.
		 */
		dbg_printf("_cds_ft_insert_replace: attach before end of key\n");

		ft_lock_ctx_init(&actx, &d, ic.txn, ic.op);
		ret = ft_attach_node(ft, d.pnfp, d.pnf,
				d.nfp, d.nf, key, key_len, d.depth, node,
				(struct cds_ft_node *) ft_node_ptr(d.nf), &ic,
				&actx);
		if (ret == 0) {
			/* Parked one-commit: +1 folds into the commit. */
			assert(ic.slot || !ft->rank_stats);
			if (d.depth >= 2)
				FT_TP(tree_edge_set, (const void *) ft,
					(const void *) d.ppnf,
					(unsigned int) (d.depth - 2),
					(uint8_t) key[d.depth - 2],
					(const void *) *d.pnfp);
		}
	}

insert_replace_done:
	/*
	 * Concurrent-writer PRE-COMMIT conflict: the build found its descended
	 * position moved under a peer and bailed before arming or committing
	 * anything, freeing its own fresh cluster.  This test comes FIRST, ahead
	 * of the cleanup below, precisely because that cleanup frees @precell --
	 * a retry must keep it.
	 */
	if (ret == -EAGAIN) {
		if (ic.txn) {
			ft_flip_txn_destroy(ic.txn);
			ic.txn = NULL;
		}
		ft_txn_attempt_bail(&optxn, true);
		goto restart_replace_attempt;
	}
	/*
	 * @node became the installed head iff node->prev still carries its
	 * pre-wired cell (not external).  A duplicate append (descent through a
	 * compressed node chained @node), the uninstalled key_shorter -EEXIST
	 * path (prev NULLed above), or a failed insert leaves @precell orphaned
	 * -- free it, and on failure restore node->prev to its zeroed state.
	 * List off: no cell, nothing to free or splice (the swap sites above are
	 * gated too) -- but a FAILED insert may have wired node->prev early in
	 * a build path whose cluster the unwind then freed: reset it so the
	 * retry does not hit the zeroed-prev entry check (see _cds_ft_insert).
	 */
	if (!ft->ordered_list) {
		if (ret != 0) {
			node->prev = NULL;
			if (ic.txn)
				ft_flip_txn_destroy(ic.txn);
		} else if (ic.slot) {
			/*
			 * One-commit (list off): commit the recorded structural
			 * publish alone (@cell == NULL); the +1 count folds into
			 * that commit.  A pure replace does not park (ic.slot
			 * unset) and falls through untouched.
			 */
			ft_replace_fault_arm_abort(ic.txn);
			cst = ft_insert_one_commit(ft, _key, _key_len, NULL, &ic);
			assert(ic.count_folded || !ft->rank_stats);
		} else if (ic.txn) {
			/* Armed but nothing parked (duplicate append): no
			 * structural publish to commit. */
			ft_flip_txn_destroy(ic.txn);
			ic.txn = NULL;
		}
	} else {
		if (ret != 0) {
			node->prev = NULL;
			ft_ord_cell_free_unpublished(ft, precell);
			if (ic.txn)
				ft_flip_txn_destroy(ic.txn);
		} else if (ft_node_external((struct cds_ft_inode_flag *) node->prev)) {
			/* Duplicate append / -EEXIST (prev NULL): cell orphaned. */
			ft_ord_cell_free_unpublished(ft, precell);
			if (ic.txn)
				ft_flip_txn_destroy(ic.txn);
		} else if (ic.slot) {
			/*
			 * Fresh head, parked one-commit: structural publish +
			 * ordinal-cell splice flip atomically.  A replace
			 * (*old_node_ret set) does not park -- it swapped @precell
			 * into the replaced head's list slot at the replace site,
			 * so it falls through here untouched.
			 */
			ft_replace_fault_arm_abort(ic.txn);
			cst = ft_insert_one_commit(ft, _key, _key_len, precell, &ic);
			assert(ic.count_folded || !ft->rank_stats);
		}
	}
	/*
	 * The one-commit's status is the op's whole outcome and MUST be checked on
	 * both arms above.  Discarding it turns an ABORT (nothing published, the
	 * fresh cluster rolled back by the txn's on-abort action) into OK or
	 * DUPLICATE_FOUND with the key absent: a lost insert reported as success,
	 * @precell leaked, and @node left with a stale ->prev that makes the
	 * caller's retry fail entry validation with -EINVAL forever.
	 *
	 * _cds_ft_insert has always captured this and re-descended; this op has no
	 * retry loop (it is contract-excluded under FINE), so it unwinds the same
	 * way a failed attempt does and surfaces the condition to the caller.
	 * MEMORY_ERROR stays defensive-only, exactly as in _cds_ft_insert: the
	 * one-commit txn is pre-reserved, so its commit cannot allocate.
	 */
	if (caa_unlikely(cst == URCU_TXN_STATUS_ABORT) && ret == 0) {
		/*
		 * A peer won the one-commit's expected-value CAS: nothing
		 * published, the fresh cluster rolled back by the txn's on-abort
		 * action, and @precell was not spliced (the commit is atomic).
		 * Re-descend.  The commit already aged the handle, so this edge
		 * KEEPS the turn -- ft_txn_attempt_end, not _bail.
		 */
		ft_txn_attempt_end(&optxn, true);
		goto restart_replace_attempt;
	}
	if (caa_unlikely(cst != URCU_TXN_STATUS_OK) && ret == 0) {
		/*
		 * Defensive only, as in _cds_ft_insert: the one-commit txn is
		 * pre-reserved, so its commit cannot allocate.  Publishing
		 * nothing while reporting OK would lose the insert.
		 */
		node->prev = NULL;
		if (ft->ordered_list)
			ft_ord_cell_free_unpublished(ft, precell);
		*old_node_ret = NULL;
		ret = -ENOMEM;
	}
	if (ret == 0) {
		if (key_len > uatomic_load(&ft->max_used_key_len, CMM_RELAXED))
			uatomic_store(&ft->max_used_key_len, key_len, CMM_RELAXED);
	}

	urcu_txn_end(&optxn);
	return ret;
}

enum cds_ft_status cds_ft_insert_replace(struct cds_ft *ft,
		const uint8_t *key, size_t key_len,
		struct cds_ft_node *node,
		struct cds_ft_node **result_node)
{
	struct cds_ft_node *old_node = NULL;
	int ret;

	CDS_FT_SCOPED_WRITER(ft);
	FT_TP_KEY(insert_replace_enter, ft, key, key_len);
	ret = _cds_ft_insert_replace(ft, key, key_len, node, &old_node);
	if (ret == -EINVAL) {
		*result_node = NULL;
		FT_TP(insert_replace_exit, (int) CDS_FT_STATUS_INVALID_ARGUMENT_ERROR);
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}
	if (ret == -EAGAIN) {
		/*
		 * A commit ABORT: a peer won an expected-value CAS or froze a
		 * guarded node, so nothing published and @node is reusable.  The
		 * catch-all below reported this as MEMORY_ERROR -- a peer conflict
		 * dressed as an allocation failure, which a caller cannot act on.
		 */
		*result_node = NULL;
		FT_TP(insert_replace_exit, (int) CDS_FT_STATUS_BUSY_ERROR);
		return CDS_FT_STATUS_BUSY_ERROR;
	}
	if (ret) {
		*result_node = NULL;
		FT_TP(insert_replace_exit, (int) CDS_FT_STATUS_MEMORY_ERROR);
		return CDS_FT_STATUS_MEMORY_ERROR;
	}
	*result_node = old_node;
	if (old_node) {
		FT_TP(insert_replace_exit, (int) CDS_FT_STATUS_DUPLICATE_FOUND);
		return CDS_FT_STATUS_DUPLICATE_FOUND;
	}
	FT_TP(insert_replace_exit, (int) CDS_FT_STATUS_OK);
	return CDS_FT_STATUS_OK;
}

static
enum cds_ft_status _cds_ft_replace_locked(struct cds_ft *ft,
		struct cds_ft_iter *iter,
		struct cds_ft_node *old_node,
		struct cds_ft_node *new_node,
		bool *need_retry,
		struct urcu_txn *op)
{
	struct cds_ft_inode_flag *holder_flag;
	struct cds_ft_inode_flag **pub_slot;
	struct cds_ft_compressed_node *cn = NULL;
	const uint8_t *iter_key;
	size_t key_len = ft_key_len(ft, ft_iter_resolve_key_len(iter));
	enum cds_ft_status s;

	FT_TP_ITER_KEY(replace_enter, iter);

	/*
	 * If the iterator has a valid path, the RCU read-side lock must
	 * be held.
	 */
	if (iter->cache_valid)
		CDS_FT_ASSERT_RCU_READ_LOCKED(ft);
	iter_debug_path_check(iter);

	if (!valid_external_node(old_node) || !valid_external_node(new_node)
			|| !valid_key_len(ft, key_len)) {
		s = CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
		FT_TP(replace_exit, (int) s);
		return s;
	}
	/* Expect zeroed next and prev pointers on new_node. */
	if (ft_node_next(new_node) || new_node->prev) {
		s = CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
		FT_TP(replace_exit, (int) s);
		return s;
	}

	iter_key = ft_iter_read_key(iter);

	dbg_printf("cds_ft_replace: old_node %p new_node %p\n", old_node, new_node);

	/*
	 * No top-down descent.  As in cds_ft_remove, @old_node is
	 * application-owned and -- with the RCU read-side lock held
	 * continuously since it was obtained -- alive; the writer mutex held
	 * here freezes the structure, so @old_node->prev is a settled live
	 * pointer to its holder.  @new_node takes @old_node's exact place in
	 * the duplicate chain, so the key count and trie shape are unchanged:
	 * only the chain link (or head slot) that points at @old_node is
	 * repointed at @new_node.  That slot is derived from the holder -- the
	 * predecessor's next for a non-head duplicate, else the head slot
	 * cds_ft_remove recovers (a compressed holder's &cn->child, an
	 * internal holder's external_nodes, or an internal body slot keyed by
	 * the last key byte).
	 */
	if (ft_node_is_removed(old_node)) {
		/* Already unlinked from the trie. */
		s = CDS_FT_STATUS_NOT_FOUND;
		FT_TP(replace_exit, (int) s);
		return s;
	}
	holder_flag = ft_node_holder(ft, old_node);
	if (!holder_flag) {
		/* Never inserted (a freshly-initialized node). */
		s = CDS_FT_STATUS_NOT_FOUND;
		FT_TP(replace_exit, (int) s);
		return s;
	}

	if (ft_node_external(holder_flag)) {
		/* Non-head duplicate: repoint the predecessor's next. */
		pub_slot = (struct cds_ft_inode_flag **)
			&((struct cds_ft_node *) holder_flag)->next;
	} else if (ft_node_compressed(holder_flag) ||
		   ft_node_skip_compressed(holder_flag)) {
		/* Compressed holder: @old_node is its single external child. */
		cn = ft_node_skip_compressed(holder_flag) ?
			ft_skip_to_compressed(ft, holder_flag) :
			ft_compressed_node_ptr(holder_flag);
		if ((struct cds_ft_node *) ft_node_ptr(cn->child) != old_node) {
			s = CDS_FT_STATUS_NOT_FOUND;
			FT_TP(replace_exit, (int) s);
			return s;
		}
		pub_slot = &cn->child;
	} else if (ft_node_external_nodes(holder_flag) ==
			(struct cds_ft_node *) old_node) {
		/* Internal holder: @old_node heads its external_nodes chain. */
		pub_slot = (struct cds_ft_inode_flag **)
			&cds_ft_item_to_metadata(ft_node_ptr(holder_flag))->external_nodes;
	} else {
		/* Internal holder: @old_node is a body child (leaf key). */
		struct cds_ft_inode_flag *child;

		child = ft_node_get_nth_skip(holder_flag, &pub_slot,
			iter_key[key_len - 1], FT_PF_NONE);
		if (!child ||
		    (struct cds_ft_node *) ft_node_ptr(child) != old_node) {
			s = CDS_FT_STATUS_NOT_FOUND;
			FT_TP(replace_exit, (int) s);
			return s;
		}
	}

	/*
	 * Splice @new_node into @old_node's chain position.  When @old_node is a
	 * chain HEAD with the ordered list on, publish a FRESH cell for @new_node
	 * (its node field set while the cell is hidden -- cell->node stays
	 * WRITE-ONCE, the way the public cds_ft_cell_node reads it as a plain
	 * pointer) and swap it in for @old_node's cell FUSED with the structural
	 * forward publish and -- for a compressed holder -- the grandparent SKIP_X
	 * dual, in ONE flip: a reader never sees @new_node at one index but
	 * @old_node (about to be freed) at another, nor a stale skip target into
	 * @old_node (the dangling-skip UAF).  A non-head duplicate (no cell) is a
	 * single predecessor->next store; a list-off head inherits the flagged
	 * parent and flips the structural + SKIP_X dual.  Mirrors ft_promote_head.
	 */
	{
		bool is_head = !ft_node_external(
			(struct cds_ft_inode_flag *) old_node->prev);
		struct cds_ft_inode_flag *parent_nf = cn ?
			ft_compressed_node_flag(cn) : holder_flag;
		struct ft_ord_cell *old_cell = (ft->ordered_list && is_head) ?
			ft_ord_cell_ptr(old_node->prev) : NULL;
		void *new_cell_flag = NULL;
		struct ft_pub_rec rec = { .n = 0 };
		struct ft_ord_cell_edge sedges[2] = { 0 };
		unsigned int n_s;
		int r;			/* head-arm commit outcome */

		/*
		 * Alloc the fresh cell BEFORE any mutation so OOM aborts cleanly
		 * (@new_node still carries its zeroed links, @old_node is intact).
		 */
		if (old_cell) {
			new_cell_flag = ft_ord_cell_alloc(ft, new_node,
				old_cell->parent);
			if (!new_cell_flag) {
				s = CDS_FT_STATUS_MEMORY_ERROR;
				FT_TP(replace_exit, (int) s);
				return s;
			}
		}
		/* @new_node's successor link is build-invisible (it is fresh). */
		new_node->next = ft_node_next(old_node);
		if (!is_head) {
			/*
			 * Non-head duplicate: atomic replace.  ft_hlist_replace_
			 * prepare marks @old_node (freeze), swings the predecessor's
			 * next (old_node -> new_node) and the successor's prev
			 * (old_node -> new_node) to @new_node, and builds @new_node's
			 * links, in ONE commit -- the freeze rides the swap (doc
			 * §4.B), so a racing del/insert_after fails its old-value
			 * check once this is a concurrent engine.  Multi-edge: readers
			 * resolve the transient interior-next proxies via
			 * cds_ft_node_next_rcu.  It marks @old_node, so the common
			 * freeze below is skipped for this case.  Abortable cleanly:
			 * on a txn-alloc OOM nothing is published and @old_node /
			 * @new_node are intact.
			 */
			struct cds_ft_inode_flag *lock_nf;
			struct cds_ft_metadata *hm = NULL;
			uintptr_t hsnap = 0;
			struct ft_flip_txn *txn;
			enum urcu_txn_status cst;

			/*
			 * {L}: a duplicate chain is owned by its HEAD-HOLDER's
			 * node lock -- ft_hlist_replace_prepare states that
			 * contract and DROPPED its multi-writer arbitration on the
			 * strength of it, so acquire the holder here exactly as the
			 * duplicate append does (ft_chain_node's caller) and as the
			 * interior unchain does (ft_unchain_node).  Released after
			 * the commit: this txn writes only hlist links, never the
			 * holder's state word, so a plain clear composes.
			 */
			/*
			 * The holder must be DERIVED, not taken from @parent_nf:
			 * for a NON-head duplicate @parent_nf does not name the
			 * chain's anchor (it can be an entry with no metadata at
			 * all, which is a straight SIGSEGV in
			 * cds_ft_item_to_metadata).  Walk prev to the head's holder
			 * exactly as the interior unchain does -- ft_unchain_node is
			 * called with a NULL holder for this same case and derives
			 * it the same way.  
			 *
			 * @old_node is a PUBLISHED chain member here -- the removed
			 * and never-inserted cases both returned NOT_FOUND above --
			 * so it HAS a holder: ASSERT it rather than proceeding
			 * unlocked.  Under sw an unlocked chain replace is a LOST
			 * UPDATE, not the benign degradation the MW store's
			 * expected-value CAS made it.  Measured unreachable: 0 NULL
			 * in 491532 ft_chain_head_holder calls.
			 */
			if (ft->lock_fine) {
				lock_nf = ft_chain_head_holder(ft, old_node);
				assert(lock_nf);
				hm = ft_flag_to_metadata(ft, lock_nf);
				{
					struct ft_descent hd;
					struct ft_lock_ctx hctx;
					struct ft_held_anchor hh;
					unsigned int hdep = 0;
					bool have_hd = false, descended = false;

					/*
					 * ANCHORED LOCK-SETS need a byte-depth,
					 * and this path derives its holder from a
					 * back-pointer.  Descend for it under the
					 * same opt-in the remove side takes
					 * (§5.3): per-node anchors the holder on
					 * itself and keeps replace handle-derived.
					 */
					if (ft->lock_spacing !=
							CDS_FT_LOCK_SPACING_PER_NODE) {
						const uint8_t *ik = iter_key;

						ft_anchor_descend(ft, &hd,
							iter_key, key_len, &ik);
						descended = true;
						if (hd.nf == lock_nf) {
							hdep = hd.depth;
							have_hd = true;
						} else if (hd.pnf == lock_nf) {
							hdep = hd.pdepth;
							have_hd = true;
						}
					} else
						have_hd = true;
					ft_lock_ctx_init(&hctx,
						descended ? &hd : NULL, NULL, op);
					/*
					 * @hh.shared is DEAD here, not defensive:
					 * this context is built with a NULL txn and
					 * fills in no extra / glue / outer, so its
					 * held set is EMPTY and the acquire has
					 * nothing to dedupe against.  Should this
					 * path ever gain a registry -- a txn, a
					 * glue -- the fold below turns into the
					 * self-refusal livelock it is elsewhere in
					 * this file, and @shared must then become
					 * the "held, owing no release" arm the two
					 * dup-chain acquires above take.
					 */
					if (!have_hd ||
							ft_acquire_member(ft,
								&hctx, lock_nf,
								hm, hdep, &hh) ||
							hh.shared) {
						new_node->next = NULL;
						*need_retry = true;
						s = CDS_FT_STATUS_OK;	/* discarded by the retry loop */
						FT_TP(replace_exit, (int) s);
						return s;
					}
					hm = hh.lock;
					hsnap = hh.lock_snap;
				}
			}
			txn = ft_flip_txn_create_bounded(ft,
					FT_HLIST_REPLACE_MAX_EDGES);
			if (!txn) {
				if (hm)
					ft_meta_lock_release(hm);
				new_node->next = NULL;
				s = CDS_FT_STATUS_MEMORY_ERROR;
				FT_TP(replace_exit, (int) s);
				return s;
			}
			(void) ft_hlist_replace_prepare(ft_flip_txn_handle(txn),
				old_node, new_node);
			cst = ft_flip_txn_commit(ft, txn);
			if (hm)
				ft_meta_lock_release(hm);
			if (cst < 0) {
				new_node->next = NULL;
				s = CDS_FT_STATUS_MEMORY_ERROR;
				FT_TP(replace_exit, (int) s);
				return s;
			}
			if (cst > 0) {
				/*
				 * Contention ABORT: NOTHING was installed.  This
				 * used to fall through as SUCCESS -- the test was
				 * "< 0", which catches only MEMORY_ERROR -- so a
				 * lost replace was silently reported as OK.  It
				 * was unreachable while the op required caller
				 * exclusion; making the op concurrent makes it
				 * live, so it must re-derive and re-attempt.
				 *
				 * BOTH links must be reset, not just next:
				 * ft_hlist_replace_prepare already stored
				 * @new_node->prev = pred as a plain build store
				 * on the (still invisible) fresh node, so leaving
				 * it set would make the next attempt fail entry
				 * validation with INVALID_ARGUMENT_ERROR rather
				 * than retry.
				 */
				new_node->next = NULL;
				new_node->prev = NULL;
				*need_retry = true;
				s = CDS_FT_STATUS_OK;	/* discarded by the retry loop */
				FT_TP(replace_exit, (int) s);
				return s;
			}
		} else if (old_cell) {
			/*
			 * Head, list on: fresh-cell swap fused with the publish.
			 * The successor's prev is a LIVE settled store (a skip
			 * resolution reads a head's prev raw, so it cannot ride the
			 * flip), so make the flip infallible by pre-reserving its
			 * txn BEFORE that store; on OOM abort with the successor
			 * untouched and the replace retriable.
			 */
			struct ft_ord_cell *new_cell =
				ft_ord_cell_ptr(new_cell_flag);
			struct ft_flip_txn *txn =
				ft_flip_txn_create_bounded(ft,
					FT_ORD_CELL_SWAP_PUBLISH_MAX_EDGES +
					FT_HLIST_FREEZE_MAX_EDGES +
					1 /* §4.B parent guard */);

			if (!txn) {
				new_node->next = NULL;
				ft_ord_cell_free_unpublished(ft, new_cell);
				s = CDS_FT_STATUS_MEMORY_ERROR;
				FT_TP(replace_exit, (int) s);
				return s;
			}
			if (new_node->next)
				new_node->next->prev = new_node;
			cds_ft_item_to_metadata(new_cell)->incoming_byte =
				cds_ft_item_to_metadata(old_cell)->incoming_byte;
			/* Fresh @new_node -> cell: build-invisible. */
			new_node->prev = new_cell_flag;
			/* VALIDATE (§4.B): guard the LIVE holder parent_nf. */
			ft_flip_txn_guard_parent(ft, txn, parent_nf);
			_ft_publish_to_parent(ft, parent_nf, pub_slot,
				(struct cds_ft_inode_flag *) new_node,
				(struct cds_ft_inode_flag *) old_node, &rec);
			n_s = ft_pub_rec_sedges(&rec, sedges);
			/*
			 * Fuse @old_node's freeze (mark old_node->next, target
			 * preserved) into the swap commit (doc §4.B); the reservation
			 * above carries the extra edge.
			 */
			ft_hlist_freeze_prepare(ft_flip_txn_handle(txn), old_node);
			ft_replace_fault_arm_abort(txn);
			r = ft_ord_cell_swap_publish_multi(ft, old_cell, new_cell,
				sedges, n_s, txn);
			if (caa_unlikely(r)) {
				/*
				 * ABORT: nothing installed, @old_node and @old_cell
				 * still LIVE.  Dropping this status freed @old_cell --
				 * a cell still linked in the ordered list -- and
				 * reported the replace as OK.  The non-head arm above
				 * already had this exact fix; the head arms did not,
				 * even though ft_ord_cell_flip_into's contract says a
				 * retry-enabled op MUST propagate ABORT, and this op IS
				 * retry-enabled (cds_ft_replace's need_retry loop).
				 *
				 * The successor's prev is the one LIVE store this arm
				 * makes BEFORE the flip (it cannot ride the commit -- a
				 * skip resolution reads a head's prev raw), so it is
				 * also the one thing the unwind must put back: point it
				 * at @old_node again, which is what it held.  Then reset
				 * BOTH of @new_node's links, or the next attempt fails
				 * entry validation with INVALID_ARGUMENT_ERROR instead
				 * of retrying.
				 */
				if (new_node->next)
					new_node->next->prev = old_node;
				new_node->next = NULL;
				new_node->prev = NULL;
				ft_ord_cell_free_unpublished(ft, new_cell);
				if (r == -EAGAIN) {
					*need_retry = true;
					s = CDS_FT_STATUS_OK;	/* discarded by the retry loop */
				} else {
					s = CDS_FT_STATUS_MEMORY_ERROR;
				}
				FT_TP(replace_exit, (int) s);
				return s;
			}
			ft_ord_cell_free(ft, old_cell);
		} else {
			/*
			 * Head, list off: no cell; flip the structural + SKIP_X
			 * dual.  The successor's prev back-edge is a LIVE settled
			 * store (uniform pre-reserve: no flip proxy on any prev
			 * slot, matching the list-on head), so make the flip
			 * infallible by PRE-RESERVING its bounded txn BEFORE that
			 * store; on OOM abort with the successor untouched and the
			 * replace retriable (@new_node restored to its fresh state).
			 */
			struct ft_flip_txn *txn =
				ft_flip_txn_create_bounded(ft, FT_PUB_SEDGE_MAX_EDGES +
					FT_HLIST_FREEZE_MAX_EDGES +
					1 /* §4.B parent guard */);

			if (!txn) {
				new_node->next = NULL;
				s = CDS_FT_STATUS_MEMORY_ERROR;
				FT_TP(replace_exit, (int) s);
				return s;
			}
			if (new_node->next)
				new_node->next->prev = new_node;
			new_node->prev = old_node->prev;
			/* VALIDATE (§4.B): guard the LIVE holder parent_nf. */
			ft_flip_txn_guard_parent(ft, txn, parent_nf);
			_ft_publish_to_parent(ft, parent_nf, pub_slot,
				(struct cds_ft_inode_flag *) new_node,
				(struct cds_ft_inode_flag *) old_node, &rec);
			n_s = ft_pub_rec_sedges(&rec, sedges);
			/* Fuse @old_node's freeze into the structural publish (doc §4.B). */
			ft_hlist_freeze_prepare(ft_flip_txn_handle(txn), old_node);
			ft_replace_fault_arm_abort(txn);
			if (caa_unlikely(ft_ord_cell_flip_into(ft, txn, sedges, n_s)
					!= URCU_TXN_STATUS_OK)) {
				/*
				 * ABORT: nothing installed.  Same unwind as the list-on
				 * head arm -- restore the successor's prev (the one live
				 * pre-flip store) and reset both of @new_node's links --
				 * minus the cell.
				 */
				if (new_node->next)
					new_node->next->prev = old_node;
				new_node->next = NULL;
				new_node->prev = NULL;
				*need_retry = true;
				s = CDS_FT_STATUS_OK;	/* discarded by the retry loop */
				FT_TP(replace_exit, (int) s);
				return s;
			}
		}
	}

	/*
	 * @old_node has left the trie (replaced by @new_node), frozen atomically
	 * inside its own commit above (the non-head swap via ft_hlist_replace_
	 * prepare, each head case via ft_hlist_freeze_prepare fused with the
	 * structural publish) -- no separate tombstone pass.
	 *
	 * The trie structure is unchanged (no recompaction), so the iterator
	 * path remains valid in cached mode.
	 */
	iter_auto_invalidate_cache(iter);
	s = CDS_FT_STATUS_OK;
	FT_TP(replace_exit, (int) s);
	return s;
}

/*
 * Public entry: FT-owned per-op read-side bracket + retry identity, mirroring
 * cds_ft_remove -- its structural twin (both take the target node explicitly and
 * derive position from node->prev with NO re-descent).  The two were asymmetric:
 * remove absorbed contention internally while replace had no retry path at all,
 * so a caller could not write the same loop around both.
 *
 * RETRY: an attempt that loses the chain's holder lock, or whose commit returns
 * ABORT, publishes NOTHING and signals @need_retry; the loop re-derives from
 * node->prev against the current tree and re-attempts.  Aging is carried on the
 * PERSISTENT @optxn via urcu_txn_conflict: after URCU_TXN_FALLBACK conflicts the
 * domain escalates this writer into the per-trie FIFO fair-mutex lane, which
 * drains the contention so the retry TERMINATES (no livelock).  An exclusive
 * trie opens nothing and never conflicts.
 *
 * Every retrying path resets @new_node->next before returning, because the entry
 * validation rejects a @new_node whose links are non-NULL -- without that reset
 * the second attempt would fail with INVALID_ARGUMENT_ERROR instead of retrying.
 */
enum cds_ft_status cds_ft_replace(struct cds_ft *ft,
		struct cds_ft_iter *iter,
		struct cds_ft_node *old_node,
		struct cds_ft_node *new_node)
{
	struct urcu_txn optxn;
	enum cds_ft_status s;
	bool need_retry;

	CDS_FT_SCOPED_WRITER(ft);
	ft_txn_op_init(ft, &optxn);
	for (;;) {
		need_retry = false;
		urcu_txn_begin(&optxn);
		s = _cds_ft_replace_locked(ft, iter, old_node, new_node,
				&need_retry, &optxn);
		if (!need_retry)
			break;
		/* Age the conflict, keep the FIFO turn, close the attempt. */
		ft_txn_attempt_bail(&optxn, true);
	}
	urcu_txn_end(&optxn);
	return s;
}
