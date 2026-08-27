// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

#ifndef _FT_TXN_HLIST_H
#define _FT_TXN_HLIST_H

/*
 * ft-txn-hlist: the fractal trie's duplicate chain, expressed as a small set of
 * TRANSACTIONAL link primitives on the RCU MCAS engine.  It is an FT-PRIVATE
 * counterpart to the generic concurrent <urcu/rcu-txn-hlist.h>, WITHOUT that
 * header's multi-writer arbitration: every chain mutation runs under the
 * head-holder's lock (MW LOCK_FINE Step A -- one writer per chain), so the
 * neighbour-mid-deletion load-validate and the marked-target -ENOENT/-EAGAIN
 * bails in insert_after/del/replace are dead and gone.  What remains from that
 * header is the forward slot as the sole serializer and the "next"-only mark --
 * the mark stays because it is BOTH the reader-visible logical-delete (readers
 * mask it) AND the freeze half the head ops fold into their structural flip-txn
 * (ft_hlist_freeze_prepare).  The MCAS engine (proxy on bit 0) also stays: the
 * chain edits still FOLD into the host op's flip-txn so a chain edit and the
 * trie<->head anchor edge commit atomically.  This does NOT reshape the chain
 * into the kernel hlist `**pprev' encoding -- it keeps FT's duplicate chain
 * EXACTLY as it is and only gives its link maintenance a clean abstraction.
 *
 * Representation (struct cds_ft_node, unchanged, see <urcu/fractal-trie.h>):
 *   next : cds_ft_node *   forward link; reader-visible; MARK-able
 *                          (CDS_FT_NODE_REMOVED_FLAG on bit 1); the engine proxy
 *                          rides bit 0.  Transacted under FT_HLIST_TAG.
 *   prev : cds_ft_node *   back link to the PREDECESSOR NODE (an interior
 *                          duplicate's predecessor).  Transacted too (the
 *                          coherent-both-directions edge); under the single
 *                          writer a del's re-read of the predecessor is stable.
 *
 * Why prev-as-node, not pprev (the FT-specific reason a generic reuse fails):
 * FT overloads cds_ft_node.prev.  For a chain HEAD it is the cell/parent (the
 * holder route, dereferenced as prev->parent); for a duplicate it is the
 * predecessor node -- and headness is decided by ft_node_external(prev)
 * (a duplicate's prev is an external node; a head's is not).  The kernel hlist's
 * pprev would make a duplicate's prev a raw slot address (&pred->next), which is
 * not an external node, so it would break headness detection and the holder
 * route across ~20 readers.  Keeping prev a node preserves all of that untouched;
 * the primitives just compute the forward slot as &pred->next when they need it.
 *
 * Head boundary (the "Option B" cut).  These ops own strictly the INTERIOR of a
 * chain (the head node's `next' inward, H -> D1 -> ... -> tail); the trie<->head
 * anchor slot (external_nodes) is a STRUCTURAL trie slot (FT_FLIP_PROXY_TAG, FT
 * descent resolver, relocated by holder recompaction, flipped by
 * external-promote), managed by FT structural code -- never by these ops.  A
 * duplicate is inserted after the head node H (on H->next), never "at head", and
 * the head is removed structurally, so these ops never touch external_nodes.
 * The one interop invariant the structural head-remove must uphold is
 * MARK(H->next) in its own commit, so a concurrent insert_after(H) onto a
 * sole-node chain sees the mark and aborts -- the same guard interior del/insert
 * already give one another.
 *
 * Bits: a live "next" carries the deletion MARK (bit 1) and the engine proxy TAG
 * (bit 0); a transacted "prev" carries only the proxy (never marked).
 * cds_ft_node is naturally (pointer) aligned, so (value & TAG) != TAG for every
 * live next/prev/NULL.  prev is read by FT's writer-side headness/holder logic
 * with a plain (unresolved) load: correct under the current retained caller
 * exclusion (no proxy at rest); those reads gain a resolve when concurrent
 * writers are enabled.
 *
 * Only the composable *_prepare primitives are provided (insert-after / del /
 * replace): FT folds each into the surrounding op's MCAS txn -- as the
 * ordered-cell list folds its splices via urcu_txn_list_*_prepare -- so it never
 * needs a self-contained _rcu bracket, and it never head-inserts a duplicate.
 */

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <urcu/compiler.h>
#include <urcu/uatomic.h>
#include <urcu/rcu-txn-mcas.h>
#include <urcu/rcu-txn.h>
#include <urcu-pointer.h>

#include <urcu/fractal-trie.h>	/* struct cds_ft_node, CDS_FT_NODE_REMOVED_FLAG */

/*
 * Engine proxy tag for every interior chain slot (a node next/prev).  The
 * interior chain holds only duplicate leaves (never the structural children
 * that carry FT_FLIP_PROXY_TAG), so the plain bit-0 engine tag suffices; it
 * coexists in one TU with the ordered-cell list's URCU_TXN_TAG (the slots never
 * overlap) and with the trie's FT_FLIP_PROXY_TAG structural edges.
 */
#define FT_HLIST_TAG	URCU_TXN_TAG

/* Logical-deletion mark: the public cds_ft_node removal tombstone (bit 1). */
#define FT_HLIST_MARK	CDS_FT_NODE_REMOVED_FLAG

/*
 * Worst-case MCAS edge counts, for the caller's txn reservation.  A tail append
 * (succ == NULL) records only the single pos->next edge; a mid-chain insert also
 * records the succ->next load-validate guard and the succ->prev back-edge.  del
 * and replace touch elem->next (mark), pred->next, next->prev plus the
 * next->next guard.  A freeze records only the elem->next mark (one edge), folded
 * into a host op's structural flip-txn (a chain head leaves through its
 * FT-structural anchor, not a predecessor->next store).
 */
#define FT_HLIST_INSERT_AFTER_MAX_EDGES	3
#define FT_HLIST_DEL_MAX_EDGES		4
#define FT_HLIST_REPLACE_MAX_EDGES	4
#define FT_HLIST_FREEZE_MAX_EDGES	1

/*
 * Every chain edge below goes through here, for one reason beyond the store:
 * these records carry no back-pointer to the flip-txn they fold into, so the
 * per-creation-site table in ft-txn-kind-stats.h cannot attribute them.  This
 * is where they are counted instead -- as one global class, which is all they
 * need to be: a chain edge is MW ON PURPOSE (the chain is not covered by the
 * structural node locks) and is not part of the conservative-MW conversion.
 */
static inline
int ft_hlist_store_mw(struct urcu_txn *txn, void **slot, void *old_ptr,
		void *new_ptr, uintptr_t tag)
{
	FT_TK_COUNT_CELL_MW();
	FT_AB_ARM(FT_AB_CELL_HANDLE, FT_AB_OWN_NA);
	return urcu_txn_store_mw(txn, slot, old_ptr, new_ptr, tag);
}

static inline
void *ft_hlist_set_mark(struct cds_ft_node *n)
{
	return (void *) ((uintptr_t) n | FT_HLIST_MARK);
}

static inline
struct cds_ft_node *ft_hlist_unmark(void *v)
{
	return (struct cds_ft_node *) ((uintptr_t) v & ~(uintptr_t) FT_HLIST_MARK);
}

/*
 * Resolve a raw "next" slot value: strip the engine proxy, then the mark.  Fast
 * path -- a clean value (neither a proxy under FT_HLIST_TAG nor MARK-ed) is
 * returned untouched, so a live-node traversal never runs the unmark AND and the
 * pointer stays out of the load-to-use dependency chain.  Only a tagged value
 * (an in-flight proxy, or a ghost's marked "next") takes the slow path.  prev is
 * writer-only, so it has no reader-side resolve.
 */
static inline
struct cds_ft_node *ft_hlist_resolve(void *raw)
{
	uintptr_t v = (uintptr_t) raw;

	if (caa_unlikely(v & (FT_HLIST_TAG | FT_HLIST_MARK)))
		return ft_hlist_unmark(urcu_txn_resolve(raw, FT_HLIST_TAG));
	return (struct cds_ft_node *) raw;
}

/* Resolved forward step (call within an RCU read-side section). */
static inline
struct cds_ft_node *ft_hlist_next_rcu(struct cds_ft_node *node)
{
	return ft_hlist_resolve((void *) rcu_dereference(node->next));
}

/*
 * ft_hlist_insert_after_prepare: record an insert of @newp immediately after
 * @pos, WITHOUT committing.  @pos is the predecessor NODE (the head node H for a
 * first duplicate, or an interior duplicate); the forward slot &pos->next
 * transitions its current successor @succ -> @newp.  @newp is built invisibly
 * (next = @succ, prev = @pos), and the two edges pos->next: succ -> newp and
 * succ->prev: pos -> newp are recorded.  A tail append (@pos == the walked tail,
 * @succ == NULL) is a 1-edge insert with no backward fixup -- FT's ft_chain_node
 * idiom.  OOM is sticky to the commit.
 *
 * Single-writer per chain (MW LOCK_FINE Step A: every chain mutation runs under
 * the head-holder's node lock -- or, before the FT-wide lock drops, that lock;
 * a disjoint-key optimistic writer owns its own chain), so @pos is never
 * concurrently deleted and @succ is never a neighbour mid-deletion.  The
 * multi-writer arbitration those cases needed -- bail -ENOENT on a marked @pos,
 * load-validate &succ->next and retry -EAGAIN on a marked neighbour -- is dead
 * and dropped.  Always returns 0; the int return is retained for caller-shape
 * parity with the concurrent front-ends (mirrors urcu_txn_sw_list_*_prepare).
 */
static inline
int ft_hlist_insert_after_prepare(struct urcu_txn *txn,
		struct cds_ft_node *newp,
		struct cds_ft_node *pos)
{
	struct cds_ft_node *succ = (struct cds_ft_node *)
			urcu_txn_load(txn, (void **) &pos->next, FT_HLIST_TAG);

	/* Build the fresh node invisibly. */
	newp->next = succ;
	newp->prev = pos;

	/* pos->next: succ -> newp ; succ->prev: pos -> newp. */
	ft_hlist_store_mw(txn, (void **) &pos->next, succ, newp, FT_HLIST_TAG);
	if (succ != NULL)
		ft_hlist_store_mw(txn, (void **) &succ->prev, pos, newp, FT_HLIST_TAG);
	return 0;
}

/*
 * ft_hlist_append_run_prepare: record the append of a whole null-terminated RUN
 * at a chain @tail (tail->next == NULL, as walked by the caller), WITHOUT
 * committing.  Unlike ft_hlist_insert_after_prepare this does NOT touch
 * @run_head->next, so @run_head's own chain (run_head->next -> ...) rides along
 * unmodified -- a run-splice, not a single insert (a merge concatenating a src
 * duplicate run onto a dst tail).  The forward link tail->next: NULL -> run_head
 * is the lone recorded edge and the serializing one (CAS old = NULL: a
 * concurrent freeze of the tail fails this commit), so only the tail carries a
 * proxy while the commit is in flight; the interior of neither run is disturbed.
 *
 * ★ @run_head->prev = tail is a PLAIN store, and the caller owns undoing it.
 * The claim that used to stand here -- "writer-only; readers never read prev,
 * and @run_head is unreachable to readers because a merge detaches and drains
 * the src side before appending it" -- describes ft_merge_spine_copy, the only
 * caller when it was written.  It is FALSE for the one-decide fold, which
 * records the src detach and this splice into ONE txn: the src side is still
 * LIVE here.  And prev IS read -- under SKIP_COMPRESSED a parent slot encodes
 * the skip onto the head itself and ft_skip_to_compressed recovers the
 * compressed node through prev.
 *
 * So this store is reader-visible and, unlike the recorded edge beside it,
 * survives an aborted commit.  The caller must be able to put it back:
 * ft_glue_record_splices snapshots the old value unconditionally and
 * ft_glue_abort restores it.  Do not re-derive "prev is writer-only" here, and
 * do not make that undo conditional on which caller you think can still abort
 * -- ft_merge_spine_copy's own commit (ft-merge.h, after the record) can return
 * non-OK too, and the next bail added anywhere after a record must be covered
 * by default rather than by a decision frozen at the call site.
 */
static inline
void ft_hlist_append_run_prepare(struct urcu_txn *txn,
		struct cds_ft_node *tail,
		struct cds_ft_node *run_head)
{
	int ret;

	run_head->prev = tail;		/* writer-only plain store */
	ret = ft_hlist_store_mw(txn, (void **) &tail->next, NULL, run_head,
			FT_HLIST_TAG);
	assert(!ret);			/* caller reserved the edge up front */
	(void) ret;
}

/*
 * ft_hlist_del_prepare: record the unlink of @elem into @txn WITHOUT committing.
 * Marks @elem (logical delete: elem->next: next -> MARK(next), the target
 * preserved so a reader parked on @elem still follows the chain to the
 * successor / end), unlinks it forward (pred->next: elem -> next) and backward
 * (next->prev: elem -> pred).  On a committed OK THIS call removed @elem; reclaim
 * it after a grace period.  OOM is sticky to the commit.
 *
 * Single-writer per chain (see ft_hlist_insert_after_prepare): @elem is never
 * already deleted and @next is never a neighbour mid-deletion, so the
 * multi-writer arbitration (-ENOENT on a marked @elem, load-validate &next->next
 * and retry -EAGAIN on a marked successor) is dead and dropped, and @pred read
 * this attempt is stable (no peer re-links it).  Always returns 0 (int retained
 * for caller-shape parity).
 */
static inline
int ft_hlist_del_prepare(struct urcu_txn *txn, struct cds_ft_node *elem)
{
	struct cds_ft_node *next = (struct cds_ft_node *)
			urcu_txn_load(txn, (void **) &elem->next, FT_HLIST_TAG);
	struct cds_ft_node *pred = (struct cds_ft_node *)
			urcu_txn_load(txn, (void **) &elem->prev, FT_HLIST_TAG);

	/*
	 * Mark elem (logical delete), unlink forward (pred->next: elem -> next)
	 * and backward (next->prev: elem -> pred).  When next is NULL the backward
	 * edge vanishes: a 2-edge delete storing MARK(NULL).  The mark on
	 * &elem->next is retained -- readers mask it (ft_hlist_resolve) and the
	 * head ops fold it (ft_hlist_freeze_prepare) for atomicity with the
	 * structural anchor edge.
	 */
	ft_hlist_store_mw(txn, (void **) &elem->next, next,
			ft_hlist_set_mark(next), FT_HLIST_TAG);
	ft_hlist_store_mw(txn, (void **) &pred->next, elem, next, FT_HLIST_TAG);
	if (next != NULL)
		ft_hlist_store_mw(txn, (void **) &next->prev, elem, pred, FT_HLIST_TAG);
	return 0;
}

/*
 * ft_hlist_replace_prepare: record the in-place replacement of @old by @newp
 * into @txn WITHOUT committing.  @newp takes @old's position -- pred->next and
 * next->prev swing to @newp -- while @old is logically removed (its next is
 * marked exactly as del does).  Argument order is (old, new).  On commit reclaim
 * @old after a grace period.  OOM is sticky to the commit.  Used for a non-head
 * duplicate replace (a head replace is FT-structural: it swaps the anchor slot).
 *
 * Single-writer per chain (see ft_hlist_insert_after_prepare): the multi-writer
 * arbitration (-ENOENT on a marked @old, load-validate &next->next and retry
 * -EAGAIN on a marked successor) is dead and dropped.  Always returns 0 (int
 * retained for caller-shape parity).
 */
static inline
int ft_hlist_replace_prepare(struct urcu_txn *txn,
		struct cds_ft_node *old, struct cds_ft_node *newp)
{
	struct cds_ft_node *next = (struct cds_ft_node *)
			urcu_txn_load(txn, (void **) &old->next, FT_HLIST_TAG);
	struct cds_ft_node *pred = (struct cds_ft_node *)
			urcu_txn_load(txn, (void **) &old->prev, FT_HLIST_TAG);

	/* Build @newp's links invisibly, then swing pred->next and next->prev. */
	newp->next = next;
	newp->prev = pred;

	ft_hlist_store_mw(txn, (void **) &old->next, next,
			ft_hlist_set_mark(next), FT_HLIST_TAG);
	ft_hlist_store_mw(txn, (void **) &pred->next, old, newp, FT_HLIST_TAG);
	if (next != NULL)
		ft_hlist_store_mw(txn, (void **) &next->prev, old, newp, FT_HLIST_TAG);
	return 0;
}

/*
 * ft_hlist_freeze_prepare: record ONLY the logical-deletion mark of @node's
 * forward slot (node->next: succ -> MARK(succ)) into @txn WITHOUT committing --
 * the freeze half of a del with no chain unlink.  A chain HEAD leaves the trie
 * through its FT-structural anchor edge (the parent slot re-point / clear), not a
 * predecessor->next store, so only the mark rides the hlist; folding it into the
 * head op's structural flip-txn makes the freeze and the anchor edge commit
 * atomically -- a reader is never shown @node's head anchor promoted away while
 * @node is still unmarked.  The target is preserved (MARK(succ), or MARK(NULL)
 * for a head with no successor) so a reader parked on @node still follows the
 * chain to the promoted new head / end.  One recorded edge; the caller reserves
 * FT_HLIST_FREEZE_MAX_EDGES on top of the host op's footprint.
 */
static inline
void ft_hlist_freeze_prepare(struct urcu_txn *txn, struct cds_ft_node *node)
{
	void *en = urcu_txn_load(txn, (void **) &node->next, FT_HLIST_TAG);
	int ret;

	ret = ft_hlist_store_mw(txn, (void **) &node->next, en,
			ft_hlist_set_mark((struct cds_ft_node *) en), FT_HLIST_TAG);
	assert(!ret);			/* caller reserved the edge up front */
	(void) ret;
}

#endif	/* _FT_TXN_HLIST_H */
