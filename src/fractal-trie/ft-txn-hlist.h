// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

#ifndef _FT_TXN_HLIST_H
#define _FT_TXN_HLIST_H

/*
 * ft-txn-hlist: the fractal trie's duplicate chain, expressed as a small set of
 * TRANSACTIONAL link primitives on the RCU MCAS engine.  It is an FT-PRIVATE
 * adaptation of the generic concurrent <urcu/rcu-txn-hlist.h>: the serialization
 * arguments (the forward slot is the sole serializer; a "next"-only mark; the
 * neighbour-mid-deletion load-validate) are that header's, transferred verbatim.
 * What differs is that this does NOT reshape the chain into the kernel hlist
 * `**pprev' encoding -- it keeps FT's duplicate chain EXACTLY as it is and only
 * gives its link maintenance a clean abstraction.
 *
 * Representation (struct cds_ft_node, unchanged, see <urcu/fractal-trie.h>):
 *   next : cds_ft_node *   forward link; reader-visible; MARK-able
 *                          (CDS_FT_NODE_REMOVED_FLAG on bit 1); the engine proxy
 *                          rides bit 0.  Transacted under FT_HLIST_TAG.
 *   prev : cds_ft_node *   back link to the PREDECESSOR NODE (an interior
 *                          duplicate's predecessor).  Transacted too, so a del's
 *                          re-read of the predecessor stays coherent under
 *                          concurrent writers.
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
#include <urcu/rcu-mcas.h>
#include <urcu/rcu-txn.h>
#include <urcu-pointer.h>

#include <urcu/fractal-trie.h>	/* struct cds_ft_node, CDS_FT_NODE_REMOVED_FLAG */

/*
 * Engine proxy tag for every interior chain slot (a node next/prev).  The
 * interior chain holds only duplicate leaves (never the structural children
 * that carry FT_FLIP_PROXY_TAG), so the plain bit-0 engine tag suffices; it
 * coexists in one TU with the ordered-cell list's URCU_MCAS_TAG (the slots never
 * overlap) and with the trie's FT_FLIP_PROXY_TAG structural edges.
 */
#define FT_HLIST_TAG	URCU_MCAS_TAG

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

static inline
void *ft_hlist_set_mark(struct cds_ft_node *n)
{
	return (void *) ((uintptr_t) n | FT_HLIST_MARK);
}

static inline
int ft_hlist_is_marked(void *v)
{
	return (int) ((uintptr_t) v & FT_HLIST_MARK);
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
		return ft_hlist_unmark(urcu_mcas_resolve(raw, FT_HLIST_TAG));
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
 * transitions its current successor @succ -> @newp and is the serializing edge:
 * any concurrent insert/delete that rewrites &pos->next fails this commit's
 * old-value check.  @newp is built invisibly (next = @succ, prev = @pos).
 * Returns 0, -ENOENT if @pos was deleted, or -EAGAIN if @succ is a neighbour
 * mid-deletion (retry); OOM is sticky to the commit.
 *
 * A tail append (@pos == the walked tail, @succ == NULL) is a 1-edge insert with
 * no backward fixup -- FT's ft_chain_node idiom, now guarded (CAS old = NULL).
 */
static inline
int ft_hlist_insert_after_prepare(struct urcu_mcas_txn *txn,
		struct cds_ft_node *newp,
		struct cds_ft_node *pos)
{
	void *pn = urcu_txn_load(txn, (void **) &pos->next, FT_HLIST_TAG);
	struct cds_ft_node *succ;

	if (ft_hlist_is_marked(pn))
		return -ENOENT;			/* @pos was deleted */
	succ = (struct cds_ft_node *) pn;
	/*
	 * We write &succ->prev but NOT &succ->next, so the slot-sorted install
	 * may reach &succ->prev before it reaches &pos->next -- driving the prev
	 * store against a @succ a concurrent del(succ) is freeing.  Load-validate
	 * succ->next (the slot del(succ) marks) into the write-set so the prev
	 * side serializes against del(succ) exactly as &pos->next does; a marked
	 * @succ aborts here.  (When @succ is NULL there is no backward edge and no
	 * such window -- a 1-edge insert.)
	 */
	if (succ != NULL &&
			ft_hlist_is_marked(urcu_txn_load_validate(txn,
				(void **) &succ->next, FT_HLIST_TAG)))
		return -EAGAIN;			/* succ (a neighbour) deleted: retry */

	/* Build the fresh node invisibly. */
	newp->next = succ;
	newp->prev = pos;

	/* pos->next: succ -> newp ; succ->prev: pos -> newp. */
	urcu_txn_store(txn, (void **) &pos->next, succ, newp, FT_HLIST_TAG);
	if (succ != NULL)
		urcu_txn_store(txn, (void **) &succ->prev, pos, newp, FT_HLIST_TAG);
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
 * @run_head->prev = tail is a writer-only plain store -- readers never read prev,
 * and @run_head is caller-guaranteed unreachable to readers here (a merge
 * detaches + drains the src side before appending it).
 */
static inline
void ft_hlist_append_run_prepare(struct urcu_mcas_txn *txn,
		struct cds_ft_node *tail,
		struct cds_ft_node *run_head)
{
	int ret;

	run_head->prev = tail;		/* writer-only plain store */
	ret = urcu_txn_store(txn, (void **) &tail->next, NULL, run_head,
			FT_HLIST_TAG);
	assert(!ret);			/* caller reserved the edge up front */
	(void) ret;
}

/*
 * ft_hlist_del_prepare: record the unlink of @elem into @txn WITHOUT committing.
 * @elem's predecessor is @elem->prev; the forward slot &pred->next transitions
 * @elem -> @next and is the serializing edge.  Returns 0 if recorded (on a
 * committed OK, THIS call removed @elem; reclaim it after a grace period),
 * -ENOENT if @elem was already deleted by a peer (nothing recorded; do NOT
 * reclaim), or -EAGAIN if the successor is mid-deletion (retry).  OOM is sticky
 * to the commit.
 */
static inline
int ft_hlist_del_prepare(struct urcu_mcas_txn *txn, struct cds_ft_node *elem)
{
	void *en = urcu_txn_load(txn, (void **) &elem->next, FT_HLIST_TAG);
	struct cds_ft_node *next, *pred;

	if (ft_hlist_is_marked(en))
		return -ENOENT;			/* already deleted by a peer */
	next = (struct cds_ft_node *) en;
	/*
	 * Read the predecessor fresh this attempt.  If a peer changed which node
	 * precedes @elem (e.g. del of @elem's predecessor rewrote elem->prev to
	 * pred's own predecessor), this @pred is stale -- but the &pred->next store
	 * below then fails its old-value check (the stale slot no longer holds
	 * @elem) and the commit aborts, so a retry re-reads the fresh predecessor.
	 */
	pred = (struct cds_ft_node *)
			urcu_txn_load(txn, (void **) &elem->prev, FT_HLIST_TAG);

	/*
	 * We rewrite &next->prev but not &next->next.  Load-validate next->next
	 * (the slot del(next) marks) so the backward unlink serializes against
	 * del(next) even when the slot-sorted install reaches &next->prev first.
	 * A marked successor is itself being deleted: retry.
	 */
	if (next != NULL &&
			ft_hlist_is_marked(urcu_txn_load_validate(
				txn, (void **) &next->next, FT_HLIST_TAG)))
		return -EAGAIN;			/* successor (a neighbour) deleted: retry */

	/*
	 * Mark elem (logical delete), unlink forward (pred->next: elem -> next),
	 * and unlink backward (next->prev: elem -> pred).  pred->next (old value
	 * elem) is the slot a racing insert-after(pred) / del(pred) shares with us,
	 * so the MCAS serializes every adjacency; marking &elem->next is what makes
	 * a racing insert_after(elem) / del(elem) terminate with -ENOENT.  When
	 * next is NULL the backward edge vanishes: a 2-edge delete storing
	 * MARK(NULL).
	 */
	urcu_txn_store(txn, (void **) &elem->next, next,
			ft_hlist_set_mark(next), FT_HLIST_TAG);
	urcu_txn_store(txn, (void **) &pred->next, elem, next, FT_HLIST_TAG);
	if (next != NULL)
		urcu_txn_store(txn, (void **) &next->prev, elem, pred, FT_HLIST_TAG);
	return 0;
}

/*
 * ft_hlist_replace_prepare: record the in-place replacement of @old by @newp
 * into @txn WITHOUT committing.  @newp takes @old's position -- pred->next and
 * next->prev swing to @newp -- while @old is logically removed (its next is
 * marked exactly as del does).  Touches the SAME slots as del_prepare (only the
 * new values differ), so it inherits del's serialization.  Argument order is
 * (old, new).  Returns 0 if recorded (reclaim @old after a grace period on
 * commit), -ENOENT if @old was already deleted, or -EAGAIN if the successor is
 * mid-deletion.  OOM is sticky to the commit.  Used for a non-head duplicate
 * replace (a head replace is FT-structural: it swaps the anchor slot).
 */
static inline
int ft_hlist_replace_prepare(struct urcu_mcas_txn *txn,
		struct cds_ft_node *old, struct cds_ft_node *newp)
{
	void *en = urcu_txn_load(txn, (void **) &old->next, FT_HLIST_TAG);
	struct cds_ft_node *next, *pred;

	if (ft_hlist_is_marked(en))
		return -ENOENT;			/* @old already deleted/replaced */
	next = (struct cds_ft_node *) en;
	pred = (struct cds_ft_node *)
			urcu_txn_load(txn, (void **) &old->prev, FT_HLIST_TAG);

	if (next != NULL &&
			ft_hlist_is_marked(urcu_txn_load_validate(
				txn, (void **) &next->next, FT_HLIST_TAG)))
		return -EAGAIN;			/* successor (a neighbour) deleted: retry */

	/* Build @newp's links invisibly, then swing pred->next and next->prev. */
	newp->next = next;
	newp->prev = pred;

	urcu_txn_store(txn, (void **) &old->next, next,
			ft_hlist_set_mark(next), FT_HLIST_TAG);
	urcu_txn_store(txn, (void **) &pred->next, old, newp, FT_HLIST_TAG);
	if (next != NULL)
		urcu_txn_store(txn, (void **) &next->prev, old, newp, FT_HLIST_TAG);
	return 0;
}

/*
 * ft_hlist_freeze_prepare: record ONLY the logical-deletion mark of @node's
 * forward slot (node->next: succ -> MARK(succ)) into @txn WITHOUT committing --
 * the freeze half of a del with no chain unlink.  A chain HEAD leaves the trie
 * through its FT-structural anchor edge (the parent slot re-point / clear), not a
 * predecessor->next store, so only the mark rides the hlist; folding it into the
 * head op's structural flip-txn makes the freeze and the anchor edge commit
 * atomically -- once concurrent, a racing insert_after(node) / del(node) shares
 * &node->next and terminates with -ENOENT.  The target is preserved (MARK(succ),
 * or MARK(NULL) for a head with no successor) so a reader parked on @node still
 * follows the chain to the promoted new head / end.  One recorded edge; the caller
 * reserves FT_HLIST_FREEZE_MAX_EDGES on top of the host op's footprint.
 */
static inline
void ft_hlist_freeze_prepare(struct urcu_mcas_txn *txn, struct cds_ft_node *node)
{
	void *en = urcu_txn_load(txn, (void **) &node->next, FT_HLIST_TAG);
	int ret;

	ret = urcu_txn_store(txn, (void **) &node->next, en,
			ft_hlist_set_mark((struct cds_ft_node *) en), FT_HLIST_TAG);
	assert(!ret);			/* caller reserved the edge up front */
	(void) ret;
}

#endif	/* _FT_TXN_HLIST_H */
