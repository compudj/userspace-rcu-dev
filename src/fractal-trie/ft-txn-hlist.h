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
 * ☠ A prev WORD IS READ BY READERS, and a record parked on it must not be
 * bit-identical to a value it can legitimately hold.  Every up-walk from an
 * external -- the skip re-anchor, ft_skip_to_compressed, ft_get_parent_rcu --
 * loads prev through ft_dereference_prev_resolved, which resolves the FT
 * flip-proxy tag (low nibble 0xF) and nothing else; and in list mode a HEAD's
 * prev is an ordinal cell pointer tagged FT_ORD_CELL_TAG, which is BIT 0 --
 * the same bit as URCU_TXN_TAG.  A prev record with the hlist tag would
 * therefore read as a cell to a reader hopping a duplicate chain (a member's
 * prev is its predecessor, a raw external pointer; the hop stops at the first
 * non-external value and takes it for the head's word).  So prev-word records
 * carry the flip-proxy tag, which no prev value has (nodes and cells are
 * 16-byte aligned), and the reader's one resolver covers them.  next-word
 * records keep the hlist tag: next is read through ft_hlist_resolve.
 */
#define FT_HLIST_PREV_TAG	FT_FLIP_PROXY_TAG

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
/*
 * The duplicate-chain hold audit lives in ft-mutation-helpers.h (it needs the
 * per-thread hold ledger, which is defined there, and this header is included
 * well before it).  Declare the coarse arm so ft_hlist_store_mw_at can report;
 * one TU, so the later definition resolves it.
 */
#ifdef FT_DEBUG_CHAIN_HOLD
static inline void ft_ch_audit_coarse_at(const char *fn, int line);
# define FT_CH_COARSE(fn, line)	ft_ch_audit_coarse_at((fn), (line))
#else
# define FT_CH_COARSE(fn, line)	do { (void) (fn); (void) (line); } while (0)
#endif

static inline
int ft_hlist_store_mw_at(const char *fn, int line, struct urcu_txn *txn,
		void **slot, void *old_ptr, void *new_ptr, uintptr_t tag)
{
	/*
	 * ☠ THE KIND COUNTER SAYS "CELL" AND THIS IS NOT A CELL.  Every store
	 * below writes a DUPLICATE-CHAIN word (cds_ft_node.next/.prev), which is
	 * [debt] -- a named owner, bound for SW under the nearest ancestor lock.
	 * The ordinal CELL list (ft_ord_cell.lnode) is [DESIGN] MW forever.
	 * Sharing one bucket means no instrument can tell them apart, and any
	 * "MW is correct here" reasoning earned by the cell list reads as though
	 * it covered the chain.  Left as-is for now so the counter's history
	 * stays comparable; the audit below is keyed per SITE precisely so the
	 * two are separable without disturbing it.
	 */
	FT_TK_COUNT_CELL_MW();
	FT_AB_ARM(FT_AB_CELL_HANDLE, FT_AB_OWN_NA);
	/*
	 * @fn/@line are the CALLER's, so every chain-word store gets its own
	 * audit row -- the whole point, since the question is per SITE.
	 */
	FT_CH_COARSE(fn, line);
	return urcu_txn_store_mw(txn, slot, old_ptr, new_ptr, tag);
}

#define ft_hlist_store_mw(txn, slot, old_ptr, new_ptr, tag)		\
	ft_hlist_store_mw_at(__func__, __LINE__, (txn), (slot),		\
		(old_ptr), (new_ptr), (tag))

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

#ifdef FT_DEBUG_MARK_REFUSAL
/*
 * TEMPORARY (validated-set probe): which test rows actually REACH the two
 * FT_HLIST_MARK refusals?  A row with a non-zero count here is a row that must
 * be in the validated set for this site; a row reading 0 proves nothing about
 * the refusal and does not belong in the set.
 */
static unsigned long ft_mark_refuse_ins, ft_mark_refuse_del;
# define FT_MARK_REFUSE_TALLY(which)	do {				\
		if ((which) == 0) __atomic_fetch_add(&ft_mark_refuse_ins,\
				1, __ATOMIC_RELAXED);			\
		else __atomic_fetch_add(&ft_mark_refuse_del, 1,		\
				__ATOMIC_RELAXED);			\
	} while (0)
static void ft_mark_refuse_report(void) __attribute__((destructor));
static void ft_mark_refuse_report(void)
{
	fprintf(stderr, "FT_MARK_REFUSE  insert_after=%lu  del=%lu\n",
		ft_mark_refuse_ins, ft_mark_refuse_del);
}
#else
# define FT_MARK_REFUSE_TALLY(which)	do { } while (0)
#endif

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
 * and dropped.
 *
 * ☠ DEFECT FT-SLOT-2: THAT DROP CONTRADICTS THE INTEROP INVARIANT THIS
 * FILE'S HEADER STATES.
 * The header promises that a structural head-remove's MARK(H->next) is what
 * makes "a concurrent insert_after(H) onto a sole-node chain see the mark and
 * abort" -- and the check below was deleted, so for a while nothing looked at
 * the mark.  The remove side upholds its
 * half (ft_hlist_freeze_sole_prepare marks a derived NULL); the insert side no
 * longer checks, and a MARK(NULL) @pos->next reads back as the bare value 2,
 * which passes `succ != NULL` and makes the second store record slot
 * &((struct cds_ft_node *) 2)->prev.  Under LOCK_FINE, where insert and remove
 * are concurrent on the same key by contract, the append's head is derived
 * before the holder acquire, so this is reachable in principle -- the mirror of
 * the remove-side routing defect.  ☑ IT IS NO LONGER "UNPROVEN BY TEST": the
 * shape is reachable, inv_concurrent_insert_replace_nolist reaches it, and the
 * check is RESTORED below as a refusal.  Returns 0, or -ENOENT when @pos is a
 * retired head; the int return was retained for exactly this.
 */
static inline
int ft_hlist_insert_after_prepare(struct urcu_txn *txn,
		struct cds_ft_node *newp,
		struct cds_ft_node *pos)
{
	struct cds_ft_node *succ = (struct cds_ft_node *)
			urcu_txn_load(txn, (void **) &pos->next, FT_HLIST_TAG);

	/*
	 * ☑ THE MARK CHECK IS BACK, and it is a REFUSAL now, not an assert.
	 *
	 * @pos->next MARKED means @pos is a RETIRED head and this append is
	 * building onto a chain that no longer exists.  Left unchecked it is a
	 * LOST INSERT: MARK(NULL) reads back as the bare value 2, passes
	 * `succ != NULL` below, and makes the second store record slot
	 * &((struct cds_ft_node *) 2)->prev.  This file's header states the
	 * interop invariant in as many words -- the structural head-remove marks
	 * H->next in its own commit "so a concurrent insert_after(H) onto a
	 * sole-node chain sees the mark and ABORTS".  Seeing it is this line.
	 *
	 * ☠ IT WAS UNPROVEN AND IS NOW PROVEN.  The obituary above asked for
	 * "either restoring the check or proving the shape unreachable"; the
	 * shape is reachable, and cds_ft_insert_replace is the producer -- it
	 * displaces a whole chain, and once it TOMBSTONES what it displaces (as
	 * its sibling always did for the single node it displaces) an append
	 * that derived @pos before the displacement lands here every time.
	 * Reached by inv_concurrent_insert_replace_nolist.
	 *
	 * -ENOENT is the caller's documented code for "tail already marked"; it
	 * maps the refusal to -EAGAIN and re-descends from the root, which is
	 * the correct answer -- the chain this append aimed at is gone, so the
	 * position must be derived again.
	 */
	if (caa_unlikely((uintptr_t) succ & FT_HLIST_MARK)) {
		FT_MARK_REFUSE_TALLY(0);
		return -ENOENT;
	}

	/* Build the fresh node invisibly. */
	newp->next = succ;
	newp->prev = pos;

	/* pos->next: succ -> newp ; succ->prev: pos -> newp. */
	ft_hlist_store_mw(txn, (void **) &pos->next, succ, newp, FT_HLIST_TAG);
	if (succ != NULL)
		ft_hlist_store_mw(txn, (void **) &succ->prev, pos, newp, FT_HLIST_PREV_TAG);
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
 * ★ @run_head->prev IS RECORDED BY THE CALLER, not stored here.  It used to
 * be a plain store on the claim "writer-only; readers never read prev, and
 * @run_head is unreachable to readers because a merge detaches and drains the
 * src side before appending it" -- true of ft_merge_spine_copy, the only
 * caller when it was written, FALSE for the one-decide fold, which records
 * the src detach and this splice into ONE txn with the src side still LIVE.
 * And prev IS read: every up-walk from an external starts at it, and a reader
 * on the retired copy of the src junction follows that copy's stale skip word
 * straight to @run_head.  A plain store there was the one torn state the
 * reader's two-descent witness cannot repair -- BOTH descents see the same
 * half-moved head before the flip -- and it survived an abort, which the
 * caller had to undo by hand.  Recorded (FT_HLIST_PREV_TAG, beside the
 * forward link, in ft_glue_record_splices) it flips atomically with the rest
 * of the move and is discarded with an abort; the reservation is two records
 * per splice.
 */
static inline
void ft_hlist_append_run_prepare(struct urcu_txn *txn,
		struct cds_ft_node *tail,
		struct cds_ft_node *run_head)
{
	int ret;

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
 * Single-writer per chain (see ft_hlist_insert_after_prepare) covers the
 * NEIGHBOURS: @next is never a neighbour mid-deletion and @pred read this
 * attempt is stable (no peer re-links it), so that half of the multi-writer
 * arbitration (load-validate &next->next, retry -EAGAIN on a marked successor)
 * stays dropped.
 *
 * ☠ IT DOES NOT COVER @elem ITSELF.  A structural head-remove marks a head it
 * retires from OUTSIDE this chain's holder -- and since cds_ft_insert_replace
 * tombstones the chain it displaces, an already-marked @elem does arrive here.
 * The -ENOENT refusal below is therefore LIVE, not dead: this function returns
 * 0, or -ENOENT when @elem is already logically deleted.
 */
static inline
int ft_hlist_del_prepare(struct urcu_txn *txn, struct cds_ft_node *elem)
{
	struct cds_ft_node *next = (struct cds_ft_node *)
			urcu_txn_load(txn, (void **) &elem->next, FT_HLIST_TAG);
	struct cds_ft_node *pred = (struct cds_ft_node *)
			urcu_txn_load(txn, (void **) &elem->prev, FT_HLIST_PREV_TAG);

	/*
	 * ☠ ALREADY MARKED means @elem is ALREADY logically deleted, and building
	 * this delete on top of it is not merely redundant -- it FAULTS.  A
	 * MARK(NULL) @elem->next reads back as the bare value 2, which passes the
	 * `next != NULL` test below and makes the backward edge record slot
	 * &((struct cds_ft_node *) 2)->prev; the commit then installs into
	 * address 2 and SEGVs.  Byte-for-byte the failure
	 * ft_hlist_insert_after_prepare's mark check exists to stop, one function
	 * over -- and it became reachable here for the same reason: nothing used
	 * to tombstone a chain displaced by cds_ft_insert_replace, so a marked
	 * node could not arrive at a del.  Now one can.
	 *
	 * -ENOENT: the caller's comment already names this case ("@node or a
	 * NEIGHBOUR mid-deletion"), destroys the txn and retries from a fresh
	 * position derivation, where the wrapper's tombstone test answers
	 * NOT_FOUND.  Nothing is recorded, so nothing installs.
	 */
	if (caa_unlikely((uintptr_t) next & FT_HLIST_MARK)) {
		FT_MARK_REFUSE_TALLY(1);
		return -ENOENT;
	}

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
		ft_hlist_store_mw(txn, (void **) &next->prev, elem, pred, FT_HLIST_PREV_TAG);
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
			urcu_txn_load(txn, (void **) &old->prev, FT_HLIST_PREV_TAG);

	/* Build @newp's links invisibly, then swing pred->next and next->prev. */
	newp->next = next;
	newp->prev = pred;

	ft_hlist_store_mw(txn, (void **) &old->next, next,
			ft_hlist_set_mark(next), FT_HLIST_TAG);
	ft_hlist_store_mw(txn, (void **) &pred->next, old, newp, FT_HLIST_TAG);
	if (next != NULL)
		ft_hlist_store_mw(txn, (void **) &next->prev, old, newp, FT_HLIST_PREV_TAG);
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

/*
 * ft_hlist_freeze_sole_prepare: the freeze of a head the caller DERIVED to be
 * its key's SOLE entry -- the key-disappearing lanes (a detach, the fused
 * ft_remove_one_commit), which prune the branch around it.  The expected-old
 * is that derivation, NULL, never the slot re-loaded here: the derivation was
 * made with nothing held, and a same-key cds_ft_insert -- concurrent with a
 * remove in contract under LOCK_FINE -- can have APPENDED a duplicate to
 * @node since.  Re-loading would mark that duplicate into the tombstone
 * (MARK(N)) and the prune would orphan it behind a retired head: an OK insert
 * whose key never resolves again, and a later remove of it that never
 * terminates (measured: "key LOST after an OK concurrent insert", both list
 * modes).  Recorded against NULL, the append fails this commit's install CAS
 * instead -- the record is MW, so a mismatch is an ABORT, which every
 * key-disappearing caller already routes to a re-derivation that then finds
 * the successor and PROMOTES it.
 */
static inline
void ft_hlist_freeze_sole_prepare(struct urcu_txn *txn, struct cds_ft_node *node)
{
	int ret;

	ret = ft_hlist_store_mw(txn, (void **) &node->next, NULL,
			ft_hlist_set_mark(NULL), FT_HLIST_TAG);
	assert(!ret);			/* caller reserved the edge up front */
	(void) ret;
}

#endif	/* _FT_TXN_HLIST_H */
