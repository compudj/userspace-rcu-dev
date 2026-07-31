// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

#ifndef _URCU_RCU_TXN_SW_HLIST_H
#define _URCU_RCU_TXN_SW_HLIST_H

/*
 * rcu-txn-sw-hlist: a kernel-hlist-shaped, single-pointer-head RCU list under a
 * SINGLE updater (writers mutually excluded, as with cds_hlist_*_rcu).  It is
 * the single-updater sibling of the concurrent-writer <urcu/rcu-txn-hlist.h>
 * and the hash-bucket sibling of the circular bidir <urcu/rcu-txn-sw-list.h>:
 * the head is a SINGLE 8-byte pointer, so a table of buckets is half the
 * footprint of the sentinel-node heads.  See design/rcu-txn-hlist.md.
 *
 * pprev encoding, forward-only readers
 * ------------------------------------
 * A node's backward link is a pointer to the SLOT that names it (&prev->next or
 * &head->first), never a node pointer -- so the head's single pointer is just
 * another "next" slot and no operation carries a head special case (see the
 * concurrent header for the full rationale).
 *
 *   Node: { struct urcu_txn_sw_hlist_node *next;    (node ptr; reader-visible)
 *           struct urcu_txn_sw_hlist_node **pprev;  (slot ptr; writer-only) }
 *
 * Readers walk FORWARD ONLY, so each structural op has exactly ONE
 * reader-visible edge -- the "next" slot it re-points (&head->first for a
 * first-position op, &prev->next otherwise).  That edge, and ONLY that edge, is
 * recorded into the flip transaction (<urcu/rcu-txn-sw.h>).
 *
 * pprev is written by a PLAIN STORE, eagerly, inside the _prepare form
 * ------------------------------------------------------------------
 * pprev is writer-only: no reader ever loads it (the walk is forward-only) and
 * a single updater has no concurrent writer, so it needs neither atomicity, nor
 * release ordering, nor a flip proxy.  Recording it would buy nothing and cost
 * a second edge on every op.  Note this is where the hlist parts ways with the
 * bidir <urcu/rcu-txn-sw-list.h>, whose readers "may iterate in either
 * direction": THAT structure's prev is reader-visible and must stay transacted.
 * The resemblance of the two backward links is superficial.
 *
 * The store is EAGER -- at _prepare time, not after commit -- and that is what
 * keeps same-bucket composition correct.  A later _prepare in the same bracket
 * reads its neighbour's pprev RAW and sees the earlier _prepare's value, because
 * under a single updater a plain store is immediately visible to that same
 * updater.  Deferring the pprev write and then reading it raw is the bug: a
 * later edit would read a STALE slot address, one naming a node the earlier edit
 * already unlinked, and re-point that dead slot instead of the live one.  See
 * urcu_txn_sw_hlist_del_prepare() for the worked adjacent-delete case.
 *
 * Invariant, for the general argument rather than one example: raw pprev here
 * equals what pprev's PENDING value would be were it transacted.  It holds at
 * bracket entry (both are the committed value) and each _prepare computes the
 * same value from the same inputs and makes it visible to the next op, so it is
 * preserved; and pprev influences committed state only by naming the slot a
 * forward-edge record targets, which is exactly what the invariant covers.
 *
 * The one thing this gives up, and the reserve() obligation it creates
 * ------------------------------------------------------------------
 * A plain store does not roll back.  A composed bracket that OOMs mid-way would
 * commit nothing yet leave pprev advanced -- writer bookkeeping permanently
 * skewed, which a LATER del would turn into a corrupt reader-visible chain.
 *
 * Each _prepare closes the FIRST-op case itself: it records before it stores and
 * returns early if that record fails, so an op that cannot be published stores
 * nothing.  A lone-op bracket is therefore safe with no reserve at all -- which
 * is every _rcu wrapper below, and any hand-rolled init/prepare/commit of one
 * edit.
 *
 * What no _prepare can close is a LATER op failing behind an earlier op's store.
 * So a bracket composing more than one op MUST urcu_txn_sw_reserve() its edge
 * bound up front, before the first _prepare.  A successful reserve makes every
 * later record() append without reallocating and pre-allocates the group block
 * install() would otherwise take lazily, so commit() cannot then report
 * MEMORY_ERROR -- no failure path is left behind any store.  Checking only
 * commit()'s status, the engine's general model, is NOT enough here.  (Bound the
 * reserve by op count: one transacted edge per op is the maximum, and adjacent
 * ops chain onto one record, so the bound is loose.)
 *
 * URCU_TXN_SW_HLIST__ASSERT_ROLLBACKABLE, on every _prepare, traps a violation
 * under DEBUG_RCU rather than leaving it to surface as corruption later.  It
 * asserts the underlying invariant -- spare capacity behind an eager store --
 * not the reserve() call itself; see it for why those differ.
 *
 * The single-op _rcu brackets below owe nothing: each records exactly ONE edge,
 * which the engine commits with a lone release store -- no proxy, no group
 * block, no allocation at all (they drive an on-stack urcu_txn_sw_init_inline
 * handle), and so no failure path to protect.
 *
 * Composition itself covers cross-structure edits (the intended use, disjoint by
 * construction) AND edits on ONE hlist whose neighbourhoods touch: each _prepare
 * reads the reader-visible "next"/first slots through the engine's
 * read-your-own-writes load (urcu_txn_sw_hlist_pending_next) and chains a
 * same-slot record rather than duplicating it, so adjacent deletes and the like
 * commit correctly.
 *
 * TWO obligations are left to the caller, not one.
 *
 * (1) Name the nodes before editing them, not by traversing mid-bracket.
 *
 * (2) EVERY ANCHOR MUST STILL BE LIVE AS THIS TRANSACTION LEAVES THE LIST.  An
 * op anchored on a node an EARLIER op of the same bracket deleted is a ghost
 * anchor, and there is nothing here to detect it: unlike the concurrent
 * sibling, a single-updater delete leaves no deletion mark, so a prepare cannot
 * tell a deleted anchor from a live one.  Naming the node up front satisfies
 * obligation (1) and says nothing about this.
 *
 * 1 -> 2 -> 3, one bracket: del_prepare(2) then add_after_prepare(9, 2).  The
 * delete records {&1->next: 2 -> 3} -- it never touches &2->next -- so the add
 * reads a pending_next of 3 off the ghost and records {&2->next: 3 -> 9}, a
 * fresh record on a slot inside it, and overwrites the delete's 3->pprev with
 * &9->next.  Pairwise-distinct slots, so install's duplicate scan passes under
 * DEBUG_RCU, no record_chain old assert is reached, EXCL_VALIDATE sees one
 * thread, and the commit reports OK.  The committed chain is 1 -> 3; node 9 is
 * reachable only through the ghost, and 3->pprev names a slot no live node
 * holds.  A LATER del_rcu(3) then follows that pprev, matches its old, commits
 * into the unreachable node, and returns success -- while 1->next still names
 * 3, which the caller frees.  Every reader walking 1 -> 3 then touches freed
 * memory.
 *
 * The add_before and replace variants chain onto the delete's record instead
 * and trip record_chain's pending-old assert under DEBUG_RCU; under NDEBUG they
 * resurrect the deleted node into the committed chain.  Only the add_after form
 * above is caught by nothing at all.
 *
 * Configurable proxy tag (a compile-time define, never stored in the head)
 * ----------------------------------------------------------------------
 * Every slot of the hlist is transacted under URCU_TXN_SW_HLIST_TAG, the flip
 * proxy tag (<urcu/rcu-txn-sw.h>).  It is a compile-time define (default
 * bit 0) rather than a per-call argument, so the head costs no
 * extra storage and call sites stay kernel-terse, and rather than a hard-coded
 * constant so an embedder whose head lives in a slot it already transacts under
 * its OWN tag (e.g. the fractal trie's low-nibble child-slot tag) can
 *     #define URCU_TXN_SW_HLIST_TAG   FT_SLOT_TAG
 * before including this header.  Every function is static inline, so TUs picking
 * different tags coexist with no ODR clash.  URCU_TXN_SW_HLIST_TAG must satisfy
 * the engine's per-record contract ((value & TAG) != TAG for every live value a
 * slot holds).
 *
 * Because there is a single updater there is NO logical-deletion mark (nothing
 * concurrent to detect): del simply re-points the naming slot.  A removed node's
 * own next/pprev are left intact (ghost), so a reader standing on it still
 * escapes forward into the live chain; the caller reclaims it after a grace
 * period, exactly as cds_hlist_del_rcu().
 *
 * Read side
 * ---------
 * Iterate under rcu_read_lock() through the resolving accessors
 * (urcu_txn_sw_hlist_first_rcu / _next_rcu) or the macros below -- never touch
 * node->next directly (a transacted slot may transiently hold a tagged proxy).
 * pprev is writer-only and needs no reader accessor.
 *
 * Write side
 * ----------
 * Writers must be mutually excluded.  Each mutator drives a urcu_txn_sw_txn: it
 * records its ONE reader-visible next edge, plain-stores pprev, and commits --
 * for a single op the commit is one release store, with no proxy parked and so
 * no grace period owed.  Include this header AFTER an RCU flavor header.
 *
 * A mutator returns 0 or -1 to match the concurrent <urcu/rcu-txn-hlist.h>, but
 * the single-op forms below allocate nothing and cannot fail.  A caller
 * composing several ops into one bracket takes on the reserve()-up-front
 * obligation described above.
 */

#include <stdlib.h>
#include <stdint.h>

#include <urcu/compiler.h>
#include <urcu/uatomic.h>
#include <urcu/call-rcu.h>		/* struct rcu_head */
#include <urcu/rcu-txn-sw.h>
#include <urcu-pointer.h>		/* rcu_dereference / rcu_assign_pointer */

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Flip proxy tag for every hlist slot (head-first and node next/pprev).
 * Override before include to drive the chain under an embedder's own tag.  Must
 * satisfy (value & TAG) != TAG for every live value any slot holds.
 */
#ifndef URCU_TXN_SW_HLIST_TAG
#define URCU_TXN_SW_HLIST_TAG	1UL
#endif

struct urcu_txn_sw_hlist_node {
	struct urcu_txn_sw_hlist_node *next;	/* node ptr; reader-visible */
	struct urcu_txn_sw_hlist_node **pprev;	/* slot ptr; writer-only */
};

/* Single 8-byte bucket head, matching struct hlist_head. */
struct urcu_txn_sw_hlist_head {
	struct urcu_txn_sw_hlist_node *first;
};

#define URCU_TXN_SW_HLIST_HEAD_INIT	{ .first = NULL }

static inline
void urcu_txn_sw_hlist_init(struct urcu_txn_sw_hlist_head *head)
{
	head->first = NULL;
}

/*
 * Resolve a "next"/head-first slot value to the node it currently denotes: a
 * tagged proxy (all of URCU_TXN_SW_HLIST_TAG's bits set -- the sw engine parks
 * &proxy | tag) resolves through the flip selector, a direct node pointer passes
 * through unchanged.
 */
static inline
struct urcu_txn_sw_hlist_node *urcu_txn_sw_hlist_resolve(
		struct urcu_txn_sw_hlist_node *ptr)
{
	return (struct urcu_txn_sw_hlist_node *)
			urcu_txn_sw_resolve(ptr, URCU_TXN_SW_HLIST_TAG);
}

/* Resolved bucket-first / forward step (call under rcu_read_lock()). */
static inline
struct urcu_txn_sw_hlist_node *urcu_txn_sw_hlist_first_rcu(
		struct urcu_txn_sw_hlist_head *head)
{
	return urcu_txn_sw_hlist_resolve(rcu_dereference(head->first));
}

static inline
struct urcu_txn_sw_hlist_node *urcu_txn_sw_hlist_next_rcu(
		struct urcu_txn_sw_hlist_node *node)
{
	return urcu_txn_sw_hlist_resolve(rcu_dereference(node->next));
}

static inline
int urcu_txn_sw_hlist_empty(struct urcu_txn_sw_hlist_head *head)
{
	return urcu_txn_sw_hlist_first_rcu(head) == NULL;
}

/*
 * WRITE-SIDE read of a reader-visible "next"/first slot: its value as @txn will
 * leave it -- this transaction's pending write to the slot if it has recorded
 * one, else the slot's committed value.  A typed wrapper over the engine's
 * read-your-own-writes load (<urcu/rcu-txn-sw.h>).  The _prepare forms below
 * read every forward neighbour link through this rather than touching ->next /
 * *slot directly -- that is what lets edits COMPOSE on one bucket (see
 * urcu_txn_sw_hlist_del_prepare()).  Writer side only -- a reader wants
 * urcu_txn_sw_hlist_first_rcu() / _next_rcu(), which resolve against the flip
 * selector and know nothing of a transaction's pending state.
 *
 * The load returns the committed value when the slot is unrecorded and the
 * pending value once it is, which is exactly what record_chain() wants as the
 * @old_ptr in either case: a fresh record's committed old, or the pending value
 * it asserts against on a chain.
 *
 * There is deliberately no _pending_pprev counterpart: pprev is written by an
 * eager plain store (see the preamble), so a RAW read of it is already this
 * transaction's pending value.
 */
static inline
struct urcu_txn_sw_hlist_node *urcu_txn_sw_hlist_pending_next(
		struct urcu_txn_sw_txn *txn,
		struct urcu_txn_sw_hlist_node **slot)
{
	return (struct urcu_txn_sw_hlist_node *) urcu_txn_sw_load(txn,
			(void **) slot, URCU_TXN_SW_HLIST_TAG);
}

/*
 * Debug guard on the eager-pprev contract, asserted at the top of every
 * _prepare form.
 *
 * The rule it enforces: an op may plain-store pprev only while the whole
 * bracket is still rollbackable -- that is, while no LATER record in it can
 * fail behind the store.  Two ways that holds.  Either nothing has been
 * recorded yet (nr == 0), so this op's own record is the first and it either
 * succeeds or stores nothing (each _prepare below returns early on a failed
 * record, which is what makes a lone-op bracket safe with no reserve at all);
 * or the handle already owns spare capacity, so this append cannot fail.
 *
 * Note what this does NOT check: whether reserve() was called.  That would be
 * the wrong question -- record() grows on demand to URCU_TXN_SW_CAP, so the
 * first several ops of an unreserved bracket are just as infallible as a
 * reserved one, and a reserve() too small for the bracket is just as unsafe.
 * Capacity is the invariant; reserve() up front is simply the reliable way for
 * a caller to guarantee it, and the only way once a bracket outgrows
 * URCU_TXN_SW_CAP.  Firing here means: reserve your edge bound before the first
 * _prepare, or the bracket's pprev bookkeeping can be left skewed by an OOM
 * that publishes nothing.
 *
 * Debug-only: urcu_assert_debug compiles out under NDEBUG, and the predicate is
 * two loads off the handle.
 */
#define URCU_TXN_SW_HLIST__ASSERT_ROLLBACKABLE(txn)			\
	urcu_assert_debug((txn)->nr == 0				\
			|| urcu_txn_sw_append_is_infallible(txn))

/*
 * urcu_txn_sw_hlist_insert_at_slot_prepare: the core composable primitive.
 * Record the edges that make @slot name @newp, given @slot will hold @succ as
 * this transaction leaves it, WITHOUT committing.  @slot is &head->first for
 * insert-at-head or &pos->next for insert-after.  Records the reader-visible
 * *slot edge and, when @succ is non-NULL, the writer-only &succ->pprev edge.
 * Always returns 0 (single updater); the int return matches the concurrent
 * variant for transition parity.
 *
 * @succ must be the value @slot will hold as this transaction leaves it, not a
 * raw read: @newp is built pointing at it (newp->next = succ), so a stale @succ
 * links @newp behind a node an earlier edit of the SAME bracket already
 * displaced.  Read it with urcu_txn_sw_hlist_pending_next() when composing on
 * one bucket -- the add_head / add_after wrappers below do.  With that, edits
 * compose on one hlist; see urcu_txn_sw_hlist_del_prepare() for the worked trap.
 */
static inline
int urcu_txn_sw_hlist_insert_at_slot_prepare(struct urcu_txn_sw_txn *txn,
		struct urcu_txn_sw_hlist_node *newp,
		struct urcu_txn_sw_hlist_node **slot,
		struct urcu_txn_sw_hlist_node *succ)
{
	URCU_TXN_SW_HLIST__ASSERT_ROLLBACKABLE(txn);

	/* Build the fresh node's links before it becomes reachable. */
	newp->next = succ;
	newp->pprev = slot;

	/* Reader-visible edge, transacted: *slot: succ -> newp. */
	if (!urcu_txn_sw_record_chain(txn, (void **) slot, succ, newp,
			URCU_TXN_SW_HLIST_TAG))
		return 0;			/* OOM: store nothing (see above) */
	/* Writer-only bookkeeping, eager plain store: succ is now named by &newp->next. */
	if (succ != NULL)
		succ->pprev = &newp->next;
	return 0;
}

/*
 * urcu_txn_sw_hlist_add_head_prepare: composable form of insert-at-head (see
 * insert_at_slot_prepare for the contract).  Always returns 0.
 */
static inline
int urcu_txn_sw_hlist_add_head_prepare(struct urcu_txn_sw_txn *txn,
		struct urcu_txn_sw_hlist_node *newp,
		struct urcu_txn_sw_hlist_head *head)
{
	return urcu_txn_sw_hlist_insert_at_slot_prepare(txn, newp,
			&head->first,
			urcu_txn_sw_hlist_pending_next(txn, &head->first));
}

/*
 * urcu_txn_sw_hlist_add_after_prepare: composable form of insert-after @pos.
 * Always returns 0.
 */
static inline
int urcu_txn_sw_hlist_add_after_prepare(struct urcu_txn_sw_txn *txn,
		struct urcu_txn_sw_hlist_node *newp,
		struct urcu_txn_sw_hlist_node *pos)
{
	return urcu_txn_sw_hlist_insert_at_slot_prepare(txn, newp,
			&pos->next,
			urcu_txn_sw_hlist_pending_next(txn, &pos->next));
}

/*
 * urcu_txn_sw_hlist_add_before_prepare: composable form of insert-before @pos
 * (kernel hlist_add_before).  @pos->pprev names the slot to re-point.  Always
 * returns 0.
 */
static inline
int urcu_txn_sw_hlist_add_before_prepare(struct urcu_txn_sw_txn *txn,
		struct urcu_txn_sw_hlist_node *newp,
		struct urcu_txn_sw_hlist_node *pos)
{
	struct urcu_txn_sw_hlist_node **slot = pos->pprev;	/* raw: eager-stored, already pending */

	URCU_TXN_SW_HLIST__ASSERT_ROLLBACKABLE(txn);

	newp->next = pos;
	newp->pprev = slot;

	/* Reader-visible edge, transacted: *slot: pos -> newp. */
	if (!urcu_txn_sw_record_chain(txn, (void **) slot, pos, newp,
			URCU_TXN_SW_HLIST_TAG))
		return 0;			/* OOM: store nothing (see above) */
	pos->pprev = &newp->next;		/* writer-only, eager plain store */
	return 0;
}

/*
 * urcu_txn_sw_hlist_del_prepare: composable form of del.  @elem between slot
 * *elem->pprev and next.  @elem's own next/pprev are left intact (ghost) so a
 * reader standing on it still escapes forward; the caller frees @elem after a
 * grace period (post-commit).  Always returns 0.
 *
 * Composition on one bucket, worked through -- delete adjacent A and B from
 * head -> A -> B -> C in one bracket.  del(A) records {*head->first: A -> B},
 * then plain-stores B->pprev = &head->first.  del(B) reads B->pprev RAW and gets
 * &head->first -- the value that store just left there, NOT the stale &A->next
 * -- so it records against &head->first, finds it already recorded, and CHAINS:
 * {*head->first: A -> C}; then plain-stores C->pprev = &head->first.  Result:
 * head->first == C, C->pprev == &head->first; A and B both unlinked, in ONE
 * recorded edge.
 *
 * What makes the raw read safe is that the pprev store is EAGER.  Defer it to
 * after the commit and del(B) would read the stale &A->next -- a slot inside the
 * node del(A) just unlinked -- and re-point that dead slot, leaving head->first
 * naming the deleted B.  Eager plain store and transacted pprev agree on every
 * committed outcome; they part only on the OOM path, which is what the
 * reserve()-up-front contract in the preamble exists to close.
 */
static inline
int urcu_txn_sw_hlist_del_prepare(struct urcu_txn_sw_txn *txn,
		struct urcu_txn_sw_hlist_node *elem)
{
	struct urcu_txn_sw_hlist_node *next =
			urcu_txn_sw_hlist_pending_next(txn, &elem->next);
	struct urcu_txn_sw_hlist_node **ppv = elem->pprev;	/* raw: eager-stored, already pending */

	URCU_TXN_SW_HLIST__ASSERT_ROLLBACKABLE(txn);

	/* Reader-visible edge, transacted: *ppv: elem -> next. */
	if (!urcu_txn_sw_record_chain(txn, (void **) ppv, elem, next,
			URCU_TXN_SW_HLIST_TAG))
		return 0;			/* OOM: store nothing (see above) */
	/* Writer-only bookkeeping, eager plain store: next inherits elem's slot. */
	if (next != NULL)
		next->pprev = ppv;
	return 0;
}

/*
 * urcu_txn_sw_hlist_replace_prepare: composable form of replace.  @newp inherits
 * @old's slot and successor; @old is left ghost for parked readers.  Argument
 * order is (old, new), as cds_list_replace_rcu().  Always returns 0.
 */
static inline
int urcu_txn_sw_hlist_replace_prepare(struct urcu_txn_sw_txn *txn,
		struct urcu_txn_sw_hlist_node *old,
		struct urcu_txn_sw_hlist_node *newp)
{
	struct urcu_txn_sw_hlist_node *next =
			urcu_txn_sw_hlist_pending_next(txn, &old->next);
	struct urcu_txn_sw_hlist_node **ppv = old->pprev;	/* raw: eager-stored, already pending */

	URCU_TXN_SW_HLIST__ASSERT_ROLLBACKABLE(txn);

	newp->next = next;
	newp->pprev = ppv;

	/* Reader-visible edge, transacted: *ppv: old -> newp. */
	if (!urcu_txn_sw_record_chain(txn, (void **) ppv, old, newp,
			URCU_TXN_SW_HLIST_TAG))
		return 0;			/* OOM: store nothing (see above) */
	/* Writer-only bookkeeping, eager plain store: next is now named by &newp->next. */
	if (next != NULL)
		next->pprev = &newp->next;
	return 0;
}

/*
 * Convenience brackets: each records its op and commits it.
 *
 * Every op records EXACTLY ONE edge (the reader-visible one; pprev is a plain
 * store), which the engine commits with a lone release store -- no proxy is
 * parked, no group block is allocated, and no grace period is owed.  So these
 * drive a caller-storage handle over a one-latch on-stack buffer
 * (urcu_txn_sw_init_inline, blessed for exactly this lone-edge use): the whole
 * bracket is allocation-free, and a single hlist mutation costs about what a
 * bare rcu_assign_pointer does.
 *
 * With no allocation there is no failure path: commit cannot report
 * MEMORY_ERROR and these cannot return -1.  The int return is kept for source
 * compatibility and for parity with the concurrent <urcu/rcu-txn-hlist.h>,
 * whose same-named forms CAN fail.
 */
static inline
int urcu_txn_sw_hlist_add_head_rcu(struct urcu_txn_sw_hlist_node *newp,
		struct urcu_txn_sw_hlist_head *head)
{
	struct urcu_txn_sw_latch buf[1];
	struct urcu_txn_sw_txn txn;

	urcu_txn_sw_init_inline(&txn, buf, 1);
	(void) urcu_txn_sw_hlist_add_head_prepare(&txn, newp, head);
	return urcu_txn_sw_commit(&txn) < 0 ? -1 : 0;
}

static inline
int urcu_txn_sw_hlist_add_after_rcu(struct urcu_txn_sw_hlist_node *newp,
		struct urcu_txn_sw_hlist_node *pos)
{
	struct urcu_txn_sw_latch buf[1];
	struct urcu_txn_sw_txn txn;

	urcu_txn_sw_init_inline(&txn, buf, 1);
	(void) urcu_txn_sw_hlist_add_after_prepare(&txn, newp, pos);
	return urcu_txn_sw_commit(&txn) < 0 ? -1 : 0;
}

static inline
int urcu_txn_sw_hlist_add_before_rcu(struct urcu_txn_sw_hlist_node *newp,
		struct urcu_txn_sw_hlist_node *pos)
{
	struct urcu_txn_sw_latch buf[1];
	struct urcu_txn_sw_txn txn;

	urcu_txn_sw_init_inline(&txn, buf, 1);
	(void) urcu_txn_sw_hlist_add_before_prepare(&txn, newp, pos);
	return urcu_txn_sw_commit(&txn) < 0 ? -1 : 0;
}

static inline
int urcu_txn_sw_hlist_del_rcu(struct urcu_txn_sw_hlist_node *elem)
{
	struct urcu_txn_sw_latch buf[1];
	struct urcu_txn_sw_txn txn;

	urcu_txn_sw_init_inline(&txn, buf, 1);
	(void) urcu_txn_sw_hlist_del_prepare(&txn, elem);
	return urcu_txn_sw_commit(&txn) < 0 ? -1 : 0;
}

static inline
int urcu_txn_sw_hlist_replace_rcu(struct urcu_txn_sw_hlist_node *old,
		struct urcu_txn_sw_hlist_node *newp)
{
	struct urcu_txn_sw_latch buf[1];
	struct urcu_txn_sw_txn txn;

	urcu_txn_sw_init_inline(&txn, buf, 1);
	(void) urcu_txn_sw_hlist_replace_prepare(&txn, old, newp);
	return urcu_txn_sw_commit(&txn) < 0 ? -1 : 0;
}

#define urcu_txn_sw_hlist_entry(ptr, type, member) \
	caa_container_of(ptr, type, member)

#define urcu_txn_sw_hlist_entry_safe(ptr, type, member) \
	__extension__ ({ __typeof__(ptr) ___ptr = (ptr); \
		___ptr ? urcu_txn_sw_hlist_entry(___ptr, type, member) : NULL; })

/* Iterate forward over a bucket (under rcu_read_lock()). */
#define urcu_txn_sw_hlist_for_each_rcu(pos, head) \
	for (pos = urcu_txn_sw_hlist_first_rcu(head); \
		(pos) != NULL; \
		pos = urcu_txn_sw_hlist_next_rcu(pos))

#define urcu_txn_sw_hlist_for_each_entry_rcu(pos, head, member) \
	for (pos = urcu_txn_sw_hlist_entry_safe( \
			urcu_txn_sw_hlist_first_rcu(head), \
			__typeof__(*(pos)), member); \
		(pos) != NULL; \
		pos = urcu_txn_sw_hlist_entry_safe( \
			urcu_txn_sw_hlist_next_rcu(&(pos)->member), \
			__typeof__(*(pos)), member))

#ifdef __cplusplus
}
#endif

#endif	/* _URCU_RCU_TXN_SW_HLIST_H */
