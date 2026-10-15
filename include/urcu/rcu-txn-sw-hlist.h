// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

#ifndef _URCU_RCU_TXN_SW_HLIST_H
#define _URCU_RCU_TXN_SW_HLIST_H

/*
 * rcu-txn-sw-hlist: an RCU list with a single-pointer head, shaped like the
 * kernel's hlist, for a single writer (writers mutually excluded, as with
 * cds_hlist_*_rcu).  It is the single-writer sibling of the concurrent
 * <urcu/rcu-txn-hlist.h>, and the hash-bucket sibling of the circular
 * <urcu/rcu-txn-sw-list.h>: its head is one pointer, half the footprint of a
 * sentinel-node head.
 *
 * Forward-only readers, and pprev
 * -------------------------------
 * A node's backward link is a pointer to the slot that names it (&prev->next or
 * &head->first), not to a node, so no operation has a head special case.
 *
 *   Node: { struct urcu_txn_sw_hlist_node *next;    (node ptr; reader-visible)
 *           struct urcu_txn_sw_hlist_node **pprev;  (slot ptr; writer-only) }
 *
 * Readers walk forward only and never load pprev, so each operation has exactly
 * one reader-visible edge: the next slot it re-points.  This is where the hlist
 * differs from <urcu/rcu-txn-sw-list.h>, whose prev is reader-visible.
 *
 * Single updates and transactions
 * -------------------------------
 * The _rcu forms make a single update.  Its one edge commits as a lone
 * store-release, on a handle over caller storage: no proxy is installed, no
 * descriptor is allocated, and the update cannot fail.  pprev is then written
 * with a plain store after the commit: the writers are mutually excluded, so no
 * other writer reads it in between.
 *
 * The _prepare forms record into a transaction of the caller's, which may
 * compose several updates, and which may fail or be cancelled.  They transact
 * pprev too, as a slot of its own.  A later _prepare of the same transaction
 * then loads its neighbour's pprev as the transaction leaves it, which is what
 * makes edits of one bucket compose (see urcu_txn_sw_hlist_del_prepare()), and
 * a transaction that fails or is cancelled leaves every pprev as it was.  An
 * update with a successor records two edges, and so takes the descriptor path
 * at commit even alone: a single update is cheapest in its _rcu form.
 *
 * Composing edits of one hlist
 * ----------------------------
 * Operations whose neighbourhoods touch may share a transaction: each _prepare
 * loads the slots it reads, next/first and pprev, as the transaction leaves
 * them, and chains a second write to a slot onto the first.  Two rules remain
 * for the caller:
 *
 *   - Name the nodes before editing, rather than by traversing the list in the
 *     middle of the transaction.
 *
 *   - Every node an operation is anchored on must still be in the list as the
 *     transaction leaves it.  A single-writer delete leaves no deletion mark,
 *     so a _prepare cannot tell a deleted anchor from a live one.
 *
 *     Example: 1 -> 2 -> 3, del_prepare(2) then add_after_prepare(9, 2).  The
 *     delete records {&1->next: 2 -> 3} and {&3->pprev: &2->next -> &1->next}.
 *     The add then records {&2->next: 3 -> 9}, and chains 3's pprev from
 *     &2->next, where the delete left it at &1->next.  A debug build trips
 *     urcu_txn_sw_record_chain()'s assertion there.  An NDEBUG build commits
 *     the chain 1 -> 3: node 9 is reachable only through the removed node 2,
 *     and 3->pprev names a slot no live node holds.  A later del_rcu(3)
 *     follows that pprev, commits into the unreachable node 9 and returns
 *     success, while 1->next still names 3, which the caller then frees.
 *
 *     add_before and replace anchored on a deleted node chain onto the delete's
 *     record of 1->next instead, with the same assertion; an NDEBUG build links
 *     the deleted node, or its replacement, back into the chain.
 *
 * Proxy tag
 * ---------
 * The reader-visible slots, head->first and node->next, are transacted under
 * URCU_TXN_SW_HLIST_TAG (bit 0 by default).  It is a compile-time define, so
 * the head needs no extra storage.  An embedder whose head lives in a slot it
 * already transacts under its own tag (e.g. the fractal trie's child-slot tag)
 * can
 *     #define URCU_TXN_SW_HLIST_TAG   FT_SLOT_TAG
 * before including this header.  Every function is static inline, so
 * translation units choosing different tags can coexist.  No live value a slot
 * holds may carry the tag: (value & TAG) != TAG.
 *
 * pprev is transacted under bit 0, whatever URCU_TXN_SW_HLIST_TAG is: it holds
 * the address of a pointer, in which bit 0 is clear.
 *
 * Read side
 * ---------
 * Iterate within an RCU read-side critical section, through
 * urcu_txn_sw_hlist_first_rcu() / _next_rcu() or the macros below.  Never read
 * node->next directly: a slot may transiently hold a tagged proxy.
 *
 * Write side
 * ----------
 * Writers must be mutually excluded.  A removed node keeps its own next, so a
 * reader standing on it still reaches the live chain; the caller reclaims it
 * after a grace period, as with cds_hlist_del_rcu().  Include this header after
 * an RCU flavor header.
 *
 * The add forms take a node that no reader can reach, and build its next with
 * a plain store, so they cannot move a node.  To move an entry to another
 * bucket, delete its node and add a copy's, in one transaction.
 *
 * Return values
 * -------------
 * Mutators return an int, negative on error, so that code written against the
 * concurrent <urcu/rcu-txn-hlist.h> compiles.  The conventions differ
 * otherwise.  There, del() returns 1 or 0 to say whether this call removed the
 * node, and callers gate reclaim on it; here a delete always removes the node
 * and returns 0.  A del site ported unchanged either stops reclaiming or frees
 * twice.  The error codes differ too, and the concurrent forms take a struct
 * urcu_txn_domain * that these do not.
 */

#include <stdlib.h>
#include <stdint.h>

#include <urcu/compiler.h>
#include <urcu/uatomic.h>
#include <urcu/call-rcu.h>		/* struct rcu_head */
#include <urcu/rcu-txn-sw.h>
#include <urcu-pointer.h>		/* rcu_dereference() */

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Proxy tag of the reader-visible slots, head->first and node->next.  Override
 * it before including this header to use an embedder's own tag.  It must be
 * non-zero and within the low 4 bits, and no live value a slot holds may carry
 * it: (value & TAG) != TAG.
 */
#ifndef URCU_TXN_SW_HLIST_TAG
#define URCU_TXN_SW_HLIST_TAG	1UL
#endif

/*
 * Under a zero tag every plain value would pass for a proxy.  A tag beyond
 * URCU_TXN_SW_TAG_MASK would overlap the address of a record.
 */
urcu_static_assert(URCU_TXN_SW_HLIST_TAG != 0 &&
			(URCU_TXN_SW_HLIST_TAG & ~URCU_TXN_SW_TAG_MASK) == 0,
		"URCU_TXN_SW_HLIST_TAG must be non-zero and within "
		"URCU_TXN_SW_TAG_MASK",
		urcu_txn_sw_hlist_tag_out_of_range);

struct urcu_txn_sw_hlist_node {
	struct urcu_txn_sw_hlist_node *next;	/* node ptr; reader-visible */
	struct urcu_txn_sw_hlist_node **pprev;	/* slot ptr; writer-only */
};

/* A bucket head is one pointer, like struct hlist_head. */
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
 * Resolve a head->first or node->next slot value to the node it currently
 * denotes: a tagged proxy resolves to its old or new target, a node pointer is
 * returned as is.
 */
static inline
struct urcu_txn_sw_hlist_node *urcu_txn_sw_hlist_resolve(
		struct urcu_txn_sw_hlist_node *ptr)
{
	return (struct urcu_txn_sw_hlist_node *)
			urcu_txn_sw_resolve(ptr, URCU_TXN_SW_HLIST_TAG);
}

/* First node and forward step, for use within an RCU read-side section. */
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

/*
 * True iff the chain is empty.  Call within an RCU read-side critical section,
 * like the accessors it is built on.
 */
static inline
int urcu_txn_sw_hlist_empty(struct urcu_txn_sw_hlist_head *head)
{
	return urcu_txn_sw_hlist_first_rcu(head) == NULL;
}

/*
 * Write-side read of a head->first or node->next slot: this transaction's
 * pending write to it if it recorded one, else the slot's committed value.  A
 * typed wrapper over urcu_txn_sw_load().  The _prepare forms read every forward
 * link through this, which is what lets edits compose on one bucket (see
 * urcu_txn_sw_hlist_del_prepare()).  Readers use urcu_txn_sw_hlist_first_rcu()
 * / _next_rcu() instead.
 */
static inline
struct urcu_txn_sw_hlist_node *urcu_txn_sw_hlist_pending_next(
		struct urcu_txn_sw_txn *txn,
		struct urcu_txn_sw_hlist_node **slot)
{
	return (struct urcu_txn_sw_hlist_node *) urcu_txn_sw_load(txn,
			(void **) slot, URCU_TXN_SW_HLIST_TAG);
}

/* Proxy tag of pprev: see "Proxy tag" above. */
#define URCU_TXN_SW_HLIST__PPREV_TAG	1UL

/* The pprev counterpart of urcu_txn_sw_hlist_pending_next(). */
static inline
struct urcu_txn_sw_hlist_node **urcu_txn_sw_hlist__pending_pprev(
		struct urcu_txn_sw_txn *txn,
		struct urcu_txn_sw_hlist_node *node)
{
	return (struct urcu_txn_sw_hlist_node **) urcu_txn_sw_load(txn,
			(void **) &node->pprev, URCU_TXN_SW_HLIST__PPREV_TAG);
}

/*
 * Record that @node, named by @old as the transaction leaves it so far, is now
 * named by @newp.
 */
static inline
void urcu_txn_sw_hlist__record_pprev(struct urcu_txn_sw_txn *txn,
		struct urcu_txn_sw_hlist_node *node,
		struct urcu_txn_sw_hlist_node **old,
		struct urcu_txn_sw_hlist_node **newp)
{
	(void) urcu_txn_sw_record_chain(txn, (void **) &node->pprev, old, newp,
			URCU_TXN_SW_HLIST__PPREV_TAG);
}

/*
 * Record the edge that makes @slot name @newp, into @txn, without committing.
 * @slot is &head->first to insert at the head, or &pos->next to insert after
 * @pos.  When @succ is non-NULL, its pprev becomes &newp->next.  Always
 * returns 0; the int return matches the concurrent variant.
 *
 * @succ must be the value @slot will hold as this transaction leaves it, read
 * with urcu_txn_sw_hlist_pending_next(): @newp is built pointing at it, and a
 * value read directly would link @newp behind a node that an earlier edit of
 * the same transaction displaced.  The add_head and add_after forms below do
 * so.
 */
static inline
int urcu_txn_sw_hlist_insert_at_slot_prepare(struct urcu_txn_sw_txn *txn,
		struct urcu_txn_sw_hlist_node *newp,
		struct urcu_txn_sw_hlist_node **slot,
		struct urcu_txn_sw_hlist_node *succ)
{
	/* Build the fresh node's links before it becomes reachable. */
	newp->next = succ;
	newp->pprev = slot;

	(void) urcu_txn_sw_record_chain(txn, (void **) slot, succ, newp,
			URCU_TXN_SW_HLIST_TAG);
	if (succ != NULL)
		urcu_txn_sw_hlist__record_pprev(txn, succ, slot, &newp->next);
	return 0;
}

/*
 * Record the insertion of @newp at the head of @head into @txn, without
 * committing.  See urcu_txn_sw_hlist_insert_at_slot_prepare().  Always
 * returns 0.
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
 * Record the insertion of @newp after @pos into @txn, without committing.  See
 * urcu_txn_sw_hlist_insert_at_slot_prepare().  Always returns 0.
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
 * Record the insertion of @newp before @pos (the kernel's hlist_add_before)
 * into @txn, without committing.  @pos's pprev names the slot to re-point.
 * Always returns 0.
 */
static inline
int urcu_txn_sw_hlist_add_before_prepare(struct urcu_txn_sw_txn *txn,
		struct urcu_txn_sw_hlist_node *newp,
		struct urcu_txn_sw_hlist_node *pos)
{
	struct urcu_txn_sw_hlist_node **slot =
			urcu_txn_sw_hlist__pending_pprev(txn, pos);

	newp->next = pos;
	newp->pprev = slot;

	(void) urcu_txn_sw_record_chain(txn, (void **) slot, pos, newp,
			URCU_TXN_SW_HLIST_TAG);
	urcu_txn_sw_hlist__record_pprev(txn, pos, slot, &newp->next);
	return 0;
}

/*
 * Record the unlink of @elem into @txn, without committing.  @elem's own
 * next/pprev are left as they are, so a reader standing on it still reaches the
 * live chain; the caller frees @elem a grace period after the commit.  Always
 * returns 0.
 *
 * Composing on one bucket, by example: delete the adjacent A and B from
 * head -> A -> B -> C in one transaction.  del(A) records
 * {&head->first: A -> B} and {&B->pprev: &A->next -> &head->first}.  del(B)
 * loads B's pprev as the transaction leaves it, &head->first, finds that slot
 * recorded, and chains: {&head->first: A -> C}.  It then records
 * {&C->pprev: &B->next -> &head->first}.
 *
 * Had del(B) read B->pprev directly, it would have found &A->next, a slot
 * inside the node that del(A) unlinks, and head->first would be left naming
 * the deleted B.
 */
static inline
int urcu_txn_sw_hlist_del_prepare(struct urcu_txn_sw_txn *txn,
		struct urcu_txn_sw_hlist_node *elem)
{
	struct urcu_txn_sw_hlist_node *next =
			urcu_txn_sw_hlist_pending_next(txn, &elem->next);
	struct urcu_txn_sw_hlist_node **ppv =
			urcu_txn_sw_hlist__pending_pprev(txn, elem);

	(void) urcu_txn_sw_record_chain(txn, (void **) ppv, elem, next,
			URCU_TXN_SW_HLIST_TAG);
	if (next != NULL)
		urcu_txn_sw_hlist__record_pprev(txn, next, &elem->next, ppv);
	return 0;
}

/*
 * Record the replacement of @old by @newp into @txn, without committing.  @newp
 * takes @old's slot and successor, and @old keeps its own next for readers
 * standing on it.  Arguments are (old, new), as cds_list_replace_rcu().  Always
 * returns 0.
 */
static inline
int urcu_txn_sw_hlist_replace_prepare(struct urcu_txn_sw_txn *txn,
		struct urcu_txn_sw_hlist_node *old,
		struct urcu_txn_sw_hlist_node *newp)
{
	struct urcu_txn_sw_hlist_node *next =
			urcu_txn_sw_hlist_pending_next(txn, &old->next);
	struct urcu_txn_sw_hlist_node **ppv =
			urcu_txn_sw_hlist__pending_pprev(txn, old);

	newp->next = next;
	newp->pprev = ppv;

	(void) urcu_txn_sw_record_chain(txn, (void **) ppv, old, newp,
			URCU_TXN_SW_HLIST_TAG);
	if (next != NULL)
		urcu_txn_sw_hlist__record_pprev(txn, next, &old->next,
				&newp->next);
	return 0;
}

/*
 * The _rcu forms: each makes a single update, outside of any transaction of the
 * caller's.
 *
 * The update's one edge commits as a lone store-release, on a handle over one
 * on-stack record (urcu_txn_sw_init_inline()): no proxy is installed, no
 * descriptor is allocated and no grace period is owed.  pprev is then written
 * with a plain store.
 *
 * Without an allocation they cannot fail, and always return 0.  The int return
 * is kept for source compatibility with the concurrent <urcu/rcu-txn-hlist.h>;
 * see "Return values" above.
 */

static inline
int urcu_txn_sw_hlist__insert_at_slot_rcu(struct urcu_txn_sw_hlist_node *newp,
		struct urcu_txn_sw_hlist_node **slot)
{
	struct urcu_txn_sw_hlist_node *succ;
	struct urcu_txn_sw_record rec;
	struct urcu_txn_sw_txn txn;

	urcu_txn_sw_init_inline(&txn, &rec);
	succ = urcu_txn_sw_hlist_pending_next(&txn, slot);
	newp->next = succ;
	newp->pprev = slot;
	(void) urcu_txn_sw_record(&txn, (void **) slot, succ, newp,
			URCU_TXN_SW_HLIST_TAG);
	(void) urcu_txn_sw_commit(&txn);
	if (succ != NULL)
		succ->pprev = &newp->next;
	return 0;
}

static inline
int urcu_txn_sw_hlist_add_head_rcu(struct urcu_txn_sw_hlist_node *newp,
		struct urcu_txn_sw_hlist_head *head)
{
	return urcu_txn_sw_hlist__insert_at_slot_rcu(newp, &head->first);
}

static inline
int urcu_txn_sw_hlist_add_after_rcu(struct urcu_txn_sw_hlist_node *newp,
		struct urcu_txn_sw_hlist_node *pos)
{
	return urcu_txn_sw_hlist__insert_at_slot_rcu(newp, &pos->next);
}

static inline
int urcu_txn_sw_hlist_add_before_rcu(struct urcu_txn_sw_hlist_node *newp,
		struct urcu_txn_sw_hlist_node *pos)
{
	struct urcu_txn_sw_hlist_node **slot = pos->pprev;
	struct urcu_txn_sw_record rec;
	struct urcu_txn_sw_txn txn;

	urcu_txn_sw_init_inline(&txn, &rec);
	newp->next = pos;
	newp->pprev = slot;
	(void) urcu_txn_sw_record(&txn, (void **) slot, pos, newp,
			URCU_TXN_SW_HLIST_TAG);
	(void) urcu_txn_sw_commit(&txn);
	pos->pprev = &newp->next;
	return 0;
}

static inline
int urcu_txn_sw_hlist_del_rcu(struct urcu_txn_sw_hlist_node *elem)
{
	struct urcu_txn_sw_hlist_node **ppv = elem->pprev;
	struct urcu_txn_sw_hlist_node *next;
	struct urcu_txn_sw_record rec;
	struct urcu_txn_sw_txn txn;

	urcu_txn_sw_init_inline(&txn, &rec);
	next = urcu_txn_sw_hlist_pending_next(&txn, &elem->next);
	(void) urcu_txn_sw_record(&txn, (void **) ppv, elem, next,
			URCU_TXN_SW_HLIST_TAG);
	(void) urcu_txn_sw_commit(&txn);
	if (next != NULL)
		next->pprev = ppv;
	return 0;
}

static inline
int urcu_txn_sw_hlist_replace_rcu(struct urcu_txn_sw_hlist_node *old,
		struct urcu_txn_sw_hlist_node *newp)
{
	struct urcu_txn_sw_hlist_node **ppv = old->pprev;
	struct urcu_txn_sw_hlist_node *next;
	struct urcu_txn_sw_record rec;
	struct urcu_txn_sw_txn txn;

	urcu_txn_sw_init_inline(&txn, &rec);
	next = urcu_txn_sw_hlist_pending_next(&txn, &old->next);
	newp->next = next;
	newp->pprev = ppv;
	(void) urcu_txn_sw_record(&txn, (void **) ppv, old, newp,
			URCU_TXN_SW_HLIST_TAG);
	(void) urcu_txn_sw_commit(&txn);
	if (next != NULL)
		next->pprev = &newp->next;
	return 0;
}

#define urcu_txn_sw_hlist_entry(ptr, type, member) \
	caa_container_of(ptr, type, member)

#define urcu_txn_sw_hlist_entry_safe(ptr, type, member) \
	__extension__ ({ __typeof__(ptr) ___ptr = (ptr); \
		___ptr ? urcu_txn_sw_hlist_entry(___ptr, type, member) \
			: NULL; })

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
