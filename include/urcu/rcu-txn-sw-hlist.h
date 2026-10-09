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
 * Readers walk forward only, so each operation has exactly one reader-visible
 * edge: the next slot it re-points.  Only that edge is recorded into the
 * transaction (<urcu/rcu-txn-sw.h>).
 *
 * pprev is writer-only state, and a single writer has no concurrent writer: it
 * needs neither atomicity nor a proxy, and is written by a plain store.  This
 * is where the hlist differs from <urcu/rcu-txn-sw-list.h>, whose prev is
 * reader-visible and transacted.
 *
 * The store happens in the _prepare form, not after the commit.  A later
 * _prepare of the same transaction then reads its neighbour's pprev directly
 * and sees the earlier one's value, which is what makes edits of one bucket
 * compose: see urcu_txn_sw_hlist_del_prepare().
 *
 * Reserve before composing
 * ------------------------
 * A plain store is not rolled back.  If a transaction composing several
 * operations ran out of memory midway, it would commit nothing and leave pprev
 * advanced, and a later delete would then corrupt the reader-visible chain.
 *
 * Each _prepare records before it stores, and stores nothing if the record
 * fails.  A transaction of a single operation is therefore safe without a
 * reserve; this covers every _rcu form below.
 *
 * A transaction composing more than one operation must urcu_txn_sw_reserve()
 * its record bound before the first _prepare, so that no later record can fail
 * behind an earlier store.  One record per operation is enough.  Checking only
 * commit()'s status, as elsewhere, is not enough here.  A debug build asserts
 * it: see URCU_TXN_SW_HLIST__ASSERT_ROLLBACKABLE.
 *
 * Composing edits of one hlist
 * ----------------------------
 * Operations whose neighbourhoods touch may share a transaction: each _prepare
 * reads the next/first slots through urcu_txn_sw_hlist_pending_next() and
 * chains a second write to a slot onto the first.  Two rules remain for the
 * caller:
 *
 *   - Name the nodes before editing, rather than by traversing the list in the
 *     middle of the transaction.
 *
 *   - Every node an operation is anchored on must still be in the list as the
 *     transaction leaves it.  A single-writer delete leaves no deletion mark,
 *     so a _prepare cannot tell a deleted anchor from a live one.
 *
 *     Example: 1 -> 2 -> 3, del_prepare(2) then add_after_prepare(9, 2).  The
 *     delete records {&1->next: 2 -> 3}.  The add then records
 *     {&2->next: 3 -> 9} and sets 3->pprev = &9->next.  The slots are distinct
 *     and the commit reports OK in every build, but the committed chain is
 *     1 -> 3: node 9 is reachable only through the removed node 2, and
 *     3->pprev names a slot no live node holds.  A later del_rcu(3) follows
 *     that pprev, commits into the unreachable node 9 and returns success,
 *     while 1->next still names 3, which the caller then frees.
 *
 *     add_before and replace anchored on a deleted node chain onto the delete's
 *     record instead.  A debug build trips urcu_txn_sw_record_chain()'s
 *     assertion; an NDEBUG build links the deleted node, or its replacement,
 *     back into the chain.
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
 * Under a zero tag every plain value would pass for a proxy.  A tag beyond the
 * low 4 bits would overlap the address of a record, which is only 16-byte
 * aligned.
 */
urcu_static_assert(URCU_TXN_SW_HLIST_TAG != 0 &&
			(URCU_TXN_SW_HLIST_TAG & ~0xFUL) == 0,
		"URCU_TXN_SW_HLIST_TAG must be non-zero and fit the low 4 bits "
		"left free by the 16-byte record alignment",
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
 *
 * pprev needs no counterpart: it is plain-stored at _prepare time, so reading
 * it directly already gives this transaction's pending value.
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
 * Debug check at the top of every _prepare: an operation may plain-store pprev
 * only while no later record of the transaction can fail behind the store.
 * That holds when nothing is recorded yet, since this operation then stores
 * nothing if its own record fails, or when the handle has spare capacity, so
 * that this append cannot fail.
 *
 * It checks capacity, not whether urcu_txn_sw_reserve() was called: an
 * unreserved handle has spare capacity for its first URCU_TXN_SW_CAP records,
 * and a reserve too small for the transaction is no protection.  When it fires,
 * reserve the transaction's record bound before the first _prepare.
 */
#define URCU_TXN_SW_HLIST__ASSERT_ROLLBACKABLE(txn)			\
	urcu_assert_debug((txn)->nr == 0				\
			|| urcu_txn_sw_append_is_infallible(txn))

/*
 * Record the edge that makes @slot name @newp, into @txn, without committing.
 * @slot is &head->first to insert at the head, or &pos->next to insert after
 * @pos.  When @succ is non-NULL, its pprev is set to &newp->next.  Always
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
	URCU_TXN_SW_HLIST__ASSERT_ROLLBACKABLE(txn);

	/* Build the fresh node's links before it becomes reachable. */
	newp->next = succ;
	newp->pprev = slot;

	/* The reader-visible edge: *slot: succ -> newp. */
	if (!urcu_txn_sw_record_chain(txn, (void **) slot, succ, newp,
			URCU_TXN_SW_HLIST_TAG))
		return 0;		/* OOM: store nothing */
	/* Writer-only: succ is now named by &newp->next. */
	if (succ != NULL)
		succ->pprev = &newp->next;
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
 * into @txn, without committing.  @pos->pprev names the slot to re-point.
 * Always returns 0.
 */
static inline
int urcu_txn_sw_hlist_add_before_prepare(struct urcu_txn_sw_txn *txn,
		struct urcu_txn_sw_hlist_node *newp,
		struct urcu_txn_sw_hlist_node *pos)
{
	/* pprev is plain-stored at _prepare time: read it directly. */
	struct urcu_txn_sw_hlist_node **slot = pos->pprev;

	URCU_TXN_SW_HLIST__ASSERT_ROLLBACKABLE(txn);

	newp->next = pos;
	newp->pprev = slot;

	/* The reader-visible edge: *slot: pos -> newp. */
	if (!urcu_txn_sw_record_chain(txn, (void **) slot, pos, newp,
			URCU_TXN_SW_HLIST_TAG))
		return 0;		/* OOM: store nothing */
	pos->pprev = &newp->next;		/* writer-only: a plain store */
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
 * {&head->first: A -> B}, then sets B->pprev = &head->first.  del(B) reads
 * B->pprev and gets &head->first, not the stale &A->next, so it records against
 * &head->first, finds it recorded, and chains: {&head->first: A -> C}.  It then
 * sets C->pprev = &head->first.  Both nodes are unlinked with a single record.
 *
 * This relies on pprev being stored at _prepare time.  Were the store deferred
 * to after the commit, del(B) would read &A->next, a slot inside the node that
 * del(A) just unlinked, and head->first would be left naming the deleted B.
 */
static inline
int urcu_txn_sw_hlist_del_prepare(struct urcu_txn_sw_txn *txn,
		struct urcu_txn_sw_hlist_node *elem)
{
	struct urcu_txn_sw_hlist_node *next =
			urcu_txn_sw_hlist_pending_next(txn, &elem->next);
	/* pprev is plain-stored at _prepare time: read it directly. */
	struct urcu_txn_sw_hlist_node **ppv = elem->pprev;

	URCU_TXN_SW_HLIST__ASSERT_ROLLBACKABLE(txn);

	/* The reader-visible edge: *ppv: elem -> next. */
	if (!urcu_txn_sw_record_chain(txn, (void **) ppv, elem, next,
			URCU_TXN_SW_HLIST_TAG))
		return 0;		/* OOM: store nothing */
	/* Writer-only: next takes elem's slot. */
	if (next != NULL)
		next->pprev = ppv;
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
	/* pprev is plain-stored at _prepare time: read it directly. */
	struct urcu_txn_sw_hlist_node **ppv = old->pprev;

	URCU_TXN_SW_HLIST__ASSERT_ROLLBACKABLE(txn);

	newp->next = next;
	newp->pprev = ppv;

	/* The reader-visible edge: *ppv: old -> newp. */
	if (!urcu_txn_sw_record_chain(txn, (void **) ppv, old, newp,
			URCU_TXN_SW_HLIST_TAG))
		return 0;		/* OOM: store nothing */
	/* Writer-only: next is now named by &newp->next. */
	if (next != NULL)
		next->pprev = &newp->next;
	return 0;
}

/*
 * The _rcu forms: each records one operation and commits it.
 *
 * An operation records exactly one edge, which commits as a lone store-release:
 * no proxy is installed, no descriptor is allocated and no grace period is
 * owed.  These forms therefore use a caller-storage handle over a one-record
 * on-stack buffer (urcu_txn_sw_init_inline()), and allocate nothing.
 *
 * Without an allocation they cannot fail, and always return 0.  The int return
 * is kept for source compatibility with the concurrent <urcu/rcu-txn-hlist.h>;
 * see "Return values" above.
 */
static inline
int urcu_txn_sw_hlist_add_head_rcu(struct urcu_txn_sw_hlist_node *newp,
		struct urcu_txn_sw_hlist_head *head)
{
	struct urcu_txn_sw_record buf[1];
	struct urcu_txn_sw_txn txn;

	urcu_txn_sw_init_inline(&txn, buf, 1);
	(void) urcu_txn_sw_hlist_add_head_prepare(&txn, newp, head);
	return urcu_txn_sw_commit(&txn) < 0 ? -1 : 0;
}

static inline
int urcu_txn_sw_hlist_add_after_rcu(struct urcu_txn_sw_hlist_node *newp,
		struct urcu_txn_sw_hlist_node *pos)
{
	struct urcu_txn_sw_record buf[1];
	struct urcu_txn_sw_txn txn;

	urcu_txn_sw_init_inline(&txn, buf, 1);
	(void) urcu_txn_sw_hlist_add_after_prepare(&txn, newp, pos);
	return urcu_txn_sw_commit(&txn) < 0 ? -1 : 0;
}

static inline
int urcu_txn_sw_hlist_add_before_rcu(struct urcu_txn_sw_hlist_node *newp,
		struct urcu_txn_sw_hlist_node *pos)
{
	struct urcu_txn_sw_record buf[1];
	struct urcu_txn_sw_txn txn;

	urcu_txn_sw_init_inline(&txn, buf, 1);
	(void) urcu_txn_sw_hlist_add_before_prepare(&txn, newp, pos);
	return urcu_txn_sw_commit(&txn) < 0 ? -1 : 0;
}

static inline
int urcu_txn_sw_hlist_del_rcu(struct urcu_txn_sw_hlist_node *elem)
{
	struct urcu_txn_sw_record buf[1];
	struct urcu_txn_sw_txn txn;

	urcu_txn_sw_init_inline(&txn, buf, 1);
	(void) urcu_txn_sw_hlist_del_prepare(&txn, elem);
	return urcu_txn_sw_commit(&txn) < 0 ? -1 : 0;
}

static inline
int urcu_txn_sw_hlist_replace_rcu(struct urcu_txn_sw_hlist_node *old,
		struct urcu_txn_sw_hlist_node *newp)
{
	struct urcu_txn_sw_record buf[1];
	struct urcu_txn_sw_txn txn;

	urcu_txn_sw_init_inline(&txn, buf, 1);
	(void) urcu_txn_sw_hlist_replace_prepare(&txn, old, newp);
	return urcu_txn_sw_commit(&txn) < 0 ? -1 : 0;
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
