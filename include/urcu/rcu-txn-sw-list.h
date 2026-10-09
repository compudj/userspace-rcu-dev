// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

#ifndef _URCU_RCU_TXN_SW_LIST_H
#define _URCU_RCU_TXN_SW_LIST_H

/*
 * rcu-txn-sw-list: a circular doubly-linked list whose forward and backward
 * chains stay mutually coherent under concurrent RCU readers.  A reader may
 * iterate in either direction, or reverse direction mid-traversal, and never
 * observe the two directions disagree.
 *
 * Why this is not just <urcu/rculist.h>
 * -------------------------------------
 * The classic RCU list publishes only the forward chain.  cds_list_del_rcu()
 * commits the forward edge with one atomic store to prev->next, but updates the
 * back edge with a plain assignment, and prev pointers are never
 * rcu_dereference()d.  A backward walk can therefore read a prev value
 * inconsistent with the forward chain, and keep stepping onto a removed node
 * for as long as a grace period.  This is why rculist.h offers no RCU-safe
 * reverse iterator.
 *
 * Every structural change to a doubly-linked list re-points exactly two
 * reader-visible edges, one forward and one backward:
 *
 *   insert E between P and N:   P->next: N -> E   and   N->prev: P -> E
 *   delete E between P and N:   P->next: E -> N   and   N->prev: E -> P
 *
 * rcu-txn-sw-list commits both edges as one RCU pseudo-transaction
 * (<urcu/rcu-txn-sw.h>): the two slots transiently hold tagged proxies that
 * share one selector, and one store-release switches both edges from old to
 * new.  At every instant the forward and backward chains describe the same
 * list.
 *
 * Coherence guarantee
 * -------------------
 * For any two nodes that are both members of the list at the instant they are
 * resolved, the links are mutual inverses: a->next == b implies b->prev == a.
 * No operation exposes a window where one direction sees a change and the other
 * does not.
 *
 * As with any RCU structure, this does not freeze the list for a long-lived
 * reader: it may observe an operation that commits during its walk.  What it
 * observes is coherent in both directions at each step.
 *
 * Coherence is a property of one resolution.  Carrying it across two
 * resolutions needs an ordering between them, which a dependency-chained walk
 * has (next-then-next, or a prev hop taken from the node the previous load
 * returned).  A reader comparing two independently-reached slots must supply
 * its own acquire under -DURCU_DEREFERENCE_USE_VOLATILE on weakly-ordered
 * hardware; see the monotonicity note in <urcu/rcu-txn-sw.h>.
 *
 * A removed node keeps its own next/prev pointing at its former neighbours, so
 * a reader standing on it can still reach the live list in either direction,
 * but the live list no longer points back at it.  The caller reclaims the node
 * after a grace period, as with cds_list_del_rcu().
 *
 * Read side
 * ---------
 * Iterate within an RCU read-side critical section.  A slot may transiently
 * hold a tagged proxy, so next/prev must be read through
 * urcu_txn_sw_list_next_rcu() / urcu_txn_sw_list_prev_rcu() or the iterators
 * below, never directly.  This is also why the node type is distinct from
 * struct cds_list_head: the cds_list_*_rcu accessors do not resolve proxies.
 *
 * Write side
 * ----------
 * Writers must be mutually excluded, as with cds_list_*_rcu.  Each mutator
 * records its forward and backward edge in a urcu_txn_sw_txn and commits them
 * together.  The transaction reclaims itself through call_rcu() after a grace
 * period, so this header must be included after an RCU flavor header (e.g.
 * <urcu-qsbr.h>).  A mutator returns 0 on success, or -1 if the transaction
 * could not be allocated, in which case the list is unchanged.
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
 * The list node, embedded in the application structure.  next/prev hold a node
 * pointer or, transiently during a mutation, a tagged proxy: always read them
 * through the accessors.
 */
struct urcu_txn_sw_list_node {
	struct urcu_txn_sw_list_node *next, *prev;
};

/*
 * The list head embeds the circular sentinel node.  Head and node are distinct
 * types, as in the concurrent <urcu/rcu-txn-list.h>.
 */
struct urcu_txn_sw_list_head {
	struct urcu_txn_sw_list_node node;	/* circular sentinel */
};

#define URCU_TXN_SW_LIST_HEAD_INIT(name) \
	{ .node = { .next = &(name).node, .prev = &(name).node } }

#define URCU_TXN_SW_LIST_HEAD(name) \
	struct urcu_txn_sw_list_head name = URCU_TXN_SW_LIST_HEAD_INIT(name)

static inline
void urcu_txn_sw_list_init(struct urcu_txn_sw_list_head *head)
{
	head->node.next = &head->node;
	head->node.prev = &head->node;
}

/*
 * A slot value with bit 0 set is a tagged struct urcu_txn_sw_record * rather
 * than a node pointer.  Nodes and records are at least pointer-aligned, so
 * bit 0 is free.
 */
#define URCU_TXN_SW_LIST_PROXY_TAG		1UL

/*
 * Resolve a slot value to the node it currently denotes: a tagged proxy
 * resolves to its old or new target, a node pointer is returned as is.
 */
static inline
struct urcu_txn_sw_list_node *urcu_txn_sw_list_resolve(
		struct urcu_txn_sw_list_node *ptr)
{
	return (struct urcu_txn_sw_list_node *)
			urcu_txn_sw_resolve(ptr, URCU_TXN_SW_LIST_PROXY_TAG);
}

/*
 * Write-side read of a link slot: this transaction's pending write to @slot if
 * it recorded one, else the slot's committed value.  A typed wrapper over
 * urcu_txn_sw_load().
 *
 * Every _prepare below reads its neighbour links through this rather than
 * ->next / ->prev, which is what lets edits compose on one list (see
 * urcu_txn_sw_list_add_after_prepare()).  Readers use
 * urcu_txn_sw_list_next_rcu() / _prev_rcu() instead.
 */
static inline
struct urcu_txn_sw_list_node *urcu_txn_sw_list_pending(
		struct urcu_txn_sw_txn *txn,
		struct urcu_txn_sw_list_node **slot)
{
	return (struct urcu_txn_sw_list_node *) urcu_txn_sw_load(txn,
			(void **) slot, URCU_TXN_SW_LIST_PROXY_TAG);
}

/* Forward and backward steps, for use within an RCU read-side section. */
static inline
struct urcu_txn_sw_list_node *urcu_txn_sw_list_next_rcu(
		struct urcu_txn_sw_list_node *node)
{
	return urcu_txn_sw_list_resolve(rcu_dereference(node->next));
}

static inline
struct urcu_txn_sw_list_node *urcu_txn_sw_list_prev_rcu(
		struct urcu_txn_sw_list_node *node)
{
	return urcu_txn_sw_list_resolve(rcu_dereference(node->prev));
}

/*
 * True iff the list is empty.  Call within an RCU read-side critical section,
 * like the accessors it is built on.
 */
static inline
int urcu_txn_sw_list_empty(struct urcu_txn_sw_list_head *head)
{
	return urcu_txn_sw_list_next_rcu(&head->node) == &head->node;
}

/*
 * Commit two edges {*slot0: old0 -> new0} and {*slot1: old1 -> new1} as one
 * transaction: both switch together, as observed by RCU readers.
 *
 * A convenience over <urcu/rcu-txn-sw.h>.  The list operations below do not use
 * it: they go through the _prepare forms, so that several operations can share
 * one transaction.
 *
 * @slot0 and @slot1 must be distinct.
 *
 * Returns 0 on success, -1 on allocation failure.
 */
static inline
int urcu_txn_sw_list_flip2(
		struct urcu_txn_sw_list_node **slot0,
		struct urcu_txn_sw_list_node *old0,
		struct urcu_txn_sw_list_node *new0,
		struct urcu_txn_sw_list_node **slot1,
		struct urcu_txn_sw_list_node *old1,
		struct urcu_txn_sw_list_node *new1)
{
	struct urcu_txn_sw_txn txn;

	urcu_posix_assert(slot0 != slot1);
	urcu_txn_sw_init(&txn);
	/* An OOM is sticky: commit reports it, and then publishes nothing. */
	(void) urcu_txn_sw_reserve(&txn, 2);
	(void) urcu_txn_sw_record(&txn, (void **) slot0, old0, new0,
			URCU_TXN_SW_LIST_PROXY_TAG);
	(void) urcu_txn_sw_record(&txn, (void **) slot1, old1, new1,
			URCU_TXN_SW_LIST_PROXY_TAG);
	return urcu_txn_sw_commit(&txn) < 0 ? -1 : 0;
}

/*
 * Record the two edges that insert @newp after @pos into the caller's
 * transaction @txn, without committing.  The caller owns the transaction, from
 * init to commit, and may combine these records with others, including records
 * of other structures: e.g. publish a node into a trie and splice it into this
 * list atomically.
 *
 * Always returns 0: under a single writer there is no concurrent deletion to
 * fail on.  The int return matches urcu_txn_list_insert_after_prepare() of the
 * concurrent <urcu/rcu-txn-list.h>.
 *
 * Composing edits of one list
 * ---------------------------
 * Several _prepare calls may share a transaction, even when their
 * neighbourhoods touch.  Each reads its neighbour links through
 * urcu_txn_sw_list_pending() and records through urcu_txn_sw_record_chain(),
 * so it sees the transaction's own pending edits, and a second write to a slot
 * chains onto the first.
 *
 * Example: P -> E1 -> E2 -> N, deleting E1 and E2 in one transaction.  del(E1)
 * records {&P->next: E1 -> E2} and {&E2->prev: E1 -> P}.  del(E2) then reads
 * E2->prev and gets the pending P, not the committed E1, so it records against
 * &P->next, finds it recorded, and chains: {&P->next: E1 -> N}.  With
 * {&N->prev: E2 -> P}, P and N end up linked and both victims unlinked.
 *
 * Two rules remain for the caller:
 *
 *   - Name the nodes before editing.  A traversal made with the _rcu accessors
 *     sees the committed list, not the transaction's pending one, so a caller
 *     that walks and edits in the same transaction may name a node that its own
 *     earlier edit displaced.
 *
 *   - Do not call urcu_txn_sw_declare_disjoint() on a transaction that composes
 *     edits.  It turns off both mechanisms above: in the example, del(E2) would
 *     read the committed E1 and record {&E1->next: E2 -> N} and
 *     {&N->prev: E2 -> E1}, and the commit would link P and N to deleted
 *     nodes.  The four slots are distinct, so only
 *     -DURCU_TXN_SW_DEBUG_DISJOINT traps it, at the load.  The _rcu wrappers
 *     below declare disjoint because each carries a single operation.
 */
static inline
int urcu_txn_sw_list_add_after_prepare(struct urcu_txn_sw_txn *txn,
		struct urcu_txn_sw_list_node *newp,
		struct urcu_txn_sw_list_node *pos)
{
	struct urcu_txn_sw_list_node *next =
			urcu_txn_sw_list_pending(txn, &pos->next);

	/* Build the fresh node's links before it becomes reachable. */
	newp->prev = pos;
	newp->next = next;

	/* pos->next: next -> newp ; next->prev: pos -> newp */
	(void) urcu_txn_sw_record_chain(txn, (void **) &pos->next, next, newp,
			URCU_TXN_SW_LIST_PROXY_TAG);
	(void) urcu_txn_sw_record_chain(txn, (void **) &next->prev, pos, newp,
			URCU_TXN_SW_LIST_PROXY_TAG);
	return 0;
}

/* Insert @newp after @pos, in a transaction of its own. */
static inline
int urcu_txn_sw_list_add_after_rcu(struct urcu_txn_sw_list_node *newp,
		struct urcu_txn_sw_list_node *pos)
{
	struct urcu_txn_sw_txn txn;

	urcu_txn_sw_init(&txn);
	/* One operation: its two slots are distinct. */
	urcu_txn_sw_declare_disjoint(&txn);
	/* An OOM is sticky: commit reports it. */
	(void) urcu_txn_sw_reserve(&txn, 2);
	(void) urcu_txn_sw_list_add_after_prepare(&txn, newp, pos);
	return urcu_txn_sw_commit(&txn) < 0 ? -1 : 0;
}

/*
 * Record the two edges that insert @newp before @pos into @txn, without
 * committing.  See urcu_txn_sw_list_add_after_prepare().  Always returns 0.
 */
static inline
int urcu_txn_sw_list_add_before_prepare(struct urcu_txn_sw_txn *txn,
		struct urcu_txn_sw_list_node *newp,
		struct urcu_txn_sw_list_node *pos)
{
	struct urcu_txn_sw_list_node *prev =
			urcu_txn_sw_list_pending(txn, &pos->prev);

	newp->next = pos;
	newp->prev = prev;

	/* prev->next: pos -> newp ; pos->prev: prev -> newp */
	(void) urcu_txn_sw_record_chain(txn, (void **) &prev->next, pos, newp,
			URCU_TXN_SW_LIST_PROXY_TAG);
	(void) urcu_txn_sw_record_chain(txn, (void **) &pos->prev, prev, newp,
			URCU_TXN_SW_LIST_PROXY_TAG);
	return 0;
}

/* Insert @newp before @pos, in a transaction of its own. */
static inline
int urcu_txn_sw_list_add_before_rcu(struct urcu_txn_sw_list_node *newp,
		struct urcu_txn_sw_list_node *pos)
{
	struct urcu_txn_sw_txn txn;

	urcu_txn_sw_init(&txn);
	/* One operation: its two slots are distinct. */
	urcu_txn_sw_declare_disjoint(&txn);
	/* An OOM is sticky: commit reports it. */
	(void) urcu_txn_sw_reserve(&txn, 2);
	(void) urcu_txn_sw_list_add_before_prepare(&txn, newp, pos);
	return urcu_txn_sw_commit(&txn) < 0 ? -1 : 0;
}

/* Add @newp at the head of the list (just after @head). */
static inline
int urcu_txn_sw_list_add_rcu(struct urcu_txn_sw_list_node *newp,
		struct urcu_txn_sw_list_head *head)
{
	return urcu_txn_sw_list_add_after_rcu(newp, &head->node);
}

/* Add @newp at the tail of the list (just before @head). */
static inline
int urcu_txn_sw_list_add_tail_rcu(struct urcu_txn_sw_list_node *newp,
		struct urcu_txn_sw_list_head *head)
{
	return urcu_txn_sw_list_add_before_rcu(newp, &head->node);
}

/*
 * Record the unlink of @elem into @txn, without committing.  See
 * urcu_txn_sw_list_add_after_prepare().  @elem's own next/prev are left as they
 * are, so a reader standing on it can still leave in either direction; the
 * caller frees @elem a grace period after the commit.  Always returns 0; the
 * int return matches urcu_txn_list_del_prepare().
 */
static inline
int urcu_txn_sw_list_del_prepare(struct urcu_txn_sw_txn *txn,
		struct urcu_txn_sw_list_node *elem)
{
	struct urcu_txn_sw_list_node *prev =
			urcu_txn_sw_list_pending(txn, &elem->prev);
	struct urcu_txn_sw_list_node *next =
			urcu_txn_sw_list_pending(txn, &elem->next);

	/* prev->next: elem -> next ; next->prev: elem -> prev */
	(void) urcu_txn_sw_record_chain(txn, (void **) &prev->next, elem, next,
			URCU_TXN_SW_LIST_PROXY_TAG);
	(void) urcu_txn_sw_record_chain(txn, (void **) &next->prev, elem, prev,
			URCU_TXN_SW_LIST_PROXY_TAG);
	return 0;
}

/*
 * Remove @elem, in a transaction of its own.  The caller frees @elem after a
 * grace period.
 */
static inline
int urcu_txn_sw_list_del_rcu(struct urcu_txn_sw_list_node *elem)
{
	struct urcu_txn_sw_txn txn;

	urcu_txn_sw_init(&txn);
	/* One operation: its two slots are distinct. */
	urcu_txn_sw_declare_disjoint(&txn);
	/* An OOM is sticky: commit reports it. */
	(void) urcu_txn_sw_reserve(&txn, 2);
	(void) urcu_txn_sw_list_del_prepare(&txn, elem);
	return urcu_txn_sw_commit(&txn) < 0 ? -1 : 0;
}

/*
 * Record the replacement of @old by @newp into @txn, without committing.  See
 * urcu_txn_sw_list_add_after_prepare().  @newp takes @old's neighbours, and
 * @old keeps its own next/prev for readers standing on it.  Always returns 0;
 * the int return matches urcu_txn_list_replace_prepare().
 */
static inline
int urcu_txn_sw_list_replace_prepare(struct urcu_txn_sw_txn *txn,
		struct urcu_txn_sw_list_node *old,
		struct urcu_txn_sw_list_node *newp)
{
	struct urcu_txn_sw_list_node *prev =
			urcu_txn_sw_list_pending(txn, &old->prev);
	struct urcu_txn_sw_list_node *next =
			urcu_txn_sw_list_pending(txn, &old->next);

	newp->prev = prev;
	newp->next = next;

	/* prev->next: old -> newp ; next->prev: old -> newp */
	(void) urcu_txn_sw_record_chain(txn, (void **) &prev->next, old, newp,
			URCU_TXN_SW_LIST_PROXY_TAG);
	(void) urcu_txn_sw_record_chain(txn, (void **) &next->prev, old, newp,
			URCU_TXN_SW_LIST_PROXY_TAG);
	return 0;
}

/*
 * Replace @old with @newp, in a transaction of its own.  The caller frees @old
 * after a grace period.
 */
static inline
int urcu_txn_sw_list_replace_rcu(struct urcu_txn_sw_list_node *old,
		struct urcu_txn_sw_list_node *newp)
{
	struct urcu_txn_sw_txn txn;

	urcu_txn_sw_init(&txn);
	/* One operation: its two slots are distinct. */
	urcu_txn_sw_declare_disjoint(&txn);
	/* An OOM is sticky: commit reports it. */
	(void) urcu_txn_sw_reserve(&txn, 2);
	(void) urcu_txn_sw_list_replace_prepare(&txn, old, newp);
	return urcu_txn_sw_commit(&txn) < 0 ? -1 : 0;
}

#define urcu_txn_sw_list_entry(ptr, type, member) \
	caa_container_of(ptr, type, member)

#define urcu_txn_sw_list_first_entry_rcu(head, type, member) \
	urcu_txn_sw_list_entry(urcu_txn_sw_list_next_rcu(&(head)->node), \
			type, member)

#define urcu_txn_sw_list_last_entry_rcu(head, type, member) \
	urcu_txn_sw_list_entry(urcu_txn_sw_list_prev_rcu(&(head)->node), \
			type, member)

/* Iterate forward over the list (under rcu_read_lock()). */
#define urcu_txn_sw_list_for_each_rcu(pos, head) \
	for (pos = urcu_txn_sw_list_next_rcu(&(head)->node); \
		(pos) != &(head)->node; \
		pos = urcu_txn_sw_list_next_rcu(pos))

/* Iterate backward over the list (under rcu_read_lock()). */
#define urcu_txn_sw_list_for_each_reverse_rcu(pos, head) \
	for (pos = urcu_txn_sw_list_prev_rcu(&(head)->node); \
		(pos) != &(head)->node; \
		pos = urcu_txn_sw_list_prev_rcu(pos))

#define urcu_txn_sw_list_for_each_entry_rcu(pos, head, member) \
	for (pos = urcu_txn_sw_list_entry( \
			urcu_txn_sw_list_next_rcu(&(head)->node), \
			__typeof__(*(pos)), member); \
		&(pos)->member != &(head)->node; \
		pos = urcu_txn_sw_list_entry( \
			urcu_txn_sw_list_next_rcu(&(pos)->member), \
			__typeof__(*(pos)), member))

#define urcu_txn_sw_list_for_each_entry_reverse_rcu(pos, head, member) \
	for (pos = urcu_txn_sw_list_entry( \
			urcu_txn_sw_list_prev_rcu(&(head)->node), \
			__typeof__(*(pos)), member); \
		&(pos)->member != &(head)->node; \
		pos = urcu_txn_sw_list_entry( \
			urcu_txn_sw_list_prev_rcu(&(pos)->member), \
			__typeof__(*(pos)), member))

#ifdef __cplusplus
}
#endif

#endif	/* _URCU_RCU_TXN_SW_LIST_H */
