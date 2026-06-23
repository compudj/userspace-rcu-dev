// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

#ifndef _URCU_RCU_BIDIR_LIST_H
#define _URCU_RCU_BIDIR_LIST_H

/*
 * rcu_bidir_list: a circular doubly-linked list whose forward AND backward
 * chains stay mutually coherent under concurrent RCU readers, so a reader
 * may iterate in either direction -- or reverse direction mid-traversal --
 * and never observe the two directions disagree.
 *
 * Why this is not just <urcu/rculist.h>
 * -------------------------------------
 * The classic RCU list (<urcu/rculist.h>) publishes only the *forward*
 * chain.  In cds_list_del_rcu() the forward edge is committed by a single
 * atomic store to prev->next, but the back edge is a plain assignment
 * (next->prev = ...): not rcu_assign_pointer, no release ordering, and the
 * prev pointers are never rcu_dereference()d.  A backward walk therefore has
 * no acquire to pair with and no atomic handoff -- it can read a prev value
 * inconsistent with the live forward chain.  And because a removed node is a
 * ghost reclaimed only after a grace period, a reverse walker can keep
 * stepping onto a node the forward chain has already dropped for as long as
 * that ghost survives: the prev view trails the next view by up to a grace
 * period.  That is why rculist.h ships only forward iterators
 * (cds_list_for_each_rcu and friends) and deliberately offers no RCU-safe
 * reverse walk.
 *
 * Every structural change to a doubly-linked list re-points exactly TWO
 * reader-visible edges -- one forward, one backward:
 *
 *   insert E between P and N:   P->next: N -> E   and   N->prev: P -> E
 *   delete E between P and N:   P->next: E -> N   and   N->prev: E -> P
 *
 * rcu_bidir_list flips both edges as ONE atomic event using the flip-latch
 * proxy mechanism (<urcu/flip-latch.h>): the two slots transiently hold
 * tagged proxies that share a single selector word; one release store flips
 * the selector and switches both edges from old to new together.  So at
 * every instant the forward and backward chains describe the same list.
 *
 * Coherence guarantee
 * -------------------
 * For any two nodes that are BOTH members of the list at the instant they
 * are resolved, the links are mutual inverses: a->next == b implies
 * b->prev == a.  An operation never exposes a window where one direction
 * sees the change and the other does not.
 *
 * As with any RCU structure this does NOT freeze the list for a long-lived
 * reader: a reader may still observe an operation that commits during its
 * walk.  What is guaranteed is that whatever it observes is internally
 * coherent in both directions at each step.
 *
 * Removed nodes are one-way ghosts (standard RCU): a deleted node keeps its
 * own next/prev pointing at its former live neighbours, so a reader sitting
 * on it can still escape into the live list in either direction, but the
 * live list no longer points back at it.  The node must be reclaimed by the
 * caller after a grace period, exactly as with cds_list_del_rcu().
 *
 * Read side
 * ---------
 * Iterate under rcu_read_lock().  Because a transacted slot may transiently
 * hold a tagged proxy, the next/prev fields MUST be read through the
 * resolving accessors (cds_bidir_list_next_rcu / cds_bidir_list_prev_rcu) or
 * the iterator macros below -- never by touching node->next / node->prev
 * directly.  This is also why the node type is distinct from
 * struct cds_list_head: the plain cds_list_*_rcu accessors would not resolve
 * proxies and must not be applied to a bidir list.
 *
 * Write side
 * ----------
 * Writers must be mutually excluded (as with cds_list_*_rcu).  Each mutator
 * drives a two-edge urcu_flip_txn (<urcu/flip-latch.h>): it records the
 * forward and the backward edge, commits them as one atomic flip, and
 * reclaims the transaction after a grace period via a caller-supplied
 * call_rcu (the flavor-appropriate one), keeping this header
 * RCU-flavor-agnostic.  A mutator returns 0 on success, or -1 if the
 * transaction could not be allocated (the list is left unchanged).
 */

#include <stdlib.h>

#include <urcu/compiler.h>
#include <urcu/uatomic.h>
#include <urcu/call-rcu.h>		/* struct rcu_head */
#include <urcu/flip-latch.h>
#include <urcu-pointer.h>		/* rcu_dereference / rcu_assign_pointer */

#ifdef __cplusplus
extern "C" {
#endif

/*
 * The doubly-linked node.  Embedded in the application structure (and used
 * directly as the list's sentinel head), like struct cds_list_head.  The
 * next/prev fields hold either a real node pointer or, transiently during a
 * mutation, a tagged flip proxy -- always read them through the accessors.
 */
struct cds_bidir_list_head {
	struct cds_bidir_list_head *next, *prev;
};

#define CDS_BIDIR_LIST_HEAD_INIT(name)	{ .next = &(name), .prev = &(name) }

#define CDS_BIDIR_LIST_HEAD(name) \
	struct cds_bidir_list_head name = CDS_BIDIR_LIST_HEAD_INIT(name)

static inline
void cds_bidir_list_init(struct cds_bidir_list_head *head)
{
	head->next = head;
	head->prev = head;
}

/*
 * Reclaim domain hook: the flavor's call_rcu (e.g. urcu/urcu-memb.h's
 * call_rcu).  Used only to free the per-operation proxy block.
 */
typedef void (*cds_bidir_list_call_rcu_fn)(struct rcu_head *head,
		void (*func)(struct rcu_head *head));

/*
 * Proxy tagging: a slot value with bit 0 set is a tagged
 * struct urcu_flip_proxy * rather than a direct node pointer.  Node and
 * proxy addresses are both at least pointer-aligned, so bit 0 is free.
 */
#define CDS_BIDIR_LIST_PROXY_TAG		1UL

static inline
void *cds_bidir_list_proxy_tag(struct urcu_flip_proxy *proxy)
{
	return (void *) ((unsigned long) proxy | CDS_BIDIR_LIST_PROXY_TAG);
}

/*
 * Resolve a slot value to the node it currently denotes: a tagged proxy
 * resolves through the flip selector (to its old or new target), a direct
 * node pointer passes through unchanged.
 */
static inline
struct cds_bidir_list_head *cds_bidir_list_resolve(struct cds_bidir_list_head *ptr)
{
	unsigned long v = (unsigned long) ptr;

	if (caa_unlikely(v & CDS_BIDIR_LIST_PROXY_TAG)) {
		struct urcu_flip_proxy *proxy = (struct urcu_flip_proxy *)
				(v & ~CDS_BIDIR_LIST_PROXY_TAG);

		return (struct cds_bidir_list_head *)
				urcu_flip_proxy_get(proxy);
	}
	return ptr;
}

/* Resolved forward / backward step (call under rcu_read_lock()). */
static inline
struct cds_bidir_list_head *cds_bidir_list_next_rcu(
		struct cds_bidir_list_head *node)
{
	return cds_bidir_list_resolve(rcu_dereference(node->next));
}

static inline
struct cds_bidir_list_head *cds_bidir_list_prev_rcu(
		struct cds_bidir_list_head *node)
{
	return cds_bidir_list_resolve(rcu_dereference(node->prev));
}

static inline
int cds_bidir_list_empty(struct cds_bidir_list_head *head)
{
	return cds_bidir_list_next_rcu(head) == head;
}

/*
 * Flip two edges atomically as one urcu_flip_txn: {*slot0: old0 -> new0} and
 * {*slot1: old1 -> new1} switch together, as observed by RCU readers.
 *
 * A thin, fixed-arity wrapper over the generic flip transaction
 * (<urcu/flip-latch.h>): create, reserve the two edges, record both, then
 * commit -- which auto-installs the proxies (selector 0 => readers still
 * resolve to old, so install is reader-transparent), flips the shared
 * selector 0 -> 1 (the one reader-visible instant, switching both edges to
 * new together), and settles each slot to its direct new target.  The
 * transaction is reclaimed after a grace period.
 *
 * A list op always transacts exactly two edges, so the txn's growable chunk
 * list, single-edge fast path and abort path are unused here; the only cost
 * over a bespoke fixed proxy block is the txn's second small allocation.
 */
static inline
int cds_bidir_list_flip2(
		struct cds_bidir_list_head **slot0,
		struct cds_bidir_list_head *old0,
		struct cds_bidir_list_head *new0,
		struct cds_bidir_list_head **slot1,
		struct cds_bidir_list_head *old1,
		struct cds_bidir_list_head *new1,
		cds_bidir_list_call_rcu_fn call_rcu_fn)
{
	struct urcu_flip_txn *txn;

	txn = urcu_flip_txn_create(cds_bidir_list_proxy_tag);
	if (caa_unlikely(!txn))
		return -1;
	if (caa_unlikely(!urcu_flip_txn_reserve(txn, 2))) {
		urcu_flip_txn_destroy(txn);
		return -1;
	}
	/* Reserved for two edges above, so neither record can fail. */
	(void) urcu_flip_txn_record(txn, (void **) slot0, old0, new0);
	(void) urcu_flip_txn_record(txn, (void **) slot1, old1, new1);
	/* Two edges => commit auto-installs, flips, settles, and owes a GP. */
	if (urcu_flip_txn_commit(txn))
		call_rcu_fn(&txn->rcu_head, urcu_flip_txn_free_rcu);
	else
		urcu_flip_txn_destroy(txn);	/* unreachable for two edges */
	return 0;
}

/* Insert @newp just after @pos (between @pos and its successor). */
static inline
int cds_bidir_list_add_after_rcu(struct cds_bidir_list_head *newp,
		struct cds_bidir_list_head *pos,
		cds_bidir_list_call_rcu_fn call_rcu_fn)
{
	struct cds_bidir_list_head *next = pos->next;

	/* Build the fresh node's links before it becomes reachable. */
	newp->prev = pos;
	newp->next = next;

	/* pos->next: next -> newp ; next->prev: pos -> newp */
	return cds_bidir_list_flip2(&pos->next, next, newp,
			&next->prev, pos, newp, call_rcu_fn);
}

/* Insert @newp just before @pos (between @pos's predecessor and @pos). */
static inline
int cds_bidir_list_add_before_rcu(struct cds_bidir_list_head *newp,
		struct cds_bidir_list_head *pos,
		cds_bidir_list_call_rcu_fn call_rcu_fn)
{
	struct cds_bidir_list_head *prev = pos->prev;

	newp->next = pos;
	newp->prev = prev;

	/* prev->next: pos -> newp ; pos->prev: prev -> newp */
	return cds_bidir_list_flip2(&prev->next, pos, newp,
			&pos->prev, prev, newp, call_rcu_fn);
}

/* Add @newp at the head of the list (just after @head). */
static inline
int cds_bidir_list_add_rcu(struct cds_bidir_list_head *newp,
		struct cds_bidir_list_head *head,
		cds_bidir_list_call_rcu_fn call_rcu_fn)
{
	return cds_bidir_list_add_after_rcu(newp, head, call_rcu_fn);
}

/* Add @newp at the tail of the list (just before @head). */
static inline
int cds_bidir_list_add_tail_rcu(struct cds_bidir_list_head *newp,
		struct cds_bidir_list_head *head,
		cds_bidir_list_call_rcu_fn call_rcu_fn)
{
	return cds_bidir_list_add_before_rcu(newp, head, call_rcu_fn);
}

/*
 * Remove @elem.  Its own next/prev are left intact (ghost) so a reader
 * standing on it can still escape in either direction; the caller frees
 * @elem after a grace period.
 */
static inline
int cds_bidir_list_del_rcu(struct cds_bidir_list_head *elem,
		cds_bidir_list_call_rcu_fn call_rcu_fn)
{
	struct cds_bidir_list_head *prev = elem->prev;
	struct cds_bidir_list_head *next = elem->next;

	/* prev->next: elem -> next ; next->prev: elem -> prev */
	return cds_bidir_list_flip2(&prev->next, elem, next,
			&next->prev, elem, prev, call_rcu_fn);
}

/* Replace @old with @newp atomically with respect to RCU readers. */
static inline
int cds_bidir_list_replace_rcu(struct cds_bidir_list_head *old,
		struct cds_bidir_list_head *newp,
		cds_bidir_list_call_rcu_fn call_rcu_fn)
{
	struct cds_bidir_list_head *prev = old->prev;
	struct cds_bidir_list_head *next = old->next;

	newp->prev = prev;
	newp->next = next;

	/* prev->next: old -> newp ; next->prev: old -> newp */
	return cds_bidir_list_flip2(&prev->next, old, newp,
			&next->prev, old, newp, call_rcu_fn);
}

#define cds_bidir_list_entry(ptr, type, member) \
	caa_container_of(ptr, type, member)

#define cds_bidir_list_first_entry_rcu(head, type, member) \
	cds_bidir_list_entry(cds_bidir_list_next_rcu(head), type, member)

#define cds_bidir_list_last_entry_rcu(head, type, member) \
	cds_bidir_list_entry(cds_bidir_list_prev_rcu(head), type, member)

/* Iterate forward over the list (under rcu_read_lock()). */
#define cds_bidir_list_for_each_rcu(pos, head) \
	for (pos = cds_bidir_list_next_rcu(head); \
		(pos) != (head); \
		pos = cds_bidir_list_next_rcu(pos))

/* Iterate backward over the list (under rcu_read_lock()). */
#define cds_bidir_list_for_each_reverse_rcu(pos, head) \
	for (pos = cds_bidir_list_prev_rcu(head); \
		(pos) != (head); \
		pos = cds_bidir_list_prev_rcu(pos))

#define cds_bidir_list_for_each_entry_rcu(pos, head, member) \
	for (pos = cds_bidir_list_entry(cds_bidir_list_next_rcu(head), \
			__typeof__(*(pos)), member); \
		&(pos)->member != (head); \
		pos = cds_bidir_list_entry( \
			cds_bidir_list_next_rcu(&(pos)->member), \
			__typeof__(*(pos)), member))

#define cds_bidir_list_for_each_entry_reverse_rcu(pos, head, member) \
	for (pos = cds_bidir_list_entry(cds_bidir_list_prev_rcu(head), \
			__typeof__(*(pos)), member); \
		&(pos)->member != (head); \
		pos = cds_bidir_list_entry( \
			cds_bidir_list_prev_rcu(&(pos)->member), \
			__typeof__(*(pos)), member))

#ifdef __cplusplus
}
#endif

#endif	/* _URCU_RCU_BIDIR_LIST_H */
