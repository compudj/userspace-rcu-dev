// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

#ifndef _URCU_RCU_TXN_SW_LIST_H
#define _URCU_RCU_TXN_SW_LIST_H

/*
 * rcu-txn-sw-list: a circular doubly-linked list whose forward AND backward
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
 * rcu-txn-sw-list flips both edges as ONE atomic event using the proxy-flip
 * mechanism of <urcu/rcu-txn-sw.h>: the two slots transiently hold
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
 * "At each step" is the precise scope: coherence is a property of a single
 * resolution, and carrying it ACROSS two resolutions needs an ordering edge
 * between them.  A dependency-chained walk (next-then-next, or a prev hop taken
 * FROM the node the previous load returned) has one on every supported
 * architecture, as does any build using the default C11 dereference.  A reader
 * comparing two independently-reached slots -- a cached pointer, or prev and
 * next read from nodes it did not chain between -- must supply its own acquire
 * under -DURCU_DEREFERENCE_USE_VOLATILE on weakly-ordered hardware.  See the
 * ordering/monotonicity note in <urcu/rcu-txn-sw.h>.
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
 * resolving accessors (urcu_txn_sw_list_next_rcu / urcu_txn_sw_list_prev_rcu) or
 * the iterator macros below -- never by touching node->next / node->prev
 * directly.  This is also why the node type is distinct from
 * struct cds_list_head: the plain cds_list_*_rcu accessors would not resolve
 * proxies and must not be applied to a bidir list.
 *
 * Write side
 * ----------
 * Writers must be mutually excluded (as with cds_list_*_rcu).  Each mutator
 * drives a two-edge urcu_txn_sw_txn (<urcu/rcu-txn-sw.h>): it records the
 * forward and the backward edge and commits them as one atomic flip; the
 * transaction layer then reclaims itself after a grace period through
 * call_rcu().  Because commit() calls call_rcu() directly, include this header
 * AFTER an RCU flavor header (e.g. <urcu-qsbr.h>) that maps call_rcu() to that
 * flavor.  A mutator returns 0 on success, or -1 if the transaction could not
 * be allocated (the list is left unchanged).
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
 * The doubly-linked node, embedded in the application structure.  The next/prev
 * fields hold either a real node pointer or, transiently during a mutation, a
 * tagged flip proxy -- always read them through the accessors.
 */
struct urcu_txn_sw_list_node {
	struct urcu_txn_sw_list_node *next, *prev;
};

/*
 * The list head embeds the circular sentinel node, so node and head are
 * distinct types -- matching the concurrent <urcu/rcu-txn-list.h>.  A
 * single-updater list has no escalation domain, so the head holds only the
 * sentinel.
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
 * Proxy tagging: a slot value with bit 0 set is a tagged
 * struct urcu_txn_sw_proxy * rather than a direct node pointer.  Node and
 * proxy addresses are both at least pointer-aligned, so bit 0 is free.
 */
#define URCU_TXN_SW_LIST_PROXY_TAG		1UL

/*
 * Resolve a slot value to the node it currently denotes: a tagged proxy
 * resolves through the flip selector (to its old or new target), a direct
 * node pointer passes through unchanged.
 */
static inline
struct urcu_txn_sw_list_node *urcu_txn_sw_list_resolve(struct urcu_txn_sw_list_node *ptr)
{
	return (struct urcu_txn_sw_list_node *)
			urcu_txn_sw_resolve(ptr, URCU_TXN_SW_LIST_PROXY_TAG);
}

/*
 * WRITE-SIDE read of a link slot: its value as @txn will leave it -- this
 * transaction's pending write to @slot if it has recorded one, else the slot's
 * committed value.  A typed wrapper over the engine's read-your-own-writes load
 * (<urcu/rcu-txn-sw.h>).
 *
 * Every _prepare below reads its neighbour links through this rather than
 * touching ->next / ->prev directly: that is what lets edits COMPOSE on one
 * list (see urcu_txn_sw_list_add_after_prepare()).  Writer-side only -- a
 * reader wants urcu_txn_sw_list_next_rcu() / _prev_rcu(), which resolve against
 * the flip selector instead and know nothing of a transaction's pending state.
 */
static inline
struct urcu_txn_sw_list_node *urcu_txn_sw_list_pending(
		struct urcu_txn_sw_txn *txn,
		struct urcu_txn_sw_list_node **slot)
{
	return (struct urcu_txn_sw_list_node *) urcu_txn_sw_load(txn,
			(void **) slot, URCU_TXN_SW_LIST_PROXY_TAG);
}

/* Resolved forward / backward step (call under rcu_read_lock()). */
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

static inline
int urcu_txn_sw_list_empty(struct urcu_txn_sw_list_head *head)
{
	return urcu_txn_sw_list_next_rcu(&head->node) == &head->node;
}

/*
 * Flip two edges atomically as one urcu_txn_sw_txn: {*slot0: old0 -> new0} and
 * {*slot1: old1 -> new1} switch together, as observed by RCU readers.
 *
 * A thin, fixed-arity wrapper over the generic flip transaction
 * (<urcu/rcu-txn-sw.h>): init an on-stack handle, reserve the two edges, record
 * both, then commit -- which auto-installs the proxies (selector 0 => readers
 * still resolve to old, so install is reader-transparent), flips the shared
 * selector 0 -> 1 (the one reader-visible instant, switching both edges to new
 * together), and settles each slot to its direct new target.  commit() owns
 * reclaim and defers the group block through call_rcu() after a grace period.
 *
 * A list op always transacts exactly two edges, so the txn's record-array
 * growth, single-edge fast path and abort path are unused here; the only cost
 * over a bespoke fixed proxy block is the record array and the group block,
 * freed together by one call_rcu.
 *
 * Returns 0 on success, -1 on allocation failure.  An OOM in reserve()/record()
 * is sticky and surfaces as commit()'s MEMORY_ERROR, so only the final commit
 * status needs checking -- the on-stack handle has no create() to fail.
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

	urcu_txn_sw_init(&txn);
	(void) urcu_txn_sw_reserve(&txn, 2);	/* sticky OOM -> commit reports it */
	(void) urcu_txn_sw_record(&txn, (void **) slot0, old0, new0, URCU_TXN_SW_LIST_PROXY_TAG);
	(void) urcu_txn_sw_record(&txn, (void **) slot1, old1, new1, URCU_TXN_SW_LIST_PROXY_TAG);
	/*
	 * Two edges => commit auto-installs, flips, settles, and owns reclaim
	 * (call_rcu).  MEMORY_ERROR (< 0) means an alloc failed and nothing was
	 * published; otherwise the flip committed.
	 */
	return urcu_txn_sw_commit(&txn) < 0 ? -1 : 0;
}

/*
 * urcu_txn_sw_list_add_after_prepare: record the edges of an add-after into the
 * caller-owned single-updater transaction @txn, WITHOUT committing.  The
 * composable form: the caller owns the bracket (init .. commit) and may fold
 * these records together with records from other structures into ONE flip --
 * e.g. publish a node into a trie and splice it into this list atomically.
 * Under a single updater there is no concurrent deletion, so it always succeeds
 * (returns 0); the int return matches the concurrent variant so callers share
 * one shape across the single-updater -> concurrent transition.  Each recorded
 * edge is tagged with URCU_TXN_SW_LIST_PROXY_TAG so the list's reader accessors
 * resolve the proxy.
 *
 * SAME-LIST COMPOSITION IS LEGAL, including edits whose neighbourhoods touch.
 * Every _prepare reads its neighbour links through urcu_txn_sw_list_pending()
 * (the engine's read-your-own-writes load) and records through
 * urcu_txn_sw_record_chain(), so a second prepare in the same bracket sees this
 * transaction's own pending edits rather than the pre-transaction values, and a
 * residual same-slot collision fuses onto the existing record instead of
 * duplicating it.  This is the same mechanism -- and the same guarantee -- as
 * urcu_txn_list_insert_after_prepare()'s in the concurrent twin.
 *
 * Adjacent deletes were the canonical trap, and are worth following through.
 * P -> E1 -> E2 -> N, deleting E1 and E2 in one flip.  del(E1) records
 * {&P->next: E1 -> E2} and {&E2->prev: E1 -> P}.  del(E2) then reads
 * E2->prev and gets the PENDING P (not the committed E1), so it records against
 * &P->next -- which it finds already recorded, and chains: {&P->next: E1 -> N}.
 * With {&N->prev: E2 -> P} that leaves P <-> N linked and both victims
 * unlinked.  Reading RAW instead -- as this header did before the engine grew
 * the RYW pair -- produced {&E1->next: E2 -> N} and {&N->prev: E2 -> E1} on
 * four PAIRWISE-DISTINCT slots, so install's duplicate scan passed and the
 * commit published P->next == E2 and N->prev == E1, both deleted nodes.
 *
 * What composition still requires of the CALLER: the nodes it names must be the
 * ones it means.  These prepares re-read the links of the node handed to them,
 * but they cannot re-run the caller's search -- a traversal made with the _rcu
 * accessors sees the committed list, not this transaction's pending one, so a
 * caller that walks and edits in the same bracket may still name a node its own
 * earlier edit has displaced.  Identify the victims first, then edit.
 */
static inline
int urcu_txn_sw_list_add_after_prepare(struct urcu_txn_sw_txn *txn,
		struct urcu_txn_sw_list_node *newp,
		struct urcu_txn_sw_list_node *pos)
{
	struct urcu_txn_sw_list_node *next = urcu_txn_sw_list_pending(txn, &pos->next);

	/* Build the fresh node's links before it becomes reachable. */
	newp->prev = pos;
	newp->next = next;

	/* pos->next: next -> newp ; next->prev: pos -> newp */
	(void) urcu_txn_sw_record_chain(txn, (void **) &pos->next, next, newp, URCU_TXN_SW_LIST_PROXY_TAG);
	(void) urcu_txn_sw_record_chain(txn, (void **) &next->prev, pos, newp, URCU_TXN_SW_LIST_PROXY_TAG);
	return 0;
}

/*
 * Insert @newp just after @pos (between @pos and its successor).  Convenience
 * bracket around urcu_txn_sw_list_add_after_prepare().
 */
static inline
int urcu_txn_sw_list_add_after_rcu(struct urcu_txn_sw_list_node *newp,
		struct urcu_txn_sw_list_node *pos)
{
	struct urcu_txn_sw_txn txn;

	urcu_txn_sw_init(&txn);
	urcu_txn_sw_declare_disjoint(&txn);	/* single-op commit: distinct slots, no same-slot WAW */
	(void) urcu_txn_sw_reserve(&txn, 2);	/* sticky OOM -> commit reports it */
	(void) urcu_txn_sw_list_add_after_prepare(&txn, newp, pos);
	return urcu_txn_sw_commit(&txn) < 0 ? -1 : 0;
}

/*
 * urcu_txn_sw_list_add_before_prepare: composable form of add-before (see
 * add_after_prepare for the contract).  Always returns 0.
 */
static inline
int urcu_txn_sw_list_add_before_prepare(struct urcu_txn_sw_txn *txn,
		struct urcu_txn_sw_list_node *newp,
		struct urcu_txn_sw_list_node *pos)
{
	struct urcu_txn_sw_list_node *prev = urcu_txn_sw_list_pending(txn, &pos->prev);

	newp->next = pos;
	newp->prev = prev;

	/* prev->next: pos -> newp ; pos->prev: prev -> newp */
	(void) urcu_txn_sw_record_chain(txn, (void **) &prev->next, pos, newp, URCU_TXN_SW_LIST_PROXY_TAG);
	(void) urcu_txn_sw_record_chain(txn, (void **) &pos->prev, prev, newp, URCU_TXN_SW_LIST_PROXY_TAG);
	return 0;
}

/*
 * Insert @newp just before @pos (between @pos's predecessor and @pos).
 * Convenience bracket around urcu_txn_sw_list_add_before_prepare().
 */
static inline
int urcu_txn_sw_list_add_before_rcu(struct urcu_txn_sw_list_node *newp,
		struct urcu_txn_sw_list_node *pos)
{
	struct urcu_txn_sw_txn txn;

	urcu_txn_sw_init(&txn);
	urcu_txn_sw_declare_disjoint(&txn);	/* single-op commit: distinct slots, no same-slot WAW */
	(void) urcu_txn_sw_reserve(&txn, 2);	/* sticky OOM -> commit reports it */
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
 * urcu_txn_sw_list_del_prepare: record the unlink of @elem into the caller-owned
 * single-updater transaction @txn, WITHOUT committing.  Composable form of del
 * (see add_after_prepare for the contract).  @elem's own next/prev are left
 * intact (ghost) so a reader standing on it can still escape in either
 * direction; the caller frees @elem after a grace period (post-commit).  Always
 * returns 0 (single updater: no concurrent deletion); the int return matches
 * urcu_txn_list_del_prepare() for transition parity.
 */
static inline
int urcu_txn_sw_list_del_prepare(struct urcu_txn_sw_txn *txn,
		struct urcu_txn_sw_list_node *elem)
{
	struct urcu_txn_sw_list_node *prev = urcu_txn_sw_list_pending(txn, &elem->prev);
	struct urcu_txn_sw_list_node *next = urcu_txn_sw_list_pending(txn, &elem->next);

	/* prev->next: elem -> next ; next->prev: elem -> prev */
	(void) urcu_txn_sw_record_chain(txn, (void **) &prev->next, elem, next, URCU_TXN_SW_LIST_PROXY_TAG);
	(void) urcu_txn_sw_record_chain(txn, (void **) &next->prev, elem, prev, URCU_TXN_SW_LIST_PROXY_TAG);
	return 0;
}

/*
 * Remove @elem.  Its own next/prev are left intact (ghost) so a reader
 * standing on it can still escape in either direction; the caller frees
 * @elem after a grace period.  Convenience bracket around
 * urcu_txn_sw_list_del_prepare().
 */
static inline
int urcu_txn_sw_list_del_rcu(struct urcu_txn_sw_list_node *elem)
{
	struct urcu_txn_sw_txn txn;

	urcu_txn_sw_init(&txn);
	urcu_txn_sw_declare_disjoint(&txn);	/* single-op commit: distinct slots, no same-slot WAW */
	(void) urcu_txn_sw_reserve(&txn, 2);	/* sticky OOM -> commit reports it */
	(void) urcu_txn_sw_list_del_prepare(&txn, elem);
	return urcu_txn_sw_commit(&txn) < 0 ? -1 : 0;
}

/*
 * urcu_txn_sw_list_replace_prepare: record the in-place replacement of @old by
 * @newp into the caller-owned single-updater transaction @txn, WITHOUT
 * committing.  Composable form of replace (see add_after_prepare for the
 * contract): @newp inherits @old's neighbours; @old is left ghost for parked
 * readers.  Always returns 0.  Mirrors urcu_txn_list_replace_prepare()
 * (single-updater: no -ENOENT/-EAGAIN, since there is no concurrent deletion).
 */
static inline
int urcu_txn_sw_list_replace_prepare(struct urcu_txn_sw_txn *txn,
		struct urcu_txn_sw_list_node *old,
		struct urcu_txn_sw_list_node *newp)
{
	struct urcu_txn_sw_list_node *prev = urcu_txn_sw_list_pending(txn, &old->prev);
	struct urcu_txn_sw_list_node *next = urcu_txn_sw_list_pending(txn, &old->next);

	newp->prev = prev;
	newp->next = next;

	/* prev->next: old -> newp ; next->prev: old -> newp */
	(void) urcu_txn_sw_record_chain(txn, (void **) &prev->next, old, newp, URCU_TXN_SW_LIST_PROXY_TAG);
	(void) urcu_txn_sw_record_chain(txn, (void **) &next->prev, old, newp, URCU_TXN_SW_LIST_PROXY_TAG);
	return 0;
}

/*
 * Replace @old with @newp atomically with respect to RCU readers.  Convenience
 * bracket around urcu_txn_sw_list_replace_prepare().
 */
static inline
int urcu_txn_sw_list_replace_rcu(struct urcu_txn_sw_list_node *old,
		struct urcu_txn_sw_list_node *newp)
{
	struct urcu_txn_sw_txn txn;

	urcu_txn_sw_init(&txn);
	urcu_txn_sw_declare_disjoint(&txn);	/* single-op commit: distinct slots, no same-slot WAW */
	(void) urcu_txn_sw_reserve(&txn, 2);	/* sticky OOM -> commit reports it */
	(void) urcu_txn_sw_list_replace_prepare(&txn, old, newp);
	return urcu_txn_sw_commit(&txn) < 0 ? -1 : 0;
}

#define urcu_txn_sw_list_entry(ptr, type, member) \
	caa_container_of(ptr, type, member)

#define urcu_txn_sw_list_first_entry_rcu(head, type, member) \
	urcu_txn_sw_list_entry(urcu_txn_sw_list_next_rcu(&(head)->node), type, member)

#define urcu_txn_sw_list_last_entry_rcu(head, type, member) \
	urcu_txn_sw_list_entry(urcu_txn_sw_list_prev_rcu(&(head)->node), type, member)

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
	for (pos = urcu_txn_sw_list_entry(urcu_txn_sw_list_next_rcu(&(head)->node), \
			__typeof__(*(pos)), member); \
		&(pos)->member != &(head)->node; \
		pos = urcu_txn_sw_list_entry( \
			urcu_txn_sw_list_next_rcu(&(pos)->member), \
			__typeof__(*(pos)), member))

#define urcu_txn_sw_list_for_each_entry_reverse_rcu(pos, head, member) \
	for (pos = urcu_txn_sw_list_entry(urcu_txn_sw_list_prev_rcu(&(head)->node), \
			__typeof__(*(pos)), member); \
		&(pos)->member != &(head)->node; \
		pos = urcu_txn_sw_list_entry( \
			urcu_txn_sw_list_prev_rcu(&(pos)->member), \
			__typeof__(*(pos)), member))

#ifdef __cplusplus
}
#endif

#endif	/* _URCU_RCU_TXN_SW_LIST_H */
