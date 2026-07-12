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
 * first-position op, &prev->next otherwise).  The pprev edge is writer-only
 * bookkeeping.  Both edges are nonetheless recorded into the flip transaction
 * (<urcu/rcu-txn-sw.h>) so a mutation is all-or-nothing under an OOM abort and
 * composes into a larger cross-structure flip via the _prepare forms -- exactly
 * as the bidir sw-list records its two edges.
 *
 * Composition is limited to SLOT-DISJOINT edits: the sw engine has no
 * transactional loads and no same-slot reconcile, so each _prepare reads its
 * neighbour slots raw and cannot see a pending edit made earlier in the SAME
 * bracket.  Cross-structure composition (the intended use) is disjoint by
 * construction; two edits on ONE hlist whose neighbourhoods touch -- adjacent
 * deletes, say -- silently commit stale pointers.  See the trap worked out in
 * urcu_txn_sw_list_add_after_prepare(); use the concurrent front-end
 * (<urcu/rcu-txn-hlist.h>), whose read-your-own-writes chains them, for those.
 *
 * Configurable proxy tag (a compile-time define, never stored in the head)
 * ----------------------------------------------------------------------
 * Every slot of the hlist is transacted under URCU_TXN_SW_HLIST_TAG, the flip
 * proxy tag (<urcu/rcu-txn-sw.h>).  It is a compile-time define (default
 * URCU_MCAS_TAG, bit 0) rather than a per-call argument, so the head costs no
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
 * records the reader-visible next edge and the writer-only pprev edge and commits
 * them as one atomic flip; the transaction reclaims itself after a grace period
 * through call_rcu().  Include this header AFTER an RCU flavor header.  A mutator
 * returns 0 on success or -1 on allocation failure (the list is left unchanged).
 */

#include <stdlib.h>
#include <stdint.h>

#include <urcu/compiler.h>
#include <urcu/uatomic.h>
#include <urcu/call-rcu.h>		/* struct rcu_head */
#include <urcu/rcu-mcas.h>		/* URCU_MCAS_TAG default */
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
#define URCU_TXN_SW_HLIST_TAG	URCU_MCAS_TAG
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
	uintptr_t v = (uintptr_t) ptr;

	if (caa_unlikely((v & URCU_TXN_SW_HLIST_TAG) == URCU_TXN_SW_HLIST_TAG)) {
		struct urcu_txn_sw_proxy *proxy = (struct urcu_txn_sw_proxy *)
				(v & ~(uintptr_t) URCU_TXN_SW_HLIST_TAG);

		return (struct urcu_txn_sw_hlist_node *)
				urcu_txn_sw_proxy_get(proxy);
	}
	return ptr;
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
 * urcu_txn_sw_hlist_insert_at_slot_prepare: the core composable primitive.
 * Record the edges that make @slot name @newp, given @slot currently holds
 * @succ (the caller read it directly -- a single updater sees settled slots),
 * WITHOUT committing.  @slot is &head->first for insert-at-head or &pos->next
 * for insert-after.  Records the reader-visible *slot edge and, when @succ is
 * non-NULL, the writer-only &succ->pprev edge.  Always returns 0 (single
 * updater); the int return matches the concurrent variant for transition parity.
 *
 * Compose only over SLOT-DISJOINT edits -- @succ is read raw by the caller, so
 * it cannot reflect a pending edit made earlier in the same bracket.  See the
 * header intro and urcu_txn_sw_list_add_after_prepare().
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

	/* *slot: succ -> newp ; succ->pprev: slot -> &newp->next. */
	(void) urcu_txn_sw_record(txn, (void **) slot, succ, newp,
			URCU_TXN_SW_HLIST_TAG);
	if (succ != NULL)
		(void) urcu_txn_sw_record(txn, (void **) &succ->pprev, slot,
				&newp->next, URCU_TXN_SW_HLIST_TAG);
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
			&head->first, head->first);
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
			&pos->next, pos->next);
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
	struct urcu_txn_sw_hlist_node **slot = pos->pprev;

	newp->next = pos;
	newp->pprev = slot;

	/* *slot: pos -> newp ; pos->pprev: slot -> &newp->next. */
	(void) urcu_txn_sw_record(txn, (void **) slot, pos, newp,
			URCU_TXN_SW_HLIST_TAG);
	(void) urcu_txn_sw_record(txn, (void **) &pos->pprev, slot,
			&newp->next, URCU_TXN_SW_HLIST_TAG);
	return 0;
}

/*
 * urcu_txn_sw_hlist_del_prepare: composable form of del.  @elem between slot
 * *elem->pprev and next.  @elem's own next/pprev are left intact (ghost) so a
 * reader standing on it still escapes forward; the caller frees @elem after a
 * grace period (post-commit).  Always returns 0.
 */
static inline
int urcu_txn_sw_hlist_del_prepare(struct urcu_txn_sw_txn *txn,
		struct urcu_txn_sw_hlist_node *elem)
{
	struct urcu_txn_sw_hlist_node *next = elem->next;
	struct urcu_txn_sw_hlist_node **ppv = elem->pprev;

	/* *ppv: elem -> next ; next->pprev: &elem->next -> ppv (if next). */
	(void) urcu_txn_sw_record(txn, (void **) ppv, elem, next,
			URCU_TXN_SW_HLIST_TAG);
	if (next != NULL)
		(void) urcu_txn_sw_record(txn, (void **) &next->pprev,
				&elem->next, ppv, URCU_TXN_SW_HLIST_TAG);
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
	struct urcu_txn_sw_hlist_node *next = old->next;
	struct urcu_txn_sw_hlist_node **ppv = old->pprev;

	newp->next = next;
	newp->pprev = ppv;

	/* *ppv: old -> newp ; next->pprev: &old->next -> &newp->next (if next). */
	(void) urcu_txn_sw_record(txn, (void **) ppv, old, newp,
			URCU_TXN_SW_HLIST_TAG);
	if (next != NULL)
		(void) urcu_txn_sw_record(txn, (void **) &next->pprev,
				&old->next, &newp->next, URCU_TXN_SW_HLIST_TAG);
	return 0;
}

/*
 * Convenience brackets: each drives a fresh urcu_txn_sw_txn (reserve up to 2
 * edges -- the sw engine's single-edge fast path serves the 1-edge empty-bucket
 * insert and last-node delete), records the op, and commits.  Return 0 on
 * success, -1 on allocation failure.
 */
static inline
int urcu_txn_sw_hlist_add_head_rcu(struct urcu_txn_sw_hlist_node *newp,
		struct urcu_txn_sw_hlist_head *head)
{
	struct urcu_txn_sw_txn txn;

	urcu_txn_sw_init(&txn);
	(void) urcu_txn_sw_reserve(&txn, 2);
	(void) urcu_txn_sw_hlist_add_head_prepare(&txn, newp, head);
	return urcu_txn_sw_commit(&txn) < 0 ? -1 : 0;
}

static inline
int urcu_txn_sw_hlist_add_after_rcu(struct urcu_txn_sw_hlist_node *newp,
		struct urcu_txn_sw_hlist_node *pos)
{
	struct urcu_txn_sw_txn txn;

	urcu_txn_sw_init(&txn);
	(void) urcu_txn_sw_reserve(&txn, 2);
	(void) urcu_txn_sw_hlist_add_after_prepare(&txn, newp, pos);
	return urcu_txn_sw_commit(&txn) < 0 ? -1 : 0;
}

static inline
int urcu_txn_sw_hlist_add_before_rcu(struct urcu_txn_sw_hlist_node *newp,
		struct urcu_txn_sw_hlist_node *pos)
{
	struct urcu_txn_sw_txn txn;

	urcu_txn_sw_init(&txn);
	(void) urcu_txn_sw_reserve(&txn, 2);
	(void) urcu_txn_sw_hlist_add_before_prepare(&txn, newp, pos);
	return urcu_txn_sw_commit(&txn) < 0 ? -1 : 0;
}

static inline
int urcu_txn_sw_hlist_del_rcu(struct urcu_txn_sw_hlist_node *elem)
{
	struct urcu_txn_sw_txn txn;

	urcu_txn_sw_init(&txn);
	(void) urcu_txn_sw_reserve(&txn, 2);
	(void) urcu_txn_sw_hlist_del_prepare(&txn, elem);
	return urcu_txn_sw_commit(&txn) < 0 ? -1 : 0;
}

static inline
int urcu_txn_sw_hlist_replace_rcu(struct urcu_txn_sw_hlist_node *old,
		struct urcu_txn_sw_hlist_node *newp)
{
	struct urcu_txn_sw_txn txn;

	urcu_txn_sw_init(&txn);
	(void) urcu_txn_sw_reserve(&txn, 2);
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
