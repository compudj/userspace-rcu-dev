/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * rcu-txn-deque.h -- a concurrent deque with O(1) interior removal, and
 * DELIBERATELY NO READ-SIDE TRAVERSAL.
 *
 * WHY THIS IS NOT rcu-txn-list.h
 *
 * That header offers RCU traversal (urcu_txn_list_next_rcu, _prev_rcu,
 * _empty) and, at one point, a move-to-tail.  Those two features cannot be
 * used by the same caller, and the type system did not say so.  A traverser
 * standing on a moved node follows its NEW next and silently skips or repeats
 * an arbitrary span -- and that is not a race to be fixed with barriers:
 * grace periods govern RECLAMATION, not logical position, so there is no
 * instant at which the old next becomes safe again.  The only RCU-correct
 * relocation is copy-publish-retire, at an allocation and a grace period per
 * move.  (Mainline's dentry_lru_isolate does list_move_tail under lru_lock;
 * what makes it safe there is exclusion, not RCU.)
 *
 * So a structure may offer relocation, or it may offer traversal.  This one
 * offers relocation.  There are no traversal accessors here, on purpose, and
 * none should be added: adding one silently breaks every rotate.
 *
 * WHAT RCU STILL PROTECTS -- and it is not what you expect.
 *
 * There are no readers, but the MUTATORS read a node's neighbours and then CAS
 * into them, so a neighbour freed in between is a use-after-free.  Every
 * mutator must therefore run inside an RCU read-side critical section, and node
 * memory must be freed via call_rcu.  RCU here protects the MUTATORS' neighbour
 * pointers, not readers.
 *
 * THE DESIGN, in one field.
 *
 * A node carries `owner`, a POINTER to the deque holding it -- pointer-width,
 * therefore an MCAS slot, therefore written IN THE SAME COMMIT as the link
 * edges.  That single property collapses three jobs a separate membership word
 * cannot do at once:
 *
 *   membership   owner != NULL, and it cannot desynchronise from the links
 *                because it is not separately maintained;
 *   identity     the pointer IS the deque, so a node knows which one it is on;
 *   exclusion    the commit is the exclusion.  No claim protocol, no BUSY
 *                state: two concurrent pushes of one node both CAS
 *                &n->owner : NULL -> D, the loser aborts, retries, sees
 *                non-NULL and reports -EEXIST.
 *
 * Consequences worth stating because they are what the list could not offer:
 *
 *   - NO DELETION MARK.  owner is the membership witness, so nothing marks
 *     next, and link slots carry no tag but the engine's own proxy.  The
 *     raw-versus-resolved distinction on a link value disappears with it.
 *   - NO PLAIN STORES.  Every edge is a store_mw against a validated
 *     expected-old.  The list's insert wrote newp->next/newp->prev plainly at
 *     prepare time, un-CAS'd and not undone on abort, which is how a re-add
 *     could overwrite a live node's edges.  Nothing here can.
 *   - Therefore the invariant holds unconditionally: for every queued node n,
 *     n->prev->next == n and n->next->prev == n.  `prev` is TRUTH, not a hint,
 *     so no rescan or hint repair is needed anywhere.
 *
 * That last one is only true because remove() LOAD-VALIDATES the reads it
 * derives its slots from; see the comment there.  It held vacuously until a
 * two-writer test caught it, so do not weaken those reads to plain loads.
 *
 * Nodes are NOT reset on removal: next/prev keep stale values, which is safe
 * because nothing dereferences them (a later push only reads and CASes them
 * away) and it saves two slots per removal.  owner is the only witness.
 */

#ifndef _URCU_RCU_TXN_DEQUE_H
#define _URCU_RCU_TXN_DEQUE_H

#include <errno.h>
#include <stdint.h>

#include <urcu/compiler.h>
#include <urcu/uatomic.h>
#include <urcu/rcu-txn-mcas.h>
#include <urcu/rcu-txn.h>

#ifdef __cplusplus
extern "C" {
#endif

struct urcu_txn_deque;

struct urcu_txn_deque_node {
	struct urcu_txn_deque_node *next;	/* transacted slot */
	struct urcu_txn_deque_node *prev;	/* transacted slot */
	struct urcu_txn_deque      *owner;	/* transacted slot; NULL = free */
};

struct urcu_txn_deque {
	struct urcu_txn_deque_node sentinel;	/* circular: next = head, prev = tail */
	unsigned long		   count;	/* APPROXIMATE; see below */
};

/*
 * count is maintained OUTSIDE the transaction and is therefore approximate.
 * That is deliberate: it is not pointer-width, so transacting it is not
 * possible, and making membership depend on a separately-maintained word again
 * is precisely the mistake this structure exists to remove.  Read it as a scan
 * budget, never as truth.
 */

static inline
void urcu_txn_deque_init(struct urcu_txn_deque *d)
{
	d->sentinel.next = &d->sentinel;
	d->sentinel.prev = &d->sentinel;
	d->sentinel.owner = NULL;
	d->count = 0;
}

static inline
void urcu_txn_deque_node_init(struct urcu_txn_deque_node *n)
{
	n->next = NULL;
	n->prev = NULL;
	n->owner = NULL;
}

/* Resolve a raw slot value: only the engine's proxy tag can be set here. */
static inline
struct urcu_txn_deque_node *urcu_txn_deque_resolve(void *raw)
{
	if (caa_unlikely((uintptr_t) raw & URCU_TXN_TAG))
		return (struct urcu_txn_deque_node *)
				urcu_txn_resolve(raw, URCU_TXN_TAG);
	return (struct urcu_txn_deque_node *) raw;
}

/*
 * Is @n queued, and on which deque?  A plain resolved load -- correct without
 * a transaction because owner only ever changes inside one, so any value read
 * is a value some commit published.  Call within an RCU read-side critical
 * section (resolving a parked proxy dereferences the writer's descriptor).
 */
static inline
struct urcu_txn_deque *urcu_txn_deque_owner(struct urcu_txn_deque_node *n)
{
	void *raw = uatomic_load((void **) &n->owner, CMM_ACQUIRE);

	if (caa_unlikely((uintptr_t) raw & URCU_TXN_TAG))
		return (struct urcu_txn_deque *) urcu_txn_resolve(raw,
								 URCU_TXN_TAG);
	return (struct urcu_txn_deque *) raw;
}

/*
 * PEEK at the oldest node, or NULL if empty.  One hop off the sentinel -- this
 * is the only read this structure offers and it is not a traversal: the caller
 * may not step from the result to its successor.  Call within an RCU read-side
 * critical section.
 */
static inline
struct urcu_txn_deque_node *urcu_txn_deque_head(struct urcu_txn_deque *d)
{
	struct urcu_txn_deque_node *h = urcu_txn_deque_resolve(
			uatomic_load((void **) &d->sentinel.next, CMM_ACQUIRE));

	return h == &d->sentinel ? NULL : h;
}

static inline
int urcu_txn_deque_empty(struct urcu_txn_deque *d)
{
	return urcu_txn_deque_head(d) == NULL;
}

/*
 * PUSH AT TAIL -- five slots:
 *
 *	&n->owner	  : NULL	-> d		(guard AND membership)
 *	&n->next	  : <read>	-> &sentinel
 *	&n->prev	  : <read>	-> oldtail
 *	&oldtail->next	  : &sentinel	-> n
 *	&sentinel.prev	  : oldtail	-> n
 *
 * The node's own two links are TRANSACTED, not plain-stored.  It would be
 * tempting to write them plainly since owner == NULL says nobody else owns the
 * node -- but that is only true once the commit SUCCEEDS, and a losing peer
 * that had already plain-written them would have corrupted a node it does not
 * own.  That exact reasoning error is what the list shipped.
 *
 * Returns 0, -EEXIST if @n is already queued (terminal, nothing recorded), or
 * -ENOMEM (sticky to the commit).  Do NOT declare the handle disjoint when
 * @oldtail is the sentinel: &oldtail->next and &sentinel.prev are then two
 * slots of one object and the empty-deque case aliases nothing, but a caller
 * composing this with other records cannot know that.
 */
static inline
int urcu_txn_deque_push_tail_prepare(struct urcu_txn *txn,
		struct urcu_txn_deque *d, struct urcu_txn_deque_node *n)
{
	struct urcu_txn_deque_node *sent = &d->sentinel, *oldtail;
	void *own, *nn, *np;
	int ret;

	own = urcu_txn_load(txn, (void **) &n->owner, URCU_TXN_TAG);
	if (own)
		return -EEXIST;			/* already on a deque */
	oldtail = urcu_txn_deque_resolve(
			urcu_txn_load(txn, (void **) &sent->prev, URCU_TXN_TAG));
	nn = urcu_txn_load(txn, (void **) &n->next, URCU_TXN_TAG);
	np = urcu_txn_load(txn, (void **) &n->prev, URCU_TXN_TAG);

	ret = urcu_txn_store_mw(txn, (void **) &n->owner, NULL, d, URCU_TXN_TAG);
	ret |= urcu_txn_store_mw(txn, (void **) &n->next, nn, sent,
			URCU_TXN_TAG);
	ret |= urcu_txn_store_mw(txn, (void **) &n->prev, np, oldtail,
			URCU_TXN_TAG);
	/*
	 * When the deque is EMPTY, oldtail == sent, so &oldtail->next is
	 * &sent->next and &sent->prev is a different slot -- still distinct.
	 * When it is non-empty they are distinct objects.  Either way no two
	 * records land on one slot, so this never needs the reconcile path.
	 */
	ret |= urcu_txn_store_mw(txn, (void **) &oldtail->next, sent, n,
			URCU_TXN_TAG);
	ret |= urcu_txn_store_mw(txn, (void **) &sent->prev, oldtail, n,
			URCU_TXN_TAG);
	return ret ? -ENOMEM : 0;
}

/*
 * REMOVE @n from @d -- three slots:
 *
 *	&n->owner	: d	-> NULL		(guard AND membership)
 *	&prev->next	: n	-> next
 *	&next->prev	: n	-> prev
 *
 * No mark and no successor guard: owner is the witness, so a peer that removed
 * @n first simply makes this one's owner CAS fail, and the retry reads NULL and
 * answers -ENOENT.  And because prev is truth rather than a hint -- every write
 * to a link slot is a CAS against the exact prior state -- there is no rescan.
 *
 * Returns 0, -ENOENT if @n is not queued (terminal; the caller must NOT
 * reclaim, a peer owns that), or -ENOMEM.
 */
static inline
int urcu_txn_deque_remove_prepare(struct urcu_txn *txn,
		struct urcu_txn_deque *d, struct urcu_txn_deque_node *n)
{
	struct urcu_txn_deque_node *prev, *next;
	void *own;
	int ret;

	own = urcu_txn_load(txn, (void **) &n->owner, URCU_TXN_TAG);
	if (!own)
		return -ENOENT;			/* not queued */
	if ((struct urcu_txn_deque *) own != d)
		return -ENOENT;			/* queued elsewhere */
	/*
	 * LOAD-VALIDATE, not load.  These two reads DERIVE the slots this
	 * transaction writes, but neither slot is written by it -- so without a
	 * validate record nothing aborts when a peer changes them under us, and
	 * the commit installs edges computed from a state that no longer exists.
	 *
	 * Concretely: remove(P) rewrites &n->prev (its own &next->prev edge)
	 * while remove(n) is deriving @prev from that same slot.  The two share
	 * no WRITTEN slot -- remove(n) writes &n->owner, &prev->next and
	 * &next->prev -- so both commit, and remove(n) records
	 * &prev->next : n -> next against a P that has just left the deque.
	 * That CAS can even SUCCEED, because a removed node's `next` is never
	 * reset and later removes keep maintaining it, which builds a ghost
	 * chain off a non-member.  Worse, remove propagates its own @prev into
	 * &next->prev, so ONE stale read poisons every successor after it and
	 * the deque never recovers: the victim ends up owned but unreachable,
	 * and its remove retries forever inside the escalation lane.
	 *
	 * Validating both makes the derivation part of the transaction's
	 * conflict set, which is what keeps `prev` truth rather than a hint.
	 */
	prev = urcu_txn_deque_resolve(
			urcu_txn_load_validate(txn, (void **) &n->prev,
					URCU_TXN_TAG));
	next = urcu_txn_deque_resolve(
			urcu_txn_load_validate(txn, (void **) &n->next,
					URCU_TXN_TAG));

	ret = urcu_txn_store_mw(txn, (void **) &n->owner, d, NULL,
			URCU_TXN_TAG);
	/*
	 * A single-element deque has prev == next == sentinel, so these two
	 * records are &sent->next and &sent->prev -- distinct slots.  A
	 * two-element deque has prev == sentinel and next == the other node, or
	 * the mirror; also distinct.  No aliasing case puts two records with
	 * different expected-olds on one slot.
	 */
	ret |= urcu_txn_store_mw(txn, (void **) &prev->next, n, next,
			URCU_TXN_TAG);
	ret |= urcu_txn_store_mw(txn, (void **) &next->prev, n, prev,
			URCU_TXN_TAG);
	return ret ? -ENOMEM : 0;
}

/*
 * ROTATE THE HEAD TO THE TAIL -- six slots, h = head, t = tail:
 *
 *	&sentinel.next	: h		-> h->next
 *	&h->next->prev	: h		-> &sentinel
 *	&h->next	: h->next	-> &sentinel
 *	&h->prev	: &sentinel	-> t
 *	&t->next	: &sentinel	-> h
 *	&sentinel.prev	: t		-> h
 *
 * HEAD-TO-TAIL ONLY, not a general move, and that restriction is the point.
 * The head's predecessor is ALWAYS the sentinel, so the several-independent-
 * loads hazard that a general move has -- where @prev, @next and @oldtail can
 * describe mutually inconsistent list states and alias each other -- shrinks to
 * two reads off one object plus one off the head.  The inconsistency that
 * remains is detectable and is rejected below rather than committed.
 *
 * Returns 0 (rotated, or a no-op when the deque is empty or holds one node),
 * -EAGAIN if the reads disagree (retry), or -ENOMEM.
 */
static inline
int urcu_txn_deque_rotate_head_prepare(struct urcu_txn *txn,
		struct urcu_txn_deque *d)
{
	struct urcu_txn_deque_node *sent = &d->sentinel, *h, *t, *hn;
	int ret;

	h = urcu_txn_deque_resolve(
			urcu_txn_load(txn, (void **) &sent->next, URCU_TXN_TAG));
	if (h == sent)
		return 0;			/* empty */
	t = urcu_txn_deque_resolve(
			urcu_txn_load(txn, (void **) &sent->prev, URCU_TXN_TAG));
	if (h == t)
		return 0;			/* single node: already the tail */
	hn = urcu_txn_deque_resolve(
			urcu_txn_load(txn, (void **) &h->next, URCU_TXN_TAG));
	/*
	 * hn == sent says the head is also the tail, which h != t just denied.
	 * The two statements cannot both describe one deque, so the loads
	 * straddled a concurrent mutation.  Reject it: committing on
	 * inconsistent reads is how a general move puts two records with
	 * different expected-olds on &sentinel.prev.
	 */
	if (hn == sent)
		return -EAGAIN;

	ret = urcu_txn_store_mw(txn, (void **) &sent->next, h, hn,
			URCU_TXN_TAG);
	ret |= urcu_txn_store_mw(txn, (void **) &hn->prev, h, sent,
			URCU_TXN_TAG);
	ret |= urcu_txn_store_mw(txn, (void **) &h->next, hn, sent,
			URCU_TXN_TAG);
	ret |= urcu_txn_store_mw(txn, (void **) &h->prev, sent, t,
			URCU_TXN_TAG);
	ret |= urcu_txn_store_mw(txn, (void **) &t->next, sent, h,
			URCU_TXN_TAG);
	ret |= urcu_txn_store_mw(txn, (void **) &sent->prev, t, h,
			URCU_TXN_TAG);
	return ret ? -ENOMEM : 0;
}

/*
 * Convenience brackets.  Each owns its handle, its domain and its retry loop.
 * NONE declares the handle disjoint: the prepares above are proved free of
 * same-slot aliasing, but a disjoint handle blind-appends, and the proof
 * depends on reads that a composing caller may have already moved.
 */
#define URCU_TXN_DEQUE_BRACKET(name, call)				\
	struct urcu_txn txn;						\
	int prep;							\
	enum urcu_txn_status st;					\
									\
	urcu_txn_init(&txn, domain);					\
	for (;;) {							\
		urcu_txn_begin(&txn);					\
		prep = (call);						\
		if (prep && prep != -EAGAIN) {				\
			/*						\
			 * NOT urcu_txn_conflict(): a terminal answer is	\
			 * not a contention event.  push_tail reports	\
			 * -EEXIST on every duplicate and remove reports	\
			 * -ENOENT on every absent node, so counting those \
			 * as conflicts inflates the handle's aging on a	\
			 * large fraction of all calls and drives the	\
			 * domain into its fallback lane permanently.	\
			 */						\
			/*						\
			 * TERMINAL BAIL.  An escalated handle KEEPS its	\
			 * lane across end() so a re-attempt does not go to \
			 * the back of the FIFO -- so a path that does NOT  \
			 * re-attempt must abandon first, or the domain's   \
			 * lane is held forever and every other writer	\
			 * parks behind it.  push_tail answers -EEXIST on   \
			 * every duplicate, which is a hot path, so this is \
			 * not a corner case: without it, two writers	\
			 * deadlock within milliseconds.		\
			 */						\
			urcu_txn_abandon(&txn);				\
			urcu_txn_end(&txn);				\
			return prep;					\
		}							\
		if (prep == -EAGAIN) {					\
			urcu_txn_conflict(&txn);			\
			urcu_txn_end(&txn);				\
			continue;					\
		}							\
		st = urcu_txn_commit(&txn);				\
		urcu_txn_end(&txn);					\
		if (st == URCU_TXN_STATUS_ABORT)			\
			continue;					\
		return st == URCU_TXN_STATUS_OK ? 0 : -ENOMEM;		\
	}

static inline
int urcu_txn_deque_push_tail(struct urcu_txn_deque *d,
		struct urcu_txn_deque_node *n,
		struct urcu_txn_domain *domain)
{
	URCU_TXN_DEQUE_BRACKET(push,
		urcu_txn_deque_push_tail_prepare(&txn, d, n))
}

static inline
int urcu_txn_deque_remove(struct urcu_txn_deque *d,
		struct urcu_txn_deque_node *n,
		struct urcu_txn_domain *domain)
{
	URCU_TXN_DEQUE_BRACKET(remove,
		urcu_txn_deque_remove_prepare(&txn, d, n))
}

static inline
int urcu_txn_deque_rotate_head(struct urcu_txn_deque *d,
		struct urcu_txn_domain *domain)
{
	URCU_TXN_DEQUE_BRACKET(rotate,
		urcu_txn_deque_rotate_head_prepare(&txn, d))
}

#ifdef __cplusplus
}
#endif

#endif /* _URCU_RCU_TXN_DEQUE_H */
