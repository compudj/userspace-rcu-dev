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
 *   membership   owner names a deque, and it cannot desynchronise from the
 *                links because it is not separately maintained;
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
 *
 * And because owner is that one witness, it is also where RECLAMATION is
 * expressed.  A caller that removes a node and frees it needs "removed, and no
 * push may ever queue it again" to be ONE decision; a remove followed by a
 * separate flag check is a window, not a guarantee.  So owner takes a third,
 * terminal value -- see URCU_TXN_DEQUE_POISON and remove_seal below.  Callers
 * that only ever push and remove can ignore it entirely, except for one thing:
 * ask membership with urcu_txn_deque_queued(), not `owner() != NULL`.
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
	unsigned long		    seq;	/* transacted slot; see below */
};

/*
 * seq -- the MEMBERSHIP SEQUENCE, and why `owner` alone is not enough.
 *
 * `owner` fixes the desync between membership and the links, because it is
 * written by the same commit that moves the edges.  It does NOT fix ABA: it
 * takes two values, so a transaction that reads owner == d, has the node
 * removed and re-added underneath it, and validates at commit, sees owner == d
 * again and accepts a derivation taken from a membership that no longer exists.
 *
 * seq is bumped by EVERY membership transition (push and remove) and never
 * decreases, so "unchanged since I read it" becomes decidable.  A remove
 * derives &prev->next from &n->prev and must therefore prove @prev is still
 * the member it read -- and it cannot do that by validating &prev->next,
 * because that is the very slot it stores to, and a validate plus a store on
 * one slot is the same-slot merge the list documents as corrupting.  seq is a
 * SEPARATE slot, which is the whole point: liveness can be checked inside the
 * conflict set without colliding with the edge being written.
 *
 * Bumped by 2, never 1: bit 0 is reserved for the engine's descriptor proxy
 * tag on every transacted slot, so the value must stay even.
 */
#define URCU_TXN_DEQUE_SEQ_STEP	2UL

/*
 * POISON -- the TERMINAL owner value, and the only state a node cannot leave.
 *
 * `owner` already gives push and remove a shared exclusion point: both CAS it,
 * so exactly one wins.  What it could not express is "and never again", because
 * its free value is NULL and NULL is exactly what a push wants.  A caller that
 * removes a node and then frees it therefore has a window it cannot close from
 * outside: between the remove's commit and the free, a concurrent push_tail may
 * legitimately find owner == NULL and queue storage already handed to call_rcu.
 * Checking some liveness flag OUTSIDE the commit does not close that -- it
 * narrows it, which is a different thing and reads the same in a five-run test.
 *
 * The seal closes it by giving `owner` a third value no push accepts:
 *
 *	urcu_txn_deque_remove_seal_prepare()   owner : d    -> POISON
 *	urcu_txn_deque_seal_prepare()          owner : NULL -> POISON
 *
 * Both CAS the slot every push CASes, so a seal racing a push is decided by the
 * engine, in one commit, exactly as two pushes are.  Afterwards
 * push_tail_prepare answers -ESTALE for ever.
 *
 * ⚠ THIS WIDENS WHAT `owner != NULL` MEANS.  It used to be exactly "queued";
 * it is now "queued OR sealed".  Code asking the MEMBERSHIP question must use
 * urcu_txn_deque_queued(), which maps POISON to NULL -- a loop shaped like
 * `while (owner(n)) remove(n);` spins for ever on a sealed node.
 *
 * ⚠ TERMINAL PER LIFETIME, NOT PER ADDRESS.  urcu_txn_deque_node_init() clears
 * it, so recycled storage starts unsealed -- which is both correct and the only
 * way to reuse a node.  Same rule `seq` has, for the same reason.
 *
 * The value is a compile-time constant that is never a real deque, with bit 0
 * clear because that bit is the engine's descriptor-proxy tag on every
 * transacted slot.  A constant rather than the address of some object,
 * deliberately: a static object defined in a header is per-translation-unit, so
 * two TUs would disagree about what "sealed" is.
 */
#define URCU_TXN_DEQUE_POISON	((struct urcu_txn_deque *) (uintptr_t) 0x100UL)

/*
 * The guard itself, compile-out-able so its load-bearingness can be MEASURED
 * rather than asserted.  A guard whose removal changes nothing is a guard that
 * the test does not exercise, and this project has shipped three rules whose
 * tests could not have failed.
 */
#ifdef URCU_TXN_DEQUE_NO_SEQ_GUARD
#define URCU_TXN_DEQUE_SEQ_GUARD(txn, node)	do { (void) (node); } while (0)
#else
#define URCU_TXN_DEQUE_SEQ_GUARD(txn, node)				\
	((void) urcu_txn_load_validate((txn), (void **) &(node)->seq,	\
			URCU_TXN_TAG))
#endif

struct urcu_txn_deque {
	struct urcu_txn_deque_node sentinel;	/* circular: next = head, prev = tail */
	unsigned long		   count;	/* APPROXIMATE; see below */
};

/*
 * count is maintained OUTSIDE the transaction by the CALLER, and is therefore
 * approximate.
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
	n->seq = 0;
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
 * Is @n queued, and on which deque?  A HINT, and only a hint.
 *
 * It is a plain resolved load, so it is a value some commit published -- but
 * nothing holds it still, and it is the ONE read in this header that is not
 * covered by a transaction.  A caller that branches on it is doing so
 * test-and-then-act: by the time it acts, a peer may have pushed or removed
 * @n.  That is SAFE only because every mutator re-derives membership inside
 * its own commit and answers -EEXIST / -ENOENT, so acting on a stale hint
 * costs a wasted attempt and never a wrong edge.
 *
 * DO NOT build a second membership record on top of it.  Caching this in a
 * separate word and maintaining that word alongside the deque re-creates
 * exactly what `owner` exists to eliminate -- two states no single commit
 * covers -- which is the defect this structure was written to replace.
 *
 * Call within an RCU read-side critical section (resolving a parked proxy
 * dereferences the writer's descriptor).
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
 * Has @n been SEALED?  Terminal once true (for this lifetime of the storage).
 *
 * Unlike the two accessors around it this one is NOT merely a hint in the
 * direction that matters: nothing clears POISON except node_init, so a true
 * answer stays true.  A false answer is a hint, as ever.
 */
static inline
int urcu_txn_deque_sealed(struct urcu_txn_deque_node *n)
{
	return urcu_txn_deque_owner(n) == URCU_TXN_DEQUE_POISON;
}

/*
 * WHICH DEQUE HOLDS @n, or NULL -- the MEMBERSHIP question, and the one almost
 * every caller actually wants.
 *
 * It exists because urcu_txn_deque_owner() stopped being able to answer it once
 * POISON was added: a sealed node has a non-NULL owner and belongs to no deque.
 * Use this wherever the old `owner(n) != NULL` meant "queued" -- notably in any
 * drain loop, where the raw accessor spins for ever on a sealed node.
 *
 * A HINT, exactly as urcu_txn_deque_owner() is, and for the same reason.
 */
static inline
struct urcu_txn_deque *urcu_txn_deque_queued(struct urcu_txn_deque_node *n)
{
	struct urcu_txn_deque *q = urcu_txn_deque_owner(n);

	return q == URCU_TXN_DEQUE_POISON ? NULL : q;
}

/*
 * PEEK at the oldest node, or NULL if empty.  One hop off the sentinel -- not
 * a traversal: the caller may not step from the result to its successor.
 *
 * A HINT, like urcu_txn_deque_owner() above and for the same reason: plain
 * load, nothing holds it, and the node may be removed (and, once a grace
 * period passes, freed) before the caller acts on it.  Hence the RCU read-side
 * requirement, which is what keeps the returned pointer dereferenceable; the
 * mutators then re-derive under their own commit.
 *
 * A sweeper must therefore treat "the head" as advisory and let its remove or
 * rotate answer authoritatively.  Re-reading this every iteration -- rather
 * than caching a cursor -- is also what keeps the no-traversal contract true.
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
	void *own, *nn, *np, *sq;
	int ret;

	own = urcu_txn_load(txn, (void **) &n->owner, URCU_TXN_TAG);
	/*
	 * SEALED is terminal and is NOT the same answer as "already queued":
	 * -EEXIST invites the caller to try again later, -ESTALE tells it the
	 * node will never accept a push again.  Both are terminal for THIS
	 * call, so both take the bracket's abandon path.
	 */
	if (caa_unlikely(own == (void *) URCU_TXN_DEQUE_POISON))
		return -ESTALE;			/* sealed; never again */
	if (own)
		return -EEXIST;			/* already on a deque */
	oldtail = urcu_txn_deque_resolve(
			urcu_txn_load(txn, (void **) &sent->prev, URCU_TXN_TAG));
	nn = urcu_txn_load(txn, (void **) &n->next, URCU_TXN_TAG);
	np = urcu_txn_load(txn, (void **) &n->prev, URCU_TXN_TAG);
	sq = urcu_txn_load(txn, (void **) &n->seq, URCU_TXN_TAG);
	/*
	 * @oldtail's identity came from &sent->prev and its &oldtail->next slot
	 * is written below, so its LINK is covered -- but that only proves the
	 * slot still holds the sentinel, not that @oldtail is the same
	 * membership we read.  Validate its seq, which is not a slot this
	 * transaction writes.
	 */
	if (oldtail != sent)
		URCU_TXN_DEQUE_SEQ_GUARD(txn, oldtail);

	ret = urcu_txn_store_mw(txn, (void **) &n->owner, NULL, d, URCU_TXN_TAG);
	ret |= urcu_txn_store_mw(txn, (void **) &n->seq, sq,
			(void *) ((uintptr_t) sq + URCU_TXN_DEQUE_SEQ_STEP),
			URCU_TXN_TAG);
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
 *
 * @newowner is the value `owner` lands on: NULL for a plain remove, or
 * URCU_TXN_DEQUE_POISON to remove AND seal in the SAME commit.  The two differ
 * in nothing else, which is the point -- a seal is not a second operation
 * layered on a remove, it is the same three-slot commit with one different
 * expected-new, so there is no instant in between at which a push could win.
 */
static inline
int urcu_txn_deque__remove_to(struct urcu_txn *txn,
		struct urcu_txn_deque *d, struct urcu_txn_deque_node *n,
		struct urcu_txn_deque *newowner)
{
	struct urcu_txn_deque_node *prev, *next;
	void *own, *sq;
	int ret;

	own = urcu_txn_load(txn, (void **) &n->owner, URCU_TXN_TAG);
	if (!own)
		return -ENOENT;			/* not queued */
	if (caa_unlikely(own == (void *) URCU_TXN_DEQUE_POISON))
		return -ESTALE;			/* sealed: not a member */
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

	sq = urcu_txn_load(txn, (void **) &n->seq, URCU_TXN_TAG);
	/*
	 * THE ABA GUARD.  @prev and @next are derived from @n's links, and this
	 * transaction writes &prev->next and &next->prev -- so their LINKS are
	 * covered, but a neighbour that was removed and re-added since the read
	 * would present the same link value and the same owner.  Their seqs are
	 * separate slots and monotone, so validating them makes "still the
	 * member I derived from" decidable.  Skipped for the sentinel, which is
	 * immortal and never transitions.
	 */
	if (prev != &d->sentinel)
		URCU_TXN_DEQUE_SEQ_GUARD(txn, prev);
	if (next != &d->sentinel && next != prev)
		URCU_TXN_DEQUE_SEQ_GUARD(txn, next);

	ret = urcu_txn_store_mw(txn, (void **) &n->owner, d, newowner,
			URCU_TXN_TAG);
	ret |= urcu_txn_store_mw(txn, (void **) &n->seq, sq,
			(void *) ((uintptr_t) sq + URCU_TXN_DEQUE_SEQ_STEP),
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

static inline
int urcu_txn_deque_remove_prepare(struct urcu_txn *txn,
		struct urcu_txn_deque *d, struct urcu_txn_deque_node *n)
{
	return urcu_txn_deque__remove_to(txn, d, n, NULL);
}

/*
 * REMOVE @n from @d AND SEAL IT, in one commit: owner : d -> POISON.
 *
 * This is what a caller that is about to FREE @n wants, and the reason it must
 * be one commit rather than a remove followed by a seal is the instant in
 * between: there, owner is NULL and a concurrent push_tail is entitled to win
 * it.  Sealing is therefore an argument to the remove, not a second operation.
 *
 * After this returns 0 the node will never be queued again (until node_init),
 * so the caller may hand it to call_rcu knowing no deque can come to name it.
 *
 * Returns 0, -ENOENT if @n is not queued or is queued elsewhere, -ESTALE if it
 * was already sealed (terminal, and NOT an error for a caller that only wants
 * the postcondition), or -ENOMEM.
 */
static inline
int urcu_txn_deque_remove_seal_prepare(struct urcu_txn *txn,
		struct urcu_txn_deque *d, struct urcu_txn_deque_node *n)
{
	return urcu_txn_deque__remove_to(txn, d, n, URCU_TXN_DEQUE_POISON);
}

/*
 * SEAL a node that is NOT on any deque: owner : NULL -> POISON.  One slot.
 *
 * The companion to remove_seal for the other half of a caller's kill path --
 * the node was never queued, or a peer removed it first.  Racing a push is
 * decided by this single CAS: if the push wins, this answers -EEXIST and the
 * caller re-reads and removes-and-seals instead; if this wins, the push answers
 * -ESTALE.  There is no third outcome, which is what makes a kill path
 * expressible as a bounded loop.
 *
 * NO SEQ BUMP, deliberately.  seq is bumped by every MEMBERSHIP transition so
 * that a peer holding a derivation can tell the membership changed -- and a
 * node that was already a non-member is in nobody's derivation.  Sealing it is
 * a transition of the node's fate, not of its membership.
 *
 * Returns 0, -EEXIST if @n is currently queued (use remove_seal), -ESTALE if it
 * was already sealed, or -ENOMEM.
 */
static inline
int urcu_txn_deque_seal_prepare(struct urcu_txn *txn,
		struct urcu_txn_deque_node *n)
{
	void *own = urcu_txn_load(txn, (void **) &n->owner, URCU_TXN_TAG);

	if (caa_unlikely(own == (void *) URCU_TXN_DEQUE_POISON))
		return -ESTALE;			/* already sealed */
	if (own)
		return -EEXIST;			/* queued: remove_seal it */
	return urcu_txn_store_mw(txn, (void **) &n->owner, NULL,
			URCU_TXN_DEQUE_POISON, URCU_TXN_TAG) ? -ENOMEM : 0;
}

/*
 * UNSEAL, for storage being recycled.  Not a concurrent operation: the caller
 * must already know no peer can reach @n -- which is exactly what having just
 * reclaimed it after a grace period establishes.  urcu_txn_deque_node_init()
 * does the same thing and more; this exists for callers that reuse a node in
 * place and want to say what they mean.
 */
static inline
void urcu_txn_deque_node_unseal(struct urcu_txn_deque_node *n)
{
	uatomic_store((void **) &n->owner, NULL, CMM_RELAXED);
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

	/*
	 * A rotate changes no node's MEMBERSHIP, so it bumps no seq -- keeping
	 * it from needlessly aborting peers.  But @hn and @t are derived
	 * identities whose slots it writes, so guard them the same way.
	 */
	URCU_TXN_DEQUE_SEQ_GUARD(txn, hn);
	if (t != hn)
		URCU_TXN_DEQUE_SEQ_GUARD(txn, t);

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
int urcu_txn_deque_remove_seal(struct urcu_txn_deque *d,
		struct urcu_txn_deque_node *n,
		struct urcu_txn_domain *domain)
{
	URCU_TXN_DEQUE_BRACKET(remove_seal,
		urcu_txn_deque_remove_seal_prepare(&txn, d, n))
}

static inline
int urcu_txn_deque_seal(struct urcu_txn_deque_node *n,
		struct urcu_txn_domain *domain)
{
	URCU_TXN_DEQUE_BRACKET(seal,
		urcu_txn_deque_seal_prepare(&txn, n))
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
