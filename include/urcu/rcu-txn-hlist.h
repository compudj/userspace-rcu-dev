// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

#ifndef _URCU_RCU_TXN_HLIST_H
#define _URCU_RCU_TXN_HLIST_H

/*
 * rcu-txn-hlist: a kernel-hlist-shaped, single-pointer-head RCU list with
 * concurrent writers, built on the RCU MCAS engine (<urcu/rcu-txn-engine.h>).  It is
 * the hash-bucket sibling of the circular bidirectional <urcu/rcu-txn-list.h>:
 * where that list embeds a full sentinel node (16 B: next+prev) as its head,
 * an hlist head is a SINGLE pointer (8 B), so a table of buckets is half the
 * footprint and packs twice as many heads per cache line.  See the design note
 * design/rcu-txn-hlist.md in the benchmark tree for the rationale.
 *
 * pprev encoding -- why a 1-pointer head needs no head special case
 * ----------------------------------------------------------------
 * A node's backward link is NOT a node pointer (as in the bidir list) but a
 * pointer to the SLOT that names the node -- either &prev->next, or the bucket
 * head slot &head->first itself:
 *
 *   Node: { struct urcu_txn_hlist_node *next;    (node ptr; transacted; MARK-able)
 *           struct urcu_txn_hlist_node **pprev;  (slot ptr; transacted) }
 *
 * Slots are the engine's currency, so with pprev the head's single pointer is
 * just another "next" slot, handled uniformly with interior slots -- there is
 * no head node and no head special case in any operation.  A first-node delete
 * CASes &head->first exactly as an interior delete CASes &prev->next, and an
 * insert-at-head CASes &head->first exactly as insert-after CASes &pos->next.
 * This uniformity is exactly why hlist gets away with a 1-pointer head.
 *
 * Configurable proxy tag (a compile-time define, never stored in the head)
 * ----------------------------------------------------------------------
 * Every slot of the hlist -- the bucket head-first slot AND every node
 * next/pprev slot -- is transacted under URCU_TXN_HLIST_TAG, the engine proxy
 * tag (see <urcu/rcu-txn-engine.h>).  It is a compile-time define (default
 * URCU_TXN_TAG, bit 0) rather than a per-call argument, so the head costs no
 * extra storage and call sites stay kernel-terse, and rather than a hard-coded
 * constant so an embedder whose head lives in a slot it already transacts under
 * its OWN tag can compile the chain under that tag: the fractal trie tags its
 * polymorphic child slots with a low-nibble pattern (not bit 0), and the
 * external-node duplicate chain's head IS such a slot, so it does
 *     #define URCU_TXN_HLIST_TAG   FT_SLOT_TAG
 * before including this header.  Every function here is static inline, so
 * translation units that pick different tags (a bit-0 hash table and the
 * nibble-tagged trie) coexist in one binary with no ODR clash.  A single TU
 * cannot mix two hlist tags -- no consumer needs to.
 *
 * URCU_TXN_HLIST_TAG must satisfy the engine's per-record contract for EVERY
 * live value any hlist slot holds -- a (possibly MARK-ed) node "next", a "pprev"
 * slot address, the head-first pointer, and NULL: (value & TAG) != TAG.  Holds
 * for bit 0 and for the fractal trie's nibble tag on >= 4-byte-aligned nodes.
 *
 * Bit encoding: a "next" value carries an optional deletion MARK on bit 1 (see
 * URCU_TXN_HLIST_MARK; matches <urcu/rcu-txn-list.h>).  A marked live "next" is
 * never mistaken for a proxy: ((n | MARK) & TAG) != TAG for every live n (with
 * bit-0 TAG immediate; with a wider TAG whose bits include MARK it still holds
 * as long as an aligned node never sets all of TAG's bits).  A pprev VALUE is
 * the address of a word-aligned pointer field, never marked (no operation
 * reaches a node ONLY through pprev), only proxy-resolved.
 *
 * Operations (each one MCAS commit; edge counts as in the bidir list, +/- 1)
 * -------------------------------------------------------------------------
 *   insert-at-slot(new, slot, succ)  -- the core primitive; *slot: succ -> new
 *                                       is the serializing store.
 *     - *slot          : succ         -> new              (edge 1)
 *     - &succ->pprev   : slot         -> &new->next       (edge 2; only if succ)
 *     Into an empty slot (succ == NULL) edge 2 vanishes: a 1-edge insert.
 *     insert-at-head is insert-at-slot(new, &head->first, first); insert-after
 *     is insert-at-slot(new, &pos->next, pos->next).
 *   del(elem)                        -- elem between slot *elem->pprev and next:
 *     - &elem->next    : next         -> MARK(next)       (logical delete)
 *     - *elem->pprev   : elem         -> next             (unlink forward)
 *     - &next->pprev   : &elem->next  -> elem->pprev      (unlink backward; only
 *                                                          if next)
 *     When elem is last (next == NULL) the third edge vanishes: a 2-edge delete,
 *     cheaper than the circular list's invariable 3.  Its first edge then stores
 *     MARK(NULL) -- legal; readers strip the mark per hop and terminate on NULL.
 *   replace(old, new)                -- del(old)'s three slots, new values.
 *
 * Why a "next"-only mark is enough (insert/delete coherence)
 * ---------------------------------------------------------
 * Identical to <urcu/rcu-txn-list.h>; transferred verbatim.  Two things detect
 * a racing deletion, and only ONE is the mark:
 *
 *  (1) A NEIGHBOUR was deleted -- handled by a structural slot conflict, NOT by
 *      the mark.  Removing a node X rewrites the SINGLE forward slot that names
 *      X (*X->pprev, i.e. &prev->next or &head->first).  An insert placed next
 *      to X, or a delete of X's predecessor, rewrites that SAME slot with the
 *      SAME expected old value, so the MCAS commits at most one of them; the
 *      loser fails its old-value check, aborts, re-reads and proceeds.  The
 *      head slot participates in exactly this way (it is named by a pprev value
 *      and CAS'd by inserts and first-node deletes just like an interior
 *      &prev->next) -- that is the single new lemma over the bidir list, and it
 *      is trivial.
 *
 *  (2) The ANCHOR ITSELF was deleted -- the mark's one and only job.  An insert
 *      or delete handed a @pos/@elem that a peer is deleting loses the shared-slot
 *      race in (1); on retry it must tell "neighbour moved, re-read" apart from
 *      "my anchor is gone, give up".  It re-reads the anchor's next, sees the
 *      mark, and returns -ENOENT.
 *
 * pprev is never marked for the same reason prev is not in the bidir list: no
 * operation reaches a node ONLY through pprev, so the forward "next" slots
 * already serialize every adjacency.  Unlike the bidir list there is no
 * next == prev collision case to skip (the "predecessor" of the first node is
 * the bare head slot, not a node whose &next->next could coincide with a write
 * slot), so del/replace need no such guard skip.
 *
 * Why pprev stays transacted here (the SW hlist took it out; that does not
 * transfer)
 * ------------------------------------------------------------------------
 * The single-updater <urcu/rcu-txn-sw-hlist.h> writes pprev with an eager plain
 * store, on the grounds that it is writer-only.  Do not mirror that here.  The
 * premise splits under concurrency: with one updater "writer-only" means
 * PRIVATE, while here it still means shared, concurrently read by peer
 * mutators, and lifetime-critical.
 *
 * The prize is real and was weighed.  The load-validate of succ->next in
 * urcu_txn_hlist_insert_at_slot_prepare() exists ONLY to guard the pprev write;
 * drop that write and the guard goes with it, leaving *slot alone to serialize
 * against del(succ) (same slot, same expected old).  An interior insert would
 * fall to nr == 1 and take the engine's bare-CAS commit -- no descriptor, no
 * proxy, no call_rcu.  Two things forbid it, and every pprev store below is on
 * an already-PUBLISHED node (&succ->pprev, &pos->pprev, &next->pprev; the fresh
 * node's own links are already plain stores, built invisibly), so both bite:
 *
 *  (1) A lost CAS is the normal protocol outcome, not a corner.  The SW header
 *      closes its plain store against OOM with an up-front reserve(); no
 *      reserve can close an abort.  A pre-commit plain store survives the abort
 *      and leaves a live node naming a slot inside a node that never linked.
 *
 *  (2) Storing post-commit instead (winner only) dodges (1) and opens a repair
 *      window, and in that window a stale pprev is not a slow hint but a
 *      DANGLING POINTER: pprev holds an address INSIDE another node.  Staleness
 *      is detectable (*pprev != elem) and repairable by a short forward walk,
 *      but hints accumulate on one slot -- insert A at &prev->next, then B at
 *      &prev->next, and A->pprev and B->pprev both name it while A's true slot
 *      is &B->next.  Unboundedly many nodes' pprev can name a single slot, and
 *      del(prev) can only fix its CURRENT successor's.  prev is freed after a
 *      grace period and the rest dangle.  RCU does not cover this: the pointer
 *      sits in a live node indefinitely, so no read-side section contains it.
 *
 * This is what the SW header's insistence on an EAGER store is really about --
 * a plain store is instantly visible to the sole updater, so no stale window
 * exists at all.  The MW equivalent of "eager" is "atomic with the commit",
 * which is the transaction.  del's &next->pprev edge is harder still: &elem->next
 * dies with elem, so it must swing before reclaim at any price.
 *
 * The alternative, if the bare-CAS insert is ever worth its cost: drop pprev
 * ENTIRELY and have del take the head, walking to find the naming slot.  Insert
 * becomes a 1-edge bare CAS, del 2 edges, and the node halves to one pointer.
 * It costs the "@elem need not know its bucket" contract of del/replace, which
 * for short hash buckets may well be the better trade.  Softening pprev to a
 * hint is not on that menu -- see (2).
 *
 * Reclaim, read/write contract, escalation
 * ----------------------------------------
 * As in <urcu/rcu-txn-list.h>: del() returns whether THIS call removed the node
 * (so two concurrent deletes cannot double-free); the node is the caller's to
 * reclaim after a grace period.  Mutators commit through <urcu/rcu-txn.h>, which
 * opens an RCU read-side section per attempt and defers descriptor reclaim
 * through the flavor's call_rcu; include this header AFTER an RCU flavor.  A
 * mutator loops internally until it commits or definitively fails: 0/1 on
 * success, -ENOENT if the anchor was deleted, -ENOMEM on descriptor OOM.
 *
 * The escalation DOMAIN is NOT embedded in the head (that is what keeps the head
 * at 8 bytes): the mutators take a struct urcu_txn_domain * explicitly, so a
 * whole table of buckets shares one domain (or a small striped set), rather than
 * paying a fair-mutex per bucket.  Readers traverse forward only, within an RCU
 * read-side section, through the accessors below (which resolve the proxy and
 * strip the mark); never touch the raw fields.
 */

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include <urcu/compiler.h>
#include <urcu/uatomic.h>
#include <urcu/call-rcu.h>
#include <urcu/rcu-txn-engine.h>
#include <urcu/rcu-txn.h>
#include <urcu-pointer.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Engine proxy tag for every hlist slot (head-first and node next/pprev).
 * Override before include to drive the chain under an embedder's own tag (e.g.
 * the fractal trie's child-slot nibble).  Must satisfy (value & TAG) != TAG for
 * every live value any slot holds -- see the header contract.
 */
#ifndef URCU_TXN_HLIST_TAG
#define URCU_TXN_HLIST_TAG	URCU_TXN_TAG
#endif

/*
 * Logical-deletion mark: bit 1 of a node's next pointer (override before
 * include; must be free in every live "next" value and not, on its own, set all
 * of URCU_TXN_HLIST_TAG's bits -- see the header contract).
 */
#ifndef URCU_TXN_HLIST_MARK
#define URCU_TXN_HLIST_MARK	2UL
#endif

/*
 * The non-aliasing contract between the two, stated in the header intro, is
 * what keeps a ghost's marked "next" from reading as an engine proxy.  Should
 * an override make MARK set all of TAG's bits, every marked next would satisfy
 * urcu_txn_is_proxy() and a reader would fabricate a record pointer out of a
 * plain node address -- a wild dereference on the read side, in a build that
 * still compiles.  It is a pure compile-time property of two macros: check it
 * at compile time.
 */
urcu_static_assert((URCU_TXN_HLIST_MARK & URCU_TXN_HLIST_TAG) !=
			URCU_TXN_HLIST_TAG,
		"URCU_TXN_HLIST_MARK must not set all of URCU_TXN_HLIST_TAG's bits: "
		"a marked next would resolve as an engine proxy",
		URCU_TXN_HLIST_MARK_aliases_TAG);

struct urcu_txn_hlist_node {
	struct urcu_txn_hlist_node *next;	/* node ptr; transacted; MARK-able */
	struct urcu_txn_hlist_node **pprev;	/* slot ptr (&prev->next or
						 * &head->first); transacted */
};

/*
 * The bucket head is a SINGLE pointer -- 8 bytes, matching the kernel's
 * struct hlist_head -- so an array of these is half the footprint of the bidir
 * list's sentinel-node heads.  It carries neither an escalation domain nor a
 * proxy tag: the domain is passed to the ops and the tag is a compile-time
 * define (see the contract above), so a table shares one domain and one tag
 * with zero per-bucket overhead.
 */
struct urcu_txn_hlist_head {
	struct urcu_txn_hlist_node *first;	/* an ordinary "next" slot */
};

#define URCU_TXN_HLIST_HEAD_INIT	{ .first = NULL }

static inline
void urcu_txn_hlist_init(struct urcu_txn_hlist_head *head)
{
	head->first = NULL;
}

static inline
void *urcu_txn_hlist_set_mark(struct urcu_txn_hlist_node *n)
{
	return (void *) ((uintptr_t) n | URCU_TXN_HLIST_MARK);
}

static inline
int urcu_txn_hlist_is_marked(void *v)
{
	return (int) ((uintptr_t) v & URCU_TXN_HLIST_MARK);
}

static inline
struct urcu_txn_hlist_node *urcu_txn_hlist_unmark(void *v)
{
	return (struct urcu_txn_hlist_node *)
			((uintptr_t) v & ~(uintptr_t) URCU_TXN_HLIST_MARK);
}

/*
 * Resolve a raw "next"/head-first slot value: strip the engine proxy, then the
 * mark.  Fast path -- a clean value (neither a proxy under URCU_TXN_HLIST_TAG
 * nor MARK-ed) is returned untouched, so a live-node traversal never runs the
 * unmark AND and the pointer stays out of the load-to-use dependency chain.
 * Only a tagged value (an in-flight proxy, or a ghost's marked "next") takes the
 * slow path.  pprev, being read only by mutators inside the bracket, has no
 * reader-side resolve.
 */
static inline
struct urcu_txn_hlist_node *urcu_txn_hlist_resolve(void *raw)
{
	uintptr_t v = (uintptr_t) raw;

	if (caa_unlikely(v & (URCU_TXN_HLIST_TAG | URCU_TXN_HLIST_MARK)))
		return urcu_txn_hlist_unmark(
				urcu_txn_resolve(raw, URCU_TXN_HLIST_TAG));
	return (struct urcu_txn_hlist_node *) raw;
}

/* Resolved bucket-first / forward step (call within an RCU read-side section). */
static inline
struct urcu_txn_hlist_node *urcu_txn_hlist_first_rcu(
		struct urcu_txn_hlist_head *head)
{
	return urcu_txn_hlist_resolve((void *) rcu_dereference(head->first));
}

static inline
struct urcu_txn_hlist_node *urcu_txn_hlist_next_rcu(
		struct urcu_txn_hlist_node *node)
{
	return urcu_txn_hlist_resolve((void *) rcu_dereference(node->next));
}

/*
 * Whether bucket @head is empty.  A reader accessor like the rest: call within
 * an RCU read-side section -- the head slot it reads may hold a proxy, and
 * resolving one dereferences a descriptor that a grace period would otherwise
 * reclaim.
 */
static inline
int urcu_txn_hlist_empty(struct urcu_txn_hlist_head *head)
{
	return urcu_txn_hlist_first_rcu(head) == NULL;
}

/*
 * urcu_txn_hlist_insert_at_slot_prepare: the core composable primitive.  Record
 * the edges that make @slot name @newp, given that @slot currently resolves to
 * @succ (the caller read it -- via urcu_txn_load on the head or on a node's next
 * -- and stripped any mark), WITHOUT committing.  @slot is &head->first for
 * insert-at-head or &pos->next for insert-after; @succ is the (possibly NULL)
 * unmarked value it holds.  The *slot store (old @succ) is the serializing edge:
 * any concurrent insert/delete that rewrites @slot fails this commit's old-value
 * check.  Returns 0, or -EAGAIN if @succ is a neighbour mid-deletion (retry);
 * OOM is sticky to the commit.
 *
 * COMPOSE ON A DEFAULT HANDLE.  The self-contained wrappers below declare their
 * write set disjoint -- sound because a SINGLE-op commit provably touches
 * distinct slots -- but that line must NOT be copied into a composed bracket
 * unless the COMBINED write set is provably distinct too.  Two edits of one
 * bucket share a slot as soon as their nodes land adjacent (both name the
 * shared neighbour's "next", or &head->first): data-dependent, not knowable up
 * front.  On a disjoint handle the second prepare's loads then silently return
 * committed values and its stores blind-append a duplicate record -- silent
 * corruption.  The default read-your-own-writes handle sees the txn's own
 * pending edits and chains the collision into one record.  See
 * urcu_txn_declare_disjoint() in <urcu/rcu-txn.h>.
 */
static inline
int urcu_txn_hlist_insert_at_slot_prepare(struct urcu_txn *txn,
		struct urcu_txn_hlist_node *newp,
		struct urcu_txn_hlist_node **slot,
		struct urcu_txn_hlist_node *succ)
{
	/*
	 * We write &succ->pprev but NOT &succ->next, so the slot-sorted install
	 * may reach &succ->pprev before it reaches @slot -- driving the pprev
	 * store against a @succ a concurrent del(succ) is freeing.  Fold a
	 * load-validate of succ->next (the slot del(succ) marks) into the
	 * write-set so the pprev side serializes against del(succ) exactly as
	 * @slot does; a marked @succ aborts here.  (When @succ is NULL there is
	 * no backward edge and no such window -- a 1-edge insert.)  Unlike the
	 * bidir list there is no next == prev collision to skip: &succ->next
	 * cannot equal @slot (that needs @succ to be its own predecessor).
	 */
	if (succ != NULL &&
			urcu_txn_hlist_is_marked(urcu_txn_load_validate(txn,
				(void **) &succ->next, URCU_TXN_HLIST_TAG)))
		return -EAGAIN;			/* succ (a neighbour) deleted: retry */

	/* Build the fresh node invisibly. */
	newp->next = succ;
	newp->pprev = slot;

	/* *slot: succ -> newp ; succ->pprev: slot -> &newp->next. */
	urcu_txn_store_mw(txn, (void **) slot, succ, newp, URCU_TXN_HLIST_TAG);
	if (succ != NULL)
		urcu_txn_store_mw(txn, (void **) &succ->pprev, slot,
				&newp->next, URCU_TXN_HLIST_TAG);
	return 0;
}

/*
 * urcu_txn_hlist_insert_after_prepare: record an insert of @newp immediately
 * after @pos, WITHOUT committing.  Composable form (see insert_at_slot_prepare
 * and urcu_txn_list_insert_after_prepare for the contract).  Returns 0, -ENOENT
 * if @pos was deleted, or -EAGAIN if the successor is mid-deletion (retry).
 */
static inline
int urcu_txn_hlist_insert_after_prepare(struct urcu_txn *txn,
		struct urcu_txn_hlist_node *newp,
		struct urcu_txn_hlist_node *pos)
{
	void *pn = urcu_txn_load(txn, (void **) &pos->next, URCU_TXN_HLIST_TAG);

	if (urcu_txn_hlist_is_marked(pn))
		return -ENOENT;			/* @pos was deleted */
	return urcu_txn_hlist_insert_at_slot_prepare(txn, newp,
			&pos->next, (struct urcu_txn_hlist_node *) pn);
}

/*
 * urcu_txn_hlist_insert_head_prepare: record an insert of @newp at the head of
 * @head (making it the bucket's first node), WITHOUT committing.  The head slot
 * is immortal in the BASE hlist -- nothing ever marks it -- so this normally
 * never observes a deleted anchor.  Returns 0, -EAGAIN if the old first node is
 * mid-deletion (retry), or -ENOENT if the head slot itself carries a mark.
 */
static inline
int urcu_txn_hlist_insert_head_prepare(struct urcu_txn *txn,
		struct urcu_txn_hlist_node *newp,
		struct urcu_txn_hlist_head *head)
{
	void *fn = urcu_txn_load(txn, (void **) &head->first, URCU_TXN_HLIST_TAG);

	/*
	 * Fail on a marked head rather than stripping the mark, which is what
	 * unmark() alone would do.  No base-hlist operation can set it, so this
	 * costs one predicted branch and is dead today -- but a MARKED HEAD IS
	 * EXACTLY the sealing primitive the incremental-rehash design reserves
	 * (seal the bucket, migrate it, retire it).  Under a stripping insert
	 * that seal is invisible: the insert records {&head->first: first ->
	 * newp} against a slot whose committed value carries the mark, the
	 * old-value check fails at every install, and the mutator loops
	 * ABORT-forever -- past URCU_TXN_FALLBACK holding the domain's lane
	 * while it does.  Reporting -ENOENT (an anchor that is gone, the same
	 * convention insert_after_prepare uses for a deleted @pos) makes the
	 * caller retire the bucket instead.
	 */
	if (urcu_txn_hlist_is_marked(fn))
		return -ENOENT;			/* head sealed: the bucket is gone */
	return urcu_txn_hlist_insert_at_slot_prepare(txn, newp,
			&head->first, (struct urcu_txn_hlist_node *) fn);
}

/*
 * Add @newp at the head of bucket @head, transacting through @domain's
 * escalation lane.  Returns 0 on success, -ENOMEM on descriptor OOM, or -ENOENT
 * if the head slot has been SEALED -- unreachable in the base hlist, whose head
 * is immortal, but see urcu_txn_hlist_insert_head_prepare().  Self-contained
 * bracket around it.
 */
static inline
int urcu_txn_hlist_add_rcu(struct urcu_txn_hlist_node *newp,
		struct urcu_txn_hlist_head *head,
		struct urcu_txn_domain *domain)
{
	struct urcu_txn txn;
	int ret, prep;

	urcu_txn_init(&txn, domain);
	urcu_txn_declare_disjoint(&txn);	/* single-op commit: distinct slots, no same-slot WAW */
	for (;;) {
		urcu_txn_begin(&txn);
		prep = urcu_txn_hlist_insert_head_prepare(&txn, newp, head);
		if (prep == -EAGAIN) {			/* old first moved: retry */
			urcu_txn_conflict(&txn);
			urcu_txn_end(&txn);
			continue;
		}
		if (prep) {				/* -ENOENT: head sealed */
			urcu_txn_end(&txn);
			return prep;		/* nothing recorded: do not commit */
		}
		ret = urcu_txn_commit(&txn);
		urcu_txn_end(&txn);
		if (ret != URCU_TXN_STATUS_ABORT)	/* ABORT: a neighbour changed */
			break;
	}
	return ret < 0 ? -ENOMEM : 0;
}

/*
 * Insert @newp immediately after @pos, transacting through @domain.  Returns 0,
 * -ENOENT if @pos was deleted, or -ENOMEM on descriptor OOM.  Self-contained
 * bracket around urcu_txn_hlist_insert_after_prepare().
 */
static inline
int urcu_txn_hlist_insert_after_rcu(struct urcu_txn_hlist_node *newp,
		struct urcu_txn_hlist_node *pos,
		struct urcu_txn_domain *domain)
{
	struct urcu_txn txn;
	int ret, prep;

	urcu_txn_init(&txn, domain);
	urcu_txn_declare_disjoint(&txn);	/* single-op commit: distinct slots, no same-slot WAW */
	for (;;) {
		urcu_txn_begin(&txn);
		prep = urcu_txn_hlist_insert_after_prepare(&txn, newp, pos);
		if (prep == -EAGAIN) {			/* successor moved: retry */
			urcu_txn_conflict(&txn);
			urcu_txn_end(&txn);
			continue;
		}
		if (prep) {				/* -ENOENT: @pos deleted */
			urcu_txn_end(&txn);
			return prep;
		}
		ret = urcu_txn_commit(&txn);
		urcu_txn_end(&txn);
		if (ret != URCU_TXN_STATUS_ABORT)
			break;
	}
	return ret < 0 ? -ENOMEM : 0;
}

/*
 * urcu_txn_hlist_insert_before_prepare: record an insert of @newp immediately
 * before @pos (kernel hlist_add_before), WITHOUT committing.  Serializes on the
 * slot that names @pos (*pos->pprev), which del(pos)/del(prev)/insert-at-slot all
 * CAS with old value @pos.  Returns 0, or -ENOENT if @pos was deleted.  OOM is
 * sticky to the commit.
 */
static inline
int urcu_txn_hlist_insert_before_prepare(struct urcu_txn *txn,
		struct urcu_txn_hlist_node *newp,
		struct urcu_txn_hlist_node *pos)
{
	/*
	 * Load-validate pos->next (the slot del(pos) marks) so the &pos->pprev
	 * store serializes against del(pos) even when the slot-sorted install
	 * reaches &pos->pprev first, and detect an already-deleted anchor.
	 */
	void *pn = urcu_txn_load_validate(txn, (void **) &pos->next,
			URCU_TXN_HLIST_TAG);
	struct urcu_txn_hlist_node **slot;

	if (urcu_txn_hlist_is_marked(pn))
		return -ENOENT;			/* @pos was deleted */
	slot = (struct urcu_txn_hlist_node **)
			urcu_txn_load(txn, (void **) &pos->pprev,
				URCU_TXN_HLIST_TAG);

	newp->next = pos;
	newp->pprev = slot;

	/* *slot: pos -> newp ; pos->pprev: slot -> &newp->next. */
	urcu_txn_store_mw(txn, (void **) slot, pos, newp, URCU_TXN_HLIST_TAG);
	urcu_txn_store_mw(txn, (void **) &pos->pprev, slot, &newp->next,
			URCU_TXN_HLIST_TAG);
	return 0;
}

/*
 * Insert @newp immediately before @pos, transacting through @domain.  Returns 0,
 * -ENOENT if @pos was deleted, or -ENOMEM on descriptor OOM.  Self-contained
 * bracket around urcu_txn_hlist_insert_before_prepare().
 */
static inline
int urcu_txn_hlist_insert_before_rcu(struct urcu_txn_hlist_node *newp,
		struct urcu_txn_hlist_node *pos,
		struct urcu_txn_domain *domain)
{
	struct urcu_txn txn;
	int ret, prep;

	urcu_txn_init(&txn, domain);
	urcu_txn_declare_disjoint(&txn);	/* single-op commit: distinct slots, no same-slot WAW */
	do {
		urcu_txn_begin(&txn);
		prep = urcu_txn_hlist_insert_before_prepare(&txn, newp, pos);
		if (prep) {				/* -ENOENT: @pos deleted */
			urcu_txn_end(&txn);
			return prep;
		}
		ret = urcu_txn_commit(&txn);
		urcu_txn_end(&txn);
	} while (ret == URCU_TXN_STATUS_ABORT);
	return ret < 0 ? -ENOMEM : 0;
}

/*
 * urcu_txn_hlist_del_prepare: record the unlink of @elem into @txn WITHOUT
 * committing.  @elem is reached by the caller (it need NOT know the bucket:
 * pprev names the slot).  Composable form of del.  Returns 0 if recorded (on a
 * committed OK, THIS call removed @elem; reclaim it after a grace period),
 * -ENOENT if @elem was already deleted by a peer (nothing recorded; do NOT
 * reclaim), or -EAGAIN if the successor is mid-deletion (retry).  OOM is sticky
 * to the commit.
 */
static inline
int urcu_txn_hlist_del_prepare(struct urcu_txn *txn,
		struct urcu_txn_hlist_node *elem)
{
	void *en = urcu_txn_load(txn, (void **) &elem->next, URCU_TXN_HLIST_TAG);
	struct urcu_txn_hlist_node *next;
	struct urcu_txn_hlist_node **ppv;

	if (urcu_txn_hlist_is_marked(en))
		return -ENOENT;			/* already deleted by a peer */
	next = (struct urcu_txn_hlist_node *) en;
	/*
	 * Read the slot that names @elem fresh this attempt.  If a peer changed
	 * which slot names @elem (e.g. del of @elem's predecessor rewrote
	 * elem->pprev), this @ppv is stale -- but the *ppv store below then fails
	 * its old-value check (the stale slot no longer holds @elem) and the
	 * commit aborts, so a retry re-reads the fresh slot.
	 */
	ppv = (struct urcu_txn_hlist_node **)
			urcu_txn_load(txn, (void **) &elem->pprev,
				URCU_TXN_HLIST_TAG);

	/*
	 * We rewrite &next->pprev but not &next->next.  Load-validate next->next
	 * (the slot del(next) marks) so the backward unlink serializes against
	 * del(next) even when the slot-sorted install reaches &next->pprev first.
	 * A marked successor is itself being deleted: retry.  No next == prev skip
	 * is needed (see the file header): &next->next never collides with *ppv
	 * (that needs next to be its own predecessor) nor with &elem->next.
	 */
	if (next != NULL &&
			urcu_txn_hlist_is_marked(urcu_txn_load_validate(
				txn, (void **) &next->next, URCU_TXN_HLIST_TAG)))
		return -EAGAIN;			/* successor (a neighbour) deleted: retry */

	/*
	 * Mark elem (logical delete), unlink forward (*ppv: elem -> next), and
	 * unlink backward (next->pprev: &elem->next -> ppv, i.e. next is now named
	 * by elem's old slot).  *ppv (old value elem) is the slot a racing
	 * insert-at-slot / del(predecessor) shares with us, so the MCAS serializes
	 * every adjacency; marking &elem->next is what makes a racing
	 * insert_after(elem) / del(elem) terminate with -ENOENT.  When next is
	 * NULL the backward edge vanishes: a 2-edge delete storing MARK(NULL).
	 */
	urcu_txn_store_mw(txn, (void **) &elem->next, next,
			urcu_txn_hlist_set_mark(next), URCU_TXN_HLIST_TAG);
	urcu_txn_store_mw(txn, (void **) ppv, elem, next, URCU_TXN_HLIST_TAG);
	if (next != NULL)
		urcu_txn_store_mw(txn, (void **) &next->pprev,
				&elem->next, ppv, URCU_TXN_HLIST_TAG);
	return 0;
}

/*
 * Remove @elem, transacting through @domain.  @elem need not know its bucket.
 * Returns 1 if THIS call removed it (reclaim @elem after a grace period), 0 if
 * it was already deleted (do NOT reclaim), or -ENOMEM on descriptor OOM.
 * Self-contained bracket around urcu_txn_hlist_del_prepare().
 */
static inline
int urcu_txn_hlist_del_rcu(struct urcu_txn_hlist_node *elem,
		struct urcu_txn_domain *domain)
{
	struct urcu_txn txn;
	int ret, prep;

	urcu_txn_init(&txn, domain);
	urcu_txn_declare_disjoint(&txn);	/* single-op commit: distinct slots, no same-slot WAW */
	for (;;) {
		urcu_txn_begin(&txn);
		prep = urcu_txn_hlist_del_prepare(&txn, elem);
		if (prep == -EAGAIN) {			/* successor moved: retry */
			urcu_txn_conflict(&txn);
			urcu_txn_end(&txn);
			continue;
		}
		if (prep) {				/* -ENOENT: already deleted */
			urcu_txn_end(&txn);
			return 0;			/* not removed by this call */
		}
		ret = urcu_txn_commit(&txn);
		urcu_txn_end(&txn);
		if (ret != URCU_TXN_STATUS_ABORT)
			break;
	}
	return ret < 0 ? -ENOMEM : 1;
}

/*
 * urcu_txn_hlist_replace_prepare: record the in-place replacement of @old by
 * @newp into @txn WITHOUT committing.  @newp takes @old's slot -- *old->pprev and
 * &next->pprev swing to @newp -- while @old is logically removed (its next is
 * marked exactly as del does).  Touches the SAME slots as del_prepare (only the
 * new values differ), so it inherits del's serialization.  Argument order is
 * (old, new), as cds_list_replace_rcu().  Returns 0 if recorded (reclaim @old
 * after a grace period on commit), -ENOENT if @old was already deleted, or
 * -EAGAIN if the successor is mid-deletion.  OOM is sticky to the commit.
 */
static inline
int urcu_txn_hlist_replace_prepare(struct urcu_txn *txn,
		struct urcu_txn_hlist_node *old,
		struct urcu_txn_hlist_node *newp)
{
	void *en = urcu_txn_load(txn, (void **) &old->next, URCU_TXN_HLIST_TAG);
	struct urcu_txn_hlist_node *next;
	struct urcu_txn_hlist_node **ppv;

	if (urcu_txn_hlist_is_marked(en))
		return -ENOENT;			/* @old already deleted/replaced */
	next = (struct urcu_txn_hlist_node *) en;
	ppv = (struct urcu_txn_hlist_node **)
			urcu_txn_load(txn, (void **) &old->pprev,
				URCU_TXN_HLIST_TAG);

	if (next != NULL &&
			urcu_txn_hlist_is_marked(urcu_txn_load_validate(
				txn, (void **) &next->next, URCU_TXN_HLIST_TAG)))
		return -EAGAIN;			/* successor (a neighbour) deleted: retry */

	/* Build @newp's links invisibly, then swing @old's slot and next->pprev. */
	newp->next = next;
	newp->pprev = ppv;

	urcu_txn_store_mw(txn, (void **) &old->next, next,
			urcu_txn_hlist_set_mark(next), URCU_TXN_HLIST_TAG);
	urcu_txn_store_mw(txn, (void **) ppv, old, newp, URCU_TXN_HLIST_TAG);
	if (next != NULL)
		urcu_txn_store_mw(txn, (void **) &next->pprev,
				&old->next, &newp->next, URCU_TXN_HLIST_TAG);
	return 0;
}

/*
 * Replace @old with @newp atomically with respect to RCU readers, transacting
 * through @domain.  Argument order is (old, new).  Returns 0 on success (reclaim
 * @old after a grace period), -ENOENT if @old was already deleted, or -ENOMEM on
 * descriptor OOM.  Self-contained bracket around urcu_txn_hlist_replace_prepare().
 */
static inline
int urcu_txn_hlist_replace_rcu(struct urcu_txn_hlist_node *old,
		struct urcu_txn_hlist_node *newp,
		struct urcu_txn_domain *domain)
{
	struct urcu_txn txn;
	int ret, prep;

	urcu_txn_init(&txn, domain);
	urcu_txn_declare_disjoint(&txn);	/* single-op commit: distinct slots, no same-slot WAW */
	for (;;) {
		urcu_txn_begin(&txn);
		prep = urcu_txn_hlist_replace_prepare(&txn, old, newp);
		if (prep == -EAGAIN) {			/* a neighbour moved: retry */
			urcu_txn_conflict(&txn);
			urcu_txn_end(&txn);
			continue;
		}
		if (prep) {				/* -ENOENT: @old already gone */
			urcu_txn_end(&txn);
			return prep;
		}
		ret = urcu_txn_commit(&txn);
		urcu_txn_end(&txn);
		if (ret != URCU_TXN_STATUS_ABORT)
			break;
	}
	return ret < 0 ? -ENOMEM : 0;
}

#define urcu_txn_hlist_entry(ptr, type, member) \
	caa_container_of(ptr, type, member)

/* container_of that maps a NULL chain terminator to a NULL entry (kernel idiom). */
#define urcu_txn_hlist_entry_safe(ptr, type, member) \
	__extension__ ({ __typeof__(ptr) ___ptr = (ptr); \
		___ptr ? urcu_txn_hlist_entry(___ptr, type, member) : NULL; })

/* Iterate forward over a bucket (within an RCU read-side section). */
#define urcu_txn_hlist_for_each_rcu(pos, head) \
	for (pos = urcu_txn_hlist_first_rcu(head); \
		(pos) != NULL; \
		pos = urcu_txn_hlist_next_rcu(pos))

#define urcu_txn_hlist_for_each_entry_rcu(pos, head, member) \
	for (pos = urcu_txn_hlist_entry_safe( \
			urcu_txn_hlist_first_rcu(head), \
			__typeof__(*(pos)), member); \
		(pos) != NULL; \
		pos = urcu_txn_hlist_entry_safe( \
			urcu_txn_hlist_next_rcu(&(pos)->member), \
			__typeof__(*(pos)), member))

#ifdef __cplusplus
}
#endif

#endif	/* _URCU_RCU_TXN_HLIST_H */
