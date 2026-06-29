// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

#ifndef _URCU_RCU_BIDIR_LIST_LOCKFREE_H
#define _URCU_RCU_BIDIR_LIST_LOCKFREE_H

/*
 * rcu-bidir-list-lockfree: a bidirectional, coherent RCU list with LOCK-FREE
 * concurrent writers, built on the lock-free flip-latch MCAS engine
 * (<urcu/rcu-mcas.h>).  It is the concurrent-writer sibling of
 * <urcu/rcu-bidir-list.h> (which requires writer mutual exclusion).
 *
 * Like the single-writer version, every structural change flips both
 * reader-visible edges -- forward and backward -- as ONE atomic event, so the
 * two directions never disagree.  Here that atomic event is an MCAS commit, and
 * multiple writers may run concurrently with guaranteed system-wide progress.
 *
 * Logical deletion (anchor invalidation)
 * --------------------------------------
 * Arbitrary-position insert needs to detect that the node it is inserting next
 * to has been deleted under it.  A node's "next" pointer carries a deletion
 * MARK (bit 1; bit 0 is the engine's proxy tag -- nodes are >=4-byte aligned so
 * both are free).  A marked next means the node is logically deleted.
 *
 *   del(elem)  = one 3-edge MCAS:
 *                  &elem->next : next  -> MARK(next)     (logical delete)
 *                  &prev->next : elem  -> next           (unlink forward)
 *                  &next->prev : elem  -> prev           (unlink backward)
 *   insert     = one 2-edge MCAS, after checking the anchor's next is unmarked.
 *
 * Why a "next"-only mark is enough (insert/delete coherence)
 * ----------------------------------------------------------
 * Two distinct things detect a racing deletion, and it matters to keep them
 * apart -- only ONE of them is the mark:
 *
 *  (1) A NEIGHBOUR was deleted -- handled for free by a structural slot
 *      conflict, NOT by the mark.  Removing a node X from a doubly-linked list
 *      MUST rewrite X's predecessor's "next" pointer (that is the only forward
 *      pointer that names X).  An insert placed next to X also rewrites some
 *      node's "next".  So an adjacent insert and delete CAS the SAME "next"
 *      slot with the SAME expected old value, and the MCAS cannot commit both.
 *
 *      Example -- list A <-> B <-> C, insert_after(newp, A) racing del(B):
 *
 *        insert_after(newp, A):        del(B):
 *          &A->next : B  -> newp         &B->next : C  -> MARK(C)
 *          &B->prev : A  -> newp         &A->next : B  -> C
 *                                        &C->prev : B  -> A
 *
 *      Both CAS &A->next with old value B; only one wins.
 *        - del(B) wins:  A->next becomes C; insert's &A->next:B->newp now fails
 *          its old-value check, aborts, and on retry re-reads A->next == C and
 *          inserts A <-> newp <-> C.  (B is gone; insert never linked to it.)
 *        - insert wins:  A->next becomes newp (A <-> newp <-> B <-> C, coherent,
 *          since A->next was still B at the commit instant); del(B)'s
 *          &A->next:B->C now fails, aborts, and on retry removes B from between
 *          newp and C, leaving A <-> newp <-> C.
 *      Either order leaves a coherent list.  The insert "notices" the delete
 *      only as a moved predecessor-next (a failed old-value check) -- never by
 *      reading a mark.  insert_before vs a deleted predecessor is the mirror
 *      image, sharing the predecessor's "next" slot in just the same way.
 *
 *  (2) The ANCHOR ITSELF was deleted -- this is the mark's one and only job.
 *      When the @pos handed to an insert is the node being deleted, the insert
 *      also loses the shared-slot race above; but on retry it must tell
 *      "neighbour moved, re-read and proceed" apart from "my anchor is gone,
 *      give up".  It re-reads @pos->next, sees the mark, and returns -ENOENT.
 *
 * That is also why @prev is never marked: no operation reaches a node ONLY
 * through a "prev" edge and would need a mark there to terminate its retry --
 * the forward "next" slots already serialize every adjacency.  (Mutators and
 * the prev accessor still defensively strip a mark from "prev" values, so the
 * code stays correct should that invariant ever be revisited.)
 *
 * Reclaim
 * -------
 * del() returns whether THIS call removed the node, so two concurrent deletes
 * of the same node cannot double-free: the single winner gets 1 (its caller
 * call_rcu()s the node), the other gets 0.  As with cds_list_del_rcu(), the node
 * itself is the caller's to reclaim after a grace period.
 *
 * Read / write contract
 * ---------------------
 * Include this header AFTER an RCU flavor (e.g. <urcu-qsbr.h>): the mutators
 * commit through the transaction bracket <urcu/rcu-txn.h>,
 * which opens an RCU read-side section per attempt and defers descriptor
 * reclaim through the flavor's call_rcu -- the read lock is what keeps an
 * in-flight MCAS descriptor alive while peers help drive it.  A mutator thus
 * brackets its own section; the caller still holds a read-side section across
 * the call so the node arguments (@pos / @elem, and whatever a traversal
 * reached them through) stay alive -- this nests harmlessly inside the
 * mutator's section.  Readers likewise run within an RCU read-side section,
 * and read next/prev only through the accessors below (they resolve the proxy
 * and strip the mark); never touch the raw fields.  Mutators loop internally
 * until they commit or definitively fail; they return 0 / 1 on success,
 * -ENOENT if the anchor was deleted, -ENOMEM on descriptor OOM.  Each transacts
 * through the list head's escalation domain, so a mutator repeatedly bypassed
 * on the optimistic path escalates into the domain's fair lane and commits
 * within a bounded number of retries -- the list is starvation-resistant, not
 * merely lock-free.
 */

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include <urcu/compiler.h>
#include <urcu/uatomic.h>
#include <urcu/call-rcu.h>
#include <urcu/rcu-mcas.h>
#include <urcu/rcu-txn.h>
#include <urcu-pointer.h>

#ifdef __cplusplus
extern "C" {
#endif

struct cds_bidir_list_lf_node {
	struct cds_bidir_list_lf_node *next, *prev;
};

/*
 * A list is its circular sentinel node plus the per-list escalation domain that
 * the lock-free transaction front-end funnels starved or oversized mutators
 * through (see <urcu/rcu-txn.h>).  The domain lives here, once
 * per list, which is why the mutators below take the head: an element is
 * reached only via its list, and a mutator needs that list's domain to stay
 * starvation-resistant under adversarial contention.
 *
 * The domain wraps a fair mutex with no static initializer, so a head must be
 * initialized at runtime with cds_bidir_list_lf_init() -- there is no static
 * HEAD_INIT form.
 */
struct cds_bidir_list_lf_head {
	struct cds_bidir_list_lf_node node;	/* circular sentinel */
	struct urcu_txn_domain domain;	/* shared escalation domain */
};

static inline
void cds_bidir_list_lf_init(struct cds_bidir_list_lf_head *head)
{
	head->node.next = &head->node;
	head->node.prev = &head->node;
	urcu_txn_domain_init(&head->domain);
}

/* Logical-deletion mark: bit 1 of a node's next pointer. */
#define CDS_BIDIR_LIST_LF_MARK	2UL

static inline
void *cds_bidir_list_lf_set_mark(struct cds_bidir_list_lf_node *n)
{
	return (void *) ((uintptr_t) n | CDS_BIDIR_LIST_LF_MARK);
}

static inline
int cds_bidir_list_lf_is_marked(void *v)
{
	return (int) ((uintptr_t) v & CDS_BIDIR_LIST_LF_MARK);
}

static inline
struct cds_bidir_list_lf_node *cds_bidir_list_lf_unmark(void *v)
{
	return (struct cds_bidir_list_lf_node *)
			((uintptr_t) v & ~(uintptr_t) CDS_BIDIR_LIST_LF_MARK);
}

/*
 * Resolve a raw next/prev slot value: strip the engine proxy, then the mark.
 *
 * Fast path -- a clean value (neither the engine's proxy tag nor the deletion
 * mark set) is returned untouched, so a live-node traversal never runs the
 * unmark AND and the pointer stays out of the load-to-use dependency chain.
 * Only a tagged value (an in-flight proxy, or a ghost's marked "next") takes
 * the slow path, where unmark + urcu_mcas_resolve handle either or both
 * bits.  The mask lists EVERY reserved low bit on a transacted slot value;
 * revisit it if more tag bits are ever added.
 */
static inline
struct cds_bidir_list_lf_node *cds_bidir_list_lf_resolve(void *raw)
{
	uintptr_t v = (uintptr_t) raw;

	if (caa_unlikely(v & (URCU_MCAS_TAG | CDS_BIDIR_LIST_LF_MARK)))
		return cds_bidir_list_lf_unmark(urcu_mcas_resolve(raw));
	return (struct cds_bidir_list_lf_node *) raw;
}

/* Resolved forward / backward step (call within an RCU read-side section). */
static inline
struct cds_bidir_list_lf_node *cds_bidir_list_lf_next_rcu(
		struct cds_bidir_list_lf_node *node)
{
	return cds_bidir_list_lf_resolve((void *) rcu_dereference(node->next));
}

static inline
struct cds_bidir_list_lf_node *cds_bidir_list_lf_prev_rcu(
		struct cds_bidir_list_lf_node *node)
{
	return cds_bidir_list_lf_resolve((void *) rcu_dereference(node->prev));
}

static inline
int cds_bidir_list_lf_empty(struct cds_bidir_list_lf_head *head)
{
	return cds_bidir_list_lf_next_rcu(&head->node) == &head->node;
}

/*
 * cds_bidir_list_lf_insert_after_prepare: record the edges of an insert-after
 * into the caller-owned transaction @txn, WITHOUT committing.  This is the
 * composable form: the caller owns the bracket (begin .. commit .. end), the
 * escalation domain (whichever @txn was init'd with), and the retry loop, and
 * may fold these records together with records from other structures into a
 * single MCAS commit -- e.g. publish a node into a trie and splice it into this
 * list atomically.  Call between urcu_txn_begin() and
 * urcu_txn_commit().  Returns 0 on success, or -ENOENT if @pos has been
 * deleted (the caller ends the bracket and bails the logical op); a descriptor
 * OOM is sticky and surfaces at the caller's commit.  @pos must be kept alive
 * by the caller's RCU read-side section (see the contract above).
 */
static inline
int cds_bidir_list_lf_insert_after_prepare(struct urcu_mcas_txn *txn,
		struct cds_bidir_list_lf_node *newp,
		struct cds_bidir_list_lf_node *pos)
{
	void *pn = urcu_txn_load(txn, (void **) &pos->next);
	struct cds_bidir_list_lf_node *succ;

	if (cds_bidir_list_lf_is_marked(pn))
		return -ENOENT;				/* @pos was deleted */
	succ = (struct cds_bidir_list_lf_node *) pn;	/* unmarked successor */

	/*
	 * We write &succ->prev but NOT &succ->next.  The next slot that serializes
	 * this insert against del(succ) is &pos->next -- but the slot-sorted MCAS
	 * may install &succ->prev BEFORE it reaches &pos->next, so the prev-side
	 * store can be driven against a succ a concurrent del(succ) is freeing
	 * (a foreign slot in the descriptor, e.g. a composing structure's, widens
	 * this window).  Fold a load-validate of succ->next -- the slot del(succ)
	 * marks -- into the write-set, so the prev side serializes against
	 * del(succ) exactly as the next side does; a marked succ aborts here.
	 */
	if (cds_bidir_list_lf_is_marked(urcu_txn_load_validate(txn,
			(void **) &succ->next)))
		return -EAGAIN;				/* succ (a neighbour) deleted: retry */

	/* Build the fresh node invisibly. */
	newp->next = succ;
	newp->prev = pos;

	/*
	 * pos->next: succ -> newp ; succ->prev: pos -> newp.
	 * &pos->next is the slot that serializes us against deletion:
	 * del(pos) marks it, and del(succ) rewrites it to skip succ -- either
	 * makes this store fail its old value, so the commit aborts and the
	 * caller retries (and on a marked pos, returns -ENOENT above).  See the
	 * "next"-only mark rationale at the top of this file.
	 */
	urcu_txn_store(txn, (void **) &pos->next, succ, newp);
	urcu_txn_store(txn, (void **) &succ->prev, pos, newp);
	return 0;
}

/*
 * Insert @newp immediately after @pos in list @head.  Returns 0 on success,
 * -ENOENT if @pos has been deleted, -ENOMEM on descriptor allocation failure.
 * @head supplies the list's escalation domain (see the contract above).  This
 * is the self-contained convenience form: a thin bracket around
 * cds_bidir_list_lf_insert_after_prepare().
 */
static inline
int cds_bidir_list_lf_insert_after_rcu(struct cds_bidir_list_lf_node *newp,
		struct cds_bidir_list_lf_node *pos,
		struct cds_bidir_list_lf_head *head)
{
	struct urcu_mcas_txn txn;
	int ret, prep;

	urcu_txn_init(&txn, &head->domain);
	for (;;) {
		urcu_txn_begin(&txn);
		prep = cds_bidir_list_lf_insert_after_prepare(&txn, newp, pos);
		if (prep == -EAGAIN) {			/* succ (a neighbour) moved: retry */
			urcu_txn_conflict(&txn);	/* age so a hot slot escalates */
			urcu_txn_end(&txn);
			continue;
		}
		if (prep) {				/* -ENOENT: @pos deleted */
			urcu_txn_end(&txn);
			return prep;
		}
		ret = urcu_txn_commit(&txn);
		urcu_txn_end(&txn);
		if (ret != URCU_TXN_STATUS_ABORT)	/* ABORT: a neighbour changed */
			break;
	}
	return ret < 0 ? -ENOMEM : 0;		/* OK committed, -ENOMEM on OOM */
}

/*
 * Insert @newp after @pos in list @head, but only if @guard_slot still holds
 * @guard_expected at the commit -- the structural insert and that check
 * linearize as ONE MCAS (a load-validate guard).  Use it when the insert
 * depends on a word the list does not otherwise touch: a per-node "live" /
 * generation marker the embedder keeps beside its node, a container-freeze
 * flag, etc.  @guard_slot must be engine-transacted (every writer of it goes
 * through the MCAS).  The guard has value-CAS semantics: it checks @guard_slot
 * resolves to @guard_expected AT the linearization point, not that it stayed so
 * throughout -- the engine itself is A-B-A-safe, but if a benign recurrence of
 * @guard_expected would be the wrong answer for your insert, put a
 * version/generation in the guarded word.  Returns 0; -ENOENT if @pos was deleted
 * or @guard_slot no longer holds @guard_expected; -ENOMEM on descriptor OOM.
 */
static inline
int cds_bidir_list_lf_insert_after_guarded_rcu(
		struct cds_bidir_list_lf_node *newp,
		struct cds_bidir_list_lf_node *pos,
		struct cds_bidir_list_lf_head *head,
		void **guard_slot, void *guard_expected)
{
	struct urcu_mcas_txn txn;
	int ret;

	urcu_txn_init(&txn, &head->domain);
	for (;;) {
		void *pn;
		struct cds_bidir_list_lf_node *succ;

		urcu_txn_begin(&txn);
		pn = urcu_txn_load(&txn, (void **) &pos->next);
		if (cds_bidir_list_lf_is_marked(pn)) {
			urcu_txn_end(&txn);
			return -ENOENT;			/* @pos was deleted */
		}
		/*
		 * Pin the guard: the commit succeeds only if @guard_slot still
		 * holds @guard_expected at install, atomically with the insert.
		 * A read != expected means it never held -- bail (the recorded
		 * guard is discarded by end).
		 */
		if (urcu_txn_load_validate(&txn, guard_slot) !=
				guard_expected) {
			urcu_txn_end(&txn);
			return -ENOENT;			/* guard no longer holds */
		}
		succ = (struct cds_bidir_list_lf_node *) pn;
		/*
		 * Guard succ->next: we write &succ->prev but not &succ->next, so the
		 * prev-side store must serialize against del(succ) -- the slot-sorted
		 * install may reach &succ->prev before &pos->next.  See
		 * insert_after_prepare.  A marked succ moved: retry.
		 */
		if (cds_bidir_list_lf_is_marked(urcu_txn_load_validate(
				&txn, (void **) &succ->next))) {
			urcu_txn_conflict(&txn);	/* age so a hot slot escalates */
			urcu_txn_end(&txn);
			continue;
		}
		newp->next = succ;
		newp->prev = pos;
		urcu_txn_store(&txn, (void **) &pos->next, succ, newp);
		urcu_txn_store(&txn, (void **) &succ->prev, pos, newp);
		ret = urcu_txn_commit(&txn);
		urcu_txn_end(&txn);
		if (ret != URCU_TXN_STATUS_ABORT)
			break;
	}
	return ret < 0 ? -ENOMEM : 0;
}

/*
 * cds_bidir_list_lf_insert_before_prepare: record the edges of an
 * insert-before into the caller-owned transaction @txn, WITHOUT committing.
 * Composable form of insert_before (see insert_after_prepare for the contract).
 * Returns 0, or -ENOENT if @pos has been deleted; OOM is sticky to the commit.
 */
static inline
int cds_bidir_list_lf_insert_before_prepare(struct urcu_mcas_txn *txn,
		struct cds_bidir_list_lf_node *newp,
		struct cds_bidir_list_lf_node *pos)
{
	/*
	 * We write &pos->prev but not &pos->next.  Load-validate pos->next (@pos
	 * is the anchor whose prev we move) so the prev-side store serializes
	 * against del(pos) atomically even when the slot-sorted install reaches
	 * &pos->prev first.  A marked pos => the anchor is gone (-ENOENT).
	 */
	void *pn = urcu_txn_load_validate(txn, (void **) &pos->next);
	struct cds_bidir_list_lf_node *prev;

	if (cds_bidir_list_lf_is_marked(pn))
		return -ENOENT;			/* @pos was deleted */
	prev = cds_bidir_list_lf_unmark(
			urcu_txn_load(txn, (void **) &pos->prev));

	newp->next = pos;
	newp->prev = prev;

	/*
	 * prev->next: pos -> newp ; pos->prev: prev -> newp.
	 * &prev->next is the slot shared with del(prev) (which marks it) and
	 * del(pos) (which rewrites it to skip pos): a moved predecessor-next
	 * aborts the commit, and a marked pos is caught on the re-read above.
	 * See the "next"-only mark rationale at the top.
	 */
	urcu_txn_store(txn, (void **) &prev->next, pos, newp);
	urcu_txn_store(txn, (void **) &pos->prev, prev, newp);
	return 0;
}

/*
 * Insert @newp immediately before @pos in list @head.  Returns 0 / -ENOENT /
 * -ENOMEM as for insert_after.  Convenience bracket around
 * cds_bidir_list_lf_insert_before_prepare().
 */
static inline
int cds_bidir_list_lf_insert_before_rcu(struct cds_bidir_list_lf_node *newp,
		struct cds_bidir_list_lf_node *pos,
		struct cds_bidir_list_lf_head *head)
{
	struct urcu_mcas_txn txn;
	int ret, prep;

	urcu_txn_init(&txn, &head->domain);
	do {
		urcu_txn_begin(&txn);
		prep = cds_bidir_list_lf_insert_before_prepare(&txn, newp, pos);
		if (prep) {				/* -ENOENT: @pos deleted */
			urcu_txn_end(&txn);
			return prep;
		}
		ret = urcu_txn_commit(&txn);
		urcu_txn_end(&txn);
	} while (ret == URCU_TXN_STATUS_ABORT);
	return ret < 0 ? -ENOMEM : 0;		/* OK committed, -ENOMEM on OOM */
}

/*
 * Add @newp at the head of list @head (just after the sentinel).  The sentinel
 * is immortal, so this never observes a deleted anchor: it returns 0, or
 * -ENOMEM on descriptor allocation failure (never -ENOENT).
 */
static inline
int cds_bidir_list_lf_add_rcu(struct cds_bidir_list_lf_node *newp,
		struct cds_bidir_list_lf_head *head)
{
	return cds_bidir_list_lf_insert_after_rcu(newp, &head->node, head);
}

/*
 * Add @newp at the tail of list @head (just before the sentinel).  Returns 0,
 * or -ENOMEM on descriptor allocation failure (never -ENOENT).
 */
static inline
int cds_bidir_list_lf_add_tail_rcu(struct cds_bidir_list_lf_node *newp,
		struct cds_bidir_list_lf_head *head)
{
	return cds_bidir_list_lf_insert_before_rcu(newp, &head->node, head);
}

/*
 * cds_bidir_list_lf_del_prepare: record the unlink of @elem into the
 * caller-owned transaction @txn, WITHOUT committing.  Composable form of del
 * (see insert_after_prepare for the contract).  Returns 0 if the unlink was
 * recorded (when the caller's commit then returns OK, THIS call removed @elem
 * and the caller reclaims it after a grace period), or -ENOENT if @elem was
 * already deleted by a peer (nothing recorded; the caller must NOT reclaim).
 * OOM is sticky to the commit.
 */
static inline
int cds_bidir_list_lf_del_prepare(struct urcu_mcas_txn *txn,
		struct cds_bidir_list_lf_node *elem)
{
	void *en = urcu_txn_load(txn, (void **) &elem->next);
	struct cds_bidir_list_lf_node *next, *prev;

	if (cds_bidir_list_lf_is_marked(en))
		return -ENOENT;			/* already deleted by a peer */
	next = (struct cds_bidir_list_lf_node *) en;
	prev = cds_bidir_list_lf_unmark(
			urcu_txn_load(txn, (void **) &elem->prev));

	/*
	 * We rewrite &next->prev but not &next->next.  Load-validate next->next
	 * (the slot del(next) marks) so the backward unlink serializes against
	 * del(next) atomically even when the slot-sorted install reaches
	 * &next->prev first.  A marked successor is itself being deleted: retry
	 * (elem->next will have moved to the successor's successor).
	 *
	 * Skip the guard when next == prev (a node whose two neighbours coincide,
	 * e.g. the sole element, where both are the sentinel): the &prev->next
	 * forward-unlink store below is then the SAME slot &next->next and already
	 * serializes it.  Recording the guard there too would put two records on
	 * one slot with different expected-old values (the guard reads the slot
	 * fresh, the store hard-codes @elem) -- the descriptor's per-slot reconcile
	 * would silently merge them into a bogus record under -DNDEBUG, committing
	 * a corrupt edge (a marked-but-still-linked node).
	 */
	if (next != prev &&
			cds_bidir_list_lf_is_marked(urcu_txn_load_validate(
				txn, (void **) &next->next)))
		return -EAGAIN;			/* successor (a neighbour) deleted: retry */

	/*
	 * Mark elem (logical delete), then unlink both neighbour edges.
	 * &prev->next (old value elem) is the slot a racing
	 * insert_after(prev) / insert_before(elem) shares with us, so the MCAS
	 * serializes insert against delete on every adjacency; marking
	 * &elem->next is what makes a racing insert_after(elem) (or
	 * insert_before(elem)) terminate with -ENOENT.  See the "next"-only
	 * mark rationale at the top of this file.
	 */
	urcu_txn_store(txn, (void **) &elem->next, next,
			cds_bidir_list_lf_set_mark(next));
	urcu_txn_store(txn, (void **) &prev->next, elem, next);
	urcu_txn_store(txn, (void **) &next->prev, elem, prev);
	return 0;
}

/*
 * Remove @elem from list @head.  Returns 1 if THIS call removed it (the caller
 * reclaims @elem after a grace period), 0 if it was already deleted (the caller
 * must NOT reclaim), or -ENOMEM on descriptor allocation failure.  Convenience
 * bracket around cds_bidir_list_lf_del_prepare().
 */
static inline
int cds_bidir_list_lf_del_rcu(struct cds_bidir_list_lf_node *elem,
		struct cds_bidir_list_lf_head *head)
{
	struct urcu_mcas_txn txn;
	int ret, prep;

	urcu_txn_init(&txn, &head->domain);
	for (;;) {
		urcu_txn_begin(&txn);
		prep = cds_bidir_list_lf_del_prepare(&txn, elem);
		if (prep == -EAGAIN) {			/* successor moved: retry */
			urcu_txn_conflict(&txn);	/* age so a hot slot escalates */
			urcu_txn_end(&txn);
			continue;
		}
		if (prep) {				/* -ENOENT: already deleted */
			urcu_txn_end(&txn);
			return 0;			/* not removed by this call */
		}
		ret = urcu_txn_commit(&txn);
		urcu_txn_end(&txn);
		if (ret != URCU_TXN_STATUS_ABORT)	/* ABORT: neighbours changed */
			break;
	}
	return ret < 0 ? -ENOMEM : 1;		/* 1 removed by this call, -ENOMEM */
}

/*
 * cds_bidir_list_lf_replace_prepare: record the in-place replacement of @old by
 * @newp into the caller-owned transaction @txn, WITHOUT committing.  Composable
 * form of replace (see insert_after_prepare for the contract).  @newp takes
 * @old's position -- &prev->next and &next->prev swing to @newp -- while @old is
 * logically removed (its next is marked exactly as del does).  This touches the
 * SAME slots as del_prepare (only the new values differ), so it inherits del's
 * serialization against every adjacent insert/delete; in particular a racing
 * del(old)/insert_after(old) sees the mark and terminates with -ENOENT, and a
 * reader standing on @old escapes forward to @old's old successor (it linearizes
 * before the replace -- @newp is reached afresh through @prev).  Returns 0 if
 * recorded (on a committed OK, THIS call replaced @old and the caller reclaims
 * @old after a grace period), -ENOENT if @old was already deleted/replaced by a
 * peer (nothing recorded; do NOT reclaim), or -EAGAIN if a neighbour is
 * mid-deletion (retry).  OOM is sticky to the commit.
 */
static inline
int cds_bidir_list_lf_replace_prepare(struct urcu_mcas_txn *txn,
		struct cds_bidir_list_lf_node *newp,
		struct cds_bidir_list_lf_node *old)
{
	void *en = urcu_txn_load(txn, (void **) &old->next);
	struct cds_bidir_list_lf_node *next, *prev;

	if (cds_bidir_list_lf_is_marked(en))
		return -ENOENT;			/* @old already deleted/replaced by a peer */
	next = (struct cds_bidir_list_lf_node *) en;
	prev = cds_bidir_list_lf_unmark(
			urcu_txn_load(txn, (void **) &old->prev));

	/*
	 * We rewrite &next->prev but not &next->next.  Load-validate next->next
	 * (the slot del(next) marks) so the backward edge serializes against
	 * del(next) atomically even when the slot-sorted install reaches
	 * &next->prev first -- identical to del_prepare, including the next == prev
	 * skip (the &prev->next store already serializes that slot; recording the
	 * guard there too would collide into a bogus merged record -- see
	 * del_prepare).
	 */
	if (next != prev &&
			cds_bidir_list_lf_is_marked(urcu_txn_load_validate(
				txn, (void **) &next->next)))
		return -EAGAIN;			/* successor (a neighbour) deleted: retry */

	/* Build @newp's links invisibly, then swing both neighbour edges to it. */
	newp->next = next;
	newp->prev = prev;

	/*
	 * Mark &old->next (logical removal of @old, as del) and point both
	 * neighbours at @newp instead of skipping.  &prev->next (old value @old)
	 * is the slot a racing insert_after(prev)/del(prev)/del(old) shares with
	 * us; marking &old->next is what makes a racing insert_after(old) or
	 * del(old) terminate with -ENOENT.  See the "next"-only mark rationale at
	 * the top of this file.
	 */
	urcu_txn_store(txn, (void **) &old->next, next,
			cds_bidir_list_lf_set_mark(next));
	urcu_txn_store(txn, (void **) &prev->next, old, newp);
	urcu_txn_store(txn, (void **) &next->prev, old, newp);
	return 0;
}

/*
 * Replace @old with @newp atomically with respect to RCU readers.  Returns 0 on
 * success (the caller reclaims @old after a grace period), -ENOENT if @old was
 * already deleted/replaced, or -ENOMEM on descriptor allocation failure.
 * Convenience bracket around cds_bidir_list_lf_replace_prepare().
 */
static inline
int cds_bidir_list_lf_replace_rcu(struct cds_bidir_list_lf_node *newp,
		struct cds_bidir_list_lf_node *old,
		struct cds_bidir_list_lf_head *head)
{
	struct urcu_mcas_txn txn;
	int ret, prep;

	urcu_txn_init(&txn, &head->domain);
	for (;;) {
		urcu_txn_begin(&txn);
		prep = cds_bidir_list_lf_replace_prepare(&txn, newp, old);
		if (prep == -EAGAIN) {			/* a neighbour moved: retry */
			urcu_txn_conflict(&txn);	/* age so a hot slot escalates */
			urcu_txn_end(&txn);
			continue;
		}
		if (prep) {				/* -ENOENT: @old already gone */
			urcu_txn_end(&txn);
			return prep;
		}
		ret = urcu_txn_commit(&txn);
		urcu_txn_end(&txn);
		if (ret != URCU_TXN_STATUS_ABORT)	/* ABORT: neighbours changed */
			break;
	}
	return ret < 0 ? -ENOMEM : 0;		/* 0 replaced (reclaim @old after GP), -ENOMEM */
}

#define cds_bidir_list_lf_entry(ptr, type, member) \
	caa_container_of(ptr, type, member)

/* Iterate forward / backward (within an RCU read-side section). */
#define cds_bidir_list_lf_for_each_rcu(pos, head) \
	for (pos = cds_bidir_list_lf_next_rcu(&(head)->node); \
		(pos) != &(head)->node; \
		pos = cds_bidir_list_lf_next_rcu(pos))

#define cds_bidir_list_lf_for_each_reverse_rcu(pos, head) \
	for (pos = cds_bidir_list_lf_prev_rcu(&(head)->node); \
		(pos) != &(head)->node; \
		pos = cds_bidir_list_lf_prev_rcu(pos))

#define cds_bidir_list_lf_for_each_entry_rcu(pos, head, member) \
	for (pos = cds_bidir_list_lf_entry( \
			cds_bidir_list_lf_next_rcu(&(head)->node), \
			__typeof__(*(pos)), member); \
		&(pos)->member != &(head)->node; \
		pos = cds_bidir_list_lf_entry( \
			cds_bidir_list_lf_next_rcu(&(pos)->member), \
			__typeof__(*(pos)), member))

#define cds_bidir_list_lf_for_each_entry_reverse_rcu(pos, head, member) \
	for (pos = cds_bidir_list_lf_entry( \
			cds_bidir_list_lf_prev_rcu(&(head)->node), \
			__typeof__(*(pos)), member); \
		&(pos)->member != &(head)->node; \
		pos = cds_bidir_list_lf_entry( \
			cds_bidir_list_lf_prev_rcu(&(pos)->member), \
			__typeof__(*(pos)), member))

#ifdef __cplusplus
}
#endif

#endif	/* _URCU_RCU_BIDIR_LIST_LOCKFREE_H */
