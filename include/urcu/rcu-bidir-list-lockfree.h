// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

#ifndef _URCU_RCU_BIDIR_LIST_LOCKFREE_H
#define _URCU_RCU_BIDIR_LIST_LOCKFREE_H

/*
 * rcu-bidir-list-lockfree: a bidirectional, coherent RCU list with LOCK-FREE
 * concurrent writers, built on the lock-free flip-latch MCAS engine
 * (<urcu/flip-latch-lockfree.h>).  It is the concurrent-writer sibling of
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
 * Every reader AND writer must run within an RCU read-side critical section
 * (rcu_read_lock()/unlock()) -- the read lock is what keeps an in-flight MCAS
 * descriptor alive while peers help drive it.  Read next/prev only through the
 * accessors below (they resolve the proxy and strip the mark); never touch the
 * raw fields.  Mutators take the flavor's call_rcu (for descriptor reclaim) and
 * loop internally until they commit or definitively fail; they return 0 / 1 on
 * success, -ENOENT if the anchor was deleted, -ENOMEM on descriptor OOM.
 */

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include <urcu/compiler.h>
#include <urcu/uatomic.h>
#include <urcu/call-rcu.h>
#include <urcu/flip-latch-lockfree.h>
#include <urcu-pointer.h>

#ifdef __cplusplus
extern "C" {
#endif

struct cds_bidir_list_lf_node {
	struct cds_bidir_list_lf_node *next, *prev;
};

typedef void (*cds_bidir_list_lf_call_rcu_fn)(struct rcu_head *head,
		void (*func)(struct rcu_head *head));

#define CDS_BIDIR_LIST_LF_HEAD_INIT(name)	{ .next = &(name), .prev = &(name) }

#define CDS_BIDIR_LIST_LF_HEAD(name) \
	struct cds_bidir_list_lf_node name = CDS_BIDIR_LIST_LF_HEAD_INIT(name)

static inline
void cds_bidir_list_lf_init(struct cds_bidir_list_lf_node *head)
{
	head->next = head;
	head->prev = head;
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
 * the slow path, where unmark + urcu_flip_lf_resolve handle either or both
 * bits.  The mask lists EVERY reserved low bit on a transacted slot value;
 * revisit it if more tag bits are ever added.
 */
static inline
struct cds_bidir_list_lf_node *cds_bidir_list_lf_resolve(void *raw)
{
	unsigned long v = (unsigned long) raw;

	if (caa_unlikely(v & (URCU_FLIP_LF_TAG | CDS_BIDIR_LIST_LF_MARK)))
		return cds_bidir_list_lf_unmark(urcu_flip_lf_resolve(raw));
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
int cds_bidir_list_lf_empty(struct cds_bidir_list_lf_node *head)
{
	return cds_bidir_list_lf_next_rcu(head) == head;
}

/*
 * Insert @newp immediately after @pos.  Returns 0 on success, -ENOENT if @pos
 * has been deleted, -ENOMEM on descriptor allocation failure.
 */
static inline
int cds_bidir_list_lf_insert_after_rcu(struct cds_bidir_list_lf_node *newp,
		struct cds_bidir_list_lf_node *pos,
		cds_bidir_list_lf_call_rcu_fn call_rcu_fn)
{
	for (;;) {
		void *pn = urcu_flip_lf_read((void **) &pos->next);
		struct cds_bidir_list_lf_node *succ;
		struct urcu_flip_lf_txn *t;

		if (cds_bidir_list_lf_is_marked(pn))
			return -ENOENT;			/* @pos was deleted */
		succ = (struct cds_bidir_list_lf_node *) pn;	/* unmarked successor */

		/* Build the fresh node invisibly. */
		newp->next = succ;
		newp->prev = pos;

		t = urcu_flip_lf_txn_create(2);
		if (caa_unlikely(!t))
			return -ENOMEM;
		/*
		 * pos->next: succ -> newp ; succ->prev: pos -> newp.
		 * &pos->next is the slot that serializes us against deletion:
		 * del(pos) marks it, and del(succ) rewrites it to skip succ --
		 * either makes this CAS fail its old value, so we abort and
		 * retry (and on a marked pos, return -ENOENT above).  See the
		 * "next"-only mark rationale at the top of this file.
		 */
		urcu_flip_lf_txn_add(t, (void **) &pos->next, succ, newp);
		urcu_flip_lf_txn_add(t, (void **) &succ->prev, pos, newp);
		if (urcu_flip_lf_txn_commit(t, call_rcu_fn))
			return 0;
		/* aborted: a neighbour changed -- re-read, retry, maybe -ENOENT */
	}
}

/*
 * Insert @newp immediately before @pos.  Returns 0 / -ENOENT / -ENOMEM as above.
 */
static inline
int cds_bidir_list_lf_insert_before_rcu(struct cds_bidir_list_lf_node *newp,
		struct cds_bidir_list_lf_node *pos,
		cds_bidir_list_lf_call_rcu_fn call_rcu_fn)
{
	for (;;) {
		void *pn = urcu_flip_lf_read((void **) &pos->next);
		struct cds_bidir_list_lf_node *prev;
		struct urcu_flip_lf_txn *t;

		if (cds_bidir_list_lf_is_marked(pn))
			return -ENOENT;			/* @pos was deleted */
		prev = cds_bidir_list_lf_unmark(
				urcu_flip_lf_read((void **) &pos->prev));

		newp->next = pos;
		newp->prev = prev;

		t = urcu_flip_lf_txn_create(2);
		if (caa_unlikely(!t))
			return -ENOMEM;
		/*
		 * prev->next: pos -> newp ; pos->prev: prev -> newp.
		 * &prev->next is the slot shared with del(prev) (which marks it)
		 * and del(pos) (which rewrites it to skip pos): a moved
		 * predecessor-next aborts us, and a marked pos is caught on the
		 * re-read above.  See the "next"-only mark rationale at the top.
		 */
		urcu_flip_lf_txn_add(t, (void **) &prev->next, pos, newp);
		urcu_flip_lf_txn_add(t, (void **) &pos->prev, prev, newp);
		if (urcu_flip_lf_txn_commit(t, call_rcu_fn))
			return 0;
	}
}

/* Add @newp at the head (just after @head).  Always succeeds (head is immortal). */
static inline
int cds_bidir_list_lf_add_rcu(struct cds_bidir_list_lf_node *newp,
		struct cds_bidir_list_lf_node *head,
		cds_bidir_list_lf_call_rcu_fn call_rcu_fn)
{
	return cds_bidir_list_lf_insert_after_rcu(newp, head, call_rcu_fn);
}

/* Add @newp at the tail (just before @head).  Always succeeds. */
static inline
int cds_bidir_list_lf_add_tail_rcu(struct cds_bidir_list_lf_node *newp,
		struct cds_bidir_list_lf_node *head,
		cds_bidir_list_lf_call_rcu_fn call_rcu_fn)
{
	return cds_bidir_list_lf_insert_before_rcu(newp, head, call_rcu_fn);
}

/*
 * Remove @elem.  Returns 1 if THIS call removed it (the caller reclaims @elem
 * after a grace period), 0 if it was already deleted (the caller must NOT
 * reclaim), or -ENOMEM on descriptor allocation failure.
 */
static inline
int cds_bidir_list_lf_del_rcu(struct cds_bidir_list_lf_node *elem,
		cds_bidir_list_lf_call_rcu_fn call_rcu_fn)
{
	for (;;) {
		void *en = urcu_flip_lf_read((void **) &elem->next);
		struct cds_bidir_list_lf_node *next, *prev;
		struct urcu_flip_lf_txn *t;

		if (cds_bidir_list_lf_is_marked(en))
			return 0;			/* already deleted by a peer */
		next = (struct cds_bidir_list_lf_node *) en;
		prev = cds_bidir_list_lf_unmark(
				urcu_flip_lf_read((void **) &elem->prev));

		t = urcu_flip_lf_txn_create(3);
		if (caa_unlikely(!t))
			return -ENOMEM;
		/*
		 * Mark elem (logical delete), then unlink both neighbour edges.
		 * &prev->next (old value elem) is the slot a racing
		 * insert_after(prev) / insert_before(elem) shares with us, so
		 * the MCAS serializes insert against delete on every adjacency;
		 * marking &elem->next is what makes a racing insert_after(elem)
		 * (or insert_before(elem)) terminate with -ENOENT.  See the
		 * "next"-only mark rationale at the top of this file.
		 */
		urcu_flip_lf_txn_add(t, (void **) &elem->next, next,
				cds_bidir_list_lf_set_mark(next));
		urcu_flip_lf_txn_add(t, (void **) &prev->next, elem, next);
		urcu_flip_lf_txn_add(t, (void **) &next->prev, elem, prev);
		if (urcu_flip_lf_txn_commit(t, call_rcu_fn))
			return 1;			/* removed by this call */
		/* aborted: neighbours changed -- retry, maybe find it deleted */
	}
}

#define cds_bidir_list_lf_entry(ptr, type, member) \
	caa_container_of(ptr, type, member)

/* Iterate forward / backward (within an RCU read-side section). */
#define cds_bidir_list_lf_for_each_rcu(pos, head) \
	for (pos = cds_bidir_list_lf_next_rcu(head); \
		(pos) != (head); \
		pos = cds_bidir_list_lf_next_rcu(pos))

#define cds_bidir_list_lf_for_each_reverse_rcu(pos, head) \
	for (pos = cds_bidir_list_lf_prev_rcu(head); \
		(pos) != (head); \
		pos = cds_bidir_list_lf_prev_rcu(pos))

#define cds_bidir_list_lf_for_each_entry_rcu(pos, head, member) \
	for (pos = cds_bidir_list_lf_entry(cds_bidir_list_lf_next_rcu(head), \
			__typeof__(*(pos)), member); \
		&(pos)->member != (head); \
		pos = cds_bidir_list_lf_entry( \
			cds_bidir_list_lf_next_rcu(&(pos)->member), \
			__typeof__(*(pos)), member))

#define cds_bidir_list_lf_for_each_entry_reverse_rcu(pos, head, member) \
	for (pos = cds_bidir_list_lf_entry(cds_bidir_list_lf_prev_rcu(head), \
			__typeof__(*(pos)), member); \
		&(pos)->member != (head); \
		pos = cds_bidir_list_lf_entry( \
			cds_bidir_list_lf_prev_rcu(&(pos)->member), \
			__typeof__(*(pos)), member))

#ifdef __cplusplus
}
#endif

#endif	/* _URCU_RCU_BIDIR_LIST_LOCKFREE_H */
