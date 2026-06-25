// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

#ifndef _URCU_FAIR_MUTEX_H
#define _URCU_FAIR_MUTEX_H

/*
 * Fair (FIFO) mutex: an MCS-style queue lock built on wfcqueue.
 *
 * cds_fair_mutex_lock() acquires the lock in strict arrival order;
 * cds_fair_mutex_unlock() hands it to the next waiter.  The caller supplies a
 * cds_fair_mutex_node (typically on the stack) that lives from lock to unlock --
 * it is this thread's place in the queue.  Acquisition is wait-free (a single
 * xchg appends the node); the holder is the queue's first node and runs the
 * critical section, then on unlock dequeues itself and grants its successor.
 * Only the current holder runs a dequeue-class wfcqueue operation, so mutual
 * exclusion follows from the single-holder invariant -- no internal lock.
 *
 *   - was-empty append  -> acquire at once (no predecessor to grant us)
 *   - non-empty append  -> park until the predecessor grants us
 *   - unlock: dequeue self; if last, the lock is now free (the next arrival
 *     self-elects); else grant the new first node.
 *
 * The park/grant layer (WAITING/GRANTED/RUNNING/TEARDOWN + futex) mirrors
 * urcu-wait.h's adaptative wait/wake, including the teardown handshake that
 * prevents the granter's futex_wake from touching a node the woken thread may
 * already have reused.
 *
 * The append xchg->link window can still be preempted; an unlock may then
 * busy-wait in wfcqueue's sync_next for a bounded preemption delay.  That window
 * is left for a possible later rseq (time-slice extension) pass.
 */

#include <poll.h>
#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#include <urcu/wfcqueue.h>
#include <urcu/uatomic.h>
#include <urcu/futex.h>
#include <urcu/compiler.h>
#include <urcu/arch.h>
#include <urcu/assert.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CDS_FAIR_MUTEX_WAIT_ATTEMPTS	1000

enum cds_fair_mutex_state {
	CDS_FAIR_MUTEX_WAITING	= 0,		/* futex compares against this */
	CDS_FAIR_MUTEX_GRANTED	= (1 << 0),
	CDS_FAIR_MUTEX_RUNNING	= (1 << 1),
	CDS_FAIR_MUTEX_TEARDOWN	= (1 << 2),
};

struct cds_fair_mutex {
	struct __cds_wfcq_head head;	/* non-locking head */
	struct cds_wfcq_tail tail;
};

struct cds_fair_mutex_node {
	struct cds_wfcq_node node;
	int32_t state;			/* enum cds_fair_mutex_state */
};

static inline
void cds_fair_mutex_init(struct cds_fair_mutex *t)
{
	__cds_wfcq_init(&t->head, &t->tail);
}

/*
 * Grant the lock to a successor. Mirrors urcu_adaptative_wake_up: the RUNNING
 * check lets us skip the futex_wake if the successor is already spinning, and
 * the unconditional TEARDOWN store releases it to reuse its node once we are
 * done touching its state word.
 */
static inline
void cds_fair_mutex_grant(struct cds_fair_mutex_node *w)
{
	urcu_posix_assert(uatomic_load(&w->state) == CDS_FAIR_MUTEX_WAITING);
	uatomic_store(&w->state, CDS_FAIR_MUTEX_GRANTED, CMM_RELEASE);
	if (!(uatomic_load(&w->state) & CDS_FAIR_MUTEX_RUNNING)) {
		if (futex_noasync(&w->state, FUTEX_WAKE, 1, NULL, NULL, 0) < 0)
			abort();
	}
	/* Allow the successor to tear down / reuse its node. */
	uatomic_or_mo(&w->state, CDS_FAIR_MUTEX_TEARDOWN, CMM_RELEASE);
}

/*
 * Park until granted. Mirrors urcu_adaptative_busy_wait.
 */
static inline
void cds_fair_mutex_park(struct cds_fair_mutex_node *w)
{
	unsigned int i;

	cmm_smp_rmb();
	for (i = 0; i < CDS_FAIR_MUTEX_WAIT_ATTEMPTS; i++) {
		if (uatomic_load(&w->state, CMM_ACQUIRE) != CDS_FAIR_MUTEX_WAITING)
			goto granted;
		caa_cpu_relax();
	}
	while (uatomic_load(&w->state, CMM_ACQUIRE) == CDS_FAIR_MUTEX_WAITING) {
		if (!futex_noasync(&w->state, FUTEX_WAIT, CDS_FAIR_MUTEX_WAITING,
				NULL, NULL, 0)) {
			/* Re-check in user-space (spurious wakeup). */
			continue;
		}
		switch (errno) {
		case EAGAIN:	/* Value already changed. */
			goto granted;
		case EINTR:	/* Retry on signal. */
			break;
		default:
			abort();
		}
	}
granted:
	/* Tell granter we are running (it may then skip the futex_wake). */
	uatomic_or(&w->state, CDS_FAIR_MUTEX_RUNNING);

	/* Wait until granter is done touching our state word. */
	for (i = 0; i < CDS_FAIR_MUTEX_WAIT_ATTEMPTS; i++) {
		if (uatomic_load(&w->state) & CDS_FAIR_MUTEX_TEARDOWN)
			return;
		caa_cpu_relax();
	}
	while (!(uatomic_load(&w->state, CMM_ACQUIRE) & CDS_FAIR_MUTEX_TEARDOWN))
		(void) poll(NULL, 0, 10);
}

/*
 * Acquire the lock: append our node and wait our turn.
 */
static inline
void cds_fair_mutex_lock(struct cds_fair_mutex *t, struct cds_fair_mutex_node *w)
{
	cds_wfcq_node_init(&w->node);
	uatomic_store(&w->state, CDS_FAIR_MUTEX_WAITING, CMM_RELAXED);

	/*
	 * Wait-free enqueue. Returns true if there was a predecessor in the
	 * queue (so we must wait to be granted), false if the queue was empty
	 * (so we are the head and self-elect as holder).
	 */
	if (cds_wfcq_enqueue(&t->head, &t->tail, &w->node))
		cds_fair_mutex_park(w);
}

/*
 * Release the lock and hand off to the FIFO successor.  Returns true if we
 * were the last holder (the queue is now empty), false if a successor was
 * granted the lock.  The "last" return lets a caller tear down episode-scoped
 * shared state exactly when the lock drains.
 */
static inline
bool cds_fair_mutex_unlock(struct cds_fair_mutex *t, struct cds_fair_mutex_node *w)
{
	struct cds_wfcq_node *self, *succ;
	int state = 0;

	/*
	 * We are the holder == first node. Dequeue ourselves. Only the holder
	 * runs dequeue-class ops, so this is serialized by the single-holder
	 * invariant; no internal lock required.
	 */
	self = __cds_wfcq_dequeue_with_state_blocking(&t->head, &t->tail,
			&state);
	urcu_posix_assert(self == &w->node);
	(void) self;

	if (state & CDS_WFCQ_STATE_LAST) {
		/*
		 * We were the last node: dequeue's cmpxchg reset the tail and
		 * released the lock. The next enqueuer will find the queue
		 * empty and self-elect. Nobody to grant.
		 */
		return true;
	}

	/*
	 * A successor exists; dequeue's sync_next has already linked it as the
	 * new first node. Grant it the lock.
	 */
	succ = __cds_wfcq_first_blocking(&t->head, &t->tail);
	urcu_posix_assert(succ != NULL);
	cds_fair_mutex_grant(caa_container_of(succ, struct cds_fair_mutex_node, node));
	return false;
}

#ifdef __cplusplus
}
#endif

#endif /* _URCU_FAIR_MUTEX_H */
