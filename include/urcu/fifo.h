// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

#ifndef _URCU_FIFO_H
#define _URCU_FIFO_H

/*
 * FIFO turnstile (fair, lock-free handoff) on top of wfcqueue.
 *
 * Participants enqueue themselves (wait-free, single xchg). The first node in
 * the queue is the token holder and runs its critical section. On exit the
 * holder dequeues itself and hands the token to its successor (MCS-style).
 * Only the current holder ever performs a dequeue-class wfcqueue operation, so
 * the dequeue/first mutual-exclusion requirement is satisfied by the
 * single-token invariant -- no lock.
 *
 *   - was-empty enqueue return  -> self-elect as holder (no grant)
 *   - non-empty enqueue return  -> park until predecessor grants us
 *   - exit: dequeue self; if STATE_LAST, token released (next enqueuer
 *     self-elects); else grant the new first node.
 *
 * The park/grant layer (WAITING/GRANTED/RUNNING/TEARDOWN + futex) mirrors
 * urcu-wait.h's adaptative wait/wake, including the teardown handshake that
 * prevents the granter's futex_wake from touching a node the woken thread may
 * already have reused.
 *
 * The enqueue xchg->link window can still be preempted; a holder's exit may
 * then busy-wait in wfcqueue's sync_next for a bounded preemption delay. That
 * window is left for a possible later rseq (time-slice extension) pass.
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

#define CDS_FIFO_WAIT_ATTEMPTS	1000

enum cds_fifo_state {
	CDS_FIFO_WAITING	= 0,		/* futex compares against this */
	CDS_FIFO_GRANTED	= (1 << 0),
	CDS_FIFO_RUNNING	= (1 << 1),
	CDS_FIFO_TEARDOWN	= (1 << 2),
};

struct cds_fifo_turnstile {
	struct __cds_wfcq_head head;	/* non-locking head */
	struct cds_wfcq_tail tail;
};

struct cds_fifo_waiter {
	struct cds_wfcq_node node;
	int32_t state;			/* enum cds_fifo_state */
};

static inline
void cds_fifo_turnstile_init(struct cds_fifo_turnstile *t)
{
	__cds_wfcq_init(&t->head, &t->tail);
}

/*
 * Grant the token to a successor. Mirrors urcu_adaptative_wake_up: the RUNNING
 * check lets us skip the futex_wake if the successor is already spinning, and
 * the unconditional TEARDOWN store releases it to reuse its node once we are
 * done touching its state word.
 */
static inline
void cds_fifo_grant(struct cds_fifo_waiter *w)
{
	urcu_posix_assert(uatomic_load(&w->state) == CDS_FIFO_WAITING);
	uatomic_store(&w->state, CDS_FIFO_GRANTED, CMM_RELEASE);
	if (!(uatomic_load(&w->state) & CDS_FIFO_RUNNING)) {
		if (futex_noasync(&w->state, FUTEX_WAKE, 1, NULL, NULL, 0) < 0)
			abort();
	}
	/* Allow the successor to tear down / reuse its node. */
	uatomic_or_mo(&w->state, CDS_FIFO_TEARDOWN, CMM_RELEASE);
}

/*
 * Park until granted. Mirrors urcu_adaptative_busy_wait.
 */
static inline
void cds_fifo_park(struct cds_fifo_waiter *w)
{
	unsigned int i;

	cmm_smp_rmb();
	for (i = 0; i < CDS_FIFO_WAIT_ATTEMPTS; i++) {
		if (uatomic_load(&w->state, CMM_ACQUIRE) != CDS_FIFO_WAITING)
			goto granted;
		caa_cpu_relax();
	}
	while (uatomic_load(&w->state, CMM_ACQUIRE) == CDS_FIFO_WAITING) {
		if (!futex_noasync(&w->state, FUTEX_WAIT, CDS_FIFO_WAITING,
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
	uatomic_or(&w->state, CDS_FIFO_RUNNING);

	/* Wait until granter is done touching our state word. */
	for (i = 0; i < CDS_FIFO_WAIT_ATTEMPTS; i++) {
		if (uatomic_load(&w->state) & CDS_FIFO_TEARDOWN)
			return;
		caa_cpu_relax();
	}
	while (!(uatomic_load(&w->state, CMM_ACQUIRE) & CDS_FIFO_TEARDOWN))
		(void) poll(NULL, 0, 10);
}

/*
 * Acquire the turnstile: take a FIFO ticket and wait our turn.
 */
static inline
void cds_fifo_enter(struct cds_fifo_turnstile *t, struct cds_fifo_waiter *w)
{
	cds_wfcq_node_init(&w->node);
	uatomic_store(&w->state, CDS_FIFO_WAITING, CMM_RELAXED);

	/*
	 * Wait-free enqueue. Returns true if there was a predecessor in the
	 * queue (so we must wait to be granted), false if the queue was empty
	 * (so we are the head and self-elect as holder).
	 */
	if (cds_wfcq_enqueue(&t->head, &t->tail, &w->node))
		cds_fifo_park(w);
}

/*
 * Release the turnstile and hand off to the FIFO successor.
 */
static inline
void cds_fifo_exit(struct cds_fifo_turnstile *t, struct cds_fifo_waiter *w)
{
	struct cds_wfcq_node *self, *succ;
	int state = 0;

	/*
	 * We are the holder == first node. Dequeue ourselves. Only the holder
	 * runs dequeue-class ops, so this is serialized by the single-token
	 * invariant; no lock required.
	 */
	self = __cds_wfcq_dequeue_with_state_blocking(&t->head, &t->tail,
			&state);
	urcu_posix_assert(self == &w->node);
	(void) self;

	if (state & CDS_WFCQ_STATE_LAST) {
		/*
		 * We were the last node: dequeue's cmpxchg reset the tail and
		 * released the token. The next enqueuer will find the queue
		 * empty and self-elect. Nobody to grant.
		 */
		return;
	}

	/*
	 * A successor exists; dequeue's sync_next has already linked it as the
	 * new first node. Grant it the token.
	 */
	succ = __cds_wfcq_first_blocking(&t->head, &t->tail);
	urcu_posix_assert(succ != NULL);
	cds_fifo_grant(caa_container_of(succ, struct cds_fifo_waiter, node));
}

#ifdef __cplusplus
}
#endif

#endif /* _URCU_FIFO_H */
