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
#include <time.h>
#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#include <urcu/wfcqueue.h>
#include <urcu/uatomic.h>
#include <urcu/futex.h>
#include <sched.h>

#include <urcu/compiler.h>
#include <urcu/arch.h>
#include <urcu/assert.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifdef CDS_FAIR_MUTEX_DBG_POLL
/* Probe: count teardown-wait sleep rungs (diagnosis builds only). */
static __thread unsigned long cds_fmtx_dbg_polls __attribute__((unused));
#endif

/*
 * One step of the graduated teardown wait: us-scale sleeps first (the
 * granter's GRANTED->TEARDOWN window is microseconds; this transition
 * is UNWAKEABLE by design -- after TEARDOWN the granter must never
 * touch our node again, so there is no futex to wake), then
 * millisecond polls doubling to a 16ms cap.  Twin of wfcqueue's
 * ___cds_wfcq_wait_rung, kept local because that helper is
 * LGPL-static and this header must stand alone.
 */
static inline void cds_fair_mutex_wait_rung(int rung)
{
	if (rung < 7) {
		struct timespec ts = { 0, (10L << rung) * 1000L };

		(void) nanosleep(&ts, NULL);
	} else if (rung < 11) {
		(void) poll(NULL, 0, 1 << (rung - 7));
	} else {
		(void) poll(NULL, 0, 16);
	}
}

#define CDS_FAIR_MUTEX_WAIT_ATTEMPTS	1000
/*
 * Grant-side confirm budget: after publishing GRANTED, the granter briefly
 * waits for the successor to acknowledge with RUNNING before falling back to
 * futex_wake.  A SPINNING successor sets RUNNING within a cross-core round-trip,
 * so we skip the (wasted) wake syscall entirely; a PARKED successor never sets
 * RUNNING, the budget expires, and we wake it.  Lost-wakeup-safe: a successor
 * parks via FUTEX_WAIT(state == WAITING), which returns EAGAIN once we have
 * stored GRANTED, so it cannot miss the grant no matter how this races.  Sized
 * to the cross-core RUNNING-ack round-trip so a parked successor pays little.
 */
#ifndef CDS_FAIR_MUTEX_GRANT_CONFIRM_ATTEMPTS
#define CDS_FAIR_MUTEX_GRANT_CONFIRM_ATTEMPTS	500
#endif

enum cds_fair_mutex_state {
	CDS_FAIR_MUTEX_WAITING	= 0,		/* futex compares against this */
	CDS_FAIR_MUTEX_GRANTED	= (1 << 0),
	CDS_FAIR_MUTEX_RUNNING	= (1 << 1),
	CDS_FAIR_MUTEX_TEARDOWN	= (1 << 2),
};

struct cds_fair_mutex {
	struct __cds_wfcq_head head;	/* non-locking head */
	struct cds_wfcq_tail tail;
	int owner_cpu;			/* sched_getcpu() of the current holder, or -1 */
};

struct cds_fair_mutex_node {
	struct cds_wfcq_node node;
	int32_t state;			/* enum cds_fair_mutex_state */
	int cpu;			/* sched_getcpu() at lock time; the granter
					 * publishes it into owner_cpu at hand-off so
					 * owner_cpu names the INCOMING holder, never a
					 * departed one's stale CPU. */
};

static inline
void cds_fair_mutex_init(struct cds_fair_mutex *t)
{
	__cds_wfcq_init(&t->head, &t->tail);
	uatomic_store(&t->owner_cpu, -1, CMM_RELAXED);
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
	unsigned int i;

	urcu_posix_assert(uatomic_load(&w->state) == CDS_FAIR_MUTEX_WAITING);
	uatomic_store(&w->state, CDS_FAIR_MUTEX_GRANTED, CMM_RELEASE);
	/*
	 * Briefly confirm a spinning successor's RUNNING ack before waking, to
	 * elide the wasted futex_wake syscall on a user-space hand-off (see
	 * CDS_FAIR_MUTEX_GRANT_CONFIRM_ATTEMPTS).
	 */
	for (i = 0; i < CDS_FAIR_MUTEX_GRANT_CONFIRM_ATTEMPTS; i++) {
		if (uatomic_load(&w->state) & CDS_FAIR_MUTEX_RUNNING)
			goto running;
		caa_cpu_relax();
	}
	if (!(uatomic_load(&w->state) & CDS_FAIR_MUTEX_RUNNING)) {
		if (futex_noasync(&w->state, FUTEX_WAKE, 1, NULL, NULL, 0) < 0)
			abort();
	}
running:
	/* Allow the successor to tear down / reuse its node. */
	uatomic_or_mo(&w->state, CDS_FAIR_MUTEX_TEARDOWN, CMM_RELEASE);
}

/*
 * Park until granted. Mirrors urcu_adaptative_busy_wait.
 */
static inline
void cds_fair_mutex_park(struct cds_fair_mutex *t, struct cds_fair_mutex_node *w)
{
	unsigned int i, attempts;
	int owner = uatomic_load(&t->owner_cpu, CMM_RELAXED);

	/*
	 * If the current holder runs on OUR CPU, spinning would only steal the
	 * core it needs to finish and hand off -- park at once and yield.  Any
	 * other case (remote or unknown holder) uses the normal spin budget.
	 */
	attempts = (owner >= 0 && owner == sched_getcpu())
			? 0 : CDS_FAIR_MUTEX_WAIT_ATTEMPTS;

	cmm_smp_rmb();
	for (i = 0; i < attempts; i++) {
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

	/*
	 * Wait until granter is done touching our state word.  The bounded spin
	 * breaks (rather than returns) so it always falls through to the acquire
	 * load below: that CMM_ACQUIRE is what synchronizes with the granter's
	 * TEARDOWN release, ordering the lock hand-off and the granter's writes to
	 * our node before we return and reuse it.  Returning straight from the
	 * (relaxed) spin would skip the acquire on the common path.  Mirrors
	 * urcu-wait.h's urcu_adaptative_busy_wait().
	 */
	for (i = 0; i < CDS_FAIR_MUTEX_WAIT_ATTEMPTS; i++) {
		if (uatomic_load(&w->state) & CDS_FAIR_MUTEX_TEARDOWN)
			break;
		caa_cpu_relax();
	}
	{
		int rung = 0;

		while (!(uatomic_load(&w->state, CMM_ACQUIRE) &
				CDS_FAIR_MUTEX_TEARDOWN)) {
#ifdef CDS_FAIR_MUTEX_DBG_POLL
			cds_fmtx_dbg_polls++;
#endif
			cds_fair_mutex_wait_rung(rung);
			if (rung < 11)
				rung++;
		}
	}
	urcu_posix_assert(uatomic_load(&w->state) & CDS_FAIR_MUTEX_TEARDOWN);
}

/*
 * Acquire the lock: append our node and wait our turn.
 */
static inline
void cds_fair_mutex_lock(struct cds_fair_mutex *t, struct cds_fair_mutex_node *w)
{
	int mycpu = sched_getcpu();

	cds_wfcq_node_init(&w->node);
	uatomic_store(&w->state, CDS_FAIR_MUTEX_WAITING, CMM_RELAXED);
	uatomic_store(&w->cpu, mycpu, CMM_RELAXED);	/* for our granter to publish */

	/*
	 * Wait-free enqueue. Returns true if there was a predecessor in the
	 * queue (so we must wait to be granted), false if the queue was empty
	 * (so we are the head and self-elect as holder).
	 */
	if (cds_wfcq_enqueue(&t->head, &t->tail, &w->node))
		cds_fair_mutex_park(t, w);	/* granted path: our granter already
						 * published our CPU into owner_cpu */
	else
		uatomic_store(&t->owner_cpu, mycpu, CMM_RELAXED);	/* self-elected */
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
	{
		struct cds_fair_mutex_node *sn =
			caa_container_of(succ, struct cds_fair_mutex_node, node);
		/* Publish the INCOMING holder's CPU before granting, so a waiter
		 * (incl. the just-released former holder re-enqueuing) reads the
		 * successor's CPU -- not this departing holder's stale one. */
		uatomic_store(&t->owner_cpu, uatomic_load(&sn->cpu, CMM_RELAXED),
				CMM_RELAXED);
		cds_fair_mutex_grant(sn);
	}
	return false;
}

#ifdef __cplusplus
}
#endif

#endif /* _URCU_FAIR_MUTEX_H */
