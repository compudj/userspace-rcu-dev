// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

#ifndef _URCU_WAIT_LADDER_H
#define _URCU_WAIT_LADDER_H

/*
 * Graduated wait ladder: the delay policy for busy-wait fallbacks.
 *
 * A flat poll(NULL, 0, 10) charges 10ms for losing a race whose window
 * is microseconds -- and under a FIFO handoff, one sleeper convoys
 * every waiter queued behind it.  The ladder escalates instead:
 * microsecond-scale nanosleeps first (the scheduler's timer slack
 * bounds the floor), then millisecond polls doubling to a capped
 * quantum.  A lost race then costs about what it lasts, bounded by a
 * <=2x overshoot, while a genuinely long wait converges to the old
 * flat-poll cost.
 *
 * Schedule (defaults, overridable before inclusion):
 *
 *	rung 0..6    10us << rung        (10us .. 640us, nanosleep)
 *	rung 7..10    1ms << (rung - 7)  (1, 2, 4, 8ms, poll)
 *	rung 11+     16ms                (poll, the cap)
 *
 * Typical use -- one call per re-check of the blocked condition; the
 * caller's spin budget is the ladder's rung -1:
 *
 *	struct urcu_wait_ladder wl = URCU_WAIT_LADDER_INIT;
 *
 *	while (!condition)
 *		urcu_wait_ladder_wait(&wl, MY_SPIN_ATTEMPTS);
 *
 * The state is one integer; reinitialize it when a NEW wait begins.
 * Callers that must observe the sleep's return value (e.g. a futex
 * emulation propagating EINTR) use urcu_wait_ladder_sleep_us()
 * directly, feeding it urcu_wait_ladder_rung_us().
 */

#include <poll.h>
#include <time.h>

#include <urcu/compiler.h>
#include <urcu/arch.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef URCU_WAIT_LADDER_US_BASE
#define URCU_WAIT_LADDER_US_BASE	10
#endif
#ifndef URCU_WAIT_LADDER_US_RUNGS
#define URCU_WAIT_LADDER_US_RUNGS	7
#endif
#ifndef URCU_WAIT_LADDER_MS_RUNGS
#define URCU_WAIT_LADDER_MS_RUNGS	4
#endif
#ifndef URCU_WAIT_LADDER_MS_CAP
#define URCU_WAIT_LADDER_MS_CAP		16
#endif

#define URCU_WAIT_LADDER_RUNG_CAP					\
	(URCU_WAIT_LADDER_US_RUNGS + URCU_WAIT_LADDER_MS_RUNGS)

struct urcu_wait_ladder {
	unsigned int step;
};

#define URCU_WAIT_LADDER_INIT	{ 0 }

static inline
void urcu_wait_ladder_init(struct urcu_wait_ladder *wl)
{
	wl->step = 0;
}

/* Duration of ladder rung @rung, in microseconds. */
static inline
unsigned int urcu_wait_ladder_rung_us(unsigned int rung)
{
	if (rung < URCU_WAIT_LADDER_US_RUNGS)
		return URCU_WAIT_LADDER_US_BASE << rung;
	if (rung < URCU_WAIT_LADDER_RUNG_CAP)
		return 1000U << (rung - URCU_WAIT_LADDER_US_RUNGS);
	return URCU_WAIT_LADDER_MS_CAP * 1000U;
}

/*
 * Sleep @usec microseconds: nanosleep below one millisecond, poll
 * above.  Returns the underlying call's result with its errno intact
 * (nanosleep interrupted by a signal returns -1/EINTR having slept a
 * shorter time -- for a wait loop that is merely an early re-check).
 */
static inline
int urcu_wait_ladder_sleep_us(unsigned int usec)
{
	if (usec < 1000) {
		struct timespec ts = { 0, (long) usec * 1000L };

		return nanosleep(&ts, NULL);
	}
	return poll(NULL, 0, (int) (usec / 1000));
}

/*
 * Hold the ladder at rung @max_rung: call after urcu_wait_ladder_wait()
 * at sites whose re-check cadence carries a bound (rung
 * URCU_WAIT_LADDER_US_RUNGS + k sleeps (1 << k) milliseconds, so
 * e.g. + 0 caps at 1ms, + 3 at 8ms).  @spin_attempts must match the
 * wait call's.
 */
static inline
void urcu_wait_ladder_clamp(struct urcu_wait_ladder *wl,
		unsigned int spin_attempts, unsigned int max_rung)
{
	if (wl->step > spin_attempts + max_rung)
		wl->step = spin_attempts + max_rung;
}

/*
 * One blocked probe: cpu_relax through the caller's spin budget, then
 * take the next rung of the ladder, holding at the cap.
 */
static inline
void urcu_wait_ladder_wait(struct urcu_wait_ladder *wl,
		unsigned int spin_attempts)
{
	if (wl->step < spin_attempts) {
		wl->step++;
		caa_cpu_relax();
		return;
	}
	(void) urcu_wait_ladder_sleep_us(
		urcu_wait_ladder_rung_us(wl->step - spin_attempts));
	if (wl->step - spin_attempts < URCU_WAIT_LADDER_RUNG_CAP)
		wl->step++;
}

#ifdef __cplusplus
}
#endif

#endif /* _URCU_WAIT_LADDER_H */
