// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

#ifndef _URCU_GETCPU_H
#define _URCU_GETCPU_H

/*
 * urcu_getcpu(): the cpu the calling thread runs on, or a negative value where
 * that cannot be told.  The value is a hint: the thread can migrate the
 * instant after.
 *
 * A C library that registers rseq for every thread (glibc >= 2.35) publishes
 * where the area is, and its own sched_getcpu() returns the area's cpu_id --
 * behind the PLT, a frame and the stack protector.  Read it here instead, an
 * inlined TLS load (~0.3 ns against ~3 ns).  The kernel rewrites cpu_id
 * whenever it moves the thread, hence a relaxed atomic load.  A negative value
 * means rseq is not registered for this thread (the tunable is off, or the
 * kernel lacks it), and urcu_getcpu_fallback() answers instead.
 *
 * urcu_getcpu_fallback() is sched_getcpu() called from liburcu-common, or -1
 * where the platform has no equivalent.  It is out of line because the C
 * library declares sched_getcpu() only under _GNU_SOURCE, which a header
 * cannot define for a translation unit that has already included a system
 * header.
 */

#include <stdlib.h>			/* a C library header, for __GLIBC_PREREQ */

#include <urcu/compiler.h>
#include <urcu/uatomic.h>

/*
 * glibc exports __rseq_offset since 2.35; the thread pointer comes from a
 * compiler builtin.
 */
#if defined(__GLIBC__) && defined(__GLIBC_PREREQ) && defined(__has_builtin)
# if __GLIBC_PREREQ(2, 35) && __has_builtin(__builtin_thread_pointer)
#  include <sys/rseq.h>
#  define URCU_GETCPU_LIBC_RSEQ	1
# endif
#endif

#ifdef __cplusplus
extern "C" {
#endif

extern int urcu_getcpu_fallback(void);

static inline
int urcu_getcpu(void)
{
#ifdef URCU_GETCPU_LIBC_RSEQ
	struct rseq *abi = (struct rseq *)
		((char *) __builtin_thread_pointer() + __rseq_offset);
	int cpu = (int) uatomic_load(&abi->cpu_id, CMM_RELAXED);

	if (caa_likely(cpu >= 0))
		return cpu;
#endif
	return urcu_getcpu_fallback();
}

#ifdef __cplusplus
}
#endif

#endif /* _URCU_GETCPU_H */
