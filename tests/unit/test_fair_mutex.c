// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

/*
 * Unit test for the fair (FIFO) mutex (urcu/fair-mutex.h).
 *
 * Validates:
 *   - single-threaded lock/unlock sanity;
 *   - mutual exclusion under contention: a non-atomic counter incremented
 *     inside the critical section must equal the number of acquisitions, and
 *     the concurrent-holder count must never exceed 1;
 *   - liveness: all threads complete (no deadlock).
 *
 * FIFO acquisition order is by construction (the wait-free enqueue xchg is the
 * single linearization point) and is not asserted here.
 */

#define _LGPL_SOURCE

#include <pthread.h>
#include <stdlib.h>
#include <stdint.h>
#include <sched.h>

#include "tap.h"
#include <urcu/uatomic.h>
#include <urcu/fair-mutex.h>

#define NR_TESTS	4

static struct cds_fair_mutex mutex;

/* Non-atomic: only consistent if mutual exclusion holds. The whole point. */
static unsigned long protected_counter;

static int32_t nr_in_cs;
static int32_t max_in_cs;
static int32_t overlap_detected;

static int nr_threads = 8;
static unsigned long nr_iter = 50000;

static void critical_section(unsigned int *seed)
{
	int32_t cur;

	cur = uatomic_add_return(&nr_in_cs, 1);
	if (cur != 1)
		uatomic_store(&overlap_detected, 1, CMM_RELAXED);
	if (cur > uatomic_load(&max_in_cs))
		uatomic_store(&max_in_cs, cur, CMM_RELAXED);

	protected_counter++;

	/* Occasionally widen the window to provoke queuing/parking. */
	if (seed && (rand_r(seed) & 0xf) == 0)
		sched_yield();

	uatomic_add(&nr_in_cs, -1);
}

static void *worker(void *arg)
{
	unsigned long i;
	unsigned int seed = (unsigned int)(uintptr_t)arg + 1;
	struct cds_fair_mutex_node w;

	for (i = 0; i < nr_iter; i++) {
		cds_fair_mutex_lock(&mutex, &w);
		critical_section(&seed);
		cds_fair_mutex_unlock(&mutex, &w);
	}
	return NULL;
}

int main(int argc, char **argv)
{
	pthread_t *tids;
	unsigned long expected, st_iter = 1000;
	int i, ret;

	if (argc > 1)
		nr_threads = atoi(argv[1]);
	if (argc > 2)
		nr_iter = strtoul(argv[2], NULL, 10);

	plan_tests(NR_TESTS);

	cds_fair_mutex_init(&mutex);

	/* 1. Single-threaded lock/unlock sanity. */
	{
		struct cds_fair_mutex_node w;

		for (i = 0; i < (int)st_iter; i++) {
			cds_fair_mutex_lock(&mutex, &w);
			protected_counter++;
			cds_fair_mutex_unlock(&mutex, &w);
		}
		ok(protected_counter == st_iter,
			"single-threaded: %lu lock/unlock cycles, counter exact",
			st_iter);
	}

	/* 2-4. Concurrent stress. */
	tids = calloc(nr_threads, sizeof(*tids));
	if (!tids)
		abort();
	for (i = 0; i < nr_threads; i++) {
		ret = pthread_create(&tids[i], NULL, worker,
				(void *)(uintptr_t)i);
		if (ret)
			abort();
	}
	for (i = 0; i < nr_threads; i++)
		pthread_join(tids[i], NULL);
	free(tids);

	expected = st_iter + (unsigned long)nr_threads * nr_iter;

	ok(!uatomic_load(&overlap_detected),
		"mutual exclusion: never more than one holder in the CS");
	ok(uatomic_load(&max_in_cs) == 1,
		"max concurrent holders observed == 1");
	ok(protected_counter == expected,
		"no lost updates: counter = %lu (expected %lu) over %d threads",
		protected_counter, expected, nr_threads);

	return exit_status();
}
