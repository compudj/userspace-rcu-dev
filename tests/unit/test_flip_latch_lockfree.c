// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

/*
 * Direct stress test for the lock-free MCAS engine <urcu/flip-latch-lockfree.h>,
 * independent of any data structure.
 *
 * Invariant (atomicity): the shared array starts all-zero, and every
 * transaction is a "transfer" that adds and subtracts equal amounts across 2 or
 * 3 distinct words, so the total sum is invariantly 0.  A torn k-CAS (one word
 * updated, another not) would leave a non-zero sum.  With a small array and many
 * threads, transactions collide constantly, exercising the install/help/settle
 * paths and the cyclic-helping resolution hard.
 *
 * Invariant (progress): every thread completes its full op count.  A livelock or
 * deadlock in the helping protocol would hang the test (caught as a timeout),
 * and the committed-op total must equal the attempts.
 *
 * Each slot packs a per-word monotonic version (see lf_bump below) so a stored
 * word never repeats a bit pattern, and bit 0 stays free for the engine's record
 * tag.  The versioning models the real target: slots holding RCU-managed
 * pointers cannot ABA.  QSBR flavor; every worker is an RCU reader so descriptors
 * stay alive while helpers drive them.
 */

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#ifndef _LGPL_SOURCE
#define _LGPL_SOURCE	/* inline RCU primitives (TSan-visible atomics) */
#endif

#include <inttypes.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include <urcu/compiler.h>
#include <urcu-qsbr.h>
#include <urcu-call-rcu.h>
#include <urcu/flip-latch-lockfree.h>

#include "tap.h"

#define NR_TESTS	2
#define NR_WORDS	16		/* small => heavy contention */
#define NR_WORKERS	8
#define OPS_PER_WORKER	60000

static void *g_word[NR_WORDS];		/* all start at (void *) 0 */

struct worker_arg {
	unsigned int seed;
	long committed;
};

static unsigned int xs(unsigned int x)
{
	x ^= x << 13;
	x ^= x >> 17;
	x ^= x << 5;
	return x;
}

/*
 * Slot layout: [ value : top 32 bits | version : bits 1..31 | tag : bit 0 ].
 * Every successful write bumps the per-word version, so a word never repeats a
 * bit pattern.  This models the real target, where slots hold RCU-managed
 * pointers that cannot ABA (a freed node is not reused within a read-side
 * section, and a removed node is not re-linked before its grace period).  Plain
 * integer values, by contrast, ABA freely -- which is NOT representative of an
 * RCU-protected structure and is what an earlier version of this test hit.
 */
static intptr_t lf_val(uintptr_t w)
{
	return (intptr_t) w >> 32;		/* arithmetic shift sign-extends */
}

static uintptr_t lf_bump(uintptr_t w, intptr_t delta)
{
	intptr_t val = lf_val(w) + delta;
	unsigned int ver = (unsigned int) ((w >> 1) & 0x7fffffffu) + 1;

	return ((uintptr_t) (uint32_t) (int32_t) val << 32)
		| ((uintptr_t) (ver & 0x7fffffffu) << 1);
}

static void *worker(void *arg)
{
	struct worker_arg *wa = (struct worker_arg *) arg;
	unsigned int rng = wa->seed;
	long n;

	rcu_register_thread();
	for (n = 0; n < OPS_PER_WORKER; n++) {
		int i, j, k, three;
		bool ok;

		rng = xs(rng);
		three = rng & 1;
		i = (int) ((rng >> 1) % NR_WORDS);
		rng = xs(rng);
		j = (int) (rng % NR_WORDS);
		if (j == i)
			j = (j + 1) % NR_WORDS;
		rng = xs(rng);
		k = (int) (rng % NR_WORDS);
		while (k == i || k == j)
			k = (k + 1) % NR_WORDS;

		do {
			struct urcu_flip_lf_txn *t;
			uintptr_t oi, oj, ok2;

			/*
			 * The updater is an RCU reader: this critical section is
			 * what keeps a descriptor alive while peers help drive
			 * it.  A no-op in QSBR (the thread is a reader between
			 * quiescent states), but required for the memb/mb flavors.
			 */
			rcu_read_lock();
			t = urcu_flip_lf_txn_create(3);
			if (!t)
				abort();
			oi = (uintptr_t) urcu_flip_lf_read(&g_word[i]);
			oj = (uintptr_t) urcu_flip_lf_read(&g_word[j]);
			urcu_flip_lf_txn_add(t, &g_word[i],
				(void *) oi, (void *) lf_bump(oi, 2));
			if (three) {
				ok2 = (uintptr_t) urcu_flip_lf_read(&g_word[k]);
				urcu_flip_lf_txn_add(t, &g_word[j],
					(void *) oj, (void *) lf_bump(oj, 2));
				urcu_flip_lf_txn_add(t, &g_word[k],
					(void *) ok2, (void *) lf_bump(ok2, -4));
			} else {
				urcu_flip_lf_txn_add(t, &g_word[j],
					(void *) oj, (void *) lf_bump(oj, -2));
			}
			ok = urcu_flip_lf_txn_commit(t, call_rcu);
			rcu_read_unlock();
		} while (!ok);

		wa->committed++;
		rcu_quiescent_state();
	}
	rcu_unregister_thread();
	return NULL;
}

int main(void)
{
	pthread_t th[NR_WORKERS];
	struct worker_arg args[NR_WORKERS];
	long total_committed = 0;
	intptr_t sum = 0;
	int i;

	plan_tests(NR_TESTS);
	rcu_register_thread();

	for (i = 0; i < NR_WORKERS; i++) {
		args[i].seed = 0x9e3779b9u + (unsigned int) i * 2654435761u;
		args[i].committed = 0;
		pthread_create(&th[i], NULL, worker, &args[i]);
	}
	rcu_thread_offline();		/* don't stall grace periods while joined */
	for (i = 0; i < NR_WORKERS; i++) {
		pthread_join(th[i], NULL);
		total_committed += args[i].committed;
	}
	rcu_thread_online();

	for (i = 0; i < NR_WORDS; i++)
		sum += lf_val((uintptr_t) g_word[i]);

	diag("%d workers x %d ops = %ld committed; final sum = %" PRIdPTR,
		NR_WORKERS, OPS_PER_WORKER, total_committed, sum);

	ok(sum == 0,
		"k-CAS stayed atomic across concurrent transfers (sum invariant)");
	ok(total_committed == (long) NR_WORKERS * OPS_PER_WORKER,
		"every transaction eventually committed (lock-free progress)");

	rcu_barrier();
	rcu_unregister_thread();
	return exit_status();
}
