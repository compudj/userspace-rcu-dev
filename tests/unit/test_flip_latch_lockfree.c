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
 * Fairness (the priority + steal engine): a second, deliberately hot phase
 * (few words, many writers) measures the worst single-operation bypass, i.e.
 * the highest retry count any one operation reached before committing.  With the
 * aging-priority contention manager this stays bounded (a starved op's priority
 * climbs until it can no longer be bypassed); without it a writer could be
 * starved unboundedly.  The phase also reports helping work -- drive() passes
 * beyond each op's own owner-drive -- which is the evidence for or against a
 * per-slot waiter queue (combining) as a follow-up.
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
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <urcu/compiler.h>
#include <urcu-qsbr.h>
#include <urcu-call-rcu.h>

/*
 * Engine instrumentation hook: count helping (drive), eviction and steal events
 * per thread.  Must be defined before the header so its inline functions pick it
 * up; compiles to nothing in any other translation unit.
 */
struct lf_stat {
	unsigned long drive;	/* drive() entries: owner-drives + helping */
	unsigned long evict;	/* foreign txns aborted by priority */
	unsigned long steal;	/* slots stolen proxy->proxy */
	unsigned long escalate;	/* single-edge ops promoted to the descriptor path */
};
static __thread struct lf_stat t_stat;
#define URCU_FLIP_LF_STAT(counter)	(t_stat.counter++)

/*
 * Force single-edge escalation to fire promptly so the mixed phase reliably
 * exercises that path (the shipped default is higher); the mechanism is what's
 * under test, not the threshold value.
 */
#define URCU_FLIP_LF_ESCALATE	4

#include <urcu/flip-latch-lockfree.h>

#include "tap.h"

#define NR_TESTS	9
#define NR_WORKERS	8

#define MILD_WORDS	16		/* moderate contention: atomicity focus */
#define MILD_OPS	60000
#define HOT_WORDS	4		/* heavy contention: fairness focus */
#define HOT_OPS		20000

/*
 * Mixed phase: a few workers run single-edge +1 ops on a tiny hot set while the
 * rest keep it proxied with multi-edge transfers.  Tests that a single-edge op
 * is not starved by the lingering proxies -- it escalates into a real, visible
 * transaction once it has retried enough.  Multi-edge transfers are net-zero, so
 * the final sum equals the number of committed single-edge increments.
 */
#define MIX_WORDS	3
#define MIX_OPS		20000
#define MIX_SINGLE	2		/* single-edge workers; the rest are multi-edge */

/*
 * Worst tolerated single-op bypass in the hot phase.  Generous: under the aging
 * priority a starved op is bypassed at most O(concurrency) before its retry
 * count dominates; a regression (raw-race arbitration / broken aging) blows past
 * this.  Calibrated from observed runs (typ. low tens) with wide headroom.
 */
#define HOT_RETRY_BOUND	512

static void *g_word[MILD_WORDS];	/* all start at (void *) 0 */

struct worker_arg {
	unsigned int seed;
	unsigned int nwords;
	long ops;
	int single;			/* 1: single-edge +1 ops; 0: multi-edge transfers */
	/* outputs */
	long committed;
	unsigned long retries_sum;	/* total failed attempts (sum of per-op retry) */
	unsigned long max_op_retry;	/* worst single-op bypass */
	struct lf_stat st;
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
	unsigned int nwords = wa->nwords;
	long n;

	rcu_register_thread();
	for (n = 0; n < wa->ops; n++) {
		unsigned long retry = 0;
		bool ok;

		if (wa->single) {
			/*
			 * Single-edge +1 on one hot word.  Its bare CAS fails
			 * whenever a multi-edge op keeps the word proxied; once it
			 * has retried past the escalation threshold the engine
			 * promotes it to a real descriptor so it cannot starve.
			 */
			int w;

			rng = xs(rng);
			w = (int) (rng % nwords);
			do {
				struct urcu_flip_lf_mcas *t;
				uintptr_t ow;

				rcu_read_lock();
				t = urcu_flip_lf_mcas_create(1, retry);
				if (!t)
					abort();
				ow = (uintptr_t) urcu_flip_lf_read(&g_word[w]);
				urcu_flip_lf_mcas_add(t, &g_word[w],
					(void *) ow, (void *) lf_bump(ow, 1));
				ok = urcu_flip_lf_mcas_commit(t, call_rcu);
				rcu_read_unlock();
				if (!ok)
					retry++;
			} while (!ok);
		} else {
			int i, j, k, three;

			rng = xs(rng);
			three = (nwords >= 3) ? (int) (rng & 1) : 0;
			i = (int) ((rng >> 1) % nwords);
			rng = xs(rng);
			j = (int) (rng % nwords);
			if (j == i)
				j = (j + 1) % (int) nwords;
			rng = xs(rng);
			k = (int) (rng % nwords);
			while (k == i || k == j)
				k = (k + 1) % (int) nwords;

			do {
				struct urcu_flip_lf_mcas *t;
				uintptr_t oi, oj, ok2;

				/*
				 * The updater is an RCU reader: this critical
				 * section keeps a descriptor alive while peers
				 * help drive it.  A no-op in QSBR, required for
				 * the memb/mb flavors.
				 */
				rcu_read_lock();
				t = urcu_flip_lf_mcas_create(3, retry);
				if (!t)
					abort();
				oi = (uintptr_t) urcu_flip_lf_read(&g_word[i]);
				oj = (uintptr_t) urcu_flip_lf_read(&g_word[j]);
				urcu_flip_lf_mcas_add(t, &g_word[i],
					(void *) oi, (void *) lf_bump(oi, 2));
				if (three) {
					ok2 = (uintptr_t) urcu_flip_lf_read(&g_word[k]);
					urcu_flip_lf_mcas_add(t, &g_word[j],
						(void *) oj, (void *) lf_bump(oj, 2));
					urcu_flip_lf_mcas_add(t, &g_word[k],
						(void *) ok2, (void *) lf_bump(ok2, -4));
				} else {
					urcu_flip_lf_mcas_add(t, &g_word[j],
						(void *) oj, (void *) lf_bump(oj, -2));
				}
				ok = urcu_flip_lf_mcas_commit(t, call_rcu);
				rcu_read_unlock();
				if (!ok)
					retry++;
			} while (!ok);
		}

		wa->committed++;
		wa->retries_sum += retry;
		if (retry > wa->max_op_retry)
			wa->max_op_retry = retry;
		rcu_quiescent_state();
	}
	rcu_unregister_thread();
	wa->st = t_stat;
	return NULL;
}

/*
 * Run one phase: @nwords shared words, @ops per worker.  Returns the final sum
 * (0 iff atomic); fills *@out_committed, *@out_max_retry and the aggregate
 * drive/evict/steal/attempt counters.
 */
static intptr_t run_phase(unsigned int nwords, long ops,
		long *out_committed, unsigned long *out_max_retry,
		struct lf_stat *out_st, unsigned long *out_attempts)
{
	pthread_t th[NR_WORKERS];
	struct worker_arg args[NR_WORKERS];
	long committed = 0;
	unsigned long max_retry = 0, attempts = 0;
	struct lf_stat st = { 0, 0, 0, 0 };
	intptr_t sum = 0;
	unsigned int i;

	for (i = 0; i < nwords; i++)
		g_word[i] = (void *) 0;

	for (i = 0; i < NR_WORKERS; i++) {
		args[i].seed = 0x9e3779b9u + i * 2654435761u;
		args[i].nwords = nwords;
		args[i].ops = ops;
		args[i].single = 0;
		args[i].committed = 0;
		args[i].retries_sum = 0;
		args[i].max_op_retry = 0;
		args[i].st = st;	/* zero */
		pthread_create(&th[i], NULL, worker, &args[i]);
	}
	rcu_thread_offline();		/* don't stall grace periods while joined */
	for (i = 0; i < NR_WORKERS; i++) {
		pthread_join(th[i], NULL);
		committed += args[i].committed;
		attempts += (unsigned long) args[i].committed + args[i].retries_sum;
		if (args[i].max_op_retry > max_retry)
			max_retry = args[i].max_op_retry;
		st.drive += args[i].st.drive;
		st.evict += args[i].st.evict;
		st.steal += args[i].st.steal;
	}
	rcu_thread_online();

	for (i = 0; i < nwords; i++)
		sum += lf_val((uintptr_t) g_word[i]);

	*out_committed = committed;
	*out_max_retry = max_retry;
	*out_st = st;
	*out_attempts = attempts;
	return sum;
}

/*
 * Mixed phase: @nsingle single-edge workers + (NR_WORKERS - @nsingle) multi-edge
 * workers on @nwords hot words.  Returns the final sum (== committed single-edge
 * increments iff no update was lost); fills the single-edge committed count and
 * worst bypass, the total committed across all workers, and the escalation count.
 */
static intptr_t run_mixed_phase(unsigned int nwords, long ops, unsigned int nsingle,
		long *out_single_committed, unsigned long *out_single_max_retry,
		long *out_total_committed, unsigned long *out_escalate)
{
	pthread_t th[NR_WORKERS];
	struct worker_arg args[NR_WORKERS];
	long single_committed = 0, total_committed = 0;
	unsigned long single_max_retry = 0, escalate = 0;
	struct lf_stat zero = { 0, 0, 0, 0 };
	intptr_t sum = 0;
	unsigned int i;

	for (i = 0; i < nwords; i++)
		g_word[i] = (void *) 0;

	for (i = 0; i < NR_WORKERS; i++) {
		args[i].seed = 0x9e3779b9u + i * 2654435761u;
		args[i].nwords = nwords;
		args[i].ops = ops;
		args[i].single = (i < nsingle);
		args[i].committed = 0;
		args[i].retries_sum = 0;
		args[i].max_op_retry = 0;
		args[i].st = zero;
		pthread_create(&th[i], NULL, worker, &args[i]);
	}
	rcu_thread_offline();
	for (i = 0; i < NR_WORKERS; i++) {
		pthread_join(th[i], NULL);
		total_committed += args[i].committed;
		escalate += args[i].st.escalate;
		if (args[i].single) {
			single_committed += args[i].committed;
			if (args[i].max_op_retry > single_max_retry)
				single_max_retry = args[i].max_op_retry;
		}
	}
	rcu_thread_online();

	for (i = 0; i < nwords; i++)
		sum += lf_val((uintptr_t) g_word[i]);

	*out_single_committed = single_committed;
	*out_single_max_retry = single_max_retry;
	*out_total_committed = total_committed;
	*out_escalate = escalate;
	return sum;
}

int main(void)
{
	long committed;
	unsigned long max_retry, attempts;
	struct lf_stat st;
	intptr_t sum;

	plan_tests(NR_TESTS);
	rcu_register_thread();

	/* --- Phase 1: moderate contention -- atomicity + progress. --- */
	sum = run_phase(MILD_WORDS, MILD_OPS, &committed, &max_retry, &st, &attempts);
	diag("mild: %d workers x %d ops over %d words = %ld committed; sum = %"
		PRIdPTR "; max single-op retry = %lu",
		NR_WORKERS, MILD_OPS, MILD_WORDS, committed, sum, max_retry);
	ok(sum == 0,
		"mild: k-CAS stayed atomic across concurrent transfers (sum invariant)");
	ok(committed == (long) NR_WORKERS * MILD_OPS,
		"mild: every transaction eventually committed (lock-free progress)");

	/* --- Phase 2: heavy contention -- fairness + helping cost. --- */
	sum = run_phase(HOT_WORDS, HOT_OPS, &committed, &max_retry, &st, &attempts);
	{
		/*
		 * owner-drives = one drive() per attempt (every commit of a
		 * >=2-edge txn drives once); the rest is helping foreign txns.
		 */
		double dpc = (double) st.drive / (double) committed;
		double helppc = (double) (st.drive - attempts) / (double) committed;

		diag("hot:  %d workers x %d ops over %d words = %ld committed; sum = %"
			PRIdPTR, NR_WORKERS, HOT_OPS, HOT_WORDS, committed, sum);
		diag("hot:  max single-op retry = %lu (bound %d); attempts = %lu",
			max_retry, HOT_RETRY_BOUND, attempts);
		diag("hot:  drive=%lu (%.2f/commit) help=%.2f/commit evict=%lu steal=%lu",
			st.drive, dpc, helppc, st.evict, st.steal);
	}
	ok(sum == 0,
		"hot: k-CAS stayed atomic under heavy contention (sum invariant)");
	ok(committed == (long) NR_WORKERS * HOT_OPS,
		"hot: every transaction eventually committed (lock-free progress)");
	ok(max_retry < HOT_RETRY_BOUND,
		"hot: worst single-op bypass stayed bounded (priority fairness)");

	/* --- Phase 3: single-edge ops vs. proxy-holding multi-edge ops. --- */
	{
		long single_committed, total_committed;
		unsigned long single_max_retry, escalate;

		sum = run_mixed_phase(MIX_WORDS, MIX_OPS, MIX_SINGLE,
			&single_committed, &single_max_retry,
			&total_committed, &escalate);
		diag("mix:  %d single-edge + %d multi-edge workers x %d ops over %d words",
			MIX_SINGLE, NR_WORKERS - MIX_SINGLE, MIX_OPS, MIX_WORDS);
		diag("mix:  single committed = %ld; sum = %" PRIdPTR
			"; single max retry = %lu (bound %d); escalations = %lu",
			single_committed, sum, single_max_retry, HOT_RETRY_BOUND, escalate);
		ok(sum == (intptr_t) single_committed,
			"mix: single-edge increments not lost amid lingering proxies");
		ok(total_committed == (long) NR_WORKERS * MIX_OPS,
			"mix: every worker completed -- single-edge ops not starved");
		ok(single_max_retry < HOT_RETRY_BOUND,
			"mix: single-edge worst bypass stayed bounded (escalation fairness)");
		ok(escalate > 0,
			"mix: single-edge ops actually escalated (mechanism exercised)");
	}

	rcu_barrier();
	rcu_unregister_thread();
	return exit_status();
}
