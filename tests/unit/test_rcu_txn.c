// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

/*
 * Test for the mixed single-writer / multi-writer transaction engine and its
 * front-end <urcu/rcu-txn.h>.
 *
 * The property under test is that one commit carrying BOTH single-writer-owned
 * (store_sw) and multi-writer (store_mw) records is ATOMIC against one
 * linearization point: a concurrent reader (or a colliding writer) never sees a
 * torn subset -- some slots flipped, others not.
 *
 * As in test_rcu_mcas.c the atomicity check is a conserved SUM.  A "transfer"
 * moves a value between words so the global sum is invariantly 0; a torn commit
 * (one word updated, another not) would corrupt it permanently, caught at
 * quiescence.  The single-writer contract is respected by construction: every
 * store_sw() word is owned by exactly one thread; only the multi-writer words
 * are shared, and those go through store_mw() (CAS-old, may abort).
 *
 * Phases:
 *   - mixed:  each worker transfers between ITS OWN sw word and a SHARED mw word,
 *             so every commit mixes one SW record and one MW record.  The shared
 *             mw words force MW contention -> abort/retry; the sum invariant
 *             proves the co-committed SW edit linearizes with the MW edit and is
 *             never applied on an aborted attempt.
 *   - all-mw: transfers among shared words, store_mw-only, committed through the
 *             sw-mw-aware urcu_txn_commit() -- the pure multi-writer path.
 *   - all-sw: each worker transfers among ITS OWN words, store_sw-only, committed
 *             through the branch-lean urcu_txn_commit_sw(); it must never
 *             contention-abort.
 *
 * Single-threaded functional checks cover a basic mixed publish, the SW-only and
 * empty commits, a deterministic MW-abort that leaves the co-recorded SW word
 * untouched, and read-your-own-writes across both kinds.  QSBR flavor; every
 * worker is an RCU reader so descriptors stay alive while peers resolve proxies.
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

/* Fire single-edge MW escalation promptly so the mixed phase exercises it. */
#define URCU_TXN_ESCALATE	4

#include <urcu/rcu-txn.h>

#include "tap.h"

#define NR_TESTS	12
#ifndef NR_WORKERS
#define NR_WORKERS	8
#endif

#define TAG		URCU_TXN_TAG

#ifndef MIX_OPS
#define MIX_OPS		20000
#endif
#define MW_WORDS	8		/* shared multi-writer words */
#ifndef ALLMW_OPS
#define ALLMW_OPS	20000
#endif
#ifndef ALLSW_OPS
#define ALLSW_OPS	20000
#endif
#define SW_PER_WORKER	4		/* private single-writer words per worker */

/*
 * Slot layout mirrors test_rcu_mcas.c: [ value : top 32 bits | version : bits
 * 1..31 | tag : bit 0 ].  Every write bumps the per-word version so a word never
 * repeats a bit pattern (models RCU-managed pointers, which cannot ABA), and bit
 * 0 stays free for the engine's record tag.
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

static unsigned int xs(unsigned int x)
{
	x ^= x << 13;
	x ^= x >> 17;
	x ^= x << 5;
	return x;
}

static struct urcu_txn_domain g_domain;

/* Shared multi-writer words and per-worker private single-writer words. */
static void *g_mw[MW_WORDS];
static void *g_sw[NR_WORKERS];			/* mixed phase: one SW word per worker */
static void *g_priv[NR_WORKERS][SW_PER_WORKER];	/* all-sw phase: private word sets */

struct worker_arg {
	unsigned int seed;
	long ops;
	int tid;
	/* outputs */
	long committed;
	unsigned long aborts;
};

/* ── mixed phase: {own sw word += d, shared mw word -= d} ─────────────────── */
static void *mixed_worker(void *arg)
{
	struct worker_arg *wa = (struct worker_arg *) arg;
	struct urcu_txn txn;
	unsigned int rng = wa->seed;
	long n;

	rcu_register_thread();
	urcu_txn_init(&txn, &g_domain);
	for (n = 0; n < wa->ops; n++) {
		enum urcu_txn_status st;
		int j;
		intptr_t d;

		rng = xs(rng);
		j = (int) (rng % MW_WORDS);
		d = (intptr_t) (1 + (rng >> 8) % 3);
		for (;;) {
			uintptr_t osw, omw;

			urcu_txn_begin(&txn);
			osw = (uintptr_t) urcu_txn_load(&txn, &g_sw[wa->tid], TAG);
			omw = (uintptr_t) urcu_txn_load(&txn, &g_mw[j], TAG);
			urcu_txn_store_sw(&txn, &g_sw[wa->tid],
				(void *) osw, (void *) lf_bump(osw, d), TAG);
			urcu_txn_store_mw(&txn, &g_mw[j],
				(void *) omw, (void *) lf_bump(omw, -d), TAG);
			st = urcu_txn_commit(&txn);
			urcu_txn_end(&txn);
			if (st == URCU_TXN_STATUS_OK)
				break;
			if (st == URCU_TXN_STATUS_MEMORY_ERROR)
				abort();
			wa->aborts++;		/* contention: retry */
		}
		wa->committed++;
		rcu_quiescent_state();
	}
	rcu_unregister_thread();
	return NULL;
}

/* ── all-mw phase: {mw[i] += d, mw[j] -= d}, store_mw only ────────────────── */
static void *allmw_worker(void *arg)
{
	struct worker_arg *wa = (struct worker_arg *) arg;
	struct urcu_txn txn;
	unsigned int rng = wa->seed;
	long n;

	rcu_register_thread();
	urcu_txn_init(&txn, &g_domain);
	for (n = 0; n < wa->ops; n++) {
		enum urcu_txn_status st;
		int i, j;
		intptr_t d;

		rng = xs(rng);
		i = (int) (rng % MW_WORDS);
		rng = xs(rng);
		j = (int) (rng % MW_WORDS);
		if (j == i)
			j = (j + 1) % MW_WORDS;
		d = (intptr_t) (1 + (rng >> 8) % 3);
		for (;;) {
			uintptr_t oi, oj;

			urcu_txn_begin(&txn);
			oi = (uintptr_t) urcu_txn_load(&txn, &g_mw[i], TAG);
			oj = (uintptr_t) urcu_txn_load(&txn, &g_mw[j], TAG);
			urcu_txn_store_mw(&txn, &g_mw[i],
				(void *) oi, (void *) lf_bump(oi, d), TAG);
			urcu_txn_store_mw(&txn, &g_mw[j],
				(void *) oj, (void *) lf_bump(oj, -d), TAG);
			st = urcu_txn_commit(&txn);
			urcu_txn_end(&txn);
			if (st == URCU_TXN_STATUS_OK)
				break;
			if (st == URCU_TXN_STATUS_MEMORY_ERROR)
				abort();
			wa->aborts++;
		}
		wa->committed++;
		rcu_quiescent_state();
	}
	rcu_unregister_thread();
	return NULL;
}

/* ── all-sw phase: {priv[a] += d, priv[b] -= d}, store_sw only, commit_sw ─── */
static void *allsw_worker(void *arg)
{
	struct worker_arg *wa = (struct worker_arg *) arg;
	struct urcu_txn txn;
	unsigned int rng = wa->seed;
	long n;

	rcu_register_thread();
	urcu_txn_init(&txn, &g_domain);
	for (n = 0; n < wa->ops; n++) {
		enum urcu_txn_status st;
		int a, b;
		intptr_t d;
		uintptr_t oa, ob;

		rng = xs(rng);
		a = (int) (rng % SW_PER_WORKER);
		rng = xs(rng);
		b = (int) (rng % SW_PER_WORKER);
		if (b == a)
			b = (b + 1) % SW_PER_WORKER;
		d = (intptr_t) (1 + (rng >> 8) % 3);

		urcu_txn_begin(&txn);
		oa = (uintptr_t) urcu_txn_load(&txn, &g_priv[wa->tid][a], TAG);
		ob = (uintptr_t) urcu_txn_load(&txn, &g_priv[wa->tid][b], TAG);
		urcu_txn_store_sw(&txn, &g_priv[wa->tid][a],
			(void *) oa, (void *) lf_bump(oa, d), TAG);
		urcu_txn_store_sw(&txn, &g_priv[wa->tid][b],
			(void *) ob, (void *) lf_bump(ob, -d), TAG);
		st = urcu_txn_commit_sw(&txn);
		urcu_txn_end(&txn);
		if (st == URCU_TXN_STATUS_OK) {
			wa->committed++;
		} else {
			/* commit_sw must never contention-abort (distinct slots). */
			wa->aborts++;
			urcu_txn_abandon(&txn);
		}
		rcu_quiescent_state();
	}
	rcu_unregister_thread();
	return NULL;
}

static void spawn_join(void *(*fn)(void *), long ops,
		long *out_committed, unsigned long *out_aborts)
{
	pthread_t th[NR_WORKERS];
	struct worker_arg args[NR_WORKERS];
	long committed = 0;
	unsigned long aborts = 0;
	int i;

	for (i = 0; i < NR_WORKERS; i++) {
		args[i].seed = 0x9e3779b9u + (unsigned int) i * 2654435761u;
		args[i].ops = ops;
		args[i].tid = i;
		args[i].committed = 0;
		args[i].aborts = 0;
		pthread_create(&th[i], NULL, fn, &args[i]);
	}
	rcu_thread_offline();		/* don't stall grace periods while joined */
	for (i = 0; i < NR_WORKERS; i++) {
		pthread_join(th[i], NULL);
		committed += args[i].committed;
		aborts += args[i].aborts;
	}
	rcu_thread_online();
	*out_committed = committed;
	*out_aborts = aborts;
}

static intptr_t sum_mw(void)
{
	intptr_t s = 0;
	int i;

	for (i = 0; i < MW_WORDS; i++)
		s += lf_val((uintptr_t) g_mw[i]);
	return s;
}

static intptr_t sum_sw(void)
{
	intptr_t s = 0;
	int i;

	for (i = 0; i < NR_WORKERS; i++)
		s += lf_val((uintptr_t) g_sw[i]);
	return s;
}

static intptr_t sum_priv(void)
{
	intptr_t s = 0;
	int i, j;

	for (i = 0; i < NR_WORKERS; i++)
		for (j = 0; j < SW_PER_WORKER; j++)
			s += lf_val((uintptr_t) g_priv[i][j]);
	return s;
}

/* ── single-threaded functional checks ───────────────────────────────────── */
static void functional_checks(void)
{
	struct urcu_txn txn;
	void *sa, *sb, *ma, *mb;
	enum urcu_txn_status st;

	/* Basic mixed publish: 2 SW + 2 MW words flip old -> new atomically. */
	sa = (void *) 0x100; sb = (void *) 0x200;
	ma = (void *) 0x300; mb = (void *) 0x400;
	urcu_txn_init(&txn, NULL);
	urcu_txn_begin(&txn);
	urcu_txn_store_sw(&txn, &sa, (void *) 0x100, (void *) 0x110, TAG);
	urcu_txn_store_sw(&txn, &sb, (void *) 0x200, (void *) 0x210, TAG);
	urcu_txn_store_mw(&txn, &ma, (void *) 0x300, (void *) 0x310, TAG);
	urcu_txn_store_mw(&txn, &mb, (void *) 0x400, (void *) 0x410, TAG);
	st = urcu_txn_commit(&txn);
	urcu_txn_end(&txn);
	ok(st == URCU_TXN_STATUS_OK && sa == (void *) 0x110 &&
		sb == (void *) 0x210 && ma == (void *) 0x310 && mb == (void *) 0x410,
		"mixed commit publishes all SW and MW slots atomically");

	/* SW-only commit through the branch-lean path. */
	sa = (void *) 0x10; sb = (void *) 0x20;
	urcu_txn_begin(&txn);
	urcu_txn_store_sw(&txn, &sa, (void *) 0x10, (void *) 0x1a, TAG);
	urcu_txn_store_sw(&txn, &sb, (void *) 0x20, (void *) 0x2a, TAG);
	st = urcu_txn_commit_sw(&txn);
	urcu_txn_end(&txn);
	ok(st == URCU_TXN_STATUS_OK && sa == (void *) 0x1a && sb == (void *) 0x2a,
		"SW-only commit_sw publishes without the MW machinery");

	/* Empty transaction commits trivially. */
	urcu_txn_begin(&txn);
	st = urcu_txn_commit(&txn);
	urcu_txn_end(&txn);
	ok(st == URCU_TXN_STATUS_OK, "empty commit returns OK");

	/*
	 * Deterministic MW abort: the MW record names a wrong old, so its install
	 * CAS mismatches and the whole txn aborts.  MW installs before SW, so the
	 * co-recorded SW word is never parked -- it must be left untouched.
	 */
	sa = (void *) 0xA0;			/* SW word */
	ma = (void *) 0xB0;			/* MW word actually holds 0xB0 */
	urcu_txn_begin(&txn);
	urcu_txn_store_sw(&txn, &sa, (void *) 0xA0, (void *) 0xA2, TAG);
	urcu_txn_store_mw(&txn, &ma, (void *) 0xBAD0, (void *) 0xB2, TAG);
	st = urcu_txn_commit(&txn);
	urcu_txn_end(&txn);
	ok(st == URCU_TXN_STATUS_ABORT && sa == (void *) 0xA0 && ma == (void *) 0xB0,
		"MW CAS mismatch aborts and leaves the co-recorded SW word untouched");

	/* Retry with the correct old commits and publishes both. */
	urcu_txn_begin(&txn);
	urcu_txn_store_sw(&txn, &sa, (void *) 0xA0, (void *) 0xA2, TAG);
	urcu_txn_store_mw(&txn, &ma, (void *) 0xB0, (void *) 0xB2, TAG);
	st = urcu_txn_commit(&txn);
	urcu_txn_end(&txn);
	ok(st == URCU_TXN_STATUS_OK && sa == (void *) 0xA2 && ma == (void *) 0xB2,
		"retry with the correct old commits the mixed edit");

	/* Read-your-own-writes across both kinds within one attempt. */
	sa = (void *) 0x1000; ma = (void *) 0x2000;
	urcu_txn_begin(&txn);
	urcu_txn_store_sw(&txn, &sa, (void *) 0x1000, (void *) 0x1002, TAG);
	urcu_txn_store_mw(&txn, &ma, (void *) 0x2000, (void *) 0x2002, TAG);
	{
		void *rsw = urcu_txn_load(&txn, &sa, TAG);
		void *rmw = urcu_txn_load(&txn, &ma, TAG);

		ok(rsw == (void *) 0x1002 && rmw == (void *) 0x2002,
			"read-your-own-writes returns the pending value for SW and MW");
	}
	(void) urcu_txn_commit(&txn);
	urcu_txn_end(&txn);
}

int main(void)
{
	long committed;
	unsigned long aborts;
	int i, j;

	plan_tests(NR_TESTS);
	rcu_register_thread();
	urcu_txn_domain_init(&g_domain);

	functional_checks();		/* 6 assertions */

	/* --- mixed phase: SW-owned + shared MW, one linearization point. --- */
	for (i = 0; i < NR_WORKERS; i++)
		g_sw[i] = (void *) 0;
	for (i = 0; i < MW_WORDS; i++)
		g_mw[i] = (void *) 0;
	spawn_join(mixed_worker, MIX_OPS, &committed, &aborts);
	diag("mixed: %d workers x %d ops; committed=%ld, aborts=%lu; sum(sw)+sum(mw)=%"
		PRIdPTR, NR_WORKERS, MIX_OPS, committed, aborts, sum_sw() + sum_mw());
	ok(sum_sw() + sum_mw() == 0,
		"mixed: SW+MW commit stayed atomic across contention (sum invariant)");
	ok(committed == (long) NR_WORKERS * MIX_OPS,
		"mixed: every mixed transaction eventually committed (progress)");

	/* --- all-mw phase: the pure multi-writer path. --- */
	for (i = 0; i < MW_WORDS; i++)
		g_mw[i] = (void *) 0;
	spawn_join(allmw_worker, ALLMW_OPS, &committed, &aborts);
	diag("all-mw: committed=%ld, aborts=%lu; sum(mw)=%" PRIdPTR,
		committed, aborts, sum_mw());
	ok(sum_mw() == 0,
		"all-mw: multi-writer-only commit stayed atomic (sum invariant)");
	ok(committed == (long) NR_WORKERS * ALLMW_OPS,
		"all-mw: every multi-writer transaction committed (progress)");

	/* --- all-sw phase: the branch-lean single-writer path. --- */
	for (i = 0; i < NR_WORKERS; i++)
		for (j = 0; j < SW_PER_WORKER; j++)
			g_priv[i][j] = (void *) 0;
	spawn_join(allsw_worker, ALLSW_OPS, &committed, &aborts);
	diag("all-sw: committed=%ld, aborts=%lu; sum(priv)=%" PRIdPTR,
		committed, aborts, sum_priv());
	ok(sum_priv() == 0,
		"all-sw: single-writer-only commit_sw stayed atomic (sum invariant)");
	ok(aborts == 0 && committed == (long) NR_WORKERS * ALLSW_OPS,
		"all-sw: commit_sw never contention-aborted (single-writer, abort-free)");

	rcu_barrier();			/* drain deferred txn reclaim callbacks */
	rcu_unregister_thread();
	return exit_status();
}
