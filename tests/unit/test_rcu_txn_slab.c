// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

/*
 * Unit test for <urcu/rcu-txn-slab.h>, the generic per-CPU size-classed
 * superblock slab that supplies MCAS transaction descriptors to both engines
 * (rcu-txn-mcas.h "mcas" instance, rcu-txn-sw.h "txn_sw" instance).  The engine
 * unit tests exercise the slab only IMPLICITLY (every commit allocates a
 * descriptor through it), so a gross defect surfaces as a crash/leak/corruption
 * there -- but the slab's own invariants are never directly asserted.  This test
 * constructs its OWN struct urcu_slab instances and checks them head-on:
 *
 *   1. enabled() tracks URCU_TXN_NO_CACHE; class_of() routing.
 *   2. alloc per class: non-NULL, 16-aligned, distinct.
 *   3. LIFO reuse: same-cpu alloc after free hands back the freed block.
 *   4. FOOTPRINT BOUND (the headline invariant): after freeing a round of N,
 *      the next round of N reuses those exact blocks -- zero new carve -- so the
 *      mapped footprint never exceeds peak-live descriptors.  Proven two ways:
 *      pointer containment (round2 subset of round1) AND st_carve unchanged.
 *   5. ORIGIN-ARENA cross-cpu free: a block allocated on cpu A and freed by a
 *      thread on cpu B returns to arena A (re-alloc on A hands back the same
 *      pointer), which is exactly the writer-allocates / reclaim-worker-frees
 *      split the slab exists for.
 *   6. Concurrent MP-push (free) / locked-pop (alloc) conservation stress:
 *      producers alloc+stamp+hand off, freers cross-thread free; every block is
 *      accounted exactly once (token-sum) with intact stamps and no crash.
 *
 * Compiled with -DURCU_TXN_CACHE_STATS so the st_reuse/st_carve counters on the
 * test's own slab instances are live and assertable (the counters are
 * unconditional struct fields; the flag only turns on the per-TU increments).
 * The slab has no teardown entry point -- its superblocks are reclaimed at
 * process exit -- so the test does not free its instances.
 */

#define _GNU_SOURCE
#define _LGPL_SOURCE
#include <assert.h>
#include <pthread.h>
#include <sched.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <urcu/compiler.h>
#include <urcu/uatomic.h>
#include <urcu/wfstack.h>
#include <urcu/rcu-txn-slab.h>

#include "tap.h"

/* Ascending byte size classes, all multiples of 16 so blocks stay 16-aligned. */
static const size_t CLASSES[] = { 32, 64, 128 };
#define NCLASS	((int) (sizeof(CLASSES) / sizeof(CLASSES[0])))

static int pin_to(int cpu)
{
	cpu_set_t set;

	CPU_ZERO(&set);
	CPU_SET(cpu, &set);
	return sched_setaffinity(0, sizeof(set), &set);
}

/* ------------------------------------------------------------------ */
/* 1. enabled() / class_of()                                          */
/* ------------------------------------------------------------------ */
static int meta_tests(struct urcu_slab *s, int want_enabled)
{
	ok(urcu_slab_enabled(s) == want_enabled,
		"enabled() == %d (URCU_TXN_NO_CACHE %s)",
		want_enabled, want_enabled ? "unset" : "set");

	ok(urcu_slab_class_of(s, 1) == 0 &&
		urcu_slab_class_of(s, 32) == 0 &&
		urcu_slab_class_of(s, 33) == 1 &&
		urcu_slab_class_of(s, 64) == 1 &&
		urcu_slab_class_of(s, 128) == 2 &&
		urcu_slab_class_of(s, 129) == -1,
		"class_of() picks smallest fitting class, -1 past the top");
	return 2;
}

/* ------------------------------------------------------------------ */
/* 2 + 3. alloc per class, alignment, distinctness, LIFO reuse         */
/* ------------------------------------------------------------------ */
static void basic_tests(struct urcu_slab *s)
{
	void *p[NCLASS];
	void *a, *b;
	int i, aligned = 1, distinct = 1;

	pin_to(0);				/* stabilise the arena the allocs land in */
	for (i = 0; i < NCLASS; i++) {
		p[i] = urcu_slab_alloc(s, i);
		if (!p[i] || ((uintptr_t) p[i] & 15))
			aligned = 0;
	}
	for (i = 0; i < NCLASS; i++) {
		int j;
		for (j = i + 1; j < NCLASS; j++)
			if (p[i] == p[j])
				distinct = 0;
	}
	ok(aligned && distinct,
		"alloc per class: non-NULL, 16-aligned, distinct addresses");
	for (i = 0; i < NCLASS; i++)
		urcu_slab_free(p[i]);

	a = urcu_slab_alloc(s, 0);
	urcu_slab_free(a);
	b = urcu_slab_alloc(s, 0);
	ok(a == b, "LIFO reuse: same-cpu alloc after free returns the freed block");
	urcu_slab_free(b);
}

/* ------------------------------------------------------------------ */
/* 4. footprint bound: a freed round is fully reused, nothing carved   */
/* ------------------------------------------------------------------ */
#define FB_N	256
static int in_set(void *const *set, int n, void *x)
{
	int i;
	for (i = 0; i < n; i++)
		if (set[i] == x)
			return 1;
	return 0;
}
static void footprint_test(void)
{
	static struct urcu_slab fs;
	void *r1[FB_N], *r2[FB_N];
	unsigned long carve1, carve2, reuse;
	int i, all_reused = 1;

	urcu_slab_init(&fs, CLASSES, NCLASS, "footprint", 8);
	pin_to(0);
	for (i = 0; i < FB_N; i++)		/* round 1: fresh -> all carve */
		r1[i] = urcu_slab_alloc(&fs, 0);
	carve1 = fs.st_carve;
	for (i = 0; i < FB_N; i++)
		urcu_slab_free(r1[i]);
	for (i = 0; i < FB_N; i++) {		/* round 2: must recycle round 1 */
		r2[i] = urcu_slab_alloc(&fs, 0);
		if (!in_set(r1, FB_N, r2[i]))
			all_reused = 0;
	}
	carve2 = fs.st_carve;
	reuse = fs.st_reuse;

	ok(all_reused && carve2 == carve1,
		"footprint bound: round-2 reuses round-1 blocks, no new carve "
		"(carve %lu==%lu)", carve1, carve2);
#ifdef URCU_TXN_CACHE_STATS
	ok(carve1 == FB_N && reuse >= FB_N,
		"stats: round 1 carved %d (=%lu), round 2 reused %lu",
		FB_N, carve1, reuse);
#else
	(void) reuse;
	skip(1, "counter assertion needs -DURCU_TXN_CACHE_STATS");
#endif
	for (i = 0; i < FB_N; i++)
		urcu_slab_free(r2[i]);
}

/* ------------------------------------------------------------------ */
/* 5. origin-arena cross-cpu free                                      */
/* ------------------------------------------------------------------ */
struct origin_arg {
	struct urcu_slab *s;
	int cpu;
	void *out;
	void *to_free;
};
static void *origin_alloc_thr(void *v)
{
	struct origin_arg *a = (struct origin_arg *) v;
	if (pin_to(a->cpu) != 0)
		a->out = NULL;
	else
		a->out = urcu_slab_alloc(a->s, 0);
	return NULL;
}
static void *origin_free_thr(void *v)
{
	struct origin_arg *a = (struct origin_arg *) v;
	if (pin_to(a->cpu) == 0)
		urcu_slab_free(a->to_free);
	return NULL;
}
static void origin_test(void)
{
	static struct urcu_slab os;
	struct origin_arg aa, fa;
	pthread_t ta, tb;
	void *b1, *b2;
	long online = sysconf(_SC_NPROCESSORS_ONLN);
	int cA = 0, cB = 1;

	if (online < 2 || pin_to(cA) != 0) {
		skip(1, "need >= 2 pinnable cpus for the origin-arena test");
		return;
	}
	urcu_slab_init(&os, CLASSES, NCLASS, "origin", 8);

	aa.s = &os; aa.cpu = cA; aa.out = NULL;
	pthread_create(&ta, NULL, origin_alloc_thr, &aa);
	pthread_join(ta, NULL);
	b1 = aa.out;

	fa.s = &os; fa.cpu = cB; fa.to_free = b1;
	pthread_create(&tb, NULL, origin_free_thr, &fa);
	pthread_join(tb, NULL);

	pin_to(cA);				/* re-alloc on the ORIGIN cpu */
	b2 = urcu_slab_alloc(&os, 0);
	ok(b1 != NULL && b1 == b2,
		"origin free: block from cpu%d freed on cpu%d returns to arena %d",
		cA, cB, cA);
	urcu_slab_free(b2);
}

/* ------------------------------------------------------------------ */
/* 6. concurrent conservation stress (cross-thread free under churn)   */
/* ------------------------------------------------------------------ */
#define NPROD	4
#define NFREE	4
#define PER	50000
#define TOTAL	(NPROD * PER)
#define TOK_K	0x9e3779b97f4a7c15UL
#define TOK_OFF	8			/* tokens live past the cds_wfs_node (8B next) */

static struct urcu_slab cs;
static struct cds_wfs_stack g_chan;	/* hand-off channel: MP push, locked pop */
static int g_tok;			/* producers fetch-add -> unique 1..TOTAL */
static int g_freed;			/* freers fetch-add -> termination */

static void *producer_thr(void *v)
{
	int cpu = (int) (intptr_t) v, i;

	pin_to(cpu);
	for (i = 0; i < PER; i++) {
		void *p = urcu_slab_alloc(&cs, 0);
		int t;

		if (!p)
			return (void *) 1;	/* OOM: fail the run */
		t = uatomic_add_return(&g_tok, 1);
		*(uint64_t *) ((char *) p + TOK_OFF) = (uint64_t) t;
		*(uint64_t *) ((char *) p + TOK_OFF + 8) = (uint64_t) t * TOK_K;
		cds_wfs_node_init((struct cds_wfs_node *) p);
		cds_wfs_push(&g_chan, (struct cds_wfs_node *) p);
	}
	return NULL;
}

struct freer_res { unsigned long sum; int cnt; int bad; };
static void *freer_thr(void *v)
{
	struct freer_res *r = (struct freer_res *) v;

	r->sum = 0; r->cnt = 0; r->bad = 0;
	while (uatomic_read(&g_freed) < TOTAL) {
		struct cds_wfs_node *node;
		void *p;
		uint64_t t, chk;

		cds_wfs_pop_lock(&g_chan);
		node = __cds_wfs_pop_blocking(&g_chan);
		cds_wfs_pop_unlock(&g_chan);
		if (!node || node == CDS_WFS_WOULDBLOCK) {
			caa_cpu_relax();
			continue;
		}
		p = (void *) node;
		t = *(uint64_t *) ((char *) p + TOK_OFF);
		chk = *(uint64_t *) ((char *) p + TOK_OFF + 8);
		if (chk != t * TOK_K)		/* stamp overwritten -> aliasing/corruption */
			r->bad++;
		r->sum += t;
		r->cnt++;
		urcu_slab_free(p);		/* cross-thread free -> origin arena */
		uatomic_add(&g_freed, 1);
	}
	return NULL;
}

static void concurrent_test(void)
{
	pthread_t prod[NPROD], freer[NFREE];
	struct freer_res res[NFREE];
	long online = sysconf(_SC_NPROCESSORS_ONLN);
	int nc = online < 1 ? 1 : (int) online;
	unsigned long got = 0, expect = (unsigned long) TOTAL * (TOTAL + 1) / 2;
	int i, cnt = 0, bad = 0;
	void *pr;
	int prod_fail = 0;

	urcu_slab_init(&cs, CLASSES, NCLASS, "concurrent", 8);
	cds_wfs_init(&g_chan);
	g_tok = 0; g_freed = 0;

	for (i = 0; i < NFREE; i++)
		pthread_create(&freer[i], NULL, freer_thr, &res[i]);
	for (i = 0; i < NPROD; i++)
		pthread_create(&prod[i], NULL, producer_thr,
				(void *) (intptr_t) (i % nc));
	for (i = 0; i < NPROD; i++) {
		pthread_join(prod[i], &pr);
		if (pr)
			prod_fail = 1;
	}
	for (i = 0; i < NFREE; i++) {
		pthread_join(freer[i], NULL);
		got += res[i].sum;
		cnt += res[i].cnt;
		bad += res[i].bad;
	}
	ok(!prod_fail && cnt == TOTAL && got == expect && bad == 0,
		"concurrent: %d blocks conserved (sum %lu==%lu), %d stamp faults",
		cnt, got, expect, bad);
	diag("[slab concurrent] reuse=%lu carve=%lu reuse%%=%.1f (peak in-flight <= %d)",
		cs.st_reuse, cs.st_carve,
		100.0 * (double) cs.st_reuse / (double) (cs.st_reuse + cs.st_carve + 1),
		TOTAL);
	ok(cs.st_carve <= TOTAL,
		"carve %lu bounded by peak in-flight %d", cs.st_carve, TOTAL);
}

/* ------------------------------------------------------------------ */
/* 6. hotplug drain: ONE cpu folds back, its neighbours are untouched  */
/* ------------------------------------------------------------------ */
/*
 * urcu_slab_drain_cpu() is the hotplug unit.  What it must guarantee is that a
 * departed cpu's rseq-only lists become reachable again -- and, just as
 * importantly, that it does NOT demote the rest of the slab: demotion is
 * one-way, so an over-broad drain would put the whole process on the atomic
 * path for one cpu's departure.  Both halves are checked here.
 *
 * The blocks are freed on cpuA, so in an rseq build they sit in cpuA's ->local,
 * reachable from cpuA alone.  After draining cpuA from ANOTHER cpu they must be
 * back on the atomic freelist and allocatable again.
 */
static void hotplug_test(void)
{
	static struct urcu_slab hs;
	long online = sysconf(_SC_NPROCESSORS_ONLN);
	int cA = 0, cB = 1, cl;
	void *p[8], *q;
	unsigned int i;
	int scoped = 1, reusable;

	if (online < 2 || pin_to(cA) != 0) {
		skip(2, "need >= 2 pinnable cpus for the hotplug-drain test");
		return;
	}
	urcu_slab_init(&hs, CLASSES, NCLASS, "hotplug", 8);

	/* populate cpuA's arenas, then hand them all back on cpuA */
	for (i = 0; i < 8; i++)
		p[i] = urcu_slab_alloc(&hs, 0);
	for (i = 0; i < 8; i++)
		urcu_slab_free(p[i]);

	/* drain cpuA from cpuB -- the cpu being drained is "gone" */
	pin_to(cB);
	urcu_slab_drain_cpu(&hs, cA);

	/* every OTHER cpu's arenas must still be armed */
	for (cl = 0; cl < NCLASS; cl++) {
		int c;

		for (c = 0; c < hs.ncpu; c++) {
			struct urcu_slab_arena *a =
				&hs.arenas[cl * hs.ncpu + c];

			if (c == cA) {
				if (a->rseq_ok)
					scoped = 0;	/* drained: must be down */
			} else if (!a->rseq_ok && urcu_slab_rseq_ready()) {
				scoped = 0;		/* collateral demote */
			}
		}
	}
	ok(scoped, "drain_cpu demotes ONLY the drained cpu's arenas "
		"(one-way, so the blast radius matters)");

	/* the folded blocks must be allocatable again from the origin cpu */
	pin_to(cA);
	q = urcu_slab_alloc(&hs, 0);
	reusable = 0;
	for (i = 0; i < 8; i++)
		if (q == p[i])
			reusable = 1;
	ok(q != NULL && reusable,
		"drain_cpu folds ->local back onto the atomic freelist "
		"(block is reachable after the cpu is drained)");
	if (q)
		urcu_slab_free(q);

	/* out-of-range and disabled slabs must be no-ops, not crashes */
	urcu_slab_drain_cpu(&hs, -1);
	urcu_slab_drain_cpu(&hs, hs.ncpu);
	urcu_slab_drain_cpu(&hs, hs.ncpu + 1000);
}

int main(void)
{
	static struct urcu_slab s;
	int want_enabled = !getenv("URCU_TXN_NO_CACHE");

	plan_no_plan();
	urcu_slab_init(&s, CLASSES, NCLASS, "meta", 8);
	meta_tests(&s, want_enabled);
	if (!want_enabled) {
		diag("URCU_TXN_NO_CACHE set: slab disabled, functional tests skipped");
		return exit_status();
	}

	basic_tests(&s);
	footprint_test();
	origin_test();
	concurrent_test();
	hotplug_test();
	return exit_status();
}
