// SPDX-FileCopyrightText: 2026 EfficiOS Inc.
//
// SPDX-License-Identifier: LGPL-2.1-or-later

/*
 * test_urcu_ft_range.c
 *
 * Regression tests for the cds_ft_range layer.
 *
 *   1. Stab self-test: insert N ranges, query at each range's
 *      midpoint, verify it's found.
 *   2. Brute-force overlap parity: random ranges + random windows,
 *      compared against a linear scan.
 *   3. Granularity culling: mix of L sizes, verify no L < g returned
 *      and all L >= g qualifying ranges are returned.
 *   4. Pan locality sanity: same window pre- and post-pan compared
 *      against brute force.
 *   5. Concurrent reader / writer (RCU): one writer mutates under
 *      mutex, multiple readers run overlap queries; assert no
 *      crashes and consistent snapshots.
 *   6. Edge ranges: T=0, T near UINT64_MAX, L=1, L power-of-2.
 *
 * Built with RCU_QSBR (see Makefile.am). Uses libtap.
 */

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <urcu/compiler.h>
#include <urcu-qsbr.h>		/* must precede fractal-trie headers */
#include <urcu/fractal-trie.h>
#include <urcu/fractal-trie-range.h>
#include <urcu-call-rcu.h>

#include <assert.h>
#include <inttypes.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "tap.h"

#define NR_TESTS 14

/* ------------------------------------------------------------------ */
/* Group helper                                                       */
/* ------------------------------------------------------------------ */

static struct cds_ft_group *make_group(void)
{
	struct cds_ft_group_attr *gattr;
	struct cds_ft_group *group;

	if (cds_ft_group_attr_create(&gattr) < 0)
		abort();
	if (cds_ft_group_attr_set_key_len(gattr, 8) < 0)
		abort();
	if (cds_ft_group_create(gattr, &group) < 0)
		abort();
	cds_ft_group_attr_destroy(gattr);
	return group;
}

/* ------------------------------------------------------------------ */
/* Test-record infrastructure                                         */
/* ------------------------------------------------------------------ */

struct rec {
	struct cds_ft_range_node node;
	struct rcu_head rcu;
	uint64_t start;
	uint64_t end;
	uint64_t id;	/* shadow for identity / debugging */
};

static atomic_ulong g_alloc, g_free;

static struct rec *rec_alloc(uint64_t start, uint64_t end, uint64_t id)
{
	struct rec *r = (struct rec *) calloc(1, sizeof(*r));
	if (!r) {
		fprintf(stderr, "rec_alloc OOM\n");
		abort();
	}
	cds_ft_range_node_init(&r->node);
	r->start = start;
	r->end = end;
	r->id = id;
	atomic_fetch_add(&g_alloc, 1);
	return r;
}

static void rec_free(struct rec *r)
{
	memset(r, 0xfe, sizeof(*r));
	free(r);
	atomic_fetch_add(&g_free, 1);
}

static void rec_free_rcu_cb(struct rcu_head *h)
{
	rec_free(caa_container_of(h, struct rec, rcu));
}

static void leak_reset(void)
{
	atomic_store(&g_alloc, 0);
	atomic_store(&g_free, 0);
}

static int leak_check(void)
{
	rcu_barrier();
	unsigned long a = atomic_load(&g_alloc);
	unsigned long f = atomic_load(&g_free);
	if (a != f) {
		fprintf(stderr, "LEAK: alloc=%lu free=%lu\n", a, f);
		return -1;
	}
	return 0;
}

/* Drain all inserted records by iterating the entire key space at every level. */
static void drain_all(struct cds_ft_range *ftr)
{
	struct cds_ft_range_iter *it;
	struct cds_ft_range_node *n;
	struct rec *r;
	struct cds_ft_range_node **batch = NULL;
	uint64_t *starts = NULL;
	size_t batch_cap = 0, batch_n = 0;

	if (cds_ft_range_iter_create(ftr, &it) < 0)
		abort();

	rcu_read_lock();
	cds_ft_range_lookup_overlap(it, 0, UINT64_MAX, 0);
	while ((n = cds_ft_range_iter_node(it)) != NULL) {
		if (batch_n == batch_cap) {
			batch_cap = batch_cap ? batch_cap * 2 : 64;
			batch = (struct cds_ft_range_node **) realloc(batch,
				batch_cap * sizeof(*batch));
			starts = (uint64_t *) realloc(starts,
				batch_cap * sizeof(*starts));
			if (!batch || !starts)
				abort();
		}
		r = caa_container_of(n, struct rec, node);
		batch[batch_n] = n;
		starts[batch_n] = r->start;
		batch_n++;
		cds_ft_range_iter_next(it);
	}
	rcu_read_unlock();
	cds_ft_range_iter_destroy(it);

	for (size_t i = 0; i < batch_n; i++) {
		struct rec *rr = caa_container_of(batch[i], struct rec, node);
		(void) cds_ft_range_remove(ftr, starts[i], batch[i]);
		call_rcu(&rr->rcu, rec_free_rcu_cb);
	}
	free(batch);
	free(starts);
}

/* ------------------------------------------------------------------ */
/* PRNG (deterministic, single-threaded helpers)                      */
/* ------------------------------------------------------------------ */

static uint64_t prng_state;

static uint64_t prng(void)
{
	/* xorshift64* */
	uint64_t x = prng_state;
	x ^= x >> 12;
	x ^= x << 25;
	x ^= x >> 27;
	prng_state = x;
	return x * 0x2545F4914F6CDD1DULL;
}

static void prng_seed(uint64_t s)
{
	prng_state = s ? s : 0xdeadbeefULL;
}

/* ------------------------------------------------------------------ */
/* Brute-force shadow                                                 */
/* ------------------------------------------------------------------ */

struct shadow {
	uint64_t start, end, id;
};

/* Linear-scan brute force: returns how many shadow records overlap [q_a, q_b)
 * with length >= g. Sorts the matched IDs into out_ids (caller-provided). */
static size_t shadow_overlap(const struct shadow *s, size_t n,
		uint64_t q_a, uint64_t q_b, uint64_t g,
		uint64_t *out_ids)
{
	size_t k = 0;
	for (size_t i = 0; i < n; i++) {
		uint64_t L = s[i].end - s[i].start;
		if (L < g)
			continue;
		if (s[i].start < q_b && s[i].end > q_a)
			out_ids[k++] = s[i].id;
	}
	return k;
}

/* ------------------------------------------------------------------ */
/* Test 1: routing function unit checks (no concurrency, no trie)     */
/* ------------------------------------------------------------------ */

static int test_routing(void)
{
	struct {
		uint64_t s, e;
		unsigned int expect;
	} cases[] = {
		{ 0, 1, 0 },			/* L=1 */
		{ 0, 2, 1 },			/* L=2 */
		{ 0, 3, 2 },			/* L=3, ceil(log2)=2 */
		{ 0, 4, 2 },			/* L=4, ceil(log2)=2 */
		{ 0, 5, 3 },			/* L=5 */
		{ 0, 8, 3 },			/* L=8 */
		{ 0, 9, 4 },			/* L=9 */
		{ 0, 16, 4 },
		{ 100, 200, 7 },		/* L=100, 2^7=128 */
		{ 0, (1ULL << 32), 32 },	/* L = 2^32 */
		{ UINT64_MAX - 1, UINT64_MAX, 0 },	/* L=1 at top */
	};

	for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
		unsigned int got = cds_ft_range_route_level(cases[i].s, cases[i].e);
		if (got != cases[i].expect) {
			fprintf(stderr,
				"routing fail: [%" PRIu64 ", %" PRIu64 ") expected k=%u got k=%u\n",
				cases[i].s, cases[i].e, cases[i].expect, got);
			return -1;
		}
	}
	return 0;
}

/* ------------------------------------------------------------------ */
/* Test 2: stab self-test                                             */
/* ------------------------------------------------------------------ */

static int test_stab_self(void)
{
	struct cds_ft_range *ftr;
	enum cds_ft_status s;
	const size_t N = 2000;
	struct shadow *shadow = NULL;
	int ret = -1;

	struct cds_ft_group *group = make_group();
	if (cds_ft_range_create(group, NULL, &ftr) < 0) {
		cds_ft_group_destroy(group);
		return -1;
	}

	shadow = (struct shadow *) calloc(N, sizeof(*shadow));
	if (!shadow)
		goto out;

	prng_seed(0xa11ce);
	for (size_t i = 0; i < N; i++) {
		uint64_t a = prng() & ((1ULL << 50) - 1);
		uint64_t L = 1 + (prng() & ((1ULL << 20) - 1));
		uint64_t b;
		struct rec *r;
		if (a > UINT64_MAX - L)
			a = UINT64_MAX - L;
		b = a + L;
		shadow[i].start = a;
		shadow[i].end = b;
		shadow[i].id = i;
		r = rec_alloc(a, b, i);
		s = cds_ft_range_insert(ftr, a, b, &r->node);
		if (s < 0) {
			fprintf(stderr, "insert fail i=%zu\n", i);
			goto out;
		}
	}

	for (size_t i = 0; i < N; i++) {
		uint64_t mid = shadow[i].start + (shadow[i].end - shadow[i].start) / 2;
		struct cds_ft_range_iter *it;
		struct cds_ft_range_node *n;
		bool found = false;

		if (cds_ft_range_iter_create(ftr, &it) < 0)
			goto out;
		rcu_read_lock();
		cds_ft_range_lookup_overlap(it, mid, mid + 1, 0);
		while ((n = cds_ft_range_iter_node(it)) != NULL) {
			struct rec *r = caa_container_of(n, struct rec, node);
			if (r->id == shadow[i].id) {
				found = true;
				break;
			}
			cds_ft_range_iter_next(it);
		}
		rcu_read_unlock();
		cds_ft_range_iter_destroy(it);
		if (!found) {
			fprintf(stderr, "stab miss for id %zu [%" PRIu64 ", %" PRIu64 ")\n",
				i, shadow[i].start, shadow[i].end);
			goto out;
		}
	}

	ret = 0;
out:
	free(shadow);
	drain_all(ftr);
	cds_ft_range_destroy(ftr);
	cds_ft_group_destroy(group);
	return ret;
}

/* ------------------------------------------------------------------ */
/* Test 3: brute-force overlap parity                                 */
/* ------------------------------------------------------------------ */

static int collect_overlap(struct cds_ft_range *ftr,
		uint64_t q_a, uint64_t q_b, uint64_t g,
		uint64_t *ids, size_t cap)
{
	struct cds_ft_range_iter *it;
	struct cds_ft_range_node *n;
	size_t k = 0;

	if (cds_ft_range_iter_create(ftr, &it) < 0)
		return -1;
	rcu_read_lock();
	cds_ft_range_lookup_overlap(it, q_a, q_b, g);
	while ((n = cds_ft_range_iter_node(it)) != NULL) {
		struct rec *r = caa_container_of(n, struct rec, node);
		if (k >= cap) {
			rcu_read_unlock();
			cds_ft_range_iter_destroy(it);
			return -1;
		}
		ids[k++] = r->id;
		cds_ft_range_iter_next(it);
	}
	rcu_read_unlock();
	cds_ft_range_iter_destroy(it);
	return (int) k;
}

static int u64_cmp(const void *a, const void *b)
{
	uint64_t x = *(const uint64_t *) a;
	uint64_t y = *(const uint64_t *) b;
	return (x > y) - (x < y);
}

static int test_overlap_parity(void)
{
	struct cds_ft_range *ftr;
	const size_t N = 300;
	const size_t Q = 20;
	struct shadow *shadow = NULL;
	uint64_t *got_ids = NULL, *want_ids = NULL;
	int ret = -1;
	size_t cap = N;

	struct cds_ft_group *group = make_group();
	if (cds_ft_range_create(group, NULL, &ftr) < 0) {
		cds_ft_group_destroy(group);
		return -1;
	}
	shadow = (struct shadow *) calloc(N, sizeof(*shadow));
	got_ids = (uint64_t *) calloc(cap, sizeof(*got_ids));
	want_ids = (uint64_t *) calloc(cap, sizeof(*want_ids));
	if (!shadow || !got_ids || !want_ids)
		goto out;

	prng_seed(0xbeef);
	for (size_t i = 0; i < N; i++) {
		uint64_t a = prng() & ((1ULL << 40) - 1);
		uint64_t L = 1 + (prng() & ((1ULL << 18) - 1));
		uint64_t b;
		struct rec *r;
		if (a > UINT64_MAX - L)
			a = UINT64_MAX - L;
		b = a + L;
		shadow[i].start = a;
		shadow[i].end = b;
		shadow[i].id = i;
		r = rec_alloc(a, b, i);
		if (cds_ft_range_insert(ftr, a, b, &r->node) < 0)
			goto out;
	}

	for (size_t q = 0; q < Q; q++) {
		uint64_t base = prng() & ((1ULL << 40) - 1);
		uint64_t W = 1 + (prng() & ((1ULL << 18) - 1));
		uint64_t q_a = base;
		uint64_t q_b;
		uint64_t g = (q & 3) ? 0 : (1ULL << ((prng() & 0xf)));
		size_t got_n, want_n;
		if (q_a > UINT64_MAX - W)
			q_a = UINT64_MAX - W;
		q_b = q_a + W;

		want_n = shadow_overlap(shadow, N, q_a, q_b, g, want_ids);
		int got = collect_overlap(ftr, q_a, q_b, g, got_ids, cap);
		if (got < 0) {
			fprintf(stderr, "collect_overlap failed at q=%zu\n", q);
			goto out;
		}
		got_n = (size_t) got;
		if (got_n != want_n) {
			fprintf(stderr,
				"overlap mismatch q=%zu [%" PRIu64 ", %" PRIu64 ") g=%" PRIu64
				": got %zu want %zu\n",
				q, q_a, q_b, g, got_n, want_n);
			goto out;
		}
		qsort(got_ids, got_n, sizeof(*got_ids), u64_cmp);
		qsort(want_ids, want_n, sizeof(*want_ids), u64_cmp);
		if (memcmp(got_ids, want_ids, got_n * sizeof(*got_ids)) != 0) {
			fprintf(stderr, "overlap id-set mismatch at q=%zu\n", q);
			goto out;
		}
	}

	ret = 0;
out:
	free(shadow);
	free(got_ids);
	free(want_ids);
	drain_all(ftr);
	cds_ft_range_destroy(ftr);
	cds_ft_group_destroy(group);
	return ret;
}

/* ------------------------------------------------------------------ */
/* Test 4: granularity culling                                        */
/* ------------------------------------------------------------------ */

static int test_granularity(void)
{
	struct cds_ft_range *ftr;
	const size_t N_PER_K = 50;
	const unsigned int K_MAX = 30;	/* lengths 1..2^30 */
	struct shadow *shadow = NULL;
	uint64_t *ids = NULL;
	size_t shadow_n = 0, cap;
	int ret = -1;

	struct cds_ft_group *group = make_group();
	if (cds_ft_range_create(group, NULL, &ftr) < 0) {
		cds_ft_group_destroy(group);
		return -1;
	}
	cap = N_PER_K * (K_MAX + 1);
	shadow = (struct shadow *) calloc(cap, sizeof(*shadow));
	ids = (uint64_t *) calloc(cap, sizeof(*ids));
	if (!shadow || !ids)
		goto out;

	prng_seed(0xc0ffee);
	for (unsigned int k = 0; k <= K_MAX; k++) {
		uint64_t L_min = (k == 0) ? 1 : ((1ULL << (k - 1)) + 1);
		uint64_t L_max = 1ULL << k;
		uint64_t L_span = L_max - L_min + 1;
		for (size_t i = 0; i < N_PER_K; i++) {
			uint64_t a = prng() & ((1ULL << 40) - 1);
			uint64_t L = L_min + (prng() % L_span);
			uint64_t b;
			struct rec *r;
			if (a > UINT64_MAX - L)
				a = UINT64_MAX - L;
			b = a + L;
			shadow[shadow_n].start = a;
			shadow[shadow_n].end = b;
			shadow[shadow_n].id = shadow_n;
			r = rec_alloc(a, b, shadow_n);
			if (cds_ft_range_insert(ftr, a, b, &r->node) < 0)
				goto out;
			shadow_n++;
		}
	}

	/* For each k_min, query the whole universe with g = 2^k_min and
	 * check we get exactly the ranges with L >= g. */
	uint64_t q_a = 0, q_b = (1ULL << 41);
	for (unsigned int g_k = 0; g_k <= K_MAX + 1; g_k++) {
		uint64_t g = (g_k == 0) ? 0 : (1ULL << (g_k - 1));
		size_t want = 0;
		for (size_t i = 0; i < shadow_n; i++) {
			uint64_t L = shadow[i].end - shadow[i].start;
			if (L < g) continue;
			if (shadow[i].start < q_b && shadow[i].end > q_a)
				want++;
		}
		int got = collect_overlap(ftr, q_a, q_b, g, ids, cap);
		if (got < 0) {
			fprintf(stderr, "granularity collect failed g=%" PRIu64 "\n", g);
			goto out;
		}
		if ((size_t) got != want) {
			fprintf(stderr,
				"granularity g=%" PRIu64 ": got %d want %zu\n",
				g, got, want);
			goto out;
		}
		/* Verify each returned id has L >= g. */
		for (int j = 0; j < got; j++) {
			struct shadow *sp = &shadow[ids[j]];
			if (sp->end - sp->start < g) {
				fprintf(stderr,
					"granularity violation id=%" PRIu64 " L=%" PRIu64
					" g=%" PRIu64 "\n",
					ids[j], sp->end - sp->start, g);
				goto out;
			}
		}
	}

	ret = 0;
out:
	free(shadow);
	free(ids);
	drain_all(ftr);
	cds_ft_range_destroy(ftr);
	cds_ft_group_destroy(group);
	return ret;
}

/* ------------------------------------------------------------------ */
/* Test 5: pan locality (sanity, not a perf test)                     */
/* ------------------------------------------------------------------ */

static int test_pan(void)
{
	struct cds_ft_range *ftr;
	const size_t N = 3000;
	const size_t PANS = 100;
	struct shadow *shadow = NULL;
	uint64_t *got_ids = NULL, *want_ids = NULL;
	int ret = -1;

	struct cds_ft_group *group = make_group();
	if (cds_ft_range_create(group, NULL, &ftr) < 0) {
		cds_ft_group_destroy(group);
		return -1;
	}
	shadow = (struct shadow *) calloc(N, sizeof(*shadow));
	got_ids = (uint64_t *) calloc(N, sizeof(*got_ids));
	want_ids = (uint64_t *) calloc(N, sizeof(*want_ids));
	if (!shadow || !got_ids || !want_ids)
		goto out;

	prng_seed(0xfeed);
	for (size_t i = 0; i < N; i++) {
		uint64_t a = prng() & ((1ULL << 40) - 1);
		uint64_t L = 1 + (prng() & ((1ULL << 18) - 1));
		uint64_t b;
		struct rec *r;
		if (a > UINT64_MAX - L)
			a = UINT64_MAX - L;
		b = a + L;
		shadow[i].start = a;
		shadow[i].end = b;
		shadow[i].id = i;
		r = rec_alloc(a, b, i);
		if (cds_ft_range_insert(ftr, a, b, &r->node) < 0)
			goto out;
	}

	uint64_t W = 1ULL << 16;
	uint64_t q_a = 1ULL << 30;
	for (size_t p = 0; p < PANS; p++) {
		uint64_t Delta = prng() & ((1ULL << 18) - 1);
		uint64_t a2 = q_a + Delta;
		uint64_t b2 = a2 + W;
		size_t want_n = shadow_overlap(shadow, N, a2, b2, 0, want_ids);
		int got = collect_overlap(ftr, a2, b2, 0, got_ids, N);
		if (got < 0)
			goto out;
		if ((size_t) got != want_n) {
			fprintf(stderr,
				"pan p=%zu Delta=%" PRIu64 " mismatch: got %d want %zu\n",
				p, Delta, got, want_n);
			goto out;
		}
		qsort(got_ids, got, sizeof(*got_ids), u64_cmp);
		qsort(want_ids, want_n, sizeof(*want_ids), u64_cmp);
		if (memcmp(got_ids, want_ids, got * sizeof(*got_ids)) != 0) {
			fprintf(stderr, "pan id-set mismatch at p=%zu\n", p);
			goto out;
		}
		q_a = a2;
	}

	ret = 0;
out:
	free(shadow);
	free(got_ids);
	free(want_ids);
	drain_all(ftr);
	cds_ft_range_destroy(ftr);
	cds_ft_group_destroy(group);
	return ret;
}

/* ------------------------------------------------------------------ */
/* Test 6: edge ranges                                                */
/* ------------------------------------------------------------------ */

static int test_edges(void)
{
	struct cds_ft_range *ftr;
	int ret = -1;

	struct cds_ft_group *group = make_group();
	if (cds_ft_range_create(group, NULL, &ftr) < 0) {
		cds_ft_group_destroy(group);
		return -1;
	}

	struct {
		uint64_t s, e;
		uint64_t id;
	} edges[] = {
		{ 0, 1, 100 },			/* L=1 at zero */
		{ 0, 2, 101 },			/* L=2 at zero */
		{ 0, 4, 102 },			/* L=4, k=2 */
		{ UINT64_MAX - 1, UINT64_MAX, 200 },	/* L=1 near top */
		{ UINT64_MAX - 8, UINT64_MAX, 201 },	/* L=8 spanning top */
		{ (1ULL << 32), (1ULL << 32) + 4, 300 },
	};
	const size_t E = sizeof(edges) / sizeof(edges[0]);
	struct rec *recs[16];
	memset(recs, 0, sizeof(recs));

	for (size_t i = 0; i < E; i++) {
		recs[i] = rec_alloc(edges[i].s, edges[i].e, edges[i].id);
		if (cds_ft_range_insert(ftr, edges[i].s, edges[i].e,
					&recs[i]->node) < 0) {
			fprintf(stderr, "edge insert fail i=%zu\n", i);
			goto out;
		}
	}

	/* q_a = 0 saturation: query [0, MAX) must find all. */
	uint64_t ids[16];
	int got = collect_overlap(ftr, 0, UINT64_MAX, 0, ids, 16);
	if (got < 0) {
		fprintf(stderr, "edge collect fail\n");
		goto out;
	}
	if ((size_t) got != E) {
		fprintf(stderr, "edge wide query: got %d want %zu\n", got, E);
		goto out;
	}

	/* Stab at 0: only ranges containing 0 (just edges[0..2]). */
	got = collect_overlap(ftr, 0, 1, 0, ids, 16);
	if (got != 3) {
		fprintf(stderr, "edge stab 0: got %d want 3\n", got);
		goto out;
	}

	/* Stab at UINT64_MAX - 1: should hit edges[3] and edges[4]. */
	got = collect_overlap(ftr, UINT64_MAX - 1, UINT64_MAX, 0, ids, 16);
	if (got != 2) {
		fprintf(stderr, "edge stab top: got %d want 2\n", got);
		goto out;
	}

	/* Granularity 2 should drop the L=1 cases (edges[0], edges[3]) when
	 * combined with a wide query. */
	got = collect_overlap(ftr, 0, UINT64_MAX, 2, ids, 16);
	if ((size_t) got != E - 2) {
		fprintf(stderr, "edge granularity g=2: got %d want %zu\n",
			got, E - 2);
		goto out;
	}

	ret = 0;
out:
	drain_all(ftr);
	cds_ft_range_destroy(ftr);
	cds_ft_group_destroy(group);
	return ret;
}

/* ------------------------------------------------------------------ */
/* Test 7: stab parity                                                */
/* ------------------------------------------------------------------ */

static int collect_stab(struct cds_ft_range *ftr, uint64_t x, uint64_t g,
		uint64_t *ids, size_t cap)
{
	struct cds_ft_range_iter *it;
	struct cds_ft_range_node *n;
	size_t k = 0;

	if (cds_ft_range_iter_create(ftr, &it) < 0)
		return -1;
	rcu_read_lock();
	cds_ft_range_lookup_stab(it, x, g);
	while ((n = cds_ft_range_iter_node(it)) != NULL) {
		struct rec *r = caa_container_of(n, struct rec, node);
		if (k >= cap) {
			rcu_read_unlock();
			cds_ft_range_iter_destroy(it);
			return -1;
		}
		ids[k++] = r->id;
		cds_ft_range_iter_next(it);
	}
	rcu_read_unlock();
	cds_ft_range_iter_destroy(it);
	return (int) k;
}

static size_t shadow_stab(const struct shadow *s, size_t n,
		uint64_t x, uint64_t g, uint64_t *out)
{
	size_t k = 0;
	for (size_t i = 0; i < n; i++) {
		uint64_t L = s[i].end - s[i].start;
		if (L < g)
			continue;
		if (s[i].start <= x && s[i].end > x)
			out[k++] = s[i].id;
	}
	return k;
}

static int test_stab(void)
{
	struct cds_ft_range *ftr;
	const size_t N = 3000;
	const size_t Q = 200;
	struct shadow *shadow = NULL;
	uint64_t *got_ids = NULL, *want_ids = NULL;
	int ret = -1;
	size_t cap = N;

	struct cds_ft_group *group = make_group();
	if (cds_ft_range_create(group, NULL, &ftr) < 0) {
		cds_ft_group_destroy(group);
		return -1;
	}
	shadow = (struct shadow *) calloc(N, sizeof(*shadow));
	got_ids = (uint64_t *) calloc(cap, sizeof(*got_ids));
	want_ids = (uint64_t *) calloc(cap, sizeof(*want_ids));
	if (!shadow || !got_ids || !want_ids)
		goto out;

	prng_seed(0x57ab);
	for (size_t i = 0; i < N; i++) {
		uint64_t a = prng() & ((1ULL << 40) - 1);
		uint64_t L = 1 + (prng() & ((1ULL << 18) - 1));
		uint64_t b;
		struct rec *r;
		if (a > UINT64_MAX - L)
			a = UINT64_MAX - L;
		b = a + L;
		shadow[i].start = a;
		shadow[i].end = b;
		shadow[i].id = i;
		r = rec_alloc(a, b, i);
		if (cds_ft_range_insert(ftr, a, b, &r->node) < 0)
			goto out;
	}

	for (size_t q = 0; q < Q; q++) {
		uint64_t x;
		uint64_t g = (q & 3) ? 0 : (1ULL << ((prng() & 0xf)));
		size_t got_n, want_n;

		/* Mix of: cursor inside an existing range (use a midpoint),
		 * arbitrary point, edge cases (0, near MAX). */
		if ((q & 7) == 0) {
			x = 0;
		} else if ((q & 7) == 1) {
			x = UINT64_MAX;
		} else if ((q & 7) == 2) {
			x = UINT64_MAX - 1;
		} else if ((q & 1) && N > 0) {
			const struct shadow *s = &shadow[prng() % N];
			x = s->start + (s->end - s->start) / 2;
		} else {
			x = prng() & ((1ULL << 41) - 1);
		}

		want_n = shadow_stab(shadow, N, x, g, want_ids);
		int got = collect_stab(ftr, x, g, got_ids, cap);
		if (got < 0) {
			fprintf(stderr, "stab collect failed q=%zu\n", q);
			goto out;
		}
		got_n = (size_t) got;
		if (got_n != want_n) {
			fprintf(stderr,
				"stab mismatch q=%zu x=%" PRIu64
				" g=%" PRIu64 ": got %zu want %zu\n",
				q, x, g, got_n, want_n);
			goto out;
		}
		qsort(got_ids, got_n, sizeof(*got_ids), u64_cmp);
		qsort(want_ids, want_n, sizeof(*want_ids), u64_cmp);
		if (memcmp(got_ids, want_ids, got_n * sizeof(*got_ids)) != 0) {
			fprintf(stderr, "stab id-set mismatch q=%zu\n", q);
			goto out;
		}
	}

	ret = 0;
out:
	free(shadow);
	free(got_ids);
	free(want_ids);
	drain_all(ftr);
	cds_ft_range_destroy(ftr);
	cds_ft_group_destroy(group);
	return ret;
}

/* ------------------------------------------------------------------ */
/* Test 8: containing parity                                          */
/* ------------------------------------------------------------------ */

static int collect_containing(struct cds_ft_range *ftr,
		uint64_t q_a, uint64_t q_b, uint64_t g,
		uint64_t *ids, size_t cap)
{
	struct cds_ft_range_iter *it;
	struct cds_ft_range_node *n;
	size_t k = 0;

	if (cds_ft_range_iter_create(ftr, &it) < 0)
		return -1;
	rcu_read_lock();
	cds_ft_range_lookup_containing(it, q_a, q_b, g);
	while ((n = cds_ft_range_iter_node(it)) != NULL) {
		struct rec *r = caa_container_of(n, struct rec, node);
		if (k >= cap) {
			rcu_read_unlock();
			cds_ft_range_iter_destroy(it);
			return -1;
		}
		ids[k++] = r->id;
		cds_ft_range_iter_next(it);
	}
	rcu_read_unlock();
	cds_ft_range_iter_destroy(it);
	return (int) k;
}

static size_t shadow_containing(const struct shadow *s, size_t n,
		uint64_t q_a, uint64_t q_b, uint64_t g, uint64_t *out)
{
	size_t k = 0;
	for (size_t i = 0; i < n; i++) {
		uint64_t L = s[i].end - s[i].start;
		if (L < g)
			continue;
		if (s[i].start <= q_a && s[i].end >= q_b)
			out[k++] = s[i].id;
	}
	return k;
}

static int test_containing(void)
{
	struct cds_ft_range *ftr;
	const size_t N = 3000;
	const size_t Q = 200;
	struct shadow *shadow = NULL;
	uint64_t *got_ids = NULL, *want_ids = NULL;
	int ret = -1;
	size_t cap = N;

	struct cds_ft_group *group = make_group();
	if (cds_ft_range_create(group, NULL, &ftr) < 0) {
		cds_ft_group_destroy(group);
		return -1;
	}
	shadow = (struct shadow *) calloc(N, sizeof(*shadow));
	got_ids = (uint64_t *) calloc(cap, sizeof(*got_ids));
	want_ids = (uint64_t *) calloc(cap, sizeof(*want_ids));
	if (!shadow || !got_ids || !want_ids)
		goto out;

	prng_seed(0xc057);
	for (size_t i = 0; i < N; i++) {
		uint64_t a = prng() & ((1ULL << 40) - 1);
		uint64_t L = 1 + (prng() & ((1ULL << 20) - 1));
		uint64_t b;
		struct rec *r;
		if (a > UINT64_MAX - L)
			a = UINT64_MAX - L;
		b = a + L;
		shadow[i].start = a;
		shadow[i].end = b;
		shadow[i].id = i;
		r = rec_alloc(a, b, i);
		if (cds_ft_range_insert(ftr, a, b, &r->node) < 0)
			goto out;
	}

	for (size_t q = 0; q < Q; q++) {
		uint64_t q_a, q_b, W;
		uint64_t g = (q & 3) ? 0 : (1ULL << ((prng() & 0xf)));
		size_t got_n, want_n;

		/* Mix of "tight window inside an existing range" (likely
		 * to find containers) and arbitrary windows (often empty).
		 * Using small W skews toward at least one container being
		 * present, exercising the result-yield path. */
		if ((q & 1) && N > 0) {
			const struct shadow *s = &shadow[prng() % N];
			uint64_t mid = s->start + (s->end - s->start) / 2;
			W = 1 + (prng() & 0xff);
			q_a = mid;
			if (q_a > UINT64_MAX - W)
				q_a = UINT64_MAX - W;
			q_b = q_a + W;
		} else {
			W = 1 + (prng() & ((1ULL << 14) - 1));
			q_a = prng() & ((1ULL << 41) - 1);
			if (q_a > UINT64_MAX - W)
				q_a = UINT64_MAX - W;
			q_b = q_a + W;
		}

		want_n = shadow_containing(shadow, N, q_a, q_b, g, want_ids);
		int got = collect_containing(ftr, q_a, q_b, g,
			got_ids, cap);
		if (got < 0) {
			fprintf(stderr, "containing collect failed q=%zu\n", q);
			goto out;
		}
		got_n = (size_t) got;
		if (got_n != want_n) {
			fprintf(stderr,
				"containing mismatch q=%zu [%" PRIu64
				", %" PRIu64 ") g=%" PRIu64
				": got %zu want %zu\n",
				q, q_a, q_b, g, got_n, want_n);
			goto out;
		}
		qsort(got_ids, got_n, sizeof(*got_ids), u64_cmp);
		qsort(want_ids, want_n, sizeof(*want_ids), u64_cmp);
		if (memcmp(got_ids, want_ids,
				got_n * sizeof(*got_ids)) != 0) {
			fprintf(stderr,
				"containing id-set mismatch q=%zu\n", q);
			goto out;
		}
	}

	ret = 0;
out:
	free(shadow);
	free(got_ids);
	free(want_ids);
	drain_all(ftr);
	cds_ft_range_destroy(ftr);
	cds_ft_group_destroy(group);
	return ret;
}

/* ------------------------------------------------------------------ */
/* Test 9: contained_in parity                                        */
/* ------------------------------------------------------------------ */

static int collect_contained_in(struct cds_ft_range *ftr,
		uint64_t q_a, uint64_t q_b, uint64_t g,
		uint64_t *ids, size_t cap)
{
	struct cds_ft_range_iter *it;
	struct cds_ft_range_node *n;
	size_t k = 0;

	if (cds_ft_range_iter_create(ftr, &it) < 0)
		return -1;
	rcu_read_lock();
	cds_ft_range_lookup_contained_in(it, q_a, q_b, g);
	while ((n = cds_ft_range_iter_node(it)) != NULL) {
		struct rec *r = caa_container_of(n, struct rec, node);
		if (k >= cap) {
			rcu_read_unlock();
			cds_ft_range_iter_destroy(it);
			return -1;
		}
		ids[k++] = r->id;
		cds_ft_range_iter_next(it);
	}
	rcu_read_unlock();
	cds_ft_range_iter_destroy(it);
	return (int) k;
}

static size_t shadow_contained_in(const struct shadow *s, size_t n,
		uint64_t q_a, uint64_t q_b, uint64_t g, uint64_t *out)
{
	size_t k = 0;
	for (size_t i = 0; i < n; i++) {
		uint64_t L = s[i].end - s[i].start;
		if (L < g)
			continue;
		if (s[i].start >= q_a && s[i].end <= q_b)
			out[k++] = s[i].id;
	}
	return k;
}

static int test_contained_in(void)
{
	struct cds_ft_range *ftr;
	const size_t N = 3000;
	const size_t Q = 200;
	struct shadow *shadow = NULL;
	uint64_t *got_ids = NULL, *want_ids = NULL;
	int ret = -1;
	size_t cap = N;

	struct cds_ft_group *group = make_group();
	if (cds_ft_range_create(group, NULL, &ftr) < 0) {
		cds_ft_group_destroy(group);
		return -1;
	}
	shadow = (struct shadow *) calloc(N, sizeof(*shadow));
	got_ids = (uint64_t *) calloc(cap, sizeof(*got_ids));
	want_ids = (uint64_t *) calloc(cap, sizeof(*want_ids));
	if (!shadow || !got_ids || !want_ids)
		goto out;

	prng_seed(0xc011);
	for (size_t i = 0; i < N; i++) {
		uint64_t a = prng() & ((1ULL << 40) - 1);
		uint64_t L = 1 + (prng() & ((1ULL << 18) - 1));
		uint64_t b;
		struct rec *r;
		if (a > UINT64_MAX - L)
			a = UINT64_MAX - L;
		b = a + L;
		shadow[i].start = a;
		shadow[i].end = b;
		shadow[i].id = i;
		r = rec_alloc(a, b, i);
		if (cds_ft_range_insert(ftr, a, b, &r->node) < 0)
			goto out;
	}

	for (size_t q = 0; q < Q; q++) {
		uint64_t base = prng() & ((1ULL << 40) - 1);
		/* Window widths: a mix of "around typical L" (most contained)
		 * and "tight" (few contained) to exercise level pruning. */
		unsigned int wbits = 1 + (prng() % 23);
		uint64_t W = 1 + (prng() & ((1ULL << wbits) - 1));
		uint64_t q_a = base;
		uint64_t q_b;
		uint64_t g = (q & 3) ? 0 : (1ULL << ((prng() & 0xf)));
		size_t got_n, want_n;

		if (q_a > UINT64_MAX - W)
			q_a = UINT64_MAX - W;
		q_b = q_a + W;

		want_n = shadow_contained_in(shadow, N, q_a, q_b, g, want_ids);
		int got = collect_contained_in(ftr, q_a, q_b, g,
			got_ids, cap);
		if (got < 0) {
			fprintf(stderr, "contained_in collect failed q=%zu\n", q);
			goto out;
		}
		got_n = (size_t) got;
		if (got_n != want_n) {
			fprintf(stderr,
				"contained_in mismatch q=%zu [%" PRIu64
				", %" PRIu64 ") g=%" PRIu64
				": got %zu want %zu\n",
				q, q_a, q_b, g, got_n, want_n);
			goto out;
		}
		qsort(got_ids, got_n, sizeof(*got_ids), u64_cmp);
		qsort(want_ids, want_n, sizeof(*want_ids), u64_cmp);
		if (memcmp(got_ids, want_ids,
				got_n * sizeof(*got_ids)) != 0) {
			fprintf(stderr,
				"contained_in id-set mismatch q=%zu\n", q);
			goto out;
		}
	}

	ret = 0;
out:
	free(shadow);
	free(got_ids);
	free(want_ids);
	drain_all(ftr);
	cds_ft_range_destroy(ftr);
	cds_ft_group_destroy(group);
	return ret;
}

/* ------------------------------------------------------------------ */
/* Test 10: overlap_band parity                                       */
/* ------------------------------------------------------------------ */

static size_t shadow_overlap_band(const struct shadow *s, size_t n,
		uint64_t q_a, uint64_t q_b,
		uint64_t length_lo, uint64_t length_hi,
		uint64_t *out_ids)
{
	size_t k = 0;
	for (size_t i = 0; i < n; i++) {
		uint64_t L = s[i].end - s[i].start;
		if (L < length_lo || L >= length_hi)
			continue;
		if (s[i].start < q_b && s[i].end > q_a)
			out_ids[k++] = s[i].id;
	}
	return k;
}

static int collect_overlap_band(struct cds_ft_range *ftr,
		uint64_t q_a, uint64_t q_b,
		uint64_t length_lo, uint64_t length_hi,
		uint64_t *ids, size_t cap)
{
	struct cds_ft_range_iter *it;
	struct cds_ft_range_node *n;
	size_t k = 0;

	if (cds_ft_range_iter_create(ftr, &it) < 0)
		return -1;
	rcu_read_lock();
	cds_ft_range_lookup_overlap_band(it, q_a, q_b, length_lo, length_hi);
	while ((n = cds_ft_range_iter_node(it)) != NULL) {
		struct rec *r = caa_container_of(n, struct rec, node);
		if (k >= cap) {
			rcu_read_unlock();
			cds_ft_range_iter_destroy(it);
			return -1;
		}
		ids[k++] = r->id;
		cds_ft_range_iter_next(it);
	}
	rcu_read_unlock();
	cds_ft_range_iter_destroy(it);
	return (int) k;
}

static int test_overlap_band(void)
{
	struct cds_ft_range *ftr;
	const size_t N = 3000;
	const size_t Q = 100;
	struct shadow *shadow = NULL;
	uint64_t *got_ids = NULL, *want_ids = NULL;
	int ret = -1;
	size_t cap = N;

	struct cds_ft_group *group = make_group();
	if (cds_ft_range_create(group, NULL, &ftr) < 0) {
		cds_ft_group_destroy(group);
		return -1;
	}
	shadow = (struct shadow *) calloc(N, sizeof(*shadow));
	got_ids = (uint64_t *) calloc(cap, sizeof(*got_ids));
	want_ids = (uint64_t *) calloc(cap, sizeof(*want_ids));
	if (!shadow || !got_ids || !want_ids)
		goto out;

	prng_seed(0xb47d);
	for (size_t i = 0; i < N; i++) {
		uint64_t a = prng() & ((1ULL << 40) - 1);
		uint64_t L = 1 + (prng() & ((1ULL << 20) - 1));
		uint64_t b;
		struct rec *r;
		if (a > UINT64_MAX - L)
			a = UINT64_MAX - L;
		b = a + L;
		shadow[i].start = a;
		shadow[i].end = b;
		shadow[i].id = i;
		r = rec_alloc(a, b, i);
		if (cds_ft_range_insert(ftr, a, b, &r->node) < 0)
			goto out;
	}

	for (size_t q = 0; q < Q; q++) {
		uint64_t base = prng() & ((1ULL << 40) - 1);
		uint64_t W = 1 + (prng() & ((1ULL << 18) - 1));
		uint64_t q_a = base;
		uint64_t q_b;
		uint64_t lo_bits = prng() & 0x1f;	/* 0..31 */
		uint64_t hi_bits = lo_bits + 1 + (prng() & 0xf);	/* lo+1..lo+16 */
		uint64_t length_lo = (lo_bits == 0) ? 0 : (1ULL << lo_bits);
		uint64_t length_hi = (hi_bits >= 63) ? UINT64_MAX
			: (1ULL << hi_bits);
		size_t got_n, want_n;

		if (q_a > UINT64_MAX - W)
			q_a = UINT64_MAX - W;
		q_b = q_a + W;

		want_n = shadow_overlap_band(shadow, N, q_a, q_b,
			length_lo, length_hi, want_ids);
		int got = collect_overlap_band(ftr, q_a, q_b,
			length_lo, length_hi, got_ids, cap);
		if (got < 0) {
			fprintf(stderr, "collect_overlap_band failed q=%zu\n", q);
			goto out;
		}
		got_n = (size_t) got;
		if (got_n != want_n) {
			fprintf(stderr,
				"overlap_band mismatch q=%zu lo=%" PRIu64
				" hi=%" PRIu64 ": got %zu want %zu\n",
				q, length_lo, length_hi, got_n, want_n);
			goto out;
		}
		qsort(got_ids, got_n, sizeof(*got_ids), u64_cmp);
		qsort(want_ids, want_n, sizeof(*want_ids), u64_cmp);
		if (memcmp(got_ids, want_ids, got_n * sizeof(*got_ids)) != 0) {
			fprintf(stderr, "overlap_band id-set mismatch q=%zu\n", q);
			goto out;
		}
	}

	ret = 0;
out:
	free(shadow);
	free(got_ids);
	free(want_ids);
	drain_all(ftr);
	cds_ft_range_destroy(ftr);
	cds_ft_group_destroy(group);
	return ret;
}

/* ------------------------------------------------------------------ */
/* Test 11: entering / leaving parity                                 */
/* ------------------------------------------------------------------ */

static size_t shadow_entering(const struct shadow *s, size_t n,
		uint64_t old_a, uint64_t old_b,
		uint64_t new_a, uint64_t new_b,
		uint64_t g, uint64_t *out)
{
	size_t k = 0;
	for (size_t i = 0; i < n; i++) {
		uint64_t L = s[i].end - s[i].start;
		bool in_new = (s[i].start < new_b && s[i].end > new_a);
		bool in_old = (s[i].start < old_b && s[i].end > old_a);
		if (L < g)
			continue;
		if (in_new && !in_old)
			out[k++] = s[i].id;
	}
	return k;
}

static size_t shadow_leaving(const struct shadow *s, size_t n,
		uint64_t old_a, uint64_t old_b,
		uint64_t new_a, uint64_t new_b,
		uint64_t g, uint64_t *out)
{
	size_t k = 0;
	for (size_t i = 0; i < n; i++) {
		uint64_t L = s[i].end - s[i].start;
		bool in_new = (s[i].start < new_b && s[i].end > new_a);
		bool in_old = (s[i].start < old_b && s[i].end > old_a);
		if (L < g)
			continue;
		if (in_old && !in_new)
			out[k++] = s[i].id;
	}
	return k;
}

static int collect_entering(struct cds_ft_range *ftr,
		uint64_t old_a, uint64_t old_b,
		uint64_t new_a, uint64_t new_b,
		uint64_t g, uint64_t *ids, size_t cap)
{
	struct cds_ft_range_iter *it;
	struct cds_ft_range_node *n;
	size_t k = 0;

	if (cds_ft_range_iter_create(ftr, &it) < 0)
		return -1;
	rcu_read_lock();
	cds_ft_range_lookup_entering(it, old_a, old_b, new_a, new_b, g);
	while ((n = cds_ft_range_iter_node(it)) != NULL) {
		struct rec *r = caa_container_of(n, struct rec, node);
		if (k >= cap) {
			rcu_read_unlock();
			cds_ft_range_iter_destroy(it);
			return -1;
		}
		ids[k++] = r->id;
		cds_ft_range_iter_next(it);
	}
	rcu_read_unlock();
	cds_ft_range_iter_destroy(it);
	return (int) k;
}

static int collect_leaving(struct cds_ft_range *ftr,
		uint64_t old_a, uint64_t old_b,
		uint64_t new_a, uint64_t new_b,
		uint64_t g, uint64_t *ids, size_t cap)
{
	struct cds_ft_range_iter *it;
	struct cds_ft_range_node *n;
	size_t k = 0;

	if (cds_ft_range_iter_create(ftr, &it) < 0)
		return -1;
	rcu_read_lock();
	cds_ft_range_lookup_leaving(it, old_a, old_b, new_a, new_b, g);
	while ((n = cds_ft_range_iter_node(it)) != NULL) {
		struct rec *r = caa_container_of(n, struct rec, node);
		if (k >= cap) {
			rcu_read_unlock();
			cds_ft_range_iter_destroy(it);
			return -1;
		}
		ids[k++] = r->id;
		cds_ft_range_iter_next(it);
	}
	rcu_read_unlock();
	cds_ft_range_iter_destroy(it);
	return (int) k;
}

/*
 * Filter a got-set against the brute-force "leaving fast path" predicate
 * (ranges that are in OLD AND not in NEW).  The library is allowed to
 * return additional ranges in mixed-transition cases (it falls back to
 * a full re-query of OLD), so a got-set that is a strict superset of
 * the want-set is still acceptable as long as every reported range
 * was at least in OLD with L >= g.
 */
static bool got_is_consistent_with_old_overlap(const struct shadow *s, size_t n,
		uint64_t old_a, uint64_t old_b, uint64_t g,
		const uint64_t *got, size_t got_n)
{
	for (size_t i = 0; i < got_n; i++) {
		const struct shadow *r = &s[got[i]];
		uint64_t L = r->end - r->start;
		if (L < g)
			return false;
		if (!(r->start < old_b && r->end > old_a))
			return false;
	}
	return true;
}

static bool got_is_consistent_with_new_overlap(const struct shadow *s, size_t n,
		uint64_t new_a, uint64_t new_b, uint64_t g,
		const uint64_t *got, size_t got_n)
{
	for (size_t i = 0; i < got_n; i++) {
		const struct shadow *r = &s[got[i]];
		uint64_t L = r->end - r->start;
		if (L < g)
			return false;
		if (!(r->start < new_b && r->end > new_a))
			return false;
	}
	return true;
}

/*
 * Verify that for "fast-path" pans (pure pan-right, pure pan-left, no
 * overlap, identity, contained-within) the reported entering / leaving
 * sets exactly match brute force.  Other transitions (widening,
 * shrinking, mixed) accept superset-of-brute results as long as the
 * superset stays within OLD or NEW respectively.
 */
static bool is_fast_path_pan(uint64_t old_a, uint64_t old_b,
		uint64_t new_a, uint64_t new_b)
{
	bool overlap = !(new_a >= old_b || new_b <= old_a);
	bool contained = (new_a >= old_a && new_b <= old_b)
		|| (new_a <= old_a && new_b >= old_b);
	bool pan_right = (new_a > old_a && new_b > old_b);
	bool pan_left = (new_a < old_a && new_b < old_b);
	if (!overlap)
		return true;	/* fall-back to full new/old overlap, exact */
	if (contained)
		return true;	/* exact (entering or leaving is empty) */
	return pan_right || pan_left;
}

static int test_entering_leaving(void)
{
	struct cds_ft_range *ftr;
	const size_t N = 3000;
	const size_t Q = 200;
	struct shadow *shadow = NULL;
	uint64_t *got_ids = NULL, *want_ids = NULL;
	int ret = -1;
	size_t cap = N;

	struct cds_ft_group *group = make_group();
	if (cds_ft_range_create(group, NULL, &ftr) < 0) {
		cds_ft_group_destroy(group);
		return -1;
	}
	shadow = (struct shadow *) calloc(N, sizeof(*shadow));
	got_ids = (uint64_t *) calloc(cap, sizeof(*got_ids));
	want_ids = (uint64_t *) calloc(cap, sizeof(*want_ids));
	if (!shadow || !got_ids || !want_ids)
		goto out;

	prng_seed(0xed91);
	for (size_t i = 0; i < N; i++) {
		uint64_t a = prng() & ((1ULL << 40) - 1);
		uint64_t L = 1 + (prng() & ((1ULL << 18) - 1));
		uint64_t b;
		struct rec *r;
		if (a > UINT64_MAX - L)
			a = UINT64_MAX - L;
		b = a + L;
		shadow[i].start = a;
		shadow[i].end = b;
		shadow[i].id = i;
		r = rec_alloc(a, b, i);
		if (cds_ft_range_insert(ftr, a, b, &r->node) < 0)
			goto out;
	}

	for (size_t q = 0; q < Q; q++) {
		uint64_t old_a = prng() & ((1ULL << 40) - 1);
		uint64_t W = 1 + (prng() & ((1ULL << 18) - 1));
		uint64_t old_b;
		int64_t delta;
		uint64_t new_a, new_b;
		uint64_t g = (q & 3) ? 0 : (1ULL << ((prng() & 0xf)));

		if (old_a > UINT64_MAX - W)
			old_a = UINT64_MAX - W;
		old_b = old_a + W;

		/* delta in [-2W, 2W). */
		delta = (int64_t)(prng() & ((4ULL * W) - 1)) - (int64_t)(2ULL * W);
		new_a = (delta >= 0)
			? (old_a + (uint64_t) delta > UINT64_MAX - W
				? UINT64_MAX - W : old_a + (uint64_t) delta)
			: ((uint64_t)(-delta) >= old_a
				? 0 : old_a - (uint64_t)(-delta));
		new_b = (new_a > UINT64_MAX - W) ? UINT64_MAX : new_a + W;
		if (new_b <= new_a)
			continue;	/* skip degenerate after clamp */

		bool fast = is_fast_path_pan(old_a, old_b, new_a, new_b);

		/* Entering. */
		size_t want_n = shadow_entering(shadow, N, old_a, old_b,
			new_a, new_b, g, want_ids);
		int got = collect_entering(ftr, old_a, old_b, new_a, new_b,
			g, got_ids, cap);
		if (got < 0)
			goto out;
		size_t got_n = (size_t) got;
		if (fast) {
			if (got_n != want_n) {
				fprintf(stderr,
					"entering count mismatch q=%zu: got %zu want %zu\n",
					q, got_n, want_n);
				goto out;
			}
			qsort(got_ids, got_n, sizeof(*got_ids), u64_cmp);
			qsort(want_ids, want_n, sizeof(*want_ids), u64_cmp);
			if (memcmp(got_ids, want_ids,
					got_n * sizeof(*got_ids)) != 0) {
				fprintf(stderr,
					"entering id-set mismatch q=%zu\n", q);
				goto out;
			}
		} else {
			if (!got_is_consistent_with_new_overlap(shadow, N,
					new_a, new_b, g, got_ids, got_n)) {
				fprintf(stderr,
					"entering fallback returned an out-of-NEW range q=%zu\n",
					q);
				goto out;
			}
		}

		/* Leaving. */
		want_n = shadow_leaving(shadow, N, old_a, old_b,
			new_a, new_b, g, want_ids);
		got = collect_leaving(ftr, old_a, old_b, new_a, new_b,
			g, got_ids, cap);
		if (got < 0)
			goto out;
		got_n = (size_t) got;
		if (fast) {
			if (got_n != want_n) {
				fprintf(stderr,
					"leaving count mismatch q=%zu: got %zu want %zu\n",
					q, got_n, want_n);
				goto out;
			}
			qsort(got_ids, got_n, sizeof(*got_ids), u64_cmp);
			qsort(want_ids, want_n, sizeof(*want_ids), u64_cmp);
			if (memcmp(got_ids, want_ids,
					got_n * sizeof(*got_ids)) != 0) {
				fprintf(stderr,
					"leaving id-set mismatch q=%zu\n", q);
				goto out;
			}
		} else {
			if (!got_is_consistent_with_old_overlap(shadow, N,
					old_a, old_b, g, got_ids, got_n)) {
				fprintf(stderr,
					"leaving fallback returned an out-of-OLD range q=%zu\n",
					q);
				goto out;
			}
		}
	}

	ret = 0;
out:
	free(shadow);
	free(got_ids);
	free(want_ids);
	drain_all(ftr);
	cds_ft_range_destroy(ftr);
	cds_ft_group_destroy(group);
	return ret;
}

/* ------------------------------------------------------------------ */
/* Test 12: empty / count_keys / count_entries                        */
/* ------------------------------------------------------------------ */

#define COUNT_TEST_N 1000

static int test_count(void)
{
	struct cds_ft_range *ftr;
	struct rec **recs = NULL;
	int ret = -1;

	struct cds_ft_group *group = make_group();
	if (cds_ft_range_create(group, NULL, &ftr) < 0) {
		cds_ft_group_destroy(group);
		return -1;
	}
	recs = (struct rec **) calloc(COUNT_TEST_N, sizeof(*recs));
	if (!recs)
		goto out;

	rcu_read_lock();
	if (!cds_ft_range_empty(ftr)) {
		fprintf(stderr, "fresh index reports non-empty\n");
		rcu_read_unlock();
		goto out;
	}
	if (cds_ft_range_count_keys(ftr) != 0) {
		fprintf(stderr, "fresh index has non-zero key count\n");
		rcu_read_unlock();
		goto out;
	}
	if (cds_ft_range_count_entries(ftr) != 0) {
		fprintf(stderr, "fresh index has non-zero entry count\n");
		rcu_read_unlock();
		goto out;
	}
	rcu_read_unlock();

	for (size_t i = 0; i < COUNT_TEST_N; i++) {
		uint64_t a = i * 17;
		uint64_t L = 1 + (i & 0xff);
		recs[i] = rec_alloc(a, a + L, i);
		if (cds_ft_range_insert(ftr, a, a + L, &recs[i]->node) < 0)
			goto out;
	}

	rcu_read_lock();
	if (cds_ft_range_empty(ftr)) {
		fprintf(stderr, "populated index reports empty\n");
		rcu_read_unlock();
		goto out;
	}
	if (cds_ft_range_count_entries(ftr) != COUNT_TEST_N) {
		fprintf(stderr,
			"count_entries: got %lu, expected %d\n",
			cds_ft_range_count_entries(ftr), COUNT_TEST_N);
		rcu_read_unlock();
		goto out;
	}
	/* Each range routes to exactly one level by its length class;
	 * starts (0, 17, 34, ...) are unique within each level, so
	 * count_keys totals to N. */
	if (cds_ft_range_count_keys(ftr) != COUNT_TEST_N) {
		fprintf(stderr,
			"count_keys: got %lu, expected %d\n",
			cds_ft_range_count_keys(ftr), COUNT_TEST_N);
		rcu_read_unlock();
		goto out;
	}
	rcu_read_unlock();

	ret = 0;
out:
	free(recs);
	drain_all(ftr);
	cds_ft_range_destroy(ftr);
	cds_ft_group_destroy(group);
	return ret;
}

/* ------------------------------------------------------------------ */
/* Test 13: merge / detach                                            */
/* ------------------------------------------------------------------ */

/*
 * Exercises the bulk merge + detach pattern.  Three sub-scenarios:
 *
 *   A. Cold load: merge a populated staging into an empty live.
 *      cds_ft_merge picks the fast subtree-graft path at every
 *      level (live has nothing under any prefix).
 *
 *   B. Chunk-disjoint incremental merge: build a second staging
 *      with starts in a higher byte-prefix range.  Merging into a
 *      now-non-empty live still hits the fast path because each
 *      level's content sits under a different prefix from live's.
 *
 *   C. Overlapping merge: build a third staging whose starts share
 *      bytes with live's existing content (forced collision via
 *      reused length class and overlapping start range).
 *      cds_ft_merge falls back to per-entry insert; the union is
 *      still correct.
 *
 * Then drains live by detaching into a new index and checks the
 * detached index has the full union.
 */
static int test_merge_detach(void)
{
	struct cds_ft_group *group;
	struct cds_ft_range *live = NULL, *detached = NULL;
	struct cds_ft_range *staging_a = NULL, *staging_b = NULL, *staging_c = NULL;
	const size_t N_A = 1000, N_B = 500, N_C = 200;
	struct rec **recs_a = NULL, **recs_b = NULL, **recs_c = NULL;
	uint64_t *got_ids = NULL;
	const size_t total = N_A + N_B + N_C;
	int ret = -1;

	group = make_group();
	if (cds_ft_range_create(group, NULL, &live) < 0
			|| cds_ft_range_create(group, NULL, &staging_a) < 0
			|| cds_ft_range_create(group, NULL, &staging_b) < 0
			|| cds_ft_range_create(group, NULL, &staging_c) < 0)
		goto out;

	recs_a = (struct rec **) calloc(N_A, sizeof(*recs_a));
	recs_b = (struct rec **) calloc(N_B, sizeof(*recs_b));
	recs_c = (struct rec **) calloc(N_C, sizeof(*recs_c));
	got_ids = (uint64_t *) calloc(total, sizeof(*got_ids));
	if (!recs_a || !recs_b || !recs_c || !got_ids)
		goto out;

	/* Staging A: starts in [0, 2^31). */
	prng_seed(0xa11ae);
	for (size_t i = 0; i < N_A; i++) {
		uint64_t a = prng() & ((1ULL << 31) - 1);
		uint64_t L = 1 + (prng() & ((1ULL << 14) - 1));
		recs_a[i] = rec_alloc(a, a + L, i);
		if (cds_ft_range_insert(staging_a, a, a + L,
				&recs_a[i]->node) < 0)
			goto out;
	}
	/* Staging B: starts in [2^33, 2^33 + 2^31) - disjoint high-byte
	 * prefix from staging A's range, exercising the fast path. */
	for (size_t i = 0; i < N_B; i++) {
		uint64_t a = (1ULL << 33) | (prng() & ((1ULL << 31) - 1));
		uint64_t L = 1 + (prng() & ((1ULL << 14) - 1));
		recs_b[i] = rec_alloc(a, a + L, N_A + i);
		if (cds_ft_range_insert(staging_b, a, a + L,
				&recs_b[i]->node) < 0)
			goto out;
	}
	/* Staging C: starts in [0, 2^31) - overlapping with staging A's
	 * range, forcing the per-entry path. */
	for (size_t i = 0; i < N_C; i++) {
		uint64_t a = prng() & ((1ULL << 31) - 1);
		uint64_t L = 1 + (prng() & ((1ULL << 14) - 1));
		recs_c[i] = rec_alloc(a, a + L, N_A + N_B + i);
		if (cds_ft_range_insert(staging_c, a, a + L,
				&recs_c[i]->node) < 0)
			goto out;
	}

	/* Sub-scenario A: merge staging_a into empty live (cold load). */
	if (cds_ft_range_merge(live, staging_a) < 0) {
		fprintf(stderr, "A: merge into empty live failed\n");
		goto out;
	}
	rcu_read_lock();
	if (!cds_ft_range_empty(staging_a)) {
		fprintf(stderr, "A: staging not empty after merge\n");
		rcu_read_unlock();
		goto out;
	}
	if (cds_ft_range_count_entries(live) != N_A) {
		fprintf(stderr, "A: live count != N_A (%lu vs %zu)\n",
			cds_ft_range_count_entries(live), N_A);
		rcu_read_unlock();
		goto out;
	}
	rcu_read_unlock();

	/* Sub-scenario B: merge staging_b into populated live, with
	 * disjoint high-byte prefixes -> cds_ft_merge fast path. */
	if (cds_ft_range_merge(live, staging_b) < 0) {
		fprintf(stderr, "B: merge of disjoint staging failed\n");
		goto out;
	}
	rcu_read_lock();
	if (!cds_ft_range_empty(staging_b)) {
		fprintf(stderr, "B: staging not empty after merge\n");
		rcu_read_unlock();
		goto out;
	}
	if (cds_ft_range_count_entries(live) != N_A + N_B) {
		fprintf(stderr, "B: live count != N_A+N_B (%lu vs %zu)\n",
			cds_ft_range_count_entries(live), N_A + N_B);
		rcu_read_unlock();
		goto out;
	}
	rcu_read_unlock();

	/* No-op merge of empty staging. */
	if (cds_ft_range_merge(live, staging_b) < 0) {
		fprintf(stderr, "no-op merge failed\n");
		goto out;
	}

	/* Sub-scenario C: merge staging_c into live with overlapping
	 * starts -> per-entry fallback in cds_ft_merge. */
	if (cds_ft_range_merge(live, staging_c) < 0) {
		fprintf(stderr, "C: merge with overlap failed\n");
		goto out;
	}
	rcu_read_lock();
	if (!cds_ft_range_empty(staging_c)) {
		fprintf(stderr, "C: staging not empty after merge\n");
		rcu_read_unlock();
		goto out;
	}
	if (cds_ft_range_count_entries(live) != total) {
		fprintf(stderr, "C: live count != total (%lu vs %zu)\n",
			cds_ft_range_count_entries(live), total);
		rcu_read_unlock();
		goto out;
	}
	int got = collect_overlap(live, 0, UINT64_MAX, 0, got_ids, total);
	if (got != (int) total) {
		fprintf(stderr,
			"live overlap returned %d, expected %zu\n",
			got, total);
		rcu_read_unlock();
		goto out;
	}
	rcu_read_unlock();

	/* Detach all of live into a new index. */
	if (cds_ft_range_detach(live, &detached) < 0) {
		fprintf(stderr, "detach failed\n");
		goto out;
	}
	cds_ft_range_make_concurrent(detached);

	rcu_read_lock();
	if (!cds_ft_range_empty(live)) {
		fprintf(stderr, "live not empty after detach\n");
		rcu_read_unlock();
		goto out;
	}
	if (cds_ft_range_count_entries(detached) != total) {
		fprintf(stderr,
			"detached count != total (got %lu)\n",
			cds_ft_range_count_entries(detached));
		rcu_read_unlock();
		goto out;
	}
	got = collect_overlap(detached, 0, UINT64_MAX, 0, got_ids, total);
	if (got != (int) total) {
		fprintf(stderr,
			"detached overlap returned %d, expected %zu\n",
			got, total);
		rcu_read_unlock();
		goto out;
	}
	rcu_read_unlock();

	ret = 0;
out:
	if (detached) {
		drain_all(detached);
		cds_ft_range_destroy(detached);
	}
	if (live) {
		drain_all(live);
		cds_ft_range_destroy(live);
	}
	if (staging_a) {
		drain_all(staging_a);
		cds_ft_range_destroy(staging_a);
	}
	if (staging_b) {
		drain_all(staging_b);
		cds_ft_range_destroy(staging_b);
	}
	if (staging_c) {
		drain_all(staging_c);
		cds_ft_range_destroy(staging_c);
	}
	cds_ft_group_destroy(group);
	free(recs_a);
	free(recs_b);
	free(recs_c);
	free(got_ids);
	return ret;
}

/* ------------------------------------------------------------------ */
/* Test 14: concurrent reader + writer                                */
/* ------------------------------------------------------------------ */

#define CONCURRENT_DURATION_MS	1500
#define CONCURRENT_KEY_SPACE	(1ULL << 20)
#define CONCURRENT_MAX_LEN	(1ULL << 14)
#define CONCURRENT_NR_READERS	3
#define CONCURRENT_INITIAL	2000

struct conc_ctx {
	struct cds_ft_range *ftr;
	pthread_mutex_t writer_lock;
	atomic_int stop;
	atomic_ulong reader_iters;
	atomic_ulong reader_violations;
};

static void *conc_writer(void *arg)
{
	struct conc_ctx *ctx = (struct conc_ctx *) arg;
	uint64_t s = 0xC0DE;
	rcu_register_thread();

	while (!atomic_load(&ctx->stop)) {
		uint64_t a, L, b;
		struct rec *r;
		s ^= s << 13; s ^= s >> 7; s ^= s << 17;
		a = s & (CONCURRENT_KEY_SPACE - 1);
		L = 1 + (s & (CONCURRENT_MAX_LEN - 1));
		b = a + L;
		r = rec_alloc(a, b, 0);
		pthread_mutex_lock(&ctx->writer_lock);
		if (cds_ft_range_insert(ctx->ftr, a, b, &r->node) < 0) {
			pthread_mutex_unlock(&ctx->writer_lock);
			rec_free(r);	/* counted as alloc'd, free here keeps balance */
			continue;
		}
		pthread_mutex_unlock(&ctx->writer_lock);

		/* Occasionally remove a random previously-inserted node by
		 * iterating one shot and unhooking the first match. */
		if ((s & 7) == 0) {
			struct cds_ft_range_iter *it;
			struct cds_ft_range_node *n;

			if (cds_ft_range_iter_create(ctx->ftr, &it) < 0)
				continue;
			rcu_read_lock();
			cds_ft_range_lookup_overlap(it, a, a + 1, 0);
			n = cds_ft_range_iter_node(it);
			rcu_read_unlock();
			if (n) {
				struct rec *victim = caa_container_of(n,
					struct rec, node);
				pthread_mutex_lock(&ctx->writer_lock);
				if (cds_ft_range_remove(ctx->ftr,
						victim->start, &victim->node)
						== CDS_FT_STATUS_OK) {
					pthread_mutex_unlock(&ctx->writer_lock);
					call_rcu(&victim->rcu, rec_free_rcu_cb);
				} else {
					pthread_mutex_unlock(&ctx->writer_lock);
				}
			}
			cds_ft_range_iter_destroy(it);
		}
		rcu_quiescent_state();
	}
	rcu_unregister_thread();
	return NULL;
}

static void *conc_reader(void *arg)
{
	struct conc_ctx *ctx = (struct conc_ctx *) arg;
	uint64_t s = 0xACE1;
	rcu_register_thread();

	while (!atomic_load(&ctx->stop)) {
		struct cds_ft_range_iter *it;
		struct cds_ft_range_node *n;
		uint64_t q_a, q_b;
		s ^= s << 13; s ^= s >> 7; s ^= s << 17;
		q_a = s & (CONCURRENT_KEY_SPACE - 1);
		q_b = q_a + 1 + (s & 0x3ff);

		if (cds_ft_range_iter_create(ctx->ftr, &it) < 0)
			continue;
		rcu_read_lock();
		cds_ft_range_lookup_overlap(it, q_a, q_b, 0);
		while ((n = cds_ft_range_iter_node(it)) != NULL) {
			struct rec *r = caa_container_of(n, struct rec, node);
			/* Invariant: returned ranges must overlap [q_a, q_b). */
			if (!(r->start < q_b && r->end > q_a)) {
				atomic_fetch_add(&ctx->reader_violations, 1);
			}
			cds_ft_range_iter_next(it);
		}
		rcu_read_unlock();
		cds_ft_range_iter_destroy(it);
		atomic_fetch_add(&ctx->reader_iters, 1);
		if ((atomic_load(&ctx->reader_iters) & 0xff) == 0)
			rcu_quiescent_state();
	}
	rcu_unregister_thread();
	return NULL;
}

static int test_concurrent(void)
{
	struct conc_ctx ctx;
	struct cds_ft_group *group;
	pthread_t writer, readers[CONCURRENT_NR_READERS];
	int ret = -1;

	group = make_group();
	if (cds_ft_range_create(group, NULL, &ctx.ftr) < 0) {
		cds_ft_group_destroy(group);
		return -1;
	}
	pthread_mutex_init(&ctx.writer_lock, NULL);
	atomic_store(&ctx.stop, 0);
	atomic_store(&ctx.reader_iters, 0);
	atomic_store(&ctx.reader_violations, 0);

	/* Pre-populate so readers see a non-empty index immediately. */
	for (size_t i = 0; i < CONCURRENT_INITIAL; i++) {
		uint64_t a = (i * 1009) & (CONCURRENT_KEY_SPACE - 1);
		uint64_t L = 1 + (i & (CONCURRENT_MAX_LEN - 1));
		struct rec *r = rec_alloc(a, a + L, i);
		if (cds_ft_range_insert(ctx.ftr, a, a + L, &r->node) < 0) {
			rec_free(r);
			goto out;
		}
	}

	if (pthread_create(&writer, NULL, conc_writer, &ctx) != 0)
		goto out;
	for (int i = 0; i < CONCURRENT_NR_READERS; i++) {
		if (pthread_create(&readers[i], NULL, conc_reader, &ctx) != 0)
			goto out;
	}

	usleep(CONCURRENT_DURATION_MS * 1000);
	atomic_store(&ctx.stop, 1);
	pthread_join(writer, NULL);
	for (int i = 0; i < CONCURRENT_NR_READERS; i++)
		pthread_join(readers[i], NULL);

	if (atomic_load(&ctx.reader_violations) != 0) {
		fprintf(stderr, "concurrent: %lu invariant violations\n",
			atomic_load(&ctx.reader_violations));
		goto out;
	}
	if (atomic_load(&ctx.reader_iters) == 0) {
		fprintf(stderr, "concurrent: readers ran zero iterations\n");
		goto out;
	}
	ret = 0;
out:
	pthread_mutex_destroy(&ctx.writer_lock);
	drain_all(ctx.ftr);
	cds_ft_range_destroy(ctx.ftr);
	cds_ft_group_destroy(group);
	return ret;
}

/* ------------------------------------------------------------------ */
/* Test runner                                                        */
/* ------------------------------------------------------------------ */

#define RUN_TEST(fn)							\
	do {								\
		leak_reset();						\
		rcu_quiescent_state();					\
		ok((fn)() == 0 && leak_check() == 0, "%s", #fn);	\
	} while (0)

int main(void)
{
	rcu_register_thread();
	plan_tests(NR_TESTS);

	RUN_TEST(test_routing);
	RUN_TEST(test_stab_self);
	RUN_TEST(test_overlap_parity);
	RUN_TEST(test_granularity);
	RUN_TEST(test_pan);
	RUN_TEST(test_edges);
	RUN_TEST(test_stab);
	RUN_TEST(test_containing);
	RUN_TEST(test_contained_in);
	RUN_TEST(test_overlap_band);
	RUN_TEST(test_entering_leaving);
	RUN_TEST(test_count);
	RUN_TEST(test_merge_detach);
	/* Concurrent test is the slowest; run last. */
	RUN_TEST(test_concurrent);

	rcu_unregister_thread();
	return exit_status();
}
