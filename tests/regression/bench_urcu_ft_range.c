// SPDX-FileCopyrightText: 2026 EfficiOS Inc.
//
// SPDX-License-Identifier: LGPL-2.1-or-later

/*
 * bench_urcu_ft_range.c
 *
 * Microbenchmark for cds_ft_range. Three measurements:
 *
 *   1. Pan locality:
 *        Insert N synthetic state ranges. Run Q queries panning by
 *        a fixed Δ, with a fixed viewport width W. Vary Δ from 1 to
 *        many W. Report per-query latency and how it scales with Δ.
 *        The expected behaviour is that small Δ (relative to W) is
 *        as cheap as a wide query because the per-level scan windows
 *        shift only slightly.
 *
 *   2. Granularity culling:
 *        Same dataset. Run Q queries with the same wide viewport at
 *        different granularity floors g. Report per-query latency
 *        and result count. Expected: per-query cost roughly scales
 *        with output (ranges with L >= g) rather than total ranges.
 *
 *   3. Brute-force baseline:
 *        For the same wide viewport, scan all N ranges linearly to
 *        confirm correctness and to put the index numbers in
 *        context.
 *
 * Built with RCU_QSBR. Single-threaded; no contention. Run under
 * -O2 -DNDEBUG (the project's benchmark profile per CLAUDE.md).
 */

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <urcu/compiler.h>
#include <urcu-qsbr.h>
#include <urcu/fractal-trie.h>
#include <urcu/fractal-trie-range.h>

struct rec {
	struct cds_ft_range_node node;
	uint64_t start;
	uint64_t end;
	uint64_t id;
};

/* ------------------------------------------------------------------ */
/* PRNG                                                               */
/* ------------------------------------------------------------------ */

static uint64_t prng_state = 0xfeedfaceULL;

static uint64_t prng(void)
{
	uint64_t x = prng_state;
	x ^= x >> 12; x ^= x << 25; x ^= x >> 27;
	prng_state = x;
	return x * 0x2545F4914F6CDD1DULL;
}

static void prng_seed(uint64_t s)
{
	prng_state = s ? s : 1;
}

/* Log-uniform on [1, 2^bits]. */
static uint64_t log_uniform(unsigned int bits)
{
	unsigned int b = 1 + (prng() % bits);
	uint64_t span = 1ULL << b;
	return 1 + (prng() & (span - 1));
}

/* ------------------------------------------------------------------ */
/* Timing                                                             */
/* ------------------------------------------------------------------ */

static double now_sec(void)
{
	struct timespec t;
	clock_gettime(CLOCK_MONOTONIC, &t);
	return (double)t.tv_sec + (double)t.tv_nsec * 1e-9;
}

/* ------------------------------------------------------------------ */
/* Group / index helpers                                              */
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
/* Workload                                                           */
/* ------------------------------------------------------------------ */

#define UNIVERSE_BITS	32		/* keys span [0, 2^32) */
#define LEN_BITS	18		/* lengths log-uniform on [1, 2^18] */

static struct rec *records;
static size_t N_records;

static void populate(struct cds_ft_range *ftr, size_t N)
{
	N_records = N;
	records = (struct rec *) calloc(N, sizeof(*records));
	if (!records)
		abort();
	prng_seed(0xC0FFEE);
	for (size_t i = 0; i < N; i++) {
		uint64_t start = prng() & ((1ULL << UNIVERSE_BITS) - 1);
		uint64_t L = log_uniform(LEN_BITS);
		if (start > UINT64_MAX - L)
			start = UINT64_MAX - L;
		records[i].start = start;
		records[i].end = start + L;
		records[i].id = i;
		cds_ft_range_node_init(&records[i].node);
		if (cds_ft_range_insert(ftr, start, start + L,
					&records[i].node) < 0)
			abort();
	}
}

static void cleanup(struct cds_ft_range *ftr)
{
	struct cds_ft_range_iter *it;
	struct cds_ft_range_node **batch = NULL;
	uint64_t *starts = NULL;
	size_t cap = 0, n = 0;

	if (cds_ft_range_iter_create(ftr, &it) < 0)
		abort();
	rcu_read_lock();
	cds_ft_range_lookup_overlap(it, 0, UINT64_MAX, 0);
	for (struct cds_ft_range_node *node;
			(node = cds_ft_range_iter_node(it)) != NULL; ) {
		if (n == cap) {
			cap = cap ? cap * 2 : 256;
			batch = (struct cds_ft_range_node **) realloc(batch,
				cap * sizeof(*batch));
			starts = (uint64_t *) realloc(starts,
				cap * sizeof(*starts));
		}
		struct rec *r = caa_container_of(node, struct rec, node);
		batch[n] = node;
		starts[n] = r->start;
		n++;
		cds_ft_range_iter_next(it);
	}
	rcu_read_unlock();
	cds_ft_range_iter_destroy(it);

	for (size_t i = 0; i < n; i++)
		cds_ft_range_remove(ftr, starts[i], batch[i]);
	free(batch);
	free(starts);
	free(records);
	records = NULL;
	N_records = 0;
}

/* ------------------------------------------------------------------ */
/* Query helpers                                                      */
/* ------------------------------------------------------------------ */

static size_t query_count(struct cds_ft_range *ftr,
		uint64_t q_a, uint64_t q_b, uint64_t g)
{
	struct cds_ft_range_iter *it;
	size_t c = 0;

	if (cds_ft_range_iter_create(ftr, &it) < 0)
		abort();
	rcu_read_lock();
	cds_ft_range_lookup_overlap(it, q_a, q_b, g);
	while (cds_ft_range_iter_node(it) != NULL) {
		c++;
		cds_ft_range_iter_next(it);
	}
	rcu_read_unlock();
	cds_ft_range_iter_destroy(it);
	return c;
}

static size_t bruteforce_count(uint64_t q_a, uint64_t q_b, uint64_t g)
{
	size_t c = 0;
	for (size_t i = 0; i < N_records; i++) {
		uint64_t L = records[i].end - records[i].start;
		if (L < g)
			continue;
		if (records[i].start < q_b && records[i].end > q_a)
			c++;
	}
	return c;
}

/* ------------------------------------------------------------------ */
/* Benchmark 1: pan locality                                          */
/* ------------------------------------------------------------------ */

static void bench_pan(struct cds_ft_range *ftr)
{
	const uint64_t W = 1ULL << 22;	/* viewport width */
	const size_t Q = 200;		/* panned queries per Δ */
	uint64_t deltas[] = {
		1, 16, 256, 4096, 65536,
		1ULL << 18, 1ULL << 20,
		1ULL << 22, 1ULL << 24,
	};
	const size_t Nd = sizeof(deltas) / sizeof(deltas[0]);

	printf("\n# pan-locality benchmark\n");
	printf("# viewport W = %" PRIu64 ", Q = %zu queries per Δ\n", W, Q);
	printf("# %-12s %-12s %-12s %-10s\n",
		"Δ", "ns/query", "results", "Δ/W");
	for (size_t di = 0; di < Nd; di++) {
		uint64_t Delta = deltas[di];
		uint64_t q_a = 1ULL << 30;	/* arbitrary anchor */
		size_t total = 0;
		double t0, t1;

		t0 = now_sec();
		for (size_t q = 0; q < Q; q++) {
			uint64_t a = q_a + q * Delta;
			if (a > UINT64_MAX - W)
				a = UINT64_MAX - W;
			total += query_count(ftr, a, a + W, 0);
		}
		t1 = now_sec();
		printf("  %-12" PRIu64 " %-12.0f %-12.0f %-10.3f\n",
			Delta, (t1 - t0) * 1e9 / Q,
			(double) total / Q, (double) Delta / W);
	}
}

/* ------------------------------------------------------------------ */
/* Benchmark 2: granularity culling                                   */
/* ------------------------------------------------------------------ */

static void bench_granularity(struct cds_ft_range *ftr)
{
	const uint64_t W = 1ULL << 22;
	const size_t Q = 500;
	uint64_t gs[] = {
		0, 16, 256, 4096, 65536, 1ULL << 18,
	};
	const size_t Ng = sizeof(gs) / sizeof(gs[0]);

	printf("\n# granularity-culling benchmark\n");
	printf("# viewport W = %" PRIu64 ", Q = %zu random windows\n", W, Q);
	printf("# %-12s %-12s %-12s\n", "g", "ns/query", "results");
	for (size_t gi = 0; gi < Ng; gi++) {
		uint64_t g = gs[gi];
		size_t total = 0;
		double t0, t1;

		prng_seed(0xC0FFEE + gi);
		t0 = now_sec();
		for (size_t q = 0; q < Q; q++) {
			uint64_t a = prng() & ((1ULL << UNIVERSE_BITS) - 1);
			if (a > UINT64_MAX - W)
				a = UINT64_MAX - W;
			total += query_count(ftr, a, a + W, g);
		}
		t1 = now_sec();
		printf("  %-12" PRIu64 " %-12.0f %-12.0f\n",
			g, (t1 - t0) * 1e9 / Q,
			(double) total / Q);
	}
}

/* ------------------------------------------------------------------ */
/* Benchmark 3: brute force comparison                                */
/* ------------------------------------------------------------------ */

static void bench_bruteforce(struct cds_ft_range *ftr)
{
	const uint64_t W = 1ULL << 22;
	const size_t Q = 100;
	double t_bf, t_idx;
	size_t total_bf = 0, total_idx = 0;

	prng_seed(0xC0FFEE);
	double t0 = now_sec();
	for (size_t q = 0; q < Q; q++) {
		uint64_t a = prng() & ((1ULL << UNIVERSE_BITS) - 1);
		if (a > UINT64_MAX - W)
			a = UINT64_MAX - W;
		total_bf += bruteforce_count(a, a + W, 0);
	}
	t_bf = now_sec() - t0;

	prng_seed(0xC0FFEE);
	t0 = now_sec();
	for (size_t q = 0; q < Q; q++) {
		uint64_t a = prng() & ((1ULL << UNIVERSE_BITS) - 1);
		if (a > UINT64_MAX - W)
			a = UINT64_MAX - W;
		total_idx += query_count(ftr, a, a + W, 0);
	}
	t_idx = now_sec() - t0;

	printf("\n# brute-force vs. index, viewport W = %" PRIu64 ", Q = %zu\n",
		W, Q);
	printf("  brute-force:  %8.0f ns/query  (mean %.0f results)\n",
		t_bf * 1e9 / Q, (double) total_bf / Q);
	printf("  cds_ft_range: %8.0f ns/query  (mean %.0f results)\n",
		t_idx * 1e9 / Q, (double) total_idx / Q);
	printf("  speedup:      %.1fx\n", t_bf / t_idx);
	if (total_bf != total_idx)
		fprintf(stderr,
			"  *** RESULT MISMATCH: bf=%zu vs idx=%zu ***\n",
			total_bf, total_idx);
}

/* ------------------------------------------------------------------ */
/* Main                                                               */
/* ------------------------------------------------------------------ */

int main(int argc, char **argv)
{
	struct cds_ft_group *group;
	struct cds_ft_range *ftr;
	size_t N = (argc > 1) ? (size_t) strtoul(argv[1], NULL, 0) : 1000000;

	rcu_register_thread();
	group = make_group();
	if (cds_ft_range_create(group, NULL, &ftr) < 0)
		abort();

	printf("# cds_ft_range benchmark\n");
	printf("# N = %zu ranges, key universe [0, 2^%u),"
		" lengths log-uniform [1, 2^%u]\n",
		N, UNIVERSE_BITS, LEN_BITS);

	double t0 = now_sec();
	populate(ftr, N);
	double t_populate = now_sec() - t0;
	printf("# populate: %.3f s (%.0f ns/insert)\n",
		t_populate, t_populate * 1e9 / N);

	bench_bruteforce(ftr);
	bench_granularity(ftr);
	bench_pan(ftr);

	cleanup(ftr);
	cds_ft_range_destroy(ftr);
	cds_ft_group_destroy(group);
	rcu_unregister_thread();
	return 0;
}
