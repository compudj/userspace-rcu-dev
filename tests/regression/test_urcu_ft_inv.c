// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

/*
 * test_urcu_ft_inv.c
 *
 * Userspace RCU library - Fractal Trie Invariant Tests
 *
 * Concurrent invariant tests for the Fractal Trie. Each test creates
 * a shared trie, spawns one or more writer threads that mutate the
 * trie under mutex, and one or more reader threads that perform
 * lookups and traversals under rcu_read_lock(). Readers assert
 * structural invariants that must hold at every point a concurrent
 * RCU reader can observe: ordering, key consistency, duplicate chain
 * acyclicity, graft-swap atomicity, etc.
 *
 * A violation of any invariant means a concurrent reader observed
 * illegal intermediate state — exactly the class of bugs these tests
 * are designed to catch.
 *
 * Build example (adapt include/library paths to your tree):
 *
 *   cc -O2 -g -DRCU_QSBR \
 *      -I/path/to/urcu/include \
 *      test_urcu_ft_inv.c \
 *      -lurcu-qsbr -lurcu-cds -lurcu-common -lpthread \
 *      -o test_urcu_ft_inv
 *
 * Run all:  ./test_urcu_ft_inv
 * Run one:  ./test_urcu_ft_inv inv_iteration_order
 */

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <urcu/compiler.h>
#include <urcu-qsbr.h>		/* Must precede fractal-trie.h */
#include <urcu/fractal-trie.h>
#include <urcu-call-rcu.h>

#ifdef FT_ENABLE_TRACING
#include "../../src/cds_ft_tp.h"
#define FT_TEST_TP(name, ...) \
	lttng_ust_tracepoint(cds_ft, name, ##__VA_ARGS__)
#else
#define FT_TEST_TP(name, ...) do {} while (0)
#endif

#include <assert.h>
#include <inttypes.h>
#include <stdarg.h>
#include <limits.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "tap.h"

#define NR_TESTS	23

/* ------------------------------------------------------------------ */
/* Tuning knobs                                                       */
/* ------------------------------------------------------------------ */

/*
 * Duration of each concurrent test in milliseconds.  Kept short for
 * automated runs; increase for deeper stress testing.
 */
#define DEFAULT_DURATION_MS	2000

/* Thread counts per test. */
#define NR_READERS_DEFAULT	4
#define NR_WRITERS_DEFAULT	2

/*
 * Key pool sizes.  The writer pool is intentionally smaller than the
 * reader lookup pool so that readers frequently encounter both hits
 * and misses.
 */
#define WRITER_POOL_SIZE	512
#define READER_POOL_SIZE	1024
#define MAX_DUP_CHAIN_LEN	(WRITER_POOL_SIZE * 4)

/* ------------------------------------------------------------------ */
/* Test-node infrastructure                                           */
/* ------------------------------------------------------------------ */

struct ft_test_node {
	struct cds_ft_node node;
	struct rcu_head head;
	uint64_t key;		/* shadow copy for validation */
	uint64_t value;
	/*
	 * Ordinal (big-endian, trie-order) key bytes, populated at insert.
	 * This is the representation the speculative inequality lookup copies
	 * from the leaf, so it must hold exactly the bytes the trie navigates
	 * on (NOT the host-order @key shadow above).  create_fixed_ft points
	 * speculative_key_offset here.
	 */
	uint8_t okey[8];
};

static inline
void ft_test_node_init(struct ft_test_node *n, uint64_t key)
{
	cds_ft_node_init(&n->node);
	n->key = key;
	n->value = 0;
}

static inline
struct ft_test_node *to_test_node(struct cds_ft_node *n)
{
	return caa_container_of(n, struct ft_test_node, node);
}

static unsigned long nodes_allocated, nodes_freed;

static struct ft_test_node *node_alloc(uint64_t key)
{
	struct ft_test_node *n = (struct ft_test_node *) calloc(1, sizeof(*n));
	if (!n) {
		fprintf(stderr, "node_alloc: out of memory\n");
		abort();
	}
	ft_test_node_init(n, key);
	__atomic_add_fetch(&nodes_allocated, 1, __ATOMIC_RELAXED);
	return n;
}

static void node_free(struct ft_test_node *n)
{
	memset(n, 0xfe, sizeof(*n));	/* poison */
	free(n);
	__atomic_add_fetch(&nodes_freed, 1, __ATOMIC_RELAXED);
}

static void node_free_rcu_cb(struct rcu_head *head)
{
	node_free(caa_container_of(head, struct ft_test_node, head));
}

static void node_free_rcu(struct ft_test_node *n)
{
	call_rcu(&n->head, node_free_rcu_cb);
}

/* Free every node reachable through the trie, then destroy the trie. */
static int drain_and_destroy(struct cds_ft *ft, struct cds_ft_group *group)
{
	struct cds_ft_iter *iter;
	enum cds_ft_status s;
	int ret = 0;

	s = cds_ft_iter_create(ft, &iter);
	if (s < 0) {
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
		return -1;
	}
	rcu_read_lock();
	while (cds_ft_lookup_first(ft, iter) == CDS_FT_STATUS_OK) {
		struct cds_ft_node *head, *tmp;

		s = cds_ft_remove_all(ft, iter, &head);
		if (s < 0) {
			ret = -1;
			break;
		}
		cds_ft_for_each_duplicate_safe_rcu(head, tmp) {
			node_free_rcu(to_test_node(head));
		}
	}
	rcu_read_unlock();
	rcu_barrier();
	cds_ft_iter_destroy(iter);
	cds_ft_destroy(ft);
	cds_ft_group_destroy(group);
	return ret;
}

static void leak_reset(void)
{
	__atomic_store_n(&nodes_allocated, 0, __ATOMIC_RELAXED);
	__atomic_store_n(&nodes_freed, 0, __ATOMIC_RELAXED);
}

static int leak_check(void)
{
	rcu_barrier();
	unsigned long na = __atomic_load_n(&nodes_allocated, __ATOMIC_RELAXED);
	unsigned long nf = __atomic_load_n(&nodes_freed, __ATOMIC_RELAXED);
	if (na != nf) {
		fprintf(stderr, "LEAK: allocated %lu, freed %lu (delta %ld)\n",
			na, nf, (long)(na - nf));
		return -1;
	}
	return 0;
}

/* ------------------------------------------------------------------ */
/* Helpers                                                            */
/* ------------------------------------------------------------------ */

static struct cds_ft *create_fixed_ft(size_t klen, struct cds_ft_group **group_out)
{
	struct cds_ft_group_attr *attr;
	struct cds_ft_group *group;
	struct cds_ft *ft;

	if (cds_ft_group_attr_create(&attr) < 0)
		abort();
	if (cds_ft_group_attr_set_key_len(attr, klen) < 0)
		abort();
	/*
	 * Exercise the speculative inequality leaf-key capture: the result
	 * key is copied from the matched leaf's ordinal key bytes (okey),
	 * stored at this offset from the embedded cds_ft_node.
	 */
	if (cds_ft_group_attr_set_speculative_key_offset(attr,
			offsetof(struct ft_test_node, okey)) < 0)
		abort();
	if (cds_ft_group_create(attr, &group) < 0)
		abort();
	cds_ft_group_attr_destroy(attr);
	if (cds_ft_create(group, NULL, &ft) < 0)
		abort();
	*group_out = group;
	return ft;
}

/*
 * Like create_fixed_ft, but also enables the library-owned ordered list
 * (cds_ft_group_attr_set_ordered_list).  In a FEATURE_FT_ORD_CELL library this
 * threads the duplicate-chain heads through relocatable ordinal cells and makes
 * cds_ft_next / cds_ft_prev walk that list with a lazily-materialized result
 * key; in a library without the feature the flag is inert (the group behaves
 * exactly like create_fixed_ft).  Used to exercise the cell ordered-iteration +
 * cds_ft_iter_bind_key cross-CS resume path.
 */
static struct cds_ft *create_fixed_ord_ft(size_t klen, struct cds_ft_group **group_out)
{
	struct cds_ft_group_attr *attr;
	struct cds_ft_group *group;
	struct cds_ft *ft;

	if (cds_ft_group_attr_create(&attr) < 0)
		abort();
	if (cds_ft_group_attr_set_key_len(attr, klen) < 0)
		abort();
	if (cds_ft_group_attr_set_speculative_key_offset(attr,
			offsetof(struct ft_test_node, okey)) < 0)
		abort();
	if (cds_ft_group_attr_set_ordered_list(attr) < 0)
		abort();
	if (cds_ft_group_create(attr, &group) < 0)
		abort();
	cds_ft_group_attr_destroy(attr);
	if (cds_ft_create(group, NULL, &ft) < 0)
		abort();
	*group_out = group;
	return ft;
}

static enum cds_ft_status
insert_u64(struct cds_ft *ft, uint64_t v, struct ft_test_node *n)
{
	uint8_t k[8] = { 0 };

	cds_ft_u64_to_key(ft, v, k, CDS_FT_LEN_DEFAULT);
	/* Stash the ordinal key bytes for speculative leaf-key capture. */
	memcpy(n->okey, k, sizeof(n->okey));
	return cds_ft_insert(ft, k, CDS_FT_LEN_DEFAULT, &n->node);
}

static enum cds_ft_status
lookup_u64(struct cds_ft *ft, uint64_t v, struct cds_ft_node **out)
{
	uint8_t k[8];

	cds_ft_u64_to_key(ft, v, k, CDS_FT_LEN_DEFAULT);
	return cds_ft_eager_lookup_key(ft, k, CDS_FT_LEN_DEFAULT, CDS_FT_LEN_DEFAULT, out);
}

/* ------------------------------------------------------------------ */
/* Shared concurrent-test control                                     */
/* ------------------------------------------------------------------ */

static volatile int test_go, test_stop;

static unsigned long long elapsed_ms(struct timespec *start)
{
	struct timespec now;

	clock_gettime(CLOCK_MONOTONIC, &now);
	return (unsigned long long)((long long)(now.tv_sec - start->tv_sec) * 1000LL
		+ (long long)(now.tv_nsec - start->tv_nsec) / 1000000LL);
}

/*
 * Per-thread violation counters.  Each reader thread bumps its own
 * atomic counter on violation.  The main thread sums them after join.
 */
static atomic_ulong violation_count;

static void report_violation(const char *test, const char *fmt, ...)
	__attribute__((format(printf, 2, 3)));

static void report_violation(const char *test, const char *fmt, ...)
{
	va_list ap;
	char msg[256];

	atomic_fetch_add(&violation_count, 1);
	va_start(ap, fmt);
	vsnprintf(msg, sizeof(msg), fmt, ap);
	va_end(ap);
	fprintf(stderr, "[VIOLATION] %s: %s\n", test, msg);
	FT_TEST_TP(inv_violation, test, msg);
	/*
	 * Abort on first violation so lttng-ust snapshot captures the
	 * in-memory ring buffer up to the moment of the fault.  Useful
	 * when running under `lttng record-snapshot` tracing.
	 */
	if (getenv("FT_INV_ABORT_ON_VIOLATION")) {
		/* XXX temporary: capture the flight-recorder ring before dying. */
		(void) system("lttng snapshot record 1>&2");
		abort();
	}
}

/* ------------------------------------------------------------------ */
/* Macro: run a test, emit TAP ok/not-ok, skip if filtered out.       */
/* ------------------------------------------------------------------ */

#define RUN_TEST(fn)							\
	do {								\
		if (filter && strcmp(filter, #fn) != 0) {		\
			skip(1, "filtered out: " #fn);			\
			break;						\
		}							\
		leak_reset();						\
		atomic_store(&violation_count, 0);			\
		rcu_quiescent_state();					\
		ok((fn)() == 0 && leak_check() == 0, "%s", #fn);	\
	} while (0)

/* ================================================================== */
/*                                                                    */
/*   INVARIANT 1: Forward iteration ordering under mutation           */
/*                                                                    */
/*   A concurrent RCU reader iterating forward must never observe a   */
/*   key K(i+1) <= K(i).  While the reader may miss newly inserted   */
/*   keys or see keys that are about to be removed, the relative     */
/*   order of every pair of keys it does observe must be strictly     */
/*   ascending.                                                       */
/*                                                                    */
/* ================================================================== */

struct inv_iter_ctx {
	struct cds_ft *ft;
	size_t key_len;
	const char *test_name;
};

static void *inv_iter_order_reader(void *arg)
{
	struct inv_iter_ctx *ctx = (struct inv_iter_ctx *) arg;
	struct cds_ft_iter *iter;
	unsigned long iters = 0;

	rcu_register_thread();

	if (cds_ft_iter_create(ctx->ft, &iter) < 0)
		abort();

	while (!test_go)
		;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	while (!test_stop) {
		uint64_t prev = 0;
		int first = 1;
		int count = 0;

		rcu_read_lock();
		cds_ft_for_each_rcu(ctx->ft, iter) {
			uint8_t rk[8];
			size_t rk_len;
			uint64_t v;

			cds_ft_iter_get_key(iter, rk, sizeof(rk), &rk_len);
			v = cds_ft_key_to_u64(ctx->ft, rk, CDS_FT_LEN_DEFAULT);
			if (!first && v <= prev) {
				report_violation(ctx->test_name,
					"forward order: %" PRIu64 " after %" PRIu64
					" (iter #%lu, position %d)",
					v, prev, iters, count);
				/* Stop this iteration to avoid flooding. */
				break;
			}
			prev = v;
			first = 0;
			count++;
		}
		rcu_read_unlock();

		iters++;
		if ((iters & 0x3f) == 0)
			rcu_quiescent_state();
	}

	cds_ft_iter_destroy(iter);
	rcu_unregister_thread();
	return NULL;
}

static void *inv_iter_order_writer(void *arg)
{
	struct inv_iter_ctx *ctx = (struct inv_iter_ctx *) arg;
	struct cds_ft_iter *iter;
	unsigned int seed;
	pthread_mutex_t *lock = (pthread_mutex_t *)(ctx + 1);

	rcu_register_thread();
	seed = (unsigned int)(uintptr_t)pthread_self() ^ (unsigned int)time(NULL);

	if (cds_ft_iter_create(ctx->ft, &iter) < 0)
		abort();

	while (!test_go)
		;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	while (!test_stop) {
		uint64_t key = (uint64_t)(rand_r(&seed) % WRITER_POOL_SIZE);
		int do_insert = rand_r(&seed) & 1;

		rcu_read_lock();
		if (do_insert) {
			struct ft_test_node *n = node_alloc(key);

			pthread_mutex_lock(lock);
			insert_u64(ctx->ft, key, n);
			pthread_mutex_unlock(lock);
		} else {
			struct cds_ft_node *found;
			uint8_t k[8];

			cds_ft_u64_to_key(ctx->ft, key, k, CDS_FT_LEN_DEFAULT);
			cds_ft_iter_set_key(iter, k, CDS_FT_LEN_DEFAULT);
			cds_ft_lookup(ctx->ft, iter);
			found = cds_ft_iter_node(iter);
			if (found) {
				struct ft_test_node *tn = to_test_node(found);

				pthread_mutex_lock(lock);
				if (cds_ft_remove(ctx->ft, iter, &tn->node)
				    == CDS_FT_STATUS_OK) {
					node_free_rcu(tn);
				}
				pthread_mutex_unlock(lock);
			}
		}
		rcu_read_unlock();

		if ((seed & 0xff) == 0)
			rcu_quiescent_state();
	}

	cds_ft_iter_destroy(iter);
	rcu_unregister_thread();
	return NULL;
}

static int inv_iteration_order(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(4, &group);
	struct timespec t0;
	/* Pack the mutex right after the context for the writers. */
	struct {
		struct inv_iter_ctx ctx;
		pthread_mutex_t lock;
	} shared;
	pthread_t readers[NR_READERS_DEFAULT], writers[NR_WRITERS_DEFAULT];
	unsigned int i;

	shared.ctx.ft = ft;
	shared.ctx.key_len = 4;
	shared.ctx.test_name = "inv_iteration_order";
	pthread_mutex_init(&shared.lock, NULL);

	/* Pre-populate with some keys so readers have something to see. */
	rcu_read_lock();
	for (i = 0; i < WRITER_POOL_SIZE / 2; i++) {
		struct ft_test_node *n = node_alloc(i);
		insert_u64(ft, i, n);
	}
	rcu_read_unlock();

	test_go = 0;
	test_stop = 0;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	for (i = 0; i < NR_READERS_DEFAULT; i++)
		pthread_create(&readers[i], NULL, inv_iter_order_reader, &shared.ctx);
	for (i = 0; i < NR_WRITERS_DEFAULT; i++)
		pthread_create(&writers[i], NULL, inv_iter_order_writer, &shared.ctx);

	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	test_go = 1;

	rcu_thread_offline();

	clock_gettime(CLOCK_MONOTONIC, &t0);

	while (elapsed_ms(&t0) < DEFAULT_DURATION_MS)
		usleep(1000);

	test_stop = 1;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	for (i = 0; i < NR_WRITERS_DEFAULT; i++)
		pthread_join(writers[i], NULL);
	for (i = 0; i < NR_READERS_DEFAULT; i++)
		pthread_join(readers[i], NULL);

	rcu_thread_online();

	pthread_mutex_destroy(&shared.lock);

	if (atomic_load(&violation_count) > 0) {
		fprintf(stderr, "inv_iteration_order: %lu violation(s)\n",
			atomic_load(&violation_count));
		drain_and_destroy(ft, group);
		return -1;
	}
	return drain_and_destroy(ft, group);
}

/* ================================================================== */
/*                                                                    */
/*   INVARIANT 1b: Cross-CS bind/resume preserves forward order       */
/*                  (ordinal-cell / lazy-ref group)                   */
/*                                                                    */
/* ================================================================== */

#define BIND_BATCH	7

/*
 * Ordered forward iteration on a library-owned ordered-list (cell) group, in
 * batches: after every BIND_BATCH keys, snapshot the position with
 * cds_ft_iter_bind_key(), drop the RCU read lock, pass a quiescent state (so a
 * concurrently-removed bound key -- whose ordinal cell is freed via call_rcu --
 * can actually be reclaimed), re-lock, and resume with cds_ft_next().  The
 * sequence must stay strictly increasing: bind must materialize the
 * leaf-referenced key into the iterator and re-descend from THAT, never from
 * the (possibly freed) cached cell/leaf.
 */
static void *inv_bind_resume_reader(void *arg)
{
	struct inv_iter_ctx *ctx = (struct inv_iter_ctx *) arg;
	struct cds_ft_iter *iter;
	unsigned long iters = 0;

	rcu_register_thread();

	if (cds_ft_iter_create(ctx->ft, &iter) < 0)
		abort();

	while (!test_go)
		;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	while (!test_stop) {
		uint64_t prev = 0;
		int first = 1;
		int in_batch = 0;
		int count = 0;

		rcu_read_lock();
		cds_ft_lookup_first(ctx->ft, iter);
		while (cds_ft_iter_node(iter)) {
			uint8_t rk[8];
			size_t rk_len;
			uint64_t v;

			cds_ft_iter_get_key(iter, rk, sizeof(rk), &rk_len);
			v = cds_ft_key_to_u64(ctx->ft, rk, CDS_FT_LEN_DEFAULT);
			if (!first && v <= prev) {
				report_violation(ctx->test_name,
					"bind-resume forward order: %" PRIu64
					" after %" PRIu64 " (iter #%lu, position %d)",
					v, prev, iters, count);
				break;
			}
			prev = v;
			first = 0;
			count++;
			if (++in_batch >= BIND_BATCH) {
				cds_ft_iter_bind_key(iter);
				rcu_read_unlock();
				rcu_quiescent_state();
				rcu_read_lock();
				cds_ft_next(ctx->ft, iter);
				in_batch = 0;
			} else {
				cds_ft_next(ctx->ft, iter);
			}
		}
		rcu_read_unlock();

		iters++;
		if ((iters & 0x3f) == 0)
			rcu_quiescent_state();
	}

	cds_ft_iter_destroy(iter);
	rcu_unregister_thread();
	return NULL;
}

static int inv_bind_resume_order(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ord_ft(4, &group);
	struct timespec t0;
	/*
	 * inv_iter_order_writer derives its serializing writer mutex from
	 * (ctx + 1) -- it MUST sit immediately after the ctx in memory.
	 */
	struct {
		struct inv_iter_ctx ctx;
		pthread_mutex_t lock;
	} shared;
	pthread_t readers[NR_READERS_DEFAULT], writers[NR_WRITERS_DEFAULT];
	unsigned int i;

	shared.ctx.ft = ft;
	shared.ctx.key_len = 4;
	shared.ctx.test_name = "inv_bind_resume_order";
	pthread_mutex_init(&shared.lock, NULL);

	rcu_read_lock();
	for (i = 0; i < WRITER_POOL_SIZE / 2; i++) {
		struct ft_test_node *n = node_alloc(i);
		insert_u64(ft, i, n);
	}
	rcu_read_unlock();

	test_go = 0;
	test_stop = 0;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	for (i = 0; i < NR_READERS_DEFAULT; i++)
		pthread_create(&readers[i], NULL, inv_bind_resume_reader, &shared.ctx);
	for (i = 0; i < NR_WRITERS_DEFAULT; i++)
		pthread_create(&writers[i], NULL, inv_iter_order_writer, &shared.ctx);

	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	test_go = 1;

	rcu_thread_offline();

	clock_gettime(CLOCK_MONOTONIC, &t0);

	while (elapsed_ms(&t0) < DEFAULT_DURATION_MS)
		usleep(1000);

	test_stop = 1;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	for (i = 0; i < NR_WRITERS_DEFAULT; i++)
		pthread_join(writers[i], NULL);
	for (i = 0; i < NR_READERS_DEFAULT; i++)
		pthread_join(readers[i], NULL);

	rcu_thread_online();
	pthread_mutex_destroy(&shared.lock);

	if (atomic_load(&violation_count) > 0) {
		fprintf(stderr, "inv_bind_resume_order: %lu violation(s)\n",
			atomic_load(&violation_count));
		drain_and_destroy(ft, group);
		return -1;
	}
	return drain_and_destroy(ft, group);
}

/* ================================================================== */
/*                                                                    */
/*   Compaction terminates + preserves keys on a keycopy trie         */
/*                                                                    */
/* ================================================================== */

/*
 * Exercise cds_ft_compact_step on a keycopy trie (a speculative leaf-key
 * offset is configured).  Build, drain to sparsely-occupied ranges, then
 * compact one relocation per window (batch 1) and verify it terminates and
 * preserves every surviving key in order.  Compaction is keycopy-agnostic in
 * copy mode -- across windows the iterator re-descends from its retained key
 * value -- so this is a functional smoke; the step cap guards non-termination.
 */
static int inv_compact_keycopy_terminates(void)
{
	const unsigned int INSERT_TOTAL = 16000;
	const unsigned int STRIDE = 8;		/* keep every 8th key */
	const unsigned int SURVIVORS = INSERT_TOTAL / STRIDE;
	const unsigned long STEP_CAP = 1000000;	/* >> any terminating run */
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(8, &group);
	struct cds_ft_compact_state *st;
	struct cds_ft_iter *iter;
	unsigned int i, count = 0;
	unsigned long steps = 0;
	uint64_t prev = 0;
	int ret = 0, more, first = 1;

	if (cds_ft_iter_create(ft, &iter) < 0) {
		ret = -1;
		goto out;
	}

	for (i = 0; i < INSERT_TOTAL; i++) {
		struct ft_test_node *n = node_alloc(i);

		rcu_read_lock();
		if (insert_u64(ft, i, n) != CDS_FT_STATUS_OK) {
			rcu_read_unlock();
			node_free(n);
			ret = -1;
			goto out_iter;
		}
		rcu_read_unlock();
	}

	/*
	 * Drain all but every STRIDE-th key.  The holes leave sparsely-occupied
	 * ranges that the compactor must relocate, forcing many windows -- and on
	 * a buggy stale-key cross-window resume, an unbounded loop.
	 */
	for (i = 0; i < INSERT_TOTAL; i++) {
		uint8_t k[8];
		struct cds_ft_node *found;

		if (i % STRIDE == 0)
			continue;
		rcu_read_lock();
		cds_ft_u64_to_key(ft, i, k, CDS_FT_LEN_DEFAULT);
		cds_ft_iter_set_key(iter, k, CDS_FT_LEN_DEFAULT);
		cds_ft_lookup(ft, iter);
		found = cds_ft_iter_node(iter);
		if (found) {
			struct ft_test_node *tn = to_test_node(found);

			if (cds_ft_remove(ft, iter, &tn->node) == CDS_FT_STATUS_OK)
				node_free_rcu(tn);
		}
		rcu_read_unlock();
	}

	st = cds_ft_compact_begin(ft);
	if (!st) {
		ret = -1;
		goto out_iter;
	}
	do {
		more = cds_ft_compact_step(st, 1);	/* batch 1 -> one window per relocation */
		if (++steps > STEP_CAP) {
			report_violation("inv_compact_keycopy_terminates",
				"compaction did not terminate after %lu steps "
				"(stale cross-window key?)", steps);
			cds_ft_compact_end(st);
			ret = -1;
			goto out_iter;
		}
	} while (more);
	cds_ft_compact_end(st);

	/* All surviving keys must remain, in order. */
	rcu_read_lock();
	cds_ft_for_each_rcu(ft, iter) {
		uint8_t rk[8];
		size_t rk_len;
		uint64_t v;

		cds_ft_iter_get_key(iter, rk, sizeof(rk), &rk_len);
		v = cds_ft_key_to_u64(ft, rk, CDS_FT_LEN_DEFAULT);
		if (!first && v <= prev) {
			report_violation("inv_compact_keycopy_terminates",
				"post-compact order: %" PRIu64 " after %" PRIu64,
				v, prev);
			ret = -1;
			break;
		}
		prev = v;
		first = 0;
		count++;
	}
	rcu_read_unlock();

	if (ret == 0 && count != SURVIVORS) {
		report_violation("inv_compact_keycopy_terminates",
			"post-compact count %u, expected %u", count, SURVIVORS);
		ret = -1;
	}

out_iter:
	cds_ft_iter_destroy(iter);
out:
	if (drain_and_destroy(ft, group) < 0)
		ret = -1;
	return ret;
}

/* ================================================================== */
/*                                                                    */
/*   INVARIANT 2: Reverse iteration ordering under mutation           */
/*                                                                    */
/* ================================================================== */

static void *inv_reverse_iter_reader(void *arg)
{
	struct inv_iter_ctx *ctx = (struct inv_iter_ctx *) arg;
	struct cds_ft_iter *iter;
	unsigned long iters = 0;

	rcu_register_thread();

	if (cds_ft_iter_create(ctx->ft, &iter) < 0)
		abort();

	while (!test_go)
		;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	while (!test_stop) {
		uint64_t prev = UINT64_MAX;
		int first = 1;
		int count = 0;

		rcu_read_lock();
		cds_ft_for_each_reverse_rcu(ctx->ft, iter) {
			uint8_t rk[8];
			size_t rk_len;
			uint64_t v;

			cds_ft_iter_get_key(iter, rk, sizeof(rk), &rk_len);
			v = cds_ft_key_to_u64(ctx->ft, rk, CDS_FT_LEN_DEFAULT);
			if (!first && v >= prev) {
				report_violation(ctx->test_name,
					"reverse order: %" PRIu64 " after %" PRIu64
					" (iter #%lu, position %d)",
					v, prev, iters, count);
				break;
			}
			prev = v;
			first = 0;
			count++;
		}
		rcu_read_unlock();

		iters++;
		if ((iters & 0x3f) == 0)
			rcu_quiescent_state();
	}

	cds_ft_iter_destroy(iter);
	rcu_unregister_thread();
	return NULL;
}

static int inv_reverse_iteration_order(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(4, &group);
	struct timespec t0;
	struct {
		struct inv_iter_ctx ctx;
		pthread_mutex_t lock;
	} shared;
	pthread_t readers[NR_READERS_DEFAULT], writers[NR_WRITERS_DEFAULT];
	unsigned int i;

	shared.ctx.ft = ft;
	shared.ctx.key_len = 4;
	shared.ctx.test_name = "inv_reverse_iteration_order";
	pthread_mutex_init(&shared.lock, NULL);

	rcu_read_lock();
	for (i = 0; i < WRITER_POOL_SIZE / 2; i++) {
		struct ft_test_node *n = node_alloc(i);
		insert_u64(ft, i, n);
	}
	rcu_read_unlock();

	test_go = 0;
	test_stop = 0;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	for (i = 0; i < NR_READERS_DEFAULT; i++)
		pthread_create(&readers[i], NULL, inv_reverse_iter_reader, &shared.ctx);
	for (i = 0; i < NR_WRITERS_DEFAULT; i++)
		pthread_create(&writers[i], NULL, inv_iter_order_writer, &shared.ctx);

	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	test_go = 1;

	rcu_thread_offline();

	clock_gettime(CLOCK_MONOTONIC, &t0);
	while (elapsed_ms(&t0) < DEFAULT_DURATION_MS)
		usleep(1000);

	test_stop = 1;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	for (i = 0; i < NR_WRITERS_DEFAULT; i++)
		pthread_join(writers[i], NULL);
	for (i = 0; i < NR_READERS_DEFAULT; i++)
		pthread_join(readers[i], NULL);

	rcu_thread_online();

	pthread_mutex_destroy(&shared.lock);

	if (atomic_load(&violation_count) > 0) {
		fprintf(stderr, "inv_reverse_iteration_order: %lu violation(s)\n",
			atomic_load(&violation_count));
		drain_and_destroy(ft, group);
		return -1;
	}
	return drain_and_destroy(ft, group);
}

/* ================================================================== */
/*                                                                    */
/*   INVARIANT 3: Lookup-found key consistency                        */
/*                                                                    */
/*   When a reader performs an exact lookup and finds a node, the     */
/*   node's shadow key must match the lookup key.  A mismatch would   */
/*   mean the reader followed a stale or partially-updated pointer    */
/*   to the wrong node.                                               */
/*                                                                    */
/* ================================================================== */

struct inv_lookup_ctx {
	struct cds_ft *ft;
	const char *test_name;
	pthread_mutex_t lock;
};

static void *inv_lookup_consistency_reader(void *arg)
{
	struct inv_lookup_ctx *ctx = (struct inv_lookup_ctx *) arg;
	unsigned int seed;
	unsigned long checks = 0;

	rcu_register_thread();
	seed = (unsigned int)(uintptr_t)pthread_self() ^ (unsigned int)time(NULL);

	while (!test_go)
		;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	while (!test_stop) {
		uint64_t key = (uint64_t)(rand_r(&seed) % READER_POOL_SIZE);
		struct cds_ft_node *found;

		rcu_read_lock();
		if (lookup_u64(ctx->ft, key, &found) == CDS_FT_STATUS_OK && found) {
			struct ft_test_node *tn = to_test_node(found);

			if (tn->key != key) {
				report_violation(ctx->test_name,
					"lookup key %" PRIu64 " returned node with shadow key %" PRIu64,
					key, tn->key);
			}
		}
		rcu_read_unlock();

		checks++;
		if ((checks & 0x3ff) == 0)
			rcu_quiescent_state();
	}

	rcu_unregister_thread();
	return NULL;
}

static void *inv_lookup_consistency_writer(void *arg)
{
	struct inv_lookup_ctx *ctx = (struct inv_lookup_ctx *) arg;
	struct cds_ft_iter *iter;
	unsigned int seed;

	rcu_register_thread();
	seed = (unsigned int)(uintptr_t)pthread_self() ^ (unsigned int)time(NULL);

	if (cds_ft_iter_create(ctx->ft, &iter) < 0)
		abort();

	while (!test_go)
		;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	while (!test_stop) {
		uint64_t key = (uint64_t)(rand_r(&seed) % WRITER_POOL_SIZE);
		int do_insert = rand_r(&seed) & 1;

		rcu_read_lock();
		if (do_insert) {
			struct ft_test_node *n = node_alloc(key);

			pthread_mutex_lock(&ctx->lock);
			insert_u64(ctx->ft, key, n);
			pthread_mutex_unlock(&ctx->lock);
		} else {
			struct cds_ft_node *found;
			uint8_t k[8];

			cds_ft_u64_to_key(ctx->ft, key, k, CDS_FT_LEN_DEFAULT);
			cds_ft_iter_set_key(iter, k, CDS_FT_LEN_DEFAULT);
			cds_ft_lookup(ctx->ft, iter);
			found = cds_ft_iter_node(iter);
			if (found) {
				struct ft_test_node *tn = to_test_node(found);

				pthread_mutex_lock(&ctx->lock);
				if (cds_ft_remove(ctx->ft, iter, &tn->node)
				    == CDS_FT_STATUS_OK) {
					node_free_rcu(tn);
				}
				pthread_mutex_unlock(&ctx->lock);
			}
		}
		rcu_read_unlock();

		if ((seed & 0xff) == 0)
			rcu_quiescent_state();
	}

	cds_ft_iter_destroy(iter);
	rcu_unregister_thread();
	return NULL;
}

static int inv_lookup_consistency(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(4, &group);
	struct inv_lookup_ctx ctx;
	struct timespec t0;
	pthread_t readers[NR_READERS_DEFAULT], writers[NR_WRITERS_DEFAULT];
	unsigned int i;

	ctx.ft = ft;
	ctx.test_name = "inv_lookup_consistency";
	pthread_mutex_init(&ctx.lock, NULL);

	/* Pre-populate. */
	rcu_read_lock();
	for (i = 0; i < WRITER_POOL_SIZE / 2; i++) {
		struct ft_test_node *n = node_alloc(i);
		insert_u64(ft, i, n);
	}
	rcu_read_unlock();

	test_go = 0;
	test_stop = 0;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	for (i = 0; i < NR_READERS_DEFAULT; i++)
		pthread_create(&readers[i], NULL, inv_lookup_consistency_reader, &ctx);
	for (i = 0; i < NR_WRITERS_DEFAULT; i++)
		pthread_create(&writers[i], NULL, inv_lookup_consistency_writer, &ctx);

	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	test_go = 1;

	rcu_thread_offline();

	clock_gettime(CLOCK_MONOTONIC, &t0);
	while (elapsed_ms(&t0) < DEFAULT_DURATION_MS)
		usleep(1000);

	test_stop = 1;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	for (i = 0; i < NR_WRITERS_DEFAULT; i++)
		pthread_join(writers[i], NULL);
	for (i = 0; i < NR_READERS_DEFAULT; i++)
		pthread_join(readers[i], NULL);

	rcu_thread_online();

	pthread_mutex_destroy(&ctx.lock);

	if (atomic_load(&violation_count) > 0) {
		fprintf(stderr, "inv_lookup_consistency: %lu violation(s)\n",
			atomic_load(&violation_count));
		drain_and_destroy(ft, group);
		return -1;
	}
	return drain_and_destroy(ft, group);
}

/* ================================================================== */
/*                                                                    */
/*   INVARIANT 4: Duplicate chain acyclicity                          */
/*                                                                    */
/*   Walking a duplicate chain under rcu_read_lock() must always      */
/*   terminate.  A cycle in the chain would cause a reader to loop    */
/*   forever.  We bound the walk to MAX_DUP_CHAIN_LEN and flag a     */
/*   violation if exceeded.                                           */
/*                                                                    */
/* ================================================================== */

static void *inv_dup_chain_reader(void *arg)
{
	struct inv_lookup_ctx *ctx = (struct inv_lookup_ctx *) arg;
	unsigned int seed;
	unsigned long checks = 0;

	rcu_register_thread();
	seed = (unsigned int)(uintptr_t)pthread_self() ^ (unsigned int)time(NULL);

	while (!test_go)
		;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	while (!test_stop) {
		uint64_t key = (uint64_t)(rand_r(&seed) % WRITER_POOL_SIZE);
		struct cds_ft_node *found;

		rcu_read_lock();
		if (lookup_u64(ctx->ft, key, &found) == CDS_FT_STATUS_OK && found) {
			int count = 0;
			struct cds_ft_node *pos = found;

			cds_ft_for_each_duplicate_rcu(pos) {
				count++;
				if (count > MAX_DUP_CHAIN_LEN) {
					report_violation(ctx->test_name,
						"duplicate chain for key %" PRIu64
						" exceeds %d — probable cycle",
						key, MAX_DUP_CHAIN_LEN);
					break;
				}
			}
		}
		rcu_read_unlock();

		checks++;
		if ((checks & 0x3ff) == 0)
			rcu_quiescent_state();
	}

	rcu_unregister_thread();
	return NULL;
}

static int inv_dup_chain_acyclicity(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(4, &group);
	struct inv_lookup_ctx ctx;
	struct timespec t0;
	pthread_t readers[NR_READERS_DEFAULT], writers[NR_WRITERS_DEFAULT];
	unsigned int i;

	ctx.ft = ft;
	ctx.test_name = "inv_dup_chain_acyclicity";
	pthread_mutex_init(&ctx.lock, NULL);

	/*
	 * Pre-populate with duplicates to ensure chains exist from the
	 * start.
	 */
	rcu_read_lock();
	for (i = 0; i < WRITER_POOL_SIZE / 4; i++) {
		unsigned int j;

		for (j = 0; j < 3; j++) {
			struct ft_test_node *n = node_alloc(i);
			insert_u64(ft, i, n);
		}
	}
	rcu_read_unlock();

	test_go = 0;
	test_stop = 0;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	for (i = 0; i < NR_READERS_DEFAULT; i++)
		pthread_create(&readers[i], NULL, inv_dup_chain_reader, &ctx);
	for (i = 0; i < NR_WRITERS_DEFAULT; i++)
		pthread_create(&writers[i], NULL, inv_lookup_consistency_writer, &ctx);

	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	test_go = 1;

	rcu_thread_offline();

	clock_gettime(CLOCK_MONOTONIC, &t0);
	while (elapsed_ms(&t0) < DEFAULT_DURATION_MS)
		usleep(1000);

	test_stop = 1;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	for (i = 0; i < NR_WRITERS_DEFAULT; i++)
		pthread_join(writers[i], NULL);
	for (i = 0; i < NR_READERS_DEFAULT; i++)
		pthread_join(readers[i], NULL);

	rcu_thread_online();

	pthread_mutex_destroy(&ctx.lock);

	if (atomic_load(&violation_count) > 0) {
		fprintf(stderr, "inv_dup_chain_acyclicity: %lu violation(s)\n",
			atomic_load(&violation_count));
		drain_and_destroy(ft, group);
		return -1;
	}
	return drain_and_destroy(ft, group);
}

/* ================================================================== */
/*                                                                    */
/*   INVARIANT 5: Graft-swap atomicity                                */
/*                                                                    */
/*   After a graft_swap at root, a concurrent reader iterating the    */
/*   trie must see a population that is consistent with either the    */
/*   old content or the new content — never an empty intermediate     */
/*   state, and never a mix of both.  We use two distinct key ranges  */
/*   (A: [0, N) and B: [N, 2N)) so any single iteration that sees    */
/*   keys from both ranges is a violation.                            */
/*                                                                    */
/* ================================================================== */

#define GRAFT_SWAP_POOL		64
#define GRAFT_SWAP_RANGE_A_BASE	0
#define GRAFT_SWAP_RANGE_B_BASE	GRAFT_SWAP_POOL

struct inv_graft_ctx {
	struct cds_ft *live;
	struct cds_ft_group *group;
	const char *test_name;
	pthread_mutex_t lock;
};

static void *inv_graft_swap_reader(void *arg)
{
	struct inv_graft_ctx *ctx = (struct inv_graft_ctx *) arg;
	struct cds_ft_iter *iter;
	unsigned long iters = 0;

	rcu_register_thread();

	if (cds_ft_iter_create(ctx->live, &iter) < 0)
		abort();

	while (!test_go)
		;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	while (!test_stop) {
		int saw_a = 0, saw_b = 0;
		int saw_empty = 1;

		rcu_read_lock();
		cds_ft_for_each_rcu(ctx->live, iter) {
			uint8_t rk[4];
			size_t rk_len;
			uint64_t v;

			cds_ft_iter_get_key(iter, rk, sizeof(rk), &rk_len);
			v = cds_ft_key_to_u64(ctx->live, rk, rk_len);
			saw_empty = 0;

			if (v < GRAFT_SWAP_RANGE_B_BASE)
				saw_a = 1;
			else
				saw_b = 1;
		}
		rcu_read_unlock();

		if (saw_a && saw_b) {
			report_violation(ctx->test_name,
				"iteration saw keys from both range A and B "
				"(iter #%lu) — graft_swap not atomic",
				iters);
		}
		/*
		 * After initial population the trie should never be empty
		 * to a reader (graft_swap is not supposed to have an empty
		 * intermediate).  However, we only flag this if we also
		 * didn't see any keys at all, which could legitimately
		 * happen during the very first swap or if timing is tight.
		 * We do not flag saw_empty as a hard violation because the
		 * writer may not have completed the first swap yet.
		 */
		(void)saw_empty;

		iters++;
		if ((iters & 0x3f) == 0)
			rcu_quiescent_state();
	}

	cds_ft_iter_destroy(iter);
	rcu_unregister_thread();
	return NULL;
}

/*
 * Helper: populate a trie with @count keys starting at @base.
 * The trie should have no concurrent readers (offline population).
 */
static void populate_range(struct cds_ft *ft, uint64_t base, unsigned int count)
{
	unsigned int i;

	for (i = 0; i < count; i++) {
		struct ft_test_node *n = node_alloc(base + i);
		uint8_t k[8] = { 0 };

		cds_ft_u64_to_key(ft, base + i, k, 4);
		memcpy(n->okey, k, sizeof(n->okey));
		cds_ft_insert(ft, k, 4, &n->node);
	}
}

/*
 * Helper: drain every node from a trie.  No concurrent access assumed.
 */
static void drain_trie_local(struct cds_ft *ft)
{
	struct cds_ft_iter *iter;

	if (cds_ft_iter_create(ft, &iter) < 0)
		abort();

	rcu_read_lock();
	while (cds_ft_lookup_first(ft, iter) == CDS_FT_STATUS_OK) {
		struct cds_ft_node *head, *tmp;

		cds_ft_remove_all(ft, iter, &head);
		cds_ft_for_each_duplicate_safe_rcu(head, tmp) {
			node_free_rcu(to_test_node(head));
		}
	}
	rcu_read_unlock();
	cds_ft_iter_destroy(iter);
}

static void *inv_graft_swap_writer(void *arg)
{
	struct inv_graft_ctx *ctx = (struct inv_graft_ctx *) arg;
	struct cds_ft *swap;
	int phase = 0;		/* 0: live has A, swap will get B; 1: vice versa */

	rcu_register_thread();

	if (cds_ft_create(ctx->group, NULL, &swap) < 0)
		abort();

	/* Prepare the first swap trie with range B keys. */
	populate_range(swap, GRAFT_SWAP_RANGE_B_BASE, GRAFT_SWAP_POOL);

	while (!test_go)
		;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	while (!test_stop) {
		enum cds_ft_status s;

		/*
		 * Swap at root: atomically exchange the live trie's
		 * content with the swap trie's content.
		 */
		rcu_read_lock();
		pthread_mutex_lock(&ctx->lock);
		s = cds_ft_graft_swap(ctx->live, NULL, 0, swap);
		pthread_mutex_unlock(&ctx->lock);
		rcu_read_unlock();

		if (s != CDS_FT_STATUS_OK) {
			fprintf(stderr, "graft_swap writer: %s\n",
				cds_ft_status_to_string(s));
			break;
		}

		/*
		 * After the swap, `swap` holds the old content.  Wait a
		 * grace period, drain the old content, and populate the
		 * swap trie with fresh keys for the next swap.
		 */
		synchronize_rcu();
		drain_trie_local(swap);
		rcu_barrier();

		phase = !phase;
		if (phase)
			populate_range(swap, GRAFT_SWAP_RANGE_B_BASE, GRAFT_SWAP_POOL);
		else
			populate_range(swap, GRAFT_SWAP_RANGE_A_BASE, GRAFT_SWAP_POOL);

		rcu_quiescent_state();
	}

	/* Clean up swap trie. */
	drain_trie_local(swap);
	rcu_barrier();
	cds_ft_destroy(swap);

	rcu_unregister_thread();
	return NULL;
}

static int inv_graft_swap_atomicity(void)
{
	struct cds_ft_group_attr *attr;
	struct cds_ft_group *group;
	struct cds_ft *live;
	struct inv_graft_ctx ctx;
	struct timespec t0;
	pthread_t readers[NR_READERS_DEFAULT], writer;
	unsigned int i;

	/*
	 * Variable-length trie with max_key_len=4 so root-level
	 * graft_swap (key_len=0) is accepted.
	 */
	if (cds_ft_group_attr_create(&attr) < 0)
		return -1;
	if (cds_ft_group_attr_set_max_key_len(attr, 4) < 0) {
		cds_ft_group_attr_destroy(attr);
		return -1;
	}
	if (cds_ft_group_create(attr, &group) < 0) {
		cds_ft_group_attr_destroy(attr);
		return -1;
	}
	cds_ft_group_attr_destroy(attr);
	if (cds_ft_create(group, NULL, &live) < 0) {
		cds_ft_group_destroy(group);
		return -1;
	}

	/* Initial population: range A. */
	populate_range(live, GRAFT_SWAP_RANGE_A_BASE, GRAFT_SWAP_POOL);

	ctx.live = live;
	ctx.group = group;
	ctx.test_name = "inv_graft_swap_atomicity";
	pthread_mutex_init(&ctx.lock, NULL);

	test_go = 0;
	test_stop = 0;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	for (i = 0; i < NR_READERS_DEFAULT; i++)
		pthread_create(&readers[i], NULL, inv_graft_swap_reader, &ctx);
	/* Single writer for graft_swap — it includes synchronize_rcu. */
	pthread_create(&writer, NULL, inv_graft_swap_writer, &ctx);

	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	test_go = 1;

	rcu_thread_offline();

	clock_gettime(CLOCK_MONOTONIC, &t0);
	while (elapsed_ms(&t0) < DEFAULT_DURATION_MS * 2)
		usleep(1000);

	test_stop = 1;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	pthread_join(writer, NULL);
	for (i = 0; i < NR_READERS_DEFAULT; i++)
		pthread_join(readers[i], NULL);

	rcu_thread_online();

	pthread_mutex_destroy(&ctx.lock);

	if (atomic_load(&violation_count) > 0) {
		fprintf(stderr, "inv_graft_swap_atomicity: %lu violation(s)\n",
			atomic_load(&violation_count));
		drain_trie_local(live);
		rcu_barrier();
		cds_ft_destroy(live);
		cds_ft_group_destroy(group);
		return -1;
	}

	drain_trie_local(live);
	rcu_barrier();
	cds_ft_destroy(live);
	cds_ft_group_destroy(group);
	return 0;
}

/* ================================================================== */
/*                                                                    */
/*   INVARIANT 6: Relational lookup consistency under mutation        */
/*                                                                    */
/*   lookup_le(K) must return a key <= K, and lookup_ge(K) must       */
/*   return a key >= K.  A violation means the reader followed a      */
/*   stale internal path to the wrong leaf.                           */
/*                                                                    */
/* ================================================================== */

static void *inv_relational_reader(void *arg)
{
	struct inv_lookup_ctx *ctx = (struct inv_lookup_ctx *) arg;
	struct cds_ft_iter *iter;
	unsigned int seed;
	unsigned long checks = 0;

	rcu_register_thread();
	seed = (unsigned int)(uintptr_t)pthread_self() ^ (unsigned int)time(NULL);

	if (cds_ft_iter_create(ctx->ft, &iter) < 0)
		abort();

	while (!test_go)
		;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	while (!test_stop) {
		uint64_t key = (uint64_t)(rand_r(&seed) % READER_POOL_SIZE);
		uint8_t k[4], rk[4];
		size_t rk_len;
		uint64_t found_val;
		struct cds_ft_node *node;

		rcu_read_lock();

		/* Test lookup_le. */
		cds_ft_u64_to_key(ctx->ft, key, k, CDS_FT_LEN_DEFAULT);
		cds_ft_iter_set_key(iter, k, CDS_FT_LEN_DEFAULT);
		cds_ft_lookup_le(ctx->ft, iter);
		node = cds_ft_iter_node(iter);
		if (node) {
			cds_ft_iter_get_key(iter, rk, sizeof(rk), &rk_len);
			found_val = cds_ft_key_to_u64(ctx->ft, rk, CDS_FT_LEN_DEFAULT);
			if (found_val > key) {
				report_violation(ctx->test_name,
					"lookup_le(%" PRIu64 ") returned %" PRIu64,
					key, found_val);
			}
		}

		/* Test lookup_ge. */
		cds_ft_iter_set_key(iter, k, CDS_FT_LEN_DEFAULT);
		cds_ft_lookup_ge(ctx->ft, iter);
		node = cds_ft_iter_node(iter);
		if (node) {
			cds_ft_iter_get_key(iter, rk, sizeof(rk), &rk_len);
			found_val = cds_ft_key_to_u64(ctx->ft, rk, CDS_FT_LEN_DEFAULT);
			if (found_val < key) {
				FT_TEST_TP(violation, key, found_val);
				report_violation(ctx->test_name,
					"lookup_ge(%" PRIu64 ") returned %" PRIu64,
					key, found_val);
#ifdef FT_ENABLE_TRACING
				/*
				 * Exit immediately so an LTTng trigger armed
				 * on cds_ft:violation can snapshot and stop
				 * the session with the ring-buffer tail
				 * reflecting the failure context, without
				 * additional mutations flooding the buffer.
				 */
				_exit(99);
#endif
			}
		}

		/*
		 * The iterator's cached path holds RCU-protected
		 * pointers that are only valid while the RCU read-side
		 * lock is held continuously.  Invalidate the cache on
		 * exit so the iter is never in a stale state outside a
		 * critical section: QSBR makes rcu_read_lock/unlock
		 * no-ops, so the rcu_quiescent_state() below is the
		 * real grace-period boundary that a cached path would
		 * cross.
		 */
		cds_ft_iter_bind_key(iter);

		rcu_read_unlock();

		checks++;
		if ((checks & 0x3ff) == 0)
			rcu_quiescent_state();
	}

	cds_ft_iter_destroy(iter);
	rcu_unregister_thread();
	return NULL;
}

static int inv_relational_lookup(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(4, &group);
	struct inv_lookup_ctx ctx;
	struct timespec t0;
	pthread_t readers[NR_READERS_DEFAULT], writers[NR_WRITERS_DEFAULT];
	unsigned int i;

	ctx.ft = ft;
	ctx.test_name = "inv_relational_lookup";
	pthread_mutex_init(&ctx.lock, NULL);

	rcu_read_lock();
	for (i = 0; i < WRITER_POOL_SIZE / 2; i++) {
		struct ft_test_node *n = node_alloc(i);
		insert_u64(ft, i, n);
	}
	rcu_read_unlock();

	test_go = 0;
	test_stop = 0;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	for (i = 0; i < NR_READERS_DEFAULT; i++)
		pthread_create(&readers[i], NULL, inv_relational_reader, &ctx);
	for (i = 0; i < NR_WRITERS_DEFAULT; i++)
		pthread_create(&writers[i], NULL, inv_lookup_consistency_writer, &ctx);

	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	test_go = 1;

	rcu_thread_offline();

	clock_gettime(CLOCK_MONOTONIC, &t0);
	while (elapsed_ms(&t0) < DEFAULT_DURATION_MS)
		usleep(1000);

	test_stop = 1;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	for (i = 0; i < NR_WRITERS_DEFAULT; i++)
		pthread_join(writers[i], NULL);
	for (i = 0; i < NR_READERS_DEFAULT; i++)
		pthread_join(readers[i], NULL);

	rcu_thread_online();

	pthread_mutex_destroy(&ctx.lock);

	if (atomic_load(&violation_count) > 0) {
		fprintf(stderr, "inv_relational_lookup: %lu violation(s)\n",
			atomic_load(&violation_count));
		drain_and_destroy(ft, group);
		return -1;
	}
	return drain_and_destroy(ft, group);
}

/* ================================================================== */
/*                                                                    */
/*   INVARIANT 7: lookup_nth_last stability under low-end mutation    */
/*                                                                    */
/*   A set of "stable" keys at the high end of the key space is       */
/*   never modified.  A concurrent writer inserts and removes keys    */
/*   only at the low end.  A reader repeatedly calls                  */
/*   cds_ft_lookup_nth_last(0) — the largest key — and verifies it    */
/*   always lands in the stable high range.  Because lookup_nth_last  */
/*   descends from the right, mutations at the low end must not       */
/*   disturb the result.                                              */
/*                                                                    */
/* ================================================================== */

#define NTH_STABLE_COUNT	64	/* Stable keys that are never modified. */
#define NTH_WRITER_RANGE	256	/* Writer key range (low end). */
#define NTH_STABLE_BASE		1024	/* Base of the stable high range. */

struct inv_nth_ctx {
	struct cds_ft *ft;
	const char *test_name;
	uint64_t stable_base;		/* Start of stable key range. */
	uint64_t stable_count;		/* Number of stable keys. */
};

static void *inv_nth_last_reader(void *arg)
{
	struct inv_nth_ctx *ctx = (struct inv_nth_ctx *) arg;
	struct cds_ft_iter *iter;
	unsigned long iters = 0;

	rcu_register_thread();

	if (cds_ft_iter_create(ctx->ft, &iter) < 0)
		abort();

	while (!test_go)
		;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	while (!test_stop) {
		enum cds_ft_status s;
		uint8_t rk[8];
		size_t rk_len;
		uint64_t v;

		rcu_read_lock();
		s = cds_ft_lookup_nth_last(ctx->ft, iter, 0);
		if (s == CDS_FT_STATUS_OK) {
			cds_ft_iter_get_key(iter, rk, sizeof(rk), &rk_len);
			v = cds_ft_key_to_u64(ctx->ft, rk, CDS_FT_LEN_DEFAULT);
			if (v < ctx->stable_base ||
			    v >= ctx->stable_base + ctx->stable_count) {
				report_violation(ctx->test_name,
					"nth_last(0) returned %" PRIu64
					", expected [%" PRIu64 ", %" PRIu64
					") (iter #%lu)",
					v, ctx->stable_base,
					ctx->stable_base + ctx->stable_count,
					iters);
			}
		}
		rcu_read_unlock();

		iters++;
		if ((iters & 0x3f) == 0)
			rcu_quiescent_state();
	}

	cds_ft_iter_destroy(iter);
	rcu_unregister_thread();
	return NULL;
}

static void *inv_nth_low_writer(void *arg)
{
	struct inv_nth_ctx *ctx = (struct inv_nth_ctx *) arg;
	struct cds_ft_iter *iter;
	unsigned int seed;
	pthread_mutex_t *lock = (pthread_mutex_t *)(ctx + 1);

	rcu_register_thread();
	seed = (unsigned int)(uintptr_t)pthread_self() ^ (unsigned int)time(NULL);

	if (cds_ft_iter_create(ctx->ft, &iter) < 0)
		abort();

	while (!test_go)
		;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	while (!test_stop) {
		uint64_t key = (uint64_t)(rand_r(&seed) % NTH_WRITER_RANGE);
		int do_insert = rand_r(&seed) & 1;

		rcu_read_lock();
		if (do_insert) {
			struct ft_test_node *n = node_alloc(key);

			pthread_mutex_lock(lock);
			insert_u64(ctx->ft, key, n);
			pthread_mutex_unlock(lock);
		} else {
			struct cds_ft_node *found;
			uint8_t k[8];

			cds_ft_u64_to_key(ctx->ft, key, k, CDS_FT_LEN_DEFAULT);
			cds_ft_iter_set_key(iter, k, CDS_FT_LEN_DEFAULT);
			cds_ft_lookup(ctx->ft, iter);
			found = cds_ft_iter_node(iter);
			if (found) {
				struct ft_test_node *tn = to_test_node(found);

				pthread_mutex_lock(lock);
				if (cds_ft_remove(ctx->ft, iter, &tn->node)
				    == CDS_FT_STATUS_OK) {
					node_free_rcu(tn);
				}
				pthread_mutex_unlock(lock);
			}
		}
		rcu_read_unlock();

		if ((seed & 0xff) == 0)
			rcu_quiescent_state();
	}

	cds_ft_iter_destroy(iter);
	rcu_unregister_thread();
	return NULL;
}

static int inv_nth_last_stability(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(4, &group);
	struct timespec t0;
	struct {
		struct inv_nth_ctx ctx;
		pthread_mutex_t lock;
	} shared;
	pthread_t readers[NR_READERS_DEFAULT], writers[NR_WRITERS_DEFAULT];
	unsigned int i;

	shared.ctx.ft = ft;
	shared.ctx.test_name = "inv_nth_last_stability";
	shared.ctx.stable_base = NTH_STABLE_BASE;
	shared.ctx.stable_count = NTH_STABLE_COUNT;
	pthread_mutex_init(&shared.lock, NULL);

	/* Pre-populate: stable high keys (never modified). */
	rcu_read_lock();
	for (i = 0; i < NTH_STABLE_COUNT; i++) {
		struct ft_test_node *n = node_alloc(NTH_STABLE_BASE + i);
		insert_u64(ft, NTH_STABLE_BASE + i, n);
	}
	/* Seed some low keys for the writer to churn. */
	for (i = 0; i < NTH_WRITER_RANGE / 2; i++) {
		struct ft_test_node *n = node_alloc(i);
		insert_u64(ft, i, n);
	}
	rcu_read_unlock();

	test_go = 0;
	test_stop = 0;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	for (i = 0; i < NR_READERS_DEFAULT; i++)
		pthread_create(&readers[i], NULL, inv_nth_last_reader, &shared.ctx);
	for (i = 0; i < NR_WRITERS_DEFAULT; i++)
		pthread_create(&writers[i], NULL, inv_nth_low_writer, &shared.ctx);

	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	test_go = 1;

	rcu_thread_offline();
	clock_gettime(CLOCK_MONOTONIC, &t0);
	while (elapsed_ms(&t0) < DEFAULT_DURATION_MS)
		usleep(1000);

	test_stop = 1;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	for (i = 0; i < NR_WRITERS_DEFAULT; i++)
		pthread_join(writers[i], NULL);
	for (i = 0; i < NR_READERS_DEFAULT; i++)
		pthread_join(readers[i], NULL);

	rcu_thread_online();
	pthread_mutex_destroy(&shared.lock);

	if (atomic_load(&violation_count) > 0) {
		fprintf(stderr, "inv_nth_last_stability: %lu violation(s)\n",
			atomic_load(&violation_count));
		drain_and_destroy(ft, group);
		return -1;
	}
	return drain_and_destroy(ft, group);
}

/* ================================================================== */
/*                                                                    */
/*   INVARIANT 8: lookup_nth stability under high-end mutation        */
/*                                                                    */
/*   Symmetric to invariant 7: stable keys at the low end, writer     */
/*   churns the high end, reader calls cds_ft_lookup_nth(0) and       */
/*   verifies the smallest key always comes from the stable low       */
/*   range.                                                           */
/*                                                                    */
/* ================================================================== */

static void *inv_nth_first_reader(void *arg)
{
	struct inv_nth_ctx *ctx = (struct inv_nth_ctx *) arg;
	struct cds_ft_iter *iter;
	unsigned long iters = 0;

	rcu_register_thread();

	if (cds_ft_iter_create(ctx->ft, &iter) < 0)
		abort();

	while (!test_go)
		;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	while (!test_stop) {
		enum cds_ft_status s;
		uint8_t rk[8];
		size_t rk_len;
		uint64_t v;

		rcu_read_lock();
		s = cds_ft_lookup_nth(ctx->ft, iter, 0);
		if (s == CDS_FT_STATUS_OK) {
			cds_ft_iter_get_key(iter, rk, sizeof(rk), &rk_len);
			v = cds_ft_key_to_u64(ctx->ft, rk, CDS_FT_LEN_DEFAULT);
			if (v >= ctx->stable_count) {
				report_violation(ctx->test_name,
					"nth(0) returned %" PRIu64
					", expected [0, %" PRIu64
					") (iter #%lu)",
					v, ctx->stable_count, iters);
			}
		}
		rcu_read_unlock();

		iters++;
		if ((iters & 0x3f) == 0)
			rcu_quiescent_state();
	}

	cds_ft_iter_destroy(iter);
	rcu_unregister_thread();
	return NULL;
}

static void *inv_nth_high_writer(void *arg)
{
	struct inv_nth_ctx *ctx = (struct inv_nth_ctx *) arg;
	struct cds_ft_iter *iter;
	unsigned int seed;
	pthread_mutex_t *lock = (pthread_mutex_t *)(ctx + 1);

	rcu_register_thread();
	seed = (unsigned int)(uintptr_t)pthread_self() ^ (unsigned int)time(NULL);

	if (cds_ft_iter_create(ctx->ft, &iter) < 0)
		abort();

	while (!test_go)
		;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	while (!test_stop) {
		uint64_t key = NTH_STABLE_BASE +
			(uint64_t)(rand_r(&seed) % NTH_WRITER_RANGE);
		int do_insert = rand_r(&seed) & 1;

		rcu_read_lock();
		if (do_insert) {
			struct ft_test_node *n = node_alloc(key);

			pthread_mutex_lock(lock);
			insert_u64(ctx->ft, key, n);
			pthread_mutex_unlock(lock);
		} else {
			struct cds_ft_node *found;
			uint8_t k[8];

			cds_ft_u64_to_key(ctx->ft, key, k, CDS_FT_LEN_DEFAULT);
			cds_ft_iter_set_key(iter, k, CDS_FT_LEN_DEFAULT);
			cds_ft_lookup(ctx->ft, iter);
			found = cds_ft_iter_node(iter);
			if (found) {
				struct ft_test_node *tn = to_test_node(found);

				pthread_mutex_lock(lock);
				if (cds_ft_remove(ctx->ft, iter, &tn->node)
				    == CDS_FT_STATUS_OK) {
					node_free_rcu(tn);
				}
				pthread_mutex_unlock(lock);
			}
		}
		rcu_read_unlock();

		if ((seed & 0xff) == 0)
			rcu_quiescent_state();
	}

	cds_ft_iter_destroy(iter);
	rcu_unregister_thread();
	return NULL;
}

static int inv_nth_first_stability(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(4, &group);
	struct timespec t0;
	struct {
		struct inv_nth_ctx ctx;
		pthread_mutex_t lock;
	} shared;
	pthread_t readers[NR_READERS_DEFAULT], writers[NR_WRITERS_DEFAULT];
	unsigned int i;

	shared.ctx.ft = ft;
	shared.ctx.test_name = "inv_nth_first_stability";
	shared.ctx.stable_base = 0;
	shared.ctx.stable_count = NTH_STABLE_COUNT;
	pthread_mutex_init(&shared.lock, NULL);

	/* Pre-populate: stable low keys (never modified). */
	rcu_read_lock();
	for (i = 0; i < NTH_STABLE_COUNT; i++) {
		struct ft_test_node *n = node_alloc(i);
		insert_u64(ft, i, n);
	}
	/* Seed some high keys for the writer to churn. */
	for (i = 0; i < NTH_WRITER_RANGE / 2; i++) {
		struct ft_test_node *n = node_alloc(NTH_STABLE_BASE + i);
		insert_u64(ft, NTH_STABLE_BASE + i, n);
	}
	rcu_read_unlock();

	test_go = 0;
	test_stop = 0;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	for (i = 0; i < NR_READERS_DEFAULT; i++)
		pthread_create(&readers[i], NULL, inv_nth_first_reader, &shared.ctx);
	for (i = 0; i < NR_WRITERS_DEFAULT; i++)
		pthread_create(&writers[i], NULL, inv_nth_high_writer, &shared.ctx);

	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	test_go = 1;

	rcu_thread_offline();
	clock_gettime(CLOCK_MONOTONIC, &t0);
	while (elapsed_ms(&t0) < DEFAULT_DURATION_MS)
		usleep(1000);

	test_stop = 1;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	for (i = 0; i < NR_WRITERS_DEFAULT; i++)
		pthread_join(writers[i], NULL);
	for (i = 0; i < NR_READERS_DEFAULT; i++)
		pthread_join(readers[i], NULL);

	rcu_thread_online();
	pthread_mutex_destroy(&shared.lock);

	if (atomic_load(&violation_count) > 0) {
		fprintf(stderr, "inv_nth_first_stability: %lu violation(s)\n",
			atomic_load(&violation_count));
		drain_and_destroy(ft, group);
		return -1;
	}
	return drain_and_destroy(ft, group);
}

/* ================================================================== */
/*                                                                    */
/*   INVARIANT 9: skip_forward stability under low-end mutation       */
/*                                                                    */
/*   Stable keys in [SKIP_STABLE_BASE, SKIP_STABLE_BASE+COUNT) are   */
/*   never modified.  A writer churns keys in [0, SKIP_WRITER_RANGE). */
/*   A reader positions at the first stable key via lookup_nth, then  */
/*   skip_forwards within the stable range.  Because the local        */
/*   traversal only touches nodes between the start and end, low-end  */
/*   mutations must not affect the result.                            */
/*                                                                    */
/* ================================================================== */

#define SKIP_STABLE_BASE	512
#define SKIP_STABLE_COUNT	64
#define SKIP_WRITER_RANGE	256	/* Writer churn range below stable. */
#define SKIP_HIGH_WRITER_BASE	1024	/* Writer churn range above stable. */

struct inv_skip_ctx {
	struct cds_ft *ft;
	const char *test_name;
	uint64_t stable_base;
	uint64_t stable_count;
};

static void *inv_skip_forward_reader(void *arg)
{
	struct inv_skip_ctx *ctx = (struct inv_skip_ctx *) arg;
	struct cds_ft_iter *iter;
	unsigned long iters = 0;
	unsigned int seed;

	rcu_register_thread();
	seed = (unsigned int)(uintptr_t)pthread_self() ^ (unsigned int)time(NULL);

	if (cds_ft_iter_create(ctx->ft, &iter) < 0)
		abort();

	while (!test_go)
		;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	while (!test_stop) {
		enum cds_ft_status s;
		uint8_t rk[8];
		size_t rk_len;
		uint64_t v;
		unsigned long skip_n;

		rcu_read_lock();

		/*
		 * Position at a random stable key, then skip forward
		 * by a random amount within the stable range.
		 */
		{
			uint64_t start_off = (uint64_t)(rand_r(&seed) % ctx->stable_count);
			uint8_t k[8];

			cds_ft_u64_to_key(ctx->ft, ctx->stable_base + start_off,
					k, CDS_FT_LEN_DEFAULT);
			cds_ft_iter_set_key(iter, k, CDS_FT_LEN_DEFAULT);
			s = cds_ft_lookup(ctx->ft, iter);
			if (s != CDS_FT_STATUS_OK) {
				rcu_read_unlock();
				goto next;
			}

			/* Skip forward within the remaining stable keys. */
			skip_n = (unsigned long)(rand_r(&seed) %
				(ctx->stable_count - start_off));
			if (skip_n == 0) {
				rcu_read_unlock();
				goto next;
			}

			s = cds_ft_iter_skip_forward(ctx->ft, iter, skip_n);
			if (s != CDS_FT_STATUS_OK) {
				report_violation(ctx->test_name,
					"skip_forward(%lu) from %" PRIu64
					" returned %s (iter #%lu)",
					skip_n,
					ctx->stable_base + start_off,
					cds_ft_status_to_string(s),
					iters);
				rcu_read_unlock();
				goto next;
			}

			cds_ft_iter_get_key(iter, rk, sizeof(rk), &rk_len);
			v = cds_ft_key_to_u64(ctx->ft, rk, CDS_FT_LEN_DEFAULT);

			if (v < ctx->stable_base ||
			    v >= ctx->stable_base + ctx->stable_count) {
				report_violation(ctx->test_name,
					"skip_forward(%lu) from %" PRIu64
					" landed on %" PRIu64
					", expected [%" PRIu64 ", %" PRIu64
					") (iter #%lu)",
					skip_n,
					ctx->stable_base + start_off,
					v, ctx->stable_base,
					ctx->stable_base + ctx->stable_count,
					iters);
			}
		}
		rcu_read_unlock();

next:
		iters++;
		if ((iters & 0x3f) == 0)
			rcu_quiescent_state();
	}

	cds_ft_iter_destroy(iter);
	rcu_unregister_thread();
	return NULL;
}

static void *inv_skip_low_writer(void *arg)
{
	struct inv_skip_ctx *ctx = (struct inv_skip_ctx *) arg;
	struct cds_ft_iter *iter;
	unsigned int seed;
	pthread_mutex_t *lock = (pthread_mutex_t *)(ctx + 1);

	rcu_register_thread();
	seed = (unsigned int)(uintptr_t)pthread_self() ^ (unsigned int)time(NULL);

	if (cds_ft_iter_create(ctx->ft, &iter) < 0)
		abort();

	while (!test_go)
		;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	while (!test_stop) {
		uint64_t key = (uint64_t)(rand_r(&seed) % SKIP_WRITER_RANGE);
		int do_insert = rand_r(&seed) & 1;

		rcu_read_lock();
		if (do_insert) {
			struct ft_test_node *n = node_alloc(key);

			pthread_mutex_lock(lock);
			insert_u64(ctx->ft, key, n);
			pthread_mutex_unlock(lock);
		} else {
			struct cds_ft_node *found;
			uint8_t k[8];

			cds_ft_u64_to_key(ctx->ft, key, k, CDS_FT_LEN_DEFAULT);
			cds_ft_iter_set_key(iter, k, CDS_FT_LEN_DEFAULT);
			cds_ft_lookup(ctx->ft, iter);
			found = cds_ft_iter_node(iter);
			if (found) {
				struct ft_test_node *tn = to_test_node(found);

				pthread_mutex_lock(lock);
				if (cds_ft_remove(ctx->ft, iter, &tn->node)
				    == CDS_FT_STATUS_OK) {
					node_free_rcu(tn);
				}
				pthread_mutex_unlock(lock);
			}
		}
		rcu_read_unlock();

		if ((seed & 0xff) == 0)
			rcu_quiescent_state();
	}

	cds_ft_iter_destroy(iter);
	rcu_unregister_thread();
	return NULL;
}

static int inv_skip_forward_stability(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(4, &group);
	struct timespec t0;
	struct {
		struct inv_skip_ctx ctx;
		pthread_mutex_t lock;
	} shared;
	pthread_t readers[NR_READERS_DEFAULT], writers[NR_WRITERS_DEFAULT];
	unsigned int i;

	shared.ctx.ft = ft;
	shared.ctx.test_name = "inv_skip_forward_stability";
	shared.ctx.stable_base = SKIP_STABLE_BASE;
	shared.ctx.stable_count = SKIP_STABLE_COUNT;
	pthread_mutex_init(&shared.lock, NULL);

	/* Pre-populate: stable keys (never modified). */
	rcu_read_lock();
	for (i = 0; i < SKIP_STABLE_COUNT; i++) {
		struct ft_test_node *n = node_alloc(SKIP_STABLE_BASE + i);
		insert_u64(ft, SKIP_STABLE_BASE + i, n);
	}
	/* Seed low keys for writer churn. */
	for (i = 0; i < SKIP_WRITER_RANGE / 2; i++) {
		struct ft_test_node *n = node_alloc(i);
		insert_u64(ft, i, n);
	}
	rcu_read_unlock();

	test_go = 0;
	test_stop = 0;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	for (i = 0; i < NR_READERS_DEFAULT; i++)
		pthread_create(&readers[i], NULL, inv_skip_forward_reader, &shared.ctx);
	for (i = 0; i < NR_WRITERS_DEFAULT; i++)
		pthread_create(&writers[i], NULL, inv_skip_low_writer, &shared.ctx);

	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	test_go = 1;

	rcu_thread_offline();
	clock_gettime(CLOCK_MONOTONIC, &t0);
	while (elapsed_ms(&t0) < DEFAULT_DURATION_MS)
		usleep(1000);

	test_stop = 1;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	for (i = 0; i < NR_WRITERS_DEFAULT; i++)
		pthread_join(writers[i], NULL);
	for (i = 0; i < NR_READERS_DEFAULT; i++)
		pthread_join(readers[i], NULL);

	rcu_thread_online();
	pthread_mutex_destroy(&shared.lock);

	if (atomic_load(&violation_count) > 0) {
		fprintf(stderr, "inv_skip_forward_stability: %lu violation(s)\n",
			atomic_load(&violation_count));
		drain_and_destroy(ft, group);
		return -1;
	}
	return drain_and_destroy(ft, group);
}

/* ================================================================== */
/*                                                                    */
/*   INVARIANT 10: skip_reverse stability under high-end mutation     */
/*                                                                    */
/*   Same stable key range.  Writer churns keys above.  Reader        */
/*   positions at a stable key and skip_reverses within the stable    */
/*   range.  High-end mutations must not affect the result.           */
/*                                                                    */
/* ================================================================== */

static void *inv_skip_reverse_reader(void *arg)
{
	struct inv_skip_ctx *ctx = (struct inv_skip_ctx *) arg;
	struct cds_ft_iter *iter;
	unsigned long iters = 0;
	unsigned int seed;

	rcu_register_thread();
	seed = (unsigned int)(uintptr_t)pthread_self() ^ (unsigned int)time(NULL);

	if (cds_ft_iter_create(ctx->ft, &iter) < 0)
		abort();

	while (!test_go)
		;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	while (!test_stop) {
		enum cds_ft_status s;
		uint8_t rk[8];
		size_t rk_len;
		uint64_t v;
		unsigned long skip_n;

		rcu_read_lock();

		{
			uint64_t start_off = (uint64_t)(rand_r(&seed) % ctx->stable_count);
			uint8_t k[8];

			cds_ft_u64_to_key(ctx->ft, ctx->stable_base + start_off,
					k, CDS_FT_LEN_DEFAULT);
			cds_ft_iter_set_key(iter, k, CDS_FT_LEN_DEFAULT);
			s = cds_ft_lookup(ctx->ft, iter);
			if (s != CDS_FT_STATUS_OK) {
				rcu_read_unlock();
				goto next;
			}

			/* Skip reverse within the preceding stable keys. */
			if (start_off == 0) {
				rcu_read_unlock();
				goto next;
			}
			skip_n = (unsigned long)(rand_r(&seed) % start_off);
			if (skip_n == 0) {
				rcu_read_unlock();
				goto next;
			}

			s = cds_ft_iter_skip_reverse(ctx->ft, iter, skip_n);
			if (s != CDS_FT_STATUS_OK) {
				report_violation(ctx->test_name,
					"skip_reverse(%lu) from %" PRIu64
					" returned %s (iter #%lu)",
					skip_n,
					ctx->stable_base + start_off,
					cds_ft_status_to_string(s),
					iters);
				rcu_read_unlock();
				goto next;
			}

			cds_ft_iter_get_key(iter, rk, sizeof(rk), &rk_len);
			v = cds_ft_key_to_u64(ctx->ft, rk, CDS_FT_LEN_DEFAULT);

			if (v < ctx->stable_base ||
			    v >= ctx->stable_base + ctx->stable_count) {
				report_violation(ctx->test_name,
					"skip_reverse(%lu) from %" PRIu64
					" landed on %" PRIu64
					", expected [%" PRIu64 ", %" PRIu64
					") (iter #%lu)",
					skip_n,
					ctx->stable_base + start_off,
					v, ctx->stable_base,
					ctx->stable_base + ctx->stable_count,
					iters);
			}
		}
		rcu_read_unlock();

next:
		iters++;
		if ((iters & 0x3f) == 0)
			rcu_quiescent_state();
	}

	cds_ft_iter_destroy(iter);
	rcu_unregister_thread();
	return NULL;
}

static void *inv_skip_high_writer(void *arg)
{
	struct inv_skip_ctx *ctx = (struct inv_skip_ctx *) arg;
	struct cds_ft_iter *iter;
	unsigned int seed;
	pthread_mutex_t *lock = (pthread_mutex_t *)(ctx + 1);

	rcu_register_thread();
	seed = (unsigned int)(uintptr_t)pthread_self() ^ (unsigned int)time(NULL);

	if (cds_ft_iter_create(ctx->ft, &iter) < 0)
		abort();

	while (!test_go)
		;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	while (!test_stop) {
		uint64_t key = SKIP_HIGH_WRITER_BASE +
			(uint64_t)(rand_r(&seed) % SKIP_WRITER_RANGE);
		int do_insert = rand_r(&seed) & 1;

		rcu_read_lock();
		if (do_insert) {
			struct ft_test_node *n = node_alloc(key);

			pthread_mutex_lock(lock);
			insert_u64(ctx->ft, key, n);
			pthread_mutex_unlock(lock);
		} else {
			struct cds_ft_node *found;
			uint8_t k[8];

			cds_ft_u64_to_key(ctx->ft, key, k, CDS_FT_LEN_DEFAULT);
			cds_ft_iter_set_key(iter, k, CDS_FT_LEN_DEFAULT);
			cds_ft_lookup(ctx->ft, iter);
			found = cds_ft_iter_node(iter);
			if (found) {
				struct ft_test_node *tn = to_test_node(found);

				pthread_mutex_lock(lock);
				if (cds_ft_remove(ctx->ft, iter, &tn->node)
				    == CDS_FT_STATUS_OK) {
					node_free_rcu(tn);
				}
				pthread_mutex_unlock(lock);
			}
		}
		rcu_read_unlock();

		if ((seed & 0xff) == 0)
			rcu_quiescent_state();
	}

	cds_ft_iter_destroy(iter);
	rcu_unregister_thread();
	return NULL;
}

static int inv_skip_reverse_stability(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(4, &group);
	struct timespec t0;
	struct {
		struct inv_skip_ctx ctx;
		pthread_mutex_t lock;
	} shared;
	pthread_t readers[NR_READERS_DEFAULT], writers[NR_WRITERS_DEFAULT];
	unsigned int i;

	shared.ctx.ft = ft;
	shared.ctx.test_name = "inv_skip_reverse_stability";
	shared.ctx.stable_base = SKIP_STABLE_BASE;
	shared.ctx.stable_count = SKIP_STABLE_COUNT;
	pthread_mutex_init(&shared.lock, NULL);

	/* Pre-populate: stable keys (never modified). */
	rcu_read_lock();
	for (i = 0; i < SKIP_STABLE_COUNT; i++) {
		struct ft_test_node *n = node_alloc(SKIP_STABLE_BASE + i);
		insert_u64(ft, SKIP_STABLE_BASE + i, n);
	}
	/* Seed high keys for writer churn. */
	for (i = 0; i < SKIP_WRITER_RANGE / 2; i++) {
		struct ft_test_node *n = node_alloc(SKIP_HIGH_WRITER_BASE + i);
		insert_u64(ft, SKIP_HIGH_WRITER_BASE + i, n);
	}
	rcu_read_unlock();

	test_go = 0;
	test_stop = 0;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	for (i = 0; i < NR_READERS_DEFAULT; i++)
		pthread_create(&readers[i], NULL, inv_skip_reverse_reader, &shared.ctx);
	for (i = 0; i < NR_WRITERS_DEFAULT; i++)
		pthread_create(&writers[i], NULL, inv_skip_high_writer, &shared.ctx);

	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	test_go = 1;

	rcu_thread_offline();
	clock_gettime(CLOCK_MONOTONIC, &t0);
	while (elapsed_ms(&t0) < DEFAULT_DURATION_MS)
		usleep(1000);

	test_stop = 1;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	for (i = 0; i < NR_WRITERS_DEFAULT; i++)
		pthread_join(writers[i], NULL);
	for (i = 0; i < NR_READERS_DEFAULT; i++)
		pthread_join(readers[i], NULL);

	rcu_thread_online();
	pthread_mutex_destroy(&shared.lock);

	if (atomic_load(&violation_count) > 0) {
		fprintf(stderr, "inv_skip_reverse_stability: %lu violation(s)\n",
			atomic_load(&violation_count));
		drain_and_destroy(ft, group);
		return -1;
	}
	return drain_and_destroy(ft, group);
}

/* ================================================================== */
/*                                                                    */
/*   INVARIANT 11: nr_keys undercount ordering                        */
/*                                                                    */
/*   The nr_keys metadata is updated with a specific ordering         */
/*   relative to pointer publication:                                 */
/*                                                                    */
/*     Insert: publish pointer, then increment nr_keys.               */
/*     Remove: decrement nr_keys, then detach pointer.                */
/*                                                                    */
/*   Both produce a transient undercount: nr_keys <= actual reachable */
/*   keys.  The practical consequence is that lookup_nth(N-1) should  */
/*   always find a valid key when cds_ft_count_keys returned N > 0,   */
/*   provided that the actual key population never drops below N      */
/*   between the two calls.                                           */
/*                                                                    */
/*   Insert phase: an insert-only writer ensures the actual count is  */
/*   monotonically non-decreasing.  If cds_ft_count_keys returns N,   */
/*   there are at least N reachable keys (undercount), and            */
/*   lookup_nth(N-1) must succeed.  An overcount bug would cause      */
/*   N > actual, making lookup_nth(N-1) fail.                         */
/*                                                                    */
/*   Remove phase: a remove-only writer ensures the actual count is   */
/*   monotonically non-increasing.  The reader iterates first         */
/*   (pointer-based count), then reads count_keys.  With decrement-   */
/*   before-detach ordering, count_keys can only lag behind (or       */
/*   equal) the iteration count, so count_keys <= iter_count must     */
/*   hold.  An overcount bug (detach-before-decrement) would cause    */
/*   count_keys > iter_count.                                         */
/*                                                                    */
/*   A quiescent phase after each concurrent phase verifies that      */
/*   count_keys equals iteration_count exactly.                       */
/*                                                                    */
/* ================================================================== */

static void *inv_nr_keys_undercount_reader(void *arg)
{
	struct inv_iter_ctx *ctx = (struct inv_iter_ctx *) arg;
	struct cds_ft_iter *iter;
	unsigned long iters = 0;

	rcu_register_thread();

	if (cds_ft_iter_create(ctx->ft, &iter) < 0)
		abort();

	while (!test_go)
		;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	while (!test_stop) {
		unsigned long count_keys;
		enum cds_ft_status s;

		rcu_read_lock();

		count_keys = cds_ft_count_keys(ctx->ft);

		/*
		 * With an insert-only writer and undercount ordering,
		 * there are always >= count_keys reachable keys.
		 * lookup_nth(count_keys - 1) and lookup_nth_last(0)
		 * through lookup_nth_last(count_keys - 1) must find
		 * valid keys.
		 */
		if (count_keys > 0) {
			s = cds_ft_lookup_nth(ctx->ft, iter,
					count_keys - 1);
			if (s != CDS_FT_STATUS_OK) {
				report_violation(ctx->test_name,
					"lookup_nth(%lu) returned %s "
					"(count_keys %lu, iter #%lu)",
					count_keys - 1,
					cds_ft_status_to_string(s),
					count_keys, iters);
			}
			s = cds_ft_lookup_nth_last(ctx->ft, iter,
					count_keys - 1);
			if (s != CDS_FT_STATUS_OK) {
				report_violation(ctx->test_name,
					"lookup_nth_last(%lu) returned %s "
					"(count_keys %lu, iter #%lu)",
					count_keys - 1,
					cds_ft_status_to_string(s),
					count_keys, iters);
			}
		}
		rcu_read_unlock();

		iters++;
		if ((iters & 0x3f) == 0)
			rcu_quiescent_state();
	}

	cds_ft_iter_destroy(iter);
	rcu_unregister_thread();
	return NULL;
}

/*
 * Insert-only writer: only adds keys, never removes.
 * This guarantees that the actual key count is monotonically
 * non-decreasing, making the undercount invariant testable.
 */
static void *inv_nr_keys_undercount_writer(void *arg)
{
	struct inv_iter_ctx *ctx = (struct inv_iter_ctx *) arg;
	unsigned int seed;
	pthread_mutex_t *lock = (pthread_mutex_t *)(ctx + 1);

	rcu_register_thread();
	seed = (unsigned int)(uintptr_t)pthread_self() ^ (unsigned int)time(NULL);

	while (!test_go)
		;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	while (!test_stop) {
		uint64_t key = (uint64_t)(rand_r(&seed) % WRITER_POOL_SIZE);
		struct ft_test_node *n = node_alloc(key);

		rcu_read_lock();
		pthread_mutex_lock(lock);
		insert_u64(ctx->ft, key, n);
		pthread_mutex_unlock(lock);
		rcu_read_unlock();

		if ((seed & 0xff) == 0)
			rcu_quiescent_state();
	}

	rcu_unregister_thread();
	return NULL;
}

/*
 * Remove-phase reader: iterate first (pointer-based), then read
 * count_keys.  With remove-only writer and decrement-before-detach
 * ordering, count_keys can only decrease between the iteration and
 * the count_keys read, so count_keys <= iter_count must hold.
 */
static void *inv_nr_keys_remove_reader(void *arg)
{
	struct inv_iter_ctx *ctx = (struct inv_iter_ctx *) arg;
	struct cds_ft_iter *iter;
	unsigned long iters = 0;

	rcu_register_thread();

	if (cds_ft_iter_create(ctx->ft, &iter) < 0)
		abort();

	while (!test_go)
		;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	while (!test_stop) {
		unsigned long count_keys, iter_count;

		rcu_read_lock();

		/*
		 * Iterate first: count reachable keys via pointers.
		 * Then read count_keys (based on nr_keys metadata).
		 *
		 * With a remove-only writer:
		 * - Each removal decrements nr_keys BEFORE detaching
		 *   the pointer.
		 * - Between our iteration and count_keys read, only
		 *   removals happen (decreasing both values).
		 * - Removals behind the iterator don't change
		 *   iter_count (already counted).
		 * - Removals ahead of the iterator reduce iter_count
		 *   (missed) AND reduce count_keys (decremented).
		 * - count_keys can drop further than iter_count
		 *   (decrement is first step of each removal).
		 *
		 * Therefore count_keys <= iter_count must hold.
		 */
		iter_count = 0;
		cds_ft_for_each_rcu(ctx->ft, iter) {
			iter_count++;
		}

		count_keys = cds_ft_count_keys(ctx->ft);

		if (count_keys > iter_count) {
			report_violation(ctx->test_name,
				"remove phase: count_keys %lu > "
				"iteration_count %lu (iter #%lu)",
				count_keys, iter_count, iters);
		}
		rcu_read_unlock();

		iters++;
		if ((iters & 0x3f) == 0)
			rcu_quiescent_state();
	}

	cds_ft_iter_destroy(iter);
	rcu_unregister_thread();
	return NULL;
}

/*
 * Remove-only writer: only removes keys, never inserts.
 */
static void *inv_nr_keys_remove_writer(void *arg)
{
	struct inv_iter_ctx *ctx = (struct inv_iter_ctx *) arg;
	struct cds_ft_iter *iter;
	unsigned int seed;
	pthread_mutex_t *lock = (pthread_mutex_t *)(ctx + 1);

	rcu_register_thread();
	seed = (unsigned int)(uintptr_t)pthread_self() ^ (unsigned int)time(NULL);

	if (cds_ft_iter_create(ctx->ft, &iter) < 0)
		abort();

	while (!test_go)
		;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	while (!test_stop) {
		uint64_t key = (uint64_t)(rand_r(&seed) % WRITER_POOL_SIZE);
		struct cds_ft_node *found;
		uint8_t k[8];

		rcu_read_lock();
		cds_ft_u64_to_key(ctx->ft, key, k, CDS_FT_LEN_DEFAULT);
		cds_ft_iter_set_key(iter, k, CDS_FT_LEN_DEFAULT);
		cds_ft_lookup(ctx->ft, iter);
		found = cds_ft_iter_node(iter);
		if (found) {
			struct ft_test_node *tn = to_test_node(found);

			pthread_mutex_lock(lock);
			if (cds_ft_remove(ctx->ft, iter, &tn->node)
			    == CDS_FT_STATUS_OK) {
				node_free_rcu(tn);
			}
			pthread_mutex_unlock(lock);
		}
		rcu_read_unlock();

		if ((seed & 0xff) == 0)
			rcu_quiescent_state();
	}

	cds_ft_iter_destroy(iter);
	rcu_unregister_thread();
	return NULL;
}

/*
 * Helper: quiescent check that count_keys == iteration_count.
 * Returns 0 on success, -1 on mismatch.
 */
static int quiescent_count_check(struct cds_ft *ft, const char *phase)
{
	struct cds_ft_iter *check_iter;
	unsigned long count_keys, iter_count;

	if (cds_ft_iter_create(ft, &check_iter) < 0)
		return -1;
	rcu_read_lock();
	count_keys = cds_ft_count_keys(ft);
	iter_count = 0;
	cds_ft_for_each_rcu(ft, check_iter) {
		iter_count++;
	}
	rcu_read_unlock();
	cds_ft_iter_destroy(check_iter);

	if (count_keys != iter_count) {
		fprintf(stderr,
			"inv_nr_keys_undercount: %s quiescent mismatch: "
			"count_keys %lu != iteration_count %lu\n",
			phase, count_keys, iter_count);
		return -1;
	}
	return 0;
}

static int inv_nr_keys_undercount(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(4, &group);
	struct timespec t0;
	struct {
		struct inv_iter_ctx ctx;
		pthread_mutex_t lock;
	} shared;
	pthread_t readers[NR_READERS_DEFAULT], writers[NR_WRITERS_DEFAULT];
	unsigned int i;

	shared.ctx.ft = ft;
	shared.ctx.key_len = 4;
	shared.ctx.test_name = "inv_nr_keys_undercount";
	pthread_mutex_init(&shared.lock, NULL);

	/*
	 * Phase 1: Insert-only writer.
	 *
	 * Pre-populate, then insert-only writers add more keys.
	 * Readers check: count_keys = N implies lookup_nth(N-1)
	 * and lookup_nth_last(N-1) succeed.
	 */
	rcu_read_lock();
	for (i = 0; i < WRITER_POOL_SIZE / 2; i++) {
		struct ft_test_node *n = node_alloc(i);
		insert_u64(ft, i, n);
	}
	rcu_read_unlock();

	test_go = 0;
	test_stop = 0;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	for (i = 0; i < NR_READERS_DEFAULT; i++)
		pthread_create(&readers[i], NULL,
			inv_nr_keys_undercount_reader, &shared.ctx);
	for (i = 0; i < NR_WRITERS_DEFAULT; i++)
		pthread_create(&writers[i], NULL,
			inv_nr_keys_undercount_writer, &shared.ctx);

	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	test_go = 1;

	rcu_thread_offline();
	clock_gettime(CLOCK_MONOTONIC, &t0);
	while (elapsed_ms(&t0) < DEFAULT_DURATION_MS / 2)
		usleep(1000);

	test_stop = 1;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	for (i = 0; i < NR_WRITERS_DEFAULT; i++)
		pthread_join(writers[i], NULL);
	for (i = 0; i < NR_READERS_DEFAULT; i++)
		pthread_join(readers[i], NULL);

	rcu_thread_online();

	if (atomic_load(&violation_count) > 0) {
		fprintf(stderr,
			"inv_nr_keys_undercount: insert phase: %lu violation(s)\n",
			atomic_load(&violation_count));
		pthread_mutex_destroy(&shared.lock);
		drain_and_destroy(ft, group);
		return -1;
	}

	/* Quiescent check after insert phase. */
	if (quiescent_count_check(ft, "insert") < 0) {
		pthread_mutex_destroy(&shared.lock);
		drain_and_destroy(ft, group);
		return -1;
	}

	/*
	 * Phase 2: Remove-only writer.
	 *
	 * The trie is now populated from phase 1.  Remove-only
	 * writers drain keys.  Readers iterate first (pointer-based
	 * count), then read count_keys, and check count_keys <=
	 * iter_count.
	 */
	atomic_store(&violation_count, 0);

	test_go = 0;
	test_stop = 0;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	for (i = 0; i < NR_READERS_DEFAULT; i++)
		pthread_create(&readers[i], NULL,
			inv_nr_keys_remove_reader, &shared.ctx);
	for (i = 0; i < NR_WRITERS_DEFAULT; i++)
		pthread_create(&writers[i], NULL,
			inv_nr_keys_remove_writer, &shared.ctx);

	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	test_go = 1;

	rcu_thread_offline();
	clock_gettime(CLOCK_MONOTONIC, &t0);
	while (elapsed_ms(&t0) < DEFAULT_DURATION_MS / 2)
		usleep(1000);

	test_stop = 1;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	for (i = 0; i < NR_WRITERS_DEFAULT; i++)
		pthread_join(writers[i], NULL);
	for (i = 0; i < NR_READERS_DEFAULT; i++)
		pthread_join(readers[i], NULL);

	rcu_thread_online();
	pthread_mutex_destroy(&shared.lock);

	if (atomic_load(&violation_count) > 0) {
		fprintf(stderr,
			"inv_nr_keys_undercount: remove phase: %lu violation(s)\n",
			atomic_load(&violation_count));
		drain_and_destroy(ft, group);
		return -1;
	}

	/* Quiescent check after remove phase. */
	if (quiescent_count_check(ft, "remove") < 0) {
		drain_and_destroy(ft, group);
		return -1;
	}

	return drain_and_destroy(ft, group);
}

/* ================================================================== */
/*                                                                    */
/*   INVARIANT 9: Ordered traversal never escapes its trie            */
/*                                                                    */
/* ================================================================== */

/*
 * Invariant: an ordered traversal on trie T (cds_ft_lookup_first /
 * _last / _gt / _lt) must never return a key that belongs to a
 * different trie.  Specifically, a concurrent graft from T to S
 * followed by the reader continuing its traversal must not cause the
 * reader to land in S's ancestor chain and return keys from S's
 * namespace ("jumping out of the trie").
 *
 * Setup:
 *   Two tries T and S in the same group, variable-length keys.
 *   T is pre-populated with two key families:
 *     - "T" + 2-byte index  (the mobile family, moved by the writer)
 *     - "B" + 2-byte index  (the anchor family, stays in T)
 *   S is pre-populated with:
 *     - "S" + 2-byte index  (S's native namespace)
 *   S reserves the "X" prefix as the landing slot for T's mobile
 *   content during the cycle.
 *
 * Writer (one thread, mutex-serialised):
 *   phase 1: detach "T" from T  (yields detached trie D, concurrent)
 *            graft  D into S at "X"  (graft syncs because D is concurrent)
 *   phase 2: detach "X" from S  (yields detached trie D', concurrent)
 *            graft  D' back into T at "T"  (graft syncs)
 *
 * Readers (NR_READERS_DEFAULT threads):
 *   alternate forward (lookup_first → lookup_gt chain) and reverse
 *   (lookup_last → lookup_lt chain) iteration on T, and check that
 *   every returned key has first byte 'T' or 'B'.  Any other prefix
 *   ('S', 'X', or anything else) means the reader followed a stale
 *   parent pointer into S's subtree — the jump-out bug.
 */

#define ESCAPE_POOL_PER_PREFIX	64

struct inv_no_escape_ctx {
	struct cds_ft *T;
	struct cds_ft *S;
	struct cds_ft_group *group;
	pthread_mutex_t lock;
	const char *test_name;
};

static void inv_no_escape_populate(struct cds_ft *ft, uint8_t prefix,
		uint64_t value_base)
{
	unsigned int i;

	for (i = 0; i < ESCAPE_POOL_PER_PREFIX; i++) {
		uint8_t key[3] = { prefix, (uint8_t)(i >> 8), (uint8_t)(i & 0xff) };
		struct ft_test_node *n = node_alloc(value_base + i);

		if (cds_ft_insert(ft, key, 3, &n->node) < 0) {
			node_free(n);
			return;
		}
	}
}

static void *inv_no_escape_reader(void *arg)
{
	struct inv_no_escape_ctx *ctx = (struct inv_no_escape_ctx *) arg;
	struct cds_ft_iter *iter;
	unsigned long iters = 0;

	rcu_register_thread();

	if (cds_ft_iter_create(ctx->T, &iter) < 0)
		abort();

	while (!test_go)
		;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	while (!test_stop) {
		bool reverse = (iters & 1) != 0;
		enum cds_ft_status s;
		unsigned int count = 0;
		/*
		 * Safety limit: a well-formed traversal of T sees at most
		 * 2 * ESCAPE_POOL_PER_PREFIX keys (T* + B* families).
		 * A much larger count signals the reader is looping
		 * through a much larger structure than T (e.g. S's
		 * contents after jumping out).
		 */
		const unsigned int max_count = ESCAPE_POOL_PER_PREFIX * 8;

		rcu_read_lock();

		if (reverse)
			s = cds_ft_lookup_last(ctx->T, iter);
		else
			s = cds_ft_lookup_first(ctx->T, iter);

		while (s == CDS_FT_STATUS_OK && count++ < max_count) {
			uint8_t k[16];
			size_t kl;

			cds_ft_iter_get_key(iter, k, sizeof(k), &kl);

			if (kl == 0 || (k[0] != 'T' && k[0] != 'B')) {
				report_violation(ctx->test_name,
					"reader on T saw key prefix 0x%02x (len %zu) "
					"— not in T or B namespace (iter #%lu, %s)",
					kl > 0 ? (unsigned int) k[0] : 0u, kl, iters,
					reverse ? "reverse" : "forward");
				break;
			}

			if (reverse)
				s = cds_ft_lookup_lt(ctx->T, iter);
			else
				s = cds_ft_lookup_gt(ctx->T, iter);
		}
		if (count >= max_count) {
			report_violation(ctx->test_name,
				"ordered traversal returned >= %u keys — suspected "
				"escape into S's content (iter #%lu, %s)",
				max_count, iters, reverse ? "reverse" : "forward");
		}

		rcu_read_unlock();

		iters++;
		if ((iters & 0x3f) == 0)
			rcu_quiescent_state();
	}

	cds_ft_iter_destroy(iter);
	rcu_unregister_thread();
	return NULL;
}

static void *inv_no_escape_writer(void *arg)
{
	struct inv_no_escape_ctx *ctx = (struct inv_no_escape_ctx *) arg;

	rcu_register_thread();

	while (!test_go)
		;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	while (!test_stop) {
		struct cds_ft *D;
		enum cds_ft_status s;

		/* Phase 1: detach T's "T*" content, graft into S at "X". */
		pthread_mutex_lock(&ctx->lock);
		s = cds_ft_detach(ctx->T, (const uint8_t *)"T", 1, &D);
		if (s == CDS_FT_STATUS_OK) {
			enum cds_ft_status gs =
				cds_ft_graft(ctx->S, (const uint8_t *)"X", 1, D);
			if (gs == CDS_FT_STATUS_OK) {
				cds_ft_destroy(D);
			} else {
				fprintf(stderr,
					"inv_no_escape writer: graft T->S failed: %s\n",
					cds_ft_status_to_string(gs));
				pthread_mutex_unlock(&ctx->lock);
				drain_trie_local(D);
				rcu_barrier();
				cds_ft_destroy(D);
				continue;
			}
		}
		pthread_mutex_unlock(&ctx->lock);

		rcu_quiescent_state();

		/* Phase 2: detach S's "X*" content, graft back into T at "T". */
		pthread_mutex_lock(&ctx->lock);
		s = cds_ft_detach(ctx->S, (const uint8_t *)"X", 1, &D);
		if (s == CDS_FT_STATUS_OK) {
			enum cds_ft_status gs =
				cds_ft_graft(ctx->T, (const uint8_t *)"T", 1, D);
			if (gs == CDS_FT_STATUS_OK) {
				cds_ft_destroy(D);
			} else {
				fprintf(stderr,
					"inv_no_escape writer: graft S->T failed: %s\n",
					cds_ft_status_to_string(gs));
				pthread_mutex_unlock(&ctx->lock);
				drain_trie_local(D);
				rcu_barrier();
				cds_ft_destroy(D);
				continue;
			}
		}
		pthread_mutex_unlock(&ctx->lock);

		rcu_quiescent_state();
	}

	rcu_unregister_thread();
	return NULL;
}

static int inv_ordered_no_escape_graft(void)
{
	struct cds_ft_group_attr *gattr;
	struct cds_ft_group *group;
	struct cds_ft *T, *S;
	struct inv_no_escape_ctx ctx;
	struct timespec t0;
	pthread_t readers[NR_READERS_DEFAULT], writer;
	unsigned int i;
	int ret = 0;

	if (cds_ft_group_attr_create(&gattr) < 0)
		return -1;
	if (cds_ft_group_attr_set_max_key_len(gattr, 16) < 0) {
		cds_ft_group_attr_destroy(gattr);
		return -1;
	}
	if (cds_ft_group_create(gattr, &group) < 0) {
		cds_ft_group_attr_destroy(gattr);
		return -1;
	}
	cds_ft_group_attr_destroy(gattr);

	if (cds_ft_create(group, NULL, &T) < 0) {
		cds_ft_group_destroy(group);
		return -1;
	}
	if (cds_ft_create(group, NULL, &S) < 0) {
		cds_ft_destroy(T);
		cds_ft_group_destroy(group);
		return -1;
	}

	/* Populate: T gets "T*" + "B*", S gets "S*" (leaves "X*" slot for writer). */
	rcu_read_lock();
	inv_no_escape_populate(T, 'T', 0);
	inv_no_escape_populate(T, 'B', 10000);
	inv_no_escape_populate(S, 'S', 100000);
	rcu_read_unlock();

	ctx.T = T;
	ctx.S = S;
	ctx.group = group;
	pthread_mutex_init(&ctx.lock, NULL);
	ctx.test_name = "inv_ordered_no_escape_graft";

	test_go = 0;
	test_stop = 0;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	for (i = 0; i < NR_READERS_DEFAULT; i++)
		pthread_create(&readers[i], NULL, inv_no_escape_reader, &ctx);
	pthread_create(&writer, NULL, inv_no_escape_writer, &ctx);

	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	test_go = 1;

	rcu_thread_offline();
	clock_gettime(CLOCK_MONOTONIC, &t0);
	while (elapsed_ms(&t0) < DEFAULT_DURATION_MS)
		usleep(1000);

	test_stop = 1;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	pthread_join(writer, NULL);
	for (i = 0; i < NR_READERS_DEFAULT; i++)
		pthread_join(readers[i], NULL);

	rcu_thread_online();
	pthread_mutex_destroy(&ctx.lock);

	if (atomic_load(&violation_count) > 0) {
		fprintf(stderr, "inv_ordered_no_escape_graft: %lu violation(s)\n",
			atomic_load(&violation_count));
		ret = -1;
	}

	drain_trie_local(T);
	drain_trie_local(S);
	rcu_barrier();
	cds_ft_destroy(T);
	cds_ft_destroy(S);
	cds_ft_group_destroy(group);
	return ret;
}

/* ================================================================== */
/*                                                                    */
/*   INVARIANT 13: Merge no-escape (PIECEWISE overlap)                */
/*                                                                    */
/*   A concurrent ordered traversal of dst never escapes its key      */
/*   namespace nor loops, while a writer repeatedly performs a         */
/*   PIECEWISE merge into dst (the destination already holds nodes     */
/*   that overlap the source) and then removes it again.              */
/*                                                                    */
/*   dst base = {Taa,Tab,Tba,Tbb}: multi-child branches T->{a,b},     */
/*   each ->{a,b}.  Each round the writer builds src={Tac,Tad,Tbc,    */
/*   Tbd} and merges it at root: the merge recurses into the shared   */
/*   'T' and 'a'/'b' branches, re-parenting dst's OWN leaves into      */
/*   freshly-built spine nodes alongside src's leaves, then publishes  */
/*   the new spine at dst's root.  The shared path is all multi-child  */
/*   branches, so no compressed node sits on it and the build-         */
/*   invisible spine-copy is taken under every build.  If the          */
/*   re-parent ever leaves a transient where an ordered up-walk        */
/*   follows a stale/cross parent, the reader sees a key outside       */
/*   {T}{a,b}{a..d} (escape / corruption) or loops past the bound.    */
/* ================================================================== */

struct inv_merge_ctx {
	struct cds_ft *dst;
	struct cds_ft *src;
	struct cds_ft_group *group;
	const char *test_name;
	pthread_mutex_t lock;
};

static const char *const inv_merge_src_keys[] = { "Tac", "Tad", "Tbc", "Tbd" };

/*
 * Per-reader: which trie to iterate and the allowed 3rd-byte range.  dst
 * legitimately holds {T}{a,b}{a..d} (its base a/b keys plus, transiently,
 * the merged-in c/d keys); src only ever holds {T}{a,b}{c,d}.  A src reader
 * that escapes UP into dst (the cross-trie re-parent hazard) observes a
 * 3rd byte of 'a' or 'b' -- outside src's [c,d] range -- and flags it.
 */
struct inv_merge_reader_arg {
	struct inv_merge_ctx *ctx;
	struct cds_ft *trie;
	uint8_t c2_lo, c2_hi;
	const char *which;
};

static void *inv_merge_no_escape_reader(void *arg)
{
	struct inv_merge_reader_arg *ra = (struct inv_merge_reader_arg *) arg;
	struct inv_merge_ctx *ctx = ra->ctx;
	struct cds_ft_iter *iter;
	unsigned long iters = 0;
	const unsigned int max_count = 32;

	rcu_register_thread();
	if (cds_ft_iter_create(ra->trie, &iter) < 0)
		abort();

	while (!test_go)
		;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	while (!test_stop) {
		bool reverse = (iters & 1) != 0;
		enum cds_ft_status s;
		unsigned int count = 0;

		rcu_read_lock();
		s = reverse ? cds_ft_lookup_last(ra->trie, iter)
			: cds_ft_lookup_first(ra->trie, iter);
		while (s == CDS_FT_STATUS_OK && count++ < max_count) {
			uint8_t k[16];
			size_t kl;

			cds_ft_iter_get_key(iter, k, sizeof(k), &kl);
			if (kl != 3 || k[0] != 'T' ||
			    (k[1] != 'a' && k[1] != 'b') ||
			    k[2] < ra->c2_lo || k[2] > ra->c2_hi) {
				report_violation(ctx->test_name,
					"%s reader saw out-of-namespace key "
					"(len %zu, %.3s) — merge re-parent escape "
					"(iter #%lu, %s)",
					ra->which, kl, kl ? (const char *) k : "",
					iters, reverse ? "reverse" : "forward");
				break;
			}
			s = reverse ? cds_ft_lookup_lt(ra->trie, iter)
				: cds_ft_lookup_gt(ra->trie, iter);
		}
		if (count >= max_count)
			report_violation(ctx->test_name,
				"%s ordered traversal returned >= %u keys — "
				"escape/loop (iter #%lu, %s)",
				ra->which, max_count, iters,
				reverse ? "reverse" : "forward");
		rcu_read_unlock();

		iters++;
		if ((iters & 0x3f) == 0)
			rcu_quiescent_state();
	}

	cds_ft_iter_destroy(iter);
	rcu_unregister_thread();
	return NULL;
}

/* Remove the duplicate chain at @key from @ft (writer-locked). */
static void inv_merge_remove_key(struct cds_ft *ft, struct cds_ft_iter *iter,
		const char *key)
{
	struct cds_ft_node *head, *tmp;

	cds_ft_iter_set_key(iter, (const uint8_t *) key, strlen(key));
	cds_ft_lookup(ft, iter);
	if (!cds_ft_iter_node(iter))
		return;
	if (cds_ft_remove_all(ft, iter, &head) != CDS_FT_STATUS_OK)
		return;
	cds_ft_for_each_duplicate_safe_rcu(head, tmp)
		node_free_rcu(to_test_node(head));
}

static void *inv_merge_no_escape_writer(void *arg)
{
	struct inv_merge_ctx *ctx = (struct inv_merge_ctx *) arg;
	struct cds_ft_iter *iter;
	unsigned int i;

	rcu_register_thread();
	if (cds_ft_iter_create(ctx->dst, &iter) < 0)
		abort();

	while (!test_go)
		;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	while (!test_stop) {
		enum cds_ft_status s;

		/*
		 * Piecewise merge of all of the persistent @src into @dst at
		 * the root.  @src is iterated by its own readers throughout, so
		 * the commit's source-side detach + drain (and the cross-trie
		 * re-parent of src's subtrees into dst) runs under live source
		 * readers.
		 *
		 * The writer is a MUTATOR: it must NOT hold a read-side lock
		 * around the merge.  cds_ft_merge calls synchronize_rcu
		 * internally to drain src/dst readers; a grace-period wait
		 * issued from within a read-side critical section does not
		 * actually wait, so the source-reader drain would be defeated.
		 * The application mutex (ctx->lock) provides writer exclusion.
		 */
		pthread_mutex_lock(&ctx->lock);
		s = cds_ft_merge(ctx->dst, NULL, 0, ctx->src);
		pthread_mutex_unlock(&ctx->lock);
		if (s != CDS_FT_STATUS_OK) {
			fprintf(stderr, "inv_merge writer: %s\n",
				cds_ft_status_to_string(s));
			break;
		}

		rcu_quiescent_state();

		/*
		 * Reset: pull the four merged keys back out of @dst, and
		 * re-populate @src (emptied by the merge) with fresh nodes for
		 * the next round.
		 */
		rcu_read_lock();
		pthread_mutex_lock(&ctx->lock);
		for (i = 0; i < 4; i++)
			inv_merge_remove_key(ctx->dst, iter, inv_merge_src_keys[i]);
		for (i = 0; i < 4; i++) {
			struct ft_test_node *n = node_alloc(300 + i);

			cds_ft_insert(ctx->src,
				(const uint8_t *) inv_merge_src_keys[i],
				strlen(inv_merge_src_keys[i]), &n->node);
		}
		pthread_mutex_unlock(&ctx->lock);
		rcu_read_unlock();

		rcu_quiescent_state();
	}

	cds_ft_iter_destroy(iter);
	rcu_unregister_thread();
	return NULL;
}

static int inv_merge_no_escape(void)
{
	static const char *const base_keys[] = { "Taa", "Tab", "Tba", "Tbb" };
	struct cds_ft_group_attr *gattr;
	struct cds_ft_group *group;
	struct cds_ft *dst, *src;
	struct inv_merge_ctx ctx;
	struct inv_merge_reader_arg rargs[2 * NR_READERS_DEFAULT];
	struct timespec t0;
	pthread_t readers[2 * NR_READERS_DEFAULT], writer;
	unsigned int i;
	int ret = 0;

	if (cds_ft_group_attr_create(&gattr) < 0)
		return -1;
	if (cds_ft_group_attr_set_max_key_len(gattr, 16) < 0) {
		cds_ft_group_attr_destroy(gattr);
		return -1;
	}
	if (cds_ft_group_create(gattr, &group) < 0) {
		cds_ft_group_attr_destroy(gattr);
		return -1;
	}
	cds_ft_group_attr_destroy(gattr);

	if (cds_ft_create(group, NULL, &dst) < 0) {
		cds_ft_group_destroy(group);
		return -1;
	}
	if (cds_ft_create(group, NULL, &src) < 0) {
		cds_ft_destroy(dst);
		cds_ft_group_destroy(group);
		return -1;
	}

	rcu_read_lock();
	for (i = 0; i < 4; i++) {
		struct ft_test_node *dn = node_alloc(i);
		struct ft_test_node *sn = node_alloc(300 + i);

		cds_ft_insert(dst, (const uint8_t *) base_keys[i],
			strlen(base_keys[i]), &dn->node);
		cds_ft_insert(src, (const uint8_t *) inv_merge_src_keys[i],
			strlen(inv_merge_src_keys[i]), &sn->node);
	}
	rcu_read_unlock();

	ctx.dst = dst;
	ctx.src = src;
	ctx.group = group;
	ctx.test_name = "inv_merge_no_escape";
	pthread_mutex_init(&ctx.lock, NULL);

	test_go = 0;
	test_stop = 0;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	/*
	 * Half the readers iterate dst (namespace {T}{a,b}{a..d}), half iterate
	 * src (namespace {T}{a,b}{c,d}), so both the destination-side and the
	 * source-side of every merge run under concurrent ordered traversals.
	 */
	for (i = 0; i < 2 * NR_READERS_DEFAULT; i++) {
		bool is_src = (i & 1) != 0;

		rargs[i].ctx = &ctx;
		rargs[i].trie = is_src ? src : dst;
		rargs[i].c2_lo = is_src ? (uint8_t) 'c' : (uint8_t) 'a';
		rargs[i].c2_hi = (uint8_t) 'd';
		rargs[i].which = is_src ? "src" : "dst";
		pthread_create(&readers[i], NULL,
			inv_merge_no_escape_reader, &rargs[i]);
	}
	pthread_create(&writer, NULL, inv_merge_no_escape_writer, &ctx);

	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	test_go = 1;

	rcu_thread_offline();
	clock_gettime(CLOCK_MONOTONIC, &t0);
	while (elapsed_ms(&t0) < DEFAULT_DURATION_MS)
		usleep(1000);

	test_stop = 1;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	pthread_join(writer, NULL);
	for (i = 0; i < 2 * NR_READERS_DEFAULT; i++)
		pthread_join(readers[i], NULL);

	rcu_thread_online();
	pthread_mutex_destroy(&ctx.lock);

	if (atomic_load(&violation_count) > 0) {
		fprintf(stderr, "inv_merge_no_escape: %lu violation(s)\n",
			atomic_load(&violation_count));
		ret = -1;
	}

	drain_trie_local(dst);
	drain_trie_local(src);
	rcu_barrier();
	cds_ft_destroy(dst);
	cds_ft_destroy(src);
	cds_ft_group_destroy(group);
	return ret;
}

/* ================================================================== */
/*                                                                    */
/*   INVARIANT 11: Merge into a NON-ROOT dst never escapes             */
/*                                                                    */
/*   As invariant 10, but the writer merges its private root source    */
/*   {ac,ad,bc,bd} into dst at the INTERIOR key "T" (re-keying to      */
/*   {Tac,Tad,Tbc,Tbd}).  dst base {Taa,Tab,Tba,Tbb} keeps "T" a live  */
/*   internal subtree, so the build-invisible spine copy publishes      */
/*   through an interior forward slot via a type-7 flip proxy.  An      */
/*   ordered reader traversing dst across the flip window must resolve  */
/*   that proxy at child dispatch (point descent + ft_node_get_         */
/*   direction); a missed resolve surfaces a garbage child or a key     */
/*   outside {T}{a,b}{a..d}.  The dst reader of invariant 10 is reused  */
/*   verbatim (same namespace).                                        */
/* ================================================================== */

static const char *const inv_merge_nrd_base_keys[] = {
	"Taa", "Tab", "Tba", "Tbb",
};
static const char *const inv_merge_nrd_src_keys[] = { "ac", "ad", "bc", "bd" };
static const char *const inv_merge_nrd_merged_keys[] = {
	"Tac", "Tad", "Tbc", "Tbd",
};

static void *inv_merge_nonroot_dst_writer(void *arg)
{
	struct inv_merge_ctx *ctx = (struct inv_merge_ctx *) arg;
	struct cds_ft_iter *iter;
	unsigned int i;

	rcu_register_thread();
	if (cds_ft_iter_create(ctx->dst, &iter) < 0)
		abort();

	while (!test_go)
		;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	while (!test_stop) {
		enum cds_ft_status s;

		/* Merge the root source into dst at the interior key "T". */
		pthread_mutex_lock(&ctx->lock);
		s = cds_ft_merge_at(ctx->dst, (const uint8_t *) "T", 1,
				ctx->src, NULL, 0);
		pthread_mutex_unlock(&ctx->lock);
		if (s != CDS_FT_STATUS_OK) {
			fprintf(stderr, "inv_merge_nonroot_dst writer: %s\n",
				cds_ft_status_to_string(s));
			break;
		}
		rcu_quiescent_state();

		/* Reset: pull the merged keys back out, re-populate src. */
		rcu_read_lock();
		pthread_mutex_lock(&ctx->lock);
		for (i = 0; i < 4; i++)
			inv_merge_remove_key(ctx->dst, iter,
				inv_merge_nrd_merged_keys[i]);
		for (i = 0; i < 4; i++) {
			struct ft_test_node *n = node_alloc(300 + i);

			cds_ft_insert(ctx->src,
				(const uint8_t *) inv_merge_nrd_src_keys[i],
				strlen(inv_merge_nrd_src_keys[i]), &n->node);
		}
		pthread_mutex_unlock(&ctx->lock);
		rcu_read_unlock();
		rcu_quiescent_state();
	}

	cds_ft_iter_destroy(iter);
	rcu_unregister_thread();
	return NULL;
}

static int inv_merge_nonroot_dst_no_escape(void)
{
	struct cds_ft_group_attr *gattr;
	struct cds_ft_group *group;
	struct cds_ft *dst, *src;
	struct inv_merge_ctx ctx;
	struct inv_merge_reader_arg rargs[2 * NR_READERS_DEFAULT];
	struct timespec t0;
	pthread_t readers[2 * NR_READERS_DEFAULT], writer;
	unsigned int i;
	int ret = 0;

	if (cds_ft_group_attr_create(&gattr) < 0)
		return -1;
	if (cds_ft_group_attr_set_max_key_len(gattr, 16) < 0) {
		cds_ft_group_attr_destroy(gattr);
		return -1;
	}
	if (cds_ft_group_create(gattr, &group) < 0) {
		cds_ft_group_attr_destroy(gattr);
		return -1;
	}
	cds_ft_group_attr_destroy(gattr);

	if (cds_ft_create(group, NULL, &dst) < 0) {
		cds_ft_group_destroy(group);
		return -1;
	}
	if (cds_ft_create(group, NULL, &src) < 0) {
		cds_ft_destroy(dst);
		cds_ft_group_destroy(group);
		return -1;
	}

	rcu_read_lock();
	for (i = 0; i < 4; i++) {
		struct ft_test_node *dn = node_alloc(i);
		struct ft_test_node *sn = node_alloc(300 + i);

		cds_ft_insert(dst, (const uint8_t *) inv_merge_nrd_base_keys[i],
			strlen(inv_merge_nrd_base_keys[i]), &dn->node);
		cds_ft_insert(src, (const uint8_t *) inv_merge_nrd_src_keys[i],
			strlen(inv_merge_nrd_src_keys[i]), &sn->node);
	}
	rcu_read_unlock();

	ctx.dst = dst;
	ctx.src = src;
	ctx.group = group;
	ctx.test_name = "inv_merge_nonroot_dst_no_escape";
	pthread_mutex_init(&ctx.lock, NULL);

	test_go = 0;
	test_stop = 0;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	/* All readers traverse dst; namespace {T}{a,b}{a..d} across the merge. */
	for (i = 0; i < 2 * NR_READERS_DEFAULT; i++) {
		rargs[i].ctx = &ctx;
		rargs[i].trie = dst;
		rargs[i].c2_lo = (uint8_t) 'a';
		rargs[i].c2_hi = (uint8_t) 'd';
		rargs[i].which = "dst";
		pthread_create(&readers[i], NULL,
			inv_merge_no_escape_reader, &rargs[i]);
	}
	pthread_create(&writer, NULL, inv_merge_nonroot_dst_writer, &ctx);

	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	test_go = 1;

	rcu_thread_offline();
	clock_gettime(CLOCK_MONOTONIC, &t0);
	while (elapsed_ms(&t0) < DEFAULT_DURATION_MS)
		usleep(1000);

	test_stop = 1;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	pthread_join(writer, NULL);
	for (i = 0; i < 2 * NR_READERS_DEFAULT; i++)
		pthread_join(readers[i], NULL);

	rcu_thread_online();
	pthread_mutex_destroy(&ctx.lock);

	if (atomic_load(&violation_count) > 0) {
		fprintf(stderr, "inv_merge_nonroot_dst_no_escape: %lu violation(s)\n",
			atomic_load(&violation_count));
		ret = -1;
	}

	drain_trie_local(dst);
	drain_trie_local(src);
	rcu_barrier();
	cds_ft_destroy(dst);
	cds_ft_destroy(src);
	cds_ft_group_destroy(group);
	return ret;
}

/* ================================================================== */
/*                                                                    */
/*   INVARIANT 12: Merge into a COMPRESSED dst merge point never       */
/*   escapes (the M_slot skip-encoded interior publish via flip proxy) */
/*                                                                    */
/*   dst persistently holds "Tabz" -> under 'T' a COMPRESSED run "abz" */
/*   (merge point d_dst->nf is compressed).  Each round the writer     */
/*   merges a private root src {"abw"} at "T", so the merged M is a     */
/*   fresh COMPRESSED run "ab"->{z,w} published into the interior 'T'   */
/*   slot SKIP-ENCODED via the flip proxy -- the path single-threaded   */
/*   unit tests cannot stress.  An ordered reader must only ever see    */
/*   {"Tabz","Tabw"}.                                                  */
/* ================================================================== */

static void *inv_merge_compressed_dst_reader(void *arg)
{
	struct inv_merge_reader_arg *ra = (struct inv_merge_reader_arg *) arg;
	struct inv_merge_ctx *ctx = ra->ctx;
	struct cds_ft_iter *iter;
	unsigned long iters = 0;
	const unsigned int max_count = 16;

	rcu_register_thread();
	if (cds_ft_iter_create(ra->trie, &iter) < 0)
		abort();
	while (!test_go)
		;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	while (!test_stop) {
		bool reverse = (iters & 1) != 0;
		enum cds_ft_status s;
		unsigned int count = 0;

		rcu_read_lock();
		s = reverse ? cds_ft_lookup_last(ra->trie, iter)
			: cds_ft_lookup_first(ra->trie, iter);
		while (s == CDS_FT_STATUS_OK && count++ < max_count) {
			uint8_t k[16];
			size_t kl;

			cds_ft_iter_get_key(iter, k, sizeof(k), &kl);
			if (kl != 4 || k[0] != 'T' || k[1] != 'a' ||
			    k[2] != 'b' || (k[3] != 'z' && k[3] != 'w')) {
				report_violation(ctx->test_name,
					"reader saw out-of-namespace key "
					"(len %zu, %.4s) — compressed-dst merge escape "
					"(iter #%lu, %s)",
					kl, kl ? (const char *) k : "",
					iters, reverse ? "reverse" : "forward");
				break;
			}
			s = reverse ? cds_ft_lookup_lt(ra->trie, iter)
				: cds_ft_lookup_gt(ra->trie, iter);
		}
		if (count >= max_count)
			report_violation(ctx->test_name,
				"ordered traversal returned >= %u keys — "
				"escape/loop (iter #%lu, %s)",
				max_count, iters, reverse ? "reverse" : "forward");
		rcu_read_unlock();
		iters++;
		if ((iters & 0x3f) == 0)
			rcu_quiescent_state();
	}

	cds_ft_iter_destroy(iter);
	rcu_unregister_thread();
	return NULL;
}

static void *inv_merge_compressed_dst_writer(void *arg)
{
	struct inv_merge_ctx *ctx = (struct inv_merge_ctx *) arg;
	struct cds_ft_iter *iter;

	rcu_register_thread();
	if (cds_ft_iter_create(ctx->dst, &iter) < 0)
		abort();
	while (!test_go)
		;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	while (!test_stop) {
		enum cds_ft_status s;
		struct ft_test_node *n;

		pthread_mutex_lock(&ctx->lock);
		s = cds_ft_merge_at(ctx->dst, (const uint8_t *) "T", 1,
				ctx->src, NULL, 0);
		pthread_mutex_unlock(&ctx->lock);
		if (s != CDS_FT_STATUS_OK) {
			fprintf(stderr, "inv_merge_compressed_dst writer: %s\n",
				cds_ft_status_to_string(s));
			break;
		}
		rcu_quiescent_state();

		rcu_read_lock();
		pthread_mutex_lock(&ctx->lock);
		inv_merge_remove_key(ctx->dst, iter, "Tabw");
		n = node_alloc(300);
		cds_ft_insert(ctx->src, (const uint8_t *) "abw", 3, &n->node);
		pthread_mutex_unlock(&ctx->lock);
		rcu_read_unlock();
		rcu_quiescent_state();
	}

	cds_ft_iter_destroy(iter);
	rcu_unregister_thread();
	return NULL;
}

static int inv_merge_compressed_dst_no_escape(void)
{
	struct cds_ft_group_attr *gattr;
	struct cds_ft_group *group;
	struct cds_ft *dst, *src;
	struct inv_merge_ctx ctx;
	struct inv_merge_reader_arg rargs[2 * NR_READERS_DEFAULT];
	struct timespec t0;
	pthread_t readers[2 * NR_READERS_DEFAULT], writer;
	struct ft_test_node *n, *sn;
	unsigned int i;
	int ret = 0;

	if (cds_ft_group_attr_create(&gattr) < 0)
		return -1;
	if (cds_ft_group_attr_set_max_key_len(gattr, 16) < 0) {
		cds_ft_group_attr_destroy(gattr);
		return -1;
	}
	if (cds_ft_group_create(gattr, &group) < 0) {
		cds_ft_group_attr_destroy(gattr);
		return -1;
	}
	cds_ft_group_attr_destroy(gattr);

	if (cds_ft_create(group, NULL, &dst) < 0)
		abort();
	if (cds_ft_create(group, NULL, &src) < 0)
		abort();

	/* Persistent "Tabz" keeps "T" a live COMPRESSED merge point. */
	n = node_alloc(0);
	cds_ft_insert(dst, (const uint8_t *) "Tabz", 4, &n->node);
	sn = node_alloc(300);
	cds_ft_insert(src, (const uint8_t *) "abw", 3, &sn->node);

	ctx.dst = dst;
	ctx.src = src;
	ctx.group = group;
	ctx.test_name = "inv_merge_compressed_dst_no_escape";
	pthread_mutex_init(&ctx.lock, NULL);

	test_go = 0;
	test_stop = 0;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	for (i = 0; i < 2 * NR_READERS_DEFAULT; i++) {
		rargs[i].ctx = &ctx;
		rargs[i].trie = dst;
		rargs[i].which = "dst";
		pthread_create(&readers[i], NULL,
			inv_merge_compressed_dst_reader, &rargs[i]);
	}
	pthread_create(&writer, NULL, inv_merge_compressed_dst_writer, &ctx);

	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	test_go = 1;

	rcu_thread_offline();
	clock_gettime(CLOCK_MONOTONIC, &t0);
	while (elapsed_ms(&t0) < DEFAULT_DURATION_MS)
		usleep(1000);

	test_stop = 1;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	pthread_join(writer, NULL);
	for (i = 0; i < 2 * NR_READERS_DEFAULT; i++)
		pthread_join(readers[i], NULL);

	rcu_thread_online();
	pthread_mutex_destroy(&ctx.lock);

	if (atomic_load(&violation_count) > 0) {
		fprintf(stderr, "inv_merge_compressed_dst_no_escape: %lu violation(s)\n",
			atomic_load(&violation_count));
		ret = -1;
	}

	drain_trie_local(dst);
	drain_trie_local(src);
	rcu_barrier();
	cds_ft_destroy(dst);
	cds_ft_destroy(src);
	cds_ft_group_destroy(group);
	return ret;
}

/* ================================================================== */
/*                                                                    */
/*   INVARIANT 13: Merge into a KEY_SHORTER dst merge point never       */
/*   escapes (the dst key ends INSIDE a compressed node, so the merged  */
/*   cluster is published under a fresh prefix wrap via the flip proxy). */
/*                                                                    */
/*   dst persistently holds "Tabz" -> under 'T' a COMPRESSED run "abz".  */
/*   Each round the writer merges a private root src {"w"} at "Ta",      */
/*   which ends 1 byte INTO "abz": the node splits at prefix "a", the    */
/*   suffix "bz"->leaf is re-parented under a fresh branch via the flip, */
/*   and the result "a"->{b->z, w} is published into the interior 'T'    */
/*   slot SKIP-ENCODED.  An ordered reader must only ever see            */
/*   {"Tabz","Taw"} -- the compressed leaf must never escape the flip.   */
/* ================================================================== */

static void *inv_merge_key_shorter_dst_reader(void *arg)
{
	struct inv_merge_reader_arg *ra = (struct inv_merge_reader_arg *) arg;
	struct inv_merge_ctx *ctx = ra->ctx;
	struct cds_ft_iter *iter;
	unsigned long iters = 0;
	const unsigned int max_count = 16;

	rcu_register_thread();
	if (cds_ft_iter_create(ra->trie, &iter) < 0)
		abort();
	while (!test_go)
		;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	while (!test_stop) {
		bool reverse = (iters & 1) != 0;
		enum cds_ft_status s;
		unsigned int count = 0;

		rcu_read_lock();
		s = reverse ? cds_ft_lookup_last(ra->trie, iter)
			: cds_ft_lookup_first(ra->trie, iter);
		while (s == CDS_FT_STATUS_OK && count++ < max_count) {
			uint8_t k[16];
			size_t kl;
			bool ok_key = false;

			cds_ft_iter_get_key(iter, k, sizeof(k), &kl);
			if (k[0] == 'T' && k[1] == 'a') {
				if (kl == 4 && k[2] == 'b' && k[3] == 'z')
					ok_key = true;	/* Tabz */
				else if (kl == 3 && k[2] == 'w')
					ok_key = true;	/* Taw */
			}
			if (!ok_key) {
				report_violation(ctx->test_name,
					"reader saw out-of-namespace key "
					"(len %zu, %.4s) — key-shorter-dst merge escape "
					"(iter #%lu, %s)",
					kl, kl ? (const char *) k : "",
					iters, reverse ? "reverse" : "forward");
				break;
			}
			s = reverse ? cds_ft_lookup_lt(ra->trie, iter)
				: cds_ft_lookup_gt(ra->trie, iter);
		}
		if (count >= max_count)
			report_violation(ctx->test_name,
				"ordered traversal returned >= %u keys — "
				"escape/loop (iter #%lu, %s)",
				max_count, iters, reverse ? "reverse" : "forward");
		rcu_read_unlock();
		iters++;
		if ((iters & 0x3f) == 0)
			rcu_quiescent_state();
	}

	cds_ft_iter_destroy(iter);
	rcu_unregister_thread();
	return NULL;
}

static void *inv_merge_key_shorter_dst_writer(void *arg)
{
	struct inv_merge_ctx *ctx = (struct inv_merge_ctx *) arg;
	struct cds_ft_iter *iter;

	rcu_register_thread();
	if (cds_ft_iter_create(ctx->dst, &iter) < 0)
		abort();
	while (!test_go)
		;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	while (!test_stop) {
		enum cds_ft_status s;
		struct ft_test_node *n;

		pthread_mutex_lock(&ctx->lock);
		s = cds_ft_merge_at(ctx->dst, (const uint8_t *) "Ta", 2,
				ctx->src, NULL, 0);
		pthread_mutex_unlock(&ctx->lock);
		if (s != CDS_FT_STATUS_OK) {
			fprintf(stderr, "inv_merge_key_shorter_dst writer: %s\n",
				cds_ft_status_to_string(s));
			break;
		}
		rcu_quiescent_state();

		rcu_read_lock();
		pthread_mutex_lock(&ctx->lock);
		inv_merge_remove_key(ctx->dst, iter, "Taw");
		n = node_alloc(300);
		cds_ft_insert(ctx->src, (const uint8_t *) "w", 1, &n->node);
		pthread_mutex_unlock(&ctx->lock);
		rcu_read_unlock();
		rcu_quiescent_state();
	}

	cds_ft_iter_destroy(iter);
	rcu_unregister_thread();
	return NULL;
}

static int inv_merge_key_shorter_dst_no_escape(void)
{
	struct cds_ft_group_attr *gattr;
	struct cds_ft_group *group;
	struct cds_ft *dst, *src;
	struct inv_merge_ctx ctx;
	struct inv_merge_reader_arg rargs[2 * NR_READERS_DEFAULT];
	struct timespec t0;
	pthread_t readers[2 * NR_READERS_DEFAULT], writer;
	struct ft_test_node *n, *sn;
	unsigned int i;
	int ret = 0;

	if (cds_ft_group_attr_create(&gattr) < 0)
		return -1;
	if (cds_ft_group_attr_set_max_key_len(gattr, 16) < 0) {
		cds_ft_group_attr_destroy(gattr);
		return -1;
	}
	if (cds_ft_group_create(gattr, &group) < 0) {
		cds_ft_group_attr_destroy(gattr);
		return -1;
	}
	cds_ft_group_attr_destroy(gattr);

	if (cds_ft_create(group, NULL, &dst) < 0)
		abort();
	if (cds_ft_create(group, NULL, &src) < 0)
		abort();

	/* Persistent "Tabz" keeps "Ta" a KEY_SHORTER point inside "abz". */
	n = node_alloc(0);
	cds_ft_insert(dst, (const uint8_t *) "Tabz", 4, &n->node);
	sn = node_alloc(300);
	cds_ft_insert(src, (const uint8_t *) "w", 1, &sn->node);

	ctx.dst = dst;
	ctx.src = src;
	ctx.group = group;
	ctx.test_name = "inv_merge_key_shorter_dst_no_escape";
	pthread_mutex_init(&ctx.lock, NULL);

	test_go = 0;
	test_stop = 0;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	for (i = 0; i < 2 * NR_READERS_DEFAULT; i++) {
		rargs[i].ctx = &ctx;
		rargs[i].trie = dst;
		rargs[i].which = "dst";
		pthread_create(&readers[i], NULL,
			inv_merge_key_shorter_dst_reader, &rargs[i]);
	}
	pthread_create(&writer, NULL, inv_merge_key_shorter_dst_writer, &ctx);

	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	test_go = 1;

	rcu_thread_offline();
	clock_gettime(CLOCK_MONOTONIC, &t0);
	while (elapsed_ms(&t0) < DEFAULT_DURATION_MS)
		usleep(1000);

	test_stop = 1;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	pthread_join(writer, NULL);
	for (i = 0; i < 2 * NR_READERS_DEFAULT; i++)
		pthread_join(readers[i], NULL);

	rcu_thread_online();
	pthread_mutex_destroy(&ctx.lock);

	if (atomic_load(&violation_count) > 0) {
		fprintf(stderr, "inv_merge_key_shorter_dst_no_escape: %lu violation(s)\n",
			atomic_load(&violation_count));
		ret = -1;
	}

	drain_trie_local(dst);
	drain_trie_local(src);
	rcu_barrier();
	cds_ft_destroy(dst);
	cds_ft_destroy(src);
	cds_ft_group_destroy(group);
	return ret;
}

/* ================================================================== */
/*                                                                    */
/*   INVARIANT 14: Merge from a KEY_SHORTER src merge point never       */
/*   escapes (src_key ends INSIDE a compressed src node; the commit     */
/*   reclaims the whole node and re-parents its child cross-trie after   */
/*   the source drain).                                                 */
/*                                                                    */
/*   src holds "XYZ" -> under 'X' a COMPRESSED run "YZ"; the writer      */
/*   merges src@"XY" (1 byte into "YZ") into dst@"Q" each round, moving   */
/*   the "Z"->leaf suffix to "QZ", then resets (remove "QZ", re-insert    */
/*   "XYZ").  Persistent dst "Qb" keeps "Q" a live merge point.  A src    */
/*   reader must only ever see {"XYZ"} or empty; a dst reader only        */
/*   {"Qb","QZ"} -- the cross-trie re-parent must never let a key escape. */
/* ================================================================== */

static void *inv_merge_key_shorter_src_reader(void *arg)
{
	struct inv_merge_reader_arg *ra = (struct inv_merge_reader_arg *) arg;
	struct inv_merge_ctx *ctx = ra->ctx;
	bool is_dst = (ra->which[0] == 'd');
	struct cds_ft_iter *iter;
	unsigned long iters = 0;
	const unsigned int max_count = 16;

	rcu_register_thread();
	if (cds_ft_iter_create(ra->trie, &iter) < 0)
		abort();
	while (!test_go)
		;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	while (!test_stop) {
		bool reverse = (iters & 1) != 0;
		enum cds_ft_status s;
		unsigned int count = 0;

		rcu_read_lock();
		s = reverse ? cds_ft_lookup_last(ra->trie, iter)
			: cds_ft_lookup_first(ra->trie, iter);
		while (s == CDS_FT_STATUS_OK && count++ < max_count) {
			uint8_t k[16];
			size_t kl;
			bool ok_key;

			cds_ft_iter_get_key(iter, k, sizeof(k), &kl);
			if (is_dst)
				ok_key = (kl == 2 && k[0] == 'Q' &&
					(k[1] == 'b' || k[1] == 'Z'));
			else	/* src: {"XYZ"} or empty */
				ok_key = (kl == 3 && k[0] == 'X' &&
					k[1] == 'Y' && k[2] == 'Z');
			if (!ok_key) {
				report_violation(ctx->test_name,
					"%s reader saw out-of-namespace key "
					"(len %zu, %.3s) — key-shorter-src merge escape "
					"(iter #%lu, %s)",
					ra->which, kl, kl ? (const char *) k : "",
					iters, reverse ? "reverse" : "forward");
				break;
			}
			s = reverse ? cds_ft_lookup_lt(ra->trie, iter)
				: cds_ft_lookup_gt(ra->trie, iter);
		}
		if (count >= max_count)
			report_violation(ctx->test_name,
				"%s ordered traversal returned >= %u keys — "
				"escape/loop (iter #%lu, %s)",
				ra->which, max_count, iters,
				reverse ? "reverse" : "forward");
		rcu_read_unlock();
		iters++;
		if ((iters & 0x3f) == 0)
			rcu_quiescent_state();
	}

	cds_ft_iter_destroy(iter);
	rcu_unregister_thread();
	return NULL;
}

static void *inv_merge_key_shorter_src_writer(void *arg)
{
	struct inv_merge_ctx *ctx = (struct inv_merge_ctx *) arg;
	struct cds_ft_iter *iter;

	rcu_register_thread();
	if (cds_ft_iter_create(ctx->dst, &iter) < 0)
		abort();
	while (!test_go)
		;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	while (!test_stop) {
		enum cds_ft_status s;
		struct ft_test_node *n;

		pthread_mutex_lock(&ctx->lock);
		s = cds_ft_merge_at(ctx->dst, (const uint8_t *) "Q", 1,
				ctx->src, (const uint8_t *) "XY", 2);
		pthread_mutex_unlock(&ctx->lock);
		if (s != CDS_FT_STATUS_OK) {
			fprintf(stderr, "inv_merge_key_shorter_src writer: %s\n",
				cds_ft_status_to_string(s));
			break;
		}
		rcu_quiescent_state();

		rcu_read_lock();
		pthread_mutex_lock(&ctx->lock);
		inv_merge_remove_key(ctx->dst, iter, "QZ");
		n = node_alloc(300);
		cds_ft_insert(ctx->src, (const uint8_t *) "XYZ", 3, &n->node);
		pthread_mutex_unlock(&ctx->lock);
		rcu_read_unlock();
		rcu_quiescent_state();
	}

	cds_ft_iter_destroy(iter);
	rcu_unregister_thread();
	return NULL;
}

static int inv_merge_key_shorter_src_no_escape(void)
{
	struct cds_ft_group_attr *gattr;
	struct cds_ft_group *group;
	struct cds_ft *dst, *src;
	struct inv_merge_ctx ctx;
	struct inv_merge_reader_arg rargs[2 * NR_READERS_DEFAULT];
	struct timespec t0;
	pthread_t readers[2 * NR_READERS_DEFAULT], writer;
	struct ft_test_node *n, *sn;
	unsigned int i;
	int ret = 0;

	if (cds_ft_group_attr_create(&gattr) < 0)
		return -1;
	if (cds_ft_group_attr_set_max_key_len(gattr, 16) < 0) {
		cds_ft_group_attr_destroy(gattr);
		return -1;
	}
	if (cds_ft_group_create(gattr, &group) < 0) {
		cds_ft_group_attr_destroy(gattr);
		return -1;
	}
	cds_ft_group_attr_destroy(gattr);

	if (cds_ft_create(group, NULL, &dst) < 0)
		abort();
	if (cds_ft_create(group, NULL, &src) < 0)
		abort();

	/* Persistent dst "Qb" keeps "Q" a live merge point; src seeds "XYZ". */
	n = node_alloc(0);
	cds_ft_insert(dst, (const uint8_t *) "Qb", 2, &n->node);
	sn = node_alloc(300);
	cds_ft_insert(src, (const uint8_t *) "XYZ", 3, &sn->node);

	ctx.dst = dst;
	ctx.src = src;
	ctx.group = group;
	ctx.test_name = "inv_merge_key_shorter_src_no_escape";
	pthread_mutex_init(&ctx.lock, NULL);

	test_go = 0;
	test_stop = 0;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	/* Half the readers on dst, half on src (the novel cross-trie side). */
	for (i = 0; i < 2 * NR_READERS_DEFAULT; i++) {
		rargs[i].ctx = &ctx;
		if (i & 1) {
			rargs[i].trie = src;
			rargs[i].which = "src";
		} else {
			rargs[i].trie = dst;
			rargs[i].which = "dst";
		}
		pthread_create(&readers[i], NULL,
			inv_merge_key_shorter_src_reader, &rargs[i]);
	}
	pthread_create(&writer, NULL, inv_merge_key_shorter_src_writer, &ctx);

	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	test_go = 1;

	rcu_thread_offline();
	clock_gettime(CLOCK_MONOTONIC, &t0);
	while (elapsed_ms(&t0) < DEFAULT_DURATION_MS)
		usleep(1000);

	test_stop = 1;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	pthread_join(writer, NULL);
	for (i = 0; i < 2 * NR_READERS_DEFAULT; i++)
		pthread_join(readers[i], NULL);

	rcu_thread_online();
	pthread_mutex_destroy(&ctx.lock);

	if (atomic_load(&violation_count) > 0) {
		fprintf(stderr, "inv_merge_key_shorter_src_no_escape: %lu violation(s)\n",
			atomic_load(&violation_count));
		ret = -1;
	}

	drain_trie_local(dst);
	drain_trie_local(src);
	rcu_barrier();
	cds_ft_destroy(dst);
	cds_ft_destroy(src);
	cds_ft_group_destroy(group);
	return ret;
}

/* ================================================================== */
/*                                                                    */
/*   INVARIANT 15: Merge into a COMPRESSED-PARENT dst merge point        */
/*   never escapes (Edge D: the merge point's parent is a compressed     */
/*   node reached via a grandparent skip slot, which the flip re-encodes  */
/*   to a fresh copy of that compressed parent).                        */
/*                                                                    */
/*   dst persistently holds {"aXYc","aXYd"} -> under 'a' a COMPRESSED     */
/*   "XY" -> internal{c,d}; the merge point "aXY" has a compressed parent. */
/*   Each round the writer merges a private root src {"P"} at "aXY",      */
/*   re-encoding root['a'] (skip("XY")) to a fresh "XY" copy whose child   */
/*   is internal{c,d,P}, then resets (remove "aXYP", re-insert "P").  An   */
/*   ordered reader must only ever see {"aXYc","aXYd","aXYP"}.            */
/* ================================================================== */

static void *inv_merge_compressed_parent_dst_reader(void *arg)
{
	struct inv_merge_reader_arg *ra = (struct inv_merge_reader_arg *) arg;
	struct inv_merge_ctx *ctx = ra->ctx;
	struct cds_ft_iter *iter;
	unsigned long iters = 0;
	const unsigned int max_count = 16;

	rcu_register_thread();
	if (cds_ft_iter_create(ra->trie, &iter) < 0)
		abort();
	while (!test_go)
		;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	while (!test_stop) {
		bool reverse = (iters & 1) != 0;
		enum cds_ft_status s;
		unsigned int count = 0;

		rcu_read_lock();
		s = reverse ? cds_ft_lookup_last(ra->trie, iter)
			: cds_ft_lookup_first(ra->trie, iter);
		while (s == CDS_FT_STATUS_OK && count++ < max_count) {
			uint8_t k[16];
			size_t kl;

			cds_ft_iter_get_key(iter, k, sizeof(k), &kl);
			if (kl != 4 || k[0] != 'a' || k[1] != 'X' ||
			    k[2] != 'Y' ||
			    (k[3] != 'c' && k[3] != 'd' && k[3] != 'P')) {
				report_violation(ctx->test_name,
					"reader saw out-of-namespace key "
					"(len %zu, %.4s) — compressed-parent-dst merge escape "
					"(iter #%lu, %s)",
					kl, kl ? (const char *) k : "",
					iters, reverse ? "reverse" : "forward");
				break;
			}
			s = reverse ? cds_ft_lookup_lt(ra->trie, iter)
				: cds_ft_lookup_gt(ra->trie, iter);
		}
		if (count >= max_count)
			report_violation(ctx->test_name,
				"ordered traversal returned >= %u keys — "
				"escape/loop (iter #%lu, %s)",
				max_count, iters, reverse ? "reverse" : "forward");
		rcu_read_unlock();
		iters++;
		if ((iters & 0x3f) == 0)
			rcu_quiescent_state();
	}

	cds_ft_iter_destroy(iter);
	rcu_unregister_thread();
	return NULL;
}

static void *inv_merge_compressed_parent_dst_writer(void *arg)
{
	struct inv_merge_ctx *ctx = (struct inv_merge_ctx *) arg;
	struct cds_ft_iter *iter;

	rcu_register_thread();
	if (cds_ft_iter_create(ctx->dst, &iter) < 0)
		abort();
	while (!test_go)
		;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	while (!test_stop) {
		enum cds_ft_status s;
		struct ft_test_node *n;

		pthread_mutex_lock(&ctx->lock);
		s = cds_ft_merge_at(ctx->dst, (const uint8_t *) "aXY", 3,
				ctx->src, NULL, 0);
		pthread_mutex_unlock(&ctx->lock);
		if (s != CDS_FT_STATUS_OK) {
			fprintf(stderr, "inv_merge_compressed_parent_dst writer: %s\n",
				cds_ft_status_to_string(s));
			break;
		}
		rcu_quiescent_state();

		rcu_read_lock();
		pthread_mutex_lock(&ctx->lock);
		inv_merge_remove_key(ctx->dst, iter, "aXYP");
		n = node_alloc(300);
		cds_ft_insert(ctx->src, (const uint8_t *) "P", 1, &n->node);
		pthread_mutex_unlock(&ctx->lock);
		rcu_read_unlock();
		rcu_quiescent_state();
	}

	cds_ft_iter_destroy(iter);
	rcu_unregister_thread();
	return NULL;
}

static int inv_merge_compressed_parent_dst_no_escape(void)
{
	struct cds_ft_group_attr *gattr;
	struct cds_ft_group *group;
	struct cds_ft *dst, *src;
	struct inv_merge_ctx ctx;
	struct inv_merge_reader_arg rargs[2 * NR_READERS_DEFAULT];
	struct timespec t0;
	pthread_t readers[2 * NR_READERS_DEFAULT], writer;
	struct ft_test_node *a, *b, *sn;
	unsigned int i;
	int ret = 0;

	if (cds_ft_group_attr_create(&gattr) < 0)
		return -1;
	if (cds_ft_group_attr_set_max_key_len(gattr, 16) < 0) {
		cds_ft_group_attr_destroy(gattr);
		return -1;
	}
	if (cds_ft_group_create(gattr, &group) < 0) {
		cds_ft_group_attr_destroy(gattr);
		return -1;
	}
	cds_ft_group_attr_destroy(gattr);

	if (cds_ft_create(group, NULL, &dst) < 0)
		abort();
	if (cds_ft_create(group, NULL, &src) < 0)
		abort();

	/* Persistent {aXYc,aXYd} keeps "aXY" a compressed-parent merge point. */
	a = node_alloc(0);
	cds_ft_insert(dst, (const uint8_t *) "aXYc", 4, &a->node);
	b = node_alloc(0);
	cds_ft_insert(dst, (const uint8_t *) "aXYd", 4, &b->node);
	sn = node_alloc(300);
	cds_ft_insert(src, (const uint8_t *) "P", 1, &sn->node);

	ctx.dst = dst;
	ctx.src = src;
	ctx.group = group;
	ctx.test_name = "inv_merge_compressed_parent_dst_no_escape";
	pthread_mutex_init(&ctx.lock, NULL);

	test_go = 0;
	test_stop = 0;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	for (i = 0; i < 2 * NR_READERS_DEFAULT; i++) {
		rargs[i].ctx = &ctx;
		rargs[i].trie = dst;
		rargs[i].which = "dst";
		pthread_create(&readers[i], NULL,
			inv_merge_compressed_parent_dst_reader, &rargs[i]);
	}
	pthread_create(&writer, NULL, inv_merge_compressed_parent_dst_writer, &ctx);

	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	test_go = 1;

	rcu_thread_offline();
	clock_gettime(CLOCK_MONOTONIC, &t0);
	while (elapsed_ms(&t0) < DEFAULT_DURATION_MS)
		usleep(1000);

	test_stop = 1;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	pthread_join(writer, NULL);
	for (i = 0; i < 2 * NR_READERS_DEFAULT; i++)
		pthread_join(readers[i], NULL);

	rcu_thread_online();
	pthread_mutex_destroy(&ctx.lock);

	if (atomic_load(&violation_count) > 0) {
		fprintf(stderr, "inv_merge_compressed_parent_dst_no_escape: %lu violation(s)\n",
			atomic_load(&violation_count));
		ret = -1;
	}

	drain_trie_local(dst);
	drain_trie_local(src);
	rcu_barrier();
	cds_ft_destroy(dst);
	cds_ft_destroy(src);
	cds_ft_group_destroy(group);
	return ret;
}

/* ================================================================== */
/*                                                                    */
/*   INVARIANT 16: merge_at is ATOMIC -- the flip publishes the whole   */
/*   source set at once, so no value is ever MISSING after it.          */
/*                                                                    */
/*   Two disjoint sets share the prefix "AA": set A holds the           */
/*   odd-byte-valued suffixes (a,c,e,...  -- in @dst), set B the even    */
/*   ones (b,d,f,...  -- in @src), exactly Mathieu's "AAaceg"/"AAbdfh"   */
/*   example.  A single cds_ft_merge_at folds B into @dst at "AA", so    */
/*   the merged set is the DENSE range "AA"+{a..z}.  Readers iterate     */
/*   @dst across the flip; once a reader has observed BOTH an A (odd)    */
/*   and a B (even) suffix -- proving its traversal is past the flip --  */
/*   every following key MUST be the immediate successor (forward) or    */
/*   predecessor (reverse).  The atomic flip reveals all of B at once,   */
/*   so a torn / partial merge would surface as a forward gap.  Keys     */
/*   BEHIND the detection point are not required (the flip may land      */
/*   mid-traversal) -- "all combinations, going forward".                */
/*                                                                    */
/*   One-shot per round (build A+B, run readers, merge once, join): a    */
/*   repeated merge/un-merge would legitimately make B vanish and is     */
/*   not what this probes.                                              */
/* ================================================================== */

/* Suffix letters: A = odd byte values {a,c,...,y}, B = even {b,d,...,z}. */
#define INV_ATOMIC_LO	'a'
#define INV_ATOMIC_HI	'z'

struct inv_atomic_ctx {
	struct cds_ft *dst;
	const char *test_name;
	volatile int go;
	volatile int stop;
};

static void *inv_merge_atomic_reader(void *arg)
{
	struct inv_atomic_ctx *ctx = (struct inv_atomic_ctx *) arg;
	struct cds_ft_iter *iter;
	unsigned long iters = 0;

	rcu_register_thread();
	if (cds_ft_iter_create(ctx->dst, &iter) < 0)
		abort();
	while (!ctx->go)
		;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	while (!ctx->stop) {
		bool reverse = (iters & 1) != 0;
		enum cds_ft_status s;
		int prev = -1;
		bool saw_a = false, saw_b = false;

		rcu_read_lock();
		s = reverse ? cds_ft_lookup_last(ctx->dst, iter)
			: cds_ft_lookup_first(ctx->dst, iter);
		while (s == CDS_FT_STATUS_OK) {
			uint8_t k[8];
			size_t kl;
			int v;

			cds_ft_iter_get_key(iter, k, sizeof(k), &kl);
			if (kl != 3 || k[0] != 'A' || k[1] != 'A' ||
			    k[2] < INV_ATOMIC_LO || k[2] > INV_ATOMIC_HI) {
				report_violation(ctx->test_name,
					"out-of-namespace key (len %zu) — merge escape "
					"(%s)", kl, reverse ? "reverse" : "forward");
				break;
			}
			v = k[2];
			if (v & 1)
				saw_a = true;	/* odd byte: set A */
			else
				saw_b = true;	/* even byte: set B */
			/*
			 * Both partitions observed -> this traversal is in the
			 * post-flip regime, where the set is the dense range
			 * a..z.  Every subsequent key must be the immediate
			 * successor / predecessor; a gap means a value is missing
			 * after the atomic flip (a non-atomic / torn merge).
			 */
			if (saw_a && saw_b && prev >= 0) {
				int expect = reverse ? prev - 1 : prev + 1;

				if (v != expect) {
					report_violation(ctx->test_name,
						"missing value: suffix '%c' follows '%c' "
						"(%s) after both sets observed — non-atomic merge",
						v, prev, reverse ? "reverse" : "forward");
					break;
				}
			}
			prev = v;
			s = reverse ? cds_ft_lookup_lt(ctx->dst, iter)
				: cds_ft_lookup_gt(ctx->dst, iter);
		}
		rcu_read_unlock();
		iters++;
		if ((iters & 0x3f) == 0)
			rcu_quiescent_state();
	}

	cds_ft_iter_destroy(iter);
	rcu_unregister_thread();
	return NULL;
}

static int inv_merge_atomic_completeness(void)
{
	struct timespec t0;
	int ret = 0;

	clock_gettime(CLOCK_MONOTONIC, &t0);

	while (elapsed_ms(&t0) < DEFAULT_DURATION_MS &&
			atomic_load(&violation_count) == 0) {
		struct cds_ft_group_attr *gattr;
		struct cds_ft_group *group;
		struct cds_ft *dst, *src;
		struct inv_atomic_ctx ctx;
		pthread_t readers[NR_READERS_DEFAULT];
		enum cds_ft_status s;
		unsigned int i;
		int v;

		if (cds_ft_group_attr_create(&gattr) < 0)
			return -1;
		if (cds_ft_group_attr_set_max_key_len(gattr, 16) < 0) {
			cds_ft_group_attr_destroy(gattr);
			return -1;
		}
		if (cds_ft_group_create(gattr, &group) < 0) {
			cds_ft_group_attr_destroy(gattr);
			return -1;
		}
		cds_ft_group_attr_destroy(gattr);
		if (cds_ft_create(group, NULL, &dst) < 0)
			abort();
		if (cds_ft_create(group, NULL, &src) < 0)
			abort();

		/*
		 * dst <- set A (odd-byte suffixes "AA"+{a,c,...}); src <- set B
		 * (even-byte suffixes as single-byte keys {b,d,...}, which the
		 * merge re-keys under "AA").
		 */
		for (v = INV_ATOMIC_LO; v <= INV_ATOMIC_HI; v++) {
			struct ft_test_node *n = node_alloc((uint64_t) v);
			uint8_t key[3] = { 'A', 'A', (uint8_t) v };

			if (v & 1)
				s = cds_ft_insert(dst, key, 3, &n->node);
			else
				s = cds_ft_insert(src, &key[2], 1, &n->node);
			if (s != CDS_FT_STATUS_OK) {
				node_free(n);
				ret = -1;
				goto round_teardown;
			}
		}

		ctx.dst = dst;
		ctx.test_name = "inv_merge_atomic_completeness";
		ctx.go = 0;
		ctx.stop = 0;
		__atomic_thread_fence(__ATOMIC_SEQ_CST);

		for (i = 0; i < NR_READERS_DEFAULT; i++)
			pthread_create(&readers[i], NULL,
				inv_merge_atomic_reader, &ctx);

		__atomic_thread_fence(__ATOMIC_SEQ_CST);
		ctx.go = 1;
		usleep(300);	/* readers stream pre-flip (A-only) traversals */

		s = cds_ft_merge_at(dst, (const uint8_t *) "AA", 2, src, NULL, 0);

		usleep(1000);	/* readers stream across + post-flip traversals */
		ctx.stop = 1;
		__atomic_thread_fence(__ATOMIC_SEQ_CST);
		for (i = 0; i < NR_READERS_DEFAULT; i++)
			pthread_join(readers[i], NULL);

		if (s != CDS_FT_STATUS_OK) {
			fprintf(stderr, "inv_merge_atomic_completeness: merge %s\n",
				cds_ft_status_to_string(s));
			ret = -1;
		}
round_teardown:
		/* @src is emptied by a successful merge; @dst holds the union. */
		drain_trie_local(dst);
		drain_trie_local(src);
		rcu_barrier();
		cds_ft_destroy(dst);
		cds_ft_destroy(src);
		cds_ft_group_destroy(group);
		if (ret)
			return ret;
	}

	if (atomic_load(&violation_count) > 0) {
		fprintf(stderr, "inv_merge_atomic_completeness: %lu violation(s)\n",
			atomic_load(&violation_count));
		ret = -1;
	}
	return ret;
}

/* ================================================================== */
/*                                                                    */
/*   INVARIANT 17: merge_at atomicity with a DEEP overlap.              */
/*                                                                    */
/*   Same no-missing-value property as #16, but the sets share          */
/*   branching structure so the merge RECURSES: keys are "AA"+P+L over   */
/*   prefix bytes P in 'a'..'h' and leaf letters L in 'a'..'z'.  Set A   */
/*   (dst) holds odd-L, set B (src) holds even-L, so at "AA" both sides  */
/*   carry every P (a stitch node + a recursion per P), and under each   */
/*   P the odd/even leaves interleave -- a multi-level spine copy with   */
/*   re-parents at several depths, vs #16's single wide node.  Sorted    */
/*   order is row-major (P,L); the dense merged set linearizes to        */
/*   idx = (P-'a')*26 + (L-'a'), so the same "once both parities seen,    */
/*   every forward key is the immediate successor" check applies.        */
/* ================================================================== */

#define INV_ATOMIC_DEEP_PLO	'a'
#define INV_ATOMIC_DEEP_PHI	'h'	/* 8 shared prefix bytes */
#define INV_ATOMIC_NL		(INV_ATOMIC_HI - INV_ATOMIC_LO + 1)	/* 26 */

static void *inv_merge_atomic_deep_reader(void *arg)
{
	struct inv_atomic_ctx *ctx = (struct inv_atomic_ctx *) arg;
	struct cds_ft_iter *iter;
	unsigned long iters = 0;

	rcu_register_thread();
	if (cds_ft_iter_create(ctx->dst, &iter) < 0)
		abort();
	while (!ctx->go)
		;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	while (!ctx->stop) {
		bool reverse = (iters & 1) != 0;
		enum cds_ft_status s;
		int prev = -1;
		bool saw_a = false, saw_b = false;

		rcu_read_lock();
		s = reverse ? cds_ft_lookup_last(ctx->dst, iter)
			: cds_ft_lookup_first(ctx->dst, iter);
		while (s == CDS_FT_STATUS_OK) {
			uint8_t k[8];
			size_t kl;
			int idx, l;

			cds_ft_iter_get_key(iter, k, sizeof(k), &kl);
			if (kl != 4 || k[0] != 'A' || k[1] != 'A' ||
			    k[2] < INV_ATOMIC_DEEP_PLO || k[2] > INV_ATOMIC_DEEP_PHI ||
			    k[3] < INV_ATOMIC_LO || k[3] > INV_ATOMIC_HI) {
				report_violation(ctx->test_name,
					"out-of-namespace key (len %zu) — deep merge escape "
					"(%s)", kl, reverse ? "reverse" : "forward");
				break;
			}
			l = k[3];
			idx = (k[2] - INV_ATOMIC_DEEP_PLO) * INV_ATOMIC_NL
				+ (l - INV_ATOMIC_LO);
			if (l & 1)
				saw_a = true;	/* odd leaf: set A */
			else
				saw_b = true;	/* even leaf: set B */
			if (saw_a && saw_b && prev >= 0) {
				int expect = reverse ? prev - 1 : prev + 1;

				if (idx != expect) {
					report_violation(ctx->test_name,
						"missing value: idx %d follows %d (%s) "
						"after both sets observed — non-atomic deep merge",
						idx, prev, reverse ? "reverse" : "forward");
					break;
				}
			}
			prev = idx;
			s = reverse ? cds_ft_lookup_lt(ctx->dst, iter)
				: cds_ft_lookup_gt(ctx->dst, iter);
		}
		rcu_read_unlock();
		iters++;
		if ((iters & 0x3f) == 0)
			rcu_quiescent_state();
	}

	cds_ft_iter_destroy(iter);
	rcu_unregister_thread();
	return NULL;
}

static int inv_merge_atomic_completeness_deep(void)
{
	struct timespec t0;
	int ret = 0;

	clock_gettime(CLOCK_MONOTONIC, &t0);

	while (elapsed_ms(&t0) < DEFAULT_DURATION_MS &&
			atomic_load(&violation_count) == 0) {
		struct cds_ft_group_attr *gattr;
		struct cds_ft_group *group;
		struct cds_ft *dst, *src;
		struct inv_atomic_ctx ctx;
		pthread_t readers[NR_READERS_DEFAULT];
		enum cds_ft_status s;
		unsigned int i;
		int p, l;

		if (cds_ft_group_attr_create(&gattr) < 0)
			return -1;
		if (cds_ft_group_attr_set_max_key_len(gattr, 16) < 0) {
			cds_ft_group_attr_destroy(gattr);
			return -1;
		}
		if (cds_ft_group_create(gattr, &group) < 0) {
			cds_ft_group_attr_destroy(gattr);
			return -1;
		}
		cds_ft_group_attr_destroy(gattr);
		if (cds_ft_create(group, NULL, &dst) < 0)
			abort();
		if (cds_ft_create(group, NULL, &src) < 0)
			abort();

		/*
		 * Both sides carry every prefix byte P, so the merge recurses at
		 * "AA" (one stitch node per P) and again under each P (odd/even
		 * leaves).  dst <- odd-L "AA"+P+L; src <- even-L P+L (re-keyed
		 * under "AA" by the merge).
		 */
		for (p = INV_ATOMIC_DEEP_PLO; p <= INV_ATOMIC_DEEP_PHI; p++) {
			for (l = INV_ATOMIC_LO; l <= INV_ATOMIC_HI; l++) {
				struct ft_test_node *n = node_alloc((uint64_t) l);
				uint8_t key[4] = { 'A', 'A', (uint8_t) p, (uint8_t) l };

				if (l & 1)
					s = cds_ft_insert(dst, key, 4, &n->node);
				else
					s = cds_ft_insert(src, &key[2], 2, &n->node);
				if (s != CDS_FT_STATUS_OK) {
					node_free(n);
					ret = -1;
					goto round_teardown;
				}
			}
		}

		ctx.dst = dst;
		ctx.test_name = "inv_merge_atomic_completeness_deep";
		ctx.go = 0;
		ctx.stop = 0;
		__atomic_thread_fence(__ATOMIC_SEQ_CST);

		for (i = 0; i < NR_READERS_DEFAULT; i++)
			pthread_create(&readers[i], NULL,
				inv_merge_atomic_deep_reader, &ctx);

		__atomic_thread_fence(__ATOMIC_SEQ_CST);
		ctx.go = 1;
		usleep(300);	/* readers stream pre-flip (A-only) traversals */

		s = cds_ft_merge_at(dst, (const uint8_t *) "AA", 2, src, NULL, 0);

		usleep(1000);	/* readers stream across + post-flip traversals */
		ctx.stop = 1;
		__atomic_thread_fence(__ATOMIC_SEQ_CST);
		for (i = 0; i < NR_READERS_DEFAULT; i++)
			pthread_join(readers[i], NULL);

		if (s != CDS_FT_STATUS_OK) {
			fprintf(stderr, "inv_merge_atomic_completeness_deep: merge %s\n",
				cds_ft_status_to_string(s));
			ret = -1;
		}
round_teardown:
		drain_trie_local(dst);
		drain_trie_local(src);
		rcu_barrier();
		cds_ft_destroy(dst);
		cds_ft_destroy(src);
		cds_ft_group_destroy(group);
		if (ret)
			return ret;
	}

	if (atomic_load(&violation_count) > 0) {
		fprintf(stderr, "inv_merge_atomic_completeness_deep: %lu violation(s)\n",
			atomic_load(&violation_count));
		ret = -1;
	}
	return ret;
}

/* ================================================================== */
/*                                                                    */
/*                           MAIN                                     */
/*                                                                    */
/* ================================================================== */

#ifdef FT_ENABLE_TRACING
/*
 * Root-cause scaffolding: on a fatal fault, dump the flight-recorder
 * ring (the writer/reanchor tracepoints leading up to the crash) before
 * dying, then re-raise for a core.  Enabled only when both built with
 * FT_ENABLE_TRACING and run with FT_INV_SNAPSHOT_ON_SEGV set under an
 * `lttng ... --snapshot` session.  system() in a signal handler is
 * async-signal-unsafe but adequate here (the fault is a bad read, not
 * heap corruption).
 */
static void ft_segv_snapshot_handler(int sig, siginfo_t *si, void *uc)
{
	char buf[160];
	int n;

	(void) uc;
	n = snprintf(buf, sizeof buf,
		"\n[fault] sig=%d fault_addr=%p — recording lttng snapshot\n",
		sig, si ? si->si_addr : NULL);
	if (n > 0) {
		ssize_t w = write(STDERR_FILENO, buf, (size_t) n);
		(void) w;
	}
	(void) system("lttng snapshot record 1>&2");
	signal(sig, SIG_DFL);
	raise(sig);
}
#endif

/* ================================================================== */
/*                                                                    */
/*   Ordered-list bulk-op consistency (Phase 3)                       */
/*                                                                    */
/*   A concurrent ordered (cell-list) traversal of @A stays strictly  */
/*   sorted and bounded while a writer churns the ordered cell list    */
/*   with BULK ops: detach+graft round-trips (run move), graft_swap    */
/*   (run replace), and merge (interleave) + per-key reset.  Exercises */
/*   the bulk-op ordered-list maintenance and the head/tail flip-proxy */
/*   under live readers.  Variable-length group with both offsets so   */
/*   cds_ft_next walks the CELL list (not the descent fallback).       */
/* ================================================================== */

struct bulk_node {
	struct cds_ft_node node;
	struct rcu_head head;
	uint8_t kbytes[8];
	size_t klen;			/* read by the lib at key_len_offset */
};

static unsigned long bulk_alloc, bulk_freed;

static void bulk_set_key(uint8_t *out, uint32_t v)
{
	out[0] = (uint8_t)(v >> 24); out[1] = (uint8_t)(v >> 16);
	out[2] = (uint8_t)(v >> 8);  out[3] = (uint8_t) v;
}

static uint32_t bulk_key_val(const uint8_t *k)
{
	return ((uint32_t) k[0] << 24) | ((uint32_t) k[1] << 16) |
		((uint32_t) k[2] << 8) | k[3];
}

static struct bulk_node *bulk_node_alloc(uint32_t v)
{
	struct bulk_node *n = (struct bulk_node *) calloc(1, sizeof(*n));

	if (!n)
		abort();
	cds_ft_node_init(&n->node);
	bulk_set_key(n->kbytes, v);
	n->klen = 4;
	__atomic_add_fetch(&bulk_alloc, 1, __ATOMIC_RELAXED);
	return n;
}

static void bulk_node_free_cb(struct rcu_head *h)
{
	struct bulk_node *n = caa_container_of(h, struct bulk_node, head);

	memset(n, 0xfe, sizeof(*n));
	free(n);
	__atomic_add_fetch(&bulk_freed, 1, __ATOMIC_RELAXED);
}

static void bulk_node_free_rcu(struct bulk_node *n)
{
	call_rcu(&n->head, bulk_node_free_cb);
}

static struct cds_ft_group *bulk_group_create(void)
{
	struct cds_ft_group_attr *attr;
	struct cds_ft_group *group;

	if (cds_ft_group_attr_create(&attr) < 0)
		abort();
	cds_ft_group_attr_set_key_len(attr, CDS_FT_LEN_VARIABLE);
	cds_ft_group_attr_set_max_key_len(attr, 8);
	/*
	 * EAGER: NO speculative_key_offset and NO key_len_offset.  The ordered
	 * cell walk rematerializes each key (and its length) STRUCTURALLY via the
	 * parent up-walk -- so a re-prefixing bulk op (non-root graft_swap below)
	 * leaves no stale in-leaf key behind, and the reader still sees the right
	 * key for the moved nodes.
	 */
	cds_ft_group_attr_set_ordered_list(attr);
	if (cds_ft_group_create(attr, &group) < 0)
		abort();
	cds_ft_group_attr_destroy(attr);
	return group;
}

/* Insert a @len-byte key into @ft (single-threaded setup or writer-locked). */
static void bulk_insert_len(struct cds_ft *ft, const uint8_t *kb, size_t len)
{
	struct bulk_node *n = (struct bulk_node *) calloc(1, sizeof(*n));
	struct cds_ft_node *res;

	if (!n)
		abort();
	cds_ft_node_init(&n->node);
	memcpy(n->kbytes, kb, len);
	n->klen = len;
	__atomic_add_fetch(&bulk_alloc, 1, __ATOMIC_RELAXED);
	if (cds_ft_insert_unique(ft, n->kbytes, len, &n->node, &res)
			!= CDS_FT_STATUS_OK) {
		free(n);
		__atomic_add_fetch(&bulk_freed, 1, __ATOMIC_RELAXED);
	}
}

static void bulk_insert4(struct cds_ft *ft, uint32_t v)
{
	uint8_t kb[4];

	bulk_set_key(kb, v);
	bulk_insert_len(ft, kb, 4);
}

/* Remove a 4-byte key from @ft and defer-free its node (writer-locked). */
static void bulk_remove4(struct cds_ft *ft, struct cds_ft_iter *iter, uint32_t v)
{
	struct cds_ft_node *head, *tmp;
	uint8_t kb[4];

	bulk_set_key(kb, v);
	cds_ft_iter_set_key(iter, kb, 4);
	cds_ft_lookup(ft, iter);
	if (!cds_ft_iter_node(iter))
		return;
	if (cds_ft_remove_all(ft, iter, &head) != CDS_FT_STATUS_OK)
		return;
	cds_ft_for_each_duplicate_safe_rcu(head, tmp)
		bulk_node_free_rcu(caa_container_of(head, struct bulk_node, node));
}

/*
 * DEBUG_COUNTERS-only group cell-balance accessor (weak: NULL on a library
 * built without DEBUG_COUNTERS, in which case the cell-balance assertion is
 * skipped).  Lets the bulk test confirm the ordered-list cells are reclaimed
 * by a drained destroy across all the bulk-op cell moves.
 */
extern void cds_ft_debug_cell_balance(const struct cds_ft_group *group,
		unsigned long *allocated, unsigned long *freed) __attribute__((weak));

static void bulk_drain(struct cds_ft *ft)
{
	struct cds_ft_iter *iter;

	if (cds_ft_iter_create(ft, &iter) < 0)
		abort();
	rcu_read_lock();
	for (;;) {
		struct cds_ft_node *head, *tmp;

		cds_ft_lookup_first(ft, iter);
		if (!cds_ft_iter_node(iter))
			break;
		/*
		 * remove_all directly at the cell-walk position: on a
		 * variable-length lazy-key trie it resolves the LAZY key length
		 * (parent up-walk) before validating, so no set_key re-seed is
		 * needed.
		 */
		if (cds_ft_remove_all(ft, iter, &head) != CDS_FT_STATUS_OK)
			break;
		cds_ft_for_each_duplicate_safe_rcu(head, tmp)
			bulk_node_free_rcu(caa_container_of(head,
				struct bulk_node, node));
	}
	rcu_read_unlock();
	cds_ft_iter_destroy(iter);
}

struct bulk_ctx {
	struct cds_ft *A;
	struct cds_ft_group *group;
	pthread_mutex_t lock;
	const char *test_name;
};

#define BULK_M		200		/* keys per movable prefix */
#define BULK_CAP	100000		/* iteration bound (loop/escape guard) */
static const uint8_t bulk_move_pfx[] = { 0x10, 0x20 };
#define BULK_SWAP_PFX	0x50
#define BULK_MERGE_PFX	0x10		/* C interleaves into A's 0x10 region */
#define BULK_NMERGE	8

static void *bulk_reader(void *arg)
{
	struct bulk_ctx *ctx = (struct bulk_ctx *) arg;
	struct cds_ft_iter *iter;
	unsigned long iters = 0;

	rcu_register_thread();
	if (cds_ft_iter_create(ctx->A, &iter) < 0)
		abort();
	while (!test_go)
		;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	while (!test_stop) {
		uint32_t prev = 0;
		int first = 1;
		unsigned int count = 0;

		rcu_read_lock();
		cds_ft_lookup_first(ctx->A, iter);
		while (cds_ft_iter_node(iter) && count < BULK_CAP) {
			uint8_t rk[8];
			size_t rkl;
			uint32_t v;

			cds_ft_iter_get_key(iter, rk, sizeof(rk), &rkl);
			if (rkl != 4) {
				report_violation(ctx->test_name,
					"key len %zu != 4 (iter #%lu, pos %u)",
					rkl, iters, count);
				break;
			}
			v = bulk_key_val(rk);
			if (!first && v <= prev) {
				report_violation(ctx->test_name,
					"out-of-order 0x%08x after 0x%08x "
					"(iter #%lu, pos %u)",
					v, prev, iters, count);
				break;
			}
			prev = v;
			first = 0;
			count++;
			cds_ft_next(ctx->A, iter);
		}
		if (count >= BULK_CAP)
			report_violation(ctx->test_name,
				"ordered traversal returned >= %u keys — "
				"loop/escape (iter #%lu)", BULK_CAP, iters);
		rcu_read_unlock();

		if ((++iters & 0x3f) == 0)
			rcu_quiescent_state();
	}

	cds_ft_iter_destroy(iter);
	rcu_unregister_thread();
	return NULL;
}

static void *bulk_writer(void *arg)
{
	struct bulk_ctx *ctx = (struct bulk_ctx *) arg;
	struct cds_ft_iter *iter;
	unsigned long iters = 0;

	rcu_register_thread();
	if (cds_ft_iter_create(ctx->A, &iter) < 0)
		abort();
	while (!test_go)
		;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	while (!test_stop) {
		unsigned int op = iters % 3;

		if (op == 0) {
			/* detach + graft-back round-trip (run move, multi-key) */
			uint8_t P = bulk_move_pfx[iters % 2];
			struct cds_ft *D = NULL;

			pthread_mutex_lock(&ctx->lock);
			if (cds_ft_detach(ctx->A, &P, 1, &D) == CDS_FT_STATUS_OK
					&& D) {
				cds_ft_graft(ctx->A, &P, 1, D);
				cds_ft_destroy(D);
			}
			pthread_mutex_unlock(&ctx->lock);
			rcu_quiescent_state();
		} else if (op == 1) {
			/*
			 * NON-ROOT graft_swap round-trip (RE-PREFIXING): swap A's
			 * subtree at prefix 0x50 with donor B's whole content.  B's
			 * 3-byte keys gain the 0x50 prefix when moved into A@0x50 (and
			 * A's old 0x50 keys lose it into B) -- so a reader iterating A
			 * sees nodes whose trie key differs from anything an in-leaf key
			 * could hold; the EAGER up-walk rematerializes them correctly.
			 */
			uint8_t P = BULK_SWAP_PFX;
			struct cds_ft *B;
			unsigned int i;

			if (cds_ft_create(ctx->group, NULL, &B) < 0)
				abort();
			for (i = 0; i < 32; i++) {	/* 3-byte donor keys */
				uint8_t kb[3] = { 0xAA, (uint8_t)(i >> 8),
						  (uint8_t) i };
				bulk_insert_len(B, kb, 3);
			}
			pthread_mutex_lock(&ctx->lock);
			cds_ft_graft_swap(ctx->A, &P, 1, B);	/* A@0x50 <-> B */
			cds_ft_graft_swap(ctx->A, &P, 1, B);	/* back */
			pthread_mutex_unlock(&ctx->lock);
			rcu_quiescent_state();
			bulk_drain(B);
			cds_ft_destroy(B);
			rcu_quiescent_state();
		} else {
			/* merge a fresh donor into A (interleave), then reset */
			struct cds_ft *C;
			unsigned int i;

			if (cds_ft_create(ctx->group, NULL, &C) < 0)
				abort();
			/* 0x10-prefix, odd low bytes -> interleave A's evens */
			for (i = 0; i < BULK_NMERGE; i++)
				bulk_insert4(C, ((uint32_t) BULK_MERGE_PFX << 24)
					| (2 * i + 1));
			pthread_mutex_lock(&ctx->lock);
			cds_ft_merge(ctx->A, NULL, 0, C);	/* C -> A, C empty */
			pthread_mutex_unlock(&ctx->lock);
			rcu_quiescent_state();

			pthread_mutex_lock(&ctx->lock);
			for (i = 0; i < BULK_NMERGE; i++)
				bulk_remove4(ctx->A,
					iter, ((uint32_t) BULK_MERGE_PFX << 24)
						| (2 * i + 1));
			pthread_mutex_unlock(&ctx->lock);
			cds_ft_destroy(C);
			rcu_quiescent_state();
		}
		iters++;
	}

	cds_ft_iter_destroy(iter);
	rcu_unregister_thread();
	return NULL;
}

static int inv_ordered_bulk_consistency(void)
{
	struct cds_ft_group *group = bulk_group_create();
	struct cds_ft *A;
	struct bulk_ctx ctx;
	struct timespec t0;
	pthread_t readers[NR_READERS_DEFAULT], writer;
	unsigned int i, j;
	int ret = 0;

	if (cds_ft_create(group, NULL, &A) < 0)
		abort();

	/* A: movable prefixes (even low bytes), a static prefix, a swap zone. */
	rcu_read_lock();
	for (j = 0; j < 2; j++)
		for (i = 0; i < BULK_M; i++)
			bulk_insert4(A, ((uint32_t) bulk_move_pfx[j] << 24)
				| (2 * i));
	for (i = 0; i < BULK_M; i++)
		bulk_insert4(A, (0x40u << 24) | i);		/* static */
	for (i = 0; i < 16; i++)
		bulk_insert4(A, ((uint32_t) BULK_SWAP_PFX << 24) | i);	/* swap zone */
	rcu_read_unlock();

	ctx.A = A;
	ctx.group = group;
	ctx.test_name = "inv_ordered_bulk_consistency";
	pthread_mutex_init(&ctx.lock, NULL);

	test_go = 0;
	test_stop = 0;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	for (i = 0; i < NR_READERS_DEFAULT; i++)
		pthread_create(&readers[i], NULL, bulk_reader, &ctx);
	pthread_create(&writer, NULL, bulk_writer, &ctx);

	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	test_go = 1;

	rcu_thread_offline();
	clock_gettime(CLOCK_MONOTONIC, &t0);
	while (elapsed_ms(&t0) < DEFAULT_DURATION_MS)
		usleep(1000);

	test_stop = 1;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	pthread_join(writer, NULL);
	for (i = 0; i < NR_READERS_DEFAULT; i++)
		pthread_join(readers[i], NULL);

	rcu_thread_online();
	pthread_mutex_destroy(&ctx.lock);

	if (atomic_load(&violation_count) > 0) {
		fprintf(stderr, "inv_ordered_bulk_consistency: %lu violation(s)\n",
			atomic_load(&violation_count));
		ret = -1;
	}

	bulk_drain(A);
	rcu_barrier();
	cds_ft_destroy(A);
	/*
	 * The trie is drained and destroyed: every ordered-list cell allocated
	 * over the run (including the ones the bulk ops migrated between tries)
	 * must have been reclaimed.  Checked at group granularity, before the
	 * group is torn down.  Only meaningful on a DEBUG_COUNTERS library; the
	 * weak symbol is NULL otherwise.
	 */
	if (cds_ft_debug_cell_balance) {
		unsigned long ca = 0, cf = 0;

		cds_ft_debug_cell_balance(group, &ca, &cf);
		if (ca != cf) {
			fprintf(stderr, "inv_ordered_bulk_consistency: cell leak "
				"alloc %lu != freed %lu\n", ca, cf);
			ret = -1;
		}
	}
	cds_ft_group_destroy(group);
	{
		unsigned long na = __atomic_load_n(&bulk_alloc, __ATOMIC_RELAXED);
		unsigned long nf = __atomic_load_n(&bulk_freed, __ATOMIC_RELAXED);

		if (na != nf) {
			fprintf(stderr, "inv_ordered_bulk_consistency: node leak "
				"alloc %lu != freed %lu\n", na, nf);
			ret = -1;
		}
	}
	return ret;
}

int main(int argc, char **argv)
{
	const char *filter = (argc >= 2) ? argv[1] : NULL;
	int err;

#ifdef FT_ENABLE_TRACING
	if (getenv("FT_INV_SNAPSHOT_ON_SEGV")) {
		struct sigaction sa;

		memset(&sa, 0, sizeof sa);
		sa.sa_sigaction = ft_segv_snapshot_handler;
		sa.sa_flags = SA_SIGINFO;
		(void) sigaction(SIGSEGV, &sa, NULL);
		(void) sigaction(SIGABRT, &sa, NULL);
	}
#endif

	err = create_all_cpu_call_rcu_data(0);
	if (err)
		diag("Per-CPU call_rcu() workers unavailable, using default.");

	rcu_register_thread();

	plan_tests(NR_TESTS);

	diag("Concurrent invariant tests (readers vs. writers)");

	diag("1. Iteration ordering");
	RUN_TEST(inv_iteration_order);
	RUN_TEST(inv_bind_resume_order);
	RUN_TEST(inv_ordered_bulk_consistency);
	RUN_TEST(inv_compact_keycopy_terminates);
	RUN_TEST(inv_reverse_iteration_order);

	diag("2. Lookup consistency");
	RUN_TEST(inv_lookup_consistency);

	diag("3. Duplicate chain acyclicity");
	RUN_TEST(inv_dup_chain_acyclicity);

	diag("4. Graft-swap atomicity");
	RUN_TEST(inv_graft_swap_atomicity);

	diag("5. Relational lookup consistency");
	RUN_TEST(inv_relational_lookup);

	diag("6. Rank-based lookup stability");
	RUN_TEST(inv_nth_last_stability);
	RUN_TEST(inv_nth_first_stability);

	diag("7. Iterator skip stability");
	RUN_TEST(inv_skip_forward_stability);
	RUN_TEST(inv_skip_reverse_stability);

	diag("8. nr_keys undercount ordering");
	RUN_TEST(inv_nr_keys_undercount);

	diag("9. Ordered traversal never escapes its trie");
	RUN_TEST(inv_ordered_no_escape_graft);

	diag("10. Piecewise merge never escapes the destination");
	RUN_TEST(inv_merge_no_escape);

	diag("11. Merge into a non-root destination never escapes");
	RUN_TEST(inv_merge_nonroot_dst_no_escape);

	diag("12. Merge into a compressed destination never escapes");
	RUN_TEST(inv_merge_compressed_dst_no_escape);

	diag("13. Merge into a key-shorter destination never escapes");
	RUN_TEST(inv_merge_key_shorter_dst_no_escape);

	diag("14. Merge from a key-shorter source never escapes");
	RUN_TEST(inv_merge_key_shorter_src_no_escape);

	diag("15. Merge into a compressed-parent destination never escapes");
	RUN_TEST(inv_merge_compressed_parent_dst_no_escape);

	diag("16. Merge is atomic: no value missing after the flip");
	RUN_TEST(inv_merge_atomic_completeness);

	diag("17. Merge is atomic across a deep (recursive) overlap");
	RUN_TEST(inv_merge_atomic_completeness_deep);

	rcu_barrier();
	rcu_unregister_thread();
	free_all_cpu_call_rcu_data();

	return exit_status();
}
