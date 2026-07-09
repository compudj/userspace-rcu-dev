// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

/*
 * Unit test for <urcu/rcu-txn-skiplist.h>.
 *
 *  1. Deterministic single-thread: out-of-order insert, membership (present /
 *     absent), duplicate rejection, ordered iteration, fsck of the multi-level
 *     structure, delete, re-check.
 *  2. Concurrent insert/delete on one shared skiplist with reader threads that
 *     walk the level-0 chain and assert it stays strictly key-ordered (every
 *     MCAS commit preserves the sorted invariant atomically, so a reader must
 *     NEVER observe an out-of-order pair).
 *  3. Move conservation: movers atomically move keys between two skiplists (one
 *     composed del+insert txn each); afterward every key must appear in exactly
 *     one list -- none lost (the orphaned-node bug the per-level mark prevents),
 *     none duplicated.
 */

#define _GNU_SOURCE
#define _LGPL_SOURCE
#include <assert.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include <urcu/compiler.h>
#include <urcu-qsbr.h>
#include <urcu-call-rcu.h>
#include <urcu/rcu-txn-skiplist.h>
#include <urcu/rcu-txn.h>

#include "tap.h"

#define NR_TESTS	12
#ifndef KEY_MAX
#define KEY_MAX		512
#endif
#ifndef NR_WRITERS
#define NR_WRITERS	4
#endif
#ifndef NR_READERS
#define NR_READERS	2
#endif
#ifndef WRITER_OPS
#define WRITER_OPS	30000
#endif
#ifndef MOVE_KEYS
#define MOVE_KEYS	512
#endif
#ifndef NR_MOVERS
#define NR_MOVERS	4
#endif
#ifndef MOVER_OPS
#define MOVER_OPS	30000
#endif
#define STEP_LIMIT	65536		/* runaway-walk guard (>> equilibrium size) */

struct node {
	unsigned long key;
	struct rcu_head rh;
	struct urcu_txn_skiplist_node sl;	/* last: flexible next[] */
};

static struct urcu_txn_domain g_dom;	/* one domain shared by all skiplists */
static volatile int g_stop;

static int node_cmp(struct urcu_txn_skiplist_node *n, void *key)
{
	unsigned long a = caa_container_of(n, struct node, sl)->key;
	unsigned long b = *(unsigned long *) key;

	return (a > b) - (a < b);
}

static unsigned long key_of(struct urcu_txn_skiplist_node *n)
{
	return caa_container_of(n, struct node, sl)->key;
}

static void node_free_cb(struct rcu_head *h)
{
	free(caa_container_of(h, struct node, rh));
}

static struct node *node_alloc(unsigned long key, unsigned int toplevel)
{
	struct node *e = (struct node *) malloc(sizeof(*e)
			+ (toplevel + 1) * sizeof(struct urcu_txn_skiplist_node *));

	if (!e)
		abort();
	e->key = key;
	urcu_txn_skiplist_node_init(&e->sl, toplevel);
	return e;
}

static unsigned int xs(unsigned int x)
{
	x ^= x << 13;
	x ^= x >> 17;
	x ^= x << 5;
	return x;
}

/*
 * Per-CPU call_rcu worker pinned to @cpu, so each mutator's reclaim runs on its
 * own worker instead of funneling through the single default worker (the
 * benchmarks do the same).  Uses the SLEEPING flavor (flags 0), NOT
 * URCU_CALL_RCU_RT: an RT worker busy-spins assuming a dedicated core, which on
 * a shared/loaded box preempts the mutators and deadlocks call_rcu_data_free()
 * (it drains its queue on a grace period a starved peer can never advance).  The
 * throughput benchmark, which owns its cores, uses the RT flavor.
 */
#ifndef CRDP_FLAGS
#define CRDP_FLAGS	0	/* sleeping per-CPU worker; -DCRDP_FLAGS=URCU_CALL_RCU_RT for RT */
#endif
static struct call_rcu_data *worker_setup(int cpu)
{
#ifdef NO_PERCPU_WORKER
	(void) cpu;
	return NULL;			/* use the default call_rcu worker */
#else
	struct call_rcu_data *crdp = create_call_rcu_data(CRDP_FLAGS, cpu);

	if (crdp)
		set_thread_call_rcu_data(crdp);
	return crdp;
#endif
}

static void worker_teardown(struct call_rcu_data *crdp)
{
	set_thread_call_rcu_data(NULL);
	if (crdp)
		call_rcu_data_free(crdp);	/* drains this worker's queue */
}

/*
 * Validate the multi-level structure (adapted from perfbook skiplist_fsck):
 * every level-L forward link lands on a level-0-downstream node tall enough to
 * carry it.  Single-threaded (call with no concurrent mutators).
 */
static void fsck(struct urcu_txn_skiplist *sl)
{
	struct urcu_txn_skiplist_node *first;

	for (first = sl->head; first != NULL;
			first = urcu_txn_skiplist_next_rcu(first, 0)) {
		unsigned int i;
		struct urcu_txn_skiplist_node *s =
				urcu_txn_skiplist_next_rcu(first, 0);

		for (i = 1; i <= first->toplevel; i++) {
			struct urcu_txn_skiplist_node *tgt =
					urcu_txn_skiplist_next_rcu(first, i);
			while (s != tgt) {
				assert(s != NULL);
				s = urcu_txn_skiplist_next_rcu(s, 0);
			}
			assert(s == NULL || s->toplevel >= i);
		}
	}
}

/* Level-0 node count; asserts strictly ascending order (single-threaded). */
static int count_sorted(struct urcu_txn_skiplist *sl)
{
	struct urcu_txn_skiplist_node *n;
	unsigned long prev = 0;
	int have_prev = 0, count = 0;

	for (n = urcu_txn_skiplist_next_rcu(sl->head, 0); n != NULL;
			n = urcu_txn_skiplist_next_rcu(n, 0)) {
		unsigned long k = key_of(n);

		if (have_prev)
			assert(k > prev);
		prev = k;
		have_prev = 1;
		count++;
	}
	return count;
}

/* --------------------------------------------------------------------- */
/* 1. Deterministic single-thread coverage.                              */
/* --------------------------------------------------------------------- */

#define DET_N	2000

static int det_test(void)
{
	struct urcu_txn_skiplist sl;
	unsigned long r = 0x9e3779b97f4a7c15UL;
	unsigned long absent = DET_N + 12345;
	unsigned long *keys = (unsigned long *) malloc(DET_N * sizeof(*keys));
	int i, ok_all = 1;

	assert(!urcu_txn_skiplist_init(&sl, node_cmp));
	for (i = 0; i < DET_N; i++)
		keys[i] = (unsigned long) i;
	for (i = DET_N - 1; i > 0; i--) {		/* shuffle */
		int j;
		r = r * 6364136223846793005UL + 1442695040888963407UL;
		j = (int) ((r >> 33) % (unsigned) (i + 1));
		{ unsigned long t = keys[i]; keys[i] = keys[j]; keys[j] = t; }
	}
	for (i = 0; i < DET_N; i++) {
		unsigned int lvl;
		struct node *e;
		r = r * 6364136223846793005UL + 1442695040888963407UL;
		lvl = urcu_txn_skiplist_random_level(r >> 17);
		e = node_alloc(keys[i], lvl);
		if (urcu_txn_skiplist_add_rcu(&sl, &e->sl, &keys[i], &g_dom) != 0)
			ok_all = 0;
	}
	ok(count_sorted(&sl) == DET_N && ok_all,
		"det: %d out-of-order inserts, level-0 sorted", DET_N);
	fsck(&sl);
	ok(1, "det: fsck after inserts");

	{
		int present_all = 1;
		for (i = 0; i < DET_N; i++) {
			unsigned long k = (unsigned long) i;
			if (!urcu_txn_skiplist_lookup_rcu(&sl, &k))
				present_all = 0;
		}
		ok(present_all && !urcu_txn_skiplist_lookup_rcu(&sl, &absent),
			"det: all present, known-absent key not found");
	}
	{
		unsigned long k = 42;
		struct node *e = node_alloc(42, 3);
		int dup = urcu_txn_skiplist_add_rcu(&sl, &e->sl, &k, &g_dom);
		free(e);
		ok(dup == -EEXIST, "det: duplicate insert rejected (-EEXIST)");
	}
	/* Delete odds; evens remain. */
	for (i = 1; i < DET_N; i += 2) {
		unsigned long k = (unsigned long) i;
		struct urcu_txn_skiplist_node *rem = NULL;
		if (urcu_txn_skiplist_del_rcu(&sl, &k, &g_dom, &rem) == 1)
			free(caa_container_of(rem, struct node, sl));
	}
	{
		unsigned long k = 1;
		int again = urcu_txn_skiplist_del_rcu(&sl, &k, &g_dom, NULL);
		int membership = 1;
		for (i = 0; i < DET_N; i++) {
			unsigned long kk = (unsigned long) i;
			int present = urcu_txn_skiplist_lookup_rcu(&sl, &kk) != NULL;
			if (present != ((i % 2) == 0))
				membership = 0;
		}
		fsck(&sl);
		ok(again == 0 && membership && count_sorted(&sl) == DET_N / 2,
			"det: delete odds, evens remain, fsck ok, re-delete returns 0");
	}
	/* Clean up. */
	{
		struct urcu_txn_skiplist_node *n =
				urcu_txn_skiplist_next_rcu(sl.head, 0);
		while (n) {
			struct urcu_txn_skiplist_node *nx =
					urcu_txn_skiplist_next_rcu(n, 0);
			free(caa_container_of(n, struct node, sl));
			n = nx;
		}
	}
	urcu_txn_skiplist_destroy(&sl);
	free(keys);
	return 5;					/* number of ok() emitted */
}

/* --------------------------------------------------------------------- */
/* 2. Concurrent insert/delete + sorted-invariant readers.               */
/* --------------------------------------------------------------------- */

static struct urcu_txn_skiplist g_sl;		/* shared by writers + readers */

struct writer_stats { unsigned int seed; int cpu; long ops; };
struct reader_stats { long walks; long violations; };

static void *cc_writer(void *arg)
{
	struct writer_stats *st = (struct writer_stats *) arg;
	unsigned int rng = st->seed;
	struct call_rcu_data *crdp;
	long n;

	rcu_register_thread();
	crdp = worker_setup(st->cpu);
	for (n = 0; n < WRITER_OPS; n++) {
		unsigned long key;

		rng = xs(rng);
		key = 1 + (rng >> 1) % KEY_MAX;
		if (rng & 1) {
			unsigned int lvl = urcu_txn_skiplist_random_level(xs(rng));
			struct node *e = node_alloc(key, lvl);
			if (urcu_txn_skiplist_add_rcu(&g_sl, &e->sl, &key, &g_dom)
					== -EEXIST)
				free(e);		/* not linked: safe to free */
		} else {
			struct urcu_txn_skiplist_node *rem = NULL;
			if (urcu_txn_skiplist_del_rcu(&g_sl, &key, &g_dom, &rem) == 1)
				call_rcu(&caa_container_of(rem, struct node, sl)->rh,
						node_free_cb);
		}
		st->ops++;
		rcu_quiescent_state();
	}
	/* Leave the reader set BEFORE draining (call_rcu_data_free waits on a
	 * grace period): a registered thread blocked in the drain would stall the
	 * very grace period it waits for. */
	rcu_unregister_thread();
	worker_teardown(crdp);
	return NULL;
}

static void *cc_reader(void *arg)
{
	struct reader_stats *st = (struct reader_stats *) arg;

	rcu_register_thread();
	while (!CMM_LOAD_SHARED(g_stop)) {
		struct urcu_txn_skiplist_node *n;
		unsigned long prev = 0;
		int have_prev = 0, steps = 0;

		rcu_read_lock();
		for (n = urcu_txn_skiplist_next_rcu(g_sl.head, 0); n != NULL;
				n = urcu_txn_skiplist_next_rcu(n, 0)) {
			unsigned long k = key_of(n);

			/*
			 * A level-0 walk under one read-side section must observe
			 * strictly ascending keys: no duplicate and no backward link.
			 */
			if (have_prev && k <= prev)
				st->violations++;
			prev = k;
			have_prev = 1;
			if (++steps > STEP_LIMIT)
				break;
		}
		rcu_read_unlock();
		st->walks++;
		rcu_quiescent_state();
	}
	rcu_unregister_thread();
	return NULL;
}

static void concurrent_test(void)
{
	pthread_t wt[NR_WRITERS], rt[NR_READERS];
	struct writer_stats ws[NR_WRITERS];
	struct reader_stats rs[NR_READERS];
	long total_ops = 0, total_viol = 0, total_walks = 0;
	int i;

	assert(!urcu_txn_skiplist_init(&g_sl, node_cmp));
	g_stop = 0;
	for (i = 0; i < NR_READERS; i++) {
		rs[i].walks = rs[i].violations = 0;
		pthread_create(&rt[i], NULL, cc_reader, &rs[i]);
	}
	for (i = 0; i < NR_WRITERS; i++) {
		ws[i].seed = 0x1234 + i * 7919;
		ws[i].cpu = i;
		ws[i].ops = 0;
		pthread_create(&wt[i], NULL, cc_writer, &ws[i]);
	}
	rcu_thread_offline();			/* offline while blocked in join/barrier */
	for (i = 0; i < NR_WRITERS; i++) {
		pthread_join(wt[i], NULL);
		total_ops += ws[i].ops;
	}
	CMM_STORE_SHARED(g_stop, 1);
	for (i = 0; i < NR_READERS; i++) {
		pthread_join(rt[i], NULL);
		total_viol += rs[i].violations;
		total_walks += rs[i].walks;
	}
	rcu_barrier();				/* drain call_rcu reclaim */
	rcu_thread_online();			/* back online for the read-side audits */
	diag("%d writers did %ld ops; %d readers did %ld walks, %ld order violations",
		NR_WRITERS, total_ops, NR_READERS, total_walks, total_viol);

	ok(total_ops == (long) NR_WRITERS * WRITER_OPS,
		"concurrent: all writer ops completed");
	ok(total_viol == 0,
		"concurrent: readers never observed an out-of-order pair");
	ok(count_sorted(&g_sl) >= 0, "concurrent: final level-0 chain sorted");
	fsck(&g_sl);
	ok(1, "concurrent: fsck of final structure");

	{
		struct urcu_txn_skiplist_node *n =
				urcu_txn_skiplist_next_rcu(g_sl.head, 0);
		while (n) {
			struct urcu_txn_skiplist_node *nx =
					urcu_txn_skiplist_next_rcu(n, 0);
			free(caa_container_of(n, struct node, sl));
			n = nx;
		}
	}
	urcu_txn_skiplist_destroy(&g_sl);
}

/* --------------------------------------------------------------------- */
/* 3. Move conservation across two skiplists.                            */
/* --------------------------------------------------------------------- */

static struct urcu_txn_skiplist g_a, g_b;

/*
 * Atomically move @key from whichever of A/B holds it to the other, in one txn
 * (composed del + insert of a fresh node).  Returns 1 on move, 0 if @key was in
 * neither at commit time (a lost race -- caller may retry another key).
 */
static int move_key(unsigned long key, unsigned int lvl,
		struct urcu_txn_skiplist_node **reclaim)
{
	struct urcu_mcas_txn txn;
	struct node *nn = node_alloc(key, lvl);
	int result = 0;

	urcu_txn_init(&txn, &g_dom);
	for (;;) {
		struct urcu_txn_skiplist_node *rem = NULL;
		struct urcu_txn_skiplist *src, *dst;
		int pdel, pins;
		enum urcu_txn_status st;

		/*
		 * Report a quiescent state each iteration: this loop can spin under
		 * contention, and a registered QSBR thread that retries without ever
		 * being quiescent stalls every grace period (deadlocking call_rcu
		 * reclaim).  Safe here -- we are outside any read-side section (the
		 * previous attempt ended with urcu_txn_end).
		 */
		rcu_quiescent_state();
		urcu_txn_begin(&txn);
		pdel = urcu_txn_skiplist_del_prepare(&txn, &g_a, &key, &rem);
		if (pdel == 0) {
			src = &g_a; dst = &g_b;
		} else if (pdel == -ENOENT) {
			pdel = urcu_txn_skiplist_del_prepare(&txn, &g_b, &key, &rem);
			src = &g_b; dst = &g_a;
		} else {
			src = NULL; dst = NULL;		/* -EAGAIN handled below */
		}
		(void) src;
		if (pdel == -EAGAIN) {
			urcu_txn_conflict(&txn); urcu_txn_end(&txn); continue;
		}
		if (pdel == -ENOENT) {			/* in neither list */
			urcu_txn_end(&txn);
			result = 0;
			break;
		}
		pins = urcu_txn_skiplist_insert_prepare(&txn, dst, &nn->sl, &key);
		if (pins == -EAGAIN || pins == -EEXIST) {
			/*
			 * -EAGAIN: a neighbour is mid-delete.  -EEXIST: a racing mover
			 * already placed @key into @dst, which means our del of @src is
			 * stale and would abort at commit anyway -- retry with fresh
			 * searches (both are legal concurrent outcomes, not errors).
			 */
			urcu_txn_conflict(&txn); urcu_txn_end(&txn); continue;
		}
		assert(pins == 0);
		st = urcu_txn_commit(&txn);
		urcu_txn_end(&txn);
		if (st == URCU_TXN_STATUS_ABORT)
			continue;
		assert(st == URCU_TXN_STATUS_OK);
		*reclaim = rem;
		result = 1;
		break;
	}
	if (!result)
		free(nn);				/* never linked */
	return result;
}

struct mover_stats { unsigned int seed; int cpu; long moves; };

static void *mover(void *arg)
{
	struct mover_stats *st = (struct mover_stats *) arg;
	unsigned int rng = st->seed;
	struct call_rcu_data *crdp;
	long n;

	rcu_register_thread();
	crdp = worker_setup(st->cpu);
	for (n = 0; n < MOVER_OPS; n++) {
		struct urcu_txn_skiplist_node *reclaim = NULL;
		unsigned long key;
		unsigned int lvl;

		rng = xs(rng);
		key = rng % MOVE_KEYS;
		lvl = urcu_txn_skiplist_random_level(xs(rng));
		if (move_key(key, lvl, &reclaim)) {
			st->moves++;
			if (reclaim)
				call_rcu(&caa_container_of(reclaim, struct node, sl)->rh,
						node_free_cb);
		}
		rcu_quiescent_state();
	}
	rcu_unregister_thread();		/* leave reader set before draining */
	worker_teardown(crdp);
	return NULL;
}

static void collect_into(struct urcu_txn_skiplist *sl, unsigned char *seen,
		int *dup, int *count)
{
	struct urcu_txn_skiplist_node *n;

	for (n = urcu_txn_skiplist_next_rcu(sl->head, 0); n != NULL;
			n = urcu_txn_skiplist_next_rcu(n, 0)) {
		unsigned long k = key_of(n);

		assert(k < MOVE_KEYS);
		if (seen[k])
			(*dup)++;
		seen[k] = 1;
		(*count)++;
	}
}

static void move_test(void)
{
	pthread_t mt[NR_MOVERS];
	struct mover_stats ms[NR_MOVERS];
	unsigned char *seen = (unsigned char *) calloc(MOVE_KEYS, 1);
	long total_moves = 0;
	int i, dup = 0, total = 0, missing = 0;

	assert(!urcu_txn_skiplist_init(&g_a, node_cmp));
	assert(!urcu_txn_skiplist_init(&g_b, node_cmp));
	/* Preload A with [0, MOVE_KEYS). */
	for (i = 0; i < MOVE_KEYS; i++) {
		unsigned long k = (unsigned long) i;
		unsigned int lvl = urcu_txn_skiplist_random_level(xs(0x51ed + i));
		struct node *e = node_alloc(k, lvl);
		assert(urcu_txn_skiplist_add_rcu(&g_a, &e->sl, &k, &g_dom) == 0);
	}
	for (i = 0; i < NR_MOVERS; i++) {
		ms[i].seed = 0xC0FFEE + i * 40503;
		ms[i].cpu = i;
		ms[i].moves = 0;
		pthread_create(&mt[i], NULL, mover, &ms[i]);
	}
	rcu_thread_offline();			/* offline while blocked in join/barrier */
	for (i = 0; i < NR_MOVERS; i++) {
		pthread_join(mt[i], NULL);
		total_moves += ms[i].moves;
	}
	rcu_barrier();
	rcu_thread_online();			/* back online for the read-side audits */
	fsck(&g_a);
	ok(1, "move: fsck of list A after concurrent moves");
	fsck(&g_b);
	ok(1, "move: fsck of list B after concurrent moves");

	collect_into(&g_a, seen, &dup, &total);
	collect_into(&g_b, seen, &dup, &total);
	for (i = 0; i < MOVE_KEYS; i++)
		if (!seen[i])
			missing++;
	diag("%d movers did %ld moves; A+B total=%d dup=%d missing=%d (expect %d/0/0)",
		NR_MOVERS, total_moves, total, dup, missing, MOVE_KEYS);
	ok(total == MOVE_KEYS && dup == 0 && missing == 0,
		"move: conservation -- every key present exactly once, none lost/duplicated");

	{
		struct urcu_txn_skiplist *lists[2] = { &g_a, &g_b };
		int li;
		for (li = 0; li < 2; li++) {
			struct urcu_txn_skiplist_node *n =
					urcu_txn_skiplist_next_rcu(lists[li]->head, 0);
			while (n) {
				struct urcu_txn_skiplist_node *nx =
						urcu_txn_skiplist_next_rcu(n, 0);
				free(caa_container_of(n, struct node, sl));
				n = nx;
			}
		}
	}
	urcu_txn_skiplist_destroy(&g_a);
	urcu_txn_skiplist_destroy(&g_b);
	free(seen);
}

int main(void)
{
	plan_tests(NR_TESTS);
	urcu_txn_domain_init(&g_dom);

	/*
	 * Main is registered throughout -- it uses the _rcu skiplist accessors
	 * (as sole mutator in the deterministic section, and in the post-join
	 * audits).  But it must go rcu_thread_offline() around the blocking waits
	 * (pthread_join, rcu_barrier): a registered online thread blocked there
	 * never reports a quiescent state and would stall every grace period,
	 * deadlocking the workers' call_rcu drain.  concurrent_test()/move_test()
	 * bracket their waits accordingly.
	 */
	rcu_register_thread();
	det_test();			/* 5 tests */
#ifndef SKIP_CONCURRENT
	concurrent_test();		/* 4 tests */
#endif
	move_test();			/* 3 tests */
	rcu_unregister_thread();

	return exit_status();
}
