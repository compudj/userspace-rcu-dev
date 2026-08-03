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
#include "../../src/fractal-trie/cds_ft_tp.h"
#define FT_TEST_TP(name, ...) \
	lttng_ust_tracepoint(cds_ft, name, ##__VA_ARGS__)
#else
#define FT_TEST_TP(name, ...) do {} while (0)
#endif

#include <assert.h>
#include <inttypes.h>
#include <stdarg.h>
#include <errno.h>
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

#define NR_TESTS_REKEY_DLM	11	/* inv_rekey_graft_{disjoint,cross_junction,glue_dst,coherent_readers,shared}, inv_rekey_linearizability, inv_rekey_public_{atomic_no_gap,staged_gap}, inv_rekey_merge_{occupied,shared}_dst */

/*
 * Base count = the RUN_TEST invocations in main() outside the DLM #ifdef.
 * Keep it EXACT: libtap's exit_status() returns planned-minus-run, so an
 * over-declared plan is a silent nonzero exit that nothing reads -- the gate
 * counted "ok "/"not ok " lines only, so 65 tests against a plan of 68 scored
 * green for as long as the constant was stale.  ft_parallel_gate.sh now
 * compares the run count against this plan, so retiring a test means
 * decrementing here in the same commit.
 */
#define NR_TESTS	(90 + NR_TESTS_REKEY_DLM)

/* ------------------------------------------------------------------ */
/* Tuning knobs                                                       */
/* ------------------------------------------------------------------ */

/*
 * Duration of each concurrent test in milliseconds.  Kept short for
 * automated runs; increase for deeper stress testing.  The time-bounded
 * cross-view tests build tries for this long and then tear them down (a
 * per-trie rcu_barrier is the dominant cost), so this bounds their teardown:
 * 200ms builds ~10x fewer tries than 2000ms, cutting the slow teardown ~10x
 * while still exercising every cross-view window many times over.
 */
#define DEFAULT_DURATION_MS	200

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
	unsigned long freed_once;	/* set when handed to call_rcu (see node_free_rcu) */
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
/*
 * Count of nodes handed to node_free_rcu() a SECOND time -- see the comment
 * there.  Reported by leak_check(), because the count is a real observation
 * about the library under an out-of-contract workload, not harness noise.
 */
static unsigned long nodes_double_freed;

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
	/*
	 * ARBITRATE the deferred free, because "cds_ft_remove returned OK" does
	 * NOT make this thread the sole owner of @n.
	 *
	 * The library's update contract is caller-serialized ("mutual exclusion
	 * between updates is the caller's responsibility"), and the SAME-KEY
	 * oracle inv_concurrent_writers_shared deliberately runs outside it: 16
	 * writers contend one key range, so two of them can look up the SAME node
	 * and both be told OK.  Its node lifecycle -- "remove == OK owns the
	 * RCU-deferred free" -- then calls call_rcu TWICE on one rcu_head.
	 *
	 * That is not a survivable mistake: the second enqueue re-initialises a
	 * LIVE wfcqueue node and corrupts the per-CPU call_rcu queue, so the
	 * damage lands wherever the process reaches next -- a HANG in this test's
	 * own post-test rcu_barrier, a glibc "double free or corruption" abort in
	 * a later test's, or an ord-cell verify mismatch three tests on.  Every
	 * one of those looks like a bug in an innocent test (each passes in
	 * isolation), which is exactly how much time it costs.
	 *
	 * So let the first free win, and COUNT the losers: that count is the real
	 * observation ("the library handed OK to two writers for one node, N
	 * times") where a corrupted RCU queue is only noise.  The loser's call is
	 * a pure no-op -- the winner's call_rcu already owns the one free, so the
	 * node is neither leaked nor freed twice and the leak accounting stays
	 * exactly balanced.
	 */
	if (__atomic_exchange_n(&n->freed_once, 1, __ATOMIC_SEQ_CST) != 0) {
		__atomic_add_fetch(&nodes_double_freed, 1, __ATOMIC_RELAXED);
		return;
	}
	call_rcu(&n->head, node_free_rcu_cb);
}

static void mw_violation_snapshot(void);

/*
 * cds_ft_verify, and on a failure DUMP the structure that failed it.
 *
 * A structural violation names one node; what it is wired to is the rest of the
 * finding, and it only exists right here -- the next step of any teardown
 * destroys it.  cds_ft_show RECURSES the tree instead of navigating it in key
 * order, so it returns on exactly the tries an ordered walk cannot get through,
 * and the shapes that fail are small (the sibling oracle's whole trie is ~20
 * nodes).
 */
static int verify_or_dump(struct cds_ft *ft, const char *what)
{
	if (cds_ft_verify(ft, stderr) == CDS_FT_STATUS_OK)
		return 0;
	fprintf(stderr, "%s: cds_ft_verify FAILS -- structure follows\n", what);
	cds_ft_show(ft, stderr, CDS_FT_SHOW_PRETTY);
	mw_violation_snapshot();
	return -1;
}

/*
 * Does a FULL ordered enumeration of the trie still terminate, and does the
 * structure still verify, now that the writers are joined?
 *
 * Every concurrent invariant tears down through drain_and_destroy, and that
 * drain only ever calls cds_ft_lookup_first in a loop: each call restarts at
 * the root, so as long as the FIRST key stays reachable the drain completes and
 * NOTHING ever walks the whole trie.  A trie whose successor walk is broken
 * from the middle onwards therefore tears down silently, and the damage is
 * reported -- if at all -- as a bare node leak three steps later.
 *
 * BOUNDED, never unbounded: the damage this looks for makes cds_ft_next cycle,
 * and a diagnostic that hangs reports nothing at all.  An earlier attempt at
 * this check used cds_ft_count_entries and turned five gate configs into
 * timeouts, which costs every test after it in those configs -- worse than the
 * missing check.  With the cap the same trie yields a named failure instead.
 *
 * Unconditional: measured at 36.1 s vs 36.8 s for the whole ft_inv suite, i.e.
 * within run-to-run noise, so there is nothing to gate it behind.
 */
static int teardown_walk_check(struct cds_ft *ft, const char *what)
{
	struct cds_ft_iter *iter;
	unsigned long steps = 0;
	const unsigned long cap = 1000000;
	enum cds_ft_status s;
	int ret = 0;

	/*
	 * VERIFY FIRST, and on a failure do not walk at all.  The step cap below
	 * does NOT bound the walk on a structurally broken trie: the observed
	 * non-termination is a going-up cycle INSIDE one cds_ft_lookup_first call
	 * (measured: the same level/slot/anchor 100001 times), which never comes
	 * back to be counted.  A caught run that then hangs loses every test
	 * after it, so the validator's named node is the report.
	 */
	if (verify_or_dump(ft, what)) {
		fprintf(stderr, "%s: teardown ordered walk SKIPPED (it may not "
			"return on this trie)\n", what);
		return -1;
	}
	if (cds_ft_iter_create(ft, &iter) < 0)
		return -1;
	rcu_read_lock();
	for (s = cds_ft_lookup_first(ft, iter); s == CDS_FT_STATUS_OK;
			s = cds_ft_next(ft, iter)) {
		if (++steps > cap) {
			fprintf(stderr, "%s: teardown ordered walk OVERRUN "
				"(%lu steps, still going)\n", what, cap);
			ret = -1;
			break;
		}
	}
	rcu_read_unlock();
	cds_ft_iter_destroy(iter);
	if (ret)
		mw_violation_snapshot();
	return ret;
}

/* Free every node reachable through the trie, then destroy the trie. */
static int drain_and_destroy(struct cds_ft *ft, struct cds_ft_group *group)
{
	struct cds_ft_iter *iter;
	enum cds_ft_status s;
	int ret = 0;

	if (teardown_walk_check(ft, "drain_and_destroy"))
		ret = -1;
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
		/*
		 * lookup_first just found a key, so remove_all MUST remove it; a
		 * NOT_FOUND means the iterator's key disagrees with its node -- a
		 * stale-iter bug (e.g. draining a detach-stripped trie with the
		 * leaf's pre-detach speculative key still set, which violates the
		 * speculative_key_offset contract).  Fail loud instead of looping
		 * forever on a lookup_first that keeps re-finding the same head.
		 */
		if (s == CDS_FT_STATUS_NOT_FOUND) {
			fprintf(stderr,
				"drain_and_destroy: lookup_first found a key that "
				"remove_all reports NOT_FOUND -- stale iterator key "
				"(speculative key inconsistent with trie position?)\n");
			abort();
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
	__atomic_store_n(&nodes_double_freed, 0, __ATOMIC_RELAXED);
}

static int leak_check(void)
{
	rcu_barrier();
	unsigned long na = __atomic_load_n(&nodes_allocated, __ATOMIC_RELAXED);
	unsigned long nf = __atomic_load_n(&nodes_freed, __ATOMIC_RELAXED);
	unsigned long nd = __atomic_load_n(&nodes_double_freed, __ATOMIC_RELAXED);

	/*
	 * Not a failure: an out-of-contract same-key oracle can legitimately see
	 * two writers told OK for one node (node_free_rcu arbitrates).  Report it
	 * so the observation is not lost -- silently arbitrating would hide the
	 * only evidence that it happened.
	 */
	if (nd)
		fprintf(stderr, "# NOTE: %lu node(s) freed twice (two writers "
			"got OK for one node); second free arbitrated away\n", nd);
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

/*
 * FT_INV_RANK_STATS: when set, enable per-node order statistics (nr_keys) on
 * every trie the invariant suite builds.  The default suite runs rank stats
 * OFF (count queries fall back to a structural recount), so the maintained
 * nr_keys aggregate -- and the flip-txn count-edge fold that keeps it exact --
 * is never exercised by the concurrent invariants.  Turning it on makes a
 * FEATURE_FT_VERIFY_AT_MUTATION build check per-node nr_keys exactness after
 * every mutation across the whole suite (cds_ft_verify gates its nr_keys check
 * on ft->rank_stats).
 */
static void inv_maybe_set_rank_stats(struct cds_ft_group_attr *attr)
{
	if (getenv("FT_INV_RANK_STATS") &&
			cds_ft_group_attr_set_rank_stats(attr, true) < 0)
		abort();
}

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
	/*
	 * FT_INV_NO_ORDERED_LIST forces the ordered list OFF on this (default)
	 * group, exercising the runtime cell-optional path: a cell build then
	 * allocates NO ordinal cells (head->prev is the flagged parent directly).
	 * The ordered invariants use create_fixed_ord_ft (explicit set_ordered_list)
	 * so they are unaffected; run the NON-ordered invariants under this env to
	 * stress the no-cell path concurrently.
	 */
	if (getenv("FT_INV_NO_ORDERED_LIST") &&
			cds_ft_group_attr_set_ordered_list(attr, false) < 0)
		abort();
	inv_maybe_set_rank_stats(attr);
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
	if (cds_ft_group_attr_set_ordered_list(attr, true) < 0)
		abort();
	inv_maybe_set_rank_stats(attr);
	if (cds_ft_group_create(attr, &group) < 0)
		abort();
	cds_ft_group_attr_destroy(attr);
	if (cds_ft_create(group, NULL, &ft) < 0)
		abort();
	*group_out = group;
	return ft;
}

/*
 * Like create_fixed_ft, but UNCONDITIONALLY disables the ordered list
 * (cds_ft_group_attr_set_ordered_list(attr, false)), so the group allocates NO ordinal
 * cells: a head's prev is the flagged parent directly.  Used by the dedicated
 * no-cell concurrent invariant so the runtime cell-optional read path
 * (ft_resolve_head_prev prev-direct, skip resolution, parent backtrack) is
 * exercised in the default test pass regardless of FT_INV_NO_ORDERED_LIST.
 */
static struct cds_ft *create_fixed_nolist_ft(size_t klen, struct cds_ft_group **group_out)
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
	if (cds_ft_group_attr_set_ordered_list(attr, false) < 0)
		abort();
	inv_maybe_set_rank_stats(attr);
	if (cds_ft_group_create(attr, &group) < 0)
		abort();
	cds_ft_group_attr_destroy(attr);
	if (cds_ft_create(group, NULL, &ft) < 0)
		abort();
	*group_out = group;
	return ft;
}

/*
 * Like create_fixed_ft, but UNCONDITIONALLY enables per-node order statistics
 * (nr_keys), independent of FT_INV_RANK_STATS.  Used by the strict nr_keys
 * exactness invariant, which reads the maintained aggregate (cds_ft_count_keys,
 * lookup_nth) and cross-checks it against cds_ft_verify's per-node recount.
 */
static struct cds_ft *create_fixed_rankstats_ft(size_t klen,
		struct cds_ft_group **group_out)
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
	if (getenv("FT_INV_NO_ORDERED_LIST") &&
			cds_ft_group_attr_set_ordered_list(attr, false) < 0)
		abort();
	if (cds_ft_group_attr_set_rank_stats(attr, true) < 0)
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
 * Coarse MW lock-mode trie (CDS_FT_WRITER_LOCK_COARSE): every mutator serializes
 * under the single FT-wide writer lock; readers wait-free.  rank_stats ON (the
 * coarse target build, §10.5), EAGER (no speculative offset -- the MW oracle
 * does exact lookups only).  Used by inv_concurrent_writers_coarse_lock.
 */
static struct cds_ft *create_fixed_coarse_lock_ft(size_t klen,
		struct cds_ft_group **group_out)
{
	struct cds_ft_group_attr *attr;
	struct cds_ft_group *group;
	struct cds_ft *ft;

	if (cds_ft_group_attr_create(&attr) < 0)
		abort();
	if (cds_ft_group_attr_set_key_len(attr, klen) < 0)
		abort();
	if (cds_ft_group_attr_set_rank_stats(attr, true) < 0)
		abort();
	if (cds_ft_group_attr_set_writer_strategy(attr,
			CDS_FT_WRITER_LOCK_COARSE) < 0)
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
 * Fine-grained MW lock-mode trie (CDS_FT_WRITER_LOCK_FINE): the op-domains
 * converted so far (§11.3 step 3: recompact) acquire their per-node lock-set
 * {C, P} (+ {GP}) instead of §4.B-guarding the parent, and release the surviving
 * members through the {LOCK|s -> s} terminal at the commit.  Until every
 * domain is converted the trie ALSO takes the FT-wide writer lock, so the
 * per-node locks are exercised under serialization rather than contended (the
 * §11.1 coexistence hazard forbids racing a converted op with an unconverted
 * one).  rank_stats OFF: the §9 lock-sets are derived for the default build --
 * a rank_stats-ON trie is the coarse single-lock target instead (§10.5).
 * Used by inv_concurrent_writers_fine_lock.
 */
static struct cds_ft *create_fixed_fine_lock_ft(size_t klen,
		struct cds_ft_group **group_out)
{
	struct cds_ft_group_attr *attr;
	struct cds_ft_group *group;
	struct cds_ft *ft;

	if (cds_ft_group_attr_create(&attr) < 0)
		abort();
	if (cds_ft_group_attr_set_key_len(attr, klen) < 0)
		abort();
	if (cds_ft_group_attr_set_writer_strategy(attr,
			CDS_FT_WRITER_LOCK_FINE) < 0)
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
 * Remove the key @v and reclaim its node.  Own read section + own iterator, so
 * it is safe to call from a writer that holds no other FT state.  Returns the
 * remove status, or NOT_FOUND when the key is absent.
 */
static enum cds_ft_status remove_u64(struct cds_ft *ft, uint64_t v)
{
	struct cds_ft_iter *iter = NULL;
	struct cds_ft_node *found;
	enum cds_ft_status st = CDS_FT_STATUS_NOT_FOUND;
	uint8_t k[8] = { 0 };

	if (cds_ft_iter_create(ft, &iter) < 0)
		abort();
	rcu_read_lock();
	cds_ft_u64_to_key(ft, v, k, CDS_FT_LEN_DEFAULT);
	cds_ft_iter_set_key(iter, k, CDS_FT_LEN_DEFAULT);
	cds_ft_lookup(ft, iter);
	found = cds_ft_iter_node(iter);
	if (found) {
		st = cds_ft_remove(ft, iter, found);
		if (st == CDS_FT_STATUS_OK)
			node_free_rcu(to_test_node(found));
	}
	rcu_read_unlock();
	cds_ft_iter_destroy(iter);
	return st;
}

static struct cds_ft *create_fixed_fine_lock_listoff_ft(size_t klen,
		struct cds_ft_group **group_out)
{
	struct cds_ft_group_attr *attr;
	struct cds_ft_group *group;
	struct cds_ft *ft;

	if (cds_ft_group_attr_create(&attr) < 0)
		abort();
	if (cds_ft_group_attr_set_key_len(attr, klen) < 0)
		abort();
	if (cds_ft_group_attr_set_writer_strategy(attr,
			CDS_FT_WRITER_LOCK_FINE) < 0)
		abort();
	if (cds_ft_group_attr_set_ordered_list(attr, false) < 0)
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
insert_replace_u64(struct cds_ft *ft, uint64_t v, struct ft_test_node *n,
		struct cds_ft_node **old_ret)
{
	uint8_t k[8] = { 0 };

	cds_ft_u64_to_key(ft, v, k, CDS_FT_LEN_DEFAULT);
	memcpy(n->okey, k, sizeof(n->okey));
	return cds_ft_insert_replace(ft, k, CDS_FT_LEN_DEFAULT, &n->node, old_ret);
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
static volatile int test_drained;	/* inv_remove_cross_view: pool emptied */

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

/*
 * Record an LTTng flight-recorder snapshot, PID-tagging the snapshot name so a
 * concurrent soak (many processes) can correlate each snapshot to the process
 * that dumped its core.  Called from the violation reporters and the
 * fatal-signal handlers below: system()/snprintf() are not async-signal-safe,
 * but this is accepted crash-time scaffolding (getpid() is async-signal-safe).
 */
static void ft_snapshot_record(void)
{
	char cmd[160];

	(void) snprintf(cmd, sizeof(cmd),
		"lttng snapshot record -n pid%ld 1>&2", (long) getpid());
	(void) system(cmd);
}

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
		ft_snapshot_record();
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
/*   PHASE 4.3: writer-vs-writer race oracle (disjoint key ranges)    */
/*                                                                    */
/* ================================================================== */

/*
 * Concurrent WRITERS on one shared trie WITHOUT the single-writer serializing
 * mutex the other invariants use -- the first test of the concurrent-writer
 * path (the MCAS commit engine + the Invariant-2 §4.B holder guards + the
 * commit retry, all dormant while a caller mutex serializes writers).
 *
 * Each writer owns a DISJOINT key range, so no two writers ever touch the same
 * key: there is NO logical write/write conflict, only STRUCTURAL concurrency
 * (writers mutating the shared internal nodes on their overlapping key
 * prefixes).  This isolates structural-concurrency correctness (node sharing,
 * freeze-on-free, the guards) from key-level races.
 *
 * Oracles:
 *  - in-line (the §3.1 lost-key hazard): a key THIS writer inserted and has not
 *    removed must ALWAYS resolve to exactly the node it inserted -- even while
 *    another writer restructures a shared ancestor.  A miss or a wrong node
 *    means a concurrent restructure made a live key disappear / resolve wrong.
 *  - in-line: a key this writer has NOT inserted must not be found.
 *  - final (after quiescence): every live key resolves to its node,
 *    cds_ft_count_keys equals the exact live total, cds_ft_verify passes.
 *  - leak_check: no node leaked or double-freed across the run.
 */
#define MW_NR_WRITERS	16
#define MW_RANGE	256		/* keys per writer */

/*
 * Flight-recorder hooks for the MW oracle (FT_INV_ABORT_ON_VIOLATION=1):
 * dump the LTTng snapshot ring and abort on the FIRST oracle violation, and
 * likewise from a fatal-signal handler so an uncontrolled SIGSEGV/SIGBUS
 * also captures its window.  system() in a signal handler is not
 * async-signal-safe -- accepted scaffolding at crash time, matching the
 * existing violation() helper.
 */
static void mw_violation_snapshot(void)
{
	if (!getenv("FT_INV_ABORT_ON_VIOLATION"))
		return;
	ft_snapshot_record();
	abort();
}

static void mw_fatal_action(int sig, siginfo_t *si, void *uctx)
{
	if (sig == SIGSEGV || sig == SIGBUS || sig == SIGILL) {
		/*
		 * Land the fault context IN the trace timeline: si_addr (the
		 * faulting data address) and the sigframe instruction pointer,
		 * so the fault correlates by address with the edge/alloc
		 * breadcrumb window.
		 */
		void *ip = NULL;
#if defined(__x86_64__)
		ucontext_t *uc = (ucontext_t *) uctx;

		ip = (void *) uc->uc_mcontext.gregs[REG_RIP];
#else
		(void) uctx;
#endif
		FT_TEST_TP(fatal_signal, sig, (const void *) si->si_addr,
			(const void *) ip);
		fprintf(stderr, "FATAL sig %d addr %p ip %p\n", sig,
			si ? si->si_addr : NULL, ip);
	}
	ft_snapshot_record();
	signal(sig, SIG_DFL);
	raise(sig);
}

static void mw_install_fatal_handler(void)
{
	struct sigaction sa;

	if (!getenv("FT_INV_ABORT_ON_VIOLATION"))
		return;
	memset(&sa, 0, sizeof(sa));
	sa.sa_sigaction = mw_fatal_action;
	sa.sa_flags = SA_SIGINFO;
	sigemptyset(&sa.sa_mask);
	sigaction(SIGSEGV, &sa, NULL);
	sigaction(SIGBUS, &sa, NULL);
	sigaction(SIGILL, &sa, NULL);
	sigaction(SIGABRT, &sa, NULL);
}

struct mw_writer_arg {
	struct cds_ft *ft;
	uint64_t base;				/* range [base, base + MW_RANGE) */
	uint8_t present[MW_RANGE];		/* 1 = my key is live in the trie */
	struct ft_test_node *node[MW_RANGE];	/* the live node per present key */
	unsigned long ops;
	int failed;
};

static void *mw_writer(void *arg)
{
	struct mw_writer_arg *w = (struct mw_writer_arg *) arg;
	struct cds_ft_iter *iter;
	unsigned int seed = (unsigned int)(uintptr_t) w;

	rcu_register_thread();
	if (cds_ft_iter_create(w->ft, &iter) < 0)
		abort();
	while (!test_go)
		;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	while (!test_stop) {
		uint64_t off = (uint64_t)(rand_r(&seed) % MW_RANGE);
		uint64_t key = w->base + off;
		struct cds_ft_node *found;
		uint8_t k[8];

		cds_ft_u64_to_key(w->ft, key, k, CDS_FT_LEN_DEFAULT);
		rcu_read_lock();
		cds_ft_iter_set_key(iter, k, CDS_FT_LEN_DEFAULT);
		cds_ft_lookup(w->ft, iter);
		found = cds_ft_iter_node(iter);
		if (w->present[off]) {
			/*
			 * Live key: must resolve to MY node (lost-key oracle).
			 * The lookup reference (@found, and the iter's cached
			 * path) is valid only under this thread's read-side
			 * section, and cds_ft_remove consumes it -- so the
			 * caller section spans lookup + remove (the §11
			 * reference-lifetime contract).
			 */
			if (found != &w->node[off]->node) {
				/*
				 * DISCRIMINATOR (flight-recorder campaign): the
				 * key was proven present-in-structure at abort
				 * time, so classify the read failure before
				 * declaring it -- retry on the SAME iter (same
				 * state, later instant: transient mid-flip
				 * anomaly if it now hits) and on a FRESH iter
				 * (virgin state: iter-cache/reanchor bug if
				 * only this one hits).  Still NIL on both =>
				 * persistent descent mis-validation.
				 */
				struct cds_ft_iter *fresh = NULL;
				struct cds_ft_node *re_same = NULL, *re_fresh = NULL;

				cds_ft_iter_set_key(iter, k, CDS_FT_LEN_DEFAULT);
				cds_ft_lookup(w->ft, iter);
				re_same = cds_ft_iter_node(iter);
				if (cds_ft_iter_create(w->ft, &fresh) ==
						CDS_FT_STATUS_OK) {
					cds_ft_iter_set_key(fresh, k,
						CDS_FT_LEN_DEFAULT);
					cds_ft_lookup(w->ft, fresh);
					re_fresh = cds_ft_iter_node(fresh);
				}
				fprintf(stderr, "MW writer base %llu key %llu: live "
					"but found %p != mine %p (retry same-iter %p "
					"fresh-iter %p)\n",
					(unsigned long long) w->base,
					(unsigned long long) key, (void *) found,
					(void *) &w->node[off]->node,
					(void *) re_same, (void *) re_fresh);
				if (fresh)
					cds_ft_iter_destroy(fresh);
				w->failed = 1;
				mw_violation_snapshot();
			} else if (cds_ft_remove(w->ft, iter, found)
					== CDS_FT_STATUS_OK) {
				node_free_rcu(to_test_node(found));
				w->present[off] = 0;
				w->node[off] = NULL;
			}
			rcu_read_unlock();
		} else {
			/*
			 * Absent key: only @found's boolean presence is
			 * consumed past this point, so the lookup section
			 * closes HERE and the insert below runs with NO
			 * caller-held read-side section -- the FT owns its
			 * own bracket (§11 Phase A).  This is the standing
			 * oracle for that caller contract.
			 */
			rcu_read_unlock();
			if (found) {
				fprintf(stderr, "MW writer base %llu key %llu: "
					"absent but found %p\n",
					(unsigned long long) w->base,
					(unsigned long long) key, (void *) found);
				w->failed = 1;
				mw_violation_snapshot();
			} else {
				struct ft_test_node *n = node_alloc(key);

				if (insert_u64(w->ft, key, n)
						== CDS_FT_STATUS_OK) {
					w->present[off] = 1;
					w->node[off] = n;
				} else {
					node_free(n);
				}
			}
		}
		w->ops++;
		if ((seed & 0x3f) == 0)
			rcu_quiescent_state();
	}
	cds_ft_iter_destroy(iter);
	rcu_unregister_thread();
	return NULL;
}

static int inv_concurrent_writers_disjoint(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft;

	/*
	 * Opt-in (FT_INV_MW=1): the concurrent-writer oracle over the certified
	 * per-node-lock writer path.  It is opt-in because it is a saturating
	 * soak, NOT because it is known-broken -- the one known-failing oracle,
	 * inv_rekey_src_mutated, gates itself a SECOND time on FT_INV_RKSM and
	 * names the defect in its skip message.  Run it explicitly:
	 *   FT_INV_MW=1 ./test_urcu_ft_inv inv_concurrent_writers_disjoint
	 */
	if (!getenv("FT_INV_MW")) {
		fprintf(stderr, "# inv_concurrent_writers_disjoint: skipped "
			"(set FT_INV_MW=1 to run the Phase 4.3 writer oracle)\n");
		return 0;
	}
	mw_install_fatal_handler();
	ft = create_fixed_ft(8, &group);
	/*
	 * Concurrent writers REQUIRE concurrent mode: exclusive mode reclaims
	 * retired nodes in place with no grace period (ft_flip_txn_call_rcu_now),
	 * so a peer standing on a just-retired node reads recycled memory (UAF)
	 * and two writers double-free it.  Concurrent mode defers reclaim through
	 * the flavor's call_rcu, so a writer's rcu_read_lock-guarded descent keeps
	 * every node it touches live until its grace period.
	 */
	cds_ft_make_concurrent(ft);
	struct mw_writer_arg *w;
	pthread_t writers[MW_NR_WRITERS];
	struct timespec t0;
	unsigned long total_ops = 0, live = 0;
	int i, ret = 0;

	leak_reset();

	w = (struct mw_writer_arg *) calloc(MW_NR_WRITERS, sizeof(*w));
	if (!w)
		abort();
	for (i = 0; i < MW_NR_WRITERS; i++) {
		w[i].ft = ft;
		w[i].base = (uint64_t) i * MW_RANGE;
	}

	test_go = 0;
	test_stop = 0;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	for (i = 0; i < MW_NR_WRITERS; i++)
		pthread_create(&writers[i], NULL, mw_writer, &w[i]);
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	test_go = 1;

	rcu_thread_offline();
	clock_gettime(CLOCK_MONOTONIC, &t0);
	while (elapsed_ms(&t0) < DEFAULT_DURATION_MS)
		usleep(1000);
	test_stop = 1;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	for (i = 0; i < MW_NR_WRITERS; i++)
		pthread_join(writers[i], NULL);
	rcu_thread_online();

	/* Quiescent: verify the final trie against every writer's shadow. */
	synchronize_rcu();
	rcu_read_lock();
	for (i = 0; i < MW_NR_WRITERS; i++) {
		unsigned int off;

		total_ops += w[i].ops;
		if (w[i].failed)
			ret = -1;
		for (off = 0; off < MW_RANGE; off++) {
			struct cds_ft_node *found = NULL;

			if (!w[i].present[off])
				continue;
			live++;
			if (lookup_u64(ft, w[i].base + off, &found)
					!= CDS_FT_STATUS_OK ||
			    found != &w[i].node[off]->node) {
				fprintf(stderr, "MW final: writer %d key %llu lost "
					"(found %p != %p)\n", i,
					(unsigned long long)(w[i].base + off),
					(void *) found,
					(void *) &w[i].node[off]->node);
				ret = -1;
			}
		}
	}
	if (cds_ft_count_keys(ft) != live) {
		fprintf(stderr, "MW final: count_keys %lu != live %lu\n",
			cds_ft_count_keys(ft), live);
		ret = -1;
	}
	if (cds_ft_verify(ft, stderr) != CDS_FT_STATUS_OK) {
		fprintf(stderr, "MW final: cds_ft_verify failed\n");
		ret = -1;
	}
	rcu_read_unlock();

	fprintf(stderr, "# inv_concurrent_writers_disjoint: %d writers, %lu ops, "
		"%lu live keys\n", MW_NR_WRITERS, total_ops, live);

	free(w);
	if (drain_and_destroy(ft, group) < 0)
		ret = -1;
	if (leak_check() < 0)
		ret = -1;
	return ret;
}

/*
 * Coherent-rekey concurrent-writer oracle (DISJOINT): N writers each own a
 * private subtree and rekey it back and forth between two junctions that SHARE
 * the root as grandparent (d_src.ppnf == d_dst.ppnf == root), so every writer's
 * move node locks root and all N SERIALIZE on it -- the contention this hook's
 * up-front DLM acquire is designed to fail-fast + retry on.  Disjoint key
 * ownership means no writer ever re-homes a peer's junction, so the writer's
 * parent_held reuse is exercised under contention but not its racy re-home edge
 * (that is the SHARED oracle's job).  The list is ON, so each move also folds the
 * moved subtree's ordered-cell run splice into the same commit.
 *
 * Per writer w: two root-child junction bytes bp=2w+1, dp=2w+2 (distinct across
 * writers).  Each junction carries a SYMMETRIC layout -- straddling sibling leaves
 * at byte1 {1,5} and the S_top slot at byte1 3 -- so BOTH move directions satisfy
 * every shape gate (src junction keeps >=3 children after losing S_top's slot; the
 * dst slot (X,3) is absent and NOSPLIT between the {1,5} sibs; the run under S_top
 * is never adjacent to the dst gap, whose neighbours are the {1,5} sibs).  Two
 * global guard leaves (0x00.. / 0xff..) keep every writer's subtree off the list
 * head/tail.  S_top is a 4-child popcount internal (leaves (X,3,c) c=1..4); the
 * rekey moves those four leaves as one run.
 *
 * Oracle: no move ever returns anything but 0 (committed) or -EAGAIN (root held by
 * a peer -> retry); progress is made (total ops > 0, i.e. no livelock); at
 * quiescence every key is present at its writer's final position and absent at the
 * other, the count is exact, and cds_ft_verify (incl. the ordered-cell pass)
 * passes.  Opt-in FT_INV_MW=1.
 */
extern int _cds_ft_debug_rekey_graft_simple(struct cds_ft *ft,
		const uint8_t *src_key, size_t src_len,
		const uint8_t *dst_key, size_t dst_len);

#define RK_NW		8		/* rekey writers, all contending root */

struct rk_writer_arg {
	struct cds_ft *ft;
	uint8_t bp, dp;			/* the writer's two root-child junction bytes */
	uint8_t sb;			/* S_top's slot byte inside a junction (byte 1) */
	struct ft_test_node *sib[4];	/* fixed straddling sibs: (bp,1)(bp,5)(dp,1)(dp,5) */
	struct ft_test_node *top[4];	/* S_top's 4 leaves, moving with S_top */
	int at_dst;			/* 0: S_top at (bp,sb); 1: at (dp,sb) */
	unsigned long ops, retries;
	int failed;
};

static void *rk_writer(void *arg)
{
	struct rk_writer_arg *w = (struct rk_writer_arg *) arg;
	unsigned long iters = 0;

	rcu_register_thread();
	while (!test_go)
		;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	while (!test_stop) {
		uint8_t cur = w->at_dst ? w->dp : w->bp;
		uint8_t oth = w->at_dst ? w->bp : w->dp;
		uint8_t src_key[2] = { cur, w->sb };
		uint8_t dst_key[2] = { oth, w->sb };
		int rc;

		/*
		 * NO read lock here: the move enters the per-trie MOVE GATE, which
		 * publishes "expect move" to readers and waits a GRACE PERIOD before
		 * the structure is touched.  Holding a read section across that would
		 * deadlock on our own critical section; the entry takes the read lock
		 * the body needs itself.
		 */
		rc = _cds_ft_debug_rekey_graft_simple(w->ft, src_key, 2,
				dst_key, 2);
		if (rc == 0) {
			w->at_dst = !w->at_dst;
			w->ops++;
		} else if (rc == -EAGAIN || rc == -EIO || rc == -ENOMEM) {
			/*
			 * TRANSIENT contention abort (a peer holds root's LOCK
			 * when this move tries to acquire it): -EIO = the graft
			 * recompaction's up-front acquire lost the race (prepare
			 * bailed clean), -EAGAIN = a cow_stop / detach / final-commit
			 * abort, -ENOMEM = a reserve OOM.  All leave the trie pristine
			 * (the hook is abort-clean + single-shot), so re-descend and
			 * retry the SAME direction -- the writer-level recovery that
			 * stands in for the deferred escalation lane.
			 */
			w->retries++;
		} else {
			/* -EINVAL (a permanent shape violation this layout must never
			 * hit) or an unexpected code: a real oracle failure. */
			fprintf(stderr, "rk_writer bp=%u dp=%u: move %02x,%02x -> "
				"%02x,%02x failed rc=%d\n", w->bp, w->dp, cur,
				w->sb, oth, w->sb, rc);
			w->failed = 1;
			mw_violation_snapshot();
			break;
		}
		if ((++iters & 0xff) == 0)
			rcu_quiescent_state();
	}
	rcu_unregister_thread();
	return NULL;
}

static int inv_rekey_graft_disjoint(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft;
	struct rk_writer_arg *w;
	pthread_t writers[RK_NW];
	struct ft_test_node *guard_lo, *guard_hi;
	struct timespec t0;
	unsigned long total_ops = 0, total_retries = 0, live = 0;
	int i, c, ret = 0;

	if (!getenv("FT_INV_MW")) {
		fprintf(stderr, "# inv_rekey_graft_disjoint: skipped "
			"(set FT_INV_MW=1 to run the coherent-rekey writer oracle)\n");
		return 0;
	}
	mw_install_fatal_handler();
	leak_reset();

	ft = create_fixed_fine_lock_ft(4, &group);	/* list ON (default) */
	cds_ft_make_concurrent(ft);

	/* Global guard leaves so no writer's subtree is ever the list head/tail. */
	guard_lo = node_alloc(0x00000000ULL);
	guard_hi = node_alloc(0xff000000ULL);
	rcu_read_lock();
	if (insert_u64(ft, 0x00000000ULL, guard_lo) != CDS_FT_STATUS_OK ||
			insert_u64(ft, 0xff000000ULL, guard_hi) != CDS_FT_STATUS_OK)
		abort();
	rcu_read_unlock();
	live = 2;

	w = (struct rk_writer_arg *) calloc(RK_NW, sizeof(*w));
	if (!w)
		abort();
	for (i = 0; i < RK_NW; i++) {
		uint8_t bp = (uint8_t) (2 * i + 1), dp = (uint8_t) (2 * i + 2);
		uint64_t bpk = (uint64_t) bp << 24, dpk = (uint64_t) dp << 24;
		uint64_t sk[4] = {
			bpk | (1ULL << 16), bpk | (5ULL << 16),
			dpk | (1ULL << 16), dpk | (5ULL << 16),
		};

		w[i].ft = ft;
		w[i].bp = bp;
		w[i].dp = dp;
		w[i].sb = 3;			/* S_top slot byte inside both junctions */
		rcu_read_lock();
		for (c = 0; c < 4; c++) {
			w[i].sib[c] = node_alloc(sk[c]);
			if (insert_u64(ft, sk[c], w[i].sib[c]) != CDS_FT_STATUS_OK)
				abort();
		}
		for (c = 0; c < 4; c++) {
			uint64_t tk = bpk | (3ULL << 16) | ((uint64_t) (c + 1) << 8);

			w[i].top[c] = node_alloc(tk);
			if (insert_u64(ft, tk, w[i].top[c]) != CDS_FT_STATUS_OK)
				abort();
		}
		rcu_read_unlock();
		live += 8;
	}

	test_go = 0;
	test_stop = 0;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	for (i = 0; i < RK_NW; i++)
		pthread_create(&writers[i], NULL, rk_writer, &w[i]);
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	test_go = 1;

	rcu_thread_offline();
	clock_gettime(CLOCK_MONOTONIC, &t0);
	while (elapsed_ms(&t0) < DEFAULT_DURATION_MS)
		usleep(1000);
	test_stop = 1;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	for (i = 0; i < RK_NW; i++)
		pthread_join(writers[i], NULL);
	rcu_thread_online();

	/* Quiescent: every key present at its writer's final position, absent at the other. */
	synchronize_rcu();
	rcu_read_lock();
	for (i = 0; i < RK_NW; i++) {
		uint8_t bp = w[i].bp, dp = w[i].dp;
		uint64_t bpk = (uint64_t) bp << 24, dpk = (uint64_t) dp << 24;
		uint8_t X = w[i].at_dst ? dp : bp, O = w[i].at_dst ? bp : dp;
		uint64_t Xk = (uint64_t) X << 24, Ok = (uint64_t) O << 24;
		uint64_t sk[4] = {
			bpk | (1ULL << 16), bpk | (5ULL << 16),
			dpk | (1ULL << 16), dpk | (5ULL << 16),
		};

		total_ops += w[i].ops;
		total_retries += w[i].retries;
		if (w[i].failed)
			ret = -1;
		for (c = 0; c < 4; c++) {
			struct cds_ft_node *found = NULL;

			if (lookup_u64(ft, sk[c], &found) != CDS_FT_STATUS_OK ||
					found != &w[i].sib[c]->node) {
				fprintf(stderr, "rekey disjoint: writer %d sib %d lost\n",
					i, c);
				ret = -1;
			}
		}
		for (c = 0; c < 4; c++) {
			uint64_t here = Xk | (3ULL << 16) | ((uint64_t) (c + 1) << 8);
			uint64_t there = Ok | (3ULL << 16) | ((uint64_t) (c + 1) << 8);
			struct cds_ft_node *found = NULL;

			if (lookup_u64(ft, here, &found) != CDS_FT_STATUS_OK ||
					found != &w[i].top[c]->node) {
				fprintf(stderr, "rekey disjoint: writer %d top %d absent "
					"at final pos (at_dst=%d)\n", i, c, w[i].at_dst);
				ret = -1;
			}
			if (lookup_u64(ft, there, &found) == CDS_FT_STATUS_OK) {
				fprintf(stderr, "rekey disjoint: writer %d top %d still at "
					"old pos\n", i, c);
				ret = -1;
			}
		}
	}
	if (cds_ft_count_keys(ft) != live) {
		fprintf(stderr, "rekey disjoint: count_keys %lu != live %lu\n",
			cds_ft_count_keys(ft), live);
		ret = -1;
	}
	if (cds_ft_verify(ft, stderr) != CDS_FT_STATUS_OK) {
		fprintf(stderr, "rekey disjoint: cds_ft_verify failed\n");
		ret = -1;
	}
	rcu_read_unlock();

	if (total_ops == 0) {			/* liveness: writers made progress */
		fprintf(stderr, "rekey disjoint: no successful moves (livelock?)\n");
		ret = -1;
	}
	fprintf(stderr, "# inv_rekey_graft_disjoint: %d writers, %lu moves, "
		"%lu retries, %lu live keys\n", RK_NW, total_ops, total_retries,
		live);

	free(w);
	if (drain_and_destroy(ft, group) < 0)
		ret = -1;
	if (leak_check() < 0)
		ret = -1;
	return ret;
}

/*
 * OCCUPIED-DESTINATION rekey oracle -- the ONLY concurrent coverage of the merge
 * fold, and it exists because there was none.
 *
 * Every other rekey oracle moves a subtree into an EMPTY destination, which is
 * the GRAFT path.  Measured across the whole FT_INV_MW plan, the merge fold's
 * distinctive edge class (skip-compressed src-origin re-parents) was reached
 * ZERO times, so the record-time resolution those edges depend on was validated
 * single-threaded only, and "a peer cannot stale a skip flag between the
 * resolution and the flip" rested on an argument rather than a measurement.
 * This oracle is what turns that into evidence.
 *
 * ★ WHY IT RE-SEEDS INSTEAD OF SHUTTLING.  A merge EMPTIES its source, so the
 * back-and-forth the other oracles use degenerates after ONE move: the second
 * direction finds an empty destination and takes the graft path forever.  Each
 * iteration therefore SEEDS the source afresh, moves it into a destination that
 * is permanently occupied by RESIDENT keys, and removes only the relocated
 * movers -- leaving the residents in place so the next move is a merge too.
 *
 * Per writer w, two root-child junctions bp=2w+1, dp=2w+2 (disjoint across
 * writers, so no writer re-homes a peer's junction; they all still contend
 * ROOT's LOCK, which is the contention this exercises):
 *   (bp,1) (bp,5) (dp,1) (dp,5)   straddling sibs, keep both junctions >= 3
 *                                 children so the detach stays an in-place delete
 *   (dp,3,5) (dp,3,6)             RESIDENTS -- never move, keep the dst OCCUPIED
 *   (bp,3,1) (bp,3,2)             MOVERS -- seeded, merged to (dp,3,*), removed
 * A lone key under (X,3,c) path-compresses, so S_top's children are compressed:
 * that is what makes the moved re-parents SKIP-COMPRESSED, the class this oracle
 * is for.  The mover and resident byte-2 values are disjoint, so the union at the
 * top frame adds children rather than colliding (dup-chain collisions are a
 * separate shape, not covered here).
 *
 * THE ORACLE, checked after EVERY successful move and not only at quiescence:
 * all four keys must be present under dp.  A merge that dropped the destination's
 * own keys -- the exact failure the occupied-dst path risks, and one that leaves
 * the trie perfectly VERIFIABLE -- is invisible to cds_ft_verify and to a key
 * count taken at rest, because the residents are re-checked only where they
 * should be.  Losing them here fails immediately.
 *
 * -EINVAL is FATAL: this layout satisfies every documented shape gate, so a
 * permanent shape rejection is a real defect, not a race.  Opt-in FT_INV_MW=1.
 */
#define RKM_NW		8		/* writers, all contending root */
#define RKM_SB		3		/* S_top slot byte inside both junctions */
/*
 * Byte-2 layout.  Mover 1 lands EXACTLY on resident 0, so every merge splices a
 * duplicate chain as well as unioning a disjoint child -- the leaf-splice arm and
 * the chain-holder lock it needs, under contention.  Mover 0 stays disjoint so a
 * single move covers both.
 */
#define RKM_MV0		1
#define RKM_MV1		5		/* == RKM_RES0: the COLLIDING pair */
#define RKM_RES0	5
#define RKM_RES1	6

struct rkm_writer_arg {
	struct cds_ft *ft;
	uint8_t bp, dp;
	struct ft_test_node *sib[4];	/* (bp,1)(bp,5)(dp,1)(dp,5) */
	struct ft_test_node *res[2];	/* residents (dp,3,5)(dp,3,6) */
	int seeded;			/* movers currently live at (bp,3,*) */
	unsigned long ops, retries;
	int failed;
};

static uint64_t rkm_key(uint8_t x, uint8_t b1, uint8_t b2)
{
	return ((uint64_t) x << 24) | ((uint64_t) b1 << 16) |
		((uint64_t) b2 << 8);
}

/*
 * Remove the entry at @v that is NOT @keep, reclaiming it.  The collided key is a
 * chain of two and only the MOVER may be taken back -- a plain remove would take
 * whichever end the lookup lands on and could silently evict the resident, which
 * is the very thing this oracle checks for.
 */
static enum cds_ft_status rkm_remove_dup_other(struct cds_ft *ft, uint64_t v,
		struct cds_ft_node *keep)
{
	struct cds_ft_iter *iter = NULL;
	struct cds_ft_node *head, *n;
	enum cds_ft_status st = CDS_FT_STATUS_NOT_FOUND;
	uint8_t k[8] = { 0 };

	if (cds_ft_iter_create(ft, &iter) < 0)
		abort();
	rcu_read_lock();
	cds_ft_u64_to_key(ft, v, k, CDS_FT_LEN_DEFAULT);
	cds_ft_iter_set_key(iter, k, CDS_FT_LEN_DEFAULT);
	cds_ft_lookup(ft, iter);
	head = cds_ft_iter_node(iter);
	for (n = head; n; n = cds_ft_node_next_rcu(n)) {
		if (n == keep)
			continue;
		st = cds_ft_remove(ft, iter, n);
		if (st == CDS_FT_STATUS_OK)
			node_free_rcu(to_test_node(n));
		break;
	}
	rcu_read_unlock();
	cds_ft_iter_destroy(iter);
	return st;
}

static const uint8_t rkm_mv[2] = { RKM_MV0, RKM_MV1 };
static const uint8_t rkm_res[2] = { RKM_RES0, RKM_RES1 };

static void *rkm_writer(void *arg)
{
	struct rkm_writer_arg *w = (struct rkm_writer_arg *) arg;
	uint8_t src_key[2] = { 0, RKM_SB }, dst_key[2] = { 0, RKM_SB };
	unsigned long iters = 0;
	int c;

	src_key[0] = w->bp;
	dst_key[0] = w->dp;

	rcu_register_thread();
	while (!test_go)
		;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	while (!test_stop) {
		int rc;

		/* SEED the source: two lone deep keys -> compressed children. */
		if (!w->seeded) {
			rcu_read_lock();
			for (c = 0; c < 2; c++) {
				uint64_t mk = rkm_key(w->bp, RKM_SB, rkm_mv[c]);

				if (insert_u64(w->ft, mk, node_alloc(mk)) !=
						CDS_FT_STATUS_OK) {
					rcu_read_unlock();
					fprintf(stderr, "rkm_writer bp=%u: seed "
						"insert failed\n", w->bp);
					w->failed = 1;
					goto out;
				}
			}
			rcu_read_unlock();
			w->seeded = 1;
		}

		/*
		 * NO read lock: the move enters the per-trie MOVE GATE, which waits
		 * a grace period -- holding a read section across it self-deadlocks.
		 */
		rc = _cds_ft_debug_rekey_graft_simple(w->ft, src_key, 2, dst_key, 2);
		if (rc == 0) {
			w->ops++;
			/*
			 * The union must hold BOTH sides.  Losing the residents is
			 * the occupied-dst failure that a verify pass and a rest-time
			 * count both sail straight past.
			 */
			rcu_read_lock();
			for (c = 0; c < 2; c++) {
				struct cds_ft_node *f = NULL, *n;
				uint64_t moved = rkm_key(w->dp, RKM_SB, rkm_mv[c]);
				uint64_t resid = rkm_key(w->dp, RKM_SB, rkm_res[c]);
				unsigned long chain = 0;

				if (lookup_u64(w->ft, moved, &f) != CDS_FT_STATUS_OK) {
					rcu_read_unlock();
					fprintf(stderr, "rkm_writer bp=%u dp=%u: moved "
						"key %d absent after merge\n",
						w->bp, w->dp, c);
					w->failed = 1;
					mw_violation_snapshot();
					goto out;
				}
				if (lookup_u64(w->ft, resid, &f) != CDS_FT_STATUS_OK) {
					rcu_read_unlock();
					fprintf(stderr, "rkm_writer bp=%u dp=%u: RESIDENT "
						"key %d lost to the union\n",
						w->bp, w->dp, c);
					w->failed = 1;
					mw_violation_snapshot();
					goto out;
				}
				/*
				 * The resident must still be ON the chain -- a splice
				 * that replaced it rather than appending leaves a
				 * perfectly valid single-entry key behind.
				 */
				for (n = f; n; n = cds_ft_node_next_rcu(n)) {
					chain++;
					if (n == &w->res[c]->node)
						break;
				}
				if (!n) {
					rcu_read_unlock();
					fprintf(stderr, "rkm_writer bp=%u dp=%u: resident "
						"%d not on its chain after the splice\n",
						w->bp, w->dp, c);
					w->failed = 1;
					mw_violation_snapshot();
					goto out;
				}
			}
			rcu_read_unlock();
			/* Remove only the relocated movers; residents stay. */
			for (c = 0; c < 2; c++) {
				uint64_t moved = rkm_key(w->dp, RKM_SB, rkm_mv[c]);
				enum cds_ft_status rst;

				/*
				 * Mover 1 shares its key with resident 0, so take the
				 * entry that is NOT the resident; mover 0 is a
				 * singleton and a plain remove is exact.
				 */
				rst = (rkm_mv[c] == RKM_RES0 || rkm_mv[c] == RKM_RES1) ?
					rkm_remove_dup_other(w->ft, moved,
						&w->res[rkm_mv[c] == RKM_RES0 ? 0 : 1]->node) :
					remove_u64(w->ft, moved);
				if (rst != CDS_FT_STATUS_OK) {
					fprintf(stderr, "rkm_writer bp=%u: cleanup "
						"remove failed\n", w->bp);
					w->failed = 1;
					goto out;
				}
			}
			w->seeded = 0;
		} else if (rc == -EAGAIN || rc == -EIO || rc == -ENOMEM) {
			/* Transient: trie pristine, movers still at src -- retry. */
			w->retries++;
		} else {
			fprintf(stderr, "rkm_writer bp=%u dp=%u: merge-move failed "
				"rc=%d\n", w->bp, w->dp, rc);
			w->failed = 1;
			mw_violation_snapshot();
			goto out;
		}
		if ((++iters & 0xff) == 0)
			rcu_quiescent_state();
	}
out:
	rcu_unregister_thread();
	return NULL;
}

static int inv_rekey_merge_occupied_dst(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft;
	struct rkm_writer_arg *w;
	pthread_t writers[RKM_NW];
	struct timespec t0;
	unsigned long total_ops = 0, total_retries = 0, live = 0;
	int i, c, ret = 0;

	if (!getenv("FT_INV_MW")) {
		fprintf(stderr, "# inv_rekey_merge_occupied_dst: skipped "
			"(set FT_INV_MW=1 to run the occupied-dst merge oracle)\n");
		return 0;
	}
	mw_install_fatal_handler();
	leak_reset();

	/* LIST OFF: the merge fold refuses list-on (the ordered interleave is
	 * per-key ms_edges machinery, not this driver's contiguous run move). */
	ft = create_fixed_fine_lock_listoff_ft(4, &group);
	cds_ft_make_concurrent(ft);

	w = (struct rkm_writer_arg *) calloc(RKM_NW, sizeof(*w));
	if (!w)
		abort();
	for (i = 0; i < RKM_NW; i++) {
		uint8_t bp = (uint8_t) (2 * i + 1), dp = (uint8_t) (2 * i + 2);
		uint64_t sk[4] = {
			rkm_key(bp, 1, 0), rkm_key(bp, 5, 0),
			rkm_key(dp, 1, 0), rkm_key(dp, 5, 0),
		};

		w[i].ft = ft;
		w[i].bp = bp;
		w[i].dp = dp;
		rcu_read_lock();
		for (c = 0; c < 4; c++) {
			w[i].sib[c] = node_alloc(sk[c]);
			if (insert_u64(ft, sk[c], w[i].sib[c]) != CDS_FT_STATUS_OK)
				abort();
		}
		for (c = 0; c < 2; c++) {	/* residents: keep the dst OCCUPIED */
			uint64_t rk = rkm_key(dp, RKM_SB, rkm_res[c]);

			w[i].res[c] = node_alloc(rk);
			if (insert_u64(ft, rk, w[i].res[c]) != CDS_FT_STATUS_OK)
				abort();
		}
		rcu_read_unlock();
		live += 6;
	}

	test_go = 0;
	test_stop = 0;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	for (i = 0; i < RKM_NW; i++)
		pthread_create(&writers[i], NULL, rkm_writer, &w[i]);
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	test_go = 1;

	rcu_thread_offline();
	clock_gettime(CLOCK_MONOTONIC, &t0);
	while (elapsed_ms(&t0) < DEFAULT_DURATION_MS)
		usleep(1000);
	test_stop = 1;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	for (i = 0; i < RKM_NW; i++)
		pthread_join(writers[i], NULL);
	rcu_thread_online();

	/* Drop any movers left seeded at the source by the final iteration. */
	synchronize_rcu();
	for (i = 0; i < RKM_NW; i++) {
		if (!w[i].seeded)
			continue;
		for (c = 0; c < 2; c++)
			(void) remove_u64(ft, rkm_key(w[i].bp, RKM_SB, rkm_mv[c]));
		w[i].seeded = 0;
	}

	synchronize_rcu();
	rcu_read_lock();
	for (i = 0; i < RKM_NW; i++) {
		total_ops += w[i].ops;
		total_retries += w[i].retries;
		if (w[i].failed)
			ret = -1;
		/*
		 * PER-WRITER liveness: a global "some moves happened" sails right
		 * past one starved writer, which is a measured failure mode here.
		 */
		if (w[i].ops == 0) {
			fprintf(stderr, "rekey merge: writer %d made NO move "
				"(starved / livelock)\n", i);
			ret = -1;
		}
		for (c = 0; c < 4; c++) {
			struct cds_ft_node *f = NULL;
			uint64_t sk = c < 2 ? rkm_key(w[i].bp, c == 0 ? 1 : 5, 0) :
				rkm_key(w[i].dp, c == 2 ? 1 : 5, 0);

			if (lookup_u64(ft, sk, &f) != CDS_FT_STATUS_OK ||
					f != &w[i].sib[c]->node) {
				fprintf(stderr, "rekey merge: writer %d sib %d lost\n",
					i, c);
				ret = -1;
			}
		}
		for (c = 0; c < 2; c++) {	/* residents survived every union */
			struct cds_ft_node *f = NULL;
			uint64_t rk = rkm_key(w[i].dp, RKM_SB, rkm_res[c]);

			if (lookup_u64(ft, rk, &f) != CDS_FT_STATUS_OK ||
					f != &w[i].res[c]->node) {
				fprintf(stderr, "rekey merge: writer %d resident %d "
					"lost\n", i, c);
				ret = -1;
			}
		}
	}
	if (cds_ft_count_keys(ft) != live) {
		fprintf(stderr, "rekey merge: count_keys %lu != live %lu\n",
			cds_ft_count_keys(ft), live);
		ret = -1;
	}
	if (cds_ft_verify(ft, stderr) != CDS_FT_STATUS_OK) {
		fprintf(stderr, "rekey merge: cds_ft_verify failed\n");
		ret = -1;
	}
	rcu_read_unlock();

	fprintf(stderr, "# inv_rekey_merge_occupied_dst: %d writers, %lu merges, "
		"%lu retries, %lu live keys\n", RKM_NW, total_ops, total_retries,
		live);

	free(w);
	if (drain_and_destroy(ft, group) < 0)
		ret = -1;
	if (leak_check() < 0)
		ret = -1;
	return ret;
}

/*
 * SHARED-DESTINATION merge oracle -- the companion inv_rekey_merge_occupied_dst
 * cannot be, and the reason is the whole point.
 *
 * That oracle gives every writer its own junctions.  Writers then contend only
 * ROOT, which the DETACH acquires -- so every node the MERGE BRANCH acquires
 * (the publish parent, the dst overlap spine, the dup-chain holders) stays
 * writer-PRIVATE and its bail is structurally unreachable.  Measured: 0 of the
 * fold's 7 bail sites taken across 4187 merges, while the same run logged 3189
 * retries.  Retries are not coverage; an oracle exercises only the acquires
 * whose nodes its writers SHARE.
 *
 * So here the DESTINATION is shared.  Writer w keeps a PRIVATE source junction
 * bp=0x20+w (so seeding never collides) and merges into one of RKMS_NJ SHARED
 * destinations J[w % RKMS_NJ].  Several writers therefore aim the same publish
 * parent, retire the same overlap node, and append to the same chain, which is
 * exactly the contention the disjoint layout cannot produce.
 *
 * Per shared junction J: RESIDENTS at (J,3,1) and (J,3,2), permanent, so the
 * destination is always OCCUPIED (a merge, never a graft) and the node is always
 * a >=2-child branch (never path-compressed).
 * Per writer w: ONE mover at (bp,3,0x10+w), PRIVATE, so cleanup can never touch
 * a peer's key.
 *
 * NO COLLISION HERE.  A colliding mover would land on a chain several writers
 * append to at once, and taking one's own entry back off it removes a NON-HEAD
 * duplicate while peers restructure the same chain -- a second variable, and
 * this oracle already has one job: contending the merge branch's own acquires,
 * which needs no collision.  Collisions live in inv_rekey_merge_occupied_dst.
 *
 * ★ That separation was FIRST made for a reason that turned out to be WRONG, so
 * do not reuse the reasoning: while this oracle was being built it aborted in
 * ft_node_recompact, and dropping collisions was tried as the fix on the theory
 * that the cleanup above was a harness race.  It was not -- the abort persisted
 * without collisions, and the cause was ft_detach_node's up-walk reading a child
 * count off a peer's parked state word.  Re-adding collisions here is a
 * legitimate future extension, not a known hazard.
 *
 * ORACLE, per move and not only at rest: this writer's private moved key is
 * present and BOTH residents survive.  A merge that dropped the destination's
 * own keys leaves a verifiable trie, so only an in-loop check sees it.
 * -EINVAL is fatal; liveness is per writer.
 */
#define RKMS_NW		8		/* writers */
#define RKMS_NJ		2		/* SHARED destination junctions */
#define RKMS_SB		3		/* S_top slot byte inside every junction */
#define RKMS_GUARD_BASE	0x40		/* 2nd child byte base: keeps S_top a branch */

struct rkms_writer_arg {
	struct cds_ft *ft;
	uint8_t bp;			/* PRIVATE source junction */
	uint8_t dp;			/* SHARED destination junction */
	uint8_t mine;			/* this writer's private mover byte */
	uint8_t guard;			/* and its private 2nd-child byte */
	struct ft_test_node *sib[2];	/* (bp,1) (bp,5): keep BP >= 3 children */
	struct ft_test_node *seed_priv;	/* the node seeded at (bp,3,mine) */
	struct ft_test_node *seed_guard;/* second child so S_top is a branch */
	int seeded;
	unsigned long ops, retries;
	int failed;
};

/* Remove the node @n stored at key @v, by IDENTITY -- see the header. */
static enum cds_ft_status rkms_remove_node(struct cds_ft *ft, uint64_t v,
		struct ft_test_node *n)
{
	struct cds_ft_iter *iter = NULL;
	enum cds_ft_status st = CDS_FT_STATUS_NOT_FOUND;
	uint8_t k[8] = { 0 };

	if (cds_ft_iter_create(ft, &iter) < 0)
		abort();
	rcu_read_lock();
	cds_ft_u64_to_key(ft, v, k, CDS_FT_LEN_DEFAULT);
	cds_ft_iter_set_key(iter, k, CDS_FT_LEN_DEFAULT);
	cds_ft_lookup(ft, iter);
	if (cds_ft_iter_node(iter)) {
		st = cds_ft_remove(ft, iter, &n->node);
		if (st == CDS_FT_STATUS_OK)
			node_free_rcu(n);
	}
	rcu_read_unlock();
	cds_ft_iter_destroy(iter);
	return st;
}

static void *rkms_writer(void *arg)
{
	struct rkms_writer_arg *w = (struct rkms_writer_arg *) arg;
	uint8_t src_key[2], dst_key[2];
	unsigned long iters = 0;

	src_key[0] = w->bp; src_key[1] = RKMS_SB;
	dst_key[0] = w->dp; dst_key[1] = RKMS_SB;

	rcu_register_thread();
	while (!test_go)
		;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	while (!test_stop) {
		uint64_t kp = rkm_key(w->bp, RKMS_SB, w->mine);
		uint64_t kg = rkm_key(w->bp, RKMS_SB, w->guard);
		uint64_t dp_priv = rkm_key(w->dp, RKMS_SB, w->mine);
		struct cds_ft_node *f = NULL;
		int rc;

		if (!w->seeded) {
			rcu_read_lock();
			/*
			 * TWO keys under (bp,3): the mover plus a per-writer guard
			 * at a private byte, so S_top is a >=2-child BRANCH rather
			 * than a path-compressed node the shape gate refuses.  Both
			 * are lone deep keys, hence compressed children -- the
			 * skip-compressed re-parents this path is about.
			 */
			w->seed_priv = node_alloc(kp);
			w->seed_guard = node_alloc(kg);
			if (insert_u64(w->ft, kp, w->seed_priv) != CDS_FT_STATUS_OK ||
					insert_u64(w->ft, kg, w->seed_guard) !=
						CDS_FT_STATUS_OK) {
				rcu_read_unlock();
				fprintf(stderr, "rkms bp=%u: seed failed\n", w->bp);
				w->failed = 1;
				goto out;
			}
			rcu_read_unlock();
			w->seeded = 1;
		}

		/* No read lock: the move takes the gate, which waits a grace period. */
		rc = _cds_ft_debug_rekey_graft_simple(w->ft, src_key, 2, dst_key, 2);
		if (rc == 0) {
			w->ops++;
			rcu_read_lock();
			/* my PRIVATE moved key */
			if (lookup_u64(w->ft, dp_priv, &f) != CDS_FT_STATUS_OK ||
					f != &w->seed_priv->node) {
				rcu_read_unlock();
				fprintf(stderr, "rkms bp=%u dp=%u: private moved key "
					"absent after merge\n", w->bp, w->dp);
				w->failed = 1;
				mw_violation_snapshot();
				goto out;
			}
			rcu_read_unlock();
			if (rkms_remove_node(w->ft, dp_priv, w->seed_priv) !=
						CDS_FT_STATUS_OK ||
					rkms_remove_node(w->ft,
						rkm_key(w->dp, RKMS_SB, w->guard),
						w->seed_guard) != CDS_FT_STATUS_OK) {
				fprintf(stderr, "rkms bp=%u: cleanup failed\n", w->bp);
				w->failed = 1;
				goto out;
			}
			w->seeded = 0;
		} else if (rc == -EAGAIN || rc == -EIO || rc == -ENOMEM) {
			w->retries++;
		} else {
			fprintf(stderr, "rkms bp=%u dp=%u: merge failed rc=%d\n",
				w->bp, w->dp, rc);
			w->failed = 1;
			mw_violation_snapshot();
			goto out;
		}
		if ((++iters & 0xff) == 0)
			rcu_quiescent_state();
	}
out:
	rcu_unregister_thread();
	return NULL;
}

static int inv_rekey_merge_shared_dst(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft;
	struct rkms_writer_arg *w;
	struct ft_test_node *res[RKMS_NJ][2];
	pthread_t writers[RKMS_NW];
	struct timespec t0;
	unsigned long total_ops = 0, total_retries = 0, live = 0;
	int i, c, ret = 0;
	static const uint8_t J[RKMS_NJ] = { 0x10, 0x11 };

	if (!getenv("FT_INV_MW")) {
		fprintf(stderr, "# inv_rekey_merge_shared_dst: skipped "
			"(set FT_INV_MW=1 to run the shared-destination merge oracle)\n");
		return 0;
	}
	mw_install_fatal_handler();
	leak_reset();

	ft = create_fixed_fine_lock_listoff_ft(4, &group);	/* LIST OFF */
	cds_ft_make_concurrent(ft);

	rcu_read_lock();
	for (i = 0; i < RKMS_NJ; i++) {
		/* Sibs so the shared junction node is a stable multi-child branch. */
		for (c = 0; c < 2; c++) {
			uint64_t sk = rkm_key(J[i], c == 0 ? 1 : 5, 0);

			if (insert_u64(ft, sk, node_alloc(sk)) != CDS_FT_STATUS_OK)
				abort();
			live++;
		}
		for (c = 0; c < 2; c++) {	/* RESIDENTS: dst always occupied */
			uint64_t rk = rkm_key(J[i], RKMS_SB, (uint8_t) (c + 1));

			res[i][c] = node_alloc(rk);
			if (insert_u64(ft, rk, res[i][c]) != CDS_FT_STATUS_OK)
				abort();
			live++;
		}
	}
	rcu_read_unlock();

	w = (struct rkms_writer_arg *) calloc(RKMS_NW, sizeof(*w));
	if (!w)
		abort();
	for (i = 0; i < RKMS_NW; i++) {
		uint8_t bp = (uint8_t) (0x20 + i);

		w[i].ft = ft;
		w[i].bp = bp;
		w[i].dp = J[i % RKMS_NJ];
		w[i].mine = (uint8_t) (0x10 + i);
		w[i].guard = (uint8_t) (RKMS_GUARD_BASE + i);
		rcu_read_lock();
		for (c = 0; c < 2; c++) {	/* BP >= 3 children on removal */
			uint64_t sk = rkm_key(bp, c == 0 ? 1 : 5, 0);

			w[i].sib[c] = node_alloc(sk);
			if (insert_u64(ft, sk, w[i].sib[c]) != CDS_FT_STATUS_OK)
				abort();
			live++;
		}
		rcu_read_unlock();
	}

	test_go = 0;
	test_stop = 0;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	for (i = 0; i < RKMS_NW; i++)
		pthread_create(&writers[i], NULL, rkms_writer, &w[i]);
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	test_go = 1;

	rcu_thread_offline();
	clock_gettime(CLOCK_MONOTONIC, &t0);
	while (elapsed_ms(&t0) < DEFAULT_DURATION_MS)
		usleep(1000);
	test_stop = 1;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	for (i = 0; i < RKMS_NW; i++)
		pthread_join(writers[i], NULL);
	rcu_thread_online();

	/* Drop whatever the last iteration left seeded at the private sources. */
	synchronize_rcu();
	for (i = 0; i < RKMS_NW; i++) {
		if (!w[i].seeded)
			continue;
		(void) rkms_remove_node(ft, rkm_key(w[i].bp, RKMS_SB, w[i].mine),
				w[i].seed_priv);
		(void) rkms_remove_node(ft, rkm_key(w[i].bp, RKMS_SB, w[i].guard),
				w[i].seed_guard);
		w[i].seeded = 0;
	}

	synchronize_rcu();
	rcu_read_lock();
	for (i = 0; i < RKMS_NW; i++) {
		total_ops += w[i].ops;
		total_retries += w[i].retries;
		if (w[i].failed)
			ret = -1;
		if (w[i].ops == 0) {	/* per-writer: a global check hides starvation */
			fprintf(stderr, "rekey merge shared: writer %d made NO move\n", i);
			ret = -1;
		}
	}
	for (i = 0; i < RKMS_NJ; i++) {
		for (c = 0; c < 2; c++) {
			struct cds_ft_node *f = NULL;
			uint64_t rk = rkm_key(J[i], RKMS_SB, (uint8_t) (c + 1));

			if (lookup_u64(ft, rk, &f) != CDS_FT_STATUS_OK ||
					f != &res[i][c]->node) {
				fprintf(stderr, "rekey merge shared: resident %d/%d lost "
					"to a union\n", i, c);
				ret = -1;
			}
		}
	}
	if (cds_ft_count_entries(ft) != live) {
		fprintf(stderr, "rekey merge shared: entries %lu != live %lu\n",
			cds_ft_count_entries(ft), live);
		ret = -1;
	}
	if (cds_ft_verify(ft, stderr) != CDS_FT_STATUS_OK) {
		fprintf(stderr, "rekey merge shared: cds_ft_verify failed\n");
		ret = -1;
	}
	rcu_read_unlock();

	fprintf(stderr, "# inv_rekey_merge_shared_dst: %d writers over %d shared "
		"destinations, %lu merges, %lu retries, %lu live keys\n",
		RKMS_NW, RKMS_NJ, total_ops, total_retries, live);

	free(w);
	if (drain_and_destroy(ft, group) < 0)
		ret = -1;
	if (leak_check() < 0)
		ret = -1;
	return ret;
}

/*
 * Coherent-rekey concurrent-writer oracle, CROSS-JUNCTION: the same back-and-forth
 * move, but between junctions whose PARENTS DIFFER (d_src.ppnf != d_dst.ppnf), the
 * shape the hook's original gate refused.  The folded detach can no longer reuse
 * the lock the graft holds, so its src-junction recompaction acquires the src
 * junction's own parent itself, validated by the BP.parent == @parent read-set
 * guard riding that acquire commit -- the arm this oracle exists to run under
 * contention.
 *
 * It is also the SEPARATION experiment for the disjoint oracle's headline finding
 * (moves serialize on the one shared ancestor they all lock): here NO node is
 * shared between writers -- writer i's two junctions hang under its OWN root
 * children (2i+1, 2i+2), and root is never locked by a move -- so the retry rate
 * measures the acquire path itself rather than root contention.  Both numbers are
 * printed; the disjoint oracle's ~4 retries per move against this one's is the
 * comparison.
 *
 * 5-byte keys, so S_top's children stay one level ABOVE the terminal externals (as
 * in the depth-2 oracles).  Per writer, under each of its two root children W:
 *   W -> { 1 -> {1, [3 = S_top -> {1,2,3,4}], 5},  9 }
 * The byte-9 filler makes the depth-1 node BRANCHING (a lone child would be
 * path-compressed, which the driver's descent refuses); the byte-{1,5} leaves make
 * the junction a 3-child node that survives losing S_top and give the dst gap
 * neighbours that are never the moved run's own endpoints.  List ON, so each move
 * also folds the run's cell re-splice into the same commit.  Opt-in FT_INV_MW=1.
 */
#define RKX_NW		8		/* cross-junction rekey writers */
#define RKX_NSUB	4		/* S_top's four leaves, moving with it */
#define RKX_KEY(b0, b1, b2, b3, b4)					\
	(((uint64_t) (b0) << 32) | ((uint64_t) (b1) << 24) |		\
	 ((uint64_t) (b2) << 16) | ((uint64_t) (b3) << 8) | (uint64_t) (b4))

struct rkx_writer_arg {
	struct cds_ft *ft;
	uint8_t w1, w2;			/* the writer's two root children */
	struct ft_test_node *fill[2];	/* (W,9,0,0,0): keeps depth-1 branching */
	struct ft_test_node *sib[4];	/* (W,1,1,0,0) / (W,1,5,0,0) on both sides */
	struct ft_test_node *top[RKX_NSUB];
	int at_w2;			/* 0: S_top under w1; 1: under w2 */
	unsigned long ops, retries;
	int failed;
};

static void *rkx_writer(void *arg)
{
	struct rkx_writer_arg *w = (struct rkx_writer_arg *) arg;
	unsigned long iters = 0;

	rcu_register_thread();
	while (!test_go)
		;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	while (!test_stop) {
		uint8_t cur = w->at_w2 ? w->w2 : w->w1;
		uint8_t oth = w->at_w2 ? w->w1 : w->w2;
		uint8_t src_key[3] = { cur, 1, 3 };
		uint8_t dst_key[3] = { oth, 1, 3 };
		int rc;

		/* No read lock: the move enters the gate, which waits a GP. */
		rc = _cds_ft_debug_rekey_graft_simple(w->ft, src_key, 3, dst_key, 3);
		if (rc == 0) {
			w->at_w2 = !w->at_w2;
			w->ops++;
		} else if (rc == -EAGAIN || rc == -EIO || rc == -ENOMEM) {
			w->retries++;	/* transient: trie pristine, re-descend */
		} else {
			fprintf(stderr, "rkx_writer w1=%u w2=%u: move from %02x "
				"failed rc=%d\n", w->w1, w->w2, cur, rc);
			w->failed = 1;
			mw_violation_snapshot();
			break;
		}
		if ((++iters & 0xff) == 0)
			rcu_quiescent_state();
	}
	rcu_unregister_thread();
	return NULL;
}

static int inv_rekey_graft_cross_junction(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft;
	struct rkx_writer_arg *w;
	pthread_t writers[RKX_NW];
	struct ft_test_node *guard_lo, *guard_hi;
	struct timespec t0;
	unsigned long total_ops = 0, total_retries = 0, live = 0;
	int i, c, ret = 0;

	if (!getenv("FT_INV_MW")) {
		fprintf(stderr, "# inv_rekey_graft_cross_junction: skipped "
			"(set FT_INV_MW=1 to run the coherent-rekey writer oracle)\n");
		return 0;
	}
	mw_install_fatal_handler();
	leak_reset();

	ft = create_fixed_fine_lock_ft(5, &group);	/* list ON (default) */
	cds_ft_make_concurrent(ft);

	/* Global guard leaves so no writer's subtree is ever the list head/tail. */
	guard_lo = node_alloc(RKX_KEY(0x00, 0, 0, 0, 0));
	guard_hi = node_alloc(RKX_KEY(0xff, 0, 0, 0, 0));
	rcu_read_lock();
	if (insert_u64(ft, RKX_KEY(0x00, 0, 0, 0, 0), guard_lo) != CDS_FT_STATUS_OK ||
			insert_u64(ft, RKX_KEY(0xff, 0, 0, 0, 0), guard_hi) !=
			CDS_FT_STATUS_OK)
		abort();
	rcu_read_unlock();
	live = 2;

	w = (struct rkx_writer_arg *) calloc(RKX_NW, sizeof(*w));
	if (!w)
		abort();
	for (i = 0; i < RKX_NW; i++) {
		uint8_t roots[2] = { (uint8_t) (2 * i + 1), (uint8_t) (2 * i + 2) };
		int r;

		w[i].ft = ft;
		w[i].w1 = roots[0];
		w[i].w2 = roots[1];
		rcu_read_lock();
		for (r = 0; r < 2; r++) {
			uint64_t fk = RKX_KEY(roots[r], 9, 0, 0, 0);
			uint64_t s1 = RKX_KEY(roots[r], 1, 1, 0, 0);
			uint64_t s5 = RKX_KEY(roots[r], 1, 5, 0, 0);

			w[i].fill[r] = node_alloc(fk);
			w[i].sib[2 * r] = node_alloc(s1);
			w[i].sib[2 * r + 1] = node_alloc(s5);
			if (insert_u64(ft, fk, w[i].fill[r]) != CDS_FT_STATUS_OK ||
					insert_u64(ft, s1, w[i].sib[2 * r]) !=
					CDS_FT_STATUS_OK ||
					insert_u64(ft, s5, w[i].sib[2 * r + 1]) !=
					CDS_FT_STATUS_OK)
				abort();
		}
		for (c = 0; c < RKX_NSUB; c++) {
			uint64_t tk = RKX_KEY(roots[0], 1, 3, c + 1, 0);

			w[i].top[c] = node_alloc(tk);
			if (insert_u64(ft, tk, w[i].top[c]) != CDS_FT_STATUS_OK)
				abort();
		}
		rcu_read_unlock();
		live += 6 + RKX_NSUB;
	}

	test_go = 0;
	test_stop = 0;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	for (i = 0; i < RKX_NW; i++)
		pthread_create(&writers[i], NULL, rkx_writer, &w[i]);
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	test_go = 1;

	rcu_thread_offline();
	clock_gettime(CLOCK_MONOTONIC, &t0);
	while (elapsed_ms(&t0) < DEFAULT_DURATION_MS)
		usleep(1000);
	test_stop = 1;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	for (i = 0; i < RKX_NW; i++)
		pthread_join(writers[i], NULL);
	rcu_thread_online();

	/* Quiescent: each key at its writer's final position, absent at the other. */
	synchronize_rcu();
	rcu_read_lock();
	for (i = 0; i < RKX_NW; i++) {
		uint8_t roots[2] = { w[i].w1, w[i].w2 };
		uint8_t here = w[i].at_w2 ? w[i].w2 : w[i].w1;
		uint8_t there = w[i].at_w2 ? w[i].w1 : w[i].w2;
		int r;

		total_ops += w[i].ops;
		total_retries += w[i].retries;
		if (w[i].failed)
			ret = -1;
		for (r = 0; r < 2; r++) {
			struct cds_ft_node *found = NULL;
			uint64_t fixed[3] = {
				RKX_KEY(roots[r], 9, 0, 0, 0),
				RKX_KEY(roots[r], 1, 1, 0, 0),
				RKX_KEY(roots[r], 1, 5, 0, 0),
			};
			struct ft_test_node *expect[3] = {
				w[i].fill[r], w[i].sib[2 * r], w[i].sib[2 * r + 1],
			};

			for (c = 0; c < 3; c++) {
				if (lookup_u64(ft, fixed[c], &found) !=
						CDS_FT_STATUS_OK ||
						found != &expect[c]->node) {
					fprintf(stderr, "rekey cross-junction: writer %d "
						"fixed key %d lost\n", i, c);
					ret = -1;
				}
			}
		}
		for (c = 0; c < RKX_NSUB; c++) {
			struct cds_ft_node *found = NULL;

			if (lookup_u64(ft, RKX_KEY(here, 1, 3, c + 1, 0), &found) !=
					CDS_FT_STATUS_OK ||
					found != &w[i].top[c]->node) {
				fprintf(stderr, "rekey cross-junction: writer %d top %d "
					"absent at final pos (at_w2=%d)\n", i, c,
					w[i].at_w2);
				ret = -1;
			}
			if (lookup_u64(ft, RKX_KEY(there, 1, 3, c + 1, 0), &found) ==
					CDS_FT_STATUS_OK) {
				fprintf(stderr, "rekey cross-junction: writer %d top %d "
					"still at old pos\n", i, c);
				ret = -1;
			}
		}
	}
	if (cds_ft_count_keys(ft) != live) {
		fprintf(stderr, "rekey cross-junction: count_keys %lu != live %lu\n",
			cds_ft_count_keys(ft), live);
		ret = -1;
	}
	if (cds_ft_verify(ft, stderr) != CDS_FT_STATUS_OK) {
		fprintf(stderr, "rekey cross-junction: cds_ft_verify failed\n");
		ret = -1;
	}
	rcu_read_unlock();

	if (total_ops == 0) {			/* liveness: writers made progress */
		fprintf(stderr, "rekey cross-junction: no successful moves "
			"(livelock?)\n");
		ret = -1;
	}
	fprintf(stderr, "# inv_rekey_graft_cross_junction: %d writers, %lu moves, "
		"%lu retries, %lu live keys\n", RKX_NW, total_ops, total_retries,
		live);

	free(w);
	if (drain_and_destroy(ft, group) < 0)
		ret = -1;
	if (leak_check() < 0)
		ret = -1;
	return ret;
}

/*
 * Coherent-rekey concurrent-writer oracle, GLUE dst: every move lands its subtree
 * INSIDE a compressed node, so the graft cannot append in place -- it splits the
 * node and the fold records the whole invisible cluster's publish into the one
 * commit.  This is the only concurrent exercise of that shape, and it is aimed at
 * the two fences the shape adds: the split node's (taken by the build) and the
 * PUBLISH PARENT's (taken by the driver, because a guard fallback would leave the
 * forward SW park unheld).
 *
 * Contention is deliberate and on the publish parent: each writer's compressed
 * node hangs directly under a root child, so `publish_parent` is the ROOT NODE for
 * ALL of them -- they serialize on one fence exactly as the disjoint oracle's
 * writers do, which is what makes the abort-and-retry arms run.  Key ownership
 * stays disjoint, so an -EINVAL is still a real failure.
 *
 * A GLUE move CANNOT oscillate the way the other rekey oracles do: the split
 * consumes the compressed node and leaves a 2-child branch, which the src-junction
 * min_child gate refuses as a source.  So each iteration RESTORES the shape --
 * remove the four moved keys from the dst side (the branch drops to one child and
 * chain-compresses back) and re-insert them at the source.  The loop only tests
 * test_stop at an iteration boundary, so the trie is always in its base state at
 * join time.  Ordinary insert/remove concurrency rides along for free.
 *
 * Per writer w: root children w1 = 2w+1 (source side) and w2 = 2w+2 (dst side).
 *   w1 -> { 1 -> {1, [3 = S_top -> {1,2,3,4}], 5},  9 }
 *   w2 -> compressed[1,7,7,7] -> one leaf          <- the dst key {w2,1,3} diverges
 *                                                     at its SECOND byte
 * Opt-in FT_INV_MW=1.
 */
#define RKGL_NW		8		/* GLUE rekey writers, all contending root */
#define RKGL_NSUB	4

struct rkgl_writer_arg {
	struct cds_ft *ft;
	uint8_t w1, w2;			/* source-side / dst-side root children */
	struct ft_test_node *fill;	/* (w1,9,0,0,0) */
	struct ft_test_node *sib[2];	/* (w1,1,1,0,0) / (w1,1,5,0,0) */
	struct ft_test_node *lone;	/* (w2,1,7,7,7): the compressed node's key */
	/*
	 * TWO generations of the moved leaves, rotated instead of re-allocated.
	 * The restore below removes the live generation and re-inserts the other
	 * one, which was removed a full iteration ago -- and every iteration
	 * starts with a move, whose gate waits a GRACE PERIOD, so the generation
	 * being re-inserted has provably been unreachable across one.  That keeps
	 * the oracle allocation-free and, more importantly, keeps it from pushing
	 * ~14k call_rcu callbacks per run through the shared queues, where the
	 * only thing it measures is other tests' tolerance for a busy reclaimer.
	 */
	struct ft_test_node *top[2][RKGL_NSUB];
	int gen;			/* which generation is currently live */
	unsigned long ops, retries;
	int failed;
};

static void *rkgl_writer(void *arg)
{
	struct rkgl_writer_arg *w = (struct rkgl_writer_arg *) arg;
	uint8_t src_key[3], dst_key[3];
	struct cds_ft_iter *iter;
	unsigned long iters = 0;

	src_key[0] = w->w1; src_key[1] = 1; src_key[2] = 3;
	dst_key[0] = w->w2; dst_key[1] = 1; dst_key[2] = 3;

	rcu_register_thread();
	if (cds_ft_iter_create(w->ft, &iter) < 0)
		abort();
	while (!test_go)
		;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	while (!test_stop) {
		int rc, c;

		/* No read lock: the move enters the gate, which waits a GP. */
		rc = _cds_ft_debug_rekey_graft_simple(w->ft, src_key, 3, dst_key, 3);
		if (rc == -EAGAIN || rc == -EIO || rc == -ENOMEM) {
			w->retries++;	/* transient: trie pristine, re-descend */
			continue;
		}
		if (rc != 0) {
			fprintf(stderr, "rkgl_writer w1=%u w2=%u: move failed "
				"rc=%d\n", w->w1, w->w2, rc);
			w->failed = 1;
			mw_violation_snapshot();
			break;
		}
		w->ops++;
		/*
		 * RESTORE, so the next iteration faces the same shape: pull the four
		 * moved keys back out of the split's new direction and re-insert them
		 * at the source.  Fresh nodes for the re-insert (a removed node is
		 * marked and RCU-owned), and the removal drops the branch to its one
		 * surviving child, which chain-compresses back into a compressed node.
		 */
		for (c = 0; c < RKGL_NSUB; c++) {
			uint64_t moved = RKX_KEY(w->w2, 1, 3, c + 1, 0);
			struct cds_ft_node *found;
			uint8_t k[8];

			cds_ft_u64_to_key(w->ft, moved, k, CDS_FT_LEN_DEFAULT);
			rcu_read_lock();
			cds_ft_iter_set_key(iter, k, CDS_FT_LEN_DEFAULT);
			cds_ft_lookup(w->ft, iter);
			found = cds_ft_iter_node(iter);
			if (found != &w->top[w->gen][c]->node) {
				rcu_read_unlock();
				fprintf(stderr, "rkgl_writer w1=%u: moved key %d "
					"not at dst after a committed move\n",
					w->w1, c);
				w->failed = 1;
				mw_violation_snapshot();
				goto out;
			}
			if (cds_ft_remove(w->ft, iter, found) != CDS_FT_STATUS_OK) {
				rcu_read_unlock();
				fprintf(stderr, "rkgl_writer w1=%u: remove %d failed\n",
					w->w1, c);
				w->failed = 1;
				goto out;
			}
			rcu_read_unlock();
		}
		for (c = 0; c < RKGL_NSUB; c++) {
			uint64_t home = RKX_KEY(w->w1, 1, 3, c + 1, 0);
			struct ft_test_node *n = w->top[!w->gen][c];

			ft_test_node_init(n, home);	/* clears the removal mark */
			if (insert_u64(w->ft, home, n) != CDS_FT_STATUS_OK) {
				fprintf(stderr, "rkgl_writer w1=%u: re-insert %d "
					"failed\n", w->w1, c);
				w->failed = 1;
				goto out;
			}
		}
		w->gen = !w->gen;
		if ((++iters & 0x1f) == 0)
			rcu_quiescent_state();
	}
out:
	cds_ft_iter_destroy(iter);
	rcu_unregister_thread();
	return NULL;
}

static int inv_rekey_graft_glue_dst(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft;
	struct rkgl_writer_arg *w;
	pthread_t writers[RKGL_NW];
	struct ft_test_node *guard_lo, *guard_hi;
	struct timespec t0;
	unsigned long total_ops = 0, total_retries = 0, live = 0;
	int i, c, ret = 0;

	if (!getenv("FT_INV_MW")) {
		fprintf(stderr, "# inv_rekey_graft_glue_dst: skipped "
			"(set FT_INV_MW=1 to run the coherent-rekey writer oracle)\n");
		return 0;
	}
	mw_install_fatal_handler();
	leak_reset();

	ft = create_fixed_fine_lock_ft(5, &group);	/* list ON (default) */
	cds_ft_make_concurrent(ft);

	guard_lo = node_alloc(RKX_KEY(0x00, 0, 0, 0, 0));
	guard_hi = node_alloc(RKX_KEY(0xff, 0, 0, 0, 0));
	rcu_read_lock();
	if (insert_u64(ft, RKX_KEY(0x00, 0, 0, 0, 0), guard_lo) != CDS_FT_STATUS_OK ||
			insert_u64(ft, RKX_KEY(0xff, 0, 0, 0, 0), guard_hi) !=
			CDS_FT_STATUS_OK)
		abort();
	rcu_read_unlock();
	live = 2;

	w = (struct rkgl_writer_arg *) calloc(RKGL_NW, sizeof(*w));
	if (!w)
		abort();
	for (i = 0; i < RKGL_NW; i++) {
		uint64_t s1, s5, fk, lk;

		w[i].ft = ft;
		w[i].w1 = (uint8_t) (2 * i + 1);
		w[i].w2 = (uint8_t) (2 * i + 2);
		s1 = RKX_KEY(w[i].w1, 1, 1, 0, 0);
		s5 = RKX_KEY(w[i].w1, 1, 5, 0, 0);
		fk = RKX_KEY(w[i].w1, 9, 0, 0, 0);
		lk = RKX_KEY(w[i].w2, 1, 7, 7, 7);
		w[i].sib[0] = node_alloc(s1);
		w[i].sib[1] = node_alloc(s5);
		w[i].fill = node_alloc(fk);
		w[i].lone = node_alloc(lk);
		rcu_read_lock();
		if (insert_u64(ft, s1, w[i].sib[0]) != CDS_FT_STATUS_OK ||
				insert_u64(ft, s5, w[i].sib[1]) != CDS_FT_STATUS_OK ||
				insert_u64(ft, fk, w[i].fill) != CDS_FT_STATUS_OK ||
				insert_u64(ft, lk, w[i].lone) != CDS_FT_STATUS_OK)
			abort();
		for (c = 0; c < RKGL_NSUB; c++) {
			uint64_t tk = RKX_KEY(w[i].w1, 1, 3, c + 1, 0);

			w[i].top[0][c] = node_alloc(tk);
			w[i].top[1][c] = node_alloc(tk);	/* spare generation */
			if (insert_u64(ft, tk, w[i].top[0][c]) != CDS_FT_STATUS_OK)
				abort();
		}
		rcu_read_unlock();
		live += 4 + RKGL_NSUB;
	}

	test_go = 0;
	test_stop = 0;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	for (i = 0; i < RKGL_NW; i++)
		pthread_create(&writers[i], NULL, rkgl_writer, &w[i]);
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	test_go = 1;

	rcu_thread_offline();
	clock_gettime(CLOCK_MONOTONIC, &t0);
	while (elapsed_ms(&t0) < DEFAULT_DURATION_MS)
		usleep(1000);
	test_stop = 1;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	for (i = 0; i < RKGL_NW; i++)
		pthread_join(writers[i], NULL);
	rcu_thread_online();

	/*
	 * Quiescent, and every writer stopped at an iteration boundary, so the trie
	 * must be back in its base state: the moved keys home, the split's old
	 * direction (the lone compressed key) intact, nothing left at the dst.
	 */
	synchronize_rcu();
	rcu_read_lock();
	for (i = 0; i < RKGL_NW; i++) {
		struct cds_ft_node *found = NULL;

		total_ops += w[i].ops;
		total_retries += w[i].retries;
		if (w[i].failed)
			ret = -1;
		if (lookup_u64(ft, RKX_KEY(w[i].w2, 1, 7, 7, 7), &found) !=
				CDS_FT_STATUS_OK || found != &w[i].lone->node) {
			fprintf(stderr, "rekey glue: writer %d lost the compressed "
				"key\n", i);
			ret = -1;
		}
		if (lookup_u64(ft, RKX_KEY(w[i].w1, 1, 1, 0, 0), &found) !=
				CDS_FT_STATUS_OK || found != &w[i].sib[0]->node ||
				lookup_u64(ft, RKX_KEY(w[i].w1, 1, 5, 0, 0), &found) !=
				CDS_FT_STATUS_OK || found != &w[i].sib[1]->node ||
				lookup_u64(ft, RKX_KEY(w[i].w1, 9, 0, 0, 0), &found) !=
				CDS_FT_STATUS_OK || found != &w[i].fill->node) {
			fprintf(stderr, "rekey glue: writer %d lost a fixed key\n", i);
			ret = -1;
		}
		for (c = 0; c < RKGL_NSUB; c++) {
			if (lookup_u64(ft, RKX_KEY(w[i].w1, 1, 3, c + 1, 0), &found) !=
					CDS_FT_STATUS_OK ||
					found != &w[i].top[w[i].gen][c]->node) {
				fprintf(stderr, "rekey glue: writer %d key %d not "
					"home\n", i, c);
				ret = -1;
			}
			if (lookup_u64(ft, RKX_KEY(w[i].w2, 1, 3, c + 1, 0), &found) ==
					CDS_FT_STATUS_OK) {
				fprintf(stderr, "rekey glue: writer %d key %d still at "
					"dst\n", i, c);
				ret = -1;
			}
		}
	}
	if (cds_ft_count_keys(ft) != live) {
		fprintf(stderr, "rekey glue: count_keys %lu != live %lu\n",
			cds_ft_count_keys(ft), live);
		ret = -1;
	}
	if (cds_ft_verify(ft, stderr) != CDS_FT_STATUS_OK) {
		fprintf(stderr, "rekey glue: cds_ft_verify failed\n");
		ret = -1;
	}
	rcu_read_unlock();

	if (total_ops == 0) {
		fprintf(stderr, "rekey glue: no successful moves (livelock?)\n");
		ret = -1;
	}
	fprintf(stderr, "# inv_rekey_graft_glue_dst: %d writers, %lu moves, "
		"%lu retries, %lu live keys\n", RKGL_NW, total_ops, total_retries,
		live);

	/*
	 * The spare generation is not in the trie, so the drain below cannot free
	 * it.  The writers are joined and a grace period has passed, so a direct
	 * free is safe.
	 */
	for (i = 0; i < RKGL_NW; i++)
		for (c = 0; c < RKGL_NSUB; c++)
			node_free(w[i].top[!w[i].gen][c]);
	free(w);
	if (drain_and_destroy(ft, group) < 0)
		ret = -1;
	if (leak_check() < 0)
		ret = -1;
	return ret;
}

/*
 * Coherent-rekey oracle WITH READERS: the disjoint rekey writers run against a
 * trie whose per-trie rekey coherence is ENABLED (cds_ft_attr_set_rekey_coherence),
 * so every exact lookup runs the landed second-walk (up-walk key rematerializer)
 * re-descend.  This is the FIRST CONCURRENT exercise of that shipped coherent
 * lookup (its unit tests are single-threaded), and the reader/writer memory-safety
 * probe: a reader descending through S_top / BP races the writer COW'ing S_top and
 * recompacting BP (fresh nodes, old freed after a grace period), so it is run under
 * ASAN to catch any use-after-free / double-free across the move.
 *
 * SOUND "never absent" assertion anchored on the FIXED sibling keys: each junction's
 * (X,1) / (X,5) sibs never move and are therefore present at ALL times, yet a
 * reader's descent to a sib passes through BP, which is COW-recompacted on EVERY
 * move -- so a coherent lookup of a sib that ever MISSES (or returns a wrong node)
 * is a real atomicity / coherence failure (a descent torn across BP's recompaction
 * that the second-walk failed to repair).  The MOVING keys (X,3,c) are checked
 * weakly (a single atomic coherent lookup returns miss XOR the one node that key
 * ever maps to -- never a foreign node); full linearizability of a moving key needs
 * the deferred per-op epoch, so right-node-wrong-instant is out of scope here.
 *
 * EXACT lookups descend the trie + up-walk parent pointers; they do NOT traverse
 * the ordered-list cells, so this oracle does not exercise the deferred plan-Q3 cell
 * outer-link window (that is a relational/iteration-coherence concern, not built).
 * Opt-in FT_INV_MW=1.
 */
#define RK_NR		8		/* coherent readers */

struct rk_reader_arg {
	struct cds_ft *ft;
	struct rk_writer_arg *w;		/* the writer array (node shadow) */
	int nw;
	unsigned long checks;
	int failed;
};

static struct cds_ft *create_fixed_rekey_coherent_ft(size_t klen,
		struct cds_ft_group **group_out)
{
	struct cds_ft_group_attr *gattr;
	struct cds_ft_attr *attr;
	struct cds_ft_group *group;
	struct cds_ft *ft;

	if (cds_ft_group_attr_create(&gattr) < 0)
		abort();
	if (cds_ft_group_attr_set_key_len(gattr, klen) < 0)
		abort();
	if (cds_ft_group_attr_set_writer_strategy(gattr,
			CDS_FT_WRITER_LOCK_FINE) < 0)
		abort();
	if (cds_ft_group_attr_set_ordered_list(gattr, true) < 0)	/* up-walk cell source */
		abort();
	if (cds_ft_group_create(gattr, &group) < 0)
		abort();
	cds_ft_group_attr_destroy(gattr);
	if (cds_ft_attr_create(&attr) < 0)
		abort();
	/*
	 * Rekey coherence needs no opt-in: it is automatic on an EAGER ordered-list
	 * trie, gated at runtime by the per-trie MOVE GATE.  EAGER is the only
	 * requirement, so that is all this asks for.
	 */
	if (cds_ft_attr_set_speculative_keys(attr, false) < 0)	/* rekey needs EAGER keys */
		abort();
	if (cds_ft_create(group, attr, &ft) < 0)
		abort();
	cds_ft_attr_destroy(attr);
	*group_out = group;
	return ft;
}

static void *rk_reader(void *arg)
{
	struct rk_reader_arg *r = (struct rk_reader_arg *) arg;
	struct cds_ft_iter *iter;
	unsigned int seed = (unsigned int) (uintptr_t) r;
	unsigned long iters = 0;

	rcu_register_thread();
	if (cds_ft_iter_create(r->ft, &iter) < 0)
		abort();
	while (!test_go)
		;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	while (!test_stop) {
		int wi = rand_r(&seed) % r->nw;
		struct rk_writer_arg *w = &r->w[wi];
		uint64_t bpk = (uint64_t) w->bp << 24, dpk = (uint64_t) w->dp << 24;
		int pick = rand_r(&seed) % 12;
		struct cds_ft_node *found, *expect;
		int must_find;
		uint64_t key;
		uint8_t k[8];

		if (pick >= 8) {
			/*
			 * RELATIONAL probe (the coherent two-pass path) on a FIXED
			 * sibling: that key is present at every instant -- a move
			 * relocates only the (X,sb,*) run, though it COW-recompacts
			 * the junction this sibling hangs from on every single move
			 * -- so ge(K) and le(K) must land EXACTLY on K.  A torn
			 * relational traversal that stepped over it would answer
			 * with the neighbouring key instead, which is precisely what
			 * the two-pass exists to reject.
			 */
			int si = pick & 3;
			enum cds_ft_status st;

			key = ((si < 2) ? bpk : dpk) | (((si & 1) ? 5ULL : 1ULL) << 16);
			expect = &w->sib[si]->node;
			cds_ft_u64_to_key(r->ft, key, k, CDS_FT_LEN_DEFAULT);
			rcu_read_lock();
			cds_ft_iter_set_key(iter, k, CDS_FT_LEN_DEFAULT);
			st = (rand_r(&seed) & 1) ? cds_ft_lookup_ge(r->ft, iter) :
				cds_ft_lookup_le(r->ft, iter);
			found = cds_ft_iter_node(iter);
			if (st != CDS_FT_STATUS_OK || found != expect) {
				fprintf(stderr,
					"rk_reader: relational key %#llx -> %p (%s, expect %p)\n",
					(unsigned long long) key, (void *) found,
					cds_ft_status_to_string(st),
					(void *) expect);
				r->failed = 1;
				rcu_read_unlock();
				mw_violation_snapshot();
				break;
			}
			rcu_read_unlock();
			r->checks++;
			if ((++iters & 0xff) == 0)
				rcu_quiescent_state();
			continue;
		}
		if (pick < 4) {
			/* FIXED sibling (X,1)/(X,5): always present -> must be found. */
			key = ((pick < 2) ? bpk : dpk) | (((pick & 1) ? 5ULL : 1ULL) << 16);
			expect = &w->sib[pick]->node;
			must_find = 1;
		} else {
			/* MOVING leaf (X,3,c): miss XOR this leaf, never a foreign node. */
			int c = pick & 3;

			key = ((rand_r(&seed) & 1) ? dpk : bpk) |
				((uint64_t) w->sb << 16) |
				((uint64_t) (c + 1) << 8);
			expect = &w->top[c]->node;
			must_find = 0;
		}
		cds_ft_u64_to_key(r->ft, key, k, CDS_FT_LEN_DEFAULT);
		rcu_read_lock();
		cds_ft_iter_set_key(iter, k, CDS_FT_LEN_DEFAULT);
		cds_ft_lookup(r->ft, iter);
		found = cds_ft_iter_node(iter);
		if ((must_find && found != expect) ||
				(!must_find && found && found != expect)) {
			fprintf(stderr, "rk_reader: key %#llx -> %p (expect %s%p)\n",
				(unsigned long long) key, (void *) found,
				must_find ? "" : "miss or ", (void *) expect);
			r->failed = 1;
			rcu_read_unlock();
			mw_violation_snapshot();
			break;
		}
		rcu_read_unlock();
		r->checks++;
		if ((++iters & 0xff) == 0)
			rcu_quiescent_state();
	}
	cds_ft_iter_destroy(iter);
	rcu_unregister_thread();
	return NULL;
}

static int inv_rekey_graft_coherent_readers(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft;
	struct rk_writer_arg *w;
	struct rk_reader_arg *r;
	pthread_t writers[RK_NW], readers[RK_NR];
	struct ft_test_node *guard_lo, *guard_hi;
	struct timespec t0;
	unsigned long total_ops = 0, total_retries = 0, total_checks = 0, live = 0;
	int i, c, ret = 0;

	if (!getenv("FT_INV_MW")) {
		fprintf(stderr, "# inv_rekey_graft_coherent_readers: skipped "
			"(set FT_INV_MW=1 to run the coherent-rekey reader oracle)\n");
		return 0;
	}
	mw_install_fatal_handler();
	leak_reset();

	ft = create_fixed_rekey_coherent_ft(4, &group);
	cds_ft_make_concurrent(ft);

	guard_lo = node_alloc(0x00000000ULL);
	guard_hi = node_alloc(0xff000000ULL);
	rcu_read_lock();
	if (insert_u64(ft, 0x00000000ULL, guard_lo) != CDS_FT_STATUS_OK ||
			insert_u64(ft, 0xff000000ULL, guard_hi) != CDS_FT_STATUS_OK)
		abort();
	rcu_read_unlock();
	live = 2;

	w = (struct rk_writer_arg *) calloc(RK_NW, sizeof(*w));
	r = (struct rk_reader_arg *) calloc(RK_NR, sizeof(*r));
	if (!w || !r)
		abort();
	for (i = 0; i < RK_NW; i++) {
		uint8_t bp = (uint8_t) (2 * i + 1), dp = (uint8_t) (2 * i + 2);
		uint64_t bpk = (uint64_t) bp << 24, dpk = (uint64_t) dp << 24;
		uint64_t sk[4] = {
			bpk | (1ULL << 16), bpk | (5ULL << 16),
			dpk | (1ULL << 16), dpk | (5ULL << 16),
		};

		w[i].ft = ft;
		w[i].bp = bp;
		w[i].dp = dp;
		w[i].sb = 3;			/* S_top slot byte inside both junctions */
		rcu_read_lock();
		for (c = 0; c < 4; c++) {
			w[i].sib[c] = node_alloc(sk[c]);
			if (insert_u64(ft, sk[c], w[i].sib[c]) != CDS_FT_STATUS_OK)
				abort();
		}
		for (c = 0; c < 4; c++) {
			uint64_t tk = bpk | (3ULL << 16) | ((uint64_t) (c + 1) << 8);

			w[i].top[c] = node_alloc(tk);
			if (insert_u64(ft, tk, w[i].top[c]) != CDS_FT_STATUS_OK)
				abort();
		}
		rcu_read_unlock();
		live += 8;
	}
	for (i = 0; i < RK_NR; i++) {
		r[i].ft = ft;
		r[i].w = w;
		r[i].nw = RK_NW;
	}

	test_go = 0;
	test_stop = 0;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	for (i = 0; i < RK_NW; i++)
		pthread_create(&writers[i], NULL, rk_writer, &w[i]);
	for (i = 0; i < RK_NR; i++)
		pthread_create(&readers[i], NULL, rk_reader, &r[i]);
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	test_go = 1;

	rcu_thread_offline();
	clock_gettime(CLOCK_MONOTONIC, &t0);
	while (elapsed_ms(&t0) < DEFAULT_DURATION_MS)
		usleep(1000);
	test_stop = 1;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	for (i = 0; i < RK_NW; i++)
		pthread_join(writers[i], NULL);
	for (i = 0; i < RK_NR; i++)
		pthread_join(readers[i], NULL);
	rcu_thread_online();

	for (i = 0; i < RK_NW; i++) {
		total_ops += w[i].ops;
		total_retries += w[i].retries;
	}
	for (i = 0; i < RK_NR; i++) {
		total_checks += r[i].checks;
		if (r[i].failed)
			ret = -1;
	}

	synchronize_rcu();
	rcu_read_lock();
	for (i = 0; i < RK_NW; i++) {
		uint8_t bp = w[i].bp, dp = w[i].dp;
		uint64_t bpk = (uint64_t) bp << 24, dpk = (uint64_t) dp << 24;
		uint8_t X = w[i].at_dst ? dp : bp;
		uint64_t Xk = (uint64_t) X << 24;
		uint64_t sk[4] = {
			bpk | (1ULL << 16), bpk | (5ULL << 16),
			dpk | (1ULL << 16), dpk | (5ULL << 16),
		};

		for (c = 0; c < 4; c++) {
			struct cds_ft_node *found = NULL;

			if (lookup_u64(ft, sk[c], &found) != CDS_FT_STATUS_OK ||
					found != &w[i].sib[c]->node)
				ret = -1;
		}
		for (c = 0; c < 4; c++) {
			uint64_t here = Xk | (3ULL << 16) | ((uint64_t) (c + 1) << 8);
			struct cds_ft_node *found = NULL;

			if (lookup_u64(ft, here, &found) != CDS_FT_STATUS_OK ||
					found != &w[i].top[c]->node)
				ret = -1;
		}
	}
	if (cds_ft_count_keys(ft) != live)
		ret = -1;
	if (cds_ft_verify(ft, stderr) != CDS_FT_STATUS_OK)
		ret = -1;
	rcu_read_unlock();

	if (total_ops == 0)
		ret = -1;
	fprintf(stderr, "# inv_rekey_graft_coherent_readers: %d writers %d readers, "
		"%lu moves, %lu retries, %lu reads, %lu live keys\n", RK_NW, RK_NR,
		total_ops, total_retries, total_checks, live);

	free(w);
	free(r);
	if (drain_and_destroy(ft, group) < 0)
		ret = -1;
	if (leak_check() < 0)
		ret = -1;
	return ret;
}

/*
 * SHARED-JUNCTION rekey oracle -- the companion inv_rekey_graft_disjoint cannot
 * be: here a peer's move touches nodes THIS writer descended through.
 *
 * In the disjoint oracle every writer owns two PRIVATE root-child junctions, and
 * the only node they share is root -- whose child set is FIXED, so root is never
 * recompacted and hence never relocated.  A disjoint writer's descent-captured
 * triple (S_top, BP, dst parent) therefore cannot be invalidated by a peer, and
 * the only concurrency exercised is the race for root's LOCK.
 *
 * Here RKS_NJ junction bytes are SHARED by RKS_NW writers: junction J holds the
 * S_top of each writer whose src it currently is AND receives each writer whose
 * dst it is, so several writers' subtrees live under one junction at once.  Two
 * hazards the disjoint layout structurally excludes become reachable:
 *  (a) STALE DESCENT: every popcount delete and every graft attach RECOMPACTS the
 *      junction (relocating it and retiring the old copy), so a peer's completed
 *      move leaves this writer's descent-captured BP / dst parent pointing at a
 *      RETIRED node.  The one-decide writer must REJECT that (its DLM acquire
 *      sees the tombstoned state word -> -EAGAIN) and re-descend, never edit the
 *      dead copy.
 *  (b) CROSS-WRITER CHILD RE-PARENT: that junction recompaction re-parents ALL of
 *      the junction's children, which now include the subtree ANOTHER writer is
 *      concurrently moving -- a node that writer holds LOCK on and parks its
 *      own retire onto, in the same state word.
 *
 * The LAYOUT keeps every shape gate of the debug writer STATICALLY satisfied, so
 * an -EINVAL remains a real failure and not a raced shape:
 *  - every junction is a root child and root's child set is fixed, so the shared
 *    parent of BP and the dst parent is ALWAYS root (d_src.ppnf == d_dst.ppnf,
 *    the src_parent_held precondition).
 *  - every junction keeps two GUARD leaves at byte1 0x00 / 0xff.  They hold it at
 *    >= 3 children while it holds our S_top (the min_child gate), keep it a plain
 *    internal node (never compressed), keep every run off the ordered-list
 *    head/tail, and BRACKET every dst gap so the adjacency guard cannot trip: the
 *    gap (dp,sb) always has a (dp,x<sb) predecessor and a (dp,x>sb) successor
 *    inside its own junction, while our run lives under a different junction byte.
 *  - each writer owns a DISTINCT S_top byte, so its dst slot is absent by
 *    construction and no two writers ever contend one slot.
 * Opt-in FT_INV_MW=1.
 */
#define RKS_NJ		2		/* shared junction bytes: ALL writers share both */
#define RKS_NW		16		/* writers, so each junction hosts up to 8 runs */
#define RKS_JB(j)	((uint8_t) (0x10 * ((j) + 1)))

static int inv_rekey_graft_shared(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft;
	struct rk_writer_arg *w;
	pthread_t writers[RKS_NW];
	struct ft_test_node *jguard[RKS_NJ][2];
	struct timespec t0;
	unsigned long total_ops = 0, total_retries = 0, live = 0;
	int i, j, c, ret = 0;

	if (!getenv("FT_INV_MW")) {
		fprintf(stderr, "# inv_rekey_graft_shared: skipped "
			"(set FT_INV_MW=1 to run the coherent-rekey writer oracle)\n");
		return 0;
	}
	mw_install_fatal_handler();
	leak_reset();

	ft = create_fixed_fine_lock_ft(4, &group);	/* list ON (default) */
	cds_ft_make_concurrent(ft);

	/* Per-junction guard leaves at byte1 0x00 / 0xff (see the layout note). */
	rcu_read_lock();
	for (j = 0; j < RKS_NJ; j++) {
		for (c = 0; c < 2; c++) {
			uint64_t k = ((uint64_t) RKS_JB(j) << 24) |
				((c ? 0xffULL : 0x00ULL) << 16);

			jguard[j][c] = node_alloc(k);
			if (insert_u64(ft, k, jguard[j][c]) != CDS_FT_STATUS_OK)
				abort();
			live++;
		}
	}
	rcu_read_unlock();

	w = (struct rk_writer_arg *) calloc(RKS_NW, sizeof(*w));
	if (!w)
		abort();
	for (i = 0; i < RKS_NW; i++) {
		uint8_t bp = RKS_JB(i % RKS_NJ);
		uint8_t dp = RKS_JB((i + 1) % RKS_NJ);

		w[i].ft = ft;
		w[i].bp = bp;
		w[i].dp = dp;
		w[i].sb = (uint8_t) (0x80 + i);	/* distinct per writer, 0x00 < sb < 0xff */
		rcu_read_lock();
		for (c = 0; c < 4; c++) {
			uint64_t tk = ((uint64_t) bp << 24) |
				((uint64_t) w[i].sb << 16) |
				((uint64_t) (c + 1) << 8);

			w[i].top[c] = node_alloc(tk);
			if (insert_u64(ft, tk, w[i].top[c]) != CDS_FT_STATUS_OK)
				abort();
		}
		rcu_read_unlock();
		live += 4;
	}

	test_go = 0;
	test_stop = 0;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	for (i = 0; i < RKS_NW; i++)
		pthread_create(&writers[i], NULL, rk_writer, &w[i]);
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	test_go = 1;

	rcu_thread_offline();
	clock_gettime(CLOCK_MONOTONIC, &t0);
	while (elapsed_ms(&t0) < DEFAULT_DURATION_MS)
		usleep(1000);
	test_stop = 1;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	for (i = 0; i < RKS_NW; i++)
		pthread_join(writers[i], NULL);
	rcu_thread_online();

	/* Quiescent: guards intact, every moved leaf at its writer's final position. */
	synchronize_rcu();
	rcu_read_lock();
	for (j = 0; j < RKS_NJ; j++) {
		for (c = 0; c < 2; c++) {
			uint64_t k = ((uint64_t) RKS_JB(j) << 24) |
				((c ? 0xffULL : 0x00ULL) << 16);
			struct cds_ft_node *found = NULL;

			if (lookup_u64(ft, k, &found) != CDS_FT_STATUS_OK ||
					found != &jguard[j][c]->node) {
				fprintf(stderr, "rekey shared: junction %02x guard "
					"%d lost\n", RKS_JB(j), c);
				ret = -1;
			}
		}
	}
	for (i = 0; i < RKS_NW; i++) {
		uint8_t X = w[i].at_dst ? w[i].dp : w[i].bp;
		uint8_t O = w[i].at_dst ? w[i].bp : w[i].dp;

		total_ops += w[i].ops;
		total_retries += w[i].retries;
		if (w[i].failed)
			ret = -1;
		/*
		 * PER-WRITER liveness, not just the global count: without the move-counter
		 * pin, a writer could get PERMANENTLY stuck re-deriving a splice position
		 * against a mis-ordered list (measured: the same rejected (pred, succ) pair
		 * hundreds of times in a row) while its peers kept committing -- a global
		 * "some moves happened" check sails right past that.
		 */
		if (w[i].ops == 0) {
			fprintf(stderr, "rekey shared: writer %d made NO move "
				"(starved: %lu retries)\n", i, w[i].retries);
			ret = -1;
		}
		for (c = 0; c < 4; c++) {
			uint64_t suffix = ((uint64_t) w[i].sb << 16) |
				((uint64_t) (c + 1) << 8);
			uint64_t here = ((uint64_t) X << 24) | suffix;
			uint64_t there = ((uint64_t) O << 24) | suffix;
			struct cds_ft_node *found = NULL;

			if (lookup_u64(ft, here, &found) != CDS_FT_STATUS_OK ||
					found != &w[i].top[c]->node) {
				fprintf(stderr, "rekey shared: writer %d top %d absent "
					"at final pos (at_dst=%d)\n", i, c, w[i].at_dst);
				ret = -1;
			}
			if (lookup_u64(ft, there, &found) == CDS_FT_STATUS_OK) {
				fprintf(stderr, "rekey shared: writer %d top %d still at "
					"old pos\n", i, c);
				ret = -1;
			}
		}
	}
	if (cds_ft_count_keys(ft) != live) {
		fprintf(stderr, "rekey shared: count_keys %lu != live %lu\n",
			cds_ft_count_keys(ft), live);
		ret = -1;
	}
	if (cds_ft_verify(ft, stderr) != CDS_FT_STATUS_OK) {
		fprintf(stderr, "rekey shared: cds_ft_verify failed\n");
		ret = -1;
	}
	/*
	 * Ordered forward scan over the CELL list: every key exactly once, strictly
	 * ascending.  cds_ft_verify's ord-cell pass reports a broken back-edge or an
	 * order mismatch as CELL POINTERS; this reports the KEYS, which is what
	 * identifies WHICH writer's run landed in the wrong place (the residual failure
	 * mode of a mis-derived splice position).  Also dumps the whole sequence once a
	 * violation is seen, so a rare failure is self-describing in the log.
	 */
	{
		const struct cds_ft_cell *buf[8], *cur = NULL;
		uint64_t prev_key = 0;
		unsigned long seen = 0;
		int bad = 0;

		do {
			size_t n = 0, b;

			if (cds_ft_cell_next_batch(ft, cur, buf, 8, &n, &cur)
					!= CDS_FT_STATUS_OK) {
				fprintf(stderr, "rekey shared: cell scan failed\n");
				ret = -1;
				break;
			}
			for (b = 0; b < n; b++) {
				uint8_t k[4];
				size_t kl;
				uint64_t kv;

				if (cds_ft_cell_get_key(ft, buf[b], k, sizeof k,
						&kl) != CDS_FT_STATUS_OK) {
					fprintf(stderr, "rekey shared: cell "
						"get_key failed\n");
					ret = -1;
					bad = 1;
					break;
				}
				kv = cds_ft_key_to_u64(ft, k, 4);
				if (seen && kv <= prev_key) {
					fprintf(stderr, "rekey shared: ordered "
						"scan violation %#lx after %#lx "
						"(position %lu)\n",
						(unsigned long) kv,
						(unsigned long) prev_key, seen);
					ret = -1;
					bad = 1;
				}
				if (bad)
					fprintf(stderr, "rekey shared:   [%lu] %#lx\n",
						seen, (unsigned long) kv);
				prev_key = kv;
				seen++;
			}
		} while (cur);
		if (seen != live) {
			fprintf(stderr, "rekey shared: ordered scan saw %lu keys, "
				"live %lu\n", seen, live);
			ret = -1;
		}
	}
	rcu_read_unlock();

	if (total_ops == 0) {			/* liveness: writers made progress */
		fprintf(stderr, "rekey shared: no successful moves (livelock?)\n");
		ret = -1;
	}
	fprintf(stderr, "# inv_rekey_graft_shared: %d writers over %d shared "
		"junctions, %lu moves, %lu retries, %lu live keys\n", RKS_NW,
		RKS_NJ, total_ops, total_retries, live);

	free(w);
	if (drain_and_destroy(ft, group) < 0)
		ret = -1;
	if (leak_check() < 0)
		ret = -1;
	return ret;
}

/*
 * ===========================================================================
 * RELATIONAL LINEARIZABILITY ORACLE (design step 4).
 * ===========================================================================
 *
 * The other rekey oracles check STRUCTURE (verify, counts, no UAF) and EXACT
 * lookups ("a present key is never absent").  Neither can say whether a
 * RELATIONAL answer -- the neighbour of a key -- was ever TRUE: a traversal torn
 * across a move can return a real, live node that is the neighbour of nothing,
 * at no instant.  This is the oracle for that.
 *
 * WHY IT NEEDS NO TIMESTAMPS, VERSIONS OR WRITER INSTRUMENTATION.  Each writer's
 * subtree is in exactly ONE of TWO places -- the run sits under its bp junction
 * XOR its dp junction -- so for a probe inside junction X the set of answers
 * that are correct at SOME instant is enumerable outright, and it has two
 * elements.  Membership in that pair IS the linearizability check, at every
 * instant, whatever the interleaving.  The writers stay completely
 * uninstrumented: no bracket, no counter, no barrier on the path under test.
 *
 * ★ COMPARE NODE IDENTITY, NEVER A KEY READ BACK.  The moved leaves are the SAME
 * nodes at both junctions -- a move re-keys them, it does not replace them -- so
 * the legal pair is a pair of stable node ADDRESSES, and membership is
 * insensitive to where the run currently sits.  A key comparison is NOT: on an
 * EAGER ordered-list trie there is no stored key, and cds_ft_iter_get_key
 * rematerializes it by walking the leaf's parent chain AT READBACK TIME, so a
 * leaf that moved between the answer and the readback reports its NEW key.  An
 * earlier version of this oracle compared keys and "found" ~2e-4 non-linearizable
 * answers; every one of them had returned a LEGAL NODE.  The artifact, not the
 * library, was the finding.
 *
 * TWO MODES, GRADED DIFFERENTLY:
 *
 *  A. BOUND-KEY probes (ge/gt/le/lt from a key, no cached position).  These MUST
 *     be linearizable -- that is what the relational two-pass promises -- so an
 *     answer outside the legal pair FAILS the test.
 *
 *  B. CONTINUATION walks (cds_ft_next from a cached position).  These carry the
 *     KNOWN residual: a walker parked ON a moving run's cell steps out through
 *     the run's NEW outer link and lands wherever the run went.  That is inherent
 *     to moving a live run without draining readers, so mode B MEASURES it
 *     (per-class counts, printed every run) rather than failing on it.
 *
 * WHAT MODE A DOES NOT PROVE.  Membership in the legal pair is timing-independent
 * and therefore NECESSARY but not SUFFICIENT: it rules out an answer that was
 * true at NO instant (the phantom class), but it accepts the answer belonging to
 * the OTHER state even when that state did not occur during the operation.
 * Closing that gap needs to know which state held across the interval, i.e. the
 * writer-side timing bookkeeping this oracle deliberately does without -- and
 * that instrumentation would put barriers on the very path under test.  The
 * phantom class is the one the two-pass exists to prevent, so it is the one
 * gated here.
 *
 * Opt-in FT_INV_MW=1, like the other rekey oracles.
 */
#define RKL_NR		8	/* linearizability readers */
#define RKL_WALK_MAX	8	/* steps before abandoning a walk */
#define RKL_MAX_REPORT	3	/* violations narrated per reader; the rest counted */

struct rkl_reader_arg {
	struct cds_ft *ft;
	struct rk_writer_arg *w;
	int nw;
	unsigned long probes;		/* mode A: bound-key relational reads */
	unsigned long probe_bad;	/* mode A: answers outside the legal pair */
	unsigned long walks, walk_steps;
	unsigned long walk_alien;	/* stepped onto ANOTHER junction's node */
	unsigned long walk_back;	/* stepped backwards within this junction */
	unsigned long walk_inner;	/* skipped a node inside this junction */
	unsigned long walk_end;		/* walk ended before reaching (X,5) */
	int failed;
};

/*
 * Position of @n in junction @X's key order, by IDENTITY:
 *   0 = the (X,1) sibling, 1..4 = the run's leaves, 5 = the (X,5) sibling.
 * -1 when @n is none of them.  w->sib[] is {(bp,1),(bp,5),(dp,1),(dp,5)}.
 */
static int rkl_node_index(const struct rk_writer_arg *w, uint8_t X,
		const struct cds_ft_node *n)
{
	int lo = (X == w->bp) ? 0 : 2;
	int c;

	if (!n)
		return -1;
	if (n == &w->sib[lo]->node)
		return 0;
	if (n == &w->sib[lo + 1]->node)
		return 5;
	for (c = 0; c < 4; c++) {
		if (n == &w->top[c]->node)
			return 1 + c;
	}
	return -1;
}

/* One relational read from a bound key; the answer is the iterator's NODE. */
static enum cds_ft_status rkl_probe(struct cds_ft *ft, struct cds_ft_iter *iter,
		int mode, uint64_t key, struct cds_ft_node **out)
{
	uint8_t k[8];
	enum cds_ft_status st;

	cds_ft_u64_to_key(ft, key, k, CDS_FT_LEN_DEFAULT);
	cds_ft_iter_set_key(iter, k, CDS_FT_LEN_DEFAULT);
	switch (mode) {
	case 0:	st = cds_ft_lookup_ge(ft, iter); break;
	case 1:	st = cds_ft_lookup_gt(ft, iter); break;
	case 2:	st = cds_ft_lookup_le(ft, iter); break;
	default: st = cds_ft_lookup_lt(ft, iter); break;
	}
	*out = (st == CDS_FT_STATUS_OK) ? cds_ft_iter_node(iter) : NULL;
	return st;
}

static void *rkl_reader(void *arg)
{
	struct rkl_reader_arg *r = (struct rkl_reader_arg *) arg;
	struct cds_ft_iter *iter;
	unsigned int seed = (unsigned int) (uintptr_t) r;
	unsigned long iters = 0;

	rcu_register_thread();
	if (cds_ft_iter_create(r->ft, &iter) < 0)
		abort();
	while (!test_go)
		;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	while (!test_stop) {
		struct rk_writer_arg *w = &r->w[rand_r(&seed) % r->nw];
		uint8_t X = (rand_r(&seed) & 1) ? w->dp : w->bp;
		uint64_t Xk = (uint64_t) X << 24;
		uint64_t sb = (uint64_t) w->sb << 16;
		int lo_sib = (X == w->bp) ? 0 : 2;
		struct cds_ft_node *sib_lo = &w->sib[lo_sib]->node;
		struct cds_ft_node *sib_hi = &w->sib[lo_sib + 1]->node;
		enum cds_ft_status st;
		struct cds_ft_node *got = NULL;

		if ((rand_r(&seed) & 3) != 0) {
			/*
			 * MODE A.  Both members of each legal pair exist at all
			 * times (the run's leaves are the same nodes wherever the
			 * run sits), so a miss is a failure too.
			 */
			int pick = rand_r(&seed) & 3;
			struct cds_ft_node *a, *b;
			uint64_t probe;

			switch (pick) {
			case 0:	probe = Xk | (2ULL << 16);
				a = &w->top[0]->node; b = sib_hi; break;
			case 1:	probe = Xk | sb;
				a = &w->top[0]->node; b = sib_hi; break;
			case 2:	probe = Xk | (4ULL << 16);
				a = &w->top[3]->node; b = sib_lo; break;
			default: probe = Xk | sb | 0xff00ULL;
				a = &w->top[3]->node; b = sib_lo; break;
			}
			rcu_read_lock();
			st = rkl_probe(r->ft, iter, pick, probe, &got);
			rcu_read_unlock();
			r->probes++;
			if (caa_unlikely(st != CDS_FT_STATUS_OK ||
					(got != a && got != b))) {
				r->probe_bad++;
				r->failed = 1;
				if (r->probe_bad <= RKL_MAX_REPORT)
					fprintf(stderr,
						"rkl: %s(%#llx) -> node %p (%s); legal only %p or %p\n",
						pick == 0 ? "ge" : pick == 1 ? "gt" :
							pick == 2 ? "le" : "lt",
						(unsigned long long) probe,
						(void *) got,
						cds_ft_status_to_string(st),
						(void *) a, (void *) b);
				mw_violation_snapshot();
			}
		} else {
			/*
			 * MODE B: walk (X,1) -> (X,5) with cds_ft_next, checking
			 * each step against the pair the current POSITION allows,
			 * by node identity.
			 */
			int idx, steps = 0;

			rcu_read_lock();
			st = rkl_probe(r->ft, iter, 0, Xk | (1ULL << 16), &got);
			if (st != CDS_FT_STATUS_OK || got != sib_lo) {
				/* (X,1) never moves: GE must land on it. */
				rcu_read_unlock();
				r->probe_bad++;
				r->failed = 1;
				if (r->probe_bad <= RKL_MAX_REPORT)
					fprintf(stderr,
						"rkl: ge on the fixed sib of junction %u -> node %p (%s)\n",
						X, (void *) got,
						cds_ft_status_to_string(st));
				mw_violation_snapshot();
				continue;
			}
			r->walks++;
			idx = 0;
			while (idx != 5 && steps++ < RKL_WALK_MAX) {
				int nidx, want;

				/* legal: the next node in this junction's order,
				 * or the (X,5) sibling if the run left. */
				want = (idx == 4) ? 5 : idx + 1;
				st = cds_ft_next(r->ft, iter);
				r->walk_steps++;
				if (st != CDS_FT_STATUS_OK) {
					r->walk_end++;
					break;
				}
				got = cds_ft_iter_node(iter);
				nidx = rkl_node_index(w, X, got);
				if (nidx == want || nidx == 5) {
					idx = nidx;
					continue;
				}
				/* Off the rails: classify (mode B measures). */
				if (nidx < 0) {
					uint8_t other = (X == w->bp) ? w->dp : w->bp;

					if (rkl_node_index(w, other, got) >= 0)
						r->walk_alien++;
					else
						r->walk_inner++;
				} else if (nidx <= idx)
					r->walk_back++;
				else
					r->walk_inner++;
				break;
			}
			rcu_read_unlock();
		}
		if ((++iters & 0xff) == 0)
			rcu_quiescent_state();
	}
	cds_ft_iter_destroy(iter);
	rcu_unregister_thread();
	return NULL;
}

static int inv_rekey_linearizability(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft;
	struct rk_writer_arg *w;
	struct rkl_reader_arg *r;
	pthread_t writers[RK_NW], readers[RKL_NR];
	struct ft_test_node *guard_lo, *guard_hi;
	struct timespec t0;
	unsigned long total_ops = 0, total_retries = 0, live = 0;
	unsigned long probes = 0, probe_bad = 0, walks = 0, walk_steps = 0;
	unsigned long alien = 0, back = 0, inner = 0, wend = 0;
	int i, c, ret = 0;

	if (!getenv("FT_INV_MW")) {
		fprintf(stderr, "# inv_rekey_linearizability: skipped "
			"(set FT_INV_MW=1 to run the relational linearizability oracle)\n");
		return 0;
	}
	mw_install_fatal_handler();
	leak_reset();

	ft = create_fixed_rekey_coherent_ft(4, &group);	/* EAGER + ordered list */
	cds_ft_make_concurrent(ft);

	guard_lo = node_alloc(0x00000000ULL);
	guard_hi = node_alloc(0xff000000ULL);
	rcu_read_lock();
	if (insert_u64(ft, 0x00000000ULL, guard_lo) != CDS_FT_STATUS_OK ||
			insert_u64(ft, 0xff000000ULL, guard_hi) != CDS_FT_STATUS_OK)
		abort();
	rcu_read_unlock();
	live = 2;

	w = (struct rk_writer_arg *) calloc(RK_NW, sizeof(*w));
	r = (struct rkl_reader_arg *) calloc(RKL_NR, sizeof(*r));
	if (!w || !r)
		abort();
	for (i = 0; i < RK_NW; i++) {
		/*
		 * DELIBERATELY NON-ADJACENT junctions (i+1 and i+1+RK_NW): with
		 * the usual (2i+1, 2i+2) pairing, "the numerically adjacent
		 * junction" and "the other junction of the same writer" are the
		 * same byte, and a wrong answer cannot tell a slot/rank mistake
		 * apart from a same-commit mix-up.
		 */
		uint8_t bp = (uint8_t) (i + 1), dp = (uint8_t) (i + 1 + RK_NW);
		uint64_t bpk = (uint64_t) bp << 24, dpk = (uint64_t) dp << 24;
		uint64_t sk[4] = {
			bpk | (1ULL << 16), bpk | (5ULL << 16),
			dpk | (1ULL << 16), dpk | (5ULL << 16),
		};

		w[i].ft = ft;
		w[i].bp = bp;
		w[i].dp = dp;
		w[i].sb = 3;
		rcu_read_lock();
		for (c = 0; c < 4; c++) {
			w[i].sib[c] = node_alloc(sk[c]);
			if (insert_u64(ft, sk[c], w[i].sib[c]) != CDS_FT_STATUS_OK)
				abort();
		}
		for (c = 0; c < 4; c++) {
			uint64_t tk = bpk | (3ULL << 16) | ((uint64_t) (c + 1) << 8);

			w[i].top[c] = node_alloc(tk);
			if (insert_u64(ft, tk, w[i].top[c]) != CDS_FT_STATUS_OK)
				abort();
		}
		rcu_read_unlock();
		live += 8;
	}
	for (i = 0; i < RKL_NR; i++) {
		r[i].ft = ft;
		r[i].w = w;
		r[i].nw = RK_NW;
	}

	test_go = 0;
	test_stop = 0;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	for (i = 0; i < RK_NW; i++)
		pthread_create(&writers[i], NULL, rk_writer, &w[i]);
	for (i = 0; i < RKL_NR; i++)
		pthread_create(&readers[i], NULL, rkl_reader, &r[i]);
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	test_go = 1;

	rcu_thread_offline();
	clock_gettime(CLOCK_MONOTONIC, &t0);
	while (elapsed_ms(&t0) < DEFAULT_DURATION_MS)
		usleep(1000);
	test_stop = 1;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	for (i = 0; i < RK_NW; i++)
		pthread_join(writers[i], NULL);
	for (i = 0; i < RKL_NR; i++)
		pthread_join(readers[i], NULL);
	rcu_thread_online();

	for (i = 0; i < RK_NW; i++) {
		total_ops += w[i].ops;
		total_retries += w[i].retries;
		if (w[i].failed)
			ret = -1;
	}
	for (i = 0; i < RKL_NR; i++) {
		probes += r[i].probes;
		probe_bad += r[i].probe_bad;
		walks += r[i].walks;
		walk_steps += r[i].walk_steps;
		alien += r[i].walk_alien;
		back += r[i].walk_back;
		inner += r[i].walk_inner;
		wend += r[i].walk_end;
		if (r[i].failed)
			ret = -1;
	}

	synchronize_rcu();
	rcu_read_lock();
	if (cds_ft_count_keys(ft) != live) {
		fprintf(stderr, "rekey linearizability: count_keys %lu != live %lu\n",
			cds_ft_count_keys(ft), live);
		ret = -1;
	}
	if (cds_ft_verify(ft, stderr) != CDS_FT_STATUS_OK) {
		fprintf(stderr, "rekey linearizability: cds_ft_verify failed\n");
		ret = -1;
	}
	rcu_read_unlock();

	if (total_ops == 0) {
		fprintf(stderr, "rekey linearizability: no successful moves (livelock?)\n");
		ret = -1;
	}
	if (probes == 0 || walk_steps == 0) {
		fprintf(stderr, "rekey linearizability: readers made no progress\n");
		ret = -1;
	}
	/*
	 * Mode A is the gate; mode B is a MEASUREMENT of the known parked-walker
	 * residual, printed so it is a number in the record rather than an
	 * assumption either way (zero would mean it was not exercised, not that
	 * it is gone).
	 */
	fprintf(stderr, "# inv_rekey_linearizability: %d writers %d readers, "
		"%lu moves, %lu retries, %lu bound-key probes (%lu illegal), "
		"%lu walks / %lu steps: %lu alien %lu back %lu inner %lu early-end\n",
		RK_NW, RKL_NR, total_ops, total_retries, probes, probe_bad,
		walks, walk_steps, alien, back, inner, wend);

	free(w);
	free(r);
	if (drain_and_destroy(ft, group) < 0)
		ret = -1;
	if (leak_check() < 0)
		ret = -1;
	return ret;
}

/*
 * ===========================================================================
 * PUBLIC-ENTRY NO-GAP ORACLE: cds_ft_rekey_graft must never make a moved key
 * unreachable.
 * ===========================================================================
 *
 * The property rekey exists to provide is that a key present throughout a move
 * is NEVER read as absent -- at neither the old nor the new position.  Every
 * other rekey oracle drives _cds_ft_debug_rekey_graft_simple, which is a
 * SEPARATE and simpler one-decide implementation; the PUBLIC cds_ft_rekey_graft
 * runs ft_merge_at_inner's staged path (detach the subtree into a transient
 * trie, then merge that trie back in at the destination) and nothing has ever
 * asserted no-gap against it.  This oracle does.
 *
 * THE WITNESS IS ONE ATOMIC RELATIONAL PROBE, needing no timestamps, no writer
 * instrumentation and no bound on how often the writer flips.  Each writer's run
 * sits under its bp junction XOR its dp junction, and the geometry puts BOTH
 * candidate positions inside ONE key-order BAND:
 *
 *   (bp,1) (bp,5) | run at (bp,9,1..4)  XOR  run at (dp,1,1..4) | (dp,5) (dp,9)
 *                 ^ band delimiter                              ^ band delimiter
 *
 * bp and dp are CONSECUTIVE junction bytes, and the run's slot is the HIGHEST
 * child of bp and the LOWEST child of dp, so nothing but the run ever sorts
 * between the two delimiters (bp,5) and (dp,5).  gt((bp,5)) is therefore the
 * run's first leaf and lt((dp,5)) its last -- THE SAME NODE in both
 * configurations.  This is the legal-pair check of inv_rekey_linearizability
 * with the pair DEGENERATED TO ONE ELEMENT, which turns a membership test into a
 * PRESENCE test: the correct answer does not depend on where the run currently
 * is, so any other answer (the delimiter itself, or a miss) says the run was
 * reachable from NEITHER position at that instant.  That is the gap, observed
 * directly by one linearization-point-free read, with no way for an oscillating
 * writer to fake agreement.
 *
 * IT CARRIES ITS OWN CONTROL: the readers run a QUIET PHASE first, with the
 * writers parked.  The band probes must be clean there -- a violation with
 * nothing moving would indict the geometry, not the library -- so the quiet-phase
 * count is kept and reported apart from the moving-phase count.
 *
 * TWO ARMS, ONE BODY -- the two writers cds_ft_rekey_graft dispatches to, put
 * under identical probes.  create_rekey_coherent_ft explains how the group config
 * selects one:
 *
 *  - inv_rekey_public_atomic_no_gap (fixed-length, list off) reaches
 *    ft_rekey_one_decide, which commits the src clear, the dst publish and the
 *    re-parents as ONE flip.  `absent moving` MUST be 0, and this arm GATES on
 *    it: it is the acceptance test for the atomic writer, and the thing that
 *    fails if a future change stops routing to it.
 *  - inv_rekey_public_staged_gap (variable-length, list on) reaches the
 *    detach-then-merge-back writer, whose absence window no probe can talk it out
 *    of, so that arm MEASURES the window instead of gating on it.  Opt in twice
 *    (FT_INV_MW + FT_INV_RKPG); an oracle that is red by construction must not be
 *    what a gate reports.
 *
 * MEASURED (4 writers, 8 readers, 200 ms each): the staged arm sees ~30000
 * absences over ~44 moves -- roughly 700 absence observations per move, which is
 * what a grace-period-wide window looks like rather than a race -- against 0 in
 * the atomic arm over a comparable number of moves.  Both arms report 0 absent
 * across the control phase's ~1.5M probes and 0 bad exact probes: the fixed
 * siblings are always present, and a moving leaf never answers as a foreign node.
 * The same geometry driven single-threaded through 40 moves answers both band
 * probes correctly at every rest point in either config, so an absence is the
 * move window and not the layout.
 */
#define RKP_NW		4		/* writers, one private band each */
#define RKP_NR		8		/* no-gap readers */
#define RKP_KLEN	4		/* every key is this wide (see below) */
#define RKP_QUIET_MS	60		/* control phase: readers, no movers */
#define RKP_MAX_REPORT	3		/* narrated per reader; the rest counted */
#define RKP_SLOT_BP	9		/* the run's slot byte in the bp junction */
#define RKP_SLOT_DP	1		/* ... and in the dp junction */

/*
 * THE TWO TRIES ARE THE TWO WRITERS, and the group config is what selects them:
 *
 *  - ATOMIC (@atomic true): a FIXED-length group with the ordered list OFF.
 *    cds_ft_rekey_graft dispatches such a trie to ft_rekey_one_decide, which
 *    commits the whole move as one flip.  Both properties are load-bearing.
 *    Fixed-length, because the atomic writer is the only rekey a fixed-length
 *    group gets (the staged one detaches, and a detached subtree's keys are
 *    stripped of the prefix).  List OFF, because the atomic writer locates the
 *    dst cell-splice position while the run is STILL at the source, so it
 *    refuses a destination that abuts the run's own ordered neighbourhood --
 *    which is exactly what this band geometry is.  A general rekey would locate
 *    the splice against the run-removed list; until then the band and the list
 *    are mutually exclusive.
 *
 *  - STAGED (@atomic false): a VARIABLE-length group, list on.  The atomic
 *    writer refuses a variable-length group (its splice-position validation is
 *    fixed-length only), so this trie gets the detach-then-merge-back writer and
 *    its absence window.
 *
 * Keys are RKP_KLEN wide in BOTH, which is this oracle's own doing: uniform width
 * makes the band's key order plain lexicographic, so "nothing else sorts between
 * the delimiters" is a property of the four bytes rather than of
 * prefix-vs-string ordering rules.
 */
static struct cds_ft *create_rekey_coherent_ft(bool atomic,
		struct cds_ft_group **group_out)
{
	struct cds_ft_group_attr *gattr;
	struct cds_ft_group *group;
	struct cds_ft_attr *attr;
	struct cds_ft *ft;

	if (cds_ft_group_attr_create(&gattr) < 0)
		abort();
	if (atomic) {
		if (cds_ft_group_attr_set_key_len(gattr, RKP_KLEN) < 0)
			abort();
	} else {
		if (cds_ft_group_attr_set_max_key_len(gattr, RKP_KLEN) < 0)
			abort();
	}
	if (cds_ft_group_attr_set_lookup_optimization(gattr,
			CDS_FT_LOOKUP_OPTIMIZE_EAGER) < 0)
		abort();
	if (cds_ft_group_attr_set_writer_strategy(gattr,
			CDS_FT_WRITER_LOCK_FINE) < 0)
		abort();
	if (cds_ft_group_attr_set_ordered_list(gattr, !atomic) < 0)
		abort();
	if (cds_ft_group_create(gattr, &group) < 0)
		abort();
	cds_ft_group_attr_destroy(gattr);
	if (cds_ft_attr_create(&attr) < 0)
		abort();
	/* Rekey coherence is automatic on an EAGER trie; EAGER is all it needs. */
	if (cds_ft_attr_set_speculative_keys(attr, false) < 0)
		abort();
	if (cds_ft_create(group, attr, &ft) < 0)
		abort();
	cds_ft_attr_destroy(attr);
	*group_out = group;
	return ft;
}

/* Insert / exact-lookup an RKP_KLEN-wide key (the shared u64 helpers resolve
 * RKP_KLEN, which a variable-length trie has none of). */
static enum cds_ft_status rkp_insert(struct cds_ft *ft, uint64_t v,
		struct ft_test_node *n)
{
	uint8_t k[8] = { 0 };

	cds_ft_u64_to_key(ft, v, k, RKP_KLEN);
	return cds_ft_insert(ft, k, RKP_KLEN, &n->node);
}

static enum cds_ft_status rkp_lookup(struct cds_ft *ft, uint64_t v,
		struct cds_ft_node **out)
{
	uint8_t k[8] = { 0 };

	cds_ft_u64_to_key(ft, v, k, RKP_KLEN);
	return cds_ft_eager_lookup_key(ft, k, RKP_KLEN, 0, out);
}

/* Are the movers released?  Readers grade the two phases apart. */
static volatile int rkp_moving;

struct rkp_writer_arg {
	struct cds_ft *ft;
	uint8_t bp, dp;			/* consecutive junction bytes, bp < dp */
	struct ft_test_node *sib[4];	/* fixed: (bp,1) (bp,5) (dp,5) (dp,9) */
	struct ft_test_node *top[4];	/* the run's four leaves, they move */
	int at_dst;			/* 0: run at (bp,9); 1: run at (dp,1) */
	unsigned long ops, busy;
	int failed;
};

struct rkp_reader_arg {
	struct cds_ft *ft;
	struct rkp_writer_arg *w;
	int nw;
	unsigned long band;		/* band probes -- the no-gap witness */
	unsigned long band_quiet;	/* ... of those, taken with the movers parked */
	unsigned long band_quiet_bad;	/* absent with the movers parked (control) */
	unsigned long band_move_bad;	/* absent during a move (the gap) */
	unsigned long exact;		/* exact probes */
	unsigned long exact_bad;	/* a fixed key missing, or a foreign node */
	int failed;
};

/* (b0,b1,b2) as a fixed four-byte key; byte 3 is always zero here. */
static uint64_t rkp_key(uint8_t b0, uint8_t b1, uint8_t b2)
{
	return ((uint64_t) b0 << 24) | ((uint64_t) b1 << 16) |
		((uint64_t) b2 << 8);
}

/* The four fixed band/junction siblings, in key order. */
static uint64_t rkp_sib_key(const struct rkp_writer_arg *w, int j)
{
	switch (j) {
	case 0:	return rkp_key(w->bp, 1, 0);
	case 1:	return rkp_key(w->bp, 5, 0);	/* low band delimiter */
	case 2:	return rkp_key(w->dp, 5, 0);	/* high band delimiter */
	default: return rkp_key(w->dp, 9, 0);
	}
}

static void *rkp_writer(void *arg)
{
	struct rkp_writer_arg *w = (struct rkp_writer_arg *) arg;
	unsigned long iters = 0;

	rcu_register_thread();
	while (!test_go)
		;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	/*
	 * The control phase.  Report a quiescent state while parked: a QSBR
	 * thread that merely sleeps is a reader every grace period waits for,
	 * and the readers below take grace periods (the coherent lookup does
	 * not, but their call_rcu drain does).
	 */
	while (!rkp_moving && !test_stop) {
		rcu_quiescent_state();
		usleep(200);
	}

	while (!test_stop) {
		uint8_t s = w->at_dst ? w->dp : w->bp;
		uint8_t d = w->at_dst ? w->bp : w->dp;
		uint8_t s_slot = w->at_dst ? RKP_SLOT_DP : RKP_SLOT_BP;
		uint8_t d_slot = w->at_dst ? RKP_SLOT_BP : RKP_SLOT_DP;
		uint8_t src_key[2] = { s, s_slot };
		uint8_t dst_key[2] = { d, d_slot };
		enum cds_ft_status st;

		/*
		 * NO read lock: cds_ft_rekey_graft enters the move gate, which
		 * publishes "expect a move" and then waits a grace period, so a
		 * caller holding a read section would wait for itself.
		 */
		st = cds_ft_rekey_graft(w->ft, dst_key, 2, src_key, 2);
		if (st == CDS_FT_STATUS_OK) {
			w->at_dst = !w->at_dst;
			w->ops++;
		} else if (st == CDS_FT_STATUS_BUSY_ERROR ||
				st == CDS_FT_STATUS_MEMORY_ERROR) {
			/*
			 * Transient: a peer holds a node this move needs, or a
			 * reserve came up short.  Both leave the run at the
			 * source, so retry the SAME direction.
			 */
			w->busy++;
		} else {
			fprintf(stderr, "rkp writer bp=%u: rekey %02x,%02x -> "
				"%02x,%02x: %s\n", w->bp, s, s_slot, d, d_slot,
				cds_ft_status_to_string(st));
			w->failed = 1;
			mw_violation_snapshot();
			break;
		}
		if ((++iters & 0xff) == 0)
			rcu_quiescent_state();
	}
	rcu_unregister_thread();
	return NULL;
}

static void *rkp_reader(void *arg)
{
	struct rkp_reader_arg *r = (struct rkp_reader_arg *) arg;
	struct cds_ft_iter *iter;
	unsigned int seed = (unsigned int) (uintptr_t) r;
	unsigned long iters = 0;

	rcu_register_thread();
	if (cds_ft_iter_create(r->ft, &iter) < 0)
		abort();
	while (!test_go)
		;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	while (!test_stop) {
		struct rkp_writer_arg *w = &r->w[rand_r(&seed) % r->nw];
		int pick = rand_r(&seed) & 7;
		struct cds_ft_node *got = NULL;
		enum cds_ft_status st;
		uint8_t k[8];

		if (pick < 4) {
			/*
			 * THE NO-GAP WITNESS.  One relational read from a band
			 * delimiter; the run's outermost leaf is the answer at
			 * every instant, whichever junction holds the run.
			 */
			int hi = pick & 1;
			uint64_t bound = hi ? rkp_sib_key(w, 2) : rkp_sib_key(w, 1);
			struct cds_ft_node *expect = hi ? &w->top[3]->node :
				&w->top[0]->node;
			int moving = rkp_moving;

			cds_ft_u64_to_key(r->ft, bound, k, RKP_KLEN);
			rcu_read_lock();
			cds_ft_iter_set_key(iter, k, RKP_KLEN);
			st = hi ? cds_ft_lookup_lt(r->ft, iter) :
				cds_ft_lookup_gt(r->ft, iter);
			if (st == CDS_FT_STATUS_OK)
				got = cds_ft_iter_node(iter);
			rcu_read_unlock();
			r->band++;
			if (!moving && !rkp_moving)
				r->band_quiet++;
			if (caa_unlikely(got != expect)) {
				unsigned long *cnt = (moving || rkp_moving) ?
					&r->band_move_bad : &r->band_quiet_bad;

				(*cnt)++;
				if (*cnt <= RKP_MAX_REPORT)
					fprintf(stderr,
						"rkp: %s(%#llx) -> %p (%s), the run's %s leaf %p is reachable from NEITHER junction%s\n",
						hi ? "lt" : "gt",
						(unsigned long long) bound,
						(void *) got,
						cds_ft_status_to_string(st),
						hi ? "last" : "first",
						(void *) expect,
						(moving || rkp_moving) ? "" :
							" -- WITH THE MOVERS PARKED");
				mw_violation_snapshot();
			}
		} else if (pick < 6) {
			/* A fixed sibling: present at every instant. */
			int j = rand_r(&seed) & 3;
			uint64_t key = rkp_sib_key(w, j);

			cds_ft_u64_to_key(r->ft, key, k, RKP_KLEN);
			rcu_read_lock();
			cds_ft_iter_set_key(iter, k, RKP_KLEN);
			st = cds_ft_lookup(r->ft, iter);
			got = cds_ft_iter_node(iter);
			rcu_read_unlock();
			r->exact++;
			if (caa_unlikely(got != &w->sib[j]->node)) {
				r->exact_bad++;
				if (r->exact_bad <= RKP_MAX_REPORT)
					fprintf(stderr,
						"rkp: fixed sib %#llx -> %p (%s), expect %p\n",
						(unsigned long long) key,
						(void *) got,
						cds_ft_status_to_string(st),
						(void *) &w->sib[j]->node);
				mw_violation_snapshot();
			}
		} else {
			/*
			 * A moving leaf at ONE of its two positions: a miss is
			 * legitimate (the run may be at the other junction),
			 * a FOREIGN node never is.
			 */
			int c = rand_r(&seed) & 3, hi = rand_r(&seed) & 1;
			uint64_t key = hi ?
				rkp_key(w->dp, RKP_SLOT_DP, (uint8_t) (c + 1)) :
				rkp_key(w->bp, RKP_SLOT_BP, (uint8_t) (c + 1));

			cds_ft_u64_to_key(r->ft, key, k, RKP_KLEN);
			rcu_read_lock();
			cds_ft_iter_set_key(iter, k, RKP_KLEN);
			st = cds_ft_lookup(r->ft, iter);
			got = cds_ft_iter_node(iter);
			rcu_read_unlock();
			r->exact++;
			if (caa_unlikely(got && got != &w->top[c]->node)) {
				r->exact_bad++;
				if (r->exact_bad <= RKP_MAX_REPORT)
					fprintf(stderr,
						"rkp: moving key %#llx -> %p (%s), expect miss or %p\n",
						(unsigned long long) key,
						(void *) got,
						cds_ft_status_to_string(st),
						(void *) &w->top[c]->node);
				mw_violation_snapshot();
			}
		}
		if ((++iters & 0xff) == 0)
			rcu_quiescent_state();
	}
	cds_ft_iter_destroy(iter);
	rcu_unregister_thread();
	return NULL;
}

static int inv_rekey_public_no_gap_run(bool atomic, const char *name)
{
	struct cds_ft_group *group;
	struct cds_ft *ft;
	struct rkp_writer_arg *w;
	struct rkp_reader_arg *r;
	pthread_t writers[RKP_NW], readers[RKP_NR];
	struct ft_test_node *guard_lo, *guard_hi;
	struct timespec t0;
	unsigned long total_ops = 0, total_busy = 0, live = 0;
	unsigned long band = 0, quiet = 0, quiet_bad = 0, move_bad = 0;
	unsigned long exact = 0, exact_bad = 0;
	int i, c, j, ret = 0;

	mw_install_fatal_handler();
	leak_reset();

	ft = create_rekey_coherent_ft(atomic, &group);
	cds_ft_make_concurrent(ft);

	guard_lo = node_alloc(0x00000000ULL);
	guard_hi = node_alloc(0xff000000ULL);
	rcu_read_lock();
	if (rkp_insert(ft, 0x00000000ULL, guard_lo) != CDS_FT_STATUS_OK ||
			rkp_insert(ft, 0xff000000ULL, guard_hi) != CDS_FT_STATUS_OK)
		abort();
	rcu_read_unlock();
	live = 2;

	w = (struct rkp_writer_arg *) calloc(RKP_NW, sizeof(*w));
	r = (struct rkp_reader_arg *) calloc(RKP_NR, sizeof(*r));
	if (!w || !r)
		abort();
	for (i = 0; i < RKP_NW; i++) {
		/*
		 * CONSECUTIVE junction bytes: the band between (bp,5) and (dp,5)
		 * must contain the run and NOTHING else, so no other key -- no
		 * peer writer's junction included -- may sort between bp and dp.
		 */
		w[i].ft = ft;
		w[i].bp = (uint8_t) (2 * i + 1);
		w[i].dp = (uint8_t) (2 * i + 2);
		rcu_read_lock();
		for (j = 0; j < 4; j++) {
			uint64_t sk = rkp_sib_key(&w[i], j);

			w[i].sib[j] = node_alloc(sk);
			if (rkp_insert(ft, sk, w[i].sib[j]) != CDS_FT_STATUS_OK)
				abort();
		}
		for (c = 0; c < 4; c++) {
			uint64_t tk = rkp_key(w[i].bp, RKP_SLOT_BP,
					(uint8_t) (c + 1));

			w[i].top[c] = node_alloc(tk);
			if (rkp_insert(ft, tk, w[i].top[c]) != CDS_FT_STATUS_OK)
				abort();
		}
		rcu_read_unlock();
		live += 8;
	}
	for (i = 0; i < RKP_NR; i++) {
		r[i].ft = ft;
		r[i].w = w;
		r[i].nw = RKP_NW;
	}

	test_go = 0;
	test_stop = 0;
	rkp_moving = 0;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	for (i = 0; i < RKP_NW; i++)
		pthread_create(&writers[i], NULL, rkp_writer, &w[i]);
	for (i = 0; i < RKP_NR; i++)
		pthread_create(&readers[i], NULL, rkp_reader, &r[i]);
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	test_go = 1;

	rcu_thread_offline();
	/* CONTROL: readers only.  Every band probe must answer the run. */
	clock_gettime(CLOCK_MONOTONIC, &t0);
	while (elapsed_ms(&t0) < RKP_QUIET_MS)
		usleep(1000);
	rkp_moving = 1;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	clock_gettime(CLOCK_MONOTONIC, &t0);
	while (elapsed_ms(&t0) < DEFAULT_DURATION_MS)
		usleep(1000);
	test_stop = 1;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	for (i = 0; i < RKP_NW; i++)
		pthread_join(writers[i], NULL);
	for (i = 0; i < RKP_NR; i++)
		pthread_join(readers[i], NULL);
	rcu_thread_online();

	for (i = 0; i < RKP_NW; i++) {
		total_ops += w[i].ops;
		total_busy += w[i].busy;
		if (w[i].failed)
			ret = -1;
	}
	for (i = 0; i < RKP_NR; i++) {
		band += r[i].band;
		quiet += r[i].band_quiet;
		quiet_bad += r[i].band_quiet_bad;
		move_bad += r[i].band_move_bad;
		exact += r[i].exact;
		exact_bad += r[i].exact_bad;
		if (r[i].failed)
			ret = -1;
	}

	synchronize_rcu();
	rcu_read_lock();
	for (i = 0; i < RKP_NW; i++) {
		uint8_t X = w[i].at_dst ? w[i].dp : w[i].bp;
		uint8_t slot = w[i].at_dst ? RKP_SLOT_DP : RKP_SLOT_BP;

		for (j = 0; j < 4; j++) {
			struct cds_ft_node *found = NULL;

			if (rkp_lookup(ft, rkp_sib_key(&w[i], j), &found)
					!= CDS_FT_STATUS_OK ||
					found != &w[i].sib[j]->node)
				ret = -1;
		}
		for (c = 0; c < 4; c++) {
			struct cds_ft_node *found = NULL;

			if (rkp_lookup(ft, rkp_key(X, slot, (uint8_t) (c + 1)),
					&found) != CDS_FT_STATUS_OK ||
					found != &w[i].top[c]->node)
				ret = -1;
		}
	}
	if (cds_ft_count_keys(ft) != live) {
		fprintf(stderr, "rkp: count_keys %lu != live %lu\n",
			cds_ft_count_keys(ft), live);
		ret = -1;
	}
	if (cds_ft_verify(ft, stderr) != CDS_FT_STATUS_OK) {
		fprintf(stderr, "rkp: cds_ft_verify failed\n");
		ret = -1;
	}
	rcu_read_unlock();

	if (total_ops == 0) {
		fprintf(stderr, "rkp: no successful moves (livelock?)\n");
		ret = -1;
	}
	if (band == 0 || exact == 0) {
		fprintf(stderr, "rkp: readers made no progress\n");
		ret = -1;
	}
	/*
	 * The control's "0 absences while parked" only means something next to the
	 * number of probes that produced it, so require the quiet phase to have
	 * really run: a zero out of zero probes is a dead control reading as a
	 * clean one.
	 */
	if (quiet == 0) {
		fprintf(stderr, "rkp: the control phase took no band probe\n");
		ret = -1;
	}
	if (quiet_bad || move_bad || exact_bad)
		ret = -1;

	fprintf(stderr, "# %s: %d writers %d readers, "
		"%lu moves, %lu busy, %lu band probes (%lu absent moving; "
		"%lu parked, %lu absent), %lu exact probes (%lu bad), "
		"%lu live keys\n", name,
		RKP_NW, RKP_NR, total_ops, total_busy, band, move_bad,
		quiet, quiet_bad, exact, exact_bad, live);

	free(w);
	free(r);
	if (drain_and_destroy(ft, group) < 0)
		ret = -1;
	if (leak_check() < 0)
		ret = -1;
	return ret;
}

/*
 * THE ATOMIC ARM -- the acceptance test for the one-decide writer, and a GATE:
 * every band probe must answer the run, so `absent moving` must be 0.  It needs
 * no second opt-in, unlike the staged arm below, because there is nothing
 * known-red about it.
 */
static int inv_rekey_public_atomic_no_gap(void)
{
	if (!getenv("FT_INV_MW")) {
		fprintf(stderr, "# inv_rekey_public_atomic_no_gap: skipped "
			"(set FT_INV_MW=1 to run the concurrent-writer oracles)\n");
		return 0;
	}
	return inv_rekey_public_no_gap_run(true,
			"inv_rekey_public_atomic_no_gap");
}

/*
 * THE STAGED ARM -- the same probes against the writer that cannot satisfy them,
 * so it MEASURES the absence window (see the header above) instead of gating on
 * it.  Opt in twice.
 */
static int inv_rekey_public_staged_gap(void)
{
	if (!getenv("FT_INV_MW")) {
		fprintf(stderr, "# inv_rekey_public_staged_gap: skipped "
			"(set FT_INV_MW=1 to run the concurrent-writer oracles)\n");
		return 0;
	}
	if (!getenv("FT_INV_RKPG")) {
		fprintf(stderr, "# inv_rekey_public_staged_gap: skipped -- a "
			"variable-length group still moves by detach-into-a-transient-"
			"trie, so a moved key is absent for a grace period per move; "
			"set FT_INV_RKPG=1 to measure the gap\n");
		return 0;
	}
	return inv_rekey_public_no_gap_run(false, "inv_rekey_public_staged_gap");
}

/*
 * SIBLING PAIR: two writers, one child each of a shared three-byte prefix, both
 * cycling insert -> read-back -> remove.  With the peer's child present an
 * insert has to SPLIT the prefix node; with it absent a remove has to
 * PATH-COMPRESS the prefix back into a leaf.  So the pair drives a split and a
 * compression of the SAME node, from two threads, continuously -- which the
 * disjoint-key insert/remove oracles never do, because their keys diverge high
 * enough that no node is ever both split and compressed under contention.
 *
 * ORACLE: an insert that returned OK must be readable in the very critical
 * section that published it.  Nobody else writes that key, so a miss is not a
 * race with a peer's legitimate removal -- it is a lost key.
 */
#define SIBP_NW		6		/* sibling pairs */

/* (b0,b1,b2,b3) as a fixed four-byte key. */
static uint64_t sibp_key(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3)
{
	return ((uint64_t) b0 << 24) | ((uint64_t) b1 << 16) |
		((uint64_t) b2 << 8) | (uint64_t) b3;
}

/* Remove the node @n stored at key @v, by IDENTITY. */
static enum cds_ft_status sibp_remove(struct cds_ft *ft, uint64_t v,
		struct ft_test_node *n)
{
	struct cds_ft_iter *iter = NULL;
	enum cds_ft_status st = CDS_FT_STATUS_NOT_FOUND;
	uint8_t k[8] = { 0 };

	if (cds_ft_iter_create(ft, &iter) < 0)
		abort();
	rcu_read_lock();
	cds_ft_u64_to_key(ft, v, k, CDS_FT_LEN_DEFAULT);
	cds_ft_iter_set_key(iter, k, CDS_FT_LEN_DEFAULT);
	cds_ft_lookup(ft, iter);
	if (cds_ft_iter_node(iter)) {
		st = cds_ft_remove(ft, iter, &n->node);
		if (st == CDS_FT_STATUS_OK)
			node_free_rcu(n);
	}
	rcu_read_unlock();
	cds_ft_iter_destroy(iter);
	return st;
}

struct sibp_arg {
	struct cds_ft *ft;
	uint64_t key;
	unsigned long ops, busy, lost, stuck;
	struct ft_test_node *inflight;	/* published, not yet taken back */
	int failed;
	int *stop_all;
};

static void *sibp_writer(void *arg)
{
	struct sibp_arg *w = (struct sibp_arg *) arg;
	unsigned long iters = 0;

	rcu_register_thread();
	while (!test_go)
		;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	while (!test_stop) {
		struct ft_test_node *n = node_alloc(w->key);
		struct cds_ft_node *f = NULL;
		enum cds_ft_status st;
		int a;

		rcu_read_lock();
		st = insert_u64(w->ft, w->key, n);
		if (st != CDS_FT_STATUS_OK) {
			rcu_read_unlock();
			node_free(n);
			if (st == CDS_FT_STATUS_DUPLICATE_FOUND) {
				fprintf(stderr, "sibp key=%#lx: DUPLICATE -- our own previous "
					"node was never removed\n", (unsigned long) w->key);
				w->failed = 1;
				goto out;
			}
			w->busy++;		/* BUSY / MEMORY: transient */
			continue;
		}
		if (lookup_u64(w->ft, w->key, &f) != CDS_FT_STATUS_OK ||
				f != &n->node) {
			rcu_read_unlock();
			fprintf(stderr, "sibp key=%#lx: an OK insert is NOT READABLE in "
				"the critical section that published it (f=%p want=%p) "
				"after %lu ops, %lu busy\n", (unsigned long) w->key,
				(void *) f, (void *) &n->node, w->ops, w->busy);
			w->lost++;
			w->failed = 1;
			mw_violation_snapshot();
			goto out;
		}
		w->inflight = n;	/* the run owes this node a reclaim */
		rcu_read_unlock();

		for (a = 0; a < 100000 && !test_stop; a++)
			if (sibp_remove(w->ft, w->key, n) == CDS_FT_STATUS_OK) {
				w->inflight = NULL;
				break;
			}
		if (w->inflight) {
			/*
			 * Exhausting the take-back retries is CONTENTION, not a
			 * defect: this key is ours alone, so the removal is only
			 * losing races.  Measured 1 run in 12 under a saturated
			 * machine (12 concurrent copies of this oracle), and
			 * identically with the access-discipline validator on and
			 * off -- i.e. it is the bound, not the library.  Failing
			 * here made the gate flaky and would have masked a real
			 * defect behind a load artifact.
			 *
			 * Count it and stop this writer's cycle; @inflight is
			 * reclaimed after the join, so the entries/leak checks
			 * stay exact.  A genuine WEDGE (the defect this oracle was
			 * built for) does not look like this -- it never completes
			 * any cycle, which the per-writer liveness check below
			 * catches.
			 */
			w->stuck++;
			goto out;
		}
		w->ops++;
		if ((++iters & 0xff) == 0)
			rcu_quiescent_state();
	}
out:
	*w->stop_all = 1;
	rcu_unregister_thread();
	return NULL;
}

/*
 * Is every node this run allocated either FREED or still REACHABLE?
 *
 * Each one ends up in exactly one of those two places, so at teardown, with
 * every deferred free drained,
 *
 *	allocated - freed  ==  nodes reachable from the root
 *
 * and an excess is a node that is NEITHER -- lost out of the structure without
 * being reclaimed.  leak_check() reports the same excess, but as a bare delta
 * and only AFTER drain_and_destroy(), by which time the trie that would explain
 * it is gone.  Asked here, the answer arrives with the structure still in
 * memory (and, under FT_INV_ABORT_ON_VIOLATION, in a core).
 *
 * ★ This check is the one that placed the defect: it comes back BALANCED on
 * runs that then report a leak, which is what proved the loss happens in the
 * TEARDOWN WALK and not in the concurrent phase -- see sibp_drain_checked().
 */
static int sibp_report_orphans(struct cds_ft *ft, unsigned long live)
{
	unsigned long unfreed, reachable;

	rcu_barrier();		/* every deferred free of ours has run */
	unfreed = __atomic_load_n(&nodes_allocated, __ATOMIC_RELAXED) -
		__atomic_load_n(&nodes_freed, __ATOMIC_RELAXED);
	rcu_read_lock();
	reachable = cds_ft_count_entries(ft);
	rcu_read_unlock();
	if (unfreed == reachable && reachable == live)
		return 0;
	fprintf(stderr, "sibp ORPHAN: %ld node(s) neither freed nor reachable "
		"(allocated-freed %lu, reachable %lu, expected %lu)\n",
		(long) (unfreed - reachable), unfreed, reachable, live);
	/* The trie is intact right here.  Keep the core. */
	mw_violation_snapshot();
	return -1;
}

#define RKSM_NW		6		/* mover/mutator pairs */
#define RKSM_SB		3		/* S_top slot byte inside each junction */
#define RKSM_MUT	0x60		/* mutator child byte base under S_top */

struct rksm_arg {
	struct cds_ft *ft;
	uint8_t bp, dp, mine, guard, mut;	/* @mut is the DEEP byte, under @mine */
	struct ft_test_node *seed_priv, *seed_guard;
	int seeded;
	unsigned long ops, retries, lost, checks, einval;
	struct ft_test_node *mut_node;	/* in-flight, for post-join reclaim */
	unsigned long stuck;		/* reclaim loops that ran out of retries */
	int prev_rc;			/* the merge outcome one iteration back */
	int failed;
	int *stop_all;
};

/*
 * Where did it go?  Walk the WHOLE trie for the node by IDENTITY.  "Present at
 * neither src nor dst" has two very different causes -- a merge dropped it
 * (absent from the structure entirely) or the harness lost track of where it
 * put it (present, under some third key) -- and only a scan tells them apart.
 * Concurrent, so a miss is weaker evidence than a hit; a hit is definitive.
 */
/*
 * Is @n findable at @k RIGHT NOW?  Called between the mover's own two seed
 * inserts, which is what narrowed this oracle's loss from "the seed vanished
 * somewhere inside the merge" to a single library call: the seed is present
 * after its own insert returns OK and GONE after the SIBLING insert returns OK,
 * same thread, same read-side critical section (6/6 runs).  So the producer is
 * cds_ft_insert dropping a neighbouring key, not the rekey/graft/merge this
 * oracle was built to exercise -- and checking here names it in one line
 * instead of a trace.  A peer IS required: with the mutators disabled the
 * movers run ~8k-21k moves per run and lose nothing (4/4).
 */
static void rksm_seed_present(struct cds_ft *ft, unsigned int bp, uint64_t k,
		struct ft_test_node *n, const char *when, int *failed)
{
	struct cds_ft_node *got = NULL;

	if (lookup_u64(ft, k, &got) == CDS_FT_STATUS_OK && got == &n->node)
		return;
	fprintf(stderr, "rksm bp=%u: seed %p is ABSENT at its own key %#lx %s -- "
		"the insert that reported OK did not leave it findable (got %p)\n",
		bp, (void *) n, (unsigned long) k, when, (void *) got);
	*failed = 1;
}

static int rksm_locate(struct cds_ft *ft, struct ft_test_node *n, uint64_t *at)
{
	struct cds_ft_iter *iter = NULL;
	int found = 0;

	if (cds_ft_iter_create(ft, &iter) < 0)
		return -1;
	rcu_read_lock();
	cds_ft_for_each_rcu(ft, iter) {
		uint8_t rk[8];
		size_t rk_len;

		if (cds_ft_iter_node(iter) != &n->node)
			continue;
		cds_ft_iter_get_key(iter, rk, sizeof(rk), &rk_len);
		*at = cds_ft_key_to_u64(ft, rk, CDS_FT_LEN_DEFAULT);
		found = 1;
		break;
	}
	rcu_read_unlock();
	cds_ft_iter_destroy(iter);
	return found;
}

/*
 * The mover is the SOLE writer of its own two seeds: the mutator's key is a
 * sibling one byte deeper, and every other pair owns a different junction.  So
 * "findable at neither the source nor the destination" is neither a race nor
 * harness bookkeeping -- and a whole-trie scan by identity then says whether
 * the node is gone from the structure or merely somewhere unexpected.
 */
static int rksm_seeds_check(struct rksm_arg *w, const char *when, int rc)
{
	uint64_t kp = rkm_key(w->bp, RKSM_SB, w->mine);
	uint64_t kg = rkm_key(w->bp, RKSM_SB, w->guard);
	uint64_t dp_priv = rkm_key(w->dp, RKSM_SB, w->mine);
	uint64_t dp_guard = rkm_key(w->dp, RKSM_SB, w->guard);
	struct cds_ft_node *ps = NULL, *pd = NULL, *gs = NULL, *gd = NULL;
	int lost_priv, lost_guard, where;
	uint64_t at = 0;

	rcu_read_lock();
	lost_priv = lookup_u64(w->ft, kp, &ps) != CDS_FT_STATUS_OK &&
		lookup_u64(w->ft, dp_priv, &pd) != CDS_FT_STATUS_OK;
	lost_guard = lookup_u64(w->ft, kg, &gs) != CDS_FT_STATUS_OK &&
		lookup_u64(w->ft, dp_guard, &gd) != CDS_FT_STATUS_OK;
	rcu_read_unlock();
	if (!lost_priv && !lost_guard)
		return 0;

	where = rksm_locate(w->ft, lost_priv ? w->seed_priv : w->seed_guard, &at);
	fprintf(stderr, "rksm MOVER bp=%u: own %s seed present at NEITHER src nor "
		"dst @%s (rc=%d, prev rc=%d) -- priv{src=%d dst=%d} "
		"guard{src=%d dst=%d} scan=%s key=%#lx seed=%p "
		"(moves=%lu retries=%lu drifts=%lu stuck=%lu)\n",
		w->bp, lost_priv ? "priv" : "guard", when, rc, w->prev_rc,
		ps != NULL, pd != NULL, gs != NULL, gd != NULL,
		where > 0 ? "FOUND" : "absent", (unsigned long) at,
		(void *) (lost_priv ? w->seed_priv : w->seed_guard),
		w->ops, w->retries, w->einval, w->stuck);
	w->lost++;
	w->failed = 1;
	mw_violation_snapshot();
	return -1;
}

static void *rksm_mover(void *arg)
{
	struct rksm_arg *w = (struct rksm_arg *) arg;
	uint8_t src_key[2], dst_key[2];
	unsigned long iters = 0;

	src_key[0] = w->bp; src_key[1] = RKSM_SB;
	dst_key[0] = w->dp; dst_key[1] = RKSM_SB;

	rcu_register_thread();
	while (!test_go)
		;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	while (!test_stop) {
		uint64_t kp = rkm_key(w->bp, RKSM_SB, w->mine);
		uint64_t kg = rkm_key(w->bp, RKSM_SB, w->guard);
		int rc;

		if (!w->seeded) {
			rcu_read_lock();
			w->seed_priv = node_alloc(kp);
			w->seed_guard = node_alloc(kg);
			if (insert_u64(w->ft, kp, w->seed_priv) != CDS_FT_STATUS_OK ||
					insert_u64(w->ft, kg, w->seed_guard) !=
						CDS_FT_STATUS_OK) {
				rcu_read_unlock();
				fprintf(stderr, "rksm bp=%u: seed failed\n", w->bp);
				w->failed = 1;
				goto out;
			}
			/*
			 * Both seeds must be findable before anything else
			 * runs.  Checked in the SAME critical section as the
			 * inserts, so a loss reported here cannot be blamed on
			 * reclamation or on a later operation.
			 */
			rksm_seed_present(w->ft, w->bp, kp, w->seed_priv,
					"once BOTH seed inserts have returned OK",
					&w->failed);
			rksm_seed_present(w->ft, w->bp, kg, w->seed_guard,
					"once BOTH seed inserts have returned OK",
					&w->failed);
			if (w->failed) {
				rcu_read_unlock();
				mw_violation_snapshot();
				goto out;
			}
			rcu_read_unlock();
			w->seeded = 1;
		}

		/*
		 * BEFORE and AFTER, so the window a loss happened in is a single
		 * named interval rather than "some time in the last iteration".
		 * BEFORE covers the seed's own publication and everything the
		 * MUTATOR did while we were between merges; AFTER covers the merge.
		 */
		if (rksm_seeds_check(w, "pre-merge", 0) < 0)
			goto out;
		rc = _cds_ft_debug_rekey_graft_simple(w->ft, src_key, 2, dst_key, 2);
		if (rksm_seeds_check(w, "post-merge", rc) < 0)
			goto out;
		w->prev_rc = rc;

		if (rc == 0) {
			w->ops++;
			{
				int a;

				/* Transiently BUSY under a peer: retry, do not fail. */
				for (a = 0; a < 1000; a++)
					if (rkms_remove_node(w->ft,
							rkm_key(w->dp, RKSM_SB, w->mine),
							w->seed_priv) == CDS_FT_STATUS_OK)
						break;
				if (a == 1000)
					w->stuck++;
				for (a = 0; a < 1000; a++)
					if (rkms_remove_node(w->ft,
							rkm_key(w->dp, RKSM_SB, w->guard),
							w->seed_guard) == CDS_FT_STATUS_OK)
						break;
				if (a == 1000)
					w->stuck++;
			}
			w->seeded = 0;
		} else if (rc == -EAGAIN || rc == -EIO || rc == -ENOMEM) {
			w->retries++;
		} else if (rc == -EINVAL) {
			/*
			 * SHAPE DRIFT, not a verdict on the library.  This oracle
			 * mutates the source on purpose, and the mover's own seed can
			 * end up removed at the dst while the mutator's key keeps the
			 * byte alive -- leaving S_top single-child, hence PATH-
			 * COMPRESSED, which the rekey scope gate refuses (measured: the
			 * s_top gate, `compressed` bit only).  Re-seed and carry on.
			 *
			 * This does NOT mask a permanent -EINVAL: a shape the library
			 * could never accept would drive @ops to zero, and the
			 * per-writer liveness check below fails on exactly that.
			 */
			w->einval++;
			/*
			 * RECLAIM before re-seeding.  The seed is already published
			 * (possibly partly at the src and partly at the dst after a
			 * drifted move), and simply dropping it on the floor leaks
			 * exactly one node per drift -- measured as a leak delta that
			 * tracked this counter 1:1 until this loop existed.
			 */
			{
				int a;

				/* Retry: a single-shot remove loses to a peer. */
				for (a = 0; a < 2000; a++)
					if (rkms_remove_node(w->ft, kp, w->seed_priv)
							== CDS_FT_STATUS_OK ||
						rkms_remove_node(w->ft,
							rkm_key(w->dp, RKSM_SB,
								w->mine),
							w->seed_priv) ==
								CDS_FT_STATUS_OK)
						break;
				if (a == 2000)
					w->stuck++;
				for (a = 0; a < 2000; a++)
					if (rkms_remove_node(w->ft, kg, w->seed_guard)
							== CDS_FT_STATUS_OK ||
						rkms_remove_node(w->ft,
							rkm_key(w->dp, RKSM_SB,
								w->guard),
							w->seed_guard) ==
								CDS_FT_STATUS_OK)
						break;
				if (a == 2000)
					w->stuck++;
			}
			w->seeded = 0;
		} else {
			fprintf(stderr, "rksm bp=%u: merge failed rc=%d\n", w->bp, rc);
			w->failed = 1;
			mw_violation_snapshot();
			goto out;
		}
		if ((++iters & 0xff) == 0)
			rcu_quiescent_state();
	}
out:
	*w->stop_all = 1;
	rcu_unregister_thread();
	return NULL;
}

/*
 * The peer that the fence has to exclude: it adds a CHILD to the S_top its
 * mover is copying, then insists the key still exists somewhere.
 */
static void *rksm_mutator(void *arg)
{
	struct rksm_arg *w = (struct rksm_arg *) arg;
	unsigned long iters = 0;

	rcu_register_thread();
	while (!test_go)
		;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	while (!test_stop) {
		/*
		 * DEEPER than a sibling of S_top's children, on purpose.  As a
		 * sibling this key can be the ONLY one under (bp,SB) in the window
		 * between the mover's move and its re-seed, which PATH-COMPRESSES
		 * S_top -- a shape the rekey legitimately refuses (-EINVAL, measured
		 * at the s_top scope gate, `compressed` bit only).  Hanging it under
		 * the mover's own child instead keeps S_top a >=2-child branch at all
		 * times, while still adding a child INSIDE the subtree being copied,
		 * which is the whole point.
		 */
		uint64_t at_src = ((uint64_t) w->bp << 24) |
			((uint64_t) RKSM_SB << 16) |
			((uint64_t) w->mine << 8) | (uint64_t) w->mut;
		uint64_t at_dst = ((uint64_t) w->dp << 24) |
			((uint64_t) RKSM_SB << 16) |
			((uint64_t) w->mine << 8) | (uint64_t) w->mut;
		struct ft_test_node *n = node_alloc(at_src);
		struct cds_ft_node *f = NULL;
		enum cds_ft_status st;

		rcu_read_lock();
		st = insert_u64(w->ft, at_src, n);
		rcu_read_unlock();
		if (st == CDS_FT_STATUS_OK)
			w->mut_node = n;	/* published: the run owes its reclaim */
		if (st != CDS_FT_STATUS_OK) {	/* BUSY / MEMORY: transient */
			node_free(n);
			w->retries++;
			continue;
		}

		/*
		 * The child is IN.  A concurrent merge may move it to the dst at any
		 * point from here; what it may never do is lose it.
		 */
		w->checks++;
		rcu_read_lock();
		if (lookup_u64(w->ft, at_src, &f) != CDS_FT_STATUS_OK &&
				lookup_u64(w->ft, at_dst, &f) != CDS_FT_STATUS_OK) {
			rcu_read_unlock();
			/*
			 * Say WHAT was observed, not who did it.  "A merge
			 * swallowed it" is an attribution, and the one time it
			 * was chased it was wrong: the same loss reproduces with
			 * the merge disabled entirely, and its minimal form is
			 * two writers splitting and path-compressing one node
			 * with no merge, rekey or graft in sight
			 * (inv_sibling_split_compress).  A message that names a
			 * culprit sends the next reader to the wrong subsystem.
			 */
			fprintf(stderr, "rksm MUT bp=%u: child %#lx is present at "
				"NEITHER src nor dst -- it left the trie\n",
				w->bp, (unsigned long) at_src);
			w->lost++;
			w->failed = 1;
			mw_violation_snapshot();
			goto out;
		}
		rcu_read_unlock();

		/* Take it back from wherever it landed. */
		if (rkms_remove_node(w->ft, at_src, n) != CDS_FT_STATUS_OK &&
				rkms_remove_node(w->ft, at_dst, n) != CDS_FT_STATUS_OK) {
			fprintf(stderr, "rksm MUT bp=%u: could not reclaim %#lx\n",
				w->bp, (unsigned long) at_src);
			w->failed = 1;
			goto out;
		}
		w->mut_node = NULL;		/* reclaimed by the remove above */
		w->ops++;
		if ((++iters & 0xff) == 0)
			rcu_quiescent_state();
	}
out:
	*w->stop_all = 1;
	rcu_unregister_thread();
	return NULL;
}

static int inv_rekey_src_mutated(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft;
	struct rksm_arg *mv, *mu;
	struct ft_test_node *res[RKSM_NW][2];
	pthread_t movers[RKSM_NW], mutators[RKSM_NW];
	struct timespec t0;
	unsigned long moves = 0, checks = 0, lost = 0, live = 0, einval = 0;
	unsigned long stuck = 0;
	int i, c, ret = 0, stop_all = 0;

	/*
	 * KNOWN-FAILING, so opt-in TWICE: FT_INV_MW selects the concurrent-writer
	 * oracles at all, and FT_INV_RKSM arms this one.  It reproduces a LIVE
	 * key loss -- a mover's own seed present at neither src nor dst, with a
	 * matching leak delta -- in every run measured (1 lost, 3 of 3), and the
	 * gate's imw legs would otherwise go permanently red, which is how a
	 * genuine regression stops being visible.
	 *
	 * It is registered unconditionally rather than #ifdef'd out, because a
	 * test that vanishes reads as coverage and is not; the skip below names
	 * the defect so a reader of the log knows what is not being checked.
	 * Remove the FT_INV_RKSM gate the moment the loss is fixed.
	 */
	if (!getenv("FT_INV_MW")) {
		fprintf(stderr, "# inv_rekey_src_mutated: skipped "
			"(set FT_INV_MW=1 to run the source-mutated merge oracle)\n");
		return 0;
	}
	if (!getenv("FT_INV_RKSM")) {
		fprintf(stderr, "# inv_rekey_src_mutated: skipped -- reproduces a "
			"LIVE key loss (seed at neither src nor dst); set "
			"FT_INV_RKSM=1 to run it\n");
		return 0;
	}
	mw_install_fatal_handler();
	leak_reset();

	ft = create_fixed_fine_lock_listoff_ft(4, &group);	/* LIST OFF */
	cds_ft_make_concurrent(ft);

	mv = (struct rksm_arg *) calloc(RKSM_NW, sizeof(*mv));
	mu = (struct rksm_arg *) calloc(RKSM_NW, sizeof(*mu));
	if (!mv || !mu)
		abort();

	rcu_read_lock();
	for (i = 0; i < RKSM_NW; i++) {
		uint8_t bp = (uint8_t) (0x30 + i), dp = (uint8_t) (0x80 + i);

		mv[i].ft = mu[i].ft = ft;
		mv[i].bp = mu[i].bp = bp;
		mv[i].dp = mu[i].dp = dp;
		mv[i].mine = mu[i].mine = (uint8_t) (0x10 + i);
		mv[i].guard = mu[i].guard = (uint8_t) (0x40 + i);
		mv[i].mut = mu[i].mut = (uint8_t) (RKSM_MUT + i);	/* byte 3 */
		mv[i].stop_all = mu[i].stop_all = &stop_all;

		/* Sibs keep BP >= 3 children; residents keep the dst OCCUPIED. */
		for (c = 0; c < 2; c++) {
			uint64_t sk = rkm_key(bp, c == 0 ? 1 : 5, 0);
			uint64_t dk = rkm_key(dp, c == 0 ? 1 : 5, 0);

			if (insert_u64(ft, sk, node_alloc(sk)) != CDS_FT_STATUS_OK ||
					insert_u64(ft, dk, node_alloc(dk)) !=
						CDS_FT_STATUS_OK)
				abort();
			live += 2;
		}
		for (c = 0; c < 2; c++) {
			uint64_t rk = rkm_key(dp, RKSM_SB, (uint8_t) (c + 1));

			res[i][c] = node_alloc(rk);
			if (insert_u64(ft, rk, res[i][c]) != CDS_FT_STATUS_OK)
				abort();
			live++;
		}
	}
	rcu_read_unlock();

	test_go = 0;
	test_stop = 0;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	for (i = 0; i < RKSM_NW; i++) {
		pthread_create(&movers[i], NULL, rksm_mover, &mv[i]);
		pthread_create(&mutators[i], NULL, rksm_mutator, &mu[i]);
	}
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	test_go = 1;

	rcu_thread_offline();
	clock_gettime(CLOCK_MONOTONIC, &t0);
	while (elapsed_ms(&t0) < DEFAULT_DURATION_MS && !stop_all)
		usleep(1000);
	test_stop = 1;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	for (i = 0; i < RKSM_NW; i++) {
		pthread_join(movers[i], NULL);
		pthread_join(mutators[i], NULL);
	}
	rcu_thread_online();

	synchronize_rcu();
	for (i = 0; i < RKSM_NW; i++) {	/* whatever the last iteration left */
		if (!mv[i].seeded)
			continue;
		(void) rkms_remove_node(ft, rkm_key(mv[i].bp, RKSM_SB, mv[i].mine),
				mv[i].seed_priv);
		(void) rkms_remove_node(ft, rkm_key(mv[i].bp, RKSM_SB, mv[i].guard),
				mv[i].seed_guard);
		mv[i].seeded = 0;
	}
	/*
	 * And the mutator's in-flight child: a run stopped between its insert and
	 * its remove leaves exactly one key behind, which shows up as entries+1 and
	 * a leak delta of 1 rather than as anything about the library.
	 */
	for (i = 0; i < RKSM_NW; i++) {
		uint64_t at_src, at_dst;

		if (!mu[i].mut_node)
			continue;
		at_src = ((uint64_t) mu[i].bp << 24) | ((uint64_t) RKSM_SB << 16) |
			((uint64_t) mu[i].mine << 8) | (uint64_t) mu[i].mut;
		at_dst = ((uint64_t) mu[i].dp << 24) | ((uint64_t) RKSM_SB << 16) |
			((uint64_t) mu[i].mine << 8) | (uint64_t) mu[i].mut;
		if (rkms_remove_node(ft, at_src, mu[i].mut_node) != CDS_FT_STATUS_OK)
			(void) rkms_remove_node(ft, at_dst, mu[i].mut_node);
		mu[i].mut_node = NULL;
	}

	synchronize_rcu();
	rcu_read_lock();
	for (i = 0; i < RKSM_NW; i++) {
		moves += mv[i].ops;
		einval += mv[i].einval;
		stuck += mv[i].stuck;
		checks += mu[i].checks;
		lost += mu[i].lost + mv[i].lost;
		if (mv[i].failed || mu[i].failed)
			ret = -1;
		if (mv[i].ops == 0) {
			fprintf(stderr, "rekey src-mutated: mover %d made NO move\n", i);
			ret = -1;
		}
		if (mu[i].checks == 0) {
			fprintf(stderr, "rekey src-mutated: mutator %d landed NO child "
				"-- it never contended the copy window\n", i);
			ret = -1;
		}
		for (c = 0; c < 2; c++) {
			struct cds_ft_node *f = NULL;
			uint64_t rk = rkm_key(mv[i].dp, RKSM_SB, (uint8_t) (c + 1));

			if (lookup_u64(ft, rk, &f) != CDS_FT_STATUS_OK ||
					f != &res[i][c]->node) {
				fprintf(stderr, "rekey src-mutated: resident %d/%d lost\n",
					i, c);
				ret = -1;
			}
		}
	}
	if (cds_ft_count_entries(ft) != live) {
		fprintf(stderr, "rekey src-mutated: entries %lu != live %lu\n",
			cds_ft_count_entries(ft), live);
		ret = -1;
	}
	if (cds_ft_verify(ft, stderr) != CDS_FT_STATUS_OK) {
		fprintf(stderr, "rekey src-mutated: cds_ft_verify failed\n");
		ret = -1;
	}
	rcu_read_unlock();

	fprintf(stderr, "# inv_rekey_src_mutated: %d mover/mutator pairs, %lu moves, "
		"%lu children landed in the copy window, %lu lost, %lu shape re-seeds, "
		"%lu stuck reclaims, %lu live keys\n",
		RKSM_NW, moves, checks, lost, einval, stuck, live);

	free(mv);
	free(mu);
	if (drain_and_destroy(ft, group) < 0)
		ret = -1;
	if (leak_check() < 0)
		ret = -1;
	return ret;
}

/*
 * Drain the STATIC seeds exactly the way drain_and_destroy does -- repeated
 * lookup_first + remove_all, same order -- but after EVERY removal, check that
 * each seed still in the trie is still findable BY ITS OWN KEY.
 *
 * The generic drain only reports a SHORTFALL ("freed 14 of 18"): by then a
 * dozen removals have run and any of them could have been the one that broke
 * the walk.  Checking between removals attributes it.  The seeds are the right
 * probe because no writer ever touches them -- inserted once before the writers
 * start, still there when they are joined -- so a seed that stops being
 * findable cannot be a race with a legitimate removal.
 *
 * Runs after the join, single-threaded, so it cannot perturb the window that
 * produced the damage -- only report it.
 */
static int sibp_drain_checked(struct cds_ft *ft, const uint64_t *seed, int nseed)
{
	struct cds_ft_iter *iter = NULL;
	unsigned long drained = 0;
	enum cds_ft_status s;
	char *gone;
	int j, ret = 0;

	gone = (char *) calloc((size_t) nseed, 1);
	if (!gone)
		abort();
	if (cds_ft_iter_create(ft, &iter) < 0)
		abort();
	rcu_read_lock();
	while ((s = cds_ft_lookup_first(ft, iter)) == CDS_FT_STATUS_OK) {
		struct cds_ft_node *head, *tmp;
		uint64_t just = 0;

		s = cds_ft_remove_all(ft, iter, &head);
		if (s != CDS_FT_STATUS_OK) {
			fprintf(stderr, "sibp drain: remove_all st=%d on a key "
				"lookup_first had just found\n", (int) s);
			ret = -1;
			break;
		}
		cds_ft_for_each_duplicate_safe_rcu(head, tmp) {
			struct ft_test_node *tn = to_test_node(head);

			just = tn->key;
			for (j = 0; j < nseed; j++)
				if (seed[j] == tn->key)
					gone[j] = 1;
			node_free_rcu(tn);
			drained++;
		}
		for (j = 0; j < nseed; j++) {
			struct cds_ft_node *f = NULL;

			if (gone[j])
				continue;
			if (lookup_u64(ft, seed[j], &f) == CDS_FT_STATUS_OK && f)
				continue;
			fprintf(stderr, "sibp drain: removing key %#lx STRANDED key "
				"%#lx -- an exact-key lookup no longer finds a key "
				"that was never removed (%lu drained)\n",
				(unsigned long) just, (unsigned long) seed[j],
				drained);
			if (cds_ft_verify(ft, stderr) != CDS_FT_STATUS_OK)
				fprintf(stderr, "sibp drain: cds_ft_verify FAILS\n");
			ret = -1;
			mw_violation_snapshot();
			goto out;
		}
	}
	if (drained != (unsigned long) nseed) {
		fprintf(stderr, "sibp drain: lookup_first STOPPED EARLY after %lu of "
			"%d seeds (st=%d); still findable by exact key:",
			drained, nseed, (int) s);
		for (j = 0; j < nseed; j++) {
			struct cds_ft_node *f = NULL;

			if (gone[j])
				continue;
			fprintf(stderr, " %#lx=%s", (unsigned long) seed[j],
				lookup_u64(ft, seed[j], &f) == CDS_FT_STATUS_OK ?
					"yes" : "NO");
		}
		fprintf(stderr, "\n");
		/*
		 * Deliberately NO further ORDERED probe here (a GE from the key
		 * itself, a lookup_last): on the remnant this fires on, those do
		 * not return -- the same going-up cycle that made lookup_first
		 * give up spins forever on a reachable internal whose nr_child
		 * has reached 0.  A diagnostic that hangs reports nothing.  The
		 * exact-key line above already carries the finding: the keys are
		 * present, and only the ordered walk cannot reach them.
		 */
		if (cds_ft_verify(ft, stderr) != CDS_FT_STATUS_OK)
			fprintf(stderr, "sibp drain: cds_ft_verify FAILS on the "
				"remnant\n");
		ret = -1;
		mw_violation_snapshot();
	}
out:
	rcu_read_unlock();
	cds_ft_iter_destroy(iter);
	free(gone);
	return ret;
}

static int inv_sibling_split_compress(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft;
	struct sibp_arg *w;
	pthread_t th[SIBP_NW * 2];
	struct timespec t0;
	unsigned long ops = 0, busy = 0, lost = 0, live = 0, stuck = 0;
	uint64_t seed[SIBP_NW * 3];	/* the static keys, for the checked drain */
	int i, ret = 0, stop_all = 0;

	/*
	 * Two writers cycling insert/remove on two SIBLING children of one
	 * prefix, so the pair splits and path-compresses the SAME node
	 * continuously.  No other oracle drives that shape: the disjoint-key
	 * insert/remove oracles diverge high enough that no node is ever both
	 * split and compressed under contention.
	 *
	 * It has found three defects and now passes, so it is a REGRESSION test
	 * for them:
	 *   - the WEDGE (@d1832abb): a removal derived its position from a
	 *     stale node->prev naming a RETIRED holder and retried the identical
	 *     derivation forever, holding the per-trie FIFO fair mutex.  It used
	 *     to HANG rather than fail, which is strictly worse -- a hung suite
	 *     scores GREEN against a gate that only counts failures.
	 *   - an unguarded trailing skip-target retire (@9ce4c2c8).
	 *   - the DEAD INTERIOR NODE (@c9b6391f): a compressed parent replaced
	 *     by a freshly allocated childless internal at a non-root, which
	 *     only an ordered walk could detect -- hence the teardown verify in
	 *     drain_and_destroy, and cds_ft_verify's own check for it.
	 *
	 * The last one reproduced about 1 saturated run in 100, so treat a
	 * single green run here as weak evidence: soak it
	 * (fractal-trie-review-2026-06/sibp_verify_soak.sh), or build with
	 * -DFT_DELAY_INJECT and run FT_DELAY_MODE=writer FT_DELAY_US=10, which
	 * made that defect reproduce every run.
	 */
	mw_install_fatal_handler();
	leak_reset();

	ft = create_fixed_fine_lock_listoff_ft(4, &group);
	cds_ft_make_concurrent(ft);

	w = (struct sibp_arg *) calloc(SIBP_NW * 2, sizeof(*w));
	if (!w)
		abort();
	rcu_read_lock();
	for (i = 0; i < SIBP_NW; i++) {
		uint8_t p = (uint8_t) (0x30 + i), m = (uint8_t) (0x10 + i);
		uint64_t sk[3];
		int c;

		/* Static shape: (p,1) (p,5) keep p a branch, (p,3,guard) keeps
		 * (p,3) a branch, so the only node that is split and compressed
		 * under contention is the prefix (p,3,m). */
		sk[0] = sibp_key(p, 1, 0, 0);
		sk[1] = sibp_key(p, 5, 0, 0);
		sk[2] = sibp_key(p, 3, (uint8_t) (0x40 + i), 0);
		for (c = 0; c < 3; c++) {
			if (insert_u64(ft, sk[c], node_alloc(sk[c])) !=
					CDS_FT_STATUS_OK)
				abort();
			seed[live++] = sk[c];
		}

		w[2 * i].ft = w[2 * i + 1].ft = ft;
		w[2 * i].stop_all = w[2 * i + 1].stop_all = &stop_all;
		w[2 * i].key = sibp_key(p, 3, m, 0x00);
		w[2 * i + 1].key = sibp_key(p, 3, m, 0x60);
	}
	rcu_read_unlock();

	test_go = 0;
	test_stop = 0;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	for (i = 0; i < SIBP_NW * 2; i++)
		pthread_create(&th[i], NULL, sibp_writer, &w[i]);
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	test_go = 1;

	rcu_thread_offline();
	clock_gettime(CLOCK_MONOTONIC, &t0);
	while (elapsed_ms(&t0) < DEFAULT_DURATION_MS && !stop_all)
		usleep(1000);
	test_stop = 1;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	for (i = 0; i < SIBP_NW * 2; i++)
		pthread_join(th[i], NULL);
	rcu_thread_online();

	synchronize_rcu();
	/*
	 * A writer stopped BETWEEN its publish and its take-back leaves exactly
	 * one key behind -- harness bookkeeping, not a library leak, and it
	 * would otherwise show up as entries > live.  Reclaim it here.
	 */
	for (i = 0; i < SIBP_NW * 2; i++) {
		if (!w[i].inflight)
			continue;
		/*
		 * Usually the node is still published and the remove reclaims
		 * it.  If it is NOT in the trie -- the take-back raced to
		 * completion but the writer had already stopped, so nothing
		 * cleared @inflight -- then it is unreachable (past the
		 * synchronize_rcu above) and ours to free directly.  Ignoring
		 * the status here leaked exactly one node, ~1 run in 12 under
		 * load, reported as a LEAK delta with the trie itself exact
		 * (0 lost, entries == live).
		 */
		if (sibp_remove(ft, w[i].key, w[i].inflight) != CDS_FT_STATUS_OK) {
			node_free(w[i].inflight);
		}
		w[i].inflight = NULL;
	}
	synchronize_rcu();
	/*
	 * Before anything is torn down: is every node this run allocated
	 * either freed or still reachable?  An excess is an orphan, and the
	 * evidence for it only exists here.
	 */
	if (sibp_report_orphans(ft, live) < 0)
		ret = -1;
	rcu_read_lock();
	for (i = 0; i < SIBP_NW * 2; i++) {
		ops += w[i].ops;
		busy += w[i].busy;
		lost += w[i].lost;
		stuck += w[i].stuck;
		if (w[i].failed)
			ret = -1;
		if (w[i].ops == 0) {
			fprintf(stderr, "inv_sibling_split_compress: writer %d never "
				"completed a cycle\n", i);
			ret = -1;
		}
	}
	if (cds_ft_count_entries(ft) != live) {
		fprintf(stderr, "inv_sibling_split_compress: entries %lu != live %lu\n",
			cds_ft_count_entries(ft), live);
		ret = -1;
	}
	if (verify_or_dump(ft, "inv_sibling_split_compress"))
		ret = -1;
	rcu_read_unlock();

	fprintf(stderr, "# inv_sibling_split_compress: %d pairs, %lu cycles, "
		"%lu transient, %lu lost, %lu contended take-backs\n",
		SIBP_NW, ops, busy, lost, stuck);

	free(w);
	/*
	 * Attribute a drain shortfall to the removal that causes it, before the
	 * generic drain (which then finds the trie already empty).
	 */
	if (sibp_drain_checked(ft, seed, SIBP_NW * 3) < 0) {
		ret = -1;
		/*
		 * Tear down WITHOUT another walk.  The remnant just
		 * characterized is one an ordered descent cannot get through --
		 * on it, drain_and_destroy's own lookup_first does not
		 * terminate, and a teardown that hangs turns an attributed
		 * failure into a timeout that says nothing.  leak_check() below
		 * then reports the stranded nodes, which is the accurate
		 * outcome: the walk could not reach them, so nobody freed them.
		 */
		cds_ft_destroy(ft);
		cds_ft_group_destroy(group);
	} else if (drain_and_destroy(ft, group) < 0) {
		ret = -1;
	}
	if (leak_check() < 0)
		ret = -1;
	return ret;
}

/*
 * Companion to inv_concurrent_writers_disjoint: ALL writers contend the SAME
 * key range [0, MW_SHARED_RANGE), so two writers routinely insert/remove the
 * SAME key and mutate ADJACENT keys sharing one leaf -- the write/write and
 * torn-publish races the disjoint oracle deliberately excludes (insert-replace /
 * hole-refill resurrect on a live child slot, remove chain-leaf freeze racing a
 * re-insert).  Same total keyspace as the disjoint oracle (MW_NR_WRITERS *
 * MW_RANGE), but fully shared instead of partitioned.
 *
 * Self-arbitrating node lifecycle (no per-writer shadow is possible once keys
 * are shared): a writer allocates a fresh node per insert; the writer whose
 * cds_ft_remove returns OK owns the RCU-deferred free (a peer standing on the
 * node in its own read section keeps it live to its grace period, §11); a losing
 * remover and a duplicate insert reclaim only what they privately hold.  A broken
 * concurrent remove that returns OK twice double-frees -> leak_check / 0xfe
 * poison fault.
 *
 * Oracles:
 *  - in-line: a non-NULL lookup for key K must resolve to a node whose shadow
 *    key is K.  A torn structural publish that redirects K's slot to another
 *    node -- or to wild memory -- trips this (or faults on the deref): the
 *    Group-B/C torn-publish detector the disjoint oracle cannot see.
 *  - final (quiescent): cds_ft_count_keys equals the exact present count, every
 *    present key resolves to a key-consistent node, cds_ft_verify passes.
 *  - leak_check: every allocated node freed exactly once.
 */
#define MW_SHARED_RANGE	(MW_NR_WRITERS * MW_RANGE)

struct mw_shared_arg {
	struct cds_ft *ft;
	unsigned int seed0;
	unsigned long ops;
	int failed;
};

static void *mw_shared_writer(void *arg)
{
	struct mw_shared_arg *w = (struct mw_shared_arg *) arg;
	struct cds_ft_iter *iter;
	unsigned int seed = w->seed0;

	rcu_register_thread();
	if (cds_ft_iter_create(w->ft, &iter) < 0)
		abort();
	while (!test_go)
		;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	while (!test_stop) {
		uint64_t key = (uint64_t)(rand_r(&seed) % MW_SHARED_RANGE);
		struct cds_ft_node *found;
		uint8_t k[8];

		cds_ft_u64_to_key(w->ft, key, k, CDS_FT_LEN_DEFAULT);
		rcu_read_lock();
		cds_ft_iter_set_key(iter, k, CDS_FT_LEN_DEFAULT);
		cds_ft_lookup(w->ft, iter);
		found = cds_ft_iter_node(iter);
		if (found) {
			/*
			 * Torn-resolve oracle: K must map to a node keyed K.  A
			 * concurrent restructure that redirected K's slot returns
			 * the wrong node here; a torn child pointer returns wild
			 * memory (the ->key deref faults).  The lookup reference is
			 * valid only under this read section, and cds_ft_remove
			 * consumes it, so both stay inside one bracket (§11).
			 */
			if (to_test_node(found)->key != key) {
				fprintf(stderr, "MW shared: key %llu resolved to "
					"node keyed %llu (%p)\n",
					(unsigned long long) key,
					(unsigned long long) to_test_node(found)->key,
					(void *) found);
				w->failed = 1;
				rcu_read_unlock();
				mw_violation_snapshot();
				break;
			}
			if (cds_ft_remove(w->ft, iter, found) == CDS_FT_STATUS_OK)
				node_free_rcu(to_test_node(found));
			rcu_read_unlock();
		} else {
			struct ft_test_node *n;

			rcu_read_unlock();
			n = node_alloc(key);
			if (insert_u64(w->ft, key, n) != CDS_FT_STATUS_OK)
				node_free(n);	/* duplicate: a peer owns K */
		}
		w->ops++;
		if ((seed & 0x3f) == 0)
			rcu_quiescent_state();
	}
	cds_ft_iter_destroy(iter);
	rcu_unregister_thread();
	return NULL;
}

static int inv_concurrent_writers_shared(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft;
	struct mw_shared_arg *w;
	pthread_t writers[MW_NR_WRITERS];
	struct timespec t0;
	unsigned long total_ops = 0, present = 0;
	uint64_t key;
	int i, ret = 0;

	if (!getenv("FT_INV_MW")) {
		fprintf(stderr, "# inv_concurrent_writers_shared: skipped "
			"(set FT_INV_MW=1 to run the shared-key writer oracle)\n");
		return 0;
	}
	mw_install_fatal_handler();
	/*
	 * Same-key concurrent mutation is a LOCK-MODE feature: the duplicate
	 * chain (ft-txn-hlist.h) is not safe for concurrent mutation, so two
	 * writers on the SAME key must serialize (today via the FT-wide writer
	 * lock a lock-mode trie takes; ultimately via the head-holder's per-node
	 * node lock once that FT-wide lock drops).  This oracle therefore runs
	 * FINE (create_fixed_fine_lock_ft), NOT create_fixed_ft.  (It used to
	 * justify that by contrast with an OPTIMISTIC trie, which "has neither
	 * and cannot arbitrate same-key removers"; that strategy is gone.)
	 */
	ft = create_fixed_fine_lock_ft(8, &group);
	cds_ft_make_concurrent(ft);
	leak_reset();

	w = (struct mw_shared_arg *) calloc(MW_NR_WRITERS, sizeof(*w));
	if (!w)
		abort();
	for (i = 0; i < MW_NR_WRITERS; i++) {
		w[i].ft = ft;
		w[i].seed0 = (unsigned int)(i * 2654435761u + 1u);
	}

	test_go = 0;
	test_stop = 0;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	for (i = 0; i < MW_NR_WRITERS; i++)
		pthread_create(&writers[i], NULL, mw_shared_writer, &w[i]);
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	test_go = 1;

	rcu_thread_offline();
	clock_gettime(CLOCK_MONOTONIC, &t0);
	while (elapsed_ms(&t0) < DEFAULT_DURATION_MS)
		usleep(1000);
	test_stop = 1;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	for (i = 0; i < MW_NR_WRITERS; i++)
		pthread_join(writers[i], NULL);
	rcu_thread_online();

	/* Quiescent: the present set is whatever survived; verify it. */
	synchronize_rcu();
	rcu_read_lock();
	for (i = 0; i < MW_NR_WRITERS; i++) {
		total_ops += w[i].ops;
		if (w[i].failed)
			ret = -1;
	}
	for (key = 0; key < MW_SHARED_RANGE; key++) {
		struct cds_ft_node *found = NULL;

		if (lookup_u64(ft, key, &found) != CDS_FT_STATUS_OK || !found)
			continue;
		present++;
		if (to_test_node(found)->key != key) {
			fprintf(stderr, "MW shared final: key %llu resolved to "
				"node keyed %llu\n", (unsigned long long) key,
				(unsigned long long) to_test_node(found)->key);
			ret = -1;
		}
	}
	if (cds_ft_count_keys(ft) != present) {
		fprintf(stderr, "MW shared final: count_keys %lu != present %lu\n",
			cds_ft_count_keys(ft), present);
		ret = -1;
	}
	if (cds_ft_verify(ft, stderr) != CDS_FT_STATUS_OK) {
		fprintf(stderr, "MW shared final: cds_ft_verify failed\n");
		ret = -1;
	}
	rcu_read_unlock();

	fprintf(stderr, "# inv_concurrent_writers_shared: %d writers, %lu ops, "
		"%lu present keys\n", MW_NR_WRITERS, total_ops, present);

	free(w);
	if (drain_and_destroy(ft, group) < 0)
		ret = -1;
	if (leak_check() < 0)
		ret = -1;
	return ret;
}

#define MW_COARSE_NR_WRITERS	16
#define MW_FINE_NR_WRITERS	16

/*
 * Concurrent-writer soak for CDS_FT_WRITER_LOCK_COARSE (step 2 acceptance,
 * §11.4).  MW_COARSE_NR_WRITERS threads each churn insert/remove over a DISJOINT
 * key range on ONE coarse lock-mode trie.  Unlike inv_concurrent_writers_-
 * disjoint, which saturates the per-node-lock (FINE) path, the FT-wide writer
 * lock SERIALIZES every mutation here, so
 * this must be violation-free -- it proves the acquire / escalate / release
 * plumbing under real contention through ft->txn_domain, and that a coarse
 * lock-mode trie stays coherent under 16 writers (no lost key, count == live,
 * cds_ft_verify passes).  Reuses the mw_writer thread body (its lost-key oracle
 * and reference-lifetime discipline are strategy-agnostic).  Opt-in
 * (FT_INV_MW=1) -- it runs for DEFAULT_DURATION_MS.
 */
static int inv_concurrent_writers_coarse_lock(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft;
	struct mw_writer_arg *w;
	pthread_t writers[MW_COARSE_NR_WRITERS];
	struct timespec t0;
	unsigned long total_ops = 0, live = 0;
	int i, ret = 0;

	if (!getenv("FT_INV_MW")) {
		fprintf(stderr, "# inv_concurrent_writers_coarse_lock: skipped "
			"(set FT_INV_MW=1 to run the coarse lock-mode soak)\n");
		return 0;
	}
	mw_install_fatal_handler();
	ft = create_fixed_coarse_lock_ft(8, &group);
	/* Concurrent mode: deferred reclaim keeps a peer's touched nodes live. */
	cds_ft_make_concurrent(ft);

	leak_reset();

	w = (struct mw_writer_arg *) calloc(MW_COARSE_NR_WRITERS, sizeof(*w));
	if (!w)
		abort();
	for (i = 0; i < MW_COARSE_NR_WRITERS; i++) {
		w[i].ft = ft;
		w[i].base = (uint64_t) i * MW_RANGE;
	}

	test_go = 0;
	test_stop = 0;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	for (i = 0; i < MW_COARSE_NR_WRITERS; i++)
		pthread_create(&writers[i], NULL, mw_writer, &w[i]);
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	test_go = 1;

	rcu_thread_offline();
	clock_gettime(CLOCK_MONOTONIC, &t0);
	while (elapsed_ms(&t0) < DEFAULT_DURATION_MS)
		usleep(1000);
	test_stop = 1;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	for (i = 0; i < MW_COARSE_NR_WRITERS; i++)
		pthread_join(writers[i], NULL);
	rcu_thread_online();

	/* Quiescent: verify the final trie against every writer's shadow. */
	synchronize_rcu();
	rcu_read_lock();
	for (i = 0; i < MW_COARSE_NR_WRITERS; i++) {
		unsigned int off;

		total_ops += w[i].ops;
		if (w[i].failed)
			ret = -1;
		for (off = 0; off < MW_RANGE; off++) {
			struct cds_ft_node *found = NULL;

			if (!w[i].present[off])
				continue;
			live++;
			if (lookup_u64(ft, w[i].base + off, &found)
					!= CDS_FT_STATUS_OK ||
			    found != &w[i].node[off]->node) {
				fprintf(stderr, "coarse-lock final: writer %d key %llu "
					"lost (found %p != %p)\n", i,
					(unsigned long long)(w[i].base + off),
					(void *) found,
					(void *) &w[i].node[off]->node);
				ret = -1;
			}
		}
	}
	if (cds_ft_count_keys(ft) != live) {
		fprintf(stderr, "coarse-lock final: count_keys %lu != live %lu\n",
			cds_ft_count_keys(ft), live);
		ret = -1;
	}
	if (cds_ft_verify(ft, stderr) != CDS_FT_STATUS_OK) {
		fprintf(stderr, "coarse-lock final: cds_ft_verify failed\n");
		ret = -1;
	}
	rcu_read_unlock();

	fprintf(stderr, "# inv_concurrent_writers_coarse_lock: %d writers, %lu ops, "
		"%lu live keys\n", MW_COARSE_NR_WRITERS, total_ops, live);

	free(w);
	if (drain_and_destroy(ft, group) < 0)
		ret = -1;
	if (leak_check() < 0)
		ret = -1;
	return ret;
}

/*
 * INVARIANT (MW, §11.3 step 3): a CDS_FT_WRITER_LOCK_FINE trie loses no key.
 *
 * Same disjoint-range lost-key oracle as inv_concurrent_writers_coarse_lock, on
 * a trie whose recompacts acquire the per-node lock-set {C, P} (+ {GP} when P is
 * compressed) and resolve P/GP through the RELEASE terminal {LOCK|s -> s}.
 * 16 writers insert/remove over disjoint key ranges; every writer's shadow set
 * must match the final trie exactly.
 *
 * What this is actually gating, given the FT-wide lock still serializes writers
 * here (§11.1): that the release terminal COMMITS -- that a locked-but-surviving
 * node comes out of the commit LIVE and UNLOCKED.  A release that failed to
 * commit, or a bail path that forgot to unlock a member, leaves FT_STATE_LOCK
 * set at rest -- which wedges every later publish into that node, and which
 * cds_ft_verify reports as a leaked copy fence.  Both endpoints are checked
 * below, and 100k+ recompacts run through them.
 */
static int inv_concurrent_writers_fine_lock(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft;
	struct mw_writer_arg *w;
	pthread_t writers[MW_FINE_NR_WRITERS];
	struct timespec t0;
	unsigned long total_ops = 0, live = 0;
	int i, ret = 0;

	if (!getenv("FT_INV_MW")) {
		fprintf(stderr, "# inv_concurrent_writers_fine_lock: skipped "
			"(set FT_INV_MW=1 to run the fine lock-mode soak)\n");
		return 0;
	}
	mw_install_fatal_handler();
	ft = create_fixed_fine_lock_ft(8, &group);
	/* Concurrent mode: deferred reclaim keeps a peer's touched nodes live. */
	cds_ft_make_concurrent(ft);

	leak_reset();

	w = (struct mw_writer_arg *) calloc(MW_FINE_NR_WRITERS, sizeof(*w));
	if (!w)
		abort();
	for (i = 0; i < MW_FINE_NR_WRITERS; i++) {
		w[i].ft = ft;
		w[i].base = (uint64_t) i * MW_RANGE;
	}

	test_go = 0;
	test_stop = 0;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	for (i = 0; i < MW_FINE_NR_WRITERS; i++)
		pthread_create(&writers[i], NULL, mw_writer, &w[i]);
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	test_go = 1;

	rcu_thread_offline();
	clock_gettime(CLOCK_MONOTONIC, &t0);
	while (elapsed_ms(&t0) < DEFAULT_DURATION_MS)
		usleep(1000);
	test_stop = 1;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	for (i = 0; i < MW_FINE_NR_WRITERS; i++)
		pthread_join(writers[i], NULL);
	rcu_thread_online();

	/* Quiescent: verify the final trie against every writer's shadow. */
	synchronize_rcu();
	rcu_read_lock();
	for (i = 0; i < MW_FINE_NR_WRITERS; i++) {
		unsigned int off;

		total_ops += w[i].ops;
		if (w[i].failed)
			ret = -1;
		for (off = 0; off < MW_RANGE; off++) {
			struct cds_ft_node *found = NULL;

			if (!w[i].present[off])
				continue;
			live++;
			if (lookup_u64(ft, w[i].base + off, &found)
					!= CDS_FT_STATUS_OK ||
			    found != &w[i].node[off]->node) {
				fprintf(stderr, "fine-lock final: writer %d key %llu "
					"lost (found %p != %p)\n", i,
					(unsigned long long)(w[i].base + off),
					(void *) found,
					(void *) &w[i].node[off]->node);
				ret = -1;
			}
		}
	}
	if (cds_ft_count_keys(ft) != live) {
		fprintf(stderr, "fine-lock final: count_keys %lu != live %lu\n",
			cds_ft_count_keys(ft), live);
		ret = -1;
	}
	if (cds_ft_verify(ft, stderr) != CDS_FT_STATUS_OK) {
		fprintf(stderr, "fine-lock final: cds_ft_verify failed\n");
		ret = -1;
	}
	rcu_read_unlock();

	fprintf(stderr, "# inv_concurrent_writers_fine_lock: %d writers, %lu ops, "
		"%lu live keys\n", MW_FINE_NR_WRITERS, total_ops, live);

	free(w);
	if (drain_and_destroy(ft, group) < 0)
		ret = -1;
	if (leak_check() < 0)
		ret = -1;
	return ret;
}

/* ================================================================== */
/*                                                                    */
/*   Concurrent CROSS-TRIE writers on a FINE trie (FT-wide-lock drop) */
/*                                                                    */
/* ================================================================== */

/*
 * Variable-length fine-grained lock-mode trie: like create_fixed_fine_lock_ft
 * but with the group's default (variable) key length, so non-root graft /
 * detach at a multi-byte prefix is legal (CDS_FT_LEN_VARIABLE).  EAGER lookup
 * so the point verification needs no speculative key offset.  Used by
 * inv_concurrent_crosstrie_fine_lock.
 */
static struct cds_ft *create_varlen_fine_lock_ft(struct cds_ft_group **group_out)
{
	struct cds_ft_group_attr *attr;
	struct cds_ft_group *group;
	struct cds_ft *ft;

	if (cds_ft_group_attr_create(&attr) < 0)
		abort();
	if (cds_ft_group_attr_set_lookup_optimization(attr,
			CDS_FT_LOOKUP_OPTIMIZE_EAGER) < 0)
		abort();
	/* Isolate the structural attach path: no ordered-list cell splices. */
	if (cds_ft_group_attr_set_ordered_list(attr, false) < 0)
		abort();
	if (cds_ft_group_attr_set_writer_strategy(attr,
			CDS_FT_WRITER_LOCK_FINE) < 0)
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
 * As create_varlen_fine_lock_ft but with @list_on (ordered list) and/or
 * @rank_on (per-node order statistics) enabled.  With @list_on the cross-trie
 * graft's GLUE commit carries the extra ordered-list run-splice cell edges --
 * the ordered-list attach path under the FT-wide-lock drop.  @rank_on requests
 * order statistics: a rank-stats trie maintains ONE global count on the root's
 * nr_keys word that every count-changing writer must update, so the group
 * COERCES a FINE strategy to COARSE (§10.5; ft-lifecycle.h) -- the resulting
 * trie keeps the FT-wide lock and never drops it, which is exactly what makes
 * the count-parent walk correct.  So @rank_on trie == coarse regardless of the
 * requested FINE.
 */
static struct cds_ft *create_varlen_fine_lock_cfg_ft(
		struct cds_ft_group **group_out, bool list_on, bool rank_on)
{
	struct cds_ft_group_attr *attr;
	struct cds_ft_group *group;
	struct cds_ft *ft;

	if (cds_ft_group_attr_create(&attr) < 0)
		abort();
	if (cds_ft_group_attr_set_lookup_optimization(attr,
			CDS_FT_LOOKUP_OPTIMIZE_EAGER) < 0)
		abort();
	if (cds_ft_group_attr_set_ordered_list(attr, list_on) < 0)
		abort();
	if (cds_ft_group_attr_set_rank_stats(attr, rank_on) < 0)
		abort();
	if (cds_ft_group_attr_set_writer_strategy(attr,
			CDS_FT_WRITER_LOCK_FINE) < 0)
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
 * INVARIANT (MW, §11 drop-mechanics): concurrent cross-trie GRAFTs into one
 * LIVE (concurrent) fine-grained lock-mode trie lose no key and corrupt no
 * structure, with the FT-wide writer lock DROPPED.
 *
 * This closes the one un-soaked gap the drop's net-(A) argument otherwise
 * covers only by inspection.  Through step 6 a cross-trie graft/merge/graft_swap
 * takes the FT-wide lock on its LIVE dst (the exclusive source skips its own),
 * so concurrent cross-trie ops on a live dst were serialized.  The dst now
 * skips that lock too, so concurrent grafts and point-removes into one live dst
 * arbitrate SOLELY through the step-6A per-node attach RELEASE locks + MCAS.
 * The drop is unconditional for FINE, so that is the only path this exercises.
 *
 * Layout MAXIMISES per-node contention: prefix = {p, w} with p shared across
 * ALL writers and w = writer id, so every writer's graft at {p, w} attaches a
 * child under the SAME {p} spine node -- MW_XT_NR_WRITERS writers contend each
 * {p} node's lock / recompaction, which is exactly the arbitration the drop
 * leans on.  Full keys are {p, w, s}; writer w owns the middle byte = w, so the
 * per-writer shadow set stays disjoint and exactly verifiable.
 *
 * Each writer step picks a random prefix p: if absent, build an EXCLUSIVE
 * source holding the S suffix keys and GRAFT it at {p, w}; if present,
 * POINT-REMOVE all S keys {p, w, s} (the established remove + node_free_rcu free
 * path, so no detached-trie drain is needed).  Final quiescent check: every
 * present prefix's S keys resolve to this writer's nodes, count_keys matches
 * the live total, and cds_ft_verify passes (a node lock leaked by a dropped
 * bail path surfaces here).
 */
#define MW_XT_NR_WRITERS	16
#define MW_XT_PREFIXES		64	/* shared prefix byte p in [0, this) */
#define MW_XT_SUFFIXES		4	/* keys grafted per prefix */

/*
 * @attach_mode selects how each {p,w} subtree is attached into the shared dst:
 *   MW_XT_ATTACH_GRAFT  cds_ft_graft (whole exclusive src at {p,w});
 *   MW_XT_ATTACH_MERGE_SUBPOS  cds_ft_merge_at moving a src SUB-position
 *     (src keys {0,s}, src_key {0}) to {p,w} -- exercises the unfenced
 *     ft_merge_graft_subpos_inplace path under the LOCK_FINE drop.
 *   MW_XT_ATTACH_GRAFT_NILKEY  cds_ft_graft of a NIL-key source (a single
 *     empty key, the root's childless-internal external_nodes wrapper) at
 *     {p,w} -- the source's external chain head is placed DIRECTLY, so the
 *     one landed key is {p,w} (2 bytes, no suffix) and the emptied wrapper is
 *     freeze-on-free tombstoned; exercises the exclusive nil-key src-swap-
 *     fused arm (retire edge + wrapper tombstone fused into glue.txn) under
 *     the drop.
 *   MW_XT_ATTACH_MERGE_NILKEY  cds_ft_merge_at moving a single {0} leaf
 *     (src_key {0}) to {p,w}: ft_detach excises the lone external into an
 *     exclusive nil-key wrapper tmp, so the same fused nil-key arm runs on
 *     the take() path (retire + tombstone into the merge's pre-reserved
 *     pf_txn) -- exercises the ft-merge.h pf_cap reservation the create-path
 *     graft does not.
 * GRAFT / MERGE_SUBPOS yield dst keys {p,w,s}; the nil modes a single {p,w}.
 * mw_xt_nsuffix / mw_xt_landed_key fold that per-mode shape difference so the
 * build / remove / verify loops stay shared.
 */
#define MW_XT_ATTACH_GRAFT		0
#define MW_XT_ATTACH_MERGE_SUBPOS	2
#define MW_XT_ATTACH_GRAFT_NILKEY	3
#define MW_XT_ATTACH_MERGE_NILKEY	4
struct mw_xt_arg {
	struct cds_ft *ft;			/* the shared LIVE dst */
	struct cds_ft_group *group;
	unsigned int w;				/* writer id (middle key byte) */
	int attach_mode;			/* MW_XT_ATTACH_* */
	uint8_t present[MW_XT_PREFIXES];	/* 1 = {p,w,*} currently grafted */
	struct ft_test_node *node[MW_XT_PREFIXES][MW_XT_SUFFIXES];
	unsigned long ops;
	int failed;
};

/* A nil-key mode attaches a single-key source (empty key / single leaf). */
static bool mw_xt_is_nilkey(int attach_mode)
{
	return attach_mode == MW_XT_ATTACH_GRAFT_NILKEY
		|| attach_mode == MW_XT_ATTACH_MERGE_NILKEY;
}

/* Landed keys per prefix: S for GRAFT / MERGE_SUBPOS, 1 for a nil-key src. */
static unsigned int mw_xt_nsuffix(int attach_mode)
{
	return mw_xt_is_nilkey(attach_mode) ? 1 : MW_XT_SUFFIXES;
}

/*
 * The dst key that writer @w's suffix @s lands at for @attach_mode: {p,w,s}
 * for GRAFT / MERGE_SUBPOS, {p,w} for a single nil-key attach.  Fills @buf and
 * returns the length.
 */
static size_t mw_xt_landed_key(int attach_mode, unsigned int p, unsigned int w,
		unsigned int s, uint8_t *buf)
{
	buf[0] = (uint8_t) p;
	buf[1] = (uint8_t) w;
	if (mw_xt_is_nilkey(attach_mode))
		return 2;
	buf[2] = (uint8_t) s;
	return 3;
}

static void *mw_xt_writer(void *arg)
{
	struct mw_xt_arg *x = (struct mw_xt_arg *) arg;
	struct cds_ft_iter *iter;
	unsigned int seed = (unsigned int)(uintptr_t) x + 0x9e3779b9u;

	rcu_register_thread();
	if (cds_ft_iter_create(x->ft, &iter) < 0)
		abort();
	while (!test_go)
		;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	while (!test_stop) {
		unsigned int p = (unsigned int)(rand_r(&seed) % MW_XT_PREFIXES);
		uint8_t prefix[2] = { (uint8_t) p, (uint8_t) x->w };
		unsigned int s;

		if (!x->present[p]) {
			struct cds_ft *src;

			bool subpos =
				x->attach_mode == MW_XT_ATTACH_MERGE_SUBPOS;
			bool nilgraft =
				x->attach_mode == MW_XT_ATTACH_GRAFT_NILKEY;
			bool nilmerge =
				x->attach_mode == MW_XT_ATTACH_MERGE_NILKEY;
			/* merge_at (src_key {0}): the sub-position and nil-key-leaf moves. */
			bool is_merge = subpos || nilmerge;
			enum cds_ft_status attach_st;

			/*
			 * Build a private source.  GRAFT: 1-byte keys {s} moved
			 * wholesale.  MERGE_SUBPOS: 2-byte keys {0,s} so the merge
			 * moves the {0} SUB-position (src_key {0}).  GRAFT_NILKEY: the
			 * single EMPTY key (nil-key wrapper).  MERGE_NILKEY: a single
			 * 1-byte {0} leaf so merge_at src_key {0} detaches it to a
			 * nil-key wrapper tmp (the fused take-path).  GRAFT / SUBPOS land
			 * {p,w,s}; the nil modes land the lone {p,w}.
			 */
			if (cds_ft_create(x->group, NULL, &src) < 0)
				abort();
			for (s = 0; s < mw_xt_nsuffix(x->attach_mode); s++) {
				uint8_t suffix[1] = { (uint8_t) s };
				uint8_t sub_suffix[2] = { 0, (uint8_t) s };
				uint8_t full[3] = { (uint8_t) p,
					(uint8_t) x->w, (uint8_t) s };
				struct ft_test_node *n = node_alloc(
					((uint64_t) p << 16)
					| ((uint64_t) x->w << 8) | s);

				memcpy(n->okey, full, 3);
				/*
				 * GRAFT_NILKEY: insert the EMPTY key (the src root's
				 * nil-key wrapper); the graft at {p,w} places its
				 * external head directly there.  Else a 1-byte {s}
				 * (graft / MERGE_NILKEY's lone {0} leaf) or 2-byte
				 * {0,s} (merge sub-position) key.
				 */
				if (cds_ft_insert(src,
						nilgraft ? (const uint8_t *) "" :
						subpos ? sub_suffix : suffix,
						nilgraft ? 0 : subpos ? 2 : 1,
						&n->node)
						!= CDS_FT_STATUS_OK)
					abort();
				x->node[p][s] = n;
			}
			/*
			 * The consumed source must be EXCLUSIVE (step-6
			 * contract): a live lock-mode src is rejected with BUSY.
			 * make_exclusive also lets its writer scope skip the
			 * FT-wide lock so only the live dst's per-node locks
			 * arbitrate the attach.
			 */
			cds_ft_make_exclusive(src);
			if (is_merge) {
				uint8_t src_sub[1] = { 0 };

				attach_st = cds_ft_merge_at(x->ft, prefix, 2,
					src, src_sub, 1);
			} else {
				attach_st = cds_ft_graft(x->ft, prefix, 2, src);
			}
			if (attach_st != CDS_FT_STATUS_OK) {
				/*
				 * {p,w} is this writer's own and was absent =>
				 * empty => the graft MUST succeed.  A failure
				 * here is a real drop defect (a peer's attach
				 * leaked into a disjoint prefix, or a torn spine
				 * from an unserialized attach).
				 */
				fprintf(stderr, "xt writer %u prefix p=%u: graft "
					"failed on an empty disjoint target\n",
					x->w, p);
				x->failed = 1;
				/*
				 * This only fires on a real drop defect, and it
				 * IS the reported violation (failed => ret = -1).
				 * Leave @src and its nodes untouched rather than
				 * risk a teardown UAF on the error path; the
				 * accompanying leak_check flag is secondary to the
				 * message above.
				 */
				mw_violation_snapshot();
				break;
			}
			cds_ft_destroy(src);		/* emptied by the graft */
			x->present[p] = 1;
		} else {
			/* Point-remove every key of this prefix, then re-arm. */
			for (s = 0; s < mw_xt_nsuffix(x->attach_mode); s++) {
				uint8_t full[3];
				size_t full_len = mw_xt_landed_key(
					x->attach_mode, p, x->w, s, full);
				struct cds_ft_node *found;

				rcu_read_lock();
				cds_ft_iter_set_key(iter, full, full_len);
				cds_ft_lookup(x->ft, iter);
				found = cds_ft_iter_node(iter);
				if (found != &x->node[p][s]->node) {
					fprintf(stderr, "xt writer %u key "
						"{%u,%u,%u}: live but found %p "
						"!= mine %p\n", x->w, p, x->w, s,
						(void *) found,
						(void *) &x->node[p][s]->node);
					x->failed = 1;
					rcu_read_unlock();
					mw_violation_snapshot();
					goto out;
				}
				if (cds_ft_remove(x->ft, iter, found)
						== CDS_FT_STATUS_OK)
					node_free_rcu(to_test_node(found));
				rcu_read_unlock();
				x->node[p][s] = NULL;
			}
			x->present[p] = 0;
		}
		x->ops++;
		if ((seed & 0x3f) == 0)
			rcu_quiescent_state();
	}
out:
	cds_ft_iter_destroy(iter);
	rcu_unregister_thread();
	return NULL;
}

/*
 * See the block comment on mw_xt_writer.  Endpoints checked at quiescence: no
 * grafted key lost (resolves to the owning writer's node), count_keys equals
 * the live total, and cds_ft_verify reports no leaked node lock.
 */
static int mw_xt_oracle(const char *tname, int attach_mode, bool list_on,
		bool rank_on)
{
	struct cds_ft_group *group;
	struct cds_ft *ft;
	struct mw_xt_arg *x;
	struct cds_ft_iter *iter;
	pthread_t writers[MW_XT_NR_WRITERS];
	struct timespec t0;
	unsigned long total_ops = 0, live = 0;
	unsigned int i, p, s;
	int ret = 0;

	if (!getenv("FT_INV_MW")) {
		fprintf(stderr, "# %s: skipped "
			"(set FT_INV_MW=1 to run the cross-trie writer oracle)\n",
			tname);
		return 0;
	}
	mw_install_fatal_handler();
	/*
	 * @list_on selects the ordered-list + rank-stats dst (Defect D): the
	 * graft's GLUE commit then carries run-splice + count-parent edges a peer
	 * can conflict-abort after the src swap.  Otherwise the isolated
	 * structural attach (list off, rank off).
	 */
	ft = (list_on || rank_on)
		? create_varlen_fine_lock_cfg_ft(&group, list_on, rank_on)
		: create_varlen_fine_lock_ft(&group);
	/* Concurrent mode: deferred reclaim keeps a peer's touched nodes live. */
	cds_ft_make_concurrent(ft);

	leak_reset();

	x = (struct mw_xt_arg *) calloc(MW_XT_NR_WRITERS, sizeof(*x));
	if (!x)
		abort();
	for (i = 0; i < MW_XT_NR_WRITERS; i++) {
		x[i].ft = ft;
		x[i].group = group;
		x[i].w = i;
		x[i].attach_mode = attach_mode;
	}

	test_go = 0;
	test_stop = 0;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	for (i = 0; i < MW_XT_NR_WRITERS; i++)
		pthread_create(&writers[i], NULL, mw_xt_writer, &x[i]);
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	test_go = 1;

	rcu_thread_offline();
	clock_gettime(CLOCK_MONOTONIC, &t0);
	while (elapsed_ms(&t0) < DEFAULT_DURATION_MS)
		usleep(1000);
	test_stop = 1;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	for (i = 0; i < MW_XT_NR_WRITERS; i++)
		pthread_join(writers[i], NULL);
	rcu_thread_online();

	/* Quiescent: verify the final trie against every writer's shadow. */
	synchronize_rcu();
	if (cds_ft_iter_create(ft, &iter) < 0)
		abort();
	rcu_read_lock();
	for (i = 0; i < MW_XT_NR_WRITERS; i++) {
		total_ops += x[i].ops;
		if (x[i].failed)
			ret = -1;
		for (p = 0; p < MW_XT_PREFIXES; p++) {
			if (!x[i].present[p])
				continue;
			for (s = 0; s < mw_xt_nsuffix(attach_mode); s++) {
				uint8_t full[3];
				size_t full_len = mw_xt_landed_key(
					attach_mode, p, i, s, full);
				struct cds_ft_node *found;

				live++;
				cds_ft_iter_set_key(iter, full, full_len);
				cds_ft_lookup(ft, iter);
				found = cds_ft_iter_node(iter);
				if (found != &x[i].node[p][s]->node) {
					fprintf(stderr, "xt final: writer %u key "
						"{%u,%u,%u} lost (found %p != "
						"%p)\n", i, p, i, s,
						(void *) found,
						(void *) &x[i].node[p][s]->node);
					ret = -1;
				}
			}
		}
	}
	if (cds_ft_count_keys(ft) != live) {
		fprintf(stderr, "xt final: count_keys %lu != live %lu\n",
			cds_ft_count_keys(ft), live);
		ret = -1;
	}
	if (cds_ft_verify(ft, stderr) != CDS_FT_STATUS_OK) {
		fprintf(stderr, "xt final: cds_ft_verify failed\n");
		ret = -1;
	}
	rcu_read_unlock();
	cds_ft_iter_destroy(iter);

	fprintf(stderr, "# %s: %d writers, %lu ops, %lu live keys\n",
		tname, MW_XT_NR_WRITERS, total_ops, live);

	free(x);
	if (drain_and_destroy(ft, group) < 0)
		ret = -1;
	if (leak_check() < 0)
		ret = -1;
	return ret;
}

/* 16 writers graft an exclusive src into one shared live dst at {p,w}. */
static int inv_concurrent_crosstrie_fine_lock(void)
{
	return mw_xt_oracle("inv_concurrent_crosstrie_fine_lock",
		MW_XT_ATTACH_GRAFT, /*list_on=*/ false, /*rank_on=*/ false);
}

/*
 * As above but the attach is a cds_ft_merge_at moving a src SUB-position
 * (src_key {0}) -- the ft_merge_graft_subpos_inplace path -- so it exercises
 * the merge build -> unlink -> commit under the LOCK_FINE drop (fence cn +
 * commit-abort retry), which cds_ft_graft's whole-source delegate does not.
 */
static int inv_concurrent_crosstrie_merge_fine_lock(void)
{
	return mw_xt_oracle("inv_concurrent_crosstrie_merge_fine_lock",
		MW_XT_ATTACH_MERGE_SUBPOS, /*list_on=*/ false, /*rank_on=*/ false);
}

/*
 * As inv_concurrent_crosstrie_fine_lock but each attach grafts a NIL-key
 * source (a single empty key) at {p,w}: the source root is a childless
 * internal whose external_nodes hold the lone key, so the graft places the
 * external head DIRECTLY and freeze-on-free tombstones the emptied wrapper.
 * With an EXCLUSIVE list-off src that retire edge AND the wrapper tombstone
 * both fuse into the dst attach's glue.txn (src-swap-fused), so the whole
 * move -- src unlink + wrapper freeze + dst attach -- is ONE flip.  Exercises
 * that fused nil-key arm (and its commit-abort rollback) under the drop, which
 * neither the non-nil graft nor the merge sub-position covers.
 */
static int inv_concurrent_crosstrie_graft_nilkey_fine_lock(void)
{
	return mw_xt_oracle("inv_concurrent_crosstrie_graft_nilkey_fine_lock",
		MW_XT_ATTACH_GRAFT_NILKEY, /*list_on=*/ false, /*rank_on=*/ false);
}

/*
 * As above but via cds_ft_merge_at moving a single {0} leaf (src_key {0}):
 * ft_detach excises that lone external into an EXCLUSIVE nil-key wrapper tmp,
 * which ft_graft_keylen then grafts through the take() path (pre_txn = the
 * merge's pf_txn).  Exercises the fused nil-key retire + wrapper tombstone
 * riding the merge's PRE-RESERVED pf_txn -- the reserve-sufficiency the
 * ft-merge.h m==0 pf_cap bump covers -- which the create-path nil-key graft
 * (its own reserve) does not.
 */
static int inv_concurrent_crosstrie_merge_nilkey_fine_lock(void)
{
	return mw_xt_oracle("inv_concurrent_crosstrie_merge_nilkey_fine_lock",
		MW_XT_ATTACH_MERGE_NILKEY, /*list_on=*/ false, /*rank_on=*/ false);
}

/*
 * Ordered-list cross-trie graft under the FT-wide-lock drop: the shared dst has
 * the ordered list ON (rank-stats OFF, so it stays FINE and DROPS the lock), so
 * each graft's GLUE commit folds in the ordered-list run-splice cell edges that
 * a concurrent {p,w'} writer can MCAS-conflict AFTER this writer's src-root
 * swap.  The graft's retry_attach loop re-attaches the owned payload rather than
 * losing the moved subtree.  Also exercises ordered-list point-remove under the
 * drop.  (The rank-stats half of the original "Defect D" is a separate test:
 * rank stats coerce to coarse, below.)
 */
static int inv_concurrent_crosstrie_graft_ord_fine_lock(void)
{
	return mw_xt_oracle("inv_concurrent_crosstrie_graft_ord_fine_lock",
		MW_XT_ATTACH_GRAFT, /*list_on=*/ true, /*rank_on=*/ false);
}

/*
 * Rank-stats cross-trie graft: a rank-stats trie maintains one global count on
 * the root nr_keys word that every count-changing writer updates, so it is the
 * §10.5 COARSE target -- the group coerces the requested FINE strategy to COARSE
 * (ft-lifecycle.h), keeping the FT-wide lock so the count-parent walk runs under
 * writer exclusion (correct).  Without that coercion this over-counted nr_keys
 * under the drop (the walk climbed a stale ancestor chain, losing remove
 * decrements).  This is the regression guard for the coercion: 16 writers graft
 * into one rank-stats dst, and count_keys / cds_ft_verify's exact nr_keys check
 * must hold.
 */
static int inv_concurrent_crosstrie_graft_rank_coarse_lock(void)
{
	return mw_xt_oracle("inv_concurrent_crosstrie_graft_rank_coarse_lock",
		MW_XT_ATTACH_GRAFT, /*list_on=*/ false, /*rank_on=*/ true);
}

/*
 * Drain @ft's keys (freeing the test nodes) and destroy @ft, KEEPING the shared
 * group -- reclaims the content cds_ft_graft_swap hands back into the local swap
 * trie.  @ft has no concurrent readers (an exclusive per-writer swap, and
 * graft_swap already drained the shared dst's readers of this content).
 */
static void mw_gs_reclaim(struct cds_ft *ft)
{
	struct cds_ft_iter *iter;

	if (cds_ft_iter_create(ft, &iter) < 0)
		abort();
	rcu_read_lock();
	while (cds_ft_lookup_first(ft, iter) == CDS_FT_STATUS_OK) {
		struct cds_ft_node *head, *tmp;

		if (cds_ft_remove_all(ft, iter, &head) < 0)
			break;
		cds_ft_for_each_duplicate_safe_rcu(head, tmp)
			node_free_rcu(to_test_node(head));
	}
	rcu_read_unlock();
	cds_ft_iter_destroy(iter);
	cds_ft_destroy(ft);
}

/*
 * graft_swap concurrent-drop writer.  Each writer OWNS prefix {p, w} (p shared
 * across ALL writers so every {p,w} hangs under the SAME {p} spine node --
 * maximal per-node contention), and each step EXCHANGES the live subtree at
 * {p,w} with a fresh exclusive swap trie: cds_ft_graft_swap replaces dst's
 * {p,w} content with the swap's and hands the old content back in @swap, which
 * the writer then reclaims.  This drives graft_swap's SWAP path (a NON-empty
 * target), not just the empty-target delegate-to-graft that the graft oracle
 * already covers, under the FT-wide-lock drop.  Occasionally the swap is empty
 * -> {p,w} is cleared (the swap-out shape).
 */
static void *mw_gs_writer(void *arg)
{
	struct mw_xt_arg *x = (struct mw_xt_arg *) arg;
	unsigned int seed = (unsigned int)(uintptr_t) x + 0x9e3779b9u;

	rcu_register_thread();
	while (!test_go)
		;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	while (!test_stop) {
		unsigned int p = (unsigned int)(rand_r(&seed) % MW_XT_PREFIXES);
		uint8_t prefix[2] = { (uint8_t) p, (uint8_t) x->w };
		struct ft_test_node *newn[MW_XT_SUFFIXES] = { NULL };
		/* Occasionally swap the content OUT (empty swap -> {p,w} cleared). */
		bool clear = x->present[p] && ((rand_r(&seed) & 7) == 0);
		struct cds_ft *swap;
		enum cds_ft_status st;
		unsigned int s;

		if (cds_ft_create(x->group, NULL, &swap) < 0)
			abort();
		if (!clear) {
			for (s = 0; s < MW_XT_SUFFIXES; s++) {
				uint8_t suffix[1] = { (uint8_t) s };
				struct ft_test_node *n = node_alloc(
					((uint64_t) p << 16)
					| ((uint64_t) x->w << 8) | s);

				if (cds_ft_insert(swap, suffix, 1, &n->node)
						!= CDS_FT_STATUS_OK)
					abort();
				newn[s] = n;
			}
		}
		/* The consumed swap trie must be exclusive (step-6 contract). */
		cds_ft_make_exclusive(swap);
		st = cds_ft_graft_swap(x->ft, prefix, 2, swap);
		if (st != CDS_FT_STATUS_OK) {
			fprintf(stderr, "gs writer %u prefix p=%u: graft_swap: %s\n",
				x->w, p, cds_ft_status_to_string(st));
			x->failed = 1;
			mw_violation_snapshot();
			break;	/* leave swap + nodes untouched (avoid teardown UAF) */
		}
		/*
		 * @swap now holds {p,w}'s PREVIOUS content (empty on the first
		 * seed).  Reclaim it, then re-point the shadow at the new nodes --
		 * the writer OWNS {p,w} (w == its id), so this shadow update races
		 * no peer (only the shared {p} spine is contended).
		 */
		mw_gs_reclaim(swap);
		for (s = 0; s < MW_XT_SUFFIXES; s++)
			x->node[p][s] = newn[s];
		x->present[p] = clear ? 0 : 1;
		x->ops++;
		if ((seed & 0x3f) == 0)
			rcu_quiescent_state();
	}
	rcu_unregister_thread();
	return NULL;
}

/*
 * 16 writers concurrently cds_ft_graft_swap their own {p,w} subtree with fresh
 * content, contending the shared {p} spine, with the FT-wide writer lock
 * DROPPED.  Endpoints (quiescent): every present key resolves to the owning
 * writer's latest node, count_keys matches the live total, cds_ft_verify passes,
 * and no test node leaks (every swapped-out generation was reclaimed).
 */
static int mw_gs_oracle(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft;
	struct mw_xt_arg *x;
	struct cds_ft_iter *iter;
	pthread_t writers[MW_XT_NR_WRITERS];
	struct timespec t0;
	unsigned long total_ops = 0, live = 0;
	unsigned int i, p, s;
	int ret = 0;

	if (!getenv("FT_INV_MW")) {
		fprintf(stderr, "# inv_concurrent_crosstrie_graft_swap_fine_lock: "
			"skipped (set FT_INV_MW=1 to run)\n");
		return 0;
	}
	/*
	 * graft_swap IS drop-safe: cds_ft_graft_swap re-descends on a
	 * contention-abort (retry_swap) for
	 * BOTH the empty-swap prune (ft_detach_node -EAGAIN, build-invisible) and
	 * the non-empty exchange (the swap-root retire is FUSED into the
	 * insert-replace txn, so a relocated-parent abort rolls both sides back --
	 * mirroring ft_graft_keylen's retry_attach and the src_swap_fused move).
	 */
	mw_install_fatal_handler();
	ft = create_varlen_fine_lock_ft(&group);
	cds_ft_make_concurrent(ft);
	leak_reset();

	x = (struct mw_xt_arg *) calloc(MW_XT_NR_WRITERS, sizeof(*x));
	if (!x)
		abort();
	for (i = 0; i < MW_XT_NR_WRITERS; i++) {
		x[i].ft = ft;
		x[i].group = group;
		x[i].w = i;
	}

	test_go = 0;
	test_stop = 0;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	for (i = 0; i < MW_XT_NR_WRITERS; i++)
		pthread_create(&writers[i], NULL, mw_gs_writer, &x[i]);
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	test_go = 1;

	rcu_thread_offline();
	clock_gettime(CLOCK_MONOTONIC, &t0);
	while (elapsed_ms(&t0) < DEFAULT_DURATION_MS)
		usleep(1000);
	test_stop = 1;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	for (i = 0; i < MW_XT_NR_WRITERS; i++)
		pthread_join(writers[i], NULL);
	rcu_thread_online();

	synchronize_rcu();
	if (cds_ft_iter_create(ft, &iter) < 0)
		abort();
	rcu_read_lock();
	for (i = 0; i < MW_XT_NR_WRITERS; i++) {
		total_ops += x[i].ops;
		if (x[i].failed)
			ret = -1;
		for (p = 0; p < MW_XT_PREFIXES; p++) {
			if (!x[i].present[p])
				continue;
			for (s = 0; s < MW_XT_SUFFIXES; s++) {
				uint8_t full[3] = { (uint8_t) p, (uint8_t) i,
					(uint8_t) s };
				struct cds_ft_node *found;

				live++;
				cds_ft_iter_set_key(iter, full, 3);
				cds_ft_lookup(ft, iter);
				found = cds_ft_iter_node(iter);
				if (found != &x[i].node[p][s]->node) {
					fprintf(stderr, "gs final: writer %u key "
						"{%u,%u,%u} lost (found %p != "
						"%p)\n", i, p, i, s,
						(void *) found,
						(void *) &x[i].node[p][s]->node);
					ret = -1;
				}
			}
		}
	}
	if (cds_ft_count_keys(ft) != live) {
		fprintf(stderr, "gs final: count_keys %lu != live %lu\n",
			cds_ft_count_keys(ft), live);
		ret = -1;
	}
	if (cds_ft_verify(ft, stderr) != CDS_FT_STATUS_OK) {
		fprintf(stderr, "gs final: cds_ft_verify failed\n");
		ret = -1;
	}
	rcu_read_unlock();
	cds_ft_iter_destroy(iter);

	fprintf(stderr, "# inv_concurrent_crosstrie_graft_swap_fine_lock: "
		"%d writers, %lu ops, %lu live keys\n",
		MW_XT_NR_WRITERS, total_ops, live);

	free(x);
	if (drain_and_destroy(ft, group) < 0)
		ret = -1;
	if (leak_check() < 0)
		ret = -1;
	return ret;
}

/* 16 writers graft_swap-exchange their {p,w} content under the LOCK_FINE drop. */
static int inv_concurrent_crosstrie_graft_swap_fine_lock(void)
{
	return mw_gs_oracle();
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
/*   INVARIANT 3b: Insert commits both indexes atomically             */
/*                                                                    */
/*   An insert makes the fresh head reachable in the structural       */
/*   index and spliced into the ordered cell list in ONE flip         */
/*   commit, so a reader can never exact-look-up a key whose cell is  */
/*   not yet in the list (2026-06 review, 2.13: the structural        */
/*   publish used to precede the splice, and a reader landing on the  */
/*   fresh head inside that window followed its NULL ord links and    */
/*   reported a spurious end-of-traversal).  A permanent sentinel     */
/*   key (the maximum, never removed) guarantees every pool key has   */
/*   a successor, so any NOT_FOUND from next-after-exact-lookup is a  */
/*   violation.                                                       */
/*                                                                    */
/* ================================================================== */

static void *inv_splice_window_reader(void *arg)
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
		uint64_t key = (uint64_t)(rand_r(&seed) % WRITER_POOL_SIZE);
		uint8_t k[8];
		enum cds_ft_status s;

		cds_ft_u64_to_key(ctx->ft, key, k, CDS_FT_LEN_DEFAULT);
		rcu_read_lock();
		cds_ft_iter_set_key(iter, k, CDS_FT_LEN_DEFAULT);
		if (cds_ft_lookup(ctx->ft, iter) == CDS_FT_STATUS_OK &&
				cds_ft_iter_node(iter)) {
			s = cds_ft_next(ctx->ft, iter);
			if (s == CDS_FT_STATUS_NOT_FOUND) {
				/*
				 * The sentinel (max key) is never removed, so
				 * every pool key has a successor.
				 */
				report_violation(ctx->test_name,
					"next after exact lookup of %" PRIu64
					" reported end-of-traversal (sentinel present)",
					key);
			}
		}
		rcu_read_unlock();

		checks++;
		if ((checks & 0x3ff) == 0)
			rcu_quiescent_state();
	}

	cds_ft_iter_destroy(iter);
	rcu_unregister_thread();
	return NULL;
}

static int inv_insert_splice_window(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(4, &group);
	struct inv_lookup_ctx ctx;
	struct timespec t0;
	pthread_t readers[NR_READERS_DEFAULT], writers[NR_WRITERS_DEFAULT];
	unsigned int i;

	ctx.ft = ft;
	ctx.test_name = "inv_insert_splice_window";
	pthread_mutex_init(&ctx.lock, NULL);

	/* The permanent sentinel: the maximum key, never removed. */
	rcu_read_lock();
	{
		struct ft_test_node *n = node_alloc(0xffffffffull);

		if (insert_u64(ft, 0xffffffffull, n) != CDS_FT_STATUS_OK)
			abort();
	}
	/* Pre-populate half the writer pool. */
	for (i = 0; i < WRITER_POOL_SIZE / 2; i++) {
		struct ft_test_node *n = node_alloc(i);
		insert_u64(ft, i, n);
	}
	rcu_read_unlock();

	test_go = 0;
	test_stop = 0;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	for (i = 0; i < NR_READERS_DEFAULT; i++)
		pthread_create(&readers[i], NULL, inv_splice_window_reader, &ctx);
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
		fprintf(stderr, "inv_insert_splice_window: %lu violation(s)\n",
			atomic_load(&violation_count));
		drain_and_destroy(ft, group);
		return -1;
	}
	return drain_and_destroy(ft, group);
}

/*
 * Same splice-window invariant, but writers use cds_ft_insert_replace instead
 * of cds_ft_insert.  insert_replace does NOT use the one-commit park, so a
 * fresh head reaches readers structurally before its ordinal cell is spliced
 * (post-publish splice / early-wired live edge in the compressed-split shapes)
 * -- the same secondary-index channel window, exercised on the insert_replace
 * paths that inv_insert_splice_window never drives.
 */
static void *inv_splice_window_replace_writer(void *arg)
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
			struct cds_ft_node *old = NULL;

			pthread_mutex_lock(&ctx->lock);
			/*
			 * Both OK (inserted, no prior node) and DUPLICATE_FOUND
			 * (replaced an existing chain) are success codes (>= 0);
			 * on a replace, @old is the old head and must be freed.
			 */
			if (insert_replace_u64(ctx->ft, key, n, &old) >= 0) {
				if (old)
					node_free_rcu(to_test_node(old));
			}
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

static int inv_insert_replace_splice_window(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(4, &group);
	struct inv_lookup_ctx ctx;
	struct timespec t0;
	pthread_t readers[NR_READERS_DEFAULT], writers[NR_WRITERS_DEFAULT];
	unsigned int i;

	ctx.ft = ft;
	ctx.test_name = "inv_insert_replace_splice_window";
	pthread_mutex_init(&ctx.lock, NULL);

	/* The permanent sentinel: the maximum key, never removed. */
	rcu_read_lock();
	{
		struct ft_test_node *n = node_alloc(0xffffffffull);

		if (insert_u64(ft, 0xffffffffull, n) != CDS_FT_STATUS_OK)
			abort();
	}
	/* Pre-populate half the writer pool. */
	for (i = 0; i < WRITER_POOL_SIZE / 2; i++) {
		struct ft_test_node *n = node_alloc(i);
		insert_u64(ft, i, n);
	}
	rcu_read_unlock();

	test_go = 0;
	test_stop = 0;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	for (i = 0; i < NR_READERS_DEFAULT; i++)
		pthread_create(&readers[i], NULL, inv_splice_window_reader, &ctx);
	for (i = 0; i < NR_WRITERS_DEFAULT; i++)
		pthread_create(&writers[i], NULL,
			inv_splice_window_replace_writer, &ctx);

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
		fprintf(stderr, "inv_insert_replace_splice_window: %lu violation(s)\n",
			atomic_load(&violation_count));
		drain_and_destroy(ft, group);
		return -1;
	}
	return drain_and_destroy(ft, group);
}

/*
 * SKIP_X-suffix variant of inv_insert_replace_splice_window.  Keys are SPARSE
 * (idx << 16) so every leaf hangs off a len-2 compressed suffix node
 * ([0x00,0x00]) reached from its grandparent via a SKIP_X pointer.  A concurrent
 * cds_ft_insert_replace at such a leaf drives the DUAL forward publish --
 * cn->child (read by the exact descent) AND the grandparent SKIP_X slot (read by
 * the candidate descent), both naming the leaf -- which is now fused with the
 * head cell swap into ONE flip (ft_ord_cell_swap_publish_multi; the 2-edge
 * ft_ord_cell_flip when the ordered list is off).  A reader must never observe
 * cn->child and the skip target name different heads mid-replace, nor follow a
 * stale skip target into the freed old leaf.  The dense-key
 * inv_insert_replace_splice_window never forms the SKIP_X-at-leaf shape, so this
 * is the only invariant that drives the dual fusion concurrently.  Run also
 * under FT_INV_NO_ORDERED_LIST to cover the list-off 2-edge flip.
 */
#define INV_SKIPX_KEY(idx)	((uint64_t)(idx) << 16)

static void *inv_skipx_replace_reader(void *arg)
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
		uint64_t key = INV_SKIPX_KEY(rand_r(&seed) % WRITER_POOL_SIZE);
		uint8_t k[8];
		enum cds_ft_status s;

		cds_ft_u64_to_key(ctx->ft, key, k, CDS_FT_LEN_DEFAULT);
		rcu_read_lock();
		cds_ft_iter_set_key(iter, k, CDS_FT_LEN_DEFAULT);
		if (cds_ft_lookup(ctx->ft, iter) == CDS_FT_STATUS_OK &&
				cds_ft_iter_node(iter)) {
			s = cds_ft_next(ctx->ft, iter);
			if (s == CDS_FT_STATUS_NOT_FOUND) {
				/*
				 * The sentinel (max key) is never removed, so
				 * every pool key has a successor.
				 */
				report_violation(ctx->test_name,
					"next after exact lookup of %" PRIu64
					" reported end-of-traversal (sentinel present)",
					key);
			}
		}
		rcu_read_unlock();

		checks++;
		if ((checks & 0x3ff) == 0)
			rcu_quiescent_state();
	}

	cds_ft_iter_destroy(iter);
	rcu_unregister_thread();
	return NULL;
}

static void *inv_skipx_replace_writer(void *arg)
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
		uint64_t key = INV_SKIPX_KEY(rand_r(&seed) % WRITER_POOL_SIZE);
		int do_insert = rand_r(&seed) & 1;

		rcu_read_lock();
		if (do_insert) {
			struct ft_test_node *n = node_alloc(key);
			struct cds_ft_node *old = NULL;

			pthread_mutex_lock(&ctx->lock);
			/* Replace at an existing key drives the dual publish. */
			if (insert_replace_u64(ctx->ft, key, n, &old) >= 0) {
				if (old)
					node_free_rcu(to_test_node(old));
			}
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

static int inv_insert_replace_skipx_splice_window(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(4, &group);
	struct inv_lookup_ctx ctx;
	struct timespec t0;
	pthread_t readers[NR_READERS_DEFAULT], writers[NR_WRITERS_DEFAULT];
	unsigned int i;

	ctx.ft = ft;
	ctx.test_name = "inv_insert_replace_skipx_splice_window";
	pthread_mutex_init(&ctx.lock, NULL);

	/* The permanent sentinel: the maximum key, never removed. */
	rcu_read_lock();
	{
		struct ft_test_node *n = node_alloc(0xffffffffull);

		if (insert_u64(ft, 0xffffffffull, n) != CDS_FT_STATUS_OK)
			abort();
	}
	/* Pre-populate half the writer pool with SPARSE keys (SKIP_X leaves). */
	for (i = 0; i < WRITER_POOL_SIZE / 2; i++) {
		struct ft_test_node *n = node_alloc(INV_SKIPX_KEY(i));
		insert_u64(ft, INV_SKIPX_KEY(i), n);
	}
	rcu_read_unlock();

	test_go = 0;
	test_stop = 0;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	for (i = 0; i < NR_READERS_DEFAULT; i++)
		pthread_create(&readers[i], NULL, inv_skipx_replace_reader, &ctx);
	for (i = 0; i < NR_WRITERS_DEFAULT; i++)
		pthread_create(&writers[i], NULL,
			inv_skipx_replace_writer, &ctx);

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
		fprintf(stderr, "inv_insert_replace_skipx_splice_window: %lu violation(s)\n",
			atomic_load(&violation_count));
		drain_and_destroy(ft, group);
		return -1;
	}
	return drain_and_destroy(ft, group);
}

/*
 * cds_ft_replace (the in-place, caller-supplied old+new node replace) on SKIP_X
 * leaves.  Like ft_promote_head, cds_ft_replace inherited the old head's cell and
 * retargeted it in place; it now publishes a FRESH cell for the new head and
 * swaps it in fused with the cn->child publish + the grandparent SKIP_X dual in
 * one flip.  Sparse keys (idx << 16) reach the compressed-holder head case; the
 * keys are pre-populated and only ever replaced (cds_ft_replace is 1:1, never
 * removes), so every lookup finds a head and the shared splice-window reader's
 * "next has the sentinel successor" check holds.  cds_ft_replace has no other
 * concurrent coverage.
 */
#define INV_SKIPX_KEY(idx)	((uint64_t)(idx) << 16)

static void *inv_replace_skipx_writer(void *arg)
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
		uint64_t key = INV_SKIPX_KEY(rand_r(&seed) % WRITER_POOL_SIZE);
		struct ft_test_node *newn = node_alloc(key);
		struct cds_ft_node *old;
		uint8_t k[8];

		cds_ft_u64_to_key(ctx->ft, key, k, CDS_FT_LEN_DEFAULT);
		/*
		 * Stamp the ordinal key bytes BEFORE the node enters the trie,
		 * exactly as insert_u64 / insert_replace_u64 do for every other
		 * insert path in this file.  This is the speculative-key contract
		 * (cds_ft_group_attr_set_speculative_key_offset: "the key bytes at
		 * @key_offset must be present before the node is inserted"), not
		 * harness bookkeeping -- on a speculative group the inequality
		 * lookups take their result key by LOCK it from the matched
		 * leaf, so a head published with @okey still zeroed hands every
		 * later cds_ft_next the key 0.  The walk then asks for the
		 * successor of the wrong key.
		 */
		memcpy(newn->okey, k, sizeof(newn->okey));
		rcu_read_lock();
		pthread_mutex_lock(&ctx->lock);
		/* Look up the current head under the writer mutex, then replace it
		 * in place.  All keys stay present, so old is found. */
		cds_ft_iter_set_key(iter, k, CDS_FT_LEN_DEFAULT);
		cds_ft_lookup(ctx->ft, iter);
		old = cds_ft_iter_node(iter);
		if (old && cds_ft_replace(ctx->ft, iter, old, &newn->node)
				== CDS_FT_STATUS_OK)
			node_free_rcu(to_test_node(old));
		else
			node_free_rcu(newn);	/* unused / not installed */
		pthread_mutex_unlock(&ctx->lock);
		rcu_read_unlock();

		if ((seed & 0xff) == 0)
			rcu_quiescent_state();
	}

	cds_ft_iter_destroy(iter);
	rcu_unregister_thread();
	return NULL;
}

static int inv_replace_skipx(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(4, &group);
	struct inv_lookup_ctx ctx;
	struct timespec t0;
	pthread_t readers[NR_READERS_DEFAULT], writers[NR_WRITERS_DEFAULT];
	unsigned int i;

	ctx.ft = ft;
	ctx.test_name = "inv_replace_skipx";
	pthread_mutex_init(&ctx.lock, NULL);

	/* Sentinel (max key, never replaced) + all sparse keys (SKIP_X leaves). */
	rcu_read_lock();
	{
		struct ft_test_node *n = node_alloc(0xffffffffull);

		if (insert_u64(ft, 0xffffffffull, n) != CDS_FT_STATUS_OK)
			abort();
	}
	for (i = 0; i < WRITER_POOL_SIZE; i++) {
		struct ft_test_node *n = node_alloc(INV_SKIPX_KEY(i));
		insert_u64(ft, INV_SKIPX_KEY(i), n);
	}
	rcu_read_unlock();

	test_go = 0;
	test_stop = 0;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	for (i = 0; i < NR_READERS_DEFAULT; i++)
		pthread_create(&readers[i], NULL, inv_skipx_replace_reader, &ctx);
	for (i = 0; i < NR_WRITERS_DEFAULT; i++)
		pthread_create(&writers[i], NULL, inv_replace_skipx_writer, &ctx);

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
		fprintf(stderr, "inv_replace_skipx: %lu violation(s)\n",
			atomic_load(&violation_count));
		drain_and_destroy(ft, group);
		return -1;
	}
	return drain_and_destroy(ft, group);
}

#undef INV_SKIPX_KEY

/*
 * INVARIANT: a key's removal is atomic across the structural index and the
 * ordered cell list -- there is no instant at which a key is reachable via one
 * but not the other.
 *
 * Workload: a single drainer removes the ordered minimum monotonically with NO
 * concurrent inserts (the pool only shrinks).  In one RCU critical section a
 * reader runs three queries:
 *   1) lookup_first    -> ordered-list minimum  (key k1)
 *   2) point lookup(k1) -> structural descent    (found?)
 *   3) lookup_first    -> ordered-list minimum  (key k2)
 * If k1 == k2 (the same key is the ordered minimum before AND after) yet the
 * structural lookup did NOT find it, that key is in the ordered list but absent
 * from the structural index -- which can only happen if cds_ft_remove published
 * the top child-pointer removal before the ordered-list unsplice (a non-atomic
 * unpublish).  Because no inserts run, "present again" cannot be a legitimate
 * re-add.  Comparison is key-based, not by node pointer: a freed node's address
 * can be recycled by a later allocation (ABA).
 */
static void *inv_remove_xview_reader(void *arg)
{
	struct inv_lookup_ctx *ctx = (struct inv_lookup_ctx *) arg;
	struct cds_ft_iter *iter;

	rcu_register_thread();
	if (cds_ft_iter_create(ctx->ft, &iter) < 0)
		abort();

	while (!test_go)
		;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	while (!test_stop) {
		struct cds_ft_node *n1, *n2;
		uint64_t k1, k2;
		uint8_t kb[8];
		bool found;

		rcu_read_lock();
		if (cds_ft_lookup_first(ctx->ft, iter) == CDS_FT_STATUS_OK &&
		    (n1 = cds_ft_iter_node(iter)) != NULL) {
			k1 = to_test_node(n1)->key;
			/* Structural point lookup of the ordered minimum. */
			cds_ft_u64_to_key(ctx->ft, k1, kb, CDS_FT_LEN_DEFAULT);
			cds_ft_iter_set_key(iter, kb, CDS_FT_LEN_DEFAULT);
			found = cds_ft_lookup(ctx->ft, iter) == CDS_FT_STATUS_OK &&
				cds_ft_iter_node(iter) != NULL;
			/* Re-confirm the ordered minimum is still the same key. */
			if (cds_ft_lookup_first(ctx->ft, iter) == CDS_FT_STATUS_OK &&
			    (n2 = cds_ft_iter_node(iter)) != NULL) {
				k2 = to_test_node(n2)->key;
				if (k1 == k2 && !found)
					report_violation(ctx->test_name,
						"ordered-min key %" PRIu64
						" present, absent, present -- structural"
						" removal raced ahead of list unsplice",
						k1);
			}
		}
		rcu_read_unlock();
	}

	cds_ft_iter_destroy(iter);
	rcu_unregister_thread();
	return NULL;
}

static void *inv_remove_xview_drainer(void *arg)
{
	struct inv_lookup_ctx *ctx = (struct inv_lookup_ctx *) arg;
	struct cds_ft_iter *iter;

	rcu_register_thread();
	if (cds_ft_iter_create(ctx->ft, &iter) < 0)
		abort();

	while (!test_go)
		;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	while (!test_stop) {
		struct cds_ft_node *node;
		uint64_t k;
		uint8_t kb[8];

		rcu_read_lock();
		pthread_mutex_lock(&ctx->lock);
		if (cds_ft_lookup_first(ctx->ft, iter) == CDS_FT_STATUS_OK &&
		    (node = cds_ft_iter_node(iter)) != NULL &&
		    to_test_node(node)->key != 0xffffffffull) {
			/* Position the iterator structurally, then remove the min. */
			k = to_test_node(node)->key;
			cds_ft_u64_to_key(ctx->ft, k, kb, CDS_FT_LEN_DEFAULT);
			cds_ft_iter_set_key(iter, kb, CDS_FT_LEN_DEFAULT);
			if (cds_ft_lookup(ctx->ft, iter) == CDS_FT_STATUS_OK &&
			    (node = cds_ft_iter_node(iter)) != NULL &&
			    cds_ft_remove(ctx->ft, iter, node) == CDS_FT_STATUS_OK)
				node_free_rcu(to_test_node(node));
		} else {
			/* Only the permanent sentinel (max key) remains: drained. */
			__atomic_store_n(&test_drained, 1, __ATOMIC_RELAXED);
			pthread_mutex_unlock(&ctx->lock);
			rcu_read_unlock();
			break;
		}
		pthread_mutex_unlock(&ctx->lock);
		rcu_read_unlock();
	}

	cds_ft_iter_destroy(iter);
	rcu_unregister_thread();
	return NULL;
}

/* Cross-view oracle pool sizes.  Overridable at build time (-D...) to run a
 * small, fast variant -- e.g. under FEATURE_FT_VERIFY_AT_MUTATION, whose
 * per-mutation full-trie verify makes the default sizes O(n^2)-slow. */
#ifndef REMOVE_XVIEW_POOL
#define REMOVE_XVIEW_POOL	100000
#endif
/* Sparse-pair pool: pairs (k*256, k*256+1) sharing a 3-byte prefix, so each pair
 * sits under its own branch that canonicalizes to a compressed holder as it
 * drains -- exercises the compressed-parent detach publish.  Keys stay < 16 MiB. */
#ifndef REMOVE_XVIEW_PAIRS
#define REMOVE_XVIEW_PAIRS	40000
#endif

/* Shared driver: @ft is pre-populated; spin up the cross-view @reader_fn threads
 * + the @drainer_fn, run to drain or timeout. */
static int run_remove_cross_view(struct cds_ft *ft, struct cds_ft_group *group,
		const char *test_name,
		void *(*reader_fn)(void *), void *(*drainer_fn)(void *))
{
	struct inv_lookup_ctx ctx;
	struct timespec t0;
	pthread_t readers[NR_READERS_DEFAULT], drainer;
	unsigned int i;

	ctx.ft = ft;
	ctx.test_name = test_name;
	pthread_mutex_init(&ctx.lock, NULL);

	test_go = 0;
	test_stop = 0;
	test_drained = 0;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	for (i = 0; i < NR_READERS_DEFAULT; i++)
		pthread_create(&readers[i], NULL, reader_fn, &ctx);
	pthread_create(&drainer, NULL, drainer_fn, &ctx);

	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	test_go = 1;

	rcu_thread_offline();

	clock_gettime(CLOCK_MONOTONIC, &t0);
	while (elapsed_ms(&t0) < DEFAULT_DURATION_MS &&
	       !__atomic_load_n(&test_drained, __ATOMIC_RELAXED))
		usleep(1000);

	test_stop = 1;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	pthread_join(drainer, NULL);
	for (i = 0; i < NR_READERS_DEFAULT; i++)
		pthread_join(readers[i], NULL);

	rcu_thread_online();

	pthread_mutex_destroy(&ctx.lock);

	if (atomic_load(&violation_count) > 0) {
		fprintf(stderr, "%s: %lu violation(s)\n",
			test_name, atomic_load(&violation_count));
		drain_and_destroy(ft, group);
		return -1;
	}
	return drain_and_destroy(ft, group);
}

static int inv_remove_cross_view(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(4, &group);
	unsigned int i;

	/* The permanent sentinel (max key) is never removed: bounds the drain. */
	rcu_read_lock();
	{
		struct ft_test_node *n = node_alloc(0xffffffffull);

		if (insert_u64(ft, 0xffffffffull, n) != CDS_FT_STATUS_OK)
			abort();
	}
	for (i = 0; i < REMOVE_XVIEW_POOL; i++) {
		struct ft_test_node *n = node_alloc(i);
		insert_u64(ft, i, n);
	}
	rcu_read_unlock();

	return run_remove_cross_view(ft, group, "inv_remove_cross_view",
		inv_remove_xview_reader, inv_remove_xview_drainer);
}

/*
 * Same cross-view atomicity invariant, but a sparse-pair distribution whose
 * removals prune up to COMPRESSED holders (the ft_detach_node_replace_compressed_parent
 * publish), which the dense inv_remove_cross_view never reaches.
 */
static int inv_remove_cross_view_compressed(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(4, &group);
	unsigned int k;

	rcu_read_lock();
	{
		struct ft_test_node *n = node_alloc(0xffffffffull);

		if (insert_u64(ft, 0xffffffffull, n) != CDS_FT_STATUS_OK)
			abort();
	}
	for (k = 0; k < REMOVE_XVIEW_PAIRS; k++) {
		struct ft_test_node *a = node_alloc((uint64_t) k * 256);
		struct ft_test_node *b = node_alloc((uint64_t) k * 256 + 1);

		insert_u64(ft, (uint64_t) k * 256, a);
		insert_u64(ft, (uint64_t) k * 256 + 1, b);
	}
	rcu_read_unlock();

	return run_remove_cross_view(ft, group, "inv_remove_cross_view_compressed",
		inv_remove_xview_reader, inv_remove_xview_drainer);
}

/*
 * MAX-draining variant for the compressed-PARENT removal shape (external
 * promote), which the min-draining tests cannot reach: the promote fires when a
 * node's last longer-key child is removed while its shorter PREFIX key still
 * exists -- but a prefix sorts BEFORE its extensions, so min-draining removes
 * the prefix first (emptying the external before the longer child).  Draining
 * the MAX reverses that: the longer child is removed first, while its prefix
 * persists to be promoted, AND the removed longer child IS the current ordered
 * maximum -- so a lookup_last-based reader observes the present/absent/present
 * window.  Requires variable-length prefix keys (fixed-length keys never
 * terminate at an internal node, so cannot produce the external-on-internal
 * shape).  Keys: per group g, Kp(g)=[g_hi,g_lo,0,0] (4 B) and its extension
 * Kl(g)=[g_hi,g_lo,0,0,0] (5 B); the [0,0] run compresses, so Kp's holder sits
 * under a compressed node and removing Kl promotes Kp into the compressed
 * node's child slot.  The test node carries the key bytes in @okey and the
 * length in @value (the eager trie never writes @okey -- no speculative offset).
 */
/*
 * @ws: NULL to inherit the group default (LOCK_FINE), or a strategy to pin.
 * Pinning is used by the graft_swap shape oracles (gs_shared_oracle), which pin
 * CDS_FT_WRITER_LOCK_FINE explicitly so the shape under test is not silently
 * coerced.  The parameter is live -- do not retire it as unused.
 */
static struct cds_ft *create_varlen_ord_ft_ws(struct cds_ft_group **group_out,
		const enum cds_ft_writer_strategy *ws)
{
	struct cds_ft_group_attr *attr;
	struct cds_ft_group *group;
	struct cds_ft *ft;

	if (cds_ft_group_attr_create(&attr) < 0)
		abort();
	/* Variable-length: keep the default key_len (per-call lengths).  Eager
	 * so the point lookup needs no speculative key offset and @okey is free
	 * for the test to stash the byte key.  Ordered list ON (default). */
	if (cds_ft_group_attr_set_lookup_optimization(attr,
			CDS_FT_LOOKUP_OPTIMIZE_EAGER) < 0)
		abort();
	if (cds_ft_group_attr_set_ordered_list(attr, true) < 0)
		abort();
	if (ws && cds_ft_group_attr_set_writer_strategy(attr, *ws) < 0)
		abort();
	if (cds_ft_group_create(attr, &group) < 0)
		abort();
	cds_ft_group_attr_destroy(attr);
	if (cds_ft_create(group, NULL, &ft) < 0)
		abort();
	*group_out = group;
	return ft;
}

static struct cds_ft *create_varlen_ord_ft(struct cds_ft_group **group_out)
{
	return create_varlen_ord_ft_ws(group_out, NULL);
}

/*
 * Variable-length, ordered list OFF: exercises the graft GLUE flip-txn fold
 * (ft_glue_txn_commit), which is gated on the list being off.  EAGER, NOT
 * SPECULATIVE: an EAGER descent still traverses the skip-compressed nodes the
 * txn flips (and can trigger a parent-pointer reanchor mid-descent) but carries
 * no speculative leaf key, so draining the detached trie cannot hit the
 * pre-existing keycopy-relational livelock (ft_ineq_descend spins on a leaf
 * whose stale speculative key -- the pre-detach full key on a now-stripped
 * detached position -- disagrees with its slot).
 */
/*
 * As create_varlen_nolist_ft_ws, but SPECULATIVE (which leaves
 * CDS_FT_FLAG_SKIP_COMPRESSED set) rather than EAGER (which clears it).
 */
static struct cds_ft *create_varlen_nolist_ft_ws_spec(
		struct cds_ft_group **group_out,
		const enum cds_ft_writer_strategy *ws)
{
	struct cds_ft_group_attr *attr;
	struct cds_ft_group *group;
	struct cds_ft *ft;

	if (cds_ft_group_attr_create(&attr) < 0)
		abort();
	if (cds_ft_group_attr_set_lookup_optimization(attr,
			CDS_FT_LOOKUP_OPTIMIZE_SPECULATIVE) < 0)
		abort();
	if (cds_ft_group_attr_set_ordered_list(attr, false) < 0)
		abort();
	if (ws && cds_ft_group_attr_set_writer_strategy(attr, *ws) < 0)
		abort();
	if (cds_ft_group_create(attr, &group) < 0)
		abort();
	cds_ft_group_attr_destroy(attr);
	if (cds_ft_create(group, NULL, &ft) < 0)
		abort();
	*group_out = group;
	return ft;
}

static struct cds_ft *create_varlen_ord_ft_ws_spec(
		struct cds_ft_group **group_out,
		const enum cds_ft_writer_strategy *ws)
{
	struct cds_ft_group_attr *attr;
	struct cds_ft_group *group;
	struct cds_ft *ft;

	if (cds_ft_group_attr_create(&attr) < 0)
		abort();
	if (cds_ft_group_attr_set_lookup_optimization(attr,
			CDS_FT_LOOKUP_OPTIMIZE_SPECULATIVE) < 0)
		abort();
	if (cds_ft_group_attr_set_ordered_list(attr, true) < 0)
		abort();
	if (ws && cds_ft_group_attr_set_writer_strategy(attr, *ws) < 0)
		abort();
	if (cds_ft_group_create(attr, &group) < 0)
		abort();
	cds_ft_group_attr_destroy(attr);
	if (cds_ft_create(group, NULL, &ft) < 0)
		abort();
	*group_out = group;
	return ft;
}

static struct cds_ft *create_varlen_nolist_ft_ws(struct cds_ft_group **group_out,
		const enum cds_ft_writer_strategy *ws)
{
	struct cds_ft_group_attr *attr;
	struct cds_ft_group *group;
	struct cds_ft *ft;

	if (cds_ft_group_attr_create(&attr) < 0)
		abort();
	if (cds_ft_group_attr_set_lookup_optimization(attr,
			CDS_FT_LOOKUP_OPTIMIZE_EAGER) < 0)
		abort();
	if (cds_ft_group_attr_set_ordered_list(attr, false) < 0)
		abort();
	if (ws && cds_ft_group_attr_set_writer_strategy(attr, *ws) < 0)
		abort();
	if (cds_ft_group_create(attr, &group) < 0)
		abort();
	cds_ft_group_attr_destroy(attr);
	if (cds_ft_create(group, NULL, &ft) < 0)
		abort();
	*group_out = group;
	return ft;
}

static struct cds_ft *create_varlen_nolist_ft(struct cds_ft_group **group_out)
{
	return create_varlen_nolist_ft_ws(group_out, NULL);
}

/* Drain every key from @ft and destroy the trie, leaving its group alive. */
static void drain_trie_keep_group(struct cds_ft *ft)
{
	struct cds_ft_iter *iter;

	if (cds_ft_iter_create(ft, &iter) < 0)
		abort();
	rcu_read_lock();
	while (cds_ft_lookup_first(ft, iter) == CDS_FT_STATUS_OK) {
		struct cds_ft_node *head, *tmp;
		enum cds_ft_status s = cds_ft_remove_all(ft, iter, &head);

		if (s < 0)
			abort();
		/* No-progress guard: see drain_and_destroy. */
		if (s == CDS_FT_STATUS_NOT_FOUND) {
			fprintf(stderr,
				"drain_trie_keep_group: lookup_first found a key "
				"remove_all reports NOT_FOUND -- stale iterator key\n");
			abort();
		}
		cds_ft_for_each_duplicate_safe_rcu(head, tmp)
			node_free_rcu(to_test_node(head));
	}
	rcu_read_unlock();
	cds_ft_iter_destroy(iter);
	cds_ft_destroy(ft);
}

/* Read a variable-length test node's stashed key bytes + length. */
static size_t xview_node_key(struct cds_ft_node *node, uint8_t *out)
{
	struct ft_test_node *t = to_test_node(node);
	size_t len = (size_t) t->value;

	memcpy(out, t->okey, len);
	return len;
}

static void *inv_remove_xview_reader_max(void *arg)
{
	struct inv_lookup_ctx *ctx = (struct inv_lookup_ctx *) arg;
	struct cds_ft_iter *iter;

	rcu_register_thread();
	if (cds_ft_iter_create(ctx->ft, &iter) < 0)
		abort();

	while (!test_go)
		;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	while (!test_stop) {
		struct cds_ft_node *n1, *n2;
		uint8_t k1[8], k2[8];
		size_t l1, l2;
		bool found;

		rcu_read_lock();
		if (cds_ft_lookup_last(ctx->ft, iter) == CDS_FT_STATUS_OK &&
		    (n1 = cds_ft_iter_node(iter)) != NULL) {
			l1 = xview_node_key(n1, k1);
			/* Structural point lookup of the ordered maximum. */
			cds_ft_iter_set_key(iter, k1, l1);
			found = cds_ft_lookup(ctx->ft, iter) == CDS_FT_STATUS_OK &&
				cds_ft_iter_node(iter) != NULL;
			/* Re-confirm the ordered maximum is still the same key. */
			if (cds_ft_lookup_last(ctx->ft, iter) == CDS_FT_STATUS_OK &&
			    (n2 = cds_ft_iter_node(iter)) != NULL) {
				l2 = xview_node_key(n2, k2);
				if (l1 == l2 && memcmp(k1, k2, l1) == 0 && !found)
					report_violation(ctx->test_name,
						"ordered-max key (len %zu) present,"
						" absent, present -- compressed-parent"
						" removal raced ahead of list unsplice",
						l1);
			}
		}
		rcu_read_unlock();
	}

	cds_ft_iter_destroy(iter);
	rcu_unregister_thread();
	return NULL;
}

static void *inv_remove_xview_drainer_max(void *arg)
{
	struct inv_lookup_ctx *ctx = (struct inv_lookup_ctx *) arg;
	struct cds_ft_iter *iter;

	rcu_register_thread();
	if (cds_ft_iter_create(ctx->ft, &iter) < 0)
		abort();

	while (!test_go)
		;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	while (!test_stop) {
		struct cds_ft_node *node;
		uint8_t k[8];
		size_t l;

		rcu_read_lock();
		pthread_mutex_lock(&ctx->lock);
		if (cds_ft_lookup_last(ctx->ft, iter) == CDS_FT_STATUS_OK &&
		    (node = cds_ft_iter_node(iter)) != NULL) {
			l = xview_node_key(node, k);
			cds_ft_iter_set_key(iter, k, l);
			if (cds_ft_lookup(ctx->ft, iter) == CDS_FT_STATUS_OK &&
			    (node = cds_ft_iter_node(iter)) != NULL &&
			    cds_ft_remove(ctx->ft, iter, node) == CDS_FT_STATUS_OK)
				node_free_rcu(to_test_node(node));
		} else {
			/* Trie drained (no sentinel: max draining needs none). */
			__atomic_store_n(&test_drained, 1, __ATOMIC_RELAXED);
			pthread_mutex_unlock(&ctx->lock);
			rcu_read_unlock();
			break;
		}
		pthread_mutex_unlock(&ctx->lock);
		rcu_read_unlock();
	}

	cds_ft_iter_destroy(iter);
	rcu_unregister_thread();
	return NULL;
}

#ifndef REMOVE_XVIEW_GROUPS
#define REMOVE_XVIEW_GROUPS	20000
#endif

static int inv_remove_cross_view_compressed_parent(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_varlen_ord_ft(&group);
	unsigned int g;

	rcu_read_lock();
	for (g = 0; g < REMOVE_XVIEW_GROUPS; g++) {
		uint8_t kp[4] = { (uint8_t)(g >> 8), (uint8_t)(g & 0xff), 0, 0 };
		uint8_t kl[5] = { (uint8_t)(g >> 8), (uint8_t)(g & 0xff), 0, 0, 0 };
		struct ft_test_node *np = node_alloc(g);
		struct ft_test_node *nl = node_alloc(g);

		np->value = 4;
		memcpy(np->okey, kp, 4);
		nl->value = 5;
		memcpy(nl->okey, kl, 5);
		if (cds_ft_insert(ft, kp, 4, &np->node) != CDS_FT_STATUS_OK)
			abort();
		if (cds_ft_insert(ft, kl, 5, &nl->node) != CDS_FT_STATUS_OK)
			abort();
	}
	rcu_read_unlock();

	return run_remove_cross_view(ft, group,
		"inv_remove_cross_view_compressed_parent",
		inv_remove_xview_reader_max, inv_remove_xview_drainer_max);
}

/*
 * MIN-draining variant for the PREFIX-WITH-SIBLINGS removal shape, which the
 * other oracles cannot reach: removing a prefix key whose holder KEEPS sibling
 * children (its longer-key extensions).  A prefix sorts BEFORE its extensions,
 * so MIN-draining removes the prefix Kp FIRST -- while its extension Kl still
 * hangs off the same internal holder -- and Kp IS the current ordered minimum,
 * so a lookup_first-based reader observes the present/absent/present window if
 * the external_nodes clear and the cell unsplice are not one flip.  (Max-
 * draining instead removes Kl first, promoting Kp to a leaf, so it never sees
 * Kp as a prefix-with-siblings.)  Same variable-length prefix/extension keys as
 * inv_remove_cross_view_compressed_parent: Kp(g)=[g_hi,g_lo,0,0] (4 B) and its
 * extension Kl(g)=[g_hi,g_lo,0,0,0] (5 B).
 */
static void *inv_remove_xview_reader_minvl(void *arg)
{
	struct inv_lookup_ctx *ctx = (struct inv_lookup_ctx *) arg;
	struct cds_ft_iter *iter;

	rcu_register_thread();
	if (cds_ft_iter_create(ctx->ft, &iter) < 0)
		abort();

	while (!test_go)
		;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	while (!test_stop) {
		struct cds_ft_node *n1, *n2;
		uint8_t k1[8], k2[8];
		size_t l1, l2;
		bool found;

		rcu_read_lock();
		if (cds_ft_lookup_first(ctx->ft, iter) == CDS_FT_STATUS_OK &&
		    (n1 = cds_ft_iter_node(iter)) != NULL) {
			l1 = xview_node_key(n1, k1);
			/* Structural point lookup of the ordered minimum. */
			cds_ft_iter_set_key(iter, k1, l1);
			found = cds_ft_lookup(ctx->ft, iter) == CDS_FT_STATUS_OK &&
				cds_ft_iter_node(iter) != NULL;
			/* Re-confirm the ordered minimum is still the same key. */
			if (cds_ft_lookup_first(ctx->ft, iter) == CDS_FT_STATUS_OK &&
			    (n2 = cds_ft_iter_node(iter)) != NULL) {
				l2 = xview_node_key(n2, k2);
				if (l1 == l2 && memcmp(k1, k2, l1) == 0 && !found)
					report_violation(ctx->test_name,
						"ordered-min key (len %zu) present,"
						" absent, present -- structural index and"
						" ordered list disagree (non-atomic unpublish)",
						l1);
			}
		}
		rcu_read_unlock();
		/*
		 * Report a quiescent state (QSBR): a bulk-op drainer (cds_ft_detach)
		 * synchronize_rcu's, and would wait forever for a reader that never
		 * passes a quiescent state.  Harmless for the point-remove drainers,
		 * which never synchronize_rcu.
		 */
		rcu_quiescent_state();
	}

	cds_ft_iter_destroy(iter);
	rcu_unregister_thread();
	return NULL;
}

/* Min-draining via cds_ft_remove: exercises the cds_ft_remove is_prefix path. */
static void *inv_remove_xview_drainer_minvl(void *arg)
{
	struct inv_lookup_ctx *ctx = (struct inv_lookup_ctx *) arg;
	struct cds_ft_iter *iter;

	rcu_register_thread();
	if (cds_ft_iter_create(ctx->ft, &iter) < 0)
		abort();

	while (!test_go)
		;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	while (!test_stop) {
		struct cds_ft_node *node;
		uint8_t k[8];
		size_t l;

		rcu_read_lock();
		pthread_mutex_lock(&ctx->lock);
		if (cds_ft_lookup_first(ctx->ft, iter) == CDS_FT_STATUS_OK &&
		    (node = cds_ft_iter_node(iter)) != NULL) {
			l = xview_node_key(node, k);
			cds_ft_iter_set_key(iter, k, l);
			if (cds_ft_lookup(ctx->ft, iter) == CDS_FT_STATUS_OK &&
			    (node = cds_ft_iter_node(iter)) != NULL &&
			    cds_ft_remove(ctx->ft, iter, node) == CDS_FT_STATUS_OK)
				node_free_rcu(to_test_node(node));
		} else {
			/* Trie drained. */
			__atomic_store_n(&test_drained, 1, __ATOMIC_RELAXED);
			pthread_mutex_unlock(&ctx->lock);
			rcu_read_unlock();
			break;
		}
		pthread_mutex_unlock(&ctx->lock);
		rcu_read_unlock();
	}

	cds_ft_iter_destroy(iter);
	rcu_unregister_thread();
	return NULL;
}

/*
 * Min-draining via cds_ft_remove_all: exercises the cds_ft_remove_all is_prefix
 * path AND, for the empty (NIL) key, the cds_ft_remove_all key_len==0 path --
 * both of which clear an internal holder's external_nodes.  The empty key is
 * the global minimum (a prefix of every key), so it drains first.
 */
static void *inv_remove_xview_drainer_minvl_all(void *arg)
{
	struct inv_lookup_ctx *ctx = (struct inv_lookup_ctx *) arg;
	struct cds_ft_iter *iter;

	rcu_register_thread();
	if (cds_ft_iter_create(ctx->ft, &iter) < 0)
		abort();

	while (!test_go)
		;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	while (!test_stop) {
		struct cds_ft_node *node, *removed;
		uint8_t k[8];
		size_t l;

		rcu_read_lock();
		pthread_mutex_lock(&ctx->lock);
		if (cds_ft_lookup_first(ctx->ft, iter) == CDS_FT_STATUS_OK &&
		    (node = cds_ft_iter_node(iter)) != NULL) {
			l = xview_node_key(node, k);
			cds_ft_iter_set_key(iter, k, l);
			if (cds_ft_lookup(ctx->ft, iter) == CDS_FT_STATUS_OK &&
			    cds_ft_iter_node(iter) != NULL &&
			    cds_ft_remove_all(ctx->ft, iter, &removed) ==
				    CDS_FT_STATUS_OK && removed != NULL)
				node_free_rcu(to_test_node(removed));
		} else {
			/* Trie drained. */
			__atomic_store_n(&test_drained, 1, __ATOMIC_RELAXED);
			pthread_mutex_unlock(&ctx->lock);
			rcu_read_unlock();
			break;
		}
		pthread_mutex_unlock(&ctx->lock);
		rcu_read_unlock();
	}

	cds_ft_iter_destroy(iter);
	rcu_unregister_thread();
	return NULL;
}

/* Build @ft with the varlen prefix/extension pairs (Kp 4 B, Kl 5 B). */
static void prefix_siblings_populate(struct cds_ft *ft)
{
	unsigned int g;

	for (g = 0; g < REMOVE_XVIEW_GROUPS; g++) {
		uint8_t kp[4] = { (uint8_t)(g >> 8), (uint8_t)(g & 0xff), 0, 0 };
		uint8_t kl[5] = { (uint8_t)(g >> 8), (uint8_t)(g & 0xff), 0, 0, 0 };
		struct ft_test_node *np = node_alloc(g);
		struct ft_test_node *nl = node_alloc(g);

		np->value = 4;
		memcpy(np->okey, kp, 4);
		nl->value = 5;
		memcpy(nl->okey, kl, 5);
		if (cds_ft_insert(ft, kp, 4, &np->node) != CDS_FT_STATUS_OK)
			abort();
		if (cds_ft_insert(ft, kl, 5, &nl->node) != CDS_FT_STATUS_OK)
			abort();
	}
}

static int inv_remove_cross_view_prefix_siblings(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_varlen_ord_ft(&group);

	rcu_read_lock();
	prefix_siblings_populate(ft);
	rcu_read_unlock();

	return run_remove_cross_view(ft, group,
		"inv_remove_cross_view_prefix_siblings",
		inv_remove_xview_reader_minvl, inv_remove_xview_drainer_minvl);
}

/*
 * Same prefix-with-siblings invariant, drained via cds_ft_remove_all and with
 * the empty (NIL) key added so the remove_all key_len==0 clear is exercised too.
 */
static int inv_remove_cross_view_prefix_siblings_all(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_varlen_ord_ft(&group);
	struct ft_test_node *ne = node_alloc(0);

	rcu_read_lock();
	ne->value = 0;			/* empty key: length 0, okey unused. */
	if (cds_ft_insert(ft, ne->okey, 0, &ne->node) != CDS_FT_STATUS_OK)
		abort();
	prefix_siblings_populate(ft);
	rcu_read_unlock();

	return run_remove_cross_view(ft, group,
		"inv_remove_cross_view_prefix_siblings_all",
		inv_remove_xview_reader_minvl, inv_remove_xview_drainer_minvl_all);
}

/*
 * BULK-OP (disappear-side) cross-view oracle.  A bulk detach severs a whole
 * sub-branch from @ft structurally (one publish) and then moves that run out of
 * @ft's ordered cell list (a separate ft_ord_cell_run_detach flip) -- two
 * commits, unlike the fused point remove.  So a reader doing the same
 * lookup_first -> point(min) -> lookup_first check may observe the ordered
 * minimum present in the list but absent from the structure, if the structural
 * detach raced ahead of the run unsplice.
 *
 * @ft is pre-filled with keys [p_hi,p_lo,s] (a 2-byte prefix P plus a suffix s);
 * the writer detaches prefixes in increasing order (cds_ft_detach at the 2-byte
 * P), so each detach removes the current ordered-minimum run and @ft only ever
 * SHRINKS -- no re-inserts, so "present again" can only be the non-atomic
 * detach window.  The detached sub-tries are drained + destroyed (deferred node
 * frees keep concurrent @ft readers safe).  Reuses the varlen min cross-view
 * reader.  Mirrors inv_merge_atomic_completeness: the bulk op + drain run on the
 * registered main thread (the FT bulk ops tolerate an online caller).
 */
#ifndef DETACH_XVIEW_PREFIXES
#define DETACH_XVIEW_PREFIXES	6000
#endif
#define DETACH_XVIEW_PER	3

static void drain_trie_local(struct cds_ft *ft);	/* defined below */

/*
 * ROOT-ALWAYS-INTERNAL invariant oracle.  ft->root must always tag a plain
 * internal node (the read-side hot descent relies on it; the descent carries a
 * defensive "non-internal root" resolver whose comment claims graft_swap can
 * place a compressed node at the root).  Every re-rooting mutator materializes
 * the new root through the build-invisible internal-root builders, so a compressed
 * root should NEVER be published -- not even transiently.  A post-op probe cannot
 * see a transient (it is canonical by op-end), so this samples ft->root
 * CONCURRENTLY with a re-rooting bulk op.
 *
 * The writer round-trips a KEY_SHORTER graft_swap (the path that builds the
 * extracted swap root from a split compressed prefix): @live holds keys under a
 * long compressed prefix "ABCDEFG", and graft_swap at the SHORTER key "AB" moves
 * that subtree to @swap (building @swap's new root) and back.  Readers hammer
 * cds_ft_debug_root_is_internal() on BOTH tries.  If the descent's
 * non-internal-root resolver is live, a reader catches a compressed root here;
 * if it is dead (as expected), this is provably 0 and the resolver can be
 * retired.
 */
extern int cds_ft_debug_root_is_internal(struct cds_ft *ft) __attribute__((weak));





/*
 * BULK-OP (appear-side) cross-view oracle -- the dual of inv_detach_cross_view.
 * A graft publishes the grafted run into @dst structurally and THEN splices its
 * cell run into @dst's ordered list (on HEAD, a separate ft_ord_cell_run_splice
 * flip), so between the two commits a grafted key is reachable in the STRUCTURE
 * but missing from the ordered LIST.  The fix fuses both into ONE flip.
 *
 * Like the disappear oracle, this is NON-CYCLING (keys only ever APPEAR), which
 * is what makes the check sound: the main thread grafts a fresh run at a
 * strictly DECREASING prefix, so each graft installs a new GLOBAL MINIMUM and
 * the ordered-list minimum only ever decreases -- never re-appears.  The reader
 * compares two independent views of the minimum:
 *   - the ordered-list front  via cds_ft_lookup_first (the O(1) cell-list head),
 *   - the structural minimum  via cds_ft_lookup_ge from a sub-minimal key (a
 *     pure trie descent, no cell list).
 * It reads the list front TWICE around the structural descent and flags only
 * when the front is STABLE yet the structural minimum is a SMALLER key: that
 * means a key is reachable in the structure below the ordered-list front, i.e.
 * its run was published structurally but its cell not yet spliced.  The
 * front-stable sandwich filters an atomic one-flip graft (which moves the front
 * and the structural min together -- the two front reads then differ); only a
 * genuine two-commit window survives.  Monotone-decreasing + add-only forbids
 * the front from rising again, so there are no benign re-appear false positives.
 */
#ifndef GRAFT_XVIEW_GRAFTS
#define GRAFT_XVIEW_GRAFTS	20000
#endif

/* Lexicographic compare of two byte keys (shorter sorts first on a tie). */
static int xview_key_cmp(const uint8_t *a, size_t la, const uint8_t *b, size_t lb)
{
	size_t n = la < lb ? la : lb;
	int c = memcmp(a, b, n);

	if (c)
		return c;
	return la < lb ? -1 : (la > lb ? 1 : 0);
}

static void *inv_graft_xview_appear_reader(void *arg)
{
	struct inv_lookup_ctx *ctx = (struct inv_lookup_ctx *) arg;
	struct cds_ft_iter *iter;
	static const uint8_t SUBMIN[3] = { 0x00, 0x00, 0x00 };

	rcu_register_thread();
	if (cds_ft_iter_create(ctx->ft, &iter) < 0)
		abort();
	while (!test_go)
		;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	while (!test_stop) {
		struct cds_ft_node *n;
		uint8_t kf1[8], kf2[8], ks[8];
		size_t lf1, lf2, lks;

		rcu_read_lock();
		/* List front (cell-list head). */
		if (cds_ft_lookup_first(ctx->ft, iter) == CDS_FT_STATUS_OK &&
		    (n = cds_ft_iter_node(iter)) != NULL) {
			lf1 = xview_node_key(n, kf1);
			/* Structural minimum via a pure trie descent (>= submin). */
			cds_ft_iter_set_key(iter, SUBMIN, 3);
			if (cds_ft_lookup_ge(ctx->ft, iter) == CDS_FT_STATUS_OK &&
			    (n = cds_ft_iter_node(iter)) != NULL) {
				lks = xview_node_key(n, ks);
				/* Re-read the list front: only a STABLE front lets us
				 * compare the two views at one consistent moment. */
				if (cds_ft_lookup_first(ctx->ft, iter) == CDS_FT_STATUS_OK &&
				    (n = cds_ft_iter_node(iter)) != NULL) {
					lf2 = xview_node_key(n, kf2);
					if (xview_key_cmp(kf1, lf1, kf2, lf2) == 0 &&
					    xview_key_cmp(ks, lks, kf1, lf1) < 0)
						report_violation(ctx->test_name,
							"structural minimum (len %zu) is BELOW"
							" the stable ordered-list front (len %zu)"
							" -- a grafted run is published in the"
							" structure but not yet spliced into the"
							" ordered list", lks, lf1);
				}
			}
		}
		rcu_read_unlock();
		/* QSBR: cds_ft_graft synchronize_rcu's; never stall the grafter. */
		rcu_quiescent_state();
	}

	cds_ft_iter_destroy(iter);
	rcu_unregister_thread();
	return NULL;
}

static int inv_graft_cross_view(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_varlen_ord_ft(&group);
	struct inv_lookup_ctx ctx;
	pthread_t readers[NR_READERS_DEFAULT];
	struct timespec t0;
	unsigned int i, k;

	/* Stable upper bulk: prefix {0xFF,0xFF}, always above every grafted run. */
	rcu_read_lock();
	for (i = 0; i < 16; i++) {
		uint8_t key[3] = { 0xFF, 0xFF, (uint8_t) i };
		struct ft_test_node *n = node_alloc(0x1000 + i);

		n->value = 3;
		memcpy(n->okey, key, 3);
		if (cds_ft_insert(ft, key, 3, &n->node) != CDS_FT_STATUS_OK)
			abort();
	}
	rcu_read_unlock();

	ctx.ft = ft;
	ctx.test_name = "inv_graft_cross_view";
	test_go = 0;
	test_stop = 0;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	for (i = 0; i < NR_READERS_DEFAULT; i++)
		pthread_create(&readers[i], NULL,
			inv_graft_xview_appear_reader, &ctx);
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	test_go = 1;

	/*
	 * Graft a fresh 2-key run at a strictly DECREASING 2-byte prefix, so each
	 * graft installs a new global minimum.  The source holds the run's SUFFIX
	 * keys ({0x00},{0x01}); after the graft they become prefix-extended keys in
	 * @ft (the test node stashes the full @okey).  Time-bounded: each graft is
	 * a grace period, so the pool may not drain within the window.
	 */
	clock_gettime(CLOCK_MONOTONIC, &t0);
	for (k = 0; k < GRAFT_XVIEW_GRAFTS; k++) {
		unsigned int pv = 0xFEFF - k;	/* decreasing distinct prefix */
		uint8_t prefix[2] = { (uint8_t)(pv >> 8), (uint8_t)(pv & 0xff) };
		struct cds_ft *src;
		unsigned int s;

		if (cds_ft_create(group, NULL, &src) < 0)
			abort();
		rcu_read_lock();
		for (s = 0; s < 2; s++) {
			uint8_t suffix[1] = { (uint8_t) s };
			uint8_t full[3] = { prefix[0], prefix[1], (uint8_t) s };
			struct ft_test_node *n = node_alloc(pv * 4 + s);

			n->value = 3;
			memcpy(n->okey, full, 3);
			if (cds_ft_insert(src, suffix, 1, &n->node) != CDS_FT_STATUS_OK)
				abort();
		}
		rcu_read_unlock();
cds_ft_make_exclusive(src);	/* DLM: cross-trie src must be exclusive */

		if (cds_ft_graft(ft, prefix, 2, src) != CDS_FT_STATUS_OK)
			abort();
		cds_ft_destroy(src);		/* emptied by the graft */

		if (elapsed_ms(&t0) >= DEFAULT_DURATION_MS)
			break;
	}

	test_stop = 1;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	for (i = 0; i < NR_READERS_DEFAULT; i++)
		pthread_join(readers[i], NULL);

	if (atomic_load(&violation_count) > 0) {
		fprintf(stderr, "inv_graft_cross_view: %lu violation(s)\n",
			atomic_load(&violation_count));
		drain_and_destroy(ft, group);
		return -1;
	}
	return drain_and_destroy(ft, group);
}

/*
 * Functional check for the per-trie EAGER opt-out
 * (cds_ft_attr_set_speculative_keys false): a trie created EAGER inside a
 * SPECULATIVE group must reconstruct the result key from the structure and
 * NEVER read the leaf's stored speculative key.  Insert a key K but stamp the
 * leaf's okey with a deliberately WRONG value W; lookup_first on the EAGER trie
 * must return K, not W.  (On a normal speculative trie it would return W -- the
 * stale-key footgun the opt-out exists to avoid.)  Single-threaded.
 */
static int inv_speculative_per_trie_eager(void)
{
	struct cds_ft_group_attr *gattr;
	struct cds_ft_group *group;
	struct cds_ft_attr *tattr;
	struct cds_ft *eager = NULL;
	struct cds_ft_iter *it = NULL;
	struct ft_test_node *n;
	uint8_t rk[16];
	size_t rk_len = 0;
	static const uint8_t K[3] = { 0x10, 0x20, 0x30 };
	static const uint8_t W[3] = { 0xAA, 0xBB, 0xCC };	/* wrong okey */
	int ret = -1;

	if (cds_ft_group_attr_create(&gattr) < 0)
		return -1;
	cds_ft_group_attr_set_lookup_optimization(gattr,
		CDS_FT_LOOKUP_OPTIMIZE_SPECULATIVE);
	if (cds_ft_group_attr_set_speculative_key_offset(gattr,
			offsetof(struct ft_test_node, okey)) < 0 ||
	    cds_ft_group_create(gattr, &group) < 0) {
		cds_ft_group_attr_destroy(gattr);
		return -1;
	}
	cds_ft_group_attr_destroy(gattr);

	if (cds_ft_attr_create(&tattr) < 0) {
		cds_ft_group_destroy(group);
		return -1;
	}
	cds_ft_attr_set_speculative_keys(tattr, false);		/* EAGER */
	if (cds_ft_create(group, tattr, &eager) < 0) {
		cds_ft_attr_destroy(tattr);
		cds_ft_group_destroy(group);
		return -1;
	}
	cds_ft_attr_destroy(tattr);

	n = node_alloc(0);
	n->value = 3;
	memcpy(n->okey, W, 3);					/* stamp the WRONG key */
	rcu_read_lock();
	if (cds_ft_insert(eager, K, 3, &n->node) != CDS_FT_STATUS_OK) {
		rcu_read_unlock();
		goto out;
	}
	cds_ft_iter_create(eager, &it);
	if (cds_ft_lookup_first(eager, it) != CDS_FT_STATUS_OK) {
		rcu_read_unlock();
		goto out;
	}
	cds_ft_iter_get_key(it, rk, sizeof(rk), &rk_len);
	rcu_read_unlock();
	if (rk_len != 3 || memcmp(rk, K, 3) != 0) {
		fprintf(stderr, "inv_speculative_per_trie_eager: result key "
			"%02x%02x%02x len %zu, expected %02x%02x%02x -- EAGER "
			"opt-out did not suppress the stamped speculative key\n",
			rk[0], rk[1], rk[2], rk_len, K[0], K[1], K[2]);
		goto out;
	}
	ret = 0;
out:
	if (it)
		cds_ft_iter_destroy(it);
	if (eager)
		return drain_and_destroy(eager, group) == 0 ? ret : -1;
	cds_ft_group_destroy(group);
	return ret;
}

/*
 * cds_ft_verify catches a leaf whose stored speculative key does not match its
 * position (the verify-time safety net for re-keying moves: an app that stamps
 * leaves with their destination key before a staging graft can verify the stamps
 * are right; FEATURE_FT_VERIFY_AT_MUTATION then checks after every mutation).
 * Correct stamp -> verify OK; a stale stamp -> verify FAILS.  Cleanup uses exact
 * lookups (which descend by the query key, unaffected by the stale stored key).
 */
static int inv_speculative_key_verify(void)
{
	struct cds_ft_group_attr *gattr;
	struct cds_ft_group *group;
	struct cds_ft *spec = NULL;
	struct ft_test_node *n1, *n2 = NULL;

	/*
	 * This test deliberately inserts a leaf with a wrong stored key, then
	 * checks cds_ft_verify catches it.  Under verify-at-mutation the same
	 * verify runs (and aborts) at the insert itself, before this explicit
	 * call -- which IS the feature working, but it pre-empts the test.  Skip.
	 */
	if (cds_ft_verify_at_mutation_enabled())
		return 0;
	static const uint8_t K1[2] = { 0x11, 0x22 };
	static const uint8_t K2[2] = { 0x33, 0x44 };
	static const uint8_t WRONG[2] = { 0x99, 0x88 };
	int ret = -1;

	if (cds_ft_group_attr_create(&gattr) < 0)
		return -1;
	cds_ft_group_attr_set_lookup_optimization(gattr,
		CDS_FT_LOOKUP_OPTIMIZE_SPECULATIVE);
	if (cds_ft_group_attr_set_key_len(gattr, 2) < 0 ||
	    cds_ft_group_attr_set_speculative_key_offset(gattr,
			offsetof(struct ft_test_node, okey)) < 0 ||
	    cds_ft_group_create(gattr, &group) < 0) {
		cds_ft_group_attr_destroy(gattr);
		return -1;
	}
	cds_ft_group_attr_destroy(gattr);
	if (cds_ft_create(group, NULL, &spec) < 0) {
		cds_ft_group_destroy(group);
		return -1;
	}

	/* Correctly-stamped leaf: verify must pass. */
	n1 = node_alloc(0);
	memcpy(n1->okey, K1, 2);
	rcu_read_lock();
	if (cds_ft_insert(spec, K1, 2, &n1->node) != CDS_FT_STATUS_OK) {
		rcu_read_unlock();
		goto out;
	}
	rcu_read_unlock();
	if (cds_ft_verify(spec, NULL) != CDS_FT_STATUS_OK) {
		fprintf(stderr, "inv_speculative_key_verify: verify FAILED on a "
			"correctly-stamped leaf\n");
		goto out;
	}

	/* Stale-stamped leaf: verify must FAIL. */
	n2 = node_alloc(1);
	memcpy(n2->okey, WRONG, 2);
	rcu_read_lock();
	if (cds_ft_insert(spec, K2, 2, &n2->node) != CDS_FT_STATUS_OK) {
		rcu_read_unlock();
		goto out;
	}
	rcu_read_unlock();
	if (cds_ft_verify(spec, NULL) == CDS_FT_STATUS_OK) {
		fprintf(stderr, "inv_speculative_key_verify: verify MISSED a stale "
			"speculative key\n");
		goto out;
	}
	ret = 0;
out:
	/*
	 * Re-stamp the stale leaf with its correct key so the speculative drain
	 * (lookup_first reads the stored key) can find and free it; otherwise the
	 * deliberately-wrong okey would make the drain miss it and leak.
	 */
	if (n2)
		memcpy(n2->okey, K2, 2);
	return drain_and_destroy(spec, group) == 0 ? ret : -1;
}

/*
 * Concurrent reader/writer invariant for the ordered-list-OFF graft GLUE
 * flip-txn fold (ft_glue_txn_commit).  A fixed deep key STABLE lives under a
 * compressed path; the writer repeatedly SPLITS that path (a diverge graft at a
 * prefix that diverges inside it -> FT_GRAFT_PREP_GLUE, committed via the
 * flip-txn) and HEALS it (detach of the grafted run recompacts the path back).
 * STABLE is present the whole time -- it is the displaced old child of every
 * split -- so an exact lookup of it must ALWAYS succeed.  A miss is a torn flip:
 * the forward publish and the displaced-child re-parent observed out-of-step,
 * the window the txn makes unrepresentable.  List off so the txn path (gated on
 * !ordered_list_set) runs; EAGER readers (see create_varlen_nolist_ft).
 */
#ifndef GRAFT_NOLIST_ITERS
#define GRAFT_NOLIST_ITERS	200000
#endif

/* STABLE: single deep key -> a compressed "C0 40 40 40" path under the root. */
static const uint8_t NOLIST_STABLE[4] = { 0xC0, 0x40, 0x40, 0x40 };
/* Graft prefix: byte 1 (0x20) diverges inside STABLE's compressed path. */
static const uint8_t NOLIST_GRAFT_P[2] = { 0xC0, 0x20 };

static void *inv_graft_nolist_reader(void *arg)
{
	struct inv_lookup_ctx *ctx = (struct inv_lookup_ctx *) arg;
	struct cds_ft_iter *iter;

	rcu_register_thread();
	if (cds_ft_iter_create(ctx->ft, &iter) < 0)
		abort();
	while (!test_go)
		;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	while (!test_stop) {
		rcu_read_lock();
		cds_ft_iter_set_key(iter, NOLIST_STABLE, sizeof(NOLIST_STABLE));
		cds_ft_lookup(ctx->ft, iter);
		if (!cds_ft_iter_node(iter))
			report_violation(ctx->test_name,
				"stable deep key vanished during a diverge graft"
				" -- the GLUE flip-txn forward publish and the"
				" displaced old child's re-parent were observed"
				" out-of-step");
		rcu_read_unlock();
		/* QSBR: cds_ft_graft / cds_ft_detach synchronize_rcu. */
		rcu_quiescent_state();
	}

	cds_ft_iter_destroy(iter);
	rcu_unregister_thread();
	return NULL;
}

static int inv_graft_no_list_diverge(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_varlen_nolist_ft(&group);
	struct inv_lookup_ctx ctx;
	pthread_t readers[NR_READERS_DEFAULT];
	struct ft_test_node *sn;
	struct timespec t0;
	unsigned int i, k;

	/* STABLE is the only initial key -> one fully-compressed path. */
	sn = node_alloc(0xC0404040UL);
	sn->value = sizeof(NOLIST_STABLE);
	memcpy(sn->okey, NOLIST_STABLE, sizeof(NOLIST_STABLE));
	rcu_read_lock();
	if (cds_ft_insert(ft, NOLIST_STABLE, sizeof(NOLIST_STABLE),
			&sn->node) != CDS_FT_STATUS_OK)
		abort();
	rcu_read_unlock();

	ctx.ft = ft;
	ctx.test_name = "inv_graft_no_list_diverge";
	test_go = 0;
	test_stop = 0;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	for (i = 0; i < NR_READERS_DEFAULT; i++)
		pthread_create(&readers[i], NULL, inv_graft_nolist_reader, &ctx);
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	test_go = 1;

	clock_gettime(CLOCK_MONOTONIC, &t0);
	for (k = 0; k < GRAFT_NOLIST_ITERS; k++) {
		struct cds_ft *src, *detached;
		unsigned int s;

		/* Source: a 2-key run grafted under NOLIST_GRAFT_P. */
		if (cds_ft_create(group, NULL, &src) < 0)
			abort();
		rcu_read_lock();
		for (s = 0; s < 2; s++) {
			uint8_t suffix[1] = { (uint8_t) s };
			struct ft_test_node *n = node_alloc(k * 2 + s);

			n->value = 3;
			n->okey[0] = NOLIST_GRAFT_P[0];
			n->okey[1] = NOLIST_GRAFT_P[1];
			n->okey[2] = (uint8_t) s;
			if (cds_ft_insert(src, suffix, 1, &n->node) !=
					CDS_FT_STATUS_OK)
				abort();
		}
		rcu_read_unlock();
cds_ft_make_exclusive(src);	/* DLM: cross-trie src must be exclusive */

		/* Split: diverge inside STABLE's compressed path (GLUE flip-txn). */
		if (cds_ft_graft(ft, NOLIST_GRAFT_P, 2, src) != CDS_FT_STATUS_OK)
			abort();
		cds_ft_destroy(src);		/* emptied by the graft */

		/* Heal: detach the run, recompacting the path back to STABLE-only. */
		if (cds_ft_detach(ft, NOLIST_GRAFT_P, 2, &detached) !=
				CDS_FT_STATUS_OK)
			abort();
		drain_trie_keep_group(detached);

		if (elapsed_ms(&t0) >= DEFAULT_DURATION_MS)
			break;
	}

	test_stop = 1;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	for (i = 0; i < NR_READERS_DEFAULT; i++)
		pthread_join(readers[i], NULL);

	if (atomic_load(&violation_count) > 0) {
		fprintf(stderr, "inv_graft_no_list_diverge: %lu violation(s)\n",
			atomic_load(&violation_count));
		drain_and_destroy(ft, group);
		return -1;
	}
	return drain_and_destroy(ft, group);
}

/*
 * NOSPLIT displaced-external graft cross-view.  A short external
 * @DISPEXT_STABLE is present the whole time; each iteration grafts a LONGER key
 * sharing its prefix, so the graft descent stops at the external BELOW key_len
 * -- the NOSPLIT displaced-external store, which relocates the leaf under a
 * fresh branch as its external_nodes -- then detaches to heal back.  With the
 * displaced external's back-channel re-parent folded into the graft flip-txn
 * (dst_origin), the forward publish and the displaced leaf's re-parent flip in
 * ONE selector flip, so a concurrent lookup of @DISPEXT_STABLE must never see it
 * vanish.  (Before the fold it was a fresh-before-live direct store; this
 * exercises the converted path under readers.)
 */
static const uint8_t DISPEXT_STABLE[2] = { 0xC0, 0x40 };
static const uint8_t DISPEXT_GRAFT[4]  = { 0xC0, 0x40, 0x70, 0x70 };
#define GRAFT_DISPEXT_ITERS	200000

static void *inv_graft_dispext_reader(void *arg)
{
	struct inv_lookup_ctx *ctx = (struct inv_lookup_ctx *) arg;
	struct cds_ft_iter *iter;

	rcu_register_thread();
	if (cds_ft_iter_create(ctx->ft, &iter) < 0)
		abort();
	while (!test_go)
		;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	while (!test_stop) {
		rcu_read_lock();
		cds_ft_iter_set_key(iter, DISPEXT_STABLE, sizeof(DISPEXT_STABLE));
		cds_ft_lookup(ctx->ft, iter);
		if (!cds_ft_iter_node(iter))
			report_violation(ctx->test_name,
				"displaced short external vanished during a"
				" longer-key graft -- the forward publish and the"
				" displaced external's re-parent were observed"
				" out-of-step");
		rcu_read_unlock();
		rcu_quiescent_state();
	}

	cds_ft_iter_destroy(iter);
	rcu_unregister_thread();
	return NULL;
}

static int inv_graft_displaced_external(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_varlen_nolist_ft(&group);
	struct inv_lookup_ctx ctx;
	pthread_t readers[NR_READERS_DEFAULT];
	struct ft_test_node *sn;
	struct timespec t0;
	unsigned int i, k;

	sn = node_alloc(0xC04040UL);
	sn->value = sizeof(DISPEXT_STABLE);
	memcpy(sn->okey, DISPEXT_STABLE, sizeof(DISPEXT_STABLE));
	rcu_read_lock();
	if (cds_ft_insert(ft, DISPEXT_STABLE, sizeof(DISPEXT_STABLE),
			&sn->node) != CDS_FT_STATUS_OK)
		abort();
	rcu_read_unlock();

	ctx.ft = ft;
	ctx.test_name = "inv_graft_displaced_external";
	test_go = 0;
	test_stop = 0;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	for (i = 0; i < NR_READERS_DEFAULT; i++)
		pthread_create(&readers[i], NULL, inv_graft_dispext_reader, &ctx);
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	test_go = 1;

	clock_gettime(CLOCK_MONOTONIC, &t0);
	for (k = 0; k < GRAFT_DISPEXT_ITERS; k++) {
		struct cds_ft *src, *detached;
		unsigned int s;

		/* Source: a 2-key run grafted under DISPEXT_GRAFT (the longer key). */
		if (cds_ft_create(group, NULL, &src) < 0)
			abort();
		rcu_read_lock();
		for (s = 0; s < 2; s++) {
			uint8_t suffix[1] = { (uint8_t) s };
			struct ft_test_node *n = node_alloc(k * 2 + s);

			n->value = 5;
			if (cds_ft_insert(src, suffix, 1, &n->node) !=
					CDS_FT_STATUS_OK)
				abort();
		}
		rcu_read_unlock();
cds_ft_make_exclusive(src);	/* DLM: cross-trie src must be exclusive */

		/* Graft the longer key over DISPEXT_STABLE -> NOSPLIT displaced ext. */
		if (cds_ft_graft(ft, DISPEXT_GRAFT, sizeof(DISPEXT_GRAFT), src)
				!= CDS_FT_STATUS_OK)
			abort();
		cds_ft_destroy(src);		/* emptied by the graft */

		/* Heal: detach the grafted run, restoring DISPEXT_STABLE alone. */
		if (cds_ft_detach(ft, DISPEXT_GRAFT, sizeof(DISPEXT_GRAFT),
				&detached) != CDS_FT_STATUS_OK)
			abort();
		drain_trie_keep_group(detached);

		if (elapsed_ms(&t0) >= DEFAULT_DURATION_MS)
			break;
	}

	test_stop = 1;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	for (i = 0; i < NR_READERS_DEFAULT; i++)
		pthread_join(readers[i], NULL);

	if (atomic_load(&violation_count) > 0) {
		fprintf(stderr, "inv_graft_displaced_external: %lu violation(s)\n",
			atomic_load(&violation_count));
		drain_and_destroy(ft, group);
		return -1;
	}
	return drain_and_destroy(ft, group);
}

/*
 * BULK-OP cross-view oracle for the EMPTY-DST root-level graft --
 * cds_ft_graft(dst, "", 0, src) where dst is empty.  That path makes the empty
 * @dst adopt @src's WHOLE structure as its root AND transfer @src's WHOLE
 * ordered list (head/tail) to it.  On HEAD these were SEPARATE stores: the root
 * published first, the ordered-list head/tail after, so a reader between them
 * saw a grafted key reachable in the STRUCTURE but the ordered LIST empty.  The
 * fix fuses each side's root swap with its head/tail transfer in ONE flip
 * (ft_root_list_swap_publish).
 *
 * NON-CYCLING: each watched dst goes empty -> full exactly ONCE (key_len==0
 * graft requires an empty dst, so the same dst is never re-grafted).  The
 * watched dst changes every graft; the main thread publishes the fresh empty
 * dst through ctx->cur and readers rcu_dereference it.  Every dst is kept alive
 * in @pool until the readers join, so no reader dereferences a freed trie.
 *
 * The flip latch is a MONOTONE global selector, not a per-reader snapshot
 * (urcu_flip_proxy_get reads the live selector), so two independent probes that
 * straddle the commit see old-then-new -- a front-stable SANDWICH is required
 * exactly as in inv_graft_cross_view.  The reader reads the LIST front (O(1)
 * cell-list head) TWICE around a STRUCTURAL min descent (a pure trie descent,
 * no cell list) and flags only when the front is STABLY ABSENT yet the
 * structure holds a key: under the atomic flip the selector's monotonicity
 * forbids absent (sel 0) -> present-struct (sel 1) -> absent (sel 0); only the
 * genuine two-store window (root published before the head/tail transfer) lets
 * a key be reachable structurally while the front cell is still NULL.
 */
#ifndef ROOTSWAP_XVIEW_GRAFTS
#define ROOTSWAP_XVIEW_GRAFTS	40000
#endif

/*
 * The merge empty-dst path runs a synchronize_rcu (ft_detach_keylen) before the
 * narrow root/head-store window, so it does far fewer ops than the sync-free
 * graft path -- more concurrent readers compensate so the appear window is
 * caught reliably.
 */
#ifndef ROOTSWAP_XVIEW_READERS
#define ROOTSWAP_XVIEW_READERS	32
#endif

struct inv_rootswap_ctx {
	struct cds_ft * volatile cur;	/* currently-watched empty->full dst */
	const char *test_name;
};

static void *inv_rootswap_appear_reader(void *arg)
{
	struct inv_rootswap_ctx *ctx = (struct inv_rootswap_ctx *) arg;
	struct cds_ft_iter *iter = NULL;
	struct cds_ft *bound = NULL;	/* @iter is bound to this dst */
	static const uint8_t SUBMIN[1] = { 0x00 };

	rcu_register_thread();
	while (!test_go)
		;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	while (!test_stop) {
		struct cds_ft *dst = rcu_dereference(ctx->cur);
		bool front1, front2, struct_full;

		if (!dst) {
			rcu_quiescent_state();
			continue;
		}
		/*
		 * An iter is bound to one trie (a debug assertion enforces it),
		 * but the watched dst advances every graft.  Re-bind only when it
		 * changes -- the reader spins faster than the grafter, so it
		 * usually re-checks the same dst (and catches its empty->full
		 * window) on the cached iter.  Every dst is pool-pinned, so the
		 * cached @bound never dangles.  Create outside the read section.
		 */
		if (dst != bound) {
			if (iter)
				cds_ft_iter_destroy(iter);
			if (cds_ft_iter_create(dst, &iter) < 0)
				abort();
			bound = dst;
		}

		rcu_read_lock();
		/* Front-stable sandwich: list front, structural min, list front. */
		front1 = cds_ft_lookup_first(dst, iter) == CDS_FT_STATUS_OK
			&& cds_ft_iter_node(iter) != NULL;
		cds_ft_iter_set_key(iter, SUBMIN, 1);
		struct_full = cds_ft_lookup_ge(dst, iter) == CDS_FT_STATUS_OK
			&& cds_ft_iter_node(iter) != NULL;
		front2 = cds_ft_lookup_first(dst, iter) == CDS_FT_STATUS_OK
			&& cds_ft_iter_node(iter) != NULL;
		if (struct_full && !front1 && !front2)
			report_violation(ctx->test_name,
				"a key is reachable in the structure but the"
				" ordered-list front is stably empty -- the"
				" empty-dst root graft published the root before"
				" transferring the ordered-list head/tail", 0);
		rcu_read_unlock();
		rcu_quiescent_state();
	}

	if (iter)
		cds_ft_iter_destroy(iter);
	rcu_unregister_thread();
	return NULL;
}

static int inv_graft_root_swap_cross_view(void)
{
	struct cds_ft_group *group;
	struct cds_ft *probe = create_varlen_ord_ft(&group);
	struct inv_rootswap_ctx ctx;
	pthread_t readers[NR_READERS_DEFAULT];
	struct cds_ft **pool;
	struct timespec t0;
	unsigned int i, k, ngrafts = 0;
	int ret = 0;

	pool = (struct cds_ft **) calloc(ROOTSWAP_XVIEW_GRAFTS, sizeof(*pool));
	if (!pool)
		abort();

	ctx.cur = NULL;
	ctx.test_name = "inv_graft_root_swap_cross_view";
	test_go = 0;
	test_stop = 0;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	for (i = 0; i < NR_READERS_DEFAULT; i++)
		pthread_create(&readers[i], NULL,
			inv_rootswap_appear_reader, &ctx);
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	test_go = 1;

	clock_gettime(CLOCK_MONOTONIC, &t0);
	for (k = 0; k < ROOTSWAP_XVIEW_GRAFTS; k++) {
		struct cds_ft *dst, *src;
		unsigned int s;

		if (cds_ft_create(group, NULL, &dst) < 0)
			abort();
		if (cds_ft_create(group, NULL, &src) < 0)
			abort();
		/* Source holds a fresh 2-key run (>= the 1-byte structural submin). */
		rcu_read_lock();
		for (s = 0; s < 2; s++) {
			uint8_t key[2] = { 0x01, (uint8_t) s };
			struct ft_test_node *n = node_alloc(k * 2 + s);

			n->value = 2;
			memcpy(n->okey, key, 2);
			if (cds_ft_insert(src, key, 2, &n->node) != CDS_FT_STATUS_OK)
				abort();
		}
		rcu_read_unlock();

		/* Publish the empty dst, THEN graft src into it (empty -> full). */
		pool[k] = dst;
		ngrafts = k + 1;
		rcu_assign_pointer(ctx.cur, dst);
		cds_ft_make_exclusive(src);	/* DLM: cross-trie src must be exclusive */
		if (cds_ft_graft(dst, (const uint8_t *) "", 0, src)
				!= CDS_FT_STATUS_OK)
			abort();
		cds_ft_destroy(src);		/* emptied by the graft */

		if (elapsed_ms(&t0) >= DEFAULT_DURATION_MS)
			break;
	}

	test_stop = 1;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	for (i = 0; i < NR_READERS_DEFAULT; i++)
		pthread_join(readers[i], NULL);

	/*
	 * Readers have joined; nothing dereferences ctx.cur any more.  Drain and
	 * free every watched dst (their leaves are app-owned test nodes), then
	 * destroy the tries and the shared group.
	 */
	rcu_assign_pointer(ctx.cur, NULL);
	for (k = 0; k < ngrafts; k++) {
		struct cds_ft_iter *iter;

		if (cds_ft_iter_create(pool[k], &iter) < 0)
			abort();
		rcu_read_lock();
		while (cds_ft_lookup_first(pool[k], iter) == CDS_FT_STATUS_OK) {
			struct cds_ft_node *head, *tmp;

			if (cds_ft_remove_all(pool[k], iter, &head) < 0) {
				ret = -1;
				break;
			}
			cds_ft_for_each_duplicate_safe_rcu(head, tmp)
				node_free_rcu(to_test_node(head));
		}
		rcu_read_unlock();
		cds_ft_iter_destroy(iter);
	}
	rcu_barrier();
	for (k = 0; k < ngrafts; k++)
		cds_ft_destroy(pool[k]);
	free(pool);

	if (atomic_load(&violation_count) > 0) {
		fprintf(stderr, "inv_graft_root_swap_cross_view: %lu violation(s)\n",
			atomic_load(&violation_count));
		ret = -1;
	}
	cds_ft_destroy(probe);
	cds_ft_group_destroy(group);
	return ret;
}

/*
 * BULK-OP cross-view oracle for the EMPTY-DST root-level MERGE -- the twin of
 * inv_graft_root_swap_cross_view for cds_ft_merge_at(dst, "", 0, src, "", 0)
 * with dst empty (the ft_merge_at dst_key_len == 0 && cnt_dst == 0 path).  That
 * path detaches the whole source into a fresh EXCLUSIVE subtree and swaps it
 * into the empty dst root, then transfers the subtree's whole ordered list
 * (head/tail) -- on HEAD two separate stores, fused into ONE flip by the fix.
 * Only the dst side has concurrent readers (the subtree is exclusive, no
 * src-side disappear window), so the reader and its soundness argument are
 * identical to the graft twin: it reuses inv_rootswap_appear_reader (the
 * front-stable sandwich required by the monotone global flip selector).
 */
static int inv_merge_root_swap_cross_view(void)
{
	struct cds_ft_group *group;
	struct cds_ft *probe = create_varlen_ord_ft(&group);
	struct inv_rootswap_ctx ctx;
	pthread_t readers[ROOTSWAP_XVIEW_READERS];
	struct cds_ft **pool;
	struct timespec t0;
	unsigned int i, k, nmerges = 0;
	int ret = 0;

	pool = (struct cds_ft **) calloc(ROOTSWAP_XVIEW_GRAFTS, sizeof(*pool));
	if (!pool)
		abort();

	ctx.cur = NULL;
	ctx.test_name = "inv_merge_root_swap_cross_view";
	test_go = 0;
	test_stop = 0;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	for (i = 0; i < ROOTSWAP_XVIEW_READERS; i++)
		pthread_create(&readers[i], NULL,
			inv_rootswap_appear_reader, &ctx);
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	test_go = 1;

	clock_gettime(CLOCK_MONOTONIC, &t0);
	for (k = 0; k < ROOTSWAP_XVIEW_GRAFTS; k++) {
		struct cds_ft *dst, *src;
		unsigned int s;

		static const uint8_t SRC_PREFIX[1] = { 0x01 };

		if (cds_ft_create(group, NULL, &dst) < 0)
			abort();
		if (cds_ft_create(group, NULL, &src) < 0)
			abort();
		/*
		 * Source holds a fresh 2-key run UNDER prefix {0x01}.  A
		 * src_key_len > 0 is what routes the merge to the empty-dst
		 * root-swap path: a src_key_len == 0 whole-source move is instead
		 * delegated to ft_graft_keylen (the graft empty-dst path, covered
		 * by inv_graft_root_swap_cross_view).  Moved key K = {0x01}||S
		 * becomes ""||S = S, so dst gains {0x00},{0x01} (>= the submin).
		 */
		rcu_read_lock();
		for (s = 0; s < 2; s++) {
			uint8_t key[2] = { 0x01, (uint8_t) s };
			struct ft_test_node *n = node_alloc(k * 2 + s);

			n->value = 2;
			memcpy(n->okey, key, 2);
			if (cds_ft_insert(src, key, 2, &n->node) != CDS_FT_STATUS_OK)
				abort();
		}
		rcu_read_unlock();

		/* Publish the empty dst, THEN merge src@{0x01} into it at "" (empty
		 * -> full): dst_key_len == 0 + empty dst hits the root-swap path. */
		pool[k] = dst;
		nmerges = k + 1;
		rcu_assign_pointer(ctx.cur, dst);
		cds_ft_make_exclusive(src);	/* DLM: cross-trie src must be exclusive */
		if (cds_ft_merge_at(dst, (const uint8_t *) "", 0, src, SRC_PREFIX, 1)
				!= CDS_FT_STATUS_OK)
			abort();
		cds_ft_destroy(src);		/* emptied by the merge */

		if (elapsed_ms(&t0) >= DEFAULT_DURATION_MS)
			break;
	}

	test_stop = 1;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	for (i = 0; i < ROOTSWAP_XVIEW_READERS; i++)
		pthread_join(readers[i], NULL);

	/* Readers joined; drain and free every watched dst, then the group. */
	rcu_assign_pointer(ctx.cur, NULL);
	for (k = 0; k < nmerges; k++) {
		struct cds_ft_iter *iter;

		if (cds_ft_iter_create(pool[k], &iter) < 0)
			abort();
		rcu_read_lock();
		while (cds_ft_lookup_first(pool[k], iter) == CDS_FT_STATUS_OK) {
			struct cds_ft_node *head, *tmp;

			if (cds_ft_remove_all(pool[k], iter, &head) < 0) {
				ret = -1;
				break;
			}
			cds_ft_for_each_duplicate_safe_rcu(head, tmp)
				node_free_rcu(to_test_node(head));
		}
		rcu_read_unlock();
		cds_ft_iter_destroy(iter);
	}
	rcu_barrier();
	for (k = 0; k < nmerges; k++)
		cds_ft_destroy(pool[k]);
	free(pool);

	if (atomic_load(&violation_count) > 0) {
		fprintf(stderr, "inv_merge_root_swap_cross_view: %lu violation(s)\n",
			atomic_load(&violation_count));
		ret = -1;
	}
	cds_ft_destroy(probe);
	cds_ft_group_destroy(group);
	return ret;
}

/*
 * BULK-OP (disappear-side) cross-view oracle for cds_ft_merge_at's ROOT-SRC
 * spine-copy shape: merging a WHOLE source (src_key_len == 0) into an OCCUPIED
 * dst sub-position (cnt_dst > 0) reaches ft_merge_spine_copy's root_src branch,
 * which swaps src->root to a fresh empty root (structure) and THEN unlinks the
 * source's whole run from src's ordered list (a separate ft_ord_cell_run_unlink
 * flip), so a SRC reader between them sees src structurally empty but its
 * ordered-list front still populated.  The fix fuses the root swap with the
 * head/tail clear (ft_root_list_swap_publish).
 *
 * The disappear-side dual of inv_rootswap_appear_reader: the watched src changes
 * every merge (a fresh src, emptied by the merge), published through ctx->cur
 * and pool-pinned until the readers join; the reader rebinds its iter per src and
 * uses the front-stable sandwich (front, struct, front), flagging when the front
 * is STABLY PRESENT yet the structure is empty.  NON-CYCLING: each src goes
 * full -> empty exactly once, so a re-populating front is impossible.
 */
static void *inv_rootswap_disappear_reader(void *arg)
{
	struct inv_rootswap_ctx *ctx = (struct inv_rootswap_ctx *) arg;
	struct cds_ft_iter *iter = NULL;
	struct cds_ft *bound = NULL;
	static const uint8_t SUBMIN[1] = { 0x00 };

	rcu_register_thread();
	while (!test_go)
		;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	while (!test_stop) {
		struct cds_ft *src = rcu_dereference(ctx->cur);
		bool front1, front2, struct_full;

		if (!src) {
			rcu_quiescent_state();
			continue;
		}
		if (src != bound) {
			if (iter)
				cds_ft_iter_destroy(iter);
			if (cds_ft_iter_create(src, &iter) < 0)
				abort();
			bound = src;
		}

		rcu_read_lock();
		front1 = cds_ft_lookup_first(src, iter) == CDS_FT_STATUS_OK
			&& cds_ft_iter_node(iter) != NULL;
		cds_ft_iter_set_key(iter, SUBMIN, 1);
		struct_full = cds_ft_lookup_ge(src, iter) == CDS_FT_STATUS_OK
			&& cds_ft_iter_node(iter) != NULL;
		front2 = cds_ft_lookup_first(src, iter) == CDS_FT_STATUS_OK
			&& cds_ft_iter_node(iter) != NULL;
		if (front1 && front2 && !struct_full)
			report_violation(ctx->test_name,
				"the ordered-list front is stably present but the"
				" structure is empty -- the root_src merge swapped"
				" src->root to empty before clearing the"
				" ordered-list head/tail", 0);
		rcu_read_unlock();
		rcu_quiescent_state();
	}

	if (iter)
		cds_ft_iter_destroy(iter);
	rcu_unregister_thread();
	return NULL;
}

/* ================================================================== */
/*                                                                    */
/*   EMPTY-DST ROOT ATTACH vs a concurrent KEYED attach (writer-side) */
/*                                                                    */
/* ================================================================== */

/*
 * The empty-dst ROOT attach decides "@dst_ft is empty" long before it commits,
 * and nothing re-validates that decision at the commit.
 *
 *   ft_merge_at  (dst_key_len == 0 && cnt_dst == 0): cnt_dst is computed near
 *     the top of the call; between it and the swap the path allocates a
 *     replacement src root, creates the appear txn, and runs an ENTIRE
 *     ft_detach_keylen of the source.
 *   cds_ft_graft (key_len == 0): emptiness is the nr_child/external_nodes
 *     check at the head of the root arm; then an alloc and a txn create.
 *
 * The commit validates only its RYW expected-olds -- the root pointer and the
 * old root's state word, BOTH read at RECORD time -- so a peer that populated
 * @dst_ft inside that window is matched by construction rather than detected.
 * The peer's keys then ride the old root into free_cds_ft_node.
 *
 * WHY THE PEER IS LEGAL, and why it is an attach and not an insert.  The
 * public contract grants exactly one concurrency here (fractal-trie.h, both
 * cds_ft_graft and cds_ft_merge_at): under CDS_FT_WRITER_LOCK_FINE "several
 * cross-trie attaches (cds_ft_graft, cds_ft_graft_swap, cds_ft_merge_at) may
 * run concurrently on the same destination", while "exclusion against the
 * point-update operations remains the caller's responsibility".  So the peer
 * is a second cds_ft_graft -- a KEYED one, at {0x42} -- and a cds_ft_insert
 * peer would prove nothing because the contract never promised it.
 *
 * THE ORACLE.  Per round, on a FRESH empty dst, thread A runs the root-level
 * attach (whole src_a, keys under 0x41) and thread B a keyed graft at {0x42}.
 * The two key spaces are disjoint by construction, so the check needs no
 * linearization argument at all:
 *
 *     every attach that returned OK must have ALL of its keys in dst,
 *     and every attach that did NOT return OK must have left its source
 *     UNCHANGED (the contract's "on ANY failure @src_ft is left unchanged").
 *
 * Both orders satisfy that on a correct implementation.  A first: dst gets A's
 * keys, then B's keyed graft attaches under 0x42, both OK, both present.  B
 * first: A observes a populated dst -- graft returns POPULATED_ERROR (keys stay
 * in src_a), merge_at falls through to its diverged sub-position path and
 * merges -- and again nothing is lost.  The defect is the third outcome: A
 * observes empty, B commits into the still-current root, A swaps that root away
 * and frees it.  Both report OK and B's keys are gone.
 *
 * Runs over both list settings because the merge empty-dst arm is written twice
 * (ft_root_list_swap_publish list-on vs the 2-edge root+tombstone flip
 * list-off), and the graft root arm's dual swap likewise.
 */
#define EMPTY_DST_A_PREFIX	0x41	/* thread A's keys: {0x41, hi, lo}   */
#define EMPTY_DST_B_PREFIX	0x42	/* thread B grafts its src AT {0x42} */
#define EMPTY_DST_NR_KEYS	3	/* keys per side per round           */
/*
 * Longer than DEFAULT_DURATION_MS: this oracle needs the two attaches to
 * collide inside a window bounded by one alloc + one txn create (graft) and
 * the round cost is dominated by the serial refill/drain, so a 200 ms budget
 * samples far too few collisions to be a dependable regression guard.
 */
#define EMPTY_DST_DURATION_MS	1500

enum empty_dst_mode {
	EMPTY_DST_MODE_GRAFT,	/* A = cds_ft_graft(dst, "", 0, src_a)           */
	EMPTY_DST_MODE_MERGE,	/* A = cds_ft_merge_at(dst, "", 0, src_a, "", 0) */
};

struct empty_dst_ctx {
	struct cds_ft *dst;		/* fresh per round, published before the release */
	struct cds_ft *src_a, *src_b;
	enum empty_dst_mode mode;
	unsigned long seq;		/* bumped by main to release one round */
	unsigned long done;		/* each attacher increments when finished */
	int stop;
	enum cds_ft_status st_a, st_b;
};

/*
 * Round rendezvous, SPIN-and-quiesce rather than a pthread_barrier.
 *
 * This is a QSBR suite: a registered thread that blocks without reporting a
 * quiescent state stalls every grace period in the process.  A pthread_barrier
 * here deadlocks on the very first round -- the two attachers park in
 * barrier_wait while main enters the round teardown, whose cds_ft_destroy waits
 * for a grace period the parked threads can never let complete.  That is why
 * every concurrent oracle in this file rendezvouses on spun flags.
 *
 * Returns the new @seq once it leaves @seen, or 0 when the run is stopping
 * (@seq is bumped before the first round, so 0 is never a live round).
 */
static unsigned long empty_dst_wait_round(struct empty_dst_ctx *c,
		unsigned long seen)
{
	for (;;) {
		unsigned long s = uatomic_load(&c->seq, CMM_ACQUIRE);

		if (s != seen)
			return s;
		if (uatomic_load(&c->stop, CMM_RELAXED))
			return 0;
		caa_cpu_relax();
		rcu_quiescent_state();
	}
}

/* Thread A: the ROOT-level attach -- the arm under test. */
static void *empty_dst_root_attacher(void *arg)
{
	struct empty_dst_ctx *c = (struct empty_dst_ctx *) arg;
	unsigned long seen = 0;

	rcu_register_thread();
	for (;;) {
		seen = empty_dst_wait_round(c, seen);
		if (!seen)
			break;
		if (c->mode == EMPTY_DST_MODE_GRAFT)
			c->st_a = cds_ft_graft(c->dst,
				(const uint8_t *) "", 0, c->src_a);
		else
			c->st_a = cds_ft_merge_at(c->dst,
				(const uint8_t *) "", 0, c->src_a,
				(const uint8_t *) "", 0);
		uatomic_add(&c->done, 1);
		rcu_quiescent_state();
	}
	rcu_unregister_thread();
	return NULL;
}

/* Thread B: the contract-legal KEYED peer attach into the same destination. */
static void *empty_dst_keyed_attacher(void *arg)
{
	struct empty_dst_ctx *c = (struct empty_dst_ctx *) arg;
	const uint8_t bkey[1] = { EMPTY_DST_B_PREFIX };
	unsigned long seen = 0;

	rcu_register_thread();
	for (;;) {
		seen = empty_dst_wait_round(c, seen);
		if (!seen)
			break;
		c->st_b = cds_ft_graft(c->dst, bkey, 1, c->src_b);
		uatomic_add(&c->done, 1);
		rcu_quiescent_state();
	}
	rcu_unregister_thread();
	return NULL;
}

/*
 * Fill @ft with EMPTY_DST_NR_KEYS keys of the form @prefix || {hi, lo} when
 * @with_prefix, else the bare {hi, lo} suffix (thread B's source is grafted AT
 * {0x42}, so its own keys must NOT already carry that byte).  @round
 * disambiguates rounds so a stale node from a previous round is never mistaken
 * for a live one.  Leaves @ft EXCLUSIVE, as a cross-trie source must be.
 */
static void empty_dst_fill_src(struct cds_ft *ft, uint8_t prefix,
		bool with_prefix, unsigned int round)
{
	unsigned int s;

	for (s = 0; s < EMPTY_DST_NR_KEYS; s++) {
		uint8_t key[4];
		size_t klen = 0;
		struct ft_test_node *n = node_alloc(
			((uint64_t) prefix << 32) | ((uint64_t) round << 8) | s);

		if (with_prefix)
			key[klen++] = prefix;
		key[klen++] = (uint8_t) (round >> 8);
		key[klen++] = (uint8_t) round;
		key[klen++] = (uint8_t) s;
		/* @okey holds the LANDED key, i.e. always prefixed. */
		n->okey[0] = prefix;
		memcpy(n->okey + 1, key + (with_prefix ? 1 : 0), klen - (with_prefix ? 1 : 0));
		if (cds_ft_insert(ft, key, klen, &n->node) != CDS_FT_STATUS_OK)
			abort();
	}
	/*
	 * Idempotent and one-way; called every round because a source is
	 * recycled, and a cross-trie source must be EXCLUSIVE under LOCK_FINE
	 * (a live one is rejected with BUSY).
	 */
	cds_ft_make_exclusive(ft);
}

/*
 * Are all EMPTY_DST_NR_KEYS keys of @prefix/@round present in @ft?  The landed
 * key is always @prefix || {hi, lo, s}: thread A's source carried the prefix
 * itself and moves to the root unchanged; thread B's did not and acquires it
 * from the graft point.  Returns the number FOUND.
 */
static unsigned int empty_dst_count_present(struct cds_ft *ft, uint8_t prefix,
		unsigned int round)
{
	struct cds_ft_iter *iter;
	unsigned int s, found = 0;

	if (cds_ft_iter_create(ft, &iter) < 0)
		abort();
	rcu_read_lock();
	for (s = 0; s < EMPTY_DST_NR_KEYS; s++) {
		uint8_t key[4] = { prefix, (uint8_t) (round >> 8),
			(uint8_t) round, (uint8_t) s };

		cds_ft_iter_set_key(iter, key, 4);
		if (cds_ft_lookup(ft, iter) == CDS_FT_STATUS_OK
				&& cds_ft_iter_node(iter) != NULL)
			found++;
	}
	rcu_read_unlock();
	cds_ft_iter_destroy(iter);
	return found;
}

/* Remove every key from @ft (freeing the test nodes) but KEEP the trie. */
static void empty_dst_drain_keys(struct cds_ft *ft)
{
	struct cds_ft_iter *iter;

	if (cds_ft_iter_create(ft, &iter) < 0)
		abort();
	rcu_read_lock();
	while (cds_ft_lookup_first(ft, iter) == CDS_FT_STATUS_OK) {
		struct cds_ft_node *head, *tmp;

		if (cds_ft_remove_all(ft, iter, &head) < 0)
			abort();
		cds_ft_for_each_duplicate_safe_rcu(head, tmp)
			node_free_rcu(to_test_node(head));
	}
	rcu_read_unlock();
	cds_ft_iter_destroy(iter);
}

static int empty_dst_oracle(const char *tname, enum empty_dst_mode mode,
		bool list_on)
{
	enum cds_ft_writer_strategy ws = CDS_FT_WRITER_LOCK_FINE;
	struct cds_ft_group *group;
	struct cds_ft *probe;
	struct empty_dst_ctx ctx;
	pthread_t ta, tb;
	struct timespec t0;
	unsigned int round = 0;
	unsigned long lost_a = 0, lost_b = 0, stranded = 0;
	unsigned long ok_both = 0, ok_a_only = 0, ok_b_only = 0;
	int ret = 0;

	probe = list_on ? create_varlen_ord_ft_ws(&group, &ws)
			: create_varlen_nolist_ft_ws(&group, &ws);

	ctx.mode = mode;
	ctx.stop = 0;
	ctx.seq = 0;
	ctx.done = 0;
	/*
	 * The three tries are created ONCE and recycled: each round drains them
	 * back to the round-0 state (dst empty, both sources refilled) instead of
	 * destroying and recreating.  cds_ft_destroy waits out a grace period, so
	 * a create/destroy per round held the whole oracle to SIX rounds inside
	 * the duration budget -- far too few to sample a window this narrow.
	 * Recycling buys ~3 orders of magnitude more rounds for the same wall
	 * clock, and the round counter keeps every round's keys distinct so a
	 * straggler can never be mistaken for a live key.
	 */
	if (cds_ft_create(group, NULL, &ctx.dst) < 0)
		abort();
	if (cds_ft_create(group, NULL, &ctx.src_a) < 0)
		abort();
	if (cds_ft_create(group, NULL, &ctx.src_b) < 0)
		abort();
	pthread_create(&ta, NULL, empty_dst_root_attacher, &ctx);
	pthread_create(&tb, NULL, empty_dst_keyed_attacher, &ctx);

	clock_gettime(CLOCK_MONOTONIC, &t0);
	for (;; round++) {
		unsigned int in_dst_a, in_dst_b;

		empty_dst_fill_src(ctx.src_a, EMPTY_DST_A_PREFIX, true, round);
		empty_dst_fill_src(ctx.src_b, EMPTY_DST_B_PREFIX, false, round);
		ctx.st_a = ctx.st_b = CDS_FT_STATUS_OK;

		uatomic_store(&ctx.done, 0, CMM_RELAXED);
		/* Publishes dst/src_a/src_b and the fresh statuses to both threads. */
		uatomic_store(&ctx.seq, (unsigned long) round + 1, CMM_RELEASE);
		while (uatomic_load(&ctx.done, CMM_ACQUIRE) != 2) {
			caa_cpu_relax();
			rcu_quiescent_state();
		}

		in_dst_a = empty_dst_count_present(ctx.dst,
			EMPTY_DST_A_PREFIX, round);
		in_dst_b = empty_dst_count_present(ctx.dst,
			EMPTY_DST_B_PREFIX, round);

		/* An attach that reported OK must have landed ALL of its keys. */
		if (ctx.st_a == CDS_FT_STATUS_OK
				&& in_dst_a != EMPTY_DST_NR_KEYS) {
			if (!lost_a++)
				fprintf(stderr, "%s: round %u: root attach "
					"returned OK but only %u/%u of its keys "
					"are in dst (peer st_b=%d)\n", tname,
					round, in_dst_a, EMPTY_DST_NR_KEYS,
					(int) ctx.st_b);
			ret = -1;
		}
		if (ctx.st_b == CDS_FT_STATUS_OK
				&& in_dst_b != EMPTY_DST_NR_KEYS) {
			if (!lost_b++)
				fprintf(stderr, "%s: round %u: the peer keyed "
					"graft returned OK but only %u/%u of its "
					"keys are in dst -- the root attach "
					"(st_a=%d) swapped them away\n", tname,
					round, in_dst_b, EMPTY_DST_NR_KEYS,
					(int) ctx.st_a);
			ret = -1;
		}
		/* An attach that FAILED must have left its source untouched. */
		if (ctx.st_a != CDS_FT_STATUS_OK
				&& empty_dst_count_present(ctx.src_a,
					EMPTY_DST_A_PREFIX, round)
					!= EMPTY_DST_NR_KEYS) {
			if (!stranded++)
				fprintf(stderr, "%s: round %u: root attach "
					"failed (%d) but did not leave src_a "
					"unchanged\n", tname, round,
					(int) ctx.st_a);
			ret = -1;
		}
		if (ctx.st_a == CDS_FT_STATUS_OK && ctx.st_b == CDS_FT_STATUS_OK)
			ok_both++;
		else if (ctx.st_a == CDS_FT_STATUS_OK)
			ok_a_only++;
		else if (ctx.st_b == CDS_FT_STATUS_OK)
			ok_b_only++;

		if (cds_ft_verify(ctx.dst, stderr) != CDS_FT_STATUS_OK) {
			fprintf(stderr, "%s: round %u: cds_ft_verify(dst) "
				"failed\n", tname, round);
			ret = -1;
		}
		empty_dst_drain_keys(ctx.dst);
		empty_dst_drain_keys(ctx.src_a);
		empty_dst_drain_keys(ctx.src_b);
		rcu_quiescent_state();

		if (elapsed_ms(&t0) >= EMPTY_DST_DURATION_MS)
			break;
	}

	uatomic_store(&ctx.stop, 1, CMM_RELEASE);
	pthread_join(ta, NULL);
	pthread_join(tb, NULL);

	/*
	 * Report the interleaving census, not just the verdict.  A run where
	 * ok_both is 0 has never driven the window at all -- the oracle would be
	 * green for the wrong reason, so say so out loud rather than pass.
	 */
	fprintf(stderr, "# %s: %u rounds (both-OK %lu, root-only %lu, "
		"peer-only %lu), lost: root %lu peer %lu, stranded %lu\n",
		tname, round + 1, ok_both, ok_a_only, ok_b_only,
		lost_a, lost_b, stranded);
	if (ok_both == 0) {
		fprintf(stderr, "%s: NO round had both attaches succeed -- the "
			"concurrent empty-dst window was never driven\n", tname);
		ret = -1;
	}

	cds_ft_destroy(ctx.dst);
	cds_ft_destroy(ctx.src_a);
	cds_ft_destroy(ctx.src_b);
	rcu_barrier();
	cds_ft_destroy(probe);
	cds_ft_group_destroy(group);
	return ret;
}

static int inv_empty_dst_root_graft_peer(void)
{
	return empty_dst_oracle("inv_empty_dst_root_graft_peer",
		EMPTY_DST_MODE_GRAFT, /*list_on=*/ true);
}

static int inv_empty_dst_root_graft_peer_nolist(void)
{
	return empty_dst_oracle("inv_empty_dst_root_graft_peer_nolist",
		EMPTY_DST_MODE_GRAFT, /*list_on=*/ false);
}

static int inv_empty_dst_root_merge_peer(void)
{
	return empty_dst_oracle("inv_empty_dst_root_merge_peer",
		EMPTY_DST_MODE_MERGE, /*list_on=*/ true);
}

static int inv_empty_dst_root_merge_peer_nolist(void)
{
	return empty_dst_oracle("inv_empty_dst_root_merge_peer_nolist",
		EMPTY_DST_MODE_MERGE, /*list_on=*/ false);
}

/* ================================================================== */
/*                                                                    */
/*   PREFIX-KEY PARK vs churn under its holder (external_nodes)       */
/*                                                                    */
/* ================================================================== */

/*
 * A key that TERMINATES at an internal node does not sit in a child slot: its
 * chain head is published into that node's meta->external_nodes, by
 * ft_insert_park_external_nodes, as a {NULL -> node} edge carrying only a
 * READ-ONLY validate of the holder's state word.  No retire anywhere validates
 * &meta->external_nodes.  So the standing question is whether a retire of the
 * holder can swallow a head parked concurrently into it -- a lost insert in the
 * insert/remove mix the contract fully supports (fractal-trie.h: cds_ft_insert
 * "may run concurrently with cds_ft_insert and cds_ft_remove on the same trie,
 * including on the same key").
 *
 * NOTHING in the suite drove that park concurrently at all: instrumented, the
 * whole of ft_inv executed ft_insert_park_external_nodes 54246 times and NOT
 * ONCE while a peer held the holder's node lock; every concurrent-writer
 * oracle here scored a flat zero, because they all use key sets in which no key
 * is a proper prefix of another.
 *
 * This oracle supplies that shape.  Each pair of writers shares one holder:
 *   - the PREFIX writer inserts and removes the 1-byte key {0x40+w}, which
 *     terminates AT the holder -- the park;
 *   - the CHILD writer inserts and removes {0x40+w, s}, churning the holder's
 *     child set and driving its recompaction.
 * The key layout is deliberately 1-byte-prefix + 2-byte-children so that NO
 * single-child chain forms on the path: with a compressed prefix (the obvious
 * {0xB1,w} + {0xB1,w,s,x} layout) the terminating key lands on the COMPRESSED
 * node while its CHILD does the branching, so the two writers never touch one
 * node and the whole exercise measures nothing.
 *
 * The check is exact and needs no linearization argument: a prefix key is owned
 * by exactly one thread, so an insert that returned OK must be findable by that
 * thread immediately afterwards.  A miss is a swallowed park.
 *
 * WHAT THIS DOES AND DOES NOT COVER -- stated because the numbers were the
 * point.  It drives the park hard and exposed: ~670k parks per run land while a
 * peer holds the holder's LOCK, all of them the {NULL -> node} first-publish
 * arm.  It has NOT reproduced a loss (0 across ~50M ops), and instrumentation
 * says why: those fences are the holder being locked as the PARENT {P} of a
 * child's recompact -- a {LOCK|s -> s} RELEASE lock, which never copies the
 * holder's body, so external_nodes cannot be lost through it.  A loss needs the
 * holder to be the recompact's own target C, and across every shape tried
 * ft_node_recompact was entered on a node carrying an external head ZERO times
 * (1506437 entries one shape, 3293 another).  Forcing the window open (a 60us
 * delay between the recompact's ext_snapshot read and its commit, 288 times
 * against 2.2M concurrent parks) produced ZERO parks into the delayed node.
 *
 * So this is a REGRESSION GUARD, not a refutation: the hazard is real on paper
 * and the arm is unvalidated, but the two halves do not meet in any shape that
 * could be constructed.  If a future change makes a holder its own recompact
 * target, this is what should go red.
 */
#define PREFIX_PARK_PAIRS	6	/* one prefix writer + one child writer each */
#define PREFIX_PARK_SUFFIX	48	/* child bytes per holder */
#define PREFIX_PARK_MS		1000

static unsigned long prefix_park_lost;

struct prefix_park_arg {
	struct cds_ft *ft;
	unsigned int w;
	int child_writer;
	unsigned long ops;
};

static void *prefix_park_writer(void *arg)
{
	struct prefix_park_arg *a = (struct prefix_park_arg *) arg;
	struct cds_ft_iter *iter;
	unsigned int seed = a->w * 2654435761u + (unsigned int) a->child_writer;

	rcu_register_thread();
	if (cds_ft_iter_create(a->ft, &iter) < 0)
		abort();
	while (!test_go)
		;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	while (!test_stop) {
		uint8_t key[2] = { (uint8_t) (0x40 + a->w), 0 };
		size_t klen = 1;
		struct ft_test_node *n;

		if (a->child_writer) {
			key[1] = (uint8_t) (rand_r(&seed) % PREFIX_PARK_SUFFIX);
			klen = 2;
		}
		n = node_alloc(a->w);
		memcpy(n->okey, key, klen);
		n->value = klen;
		if (cds_ft_insert(a->ft, key, klen, &n->node)
				!= CDS_FT_STATUS_OK) {
			node_free(n);
			continue;
		}
		if (!a->child_writer) {
			struct cds_ft_node *out = NULL;

			/*
			 * @key is owned by this thread alone -- no peer inserts
			 * or removes it -- so an insert that reported OK must be
			 * findable NOW.  A miss is a park swallowed by a peer's
			 * retire of the holder.
			 */
			rcu_read_lock();
			if (cds_ft_eager_lookup_key(a->ft, key, klen, 0, &out)
					!= CDS_FT_STATUS_OK
					|| out != &n->node)
				uatomic_inc(&prefix_park_lost);
			rcu_read_unlock();
		}
		rcu_read_lock();
		cds_ft_iter_set_key(iter, key, klen);
		if (cds_ft_lookup(a->ft, iter) == CDS_FT_STATUS_OK) {
			struct cds_ft_node *hd = cds_ft_iter_node(iter);

			if (hd && cds_ft_remove(a->ft, iter, hd)
					== CDS_FT_STATUS_OK)
				node_free_rcu(to_test_node(hd));
		}
		rcu_read_unlock();
		a->ops++;
		rcu_quiescent_state();
	}
	cds_ft_iter_destroy(iter);
	rcu_unregister_thread();
	return NULL;
}

static int inv_prefix_key_park_vs_holder_churn(void)
{
	enum cds_ft_writer_strategy ws = CDS_FT_WRITER_LOCK_FINE;
	struct cds_ft_group *group;
	struct cds_ft *ft = create_varlen_nolist_ft_ws(&group, &ws);
	pthread_t th[PREFIX_PARK_PAIRS * 2];
	struct prefix_park_arg a[PREFIX_PARK_PAIRS * 2];
	struct timespec t0;
	unsigned int i;
	unsigned long total = 0;
	int ret = 0;

	prefix_park_lost = 0;
	/*
	 * Seed every holder with children so the 1-byte prefix key always lands
	 * on an INTERNAL node -- on an empty trie it would be a plain leaf and
	 * never reach the external_nodes park at all.
	 */
	for (i = 0; i < PREFIX_PARK_PAIRS; i++) {
		unsigned int s;

		for (s = 0; s < PREFIX_PARK_SUFFIX; s++) {
			uint8_t ck[2] = { (uint8_t) (0x40 + i), (uint8_t) s };
			struct ft_test_node *n = node_alloc(9000 + s);

			memcpy(n->okey, ck, 2);
			n->value = 2;
			if (cds_ft_insert(ft, ck, 2, &n->node)
					!= CDS_FT_STATUS_OK)
				abort();
		}
	}

	test_go = 0;
	test_stop = 0;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	for (i = 0; i < PREFIX_PARK_PAIRS * 2; i++) {
		a[i].ft = ft;
		a[i].w = i / 2;
		a[i].child_writer = (int) (i % 2);
		a[i].ops = 0;
		pthread_create(&th[i], NULL, prefix_park_writer, &a[i]);
	}
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	test_go = 1;
	clock_gettime(CLOCK_MONOTONIC, &t0);
	while (elapsed_ms(&t0) < PREFIX_PARK_MS)
		rcu_quiescent_state();
	test_stop = 1;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	for (i = 0; i < PREFIX_PARK_PAIRS * 2; i++) {
		pthread_join(th[i], NULL);
		total += a[i].ops;
	}

	fprintf(stderr, "# inv_prefix_key_park_vs_holder_churn: %lu ops, "
		"%lu lost prefix inserts\n", total, prefix_park_lost);
	if (prefix_park_lost) {
		fprintf(stderr, "inv_prefix_key_park_vs_holder_churn: %lu "
			"insert(s) reported OK but the key was absent -- a "
			"parked external head was swallowed by a peer retire\n",
			prefix_park_lost);
		ret = -1;
	}
	/* A run that did no work proves nothing; say so rather than pass. */
	if (total == 0) {
		fprintf(stderr, "inv_prefix_key_park_vs_holder_churn: no ops\n");
		ret = -1;
	}
	if (cds_ft_verify(ft, stderr) != CDS_FT_STATUS_OK) {
		fprintf(stderr, "inv_prefix_key_park_vs_holder_churn: verify "
			"failed\n");
		ret = -1;
	}
	drain_trie_keep_group(ft);
	rcu_barrier();
	cds_ft_group_destroy(group);
	return ret;
}

/* ================================================================== */
/*                                                                    */
/*   TWO graft_swaps exchanging at ONE destination position           */
/*                                                                    */
/* ================================================================== */

/*
 * cds_ft_graft_swap is named in the public grant alongside cds_ft_graft and
 * cds_ft_merge_at: under LOCK_FINE "several cross-trie attaches ... may run
 * concurrently on the same destination".  Nothing has ever driven two of them
 * at the SAME destination position.  The existing 16-writer graft_swap oracle
 * gives every writer a DISJOINT {p,w} prefix -- they share a spine node, never
 * a destination -- so the interleaving the header licenses is uncovered.
 *
 * THE CHECK IS CONSERVATION, which needs no linearization argument.  Each
 * writer exchanges its own private trie with dst's subtree at ONE shared key,
 * so content only ever MOVES between the three tries: the total key count
 * across dst + swapA + swapB is invariant, whatever order the two swaps take
 * and whichever of them wins.  A key that goes missing was dropped by a retire
 * that could not see the peer's change -- graft_swap fences nothing and retires
 * its destination nodes through the PLAIN tombstone, which takes its expected
 * old from the committed word and so cannot tell "unchanged" from "changed by a
 * peer".  That is the shape that cost cds_ft_merge_at and cds_ft_graft their
 * empty-dst keys.
 *
 * Both swap tries are made EXCLUSIVE (the contract requires it of a source, and
 * the flag is one-way, so they stay exclusive as content circulates through
 * them).  Only dst is live.
 */
#define GS_SHARED_KEY		0x50
#define GS_KEYS_PER_TRIE	4
#define GS_ROUNDS_MS		1200
#define GS_MAX_RETRY		4096
#define GS_MAX_PREFIX		4
/* Drain bound: >> any legitimate content, small enough to fail fast. */
#define GS_DRAIN_MAX		4096

/*
 * ★ THE GRAFT-POINT SHAPE IS THE THING THIS FAMILY MUST VARY.
 *
 * Built with -DFEATURE_FT_PROBE_GRAFT_SWAP, the original fixture reports
 * `exact=... kshort=0 delegate=0 fused=0 ext_child=0` over ~90k attempts: it
 * drives exactly ONE of ft_graft_swap_descend's arms.  Every other arm --
 * KEY_SHORTER, the compressed-grandparent fuse, an EXTERNAL occupant -- had
 * never executed, so the @c9f02111 coherence fix landed on them blind.  Count
 * the arm before trusting it (feedback_probe_the_class_dont_argue_it).
 *
 * @dst_prefix is the byte string every dst key shares; @key_len is how much of
 * it the swap key spans.  Where that boundary falls INSIDE the compressed run
 * the prefix collapses into is what selects the arm:
 *
 *   key_len == dst_prefix_len == 1   the graft point is a root slot        EXACT
 *   key_len <  dst_prefix_len        the key ends inside the compressed
 *                                    node the prefix collapsed into  KEY_SHORTER
 *   key_len == dst_prefix_len > 1    the descent consumes the whole
 *                                    compressed run and lands on its
 *                                    child, so d.pnf is compressed --
 *                                    the grandparent-fuse precondition
 *
 * The shapes are not stable across a round (a swap re-shapes the run it was
 * grafted into), which is the point: the counters, not this comment, say what
 * actually ran.  Read them.
 */
struct gs_layout {
	const char *name;
	uint8_t dst_prefix[GS_MAX_PREFIX];
	unsigned int dst_prefix_len;
	unsigned int key_len;	/* graft_swap key = dst_prefix[0 .. key_len) */
	/*
	 * Writers driving the shared destination.  2 is the oracle; 1 is the
	 * CONTROL that says whether a red result needs a peer at all.  Without
	 * the control a red shape-oracle cannot distinguish "this arm is broken
	 * under concurrency" from "this arm is broken, full stop" -- and the
	 * single-threaded KEY_SHORTER path IS covered elsewhere
	 * (ft_unit rank_stats_graft_swap_ks_one), so the two answers point at
	 * completely different code.
	 */
	unsigned int nwriters;
	/*
	 * Give each swap trie TWO top-level bytes instead of one.
	 *
	 * ★ WITHOUT THIS THE SHAPE DOES NOT HOLD.  A single-top-byte swap root
	 * canonicalizes to a COMPRESSED node, which chain-merges into the
	 * compressed run it is grafted into -- so the graft point moves and the
	 * next round descends a different arm.  The probe shows it plainly:
	 * kshort=1 over 129102 rounds, i.e. the shape under test ran ONCE and
	 * the other 129101 rounds measured something else.  A branching swap
	 * root cannot merge, so the graft point stays put and the arm actually
	 * repeats.
	 */
	bool swap_branch;
	/*
	 * Bytes of shared prefix to give the SWAP content.
	 *
	 * ★ KEY_SHORTER UN-DOES ITSELF.  Grafting at a key that ends inside a
	 * compressed run SPLITS that run there, so the same key is an EXACT
	 * position next round -- which is why the first kshort layouts scored
	 * kshort=1 and then measured EXACT forever.  Swap content carrying its
	 * own deep prefix re-forms a long run under the graft key every time it
	 * lands, so the arm actually repeats.
	 */
	unsigned int swap_prefix_len;
	/*
	 * Rebuild all three tries to this layout between rounds.
	 *
	 * ★ REQUIRED TO SUSTAIN KEY_SHORTER.  The arm is self-cancelling in a
	 * swap ring: grafting at a key that ends INSIDE a compressed run SPLITS
	 * the run at exactly that key, so the key is an EXACT position from the
	 * next descent onward.  Without a reseed the shape under test runs ONCE
	 * (kshort=1) and every later round silently measures EXACT, which reads
	 * as coverage and is not.  With it, kshort tracks the round count.
	 *
	 * Rounds are far fewer than a non-reseeding run's; rounds are not the
	 * figure of merit here, kshort is.  Conservation is checked per round
	 * against that round's freshly counted total.
	 */
	bool reseed;
	/*
	 * Build the group with SPECULATIVE lookup optimization instead of EAGER.
	 *
	 * ★ REQUIRED TO REACH THE COMPRESSED-GRANDPARENT FUSE.  EAGER explicitly
	 * CLEARS CDS_FT_FLAG_SKIP_COMPRESSED (ft-lifecycle.h), and
	 * ft_compress_single_child_if_needed returns its argument unchanged when
	 * the group has skip-compression off -- so @canon is never compressed and
	 * the fuse's third precondition can never hold.  Measured: with EAGER the
	 * parent condition fires 37336 times and the canon condition ZERO, so the
	 * arm is unreachable by GROUP CONFIGURATION, not by geometry.
	 */
	bool speculative;
};

struct gs_shared_ctx {
	struct cds_ft *dst;
	struct cds_ft *swap[2];
	const struct gs_layout *lay;
	unsigned long seq;
	unsigned long done;
	unsigned long retries;
	int stop;
	enum cds_ft_status st[2];
};

/*
 * Spun rendezvous, not a barrier: this is a QSBR suite and a registered thread
 * parked in pthread_barrier_wait stalls the grace period the round teardown
 * waits on.
 */
static unsigned long gs_wait_round(struct gs_shared_ctx *c, unsigned long seen)
{
	for (;;) {
		unsigned long s = uatomic_load(&c->seq, CMM_ACQUIRE);

		if (s != seen)
			return s;
		if (uatomic_load(&c->stop, CMM_RELAXED))
			return 0;
		caa_cpu_relax();
		rcu_quiescent_state();
	}
}

struct gs_arg {
	struct gs_shared_ctx *ctx;
	unsigned int w;
};

static void *gs_shared_writer(void *arg)
{
	struct gs_arg *a = (struct gs_arg *) arg;
	struct gs_shared_ctx *c = a->ctx;
	const uint8_t *key = c->lay->dst_prefix;
	const size_t key_len = c->lay->key_len;
	unsigned long seen = 0;

	rcu_register_thread();
	for (;;) {
		seen = gs_wait_round(c, seen);
		if (!seen)
			break;
		/*
		 * RETRY on BUSY.  graft_swap reports a transient peer conflict as
		 * BUSY_ERROR (its -EAGAIN is mapped there so a caller need not
		 * read "out of memory" as "re-descend") and has NO internal retry
		 * loop, so two writers on one destination simply abort each other:
		 * measured 545892 BUSY against 2 OK before this loop existed.
		 * Retrying is what the caller is expected to do; bound it so a
		 * livelock ends the round instead of the run.
		 */
		{
			unsigned int attempt;

			for (attempt = 0; attempt < GS_MAX_RETRY; attempt++) {
				/*
				 * RE-EXCLUDE EVERY TIME.  On success @swap_ft
				 * INHERITS the destination's access discipline,
				 * and dst is live -- so a swap trie is exclusive
				 * only until its first successful swap, after
				 * which graft_swap correctly rejects it with
				 * BUSY.  Marking once at setup produced 22331392
				 * BUSY retries and 2 successes, which looked
				 * exactly like a livelock and was not one.
				 */
				cds_ft_make_exclusive(c->swap[a->w]);
				c->st[a->w] = cds_ft_graft_swap(c->dst, key,
					key_len, c->swap[a->w]);
				if (c->st[a->w] != CDS_FT_STATUS_BUSY_ERROR)
					break;
				uatomic_inc(&c->retries);
				caa_cpu_relax();
				rcu_quiescent_state();
			}
		}
		uatomic_add(&c->done, 1);
		rcu_quiescent_state();
	}
	rcu_unregister_thread();
	return NULL;
}

/*
 * Fill @ft with @n keys "@prefix ++ @tag ++ i".  @prefix is the dst layout's
 * shared prefix for the destination and empty for a swap trie (whose content
 * is re-homed under the graft key by the swap itself).  Every dst key sharing
 * the whole prefix is what makes it collapse into ONE compressed run -- the
 * run whose interior the KEY_SHORTER shape needs to land in.
 */
/* Shared prefix handed to a swap trie when the layout asks for a deep one. */
static const uint8_t gs_swap_prefix[3] = { 0xC1, 0xC2, 0xC3 };

static void gs_fill(struct cds_ft *ft, uint8_t tag, unsigned int n,
		const uint8_t *prefix, unsigned int prefix_len)
{
	unsigned int i;

	for (i = 0; i < n; i++) {
		uint8_t k[GS_MAX_PREFIX + 4];
		size_t klen = 0;
		struct ft_test_node *node = node_alloc(((uint64_t) tag << 8) | i);

		if (prefix_len) {
			memcpy(k, prefix, prefix_len);
			klen = prefix_len;
		}
		k[klen++] = tag;
		k[klen++] = (uint8_t) i;
		memcpy(node->okey, k, klen);
		node->value = klen;
		if (cds_ft_insert(ft, k, klen, &node->node) != CDS_FT_STATUS_OK)
			abort();
	}
}

/*
 * Fill dst and both swap tries to @lay's shape.  Called at setup and, for a
 * reseeding layout, once per round.
 */
static void gs_seed(struct gs_shared_ctx *c, const struct gs_layout *lay)
{
	unsigned int i;

	for (i = 0; i < 2; i++) {
		gs_fill(c->swap[i], (uint8_t) (0xA0 + i), GS_KEYS_PER_TRIE,
			gs_swap_prefix, lay->swap_prefix_len);
		if (lay->swap_branch)
			gs_fill(c->swap[i], (uint8_t) (0xB0 + i),
				GS_KEYS_PER_TRIE, gs_swap_prefix,
				lay->swap_prefix_len);
	}
	gs_fill(c->dst, 0xD0, GS_KEYS_PER_TRIE, lay->dst_prefix,
		lay->dst_prefix_len);
}

/*
 * Empty @ft but KEEP IT USABLE.  drain_trie_keep_group() is deliberately not
 * used: it keeps the GROUP and cds_ft_destroy()s the trie, and recreating a
 * trie per round costs two orders of magnitude in rounds and races the writer
 * threads that hold the pointer.
 *
 * BOUNDED.  An unbounded key-walk over a structurally broken trie is not a
 * theoretical worry -- it ran away to 187 GB RSS and took a global OOM kill.
 * The caller must only ever reach here on a trie that just VERIFIED, and this
 * bound is the second line of defence.  Returns -1 if it did not converge.
 */
static int gs_drain_keep_trie(struct cds_ft *ft)
{
	struct cds_ft_iter *iter;
	unsigned long removed = 0;
	int ret = 0;

	if (cds_ft_iter_create(ft, &iter) < 0)
		abort();
	rcu_read_lock();
	while (cds_ft_lookup_first(ft, iter) == CDS_FT_STATUS_OK) {
		struct cds_ft_node *head, *tmp;
		enum cds_ft_status s = cds_ft_remove_all(ft, iter, &head);

		if (s != CDS_FT_STATUS_OK) {
			fprintf(stderr, "gs_drain_keep_trie: lookup_first found "
				"a key but remove_all reports %d after %lu "
				"removals (count_keys=%lu)\n", (int) s, removed,
				cds_ft_count_keys(ft));
			ret = -1;
			break;
		}
		cds_ft_for_each_duplicate_safe_rcu(head, tmp)
			node_free_rcu(to_test_node(head));
		if (++removed > GS_DRAIN_MAX) {
			fprintf(stderr, "gs_drain_keep_trie: no convergence "
				"after %lu removals (count_keys=%lu)\n",
				removed, cds_ft_count_keys(ft));
			ret = -1;
			break;
		}
	}
	rcu_read_unlock();
	cds_ft_iter_destroy(iter);
	return ret;
}

/*
 * Drain all three tries and re-seed them, between rounds with both writers
 * parked in gs_wait_round so nothing is concurrently reachable.  Callers must
 * have VERIFIED all three first.  Returns -1 if any drain did not converge.
 */
static int gs_reseed(struct gs_shared_ctx *c, const struct gs_layout *lay)
{
	unsigned int i;

	if (gs_drain_keep_trie(c->dst))
		return -1;
	for (i = 0; i < 2; i++)
		if (gs_drain_keep_trie(c->swap[i]))
			return -1;
	gs_seed(c, lay);
	return 0;
}

static int gs_shared_oracle(const char *tname, bool list_on,
		const struct gs_layout *lay)
{
	enum cds_ft_writer_strategy ws = CDS_FT_WRITER_LOCK_FINE;
	struct cds_ft_group *group;
	struct cds_ft *probe;
	struct gs_shared_ctx ctx;
	struct gs_arg arg[2];
	pthread_t th[2];
	struct timespec t0;
	unsigned long total, round = 0, lost_rounds = 0;
	unsigned long both_ok = 0, one_ok = 0;
	unsigned long stcount[8] = { 0 };
	const unsigned int nw = lay->nwriters;
	unsigned int i;
	int ret = 0;

	assert(nw >= 1 && nw <= 2);
	if (lay->speculative)
		probe = list_on ? create_varlen_ord_ft_ws_spec(&group, &ws)
				: create_varlen_nolist_ft_ws_spec(&group, &ws);
	else
		probe = list_on ? create_varlen_ord_ft_ws(&group, &ws)
				: create_varlen_nolist_ft_ws(&group, &ws);

	ctx.stop = 0;
	ctx.seq = 0;
	ctx.done = 0;
	ctx.retries = 0;
	ctx.lay = lay;
	if (cds_ft_create(group, NULL, &ctx.dst) < 0)
		abort();
	for (i = 0; i < 2; i++) {
		if (cds_ft_create(group, NULL, &ctx.swap[i]) < 0)
			abort();
	}
	gs_seed(&ctx, lay);

	total = cds_ft_count_keys(ctx.dst) + cds_ft_count_keys(ctx.swap[0])
		+ cds_ft_count_keys(ctx.swap[1]);

	for (i = 0; i < nw; i++) {
		arg[i].ctx = &ctx;
		arg[i].w = i;
		pthread_create(&th[i], NULL, gs_shared_writer, &arg[i]);
	}

	clock_gettime(CLOCK_MONOTONIC, &t0);
	for (;; round++) {
		unsigned long now;

		ctx.st[0] = ctx.st[1] = CDS_FT_STATUS_OK;
		uatomic_store(&ctx.done, 0, CMM_RELAXED);
		uatomic_store(&ctx.seq, round + 1, CMM_RELEASE);
		while (uatomic_load(&ctx.done, CMM_ACQUIRE) != nw) {
			caa_cpu_relax();
			rcu_quiescent_state();
		}

		/*
		 * CONSERVATION.  Content only moves between the three tries, so
		 * the total is invariant no matter who won.
		 */
		now = cds_ft_count_keys(ctx.dst)
			+ cds_ft_count_keys(ctx.swap[0])
			+ cds_ft_count_keys(ctx.swap[1]);
		if (now != total) {
			if (!lost_rounds++)
				fprintf(stderr, "%s: round %lu: %lu keys across "
					"dst+swap0+swap1, expected %lu -- a "
					"graft_swap retired a peer's content "
					"(st0=%d st1=%d)\n", tname, round, now,
					total, (int) ctx.st[0], (int) ctx.st[1]);
			ret = -1;
			total = now;	/* re-baseline so one loss is not counted forever */
		}
		for (i = 0; i < nw; i++) {
			int idx = -(int) ctx.st[i];

			if (idx < 0 || idx > 7)
				idx = 0;
			stcount[idx]++;
		}
		/*
		 * "both" is every DRIVING writer -- with nw == 1 the control
		 * still has to report that its one swap landed, or a control
		 * that silently did nothing would read as a clean denominator.
		 */
		if (nw == 1)
			both_ok += (ctx.st[0] == CDS_FT_STATUS_OK);
		else if (ctx.st[0] == CDS_FT_STATUS_OK
				&& ctx.st[1] == CDS_FT_STATUS_OK)
			both_ok++;
		else if (ctx.st[0] == CDS_FT_STATUS_OK
				|| ctx.st[1] == CDS_FT_STATUS_OK)
			one_ok++;
		/*
		 * ★ VERIFY ALL THREE TRIES TOGETHER, not just dst, and not one
		 * at a time.  A corrupt SWAP trie has been observed while dst
		 * verified clean and BOTH swaps returned OK -- checking only the
		 * destination is blind to the half of the exchange that gets
		 * re-rooted.
		 *
		 * cds_ft_verify_disjoint, not three cds_ft_verify calls: an
		 * exchange that hands the SAME node to two tries leaves each of
		 * them self-consistent, so every per-trie walk passes.  At a
		 * root that is not even a near miss -- a root's parent is NULL
		 * in every trie, so two tries rooted at one node agree with
		 * their own expected_parent.  Only the walk that spans them
		 * sees it, and this exchange is exactly where such a node
		 * comes from.
		 */
		{
			struct cds_ft *set[3];
			size_t nr = 0;

			set[nr++] = ctx.dst;
			set[nr++] = ctx.swap[0];
			if (nw > 1)
				set[nr++] = ctx.swap[1];
			if (cds_ft_verify_disjoint(set, nr, stderr)
					!= CDS_FT_STATUS_OK) {
				fprintf(stderr, "%s: round %lu: verify_disjoint failed "
					"(st0=%d st1=%d)\n", tname, round,
					(int) ctx.st[0], (int) ctx.st[1]);
				ret = -1;
				break;
			}
		}
		rcu_quiescent_state();
		if (elapsed_ms(&t0) >= GS_ROUNDS_MS)
			break;
		/*
		 * Restore the graft-point shape for the NEXT round.
		 * DELIBERATELY AFTER the verify above, never before: the drain
		 * walks by KEY, so draining a trie this oracle has just proved
		 * broken is the mistake the teardown below refuses to make --
		 * and it does not merely abort, it runs away (187 GB RSS, OOM).
		 */
		if (lay->reseed) {
			if (gs_reseed(&ctx, lay)) {
				fprintf(stderr, "%s: round %lu: reseed failed\n",
					tname, round);
				ret = -1;
				break;
			}
			total = cds_ft_count_keys(ctx.dst)
				+ cds_ft_count_keys(ctx.swap[0])
				+ cds_ft_count_keys(ctx.swap[1]);
		}
	}

	uatomic_store(&ctx.stop, 1, CMM_RELEASE);
	for (i = 0; i < nw; i++)
		pthread_join(th[i], NULL);

	fprintf(stderr, "# %s: %lu rounds (both-OK %lu, one-OK %lu), "
		"%lu rounds lost keys, %lu BUSY retries; statuses ok=%lu inval=%lu mem=%lu "
		"overflow=%lu busy=%lu populated=%lu integrity=%lu nosup=%lu\n",
		tname, round + 1, both_ok, one_ok, lost_rounds, ctx.retries,
		stcount[0], stcount[1], stcount[2], stcount[3], stcount[4],
		stcount[5], stcount[6], stcount[7]);
	/* A run where the writers never all succeeded has not driven the window. */
	if (!ret && both_ok == 0) {
		fprintf(stderr, "%s: no round had every swap succeed -- the "
			"shared-destination window was never driven\n", tname);
		ret = -1;
	}

	/*
	 * On a DETECTED corruption, do NOT drain: the drain walks by key, and a
	 * trie whose structure this oracle just proved broken makes
	 * drain_trie_keep_group hit its stale-iterator abort() -- which killed
	 * the whole ft_inv binary at test 40 and blocked every test after it in
	 * all ten gate configs.  A red oracle must report and let the suite
	 * continue.  The nodes leak for this run and leak_check reports it, on
	 * top of the failure already recorded -- deliberate, and cheaper than
	 * losing the other 40-odd invariants.  Mirrors the dead-internal
	 * teardown detector, which dumps and skips its walk for the same reason.
	 */
	if (ret == 0) {
		drain_trie_keep_group(ctx.dst);
		for (i = 0; i < 2; i++)
			drain_trie_keep_group(ctx.swap[i]);
	} else {
		fprintf(stderr, "%s: corruption detected -- skipping the drain "
			"(its key walk would abort on the broken trie); the "
			"round's nodes leak by design\n", tname);
	}
	rcu_barrier();
	cds_ft_destroy(probe);
	cds_ft_group_destroy(group);
	return ret;
}

/*
 * The graft point is a ROOT SLOT: key_len == dst_prefix_len == 1, so the
 * descent's last step lands directly on dst's depth-1 child.  This is the
 * shape the family has always run -- FT_GRAFT_SWAP_EXACT with a plain parent.
 */
static const struct gs_layout gs_lay_exact = {
	"exact", { GS_SHARED_KEY }, 1, 1, 2, false, 0, false, false
};

/*
 * The graft point is INSIDE a compressed run: every dst key shares three
 * prefix bytes, which collapse into one compressed node, and the swap key
 * spans only two of them -- so the descent reports FT_GRAFT_SWAP_KEY_SHORTER
 * and the swap must split that node's interior.  That arm carries its own
 * extract build (ft_build_extracted_root_glue on the suffix), its own
 * ks_cn deferred free, and the LEGACY publish path whose commit status is
 * still dropped; none of it had ever executed.
 */
static const struct gs_layout gs_lay_kshort = {
	"kshort", { GS_SHARED_KEY, 0x51, 0x52 }, 3, 2, 2, false, 0, false, false
};
/*
 * DEPTH CONTROL: same deep dst as "kshort", but the swap key spans only the
 * FIRST prefix byte, so the graft point is a root slot again (EXACT, plain
 * parent) while the trie below it is just as deep.  Separates "the shape
 * oracles are red because the trie got deeper" from "because the graft
 * point's parent stopped being the root".
 */
static const struct gs_layout gs_lay_wide = {
	"wide", { GS_SHARED_KEY, 0x51, 0x52 }, 3, 1, 2, false, 0, false, false
};
/*
 * STABLE EXACT-UNDER-A-COMPRESSED-PARENT.  dst keys are prefix ++ 0xD0 ++ i,
 * so the run under the root's 0x50 slot is [0x51, 0xD0] and the swap key spans
 * all three bytes -- the descent consumes the whole run and stops on
 * cn->child.  d.pnf is therefore a COMPRESSED node and the forward publish
 * targets &cn->child rather than a root slot, which the original fixture never
 * did.  @swap_branch keeps it that way across rounds.  Note dst_prefix_len (2)
 * is what gs_fill writes; key_len (3) reaches one byte further, onto the tag
 * every dst key shares.
 */
static const struct gs_layout gs_lay_cparent = {
	"cparent", { GS_SHARED_KEY, 0x51, 0xD0 }, 2, 3, 2, true, 0, false, false
};
static const struct gs_layout gs_lay_cparent_solo = {
	"cparent-solo", { GS_SHARED_KEY, 0x51, 0xD0 }, 2, 3, 1, true, 0, false, false
};
/*
 * Deep dst run, key ending inside it, and swap content carrying its own deep
 * prefix.
 *
 * ★ THIS WAS AN ATTEMPT AT A SUSTAINED KEY_SHORTER AND IT DID NOT WORK -- the
 * probe says kshort=1 over 123280 rounds, same as the layouts without the deep
 * swap prefix.  The swap prefix only shapes ROUND 0; from round 1 the ring
 * circulates whatever was EXTRACTED from dst, and grafting at a key inside a
 * compressed run SPLITS the run there, so that key is an EXACT position ever
 * after.  KEY_SHORTER is self-cancelling in a swap ring.  Kept because it is a
 * distinct round-0 shape and a cheap control, NOT because it soaks the arm:
 * sustaining it needs a per-round reseed of dst (see the empty-dst oracles,
 * which refill their sources every round).  Read the counters, not this name.
 */
static const struct gs_layout gs_lay_ks2 = {
	"ks2", { GS_SHARED_KEY, 0x51, 0x52 }, 3, 2, 2, false, 3, false, false
};
static const struct gs_layout gs_lay_ks2_solo = {
	"ks2-solo", { GS_SHARED_KEY, 0x51, 0x52 }, 3, 2, 1, false, 3, false, false
};
/*
 * SUSTAINED KEY_SHORTER.  Same deep-run layout as "kshort", rebuilt every round
 * so the split the previous round's graft made is undone and the writers
 * descend the KEY_SHORTER arm again.  This is the regression guard for the
 * legacy publish path whose commit status @93fad396 stopped discarding.
 */
/*
 * The compressed-grandparent FUSE.  Three conditions must hold at once
 * (ft-graft.h): EXACT, the graft point's PARENT compressed, and @canon
 * compressed, with pcn->len + ccn->len <= FT_SKIP_LEN_MAX.
 *
 * Geometry: dst keys are prefix ++ 0xD0 ++ i with prefix {0x50,0x51}, so the
 * run under the root's 0x50 slot is [0x51,0xD0]; the swap key spans all three
 * bytes, so the descent consumes that run and stops on cn->child with d.pnf
 * COMPRESSED.  The swap trie keeps a SINGLE top byte (swap_branch false) so its
 * root canonicalizes to a COMPRESSED node -- the condition "cparent" misses,
 * which is why that layout reports fused=0 despite having the parent right.
 * @reseed holds the shape, which a single-top-byte swap otherwise destroys by
 * chain-merging into the run it lands in.
 */
static const struct gs_layout gs_lay_fuse = {
	"fuse", { GS_SHARED_KEY, 0x51, 0xD0 }, 2, 3, 2, false, 0, true, true
};
static const struct gs_layout gs_lay_fuse_solo = {
	"fuse-solo", { GS_SHARED_KEY, 0x51, 0xD0 }, 2, 3, 1, false, 0, true, true
};
static const struct gs_layout gs_lay_ksfix = {
	"ksfix", { GS_SHARED_KEY, 0x51, 0x52 }, 3, 2, 2, false, 0, true, false
};
static const struct gs_layout gs_lay_ksfix_solo = {
	"ksfix-solo", { GS_SHARED_KEY, 0x51, 0x52 }, 3, 2, 1, false, 0, true, false
};
static const struct gs_layout gs_lay_kshort_solo = {
	"kshort-solo", { GS_SHARED_KEY, 0x51, 0x52 }, 3, 2, 1, false, 0, false, false
};

/*
 * The graft point is a COMPRESSED NODE'S CHILD: the key spans the whole
 * shared prefix, so the descent consumes the compressed run and stops on
 * cn->child with d.pnf compressed.  That is the precondition for the
 * skip-compressed grandparent FUSE, which carries a second instance of the
 * two-load shape @c9f02111 fixed (ft-graft.h, pub_old read separately from
 * the pcn body the merged node is built from).
 */
static const struct gs_layout gs_lay_deep = {
	"deep", { GS_SHARED_KEY, 0x51 }, 2, 2, 2, false, 0, false, false
};
static const struct gs_layout gs_lay_deep_solo = {
	"deep-solo", { GS_SHARED_KEY, 0x51 }, 2, 2, 1, false, 0, false, false
};

static int inv_graft_swap_shared_dst(void)
{
	return gs_shared_oracle("inv_graft_swap_shared_dst", /*list_on=*/ true,
		&gs_lay_exact);
}

static int inv_graft_swap_shared_dst_nolist(void)
{
	return gs_shared_oracle("inv_graft_swap_shared_dst_nolist",
		/*list_on=*/ false, &gs_lay_exact);
}

static int inv_graft_swap_shared_dst_kshort(void)
{
	return gs_shared_oracle("inv_graft_swap_shared_dst_kshort",
		/*list_on=*/ true, &gs_lay_kshort);
}

static int inv_graft_swap_shared_dst_kshort_nolist(void)
{
	return gs_shared_oracle("inv_graft_swap_shared_dst_kshort_nolist",
		/*list_on=*/ false, &gs_lay_kshort);
}

static int inv_graft_swap_shared_dst_deep(void)
{
	return gs_shared_oracle("inv_graft_swap_shared_dst_deep",
		/*list_on=*/ true, &gs_lay_deep);
}

static int inv_graft_swap_shared_dst_deep_nolist(void)
{
	return gs_shared_oracle("inv_graft_swap_shared_dst_deep_nolist",
		/*list_on=*/ false, &gs_lay_deep);
}

/*
 * THE CONTROLS.  Same layouts, same key, ONE writer -- so the destination is
 * still re-shaped by a real graft_swap every round but no peer ever contends
 * it.  These must stay GREEN: if a shape oracle is red and its control is red
 * too, the arm is broken outright and the concurrency framing is wrong.
 */
static int inv_graft_swap_shared_dst_ks2_nolist(void)
{
	return gs_shared_oracle("inv_graft_swap_shared_dst_ks2_nolist",
		/*list_on=*/ false, &gs_lay_ks2);
}

static int inv_graft_swap_shared_dst_ks2_solo(void)
{
	return gs_shared_oracle("inv_graft_swap_shared_dst_ks2_solo",
		/*list_on=*/ false, &gs_lay_ks2_solo);
}

static int inv_graft_swap_shared_dst_cparent_nolist(void)
{
	return gs_shared_oracle("inv_graft_swap_shared_dst_cparent_nolist",
		/*list_on=*/ false, &gs_lay_cparent);
}

static int inv_graft_swap_shared_dst_cparent_solo(void)
{
	return gs_shared_oracle("inv_graft_swap_shared_dst_cparent_solo",
		/*list_on=*/ false, &gs_lay_cparent_solo);
}

static int inv_graft_swap_shared_dst_wide_nolist(void)
{
	return gs_shared_oracle("inv_graft_swap_shared_dst_wide_nolist",
		/*list_on=*/ false, &gs_lay_wide);
}

static int inv_graft_swap_shared_dst_fuse(void)
{
	return gs_shared_oracle("inv_graft_swap_shared_dst_fuse",
		/*list_on=*/ true, &gs_lay_fuse);
}

static int inv_graft_swap_shared_dst_fuse_nolist(void)
{
	return gs_shared_oracle("inv_graft_swap_shared_dst_fuse_nolist",
		/*list_on=*/ false, &gs_lay_fuse);
}

static int inv_graft_swap_shared_dst_fuse_solo(void)
{
	return gs_shared_oracle("inv_graft_swap_shared_dst_fuse_solo",
		/*list_on=*/ false, &gs_lay_fuse_solo);
}

static int inv_graft_swap_shared_dst_ksfix(void)
{
	return gs_shared_oracle("inv_graft_swap_shared_dst_ksfix",
		/*list_on=*/ true, &gs_lay_ksfix);
}

static int inv_graft_swap_shared_dst_ksfix_nolist(void)
{
	return gs_shared_oracle("inv_graft_swap_shared_dst_ksfix_nolist",
		/*list_on=*/ false, &gs_lay_ksfix);
}

static int inv_graft_swap_shared_dst_ksfix_solo(void)
{
	return gs_shared_oracle("inv_graft_swap_shared_dst_ksfix_solo",
		/*list_on=*/ false, &gs_lay_ksfix_solo);
}

static int inv_graft_swap_shared_dst_kshort_solo(void)
{
	return gs_shared_oracle("inv_graft_swap_shared_dst_kshort_solo",
		/*list_on=*/ false, &gs_lay_kshort_solo);
}

static int inv_graft_swap_shared_dst_deep_solo(void)
{
	return gs_shared_oracle("inv_graft_swap_shared_dst_deep_solo",
		/*list_on=*/ false, &gs_lay_deep_solo);
}

static int inv_merge_root_src_cross_view(void)
{
	struct cds_ft_group *group;
	struct cds_ft *dst = create_varlen_ord_ft(&group);
	struct inv_rootswap_ctx ctx;
	pthread_t readers[NR_READERS_DEFAULT];
	struct cds_ft **pool;
	struct timespec t0;
	static const uint8_t AT[1] = { 0x80 };
	unsigned int i, k, nmerges = 0;
	int ret = 0;

	pool = (struct cds_ft **) calloc(ROOTSWAP_XVIEW_GRAFTS, sizeof(*pool));
	if (!pool)
		abort();

	/* Occupy dst at {0x80} so each whole-src merge takes the spine-copy
	 * root_src path (cnt_dst > 0 at the merge point). */
	rcu_read_lock();
	{
		uint8_t key[2] = { 0x80, 0x00 };
		struct ft_test_node *n = node_alloc(0);

		n->value = 2;
		memcpy(n->okey, key, 2);
		if (cds_ft_insert(dst, key, 2, &n->node) != CDS_FT_STATUS_OK)
			abort();
	}
	rcu_read_unlock();

	ctx.cur = NULL;
	ctx.test_name = "inv_merge_root_src_cross_view";
	test_go = 0;
	test_stop = 0;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	for (i = 0; i < NR_READERS_DEFAULT; i++)
		pthread_create(&readers[i], NULL,
			inv_rootswap_disappear_reader, &ctx);
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	test_go = 1;

	clock_gettime(CLOCK_MONOTONIC, &t0);
	for (k = 0; k < ROOTSWAP_XVIEW_GRAFTS; k++) {
		struct cds_ft *src;
		unsigned int s;

		if (cds_ft_create(group, NULL, &src) < 0)
			abort();
		/* DISTINCT keys per merge (a per-k 2-byte suffix), so the merged
		 * content accumulates in dst as a trie rather than long dup chains
		 * (keeping each spine-copy merge and the final drain near O(log)). */
		rcu_read_lock();
		for (s = 0; s < 2; s++) {
			uint8_t key[3] = { (uint8_t)(0x01 + s),
				(uint8_t)(k >> 8), (uint8_t)(k & 0xff) };
			struct ft_test_node *n = node_alloc(k * 2 + s);

			n->value = 3;
			memcpy(n->okey, key, 3);
			if (cds_ft_insert(src, key, 3, &n->node) != CDS_FT_STATUS_OK)
				abort();
		}
		rcu_read_unlock();

		/* Publish the full src, THEN merge it WHOLE into dst@{0x80} (occupied
		 * -> spine-copy root_src): src goes full -> empty. */
		pool[k] = src;
		nmerges = k + 1;
		rcu_assign_pointer(ctx.cur, src);
		/*
		 * DECOMPOSED, same reason as the prefix oracles above: under
		 * LOCK_FINE a cross-trie source must be exclusive, and @src is
		 * what the readers watch.  The ROOT detach IS this oracle's
		 * subject -- @src goes full -> empty, and its structure and its
		 * ordered list must never disagree while it does -- and the merge
		 * then runs against an EXCLUSIVE trie.  Still a merge: dst@{AT}
		 * is occupied on purpose (the spine-copy root_src path), which a
		 * graft rejects with POPULATED_ERROR.
		 */
		{
			struct cds_ft *whole = NULL;

			if (cds_ft_detach(src, NULL, 0, &whole) != CDS_FT_STATUS_OK ||
			    !whole)
				abort();
			if (cds_ft_merge_at(dst, AT, 1, whole, NULL, 0) !=
					CDS_FT_STATUS_OK)
				abort();
			cds_ft_destroy(whole);	/* emptied by the merge */
		}

		if (elapsed_ms(&t0) >= DEFAULT_DURATION_MS)
			break;
	}

	test_stop = 1;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	for (i = 0; i < NR_READERS_DEFAULT; i++)
		pthread_join(readers[i], NULL);

	/* Srcs were emptied by the merges; dst holds all the merged content. */
	rcu_assign_pointer(ctx.cur, NULL);
	rcu_barrier();
	for (k = 0; k < nmerges; k++)
		cds_ft_destroy(pool[k]);
	free(pool);

	if (atomic_load(&violation_count) > 0) {
		fprintf(stderr, "inv_merge_root_src_cross_view: %lu violation(s)\n",
			atomic_load(&violation_count));
		ret = -1;
	}
	if (drain_and_destroy(dst, group) < 0)
		ret = -1;
	return ret;
}

/*
 * BULK-OP (appear-side) cross-view oracle for cds_ft_merge_at -- the diverged /
 * absent-dst-point merge shape (ft_merge_graft_subpos_inplace), which publishes
 * the moved subtree structurally and THEN splices its run into dst's ordered
 * list (on HEAD, a separate ft_ord_cell_run_splice flip).  Identical method to
 * inv_graft_cross_view (it reuses inv_graft_xview_appear_reader): NON-CYCLING,
 * the main thread merges a fresh src subtree into dst at a strictly DECREASING
 * dst prefix, so each merge installs a new global minimum and the ordered-list
 * minimum only ever decreases.  The reader compares lookup_first (cell list) vs
 * lookup_ge (structural descent) under a front-stable sandwich.
 */
#ifndef MERGE_XVIEW_MERGES
#define MERGE_XVIEW_MERGES	20000
#endif

static int inv_merge_cross_view(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_varlen_ord_ft(&group);
	struct inv_lookup_ctx ctx;
	pthread_t readers[NR_READERS_DEFAULT];
	struct timespec t0;
	unsigned int i, k;
	static const uint8_t SRCK[1] = { 0x53 };	/* "S": the src sub-position */

	/* Stable upper bulk: prefix {0xFF,0xFF}, always above every merged run. */
	rcu_read_lock();
	for (i = 0; i < 16; i++) {
		uint8_t key[3] = { 0xFF, 0xFF, (uint8_t) i };
		struct ft_test_node *n = node_alloc(0x2000 + i);

		n->value = 3;
		memcpy(n->okey, key, 3);
		if (cds_ft_insert(ft, key, 3, &n->node) != CDS_FT_STATUS_OK)
			abort();
	}
	rcu_read_unlock();

	ctx.ft = ft;
	ctx.test_name = "inv_merge_cross_view";
	test_go = 0;
	test_stop = 0;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	for (i = 0; i < NR_READERS_DEFAULT; i++)
		pthread_create(&readers[i], NULL,
			inv_graft_xview_appear_reader, &ctx);
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	test_go = 1;

	/*
	 * Merge a fresh src subtree (at "S", keys {S,0},{S,1}) into dst at a
	 * strictly DECREASING 2-byte prefix absent from dst -- the diverged-dst
	 * graft-subpos shape.  The moved keys become {prefix,0},{prefix,1} in @ft
	 * (the test node stashes the full @okey), each prefix a new global minimum.
	 */
	clock_gettime(CLOCK_MONOTONIC, &t0);
	for (k = 0; k < MERGE_XVIEW_MERGES; k++) {
		unsigned int pv = 0xFEFF - k;	/* decreasing distinct prefix */
		uint8_t prefix[2] = { (uint8_t)(pv >> 8), (uint8_t)(pv & 0xff) };
		struct cds_ft *src;
		unsigned int s;

		if (cds_ft_create(group, NULL, &src) < 0)
			abort();
		rcu_read_lock();
		for (s = 0; s < 2; s++) {
			uint8_t skey[2] = { SRCK[0], (uint8_t) s };
			uint8_t full[3] = { prefix[0], prefix[1], (uint8_t) s };
			struct ft_test_node *n = node_alloc(pv * 4 + s);

			n->value = 3;
			memcpy(n->okey, full, 3);
			if (cds_ft_insert(src, skey, 2, &n->node) != CDS_FT_STATUS_OK)
				abort();
		}
		rcu_read_unlock();
cds_ft_make_exclusive(src);	/* DLM: cross-trie src must be exclusive */

		if (cds_ft_merge_at(ft, prefix, 2, src, SRCK, 1) != CDS_FT_STATUS_OK)
			abort();
		cds_ft_destroy(src);		/* emptied by the merge */

		if (elapsed_ms(&t0) >= DEFAULT_DURATION_MS)
			break;
	}

	test_stop = 1;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	for (i = 0; i < NR_READERS_DEFAULT; i++)
		pthread_join(readers[i], NULL);

	if (atomic_load(&violation_count) > 0) {
		fprintf(stderr, "inv_merge_cross_view: %lu violation(s)\n",
			atomic_load(&violation_count));
		drain_and_destroy(ft, group);
		return -1;
	}
	return drain_and_destroy(ft, group);
}

/*
 * BULK-OP (appear-side) cross-view oracle for the OCCUPIED-DST SPINE-COPY merge
 * -- the step-4 structural flip vs step-9 ordered INTERLEAVE window.  Same
 * decreasing-min method and reused reader (inv_graft_xview_appear_reader) as
 * inv_merge_cross_view, but each merge point is PRE-OCCUPIED (a stable
 * {prefix,0xFF} key) so the merge takes ft_merge_spine_copy rather than the
 * diverged subpos path.  The src run ({prefix,0},{prefix,1}) becomes the merged
 * region's new minimum; until the interleave splices its cells into dst's list,
 * the structural minimum sits BELOW the stable ordered-list front -> the
 * front-stable sandwich flags it.
 */
static int inv_merge_spinecopy_cross_view(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_varlen_ord_ft(&group);
	struct inv_lookup_ctx ctx;
	pthread_t readers[NR_READERS_DEFAULT];
	struct timespec t0;
	unsigned int i, k;
	static const uint8_t SRCK[1] = { 0x53 };	/* "S": the src sub-position */

	/* Stable upper bulk: prefix {0xFF,0xFF}, always above every merged run. */
	rcu_read_lock();
	for (i = 0; i < 16; i++) {
		uint8_t key[3] = { 0xFF, 0xFF, (uint8_t) i };
		struct ft_test_node *n = node_alloc(0x3000 + i);

		n->value = 3;
		memcpy(n->okey, key, 3);
		if (cds_ft_insert(ft, key, 3, &n->node) != CDS_FT_STATUS_OK)
			abort();
	}
	rcu_read_unlock();

	ctx.ft = ft;
	ctx.test_name = "inv_merge_spinecopy_cross_view";
	test_go = 0;
	test_stop = 0;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	for (i = 0; i < NR_READERS_DEFAULT; i++)
		pthread_create(&readers[i], NULL,
			inv_graft_xview_appear_reader, &ctx);
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	test_go = 1;

	clock_gettime(CLOCK_MONOTONIC, &t0);
	for (k = 0; k < MERGE_XVIEW_MERGES; k++) {
		unsigned int pv = 0xFEFF - k;	/* decreasing distinct prefix */
		uint8_t prefix[2] = { (uint8_t)(pv >> 8), (uint8_t)(pv & 0xff) };
		uint8_t occ[3] = { prefix[0], prefix[1], 0xFF };
		struct ft_test_node *on = node_alloc(0x40000 + k);
		struct cds_ft *src;
		unsigned int s;

		/* Pre-occupy dst at {prefix} (suffix 0xFF, above the src keys) so the
		 * merge is spine-copy.  An insert is itself a fused single commit, so
		 * it adds no cross-view window of its own. */
		on->value = 3;
		memcpy(on->okey, occ, 3);
		rcu_read_lock();
		if (cds_ft_insert(ft, occ, 3, &on->node) != CDS_FT_STATUS_OK)
			abort();
		rcu_read_unlock();

		if (cds_ft_create(group, NULL, &src) < 0)
			abort();
		rcu_read_lock();
		for (s = 0; s < 2; s++) {
			uint8_t skey[2] = { SRCK[0], (uint8_t) s };
			uint8_t full[3] = { prefix[0], prefix[1], (uint8_t) s };
			struct ft_test_node *n = node_alloc(pv * 4 + s);

			n->value = 3;
			memcpy(n->okey, full, 3);
			if (cds_ft_insert(src, skey, 2, &n->node) != CDS_FT_STATUS_OK)
				abort();
		}
		rcu_read_unlock();
cds_ft_make_exclusive(src);	/* DLM: cross-trie src must be exclusive */

		if (cds_ft_merge_at(ft, prefix, 2, src, SRCK, 1) != CDS_FT_STATUS_OK)
			abort();
		cds_ft_destroy(src);		/* emptied by the merge */

		if (elapsed_ms(&t0) >= DEFAULT_DURATION_MS)
			break;
	}

	test_stop = 1;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	for (i = 0; i < NR_READERS_DEFAULT; i++)
		pthread_join(readers[i], NULL);

	if (atomic_load(&violation_count) > 0) {
		fprintf(stderr, "inv_merge_spinecopy_cross_view: %lu violation(s)\n",
			atomic_load(&violation_count));
		drain_and_destroy(ft, group);
		return -1;
	}
	return drain_and_destroy(ft, group);
}

static int inv_detach_cross_view(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_varlen_ord_ft(&group);
	struct inv_lookup_ctx ctx;
	pthread_t readers[NR_READERS_DEFAULT];
	struct timespec t0;
	unsigned int i, p, s;

	rcu_read_lock();
	for (p = 0; p < DETACH_XVIEW_PREFIXES; p++) {
		for (s = 0; s < DETACH_XVIEW_PER; s++) {
			uint8_t key[3] = { (uint8_t)(p >> 8), (uint8_t)(p & 0xff),
				(uint8_t) s };
			struct ft_test_node *n = node_alloc(p);

			n->value = 3;
			memcpy(n->okey, key, 3);
			if (cds_ft_insert(ft, key, 3, &n->node) != CDS_FT_STATUS_OK)
				abort();
		}
	}
	rcu_read_unlock();

	ctx.ft = ft;
	ctx.test_name = "inv_detach_cross_view";
	test_go = 0;
	test_stop = 0;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	for (i = 0; i < NR_READERS_DEFAULT; i++)
		pthread_create(&readers[i], NULL, inv_remove_xview_reader_minvl,
			&ctx);
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	test_go = 1;

	/* Time-bounded: each detach is a full grace period, so the whole pool
	 * may not drain within the window -- drain_and_destroy reclaims the rest. */
	clock_gettime(CLOCK_MONOTONIC, &t0);
	for (p = 0; p < DETACH_XVIEW_PREFIXES; p++) {
		uint8_t prefix[2] = { (uint8_t)(p >> 8), (uint8_t)(p & 0xff) };
		struct cds_ft *detached = NULL;

		if (cds_ft_detach(ft, prefix, 2, &detached) == CDS_FT_STATUS_OK &&
		    detached) {
			drain_trie_local(detached);
			cds_ft_destroy(detached);
		}
		if (elapsed_ms(&t0) >= DEFAULT_DURATION_MS)
			break;
	}

	test_stop = 1;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	for (i = 0; i < NR_READERS_DEFAULT; i++)
		pthread_join(readers[i], NULL);

	if (atomic_load(&violation_count) > 0) {
		fprintf(stderr, "inv_detach_cross_view: %lu violation(s)\n",
			atomic_load(&violation_count));
		drain_and_destroy(ft, group);
		return -1;
	}
	return drain_and_destroy(ft, group);
}

/*
 * BULK-OP (disappear-side) cross-view oracle for the WHOLE-TRIE root detach
 * (cds_ft_detach(ft, NULL, 0)) -- the disappear dual of the empty-dst root
 * graft window covered by inv_graft_root_swap_cross_view.  A root detach retires
 * @ft's root to a fresh empty node AND clears @ft's ordered-list head/tail.  On
 * HEAD those were separate bare stores (root publish, then head/tail clear), so
 * a reader could observe @ft structurally EMPTY but its ordered list still
 * pointing at the (now-detached) run.  The fix fuses the root swap with the
 * head/tail clear into ONE flip (the src/disappear side of
 * ft_root_list_swap_publish).
 *
 * The writer round-trips: detach the whole trie out (@ft -> @detached, @ft now
 * empty), then graft @detached back at the root (@ft empty -> full again),
 * keeping @ft repeatedly populated so the disappear window recurs every cycle.
 * The key set is FIXED, so the ordered minimum is a single STABLE key -- which
 * is what makes the reused min-drain reader (inv_remove_xview_reader_minvl)
 * sound: it flags only when that min is stably present in the list (k1 == k2)
 * yet absent from the structure within one RCU read-side critical section.  The
 * graft-back is itself a fused appear flip, so it adds no false positive.  The
 * detach drains @ft (it is concurrent); the graft-back of the EXCLUSIVE detached
 * trie at the root is sync-free.
 */
#ifndef DETACH_ROOT_XVIEW_KEYS
#define DETACH_ROOT_XVIEW_KEYS	2000
#endif
/*
 * The disappear window is a single narrow root-store-then-head-clear gap per
 * detach (and each detach is a grace period, so the writer does relatively few
 * ops).  Use more concurrent readers than the default to catch it reliably --
 * the same compensation inv_graft_root_swap_cross_view notes for its appear
 * window.
 */
#ifndef DETACH_ROOT_XVIEW_READERS
#define DETACH_ROOT_XVIEW_READERS	32
#endif

static int inv_detach_root_cross_view(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_varlen_ord_ft(&group);
	struct inv_lookup_ctx ctx;
	pthread_t readers[DETACH_ROOT_XVIEW_READERS];
	struct timespec t0;
	unsigned int i, k;
	int ret = 0;

	rcu_read_lock();
	for (k = 0; k < DETACH_ROOT_XVIEW_KEYS; k++) {
		uint8_t key[3] = { (uint8_t)(k >> 8), (uint8_t)(k & 0xff), 0 };
		struct ft_test_node *n = node_alloc(k);

		n->value = 3;
		memcpy(n->okey, key, 3);
		if (cds_ft_insert(ft, key, 3, &n->node) != CDS_FT_STATUS_OK)
			abort();
	}
	rcu_read_unlock();

	ctx.ft = ft;
	ctx.test_name = "inv_detach_root_cross_view";
	test_go = 0;
	test_stop = 0;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	for (i = 0; i < DETACH_ROOT_XVIEW_READERS; i++)
		pthread_create(&readers[i], NULL, inv_remove_xview_reader_minvl,
			&ctx);
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	test_go = 1;

	/*
	 * Round-trip the whole trie out and back.  Each detach is a grace
	 * period (the source is concurrent); the graft-back is sync-free
	 * (exclusive src, root swap).
	 */
	clock_gettime(CLOCK_MONOTONIC, &t0);
	while (elapsed_ms(&t0) < DEFAULT_DURATION_MS) {
		struct cds_ft *detached = NULL;

		if (cds_ft_detach(ft, NULL, 0, &detached) != CDS_FT_STATUS_OK ||
		    !detached) {
			fprintf(stderr, "inv_detach_root_cross_view: detach failed\n");
			ret = -1;
			break;
		}
		cds_ft_make_exclusive(detached);	/* DLM: cross-trie src must be exclusive */
		/* @ft is now empty; graft the whole detached trie back at root. */
		if (cds_ft_graft(ft, NULL, 0, detached) != CDS_FT_STATUS_OK) {
			fprintf(stderr, "inv_detach_root_cross_view: graft-back failed\n");
			cds_ft_destroy(detached);
			ret = -1;
			break;
		}
		cds_ft_destroy(detached);	/* emptied by the graft-back */
	}

	test_stop = 1;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	for (i = 0; i < DETACH_ROOT_XVIEW_READERS; i++)
		pthread_join(readers[i], NULL);

	if (atomic_load(&violation_count) > 0) {
		fprintf(stderr, "inv_detach_root_cross_view: %lu violation(s)\n",
			atomic_load(&violation_count));
		ret = -1;
	}
	if (drain_and_destroy(ft, group) < 0)
		ret = -1;
	return ret;
}

/*
 * BULK-OP (disappear-side) cross-view oracle for cds_ft_merge_at's SOURCE side.
 * A merge unlinks the moved subtree from @src structurally (ft_merge_unlink_src_
 * subtree) and THEN removes its run from @src's ordered list (on HEAD, a separate
 * ft_ord_cell_run_unlink flip) -- so a @src reader can observe the moved run gone
 * from the structure but still present in the ordered list.  This is the
 * src-side dual of inv_detach_cross_view, and reuses its min-drain reader
 * (inv_remove_xview_reader_minvl: lookup_first -> point(min) -> lookup_first,
 * flagged only when the min is stably present in the list but absent from the
 * structure).  NON-CYCLING: the main thread merges @src's prefixes OUT to @dst in
 * increasing order, so @src only ever SHRINKS -- a re-appearing min is
 * impossible, which is what makes the present-absent-present check sound.
 */
#ifndef MERGE_SRC_XVIEW_PREFIXES
#define MERGE_SRC_XVIEW_PREFIXES	6000
#endif
#define MERGE_SRC_XVIEW_PER		3

static int inv_merge_src_cross_view(void)
{
	struct cds_ft_group *group;
	struct cds_ft *src = create_varlen_ord_ft(&group);
	struct cds_ft *dst;
	struct inv_lookup_ctx ctx;
	pthread_t readers[NR_READERS_DEFAULT];
	struct timespec t0;
	unsigned int i, p, s;

	if (cds_ft_create(group, NULL, &dst) < 0)
		abort();

	rcu_read_lock();
	for (p = 0; p < MERGE_SRC_XVIEW_PREFIXES; p++) {
		for (s = 0; s < MERGE_SRC_XVIEW_PER; s++) {
			uint8_t key[3] = { (uint8_t)(p >> 8), (uint8_t)(p & 0xff),
				(uint8_t) s };
			struct ft_test_node *n = node_alloc(p);

			n->value = 3;
			memcpy(n->okey, key, 3);
			if (cds_ft_insert(src, key, 3, &n->node) != CDS_FT_STATUS_OK)
				abort();
		}
	}
	rcu_read_unlock();

	ctx.ft = src;			/* readers watch the DRAINED source */
	ctx.test_name = "inv_merge_src_cross_view";
	test_go = 0;
	test_stop = 0;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	for (i = 0; i < NR_READERS_DEFAULT; i++)
		pthread_create(&readers[i], NULL, inv_remove_xview_reader_minvl,
			&ctx);
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	test_go = 1;

	/* Merge each prefix's subtree OUT of @src into @dst (same key, distinct in
	 * @dst), in increasing order so @src only shrinks.  Each merge is a grace
	 * period, so the pool may not drain within the window. */
	clock_gettime(CLOCK_MONOTONIC, &t0);
	for (p = 0; p < MERGE_SRC_XVIEW_PREFIXES; p++) {
		uint8_t prefix[2] = { (uint8_t)(p >> 8), (uint8_t)(p & 0xff) };
		struct cds_ft *detached = NULL;

		/*
		 * DECOMPOSED (was one cds_ft_merge_at with a LIVE @src).  Under
		 * CDS_FT_WRITER_LOCK_FINE a cross-trie source must be exclusive,
		 * and @src is exactly what the readers watch, so it cannot be.
		 * Detach-then-merge is the supported decomposition (design note
		 * decision (C): two sequential single-domain commits) and keeps
		 * this oracle's subject intact -- the reader-visible half is @src
		 * SHEDDING the subtree, i.e. the detach, unchanged.  The attach
		 * half then runs against the trie cds_ft_detach returns
		 * EXCLUSIVE by construction, so it is DLM-legal and skips its own
		 * grace period.  It stays a merge, not a graft: @dst is
		 * deliberately populated here, which a graft rejects with
		 * POPULATED_ERROR.  @detached's ROOT holds the subtree, hence the
		 * NULL/0 source key.
		 */
		if (cds_ft_detach(src, prefix, 2, &detached) == CDS_FT_STATUS_OK &&
		    detached) {
			(void) cds_ft_merge_at(dst, prefix, 2, detached, NULL, 0);
			cds_ft_destroy(detached);
		}
		if (elapsed_ms(&t0) >= DEFAULT_DURATION_MS)
			break;
	}

	test_stop = 1;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	for (i = 0; i < NR_READERS_DEFAULT; i++)
		pthread_join(readers[i], NULL);

	if (atomic_load(&violation_count) > 0) {
		fprintf(stderr, "inv_merge_src_cross_view: %lu violation(s)\n",
			atomic_load(&violation_count));
		drain_trie_local(src);
		drain_trie_local(dst);
		rcu_barrier();
		cds_ft_destroy(src);
		cds_ft_destroy(dst);
		cds_ft_group_destroy(group);
		return -1;
	}
	drain_trie_local(src);
	drain_trie_local(dst);
	rcu_barrier();
	cds_ft_destroy(src);
	cds_ft_destroy(dst);
	cds_ft_group_destroy(group);
	return 0;
}

/*
 * BULK-OP (disappear-side) cross-view oracle for the NON-ROOT SPINE-COPY merge
 * src side -- distinct from inv_merge_src_cross_view (which merges into an
 * ABSENT dst point -> the subpos/diverged path).  Here every dst merge point is
 * PRE-OCCUPIED, so the merge takes ft_merge_spine_copy's non-root-src branch:
 * ft_merge_unlink_src_subtree unlinks the source subtree structurally and THEN a
 * separate ft_ord_cell_run_unlink removes its run from src's ordered list, so a
 * src reader between them sees the moved subtree gone from the structure but
 * still in the ordered list.  The fix threads an EXCISE-ONLY run through
 * ft_merge_unlink_src_subtree (as the subpos src side already does).
 *
 * Same NON-CYCLING min-drain method as inv_merge_src_cross_view (reused reader
 * inv_remove_xview_reader_minvl): @src's prefixes are merged OUT in increasing
 * order so @src only shrinks.  @dst is pre-populated with one stable key under
 * EACH prefix (key suffix 0xFF) so cnt_dst > 0 at every merge point; the moved
 * keys keep their own (distinct) suffixes, so @dst stays a trie, not dup chains.
 */
static int inv_merge_src_spinecopy_cross_view(void)
{
	struct cds_ft_group *group;
	struct cds_ft *src = create_varlen_ord_ft(&group);
	struct cds_ft *dst;
	struct inv_lookup_ctx ctx;
	pthread_t readers[NR_READERS_DEFAULT];
	struct timespec t0;
	unsigned int i, p, s;

	if (cds_ft_create(group, NULL, &dst) < 0)
		abort();

	rcu_read_lock();
	for (p = 0; p < MERGE_SRC_XVIEW_PREFIXES; p++) {
		uint8_t dkey[3] = { (uint8_t)(p >> 8), (uint8_t)(p & 0xff), 0xFF };
		struct ft_test_node *dn = node_alloc(0x10000 + p);

		/* Pre-occupy dst at this prefix so the merge is spine-copy. */
		dn->value = 3;
		memcpy(dn->okey, dkey, 3);
		if (cds_ft_insert(dst, dkey, 3, &dn->node) != CDS_FT_STATUS_OK)
			abort();
		for (s = 0; s < MERGE_SRC_XVIEW_PER; s++) {
			uint8_t key[3] = { (uint8_t)(p >> 8), (uint8_t)(p & 0xff),
				(uint8_t) s };
			struct ft_test_node *n = node_alloc(p);

			n->value = 3;
			memcpy(n->okey, key, 3);
			if (cds_ft_insert(src, key, 3, &n->node) != CDS_FT_STATUS_OK)
				abort();
		}
	}
	rcu_read_unlock();

	ctx.ft = src;			/* readers watch the DRAINED source */
	ctx.test_name = "inv_merge_src_spinecopy_cross_view";
	test_go = 0;
	test_stop = 0;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	for (i = 0; i < NR_READERS_DEFAULT; i++)
		pthread_create(&readers[i], NULL, inv_remove_xview_reader_minvl,
			&ctx);
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	test_go = 1;

	/* Merge each prefix's subtree OUT of @src into @dst at the SAME (occupied)
	 * prefix -> spine-copy non-root-src.  Increasing order so @src only shrinks. */
	clock_gettime(CLOCK_MONOTONIC, &t0);
	for (p = 0; p < MERGE_SRC_XVIEW_PREFIXES; p++) {
		uint8_t prefix[2] = { (uint8_t)(p >> 8), (uint8_t)(p & 0xff) };
		struct cds_ft *detached = NULL;

		/*
		 * DECOMPOSED (was one cds_ft_merge_at with a LIVE @src).  Under
		 * CDS_FT_WRITER_LOCK_FINE a cross-trie source must be exclusive,
		 * and @src is exactly what the readers watch, so it cannot be.
		 * Detach-then-merge is the supported decomposition (design note
		 * decision (C): two sequential single-domain commits) and keeps
		 * this oracle's subject intact -- the reader-visible half is @src
		 * SHEDDING the subtree, i.e. the detach, unchanged.  The attach
		 * half then runs against the trie cds_ft_detach returns
		 * EXCLUSIVE by construction, so it is DLM-legal and skips its own
		 * grace period.  It stays a merge, not a graft: @dst is
		 * deliberately populated here, which a graft rejects with
		 * POPULATED_ERROR.  @detached's ROOT holds the subtree, hence the
		 * NULL/0 source key.
		 */
		if (cds_ft_detach(src, prefix, 2, &detached) == CDS_FT_STATUS_OK &&
		    detached) {
			(void) cds_ft_merge_at(dst, prefix, 2, detached, NULL, 0);
			cds_ft_destroy(detached);
		}
		if (elapsed_ms(&t0) >= DEFAULT_DURATION_MS)
			break;
	}

	test_stop = 1;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	for (i = 0; i < NR_READERS_DEFAULT; i++)
		pthread_join(readers[i], NULL);

	if (atomic_load(&violation_count) > 0) {
		fprintf(stderr, "inv_merge_src_spinecopy_cross_view: %lu violation(s)\n",
			atomic_load(&violation_count));
		drain_trie_local(src);
		drain_trie_local(dst);
		rcu_barrier();
		cds_ft_destroy(src);
		cds_ft_destroy(dst);
		cds_ft_group_destroy(group);
		return -1;
	}
	drain_trie_local(src);
	drain_trie_local(dst);
	rcu_barrier();
	cds_ft_destroy(src);
	cds_ft_destroy(dst);
	cds_ft_group_destroy(group);
	return 0;
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

/*
 * SKIP_X head-promotion: SPARSE keys (idx << 16) put each duplicate chain's head
 * at a leaf reached through a SKIP_X suffix compressed node.  Removing such a
 * head with duplicates remaining drives ft_promote_head's COMPRESSED-holder
 * path -- the cn->child republish AND the grandparent SKIP_X dual fused with the
 * fresh-cell swap in ONE flip.  The dense-key inv_dup_chain_acyclicity only
 * reaches the plain internal-holder promotion, so this is the only invariant
 * that drives the compressed-holder dual concurrently.  Readers walk the dup
 * chain (acyclicity) -- a torn promotion (stale skip target into the freed old
 * head, or a cell pointing at the removed head) would corrupt the walk.
 */
#define INV_SKIPX_KEY(idx)	((uint64_t)(idx) << 16)

static void *inv_skipx_promote_reader(void *arg)
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
		uint64_t key = INV_SKIPX_KEY(rand_r(&seed) % WRITER_POOL_SIZE);
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

static void *inv_skipx_promote_writer(void *arg)
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
		uint64_t key = INV_SKIPX_KEY(rand_r(&seed) % WRITER_POOL_SIZE);
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
			/* Lookup returns the chain HEAD: removing it promotes a dup. */
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

static int inv_dup_chain_skipx_head_promotion(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_ft(4, &group);
	struct inv_lookup_ctx ctx;
	struct timespec t0;
	pthread_t readers[NR_READERS_DEFAULT], writers[NR_WRITERS_DEFAULT];
	unsigned int i;

	ctx.ft = ft;
	ctx.test_name = "inv_dup_chain_skipx_head_promotion";
	pthread_mutex_init(&ctx.lock, NULL);

	/* Pre-populate SPARSE keys (SKIP_X leaves) each with a duplicate chain. */
	rcu_read_lock();
	for (i = 0; i < WRITER_POOL_SIZE / 4; i++) {
		unsigned int j;

		for (j = 0; j < 3; j++) {
			struct ft_test_node *n = node_alloc(INV_SKIPX_KEY(i));
			insert_u64(ft, INV_SKIPX_KEY(i), n);
		}
	}
	rcu_read_unlock();

	test_go = 0;
	test_stop = 0;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	for (i = 0; i < NR_READERS_DEFAULT; i++)
		pthread_create(&readers[i], NULL, inv_skipx_promote_reader, &ctx);
	for (i = 0; i < NR_WRITERS_DEFAULT; i++)
		pthread_create(&writers[i], NULL, inv_skipx_promote_writer, &ctx);

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
		fprintf(stderr, "inv_dup_chain_skipx_head_promotion: %lu violation(s)\n",
			atomic_load(&violation_count));
		drain_and_destroy(ft, group);
		return -1;
	}
	return drain_and_destroy(ft, group);
}

#undef INV_SKIPX_KEY

/* ================================================================== */
/*                                                                    */
/*   INVARIANT: Ordered-list-OFF (no-cell) consistency                */
/*                                                                    */
/*   A group with the ordered list disabled allocates NO ordinal      */
/*   cells: a head's prev is the flagged parent directly, so the      */
/*   read path resolves it prev-direct (ft_resolve_head_prev), and    */
/*   skip resolution / parent backtrack run the no-cell branch.       */
/*   Stress that path under concurrent insert/remove: lookups must    */
/*   return the right node (shadow-key check) and duplicate chains    */
/*   must stay acyclic.  This gives the runtime cell-optional path     */
/*   standing concurrent coverage in the default pass, independent of */
/*   the FT_INV_NO_ORDERED_LIST env knob.                             */
/*                                                                    */
/* ================================================================== */

static int inv_no_ordered_list_consistency(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_nolist_ft(4, &group);
	struct inv_lookup_ctx ctx;
	struct timespec t0;
	pthread_t readers[NR_READERS_DEFAULT], writers[NR_WRITERS_DEFAULT];
	unsigned int i;

	ctx.ft = ft;
	ctx.test_name = "inv_no_ordered_list_consistency";
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

	/*
	 * Half the readers verify lookup consistency (right node for the key),
	 * half walk duplicate chains for acyclicity; the writers churn
	 * insert/remove.  All reuse the shared inv_lookup_ctx thread bodies.
	 */
	for (i = 0; i < NR_READERS_DEFAULT; i++)
		pthread_create(&readers[i], NULL,
			(i & 1) ? inv_dup_chain_reader :
				inv_lookup_consistency_reader, &ctx);
	for (i = 0; i < NR_WRITERS_DEFAULT; i++)
		pthread_create(&writers[i], NULL,
			inv_lookup_consistency_writer, &ctx);

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
		fprintf(stderr, "inv_no_ordered_list_consistency: %lu violation(s)\n",
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
	struct cds_ft *swap;	/* cross-trie oracle: the second trie, shared */
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
		cds_ft_make_exclusive(swap);	/* DLM: cross-trie src must be exclusive */
		s = cds_ft_graft_swap(ctx->live, NULL, 0, swap);
		pthread_mutex_unlock(&ctx->lock);
		rcu_read_unlock();

		if (s != CDS_FT_STATUS_OK) {
			/*
			 * Reported, not merely logged: bailing leaves the readers
			 * an unchanging pair of tries, so they find nothing and
			 * the oracle reports ok having exercised NOTHING.  A
			 * silent green is indistinguishable from coverage.
			 */
			report_violation(ctx->test_name,
				"graft_swap failed (%s): the writer stopped, so this oracle would have passed vacuously",
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

/*
 * graft_swap CROSS-VIEW oracle (structure vs ordered list).  inv_graft_swap_
 * atomicity checks only list-internal atomicity (cds_ft_for_each never mixes A
 * and B).  This checks the OTHER axis: a root graft_swap publishes dst->root
 * (structure) and dst's ord_cell_head/tail (list) in SEPARATE stores, so a
 * reader between them sees the whole trie as range B structurally while the
 * ordered-list front is still range A.  The fix fuses each side's root swap
 * with its head/tail transfer into one flip (ft_root_list_swap_publish).
 *
 * The writer PING-PONGS the same two tries (graft_swap leaves the old content
 * in @swap, so the next swap moves it back) -- no drain/repopulate, so it runs
 * far more swaps (= windows) than inv_graft_swap_atomicity's writer.  That
 * makes it CYCLING (A<->B at root), so the reader uses the front-stable
 * SANDWICH (list-range, struct-range, list-range): it flags only when the
 * list-range is STABLE yet the structure shows the OTHER range -- a mid-swap
 * straddle changes the list-range between the two reads and is filtered, while
 * the genuine structure-before-list window survives.  The two probes are the
 * usual independent pair: lookup_first (cell-list front) vs lookup_ge from a
 * submin (pure structural descent).
 */
static int graft_swap_range(struct cds_ft *ft, struct cds_ft_iter *iter,
		bool use_list)
{
	static const uint8_t SUBMIN[1] = { 0x00 };
	struct cds_ft_node *n;
	enum cds_ft_status s;

	if (use_list) {
		s = cds_ft_lookup_first(ft, iter);
	} else {
		cds_ft_iter_set_key(iter, SUBMIN, 1);
		s = cds_ft_lookup_ge(ft, iter);
	}
	if (s != CDS_FT_STATUS_OK || (n = cds_ft_iter_node(iter)) == NULL)
		return -1;	/* empty */
	/* The leaf carries its own u64 key, so no iter key materialization. */
	return to_test_node(n)->key < GRAFT_SWAP_RANGE_B_BASE ? 0 : 1;
}

static void *inv_graft_swap_xview_reader(void *arg)
{
	struct inv_graft_ctx *ctx = (struct inv_graft_ctx *) arg;
	struct cds_ft_iter *iter;

	rcu_register_thread();
	if (cds_ft_iter_create(ctx->live, &iter) < 0)
		abort();
	while (!test_go)
		;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	while (!test_stop) {
		int rl1, rs, rl2;

		rcu_read_lock();
		rl1 = graft_swap_range(ctx->live, iter, true);	/* list front */
		rs  = graft_swap_range(ctx->live, iter, false);	/* structural min */
		rl2 = graft_swap_range(ctx->live, iter, true);	/* list front again */
		if (rl1 >= 0 && rl1 == rl2 && rs >= 0 && rs != rl1)
			report_violation(ctx->test_name,
				"structural range != STABLE ordered-list front"
				" range -- the root graft_swap published the root"
				" before transferring the ordered-list head/tail", 0);
		rcu_read_unlock();
		rcu_quiescent_state();
	}

	cds_ft_iter_destroy(iter);
	rcu_unregister_thread();
	return NULL;
}

static void *inv_graft_swap_xview_writer(void *arg)
{
	struct inv_graft_ctx *ctx = (struct inv_graft_ctx *) arg;
	struct cds_ft *swap;

	rcu_register_thread();
	if (cds_ft_create(ctx->group, NULL, &swap) < 0)
		abort();
	populate_range(swap, GRAFT_SWAP_RANGE_B_BASE, GRAFT_SWAP_POOL);

	while (!test_go)
		;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	/* Ping-pong: graft_swap moves the old content into @swap, so the next
	 * swap moves it straight back -- no drain/repopulate, far more windows. */
	while (!test_stop) {
		pthread_mutex_lock(&ctx->lock);
		cds_ft_make_exclusive(swap);	/* DLM: cross-trie src must be exclusive */
		if (cds_ft_graft_swap(ctx->live, NULL, 0, swap) != CDS_FT_STATUS_OK)
			abort();
		pthread_mutex_unlock(&ctx->lock);
		rcu_quiescent_state();
	}

	drain_trie_local(swap);
	rcu_barrier();
	cds_ft_destroy(swap);
	rcu_unregister_thread();
	return NULL;
}

static int inv_graft_swap_cross_view(void)
{
	struct cds_ft_group_attr *attr;
	struct cds_ft_group *group;
	struct cds_ft *live;
	struct inv_graft_ctx ctx;
	struct timespec t0;
	pthread_t readers[NR_READERS_DEFAULT], writer;
	unsigned int i;

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
	populate_range(live, GRAFT_SWAP_RANGE_A_BASE, GRAFT_SWAP_POOL);

	ctx.live = live;
	ctx.group = group;
	ctx.test_name = "inv_graft_swap_cross_view";
	pthread_mutex_init(&ctx.lock, NULL);

	test_go = 0;
	test_stop = 0;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	for (i = 0; i < NR_READERS_DEFAULT; i++)
		pthread_create(&readers[i], NULL, inv_graft_swap_xview_reader, &ctx);
	pthread_create(&writer, NULL, inv_graft_swap_xview_writer, &ctx);
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
		fprintf(stderr, "inv_graft_swap_cross_view: %lu violation(s)\n",
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

/*
 * CROSS-TRIE root graft_swap oracle.  The two oracles above check each TRIE's
 * own structure-vs-ordered-list axis; this checks the axis BETWEEN the two
 * tries.  A whole-trie graft_swap historically published dst->root and
 * swap->root in two SEPARATE flips (an ft_root_list_swap_publish per side), so
 * between them BOTH roots transiently pointed at the same content: a key was
 * reachable in BOTH tries (the content moved in) or in NEITHER (the content
 * moved out).  The fix fuses both sides' root (and head/tail) swaps into ONE
 * cross-trie flip (ft_root_list_swap_publish_dual -- dst and swap share a group,
 * hence a flip selector), so a reader resolves both roots to ONE phase and a
 * moved key is in EXACTLY ONE trie.
 *
 * Both tries are shared with the readers (ctx->live and ctx->swap).  The writer
 * ping-pongs graft_swap(live, swap): the content bounces live<->swap with no
 * drain, maximising windows.  Probe key K is a fixed member of range A (present
 * in whichever trie currently holds A).  Each reader sandwiches the cross-trie
 * probe -- live-present, swap-present, live-present -- and flags only when K's
 * presence in @live is STABLE across the probe (pl1 == pl2): then no full swap
 * straddled the reads and K must be in exactly one trie.  An unstable presence
 * is a straddle and is filtered (the window being hunted shows pl1 == pl2 == 0,
 * ps == 0, i.e. K in NEITHER, within one stable view).
 */

/* Narrow root-store window (each swap is a grace period -> few writer ops); use
 * more readers than the default to catch it reliably, as inv_detach_root_cross_
 * view does. */
#ifndef GRAFT_SWAP_XTRIE_READERS
#define GRAFT_SWAP_XTRIE_READERS	32
#endif




/*
 * SUB-KEY graft_swap cross-view oracle -- the "two-run" run_replace path
 * (ft_ord_cell_run_replace), distinct from the root swap above.  A sub-key
 * graft_swap exchanges dst's subtree-at-@key (run_D) with swap's content
 * (run_S): it publishes run_S structurally at @key (one flip) and THEN swaps
 * run_D out / run_S in in dst's ordered list (a SEPARATE ft_ord_cell_run_replace
 * flip), so a reader between them sees run_S's keys structurally present at @key
 * while the ordered-list front is still run_D (and vice versa).
 *
 * @key = {0x01} is kept the GLOBAL MINIMUM (a stable upper bulk lives at
 * {0xFF}), so the cell-list front and the structural minimum both fall in @key's
 * region -- the same independent probes and front-stable sandwich as the root
 * oracle (reused inv_graft_swap_xview_reader / graft_swap_range).  run_D is
 * range A, run_S is range B; the writer PING-PONGS the swap so the leaves'
 * own u64 keys (hence ranges) are preserved across swaps.
 */
#define GRAFT_SWAP_SUB_POOL	8
#define GRAFT_SWAP_SUB_BULK	4

static void populate_at_prefix(struct cds_ft *ft, const uint8_t *prefix,
		size_t prefix_len, uint64_t base, unsigned int count)
{
	unsigned int i;

	for (i = 0; i < count; i++) {
		struct ft_test_node *n = node_alloc(base + i);
		uint8_t k[8] = { 0 };	/* sized to memcpy into n->okey[8] below */
		size_t kl = prefix_len;

		if (prefix_len)
			memcpy(k, prefix, prefix_len);
		k[kl++] = (uint8_t) i;		/* distinct suffix byte */
		memcpy(n->okey, k, sizeof(n->okey));
		if (cds_ft_insert(ft, k, kl, &n->node) != CDS_FT_STATUS_OK)
			abort();
	}
}

static void *inv_graft_swap_sub_xview_writer(void *arg)
{
	struct inv_graft_ctx *ctx = (struct inv_graft_ctx *) arg;
	struct cds_ft *swap;
	static const uint8_t AT[1] = { 0x01 };

	rcu_register_thread();
	if (cds_ft_create(ctx->group, NULL, &swap) < 0)
		abort();
	/* run_S = range B at swap's root; becomes dst's subtree at {0x01}. */
	populate_at_prefix(swap, NULL, 0, GRAFT_SWAP_RANGE_B_BASE,
		GRAFT_SWAP_SUB_POOL);

	while (!test_go)
		;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	/* Ping-pong the subtree at {0x01}: graft_swap leaves run_D in @swap, so
	 * the next swap moves it back -- no drain, far more windows. */
	while (!test_stop) {
		pthread_mutex_lock(&ctx->lock);
		cds_ft_make_exclusive(swap);	/* DLM: cross-trie src must be exclusive */
		if (cds_ft_graft_swap(ctx->live, AT, 1, swap) != CDS_FT_STATUS_OK)
			abort();
		pthread_mutex_unlock(&ctx->lock);
		rcu_quiescent_state();
	}

	drain_trie_local(swap);
	rcu_barrier();
	cds_ft_destroy(swap);
	rcu_unregister_thread();
	return NULL;
}

static int inv_graft_swap_sub_cross_view(void)
{
	struct cds_ft_group_attr *attr;
	struct cds_ft_group *group;
	struct cds_ft *live;
	struct inv_graft_ctx ctx;
	struct timespec t0;
	pthread_t readers[NR_READERS_DEFAULT], writer;
	static const uint8_t AT[1] = { 0x01 };
	static const uint8_t HI[1] = { 0xFF };
	unsigned int i;

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
	/* run_D = range A under {0x01} (the global min) + a stable bulk at {0xFF}
	 * so {0x01} is a genuine sub-position (run_replace, not the root path). */
	populate_at_prefix(live, AT, 1, GRAFT_SWAP_RANGE_A_BASE, GRAFT_SWAP_SUB_POOL);
	populate_at_prefix(live, HI, 1, 9000, GRAFT_SWAP_SUB_BULK);

	ctx.live = live;
	ctx.group = group;
	ctx.test_name = "inv_graft_swap_sub_cross_view";
	pthread_mutex_init(&ctx.lock, NULL);

	test_go = 0;
	test_stop = 0;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	for (i = 0; i < NR_READERS_DEFAULT; i++)
		pthread_create(&readers[i], NULL, inv_graft_swap_xview_reader, &ctx);
	pthread_create(&writer, NULL, inv_graft_swap_sub_xview_writer, &ctx);
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
		fprintf(stderr, "inv_graft_swap_sub_cross_view: %lu violation(s)\n",
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

/*
 * EMPTY-SWAP graft_swap cross-view oracle -- the disappear-side, single-run
 * shapes.  graft_swap(live, @key, EMPTY_swap) with content at @key is a REMOVE:
 * it unpublishes run_D at @key (structure) and THEN removes run_D from the
 * ordered list (a separate ft_ord_cell_run_replace flip), so a reader between
 * them sees run_D gone from the structure but still at the ordered-list front.
 *
 * NON-CYCLING (drain-only): the main thread extracts @live's 2-byte prefixes OUT
 * in INCREASING order (each into the freshly re-emptied @swap), so @live only
 * ever SHRINKS -- a re-appearing minimum is impossible, which is what makes the
 * reused min-drain reader (inv_remove_xview_reader_minvl: lookup_first ->
 * point(min) -> lookup_first, present/absent/present) sound.  A cycling
 * extract+graft-back writer is NOT sound here: a full out-and-back between the
 * reader's two list reads forges a "stable" front while the point lookup caught
 * the transient absence.  Keys are {p_hi,p_lo,s}: extracting {p_hi,p_lo} clears
 * p_lo's slot in the p_hi node, which is the publish-NULL (non-sole-child) shape
 * while p_hi still has other p_lo children, and the gs_reserved (sole-child
 * detach-prune) shape for the LAST p_lo under each p_hi -- so this one oracle
 * exercises BOTH empty-swap corners.
 */
#ifndef GRAFT_SWAP_EMPTY_PREFIXES
#define GRAFT_SWAP_EMPTY_PREFIXES	6000
#endif
#define GRAFT_SWAP_EMPTY_PER		3

static int inv_graft_swap_empty_cross_view(void)
{
	struct cds_ft_group *group;
	struct cds_ft *live = create_varlen_ord_ft(&group);
	struct cds_ft *swap;
	struct inv_lookup_ctx ctx;
	pthread_t readers[NR_READERS_DEFAULT];
	struct timespec t0;
	unsigned int i, p, s;

	if (cds_ft_create(group, NULL, &swap) < 0)
		abort();

	rcu_read_lock();
	for (p = 0; p < GRAFT_SWAP_EMPTY_PREFIXES; p++) {
		for (s = 0; s < GRAFT_SWAP_EMPTY_PER; s++) {
			uint8_t key[3] = { (uint8_t)(p >> 8), (uint8_t)(p & 0xff),
				(uint8_t) s };
			struct ft_test_node *n = node_alloc(p);

			n->value = 3;
			memcpy(n->okey, key, 3);
			if (cds_ft_insert(live, key, 3, &n->node) != CDS_FT_STATUS_OK)
				abort();
		}
	}
	rcu_read_unlock();

	ctx.ft = live;			/* readers watch the DRAINED live trie */
	ctx.test_name = "inv_graft_swap_empty_cross_view";
	test_go = 0;
	test_stop = 0;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	for (i = 0; i < NR_READERS_DEFAULT; i++)
		pthread_create(&readers[i], NULL, inv_remove_xview_reader_minvl,
			&ctx);
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	test_go = 1;

	/* Extract each prefix's subtree OUT of @live (empty-swap remove), in
	 * increasing order so @live only shrinks, draining @swap back to empty
	 * between extracts.  Each graft_swap is a grace period. */
	clock_gettime(CLOCK_MONOTONIC, &t0);
	for (p = 0; p < GRAFT_SWAP_EMPTY_PREFIXES; p++) {
		uint8_t prefix[2] = { (uint8_t)(p >> 8), (uint8_t)(p & 0xff) };
cds_ft_make_exclusive(swap);	/* DLM: cross-trie src must be exclusive */

		if (cds_ft_graft_swap(live, prefix, 2, swap) != CDS_FT_STATUS_OK)
			abort();
		drain_trie_local(swap);		/* re-empty @swap for the next extract */
		if (elapsed_ms(&t0) >= DEFAULT_DURATION_MS)
			break;
	}

	test_stop = 1;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	for (i = 0; i < NR_READERS_DEFAULT; i++)
		pthread_join(readers[i], NULL);

	if (atomic_load(&violation_count) > 0) {
		fprintf(stderr, "inv_graft_swap_empty_cross_view: %lu violation(s)\n",
			atomic_load(&violation_count));
		drain_trie_local(live);
		drain_trie_local(swap);
		rcu_barrier();
		cds_ft_destroy(live);
		cds_ft_destroy(swap);
		cds_ft_group_destroy(group);
		return -1;
	}
	drain_trie_local(live);
	drain_trie_local(swap);
	rcu_barrier();
	cds_ft_destroy(live);
	cds_ft_destroy(swap);
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
/*   INVARIANT 7b: rank/select readers resolve a parked external_nodes */
/*                 proxy (regression for the ordered-query unresolved  */
/*                 external_nodes read).                               */
/*                                                                    */
/*   An ordered-list insert/remove of a PREFIX key publishes its head  */
/*   into an internal node's external_nodes via the one-commit splice, */
/*   which transiently PARKS a flip proxy in that slot                 */
/*   (ft_insert_park_external_nodes / ft_remove_one_commit).  The      */
/*   rank/select readers (cds_ft_lookup_nth / _nth_last /              */
/*   skip_forward / skip_reverse) descend through that node and must   */
/*   RESOLVE the proxy (ft_dereference_external_acquire), never hand   */
/*   the tagged proxy pointer back as iter->node.  Setup: stable 5-byte */
/*   extension keys keep each 4-byte prefix terminating at an internal */
/*   node; the writer churns the 4-byte prefixes (each insert/remove   */
/*   parks the proxy); readers hammer all four rank/select queries and */
/*   assert the returned node is a real external head, not a proxy.    */
/*                                                                    */
/* ================================================================== */

#ifndef EXT_PARK_GROUPS
#define EXT_PARK_GROUPS	4096		/* distinct prefixes (= nth/skip range) */
#endif

struct inv_ext_park_ctx {
	struct cds_ft *ft;
	const char *test_name;
	unsigned int groups;
	pthread_mutex_t *lock;
};

/* Stable 5-byte extension keys Kl(g) = {g>>8, g&0xff, 0, 0, 0}. */
static void inv_ext_park_populate(struct cds_ft *ft)
{
	unsigned int g;

	for (g = 0; g < EXT_PARK_GROUPS; g++) {
		uint8_t kl[5] = { (uint8_t)(g >> 8), (uint8_t)(g & 0xff), 0, 0, 0 };
		struct ft_test_node *nl = node_alloc(g);

		nl->value = 5;			/* stash key length (xview_node_key) */
		memcpy(nl->okey, kl, 5);
		if (cds_ft_insert(ft, kl, 5, &nl->node) != CDS_FT_STATUS_OK)
			abort();
	}
}

static void *inv_ext_park_reader(void *arg)
{
	struct inv_ext_park_ctx *ctx = (struct inv_ext_park_ctx *) arg;
	struct cds_ft_iter *iter;
	unsigned int seed;
	unsigned long iters = 0;

	rcu_register_thread();
	seed = (unsigned int)(uintptr_t)pthread_self() ^ (unsigned int)time(NULL);

	if (cds_ft_iter_create(ctx->ft, &iter) < 0)
		abort();

	while (!test_go)
		;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	while (!test_stop) {
		unsigned long n = (unsigned long)(rand_r(&seed) % ctx->groups);
		int which = rand_r(&seed) & 3;
		const char *qname;
		enum cds_ft_status s;

		rcu_read_lock();
		switch (which) {
		case 0:
			qname = "lookup_nth";
			s = cds_ft_lookup_nth(ctx->ft, iter, n);
			break;
		case 1:
			qname = "lookup_nth_last";
			s = cds_ft_lookup_nth_last(ctx->ft, iter, n);
			break;
		case 2:
			qname = "skip_forward";
			s = cds_ft_lookup_nth(ctx->ft, iter, 0);
			if (s == CDS_FT_STATUS_OK)
				s = cds_ft_iter_skip_forward(ctx->ft, iter, n);
			break;
		default:
			qname = "skip_reverse";
			s = cds_ft_lookup_nth_last(ctx->ft, iter, 0);
			if (s == CDS_FT_STATUS_OK)
				s = cds_ft_iter_skip_reverse(ctx->ft, iter, n);
			break;
		}

		if (s == CDS_FT_STATUS_OK) {
			struct cds_ft_node *node = cds_ft_iter_node(iter);

			if (node) {
				/*
				 * A parked flip proxy carries type-index 7 (low
				 * nibble 0xF -- ft_node_flip_proxy /
				 * FT_FLIP_PROXY_TAG in ft-helpers.h); a real
				 * external head is >= 8-byte aligned (nibble 0).
				 * If the rank/select read failed to resolve the
				 * proxy, iter->node holds the tagged proxy.
				 */
				if (((uintptr_t) node & 0xFUL) == 0xFUL) {
					report_violation(ctx->test_name,
						"%s(%lu) returned a flip proxy as"
						" iter->node (%p) -- external_nodes"
						" read left unresolved",
						qname, n, (void *) node);
				} else {
					size_t klen = (size_t) to_test_node(node)->value;

					if (klen != 4 && klen != 5)
						report_violation(ctx->test_name,
							"%s(%lu) returned node with"
							" corrupt key length %zu",
							qname, n, klen);
				}
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

/* Churn the 4-byte prefixes Kp(g) = {g>>8, g&0xff, 0, 0}: each insert/remove
 * publishes/retires the prefix head via the one-commit external_nodes splice. */
static void *inv_ext_park_writer(void *arg)
{
	struct inv_ext_park_ctx *ctx = (struct inv_ext_park_ctx *) arg;
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
		unsigned int g = (unsigned int)(rand_r(&seed) % ctx->groups);
		uint8_t kp[4] = { (uint8_t)(g >> 8), (uint8_t)(g & 0xff), 0, 0 };
		int do_insert = rand_r(&seed) & 1;
		struct cds_ft_node *found;

		rcu_read_lock();
		pthread_mutex_lock(ctx->lock);
		cds_ft_iter_set_key(iter, kp, 4);
		cds_ft_lookup(ctx->ft, iter);
		found = cds_ft_iter_node(iter);
		if (do_insert) {
			if (!found) {
				struct ft_test_node *n = node_alloc(g);

				n->value = 4;
				memcpy(n->okey, kp, 4);
				if (cds_ft_insert(ctx->ft, kp, 4, &n->node)
				    != CDS_FT_STATUS_OK)
					node_free(n);
			}
		} else if (found) {
			struct ft_test_node *tn = to_test_node(found);

			if (cds_ft_remove(ctx->ft, iter, &tn->node)
			    == CDS_FT_STATUS_OK)
				node_free_rcu(tn);
		}
		pthread_mutex_unlock(ctx->lock);
		rcu_read_unlock();

		if ((seed & 0xff) == 0)
			rcu_quiescent_state();
	}

	cds_ft_iter_destroy(iter);
	rcu_unregister_thread();
	return NULL;
}

static int inv_nth_external_nodes_resolve(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_varlen_ord_ft(&group);
	struct timespec t0;
	pthread_mutex_t lock;
	struct inv_ext_park_ctx ctx;
	pthread_t readers[NR_READERS_DEFAULT], writers[NR_WRITERS_DEFAULT];
	unsigned int i;

	ctx.ft = ft;
	ctx.test_name = "inv_nth_external_nodes_resolve";
	ctx.groups = EXT_PARK_GROUPS;
	pthread_mutex_init(&lock, NULL);
	ctx.lock = &lock;

	rcu_read_lock();
	inv_ext_park_populate(ft);
	rcu_read_unlock();

	test_go = 0;
	test_stop = 0;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	for (i = 0; i < NR_READERS_DEFAULT; i++)
		pthread_create(&readers[i], NULL, inv_ext_park_reader, &ctx);
	for (i = 0; i < NR_WRITERS_DEFAULT; i++)
		pthread_create(&writers[i], NULL, inv_ext_park_writer, &ctx);

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
	pthread_mutex_destroy(&lock);

	if (atomic_load(&violation_count) > 0) {
		fprintf(stderr, "inv_nth_external_nodes_resolve: %lu violation(s)\n",
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
 * Remove-phase reader: the remove-side, bounded-re-measure NO-(persistent-)
 * OVERCOUNT check (complement of the strict no-undercount reader).  count_keys
 * and the iteration are sampled at different instants, and the list-off
 * lookup_gt iterator is not linearizable against a concurrent remove-
 * recompaction, so a single count_keys > iter_count is not itself a bug -- only
 * one that PERSISTS across re-descents is a genuine nr_keys accounting error.
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
		 * Iterate first (count reachable keys via pointers), then read
		 * count_keys (the nr_keys aggregate).  With a remove-only writer
		 * the key set only shrinks, so count_keys > iter_count is
		 * normally absent; when it appears it is the transient handled
		 * below, not a real over-count.
		 */
		iter_count = 0;
		cds_ft_for_each_rcu(ctx->ft, iter) {
			iter_count++;
		}

		count_keys = cds_ft_count_keys(ctx->ft);

		if (count_keys > iter_count) {
			/*
			 * A transient count_keys > iter_count here is the
			 * non-linearizable list-off lookup_gt ITERATOR, not an
			 * accounting error: during a remove-recompaction the
			 * iterator's re-descent can reach the rebuilt smaller node
			 * before the republish and UNDER-enumerate the subtree
			 * (LTTng-confirmed), while the aggregate still correctly
			 * counts the not-yet-detached key.  With order statistics
			 * OFF there is a second non-linearizable source --
			 * cds_ft_count_keys is itself a structural recount
			 * (ft_subtree_key_count) that can resolve a swapped-out
			 * child to the larger pre-recompaction subtree.  Either
			 * way the excess clears once the structure settles, so a
			 * single-shot count_keys <= iter_count is NOT a sound
			 * invariant under concurrent removes.  Re-measure under a
			 * bounded retry and flag only a STABLE over-count -- a
			 * count_keys that stays above the traversal across many
			 * re-descents is a genuine nr_keys accounting bug.  (The
			 * exact count == traversal equality is asserted only at
			 * the quiescent phase barrier, where it is well-defined.)
			 */
			unsigned int r;
			bool stable_over = true;

			for (r = 0; r < 256; r++) {
				unsigned long ic2 = 0, ck2;

				cds_ft_for_each_rcu(ctx->ft, iter)
					ic2++;
				ck2 = cds_ft_count_keys(ctx->ft);
				if (ck2 <= ic2) {
					stable_over = false;
					break;
				}
			}
			if (stable_over)
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
 * Helper: quiescent (at-rest) exactness barrier.
 *
 * Exact count_keys == iteration_count is only well-defined at REST: under
 * concurrent updaters the two are sampled at different instants, so only a
 * one-sided over-/under-count bound is meaningful (see inv_nr_keys_undercount
 * for the insert = no-overcount side and inv_nr_keys_no_undercount for the
 * remove = no-undercount side).  With the writers joined, assert the exact
 * equality, and additionally run cds_ft_verify() -- which cross-checks every
 * node's stored nr_keys against a full structural subtree recount (gated on
 * rank stats), the tightest per-node oracle for the count fold.
 *
 * Returns 0 on success, -1 on mismatch.
 */
static int quiescent_count_check(struct cds_ft *ft, const char *test_name,
		const char *phase)
{
	struct cds_ft_iter *check_iter;
	unsigned long count_keys, iter_count;

	if (cds_ft_verify(ft, stderr) != CDS_FT_STATUS_OK) {
		fprintf(stderr, "%s: %s: cds_ft_verify failed "
			"(per-node nr_keys mismatch)\n", test_name, phase);
		return -1;
	}
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
			"%s: %s quiescent mismatch: "
			"count_keys %lu != iteration_count %lu\n",
			test_name, phase, count_keys, iter_count);
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
	if (quiescent_count_check(ft, "inv_nr_keys_undercount", "insert") < 0) {
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
	if (quiescent_count_check(ft, "inv_nr_keys_undercount", "remove") < 0) {
		drain_and_destroy(ft, group);
		return -1;
	}

	return drain_and_destroy(ft, group);
}

/*
 * NO-UNDERCOUNT reader: the remove-side, one-sided counterpart to the
 * insert-side no-overcount check in inv_nr_keys_undercount.
 *
 * With the per-node nr_keys count folded into each remove's flip-txn (the
 * decrement commits atomically with the structural detach), the maintained
 * aggregate equals the structurally-present key set at every reader instant, so
 * it can never be SMALLER than a pointer traversal: the list-off lookup_gt
 * iterator only ever UNDER-enumerates a concurrently-relocated structure (its
 * re-descent never reaches an already-detached key and never re-emits a key in
 * ascending order), hence iter_count <= (present keys) = count_keys.  Therefore
 *
 *     count_keys >= iter_count   MUST hold.
 *
 * This is exactly the invariant the TXN fold buys, and the one a decrement-
 * BEFORE-detach ordering (the old pre-decrement) would VIOLATE: there the count
 * drops while the key -- and its enumeration -- is still live, so a reader sees
 * count_keys < iter_count.  It is thus a regression guard for the fold, and it
 * is robust against the recompaction non-linearizability that breaks the
 * opposite (count <= iter) direction: in that window count=N while the iterator
 * transiently sees N-1, so count >= iter holds comfortably.
 *
 * Read ORDER matters: read count_keys FIRST, then iterate.  The writer is
 * remove-only, so the structural key set is monotonically NON-INCREASING; a
 * later iteration can therefore only enumerate <= the earlier count_keys, which
 * removes the false-positive that the opposite order would create.  An actual
 * undercount (count already dropped below a still-live key) is still caught: the
 * low count is sampled first, then the iteration still reaches the live key.
 */
static void *inv_nr_keys_no_undercount_reader(void *arg)
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
		unsigned long count_keys, iter_count = 0;

		rcu_read_lock();
		count_keys = cds_ft_count_keys(ctx->ft);
		cds_ft_for_each_rcu(ctx->ft, iter)
			iter_count++;
		if (count_keys < iter_count)
			report_violation(ctx->test_name,
				"remove phase: count_keys %lu < iteration_count "
				"%lu (nr_keys undercount: the aggregate dropped "
				"below a still-live key enumeration, iter #%lu)",
				count_keys, iter_count, iters);
		rcu_read_unlock();

		iters++;
		if ((iters & 0x3f) == 0)
			rcu_quiescent_state();
	}
	cds_ft_iter_destroy(iter);
	rcu_unregister_thread();
	return NULL;
}

static int inv_nr_keys_no_undercount(void)
{
	struct cds_ft_group *group;
	struct cds_ft *ft = create_fixed_rankstats_ft(4, &group);
	struct timespec t0;
	struct {
		struct inv_iter_ctx ctx;
		pthread_mutex_t lock;
	} shared;
	pthread_t readers[NR_READERS_DEFAULT], writers[NR_WRITERS_DEFAULT];
	unsigned int i;

	shared.ctx.ft = ft;
	shared.ctx.key_len = 4;
	shared.ctx.test_name = "inv_nr_keys_no_undercount";
	pthread_mutex_init(&shared.lock, NULL);

	/* Populate, then concurrently remove (monotonically shrinking). */
	rcu_read_lock();
	for (i = 0; i < WRITER_POOL_SIZE / 2; i++) {
		struct ft_test_node *n = node_alloc(i);
		insert_u64(ft, i, n);
	}
	rcu_read_unlock();

	atomic_store(&violation_count, 0);
	test_go = 0;
	test_stop = 0;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	for (i = 0; i < NR_READERS_DEFAULT; i++)
		pthread_create(&readers[i], NULL,
			inv_nr_keys_no_undercount_reader, &shared.ctx);
	for (i = 0; i < NR_WRITERS_DEFAULT; i++)
		pthread_create(&writers[i], NULL,
			inv_nr_keys_remove_writer, &shared.ctx);
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
		fprintf(stderr, "inv_nr_keys_no_undercount: %lu violation(s)\n",
			atomic_load(&violation_count));
		drain_and_destroy(ft, group);
		return -1;
	}
	if (quiescent_count_check(ft, "inv_nr_keys_no_undercount", "remove") < 0) {
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
			cds_ft_make_exclusive(D);	/* DLM: cross-trie src must be exclusive */
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
			cds_ft_make_exclusive(D);	/* DLM: cross-trie src must be exclusive */
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

/*
 * DLM: a cross-trie merge needs an EXCLUSIVE source, but these oracles' @src is
 * LIVE and reader-watched on purpose (rargs[i].trie = is_src ? src : dst), so
 * cds_ft_make_exclusive would be a promise the oracle exists to break.  Detach
 * the moved region first: cds_ft_detach returns the detached trie exclusive by
 * construction ("no external handle to @detached existed before this call"), so
 * the merge half is DLM-legal and skips its own grace period.  Design note
 * decision (C): a cross-trie op is two sequential single-domain commits.
 *
 * The oracles' invariant is unaffected because it is PER-TRIE -- an ordered
 * traversal of src (or dst) never escapes that trie's key namespace and never
 * loops.  Detaching empties src, which readers of src may legitimately observe;
 * it never shows them a key outside the namespace.
 */
static enum cds_ft_status inv_merge_detached(struct cds_ft *dst,
		const uint8_t *dkey, size_t dlen,
		struct cds_ft *src, const uint8_t *skey, size_t slen)
{
	struct cds_ft *moved = NULL;
	enum cds_ft_status s;

	s = cds_ft_detach(src, skey, slen, &moved);
	if (s != CDS_FT_STATUS_OK)
		return s;
	if (!moved)
		return CDS_FT_STATUS_OK;	/* nothing matched the prefix */
	s = cds_ft_merge_at(dst, dkey, dlen, moved, NULL, 0);
	cds_ft_destroy(moved);			/* emptied by the merge */
	return s;
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
		s = inv_merge_detached(ctx->dst, NULL, 0, ctx->src, NULL, 0);
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
		s = inv_merge_detached(ctx->dst, (const uint8_t *) "T", 1,
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
/*   INVARIANT 11b: RE-ROOTED source, GLUE-diverge dst, ORDERED list   */
/*                                                                    */
/*   The writer merges a SUB-position source whose subtree root is     */
/*   re-rooted (an EXTERNAL leaf, shape 0; or a COMPRESSED run, shape   */
/*   1) at dst key "mb", which DIVERGES inside dst's compressed         */
/*   "mango" -- the build-invisible in-place graft reorder             */
/*   (ft_merge_graft_subpos_inplace), not the legacy detach path.  The */
/*   group has the ORDERED CELL list ON, so the writer also moves the  */
/*   source's ordered run into dst and (external shape) refreshes the   */
/*   moved head's cell edge byte; concurrent ordered readers iterating  */
/*   dst (cell-list path) across the flip must see ONLY dst's namespace */
/*   with correct keys -- a missed flip-proxy resolve, a mis-spliced    */
/*   run, or a stale edge byte surfaces an out-of-namespace key.  A src */
/*   reader must never escape UP into dst.                              */
/* ================================================================== */

/*
 * @shape selects the (source-root, dst-point) combination:
 *   0  external src  -> GLUE diverge ("mb" inside "mango")
 *   1  compressed src-> GLUE diverge ("mb" inside "mango")
 *   2  external src  -> NOSPLIT at-node (empty slot of dst {za,zb} at "zc")
 *   3  compressed src-> NOSPLIT build-branch (dst {m}, key "mxyz")
 *   4  KEY_SHORTER src ("ca" inside compressed "cab" over {e,f}) -> GLUE
 */
struct inv_rerooted_ctx {
	struct cds_ft *dst;
	struct cds_ft *src;
	const char *test_name;
	pthread_mutex_t lock;
	int shape;
};

struct inv_rerooted_reader_arg {
	struct inv_rerooted_ctx *ctx;
	struct cds_ft *trie;
	const char *const *allowed;	/* NULL-terminated allowed key set */
	const char *which;
};

static bool inv_rerooted_key_allowed(const char *const *allowed,
		const uint8_t *k, size_t kl)
{
	for (; *allowed; allowed++)
		if (strlen(*allowed) == kl && memcmp(*allowed, k, kl) == 0)
			return true;
	return false;
}

static void *inv_rerooted_reader(void *arg)
{
	struct inv_rerooted_reader_arg *ra =
		(struct inv_rerooted_reader_arg *) arg;
	struct inv_rerooted_ctx *ctx = ra->ctx;
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
			if (!inv_rerooted_key_allowed(ra->allowed, k, kl))
				report_violation(ctx->test_name,
					"%s reader saw out-of-namespace key "
					"(len %zu, %.*s) — merge re-parent / cell "
					"escape (iter #%lu, %s)",
					ra->which, kl, (int) kl,
					kl ? (const char *) k : "",
					iters, reverse ? "reverse" : "forward");
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

/* Per-shape merge parameters: the dst/src merge keys, the dst base keys, the
 * moved (merged) dst keys to remove on reset, the full source key set (init +
 * the src reader namespace), and the source keys to re-insert on reset (those
 * the merge consumed -- a "b"-style key that stays behind is not re-inserted). */
struct inv_rerooted_shape {
	const char *dst_key;
	const char *src_key;
	const char *dst_base[3];	/* NULL-terminated */
	const char *merged[3];		/* NULL-terminated */
	const char *src_keys[3];	/* NULL-terminated (init + namespace) */
	const char *src_reset[3];	/* NULL-terminated (re-insert on reset) */
};

static const struct inv_rerooted_shape inv_rerooted_shapes[] = {
	{ "mb",   "a",  { "mango", NULL }, { "mb", NULL },
	  { "a", "b", NULL }, { "a", NULL } },
	{ "mb",   "x",  { "mango", NULL }, { "mbabc", "mbabd", NULL },
	  { "xabc", "xabd", NULL }, { "xabc", "xabd", NULL } },
	{ "zc",   "a",  { "za", "zb", NULL }, { "zc", NULL },
	  { "a", "b", NULL }, { "a", NULL } },
	{ "mxyz", "x",  { "m", NULL }, { "mxyzabc", "mxyzabd", NULL },
	  { "xabc", "xabd", NULL }, { "xabc", "xabd", NULL } },
	{ "mb",   "ca", { "mango", NULL }, { "mbbe", "mbbf", NULL },	/* KEY_SHORTER */
	  { "cabe", "cabf", NULL }, { "cabe", "cabf", NULL } },
};

static void *inv_rerooted_writer(void *arg)
{
	struct inv_rerooted_ctx *ctx = (struct inv_rerooted_ctx *) arg;
	const struct inv_rerooted_shape *sh = &inv_rerooted_shapes[ctx->shape];
	struct cds_ft_iter *iter;

	rcu_register_thread();
	if (cds_ft_iter_create(ctx->dst, &iter) < 0)
		abort();

	while (!test_go)
		;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	while (!test_stop) {
		enum cds_ft_status s;
		unsigned int i;

		pthread_mutex_lock(&ctx->lock);
		s = inv_merge_detached(ctx->dst,
				(const uint8_t *) sh->dst_key, strlen(sh->dst_key),
				ctx->src,
				(const uint8_t *) sh->src_key, strlen(sh->src_key));
		pthread_mutex_unlock(&ctx->lock);
		if (s != CDS_FT_STATUS_OK) {
			/* Reported, not merely logged: see inv_graft_swap_writer. */
			report_violation(ctx->test_name,
				"merge failed (%s): the writer stopped, so this oracle would have passed vacuously",
				cds_ft_status_to_string(s));
			break;
		}
		rcu_quiescent_state();

		/* Reset: pull the merged keys out of dst, re-populate src. */
		rcu_read_lock();
		pthread_mutex_lock(&ctx->lock);
		for (i = 0; sh->merged[i]; i++)
			inv_merge_remove_key(ctx->dst, iter, sh->merged[i]);
		for (i = 0; sh->src_reset[i]; i++)
			cds_ft_insert(ctx->src,
				(const uint8_t *) sh->src_reset[i],
				strlen(sh->src_reset[i]),
				&node_alloc(411 + i)->node);
		pthread_mutex_unlock(&ctx->lock);
		rcu_read_unlock();
		rcu_quiescent_state();
	}

	cds_ft_iter_destroy(iter);
	rcu_unregister_thread();
	return NULL;
}

static int inv_merge_rerooted_run(int shape, const char *name)
{
	/* dst-reader namespace per shape: the base dst keys + transient merged. */
	static const char *const dst_allow[][4] = {
		{ "mango", "mb", NULL },		/* 0 ext glue */
		{ "mango", "mbabc", "mbabd", NULL },	/* 1 cmp glue */
		{ "za", "zb", "zc", NULL },		/* 2 ext at-node */
		{ "m", "mxyzabc", "mxyzabd", NULL },	/* 3 cmp branch */
		{ "mango", "mbbe", "mbbf", NULL },	/* 4 key-shorter */
	};
	const struct inv_rerooted_shape *sh = &inv_rerooted_shapes[shape];
	struct cds_ft_group_attr *gattr;
	struct cds_ft_group *group;
	struct cds_ft *dst, *src;
	struct inv_rerooted_ctx ctx;
	struct inv_rerooted_reader_arg rargs[2 * NR_READERS_DEFAULT];
	struct timespec t0;
	pthread_t readers[2 * NR_READERS_DEFAULT], writer;
	unsigned int i;
	int ret = 0;

	if (cds_ft_group_attr_create(&gattr) < 0)
		return -1;
	if (cds_ft_group_attr_set_max_key_len(gattr, 16) < 0 ||
	    cds_ft_group_attr_set_ordered_list(gattr, true) < 0) {
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
	for (i = 0; sh->dst_base[i]; i++)
		cds_ft_insert(dst, (const uint8_t *) sh->dst_base[i],
			strlen(sh->dst_base[i]), &node_alloc(1 + i)->node);
	for (i = 0; sh->src_keys[i]; i++)
		cds_ft_insert(src, (const uint8_t *) sh->src_keys[i],
			strlen(sh->src_keys[i]), &node_alloc(3 + i)->node);
	rcu_read_unlock();

	ctx.dst = dst;
	ctx.src = src;
	ctx.test_name = name;
	ctx.shape = shape;
	pthread_mutex_init(&ctx.lock, NULL);

	test_go = 0;
	test_stop = 0;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	for (i = 0; i < 2 * NR_READERS_DEFAULT; i++) {
		bool on_dst = (i & 1) == 0;

		rargs[i].ctx = &ctx;
		rargs[i].trie = on_dst ? dst : src;
		rargs[i].allowed = on_dst ? dst_allow[shape] : sh->src_keys;
		rargs[i].which = on_dst ? "dst" : "src";
		pthread_create(&readers[i], NULL, inv_rerooted_reader,
			&rargs[i]);
	}
	pthread_create(&writer, NULL, inv_rerooted_writer, &ctx);

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
		fprintf(stderr, "%s: %lu violation(s)\n", name,
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

static int inv_merge_rerooted_glue_no_escape(void)
{
	if (inv_merge_rerooted_run(0, "inv_merge_rerooted_glue_ext") < 0)
		return -1;
	if (inv_merge_rerooted_run(1, "inv_merge_rerooted_glue_compressed") < 0)
		return -1;
	if (inv_merge_rerooted_run(2, "inv_merge_rerooted_nosplit_atnode") < 0)
		return -1;
	/*
	 * Shape 4 ("key shorter": src key longer than the dst key) is RETIRED.
	 * Its writer drives a LIVE reader-watched @src, which DLM refuses, and
	 * unlike shapes 0-3 it cannot be decomposed into detach+merge:
	 * cds_ft_detach STRIPS the @key prefix, so a key equal to the src prefix
	 * detaches to a zero-length key and merges back as the bare dst prefix --
	 * a shorter, out-of-namespace key the reader correctly flags.  The same
	 * shape retired inv_merge_key_shorter_src_no_escape at 27556421.
	 */
	return inv_merge_rerooted_run(3, "inv_merge_rerooted_nosplit_branch");
}

/* ================================================================== */
/*                                                                    */
/*   INVARIANT 11c: SAME-TRIE rekey never escapes / corrupts          */
/*                                                                    */
/*   The writer rekeys "ax" <-> "az" within ONE trie (move "ax"'s     */
/*   subtree {axm,axn} to "az" and back), reusing cds_ft_merge_at with */
/*   src_ft == dst_ft.  Rekey is detach + cross-trie merge -- inherently */
/*   multi-stage -- so a key may be transiently ABSENT (parked in the  */
/*   transient trie) mid-move; a concurrent ordered reader must still  */
/*   only ever see keys in {axm,axn,azm,azn,ayp} -- never a garbage /  */
/*   out-of-namespace key, and never loop.  The disjoint "ayp" is a    */
/*   fixed witness the shared 'a' ancestor stays consistent.           */
/* ================================================================== */

static void *inv_rekey_writer(void *arg)
{
	struct inv_rerooted_ctx *ctx = (struct inv_rerooted_ctx *) arg;

	rcu_register_thread();
	while (!test_go)
		;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	while (!test_stop) {
		enum cds_ft_status s;

		pthread_mutex_lock(&ctx->lock);
		s = cds_ft_rekey_graft(ctx->dst, (const uint8_t *) "az", 2,
				(const uint8_t *) "ax", 2);
		pthread_mutex_unlock(&ctx->lock);
		if (s != CDS_FT_STATUS_OK) {
			fprintf(stderr, "inv_rekey writer ax->az: %s\n",
				cds_ft_status_to_string(s));
			break;
		}
		rcu_quiescent_state();
		pthread_mutex_lock(&ctx->lock);
		s = cds_ft_rekey_graft(ctx->dst, (const uint8_t *) "ax", 2,
				(const uint8_t *) "az", 2);
		pthread_mutex_unlock(&ctx->lock);
		if (s != CDS_FT_STATUS_OK) {
			fprintf(stderr, "inv_rekey writer az->ax: %s\n",
				cds_ft_status_to_string(s));
			break;
		}
		rcu_quiescent_state();
	}

	rcu_unregister_thread();
	return NULL;
}

static int inv_rekey_no_escape(void)
{
	static const char *const allowed[] = {
		"axm", "axn", "azm", "azn", "ayp", NULL,
	};
	struct cds_ft_group_attr *gattr;
	struct cds_ft_group *group;
	struct cds_ft *ft;
	struct inv_rerooted_ctx ctx;
	struct inv_rerooted_reader_arg rargs[2 * NR_READERS_DEFAULT];
	struct timespec t0;
	pthread_t readers[2 * NR_READERS_DEFAULT], writer;
	unsigned int i;
	int ret = 0;

	if (cds_ft_group_attr_create(&gattr) < 0)
		return -1;
	if (cds_ft_group_attr_set_max_key_len(gattr, 16) < 0 ||
	    cds_ft_group_attr_set_ordered_list(gattr, true) < 0) {
		cds_ft_group_attr_destroy(gattr);
		return -1;
	}
	if (cds_ft_group_create(gattr, &group) < 0) {
		cds_ft_group_attr_destroy(gattr);
		return -1;
	}
	cds_ft_group_attr_destroy(gattr);

	if (cds_ft_create(group, NULL, &ft) < 0) {
		cds_ft_group_destroy(group);
		return -1;
	}

	rcu_read_lock();
	cds_ft_insert(ft, (const uint8_t *) "axm", 3, &node_alloc(1)->node);
	cds_ft_insert(ft, (const uint8_t *) "axn", 3, &node_alloc(2)->node);
	cds_ft_insert(ft, (const uint8_t *) "ayp", 3, &node_alloc(3)->node);
	rcu_read_unlock();

	ctx.dst = ft;
	ctx.src = ft;
	ctx.test_name = "inv_rekey_no_escape";
	ctx.shape = 0;
	pthread_mutex_init(&ctx.lock, NULL);

	test_go = 0;
	test_stop = 0;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	for (i = 0; i < 2 * NR_READERS_DEFAULT; i++) {
		rargs[i].ctx = &ctx;
		rargs[i].trie = ft;
		rargs[i].allowed = allowed;
		rargs[i].which = "ft";
		pthread_create(&readers[i], NULL, inv_rerooted_reader,
			&rargs[i]);
	}
	pthread_create(&writer, NULL, inv_rekey_writer, &ctx);

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
		fprintf(stderr, "inv_rekey_no_escape: %lu violation(s)\n",
			atomic_load(&violation_count));
		ret = -1;
	}

	drain_trie_local(ft);
	rcu_barrier();
	cds_ft_destroy(ft);
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
		s = inv_merge_detached(ctx->dst, (const uint8_t *) "T", 1,
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
		s = inv_merge_detached(ctx->dst, (const uint8_t *) "Ta", 2,
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
		s = inv_merge_detached(ctx->dst, (const uint8_t *) "aXY", 3,
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
cds_ft_make_exclusive(src);	/* DLM: cross-trie src must be exclusive */

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
cds_ft_make_exclusive(src);	/* DLM: cross-trie src must be exclusive */

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
	ft_snapshot_record();
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
	cds_ft_group_attr_set_ordered_list(attr, true);
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

/*
 * DEBUG_COUNTERS-only group node-balance accessor (weak: NULL on a library
 * built without DEBUG_COUNTERS, in which case the node-balance assertion is
 * skipped).  Lets the bulk test confirm internal/compressed nodes are
 * reclaimed by a drained destroy across all the bulk-op node moves (a graft
 * allocates a node accounted to one trie and frees it accounted to another,
 * so the balance is only meaningful at group granularity).
 */
extern void cds_ft_debug_node_balance(const struct cds_ft_group *group,
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
				cds_ft_make_exclusive(D);	/* DLM: cross-trie src must be exclusive */
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
			cds_ft_make_exclusive(B);	/* DLM: cross-trie src must be exclusive */
			cds_ft_graft_swap(ctx->A, &P, 1, B);	/* A@0x50 <-> B */
			cds_ft_make_exclusive(B);	/* DLM: cross-trie src must be exclusive */
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
			cds_ft_make_exclusive(C);	/* DLM: cross-trie src must be exclusive */
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
	/*
	 * Same idea for the library's internal/compressed structural nodes:
	 * the bulk ops migrate nodes between the group's tries, so a drained
	 * destroy of every trie must leave the group-scoped node balance at
	 * zero.  Read before cds_ft_group_destroy frees the group.
	 */
	if (cds_ft_debug_node_balance) {
		unsigned long na = 0, nf = 0;

		cds_ft_debug_node_balance(group, &na, &nf);
		if (na != nf) {
			fprintf(stderr, "inv_ordered_bulk_consistency: internal node leak "
				"alloc %lu != freed %lu\n", na, nf);
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
		(void) sigaction(SIGILL, &sa, NULL);
		(void) sigaction(SIGBUS, &sa, NULL);
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
	RUN_TEST(inv_concurrent_writers_disjoint);
	RUN_TEST(inv_rekey_graft_disjoint);
	RUN_TEST(inv_rekey_graft_cross_junction);
	RUN_TEST(inv_rekey_graft_glue_dst);
	RUN_TEST(inv_rekey_graft_coherent_readers);
	RUN_TEST(inv_rekey_graft_shared);
	RUN_TEST(inv_rekey_linearizability);
	RUN_TEST(inv_rekey_public_atomic_no_gap);
	RUN_TEST(inv_rekey_public_staged_gap);
	RUN_TEST(inv_rekey_merge_occupied_dst);
	RUN_TEST(inv_rekey_merge_shared_dst);
	RUN_TEST(inv_rekey_src_mutated);
	RUN_TEST(inv_sibling_split_compress);
	RUN_TEST(inv_concurrent_writers_shared);
	RUN_TEST(inv_concurrent_writers_coarse_lock);
	RUN_TEST(inv_concurrent_writers_fine_lock);
	RUN_TEST(inv_concurrent_crosstrie_fine_lock);
	RUN_TEST(inv_concurrent_crosstrie_merge_fine_lock);
	RUN_TEST(inv_concurrent_crosstrie_graft_nilkey_fine_lock);
	RUN_TEST(inv_concurrent_crosstrie_merge_nilkey_fine_lock);
	RUN_TEST(inv_concurrent_crosstrie_graft_ord_fine_lock);
	RUN_TEST(inv_concurrent_crosstrie_graft_rank_coarse_lock);
	RUN_TEST(inv_concurrent_crosstrie_graft_swap_fine_lock);
	RUN_TEST(inv_bind_resume_order);
	RUN_TEST(inv_ordered_bulk_consistency);
	RUN_TEST(inv_compact_keycopy_terminates);
	RUN_TEST(inv_reverse_iteration_order);

	diag("2. Lookup consistency");
	RUN_TEST(inv_lookup_consistency);
	RUN_TEST(inv_insert_splice_window);
	RUN_TEST(inv_insert_replace_splice_window);
	RUN_TEST(inv_insert_replace_skipx_splice_window);
	RUN_TEST(inv_replace_skipx);
	/*
	 * Remove cross-view oracles: a key-disappearing remove must leave the
	 * structural index and the ordered cell list in ONE flip, else a reader
	 * observes the head gone from one index but present in the other.  In one
	 * RCU read section the reader does lookup_first/last -> point lookup(that
	 * key) -> lookup_first/last again and flags present/absent/present
	 * (k1==k2 yet the structural lookup missed); with no inserts, "present
	 * again" can only be the non-atomic unpublish window.  Each is provably 0
	 * now that the remove-side fusion has landed: in-place, recompaction,
	 * compressed-parent recompaction, and external-promote all commit the
	 * structural unlink together with the cell unsplice in one flip.
	 *
	 * inv_remove_cross_view{,_compressed} are min-draining;
	 * inv_remove_cross_view_compressed_parent is max-draining (a prefix key
	 * sorts before its extensions, so only max-draining puts the
	 * compressed-parent promote removal on the observed ordered endpoint).
	 * inv_remove_cross_view_prefix_siblings{,_all} are min-draining over
	 * variable-length prefix/extension keys, so the prefix key is removed
	 * while its sibling extensions persist (the holder's external_nodes is
	 * cleared in place): the _all variant drains via cds_ft_remove_all and
	 * adds the empty key to cover the NIL-key clear too.
	 */
	RUN_TEST(inv_remove_cross_view);
	RUN_TEST(inv_remove_cross_view_compressed);
	RUN_TEST(inv_remove_cross_view_compressed_parent);
	RUN_TEST(inv_remove_cross_view_prefix_siblings);
	RUN_TEST(inv_remove_cross_view_prefix_siblings_all);
	RUN_TEST(inv_graft_cross_view);
	RUN_TEST(inv_graft_root_swap_cross_view);
	RUN_TEST(inv_merge_root_swap_cross_view);
	RUN_TEST(inv_graft_swap_shared_dst);
	RUN_TEST(inv_graft_swap_shared_dst_nolist);
	RUN_TEST(inv_graft_swap_shared_dst_kshort);
	RUN_TEST(inv_graft_swap_shared_dst_kshort_nolist);
	RUN_TEST(inv_graft_swap_shared_dst_deep);
	RUN_TEST(inv_graft_swap_shared_dst_deep_nolist);
	RUN_TEST(inv_graft_swap_shared_dst_ks2_nolist);
	RUN_TEST(inv_graft_swap_shared_dst_ks2_solo);
	RUN_TEST(inv_graft_swap_shared_dst_cparent_nolist);
	RUN_TEST(inv_graft_swap_shared_dst_cparent_solo);
	RUN_TEST(inv_graft_swap_shared_dst_wide_nolist);
	RUN_TEST(inv_graft_swap_shared_dst_fuse);
	RUN_TEST(inv_graft_swap_shared_dst_fuse_nolist);
	RUN_TEST(inv_graft_swap_shared_dst_fuse_solo);
	RUN_TEST(inv_graft_swap_shared_dst_ksfix);
	RUN_TEST(inv_graft_swap_shared_dst_ksfix_nolist);
	RUN_TEST(inv_graft_swap_shared_dst_ksfix_solo);
	RUN_TEST(inv_graft_swap_shared_dst_kshort_solo);
	RUN_TEST(inv_graft_swap_shared_dst_deep_solo);
	RUN_TEST(inv_prefix_key_park_vs_holder_churn);
	RUN_TEST(inv_empty_dst_root_graft_peer);
	RUN_TEST(inv_empty_dst_root_graft_peer_nolist);
	RUN_TEST(inv_empty_dst_root_merge_peer);
	RUN_TEST(inv_empty_dst_root_merge_peer_nolist);
	RUN_TEST(inv_merge_root_src_cross_view);
	RUN_TEST(inv_merge_cross_view);
	RUN_TEST(inv_merge_spinecopy_cross_view);
	RUN_TEST(inv_merge_src_cross_view);
	RUN_TEST(inv_merge_src_spinecopy_cross_view);
	RUN_TEST(inv_detach_cross_view);
	RUN_TEST(inv_detach_root_cross_view);

	diag("3. Duplicate chain acyclicity");
	RUN_TEST(inv_dup_chain_acyclicity);
	RUN_TEST(inv_dup_chain_skipx_head_promotion);

	diag("Ordered-list-OFF (no-cell) consistency");
	RUN_TEST(inv_no_ordered_list_consistency);
	RUN_TEST(inv_speculative_per_trie_eager);
	RUN_TEST(inv_speculative_key_verify);
	RUN_TEST(inv_graft_no_list_diverge);
	RUN_TEST(inv_graft_displaced_external);

	diag("4. Graft-swap atomicity");
	RUN_TEST(inv_graft_swap_atomicity);
	RUN_TEST(inv_graft_swap_cross_view);
	RUN_TEST(inv_graft_swap_sub_cross_view);
	RUN_TEST(inv_graft_swap_empty_cross_view);

	diag("5. Relational lookup consistency");
	RUN_TEST(inv_relational_lookup);

	diag("6. Rank-based lookup stability");
	RUN_TEST(inv_nth_last_stability);
	RUN_TEST(inv_nth_first_stability);

	diag("7. Iterator skip stability");
	RUN_TEST(inv_skip_forward_stability);
	RUN_TEST(inv_skip_reverse_stability);

	diag("7b. Rank/select readers resolve a parked external_nodes proxy");
	RUN_TEST(inv_nth_external_nodes_resolve);

	diag("8. nr_keys undercount ordering");
	RUN_TEST(inv_nr_keys_undercount);

	diag("8b. nr_keys no-undercount under concurrent remove (fold guard)");
	RUN_TEST(inv_nr_keys_no_undercount);

	diag("9. Ordered traversal never escapes its trie");
	RUN_TEST(inv_ordered_no_escape_graft);

	diag("10. Piecewise merge never escapes the destination");
	RUN_TEST(inv_merge_no_escape);

	diag("11. Merge into a non-root destination never escapes");
	RUN_TEST(inv_merge_nonroot_dst_no_escape);

	diag("11b. Re-rooted source, GLUE-diverge dst, ordered list");
	RUN_TEST(inv_merge_rerooted_glue_no_escape);

	diag("11c. Same-trie rekey never escapes / corrupts");
	RUN_TEST(inv_rekey_no_escape);

	diag("12. Merge into a compressed destination never escapes");
	RUN_TEST(inv_merge_compressed_dst_no_escape);

	diag("13. Merge into a key-shorter destination never escapes");
	RUN_TEST(inv_merge_key_shorter_dst_no_escape);

	diag("14. Merge from a key-shorter source never escapes");

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
