// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

/*
 * White-box test for the COPY_SLOT engine record (urcu_mcas_add_copy_slot /
 * urcu_txn_copy_slot): the helpable building block recompaction uses to copy one
 * child slot of a node into the fresh node while FREEZING the source.
 *
 * A COPY_SLOT record is a CAS record with old == new == V -- an identity edge
 * that merely freezes the source slot for the transaction (readers resolve it to
 * V; settle restores it to V on commit AND abort, so the source stays
 * traversable either way) -- PLUS a side effect: at install it publishes V into
 * a second, still-unpublished word @dst (the fresh node's slot).
 *
 * Destination lifetime (mirrors the recompaction reclamation contract).
 * -------------------------------------------------------------------------
 * Unlike @slot, @dst is NOT reader-reachable: the edge that publishes the fresh
 * node is another record of the SAME transaction, so nothing names that node
 * until the transaction linearizes -- and on abort nothing ever does.  The
 * lifetime hazard here is therefore writer-vs-writer (a driver's *dst = V store
 * racing the owner's free of the node), not reader-vs-free, and a grace period
 * is not what closes it.
 *
 * What closes it is the record's install latch.  Only the PLANTER stores @dst,
 * and it does so under the latch, before release-storing DONE; the owner's
 * urcu_mcas_settle() claims every record's install word -- spinning out a BUSY
 * planter, and ACQUIRING that release -- before urcu_txn_commit() returns.  So
 * once commit returns, every *dst store has both completed and become visible,
 * and the owner may free the fresh node IMMEDIATELY, with no grace period.  That
 * is exactly what recompaction does with N' on its abort path.
 *
 * So this test FREES THE DESTINATION IMMEDIATELY (free_fresh() below), on both
 * outcomes, deliberately: that is the contract recompaction depends on, and under
 * -fsanitize=address this test is its regression detector.  Unlatch the dst publish,
 * or stop urcu_mcas_settle() from claiming each install word and spinning out a BUSY
 * planter, and a lagging helper's store lands in freed memory -- ASAN reports a
 * heap-use-after-free (WRITE of size 8) from urcu_mcas_plant().  A grace-period
 * defer here would hide exactly that regression, which is why there is none.
 *
 * What this does NOT catch is the loss of the ACQUIRE with which settle() and
 * plant() read DONE.  That one is a VISIBILITY bug, not a sequencing bug: the
 * planter's store has completed either way, and on x86 uatomic_cmpxchg_mo() discards
 * both memory orders (lock cmpxchg is a full barrier), so removing the acquire is
 * byte-identical machine code here and this test stays green.  It bites only on the
 * compiler-builtins backend (--enable-compiler-atomic-builtins) and on weakly-ordered
 * targets.  Reach for TSAN or an ARM/POWER box for that half; ASAN cannot see it.
 *
 * The one exception is -DURCU_MCAS_NO_ABA_FIX, which compiles out both the install
 * latch and settle's claim.  The pre-latch rule then applies again -- *dst is
 * stored unlatched by ANY driver, including a helper that read the transaction
 * UNDECIDED and then stalled -- so free_fresh() call_rcu()s instead.  That knob is
 * ABA-unsafe by construction and is not expected to PASS (property 3 fails,
 * intermittently); deferring only keeps its failure from being heap corruption.
 *
 * The destination is allocated FRESH PER ATTEMPT in both modes: never a reused
 * stack slot, which a late store would corrupt under the unlatched engine.
 *
 * Three properties are checked:
 *
 *  1. Single-threaded, multi-record: a txn of two COPY_SLOT records publishes
 *     each source value into its dst and leaves the sources unchanged.
 *
 *  2. Single-threaded, lone record: a txn of ONE COPY_SLOT still publishes dst
 *     and restores the source -- it must NOT take the nr==1 bare-CAS fast path,
 *     whose old==new no-op CAS would skip the dst publish.
 *
 *  3. Concurrent atomicity + window-catch: writers run transfer transactions
 *     over a pair of source words (sum invariant 0); copiers snapshot the pair
 *     with two COPY_SLOT records into a fresh node and commit.  Every committed
 *     snapshot has sum 0 -- the two copies linearize TOGETHER at the one
 *     status-word commit, and a writer that changes a source in the
 *     prep->install window aborts the copier (read-set check src == V) rather
 *     than letting it publish a torn snapshot -- and the copies never disturb
 *     the sources (the final source sum stays 0).
 *
 * Slots carry a per-word monotonic version (lf_bump) so a stored word never
 * repeats a bit pattern (the engine's non-ABA precondition).  QSBR flavor.
 */

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#ifndef _LGPL_SOURCE
#define _LGPL_SOURCE
#endif

#include <inttypes.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include <urcu/compiler.h>
#include <urcu/uatomic.h>
#include <urcu-qsbr.h>
#include <urcu-call-rcu.h>
#include <urcu/rcu-txn.h>

#include "tap.h"

#define NR_TESTS	5
#define NR_WRITERS	4
#define NR_COPIERS	4
#define NR_WORKERS	(NR_WRITERS + NR_COPIERS)
#define OPS_PER_WORKER	40000
#define RETRY_BOUND	512		/* generous worst single-op bypass bound */

/* The transfer pair snapshotted by copiers and mutated by writers. */
static void *g_src[2];

/*
 * The recompaction "fresh node" analog: a heap-allocated destination whose slots
 * receive the COPY_SLOT publishes.
 */
struct fresh_node {
	void *slot[2];
	struct rcu_head rcu;
};

/*
 * Reclaim the destination, per engine mode -- see "Destination lifetime" above.
 *
 * Latched engine (default): once urcu_txn_commit() has returned, urcu_mcas_settle()
 * has claimed every record's install word -- spinning out a BUSY planter and
 * ACQUIRING its release of DONE -- so no driver can still be storing *dst.  Free
 * IMMEDIATELY, exactly as the fractal trie's on-abort rollback does.  Running this
 * test under -fsanitize=address then REGRESSION-TESTS the SEQUENCING half of that
 * contract: delete settle's claim loop, or unlatch the dst publish, and a helper's
 * late store lands in freed memory (observed: heap-use-after-free, WRITE of size 8
 * from urcu_mcas_plant() via urcu_mcas_engage_foreign / urcu_mcas_drive_install).
 * Do not "simplify" this back to an unconditional defer: the defer is what would
 * HIDE such a regression.  (The VISIBILITY half -- the acquire -- is invisible to
 * ASAN on x86; see the header comment.)
 *
 * -DURCU_MCAS_NO_ABA_FIX: the install latch and settle's claim are compiled out,
 * *dst is stored unlatched by ANY driver, and a stalled helper's store can land
 * after the owner returned from commit.  Defer through a grace period there -- that
 * knob is ABA-unsafe by construction and is not expected to pass (property 3 fails,
 * intermittently); the defer merely keeps its failure from being heap corruption.
 */
#ifdef URCU_MCAS_NO_ABA_FIX
static void free_fresh_rcu(struct rcu_head *h)
{
	free(caa_container_of(h, struct fresh_node, rcu));
}

static void free_fresh(struct fresh_node *fn)
{
	call_rcu(&fn->rcu, free_fresh_rcu);
}
#else
static void free_fresh(struct fresh_node *fn)
{
	free(fn);
}
#endif

struct worker_arg {
	long committed;
	unsigned long max_retry;
	long violations;		/* copier: torn snapshots observed (must be 0) */
};

static intptr_t lf_val(uintptr_t w) { return (intptr_t) w >> 32; }
static uintptr_t lf_bump(uintptr_t w, intptr_t delta)
{
	intptr_t val = lf_val(w) + delta;
	unsigned int ver = (unsigned int) ((w >> 1) & 0x7fffffffu) + 1;
	return ((uintptr_t) (uint32_t) (int32_t) val << 32)
		| ((uintptr_t) (ver & 0x7fffffffu) << 1);
}

/* Writer: {g_src[0] += 2, g_src[1] -= 2} -- keeps lf_val sum invariantly 0. */
static void *writer(void *arg)
{
	struct worker_arg *wa = (struct worker_arg *) arg;
	long n;

	rcu_register_thread();
	for (n = 0; n < OPS_PER_WORKER; n++) {
		struct urcu_mcas_txn tx;
		int ret;

		urcu_txn_init(&tx, NULL);
		do {
			uintptr_t o0, o1;

			urcu_txn_begin(&tx);
			o0 = (uintptr_t) urcu_txn_load(&tx, &g_src[0], URCU_MCAS_TAG);
			o1 = (uintptr_t) urcu_txn_load(&tx, &g_src[1], URCU_MCAS_TAG);
			urcu_txn_store(&tx, &g_src[0], (void *) o0,
					(void *) lf_bump(o0, 2), URCU_MCAS_TAG);
			urcu_txn_store(&tx, &g_src[1], (void *) o1,
					(void *) lf_bump(o1, -2), URCU_MCAS_TAG);
			ret = urcu_txn_commit(&tx);
			urcu_txn_end(&tx);
			if (ret < 0)
				abort();	/* MEMORY_ERROR */
		} while (ret == URCU_TXN_STATUS_ABORT);

		if (tx.retry > wa->max_retry)
			wa->max_retry = tx.retry;
		wa->committed++;
		rcu_quiescent_state();
	}
	rcu_unregister_thread();
	return NULL;
}

/* Copier: snapshot the pair via two COPY_SLOT records; committed sum must be 0. */
static void *copier(void *arg)
{
	struct worker_arg *wa = (struct worker_arg *) arg;
	long n;

	rcu_register_thread();
	for (n = 0; n < OPS_PER_WORKER; n++) {
		struct urcu_mcas_txn tx;
		struct fresh_node *fn = NULL;
		void *d0, *d1;
		int ret;

		urcu_txn_init(&tx, NULL);
		do {
			void *v0, *v1;

			/*
			 * Fresh destination per ATTEMPT.  With the install latch
			 * compiled in, an aborted attempt's node is quiescent once
			 * commit returns and could be reused; under
			 * -DURCU_MCAS_NO_ABA_FIX a lagging helper may still write
			 * it.  Never reused: it is call_rcu'd below and a new one
			 * allocated.
			 */
			fn = malloc(sizeof(*fn));
			if (!fn)
				abort();
			uatomic_store(&fn->slot[0], NULL, CMM_RELAXED);
			uatomic_store(&fn->slot[1], NULL, CMM_RELAXED);

			urcu_txn_begin(&tx);
			v0 = urcu_txn_load(&tx, &g_src[0], URCU_MCAS_TAG);
			v1 = urcu_txn_load(&tx, &g_src[1], URCU_MCAS_TAG);
			(void) urcu_txn_copy_slot(&tx, &g_src[0], v0, &fn->slot[0],
					URCU_MCAS_TAG);
			(void) urcu_txn_copy_slot(&tx, &g_src[1], v1, &fn->slot[1],
					URCU_MCAS_TAG);
			ret = urcu_txn_commit(&tx);
			urcu_txn_end(&tx);
			if (ret < 0)
				abort();	/* MEMORY_ERROR */
			if (ret == URCU_TXN_STATUS_ABORT) {
				/*
				 * Aborted: settle left this node quiescent, so the
				 * latched engine frees it immediately -- the same
				 * thing recompaction's on-abort rollback does, and
				 * the case a sanitizer must find clean.
				 */
				free_fresh(fn);
				fn = NULL;
			}
		} while (ret == URCU_TXN_STATUS_ABORT);

		/* Committed: the fresh node holds a consistent snapshot of the pair. */
		d0 = uatomic_load(&fn->slot[0], CMM_ACQUIRE);
		d1 = uatomic_load(&fn->slot[1], CMM_ACQUIRE);
		if (lf_val((uintptr_t) d0) + lf_val((uintptr_t) d1) != 0)
			wa->violations++;
		/* Committed: quiescent once commit returned -- same reclaim rule. */
		free_fresh(fn);

		if (tx.retry > wa->max_retry)
			wa->max_retry = tx.retry;
		wa->committed++;
		rcu_quiescent_state();
	}
	rcu_unregister_thread();
	return NULL;
}

/*
 * Single-threaded sanity: run @nr COPY_SLOT records (1 or 2) copying g_src[0..nr)
 * into a fresh node, then check every dst got the source value and every source
 * is unchanged.  Single-threaded, so no helper can touch the node: a plain free
 * is enough.  Returns true on success.
 */
static bool sanity_copy(unsigned int nr)
{
	struct urcu_mcas_txn tx;
	struct fresh_node *fn = malloc(sizeof(*fn));
	uintptr_t before[2];
	unsigned int i;
	bool ok_res = true;
	int ret;

	if (!fn)
		return false;
	for (i = 0; i < nr; i++) {
		before[i] = (uintptr_t) g_src[i];
		fn->slot[i] = NULL;
	}

	urcu_txn_init(&tx, NULL);
	do {
		urcu_txn_begin(&tx);
		for (i = 0; i < nr; i++) {
			void *v = urcu_txn_load(&tx, &g_src[i], URCU_MCAS_TAG);
			(void) urcu_txn_copy_slot(&tx, &g_src[i], v, &fn->slot[i],
					URCU_MCAS_TAG);
		}
		ret = urcu_txn_commit(&tx);
		urcu_txn_end(&tx);
		if (ret < 0) {
			free(fn);
			return false;
		}
	} while (ret == URCU_TXN_STATUS_ABORT);

	for (i = 0; i < nr; i++) {
		if ((uintptr_t) fn->slot[i] != before[i])	/* dst published */
			ok_res = false;
		if ((uintptr_t) g_src[i] != before[i])	/* source restored/unchanged */
			ok_res = false;
	}
	free(fn);
	return ok_res;
}

int main(void)
{
	pthread_t th[NR_WORKERS];
	struct worker_arg args[NR_WORKERS];
	long total = 0, violations = 0;
	unsigned long max_retry = 0;
	intptr_t sum;
	bool multi_ok, lone_ok;
	int i;

	plan_tests(NR_TESTS);
	rcu_register_thread();

	/* (1) + (2) single-threaded, before any contention. */
	g_src[0] = (void *) lf_bump(0, 7);
	g_src[1] = (void *) lf_bump(0, -7);
	multi_ok = sanity_copy(2);
	ok(multi_ok,
		"COPY_SLOT (2 records) publishes each dst and leaves the sources intact");

	g_src[0] = (void *) lf_bump((uintptr_t) g_src[0], 3);
	lone_ok = sanity_copy(1);
	ok(lone_ok,
		"lone COPY_SLOT publishes dst (skips the nr==1 bare-CAS fast path)");

	/* (3)-(5) concurrent: reset the pair to sum 0. */
	g_src[0] = NULL;
	g_src[1] = NULL;
	for (i = 0; i < NR_WORKERS; i++) {
		args[i].committed = 0;
		args[i].max_retry = 0;
		args[i].violations = 0;
		pthread_create(&th[i], NULL,
				i < NR_WRITERS ? writer : copier, &args[i]);
	}
	rcu_thread_offline();
	for (i = 0; i < NR_WORKERS; i++) {
		pthread_join(th[i], NULL);
		total += args[i].committed;
		violations += args[i].violations;
		if (args[i].max_retry > max_retry)
			max_retry = args[i].max_retry;
	}
	rcu_thread_online();

	sum = lf_val((uintptr_t) g_src[0]) + lf_val((uintptr_t) g_src[1]);

	diag("%d writers + %d copiers x %d ops = %ld committed; snapshot violations "
		"= %ld; final source sum = %" PRIdPTR "; max single-op retry = %lu",
		NR_WRITERS, NR_COPIERS, OPS_PER_WORKER, total, violations, sum,
		max_retry);

	ok(violations == 0,
		"every committed COPY_SLOT snapshot was atomic (pair sum invariant 0)");
	ok(sum == 0,
		"COPY_SLOT never disturbed a source (writers' sum invariant preserved)");
	ok(total == (long) NR_WORKERS * OPS_PER_WORKER && max_retry < RETRY_BOUND,
		"every transaction eventually committed with bounded retry");

	rcu_barrier();
	rcu_unregister_thread();
	return exit_status();
}
