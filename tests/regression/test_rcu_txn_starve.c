// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: GPL-2.0-or-later

/*
 * Starvation regression test for the <urcu/rcu-txn.h> escalation fallback.
 *
 * The optimistic retry is bounded-blocking but NOT starvation-free: a large or
 * repeatedly-bypassed transaction can be defeated by a stream of smaller ones,
 * because the single-edge fast path and the read->install window let a
 * committer change a footprint slot between this op's read and its install.
 * When a handle crosses URCU_TXN_FALLBACK retries it escalates into the
 * domain's fair mutex and publishes domain->active, funnelling every FUTURE
 * transaction through the same lane.  That closes the optimistic-writer set,
 * so the starved op then contends only with the finite in-flight set and
 * commits within a bounded number of retries -- progress is guaranteed with no
 * quiescence.
 *
 * The unit tests for this (test_rcu_txn_fallback,
 * test_rcu_txn_fallback_publish) force the threshold down to 8, and one of
 * them white-box assigns txn.retry outright.  Nothing ever starved a
 * transaction ORGANICALLY, at the real default of URCU_TXN_FALLBACK.  So the
 * reactive trigger -- and the joiner-promotion branch of the escalation funnel
 * -- had no coverage at all outside those forced-threshold constructions.
 * This test provides it.
 *
 * The workload.  One shared domain over an array of transacted slots, and two
 * writer classes:
 *
 *   SMALL (many): a single-edge txn on one random slot.  With nr == 1 it
 *     commits with a bare CAS until it has retried past URCU_MCAS_ESCALATE --
 *     no descriptor, no proxy, nothing to block against.  This is the "stream
 *     of smaller ones".
 *
 *   WIDE (few): one txn over @width DISTINCT slots.  By default @width ==
 *     @nslots, so it touches EVERY slot: any small commit at all, anywhere,
 *     between its loads and its install invalidates the attempt.  It starves.
 *
 * The lane is earned by RETRIES ALONE, and the budget of retries a handle gets
 * is its own: PER_COST * cost, where cost is the loads plus the write-set
 * records of one attempt.  The wide op loads and stores every slot, so its
 * cost is 2 * @width and its budget is PER_COST * 2 * @width -- which is what
 * this test asserts it starves past.
 *
 * What is checked.  The escalation state lives in the caller-owned on-stack
 * handle, so a writer samples it directly after each urcu_txn_begin():
 * in_fallback going 0 -> 1 is a lane entry; fb_published tells an INITIATOR
 * (met a trigger itself) from a JOINER (escalated only because it read
 * domain->active); and fb_published rising while already in the lane is a
 * PROMOTION -- a joiner that went on starving after the initiator departed,
 * and was promoted so the funnel persists as long as some transaction still
 * needs it.
 *
 * Run it directly for the numbers, including the control:
 *   ./test_rcu_txn_starve --nsmall 64 --nwide 4 --nslots 32 --duration 5000
 *   ./test_rcu_txn_starve --domain 0     # no lane: the wide txns never finish
 */

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#ifndef _LGPL_SOURCE
#define _LGPL_SOURCE
#endif

#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <urcu/compiler.h>
#include <urcu/uatomic.h>
#include <urcu-qsbr.h>			/* generic rcu_* names => QSBR flavor */
#include <urcu-call-rcu.h>
#include <urcu/rcu-txn.h>		/* include AFTER the RCU flavor */

#include "tap.h"

#define NR_TESTS	5

/* ── knobs ────────────────────────────────────────────────────────────── */
static int  nslots      = 32;	/* transacted words the writers fight over */
static int  nsmall      = 32;	/* single-edge writers (the starving stream) */
static int  nwide       = 4;	/* wide writers (the victims) */
static int  width;		/* edges per wide txn; 0 => nslots */
static long duration_ms = 5000;
static int  use_disjoint = 1;	/* the wide write set IS pairwise distinct */
/*
 * --domain 0 is the CONTROL: hand every handle a NULL domain, disabling the
 * fallback entirely (pure optimistic retry -- see urcu_txn_init).  It is what
 * shows the lane DOES something rather than merely being entered: with no lane
 * nothing ever closes the optimistic-writer set, so the wide transactions are
 * defeated indefinitely.  Not a passing configuration -- the escalation checks
 * are skipped -- it is here to be run by hand.
 */
static int  use_domain  = 1;

/* The contended slots.  Values are small even integers: bit 0 is the engine's
 * proxy tag and a live value must never carry it, so every commit adds 2. */
static void **slot;

/* ONE escalation domain: every writer contends for the same fair lane. */
static struct urcu_txn_domain g_dom;

static int g_stop;

struct stats {
	long long commits;
	long long aborts;
	long long errors;		/* MEMORY_ERROR: terminal, releases the turn */
	long long abandoned;		/* gave up mid-starvation at shutdown */
	unsigned long max_retry;
	long long lane_entries;		/* in_fallback 0 -> 1 */
	long long as_initiator;		/* ... and we published the episode */
	long long as_joiner;		/* ... and we merely read domain->active */
	long long promotions;		/* joiner starved in-lane -> initiator */
	long long applied;		/* edges committed (for conservation) */
};

struct arg {
	int id;
	struct stats st;
};

/*
 * The wide op's escalation budget.  One attempt loads @width slots and records
 * @width edges, so its cost is 2 * @width and its retry budget is that scaled
 * by PER_COST_NUM/PER_COST_DEN (capped).  With a flat budget (PER_COST_NUM ==
 * 0) it is simply URCU_TXN_FALLBACK.  This is the threshold the wide op must
 * starve past.
 */
static unsigned long wide_budget(void)
{
	unsigned long t;

	if (!URCU_TXN_FALLBACK_PER_COST_NUM)
		return URCU_TXN_FALLBACK;
	t = ((unsigned long) URCU_TXN_FALLBACK_PER_COST_NUM * 2UL *
			(unsigned long) width) / URCU_TXN_FALLBACK_PER_COST_DEN;
	if (!t)
		t = 1;
	return t > URCU_TXN_FALLBACK_MAX ? URCU_TXN_FALLBACK_MAX : t;
}

/* xorshift64 per-thread RNG (no shared rand() lock). */
static inline unsigned long xrand(unsigned long *s)
{
	unsigned long x = *s;

	x ^= x << 13;
	x ^= x >> 7;
	x ^= x << 17;
	return *s = x;
}

/*
 * Sample the handle's escalation state right after begin().  @in/@pub carry
 * the previous attempt's view, so we see the 0 -> 1 EDGES rather than levels:
 * a lane entry, and (the interesting one) a joiner being promoted mid-episode.
 */
static inline void sample_lane(struct urcu_mcas_txn *txn, struct stats *st,
		int *in, int *pub)
{
	int now_in = uatomic_load(&txn->in_fallback, CMM_RELAXED);
	int now_pub = txn->fb_published;		/* thread-private */

	if (now_in && !*in) {
		st->lane_entries++;
		if (now_pub)
			st->as_initiator++;		/* met a trigger itself */
		else
			st->as_joiner++;		/* funnelled by domain->active */
	} else if (now_in && *in && now_pub && !*pub) {
		st->promotions++;			/* maybe_publish promoted us */
	}
	*in = now_in;
	*pub = now_pub;
}

/*
 * Close an attempt: account for it, end the bracket, and say whether to
 * re-run.
 *
 * An ABORT is NOT terminal -- end() deliberately KEEPS the handle's fallback
 * turn so the next attempt re-enters the lane as the same head.  So a writer
 * that walks away from an aborted transaction must urcu_txn_abandon() first,
 * or it holds the domain's lane forever and stalls every writer in the domain.
 * Which is precisely what these threads do: the stop flag lands while a wide
 * txn is mid-starvation, and without the abandon the run would hang in
 * pthread_join() behind a lane nobody will ever release.
 */
static inline int attempt_done(struct urcu_mcas_txn *txn, struct stats *st,
		enum urcu_txn_status status)
{
	int again = 0;

	if (txn->retry > st->max_retry)
		st->max_retry = txn->retry;	/* sample before end() */
	switch (status) {
	case URCU_TXN_STATUS_ABORT:
		st->aborts++;
		if (uatomic_load(&g_stop, CMM_RELAXED)) {
			st->abandoned++;
			urcu_txn_abandon(txn);	/* give up: forfeit the turn */
		} else {
			again = 1;
		}
		break;
	case URCU_TXN_STATUS_MEMORY_ERROR:
		st->errors++;			/* terminal: end() releases the turn */
		break;
	default:				/* OK */
		break;
	}
	urcu_txn_end(txn);
	return again;
}

/* The stream: one single-edge txn on a random slot (nr == 1 => bare-CAS path). */
static void *small_writer(void *p)
{
	struct arg *me = p;
	unsigned long seed = 0x9e3779b97f4a7c15UL ^ (unsigned long) (me->id + 1);

	rcu_register_thread();
	rcu_thread_offline();			/* a writer is not a long-term reader */

	while (!uatomic_load(&g_stop, CMM_RELAXED)) {
		struct urcu_mcas_txn txn;
		int i = (int) (xrand(&seed) % (unsigned long) nslots);
		int in = 0, pub = 0;
		enum urcu_txn_status st;

		rcu_thread_online();
		urcu_txn_init(&txn, use_domain ? &g_dom : NULL);
		urcu_txn_declare_disjoint(&txn);	/* one slot: trivially distinct */
		do {
			void *v;

			urcu_txn_begin(&txn);
			sample_lane(&txn, &me->st, &in, &pub);
			v = urcu_txn_load(&txn, &slot[i], URCU_MCAS_TAG);
			urcu_txn_store(&txn, &slot[i], v,
					(void *) ((uintptr_t) v + 2),
					URCU_MCAS_TAG);
			st = urcu_txn_commit(&txn);
		} while (attempt_done(&txn, &me->st, st));
		if (st == URCU_TXN_STATUS_OK) {
			me->st.commits++;
			me->st.applied += 1;
		}
		rcu_quiescent_state();		/* let call_rcu grace periods advance */
		rcu_thread_offline();
	}
	rcu_thread_online();
	rcu_unregister_thread();
	return NULL;
}

/*
 * The victim: one txn over @width DISTINCT slots.  Its footprint spans the
 * array, so a small commit inside it between this op's loads and its install
 * invalidates the attempt.  This is the transaction that starves.
 */
static void *wide_writer(void *p)
{
	struct arg *me = p;
	unsigned long seed = 0xdeadbeefcafef00dUL ^ (unsigned long) (me->id + 1);
	int *pick = calloc((size_t) nslots, sizeof(*pick));
	int k;

	if (!pick)
		return NULL;
	for (k = 0; k < nslots; k++)
		pick[k] = k;

	rcu_register_thread();
	rcu_thread_offline();

	while (!uatomic_load(&g_stop, CMM_RELAXED)) {
		struct urcu_mcas_txn txn;
		int in = 0, pub = 0;
		enum urcu_txn_status st;

		/* Partial Fisher-Yates: @width distinct slots, fresh each op. */
		for (k = 0; k < width; k++) {
			int j = k + (int) (xrand(&seed) %
					(unsigned long) (nslots - k));
			int t = pick[k];

			pick[k] = pick[j];
			pick[j] = t;
		}

		rcu_thread_online();
		urcu_txn_init(&txn, use_domain ? &g_dom : NULL);
		if (use_disjoint)
			urcu_txn_declare_disjoint(&txn);	/* picks are distinct */
		do {
			urcu_txn_begin(&txn);
			sample_lane(&txn, &me->st, &in, &pub);
			for (k = 0; k < width; k++) {
				void **s = &slot[pick[k]];
				void *v = urcu_txn_load(&txn, s, URCU_MCAS_TAG);

				urcu_txn_store(&txn, s, v,
						(void *) ((uintptr_t) v + 2),
						URCU_MCAS_TAG);
			}
			st = urcu_txn_commit(&txn);
		} while (attempt_done(&txn, &me->st, st));
		if (st == URCU_TXN_STATUS_OK) {
			me->st.commits++;
			me->st.applied += width;
		}
		rcu_quiescent_state();
		rcu_thread_offline();
	}
	rcu_thread_online();
	rcu_unregister_thread();
	free(pick);
	return NULL;
}

static void usage(char *argv[])
{
	diag("Usage: %s [--nslots N] [--nsmall N] [--nwide N] [--width N]", argv[0]);
	diag("          [--duration MS] [--disjoint 0|1] [--domain 0|1]");
	diag("  --width N   edges per wide txn (default: --nslots).  Sets the wide");
	diag("              op's escalation budget: cost 2*width, budget %lu retries.",
		wide_budget());
	diag("  --domain 0  disable the escalation lane (the control): the wide");
	diag("              transactions then never finish.");
	exit(1);
}

int main(int argc, char *argv[])
{
	struct arg *sa, *wa;
	pthread_t *stid, *wtid;
	struct stats S, W;
	long long sum = 0, expect;
	int i;

	for (i = 1; i < argc; i++) {
		if (!strcmp(argv[i], "--nslots") && i + 1 < argc)
			nslots = (int) strtol(argv[++i], NULL, 0);
		else if (!strcmp(argv[i], "--nsmall") && i + 1 < argc)
			nsmall = (int) strtol(argv[++i], NULL, 0);
		else if (!strcmp(argv[i], "--nwide") && i + 1 < argc)
			nwide = (int) strtol(argv[++i], NULL, 0);
		else if (!strcmp(argv[i], "--width") && i + 1 < argc)
			width = (int) strtol(argv[++i], NULL, 0);
		else if (!strcmp(argv[i], "--duration") && i + 1 < argc)
			duration_ms = strtol(argv[++i], NULL, 0);
		else if (!strcmp(argv[i], "--disjoint") && i + 1 < argc)
			use_disjoint = (int) strtol(argv[++i], NULL, 0);
		else if (!strcmp(argv[i], "--domain") && i + 1 < argc)
			use_domain = (int) strtol(argv[++i], NULL, 0);
		else
			usage(argv);
	}
	if (!width)
		width = nslots;		/* the wide txn touches EVERY slot */

	plan_tests(NR_TESTS);

	if (nslots < 2 || nsmall < 1 || nwide < 1 || width < 2 || width > nslots)
		usage(argv);

	slot = calloc((size_t) nslots, sizeof(*slot));
	sa = calloc((size_t) nsmall, sizeof(*sa));
	wa = calloc((size_t) nwide, sizeof(*wa));
	stid = calloc((size_t) nsmall, sizeof(*stid));
	wtid = calloc((size_t) nwide, sizeof(*wtid));
	if (!slot || !sa || !wa || !stid || !wtid) {
		diag("out of memory");
		return EXIT_FAILURE;
	}
	memset(&S, 0, sizeof(S));
	memset(&W, 0, sizeof(W));
	urcu_txn_domain_init(&g_dom);

	diag("slots: %d  small: %d  wide: %d  width: %d  disjoint: %s  domain: %s",
		nslots, nsmall, nwide, width, use_disjoint ? "on" : "off",
		use_domain ? "on" : "OFF (control)");
	diag("escalation is REACTIVE ONLY: budget = cost * %d/%d",
		URCU_TXN_FALLBACK_PER_COST_NUM, URCU_TXN_FALLBACK_PER_COST_DEN);
	diag("wide op: cost %d (loads %d + records %d) => budget %lu retries  |  URCU_MCAS_ESCALATE=%d",
		2 * width, width, width, wide_budget(), URCU_MCAS_ESCALATE);

	rcu_register_thread();
	rcu_thread_offline();

	for (i = 0; i < nwide; i++) {
		wa[i].id = i;
		if (pthread_create(&wtid[i], NULL, wide_writer, &wa[i]))
			return EXIT_FAILURE;
	}
	for (i = 0; i < nsmall; i++) {
		sa[i].id = i;
		if (pthread_create(&stid[i], NULL, small_writer, &sa[i]))
			return EXIT_FAILURE;
	}

	usleep((useconds_t) duration_ms * 1000);
	uatomic_store(&g_stop, 1, CMM_RELAXED);
	for (i = 0; i < nwide; i++)
		(void) pthread_join(wtid[i], NULL);
	for (i = 0; i < nsmall; i++)
		(void) pthread_join(stid[i], NULL);

	rcu_thread_online();
	rcu_barrier();			/* flush outstanding call_rcu reclamations */
	rcu_unregister_thread();

#define ACC(dst, src) do {						\
		(dst).commits += (src).commits;				\
		(dst).aborts += (src).aborts;				\
		(dst).errors += (src).errors;				\
		(dst).abandoned += (src).abandoned;			\
		(dst).lane_entries += (src).lane_entries;		\
		(dst).as_initiator += (src).as_initiator;		\
		(dst).as_joiner += (src).as_joiner;			\
		(dst).promotions += (src).promotions;			\
		(dst).applied += (src).applied;				\
		if ((src).max_retry > (dst).max_retry)			\
			(dst).max_retry = (src).max_retry;		\
	} while (0)
	for (i = 0; i < nsmall; i++)
		ACC(S, sa[i].st);
	for (i = 0; i < nwide; i++)
		ACC(W, wa[i].st);
#undef ACC

	diag("SMALL commits: %lld  aborts: %lld  max retry: %lu",
		S.commits, S.aborts, S.max_retry);
	diag("WIDE  commits: %lld  aborts: %lld  retries/commit: %.1f  max retry: %lu",
		W.commits, W.aborts,
		W.commits ? (double) W.aborts / (double) W.commits : 0.0,
		W.max_retry);
	diag("FUNNEL wide: lane entries %lld (initiator %lld, joiner %lld), promotions %lld",
		W.lane_entries, W.as_initiator, W.as_joiner, W.promotions);
	diag("FUNNEL small: lane entries %lld (initiator %lld, joiner %lld), promotions %lld",
		S.lane_entries, S.as_initiator, S.as_joiner, S.promotions);
	diag("shutdown: abandoned mid-retry %lld (small) %lld (wide); memory errors %lld",
		S.abandoned, W.abandoned, S.errors + W.errors);

	/*
	 * Conservation.  Every commit added 2 to each slot it touched, so the
	 * slots must account for exactly the edges the writers believe they
	 * applied.  An edge lost or double-applied through the funnel lands
	 * here.
	 */
	for (i = 0; i < nslots; i++)
		sum += (long long) ((uintptr_t) slot[i] / 2);
	expect = S.applied + W.applied;
	ok(sum == expect,
		"conservation: the slots account for exactly the committed edges (%lld == %lld)",
		sum, expect);

	/*
	 * The small stream must actually be a stream, or nothing starves and
	 * the rest of this test proves nothing.
	 */
	ok(S.commits > 0, "the small single-edge writers made progress (%lld commits)",
		S.commits);

	if (!use_domain) {
		skip(3, "the escalation lane is DISABLED (--domain 0, the control): "
			"wide committed %lld, max retry %lu", W.commits, W.max_retry);
		goto out;
	}

	/*
	 * THE POINT.  The wide transaction starved past the real, default
	 * threshold and escalated on its own merits -- the reactive trigger,
	 * which no forced-threshold unit test can reach.
	 */
	ok(W.max_retry >= wide_budget() && W.as_initiator > 0,
		"reactive escalation FIRED: a wide txn starved past its cost-scaled budget (max retry %lu >= %lu) and initiated %lld episode(s)",
		W.max_retry, wide_budget(), W.as_initiator);

	/*
	 * And the episode funnelled the others: that is what closes the
	 * optimistic-writer set and bounds the starved op's progress.
	 */
	ok(W.as_joiner + S.as_joiner > 0,
		"the episode FUNNELLED other writers into the lane (%lld joiner entries)",
		W.as_joiner + S.as_joiner);

	/*
	 * And it worked: the starved transaction actually committed.  Without
	 * the lane it does not (run --domain 0 to watch wide commits collapse
	 * to a handful while max retry runs to six figures).
	 */
	ok(W.commits > 0,
		"the starved wide transactions COMMITTED (%lld), so the lane bounded their progress",
		W.commits);
out:
	free(slot);
	free(sa);
	free(wa);
	free(stid);
	free(wtid);
	return exit_status();
}
