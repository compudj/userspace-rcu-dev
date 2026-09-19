// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

/*
 * urcu-txn: shared library state of the RCU transaction engines.
 *
 * The engines themselves -- <urcu/rcu-txn-mcas.h> (concurrent MCAS) and
 * <urcu/rcu-txn-sw.h> (single-updater) -- are header-inline.  What lives here
 * is their only shared MUTABLE state: one per-CPU size-classed descriptor slab
 * per engine (<urcu/rcu-txn-slab.h>), plus the constructor initializing both.
 *
 * Defining the instances once, in liburcu-common, makes every TU that includes
 * an engine header share one slab per engine: freelists recycle blocks across
 * TUs and the arena footprint (nclass x ncpu) is paid once per process.  A
 * header-static definition would instead hand each including TU its own arenas
 * and superblocks -- cross-TU frees would still be safe (origin-arena lookup
 * from the superblock header), but nothing would ever share.
 *
 * Running the constructor here also narrows the init-ordering window: an ELF
 * object's dependencies are initialized before it, so any application or
 * library constructor that commits a transaction finds the slabs already
 * initialized when linked against the shared liburcu-common.  (Under static
 * linking the ordering among archive members and application TUs is weaker; a
 * transaction committed before this constructor runs simply sees the slab
 * disabled and falls back to exact posix_memalign blocks.)
 */

#ifndef _GNU_SOURCE
#define _GNU_SOURCE		/* sched_getcpu (rcu-txn-slab.h) */
#endif
#ifndef _LGPL_SOURCE
#define _LGPL_SOURCE
#endif

#include <stdio.h>
#include <urcu/rcu-txn-sw.h>
#include <urcu/rcu-txn-mcas.h>	/* engine layer only (flavor-free) */

struct urcu_slab urcu_txn_sw_slab;
struct urcu_slab urcu_txn_slab;

/* Byte size per record-count class; filled at init, must outlive the slab. */
static size_t urcu_txn_sw_slab_bytes[URCU_TXN_SW_SLAB_NCLASS];
static size_t urcu_txn_slab_bytes[URCU_TXN_SLAB_NCLASS];

static __attribute__((constructor))
void urcu_txn_slab_ctor(void)
{
	int i;

	for (i = 0; i < URCU_TXN_SW_SLAB_NCLASS; i++)
		urcu_txn_sw_slab_bytes[i] =
				urcu_txn_sw_blocksize(urcu_txn_sw_slab_rc[i]);
	urcu_slab_init(&urcu_txn_sw_slab, urcu_txn_sw_slab_bytes,
			URCU_TXN_SW_SLAB_NCLASS, "txn_sw");
	for (i = 0; i < URCU_TXN_SLAB_NCLASS; i++)
		urcu_txn_slab_bytes[i] =
				urcu_txn_blocksize(urcu_txn_slab_rc[i]);
	urcu_slab_init(&urcu_txn_slab, urcu_txn_slab_bytes,
			URCU_TXN_SLAB_NCLASS, "txn");
}

/* See urcu_txn_in_fallback(): the invariant this exists to let embedders assert. */
__thread int urcu_txn_fb_depth;

/*
 * ☠ A DESTRUCTOR CENSUS CANNOT SURVIVE THE FAILURE IT MEASURES.  The regime
 * worth measuring here is the one where the process is SIGKILLed by a memcg --
 * and SIGKILL runs no destructor, so every number this file collects is lost
 * exactly when it matters.  Measured: the 64 GiB arm died with no census at all.
 *
 * So dump on a timer as well.  URCU_CENSUS_MS=N starts a detached thread that
 * prints every N ms; the last line before the kill is then the reading from
 * INSIDE the blowup rather than from the healthy run that survived to exit.
 */
#ifdef URCU_TXN_CENSUS_PERIODIC
#include <pthread.h>

void urcu_txn_census_dump(void);		/* prints whatever is compiled in */

static void *urcu_txn_census_thread(void *arg)
{
	unsigned long ms = (unsigned long) (uintptr_t) arg;

	for (;;) {
		struct timespec ts = {
			.tv_sec = (time_t) (ms / 1000),
			.tv_nsec = (long) (ms % 1000) * 1000000L,
		};

		nanosleep(&ts, NULL);
		urcu_txn_census_dump();
	}
	return NULL;
}

static __attribute__((constructor))
void urcu_txn_census_init(void)
{
	const char *e = getenv("URCU_CENSUS_MS");
	unsigned long ms;
	pthread_attr_t attr;
	pthread_t th;

	if (!e)
		return;
	ms = strtoul(e, NULL, 10);
	if (!ms)
		return;
	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
	if (pthread_create(&th, &attr, urcu_txn_census_thread,
			(void *) (uintptr_t) ms))
		fprintf(stderr, "# URCU census thread failed to start\n");
	pthread_attr_destroy(&attr);
	fprintf(stderr, "# URCU census armed: every %lu ms\n", ms);
}
#endif /* URCU_TXN_CENSUS_PERIODIC */

#ifdef URCU_TXN_SEMANTIC_RETRY
unsigned long urcu_txn_sem_wait, urcu_txn_sem_won, urcu_txn_sem_capped;
unsigned long urcu_txn_sem_patience = URCU_TXN_WAIT_PATIENCE;

static __attribute__((constructor))
void urcu_txn_sem_init(void)
{
	const char *e = getenv("URCU_TXN_SEM_PATIENCE");

	if (e)
		urcu_txn_sem_patience = strtoul(e, NULL, 10);
	fprintf(stderr, "# URCU_TXN SEMANTIC RETRY armed: patience=%lu%s\n",
		urcu_txn_sem_patience,
		urcu_txn_sem_patience ? " (back-off)" : " (BLOCKING)");
}

static __attribute__((destructor))
void urcu_txn_sem_report(void)
{
	unsigned long w = uatomic_load(&urcu_txn_sem_wait, CMM_RELAXED);
	unsigned long k = uatomic_load(&urcu_txn_sem_won, CMM_RELAXED);
	unsigned long c = uatomic_load(&urcu_txn_sem_capped, CMM_RELAXED);

	if (!(w | k | c))
		return;
	fprintf(stderr, "# URCU_TXN SEMANTIC RETRY: engaged=%lu won=%lu capped=%lu won%%=%.1f\n",
		w, k, c, 100.0 * (double) k / (double) (w + 1));
}
#endif /* URCU_TXN_SEMANTIC_RETRY */

#ifdef URCU_TXN_DEBUG_ABORT_PLANTED
unsigned long urcu_txn_abort_planted0;
unsigned long urcu_txn_abort_plantedN;
unsigned long urcu_txn_abort_lost_validate;
unsigned long urcu_txn_abort_lost_write;

static __attribute__((destructor))
void urcu_txn_abort_planted_report(void)
{
	unsigned long z = uatomic_load(&urcu_txn_abort_planted0, CMM_RELAXED);
	unsigned long n = uatomic_load(&urcu_txn_abort_plantedN, CMM_RELAXED);

	if (!(z | n))
		return;
	fprintf(stderr, "# URCU_TXN ABORT reclaim: planted==0 (owes NO grace period)=%lu "
		"planted>0 (owes one)=%lu  wasted%%=%.1f\n",
		z, n, 100.0 * (double) z / (double) (z + n + 1));
	fprintf(stderr, "# URCU_TXN ABORT kind: lost a VALIDATE (guard)=%lu lost a WRITE=%lu validate%%=%.1f\n",
		uatomic_load(&urcu_txn_abort_lost_validate, CMM_RELAXED),
		uatomic_load(&urcu_txn_abort_lost_write, CMM_RELAXED),
		100.0 * (double) uatomic_load(&urcu_txn_abort_lost_validate, CMM_RELAXED) /
		(double) (uatomic_load(&urcu_txn_abort_lost_validate, CMM_RELAXED) +
			uatomic_load(&urcu_txn_abort_lost_write, CMM_RELAXED) + 1));
}
#endif /* URCU_TXN_DEBUG_ABORT_PLANTED */

#ifdef URCU_TXN_SETTLE_DELAY
/*
 * Width of the settle delay seam, in cpu_relax spins.  Read once; 0 = inert.
 */
unsigned long urcu_txn_settle_delay_spins;

static __attribute__((constructor))
void urcu_txn_settle_delay_init(void)
{
	const char *e = getenv("URCU_TXN_SETTLE_DELAY_SPINS");

	if (e)
		urcu_txn_settle_delay_spins = strtoul(e, NULL, 10);
	fprintf(stderr, "# URCU_TXN_SETTLE_DELAY armed: spins=%lu\n",
		urcu_txn_settle_delay_spins);
}
#endif /* URCU_TXN_SETTLE_DELAY */

#ifdef URCU_TXN_DEBUG_SETTLE
/*
 * Counters for the settle-premise check (urcu_txn_dbg_parked_check).
 * Diagnostic only, and defined here for the same reason the other engine
 * instances are: one definition in liburcu-common rather than one per TU.
 */
__thread void *urcu_txn_dbg_foreign_slot[URCU_TXN_DBG_FOREIGN_MAX];
__thread void *urcu_txn_dbg_foreign_cur[URCU_TXN_DBG_FOREIGN_MAX];
__thread unsigned int urcu_txn_dbg_foreign_n;
unsigned long urcu_txn_dbg_settle_checked;
unsigned long urcu_txn_dbg_settle_foreign;
unsigned long urcu_txn_dbg_settle_sibling;

/*
 * The census.  checked is what makes a zero FOREIGN mean something: a zero with
 * checked=0 says the settle never ran, not that the premise held.
 */
__attribute__((destructor))
static void urcu_txn_dbg_settle_report(void)
{
	fprintf(stderr,
		"# URCU_TXN_SETTLE checked=%lu FOREIGN=%lu same_slot_sibling=%lu\n",
		uatomic_load(&urcu_txn_dbg_settle_checked, CMM_RELAXED),
		uatomic_load(&urcu_txn_dbg_settle_foreign, CMM_RELAXED),
		uatomic_load(&urcu_txn_dbg_settle_sibling, CMM_RELAXED));
}
#endif /* URCU_TXN_DEBUG_SETTLE */

#ifdef URCU_TXN_CENSUS_PERIODIC
/*
 * The periodic dump: whatever counters this build compiled in.  Same TU as the
 * destructors, so it simply calls them -- one definition of each line, whether
 * it is printed on a timer or at exit.
 */
void urcu_txn_census_dump(void)
{
#ifdef URCU_TXN_DEBUG_ABORT_PLANTED
	urcu_txn_abort_planted_report();
#endif
#ifdef URCU_TXN_SEMANTIC_RETRY
	urcu_txn_sem_report();
#endif
#ifdef URCU_TXN_DEBUG_SETTLE
	urcu_txn_dbg_settle_report();
#endif
}
#endif /* URCU_TXN_CENSUS_PERIODIC */
