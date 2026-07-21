// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

/*
 * urcu-txn: shared library state of the RCU transaction engines.
 *
 * The engines themselves -- <urcu/rcu-mcas.h> (concurrent MCAS) and
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

#include <urcu/rcu-mcas.h>
#include <urcu/rcu-txn-sw.h>
#include <urcu/rcu-mcas-sw-mw.h>	/* engine layer only (flavor-free) */

struct urcu_slab urcu_mcas_slab;
struct urcu_slab urcu_txn_sw_slab;
struct urcu_slab urcu_txn_sw_mw_slab;

/* Byte size per record-count class; filled at init, must outlive the slab. */
static size_t urcu_mcas_slab_bytes[URCU_MCAS_SLAB_NCLASS];
static size_t urcu_txn_sw_slab_bytes[URCU_TXN_SW_SLAB_NCLASS];
static size_t urcu_txn_sw_mw_slab_bytes[URCU_TXN_SW_MW_SLAB_NCLASS];

static __attribute__((constructor))
void urcu_txn_slab_ctor(void)
{
	int i;

	for (i = 0; i < URCU_MCAS_SLAB_NCLASS; i++)
		urcu_mcas_slab_bytes[i] = urcu_mcas_blocksize(urcu_mcas_slab_rc[i]);
	urcu_slab_init(&urcu_mcas_slab, urcu_mcas_slab_bytes,
			URCU_MCAS_SLAB_NCLASS, "mcas");
	for (i = 0; i < URCU_TXN_SW_SLAB_NCLASS; i++)
		urcu_txn_sw_slab_bytes[i] =
				urcu_txn_sw_blocksize(urcu_txn_sw_slab_rc[i]);
	urcu_slab_init(&urcu_txn_sw_slab, urcu_txn_sw_slab_bytes,
			URCU_TXN_SW_SLAB_NCLASS, "txn_sw");
	for (i = 0; i < URCU_TXN_SW_MW_SLAB_NCLASS; i++)
		urcu_txn_sw_mw_slab_bytes[i] =
				urcu_txn_sw_mw_blocksize(urcu_txn_sw_mw_slab_rc[i]);
	urcu_slab_init(&urcu_txn_sw_mw_slab, urcu_txn_sw_mw_slab_bytes,
			URCU_TXN_SW_MW_SLAB_NCLASS, "txn_sw_mw");
}
