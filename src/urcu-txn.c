// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

/*
 * urcu-txn: shared library state of the RCU transaction engines.
 *
 * The engines themselves -- <urcu/rcu-txn-mcas.h> (concurrent MCAS) and
 * <urcu/rcu-txn-sw.h> (single-writer) -- are header-inline.  What lives here
 * is their only shared MUTABLE state: one per-CPU size-classed descriptor slab
 * per engine (<urcu/rcu-txn-slab.h>), plus the constructor initializing both.
 * The single-writer engine also keeps its two slow paths here, out of line: the
 * commit that installs proxies and the growth of a full handle.
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

#ifndef _LGPL_SOURCE
#define _LGPL_SOURCE
#endif

#include <urcu/rcu-txn-sw.h>
#include <urcu/rcu-txn-mcas.h>	/* engine layer only (flavor-free) */

#include "rcu-txn-sw-internal.h"

struct urcu_slab urcu_txn_sw_slab;
struct urcu_slab urcu_txn_slab;

/*
 * The commit of a single-writer transaction that installs proxies: see
 * <urcu/rcu-txn-sw.h>.
 */
void urcu_txn_sw__commit_desc(struct urcu_txn_sw_desc *desc, unsigned int nr,
		void (*call_rcu_fn)(struct rcu_head *,
			void (*)(struct rcu_head *)),
		const struct rcu_flavor_struct *flavor)
{
	struct urcu_txn_sw_record *records = desc->records;
	unsigned int i;

	urcu_txn_sw__assert_distinct_slots(records, nr);
	/* Install: readers still resolve to old, the selector being 0. */
	for (i = 0; i < nr; i++) {
		urcu_txn_sw__excl_slot_parkable(&records[i]);
		urcu_txn_sw__record_install(&desc->group, &records[i]);
	}
	/* Flip: every proxy resolves to new. */
	urcu_txn_sw__group_commit(&desc->group);
	/* Settle. */
	for (i = 0; i < nr; i++) {
		urcu_txn_sw__excl_slot_ours(&records[i]);
		urcu_txn_sw__record_settle(&records[i]);
	}
	urcu_txn_sw__retire(desc, call_rcu_fn, flavor);
}

/* See <urcu/rcu-txn-sw.h>. */
bool urcu_txn_sw__grow(struct urcu_txn_sw_txn *t)
{
	return urcu_txn_sw__resize(t, t->cap ? t->cap * 2 : URCU_TXN_SW_CAP);
}

/* Byte size per record-count class; filled at init, must outlive the slab. */
static size_t urcu_txn_sw_slab_bytes[URCU_TXN_SW_SLAB_NCLASS];
static size_t urcu_txn_slab_bytes[URCU_TXN_SLAB_NCLASS];

static __attribute__((constructor))
void urcu_txn_slab_ctor(void)
{
	int i;

#ifdef URCU_SLAB_RSEQ
	/*
	 * librseq requires rseq_init() once per process before rseq_offset and
	 * rseq_size mean anything.  liburcu never called it: it relied on
	 * something else in the process happening to, which is not a missed
	 * optimization but a crash.  rseq_registered() is what gates every rseq
	 * path here, and until initialization runs, rseq_size holds -1U while
	 * rseq_offset holds PTRDIFF_MIN -- so the gate opened and the first
	 * per-cpu allocation dereferenced an rseq area built from a garbage
	 * offset.
	 *
	 * The return value needs no inspection.  Every way this can fail leaves
	 * rseq_size at a value rseq_registered() rejects: -1U when the libc does
	 * not export the rseq symbols, 0 when registration was refused.  It must
	 * however run BEFORE urcu_slab_init(), which consults rseq_registered()
	 * to decide whether to register the expedited-rseq membarrier.
	 */
	(void) rseq_init();
#endif
	for (i = 0; i < URCU_TXN_SW_SLAB_NCLASS; i++)
		urcu_txn_sw_slab_bytes[i] =
				urcu_txn_sw__desc_size(urcu_txn_sw_slab_rc[i]);
	/*
	 * Link the slab's lists through each block's rcu_head, never offset 0:
	 * a block enters the pending list at commit, a grace period before
	 * readers are done with it, and offset 0 is live reader state
	 * (urcu_txn_sw_desc::group / urcu_txn_desc::status).  rcu_head is the
	 * one field nobody reads -- call_rcu writes it at deferral time for
	 * exactly this reason.
	 */
	urcu_slab_init(&urcu_txn_sw_slab, urcu_txn_sw_slab_bytes,
			URCU_TXN_SW_SLAB_NCLASS, "txn_sw",
			offsetof(struct urcu_txn_sw_desc, rcu_head));
	for (i = 0; i < URCU_TXN_SLAB_NCLASS; i++)
		urcu_txn_slab_bytes[i] =
				urcu_txn_blocksize(urcu_txn_slab_rc[i]);
	urcu_slab_init(&urcu_txn_slab, urcu_txn_slab_bytes,
			URCU_TXN_SLAB_NCLASS, "txn",
			offsetof(struct urcu_txn_desc, rcu_head));
}
