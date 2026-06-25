// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

#ifndef _URCU_FLIP_LATCH_TXN_LOCKFREE_H
#define _URCU_FLIP_LATCH_TXN_LOCKFREE_H

/*
 * Transaction front-end for the lock-free flip-latch.
 *
 * <urcu/flip-latch-lockfree.h> is the multi-word CAS engine
 * (urcu_flip_lf_mcas_*): a frozen set of {slot, old, new} records committed
 * atomically.  This header wraps it in a begin / store / commit / end
 * transaction whose state is a small on-stack handle, so a mutator reads and
 * buffers writes imperatively while the engine handles the retry bookkeeping.
 * Loads are plain urcu_flip_lf_read() calls inside the bracket; begin/end mark
 * the scope, and only writes are buffered:
 *
 *     struct urcu_flip_lf_txn txn;
 *     int ret;
 *
 *     urcu_flip_lf_txn_init(&txn);
 *     do {
 *         urcu_flip_lf_txn_begin(&txn);
 *         succ = urcu_flip_lf_read((void **) &pos->next);
 *         if (is_marked(succ)) { urcu_flip_lf_txn_end(&txn); return -ENOENT; }
 *         urcu_flip_lf_txn_store(&txn, (void **) &pos->next, succ, newp);
 *         urcu_flip_lf_txn_store(&txn, (void **) &succ->prev, pos,  newp);
 *         ret = urcu_flip_lf_txn_commit(&txn);
 *         urcu_flip_lf_txn_end(&txn);
 *     } while (ret == 0);    // 0 = retry, 1 = committed, <0 = error
 *
 * Retry / aging.  The handle carries a retry count, advanced on each contention
 * abort and threaded into the MCAS as the aging priority, so a starved
 * transaction climbs in priority without the caller threading anything.
 *
 * Writes.  urcu_flip_lf_txn_store() buffers a write whose old the caller
 * supplies.  Only writes are committed and validated: the MCAS install checks
 * each slot == old, which covers every read that became a write-old (read
 * subset of write for structural mutations).  There is no read-set.
 *
 * Wait-free fallback.  A wait-free escalation lane (an exclusive FIFO turn
 * taken once the retry count crosses a threshold) is NOT implemented; the
 * bracket and the retry count are its intended insertion points.
 *
 * RCU.  The bracket opens an RCU read-side section per attempt, and commit
 * uses the flavor's call_rcu, so include this header AFTER an RCU flavor
 * header (e.g. <urcu-qsbr.h>); register threads and pass through quiescent
 * states as usual.
 *
 * The write-set is buffered on-stack, bounded by URCU_FLIP_LF_TXN_CAP.
 */

#include <errno.h>
#include <stdbool.h>

#include <urcu/assert.h>
#include <urcu/flip-latch-lockfree.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef URCU_FLIP_LF_TXN_CAP
#define URCU_FLIP_LF_TXN_CAP	16
#endif

struct urcu_flip_lf_txn_write {
	void **slot;
	void *old_ptr;
	void *new_ptr;
};

struct urcu_flip_lf_txn {
	unsigned long retry;		/* attempts so far; aging priority for the MCAS */
	unsigned int nr;		/* writes buffered this attempt */
	struct urcu_flip_lf_txn_write w[URCU_FLIP_LF_TXN_CAP];
};

/* Initialize a handle before its retry loop (retry := 0). */
static inline
void urcu_flip_lf_txn_init(struct urcu_flip_lf_txn *txn)
{
	txn->retry = 0;
	txn->nr = 0;
}

/* Begin one attempt: clear the write-set and open the RCU read-side section. */
static inline
void urcu_flip_lf_txn_begin(struct urcu_flip_lf_txn *txn)
{
	txn->nr = 0;
	rcu_read_lock();
}

/* Buffer a write {*slot: old -> new}.  @old_ptr is the value the caller saw. */
static inline
void urcu_flip_lf_txn_store(struct urcu_flip_lf_txn *txn, void **slot,
		void *old_ptr, void *new_ptr)
{
	struct urcu_flip_lf_txn_write *w;

	urcu_posix_assert(txn->nr < URCU_FLIP_LF_TXN_CAP);
	w = &txn->w[txn->nr++];
	w->slot = slot;
	w->old_ptr = old_ptr;
	w->new_ptr = new_ptr;
}

/*
 * Commit the buffered write-set through the MCAS.  Returns 1 on commit, 0 on a
 * contention abort (the caller re-runs begin..commit; the retry count is
 * advanced internally), or a negative errno on error (-ENOMEM).  Reclaim is
 * deferred through the flavor's call_rcu.  Call between begin and end.
 */
static inline
int urcu_flip_lf_txn_commit(struct urcu_flip_lf_txn *txn)
{
	struct urcu_flip_lf_mcas *m;
	unsigned int i;

	m = urcu_flip_lf_mcas_create(txn->nr, txn->retry);
	if (caa_unlikely(!m))
		return -ENOMEM;
	for (i = 0; i < txn->nr; i++)
		urcu_flip_lf_mcas_add(m, txn->w[i].slot, txn->w[i].old_ptr,
				txn->w[i].new_ptr);
	if (urcu_flip_lf_mcas_commit(m, call_rcu))
		return 1;
	txn->retry++;			/* aged for the next attempt */
	return 0;
}

/* End the attempt: close the RCU read-side section.  Always pair with begin. */
static inline
void urcu_flip_lf_txn_end(struct urcu_flip_lf_txn *txn)
{
	(void) txn;
	rcu_read_unlock();
}

#ifdef __cplusplus
}
#endif

#endif	/* _URCU_FLIP_LATCH_TXN_LOCKFREE_H */
