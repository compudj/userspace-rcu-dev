// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

#ifndef _URCU_FLIP_LATCH_TXN_LOCKFREE_H
#define _URCU_FLIP_LATCH_TXN_LOCKFREE_H

/*
 * Transaction front-end for the lock-free flip-latch.
 *
 * <urcu/flip-latch-lockfree.h> is the multi-word CAS engine
 * (urcu_flip_lf_mcas_*): a set of {slot, old, new} records committed atomically.
 * This header wraps it in a begin / store / commit / end transaction whose state
 * is a small on-stack handle, so a mutator reads and buffers writes imperatively
 * while the engine handles the retry bookkeeping.  Loads are plain
 * urcu_flip_lf_read() calls inside the bracket; begin/end mark the scope, and
 * only writes are buffered:
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
 * The handle holds only what the engine does not: the cross-attempt retry count
 * (aging priority) and a pointer to this attempt's descriptor -- the write-set
 * itself lives once, in the engine descriptor, not in a second buffer.  The
 * descriptor is allocated lazily on the first store and grown as needed, so the
 * write-set is dynamically sized (no fixed cap) and an attempt that reads and
 * bails before storing allocates nothing.
 *
 * Retry / aging.  The handle carries a retry count, advanced on each contention
 * abort and threaded into the descriptor as the aging priority, so a starved
 * transaction climbs in priority without the caller threading anything.
 *
 * Writes.  urcu_flip_lf_txn_store() buffers a write whose old the caller
 * supplies.  Only writes are committed and validated: the MCAS install checks
 * each slot == old, which covers every read that became a write-old (read
 * subset of write for structural mutations).  There is no read-set.  A store can
 * fail to allocate; rather than make the caller check every store, the failure
 * is sticky -- the pending commit then reports -ENOMEM -- so a mutator only has
 * to test commit's result (which it already does).
 *
 * Reserve.  A mutator that knows its edge count up front may call
 * urcu_flip_lf_txn_reserve() right after begin: it allocates the descriptor to
 * that floor, so an OOM is reported before the mutator builds any nodes, and
 * later attempts start pre-sized rather than growing into it.  Optional -- store()
 * allocates lazily and grows on its own without it.
 *
 * Wait-free fallback.  A wait-free escalation lane (an exclusive FIFO turn
 * taken once the retry count crosses a threshold) is NOT implemented; the
 * bracket and the retry count are its intended insertion points.
 *
 * RCU.  The bracket opens an RCU read-side section per attempt, and commit
 * uses the flavor's call_rcu, so include this header AFTER an RCU flavor
 * header (e.g. <urcu-qsbr.h>); register threads and pass through quiescent
 * states as usual.
 */

#include <errno.h>

#include <urcu/compiler.h>
#include <urcu/flip-latch-lockfree.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Default capacity the descriptor is created with on the first store of an
 * attempt, when the handle carries no floor (txn->min_alloc == 0); it grows
 * (doubling) from there, so this only sets the no-realloc fast path for small
 * transactions.
 */
#ifndef URCU_FLIP_LF_TXN_INIT
#define URCU_FLIP_LF_TXN_INIT	4
#endif

/*
 * Sticky out-of-memory marker parked in txn->mcas by a failed store: distinct
 * from NULL (no write buffered yet) and from any real descriptor, so commit can
 * tell "nothing to do" from "a store could not allocate".
 */
#define URCU_FLIP_LF_TXN_ENOMEM	((struct urcu_flip_lf_mcas *) -1L)

struct urcu_flip_lf_txn {
	unsigned long retry;		/* attempts so far; aging priority for the MCAS */
	unsigned int min_alloc;		/* floor for the attempt's initial descriptor
					 * capacity, grown past if exceeded (0 -> INIT
					 * default).  Set via reserve(); refreshed to the
					 * realized size at commit so retries don't re-grow. */
	struct urcu_flip_lf_mcas *mcas;	/* this attempt's descriptor: NULL (none yet),
					 * a live descriptor, or the ENOMEM marker */
};

/* Initialize a handle before its retry loop (retry := 0, no reservation). */
static inline
void urcu_flip_lf_txn_init(struct urcu_flip_lf_txn *txn)
{
	txn->retry = 0;
	txn->min_alloc = 0;
	txn->mcas = NULL;
}

/* Begin one attempt: clear the write-set and open the RCU read-side section. */
static inline
void urcu_flip_lf_txn_begin(struct urcu_flip_lf_txn *txn)
{
	txn->mcas = NULL;		/* prior attempt's descriptor already consumed/freed */
	rcu_read_lock();
}

/*
 * Pre-reserve room for @n writes this attempt and record @n as the min_alloc
 * floor.  Allocates the descriptor up front, so an algorithm that knows its edge
 * count can fail early with -ENOMEM -- before building the nodes it meant to
 * link -- rather than discovering the failure partway through its stores.  The
 * floor also sizes every later attempt's initial descriptor, so retries start at
 * @n instead of growing into it.  Returns 0, or -ENOMEM (sticky: the pending
 * commit then also returns -ENOMEM).  Optional; call after begin, before the
 * first store.  store() still grows the descriptor should the write-set exceed @n.
 */
static inline
int urcu_flip_lf_txn_reserve(struct urcu_flip_lf_txn *txn, unsigned int n)
{
	struct urcu_flip_lf_mcas *m;

	txn->min_alloc = n;
	if (caa_unlikely(txn->mcas == URCU_FLIP_LF_TXN_ENOMEM))
		return -ENOMEM;		/* sticky: an earlier alloc already failed */
	if (!n)
		return 0;		/* no reservation; the INIT default applies */
	if (!txn->mcas) {
		m = urcu_flip_lf_mcas_create(n, txn->retry);
		if (caa_unlikely(!m)) {
			txn->mcas = URCU_FLIP_LF_TXN_ENOMEM;
			return -ENOMEM;
		}
		txn->mcas = m;
		return 0;
	}
	while (txn->mcas->cap < n) {	/* already buffering: grow to fit @n */
		m = urcu_flip_lf_mcas_grow(txn->mcas);
		if (caa_unlikely(!m)) {
			urcu_flip_lf_mcas_destroy(txn->mcas);	/* unpublished: sync free */
			txn->mcas = URCU_FLIP_LF_TXN_ENOMEM;
			return -ENOMEM;
		}
		txn->mcas = m;
	}
	return 0;
}

/*
 * Buffer a write {*slot: old -> new}.  @old_ptr is the value the caller saw.
 * Returns 0, or -ENOMEM if the descriptor could not be allocated or grown -- in
 * which case the failure is recorded so the pending commit also returns -ENOMEM;
 * the caller may therefore ignore this return and test only commit.
 */
static inline
int urcu_flip_lf_txn_store(struct urcu_flip_lf_txn *txn, void **slot,
		void *old_ptr, void *new_ptr)
{
	struct urcu_flip_lf_mcas *m = txn->mcas;

	if (caa_unlikely(m == URCU_FLIP_LF_TXN_ENOMEM))
		return -ENOMEM;		/* sticky: an earlier store already failed */
	if (!m) {
		m = urcu_flip_lf_mcas_create(txn->min_alloc ? txn->min_alloc :
				URCU_FLIP_LF_TXN_INIT, txn->retry);
		if (caa_unlikely(!m)) {
			txn->mcas = URCU_FLIP_LF_TXN_ENOMEM;
			return -ENOMEM;
		}
		txn->mcas = m;
	}
	if (caa_unlikely(!urcu_flip_lf_mcas_add(m, slot, old_ptr, new_ptr))) {
		/* Descriptor full: grow (may move it) and retry the add. */
		m = urcu_flip_lf_mcas_grow(m);
		if (caa_unlikely(!m)) {
			urcu_flip_lf_mcas_destroy(txn->mcas);	/* unpublished: sync free */
			txn->mcas = URCU_FLIP_LF_TXN_ENOMEM;
			return -ENOMEM;
		}
		txn->mcas = m;
		urcu_flip_lf_mcas_add(m, slot, old_ptr, new_ptr);	/* room now */
	}
	return 0;
}

/*
 * Commit the buffered write-set through the MCAS.  Returns 1 on commit, 0 on a
 * contention abort (the caller re-runs begin..commit; the retry count is
 * advanced internally), or a negative errno on error (-ENOMEM, including a store
 * that could not allocate).  Reclaim is deferred through the flavor's call_rcu.
 * Call between begin and end.
 */
static inline
int urcu_flip_lf_txn_commit(struct urcu_flip_lf_txn *txn)
{
	struct urcu_flip_lf_mcas *m = txn->mcas;

	if (caa_unlikely(m == URCU_FLIP_LF_TXN_ENOMEM)) {
		txn->mcas = NULL;
		return -ENOMEM;
	}
	if (!m)
		return 1;		/* empty write-set: trivially committed */
	txn->min_alloc = m->nr;		/* learn the realized size: a retry won't re-grow */
	txn->mcas = NULL;		/* mcas_commit consumes the descriptor */
	if (urcu_flip_lf_mcas_commit(m, call_rcu))
		return 1;
	txn->retry++;			/* aged for the next attempt */
	return 0;
}

/* End the attempt: close the RCU read-side section.  Always pair with begin. */
static inline
void urcu_flip_lf_txn_end(struct urcu_flip_lf_txn *txn)
{
	struct urcu_flip_lf_mcas *m = txn->mcas;

	/*
	 * A live descriptor survives to here only when the attempt buffered
	 * stores but bailed before commit; it was never parked, so free it
	 * synchronously.  After commit, mcas is NULL (consumed); after a store
	 * OOM it is the marker -- neither needs freeing.
	 */
	if (m && m != URCU_FLIP_LF_TXN_ENOMEM)
		urcu_flip_lf_mcas_destroy(m);
	txn->mcas = NULL;
	rcu_read_unlock();
}

#ifdef __cplusplus
}
#endif

#endif	/* _URCU_FLIP_LATCH_TXN_LOCKFREE_H */
