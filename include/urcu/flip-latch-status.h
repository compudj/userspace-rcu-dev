// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

#ifndef _URCU_FLIP_LATCH_STATUS_H
#define _URCU_FLIP_LATCH_STATUS_H

/*
 * Common commit status for the flip-latch transaction front-ends.
 *
 * Both <urcu/flip-latch.h> (single-updater) and
 * <urcu/flip-latch-txn-lockfree.h> (lock-free) return this enum from their
 * commit() entry point, so an embedder tests a commit the same way no matter
 * which front-end it drives -- and can migrate between them without rewriting
 * its commit handling.
 *
 * Sign convention (the same one used across the tree): a value < 0 is an
 * error, == 0 is success, and > 0 is a transient outcome that is NOT an error
 * (the caller retries):
 *
 *   - MEMORY_ERROR (< 0): the transaction could not allocate; nothing was
 *     published and the txn has been consumed.  This is the error region; it
 *     is extensible to further negative codes should other errors arise.
 *   - OK (0): committed.
 *   - ABORT (> 0): a contention abort -- re-run the begin..commit bracket.
 *     Only the lock-free front-end ever returns this; the single-updater has
 *     no contention and so never aborts.
 */
enum urcu_flip_txn_status {
	URCU_FLIP_TXN_STATUS_MEMORY_ERROR = -1,	/* < 0: error (extensible) */
	URCU_FLIP_TXN_STATUS_OK           = 0,	/* committed */
	URCU_FLIP_TXN_STATUS_ABORT        = 1,	/* contention; retry. NOT an error */
};

#endif /* _URCU_FLIP_LATCH_STATUS_H */
