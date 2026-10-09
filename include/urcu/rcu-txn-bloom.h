// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

#ifndef _URCU_RCU_TXN_BLOOM_H
#define _URCU_RCU_TXN_BLOOM_H

/*
 * rcu-txn-bloom.h
 *
 * The RCU pseudo-transaction read-your-own-writes Bloom filter, shared by
 * the transaction engines: <urcu/rcu-txn.h> (concurrent/MCAS) and
 * <urcu/rcu-txn-sw.h> (single-writer).  This is the mechanism only: who
 * maintains the filter and when it is reset is each engine's policy; see
 * the RYW sections of those headers.
 *
 * An RCU pseudo-transaction with read-your-own-writes must decide, per
 * access, whether a slot is already in this transaction's write set.
 * The authoritative test is a linear scan of the records, so a write
 * set of n slots costs O(n) per access and O(n^2) to build -- and the
 * scan is pure overhead on the dominant case, the lookup miss.
 *
 * This Bloom filter is an optimisation to answer the miss in O(1)
 * without false negatives: a clear bit means the slot is definitely
 * absent, so the scan is skipped outright.  All k bits set means
 * present or a false positive, which falls through to the authoritative
 * find.
 *
 * Correctness never depends on the filter's presence, width or k: a
 * false positive costs the find, or at age 0 one extra attempt.  It does
 * depend on the absence of false negatives, which is an obligation on
 * the engine: the filter must be set for every slot recorded while it is
 * live (arming rebuilds it from all records, so this holds from the
 * moment it goes live).
 *
 * The state belongs to the caller: an RCU pseudo-transaction declares
 * its own uint64_t [URCU_TXN_BLOOM_WORDS] array (an on-stack handle,
 * typically -- never in a slab-sized descriptor whose size is baked
 * into an allocator) and owns zeroing it.  These helpers are pure
 * functions over it.
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * URCU_TXN_BLOOM_WORDS sets the filter width (64 bits each; default 16 = 1024
 * bits).  With k bits over m = 64*WORDS bits and n recorded slots the
 * false-positive rate is ~(1 - e^{-kn/m})^k, which for a sparse filter is
 * (kn/m)^k: doubling the width divides it by ~2^k.
 *
 * Both width and k are compile-time tunables that only ever trade filter cost
 * against the false-positive rate: correctness never depends on either.  The
 * 16-word / k=3 default is based on experimental workload measurements.
 */
#ifndef URCU_TXN_BLOOM_WORDS
# define URCU_TXN_BLOOM_WORDS	16
#endif

/*
 * URCU_TXN_BLOOM_K sets the number of hash bits a slot maps to (default 3).
 * Raising k by one multiplies the false-positive rate by the fill factor kn/m:
 * a win only while the filter is sparse.
 */
#ifndef URCU_TXN_BLOOM_K
# define URCU_TXN_BLOOM_K	3
#endif

/*
 * URCU_TXN_BLOOM_MIN is the write-set size below which the filter is not built
 * at all: an exact scan of that many records is cheaper than zeroing the filter
 * and hashing into it, and it is exact, so it also spares the caller the
 * spurious escalations a false positive would cause.  Only above this does the
 * filter start paying.  Correctness never depends on the value -- it selects
 * which of two answers-agreeing paths runs.  The sw engine keeps its own knob
 * with the same default and the same rationale (URCU_TXN_SW_BLOOM_MIN); it is
 * an independent define, so overriding this one does not move it.
 */
#ifndef URCU_TXN_BLOOM_MIN
# define URCU_TXN_BLOOM_MIN	8
#endif

#define URCU_TXN_BLOOM_BITS	(64ULL * URCU_TXN_BLOOM_WORDS)

/*
 * Two independent hashes of the slot.  A single multiply leaves the k derived
 * positions correlated (slot addresses are aligned and clustered).
 * Minimal-cost Kirsch-Mitzenmacher: run one SplitMix64 avalanche (two
 * multiplies) and split its fully-mixed 64 bits into two independent 32-bit
 * lanes -- one hash yields both h1,h2, half the cost of two separate hashes and
 * far cheaper than a multiply-free chain (Thomas Wang) whose long dependency
 * chain is slower in practice.  h2 is forced odd so the progression h1 + i*h2
 * visits k distinct bits.
 */
static inline
void urcu_txn_bloom_h1h2(void **slot, uint64_t *h1, uint64_t *h2)
{
	/*
	 * Shift by the alignment actually guaranteed.  >> 3 discards a live
	 * address bit on ILP32, where slots are only 4-byte aligned, so adjacent
	 * slots collapse onto identical k positions -- a false-positive rate the
	 * model does not predict.  On LP64 the one residual zero bit is absorbed
	 * by the SplitMix64 avalanche on the very next line.
	 */
	uint64_t x = (uint64_t) (uintptr_t) slot >> 2;

	x ^= x >> 30; x *= 0xbf58476d1ce4e5b9ULL;
	x ^= x >> 27; x *= 0x94d049bb133111ebULL;
	x ^= x >> 31;
	*h1 = x & 0xffffffffULL;		/* low lane */
	*h2 = (x >> 32) | 1;			/* high lane, odd stride */
}

static inline
bool urcu_txn_bloom_test(const uint64_t *bloom, void **slot)
{
	uint64_t h1, h2;
	unsigned int i;

	urcu_txn_bloom_h1h2(slot, &h1, &h2);
	for (i = 0; i < URCU_TXN_BLOOM_K; i++) {
		uint64_t idx = (h1 + (uint64_t) i * h2) % URCU_TXN_BLOOM_BITS;

		if (!(bloom[idx >> 6] & ((uint64_t) 1 << (idx & 63))))
			return false;	/* a clear bit: the slot is definitely absent */
	}
	return true;			/* all k bits set: present (or a false positive) */
}

static inline
void urcu_txn_bloom_set(uint64_t *bloom, void **slot)
{
	uint64_t h1, h2;
	unsigned int i;

	urcu_txn_bloom_h1h2(slot, &h1, &h2);
	for (i = 0; i < URCU_TXN_BLOOM_K; i++) {
		uint64_t idx = (h1 + (uint64_t) i * h2) % URCU_TXN_BLOOM_BITS;

		bloom[idx >> 6] |= (uint64_t) 1 << (idx & 63);
	}
}

static inline
bool urcu_txn_bloom_test_and_set(uint64_t *bloom, void **slot)
{
	uint64_t h1, h2;
	unsigned int i;
	bool was_set = true;

	urcu_txn_bloom_h1h2(slot, &h1, &h2);
	for (i = 0; i < URCU_TXN_BLOOM_K; i++) {
		uint64_t idx = (h1 + (uint64_t) i * h2) % URCU_TXN_BLOOM_BITS;
		unsigned int w = (unsigned int) (idx >> 6);
		uint64_t bit = (uint64_t) 1 << (idx & 63);

		if (!(bloom[w] & bit))
			was_set = false;
		bloom[w] |= bit;
	}
	return was_set;
}

#ifdef __cplusplus
}
#endif

#endif /* _URCU_RCU_TXN_BLOOM_H */
