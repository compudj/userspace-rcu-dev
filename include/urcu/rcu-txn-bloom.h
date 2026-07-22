// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

#ifndef _URCU_RCU_TXN_BLOOM_H
#define _URCU_RCU_TXN_BLOOM_H

/*
 * rcu-txn-bloom.h
 *
 * The read-your-own-writes lookup filter shared by the transaction engines:
 * <urcu/rcu-txn.h> (concurrent/MCAS) and <urcu/rcu-txn-sw.h> (single-updater).
 * Mechanism only -- who maintains it, when it is reset, and whether it is worth
 * maintaining at all are POLICY, and each engine states its own; see the RYW
 * sections of those headers.
 *
 * The problem it solves is common to both.  An engine with read-your-own-writes
 * must decide, per access, whether a slot is already in this transaction's write
 * set.  The authoritative test is a linear scan of the records, so a write set
 * of n slots costs O(n) per access and O(n^2) to build -- and the scan is pure
 * overhead on the dominant case, the MISS.
 *
 * This filter answers the miss in O(1) and never lies about it: a clear bit
 * means the slot is DEFINITELY absent, so the scan is skipped outright.  All k
 * bits set means present OR a false positive, which falls through to the
 * authoritative find.  It can therefore only ever save the scan, never change a
 * returned value -- correctness never depends on the filter, only speed does.
 * That is what lets each engine adopt, skip, or A/B it freely.
 *
 * The state is the caller's: an engine declares its own uint64_t
 * [URCU_TXN_BLOOM_WORDS] array wherever it wants it (an on-stack handle,
 * typically -- never in a slab-sized descriptor whose size is baked into an
 * allocator) and owns zeroing it.  These helpers are pure functions over it.
 */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * URCU_TXN_BLOOM_WORDS sets the filter width (64 bits each; default 16 = 1024
 * bits).  Widening it lowers the false-positive rate ~linearly (FP ~= k*records
 * / (64*WORDS)).
 *
 * Both width and k are compile-time tunables that only ever trade filter cost
 * against the false-positive rate: correctness never depends on either.  The
 * 16-word / k=3 default sits at the false-positive knee and wins across every
 * workload measured for <urcu/rcu-txn.h>.
 */
#ifndef URCU_TXN_BLOOM_WORDS
# define URCU_TXN_BLOOM_WORDS	16
#endif
/*
 * URCU_TXN_BLOOM_K sets the number of hash BITS a slot maps to (default 3).
 * With k bits over m = 64*WORDS bits and n recorded slots the false-positive
 * rate is ~(1 - e^{-kn/m})^k, which for a sparse filter falls off as (kn/m)^k
 * -- so raising k cuts false positives super-linearly where widening WORDS only
 * helps linearly.  The filter is a double-hashed k-bit filter built from two
 * INDEPENDENT avalanche hashes h1,h2 (position i = h1 + i*h2); the age-0/age-1
 * study used k as the lever to drive the filter-FP escalation component toward
 * zero and isolate the genuine-RYW rate.  A degenerate k=1 is valid too (one
 * position).
 */
#ifndef URCU_TXN_BLOOM_K
# define URCU_TXN_BLOOM_K	3
#endif

#define URCU_TXN_BLOOM_BITS	(64ULL * URCU_TXN_BLOOM_WORDS)
/*
 * Two INDEPENDENT hashes of the slot.  A single multiply leaves the k derived
 * positions correlated (slot addresses are aligned and clustered).
 * Minimal-cost Kirsch-Mitzenmacher: run ONE SplitMix64 avalanche (two
 * multiplies) and split its fully-mixed 64 bits into two independent 32-bit
 * lanes -- one hash yields both h1,h2, half the cost of two separate hashes and
 * far cheaper than a multiply-free chain (Thomas Wang) whose long dependency
 * chain is slower in practice.  h2 is forced odd so the progression h1 + i*h2
 * visits k distinct bits.
 */
static inline
void urcu_txn__ryw_bloom_h1h2(void **slot, uint64_t *h1, uint64_t *h2)
{
	uint64_t x = (uint64_t) (uintptr_t) slot >> 3;	/* slots are pointer-aligned */

	x ^= x >> 30; x *= 0xbf58476d1ce4e5b9ULL;
	x ^= x >> 27; x *= 0x94d049bb133111ebULL;
	x ^= x >> 31;
	*h1 = x & 0xffffffffULL;		/* low lane */
	*h2 = (x >> 32) | 1;		/* high lane, odd stride */
}
static inline
int urcu_txn__ryw_bloom_test(const uint64_t *bloom, void **slot)
{
	uint64_t h1, h2;
	unsigned int i;

	urcu_txn__ryw_bloom_h1h2(slot, &h1, &h2);
	for (i = 0; i < URCU_TXN_BLOOM_K; i++) {
		uint64_t idx = (h1 + (uint64_t) i * h2) % URCU_TXN_BLOOM_BITS;

		if (!(bloom[idx >> 6] & ((uint64_t) 1 << (idx & 63))))
			return 0;	/* a clear bit: the slot is definitely absent */
	}
	return 1;			/* all k bits set: present (or a false positive) */
}
static inline
void urcu_txn__ryw_bloom_set(uint64_t *bloom, void **slot)
{
	uint64_t h1, h2;
	unsigned int i;

	urcu_txn__ryw_bloom_h1h2(slot, &h1, &h2);
	for (i = 0; i < URCU_TXN_BLOOM_K; i++) {
		uint64_t idx = (h1 + (uint64_t) i * h2) % URCU_TXN_BLOOM_BITS;

		bloom[idx >> 6] |= (uint64_t) 1 << (idx & 63);
	}
}
static inline
int urcu_txn__ryw_bloom_test_and_set(uint64_t *bloom, void **slot)
{
	uint64_t h1, h2;
	unsigned int i;
	int was_set = 1;

	urcu_txn__ryw_bloom_h1h2(slot, &h1, &h2);
	for (i = 0; i < URCU_TXN_BLOOM_K; i++) {
		uint64_t idx = (h1 + (uint64_t) i * h2) % URCU_TXN_BLOOM_BITS;
		unsigned int w = (unsigned int) (idx >> 6);
		uint64_t bit = (uint64_t) 1 << (idx & 63);

		if (!(bloom[w] & bit))
			was_set = 0;
		bloom[w] |= bit;
	}
	return was_set;
}

#ifdef __cplusplus
}
#endif

#endif /* _URCU_RCU_TXN_BLOOM_H */
