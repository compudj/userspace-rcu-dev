// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-only

#ifndef _CDS_BITMAP_H
#define _CDS_BITMAP_H

#include <urcu/compiler.h>
#include <stdbool.h>

static inline
void cds_set_bit_relaxed(unsigned long *bitmap, unsigned int bit)
{
	unsigned int long_idx = bit / CAA_BITS_PER_LONG,
		bit_idx = bit % CAA_BITS_PER_LONG;
	bitmap[long_idx] |= (1UL << bit_idx);
}

static inline
void cds_clear_bit_relaxed(unsigned long *bitmap, unsigned int bit)
{
	unsigned int long_idx = bit / CAA_BITS_PER_LONG,
		bit_idx = bit % CAA_BITS_PER_LONG;
	bitmap[long_idx] &= ~(1UL << bit_idx);
}

static inline
bool cds_test_bit(unsigned long *bitmap, unsigned int bit)
{
	unsigned int long_idx = bit / CAA_BITS_PER_LONG,
		bit_idx = bit % CAA_BITS_PER_LONG;
	return !!(bitmap[long_idx] & (1UL << bit_idx));
}

/* Find next set bit where bit >= start_bit. */
static inline
int cds_find_next_bit(unsigned long *bitmap, unsigned int nr_bits, int start_bit)
{
	unsigned int long_idx = start_bit / CAA_BITS_PER_LONG;
	unsigned int bit_idx = start_bit % CAA_BITS_PER_LONG;
	unsigned int nr_longs = (nr_bits + CAA_BITS_PER_LONG - 1) / CAA_BITS_PER_LONG;
	unsigned long val;

	/* Handle out of range start_bit: start_bit < 0 and start_bit >= nr_bits. */
	if ((unsigned int) start_bit >= nr_bits)
		return -1;

	/* Handle the first long (mask out bits before start_bit). */
	val = bitmap[long_idx] & (~0UL << bit_idx);

	if (val) {
		int res = (long_idx * CAA_BITS_PER_LONG) + __builtin_ctzl(val);
		return (res < (int) nr_bits) ? res : -1;
	}

	/* Scan subsequent longs. */
	for (long_idx++; long_idx < nr_longs; long_idx++) {
		val = bitmap[long_idx];
		if (val) {
			int res = (long_idx * CAA_BITS_PER_LONG) + __builtin_ctzl(val);
			return (res < (int) nr_bits) ? res : -1;
		}
	}

	return -1;
}

/* Find previous set bit where bit <= start_bit. */
static inline
int cds_find_prev_bit(unsigned long *bitmap, unsigned int nr_bits, int start_bit)
{
	unsigned long mask, val;
	unsigned int bit_idx;
	int long_idx;

	/* Handle out of range start_bit: start_bit < 0 and start_bit >= nr_bits. */
	if ((unsigned int) start_bit >= nr_bits)
		return -1;
	long_idx = start_bit / CAA_BITS_PER_LONG;
	bit_idx = start_bit % CAA_BITS_PER_LONG;

	/*
	 * Handle the first long (mask out bits after start_bit).
	 * Shift right to clear bits higher than bit_idx, then shift
	 * back.
	 */
	mask = ~0UL >> (CAA_BITS_PER_LONG - 1 - bit_idx);
	val = bitmap[long_idx] & mask;

	if (val) {
		/* clz returns leading zeros. Bit index = (total_bits - 1) - leading_zeros. */
		return (long_idx * CAA_BITS_PER_LONG) + (CAA_BITS_PER_LONG - 1 - __builtin_clzl(val));
	}

	/* Scan previous longs. */
	for (long_idx--; long_idx >= 0; long_idx--) {
		val = bitmap[long_idx];
		if (val) {
			return (long_idx * CAA_BITS_PER_LONG) + (CAA_BITS_PER_LONG - 1 - __builtin_clzl(val));
		}
	}

	return -1;
}

/* Find first set bit. */
static inline
int cds_find_first_bit(unsigned long *bitmap, unsigned int nr_bits)
{
	return cds_find_next_bit(bitmap, nr_bits, 0);
}

/* Find last set bit. */
static inline
int cds_find_last_bit(unsigned long *bitmap, unsigned int nr_bits)
{
	return cds_find_prev_bit(bitmap, nr_bits, nr_bits - 1);
}

#endif /* _CDS_BITMAP_H */
