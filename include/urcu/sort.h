// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

#ifndef _URCU_SORT_H
#define _URCU_SORT_H

/*
 * In-place introsort, generated per element type.
 *
 * WHY NOT qsort(3).  The library sorts small arrays of fat records by one
 * integer field, and for that shape libc's qsort is the wrong tool: the
 * comparison is an indirect call it cannot inline, and elements are moved by a
 * generic byte-wise swap.  Measured on 48-byte records with a pointer key, it
 * loses to everything -- 3 to 5x slower than an inlined sort at small n, and
 * still 2 to 4x slower at n = 2048, where its asymptotics should be winning.
 * Generating the sort per type instead lets the key comparison and the element
 * move both inline, which is where the difference comes from.
 *
 * WHICH ALGORITHM.  Four were measured on the real record shape, over random
 * and reverse-sorted input (ns, random / reverse):
 *
 *      n      insertion      shell       quicksort     heapsort     qsort(3)
 *     16       51 / 117     72 /  62      58 /  52    117 / 105    263 / 276
 *    128     3121 / 6800   1257 / 908    832 / 574   1602 / 1371  3123 / 2456
 *   2048   863238/1720901  29976/26020  21300/15002  39766/35102  79620/51506
 *
 * Quicksort wins from n = 16 up and insertion sort below it; the cutoff is set
 * at 12 rather than 16 because a partition of exactly 16 reverse-sorted
 * elements costs 45 ns against insertion sort's 117 -- the crossover for
 * ADVERSARIAL input is lower than for random, and the worst case is what a
 * retry path should be sized for.  Heapsort was tried on the theory that its
 * poor locality stops mattering once the array fits in L1 -- at n = 128 it is 6
 * KiB, so it does -- but it still loses by ~2x: its cost is not misses but that
 * it does ~2n log n comparisons to quicksort's ~1.4n log n, and each sift level
 * is a dependent load-compare chain with no instruction-level parallelism,
 * whereas a partition scan is linear and well predicted.
 *
 * Heapsort earns its place as the DEPTH FALLBACK instead.  Quicksort is
 * quadratic on adversarial input, and here the sort key is an address of
 * caller-owned memory -- not something the library gets to assume is benign.
 * Switching to heapsort once the recursion depth passes 2*floor(log2(n)) caps
 * the worst case at O(n log n) while leaving the common path untouched.
 *
 * The recursion is explicit: recurse into the smaller partition and iterate on
 * the larger, so the stack is bounded by log2(n) entries and no call is made.
 *
 * USAGE.  @less is any expression over two @type values yielding true when the
 * first must sort earlier.  It is expanded inside the generated function, so it
 * must be free of side effects -- it is evaluated many times per element.
 *
 *   #define my_less(a, b)   ((uintptr_t)(a).key < (uintptr_t)(b).key)
 *   URCU_SORT_DEFINE(my_sort, struct my_rec, my_less)
 *   ...
 *   my_sort(array, n);
 */

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Partitions shorter than this are left to insertion sort, which wins below
 * roughly this size and is also what makes the final pass over an
 * almost-sorted array cheap.
 */
#ifndef URCU_SORT_CUTOFF
#define URCU_SORT_CUTOFF	12u
#endif

/*
 * Enough for the iterative quicksort's stack: it only ever pushes the SMALLER
 * side, so a frame costs at least a halving and depth cannot exceed
 * log2(SIZE_MAX).
 */
#define URCU_SORT_STACK_MAX	64

#define URCU_SORT_DEFINE(name, type, less)				\
static inline								\
void name##__ins(type *a, size_t lo, size_t hi)				\
{									\
	size_t i, j;							\
									\
	for (i = lo + 1; i <= hi; i++) {				\
		type key = a[i];					\
									\
		for (j = i; j > lo && (less(key, a[j - 1])); j--)	\
			a[j] = a[j - 1];				\
		a[j] = key;						\
	}								\
}									\
									\
/* Sift a[root] down within [lo, hi], carrying the hole in a register. */\
static inline								\
void name##__sift(type *a, size_t lo, size_t root, size_t hi)		\
{									\
	type tmp = a[root];						\
	size_t child;							\
									\
	while ((child = lo + 2u * (root - lo) + 1u) <= hi) {		\
		if (child + 1u <= hi && (less(a[child], a[child + 1u])))	\
			child++;					\
		if (!(less(tmp, a[child])))				\
			break;						\
		a[root] = a[child];					\
		root = child;						\
	}								\
	a[root] = tmp;							\
}									\
									\
/* Worst-case fallback; only reached when quicksort splits badly. */	\
static inline								\
void name##__heap(type *a, size_t lo, size_t hi)			\
{									\
	size_t n = hi - lo + 1u, i;					\
									\
	if (n < 2u)							\
		return;							\
	for (i = lo + n / 2u; i-- > lo; )				\
		name##__sift(a, lo, i, hi);				\
	for (i = hi; i > lo; i--) {					\
		type tmp = a[lo];					\
									\
		a[lo] = a[i];						\
		a[i] = tmp;						\
		name##__sift(a, lo, lo, i - 1u);			\
	}								\
}									\
									\
static inline								\
void name(type *a, size_t n)						\
{									\
	if (n < 2u)							\
		return;							\
	/*								\
	 * Small arrays take insertion sort DIRECTLY, before the		\
	 * partition machinery exists: the explicit stack is ~1.5 KiB of	\
	 * frame and the depth limit costs a log2 loop, and paying either	\
	 * for an array that never partitions showed up as ~10 ns on an	\
	 * 8-element sort.						\
	 */								\
	if (n <= URCU_SORT_CUTOFF) {					\
		name##__ins(a, 0u, n - 1u);				\
		return;							\
	}								\
	{								\
	struct { size_t lo, hi; unsigned int depth; }			\
			stack[URCU_SORT_STACK_MAX];			\
	size_t lo, hi;							\
	unsigned int depth, limit;					\
	int sp = 0;							\
									\
	/* depth limit = 2*floor(log2(n)), the introsort switch point */\
	for (limit = 0u, hi = n; hi > 1u; hi >>= 1)			\
		limit++;						\
	limit *= 2u;							\
	lo = 0u;							\
	hi = n - 1u;							\
	depth = 0u;							\
	for (;;) {							\
		if (hi - lo < URCU_SORT_CUTOFF) {			\
			name##__ins(a, lo, hi);				\
			if (sp == 0)					\
				return;					\
			sp--;						\
			lo = stack[sp].lo;				\
			hi = stack[sp].hi;				\
			depth = stack[sp].depth;			\
			continue;					\
		}							\
		if (depth >= limit) {					\
			name##__heap(a, lo, hi);			\
			if (sp == 0)					\
				return;					\
			sp--;						\
			lo = stack[sp].lo;				\
			hi = stack[sp].hi;				\
			depth = stack[sp].depth;			\
			continue;					\
		}							\
		{							\
			size_t mid = lo + ((hi - lo) >> 1);		\
			size_t i = lo, j = hi;				\
			type tmp, pivot;				\
									\
			/* median of three, so sorted input is not the	\
			 * quadratic case */				\
			if (less(a[mid], a[lo])) {			\
				tmp = a[mid]; a[mid] = a[lo]; a[lo] = tmp; \
			}						\
			if (less(a[hi], a[lo])) {			\
				tmp = a[hi]; a[hi] = a[lo]; a[lo] = tmp; \
			}						\
			if (less(a[hi], a[mid])) {			\
				tmp = a[hi]; a[hi] = a[mid]; a[mid] = tmp; \
			}						\
			pivot = a[mid];					\
			for (;;) {	/* Hoare partition */		\
				while (less(a[i], pivot))		\
					i++;				\
				while (less(pivot, a[j]))		\
					j--;				\
				if (i >= j)				\
					break;				\
				tmp = a[i]; a[i] = a[j]; a[j] = tmp;	\
				i++;					\
				j--;					\
			}						\
			depth++;					\
			/* push the SMALLER side, iterate on the larger:	\
			 * bounds the stack at log2(n) */		\
			if (j - lo < hi - j) {				\
				if (j > lo) {				\
					stack[sp].lo = lo;		\
					stack[sp].hi = j;		\
					stack[sp].depth = depth;	\
					sp++;				\
				}					\
				lo = j + 1u;				\
			} else {					\
				if (hi > j + 1u) {			\
					stack[sp].lo = j + 1u;		\
					stack[sp].hi = hi;		\
					stack[sp].depth = depth;	\
					sp++;				\
				}					\
				hi = j;					\
			}						\
		}							\
	}								\
	}								\
}

#ifdef __cplusplus
}
#endif

#endif /* _URCU_SORT_H */
