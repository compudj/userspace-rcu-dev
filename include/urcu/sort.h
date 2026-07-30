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
 * the larger, so no call is made.  The stack is bounded by the DEPTH LIMIT, not
 * by log2(n) -- see URCU_SORT_STACK_MAX.
 *
 * USAGE.  @less is any expression over two @type values yielding true when the
 * first must sort earlier.  It is expanded inside the generated function, so it
 * must be free of side effects -- it is evaluated many times per element -- and
 * it must be a strict weak ordering: an inconsistent comparison collapses the
 * Hoare scan's sentinel and walks off the array, so it is not merely a wrong
 * answer.  Names it mentions are safe from capture (see IDENTIFIER CAPTURE).
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
 * Enough for the iterative quicksort's stack.
 *
 * The bound is the DEPTH LIMIT, not log2(n).  Pushing only the smaller side
 * bounds a frame against its own parent range, but the loop then iterates on
 * the larger side, so successive frames are not each a halving -- there is no
 * log2(n) bound on @sp from that argument alone.  What does bound it is that
 * every frame on the stack carries a strictly increasing @depth in [1, limit],
 * so sp <= limit = 2*floor(log2 n).  That is tight, not loose: an organ-pipe
 * input reaches sp == limit exactly.
 *
 * 64 frames therefore covers n < 2^32.  Rather than size the array for a
 * 64-bit size_t (126 frames), @limit is clamped to this below: since sp <=
 * limit holds exactly, clamping caps the stack, and the only effect is that
 * heapsort engages a little earlier on absurdly large inputs.
 */
#define URCU_SORT_STACK_MAX	64

/*
 * IDENTIFIER CAPTURE.  Everything the generated code declares is prefixed
 * urcu_sort_, including the two parameters.  @less is expanded inside these
 * functions, so any name it mentions that the generated code also declares
 * would silently bind to the generated one instead of the caller's -- which
 * produced both wrong output and an out-of-bounds access in review when the
 * comparison referred to a variable called n or tmp.  The prefix is what keeps
 * a caller's names visible; do not un-prefix these.
 */
#define URCU_SORT_DEFINE(name, type, less)				\
static inline								\
void name##_urcu_ins(type *urcu_sort_a, size_t urcu_sort_lo,		\
		size_t urcu_sort_hi)					\
{									\
	size_t urcu_sort_i, urcu_sort_j;				\
									\
	for (urcu_sort_i = urcu_sort_lo + 1; urcu_sort_i <= urcu_sort_hi; \
			urcu_sort_i++) {				\
		type urcu_sort_key = urcu_sort_a[urcu_sort_i];		\
									\
		for (urcu_sort_j = urcu_sort_i;				\
				urcu_sort_j > urcu_sort_lo &&		\
				(less(urcu_sort_key,			\
					urcu_sort_a[urcu_sort_j - 1]));	\
				urcu_sort_j--)				\
			urcu_sort_a[urcu_sort_j] =			\
					urcu_sort_a[urcu_sort_j - 1];	\
		urcu_sort_a[urcu_sort_j] = urcu_sort_key;		\
	}								\
}									\
									\
/* Sift down within [lo, hi], carrying the hole in a register. */	\
static inline								\
void name##_urcu_sift(type *urcu_sort_a, size_t urcu_sort_lo,		\
		size_t urcu_sort_root, size_t urcu_sort_hi)		\
{									\
	type urcu_sort_tmp = urcu_sort_a[urcu_sort_root];		\
	size_t urcu_sort_child;						\
									\
	while ((urcu_sort_child = urcu_sort_lo +			\
			2u * (urcu_sort_root - urcu_sort_lo) + 1u)	\
			<= urcu_sort_hi) {				\
		if (urcu_sort_child + 1u <= urcu_sort_hi &&		\
				(less(urcu_sort_a[urcu_sort_child],	\
				urcu_sort_a[urcu_sort_child + 1u])))	\
			urcu_sort_child++;				\
		if (!(less(urcu_sort_tmp, urcu_sort_a[urcu_sort_child])))	\
			break;						\
		urcu_sort_a[urcu_sort_root] = urcu_sort_a[urcu_sort_child];	\
		urcu_sort_root = urcu_sort_child;			\
	}								\
	urcu_sort_a[urcu_sort_root] = urcu_sort_tmp;			\
}									\
									\
/*									\
 * Worst-case fallback; only reached when quicksort splits badly.	\
 * Callers must pass lo <= hi: unlike the insertion sort, which is a	\
 * no-op on an inverted range, this would underflow the count below.	\
 */									\
static inline								\
void name##_urcu_heap(type *urcu_sort_a, size_t urcu_sort_lo,		\
		size_t urcu_sort_hi)					\
{									\
	size_t urcu_sort_n = urcu_sort_hi - urcu_sort_lo + 1u;		\
	size_t urcu_sort_i;						\
									\
	if (urcu_sort_n < 2u)						\
		return;							\
	for (urcu_sort_i = urcu_sort_lo + urcu_sort_n / 2u;		\
			urcu_sort_i-- > urcu_sort_lo; )			\
		name##_urcu_sift(urcu_sort_a, urcu_sort_lo,		\
				urcu_sort_i, urcu_sort_hi);		\
	for (urcu_sort_i = urcu_sort_hi; urcu_sort_i > urcu_sort_lo;	\
			urcu_sort_i--) {				\
		type urcu_sort_tmp = urcu_sort_a[urcu_sort_lo];		\
									\
		urcu_sort_a[urcu_sort_lo] = urcu_sort_a[urcu_sort_i];	\
		urcu_sort_a[urcu_sort_i] = urcu_sort_tmp;		\
		name##_urcu_sift(urcu_sort_a, urcu_sort_lo,		\
				urcu_sort_lo, urcu_sort_i - 1u);	\
	}								\
}									\
									\
static inline								\
void name(type *urcu_sort_a, size_t urcu_sort_n)			\
{									\
	if (urcu_sort_n < 2u)						\
		return;							\
	/*								\
	 * Small arrays take insertion sort directly, skipping the depth	\
	 * limit's log2 loop.  Note the stack array below still costs its	\
	 * frame -- gcc allocates it on entry, before this test -- so this	\
	 * saves the computation, not the frame.			\
	 */								\
	if (urcu_sort_n <= URCU_SORT_CUTOFF) {				\
		name##_urcu_ins(urcu_sort_a, 0u, urcu_sort_n - 1u);	\
		return;							\
	}								\
	{								\
	struct { size_t lo, hi; unsigned int depth; }			\
			urcu_sort_stack[URCU_SORT_STACK_MAX];		\
	size_t urcu_sort_lo, urcu_sort_hi;				\
	unsigned int urcu_sort_depth, urcu_sort_limit;			\
	int urcu_sort_sp = 0;						\
									\
	/* depth limit = 2*floor(log2 n), the introsort switch point */	\
	for (urcu_sort_limit = 0u, urcu_sort_hi = urcu_sort_n;		\
			urcu_sort_hi > 1u; urcu_sort_hi >>= 1)		\
		urcu_sort_limit++;					\
	urcu_sort_limit *= 2u;						\
	/* sp <= limit exactly, so this is what bounds the stack */	\
	if (urcu_sort_limit > (unsigned int) URCU_SORT_STACK_MAX)	\
		urcu_sort_limit = (unsigned int) URCU_SORT_STACK_MAX;	\
	urcu_sort_lo = 0u;						\
	urcu_sort_hi = urcu_sort_n - 1u;				\
	urcu_sort_depth = 0u;						\
	for (;;) {							\
		if (urcu_sort_hi - urcu_sort_lo < URCU_SORT_CUTOFF) {	\
			name##_urcu_ins(urcu_sort_a, urcu_sort_lo,	\
					urcu_sort_hi);			\
			if (urcu_sort_sp == 0)				\
				return;					\
			urcu_sort_sp--;					\
			urcu_sort_lo = urcu_sort_stack[urcu_sort_sp].lo;	\
			urcu_sort_hi = urcu_sort_stack[urcu_sort_sp].hi;	\
			urcu_sort_depth =				\
				urcu_sort_stack[urcu_sort_sp].depth;	\
			continue;					\
		}							\
		if (urcu_sort_depth >= urcu_sort_limit) {		\
			name##_urcu_heap(urcu_sort_a, urcu_sort_lo,	\
					urcu_sort_hi);			\
			if (urcu_sort_sp == 0)				\
				return;					\
			urcu_sort_sp--;					\
			urcu_sort_lo = urcu_sort_stack[urcu_sort_sp].lo;	\
			urcu_sort_hi = urcu_sort_stack[urcu_sort_sp].hi;	\
			urcu_sort_depth =				\
				urcu_sort_stack[urcu_sort_sp].depth;	\
			continue;					\
		}							\
		{							\
			size_t urcu_sort_mid = urcu_sort_lo +		\
				((urcu_sort_hi - urcu_sort_lo) >> 1);	\
			size_t urcu_sort_i = urcu_sort_lo;		\
			size_t urcu_sort_j = urcu_sort_hi;		\
			type urcu_sort_tmp, urcu_sort_pivot;		\
									\
			/* median of three: sorted input is not the	\
			 * quadratic case */				\
			if (less(urcu_sort_a[urcu_sort_mid],		\
					urcu_sort_a[urcu_sort_lo])) {	\
				urcu_sort_tmp = urcu_sort_a[urcu_sort_mid];	\
				urcu_sort_a[urcu_sort_mid] =		\
					urcu_sort_a[urcu_sort_lo];	\
				urcu_sort_a[urcu_sort_lo] = urcu_sort_tmp;	\
			}						\
			if (less(urcu_sort_a[urcu_sort_hi],		\
					urcu_sort_a[urcu_sort_lo])) {	\
				urcu_sort_tmp = urcu_sort_a[urcu_sort_hi];	\
				urcu_sort_a[urcu_sort_hi] =		\
					urcu_sort_a[urcu_sort_lo];	\
				urcu_sort_a[urcu_sort_lo] = urcu_sort_tmp;	\
			}						\
			if (less(urcu_sort_a[urcu_sort_hi],		\
					urcu_sort_a[urcu_sort_mid])) {	\
				urcu_sort_tmp = urcu_sort_a[urcu_sort_hi];	\
				urcu_sort_a[urcu_sort_hi] =		\
					urcu_sort_a[urcu_sort_mid];	\
				urcu_sort_a[urcu_sort_mid] = urcu_sort_tmp;	\
			}						\
			urcu_sort_pivot = urcu_sort_a[urcu_sort_mid];	\
			for (;;) {	/* Hoare partition */		\
				while (less(urcu_sort_a[urcu_sort_i],	\
						urcu_sort_pivot))	\
					urcu_sort_i++;			\
				while (less(urcu_sort_pivot,		\
						urcu_sort_a[urcu_sort_j]))	\
					urcu_sort_j--;			\
				if (urcu_sort_i >= urcu_sort_j)		\
					break;				\
				urcu_sort_tmp = urcu_sort_a[urcu_sort_i];	\
				urcu_sort_a[urcu_sort_i] =		\
					urcu_sort_a[urcu_sort_j];	\
				urcu_sort_a[urcu_sort_j] = urcu_sort_tmp;	\
				urcu_sort_i++;				\
				urcu_sort_j--;				\
			}						\
			urcu_sort_depth++;				\
			/* push the SMALLER side, iterate on the larger */	\
			if (urcu_sort_j - urcu_sort_lo <		\
					urcu_sort_hi - urcu_sort_j) {	\
				if (urcu_sort_j > urcu_sort_lo) {	\
					urcu_sort_stack[urcu_sort_sp].lo =	\
						urcu_sort_lo;		\
					urcu_sort_stack[urcu_sort_sp].hi =	\
						urcu_sort_j;		\
					urcu_sort_stack[urcu_sort_sp].depth =	\
						urcu_sort_depth;	\
					urcu_sort_sp++;			\
				}					\
				urcu_sort_lo = urcu_sort_j + 1u;	\
			} else {					\
				if (urcu_sort_hi > urcu_sort_j + 1u) {	\
					urcu_sort_stack[urcu_sort_sp].lo =	\
						urcu_sort_j + 1u;	\
					urcu_sort_stack[urcu_sort_sp].hi =	\
						urcu_sort_hi;		\
					urcu_sort_stack[urcu_sort_sp].depth =	\
						urcu_sort_depth;	\
					urcu_sort_sp++;			\
				}					\
				urcu_sort_hi = urcu_sort_j;		\
			}						\
		}							\
	}								\
	}								\
}

#ifdef __cplusplus
}
#endif

#endif /* _URCU_SORT_H */
