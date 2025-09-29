// SPDX-FileCopyrightText: 2018 Michael Jeanson <mjeanson@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

#ifndef _URCU_UTILS_H
#define _URCU_UTILS_H

/*
 * Userspace RCU library internal utils
 */

#include <urcu/config.h>
#include <unistd.h>

#define URCU_DEFAULT_PAGE_SIZE	4096

#define urcu_stringify(a) _urcu_stringify(a)
#define _urcu_stringify(a) #a

#define max_t(type, x, y)				\
	({						\
		type __max1 = (x);              	\
		type __max2 = (y);              	\
		__max1 > __max2 ? __max1: __max2;	\
	})

#define min_t(type, x, y)				\
	({						\
		type __min1 = (x);              	\
		type __min2 = (y);              	\
		__min1 <= __min2 ? __min1: __min2;	\
	})

#define __urcu_align_mask(v, mask)	(((v) + (mask)) & ~(mask))
#define urcu_align(v, align)		__urcu_align_mask(v, (__typeof__(v)) (align) - 1)

static inline
unsigned int urcu_fls_u64(uint64_t x)
{
	unsigned int r = 64;

	if (!x)
		return 0;

	if (!(x & 0xFFFFFFFF00000000ULL)) {
		x <<= 32;
		r -= 32;
	}
	if (!(x & 0xFFFF000000000000ULL)) {
		x <<= 16;
		r -= 16;
	}
	if (!(x & 0xFF00000000000000ULL)) {
		x <<= 8;
		r -= 8;
	}
	if (!(x & 0xF000000000000000ULL)) {
		x <<= 4;
		r -= 4;
	}
	if (!(x & 0xC000000000000000ULL)) {
		x <<= 2;
		r -= 2;
	}
	if (!(x & 0x8000000000000000ULL)) {
		x <<= 1;
		r -= 1;
	}
	return r;
}

static inline
unsigned int urcu_fls_u32(uint32_t x)
{
	unsigned int r = 32;

	if (!x)
		return 0;
	if (!(x & 0xFFFF0000U)) {
		x <<= 16;
		r -= 16;
	}
	if (!(x & 0xFF000000U)) {
		x <<= 8;
		r -= 8;
	}
	if (!(x & 0xF0000000U)) {
		x <<= 4;
		r -= 4;
	}
	if (!(x & 0xC0000000U)) {
		x <<= 2;
		r -= 2;
	}
	if (!(x & 0x80000000U)) {
		x <<= 1;
		r -= 1;
	}
	return r;
}

static inline
unsigned int urcu_fls_ulong(unsigned long x)
{
#if CAA_BITS_PER_LONG == 32
	return urcu_fls_u32(x);
#else
	return urcu_fls_u64(x);
#endif
}

/*
 * Return the minimum order for which x <= (1UL << order).
 * Return -1 if x is 0.
 */
static inline
int urcu_get_count_order_ulong(unsigned long x)
{
	if (!x)
		return -1;

	return urcu_fls_ulong(x - 1);
}

static inline
unsigned long urcu_get_page_len(void)
{
	long page_len = sysconf(_SC_PAGE_SIZE);

	if (page_len < 0)
		page_len = URCU_DEFAULT_PAGE_SIZE;
	return (unsigned long) page_len;
}

static inline
bool urcu_is_pow2(uint64_t x)
{
	return !(x & (x - 1));
}

#endif /* _URCU_UTILS_H */
