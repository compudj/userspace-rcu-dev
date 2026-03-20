/*
 * fractal_trie-testpop.c
 *
 * Userspace RCU library - RCU Judy Array population size test
 *
 * Copyright 2012-2025 - Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

/*
 * This program generates random populations, and shows the largest
 * sub-class generated, as well as the distribution of sub-class size
 * for the largest sub-class of each population.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <time.h>
#include <string.h>
#include <limits.h>
#include <assert.h>
#include <inttypes.h>
#include <urcu/compiler.h>

#define DEFAULT_NR_ITEMS	50
#define DEFAULT_NR_CLASSES	2
#define DEFAULT_NR_GREEDY	0
#define DEFAULT_NR_POP	10000000ULL

static int nr_items = DEFAULT_NR_ITEMS;
static int opt_nr_classes = DEFAULT_NR_CLASSES;
static int opt_nr_greedy = DEFAULT_NR_GREEDY;
static uint64_t nr_pop = DEFAULT_NR_POP;
static int opt_bit_breakdown = 0;

static uint8_t pop[256];

static uint8_t nr_one[8];

static uint8_t nr_2d_11[8][8];
static uint8_t nr_2d_10[8][8];
static uint8_t nr_2d_01[8][8];
static uint8_t nr_2d_00[8][8];

static uint8_t nr_3d_111[8][8][8];
static uint8_t nr_3d_110[8][8][8];
static uint8_t nr_3d_101[8][8][8];
static uint8_t nr_3d_100[8][8][8];
static uint8_t nr_3d_011[8][8][8];
static uint8_t nr_3d_010[8][8][8];
static uint8_t nr_3d_001[8][8][8];
static uint8_t nr_3d_000[8][8][8];

static uint8_t nr_4d_1111[8][8][8][8];
static uint8_t nr_4d_1110[8][8][8][8];
static uint8_t nr_4d_1101[8][8][8][8];
static uint8_t nr_4d_1100[8][8][8][8];
static uint8_t nr_4d_1011[8][8][8][8];
static uint8_t nr_4d_1010[8][8][8][8];
static uint8_t nr_4d_1001[8][8][8][8];
static uint8_t nr_4d_1000[8][8][8][8];
static uint8_t nr_4d_0111[8][8][8][8];
static uint8_t nr_4d_0110[8][8][8][8];
static uint8_t nr_4d_0101[8][8][8][8];
static uint8_t nr_4d_0100[8][8][8][8];
static uint8_t nr_4d_0011[8][8][8][8];
static uint8_t nr_4d_0010[8][8][8][8];
static uint8_t nr_4d_0001[8][8][8][8];
static uint8_t nr_4d_0000[8][8][8][8];

struct bit_distance {
	unsigned int bit;
	unsigned int distance;
};

/*
 * global_min_distance_max_class_items keeps track of the largest
 * observed min_distance_max_class_items across all generated
 * populations.
 */
static unsigned int global_min_distance_max_class_items = 0;

/*
 * histogram_min_distance_max_class_items is an histogram counting the number of
 * populations per min_distance_max_class_items across all generated
 * populations.
 */
static unsigned int histogram_min_distance_max_class_items[256];

static int verbose;

#define BITMASK_2(a, b)		(1U << (a) | 1U << (b))
#define BITMASK_3(a, b, c)	(1U << (a) | 1U << (b) | 1U << (c))
#define BITMASK_4(a, b, c, d)	(1U << (a) | 1U << (b) | 1U << (c) | 1U << (d))

static
const uint8_t combinations_2_in_8[] = {
	BITMASK_2(0, 1), BITMASK_2(0, 2), BITMASK_2(0, 3), BITMASK_2(0, 4), BITMASK_2(0, 5), BITMASK_2(0, 6), BITMASK_2(0, 7),
	BITMASK_2(1, 2), BITMASK_2(1, 3), BITMASK_2(1, 4), BITMASK_2(1, 5), BITMASK_2(1, 6), BITMASK_2(1, 7),
	BITMASK_2(2, 3), BITMASK_2(2, 4), BITMASK_2(2, 5), BITMASK_2(2, 6), BITMASK_2(2, 7),
	BITMASK_2(3, 4), BITMASK_2(3, 5), BITMASK_2(3, 6), BITMASK_2(3, 7),
	BITMASK_2(4, 5), BITMASK_2(4, 6), BITMASK_2(4, 7),
	BITMASK_2(5, 6), BITMASK_2(5, 7),
	BITMASK_2(6, 7)
};

static
const uint8_t combinations_3_in_8[] = {
	BITMASK_3(0, 1, 2), BITMASK_3(0, 1, 3), BITMASK_3(0, 1, 4), BITMASK_3(0, 1, 5), BITMASK_3(0, 1, 6), BITMASK_3(0, 1, 7),
	BITMASK_3(0, 2, 3), BITMASK_3(0, 2, 4), BITMASK_3(0, 2, 5), BITMASK_3(0, 2, 6), BITMASK_3(0, 2, 7),
	BITMASK_3(0, 3, 4), BITMASK_3(0, 3, 5), BITMASK_3(0, 3, 6), BITMASK_3(0, 3, 7),
	BITMASK_3(0, 4, 5), BITMASK_3(0, 4, 6), BITMASK_3(0, 4, 7),
	BITMASK_3(0, 5, 6), BITMASK_3(0, 5, 7),
	BITMASK_3(0, 6, 7),
	BITMASK_3(1, 2, 3), BITMASK_3(1, 2, 4), BITMASK_3(1, 2, 5), BITMASK_3(1, 2, 6), BITMASK_3(1, 2, 7),
	BITMASK_3(1, 3, 4), BITMASK_3(1, 3, 5), BITMASK_3(1, 3, 6), BITMASK_3(1, 3, 7),
	BITMASK_3(1, 4, 5), BITMASK_3(1, 4, 6), BITMASK_3(1, 4, 7),
	BITMASK_3(1, 5, 6), BITMASK_3(1, 5, 7),
	BITMASK_3(1, 6, 7),
	BITMASK_3(2, 3, 4), BITMASK_3(2, 3, 5), BITMASK_3(2, 3, 6), BITMASK_3(2, 3, 7),
	BITMASK_3(2, 4, 5), BITMASK_3(2, 4, 6), BITMASK_3(2, 4, 7),
	BITMASK_3(2, 5, 6), BITMASK_3(2, 5, 7),
	BITMASK_3(2, 6, 7),
	BITMASK_3(3, 4, 5), BITMASK_3(3, 4, 6), BITMASK_3(3, 4, 7),
	BITMASK_3(3, 5, 6), BITMASK_3(3, 5, 7),
	BITMASK_3(3, 6, 7),
	BITMASK_3(4, 5, 6), BITMASK_3(4, 5, 7),
	BITMASK_3(4, 6, 7),
	BITMASK_3(5, 6, 7)
};

static
const uint8_t combinations_4_in_8[] = {
	BITMASK_4(0, 1, 2, 3), BITMASK_4(0, 2, 3, 4), BITMASK_4(0, 3, 4, 5), BITMASK_4(0, 4, 5, 6), BITMASK_4(0, 5, 6, 7),
	BITMASK_4(0, 1, 2, 4), BITMASK_4(0, 2, 3, 5), BITMASK_4(0, 3, 4, 6), BITMASK_4(0, 4, 5, 7),
	BITMASK_4(0, 1, 2, 5), BITMASK_4(0, 2, 3, 6), BITMASK_4(0, 3, 4, 7), BITMASK_4(0, 4, 6, 7),
	BITMASK_4(0, 1, 2, 6), BITMASK_4(0, 2, 3, 7), BITMASK_4(0, 3, 5, 6),
	BITMASK_4(0, 1, 2, 7), BITMASK_4(0, 2, 4, 5), BITMASK_4(0, 3, 5, 7),
	BITMASK_4(0, 1, 3, 4), BITMASK_4(0, 2, 4, 6), BITMASK_4(0, 3, 6, 7),
	BITMASK_4(0, 1, 3, 5), BITMASK_4(0, 2, 4, 7),
	BITMASK_4(0, 1, 3, 6), BITMASK_4(0, 2, 5, 6),
	BITMASK_4(0, 1, 3, 7), BITMASK_4(0, 2, 5, 7),
	BITMASK_4(0, 1, 4, 5), BITMASK_4(0, 2, 6, 7),
	BITMASK_4(0, 1, 4, 6),
	BITMASK_4(0, 1, 4, 7),
	BITMASK_4(0, 1, 5, 6),
	BITMASK_4(0, 1, 5, 7),
	BITMASK_4(0, 1, 6, 7),
	BITMASK_4(1, 2, 3, 4), BITMASK_4(2, 3, 4, 5), BITMASK_4(3, 4, 5, 6),
	BITMASK_4(1, 2, 3, 5), BITMASK_4(2, 3, 4, 6), BITMASK_4(3, 4, 5, 7),
	BITMASK_4(1, 2, 3, 6), BITMASK_4(2, 3, 4, 7), BITMASK_4(3, 4, 6, 7),
	BITMASK_4(1, 2, 3, 7), BITMASK_4(2, 3, 5, 6), BITMASK_4(3, 5, 6, 7),
	BITMASK_4(1, 2, 4, 5), BITMASK_4(2, 3, 5, 7), BITMASK_4(4, 5, 6, 7),
	BITMASK_4(1, 2, 4, 6), BITMASK_4(2, 3, 6, 7),
	BITMASK_4(1, 2, 4, 7), BITMASK_4(2, 4, 5, 6),
	BITMASK_4(1, 2, 5, 6), BITMASK_4(2, 4, 5, 7),
	BITMASK_4(1, 2, 5, 7), BITMASK_4(2, 4, 6, 7),
	BITMASK_4(1, 2, 6, 7), BITMASK_4(2, 5, 6, 7),
	BITMASK_4(1, 3, 4, 5),
	BITMASK_4(1, 3, 4, 6),
	BITMASK_4(1, 3, 4, 7),
	BITMASK_4(1, 3, 5, 6),
	BITMASK_4(1, 3, 5, 7),
	BITMASK_4(1, 3, 6, 7),
	BITMASK_4(1, 4, 5, 6),
	BITMASK_4(1, 4, 5, 7),
	BITMASK_4(1, 4, 6, 7),
	BITMASK_4(1, 5, 6, 7)
};

struct combinations {
	size_t len;
	const uint8_t *array;
};

static
const struct combinations combinations_n_in_8[] = {
	[2] = {
		.len = CAA_ARRAY_SIZE(combinations_2_in_8),
		.array = combinations_2_in_8,
	},
	[3] = {
		.len = CAA_ARRAY_SIZE(combinations_3_in_8),
		.array = combinations_3_in_8,
	},
	[4] = {
		.len = CAA_ARRAY_SIZE(combinations_4_in_8),
		.array = combinations_4_in_8,
	},
};

static
uint64_t diff_time(struct timespec *time1, struct timespec *time2)
{
	uint64_t result = time2->tv_nsec - time1->tv_nsec;

	return (time2->tv_sec - time1->tv_sec) * 1000000000ULL + result;
}

/* return an index within the combination table C(n=8,r) associated to mask. */
static inline
unsigned int bitmask_to_index(unsigned int r, uint8_t mask)
{
	const struct combinations *combinations;
	unsigned int i;

	assert(__builtin_popcount(mask) == r);
	if (r == 1)
		return __builtin_ctz(mask);
	assert(r >= 2 && r <= 4);
	combinations = &combinations_n_in_8[r];
	for (i = 0; i < combinations->len; i++)
		if (combinations->array[i] == mask)
			return i;
	abort();
}

static inline
uint8_t index_to_bitmask(unsigned int r, unsigned int index)
{
	const struct combinations *combinations;

	if (r == 1)
		return 1U << index;
	assert(r >= 2 && r <= 4);
	combinations = &combinations_n_in_8[r];
	return combinations->array[index];
}

/*
 * Keep only the "mask" bits from value, and move them to LSB to form a
 * subclass index.
 */
static inline
unsigned int value_and_mask_to_subclass_index(unsigned int value, unsigned int mask)
{
	unsigned int subclass_index = 0;
	int bit, bit_out = 0;

	for (bit = 0; bit < 8; bit++) {
		if (mask & (1U << bit)) {
			if (value & (1U << bit))
				subclass_index |= (1U << bit_out);
			bit_out++;
		}
	}
	return subclass_index;
}

static
uint8_t random_char(void)
{
	return (uint8_t) random();
}

static
void print_pop(void)
{
	int i;

	printf("Population: ");
	for (i = 0; i < nr_items; i++)
		printf("%d ", (int) pop[i]);
	printf("\n");
}

static
void gen_pop(void)
{
	uint8_t src_pop[256];
	int i, nr_left = 256;

	memset(pop, 0, sizeof(pop));
	for (i = 0; i < 256; i++)
		src_pop[i] = (uint8_t) i;
	for (i = 0; i < nr_items; i++) {
		int sel;

		sel = random_char() % nr_left;
		pop[i] = src_pop[sel];
		src_pop[sel] = src_pop[nr_left - 1];
		nr_left--;
	}
}

static
void count_pop(int nr_classes)
{
	int bit, bit_i, bit_j, bit_k, bit_l;
	int i;

	memset(nr_one, 0, sizeof(nr_one));

	switch (nr_classes) {
	case 4:
		memset(nr_2d_11, 0, sizeof(nr_2d_11));
		memset(nr_2d_10, 0, sizeof(nr_2d_10));
		memset(nr_2d_01, 0, sizeof(nr_2d_01));
		memset(nr_2d_00, 0, sizeof(nr_2d_00));
		break;
	case 8:
		memset(nr_3d_111, 0, sizeof(nr_3d_111));
		memset(nr_3d_110, 0, sizeof(nr_3d_110));
		memset(nr_3d_101, 0, sizeof(nr_3d_101));
		memset(nr_3d_100, 0, sizeof(nr_3d_100));
		memset(nr_3d_011, 0, sizeof(nr_3d_011));
		memset(nr_3d_010, 0, sizeof(nr_3d_010));
		memset(nr_3d_001, 0, sizeof(nr_3d_001));
		memset(nr_3d_000, 0, sizeof(nr_3d_000));
		break;
	case 16:
		memset(nr_4d_1111, 0, sizeof(nr_4d_1111));
		memset(nr_4d_1110, 0, sizeof(nr_4d_1110));
		memset(nr_4d_1101, 0, sizeof(nr_4d_1101));
		memset(nr_4d_1100, 0, sizeof(nr_4d_1100));
		memset(nr_4d_1011, 0, sizeof(nr_4d_1011));
		memset(nr_4d_1010, 0, sizeof(nr_4d_1010));
		memset(nr_4d_1001, 0, sizeof(nr_4d_1001));
		memset(nr_4d_1000, 0, sizeof(nr_4d_1000));
		memset(nr_4d_0111, 0, sizeof(nr_4d_0111));
		memset(nr_4d_0110, 0, sizeof(nr_4d_0110));
		memset(nr_4d_0101, 0, sizeof(nr_4d_0101));
		memset(nr_4d_0100, 0, sizeof(nr_4d_0100));
		memset(nr_4d_0011, 0, sizeof(nr_4d_0011));
		memset(nr_4d_0010, 0, sizeof(nr_4d_0010));
		memset(nr_4d_0001, 0, sizeof(nr_4d_0001));
		memset(nr_4d_0000, 0, sizeof(nr_4d_0000));
		break;
	}

	for (i = 0; i < nr_items; i++) {
		/* 1 selection bit */
		for (bit = 0; bit < 8; bit++) {
			if (pop[i] & (1U << bit))
				nr_one[bit]++;
		}

		switch (nr_classes) {
		case 4:
			/* 2 selection bits */
			for (bit_i = 0; bit_i < 8; bit_i++) {
				for (bit_j = bit_i + 1; bit_j < 8; bit_j++) {
					if (pop[i] & (1U << bit_i)) {
						if (pop[i] & (1U << bit_j)) {
							nr_2d_11[bit_i][bit_j]++;
						} else {
							nr_2d_10[bit_i][bit_j]++;
						}
					} else {
						if (pop[i] & (1U << bit_j)) {
							nr_2d_01[bit_i][bit_j]++;
						} else {
							nr_2d_00[bit_i][bit_j]++;
						}
					}
				}
			}
			break;
		case 8:
			/* 3 selection bits */
			for (bit_i = 0; bit_i < 8; bit_i++) {
				for (bit_j = bit_i + 1; bit_j < 8; bit_j++) {
					for (bit_k = bit_j + 1; bit_k < 8; bit_k++) {
						if (pop[i] & (1U << bit_i)) {
							if (pop[i] & (1U << bit_j)) {
								if (pop[i] & (1U << bit_k)) {
									nr_3d_111[bit_i][bit_j][bit_k]++;
								} else {
									nr_3d_110[bit_i][bit_j][bit_k]++;
								}
							} else {
								if (pop[i] & (1U << bit_k)) {
									nr_3d_101[bit_i][bit_j][bit_k]++;
								} else {
									nr_3d_100[bit_i][bit_j][bit_k]++;
								}
							}
						} else {
							if (pop[i] & (1U << bit_j)) {
								if (pop[i] & (1U << bit_k)) {
									nr_3d_011[bit_i][bit_j][bit_k]++;
								} else {
									nr_3d_010[bit_i][bit_j][bit_k]++;
								}
							} else {
								if (pop[i] & (1U << bit_k)) {
									nr_3d_001[bit_i][bit_j][bit_k]++;
								} else {
									nr_3d_000[bit_i][bit_j][bit_k]++;
								}
							}
						}
					}
				}
			}
			break;

		case 16:
			/* 4 selection bits */
			for (bit_i = 0; bit_i < 8; bit_i++) {
				for (bit_j = bit_i + 1; bit_j < 8; bit_j++) {
					for (bit_k = bit_j + 1; bit_k < 8; bit_k++) {
						for (bit_l = bit_k + 1; bit_l < 8; bit_l++) {
							if (pop[i] & (1U << bit_i)) {
								if (pop[i] & (1U << bit_j)) {
									if (pop[i] & (1U << bit_k)) {
										if (pop[i] & (1U << bit_l)) {
											nr_4d_1111[bit_i][bit_j][bit_k][bit_l]++;
										} else {
											nr_4d_1110[bit_i][bit_j][bit_k][bit_l]++;
										}
									} else {
										if (pop[i] & (1U << bit_l)) {
											nr_4d_1101[bit_i][bit_j][bit_k][bit_l]++;
										} else {
											nr_4d_1100[bit_i][bit_j][bit_k][bit_l]++;
										}
									}
								} else {
									if (pop[i] & (1U << bit_k)) {
										if (pop[i] & (1U << bit_l)) {
											nr_4d_1011[bit_i][bit_j][bit_k][bit_l]++;
										} else {
											nr_4d_1010[bit_i][bit_j][bit_k][bit_l]++;
										}
									} else {
										if (pop[i] & (1U << bit_l)) {
											nr_4d_1001[bit_i][bit_j][bit_k][bit_l]++;
										} else {
											nr_4d_1000[bit_i][bit_j][bit_k][bit_l]++;
										}
									}
								}
							} else {
								if (pop[i] & (1U << bit_j)) {
									if (pop[i] & (1U << bit_k)) {
										if (pop[i] & (1U << bit_l)) {
											nr_4d_0111[bit_i][bit_j][bit_k][bit_l]++;
										} else {
											nr_4d_0110[bit_i][bit_j][bit_k][bit_l]++;
										}
									} else {
										if (pop[i] & (1U << bit_l)) {
											nr_4d_0101[bit_i][bit_j][bit_k][bit_l]++;
										} else {
											nr_4d_0100[bit_i][bit_j][bit_k][bit_l]++;
										}
									}
								} else {
									if (pop[i] & (1U << bit_k)) {
										if (pop[i] & (1U << bit_l)) {
											nr_4d_0011[bit_i][bit_j][bit_k][bit_l]++;
										} else {
											nr_4d_0010[bit_i][bit_j][bit_k][bit_l]++;
										}
									} else {
										if (pop[i] & (1U << bit_l)) {
											nr_4d_0001[bit_i][bit_j][bit_k][bit_l]++;
										} else {
											nr_4d_0000[bit_i][bit_j][bit_k][bit_l]++;
										}
									}
								}
							}
						}
					}
				}
			}
			break;
		}
	}
}

static
void print_count(int nr_classes)
{
	printf("Distribution:\n");

	switch (nr_classes) {
	case 2:
	{
		int bit_i;

		printf("       count per class\n");
		printf("bit  |    0      1\n");
		printf("------------------------\n");
		for (bit_i = 0; bit_i < 8; bit_i++) {
			printf("%1u    |  %3d    %3d\n",
				bit_i, nr_items - nr_one[bit_i], nr_one[bit_i]);
		}
		break;
	}
	case 4:
	{
		int bit_i, bit_j;

		printf("                count per class\n");
		printf("bits   |    00     01     10     11\n");
		printf("-------------------------------------\n");
		for (bit_i = 0; bit_i < 8; bit_i++) {
			for (bit_j = bit_i + 1; bit_j < 8; bit_j++) {
				printf("(%1u, %1u) |   %3d    %3d    %3d    %3d\n",
					bit_i, bit_j,
					nr_2d_00[bit_i][bit_j],
					nr_2d_01[bit_i][bit_j],
					nr_2d_10[bit_i][bit_j],
					nr_2d_11[bit_i][bit_j]);
			}
		}
		break;
	}
	case 8:
	{
		int bit_i, bit_j, bit_k;

		printf("                            count per class\n");
		printf("bits      |    000   001   010   011   100   101   110   111\n");
		printf("--------------------------------------------------------------\n");
		for (bit_i = 0; bit_i < 8; bit_i++) {
			for (bit_j = bit_i + 1; bit_j < 8; bit_j++) {
				for (bit_k = bit_j + 1; bit_k < 8; bit_k++) {
					printf("(%1u, %1u, %1u) |    %3d   %3d   %3d   %3d   %3d   %3d   %3d   %3d\n",
						bit_i, bit_j, bit_k,
						nr_3d_000[bit_i][bit_j][bit_k],
						nr_3d_001[bit_i][bit_j][bit_k],
						nr_3d_010[bit_i][bit_j][bit_k],
						nr_3d_011[bit_i][bit_j][bit_k],
						nr_3d_100[bit_i][bit_j][bit_k],
						nr_3d_101[bit_i][bit_j][bit_k],
						nr_3d_110[bit_i][bit_j][bit_k],
						nr_3d_111[bit_i][bit_j][bit_k]);
				}
			}
		}
		break;
	}
	case 16:
	{
		int bit_i, bit_j, bit_k, bit_l;

		printf("                                        count per class\n");
		printf("bits        |    0000  0001  0010  0011  0100  0101  0110  0111  1000  1001  1010  1011  1100  1101  1110  1111\n");
		printf("-----------------------------------------------------------------------------------------------------------------------------------------------------\n");
		for (bit_i = 0; bit_i < 8; bit_i++) {
			for (bit_j = bit_i + 1; bit_j < 8; bit_j++) {
				for (bit_k = bit_j + 1; bit_k < 8; bit_k++) {
					for (bit_l = bit_k + 1; bit_l < 8; bit_l++) {
						printf("(%1u, %1u, %1u, %1u) |   %4d  %4d  %4d  %4d  %4d  %4d  %4d  %4d  %4d  %4d  %4d  %4d  %4d  %4d  %4d  %4d\n",
							bit_i, bit_j, bit_k, bit_l,
							nr_4d_0000[bit_i][bit_j][bit_k][bit_l],
							nr_4d_0001[bit_i][bit_j][bit_k][bit_l],
							nr_4d_0010[bit_i][bit_j][bit_k][bit_l],
							nr_4d_0011[bit_i][bit_j][bit_k][bit_l],
							nr_4d_0100[bit_i][bit_j][bit_k][bit_l],
							nr_4d_0101[bit_i][bit_j][bit_k][bit_l],
							nr_4d_0110[bit_i][bit_j][bit_k][bit_l],
							nr_4d_0111[bit_i][bit_j][bit_k][bit_l],
							nr_4d_1000[bit_i][bit_j][bit_k][bit_l],
							nr_4d_1001[bit_i][bit_j][bit_k][bit_l],
							nr_4d_1010[bit_i][bit_j][bit_k][bit_l],
							nr_4d_1011[bit_i][bit_j][bit_k][bit_l],
							nr_4d_1100[bit_i][bit_j][bit_k][bit_l],
							nr_4d_1101[bit_i][bit_j][bit_k][bit_l],
							nr_4d_1110[bit_i][bit_j][bit_k][bit_l],
							nr_4d_1111[bit_i][bit_j][bit_k][bit_l]);
					}
				}
			}
		}
		break;
	}

	default:
		abort();
		break;
	}
	printf("\n");
}

static
void select_bits_greedy(void)
{
	uint8_t best_bits[4];
	uint8_t bit_one;
	uint8_t bit_classes4[4];
	uint8_t bit_classes8[8];
	uint8_t bit_classes16[16];
	uint8_t mask[4];
	unsigned int bit;
	int item, min_distance_to_best, best_bit;
	int min_distance[4];
	struct timespec t1, t2;

	clock_gettime(CLOCK_MONOTONIC, &t1);
	memset(mask, 0, sizeof(mask));
	memset(best_bits, 0, sizeof(best_bits));

	/* Select first bit. */
	best_bit = -1;
	min_distance_to_best = INT_MAX;
	for (bit = 0; bit < 8; bit++) {
		int distance_to_best;

		bit_one = 0;
		for (item = 0; item < nr_items; item++) {
			if (pop[item] & (1U << bit))
				bit_one++;
		}
		distance_to_best = ((unsigned int) bit_one << 1U) - nr_items;
		if (distance_to_best < 0)
			distance_to_best = -distance_to_best;
		if (distance_to_best < min_distance_to_best) {
			min_distance_to_best = distance_to_best;
			best_bit = bit;
		}
	}
	best_bits[0] = best_bit;
	mask[0] = 1U << best_bit;
	min_distance[0] = min_distance_to_best;

	/* Select second bit. */
	best_bit = -1;
	min_distance_to_best = INT_MAX;
	for (bit = 0; bit < 8; bit++) {
		int max_distance_to_best = INT_MIN;
		unsigned int class_index;

		if (bit == best_bits[0])
			continue;
		memset(bit_classes4, 0, sizeof(bit_classes4));
		for (item = 0; item < nr_items; item++) {
			class_index = (((pop[item] & mask[0]) >> best_bits[0]) << 1) | ((pop[item] & (1U << bit)) >> bit);
			assert(class_index < 4);
			bit_classes4[class_index]++;
		}
		for (class_index = 0; class_index < 4; class_index++) {
			int distance_to_best;

			distance_to_best = ((unsigned int) bit_classes4[class_index] * 4) - nr_items;
			if (distance_to_best < 0)
				distance_to_best = -distance_to_best;
			if (distance_to_best > max_distance_to_best) {
				max_distance_to_best = distance_to_best;
			}
		}
		if (max_distance_to_best < min_distance_to_best) {
			min_distance_to_best = max_distance_to_best;
			best_bit = bit;
		}
	}
	best_bits[1] = best_bit;
	mask[1] = 1U << best_bit;
	min_distance[1] = min_distance_to_best;

	/* Select 3rd bit. */
	best_bit = -1;
	min_distance_to_best = INT_MAX;
	for (bit = 0; bit < 8; bit++) {
		int max_distance_to_best = INT_MIN;
		unsigned int class_index;

		if (bit == best_bits[0] || bit == best_bits[1])
			continue;
		memset(bit_classes8, 0, sizeof(bit_classes8));
		for (item = 0; item < nr_items; item++) {
			class_index = (((pop[item] & mask[0]) >> best_bits[0]) << 2) |
					(((pop[item] & mask[1]) >> best_bits[1]) << 1) |
					((pop[item] & (1U << bit)) >> bit);
			assert(class_index < 8);
			bit_classes8[class_index]++;
		}
		for (class_index = 0; class_index < 8; class_index++) {
			int distance_to_best = ((unsigned int) bit_classes8[class_index] * 8) - nr_items;

			if (distance_to_best < 0)
				distance_to_best = -distance_to_best;
			if (distance_to_best > max_distance_to_best) {
				max_distance_to_best = distance_to_best;
			}
		}
		if (max_distance_to_best < min_distance_to_best) {
			min_distance_to_best = max_distance_to_best;
			best_bit = bit;
		}
	}
	best_bits[2] = best_bit;
	mask[2] = 1U << best_bit;
	min_distance[2] = min_distance_to_best;

	/* Select 4th bit. */
	best_bit = -1;
	min_distance_to_best = INT_MAX;
	for (bit = 0; bit < 8; bit++) {
		int max_distance_to_best = INT_MIN;
		unsigned int class_index;

		if (bit == best_bits[0] || bit == best_bits[1] || bit == best_bits[2])
			continue;
		memset(bit_classes16, 0, sizeof(bit_classes16));
		for (item = 0; item < nr_items; item++) {
			class_index = (((pop[item] & mask[0]) >> best_bits[0]) << 3) |
					(((pop[item] & mask[1]) >> best_bits[1]) << 2) |
					(((pop[item] & mask[2]) >> best_bits[2]) << 1) |
					((pop[item] & (1U << bit)) >> bit);
			assert(class_index < 16);
			bit_classes16[class_index]++;
		}
		for (class_index = 0; class_index < 16; class_index++) {
			int distance_to_best = ((unsigned int) bit_classes16[class_index] * 16) - nr_items;

			if (distance_to_best < 0)
				distance_to_best = -distance_to_best;
			if (distance_to_best > max_distance_to_best) {
				max_distance_to_best = distance_to_best;
			}
		}
		if (max_distance_to_best < min_distance_to_best) {
			min_distance_to_best = max_distance_to_best;
			best_bit = bit;
		}
	}
	best_bits[3] = best_bit;
	mask[3] = 1U << best_bit;
	min_distance[3] = min_distance_to_best;
	clock_gettime(CLOCK_MONOTONIC, &t2);
	if (verbose) {
		printf("Best bits selection (greedy): (%u, %u, %u, %u) min_distance (%d, %d, %d, %d)\n",
			(unsigned int)best_bits[0],
			(unsigned int)best_bits[1],
			(unsigned int)best_bits[2],
			(unsigned int)best_bits[3],
			min_distance[0],
			min_distance[1],
			min_distance[2],
			min_distance[3]);
		printf("Calculated in %" PRIu64 "ns\n", diff_time(&t1, &t2));
	}
}

/*
 * distance_to_best is the delta between an ideal two-subclasses split
 * (50%, 50%) and the number of items within each sub-classes for the
 * given bit selection.
 */
static
void select_1bit_bruteforce(uint8_t *result)
{
	uint8_t best_bits[4];
	uint8_t bit_one[8];
	unsigned int i, bit;
	int item, min_distance_to_best, best_bit;
	struct timespec t1, t2;
	int min_distance_max_class_items;

	memset(best_bits, 0, sizeof(best_bits));

	/* Select first bit. */
	clock_gettime(CLOCK_MONOTONIC, &t1);
	memset(bit_one, 0, sizeof(bit_one));
	for (item = 0; item < nr_items; item++) {
		for (bit = 0; bit < 8; bit++) {
			if (pop[item] & (1U << bit))
				bit_one[bit]++;
		}
	}
	best_bit = -1;
	min_distance_to_best = INT_MAX;
	for (i = 0; i < 8; i++) {
		int distance_to_best;

		distance_to_best = ((unsigned int) bit_one[i] << 1U) - nr_items;
		if (distance_to_best < 0)
			distance_to_best = -distance_to_best;
		if (distance_to_best < min_distance_to_best) {
			min_distance_to_best = distance_to_best;
			best_bit = i;
		}
	}
	best_bits[0] = best_bit;
	min_distance_max_class_items = (min_distance_to_best + nr_items) >> 1U;
	clock_gettime(CLOCK_MONOTONIC, &t2);

	if (result) {
		result[0] = best_bits[0];
	} else {
		if (verbose) {
			printf("Best single-bit selection (brute force): (%u) min_distance: %d max_class_items: %d\n",
				(unsigned int) best_bits[0], min_distance_to_best, min_distance_max_class_items);
			printf("Calculated in %" PRIu64 "ns\n", diff_time(&t1, &t2));
		}
	}
	if (min_distance_max_class_items > (int)global_min_distance_max_class_items) {
		global_min_distance_max_class_items = min_distance_max_class_items;
	}
	histogram_min_distance_max_class_items[min_distance_max_class_items]++;
}

/*
 * distance_to_best is the delta between an ideal 4-subclasses split
 * (each 25%) and the number of items within each sub-classes for the
 * given bit selection.
 */
static
void select_2bits_bruteforce(uint8_t *result, int nr_greedy)
{
	uint8_t best_bits[4];
	uint8_t best_mask;
	unsigned int i, combination, best_combination;
	int item, min_distance_to_best;
	size_t nr_combinations;
	struct timespec t1, t2;
	int min_distance_max_class_items = -1;

	memset(best_bits, 0, sizeof(best_bits));

	if (nr_greedy >= 1)
		select_1bit_bruteforce(best_bits);

	/* Select 2 bits. */
	clock_gettime(CLOCK_MONOTONIC, &t1);
	best_mask = 0;
	best_combination = -1;
	min_distance_to_best = INT_MAX;
	nr_combinations = combinations_n_in_8[2].len;
	for (combination = 0; combination < nr_combinations; combination++) {
		uint8_t bitmask = index_to_bitmask(2, combination);
		int max_distance_to_best = INT_MIN;
		int max_class_items = INT_MIN;
		uint8_t class_count[4];

		if (nr_greedy >= 1) {
			if (!(bitmask & (1U << best_bits[0])))
				continue;
		}

		memset(class_count, 0, sizeof(class_count));
		for (item = 0; item < nr_items; item++) {
			unsigned int subclass = value_and_mask_to_subclass_index(pop[item], bitmask);
			assert(subclass < 4);
			class_count[subclass]++;
		}
		for (i = 0; i < 4; i++) {
			int distance_to_best;

			distance_to_best = ((unsigned int) class_count[i] << 2U) - nr_items;
			if (distance_to_best < 0)
				distance_to_best = -distance_to_best;
			if (distance_to_best > max_distance_to_best)
				max_distance_to_best = distance_to_best;
			if (max_class_items < class_count[i])
				max_class_items = class_count[i];
		}
		if (max_distance_to_best < min_distance_to_best) {
			min_distance_to_best = max_distance_to_best;
			best_mask = bitmask;
			best_combination = combination;
			min_distance_max_class_items = max_class_items;
		}
	}
	best_bits[0] = __builtin_ctz(best_mask);
	best_bits[1] = __builtin_ctz(best_mask >> (best_bits[0] + 1)) + (best_bits[0] + 1);
	clock_gettime(CLOCK_MONOTONIC, &t2);

	if (result) {
		result[0] = best_bits[0];
		result[1] = best_bits[1];
	} else {
		if (verbose) {
			printf("Best 2 bits selection (brute force, greedy=%d): (%u, %u) mask: 0x%x combination_index: %u min_distance: %d max_class_items: %d\n",
				nr_greedy, (unsigned int) best_bits[0], (unsigned int) best_bits[1],
				(unsigned int) best_mask, best_combination, min_distance_to_best, min_distance_max_class_items);
			printf("Calculated in %" PRIu64 "ns\n", diff_time(&t1, &t2));
		}
	}
	if (min_distance_max_class_items > (int)global_min_distance_max_class_items) {
		global_min_distance_max_class_items = min_distance_max_class_items;
	}
	histogram_min_distance_max_class_items[min_distance_max_class_items]++;
}

/*
 * distance_to_best is the delta between an ideal 8-subclasses split
 * (each 12.5%) and the number of items within each sub-classes for the
 * given bit selection.
 */
static
void select_3bits_bruteforce(uint8_t *result, int nr_greedy)
{
	uint8_t best_bits[4];
	uint8_t best_mask;
	unsigned int i, combination, best_combination;
	int item, min_distance_to_best;
	size_t nr_combinations;
	struct timespec t1, t2;
	int min_distance_max_class_items = -1;

	memset(best_bits, 0, sizeof(best_bits));

	if (nr_greedy >= 2)
		select_2bits_bruteforce(best_bits, nr_greedy);
	else if (nr_greedy >= 1)
		select_1bit_bruteforce(best_bits);

	/* Select 3 bits. */
	clock_gettime(CLOCK_MONOTONIC, &t1);
	best_mask = 0;
	best_combination = -1;
	min_distance_to_best = INT_MAX;
	nr_combinations = combinations_n_in_8[3].len;
	for (combination = 0; combination < nr_combinations; combination++) {
		uint8_t bitmask = index_to_bitmask(3, combination);
		int max_distance_to_best = INT_MIN;
		int max_class_items = INT_MIN;
		uint8_t class_count[8];

		if (nr_greedy >= 1) {
			if (!(bitmask & (1U << best_bits[0])))
				continue;
		}
		if (nr_greedy >= 2) {
			if (!(bitmask & (1U << best_bits[1])))
				continue;
		}

		memset(class_count, 0, sizeof(class_count));
		for (item = 0; item < nr_items; item++) {
			unsigned int subclass = value_and_mask_to_subclass_index(pop[item], bitmask);
			assert(subclass < 8);
			class_count[subclass]++;
		}
		for (i = 0; i < 8; i++) {
			int distance_to_best;

			distance_to_best = ((unsigned int) class_count[i] << 3U) - nr_items;
			if (distance_to_best < 0)
				distance_to_best = -distance_to_best;
			if (distance_to_best > max_distance_to_best)
				max_distance_to_best = distance_to_best;
			if (max_class_items < class_count[i])
				max_class_items = class_count[i];
		}
		if (max_distance_to_best < min_distance_to_best) {
			min_distance_to_best = max_distance_to_best;
			best_mask = bitmask;
			best_combination = combination;
			min_distance_max_class_items = max_class_items;
		}
	}
	best_bits[0] = __builtin_ctz(best_mask);
	best_bits[1] = __builtin_ctz(best_mask >> (best_bits[0] + 1)) + best_bits[0] + 1;
	best_bits[2] = __builtin_ctz(best_mask >> (best_bits[1] + 1)) + best_bits[1] + 1;
	clock_gettime(CLOCK_MONOTONIC, &t2);

	if (result) {
		result[0] = best_bits[0];
		result[1] = best_bits[1];
		result[2] = best_bits[2];
	} else {
		if (verbose) {
			printf("Best 3 bits selection (brute force, greedy=%d): (%u, %u, %u) mask: 0x%x combination_index: %u min_distance: %d max_class_items: %d\n",
				nr_greedy, (unsigned int) best_bits[0], (unsigned int) best_bits[1], (unsigned int) best_bits[2],
				(unsigned int) best_mask, best_combination, min_distance_to_best, min_distance_max_class_items);
			printf("Calculated in %" PRIu64 "ns\n", diff_time(&t1, &t2));
		}
	}
	if (min_distance_max_class_items > (int)global_min_distance_max_class_items) {
		global_min_distance_max_class_items = min_distance_max_class_items;
	}
	histogram_min_distance_max_class_items[min_distance_max_class_items]++;
}

/*
 * distance_to_best is the delta between an ideal 16-subclasses split
 * (each 6.25%) and the number of items within each sub-classes for the
 * given bit selection.
 */
static
void select_4bits_bruteforce(uint8_t *result, int nr_greedy)
{
	uint8_t best_bits[4];
	uint8_t best_mask;
	unsigned int i, combination, best_combination;
	int item, min_distance_to_best;
	size_t nr_combinations;
	struct timespec t1, t2;
	int min_distance_max_class_items = -1;

	memset(best_bits, 0, sizeof(best_bits));

	if (nr_greedy >= 3)
		select_3bits_bruteforce(best_bits, nr_greedy);
	else if (nr_greedy >= 2)
		select_2bits_bruteforce(best_bits, nr_greedy);
	else if (nr_greedy >= 1)
		select_1bit_bruteforce(best_bits);

	/* Select 4 bits. */
	clock_gettime(CLOCK_MONOTONIC, &t1);
	best_mask = 0;
	best_combination = -1;
	min_distance_to_best = INT_MAX;
	nr_combinations = combinations_n_in_8[4].len;
	for (combination = 0; combination < nr_combinations; combination++) {
		uint8_t bitmask = index_to_bitmask(4, combination);
		int max_distance_to_best = INT_MIN;
		int max_class_items = INT_MIN;
		uint8_t class_count[16];

		if (nr_greedy >= 1) {
			if (!(bitmask & (1U << best_bits[0])))
				continue;
		}
		if (nr_greedy >= 2) {
			if (!(bitmask & (1U << best_bits[1])))
				continue;
		}
		if (nr_greedy >= 3) {
			if (!(bitmask & (1U << best_bits[2])))
				continue;
		}

		memset(class_count, 0, sizeof(class_count));
		for (item = 0; item < nr_items; item++) {
			unsigned int subclass = value_and_mask_to_subclass_index(pop[item], bitmask);
			assert(subclass < 16);
			class_count[subclass]++;
		}
		for (i = 0; i < 16; i++) {
			int distance_to_best;

			distance_to_best = ((unsigned int) class_count[i] << 4U) - nr_items;
			if (distance_to_best < 0)
				distance_to_best = -distance_to_best;
			if (distance_to_best > max_distance_to_best)
				max_distance_to_best = distance_to_best;
			if (max_class_items < class_count[i])
				max_class_items = class_count[i];
		}
		if (max_distance_to_best < min_distance_to_best) {
			min_distance_to_best = max_distance_to_best;
			best_mask = bitmask;
			best_combination = combination;
			min_distance_max_class_items = max_class_items;
		}
	}
	best_bits[0] = __builtin_ctz(best_mask);
	best_bits[1] = __builtin_ctz(best_mask >> (best_bits[0] + 1)) + best_bits[0] + 1;
	best_bits[2] = __builtin_ctz(best_mask >> (best_bits[1] + 1)) + best_bits[1] + 1;
	best_bits[3] = __builtin_ctz(best_mask >> (best_bits[2] + 1)) + best_bits[2] + 1;
	clock_gettime(CLOCK_MONOTONIC, &t2);

	if (result) {
		result[0] = best_bits[0];
		result[1] = best_bits[1];
		result[2] = best_bits[2];
		result[3] = best_bits[3];
	} else {
		if (verbose) {
			printf("Best 4 bits selection (brute force, greedy=%d): (%u, %u, %u, %u) mask: 0x%x combination_index: %u min_distance: %d max_class_items: %d\n",
				nr_greedy, (unsigned int) best_bits[0], (unsigned int) best_bits[1], (unsigned int) best_bits[2],
				(unsigned int) best_bits[3],
				(unsigned int) best_mask, best_combination, min_distance_to_best, min_distance_max_class_items);
			printf("Calculated in %" PRIu64 "ns\n", diff_time(&t1, &t2));
		}
	}

	if (min_distance_max_class_items > (int)global_min_distance_max_class_items) {
		global_min_distance_max_class_items = min_distance_max_class_items;
	}
	histogram_min_distance_max_class_items[min_distance_max_class_items]++;
}

/*
 * distance_to_best is the delta between an ideal four-subclasses split
 * (25%, 25%, 25%, 25%) and the number of items within the sub-class
 * which is at the largest distance from the ideal for the given bit
 * selection.
 */
static
void select_2bits_bruteforce_bit_breakdown(void)
{
	int overall_best_distance = INT_MAX;
	int min_distance_max_class_items = 0;
	int bit_i, bit_j, i;
	uint8_t bit_2d_11[8][8];
	uint8_t bit_2d_10[8][8];
	uint8_t bit_2d_01[8][8];
	uint8_t bit_2d_00[8][8];
	uint8_t best_bits[2];
	struct timespec t1, t2;

	memset(best_bits, 0, sizeof(best_bits));
	memset(bit_2d_11, 0, sizeof(bit_2d_11));
	memset(bit_2d_10, 0, sizeof(bit_2d_10));
	memset(bit_2d_01, 0, sizeof(bit_2d_01));
	memset(bit_2d_00, 0, sizeof(bit_2d_00));

	clock_gettime(CLOCK_MONOTONIC, &t1);
	for (i = 0; i < nr_items; i++) {
		/* 2 selection bits */
		for (bit_i = 0; bit_i < 8; bit_i++) {
			for (bit_j = bit_i + 1; bit_j < 8; bit_j++) {
				if (pop[i] & (1U << bit_i)) {
					if (pop[i] & (1U << bit_j)) {
						bit_2d_11[bit_i][bit_j]++;
					} else {
						bit_2d_10[bit_i][bit_j]++;
					}
				} else {
					if (pop[i] & (1U << bit_j)) {
						bit_2d_01[bit_i][bit_j]++;
					} else {
						bit_2d_00[bit_i][bit_j]++;
					}
				}
			}
		}
	}

	for (bit_i = 0; bit_i < 8; bit_i++) {
		for (bit_j = bit_i + 1; bit_j < 8; bit_j++) {
			int distance_to_best[4], subclass_len[4];

			distance_to_best[0] = ((unsigned int) bit_2d_11[bit_i][bit_j] << 2U) - nr_items;
			distance_to_best[1] = ((unsigned int) bit_2d_10[bit_i][bit_j] << 2U) - nr_items;
			distance_to_best[2] = ((unsigned int) bit_2d_01[bit_i][bit_j] << 2U) - nr_items;
			distance_to_best[3] = ((unsigned int) bit_2d_00[bit_i][bit_j] << 2U) - nr_items;

			subclass_len[0] = bit_2d_11[bit_i][bit_j];
			subclass_len[1] = bit_2d_10[bit_i][bit_j];
			subclass_len[2] = bit_2d_01[bit_i][bit_j];
			subclass_len[3] = bit_2d_00[bit_i][bit_j];

			/* Consider worse distance above best */
			if (distance_to_best[1] > 0 && distance_to_best[1] > distance_to_best[0]) {
				distance_to_best[0] = distance_to_best[1];
				subclass_len[0] = subclass_len[1];
			}
			if (distance_to_best[2] > 0 && distance_to_best[2] > distance_to_best[0]) {
				distance_to_best[0] = distance_to_best[2];
				subclass_len[0] = subclass_len[2];
			}
			if (distance_to_best[3] > 0 && distance_to_best[3] > distance_to_best[0]) {
				distance_to_best[0] = distance_to_best[3];
				subclass_len[0] = subclass_len[3];
			}

			/*
			 * If our worse distance is better than overall,
			 * we become new best candidate.
			 */
			if (distance_to_best[0] < overall_best_distance) {
				overall_best_distance = distance_to_best[0];
				min_distance_max_class_items = subclass_len[0];
				best_bits[0] = bit_i;
				best_bits[1] = bit_j;
			}
		}
	}
	clock_gettime(CLOCK_MONOTONIC, &t2);
	if (verbose) {
		printf("Bits-breakdown 2 bits selection (brute force): (%u, %u) min_distance: %d max_class_items: %d\n",
			(unsigned int) best_bits[0], (unsigned int) best_bits[1],
			overall_best_distance, min_distance_max_class_items);
		printf("Calculated in %" PRIu64 "ns\n", diff_time(&t1, &t2));
	}
	if (min_distance_max_class_items > (int) global_min_distance_max_class_items) {
		global_min_distance_max_class_items = min_distance_max_class_items;
	}
	histogram_min_distance_max_class_items[min_distance_max_class_items]++;
}


static
void stat_count(int nr_classes, int nr_greedy)
{
	select_bits_greedy();

	switch (nr_classes) {
	case 2:
		select_1bit_bruteforce(NULL);
		break;
	case 4:
		if (opt_bit_breakdown)
			select_2bits_bruteforce_bit_breakdown();
		else
			select_2bits_bruteforce(NULL, nr_greedy);
		break;
	case 8:
		select_3bits_bruteforce(NULL, nr_greedy);
		break;
	case 16:
		select_4bits_bruteforce(NULL, nr_greedy);
		break;
	}
}

static
void print_distrib(void)
{
	int i;
	unsigned long long tot = 0;

	for (i = 0; i < 256; i++) {
		tot += histogram_min_distance_max_class_items[i];
	}
	if (tot == 0)
		return;
	printf("Population distribution: (max_subclass_items, population count, ratio (%%)): ");
	for (i = 0; i < 256; i++) {
		if (!histogram_min_distance_max_class_items[i])
			continue;
		printf("(%u, %u, %llu%%) ",
			i, histogram_min_distance_max_class_items[i],
			100 * (unsigned long long) histogram_min_distance_max_class_items[i] / tot);
	}
	printf("\n");
	printf("\n");
}

static
void print_stat(uint64_t i)
{
	printf("After %llu populations, the largest observed min-distance max subclass items is: %u\n",
		(unsigned long long) i, global_min_distance_max_class_items);
	print_distrib();
}

static
void show_usage(char **argv)
{
	printf("Usage : %s <OPTIONS>\n", argv[0]);
	printf("OPTIONS:\n");
	printf("    [-s N]: Split each population into N sub-classes (2, 4, 8, or 16. Default %d).\n",
		DEFAULT_NR_CLASSES);
	printf("    [-i N]: Number of items to insert into each population (between 1 and 256, default %d).\n",
		DEFAULT_NR_ITEMS);
	printf("    [-p N]: Number of populations to generate (default %llu).\n",
		DEFAULT_NR_POP);
	printf("    [-g N]: Number of greedy bit search (default %u).\n",
		DEFAULT_NR_GREEDY);
	printf("    [-B]: Bit-breakdown.\n");
	printf("    [-v]: Verbose output.\n");
	printf("    [-h]: Show help.\n");
	printf("\n");
}

static
int parse_args(int argc, char **argv)
{
	int i;

	for (i = 1; i < argc; i++) {
		if (argv[i][0] != '-') {
			printf("Unrecognized argument %s\n", argv[i]);
			goto error;
		}
		switch (argv[i][1]) {
		case 'i':
			if (argc < i + 2)
				goto error;
			nr_items = atoi(argv[++i]);
			break;
		case 's':
			if (argc < i + 2)
				goto error;
			opt_nr_classes = atoi(argv[++i]);
			break;
		case 'g':
			if (argc < i + 2)
				goto error;
			opt_nr_greedy = atoi(argv[++i]);
			break;
		case 'p':
		{
			char *endptr = NULL;

			if (argc < i + 2)
				goto error;
			nr_pop = strtoull(argv[++i], &endptr, 10);
			if (endptr == argv[i]) {
				printf("Unrecognized number of populations\n");
				goto error;
			}
			break;
		}
		case 'B':
			opt_bit_breakdown = 1;
			break;
		case 'v':
			verbose = 1;
			break;
		case 'h':
			show_usage(argv);
			return 1;
		}
	}
	return 0;

error:
	show_usage(argv);
	return -1;
}

int main(int argc, char **argv)
{
	uint64_t i = 0;
	int ret;

	srandom(time(NULL));

	ret = parse_args(argc, argv);
	if (ret < 0)
		return ret;
	if (ret > 0)
		return 0;	/* Show usage. */

	printf("Number of populations to generate: %" PRIu64 "\n", nr_pop);
	printf("Number of items to insert into each population: %d\n", nr_items);
	printf("Number of sub-classes: %d\n", opt_nr_classes);
	printf("N levels of greedy search: %d\n", opt_nr_greedy);
	printf("Bit-breakdown search: %d\n", opt_bit_breakdown);

	if (nr_items > 256 || nr_items < 1) {
		printf("Wrong number of items to insert: %d\n", nr_items);
		show_usage(argv);
		return -1;
	}
	switch (opt_nr_classes) {
	case 2:
	case 4:
	case 8:
	case 16:
		break;
	default:
		printf("Wrong number of population sub-classes '%d'.\n",
			opt_nr_classes);
		show_usage(argv);
		return -1;
	}

	if (opt_bit_breakdown && opt_nr_classes != 4) {
		printf("bit-breakdown only implemented for 4 classes\n");
		show_usage(argv);
		return -1;
	}

	for (i = 0; i < nr_pop; i++) {
		if (verbose)
			printf("-------------------------------------------------------------------\n");
		if (i && !(i % 10000ULL))
			print_stat(i);
		gen_pop();
		count_pop(opt_nr_classes);
		stat_count(opt_nr_classes, opt_nr_greedy);
		if (verbose) {
			print_pop();
			print_count(opt_nr_classes);
		}
	}
	print_stat(i);

	return 0;
}
