/*
 * rcuja-testpop.c
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

#define DEFAULT_NR_ITEMS	50
#define DEFAULT_NR_CLASSES	2
#define DEFAULT_NR_POP	10000000ULL

static int nr_items = DEFAULT_NR_ITEMS;
static int nr_classes = DEFAULT_NR_CLASSES;
static uint64_t nr_pop = DEFAULT_NR_POP;

static uint8_t pop[256];
static uint8_t nr_one[8];
static uint8_t nr_2d_11[8][8];
static uint8_t nr_2d_10[8][8];
static uint8_t nr_2d_01[8][8];
static uint8_t nr_2d_00[8][8];

/*
 * global_max_minsubclass_len keeps track of the largest observed
 * overall_minsubclass_len across all generated populations.
 */
static unsigned int global_max_minsubclass_len = 0;

/*
 * subclass_len_distrib is an histogram counting the number of
 * populations per overall_minsubclass_len across all generated
 * populations.
 */
static unsigned int subclass_len_distrib[256];

static int verbose;

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
void count_pop(void)
{
	int i;

	memset(nr_one, 0, sizeof(nr_one));
	memset(nr_2d_11, 0, sizeof(nr_2d_11));
	memset(nr_2d_10, 0, sizeof(nr_2d_10));
	memset(nr_2d_01, 0, sizeof(nr_2d_01));
	memset(nr_2d_00, 0, sizeof(nr_2d_00));

	for (i = 0; i < nr_items; i++) {
		switch (nr_classes) {
		case 2:
		{
			int bit;

			for (bit = 0; bit < 8; bit++) {
				if (pop[i] & (1U << bit))
					nr_one[bit]++;
			}
			break;
		}
		case 4:
		{
			int bit_i, bit_j;

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
		}
		default:
			abort();
			break;
		}
	}
}

static
void print_count(void)
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
	default:
		abort();
		break;
	}
	printf("\n");
}

/*
 * distance_to_best is the delta between an ideal two-subclasses split
 * (50%, 50%) and the number of items within each sub-classes for the
 * given bit selection.
 *
 * overall_best_distance is the minimum distance_to_best across all
 * possible bits.
 *
 * overall_minsubclass_len is the size of the largest of the two
 * resulting subclasses associated with overall_best_distance.
 */
static
void stat_count_1d(void)
{
	unsigned int overall_best_distance = UINT_MAX;
	unsigned int overall_minsubclass_len;
	int i;

	for (i = 0; i < 8; i++) {
		int distance_to_best;

		distance_to_best = ((unsigned int) nr_one[i] << 1U) - nr_items;
		if (distance_to_best < 0)
			distance_to_best = -distance_to_best;
		if ((unsigned int) distance_to_best < overall_best_distance) {
			overall_best_distance = distance_to_best;
		}
	}
	overall_minsubclass_len = (overall_best_distance + nr_items) >> 1UL;
	if (overall_minsubclass_len > global_max_minsubclass_len) {
		global_max_minsubclass_len = overall_minsubclass_len;
	}
	subclass_len_distrib[overall_minsubclass_len]++;
}

/*
 * distance_to_best is the delta between an ideal four-subclasses split
 * (25%, 25%, 25%, 25%) and the number of items within the sub-class
 * which is at the largest distance from the ideal for the given bit
 * selection.
 *
 * overall_best_distance is the minimum distance_to_best across all
 * possible pairs of bit selection.
 *
 * overall_minsubclass_len is the size of the largest of the four
 * resulting subclasses associated with overall_best_distance.
 */
static
void stat_count_2d(void)
{
	int overall_best_distance = INT_MAX;
	unsigned int overall_minsubclass_len = 0;
	int bit_i, bit_j;

	for (bit_i = 0; bit_i < 8; bit_i++) {
		for (bit_j = bit_i + 1; bit_j < 8; bit_j++) {
			int distance_to_best[4], subclass_len[4];

			distance_to_best[0] = ((unsigned int) nr_2d_11[bit_i][bit_j] << 2U) - nr_items;
			distance_to_best[1] = ((unsigned int) nr_2d_10[bit_i][bit_j] << 2U) - nr_items;
			distance_to_best[2] = ((unsigned int) nr_2d_01[bit_i][bit_j] << 2U) - nr_items;
			distance_to_best[3] = ((unsigned int) nr_2d_00[bit_i][bit_j] << 2U) - nr_items;

			subclass_len[0] = nr_2d_11[bit_i][bit_j];
			subclass_len[1] = nr_2d_10[bit_i][bit_j];
			subclass_len[2] = nr_2d_01[bit_i][bit_j];
			subclass_len[3] = nr_2d_00[bit_i][bit_j];

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
				overall_minsubclass_len = subclass_len[0];
			}
		}
	}
	if (overall_minsubclass_len > global_max_minsubclass_len) {
		global_max_minsubclass_len = overall_minsubclass_len;
	}
	subclass_len_distrib[overall_minsubclass_len]++;
}

static
void stat_count(void)
{
	switch (nr_classes) {
	case 2:
		stat_count_1d();
		break;
	case 4:
		stat_count_2d();
		break;
	default:
		abort();
		break;
	}
}

static
void print_distrib(void)
{
	int i;
	unsigned long long tot = 0;

	for (i = 0; i < 256; i++) {
		tot += subclass_len_distrib[i];
	}
	if (tot == 0)
		return;
	printf("Population distribution: (subclass_len, population count, ratio (%%))\n");
	for (i = 0; i < 256; i++) {
		if (!subclass_len_distrib[i])
			continue;
		printf("(%u, %u, %llu%%) ",
			i, subclass_len_distrib[i],
			100 * (unsigned long long) subclass_len_distrib[i] / tot);
	}
	printf("\n");
	printf("\n");
}

static
void print_stat(uint64_t i)
{
	printf("After %llu populations, the largest observed minimum population subclass length (subclass_len) is: %u\n",
		(unsigned long long) i, global_max_minsubclass_len);
	print_distrib();
}

static
void show_usage(char **argv)
{
	printf("Usage : %s <OPTIONS>\n", argv[0]);
	printf("OPTIONS:\n");
	printf("    [-s N]: Split each population into N sub-classes (2 or 4, default %d).\n",
		DEFAULT_NR_CLASSES);
	printf("    [-i N]: Number of items to insert into each population (between 1 and 256, default %d).\n",
		DEFAULT_NR_ITEMS);
	printf("    [-p N]: Number of populations to generate (default %llu).\n",
		DEFAULT_NR_POP);
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
			nr_classes = atoi(argv[++i]);
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
	printf("Number of sub-classes: %d\n", nr_classes);

	if (nr_items > 256 || nr_items < 1) {
		printf("Wrong number of items to insert: %d\n", nr_items);
		show_usage(argv);
		return -1;
	}
	if (nr_classes != 2 && nr_classes != 4) {
		printf("Wrong number of population sub-classes '%d'. Only 2 and 4 supported.\n",
			nr_classes);
		show_usage(argv);
		return -1;
	}

	for (i = 0; i < nr_pop; i++) {
		if (i && !(i % 100000ULL))
			print_stat(i);
		gen_pop();
		count_pop();
		if (verbose) {
			print_pop();
			print_count();
		}
		stat_count();
	}
	print_stat(i);

	return 0;
}
