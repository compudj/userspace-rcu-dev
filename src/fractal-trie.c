// SPDX-FileCopyrightText: 2012-2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-only

/*
 * src/fractal-trie.c
 *
 * Userspace RCU library - Fractal Trie
 */

#define _LGPL_SOURCE
#include <stdint.h>
#include <errno.h>
#include <limits.h>
#include <string.h>
#include <assert.h>
#include <endian.h>
#include <stdbool.h>
#include <urcu/fractal-trie.h>
#include <urcu/compiler.h>
#include <urcu/arch.h>
#include <urcu-pointer.h>
#include <urcu/uatomic.h>

#include "fractal-trie-internal.h"
#include "bitmap.h"

#if defined(FEATURE_SIMD_LOOKUP) && (defined(__AVX2__) || defined(__SSE2__))
# include <immintrin.h>
#endif

#ifndef abs
#define abs_int(a)	((int) (a) > 0 ? (int) (a) : -((int) (a)))
#endif

#define CDS_FT_LEN_ERROR		SIZE_MAX

struct cds_ft_attr {
	size_t key_len;
	size_t max_key_len;
	struct cds_ft_key_map key_map;
};

enum cds_ft_type_class {
	FT_LINEAR = 0,	/* Type A */
			/* 32-bit: 1 to 25 children, 8 to 128 bytes */
			/* 64-bit: 1 to 28 children, 16 to 256 bytes */
	FT_POOL = 1,	/* Type B */
			/* 32-bit: 26 to 100 children, 256 to 512 bytes */
			/* 64-bit: 29 to 112 children, 512 to 1024 bytes */
	FT_PIGEON = 2,	/* Type C */
			/* 32-bit: 101 to 256 children, 1024 bytes */
			/* 64-bit: 113 to 256 children, 2048 bytes */
	/* Leaf nodes are implicit from their height in the tree */
	FT_NR_TYPES,

	FT_NULL,	/* not an encoded type, but keeps code regular */
};

struct cds_ft_type {
	enum cds_ft_type_class type_class;
	uint16_t min_child;		/* minimum number of children: 1 to 256 */
	uint16_t max_child;		/* maximum number of children: 1 to 256 */
	uint16_t max_linear_child;	/* per-pool max nr. children: 1 to 256 */
	uint16_t order;			/* node size is (1 << order), in bytes */
	uint16_t nr_pool_order;		/* number of pools */
	uint16_t pool_size_order;	/* pool size */
	bool bitmap;			/* allocate bitmap */
};

/*
 * Iteration on the array to find the right node size for the number of
 * children stops when it reaches .max_child == 256 (this is the largest
 * possible node size, which contains 256 children).
 * The min_child overlaps with the previous max_child to provide an
 * hysteresis loop to reallocation for patterns of cyclic add/removal
 * within the same node.
 * The node the index within the following arrays is represented on 3
 * bits. It identifies the node type, min/max number of children, and
 * the size order.
 * The max_child values for the FT_POOL below result from
 * statistical approximation: over million populations, the max_child
 * covers between 97% and 99% of the populations generated. Therefore, a
 * fallback should exist to cover the rare extreme population unbalance
 * cases, but it will not have a major impact on speed nor space
 * consumption, since those are rare cases.
 */

/*
 * The smallest allocation order we can use is 4:
 * - 1 bit is reserved for internal vs external flag,
 * - 3 bits are reserved to encode the node type.
 */

/*
 * The cds_ft_node contains the compressed node data needed for
 * read-side. For linear node and pool sub-nodes, it starts with a byte
 * counting the number of children values (which may have NULL or
 * non-NULL pointers) in the node. Then, the node-specific data is
 * placed. For all configuration, the number of children (with non-NULL
 * pointers) is kept in the metadata associated to node.
 */

#define DECLARE_LINEAR_NODE(index)								\
	struct {										\
		uint8_t nr_child;								\
		uint8_t child_value[ft_type_## index ##_max_linear_child];			\
		struct cds_ft_inode_flag *child_ptr[ft_type_## index ##_max_linear_child];	\
	}

#define DECLARE_POOL_NODE(index)								\
	struct {										\
		struct {									\
			uint8_t nr_child;							\
			uint8_t child_value[ft_type_## index ##_max_linear_child];		\
			struct cds_ft_inode_flag *child_ptr[ft_type_## index ##_max_linear_child]; \
		} linear[1U << ft_type_## index ##_nr_pool_order];				\
	}

#if (CAA_BITS_PER_LONG < 64)

/* 32-bit pointers */
enum {
	ft_type_0_max_child = 3,
	ft_type_1_max_child = 6,
	ft_type_2_max_child = 12,
	ft_type_3_max_child = 25,
	ft_type_4_max_child = 48,
	ft_type_5_max_child = 92,
	ft_type_6_max_child = 256,
	ft_type_7_max_child = 0,	/* NULL */
};

enum {
	ft_type_0_max_linear_child = 3,
	ft_type_1_max_linear_child = 6,
	ft_type_2_max_linear_child = 12,
	ft_type_3_max_linear_child = 25,
	ft_type_4_max_linear_child = 24,
	ft_type_5_max_linear_child = 23,
};

enum {
	ft_type_4_nr_pool_order = 1,
	ft_type_5_nr_pool_order = 2,
};

const struct cds_ft_type ft_types[] = {
	[0] = { .type_class = FT_LINEAR, .min_child = 1, .max_child = ft_type_0_max_child, .max_linear_child = ft_type_0_max_linear_child, .order = 4, .bitmap = FT_NO_BITMAP },
	[1] = { .type_class = FT_LINEAR, .min_child = 3, .max_child = ft_type_1_max_child, .max_linear_child = ft_type_1_max_linear_child, .order = 5, .bitmap = FT_NO_BITMAP },
	[2] = { .type_class = FT_LINEAR, .min_child = 4, .max_child = ft_type_2_max_child, .max_linear_child = ft_type_2_max_linear_child, .order = 6, .bitmap = FT_NO_BITMAP },
	[3] = { .type_class = FT_LINEAR, .min_child = 10, .max_child = ft_type_3_max_child, .max_linear_child = ft_type_3_max_linear_child, .order = 7, .bitmap = FT_NO_BITMAP },

	/* Pools may fill sooner than max_child */
	/* This pool is hardcoded at index 4. See ft_node_ptr(). */
	[FT_POOL_IDX_A] = { .type_class = FT_POOL, .min_child = 20, .max_child = ft_type_4_max_child, .max_linear_child = ft_type_4_max_linear_child, .order = 8, .nr_pool_order = ft_type_4_nr_pool_order, .pool_size_order = 7, .bitmap = FT_NO_BITMAP },
	/* This pool is hardcoded at index 5. See ft_node_ptr(). */
	[FT_POOL_IDX_B] = { .type_class = FT_POOL, .min_child = 45, .max_child = ft_type_5_max_child, .max_linear_child = ft_type_5_max_linear_child, .order = 9, .nr_pool_order = ft_type_5_nr_pool_order, .pool_size_order = 7, .bitmap = FT_BITMAP },

	/*
	 * Upon node removal below min_child, if child pool is filled
	 * beyond capacity, we roll back to pigeon.
	 */
	[6] = { .type_class = FT_PIGEON, .min_child = 83, .max_child = ft_type_6_max_child, .order = 10, .bitmap = FT_BITMAP },

	[7] = { .type_class = FT_NULL, .min_child = 0, .max_child = ft_type_7_max_child, .bitmap = FT_NO_BITMAP },
};

struct cds_ft_inode {
	union {
		/* Linear configuration */
		DECLARE_LINEAR_NODE(0) conf_0;
		DECLARE_LINEAR_NODE(1) conf_1;
		DECLARE_LINEAR_NODE(2) conf_2;
		DECLARE_LINEAR_NODE(3) conf_3;

		/* Pool configuration */
		DECLARE_POOL_NODE(4) conf_4;
		DECLARE_POOL_NODE(5) conf_5;

		/* Pigeon configuration */
		struct {
			struct cds_ft_inode_flag *child[ft_type_6_max_child];
		} conf_6;
		/* data aliasing nodes for computed accesses */
		uint8_t data[sizeof(struct cds_ft_inode_flag *) * ft_type_6_max_child];
	} u;
};
#else /* !(CAA_BITS_PER_LONG < 64) */
/* 64-bit pointers */
enum {
	ft_type_0_max_child = 1,
	ft_type_1_max_child = 3,
	ft_type_2_max_child = 7,
	ft_type_3_max_child = 14,
	ft_type_4_max_child = 28,
	ft_type_5_max_child = 54,
	ft_type_6_max_child = 104,
	ft_type_7_max_child = 256,
	ft_type_8_max_child = 256,
};

enum {
	ft_type_0_max_linear_child = 1,
	ft_type_1_max_linear_child = 3,
	ft_type_2_max_linear_child = 7,
	ft_type_3_max_linear_child = 14,
	ft_type_4_max_linear_child = 28,
	ft_type_5_max_linear_child = 27,
	ft_type_6_max_linear_child = 26,
};

enum {
	ft_type_5_nr_pool_order = 1,
	ft_type_6_nr_pool_order = 2,
};

const struct cds_ft_type ft_types[] = {
	[0] = { .type_class = FT_LINEAR, .min_child = 1, .max_child = ft_type_0_max_child, .max_linear_child = ft_type_0_max_linear_child, .order = 4, .bitmap = FT_NO_BITMAP },
	[1] = { .type_class = FT_LINEAR, .min_child = 1, .max_child = ft_type_1_max_child, .max_linear_child = ft_type_1_max_linear_child, .order = 5, .bitmap = FT_NO_BITMAP },
	[2] = { .type_class = FT_LINEAR, .min_child = 3, .max_child = ft_type_2_max_child, .max_linear_child = ft_type_2_max_linear_child, .order = 6, .bitmap = FT_NO_BITMAP },
	[3] = { .type_class = FT_LINEAR, .min_child = 5, .max_child = ft_type_3_max_child, .max_linear_child = ft_type_3_max_linear_child, .order = 7, .bitmap = FT_NO_BITMAP },
	[4] = { .type_class = FT_LINEAR, .min_child = 10, .max_child = ft_type_4_max_child, .max_linear_child = ft_type_4_max_linear_child, .order = 8, .bitmap = FT_NO_BITMAP },

	/* Pools may fill sooner than max_child. */
	/* This pool is hardcoded at index 5. See ft_node_ptr(). */
	[FT_POOL_IDX_A] = { .type_class = FT_POOL, .min_child = 22, .max_child = ft_type_5_max_child, .max_linear_child = ft_type_5_max_linear_child, .order = 9, .nr_pool_order = ft_type_5_nr_pool_order, .pool_size_order = 8, .bitmap = FT_NO_BITMAP },
	/* This pool is hardcoded at index 6. See ft_node_ptr(). */
	[FT_POOL_IDX_B] = { .type_class = FT_POOL, .min_child = 51, .max_child = ft_type_6_max_child, .max_linear_child = ft_type_6_max_linear_child, .order = 10, .nr_pool_order = ft_type_6_nr_pool_order, .pool_size_order = 8, .bitmap = FT_BITMAP },

	/*
	 * Upon node removal below min_child, if child pool is filled
	 * beyond capacity, we roll back to pigeon.
	 */
	[7] = { .type_class = FT_PIGEON, .min_child = 95, .max_child = ft_type_7_max_child, .order = 11, .bitmap = FT_BITMAP },

	[8] = { .type_class = FT_NULL, .min_child = 0, .max_child = ft_type_8_max_child, .bitmap = FT_NO_BITMAP },
};

struct cds_ft_inode {
	union {
		/* Linear configuration */
		DECLARE_LINEAR_NODE(0) conf_0;
		DECLARE_LINEAR_NODE(1) conf_1;
		DECLARE_LINEAR_NODE(2) conf_2;
		DECLARE_LINEAR_NODE(3) conf_3;
		DECLARE_LINEAR_NODE(4) conf_4;

		/* Pool configuration */
		DECLARE_POOL_NODE(5) conf_5;
		DECLARE_POOL_NODE(6) conf_6;

		/* Pigeon configuration */
		struct {
			struct cds_ft_inode_flag *child[ft_type_7_max_child];
		} conf_7;
		/* data aliasing nodes for computed accesses */
		uint8_t data[sizeof(struct cds_ft_inode_flag *) * ft_type_7_max_child];
	} u;
};
#endif /* !(BITS_PER_LONG < 64) */

static inline __attribute__((unused))
void static_array_size_check(void)
{
	CAA_BUILD_BUG_ON(CAA_ARRAY_SIZE(ft_types) < FT_TYPE_MAX_NR);
}

enum ft_recompact {
	FT_RECOMPACT_ADD_SAME,
	FT_RECOMPACT_ADD_NEXT,
	FT_RECOMPACT_DEL,
};

enum ft_lookup_inequality {
	FT_LOOKUP_GE,
	FT_LOOKUP_LE,
	FT_LOOKUP_GT,
	FT_LOOKUP_LT,
};

enum ft_lookup_limit {
	FT_LOOKUP_LIMIT_NONE,
	FT_LOOKUP_LIMIT_FIRST,
	FT_LOOKUP_LIMIT_LAST,
};

enum ft_direction {
	FT_LEFT,
	FT_RIGHT,
	FT_LEFTMOST,
	FT_RIGHTMOST,
};

/*
 * Fractal Trie iterator object. Can be used to keep backtracking state
 * across API calls. Path use for backtracking requires to keep RCU
 * read-side lock held across calls.
 *
 * The iterator lifetime is bound to the Trie. The Trie must not be
 * destroyed while iterators to that trie exist.
 *
 * The @prefix_len is the length of the key prefix within the key for
 * traversal under a given key prefix. Iterate over the entire Trie when
 * @prefix_len=0.
 */
struct cds_ft_iter {
	struct cds_ft *ft;		/* Point to the associated Fractal Trie. */
	enum cds_ft_status status;	/* Iteration status. */
	bool path_valid;		/* Whether this iterator has a valid path. */
	size_t path_len;		/* Populated path_node array length. */
	size_t key_len;			/* Key length of the current node. */
	size_t prefix_len;		/* Key prefix length. */
	struct cds_ft_node *node;	/* Current external node. */
	/*
	 * Keep a copy of each rcu_dereferenced nodes encountered within
	 * traversal along with their associated keys, thus forming a
	 * path for backtracking.
	 */
	struct cds_ft_inode_flag *path_node[FT_MAX_DEPTH];
	uint8_t key[FT_MAX_KEY_LEN];
};

#define BITMASK_2(a, b)					\
	{						\
		.mask = (1U << (a) | 1U << (b)),	\
		.bit = {				\
			[0] = (a),			\
			[1] = (b),			\
		},					\
	}

struct combination_table {
	uint8_t mask;
	uint8_t bit[2];
};

/*
 * Combination table C(n=8,r=2) = 28.
 */
static
const struct combination_table C_n8_r2[] = {
	BITMASK_2(0, 1), BITMASK_2(0, 2), BITMASK_2(0, 3), BITMASK_2(0, 4), BITMASK_2(0, 5), BITMASK_2(0, 6), BITMASK_2(0, 7),
	BITMASK_2(1, 2), BITMASK_2(1, 3), BITMASK_2(1, 4), BITMASK_2(1, 5), BITMASK_2(1, 6), BITMASK_2(1, 7),
	BITMASK_2(2, 3), BITMASK_2(2, 4), BITMASK_2(2, 5), BITMASK_2(2, 6), BITMASK_2(2, 7),
	BITMASK_2(3, 4), BITMASK_2(3, 5), BITMASK_2(3, 6), BITMASK_2(3, 7),
	BITMASK_2(4, 5), BITMASK_2(4, 6), BITMASK_2(4, 7),
	BITMASK_2(5, 6), BITMASK_2(5, 7),
	BITMASK_2(6, 7)
};

/* return an index within the combination table C(n=8,r) associated to mask. */
static inline
unsigned int mask_to_index_C_n8_r2(uint8_t mask)
{
	unsigned int i;

	assert(__builtin_popcount(mask) == 2);
	for (i = 0; i < CAA_ARRAY_SIZE(C_n8_r2); i++)
		if (C_n8_r2[i].mask == mask)
			return i;
	abort();
}

static inline
void index_to_bits_C_n8_r2(unsigned int index, uint8_t *bits)
{
	assert(index < CAA_ARRAY_SIZE(C_n8_r2));
	bits[0] = C_n8_r2[index].bit[0];
	bits[1] = C_n8_r2[index].bit[1];
}

/*
 * Keep only the requested 2 bits from value, and move them to LSB to
 * form a subclass index.
 */
static inline
unsigned int value_and_bits_to_subclass_index(uint8_t value, const uint8_t *bits)
{

	return (((value >> bits[0]) & 0x1) << 1) | ((value >> bits[1]) & 0x1);
}

static
unsigned long ft_node_pool_1d_bitsel(struct cds_ft_inode_flag *node)
{
	return ((unsigned long) node & FT_POOL_1D_MASK) >> (FT_TYPE_BITS + FT_INTERNAL_BITS);
}

static
void ft_node_pool_2d_index(struct cds_ft_inode_flag *node, unsigned int *index)
{
	*index = ((unsigned long) node & FT_POOL_2D_MASK) >> (FT_TYPE_BITS + FT_INTERNAL_BITS);
}

static
size_t ft_key_len(const struct cds_ft *ft, size_t key_len)
{
	struct cds_ft_group *ft_group = ft->group;

	if (key_len == CDS_FT_LEN_DEFAULT) {
		if (ft_group->key_len == CDS_FT_LEN_VARIABLE)
			return CDS_FT_LEN_ERROR;
		return ft_group->key_len;
	}
	/* Validate that explicit and implicit key lengths match for fixed length Fractal Trie. */
	if (ft_group->key_len != CDS_FT_LEN_VARIABLE && key_len != ft_group->key_len)
		return CDS_FT_LEN_ERROR;
	return key_len;
}

uint64_t cds_ft_key_to_u64(const struct cds_ft *ft, const uint8_t *key,
		size_t _key_len)
{
	size_t key_len = ft_key_len(ft, _key_len);
	union {
		uint64_t v64;
		uint8_t array[8];
	} u;

	assert(key_len <= 8);
	if (key_len > 8)
		return 0;
	u.v64 = 0;
	/* Copy len LSB. */
	memcpy(u.array + sizeof(u.array) - key_len , key, key_len);
	/* Big endian to host endianness. */
	return be64toh(u.v64);
}

void cds_ft_u64_to_key(const struct cds_ft *ft, uint64_t v, uint8_t *key,
		size_t _key_len)
{
	size_t key_len = ft_key_len(ft, _key_len);
	union {
		uint64_t v64;
		uint8_t array[8];
	} u;

	assert(key_len <= 8);
	if (key_len > 8)
		return;
	/* Host endianness to big endian. */
	u.v64 = htobe64(v);
	/* Copy len LSB. */
	memcpy(key, u.array + sizeof(u.array) - key_len , key_len);
}

uint32_t cds_ft_key_to_u32(const struct cds_ft *ft, const uint8_t *key,
		size_t _key_len)
{
	size_t key_len = ft_key_len(ft, _key_len);
	union {
		uint32_t v32;
		uint8_t array[4];
	} u;

	assert(key_len <= 4);
	if (key_len > 4)
		return 0;
	u.v32 = 0;
	/* Copy len LSB. */
	memcpy(u.array + sizeof(u.array) - key_len , key, key_len);
	/* Big endian to host endianness. */
	return be32toh(u.v32);
}

void cds_ft_u32_to_key(const struct cds_ft *ft, uint32_t v, uint8_t *key,
		size_t _key_len)
{
	size_t key_len = ft_key_len(ft, _key_len);
	union {
		uint32_t v32;
		uint8_t array[4];
	} u;

	assert(key_len <= 4);
	if (key_len > 4)
		return;
	/* Host endianness to big endian. */
	u.v32 = htobe32(v);
	/* Copy len LSB. */
	memcpy(key, u.array + sizeof(u.array) - key_len , key_len);
}

/*
 * Signed integer key helpers.
 *
 * Signed integers need a sign-bit flip (XOR with the MSB of the
 * key-width value) so that the big-endian byte ordering used by the
 * Fractal Trie preserves the natural signed ordering.
 *
 * When the key is the full width of the integer type (e.g. 8 bytes
 * for int64_t), the mapping is:
 *
 *   INT64_MIN  -> 0x0000000000000000   (sorts first)
 *   -1         -> 0x7FFFFFFFFFFFFFFF
 *    0         -> 0x8000000000000000
 *   INT64_MAX  -> 0xFFFFFFFFFFFFFFFF   (sorts last)
 *
 * The same principle applies to 32-bit signed integers.
 *
 * When the key is narrower than the integer type (e.g. a 2-byte key
 * representing a signed 16-bit range within a 64-bit integer), the
 * sign bit is at position (key_len * 8 - 1), not at the MSB of the
 * full integer.  The key-to-integer direction therefore sign-extends
 * from the key's MSB to fill the integer.
 */

int64_t cds_ft_key_to_s64(const struct cds_ft *ft, const uint8_t *key,
		size_t _key_len)
{
	size_t key_len = ft_key_len(ft, _key_len);
	unsigned int shift;
	uint64_t u;

	assert(key_len <= 8);
	if (key_len == 0 || key_len > 8)
		return 0;
	u = cds_ft_key_to_u64(ft, key, _key_len);
	shift = key_len * 8;
	/* Flip sign bit (MSB of key-width value) to recover signed encoding. */
	u ^= 1ULL << (shift - 1);
	/* Sign-extend from key width to 64 bits. */
	if (shift < 64) {
		uint64_t sign_bit = 1ULL << (shift - 1);

		if (u & sign_bit)
			u |= ~((1ULL << shift) - 1);
	}
	return (int64_t) u;
}

void cds_ft_s64_to_key(const struct cds_ft *ft, int64_t v, uint8_t *key,
		size_t _key_len)
{
	size_t key_len = ft_key_len(ft, _key_len);
	unsigned int shift;

	assert(key_len <= 8);
	if (key_len == 0 || key_len > 8)
		return;
	shift = key_len * 8;
	/* Flip sign bit so that negative values sort before positive. */
	cds_ft_u64_to_key(ft, (uint64_t) v ^ ( 1ULL << (shift - 1)), key, _key_len);
}

int32_t cds_ft_key_to_s32(const struct cds_ft *ft, const uint8_t *key,
		size_t _key_len)
{
	size_t key_len = ft_key_len(ft, _key_len);
	unsigned int shift;
	uint32_t u;

	assert(key_len <= 4);
	if (key_len == 0 || key_len > 4)
		return 0;
	u = cds_ft_key_to_u32(ft, key, _key_len);
	shift = key_len * 8;
	/* Flip sign bit (MSB of key-width value) to recover signed encoding. */
	u ^= 1U << (shift - 1);
	/* Sign-extend from key width to 32 bits. */
	if (shift < 32) {
		uint32_t sign_bit = 1U << (shift - 1);

		if (u & sign_bit)
			u |= ~((1U << shift) - 1);
	}
	return (int32_t) u;
}

void cds_ft_s32_to_key(const struct cds_ft *ft, int32_t v, uint8_t *key,
		size_t _key_len)
{
	size_t key_len = ft_key_len(ft, _key_len);
	unsigned int shift;

	assert(key_len <= 4);
	if (key_len == 0 || key_len > 4)
		return;
	shift = key_len * 8;
	/* Flip sign bit so that negative values sort before positive. */
	cds_ft_u32_to_key(ft, (uint32_t) v ^ (1U << (shift - 1)), key, _key_len);
}

static
uint8_t key_to_ordinal(const struct cds_ft *ft, uint8_t key)
{
	if (caa_likely(ft->group->key_map.identity))
		return key;
	return ft->group->key_map.key_to_ordinal[key];
}

static
uint8_t ordinal_to_key(const struct cds_ft *ft, uint8_t ordinal)
{
	if (caa_likely(ft->group->key_map.identity))
		return ordinal;
	return ft->group->key_map.ordinal_to_key[ordinal];
}

static
struct cds_ft_inode *_ft_node_mask_ptr(struct cds_ft_inode_flag *node)
{
	return (struct cds_ft_inode *) (((unsigned long) node) & FT_PTR_MASK);
}

static
unsigned long ft_node_type(struct cds_ft_inode_flag *node)
{
	unsigned long type;

	if (_ft_node_mask_ptr(node) == NULL) {
		return NODE_INDEX_NULL;
	}
	type = (unsigned int) (((unsigned long) node & FT_TYPE_MASK) >> FT_INTERNAL_BITS);
	assert(type < (1UL << FT_TYPE_BITS));
	return type;
}

static
bool ft_node_internal(struct cds_ft_inode_flag *node)
{
	return (unsigned long) node & FT_INTERNAL_MASK;
}

static
bool valid_external_node(struct cds_ft_node *node)
{
	return node != NULL && !ft_node_internal((struct cds_ft_inode_flag *) node);
}

static
bool valid_key_len(struct cds_ft *ft, size_t key_len)
{
	size_t max_key_len = ft->group->max_key_len;

	assert(max_key_len != CDS_FT_MAX_LEN_UNLIMITED);
	if (key_len == CDS_FT_LEN_ERROR || key_len > max_key_len)
		return false;
	return true;
}

static
struct cds_ft_inode *alloc_cds_ft_node(struct cds_ft *ft,
		const struct cds_ft_type *ft_type,
		struct cds_ft_metadata **_metadata)
{
	struct cds_ft_metadata *metadata;
	void *p;

	metadata = cds_ft_alloc_item(ft, ft_type->order, ft_type->bitmap);
	if (!metadata) {
		return NULL;
	}
	p = cds_ft_metadata_to_item(metadata);
	if (ft_debug_counters())
		uatomic_inc(&ft->nr_nodes_allocated);
	*_metadata = metadata;
	return p;
}

static
void free_cds_ft_node(struct cds_ft *ft, struct cds_ft_inode *node)
{
	struct cds_ft_metadata *metadata = cds_ft_item_to_metadata(node);

	cds_ft_free_item(metadata);
	if (ft_debug_counters() && node)
		uatomic_inc(&ft->nr_nodes_freed);
}

#define __FT_ALIGN_MASK(v, mask)	(((v) + (mask)) & ~(mask))
#define FT_ALIGN(v, align)		__FT_ALIGN_MASK(v, (typeof(v)) (align) - 1)
#define __FT_FLOOR_MASK(v, mask)	((v) & ~(mask))
#define FT_FLOOR(v, align)		__FT_FLOOR_MASK(v, (typeof(v)) (align) - 1)

static inline_lookup
uint8_t *align_ptr_size(uint8_t *ptr)
{
	return (uint8_t *) FT_ALIGN((unsigned long) ptr, sizeof(void *));
}

static inline_lookup
uint8_t ft_linear_node_get_nr_child(const struct cds_ft_type *type,
		struct cds_ft_inode *node)
{
	assert(type->type_class == FT_LINEAR || type->type_class == FT_POOL);
	/* load-acquire orders nr_child load before values and pointers */
	return uatomic_load(&node->u.data[0], CMM_ACQUIRE);
}

/*
 * The order in which values and pointers are does does not matter: if
 * a value is missing, we return NULL. If a value is there, but its
 * associated pointers is still NULL, we return NULL too.
 */

#if defined(FEATURE_SIMD_LOOKUP) && defined(__AVX2__)

static inline_lookup
struct cds_ft_inode_flag *ft_linear_node_get_nth(const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		struct cds_ft_inode_flag ***node_flag_ptr,
		uint8_t n)
{
	uint8_t nr_child = ft_linear_node_get_nr_child(type, node);
	uint8_t *data = &node->u.data[0]; /* data[0] is nr_child */
	unsigned int phys_idx;
	uint32_t mask;

	/* Broadcast target byte into a 256-bit vector. */
	__m256i target = _mm256_set1_epi8(n);

	/* Load and compare first 32 bytes (includes count). */
	__m256i chunk0 = _mm256_loadu_si256((__m256i*)data);
	mask = _mm256_movemask_epi8(_mm256_cmpeq_epi8(chunk0, target));

	mask &= ~1U; /* Ignore the count byte at index 0. */

	if (mask) {
		phys_idx = __builtin_ctz(mask);
		if (caa_likely(phys_idx <= nr_child))
			goto found;
	}

	/* Load and compare next 32 bytes (bytes 32-63). */
	if (nr_child >= 32) {
		__m256i chunk1 = _mm256_loadu_si256((__m256i*)(data + 32));
		mask = _mm256_movemask_epi8(_mm256_cmpeq_epi8(chunk1, target));

		if (mask) {
			phys_idx = 32 + __builtin_ctz(mask);
			if (phys_idx <= nr_child)
				goto found;
		}
	}

	if (caa_unlikely(node_flag_ptr))
		*node_flag_ptr = NULL;
	return NULL;

found:
	{
		/* logical_idx = phys_idx - 1 (because data[0] is nr_child). */
		unsigned int i = phys_idx - 1;
		struct cds_ft_inode_flag **pointers = (struct cds_ft_inode_flag **)
			align_ptr_size(&node->u.data[1] + type->max_linear_child);

		if (caa_unlikely(node_flag_ptr))
			*node_flag_ptr = &pointers[i];
		return rcu_dereference(pointers[i]);
	}
}

#elif defined(FEATURE_SIMD_LOOKUP)
/*
 * Define a 32-byte vector of unsigned bytes.
 * GCC will handle the mapping to hardware registers.
 */
typedef uint8_t v32u8 __attribute__ ((vector_size (32)));

static inline uint32_t get_bitmask(v32u8 v) {
#if defined(__AVX2__)
	/* If compiled with -mavx2, use the 256-bit intrinsic. */
	return _mm256_movemask_epi8((__m256i)v);
#elif defined(__SSE2__)
	/*
	 * If only -msse2 or -msse4.2, split the 32-byte generic vector
	 * into two 16-byte moves.
	 */
	__m128i low = _mm_loadu_si128((__m128i*)&v);
	__m128i high = _mm_loadu_si128((__m128i*)((uint8_t*)&v + 16));
	return _mm_movemask_epi8(low) | (_mm_movemask_epi8(high) << 16);
#else
	/* Fallback for non-x86 (ARM/NEON etc.) */
	uint32_t m = 0;

	for (int i = 0; i<32; i++) {
		if (((uint8_t*)&v)[i]) {
			m |= (1U << i);
		}
	}
	return m;
#endif
}

static struct cds_ft_inode_flag *ft_linear_node_get_nth(
		const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		struct cds_ft_inode_flag ***node_flag_ptr,
		uint8_t n)
{
	uint8_t nr_child = ft_linear_node_get_nr_child(type, node);
	uint8_t *data = &node->u.data[0]; /* data[0] is nr_child */
	unsigned int phys_idx;

	/*
	 * Broadcast target 'n' into a vector.
	 * GCC optimizes this to a single broadcast instruction.
	 */
	v32u8 target_v = { n,n,n,n,n,n,n,n,n,n,n,n,n,n,n,n,
			   n,n,n,n,n,n,n,n,n,n,n,n,n,n,n,n };

	/* --- BLOCK 1: Bytes 0-31 (Includes Count) --- */
	v32u8 chunk0;
	__builtin_memcpy(&chunk0, data, 32);

	/* Vector comparison: results in 0xFF for match, 0x00 for no match */
	v32u8 res0 = (chunk0 == target_v);

	/*
	 * Convert vector result to a bitmask.
	 * Note: On x86, the compiler will use VPMOVMSKB.
	 */
	uint32_t mask0 = get_bitmask(res0);

	mask0 &= ~1U; /* Force ignore of the nr_child byte at index 0. */
	if (mask0) {
		phys_idx = __builtin_ctz(mask0);
		if (phys_idx <= nr_child)
			goto found;
	}

	/* --- BLOCK 2: Bytes 32-63 --- */
	if (nr_child >= 32) {
		v32u8 chunk1;
		__builtin_memcpy(&chunk1, data + 32, 32);
		v32u8 res1 = (chunk1 == target_v);

		uint32_t mask1 = get_bitmask(res1);
		if (mask1) {
			phys_idx = 32 + __builtin_ctz(mask1);
			if (phys_idx <= nr_child)
				goto found;
		}
	}

	if (caa_unlikely(node_flag_ptr))
		*node_flag_ptr = NULL;
	return NULL;

found:
	{
		unsigned int i = phys_idx - 1; /* Convert to logical 0-index */
		struct cds_ft_inode_flag **pointers = (struct cds_ft_inode_flag **)
			align_ptr_size(&node->u.data[1] + type->max_linear_child);

		if (caa_unlikely(node_flag_ptr)) *node_flag_ptr = &pointers[i];
		return rcu_dereference(pointers[i]);
	}
}

#elif defined(FEATURE_SWAR_LOOKUP)

/* Generate a mask of 0x01 bytes for the current word size. */
#define L_ONES (-1UL / 255)
/* Generate a mask of 0x80 bytes for the current word size. */
#define L_HIGHS (L_ONES * 0x80)

/*
 * This function can load beyond nr_child, but always compare with the
 * nr_child limit if it finds a match. Loading a full word beyond
 * nr_child is OK because the node is word-aligned and its size is a
 * multiple of the word-size.
 */
static inline_lookup
struct cds_ft_inode_flag *ft_linear_node_get_nth(const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		struct cds_ft_inode_flag ***node_flag_ptr,
		uint8_t n)
{
	/* node->u.data is always aligned on sizeof(unsigned long) */
	unsigned long *data_words = (unsigned long *)node->u.data,
		first_word = data_words[0], mask = n * L_ONES, xor_res, has_zero;
	uint8_t nr_child;
	unsigned int i;

	/* Extract nr_child from the first byte (architecture dependent) */
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
	nr_child = (uint8_t)(first_word & 0xFFUL);
#else
	nr_child = (uint8_t)(first_word >> ((sizeof(unsigned long) - 1) * 8));
#endif
	assert(type->type_class == FT_LINEAR || type->type_class == FT_POOL);
	assert(type->type_class != FT_LINEAR || nr_child == 0 || nr_child >= type->min_child);
	assert(nr_child <= type->max_linear_child && nr_child != 255);

	/* Empty node (root with 0 children). */
	if (caa_unlikely(nr_child == 0)) {
		if (caa_unlikely(node_flag_ptr))
			*node_flag_ptr = NULL;
		return NULL;
	}

	/*
	 * Prepare the poisoned mask for the first word.
	 * nr_child != 255, use that value as poison.
	 * Search the first word using the poisoned mask.
	 */
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
	/* For Little-Endian, nr_child is at the low end of the word. */
	xor_res = first_word ^ (mask | 0xFFUL);
	has_zero = (xor_res - L_ONES) & ~xor_res & L_HIGHS;
	if (has_zero) {
		i = (__builtin_ctzl(has_zero) >> 3) - 1; // -1 because index 0 is nr_child
		if (i < nr_child)
			goto found;
	}
#else
	/* For Big-Endian, nr_child is at the high end of the word. */
	xor_res = first_word ^ (mask | (0xFFUL << ((sizeof(unsigned long) - 1) * 8)));
	has_zero = (xor_res - L_ONES) & ~xor_res & L_HIGHS;
	if (has_zero) {
		i = (__builtin_clzl(has_zero) >> 3) - 1;
		if (i < nr_child)
			goto found;
	}
#endif

	/* Search subsequent words using the standard mask. */
	for (unsigned int w = 1; w * sizeof(unsigned long) <= type->max_linear_child; w++) {
		xor_res = data_words[w] ^ mask;
		has_zero = (xor_res - L_ONES) & ~xor_res & L_HIGHS;

		if (has_zero) {
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
			i = (w * sizeof(unsigned long)) + (__builtin_ctzl(has_zero) >> 3) - 1;
#else
			i = (w * sizeof(unsigned long)) + (__builtin_clzl(has_zero) >> 3) - 1;
#endif
			if (i < nr_child)
				goto found;
		}
	}
	if (caa_unlikely(node_flag_ptr))
		*node_flag_ptr = NULL;
	return NULL;

found:
	{
		uint8_t *values = &node->u.data[1];
		struct cds_ft_inode_flag **pointers = (struct cds_ft_inode_flag **) align_ptr_size(&values[type->max_linear_child]);
		struct cds_ft_inode_flag *ptr = rcu_dereference(pointers[i]);

		if (caa_unlikely(node_flag_ptr))
			*node_flag_ptr = &pointers[i];
		return ptr;
	}
}
#else
static inline_lookup
struct cds_ft_inode_flag *ft_linear_node_get_nth(const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		struct cds_ft_inode_flag ***node_flag_ptr,
		uint8_t n)
{
	uint8_t nr_child;
	uint8_t *values;
	struct cds_ft_inode_flag **pointers;
	struct cds_ft_inode_flag *ptr;
	unsigned int i;

	assert(type->type_class == FT_LINEAR || type->type_class == FT_POOL);

	nr_child = ft_linear_node_get_nr_child(type, node);
	assert(nr_child <= type->max_linear_child);
	assert(type->type_class != FT_LINEAR || nr_child == 0 || nr_child >= type->min_child);

	values = &node->u.data[1];
	for (i = 0; i < nr_child; i++) {
		if (uatomic_load(&values[i], CMM_RELAXED) == n)
			break;
	}
	if (i >= nr_child) {
		if (caa_unlikely(node_flag_ptr))
			*node_flag_ptr = NULL;
		return NULL;
	}
	pointers = (struct cds_ft_inode_flag **) align_ptr_size(&values[type->max_linear_child]);
	ptr = rcu_dereference(pointers[i]);
	if (caa_unlikely(node_flag_ptr))
		*node_flag_ptr = &pointers[i];
	return ptr;
}
#endif

static inline_lookup
struct cds_ft_inode_flag *ft_linear_node_get_direction(const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		int n, uint8_t *result_key,
		enum ft_direction dir)
{
	uint8_t nr_child;
	uint8_t *values;
	struct cds_ft_inode_flag **pointers;
	struct cds_ft_inode_flag *ptr, *match_ptr = NULL;
	unsigned int i;
	int match_v;

	assert(type->type_class == FT_LINEAR || type->type_class == FT_POOL);
	assert(dir == FT_LEFT || dir == FT_RIGHT);

	if (dir == FT_LEFT) {
		match_v = -1;
	} else {
		match_v = FT_ENTRY_PER_NODE;
	}

	nr_child = ft_linear_node_get_nr_child(type, node);
	cmm_smp_rmb();	/* read nr_child before values and pointers */
	assert(nr_child <= type->max_linear_child);
	assert(type->type_class != FT_LINEAR || nr_child == 0 || nr_child >= type->min_child);

	values = &node->u.data[1];
	pointers = (struct cds_ft_inode_flag **) align_ptr_size(&values[type->max_linear_child]);
	for (i = 0; i < nr_child; i++) {
		unsigned int v;

		v = uatomic_load(&values[i], CMM_RELAXED);
		ptr = rcu_dereference(pointers[i]);
		if (!ptr)
			continue;
		if (dir == FT_LEFT) {
			if ((int) v < n && (int) v > match_v) {
				match_v = v;
				match_ptr = ptr;
				/* Found value immediately left of n. */
				if (match_v == n - 1)
					break;
			}
		} else {
			if ((int) v > n && (int) v < match_v) {
				match_v = v;
				match_ptr = ptr;
				/* Found value immediately right of n. */
				if (match_v == n + 1)
					break;
			}
		}
	}

	if (!match_ptr) {
		return NULL;
	}
	assert(match_v >= 0 && match_v < FT_ENTRY_PER_NODE);

	*result_key = (uint8_t) match_v;
	return match_ptr;
}

static inline_lookup
void ft_linear_node_get_ith_pos(const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		uint8_t i,
		uint8_t *v,
		struct cds_ft_inode_flag **iter)
{
	uint8_t *values;
	struct cds_ft_inode_flag **pointers;

	assert(type->type_class == FT_LINEAR || type->type_class == FT_POOL);
	assert(i < ft_linear_node_get_nr_child(type, node));

	values = &node->u.data[1];
	*v = values[i];
	pointers = (struct cds_ft_inode_flag **) align_ptr_size(&values[type->max_linear_child]);
	*iter = rcu_dereference(pointers[i]);
}

static inline_lookup
struct cds_ft_inode_flag *ft_pool_node_get_nth(const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		struct cds_ft_inode_flag *node_flag,
		struct cds_ft_inode_flag ***node_flag_ptr,
		uint8_t n)
{
	struct cds_ft_inode *linear;

	assert(type->type_class == FT_POOL);

	switch (type->nr_pool_order) {
	case 1:
	{
		unsigned long bitsel, index;

		bitsel = ft_node_pool_1d_bitsel(node_flag);
		assert(bitsel < CHAR_BIT);
		index = ((unsigned long) n >> bitsel) & 0x1;
		linear = (struct cds_ft_inode *) &node->u.data[index << type->pool_size_order];
		break;
	}
	case 2:
	{
		unsigned int C_n8_r2_index, subclass_index;
		uint8_t bits[2];

		ft_node_pool_2d_index(node_flag, &C_n8_r2_index);
		index_to_bits_C_n8_r2(C_n8_r2_index, bits);
		subclass_index = value_and_bits_to_subclass_index(n, bits);
		linear = (struct cds_ft_inode *) &node->u.data[subclass_index << type->pool_size_order];
		break;
	}
	default:
		linear = NULL;
		assert(0);
	}
	return ft_linear_node_get_nth(type, linear, node_flag_ptr, n);
}

static inline_lookup
struct cds_ft_inode *ft_pool_node_get_ith_pool(const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		uint8_t i)
{
	assert(type->type_class == FT_POOL);
	return (struct cds_ft_inode *)
		&node->u.data[(unsigned int) i << type->pool_size_order];
}

static inline_lookup
struct cds_ft_inode_flag *ft_pool_node_get_direction(const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		struct cds_ft_inode_flag *node_flag __attribute__((unused)),
		int n, uint8_t *result_key,
		enum ft_direction dir)
{
	unsigned int pool_nr;
	int match_v;
	struct cds_ft_inode_flag *match_node_flag = NULL;

	assert(type->type_class == FT_POOL);
	assert(dir == FT_LEFT || dir == FT_RIGHT);

#ifdef FEATURE_USE_BITMAP_SCAN
	if (type->bitmap) {
		struct cds_ft_bitmap *bitmap = cds_ft_item_to_bitmap(node, type->order);
retry:
		if (dir == FT_LEFT)
			match_v = cds_find_prev_bit(bitmap->bitmap, FT_ENTRY_PER_NODE, n - 1);
		else
			match_v = cds_find_next_bit(bitmap->bitmap, FT_ENTRY_PER_NODE, n + 1);
		if (match_v >= 0) {
			match_node_flag = ft_pool_node_get_nth(type, node, node_flag, NULL, (uint8_t) match_v);
			/*
			 * The source of truth is the pointer load from
			 * get_nth. Continue the bitmap scan if the node
			 * is not found.
			 */
			if (!match_node_flag) {
				n = match_v;
				goto retry;
			}
		}
		goto end;
	}
#endif

	if (dir == FT_LEFT) {
		match_v = -1;
	} else {
		match_v = FT_ENTRY_PER_NODE;
	}

	for (pool_nr = 0; pool_nr < (1U << type->nr_pool_order); pool_nr++) {
		struct cds_ft_inode *pool =
			ft_pool_node_get_ith_pool(type,
				node, pool_nr);
		uint8_t nr_child =
			ft_linear_node_get_nr_child(type, pool);
		unsigned int j;

		for (j = 0; j < nr_child; j++) {
			struct cds_ft_inode_flag *iter;
			uint8_t v;

			ft_linear_node_get_ith_pos(type, pool,
					j, &v, &iter);
			if (!iter)
				continue;
			if (dir == FT_LEFT) {
				if ((int) v < n && (int) v > match_v) {
					match_v = v;
					match_node_flag = iter;
					/* Found value immediately left of n. */
					if (match_v == n - 1)
						goto end;
				}
			} else {
				if ((int) v > n && (int) v < match_v) {
					match_v = v;
					match_node_flag = iter;
					/* Found value immediately right of n. */
					if (match_v == n + 1)
						goto end;
				}
			}
		}
	}
end:
	if (match_node_flag)
		*result_key = (uint8_t) match_v;
	return match_node_flag;
}

static inline_lookup
struct cds_ft_inode_flag *ft_pigeon_node_get_nth(const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		struct cds_ft_inode_flag ***node_flag_ptr,
		uint8_t n)
{
	struct cds_ft_inode_flag **child_node_flag_ptr;
	struct cds_ft_inode_flag *child_node_flag;

	assert(type->type_class == FT_PIGEON);
	child_node_flag_ptr = &((struct cds_ft_inode_flag **) node->u.data)[n];
	child_node_flag = rcu_dereference(*child_node_flag_ptr);
	//dbg_printf("ft_pigeon_node_get_nth child_node_flag_ptr %p\n",
	//	child_node_flag_ptr);
	if (caa_unlikely(node_flag_ptr))
		*node_flag_ptr = child_node_flag_ptr;
	return child_node_flag;
}

static inline_lookup
struct cds_ft_inode_flag *ft_pigeon_node_get_direction(const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		int n, uint8_t *result_key,
		enum ft_direction dir)
{
	struct cds_ft_inode_flag **child_node_flag_ptr;
	struct cds_ft_inode_flag *child_node_flag;
#ifdef FEATURE_USE_BITMAP_SCAN
	struct cds_ft_bitmap *bitmap = cds_ft_item_to_bitmap(node, type->order);
#endif
	int i;

	assert(type->type_class == FT_PIGEON);
	assert(dir == FT_LEFT || dir == FT_RIGHT);

#ifdef FEATURE_USE_BITMAP_SCAN
retry:
	if (dir == FT_LEFT)
		i = cds_find_prev_bit(bitmap->bitmap, FT_ENTRY_PER_NODE, n - 1);
	else
		i = cds_find_next_bit(bitmap->bitmap, FT_ENTRY_PER_NODE, n + 1);
	if (i >= 0) {
		child_node_flag_ptr = &((struct cds_ft_inode_flag **) node->u.data)[i];
		child_node_flag = rcu_dereference(*child_node_flag_ptr);
		if (!child_node_flag) {
			/*
			 * The source of truth is the pointer load.
			 * Continue the bitmap scan if the node is
			 * not found.
			 */
			n = i;
			goto retry;
		}
		dbg_printf("ft_pigeon_node_get child_node_flag %p\n", child_node_flag);
		*result_key = (uint8_t) i;
		return child_node_flag;
	}
#else
	if (dir == FT_LEFT) {
		/* n - 1 is first value left of n */
		for (i = n - 1; i >= 0; i--) {
			child_node_flag_ptr = &((struct cds_ft_inode_flag **) node->u.data)[i];
			child_node_flag = rcu_dereference(*child_node_flag_ptr);
			if (child_node_flag) {
				dbg_printf("ft_pigeon_node_get_left child_node_flag %p\n",
					child_node_flag);
				*result_key = (uint8_t) i;
				return child_node_flag;
			}
		}
	} else {
		/* n + 1 is first value right of n */
		for (i = n + 1; i < FT_ENTRY_PER_NODE; i++) {
			child_node_flag_ptr = &((struct cds_ft_inode_flag **) node->u.data)[i];
			child_node_flag = rcu_dereference(*child_node_flag_ptr);
			if (child_node_flag) {
				dbg_printf("ft_pigeon_node_get_right child_node_flag %p\n",
					child_node_flag);
				*result_key = (uint8_t) i;
				return child_node_flag;
			}
		}
	}
#endif
	return NULL;
}

static inline_lookup
struct cds_ft_inode_flag *ft_pigeon_node_get_ith_pos(const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		uint8_t i)
{
	return ft_pigeon_node_get_nth(type, node, NULL, i);
}

/*
 * ft_node_get_nth: get nth item from a node.
 * node_flag is already rcu_dereference'd.
 */
static inline_lookup
struct cds_ft_inode_flag *ft_node_get_nth(struct cds_ft_inode_flag *node_flag,
		struct cds_ft_inode_flag ***node_flag_ptr,
		uint8_t n)
{
	unsigned int type_index;
	struct cds_ft_inode *node;
	const struct cds_ft_type *type;

	node = ft_node_ptr(node_flag);
	assert(node != NULL);
	type_index = ft_node_type(node_flag);
	type = &ft_types[type_index];

	switch (type->type_class) {
	case FT_LINEAR:
		return ft_linear_node_get_nth(type, node,
				node_flag_ptr, n);
	case FT_POOL:
		return ft_pool_node_get_nth(type, node, node_flag,
				node_flag_ptr, n);
	case FT_PIGEON:
		return ft_pigeon_node_get_nth(type, node,
				node_flag_ptr, n);
	default:
		assert(0);
		return (void *) -1UL;
	}
}

static inline_lookup
struct cds_ft_inode_flag *ft_node_get_direction(struct cds_ft_inode_flag *node_flag,
		int n, uint8_t *result_key,
		enum ft_direction dir)
{
	unsigned int type_index;
	struct cds_ft_inode *node;
	const struct cds_ft_type *type;

	node = ft_node_ptr(node_flag);
	assert(node != NULL);
	type_index = ft_node_type(node_flag);
	type = &ft_types[type_index];

	switch (type->type_class) {
	case FT_LINEAR:
		return ft_linear_node_get_direction(type, node, n, result_key, dir);
	case FT_POOL:
		return ft_pool_node_get_direction(type, node, node_flag, n, result_key, dir);
	case FT_PIGEON:
		return ft_pigeon_node_get_direction(type, node, n, result_key, dir);
	default:
		assert(0);
		return (void *) -1UL;
	}
}

static inline_lookup
struct cds_ft_inode_flag *ft_node_get_leftright(struct cds_ft_inode_flag *node_flag,
		unsigned int n, uint8_t *result_key,
		enum ft_direction dir)
{
	return ft_node_get_direction(node_flag, n, result_key, dir);
}

static inline_lookup
struct cds_ft_inode_flag *ft_node_get_minmax(struct cds_ft_inode_flag *node_flag,
		uint8_t *result_key,
		enum ft_direction dir,
		bool is_root)
{
	struct cds_ft_inode_flag *ret;

	switch (dir) {
	case FT_LEFTMOST:
		ret = ft_node_get_direction(node_flag,
				-1, result_key, FT_RIGHT);
		break;
	case FT_RIGHTMOST:
		ret = ft_node_get_direction(node_flag,
				FT_ENTRY_PER_NODE, result_key, FT_LEFT);
		break;
	default:
		assert(0);
	}
	/*
	 * attach/detach semantic guarantees that ft_node_get_minmax
	 * cannot return NULL except when called on an empty root node.
	 */
	assert(is_root || ft_node_ptr(ret));
	return ret;
}

static
int ft_linear_node_set_nth(const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		struct cds_ft_metadata *metadata,
		uint8_t n,
		struct cds_ft_inode_flag *child_node_flag,
		bool *_replace_old_ptr)
{
	uint8_t nr_child;
	uint8_t *values, *nr_child_ptr;
	struct cds_ft_inode_flag **pointers;
	unsigned int i, unused = 0;
	bool replace_old_ptr = false;

	assert(type->type_class == FT_LINEAR || type->type_class == FT_POOL);

	nr_child_ptr = &node->u.data[0];
	dbg_printf("linear set nth: n %u, nr_child_ptr %p\n",
		(unsigned int) n, nr_child_ptr);
	nr_child = *nr_child_ptr;
	assert(nr_child <= type->max_linear_child);

	values = &node->u.data[1];
	pointers = (struct cds_ft_inode_flag **) align_ptr_size(&values[type->max_linear_child]);
	/* Check if node value is already populated */
	for (i = 0; i < nr_child; i++) {
		if (values[i] == n) {
			if (pointers[i])
				replace_old_ptr = true;
			break;
		} else {
			if (!pointers[i])
				unused++;
		}
	}
	if (i == nr_child && nr_child >= type->max_linear_child) {
		if (unused)
			return -ERANGE;	/* recompact node */
		else
			return -ENOSPC;	/* No space left in this node type */
	}

	/* If we expanded the nr_child, increment it */
	if (i == nr_child) {
		assert(pointers[i] == NULL);
		uatomic_store(&pointers[i], child_node_flag, CMM_RELAXED);
		uatomic_store(&values[nr_child], n, CMM_RELAXED);
		/* store-release: write pointer and value before nr_child */
		uatomic_store(nr_child_ptr, nr_child + 1, CMM_RELEASE);
	} else {
		/* Replacing a NULL or external node pointer. */
		rcu_assign_pointer(pointers[i], child_node_flag);
	}
	if (!replace_old_ptr)
		metadata->nr_child++;
	dbg_printf("linear set nth: %u child, metadata: %u child, for node %p\n",
		(unsigned int) uatomic_load(nr_child_ptr, CMM_RELAXED),
		(unsigned int) metadata->nr_child,
		node);
	if (_replace_old_ptr)
		*_replace_old_ptr = replace_old_ptr;
	return 0;
}

static
int ft_pool_node_set_nth(const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		struct cds_ft_inode_flag *node_flag,
		struct cds_ft_metadata *metadata,
		uint8_t n,
		struct cds_ft_inode_flag *child_node_flag)
{
	struct cds_ft_inode *linear;
	bool replace_old_ptr = false;
	int ret;

	assert(type->type_class == FT_POOL);

	switch (type->nr_pool_order) {
	case 1:
	{
		unsigned long bitsel, index;

		bitsel = ft_node_pool_1d_bitsel(node_flag);
		assert(bitsel < CHAR_BIT);
		index = ((unsigned long) n >> bitsel) & 0x1;
		linear = (struct cds_ft_inode *) &node->u.data[index << type->pool_size_order];
		break;
	}
	case 2:
	{
		unsigned int C_n8_r2_index, subclass_index;
		uint8_t bits[2];

		ft_node_pool_2d_index(node_flag, &C_n8_r2_index);
		index_to_bits_C_n8_r2(C_n8_r2_index, bits);
		subclass_index = value_and_bits_to_subclass_index(n, bits);
		linear = (struct cds_ft_inode *) &node->u.data[subclass_index << type->pool_size_order];
		break;
	}
	default:
		linear = NULL;
		assert(0);
	}

	ret = ft_linear_node_set_nth(type, linear, metadata, n, child_node_flag, &replace_old_ptr);
#ifdef FEATURE_USE_BITMAP_SCAN
	if (ret == 0 && !replace_old_ptr && type->bitmap) {
		struct cds_ft_bitmap *bitmap = cds_ft_item_to_bitmap(node, type->order);

		/* Set n in bitmap. */
		cds_set_bit_relaxed(bitmap->bitmap, n);
	}
#endif
	return ret;
}

static
int ft_pigeon_node_set_nth(const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		struct cds_ft_metadata *metadata,
		uint8_t n,
		struct cds_ft_inode_flag *child_node_flag)
{
	struct cds_ft_inode_flag **ptr;
	bool replace_old_ptr = false;

	assert(type->type_class == FT_PIGEON);
	ptr = &((struct cds_ft_inode_flag **) node->u.data)[n];
	if (*ptr)
		replace_old_ptr = true;
	rcu_assign_pointer(*ptr, child_node_flag);
	if (!replace_old_ptr) {
#ifdef FEATURE_USE_BITMAP_SCAN
		struct cds_ft_bitmap *bitmap = cds_ft_item_to_bitmap(node, type->order);

		/* Set n in bitmap. */
		cds_set_bit_relaxed(bitmap->bitmap, n);
#endif
		metadata->nr_child++;
	}
	return 0;
}

/*
 * _ft_node_set_nth: set nth item within a node. Return an error
 * (negative error value) if it is already there.
 */
static
int _ft_node_set_nth(const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		struct cds_ft_inode_flag *node_flag,
		struct cds_ft_metadata *metadata,
		uint8_t n,
		struct cds_ft_inode_flag *child_node_flag)
{
	switch (type->type_class) {
	case FT_LINEAR:
		return ft_linear_node_set_nth(type, node, metadata, n, child_node_flag, NULL);
	case FT_POOL:
		return ft_pool_node_set_nth(type, node, node_flag, metadata, n, child_node_flag);
	case FT_PIGEON:
		return ft_pigeon_node_set_nth(type, node, metadata, n, child_node_flag);
	case FT_NULL:
		return -ENOSPC;
	default:
		assert(0);
		return -EINVAL;
	}

	return 0;
}

static
int ft_linear_node_replace_ptr(const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		struct cds_ft_metadata *metadata,
		struct cds_ft_inode_flag **node_flag_ptr,
		struct cds_ft_inode_flag *newptr)
{
	uint8_t nr_child;
	uint8_t *nr_child_ptr;

	assert(type->type_class == FT_LINEAR || type->type_class == FT_POOL);

	nr_child_ptr = &node->u.data[0];
	nr_child = *nr_child_ptr;
	assert(nr_child <= type->max_linear_child);

	if (type->type_class == FT_LINEAR && !newptr) {
		assert(!metadata->fallback_removal_count);
		if (metadata->nr_child <= type->min_child) {
			/* We need to try recompacting the node */
			return -EFBIG;
		}
	}
	dbg_printf("linear replace ptr: nr_child_ptr %p\n", nr_child_ptr);
	assert(*node_flag_ptr != NULL);
	rcu_assign_pointer(*node_flag_ptr, newptr);
	/*
	 * Value and nr_child are never changed (would cause ABA issue).
	 * Instead, we leave the pointer to NULL and recompact the node
	 * once in a while. It is allowed to set a NULL pointer to a new
	 * value without recompaction though.
	 * Only update the metadata node accounting.
	 */
	if (!newptr)
		metadata->nr_child--;
	dbg_printf("linear replace ptr: %u child, metadata: %u child, for node %p newptr %p\n",
		(unsigned int) uatomic_load(nr_child_ptr, CMM_RELAXED),
		(unsigned int) metadata->nr_child,
		node, newptr);
	return 0;
}

static
int ft_pool_node_replace_ptr(const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		struct cds_ft_inode_flag *node_flag,
		struct cds_ft_metadata *metadata,
		struct cds_ft_inode_flag **node_flag_ptr,
		uint8_t n,
		struct cds_ft_inode_flag *newptr)
{
	struct cds_ft_inode *linear;
	int ret;

	assert(type->type_class == FT_POOL);

	if (!newptr) {
		if (metadata->fallback_removal_count) {
			metadata->fallback_removal_count--;
		} else {
			/* We should try recompacting the node */
			if (metadata->nr_child <= type->min_child)
				return -EFBIG;
		}
	}

	switch (type->nr_pool_order) {
	case 1:
	{
		unsigned long bitsel, index;

		bitsel = ft_node_pool_1d_bitsel(node_flag);
		assert(bitsel < CHAR_BIT);
		index = ((unsigned long) n >> bitsel) & type->nr_pool_order;
		linear = (struct cds_ft_inode *) &node->u.data[index << type->pool_size_order];
		break;
	}
	case 2:
	{
		unsigned int C_n8_r2_index, subclass_index;
		uint8_t bits[2];

		ft_node_pool_2d_index(node_flag, &C_n8_r2_index);
		index_to_bits_C_n8_r2(C_n8_r2_index, bits);
		subclass_index = value_and_bits_to_subclass_index(n, bits);
		linear = (struct cds_ft_inode *) &node->u.data[subclass_index << type->pool_size_order];
		break;
	}
	default:
		linear = NULL;
		assert(0);
	}

	ret = ft_linear_node_replace_ptr(type, linear, metadata, node_flag_ptr, newptr);
#ifdef FEATURE_USE_BITMAP_SCAN
	if (ret == 0 && !newptr && type->bitmap) {
		struct cds_ft_bitmap *bitmap = cds_ft_item_to_bitmap(node, type->order);

		/* Clear n in bitmap. */
		cds_clear_bit_relaxed(bitmap->bitmap, n);
	}
#endif
	return ret;
}

static
int ft_pigeon_node_replace_ptr(const struct cds_ft_type *type,
		struct cds_ft_inode *node __attribute__((unused)),
		struct cds_ft_metadata *metadata,
		struct cds_ft_inode_flag **node_flag_ptr,
		uint8_t n __attribute__((unused)),
		struct cds_ft_inode_flag *newptr)
{
	assert(type->type_class == FT_PIGEON);

	if (!newptr) {
		if (metadata->fallback_removal_count) {
			metadata->fallback_removal_count--;
		} else {
			/* We should try recompacting the node */
			if (metadata->nr_child <= type->min_child)
				return -EFBIG;
		}
	}
	dbg_printf("ft_pigeon_node_replace_ptr: replace ptr: %p by %p\n", *node_flag_ptr, newptr);
	assert(*node_flag_ptr != NULL);
	rcu_assign_pointer(*node_flag_ptr, newptr);
	if (!newptr) {
#ifdef FEATURE_USE_BITMAP_SCAN
		struct cds_ft_bitmap *bitmap = cds_ft_item_to_bitmap(node, type->order);

		/* Clear n in bitmap. */
		cds_clear_bit_relaxed(bitmap->bitmap, n);
#endif
		metadata->nr_child--;
	}
	return 0;
}

/*
 * _ft_node_replace_ptr: replace ptr item within a node. Return an error
 * (negative error value) if it is not found (-ENOENT).
 */
static
int _ft_node_replace_ptr(const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		struct cds_ft_inode_flag *node_flag,
		struct cds_ft_metadata *metadata,
		struct cds_ft_inode_flag **node_flag_ptr,
		uint8_t n, struct cds_ft_inode_flag *newptr)
{
	switch (type->type_class) {
	case FT_LINEAR:
		return ft_linear_node_replace_ptr(type, node, metadata, node_flag_ptr, newptr);
	case FT_POOL:
		return ft_pool_node_replace_ptr(type, node, node_flag, metadata, node_flag_ptr, n, newptr);
	case FT_PIGEON:
		return ft_pigeon_node_replace_ptr(type, node, metadata, node_flag_ptr, n, newptr);
	case FT_NULL:
		return -ENOENT;
	default:
		assert(0);
		return -EINVAL;
	}

	return 0;
}

/*
 * Calculate bit distribution. Returns the bit (0 to 7) that splits the
 * distribution in two sub-distributions containing as much elements one
 * compared to the other.
 */
static
unsigned int ft_node_sum_distribution_1d(enum ft_recompact mode,
		const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		uint8_t n,
		struct cds_ft_inode_flag **nullify_node_flag_ptr)
{
	uint8_t nr_one[FT_BITS_PER_BYTE];
	unsigned int bitsel = 0, bit_i, overall_best_distance = UINT_MAX;
	unsigned int distrib_nr_child = 0;

	memset(nr_one, 0, sizeof(nr_one));

	switch (type->type_class) {
	case FT_LINEAR:
	{
		uint8_t nr_child =
			ft_linear_node_get_nr_child(type, node);
		unsigned int i;

		for (i = 0; i < nr_child; i++) {
			struct cds_ft_inode_flag *iter;
			uint8_t v;

			ft_linear_node_get_ith_pos(type, node, i, &v, &iter);
			if (!iter)
				continue;
			if (mode == FT_RECOMPACT_DEL && *nullify_node_flag_ptr == iter)
				continue;
			for (bit_i = 0; bit_i < FT_BITS_PER_BYTE; bit_i++) {
				if (v & (1U << bit_i))
					nr_one[bit_i]++;
			}
			distrib_nr_child++;
		}
		break;
	}
	case FT_POOL:
	{
		unsigned int pool_nr;

		for (pool_nr = 0; pool_nr < (1U << type->nr_pool_order); pool_nr++) {
			struct cds_ft_inode *pool =
				ft_pool_node_get_ith_pool(type,
					node, pool_nr);
			uint8_t nr_child =
				ft_linear_node_get_nr_child(type, pool);
			unsigned int j;

			for (j = 0; j < nr_child; j++) {
				struct cds_ft_inode_flag *iter;
				uint8_t v;

				ft_linear_node_get_ith_pos(type, pool,
						j, &v, &iter);
				if (!iter)
					continue;
				if (mode == FT_RECOMPACT_DEL && *nullify_node_flag_ptr == iter)
					continue;
				for (bit_i = 0; bit_i < FT_BITS_PER_BYTE; bit_i++) {
					if (v & (1U << bit_i))
						nr_one[bit_i]++;
				}
				distrib_nr_child++;
			}
		}
		break;
	}
	case FT_PIGEON:
	{
		unsigned int i;

		assert(mode == FT_RECOMPACT_DEL);
		for (i = 0; i < FT_ENTRY_PER_NODE; i++) {
			struct cds_ft_inode_flag *iter;

			iter = ft_pigeon_node_get_ith_pos(type, node, i);
			if (!iter)
				continue;
			if (mode == FT_RECOMPACT_DEL && *nullify_node_flag_ptr == iter)
				continue;
			for (bit_i = 0; bit_i < FT_BITS_PER_BYTE; bit_i++) {
				if (i & (1U << bit_i))
					nr_one[bit_i]++;
			}
			distrib_nr_child++;
		}
		break;
	}
	case FT_NULL:
		assert(mode == FT_RECOMPACT_ADD_NEXT);
		break;
	default:
		assert(0);
		break;
	}

	if (mode == FT_RECOMPACT_ADD_NEXT || mode == FT_RECOMPACT_ADD_SAME) {
		for (bit_i = 0; bit_i < FT_BITS_PER_BYTE; bit_i++) {
			if (n & (1U << bit_i))
				nr_one[bit_i]++;
		}
		distrib_nr_child++;
	}

	/*
	 * The best bit selector is that for which the number of ones is
	 * closest to half of the number of children in the
	 * distribution. We calculate the distance using the double of
	 * the sub-distribution sizes to eliminate truncation error.
	 */
	for (bit_i = 0; bit_i < FT_BITS_PER_BYTE; bit_i++) {
		unsigned int distance_to_best;

		distance_to_best = abs_int(((unsigned int) nr_one[bit_i] << 1U) - distrib_nr_child);
		if (distance_to_best < overall_best_distance) {
			overall_best_distance = distance_to_best;
			bitsel = bit_i;
		}
	}
	dbg_printf("1 dimension pool bit selection: (%u)\n", bitsel);
	return bitsel;
}

/*
 * Calculate bit distribution in two dimensions. Returns the two bits
 * (each 0 to 7) that splits the distribution in four sub-distributions
 * containing as much elements one compared to the other.
 */
static
void ft_node_sum_distribution_2d(enum ft_recompact mode,
		const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		uint8_t n,
		struct cds_ft_inode_flag **nullify_node_flag_ptr,
		unsigned int *_bitsel)
{
	uint8_t nr_2d_11[FT_BITS_PER_BYTE][FT_BITS_PER_BYTE],
		nr_2d_10[FT_BITS_PER_BYTE][FT_BITS_PER_BYTE],
		nr_2d_01[FT_BITS_PER_BYTE][FT_BITS_PER_BYTE],
		nr_2d_00[FT_BITS_PER_BYTE][FT_BITS_PER_BYTE];
	unsigned int bitsel[2] = { 0, 1 };
	unsigned int bit_i, bit_j;
	int overall_best_distance = INT_MAX;
	unsigned int distrib_nr_child = 0;

	memset(nr_2d_11, 0, sizeof(nr_2d_11));
	memset(nr_2d_10, 0, sizeof(nr_2d_10));
	memset(nr_2d_01, 0, sizeof(nr_2d_01));
	memset(nr_2d_00, 0, sizeof(nr_2d_00));

	switch (type->type_class) {
	case FT_LINEAR:
	{
		uint8_t nr_child =
			ft_linear_node_get_nr_child(type, node);
		unsigned int i;

		for (i = 0; i < nr_child; i++) {
			struct cds_ft_inode_flag *iter;
			uint8_t v;

			ft_linear_node_get_ith_pos(type, node, i, &v, &iter);
			if (!iter)
				continue;
			if (mode == FT_RECOMPACT_DEL && *nullify_node_flag_ptr == iter)
				continue;
			for (bit_i = 0; bit_i < FT_BITS_PER_BYTE; bit_i++) {
				for (bit_j = bit_i + 1; bit_j < FT_BITS_PER_BYTE; bit_j++) {
					if (v & (1U << bit_i)) {
						if (v & (1U << bit_j)) {
							nr_2d_11[bit_i][bit_j]++;
						} else {
							nr_2d_10[bit_i][bit_j]++;
						}
					} else {
						if (v & (1U << bit_j)) {
							nr_2d_01[bit_i][bit_j]++;
						} else {
							nr_2d_00[bit_i][bit_j]++;
						}
					}
				}
			}
			distrib_nr_child++;
		}
		break;
	}
	case FT_POOL:
	{
		unsigned int pool_nr;

		for (pool_nr = 0; pool_nr < (1U << type->nr_pool_order); pool_nr++) {
			struct cds_ft_inode *pool =
				ft_pool_node_get_ith_pool(type,
					node, pool_nr);
			uint8_t nr_child =
				ft_linear_node_get_nr_child(type, pool);
			unsigned int j;

			for (j = 0; j < nr_child; j++) {
				struct cds_ft_inode_flag *iter;
				uint8_t v;

				ft_linear_node_get_ith_pos(type, pool,
						j, &v, &iter);
				if (!iter)
					continue;
				if (mode == FT_RECOMPACT_DEL && *nullify_node_flag_ptr == iter)
					continue;
				for (bit_i = 0; bit_i < FT_BITS_PER_BYTE; bit_i++) {
					for (bit_j = bit_i + 1; bit_j < FT_BITS_PER_BYTE; bit_j++) {
						if (v & (1U << bit_i)) {
							if (v & (1U << bit_j)) {
								nr_2d_11[bit_i][bit_j]++;
							} else {
								nr_2d_10[bit_i][bit_j]++;
							}
						} else {
							if (v & (1U << bit_j)) {
								nr_2d_01[bit_i][bit_j]++;
							} else {
								nr_2d_00[bit_i][bit_j]++;
							}
						}
					}
				}
				distrib_nr_child++;
			}
		}
		break;
	}
	case FT_PIGEON:
	{
		unsigned int i;

		assert(mode == FT_RECOMPACT_DEL);
		for (i = 0; i < FT_ENTRY_PER_NODE; i++) {
			struct cds_ft_inode_flag *iter;

			iter = ft_pigeon_node_get_ith_pos(type, node, i);
			if (!iter)
				continue;
			if (mode == FT_RECOMPACT_DEL && *nullify_node_flag_ptr == iter)
				continue;
			for (bit_i = 0; bit_i < FT_BITS_PER_BYTE; bit_i++) {
				for (bit_j = bit_i + 1; bit_j < FT_BITS_PER_BYTE; bit_j++) {
					if (i & (1U << bit_i)) {
						if (i & (1U << bit_j)) {
							nr_2d_11[bit_i][bit_j]++;
						} else {
							nr_2d_10[bit_i][bit_j]++;
						}
					} else {
						if (i & (1U << bit_j)) {
							nr_2d_01[bit_i][bit_j]++;
						} else {
							nr_2d_00[bit_i][bit_j]++;
						}
					}
				}
			}
			distrib_nr_child++;
		}
		break;
	}
	case FT_NULL:
		assert(mode == FT_RECOMPACT_ADD_NEXT);
		break;
	default:
		assert(0);
		break;
	}

	if (mode == FT_RECOMPACT_ADD_NEXT || mode == FT_RECOMPACT_ADD_SAME) {
		for (bit_i = 0; bit_i < FT_BITS_PER_BYTE; bit_i++) {
			for (bit_j = bit_i + 1; bit_j < FT_BITS_PER_BYTE; bit_j++) {
				if (n & (1U << bit_i)) {
					if (n & (1U << bit_j)) {
						nr_2d_11[bit_i][bit_j]++;
					} else {
						nr_2d_10[bit_i][bit_j]++;
					}
				} else {
					if (n & (1U << bit_j)) {
						nr_2d_01[bit_i][bit_j]++;
					} else {
						nr_2d_00[bit_i][bit_j]++;
					}
				}
			}
		}
		distrib_nr_child++;
	}

	/*
	 * The best bit selector is that for which the number of nodes
	 * in each sub-class is closest to one-fourth of the number of
	 * children in the distribution. We calculate the distance using
	 * 4 times the size of the sub-distribution to eliminate
	 * truncation error.
	 */
	for (bit_i = 0; bit_i < FT_BITS_PER_BYTE; bit_i++) {
		for (bit_j = bit_i + 1; bit_j < FT_BITS_PER_BYTE; bit_j++) {
			int distance_to_best[4];

			distance_to_best[0] = ((unsigned int) nr_2d_11[bit_i][bit_j] << 2U) - distrib_nr_child;
			distance_to_best[1] = ((unsigned int) nr_2d_10[bit_i][bit_j] << 2U) - distrib_nr_child;
			distance_to_best[2] = ((unsigned int) nr_2d_01[bit_i][bit_j] << 2U) - distrib_nr_child;
			distance_to_best[3] = ((unsigned int) nr_2d_00[bit_i][bit_j] << 2U) - distrib_nr_child;

			/* Consider worse distance above best */
			if (distance_to_best[1] > 0 && distance_to_best[1] > distance_to_best[0])
				distance_to_best[0] = distance_to_best[1];
			if (distance_to_best[2] > 0 && distance_to_best[2] > distance_to_best[0])
				distance_to_best[0] = distance_to_best[2];
			if (distance_to_best[3] > 0 && distance_to_best[3] > distance_to_best[0])
				distance_to_best[0] = distance_to_best[3];

			/*
			 * If our worse distance is better than overall,
			 * we become new best candidate.
			 */
			if (distance_to_best[0] < overall_best_distance) {
				overall_best_distance = distance_to_best[0];
				bitsel[0] = bit_i;
				bitsel[1] = bit_j;
			}
		}
	}

	dbg_printf("2 dimensions pool bit selection: (%u,%u)\n", bitsel[0], bitsel[1]);

	/* Return our bit selection */
	_bitsel[0] = bitsel[0];
	_bitsel[1] = bitsel[1];
}

static
unsigned int find_nearest_type_index(unsigned int type_index,
		unsigned int nr_nodes, bool is_root)
{
	const struct cds_ft_type *type;

	assert(type_index != NODE_INDEX_NULL);
	if (nr_nodes == 0) {
		/*
		 * The root node is kept alive with 0 children (smallest
		 * linear type).  All other nodes are pruned.
		 */
		return is_root ? 0 : NODE_INDEX_NULL;
	}
	for (;;) {
		type = &ft_types[type_index];
		if (nr_nodes < type->min_child)
			type_index--;
		else if (nr_nodes > type->max_child)
			type_index++;
		else
			break;
	}
	return type_index;
}

/*
 * ft_node_recompact_add: recompact a node, adding a new child.
 * Return 0 on success or negative error value on error.
 */
static
int ft_node_recompact(enum ft_recompact mode,
		struct cds_ft *ft,
		unsigned int old_type_index,
		const struct cds_ft_type *old_type,
		struct cds_ft_inode *old_node,
		struct cds_ft_metadata *metadata,
		struct cds_ft_inode_flag **old_node_flag_ptr, uint8_t n,
		struct cds_ft_inode_flag *child_node_flag,
		struct cds_ft_inode_flag **nullify_node_flag_ptr,
		struct cds_ft_inode **old_node_ret,
		bool is_root)
{
	unsigned int new_type_index;
	struct cds_ft_inode *new_node;
	struct cds_ft_metadata *new_metadata;
	const struct cds_ft_type *new_type;
	struct cds_ft_inode_flag *new_node_flag;
	int ret;
	int fallback = 0;

	/*
	 * Need to find nearest type index even for ADD_SAME, because
	 * this recompaction, when applied to linear nodes, will garbage
	 * collect dummy (NULL) entries, and can therefore cause a few
	 * linear representations to be skipped.
	 */
	switch (mode) {
	case FT_RECOMPACT_ADD_SAME:
		new_type_index = find_nearest_type_index(old_type_index,
			metadata->nr_child + 1, false);
		dbg_printf("Recompact for node with %u children\n",
			metadata->nr_child + 1);
		break;
	case FT_RECOMPACT_ADD_NEXT:
		if (!metadata || old_type_index == NODE_INDEX_NULL) {
			new_type_index = 0;
			dbg_printf("Recompact for NULL\n");
		} else {
			new_type_index = find_nearest_type_index(old_type_index,
				metadata->nr_child + 1, false);
			dbg_printf("Recompact for node with %u children\n",
				metadata->nr_child + 1);
		}
		break;
	case FT_RECOMPACT_DEL:
		new_type_index = find_nearest_type_index(old_type_index,
			metadata->nr_child - 1, is_root);
		dbg_printf("Recompact for node with %u children\n",
			metadata->nr_child - 1);
		break;
	default:
		assert(0);
	}

retry:		/* for fallback */
	new_metadata = NULL;
	dbg_printf("Recompact from type %d to type %d\n",
			old_type_index, new_type_index);
	new_type = &ft_types[new_type_index];
	if (new_type_index != NODE_INDEX_NULL) {
		new_node = alloc_cds_ft_node(ft, new_type, &new_metadata);
		if (!new_node)
			return -ENOMEM;

		if (new_type->type_class == FT_POOL) {
			switch (new_type->nr_pool_order) {
			case 1:
			{
				unsigned int node_distrib_bitsel;

				node_distrib_bitsel =
					ft_node_sum_distribution_1d(mode,
						old_type, old_node,
						n, nullify_node_flag_ptr);
				assert(!((unsigned long) new_node & FT_POOL_1D_MASK));
				new_node_flag = ft_node_flag_pool_1d(new_node,
					new_type_index, node_distrib_bitsel);
				break;
			}
			case 2:
			{
				unsigned int node_distrib_bitsel[2];
				unsigned int subclass_index;
				uint8_t mask;

				ft_node_sum_distribution_2d(mode,
					old_type, old_node,
					n, nullify_node_flag_ptr,
					node_distrib_bitsel);
				assert(!((unsigned long) new_node & FT_POOL_2D_MASK));
				mask = (1U << node_distrib_bitsel[0]) | (1U << node_distrib_bitsel[1]);
				subclass_index = mask_to_index_C_n8_r2(mask);
				new_node_flag = ft_node_flag_pool_2d(new_node,
					new_type_index, subclass_index);
				break;
			}
			default:
				assert(0);
			}
		} else {
			new_node_flag = ft_node_flag(new_node, new_type_index);
		}

		dbg_printf("Recompact inherit from %p\n", metadata);
		if (metadata)
			new_metadata->fallback_removal_count = metadata->fallback_removal_count;
		if (fallback)
			new_metadata->fallback_removal_count =
						FT_FALLBACK_REMOVAL_COUNT;
	} else {
		new_node = NULL;
		new_node_flag = NULL;
	}

	assert(mode != FT_RECOMPACT_ADD_NEXT || old_type->type_class != FT_PIGEON);

	if (new_type_index == NODE_INDEX_NULL)
		goto skip_copy;

	switch (old_type->type_class) {
	case FT_LINEAR:
	{
		uint8_t nr_child =
			ft_linear_node_get_nr_child(old_type, old_node);
		unsigned int i;

		for (i = 0; i < nr_child; i++) {
			struct cds_ft_inode_flag *iter;
			uint8_t v;

			ft_linear_node_get_ith_pos(old_type, old_node, i, &v, &iter);
			if (!iter)
				continue;
			if (mode == FT_RECOMPACT_DEL && *nullify_node_flag_ptr == iter)
				continue;
			ret = _ft_node_set_nth(new_type, new_node, new_node_flag,
					new_metadata, v, iter);
			if (new_type->type_class == FT_POOL && ret) {
				goto fallback_toosmall;
			}
			assert(!ret);
		}
		break;
	}
	case FT_POOL:
	{
		unsigned int pool_nr;

		for (pool_nr = 0; pool_nr < (1U << old_type->nr_pool_order); pool_nr++) {
			struct cds_ft_inode *pool =
				ft_pool_node_get_ith_pool(old_type,
					old_node, pool_nr);
			uint8_t nr_child =
				ft_linear_node_get_nr_child(old_type, pool);
			unsigned int j;

			for (j = 0; j < nr_child; j++) {
				struct cds_ft_inode_flag *iter;
				uint8_t v;

				ft_linear_node_get_ith_pos(old_type, pool,
						j, &v, &iter);
				if (!iter)
					continue;
				if (mode == FT_RECOMPACT_DEL && *nullify_node_flag_ptr == iter)
					continue;
				ret = _ft_node_set_nth(new_type, new_node, new_node_flag,
						new_metadata, v, iter);
				if (new_type->type_class == FT_POOL
						&& ret) {
					goto fallback_toosmall;
				}
				assert(!ret);
			}
		}
		break;
	}
	case FT_NULL:
		assert(mode == FT_RECOMPACT_ADD_NEXT);
		break;
	case FT_PIGEON:
	{
		unsigned int i;

		assert(mode == FT_RECOMPACT_DEL);
		for (i = 0; i < FT_ENTRY_PER_NODE; i++) {
			struct cds_ft_inode_flag *iter;

			iter = ft_pigeon_node_get_ith_pos(old_type, old_node, i);
			if (!iter)
				continue;
			if (mode == FT_RECOMPACT_DEL && *nullify_node_flag_ptr == iter)
				continue;
			ret = _ft_node_set_nth(new_type, new_node, new_node_flag,
					new_metadata, i, iter);
			if (new_type->type_class == FT_POOL && ret) {
				goto fallback_toosmall;
			}
			assert(!ret);
		}
		break;
	}
	default:
		assert(0);
		ret = -EINVAL;
		goto end;
	}
skip_copy:

	if (mode == FT_RECOMPACT_ADD_NEXT || mode == FT_RECOMPACT_ADD_SAME) {
		/* add node */
		ret = _ft_node_set_nth(new_type, new_node, new_node_flag,
				new_metadata, n, child_node_flag);
		if (new_type->type_class == FT_POOL && ret) {
			goto fallback_toosmall;
		}
		assert(!ret);
	}

	if (fallback) {
		dbg_printf("Using fallback for %u children, node type index: %u, mode %s\n",
			new_metadata->nr_child, old_type_index, mode == FT_RECOMPACT_ADD_NEXT ? "add_next" :
				(mode == FT_RECOMPACT_DEL ? "del" : "add_same"));
		if (ft_debug_counters())
			uatomic_inc(&ft->node_fallback_count_distribution[new_metadata->nr_child]);
	}

	/* Return pointer to new recompacted node through old_node_flag_ptr */
	*old_node_flag_ptr = new_node_flag;
	if (old_node && old_node_ret)
		*old_node_ret = old_node;

	ret = 0;
end:
	return ret;

fallback_toosmall:
	/* fallback if next pool is too small */
	free_cds_ft_node(ft, new_node);

	switch (mode) {
	case FT_RECOMPACT_ADD_SAME:
		/*
		 * FT_RECOMPACT_ADD_SAME is only triggered if a linear
		 * node within a pool has unused entries. It should
		 * therefore _never_ be too small.
		 */
		assert(0);

		/* Fall-through */
	case FT_RECOMPACT_ADD_NEXT:
	{
		const struct cds_ft_type *next_type;

		/*
		 * Recompaction attempt on add failed. Should only
		 * happen if target node type is pool. Caused by
		 * hard-to-split distribution. Recompact using the next
		 * distribution size.
		 */
		assert(new_type->type_class == FT_POOL);
		next_type = &ft_types[new_type_index + 1];
		/*
		 * Try going to the next pool size if our population
		 * fits within its range. This is not flagged as a
		 * fallback.
		 */
		if (metadata->nr_child + 1 >= next_type->min_child
				&& metadata->nr_child + 1 <= next_type->max_child) {
			new_type_index++;
			goto retry;
		} else {
			new_type_index++;
			dbg_printf("Add fallback to type %d\n", new_type_index);
			if (ft_debug_counters())
				uatomic_inc(&ft->nr_fallback);
			fallback = 1;
			goto retry;
		}
		break;
	}
	case FT_RECOMPACT_DEL:
		/*
		 * Recompaction attempt on delete failed. Should only
		 * happen if target node type is pool. This is caused by
		 * a hard-to-split distribution. Recompact on same node
		 * size, but flag current node as "fallback" to ensure
		 * we don't attempt recompaction before some activity
		 * has reshuffled our node.
		 */
		assert(new_type->type_class == FT_POOL);
		new_type_index = old_type_index;
		dbg_printf("Delete fallback keeping type %d\n", new_type_index);
		uatomic_inc(&ft->nr_fallback);
		fallback = 1;
		goto retry;
	default:
		assert(0);
		return -EINVAL;
	}

	/*
	 * Last resort fallback: pigeon.
	 */
	new_type_index = (1UL << FT_TYPE_BITS) - 1;
	dbg_printf("Fallback to type %d\n", new_type_index);
	uatomic_inc(&ft->nr_fallback);
	fallback = 1;
	goto retry;
}

/*
 * Return 0 on success or negative error value on error.
 */
static
int ft_node_set_nth(struct cds_ft *ft,
		struct cds_ft_inode_flag **node_flag, uint8_t n,
		struct cds_ft_inode_flag *child_node_flag,
		struct cds_ft_inode **old_node_ret,
		struct cds_ft_metadata *metadata)
{
	int ret;
	unsigned int type_index;
	const struct cds_ft_type *type;
	struct cds_ft_inode *node;

	dbg_printf("ft_node_set_nth for n=%u, node %p\n", (unsigned int) n, ft_node_ptr(*node_flag));

	node = ft_node_ptr(*node_flag);
	type_index = ft_node_type(*node_flag);
	type = &ft_types[type_index];
	ret = _ft_node_set_nth(type, node, *node_flag, metadata, n, child_node_flag);
	switch (ret) {
	case -ENOSPC:
		/* Not enough space in node, need to recompact to next type. */
		ret = ft_node_recompact(FT_RECOMPACT_ADD_NEXT, ft, type_index, type, node,
					metadata, node_flag, n, child_node_flag, NULL,
					old_node_ret, false);
		break;
	case -ERANGE:
		/* Node needs to be recompacted. */
		ret = ft_node_recompact(FT_RECOMPACT_ADD_SAME, ft, type_index, type, node,
					metadata, node_flag, n, child_node_flag, NULL,
					old_node_ret, false);
		break;
	}
	return ret;
}

/*
 * Return 0 on success or negative error value on error.
 */
static
int ft_node_replace_ptr(struct cds_ft *ft,
		struct cds_ft_inode_flag **node_flag_ptr,		/* Pointer to location to nullify */
		struct cds_ft_inode_flag **parent_node_flag_ptr,	/* Address of parent ptr in its parent */
		struct cds_ft_inode **old_node_ret,
		struct cds_ft_metadata *metadata,			/* of parent */
		uint8_t n,
		struct cds_ft_inode_flag *newptr,
		bool is_root)
{
	int ret;
	unsigned int type_index;
	const struct cds_ft_type *type;
	struct cds_ft_inode *node;

	dbg_printf("ft_node_replace_ptr for node %p, target ptr %p\n",
		ft_node_ptr(*parent_node_flag_ptr), node_flag_ptr);

	node = ft_node_ptr(*parent_node_flag_ptr);
	type_index = ft_node_type(*parent_node_flag_ptr);
	type = &ft_types[type_index];
	ret = _ft_node_replace_ptr(type, node, *parent_node_flag_ptr, metadata, node_flag_ptr, n, newptr);
	if (ret == -EFBIG) {
		assert(!newptr);
		/* Should try recompaction. */
		ret = ft_node_recompact(FT_RECOMPACT_DEL, ft, type_index, type, node,
				metadata, parent_node_flag_ptr, n, NULL,
				node_flag_ptr, old_node_ret, is_root);
	}
	return ret;
}

enum ft_prefix_tracking {
	FT_PREFIX_TRACK_NONE,		/* No prefix tracking. */
	FT_PREFIX_TRACK_PARTIAL,	/* Track closest ancestor with external nodes. */
	FT_PREFIX_TRACK_LONGEST,	/* Track deepest match, even internal-only. */
};

/*
 * Sentinel value indicating that prefix tracking never recorded a
 * match. Used by FT_PREFIX_TRACK_LONGEST to distinguish "empty trie"
 * from "matched at root with no external nodes" (both have
 * match_node == NULL, but the latter sets match_len = 0).
 */
#define FT_MATCH_LEN_NONE	SIZE_MAX

static
enum cds_ft_status do_cds_ft_lookup(struct cds_ft *ft,
		const uint8_t *key, size_t _key_len,
		struct cds_ft_node **result_node,
		struct cds_ft_iter *iter,
		enum ft_prefix_tracking tracking,
		size_t *tracking_match_len,
		struct cds_ft_node **tracking_match_node)
{
	size_t key_len = ft_key_len(ft, _key_len);
	struct cds_ft_inode_flag *node_flag;
	struct cds_ft_node *found = NULL;
	unsigned int key_depth, i;
	enum cds_ft_status status;
	size_t iter_path_len = 0;
	bool track = (tracking != FT_PREFIX_TRACK_NONE);
	bool track_longest = (tracking == FT_PREFIX_TRACK_LONGEST);
	size_t match_len = track_longest ? FT_MATCH_LEN_NONE : 0;
	struct cds_ft_node *match_node = NULL;

	if (!valid_key_len(ft, key_len)) {
		status = CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
		goto end;
	}
	key_depth = key_len + 1;
	node_flag = rcu_dereference(ft->root);

	if (iter) {
		iter->path_node[0] = node_flag;
		iter_path_len = 1;
	}

	/*
	 * Root is always internal. For key_len == 0, return the root's
	 * metadata external_nodes (NIL-key entries).
	 */
	if (!key_len) {
		const struct cds_ft_type *type = &ft_types[ft_node_type(node_flag)];
		struct cds_ft_metadata *metadata = cds_ft_item_to_metadata_fast(ft_node_ptr(node_flag),
							type->order);
		found = rcu_dereference(metadata->external_nodes);
		status = found ? CDS_FT_STATUS_OK : CDS_FT_STATUS_NOT_FOUND;
		if (track) {
			match_len = 0;
			match_node = found;
		}
		goto end;
	}

	/*
	 * Consider external node in root metadata as possible match.
	 * For longest-match tracking, record the root position even
	 * when there are no external nodes.
	 */
	if (track) {
		const struct cds_ft_type *type = &ft_types[ft_node_type(node_flag)];
		struct cds_ft_metadata *metadata = cds_ft_item_to_metadata_fast(ft_node_ptr(node_flag),
							type->order);
		struct cds_ft_node *external_nodes = rcu_dereference(metadata->external_nodes);

		if (external_nodes || track_longest) {
			match_len = 0;
			match_node = external_nodes;
		}
	}

	for (i = 1; i < key_depth; i++) {
		uint8_t iter_key;

		iter_key = key_to_ordinal(ft, *(key++));
		node_flag = ft_node_get_nth(node_flag, NULL, iter_key);
		dbg_printf("cds_ft_lookup iter key lookup %u finds node_flag %p\n",
				(unsigned int) iter_key, node_flag);
		if (!ft_node_ptr(node_flag)) {
			status = CDS_FT_STATUS_NOT_FOUND;
			goto end;
		}
		if (iter) {
			iter->path_node[i] = node_flag;
			iter_path_len = i + 1;
		}
		/* Found external node before end of key. */
		if (i < key_depth - 1 && !ft_node_internal(node_flag)) {
			if (track) {
				match_len = i;
				match_node = (struct cds_ft_node *) node_flag;
			}
			status = CDS_FT_STATUS_NOT_FOUND;
			goto end;
		}
		/*
		 * Track prefix match at internal node.
		 * For partial tracking, only record when external nodes
		 * are present. For longest-match tracking, record the
		 * position unconditionally.
		 * Skip the last level. It is handled after the loop.
		 */
		if (track && i < key_depth - 1 && ft_node_internal(node_flag)) {
			const struct cds_ft_type *type = &ft_types[ft_node_type(node_flag)];
			struct cds_ft_metadata *metadata = cds_ft_item_to_metadata_fast(
					ft_node_ptr(node_flag), type->order);
			struct cds_ft_node *external_nodes = rcu_dereference(metadata->external_nodes);

			if (external_nodes || track_longest) {
				match_len = i;
				match_node = external_nodes;
			}
		}
	}

	/*
	 * Reached key_depth, check for terminal node: either external
	 * nodes or internal node associated with external nodes.
	 */
	if (ft_node_internal(node_flag)) {
		const struct cds_ft_type *type = &ft_types[ft_node_type(node_flag)];
		struct cds_ft_metadata *metadata = cds_ft_item_to_metadata_fast(ft_node_ptr(node_flag),
							type->order);
		found = rcu_dereference(metadata->external_nodes);
		status = found ? CDS_FT_STATUS_OK : CDS_FT_STATUS_NOT_FOUND;
		if (track && (found || track_longest)) {
			match_len = key_len;
			match_node = found;
		}
	} else {
		found = (struct cds_ft_node *) node_flag;
		status = CDS_FT_STATUS_OK;
		if (track) {
			match_len = key_len;
			match_node = found;
		}
	}

end:
	if (result_node)
		*result_node = found;
	if (iter) {
		iter->node = found;
		iter->status = status;
		iter->path_len = iter_path_len;
		/*
		 * The path is valid for backtracking when we
		 * successfully descended into the trie, even if the
		 * exact key was not found.
		 */
		iter->path_valid = (status == CDS_FT_STATUS_OK);
	}
	if (track) {
		*tracking_match_len = match_len;
		*tracking_match_node = match_node;
	}
	return status;
}

enum cds_ft_status cds_ft_lookup_key(struct cds_ft *ft,
		const uint8_t *key, size_t key_len,
		struct cds_ft_node **result_node)
{
	return do_cds_ft_lookup(ft, key, key_len, result_node, NULL,
				FT_PREFIX_TRACK_NONE, NULL, NULL);
}

enum cds_ft_status cds_ft_lookup(struct cds_ft *ft,
		struct cds_ft_iter *iter)
{
	return do_cds_ft_lookup(ft, iter->key, iter->key_len, NULL, iter,
				FT_PREFIX_TRACK_NONE, NULL, NULL);
}

enum cds_ft_status cds_ft_lookup_partial_key(struct cds_ft *ft,
		const uint8_t *key, size_t _key_len, size_t *match_len,
		struct cds_ft_node **result_node)
{
	struct cds_ft_node *partial_node = NULL;
	size_t partial_len = 0;

	do_cds_ft_lookup(ft, key, _key_len, NULL, NULL,
			 FT_PREFIX_TRACK_PARTIAL, &partial_len, &partial_node);

	*match_len = partial_len;
	*result_node = partial_node;
	return partial_node ? CDS_FT_STATUS_OK : CDS_FT_STATUS_NOT_FOUND;
}

enum cds_ft_status cds_ft_lookup_partial(struct cds_ft *ft,
		struct cds_ft_iter *iter)
{
	struct cds_ft_node *partial_node = NULL;
	size_t partial_len = 0;

	/*
	 * Perform the full lookup (populating the iterator path for
	 * backtracking) while simultaneously tracking the closest
	 * ancestor with external nodes for partial-match semantics.
	 */
	do_cds_ft_lookup(ft, iter->key, iter->key_len, NULL, iter,
			 FT_PREFIX_TRACK_PARTIAL, &partial_len, &partial_node);

	/*
	 * Override the iterator's node and status with the partial-match
	 * result.
	 */
	iter->node = partial_node;
	iter->key_len = partial_len;
	iter->path_len = partial_len + 1;
	iter->status = partial_node ? CDS_FT_STATUS_OK : CDS_FT_STATUS_NOT_FOUND;
	return iter->status;
}

enum cds_ft_status cds_ft_lookup_longest_match_key(struct cds_ft *ft,
		const uint8_t *key, size_t key_len, size_t *match_len,
		struct cds_ft_node **result_node)
{
	struct cds_ft_node *match_node = NULL;
	size_t longest_len = 0;
	enum cds_ft_status ret;

	ret = do_cds_ft_lookup(ft, key, key_len, NULL, NULL,
			       FT_PREFIX_TRACK_LONGEST, &longest_len, &match_node);

	if (ret < 0) {
		*match_len = 0;
		*result_node = NULL;
		return ret;
	}
	if (longest_len == FT_MATCH_LEN_NONE) {
		*match_len = 0;
		*result_node = NULL;
		return CDS_FT_STATUS_NOT_FOUND;
	}
	*match_len = longest_len;
	*result_node = match_node;
	return match_node ? CDS_FT_STATUS_OK : CDS_FT_STATUS_INTERNAL_MATCH;
}

enum cds_ft_status cds_ft_lookup_longest_match(struct cds_ft *ft,
		struct cds_ft_iter *iter)
{
	struct cds_ft_node *match_node = NULL;
	size_t longest_len = 0;
	enum cds_ft_status ret;

	ret = do_cds_ft_lookup(ft, iter->key, iter->key_len, NULL, iter,
			       FT_PREFIX_TRACK_LONGEST, &longest_len, &match_node);

	if (ret < 0) {
		iter->node = NULL;
		iter->status = ret;
		return ret;
	}
	if (longest_len == FT_MATCH_LEN_NONE) {
		iter->node = NULL;
		iter->status = CDS_FT_STATUS_NOT_FOUND;
		return CDS_FT_STATUS_NOT_FOUND;
	}
	iter->node = match_node;
	iter->key_len = longest_len;
	iter->path_len = longest_len + 1;
	iter->status = match_node ? CDS_FT_STATUS_OK : CDS_FT_STATUS_INTERNAL_MATCH;
	iter->path_valid = true;
	return iter->status;
}

/*
 * Iterator-based inequality lookup. The input key and key_len are read
 * from @iter (set via cds_ft_iter_set_key). On success the result key,
 * key length, node, status, and path are written back into @iter so that
 * subsequent iteration / backtracking calls can reuse the state.
 *
 * @limit overrides the key: FT_LOOKUP_LIMIT_FIRST uses key_len
 * prefix_len, FT_LOOKUP_LIMIT_LAST uses max_key_len.
 *
 * Prefix-scoped traversal: when iter->prefix_len > 0, the traversal
 * is confined to the subtree rooted at the prefix. Backtracking stops
 * at the prefix boundary instead of the root. LIMIT_FIRST finds the
 * smallest key within the prefix subtree. LIMIT_LAST descends using
 * the actual prefix key bytes followed by 0xFF to find the greatest
 * key within the prefix subtree.
 */
static
enum cds_ft_status cds_ft_lookup_inequality(struct cds_ft *ft,
		struct cds_ft_iter *iter,
		enum ft_lookup_inequality mode,
		enum ft_lookup_limit limit)
{
	ssize_t key_depth, level;
	struct cds_ft_inode_flag *node_flag;
	struct cds_ft_node *ret_node;
	uint8_t ordinal_key[FT_MAX_KEY_LEN];
	enum ft_direction dir;
	const uint8_t input_key[FT_MAX_KEY_LEN];
	const uint8_t *iter_key;
	size_t key_len;
	bool going_up = false, skip_eq_external_nodes;

	switch (limit) {
	case FT_LOOKUP_LIMIT_NONE:
		key_len = ft_key_len(ft, iter->key_len);
		if (!valid_key_len(ft, key_len)) {
			iter->node = NULL;
			iter->path_valid = false;
			iter->status = CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
			return iter->status;
		}
		break;
	case FT_LOOKUP_LIMIT_FIRST:
		key_len = iter->prefix_len;
		break;
	case FT_LOOKUP_LIMIT_LAST:
		key_len = ft->group->max_key_len;
		break;
	}

	key_depth = key_len + 1;

	switch (mode) {
	case FT_LOOKUP_GE:
	case FT_LOOKUP_LE:
	case FT_LOOKUP_GT:
	case FT_LOOKUP_LT:
		break;
	default:
		abort();	/* Internal library error. */
	}

	/*
	 * Snapshot the input key so that iter->key can be overwritten
	 * with the result key without corrupting the input during the
	 * backtracking phase (which re-reads the input via iter_key).
	 */
	memcpy((uint8_t *) input_key, iter->key, key_len);
	iter_key = input_key;

	memset(ordinal_key, 0, ft->group->max_key_len * sizeof(ordinal_key[0]));
	node_flag = rcu_dereference(ft->root);
	iter->path_node[0] = node_flag;

	/* Root is always present and always internal. */

	{
		unsigned int type_idx = ft_node_type(node_flag);
		const struct cds_ft_type *type = &ft_types[type_idx];

		/*
		 * Empty root short-circuit: when the root has no children,
		 * there is nothing to traverse and no inequality match is
		 * possible. An empty root is always a type-0 linear node
		 * with data[0] == 0.
		 */
		if (type->type_class == FT_LINEAR &&
				ft_linear_node_get_nr_child(type, ft_node_ptr(node_flag)) == 0) {

			/* A NIL key might still be stored directly in the root's metadata. */
			struct cds_ft_metadata *metadata = cds_ft_item_to_metadata_fast(
					ft_node_ptr(node_flag), type->order);

			if (!uatomic_load(&metadata->external_nodes, CMM_RELAXED)) {
				iter->node = NULL;
				iter->path_valid = true;
				iter->path_len = 1;
				iter->status = CDS_FT_STATUS_NOT_FOUND;
				return iter->status;
			}
		}
	}

	/*
	 * Fast path: reuse the iterator's cached path from a prior
	 * traversal when it is still valid and covers the full key
	 * depth.  This avoids redundant per-level ft_node_get_nth()
	 * lookups (which are the expensive, cache-miss-prone part of
	 * the downward walk).  The caller must hold the RCU read-side
	 * lock continuously for the cached pointers to remain valid.
	 */
	if (iter->path_valid && (ssize_t)iter->path_len >= key_depth &&
			key_depth > 1) {
		for (level = 1; level < key_depth; level++) {
			switch (limit) {
			case FT_LOOKUP_LIMIT_NONE:
				ordinal_key[level - 1] =
					key_to_ordinal(ft, input_key[level - 1]);
				break;
			case FT_LOOKUP_LIMIT_FIRST:
				ordinal_key[level - 1] =
					key_to_ordinal(ft, input_key[level - 1]);
				break;
			case FT_LOOKUP_LIMIT_LAST:
				if ((size_t) level <= iter->prefix_len)
					ordinal_key[level - 1] =
						key_to_ordinal(ft, input_key[level - 1]);
				else
					ordinal_key[level - 1] = 0xff;
				break;
			}
		}
		node_flag = iter->path_node[key_depth - 1];
		/*
		 * Reconstruct the loop exit value of @level to match
		 * what the traversal loop would have produced:
		 *  - key_depth     if last node is internal (loop ran
		 *                  to completion),
		 *  - key_depth - 1 if last node is external, NULL, or
		 *                  the path ended early (loop broke).
		 */
		if (ft_node_ptr(node_flag) && ft_node_internal(node_flag))
			level = key_depth;
		else
			level = key_depth - 1;
		goto post_traversal;
	}

	for (level = 1; level < key_depth; level++) {
		uint8_t key_value;

		switch (limit) {
		case FT_LOOKUP_LIMIT_NONE:
			key_value = key_to_ordinal(ft, *(iter_key++));
			break;
		case FT_LOOKUP_LIMIT_FIRST:
			key_value = key_to_ordinal(ft, input_key[level - 1]);
			break;
		case FT_LOOKUP_LIMIT_LAST:
			if ((size_t) level <= iter->prefix_len)
				key_value = key_to_ordinal(ft, input_key[level - 1]);
			else
				key_value = 0xff;
			break;
		}
		node_flag = ft_node_get_nth(node_flag, NULL, key_value);
		if (!ft_node_ptr(node_flag))
			break;
		ordinal_key[level - 1] = key_value;
		iter->path_node[level] = node_flag;
		dbg_printf("cds_ft_lookup_inequality iter key lookup %u finds node_flag %p\n",
				(unsigned int) key_value, node_flag);
		if (!ft_node_internal(node_flag))
			break;
	}

post_traversal:
	switch (mode) {
	case FT_LOOKUP_LE:
	case FT_LOOKUP_GE:
		if (level == key_depth - 1) {
			struct cds_ft_node *external_nodes;

			if (ft_node_internal(node_flag)) {
				struct cds_ft_metadata *metadata;
				const struct cds_ft_type *type = &ft_types[ft_node_type(node_flag)];

				metadata = cds_ft_item_to_metadata_fast(ft_node_ptr(node_flag), type->order);
				external_nodes = rcu_dereference(metadata->external_nodes);
			} else {
				external_nodes = (struct cds_ft_node *) node_flag;
			}
			if (external_nodes) {
				/* End of key lookup succeded. We got an equal match. */
				iter->key_len = key_len;
				memcpy(iter->key, input_key, key_len);
				iter->node = external_nodes;
				iter->path_valid = true;
				iter->path_len = level + 1;
				iter->status = CDS_FT_STATUS_OK;
				return iter->status;
			}
		}
		break;
	case FT_LOOKUP_LT:
	case FT_LOOKUP_GT:
		break;
	default:
		assert(0);
	}

	/* If we reach end of key, we need to go one level backward. */
	if (level >= key_depth)
		level = key_depth - 1;

	/*
	 * For GE/GT: if the descent completed and the node at end-of-key
	 * is internal, any descendant key is strictly longer and therefore
	 * strictly greater. Skip backtracking and descend into children
	 * directly.
	 *
	 * For GE, the post-traversal above already returned if the node
	 * had external_nodes (the equal match). Reaching this point means
	 * no equal match exists, so descendant keys are the closest >=.
	 *
	 * For GT, the skip_eq_external_nodes flag (set below) will
	 * prevent the minmax descent from returning this node's own
	 * external_nodes (which are the equal match, not GT).
	 *
	 * LE/LT do not need this: their upward backtracking already
	 * checks external_nodes at each internal node going up, which is
	 * the correct direction to find shorter (lesser) prefix keys.
	 */
	if ((mode == FT_LOOKUP_GT || mode == FT_LOOKUP_GE) &&
			ft_node_ptr(node_flag) && ft_node_internal(node_flag))
		goto descend_children;

	/* Ensure iter_key is exactly at the position matching the level we stopped at. */
	iter_key = input_key + level;

	/*
	 * Find highest value left/right of current node.
	 * Current node is iter->path_node[level].
	 * Start at current level. If we cannot find any key left/right
	 * of ours, go one level up, seek highest value left/right of
	 * current (recursively), and when we find one, get the
	 * rightmost/leftmost child of its rightmost/leftmost child
	 * (recursively).
	 *
	 * Prefix-scoped traversal: backtracking stops at
	 * iter->prefix_len instead of 0, confining the search to the
	 * prefix subtree. When prefix_len == 0 this is identical to
	 * the original behavior.
	 */
	switch (mode) {
	case FT_LOOKUP_LE:
	case FT_LOOKUP_LT:
		dir = FT_LEFT;
		break;
	case FT_LOOKUP_GE:
	case FT_LOOKUP_GT:
		dir = FT_RIGHT;
		break;
	default:
		assert(0);
	}
	for (; level > (ssize_t) iter->prefix_len; level--) {
		uint8_t key_value;

		/*
		 * Return external node if trying to find LE/LT
		 * inequality and encountering an external node when
		 * going upward.
		 */
		if (going_up && dir == FT_LEFT && ft_node_internal(iter->path_node[level])) {
			const struct cds_ft_type *type = &ft_types[ft_node_type(iter->path_node[level])];
			struct cds_ft_metadata *metadata = cds_ft_item_to_metadata_fast(ft_node_ptr(iter->path_node[level]), type->order);
			struct cds_ft_node *external_nodes = rcu_dereference(metadata->external_nodes);

			if (external_nodes) {
				int j;

				assert(ft->group->key_len == CDS_FT_LEN_VARIABLE || level <= (int) ft->group->key_len);
				iter->key_len = level;
				for (j = 0; j < level; j++)
					iter->key[j] = ordinal_to_key(ft, ordinal_key[j]);
				iter->node = external_nodes;
				iter->path_valid = true;
				iter->path_len = level + 1;
				iter->status = CDS_FT_STATUS_OK;
				return iter->status;
			}
		}

		switch (limit) {
		case FT_LOOKUP_LIMIT_NONE:
			key_value = key_to_ordinal(ft, *(--iter_key));
			break;
		case FT_LOOKUP_LIMIT_FIRST:
			key_value = key_to_ordinal(ft, input_key[level - 1]);
			break;
		case FT_LOOKUP_LIMIT_LAST:
			if ((size_t) level <= iter->prefix_len)
				key_value = key_to_ordinal(ft, input_key[level - 1]);
			else
				key_value = 0xff;
			break;
		}
		/*
		 * Standard sibling lookup. Parent is level - 1. We are
		 * looking for sibling of the byte at ordinal_key[level - 1].
		 */
		node_flag = ft_node_get_leftright(iter->path_node[level - 1],
				key_value, &ordinal_key[level - 1], dir);
		dbg_printf("cds_ft_lookup_inequality find sibling from %u at %u finds node_flag %p\n",
				(unsigned int) key_value, (unsigned int) ordinal_key[level - 1],
				node_flag);
		/* If found left/right sibling, find rightmost/leftmost child. */
		if (ft_node_ptr(node_flag)) {
			/* Record the sibling in the path. */
			iter->path_node[level] = node_flag;
			break;
		}
		going_up = true;
	}

	/*
	 * Prefix-scoped traversal: if backtracking exhausted the
	 * scope without finding a sibling, handle the prefix
	 * boundary.
	 *
	 * For LE/LT the prefix key itself (shorter than the search
	 * key) may be the closest match: return its external_nodes
	 * if present.
	 *
	 * For GE/GT no key within the scope satisfies the inequality.
	 *
	 * When going_up is false (e.g. LIMIT_FIRST/LIMIT_LAST
	 * reaching the prefix node without backtracking), we fall
	 * through to the downward min/max search below.
	 */
	if (going_up && level == (ssize_t) iter->prefix_len) {
		if (dir == FT_LEFT) {
			struct cds_ft_inode_flag *pfx_flag =
				iter->path_node[iter->prefix_len];

			if (ft_node_ptr(pfx_flag) && ft_node_internal(pfx_flag)) {
				const struct cds_ft_type *type =
					&ft_types[ft_node_type(pfx_flag)];
				struct cds_ft_metadata *metadata =
					cds_ft_item_to_metadata_fast(
						ft_node_ptr(pfx_flag),
						type->order);
				struct cds_ft_node *external_nodes =
					rcu_dereference(metadata->external_nodes);

				if (external_nodes) {
					int j;

					iter->key_len = iter->prefix_len;
					for (j = 0; j < (int) iter->prefix_len; j++)
						iter->key[j] = ordinal_to_key(ft, ordinal_key[j]);
					iter->node = external_nodes;
					iter->path_valid = true;
					iter->path_len = iter->prefix_len + 1;
					iter->status = CDS_FT_STATUS_OK;
					return iter->status;
				}
			}
		}
		iter->node = NULL;
		iter->path_valid = true;
		iter->path_len = iter->prefix_len + 1;
		iter->status = CDS_FT_STATUS_NOT_FOUND;
		return iter->status;
	}

descend_children:
	if (!ft_node_internal(node_flag)) {
		int j;

		assert(ft->group->key_len == CDS_FT_LEN_VARIABLE || level <= (int) ft->group->key_len);
		iter->key_len = level;
		for (j = 0; j < level; j++)
			iter->key[j] = ordinal_to_key(ft, ordinal_key[j]);
		iter->node = (struct cds_ft_node *) ft_node_ptr(node_flag);
		iter->path_valid = true;
		iter->path_len = level + 1;
		iter->status = CDS_FT_STATUS_OK;
		return iter->status;
	}

	level++;

	/*
	 * From this point, we are guaranteed to be able to find a
	 * "lower than"/"greater than" match. ft_attach_node() and
	 * ft_detach_node() both guarantee that it is not possible for a
	 * lookup to reach a dead-end.
	 */

	/*
	 * Find rightmost/leftmost child of rightmost/leftmost child
	 * (recursively).
	 *
	 * skip_eq_external_nodes: when entering the minmax descent
	 * without backtracking (going_up == false) and the mode is
	 * strictly GT, the external_nodes at the first node are at the
	 * same position as the search key — equal, not strictly
	 * greater. Skip them on the first iteration so the descent
	 * continues to a proper child.
	 */
	skip_eq_external_nodes = (!going_up && mode == FT_LOOKUP_GT);
	switch (mode) {
	case FT_LOOKUP_LE:
	case FT_LOOKUP_LT:
		dir = FT_RIGHTMOST;
		break;
	case FT_LOOKUP_GE:
	case FT_LOOKUP_GT:
		dir = FT_LEFTMOST;
		break;
	default:
		assert(0);
	}
	for (; level < (int) ft->group->max_tree_depth; level++) {
		/*
		 * Return external node associated to internal node if
		 * trying to find GE/GT inequality and encountering an
		 * external node when going downward.
		 */
		if (dir == FT_LEFTMOST && ft_node_internal(node_flag)
				&& !skip_eq_external_nodes) {
			const struct cds_ft_type *type = &ft_types[ft_node_type(node_flag)];
			struct cds_ft_metadata *metadata = cds_ft_item_to_metadata_fast(
					ft_node_ptr(node_flag), type->order);
			struct cds_ft_node *external_nodes = rcu_dereference(metadata->external_nodes);

			if (external_nodes) {
				ret_node = external_nodes;
				level--;
				goto end;
			}
		}
		skip_eq_external_nodes = false;
		/* Return external node. */
		if (!ft_node_internal(node_flag))
			break;
		node_flag = ft_node_get_minmax(node_flag, &ordinal_key[level - 1], dir, level == 1);
		/*
		 * If minmax returns NULL, it was an empty root. We found nothing.
		 */
		if (caa_unlikely(!ft_node_ptr(node_flag))) {
			iter->node = NULL;
			iter->path_valid = true;
			iter->path_len = level;
			iter->status = CDS_FT_STATUS_NOT_FOUND;
			return iter->status;
		}
		iter->path_node[level] = node_flag;
		dbg_printf("cds_ft_lookup_inequality find minmax at %u finds node_flag %p\n",
				(unsigned int) ordinal_key[level - 1], node_flag);
		if (!ft_node_internal(node_flag))
			break;
	}
	/* attach/detach semantic guarantees that ft_node_get_minmax cannot return NULL. */
	assert(ft_node_ptr(node_flag));
	ret_node = (struct cds_ft_node *) node_flag;
	/*
	 * The trie should always have external nodes at the
	 * very last level, so level should never grow large enough to overflow
	 * max_key_len.
	 */
	assert(level <= (int) ft->group->max_key_len);
end:
	{
		int j;

		iter->key_len = level;
		for (j = 0; j < level; j++)
			iter->key[j] = ordinal_to_key(ft, ordinal_key[j]);
		iter->node = ret_node;
		iter->path_valid = true;
		iter->path_len = level + 1;
		iter->status = ret_node ? CDS_FT_STATUS_OK : CDS_FT_STATUS_NOT_FOUND;
		return iter->status;
	}
}

/*
 * Iterator-based inequality lookup public API.
 * The caller sets the key via cds_ft_iter_set_key() before calling.
 * On return the iterator holds the result key, key length, node, path,
 * and status.
 */
enum cds_ft_status cds_ft_lookup_le(struct cds_ft *ft,
		struct cds_ft_iter *iter)
{
	dbg_printf("cds_ft_lookup_le\n");
	return cds_ft_lookup_inequality(ft, iter,
			FT_LOOKUP_LE, FT_LOOKUP_LIMIT_NONE);
}

enum cds_ft_status cds_ft_lookup_ge(struct cds_ft *ft,
		struct cds_ft_iter *iter)
{
	dbg_printf("cds_ft_lookup_ge\n");
	return cds_ft_lookup_inequality(ft, iter,
			FT_LOOKUP_GE, FT_LOOKUP_LIMIT_NONE);
}

enum cds_ft_status cds_ft_lookup_lt(struct cds_ft *ft,
		struct cds_ft_iter *iter)
{
	dbg_printf("cds_ft_lookup_lt\n");
	return cds_ft_lookup_inequality(ft, iter,
			FT_LOOKUP_LT, FT_LOOKUP_LIMIT_NONE);
}

enum cds_ft_status cds_ft_lookup_gt(struct cds_ft *ft,
		struct cds_ft_iter *iter)
{
	dbg_printf("cds_ft_lookup_gt\n");
	return cds_ft_lookup_inequality(ft, iter,
			FT_LOOKUP_GT, FT_LOOKUP_LIMIT_NONE);
}

enum cds_ft_status cds_ft_lookup_first(struct cds_ft *ft,
		struct cds_ft_iter *iter)
{
	size_t saved_key_len = iter->key_len;
	enum cds_ft_status status;

	dbg_printf("cds_ft_lookup_first\n");
	/*
	 * LIMIT_FIRST sets key_len to prefix_len internally.
	 * When prefix_len == 0 this corresponds to a traversal of the
	 * entire trie.
	 * When prefix_len > 0 it descends through the prefix key
	 * bytes, then the GE post-traversal returns the prefix key
	 * itself if it has external_nodes, or the LEFTMOST minmax
	 * descent finds the smallest descendant.
	 */
	iter->key_len = iter->prefix_len;
	status = cds_ft_lookup_inequality(ft, iter,
			FT_LOOKUP_GE, FT_LOOKUP_LIMIT_FIRST);
	if (status < 0)
		iter->key_len = saved_key_len;
	return status;
}

enum cds_ft_status cds_ft_lookup_last(struct cds_ft *ft,
		struct cds_ft_iter *iter)
{
	size_t saved_key_len = iter->key_len;
	enum cds_ft_status status;

	dbg_printf("cds_ft_lookup_last\n");
	/*
	 * LIMIT_LAST always uses key_len = max_key_len. When
	 * prefix_len > 0, the traversal uses actual prefix key bytes
	 * for levels 1..prefix_len then 0xFF for the remaining
	 * levels, descending as deep as possible along the rightmost
	 * path within the prefix subtree. LE backtracking (bounded
	 * at prefix_len) then finds the greatest actual key.
	 */
	iter->key_len = ft->group->max_key_len;
	status = cds_ft_lookup_inequality(ft, iter,
			FT_LOOKUP_LE, FT_LOOKUP_LIMIT_LAST);
	if (status < 0)
		iter->key_len = saved_key_len;
	return status;
}

/*
 * We reached an unpopulated node. Create it and the children we need,
 * and then attach the entire branch to the current node. This may
 * trigger recompaction of the current node.
 *
 * ft_attach_node() ensures that a lookup will _never_ see a branch that
 * leads to a dead-end: before attaching a branch, the entire content of
 * the new branch is populated, thus creating a cluster, before
 * attaching the cluster to the rest of the tree, thus making it visible
 * to lookups.
 *
 * @external_node argument is either NULL or a pointer to the external
 * node we are replacing at the attachment location. We need to chain
 * this external node in the topmost internal node external node list in
 * that case.
 */
static
int ft_attach_node(struct cds_ft *ft,
		struct cds_ft_inode_flag **attach_node_flag_ptr,
		struct cds_ft_inode_flag *attach_node_flag,
		struct cds_ft_inode_flag **old_node_flag_ptr,
		struct cds_ft_inode_flag *old_node_flag,
		const uint8_t *key,
		size_t key_len,
		unsigned int level,
		struct cds_ft_node *child_node,
		struct cds_ft_node *external_nodes)
{
	struct cds_ft_metadata *metadata = NULL;
	struct cds_ft_inode_flag *iter_node_flag, *iter_dest_node_flag,
				*created_nodes[FT_MAX_DEPTH];
	struct cds_ft_inode *old_recompacted_node = NULL;
	int ret, i, nr_created_nodes = 0;
	const uint8_t *iter_key = key + key_len;

	dbg_printf("Attach node at level %u (old_node_flag %p, attach_node_flag_ptr %p attach_node_flag %p)\n",
		level, old_node_flag, attach_node_flag_ptr, attach_node_flag);

	assert(!old_node_flag || external_nodes);
	assert(level > 0);	/* Root is always internal; level 0 is handled directly. */
	if (attach_node_flag)
		metadata = cds_ft_item_to_metadata(ft_node_ptr(attach_node_flag));

	/* Concurrent update prevented by mutual exclusion. */
	assert(!(old_node_flag_ptr && (ft_node_ptr(*old_node_flag_ptr) && !external_nodes)));

	/* Concurrent update prevented by mutual exclusion. */
	assert(!(attach_node_flag_ptr && ft_node_ptr(*attach_node_flag_ptr) !=
			ft_node_ptr(attach_node_flag)));

	/* Create new branch, starting from bottom */
	iter_node_flag = (struct cds_ft_inode_flag *) child_node;

	for (i = key_len; i > (int) level; i--) {
		uint8_t key_value;

		key_value = key_to_ordinal(ft, *(--iter_key));
		dbg_printf("branch creation level %d, key %u\n",
				i, (unsigned int) key_value);
		iter_dest_node_flag = NULL;
		ret = ft_node_set_nth(ft, &iter_dest_node_flag, key_value, iter_node_flag, NULL, NULL);
		if (ret) {
			dbg_printf("branch creation error %d\n", ret);
			goto check_error;
		}
		created_nodes[nr_created_nodes++] = iter_dest_node_flag;
		iter_node_flag = iter_dest_node_flag;
	}

	/* Chain previous external node into new branch topmost internal node metadata. */
	if (external_nodes) {
		struct cds_ft_metadata *iter_node_metadata;

		iter_node_metadata = cds_ft_item_to_metadata(ft_node_ptr(iter_node_flag));
		iter_node_metadata->external_nodes = external_nodes;
	}

	/* Publish branch. */
	{
		uint8_t key_value;

		key_value = key_to_ordinal(ft, *(--iter_key));
		dbg_printf("publish branch at level %d, key %u\n", level - 1, (unsigned int) key_value);
		/* We need to use set_nth on the previous level. */
		iter_dest_node_flag = attach_node_flag;
		ret = ft_node_set_nth(ft, &iter_dest_node_flag, key_value, iter_node_flag,
				&old_recompacted_node, metadata);
		if (ret) {
			dbg_printf("branch publish error %d\n", ret);
			goto check_error;
		}
		/* Attach branch (unlink the old node from the trie). */
		rcu_assign_pointer(*attach_node_flag_ptr, iter_dest_node_flag);

		/* Reclaim safely after unlink.*/
		if (old_recompacted_node)
			free_cds_ft_node(ft, old_recompacted_node);
	}

	/* Success */
	ret = 0;

check_error:
	if (ret) {
		for (i = 0; i < nr_created_nodes; i++)
			free_cds_ft_node(ft, ft_node_ptr(created_nodes[i]));
	}
	return ret;
}

static
void ft_chain_node(struct cds_ft_node *last_node, struct cds_ft_node *node)
{
	/*
	 * Add node to tail of list to ensure that RCU traversals will
	 * always see either the prior node or the newly added if
	 * executed concurrently with a sequence of add followed by del
	 * on the same key. Safe against concurrent RCU read traversals.
	 */
	node->next = NULL;
	rcu_assign_pointer(last_node->next, node);
}

/*
 * There are a few cases to cover for add:
 *
 * 1) There is already an external node at that key. Chain this new node
 *    with the existing node (duplicate).
 * 2) There is already an internal node with associated external node at
 *    that key. Chain this new node with the existing node (duplicate).
 * 3) The traversal ends before reaching the end of the lookup key:
 *    3.1) The last node encountered during traversal is an internal
 *         node. Attach a new cluster as child of this internal node.
 *    3.2) The last node encountered during traversal is an external
 *         node. Need to transform this external node into an internal
 *         node with associated external node, attach a new cluster as
 *         child of this internal node, and populate this new internal
 *         node into the tree to replace the prior external node.
 */
static
int _cds_ft_insert(struct cds_ft *ft,
		const uint8_t *key, size_t _key_len,
		struct cds_ft_node *node,
		struct cds_ft_node **unique_node_ret)
{
	unsigned int i, key_depth;
	struct cds_ft_inode_flag *attach_node_flag, *parent_node_flag,
		*parent2_node_flag, *node_flag;
	struct cds_ft_inode_flag **attach_node_flag_ptr,
		**parent_node_flag_ptr, **node_flag_ptr;
	const uint8_t *iter_key = key;
	size_t key_len = ft_key_len(ft, _key_len);
	int ret;

	if (!valid_external_node(node) || !valid_key_len(ft, key_len))
		return -EINVAL;
	/* Expect zeroed next pointer. This catches some double-insert misuses. */
	if (node->next)
		return -EINVAL;

	key_depth = key_len + 1;

	dbg_printf("cds_ft_insert attempt: node %p\n", node);
	parent2_node_flag = NULL;
	parent_node_flag = NULL;
	parent_node_flag_ptr = NULL;
	node_flag = ft->root;
	node_flag_ptr = &ft->root;

	for (i = 0; i < key_depth - 1; i++) {
		uint8_t key_value;

		if (!ft_node_ptr(node_flag))
			break;
		/* Found external node. */
		if (!ft_node_internal(node_flag))
			break;
		dbg_printf("cds_ft_insert iter parent2_node_flag %p parent_node_flag %p node_flag_ptr %p node_flag %p\n",
				parent2_node_flag, parent_node_flag, node_flag_ptr, node_flag);
		key_value = key_to_ordinal(ft, *(iter_key++));
		parent2_node_flag = parent_node_flag;
		parent_node_flag = node_flag;
		parent_node_flag_ptr = node_flag_ptr;
		node_flag = ft_node_get_nth(node_flag, &node_flag_ptr, key_value);
	}

	if (i == key_depth - 1) {
		/* Found either an internal, external node or NULL at end of key. */
		if (!ft_node_ptr(node_flag)) {
			dbg_printf("cds_ft_insert NULL parent2_node_flag %p parent_node_flag %p node_flag_ptr %p node_flag %p\n",
					parent2_node_flag, parent_node_flag, node_flag_ptr, node_flag);

			attach_node_flag = parent_node_flag;
			attach_node_flag_ptr = parent_node_flag_ptr;

			ret = ft_attach_node(ft, attach_node_flag_ptr, attach_node_flag,
					node_flag_ptr, node_flag, key, key_len, i, node,
					NULL);

		} else if (ft_node_internal(node_flag)) {
			struct cds_ft_node *external_nodes;
			struct cds_ft_metadata *metadata;

			metadata = cds_ft_item_to_metadata(ft_node_ptr(node_flag));
			external_nodes = metadata->external_nodes;
			if (external_nodes) {
				struct cds_ft_node *iter_node, *last_node = NULL;

				if (unique_node_ret) {
					*unique_node_ret = external_nodes;
					return -EEXIST;
				}
				/* Find last duplicate */
				iter_node = external_nodes;
				cds_ft_for_each_duplicate(iter_node)
					last_node = iter_node;

				dbg_printf("cds_ft_insert duplicate internal parent2_node_flag %p parent_node_flag %p node_flag_ptr %p node_flag %p\n",
						parent2_node_flag, parent_node_flag, node_flag_ptr, node_flag);

				ft_chain_node(last_node, node);
				ret = 0;
			} else {
				node->next = NULL;
				rcu_assign_pointer(metadata->external_nodes, node);
				ret = 0;
			}
		} else {
			struct cds_ft_node *iter_node, *last_node = NULL;

			if (unique_node_ret) {
				*unique_node_ret = (struct cds_ft_node *) ft_node_ptr(node_flag);
				return -EEXIST;
			}
			/* Find last duplicate */
			iter_node = (struct cds_ft_node *) ft_node_ptr(node_flag);
			cds_ft_for_each_duplicate(iter_node)
				last_node = iter_node;

			dbg_printf("cds_ft_insert duplicate external parent2_node_flag %p parent_node_flag %p node_flag_ptr %p node_flag %p\n",
					parent2_node_flag, parent_node_flag, node_flag_ptr, node_flag);

			ft_chain_node(last_node, node);
			ret = 0;
		}
	} else {
		/* Found NULL node or external node before end of key. */

		/*
		 * If the last node encountered during traversal is an external node,
		 * transform this external node into an internal node with associated
		 * external node, attach a new cluster as child of this internal node, and
		 * populate this new internal node into the tree to replace the prior
		 * external node.
		 * It's the same for NULL node, only that there is no need to chain any
		 * external node.
		 */

		dbg_printf("cds_ft_insert NULL or external parent2_node_flag %p parent_node_flag %p node_flag_ptr %p node_flag %p\n",
				parent2_node_flag, parent_node_flag, node_flag_ptr, node_flag);

		attach_node_flag = parent_node_flag;
		attach_node_flag_ptr = parent_node_flag_ptr;

		ret = ft_attach_node(ft, attach_node_flag_ptr, attach_node_flag,
				node_flag_ptr, node_flag, key, key_len, i, node,
				(struct cds_ft_node *) ft_node_ptr(node_flag));
	}

	if (ret == 0) {
		if (key_len > uatomic_load(&ft->max_used_key_len, CMM_RELAXED))
			uatomic_store(&ft->max_used_key_len, key_len, CMM_RELAXED);
	}

	return ret;
}

enum cds_ft_status cds_ft_insert(struct cds_ft *ft,
		const uint8_t *key, size_t key_len,
		struct cds_ft_node *node)
{
	int ret = _cds_ft_insert(ft, key, key_len, node, NULL);

	if (ret == 0)
		return CDS_FT_STATUS_OK;
	if (ret == -EINVAL)
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	return CDS_FT_STATUS_MEMORY_ERROR;
}

enum cds_ft_status cds_ft_insert_unique(struct cds_ft *ft,
		const uint8_t *key, size_t key_len,
		struct cds_ft_node *node,
		struct cds_ft_node **result_node)
{
	int ret;
	struct cds_ft_node *ret_node = NULL;

	ret = _cds_ft_insert(ft, key, key_len, node, &ret_node);
	if (ret == -EEXIST) {
		*result_node = ret_node;
		return CDS_FT_STATUS_DUPLICATE_FOUND;
	}
	if (ret == -EINVAL) {
		*result_node = NULL;
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}
	if (ret) {
		*result_node = NULL;
		return CDS_FT_STATUS_MEMORY_ERROR;
	}
	*result_node = node;
	return CDS_FT_STATUS_OK;
}

/*
 * Insert a node, replacing the entire existing duplicate chain at the
 * same key if one exists.
 *
 * On success, *@old_node_ret is set to the head of the replaced chain
 * (or NULL if no prior node existed). The caller must wait for a grace
 * period before reclaiming the old chain.
 *
 * Returns 0 on success, -EINVAL on bad arguments, or a negative errno
 * on memory allocation failure.
 */
static
int _cds_ft_insert_replace(struct cds_ft *ft,
		const uint8_t *key, size_t _key_len,
		struct cds_ft_node *node,
		struct cds_ft_node **old_node_ret)
{
	unsigned int i, key_depth;
	struct cds_ft_inode_flag *attach_node_flag, *parent_node_flag,
		*parent2_node_flag, *node_flag;
	struct cds_ft_inode_flag **attach_node_flag_ptr,
		**parent_node_flag_ptr, **node_flag_ptr;
	const uint8_t *iter_key;
	size_t key_len = ft_key_len(ft, _key_len);
	int ret;

	*old_node_ret = NULL;

	if (!valid_external_node(node) || !valid_key_len(ft, key_len))
		return -EINVAL;
	/* Expect zeroed next pointer. */
	if (node->next)
		return -EINVAL;

	key_depth = key_len + 1;

	dbg_printf("_cds_ft_insert_replace attempt: node %p\n", node);
	iter_key = key;
	parent2_node_flag = NULL;
	parent_node_flag = NULL;
	parent_node_flag_ptr = NULL;
	node_flag = ft->root;
	node_flag_ptr = &ft->root;

	for (i = 0; i < key_depth - 1; i++) {
		uint8_t key_value;

		if (!ft_node_ptr(node_flag))
			break;
		/* Found external node. */
		if (!ft_node_internal(node_flag))
			break;
		dbg_printf("_cds_ft_insert_replace iter parent2_node_flag %p parent_node_flag %p node_flag_ptr %p node_flag %p\n",
				parent2_node_flag, parent_node_flag, node_flag_ptr, node_flag);
		key_value = key_to_ordinal(ft, *(iter_key++));
		parent2_node_flag = parent_node_flag;
		parent_node_flag = node_flag;
		parent_node_flag_ptr = node_flag_ptr;
		node_flag = ft_node_get_nth(node_flag, &node_flag_ptr, key_value);
	}

	if (i == key_depth - 1) {
		/* Found either an internal, external node or NULL at end of key. */
		if (!ft_node_ptr(node_flag)) {
			/* No existing node. Regular attach. */
			dbg_printf("_cds_ft_insert_replace NULL at end of key\n");

			attach_node_flag = parent_node_flag;
			attach_node_flag_ptr = parent_node_flag_ptr;

			ret = ft_attach_node(ft, attach_node_flag_ptr, attach_node_flag,
					node_flag_ptr, node_flag, key, key_len, i, node,
					NULL);
		} else if (ft_node_internal(node_flag)) {
			struct cds_ft_node *external_nodes;
			struct cds_ft_metadata *metadata;

			metadata = cds_ft_item_to_metadata(ft_node_ptr(node_flag));
			external_nodes = metadata->external_nodes;
			if (external_nodes) {
				dbg_printf("_cds_ft_insert_replace: replacing internal metadata chain %p\n",
						external_nodes);
				/* Replace entire existing chain. */
				*old_node_ret = external_nodes;
				node->next = NULL;
				rcu_assign_pointer(metadata->external_nodes, node);
			} else {
				/* No external nodes yet. Fresh insert. */
				node->next = NULL;
				rcu_assign_pointer(metadata->external_nodes, node);
			}
			ret = 0;
		} else {
			dbg_printf("_cds_ft_insert_replace: replacing external chain %p\n",
					ft_node_ptr(node_flag));
			/* External node at end of key. Replace chain. */
			*old_node_ret = (struct cds_ft_node *) ft_node_ptr(node_flag);
			node->next = NULL;
			rcu_assign_pointer(*node_flag_ptr, (struct cds_ft_inode_flag *) node);
			ret = 0;
		}
	} else {
		/*
		 * Found NULL node or external node before end of key.
		 * Attach a new branch, displacing any shorter-key
		 * external node into the new branch's metadata.
		 */
		dbg_printf("_cds_ft_insert_replace: attach before end of key\n");

		attach_node_flag = parent_node_flag;
		attach_node_flag_ptr = parent_node_flag_ptr;

		ret = ft_attach_node(ft, attach_node_flag_ptr, attach_node_flag,
				node_flag_ptr, node_flag, key, key_len, i, node,
				(struct cds_ft_node *) ft_node_ptr(node_flag));
	}

	if (ret == 0) {
		if (key_len > uatomic_load(&ft->max_used_key_len, CMM_RELAXED))
			uatomic_store(&ft->max_used_key_len, key_len, CMM_RELAXED);
	}

	return ret;
}

enum cds_ft_status cds_ft_insert_replace(struct cds_ft *ft,
		const uint8_t *key, size_t key_len,
		struct cds_ft_node *node,
		struct cds_ft_node **result_node)
{
	struct cds_ft_node *old_node = NULL;
	int ret;

	ret = _cds_ft_insert_replace(ft, key, key_len, node, &old_node);
	if (ret == -EINVAL) {
		*result_node = NULL;
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}
	if (ret) {
		*result_node = NULL;
		return CDS_FT_STATUS_MEMORY_ERROR;
	}
	*result_node = old_node;
	if (old_node) {
		return CDS_FT_STATUS_DUPLICATE_FOUND;
	}
	return CDS_FT_STATUS_OK;
}

enum cds_ft_status cds_ft_replace(struct cds_ft *ft,
		struct cds_ft_iter *iter,
		struct cds_ft_node *old_node,
		struct cds_ft_node *new_node)
{
	unsigned int i, key_depth;
	struct cds_ft_inode_flag *node_flag;
	struct cds_ft_inode_flag **node_flag_ptr;
	struct cds_ft_node *iter_node, **iter_node_ptr, **prev_node_ptr, *match;
	const uint8_t *iter_key;
	size_t key_len = ft_key_len(ft, iter->key_len);

	if (!valid_external_node(old_node) || !valid_external_node(new_node)
			|| !valid_key_len(ft, key_len))
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	/* Expect zeroed next pointer on new_node. */
	if (new_node->next)
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;

	key_depth = key_len + 1;
	iter_key = iter->key;

	dbg_printf("cds_ft_replace: old_node %p new_node %p\n", old_node, new_node);

	node_flag = rcu_dereference(ft->root);
	node_flag_ptr = &ft->root;

	/* Root is always present and always internal. */

	/*
	 * Handle NIL key (key_len == 0).
	 */
	if (!key_len) {
		struct cds_ft_metadata *metadata;

		metadata = cds_ft_item_to_metadata(ft_node_ptr(node_flag));
		if (!metadata->external_nodes)
			return CDS_FT_STATUS_NOT_FOUND;
		iter_node_ptr = (struct cds_ft_node **) &metadata->external_nodes;
		iter_node = metadata->external_nodes;
		goto find_and_replace;
	}

	/* Traverse internal levels. */
	for (i = 1; i < key_depth; i++) {
		uint8_t key_value;

		if (!ft_node_internal(node_flag))
			return CDS_FT_STATUS_NOT_FOUND;
		key_value = key_to_ordinal(ft, *(iter_key++));
		node_flag = ft_node_get_nth(node_flag, &node_flag_ptr, key_value);
		if (!ft_node_ptr(node_flag))
			return CDS_FT_STATUS_NOT_FOUND;
	}

	/* Reached end of key. Locate the duplicate chain. */
	if (ft_node_internal(node_flag)) {
		struct cds_ft_metadata *metadata;

		metadata = cds_ft_item_to_metadata(ft_node_ptr(node_flag));
		if (!metadata->external_nodes)
			return CDS_FT_STATUS_NOT_FOUND;
		iter_node_ptr = (struct cds_ft_node **) &metadata->external_nodes;
		iter_node = metadata->external_nodes;
	} else {
		iter_node_ptr = (struct cds_ft_node **) node_flag_ptr;
		iter_node = (struct cds_ft_node *) ft_node_ptr(node_flag);
	}

find_and_replace:
	/*
	 * Walk the duplicate chain to find old_node and track the
	 * pointer that references it (prev_node_ptr).
	 */
	prev_node_ptr = NULL;
	match = NULL;
	cds_ft_for_each_duplicate(iter_node) {
		if (match)
			continue;
		if (iter_node == old_node) {
			prev_node_ptr = iter_node_ptr;
			match = iter_node;
		}
		iter_node_ptr = &iter_node->next;
	}

	if (!match)
		return CDS_FT_STATUS_NOT_FOUND;

	/*
	 * Splice new_node into the chain in place of old_node.
	 * new_node inherits old_node's successor. The write barrier
	 * within rcu_assign_pointer ensures new_node->next is visible
	 * before the pointer that publishes new_node.
	 */
	new_node->next = old_node->next;
	rcu_assign_pointer(*prev_node_ptr, new_node);

	/*
	 * The trie structure is unchanged (no recompaction), so the
	 * iterator path remains valid.
	 */
	return CDS_FT_STATUS_OK;
}

/*
 * Note: there is no need to lookup the pointer address associated with
 * each node's nth item: it's already been done by cds_ft_remove, and
 * cds_ft_remove is protected by mutual exclusion of updaters.
 *
 * ft_detach_node() ensures that a lookup will _never_ see a branch that
 * leads to a dead-end: when removing branch, it makes sure to perform
 * the "cut" at the highest node that has only one child, effectively
 * replacing it with a NULL pointer.
 *
 * Internal nodes are considered empty if they have no internal and no
 * external node children, *and* their associated list of external nodes
 * is empty. When detaching an internal node which has no children, but
 * has an associated list of external nodes, it is replaced by a pointer
 * to the external nodes.
 *
 * During descent, the detach point pointers are updated when:
 * - A node with nr_child > 1 is encountered (direct termination point
 *   for the upward walk).
 * - A single-child node with external_nodes is encountered (triggers
 *   termination one level above via prev_external_nodes_found).
 * - Root level (always a termination point via i == 0).
 *
 * The last update during top-down descent corresponds to the deepest
 * level where the bottom-up walk would terminate, so the pre-tracked
 * pointers always match the termination level.
 */
static
int ft_detach_node(struct cds_ft *ft,
		struct cds_ft_inode_flag **snapshot,
		uint8_t *snapshot_n,
		int nr_snapshot,
		struct cds_ft_inode_flag **detach_node_flag_ptr,
		struct cds_ft_inode_flag **detach_parent_flag_ptr)
{
	struct cds_ft_metadata *metadata_stack[FT_MAX_DEPTH];
	struct cds_ft_inode_flag *iter_node_flag;
	struct cds_ft_inode *old_recompacted_node = NULL;
	int ret, i, nr_metadata = 0, nr_clear = 0, nr_branch = 0;
	uint8_t n = 0;
	struct cds_ft_node *topmost_external_nodes = NULL;
	bool prev_external_nodes_found = false;

	/*
	 * From the last internal level node going up, lookup the
	 * metadata, check if the node has only one child left. If it is
	 * the case, we continue iterating upward. When we reach a node
	 * which has more that one child left or has an associated
	 * external node, we lookup the parent, and proceed to the node
	 * deletion (removing its children too), replacing it with its
	 * external node pointer (if any).
	 */
	for (i = nr_snapshot - 2; i >= 0; i--) {
		struct cds_ft_metadata *metadata;

		metadata = cds_ft_item_to_metadata(ft_node_ptr(snapshot[i]));
		metadata_stack[nr_metadata++] = metadata;

		assert(metadata->nr_child > 0);
		if (!prev_external_nodes_found && (metadata->nr_child == 1 && i > 0)) {
			nr_clear++;
			/*
			 * Keep track of the external nodes pointer of
			 * the topmost internal node in the branch.
			 */
			topmost_external_nodes = metadata->external_nodes;
		}
		nr_branch++;
		if (prev_external_nodes_found || metadata->nr_child > 1 || i == 0) {
			if (i > 0) {
				metadata = cds_ft_item_to_metadata(ft_node_ptr(snapshot[i - 1]));
			}
			/*
			 * When i == 0 we are at the root.  The root's own
			 * metadata is already in metadata_stack (just pushed
			 * above); we reuse it as the "parent" metadata for
			 * the replace_ptr call below.
			 */
			if (i > 0)
				metadata_stack[nr_metadata++] = metadata;

			n = snapshot_n[i + 1];
			break;
		}
		if (topmost_external_nodes)
			prev_external_nodes_found = true;
	}

	iter_node_flag = *detach_parent_flag_ptr;
	/* Replace within parent */
	ret = ft_node_replace_ptr(ft,
		detach_node_flag_ptr,	/* Pointer to location to nullify */
		&iter_node_flag,	/* Old new parent ptr in its parent */
		&old_recompacted_node,
		metadata_stack[nr_branch - 1],	/* of parent */
		n, (struct cds_ft_inode_flag *) topmost_external_nodes,
		detach_parent_flag_ptr == &ft->root);
	if (ret)
		goto end;

	dbg_printf("ft_detach_node: publish %p instead of %p\n",
		iter_node_flag, *detach_parent_flag_ptr);

	/* Update address of parent ptr in its parent */
	rcu_assign_pointer(*detach_parent_flag_ptr, iter_node_flag);
end:
	/* Reclaim safely after replacement.*/
	if (old_recompacted_node)
		free_cds_ft_node(ft, old_recompacted_node);

	/*
	 * At this point, we want to delete all nodes that are about to
	 * be removed from metadata_stack (except the last one, which is
	 * the parent of the topmost node with 1 child, or the root
	 * itself when the entire branch goes up to the root).
	 */
	for (i = 0; i < nr_clear; i++)
		free_cds_ft_node(ft, cds_ft_metadata_to_item(metadata_stack[i]));

	return ret;
}

static
void ft_unchain_node(struct cds_ft_node **prev_node_ptr,
		struct cds_ft_node *node)
{
	uatomic_store(prev_node_ptr, node->next, CMM_RELAXED);
}

/*
 * Called with RCU read lock held.
 *
 * There are a few cases to cover for delete:
 *
 * 1) The node belongs to a list of external nodes duplicates with two
 *    or more items. Remove the node by unlinking it from its list.
 * 2) There is only one external node within this node's list.
 *    2.1) The node is within an external nodes list for which the list
 *         head is an standalone external nodes pointer. The external
 *         nodes list for this key should be removed. Removing an
 *         external nodes list should prune the entire branch leading to
 *         that list so no lookup observe empty internal nodes. This is
 *         done by ft_detach_node(). Internal nodes are considered empty
 *         if they have no internal and no external node children, *and*
 *         their associated list of external nodes is empty. When
 *         detaching an internal node which has no children, but has
 *         an associated list of external nodes, it is replaced by a
 *         pointer to the external nodes.
 *    2.2) The node is within an external nodes list which is associated
 *         with an internal node. Unlink the node from its list, leaving
 *         the external nodes list empty.
 */
enum cds_ft_status cds_ft_remove(struct cds_ft *ft,
		struct cds_ft_iter *iter,
		struct cds_ft_node *node)
{
	unsigned int i, key_depth;
	struct cds_ft_inode_flag *snapshot[FT_MAX_DEPTH];
	uint8_t snapshot_n[FT_MAX_DEPTH];
	struct cds_ft_inode_flag *node_flag;
	struct cds_ft_inode_flag **node_flag_ptr;
	/*
	 * Detach point pointers tracked during descent. These are
	 * updated at potential upward-walk termination points.
	 */
	struct cds_ft_inode_flag **detach_node_flag_ptr,
			**detach_parent_flag_ptr;
	struct cds_ft_inode_flag **pp_flag_ptr, **p_flag_ptr;
	bool pending_detach_node;
	struct cds_ft_node *iter_node, **iter_node_ptr, **prev_node_ptr, *match;
	int nr_snapshot, ret, count = 0;
	const uint8_t *iter_key;
	size_t key_len = ft_key_len(ft, iter->key_len);

	if (!valid_external_node(node) || !valid_key_len(ft, key_len))
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;

	key_depth = key_len + 1;

	nr_snapshot = 0;
	iter_key = iter->key;
	dbg_printf("cds_ft_remove attempt: node %p\n", node);

	node_flag = rcu_dereference(ft->root);
	node_flag_ptr = &ft->root;

	/*
	 * Initialize detach tracking for root level. The root always
	 * terminates the upward walk (i == 0), so it serves as the
	 * default detach point. detach_node_flag_ptr is set after the
	 * first successful get_nth (pending).
	 */
	pp_flag_ptr = NULL;
	p_flag_ptr = &ft->root;
	detach_parent_flag_ptr = &ft->root;
	detach_node_flag_ptr = NULL;
	pending_detach_node = true;

	/* Iterate on all internal levels */
	for (i = 1; i < key_depth; i++) {
		uint8_t key_value;
		const struct cds_ft_metadata *metadata;

		dbg_printf("cds_ft_remove iter node_flag %p\n",
				node_flag);
		if (!ft_node_ptr(node_flag)) {
			return CDS_FT_STATUS_NOT_FOUND;
		}

		/*
		 * Track pointers for the detach point during descent.
		 * Update when encountering a potential upward-walk
		 * termination point.
		 */
		metadata = cds_ft_item_to_metadata(ft_node_ptr(node_flag));
		if (metadata->nr_child > 1) {
			detach_parent_flag_ptr = p_flag_ptr;
			pending_detach_node = true;
		} else if (i > 1 && metadata->external_nodes) {
			/*
			 * Single-child node with external_nodes: the
			 * upward walk terminates one level above via
			 * prev_external_nodes_found, so save the
			 * pointers from the previous level.
			 */
			detach_node_flag_ptr = p_flag_ptr;
			detach_parent_flag_ptr = pp_flag_ptr;
			pending_detach_node = false;
		}

		key_value = key_to_ordinal(ft, *(iter_key++));
		snapshot_n[nr_snapshot + 1] = key_value;
		snapshot[nr_snapshot++] = node_flag;
		node_flag = ft_node_get_nth(node_flag, &node_flag_ptr, key_value);
		if (node_flag) {
			pp_flag_ptr = p_flag_ptr;
			p_flag_ptr = node_flag_ptr;
			if (pending_detach_node) {
				detach_node_flag_ptr = node_flag_ptr;
				pending_detach_node = false;
			}
		}
		dbg_printf("cds_ft_remove iter key lookup %u finds node_flag %p, node_flag_ptr %p\n",
				(unsigned int) key_value, node_flag,
				node_flag_ptr);
	}
	/*
	 * We reached end of key, try to find the node we are trying to
	 * remove. Fail if we cannot find it.
	 */
	if (!ft_node_ptr(node_flag)) {
		dbg_printf("cds_ft_remove: no node found for key\n");
		return CDS_FT_STATUS_NOT_FOUND;
	}

	if (ft_node_internal(node_flag)) {
		/* Found internal node at end of key. */
		struct cds_ft_node *external_nodes;
		struct cds_ft_metadata *metadata;

		metadata = cds_ft_item_to_metadata(ft_node_ptr(node_flag));
		external_nodes = metadata->external_nodes;
		if (external_nodes) {
			/*
			 * Find the previous node's next pointer pointing to our node,
			 * so we can update it.
			 */
			prev_node_ptr = NULL;
			iter_node_ptr = (struct cds_ft_node **) &metadata->external_nodes;
			iter_node = (struct cds_ft_node *) external_nodes;
			match = NULL;
			cds_ft_for_each_duplicate(iter_node) {
				if (match)
					continue;
				dbg_printf("cds_ft_remove: compare %p with iter_node %p\n", node, iter_node);
				if (iter_node == node) {
					prev_node_ptr = iter_node_ptr;
					match = iter_node;
				}
				iter_node_ptr = &iter_node->next;
			}
			if (!match) {
				dbg_printf("cds_ft_remove: no node match for node %p key\n", node);
				return CDS_FT_STATUS_NOT_FOUND;
			}
			ft_unchain_node(prev_node_ptr, match);
			ret = 0;
		} else {
			dbg_printf("cds_ft_remove: no metadata external node found for key\n");
			return CDS_FT_STATUS_NOT_FOUND;
		}
	} else {
		/* Found external node at end of key. */

		/*
		 * Find the previous node's next pointer pointing to our node,
		 * so we can update it.
		 */
		prev_node_ptr = NULL;
		iter_node_ptr = (struct cds_ft_node **) node_flag_ptr;
		iter_node = (struct cds_ft_node *) ft_node_ptr(node_flag);
		count = 0;
		match = NULL;
		cds_ft_for_each_duplicate(iter_node) {
			count++;
			if (match)
				continue;
			dbg_printf("cds_ft_remove: compare %p with iter_node %p\n", node, iter_node);
			if (iter_node == node) {
				prev_node_ptr = iter_node_ptr;
				match = iter_node;
			}
			iter_node_ptr = &iter_node->next;
		}
		if (!match) {
			dbg_printf("cds_ft_remove: no node match for node %p key\n", node);
			return CDS_FT_STATUS_NOT_FOUND;
		}
		assert(count > 0);
		if (count == 1) {
			/*
			 * Removing last of duplicates. Last snapshot
			 * does not have metadata (external leafs).
			 */
			snapshot[nr_snapshot++] = node_flag;
			ret = ft_detach_node(ft, snapshot,
					snapshot_n, nr_snapshot,
					detach_node_flag_ptr,
					detach_parent_flag_ptr);
		} else {
			ft_unchain_node(prev_node_ptr, match);
			ret = 0;
		}
	}

	/*
	 * detach should not replace a NULL pointer because it has been
	 * found by a mutex-protected traversal within this function.
	 */
	assert(ret != -ENOENT);

	/*
	 * Invalidate the iterator path. The trie structure may have
	 * changed due to node recompaction during detach, making the
	 * cached path stale.
	 */
	iter->path_valid = false;
	iter->path_len = 0;

	switch (ret) {
	case 0:
		return CDS_FT_STATUS_OK;
	case -ENOMEM:
		return CDS_FT_STATUS_MEMORY_ERROR;
	default:
		abort();
	}
}

enum cds_ft_status cds_ft_remove_all(struct cds_ft *ft,
		struct cds_ft_iter *iter,
		struct cds_ft_node **result_node)
{
	unsigned int i, key_depth;
	struct cds_ft_inode_flag *snapshot[FT_MAX_DEPTH];
	uint8_t snapshot_n[FT_MAX_DEPTH];
	struct cds_ft_inode_flag *node_flag;
	struct cds_ft_inode_flag **node_flag_ptr;
	struct cds_ft_inode_flag **detach_node_flag_ptr,
			**detach_parent_flag_ptr;
	struct cds_ft_inode_flag **pp_flag_ptr, **p_flag_ptr;
	bool pending_detach_node;
	int nr_snapshot, ret;
	const uint8_t *iter_key;
	size_t key_len = ft_key_len(ft, iter->key_len);

	if (!valid_key_len(ft, key_len)) {
		*result_node = NULL;
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}

	key_depth = key_len + 1;

	/*
	 * Handle NIL key (key_len == 0): root is always internal,
	 * remove its external_nodes chain.
	 */
	if (!key_len) {
		struct cds_ft_metadata *metadata;
		struct cds_ft_node *external_nodes;

		metadata = ft_root_metadata(ft);
		external_nodes = metadata->external_nodes;
		if (!external_nodes) {
			*result_node = NULL;
			return CDS_FT_STATUS_NOT_FOUND;
		}
		*result_node = external_nodes;
		rcu_assign_pointer(metadata->external_nodes, NULL);
		return CDS_FT_STATUS_OK;
	}

	nr_snapshot = 0;
	iter_key = iter->key;
	dbg_printf("cds_ft_remove_all attempt\n");

	node_flag = rcu_dereference(ft->root);
	node_flag_ptr = &ft->root;

	pp_flag_ptr = NULL;
	p_flag_ptr = &ft->root;
	detach_parent_flag_ptr = &ft->root;
	detach_node_flag_ptr = NULL;
	pending_detach_node = true;

	/* Iterate on all internal levels. */
	for (i = 1; i < key_depth; i++) {
		uint8_t key_value;
		const struct cds_ft_metadata *metadata;

		dbg_printf("cds_ft_remove_all iter node_flag %p\n", node_flag);
		if (!ft_node_ptr(node_flag)) {
			*result_node = NULL;
			return CDS_FT_STATUS_NOT_FOUND;
		}

		/* Track detach point pointers during descent. */
		metadata = cds_ft_item_to_metadata(ft_node_ptr(node_flag));
		if (metadata->nr_child > 1) {
			detach_parent_flag_ptr = p_flag_ptr;
			pending_detach_node = true;
		} else if (i > 1 && metadata->external_nodes) {
			detach_node_flag_ptr = p_flag_ptr;
			detach_parent_flag_ptr = pp_flag_ptr;
			pending_detach_node = false;
		}

		key_value = key_to_ordinal(ft, *(iter_key++));
		snapshot_n[nr_snapshot + 1] = key_value;
		snapshot[nr_snapshot++] = node_flag;
		node_flag = ft_node_get_nth(node_flag, &node_flag_ptr, key_value);
		if (node_flag) {
			pp_flag_ptr = p_flag_ptr;
			p_flag_ptr = node_flag_ptr;
			if (pending_detach_node) {
				detach_node_flag_ptr = node_flag_ptr;
				pending_detach_node = false;
			}
		}
		dbg_printf("cds_ft_remove_all iter key lookup %u finds node_flag %p, node_flag_ptr %p\n",
				(unsigned int) key_value, node_flag, node_flag_ptr);
	}

	/* Reached end of key. */
	if (!ft_node_ptr(node_flag)) {
		dbg_printf("cds_ft_remove_all: no node found for key\n");
		*result_node = NULL;
		return CDS_FT_STATUS_NOT_FOUND;
	}

	if (ft_node_internal(node_flag)) {
		/* Internal node at end of key. Remove all external nodes from metadata. */
		struct cds_ft_node *external_nodes;
		struct cds_ft_metadata *metadata;

		metadata = cds_ft_item_to_metadata(ft_node_ptr(node_flag));
		external_nodes = metadata->external_nodes;
		if (!external_nodes) {
			dbg_printf("cds_ft_remove_all: no metadata external nodes for key\n");
			*result_node = NULL;
			return CDS_FT_STATUS_NOT_FOUND;
		}
		/*
		 * Atomically remove the entire chain. The internal
		 * node itself remains (it still has children). A grace
		 * period must be observed before reclaiming any node
		 * in the old chain.
		 */
		*result_node = external_nodes;
		rcu_assign_pointer(metadata->external_nodes, NULL);
		ret = 0;
	} else {
		/*
		 * External node at end of key. Detach the branch.
		 * This is the same as removing the last duplicate in
		 * cds_ft_remove().
		 */
		*result_node = (struct cds_ft_node *) ft_node_ptr(node_flag);
		snapshot[nr_snapshot++] = node_flag;
		ret = ft_detach_node(ft, snapshot,
				snapshot_n, nr_snapshot,
				detach_node_flag_ptr,
				detach_parent_flag_ptr);
	}

	/*
	 * detach should not replace a NULL pointer because it has been
	 * found by a mutex-protected traversal within this function.
	 */
	assert(ret != -ENOENT);

	iter->path_valid = false;
	iter->path_len = 0;

	if (ret)
		return CDS_FT_STATUS_NOT_FOUND;

	return CDS_FT_STATUS_OK;
}

/*
 * Graft-point descriptor returned by ft_descend_to_graft_point().
 */
struct ft_graft_point {
	unsigned int depth;		/* Depth reached (0..key_len). */
	struct cds_ft_inode_flag *nf;	/* Node flag at the graft slot. */
	struct cds_ft_inode_flag **nfp;	/* Pointer to the graft slot. */
	struct cds_ft_inode_flag *pnf;	/* Parent node flag. */
	struct cds_ft_inode_flag **pnfp;/* Pointer to parent in its parent. */
};

/*
 * Descend through the trie to the child slot at depth @key_len.
 * Stops early if the traversal hits NULL or an external node.
 *
 * On return, gp->depth is the number of internal levels successfully
 * traversed (0..key_len).  If gp->depth == key_len, the graft slot
 * is at gp->nf / gp->nfp.  Otherwise the path was incomplete.
 *
 * Used by cds_ft_graft and cds_ft_graft_swap.  cds_ft_detach uses
 * its own descent loop because it records snapshot state for
 * ft_detach_node's upward pruning walk.
 */
static
void ft_descend_to_graft_point(struct cds_ft *ft,
		const uint8_t *key, size_t key_len,
		struct ft_graft_point *gp)
{
	unsigned int i;
	const uint8_t *ik = key;

	gp->nf = rcu_dereference(ft->root);
	gp->nfp = &ft->root;
	gp->pnf = NULL;
	gp->pnfp = NULL;

	for (i = 0; i < key_len; i++) {
		uint8_t kv;

		if (!ft_node_ptr(gp->nf) || !ft_node_internal(gp->nf))
			break;

		kv = key_to_ordinal(ft, *(ik++));
		gp->pnf = gp->nf;
		gp->pnfp = gp->nfp;
		gp->nf = ft_node_get_nth(gp->nf, &gp->nfp, kv);
	}
	gp->depth = i;
}

/*
 * Build a single-child chain of internal nodes for
 * key[start .. end-1] with @leaf at the bottom.  Returns the topmost
 * flagged node, or NULL on allocation failure (all nodes freed).
 */
static
struct cds_ft_inode_flag *ft_build_branch(struct cds_ft *ft,
		const uint8_t *key, unsigned int start, unsigned int end,
		struct cds_ft_inode_flag *leaf)
{
	struct cds_ft_inode_flag *created[FT_MAX_DEPTH];
	struct cds_ft_inode_flag *cur = leaf;
	int nr = 0, j, ret;
	int i;

	for (i = (int) end - 1; i >= (int) start; i--) {
		struct cds_ft_inode_flag *dest = NULL;
		uint8_t kv = key_to_ordinal(ft, key[i]);

		ret = ft_node_set_nth(ft, &dest, kv, cur, NULL, NULL);
		if (ret) {
			for (j = 0; j < nr; j++)
				free_cds_ft_node(ft, ft_node_ptr(created[j]));
			return NULL;
		}
		created[nr++] = dest;
		cur = dest;
	}
	return cur;
}

/*
 * Store graft_payload at the graft point described by @gp.
 *
 * Handles two cases:
 * - gp->depth == key_len: the slot exists; add via ft_node_set_nth.
 * - gp->depth < key_len: the path is incomplete; build intermediate
 *   internal nodes via ft_build_branch, displacing any external node
 *   on the path into the branch's metadata.
 *
 * Return CDS_FT_STATUS_OK on success, CDS_FT_STATUS_POPULATED_ERROR
 * if the slot is already occupied, CDS_FT_STATUS_MEMORY_ERROR on
 * allocation failure.
 */
static
enum cds_ft_status ft_store_at_graft_point(struct cds_ft *ft,
		const uint8_t *key, size_t key_len,
		struct ft_graft_point *gp,
		struct cds_ft_inode_flag *graft_payload)
{
	struct cds_ft_inode *old_recompacted_node = NULL;

	if (gp->depth == key_len) {
		struct cds_ft_metadata *pmeta;
		struct cds_ft_inode_flag *dest;
		int ret;

		if (ft_node_ptr(gp->nf))
			return CDS_FT_STATUS_POPULATED_ERROR;

		pmeta = cds_ft_item_to_metadata(ft_node_ptr(gp->pnf));

		dest = gp->pnf;
		ret = ft_node_set_nth(ft, &dest,
			key_to_ordinal(ft, key[key_len - 1]),
			graft_payload, &old_recompacted_node, pmeta);
		if (ret)
			return CDS_FT_STATUS_MEMORY_ERROR;

		rcu_assign_pointer(*gp->pnfp, dest);

		if (old_recompacted_node)
			free_cds_ft_node(ft, old_recompacted_node);
	} else {
		unsigned int i = gp->depth;
		struct cds_ft_inode_flag *branch;
		struct cds_ft_node *displaced = NULL;

		if (ft_node_ptr(gp->nf) && !ft_node_internal(gp->nf))
			displaced = (struct cds_ft_node *)
				ft_node_ptr(gp->nf);

		branch = ft_build_branch(ft, key, i, key_len, graft_payload);
		if (!branch)
			return CDS_FT_STATUS_MEMORY_ERROR;

		if (displaced) {
			struct cds_ft_metadata *bm =
				cds_ft_item_to_metadata(
					ft_node_ptr(branch));
			bm->external_nodes = displaced;
		}

		if (displaced) {
			rcu_assign_pointer(*gp->nfp, branch);
		} else {
			struct cds_ft_inode_flag *dest = gp->pnf;
			struct cds_ft_metadata *pmeta;
			int ret;

			pmeta = cds_ft_item_to_metadata(
					ft_node_ptr(gp->pnf));

			ret = ft_node_set_nth(ft, &dest,
				key_to_ordinal(ft, key[i - 1]),
				branch, &old_recompacted_node, pmeta);
			if (ret)
				return CDS_FT_STATUS_MEMORY_ERROR;

			rcu_assign_pointer(*gp->pnfp, dest);

			if (old_recompacted_node)
				free_cds_ft_node(ft, old_recompacted_node);
		}
	}
	return CDS_FT_STATUS_OK;
}

enum cds_ft_status cds_ft_graft(struct cds_ft *dst_ft,
		const uint8_t *key, size_t _key_len,
		struct cds_ft *src_ft)
{
	struct cds_ft_metadata *src_rmeta;
	size_t key_len, src_max;
	enum cds_ft_status status;

	if (!dst_ft || !src_ft || dst_ft == src_ft)
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	if (dst_ft->group != src_ft->group)
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;

	/*
	 * Root-level graft (key_len == 0) is valid for both
	 * variable-length and fixed-length groups: it swaps the entire
	 * root, so no key-length constraint applies.  Bypass
	 * ft_key_len() which would reject 0 != fixed_len.
	 */
	if (_key_len == 0) {
		key_len = 0;
	} else {
		key_len = ft_key_len(dst_ft, _key_len);
		if (!valid_key_len(dst_ft, key_len) ||
				dst_ft->group->key_len != CDS_FT_LEN_VARIABLE)
			return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}

	src_max = uatomic_load(&src_ft->max_used_key_len, CMM_RELAXED);
	if (key_len > 0 && src_max > dst_ft->group->max_key_len - key_len)
		return CDS_FT_STATUS_OVERFLOW_ERROR;

	src_rmeta = ft_root_metadata(src_ft);

	/* Check if source trie is empty. */
	if (src_rmeta->nr_child == 0 && !src_rmeta->external_nodes)
		return CDS_FT_STATUS_OK;

	if (key_len == 0) {
		struct cds_ft_metadata *dst_rmeta = ft_root_metadata(dst_ft);
		struct cds_ft_inode *fresh_root;
		struct cds_ft_metadata *fresh_meta;

		/* Destination must be empty for a root-level graft. */
		if (dst_rmeta->nr_child != 0 || dst_rmeta->external_nodes)
			return CDS_FT_STATUS_POPULATED_ERROR;

		/*
		 * Allocate a fresh empty root for the source before
		 * swapping, so the source remains a valid trie.
		 */
		fresh_root = alloc_cds_ft_node(dst_ft, &ft_types[0], &fresh_meta);
		if (!fresh_root)
			return CDS_FT_STATUS_MEMORY_ERROR;

		/*
		 * Swap root pointers.  The source's root carries all
		 * metadata (nr_child, external_nodes) with it.
		 */
		rcu_assign_pointer(dst_ft->root, src_ft->root);
		rcu_assign_pointer(src_ft->root, ft_node_flag(fresh_root, 0));
		goto done;
	}

	{
		struct ft_graft_point gp;

		ft_descend_to_graft_point(dst_ft, key, key_len, &gp);

		/*
		 * The source root node becomes the graft payload.  Its
		 * metadata.external_nodes (NIL-key entries in the source)
		 * naturally becomes the entries at depth key_len in the
		 * destination.  No relocation needed.
		 */
		status = ft_store_at_graft_point(dst_ft, key, key_len,
						  &gp, src_ft->root);
		if (status != CDS_FT_STATUS_OK)
			return status;
	}

	/* Give source a fresh empty root. */
	{
		struct cds_ft_inode *fresh_node;
		struct cds_ft_metadata *fresh_meta;

		fresh_node = alloc_cds_ft_node(src_ft, &ft_types[0], &fresh_meta);
		if (!fresh_node) {
			/*
			 * The graft has already been published. We cannot
			 * roll back, but we must leave source in a valid
			 * state. This should not happen in practice.
			 */
			abort();
		}
		rcu_assign_pointer(src_ft->root, ft_node_flag(fresh_node, 0));
	}

done:
	{
		size_t nm = key_len + src_max;

		if (nm > uatomic_load(&dst_ft->max_used_key_len, CMM_RELAXED))
			uatomic_store(&dst_ft->max_used_key_len, nm,
				      CMM_RELAXED);
	}

	uatomic_store(&src_ft->max_used_key_len, 0, CMM_RELAXED);

	return CDS_FT_STATUS_OK;
}

enum cds_ft_status cds_ft_graft_swap(struct cds_ft *dst_ft,
		const uint8_t *key, size_t _key_len,
		struct cds_ft *swap_ft)
{
	size_t key_len, swap_max;

	if (!dst_ft || !swap_ft || dst_ft == swap_ft)
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	if (dst_ft->group != swap_ft->group)
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;

	/*
	 * Root-level swap (key_len == 0) is valid for both
	 * variable-length and fixed-length groups.  See cds_ft_graft.
	 */
	if (_key_len == 0) {
		key_len = 0;
	} else {
		key_len = ft_key_len(dst_ft, _key_len);
		if (!valid_key_len(dst_ft, key_len) ||
				dst_ft->group->key_len != CDS_FT_LEN_VARIABLE)
			return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}

	swap_max = uatomic_load(&swap_ft->max_used_key_len, CMM_RELAXED);
	if (key_len > 0 && swap_max > dst_ft->group->max_key_len - key_len)
		return CDS_FT_STATUS_OVERFLOW_ERROR;

	if (key_len == 0) {
		/*
		 * Swap entire tries: exchange root pointers.
		 * Each root carries its own metadata (nr_child,
		 * external_nodes), so no relocation is needed.
		 */
		struct cds_ft_inode_flag *tmp = dst_ft->root;
		size_t dm;

		rcu_assign_pointer(dst_ft->root, swap_ft->root);
		rcu_assign_pointer(swap_ft->root, tmp);

		dm = uatomic_load(&dst_ft->max_used_key_len, CMM_RELAXED);
		if (swap_max > dm)
			uatomic_store(&dst_ft->max_used_key_len,
				      swap_max, CMM_RELAXED);
		uatomic_store(&swap_ft->max_used_key_len, dm,
			      CMM_RELAXED);

		return CDS_FT_STATUS_OK;
	}

	{
		struct ft_graft_point gp;
		struct cds_ft_metadata *pmeta, *swap_rmeta;
		struct cds_ft_inode_flag *old_child, *old_swap_root;
		bool swap_empty;

		ft_descend_to_graft_point(dst_ft, key, key_len, &gp);

		if (gp.depth < key_len) {
			/*
			 * Path incomplete: nothing at or below the graft
			 * point.  swap_ft receives empty content.
			 * Delegate to graft for intermediate-node creation.
			 */
			return cds_ft_graft(dst_ft, key, _key_len, swap_ft);
		}

		/* Snapshot old content at the graft slot. */
		old_child = gp.nf;

		old_swap_root = swap_ft->root;
		swap_rmeta = ft_root_metadata(swap_ft);
		swap_empty = (swap_rmeta->nr_child == 0
				&& !swap_rmeta->external_nodes);

		/*
		 * Atomic store at graft point.  If swap is empty,
		 * place NULL (removing the subtree); otherwise place
		 * the swap root node directly.
		 */
		rcu_assign_pointer(*gp.nfp,
			swap_empty ? NULL : old_swap_root);

		/* Update parent nr_child on NULL <-> non-NULL transition. */
		pmeta = cds_ft_item_to_metadata(ft_node_ptr(gp.pnf));
		if (!ft_node_ptr(old_child) && !swap_empty)
			pmeta->nr_child++;
		else if (ft_node_ptr(old_child) && swap_empty)
			pmeta->nr_child--;

		/*
		 * Set up swap_ft to hold old content from the graft
		 * point.  If old_child is an internal node, it
		 * becomes swap_ft's root directly (its
		 * metadata.external_nodes carries the entries at the
		 * graft key).  Otherwise, allocate a fresh root and
		 * place any external node chain as NIL-key entries.
		 */
		if (ft_node_ptr(old_child)
				&& ft_node_internal(old_child)) {
			rcu_assign_pointer(swap_ft->root, old_child);
			if (swap_empty)
				free_cds_ft_node(swap_ft,
					ft_node_ptr(old_swap_root));
		} else if (swap_empty) {
			if (ft_node_ptr(old_child))
				swap_rmeta->external_nodes =
					(struct cds_ft_node *)
					ft_node_ptr(old_child);
		} else {
			struct cds_ft_inode *fresh;
			struct cds_ft_metadata *fresh_meta;

			fresh = alloc_cds_ft_node(swap_ft,
				&ft_types[0], &fresh_meta);
			if (!fresh)
				abort();
			rcu_assign_pointer(swap_ft->root, ft_node_flag(fresh, 0));
			if (ft_node_ptr(old_child))
				fresh_meta->external_nodes =
					(struct cds_ft_node *)
					ft_node_ptr(old_child);
		}

		{
			size_t nm = key_len + swap_max;
			size_t dm = uatomic_load(&dst_ft->max_used_key_len,
						 CMM_RELAXED);
			if (nm > dm)
				uatomic_store(&dst_ft->max_used_key_len, nm,
					      CMM_RELAXED);
			uatomic_store(&swap_ft->max_used_key_len,
				      dm > key_len ? dm - key_len : 0,
				      CMM_RELAXED);
		}
		return CDS_FT_STATUS_OK;
	}
}

enum cds_ft_status cds_ft_detach(struct cds_ft *ft,
		const uint8_t *key, size_t _key_len,
		struct cds_ft **result_ft)
{
	struct cds_ft *detached;
	struct cds_ft_inode_flag *child;
	size_t key_len;
	enum cds_ft_status status;

	*result_ft = NULL;

	if (!ft)
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;

	/*
	 * Root-level detach (key_len == 0) is valid for both
	 * variable-length and fixed-length groups.  See cds_ft_graft.
	 */
	if (_key_len == 0) {
		key_len = 0;
	} else {
		key_len = ft_key_len(ft, _key_len);
		if (!valid_key_len(ft, key_len) ||
				ft->group->key_len != CDS_FT_LEN_VARIABLE)
			return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}

	if (key_len == 0) {
		struct cds_ft_metadata *rmeta = ft_root_metadata(ft);
		struct cds_ft_inode *fresh_node;
		struct cds_ft_metadata *fresh_meta;

		/* Check if source trie is empty. */
		if (rmeta->nr_child == 0 && !rmeta->external_nodes)
			return CDS_FT_STATUS_NOT_FOUND;

		status = cds_ft_create(ft->group, &detached);
		if (status != CDS_FT_STATUS_OK)
			return status;

		/*
		 * Allocate a fresh empty root for the source trie
		 * before swapping.
		 */
		fresh_node = alloc_cds_ft_node(ft, &ft_types[0], &fresh_meta);
		if (!fresh_node) {
			cds_ft_destroy(detached);
			return CDS_FT_STATUS_MEMORY_ERROR;
		}

		/*
		 * Move the source root into the detached trie.
		 * Free the empty root that cds_ft_create allocated for
		 * the detached trie, and replace it with the source root.
		 */
		free_cds_ft_node(detached, ft_node_ptr(detached->root));
		/* No readers in detached root yet. */
		detached->root = ft->root;
		uatomic_store(&detached->max_used_key_len,
			      uatomic_load(&ft->max_used_key_len, CMM_RELAXED),
			      CMM_RELAXED);

		/* Give source a fresh empty root. */
		rcu_assign_pointer(ft->root, ft_node_flag(fresh_node, 0));

		*result_ft = detached;
		return CDS_FT_STATUS_OK;
	}

	/*
	 * key_len > 0: descent with snapshot tracking for
	 * ft_detach_node's upward pruning walk.
	 */
	{
		unsigned int i;
		struct cds_ft_inode_flag *nf;
		struct cds_ft_inode_flag **nfp;
		const uint8_t *ik = key;

		struct cds_ft_inode_flag *snapshot[FT_MAX_DEPTH];
		uint8_t snapshot_n[FT_MAX_DEPTH];
		int nr_snapshot = 0;
		struct cds_ft_inode_flag **det_nfp = NULL;
		struct cds_ft_inode_flag **det_pfp;
		struct cds_ft_inode_flag **pp_fp, **p_fp;
		bool pending;

		nf  = rcu_dereference(ft->root);
		nfp = &ft->root;

		pp_fp   = NULL;
		p_fp    = &ft->root;
		det_pfp = &ft->root;
		pending = true;

		for (i = 1; i <= key_len; i++) {
			uint8_t kv;
			const struct cds_ft_metadata *meta;

			if (!ft_node_ptr(nf))
				return CDS_FT_STATUS_NOT_FOUND;
			if (!ft_node_internal(nf))
				return CDS_FT_STATUS_NOT_FOUND;

			meta = cds_ft_item_to_metadata(ft_node_ptr(nf));
			if (meta->nr_child > 1) {
				det_pfp = p_fp;
				pending = true;
			} else if (i > 1 && meta->external_nodes) {
				det_nfp = p_fp;
				det_pfp = pp_fp;
				pending = false;
			}

			kv = key_to_ordinal(ft, *(ik++));
			snapshot_n[nr_snapshot + 1] = kv;
			snapshot[nr_snapshot++] = nf;
			nf = ft_node_get_nth(nf, &nfp, kv);
			if (nf) {
				pp_fp = p_fp;
				p_fp  = nfp;
				if (pending) {
					det_nfp = nfp;
					pending = false;
				}
			}
		}

		child = nf;

		if (!ft_node_ptr(child))
			return CDS_FT_STATUS_NOT_FOUND;

		status = cds_ft_create(ft->group, &detached);
		if (status != CDS_FT_STATUS_OK)
			return status;

		/*
		 * Detach child from the source trie and prune empty
		 * branches above.  After this, child is no longer
		 * reachable from the live trie for new readers.
		 */
		snapshot[nr_snapshot++] = child;
		{
			int ret = ft_detach_node(ft, snapshot, snapshot_n,
						 nr_snapshot,
						 det_nfp, det_pfp);
			assert(ret != -ENOENT);
			(void) ret;
		}

		/*
		 * If the detached child is an internal node, it
		 * becomes the detached trie's root directly.  Its
		 * metadata.external_nodes carries the entries at the
		 * detach key, which become NIL-key entries in the
		 * detached trie.  Free the empty root that
		 * cds_ft_create allocated and replace it.
		 *
		 * If the child is an external node, place it in the
		 * detached trie's (empty) root metadata as a NIL-key
		 * entry.
		 */
		if (ft_node_internal(child)) {
			free_cds_ft_node(detached,
				ft_node_ptr(detached->root));
			/* No readers in detached root yet. */
			detached->root = child;
		} else {
			struct cds_ft_metadata *dmeta =
				ft_root_metadata(detached);
			dmeta->external_nodes =
				(struct cds_ft_node *) ft_node_ptr(child);
		}
		{
			size_t fm = uatomic_load(&ft->max_used_key_len,
						 CMM_RELAXED);
			uatomic_store(&detached->max_used_key_len,
				      fm > key_len ? fm - key_len : 0,
				      CMM_RELAXED);
		}

		*result_ft = detached;
		return CDS_FT_STATUS_OK;
	}
}

size_t cds_ft_key_len(const struct cds_ft *ft)
{
	return ft->group->key_len;
}

size_t cds_ft_max_key_len(const struct cds_ft *ft)
{
	return ft->group->max_key_len;
}

size_t cds_ft_max_used_key_len(const struct cds_ft *ft)
{
	return uatomic_load(&ft->max_used_key_len, CMM_RELAXED);
}

enum cds_ft_status cds_ft_key_map(const struct cds_ft *ft, uint8_t *key_to_ordinal, uint8_t *ordinal_to_key)
{
	if (ft->group->key_map.identity)
		return CDS_FT_STATUS_NOT_FOUND;
	memcpy(key_to_ordinal, ft->group->key_map.key_to_ordinal, sizeof(ft->group->key_map.key_to_ordinal));
	memcpy(ordinal_to_key, ft->group->key_map.ordinal_to_key, sizeof(ft->group->key_map.ordinal_to_key));
	return CDS_FT_STATUS_OK;
}

bool cds_ft_empty(struct cds_ft *ft)
{
	struct cds_ft_inode_flag *root_flag = rcu_dereference(ft->root);
	struct cds_ft_inode *root_node = ft_node_ptr(root_flag);
	unsigned int type_idx = ft_node_type(root_flag);
	const struct cds_ft_type *type = &ft_types[type_idx];
	struct cds_ft_metadata *rmeta = cds_ft_item_to_metadata(root_node);

	/*
	 * As a root node special-case, only a type-0 linear node with
	 * data[0] == 0 represents an empty node.
	 */
	if (type->type_class != FT_LINEAR)
		return false;
	if (ft_linear_node_get_nr_child(type, root_node) != 0)
		return false;
	return !uatomic_load(&rmeta->external_nodes, CMM_RELAXED);
}

unsigned long cds_ft_count(struct cds_ft *ft)
{
	struct cds_ft_iter *iter;
	enum cds_ft_status status;
	unsigned long count = 0;

	status = cds_ft_iter_create(ft, &iter);
	if (status != CDS_FT_STATUS_OK)
		return 0;
	cds_ft_for_each_rcu(ft, iter) {
		struct cds_ft_node *node = cds_ft_iter_node(iter);

		cds_ft_for_each_duplicate_rcu(node)
			count++;
	}
	if (cds_ft_iter_status(iter) < 0)
		count = 0;
	cds_ft_iter_destroy(iter);
	return count;
}

enum cds_ft_status cds_ft_attr_create(struct cds_ft_attr **result)
{
	struct cds_ft_attr *attr = calloc(1, sizeof(struct cds_ft_attr));

	if (!attr) {
		*result = NULL;
		return CDS_FT_STATUS_MEMORY_ERROR;
	}
	attr->key_len = CDS_FT_LEN_DEFAULT;
	attr->max_key_len = FT_MAX_KEY_LEN;
	attr->key_map.identity = true;
	*result = attr;
	return CDS_FT_STATUS_OK;
}

void cds_ft_attr_destroy(struct cds_ft_attr *attr)
{
	free(attr);
}

enum cds_ft_status cds_ft_attr_set_key_len(struct cds_ft_attr *attr, size_t key_len)
{
	attr->key_len = key_len;
	return CDS_FT_STATUS_OK;
}

enum cds_ft_status cds_ft_attr_set_max_key_len(struct cds_ft_attr *attr, size_t max_key_len)
{
	if (max_key_len == CDS_FT_MAX_LEN_UNLIMITED) {
		attr->max_key_len = FT_MAX_KEY_LEN;
		return CDS_FT_STATUS_OK;
	}
	if (max_key_len > FT_MAX_KEY_LEN)
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	attr->max_key_len = max_key_len;
	return CDS_FT_STATUS_OK;
}

enum cds_ft_status cds_ft_attr_set_key_map(struct cds_ft_attr *attr,
		const uint8_t *key_to_ordinal, const uint8_t *ordinal_to_key)
{
	attr->key_map.identity = false;
	memcpy(attr->key_map.key_to_ordinal, key_to_ordinal, sizeof(attr->key_map.key_to_ordinal));
	memcpy(attr->key_map.ordinal_to_key, ordinal_to_key, sizeof(attr->key_map.ordinal_to_key));
	return CDS_FT_STATUS_OK;
}

enum cds_ft_status _cds_ft_group_create(const struct cds_ft_attr *attr,
		struct cds_ft_group **result_ft_group,
		const struct rcu_flavor_struct *flavor)
{
	struct cds_ft_group *ft_group;
	size_t key_len = CDS_FT_LEN_DEFAULT,
	       max_key_len = FT_MAX_KEY_LEN;

	if (attr) {
		key_len = attr->key_len;
		max_key_len = attr->max_key_len;
	}
	/* max_tree_depth 0 is for pointer to root node */
	if (key_len != CDS_FT_LEN_VARIABLE && key_len > max_key_len) {
		*result_ft_group = NULL;
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	}
	ft_group = calloc(1, sizeof(*ft_group));
	if (!ft_group) {
		*result_ft_group = NULL;
		return CDS_FT_STATUS_MEMORY_ERROR;
	}
	ft_group->key_len = key_len;
	ft_group->max_key_len = max_key_len;
	ft_group->max_tree_depth = max_key_len + 1;
	assert(ft_group->max_tree_depth <= FT_MAX_DEPTH);
	ft_group->flavor = flavor;
	if (attr)
		ft_group->key_map = attr->key_map;
	else
		ft_group->key_map.identity = true;
	*result_ft_group = ft_group;
	return CDS_FT_STATUS_OK;
}

enum cds_ft_status cds_ft_group_destroy(struct cds_ft_group *ft_group)
{
	if (uatomic_load(&ft_group->nr_ft_instances, CMM_RELAXED) != 0)
		return CDS_FT_STATUS_BUSY_ERROR;
	cds_ft_free_all_arenas(ft_group);
	free(ft_group);
	return CDS_FT_STATUS_OK;
}

enum cds_ft_status cds_ft_create(struct cds_ft_group *ft_group,
		struct cds_ft **result_ft)
{
	struct cds_ft *ft;
	struct cds_ft_inode *root_node;
	struct cds_ft_metadata *metadata;
	const struct cds_ft_type *type0 = &ft_types[0];

	ft = calloc(1, sizeof(*ft));
	if (!ft) {
		*result_ft = NULL;
		return CDS_FT_STATUS_MEMORY_ERROR;
	}
	ft->group = ft_group;

	/*
	 * Allocate the root node (smallest linear type, initially empty).
	 *
	 * ft->root always points to an internal node, even when the
	 * trie has no entries (nr_child == 0).  This is the one place
	 * where a node with 0 children is allowed; all other internal
	 * nodes are pruned when their last child is removed.  The node
	 * itself may be replaced by graft or graft-swap, but the
	 * invariant on the slot is maintained across all operations.
	 *
	 * The root is a regular internal node whose metadata
	 * (nr_child, external_nodes) is accessed the same way as any
	 * other node's.  Its metadata carries the NIL-key entries, so
	 * transplanting a root node between tries is a single pointer
	 * swap with no metadata relocation.
	 */
	root_node = alloc_cds_ft_node(ft, type0, &metadata);
	if (!root_node) {
		free(ft);
		*result_ft = NULL;
		return CDS_FT_STATUS_MEMORY_ERROR;
	}
	ft->root = ft_node_flag(root_node, 0);

	uatomic_inc(&ft_group->nr_ft_instances, CMM_RELAXED);
	*result_ft = ft;
	return CDS_FT_STATUS_OK;
}

static
void print_debug_fallback_distribution(struct cds_ft *ft)
{
	int i;

	fprintf(stderr, "Fallback node distribution:\n");
	for (i = 0; i < FT_ENTRY_PER_NODE; i++) {
		if (!ft->node_fallback_count_distribution[i])
			continue;
		fprintf(stderr, "	%3u: %4lu\n",
			i, ft->node_fallback_count_distribution[i]);
	}
}

static
void ft_final_checks(struct cds_ft *ft)
{
	double fallback_ratio;
	unsigned long na, nf, nr_fallback;

	if (!ft_debug_counters())
		return;

	fallback_ratio = (double) uatomic_read(&ft->nr_fallback);
	fallback_ratio /= (double) uatomic_read(&ft->nr_nodes_allocated);
	nr_fallback = uatomic_read(&ft->nr_fallback);
	if (nr_fallback)
		fprintf(stderr,
			"[warning] RCU Fractal Trie used %lu fallback node(s) (ratio: %g)\n",
			uatomic_read(&ft->nr_fallback),
			fallback_ratio);

	na = uatomic_read(&ft->nr_nodes_allocated);
	nf = uatomic_read(&ft->nr_nodes_freed);
	dbg_printf("Nodes allocated: %lu, Nodes freed: %lu.\n", na, nf);
	if (nr_fallback)
		print_debug_fallback_distribution(ft);

	if (na != nf) {
		fprintf(stderr, "[error] Fractal Trie leaked %ld nodes. Allocated: %lu, freed: %lu.\n",
			(long) na - nf, na, nf);
		abort();
	}
}

/*
 * There should be no more concurrent add, delete, nor look-up performed
 * on the Fractal Trie while it is being destroyed (ensured by the
 * caller).
 */
void cds_ft_destroy(struct cds_ft *ft)
{
	const struct rcu_flavor_struct *flavor = ft->group->flavor;

	/* Free root node. No concurrent readers at this point. */
	free_cds_ft_node(ft, ft_node_ptr(ft->root));
	/* Wait for in-flight call_rcu free to complete. */
	flavor->barrier();
	ft_final_checks(ft);
	uatomic_dec(&ft->group->nr_ft_instances, CMM_RELAXED);
	free(ft);
}

static
void print_indent(FILE *out, int level)
{
	int i;

	for (i = 0; i < level; i++)
		fprintf(out, "	");
}

static
void show_node_recursive(const struct cds_ft *ft, FILE *out, struct cds_ft_inode_flag *node_flag, int level)
{
	unsigned int key;

	print_indent(out, level);
	fprintf(out, "Level %d within node %p\n", level, node_flag);
	for (key = 0; key < 256; key++) {
		struct cds_ft_inode_flag *child_node_flag;

		child_node_flag = ft_node_get_nth(node_flag, NULL, (uint8_t) key);
		if (!ft_node_ptr(child_node_flag))
			continue;
		/* Found external node before end of key. */
		if (ft_node_internal(child_node_flag)) {
			struct cds_ft_metadata *metadata = cds_ft_item_to_metadata(ft_node_ptr(child_node_flag));
			struct cds_ft_node *external_nodes = rcu_dereference(metadata->external_nodes);

			print_indent(out, level);
			fprintf(out, "Level %d, key value: %u, internal node: %p, nr_children: %u\n",
				level, key, child_node_flag, metadata->nr_child);
			if (external_nodes) {
				print_indent(out, level);
				fprintf(out, "Level %d, key value: %u, (meta)external node list ptr: %p\n",
					level, key, external_nodes);
			}
			show_node_recursive(ft, out, child_node_flag, level + 1);
		} else {
			print_indent(out, level);
			fprintf(out, "Level %d, key value: %u, external node list ptr: %p\n",
				level, key, ft_node_ptr(child_node_flag));
		}
	}
}

void cds_ft_show(const struct cds_ft *ft, FILE *out)
{
	struct cds_ft_inode_flag *node_flag;
	int level = 0;

	fprintf(out, "Show Fractal Trie %p\n", ft);
	fprintf(out, "---------------------------------------------------\n");

	node_flag = rcu_dereference(ft->root);

	/* Root is always present and always internal. */
	print_indent(out, level);
	fprintf(out, "Level 0: root node %p\n", node_flag);
	show_node_recursive(ft, out, node_flag, level + 1);
	fprintf(out, "---------------------------------------------------\n");
}

struct cds_ft_node_stats {
	uint64_t count;
	uint64_t distribution[257];
};

struct cds_ft_stats_level {
	uint64_t nr_external_nodes;
	uint64_t nr_metadata_external_nodes;
	uint64_t nr_duplicate_external_nodes;
	uint64_t nr_internal_nodes;
	struct cds_ft_node_stats node_stats[FT_TYPE_MAX_NR];
	bool has_nodes;
};

struct cds_ft_stats {
	struct cds_ft_stats_level level[FT_MAX_DEPTH];
};

enum cds_ft_status cds_ft_recompute_stats(struct cds_ft *ft)
{
	struct cds_ft_iter *iter;
	enum cds_ft_status status;
	size_t max_len = 0;

	status = cds_ft_iter_create(ft, &iter);
	if (status != CDS_FT_STATUS_OK)
		return status;
	cds_ft_for_each_rcu(ft, iter) {
		if (iter->key_len > max_len)
			max_len = iter->key_len;
	}
	status = cds_ft_iter_status(iter);
	cds_ft_iter_destroy(iter);
	if (status < 0)
		return status;
	uatomic_store(&ft->max_used_key_len, max_len, CMM_RELAXED);
	return CDS_FT_STATUS_OK;
}

static
void calc_stats_node(const struct cds_ft *ft __attribute__((unused)),
		struct cds_ft_inode_flag *node_flag, struct cds_ft_stats *stats, int level)
{
	unsigned long node_type = ft_node_type(node_flag);
	struct cds_ft_node_stats *node_stats = &stats->level[level].node_stats[node_type];
	const struct cds_ft_metadata *metadata;

	metadata = cds_ft_item_to_metadata(ft_node_ptr(node_flag));
	node_stats->count++;
	node_stats->distribution[metadata->nr_child]++;
	stats->level[level].nr_internal_nodes++;
	stats->level[level].has_nodes = true;
}

static
void calc_stats_node_recursive(const struct cds_ft *ft, struct cds_ft_inode_flag *node_flag,
		struct cds_ft_stats *stats, int level)
{
	unsigned int key;

	for (key = 0; key < 256; key++) {
		struct cds_ft_inode_flag *child_node_flag;

		child_node_flag = ft_node_get_nth(node_flag, NULL, (uint8_t) key);
		if (!ft_node_ptr(child_node_flag))
			continue;
		if (ft_node_internal(child_node_flag)) {
			struct cds_ft_metadata *metadata = cds_ft_item_to_metadata(ft_node_ptr(child_node_flag));
			struct cds_ft_node *external_nodes = rcu_dereference(metadata->external_nodes);

			calc_stats_node(ft, child_node_flag, stats, level);
			if (external_nodes) {
				struct cds_ft_node *iter_node;
				unsigned int count = 0;

				iter_node = external_nodes;
				cds_ft_for_each_duplicate(iter_node) {
					if (count++ == 0)
						stats->level[level].nr_metadata_external_nodes++;
					else
						stats->level[level].nr_duplicate_external_nodes++;
					stats->level[level].has_nodes = true;
				}
			}
			calc_stats_node_recursive(ft, child_node_flag, stats, level + 1);
		} else {
			struct cds_ft_node *iter_node;
			unsigned int count = 0;

			iter_node = (struct cds_ft_node *) ft_node_ptr(child_node_flag);
			cds_ft_for_each_duplicate(iter_node) {
				if (count++ == 0)
					stats->level[level].nr_external_nodes++;
				else
					stats->level[level].nr_duplicate_external_nodes++;
				stats->level[level].has_nodes = true;
			}
		}
	}
}

static
void do_show_stats(const struct cds_ft *ft, FILE *out, const struct cds_ft_stats *stats)
{
	int level;

	fprintf(out, "Fractal Trie (%p) Statistics\n", ft);
	fprintf(out, "---------------------------------------------------\n");
	for (level = 0; level < FT_MAX_DEPTH; level++) {
		const struct cds_ft_stats_level *stats_level = &stats->level[level];
		unsigned long type;

		if (!stats_level->has_nodes)
			break;
		fprintf(out, "Level: %d\n", level);
		if (stats_level->nr_external_nodes) {
			print_indent(out, 1);
			fprintf(out, "External nodes: %" PRIu64 "\n", stats_level->nr_external_nodes);
		}
		if (stats_level->nr_metadata_external_nodes) {
			print_indent(out, 1);
			fprintf(out, "Metadata external nodes: %" PRIu64 "\n", stats_level->nr_metadata_external_nodes);
		}
		if (stats_level->nr_duplicate_external_nodes) {
			print_indent(out, 1);
			fprintf(out, "Duplicate external nodes: %" PRIu64 "\n", stats_level->nr_duplicate_external_nodes);
		}
		if (stats_level->nr_internal_nodes) {
			print_indent(out, 1);
			fprintf(out, "Internal nodes: %" PRIu64 "\n", stats_level->nr_internal_nodes);
		}
		for (type = 0; type < FT_TYPE_MAX_NR; type++) {
			const struct cds_ft_node_stats *node_stats = &stats->level[level].node_stats[type];
			uint64_t nr_nodes = node_stats->count;

			if (nr_nodes) {
				unsigned int i;
				bool first = true;

				print_indent(out, 2);
				fprintf(out, "Internal node type %lu: %" PRIu64 " (", type, nr_nodes);
				for (i = 0; i <= 256; i++) {
					if (node_stats->distribution[i]) {
						fprintf(out, "%s%u: %" PRIu64,
							(!first ? ", " : ""), i, node_stats->distribution[i]);
						first = false;
					}
				}
				fprintf(out, ")\n");
			}
		}
	}
	fprintf(out, "---------------------------------------------------\n");
}

void cds_ft_show_stats(const struct cds_ft *ft, FILE *out)
{
	struct cds_ft_inode_flag *node_flag;
	struct cds_ft_stats stats = {};
	int level = 0;

	node_flag = rcu_dereference(ft->root);

	/* Root is always present and always internal. */
	calc_stats_node(ft, node_flag, &stats, level);
	calc_stats_node_recursive(ft, node_flag, &stats, level + 1);
	do_show_stats(ft, out, &stats);
}

const char *cds_ft_status_to_string(enum cds_ft_status status)
{
	switch (status) {
	/* Success return codes (>= 0). */
	case CDS_FT_STATUS_OK:
		return "Operation completed successfully";
	case CDS_FT_STATUS_NOT_FOUND:
		return "No node found";
	case CDS_FT_STATUS_DUPLICATE_FOUND:
		return "Duplicate node exists";
	case CDS_FT_STATUS_INTERNAL_MATCH:
		return "Match ends at an internal node";

	/* Error return codes (< 0). */
	case CDS_FT_STATUS_INVALID_ARGUMENT_ERROR:
		return "Invalid argument";
	case CDS_FT_STATUS_MEMORY_ERROR:
		return "Memory allocation failure";
	case CDS_FT_STATUS_OVERFLOW_ERROR:
		return "Buffer too small for key length";
	case CDS_FT_STATUS_BUSY_ERROR:
		return "Resource busy";
	case CDS_FT_STATUS_POPULATED_ERROR:
		return "Destination already populated";

	default:
		return "Unknown status value";
	}
}

enum cds_ft_status cds_ft_iter_create(struct cds_ft *ft, struct cds_ft_iter **result_iter)
{
	struct cds_ft_iter *iter = calloc(1, sizeof(struct cds_ft_iter));

	if (!iter) {
		*result_iter = NULL;
		return CDS_FT_STATUS_MEMORY_ERROR;
	}
	iter->ft = ft;
	*result_iter = iter;
	return CDS_FT_STATUS_OK;
}

void cds_ft_iter_destroy(struct cds_ft_iter *iter)
{
	free(iter);
}

enum cds_ft_status cds_ft_iter_status(const struct cds_ft_iter *iter)
{
	return iter->status;
}

enum cds_ft_status cds_ft_iter_get_key(struct cds_ft_iter *iter,
		uint8_t *result_key, size_t result_key_max_len, size_t *result_key_len)
{
	*result_key_len = iter->key_len;
	if (iter->key_len > result_key_max_len)
		return CDS_FT_STATUS_OVERFLOW_ERROR;
	memcpy(result_key, iter->key, iter->key_len);
	return CDS_FT_STATUS_OK;
}

enum cds_ft_status cds_ft_iter_get_prefix(struct cds_ft_iter *iter,
		uint8_t *result_key, size_t result_key_max_len, size_t *result_key_len)
{
	*result_key_len = iter->prefix_len;
	if (iter->prefix_len > result_key_max_len)
		return CDS_FT_STATUS_OVERFLOW_ERROR;
	memcpy(result_key, iter->key, iter->prefix_len);
	return CDS_FT_STATUS_OK;
}

enum cds_ft_status cds_ft_iter_set_key(struct cds_ft_iter *iter, const uint8_t *key, size_t key_len)
{
	bool subset = false;

	key_len = ft_key_len(iter->ft, key_len);
	if (key_len > iter->ft->group->max_key_len)
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	if (key_len <= iter->key_len && !memcmp(key, iter->key, key_len))
		subset = true;
	/*
	 * If new key is a subset of current key, the path stays valid,
	 * otherwise invalidate the path.
	 */
	if (!subset) {
		memcpy(iter->key, key, key_len);
		iter->path_valid = false;
		iter->path_len = 0;
	} else {
		iter->path_len = key_len + 1;
	}
	iter->key_len = key_len;
	return CDS_FT_STATUS_OK;
}

/*
 * The prefix is a subset of the current key. Set the key before setting
 * the prefix length.
 */
enum cds_ft_status cds_ft_iter_set_prefix_len(struct cds_ft_iter *iter, size_t prefix_len)
{
	if (prefix_len > iter->key_len)
		return CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;
	iter->prefix_len = prefix_len;
	return CDS_FT_STATUS_OK;
}

void cds_ft_iter_reset(struct cds_ft_iter *iter)
{
	iter->path_valid = false;
	iter->status = CDS_FT_STATUS_OK;
	iter->path_len = 0;
	iter->key_len = 0;
	iter->prefix_len = 0;
	iter->node = NULL;
#ifdef DEBUG_CLEAR_ITER
	/* Reset to 0 for debugging. */
	memset(iter->path_node, 0, sizeof(iter->path_node));
	memset(iter->key, 0, sizeof(iter->key));
#endif
}

void cds_ft_iter_copy(struct cds_ft_iter *dst, const struct cds_ft_iter *src)
{
	memcpy(dst, src, sizeof(*dst));
}

struct cds_ft_node *cds_ft_iter_node(const struct cds_ft_iter *iter)
{
	return iter->node;
}
