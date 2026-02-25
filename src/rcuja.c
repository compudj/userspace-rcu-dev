// SPDX-FileCopyrightText: 2000-2002 Hewlett-Packard Company
// SPDX-FileCopyrightText: 2012-2025 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-only

/*
 * rcuja/rcuja.c
 *
 * Userspace RCU library - RCU Judy Array
 */

#define _LGPL_SOURCE
#include <stdint.h>
#include <errno.h>
#include <limits.h>
#include <string.h>
#include <assert.h>
#include <endian.h>
#include <stdbool.h>
#include <urcu/rcuja.h>
#include <urcu/compiler.h>
#include <urcu/arch.h>
#include <urcu-pointer.h>
#include <urcu/uatomic.h>

#include "rcuja-internal.h"

#ifndef abs
#define abs_int(a)	((int) (a) > 0 ? (int) (a) : -((int) (a)))
#endif

#define CDS_JA_DEFAULT_MAX_KEY_LEN	JA_MAX_KEY_LEN
#define CDS_JA_DEFAULT_KEY_LEN		0

struct cds_ja_attr {
	size_t key_len;
	size_t max_key_len;
};

enum cds_ja_type_class {
	RCU_JA_LINEAR = 0,	/* Type A */
			/* 32-bit: 1 to 25 children, 8 to 128 bytes */
			/* 64-bit: 1 to 28 children, 16 to 256 bytes */
	RCU_JA_POOL = 1,	/* Type B */
			/* 32-bit: 26 to 100 children, 256 to 512 bytes */
			/* 64-bit: 29 to 112 children, 512 to 1024 bytes */
	RCU_JA_PIGEON = 2,	/* Type C */
			/* 32-bit: 101 to 256 children, 1024 bytes */
			/* 64-bit: 113 to 256 children, 2048 bytes */
	/* Leaf nodes are implicit from their height in the tree */
	RCU_JA_NR_TYPES,

	RCU_JA_NULL,	/* not an encoded type, but keeps code regular */
};

struct cds_ja_type {
	enum cds_ja_type_class type_class;
	uint16_t min_child;		/* minimum number of children: 1 to 256 */
	uint16_t max_child;		/* maximum number of children: 1 to 256 */
	uint16_t max_linear_child;	/* per-pool max nr. children: 1 to 256 */
	uint16_t order;			/* node size is (1 << order), in bytes */
	uint16_t nr_pool_order;		/* number of pools */
	uint16_t pool_size_order;	/* pool size */
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
 * The max_child values for the RCU_JA_POOL below result from
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
 * The cds_ja_node contains the compressed node data needed for
 * read-side. For linear and pool node configurations, it starts with a
 * byte counting the number of children in the node.  Then, the
 * node-specific data is placed.
 * For the pigeon configuration, the number of children is kept in the
 * metadata associated to node.
 */

#define DECLARE_LINEAR_NODE(index)								\
	struct {										\
		uint8_t nr_child;								\
		uint8_t child_value[ja_type_## index ##_max_linear_child];			\
		struct cds_ja_inode_flag *child_ptr[ja_type_## index ##_max_linear_child];	\
	}

#define DECLARE_POOL_NODE(index)								\
	struct {										\
		struct {									\
			uint8_t nr_child;							\
			uint8_t child_value[ja_type_## index ##_max_linear_child];		\
			struct cds_ja_inode_flag *child_ptr[ja_type_## index ##_max_linear_child]; \
		} linear[1U << ja_type_## index ##_nr_pool_order];				\
	}

#if (CAA_BITS_PER_LONG < 64)

/* 32-bit pointers */
enum {
	ja_type_0_max_child = 3,
	ja_type_1_max_child = 6,
	ja_type_2_max_child = 12,
	ja_type_3_max_child = 25,
	ja_type_4_max_child = 48,
	ja_type_5_max_child = 92,
	ja_type_6_max_child = 256,
	ja_type_7_max_child = 0,	/* NULL */
};

enum {
	ja_type_0_max_linear_child = 3,
	ja_type_1_max_linear_child = 6,
	ja_type_2_max_linear_child = 12,
	ja_type_3_max_linear_child = 25,
	ja_type_4_max_linear_child = 24,
	ja_type_5_max_linear_child = 23,
};

enum {
	ja_type_4_nr_pool_order = 1,
	ja_type_5_nr_pool_order = 2,
};

const struct cds_ja_type ja_types[] = {
	[0] = { .type_class = RCU_JA_LINEAR, .min_child = 1, .max_child = ja_type_0_max_child, .max_linear_child = ja_type_0_max_linear_child, .order = 4, },
	[1] = { .type_class = RCU_JA_LINEAR, .min_child = 3, .max_child = ja_type_1_max_child, .max_linear_child = ja_type_1_max_linear_child, .order = 5, },
	[2] = { .type_class = RCU_JA_LINEAR, .min_child = 4, .max_child = ja_type_2_max_child, .max_linear_child = ja_type_2_max_linear_child, .order = 6, },
	[3] = { .type_class = RCU_JA_LINEAR, .min_child = 10, .max_child = ja_type_3_max_child, .max_linear_child = ja_type_3_max_linear_child, .order = 7, },

	/* Pools may fill sooner than max_child */
	/* This pool is hardcoded at index 4. See ja_node_ptr(). */
	[RCU_JA_POOL_IDX_A] = { .type_class = RCU_JA_POOL, .min_child = 20, .max_child = ja_type_4_max_child, .max_linear_child = ja_type_4_max_linear_child, .order = 8, .nr_pool_order = ja_type_4_nr_pool_order, .pool_size_order = 7, },
	/* This pool is hardcoded at index 5. See ja_node_ptr(). */
	[RCU_JA_POOL_IDX_B] = { .type_class = RCU_JA_POOL, .min_child = 45, .max_child = ja_type_5_max_child, .max_linear_child = ja_type_5_max_linear_child, .order = 9, .nr_pool_order = ja_type_5_nr_pool_order, .pool_size_order = 7, },

	/*
	 * Upon node removal below min_child, if child pool is filled
	 * beyond capacity, we roll back to pigeon.
	 */
	[6] = { .type_class = RCU_JA_PIGEON, .min_child = 83, .max_child = ja_type_6_max_child, .order = 10, },

	[7] = { .type_class = RCU_JA_NULL, .min_child = 0, .max_child = ja_type_7_max_child, },
};

struct cds_ja_inode {
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
			struct cds_ja_inode_flag *child[ja_type_6_max_child];
		} conf_6;
		/* data aliasing nodes for computed accesses */
		uint8_t data[sizeof(struct cds_ja_inode_flag *) * ja_type_6_max_child];
	} u;
};
#else /* !(CAA_BITS_PER_LONG < 64) */
/* 64-bit pointers */
enum {
	ja_type_0_max_child = 1,
	ja_type_1_max_child = 3,
	ja_type_2_max_child = 7,
	ja_type_3_max_child = 14,
	ja_type_4_max_child = 28,
	ja_type_5_max_child = 54,
	ja_type_6_max_child = 104,
	ja_type_7_max_child = 256,
	ja_type_8_max_child = 256,
};

enum {
	ja_type_0_max_linear_child = 1,
	ja_type_1_max_linear_child = 3,
	ja_type_2_max_linear_child = 7,
	ja_type_3_max_linear_child = 14,
	ja_type_4_max_linear_child = 28,
	ja_type_5_max_linear_child = 27,
	ja_type_6_max_linear_child = 26,
};

enum {
	ja_type_5_nr_pool_order = 1,
	ja_type_6_nr_pool_order = 2,
};

const struct cds_ja_type ja_types[] = {
	[0] = { .type_class = RCU_JA_LINEAR, .min_child = 1, .max_child = ja_type_0_max_child, .max_linear_child = ja_type_0_max_linear_child, .order = 4, },
	[1] = { .type_class = RCU_JA_LINEAR, .min_child = 1, .max_child = ja_type_1_max_child, .max_linear_child = ja_type_1_max_linear_child, .order = 5, },
	[2] = { .type_class = RCU_JA_LINEAR, .min_child = 3, .max_child = ja_type_2_max_child, .max_linear_child = ja_type_2_max_linear_child, .order = 6, },
	[3] = { .type_class = RCU_JA_LINEAR, .min_child = 5, .max_child = ja_type_3_max_child, .max_linear_child = ja_type_3_max_linear_child, .order = 7, },
	[4] = { .type_class = RCU_JA_LINEAR, .min_child = 10, .max_child = ja_type_4_max_child, .max_linear_child = ja_type_4_max_linear_child, .order = 8, },

	/* Pools may fill sooner than max_child. */
	/* This pool is hardcoded at index 5. See ja_node_ptr(). */
	[RCU_JA_POOL_IDX_A] = { .type_class = RCU_JA_POOL, .min_child = 22, .max_child = ja_type_5_max_child, .max_linear_child = ja_type_5_max_linear_child, .order = 9, .nr_pool_order = ja_type_5_nr_pool_order, .pool_size_order = 8, },
	/* This pool is hardcoded at index 6. See ja_node_ptr(). */
	[RCU_JA_POOL_IDX_B] = { .type_class = RCU_JA_POOL, .min_child = 51, .max_child = ja_type_6_max_child, .max_linear_child = ja_type_6_max_linear_child, .order = 10, .nr_pool_order = ja_type_6_nr_pool_order, .pool_size_order = 8, },

	/*
	 * Upon node removal below min_child, if child pool is filled
	 * beyond capacity, we roll back to pigeon.
	 */
	[7] = { .type_class = RCU_JA_PIGEON, .min_child = 95, .max_child = ja_type_7_max_child, .order = 11, },

	[8] = { .type_class = RCU_JA_NULL, .min_child = 0, .max_child = ja_type_8_max_child, },
};

struct cds_ja_inode {
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
			struct cds_ja_inode_flag *child[ja_type_7_max_child];
		} conf_7;
		/* data aliasing nodes for computed accesses */
		uint8_t data[sizeof(struct cds_ja_inode_flag *) * ja_type_7_max_child];
	} u;
};
#endif /* !(BITS_PER_LONG < 64) */

static inline __attribute__((unused))
void static_array_size_check(void)
{
	CAA_BUILD_BUG_ON(CAA_ARRAY_SIZE(ja_types) < JA_TYPE_MAX_NR);
}

enum ja_recompact {
	JA_RECOMPACT_ADD_SAME,
	JA_RECOMPACT_ADD_NEXT,
	JA_RECOMPACT_DEL,
};

enum ja_lookup_inequality {
	JA_LOOKUP_GE,
	JA_LOOKUP_LE,
	JA_LOOKUP_GT,
	JA_LOOKUP_LT,
};

enum ja_direction {
	JA_LEFT,
	JA_RIGHT,
	JA_LEFTMOST,
	JA_RIGHTMOST,
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
unsigned long ja_node_pool_1d_bitsel(struct cds_ja_inode_flag *node)
{
	return ((unsigned long) node & JA_POOL_1D_MASK) >> (JA_TYPE_BITS + JA_INTERNAL_BITS);
}

static
void ja_node_pool_2d_index(struct cds_ja_inode_flag *node, unsigned int *index)
{
	*index = ((unsigned long) node & JA_POOL_2D_MASK) >> (JA_TYPE_BITS + JA_INTERNAL_BITS);
}

static
size_t ja_key_len(const struct cds_ja *ja, size_t key_len)
{
	if (!key_len)
		return ja->key_len;
	/* Validate that explicit and implicit key lengths match for fixed length Judy Array. */
	if (ja->key_len && key_len != ja->key_len)
		return 0;
	return key_len;
}

uint64_t cds_ja_key_to_u64(const struct cds_ja *ja, const uint8_t *key,
		size_t _key_len)
{
	size_t key_len = ja_key_len(ja, _key_len);
	union {
		uint64_t v64;
		uint8_t array[8];
	} u;

	assert(key_len > 0 && key_len <= 8);
	u.v64 = 0;
	/* Copy len LSB. */
	memcpy(u.array + sizeof(u.array) - key_len , key, key_len);
	/* Big endian to host endianness. */
	return be64toh(u.v64);
}

void cds_ja_u64_to_key(const struct cds_ja *ja, uint64_t v, uint8_t *key,
		size_t _key_len)
{
	size_t key_len = ja_key_len(ja, _key_len);
	union {
		uint64_t v64;
		uint8_t array[8];
	} u;

	assert(key_len > 0 && key_len <= 8);
	/* Host endianness to big endian. */
	u.v64 = htobe64(v);
	/* Copy len LSB. */
	memcpy(key, u.array + sizeof(u.array) - key_len , key_len);
}

uint32_t cds_ja_key_to_u32(const struct cds_ja *ja, const uint8_t *key,
		size_t _key_len)
{
	size_t key_len = ja_key_len(ja, _key_len);
	union {
		uint32_t v32;
		uint8_t array[4];
	} u;

	assert(key_len > 0 && key_len <= 4);
	u.v32 = 0;
	/* Copy len LSB. */
	memcpy(u.array + sizeof(u.array) - key_len , key, key_len);
	/* Big endian to host endianness. */
	return be32toh(u.v32);
}

void cds_ja_u32_to_key(const struct cds_ja *ja, uint32_t v, uint8_t *key,
		size_t _key_len)
{
	size_t key_len = ja_key_len(ja, _key_len);
	union {
		uint32_t v32;
		uint8_t array[4];
	} u;

	assert(key_len > 0 && key_len <= 4);
	/* Host endianness to big endian. */
	u.v32 = htobe32(v);
	/* Copy len LSB. */
	memcpy(key, u.array + sizeof(u.array) - key_len , key_len);
}

static
struct cds_ja_inode *_ja_node_mask_ptr(struct cds_ja_inode_flag *node)
{
	return (struct cds_ja_inode *) (((unsigned long) node) & JA_PTR_MASK);
}

static
unsigned long ja_node_type(struct cds_ja_inode_flag *node)
{
	unsigned long type;

	if (_ja_node_mask_ptr(node) == NULL) {
		return NODE_INDEX_NULL;
	}
	type = (unsigned int) (((unsigned long) node & JA_TYPE_MASK) >> JA_INTERNAL_BITS);
	assert(type < (1UL << JA_TYPE_BITS));
	return type;
}

static
bool ja_node_internal(struct cds_ja_inode_flag *node)
{
	return (unsigned long) node & JA_INTERNAL_MASK;
}

static
bool valid_external_node(struct cds_ja_node *node)
{
	return !ja_node_internal((struct cds_ja_inode_flag *) node);
}

static
bool valid_key_len(struct cds_ja *ja, size_t key_len)
{
	size_t max_key_len = ja->max_key_len;

	if (!key_len)
		return false;
	if (max_key_len && key_len > max_key_len)
		return false;
	return true;
}

static
struct cds_ja_inode *alloc_cds_ja_node(struct cds_ja *ja,
		const struct cds_ja_type *ja_type,
		struct cds_ja_metadata **_metadata)
{
	struct cds_ja_metadata *metadata;
	void *p;

	metadata = cds_ja_alloc_item(ja, ja_type->order);
	if (!metadata) {
		return NULL;
	}
	p = cds_ja_metadata_to_item(metadata);
	if (ja_debug_counters())
		uatomic_inc(&ja->nr_nodes_allocated);
	*_metadata = metadata;
	return p;
}

static
void free_cds_ja_node(struct cds_ja *ja, struct cds_ja_inode *node)
{
	struct cds_ja_metadata *metadata = cds_ja_item_to_metadata(node);

	cds_ja_free_item(metadata);
	if (ja_debug_counters() && node)
		uatomic_inc(&ja->nr_nodes_freed);
}

#define __JA_ALIGN_MASK(v, mask)	(((v) + (mask)) & ~(mask))
#define JA_ALIGN(v, align)		__JA_ALIGN_MASK(v, (typeof(v)) (align) - 1)
#define __JA_FLOOR_MASK(v, mask)	((v) & ~(mask))
#define JA_FLOOR(v, align)		__JA_FLOOR_MASK(v, (typeof(v)) (align) - 1)

static
uint8_t *align_ptr_size(uint8_t *ptr)
{
	return (uint8_t *) JA_ALIGN((unsigned long) ptr, sizeof(void *));
}

static
uint8_t ja_linear_node_get_nr_child(const struct cds_ja_type *type,
		struct cds_ja_inode *node)
{
	assert(type->type_class == RCU_JA_LINEAR || type->type_class == RCU_JA_POOL);
	/* load-acquire orders nr_child load before values and pointers */
	return uatomic_load(&node->u.data[0], CMM_ACQUIRE);
}

/*
 * The order in which values and pointers are does does not matter: if
 * a value is missing, we return NULL. If a value is there, but its
 * associated pointers is still NULL, we return NULL too.
 */
static
struct cds_ja_inode_flag *ja_linear_node_get_nth(const struct cds_ja_type *type,
		struct cds_ja_inode *node,
		struct cds_ja_inode_flag ***node_flag_ptr,
		uint8_t n)
{
	uint8_t nr_child;
	uint8_t *values;
	struct cds_ja_inode_flag **pointers;
	struct cds_ja_inode_flag *ptr;
	unsigned int i;

	assert(type->type_class == RCU_JA_LINEAR || type->type_class == RCU_JA_POOL);

	nr_child = ja_linear_node_get_nr_child(type, node);
	assert(nr_child <= type->max_linear_child);
	assert(type->type_class != RCU_JA_LINEAR || nr_child >= type->min_child);

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
	pointers = (struct cds_ja_inode_flag **) align_ptr_size(&values[type->max_linear_child]);
	ptr = rcu_dereference(pointers[i]);
	if (caa_unlikely(node_flag_ptr))
		*node_flag_ptr = &pointers[i];
	return ptr;
}

static
struct cds_ja_inode_flag *ja_linear_node_get_direction(const struct cds_ja_type *type,
		struct cds_ja_inode *node,
		int n, uint8_t *result_key,
		enum ja_direction dir)
{
	uint8_t nr_child;
	uint8_t *values;
	struct cds_ja_inode_flag **pointers;
	struct cds_ja_inode_flag *ptr, *match_ptr = NULL;
	unsigned int i;
	int match_v;

	assert(type->type_class == RCU_JA_LINEAR || type->type_class == RCU_JA_POOL);
	assert(dir == JA_LEFT || dir == JA_RIGHT);

	if (dir == JA_LEFT) {
		match_v = -1;
	} else {
		match_v = JA_ENTRY_PER_NODE;
	}

	nr_child = ja_linear_node_get_nr_child(type, node);
	cmm_smp_rmb();	/* read nr_child before values and pointers */
	assert(nr_child <= type->max_linear_child);
	assert(type->type_class != RCU_JA_LINEAR || nr_child >= type->min_child);

	values = &node->u.data[1];
	pointers = (struct cds_ja_inode_flag **) align_ptr_size(&values[type->max_linear_child]);
	for (i = 0; i < nr_child; i++) {
		unsigned int v;

		v = uatomic_load(&values[i], CMM_RELAXED);
		ptr = rcu_dereference(pointers[i]);
		if (!ptr)
			continue;
		if (dir == JA_LEFT) {
			if ((int) v < n && (int) v > match_v) {
				match_v = v;
				match_ptr = ptr;
			}
		} else {
			if ((int) v > n && (int) v < match_v) {
				match_v = v;
				match_ptr = ptr;
			}
		}
	}

	if (!match_ptr) {
		return NULL;
	}
	assert(match_v >= 0 && match_v < JA_ENTRY_PER_NODE);

	*result_key = (uint8_t) match_v;
	return match_ptr;
}

static
void ja_linear_node_get_ith_pos(const struct cds_ja_type *type,
		struct cds_ja_inode *node,
		uint8_t i,
		uint8_t *v,
		struct cds_ja_inode_flag **iter)
{
	uint8_t *values;
	struct cds_ja_inode_flag **pointers;

	assert(type->type_class == RCU_JA_LINEAR || type->type_class == RCU_JA_POOL);
	assert(i < ja_linear_node_get_nr_child(type, node));

	values = &node->u.data[1];
	*v = values[i];
	pointers = (struct cds_ja_inode_flag **) align_ptr_size(&values[type->max_linear_child]);
	*iter = rcu_dereference(pointers[i]);
}

static
struct cds_ja_inode_flag *ja_pool_node_get_nth(const struct cds_ja_type *type,
		struct cds_ja_inode *node,
		struct cds_ja_inode_flag *node_flag,
		struct cds_ja_inode_flag ***node_flag_ptr,
		uint8_t n)
{
	struct cds_ja_inode *linear;

	assert(type->type_class == RCU_JA_POOL);

	switch (type->nr_pool_order) {
	case 1:
	{
		unsigned long bitsel, index;

		bitsel = ja_node_pool_1d_bitsel(node_flag);
		assert(bitsel < CHAR_BIT);
		index = ((unsigned long) n >> bitsel) & 0x1;
		linear = (struct cds_ja_inode *) &node->u.data[index << type->pool_size_order];
		break;
	}
	case 2:
	{
		unsigned int C_n8_r2_index, subclass_index;
		uint8_t bits[2];

		ja_node_pool_2d_index(node_flag, &C_n8_r2_index);
		index_to_bits_C_n8_r2(C_n8_r2_index, bits);
		subclass_index = value_and_bits_to_subclass_index(n, bits);
		linear = (struct cds_ja_inode *) &node->u.data[subclass_index << type->pool_size_order];
		break;
	}
	default:
		linear = NULL;
		assert(0);
	}
	return ja_linear_node_get_nth(type, linear, node_flag_ptr, n);
}

static
struct cds_ja_inode *ja_pool_node_get_ith_pool(const struct cds_ja_type *type,
		struct cds_ja_inode *node,
		uint8_t i)
{
	assert(type->type_class == RCU_JA_POOL);
	return (struct cds_ja_inode *)
		&node->u.data[(unsigned int) i << type->pool_size_order];
}

static
struct cds_ja_inode_flag *ja_pool_node_get_direction(const struct cds_ja_type *type,
		struct cds_ja_inode *node,
		int n, uint8_t *result_key,
		enum ja_direction dir)
{
	unsigned int pool_nr;
	int match_v;
	struct cds_ja_inode_flag *match_node_flag = NULL;

	assert(type->type_class == RCU_JA_POOL);
	assert(dir == JA_LEFT || dir == JA_RIGHT);

	if (dir == JA_LEFT) {
		match_v = -1;
	} else {
		match_v = JA_ENTRY_PER_NODE;
	}

	for (pool_nr = 0; pool_nr < (1U << type->nr_pool_order); pool_nr++) {
		struct cds_ja_inode *pool =
			ja_pool_node_get_ith_pool(type,
				node, pool_nr);
		uint8_t nr_child =
			ja_linear_node_get_nr_child(type, pool);
		unsigned int j;

		for (j = 0; j < nr_child; j++) {
			struct cds_ja_inode_flag *iter;
			uint8_t v;

			ja_linear_node_get_ith_pos(type, pool,
					j, &v, &iter);
			if (!iter)
				continue;
			if (dir == JA_LEFT) {
				if ((int) v < n && (int) v > match_v) {
					match_v = v;
					match_node_flag = iter;
				}
			} else {
				if ((int) v > n && (int) v < match_v) {
					match_v = v;
					match_node_flag = iter;
				}
			}
		}
	}
	if (match_node_flag)
		*result_key = (uint8_t) match_v;
	return match_node_flag;
}

static
struct cds_ja_inode_flag *ja_pigeon_node_get_nth(const struct cds_ja_type *type,
		struct cds_ja_inode *node,
		struct cds_ja_inode_flag ***node_flag_ptr,
		uint8_t n)
{
	struct cds_ja_inode_flag **child_node_flag_ptr;
	struct cds_ja_inode_flag *child_node_flag;

	assert(type->type_class == RCU_JA_PIGEON);
	child_node_flag_ptr = &((struct cds_ja_inode_flag **) node->u.data)[n];
	child_node_flag = rcu_dereference(*child_node_flag_ptr);
	dbg_printf("ja_pigeon_node_get_nth child_node_flag_ptr %p\n",
		child_node_flag_ptr);
	if (caa_unlikely(node_flag_ptr))
		*node_flag_ptr = child_node_flag_ptr;
	return child_node_flag;
}

static
struct cds_ja_inode_flag *ja_pigeon_node_get_direction(const struct cds_ja_type *type,
		struct cds_ja_inode *node,
		int n, uint8_t *result_key,
		enum ja_direction dir)
{
	struct cds_ja_inode_flag **child_node_flag_ptr;
	struct cds_ja_inode_flag *child_node_flag;
	int i;

	assert(type->type_class == RCU_JA_PIGEON);
	assert(dir == JA_LEFT || dir == JA_RIGHT);

	if (dir == JA_LEFT) {
		/* n - 1 is first value left of n */
		for (i = n - 1; i >= 0; i--) {
			child_node_flag_ptr = &((struct cds_ja_inode_flag **) node->u.data)[i];
			child_node_flag = rcu_dereference(*child_node_flag_ptr);
			if (child_node_flag) {
				dbg_printf("ja_pigeon_node_get_left child_node_flag %p\n",
					child_node_flag);
				*result_key = (uint8_t) i;
				return child_node_flag;
			}
		}
	} else {
		/* n + 1 is first value right of n */
		for (i = n + 1; i < JA_ENTRY_PER_NODE; i++) {
			child_node_flag_ptr = &((struct cds_ja_inode_flag **) node->u.data)[i];
			child_node_flag = rcu_dereference(*child_node_flag_ptr);
			if (child_node_flag) {
				dbg_printf("ja_pigeon_node_get_right child_node_flag %p\n",
					child_node_flag);
				*result_key = (uint8_t) i;
				return child_node_flag;
			}
		}
	}
	return NULL;
}

static
struct cds_ja_inode_flag *ja_pigeon_node_get_ith_pos(const struct cds_ja_type *type,
		struct cds_ja_inode *node,
		uint8_t i)
{
	return ja_pigeon_node_get_nth(type, node, NULL, i);
}

/*
 * ja_node_get_nth: get nth item from a node.
 * node_flag is already rcu_dereference'd.
 */
static
struct cds_ja_inode_flag *ja_node_get_nth(struct cds_ja_inode_flag *node_flag,
		struct cds_ja_inode_flag ***node_flag_ptr,
		uint8_t n)
{
	unsigned int type_index;
	struct cds_ja_inode *node;
	const struct cds_ja_type *type;

	node = ja_node_ptr(node_flag);
	assert(node != NULL);
	type_index = ja_node_type(node_flag);
	type = &ja_types[type_index];

	switch (type->type_class) {
	case RCU_JA_LINEAR:
		return ja_linear_node_get_nth(type, node,
				node_flag_ptr, n);
	case RCU_JA_POOL:
		return ja_pool_node_get_nth(type, node, node_flag,
				node_flag_ptr, n);
	case RCU_JA_PIGEON:
		return ja_pigeon_node_get_nth(type, node,
				node_flag_ptr, n);
	default:
		assert(0);
		return (void *) -1UL;
	}
}

static
struct cds_ja_inode_flag *ja_node_get_direction(struct cds_ja_inode_flag *node_flag,
		int n, uint8_t *result_key,
		enum ja_direction dir)
{
	unsigned int type_index;
	struct cds_ja_inode *node;
	const struct cds_ja_type *type;

	node = ja_node_ptr(node_flag);
	assert(node != NULL);
	type_index = ja_node_type(node_flag);
	type = &ja_types[type_index];

	switch (type->type_class) {
	case RCU_JA_LINEAR:
		return ja_linear_node_get_direction(type, node, n, result_key, dir);
	case RCU_JA_POOL:
		return ja_pool_node_get_direction(type, node, n, result_key, dir);
	case RCU_JA_PIGEON:
		return ja_pigeon_node_get_direction(type, node, n, result_key, dir);
	default:
		assert(0);
		return (void *) -1UL;
	}
}

static
struct cds_ja_inode_flag *ja_node_get_leftright(struct cds_ja_inode_flag *node_flag,
		unsigned int n, uint8_t *result_key,
		enum ja_direction dir)
{
	return ja_node_get_direction(node_flag, n, result_key, dir);
}

static
struct cds_ja_inode_flag *ja_node_get_minmax(struct cds_ja_inode_flag *node_flag,
		uint8_t *result_key,
		enum ja_direction dir)
{
	switch (dir) {
	case JA_LEFTMOST:
		return ja_node_get_direction(node_flag,
				-1, result_key, JA_RIGHT);
	case JA_RIGHTMOST:
		return ja_node_get_direction(node_flag,
				JA_ENTRY_PER_NODE, result_key, JA_LEFT);
	default:
		assert(0);
	}
}

static
int ja_linear_node_set_nth(const struct cds_ja_type *type,
		struct cds_ja_inode *node,
		struct cds_ja_metadata *metadata,
		uint8_t n,
		struct cds_ja_inode_flag *child_node_flag)
{
	uint8_t nr_child;
	uint8_t *values, *nr_child_ptr;
	struct cds_ja_inode_flag **pointers;
	unsigned int i, unused = 0;

	assert(type->type_class == RCU_JA_LINEAR || type->type_class == RCU_JA_POOL);

	nr_child_ptr = &node->u.data[0];
	dbg_printf("linear set nth: n %u, nr_child_ptr %p\n",
		(unsigned int) n, nr_child_ptr);
	nr_child = *nr_child_ptr;
	assert(nr_child <= type->max_linear_child);

	values = &node->u.data[1];
	pointers = (struct cds_ja_inode_flag **) align_ptr_size(&values[type->max_linear_child]);
	/* Check if node value is already populated */
	for (i = 0; i < nr_child; i++) {
		if (values[i] == n) {
			if (pointers[i])
				return -EEXIST;
			else
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

	assert(pointers[i] == NULL);
	/* If we expanded the nr_child, increment it */
	if (i == nr_child) {
		uatomic_store(&pointers[i], child_node_flag, CMM_RELAXED);
		uatomic_store(&values[nr_child], n, CMM_RELAXED);
		/* store-release: write pointer and value before nr_child */
		uatomic_store(nr_child_ptr, nr_child + 1, CMM_RELEASE);
	} else {
		/* Replacing a NULL pointer. */
		rcu_assign_pointer(pointers[i], child_node_flag);
	}
	metadata->nr_child++;
	dbg_printf("linear set nth: %u child, metadata: %u child, for node %p\n",
		(unsigned int) uatomic_load(nr_child_ptr, CMM_RELAXED),
		(unsigned int) metadata->nr_child,
		node);

	return 0;
}

static
int ja_pool_node_set_nth(const struct cds_ja_type *type,
		struct cds_ja_inode *node,
		struct cds_ja_inode_flag *node_flag,
		struct cds_ja_metadata *metadata,
		uint8_t n,
		struct cds_ja_inode_flag *child_node_flag)
{
	struct cds_ja_inode *linear;

	assert(type->type_class == RCU_JA_POOL);

	switch (type->nr_pool_order) {
	case 1:
	{
		unsigned long bitsel, index;

		bitsel = ja_node_pool_1d_bitsel(node_flag);
		assert(bitsel < CHAR_BIT);
		index = ((unsigned long) n >> bitsel) & 0x1;
		linear = (struct cds_ja_inode *) &node->u.data[index << type->pool_size_order];
		break;
	}
	case 2:
	{
		unsigned int C_n8_r2_index, subclass_index;
		uint8_t bits[2];

		ja_node_pool_2d_index(node_flag, &C_n8_r2_index);
		index_to_bits_C_n8_r2(C_n8_r2_index, bits);
		subclass_index = value_and_bits_to_subclass_index(n, bits);
		linear = (struct cds_ja_inode *) &node->u.data[subclass_index << type->pool_size_order];
		break;
	}
	default:
		linear = NULL;
		assert(0);
	}

	return ja_linear_node_set_nth(type, linear, metadata, n, child_node_flag);
}

static
int ja_pigeon_node_set_nth(const struct cds_ja_type *type,
		struct cds_ja_inode *node,
		struct cds_ja_metadata *metadata,
		uint8_t n,
		struct cds_ja_inode_flag *child_node_flag)
{
	struct cds_ja_inode_flag **ptr;

	assert(type->type_class == RCU_JA_PIGEON);
	ptr = &((struct cds_ja_inode_flag **) node->u.data)[n];
	if (*ptr)
		return -EEXIST;
	rcu_assign_pointer(*ptr, child_node_flag);
	metadata->nr_child++;
	return 0;
}

/*
 * _ja_node_set_nth: set nth item within a node. Return an error
 * (negative error value) if it is already there.
 */
static
int _ja_node_set_nth(const struct cds_ja_type *type,
		struct cds_ja_inode *node,
		struct cds_ja_inode_flag *node_flag,
		struct cds_ja_metadata *metadata,
		uint8_t n,
		struct cds_ja_inode_flag *child_node_flag)
{
	switch (type->type_class) {
	case RCU_JA_LINEAR:
		return ja_linear_node_set_nth(type, node, metadata, n, child_node_flag);
	case RCU_JA_POOL:
		return ja_pool_node_set_nth(type, node, node_flag, metadata, n, child_node_flag);
	case RCU_JA_PIGEON:
		return ja_pigeon_node_set_nth(type, node, metadata, n, child_node_flag);
	case RCU_JA_NULL:
		return -ENOSPC;
	default:
		assert(0);
		return -EINVAL;
	}

	return 0;
}

static
int ja_linear_node_clear_ptr(const struct cds_ja_type *type,
		struct cds_ja_inode *node,
		struct cds_ja_metadata *metadata,
		struct cds_ja_inode_flag **node_flag_ptr)
{
	uint8_t nr_child;
	uint8_t *nr_child_ptr;

	assert(type->type_class == RCU_JA_LINEAR || type->type_class == RCU_JA_POOL);

	nr_child_ptr = &node->u.data[0];
	nr_child = *nr_child_ptr;
	assert(nr_child <= type->max_linear_child);

	if (type->type_class == RCU_JA_LINEAR) {
		assert(!metadata->fallback_removal_count);
		if (metadata->nr_child <= type->min_child) {
			/* We need to try recompacting the node */
			return -EFBIG;
		}
	}
	dbg_printf("linear clear ptr: nr_child_ptr %p\n", nr_child_ptr);
	assert(*node_flag_ptr != NULL);
	rcu_assign_pointer(*node_flag_ptr, NULL);
	/*
	 * Value and nr_child are never changed (would cause ABA issue).
	 * Instead, we leave the pointer to NULL and recompact the node
	 * once in a while. It is allowed to set a NULL pointer to a new
	 * value without recompaction though.
	 * Only update the metadata node accounting.
	 */
	metadata->nr_child--;
	dbg_printf("linear clear ptr: %u child, metadata: %u child, for node %p\n",
		(unsigned int) uatomic_load(nr_child_ptr, CMM_RELAXED),
		(unsigned int) metadata->nr_child,
		node);
	return 0;
}

static
int ja_pool_node_clear_ptr(const struct cds_ja_type *type,
		struct cds_ja_inode *node,
		struct cds_ja_inode_flag *node_flag,
		struct cds_ja_metadata *metadata,
		struct cds_ja_inode_flag **node_flag_ptr,
		uint8_t n)
{
	struct cds_ja_inode *linear;

	assert(type->type_class == RCU_JA_POOL);

	if (metadata->fallback_removal_count) {
		metadata->fallback_removal_count--;
	} else {
		/* We should try recompacting the node */
		if (metadata->nr_child <= type->min_child)
			return -EFBIG;
	}

	switch (type->nr_pool_order) {
	case 1:
	{
		unsigned long bitsel, index;

		bitsel = ja_node_pool_1d_bitsel(node_flag);
		assert(bitsel < CHAR_BIT);
		index = ((unsigned long) n >> bitsel) & type->nr_pool_order;
		linear = (struct cds_ja_inode *) &node->u.data[index << type->pool_size_order];
		break;
	}
	case 2:
	{
		unsigned int C_n8_r2_index, subclass_index;
		uint8_t bits[2];

		ja_node_pool_2d_index(node_flag, &C_n8_r2_index);
		index_to_bits_C_n8_r2(C_n8_r2_index, bits);
		subclass_index = value_and_bits_to_subclass_index(n, bits);
		linear = (struct cds_ja_inode *) &node->u.data[subclass_index << type->pool_size_order];
		break;
	}
	default:
		linear = NULL;
		assert(0);
	}

	return ja_linear_node_clear_ptr(type, linear, metadata, node_flag_ptr);
}

static
int ja_pigeon_node_clear_ptr(const struct cds_ja_type *type,
		struct cds_ja_metadata *metadata,
		struct cds_ja_inode_flag **node_flag_ptr)
{
	assert(type->type_class == RCU_JA_PIGEON);

	if (metadata->fallback_removal_count) {
		metadata->fallback_removal_count--;
	} else {
		/* We should try recompacting the node */
		if (metadata->nr_child <= type->min_child)
			return -EFBIG;
	}
	dbg_printf("ja_pigeon_node_clear_ptr: clearing ptr: %p\n", *node_flag_ptr);
	rcu_assign_pointer(*node_flag_ptr, NULL);
	metadata->nr_child--;
	return 0;
}

/*
 * _ja_node_clear_ptr: clear ptr item within a node. Return an error
 * (negative error value) if it is not found (-ENOENT).
 */
static
int _ja_node_clear_ptr(const struct cds_ja_type *type,
		struct cds_ja_inode *node,
		struct cds_ja_inode_flag *node_flag,
		struct cds_ja_metadata *metadata,
		struct cds_ja_inode_flag **node_flag_ptr,
		uint8_t n)
{
	switch (type->type_class) {
	case RCU_JA_LINEAR:
		return ja_linear_node_clear_ptr(type, node, metadata, node_flag_ptr);
	case RCU_JA_POOL:
		return ja_pool_node_clear_ptr(type, node, node_flag, metadata, node_flag_ptr, n);
	case RCU_JA_PIGEON:
		return ja_pigeon_node_clear_ptr(type, metadata, node_flag_ptr);
	case RCU_JA_NULL:
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
unsigned int ja_node_sum_distribution_1d(enum ja_recompact mode,
		const struct cds_ja_type *type,
		struct cds_ja_inode *node,
		uint8_t n,
		struct cds_ja_inode_flag **nullify_node_flag_ptr)
{
	uint8_t nr_one[JA_BITS_PER_BYTE];
	unsigned int bitsel = 0, bit_i, overall_best_distance = UINT_MAX;
	unsigned int distrib_nr_child = 0;

	memset(nr_one, 0, sizeof(nr_one));

	switch (type->type_class) {
	case RCU_JA_LINEAR:
	{
		uint8_t nr_child =
			ja_linear_node_get_nr_child(type, node);
		unsigned int i;

		for (i = 0; i < nr_child; i++) {
			struct cds_ja_inode_flag *iter;
			uint8_t v;

			ja_linear_node_get_ith_pos(type, node, i, &v, &iter);
			if (!iter)
				continue;
			if (mode == JA_RECOMPACT_DEL && *nullify_node_flag_ptr == iter)
				continue;
			for (bit_i = 0; bit_i < JA_BITS_PER_BYTE; bit_i++) {
				if (v & (1U << bit_i))
					nr_one[bit_i]++;
			}
			distrib_nr_child++;
		}
		break;
	}
	case RCU_JA_POOL:
	{
		unsigned int pool_nr;

		for (pool_nr = 0; pool_nr < (1U << type->nr_pool_order); pool_nr++) {
			struct cds_ja_inode *pool =
				ja_pool_node_get_ith_pool(type,
					node, pool_nr);
			uint8_t nr_child =
				ja_linear_node_get_nr_child(type, pool);
			unsigned int j;

			for (j = 0; j < nr_child; j++) {
				struct cds_ja_inode_flag *iter;
				uint8_t v;

				ja_linear_node_get_ith_pos(type, pool,
						j, &v, &iter);
				if (!iter)
					continue;
				if (mode == JA_RECOMPACT_DEL && *nullify_node_flag_ptr == iter)
					continue;
				for (bit_i = 0; bit_i < JA_BITS_PER_BYTE; bit_i++) {
					if (v & (1U << bit_i))
						nr_one[bit_i]++;
				}
				distrib_nr_child++;
			}
		}
		break;
	}
	case RCU_JA_PIGEON:
	{
		unsigned int i;

		assert(mode == JA_RECOMPACT_DEL);
		for (i = 0; i < JA_ENTRY_PER_NODE; i++) {
			struct cds_ja_inode_flag *iter;

			iter = ja_pigeon_node_get_ith_pos(type, node, i);
			if (!iter)
				continue;
			if (mode == JA_RECOMPACT_DEL && *nullify_node_flag_ptr == iter)
				continue;
			for (bit_i = 0; bit_i < JA_BITS_PER_BYTE; bit_i++) {
				if (i & (1U << bit_i))
					nr_one[bit_i]++;
			}
			distrib_nr_child++;
		}
		break;
	}
	case RCU_JA_NULL:
		assert(mode == JA_RECOMPACT_ADD_NEXT);
		break;
	default:
		assert(0);
		break;
	}

	if (mode == JA_RECOMPACT_ADD_NEXT || mode == JA_RECOMPACT_ADD_SAME) {
		for (bit_i = 0; bit_i < JA_BITS_PER_BYTE; bit_i++) {
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
	for (bit_i = 0; bit_i < JA_BITS_PER_BYTE; bit_i++) {
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
void ja_node_sum_distribution_2d(enum ja_recompact mode,
		const struct cds_ja_type *type,
		struct cds_ja_inode *node,
		uint8_t n,
		struct cds_ja_inode_flag **nullify_node_flag_ptr,
		unsigned int *_bitsel)
{
	uint8_t nr_2d_11[JA_BITS_PER_BYTE][JA_BITS_PER_BYTE],
		nr_2d_10[JA_BITS_PER_BYTE][JA_BITS_PER_BYTE],
		nr_2d_01[JA_BITS_PER_BYTE][JA_BITS_PER_BYTE],
		nr_2d_00[JA_BITS_PER_BYTE][JA_BITS_PER_BYTE];
	unsigned int bitsel[2] = { 0, 1 };
	unsigned int bit_i, bit_j;
	int overall_best_distance = INT_MAX;
	unsigned int distrib_nr_child = 0;

	memset(nr_2d_11, 0, sizeof(nr_2d_11));
	memset(nr_2d_10, 0, sizeof(nr_2d_10));
	memset(nr_2d_01, 0, sizeof(nr_2d_01));
	memset(nr_2d_00, 0, sizeof(nr_2d_00));

	switch (type->type_class) {
	case RCU_JA_LINEAR:
	{
		uint8_t nr_child =
			ja_linear_node_get_nr_child(type, node);
		unsigned int i;

		for (i = 0; i < nr_child; i++) {
			struct cds_ja_inode_flag *iter;
			uint8_t v;

			ja_linear_node_get_ith_pos(type, node, i, &v, &iter);
			if (!iter)
				continue;
			if (mode == JA_RECOMPACT_DEL && *nullify_node_flag_ptr == iter)
				continue;
			for (bit_i = 0; bit_i < JA_BITS_PER_BYTE; bit_i++) {
				for (bit_j = bit_i + 1; bit_j < JA_BITS_PER_BYTE; bit_j++) {
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
	case RCU_JA_POOL:
	{
		unsigned int pool_nr;

		for (pool_nr = 0; pool_nr < (1U << type->nr_pool_order); pool_nr++) {
			struct cds_ja_inode *pool =
				ja_pool_node_get_ith_pool(type,
					node, pool_nr);
			uint8_t nr_child =
				ja_linear_node_get_nr_child(type, pool);
			unsigned int j;

			for (j = 0; j < nr_child; j++) {
				struct cds_ja_inode_flag *iter;
				uint8_t v;

				ja_linear_node_get_ith_pos(type, pool,
						j, &v, &iter);
				if (!iter)
					continue;
				if (mode == JA_RECOMPACT_DEL && *nullify_node_flag_ptr == iter)
					continue;
				for (bit_i = 0; bit_i < JA_BITS_PER_BYTE; bit_i++) {
					for (bit_j = bit_i + 1; bit_j < JA_BITS_PER_BYTE; bit_j++) {
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
	case RCU_JA_PIGEON:
	{
		unsigned int i;

		assert(mode == JA_RECOMPACT_DEL);
		for (i = 0; i < JA_ENTRY_PER_NODE; i++) {
			struct cds_ja_inode_flag *iter;

			iter = ja_pigeon_node_get_ith_pos(type, node, i);
			if (!iter)
				continue;
			if (mode == JA_RECOMPACT_DEL && *nullify_node_flag_ptr == iter)
				continue;
			for (bit_i = 0; bit_i < JA_BITS_PER_BYTE; bit_i++) {
				for (bit_j = bit_i + 1; bit_j < JA_BITS_PER_BYTE; bit_j++) {
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
	case RCU_JA_NULL:
		assert(mode == JA_RECOMPACT_ADD_NEXT);
		break;
	default:
		assert(0);
		break;
	}

	if (mode == JA_RECOMPACT_ADD_NEXT || mode == JA_RECOMPACT_ADD_SAME) {
		for (bit_i = 0; bit_i < JA_BITS_PER_BYTE; bit_i++) {
			for (bit_j = bit_i + 1; bit_j < JA_BITS_PER_BYTE; bit_j++) {
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
	for (bit_i = 0; bit_i < JA_BITS_PER_BYTE; bit_i++) {
		for (bit_j = bit_i + 1; bit_j < JA_BITS_PER_BYTE; bit_j++) {
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
		unsigned int nr_nodes)
{
	const struct cds_ja_type *type;

	assert(type_index != NODE_INDEX_NULL);
	if (nr_nodes == 0)
		return NODE_INDEX_NULL;
	for (;;) {
		type = &ja_types[type_index];
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
 * ja_node_recompact_add: recompact a node, adding a new child.
 * Return 0 on success, -EAGAIN if need to retry, or other negative
 * error value otherwise.
 */
static
int ja_node_recompact(enum ja_recompact mode,
		struct cds_ja *ja,
		unsigned int old_type_index,
		const struct cds_ja_type *old_type,
		struct cds_ja_inode *old_node,
		struct cds_ja_metadata *metadata,
		struct cds_ja_inode_flag **old_node_flag_ptr, uint8_t n,
		struct cds_ja_inode_flag *child_node_flag,
		struct cds_ja_inode_flag **nullify_node_flag_ptr)
{
	unsigned int new_type_index;
	struct cds_ja_inode *new_node;
	struct cds_ja_metadata *new_metadata;
	const struct cds_ja_type *new_type;
	struct cds_ja_inode_flag *new_node_flag;
	int ret;
	int fallback = 0;

	/*
	 * Need to find nearest type index even for ADD_SAME, because
	 * this recompaction, when applied to linear nodes, will garbage
	 * collect dummy (NULL) entries, and can therefore cause a few
	 * linear representations to be skipped.
	 */
	switch (mode) {
	case JA_RECOMPACT_ADD_SAME:
		new_type_index = find_nearest_type_index(old_type_index,
			metadata->nr_child + 1);
		dbg_printf("Recompact for node with %u children\n",
			metadata->nr_child + 1);
		break;
	case JA_RECOMPACT_ADD_NEXT:
		if (!metadata || old_type_index == NODE_INDEX_NULL) {
			new_type_index = 0;
			dbg_printf("Recompact for NULL\n");
		} else {
			new_type_index = find_nearest_type_index(old_type_index,
				metadata->nr_child + 1);
			dbg_printf("Recompact for node with %u children\n",
				metadata->nr_child + 1);
		}
		break;
	case JA_RECOMPACT_DEL:
		new_type_index = find_nearest_type_index(old_type_index,
			metadata->nr_child - 1);
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
	new_type = &ja_types[new_type_index];
	if (new_type_index != NODE_INDEX_NULL) {
		new_node = alloc_cds_ja_node(ja, new_type, &new_metadata);
		if (!new_node)
			return -ENOMEM;

		if (new_type->type_class == RCU_JA_POOL) {
			switch (new_type->nr_pool_order) {
			case 1:
			{
				unsigned int node_distrib_bitsel;

				node_distrib_bitsel =
					ja_node_sum_distribution_1d(mode,
						old_type, old_node,
						n, nullify_node_flag_ptr);
				assert(!((unsigned long) new_node & JA_POOL_1D_MASK));
				new_node_flag = ja_node_flag_pool_1d(new_node,
					new_type_index, node_distrib_bitsel);
				break;
			}
			case 2:
			{
				unsigned int node_distrib_bitsel[2];
				unsigned int subclass_index;
				uint8_t mask;

				ja_node_sum_distribution_2d(mode,
					old_type, old_node,
					n, nullify_node_flag_ptr,
					node_distrib_bitsel);
				assert(!((unsigned long) new_node & JA_POOL_2D_MASK));
				mask = (1U << node_distrib_bitsel[0]) | (1U << node_distrib_bitsel[1]);
				subclass_index = mask_to_index_C_n8_r2(mask);
				new_node_flag = ja_node_flag_pool_2d(new_node,
					new_type_index, subclass_index);
				break;
			}
			default:
				assert(0);
			}
		} else {
			new_node_flag = ja_node_flag(new_node, new_type_index);
		}

		dbg_printf("Recompact inherit from %p\n", metadata);
		if (metadata)
			new_metadata->fallback_removal_count = metadata->fallback_removal_count;
		if (fallback)
			new_metadata->fallback_removal_count =
						JA_FALLBACK_REMOVAL_COUNT;
	} else {
		new_node = NULL;
		new_node_flag = NULL;
	}

	assert(mode != JA_RECOMPACT_ADD_NEXT || old_type->type_class != RCU_JA_PIGEON);

	if (new_type_index == NODE_INDEX_NULL)
		goto skip_copy;

	switch (old_type->type_class) {
	case RCU_JA_LINEAR:
	{
		uint8_t nr_child =
			ja_linear_node_get_nr_child(old_type, old_node);
		unsigned int i;

		for (i = 0; i < nr_child; i++) {
			struct cds_ja_inode_flag *iter;
			uint8_t v;

			ja_linear_node_get_ith_pos(old_type, old_node, i, &v, &iter);
			if (!iter)
				continue;
			if (mode == JA_RECOMPACT_DEL && *nullify_node_flag_ptr == iter)
				continue;
			ret = _ja_node_set_nth(new_type, new_node, new_node_flag,
					new_metadata, v, iter);
			if (new_type->type_class == RCU_JA_POOL && ret) {
				goto fallback_toosmall;
			}
			assert(!ret);
		}
		break;
	}
	case RCU_JA_POOL:
	{
		unsigned int pool_nr;

		for (pool_nr = 0; pool_nr < (1U << old_type->nr_pool_order); pool_nr++) {
			struct cds_ja_inode *pool =
				ja_pool_node_get_ith_pool(old_type,
					old_node, pool_nr);
			uint8_t nr_child =
				ja_linear_node_get_nr_child(old_type, pool);
			unsigned int j;

			for (j = 0; j < nr_child; j++) {
				struct cds_ja_inode_flag *iter;
				uint8_t v;

				ja_linear_node_get_ith_pos(old_type, pool,
						j, &v, &iter);
				if (!iter)
					continue;
				if (mode == JA_RECOMPACT_DEL && *nullify_node_flag_ptr == iter)
					continue;
				ret = _ja_node_set_nth(new_type, new_node, new_node_flag,
						new_metadata, v, iter);
				if (new_type->type_class == RCU_JA_POOL
						&& ret) {
					goto fallback_toosmall;
				}
				assert(!ret);
			}
		}
		break;
	}
	case RCU_JA_NULL:
		assert(mode == JA_RECOMPACT_ADD_NEXT);
		break;
	case RCU_JA_PIGEON:
	{
		unsigned int i;

		assert(mode == JA_RECOMPACT_DEL);
		for (i = 0; i < JA_ENTRY_PER_NODE; i++) {
			struct cds_ja_inode_flag *iter;

			iter = ja_pigeon_node_get_ith_pos(old_type, old_node, i);
			if (!iter)
				continue;
			if (mode == JA_RECOMPACT_DEL && *nullify_node_flag_ptr == iter)
				continue;
			ret = _ja_node_set_nth(new_type, new_node, new_node_flag,
					new_metadata, i, iter);
			if (new_type->type_class == RCU_JA_POOL && ret) {
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

	if (mode == JA_RECOMPACT_ADD_NEXT || mode == JA_RECOMPACT_ADD_SAME) {
		/* add node */
		ret = _ja_node_set_nth(new_type, new_node, new_node_flag,
				new_metadata, n, child_node_flag);
		if (new_type->type_class == RCU_JA_POOL && ret) {
			goto fallback_toosmall;
		}
		assert(!ret);
	}

	if (fallback) {
		dbg_printf("Using fallback for %u children, node type index: %u, mode %s\n",
			new_metadata->nr_child, old_type_index, mode == JA_RECOMPACT_ADD_NEXT ? "add_next" :
				(mode == JA_RECOMPACT_DEL ? "del" : "add_same"));
		if (ja_debug_counters())
			uatomic_inc(&ja->node_fallback_count_distribution[new_metadata->nr_child]);
	}

	/* Return pointer to new recompacted node through old_node_flag_ptr */
	*old_node_flag_ptr = new_node_flag;
	if (old_node)
		free_cds_ja_node(ja, old_node);

	ret = 0;
end:
	return ret;

fallback_toosmall:
	/* fallback if next pool is too small */
	free_cds_ja_node(ja, new_node);

	switch (mode) {
	case JA_RECOMPACT_ADD_SAME:
		/*
		 * JA_RECOMPACT_ADD_SAME is only triggered if a linear
		 * node within a pool has unused entries. It should
		 * therefore _never_ be too small.
		 */
		assert(0);

		/* Fall-through */
	case JA_RECOMPACT_ADD_NEXT:
	{
		const struct cds_ja_type *next_type;

		/*
		 * Recompaction attempt on add failed. Should only
		 * happen if target node type is pool. Caused by
		 * hard-to-split distribution. Recompact using the next
		 * distribution size.
		 */
		assert(new_type->type_class == RCU_JA_POOL);
		next_type = &ja_types[new_type_index + 1];
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
			if (ja_debug_counters())
				uatomic_inc(&ja->nr_fallback);
			fallback = 1;
			goto retry;
		}
		break;
	}
	case JA_RECOMPACT_DEL:
		/*
		 * Recompaction attempt on delete failed. Should only
		 * happen if target node type is pool. This is caused by
		 * a hard-to-split distribution. Recompact on same node
		 * size, but flag current node as "fallback" to ensure
		 * we don't attempt recompaction before some activity
		 * has reshuffled our node.
		 */
		assert(new_type->type_class == RCU_JA_POOL);
		new_type_index = old_type_index;
		dbg_printf("Delete fallback keeping type %d\n", new_type_index);
		uatomic_inc(&ja->nr_fallback);
		fallback = 1;
		goto retry;
	default:
		assert(0);
		return -EINVAL;
	}

	/*
	 * Last resort fallback: pigeon.
	 */
	new_type_index = (1UL << JA_TYPE_BITS) - 1;
	dbg_printf("Fallback to type %d\n", new_type_index);
	uatomic_inc(&ja->nr_fallback);
	fallback = 1;
	goto retry;
}

/*
 * Return 0 on success, -EAGAIN if need to retry, or other negative
 * error value otherwise.
 */
static
int ja_node_set_nth(struct cds_ja *ja,
		struct cds_ja_inode_flag **node_flag, uint8_t n,
		struct cds_ja_inode_flag *child_node_flag,
		struct cds_ja_metadata *metadata)
{
	int ret;
	unsigned int type_index;
	const struct cds_ja_type *type;
	struct cds_ja_inode *node;

	dbg_printf("ja_node_set_nth for n=%u, node %p\n", (unsigned int) n, ja_node_ptr(*node_flag));

	node = ja_node_ptr(*node_flag);
	type_index = ja_node_type(*node_flag);
	type = &ja_types[type_index];
	ret = _ja_node_set_nth(type, node, *node_flag, metadata, n, child_node_flag);
	switch (ret) {
	case -ENOSPC:
		/* Not enough space in node, need to recompact to next type. */
		ret = ja_node_recompact(JA_RECOMPACT_ADD_NEXT, ja, type_index, type, node,
					metadata, node_flag, n, child_node_flag, NULL);
		break;
	case -ERANGE:
		/* Node needs to be recompacted. */
		ret = ja_node_recompact(JA_RECOMPACT_ADD_SAME, ja, type_index, type, node,
					metadata, node_flag, n, child_node_flag, NULL);
		break;
	}
	return ret;
}

/*
 * Return 0 on success, -EAGAIN if need to retry, or other negative
 * error value otherwise.
 */
static
int ja_node_clear_ptr(struct cds_ja *ja,
		struct cds_ja_inode_flag **node_flag_ptr,	/* Pointer to location to nullify */
		struct cds_ja_inode_flag **parent_node_flag_ptr,	/* Address of parent ptr in its parent */
		struct cds_ja_metadata *metadata,		/* of parent */
		uint8_t n)
{
	int ret;
	unsigned int type_index;
	const struct cds_ja_type *type;
	struct cds_ja_inode *node;

	dbg_printf("ja_node_clear_ptr for node %p, target ptr %p\n",
		ja_node_ptr(*parent_node_flag_ptr), node_flag_ptr);

	node = ja_node_ptr(*parent_node_flag_ptr);
	type_index = ja_node_type(*parent_node_flag_ptr);
	type = &ja_types[type_index];
	ret = _ja_node_clear_ptr(type, node, *parent_node_flag_ptr, metadata, node_flag_ptr, n);
	if (ret == -EFBIG) {
		/* Should try recompaction. */
		ret = ja_node_recompact(JA_RECOMPACT_DEL, ja, type_index, type, node,
				metadata, parent_node_flag_ptr, n, NULL,
				node_flag_ptr);
	}
	return ret;
}

struct cds_ja_node *cds_ja_lookup(struct cds_ja *ja, const uint8_t *key, size_t _key_len)
{
	size_t key_len = ja_key_len(ja, _key_len);
	struct cds_ja_inode_flag *node_flag;
	struct cds_ja_metadata *metadata;
	unsigned int key_depth, i;

	if (!valid_key_len(ja, key_len))
		return NULL;
	key_depth = key_len + 1;
	node_flag = rcu_dereference(ja->root);

	/* level 0: root node */
	if (!ja_node_ptr(node_flag))
		return NULL;

	for (i = 1; i < key_depth; i++) {
		uint8_t iter_key;

		iter_key = *(key++);
		node_flag = ja_node_get_nth(node_flag, NULL, iter_key);
		dbg_printf("cds_ja_lookup iter key lookup %u finds node_flag %p\n",
				(unsigned int) iter_key, node_flag);
		if (!ja_node_ptr(node_flag))
			return NULL;
		/* Found external node before end of key. */
		if (i < key_depth - 1 && !ja_node_internal(node_flag))
			return NULL;
	}

	/*
	 * Reached key_depth, check for terminal node: either external
	 * nodes or internal node associated with external nodes.
	 */
	if (ja_node_internal(node_flag)) {
		struct cds_ja_metadata *metadata = cds_ja_item_to_metadata(ja_node_ptr(node_flag));
		return rcu_dereference(metadata->external_nodes);
	}
	return (struct cds_ja_node *) node_flag;
}

struct cds_ja_node *cds_ja_lookup_partial(struct cds_ja *ja, const uint8_t *key, size_t _key_len, size_t *_match_len)
{
	size_t key_len = ja_key_len(ja, _key_len), match_len = 0;
	struct cds_ja_node *match_node = NULL;
	struct cds_ja_inode_flag *node_flag;
	unsigned int key_depth, i;

	if (!valid_key_len(ja, key_len))
		goto end;
	key_depth = key_len + 1;
	node_flag = rcu_dereference(ja->root);

	/* level 0: root node */
	if (!ja_node_ptr(node_flag))
		goto end;

	for (i = 1; i < key_depth; i++) {
		struct cds_ja_node *external_nodes;
		struct cds_ja_metadata *metadata;
		uint8_t iter_key;

		iter_key = *(key++);
		node_flag = ja_node_get_nth(node_flag, NULL, iter_key);
		dbg_printf("cds_ja_lookup iter key lookup %u finds node_flag %p\n",
				(unsigned int) iter_key, node_flag);

		/* Found no child for this key byte. */
		if (!ja_node_ptr(node_flag))
			break;
		/* Found external node. */
		if (!ja_node_internal(node_flag)) {
			match_len = i;
			match_node = (struct cds_ja_node *) node_flag;
			break;
		}
		/*
		 * Internal node: keep track of closest external node
		 * ancestor for partial match. This also covers the case
		 * where the complete match finds an internal node with
		 * associated external nodes.
		 */
		metadata = cds_ja_item_to_metadata(ja_node_ptr(node_flag));
		external_nodes = rcu_dereference(metadata->external_nodes);
		if (external_nodes) {
			match_len = i;
			match_node = external_nodes;
		}
	}
end:
	*_match_len = match_len;
	return match_node;
}

static
struct cds_ja_node *cds_ja_lookup_inequality(struct cds_ja *ja,
		const uint8_t *key, size_t _key_len,
		uint8_t *result_key, size_t *result_key_len,
		enum ja_lookup_inequality mode)
{
	int key_depth, level;
	struct cds_ja_inode_flag *node_flag, *cur_node_depth[JA_MAX_DEPTH];
	uint8_t cur_key[JA_MAX_DEPTH];	//TODO: JA_MAX_DEPTH - 1
	enum ja_direction dir;
	const uint8_t *iter_key = key;
	size_t key_len = ja_key_len(ja, _key_len);

	if (!valid_key_len(ja, key_len))
		return NULL;
	key_depth = key_len + 1;

	switch (mode) {
	case JA_LOOKUP_GE:
	case JA_LOOKUP_LE:
	case JA_LOOKUP_GT:
	case JA_LOOKUP_LT:
		break;
	default:
		return NULL;
	}

	memset(cur_node_depth, 0, sizeof(cur_node_depth));	//TODO: len = (key_depth + 1) * sizeof(void *)
	memset(cur_key, 0, sizeof(cur_key));			//TODO: len = (key_depth)
	node_flag = rcu_dereference(ja->root);
	cur_node_depth[0] = node_flag;

	/* level 0: root node */
	if (!ja_node_ptr(node_flag))
		return NULL;

	for (level = 1; level < key_depth; level++) {	//TODO var len
		uint8_t key_value;

		key_value = *(iter_key++);
		node_flag = ja_node_get_nth(node_flag, NULL, key_value);
		if (!ja_node_ptr(node_flag))
			break;
		cur_key[level - 1] = key_value;
		cur_node_depth[level] = node_flag;
		dbg_printf("cds_ja_lookup_inequality iter key lookup %u finds node_flag %p\n",
				(unsigned int) key_value, node_flag);
		if (!ja_node_internal(node_flag))
			break;
	}

	switch (mode) {
	case JA_LOOKUP_LE:
	case JA_LOOKUP_GE:
		if (level == key_depth) {
			struct cds_ja_node *external_nodes;

			if (ja_node_internal(node_flag)) {
				struct cds_ja_metadata *metadata;

				metadata = cds_ja_item_to_metadata(ja_node_ptr(node_flag));
				external_nodes = rcu_dereference(metadata->external_nodes);
			} else {
				external_nodes = (struct cds_ja_node *) node_flag;
			}
			if (external_nodes) {
				/* End of key lookup succeded. We got an equal match. */
				if (result_key)
					memcpy(result_key, key, key_len);
				if (result_key_len)
					*result_key_len = key_len;
				return external_nodes;
			}
		}
		break;
	case JA_LOOKUP_LT:
	case JA_LOOKUP_GT:
		break;
	default:
		assert(0);
	}

	/*
	 * Find highest value left/right of current node.
	 * Current node is cur_node_depth[level].
	 * Start at current level. If we cannot find any key left/right
	 * of ours, go one level up, seek highest value left/right of
	 * current (recursively), and when we find one, get the
	 * rightmost/leftmost child of its rightmost/leftmost child
	 * (recursively).
	 */
	switch (mode) {
	case JA_LOOKUP_LE:
	case JA_LOOKUP_LT:
		dir = JA_LEFT;
		break;
	case JA_LOOKUP_GE:
	case JA_LOOKUP_GT:
		dir = JA_RIGHT;
		break;
	default:
		assert(0);
	}
	for (; level > 0; level--) {
		uint8_t key_value;

		key_value = *(--iter_key);
		node_flag = ja_node_get_leftright(cur_node_depth[level - 1],
				key_value, &cur_key[level - 1], dir);
		dbg_printf("cds_ja_lookup_inequality find sibling from %u at %u finds node_flag %p\n",
				(unsigned int) key_value, (unsigned int) cur_key[level - 1],
				node_flag);
		/* If found left/right sibling, find rightmost/leftmost child. */
		if (ja_node_ptr(node_flag))
			break;
		/*
		 * Return external node if trying to find LE/LT
		 * inequality and encountering an external node when
		 * going upward.
		 */
		if (dir == JA_LEFT) {
			struct cds_ja_metadata *metadata = cds_ja_item_to_metadata(ja_node_ptr(cur_node_depth[level - 1]));
			struct cds_ja_node *external_nodes = rcu_dereference(metadata->external_nodes);

			if (external_nodes) {
				if (result_key) {
					int i;

					for (i = 1; i < level; i++)
						*(result_key++) = cur_key[i - 1];
				}
				if (result_key_len)
					*result_key_len = level - 1;
				return external_nodes;
			}
		}
	}

	if (!level) {
		/* Reached the root and could not find a left/right sibling. */
		return NULL;
	}

	level++;

	/*
	 * From this point, we are guaranteed to be able to find a
	 * "lower than"/"greater than" match. ja_attach_node() and
	 * ja_detach_node() both guarantee that it is not possible for a
	 * lookup to reach a dead-end.
	 */

	/*
	 * Find rightmost/leftmost child of rightmost/leftmost child
	 * (recursively).
	 */
	switch (mode) {
	case JA_LOOKUP_LE:
	case JA_LOOKUP_LT:
		dir = JA_RIGHTMOST;
		break;
	case JA_LOOKUP_GE:
	case JA_LOOKUP_GT:
		dir = JA_LEFTMOST;
		break;
	default:
		assert(0);
	}
	for (; level < key_depth; level++) {
		/*
		 * Return external node if trying to find GE/GT
		 * inequality and encountering an external node when
		 * going downward.
		 */
		if (dir == JA_LEFTMOST) {
			struct cds_ja_metadata *metadata = cds_ja_item_to_metadata(ja_node_ptr(node_flag));
			struct cds_ja_node *external_nodes = rcu_dereference(metadata->external_nodes);

			if (external_nodes) {
				if (result_key) {
					int i;

					for (i = 1; i < level; i++)
						*(result_key++) = cur_key[i - 1];
				}
				if (result_key_len)
					*result_key_len = level - 1;
				return external_nodes;
			}
		}
		node_flag = ja_node_get_minmax(node_flag, &cur_key[level - 1], dir);
		dbg_printf("cds_ja_lookup_inequality find minmax at %u finds node_flag %p\n",
				(unsigned int) cur_key[level - 1],
				node_flag);
		if (!ja_node_ptr(node_flag))
			break;
	}

	if (result_key) {
		int i;

		for (i = 1; i < level; i++)
			*(result_key++) = cur_key[i - 1];
	}
	if (result_key_len)
		*result_key_len = level - 1;
	return (struct cds_ja_node *) node_flag;
}

struct cds_ja_node *cds_ja_lookup_lower_equal(struct cds_ja *ja,
		const uint8_t *key, size_t key_len,
		uint8_t *result_key, size_t *result_key_len)

{
	dbg_printf("cds_ja_lookup_lower_equal\n");
	return cds_ja_lookup_inequality(ja, key, key_len,
			result_key, result_key_len, JA_LOOKUP_LE);
}

struct cds_ja_node *cds_ja_lookup_greater_equal(struct cds_ja *ja,
		const uint8_t *key, size_t key_len,
		uint8_t *result_key, size_t *result_key_len)
{
	dbg_printf("cds_ja_lookup_greater_equal\n");
	return cds_ja_lookup_inequality(ja, key, key_len,
		result_key, result_key_len, JA_LOOKUP_GE);
}

struct cds_ja_node *cds_ja_lookup_lower_than(struct cds_ja *ja,
		const uint8_t *key, size_t key_len,
		uint8_t *result_key, size_t *result_key_len)
{
	dbg_printf("cds_ja_lookup_lower_than\n");
	return cds_ja_lookup_inequality(ja, key, key_len,
		result_key, result_key_len, JA_LOOKUP_LT);
}

struct cds_ja_node *cds_ja_lookup_greater_than(struct cds_ja *ja,
		const uint8_t *key, size_t key_len,
		uint8_t *result_key, size_t *result_key_len)
{
	dbg_printf("cds_ja_lookup_greater_than\n");
	return cds_ja_lookup_inequality(ja, key, key_len,
		result_key, result_key_len, JA_LOOKUP_GT);
}

/*
 * We reached an unpopulated node. Create it and the children we need,
 * and then attach the entire branch to the current node. This may
 * trigger recompaction of the current node.
 *
 * ja_attach_node() ensures that a lookup will _never_ see a branch that
 * leads to a dead-end: before attaching a branch, the entire content of
 * the new branch is populated, thus creating a cluster, before
 * attaching the cluster to the rest of the tree, thus making it visible
 * to lookups.
 */
static
int ja_attach_node(struct cds_ja *ja,
		struct cds_ja_inode_flag **attach_node_flag_ptr,
		struct cds_ja_inode_flag *attach_node_flag,
		struct cds_ja_inode_flag **old_node_flag_ptr,
		struct cds_ja_inode_flag *old_node_flag,
		const uint8_t *key,
		size_t key_len,
		unsigned int level,
		struct cds_ja_node *child_node)
{
	struct cds_ja_metadata *metadata = NULL;
	struct cds_ja_inode_flag *iter_node_flag, *iter_dest_node_flag,
				*created_nodes[JA_MAX_DEPTH];
	int ret, i, nr_created_nodes = 0;
	const uint8_t *iter_key = key + ja->key_len;

	dbg_printf("Attach node at level %u (old_node_flag %p, attach_node_flag_ptr %p attach_node_flag %p)\n",
		level, old_node_flag, attach_node_flag_ptr, attach_node_flag);

	assert(!old_node_flag);
	if (attach_node_flag && level > 1)
		metadata = cds_ja_item_to_metadata(ja_node_ptr(attach_node_flag));

	/* Concurrent update prevented by mutual exclusion. */
	assert(!(old_node_flag_ptr && ja_node_ptr(*old_node_flag_ptr)));

	/* Concurrent update prevented by mutual exclusion. */
	assert(!(attach_node_flag_ptr && ja_node_ptr(*attach_node_flag_ptr) !=
			ja_node_ptr(attach_node_flag)));

	/* Create new branch, starting from bottom */
	iter_node_flag = (struct cds_ja_inode_flag *) child_node;

	for (i = key_len + 1; i >= (int) level; i--) {
		uint8_t key_value;

		key_value = *(--iter_key);
		dbg_printf("branch creation level %d, key %u\n",
				i, (unsigned int) key_value);
		iter_dest_node_flag = NULL;
		ret = ja_node_set_nth(ja, &iter_dest_node_flag, key_value, iter_node_flag, NULL);
		if (ret) {
			dbg_printf("branch creation error %d\n", ret);
			goto check_error;
		}
		created_nodes[nr_created_nodes++] = iter_dest_node_flag;
		iter_node_flag = iter_dest_node_flag;
	}
	assert(level > 0);

	/* Publish branch */
	if (level == 1) {
		/*
		 * Attaching to root node.
		 */
		rcu_assign_pointer(ja->root, iter_node_flag);
	} else {
		uint8_t key_value;

		key_value = *(--iter_key);
		dbg_printf("publish branch at level %d, key %u\n", level - 1, (unsigned int) key_value);
		/* We need to use set_nth on the previous level. */
		iter_dest_node_flag = attach_node_flag;
		ret = ja_node_set_nth(ja, &iter_dest_node_flag, key_value, iter_node_flag, metadata);
		if (ret) {
			dbg_printf("branch publish error %d\n", ret);
			goto check_error;
		}
		/*
		 * Attach branch
		 */
		rcu_assign_pointer(*attach_node_flag_ptr, iter_dest_node_flag);
	}

	/* Success */
	ret = 0;

check_error:
	if (ret) {
		for (i = 0; i < nr_created_nodes; i++)
			free_cds_ja_node(ja, ja_node_ptr(created_nodes[i]));
	}
	return ret;
}

static
void ja_chain_node(struct cds_ja_node *last_node, struct cds_ja_node *node)
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

static
int _cds_ja_add(struct cds_ja *ja,
		const uint8_t *key, size_t _key_len,
		struct cds_ja_node *node,
		struct cds_ja_node **unique_node_ret)
{
	unsigned int max_tree_depth, i;	//TODO var len
	struct cds_ja_inode_flag *attach_node_flag, *parent_node_flag,
		*parent2_node_flag, *node_flag;
	struct cds_ja_inode_flag **attach_node_flag_ptr,
		**parent_node_flag_ptr, **node_flag_ptr;
	const uint8_t *iter_key = key;
	size_t key_len = ja_key_len(ja, _key_len);
	int ret;

	if (!valid_external_node(node) || !valid_key_len(ja, key_len))
		return -EINVAL;

	key_depth = key_len + 1;

retry:
	dbg_printf("cds_ja_add attempt: node %p\n", node);
	parent2_node_flag = NULL;
	parent_node_flag = (struct cds_ja_inode_flag *) &ja->root;
	parent_node_flag_ptr = NULL;
	node_flag = rcu_dereference(ja->root);
	node_flag_ptr = &ja->root;

	for (i = 1; i < key_depth; i++) {
		uint8_t key_value;

		if (!ja_node_ptr(node_flag))
			break;
		/* Found external node before end of key. */
		if (i < key_depth - 1 && !ja_node_internal(node_flag))
			break;
		dbg_printf("cds_ja_add iter parent2_node_flag %p parent_node_flag %p node_flag_ptr %p node_flag %p\n",
				parent2_node_flag, parent_node_flag, node_flag_ptr, node_flag);
		key_value = *(iter_key++);
		parent2_node_flag = parent_node_flag;
		parent_node_flag = node_flag;
		parent_node_flag_ptr = node_flag_ptr;
		node_flag = ja_node_get_nth(node_flag, &node_flag_ptr, key_value);
	}

	/*
	 * We reached either end of key, external node, or internal NULL node,
	 * simply add node to last internal level, or chain it if key is
	 * already present.
	 * TODO: ....
	 */
	if (!ja_node_ptr(node_flag)) {
		dbg_printf("cds_ja_add NULL parent2_node_flag %p parent_node_flag %p node_flag_ptr %p node_flag %p\n",
				parent2_node_flag, parent_node_flag, node_flag_ptr, node_flag);

		attach_node_flag = parent_node_flag;
		attach_node_flag_ptr = parent_node_flag_ptr;

		ret = ja_attach_node(ja, attach_node_flag_ptr, attach_node_flag,
				node_flag_ptr, node_flag, key, key_len, i, node);
	} else {
		struct cds_ja_node *iter_node, *last_node = NULL;

		if (unique_node_ret) {
			*unique_node_ret = (struct cds_ja_node *) ja_node_ptr(node_flag);
			return -EEXIST;
		}

		/* Find last duplicate */
		iter_node = (struct cds_ja_node *) ja_node_ptr(node_flag);
		cds_ja_for_each_duplicate_rcu(iter_node)
			last_node = iter_node;

		dbg_printf("cds_ja_add duplicate parent2_node_flag %p parent_node_flag %p node_flag_ptr %p node_flag %p\n",
				parent2_node_flag, parent_node_flag, node_flag_ptr, node_flag);

		ja_chain_node(last_node, node);
		ret = 0;
	}
	if (ret == -EAGAIN || ret == -EEXIST)
		goto retry;

	return ret;
}

int cds_ja_add(struct cds_ja *ja, const uint8_t *key, size_t key_len,
		struct cds_ja_node *node)
{
	return _cds_ja_add(ja, key, key_len, node, NULL);
}

struct cds_ja_node *cds_ja_add_unique(struct cds_ja *ja, const uint8_t *key,
		size_t key_len, struct cds_ja_node *node)
{
	int ret;
	struct cds_ja_node *ret_node;

	ret = _cds_ja_add(ja, key, key_len, node, &ret_node);
	if (ret == -EEXIST)
		return ret_node;
	else
		return node;
}

/*
 * Note: there is no need to lookup the pointer address associated with
 * each node's nth item: it's already been done by cds_ja_del while
 * holding the rcu read-side lock, and our node rules ensure that when a
 * match value -> pointer is found in a node, it is _NEVER_ changed for
 * that node without recompaction, and recompaction reallocates the
 * node.
 * However, when a child is removed from "linear" nodes, its pointer
 * is set to NULL. We therefore check if this pointer is NULL, and
 * return -ENOENT to the caller if it is the case.
 *
 * ja_detach_node() ensures that a lookup will _never_ see a branch that
 * leads to a dead-end: when removing branch, it makes sure to perform
 * the "cut" at the highest node that has only one child, effectively
 * replacing it with a NULL pointer.
 */
static
int ja_detach_node(struct cds_ja *ja,
		struct cds_ja_inode_flag **snapshot,
		struct cds_ja_inode_flag ***snapshot_ptr,
		uint8_t *snapshot_n,
		int nr_snapshot)
{
	struct cds_ja_metadata *metadata_stack[JA_MAX_DEPTH];	//TODO var len
	struct cds_ja_inode_flag **node_flag_ptr = NULL,
			*parent_node_flag = NULL,
			**parent_node_flag_ptr = NULL;
	struct cds_ja_inode_flag *iter_node_flag;
	int ret, i, nr_metadata = 0, nr_clear = 0, nr_branch = 0;
	uint8_t n = 0;

	assert(nr_snapshot == (int)ja->max_tree_depth + 1);	//TODO var len

	/*
	 * From the last internal level node going up, lookup the
	 * metadata, check if the node has only one child left. If it is
	 * the case, we continue iterating upward. When we reach a node
	 * which has more that one child left, we lookup the parent, and
	 * proceed to the node deletion (removing its children too).
	 */
	for (i = nr_snapshot - 2; i >= 1; i--) {
		struct cds_ja_metadata *metadata;

		metadata = cds_ja_item_to_metadata(ja_node_ptr(snapshot[i]));
		metadata_stack[nr_metadata++] = metadata;
		assert(snapshot_ptr[i + 1]);
		/* Mutual exclusion prevents concurrent update. */
		assert(!(ja_node_ptr(*snapshot_ptr[i + 1])
				!= ja_node_ptr(snapshot[i + 1])));

		assert(metadata->nr_child > 0);
		if (metadata->nr_child == 1 && i > 1)
			nr_clear++;
		nr_branch++;
		if (metadata->nr_child > 1 || i == 1) {
			if (snapshot[i - 1] != (struct cds_ja_inode_flag *) &ja->root) {
				metadata = cds_ja_item_to_metadata(ja_node_ptr(snapshot[i - 1]));
			} else {
				metadata = &ja->root_metadata;
			}
			metadata_stack[nr_metadata++] = metadata;

			assert(snapshot_ptr[i]);
			/* Mutual exclusion prevents concurrent update. */
			assert(!(ja_node_ptr(*snapshot_ptr[i])
					!= ja_node_ptr(snapshot[i])));

			node_flag_ptr = snapshot_ptr[i + 1];
			n = snapshot_n[i + 1];
			parent_node_flag_ptr = snapshot_ptr[i];
			parent_node_flag = snapshot[i];
			break;
		}
	}

	/*
	 * At this point, we want to delete all nodes that are about to
	 * be removed from metadata_stack (except the last one, which is
	 * either the root or the parent of the upmost node with 1
	 * child).
	 */
	for (i = 0; i < nr_clear; i++)
		free_cds_ja_node(ja, cds_ja_metadata_to_item(metadata_stack[i]));

	iter_node_flag = parent_node_flag;
	/* Remove from parent */
	ret = ja_node_clear_ptr(ja,
		node_flag_ptr, 		/* Pointer to location to nullify */
		&iter_node_flag,	/* Old new parent ptr in its parent */
		metadata_stack[nr_branch - 1],	/* of parent */
		n, nr_branch - 1);
	if (ret)
		goto end;

	dbg_printf("ja_detach_node: publish %p instead of %p\n",
		iter_node_flag, *parent_node_flag_ptr);
	/* Update address of parent ptr in its parent */
	rcu_assign_pointer(*parent_node_flag_ptr, iter_node_flag);

end:
	return ret;
}

static
void ja_unchain_node(struct cds_ja_node **prev_node_ptr,
		struct cds_ja_node *node)
{
	uatomic_store(prev_node_ptr, node->next, CMM_RELAXED);
}

/*
 * Called with RCU read lock held.
 */
int cds_ja_del(struct cds_ja *ja, const uint8_t *key, size_t _key_len,
		struct cds_ja_node *node)
{
	unsigned int max_tree_depth, i;				//TODO var len
	struct cds_ja_inode_flag *snapshot[JA_MAX_DEPTH];	//TODO var len
	struct cds_ja_inode_flag **snapshot_ptr[JA_MAX_DEPTH];	//TODO var len
	uint8_t snapshot_n[JA_MAX_DEPTH];			//TODO var len
	struct cds_ja_inode_flag *node_flag;
	struct cds_ja_inode_flag **prev_node_flag_ptr,
		**node_flag_ptr;
	struct cds_ja_node *iter_node, **iter_node_ptr, **prev_node_ptr, *match;
	int nr_snapshot, ret, count = 0;
	const uint8_t *iter_key = key;
	size_t key_len = ja_key_len(ja, _key_len);

	if (!valid_external_node(node) || !valid_key_len(ja, key_len))
		return -EINVAL;

	max_tree_depth = ja->max_tree_depth;				//TODO var len

retry:
	nr_snapshot = 0;
	dbg_printf("cds_ja_del attempt: node %p\n", node);

	/* snapshot for level 0 is for metadata lookup of root node. */
	snapshot_n[0] = 0;
	snapshot_n[1] = 0;
	snapshot_ptr[nr_snapshot] = NULL;
	snapshot[nr_snapshot++] = (struct cds_ja_inode_flag *) &ja->root;
	node_flag = rcu_dereference(ja->root);
	prev_node_flag_ptr = &ja->root;
	node_flag_ptr = &ja->root;

	/* Iterate on all internal levels */
	for (i = 1; i < max_tree_depth; i++) {			//TODO var len
		uint8_t key_value;

		dbg_printf("cds_ja_del iter node_flag %p\n",
				node_flag);
		if (!ja_node_ptr(node_flag)) {
			return -ENOENT;
		}
		key_value = *(iter_key++);
		snapshot_n[nr_snapshot + 1] = key_value;
		snapshot_ptr[nr_snapshot] = prev_node_flag_ptr;
		snapshot[nr_snapshot++] = node_flag;
		node_flag = ja_node_get_nth(node_flag, &node_flag_ptr, key_value);
		if (node_flag)
			prev_node_flag_ptr = node_flag_ptr;
		dbg_printf("cds_ja_del iter key lookup %u finds node_flag %p, prev_node_flag_ptr %p\n",
				(unsigned int) key_value, node_flag,
				prev_node_flag_ptr);
	}
	/*
	 * We reached bottom of tree, try to find the node we are trying
	 * to remove. Fail if we cannot find it.
	 */
	if (!ja_node_ptr(node_flag)) {
		dbg_printf("cds_ja_del: no node found for key\n");
		return -ENOENT;
	}

	/*
	 * Find the previous node's next pointer pointing to our node,
	 * so we can update it.
	 */
	prev_node_ptr = NULL;
	iter_node_ptr = (struct cds_ja_node **) node_flag_ptr;
	iter_node = (struct cds_ja_node *) ja_node_ptr(node_flag);
	count = 0;
	match = NULL;
	cds_ja_for_each_duplicate(iter_node) {
		count++;
		if (match)
			continue;
		dbg_printf("cds_ja_del: compare %p with iter_node %p\n", node, iter_node);
		if (iter_node == node) {
			prev_node_ptr = iter_node_ptr;
			match = iter_node;
		}
		iter_node_ptr = &iter_node->next;
	}
	if (!match) {
		dbg_printf("cds_ja_del: no node match for node %p key\n", node);
		return -ENOENT;
	}
	assert(count > 0);

	if (count == 1) {
		/*
		 * Removing last of duplicates. Last snapshot
		 * does not have metadata (external leafs).
		 */
		snapshot_ptr[nr_snapshot] = prev_node_flag_ptr;
		snapshot[nr_snapshot++] = node_flag;
		ret = ja_detach_node(ja, snapshot, snapshot_ptr,
				snapshot_n, nr_snapshot);
	} else {
		ja_unchain_node(prev_node_ptr, match);
		ret = 0;
	}

	/*
	 * Explanation of -ENOENT handling: caused by concurrent delete
	 * between RCU lookup and actual removal. Need to re-do the
	 * lookup and removal attempt.
	 */
	if (ret == -EAGAIN || ret == -ENOENT)
		goto retry;
	return ret;
}

size_t cds_ja_key_len(const struct cds_ja *ja)
{
	return ja->key_len;
}

size_t cds_ja_max_key_len(const struct cds_ja *ja)
{
	return ja->max_key_len;
}

struct cds_ja_attr *cds_ja_attr_create(void)
{
	struct cds_ja_attr *attr = calloc(1, sizeof(struct cds_ja_attr));

	if (!attr)
		return NULL;
	attr->key_len = CDS_JA_DEFAULT_KEY_LEN;
	attr->max_key_len = CDS_JA_DEFAULT_MAX_KEY_LEN;
	return attr;
}

void cds_ja_attr_destroy(struct cds_ja_attr *attr)
{
	free(attr);
}

int cds_ja_attr_set_key_len(struct cds_ja_attr *attr, size_t key_len)
{
	attr->key_len = key_len;
	return 0;
}

int cds_ja_attr_set_max_key_len(struct cds_ja_attr *attr, size_t max_key_len)
{
	if (max_key_len > JA_MAX_KEY_LEN)
		return -EINVAL;
	attr->max_key_len = max_key_len;
	return 0;
}

struct cds_ja *_cds_ja_create(const struct cds_ja_attr *attr,
		const struct rcu_flavor_struct *flavor)
{
	struct cds_ja *ja;
	size_t key_len = CDS_JA_DEFAULT_KEY_LEN,
	       max_key_len = CDS_JA_DEFAULT_MAX_KEY_LEN;

	if (attr) {
		key_len = attr->key_len;
		max_key_len = attr->max_key_len;
	}
	/* ja->root is NULL */
	/* max_tree_depth 0 is for pointer to root node */
	if (max_key_len && key_len > max_key_len)
		return NULL;
	ja = calloc(sizeof(*ja), 1);
	if (!ja)
		return NULL;
	ja->key_len = key_len;
	ja->max_key_len = max_key_len;
	ja->max_tree_depth = max_key_len + 1;
	assert(ja->max_tree_depth <= JA_MAX_DEPTH);
	ja->flavor = flavor;
	return ja;
}

static
void print_debug_fallback_distribution(struct cds_ja *ja)
{
	int i;

	fprintf(stderr, "Fallback node distribution:\n");
	for (i = 0; i < JA_ENTRY_PER_NODE; i++) {
		if (!ja->node_fallback_count_distribution[i])
			continue;
		fprintf(stderr, "	%3u: %4lu\n",
			i, ja->node_fallback_count_distribution[i]);
	}
}

static
int ja_final_checks(struct cds_ja *ja)
{
	double fallback_ratio;
	unsigned long na, nf, nr_fallback;
	int ret = 0;

	if (!ja_debug_counters())
		return 0;

	fallback_ratio = (double) uatomic_read(&ja->nr_fallback);
	fallback_ratio /= (double) uatomic_read(&ja->nr_nodes_allocated);
	nr_fallback = uatomic_read(&ja->nr_fallback);
	if (nr_fallback)
		fprintf(stderr,
			"[warning] RCU Judy Array used %lu fallback node(s) (ratio: %g)\n",
			uatomic_read(&ja->nr_fallback),
			fallback_ratio);

	na = uatomic_read(&ja->nr_nodes_allocated);
	nf = uatomic_read(&ja->nr_nodes_freed);
	dbg_printf("Nodes allocated: %lu, Nodes freed: %lu.\n", na, nf);
	if (nr_fallback)
		print_debug_fallback_distribution(ja);

	if (na != nf) {
		fprintf(stderr, "[error] Judy array leaked %ld nodes. Allocated: %lu, freed: %lu.\n",
			(long) na - nf, na, nf);
		ret = -1;
	}
	return ret;
}

/*
 * There should be no more concurrent add, delete, nor look-up performed
 * on the Judy array while it is being destroyed (ensured by the
 * caller).
 */
int cds_ja_destroy(struct cds_ja *ja)
{
	const struct rcu_flavor_struct *flavor = ja->flavor;
	int ret;

	/* Wait for in-flight call_rcu free to complete. */
	flavor->barrier();
	cds_ja_free_all_arenas(ja);
	ret = ja_final_checks(ja);
	free(ja);

	return ret;
}
