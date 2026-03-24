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

#ifndef abs
#define abs_int(a)	((int) (a) > 0 ? (int) (a) : -((int) (a)))
#endif

#define CDS_FT_DEFAULT_MAX_KEY_LEN	FT_MAX_KEY_LEN
#define CDS_FT_DEFAULT_KEY_LEN		0

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
 * read-side. For linear and pool node configurations, it starts with a
 * byte counting the number of children in the node.  Then, the
 * node-specific data is placed.
 * For the pigeon configuration, the number of children is kept in the
 * metadata associated to node.
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

enum ft_direction {
	FT_LEFT,
	FT_RIGHT,
	FT_LEFTMOST,
	FT_RIGHTMOST,
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
	if (!key_len)
		return ft->key_len;
	/* Validate that explicit and implicit key lengths match for fixed length Fractal Trie. */
	if (ft->key_len && key_len != ft->key_len)
		return 0;
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

	assert(key_len > 0 && key_len <= 8);
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

	assert(key_len > 0 && key_len <= 8);
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

	assert(key_len > 0 && key_len <= 4);
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

	assert(key_len > 0 && key_len <= 4);
	/* Host endianness to big endian. */
	u.v32 = htobe32(v);
	/* Copy len LSB. */
	memcpy(key, u.array + sizeof(u.array) - key_len , key_len);
}

static
uint8_t key_to_ordinal(const struct cds_ft *ft, uint8_t key)
{
	if (caa_likely(ft->key_map.identity))
		return key;
	return ft->key_map.key_to_ordinal[key];
}

static
uint8_t ordinal_to_key(const struct cds_ft *ft, uint8_t ordinal)
{
	if (caa_likely(ft->key_map.identity))
		return ordinal;
	return ft->key_map.ordinal_to_key[ordinal];
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
	return !ft_node_internal((struct cds_ft_inode_flag *) node);
}

static
bool valid_key_len(struct cds_ft *ft, size_t key_len)
{
	size_t max_key_len = ft->max_key_len;

	if (!key_len)
		return false;
	if (max_key_len && key_len > max_key_len)
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

static
uint8_t *align_ptr_size(uint8_t *ptr)
{
	return (uint8_t *) FT_ALIGN((unsigned long) ptr, sizeof(void *));
}

static
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
static
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
	assert(type->type_class != FT_LINEAR || nr_child >= type->min_child);

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

static
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
	assert(type->type_class != FT_LINEAR || nr_child >= type->min_child);

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

static
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

static
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

static
struct cds_ft_inode *ft_pool_node_get_ith_pool(const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		uint8_t i)
{
	assert(type->type_class == FT_POOL);
	return (struct cds_ft_inode *)
		&node->u.data[(unsigned int) i << type->pool_size_order];
}

static
struct cds_ft_inode_flag *ft_pool_node_get_direction(const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		int n, uint8_t *result_key,
		enum ft_direction dir)
{
	unsigned int pool_nr;
	int match_v;
	struct cds_ft_inode_flag *match_node_flag = NULL;

	assert(type->type_class == FT_POOL);
	assert(dir == FT_LEFT || dir == FT_RIGHT);

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

static
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

static
struct cds_ft_inode_flag *ft_pigeon_node_get_direction(const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		int n, uint8_t *result_key,
		enum ft_direction dir)
{
	struct cds_ft_inode_flag **child_node_flag_ptr;
	struct cds_ft_inode_flag *child_node_flag;
	int i;

	assert(type->type_class == FT_PIGEON);
	assert(dir == FT_LEFT || dir == FT_RIGHT);

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
	return NULL;
}

static
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
static
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

static
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
		return ft_pool_node_get_direction(type, node, n, result_key, dir);
	case FT_PIGEON:
		return ft_pigeon_node_get_direction(type, node, n, result_key, dir);
	default:
		assert(0);
		return (void *) -1UL;
	}
}

static
struct cds_ft_inode_flag *ft_node_get_leftright(struct cds_ft_inode_flag *node_flag,
		unsigned int n, uint8_t *result_key,
		enum ft_direction dir)
{
	return ft_node_get_direction(node_flag, n, result_key, dir);
}

static
struct cds_ft_inode_flag *ft_node_get_minmax(struct cds_ft_inode_flag *node_flag,
		uint8_t *result_key,
		enum ft_direction dir)
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
	/* attach/detach semantic guarantees that ft_node_get_minmax cannot return NULL. */
	assert(ft_node_ptr(ret));
	return ret;
}

static
int ft_linear_node_set_nth(const struct cds_ft_type *type,
		struct cds_ft_inode *node,
		struct cds_ft_metadata *metadata,
		uint8_t n,
		struct cds_ft_inode_flag *child_node_flag)
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

	return ft_linear_node_set_nth(type, linear, metadata, n, child_node_flag);
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
	if (!replace_old_ptr)
		metadata->nr_child++;
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
		return ft_linear_node_set_nth(type, node, metadata, n, child_node_flag);
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

	return ft_linear_node_replace_ptr(type, linear, metadata, node_flag_ptr, newptr);
}

static
int ft_pigeon_node_replace_ptr(const struct cds_ft_type *type,
		struct cds_ft_metadata *metadata,
		struct cds_ft_inode_flag **node_flag_ptr,
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
	if (!newptr)
		metadata->nr_child--;
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
		return ft_pigeon_node_replace_ptr(type, metadata, node_flag_ptr, newptr);
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
		unsigned int nr_nodes)
{
	const struct cds_ft_type *type;

	assert(type_index != NODE_INDEX_NULL);
	if (nr_nodes == 0)
		return NODE_INDEX_NULL;
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
 * Return 0 on success, -EAGAIN if need to retry, or other negative
 * error value otherwise.
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
		struct cds_ft_inode_flag **nullify_node_flag_ptr)
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
			metadata->nr_child + 1);
		dbg_printf("Recompact for node with %u children\n",
			metadata->nr_child + 1);
		break;
	case FT_RECOMPACT_ADD_NEXT:
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
	case FT_RECOMPACT_DEL:
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
	if (old_node)
		free_cds_ft_node(ft, old_node);

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
 * Return 0 on success, -EAGAIN if need to retry, or other negative
 * error value otherwise.
 */
static
int ft_node_set_nth(struct cds_ft *ft,
		struct cds_ft_inode_flag **node_flag, uint8_t n,
		struct cds_ft_inode_flag *child_node_flag,
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
					metadata, node_flag, n, child_node_flag, NULL);
		break;
	case -ERANGE:
		/* Node needs to be recompacted. */
		ret = ft_node_recompact(FT_RECOMPACT_ADD_SAME, ft, type_index, type, node,
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
int ft_node_replace_ptr(struct cds_ft *ft,
		struct cds_ft_inode_flag **node_flag_ptr,		/* Pointer to location to nullify */
		struct cds_ft_inode_flag **parent_node_flag_ptr,	/* Address of parent ptr in its parent */
		struct cds_ft_metadata *metadata,			/* of parent */
		uint8_t n,
		struct cds_ft_inode_flag *newptr)
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
				node_flag_ptr);
	}
	return ret;
}

struct cds_ft_node *cds_ft_lookup(struct cds_ft *ft, const uint8_t *key, size_t _key_len)
{
	size_t key_len = ft_key_len(ft, _key_len);
	struct cds_ft_inode_flag *node_flag;
	unsigned int key_depth, i;

	if (!valid_key_len(ft, key_len))
		return NULL;
	key_depth = key_len + 1;
	node_flag = rcu_dereference(ft->root);

	/* level 0: root node */
	if (!ft_node_ptr(node_flag))
		return NULL;

	for (i = 1; i < key_depth; i++) {
		uint8_t iter_key;

		iter_key = key_to_ordinal(ft, *(key++));
		node_flag = ft_node_get_nth(node_flag, NULL, iter_key);
		dbg_printf("cds_ft_lookup iter key lookup %u finds node_flag %p\n",
				(unsigned int) iter_key, node_flag);
		if (!ft_node_ptr(node_flag))
			return NULL;
		/* Found external node before end of key. */
		if (i < key_depth - 1 && !ft_node_internal(node_flag))
			return NULL;
	}

	/*
	 * Reached key_depth, check for terminal node: either external
	 * nodes or internal node associated with external nodes.
	 */
	if (ft_node_internal(node_flag)) {
		struct cds_ft_metadata *metadata = cds_ft_item_to_metadata(ft_node_ptr(node_flag));
		return rcu_dereference(metadata->external_nodes);
	}
	return (struct cds_ft_node *) node_flag;
}

struct cds_ft_node *cds_ft_lookup_partial(struct cds_ft *ft, const uint8_t *key, size_t _key_len, size_t *_match_len)
{
	size_t key_len = ft_key_len(ft, _key_len), match_len = 0;
	struct cds_ft_node *match_node = NULL;
	struct cds_ft_inode_flag *node_flag;
	unsigned int key_depth, i;

	if (!valid_key_len(ft, key_len))
		goto end;
	key_depth = key_len + 1;
	node_flag = rcu_dereference(ft->root);

	/* level 0: root node */
	if (!ft_node_ptr(node_flag))
		goto end;

	for (i = 1; i < key_depth; i++) {
		struct cds_ft_node *external_nodes;
		struct cds_ft_metadata *metadata;
		uint8_t iter_key;

		iter_key = key_to_ordinal(ft, *(key++));
		node_flag = ft_node_get_nth(node_flag, NULL, iter_key);
		dbg_printf("cds_ft_lookup iter key lookup %u finds node_flag %p\n",
				(unsigned int) iter_key, node_flag);

		/* Found no child for this key byte. */
		if (!ft_node_ptr(node_flag))
			break;
		/* Found external node. */
		if (!ft_node_internal(node_flag)) {
			match_len = i;
			match_node = (struct cds_ft_node *) node_flag;
			break;
		}
		/*
		 * Internal node: keep track of closest external node
		 * ancestor for partial match. This also covers the case
		 * where the complete match finds an internal node with
		 * associated external nodes.
		 */
		metadata = cds_ft_item_to_metadata(ft_node_ptr(node_flag));
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
struct cds_ft_node *cds_ft_lookup_inequality(struct cds_ft *ft,
		const uint8_t *key, size_t _key_len,
		uint8_t *result_key, size_t *result_key_len,
		enum ft_lookup_inequality mode)
{
	int key_depth, level;
	struct cds_ft_inode_flag *node_flag, *cur_node_depth[FT_MAX_DEPTH];
	struct cds_ft_node *ret_node;
	uint8_t cur_key[FT_MAX_DEPTH - 1];
	enum ft_direction dir;
	const uint8_t *iter_key = key;
	size_t key_len = ft_key_len(ft, _key_len);
	bool going_up = false;

	if (!valid_key_len(ft, key_len))
		return NULL;
	key_depth = key_len + 1;

	switch (mode) {
	case FT_LOOKUP_GE:
	case FT_LOOKUP_LE:
	case FT_LOOKUP_GT:
	case FT_LOOKUP_LT:
		break;
	default:
		return NULL;
	}

	memset(cur_node_depth, 0, (ft->max_tree_depth + 1) * sizeof(cur_node_depth[0]));
	memset(cur_key, 0, ft->max_tree_depth * sizeof(cur_key[0]));
	node_flag = rcu_dereference(ft->root);
	cur_node_depth[0] = node_flag;

	/* level 0: root node */
	if (!ft_node_ptr(node_flag))
		return NULL;

	for (level = 1; level < key_depth; level++) {
		uint8_t key_value;

		key_value = key_to_ordinal(ft, *(iter_key++));
		node_flag = ft_node_get_nth(node_flag, NULL, key_value);
		if (!ft_node_ptr(node_flag))
			break;
		cur_key[level - 1] = key_value;
		cur_node_depth[level] = node_flag;
		dbg_printf("cds_ft_lookup_inequality iter key lookup %u finds node_flag %p\n",
				(unsigned int) key_value, node_flag);
		if (!ft_node_internal(node_flag))
			break;
	}

	switch (mode) {
	case FT_LOOKUP_LE:
	case FT_LOOKUP_GE:
		if (level == key_depth - 1) {
			struct cds_ft_node *external_nodes;

			if (ft_node_internal(node_flag)) {
				struct cds_ft_metadata *metadata;

				metadata = cds_ft_item_to_metadata(ft_node_ptr(node_flag));
				external_nodes = rcu_dereference(metadata->external_nodes);
			} else {
				external_nodes = (struct cds_ft_node *) node_flag;
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
	case FT_LOOKUP_LT:
	case FT_LOOKUP_GT:
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
	for (; level > 0; level--) {
		uint8_t key_value;

		/*
		 * Return external node if trying to find LE/LT
		 * inequality and encountering an external node when
		 * going upward.
		 */
		if (going_up && dir == FT_LEFT && ft_node_internal(cur_node_depth[level - 1])) {
			struct cds_ft_metadata *metadata = cds_ft_item_to_metadata(ft_node_ptr(cur_node_depth[level - 1]));
			struct cds_ft_node *external_nodes = rcu_dereference(metadata->external_nodes);

			if (external_nodes) {
				assert(!ft->key_len || level <= (int) ft->key_len);
				if (result_key) {
					int i;

					for (i = 0; i < level; i++)
						*(result_key++) = ordinal_to_key(ft, cur_key[i]);
				}
				if (result_key_len)
					*result_key_len = level;
				return external_nodes;
			}
		}

		key_value = key_to_ordinal(ft, *(--iter_key));
		node_flag = ft_node_get_leftright(cur_node_depth[level - 1],
				key_value, &cur_key[level - 1], dir);
		dbg_printf("cds_ft_lookup_inequality find sibling from %u at %u finds node_flag %p\n",
				(unsigned int) key_value, (unsigned int) cur_key[level - 1],
				node_flag);
		/* If found left/right sibling, find rightmost/leftmost child. */
		if (ft_node_ptr(node_flag))
			break;
		going_up = true;
	}

	if (!level) {
		/* Reached the root and could not find a left/right sibling. */
		return NULL;
	}

	if (!ft_node_internal(node_flag)) {
		assert(!ft->key_len || level <= (int) ft->key_len);
		if (result_key) {
			int i;

			for (i = 0; i < level; i++)
				*(result_key++) = ordinal_to_key(ft, cur_key[i]);
		}
		if (result_key_len)
			*result_key_len = level;
		return (struct cds_ft_node *) ft_node_ptr(node_flag);
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
	 */
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
	for (; level < (int) ft->max_tree_depth; level++) {
		/*
		 * Return external node associated to internal node if
		 * trying to find GE/GT inequality and encountering an
		 * external node when going downward.
		 */
		if (dir == FT_LEFTMOST && ft_node_internal(node_flag)) {
			struct cds_ft_metadata *metadata = cds_ft_item_to_metadata(ft_node_ptr(node_flag));
			struct cds_ft_node *external_nodes = rcu_dereference(metadata->external_nodes);

			if (external_nodes) {
				ret_node = external_nodes;
				goto end;
			}
		}
		/* Return external node. */
		if (!ft_node_internal(node_flag))
			break;
		node_flag = ft_node_get_minmax(node_flag, &cur_key[level - 1], dir);
		dbg_printf("cds_ft_lookup_inequality find minmax at %u finds node_flag %p\n",
				(unsigned int) cur_key[level - 1], node_flag);
		if (!ft_node_internal(node_flag))
			break;
	}
	/* attach/detach semantic guarantees that ft_node_get_minmax cannot return NULL. */
	assert(ft_node_ptr(node_flag));
	ret_node = (struct cds_ft_node *) node_flag;
end:
	if (result_key) {
		int i;

		for (i = 0; i < level; i++)
			*(result_key++) = ordinal_to_key(ft, cur_key[i]);
	}
	if (result_key_len)
		*result_key_len = level;
	return ret_node;
}

struct cds_ft_node *cds_ft_lookup_lower_equal(struct cds_ft *ft,
		const uint8_t *key, size_t key_len,
		uint8_t *result_key, size_t *result_key_len)

{
	dbg_printf("cds_ft_lookup_lower_equal\n");
	return cds_ft_lookup_inequality(ft, key, key_len,
			result_key, result_key_len, FT_LOOKUP_LE);
}

struct cds_ft_node *cds_ft_lookup_greater_equal(struct cds_ft *ft,
		const uint8_t *key, size_t key_len,
		uint8_t *result_key, size_t *result_key_len)
{
	dbg_printf("cds_ft_lookup_greater_equal\n");
	return cds_ft_lookup_inequality(ft, key, key_len,
		result_key, result_key_len, FT_LOOKUP_GE);
}

struct cds_ft_node *cds_ft_lookup_lower_than(struct cds_ft *ft,
		const uint8_t *key, size_t key_len,
		uint8_t *result_key, size_t *result_key_len)
{
	dbg_printf("cds_ft_lookup_lower_than\n");
	return cds_ft_lookup_inequality(ft, key, key_len,
		result_key, result_key_len, FT_LOOKUP_LT);
}

struct cds_ft_node *cds_ft_lookup_greater_than(struct cds_ft *ft,
		const uint8_t *key, size_t key_len,
		uint8_t *result_key, size_t *result_key_len)
{
	dbg_printf("cds_ft_lookup_greater_than\n");
	return cds_ft_lookup_inequality(ft, key, key_len,
		result_key, result_key_len, FT_LOOKUP_GT);
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
	int ret, i, nr_created_nodes = 0;
	const uint8_t *iter_key = key + key_len;

	dbg_printf("Attach node at level %u (old_node_flag %p, attach_node_flag_ptr %p attach_node_flag %p)\n",
		level, old_node_flag, attach_node_flag_ptr, attach_node_flag);

	assert(!old_node_flag || external_nodes);
	if (level == 0)
		metadata = &ft->root_metadata;
	else if (attach_node_flag)
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
		ret = ft_node_set_nth(ft, &iter_dest_node_flag, key_value, iter_node_flag, NULL);
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
	if (level == 0) {
		if (!ft_node_ptr(ft->root))
			metadata->nr_child++;
		/*
		 * Attaching to root node.
		 */
		rcu_assign_pointer(ft->root, iter_node_flag);
	} else {
		uint8_t key_value;

		key_value = key_to_ordinal(ft, *(--iter_key));
		dbg_printf("publish branch at level %d, key %u\n", level - 1, (unsigned int) key_value);
		/* We need to use set_nth on the previous level. */
		iter_dest_node_flag = attach_node_flag;
		ret = ft_node_set_nth(ft, &iter_dest_node_flag, key_value, iter_node_flag, metadata);
		if (ret) {
			dbg_printf("branch publish error %d\n", ret);
			goto check_error;
		}
		/* Attach branch. */
		rcu_assign_pointer(*attach_node_flag_ptr, iter_dest_node_flag);
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
int _cds_ft_add(struct cds_ft *ft,
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

	key_depth = key_len + 1;

retry:
	dbg_printf("cds_ft_add attempt: node %p\n", node);
	parent2_node_flag = NULL;
	parent_node_flag = (struct cds_ft_inode_flag *) &ft->root;
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
		dbg_printf("cds_ft_add iter parent2_node_flag %p parent_node_flag %p node_flag_ptr %p node_flag %p\n",
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
			dbg_printf("cds_ft_add NULL parent2_node_flag %p parent_node_flag %p node_flag_ptr %p node_flag %p\n",
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

				dbg_printf("cds_ft_add duplicate internal parent2_node_flag %p parent_node_flag %p node_flag_ptr %p node_flag %p\n",
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

			dbg_printf("cds_ft_add duplicate external parent2_node_flag %p parent_node_flag %p node_flag_ptr %p node_flag %p\n",
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

		dbg_printf("cds_ft_add NULL or external parent2_node_flag %p parent_node_flag %p node_flag_ptr %p node_flag %p\n",
				parent2_node_flag, parent_node_flag, node_flag_ptr, node_flag);

		attach_node_flag = parent_node_flag;
		attach_node_flag_ptr = parent_node_flag_ptr;

		ret = ft_attach_node(ft, attach_node_flag_ptr, attach_node_flag,
				node_flag_ptr, node_flag, key, key_len, i, node,
				(struct cds_ft_node *) ft_node_ptr(node_flag));
	}

	if (ret == -EAGAIN || ret == -EEXIST)
		goto retry;

	return ret;
}

int cds_ft_add(struct cds_ft *ft, const uint8_t *key, size_t key_len,
		struct cds_ft_node *node)
{
	return _cds_ft_add(ft, key, key_len, node, NULL);
}

struct cds_ft_node *cds_ft_add_unique(struct cds_ft *ft, const uint8_t *key,
		size_t key_len, struct cds_ft_node *node)
{
	int ret;
	struct cds_ft_node *ret_node;

	ret = _cds_ft_add(ft, key, key_len, node, &ret_node);
	if (ret == -EEXIST)
		return ret_node;
	else
		return node;
}

/*
 * Note: there is no need to lookup the pointer address associated with
 * each node's nth item: it's already been done by cds_ft_del, and
 * cds_ft_del is protected by mutual exclusion of updaters.
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
 */
static
int ft_detach_node(struct cds_ft *ft,
		struct cds_ft_inode_flag **snapshot,
		struct cds_ft_inode_flag ***snapshot_ptr,
		uint8_t *snapshot_n,
		int nr_snapshot)
{
	struct cds_ft_metadata *metadata_stack[FT_MAX_DEPTH];
	struct cds_ft_inode_flag **node_flag_ptr = NULL,
			*parent_node_flag = NULL,
			**parent_node_flag_ptr = NULL;
	struct cds_ft_inode_flag *iter_node_flag;
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
	for (i = nr_snapshot - 2; i >= 1; i--) {
		struct cds_ft_metadata *metadata;

		metadata = cds_ft_item_to_metadata(ft_node_ptr(snapshot[i]));
		metadata_stack[nr_metadata++] = metadata;
		assert(snapshot_ptr[i + 1]);
		/* Mutual exclusion prevents concurrent update. */
		assert(!(ft_node_ptr(*snapshot_ptr[i + 1])
				!= ft_node_ptr(snapshot[i + 1])));

		assert(metadata->nr_child > 0);
		if (!prev_external_nodes_found && (metadata->nr_child == 1 && i > 1)) {
			nr_clear++;
			/*
			 * Keep track of the external nodes pointer of
			 * the topmost internal node in the branch.
			 */
			topmost_external_nodes = metadata->external_nodes;
		}
		nr_branch++;
		if (prev_external_nodes_found || metadata->nr_child > 1 || i == 1) {
			if (snapshot[i - 1] != (struct cds_ft_inode_flag *) &ft->root) {
				metadata = cds_ft_item_to_metadata(ft_node_ptr(snapshot[i - 1]));
			} else {
				metadata = &ft->root_metadata;
			}
			metadata_stack[nr_metadata++] = metadata;

			assert(snapshot_ptr[i]);
			/* Mutual exclusion prevents concurrent update. */
			assert(!(ft_node_ptr(*snapshot_ptr[i])
					!= ft_node_ptr(snapshot[i])));

			node_flag_ptr = snapshot_ptr[i + 1];
			n = snapshot_n[i + 1];
			parent_node_flag_ptr = snapshot_ptr[i];
			parent_node_flag = snapshot[i];
			break;
		}
		if (topmost_external_nodes)
			prev_external_nodes_found = true;
	}

	/*
	 * At this point, we want to delete all nodes that are about to
	 * be removed from metadata_stack (except the last one, which is
	 * either the root or the parent of the topmost node with 1
	 * child).
	 */
	for (i = 0; i < nr_clear; i++)
		free_cds_ft_node(ft, cds_ft_metadata_to_item(metadata_stack[i]));

	iter_node_flag = parent_node_flag;
	/* Replace within parent */
	ret = ft_node_replace_ptr(ft,
		node_flag_ptr, 		/* Pointer to location to nullify */
		&iter_node_flag,	/* Old new parent ptr in its parent */
		metadata_stack[nr_branch - 1],	/* of parent */
		n, (struct cds_ft_inode_flag *) topmost_external_nodes);
	if (ret)
		goto end;

	dbg_printf("ft_detach_node: publish %p instead of %p\n",
		iter_node_flag, *parent_node_flag_ptr);
	if (parent_node_flag_ptr == &ft->root) {
		if (*parent_node_flag_ptr && !iter_node_flag)
			ft->root_metadata.nr_child--;
	}
	/* Update address of parent ptr in its parent */
	rcu_assign_pointer(*parent_node_flag_ptr, iter_node_flag);

end:
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
int cds_ft_del(struct cds_ft *ft, const uint8_t *key, size_t _key_len,
		struct cds_ft_node *node)
{
	unsigned int i, key_depth;
	struct cds_ft_inode_flag *snapshot[FT_MAX_DEPTH];
	struct cds_ft_inode_flag **snapshot_ptr[FT_MAX_DEPTH];
	uint8_t snapshot_n[FT_MAX_DEPTH];
	struct cds_ft_inode_flag *node_flag;
	struct cds_ft_inode_flag **prev_node_flag_ptr,
		**node_flag_ptr;
	struct cds_ft_node *iter_node, **iter_node_ptr, **prev_node_ptr, *match;
	int nr_snapshot, ret, count = 0;
	const uint8_t *iter_key = key;
	size_t key_len = ft_key_len(ft, _key_len);

	if (!valid_external_node(node) || !valid_key_len(ft, key_len))
		return -EINVAL;

	key_depth = key_len + 1;

retry:
	nr_snapshot = 0;
	dbg_printf("cds_ft_del attempt: node %p\n", node);

	/* snapshot for level 0 is for metadata lookup of root node. */
	snapshot_n[0] = 0;
	snapshot_n[1] = 0;
	snapshot_ptr[nr_snapshot] = NULL;
	snapshot[nr_snapshot++] = (struct cds_ft_inode_flag *) &ft->root;
	node_flag = rcu_dereference(ft->root);
	prev_node_flag_ptr = &ft->root;
	node_flag_ptr = &ft->root;

	/* Iterate on all internal levels */
	for (i = 1; i < key_depth; i++) {
		uint8_t key_value;

		dbg_printf("cds_ft_del iter node_flag %p\n",
				node_flag);
		if (!ft_node_ptr(node_flag)) {
			return -ENOENT;
		}
		key_value = key_to_ordinal(ft, *(iter_key++));
		snapshot_n[nr_snapshot + 1] = key_value;
		snapshot_ptr[nr_snapshot] = prev_node_flag_ptr;
		snapshot[nr_snapshot++] = node_flag;
		node_flag = ft_node_get_nth(node_flag, &node_flag_ptr, key_value);
		if (node_flag)
			prev_node_flag_ptr = node_flag_ptr;
		dbg_printf("cds_ft_del iter key lookup %u finds node_flag %p, prev_node_flag_ptr %p\n",
				(unsigned int) key_value, node_flag,
				prev_node_flag_ptr);
	}
	/*
	 * We reached end of key, try to find the node we are trying to
	 * remove. Fail if we cannot find it.
	 */
	if (!ft_node_ptr(node_flag)) {
		dbg_printf("cds_ft_del: no node found for key\n");
		return -ENOENT;
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
				dbg_printf("cds_ft_del: compare %p with iter_node %p\n", node, iter_node);
				if (iter_node == node) {
					prev_node_ptr = iter_node_ptr;
					match = iter_node;
				}
				iter_node_ptr = &iter_node->next;
			}
			if (!match) {
				dbg_printf("cds_ft_del: no node match for node %p key\n", node);
				return -ENOENT;
			}
			ft_unchain_node(prev_node_ptr, match);
			ret = 0;
		} else {
			dbg_printf("cds_ft_del: no metadata external node found for key\n");
			return -ENOENT;
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
			dbg_printf("cds_ft_del: compare %p with iter_node %p\n", node, iter_node);
			if (iter_node == node) {
				prev_node_ptr = iter_node_ptr;
				match = iter_node;
			}
			iter_node_ptr = &iter_node->next;
		}
		if (!match) {
			dbg_printf("cds_ft_del: no node match for node %p key\n", node);
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
			ret = ft_detach_node(ft, snapshot, snapshot_ptr,
					snapshot_n, nr_snapshot);
		} else {
			ft_unchain_node(prev_node_ptr, match);
			ret = 0;
		}
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

size_t cds_ft_key_len(const struct cds_ft *ft)
{
	return ft->key_len;
}

size_t cds_ft_max_key_len(const struct cds_ft *ft)
{
	return ft->max_key_len;
}

int cds_ft_key_map(struct cds_ft *ft, uint8_t *key_to_ordinal, uint8_t *ordinal_to_key)
{
	if (ft->key_map.identity)
		return -ENOENT;
	memcpy(key_to_ordinal, ft->key_map.key_to_ordinal, sizeof(ft->key_map.key_to_ordinal));
	memcpy(ordinal_to_key, ft->key_map.ordinal_to_key, sizeof(ft->key_map.ordinal_to_key));
	return 0;
}

struct cds_ft_attr *cds_ft_attr_create(void)
{
	struct cds_ft_attr *attr = calloc(1, sizeof(struct cds_ft_attr));

	if (!attr)
		return NULL;
	attr->key_len = CDS_FT_DEFAULT_KEY_LEN;
	attr->max_key_len = CDS_FT_DEFAULT_MAX_KEY_LEN;
	attr->key_map.identity = true;
	return attr;
}

void cds_ft_attr_destroy(struct cds_ft_attr *attr)
{
	free(attr);
}

int cds_ft_attr_set_key_len(struct cds_ft_attr *attr, size_t key_len)
{
	attr->key_len = key_len;
	return 0;
}

int cds_ft_attr_set_max_key_len(struct cds_ft_attr *attr, size_t max_key_len)
{
	if (max_key_len > FT_MAX_KEY_LEN)
		return -EINVAL;
	attr->max_key_len = max_key_len;
	return 0;
}

int cds_ft_attr_set_key_map(struct cds_ft_attr *attr, const uint8_t *key_to_ordinal, const uint8_t *ordinal_to_key)
{
	attr->key_map.identity = false;
	memcpy(attr->key_map.key_to_ordinal, key_to_ordinal, sizeof(attr->key_map.key_to_ordinal));
	memcpy(attr->key_map.ordinal_to_key, ordinal_to_key, sizeof(attr->key_map.ordinal_to_key));
	return 0;
}

struct cds_ft *_cds_ft_create(const struct cds_ft_attr *attr,
		const struct rcu_flavor_struct *flavor)
{
	struct cds_ft *ft;
	size_t key_len = CDS_FT_DEFAULT_KEY_LEN,
	       max_key_len = CDS_FT_DEFAULT_MAX_KEY_LEN;

	if (attr) {
		key_len = attr->key_len;
		max_key_len = attr->max_key_len;
	}
	/* ft->root is NULL */
	/* max_tree_depth 0 is for pointer to root node */
	if (max_key_len && key_len > max_key_len)
		return NULL;
	ft = calloc(1, sizeof(*ft));
	if (!ft)
		return NULL;
	ft->key_len = key_len;
	ft->max_key_len = max_key_len;
	ft->max_tree_depth = max_key_len + 1;
	assert(ft->max_tree_depth <= FT_MAX_DEPTH);
	ft->flavor = flavor;
	if (attr)
		ft->key_map = attr->key_map;
	else
		ft->key_map.identity = true;
	return ft;
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
int ft_final_checks(struct cds_ft *ft)
{
	double fallback_ratio;
	unsigned long na, nf, nr_fallback;
	int ret = 0;

	if (!ft_debug_counters())
		return 0;

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
		ret = -1;
	}
	return ret;
}

/*
 * There should be no more concurrent add, delete, nor look-up performed
 * on the Fractal Trie while it is being destroyed (ensured by the
 * caller).
 */
int cds_ft_destroy(struct cds_ft *ft)
{
	const struct rcu_flavor_struct *flavor = ft->flavor;
	int ret;

	/* Wait for in-flight call_rcu free to complete. */
	flavor->barrier();
	cds_ft_free_all_arenas(ft);
	ret = ft_final_checks(ft);
	free(ft);

	return ret;
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

	/* level 0: root node */
	if (ft_node_ptr(node_flag)) {
		print_indent(out, level);
		fprintf(out, "Level 0: root node %p\n", node_flag);
		show_node_recursive(ft, out, node_flag, level + 1);
	}
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

static
void calc_stats_node(const struct cds_ft *ft, struct cds_ft_inode_flag *node_flag, struct cds_ft_stats *stats, int level)
{
	unsigned long node_type = ft_node_type(node_flag);
	struct cds_ft_node_stats *node_stats = &stats->level[level].node_stats[node_type];
	const struct cds_ft_metadata *metadata;

	if (node_flag != ft->root)
		metadata = cds_ft_item_to_metadata(ft_node_ptr(node_flag));
	else
		metadata = &ft->root_metadata;
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

	/* level 0: root node */
	if (ft_node_ptr(node_flag)) {
		calc_stats_node(ft, node_flag, &stats, level);
		calc_stats_node_recursive(ft, node_flag, &stats, level + 1);
	}
	do_show_stats(ft, out, &stats);
}
