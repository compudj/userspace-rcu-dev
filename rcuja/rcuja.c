/*
 * rcuja/rcuja.c
 *
 * Userspace RCU library - RCU Judy Array
 *
 * Copyright 2012 - Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
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

#define _LGPL_SOURCE
#include <stdint.h>
#include <errno.h>
#include <limits.h>
#include <urcu/rcuja.h>
#include <urcu/compiler.h>
#include <urcu/arch.h>
#include <assert.h>
#include <urcu-pointer.h>
#include <urcu/uatomic.h>
#include <stdint.h>

#include "rcuja-internal.h"
#include "bitfield.h"

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
 * Number of least significant pointer bits reserved to represent the
 * child type.
 */
#define JA_TYPE_BITS	3
#define JA_TYPE_MAX_NR	(1UL << JA_TYPE_BITS)
#define JA_TYPE_MASK	(JA_TYPE_MAX_NR - 1)
#define JA_PTR_MASK	(~JA_TYPE_MASK)

#define JA_ENTRY_PER_NODE	256UL
#define JA_BITS_PER_BYTE	3

#define JA_MAX_DEPTH	5	/* Maximum depth, including leafs */

/*
 * Entry for NULL node is at index 8 of the table. It is never encoded
 * in flags.
 */
#define NODE_INDEX_NULL		8

/*
 * Number of removals needed on a fallback node before we try to shrink
 * it.
 */
#define JA_FALLBACK_REMOVAL_COUNT	8

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

#if (CAA_BITS_PER_LONG < 64)
/* 32-bit pointers */
enum {
	ja_type_0_max_child = 1,
	ja_type_1_max_child = 3,
	ja_type_2_max_child = 6,
	ja_type_3_max_child = 12,
	ja_type_4_max_child = 25,
	ja_type_5_max_child = 48,
	ja_type_6_max_child = 92,
	ja_type_7_max_child = 256,
	ja_type_8_max_child = 0,	/* NULL */
};

enum {
	ja_type_0_max_linear_child = 1,
	ja_type_1_max_linear_child = 3,
	ja_type_2_max_linear_child = 6,
	ja_type_3_max_linear_child = 12,
	ja_type_4_max_linear_child = 25,
	ja_type_5_max_linear_child = 24,
	ja_type_6_max_linear_child = 23,
};

enum {
	ja_type_5_nr_pool_order = 1,
	ja_type_6_nr_pool_order = 2,
};

const struct cds_ja_type ja_types[] = {
	{ .type_class = RCU_JA_LINEAR, .min_child = 1, .max_child = ja_type_0_max_child, .max_linear_child = ja_type_0_max_linear_child, .order = 3, },
	{ .type_class = RCU_JA_LINEAR, .min_child = 1, .max_child = ja_type_1_max_child, .max_linear_child = ja_type_1_max_linear_child, .order = 4, },
	{ .type_class = RCU_JA_LINEAR, .min_child = 3, .max_child = ja_type_2_max_child, .max_linear_child = ja_type_2_max_linear_child, .order = 5, },
	{ .type_class = RCU_JA_LINEAR, .min_child = 4, .max_child = ja_type_3_max_child, .max_linear_child = ja_type_3_max_linear_child, .order = 6, },
	{ .type_class = RCU_JA_LINEAR, .min_child = 10, .max_child = ja_type_4_max_child, .max_linear_child = ja_type_4_max_linear_child, .order = 7, },

	/* Pools may fill sooner than max_child */
	{ .type_class = RCU_JA_POOL, .min_child = 20, .max_child = ja_type_5_max_child, .max_linear_child = ja_type_5_max_linear_child, .order = 8, .nr_pool_order = ja_type_5_nr_pool_order, .pool_size_order = 7, },
	{ .type_class = RCU_JA_POOL, .min_child = 45, .max_child = ja_type_6_max_child, .max_linear_child = ja_type_6_max_linear_child, .order = 9, .nr_pool_order = ja_type_6_nr_pool_order, .pool_size_order = 7, },

	/*
	 * TODO: Upon node removal below min_child, if child pool is
	 * filled beyond capacity, we need to roll back to pigeon.
	 */
	{ .type_class = RCU_JA_PIGEON, .min_child = 89, .max_child = ja_type_7_max_child, .order = 10, },

	{ .type_class = RCU_JA_NULL, .min_child = 0, .max_child = ja_type_8_max_child, },
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
	{ .type_class = RCU_JA_LINEAR, .min_child = 1, .max_child = ja_type_0_max_child, .max_linear_child = ja_type_0_max_linear_child, .order = 4, },
	{ .type_class = RCU_JA_LINEAR, .min_child = 1, .max_child = ja_type_1_max_child, .max_linear_child = ja_type_1_max_linear_child, .order = 5, },
	{ .type_class = RCU_JA_LINEAR, .min_child = 3, .max_child = ja_type_2_max_child, .max_linear_child = ja_type_2_max_linear_child, .order = 6, },
	{ .type_class = RCU_JA_LINEAR, .min_child = 5, .max_child = ja_type_3_max_child, .max_linear_child = ja_type_3_max_linear_child, .order = 7, },
	{ .type_class = RCU_JA_LINEAR, .min_child = 10, .max_child = ja_type_4_max_child, .max_linear_child = ja_type_4_max_linear_child, .order = 8, },

	/* Pools may fill sooner than max_child. */
	{ .type_class = RCU_JA_POOL, .min_child = 22, .max_child = ja_type_5_max_child, .max_linear_child = ja_type_5_max_linear_child, .order = 9, .nr_pool_order = ja_type_5_nr_pool_order, .pool_size_order = 8, },
	{ .type_class = RCU_JA_POOL, .min_child = 51, .max_child = ja_type_6_max_child, .max_linear_child = ja_type_6_max_linear_child, .order = 10, .nr_pool_order = ja_type_6_nr_pool_order, .pool_size_order = 8, },

	/*
	 * TODO: Upon node removal below min_child, if child pool is
	 * filled beyond capacity, we need to roll back to pigeon.
	 */
	{ .type_class = RCU_JA_PIGEON, .min_child = 101, .max_child = ja_type_7_max_child, .order = 11, },

	{ .type_class = RCU_JA_NULL, .min_child = 0, .max_child = ja_type_8_max_child, },
};
#endif /* !(BITS_PER_LONG < 64) */

static inline __attribute__((unused))
void static_array_size_check(void)
{
	CAA_BUILD_BUG_ON(CAA_ARRAY_SIZE(ja_types) < JA_TYPE_MAX_NR);
}

/*
 * The cds_ja_node contains the compressed node data needed for
 * read-side. For linear and pool node configurations, it starts with a
 * byte counting the number of children in the node.  Then, the
 * node-specific data is placed.
 * The node mutex, if any is needed, protecting concurrent updated of
 * each node is placed in a separate hash table indexed by node address.
 * For the pigeon configuration, the number of children is also kept in
 * a separate hash table, indexed by node address, because it is only
 * required for updates.
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

static
struct cds_ja_inode_flag *ja_node_flag(struct cds_ja_inode *node,
		unsigned long type)
{
	assert(type < (1UL << JA_TYPE_BITS));
	return (struct cds_ja_inode_flag *) (((unsigned long) node) | type);
}

static
struct cds_ja_inode *ja_node_ptr(struct cds_ja_inode_flag *node)
{
	return (struct cds_ja_inode *) (((unsigned long) node) & JA_PTR_MASK);
}

static
unsigned long ja_node_type(struct cds_ja_inode_flag *node)
{
	unsigned long type;

	if (ja_node_ptr(node) == NULL) {
		return NODE_INDEX_NULL;
	}
	type = (unsigned int) ((unsigned long) node & JA_TYPE_MASK);
	assert(type < (1UL << JA_TYPE_BITS));
	return type;
}

struct cds_ja_inode *alloc_cds_ja_node(const struct cds_ja_type *ja_type)
{
	return calloc(1U << ja_type->order, sizeof(char));
}

void free_cds_ja_node(struct cds_ja_inode *node)
{
	free(node);
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
	return CMM_LOAD_SHARED(node->u.data[0]);
}

/*
 * The order in which values and pointers are does does not matter: if
 * a value is missing, we return NULL. If a value is there, but its
 * associated pointers is still NULL, we return NULL too.
 */
static
struct cds_ja_inode_flag *ja_linear_node_get_nth(const struct cds_ja_type *type,
		struct cds_ja_inode *node,
		struct cds_ja_inode_flag ***child_node_flag_ptr,
		uint8_t n)
{
	uint8_t nr_child;
	uint8_t *values;
	struct cds_ja_inode_flag **pointers;
	struct cds_ja_inode_flag *ptr;
	unsigned int i;

	assert(type->type_class == RCU_JA_LINEAR || type->type_class == RCU_JA_POOL);

	nr_child = ja_linear_node_get_nr_child(type, node);
	cmm_smp_rmb();	/* read nr_child before values and pointers */
	assert(nr_child <= type->max_linear_child);
	assert(type->type_class != RCU_JA_LINEAR || nr_child >= type->min_child);

	values = &node->u.data[1];
	for (i = 0; i < nr_child; i++) {
		if (CMM_LOAD_SHARED(values[i]) == n)
			break;
	}
	if (i >= nr_child)
		return NULL;
	pointers = (struct cds_ja_inode_flag **) align_ptr_size(&values[type->max_linear_child]);
	if (caa_unlikely(child_node_flag_ptr))
		*child_node_flag_ptr = &pointers[i];
	ptr = rcu_dereference(pointers[i]);
	assert(ja_node_ptr(ptr) != NULL);
	return ptr;
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
	*iter = pointers[i];
}

static
struct cds_ja_inode_flag *ja_pool_node_get_nth(const struct cds_ja_type *type,
		struct cds_ja_inode *node,
		struct cds_ja_inode_flag ***child_node_flag_ptr,
		uint8_t n)
{
	struct cds_ja_inode *linear;

	assert(type->type_class == RCU_JA_POOL);
	/*
	 * TODO: currently, we select the pool by highest bits. We
	 * should support various encodings.
	 */
	linear = (struct cds_ja_inode *)
		&node->u.data[((unsigned long) n >> (CHAR_BIT - type->nr_pool_order)) << type->pool_size_order];
	return ja_linear_node_get_nth(type, linear, child_node_flag_ptr, n);
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
struct cds_ja_inode_flag *ja_pigeon_node_get_nth(const struct cds_ja_type *type,
		struct cds_ja_inode *node,
		struct cds_ja_inode_flag ***child_node_flag_ptr,
		uint8_t n)
{
	struct cds_ja_inode_flag **child_node_flag;

	assert(type->type_class == RCU_JA_PIGEON);
	child_node_flag = &((struct cds_ja_inode_flag **) node->u.data)[n];
	if (caa_unlikely(child_node_flag_ptr))
		*child_node_flag_ptr = child_node_flag;
	return rcu_dereference(*child_node_flag);
}

/*
 * ja_node_get_nth: get nth item from a node.
 * node_flag is already rcu_dereference'd.
 */
static
struct cds_ja_inode_flag * ja_node_get_nth(struct cds_ja_inode_flag *node_flag,
		struct cds_ja_inode_flag ***child_node_flag_ptr,
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
				child_node_flag_ptr, n);
	case RCU_JA_POOL:
		return ja_pool_node_get_nth(type, node,
				child_node_flag_ptr, n);
	case RCU_JA_PIGEON:
		return ja_pigeon_node_get_nth(type, node,
				child_node_flag_ptr, n);
	default:
		assert(0);
		return (void *) -1UL;
	}
}

/*
 * TODO: use ja_get_nr_child to monitor limits triggering shrink
 * recompaction.
 * Also use ja_get_nr_child to make the difference between resize and
 * pool change of compaction bit(s).
 */
static
unsigned int ja_get_nr_child(struct cds_ja_shadow_node *shadow_node)
{
	return shadow_node->nr_child;
}

static
int ja_linear_node_set_nth(const struct cds_ja_type *type,
		struct cds_ja_inode *node,
		struct cds_ja_shadow_node *shadow_node,
		uint8_t n,
		struct cds_ja_inode_flag *child_node_flag)
{
	uint8_t nr_child;
	uint8_t *values, *nr_child_ptr;
	struct cds_ja_inode_flag **pointers;
	unsigned int i;

	assert(type->type_class == RCU_JA_LINEAR || type->type_class == RCU_JA_POOL);

	nr_child_ptr = &node->u.data[0];
	dbg_printf("linear set nth: nr_child_ptr %p\n", nr_child_ptr);
	nr_child = *nr_child_ptr;
	assert(nr_child <= type->max_linear_child);

	values = &node->u.data[1];
	for (i = 0; i < nr_child; i++) {
		if (values[i] == n)
			return -EEXIST;
	}
	if (nr_child >= type->max_linear_child) {
		/* No space left in this node type */
		return -ENOSPC;
	}
	pointers = (struct cds_ja_inode_flag **) align_ptr_size(&values[type->max_linear_child]);
	assert(pointers[nr_child] == NULL);
	rcu_assign_pointer(pointers[nr_child], child_node_flag);
	CMM_STORE_SHARED(values[nr_child], n);
	cmm_smp_wmb();	/* write value and pointer before nr_child */
	CMM_STORE_SHARED(*nr_child_ptr, nr_child + 1);
	shadow_node->nr_child++;
	dbg_printf("linear set nth: %u child, shadow: %u child, for node %p shadow %p\n",
		(unsigned int) CMM_LOAD_SHARED(*nr_child_ptr),
		(unsigned int) shadow_node->nr_child,
		node, shadow_node);

	return 0;
}

static
int ja_pool_node_set_nth(const struct cds_ja_type *type,
		struct cds_ja_inode *node,
		struct cds_ja_shadow_node *shadow_node,
		uint8_t n,
		struct cds_ja_inode_flag *child_node_flag)
{
	struct cds_ja_inode *linear;

	assert(type->type_class == RCU_JA_POOL);
	linear = (struct cds_ja_inode *)
		&node->u.data[((unsigned long) n >> (CHAR_BIT - type->nr_pool_order)) << type->pool_size_order];
	return ja_linear_node_set_nth(type, linear, shadow_node,
			n, child_node_flag);
}

static
int ja_pigeon_node_set_nth(const struct cds_ja_type *type,
		struct cds_ja_inode *node,
		struct cds_ja_shadow_node *shadow_node,
		uint8_t n,
		struct cds_ja_inode_flag *child_node_flag)
{
	struct cds_ja_inode_flag **ptr;

	assert(type->type_class == RCU_JA_PIGEON);
	ptr = &((struct cds_ja_inode_flag **) node->u.data)[n];
	if (*ptr)
		return -EEXIST;
	rcu_assign_pointer(*ptr, child_node_flag);
	shadow_node->nr_child++;
	return 0;
}

/*
 * _ja_node_set_nth: set nth item within a node. Return an error
 * (negative error value) if it is already there.
 * TODO: exclusive access on node.
 */
static
int _ja_node_set_nth(const struct cds_ja_type *type,
		struct cds_ja_inode *node,
		struct cds_ja_shadow_node *shadow_node,
		uint8_t n,
		struct cds_ja_inode_flag *child_node_flag)
{
	switch (type->type_class) {
	case RCU_JA_LINEAR:
		return ja_linear_node_set_nth(type, node, shadow_node, n,
				child_node_flag);
	case RCU_JA_POOL:
		return ja_pool_node_set_nth(type, node, shadow_node, n,
				child_node_flag);
	case RCU_JA_PIGEON:
		return ja_pigeon_node_set_nth(type, node, shadow_node, n,
				child_node_flag);
	case RCU_JA_NULL:
		return -ENOSPC;
	default:
		assert(0);
		return -EINVAL;
	}

	return 0;
}

/*
 * ja_node_recompact_add: recompact a node, adding a new child.
 * TODO: for pool type, take selection bit(s) into account.
 * Return 0 on success, -ENOENT if need to retry, or other negative
 * error value otherwise.
 */
static
int ja_node_recompact_add(struct cds_ja *ja,
		unsigned int old_type_index,
		const struct cds_ja_type *old_type,
		struct cds_ja_inode *old_node,
		struct cds_ja_shadow_node *shadow_node,
		struct cds_ja_inode_flag **old_node_flag, uint8_t n,
		struct cds_ja_inode_flag *child_node_flag)
{
	unsigned int new_type_index;
	struct cds_ja_inode *new_node;
	struct cds_ja_shadow_node *new_shadow_node;
	const struct cds_ja_type *new_type;
	struct cds_ja_inode_flag *new_node_flag;
	int ret;
	int fallback = 0;

	if (!shadow_node || old_type_index == NODE_INDEX_NULL) {
		new_type_index = 0;
	} else {
		new_type_index = old_type_index + 1;
	}

retry:		/* for fallback */
	dbg_printf("Recompact to type %d\n", new_type_index);
	new_type = &ja_types[new_type_index];
	new_node = alloc_cds_ja_node(new_type);
	if (!new_node)
		return -ENOMEM;
	new_node_flag = ja_node_flag(new_node, new_type_index);

	dbg_printf("Recompact inherit lock from %p\n", shadow_node);
	new_shadow_node = rcuja_shadow_set(ja->ht, new_node, shadow_node);
	if (!new_shadow_node) {
		free(new_node);
		return -ENOMEM;
	}
	if (fallback)
		new_shadow_node->fallback_removal_count =
					JA_FALLBACK_REMOVAL_COUNT;

	assert(old_type->type_class != RCU_JA_PIGEON);
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
			ret = _ja_node_set_nth(new_type, new_node,
					new_shadow_node,
					v, iter);
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
				ret = _ja_node_set_nth(new_type, new_node,
						new_shadow_node,
						v, iter);
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
		/* Nothing to copy */
		break;
	case RCU_JA_PIGEON:
	default:
		assert(0);
		ret = -EINVAL;
		goto end;
	}

	/* add node */
	ret = _ja_node_set_nth(new_type, new_node,
			new_shadow_node,
			n, child_node_flag);
	assert(!ret);
	/* Return pointer to new recompacted new through old_node_flag */
	*old_node_flag = new_node_flag;
	if (old_node) {
		ret = rcuja_shadow_clear(ja->ht, old_node, shadow_node,
				RCUJA_SHADOW_CLEAR_FREE_NODE);
		assert(!ret);
	}

	ret = 0;
end:
	return ret;

fallback_toosmall:
	/* fallback if next pool is too small */
	ret = rcuja_shadow_clear(ja->ht, new_node, new_shadow_node,
			RCUJA_SHADOW_CLEAR_FREE_NODE);
	assert(!ret);

	/* Last type: pigeon */
	new_type_index = (1UL << JA_TYPE_BITS) - 1;
	dbg_printf("Fallback to type %d\n", new_type_index);
	uatomic_inc(&ja->nr_fallback);
	fallback = 1;
	goto retry;
}

/*
 * Return 0 on success, -ENOENT if need to retry, or other negative
 * error value otherwise.
 */
static
int ja_node_set_nth(struct cds_ja *ja,
		struct cds_ja_inode_flag **node_flag, uint8_t n,
		struct cds_ja_inode_flag *child_node_flag,
		struct cds_ja_shadow_node *shadow_node)
{
	int ret;
	unsigned int type_index;
	const struct cds_ja_type *type;
	struct cds_ja_inode *node;

	dbg_printf("ja_node_set_nth for n=%u, node %p, shadow %p\n",
		(unsigned int) n, ja_node_ptr(*node_flag), shadow_node);

	node = ja_node_ptr(*node_flag);
	type_index = ja_node_type(*node_flag);
	type = &ja_types[type_index];
	ret = _ja_node_set_nth(type, node, shadow_node,
			n, child_node_flag);
	if (ret == -ENOSPC) {
		/* Not enough space in node, need to recompact. */
		ret = ja_node_recompact_add(ja, type_index, type, node,
				shadow_node, node_flag, n, child_node_flag);
	}
	return ret;
}

struct cds_hlist_head *cds_ja_lookup(struct cds_ja *ja, uint64_t key)
{
	unsigned int tree_depth, i;
	struct cds_ja_inode_flag *node_flag;

	if (caa_unlikely(key > ja->key_max))
		return NULL;
	tree_depth = ja->tree_depth;
	node_flag = rcu_dereference(ja->root);

	/* level 0: root node */
	if (!ja_node_ptr(node_flag))
		return NULL;

	for (i = 1; i < tree_depth; i++) {
		uint8_t iter_key;

		iter_key = (uint8_t) (key >> (JA_BITS_PER_BYTE * (tree_depth - i - 1)));
		node_flag = ja_node_get_nth(node_flag, NULL,
			iter_key);
		if (!ja_node_ptr(node_flag))
			return NULL;
	}

	/* Last level lookup succeded. We got an actual match. */
	return (struct cds_hlist_head *) node_flag;
}

/*
 * We reached an unpopulated node. Create it and the children we need,
 * and then attach the entire branch to the current node. This may
 * trigger recompaction of the current node.  Locks needed: node lock
 * (for add), and, possibly, parent node lock (to update pointer due to
 * node recompaction).
 *
 * First take node lock, check if recompaction is needed, then take
 * parent lock (if needed).  Then we can proceed to create the new
 * branch. Publish the new branch, and release locks.
 * TODO: we currently always take the parent lock even when not needed.
 */
static
int ja_attach_node(struct cds_ja *ja,
		struct cds_ja_inode_flag **node_flag_ptr,
		struct cds_ja_inode_flag *node_flag,
		struct cds_ja_inode_flag *parent_node_flag,
		uint64_t key,
		unsigned int level,
		struct cds_ja_node *child_node)
{
	struct cds_ja_shadow_node *shadow_node = NULL,
			*parent_shadow_node = NULL,
			*iter_shadow_node;
	struct cds_ja_inode *node = ja_node_ptr(node_flag);
	struct cds_ja_inode *parent_node = ja_node_ptr(parent_node_flag);
	struct cds_hlist_head head;
	struct cds_ja_inode_flag *iter_node_flag, *iter_dest_node_flag;
	int ret, i;
	struct cds_ja_inode_flag *created_nodes[JA_MAX_DEPTH];
	int nr_created_nodes = 0;

	dbg_printf("Attach node at level %u\n", level);

	assert(node);
	shadow_node = rcuja_shadow_lookup_lock(ja->ht, node);
	if (!shadow_node) {
		ret = -ENOENT;
		goto end;
	}
	if (parent_node) {
		parent_shadow_node = rcuja_shadow_lookup_lock(ja->ht,
						parent_node);
		if (!parent_shadow_node) {
			ret = -ENOENT;
			goto unlock_shadow;
		}
	}

	/* Create new branch, starting from bottom */
	CDS_INIT_HLIST_HEAD(&head);
	cds_hlist_add_head_rcu(&child_node->list, &head);
	iter_node_flag = (struct cds_ja_inode_flag *) head.next;

	/* Create shadow node for the leaf node */
	dbg_printf("leaf shadow node creation\n");
	iter_shadow_node = rcuja_shadow_set(ja->ht,
			ja_node_ptr(iter_node_flag), NULL);
	if (!iter_shadow_node) {
		ret = -ENOMEM;
		goto check_error;
	}
	created_nodes[nr_created_nodes++] = iter_node_flag;

	for (i = ja->tree_depth; i > (int) level; i--) {
		uint8_t iter_key;

		iter_key = (uint8_t) (key >> (JA_BITS_PER_BYTE * (ja->tree_depth - i)));
		dbg_printf("branch creation level %d, key %u\n",
				i - 1, (unsigned int) iter_key);
		iter_dest_node_flag = NULL;
		ret = ja_node_set_nth(ja, &iter_dest_node_flag,
			iter_key,
			iter_node_flag,
			NULL);
		if (ret)
			goto check_error;
		created_nodes[nr_created_nodes++] = iter_dest_node_flag;
		iter_node_flag = iter_dest_node_flag;
	}

	if (level > 1) {
		uint8_t iter_key;

		iter_key = (uint8_t) (key >> (JA_BITS_PER_BYTE * (ja->tree_depth - level)));
		/* We need to use set_nth on the previous level. */
		iter_dest_node_flag = node_flag;
		ret = ja_node_set_nth(ja, &iter_dest_node_flag,
			iter_key,
			iter_node_flag,
			shadow_node);
		if (ret)
			goto check_error;
		created_nodes[nr_created_nodes++] = iter_dest_node_flag;
		iter_node_flag = iter_dest_node_flag;
	}

	/* Publish new branch */
	dbg_printf("Publish branch %p, replacing %p\n",
		iter_node_flag, *node_flag_ptr);
	rcu_assign_pointer(*node_flag_ptr, iter_node_flag);

	/* Success */
	ret = 0;

check_error:
	if (ret) {
		for (i = 0; i < nr_created_nodes; i++) {
			int tmpret;
			int flags;

			flags = RCUJA_SHADOW_CLEAR_FREE_LOCK;
			if (i)
				flags |= RCUJA_SHADOW_CLEAR_FREE_NODE;
			tmpret = rcuja_shadow_clear(ja->ht,
					ja_node_ptr(created_nodes[i]),
					NULL,
					flags);
			assert(!tmpret);
		}
	}
	if (parent_shadow_node)
		rcuja_shadow_unlock(parent_shadow_node);
unlock_shadow:
	if (shadow_node)
		rcuja_shadow_unlock(shadow_node);
end:
	return ret;
}

/*
 * Lock the hlist head shadow node mutex, and add node to list of
 * duplicates. Failure can happen if concurrent removal removes the last
 * node with same key before we get the lock.
 * Return 0 on success, negative error value on failure.
 */
static
int ja_chain_node(struct cds_ja *ja,
		struct cds_hlist_head *head,
		struct cds_ja_node *node)
{
	struct cds_ja_shadow_node *shadow_node;

	shadow_node = rcuja_shadow_lookup_lock(ja->ht,
		(struct cds_ja_inode *) head);
	if (!shadow_node)
		return -ENOENT;
	cds_hlist_add_head_rcu(&node->list, head);
	rcuja_shadow_unlock(shadow_node);
	return 0;
}

int cds_ja_add(struct cds_ja *ja, uint64_t key,
		struct cds_ja_node *new_node)
{
	unsigned int tree_depth, i;
	struct cds_ja_inode_flag **node_flag_ptr;	/* in parent */
	struct cds_ja_inode_flag *node_flag,
		*parent_node_flag,
		*parent2_node_flag;
	int ret;

	if (caa_unlikely(key > ja->key_max))
		return -EINVAL;
	tree_depth = ja->tree_depth;

retry:
	dbg_printf("cds_ja_add attempt: key %" PRIu64 ", node %p\n",
		key, new_node);
	parent2_node_flag = NULL;
	parent_node_flag =
		(struct cds_ja_inode_flag *) &ja->root;	/* Use root ptr address as key for mutex */
	node_flag_ptr = &ja->root;
	node_flag = rcu_dereference(*node_flag_ptr);

	/* Iterate on all internal levels */
	for (i = 1; i < tree_depth; i++) {
		uint8_t iter_key;

		if (!ja_node_ptr(node_flag)) {
			ret = ja_attach_node(ja, node_flag_ptr,
					parent_node_flag, parent2_node_flag,
					key, i, new_node);
			if (ret == -ENOENT || ret == -EEXIST)
				goto retry;
			else
				goto end;
		}
		iter_key = (uint8_t) (key >> (JA_BITS_PER_BYTE * (tree_depth - i - 1)));
		parent2_node_flag = parent_node_flag;
		parent_node_flag = node_flag;
		node_flag = ja_node_get_nth(node_flag,
			&node_flag_ptr,
			iter_key);
	}

	/*
	 * We reached bottom of tree, simply add node to last internal
	 * level, or chain it if key is already present.
	 */
	if (!ja_node_ptr(node_flag)) {
		ret = ja_attach_node(ja, node_flag_ptr, parent_node_flag,
				parent2_node_flag, key, i, new_node);
	} else {
		ret = ja_chain_node(ja,
			(struct cds_hlist_head *) ja_node_ptr(node_flag),
			new_node);
	}
	if (ret == -ENOENT)
		goto retry;
end:
	return ret;
}

struct cds_ja *_cds_ja_new(unsigned int key_bits,
		const struct rcu_flavor_struct *flavor)
{
	struct cds_ja *ja;
	int ret;
	struct cds_ja_shadow_node *root_shadow_node;

	ja = calloc(sizeof(*ja), 1);
	if (!ja)
		goto ja_error;

	switch (key_bits) {
	case 8:
		ja->key_max = UINT8_MAX;
		break;
	case 16:
		ja->key_max = UINT16_MAX;
		break;
	case 32:
		ja->key_max = UINT32_MAX;
		break;
	case 64:
		ja->key_max = UINT64_MAX;
		break;
	default:
		goto check_error;
	}

	/* ja->root is NULL */
	/* tree_depth 0 is for pointer to root node */
	ja->tree_depth = (key_bits >> JA_BITS_PER_BYTE) + 1;
	assert(ja->tree_depth <= JA_MAX_DEPTH);
	ja->ht = rcuja_create_ht(flavor);
	if (!ja->ht)
		goto ht_error;

	/*
	 * Note: we should not free this node until judy array destroy.
	 */
	root_shadow_node = rcuja_shadow_set(ja->ht,
			ja_node_ptr((struct cds_ja_inode_flag *) &ja->root),
			NULL);
	if (!root_shadow_node) {
		ret = -ENOMEM;
		goto ht_node_error;
	}

	return ja;

ht_node_error:
	ret = rcuja_delete_ht(ja->ht);
	assert(!ret);
ht_error:
check_error:
	free(ja);
ja_error:
	return NULL;
}

/*
 * There should be no more concurrent add to the judy array while it is
 * being destroyed (ensured by the caller).
 */
int cds_ja_destroy(struct cds_ja *ja)
{
	int ret;

	rcuja_shadow_prune(ja->ht,
		RCUJA_SHADOW_CLEAR_FREE_NODE | RCUJA_SHADOW_CLEAR_FREE_LOCK);
	ret = rcuja_delete_ht(ja->ht);
	if (ret)
		return ret;
	if (uatomic_read(&ja->nr_fallback))
		fprintf(stderr,
			"[warning] RCU Judy Array used %lu fallback node(s)\n",
			uatomic_read(&ja->nr_fallback));
	free(ja);
	return 0;
}
