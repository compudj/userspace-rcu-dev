// SPDX-FileCopyrightText: 2000-2002 Hewlett-Packard Company
// SPDX-FileCopyrightText: 2012-2025 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-only

#ifndef _URCU_RCUJA_INTERNAL_H
#define _URCU_RCUJA_INTERNAL_H

/*
 * rcuja/rcuja-internal.h
 *
 * Userspace RCU library - RCU Judy Array Internal Header
 */

#include <pthread.h>
#include <stdio.h>
#include <inttypes.h>
#include <unistd.h>
#include <urcu/rculfhash.h>
#include <assert.h>


/*
 * If the internal bit is set in a pointer, it points to an internal
 * Judy array node, else it points to a node outside of the Judy array.
 * This can be used for variable length keys to identify the end of key.
 */
#define JA_INTERNAL_BITS	1
#define JA_INTERNAL_MASK	(1U << 0)

/*
 * This if followed by a number of bits reserved to represent the child
 * type.
 */
#define JA_TYPE_BITS	3
#define JA_TYPE_MAX_NR	(1UL << JA_TYPE_BITS)
#define JA_TYPE_MASK	((JA_TYPE_MAX_NR - 1) << JA_INTERNAL_BITS)
#define JA_PTR_MASK	(~(JA_TYPE_MASK | JA_INTERNAL_MASK))

#define JA_ENTRY_PER_NODE	256
#define JA_LOG2_BITS_PER_BYTE	3U
#define JA_BITS_PER_BYTE	(1U << JA_LOG2_BITS_PER_BYTE)

#define JA_POOL_1D_MASK	((JA_BITS_PER_BYTE - 1) << (JA_TYPE_BITS + JA_INTERNAL_BITS))
/* 2D mask has C(n=8,r=2) = 28 possibilities (fits in 5 bits). */
#define JA_POOL_2D_MASK	(((1U << 5) - 1) << (JA_TYPE_BITS + JA_INTERNAL_BITS))

#define JA_MAX_DEPTH	9	/* Maximum depth, including leafs */

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

#define RCU_JA_ALLOC_ORDER_MAX		12

/* Never declared. Opaque type used to store flagged node pointers. */
struct cds_ja_inode_flag;
struct cds_ja_inode;

struct cds_ja_alloc_arena;
struct cds_ja_metadata_alloc;

struct cds_ja_metadata {
	unsigned int nr_child;			/* Number of children in node. */
	int fallback_removal_count;		/* Removals left keeping fallback. */
	int level;				/* Level in the tree. */
};

struct cds_ja_metadata_alloc {
	union {
		struct rcu_head rcu_head;			/* For deferred node reclaim. */
		struct cds_ja_metadata_alloc *free_list_next;	/* Free list next pointer. */
	};
	unsigned int alloc_index;
	struct cds_ja_metadata metadata;
};

struct cds_ja {
	struct cds_ja_inode_flag *root;
	struct cds_ja_metadata root_metadata;

	unsigned int tree_depth;
	unsigned int key_len;
	unsigned long nr_fallback;	/* Number of fallback nodes used */

	const struct rcu_flavor_struct *flavor;

	/* Allocation arenas. */
	struct cds_ja_alloc_arena *arena_order[RCU_JA_ALLOC_ORDER_MAX + 1];

	/* For debugging */
	unsigned long node_fallback_count_distribution[JA_ENTRY_PER_NODE];
	unsigned long nr_nodes_allocated, nr_nodes_freed;
};

static inline
struct cds_ja_inode_flag *ja_node_flag(struct cds_ja_inode *node,
		unsigned long type)
{
	assert(type < (1UL << JA_TYPE_BITS));
	return (struct cds_ja_inode_flag *) (((unsigned long) node) |
		(type << JA_INTERNAL_BITS) |
		JA_INTERNAL_MASK);
}

static inline
struct cds_ja_inode_flag *ja_node_flag_pool_1d(struct cds_ja_inode *node,
		unsigned long type, unsigned long bitsel)
{
	assert(type < (1UL << JA_TYPE_BITS));
	assert(bitsel < JA_BITS_PER_BYTE);
	return (struct cds_ja_inode_flag *) (((unsigned long) node) |
		(bitsel << (JA_TYPE_BITS + JA_INTERNAL_BITS)) |
		(type << JA_INTERNAL_BITS) |
		JA_INTERNAL_MASK);
}

static inline
struct cds_ja_inode_flag *ja_node_flag_pool_2d(struct cds_ja_inode *node,
		unsigned long type, unsigned int subclass_index)
{
	assert(type < (1UL << JA_TYPE_BITS));
	return (struct cds_ja_inode_flag *) (((unsigned long) node) |
		(subclass_index << (JA_TYPE_BITS + JA_INTERNAL_BITS)) |
		(type << JA_INTERNAL_BITS) |
		JA_INTERNAL_MASK);
}

/* Hardcoded pool indexes for fast path */
#define RCU_JA_POOL_IDX_5	5
#define RCU_JA_POOL_IDX_6	6
static inline
struct cds_ja_inode *ja_node_ptr(struct cds_ja_inode_flag *node)
{
	unsigned long v, type_idx;

	if (!node)
		return NULL;	/* RCU_JA_NULL */
	v = (unsigned long) node;
	type_idx = (v & JA_TYPE_MASK) >> JA_INTERNAL_MASK;

	switch (type_idx) {
	case RCU_JA_POOL_IDX_5:
		v &= ~(JA_POOL_1D_MASK | JA_TYPE_MASK | JA_INTERNAL_MASK);
		break;
	case RCU_JA_POOL_IDX_6:
		v &= ~(JA_POOL_2D_MASK | JA_TYPE_MASK | JA_INTERNAL_MASK);
		break;
	default:
		/* RCU_JA_LINEAR or RCU_JA_PIGEON */
		v &= JA_PTR_MASK;
		break;
	}
	return (struct cds_ja_inode *) v;
}

__attribute__((visibility("hidden")))
unsigned long ja_node_type(struct cds_ja_inode_flag *node);

__attribute__((visibility("hidden")))
void cds_ja_free_all_arenas(struct cds_ja *ja);

__attribute__((visibility("hidden")))
struct cds_ja_metadata *cds_ja_item_to_metadata(void *p);

__attribute__((visibility("hidden")))
void *cds_ja_metadata_to_item(struct cds_ja_metadata *metadata);

__attribute__((visibility("hidden")))
struct cds_ja_metadata *cds_ja_alloc_item(struct cds_ja *ja, size_t item_len_order);

__attribute__((visibility("hidden")))
void cds_ja_free_item(struct cds_ja_metadata *metadata);

/*
 * Iterate through duplicates returned by cds_ja_lookup*()
 * Receives a struct cds_ja_node * as parameter, which is used as start
 * of duplicate list and loop cursor.
 */
#define cds_ja_for_each_duplicate(pos)				\
       for (; (pos) != NULL; (pos) = (pos)->next)

//#define DEBUG
//#define DEBUG_COUNTERS

#ifdef __linux__
#include <syscall.h>
#endif

#ifdef DEBUG
#define dbg_printf(fmt, args...)				\
	fprintf(stderr, "[debug rcuja %s()@%s:%u] " fmt,	\
		__func__, __FILE__, __LINE__, ## args)

#else
#define dbg_printf(fmt, args...)				\
do {								\
	/* do nothing but check printf format */		\
	if (0)							\
		fprintf(stderr, "[debug rcuja %s()@%s:%u] " fmt, \
			__func__, __FILE__, __LINE__, ## args);	\
} while (0)
#endif

#ifdef DEBUG_COUNTERS
static inline
int ja_debug_counters(void)
{
	return 1;
}
#else
static inline
int ja_debug_counters(void)
{
	return 0;
}
#endif

#endif /* _URCU_RCUJA_INTERNAL_H */
