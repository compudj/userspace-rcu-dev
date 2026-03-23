// SPDX-FileCopyrightText: 2025-2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-only

/*
 * RCU Fractal Trie allocator for internal nodes.
 *
 * This allocator maximizes cache locality for read-side fast paths
 * (lookups and traversals) by placing item data and metadata on
 * different cache lines.
 *
 * This is achieved using a strided allocation approach with the
 * following layout:
 *
 * Layout of allocation arena ranges for read-side items of size
 * item_len (power of 2):
 *
 *   nr_items = page_size / item_len.
 *
 *   Offset                 Content
 *
 *   0:                     array of nr_items elements of item_len each
 *   page_size:             struct cds_ft_alloc_range
 *   page_size + sizeof(struct cds_ft_alloc_range):
 *                          array of nr_items struct cds_ft_metadata_alloc
 *
 * An allocation arena contains a linked list of allocation ranges.
 */

#include <errno.h>
#include <string.h>
#include <urcu/fractal-trie.h>
#include <urcu/list.h>
#include "fractal-trie-internal.h"
#include "urcu-utils.h"

struct cds_ft_metadata_alloc;
struct cds_ft_metadata;
struct cds_ft_alloc_range;
struct cds_ft_alloc_arena;

static size_t page_size;

struct cds_ft_alloc_range {
	struct cds_list_head node;		/* Linked list of ranges. */
	struct cds_ft_alloc_arena *arena;	/* Backward reference to arena. */
	size_t next_unused;

	struct cds_ft_metadata_alloc metadata[];
};

struct cds_ft_alloc_arena {
	struct cds_ft *ft;
	struct cds_list_head ranges;			/* List head of struct cds_ft_alloc_range. */
	size_t item_len_order;
	size_t max_nr_items_per_range;
	struct cds_ft_metadata_alloc *free_list_head;	/* NULL terminated singly-linked list. */
	pthread_mutex_t lock;
	char *name;
};

static
struct cds_ft_metadata *cds_ft_range_get_nth_metadata(struct cds_ft_alloc_range *range, size_t n)
{
	return &range->metadata[n].metadata;
}

static
void *cds_ft_range_get_nth_item(struct cds_ft_alloc_range *range, size_t n)
{
	return (((char *) range) - page_size) + (n << range->arena->item_len_order);
}

static
struct cds_ft_alloc_range *cds_ft_item_to_range(void *p)
{
	void *base = (void *)((unsigned long)p & ~(page_size - 1));

	return (struct cds_ft_alloc_range *) (base + page_size);
}

static
struct cds_ft_alloc_range *cds_ft_metadata_to_range(struct cds_ft_metadata *metadata)
{
	struct cds_ft_metadata_alloc *metadata_alloc =
		caa_container_of(metadata, struct cds_ft_metadata_alloc, metadata);
	size_t index = metadata_alloc->alloc_index;

	return (struct cds_ft_alloc_range *)((char *)(metadata_alloc - index) - sizeof(struct cds_ft_alloc_range));
}

struct cds_ft_metadata *cds_ft_item_to_metadata(void *p)
{
	struct cds_ft_alloc_range *range = cds_ft_item_to_range(p);
	size_t page_offset = (unsigned long) p & (page_size - 1);
	size_t index = page_offset >> range->arena->item_len_order;

	return cds_ft_range_get_nth_metadata(range, index);
}

void *cds_ft_metadata_to_item(struct cds_ft_metadata *metadata)
{
	struct cds_ft_metadata_alloc *metadata_alloc =
		caa_container_of(metadata, struct cds_ft_metadata_alloc, metadata);
	size_t index = metadata_alloc->alloc_index;
	struct cds_ft_alloc_range *range = cds_ft_metadata_to_range(metadata);

	return cds_ft_range_get_nth_item(range, index);
}

static
size_t cds_ft_arena_range_alloc_size(size_t item_len_order)
{
	return page_size + sizeof(struct cds_ft_alloc_range) +
		(page_size >> item_len_order) * sizeof(struct cds_ft_metadata_alloc);
}

static
struct cds_ft_alloc_range *range_create(struct cds_ft_alloc_arena *arena)
{
	size_t alloc_size = cds_ft_arena_range_alloc_size(arena->item_len_order);
	void *ptr = aligned_alloc(page_size, alloc_size);
	struct cds_ft_alloc_range *range;

	memset(ptr, 0, alloc_size);
	range = (struct cds_ft_alloc_range *) (ptr + page_size);
	range->arena = arena;
	return range;
}

static
void range_destroy(struct cds_ft_alloc_range *range)
{
	void *p = (void *) range - page_size;

	cds_list_del(&range->node);
	free(p);
}

static
struct cds_ft_alloc_arena *cds_ft_arena_create(struct cds_ft *ft, const char *arena_name, size_t item_len_order)
{
	struct cds_ft_alloc_arena *arena;

	if (!page_size)
		page_size = urcu_get_page_len();

	/* item_len must be no larger than page_size. */
	if ((1UL << item_len_order) > page_size) {
		errno = EINVAL;
		return NULL;
	}
	arena = calloc(1, sizeof(struct cds_ft_alloc_arena));
	if (!arena)
		goto error_alloc;
	arena->ft = ft;
	arena->item_len_order = item_len_order;
	arena->max_nr_items_per_range = page_size >> item_len_order;
	CDS_INIT_LIST_HEAD(&arena->ranges);
	if (arena_name) {
		arena->name = strdup(arena_name);
		if (!arena_name)
			goto error_alloc;
	}
	pthread_mutex_init(&arena->lock, NULL);
	return arena;

error_alloc:
	free(arena);
	errno = ENOMEM;
	return NULL;
}

static
void cds_ft_arena_destroy(struct cds_ft_alloc_arena *arena)
{
	struct cds_ft_alloc_range *range, *tmp;

	if (!arena)
		return;
	pthread_mutex_destroy(&arena->lock);
	cds_list_for_each_entry_safe(range, tmp, &arena->ranges, node)
		range_destroy(range);
	free(arena->name);
	free(arena);
}

static
struct cds_ft_metadata *cds_ft_arena_alloc(struct cds_ft_alloc_arena *arena)
{
	struct cds_ft_metadata_alloc *free_list_head, *item;
	struct cds_ft_alloc_range *range;
	size_t item_index;
	void *p;

	pthread_mutex_lock(&arena->lock);
	free_list_head = arena->free_list_head;

	/* Return head of free list. */
	if (free_list_head) {
		arena->free_list_head = free_list_head->free_list_next;
		p = cds_ft_metadata_to_item(&free_list_head->metadata);
		memset(p, 0, 1U << arena->item_len_order);
		memset(&free_list_head->metadata, 0, sizeof(free_list_head->metadata));
		pthread_mutex_unlock(&arena->lock);
		return &free_list_head->metadata;
	}
	/*
	 * If there are no ranges, or if the most recent range (first in
	 * list) does not have any room left, create a new range and
	 * prepend it to the list head.
	 */
	if (cds_list_empty(&arena->ranges))
		goto create_range;
	range = cds_list_first_entry(&arena->ranges, struct cds_ft_alloc_range, node);
	if (range->next_unused + 1 > arena->max_nr_items_per_range)
		goto create_range;
	else
		goto room_left;
create_range:
	range = range_create(arena);
	if (!range) {
		errno = ENOMEM;
		pthread_mutex_unlock(&arena->lock);
		return NULL;
	}
	/* Add range to head of list. */
	cds_list_add(&range->node, &arena->ranges);
room_left:
	/* First range in list has room left. */
	item_index = range->next_unused++;
	item = &range->metadata[item_index];
	item->alloc_index = item_index;
	pthread_mutex_unlock(&arena->lock);
	return &item->metadata;
}

struct cds_ft_metadata *cds_ft_alloc_item(struct cds_ft *ft, size_t item_len_order)
{
	struct cds_ft_alloc_arena **arena_p;

	if (!page_size)
		page_size = urcu_get_page_len();
	if ((1UL << item_len_order) > page_size) {
		errno = EINVAL;
		return NULL;
	}
	arena_p = &ft->arena_order[item_len_order];
	if (!*arena_p) {
		*arena_p = cds_ft_arena_create(ft, "cds_ft_alloc", item_len_order);
		if (!*arena_p)
			return NULL;
	}
	return cds_ft_arena_alloc(*arena_p);
}

static
void cds_ft_free_item_rcu(struct rcu_head *rcu_head)
{
	struct cds_ft_metadata_alloc *metadata_alloc =
		caa_container_of(rcu_head, struct cds_ft_metadata_alloc, rcu_head);
	struct cds_ft_alloc_arena *arena =
		cds_ft_metadata_to_range(&metadata_alloc->metadata)->arena;

	pthread_mutex_lock(&arena->lock);
	metadata_alloc->free_list_next = arena->free_list_head;
	arena->free_list_head = metadata_alloc;
	pthread_mutex_unlock(&arena->lock);
}

void cds_ft_free_item(struct cds_ft_metadata *metadata)
{
	struct cds_ft_metadata_alloc *metadata_alloc =
		caa_container_of(metadata, struct cds_ft_metadata_alloc, metadata);
	struct cds_ft_alloc_range *range = cds_ft_metadata_to_range(&metadata_alloc->metadata);
	struct cds_ft_alloc_arena *arena = range->arena;
	const struct rcu_flavor_struct *flavor = arena->ft->flavor;

	flavor->update_call_rcu(&metadata_alloc->rcu_head, cds_ft_free_item_rcu);
}

void cds_ft_free_all_arenas(struct cds_ft *ft)
{
	int i;

	for (i = 0; i <= FT_ALLOC_ORDER_MAX; i++) {
		if (!ft->arena_order[i])
			continue;
		cds_ft_arena_destroy(ft->arena_order[i]);
		ft->arena_order[i] = NULL;
	}
}
