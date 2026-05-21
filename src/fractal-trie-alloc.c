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
 *   nr_items = cds_ft_page_size / item_len.
 *
 *   Offset                 Content
 *
 *   0:                     array of nr_items elements of item_len each
 *   cds_ft_page_size:             struct cds_ft_alloc_range
 *   cds_ft_page_size + sizeof(struct cds_ft_alloc_range):
 *                          array of nr_items struct cds_ft_metadata_alloc
 *   2 * cds_ft_page_size - nr_items * sizeof(struct cds_ft_bitmap):
 *                          reverse array of nr_items struct cds_ft_bitmap (only for bitmap-bearing arenas)
 *
 * An allocation arena contains a linked list of allocation ranges.
 */

#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <sys/mman.h>
#ifdef __linux__
#include <sys/syscall.h>
#include <unistd.h>
#endif
#include <urcu/fractal-trie.h>
#include <urcu/list.h>
#include <urcu/uatomic.h>
#include "fractal-trie-internal.h"
#include "urcu-utils.h"

/*
 * Superblock size for the bump-allocator backing range_create.  Each
 * arena owns a list of superblocks; ranges are carved out of the head
 * superblock with a bump pointer, and a new superblock is mmap'd when
 * the head fills up.  Sized large enough that VMA fragmentation is
 * negligible and small enough to avoid wasting address space on tiny
 * tries.
 */
#ifndef FT_SUPERBLOCK_SIZE
#define FT_SUPERBLOCK_SIZE (64UL * 1024 * 1024)
#endif

struct cds_ft_alloc_arena;

struct cds_ft_alloc_superblock {
	void *base;
	size_t size;
	size_t used;
	struct cds_list_head node;
};

__attribute__((visibility("hidden")))
size_t cds_ft_page_size;

/*
 * struct cds_ft_metadata_alloc and struct cds_ft_alloc_range are defined
 * in fractal-trie-internal.h so that the hot-path helpers
 * (cds_ft_item_to_range, cds_ft_item_to_metadata_fast,
 * cds_ft_item_to_bitmap) can inline into fractal-trie.c.  The arena
 * struct stays opaque to readers and is defined here.
 */
struct cds_ft_alloc_arena {
	struct cds_ft_group *ft_group;
	struct cds_list_head ranges;			/* List head of struct cds_ft_alloc_range. */
	struct cds_list_head superblocks;		/* List head of struct cds_ft_alloc_superblock. */
	size_t item_len_order;
	size_t max_nr_items_per_range;
	struct cds_ft_metadata_alloc *free_list_head;	/* NULL terminated singly-linked list. */
	pthread_mutex_t lock;
	char *name;
	bool bitmap;
};

static
void *cds_ft_range_get_nth_item(struct cds_ft_alloc_range *range, size_t n)
{
	return (((char *) range) - cds_ft_page_size) + (n << range->arena->item_len_order);
}

static
struct cds_ft_alloc_range *cds_ft_metadata_to_range(struct cds_ft_metadata *metadata)
{
	struct cds_ft_metadata_alloc *metadata_alloc =
		caa_container_of(metadata, struct cds_ft_metadata_alloc, metadata);
	size_t index = metadata->alloc_index;
	struct cds_ft_alloc_range *range;

	range = (struct cds_ft_alloc_range *)((char *)(metadata_alloc - index) - sizeof(struct cds_ft_alloc_range));
	assert(index < range->arena->max_nr_items_per_range);
	return range;
}

struct cds_ft_metadata *cds_ft_item_to_metadata(void *p)
{
	struct cds_ft_alloc_range *range = cds_ft_item_to_range(p);
	return cds_ft_item_to_metadata_fast(p, range->arena->item_len_order);
}

size_t cds_ft_item_order(void *p)
{
	struct cds_ft_alloc_range *range = cds_ft_item_to_range(p);
	return range->arena->item_len_order;
}

void *cds_ft_metadata_to_item(struct cds_ft_metadata *metadata)
{
	size_t index = metadata->alloc_index;
	struct cds_ft_alloc_range *range = cds_ft_metadata_to_range(metadata);

	return cds_ft_range_get_nth_item(range, index);
}

static
size_t cds_ft_arena_range_alloc_size(size_t item_len_order, bool bitmap)
{
	if (bitmap)
		return 2 * cds_ft_page_size;
	else
		return cds_ft_page_size + sizeof(struct cds_ft_alloc_range) +
			(cds_ft_page_size >> item_len_order) * sizeof(struct cds_ft_metadata_alloc);
}

/*
 * mbind() with MPOL_INTERLEAVE round-robins page placement across
 * the calling thread's allowed NUMA nodes, applied at superblock
 * creation while no page is faulted yet.
 *
 * Enabled by default: the fractal trie targets workloads with many
 * concurrent readers traversing shared data, where spreading the
 * arena across NUMA nodes wins by a wide margin over concentrating
 * it on a single node via first-touch.  Set CDS_FT_NUMA_INTERLEAVE=0
 * to opt out and fall back to first-touch placement (useful for
 * workloads with thread-local working sets or for benchmarking
 * against the no-policy baseline).
 */
#ifdef __linux__
#define FT_MPOL_INTERLEAVE	3
#define FT_MPOL_F_MEMS_ALLOWED	(1U << 2)
#define FT_MAX_NUMA_NODES	1024
#define FT_NODEMASK_LONGS	(FT_MAX_NUMA_NODES / (sizeof(unsigned long) * 8))

static
int ft_interleave_enabled(void)
{
	static int cached = -1;
	const char *env;
	int v;

	v = uatomic_load(&cached, CMM_RELAXED);
	if (v != -1)
		return v;
	env = getenv("CDS_FT_NUMA_INTERLEAVE");
	v = (env && env[0] == '0') ? 0 : 1;
	uatomic_store(&cached, v, CMM_RELAXED);
	return v;
}

static
void ft_apply_interleave(void *base, size_t size)
{
	unsigned long nodemask[FT_NODEMASK_LONGS] = { 0 };
	unsigned long any = 0;
	size_t i;
	long r;

	if (!ft_interleave_enabled())
		return;
	r = syscall(__NR_get_mempolicy, NULL, nodemask, (unsigned long) FT_MAX_NUMA_NODES,
			NULL, FT_MPOL_F_MEMS_ALLOWED);
	if (r < 0)
		return;
	for (i = 0; i < FT_NODEMASK_LONGS; i++)
		any |= nodemask[i];
	if (!any)
		return;
	(void) syscall(__NR_mbind, base, size, FT_MPOL_INTERLEAVE,
			nodemask, (unsigned long) FT_MAX_NUMA_NODES, 0);
}
#else
static inline void ft_apply_interleave(void *base __attribute__((unused)),
		size_t size __attribute__((unused))) {}
#endif

/*
 * Allocate a fresh superblock big enough to host at least one
 * range of size min_size.  Pages are mmap'd anonymous, so they are
 * lazily zero-initialized on first touch.  When CDS_FT_NUMA_INTERLEAVE
 * is set, mbind(MPOL_INTERLEAVE) is applied before any page is
 * faulted, so the kernel round-robins placement across the calling
 * thread's allowed NUMA nodes.
 */
static
struct cds_ft_alloc_superblock *superblock_create(size_t min_size)
{
	struct cds_ft_alloc_superblock *sb;
	size_t size;
	void *base;

	size = FT_SUPERBLOCK_SIZE > min_size ? FT_SUPERBLOCK_SIZE : min_size;
	/* Round up to page boundary. */
	size = (size + cds_ft_page_size - 1) & ~(cds_ft_page_size - 1);
	base = mmap(NULL, size, PROT_READ | PROT_WRITE,
			MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
	if (base == MAP_FAILED)
		return NULL;
	ft_apply_interleave(base, size);
	sb = malloc(sizeof(*sb));
	if (!sb) {
		munmap(base, size);
		return NULL;
	}
	sb->base = base;
	sb->size = size;
	sb->used = 0;
	return sb;
}

static
void superblock_destroy(struct cds_ft_alloc_superblock *sb)
{
	cds_list_del(&sb->node);
	munmap(sb->base, sb->size);
	free(sb);
}

/*
 * Carve a range out of the arena's current head superblock.  If the
 * head has insufficient room (or there are no superblocks yet),
 * allocate a new one and prepend it.
 *
 * Caller must hold arena->lock.
 */
static
struct cds_ft_alloc_range *range_create(struct cds_ft_alloc_arena *arena)
{
	size_t alloc_size = cds_ft_arena_range_alloc_size(arena->item_len_order, arena->bitmap);
	size_t alloc_size_aligned = (alloc_size + cds_ft_page_size - 1) & ~(cds_ft_page_size - 1);
	struct cds_ft_alloc_superblock *sb;
	struct cds_ft_alloc_range *range;
	void *ptr;

	if (cds_list_empty(&arena->superblocks))
		goto create_sb;
	sb = cds_list_first_entry(&arena->superblocks,
			struct cds_ft_alloc_superblock, node);
	if (sb->used + alloc_size_aligned > sb->size)
		goto create_sb;
	goto carve;
create_sb:
	sb = superblock_create(alloc_size_aligned);
	if (!sb)
		return NULL;
	cds_list_add(&sb->node, &arena->superblocks);
carve:
	ptr = (char *) sb->base + sb->used;
	sb->used += alloc_size_aligned;
	/* mmap'd anonymous pages are zero-initialized; no memset needed. */
	range = (struct cds_ft_alloc_range *) ((char *) ptr + cds_ft_page_size);
	range->arena = arena;
	return range;
}

/*
 * Remove a range from the arena's range list.  Memory is owned by
 * the superblock; it is reclaimed when the superblock is destroyed.
 */
static
void range_destroy(struct cds_ft_alloc_range *range)
{
	cds_list_del(&range->node);
}

static
struct cds_ft_alloc_arena *cds_ft_arena_create(struct cds_ft_group *ft_group,
		const char *arena_name, size_t item_len_order, bool bitmap)
{
	struct cds_ft_alloc_arena *arena;
	size_t max_items_per_range;

	if (!cds_ft_page_size)
		cds_ft_page_size = urcu_get_page_len();

	/* Reject page sizes larger than the compile-time maximum. */
	if (cds_ft_page_size > (1UL << FT_MAX_PAGE_ORDER)) {
		errno = EINVAL;
		return NULL;
	}
#ifdef FT_PAGE_SIZE_FIXED
	/*
	 * Architectures that hardcode page_size in the inline helpers
	 * (cds_ft_get_page_size) must match the kernel's reported page
	 * size at runtime.  Reject otherwise — a mismatch would corrupt
	 * item-to-metadata address derivation on the read-side fast path.
	 */
	if (cds_ft_page_size != FT_PAGE_SIZE_FIXED) {
		errno = EINVAL;
		return NULL;
	}
#endif
	/* item_len must be no larger than cds_ft_page_size. */
	if ((1UL << item_len_order) > cds_ft_page_size) {
		errno = EINVAL;
		return NULL;
	}
	max_items_per_range = cds_ft_page_size >> item_len_order;
	/* Ensure that range header, metadata array and bitmaps fit in a page. */
	if (bitmap && (sizeof(struct cds_ft_alloc_range) +
			max_items_per_range * (sizeof(struct cds_ft_metadata_alloc) +
				sizeof(struct cds_ft_bitmap)) > cds_ft_page_size)) {
		errno = EINVAL;
		return NULL;
	}
	arena = calloc(1, sizeof(struct cds_ft_alloc_arena));
	if (!arena)
		goto error_alloc;
	arena->ft_group = ft_group;
	arena->item_len_order = item_len_order;
	arena->max_nr_items_per_range = max_items_per_range;
	arena->bitmap = bitmap;
	CDS_INIT_LIST_HEAD(&arena->ranges);
	CDS_INIT_LIST_HEAD(&arena->superblocks);
	if (arena_name) {
		arena->name = strdup(arena_name);
		if (!arena->name)
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
	struct cds_ft_alloc_range *range, *range_tmp;
	struct cds_ft_alloc_superblock *sb, *sb_tmp;

	if (!arena)
		return;
	pthread_mutex_destroy(&arena->lock);
	cds_list_for_each_entry_safe(range, range_tmp, &arena->ranges, node)
		range_destroy(range);
	cds_list_for_each_entry_safe(sb, sb_tmp, &arena->superblocks, node)
		superblock_destroy(sb);
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
		uint16_t saved_alloc_index = free_list_head->metadata.alloc_index;

		arena->free_list_head = free_list_head->free_list_next;
		p = cds_ft_metadata_to_item(&free_list_head->metadata);
		memset(p, 0, 1U << arena->item_len_order);
		memset(&free_list_head->metadata, 0, sizeof(free_list_head->metadata));
		free_list_head->metadata.alloc_index = saved_alloc_index;
		if (arena->bitmap) {
			struct cds_ft_bitmap *bitmap = cds_ft_item_to_bitmap(p, arena->item_len_order);
			memset(bitmap, 0, sizeof(struct cds_ft_bitmap));
		}
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
	item->metadata.alloc_index = item_index;
	pthread_mutex_unlock(&arena->lock);
	return &item->metadata;
}

struct cds_ft_metadata *cds_ft_alloc_item(struct cds_ft *ft, size_t item_len_order, bool bitmap)
{
	struct cds_ft_alloc_arena **arena_p;
	struct cds_ft_alloc_arena *arena;

	if (!cds_ft_page_size)
		cds_ft_page_size = urcu_get_page_len();
	if ((1UL << item_len_order) > cds_ft_page_size) {
		errno = EINVAL;
		return NULL;
	}
	arena_p = &ft->group->arena_order[item_len_order];
	arena = uatomic_load(arena_p, CMM_ACQUIRE);
	if (caa_unlikely(!arena)) {
		pthread_mutex_lock(&ft->group->arena_lock);
		arena = *arena_p;
		if (!arena) {
			arena = cds_ft_arena_create(ft->group, "cds_ft_alloc", item_len_order, bitmap);
			if (!arena) {
				pthread_mutex_unlock(&ft->group->arena_lock);
				return NULL;
			}
			uatomic_store(arena_p, arena, CMM_RELEASE);
		}
		pthread_mutex_unlock(&ft->group->arena_lock);
	}
	return cds_ft_arena_alloc(arena);
}

/*
 * Synchronous free body shared by the call_rcu callback and the
 * exclusive-mode fast path.  Frees the extended density counters,
 * then either poisons the slot (FT_IMMEDIATE_FREE testing mode) or
 * returns the slot to the arena free list.
 */
static
void cds_ft_do_free_item(struct cds_ft_metadata *metadata)
{
	struct cds_ft_metadata_alloc *metadata_alloc =
		caa_container_of(metadata, struct cds_ft_metadata_alloc, metadata);

#ifdef FT_IMMEDIATE_FREE
	/*
	 * Immediate-free testing mode: poison metadata and node data
	 * so any subsequent access crashes deterministically and the
	 * slot is never reused.  See cds_ft_free_item() docstring for
	 * the safety constraints.
	 */
	{
		struct cds_ft_alloc_range *range =
			cds_ft_metadata_to_range(metadata);
		struct cds_ft_alloc_arena *arena = range->arena;
		size_t item_len = 1UL << arena->item_len_order;
		void *item = cds_ft_metadata_to_item(metadata);

		memset(item, 0xfe, item_len);
		memset(metadata_alloc, 0xfe, sizeof(*metadata_alloc));
	}
#else
	{
		struct cds_ft_alloc_arena *arena =
			cds_ft_metadata_to_range(metadata)->arena;

		pthread_mutex_lock(&arena->lock);
		metadata_alloc->free_list_next = arena->free_list_head;
		arena->free_list_head = metadata_alloc;
		pthread_mutex_unlock(&arena->lock);
	}
#endif
}

static
void cds_ft_free_item_rcu(struct rcu_head *rcu_head)
{
	struct cds_ft_metadata_alloc *metadata_alloc =
		caa_container_of(rcu_head, struct cds_ft_metadata_alloc, rcu_head);
	cds_ft_do_free_item(&metadata_alloc->metadata);
}

/*
 * Release a metadata slot back to its arena.
 *
 * Concurrent-mode tries defer the actual freelist push by call_rcu
 * so concurrent RCU readers cannot dereference a slot the writer
 * has just unlinked.  Exclusive-mode tries forbid concurrent
 * readers (cds_ft_make_exclusive() drains pre-existing readers via
 * synchronize_rcu before flipping the flag), so the slot can be
 * pushed back synchronously and reused immediately by the next
 * allocation, avoiding the call_rcu round-trip.
 *
 * FT_IMMEDIATE_FREE testing mode poisons every slot in either
 * mode (see cds_ft_do_free_item).
 */
void cds_ft_free_item(struct cds_ft *ft, struct cds_ft_metadata *metadata)
{
#ifdef FT_IMMEDIATE_FREE
	/*
	 * Immediate free for use-after-free detection by mutation
	 * code.  Poisons metadata and node data so any subsequent
	 * access crashes deterministically.  Do NOT return to the
	 * free list.
	 *
	 * IMPORTANT: this mode is only safe for single-threaded
	 * mutation testing WITHOUT concurrent RCU readers.
	 *
	 * The item data poison would corrupt concurrent exact
	 * lookups (which only traverse item data and never touch
	 * metadata).  The metadata poison would additionally
	 * corrupt concurrent inequality lookups, iteration, and
	 * skip-compressed traversal, which read metadata fields
	 * (parent, external_nodes, nr_keys) on the read-side.
	 *
	 * Use this mode exclusively for validating that mutation
	 * paths do not access freed memory.
	 */
	(void) ft;
	cds_ft_do_free_item(metadata);
#else
	if (ft->exclusive) {
		cds_ft_do_free_item(metadata);
	} else {
		struct cds_ft_metadata_alloc *metadata_alloc =
			caa_container_of(metadata, struct cds_ft_metadata_alloc, metadata);
		struct cds_ft_alloc_range *range =
			cds_ft_metadata_to_range(metadata);
		struct cds_ft_alloc_arena *arena = range->arena;
		const struct rcu_flavor_struct *flavor = arena->ft_group->flavor;

		flavor->update_call_rcu(&metadata_alloc->rcu_head, cds_ft_free_item_rcu);
	}
#endif
}

/*
 * Immediate-free path for items that were never published — no reader
 * can hold a reference, so call_rcu would only delay arena reuse.
 * Always routes through the synchronous body, regardless of exclusive
 * mode or FT_IMMEDIATE_FREE configuration.  See declaration in
 * fractal-trie-internal.h for the safety contract.
 */
void cds_ft_free_item_unpublished(struct cds_ft *ft __attribute__((unused)),
		struct cds_ft_metadata *metadata)
{
	cds_ft_do_free_item(metadata);
}

void cds_ft_free_all_arenas(struct cds_ft_group *ft_group)
{
	int i;

	for (i = 0; i <= FT_ALLOC_ORDER_MAX; i++) {
		if (!ft_group->arena_order[i])
			continue;
		cds_ft_arena_destroy(ft_group->arena_order[i]);
		ft_group->arena_order[i] = NULL;
	}
}

