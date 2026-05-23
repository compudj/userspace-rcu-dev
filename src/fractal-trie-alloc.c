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
 *
 * Transparent hugepages are a separate, orthogonal concern: FT actively
 * opts OUT of THP on its arenas (ft_disable_thp / MADV_NOHUGEPAGE) because
 * 2 MiB pages are a measured net loss for the trie's random pointer-chase
 * (they pin the L2/L3 page-colour bits to the virtual offset, so the
 * arena's strided node accesses self-conflict in the cache).  The
 * 2 MiB-granular interleave below is about NUMA placement only and is
 * unaffected — it stands whether the backing pages are 4 KiB or larger.
 */
#ifdef __linux__
/*
 * mempolicy mode numbers — match the linux/mempolicy.h enum.  Re-declared
 * locally to avoid pulling in the libnuma headers.
 */
#define FT_MPOL_DEFAULT		0
#define FT_MPOL_PREFERRED	1
#define FT_MPOL_BIND		2
#define FT_MPOL_INTERLEAVE	3
#define FT_MPOL_LOCAL		4	/* Linux 3.8+ */
#define FT_MPOL_F_MEMS_ALLOWED	(1U << 2)
#define FT_MAX_NUMA_NODES	1024
#define FT_NODEMASK_LONGS	(FT_MAX_NUMA_NODES / (sizeof(unsigned long) * 8))
#define FT_NODEMASK_BITS_PER_LONG	(sizeof(unsigned long) * 8)
#define FT_HUGEPAGE_SIZE	(2UL * 1024 * 1024)	/* 2 MiB */

/*
 * MADV_NOHUGEPAGE (asm-generic/mman-common.h value 15).  glibc only
 * exposes it under _GNU_SOURCE, so define it locally — like the FT_MPOL_*
 * numbers above — to guarantee FT can opt its arenas out of THP
 * regardless of the feature-test macros the translation unit was built
 * with.  Without this the madvise() silently compiles out and THP is NOT
 * disabled on a kernel configured with transparent_hugepage=always.
 */
#ifndef MADV_NOHUGEPAGE
#define MADV_NOHUGEPAGE		15
#endif

/*
 * Returns 1 if the env var CDS_FT_NUMA_INTERLEAVE=0 is set — a debug
 * override telling the library to skip ALL mbind() calls regardless of
 * group policy.  Defers entirely to whatever the kernel / process
 * policy decides.  Returns 0 otherwise.
 */
static
int ft_interleave_env_forces_skip(void)
{
	static int cached = -1;
	const char *env;
	int v;

	v = uatomic_load(&cached, CMM_RELAXED);
	if (v != -1)
		return v;
	env = getenv("CDS_FT_NUMA_INTERLEAVE");
	v = (env && env[0] == '0') ? 1 : 0;
	uatomic_store(&cached, v, CMM_RELAXED);
	return v;
}

/*
 * Disable transparent hugepages (THP) on an FT arena mapping.
 *
 * THP is a measured NET LOSS for the fractal trie and is therefore
 * actively opted out of (MADV_NOHUGEPAGE), not merely left un-hinted —
 * the latter is insufficient on systems configured with THP "always",
 * where the kernel would back the arena with 2 MiB pages regardless.
 *
 * Why THP hurts FT (load-names dns, single-thread, Zen 4; perf-verified):
 *
 *   2 MiB pages slash TLB misses (~550x fewer L1 DTLB misses, ~170x
 *   fewer page-table walks) — but that is NOT the FT lookup bottleneck.
 *   A lookup is a random pointer-chase through a working set far larger
 *   than the LLC, so it is ~68% bound on LLC-miss DRAM latency.  Page
 *   size cannot change whether a line is cached, so it cannot touch that
 *   68%.  Meanwhile the TLB misses it does eliminate are CHEAP: the page
 *   tables for the hot set stay cache-resident, so each walk is ~15-20
 *   cycles, not a DRAM hit — only ~3% of cycles total.
 *
 *   Worse, 2 MiB pages ADD last-level cache-conflict misses (+8% LLC
 *   misses measured, net +5% cycles).  The L2/L3 are physically indexed,
 *   so the set index is a function of the physical "page colour" bits
 *   above the 4 KiB offset.  With 4 KiB pages the OS scatters frames,
 *   randomising those bits and spreading the arena's regular, strided
 *   node accesses across cache sets.  Inside a 2 MiB hugepage physical
 *   == virtual, so the colour bits are pinned to the virtual offset: the
 *   arena stride survives straight into the cache index and strided
 *   nodes collide into far fewer sets.  (L1 is virtually indexed from
 *   the page offset and is unaffected — consistent with the measured
 *   L1-flat / LLC-up signature.)
 *
 * So FT keeps 4 KiB pages for their page-colour entropy, and gets its
 * NUMA-locality win from the orthogonal 2 MiB-granular mbind interleave
 * (ft_apply_interleave), which is independent of page size.
 */
static
void ft_disable_thp(void *base __attribute__((unused)),
		size_t size __attribute__((unused)))
{
#ifdef MADV_NOHUGEPAGE
	(void) madvise(base, size, MADV_NOHUGEPAGE);
#endif
}

/*
 * Query the calling thread's process-default NUMA policy mode (set by
 * numactl, set_mempolicy(), or libnuma).  Returns one of FT_MPOL_*.
 * Returns FT_MPOL_DEFAULT on query failure (treat as "no policy set").
 */
static
int ft_query_process_mempolicy_mode(void)
{
	int mode = FT_MPOL_DEFAULT;
	long r;

	r = syscall(__NR_get_mempolicy, &mode, NULL, 0UL, NULL, 0UL);
	if (r < 0)
		return FT_MPOL_DEFAULT;
	return mode;
}

/*
 * Apply the configured NUMA placement policy to the @size bytes at
 * @base.  Three policies, all subject to the CDS_FT_NUMA_INTERLEAVE=0
 * env var (which forces a skip):
 *
 *   - CDS_FT_NUMA_INTERLEAVE: per-2 MiB-chunk mbind(MPOL_BIND) round-
 *     robin across the calling thread's allowed nodes.  Binding whole
 *     2 MiB spans (rather than the kernel's 4 KiB-fine interleave) keeps
 *     contiguous virtual ranges node-local, which the hardware
 *     prefetcher and NUMA locality favour — this coarse granularity is
 *     the measured win.  (FT pages stay 4 KiB; THP is disabled, see
 *     ft_disable_thp.)  Falls back to whole-region MPOL_INTERLEAVE at
 *     native page granularity when @base isn't 2 MiB-aligned, the region
 *     is smaller than 2 MiB, or only one node is allowed.
 *
 *   - CDS_FT_NUMA_LOCAL: single mbind(MPOL_LOCAL) over the whole
 *     region.  Pages allocated within @base land on the local node
 *     of whichever thread first faults each page.  Persists even if
 *     the process policy is set to something else (e.g., interleave-
 *     all under a numactl wrapper).
 *
 *   - CDS_FT_NUMA_DEFAULT: query the process / libnuma policy via
 *     get_mempolicy().  If the process policy is INTERLEAVE, treat
 *     this group as INTERLEAVE (apply 2 MiB-chunk MPOL_BIND so the
 *     kernel's intent gets THP-friendly placement).  For any other
 *     process policy (DEFAULT/PREFERRED/BIND/LOCAL), skip mbind and
 *     let the kernel honor the process policy directly.
 */
static
void ft_apply_interleave(void *base, size_t size,
		enum cds_ft_numa_policy policy)
{
	unsigned long nodemask[FT_NODEMASK_LONGS] = { 0 };
	int allowed_nodes[FT_MAX_NUMA_NODES];
	unsigned long any = 0;
	int nr_allowed = 0;
	size_t i, off;
	long r;

	if (ft_interleave_env_forces_skip())
		return;

	if (policy == CDS_FT_NUMA_DEFAULT) {
		/*
		 * Query process policy.  Translate INTERLEAVE upward so
		 * the kernel's process-wide intent gets our THP-friendly
		 * 2 MiB-granular placement.  Other modes: leave the
		 * region untouched and let the kernel honor the process
		 * policy at fault time.
		 */
		if (ft_query_process_mempolicy_mode() != FT_MPOL_INTERLEAVE)
			return;
		policy = CDS_FT_NUMA_INTERLEAVE;
	}

	if (policy == CDS_FT_NUMA_LOCAL) {
		/*
		 * MPOL_LOCAL (Linux 3.8+).  Single mbind over the whole
		 * region.  Pages fault local to the thread that touches
		 * them first; if THP is enabled, hugepages allocate
		 * locally too.  No nodemask needed.
		 */
		(void) syscall(__NR_mbind, base, size, FT_MPOL_LOCAL,
				NULL, 0UL, 0);
		return;
	}

	/* CDS_FT_NUMA_INTERLEAVE — per-2 MiB-chunk MPOL_BIND round-robin. */
	r = syscall(__NR_get_mempolicy, NULL, nodemask, (unsigned long) FT_MAX_NUMA_NODES,
			NULL, FT_MPOL_F_MEMS_ALLOWED);
	if (r < 0)
		return;
	for (i = 0; i < FT_NODEMASK_LONGS; i++)
		any |= nodemask[i];
	if (!any)
		return;
	/* Enumerate allowed node IDs. */
	for (i = 0; i < FT_MAX_NUMA_NODES; i++) {
		size_t idx = i / FT_NODEMASK_BITS_PER_LONG;
		size_t bit = i % FT_NODEMASK_BITS_PER_LONG;
		if (nodemask[idx] & (1UL << bit))
			allowed_nodes[nr_allowed++] = (int) i;
	}
	/*
	 * Per-2MB-chunk MPOL_BIND fast path: needs 2 MiB alignment, at
	 * least one hugepage of room, and multiple allowed nodes for
	 * round-robin to do anything useful.
	 */
	if (nr_allowed > 1 &&
	    ((uintptr_t) base & (FT_HUGEPAGE_SIZE - 1)) == 0 &&
	    size >= FT_HUGEPAGE_SIZE) {
		unsigned int chunk = 0;
		for (off = 0; off + FT_HUGEPAGE_SIZE <= size;
				off += FT_HUGEPAGE_SIZE, chunk++) {
			unsigned long single[FT_NODEMASK_LONGS] = { 0 };
			int node = allowed_nodes[chunk % (unsigned) nr_allowed];
			single[node / FT_NODEMASK_BITS_PER_LONG] =
				1UL << (node % FT_NODEMASK_BITS_PER_LONG);
			(void) syscall(__NR_mbind,
				(char *) base + off, FT_HUGEPAGE_SIZE,
				FT_MPOL_BIND, single,
				(unsigned long) FT_MAX_NUMA_NODES, 0);
		}
		/*
		 * Trailing sub-2MB region: spread it across all allowed
		 * nodes at native-page granularity.
		 */
		if (off < size) {
			(void) syscall(__NR_mbind,
				(char *) base + off, size - off,
				FT_MPOL_INTERLEAVE, nodemask,
				(unsigned long) FT_MAX_NUMA_NODES, 0);
		}
		return;
	}
	/* Fallback: whole region INTERLEAVE at native page granularity. */
	(void) syscall(__NR_mbind, base, size, FT_MPOL_INTERLEAVE,
			nodemask, (unsigned long) FT_MAX_NUMA_NODES, 0);
}
#else
static inline void ft_apply_interleave(void *base __attribute__((unused)),
		size_t size __attribute__((unused)),
		enum cds_ft_numa_policy policy __attribute__((unused))) {}
#endif

/*
 * Allocate a fresh superblock big enough to host at least one
 * range of size min_size.  Pages are mmap'd anonymous, so they are
 * lazily zero-initialized on first touch.
 *
 * NUMA placement: if @numa_policy is INTERLEAVE/DEFAULT (and not
 * overridden by CDS_FT_NUMA_INTERLEAVE=0), apply per-2 MiB-chunk
 * mbind(MPOL_BIND) round-robin across the calling thread's allowed
 * NUMA nodes.  The 2 MiB granularity (rather than the kernel's default
 * 4 KiB-fine interleave) keeps each contiguous 2 MiB virtual span on a
 * single node, which the hardware prefetcher and NUMA locality both
 * favour — this coarse interleave is the real measured win and is
 * independent of page size.
 *
 * Transparent hugepages are explicitly DISABLED on FT arenas
 * (MADV_NOHUGEPAGE); see ft_disable_thp() for the rationale.
 */
static
struct cds_ft_alloc_superblock *superblock_create(size_t min_size,
		enum cds_ft_numa_policy numa_policy)
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
	ft_apply_interleave(base, size, numa_policy);
	ft_disable_thp(base, size);
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
	sb = superblock_create(alloc_size_aligned,
			arena->ft_group->numa_policy);
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

static
struct cds_ft_metadata *cds_ft_alloc_item_from(struct cds_ft *ft,
		struct cds_ft_alloc_arena **arena_p,
		const char *arena_name,
		size_t item_len_order, bool bitmap)
{
	struct cds_ft_alloc_arena *arena;

	if (!cds_ft_page_size)
		cds_ft_page_size = urcu_get_page_len();
	if ((1UL << item_len_order) > cds_ft_page_size) {
		errno = EINVAL;
		return NULL;
	}
	arena = uatomic_load(arena_p, CMM_ACQUIRE);
	if (caa_unlikely(!arena)) {
		pthread_mutex_lock(&ft->group->arena_lock);
		arena = *arena_p;
		if (!arena) {
			arena = cds_ft_arena_create(ft->group, arena_name, item_len_order, bitmap);
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

struct cds_ft_metadata *cds_ft_alloc_item(struct cds_ft *ft, size_t item_len_order, bool bitmap)
{
	return cds_ft_alloc_item_from(ft,
		&ft->group->arena_order[item_len_order],
		"cds_ft_alloc", item_len_order, bitmap);
}

/*
 * Compressed-node allocation for skip-compressed groups: routes to a
 * separate arena set so compressed-node pages don't share allocator
 * pages with the internal nodes that ARE on the cand-mode descent
 * hot path.  Cold for lookup in skip-compressed groups; the descent
 * never reads compressed-node bodies, only the skip_len high bits.
 *
 * Non-speculative groups can still use this entry point but get the
 * same single-arena-set behavior as cds_ft_alloc_item (caller routes).
 */
__attribute__((visibility("hidden")))
struct cds_ft_metadata *cds_ft_alloc_compressed_item(struct cds_ft *ft,
		size_t item_len_order)
{
	struct cds_ft_alloc_arena **arena_p;

	if (ft->group->speculative)
		arena_p = &ft->group->compressed_arena_order[item_len_order];
	else
		arena_p = &ft->group->arena_order[item_len_order];
	return cds_ft_alloc_item_from(ft, arena_p,
		ft->group->speculative ? "cds_ft_alloc_compressed" : "cds_ft_alloc",
		item_len_order, false);
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
		if (ft_group->arena_order[i]) {
			cds_ft_arena_destroy(ft_group->arena_order[i]);
			ft_group->arena_order[i] = NULL;
		}
		if (ft_group->compressed_arena_order[i]) {
			cds_ft_arena_destroy(ft_group->compressed_arena_order[i]);
			ft_group->compressed_arena_order[i] = NULL;
		}
	}
}

/*
 * External-node arena (public API): power-of-two size-class buddy
 * allocator backed by a linked list of mmap'd ranges.
 *
 * Each range is FT_EXT_ARENA_RANGE_SIZE bytes, aligned to its own
 * size (via over-mmap-and-trim) so range_base = item & ~(SIZE-1).
 * A 1 MiB per-cell metadata bitmap (1 byte / 16-byte cell) at the
 * start of each range encodes "is this cell the start of an
 * allocated/free block, and at what order".  cds_ft_external_arena_free
 * recovers the order from any item pointer with one mask + one load.
 *
 * Cross-class buddy splitting: alloc(O) first checks freelist[O];
 * on miss, scans freelist[O+1..MAX] for a larger free block, pops
 * it, and splits it down to order O — each split pushes one
 * smaller-order half onto the corresponding freelist.
 *
 * Cross-class buddy merging: free(ptr) reads the block's order
 * from the cell map, then checks its buddy (at offset ^ (1 << O))
 * — if the buddy is also free at the same order, removes it from
 * its freelist, marks one cell as "interior", merges into an
 * order-(O+1) block, and recurses upward.
 *
 * The trailing FT_EXT_ARENA_GUARD_SIZE bytes of every range are
 * reserved as a never-allocated guard so the library's 32-byte
 * SIMD leaf compare can safely over-read past the last bump-
 * allocated slot in the range.
 */

#define FT_EXT_ARENA_MIN_ORDER		4	/* 16 B minimum slot */
#define FT_EXT_ARENA_MAX_ORDER		22	/* 4 MiB maximum slot */
#define FT_EXT_ARENA_NR_CLASSES					\
	(FT_EXT_ARENA_MAX_ORDER - FT_EXT_ARENA_MIN_ORDER + 1)
#define FT_EXT_ARENA_RANGE_ORDER	24	/* 16 MiB per range */
#define FT_EXT_ARENA_RANGE_SIZE		(1UL << FT_EXT_ARENA_RANGE_ORDER)
#define FT_EXT_ARENA_RANGE_MASK		(FT_EXT_ARENA_RANGE_SIZE - 1)
#define FT_EXT_ARENA_GUARD_SIZE		4096UL
#define FT_EXT_ARENA_MIN_SLOT_SIZE	(1UL << FT_EXT_ARENA_MIN_ORDER)
#define FT_EXT_ARENA_NUM_CELLS						\
	(FT_EXT_ARENA_RANGE_SIZE / FT_EXT_ARENA_MIN_SLOT_SIZE)
/*
 * Cell sentinel for "not a block start" (interior of a larger
 * block, or unallocated metadata region).  Real cell values
 * encode bit 0 = free, bits 1-7 = (order - MIN_ORDER); 0xFF is
 * unreachable as a real value (order_bias 0x7F = order 131 is
 * impossibly large).
 */
#define FT_EXT_ARENA_CELL_NONE		0xFF

static inline
uint8_t ft_ext_arena_cell_pack(int order, int is_free)
{
	return (uint8_t) ((((order) - FT_EXT_ARENA_MIN_ORDER) << 1) |
			((is_free) ? 1 : 0));
}

static inline
int ft_ext_arena_cell_order(uint8_t c)
{
	return (int) ((c >> 1) & 0x7F) + FT_EXT_ARENA_MIN_ORDER;
}

static inline
int ft_ext_arena_cell_is_free(uint8_t c)
{
	return c & 0x01;
}

/* Doubly-linked freelist node — fits in 16 B (MIN_SLOT_SIZE). */
struct cds_ft_external_arena_freenode {
	struct cds_ft_external_arena_freenode *next;
	struct cds_ft_external_arena_freenode *prev;
};

struct cds_ft_external_arena_range {
	struct cds_list_head node;		/* per-arena list */
	size_t bump;				/* next un-bumped byte offset */
	uint8_t cells[FT_EXT_ARENA_NUM_CELLS];	/* per-cell block metadata */
	/*
	 * Slots start at the first MIN_SLOT-aligned offset past
	 * sizeof(struct cds_ft_external_arena_range).
	 */
};

struct cds_ft_external_arena {
	pthread_mutex_t lock;
	struct cds_list_head ranges;
	struct cds_ft_external_arena_range *active_range;
	struct cds_ft_external_arena_freenode *freelist[FT_EXT_ARENA_NR_CLASSES];
};

static
int ft_ext_arena_order_for_size(size_t size)
{
	int o;

	if (size <= (1UL << FT_EXT_ARENA_MIN_ORDER))
		return FT_EXT_ARENA_MIN_ORDER;
	/* Smallest order with (1 << order) >= size. */
	o = 64 - __builtin_clzll((unsigned long long)(size - 1));
	return o;
}

static inline
size_t ft_ext_arena_offset_in_range(const void *ptr)
{
	return (uintptr_t) ptr & FT_EXT_ARENA_RANGE_MASK;
}

static inline
struct cds_ft_external_arena_range *ft_ext_arena_range_of(const void *ptr)
{
	return (struct cds_ft_external_arena_range *)
		((uintptr_t) ptr & ~(uintptr_t) FT_EXT_ARENA_RANGE_MASK);
}

static inline
size_t ft_ext_arena_cell_index(const void *ptr)
{
	return ft_ext_arena_offset_in_range(ptr) / FT_EXT_ARENA_MIN_SLOT_SIZE;
}

static
struct cds_ft_external_arena_range *
ft_ext_arena_range_create(void)
{
	void *raw, *aligned;
	uintptr_t pre, post;
	size_t raw_size = 2 * FT_EXT_ARENA_RANGE_SIZE;
	struct cds_ft_external_arena_range *r;

	raw = mmap(NULL, raw_size, PROT_READ | PROT_WRITE,
		MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
	if (raw == MAP_FAILED)
		return NULL;
	aligned = (void *)(((uintptr_t) raw + FT_EXT_ARENA_RANGE_MASK) &
			~(uintptr_t) FT_EXT_ARENA_RANGE_MASK);
	pre = (uintptr_t) aligned - (uintptr_t) raw;
	post = raw_size - pre - FT_EXT_ARENA_RANGE_SIZE;
	if (pre)
		(void) munmap(raw, pre);
	if (post)
		(void) munmap((char *) aligned + FT_EXT_ARENA_RANGE_SIZE, post);
	/*
	 * External arena isn't tied to a specific group, so it uses
	 * CDS_FT_NUMA_DEFAULT: defer to the process / libnuma policy.
	 * When the process policy is INTERLEAVE (typical numactl
	 * --interleave wrapper), ft_apply_interleave promotes it to
	 * 2 MiB-chunk MPOL_BIND for NUMA-local contiguous spans.  Otherwise
	 * (no process policy, or LOCAL/PREFERRED/BIND), the kernel
	 * honors the process choice at fault time.
	 */
	ft_apply_interleave(aligned, FT_EXT_ARENA_RANGE_SIZE,
			CDS_FT_NUMA_DEFAULT);
	/* Opt out of THP (net loss for FT — see ft_disable_thp). */
	ft_disable_thp(aligned, FT_EXT_ARENA_RANGE_SIZE);

	r = (struct cds_ft_external_arena_range *) aligned;
	CDS_INIT_LIST_HEAD(&r->node);
	/*
	 * First slot offset: round struct size up to MIN_SLOT_SIZE
	 * so the cell-index calculation works uniformly.  All cells
	 * (including those covering the header bytes) start at the
	 * "no block here" sentinel so merge buddy-checks against
	 * the unbumped tail or the header region read as "not a
	 * block start" and skip.
	 */
	memset(r->cells, FT_EXT_ARENA_CELL_NONE, sizeof(r->cells));
	r->bump = (sizeof(*r) + FT_EXT_ARENA_MIN_SLOT_SIZE - 1)
			& ~(FT_EXT_ARENA_MIN_SLOT_SIZE - 1);
	return r;
}

static inline
void ft_ext_arena_freelist_push(struct cds_ft_external_arena *a,
		int order, struct cds_ft_external_arena_freenode *fn)
{
	int idx = order - FT_EXT_ARENA_MIN_ORDER;
	struct cds_ft_external_arena_freenode *head = a->freelist[idx];

	fn->next = head;
	fn->prev = NULL;
	if (head)
		head->prev = fn;
	a->freelist[idx] = fn;
}

static inline
void ft_ext_arena_freelist_remove(struct cds_ft_external_arena *a,
		int order, struct cds_ft_external_arena_freenode *fn)
{
	int idx = order - FT_EXT_ARENA_MIN_ORDER;

	if (fn->prev)
		fn->prev->next = fn->next;
	else
		a->freelist[idx] = fn->next;
	if (fn->next)
		fn->next->prev = fn->prev;
}

static inline
struct cds_ft_external_arena_freenode *
ft_ext_arena_freelist_pop(struct cds_ft_external_arena *a, int order)
{
	int idx = order - FT_EXT_ARENA_MIN_ORDER;
	struct cds_ft_external_arena_freenode *fn = a->freelist[idx];

	if (fn)
		ft_ext_arena_freelist_remove(a, order, fn);
	return fn;
}

static
void *ft_ext_arena_bump_alloc(struct cds_ft_external_arena *a, int order)
{
	struct cds_ft_external_arena_range *r;
	size_t slot_size = 1UL << order;
	size_t aligned_bump;
	void *p;

	if (!a->active_range) {
		r = ft_ext_arena_range_create();
		if (!r)
			return NULL;
		cds_list_add(&r->node, &a->ranges);
		a->active_range = r;
	}
	r = a->active_range;
	aligned_bump = (r->bump + slot_size - 1) & ~(slot_size - 1);
	if (aligned_bump + slot_size + FT_EXT_ARENA_GUARD_SIZE
			> FT_EXT_ARENA_RANGE_SIZE) {
		/* Doesn't fit; start a fresh range. */
		r = ft_ext_arena_range_create();
		if (!r)
			return NULL;
		cds_list_add(&r->node, &a->ranges);
		a->active_range = r;
		aligned_bump = (r->bump + slot_size - 1) & ~(slot_size - 1);
		if (aligned_bump + slot_size + FT_EXT_ARENA_GUARD_SIZE
				> FT_EXT_ARENA_RANGE_SIZE)
			return NULL;	/* even a fresh range too small */
	}
	p = (char *) r + aligned_bump;
	r->bump = aligned_bump + slot_size;
	return p;
}

struct cds_ft_external_arena *cds_ft_external_arena_create(void)
{
	struct cds_ft_external_arena *a;
	int i;

	if (!cds_ft_page_size)
		cds_ft_page_size = urcu_get_page_len();
	a = calloc(1, sizeof(*a));
	if (!a)
		return NULL;
	pthread_mutex_init(&a->lock, NULL);
	CDS_INIT_LIST_HEAD(&a->ranges);
	a->active_range = NULL;
	for (i = 0; i < FT_EXT_ARENA_NR_CLASSES; i++)
		a->freelist[i] = NULL;
	return a;
}

void *cds_ft_external_arena_alloc(struct cds_ft_external_arena *a,
		size_t size)
{
	int target_order, cur;
	void *p = NULL;
	struct cds_ft_external_arena_range *r;

	if (!a || !size)
		return NULL;
	target_order = ft_ext_arena_order_for_size(size);
	if (target_order > FT_EXT_ARENA_MAX_ORDER)
		return NULL;
	if (target_order < FT_EXT_ARENA_MIN_ORDER)
		target_order = FT_EXT_ARENA_MIN_ORDER;

	pthread_mutex_lock(&a->lock);
	/* Scan freelists from target order upward. */
	for (cur = target_order; cur <= FT_EXT_ARENA_MAX_ORDER; cur++) {
		if (a->freelist[cur - FT_EXT_ARENA_MIN_ORDER]) {
			p = ft_ext_arena_freelist_pop(a, cur);
			break;
		}
	}
	if (!p) {
		/* No freelist; bump from active range (creating one if needed). */
		p = ft_ext_arena_bump_alloc(a, target_order);
		cur = target_order;
		if (!p)
			goto unlock;
	}
	/* Split @p (currently at order @cur) down to @target_order. */
	while (cur > target_order) {
		size_t lower_off = ft_ext_arena_offset_in_range(p);
		size_t upper_off = lower_off + (1UL << (cur - 1));
		struct cds_ft_external_arena_range *rr = ft_ext_arena_range_of(p);
		struct cds_ft_external_arena_freenode *upper =
			(struct cds_ft_external_arena_freenode *)
			((char *) rr + upper_off);

		cur--;
		rr->cells[upper_off / FT_EXT_ARENA_MIN_SLOT_SIZE] =
			ft_ext_arena_cell_pack(cur, 1);
		ft_ext_arena_freelist_push(a, cur, upper);
	}
	r = ft_ext_arena_range_of(p);
	r->cells[ft_ext_arena_cell_index(p)] = ft_ext_arena_cell_pack(target_order, 0);
unlock:
	pthread_mutex_unlock(&a->lock);
	if (p)
		memset(p, 0, 1UL << target_order);
	return p;
}

void cds_ft_external_arena_free(struct cds_ft_external_arena *a, void *ptr)
{
	struct cds_ft_external_arena_range *r;
	int order;
	uint8_t cell;

	if (!a || !ptr)
		return;
	r = ft_ext_arena_range_of(ptr);
	cell = r->cells[ft_ext_arena_cell_index(ptr)];
	order = ft_ext_arena_cell_order(cell);

	pthread_mutex_lock(&a->lock);
	/* Try to merge upward with the buddy at each order. */
	while (order < FT_EXT_ARENA_MAX_ORDER) {
		size_t off = ft_ext_arena_offset_in_range(ptr);
		size_t buddy_off = off ^ (1UL << order);
		uint8_t buddy_cell;
		struct cds_ft_external_arena_freenode *buddy;
		void *merged;
		void *cleared;

		buddy_cell = r->cells[buddy_off / FT_EXT_ARENA_MIN_SLOT_SIZE];
		if (buddy_cell != ft_ext_arena_cell_pack(order, 1))
			break;
		buddy = (struct cds_ft_external_arena_freenode *)
			((char *) r + buddy_off);
		ft_ext_arena_freelist_remove(a, order, buddy);
		merged = (off < buddy_off) ? ptr : (void *) buddy;
		cleared = (off < buddy_off) ? (void *) buddy : ptr;
		r->cells[ft_ext_arena_cell_index(cleared)] = FT_EXT_ARENA_CELL_NONE;
		ptr = merged;
		order++;
	}
	r->cells[ft_ext_arena_cell_index(ptr)] = ft_ext_arena_cell_pack(order, 1);
	ft_ext_arena_freelist_push(a, order,
		(struct cds_ft_external_arena_freenode *) ptr);
	pthread_mutex_unlock(&a->lock);
}

void cds_ft_external_arena_destroy(struct cds_ft_external_arena *a)
{
	struct cds_ft_external_arena_range *r, *tmp;

	if (!a)
		return;
	cds_list_for_each_entry_safe(r, tmp, &a->ranges, node) {
		cds_list_del(&r->node);
		(void) munmap(r, FT_EXT_ARENA_RANGE_SIZE);
	}
	pthread_mutex_destroy(&a->lock);
	free(a);
}

