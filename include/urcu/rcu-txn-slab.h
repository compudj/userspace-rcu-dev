/* SPDX-License-Identifier: LGPL-2.1-or-later */
#ifndef _URCU_RCU_TXN_SLAB_H
#define _URCU_RCU_TXN_SLAB_H

/*
 * Generic per-CPU size-classed superblock slab (transaction descriptors).
 *
 * Both the concurrent (rcu-txn-mcas.h) and single-writer (rcu-txn-sw.h) engines
 * allocate one variable-size descriptor block per attempt and free it, cross
 * thread, from the reclaim worker.  A per-thread malloc cache cannot recycle a
 * cross-thread free, and at scale servicing every attempt from glibc serializes
 * on the process mmap_lock (arena growth -> mprotect).  This slab removes both
 * without an external allocator, and is shared by both engines: each declares a
 * struct urcu_slab with its own byte size-classes.
 *
 * One arena per (size class, cpu): a wfstack freelist plus a bump pointer into
 * RANGE-aligned mmap'd superblocks.  free() finds a block's ORIGIN arena from
 * the superblock header (RANGE-aligned, so header = ptr & ~(RANGE-1)), so a
 * block allocated on cpu X and freed by the reclaim worker -- on whatever cpu
 * it runs -- returns to arena X.  MP-producer (workers free) / single-consumer
 * (the pinned writer allocs): push never blocks the writer, pop is under the
 * wfstack pop lock (uncontended for one writer/arena; also serializes the cold
 * carve).  The push is wait-free in the sense the wfstack means it -- the
 * PRODUCER always completes -- but one caught between its xchg and its link
 * store leaves the chain momentarily broken, and a concurrent pop busy-waits
 * for it: the CONSUMER is bounded-blocking, the same progress class as the
 * engine itself.
 *
 * Growth is capped by construction: alloc reuses a freed block before carving,
 * so the mapped footprint never exceeds peak live descriptors -- PER (class,
 * cpu) ARENA, not globally.  A writer that migrates carves on its new cpu while
 * the blocks it freed strand in their origin arena, so the process-wide
 * footprint tracks the SUM of the per-arena peaks: bounded, not minimal.
 *
 * Superblocks are left demand-paged (no MADV_HUGEPAGE: a partial superblock then
 * stays resident only for touched pages).  URCU_TXN_NO_CACHE (environment,
 * checked at init) disables the slab (the engine falls back to malloc).
 * URCU_TXN_CACHE_STATS dumps reuse/footprint; the engines' slab INSTANCES are
 * defined once in liburcu-common (src/urcu-txn.c) where init -- hence stats
 * registration -- runs, so define it for the whole build (library and embedder
 * TUs alike): a TU-local define only adds increment instrumentation.
 */

#include <stddef.h>			/* offsetof, size_t */
#include <stdlib.h>			/* calloc, getenv */
#include <stdint.h>			/* uintptr_t */
#include <sched.h>			/* sched_getcpu */
#include <unistd.h>			/* sysconf */
#include <sys/mman.h>			/* mmap */
#include <urcu/compiler.h>
#include <urcu/uatomic.h>
#include <urcu/wfstack.h>		/* freelist: wait-free push, SC pop */

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Superblock size/alignment.  Overridable, but it must be IDENTICAL across
 * every TU of a process: a slab is shared process-wide (one instance in
 * liburcu-common), and free() derives a block's superblock header by masking
 * with the freeing TU's RANGE.
 */
#ifndef URCU_SLAB_RANGE
#define URCU_SLAB_RANGE		(1UL << 21)	/* 2 MiB superblocks (1 THP) */
#endif
#define URCU_SLAB_RANGE_MASK	(URCU_SLAB_RANGE - 1)

/*
 * free() recovers a block's superblock header by masking the block address with
 * ~RANGE_MASK.  That identity holds only for a power-of-two RANGE; anything
 * else silently derives a garbage header (hence a garbage sb->owner arena).
 */
urcu_static_assert(!(URCU_SLAB_RANGE & URCU_SLAB_RANGE_MASK),
		"URCU_SLAB_RANGE must be a power of two: free() masks with it "
		"to find a block's superblock header",
		URCU_SLAB_RANGE_not_a_power_of_two);

struct urcu_slab_arena;
struct urcu_slab_sb {			/* header at the RANGE-aligned superblock base */
	struct urcu_slab_arena *owner;
	size_t bump;			/* next free byte offset within this superblock */
	struct urcu_slab_sb *next;	/* arena's superblock list (teardown/audit) */
};
struct urcu_slab_arena {
	struct cds_wfs_stack freelist;	/* MP wait-free push (free), SC pop (alloc) */
	struct urcu_slab_sb *sb;	/* current bump superblock + list head */
	size_t obj;			/* block size for this arena's class */
	char _pad[64];			/* keep arenas off each other's cachelines */
};
struct urcu_slab {
	struct urcu_slab_arena *arenas;	/* [nclass * ncpu], row-major by class */
	const size_t *class_size;	/* ascending byte size per class */
	int nclass;
	int ncpu;			/* 0 => disabled (engine falls back to malloc) */
	/*
	 * Stats fields are UNCONDITIONAL: the instance is shared across TUs
	 * (one strong definition in liburcu-common), while the URCU_SLAB_STAT
	 * increments compile per-TU -- the struct layout must not depend on a
	 * per-TU flag, or a stats-built embedder would scribble past a
	 * non-stats-built library object.
	 */
	const char *name;
	unsigned long st_reuse, st_carve, st_sbs;
};

#ifdef URCU_TXN_CACHE_STATS
#include <stdio.h>
#define URCU_SLAB_MAX_REG	8
static struct urcu_slab *urcu_slab_registry[URCU_SLAB_MAX_REG];
static int urcu_slab_nreg;
#define URCU_SLAB_STAT(s, f)	uatomic_inc(&(s)->st_##f)
static __attribute__((destructor))
void urcu_slab_stats_dump(void)
{
	int i;

	for (i = 0; i < urcu_slab_nreg; i++) {
		struct urcu_slab *s = urcu_slab_registry[i];
		unsigned long r = s->st_reuse, c = s->st_carve, sb = s->st_sbs;

		fprintf(stderr,
			"[slab %s] alloc reuse=%lu carve=%lu reuse%%=%.1f | mapped=%lu"
			" superblocks (%.1f MiB) over %d classes x %d cpu\n",
			s->name, r, c, 100.0 * (double) r / (double) (r + c + 1), sb,
			(double) sb * (double) URCU_SLAB_RANGE / (1024.0 * 1024.0),
			s->nclass, s->ncpu);
	}
}
#else
#define URCU_SLAB_STAT(s, f)	do { } while (0)
#endif

/* Smallest class that fits @bytes, or -1 if larger than the top class. */
static inline
int urcu_slab_class_of(const struct urcu_slab *s, size_t bytes)
{
	int i;

	for (i = 0; i < s->nclass; i++)
		if (bytes <= s->class_size[i])
			return i;
	return -1;
}

static inline
int urcu_slab_enabled(const struct urcu_slab *s)
{
	return s->ncpu > 0;
}

/*
 * Initialize @s with @nclass ascending byte size-classes (the array must stay
 * live -- pass a static const).  URCU_TXN_NO_CACHE, an invalid class table, or
 * OOM leaves it disabled (the engine falls back to malloc).  Call once, from
 * the engine's constructor, before any alloc.
 *
 * The class table must satisfy: sizes ASCENDING, each a multiple of 16, and
 * each one small enough to fit a superblock past its header.  All three are
 * checked here.
 */
static inline
void urcu_slab_init(struct urcu_slab *s, const size_t *class_size, int nclass,
		const char *name)
{
	size_t hdr;
	long n;
	int cl, c;

	s->arenas = NULL;
	s->ncpu = 0;
	s->class_size = class_size;
	s->nclass = nclass;
	s->name = name;
	s->st_reuse = s->st_carve = s->st_sbs = 0;
	if (getenv("URCU_TXN_NO_CACHE"))
		return;
	/*
	 * Validate the table BEFORE enabling anything.  Each precondition below
	 * is silently catastrophic if violated and costs one check, once; a bad
	 * table leaves the slab disabled -- degraded, never corrupt -- exactly
	 * like the OOM path just below.
	 *
	 *  - FITS a superblock past the header.  Otherwise carve returns a
	 *    block extending past the RANGE mapping (SIGSEGV on touch), and
	 *    free()'s mask then derives sb->owner from the NEXT window: a
	 *    garbage arena.
	 *  - MULTIPLE OF 16.  The first block starts 16-aligned and carve bumps
	 *    by exactly obj, so this is what keeps every later block in the
	 *    superblock 16-aligned -- which is what leaves a descriptor's low 4
	 *    bits free for the engine's proxy tag.
	 *  - ASCENDING, so urcu_slab_class()'s first-fit scan returns the
	 *    SMALLEST class that fits rather than an arbitrary one.
	 */
	hdr = (sizeof(struct urcu_slab_sb) + 15) & ~(size_t) 15;
	for (cl = 0; cl < nclass; cl++) {
		if (!class_size[cl] || class_size[cl] % 16)
			return;			/* not a 16-byte multiple */
		if (class_size[cl] > URCU_SLAB_RANGE - hdr)
			return;			/* would carve past the superblock */
		if (cl && class_size[cl] <= class_size[cl - 1])
			return;			/* not ascending */
	}
	n = sysconf(_SC_NPROCESSORS_CONF);
	if (n < 1)
		n = 1;
	s->arenas = (struct urcu_slab_arena *)
		calloc((size_t) nclass * (size_t) n, sizeof(*s->arenas));
	if (!s->arenas)
		return;				/* OOM: stay disabled */
	for (cl = 0; cl < nclass; cl++) {
		for (c = 0; c < (int) n; c++) {
			struct urcu_slab_arena *a = &s->arenas[cl * (int) n + c];

			cds_wfs_init(&a->freelist);
			a->obj = class_size[cl];
		}
	}
	s->ncpu = (int) n;
#ifdef URCU_TXN_CACHE_STATS
	if (urcu_slab_nreg < URCU_SLAB_MAX_REG)
		urcu_slab_registry[urcu_slab_nreg++] = s;
#endif
}

/* Map one RANGE-aligned superblock and prepend it to arena @a's list. */
static inline
struct urcu_slab_sb *urcu_slab_sb_new(struct urcu_slab *s, struct urcu_slab_arena *a)
{
	size_t raw = 2 * URCU_SLAB_RANGE;
	char *p = (char *) mmap(NULL, raw, PROT_READ | PROT_WRITE,
			MAP_PRIVATE | MAP_ANONYMOUS | MAP_NORESERVE, -1, 0);
	char *base;
	struct urcu_slab_sb *sb;

	(void) s;
	if (p == MAP_FAILED)
		return NULL;
	base = (char *) (((uintptr_t) p + URCU_SLAB_RANGE_MASK) &
			~(uintptr_t) URCU_SLAB_RANGE_MASK);
	if (base != p)				/* trim slack so only the aligned RANGE stays mapped */
		munmap(p, base - p);
	if (base + URCU_SLAB_RANGE != p + raw)
		munmap(base + URCU_SLAB_RANGE, (p + raw) - (base + URCU_SLAB_RANGE));
	sb = (struct urcu_slab_sb *) base;
	sb->owner = a;
	sb->bump = (sizeof(*sb) + 15) & ~(size_t) 15;	/* objects start past the header */
	sb->next = a->sb;
	URCU_SLAB_STAT(s, sbs);
	return sb;
}

/*
 * Allocate one block of size class @cl from the current CPU's arena, or NULL.
 * @s MUST be enabled (urcu_slab_enabled()) and @cl a valid class index: a
 * disabled slab has arenas == NULL and this dereferences it.  Every in-tree
 * caller gates on urcu_slab_enabled() and falls back to malloc.
 */
static inline
void *urcu_slab_alloc(struct urcu_slab *s, int cl)
{
	int cpu = sched_getcpu();
	struct urcu_slab_arena *a;
	struct cds_wfs_node *node;
	void *p;

	if (cpu < 0 || cpu >= s->ncpu)
		cpu = 0;
	a = &s->arenas[cl * s->ncpu + cpu];
	cds_wfs_pop_lock(&a->freelist);
	node = __cds_wfs_pop_blocking(&a->freelist);
	if (node) {				/* reuse before carve -- caps the footprint */
		cds_wfs_pop_unlock(&a->freelist);
		URCU_SLAB_STAT(s, reuse);
		return (void *) node;
	}
	/* carve from the active superblock (cold), under the pop lock so two
	 * would-be consumers never carve the same bump concurrently */
	if (!a->sb || a->sb->bump + a->obj > URCU_SLAB_RANGE) {
		struct urcu_slab_sb *nsb = urcu_slab_sb_new(s, a);

		if (!nsb) {
			cds_wfs_pop_unlock(&a->freelist);
			return NULL;
		}
		a->sb = nsb;
	}
	p = (char *) a->sb + a->sb->bump;
	a->sb->bump += a->obj;
	cds_wfs_pop_unlock(&a->freelist);
	URCU_SLAB_STAT(s, carve);
	return p;
}

/*
 * Free @block to its ORIGIN arena (found from the RANGE-aligned superblock),
 * wait-free: the reclaim worker never blocks the writer.  @block must be a slab
 * block (the caller distinguishes slab vs exact-malloc blocks by its own tag,
 * e.g. a capacity field, before calling this).
 */
static inline
void urcu_slab_free(void *block)
{
	struct urcu_slab_sb *sb = (struct urcu_slab_sb *)
			((uintptr_t) block & ~(uintptr_t) URCU_SLAB_RANGE_MASK);
	struct urcu_slab_arena *a = sb->owner;
	struct cds_wfs_node *node = (struct cds_wfs_node *) block;

	cds_wfs_node_init(node);
	cds_wfs_push(&a->freelist, node);
}

#ifdef __cplusplus
}
#endif

#endif /* _URCU_RCU_TXN_SLAB_H */
