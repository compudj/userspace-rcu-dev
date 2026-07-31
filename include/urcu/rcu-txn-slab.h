// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later
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
 * One arena per (size class, cpu): an lfstack freelist plus a bump pointer into
 * RANGE-aligned mmap'd superblocks.  free() finds a block's ORIGIN arena from
 * the superblock header (RANGE-aligned, so header = ptr & ~(RANGE-1)), so a
 * block allocated on cpu X and freed by the reclaim worker -- on whatever cpu
 * it runs -- returns to arena X.
 *
 * WHY LFSTACK AND NOT WFSTACK.  The asymmetry that matters here is that the
 * APPLICATION pops (it allocates a descriptor on its commit path) while the
 * reclaim worker pushes.  wfstack optimizes the other side: its push is
 * wait-free, but because it publishes the node with an xchg BEFORE linking
 * node->next, a producer preempted in between leaves the chain momentarily
 * broken and a concurrent pop busy-waits it out via ___cds_wfs_node_sync_next()
 * -- so the consumer, i.e. the latency-sensitive writer, is bounded-blocking on
 * a preempted worker.  lfstack links node->next while the node is still private
 * and publishes with a cmpxchg, so a chain is ALWAYS fully linked: the pop is
 * lock-free and never waits on anyone.  The price is that push becomes a
 * cmpxchg retry loop rather than wait-free, which the worker can afford.
 *
 * That the chain is always linked is also what lets a whole batch be handed on
 * WITHOUT walking it -- see the pending stack below.
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
#include <stdlib.h>			/* aligned_alloc, getenv */
#include <string.h>			/* memset */
#ifdef URCU_SLAB_DEBUG_OWNER
#include <stdio.h>
#endif
#include <stdint.h>			/* uintptr_t */
#include <sched.h>			/* sched_getcpu */
#include <unistd.h>			/* sysconf */
#include <sys/mman.h>			/* mmap */
#include <pthread.h>			/* floor bootstrap mutex */
#include <urcu/compiler.h>
#include <urcu/uatomic.h>
#include <urcu/lfstack.h>		/* freelist: link-before-publish push, LF pop */
#include <urcu/call-rcu.h>		/* struct rcu_head: the drain schedules itself */
#ifdef URCU_SLAB_RSEQ
#include <rseq/rseq.h>
#include <syscall.h>
#include <linux/membarrier.h>
#endif

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

/*
 * Default footprint budget for the WHOLE slab, in MiB (URCU_TXN_SLAB_MAX_MB;
 * 0 = unlimited).
 *
 * Process-wide, not per-arena.  A per-arena cap has to be guessed against
 * demand that is not a property of the arena at all: blocks in flight are
 * rate * reclaim latency, so on this machine a single pinned writer at ~50
 * ns/txn against a ~10 ms drain needs ~24 superblocks in ONE arena, while 2304
 * arenas sit empty.  Any per-arena number is then simultaneously too small for
 * the busy arena and meaningless as a bound on the process.  Budget the total
 * and let arenas take what they need.
 */
#ifndef URCU_SLAB_MAX_MB
#define URCU_SLAB_MAX_MB	1024UL
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

/*
 * A CLOSED batch, overlaid on the block that terminates it.
 *
 * The batch has to carry its own rcu_head, not borrow one from the arena.  With
 * a single per-arena head only one batch can be in flight, so a batch can only
 * be closed once the previous callback has run -- which leaves each block
 * sitting in the open batch for a whole worker cycle BEFORE its grace period
 * even starts, i.e. up to two cycles of latency and twice the in-flight
 * footprint.  Putting the head in the batch lets close happen on a size
 * threshold, independently of callback cadence, so a block waits only for its
 * batch to fill plus one grace period.
 *
 * The terminating block is the outgoing floor: it is already out of
 * circulation and its contents are dead, so it is free real estate.  ->node
 * stays at offset 0 because the splice writes it as the chain's tail.
 *
 * CONTRACT: once a block is handed to urcu_slab_free()/free_pending(), the slab
 * owns its first sizeof(struct urcu_slab_batch) bytes and will scribble list
 * and batch metadata there.  A freed block's contents are dead by definition,
 * so this costs nothing -- but an embedder that expects to read anything back
 * out of a freed block will not get it.
 */
struct urcu_slab_batch {
	struct cds_lfs_node *head;	/* the batch's chain head */
};

/* Close a batch once this many blocks have accumulated (URCU_TXN_BATCH_MAX). */
#ifndef URCU_SLAB_BATCH_MAX
#define URCU_SLAB_BATCH_MAX	1024UL
#endif

struct urcu_slab_arena;

struct urcu_slab_sb {			/* header at the RANGE-aligned superblock base */
	struct urcu_slab_arena *owner;
	size_t bump;			/* next free byte offset within this superblock */
	struct urcu_slab_sb *next;	/* arena's superblock list (teardown/audit) */
};

struct urcu_slab_arena {
	struct cds_lfs_stack freelist;	/* MP push (free), LF pop (alloc) */
	/*
	 * Blocks handed to urcu_slab_free_pending() land here instead of on the
	 * freelist, and urcu_slab_drain() later moves the WHOLE batch across
	 * with one xchg and one cmpxchg -- touching no block but the tail, so a
	 * batch of any size costs the same and the blocks stay cold until they
	 * are actually reused.  @floor is that batch's bottom: pop_all hands
	 * back only a head, so instead of swapping in NULL the drain swaps in a
	 * floor block of its own, and the chain it gets back is terminated by
	 * the floor installed on the PREVIOUS drain -- the tail is recalled,
	 * never discovered.  The floor rides back into the freelist with its
	 * batch, so it must be (and is) a real block of this arena's class.
	 */

	/*
	 * ---- CACHE LINE 0-1: the atomically shared heads ----
	 *
	 * Written by threads OTHER than this arena's cpu: a block returns to
	 * its ORIGIN arena, so a cross-cpu free lands on ->freelist here and a
	 * cross-cpu deferred free on ->pending, and the closer xchgs ->pending
	 * from the reclaim worker.  Keeping them away from the rseq-only state
	 * below is the point of the grouping: those remote writes must not
	 * invalidate the line the owning cpu reads on its fast path.
	 */
	struct cds_lfs_stack pending;
	struct cds_lfs_node *floor;
	size_t obj;			/* block size for this arena's class (carve only) */
	struct rcu_head close_head;	/* fallback closer, for a trickle of frees */

	/*
	 * ---- CACHE LINE 2: cold, or written once ----
	 */
	unsigned int close_queued;	/* a fallback closer is in flight */
	/*
	 * The local batch can only be closed by this arena's own cpu (its close
	 * commits with an rseq critical section), so the fallback closer -- which
	 * runs on the call_rcu worker, essentially never that cpu -- cannot do
	 * it.  It leaves this behind instead, and the next deferred free on the
	 * owning cpu closes the batch and clears it.  Two 32-bit flags share the
	 * word the single close_queued used to occupy, so the arena stays 4 lines.
	 */
	unsigned int close_local_req;	/* the closer wants the local batch closed */
	/*
	 * Serializes the floor bootstrap AND the batch close.  Both are
	 * close-side-only state, so a per-arena mutex costs the free fast path
	 * nothing: bootstrap runs once per arena ever, and a close is taken with
	 * pthread_mutex_trylock() -- a closer that finds the arena already being
	 * closed simply lets that closer cover it.
	 *
	 * Lock order is boot -> freelist pop lock (urcu_slab_take_floor() runs
	 * under boot); nothing takes boot while holding the pop lock.
	 */
	pthread_mutex_t boot;
	struct urcu_slab *slab;		/* owning slab */
	int idx;			/* this arena's index within slab->arenas */
	int cls;			/* size class */

	/*
	 * ---- CACHE LINE 3: state only this arena's cpu touches ----
	 *
	 * The rseq fast paths -- alloc's local pop, free's local push, the
	 * deferred free's local_pending push -- read and write only these, so
	 * they sit together and away from the shared heads above.  @cpu opens
	 * the line because every one of those paths compares it against the
	 * rseq cpu before entering its critical section.
	 *
	 * @nr_pending is the exception and is deliberate: a CROSS-CPU deferred
	 * free bumps the origin arena's counter, so a remote write does land
	 * here.  It is kept anyway because the same-cpu case -- 99.99% of
	 * deferred frees, measured -- writes it alongside @local_pending and
	 * @local_floor in this same line, and paying a second line on the
	 * common path to spare the rare one is the wrong trade.
	 */
	int cpu;			/* cpu this arena belongs to */
	/*
	 * Non-zero while this arena may be operated on with rseq critical
	 * sections instead of atomics.  Read INSIDE every such section, so a
	 * demote followed by membarrier(...EXPEDITED_RSEQ) provably evicts any
	 * section still running on a stale decision -- a check before the
	 * section would be sampled once and survive the restart.
	 *
	 * One-way: an operation forced to run from the wrong cpu clears it for
	 * good and everything falls back to the atomic path.  Promotion back
	 * would have to prove no atomic operation is still in flight, which
	 * costs more than a permanently-demoted arena does.
	 */
	/*
	 * LOCAL freelist: a plain pointer, mutated ONLY by rseq critical
	 * sections running on this arena's own cpu.  Disjoint from ->freelist,
	 * which stays atomic -- that disjointness is the whole point.  An rseq
	 * plain-store commit and an atomic RMW cannot share a word (rseq's
	 * atomicity is scoped to one cpu, the RMW's is global, and neither can
	 * arbitrate the other), so instead of arbitrating them we keep them on
	 * separate lists and never let them meet.  No mode word, no membarrier.
	 *
	 *   same-cpu free  -> local  (rseq, no atomic at all)
	 *   cross-cpu free -> freelist (atomic; ORIGIN arena, contract intact)
	 *   alloc          -> pop local; when dry, pull the WHOLE freelist
	 *                     across with one xchg and install it as local
	 *
	 * One reserved value: &slab->dead, the self-linked node a drain installs
	 * here to retire the list (urcu_slab_take_local()).  It is never a real
	 * block and never appears in a chain.
	 */
	struct cds_lfs_node *local;
	/*
	 * The DEFERRED-FREE counterpart of ->local, and the other half of the
	 * integration: a same-cpu urcu_slab_free_pending() pushes here with an
	 * rseq critical section instead of a cmpxchg into ->pending.  Leaving
	 * this list atomic was worth ~35% of cycles -- the commit path performs
	 * exactly one of these per transaction, so it is as hot as the alloc it
	 * pairs with, and rseq-ifying only the alloc side left most of the win
	 * on the table.
	 *
	 * Same floor discipline as ->pending: the batch's tail is the floor
	 * installed by the previous close, recalled rather than discovered.
	 */
	struct cds_lfs_node *local_pending;
	struct cds_lfs_node *local_floor;
	/*
	 * Non-zero while the local lists may be touched with rseq.  Read INSIDE
	 * every critical section that STORES A VALUE OF ITS OWN CHOOSING -- the
	 * pushes, the refill install, the local close -- so clearing it and then
	 * issuing membarrier(...EXPEDITED_RSEQ) provably evicts any section still
	 * running on a stale decision; a check before the section would be
	 * sampled once and survive the restart.
	 *
	 * The local POP is the exception, and deliberately so: its two compares
	 * are spent on the head and on head->next (without the second one it has
	 * the ABA of an unlocked Treiber pop -- see urcu_slab_local_pop()), which
	 * leaves no slot for this flag.  A pop is stopped instead by the value in
	 * the head word, which is sound only because a pop's commit value is
	 * DERIVED from what it loaded: parking the self-linked &slab->dead there
	 * makes the pop store @dead straight back.  See urcu_slab_take_local().
	 *
	 * One-way.  Promotion back would have to prove no atomic operation is
	 * still in flight against ->local, which needs the fast and slow paths
	 * to share a critical section -- a much larger change than a
	 * permanently-demoted arena costs.
	 */
	unsigned long rseq_ok;

	struct urcu_slab_sb *sb;	/* current bump superblock + list head */
	unsigned long nr_sb;		/* superblocks mapped by this arena */
	/*
	 * Pushes since the last close (APPROXIMATE -- see urcu_slab_free_pending).
	 *
	 * Kept here, with @local_pending/@local_floor/@rseq_ok, because the
	 * deferred-free fast path touches all four: sitting up beside @floor it
	 * cost that path a SECOND arena cacheline for a counter that only picks
	 * a close point.  @obj took its place, being read on the carve path
	 * only.  The arena is 4 lines exactly, so this is a swap, not growth.
	 */
	unsigned long nr_pending;
	/*
	 * Arenas must not share cachelines: neighbours belong to OTHER cpus, so
	 * false sharing here is cross-cpu traffic on the hottest structure in
	 * the allocator.  A trailing pad alone never achieved that -- it left
	 * alignof at 8 and the size at a non-multiple of 64, so consecutive
	 * arenas straddled lines regardless.  Align the type, and allocate the
	 * array with matching alignment (calloc only promises 16).
	 */
} __attribute__((aligned(64)));

struct urcu_slab {
	struct urcu_slab_arena *arenas;	/* [nclass * ncpu], row-major by class */
	const size_t *class_size;	/* ascending byte size per class */
	int nclass;
	int ncpu;			/* 0 => disabled (engine falls back to malloc) */
	/*
	 * Footprint cap, in superblocks per arena (0 = unlimited).
	 *
	 * Superblocks are NEVER unmapped, so without a cap a transient burst --
	 * anything that makes allocation outrun the drain, since freed blocks
	 * only become reusable when a batch is spliced back -- inflates the
	 * process permanently.  Past the cap urcu_slab_alloc() returns NULL and
	 * the engines fall back to posix_memalign(), which is slower but hands
	 * the memory back to libc when the burst ends.  So the cap bounds what
	 * is permanent, not what is live.
	 */
	/*
	 * Byte offset within a block of the scratch word the slab links through.
	 *
	 * It CANNOT be offset 0.  A block reaches free_pending() at commit, one
	 * grace period BEFORE readers are done with it -- offset 0 is live
	 * reader state in both engines (urcu_txn_desc::status, which a proxy
	 * resolve loads; urcu_txn_sw_block::group, which parked proxies point
	 * at).  Linking there corrupts exactly the field the deferral exists to
	 * protect.  Point this at the block's rcu_head, which call_rcu already
	 * writes at deferral time for the same reason: nobody reads it.
	 */
	size_t link_off;
	/*
	 * Where a closed batch's metadata is overlaid on its floor block.  It
	 * must clear the link node: the floor is the chain's TAIL, so its link
	 * word holds the terminator (and later the splice's forward pointer)
	 * while its batch rcu_head is simultaneously queued in call_rcu.  Same
	 * block, two live roles, so they cannot share an offset.
	 */
	size_t batch_off;
	unsigned long max_sb_total;	/* budget, in superblocks (0 = unlimited) */
	unsigned long nr_sb_total;	/* superblocks currently mapped */
	unsigned long batch_max;	/* close a batch at this many blocks */
	/*
	 * Deferral used to schedule batch closes and splices.  A parameter, not a
	 * hardcoded call_rcu(), so the slab stays flavor-agnostic like the engines
	 * above it; struct rcu_head itself is flavor-independent.
	 *
	 * PRECONDITION: every urcu_slab_free_pending() caller of one slab must
	 * pass the SAME function.  A grace period is flavor-scoped, and this field
	 * is slab-wide while the engines' slab instances are process-wide, so two
	 * embedders on two flavors would have whichever freed last decide when a
	 * batch full of the OTHER flavor's descriptors becomes allocatable --
	 * reader use-after-free.  urcu_slab_free() has no such constraint: there,
	 * each caller's own flavor gates its own free.  free_pending() asserts on
	 * a mismatch; an NDEBUG build cannot detect the violation at all.
	 */
	void (*call_rcu_fn)(struct rcu_head *, void (*)(struct rcu_head *));
	/*
	 * Retirement marker for the rseq local freelists: a node whose ->next is
	 * itself.  Parked in an arena's ->local by urcu_slab_take_local(), where
	 * it turns any straggling rseq pop into a no-op that stores @dead back.
	 * One per SLAB, not per TU: a file-scope static would give each
	 * translation unit its own marker, and a pop comparing against the wrong
	 * one would hand @dead out as a block.
	 */
	struct cds_lfs_node dead;
	/*
	 * Stats fields are UNCONDITIONAL: the instance is shared across TUs
	 * (one strong definition in liburcu-common), while the URCU_SLAB_STAT
	 * increments compile per-TU -- the struct layout must not depend on a
	 * per-TU flag, or a stats-built embedder would scribble past a
	 * non-stats-built library object.
	 */
	const char *name;
	unsigned long st_reuse, st_carve, st_sbs;
	/* which PATH served the op: rseq-local vs atomic fallback */
	unsigned long st_a_local, st_a_refill, st_a_slow;
	unsigned long st_f_local, st_f_slow;
	unsigned long st_p_local, st_p_slow;
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
		{
			unsigned long al = s->st_a_local, ar = s->st_a_refill,
				as = s->st_a_slow, fl = s->st_f_local,
				fs = s->st_f_slow, pl = s->st_p_local,
				ps = s->st_p_slow;

			fprintf(stderr,
			  "[slab %s]   alloc: local=%lu refill=%lu slow=%lu (local%%=%.2f)\n"
			  "[slab %s]   free : local=%lu slow=%lu (local%%=%.2f)\n"
			  "[slab %s]   defer: local=%lu slow=%lu (local%%=%.2f)\n",
			  s->name, al, ar, as,
			  100.0 * (double) al / (double) (al + ar + as + 1),
			  s->name, fl, fs,
			  100.0 * (double) fl / (double) (fl + fs + 1),
			  s->name, pl, ps,
			  100.0 * (double) pl / (double) (pl + ps + 1));
		}
	}
}
#else
#define URCU_SLAB_STAT(s, f)	do { } while (0)
#endif

/*
 * Current cpu.  rseq's cpu_id is an inlined TLS load (~0.3 ns) against
 * sched_getcpu's un-inlinable PLT call (~3.1 ns); the value is the same.
 */
static inline
int urcu_slab_cpu(void)
{
#ifdef URCU_SLAB_RSEQ
	if (caa_likely(rseq_registered()))
		return rseq_current_cpu_raw();
#endif
	return sched_getcpu();
}

static inline
struct cds_lfs_node *urcu_slab_node(const struct urcu_slab *s, void *block)
{
	return (struct cds_lfs_node *) ((char *) block + s->link_off);
}

static inline
void *urcu_slab_block(const struct urcu_slab *s, struct cds_lfs_node *node)
{
	return (void *) ((char *) node - s->link_off);
}

static inline
struct urcu_slab_batch *urcu_slab_batch_of(const struct urcu_slab *s, void *block)
{
	return (struct urcu_slab_batch *) ((char *) block + s->batch_off);
}

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

/*
 * @ncpu doubles as the enable flag, and readers dereference ->arenas straight
 * after seeing it non-zero -- so it is published with a release store and read
 * with an acquire load.  The normal sequence (constructor, before threads) does
 * not need it, but a constructor-spawned thread committing while another
 * constructor is still inside urcu_slab_init() would otherwise be allowed to
 * see the count without the arenas.
 */
static inline
int urcu_slab_enabled(const struct urcu_slab *s)
{
	return uatomic_load(&s->ncpu, CMM_ACQUIRE) > 0;
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
		const char *name, size_t link_off)
{
	size_t hdr;
	long n;
	int cl, c;

	s->arenas = NULL;
	s->ncpu = 0;
	s->class_size = class_size;
	s->nclass = nclass;
	s->name = name;
	s->link_off = link_off;
	/*
	 * Just past the rcu_head.  The floor block's rcu_head does double duty:
	 * it is the chain's terminator, and it is the handle call_rcu queues --
	 * but never at the same time.  It terminates until the callback fires;
	 * the splice writes it as a forward pointer only afterwards.  So the
	 * batch needs to stash exactly one extra word, its chain head.
	 */
	s->batch_off = (link_off + sizeof(struct rcu_head) + 7) & ~(size_t) 7;
	s->st_reuse = s->st_carve = s->st_sbs = 0;
	s->call_rcu_fn = NULL;
	s->dead.next = &s->dead;		/* self-linked: see urcu_slab::dead */
	s->batch_max = URCU_SLAB_BATCH_MAX;
	{
		const char *e = getenv("URCU_TXN_BATCH_MAX");

		if (e)
			s->batch_max = strtoul(e, NULL, 10);
		if (!s->batch_max)
			s->batch_max = 1;
	}
	s->nr_sb_total = 0;
	s->max_sb_total = (URCU_SLAB_MAX_MB << 20) / URCU_SLAB_RANGE;
	{
		const char *e = getenv("URCU_TXN_SLAB_MAX_MB");

		if (e)
			s->max_sb_total = (strtoul(e, NULL, 10) << 20) /
					URCU_SLAB_RANGE;
	}
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
		if (class_size[cl] < s->batch_off + sizeof(struct urcu_slab_batch))
			return;			/* no room for the batch overlay */
		if (class_size[cl] > URCU_SLAB_RANGE - hdr)
			return;			/* would carve past the superblock */
		if (cl && class_size[cl] <= class_size[cl - 1])
			return;			/* not ascending */
	}
	n = sysconf(_SC_NPROCESSORS_CONF);
	if (n < 1)
		n = 1;
	{
		size_t bytes = (size_t) nclass * (size_t) n * sizeof(*s->arenas);

		s->arenas = (struct urcu_slab_arena *) aligned_alloc(64, bytes);
		if (!s->arenas)
			return;			/* OOM: stay disabled */
		memset(s->arenas, 0, bytes);
	}
	for (cl = 0; cl < nclass; cl++) {
		for (c = 0; c < (int) n; c++) {
			struct urcu_slab_arena *a = &s->arenas[cl * (int) n + c];

			cds_lfs_init(&a->freelist);
			cds_lfs_init(&a->pending);
			a->floor = NULL;
			pthread_mutex_init(&a->boot, NULL);
			a->slab = s;
			a->idx = cl * (int) n + c;
			a->cls = cl;
			a->cpu = c;
			a->local = NULL;
			a->local_pending = NULL;
			a->local_floor = NULL;
			a->rseq_ok = 0;
			a->nr_pending = 0;
			a->close_queued = 0;
			a->close_local_req = 0;
			a->obj = class_size[cl];
		}
	}
#ifdef URCU_SLAB_RSEQ
	/*
	 * Enable the rseq local lists only if we can ALSO fence.  Without
	 * membarrier(...EXPEDITED_RSEQ) a local list could never be drained
	 * from another cpu, so hotplug and teardown would have no safe way to
	 * reach it -- better to stay on the atomic path entirely.
	 */
	if (rseq_registered() &&
			!syscall(__NR_membarrier,
				MEMBARRIER_CMD_REGISTER_PRIVATE_EXPEDITED_RSEQ, 0, 0)) {
		for (cl = 0; cl < nclass; cl++)
			for (c = 0; c < (int) n; c++)
				s->arenas[cl * (int) n + c].rseq_ok = 1;
	}
#endif
	uatomic_store(&s->ncpu, (int) n, CMM_RELEASE);	/* publishes ->arenas */
#ifdef URCU_TXN_CACHE_STATS
	if (urcu_slab_nreg < URCU_SLAB_MAX_REG)
		urcu_slab_registry[urcu_slab_nreg++] = s;
#endif
}

/* Map one RANGE-aligned superblock and prepend it to arena @a's list. */
/*
 * Claim one superblock against the budget.  Returns 0 when the budget is spent,
 * at which point the caller returns NULL and the engines spill to
 * posix_memalign -- slower, but memory libc hands back when the burst ends,
 * whereas a superblock is never unmapped.
 */
static inline
int urcu_slab_reserve_sb(struct urcu_slab *s)
{
	unsigned long n;

	if (!s->max_sb_total)
		return 1;				/* unlimited */
	n = uatomic_add_return(&s->nr_sb_total, 1);
	if (n > s->max_sb_total) {
		uatomic_dec(&s->nr_sb_total);
		return 0;
	}
	return 1;
}

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
	a->nr_sb++;
	URCU_SLAB_STAT(s, sbs);
	return sb;
}

/*
 * Allocate one block of size class @cl from the current CPU's arena, or NULL.
 * @s MUST be enabled (urcu_slab_enabled()) and @cl a valid class index: a
 * disabled slab has arenas == NULL and this dereferences it.  Every in-tree
 * caller gates on urcu_slab_enabled() and falls back to malloc.
 */

/*
 * cds_lfs_push() primes its expected head with NULL, so on a NON-EMPTY stack
 * its first cmpxchg is guaranteed to fail: two locked RMWs per push.  That is
 * the right bet for a stack usually found empty and the wrong one for ours --
 * the pending stack always carries a floor, so it is never empty.  Prime with a
 * load instead and the common case costs one cmpxchg.
 */
static inline
void urcu_slab_push(struct cds_lfs_stack *s, struct cds_lfs_node *node)
{
	struct cds_lfs_head *head, *old;
	struct cds_lfs_head *new_head =
		caa_container_of(node, struct cds_lfs_head, node);

	head = uatomic_load(&s->head, CMM_RELAXED);
	for (;;) {
		old = head;
		node->next = &head->node;
		cmm_emit_legacy_smp_mb();
		head = uatomic_cmpxchg_mo(&s->head, old, new_head,
				CMM_SEQ_CST, CMM_SEQ_CST);
		if (caa_likely(old == head))
			return;
	}
}

#ifdef URCU_SLAB_RSEQ
/*
 * The local-list WRITERS -- both pushes, the refill install, the local close --
 * are the same shape: verify the head has not moved AND that the arena is still
 * rseq-eligible, then commit one plain store.  No atomic, and none is needed:
 * the list is only ever touched from this arena's own cpu, and rseq restarts us
 * if we were preempted or migrated anywhere inside.
 *
 * Whatever is written into the node happens BEFORE the commit and while the
 * node is still unreachable, so an abort leaves nothing observable: the same
 * link-before-publish discipline lfstack uses.
 *
 * The POP is shaped differently -- see below.
 */

/*
 * Pop one block off the local list.
 *
 * The successor MUST be loaded inside the critical section.  Reading
 * head->next outside it and passing the value in as the commit operand gives
 * this pop the exact ABA of an unlocked Treiber pop: the section value-checks
 * the head only, so a thread preempted between the two -- on THIS cpu, which is
 * the case the local list exists for -- can resume after its head has been
 * allocated, freed and pushed back, and install a stale successor that is by
 * then a live block.  Two allocations then return the same descriptor.
 * rseq_load_cbeq_store_add_load_store__ptr() dereferences the head inside the
 * section (add voffp, load, store), so the value it commits is correct AT the
 * commit instant regardless of what happened before it.
 *
 * The price is the second compare, which the writers spend on @rseq_ok: this
 * section cannot also check the demote flag.  What retires it instead is
 * @slab->dead sitting in the head word -- self-linked, so the section pops
 * @dead, stores @dead->next (i.e. @dead) back, and leaves the list exactly as
 * it found it.  The caller recognizes it and falls back to the atomic path.
 *
 * Returns NULL for dry, migrated, preempted and demoted alike: the caller
 * answers all four the same way, by refilling from the atomic freelist.
 */
static inline
struct cds_lfs_node *urcu_slab_local_pop(struct urcu_slab *s,
		struct urcu_slab_arena *a, int cpu)
{
	intptr_t popped = 0;

	if (rseq_unlikely(rseq_load_cbeq_store_add_load_store__ptr(
			RSEQ_MO_RELAXED, RSEQ_PERCPU_CPU_ID,
			(intptr_t *) &a->local, (intptr_t) NULL,
			(long) offsetof(struct cds_lfs_node, next),
			&popped, cpu)))
		return NULL;			/* empty, migrated or preempted */
	if (caa_unlikely((struct cds_lfs_node *) popped == &s->dead))
		return NULL;			/* retired: caller goes atomic */
	return (struct cds_lfs_node *) popped;
}

static inline
int urcu_slab_local_push(struct urcu_slab *s, struct urcu_slab_arena *a,
		struct cds_lfs_node *node, int cpu)
{
	for (;;) {
		struct cds_lfs_node *head = RSEQ_READ_ONCE(a->local);
		int ret;

		if (caa_unlikely(head == &s->dead))
			return -1;		/* retired: caller goes atomic */
		node->next = head;			/* private until the commit */
		ret = rseq_load_cbne_load_cbne_store__ptr(RSEQ_MO_RELAXED,
				RSEQ_PERCPU_CPU_ID, (intptr_t *) &a->local,
				(intptr_t) head, (intptr_t *) &a->rseq_ok,
				(intptr_t) 1, (intptr_t) node, cpu);
		if (rseq_likely(!ret))
			return 0;
		/*
		 * @cpu was sampled by the caller, which also used it to pick
		 * @a.  Once we migrate off it the cpu check aborts EVERY
		 * attempt, and retrying here would spin forever -- @cpu is
		 * loop-invariant, so nothing that made this fail can change.
		 * Re-sampling would not help either: the arena belongs to the
		 * old cpu.  Bail out and let the caller fall through to the
		 * atomic freelist.
		 */
		if (ret < 0)
			return -1;		/* migrated: caller goes atomic */
		if (!uatomic_load(&a->rseq_ok, CMM_RELAXED))
			return -1;		/* demoted: caller goes atomic */
		/* ne: raced on our own cpu, so a retry can make progress */
	}
}

/*
 * Install a whole chain as the local list, iff it is still empty.  A retired
 * list holds &slab->dead, which is not NULL, so this fails on it too.
 */
static inline
int urcu_slab_local_install(struct urcu_slab_arena *a,
		struct cds_lfs_node *chain, int cpu)
{
	return rseq_load_cbne_load_cbne_store__ptr(RSEQ_MO_RELAXED,
			RSEQ_PERCPU_CPU_ID, (intptr_t *) &a->local,
			(intptr_t) NULL, (intptr_t *) &a->rseq_ok, (intptr_t) 1,
			(intptr_t) chain, cpu);
}

static inline int urcu_slab_rseq_ready(void) { return rseq_registered(); }
#else
static inline int urcu_slab_rseq_ready(void) { return 0; }
#endif /* URCU_SLAB_RSEQ */

/*
 * Demote @a: ->local becomes off-limits to rseq, so it can be drained from any
 * cpu.  This is the ONLY way an arena's local list can be reached remotely --
 * hotplug of its cpu, or process teardown, both of which are inherently one
 * thread touching lists that belong to every other cpu.
 *
 * Split in two on purpose.  Clearing the flag is per-arena; the fence that
 * makes the clear observable to in-flight critical sections is process-wide, so
 * a caller demoting many arenas clears them ALL and then fences ONCE.
 */
static inline
void urcu_slab_demote_flag(struct urcu_slab_arena *a)
{
	uatomic_store(&a->rseq_ok, 0, CMM_SEQ_CST);
}

static inline
int urcu_slab_demote_fence(void)
{
#ifdef URCU_SLAB_RSEQ
	return syscall(__NR_membarrier,
			MEMBARRIER_CMD_PRIVATE_EXPEDITED_RSEQ, 0, 0);
#else
	return 0;
#endif
}

static inline void urcu_slab_closer_cb(struct rcu_head *head);

/*
 * Arm the fallback closer on @a, unless one is already in flight.
 */
static inline
void urcu_slab_arm_closer(struct urcu_slab *s, struct urcu_slab_arena *a)
{
	void (*call_rcu_fn)(struct rcu_head *, void (*)(struct rcu_head *)) =
		uatomic_load(&s->call_rcu_fn, CMM_RELAXED);

	if (!call_rcu_fn)
		return;				/* nobody uses free_pending here */
	if (!uatomic_load(&a->close_queued, CMM_RELAXED) &&
			uatomic_cmpxchg(&a->close_queued, 0, 1) == 0)
		call_rcu_fn(&a->close_head, urcu_slab_closer_cb);
}

/*
 * Retire @a->local: take whatever is on it and park &slab->dead in the head
 * word so no rseq pop can ever operate on it again.
 *
 * Unlike the pushes and the install, the pop does not compare @rseq_ok (see
 * urcu_slab_local_pop()), so urcu_slab_demote_flag() does not stop it.  What
 * stops it is @dead: a pop that loads it stores @dead->next -- @dead itself --
 * straight back, so the marker is self-preserving and the pop is a no-op.
 *
 * That leaves exactly one window.  A section that loaded a REAL head just
 * before our xchg, and commits just after it, overwrites @dead with a node of
 * the chain we now own -- and hands the node above it to an allocator.  The
 * fence evicts every section still executing; one that slipped past the fence
 * had to have committed, and re-reading the head detects that, because once
 * @dead is gone nothing can put it back: only a pop that READS @dead restores
 * it, and @dead is never linked into a chain.  So we simply take again.  The
 * retry keeps the surviving suffix and drops the prefix, which is precisely the
 * set of nodes that were handed out.  It terminates: pushes are already dead
 * (they do compare @rseq_ok) and every wipe consumes one node.
 *
 * An empty or already-retired list needs no fence -- rseq sections on one cpu
 * are mutually exclusive, so the only section that could be in flight is one
 * that loaded the same empty head, and an empty pop commits nothing.
 */
static inline
struct cds_lfs_node *urcu_slab_take_local(struct urcu_slab_arena *a)
{
	struct cds_lfs_node *dead = &a->slab->dead;

	for (;;) {
		struct cds_lfs_node *n = uatomic_xchg(&a->local, dead);

		if (!n || n == dead)
			return NULL;
		if (urcu_slab_demote_fence())
			return NULL;		/* strand rather than corrupt */
		if (uatomic_load(&a->local, CMM_RELAXED) == dead)
			return n;
	}
}

/*
 * Move a demoted arena's local list onto its atomic freelist.  Safe from any
 * cpu, but ONLY after urcu_slab_demote_flag() + urcu_slab_demote_fence():
 * before the fence a critical section past its compare can still commit, and
 * would race this.
 *
 * Pushes block by block rather than splicing: splicing needs the chain's tail,
 * which would mean walking anyway, and this runs on teardown/hotplug, not on
 * any hot path.
 */
static inline
void urcu_slab_drain_local(struct urcu_slab_arena *a)
{
	struct cds_lfs_node *n = urcu_slab_take_local(a);
	int moved_pending = 0;

	while (n) {
		struct cds_lfs_node *next = n->next;

		cds_lfs_node_init(n);
		urcu_slab_push(&a->freelist, n);
		n = next;
	}
	/*
	 * ->local_pending goes to ->pending, NOT to the freelist: those blocks
	 * are still awaiting their grace period, and handing them straight back
	 * to alloc would be a use-after-free.  It needs no @dead marker: every
	 * section that writes it does compare @rseq_ok, so the demote fence has
	 * already retired the list.
	 */
	n = uatomic_xchg(&a->local_pending, NULL);
	a->local_floor = NULL;
	while (n) {
		struct cds_lfs_node *next = n->next;

		cds_lfs_node_init(n);
		urcu_slab_push(&a->pending, n);
		n = next;
		moved_pending = 1;
	}
	/*
	 * Those blocks are owed a grace period and now sit on ->pending, where
	 * only a close can reach them.  Nothing else will arm one: the cpu whose
	 * frees used to do it is exactly the cpu that just went away.
	 */
	if (moved_pending)
		urcu_slab_arm_closer(a->slab, a);
}

/*
 * Demote every arena and fold its local list back.  One fence for the whole
 * slab, not one per arena.
 */
static inline
void urcu_slab_demote_all(struct urcu_slab *s)
{
	int i, n;

	if (!urcu_slab_enabled(s))
		return;
	n = s->nclass * s->ncpu;
	for (i = 0; i < n; i++)
		urcu_slab_demote_flag(&s->arenas[i]);
	if (urcu_slab_demote_fence()) {
		/*
		 * Without the fence a critical section past its compare can
		 * still commit into ->local, and draining would race it.
		 * Stranding those blocks is strictly better than corrupting
		 * them -- and init only enables rseq when the fence is known
		 * available, so this should not be reachable.
		 */
		return;
	}
	for (i = 0; i < n; i++)
		urcu_slab_drain_local(&s->arenas[i]);
}

/*
 * Fold ONE cpu's arenas back onto their atomic lists -- the hotplug unit.
 *
 * A cpu going offline strands whatever sits in its arenas' ->local and
 * ->local_pending: those lists are rseq-only, reachable from that cpu and no
 * other, so once it is gone nothing can pop them.  Worse for ->local_pending,
 * whose blocks are still owed a grace period and would never reach ->pending.
 * This is the sequence that makes both reachable again -- demote every arena of
 * @cpu, fence ONCE so no critical section can still commit into them, then fold
 * ->local onto the freelist and ->local_pending onto ->pending.
 *
 * Prefer this to urcu_slab_demote_all() for hotplug.  Offlining one cpu has no
 * business demoting the other ncpu-1: demotion is ONE-WAY, so demote_all would
 * drop the WHOLE process onto the atomic path for a single cpu's departure.
 * Here the blast radius is @cpu's own nclass arenas, while the membarrier --
 * the expensive part, an IPI to every cpu -- is still issued exactly once.
 *
 * The one-way property still bites inside that radius: if @cpu comes back
 * online its arenas stay atomic for the life of the process.  That is the
 * documented trade (see ->rseq_ok); the point here is that it costs one cpu's
 * arenas per hotplug cycle instead of every arena in the slab.
 *
 * Safe to call from any cpu, which is the whole point -- @cpu itself may
 * already be gone.
 */
static inline
void urcu_slab_drain_cpu(struct urcu_slab *s, int cpu)
{
	int cl;

	if (!urcu_slab_enabled(s) || cpu < 0 || cpu >= s->ncpu)
		return;
	for (cl = 0; cl < s->nclass; cl++)
		urcu_slab_demote_flag(&s->arenas[cl * s->ncpu + cpu]);
	if (urcu_slab_demote_fence()) {
		/* see urcu_slab_demote_all(): strand rather than corrupt */
		return;
	}
	for (cl = 0; cl < s->nclass; cl++)
		urcu_slab_drain_local(&s->arenas[cl * s->ncpu + cpu]);
}

static inline
void *urcu_slab_alloc(struct urcu_slab *s, int cl)
{
	int cpu = urcu_slab_cpu();
	struct urcu_slab_arena *a;
	struct cds_lfs_node *node;
	void *p;

	if (cpu < 0 || cpu >= s->ncpu)
		cpu = 0;
	a = &s->arenas[cl * s->ncpu + cpu];
#ifdef URCU_SLAB_RSEQ
	if (caa_likely(urcu_slab_rseq_ready() &&
			uatomic_load(&a->rseq_ok, CMM_RELAXED))) {
		struct cds_lfs_head *chain;

		node = urcu_slab_local_pop(s, a, cpu);
		if (caa_likely(node != NULL)) {
			URCU_SLAB_STAT(s, reuse);
			URCU_SLAB_STAT(s, a_local);
			return urcu_slab_block(s, node);
		}
		/*
		 * Local dry: take the WHOLE atomic freelist in one xchg and
		 * install it, so the cross-cpu frees accumulated there cost one
		 * atomic per refill instead of one per block.
		 *
		 * Under the pop lock, even though pop_all is itself atomic:
		 * lfstack's synchronization matrix requires __cds_lfs_pop() to be
		 * serialized against pop_all as well as against other pops, and
		 * this arena's other consumers (the alloc slow path, and
		 * urcu_slab_take_floor()) take that lock.  A mutex only some
		 * consumers hold satisfies none of the matrix's three options, and
		 * the hole is the classic pop-side ABA -- a locked popper resuming
		 * with a stale successor that pop_all has since handed out.  One
		 * lock per refill (not per alloc) restores it.
		 */
		cds_lfs_pop_lock(&a->freelist);
		chain = __cds_lfs_pop_all(&a->freelist);
		cds_lfs_pop_unlock(&a->freelist);
		if (chain) {
			struct cds_lfs_node *first = &chain->node;
			struct cds_lfs_node *rest = first->next;

			if (!rest || !urcu_slab_local_install(a, rest, cpu)) {
				URCU_SLAB_STAT(s, reuse);
				URCU_SLAB_STAT(s, a_refill);
				return urcu_slab_block(s, first);
			}
			/*
			 * Raced, migrated or demoted mid-install: hand the WHOLE
			 * chain back, node by node.  urcu_slab_push() overwrites
			 * the node's next with the freelist head, so pushing only
			 * @first would sever @rest -- a permanent leak of every
			 * block accumulated since the last refill, on an
			 * interleaving as benign as one same-cpu free landing
			 * between our dry pop and the install.  The path is cold.
			 */
			while (first) {
				struct cds_lfs_node *next = first->next;

				cds_lfs_node_init(first);
				urcu_slab_push(&a->freelist, first);
				first = next;
			}
		}
		goto carve;
	}
#endif
	cds_lfs_pop_lock(&a->freelist);
	node = __cds_lfs_pop(&a->freelist);
	if (node) {				/* reuse before carve -- caps the footprint */
		cds_lfs_pop_unlock(&a->freelist);
		URCU_SLAB_STAT(s, reuse);
		URCU_SLAB_STAT(s, a_slow);
		return urcu_slab_block(s, node);
	}
	cds_lfs_pop_unlock(&a->freelist);
#ifdef URCU_SLAB_RSEQ
carve:
#endif
	cds_lfs_pop_lock(&a->freelist);
	/* carve from the active superblock (cold), under the pop lock so two
	 * would-be consumers never carve the same bump concurrently */
	if (!a->sb || a->sb->bump + a->obj > URCU_SLAB_RANGE) {
		struct urcu_slab_sb *nsb;

		/*
		 * Budget spent: refuse rather than map more.  The caller falls
		 * back to posix_memalign, so the transaction still proceeds --
		 * it just stops making the permanent footprint bigger.
		 */
		if (!urcu_slab_reserve_sb(s)) {
			cds_lfs_pop_unlock(&a->freelist);
			return NULL;
		}
		nsb = urcu_slab_sb_new(s, a);
		if (!nsb)
			uatomic_dec(&s->nr_sb_total);
		if (!nsb) {
			cds_lfs_pop_unlock(&a->freelist);
			return NULL;
		}
		a->sb = nsb;
	}
	p = (char *) a->sb + a->sb->bump;
	a->sb->bump += a->obj;
	cds_lfs_pop_unlock(&a->freelist);
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
	struct urcu_slab_arena *a = sb->owner;		/* ORIGIN arena */
	struct cds_lfs_node *node = urcu_slab_node(a->slab, block);

	cds_lfs_node_init(node);
#ifdef URCU_SLAB_RSEQ
	/*
	 * Freeing on the origin arena's own cpu is the common case (the block
	 * was allocated here, and with per-cpu reclaim workers it comes back
	 * here) -- take the atomic-free local path.  A cross-cpu free still
	 * lands in the ORIGIN arena, just on its atomic list, so the contract
	 * holds and the two regimes never share a word.
	 */
	if (caa_likely(urcu_slab_rseq_ready() &&
			uatomic_load(&a->rseq_ok, CMM_RELAXED))) {
		int cpu = rseq_current_cpu_raw();

		if (cpu == a->cpu &&
				!urcu_slab_local_push(a->slab, a, node, cpu)) {
			URCU_SLAB_STAT(a->slab, f_local);
			return;
		}
	}
#endif
	URCU_SLAB_STAT(a->slab, f_slow);
	urcu_slab_push(&a->freelist, node);
}


/* Take one block for use as a floor: freelist first, else carve. */
static inline
struct cds_lfs_node *urcu_slab_take_floor(struct urcu_slab *s,
		struct urcu_slab_arena *a)
{
	struct cds_lfs_node *n;

	cds_lfs_pop_lock(&a->freelist);
	n = __cds_lfs_pop(&a->freelist);
	if (!n) {
		/*
		 * Exempt from the budget REFUSAL, but not from its ledger: the
		 * close needs exactly one block to make progress, and refusing
		 * it would strand the whole pending batch forever -- the
		 * freelist would stay empty, so every later close would fail the
		 * same way.  So map unconditionally, and still account it: a
		 * superblock is never unmapped, and nothing reconciles the count
		 * later, so skipping the add would make every floor carve widen
		 * the real footprint past URCU_TXN_SLAB_MAX_MB invisibly.  The
		 * overrun is one superblock per floor carve -- not, as this
		 * comment used to claim, one per arena.
		 */
		if (!a->sb || a->sb->bump + a->obj > URCU_SLAB_RANGE) {
			struct urcu_slab_sb *nsb;

			uatomic_add(&s->nr_sb_total, 1);
			nsb = urcu_slab_sb_new(s, a);
			if (!nsb) {
				uatomic_dec(&s->nr_sb_total);
				cds_lfs_pop_unlock(&a->freelist);
				return NULL;
			}
			a->sb = nsb;
		}
		n = urcu_slab_node(s, (char *) a->sb + a->sb->bump);
		a->sb->bump += a->obj;
		URCU_SLAB_STAT(s, carve);
	}
	cds_lfs_pop_unlock(&a->freelist);
	return n;
}

/*
 * Splice a closed batch onto its arena's freelist.  Runs one grace period after
 * the close -- and because the batch was closed BEFORE this callback was armed,
 * one grace period is all it needs.
 */
static inline
void urcu_slab_splice_cb(struct rcu_head *rh)
{
	struct urcu_slab_sb *sb = (struct urcu_slab_sb *)
			((uintptr_t) rh & ~(uintptr_t) URCU_SLAB_RANGE_MASK);
	struct urcu_slab_arena *a = sb->owner;
	struct urcu_slab *s = a->slab;
	struct cds_lfs_node *tail = (struct cds_lfs_node *) rh;	/* rh sits AT link_off */
	void *tail_block = urcu_slab_block(s, tail);
	struct cds_lfs_node *chain = urcu_slab_batch_of(s, tail_block)->head;
	struct cds_lfs_head *old;

	do {
		old = uatomic_load(&a->freelist.head, CMM_RELAXED);
		tail->next = &old->node;
	} while (uatomic_cmpxchg_mo(&a->freelist.head, old,
			caa_container_of(chain, struct cds_lfs_head, node),
			CMM_SEQ_CST, CMM_SEQ_CST) != old);
}

/*
 * A close is the one operation on an arena that is not self-serializing, so it
 * runs under @boot, taken with trylock: a closer that finds the arena already
 * being closed lets that closer do the work rather than queueing behind it.
 * Nothing here is on the free fast path -- the threshold trips once per
 * batch_max blocks, the fallback closer once per grace period.
 */
static inline
int urcu_slab_close_trylock(struct urcu_slab_arena *a)
{
	return pthread_mutex_trylock(&a->boot) == 0;
}

static inline
void urcu_slab_close_unlock(struct urcu_slab_arena *a)
{
	(void) pthread_mutex_unlock(&a->boot);
}

#ifdef URCU_SLAB_RSEQ
/*
 * Close ->local_pending and arm its splice.  Caller holds the close lock.
 *
 * MUST run on the arena's own cpu.  The critical section's cpu check does NOT
 * enforce that by itself: it compares @cpu against the cpu the section runs on,
 * so a caller passing its own current cpu makes it a tautology and the section
 * commits a plain store into another cpu's list -- against which nothing
 * arbitrates, because rseq atomicity is scoped to one cpu.  The owner's
 * concurrent push and this close would then both pass their compares and both
 * commit: either the pushed block is dropped from the list (a GP-owed block
 * that is never spliced) or the batch we just recorded stays reachable through
 * ->local_pending and gets spliced a SECOND time, which puts one block on the
 * freelist twice.  Hence the explicit comparison below.
 *
 * A caller on the wrong cpu leaves a request instead; the next deferred free on
 * the owning cpu picks it up (see urcu_slab_free_pending()).
 */
static inline
int urcu_slab_close_local(struct urcu_slab *s, struct urcu_slab_arena *a, int cpu)
{
	struct cds_lfs_node *head, *tail, *new_floor;

	if (!uatomic_load(&a->local_floor, CMM_RELAXED) ||
			!uatomic_load(&s->call_rcu_fn, CMM_RELAXED))
		return 0;
	if (cpu != a->cpu) {
		/*
		 * Racy off-cpu reads, deliberately: they only decide whether to
		 * leave a request, and an over-request costs one extra close
		 * attempt.
		 */
		if (uatomic_load(&a->local_pending, CMM_RELAXED) !=
				uatomic_load(&a->local_floor, CMM_RELAXED))
			uatomic_store(&a->close_local_req, 1, CMM_RELAXED);
		return 0;
	}
	head = RSEQ_READ_ONCE(a->local_pending);
	if (head == a->local_floor)
		return 0;				/* nothing pushed */
	new_floor = urcu_slab_take_floor(s, a);
	if (!new_floor)
		return 0;
	new_floor->next = NULL;
	if (rseq_load_cbne_load_cbne_store__ptr(RSEQ_MO_RELAXED,
			RSEQ_PERCPU_CPU_ID, (intptr_t *) &a->local_pending,
			(intptr_t) head, (intptr_t *) &a->rseq_ok, (intptr_t) 1,
			(intptr_t) new_floor, cpu)) {
		/* raced, migrated or demoted: give the floor back untouched */
		cds_lfs_node_init(new_floor);
		urcu_slab_push(&a->freelist, new_floor);
		return 0;
	}
	tail = a->local_floor;			/* recalled, not discovered */
	a->local_floor = new_floor;
	urcu_slab_batch_of(s, urcu_slab_block(s, tail))->head = head;
	s->call_rcu_fn((struct rcu_head *) tail, urcu_slab_splice_cb);
	return 1;
}
#endif

/*
 * Close the open batch and arm its splice.  One xchg takes the chain and
 * installs the next floor; the outgoing floor becomes the batch's tail and
 * carries its rcu_head.  Returns 1 if a batch was closed.
 *
 * Caller holds the close lock.  Without it two closers both recall @floor as
 * the batch tail -- it is read and written plainly here, and the batch_max
 * trigger is deliberately imprecise, so two freeing threads (or a freeing
 * thread and the fallback closer) reach this concurrently.  Both would then
 * queue the SAME rcu_head into call_rcu, corrupting the callback queue, and the
 * loser's chain would be recorded with a tail that is not its own: one closer's
 * whole chain orphaned, and the freelist truncated where the splice lands.
 */
static inline
int urcu_slab_close_and_arm(struct urcu_slab *s, struct urcu_slab_arena *a)
{
	struct cds_lfs_head *chain;
	struct cds_lfs_node *new_floor, *tail;

	if (!a->floor || !uatomic_load(&s->call_rcu_fn, CMM_RELAXED))
		return 0;
	if (uatomic_load(&a->pending.head, CMM_RELAXED) ==
			caa_container_of(a->floor, struct cds_lfs_head, node))
		return 0;				/* nothing pushed */
	new_floor = urcu_slab_take_floor(s, a);
	if (!new_floor)
		return 0;				/* OOM: retry later */
	new_floor->next = NULL;
	chain = uatomic_xchg_mo(&a->pending.head,
			caa_container_of(new_floor, struct cds_lfs_head, node),
			CMM_SEQ_CST);
	tail = a->floor;			/* recalled, not discovered */
	a->floor = new_floor;
	uatomic_store(&a->nr_pending, 0, CMM_RELAXED);

	urcu_slab_batch_of(s, urcu_slab_block(s, tail))->head = &chain->node;
	s->call_rcu_fn((struct rcu_head *) tail, urcu_slab_splice_cb);
	return 1;
}

/*
 * Close both of @a's open batches -- the rseq-local one (own cpu only) and the
 * atomic one -- under the arena's close lock.  @cpu is the CALLER's cpu, or -1
 * when it has none to offer.
 *
 * Returns 1 if a batch was closed.  A failed trylock returns 0 even though
 * another closer is running: it may have sampled the pending head before our
 * push, so our block is in the batch it did NOT take, and the caller should arm
 * the fallback closer.
 */
static inline
int urcu_slab_close_arena(struct urcu_slab *s, struct urcu_slab_arena *a, int cpu)
{
	int closed;

	if (!urcu_slab_close_trylock(a))
		return 0;
#ifdef URCU_SLAB_RSEQ
	closed = urcu_slab_close_local(s, a, cpu);
#else
	(void) cpu;
	closed = 0;
#endif
	closed |= urcu_slab_close_and_arm(s, a);
	urcu_slab_close_unlock(a);
	return closed;
}

/*
 * Fallback closer: a trickle of frees may never reach batch_max, so the first
 * push of a batch also arms this.  It closes whatever is open one grace period
 * later, and re-arms only while frees keep arriving.
 *
 * The re-arm condition covers the ATOMIC batch only.  The local batch cannot be
 * closed from here (this runs on the call_rcu worker, essentially never the
 * arena's own cpu), so re-arming for it would queue one callback per grace
 * period forever on a cpu that has gone quiet -- and keep taking grace periods
 * to do it.  urcu_slab_close_local() latches a request instead.  The residual
 * is stated at urcu_slab_free_pending().
 */
static inline
void urcu_slab_closer_cb(struct rcu_head *head)
{
	struct urcu_slab_arena *a = caa_container_of(head,
			struct urcu_slab_arena, close_head);
	struct urcu_slab *s = a->slab;
	int cpu = -1;

#ifdef URCU_SLAB_RSEQ
	if (urcu_slab_rseq_ready())
		cpu = rseq_current_cpu_raw();
#endif
	uatomic_store(&a->close_queued, 0, CMM_SEQ_CST);
	(void) urcu_slab_close_arena(s, a, cpu);
	/*
	 * Clear before re-checking, so a push racing us either sees
	 * close_queued == 0 and arms a closer itself, or landed early enough
	 * for the check below to catch it.
	 */
	if (a->floor &&
			uatomic_load(&a->pending.head, CMM_RELAXED) !=
				caa_container_of(a->floor, struct cds_lfs_head, node))
		urcu_slab_arm_closer(s, a);
}

/*
 * Free @block to its origin arena's PENDING stack rather than to the freelist,
 * BEFORE its grace period has elapsed.  The slab supplies the deferral: the
 * block becomes allocatable only when the batch it lands in is closed and then
 * spliced by urcu_slab_splice_cb(), one grace period after the close.  So the
 * caller owes the grace period at the SPLICE, not at this call -- which is the
 * whole difference from urcu_slab_free().
 *
 * That is also the whole point.  Deferring through call_rcu() costs a function
 * pointer stored per block and a per-block visit by the worker to invoke it,
 * over blocks gone cold during the grace period; here the destination is
 * implicit, so a batch is retired without the worker reading a single one of
 * them.
 *
 * OPTING IN.  An embedder passes urcu_txn_desc_commit() (or the sw engine's
 * equivalent) a deferral function of its own instead of call_rcu().  The engine
 * calls it with the block's rcu_head, which sits at @link_off, so:
 *
 *	static void my_defer(struct rcu_head *rh, void (*fn)(struct rcu_head *))
 *	{
 *		struct urcu_txn_desc *t = caa_container_of(rh,
 *				struct urcu_txn_desc, rcu_head);
 *
 *		if (t->slab)
 *			urcu_slab_free_pending(t, call_rcu);
 *		else
 *			call_rcu(rh, fn);	// exact-malloc block: stock path
 *	}
 *
 * The @call_rcu_fn argument is what the slab uses to schedule its OWN closes
 * and splices; it is never called on @block, and @fn is not called at all on
 * the slab arm.  A deferral function that instead hands the block straight to
 * urcu_txn_free() (hence urcu_slab_free()) makes it allocatable AT COMMIT TIME,
 * a full grace period early: the next alloc reuses the descriptor and
 * overwrites offset 0 while a reader is still resolving a parked proxy through
 * it.  That is exactly the use-after-free the link_off != 0 design exists to
 * prevent, so do not do it.
 *
 * There is no embedder-driven drain, and no urcu_slab_drain(): closes and
 * splices arm themselves.
 *
 * PRECONDITIONS
 *
 *  - All free_pending callers of one slab must pass the same @call_rcu_fn.  A
 *    grace period is flavor-scoped; see urcu_slab::call_rcu_fn.
 *  - The bytes [link_off, batch_off + 8) of @block must be dead to readers from
 *    this call until the splice; see struct urcu_slab_batch.
 *
 * MIXING WITH urcu_slab_free().  Allowed, per block: what matters is that a
 * block's path matches its grace-period state -- urcu_slab_free() only after
 * the caller's grace period, urcu_slab_free_pending() before it.  (The two
 * lists are disjoint, and a block on @pending was pushed before its batch's
 * close whatever the other list is doing, so there is no per-arena rule.)
 *
 * RESIDUAL, rseq builds only: the local batch is closed by its own cpu, so a
 * cpu that stops issuing deferred frees leaves up to one open batch pending
 * until its next deferred free -- or until urcu_slab_drain_cpu(), which is what
 * an embedder that needs reclaim on quiesce should call.
 */
static inline
void urcu_slab_free_pending(void *block,
		void (*call_rcu_fn)(struct rcu_head *, void (*)(struct rcu_head *)))
{
	unsigned long n_pending;
	struct urcu_slab_sb *sb = (struct urcu_slab_sb *)
			((uintptr_t) block & ~(uintptr_t) URCU_SLAB_RANGE_MASK);
	struct urcu_slab_arena *a = sb->owner;		/* ORIGIN arena */
	struct cds_lfs_node *node = urcu_slab_node(a->slab, block);
	int cpu = -1;

	cds_lfs_node_init(node);
	if (caa_unlikely(!uatomic_load(&a->floor, CMM_RELAXED))) {
		/*
		 * The floor must be the FIRST node ever pushed or it will not
		 * be at the bottom, so establish it under the mutex and install
		 * it as the stack contents directly.  Once per arena, ever: a
		 * pusher only reaches here while @floor is NULL, and no one can
		 * have pushed before the winner publishes it.
		 */
		pthread_mutex_lock(&a->boot);
		if (!a->floor) {
			node->next = NULL;
			uatomic_store(&a->pending.head,
				caa_container_of(node, struct cds_lfs_head, node),
				CMM_RELEASE);
			uatomic_store(&a->floor, node, CMM_RELEASE);
			pthread_mutex_unlock(&a->boot);
			return;
		}
		pthread_mutex_unlock(&a->boot);
	}
	uatomic_store(&a->slab->call_rcu_fn, call_rcu_fn, CMM_RELAXED);
#ifdef URCU_SLAB_RSEQ
	/*
	 * Same-cpu deferred free: one rseq commit, no atomic.  A cross-cpu free
	 * still goes to ->pending, so the ORIGIN-arena contract holds and the
	 * two atomicity regimes stay on separate lists.
	 */
	if (caa_likely(urcu_slab_rseq_ready() &&
			uatomic_load(&a->rseq_ok, CMM_RELAXED))) {
		cpu = rseq_current_cpu_raw();

		if (cpu == a->cpu) {
			if (caa_unlikely(!a->local_floor)) {
				struct cds_lfs_node *f =
					urcu_slab_take_floor(a->slab, a);

				if (f) {
					f->next = NULL;
					if (!rseq_load_cbne_load_cbne_store__ptr(
							RSEQ_MO_RELAXED,
							RSEQ_PERCPU_CPU_ID,
							(intptr_t *) &a->local_pending,
							(intptr_t) NULL,
							(intptr_t *) &a->rseq_ok,
							(intptr_t) 1,
							(intptr_t) f, cpu)) {
						a->local_floor = f;
					} else {
						cds_lfs_node_init(f);
						urcu_slab_push(&a->freelist, f);
					}
				}
			}
			if (a->local_floor) {
				for (;;) {
					struct cds_lfs_node *h =
						RSEQ_READ_ONCE(a->local_pending);
					int ret;

					node->next = h;
					ret = rseq_load_cbne_load_cbne_store__ptr(
						RSEQ_MO_RELAXED, RSEQ_PERCPU_CPU_ID,
						(intptr_t *) &a->local_pending,
						(intptr_t) h,
						(intptr_t *) &a->rseq_ok,
						(intptr_t) 1,
						(intptr_t) node, cpu);
					if (rseq_likely(!ret))
						goto armed;
					if (ret < 0 ||
						!uatomic_load(&a->rseq_ok, CMM_RELAXED))
						break;		/* migrated/demoted */
				}
			}
		}
	}
#endif
	URCU_SLAB_STAT(a->slab, p_slow);
	urcu_slab_push(&a->pending, node);
#ifdef URCU_SLAB_RSEQ
	goto counted;
armed:
	URCU_SLAB_STAT(a->slab, p_local);
counted:
#endif
	/*
	 * One callback per BATCH, never per block.
	 *
	 * The count is deliberately IMPRECISE -- it only picks a close point --
	 * but imprecise is not the same as unsynchronized.  It is written by
	 * more than one thread: a block returns to its ORIGIN arena, so a free
	 * running on another cpu bumps this arena's counter, and the fallback
	 * closer resets it from the call_rcu worker.  A plain ++ against those
	 * uatomic_store()s is a data race in the memory model (and one TSan
	 * reports) whatever the hardware does with it.
	 *
	 * So keep every access atomic, but relaxed, and NOT an atomic RMW: a
	 * load-add-store loses concurrent increments exactly as the plain ++
	 * did -- which is the intended, documented behaviour -- while costing
	 * the same two plain instructions.  A locked RMW would buy precision
	 * this counter has no use for, on the very path the rseq work exists to
	 * keep free of locked instructions.
	 *
	 * A lost increment closes a little late; a doubled close finds nothing
	 * pending and returns.  Neither can stall reclaim: every push that does
	 * not close a batch arms the fallback closer below, so a batch is closed
	 * at least once per grace period no matter what this says.
	 */
	n_pending = uatomic_load(&a->nr_pending, CMM_RELAXED) + 1;
	uatomic_store(&a->nr_pending, n_pending, CMM_RELAXED);
	if (n_pending >= a->slab->batch_max ||
			caa_unlikely(uatomic_load(&a->close_local_req,
					CMM_RELAXED))) {
		/*
		 * Reset HERE, not inside close_and_arm().  There are now two
		 * lists, and whichever one is empty makes its close return
		 * early -- if the reset lives there, an empty ->pending leaves
		 * the counter latched at the threshold and every subsequent
		 * free arms a callback, i.e. one per descriptor: precisely the
		 * per-node cost the batching exists to remove.
		 *
		 * Clear the request BEFORE closing, so one arriving while we
		 * close is kept rather than swallowed.
		 */
		uatomic_store(&a->nr_pending, 0, CMM_RELAXED);
		uatomic_store(&a->close_local_req, 0, CMM_RELAXED);
		if (urcu_slab_close_arena(a->slab, a, cpu))
			return;
		/*
		 * Nothing closed: the arena was already being closed by someone
		 * who may have sampled the head before our push, or take_floor
		 * came up empty.  Fall through and arm the fallback closer
		 * rather than wait for the next batch_max blocks.
		 */
	}
	urcu_slab_arm_closer(a->slab, a);
}

#ifdef __cplusplus
}
#endif

#endif /* _URCU_RCU_TXN_SLAB_H */
