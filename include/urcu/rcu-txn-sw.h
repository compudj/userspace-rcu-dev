// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

#ifndef _URCU_RCU_TXN_SW_H
#define _URCU_RCU_TXN_SW_H

/*
 * Single-updater transaction: atomically switch a *set* of pointers from an "old" value
 * to a "new" value with a single store, as observed by concurrent RCU
 * readers.
 *
 * Motivation
 * ----------
 * A structural RCU mutation often needs to re-point many slots (e.g. the
 * back-pointers of a set of live nodes being re-parented) such that a
 * reader never observes a partially-updated set.  Re-pointing the slots
 * one by one exposes intermediate states: a reader walking several of
 * them sees a mix of old and new targets.  The usual remedy is a
 * grace-period drain around the update, which is costly.
 *
 * The latch flip removes the intermediate states by one level of
 * indirection.  Each slot in the set is made to hold a tagged pointer to
 * a small "proxy" latch instead of the target directly.  A proxy holds
 * {old_ptr, new_ptr} and a pointer to a shared "flip group" carrying a
 * boolean selector word.  Resolving a proxy returns old_ptr while the
 * selector is 0 and new_ptr once it is 1.  Because every proxy in a group
 * reads the *same* selector, a single store flipping that selector
 * switches the whole set atomically: at any instant every proxy resolves
 * consistently to the old or the new target.
 *
 * The embedding data structure is responsible for:
 *   - tagging a proxy pointer so its readers recognise it (e.g. a spare
 *     low/high bit, or a reserved type code in an already-tagged pointer)
 *     and routing proxy resolution through urcu_txn_sw_proxy_get();
 *   - allocating proxies and the group (often a single backing block with
 *     one rcu_head), with whatever alignment its tagging scheme needs;
 *   - reclaiming them with call_rcu() after they are unpublished (every
 *     slot rewritten from the tagged proxy to the resolved target).
 *
 * Lifecycle (writer)
 * ------------------
 *   1. Build the new structure invisibly.
 *   2. urcu_txn_sw_group_init(group); for each slot, init a proxy with its
 *      {old, new} target and point the slot at the tagged proxy.  The
 *      selector is 0, so this is transparent to readers (they still
 *      resolve to old).
 *   3. urcu_txn_sw_group_commit(group): one release store, 0 -> 1.  Every proxy
 *      now resolves to new, atomically.
 *   4. "Settle": rewrite each slot from the tagged proxy back to the
 *      direct new target (idempotent for readers, since the proxy already
 *      resolves to new), then call_rcu() the proxies and group.
 *
 * Ordering / monotonicity
 * ------------------------
 * The selector is written exactly once (0 -> 1) and never back, so a
 * reader resolving several proxies over time observes a monotone
 * old...old,new...new sequence -- never new-then-old.  Embedders can rely
 * on this together with a fixed read order to avoid a per-traversal
 * snapshot (e.g. publishing the back edges of a re-parent through the
 * latch *before* the forward edge, so a reader -- which descends before it
 * walks back up -- can only ever progress old->new, never regress).
 *
 * Two layers
 * ----------
 * This header provides two layers:
 *
 *   1. The low-level flip group / proxy primitive above (urcu_txn_sw_group,
 *      urcu_txn_sw_proxy, urcu_txn_sw_proxy_get, urcu_txn_sw_commit).  The embedder
 *      allocates and reclaims the proxies, tags them, and routes resolution --
 *      as in the four-step lifecycle described above.
 *
 *   2. A growable multi-edge transaction, urcu_txn_sw_txn, built on that
 *      primitive (defined lower in this file).  It owns proxy allocation,
 *      installation, settle and reclaim, so an embedder records edges and
 *      commits a whole set atomically instead of hand-managing proxies.  Its
 *      lifecycle is a state machine:
 *
 *        init -> PREPARE --record--> ... --commit--> committed
 *
 *      record() appends an edge {slot, old, new} into the (realloc-grown)
 *      record array but installs nothing; the record set is FROZEN once commit()
 *      parks the proxies, matching the concurrent engine's contract (no edge may
 *      be added once a proxy is parked), so an embedder written against this
 *      transaction can migrate to <urcu/rcu-mcas.h> mechanically.
 *      commit() parks every recorded proxy, flips the group and settles to new,
 *      then OWNS reclaim: it defers the txn through call_rcu() when it parked
 *      proxies, or frees it at once on the single-edge / empty / OOM paths.  Each
 *      recorded edge carries its own tag (the bits OR'd into that slot's parked
 *      proxy value), so one transaction can mix slots with different tags; the
 *      embedder only checks commit()'s status.  See the urcu_txn_sw_txn block.
 *
 * RCU flavor
 * ----------
 * The transaction layer reclaims its parked proxies with call_rcu(), so this
 * header must be included AFTER an RCU flavor header (e.g. <urcu-qsbr.h>) that
 * maps call_rcu() to that flavor.  (The low-level flip group / proxy primitive
 * above has no such dependency -- an embedder using only it drives its own
 * reclaim.)
 */

#include <stdbool.h>
#include <stddef.h>			/* offsetof */
#include <stdlib.h>
#include <string.h>
#include <urcu/assert.h>
#include <urcu/compiler.h>
#include <urcu/uatomic.h>
#include <urcu/call-rcu.h>		/* struct rcu_head + call_rcu (commit reclaim) */
#include <urcu/rcu-txn-status.h>	/* enum urcu_txn_status */
#include <urcu/rcu-txn-slab.h>		/* shared per-CPU size-classed descriptor slab */

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Shared selector for a flip group.  selector == 0 -> proxies resolve to
 * old_ptr; selector == 1 -> new_ptr.  Written once (0 -> 1) with release
 * semantics by urcu_txn_sw_group_commit(); read with acquire by
 * urcu_txn_sw_proxy_get().
 */
struct urcu_txn_sw_group {
	unsigned long selector;
};

/*
 * ptr[0] is the old target, ptr[1] the new one: the selector (0 -> old,
 * 1 -> new) indexes this array directly in urcu_txn_sw_proxy_get().
 */
struct urcu_txn_sw_proxy {
	void *ptr[2];
	struct urcu_txn_sw_group *group;
};

static inline
void urcu_txn_sw_group_init(struct urcu_txn_sw_group *group)
{
	group->selector = 0;
}

static inline
void urcu_txn_sw_proxy_init(struct urcu_txn_sw_proxy *proxy,
		struct urcu_txn_sw_group *group, void *old_ptr, void *new_ptr)
{
	proxy->ptr[0] = old_ptr;
	proxy->ptr[1] = new_ptr;
	proxy->group = group;
}

/*
 * Resolve a proxy to its current target.
 *
 * @proxy must have been obtained by dereferencing (rcu_dereference) the
 * slot that holds the tagged proxy pointer, so the dependency chain makes
 * the proxy's immutable fields (old_ptr, new_ptr, group) visible.  The
 * selector is the only mutable field: load it with acquire so the new
 * target's contents -- published before urcu_txn_sw_group_commit()'s release store
 * -- are visible whenever selector == 1 is observed.
 *
 * The selector indexes proxy->ptr[] directly, so resolution is a pure data
 * dependency rather than a conditional branch.
 */
static inline
void *urcu_txn_sw_proxy_get(const struct urcu_txn_sw_proxy *proxy)
{
	return proxy->ptr[uatomic_load(&proxy->group->selector, CMM_ACQUIRE)];
}

/*
 * Commit the flip: switch every proxy in @group from old to new with a
 * single release store.  Must be called after the new targets are fully
 * built (the release pairs with urcu_txn_sw_proxy_get()'s acquire).
 */
static inline
void urcu_txn_sw_group_commit(struct urcu_txn_sw_group *group)
{
	uatomic_store(&group->selector, 1, CMM_RELEASE);
}

/*
 * Multi-edge flip transaction (urcu_txn_sw_txn)
 * ===========================================
 *
 * A growable transaction over a set of slots, built on the flip group above.
 * The handle is a small on-stack object the embedder declares and inits with
 * urcu_txn_sw_init(); nothing is allocated until edges are recorded, so the
 * init step has no failure mode.
 *
 *   init --> PREPARE --record--> PREPARE --commit--> committed (handle done)
 *              |                            |
 *              | record()/reserve() OOM     | commit owns reclaim:
 *              v (sticky URCU_TXN_SW_OOM)  | call_rcu group block (parked)
 *        commit reports MEMORY_ERROR         | or immediate record-array free
 *        and frees the record array          v (no proxy: empty/single)
 *                                          freed
 *
 * record() appends a latch descriptor {slot, old, new} into the record array
 * but installs NOTHING -- no proxy address is live yet, so the array grows by
 * realloc.  The record set is FROZEN once proxies are installed (which commit()
 * does internally), so record() must precede commit().  This is the same
 * frozen-set contract as the concurrent engine (<urcu/rcu-mcas.h>), so
 * a single-writer embedder can later migrate to concurrent writers without
 * restructuring its mutations.  Two more contracts shared with that engine:
 * records must target PAIRWISE-DISTINCT slots -- unlike the concurrent
 * front-end, record() performs no same-slot reconcile, so a duplicate would
 * park two proxies on one slot and settle both in record order, silently
 * last-wins (a debug build asserts against it at install) -- and nothing
 * observes buffered writes (there are no transactional loads at all).
 *
 * commit() publishes the whole set atomically: it parks every recorded latch's
 * tagged proxy (readers still resolve to old; selector == 0), flips the group
 * with one release store (every proxy resolves to new at once), then settles
 * each slot to its direct new value.  commit() OWNS reclaim:
 *   - nr >= 2 (proxies parked): a reader may hold a proxy, whose old/new targets
 *     and selector live in the record array and the group block; both are
 *     deferred through one call_rcu(urcu_txn_sw_free_rcu) and reclaimed
 *     together after a grace period.
 *   - nr <= 1 / sticky OOM (no proxy ever published): the record array is freed
 *     at once and no group block is ever allocated.
 * It returns enum urcu_txn_status: OK on commit, or MEMORY_ERROR if a
 * reserve()/record() (or the lazy group block) allocation failed -- never ABORT
 * (a single updater has no contention).  An embedder checks only commit()'s
 * status: the stack handle needs no destroy, and there is no create()==NULL
 * checkpoint.  Fresh nodes are the embedder's; the txn never tracks them.
 *
 * OOM is sticky: a reserve()/record() that cannot allocate latches the handle
 * into URCU_TXN_SW_OOM and a later commit() reports MEMORY_ERROR (and frees
 * the record array), so an embedder may ignore the bool returns of
 * reserve()/record() and test only commit() -- matching the concurrent
 * front-end's contract.
 *
 * Each recorded edge carries its own tag (urcu_txn_sw_record's @tag): the bits
 * OR'd into that slot's parked proxy value so its readers recognise the proxy
 * (e.g. a reserved type code).  The latch array is allocated 16-byte aligned
 * (posix_memalign), so each latch -- hence each tagged proxy -- has its low 4
 * bits free for the embedder's tag.
 */

enum urcu_txn_sw_state {
	URCU_TXN_SW_PREPARE = 0,
	URCU_TXN_SW_INSTALLED,	/* internal: set once proxies are parked */
	URCU_TXN_SW_OOM,		/* sticky: commit -> MEMORY_ERROR */
};

struct urcu_txn_sw_latch {
	struct urcu_txn_sw_proxy proxy;	/* ptr[0]=old, ptr[1]=new, group */
	void **slot;			/* install / settle target */
	uintptr_t tag;			/* embedder tag bits OR'd into THIS slot's parked
					 * proxy value at install (see urcu_txn_sw_record).
					 * Per-record so heterogeneous slots -- e.g. a
					 * 0xF-tagged structural edge and a bit-0 list edge --
					 * can share one transaction. */
} __attribute__((aligned(16)));

/*
 * Transaction block (HEAP path): the single allocation that carries a committed
 * transaction across its grace period, mirroring the concurrent engine's struct
 * urcu_mcas (one block = header + inline records).  It holds the flip group every
 * parked proxy reads (&block->group, a plain pointer -- never tagged -- so it
 * needs no alignment of its own), the rcu_head that defers reclaim, and the
 * record array INLINE (the proxies themselves live there).  Allocated on the
 * first record()/reserve() from the shared per-CPU slab and grown in PREPARE (no
 * proxy is live yet, so it may move); ONE free -- record array and all --
 * reclaims it.  The 32-byte header keeps latches[] 16-byte aligned so each tagged
 * proxy has its low 4 bits free.  @cap is the physical capacity: a slab block's
 * cap is its class size, so cap <= URCU_TXN_SW_SLAB_MAXCAP on free identifies it.
 * The INLINE path (urcu_txn_sw_init_inline, nr <= 1) never allocates a block.
 */
struct urcu_txn_sw_block {
	struct urcu_txn_sw_group group;		/* selector; parked proxies read &block->group */
	struct rcu_head rcu_head;		/* deferred-free handle */
	unsigned int cap;			/* physical capacity; identifies the slab class on free */
	unsigned int _pad;			/* pad so latches[] stays 16-byte aligned */
	struct urcu_txn_sw_latch latches[];	/* INLINE record array (frozen at install) */
};

urcu_static_assert(!(offsetof(struct urcu_txn_sw_block, latches) % 16),
		"urcu_txn_sw_block.latches must be 16-byte aligned for proxy tagging",
		urcu_txn_sw_block_latches_aligned);

/*
 * Transaction handle: a small on-stack object with no allocation of its own.
 * The record array @latches grows in PREPARE -- safe because no proxy is
 * installed yet (no slot points into it) -- and is frozen once commit() parks
 * proxies.  @block is the lazily-allocated persistent group block (NULL until
 * proxies are parked); commit() hands @latches to it so a held proxy and its
 * selector are reclaimed together after a grace period.  The tag room that
 * matters is on the tagged proxies stored in slots; those live in @latches,
 * allocated 16-byte aligned (posix_memalign) so each latch -- hence each
 * tagged proxy -- keeps its low 4 bits free, letting a low-4-bit
 * pointer-tagging embedder (e.g. the fractal trie) route its slots through
 * this engine.
 */
struct urcu_txn_sw_txn {
	enum urcu_txn_sw_state state;
	struct urcu_txn_sw_latch *latches;	/* record array (realloc-grown, or caller-owned if @latches_inline) */
	struct urcu_txn_sw_block *block;	/* heap path: single allocation (latches inline); NULL until first record */
	unsigned int nr;
	unsigned int cap;
	bool latches_inline;			/* @latches is caller storage: never realloc'd, never freed */
};

#define URCU_TXN_SW_CAP	8	/* initial record-array capacity */

/*
 * Initialize an on-stack transaction handle.  No allocation, so this cannot
 * fail; the first record()/reserve() is the first OOM checkpoint, and it is
 * sticky (commit reports MEMORY_ERROR).  Each edge carries its own tag (passed
 * to urcu_txn_sw_record), so the handle holds no per-txn tag hook.
 */
static inline
void urcu_txn_sw_init(struct urcu_txn_sw_txn *t)
{
	t->state = URCU_TXN_SW_PREPARE;
	t->latches = NULL;
	t->block = NULL;
	t->nr = 0;
	t->cap = 0;
	t->latches_inline = false;
}

/*
 * Initialize a transaction whose record array is CALLER-PROVIDED storage @buf,
 * holding @cap latches (e.g. a 16-byte-aligned on-stack array -- struct
 * urcu_txn_sw_latch is aligned(16), so an array of it is, keeping each tagged
 * proxy's low 4 bits free).  No allocation, so it cannot fail; record() never
 * realloc-grows the inline array (overflow past @cap is an embedder sizing bug),
 * and commit() never frees it.  Use ONLY where the transaction need not outlive
 * the call: a lone-edge (or empty) commit parks no proxy and owes no grace
 * period, so nothing references the txn -- or its inline storage -- once commit
 * returns.  A txn that would park proxies (nr >= 2) MUST own heap storage
 * (urcu_txn_sw_init + record/reserve), since the parked record array has to
 * survive until a grace period; the engine asserts an inline buffer never
 * reaches that path.  This is the public analog of the bounded on-stack flip
 * that the fractal trie uses for its single-pointer publishes.
 */
static inline
void urcu_txn_sw_init_inline(struct urcu_txn_sw_txn *t,
		struct urcu_txn_sw_latch *buf, unsigned int cap)
{
	t->state = URCU_TXN_SW_PREPARE;
	t->latches = buf;
	t->block = NULL;
	t->nr = 0;
	t->cap = cap;
	t->latches_inline = true;
}

#define urcu_txn_sw_blocksize(cap)	\
	(sizeof(struct urcu_txn_sw_block) + (size_t) (cap) * sizeof(struct urcu_txn_sw_latch))

/*
 * The heap-path transaction block is served from the shared per-CPU size-classed
 * slab in <urcu/rcu-txn-slab.h> (same machinery as the concurrent engine).
 * Latch-count classes {4,8,16,32,64,128} map to byte sizes; a request over the
 * top class is an exact, uncached posix_memalign.  URCU_TXN_NO_CACHE disables it.
 */
#define URCU_TXN_SW_SLAB_MAXCAP	128u

static const unsigned int urcu_txn_sw_slab_rc[] = { 4u, 8u, 16u, 32u, 64u, 128u };
#define URCU_TXN_SW_SLAB_NCLASS	\
	((int) (sizeof(urcu_txn_sw_slab_rc) / sizeof(urcu_txn_sw_slab_rc[0])))

static size_t urcu_txn_sw_slab_bytes[URCU_TXN_SW_SLAB_NCLASS];	/* blocksize per class, filled at init */
static struct urcu_slab urcu_txn_sw_slab;

/* Smallest latch-count class that fits @req latches, or -1 if over the top. */
static inline
int urcu_txn_sw_slab_class_of(unsigned int req)
{
	int i;

	for (i = 0; i < URCU_TXN_SW_SLAB_NCLASS; i++)
		if (req <= urcu_txn_sw_slab_rc[i])
			return i;
	return -1;
}

static __attribute__((constructor))
void urcu_txn_sw_slab_ctor(void)
{
	int i;

	for (i = 0; i < URCU_TXN_SW_SLAB_NCLASS; i++)
		urcu_txn_sw_slab_bytes[i] = urcu_txn_sw_blocksize(urcu_txn_sw_slab_rc[i]);
	urcu_slab_init(&urcu_txn_sw_slab, urcu_txn_sw_slab_bytes,
			URCU_TXN_SW_SLAB_NCLASS, "txn_sw");
}

/*
 * Allocate a transaction block for >= @cap latches, 16-byte aligned (so the
 * inline latches[] -- hence every tagged proxy -- keeps its low 4 bits free).
 * From the slab when the request fits a class (its physical cap becomes the
 * class size), else an exact posix_memalign.  The flip group is initialized
 * here.  Returns NULL on OOM.
 */
static inline
struct urcu_txn_sw_block *urcu_txn_sw__block_alloc(unsigned int cap)
{
	struct urcu_txn_sw_block *blk;
	int cl;

	if (urcu_slab_enabled(&urcu_txn_sw_slab) &&
			(cl = urcu_txn_sw_slab_class_of(cap)) >= 0) {
		blk = (struct urcu_txn_sw_block *)
				urcu_slab_alloc(&urcu_txn_sw_slab, cl);
		if (caa_unlikely(!blk))
			return NULL;
		cap = urcu_txn_sw_slab_rc[cl];		/* physical class cap */
	} else {
		void *p;

		if (posix_memalign(&p, 16, urcu_txn_sw_blocksize(cap)))
			return NULL;
		blk = (struct urcu_txn_sw_block *) p;
	}
	urcu_txn_sw_group_init(&blk->group);
	blk->cap = cap;
	return blk;
}

/* Free a block to its slab ORIGIN arena, or to malloc for an exact block. */
static inline
void urcu_txn_sw__block_free(struct urcu_txn_sw_block *blk)
{
	if (!blk)
		return;
	if (urcu_slab_enabled(&urcu_txn_sw_slab) && blk->cap <= URCU_TXN_SW_SLAB_MAXCAP)
		urcu_slab_free(blk);
	else
		free(blk);
}

/*
 * Pre-size the PREPARE record array to hold at least @cap latches.  Optional:
 * record() already realloc-grows the array on demand and fails cleanly on OOM,
 * which is the general (unbounded) model.  But an embedder whose edge count is
 * bounded by construction can reserve that bound once, up front, where failure
 * is clean -- then every subsequent record() appends without reallocating and
 * so cannot fail.  This trades the design's "fail-and-destroy replaces the count
 * pass" for a single up-front alloc, which is the right call for a bounded txn
 * whose records are interleaved through a build that does not otherwise thread
 * an OOM return.  Call after init, before any install; a handle already
 * buffering (an earlier reserve or record) grows to fit @cap, mirroring the
 * concurrent front-end's urcu_txn_reserve().  Returns
 * false on OOM; the failure is sticky (URCU_TXN_SW_OOM), so the caller may
 * ignore this return and let commit() report MEMORY_ERROR.  The one non-OOM,
 * NON-sticky false: asking a caller-storage (init_inline) handle for more than
 * its fixed buffer -- that is an embedder sizing bug, not a memory failure, and
 * caller storage can neither grow nor be adopted by the engine.
 */
static inline
bool urcu_txn_sw_reserve(struct urcu_txn_sw_txn *t, unsigned int cap)
{
	struct urcu_txn_sw_block *blk;

	if (caa_unlikely(t->state == URCU_TXN_SW_OOM))
		return false;			/* sticky: an earlier alloc failed */
	if (t->latches) {
		if (cap <= t->cap)
			return true;		/* already sized (heap block or inline buf) */
		if (t->latches_inline) {
			/* Fixed caller storage: a sizing bug, not an OOM. */
			urcu_assert_debug(!t->latches_inline);
			return false;
		}
		/*
		 * Already buffering: grow to fit @cap.  No proxy is live in
		 * PREPARE, so the array may move (same rationale as record()'s
		 * grow); copy the buffered records into a fresh aligned block.
		 */
		blk = urcu_txn_sw__block_alloc(cap);
		if (!blk) {
			t->state = URCU_TXN_SW_OOM;	/* sticky: commit reports it */
			return false;
		}
		memcpy(blk->latches, t->latches,
				(size_t) t->nr * sizeof(*blk->latches));
		urcu_txn_sw__block_free(t->block);
		t->block = blk;
		t->latches = blk->latches;
		t->cap = blk->cap;
		return true;
	}
	if (cap < URCU_TXN_SW_CAP)
		cap = URCU_TXN_SW_CAP;
	blk = urcu_txn_sw__block_alloc(cap);
	if (!blk) {
		t->state = URCU_TXN_SW_OOM;	/* sticky: commit reports it */
		return false;
	}
	t->block = blk;
	t->latches = blk->latches;		/* records live inline in the block */
	t->cap = blk->cap;			/* physical class capacity */
	return true;
}

/* Free the record array of a handle that never parked proxies (no grace period).
 * Caller-owned (inline) storage is never freed by the engine. */
static inline
void urcu_txn_sw__free_records(struct urcu_txn_sw_txn *t)
{
	if (!t->latches_inline)
		urcu_txn_sw__block_free(t->block);	/* one free; inline storage is caller-owned */
	t->block = NULL;
	t->latches = NULL;
}

/*
 * call_rcu callback: free a committed transaction's persistent group block and
 * the record array it carries, once the grace period has retired any proxy a
 * reader may still have held.
 */
static inline
void urcu_txn_sw_free_rcu(struct rcu_head *head)
{
	struct urcu_txn_sw_block *blk = caa_container_of(head,
			struct urcu_txn_sw_block, rcu_head);

	urcu_txn_sw__block_free(blk);		/* record array is inline -- one free */
}

/* Record a latch's {old, new, slot, tag}; its proxy->group is bound at install. */
static inline
void urcu_txn_sw_latch_set(struct urcu_txn_sw_latch *l,
		void **slot, void *old_ptr, void *new_ptr, uintptr_t tag)
{
	l->proxy.ptr[0] = old_ptr;
	l->proxy.ptr[1] = new_ptr;
	l->slot = slot;
	l->tag = tag;
}

/* Park latch @l's tagged proxy into its slot (readers resolve to old).  The
 * parked value is &l->proxy OR'd with the slot's per-record tag; the latch
 * array is 16-byte aligned, so &l->proxy keeps its low 4 bits free for it. */
static inline
void urcu_txn_sw_latch_install(struct urcu_txn_sw_txn *t, struct urcu_txn_sw_latch *l)
{
	l->proxy.group = &t->block->group;	/* bind to the now-allocated group */
	uatomic_store(l->slot,
			(void *) ((uintptr_t) &l->proxy | l->tag), CMM_RELEASE);
}

/*
 * Record one edge {*slot: old -> new} tagged with @tag (the bits OR'd into the
 * parked proxy value installed in *slot, so that slot's readers recognise the
 * proxy and route resolution -- e.g. the fractal trie's 0xF type code or a
 * list's bit-0).  PREPARE only -- the record set is frozen
 * once proxies are installed, so this must run before commit() (record() after a
 * commit/install is a usage error).  Records must target pairwise-distinct
 * slots: record() appends blindly -- no same-slot reconcile -- so recording one
 * slot twice parks two proxies on it and settles both in record order,
 * silently last-wins (install asserts against it in a debug build).  Returns
 * false on OOM (the only failure);
 * the failure is sticky (URCU_TXN_SW_OOM), so the caller may ignore this
 * return and let commit() report MEMORY_ERROR.  The record array realloc-grows
 * on demand and nothing is installed here (no proxy address is live until commit
 * parks them).
 */
static inline
bool urcu_txn_sw_record(struct urcu_txn_sw_txn *t, void **slot,
		void *old_ptr, void *new_ptr, uintptr_t tag)
{
	struct urcu_txn_sw_latch *l;

	if (caa_unlikely(t->state == URCU_TXN_SW_OOM))
		return false;			/* sticky: an earlier alloc failed */
	urcu_posix_assert(t->state == URCU_TXN_SW_PREPARE);
	if (t->nr == t->cap) {
		unsigned int newcap = t->cap ? t->cap * 2 : URCU_TXN_SW_CAP;

		/*
		 * Caller-owned (inline) storage is sized to the embedder's edge
		 * bound and must never grow -- realloc-ing it would move caller
		 * (e.g. on-stack) memory.  Overflow here is an embedder sizing bug.
		 */
		urcu_posix_assert(!t->latches_inline);
		/*
		 * No proxy address is live yet -> the array may move.  No
		 * aligned realloc exists, so allocate a fresh 16-byte-aligned
		 * block, copy the buffered records, and free the old one.
		 */
		struct urcu_txn_sw_block *nb = urcu_txn_sw__block_alloc(newcap);

		if (!nb) {
			t->state = URCU_TXN_SW_OOM;	/* sticky: commit reports it */
			return false;
		}
		memcpy(nb->latches, t->latches,
				(size_t) t->nr * sizeof(*nb->latches));
		urcu_txn_sw__block_free(t->block);	/* NULL on the first grow */
		t->block = nb;
		t->latches = nb->latches;
		t->cap = nb->cap;
	}
	l = &t->latches[t->nr++];
	urcu_txn_sw_latch_set(l, slot, old_ptr, new_ptr, tag);
	return true;
}

/*
 * PREPARE -> INSTALLED, parking every recorded latch's proxy into its slot
 * (readers still resolve to old; selector == 0).  Allocates the group block
 * lazily; on OOM it leaves the handle sticky (commit reports MEMORY_ERROR) and
 * parks nothing.  commit() drives this for nr >= 2; a white-box caller that
 * needs work BETWEEN install and the flip may call it directly.
 */
static inline
void urcu_txn_sw_install(struct urcu_txn_sw_txn *t)
{
	unsigned int i;

	if (caa_unlikely(!t->block)) {		/* white-box install with no record */
		t->block = urcu_txn_sw__block_alloc(URCU_TXN_SW_CAP);
		if (caa_unlikely(!t->block)) {
			t->state = URCU_TXN_SW_OOM;	/* sticky; nothing parked */
			return;
		}
		t->latches = t->block->latches;
		t->cap = t->block->cap;
	}
	/*
	 * Engine precondition: records target pairwise-distinct slots.  record()
	 * has no same-slot reconcile, so a duplicate parks two proxies on one
	 * slot and settles both in record order -- silently last-wins.  The
	 * record array is unsorted (record order is the embedder's), so pair-scan
	 * mirroring the concurrent engine's adjacent check after its sort.
	 * Debug-only: no cost under NDEBUG, and sw transactions are small.
	 */
	for (i = 1; i < t->nr; i++) {
		unsigned int j;

		for (j = 0; j < i; j++)
			urcu_assert_debug(t->latches[i].slot != t->latches[j].slot);
	}
	t->state = URCU_TXN_SW_INSTALLED;
	for (i = 0; i < t->nr; i++)
		urcu_txn_sw_latch_install(t, &t->latches[i]);
}

/*
 * Commit: flip the group (every proxy resolves to new atomically), then settle
 * each slot to its direct new value.  commit() OWNS reclaim and consumes the
 * handle's allocations -- the caller must re-init the handle to reuse it.
 * Returns enum urcu_txn_status: MEMORY_ERROR if a reserve()/record()/group-
 * block alloc had failed (sticky), otherwise OK.  A single updater has no
 * contention, so ABORT is never returned.
 *
 * @call_rcu_fn is the reclaim deferral: when proxies are parked (nr >= 2), the
 * group block (carrying the record array) is handed to call_rcu_fn(block,
 * urcu_txn_sw_free_rcu) so it is freed after a grace period retires any proxy a
 * reader may still hold.  It has the flavor call_rcu signature, so an embedder
 * passes its RCU flavor's call_rcu directly -- letting a FLAVOR-AGNOSTIC
 * embedder (one that selects its flavor at runtime, e.g. through a
 * rcu_flavor_struct vtable: flavor->update_call_rcu) drive reclaim without this
 * header binding a compile-time flavor.  An embedder that KNOWS no reader can
 * hold a proxy (a single-threaded / exclusive build) may pass a synchronous
 * shim -- void f(head, func){ func(head); } -- to free in place with no grace
 * period.  The convenience wrapper urcu_txn_sw_commit() passes the
 * compile-time-selected call_rcu (hence its include-after-flavor requirement).
 *
 * Reclaim:
 *   - nr >= 2 (proxies parked): the group block (carrying the record array) is
 *     deferred through call_rcu_fn(.., urcu_txn_sw_free_rcu).
 *   - nr <= 1 / sticky OOM (no proxy ever published): the record array is freed
 *     at once (caller-owned inline storage is left untouched); no group block
 *     was allocated, and @call_rcu_fn is never invoked.
 *
 * Two PREPARE shortcuts let an embedder record then commit WITHOUT an explicit
 * urcu_txn_sw_install():
 *
 *   - Single edge (nr == 1): one recorded edge has no cross-edge atomicity to
 *     provide -- a lone release store to its slot IS already an atomic commit --
 *     so no proxy is installed and no group block is allocated.  A reader of that
 *     slot observes the old or the new target directly, never a proxy, so none
 *     can be held: no grace period is owed and the record array is freed at once.
 *     This makes the common one-pointer publish as cheap as a bare
 *     rcu_assign_pointer, with no proxy alloc / install / settle / GP reclaim.
 *
 *   - Multi-edge (nr >= 2): auto-install -- allocate the group block, park every
 *     proxy, then flip.  (A white-box caller that needs work BETWEEN install and
 *     the flip can call urcu_txn_sw_install() itself; commit then reuses the
 *     block it allocated and takes the call_rcu_fn reclaim path.)
 */
static inline
enum urcu_txn_status urcu_txn_sw_commit_flavor(struct urcu_txn_sw_txn *t,
		void (*call_rcu_fn)(struct rcu_head *,
			void (*)(struct rcu_head *)))
{
	struct urcu_txn_sw_block *blk;
	unsigned int i;

	if (caa_unlikely(t->state == URCU_TXN_SW_OOM)) {
		urcu_txn_sw__free_records(t);
		return URCU_TXN_STATUS_MEMORY_ERROR;
	}
	if (t->state == URCU_TXN_SW_PREPARE) {
		if (t->nr <= 1) {
			if (t->nr == 1) {
				struct urcu_txn_sw_latch *l = &t->latches[0];

				uatomic_store(l->slot, l->proxy.ptr[1],
						CMM_RELEASE);
			}
			/* nr == 0: empty txn, nothing published. */
			urcu_txn_sw__free_records(t);	/* no proxy: free now */
			return URCU_TXN_STATUS_OK;
		}
		urcu_txn_sw_install(t);	/* lazily allocs block, parks proxies */
		if (caa_unlikely(t->state == URCU_TXN_SW_OOM)) {
			urcu_txn_sw__free_records(t);
			return URCU_TXN_STATUS_MEMORY_ERROR;
		}
	}

	/*
	 * INSTALLED: proxies parked in the block, which carries the record array
	 * inline.  The GP free reclaims it, so it must be heap-owned: an inline
	 * (caller-storage) txn never parks proxies (record() asserts it cannot
	 * grow past its lone-edge bound), so nr >= 2 here implies a heap block.
	 */
	urcu_posix_assert(!t->latches_inline);
	blk = t->block;
	urcu_txn_sw_group_commit(&blk->group);
	for (i = 0; i < t->nr; i++) {
		struct urcu_txn_sw_latch *l = &blk->latches[i];

		uatomic_store(l->slot, l->proxy.ptr[1], CMM_RELEASE);
	}
	call_rcu_fn(&blk->rcu_head, urcu_txn_sw_free_rcu);	/* a reader may hold a proxy */
	t->block = NULL;		/* handle consumed */
	t->latches = NULL;
	return URCU_TXN_STATUS_OK;
}

/*
 * Commit deferring reclaim through the compile-time-selected RCU flavor's
 * call_rcu (so this header must be included after an RCU flavor header).  A thin
 * wrapper over urcu_txn_sw_commit_flavor(); see it for the full contract.
 */
static inline
enum urcu_txn_status urcu_txn_sw_commit(struct urcu_txn_sw_txn *t)
{
	return urcu_txn_sw_commit_flavor(t, call_rcu);
}

#ifdef __cplusplus
}
#endif

#endif /* _URCU_RCU_TXN_SW_H */
