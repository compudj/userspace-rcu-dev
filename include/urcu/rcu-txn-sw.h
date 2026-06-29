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
 *      proxies, or frees it at once on the single-edge / empty / OOM paths.  The
 *      single embedder hook is a tag function (proxy -> tagged slot value); the
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
#include <stdlib.h>
#include <string.h>
#include <urcu/assert.h>
#include <urcu/compiler.h>
#include <urcu/uatomic.h>
#include <urcu/call-rcu.h>		/* struct rcu_head + call_rcu (commit reclaim) */
#include <urcu/rcu-txn-status.h>	/* enum urcu_txn_status */

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
 * restructuring its mutations.
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
 * The single embedder hook is @tag: given a recorded proxy, return the tagged
 * pointer value to store in the slot (e.g. set a reserved type code).  The
 * latch array is allocated 16-byte aligned (posix_memalign), so each latch --
 * hence each tagged proxy -- has its low 4 bits free for the embedder's tag.
 */

enum urcu_txn_sw_state {
	URCU_TXN_SW_PREPARE = 0,
	URCU_TXN_SW_INSTALLED,	/* internal: set once proxies are parked */
	URCU_TXN_SW_OOM,		/* sticky: commit -> MEMORY_ERROR */
};

struct urcu_txn_sw_latch {
	struct urcu_txn_sw_proxy proxy;	/* ptr[0]=old, ptr[1]=new, group */
	void **slot;			/* install / settle target */
} __attribute__((aligned(16)));

/*
 * Persistent group block: the part of a committed transaction that must outlive
 * the commit until a grace period.  It carries the flip group every parked
 * proxy reads (&block->group is stored in each proxy as a plain pointer -- never
 * tagged -- so it needs no alignment of its own), the rcu_head that defers its
 * reclaim, and the record array (the proxies themselves live there).  It is
 * allocated lazily, only when a commit actually parks proxies (nr >= 2), and
 * freed -- record array and all -- by the single call_rcu the proxy lifetime
 * requires.  The single-edge and empty paths never allocate it.
 */
struct urcu_txn_sw_group_block {
	struct urcu_txn_sw_group group;
	struct rcu_head rcu_head;		/* deferred-free handle */
	struct urcu_txn_sw_latch *latches;	/* record array, freed with the block */
};

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
	void *(*tag)(struct urcu_txn_sw_proxy *proxy);
	enum urcu_txn_sw_state state;
	struct urcu_txn_sw_latch *latches;	/* record array (realloc-grown) */
	struct urcu_txn_sw_group_block *block;	/* lazy: NULL until proxies parked */
	unsigned int nr;
	unsigned int cap;
};

#define URCU_TXN_SW_CAP	8	/* initial record-array capacity */

/*
 * Initialize an on-stack transaction handle with the embedder's @tag hook.  No
 * allocation, so this cannot fail; the first record()/reserve() is the first OOM
 * checkpoint, and it is sticky (commit reports MEMORY_ERROR).
 */
static inline
void urcu_txn_sw_init(struct urcu_txn_sw_txn *t,
		void *(*tag)(struct urcu_txn_sw_proxy *))
{
	t->tag = tag;
	t->state = URCU_TXN_SW_PREPARE;
	t->latches = NULL;
	t->block = NULL;
	t->nr = 0;
	t->cap = 0;
}

/*
 * Allocate a record array of @size bytes, 16-byte aligned.  Each latch's proxy
 * is tagged into a slot, so the array base must be 16-byte aligned for every
 * latch -- hence every tagged proxy -- to keep its low 4 bits free for the
 * embedder's tag.  posix_memalign guarantees that portably; plain malloc would
 * only promise max_align_t (8 on some 32-bit ABIs).  Returns NULL on OOM.
 */
static inline
struct urcu_txn_sw_latch *urcu_txn_sw__latches_alloc(size_t size)
{
	void *p;

	if (posix_memalign(&p, 16, size))
		return NULL;
	return (struct urcu_txn_sw_latch *) p;
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
 * an OOM return.  Call once, right after init, before any record.  Returns
 * false on OOM; the failure is sticky (URCU_TXN_SW_OOM), so the caller may
 * ignore this return and let commit() report MEMORY_ERROR.
 */
static inline
bool urcu_txn_sw_reserve(struct urcu_txn_sw_txn *t, unsigned int cap)
{
	struct urcu_txn_sw_latch *l;

	if (caa_unlikely(t->state == URCU_TXN_SW_OOM))
		return false;			/* sticky: an earlier alloc failed */
	if (t->latches)
		return cap <= t->cap;		/* already sized */
	if (cap < URCU_TXN_SW_CAP)
		cap = URCU_TXN_SW_CAP;
	l = urcu_txn_sw__latches_alloc((size_t) cap * sizeof(*l));
	if (!l) {
		t->state = URCU_TXN_SW_OOM;	/* sticky: commit reports it */
		return false;
	}
	t->latches = l;
	t->cap = cap;
	return true;
}

/* Free the record array of a handle that never parked proxies (no grace period). */
static inline
void urcu_txn_sw__free_records(struct urcu_txn_sw_txn *t)
{
	free(t->latches);
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
	struct urcu_txn_sw_group_block *blk = caa_container_of(head,
			struct urcu_txn_sw_group_block, rcu_head);

	free(blk->latches);
	free(blk);
}

/* Record a latch's {old, new, slot}; its proxy->group is bound at install. */
static inline
void urcu_txn_sw_latch_set(struct urcu_txn_sw_latch *l,
		void **slot, void *old_ptr, void *new_ptr)
{
	l->proxy.ptr[0] = old_ptr;
	l->proxy.ptr[1] = new_ptr;
	l->slot = slot;
}

/* Park latch @l's tagged proxy into its slot (readers resolve to old). */
static inline
void urcu_txn_sw_latch_install(struct urcu_txn_sw_txn *t, struct urcu_txn_sw_latch *l)
{
	l->proxy.group = &t->block->group;	/* bind to the now-allocated group */
	uatomic_store(l->slot, t->tag(&l->proxy), CMM_RELEASE);
}

/*
 * Record one edge {*slot: old -> new}.  PREPARE only -- the record set is frozen
 * once proxies are installed, so this must run before commit() (record() after a
 * commit/install is a usage error).  Returns false on OOM (the only failure);
 * the failure is sticky (URCU_TXN_SW_OOM), so the caller may ignore this
 * return and let commit() report MEMORY_ERROR.  The record array realloc-grows
 * on demand and nothing is installed here (no proxy address is live until commit
 * parks them).
 */
static inline
bool urcu_txn_sw_record(struct urcu_txn_sw_txn *t, void **slot,
		void *old_ptr, void *new_ptr)
{
	struct urcu_txn_sw_latch *l;

	if (caa_unlikely(t->state == URCU_TXN_SW_OOM))
		return false;			/* sticky: an earlier alloc failed */
	urcu_posix_assert(t->state == URCU_TXN_SW_PREPARE);
	if (t->nr == t->cap) {
		unsigned int newcap = t->cap ? t->cap * 2 : URCU_TXN_SW_CAP;
		struct urcu_txn_sw_latch *nl;

		/*
		 * No proxy address is live yet -> the array may move.  No
		 * aligned realloc exists, so allocate a fresh 16-byte-aligned
		 * block, copy the buffered records, and free the old one.
		 */
		nl = urcu_txn_sw__latches_alloc(
				(size_t) newcap * sizeof(*nl));
		if (!nl) {
			t->state = URCU_TXN_SW_OOM;	/* sticky: commit reports it */
			return false;
		}
		memcpy(nl, t->latches, (size_t) t->nr * sizeof(*nl));
		free(t->latches);
		t->latches = nl;
		t->cap = newcap;
	}
	l = &t->latches[t->nr++];
	urcu_txn_sw_latch_set(l, slot, old_ptr, new_ptr);
	return true;
}

/*
 * Lazily allocate the persistent group block the first time proxies are parked
 * (commit's auto-install, or an explicit urcu_txn_sw_install()).  Returns
 * false on OOM (sticky URCU_TXN_SW_OOM); commit then reports MEMORY_ERROR.
 */
static inline
bool urcu_txn_sw__ensure_group(struct urcu_txn_sw_txn *t)
{
	struct urcu_txn_sw_group_block *blk;

	if (t->block)
		return true;
	blk = (struct urcu_txn_sw_group_block *) malloc(sizeof(*blk));
	if (caa_unlikely(!blk)) {
		t->state = URCU_TXN_SW_OOM;	/* sticky: commit reports it */
		return false;
	}
	urcu_txn_sw_group_init(&blk->group);
	blk->latches = NULL;			/* commit hands the record array over */
	t->block = blk;
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

	if (caa_unlikely(!urcu_txn_sw__ensure_group(t)))
		return;				/* OOM: sticky; nothing parked */
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
 * Reclaim:
 *   - nr >= 2 (proxies parked): the group block (carrying the record array) is
 *     deferred through call_rcu(urcu_txn_sw_free_rcu) (the flavor's call_rcu,
 *     hence the include-after-flavor requirement).
 *   - nr <= 1 / sticky OOM (no proxy ever published): the record array is freed
 *     at once; no group block was allocated.
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
 *     block it allocated and takes the call_rcu reclaim path.)
 */
static inline
enum urcu_txn_status urcu_txn_sw_commit(struct urcu_txn_sw_txn *t)
{
	struct urcu_txn_sw_group_block *blk;
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

	/* INSTALLED: proxies parked, group block allocated. */
	blk = t->block;
	blk->latches = t->latches;	/* the GP free reclaims the record array */
	t->latches = NULL;
	urcu_txn_sw_group_commit(&blk->group);
	for (i = 0; i < t->nr; i++) {
		struct urcu_txn_sw_latch *l = &blk->latches[i];

		uatomic_store(l->slot, l->proxy.ptr[1], CMM_RELEASE);
	}
	call_rcu(&blk->rcu_head, urcu_txn_sw_free_rcu);	/* a reader may hold a proxy */
	return URCU_TXN_STATUS_OK;
}

#endif /* _URCU_RCU_TXN_SW_H */
