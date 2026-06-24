// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

#ifndef _URCU_FLIP_LATCH_H
#define _URCU_FLIP_LATCH_H

/*
 * Flip-latch: atomically switch a *set* of pointers from an "old" value
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
 * The flip-latch removes the intermediate states by one level of
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
 *     and routing proxy resolution through urcu_flip_proxy_get();
 *   - allocating proxies and the group (often a single backing block with
 *     one rcu_head), with whatever alignment its tagging scheme needs;
 *   - reclaiming them with call_rcu() after they are unpublished (every
 *     slot rewritten from the tagged proxy to the resolved target).
 *
 * Lifecycle (writer)
 * ------------------
 *   1. Build the new structure invisibly.
 *   2. urcu_flip_group_init(group); for each slot, init a proxy with its
 *      {old, new} target and point the slot at the tagged proxy.  The
 *      selector is 0, so this is transparent to readers (they still
 *      resolve to old).
 *   3. urcu_flip_commit(group): one release store, 0 -> 1.  Every proxy
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
 *   1. The low-level flip group / proxy primitive above (urcu_flip_group,
 *      urcu_flip_proxy, urcu_flip_proxy_get, urcu_flip_commit).  The embedder
 *      allocates and reclaims the proxies, tags them, and routes resolution --
 *      as in the four-step lifecycle described above.
 *
 *   2. A growable multi-edge transaction, urcu_flip_txn, built on that
 *      primitive (defined lower in this file).  It owns proxy allocation,
 *      installation, settle and reclaim, so an embedder records edges and
 *      commits a whole set atomically instead of hand-managing proxies.  Its
 *      lifecycle is a state machine:
 *
 *        create -> PREPARE --record--> ... --commit--> committed
 *
 *      record() appends an edge {slot, old, new} into the (realloc-grown)
 *      record array but installs nothing; the record set is FROZEN once commit()
 *      parks the proxies, matching the lock-free engine's contract (no edge may
 *      be added once a proxy is parked), so an embedder written against this
 *      transaction can migrate to <urcu/flip-latch-lockfree.h> mechanically.
 *      commit() parks every recorded proxy, flips the group and settles to new;
 *      an embedder that records nothing or bails on a record() OOM frees the txn
 *      with urcu_flip_txn_destroy.  The single embedder hook is a tag function
 *      (proxy -> tagged slot value); the embedder drives reclaim from commit()'s
 *      "owe a grace period" return.  See the urcu_flip_txn block below.
 */

#include <stdbool.h>
#include <stdlib.h>
#include <urcu/assert.h>
#include <urcu/compiler.h>
#include <urcu/uatomic.h>
#include <urcu/call-rcu.h>		/* struct rcu_head (embedder reclaim) */

/*
 * Shared selector for a flip group.  selector == 0 -> proxies resolve to
 * old_ptr; selector == 1 -> new_ptr.  Written once (0 -> 1) with release
 * semantics by urcu_flip_commit(); read with acquire by
 * urcu_flip_proxy_get().
 */
struct urcu_flip_group {
	unsigned long selector;
};

/*
 * ptr[0] is the old target, ptr[1] the new one: the selector (0 -> old,
 * 1 -> new) indexes this array directly in urcu_flip_proxy_get().
 */
struct urcu_flip_proxy {
	void *ptr[2];
	struct urcu_flip_group *group;
};

static inline
void urcu_flip_group_init(struct urcu_flip_group *group)
{
	group->selector = 0;
}

static inline
void urcu_flip_proxy_init(struct urcu_flip_proxy *proxy,
		struct urcu_flip_group *group, void *old_ptr, void *new_ptr)
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
 * target's contents -- published before urcu_flip_commit()'s release store
 * -- are visible whenever selector == 1 is observed.
 *
 * The selector indexes proxy->ptr[] directly, so resolution is a pure data
 * dependency rather than a conditional branch.
 */
static inline
void *urcu_flip_proxy_get(const struct urcu_flip_proxy *proxy)
{
	return proxy->ptr[uatomic_load(&proxy->group->selector, CMM_ACQUIRE)];
}

/*
 * Commit the flip: switch every proxy in @group from old to new with a
 * single release store.  Must be called after the new targets are fully
 * built (the release pairs with urcu_flip_proxy_get()'s acquire).
 */
static inline
void urcu_flip_commit(struct urcu_flip_group *group)
{
	uatomic_store(&group->selector, 1, CMM_RELEASE);
}

/*
 * Multi-edge flip transaction (urcu_flip_txn)
 * ===========================================
 *
 * A growable transaction over a set of slots, built on the flip group above.
 *
 *   create --> PREPARE --record--> PREPARE --commit--> committed
 *                 |
 *                 | destroy (e.g. OOM bail: nothing published, no grace period)
 *                 v
 *               freed
 *
 * record() appends a latch descriptor {slot, old, new} into the record array
 * but installs NOTHING -- no proxy address is live yet, so the array grows by
 * realloc.  The record set is FROZEN once proxies are installed (which commit()
 * does internally), so record() must precede commit().  This is the same
 * frozen-set contract as the lock-free engine (<urcu/flip-latch-lockfree.h>), so
 * a single-writer embedder can later migrate to lock-free writers without
 * restructuring its mutations.
 *
 * commit() publishes the whole set atomically: it parks every recorded latch's
 * tagged proxy (readers still resolve to old; selector == 0), flips the group
 * with one release store (every proxy resolves to new at once), then settles
 * each slot to its direct new value.  It returns true when a reader may hold a
 * proxy and a grace period is owed before reclaim -- the embedder then defers
 * urcu_flip_txn_free_rcu via its call_rcu.  It returns false when no proxy was
 * ever published (the single-edge fast path and the empty txn); the embedder
 * frees immediately with urcu_flip_txn_destroy, which is also how it bails out
 * after a record() OOM.  Fresh nodes are the embedder's; the txn never tracks
 * them.
 *
 * The single embedder hook is @tag: given a recorded proxy, return the tagged
 * pointer value to store in the slot (e.g. set a reserved type code).  The
 * latch is 16-byte aligned so the tag may use the low 4 bits.
 */

enum urcu_flip_txn_state {
	URCU_FLIP_TXN_PREPARE = 0,
	URCU_FLIP_TXN_INSTALLED,	/* internal: set once proxies are parked */
};

struct urcu_flip_latch {
	struct urcu_flip_proxy proxy;	/* ptr[0]=old, ptr[1]=new, group */
	void **slot;			/* install / settle target */
} __attribute__((aligned(16)));

/*
 * The txn header (group + rcu_head) is a stable allocation -- every parked proxy
 * holds &t->group, so it must never move.  The record array @latches is a
 * separate block that realloc-grows in PREPARE; that is safe because no proxy is
 * installed yet (no slot points into the array), and once commit() parks proxies
 * the array is frozen and never reallocated again.
 *
 * The header carries no alignment of its own: &t->group is stored in each proxy
 * as a plain pointer and only dereferenced (proxy->group->selector) -- it is
 * never tagged, so it needs no free low bits.  The tag room that matters is on
 * the tagged proxies an embedder actually stores in its slots; those live in
 * @latches, and struct urcu_flip_latch's own 16-byte alignment keeps each one
 * 16-byte aligned (low 4 bits free), so a low-4-bit pointer-tagging embedder
 * (e.g. the fractal trie) can route its slots through this engine.
 */
struct urcu_flip_txn {
	struct urcu_flip_group group;
	struct rcu_head rcu_head;	/* embedder's deferred-free handle */
	void *(*tag)(struct urcu_flip_proxy *proxy);
	enum urcu_flip_txn_state state;
	struct urcu_flip_latch *latches;	/* record array (realloc-grown) */
	unsigned int nr;
	unsigned int cap;
};

#define URCU_FLIP_TXN_CAP	8	/* initial record-array capacity */

static inline
struct urcu_flip_txn *urcu_flip_txn_create(void *(*tag)(struct urcu_flip_proxy *))
{
	struct urcu_flip_txn *t;

	t = (struct urcu_flip_txn *) malloc(sizeof(*t));
	if (!t)
		return NULL;
	urcu_flip_group_init(&t->group);
	t->tag = tag;
	t->state = URCU_FLIP_TXN_PREPARE;
	t->latches = NULL;
	t->nr = 0;
	t->cap = 0;
	return t;
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
 * an OOM return.  Call once, right after create, before any record.  Returns
 * false on OOM (the caller destroys the txn).
 */
static inline
bool urcu_flip_txn_reserve(struct urcu_flip_txn *t, unsigned int cap)
{
	struct urcu_flip_latch *l;

	if (t->latches)
		return cap <= t->cap;		/* already sized */
	if (cap < URCU_FLIP_TXN_CAP)
		cap = URCU_FLIP_TXN_CAP;
	l = (struct urcu_flip_latch *) malloc((size_t) cap * sizeof(*l));
	if (!l)
		return false;
	t->latches = l;
	t->cap = cap;
	return true;
}

/* Free the record array and the txn header (no grace period). */
static inline
void urcu_flip_txn_destroy(struct urcu_flip_txn *t)
{
	free(t->latches);
	free(t);
}

/* call_rcu callback: deferred urcu_flip_txn_destroy. */
static inline
void urcu_flip_txn_free_rcu(struct rcu_head *head)
{
	urcu_flip_txn_destroy(caa_container_of(head, struct urcu_flip_txn,
				rcu_head));
}

static inline
void urcu_flip_latch_set(struct urcu_flip_txn *t, struct urcu_flip_latch *l,
		void **slot, void *old_ptr, void *new_ptr)
{
	urcu_flip_proxy_init(&l->proxy, &t->group, old_ptr, new_ptr);
	l->slot = slot;
}

/* Park latch @l's tagged proxy into its slot (readers resolve to old). */
static inline
void urcu_flip_latch_install(struct urcu_flip_txn *t, struct urcu_flip_latch *l)
{
	uatomic_store(l->slot, t->tag(&l->proxy), CMM_RELEASE);
}

/*
 * Record one edge {*slot: old -> new}.  PREPARE only -- the record set is frozen
 * once proxies are installed, so this must run before commit() (record() after a
 * commit/install is a usage error).  Returns false on OOM (the only failure);
 * the caller then frees the txn with urcu_flip_txn_destroy.  The record array
 * realloc-grows on demand and nothing is installed here (no proxy address is
 * live until commit parks them).
 */
static inline
bool urcu_flip_txn_record(struct urcu_flip_txn *t, void **slot,
		void *old_ptr, void *new_ptr)
{
	struct urcu_flip_latch *l;

	urcu_posix_assert(t->state == URCU_FLIP_TXN_PREPARE);
	if (t->nr == t->cap) {
		unsigned int newcap = t->cap ? t->cap * 2 : URCU_FLIP_TXN_CAP;
		struct urcu_flip_latch *nl;

		/* No proxy address is live yet -> realloc may move the array. */
		nl = (struct urcu_flip_latch *) realloc(t->latches,
				(size_t) newcap * sizeof(*nl));
		if (!nl)
			return false;
		t->latches = nl;
		t->cap = newcap;
	}
	l = &t->latches[t->nr++];
	urcu_flip_latch_set(t, l, slot, old_ptr, new_ptr);
	return true;
}

/*
 * Internal: PREPARE -> INSTALLED, parking every recorded latch's proxy into its
 * slot (readers still resolve to old; selector == 0).  commit() drives this; it
 * is not part of the embedder API (embedders record() then commit()).
 */
static inline
void urcu_flip_txn_install(struct urcu_flip_txn *t)
{
	unsigned int i;

	t->state = URCU_FLIP_TXN_INSTALLED;
	for (i = 0; i < t->nr; i++)
		urcu_flip_latch_install(t, &t->latches[i]);
}

/*
 * Commit: flip the group (every proxy resolves to new atomically), then settle
 * each slot to its direct new value.  Returns true: a reader may hold a proxy,
 * so the embedder owes a grace period before urcu_flip_txn_destroy.
 *
 * Two PREPARE shortcuts let an embedder record then commit WITHOUT an explicit
 * urcu_flip_txn_install():
 *
 *   - Single edge (nr == 1): one recorded edge has no cross-edge atomicity to
 *     provide -- a lone release store to its slot IS already an atomic commit --
 *     so no proxy is installed at all.  A reader of that slot observes the old
 *     or the new target directly, never a proxy, so none can be held: NO grace
 *     period is owed (returns false) and the embedder frees the txn at once.
 *     This makes the common one-pointer publish as cheap as a bare
 *     rcu_assign_pointer, with no proxy alloc / install / settle / GP reclaim
 *     and no per-read proxy resolution.
 *
 *   - Multi-edge (nr >= 2): auto-install -- park every proxy first, then flip
 *     the group as usual.  (A white-box caller that needs work BETWEEN install
 *     and the flip can call the internal urcu_flip_txn_install() itself.)
 */
static inline
bool urcu_flip_txn_commit(struct urcu_flip_txn *t)
{
	unsigned int i;

	if (t->state == URCU_FLIP_TXN_PREPARE) {
		if (t->nr == 1) {
			struct urcu_flip_latch *l = &t->latches[0];

			uatomic_store(l->slot, l->proxy.ptr[1], CMM_RELEASE);
			return false;
		}
		if (t->nr == 0)
			return false;		/* empty txn: nothing to do */
		urcu_flip_txn_install(t);	/* auto-install before the flip */
	}

	urcu_flip_commit(&t->group);
	for (i = 0; i < t->nr; i++) {
		struct urcu_flip_latch *l = &t->latches[i];

		uatomic_store(l->slot, l->proxy.ptr[1], CMM_RELEASE);
	}
	return true;
}

#endif /* _URCU_FLIP_LATCH_H */
