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
 *   2. A growable, abortable multi-edge transaction, urcu_flip_txn, built on
 *      that primitive (defined lower in this file).  It owns proxy allocation,
 *      installation, settle and reclaim, so an embedder records edges and
 *      commits or aborts a whole set atomically instead of hand-managing
 *      proxies.  Its lifecycle is a state machine:
 *
 *        create -> PREPARE --record--> ... --install--> INSTALLED --commit-->
 *
 *      record() in PREPARE appends an edge {slot, old, new} but installs
 *      nothing (the head chunk grows by realloc, no proxy address is live);
 *      install() parks every recorded proxy; commit() flips the group and
 *      settles to new; abort() restores to old (a no-op before install).  The
 *      single embedder hook is a tag function (proxy -> tagged slot value); the
 *      embedder drives reclaim from commit()/abort()'s "owe a grace period"
 *      return.  See the urcu_flip_txn block below and, for the design
 *      rationale, doc/design/transactional-flip-latch.md.
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
 * A growable, abortable transaction over a set of slots, built on the flip
 * group above.  See doc/design/transactional-flip-latch.md.
 *
 *   create --> PREPARE --record--> PREPARE --install--> INSTALLED --commit--> committed
 *                 |                                          |
 *                 | abort (no GP)                            |
 *                 v                                          v
 *               destroy                              INSTALLED --abort--> aborted (GP)
 *
 * In PREPARE, record() appends a latch descriptor {slot, old, new} but installs
 * NOTHING -- no proxy address is live, so the single edge chunk grows by realloc.
 * record() is valid ONLY in PREPARE: the edge set must be FROZEN before install
 * (no append after install), which is what a future MCAS commit body needs -- it
 * acquires the slots in sorted address order, so the set cannot grow once any
 * proxy is live.  install() freezes the chunk and parks every recorded latch's
 * tagged proxy into its slot (readers still resolve to old; selector == 0).
 * commit() flips the group then settles every slot to its new value; abort()
 * restores every installed slot to its old value (a no-op in PREPARE).  commit()
 * and a post-install abort() return true ("owe a grace period" -- a reader may
 * hold a proxy), so the embedder defers urcu_flip_txn_free_rcu via its call_rcu;
 * a PREPARE abort returns false and the embedder frees immediately with
 * urcu_flip_txn_destroy.  Fresh nodes are the embedder's; the txn never tracks them.
 *
 * The single embedder hook is @tag: given a recorded proxy, return the tagged
 * pointer value to store in the slot (e.g. set a reserved type code).  The
 * latch is 16-byte aligned so the tag may use the low 4 bits.
 */

enum urcu_flip_txn_state {
	URCU_FLIP_TXN_PREPARE = 0,
	URCU_FLIP_TXN_INSTALLED,
};

struct urcu_flip_latch {
	struct urcu_flip_proxy proxy;	/* ptr[0]=old, ptr[1]=new, group */
	void **slot;			/* install / settle / restore target */
} __attribute__((aligned(16)));

struct urcu_flip_chunk {
	unsigned int nr;
	unsigned int cap;
	struct urcu_flip_latch latches[];
};

/*
 * aligned(16): a bounded txn embeds its head chunk immediately after the header
 * (at `t + 1`) in one allocation, and that chunk's latches[] are aligned(16).
 * Forcing the header's size to a multiple of 16 keeps `t + 1` -- hence the
 * inline latches -- 16-aligned (otherwise the compiler's aligned vector moves on
 * an aligned(16) latch fault).  Making it explicit avoids depending on the field
 * count happening to sum to a multiple of 16.
 */
struct urcu_flip_txn {
	struct urcu_flip_group group;
	struct rcu_head rcu_head;	/* embedder's deferred-free handle */
	void *(*tag)(struct urcu_flip_proxy *proxy);
	enum urcu_flip_txn_state state;
	struct urcu_flip_chunk *head;	/* the single edge chunk (realloc-grows in PREPARE) */
	unsigned int nr;
	bool head_inline;		/* head chunk embedded in this allocation */
} __attribute__((aligned(16)));

#define URCU_FLIP_CHUNK0_CAP	8	/* initial head-chunk capacity */

/*
 * Byte size of a bounded txn holding up to @cap edges: the header, the inline
 * head chunk, and @cap latches, laid out exactly as urcu_flip_txn_create_bounded
 * mallocs them.  Lets an embedder back a bounded txn with caller storage (e.g. a
 * 16-aligned on-stack buffer) and urcu_flip_txn_init_bounded it -- no allocation,
 * hence no failure -- for a commit whose edge count is small and bounded (e.g. a
 * lone-edge flip, which parks no proxy and owes no grace period, so the txn need
 * not outlive the call and is never freed).
 */
#define URCU_FLIP_TXN_BOUNDED_BYTES(cap)				\
	(sizeof(struct urcu_flip_txn) + sizeof(struct urcu_flip_chunk)	\
	 + (size_t) (cap) * sizeof(struct urcu_flip_latch))

static inline
struct urcu_flip_chunk *urcu_flip_chunk_alloc(unsigned int cap)
{
	struct urcu_flip_chunk *c;

	c = (struct urcu_flip_chunk *) malloc(sizeof(*c) +
			(size_t) cap * sizeof(struct urcu_flip_latch));
	if (!c)
		return NULL;
	c->nr = 0;
	c->cap = cap;
	return c;
}

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
	t->head = NULL;
	t->nr = 0;
	t->head_inline = false;
	return t;
}

/*
 * Single-allocation bounded txn: the header plus an inline head chunk sized for
 * @cap latches, in ONE malloc -- so a transaction whose edge count is bounded
 * by construction costs one allocation (and one free), like a flat bespoke flip
 * batch, instead of create + reserve's two.  @cap must cover every record:
 * record() never grows the inline chunk (the freeze-before-install model -- no
 * append after install -- needs the edge set bounded up front anyway), so the
 * embedder sizes @cap to its edge bound.  Returns NULL on OOM (the embedder
 * falls back to a degraded direct/sequential publish).
 */
/*
 * Initialize a bounded txn in caller-provided storage @t, which must be at least
 * URCU_FLIP_TXN_BOUNDED_BYTES(cap) and aligned to alignof(struct urcu_flip_txn)
 * (16).  No allocation, so it cannot fail.  The head chunk is inline (at t + 1),
 * sized to @cap; record() never grows it.  The caller owns @t's storage: such a
 * txn must NOT be passed to urcu_flip_txn_destroy / freed -- use it only where
 * the txn need not outlive the call (a lone-edge commit parks no proxy and owes
 * no grace period, so nothing references it after commit returns).
 */
static inline
void urcu_flip_txn_init_bounded(struct urcu_flip_txn *t,
		void *(*tag)(struct urcu_flip_proxy *), unsigned int cap)
{
	struct urcu_flip_chunk *c;

	urcu_flip_group_init(&t->group);
	t->tag = tag;
	t->state = URCU_FLIP_TXN_PREPARE;
	c = (struct urcu_flip_chunk *) (t + 1);
	c->nr = 0;
	c->cap = cap;
	t->head = c;
	t->nr = 0;
	t->head_inline = true;
}

static inline
struct urcu_flip_txn *urcu_flip_txn_create_bounded(
		void *(*tag)(struct urcu_flip_proxy *), unsigned int cap)
{
	struct urcu_flip_txn *t;

	t = (struct urcu_flip_txn *) malloc(URCU_FLIP_TXN_BOUNDED_BYTES(cap));
	if (!t)
		return NULL;
	urcu_flip_txn_init_bounded(t, tag, cap);
	return t;
}

/*
 * Pre-size the PREPARE head chunk to hold at least @cap latches.  Optional:
 * record() already realloc-grows the head chunk on demand and aborts on OOM,
 * which is the general (unbounded) model.  But an embedder whose edge count is
 * bounded by construction can reserve that bound once, up front, where failure
 * is clean -- then every subsequent record() in PREPARE appends without
 * reallocating and so cannot fail.  This trades the design's "abort replaces
 * the count pass" for a single up-front alloc, which is the right call for a
 * bounded transaction whose records are interleaved through a build that does
 * not otherwise thread an OOM return.  Call once, right after create, before
 * any record.  Returns false on OOM (the caller destroys the txn).
 */
static inline
bool urcu_flip_txn_reserve(struct urcu_flip_txn *t, unsigned int cap)
{
	struct urcu_flip_chunk *c;

	if (t->head)
		return cap <= t->head->cap;	/* already sized */
	if (cap < URCU_FLIP_CHUNK0_CAP)
		cap = URCU_FLIP_CHUNK0_CAP;
	c = urcu_flip_chunk_alloc(cap);
	if (!c)
		return false;
	t->head = c;
	return true;
}

/* Free the edge chunk and the txn header (no grace period).  A bounded txn's
 * head chunk is embedded in the header allocation, so it is freed with the
 * header, not separately. */
static inline
void urcu_flip_txn_destroy(struct urcu_flip_txn *t)
{
	if (t->head && !t->head_inline)
		free(t->head);
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
 * Record one edge {*slot: old -> new}.  Returns false on OOM (the only failure);
 * the caller then aborts.  Valid ONLY in PREPARE: the freeze-before-install model
 * forbids appending after install (the MCAS sorted-address-acquisition constraint
 * -- the edge set must be frozen before any proxy goes live), so the txn has a
 * single chunk that realloc-grows while nothing is installed.
 */
static inline
bool urcu_flip_txn_record(struct urcu_flip_txn *t, void **slot,
		void *old_ptr, void *new_ptr)
{
	struct urcu_flip_chunk *c = t->head;
	struct urcu_flip_latch *l;

	urcu_posix_assert(t->state == URCU_FLIP_TXN_PREPARE);
	if (!c) {
		c = urcu_flip_chunk_alloc(URCU_FLIP_CHUNK0_CAP);
		if (!c)
			return false;
		t->head = c;
	} else if (c->nr == c->cap) {
		unsigned int newcap = c->cap * 2;
		struct urcu_flip_chunk *nc;

		/*
		 * A bounded txn's head chunk is embedded in the header allocation
		 * and sized to the embedder's edge bound, so it must never grow --
		 * realloc-ing it would move freed-with-the-header memory.  Overflow
		 * here is an embedder sizing bug.
		 */
		urcu_posix_assert(!t->head_inline);
		/* No address is live yet -> realloc may move the chunk. */
		nc = (struct urcu_flip_chunk *) realloc(c, sizeof(*c) +
			(size_t) newcap * sizeof(struct urcu_flip_latch));
		if (!nc)
			return false;
		nc->cap = newcap;
		t->head = c = nc;
	}
	l = &c->latches[c->nr++];
	urcu_flip_latch_set(t, l, slot, old_ptr, new_ptr);
	t->nr++;
	return true;
}

/* PREPARE -> INSTALLED: park every recorded latch's proxy into its slot. */
static inline
void urcu_flip_txn_install(struct urcu_flip_txn *t)
{
	struct urcu_flip_chunk *c = t->head;
	unsigned int i;

	t->state = URCU_FLIP_TXN_INSTALLED;
	if (c)
		for (i = 0; i < c->nr; i++)
			urcu_flip_latch_install(t, &c->latches[i]);
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
 *     the group as usual.  (An embedder that needs work BETWEEN install and the
 *     flip, e.g. a record() in INSTALLED state, still calls install() itself.)
 */
static inline
bool urcu_flip_txn_commit(struct urcu_flip_txn *t)
{
	struct urcu_flip_chunk *c;
	unsigned int i;

	if (t->state == URCU_FLIP_TXN_PREPARE) {
		/*
		 * Freeze-before-install: nothing is ever stored in a slot until
		 * install/commit, so no proxy is live in PREPARE and the
		 * shortcuts always apply.
		 */
		if (t->nr == 1) {
			struct urcu_flip_latch *l = &t->head->latches[0];

			uatomic_store(l->slot, l->proxy.ptr[1], CMM_RELEASE);
			return false;
		}
		if (t->nr == 0)
			return false;	/* empty txn: nothing to do */
		urcu_flip_txn_install(t);	/* auto-install before the flip */
	}

	urcu_flip_commit(&t->group);
	c = t->head;
	if (c)
		for (i = 0; i < c->nr; i++) {
			struct urcu_flip_latch *l = &c->latches[i];

			uatomic_store(l->slot, l->proxy.ptr[1], CMM_RELEASE);
		}
	return true;
}

/*
 * Abort.  PREPARE: nothing is installed (freeze-before-install -- no slot holds
 * a proxy yet) -> returns false (embedder frees immediately, no GP).  INSTALLED:
 * restore each slot to its old value (a reader sees old via the proxy or old
 * direct -- the same view) and return true (the proxy memory still owes a GP).
 * Either way the embedder frees its fresh nodes.
 *
 * The restore is RELAXED, not release: it moves the slot BACK to ptr[0], a value
 * it already held before this transaction (already published, grace periods
 * elapsed) -- nothing new is being published, so there is no prior write to
 * release-order.  (Commit's settle, by contrast, publishes the freshly-built
 * ptr[1] and must be a release.)
 */
static inline
bool urcu_flip_txn_abort(struct urcu_flip_txn *t)
{
	struct urcu_flip_chunk *c;
	unsigned int i;

	if (t->state == URCU_FLIP_TXN_PREPARE)
		return false;
	c = t->head;
	if (c)
		for (i = 0; i < c->nr; i++) {
			struct urcu_flip_latch *l = &c->latches[i];

			uatomic_store(l->slot, l->proxy.ptr[0], CMM_RELAXED);
		}
	return true;
}

#endif /* _URCU_FLIP_LATCH_H */
