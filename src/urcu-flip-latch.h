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
 *                 |                                          |  ^
 *                 | abort (no GP)                            |  | record (append + install)
 *                 v                                          v  |
 *               destroy                                  INSTALLED --abort--> aborted (GP)
 *
 * In PREPARE, record() appends a latch descriptor {slot, old, new} but installs
 * NOTHING -- no proxy address is live, so the head chunk grows by realloc.
 * install() freezes the head chunk and parks every recorded latch's tagged
 * proxy into its slot (readers still resolve to old; selector == 0).  In
 * INSTALLED, record() appends to a fresh chunk (never moving an installed
 * proxy) and installs immediately.  commit() flips the group then settles every
 * slot to its new value; abort() restores every installed slot to its old value
 * (a no-op in PREPARE).  commit() and a post-install abort() return true ("owe a
 * grace period" -- a reader may hold a proxy), so the embedder defers
 * urcu_flip_txn_free_rcu via its call_rcu; a PREPARE abort returns false and the
 * embedder frees immediately with urcu_flip_txn_destroy.  Fresh nodes are the
 * embedder's; the txn never tracks them.
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
	struct urcu_flip_chunk *next;
	unsigned int nr;
	unsigned int cap;
	struct urcu_flip_latch latches[];
};

struct urcu_flip_txn {
	struct urcu_flip_group group;
	struct rcu_head rcu_head;	/* embedder's deferred-free handle */
	void *(*tag)(struct urcu_flip_proxy *proxy);
	enum urcu_flip_txn_state state;
	struct urcu_flip_chunk *head;	/* chunk 0 (realloc in PREPARE) ... */
	struct urcu_flip_chunk *tail;	/* ... appended fixed chunks */
	unsigned int nr;
	bool placed;			/* a urcu_flip_txn_reserve_slot proxy is live */
};

#define URCU_FLIP_CHUNK0_CAP	8	/* initial head-chunk capacity */
#define URCU_FLIP_CHUNKN_CAP	8	/* fixed post-install chunk capacity */

static inline
struct urcu_flip_chunk *urcu_flip_chunk_alloc(unsigned int cap)
{
	struct urcu_flip_chunk *c;

	c = (struct urcu_flip_chunk *) malloc(sizeof(*c) +
			(size_t) cap * sizeof(struct urcu_flip_latch));
	if (!c)
		return NULL;
	c->next = NULL;
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
	t->tail = NULL;
	t->nr = 0;
	t->placed = false;
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
	t->head = t->tail = c;
	return true;
}

/* Free every chunk and the txn header (no grace period). */
static inline
void urcu_flip_txn_destroy(struct urcu_flip_txn *t)
{
	struct urcu_flip_chunk *c = t->head;

	while (c) {
		struct urcu_flip_chunk *next = c->next;

		free(c);
		c = next;
	}
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
 * the caller then aborts.  In PREPARE the head chunk realloc-grows and nothing
 * installs; in INSTALLED a fresh chunk is appended and the proxy installs now.
 */
static inline
bool urcu_flip_txn_record(struct urcu_flip_txn *t, void **slot,
		void *old_ptr, void *new_ptr)
{
	struct urcu_flip_chunk *c;
	struct urcu_flip_latch *l;

	if (t->state == URCU_FLIP_TXN_PREPARE) {
		c = t->head;
		if (!c) {
			c = urcu_flip_chunk_alloc(URCU_FLIP_CHUNK0_CAP);
			if (!c)
				return false;
			t->head = t->tail = c;
		} else if (c->nr == c->cap) {
			unsigned int newcap = c->cap * 2;
			struct urcu_flip_chunk *nc;

			/* No address is live yet -> realloc may move the chunk. */
			nc = (struct urcu_flip_chunk *) realloc(c, sizeof(*c) +
				(size_t) newcap * sizeof(struct urcu_flip_latch));
			if (!nc)
				return false;
			nc->cap = newcap;
			t->head = t->tail = c = nc;
		}
		l = &c->latches[c->nr++];
		urcu_flip_latch_set(t, l, slot, old_ptr, new_ptr);
		t->nr++;
		return true;
	}

	/* INSTALLED: append to a fresh fixed chunk, install immediately. */
	c = t->tail;
	if (!c || c->nr == c->cap) {
		struct urcu_flip_chunk *nc =
			urcu_flip_chunk_alloc(URCU_FLIP_CHUNKN_CAP);

		if (!nc)
			return false;
		if (t->tail)
			t->tail->next = nc;
		else
			t->head = nc;
		t->tail = nc;
		c = nc;
	}
	l = &c->latches[c->nr++];
	urcu_flip_latch_set(t, l, slot, old_ptr, new_ptr);
	urcu_flip_latch_install(t, l);
	t->nr++;
	return true;
}

/*
 * Reserve one slot-flip latch whose target slot is not yet known, and return
 * its tagged proxy for the embedder to place itself.  The txn analogue of an
 * embedder-managed flip-batch "add": use it when the value to flip must be
 * handed to a builder that decides WHERE it lands (e.g. a node insert that may
 * relocate the slot by recompacting), so the proxy is needed BEFORE the slot
 * address exists.  The returned proxy belongs to the txn's group and resolves
 * to @old_ptr until commit; the embedder stores the returned tagged pointer
 * wherever the value ends up, then records the final slot with
 * urcu_flip_txn_bind_slot once known so commit can settle it (and a post-place
 * abort can restore it).  *@latch returns the latch handle for that later bind.
 *
 * Valid ONLY on a txn RESERVED (urcu_flip_txn_reserve) to cover this latch and
 * still in PREPARE: the returned proxy address must stay stable, but an
 * unreserved PREPARE head chunk realloc-grows and would move it.  Asserts the
 * reservation; never reallocates and never installs (there is no slot yet) --
 * subsequent record()s still append into the reserved head chunk without
 * allocating.  Because the placed proxy is immediately reader-visible once the
 * embedder stores it, commit() must flip the group rather than take the
 * single-edge bare-store fast path: @placed records that, so the embedder need
 * not call install() explicitly.  The embedder MUST bind the slot before commit.
 */
static inline
void *urcu_flip_txn_reserve_slot(struct urcu_flip_txn *t, void *old_ptr,
		void *new_ptr, struct urcu_flip_latch **latch)
{
	struct urcu_flip_chunk *c = t->head;
	struct urcu_flip_latch *l;

	urcu_posix_assert(t->state == URCU_FLIP_TXN_PREPARE);
	urcu_posix_assert(c && c->nr < c->cap);	/* reserved -> stable, no realloc */
	l = &c->latches[c->nr++];
	urcu_flip_proxy_init(&l->proxy, &t->group, old_ptr, new_ptr);
	l->slot = NULL;				/* bound via urcu_flip_txn_bind_slot */
	t->nr++;
	t->placed = true;
	*latch = l;
	return t->tag(&l->proxy);
}

/*
 * Bind the target slot of a latch obtained from urcu_flip_txn_reserve_slot,
 * once the embedder knows where it placed the proxy.  commit() then settles
 * @slot to the new target (and a post-place abort() restores it to old).
 */
static inline
void urcu_flip_txn_bind_slot(struct urcu_flip_latch *l, void **slot)
{
	l->slot = slot;
}

/* PREPARE -> INSTALLED: park every recorded latch's proxy into its slot. */
static inline
void urcu_flip_txn_install(struct urcu_flip_txn *t)
{
	struct urcu_flip_chunk *c;
	unsigned int i;

	t->state = URCU_FLIP_TXN_INSTALLED;
	for (c = t->head; c; c = c->next)
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
		 * The PREPARE shortcuts assume no proxy is live yet.  A
		 * urcu_flip_txn_reserve_slot proxy IS already live in its slot
		 * (the embedder placed it), so a reader may hold it: skip the
		 * shortcuts and flip the group (install re-parks it idempotently).
		 */
		if (!t->placed) {
			if (t->nr == 1) {
				struct urcu_flip_latch *l = &t->head->latches[0];

				uatomic_store(l->slot, l->proxy.ptr[1],
						CMM_RELEASE);
				return false;
			}
			if (t->nr == 0)
				return false;	/* empty txn: nothing to do */
		}
		urcu_flip_txn_install(t);	/* auto-install before the flip */
	}

	urcu_flip_commit(&t->group);
	for (c = t->head; c; c = c->next)
		for (i = 0; i < c->nr; i++) {
			struct urcu_flip_latch *l = &c->latches[i];

			uatomic_store(l->slot, l->proxy.ptr[1], CMM_RELEASE);
		}
	return true;
}

/*
 * Abort.  PREPARE with nothing placed: nothing is installed -> returns false
 * (embedder frees immediately, no GP).  INSTALLED, or PREPARE holding a live
 * urcu_flip_txn_reserve_slot proxy: restore each slot to its old value (a reader
 * sees old via the proxy or old direct -- the same view) and return true (the
 * proxy memory still owes a GP).  Either way the embedder frees its fresh nodes.
 * Slots not yet bound (a reserve_slot latch whose placement failed before
 * urcu_flip_txn_bind_slot) hold no proxy and are skipped.
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

	if (t->state == URCU_FLIP_TXN_PREPARE && !t->placed)
		return false;
	for (c = t->head; c; c = c->next)
		for (i = 0; i < c->nr; i++) {
			struct urcu_flip_latch *l = &c->latches[i];

			if (l->slot)
				uatomic_store(l->slot, l->proxy.ptr[0],
						CMM_RELAXED);
		}
	return true;
}

#endif /* _URCU_FLIP_LATCH_H */
