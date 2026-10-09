// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

#ifndef _URCU_RCU_TXN_SW_H
#define _URCU_RCU_TXN_SW_H

/*
 * rcu-txn-sw.h implements "Single-Writer RCU Pseudo-Transactions".
 * This abstraction atomically switches a *set* of pointers from an
 * "old" value to a "new" value with a single store, as observed by
 * concurrent RCU readers.
 *
 * Motivation
 * ----------
 * A structural RCU mutation often needs to re-point many slots such
 * that a reader never observes a partially-updated set.  Re-pointing
 * the slots one by one exposes intermediate states: a reader walking
 * several of them sees a mix of old and new targets.  The usual remedy
 * is a grace-period drain around the update, which is costly.
 *
 * The RCU pseudo-transaction removes the intermediate states by one
 * level of indirection.  Each slot in the set is made to hold a tagged
 * pointer to an RCU transaction record instead of the target directly.
 * The transaction record acts as proxy.  It holds {old_ptr, new_ptr}
 * and a pointer to a shared "flip group" carrying a boolean selector
 * word.  Resolving a proxy returns old_ptr while the selector is 0 and
 * new_ptr once it is 1.  Because every proxy in a group reads the
 * *same* selector, a single store flipping that selector switches the
 * whole set atomically: at any instant every proxy resolves
 * consistently to the old or the new target.
 *
 * The embedding data structure is responsible for:
 *   - excluding concurrent writers from the slots a transaction commits;
 *   - choosing the tag that marks a proxy pointer so its readers recognise
 *     it (a spare low bit, or a reserved type code in an already-tagged
 *     pointer), and never storing a live value that carries it;
 *   - routing its readers' slot loads through urcu_txn_sw_resolve();
 *   - supplying the call_rcu() of its readers' RCU flavor.
 *
 * The transaction allocates the records and their flip group as a single
 * descriptor, and reclaims it after a grace period.
 *
 * Lifecycle (writer)
 * ------------------
 *   1. Build the new structure invisibly, adding records to the RCU
 *      pseudo-transaction's record set.
 *   2. Install: for each slot, install a tagged proxy pointer pointing
 *      to its associated record.  The selector is 0, so this is
 *      transparent to readers (they still resolve to old).
 *   3. Commit: one store-release, 0 -> 1.  Every installed tagged proxy
 *      pointer now resolves to new, atomically.
 *   4. "Settle": rewrite each slot from the tagged proxy back to the
 *      direct new target (idempotent for readers, since the proxy already
 *      resolves to new), then call_rcu() the transaction descriptor.
 *
 * Ordering / monotonicity
 * ------------------------
 * The selector is written exactly once (0 -> 1) and never back, so a
 * reader resolving several proxy records over time observes a monotone
 * old...old,new...new sequence -- never new-then-old.  Embedders can rely
 * on this together with a fixed read order to avoid a per-traversal
 * snapshot (e.g. publishing the back edges of a re-parent through the
 * latch *before* the forward edge, so a reader descending before it
 * walks back up can only ever progress old->new, never regress).
 *
 * Scope of "never new-then-old":  Monotonicity of the selector is
 * unconditional.  It is one word, stored once.  What a reader's sequence of
 * resolutions inherits from it depends on how the reader got from one slot to
 * the next.  A dependency-chained traversal (each hop's address derived from
 * the value the previous hop loaded) carries the order with rcu_dereference.  A
 * reader that hops without that chain -- re-reading a pointer it cached
 * earlier, or walking prev-then-next between two independently-reached slots --
 * has no such edge: under -DURCU_DEREFERENCE_USE_VOLATILE on weakly-ordered
 * hardware its two loads may be reordered, and it can observe the new
 * selector's effect on the later slot and the old on the earlier one.  Such a
 * reader owes itself an explicit acquire (or a dependency).
 *
 * RCU flavor
 * ----------
 * urcu_txn_sw_commit() reclaims the transaction descriptor with call_rcu(), so
 * this header must be included after an RCU flavor header (e.g. <urcu-qsbr.h>)
 * that maps call_rcu() to that flavor.
 */

#include <stdbool.h>
#include <stddef.h>			/* offsetof */
#include <stdlib.h>
#include <string.h>
#include <urcu/assert.h>
#include <urcu/compiler.h>
#include <urcu/uatomic.h>
#include <urcu/call-rcu.h>		/* struct rcu_head, call_rcu() */
#include <urcu/rcu-txn-bloom.h>	/* read-your-own-writes filter */
#include <urcu/rcu-txn-status.h>	/* enum urcu_txn_status */
#include <urcu/rcu-txn-slab.h>		/* per-CPU descriptor slab */

#ifdef URCU_TXN_SW_EXCL_VALIDATE
# include <pthread.h>
#endif
#if defined(URCU_TXN_SW_EXCL_VALIDATE) || defined(URCU_TXN_SW_DEBUG_DISJOINT)
# include <stdio.h>			/* the validators' reports */
#endif

#ifdef __cplusplus
extern "C" {
#endif

/*
 * First capacity of an unreserved handle: the smallest descriptor class.  A
 * two-record commit from the next class up was measured at nearly twice the
 * cost.  A transaction that records more should urcu_txn_sw_reserve() its
 * bound rather than grow to it.
 */
#define URCU_TXN_SW_CAP	4

/*
 * RCU pseudo-transaction (urcu_txn_sw_txn)
 * ========================================
 *
 * A growable transaction over a set of slots.  The handle is a small
 * on-stack object the embedder declares and inits with urcu_txn_sw_init();
 * nothing is allocated until edges are recorded, so the init step has
 * no failure mode.
 *
 *   init --> PREPARE --record--> PREPARE --commit--> committed (handle done)
 *              |                            |
 *              | record()/reserve() OOM     | commit owns reclaim:
 *              v (sticky URCU_TXN_SW_OOM)   | call_rcu txn descriptor
 *        commit reports MEMORY_ERROR        | or immediate record-array free
 *        and frees the record array         v (no proxy: empty/single)
 *                                         freed
 *
 * record() appends a record {slot, old, new} into the record array but
 * does not install any proxy address, so the array grows by realloc.
 * The record set is frozen once proxies are installed (which commit()
 * does internally), so record() must precede commit().  Records must
 * target pairwise-distinct slots.
 *
 * commit() publishes the whole set atomically: it installs every
 * record's tagged proxy (readers still resolve to old; selector == 0),
 * flips the group's selector with one store-release (every proxy
 * resolves to new at once), then settles each slot to its direct new
 * value.  commit() owns reclaim:
 *   - nr >= 2 (proxies installed): a reader may hold a proxy, whose old/new
 *     targets and selector live in the transaction descriptor.  The
 *     transaction descriptor is reclaimed through call_rcu() (after a grace
 *     period).
 *   - nr <= 1 / sticky OOM (no proxy ever published): the transaction
 *     descriptor is freed at once.
 *
 * OOM is sticky: a reserve()/record() that cannot allocate a descriptor
 * causes the following reserve()/record() operations to be no-ops, and the
 * later commit() to report MEMORY_ERROR and to free the descriptor, so an
 * embedder may ignore the bool returns of reserve()/record() and test only
 * commit() -- matching the concurrent front-end's contract.
 *
 * Each recorded edge carries its own tag (urcu_txn_sw_record's @tag): the bits
 * OR'd into that slot's installed proxy value so its readers recognise the
 * proxy (e.g. a reserved type code).  The record array is allocated 16-byte
 * aligned, so each record -- hence each tagged proxy -- has its low 4 bits
 * free for the embedder's tag.
 */

enum urcu_txn_sw_state {
	URCU_TXN_SW_PREPARE = 0,
	URCU_TXN_SW_INSTALLED,	/* internal: set once proxies are installed */
	URCU_TXN_SW_OOM,		/* sticky: commit -> MEMORY_ERROR */
	/*
	 * Terminal: commit() consumed the handle.  Reusing it without
	 * initializing it again trips an assertion, where it would otherwise
	 * store through a stale record array.
	 */
	URCU_TXN_SW_DONE,
};

/*
 * Shared selector for a flip group.  selector == 0 -> records resolve to
 * old_ptr; selector == 1 -> new_ptr.  Written once (0 -> 1) with a
 * store-release by urcu_txn_sw_group_commit(); read with a load-acquire by
 * urcu_txn_sw_proxy_resolve().
 */
struct urcu_txn_sw_group {
	unsigned long selector;
};

/*
 * Records are 16-byte aligned so each tagged proxy has its low 4 bits free.
 */
struct urcu_txn_sw_record {
	void *ptr[2];			/* [0] old, [1] new; selector-indexed */
	struct urcu_txn_sw_group *group;
	void **slot;			/* install / settle target */
	uintptr_t tag;			/*
					 * Embedder tag bits OR'd into this
					 * slot's proxy value at install (see
					 * urcu_txn_sw_record).  Per-record so
					 * heterogeneous slots -- e.g. a
					 * 0xF-tagged structural edge and a
					 * bit-0 list edge -- can share one
					 * transaction.
					 */
} __attribute__((aligned(16)));

/*
 * Transaction descriptor: the single allocation that carries a committed
 * transaction across its grace period.  It holds the flip group every installed
 * proxy reads, the rcu_head that defers reclaim, and the record array inline
 * (the proxies themselves live there).  Allocated on the first
 * record()/reserve() from the shared per-CPU slab and grown in PREPARE (no
 * proxy is live yet, so it may move); delayed-reclaimed after settle or
 * reclaimed immediately on commit failure.  The 32-byte header keeps records[]
 * 16-byte aligned so each tagged proxy has its low 4 bits free.  @cap is the
 * physical capacity; @slab (stamped at alloc) is the descriptor's origin, so it
 * is freed on the path that allocated it even if the slab enables in between.
 * The inline path (urcu_txn_sw_init_inline, nr <= 1) never allocates a
 * descriptor.
 */
struct urcu_txn_sw_desc {
	struct urcu_txn_sw_group group;		/* read by installed proxies */
	/*
	 * Keep @rcu_head at this offset.  The slab threads its pending list
	 * through it and lays a closed batch's metadata just past it, so the
	 * smallest usable size class is
	 *
	 *   offsetof(rcu_head) + sizeof(struct rcu_head)
	 *			+ sizeof(struct urcu_slab_batch)
	 *
	 * which is 8 + 16 + 8 = 32 bytes.  Moving it later raises that floor,
	 * and the slab would disable itself without a word.  It cannot be first
	 * either: installed proxies point at @group.
	 */
	struct rcu_head rcu_head;		/* deferred-free handle */
	unsigned int cap;			/* physical capacity */
	unsigned int slab;			/*
						 * Descriptor origin: slab (1) /
						 * exact malloc (0); the free
						 * discriminator, doubling as
						 * the pad that keeps records[]
						 * 16-byte aligned.
						 */
	struct urcu_txn_sw_record records[];	/* frozen at install */
};

urcu_static_assert(!(offsetof(struct urcu_txn_sw_desc, records) % 16),
		"urcu_txn_sw_desc.records must be 16-byte aligned for proxy tagging",
		urcu_txn_sw_desc_records_aligned);

/*
 * Transaction handle: a small on-stack object with no allocation of its own.
 * The record array @records grows in PREPARE -- safe because no proxy is
 * installed yet (no slot points into it) -- and is frozen once commit()
 * installs proxies.  @desc is the transaction descriptor (NULL until the first
 * record()/reserve()); it carries @records inline, so a held proxy and its
 * selector are reclaimed together after a grace period.  The tag room that
 * matters is on the tagged proxies stored in slots; those live in @records,
 * allocated 16-byte aligned (posix_memalign) so each record -- hence each
 * tagged proxy -- keeps its low 4 bits free, letting a low-4-bit
 * pointer-tagging embedder (e.g. the fractal trie) route its slots through this
 * engine.
 */
struct urcu_txn_sw_txn {
	/*
	 * Ordered by access and packed: what every record() uses fits the
	 * first cache line, without holes.  The handle is private to one
	 * writer, so there is no false sharing to avoid.
	 */
	struct urcu_txn_sw_record *records;	/* grown, or caller storage */
	struct urcu_txn_sw_desc *desc;		/* NULL until first allocated */
	unsigned int nr;
	unsigned int cap;
	enum urcu_txn_sw_state state;
	bool records_inline;			/* @records is caller storage */
	bool disjoint;				/* declared slot-disjoint */
	bool bloom_live;			/* @ryw_bloom is armed */
#ifdef URCU_TXN_SW_EXCL_VALIDATE
	/* Thread that initialized the handle, and must drive it end to end. */
	pthread_t excl_owner;
#endif
	/*
	 * Last, after the conditional member, so that init clears everything
	 * before it in one memset and leaves these 128 bytes alone: the filter
	 * is armed lazily (urcu_txn_sw__bloom_arm).
	 */
	uint64_t ryw_bloom[URCU_TXN_BLOOM_WORDS];  /* RYW certain-miss filter */
};

static inline
void urcu_txn_sw_group_init(struct urcu_txn_sw_group *group)
{
	group->selector = 0;
}

/*
 * Does @v carry all of @tag's bits, i.e. is it an installed proxy rather than a
 * live value?  No live value an embedder stores in a transacted slot may carry
 * them: see urcu_txn_sw_record().
 */
static inline
int urcu_txn_sw_is_proxy(const void *v, uintptr_t tag)
{
	return ((uintptr_t) v & tag) == tag;
}

/*
 * Recover the proxy address from a parked slot value.
 *
 * Subtract the tag rather than masking it off.  The two are equivalent here:
 * urcu_txn_sw_is_proxy() has shown every tag bit set in @v, and those bits are
 * clear in the proxy address (records are 16-byte aligned, and the tag is
 * within the low 4 bits).
 *
 * The subtraction generates better code.  With a constant @tag the compiler
 * folds it into the displacement of the loads that follow, where a mask is one
 * more operation in the reader's chain of dependent loads (proxy -> group ->
 * selector -> ptr[sel]).
 */
static inline
struct urcu_txn_sw_record *urcu_txn_sw_untag(void *v, uintptr_t tag)
{
	urcu_assert_debug(urcu_txn_sw_is_proxy(v, tag));
	return (struct urcu_txn_sw_record *) ((uintptr_t) v - tag);
}

/*
 * Resolve a proxy to its current target.
 *
 * @proxy must have been obtained by dereferencing (rcu_dereference) the slot
 * that holds the tagged proxy pointer, so the dependency chain makes the
 * proxy's immutable fields (old_ptr, new_ptr, group) visible.  The selector is
 * the only mutable field: load it with acquire so the new target's contents --
 * published before urcu_txn_sw_group_commit()'s store-release -- are visible
 * whenever selector == 1 is observed.
 *
 * The selector indexes proxy->ptr[] directly, so resolution is a pure data
 * dependency rather than a conditional branch.
 */
static inline
void *urcu_txn_sw_proxy_resolve(const struct urcu_txn_sw_record *proxy)
{
	/*
	 * Load-acquire (A) pairs with the store-release (B) of
	 * urcu_txn_sw_group_commit().
	 */
	return proxy->ptr[uatomic_load(&proxy->group->selector, CMM_ACQUIRE)];
}

/*
 * Resolve a value loaded from a slot transacted under @tag: a plain value is
 * returned as is, an installed proxy resolves through its flip group's
 * selector.  The embedders' reader accessors are wrappers over this.
 */
static inline
void *urcu_txn_sw_resolve(void *v, uintptr_t tag)
{
	if (caa_likely(!urcu_txn_sw_is_proxy(v, tag)))
		return v;
	return urcu_txn_sw_proxy_resolve(urcu_txn_sw_untag(v, tag));
}

/*
 * Commit the flip group: switch every record in the transaction from old to new
 * with a single store-release.  Must be called after the new targets are fully
 * built.
 */
static inline
void urcu_txn_sw_group_commit(struct urcu_txn_sw_group *group)
{
	/*
	 * Store-release (B) pairs with the load-acquire (A) of
	 * urcu_txn_sw_proxy_resolve().
	 */
	uatomic_store(&group->selector, 1, CMM_RELEASE);
}

/*
 * URCU_TXN_SW_EXCL_VALIDATE: runtime validation of the single-writer contract.
 *
 * This engine requires writer mutual exclusion -- it has no install-time CAS,
 * no conflict detection and no abort, so two writers racing on one slot simply
 * corrupt it, and nothing in a default build says so.  Enable with
 * -DURCU_TXN_SW_EXCL_VALIDATE to have a violation abort the process with a
 * report identifying the violated handle or slot.  Off by default (zero
 * overhead: no checks, and the handle does not even carry the owner field).
 *
 * There is no per-structure object to claim an owner on, so the
 * validator claims the two things that do exist:
 *
 *   - the handle.  Its owner is the thread that initialized it, and
 *     reserve / record / install / commit must all run on that thread.
 *     Catches a transaction handed between threads mid-flight.
 *
 *   - the slot, which is what two racing writers actually share, and so is
 *     where the real violation is visible.  In a correct single-writer
 *     program a slot being recorded or parked cannot already hold an
 *     installed proxy.  A proxy sitting there therefore means another
 *     writer is mid-transaction on that very slot right now.
 *     Symmetrically, at settle each slot must still hold this writer's
 *     proxy; anything else means a concurrent writer overwrote it.
 */

#ifdef URCU_TXN_SW_EXCL_VALIDATE

#define urcu_txn_sw__excl_abort(...)					\
	do {								\
		fprintf(stderr, "urcu-txn-sw single-writer violation: "	\
			__VA_ARGS__);					\
		fflush(stderr);						\
		abort();						\
	} while (0)

/*
 * urcu_txn_sw_is_proxy() with tag == 0 excluded: under a zero tag the bare
 * predicate is true for every value, and the validator would abort on a clean
 * slot.
 */
static inline
int urcu_txn_sw__is_proxy(const void *v, uintptr_t tag)
{
	return tag != 0 && urcu_txn_sw_is_proxy(v, tag);
}

static inline
void urcu_txn_sw__excl_claim(struct urcu_txn_sw_txn *t)
{
	t->excl_owner = pthread_self();
}

static inline
void urcu_txn_sw__excl_owner(const struct urcu_txn_sw_txn *t, const char *what)
{
	pthread_t self = pthread_self();

	if (!pthread_equal(t->excl_owner, self))
		urcu_txn_sw__excl_abort("txn=%p: %s on a thread other than the one that initialized the handle -- one transaction is driven end to end by one thread\n",
			(const void *) t, what);
}

/* @slot must not already be parked by another writer. */
static inline
void urcu_txn_sw__excl_slot_free(void **slot, uintptr_t tag, const char *what)
{
	void *v = uatomic_load(slot, CMM_RELAXED);

	if (urcu_txn_sw__is_proxy(v, tag))
		urcu_txn_sw__excl_abort("slot=%p already holds an installed proxy (%p) at %s: another writer is mid-transaction on it\n",
			(void *) slot, v, what);
}

/* At settle, @r's slot must still hold the proxy this writer installed. */
static inline
void urcu_txn_sw__excl_slot_ours(struct urcu_txn_sw_record *r)
{
	void *want = (void *) ((uintptr_t) r | r->tag);
	void *v = uatomic_load(r->slot, CMM_RELAXED);

	if (v != want)
		urcu_txn_sw__excl_abort("slot=%p holds %p at settle, expected our installed proxy %p: a concurrent writer overwrote it\n",
			(void *) r->slot, v, want);
}

/*
 * The single-edge commit installs no proxy, so its only witness is the value:
 * @r's slot must still hold the recorded old.
 */
static inline
void urcu_txn_sw__excl_slot_unchanged(struct urcu_txn_sw_record *r)
{
	void *v = uatomic_load(r->slot, CMM_RELAXED);

	if (v != r->ptr[0])
		urcu_txn_sw__excl_abort("slot=%p holds %p at the single-edge commit, but %p was recorded as its old: a concurrent writer changed it\n",
			(void *) r->slot, v, r->ptr[0]);
}

/*
 * At the multi-edge install, @r's slot must hold no proxy and must still hold
 * the recorded old.
 *
 * The value check catches a racing writer that ran a complete transaction on
 * the slot -- install, flip, settle -- between this transaction's record() and
 * its install.  That writer leaves a plain value behind, so the slot looks
 * free; installing over it would publish a stale old to readers, then settle
 * over the other writer's committed value.
 *
 * A correct single-writer program never trips it: this transaction has not
 * written the slot yet, the writer's earlier transactions settled before
 * returning, and chaining keeps the committed old in ptr[0].
 */
static inline
void urcu_txn_sw__excl_slot_parkable(struct urcu_txn_sw_record *r)
{
	void *v = uatomic_load(r->slot, CMM_RELAXED);

	if (urcu_txn_sw__is_proxy(v, r->tag))
		urcu_txn_sw__excl_abort("slot=%p already holds an installed proxy (%p) at install: another writer is mid-transaction on it\n",
			(void *) r->slot, v);
	if (v != r->ptr[0])
		urcu_txn_sw__excl_abort("slot=%p holds %p at install, but %p was recorded as its old: a concurrent writer committed over it\n",
			(void *) r->slot, v, r->ptr[0]);
}

#else	/* !URCU_TXN_SW_EXCL_VALIDATE */

# define urcu_txn_sw__excl_claim(t)			do { } while (0)
# define urcu_txn_sw__excl_owner(t, what)		do { } while (0)
# define urcu_txn_sw__excl_slot_free(slot, tag, what)	do { } while (0)
# define urcu_txn_sw__excl_slot_parkable(r)		do { } while (0)
# define urcu_txn_sw__excl_slot_ours(r)			do { } while (0)
# define urcu_txn_sw__excl_slot_unchanged(r)		do { } while (0)

#endif	/* URCU_TXN_SW_EXCL_VALIDATE */

/*
 * Common body of the two public initializers.  @buf == NULL selects the
 * heap-growing form.
 *
 * Clears everything up to, and not including, @ryw_bloom: the filter is armed
 * lazily, and zeroing its 128 bytes per transaction would cost what the lazy
 * arming saves.  This is why @ryw_bloom is the last member.
 */
static inline
void urcu_txn_sw__init(struct urcu_txn_sw_txn *t,
		struct urcu_txn_sw_record *buf, unsigned int cap)
{
	memset(t, 0, offsetof(struct urcu_txn_sw_txn, ryw_bloom));
	t->state = URCU_TXN_SW_PREPARE;
	t->records = buf;
	t->cap = cap;
	t->records_inline = (buf != NULL);
	urcu_txn_sw__excl_claim(t);
}

/*
 * Initialize an on-stack transaction handle.  Allocates nothing, so it cannot
 * fail: the first record()/reserve() is the first OOM checkpoint, and that
 * failure is sticky (commit reports MEMORY_ERROR).
 */
static inline
void urcu_txn_sw_init(struct urcu_txn_sw_txn *t)
{
	urcu_txn_sw__init(t, NULL, 0);
}

/*
 * Initialize a transaction whose record array is caller-provided storage @buf,
 * holding @cap records.  struct urcu_txn_sw_record is aligned(16), so an
 * on-stack array of it keeps each tagged proxy's low 4 bits free.
 *
 * Allocates nothing, so it cannot fail.  record() never grows the array and
 * commit() never frees it.
 *
 * Use it only for a transaction of at most one record.  Such a commit installs
 * no proxy and owes no grace period, so nothing references the handle or @buf
 * once commit() returns.  A transaction that installs proxies (nr >= 2) must
 * use urcu_txn_sw_init(): its records have to outlive a grace period.  record()
 * and install() both assert it.
 */
static inline
void urcu_txn_sw_init_inline(struct urcu_txn_sw_txn *t,
		struct urcu_txn_sw_record *buf, unsigned int cap)
{
	urcu_txn_sw__init(t, buf, cap);
}

#define urcu_txn_sw_descsize(cap)				\
	(sizeof(struct urcu_txn_sw_desc) +			\
	 (size_t) (cap) * sizeof(struct urcu_txn_sw_record))

/*
 * Transaction descriptors come from the per-CPU size-classed slab of
 * <urcu/rcu-txn-slab.h>, the same machinery as the concurrent engine.  The
 * classes hold {4,8,16,32,64,128} records; a larger request is an exact,
 * uncached posix_memalign.  Each descriptor is stamped with its origin
 * (urcu_txn_sw_desc.slab), which the free path consults.  URCU_TXN_NO_CACHE
 * disables the slab.
 */
static const unsigned int urcu_txn_sw_slab_rc[] =
	{ 4u, 8u, 16u, 32u, 64u, 128u };
#define URCU_TXN_SW_SLAB_NCLASS	\
	((int) (sizeof(urcu_txn_sw_slab_rc) / sizeof(urcu_txn_sw_slab_rc[0])))

/*
 * The slab instance is defined once, in liburcu-common (src/urcu-txn.c), whose
 * constructor initializes it: this header requires linking liburcu-common.  A
 * header-static definition would give each translation unit its own arenas; see
 * <urcu/rcu-txn-mcas.h>.
 */
extern struct urcu_slab urcu_txn_sw_slab;

/* Smallest class that fits @req records, or -1 if none does. */
static inline
int urcu_txn_sw_slab_class_of(unsigned int req)
{
	int i;

	for (i = 0; i < URCU_TXN_SW_SLAB_NCLASS; i++)
		if (req <= urcu_txn_sw_slab_rc[i])
			return i;
	return -1;
}

/*
 * Allocate a transaction descriptor for at least @cap records, 16-byte aligned
 * so that every tagged proxy keeps its low 4 bits free.  It comes from the slab
 * when the request fits a class, and its capacity is then the class size;
 * otherwise from posix_memalign.  Initializes the flip group.  Returns NULL on
 * OOM.
 */
static inline
struct urcu_txn_sw_desc *urcu_txn_sw__desc_alloc(unsigned int cap)
{
	struct urcu_txn_sw_desc *desc;
	int cl;

	desc = NULL;
	if (urcu_slab_enabled(&urcu_txn_sw_slab) &&
			(cl = urcu_txn_sw_slab_class_of(cap)) >= 0) {
		desc = (struct urcu_txn_sw_desc *)
				urcu_slab_alloc(&urcu_txn_sw_slab, cl);
		if (caa_likely(desc != NULL)) {
			cap = urcu_txn_sw_slab_rc[cl];	/* physical class cap */
			desc->slab = 1;
		}
		/*
		 * NULL means the arena hit its footprint cap, or OOM: fall
		 * through to the exact allocator.  Superblocks are never
		 * unmapped, so the cap bounds what a burst makes permanent,
		 * and the burst still has to complete.
		 */
	}
	if (!desc) {
		void *p;

		if (posix_memalign(&p, 16, urcu_txn_sw_descsize(cap)))
			return NULL;
		desc = (struct urcu_txn_sw_desc *) p;
		desc->slab = 0;
	}
	urcu_txn_sw_group_init(&desc->group);
	desc->cap = cap;
	return desc;
}

/*
 * Free a descriptor where it was allocated: to its slab arena, or to malloc.
 * The origin stamp decides, not the slab's current state, so a descriptor
 * allocated before the slab constructor ran is not misrouted once the slab is
 * enabled.
 */
static inline
void urcu_txn_sw__desc_free(struct urcu_txn_sw_desc *desc)
{
	if (!desc)
		return;
	if (desc->slab)
		urcu_slab_free(desc);
	else
		free(desc);
}

/*
 * Move the handle's records to a new descriptor holding at least @cap of them.
 * PREPARE only: no proxy is installed yet, so the records may move.  There is
 * no aligned realloc: allocate a new descriptor, copy, free the old one.
 * Returns false on OOM, leaving the handle in the sticky URCU_TXN_SW_OOM state.
 */
static inline
bool urcu_txn_sw__resize(struct urcu_txn_sw_txn *t, unsigned int cap)
{
	struct urcu_txn_sw_desc *desc;

	desc = urcu_txn_sw__desc_alloc(cap);
	if (!desc) {
		t->state = URCU_TXN_SW_OOM;	/* sticky */
		return false;
	}
	if (t->nr)		/* @records is NULL until first allocated */
		memcpy(desc->records, t->records,
				(size_t) t->nr * sizeof(*desc->records));
	urcu_txn_sw__desc_free(t->desc);	/* NULL if it had none */
	t->desc = desc;
	t->records = desc->records;
	t->cap = desc->cap;
	/*
	 * The storage now belongs to the engine.  The flag can still be set
	 * here in an NDEBUG build, where the assert of urcu_txn_sw__grow() is
	 * compiled out and an oversized caller-storage handle migrates to the
	 * heap.  Clear it, or the new descriptor would never be freed.
	 */
	t->records_inline = false;
	return true;
}

/*
 * Pre-size the record array to hold at least @cap records.
 *
 * Optional: record() grows the array on demand and fails cleanly on OOM.  An
 * embedder whose record count is bounded can reserve that bound up front, where
 * a failure is easy to handle; every later record() then appends without
 * allocating and cannot fail.
 *
 * The descriptor comes from the smallest slab class that holds @cap.
 * URCU_TXN_SW_CAP is not a floor: it is only where an unreserved handle starts.
 * Reserving less than the transaction goes on to record is correct, and costs
 * the grow that a reserve exists to avoid.
 *
 * Call after init and before install.  A handle that already buffers records
 * grows to fit @cap.
 *
 * Returns false on OOM.  The failure is sticky, so the caller may ignore the
 * return and let commit() report MEMORY_ERROR.  It also returns false, without
 * becoming sticky, when a caller-storage (init_inline) handle is asked for more
 * than its buffer holds: that is a sizing bug, not a memory failure.
 */
static inline
bool urcu_txn_sw_reserve(struct urcu_txn_sw_txn *t, unsigned int cap)
{
	if (caa_unlikely(t->state == URCU_TXN_SW_OOM))
		return false;		/* sticky: an earlier alloc failed */
	/*
	 * PREPARE only, like record().  With proxies installed, the resize
	 * below would free the descriptor that live slots point into.
	 */
	urcu_posix_assert(t->state == URCU_TXN_SW_PREPARE);
	urcu_txn_sw__excl_owner(t, "reserve()");
	if (t->records && cap <= t->cap)
		return true;		/* already large enough */
	if (t->records_inline) {
		/* Fixed caller storage: a sizing bug, not an OOM. */
		urcu_assert_debug(!t->records_inline);
		return false;
	}
	return urcu_txn_sw__resize(t, cap);
}

/*
 * True when the next record() on @t appends into capacity the handle already
 * owns, and so cannot fail.  A reserve() covering the transaction's record
 * bound is the deliberate way to make this hold; an unreserved handle also
 * satisfies it after its first record(), until URCU_TXN_SW_CAP records are
 * used.
 *
 * For embedders that change state outside the transaction as they record, such
 * as <urcu/rcu-txn-sw-hlist.h>'s writer-only pprev, a plain store that cannot
 * be rolled back.  They are safe only while no later record of the same
 * transaction can fail.  A handle in URCU_TXN_SW_OOM answers false.
 */
static inline
bool urcu_txn_sw_append_is_infallible(const struct urcu_txn_sw_txn *t)
{
	return t->state == URCU_TXN_SW_PREPARE && t->nr < t->cap;
}

/*
 * Free the record array of a handle that never installed proxies (no grace
 * period).  Caller-owned (inline) storage is never freed by the engine.
 */
static inline
void urcu_txn_sw__free_records(struct urcu_txn_sw_txn *t)
{
	if (!t->records_inline)
		urcu_txn_sw__desc_free(t->desc);
	t->desc = NULL;
	t->records = NULL;
}

/*
 * call_rcu callback: free a committed transaction's descriptor once the grace
 * period guarantees no reader still holds one of its proxies.
 */
static inline
void urcu_txn_sw_free_rcu(struct rcu_head *head)
{
	struct urcu_txn_sw_desc *desc = caa_container_of(head,
			struct urcu_txn_sw_desc, rcu_head);

	urcu_txn_sw__desc_free(desc);
}

/*
 * Set a record's {old, new, slot, tag}; its group is bound at install.
 */
static inline
void urcu_txn_sw_record_set(struct urcu_txn_sw_record *r,
		void **slot, void *old_ptr, void *new_ptr, uintptr_t tag)
{
	r->ptr[0] = old_ptr;
	r->ptr[1] = new_ptr;
	r->slot = slot;
	r->tag = tag;
}

/*
 * Install record @r's tagged proxy into its slot.  Readers resolve it to old
 * until the commit.  The parked value is @r's address OR'd with the record's
 * tag; records are 16-byte aligned, so the address has its low 4 bits free.
 */
static inline
void urcu_txn_sw_record_install(struct urcu_txn_sw_txn *t,
		struct urcu_txn_sw_record *r)
{
	/*
	 * Installing requires a non-zero tag within the low 4 bits, none of
	 * which is set in the record's address.  Under a zero tag every plain
	 * value passes urcu_txn_sw_is_proxy(), and readers would resolve live
	 * values as proxies.  With a tag bit already set in the address, the OR
	 * changes nothing and urcu_txn_sw_untag() subtracts to a wrong address.
	 */
	urcu_assert_debug(r->tag != 0 && r->tag <= 0xf);
	urcu_assert_debug(!((uintptr_t) r & r->tag));
	r->group = &t->desc->group;	/* bind to the transaction's group */
	/* Store-release of slot pairs with rcu_dereference(). */
	uatomic_store(r->slot,
			(void *) ((uintptr_t) r | r->tag), CMM_RELEASE);
}

/*
 * Scan this transaction's records for @slot, or NULL.  Linear, so O(nr)
 * per call and O(nr^2) to build a write set -- urcu_txn_sw__find_ryw()
 * below is what keeps that off the hot path.  The returned pointer is
 * invalidated by the next record() (a grow may move the array), so use
 * it before recording again.
 */
static inline
struct urcu_txn_sw_record *urcu_txn_sw__scan(const struct urcu_txn_sw_txn *t,
		void **slot)
{
	unsigned int i;

	for (i = 0; i < t->nr; i++)
		if (t->records[i].slot == slot)
			return &t->records[i];
	return NULL;
}

/*
 * URCU_TXN_SW_BLOOM_MIN: the record count from which the read-your-own-writes
 * filter (<urcu/rcu-txn-bloom.h>) is used.  Below it the filter is never armed,
 * nor even zeroed.
 *
 * urcu_txn_sw_load() is only called on a slot about to be recorded, so loads
 * track records one for one, and most transactions are tiny: one edge for a
 * bitmap bit, two for a list operation.  Scanning that many records is cheaper
 * than hashing.
 *
 * The filter is built and armed by the first urcu_txn_sw__find_ryw() that faces
 * at least this many records, then kept current by every record().  Arming on
 * the record that crosses the threshold instead would make a transaction of
 * exactly URCU_TXN_SW_BLOOM_MIN records build a filter it never tests, which
 * was measured at about 8% of an 8-record commit.
 */
#ifndef URCU_TXN_SW_BLOOM_MIN
# define URCU_TXN_SW_BLOOM_MIN	8
#endif

/*
 * Build the filter from the records so far and arm it.  See
 * URCU_TXN_SW_BLOOM_MIN.
 */
static inline
void urcu_txn_sw__bloom_arm(struct urcu_txn_sw_txn *t)
{
	unsigned int i;

	memset(t->ryw_bloom, 0, sizeof(t->ryw_bloom));
	for (i = 0; i < t->nr; i++)
		urcu_txn_bloom_set(t->ryw_bloom, t->records[i].slot);
	t->bloom_live = true;
}

/*
 * urcu_txn_sw__scan() behind the filter: a clear bit means the slot is not
 * recorded, and the scan is skipped.  All k bits set falls through to the scan,
 * which settles a false positive.  The filter never changes the answer.
 */
static inline
struct urcu_txn_sw_record *urcu_txn_sw__find_ryw(struct urcu_txn_sw_txn *t,
		void **slot)
{
#ifndef URCU_TXN_SW_RYW_NO_BLOOM
	if (caa_unlikely(!t->bloom_live)) {
		/* Armed on first use: see URCU_TXN_SW_BLOOM_MIN. */
		if (t->nr < URCU_TXN_SW_BLOOM_MIN)
			return urcu_txn_sw__scan(t, slot);
		urcu_txn_sw__bloom_arm(t);
	}
	if (!urcu_txn_bloom_test(t->ryw_bloom, slot))
		return NULL;			/* definitely absent */
#endif
	return urcu_txn_sw__scan(t, slot);
}

/*
 * Grow a full record array: double it, or give an unreserved handle its first
 * URCU_TXN_SW_CAP records.  Returns false on OOM, leaving the handle in the
 * sticky URCU_TXN_SW_OOM state.
 *
 * Out of line on purpose: with this path in its body, the compiler does not
 * inline urcu_txn_sw_record(), and every recorded edge pays a call for an
 * append that is five stores.  `unused` because a translation unit that only
 * reads never records.
 */
static __attribute__((noinline, unused))
bool urcu_txn_sw__grow(struct urcu_txn_sw_txn *t)
{
	/*
	 * Caller-owned (inline) storage must never grow: overflowing it is an
	 * embedder sizing bug.
	 */
	urcu_posix_assert(!t->records_inline);
	return urcu_txn_sw__resize(t, t->cap ? t->cap * 2 : URCU_TXN_SW_CAP);
}

/*
 * Record one edge {*slot: old -> new}.  @tag is OR'd into the proxy value
 * installed in *slot, so that the slot's readers recognise the proxy and route
 * its resolution (e.g. the fractal trie's 0xF type code, or a list's bit 0).
 *
 * PREPARE only: the record set is frozen once proxies are installed, so this
 * must run before commit().  Nothing is installed here.
 *
 * Records must target pairwise-distinct slots.  record() appends without
 * looking for an earlier record of the same slot; recording a slot twice
 * installs two proxies on it and settles both in record order, so the earlier
 * edit is lost.  A debug build asserts against it at install.  Use
 * urcu_txn_sw_record_chain() when the same slot may be written twice.
 *
 * Returns false on OOM, the only failure.  It is sticky, so the caller may
 * ignore the return and let commit() report MEMORY_ERROR.
 */
static inline
bool urcu_txn_sw_record(struct urcu_txn_sw_txn *t, void **slot,
		void *old_ptr, void *new_ptr, uintptr_t tag)
{
	struct urcu_txn_sw_record *r;

	if (caa_unlikely(t->state == URCU_TXN_SW_OOM))
		return false;		/* sticky: an earlier alloc failed */
	urcu_posix_assert(t->state == URCU_TXN_SW_PREPARE);
	/*
	 * Neither value may look like a proxy under @tag.  After settle the
	 * slot holds @new_ptr directly: were it tagged, every later reader
	 * would resolve it as a proxy and dereference garbage.
	 */
	urcu_assert_debug(!urcu_txn_sw_is_proxy(old_ptr, tag));
	urcu_assert_debug(!urcu_txn_sw_is_proxy(new_ptr, tag));
	/*
	 * A caller-storage handle takes one record: no commit path accepts it
	 * with more.  Trap here, at the second record, rather than at the
	 * commit.
	 */
	urcu_posix_assert(!t->records_inline || t->nr == 0);
	urcu_txn_sw__excl_owner(t, "record()");
	urcu_txn_sw__excl_slot_free(slot, tag, "record()");
	if (caa_unlikely(t->nr == t->cap) && !urcu_txn_sw__grow(t))
		return false;			/* OOM, now sticky */
	r = &t->records[t->nr++];
	urcu_txn_sw_record_set(r, slot, old_ptr, new_ptr, tag);
	if (caa_unlikely(t->bloom_live))	/* armed: keep it current */
		urcu_txn_bloom_set(t->ryw_bloom, slot);
	return true;
}


/*
 * Read-your-own-writes pair (urcu_txn_sw_load / urcu_txn_sw_record_chain)
 * =======================================================================
 *
 * urcu_txn_sw_record() requires pairwise-distinct slots.  This suits an
 * embedder that knows its write sites, such as a list operation recording two
 * distinct edges: it pays no scan per store.
 *
 * An embedder whose write site is computed needs the opposite.  In a bitmap, 63
 * logical bits share one transacted word, so flips of distinct bits routinely
 * land on the same slot, and a same-slot store must fuse with the pending one
 * rather than duplicate it.  This pair does that:
 *
 *     old = urcu_txn_sw_load(t, slot, TAG);
 *     urcu_txn_sw_record_chain(t, slot, old, f(old), TAG);
 *
 * The transaction then keeps at most one record per slot.  The pair mirrors
 * urcu_txn_load() / urcu_txn_store() of <urcu/rcu-txn.h>, so an embedder
 * written against it can move to the concurrent engine.  There is no read set:
 * a load records nothing and is not validated, since a single writer has
 * nothing to validate against.
 *
 * Scope.  The pair fuses same-slot stores.  It does not make every composition
 * sound: an embedder whose new value is a neighbour's pointer (a list) can
 * still build its write set from stale reads of distinct slots, unless its
 * traversal also reads through urcu_txn_sw_load().  See
 * urcu_txn_sw_list_add_after_prepare().
 *
 * After a sticky OOM the pair no longer reads its own writes.  The failed
 * reserve()/record() appended nothing, so a later load() of that slot returns
 * the committed value, while slots recorded earlier still return pending ones.
 * commit() reports MEMORY_ERROR and publishes nothing, but an embedder that
 * changes state outside the transaction as it records (sw-hlist's writer-only
 * pprev) would derive those changes from a mixed view.  Such an embedder must
 * gate on urcu_txn_sw_append_is_infallible(), or check every record() return.
 */

/*
 * Declare this handle's write set slot-disjoint: no slot is recorded twice.
 * urcu_txn_sw_load() then returns the committed value without looking for a
 * pending one, and urcu_txn_sw_record_chain() appends without looking for an
 * earlier record.  The single-writer counterpart of
 * urcu_txn_declare_disjoint() (<urcu/rcu-txn.h>).
 *
 * Distinctness is a property of the slot address, not of a key.  A bitmap packs
 * 63 logical bits into one word, so distinct bit indexes are not distinct
 * slots.
 *
 * The lookup costs O(nr) per recorded edge, so O(n^2) for n edges.  Declaring
 * disjoint was measured as worth about nothing up to 8 edges, 1.6 times at 32
 * and 18 times at 800.  urcu_txn_sw_bitmap_set_range_prepare() alone in its
 * transaction is the typical case: it touches each word once.
 *
 * If a store does hit a recorded slot, two proxies are installed on it and
 * both new values are settled in record order: the earlier edit is lost and
 * commit() still reports OK.  A debug build traps the duplicate at install.
 * -DURCU_TXN_SW_DEBUG_DISJOINT traps earlier, at the offending record or load.
 *
 * Call after init and before the first record().
 */
static inline
void urcu_txn_sw_declare_disjoint(struct urcu_txn_sw_txn *t)
{
	urcu_posix_assert(t->state == URCU_TXN_SW_PREPARE);
	urcu_posix_assert(!t->nr);		/* before the first record */
	t->disjoint = true;
}

/*
 * Load @slot as this transaction will leave it: the pending new value of its
 * record for @slot if it has one, else the slot's current value.
 *
 * @tag is the slot's proxy tag.  A debug build checks that an unrecorded slot
 * holds no proxy: this writer owns every store to the slot, and its earlier
 * transactions settled before commit() returned.
 *
 * On a handle declared disjoint, this returns the committed value without
 * looking for a pending one.
 */
static inline
void *urcu_txn_sw_load(struct urcu_txn_sw_txn *t, void **slot,
		uintptr_t tag)
{
	struct urcu_txn_sw_record *r;
	void *v;

	urcu_txn_sw__excl_owner(t, "load()");
	urcu_posix_assert(t->state == URCU_TXN_SW_PREPARE ||
			t->state == URCU_TXN_SW_OOM);
	if (!t->disjoint && (r = urcu_txn_sw__find_ryw(t, slot)) != NULL)
		return r->ptr[1];		/* our own pending write */
#ifdef URCU_TXN_SW_DEBUG_DISJOINT
	/*
	 * The load side of the disjoint declaration.  On a disjoint handle a
	 * load of a recorded slot returns the committed value; the caller then
	 * computes its next edge from pre-transaction state, and since every
	 * slot it records is distinct, nothing else traps.
	 */
	if (t->disjoint && urcu_txn_sw__scan(t, slot) != NULL) {
		fprintf(stderr, "urcu-txn-sw: disjoint-contract violation: "
			"slot %p is read after this transaction recorded it, but the "
			"handle declared its write set disjoint via "
			"urcu_txn_sw_declare_disjoint(), so this load returns the "
			"COMMITTED value and not the pending one.  Anything computed "
			"from it names pre-transaction state.  Use the default (do not "
			"declare disjoint) for a composed bracket.\n", (void *) slot);
		abort();
	}
#endif
	v = uatomic_load(slot, CMM_RELAXED);
	/*
	 * A zero tag is legal for a transaction that installs no proxy, and
	 * every value would match it: skip the check.
	 */
	urcu_assert_debug(!tag || ((uintptr_t) v & tag) != tag);
	return v;
}

/*
 * The read-your-own-writes half of urcu_txn_sw_record_chain(): chain onto this
 * transaction's record for @slot, or append one if it has none.
 */
static inline
bool urcu_txn_sw__chain(struct urcu_txn_sw_txn *t, void **slot,
		void *old_ptr, void *new_ptr, uintptr_t tag)
{
	struct urcu_txn_sw_record *r;

	r = urcu_txn_sw__find_ryw(t, slot);
	if (!r)
		return urcu_txn_sw_record(t, slot, old_ptr, new_ptr, tag);
	urcu_assert_debug(r->tag == tag);
	/* The caller must have read its own write. */
	urcu_assert_debug(r->ptr[1] == old_ptr);
	r->ptr[1] = new_ptr;		/* committed old preserved */
	return true;
}

/*
 * Record edge {*slot: @old_ptr -> @new_ptr} tagged @tag, chaining onto this
 * transaction's record for @slot if it has one: that record keeps its
 * committed old and takes @new_ptr.  Otherwise append, as
 * urcu_txn_sw_record() does.  See the pair's contract above.
 *
 * @old_ptr must be what urcu_txn_sw_load() returned for @slot.  On the chain
 * path that is the record's pending value, and a different @old_ptr means the
 * caller computed @new_ptr from a stale read (debug assert).
 *
 * Returns false on OOM, which is sticky, as record() does.
 */
static inline
bool urcu_txn_sw_record_chain(struct urcu_txn_sw_txn *t, void **slot,
		void *old_ptr, void *new_ptr, uintptr_t tag)
{
	if (caa_unlikely(t->state == URCU_TXN_SW_OOM))
		return false;		/* sticky: an earlier alloc failed */
	urcu_posix_assert(t->state == URCU_TXN_SW_PREPARE);
	urcu_txn_sw__excl_owner(t, "record_chain()");
	if (t->disjoint) {		/* no slot repeats: skip the lookup */
#ifdef URCU_TXN_SW_DEBUG_DISJOINT
		if (urcu_txn_sw__scan(t, slot) != NULL) {
			fprintf(stderr, "urcu-txn-sw: disjoint-contract violation: "
				"slot %p is already recorded in this transaction, but the "
				"handle declared its write set disjoint via "
				"urcu_txn_sw_declare_disjoint().  The blind append would "
				"install a second proxy on it and settle both in record order, "
				"silently losing the earlier edit.  Use the default (do not "
				"declare disjoint) for a mutator whose composed edges can "
				"alias a slot.\n", (void *) slot);
			abort();
		}
#endif
		return urcu_txn_sw_record(t, slot, old_ptr, new_ptr, tag);
	}
	return urcu_txn_sw__chain(t, slot, old_ptr, new_ptr, tag);
}

/*
 * PREPARE -> INSTALLED: install every record's proxy into its slot.  Readers
 * still resolve to old, the selector being 0.  commit() calls this when
 * nr >= 2; a caller that needs to work between the install and the flip may
 * call it directly.  On OOM the handle becomes sticky and nothing is installed.
 */
static inline
void urcu_txn_sw_install(struct urcu_txn_sw_txn *t)
{
	unsigned int i;
	/*
	 * @nr does not change from here on.  Read it once: the installs store
	 * through record->slot, a void ** the compiler cannot prove distinct
	 * from the handle, so it would reload the field on every iteration.
	 */
	const unsigned int nr = t->nr;

	urcu_txn_sw__excl_owner(t, "install()");
	/*
	 * A caller-storage handle must never install proxies: its records do
	 * not outlive the call, and it has no descriptor to hold their group.
	 * Without this check, the branch below would give it one and install
	 * proxies through that descriptor's uninitialized slot pointers.
	 */
	urcu_posix_assert(!t->records_inline);
	if (caa_unlikely(!t->desc)) {	/* nothing recorded or reserved */
		/*
		 * A heap handle has no descriptor only if nothing was recorded
		 * or reserved.
		 */
		urcu_posix_assert(!nr);
		t->desc = urcu_txn_sw__desc_alloc(URCU_TXN_SW_CAP);
		if (caa_unlikely(!t->desc)) {
			t->state = URCU_TXN_SW_OOM;	/* sticky */
			return;
		}
		t->records = t->desc->records;
		t->cap = t->desc->cap;
	}
	/*
	 * Records must target pairwise-distinct slots: see
	 * urcu_txn_sw_record().  Debug-only check; the array is in record
	 * order, hence the pairwise scan.
	 */
	for (i = 1; i < nr; i++) {
		unsigned int j;

		for (j = 0; j < i; j++)
			urcu_assert_debug(t->records[i].slot !=
					t->records[j].slot);
	}
	t->state = URCU_TXN_SW_INSTALLED;
	for (i = 0; i < nr; i++) {
		urcu_txn_sw__excl_slot_parkable(&t->records[i]);
		urcu_txn_sw_record_install(t, &t->records[i]);
	}
}

/*
 * Commit: flip the group, so that every proxy resolves to new atomically, then
 * settle each slot to its new value.  commit() owns reclaim and consumes the
 * handle, which must be initialized again before it is reused.
 *
 * Returns MEMORY_ERROR if an allocation failed (sticky), otherwise OK.  A
 * single writer has no contention, so ABORT is never returned.
 *
 * With one record and no explicit urcu_txn_sw_install(), a lone store-release
 * to the slot is already an atomic commit.  No proxy is installed, so no reader
 * can hold one: no grace period is owed and the descriptor is freed at once.
 * The common one-pointer publish thus needs no install, settle or deferred
 * reclaim.  An empty transaction publishes nothing.
 *
 * With two or more records, or after an explicit install, commit() installs the
 * proxies if that is not done yet, flips, settles, then defers the descriptor's
 * reclaim: a reader may still hold a proxy.
 *
 * @call_rcu_fn defers that reclaim, and is called only when proxies were
 * installed.  It has the signature of a flavor's call_rcu(), so an embedder
 * that selects its flavor at runtime passes flavor->update_call_rcu, and this
 * header binds no flavor at compile time.  An embedder that knows no reader can
 * hold a proxy (single-threaded, or exclusive) may pass a function that calls
 * back synchronously.
 *
 * @flavor names the flavor whose readers may hold a proxy, so that the
 * descriptor can be retired in that flavor's batches (the default;
 * -DURCU_TXN_SLAB_NO_BATCH opts out).  It is used only when @call_rcu_fn is
 * @flavor->update_call_rcu, and may be NULL.
 */
static inline
enum urcu_txn_status urcu_txn_sw_commit_flavor(struct urcu_txn_sw_txn *t,
		void (*call_rcu_fn)(struct rcu_head *,
			void (*)(struct rcu_head *)),
		const struct rcu_flavor_struct *flavor)
{
	struct urcu_txn_sw_desc *desc;
	unsigned int i;
	/*
	 * Read once: the settle stores go through record->slot, so the compiler
	 * must assume each may change it.
	 */
	const unsigned int nr = t->nr;

	urcu_txn_sw__excl_owner(t, "commit()");
	/* A consumed handle must be initialized again before reuse. */
	urcu_posix_assert(t->state != URCU_TXN_SW_DONE);
	if (caa_unlikely(t->state == URCU_TXN_SW_OOM)) {
		urcu_txn_sw__free_records(t);
		t->state = URCU_TXN_SW_DONE;
		return URCU_TXN_STATUS_MEMORY_ERROR;
	}
	if (t->state == URCU_TXN_SW_PREPARE) {
		if (nr <= 1) {
			if (nr == 1) {
				struct urcu_txn_sw_record *r = &t->records[0];

				urcu_txn_sw__excl_slot_unchanged(r);
				/*
				 * Store-release of slot pairs with
				 * rcu_dereference().
				 */
				uatomic_store(r->slot, r->ptr[1],
						CMM_RELEASE);
			}
			/* nr == 0: empty txn, nothing published. */
			urcu_txn_sw__free_records(t);	/* no proxy: free now */
			t->state = URCU_TXN_SW_DONE;
			return URCU_TXN_STATUS_OK;
		}
		urcu_txn_sw_install(t);	/* installs the proxies */
		if (caa_unlikely(t->state == URCU_TXN_SW_OOM)) {
			urcu_txn_sw__free_records(t);
			t->state = URCU_TXN_SW_DONE;
			return URCU_TXN_STATUS_MEMORY_ERROR;
		}
	}

	/*
	 * INSTALLED: the proxies live in the descriptor, which is reclaimed
	 * after a grace period and so must be heap-owned.
	 */
	urcu_posix_assert(!t->records_inline);
	desc = t->desc;
	urcu_txn_sw_group_commit(&desc->group);
	for (i = 0; i < nr; i++) {
		struct urcu_txn_sw_record *r = &desc->records[i];

		urcu_txn_sw__excl_slot_ours(r);
		/* Store-release of slot pairs with rcu_dereference(). */
		uatomic_store(r->slot, r->ptr[1], CMM_RELEASE);
	}
	/*
	 * A reader may hold a proxy into @desc, so it must not be freed before
	 * a grace period.  By default the slab retires it in a batch (one
	 * call_rcu per batch); -DURCU_TXN_SLAB_NO_BATCH keeps one call_rcu per
	 * descriptor.  Only slab-stamped descriptors, under @flavor's own
	 * deferral, may take the batch route: see urcu_txn_retire() in
	 * <urcu/rcu-txn-mcas.h>.
	 */
#ifndef URCU_TXN_SLAB_NO_BATCH
	if (!(caa_likely(desc->slab && flavor &&
			flavor->update_call_rcu == call_rcu_fn) &&
			urcu_slab_free_pending(desc, flavor)))
		call_rcu_fn(&desc->rcu_head, urcu_txn_sw_free_rcu);
#else
	(void) flavor;
	call_rcu_fn(&desc->rcu_head, urcu_txn_sw_free_rcu);
#endif
	t->desc = NULL;		/* handle consumed */
	t->records = NULL;
	t->state = URCU_TXN_SW_DONE;
	return URCU_TXN_STATUS_OK;
}

/*
 * Commit, deferring reclaim through the call_rcu() of the RCU flavor selected
 * at compile time: this header must be included after an RCU flavor header.
 * See urcu_txn_sw_commit_flavor() for the contract.
 */
static inline
enum urcu_txn_status urcu_txn_sw_commit(struct urcu_txn_sw_txn *t)
{
	return urcu_txn_sw_commit_flavor(t, call_rcu, &rcu_flavor);
}

#ifdef __cplusplus
}
#endif

#endif /* _URCU_RCU_TXN_SW_H */
