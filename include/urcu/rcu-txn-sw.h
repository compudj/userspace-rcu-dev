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
 * urcu_txn_sw_commit() performs steps 2 to 4.
 *
 * Store-release / load-acquire pairs
 * ----------------------------------
 * What each of the writer's store-release orders, and against which load:
 *
 *   - Install.  The store-release of the tagged proxy into the slot pairs
 *     with the reader's rcu_dereference() of that slot.  It orders the
 *     writer's earlier stores to the transaction descriptor, which are the
 *     record's {old, new, group} and the selector's initialization to 0,
 *     before their respective loads by urcu_txn_sw__proxy_resolve().
 *
 *   - Commit and settle.  The store-release of the selector (0 -> 1) pairs
 *     with the load-acquire of the selector in urcu_txn_sw__proxy_resolve().
 *     The store-release of the new target into the slot pairs with the
 *     reader's rcu_dereference() of that slot.  Each orders the writer's
 *     earlier stores to the new data being published before their respective
 *     loads by the reader, which is the usual RCU publication guarantee.  A
 *     reader is covered by the first pair when its rcu_dereference() returned
 *     a tagged proxy, and by the second when it returned the new target.
 *
 * A single-record commit installs no proxy: it is the settle store alone.
 *
 * Ordering / monotonicity
 * ------------------------
 * The selector is written exactly once (0 -> 1) and never back, so a
 * reader resolving several proxy records over time observes a monotone
 * old...old,new...new sequence -- never new-then-old.  Embedders can rely
 * on this together with a fixed read order to avoid a per-traversal
 * snapshot (e.g. publishing the back edges of a re-parent through the
 * transaction *before* the forward edge, so a reader descending before it
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
#define URCU_TXN_SW_CAP	URCU_TXN_SLAB_MIN_CAP

/*
 * RCU pseudo-transaction (urcu_txn_sw_txn)
 * ========================================
 *
 * A growable transaction over a set of slots.  The handle is a small on-stack
 * object the embedder declares and initializes with urcu_txn_sw_init().
 * Nothing is allocated before the first record() or reserve(), so init cannot
 * fail.
 *
 *   init() --> PREPARE --commit() or cancel()--> DONE
 *                 |                               ^
 *                 | reserve() or record()         |
 *                 | cannot allocate               |
 *                 v                               |
 *                OOM ---commit() or cancel()------+
 *
 * record() appends a record {slot, old, new} to the transaction's descriptor
 * and installs nothing, so the descriptor may still move as it grows.  The
 * record set is frozen once commit() installs the proxies.  Records must target
 * pairwise-distinct slots: see urcu_txn_sw_record().
 *
 * A handle ends in commit(), which publishes the record set, or in cancel(),
 * which drops it.  Either consumes the handle, which must be initialized again
 * before it is reused.
 *
 * One thread drives a handle from init() to its end.  A handle that holds no
 * record yet may change hands once: see urcu_txn_sw_adopt().
 *
 * OOM is sticky.  Once a reserve() or a record() has failed to allocate, later
 * ones do nothing and commit() publishes nothing and reports MEMORY_ERROR.  An
 * embedder may therefore ignore their return values and test only commit()'s.
 *
 * Each record carries its own tag: the bits OR'd into the proxy value installed
 * in its slot, by which the slot's readers recognise a proxy (a reserved type
 * code, for instance).  It must be within URCU_TXN_SW_TAG_MASK.
 */

enum urcu_txn_sw_state {
	URCU_TXN_SW_PREPARE = 0,
	URCU_TXN_SW_OOM,		/* sticky: commit -> MEMORY_ERROR */
	/*
	 * Terminal: commit() or cancel() consumed the handle.  Using it again
	 * without initializing it trips an assertion.
	 */
	URCU_TXN_SW_DONE,
};

/*
 * Shared selector for a flip group.  selector == 0 -> records resolve to
 * old_ptr; selector == 1 -> new_ptr.  Written once (0 -> 1) with a
 * store-release by urcu_txn_sw__group_commit(); read with a load-acquire by
 * urcu_txn_sw__proxy_resolve().
 */
struct urcu_txn_sw_group {
	unsigned long selector;
};

/*
 * Records are aligned on URCU_TXN_SW_RECORD_ALIGN bytes, which leaves the low
 * bits of a record's address, URCU_TXN_SW_TAG_MASK, free for the tag that marks
 * an installed proxy.
 */
#define URCU_TXN_SW_RECORD_ALIGN	16
#define URCU_TXN_SW_TAG_MASK		(URCU_TXN_SW_RECORD_ALIGN - 1UL)

struct urcu_txn_sw_record {
	void *ptr[2];			/* [0] old, [1] new; selector-indexed */
	struct urcu_txn_sw_group *group;
	void **slot;			/* install / settle target */
	/*
	 * Tag bits OR'd into the proxy value installed in @slot.  Per record,
	 * so that slots tagged differently (a 0xF type code, a list's bit 0)
	 * can share one transaction.
	 */
	uintptr_t tag;
} __attribute__((aligned(URCU_TXN_SW_RECORD_ALIGN)));

/*
 * Transaction descriptor: the single allocation that carries a committed
 * transaction across its grace period.  It holds the flip group every installed
 * proxy reads, the rcu_head that defers its reclaim, and the records, which are
 * the proxies.  The first record() or reserve() allocates it, and it may move
 * as it grows in PREPARE, no proxy being installed yet.  A caller-storage
 * handle (urcu_txn_sw_init_inline()) has none.
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
	unsigned int cap;			/* records it can hold */
	/*
	 * Origin: 1 from the slab, 0 from posix_memalign().  The free path
	 * goes by it.  It also pads the header, so that @records keeps the
	 * alignment of a record.
	 */
	unsigned int slab;
	struct urcu_txn_sw_record records[];	/* frozen at install */
};

urcu_static_assert(!(offsetof(struct urcu_txn_sw_desc, records) %
			URCU_TXN_SW_RECORD_ALIGN),
		"urcu_txn_sw_desc.records must keep the alignment of a record",
		urcu_txn_sw_desc_records_aligned);

/*
 * Transaction handle: a small on-stack object, private to one writer.
 * @records points into @desc, or at caller storage for a handle initialized
 * with urcu_txn_sw_init_inline(), which has no descriptor.
 */
struct urcu_txn_sw_txn {
	/*
	 * Ordered by access and packed: what every record() uses fits the
	 * first cache line, without holes.
	 */
	struct urcu_txn_sw_record *records;	/* @desc's, or caller storage */
	struct urcu_txn_sw_desc *desc;		/* NULL until first allocated */
	unsigned int nr;
	unsigned int cap;
	enum urcu_txn_sw_state state;
	bool records_inline;			/* @records is caller storage */
	bool disjoint;				/* declared slot-disjoint */
	bool bloom_live;			/* @ryw_bloom is armed */
	/*
	 * After everything init clears, in one memset that leaves these 128
	 * bytes alone: the filter is armed lazily (urcu_txn_sw__bloom_arm).
	 */
	uint64_t ryw_bloom[URCU_TXN_BLOOM_WORDS];  /* RYW certain-miss filter */
#ifdef URCU_TXN_SW_EXCL_VALIDATE
	/*
	 * Thread that initialized the handle, or adopted it, and must drive it
	 * to its end.
	 * Last: liburcu-common is handed the handle (urcu_txn_sw__grow()) and
	 * may be built without it, so no other member's offset may depend on
	 * it.
	 */
	pthread_t excl_owner;
#endif
};

/*
 * Reader side
 * ===========
 *
 * A reader loads a transacted slot with rcu_dereference() and passes the value
 * to urcu_txn_sw_resolve().
 */

/*
 * Does @v carry all of @tag's bits, i.e. is it an installed proxy rather than a
 * live value?  No live value an embedder stores in a transacted slot may carry
 * them: see urcu_txn_sw_record().
 */
static inline
bool urcu_txn_sw_is_proxy(const void *v, uintptr_t tag)
{
	return ((uintptr_t) v & tag) == tag;
}

/*
 * Does @v, loaded from a slot transacted under @tag, show the slot parked?
 * Unlike urcu_txn_sw_is_proxy(), this answers false under a zero tag, which
 * every value matches and which marks no proxy.  A zero tag is legal for a
 * transaction that installs none.
 */
static inline
bool urcu_txn_sw__is_parked(const void *v, uintptr_t tag)
{
	return tag && urcu_txn_sw_is_proxy(v, tag);
}

/*
 * Recover the record's address from a parked slot value.
 *
 * Subtracting the tag is equivalent to masking it off, since
 * urcu_txn_sw_is_proxy() has shown every tag bit set in @v.  It lets the
 * compiler fold a constant @tag into the displacement of the loads that follow,
 * where a mask would be one more operation in the reader's chain of dependent
 * loads.
 */
static inline
struct urcu_txn_sw_record *urcu_txn_sw__untag(void *v, uintptr_t tag)
{
	urcu_assert_debug(urcu_txn_sw_is_proxy(v, tag));
	return (struct urcu_txn_sw_record *) ((uintptr_t) v - tag);
}

/*
 * Resolve a proxy to its current target.
 *
 * @proxy must come from an rcu_dereference() of the slot that holds it.  What
 * orders the loads below is described under "Store-release / load-acquire
 * pairs" at the top of this file.
 *
 * The selector indexes proxy->ptr[] directly, so resolution is a data
 * dependency rather than a conditional branch.
 */
static inline
void *urcu_txn_sw__proxy_resolve(const struct urcu_txn_sw_record *proxy)
{
	/*
	 * Load-acquire (A) pairs with the store-release (B) of
	 * urcu_txn_sw__group_commit().
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
	return urcu_txn_sw__proxy_resolve(urcu_txn_sw__untag(v, tag));
}

/*
 * Flip-group and record primitives
 * ================================
 *
 * What the inline paths need: the allocation of a descriptor, record(), and
 * the commit of a single record.  A record is installed and a group committed
 * in liburcu-common: see urcu_txn_sw__commit_desc().
 */

static inline
void urcu_txn_sw__group_init(struct urcu_txn_sw_group *group)
{
	group->selector = 0;
}

/*
 * Set a record's {old, new, slot, tag}; its group is bound at install.
 */
static inline
void urcu_txn_sw__record_set(struct urcu_txn_sw_record *r,
		void **slot, void *old_ptr, void *new_ptr, uintptr_t tag)
{
	r->ptr[0] = old_ptr;
	r->ptr[1] = new_ptr;
	r->slot = slot;
	r->tag = tag;
}

/*
 * Settle record @r's slot: store its new target there, over the proxy.  For a
 * transaction of one record, which installs no proxy, this store is the whole
 * commit.
 */
static inline
void urcu_txn_sw__record_settle(struct urcu_txn_sw_record *r)
{
	/* Store-release of slot pairs with rcu_dereference(). */
	uatomic_store(r->slot, r->ptr[1], CMM_RELEASE);
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
 *   - the handle.  Its owner is the thread that initialized it, or the one
 *     that adopted it before anything was recorded, and reserve / record /
 *     load / commit / cancel must all run on that thread.  Catches a
 *     transaction handed between threads mid-flight.
 *
 *   - the slot, which is what two racing writers actually share, and so is
 *     where the real violation is visible.  In a correct single-writer
 *     program a slot being recorded or parked cannot already hold an
 *     installed proxy.  A proxy sitting there therefore means another
 *     writer is mid-transaction on that very slot right now.
 *     Symmetrically, at settle each slot must still hold this writer's
 *     proxy; anything else means a concurrent writer overwrote it.
 *
 * The install and settle checks run in liburcu-common, and so only in a library
 * that was itself built with -DURCU_TXN_SW_EXCL_VALIDATE.  The others run in
 * the file that defines it.
 */

#ifdef URCU_TXN_SW_EXCL_VALIDATE

#define urcu_txn_sw__excl_abort(...)					\
	do {								\
		fprintf(stderr, "urcu-txn-sw single-writer violation: "	\
			__VA_ARGS__);					\
		fflush(stderr);						\
		abort();						\
	} while (0)

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

/* @t must hold no record: one that changes hands with some is mid-flight. */
static inline
void urcu_txn_sw__excl_adoptable(const struct urcu_txn_sw_txn *t)
{
	if (t->nr)
		urcu_txn_sw__excl_abort("txn=%p: adopt() of a handle that holds %u records -- a transaction does not change hands once it has recorded\n",
			(const void *) t, t->nr);
}

/* @slot must hold no proxy: it would be another writer's. */
static inline
void urcu_txn_sw__excl_slot_free(void **slot, uintptr_t tag, const char *what)
{
	void *v = uatomic_load(slot, CMM_RELAXED);

	if (urcu_txn_sw__is_parked(v, tag))
		urcu_txn_sw__excl_abort("slot=%p already holds an installed proxy (%p) at %s: another writer is mid-transaction on it\n",
			(void *) slot, v, what);
}

/*
 * @r's slot must still hold the recorded old.  This is the only witness of a
 * single-edge commit, which installs no proxy.
 */
static inline
void urcu_txn_sw__excl_slot_unchanged(struct urcu_txn_sw_record *r,
		const char *what)
{
	void *v = uatomic_load(r->slot, CMM_RELAXED);

	if (v != r->ptr[0])
		urcu_txn_sw__excl_abort("slot=%p holds %p at %s, but %p was recorded as its old: a concurrent writer changed it\n",
			(void *) r->slot, v, what, r->ptr[0]);
}

#else	/* !URCU_TXN_SW_EXCL_VALIDATE */

static inline
void urcu_txn_sw__excl_claim(struct urcu_txn_sw_txn *t __attribute__((unused)))
{
}

static inline
void urcu_txn_sw__excl_owner(
		const struct urcu_txn_sw_txn *t __attribute__((unused)),
		const char *what __attribute__((unused)))
{
}

static inline
void urcu_txn_sw__excl_adoptable(
		const struct urcu_txn_sw_txn *t __attribute__((unused)))
{
}

static inline
void urcu_txn_sw__excl_slot_free(void **slot __attribute__((unused)),
		uintptr_t tag __attribute__((unused)),
		const char *what __attribute__((unused)))
{
}

static inline
void urcu_txn_sw__excl_slot_unchanged(
		struct urcu_txn_sw_record *r __attribute__((unused)),
		const char *what __attribute__((unused)))
{
}

#endif	/* URCU_TXN_SW_EXCL_VALIDATE */

/*
 * Descriptor allocation and reclaim
 * =================================
 */

/* Size in bytes of a descriptor holding @cap records. */
static inline
size_t urcu_txn_sw__desc_size(unsigned int cap)
{
	return sizeof(struct urcu_txn_sw_desc) +
			(size_t) cap * sizeof(struct urcu_txn_sw_record);
}

/*
 * Transaction descriptors come from the per-CPU size-classed slab of
 * <urcu/rcu-txn-slab.h>, in the classes that header defines for both engines
 * (urcu_txn_slab_class_cap[]); a larger request is an exact, uncached
 * posix_memalign.  Each descriptor is stamped with its origin
 * (urcu_txn_sw_desc.slab), which the free path consults.  URCU_TXN_NO_CACHE
 * disables the slab.
 *
 * The slab instance is defined once, in liburcu-common (src/urcu-txn.c), whose
 * constructor initializes it: this header requires linking liburcu-common.  A
 * header-static definition would give each translation unit its own arenas; see
 * <urcu/rcu-txn-mcas.h>.
 */
extern struct urcu_slab urcu_txn_sw_slab;

/*
 * Allocate a transaction descriptor for at least @cap records.  It comes from
 * the slab when the request fits a class, and its capacity is then the
 * class's; otherwise from posix_memalign().  Initializes the flip group.
 * Returns NULL on OOM.
 */
static inline
struct urcu_txn_sw_desc *urcu_txn_sw__desc_alloc(unsigned int cap)
{
	struct urcu_txn_sw_desc *desc = NULL;
	int cl = -1;

	if (urcu_slab_enabled(&urcu_txn_sw_slab))
		cl = urcu_txn_slab_class_of(cap);
	if (cl >= 0) {
		desc = (struct urcu_txn_sw_desc *)
				urcu_slab_alloc(&urcu_txn_sw_slab, cl);
		if (caa_likely(desc != NULL)) {
			cap = urcu_txn_slab_class_cap[cl];
			desc->slab = 1;
		}
	}
	if (!desc) {
		void *p;

		/*
		 * No class holds @cap, or the arena hit its footprint cap.
		 * Superblocks are never unmapped, so the cap bounds what a
		 * burst makes permanent, and the burst still has to complete:
		 * serve it with an exact allocation.
		 */
		if (posix_memalign(&p, URCU_TXN_SW_RECORD_ALIGN,
				urcu_txn_sw__desc_size(cap)))
			return NULL;
		desc = (struct urcu_txn_sw_desc *) p;
		desc->slab = 0;
	}
	urcu_txn_sw__group_init(&desc->group);
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
 * Writer side
 * ===========
 *
 * The handle's contract is stated under "RCU pseudo-transaction" above.
 */

/*
 * Common body of the two public initializers.  @buf == NULL selects the
 * heap-growing form; otherwise @buf is caller storage for @cap records.
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
 * Initialize a transaction handle.  Allocates nothing, so it cannot fail.
 */
static inline
void urcu_txn_sw_init(struct urcu_txn_sw_txn *t)
{
	urcu_txn_sw__init(t, NULL, 0);
}

/*
 * Initialize a transaction of at most one record, whose record lives in caller
 * storage @rec.  Allocates nothing, so it cannot fail, and commit() never frees
 * @rec.
 *
 * Such a commit installs no proxy and owes no grace period, so nothing
 * references the handle or @rec once commit() returns.  A transaction that
 * installs proxies (nr >= 2) must use urcu_txn_sw_init(): its records have to
 * outlive a grace period.  A second record() asserts it.
 */
static inline
void urcu_txn_sw_init_inline(struct urcu_txn_sw_txn *t,
		struct urcu_txn_sw_record *rec)
{
	urcu_txn_sw__init(t, rec, 1);
}

/*
 * Move the handle's records to a new descriptor holding at least @cap of them.
 * PREPARE only: no proxy is installed yet, so the records may move.  There is
 * no aligned realloc: allocate a new descriptor, copy, free the old one.
 * Returns false on OOM, leaving the handle in the URCU_TXN_SW_OOM state.
 */
static inline
bool urcu_txn_sw__resize(struct urcu_txn_sw_txn *t, unsigned int cap)
{
	struct urcu_txn_sw_desc *desc;

	/*
	 * Caller storage is never outgrown: needing more is the embedder's
	 * sizing bug.  An NDEBUG build lets it through, and the records move to
	 * a descriptor.
	 */
	urcu_posix_assert(!t->records_inline);
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
	t->records_inline = false;	/* now the engine's to free */
	return true;
}

/*
 * Give the handle a descriptor that holds at least @cap records.
 *
 * Optional: record() grows the descriptor on demand.  An embedder whose record
 * count is bounded can reserve that bound up front, where a failure is easy to
 * handle; every later record() then appends without allocating and cannot
 * fail.
 *
 * The descriptor comes from the smallest slab class that holds @cap.
 * URCU_TXN_SW_CAP is not a floor: it is only where an unreserved handle starts.
 * Reserving less than the transaction goes on to record is correct, and costs
 * the grow that a reserve exists to avoid.
 *
 * Call after init and before commit.  A handle that already buffers records
 * grows to fit @cap.
 *
 * Returns false on OOM, which is sticky.  It also returns false, without
 * becoming sticky, when a caller-storage handle is asked for more than its one
 * record: that is a sizing bug, not a memory failure.
 */
static inline
bool urcu_txn_sw_reserve(struct urcu_txn_sw_txn *t, unsigned int cap)
{
	if (caa_unlikely(t->state == URCU_TXN_SW_OOM))
		return false;		/* sticky */
	/* PREPARE only, like record(): commit() consumed the handle. */
	urcu_posix_assert(t->state == URCU_TXN_SW_PREPARE);
	urcu_txn_sw__excl_owner(t, "reserve()");
	if (t->records && cap <= t->cap)
		return true;		/* already large enough */
	/* Fixed caller storage: a sizing bug, not an OOM. */
	urcu_assert_debug(!t->records_inline);
	if (t->records_inline)
		return false;
	return urcu_txn_sw__resize(t, cap);
}

/*
 * Take over a handle that another thread got ready.
 *
 * One thread drives a transaction end to end.  An embedder may still get a
 * handle ready on one thread, where a failure is easy to handle, with init()
 * and reserve(), and have another record and commit, or cancel.  The thread
 * that takes over calls this first, on a handle that holds no record yet.  The
 * embedder makes sure that the thread which got the handle ready is done with
 * it.
 *
 * It changes which thread URCU_TXN_SW_EXCL_VALIDATE expects, and nothing else.
 */
static inline
void urcu_txn_sw_adopt(struct urcu_txn_sw_txn *t)
{
	urcu_txn_sw__excl_adoptable(t);
	urcu_txn_sw__excl_claim(t);
}

/*
 * True when the next record() on @t appends into capacity the handle already
 * owns, and so cannot fail.  A reserve() covering the transaction's record
 * bound is the deliberate way to make this hold; an unreserved handle also
 * satisfies it after its first record(), until URCU_TXN_SW_CAP records are
 * used.  A handle in the OOM state answers false.
 *
 * This is for embedders that change state outside the transaction as they
 * record, with a plain store that cannot be rolled back, such as the
 * writer-only pprev of <urcu/rcu-txn-sw-hlist.h>.  They are safe only while no
 * later record of the same transaction can fail.
 */
static inline
bool urcu_txn_sw_append_is_infallible(const struct urcu_txn_sw_txn *t)
{
	return t->state == URCU_TXN_SW_PREPARE && t->nr < t->cap;
}

/*
 * Release the handle's descriptor.  No proxy of it was installed, so no grace
 * period is owed.  Caller storage is not the engine's to free.
 */
static inline
void urcu_txn_sw__release_desc(struct urcu_txn_sw_txn *t)
{
	if (!t->records_inline)
		urcu_txn_sw__desc_free(t->desc);
}

/*
 * Mark the handle consumed, by commit() or by cancel(): it no longer owns a
 * descriptor, and must be initialized again before it is reused.
 */
static inline
void urcu_txn_sw__done(struct urcu_txn_sw_txn *t)
{
	t->desc = NULL;
	t->records = NULL;
	t->state = URCU_TXN_SW_DONE;
}

/*
 * Scan this transaction's records for @slot, or NULL.  Linear, so O(nr)
 * per call and O(nr^2) to build a write set -- urcu_txn_sw__find_ryw()
 * below is what keeps that off the hot path.  The returned pointer is
 * invalidated by the next record() (a grow may move the records), so use
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
 * which rules out a false positive.  The filter never changes the answer.
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
 * Grow a full handle: double its capacity, or give an unreserved handle its
 * first URCU_TXN_SW_CAP records.  Returns false on OOM.
 *
 * In liburcu-common: with this path in its body, the compiler does not inline
 * urcu_txn_sw_record(), and every recorded edge pays a call for an append that
 * is five stores.
 */
extern bool urcu_txn_sw__grow(struct urcu_txn_sw_txn *t);

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
 * edit is lost.  A debug build of liburcu asserts against it at install.  Use
 * urcu_txn_sw_record_chain() when the same slot may be written twice.
 *
 * Returns false on OOM, the only failure, which is sticky.
 */
static inline
bool urcu_txn_sw_record(struct urcu_txn_sw_txn *t, void **slot,
		void *old_ptr, void *new_ptr, uintptr_t tag)
{
	struct urcu_txn_sw_record *r;

	if (caa_unlikely(t->state == URCU_TXN_SW_OOM))
		return false;		/* sticky */
	urcu_posix_assert(t->state == URCU_TXN_SW_PREPARE);
	/*
	 * Neither value may look like a proxy under @tag.  After settle the
	 * slot holds @new_ptr directly: were it tagged, every later reader
	 * would resolve it as a proxy and dereference garbage.
	 */
	urcu_assert_debug(!urcu_txn_sw__is_parked(old_ptr, tag));
	urcu_assert_debug(!urcu_txn_sw__is_parked(new_ptr, tag));
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
	urcu_txn_sw__record_set(r, slot, old_ptr, new_ptr, tag);
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
 * After an OOM the pair no longer reads its own writes: the failed record()
 * appended nothing, so a later load() of that slot returns the committed value.
 * commit() then publishes nothing, but an embedder that changes state outside
 * the transaction as it records must not derive it from such a load: see
 * urcu_txn_sw_append_is_infallible().
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
 * This skips a lookup among the records so far on every load and record, which
 * pays off only past a few dozen edges.  urcu_txn_sw_bitmap_set_range_prepare()
 * alone in its transaction is the typical case: it touches each word once.
 *
 * Recording a slot twice on such a handle breaks the rule of
 * urcu_txn_sw_record(), with the consequence stated there.
 * -DURCU_TXN_SW_DEBUG_DISJOINT traps it at the offending record or load.
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
 * URCU_TXN_SW_DEBUG_DISJOINT: on a handle declared disjoint, abort the process
 * when a load or a record names a slot the transaction has already recorded.
 * The load would return the committed value where its caller expects the
 * pending one, and the record would install a second proxy on the slot.  Off by
 * default.
 */
#ifdef URCU_TXN_SW_DEBUG_DISJOINT

static inline
void urcu_txn_sw__disjoint_check(const struct urcu_txn_sw_txn *t, void **slot,
		const char *what)
{
	if (!t->disjoint || !urcu_txn_sw__scan(t, slot))
		return;
	fprintf(stderr, "urcu-txn-sw disjoint violation: slot=%p is already recorded at %s, on a handle declared disjoint\n",
		(void *) slot, what);
	fflush(stderr);
	abort();
}

#else	/* !URCU_TXN_SW_DEBUG_DISJOINT */

static inline
void urcu_txn_sw__disjoint_check(
		const struct urcu_txn_sw_txn *t __attribute__((unused)),
		void **slot __attribute__((unused)),
		const char *what __attribute__((unused)))
{
}

#endif	/* URCU_TXN_SW_DEBUG_DISJOINT */

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
	if (!t->disjoint) {
		r = urcu_txn_sw__find_ryw(t, slot);
		if (r)
			return r->ptr[1];	/* our own pending write */
	}
	urcu_txn_sw__disjoint_check(t, slot, "load()");
	v = uatomic_load(slot, CMM_RELAXED);
	urcu_assert_debug(!urcu_txn_sw__is_parked(v, tag));
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
		return false;		/* sticky */
	urcu_posix_assert(t->state == URCU_TXN_SW_PREPARE);
	urcu_txn_sw__excl_owner(t, "record_chain()");
	if (t->disjoint) {		/* no slot repeats: skip the lookup */
		urcu_txn_sw__disjoint_check(t, slot, "record_chain()");
		return urcu_txn_sw_record(t, slot, old_ptr, new_ptr, tag);
	}
	return urcu_txn_sw__chain(t, slot, old_ptr, new_ptr, tag);
}

/*
 * Cancel: drop the transaction instead of committing it.  No proxy was
 * installed, so nothing is published and the descriptor is freed at once, with
 * no grace period.  Like commit(), it consumes the handle, which must be
 * initialized again before it is reused.
 *
 * A handle ends in commit() or in cancel(): one that is merely dropped leaks
 * its descriptor.  A handle in the OOM state may be cancelled as well.
 *
 * Only the records are dropped.  What an embedder changed outside the
 * transaction as it recorded is not rolled back, so such a transaction must be
 * committed: see <urcu/rcu-txn-sw-hlist.h>.
 */
static inline
void urcu_txn_sw_cancel(struct urcu_txn_sw_txn *t)
{
	urcu_txn_sw__excl_owner(t, "cancel()");
	/* A consumed handle must be initialized again before reuse. */
	urcu_posix_assert(t->state != URCU_TXN_SW_DONE);
	urcu_txn_sw__release_desc(t);	/* no proxy: free now */
	urcu_txn_sw__done(t);
}

/*
 * Install, flip, settle and retire the first @nr records of @desc: the commit
 * of a transaction that installs proxies, which takes two records or more.
 *
 * In liburcu-common.  URCU_TXN_SW_EXCL_VALIDATE, DEBUG_RCU and
 * URCU_TXN_SLAB_NO_BATCH therefore apply to it as the library was built, not as
 * the calling file is.  See urcu_txn_sw_commit_flavor() for @call_rcu_fn and
 * @flavor.
 */
extern void urcu_txn_sw__commit_desc(struct urcu_txn_sw_desc *desc,
		unsigned int nr,
		void (*call_rcu_fn)(struct rcu_head *,
			void (*)(struct rcu_head *)),
		const struct rcu_flavor_struct *flavor);

/*
 * Commit: install every record's proxy, flip the group so that every proxy
 * resolves to new atomically, then settle each slot to its new value.  commit()
 * owns reclaim and consumes the handle, which must be initialized again before
 * it is reused.
 *
 * Returns MEMORY_ERROR if an allocation failed (sticky), otherwise OK.  A
 * single writer has no contention, so ABORT is never returned.
 *
 * With one record, a lone store-release to the slot is already an atomic
 * commit.  No proxy is installed, so no reader can hold one: no grace period is
 * owed and the descriptor is freed at once.  The common one-pointer publish
 * thus needs no install, settle or deferred reclaim.  An empty transaction
 * publishes nothing.
 *
 * With two or more records, urcu_txn_sw__commit_desc() does the work, and the
 * descriptor's reclaim is deferred: a reader may still hold a proxy.
 *
 * @call_rcu_fn defers that reclaim, and is called only with two or more
 * records.  It has the signature of a flavor's call_rcu(), so an embedder
 * that selects its flavor at runtime passes flavor->update_call_rcu, and this
 * header binds no flavor at compile time.  An embedder that knows no reader can
 * hold a proxy (single-threaded, or exclusive) may pass a function that calls
 * back synchronously.
 *
 * @flavor names the flavor whose readers may hold a proxy, so that the
 * descriptor can be retired in that flavor's batches.  It is used only when
 * @call_rcu_fn is @flavor->update_call_rcu, and may be NULL.
 */
static inline
enum urcu_txn_status urcu_txn_sw_commit_flavor(struct urcu_txn_sw_txn *t,
		void (*call_rcu_fn)(struct rcu_head *,
			void (*)(struct rcu_head *)),
		const struct rcu_flavor_struct *flavor)
{
	const unsigned int nr = t->nr;

	urcu_txn_sw__excl_owner(t, "commit()");
	/* A consumed handle must be initialized again before reuse. */
	urcu_posix_assert(t->state != URCU_TXN_SW_DONE);
	if (caa_unlikely(t->state == URCU_TXN_SW_OOM)) {
		urcu_txn_sw_cancel(t);
		return URCU_TXN_STATUS_MEMORY_ERROR;
	}
	if (nr <= 1) {
		if (nr == 1) {
			/* One record: its settle store is the whole commit. */
			urcu_txn_sw__excl_slot_unchanged(&t->records[0],
					"the single-edge commit");
			urcu_txn_sw__record_settle(&t->records[0]);
		}
		urcu_txn_sw__release_desc(t);	/* no proxy: free now */
	} else {
		/*
		 * The proxies live in the descriptor, which is reclaimed after
		 * a grace period: a caller-storage handle has none.
		 */
		urcu_posix_assert(!t->records_inline);
		urcu_txn_sw__commit_desc(t->desc, nr, call_rcu_fn, flavor);
	}
	urcu_txn_sw__done(t);
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
