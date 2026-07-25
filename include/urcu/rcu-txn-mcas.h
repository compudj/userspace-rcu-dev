// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

#ifndef _URCU_RCU_TXN_MCAS_H
#define _URCU_RCU_TXN_MCAS_H

/*
 * RCU mixed single-writer / multi-writer transaction MCAS.
 *
 * This is the flavor-free multi-slot atomic-commit primitive: the unified
 * descriptor (a record set sharing one control word) and its resolve / install /
 * commit protocol, plus the descriptor's per-CPU slab.  It is a practical MCAS
 * generalized so a slot installs either by CAS (MW) or by a caller-exclusive
 * park (SW) -- the atomicity guarantee (a set of slots switched as one against
 * the control word) is the same either way.  The begin/load/store/commit
 * bracket with aging escalation and the fair-mutex fallback lane lives in the
 * front-end <urcu/rcu-txn.h> (which includes this header).
 *
 * One transaction commit can carry BOTH single-writer (SW) and multi-writer
 * (MW) records, committed atomically against ONE linearization point.  This
 * unifies two install disciplines:
 *
 *   - <urcu/rcu-txn-sw.h> (SW): a group selector flipped once by the sole
 *     writer, records PARKED with a plain store (the caller holds a lock over
 *     every recorded slot, so the park cannot be raced and cannot fail).
 *   - MW (a practical k-CAS): records resolved through a status word, PLANTED
 *     with a sole-driver CAS -- a concurrent writer that changed the slot fails
 *     the CAS, so the whole transaction aborts.
 *
 * A structural edit that must atomically touch a single-writer-owned slot AND a
 * multi-writer slot had no single commit spanning both in those engines; it
 * bridged them with an external lock.  This engine removes that bridge.
 *
 * The mechanism -- one control word, two record kinds
 * ---------------------------------------------------
 * A transaction owns one status word (the group / control word).  EVERY record,
 * SW or MW, shares the same RESOLVE HEADER {old_ptr, new_ptr, desc} and is
 * resolved identically by a reader -- return new_ptr if the status is
 * SUCCEEDED, else old_ptr -- regardless of kind.  Because the two kinds share
 * the resolve header, a tagged slot resolves uniformly no matter which kind
 * parked it: there is no ambiguous-tag / wrong-resolver hazard from mixing (the
 * historic "an SW proxy and an MW descriptor collide on one bit-0 tag" problem
 * was two INCOMPATIBLE layouts at one tag; here they are one layout).  The kinds
 * differ ONLY on the writer side (install and settle), never on resolve.
 *
 * Commit is single-driver, no helping.  The
 * owner drives install / decide / settle end to end; a contended MW slot's proxy
 * is a spinlatch a peer waits out (bounded) or escalates past, never one it
 * drives.  Because no helper competes on the control word, the commit point is a
 * single owner-exclusive RELEASE STORE of the status -- identical in shape to
 * the SW selector flip.
 *
 * Lifecycle
 *   create(cap, retry)                  one descriptor, status = UNDECIDED
 *   add(slot, old, new, tag, kind)      per slot, SW or MW
 *   desc_commit:
 *       install MW records (CAS-old, may abort), then SW records (plain store)
 *       if aborted: settle the parked prefix back to old; retry
 *       else: RELEASE-store status = SUCCEEDED   -- THE linearization point
 *             settle: write the direct new into every slot
 *       defer-free after a grace period
 *
 * Install order.  MW records install FIRST: an MW abort then wastes zero SW
 * parks (none are placed yet).  Nothing is decided until every record is
 * installed, so until the status store every record still resolves to old, for
 * SW and MW slots alike; the single store flips them all to new at once.
 *
 * Abort is MW-only.  An abort arises solely from an MW install CAS-old mismatch
 * (a concurrent writer committed a change).  SW parks never fail, so a
 * transaction with NO MW records NEVER aborts -- pure single-writer edits stay
 * on the cheap, abort-free path.
 *
 * Two commit entry points.  urcu_txn_desc_commit() is the sw-mw-aware
 * path: it partitions MW ahead of SW, sorts and CAS-installs the MW records, and
 * handles the abort.  An embedder that KNOWS a descriptor carries no MW records
 * commits it through urcu_txn_desc_commit_sw() instead, which skips all of
 * that (park, flip, settle) and cannot contention-abort.  So the sw-mw-aware
 * commit is paid for only when MW records are actually present.
 *
 * Correctness invariants
 *   1. ONE linearization point per txn -- the single status store.  All records
 *      (both kinds) resolve against it, so the commit is atomic across kinds.
 *   2. SW slots require CALLER exclusion -- the embedder holds a lock over every
 *      SW slot from before install through settle (single-writer, no CAS).  MW
 *      slots need no such lock (CAS-old + spinlatch handle concurrency).
 *   3. Abort is MW-only.  SW parks never fail.
 *   4. No helping.  The owner drives install/commit/settle end to end; peers on
 *      an MW slot wait on its proxy (bounded-blocking) or observe its status.
 *   5. Resolve is kind-agnostic and never drives.  A reader only reads
 *      {old, new, *status} -- so mixing kinds in one structure is safe by
 *      construction.
 *
 * Constraints:
 *   - the engine owns tag bit 0 by default, so every value stored in a
 *     transacted slot must be at least 2-byte aligned (bit 0 clear); an embedder
 *     may use a wider per-record tag (see the proxy tag scheme);
 *   - the record set is frozen at commit;
 *   - a transaction's records must target pairwise-distinct slots.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <urcu/assert.h>
#include <urcu/compiler.h>
#include <urcu/uatomic.h>
#include <urcu/call-rcu.h>		/* struct rcu_head */
#include <urcu/arch.h>			/* caa_cpu_relax */
#include <urcu/rcu-txn-slab.h>		/* shared per-CPU size-classed descriptor slab */

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Instrumentation hook -- compiles to nothing unless the embedder defines
 * URCU_TXN_STAT(counter) before including this header.
 */
#ifndef URCU_TXN_STAT
#define URCU_TXN_STAT(counter)	do { } while (0)
#endif

/*
 * Single-edge escalation threshold: a lone MW record commits with a bare CAS and
 * no descriptor until it has retried this many times, after which it commits
 * through the full descriptor protocol so it can hold the slot latched against
 * contenders.
 */
#ifndef URCU_TXN_ESCALATE
#define URCU_TXN_ESCALATE 16
#endif

/*
 * Spins on a blocker's status before an installer escalates (aborts and retries
 * at a higher aging priority).
 */
#ifndef URCU_TXN_WAIT_PATIENCE
#define URCU_TXN_WAIT_PATIENCE 8192
#endif

enum urcu_txn_desc_status {
	URCU_TXN_DESC_UNDECIDED = 0,
	URCU_TXN_DESC_SUCCEEDED = 1,
	URCU_TXN_DESC_FAILED    = 2,
};

/*
 * Record kind: it selects the INSTALL discipline only (SW = plain caller-
 * exclusive store, never fails; MW = sole-driver CAS-old, may abort).  Resolve
 * is identical for both.
 *
 * Kind is a property of the SLOT, not a free per-record choice.  A slot is
 * SW (the embedder holds a lock over it, so no writer races the plain park) XOR
 * MW (concurrent writers, so the CAS-old serializes them) -- the two are
 * mutually-exclusive concurrency disciplines of the slot itself.  Two facts are
 * asymmetric:
 *
 *   - MW is ALWAYS SAFE: a CAS-old install is correct whether or not anyone
 *     else touches the slot.
 *   - SW is a PROMISE of exclusion: the plain park is correct only while the
 *     caller's lock actually excludes every other writer of that slot.  It is
 *     the optimization to opt into when that promise holds.
 *
 * So the kind of a slot must be GLOBALLY CONSISTENT: every transaction that
 * records that slot -- across threads -- must use the same kind.  Treating one
 * slot as SW in one txn and MW in another lets the MW writer's CAS race the SW
 * writer's unlocked plain park (the MW path never takes the SW lock), which
 * tears or loses a write, possibly a use-after-free.  The engine CANNOT check
 * this: each txn sees only its own records.  It is the embedder's invariant, as
 * "the same datum is guarded by the same lock everywhere" is.  A QUIESCENT
 * temporal transition (a slot MW for one phase, SW for another, with a grace
 * period / global barrier between so no two concurrent writers disagree) is
 * fine; concurrent disagreement is not.  When in doubt, use MW.
 *
 * Recording BOTH kinds on one slot WITHIN one txn is the same contradiction in
 * miniature; urcu_txn_record_chain() resolves it fail-safe (MW dominates) and
 * debug builds trap the likely bug.
 */
enum urcu_txn_kind {
	URCU_TXN_KIND_SW = 0,
	URCU_TXN_KIND_MW = 1,
};

struct urcu_txn_desc;

struct urcu_txn_record {
	void **slot;			/* transacted word (bit 0 must be free) */
	void *old_ptr;			/* expected old value (resolve: status != SUCCEEDED) */
	void *new_ptr;			/* committed new value (resolve: status == SUCCEEDED) */
	uintptr_t proxy_tag;		/*
					 * Embedder's tag bits for THIS record's
					 * slot.  The parked proxy value is
					 * (&record | proxy_tag).  Carried per
					 * record so heterogeneous slots share
					 * one engine, each resolved through its
					 * own tag.
					 */
	struct urcu_txn_desc *desc;	/* back-pointer: shared status word */
	unsigned int kind;		/* enum urcu_txn_kind (writer-side only) */
} __attribute__((aligned(16)));

/*
 * The records are stored inline after the header; a parked slot holds the tagged
 * address of one record.  Each inline record is 16-byte aligned (the recs[]
 * offset and the record stride are both multiples of 16), so every inline record
 * address has its low 4 bits free for an embedder's wider tag.  The descriptor
 * is allocated with posix_memalign(16), so the 16-byte base alignment holds
 * portably.
 */
struct urcu_txn_desc {
	unsigned long status;		/* enum urcu_txn_desc_status */
	struct rcu_head rcu_head;	/* owner's deferred-free handle */
	unsigned int nr;
	unsigned int nr_mw;		/*
					 * MW-kind records so far, tracked at add
					 * time.  Lets commit SKIP the partition
					 * pass unless the write-set is genuinely
					 * MIXED (0 < nr_mw < nr): an all-MW or
					 * all-SW commit is already trivially
					 * grouped, so a pure-MW commit costs
					 * exactly what the multi-writer-only
					 * engine does.
					 */
	unsigned int cap;
	unsigned int retry;		/* aging priority: prior retries (tiny, bounded by the fallback) */
	unsigned int poisoned;		/* set if a same-slot reconcile disagreed on old */
	unsigned int slab;		/* block origin: per-CPU slab (1) or posix_memalign (0) */
	struct urcu_txn_record recs[];	/* frozen at commit */
};

urcu_static_assert(!(offsetof(struct urcu_txn_desc, recs) % 16),
		"urcu_txn_desc.recs must be 16-byte aligned within the descriptor",
		urcu_txn_recs_aligned);
urcu_static_assert(!(sizeof(struct urcu_txn_record) % 16),
		"urcu_txn_record stride must keep inline records 16-byte aligned",
		urcu_txn_record_stride_aligned);
urcu_static_assert(!(__alignof__(struct urcu_txn_desc) % 16),
		"urcu_txn_desc must inherit 16-byte alignment from its recs[] member",
		urcu_txn_desc_aligned);

/* A convenient default tag (bit 0) for an embedder that keeps bit 0 free. */
#define URCU_TXN_TAG	1UL

/*
 * Does @v carry ALL of @tag's bits -- i.e. is it a parked proxy rather than a
 * live value?  The embedder's tag contract is that no live value a transacted
 * slot holds may do so.
 */
static inline
int urcu_txn_is_proxy(void *v, uintptr_t tag)
{
	return ((uintptr_t) v & tag) == tag;
}

static inline
void *urcu_txn_tag(struct urcu_txn_record *r, uintptr_t tag)
{
	/*
	 * The tag bits must be FREE in the record address, or OR-ing them in
	 * is not reversible: recs[] is 16-byte aligned with a 16-byte stride
	 * and the tag lives in the low 4 bits.  This is the standing invariant
	 * of the encoding, not a new one -- untag by mask needs it just as
	 * much -- so state it once here, where the tagged value is made.
	 */
	urcu_assert_debug(!((uintptr_t) r & tag));
	return (void *) ((uintptr_t) r | tag);
}

/*
 * Recover the record address from a parked slot value.
 *
 * SUBTRACT the tag rather than masking it off.  The two are exactly equivalent
 * here: untag is only ever reached once urcu_txn_is_proxy() has proven every
 * tag bit SET in @v, and urcu_txn_tag() asserts every tag bit CLEAR in the
 * record address, so the tag bits are precisely the difference between the two.
 *
 * The subtraction generates better code on the resolve path.  With a
 * compile-time-constant @tag the compiler folds it into the DISPLACEMENT of the
 * loads that follow -- r->desc becomes one mov at [v + (offsetof(desc) - tag)]
 * -- so the head of the resolve's load-to-use chain issues straight off the raw
 * tagged value.  The AND cannot fold: it is a real ALU op sitting between the
 * slot load and the first dependent load, adding a cycle to a chain that is
 * already three dependent loads deep (record -> desc -> status).  Same trick,
 * same reason, as the fractal trie's FT_NODE_SUB_TAG.
 */
static inline
struct urcu_txn_record *urcu_txn_untag(void *v, uintptr_t tag)
{
	urcu_assert_debug(urcu_txn_is_proxy(v, tag));
	return (struct urcu_txn_record *) ((uintptr_t) v - tag);
}

/* Strong try-CAS returning a success bit. */
static inline
int urcu_txn_try_cas(void **slot, void *expect, void *desired)
{
	return __atomic_compare_exchange_n(slot, &expect, desired,
			/*weak=*/0, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE);
}

static inline
unsigned long urcu_txn_desc_status(const struct urcu_txn_desc *t)
{
	return uatomic_load(&t->status, CMM_ACQUIRE);
}

/*
 * Resolve a parked record directly to the value it currently denotes: its
 * transaction's new on SUCCEEDED, old otherwise.  Kind-agnostic.  Call from
 * within an RCU read-side section.
 */
static inline
void *urcu_txn_resolve_record(struct urcu_txn_record *r)
{
	return urcu_txn_desc_status(r->desc) == URCU_TXN_DESC_SUCCEEDED ?
			r->new_ptr : r->old_ptr;
}

/*
 * Resolve a value loaded from a transacted slot.  A plain value passes through;
 * a parked record resolves through its transaction's status word.
 */
static inline
void *urcu_txn_resolve(void *v, uintptr_t tag)
{
	if (caa_likely(!urcu_txn_is_proxy(v, tag)))
		return v;
	return urcu_txn_resolve_record(urcu_txn_untag(v, tag));
}

/*
 * Drive @t to a terminal status: @t's OWN decision, taken by its owner.  With no
 * helper the owner is the sole writer of its status and writes it at most once,
 * so this is a plain RELEASE store -- the commit point.
 */
static inline
void urcu_txn_decide(struct urcu_txn_desc *t, unsigned long to)
{
	urcu_assert_debug(urcu_txn_desc_status(t) == URCU_TXN_DESC_UNDECIDED);
	uatomic_store(&t->status, to, CMM_RELEASE);
}

/*
 * Plant an MW record: CAS its slot from old_ptr to the record's tagged proxy.
 * Returns 1 if planted, 0 if the CAS raced the slot away.  A bare value-CAS is
 * correct because the owner is the SOLE driver.
 */
static inline
int urcu_txn_plant(struct urcu_txn_record *r)
{
	void *tagv = urcu_txn_tag(r, r->proxy_tag);

	return uatomic_cmpxchg(r->slot, r->old_ptr, tagv) == r->old_ptr;
}

/*
 * Park an SW record: a plain RELEASE store of the tagged proxy.  The caller holds
 * a lock over the slot (single-writer), so this cannot be raced and cannot fail.
 */
static inline
void urcu_txn_park(struct urcu_txn_record *r)
{
	uatomic_store(r->slot, urcu_txn_tag(r, r->proxy_tag), CMM_RELEASE);
}

/*
 * Partition the record array so every MW-kind record precedes every SW-kind
 * record, returning the MW count.  MW records install first (so an MW abort
 * wastes zero SW parks) and only MW records need the slot-address sort for
 * deadlock-freedom (SW parks never wait).  A stable partition is not required.
 */
static inline
unsigned int urcu_txn_partition(struct urcu_txn_desc *t)
{
	unsigned int i, k = 0;

	for (i = 0; i < t->nr; i++) {
		if (t->recs[i].kind == URCU_TXN_KIND_MW) {
			if (i != k) {
				struct urcu_txn_record tmp = t->recs[k];

				t->recs[k] = t->recs[i];
				t->recs[i] = tmp;
			}
			k++;
		}
	}
	return k;
}

/* Insertion-sort records [0..n) by slot address (transactions are small). */
static inline
void urcu_txn_sort(struct urcu_txn_desc *t, unsigned int n)
{
	unsigned int i, j;

	for (i = 1; i < n; i++) {
		struct urcu_txn_record key = t->recs[i];

		for (j = i; j > 0 &&
				(uintptr_t) t->recs[j - 1].slot >
				(uintptr_t) key.slot; j--)
			t->recs[j] = t->recs[j - 1];
		t->recs[j] = key;
	}
}

/*
 * Age-0 flat install of the MW records [0..nr_mw): one strong try-CAS per record,
 * bail at the first conflict.  On a conflict the owner DECIDES FAILED and returns
 * the planted prefix length; on success it leaves the status UNDECIDED (the SW
 * records still install before the single SUCCEEDED store) and returns nr_mw.
 * @*failed reports the outcome.
 */
static inline
unsigned int urcu_txn_install_mw_flat(struct urcu_txn_desc *t,
		unsigned int nr_mw, int *failed)
{
	unsigned int i;

	URCU_TXN_STAT(drive);
	for (i = 0; i < nr_mw; i++) {
		struct urcu_txn_record *r = &t->recs[i];

		if (caa_unlikely(!urcu_txn_try_cas(r->slot, r->old_ptr,
				urcu_txn_tag(r, r->proxy_tag)))) {
			urcu_txn_decide(t, URCU_TXN_DESC_FAILED);
			*failed = 1;
			return i;	/* prefix [0..i) planted */
		}
	}
	*failed = 0;
	return nr_mw;
}

/*
 * Age-1+ sorted, blocking install of the MW records [0..nr_mw): plant each in
 * slot-address order, waiting a bounded spin on a foreign proxy (never driving
 * it) before escalating.  Decides FAILED on a read-set mismatch or a capped wait;
 * on success leaves the status UNDECIDED and returns nr_mw.
 */
static inline
unsigned int urcu_txn_install_mw_depth(struct urcu_txn_desc *t,
		unsigned int nr_mw, int *failed)
{
	unsigned int i;

	URCU_TXN_STAT(drive);
	for (i = 0; i < nr_mw; i++) {
		struct urcu_txn_record *r = &t->recs[i];
		void *tagv = urcu_txn_tag(r, r->proxy_tag);

		for (;;) {
			void *v;

			if (urcu_txn_desc_status(t) != URCU_TXN_DESC_UNDECIDED) {
				*failed = 1;
				return i;
			}
			v = uatomic_load(r->slot, CMM_ACQUIRE);
			if (v == tagv)
				break;		/* already installed */
			if (urcu_txn_is_proxy(v, r->proxy_tag)) {
				struct urcu_txn_record *fr =
					urcu_txn_untag(v, r->proxy_tag);

				if (fr->desc == t)
					break;	/* own proxy (distinct-slot inv.) */
				{
					unsigned int patience =
						URCU_TXN_WAIT_PATIENCE;

					while (urcu_txn_is_proxy(
						uatomic_load(r->slot, CMM_ACQUIRE),
						r->proxy_tag)) {
						if (urcu_txn_desc_status(t) !=
								URCU_TXN_DESC_UNDECIDED) {
							*failed = 1;
							return i;
						}
						if (patience-- == 0) {
							URCU_TXN_STAT(wait_capped);
							urcu_txn_decide(t,
								URCU_TXN_DESC_FAILED);
							*failed = 1;
							return i;
						}
						caa_cpu_relax();
					}
					continue;	/* slot plain now: re-read */
				}
			}
			if (v != r->old_ptr) {
				urcu_txn_decide(t, URCU_TXN_DESC_FAILED);
				*failed = 1;
				return i;
			}
			if (urcu_txn_plant(r))
				break;		/* planted */
			/* raced: slot changed under us -> re-evaluate */
		}
	}
	*failed = 0;
	return nr_mw;
}

/*
 * Settle: make @t's own slots plain -- its new value on SUCCEEDED, its old on
 * FAILED.  Converts the parked prefix [0..planted) with plain RELEASE stores.
 * Owner-only, never waits; works for both kinds (an SW slot is caller-exclusive,
 * an MW slot still holds OUR proxy).
 */
static inline
void urcu_txn_settle(struct urcu_txn_desc *t, unsigned int planted)
{
	unsigned long st = urcu_txn_desc_status(t);
	unsigned int i;

	for (i = 0; i < planted; i++) {
		struct urcu_txn_record *r = &t->recs[i];
		void *want = (st == URCU_TXN_DESC_SUCCEEDED) ?
				r->new_ptr : r->old_ptr;

		uatomic_store(r->slot, want, CMM_RELEASE);
	}
}

/*
 * Load @slot and return the value it currently denotes, waiting a bounded spin
 * for an undecided owner to decide (so the returned value is stable), but never
 * driving it.  Use to read the current value of a word you intend to transact.
 */
static inline
void *urcu_txn_read(void **slot, uintptr_t tag)
{
	for (;;) {
		void *v = uatomic_load(slot, CMM_ACQUIRE);
		struct urcu_txn_desc *e;

		if (caa_likely(!urcu_txn_is_proxy(v, tag)))
			return v;
		e = urcu_txn_untag(v, tag)->desc;
		if (urcu_txn_desc_status(e) != URCU_TXN_DESC_UNDECIDED)
			return urcu_txn_resolve(v, tag);
		{
			unsigned int patience = URCU_TXN_WAIT_PATIENCE;

			while (urcu_txn_desc_status(e) == URCU_TXN_DESC_UNDECIDED) {
				if (patience-- == 0)
					return urcu_txn_resolve(v, tag);
				caa_cpu_relax();
			}
			continue;
		}
	}
}

/*
 * Load @slot and return its logical value WITHOUT waiting on an undecided
 * transaction: the non-blocking counterpart of urcu_txn_read().  Safe for
 * a read set -- a stale optimistic read is reconciled at install (an extra abort,
 * never a wrong commit).
 */
static inline
void *urcu_txn_read_optimistic(void **slot, uintptr_t tag)
{
	void *v = uatomic_load(slot, CMM_ACQUIRE);

	if (caa_likely(!urcu_txn_is_proxy(v, tag)))
		return v;
	return urcu_txn_resolve(v, tag);
}

/*
 * ─────────────────────────────────────────────────────────────────────────
 * Descriptor allocation: one per-CPU size-classed slab.
 * The mixed descriptor header and record stride equal the MW engine's, so the
 * class byte-sizes coincide; a dedicated instance keeps the engines decoupled.
 * ─────────────────────────────────────────────────────────────────────────
 */
static inline
struct urcu_txn_desc *urcu_txn_alloc(size_t size)
{
	void *p;

	if (posix_memalign(&p, 16, size))
		return NULL;
	return (struct urcu_txn_desc *) p;
}

#define urcu_txn_blocksize(cap)	\
	(sizeof(struct urcu_txn_desc) + \
	 (size_t) (cap) * sizeof(struct urcu_txn_record))

static const unsigned int urcu_txn_slab_rc[] = { 4u, 8u, 16u, 32u, 64u, 128u };
#define URCU_TXN_SLAB_NCLASS	\
	((int) (sizeof(urcu_txn_slab_rc) / sizeof(urcu_txn_slab_rc[0])))

extern struct urcu_slab urcu_txn_slab;

static inline
int urcu_txn_slab_class_of(unsigned int req)
{
	int i;

	for (i = 0; i < URCU_TXN_SLAB_NCLASS; i++)
		if (req <= urcu_txn_slab_rc[i])
			return i;
	return -1;
}

static inline
struct urcu_txn_desc *urcu_txn_alloc_cap(unsigned int req)
{
	struct urcu_txn_desc *t;
	int cl;

	if (urcu_slab_enabled(&urcu_txn_slab) &&
			(cl = urcu_txn_slab_class_of(req)) >= 0) {
		t = (struct urcu_txn_desc *) urcu_slab_alloc(&urcu_txn_slab, cl);
		if (caa_likely(t != NULL)) {
			t->cap = urcu_txn_slab_rc[cl];
			t->slab = 1;
		}
	} else {
		t = urcu_txn_alloc(urcu_txn_blocksize(req));
		if (caa_likely(t != NULL)) {
			t->cap = req;
			t->slab = 0;
		}
	}
	return t;
}

static inline
void urcu_txn_free(struct urcu_txn_desc *t)
{
	if (t->slab)
		urcu_slab_free(t);
	else
		free(t);
}

/*
 * Create a transaction with room for @cap records.  @retry is the caller's aging
 * count (selects install strategy and single-edge escalation).
 */
static inline
struct urcu_txn_desc *urcu_txn_create(unsigned int cap,
		unsigned long retry)
{
	struct urcu_txn_desc *t;

	t = urcu_txn_alloc_cap(cap);
	if (!t)
		return NULL;
	t->status = URCU_TXN_DESC_UNDECIDED;
	t->retry = (unsigned int) retry;
	t->nr = 0;
	t->nr_mw = 0;
	t->poisoned = 0;
	return t;
}

/*
 * Append one edge {*slot: old -> new} of kind @kind.  Before commit only.
 * Returns false if the descriptor is full.  The back-pointer (r->desc) is set at
 * commit, after the write-set stops growing (a grow may move the descriptor).
 */
static inline
bool urcu_txn_add(struct urcu_txn_desc *t, void **slot,
		void *old_ptr, void *new_ptr, uintptr_t tag, unsigned int kind)
{
	struct urcu_txn_record *r;

	urcu_assert_debug(!urcu_txn_is_proxy(old_ptr, tag));
	urcu_assert_debug(!urcu_txn_is_proxy(new_ptr, tag));
	if (t->nr == t->cap)
		return false;
	r = &t->recs[t->nr++];
	r->slot = slot;
	r->old_ptr = old_ptr;
	r->new_ptr = new_ptr;
	r->proxy_tag = tag;
	r->kind = kind;
	if (kind == URCU_TXN_KIND_MW)
		t->nr_mw++;
	return true;
}

/* Append a MULTI-WRITER edge (CAS-old install).  Convenience over urcu_txn_add. */
static inline
bool urcu_txn_add_mw(struct urcu_txn_desc *t, void **slot,
		void *old_ptr, void *new_ptr, uintptr_t tag)
{
	return urcu_txn_add(t, slot, old_ptr, new_ptr, tag, URCU_TXN_KIND_MW);
}

/* Append a SINGLE-WRITER edge (caller-exclusive plain park).  Convenience. */
static inline
bool urcu_txn_add_sw(struct urcu_txn_desc *t, void **slot,
		void *old_ptr, void *new_ptr, uintptr_t tag)
{
	return urcu_txn_add(t, slot, old_ptr, new_ptr, tag, URCU_TXN_KIND_SW);
}

static inline
struct urcu_txn_record *urcu_txn_find(struct urcu_txn_desc *t,
		void **slot)
{
	unsigned int i;

	for (i = 0; i < t->nr; i++) {
		if (t->recs[i].slot == slot)
			return &t->recs[i];
	}
	return NULL;
}

/*
 * Record edge {*slot: old -> new} of kind @kind under read-your-own-writes: a
 * same-slot reconcile matches against new_ptr and CHAINS (keeps the original
 * old_ptr, advances new_ptr).  A value mismatch poisons the descriptor (commit
 * aborts).
 *
 * Kind conflict on an already-recorded slot -- an SW record and an MW record on
 * the SAME slot in one txn -- is a contradiction (a slot is SW xor MW; see
 * enum urcu_txn_kind).  Resolve it FAIL-SAFE: MW DOMINATES.  Promote the record
 * to the CAS-old install, which is correct whether or not the slot is actually
 * shared, where keeping the plain SW park would race a concurrent MW writer and
 * tear or lose a write.  It is still the embedder's bug, so debug builds assert
 * it (an abort here would only livelock -- the retry re-hits the same store
 * sequence -- so release fails safe rather than poisoning).
 */
static inline
bool urcu_txn_record_chain(struct urcu_txn_desc *t, void **slot,
		void *old_ptr, void *new_ptr, int upgrade, uintptr_t tag,
		unsigned int kind)
{
	struct urcu_txn_record *r = urcu_txn_find(t, slot);

	if (r != NULL) {
		urcu_assert_debug(r->kind == kind);
		if (kind == URCU_TXN_KIND_MW && r->kind != URCU_TXN_KIND_MW) {
			r->kind = URCU_TXN_KIND_MW;	/* MW dominates: fail-safe to CAS */
			t->nr_mw++;			/* promoted SW -> MW: now counts */
		}
		if (caa_unlikely(r->new_ptr != old_ptr)) {
			t->poisoned = 1;
			return true;
		}
		if (upgrade)
			r->new_ptr = new_ptr;
		return true;
	}
	return urcu_txn_add(t, slot, old_ptr, new_ptr, tag, kind);
}

/*
 * Grow @t's record capacity (room for at least one more), returning the possibly-
 * moved descriptor, or NULL on OOM with @t left intact.  Valid only before commit.
 */
static inline
struct urcu_txn_desc *urcu_txn_grow(struct urcu_txn_desc *t)
{
	unsigned int newcap = t->cap < 2 ? 2 : t->cap * 2;
	unsigned int ncap, nslab;
	struct urcu_txn_desc *n;

	n = urcu_txn_alloc_cap(newcap);
	if (!n)
		return NULL;
	ncap = n->cap;
	nslab = n->slab;
	memcpy(n, t, sizeof(*t) +
			(size_t) t->nr * sizeof(struct urcu_txn_record));
	n->cap = ncap;
	n->slab = nslab;
	urcu_txn_free(t);
	return n;
}

static inline
void urcu_txn_destroy(struct urcu_txn_desc *t)
{
	urcu_txn_free(t);
}

static inline
void urcu_txn_free_rcu(struct rcu_head *head)
{
	urcu_txn_free(caa_container_of(head,
			struct urcu_txn_desc, rcu_head));
}

/*
 * Commit @t (the sw-mw-aware path): install MW records (may abort), then SW
 * records, decide, settle.  Returns true on commit (SUCCEEDED), false on abort
 * (FAILED) -- the caller re-reads and retries.  Reclaim is deferred through
 * @call_rcu_fn; a lone edge commits without a proxy and frees immediately (no
 * grace period).  Call within an RCU read-side section.
 *
 * Use urcu_txn_desc_commit_sw() instead when the descriptor is known to
 * carry NO MW records: it skips the partition, sort, CAS-install and abort path
 * that this one must branch through.
 */
static inline
bool urcu_txn_desc_commit(struct urcu_txn_desc *t,
		void (*call_rcu_fn)(struct rcu_head *,
			void (*)(struct rcu_head *)))
{
	unsigned int i, nr_mw, planted;
	int failed;

	if (caa_unlikely(t->poisoned)) {
		urcu_txn_destroy(t);
		return false;
	}
	if (t->nr == 0) {
		urcu_txn_destroy(t);
		return true;
	}
	if (t->nr == 1) {
		struct urcu_txn_record *r = &t->recs[0];

		if (r->kind == URCU_TXN_KIND_SW) {
			/*
			 * Lone SW edge: caller-exclusive, so a plain release
			 * store IS the atomic commit -- no proxy, no grace
			 * period (as <urcu/rcu-txn-sw.h>).
			 */
			uatomic_store(r->slot, r->new_ptr, CMM_RELEASE);
			urcu_txn_destroy(t);
			return true;
		}
		if (t->retry < URCU_TXN_ESCALATE) {
			/*
			 * Lone MW edge, not yet starved: the CAS itself is the
			 * atomic commit -- no proxy, no grace period.
			 */
			bool committed = uatomic_cmpxchg(r->slot, r->old_ptr,
					r->new_ptr) == r->old_ptr;

			urcu_txn_destroy(t);
			return committed;
		}
		URCU_TXN_STAT(escalate);	/* starved single edge: full path */
	}
	/*
	 * Set the record back-pointers now, deferred from add time: from here the
	 * descriptor is frozen and about to be parked.
	 */
	for (i = 0; i < t->nr; i++)
		t->recs[i].desc = t;
	/*
	 * MW records first (an MW abort then wastes zero SW parks); only the MW
	 * prefix needs the slot-address sort (age 1+) for deadlock-freedom.  The
	 * O(nr) reorder is needed only for a genuinely MIXED write-set: an all-MW
	 * or all-SW descriptor is already trivially grouped (nr_mw is 0 or nr), so
	 * a pure-MW commit skips the partition and costs exactly what the
	 * multi-writer-only engine does.
	 */
	nr_mw = t->nr_mw;
	if (nr_mw != 0 && nr_mw != t->nr) {
		unsigned int k = urcu_txn_partition(t);

		urcu_assert_debug(k == nr_mw);
		(void) k;
	}
#if defined(DEBUG_RCU) || defined(CONFIG_RCU_DEBUG)
	for (i = 1; i < t->nr; i++) {
		unsigned int j;

		for (j = 0; j < i; j++)
			urcu_assert_debug(t->recs[i].slot != t->recs[j].slot);
	}
#endif
	if (t->retry != 0)
		urcu_txn_sort(t, nr_mw);
	if (t->retry == 0)
		planted = urcu_txn_install_mw_flat(t, nr_mw, &failed);
	else
		planted = urcu_txn_install_mw_depth(t, nr_mw, &failed);
	if (failed) {
		/* Abort: restore the parked MW prefix to old, then reclaim. */
		urcu_txn_settle(t, planted);
		call_rcu_fn(&t->rcu_head, urcu_txn_free_rcu);
		return false;
	}
	/* Every MW record installed; the status is still UNDECIDED. */
	for (i = nr_mw; i < t->nr; i++)
		urcu_txn_park(&t->recs[i]);	/* SW parks: plain, never fail */
	urcu_txn_decide(t, URCU_TXN_DESC_SUCCEEDED);	/* linearization point */
	urcu_txn_settle(t, t->nr);
	call_rcu_fn(&t->rcu_head, urcu_txn_free_rcu);
	return true;
}

/*
 * Commit @t assuming EVERY record is SW-kind (caller-exclusive slots): park all,
 * flip, settle.  No partition, no sort, no CAS, no contention abort -- the
 * branch-lean path for a transaction the embedder knows carries no MW records.
 * Returns true when published; false only when the descriptor was poisoned by a
 * torn same-slot read-set (never a contention abort -- SW parks cannot fail).
 * Use urcu_txn_desc_commit() when MW records may be present.
 */
static inline
bool urcu_txn_desc_commit_sw(struct urcu_txn_desc *t,
		void (*call_rcu_fn)(struct rcu_head *,
			void (*)(struct rcu_head *)))
{
	unsigned int i;

	urcu_assert_debug(t->nr_mw == 0);	/* caller promised store_sw-only */
	if (caa_unlikely(t->poisoned)) {
		urcu_txn_destroy(t);
		return false;
	}
	if (t->nr == 0) {
		urcu_txn_destroy(t);
		return true;
	}
	if (t->nr == 1) {
		struct urcu_txn_record *r = &t->recs[0];

		urcu_assert_debug(r->kind == URCU_TXN_KIND_SW);
		/* Lone SW edge: caller-exclusive plain store, no proxy, no GP. */
		uatomic_store(r->slot, r->new_ptr, CMM_RELEASE);
		urcu_txn_destroy(t);
		return true;
	}
	for (i = 0; i < t->nr; i++) {
		urcu_assert_debug(t->recs[i].kind == URCU_TXN_KIND_SW);
		t->recs[i].desc = t;		/* back-pointer: readers resolve through it */
	}
	for (i = 0; i < t->nr; i++)
		urcu_txn_park(&t->recs[i]);		/* plain stores, never fail */
	urcu_txn_decide(t, URCU_TXN_DESC_SUCCEEDED);	/* linearization point */
	urcu_txn_settle(t, t->nr);
	call_rcu_fn(&t->rcu_head, urcu_txn_free_rcu);
	return true;
}

#ifdef __cplusplus
}
#endif

#endif	/* _URCU_RCU_TXN_MCAS_H */
