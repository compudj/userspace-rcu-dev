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
 * ABORT ATTRIBUTION (opt-in, default inert).  A commit that returns ABORT says
 * only that SOMETHING lost; the engine knows exactly WHICH record lost and has
 * never reported it, so an embedder measuring its own abort rate can attribute
 * it only by the transaction it started -- which is not attribution when one
 * transaction carries records of several kinds.
 *
 * URCU_TXN_STAT_ABORT(t, r) is handed the descriptor and the LOSING record --
 * the latter NULL where the abort has none (a poisoned descriptor).  BOTH,
 * because a losing record is read against its SIBLINGS ("these two records
 * disagree about whether the op holds that word" is the diagnosis; "this CAS
 * lost" is not), and because @r->desc is NOT yet set on the lone-MW-edge path:
 * the back-pointers are filled in after that fast path has already returned.  ☠ It fires at THREE exits, not one: the
 * install loop's failure, AND the lone-MW-edge fast path, whose CAS is the whole
 * commit and never builds a descriptor at all.  An instrument hooked only at the
 * first is blind to every single-record contention abort.
 *
 * URCU_TXN_REC_DBG_STAMP / _CHAIN let the embedder label a record as it is
 * added, so the label travels WITH the record through the partition and the
 * slot-address sort that reorder the write set before install.  _CHAIN fires
 * where a read-your-own-writes store folds onto a record that already exists:
 * the embedder decides what two labels on one word mean, because the engine
 * cannot.
 */
#ifndef URCU_TXN_STAT_ABORT
#define URCU_TXN_STAT_ABORT(t, r)	do { } while (0)
#endif
#ifndef URCU_TXN_REC_DBG_STAMP
#define URCU_TXN_REC_DBG_STAMP(r)	do { } while (0)
#endif
#ifndef URCU_TXN_REC_DBG_CHAIN
#define URCU_TXN_REC_DBG_CHAIN(r)	do { } while (0)
#endif

/*
 * URCU_TXN_REC_WROTE(r, v) is the WINNER's half of the same attribution
 * story.  URCU_TXN_STAT_ABORT names the record that LOST a slot; nothing
 * names the write it lost TO.  This hook fires immediately AFTER every store
 * the engine makes to a record's slot -- the proxy plants (CAS and park), the
 * settle stores of both passes (the new value on SUCCEEDED, the old on the
 * FAILED restore, which a loser can observe just the same), and the lone-edge
 * fast paths whose single store IS the commit -- handing the embedder the
 * record and the value just written, so it can keep a last-writer ledger an
 * aborting peer may consult.
 *
 * ☠ AFTER the store, never before: a hook that fires first publishes a
 * "winner" that may yet lose its own CAS, and the ledger would then name a
 * writer of a value the slot never held.  The window this leaves (store
 * done, ledger not yet) is the instrument's to account for, not to hide --
 * the embedder can match the ledger's value against the slot before
 * believing it.
 */
#ifndef URCU_TXN_REC_WROTE
#define URCU_TXN_REC_WROTE(r, v)	do { } while (0)
#endif

/*
 * URCU_TXN_REC_LOST(r, seen) hands the embedder the value the losing CAS
 * OBSERVED, at the instant of the loss.  The CAS instruction returns that
 * value and the engine has always thrown it away -- an embedder reconstructing
 * it later from a re-read is racing every subsequent writer of the slot.
 * @seen NULL where the exit has no observed value in hand (the pre-load
 * status check).  Fires before URCU_TXN_STAT_ABORT for the same record; the
 * embedder pairs the two.
 */
#ifndef URCU_TXN_REC_LOST
#define URCU_TXN_REC_LOST(r, seen)	do { } while (0)
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
#ifdef URCU_TXN_REC_DBG
	/*
	 * The embedder's label for this record (URCU_TXN_REC_DBG_STAMP), so an
	 * abort can name the KIND of edge that lost rather than the transaction
	 * that carried it.  Opaque to the engine: written by the embedder's hook,
	 * read by its own.
	 *
	 * ☠ GATED ON A COMPILATION-WIDE DEFINE, not on an embedder header,
	 * because the DESCRIPTOR is allocated by one translation unit
	 * (urcu-txn.c's slab, sized with sizeof(struct urcu_txn_record)) and
	 * filled by another.  A field only some TUs can see is a layout split,
	 * not a diagnostic.  Pass -DURCU_TXN_REC_DBG in CFLAGS or not at all.
	 * It lands in the tail padding the aligned(16) already reserves, so the
	 * record does not grow.
	 */
	unsigned int dbg_embedder;
#endif
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
	unsigned int poisoned;		/* this attempt cannot commit: a same-slot
					 * reconcile disagreed on old, or a record
					 * value was a proxy (see urcu_txn_add) */
	unsigned int slab;		/* block origin: per-CPU slab (1) or posix_memalign (0) */
	/*
	 * THE WORDS THAT HAND OWNERSHIP OVER, SETTLED LAST.
	 *
	 * A settle is a plain store per parked word, so between the DECIDE and
	 * the last of those stores an embedder's LOCK word can already read
	 * "free" while the words that lock protects are still this descriptor's
	 * proxies.  A peer that takes the lock there is a CORRECT holder --
	 * nothing it does is wrong -- but the word it writes is one this settle
	 * is about to store over, and what it published is lost.
	 *
	 * Declaring the ownership words' proxy tag makes settle run them in a
	 * SECOND pass, so the lock is handed over only once every word it
	 * protects is plain.  0 = no late class (one forward pass): an embedder
	 * that declares nothing keeps exactly the previous behaviour.
	 *
	 * The header grows 48 -> 64 bytes: recs[] is 16-byte aligned (a parked
	 * slot is (&record | tag), so the tag bits must stay free), which
	 * urcu_txn_recs_aligned above already asserts.
	 */
	uintptr_t late_tag;
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

static inline
void *urcu_txn_tag(struct urcu_txn_record *r, uintptr_t tag)
{
	return (void *) ((uintptr_t) r | tag);
}

static inline
struct urcu_txn_record *urcu_txn_untag(void *v, uintptr_t tag)
{
	return (struct urcu_txn_record *) ((uintptr_t) v & ~tag);
}

static inline
int urcu_txn_is_proxy(void *v, uintptr_t tag)
{
	return ((uintptr_t) v & tag) == tag;
}

/* Strong try-CAS returning a success bit. */
static inline
int urcu_txn_try_cas(void **slot, void *expect, void *desired)
{
	return __atomic_compare_exchange_n(slot, &expect, desired,
			/*weak=*/0, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE);
}

/*
 * Strong try-CAS that also reports the OBSERVED value through @seen -- the
 * word the compare-exchange returns anyway.  On success *@seen == @expect.
 * The inner shape mirrors urcu_txn_try_cas exactly; a caller whose @seen is
 * dead (the hooks compiled out) folds to the same code.
 */
static inline
int urcu_txn_try_cas_seen(void **slot, void *expect, void *desired,
		void **seen)
{
	int ok = __atomic_compare_exchange_n(slot, &expect, desired,
			/*weak=*/0, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE);

	*seen = expect;
	return ok;
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
	int planted = uatomic_cmpxchg(r->slot, r->old_ptr, tagv) == r->old_ptr;

	if (planted)
		URCU_TXN_REC_WROTE(r, tagv);
	return planted;
}

/*
 * Park an SW record: a plain RELEASE store of the tagged proxy.  The caller holds
 * a lock over the slot (single-writer), so this cannot be raced and cannot fail.
 */
static inline
void urcu_txn_park(struct urcu_txn_record *r)
{
	uatomic_store(r->slot, urcu_txn_tag(r, r->proxy_tag), CMM_RELEASE);
	URCU_TXN_REC_WROTE(r, urcu_txn_tag(r, r->proxy_tag));
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
		void *seen;

		if (caa_unlikely(!urcu_txn_try_cas_seen(r->slot, r->old_ptr,
				urcu_txn_tag(r, r->proxy_tag), &seen))) {
			URCU_TXN_REC_LOST(r, seen);
			urcu_txn_decide(t, URCU_TXN_DESC_FAILED);
			*failed = 1;
			return i;	/* prefix [0..i) planted */
		}
		URCU_TXN_REC_WROTE(r, urcu_txn_tag(r, r->proxy_tag));
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
				URCU_TXN_REC_LOST(r, NULL);
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
						(v = uatomic_load(r->slot,
							CMM_ACQUIRE)),
						r->proxy_tag)) {
						if (urcu_txn_desc_status(t) !=
								URCU_TXN_DESC_UNDECIDED) {
							URCU_TXN_REC_LOST(r, v);
							*failed = 1;
							return i;
						}
						if (patience-- == 0) {
							URCU_TXN_STAT(wait_capped);
							URCU_TXN_REC_LOST(r, v);
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
				URCU_TXN_REC_LOST(r, v);
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
 *
 * OWNERSHIP WORDS LAST (@t->late_tag).  These stores are plain and unordered
 * against each other, so a word that hands ownership over -- an embedder's lock
 * -- must not become free while the words it protects are still this
 * descriptor's proxies: a peer would then take the lock legitimately, write one
 * of those words, and the store below would put our value back over it.  Both
 * passes stay inside ONE commit, so nothing about the transaction's atomicity
 * changes: every record decided together at the linearization point, and a
 * reader resolving a parked slot never sees this order at all.
 */

#ifdef URCU_TXN_DEBUG_SETTLE
/*
 * =====================================================================
 * DIAGNOSTIC (opt-in: -DURCU_TXN_DEBUG_SETTLE).  NOT part of the engine.
 * =====================================================================
 *
 * CHECK THE PREMISE THE SETTLE IS WRITTEN ON.  urcu_txn_settle converts a
 * parked slot with a PLAIN release store, which is sound only while nothing
 * else can write that word between the park and the settle -- "an SW slot is
 * caller-exclusive, an MW slot still holds OUR proxy".  Both halves are
 * EMBEDDER obligations (the lock-set for SW, the MW protocol for MW) and
 * neither is checked, so a violation is invisible: the settle silently
 * OVERWRITES the foreign value, re-publishing whatever this descriptor wrote
 * into a slot its owner has since replaced.
 *
 * A parked slot holds exactly urcu_txn_tag(r, r->proxy_tag) whichever kind
 * parked it, so the check is one load and one compare per record.
 *
 * Validated by tests/unit/test_rcu_txn_settle_premise.c, which CONSTRUCTS the
 * violation rather than waiting for one -- see the note there on why the
 * obvious validation (re-inject a historical defect into a contended arm) no
 * longer reproduces on this tree.
 */
#define URCU_TXN_DBG_FOREIGN_MAX	8
extern __thread void *urcu_txn_dbg_foreign_slot[URCU_TXN_DBG_FOREIGN_MAX];
extern __thread void *urcu_txn_dbg_foreign_cur[URCU_TXN_DBG_FOREIGN_MAX];
extern __thread unsigned int urcu_txn_dbg_foreign_n;
extern unsigned long urcu_txn_dbg_settle_checked;
extern unsigned long urcu_txn_dbg_settle_foreign;
extern unsigned long urcu_txn_dbg_settle_sibling;

static inline
void urcu_txn_dbg_parked_check(const struct urcu_txn_desc *t,
		struct urcu_txn_record *r, unsigned int i)
{
	void *parked = urcu_txn_tag(r, r->proxy_tag);
	void *cur = uatomic_load(r->slot, CMM_ACQUIRE);
	unsigned int j, sib = (unsigned int) -1;

	uatomic_inc(&urcu_txn_dbg_settle_checked);
	if (caa_likely(cur == parked))
		return;
	/*
	 * RULE OUT THE DESCRIPTOR ITSELF BEFORE ACCUSING A PEER.  Two records on
	 * ONE slot park over each other and settle in index order, so the earlier
	 * one's settled value is what a later one finds -- a violation of this
	 * premise that is entirely THIS descriptor's doing, and a different defect
	 * (the engine's distinct-slot invariant) from a peer writing a word we
	 * hold parked.  Scanned only on a FAILED check, so the O(nr) cost is paid
	 * once per violation and never per settle.
	 */
	for (j = 0; j < t->nr; j++) {
		if (j != i && t->recs[j].slot == r->slot) {
			sib = j;
			break;
		}
	}
	if (sib != (unsigned int) -1) {
		uatomic_inc(&urcu_txn_dbg_settle_sibling);
		return;
	}
	uatomic_inc(&urcu_txn_dbg_settle_foreign);
	/*
	 * Hand the violating SLOTS back to the embedder: the engine knows the
	 * word and the kind but not WHO declared the record SW, and that call
	 * site is the whole answer, since an SW record is a promise of exclusion
	 * this word did not keep.  Drained by the commit's caller in the same
	 * thread, while its per-slot recording sites are still around.
	 */
	if (urcu_txn_dbg_foreign_n < URCU_TXN_DBG_FOREIGN_MAX) {
		urcu_txn_dbg_foreign_cur[urcu_txn_dbg_foreign_n] = cur;
		urcu_txn_dbg_foreign_slot[urcu_txn_dbg_foreign_n++] =
			(void *) r->slot;
	}
}
#else
#define urcu_txn_dbg_parked_check(t, r, i)	do { } while (0)
#endif /* URCU_TXN_DEBUG_SETTLE */

static inline
void urcu_txn_settle(struct urcu_txn_desc *t, unsigned int planted)
{
	unsigned long st = urcu_txn_desc_status(t);
	unsigned int i;

	for (i = 0; i < planted; i++) {
		struct urcu_txn_record *r = &t->recs[i];
		void *want = (st == URCU_TXN_DESC_SUCCEEDED) ?
				r->new_ptr : r->old_ptr;

		if (t->late_tag && r->proxy_tag == t->late_tag)
			continue;		/* second pass, below */
		urcu_txn_dbg_parked_check(t, r, i);
		uatomic_store(r->slot, want, CMM_RELEASE);
		URCU_TXN_REC_WROTE(r, want);
	}
	if (t->late_tag) {
		for (i = 0; i < planted; i++) {
			struct urcu_txn_record *r = &t->recs[i];

			if (r->proxy_tag != t->late_tag)
				continue;
			urcu_txn_dbg_parked_check(t, r, i);
			uatomic_store(r->slot,
				st == URCU_TXN_DESC_SUCCEEDED ?
					r->new_ptr : r->old_ptr,
				CMM_RELEASE);
			URCU_TXN_REC_WROTE(r,
				st == URCU_TXN_DESC_SUCCEEDED ?
					r->new_ptr : r->old_ptr);
		}
	}
}

/*
 * Declare the proxy tag of this descriptor's OWNERSHIP words (see @late_tag).
 * Call before commit; 0 restores the single forward pass.
 */
static inline
void urcu_txn_desc_set_late_tag(struct urcu_txn_desc *t, uintptr_t tag)
{
	/*
	 * URCU_TXN_ENOMEM is the LAYER ABOVE's sentinel (<urcu/rcu-txn.h>, which
	 * includes this header, not the other way round), so spell it out rather
	 * than reach forward for the macro.
	 */
	if (t && t != (struct urcu_txn_desc *) -1L)
		t->late_tag = tag;
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
	t->late_tag = 0;	/* recycled slab block: declare nothing by default */
	return t;
}

/*
 * Append one edge {*slot: old -> new} of kind @kind.  Before commit only.
 * Returns false if the descriptor is full.  The back-pointer (r->desc) is set at
 * commit, after the write-set stops growing (a grow may move the descriptor).
 *
 * A record VALUE is never a proxy, and this is where that is enforced.  It is
 * the ONE way a foreign parked record can escape into a live slot: settle()
 * stores old_ptr (abort) or new_ptr (commit) BLIND, so a proxy sitting in
 * either field is published over the word -- and stays there for good, because
 * the record it denotes belongs to a transaction that decided and settled long
 * before.  Every later reader of that word sees a parked proxy no one will ever
 * clear: an embedder that treats "parked" as busy (an FT node-lock acquire)
 * then fails on it forever.
 *
 * It means the embedder read the slot RAW instead of through urcu_txn_load() /
 * urcu_txn_read(), so its expected-old does not describe the slot either way
 * and the attempt cannot correctly commit.  Poison rather than trap: the commit
 * aborts having parked nothing (freeze-before-install leaves the structure
 * byte-for-byte untouched), and the retry re-reads a slot whose transient proxy
 * has since settled.  Debug builds still trap, to name the raw read.
 */
static inline
bool urcu_txn_add(struct urcu_txn_desc *t, void **slot,
		void *old_ptr, void *new_ptr, uintptr_t tag, unsigned int kind)
{
	struct urcu_txn_record *r;

	urcu_assert_debug(!urcu_txn_is_proxy(old_ptr, tag));
	urcu_assert_debug(!urcu_txn_is_proxy(new_ptr, tag));
	if (caa_unlikely(urcu_txn_is_proxy(old_ptr, tag) ||
			urcu_txn_is_proxy(new_ptr, tag)))
		t->poisoned = 1;	/* recorded, but this attempt cannot commit */
	if (t->nr == t->cap)
		return false;
	r = &t->recs[t->nr++];
	r->slot = slot;
	r->old_ptr = old_ptr;
	r->new_ptr = new_ptr;
	r->proxy_tag = tag;
	r->kind = kind;
	URCU_TXN_REC_DBG_STAMP(r);
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
		URCU_TXN_REC_DBG_CHAIN(r);
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
		URCU_TXN_STAT_ABORT(t, NULL);	/* no losing record: poisoned */
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
			URCU_TXN_REC_WROTE(r, r->new_ptr);
			urcu_txn_destroy(t);
			return true;
		}
		if (t->retry < URCU_TXN_ESCALATE) {
			/*
			 * Lone MW edge, not yet starved: the CAS itself is the
			 * atomic commit -- no proxy, no grace period.
			 */
			void *seen = uatomic_cmpxchg(r->slot, r->old_ptr,
					r->new_ptr);
			bool committed = seen == r->old_ptr;

			if (caa_unlikely(!committed)) {
				URCU_TXN_REC_LOST(r, seen);
				URCU_TXN_STAT_ABORT(t, r);
			} else {
				URCU_TXN_REC_WROTE(r, r->new_ptr);
			}
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
		URCU_TXN_STAT_ABORT(t, &t->recs[planted]);
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
 * Returns true when published; false only when the descriptor was poisoned --
 * a torn same-slot read-set, or a proxy-valued record (never a contention
 * abort -- SW parks cannot fail).
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
		URCU_TXN_STAT_ABORT(t, NULL);	/* no losing record: poisoned */
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
		URCU_TXN_REC_WROTE(r, r->new_ptr);
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
