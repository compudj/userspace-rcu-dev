// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

#ifndef _URCU_FLIP_LATCH_LOCKFREE_H
#define _URCU_FLIP_LATCH_LOCKFREE_H

/*
 * Lock-free flip-latch: a multi-word compare-and-swap (k-CAS) engine.
 *
 * This is the concurrent-writer sibling of <urcu/flip-latch.h>.  It switches
 * a *set* of words from their old values to new ones atomically, as observed
 * by both concurrent RCU readers AND concurrent writers, with lock-free
 * progress (a stalled writer never blocks others -- any thread that trips
 * over an in-flight transaction helps drive it to completion).
 *
 * Model (a "practical MCAS", Harris-style, specialised to the flip-latch)
 * ----------------------------------------------------------------------
 * A transaction is a frozen set of records {slot, old, new} plus one tri-state
 * status word:
 *
 *     UNDECIDED --(install all, then)--> SUCCEEDED   (commit)
 *               --(read-set invalid)---> FAILED      (abort)
 *
 * Each slot is a 3-state machine, strictly monotone, never backward:
 *
 *     old --install--> flip(record) --settle--> new        (status SUCCEEDED)
 *                                   --restore-> old        (status FAILED)
 *
 * "flip(record)" is the record's tagged address parked in the slot.  EVERY
 * transition is a descriptor-naming CAS, so a slot reused across transaction
 * epochs can never be dragged backward by a straggler (the value alone is
 * ambiguous -- one txn's new is the next txn's old -- only the descriptor
 * identity disambiguates).
 *
 * Resolution.  A reader (or a writer traversing) that loads a slot holding
 * flip(r) reads r->txn->status: SUCCEEDED resolves to r->new_ptr, UNDECIDED or
 * FAILED to r->old_ptr.  Because resolution goes through the status word, planting
 * a terminal descriptor in a slot is *correct*, not a bug -- which is why no
 * RDCSS is needed.
 *
 * Liveness.  Records are installed in a single global order (sorted by slot
 * address) so two conflicting transactions always meet at their lowest shared
 * slot, where exactly one CAS wins; the loser HELPS the winner forward (never
 * aborts it) and retries.  A transaction aborts only *itself*, and only
 * because a foreign transaction committed under it (read-set invalid) -- so
 * every self-abort is paid for by real global progress.  This is lock-free,
 * not wait-free.
 *
 * Existence.  Mutators run as RCU readers (rcu_read_lock around the whole
 * operation).  Any descriptor a helper reaches through a slot stays alive
 * until the helper leaves its read-side section; the owner call_rcu()s the
 * descriptor once it is terminal and fully settled.  No refcount.
 *
 * Constraints (vs the single-writer <urcu/flip-latch.h>):
 *   - the engine owns tag bit 0, so every value stored in a transacted slot
 *     must be at least 2-byte aligned (bit 0 clear) -- true for any pointer,
 *     or store small integers shifted left by 1;
 *   - the record set is frozen at commit: no record may be added after the
 *     first install (helpers walk the immutable, sorted record list);
 *   - a transaction's records must target pairwise-distinct slots.
 *
 * PRECONDITION -- slot values must be non-ABA-able within a reader's critical
 * section.  The engine deliberately omits RDCSS: it installs a descriptor with a
 * plain CAS conditioned on the word holding its expected old.  That is safe ONLY
 * if a word cannot cycle back to a prior value while a thread sits between
 * reading that value and installing -- otherwise a delayed install of an
 * already-decided descriptor clobbers the word with the descriptor's stale new.
 * For RCU-managed POINTER slots this holds for free: a freed node's address is
 * not reused within a read-side section, and a removed node is not re-linked
 * before its grace period, so a slot never ABAs.  An MCAS over plain reusable
 * values (e.g. integer counters) does NOT satisfy this and would need per-word
 * versioning or a real RDCSS install.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>

#include <urcu/compiler.h>
#include <urcu/uatomic.h>
#include <urcu/call-rcu.h>		/* struct rcu_head */

#ifdef __cplusplus
extern "C" {
#endif

enum urcu_flip_lf_status {
	URCU_FLIP_LF_UNDECIDED = 0,
	URCU_FLIP_LF_SUCCEEDED = 1,
	URCU_FLIP_LF_FAILED    = 2,
};

struct urcu_flip_lf_txn;

struct urcu_flip_lf_record {
	void **slot;			/* transacted word (bit 0 must be free) */
	void *old_ptr;			/* expected old value */
	void *new_ptr;			/* committed new value */
	struct urcu_flip_lf_txn *txn;	/* back-pointer (status + sibling records) */
};

/*
 * The records are stored inline after the header and a parked slot holds the
 * tagged address of one record (urcu_flip_lf_tag()).  The engine itself owns
 * only tag bit 0, but the txn is 16-byte aligned and both the recs[] offset and
 * the record stride are multiples of 16, so every inline record address has its
 * low 4 bits free.  An embedder that tags a transacted slot with a wider type
 * code (e.g. the fractal trie's low-4-bit pointer tags) can thus park records
 * through this engine without losing tag room.  The static asserts below pin
 * that guarantee against future field changes.
 */
struct urcu_flip_lf_txn {
	unsigned long status;		/* enum urcu_flip_lf_status, CAS-updated */
	struct rcu_head rcu_head;	/* owner's deferred-free handle */
	unsigned int nr;
	unsigned int cap;
	struct urcu_flip_lf_record recs[];	/* frozen + slot-sorted at commit */
} __attribute__((aligned(16)));

urcu_static_assert(!(offsetof(struct urcu_flip_lf_txn, recs) % 16),
		"urcu_flip_lf_txn.recs must be 16-byte aligned within the txn",
		urcu_flip_lf_txn_recs_aligned);
urcu_static_assert(!(sizeof(struct urcu_flip_lf_record) % 16),
		"urcu_flip_lf_record stride must keep inline records 16-byte aligned",
		urcu_flip_lf_record_stride_aligned);

/* The reserved tag bit marking a slot value as a parked record (proxy). */
#define URCU_FLIP_LF_TAG	1UL

static inline
int urcu_flip_lf_is_proxy(void *v)
{
	return (int) ((uintptr_t) v & URCU_FLIP_LF_TAG);
}

static inline
struct urcu_flip_lf_record *urcu_flip_lf_untag(void *v)
{
	return (struct urcu_flip_lf_record *)
			((uintptr_t) v & ~(uintptr_t) URCU_FLIP_LF_TAG);
}

static inline
void *urcu_flip_lf_tag(struct urcu_flip_lf_record *r)
{
	return (void *) ((uintptr_t) r | URCU_FLIP_LF_TAG);
}

static inline
unsigned long urcu_flip_lf_status(struct urcu_flip_lf_txn *t)
{
	return uatomic_load(&t->status, CMM_ACQUIRE);
}

/*
 * Resolve a value loaded from a transacted slot to the value it currently
 * denotes.  A plain value passes through; a parked record resolves through its
 * transaction's status word.  Call from within an RCU read-side section.
 */
static inline
void *urcu_flip_lf_resolve(void *v)
{
	struct urcu_flip_lf_record *r;

	if (caa_likely(!urcu_flip_lf_is_proxy(v)))
		return v;
	r = urcu_flip_lf_untag(v);
	return urcu_flip_lf_status(r->txn) == URCU_FLIP_LF_SUCCEEDED ?
			r->new_ptr : r->old_ptr;
}

/*
 * Drive transaction @t to a terminal, fully-settled state.  Safe to call on
 * one's own transaction or on any foreign transaction encountered in a slot
 * (helping).  Idempotent and re-entrant under the global install order:
 * helping follows strictly increasing slot addresses, so recursion is bounded
 * by the number of concurrently-conflicting transactions.
 */
static inline
void urcu_flip_lf_drive(struct urcu_flip_lf_txn *t)
{
	unsigned long st = urcu_flip_lf_status(t);
	unsigned int i;

	/* --- install phase (only while UNDECIDED) --- */
	if (st == URCU_FLIP_LF_UNDECIDED) {
		for (i = 0; i < t->nr; i++) {
			struct urcu_flip_lf_record *r = &t->recs[i];
			void *tagv = urcu_flip_lf_tag(r);

			for (;;) {
				void *v;

				st = urcu_flip_lf_status(t);
				if (st != URCU_FLIP_LF_UNDECIDED)
					goto settle;	/* decided by a helper */
				v = uatomic_load(r->slot, CMM_ACQUIRE);
				if (v == tagv)
					break;		/* already installed */
				if (urcu_flip_lf_is_proxy(v)) {
					struct urcu_flip_lf_record *fr =
						urcu_flip_lf_untag(v);

					if (fr->txn != t)
						urcu_flip_lf_drive(fr->txn);
					else
						break;	/* own (distinct-slot inv.) */
					continue;	/* re-read this slot */
				}
				if (v != r->old_ptr) {
					/* read-set invalid -> abort this txn */
					uatomic_cmpxchg(&t->status,
						URCU_FLIP_LF_UNDECIDED,
						URCU_FLIP_LF_FAILED);
					goto settle;
				}
				if (uatomic_cmpxchg(r->slot, r->old_ptr, tagv) == r->old_ptr)
					break;		/* installed */
				/* slot changed under us -> re-evaluate */
			}
		}
		/* every record installed -> commit */
		uatomic_cmpxchg(&t->status, URCU_FLIP_LF_UNDECIDED,
				URCU_FLIP_LF_SUCCEEDED);
	}

settle:
	/* --- settle (SUCCEEDED) or restore (FAILED) -- both idempotent CAS --- */
	st = urcu_flip_lf_status(t);
	for (i = 0; i < t->nr; i++) {
		struct urcu_flip_lf_record *r = &t->recs[i];
		void *want = (st == URCU_FLIP_LF_SUCCEEDED) ? r->new_ptr : r->old_ptr;

		(void) uatomic_cmpxchg(r->slot, urcu_flip_lf_tag(r), want);
	}
}

/*
 * Load @slot and, if it holds a parked record, help that transaction settle so
 * a plain value is returned.  Use this to read the current value of a word you
 * intend to transact (its old).  Call within an RCU read-side section.
 */
static inline
void *urcu_flip_lf_read(void **slot)
{
	for (;;) {
		void *v = uatomic_load(slot, CMM_ACQUIRE);

		if (caa_likely(!urcu_flip_lf_is_proxy(v)))
			return v;
		urcu_flip_lf_drive(urcu_flip_lf_untag(v)->txn);
	}
}

static inline
struct urcu_flip_lf_txn *urcu_flip_lf_txn_create(unsigned int cap)
{
	struct urcu_flip_lf_txn *t;

	t = (struct urcu_flip_lf_txn *) malloc(sizeof(*t) +
			(size_t) cap * sizeof(struct urcu_flip_lf_record));
	if (!t)
		return NULL;
	t->status = URCU_FLIP_LF_UNDECIDED;
	t->nr = 0;
	t->cap = cap;
	return t;
}

/* Append one edge {*slot: old -> new}.  Before commit only.  No install yet. */
static inline
bool urcu_flip_lf_txn_add(struct urcu_flip_lf_txn *t, void **slot,
		void *old_ptr, void *new_ptr)
{
	struct urcu_flip_lf_record *r;

	if (t->nr == t->cap)
		return false;
	r = &t->recs[t->nr++];
	r->slot = slot;
	r->old_ptr = old_ptr;
	r->new_ptr = new_ptr;
	r->txn = t;
	return true;
}

static inline
void urcu_flip_lf_txn_destroy(struct urcu_flip_lf_txn *t)
{
	free(t);
}

/* call_rcu callback: deferred destroy. */
static inline
void urcu_flip_lf_txn_free_rcu(struct rcu_head *head)
{
	urcu_flip_lf_txn_destroy(caa_container_of(head,
			struct urcu_flip_lf_txn, rcu_head));
}

/* Sort records by slot address (insertion sort -- transactions are small). */
static inline
void urcu_flip_lf_txn_sort(struct urcu_flip_lf_txn *t)
{
	unsigned int i, j;

	for (i = 1; i < t->nr; i++) {
		struct urcu_flip_lf_record key = t->recs[i];

		for (j = i; j > 0 &&
				(uintptr_t) t->recs[j - 1].slot >
				(uintptr_t) key.slot; j--)
			t->recs[j] = t->recs[j - 1];
		t->recs[j] = key;
	}
}

/*
 * Commit @t: install in slot-address order, decide, settle.  Returns true if
 * the transaction committed (SUCCEEDED), false if it aborted (FAILED) -- the
 * caller re-reads and retries an aborted transaction.  Reclaim is deferred
 * through @call_rcu_fn (the flavor's call_rcu); a single-edge transaction
 * commits with a bare CAS and frees immediately (no proxy, no grace period).
 *
 * Call within an RCU read-side section (descriptor existence for helpers).
 */
static inline
bool urcu_flip_lf_txn_commit(struct urcu_flip_lf_txn *t,
		void (*call_rcu_fn)(struct rcu_head *,
			void (*)(struct rcu_head *)))
{
	bool committed;

	if (t->nr == 0) {
		urcu_flip_lf_txn_destroy(t);
		return true;
	}
	if (t->nr == 1) {
		/* Single edge: the CAS itself is the atomic commit. */
		struct urcu_flip_lf_record *r = &t->recs[0];

		committed = uatomic_cmpxchg(r->slot, r->old_ptr, r->new_ptr) == r->old_ptr;
		urcu_flip_lf_txn_destroy(t);
		return committed;
	}
	urcu_flip_lf_txn_sort(t);
	urcu_flip_lf_drive(t);
	committed = urcu_flip_lf_status(t) == URCU_FLIP_LF_SUCCEEDED;
	call_rcu_fn(&t->rcu_head, urcu_flip_lf_txn_free_rcu);
	return committed;
}

#ifdef __cplusplus
}
#endif

#endif	/* _URCU_FLIP_LATCH_LOCKFREE_H */
