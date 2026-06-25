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
 * Settle is owner-only.  A helper drives a foreign transaction's *install*
 * forward (so a stalled writer never blocks others) but never settles it: only
 * the owner, in commit(), rewrites its own parked records back to plain values.
 * A terminal descriptor therefore lingers in its slots until the owner reclaims
 * it; readers and contenders resolve it through the status word in the meantime.
 * The payoff is that a contended slot never reverts to a plain value mid-episode
 * -- so it is never momentarily up for grabs by a newcomer's plain CAS -- it is
 * handed owner-to-owner by steals and only decays to plain once quiescent.  The
 * cost is that readers resolve a lingering proxy rather than load a settled
 * value; an aborted transaction settles right after, so the window is short.
 *
 * Liveness.  Records install in one global order (sorted by slot address), so
 * two conflicting transactions always meet at their lowest shared slot.  There
 * a strict per-transaction priority picks the winner -- the transaction that
 * has retried more, ties broken by descriptor address (urcu_flip_lf_outranks());
 * a starved transaction's priority climbs until it can no longer be bypassed
 * (bounded bypass).  The lower-priority transaction is aborted and the winner
 * STEALS the slot in a single CAS -- its parked record replaces the loser's
 * directly, rather than the slot first reverting to a plain value a newcomer
 * could grab.  Because the priority order is total and both parties compute it
 * identically, the two never abort each other, so eviction cannot livelock.
 * This is lock-free, not wait-free.
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

/*
 * Optional instrumentation hook.  Compiles to nothing unless the embedder
 * defines URCU_FLIP_LF_STAT(counter) before including this header (the fairness
 * falsifier uses it to count helping/eviction/steal work).  Not part of the
 * engine contract.
 */
#ifndef URCU_FLIP_LF_STAT
#define URCU_FLIP_LF_STAT(counter)	do { } while (0)
#endif

/*
 * Single-edge escalation threshold.  A one-record transaction normally commits
 * with a bare CAS and no descriptor (cheap, but invisible to the priority /
 * steal protocol, so it can only win when the slot is plain -- it accrues no
 * priority and cannot break into a slot a multi-edge transaction keeps proxied).
 * Once such an op has retried this many times it stops taking the fast path and
 * commits through the full descriptor protocol instead, so it installs a real
 * proxy and contends on the same priority-ordered footing as everyone else (at
 * the cost of a descriptor and one grace period).  This lifts a *lockout*; it
 * does not make the op wait-free -- the engine is lock-free, not wait-free.  The
 * retry count the caller threads into urcu_flip_lf_txn_create() drives this.
 */
#ifndef URCU_FLIP_LF_ESCALATE
#define URCU_FLIP_LF_ESCALATE 16
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
} __attribute__((aligned(16)));

/*
 * The records are stored inline after the header and a parked slot holds the
 * tagged address of one record (urcu_flip_lf_tag()).  The engine itself owns
 * only tag bit 0, but each inline record is 16-byte aligned -- which makes the
 * recs[] offset and the record stride both multiples of 16, and (since recs[] is
 * a member) gives the txn itself 16-byte alignment -- so every inline record
 * address has its low 4 bits free.  An embedder that tags a transacted slot with
 * a wider type code (e.g. the fractal trie's low-4-bit pointer tags) can thus
 * park records through this engine without losing tag room.  The static asserts
 * below pin that guarantee against future field changes.  (The record's own
 * alignment carries all of this -- the txn needs no alignment attribute of its
 * own; only records, never the txn or its status word, are tagged into a slot.)
 */
struct urcu_flip_lf_txn {
	unsigned long status;		/* enum urcu_flip_lf_status, CAS-updated */
	unsigned long retry;		/* aging priority: prior retries of this op */
	struct rcu_head rcu_head;	/* owner's deferred-free handle */
	unsigned int nr;
	unsigned int cap;
	struct urcu_flip_lf_record recs[];	/* frozen + slot-sorted at commit */
};

urcu_static_assert(!(offsetof(struct urcu_flip_lf_txn, recs) % 16),
		"urcu_flip_lf_txn.recs must be 16-byte aligned within the txn",
		urcu_flip_lf_txn_recs_aligned);
urcu_static_assert(!(sizeof(struct urcu_flip_lf_record) % 16),
		"urcu_flip_lf_record stride must keep inline records 16-byte aligned",
		urcu_flip_lf_record_stride_aligned);
urcu_static_assert(!(__alignof__(struct urcu_flip_lf_txn) % 16),
		"urcu_flip_lf_txn must inherit 16-byte alignment from its recs[] member",
		urcu_flip_lf_txn_aligned);

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
 * Strict total priority order over two distinct transactions: the one that has
 * retried more wins; ties break by descriptor address (lower wins).  Both
 * fields are immutable for a descriptor's life, so the two parties to a slot
 * conflict compute the identical verdict -- the order is antisymmetric, so they
 * never each abort the other (which would livelock).  Returns true if @a
 * outranks @b.  @a and @b must differ (a txn never conflicts with itself: its
 * records target pairwise-distinct slots).
 */
static inline
bool urcu_flip_lf_outranks(const struct urcu_flip_lf_txn *a,
		const struct urcu_flip_lf_txn *b)
{
	if (a->retry != b->retry)
		return a->retry > b->retry;
	return (uintptr_t) a < (uintptr_t) b;
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
 * Install phase: drive transaction @t to a terminal status (SUCCEEDED or
 * FAILED) by installing its records in slot-address order.  Does NOT settle the
 * slots -- settle is owner-only (urcu_flip_lf_settle), so a terminal proxy may
 * linger in a slot until its owner reclaims it; readers and contenders resolve
 * it through the status word rather than waiting for a plain value.  Safe to
 * call on one's own transaction or on any foreign one met in a slot (helping
 * its install forward, never settling it).  Idempotent and re-entrant under the
 * global install order: helping follows strictly increasing slot addresses, so
 * recursion is bounded by the number of concurrently-conflicting transactions.
 */
static inline
void urcu_flip_lf_drive_install(struct urcu_flip_lf_txn *t)
{
	unsigned long st = urcu_flip_lf_status(t);
	unsigned int i;

	URCU_FLIP_LF_STAT(drive);

	if (st != URCU_FLIP_LF_UNDECIDED)
		return;			/* already terminal */
	for (i = 0; i < t->nr; i++) {
		struct urcu_flip_lf_record *r = &t->recs[i];
		void *tagv = urcu_flip_lf_tag(r);

		for (;;) {
			void *v;

			st = urcu_flip_lf_status(t);
			if (st != URCU_FLIP_LF_UNDECIDED)
				return;		/* decided by a helper/evictor */
			v = uatomic_load(r->slot, CMM_ACQUIRE);
			if (v == tagv)
				break;		/* already installed */
			if (urcu_flip_lf_is_proxy(v)) {
				struct urcu_flip_lf_record *fr =
					urcu_flip_lf_untag(v);
				struct urcu_flip_lf_txn *e = fr->txn;
				unsigned long est;
				void *resolved;

				if (e == t)
					break;	/* own proxy (distinct-slot inv.) */
				est = urcu_flip_lf_status(e);
				if (est == URCU_FLIP_LF_UNDECIDED) {
					if (!urcu_flip_lf_outranks(t, e)) {
						/* E outranks us: help it decide
						 * (install only), then re-read. */
						urcu_flip_lf_drive_install(e);
						continue;
					}
					/* We outrank E: evict the lower priority. */
					URCU_FLIP_LF_STAT(evict);
					uatomic_cmpxchg(&e->status,
						URCU_FLIP_LF_UNDECIDED,
						URCU_FLIP_LF_FAILED);
					est = urcu_flip_lf_status(e);
				}
				/*
				 * E is terminal now (possibly still unsettled --
				 * owner-only settle).  It resolves this slot to its
				 * new (SUCCEEDED) or old (FAILED).
				 */
				resolved = (est == URCU_FLIP_LF_SUCCEEDED) ?
						fr->new_ptr : fr->old_ptr;
				if (resolved != r->old_ptr) {
					/* read-set invalid -> abort this txn */
					uatomic_cmpxchg(&t->status,
						URCU_FLIP_LF_UNDECIDED,
						URCU_FLIP_LF_FAILED);
					return;
				}
				/*
				 * Steal E's terminal proxy in one CAS: our parked
				 * record replaces it directly, never through a plain
				 * value a newcomer could grab.  Sound because the
				 * slot's logical value (resolved == r->old_ptr) is
				 * unchanged by the handoff.
				 */
				if (uatomic_cmpxchg(r->slot, v, tagv) == v) {
					URCU_FLIP_LF_STAT(steal);
					break;	/* installed */
				}
				continue;	/* raced (owner settled / stolen): re-read */
			}
			if (v != r->old_ptr) {
				/* read-set invalid -> abort this txn */
				uatomic_cmpxchg(&t->status,
					URCU_FLIP_LF_UNDECIDED,
					URCU_FLIP_LF_FAILED);
				return;
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

/*
 * Settle phase (owner-only): make @t's own slots plain -- its new value on
 * SUCCEEDED, its old on FAILED.  Called from commit() once @t is terminal.  A
 * slot a higher-priority transaction already stole holds that thief's proxy, so
 * the CAS just fails and is ignored; once settle returns, no slot still names a
 * record of @t, so the descriptor can be reclaimed.  Idempotent.
 */
static inline
void urcu_flip_lf_settle(struct urcu_flip_lf_txn *t)
{
	unsigned long st = urcu_flip_lf_status(t);
	unsigned int i;

	for (i = 0; i < t->nr; i++) {
		struct urcu_flip_lf_record *r = &t->recs[i];
		void *want = (st == URCU_FLIP_LF_SUCCEEDED) ? r->new_ptr : r->old_ptr;

		(void) uatomic_cmpxchg(r->slot, urcu_flip_lf_tag(r), want);
	}
}

/*
 * Load @slot and return the value it currently denotes: a plain value as-is, or,
 * for a parked record, the value resolved through its transaction's status.  If
 * that transaction is still UNDECIDED, help drive its install to a decision
 * first (so the returned value is stable), but never settle it -- the proxy is
 * left in place (owner-only settle) and the value returned is the *logical* one.
 * Use this to read the current value of a word you intend to transact (its old);
 * commit() then reconciles that logical old against whatever physical value --
 * plain or a foreign proxy -- the slot holds at install time.  Call within an
 * RCU read-side section.
 */
static inline
void *urcu_flip_lf_read(void **slot)
{
	for (;;) {
		void *v = uatomic_load(slot, CMM_ACQUIRE);
		struct urcu_flip_lf_txn *e;

		if (caa_likely(!urcu_flip_lf_is_proxy(v)))
			return v;
		e = urcu_flip_lf_untag(v)->txn;
		if (urcu_flip_lf_status(e) != URCU_FLIP_LF_UNDECIDED)
			return urcu_flip_lf_resolve(v);	/* terminal: logical value */
		urcu_flip_lf_drive_install(e);		/* help decide, then re-read */
	}
}

/*
 * Create a transaction with room for @cap records.  @retry is the caller's
 * aging-priority: the number of times this logical operation has already retried
 * (0 on the first attempt, incremented and passed back in on each retry).  A
 * higher value wins contended slots, so a starved operation eventually cannot be
 * bypassed -- see urcu_flip_lf_outranks().
 */
static inline
struct urcu_flip_lf_txn *urcu_flip_lf_txn_create(unsigned int cap,
		unsigned long retry)
{
	struct urcu_flip_lf_txn *t;

	t = (struct urcu_flip_lf_txn *) malloc(sizeof(*t) +
			(size_t) cap * sizeof(struct urcu_flip_lf_record));
	if (!t)
		return NULL;
	t->status = URCU_FLIP_LF_UNDECIDED;
	t->retry = retry;
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
 * through @call_rcu_fn (the flavor's call_rcu); an un-escalated single-edge
 * transaction commits with a bare CAS and frees immediately (no proxy, no grace
 * period).  A single-edge op that has retried past URCU_FLIP_LF_ESCALATE falls
 * through to the full descriptor path so it installs a real proxy and contends
 * on the same priority-ordered footing as everyone else, instead of being shut
 * out of a slot a multi-edge op keeps proxied.  (This lifts a lockout; like any
 * transaction here it remains lock-free, not wait-free.)
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
	if (t->nr == 1 && t->retry < URCU_FLIP_LF_ESCALATE) {
		/*
		 * Single edge, not yet starved: the CAS itself is the atomic
		 * commit.  Fast and -- while the slot is plain -- fair; once it
		 * has retried enough to suspect a proxy is locking it out, the
		 * escalation below makes it a real, visible transaction instead.
		 */
		struct urcu_flip_lf_record *r = &t->recs[0];

		committed = uatomic_cmpxchg(r->slot, r->old_ptr, r->new_ptr) == r->old_ptr;
		urcu_flip_lf_txn_destroy(t);
		return committed;
	}
	if (t->nr == 1)
		URCU_FLIP_LF_STAT(escalate);	/* single edge, retried past the threshold */
	urcu_flip_lf_txn_sort(t);
	urcu_flip_lf_drive_install(t);		/* install to a decision (helpers help) */
	urcu_flip_lf_settle(t);			/* owner-only: make our own slots plain */
	committed = urcu_flip_lf_status(t) == URCU_FLIP_LF_SUCCEEDED;
	call_rcu_fn(&t->rcu_head, urcu_flip_lf_txn_free_rcu);
	return committed;
}

#ifdef __cplusplus
}
#endif

#endif	/* _URCU_FLIP_LATCH_LOCKFREE_H */
