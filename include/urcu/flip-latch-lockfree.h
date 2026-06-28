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
 * transition is a descriptor-naming CAS: a parked slot names a descriptor, and
 * resolution goes through that descriptor's status word, so the slot's plain
 * value alone is never load-bearing (one txn's new is the next txn's old -- only
 * the descriptor identity disambiguates).  Combined with the per-record install
 * latch (below), this makes the engine indifferent to slot-value A-B-A: a
 * straggler can never drag a slot backward by re-planting a descriptor after it
 * linearized, no matter how the slot value recurred.
 *
 * Resolution.  A reader (or a writer traversing) that loads a slot holding
 * flip(r) reads r->mcas->status: SUCCEEDED resolves to r->new_ptr, UNDECIDED or
 * FAILED to r->old_ptr.  Because resolution goes through the status word, planting
 * a terminal descriptor in a slot is *correct*, not a bug -- which is why no
 * RDCSS is needed.
 *
 * Settle is owner-driven, with one exception.  A helper drives a foreign
 * transaction's *install* forward (so a stalled writer never blocks others); the
 * bulk settle that rewrites a transaction's parked records back to plain values
 * is the owner's, in commit().  The one exception is the INSTALLER-SELF-SETTLE
 * (urcu_flip_lf_plant()): a driver that has just planted a record under its
 * install latch, and reads the transaction already terminal, converts THAT one
 * proxy immediately -- it must not leave a record it planted post-linearization
 * to linger past the owner's reclaim.  This is still a settle of the planter's
 * own (just-installed) record, never of a foreign one, and it only fires once the
 * transaction is terminal.  A terminal descriptor otherwise lingers in its slots
 * until the owner reclaims it; readers and contenders resolve it through the
 * status word in the meantime.
 *
 * A contended slot never reverts to plain while its transaction is still
 * UNDECIDED -- only a terminal transaction's slots decay to plain (by settle or
 * self-settle), and a higher-priority contender takes a slot it wants by STEAL
 * (one CAS replacing the proxy directly) rather than waiting for plain, so the
 * slot is handed owner-to-owner rather than momentarily up for grabs.  The cost
 * is that readers may resolve a lingering proxy rather than load a settled value;
 * an aborted transaction settles right after, so the window is short.
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
 * Slot-value A-B-A -- TOLERATED.  The engine omits RDCSS: it installs a
 * descriptor with a plain CAS conditioned on the word holding its expected old.
 * Were that the whole story it would be safe ONLY if a word could not cycle back
 * to a prior value between a thread reading that value and installing -- a delayed
 * (stale) install of an already-decided descriptor would otherwise pass its CAS
 * on the recurred old and re-plant the descriptor's proxy AFTER the transaction
 * linearized, resurrecting a stale value -> use-after-free.  Such recurrence is
 * real even under RCU, which prevents ADDRESS reuse but not VALUE recurrence: a
 * doubly-linked list's next-pointer cycles B -> X -> B (insert then delete of X)
 * with B a live successor RCU never frees.
 *
 * The PER-RECORD INSTALL LATCH (struct urcu_flip_lf_record::latch, installed)
 * closes that directly, so the engine no longer requires slots to be
 * non-ABA-able at all -- ANY slot-value A-B-A is safe, whatever recurs the value
 * (live-successor recurrence, a counter revisiting a number, an embedder's own
 * reuse).  Every plant -- plain install and steal alike -- runs through
 * urcu_flip_lf_plant() under the record's latch, which makes {install-once test,
 * plant CAS, installed-set, first self-settle} atomic.  A stale second install is
 * gated by the install-once FLAG, not by the slot value, so the recurred value is
 * irrelevant; the self-settle closes the matching install-vs-settle ordering
 * hole.  What the engine DOES still require is unrelated to slot values: tag bit 0
 * free, pairwise-distinct slots per txn, and -- the EXISTENCE model -- that a
 * descriptor reachable through a slot is reclaimed only after a grace period (a
 * helper/reader may dereference it).  Note this is value-CAS atomicity: a record
 * is validated to hold its old at the LINEARIZATION point, not to have been stable
 * throughout; an embedder needing the latter (snapshot/version semantics) layers
 * its own versioning on top, as with any value-based MCAS -- but that is a
 * stronger guarantee than memory safety, which holds unconditionally here.
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
#include <urcu/fair-mutex.h>		/* per-record install latch */

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
 * retry count the caller threads into urcu_flip_lf_mcas_create() drives this.
 */
#ifndef URCU_FLIP_LF_ESCALATE
#define URCU_FLIP_LF_ESCALATE 16
#endif

enum urcu_flip_lf_status {
	URCU_FLIP_LF_UNDECIDED = 0,
	URCU_FLIP_LF_SUCCEEDED = 1,
	URCU_FLIP_LF_FAILED    = 2,
};

struct urcu_flip_lf_mcas;

struct urcu_flip_lf_record {
	void **slot;			/* transacted word (bit 0 must be free) */
	void *old_ptr;			/* expected old value */
	void *new_ptr;			/* committed new value */
	struct urcu_flip_lf_mcas *mcas;	/* back-pointer (status + sibling records) */
	/*
	 * Per-record install latch.  Guards EXACTLY ONE thing: this record's
	 * install decision -- the install-once test, the single plant CAS, the
	 * "installed" set, and the installer's first ordered self-settle (see
	 * urcu_flip_lf_plant()).  It does NOT guard the slot: every operation
	 * that REMOVES or CONVERTS a parked proxy -- the regular settle, the
	 * installer self-settle, and a thief's steal that displaces the victim's
	 * proxy -- is a lock-free CAS handoff that takes no latch.  Because the
	 * latch is per-RECORD, two records sharing one slot (a thief's r and its
	 * victim's fr) transition that slot under DIFFERENT latches and so always
	 * resolve by CAS; keeping settle mutex-free is what keeps that boundary
	 * clean and the latch's scope minimal.  A thread holds at most one such
	 * latch at a time (helping/resolving runs outside it), so there is no
	 * hold-and-wait and no lock order to maintain.
	 */
	struct cds_fair_mutex latch;
	unsigned long installed;	/* set once, under latch, on first plant */
} __attribute__((aligned(16)));

/*
 * The records are stored inline after the header and a parked slot holds the
 * tagged address of one record (urcu_flip_lf_tag()).  The engine's HARD
 * requirement is only tag bit 0 of a record address; the wider guarantee below
 * is what lets an embedder share a transacted slot with its own tag bits.
 *
 * Each inline record is 16-byte aligned -- the recs[] offset and the record
 * stride are both multiples of 16 (and, recs[] being a member, the txn
 * itself is 16-byte aligned) -- so every inline record address has its low 4
 * bits free.  An embedder that tags a transacted slot with a wider type code
 * (e.g. the fractal trie's low-4-bit pointer tags) can thus park records
 * through this engine without losing tag room.  The descriptor is allocated
 * with posix_memalign(16) (urcu_flip_lf_mcas_alloc), so the 16-byte base
 * alignment holds PORTABLY -- not merely where plain malloc happens to return
 * blocks aligned to >= 16 (max_align_t is only 8 on some 32-bit ABIs).  The
 * static asserts below pin the record TYPE's layout (offset and stride both
 * multiples of 16) so that, given that aligned base, every inline record
 * inherits it; they constrain the struct, not the allocator.  (The record's
 * own alignment carries all of this -- the txn needs no alignment attribute
 * of its own; only records, never the txn or its status word, are tagged into
 * a slot.)
 */
struct urcu_flip_lf_mcas {
	unsigned long status;		/* enum urcu_flip_lf_status, CAS-updated */
	unsigned long retry;		/* aging priority: prior retries of this op */
	struct rcu_head rcu_head;	/* owner's deferred-free handle */
	unsigned int nr;
	unsigned int cap;
	struct urcu_flip_lf_record recs[];	/* frozen + slot-sorted at commit */
};

urcu_static_assert(!(offsetof(struct urcu_flip_lf_mcas, recs) % 16),
		"urcu_flip_lf_mcas.recs must be 16-byte aligned within the txn",
		urcu_flip_lf_mcas_recs_aligned);
urcu_static_assert(!(sizeof(struct urcu_flip_lf_record) % 16),
		"urcu_flip_lf_record stride must keep inline records 16-byte aligned",
		urcu_flip_lf_record_stride_aligned);
urcu_static_assert(!(__alignof__(struct urcu_flip_lf_mcas) % 16),
		"urcu_flip_lf_mcas must inherit 16-byte alignment from its recs[] member",
		urcu_flip_lf_mcas_aligned);

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
unsigned long urcu_flip_lf_status(struct urcu_flip_lf_mcas *t)
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
bool urcu_flip_lf_outranks(const struct urcu_flip_lf_mcas *a,
		const struct urcu_flip_lf_mcas *b)
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
	return urcu_flip_lf_status(r->mcas) == URCU_FLIP_LF_SUCCEEDED ?
			r->new_ptr : r->old_ptr;
}

/*
 * Optional test hook.  Compiles to nothing unless the embedder defines
 * URCU_FLIP_LF_PREINSTALL(t, r) before including this header.  Fires in the
 * install path at the point a driver has decided to plant @r, just BEFORE it
 * takes @r's install latch -- so a deterministic single-threaded test can
 * interpose the exact interleaving that used to re-plant (drive @t's own install
 * + commit + settle, then A-B-A the slot back to r->old_ptr) from inside the
 * hook, itself taking and releasing the latch, before the stale driver proceeds
 * into urcu_flip_lf_plant() and is stopped by the install-once flag.  Not part of
 * the engine contract.
 */
#ifndef URCU_FLIP_LF_PREINSTALL
#define URCU_FLIP_LF_PREINSTALL(t, r)	do { } while (0)
#endif

/*
 * Plant transaction @t's record @r into its slot under @r's install latch.
 * @expect is the slot value to CAS over: r->old_ptr for a plain install, or the
 * victim's parked proxy for a steal.  This is the ONLY writer of r->installed and
 * the ONLY latch-protected slot write; everything else that touches the slot
 * (regular settle, a thief's steal of THIS proxy later) is a lock-free CAS.
 *
 * Three things happen atomically under the latch, which is exactly what closes
 * the re-plant use-after-free that a value-CAS alone cannot (the slot value can
 * A-B-A back to r->old_ptr after @t linearizes, so the CAS's own comparison is
 * not enough to tell a first install from a stale second one):
 *
 *   1. install-once -- if r was already planted (by us or another driver of @t),
 *      skip.  The flag, not the slot value, is the gate, so any A-B-A is inert.
 *   2. the single plant CAS.
 *   3. installer-self-settle -- read @t's status AFTER the plant.  If @t is
 *      already terminal, convert our just-planted proxy to its resolved value
 *      now, so it cannot linger past @t's reclaim (the install-vs-settle hole:
 *      a late first install after the owner already settled).  The status read
 *      ordered AFTER the plant (the plant CAS is a full barrier; the status load
 *      is acquire) is what manufactures the happens-before that makes the owner's
 *      unconditional settle in commit() the backstop for the UNDECIDED case.
 *
 * Returns:
 *   1 -- we planted the proxy (caller advances to the next record; the steal
 *        site also bumps its steal counter);
 *   2 -- r was already installed (caller advances; no steal counted);
 *   0 -- the CAS raced the slot away (caller re-reads and retries).
 */
static inline
int urcu_flip_lf_plant(struct urcu_flip_lf_mcas *t,
		struct urcu_flip_lf_record *r, void *expect)
{
	void *tagv = urcu_flip_lf_tag(r);
	struct cds_fair_mutex_node node;
	int ret;

	cds_fair_mutex_lock(&r->latch, &node);
	if (
#ifndef URCU_FLIP_LF_NO_ABA_FIX
	    uatomic_load(&r->installed, CMM_RELAXED)	/* install-once: the A-B-A fix */
#else
	    0			/* test knob: drop the install-once gate (buggy) */
#endif
	   ) {
		ret = 2;			/* already installed: skip */
	} else if (uatomic_cmpxchg(r->slot, expect, tagv) == expect) {
		uatomic_store(&r->installed, 1, CMM_RELAXED);
		/*
		 * Self-settle: read status AFTER the plant.  Correctness is an
		 * SB (store-buffer) exclusion -- the plant CAS above is a FULL
		 * barrier on success and the commit-flip + owner settle in
		 * commit() are full-barrier CASes, so it is impossible for BOTH
		 * "we read UNDECIDED here" AND "the owner's settle ran before our
		 * plant".  Hence either we self-settle now (status terminal), or
		 * the owner's settle, ordered after our plant, converts it.  The
		 * RELAXED installed accesses are fine: they are read/written only
		 * under this latch, which carries them release-to-acquire.
		 */
#ifndef URCU_FLIP_LF_NO_ABA_FIX
		{
			unsigned long st = urcu_flip_lf_status(t);

			if (st != URCU_FLIP_LF_UNDECIDED) {
				void *want = (st == URCU_FLIP_LF_SUCCEEDED) ?
						r->new_ptr : r->old_ptr;

				(void) uatomic_cmpxchg(r->slot, tagv, want);
			}
		}
#endif		/* else (test knob): drop the self-settle (buggy) */
		ret = 1;			/* planted */
	} else {
		ret = 0;			/* raced: retry */
	}
	cds_fair_mutex_unlock(&r->latch, &node);
#ifdef URCU_FLIP_LF_NO_ABA_FIX
	(void) t;		/* the self-settle, t's only use here, is gated out */
#endif
	return ret;
}

/*
 * Install phase: drive transaction @t to a terminal status (SUCCEEDED or
 * FAILED) by installing its records in slot-address order.  Each plant (plain
 * install or steal) goes through urcu_flip_lf_plant() under the record's install
 * latch (install-once); a plant that finds @t already terminal self-settles that
 * one record, but the bulk settle is left to the owner (urcu_flip_lf_settle), so
 * a terminal proxy may otherwise linger in a slot until its owner reclaims it;
 * readers and contenders resolve it through the status word rather than waiting
 * for a plain value.  Safe to call on one's own transaction or on any foreign one
 * met in a slot (helping its install forward, never bulk-settling it).  Idempotent
 * and re-entrant under the global install order: helping follows strictly
 * increasing slot addresses, so recursion is bounded by the number of
 * concurrently-conflicting transactions; a thread holds at most one record latch
 * at a time (helping/resolving runs outside the latch), so there is no
 * hold-and-wait.
 */
static inline
void urcu_flip_lf_drive_install(struct urcu_flip_lf_mcas *t)
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
			int planted;

			st = urcu_flip_lf_status(t);
			if (st != URCU_FLIP_LF_UNDECIDED)
				return;		/* decided by a helper/evictor */
			v = uatomic_load(r->slot, CMM_ACQUIRE);
			if (v == tagv)
				break;		/* already installed */
			if (urcu_flip_lf_is_proxy(v)) {
				struct urcu_flip_lf_record *fr =
					urcu_flip_lf_untag(v);
				struct urcu_flip_lf_mcas *e = fr->mcas;
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
				 * Steal E's terminal proxy: our parked record
				 * replaces it directly, never through a plain value
				 * a newcomer could grab.  Sound because the slot's
				 * logical value (resolved == r->old_ptr) is unchanged
				 * by the handoff.  Under r's install latch (NOT E's --
				 * displacing E's proxy is a lock-free CAS that races
				 * E's owner-only settle harmlessly), with install-once
				 * so a stale re-steal after @t linearizes is inert.
				 */
				URCU_FLIP_LF_PREINSTALL(t, r);
				planted = urcu_flip_lf_plant(t, r, v);
				if (planted) {
					if (planted == 1)
						URCU_FLIP_LF_STAT(steal);
					break;	/* installed (or already) */
				}
				continue;	/* raced (settled / stolen): re-read */
			}
			if (v != r->old_ptr) {
				/* read-set invalid -> abort this txn */
				uatomic_cmpxchg(&t->status,
					URCU_FLIP_LF_UNDECIDED,
					URCU_FLIP_LF_FAILED);
				return;
			}
			/*
			 * Plain install under r's latch: install-once + the
			 * self-settle make a stale second install after @t
			 * linearizes (slot A-B-A'd back to r->old_ptr) inert.
			 */
			URCU_FLIP_LF_PREINSTALL(t, r);
			planted = urcu_flip_lf_plant(t, r, r->old_ptr);
			if (planted)
				break;		/* installed (or already) */
			/* raced: slot changed under us -> re-evaluate */
		}
	}
	/* every record installed -> commit */
	uatomic_cmpxchg(&t->status, URCU_FLIP_LF_UNDECIDED,
			URCU_FLIP_LF_SUCCEEDED);
}

/*
 * Settle phase (owner, in commit()): make @t's own slots plain -- its new value
 * on SUCCEEDED, its old on FAILED.  Called once @t is terminal.  A slot a
 * higher-priority transaction already stole holds that thief's proxy, or one an
 * installer already self-settled holds the plain value, so those CASes just fail
 * and are ignored; once settle returns, no slot still names a record of @t, so
 * the descriptor can be reclaimed.  Runs WITHOUT the install latch -- it only
 * removes/converts proxies (a lock-free CAS handoff), never installs -- and is
 * idempotent.  It is the BACKSTOP for the install-vs-settle order: any record an
 * installer planted while @t was still UNDECIDED (so it did NOT self-settle) is
 * converted here, and the installer's status-read-after-plant (urcu_flip_lf_plant)
 * is what orders that plant before this settle.
 */
static inline
void urcu_flip_lf_settle(struct urcu_flip_lf_mcas *t)
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
		struct urcu_flip_lf_mcas *e;

		if (caa_likely(!urcu_flip_lf_is_proxy(v)))
			return v;
		e = urcu_flip_lf_untag(v)->mcas;
		if (urcu_flip_lf_status(e) != URCU_FLIP_LF_UNDECIDED)
			return urcu_flip_lf_resolve(v);	/* terminal: logical value */
		urcu_flip_lf_drive_install(e);		/* help decide, then re-read */
	}
}

/*
 * Allocate a descriptor blob of @size bytes, 16-byte aligned.  The descriptor
 * holds the inline records that get tagged into slots, so its base must be
 * 16-byte aligned for every inline record to keep its low 4 bits free (see the
 * layout note above struct urcu_flip_lf_mcas).  posix_memalign guarantees that
 * portably; plain malloc would only promise max_align_t (8 on some 32-bit
 * ABIs).  Returns NULL on OOM.
 */
static inline
struct urcu_flip_lf_mcas *urcu_flip_lf_mcas_alloc(size_t size)
{
	void *p;

	if (posix_memalign(&p, 16, size))
		return NULL;
	return (struct urcu_flip_lf_mcas *) p;
}

/*
 * Create a transaction with room for @cap records.  @retry is the caller's
 * aging-priority: the number of times this logical operation has already retried
 * (0 on the first attempt, incremented and passed back in on each retry).  A
 * higher value wins contended slots, so a starved operation eventually cannot be
 * bypassed -- see urcu_flip_lf_outranks().
 */
static inline
struct urcu_flip_lf_mcas *urcu_flip_lf_mcas_create(unsigned int cap,
		unsigned long retry)
{
	struct urcu_flip_lf_mcas *t;

	t = urcu_flip_lf_mcas_alloc(sizeof(*t) +
			(size_t) cap * sizeof(struct urcu_flip_lf_record));
	if (!t)
		return NULL;
	t->status = URCU_FLIP_LF_UNDECIDED;
	t->retry = retry;
	t->nr = 0;
	t->cap = cap;
	return t;
}

/*
 * Append one edge {*slot: old -> new}.  Before commit only; no install yet.
 * Returns false if the descriptor is full -- the caller grows it first
 * (urcu_flip_lf_mcas_grow) and retries.  The record's back-pointer (r->mcas) is
 * deliberately NOT set here: a pre-commit grow reallocs the descriptor and may
 * move it, which would strand any add-time back-pointer.  It is filled in once,
 * at commit, after the write-set has stopped growing (see mcas_commit).
 */
static inline
bool urcu_flip_lf_mcas_add(struct urcu_flip_lf_mcas *t, void **slot,
		void *old_ptr, void *new_ptr)
{
	struct urcu_flip_lf_record *r;

	if (t->nr == t->cap)
		return false;
	r = &t->recs[t->nr++];
	r->slot = slot;
	r->old_ptr = old_ptr;
	r->new_ptr = new_ptr;
	return true;
}

/*
 * Record edge {*slot: old -> new} keeping at most one record per @slot, so the
 * engine's distinct-slot precondition holds by construction.  If a record
 * already targets @slot (a prior store, or a load-validate guard), reconcile it
 * rather than append a duplicate: @old_ptr must equal the record's old_ptr
 * (else the caller read @slot twice and saw it move -- an inconsistent txn).
 * @upgrade picks new_ptr -- a store advances it to @new_ptr, a load-validate
 * leaves a pending write intact.  Returns false only when a new record is
 * needed and the descriptor is full; the caller grows and retries.
 */
static inline
bool urcu_flip_lf_mcas_record(struct urcu_flip_lf_mcas *t, void **slot,
		void *old_ptr, void *new_ptr, int upgrade)
{
	unsigned int i;

	for (i = 0; i < t->nr; i++) {
		if (t->recs[i].slot != slot)
			continue;
		urcu_assert_debug(t->recs[i].old_ptr == old_ptr);
		if (upgrade)
			t->recs[i].new_ptr = new_ptr;
		return true;
	}
	return urcu_flip_lf_mcas_add(t, slot, old_ptr, new_ptr);
}

/*
 * Grow @t's record capacity (room for at least one more), returning the
 * possibly-moved descriptor, or NULL on OOM with @t left intact for the caller
 * to free.  Valid only before commit: the descriptor is not yet parked in any
 * slot, so it may move freely.  There is no aligned realloc, so rather than
 * realloc (which would preserve only malloc's max_align_t alignment, not the
 * 16-byte tag room) we allocate a fresh 16-byte-aligned block, copy the header
 * and the records buffered so far, and free the old one.  Back-pointers are set
 * at commit, after the last grow, so a move here strands nothing.
 */
static inline
struct urcu_flip_lf_mcas *urcu_flip_lf_mcas_grow(struct urcu_flip_lf_mcas *t)
{
	unsigned int newcap = t->cap < 2 ? 2 : t->cap * 2;
	struct urcu_flip_lf_mcas *n;

	n = urcu_flip_lf_mcas_alloc(sizeof(*t) +
			(size_t) newcap * sizeof(struct urcu_flip_lf_record));
	if (!n)
		return NULL;
	memcpy(n, t, sizeof(*t) +
			(size_t) t->nr * sizeof(struct urcu_flip_lf_record));
	n->cap = newcap;
	free(t);
	return n;
}

static inline
void urcu_flip_lf_mcas_destroy(struct urcu_flip_lf_mcas *t)
{
	free(t);
}

/* call_rcu callback: deferred destroy. */
static inline
void urcu_flip_lf_mcas_free_rcu(struct rcu_head *head)
{
	urcu_flip_lf_mcas_destroy(caa_container_of(head,
			struct urcu_flip_lf_mcas, rcu_head));
}

/* Sort records by slot address (insertion sort -- transactions are small). */
static inline
void urcu_flip_lf_mcas_sort(struct urcu_flip_lf_mcas *t)
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
bool urcu_flip_lf_mcas_commit(struct urcu_flip_lf_mcas *t,
		void (*call_rcu_fn)(struct rcu_head *,
			void (*)(struct rcu_head *)))
{
	bool committed;
	unsigned int i;

	if (t->nr == 0) {
		urcu_flip_lf_mcas_destroy(t);
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
		urcu_flip_lf_mcas_destroy(t);
		return committed;
	}
	if (t->nr == 1)
		URCU_FLIP_LF_STAT(escalate);	/* single edge, retried past the threshold */
	urcu_flip_lf_mcas_sort(t);
	/*
	 * Engine precondition: a transaction's records must target pairwise-
	 * distinct slots (the install/steal protocol's "own proxy"
	 * short-circuit and the read-set checks assume it).  After the
	 * slot-address sort a duplicate shows up as an adjacent equal slot, so
	 * a single linear pass catches an embedder that buffered the same slot
	 * twice.  Debug-only: no cost under NDEBUG.
	 */
	for (i = 1; i < t->nr; i++)
		urcu_assert_debug(t->recs[i].slot != t->recs[i - 1].slot);
	/*
	 * Set the record back-pointers now, deferred from add time, and arm each
	 * record's install latch (install-once flag clear, fair-mutex empty).
	 * The write-set may have been grown (realloc'd, hence moved) while it was
	 * being buffered; from here the descriptor is frozen and about to be
	 * parked, so every record can finally name its now-stable descriptor.
	 * Arming here (post-grow, pre-install) is correct: no driver can take the
	 * latch until the descriptor is parked, which only drive_install below
	 * does.
	 */
	for (i = 0; i < t->nr; i++) {
		t->recs[i].mcas = t;
		t->recs[i].installed = 0;
		cds_fair_mutex_init(&t->recs[i].latch);
	}
	urcu_flip_lf_drive_install(t);		/* install to a decision (helpers help) */
	urcu_flip_lf_settle(t);			/* owner-only: make our own slots plain */
	committed = urcu_flip_lf_status(t) == URCU_FLIP_LF_SUCCEEDED;
	call_rcu_fn(&t->rcu_head, urcu_flip_lf_mcas_free_rcu);
	return committed;
}

#ifdef __cplusplus
}
#endif

#endif	/* _URCU_FLIP_LATCH_LOCKFREE_H */
