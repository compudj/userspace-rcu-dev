// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

#ifndef _URCU_RCU_MCAS_H
#define _URCU_RCU_MCAS_H

/*
 * RCU MCAS: a multi-word compare-and-swap (k-CAS) engine.
 *
 * This is the concurrent-writer sibling of <urcu/rcu-txn-sw.h>.  It switches
 * a *set* of words from their old values to new ones atomically, as observed
 * by both concurrent RCU readers AND concurrent writers.  Progress is
 * bounded-blocking: a thread that trips over an in-flight transaction helps
 * drive its install forward rather than waiting on it, and the only blocking
 * is a short per-record install word (a tri-state try-lock, below), held only
 * across a bounded, non-blocking section -- no unbounded waiting, no deadlock.
 *
 * Model (a "practical MCAS", Harris-style, linearized by a status flip)
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
 * is the owner's, in commit().  The owner's settle also CLAIMS each record's
 * install word (FREE -> DONE) before converting its slot, so once settle returns
 * no plant can begin at all -- that claim, not the plant-side checks, is what
 * makes the post-settle reclaim safe against a stale FIRST install of a record
 * nobody had planted yet (see urcu_mcas_settle()).  The one exception to
 * owner-driven settling is the INSTALLER-SELF-SETTLE (urcu_mcas_plant()): a
 * driver that has just planted a record under its install latch, and reads the
 * transaction already terminal, converts THAT one proxy immediately -- before
 * releasing the latch to DONE -- so the owner's settle, spinning out the BUSY
 * window, finds the slot already plain and a post-decision plant never outlives
 * it.  This is still a settle of the planter's own (just-installed) record,
 * never of a foreign one, and it only fires once the
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
 * has retried more, ties broken by descriptor address (urcu_mcas_outranks());
 * a starved transaction's priority climbs until it can no longer be bypassed
 * (bounded bypass).  The lower-priority transaction is aborted and the winner
 * STEALS the slot in a single CAS -- its parked record replaces the loser's
 * directly, rather than the slot first reverting to a plain value a newcomer
 * could grab.  Because the priority order is total and both parties compute it
 * identically, the two never abort each other, so eviction cannot livelock.
 * The engine is bounded-blocking (the per-record install latch), not lock-free.
 *
 * Existence.  Mutators run as RCU readers (rcu_read_lock around the whole
 * operation).  Any descriptor a helper reaches through a slot stays alive
 * until the helper leaves its read-side section; the owner call_rcu()s the
 * descriptor once it is terminal and fully settled.  No refcount.
 *
 * Constraints (vs the single-writer <urcu/rcu-txn-sw.h>):
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
 * The PER-RECORD INSTALL WORD (struct urcu_mcas_record::state -- a tri-state
 * FREE/BUSY/DONE try-lock) closes that directly, so the engine no longer requires slots to be
 * non-ABA-able at all -- ANY slot-value A-B-A is safe, whatever recurs the value
 * (live-successor recurrence, a counter revisiting a number, an embedder's own
 * reuse).  Every plant -- plain install and steal alike -- runs through
 * urcu_mcas_plant(), which claims the word (FREE->BUSY) so {install-once, plant
 * CAS, self-settle} are atomic in one word.  A stale second install is
 * gated by the DONE state, not by the slot value, so the recurred value is
 * irrelevant; and the owner's settle claims every record's install word
 * (FREE->DONE) before the descriptor is reclaimed, so a stale FIRST install of a
 * never-planted record is gated the same way -- after settle, no plant of any
 * kind can republish the descriptor (see urcu_mcas_settle()); the self-settle
 * keeps a post-decision plant from lingering until settle reaches it.  What the
 * engine DOES still require is unrelated to slot values: tag bit 0
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
#include <urcu/arch.h>			/* caa_cpu_relax */
#include <urcu/rcu-txn-slab.h>		/* shared per-CPU size-classed descriptor slab */

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Optional instrumentation hook.  Compiles to nothing unless the embedder
 * defines URCU_MCAS_STAT(counter) before including this header (the fairness
 * falsifier uses it to count helping/eviction/steal work).  Not part of the
 * engine contract.
 */
#ifndef URCU_MCAS_STAT
#define URCU_MCAS_STAT(counter)	do { } while (0)
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
 * does not make the op wait-free -- the engine is bounded-blocking, not wait-free.  The
 * retry count the caller threads into urcu_mcas_create() drives this.
 */
#ifndef URCU_MCAS_ESCALATE
#define URCU_MCAS_ESCALATE 16
#endif

/*
 * Help-recursion depth cap.  drive_install() recurses to push a higher-priority
 * blocking transaction forward (see urcu_mcas_drive_install_depth()).  That
 * recursion is acyclic -- it follows the strict total priority order, so it can
 * only descend into a strictly-higher-priority transaction -- and is therefore
 * bounded by the number of concurrently-conflicting transactions; but that bound
 * is realized as real C-stack frames and scales with the writer count.
 *
 * Past this depth a helper does NOT spin on the blocker.  Spinning here would be
 * an unbounded wait on the blocker's *entire* transaction (arbitrary record
 * count, across arbitrarily many preemptions of its owner) -- categorically
 * unlike the install-latch spin, whose few-instruction window rseq time-slice
 * extension can cover; TSE does nothing for a whole-transaction-length wait.
 * Instead the capped helper ESCALATES: it CASes FAILED on the transaction it is
 * currently driving -- its own at depth 0; at deeper levels the intermediate
 * FOREIGN transaction it was helping (whose owner simply retries).  The abort
 * unwinds the descent, and an aborted own transaction retries with a higher
 * aging-priority; once the retrying
 * transaction out-retries the blocker it outranks it and evicts rather than helps,
 * so progress comes from the transaction's own escalation -- never from the
 * blocker's owner being scheduled.  Sound because the globally highest-priority
 * transaction never recurses (it always evicts), and ANY UNDECIDED transaction may
 * spuriously abort; helping is a latency optimization, not a progress requirement.
 */
#ifndef URCU_MCAS_HELP_MAX_DEPTH
#define URCU_MCAS_HELP_MAX_DEPTH 8
#endif

enum urcu_mcas_status {
	URCU_MCAS_UNDECIDED = 0,
	URCU_MCAS_SUCCEEDED = 1,
	URCU_MCAS_FAILED    = 2,
};

struct urcu_mcas;

struct urcu_mcas_record {
	void **slot;			/* transacted word (bit 0 must be free) */
	void *old_ptr;			/* expected old value */
	void *new_ptr;			/* committed new value */
	uintptr_t proxy_tag;		/* embedder's tag bits for THIS record's slot.
					 * The parked proxy value is (&record | proxy_tag),
					 * the record is recovered as (value & ~proxy_tag),
					 * and a value is this record's proxy iff
					 * (value & proxy_tag) == proxy_tag.  Carried per
					 * record -- not a per-TU macro -- so heterogeneous
					 * slots (e.g. fractal-trie 0xF vs a list's bit 0)
					 * share one engine, and a helper finishing a foreign
					 * txn untags/installs each foreign record through
					 * that record's own tag. */
	struct urcu_mcas *mcas;	/* back-pointer (status + sibling records) */
	/*
	 * Per-record install word -- one tri-state int making {install-once, plant
	 * CAS, self-settle} atomic without a separate flag or lock library:
	 *
	 *     FREE --(try-CAS)--> BUSY --(plant)--> DONE
	 *     FREE --(settle claim)--------------> DONE
	 *
	 * The FREE->BUSY CAS is the install lock; DONE is the install-once gate.  It
	 * guards EXACTLY this record's install, NOT the slot: the ops that REMOVE or
	 * CONVERT a parked proxy -- the installer self-settle and a thief's steal
	 * that displaces the victim's proxy -- are plain CASes that touch no install
	 * word.  The owner's settle is the exception: before converting each slot it
	 * CLAIMS that record's word (FREE -> DONE, second arc above), so once settle
	 * returns no plant -- even a stale FIRST install of a never-planted record --
	 * can begin and republish a retired descriptor (see urcu_mcas_settle()).
	 * Because the word is per-RECORD, two records sharing
	 * one slot (a thief's r and its victim's fr) transition it via DIFFERENT words
	 * and so always resolve by CAS.  A thread owns at most one install (BUSY) at a
	 * time and never blocks while owning it -- no hold-and-wait, no lock order.
	 */
	int state;
} __attribute__((aligned(16)));

/* Per-record install word values + initializer (see struct urcu_mcas_record). */
enum {
	URCU_MCAS_INSTALL_FREE = 0,	/* installable; a FREE->BUSY CAS claims it */
	URCU_MCAS_INSTALL_BUSY = 1,	/* a driver owns the install (try-lock held) */
	URCU_MCAS_INSTALL_DONE = 2,	/* planted, or claimed by settle (the install-once gate) */
};
#define urcu_mcas_latch_init(r)	\
	uatomic_store(&(r)->state, URCU_MCAS_INSTALL_FREE, CMM_RELAXED)

/*
 * The records are stored inline after the header and a parked slot holds the
 * tagged address of one record (urcu_mcas_tag()).  The engine's HARD
 * requirement is only tag bit 0 of a record address; the wider guarantee below
 * is what lets an embedder share a transacted slot with its own tag bits.
 *
 * Each inline record is 16-byte aligned -- the recs[] offset and the record
 * stride are both multiples of 16 (and, recs[] being a member, the txn
 * itself is 16-byte aligned) -- so every inline record address has its low 4
 * bits free.  An embedder that tags a transacted slot with a wider type code
 * (e.g. the fractal trie's low-4-bit pointer tags) can thus park records
 * through this engine without losing tag room.  The descriptor is allocated
 * with posix_memalign(16) (urcu_mcas_alloc), so the 16-byte base
 * alignment holds PORTABLY -- not merely where plain malloc happens to return
 * blocks aligned to >= 16 (max_align_t is only 8 on some 32-bit ABIs).  The
 * static asserts below pin the record TYPE's layout (offset and stride both
 * multiples of 16) so that, given that aligned base, every inline record
 * inherits it; they constrain the struct, not the allocator.  (The record's
 * own alignment carries all of this -- the txn needs no alignment attribute
 * of its own; only records, never the txn or its status word, are tagged into
 * a slot.)
 */
struct urcu_mcas {
	unsigned long status;		/* enum urcu_mcas_status, CAS-updated */
	unsigned long retry;		/* aging priority: prior retries of this op */
	struct rcu_head rcu_head;	/* owner's deferred-free handle */
	unsigned int nr;
	unsigned int cap;
	unsigned int poisoned;		/* set if a same-slot reconcile disagreed on old */
	unsigned int slab;		/* block origin: per-CPU slab (1) or exact
					 * posix_memalign (0).  Stamped at alloc and
					 * consulted by free -- self-describing, so a
					 * descriptor allocated while the slab was still
					 * uninitialized (constructor ordering) or disabled
					 * is freed on the right path even if the slab
					 * enables in between.  Occupies what was padding:
					 * recs[] stays 16-byte aligned (asserted below). */
	struct urcu_mcas_record recs[];	/* frozen + slot-sorted at commit */
};

urcu_static_assert(!(offsetof(struct urcu_mcas, recs) % 16),
		"urcu_mcas.recs must be 16-byte aligned within the txn",
		urcu_mcas_recs_aligned);
urcu_static_assert(!(sizeof(struct urcu_mcas_record) % 16),
		"urcu_mcas_record stride must keep inline records 16-byte aligned",
		urcu_mcas_record_stride_aligned);
urcu_static_assert(!(__alignof__(struct urcu_mcas) % 16),
		"urcu_mcas must inherit 16-byte alignment from its recs[] member",
		urcu_mcas_aligned);

/* A convenient default tag (bit 0) for an embedder that keeps bit 0 free. */
#define URCU_MCAS_TAG	1UL

/*
 * Proxy tag scheme -- carried PER RECORD (urcu_mcas_record.proxy_tag, set from
 * the urcu_mcas_add() argument), not as a per-TU macro.  A parked slot value is
 * (record address | tag); a record is 16-byte aligned, so its low 4 bits are
 * free for the embedder's tag.  Storing the TAG (not a pre-tagged pointer) and
 * forming the proxy from the record's CURRENT address at install is what makes a
 * descriptor grow/realloc safe: the tag travels with the record content through
 * the move, and the installer re-derives the proxy from the record's final
 * address, so nothing is stranded.  Passing the tag explicitly lets heterogeneous
 * slots share one engine -- the fractal trie tags with its type-7 / 0xF low
 * nibble (a value no real node, NULL or skip pointer carries), a doubly-linked
 * list with bit 0, etc.  The contract a tag must satisfy: (live_value & tag) !=
 * tag for EVERY non-proxy value the embedder stores in a transacted slot, so the
 * engine never mistakes a live value for one of its records.
 *
 * urcu_mcas_tag()/untag() take @tag explicitly; the installer passes the record's
 * own r->proxy_tag, and a reader/helper passes the (agreed) tag of the slot it is
 * resolving.
 */
static inline
void *urcu_mcas_tag(struct urcu_mcas_record *r, uintptr_t tag)
{
	return (void *) ((uintptr_t) r | tag);
}

static inline
struct urcu_mcas_record *urcu_mcas_untag(void *v, uintptr_t tag)
{
	return (struct urcu_mcas_record *) ((uintptr_t) v & ~tag);
}

/*
 * @v is a parked record iff it carries all of @tag's bits -- equivalently
 * v == (urcu_mcas_untag(v, tag) | tag), i.e. @v is some record address OR'd with
 * @tag.
 */
static inline
int urcu_mcas_is_proxy(void *v, uintptr_t tag)
{
	return ((uintptr_t) v & tag) == tag;
}

static inline
unsigned long urcu_mcas_status(struct urcu_mcas *t)
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
bool urcu_mcas_outranks(const struct urcu_mcas *a,
		const struct urcu_mcas *b)
{
	if (a->retry != b->retry)
		return a->retry > b->retry;
	return (uintptr_t) a < (uintptr_t) b;
}

/*
 * Resolve a parked record directly to the value it currently denotes (its
 * transaction's new on success, old otherwise).  Split out of
 * urcu_mcas_resolve() for an embedder that applies its OWN (wider) tag: it
 * untags with its own mask -- so it already holds the record, not the
 * engine-tagged slot value -- and resolves through this.  Call from within an
 * RCU read-side section.
 */
static inline
void *urcu_mcas_resolve_record(struct urcu_mcas_record *r)
{
	return urcu_mcas_status(r->mcas) == URCU_MCAS_SUCCEEDED ?
			r->new_ptr : r->old_ptr;
}

/*
 * Resolve a value loaded from a transacted slot to the value it currently
 * denotes.  A plain value passes through; a parked record resolves through its
 * transaction's status word.  Call from within an RCU read-side section.
 */
static inline
void *urcu_mcas_resolve(void *v, uintptr_t tag)
{
	if (caa_likely(!urcu_mcas_is_proxy(v, tag)))
		return v;
	return urcu_mcas_resolve_record(urcu_mcas_untag(v, tag));
}

/*
 * Optional test hook.  Compiles to nothing unless the embedder defines
 * URCU_MCAS_PREINSTALL(t, r) before including this header.  Fires in the
 * install path at the point a driver has decided to plant @r, just BEFORE it
 * claims @r's install word -- so a deterministic single-threaded test can
 * interpose the exact interleaving that used to re-plant (drive @t's own install
 * + commit + settle, then A-B-A the slot back to r->old_ptr) from inside the
 * hook, itself driving @r's install, before the stale driver proceeds
 * into urcu_mcas_plant() and is stopped by the DONE state.  Not part of
 * the engine contract.
 */
#ifndef URCU_MCAS_PREINSTALL
#define URCU_MCAS_PREINSTALL(t, r)	do { } while (0)
#endif

/*
 * Plant transaction @t's record @r: claim @r's install word (FREE->BUSY), CAS
 * @r's slot from @expect (r->old_ptr for a plain install, the victim's parked
 * proxy for a steal) to the record's tagged proxy, then publish DONE.  A later
 * steal of THIS proxy is a plain CAS that takes no install word; the owner's
 * settle CLAIMS the word (FREE->DONE) before its own slot CAS, so plant and
 * settle serialize on the word, never on the slot value.
 *
 * The install word makes {install-once, plant CAS, self-settle} atomic, which is
 * exactly what closes the re-plant use-after-free a value-CAS alone cannot (the
 * slot value can A-B-A back to r->old_ptr after @t linearizes, so the CAS's own
 * comparison cannot tell a first install from a stale second one):
 *
 *   1. install-once -- DONE means installed OR settle-claimed; a stale driver's
 *      FREE->BUSY fails, so any A-B-A of the slot VALUE is inert (the word, not
 *      the value, gates), and no plant at all can begin once @t's settle has
 *      claimed the word (a stale FIRST install lands here too).
 *   2. the single plant CAS, owned exclusively while BUSY.
 *   3. installer-self-settle -- read @t's status AFTER the plant; if @t is
 *      already terminal, convert our just-planted proxy now, before releasing
 *      the word to DONE, so the owner's settle (which spins out our BUSY window
 *      before claiming) finds the slot already plain -- a post-decision plant
 *      never outlives urcu_mcas_settle().  The status read ordered
 *      after the full-barrier plant makes the owner's unconditional settle in
 *      commit() the backstop for the UNDECIDED case.
 *
 * Returns:
 *   1 -- we planted the proxy (caller advances; the steal site bumps its counter);
 *   2 -- @r was already installed (caller advances; no steal counted);
 *   0 -- the CAS raced the slot away (caller re-reads and retries).
 */
#ifdef URCU_MCAS_NO_ABA_FIX
static inline
int urcu_mcas_plant(struct urcu_mcas *t,
		struct urcu_mcas_record *r, void *expect)
{
	void *tagv = urcu_mcas_tag(r, r->proxy_tag);

	/*
	 * Test knob: a bare value-CAS with no install-once gate, no self-settle,
	 * and no settle-time claim (urcu_mcas_settle) -- the original A-B-A-unsafe
	 * behaviour the install word closes.  For the
	 * regression tests (build -DURCU_MCAS_NO_ABA_FIX); never in production.
	 */
	(void) t;
	if (uatomic_cmpxchg(r->slot, expect, tagv) == expect)
		return 1;
	return 0;
}
#else
static inline
int urcu_mcas_plant(struct urcu_mcas *t,
		struct urcu_mcas_record *r, void *expect)
{
	void *tagv = urcu_mcas_tag(r, r->proxy_tag);
	int v;

	/* Claim r's install: FREE -> BUSY (acquire). */
	for (;;) {
		v = uatomic_cmpxchg(&r->state, URCU_MCAS_INSTALL_FREE,
				URCU_MCAS_INSTALL_BUSY);
		if (v == URCU_MCAS_INSTALL_DONE)
			return 2;		/* already installed: advance */
		if (v == URCU_MCAS_INSTALL_FREE)
			break;			/* we own r's install */
		/*
		 * BUSY: a co-driver of THIS txn is installing r.  Wait this attempt
		 * out -- bounded TTAS on a relaxed load (no cache-line RMW storm) --
		 * then re-decide.  BUSY is cooperative install progress, not a
		 * lost-slot conflict, so it never advances the retry counter.
		 */
		while (uatomic_load(&r->state, CMM_RELAXED) == URCU_MCAS_INSTALL_BUSY)
			caa_cpu_relax();
	}
	if (uatomic_cmpxchg(r->slot, expect, tagv) == expect) {
		/*
		 * Self-settle: status read ordered AFTER the full-barrier plant, so a
		 * proxy we plant after @t linearized is converted before we release
		 * DONE -- the owner's settle, claiming this word, then finds the slot
		 * already plain.
		 */
		unsigned long st = urcu_mcas_status(t);

		if (st != URCU_MCAS_UNDECIDED) {
			void *want = (st == URCU_MCAS_SUCCEEDED) ?
					r->new_ptr : r->old_ptr;

			(void) uatomic_cmpxchg(r->slot, tagv, want);
		}
		uatomic_store(&r->state, URCU_MCAS_INSTALL_DONE, CMM_RELEASE);
		return 1;			/* planted */
	}
	uatomic_store(&r->state, URCU_MCAS_INSTALL_FREE, CMM_RELEASE);
	return 0;				/* slot raced; r still un-installed */
}
#endif

/*
 * Install phase: drive transaction @t to a terminal status (SUCCEEDED or
 * FAILED) by installing its records in slot-address order.  Each plant (plain
 * install or steal) goes through urcu_mcas_plant() under the record's install
 * latch (install-once); a plant that finds @t already terminal self-settles that
 * one record, but the bulk settle is left to the owner (urcu_mcas_settle), so
 * a terminal proxy may otherwise linger in a slot until its owner reclaims it;
 * readers and contenders resolve it through the status word rather than waiting
 * for a plain value.  Safe to call on one's own transaction or on any foreign one
 * met in a slot (helping its install forward, never bulk-settling it).  Idempotent
 * and re-entrant under the global install order: helping follows strictly
 * increasing slot addresses, so recursion is bounded by the number of
 * concurrently-conflicting transactions; a thread holds at most one record latch
 * at a time (helping/resolving runs outside the latch), so there is no
 * hold-and-wait.  That bound is still O(writers) of real stack, so the recursive
 * descent is additionally capped at URCU_MCAS_HELP_MAX_DEPTH (see the note there);
 * @depth is the current help-recursion depth (0 at the owner/reader entry).
 */
static inline
void urcu_mcas_drive_install_depth(struct urcu_mcas *t, unsigned int depth)
{
	unsigned long st = urcu_mcas_status(t);
	unsigned int i;

	URCU_MCAS_STAT(drive);

	if (st != URCU_MCAS_UNDECIDED)
		return;			/* already terminal */
	for (i = 0; i < t->nr; i++) {
		struct urcu_mcas_record *r = &t->recs[i];
		void *tagv = urcu_mcas_tag(r, r->proxy_tag);

		for (;;) {
			void *v;
			int planted;

			st = urcu_mcas_status(t);
			if (st != URCU_MCAS_UNDECIDED)
				return;		/* decided by a helper/evictor */
			v = uatomic_load(r->slot, CMM_ACQUIRE);
			if (v == tagv)
				break;		/* already installed */
			if (urcu_mcas_is_proxy(v, r->proxy_tag)) {
				/*
				 * A foreign proxy in r->slot uses the same slot's
				 * (agreed) tag, so untag it with r->proxy_tag.
				 */
				struct urcu_mcas_record *fr =
					urcu_mcas_untag(v, r->proxy_tag);
				struct urcu_mcas *e = fr->mcas;
				unsigned long est;
				void *resolved;

				if (e == t)
					break;	/* own proxy (distinct-slot inv.) */
				est = urcu_mcas_status(e);
				if (est == URCU_MCAS_UNDECIDED) {
					if (!urcu_mcas_outranks(t, e)) {
						/*
						 * E outranks us: help it decide
						 * (install only), then re-read -- but
						 * cap the C-stack descent.  Past the cap
						 * we do NOT spin on E: that is an
						 * unbounded wait on E's whole transaction,
						 * not a TSE-coverable few-insn window like
						 * the install latch.  Escalate instead --
						 * abort T (FAILED) so the caller retries
						 * with a higher aging-priority; once T
						 * out-retries E it outranks and evicts E,
						 * so progress never hinges on E's owner
						 * being scheduled.
						 */
						if (depth >= URCU_MCAS_HELP_MAX_DEPTH) {
							URCU_MCAS_STAT(help_capped);
							uatomic_cmpxchg(&t->status,
								URCU_MCAS_UNDECIDED,
								URCU_MCAS_FAILED);
							return;
						}
						urcu_mcas_drive_install_depth(e,
							depth + 1);
						continue;
					}
					/* We outrank E: evict the lower priority. */
					URCU_MCAS_STAT(evict);
					uatomic_cmpxchg(&e->status,
						URCU_MCAS_UNDECIDED,
						URCU_MCAS_FAILED);
					est = urcu_mcas_status(e);
				}
				/*
				 * E is terminal now (possibly still unsettled --
				 * owner-only settle).  It resolves this slot to its
				 * new (SUCCEEDED) or old (FAILED).
				 */
				resolved = (est == URCU_MCAS_SUCCEEDED) ?
						fr->new_ptr : fr->old_ptr;
				if (resolved != r->old_ptr) {
					/* read-set invalid -> abort this txn */
					uatomic_cmpxchg(&t->status,
						URCU_MCAS_UNDECIDED,
						URCU_MCAS_FAILED);
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
				URCU_MCAS_PREINSTALL(t, r);
				planted = urcu_mcas_plant(t, r, v);
				if (planted) {
					if (planted == 1)
						URCU_MCAS_STAT(steal);
					break;	/* installed (or already) */
				}
				continue;	/* raced (settled / stolen): re-read */
			}
			if (v != r->old_ptr) {
				/* read-set invalid -> abort this txn */
				uatomic_cmpxchg(&t->status,
					URCU_MCAS_UNDECIDED,
					URCU_MCAS_FAILED);
				return;
			}
			/*
			 * Plain install under r's latch: install-once + the
			 * self-settle make a stale second install after @t
			 * linearizes (slot A-B-A'd back to r->old_ptr) inert.
			 */
			URCU_MCAS_PREINSTALL(t, r);
			planted = urcu_mcas_plant(t, r, r->old_ptr);
			if (planted)
				break;		/* installed (or already) */
			/* raced: slot changed under us -> re-evaluate */
		}
	}
	/* every record installed -> commit */
	uatomic_cmpxchg(&t->status, URCU_MCAS_UNDECIDED,
			URCU_MCAS_SUCCEEDED);
}

/*
 * Owner/reader entry point: drive @t to a terminal status, starting a fresh
 * help-recursion budget (depth 0).  Callers outside the engine use this; the
 * recursive helping path re-enters urcu_mcas_drive_install_depth() directly.
 */
static inline
void urcu_mcas_drive_install(struct urcu_mcas *t)
{
	urcu_mcas_drive_install_depth(t, 0);
}

/*
 * Settle phase (owner, in commit()): make @t's own slots plain -- its new value
 * on SUCCEEDED, its old on FAILED.  Called once @t is terminal.
 *
 * Per record, settle first CLAIMS the install word (FREE -> DONE), then converts
 * the slot.  The claim is what makes the reclaim contract unconditional: after
 * settle returns, every record's install word is DONE, so NO plant can begin --
 * in particular a stale FIRST install by a driver that read @t UNDECIDED, loaded
 * the slot, stalled, and resumed after @t was evicted and settled.  Such a plant
 * fails its FREE->BUSY CAS against DONE and returns "already installed" without
 * touching the slot.  Without the claim, that stale plant would transiently
 * REPUBLISH a proxy of the already-retired @t (its self-settle removes it again,
 * but not atomically with the plant), and a reader whose read-side section began
 * after the owner's call_rcu() -- one the grace period does not wait for --
 * could resolve that proxy and dereference @t after the GP ends: use-after-free.
 * The install-once gate alone cannot close this: it stops a stale SECOND install
 * of a planted record, but a never-planted record's word is still FREE.  The
 * claim turns "no proxy lingers" into "no proxy can appear at all", which is
 * what reclaim needs -- once settle returns, no slot names a record of @t and
 * none ever will again, so the descriptor can be reclaimed.
 *
 * A record caught BUSY (a planter between its FREE->BUSY and its DONE store) is
 * spun out -- the same bounded few-instruction window the install latch already
 * tolerates in urcu_mcas_plant().  That planter either fails its slot CAS and
 * releases FREE (settle then claims it), or plants and -- @t being terminal --
 * self-settles before storing DONE, so the slot is already plain when settle's
 * own slot CAS runs (it just fails, ignored).  A slot a higher-priority
 * transaction already stole holds that thief's proxy; that CAS fails and is
 * ignored too.  Claiming FREE->DONE cannot forge a bogus commit: the final
 * UNDECIDED->SUCCEEDED CAS in drive_install fails because @t is already
 * terminal whenever settle runs.  Settle is idempotent and owner-only; it
 * never waits while holding a latch (the claim IS the terminal state).
 */
static inline
void urcu_mcas_settle(struct urcu_mcas *t)
{
	unsigned long st = urcu_mcas_status(t);
	unsigned int i;

	for (i = 0; i < t->nr; i++) {
		struct urcu_mcas_record *r = &t->recs[i];
		void *want = (st == URCU_MCAS_SUCCEEDED) ? r->new_ptr : r->old_ptr;

#ifndef URCU_MCAS_NO_ABA_FIX
		/*
		 * Claim r's install word (FREE -> DONE) so no plant can begin
		 * once settle returns (see above), spinning out a planter's
		 * bounded BUSY window.
		 */
		for (;;) {
			int v = uatomic_cmpxchg(&r->state,
					URCU_MCAS_INSTALL_FREE,
					URCU_MCAS_INSTALL_DONE);

			if (v != URCU_MCAS_INSTALL_BUSY)
				break;		/* claimed FREE, or already DONE */
			while (uatomic_load(&r->state, CMM_RELAXED) ==
					URCU_MCAS_INSTALL_BUSY)
				caa_cpu_relax();
		}
#endif
		(void) uatomic_cmpxchg(r->slot, urcu_mcas_tag(r, r->proxy_tag), want);
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
void *urcu_mcas_read(void **slot, uintptr_t tag)
{
	for (;;) {
		void *v = uatomic_load(slot, CMM_ACQUIRE);
		struct urcu_mcas *e;

		if (caa_likely(!urcu_mcas_is_proxy(v, tag)))
			return v;
		e = urcu_mcas_untag(v, tag)->mcas;
		if (urcu_mcas_status(e) != URCU_MCAS_UNDECIDED)
			return urcu_mcas_resolve(v, tag);	/* terminal: logical value */
		urcu_mcas_drive_install(e);		/* help decide, then re-read */
	}
}

/*
 * Allocate a descriptor blob of @size bytes, 16-byte aligned.  The descriptor
 * holds the inline records that get tagged into slots, so its base must be
 * 16-byte aligned for every inline record to keep its low 4 bits free (see the
 * layout note above struct urcu_mcas).  posix_memalign guarantees that
 * portably; plain malloc would only promise max_align_t (8 on some 32-bit
 * ABIs).  Returns NULL on OOM.  The caller stamps the descriptor's identity
 * fields -- cap and the slab-origin flag -- as urcu_mcas_alloc_cap() does.
 */
static inline
struct urcu_mcas *urcu_mcas_alloc(size_t size)
{
	void *p;

	if (posix_memalign(&p, 16, size))
		return NULL;
	return (struct urcu_mcas *) p;
}

/*
 * ─────────────────────────────────────────────────────────────────────────
 * Per-CPU descriptor slab.
 *
 * The per-attempt descriptor (header + inline recs[]) is served from the shared
 * per-CPU size-classed slab in <urcu/rcu-txn-slab.h>.  Record-count classes
 * {4,8,16,32,64,128} map to byte sizes urcu_mcas_blocksize(cap); a request over
 * the top class is an exact, uncached posix_memalign.  Every descriptor is
 * STAMPED at allocation with its origin (urcu_mcas.slab) and free consults
 * that stamp, so a block is always freed on the path that allocated it --
 * including one allocated before the slab's constructor ran or while it was
 * disabled.  See rcu-txn-slab.h for the arena / superblock / wfstack mechanics
 * and the growth bound.  URCU_TXN_NO_CACHE
 * disables it (falls back to posix_memalign/free).
 * ─────────────────────────────────────────────────────────────────────────
 */
#define urcu_mcas_blocksize(cap)	\
	(sizeof(struct urcu_mcas) + (size_t) (cap) * sizeof(struct urcu_mcas_record))

static const unsigned int urcu_mcas_slab_rc[] = { 4u, 8u, 16u, 32u, 64u, 128u };
#define URCU_MCAS_SLAB_NCLASS	\
	((int) (sizeof(urcu_mcas_slab_rc) / sizeof(urcu_mcas_slab_rc[0])))

/*
 * The slab INSTANCE lives once, in liburcu-common (src/urcu-txn.c), which also
 * initializes it from a library constructor -- so this header requires linking
 * liburcu-common.  A header-static definition here would hand every including
 * TU its own arenas and superblocks: cross-TU frees would still be safe (the
 * origin arena is found from the superblock header), but the freelists would
 * never share and the footprint would multiply per allocating TU.  The single
 * library constructor also runs at liburcu-common init time -- before the
 * initializers of anything that depends on the shared library -- narrowing the
 * window where a constructor-context transaction could precede slab init.
 */
extern struct urcu_slab urcu_mcas_slab;

/* Smallest record-count class that fits @req records, or -1 if over the top. */
static inline
int urcu_mcas_slab_class_of(unsigned int req)
{
	int i;

	for (i = 0; i < URCU_MCAS_SLAB_NCLASS; i++)
		if (req <= urcu_mcas_slab_rc[i])
			return i;
	return -1;
}

/*
 * Allocate a descriptor with room for >= @req records, setting its PHYSICAL
 * cap and stamping its origin (t->slab, the free discriminator).  A request
 * that fits a size class comes from the per-CPU slab (physical cap == the
 * class size); a larger one -- or any request while the slab is uninitialized
 * or disabled -- is an exact, uncached posix_memalign.
 */
static inline
struct urcu_mcas *urcu_mcas_alloc_cap(unsigned int req)
{
	struct urcu_mcas *t;
	int cl;

	if (urcu_slab_enabled(&urcu_mcas_slab) && (cl = urcu_mcas_slab_class_of(req)) >= 0) {
		t = (struct urcu_mcas *) urcu_slab_alloc(&urcu_mcas_slab, cl);
		if (caa_likely(t != NULL)) {
			t->cap = urcu_mcas_slab_rc[cl];
			t->slab = 1;
		}
	} else {
		t = urcu_mcas_alloc(urcu_mcas_blocksize(req));
		if (caa_likely(t != NULL)) {
			t->cap = req;
			t->slab = 0;
		}
	}
	return t;
}

/*
 * Free a descriptor on the path that allocated it (the t->slab stamp): a slab
 * block returns to its ORIGIN
 * arena regardless of which thread frees it (origin found from the superblock
 * header), so writer-context and reclaim-worker frees are identical -- no
 * local/remote split.  An exact (uncached) block goes back to malloc.  The
 * stamp -- not the slab's current enabled state -- decides, so a block
 * allocated before the slab constructor ran is never misrouted to
 * urcu_slab_free() after the slab enables.
 */
static inline
void urcu_mcas_free_local(struct urcu_mcas *t)
{
	if (t->slab)
		urcu_slab_free(t);
	else
		free(t);
}

static inline
void urcu_mcas_free_remote(struct urcu_mcas *t)
{
	urcu_mcas_free_local(t);
}

/*
 * Create a transaction with room for @cap records.  @retry is the caller's
 * aging-priority: the number of times this logical operation has already retried
 * (0 on the first attempt, incremented and passed back in on each retry).  A
 * higher value wins contended slots, so a starved operation eventually cannot be
 * bypassed -- see urcu_mcas_outranks().
 */
static inline
struct urcu_mcas *urcu_mcas_create(unsigned int cap,
		unsigned long retry)
{
	struct urcu_mcas *t;

	t = urcu_mcas_alloc_cap(cap);
	if (!t)
		return NULL;
	t->status = URCU_MCAS_UNDECIDED;
	t->retry = retry;
	t->nr = 0;
	/* t->cap set to the physical capacity by urcu_mcas_alloc_cap() */
	t->poisoned = 0;
	return t;
}

/*
 * Append one edge {*slot: old -> new}.  Before commit only; no install yet.
 * Returns false if the descriptor is full -- the caller grows it first
 * (urcu_mcas_grow) and retries.  The record's back-pointer (r->mcas) is
 * deliberately NOT set here: a pre-commit grow reallocs the descriptor and may
 * move it, which would strand any add-time back-pointer.  It is filled in once,
 * at commit, after the write-set has stopped growing (see mcas_commit).
 */
static inline
bool urcu_mcas_add(struct urcu_mcas *t, void **slot,
		void *old_ptr, void *new_ptr, uintptr_t tag)
{
	struct urcu_mcas_record *r;

	if (t->nr == t->cap)
		return false;
	r = &t->recs[t->nr++];
	r->slot = slot;
	r->old_ptr = old_ptr;
	r->new_ptr = new_ptr;
	r->proxy_tag = tag;	/* the slot's tag; travels with the record */
	return true;
}

/*
 * Record edge {*slot: old -> new} keeping at most one record per @slot, so the
 * engine's distinct-slot precondition holds by construction.  If a record
 * already targets @slot (a prior store, or a load-validate guard), reconcile it
 * rather than append a duplicate: @old_ptr must equal the record's old_ptr
 * (else the caller read @slot twice and saw it move -- an inconsistent,
 * torn-read txn).  A disagreement is NOT silently merged: it POISONS the
 * descriptor so commit() aborts the attempt (the caller re-reads consistently
 * and retries) -- merging would otherwise forge a record whose old no longer
 * matches the intended write and commit a corrupt edge.  Note the disagreement
 * is a LEGAL race, not an embedder bug: a peer may commit between two reads of
 * the same slot in one attempt, so poison-and-retry is the only correct
 * response (no assert).  @upgrade picks new_ptr -- a store advances it to
 * @new_ptr, a load-validate leaves a pending write intact.  Returns false only
 * when a new record is needed and the descriptor is full; the caller grows and
 * retries.
 */
static inline
bool urcu_mcas_record(struct urcu_mcas *t, void **slot,
		void *old_ptr, void *new_ptr, int upgrade, uintptr_t tag)
{
	unsigned int i;

	for (i = 0; i < t->nr; i++) {
		if (t->recs[i].slot != slot)
			continue;
		if (caa_unlikely(t->recs[i].old_ptr != old_ptr)) {
			/*
			 * Legal race (a peer committed between two reads of
			 * this slot), so no assert: aborting the process on a
			 * nondeterministic interleaving would make a
			 * survivable conflict fatal.
			 */
			t->poisoned = 1;	/* torn read-set: commit will abort */
			return true;
		}
		/* Same slot -> same (agreed) tag; reconcile keeps the record's. */
		if (upgrade)
			t->recs[i].new_ptr = new_ptr;
		return true;
	}
	return urcu_mcas_add(t, slot, old_ptr, new_ptr, tag);
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
struct urcu_mcas *urcu_mcas_grow(struct urcu_mcas *t)
{
	unsigned int newcap = t->cap < 2 ? 2 : t->cap * 2;
	unsigned int ncap, nslab;
	struct urcu_mcas *n;

	n = urcu_mcas_alloc_cap(newcap);
	if (!n)
		return NULL;
	ncap = n->cap;				/* the new block's identity, set by alloc_cap ... */
	nslab = n->slab;
	memcpy(n, t, sizeof(*t) +
			(size_t) t->nr * sizeof(struct urcu_mcas_record));
	n->cap = ncap;				/* ... which the header memcpy clobbered */
	n->slab = nslab;
	urcu_mcas_free_local(t);		/* pre-commit: writer-context free */
	return n;
}

static inline
void urcu_mcas_destroy(struct urcu_mcas *t)
{
	urcu_mcas_free_local(t);
}

/* call_rcu callback: deferred destroy. */
static inline
void urcu_mcas_free_rcu(struct rcu_head *head)
{
	urcu_mcas_free_remote(caa_container_of(head,
			struct urcu_mcas, rcu_head));
}

/* Sort records by slot address (insertion sort -- transactions are small). */
static inline
void urcu_mcas_sort(struct urcu_mcas *t)
{
	unsigned int i, j;

	for (i = 1; i < t->nr; i++) {
		struct urcu_mcas_record key = t->recs[i];

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
 * period).  A single-edge op that has retried past URCU_MCAS_ESCALATE falls
 * through to the full descriptor path so it installs a real proxy and contends
 * on the same priority-ordered footing as everyone else, instead of being shut
 * out of a slot a multi-edge op keeps proxied.  (This lifts a lockout; like any
 * transaction here it remains bounded-blocking, not wait-free.)
 *
 * Call within an RCU read-side section (descriptor existence for helpers).
 */
static inline
bool urcu_mcas_commit(struct urcu_mcas *t,
		void (*call_rcu_fn)(struct rcu_head *,
			void (*)(struct rcu_head *)))
{
	bool committed;
	unsigned int i;

	if (caa_unlikely(t->poisoned)) {
		/*
		 * A same-slot reconcile disagreed on the expected old (a torn
		 * read-set): the write-set is inconsistent.  Never parked, so free
		 * synchronously and abort -- the caller re-reads and retries.  Must
		 * precede the nr==1 fast path: a poisoned descriptor can still have a
		 * single record (two stores to one slot reconcile to one).
		 */
		urcu_mcas_destroy(t);
		return false;
	}
	if (t->nr == 0) {
		urcu_mcas_destroy(t);
		return true;
	}
	if (t->nr == 1 && t->retry < URCU_MCAS_ESCALATE) {
		/*
		 * Single edge, not yet starved: the CAS itself is the atomic
		 * commit.  Fast and -- while the slot is plain -- fair; once it
		 * has retried enough to suspect a proxy is locking it out, the
		 * escalation below makes it a real, visible transaction instead.
		 */
		struct urcu_mcas_record *r = &t->recs[0];

		committed = uatomic_cmpxchg(r->slot, r->old_ptr, r->new_ptr) == r->old_ptr;
		urcu_mcas_destroy(t);
		return committed;
	}
	if (t->nr == 1)
		URCU_MCAS_STAT(escalate);	/* single edge, retried past the threshold */
	urcu_mcas_sort(t);
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
		urcu_mcas_latch_init(&t->recs[i]);
	}
	urcu_mcas_drive_install(t);		/* install to a decision (helpers help) */
	urcu_mcas_settle(t);			/* owner-only: make our own slots plain */
	committed = urcu_mcas_status(t) == URCU_MCAS_SUCCEEDED;
	call_rcu_fn(&t->rcu_head, urcu_mcas_free_rcu);
	return committed;
}

#ifdef __cplusplus
}
#endif

#endif	/* _URCU_RCU_MCAS_H */
