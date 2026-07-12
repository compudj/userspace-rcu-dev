// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

#ifndef _URCU_RCU_MCAS_H
#define _URCU_RCU_MCAS_H

/*
 * RCU MCAS: a multi-word compare-and-swap (k-CAS) engine.
 *
 * This is the concurrent-writer sibling of <urcu/rcu-txn-sw.h>.  It switches a
 * *set* of words from their old values to new ones atomically, as observed by
 * both concurrent RCU readers AND concurrent writers.  Progress is
 * bounded-blocking and single-driver: a transaction is driven only by its
 * owner, and a thread that trips over an in-flight transaction's proxy waits a
 * bounded spin for that proxy's owner to settle the slot, then escalates
 * (aborts and retries at a rising aging priority, ultimately through the
 * front-end's fair-mutex fallback) -- no helping, no stealing, no deadlock.
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
 * value alone is never load-bearing (one txn's new is the next txn's old --
 * only the descriptor identity disambiguates).  With a single driver per
 * transaction this makes the engine indifferent to slot-value A-B-A: no
 * straggler can drag a slot backward by re-planting a descriptor after it
 * linearized, because the only thread that plants a transaction's records is
 * its owner (see "Slot-value A-B-A" below).
 *
 * Resolution.  A reader (or a writer traversing) that loads a slot holding
 * flip(r) reads r->mcas->status: SUCCEEDED resolves to r->new_ptr, UNDECIDED or
 * FAILED to r->old_ptr.  Because resolution goes through the status word,
 * planting a terminal descriptor in a slot is *correct*, not a bug -- which is
 * why no RDCSS is needed.
 *
 * Settle is owner-only.  A transaction is installed and settled by its owner
 * alone, in commit(), sequential with its own settle(): no thread ever drives,
 * evicts, or steals a foreign transaction's records.  The owner's settle
 * rewrites its parked records back to plain values (its new on SUCCEEDED, its
 * old on FAILED), converting exactly the prefix the install planted; since
 * nothing foreign ever plants into those records, once settle returns no proxy
 * of the transaction names any slot -- which is what makes the post-settle
 * reclaim safe.
 *
 * A contended slot never reverts to plain while its transaction is still
 * UNDECIDED -- only a terminal transaction's slots decay to plain, by the
 * owner's settle.  A committer that needs a slot another transaction holds
 * UNDECIDED spins a bounded wait for that owner to settle it plain, then
 * acquires it -- the proxy is a pure spinlatch.  The cost is that readers may
 * resolve a lingering proxy rather than load a settled value; an aborted
 * transaction settles right after, so the window is short.
 *
 * Liveness.  Records install in one global order (sorted by slot address at age
 * 1+; age 0 installs flat and bails at the first foreign proxy), so a committer
 * holds only lower slots while waiting on a shared one and cannot deadlock: the
 * owner it waits on, having reached that slot, already passed every lower one.
 * The wait is bounded; on cap-out the committer aborts and retries at a rising
 * aging priority, and the transaction front-end's fair-mutex fallback forces
 * progress even against a preempted owner.  The engine is bounded-BLOCKING, not
 * lock-free.
 *
 * Existence.  Mutators run as RCU readers (rcu_read_lock around the whole
 * operation).  A descriptor reachable through a slot stays alive until every
 * reader that reached it leaves its read-side section; the owner call_rcu()s
 * the descriptor once it is terminal and fully settled.  No refcount.
 *
 * Constraints (vs the single-writer <urcu/rcu-txn-sw.h>):
 *   - the engine owns tag bit 0, so every value stored in a transacted slot
 *     must be at least 2-byte aligned (bit 0 clear) -- true for any pointer,
 *     or store small integers shifted left by 1;
 *   - the record set is frozen at commit: no record may be added after the
 *     first install;
 *   - a transaction's records must target pairwise-distinct slots.
 *
 * Slot-value A-B-A -- SAFE.  The engine omits RDCSS: it installs a descriptor
 * with a plain CAS conditioned on the word holding its expected old.  With a
 * single driver per transaction that suffices.  The only way a plain install
 * CAS could resurrect a stale value is a delayed SECOND install of an
 * already-decided descriptor passing its CAS on a value that recurred to the
 * old -- and that needs a second, stalled driver of the same transaction.  No
 * such driver exists here: the owner plants its whole prefix before it decides,
 * and no foreign thread ever plants, drives, or steals its records.  So ANY
 * slot-value A-B-A is inert, whatever recurs the value -- a doubly-linked
 * next-pointer cycling B -> X -> B (RCU prevents ADDRESS reuse but not VALUE
 * recurrence, with B a live successor it never frees), a counter revisiting a
 * number, an embedder's own reuse.  What the engine DOES require is unrelated
 * to slot values: tag bit 0 free, pairwise- distinct slots per txn, and -- the
 * EXISTENCE model -- that a descriptor reachable through a slot is reclaimed
 * only after a grace period (a reader may dereference it).  Note this is
 * value-CAS atomicity: a record is validated to hold its old at the
 * LINEARIZATION point, not to have been stable throughout; an embedder needing
 * the latter (snapshot/version semantics) layers its own versioning on top, as
 * with any value-based MCAS -- but that is a stronger guarantee than memory
 * safety, which holds unconditionally here.
 *
 * Every install is a bare value-CAS (slot: old -> tagged proxy).  Under one
 * driver per transaction that is sufficient: the classic re-plant A-B-A -- a
 * stale second plant resurrecting a retired descriptor after the slot value
 * cycled back -- needs a foreign driver of our records, which the sole-driver
 * regime excludes, so no per-record install latch is needed.
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
 * The single-driver spinlatch above is the only install now.  The historical
 * helping/stealing install and its URCU_MCAS_STOCK A/B switch (which toggled
 * the former NO_HELP / NO_STEAL / AGE0_TRYLATCH study flags) have been retired
 * -- the sole-driver regime won across every measured workload.  The txn
 * layer's matching defaults (age escalation, the k=3/1024-bit Bloom) are
 * likewise unconditional now -- see <urcu/rcu-txn.h>.
 */

/*
 * Optional instrumentation hook.  Compiles to nothing unless the embedder
 * defines URCU_MCAS_STAT(counter) before including this header (the unit tests
 * use it to count install-driver and escalation work).  Not part of the
 * engine contract.
 */
#ifndef URCU_MCAS_STAT
#define URCU_MCAS_STAT(counter)	do { } while (0)
#endif

/*
 * Single-edge escalation threshold.  A one-record transaction normally commits
 * with a bare CAS and no descriptor (cheap, but it installs no proxy, so it can
 * only win when the slot is already plain -- it cannot break into a slot a
 * multi-edge transaction keeps proxied under its spinlatch).
 * Once such an op has retried this many times it stops taking the fast path and
 * commits through the full descriptor protocol instead, so it installs a real
 * proxy and can hold the slot latched against contenders (at the cost of a
 * descriptor and one grace period).  This lifts a *lockout*; it does not make
 * the op wait-free -- the engine is bounded-blocking, not wait-free.  The
 * retry count the caller threads into urcu_mcas_create() drives this.
 */
#ifndef URCU_MCAS_ESCALATE
#define URCU_MCAS_ESCALATE 16
#endif

/*
 * Single-driver spinlatch: a transaction is driven only by its owner.  A
 * committer that meets a FOREIGN proxy in one of its own record slots -- and a
 * reader (urcu_mcas_read) that lands on an undecided proxy -- never drives,
 * evicts, or steals it: the committer waits a bounded spin for the proxy's
 * owner to settle the slot plain, then acquires it (the proxy is a pure
 * spinlatch), and ESCALATES if the wait caps out -- aborts the transaction it
 * is committing so it retries with a higher aging priority and, via the
 * transaction front-end's fair-mutex fallback, eventually makes progress even
 * against a preempted owner.  So there is exactly ONE driver per transaction:
 * its owner, in commit(), sequential with its own settle().  The engine is
 * therefore bounded-BLOCKING, not lock-free (a preempted owner blocks waiters
 * until aging/fallback resolves it), which the measurements show wins anyway.
 */
#ifndef URCU_MCAS_WAIT_PATIENCE
#define URCU_MCAS_WAIT_PATIENCE 8192	/* spins on a blocker's status before escalating */
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
	uintptr_t proxy_tag;		/*
					 * Embedder's tag bits for THIS record's
					 * slot.  The parked proxy value is
					 * (&record | proxy_tag), the record is
					 * recovered as (value & ~proxy_tag),
					 * and a value is this record's proxy
					 * iff (value & proxy_tag) == proxy_tag.
					 * Carried per record -- not a per-TU
					 * macro -- so heterogeneous slots (e.g.
					 * fractal-trie 0xF vs a list's bit 0)
					 * share one engine, each resolved
					 * through its own tag.
					 */
	struct urcu_mcas *mcas;	/* back-pointer (status + sibling records) */
} __attribute__((aligned(16)));

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
	unsigned int slab;		/*
					 * Block origin: per-CPU slab (1) or
					 * exact posix_memalign (0).  Stamped at
					 * alloc and consulted by free --
					 * self-describing, so a descriptor
					 * allocated while the slab was still
					 * uninitialized (constructor ordering)
					 * or disabled is freed on the right
					 * path even if the slab enables in
					 * between.  Occupies what was padding:
					 * recs[] stays 16-byte aligned
					 * (asserted below).
					 */
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
 * forming the proxy from the record's CURRENT address at install is what makes
 * a descriptor grow/realloc safe: the tag travels with the record content
 * through the move, and the installer re-derives the proxy from the record's
 * final address, so nothing is stranded.  Passing the tag explicitly lets
 * heterogeneous slots share one engine -- the fractal trie tags with its type-7
 * / 0xF low nibble (a value no real node, NULL or skip pointer carries), a
 * doubly-linked list with bit 0, etc.  The contract a tag must satisfy:
 * (live_value & tag) != tag for EVERY non-proxy value the embedder stores in a
 * transacted slot, so the engine never mistakes a live value for one of its
 * records.
 *
 * urcu_mcas_tag()/untag() take @tag explicitly; the installer passes the
 * record's own r->proxy_tag, and a reader passes the (agreed) tag of the slot
 * it is resolving.
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
 * @v is a parked record iff it carries all of @tag's bits -- equivalently v ==
 * (urcu_mcas_untag(v, tag) | tag), i.e. @v is some record address OR'd with
 * @tag.
 */
static inline
int urcu_mcas_is_proxy(void *v, uintptr_t tag)
{
	return ((uintptr_t) v & tag) == tag;
}

/*
 * A true try-CAS: swap @*slot from @expect to @desired, returning 1 on success
 * (the slot held @expect) and 0 on failure, WITHOUT the caller re-comparing an
 * old value.  uatomic_cmpxchg returns the prior value, so extracting a success
 * bit costs a compare-and-branch per call; the compiler's
 * __atomic_compare_exchange_n reports success directly (x86: the ZF of a single
 * `lock cmpxchg`), which lets the flat install accumulate outcomes with a
 * bitwise OR instead of a per-result branch.  This is the primitive uatomic
 * lacks; a future uatomic_try_cmpxchg_mo would replace this shim verbatim.
 * Strong (no spurious failure); ACQ_REL on success, ACQUIRE on failure -- on
 * x86 both lower to the same barrier-free `lock cmpxchg`, so the memory-order
 * choice is free here.
 */
static inline
int urcu_mcas_try_cas(void **slot, void *expect, void *desired)
{
	return __atomic_compare_exchange_n(slot, &expect, desired,
			/*weak=*/0, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE);
}

static inline
unsigned long urcu_mcas_status(struct urcu_mcas *t)
{
	return uatomic_load(&t->status, CMM_ACQUIRE);
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
 * Plant record @r: CAS @r's slot from r->old_ptr to the record's tagged proxy.
 * Returns 1 if we planted it, 0 if the CAS raced the slot away (the caller
 * re-reads and retries).
 *
 * A bare value-CAS is correct because we are the SOLE driver of @r's
 * transaction (single-driver engine): only the owner ever plants @r, it plants
 * exactly once while the transaction is UNDECIDED (the drive loop re-checks the
 * status before each plant and decides only after the record loop), and no
 * foreign thread re-plants or settles @r.  So the classic re-plant A-B-A -- the
 * slot value cycling back to r->old_ptr and a STALE second plant resurrecting a
 * retired descriptor -- cannot arise: there is no second plant.  That is what
 * retired the per-record install latch (FREE/BUSY/DONE) and the installer
 * self-settle; a concurrent transaction that changed the slot since the
 * caller's load simply fails this CAS, which the caller handles by re-reading.
 */
static inline
int urcu_mcas_plant(struct urcu_mcas_record *r)
{
	void *tagv = urcu_mcas_tag(r, r->proxy_tag);

	return uatomic_cmpxchg(r->slot, r->old_ptr, tagv) == r->old_ptr;
}

/*
 * Install phase: drive transaction @t to a terminal status (SUCCEEDED or
 * FAILED) by installing its records in slot-address order.  Each install is a
 * bare value-CAS (urcu_mcas_plant) -- the owner is the sole driver -- so a
 * foreign proxy met in a slot is a spinlatch it waits out (bounded) or
 * escalates past, never one it drives.  The owner's own settle
 * (urcu_mcas_settle) converts its planted proxies to plain values; until then a
 * terminal proxy is resolved by readers and contenders through the status word
 * rather than waited on.
 */
/*
 * Drive @t to a terminal status (SUCCEEDED or FAILED): @t's OWN decision, taken
 * by its owner.  With no helper and no evictor the owner is the SOLE writer of
 * its status and writes it at most once per drive, so this is a plain RELEASE
 * store -- the commit point, like existence's group-word flip.  RELEASE so the
 * planted proxies and committed values publish before a reader can observe the
 * terminal status.
 */
static inline
void urcu_mcas_decide(struct urcu_mcas *t, unsigned long to)
{
	/*
	 * Sole-writer invariant: @t is written only by its owner, at most once
	 * per drive (every FAILED site returns immediately; SUCCEEDED is only
	 * reached once, after the record loop).  So the status MUST be
	 * UNDECIDED here -- a plain store would otherwise clobber an
	 * already-published decision.  Assert it (debug-only) so any future
	 * path that breaks single-writer -- or reaches a decision twice --
	 * trips instead of silently overwriting.
	 */
	urcu_assert_debug(urcu_mcas_status(t) == URCU_MCAS_UNDECIDED);
	uatomic_store(&t->status, to, CMM_RELEASE);
}

/*
 * Age-0 sole-driver install, FLAT over the records: no per-record slot load or
 * branch, no foreign-proxy wait, no slot-address sort.  Precondition: retry ==
 * 0 (the caller dispatches).
 *
 * Plant each record with ONE strong try-CAS (slot: old_ptr -> our proxy), OR
 * the outcome into @fail, and bail at the FIRST conflict.  The goal is not "no
 * branches" but "no MISPREDICTED branches": the age-1+ install (drive_install_
 * depth) loads each slot and tests it (is-this-a-foreign-proxy?, does-the-slot-
 * still-hold-old?) before its CAS, and those tests are data-dependent on a
 * freshly loaded slot value, so they mispredict exactly WHEN a slot is
 * contended -- the costly case.  Here the try-CAS is itself the contention
 * check (it fails iff the slot is not old_ptr -- a foreign proxy, or a value a
 * concurrent transaction changed), its outcome is OR-accumulated rather than
 * branched on, and the only branch left -- the early break -- is a register
 * test on @fail AFTER the CAS (so it never gates the CAS) that is not-taken on
 * every iteration of a committing txn (so it does not mispredict).  We also
 * drop the pre-CAS load, which would pull the slot line Shared only for the CAS
 * to upgrade it to Exclusive.
 *
 * Correct and deadlock-free precisely because we are the SOLE driver:
 *
 *   - The plant is a bare value-CAS for the same reason the age-1+ path's is
 *     (see urcu_mcas_plant): one driver, @t stays UNDECIDED until the single
 *     store below, and no foreign thread re-plants our records -- so a
 *     slot-value A-B-A is inert and no install latch is needed.
 *   - We never WAIT on a foreign proxy (the CAS fails and we bail), so there is
 *     no hold-and-wait and the records need no slot-address sort -- the sort
 *     and the bounded wait are exactly what the age-1+ path adds on top of
 *     this.
 *
 * Bailing at the first conflict (rather than planting the whole write-set past
 * it) keeps the plant a contiguous PREFIX [0..i): it parks fewer transient
 * proxies -- planting past a conflict amplifies aborts, since each parked proxy
 * fail-fasts other age-0 txns -- and it makes @i the exact planted count for a
 * load-free settle (SUCCEEDED: i == nr; FAILED: i == the failing index), at no
 * fast-path cost since @i is the loop induction variable already in a register.
 *
 * Commit point (arithmetic, no branch): SUCCEEDED == 1 and FAILED == 2, and
 * @fail is 0 iff every CAS won, so the terminal status is SUCCEEDED + (fail !=
 * 0).  RELEASE so the planted proxies publish before a reader can observe the
 * status.
 */
static inline
unsigned int urcu_mcas_drive_install_age0_flat(struct urcu_mcas *t)
{
	unsigned long fail = 0;
	unsigned int i;

	URCU_MCAS_STAT(drive);
	for (i = 0; i < t->nr; i++) {
		struct urcu_mcas_record *r = &t->recs[i];

		fail |= (unsigned long) !urcu_mcas_try_cas(r->slot, r->old_ptr,
				urcu_mcas_tag(r, r->proxy_tag));
		if (caa_unlikely(fail))
			break;	/* first conflict: prefix [0..i) planted */
	}
	urcu_assert_debug(urcu_mcas_status(t) == URCU_MCAS_UNDECIDED);
	uatomic_store(&t->status, URCU_MCAS_SUCCEEDED + (fail != 0),
			CMM_RELEASE);
	return i;	/* planted count: nr on success, fail index on abort */
}

static inline
void urcu_mcas_drive_install_depth(struct urcu_mcas *t, unsigned int *plantedp)
{
	unsigned long st = urcu_mcas_status(t);
	unsigned int i;

	URCU_MCAS_STAT(drive);
	/*
	 * Sole driver: record how far we plant so the owner's settle converts
	 * exactly [0..*plantedp) with a plain store and never touches the
	 * un-planted tail (a FAILED drive stops at a foreign proxy).  *plantedp
	 * is always valid -- the wrapper passes a local (readers ignore its
	 * result); a commit passes settle the same value, and since
	 * drive+settle run on one thread with nothing driving @t in between, no
	 * persistent field is needed.
	 */
	*plantedp = 0;

	if (st != URCU_MCAS_UNDECIDED)
		return;			/* already terminal */
	for (i = 0; i < t->nr; i++) {
		struct urcu_mcas_record *r = &t->recs[i];
		void *tagv = urcu_mcas_tag(r, r->proxy_tag);

		for (;;) {
			void *v;

			st = urcu_mcas_status(t);
			if (st != URCU_MCAS_UNDECIDED)
				return;		/* defensive: owner is the sole decider */
			v = uatomic_load(r->slot, CMM_ACQUIRE);
			if (v == tagv)
				break;		/* already installed */
			if (urcu_mcas_is_proxy(v, r->proxy_tag)) {
				/*
				 * A foreign proxy in r->slot uses the same
				 * slot's (agreed) tag, so untag it with
				 * r->proxy_tag.
				 */
				struct urcu_mcas_record *fr =
					urcu_mcas_untag(v, r->proxy_tag);
				struct urcu_mcas *e = fr->mcas;

				if (e == t)
					break;	/* own proxy (distinct-slot inv.) */
				if (t->retry == 0) {
					/*
					 * Age-0 try-latch: a foreign proxy is
					 * contention.  Never wait -- fail fast
					 * and let the caller escalate to age 1
					 * (sorted, blocking install).  Because
					 * age 0 never waits on a foreign proxy
					 * there is no hold-and-wait, so its
					 * records need no slot-address sort to
					 * stay deadlock-free.
					 */
					urcu_mcas_decide(t, URCU_MCAS_FAILED);
					return;
				}
				/*
				 * Never displace E's proxy.  Wait for E's owner
				 * to SETTLE this slot to a plain value, then
				 * re-read and acquire it plainly -- the proxy
				 * is a pure spinlatch.  Records are installed
				 * in slot-address order (commit sorts), so
				 * holding the lower slots while waiting on this
				 * one cannot deadlock: E, having reached this
				 * slot, already passed every lower one without
				 * blocking, so it holds none we hold.  Bounded,
				 * then escalate (abort T -> retry -> fair-mutex
				 * fallback), since with no eviction a preempted
				 * owner is not rescued by aging.
				 */
				{
					unsigned int patience =
						URCU_MCAS_WAIT_PATIENCE;

					while (urcu_mcas_is_proxy(
						uatomic_load(r->slot, CMM_ACQUIRE),
						r->proxy_tag)) {
						if (urcu_mcas_status(t) !=
								URCU_MCAS_UNDECIDED)
							return;
						if (patience-- == 0) {
							URCU_MCAS_STAT(wait_capped);
							urcu_mcas_decide(t,
								URCU_MCAS_FAILED);
							return;
						}
						caa_cpu_relax();
					}
					continue;	/* slot plain now: re-read + acquire */
				}
			}
			if (v != r->old_ptr) {
				/* read-set invalid -> abort this txn */
				urcu_mcas_decide(t, URCU_MCAS_FAILED);
				return;
			}
			/*
			 * Slot is plain and holds our expected old.  Plant with
			 * a bare CAS: sole driver, @t still UNDECIDED
			 * (re-checked at the top of this loop), no foreign
			 * re-plant -- so no latch is needed (see
			 * urcu_mcas_plant).  A concurrent transaction that
			 * changed the slot since our load fails the CAS; we
			 * re-read and re-evaluate.
			 */
			if (urcu_mcas_plant(r))
				break;		/* planted */
			/* raced: slot changed under us -> re-evaluate */
		}
		/*
		 * Record i now holds our proxy.  A FAILED return leaves this at
		 * the last-planted count = the failing index (records [0..i-1]
		 * done); on SUCCESS it reaches t->nr, so settle can loop
		 * [0..*plantedp) either way.
		 */
		*plantedp = i + 1;
	}
	/* every record installed -> commit */
	urcu_mcas_decide(t, URCU_MCAS_SUCCEEDED);
}

/*
 * Owner/reader entry point: drive @t to a terminal status.  Returns the number
 * of records this drive planted, for the owner's settle (see
 * urcu_mcas_settle()); readers ignore it.
 */
static inline
unsigned int urcu_mcas_drive_install(struct urcu_mcas *t)
{
	unsigned int planted = 0;

	urcu_mcas_drive_install_depth(t, &planted);
	return planted;
}

/*
 * Settle phase (owner, in commit()): make @t's own slots plain -- its new value
 * on SUCCEEDED, its old on FAILED.  Called once @t is terminal.
 *
 * As the sole driver of @t we planted exactly [0..@planted) (drive_install
 * returned the count) and nothing helps, steals, or self-settles our records,
 * so settle just converts that prefix with plain RELEASE stores.  The reclaim
 * contract -- no proxy of @t names a slot once settle returns -- holds because
 * we planted only that prefix and convert all of it; the un-planted tail is a
 * foreign proxy (or a slot we never reached) and is left untouched.  Settle is
 * owner-only and never waits.
 */
static inline
void urcu_mcas_settle(struct urcu_mcas *t, unsigned int planted)
{
	unsigned long st = urcu_mcas_status(t);
	unsigned int i;

	/*
	 * Sole driver: the drive told us it planted exactly [0..@planted) --
	 * @planted == t->nr on SUCCESS, the failing index on a partial FAILED
	 * drive.  Every one of those slots still holds OUR proxy: nothing helps
	 * or steals, and no plant self-settles while @t stays UNDECIDED through
	 * the whole drive.  So convert each with a plain RELEASE store -- no
	 * load-test, no CAS.  The un-planted tail [@planted..nr) (a foreign
	 * proxy where the drive stopped, or a slot we never reached) is simply
	 * not visited, so nothing there is clobbered, and no proxy of @t
	 * lingers past settle -- the reclaim contract holds because we planted
	 * none.
	 */
	for (i = 0; i < planted; i++) {
		struct urcu_mcas_record *r = &t->recs[i];
		void *want = (st == URCU_MCAS_SUCCEEDED) ? r->new_ptr : r->old_ptr;

		uatomic_store(r->slot, want, CMM_RELEASE);
	}
}

/*
 * Load @slot and return the value it currently denotes: a plain value as-is,
 * or, for a parked record, the value resolved through its transaction's status.
 * If that transaction is still UNDECIDED, wait a bounded spin for its owner to
 * decide it (so the returned value is stable), but never drive or settle it --
 * the proxy is owner-only, and the value returned is the *logical* one.  Use
 * this to read the current value of a word you intend to transact (its old);
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
		{
			/*
			 * Never drive E.  Wait a bounded spin for E's owner to
			 * decide it, then loop to re-read (the freshest value);
			 * if it stays undecided past the cap, return its
			 * logical value (E's old) -- for a read-set caller that
			 * is an optimistic old, reconciled at commit exactly
			 * like urcu_mcas_read_optimistic().
			 */
			unsigned int patience = URCU_MCAS_WAIT_PATIENCE;

			while (urcu_mcas_status(e) == URCU_MCAS_UNDECIDED) {
				if (patience-- == 0)
					return urcu_mcas_resolve(v, tag);
				caa_cpu_relax();
			}
			continue;	/* E decided: re-read for the freshest value */
		}
	}
}

/*
 * Load @slot and return the value it currently denotes, WITHOUT waiting on an
 * undecided transaction: the non-blocking counterpart of urcu_mcas_read().
 *
 * A slot parked with an UNDECIDED transaction's proxy still logically holds
 * that record's old_ptr -- exactly what urcu_mcas_resolve() returns.  This
 * reader takes that logical old and moves on immediately, where
 * urcu_mcas_read() would first spin a bounded number of times for the owner to
 * settle the slot plain.
 *
 * What this gives up against urcu_mcas_read() is only STABILITY of the returned
 * value, never safety:
 *
 *   - It is not weaker for observers.  A plain reader already resolves each
 *     slot against E's status at the moment it looks, so a walk spanning two of
 *     E's slots across E's commit could always straddle it.  The bounded wait
 *     in urcu_mcas_read() does not fix that either: a *later* slot may be
 *     parked by a different transaction that commits in between.
 *
 *   - It is safe for a transaction's read set.  The logical old returned here
 *     is reconciled at install: commit() re-reads the slot, resolves whatever
 *     proxy it finds, and compares against the recorded old_ptr -- a value that
 *     has since changed aborts, one that has not is taken.  A stale optimistic
 *     read is therefore an extra abort, never a wrong commit.
 *
 *   - Progress is unaffected.  A descheduled owner's proxy is resolved to its
 *     logical value on sight rather than blocking the reader at all; only the
 *     stability spin is skipped.
 *
 * Prefer this for traversal -- reads whose value feeds a search and will be
 * validated at commit.  Prefer urcu_mcas_read() when the caller needs the value
 * to be stable at the point of the read itself.  Call within an RCU read-side
 * section.
 */
static inline
void *urcu_mcas_read_optimistic(void **slot, uintptr_t tag)
{
	void *v = uatomic_load(slot, CMM_ACQUIRE);

	if (caa_likely(!urcu_mcas_is_proxy(v, tag)))
		return v;
	return urcu_mcas_resolve(v, tag);	/* undecided: E's old, its logical value */
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
void urcu_mcas_free(struct urcu_mcas *t)
{
	if (t->slab)
		urcu_slab_free(t);
	else
		free(t);
}

/*
 * Create a transaction with room for @cap records.  @retry is the caller's
 * aging count: the number of times this logical operation has already retried
 * (0 on the first attempt, incremented and passed back in on each retry).  It
 * selects the install strategy -- retry 0 takes the flat fail-fast install
 * (never waits on a foreign proxy), retry >= 1 takes the sorted blocking
 * install that waits out a foreign proxy's owner -- and drives the single-edge
 * escalation threshold (URCU_MCAS_ESCALATE), so a starved single-record op
 * eventually stops taking the bare-CAS fast path and commits through a real
 * proxy instead.
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
 * Find the record this descriptor already buffers for @slot, or NULL.  The
 * write-set IS the read-your-own-writes overlay: a record's new_ptr is the
 * value this transaction believes @slot holds.  Linear scan -- the write-set is
 * small (a handful of edges), and the caller's hot path is the MISS (a
 * traversal walks many slots, few of them transacted), so the scan is bounded
 * by nr and the no-descriptor case is filtered by the caller before we are
 * reached.
 */
static inline
struct urcu_mcas_record *urcu_mcas_find(struct urcu_mcas *t, void **slot)
{
	unsigned int i;

	for (i = 0; i < t->nr; i++) {
		if (t->recs[i].slot == slot)
			return &t->recs[i];
	}
	return NULL;
}

/*
 * Record edge {*slot: old -> new} under READ-YOUR-OWN-WRITES semantics: the
 * caller's @old_ptr is the value it observed through a RYW load, i.e. the
 * record's PENDING new_ptr when one exists, not the committed value.  So a
 * same-slot reconcile matches against new_ptr and CHAINS: the record keeps its
 * original old_ptr (the committed value the commit will verify) and advances
 * its new_ptr, collapsing {old -> mid} then {mid -> new} into the single {old
 * -> new} pair an MCAS can represent.  This is exact: commit only ever verifies
 * old_ptr against memory and installs new_ptr, and no intermediate is ever
 * published (a parked proxy resolves to old-or-new), so composing stores
 * functionally on a slot is indistinguishable from applying them in sequence.
 *
 * It also subsumes a load-validate guard's upgrade, the degenerate chain where
 * new_ptr == old_ptr.
 *
 * @old_ptr matching NEITHER the pending new_ptr is a genuine torn read (the
 * caller reached this slot with a value no longer consistent with the
 * transaction's own view -- e.g. it kept a value read before another edge in
 * this transaction rewrote the slot).  Poison it (a LEGAL race, not an embedder
 * bug -- a peer may commit between two reads of the same slot -- so no assert):
 * commit aborts and the caller re-reads.  Never merge, which would forge a
 * record whose old no longer matches the intended write.
 */
static inline
bool urcu_mcas_record_chain(struct urcu_mcas *t, void **slot,
		void *old_ptr, void *new_ptr, int upgrade, uintptr_t tag)
{
	struct urcu_mcas_record *r = urcu_mcas_find(t, slot);

	if (r != NULL) {
		if (caa_unlikely(r->new_ptr != old_ptr)) {
			t->poisoned = 1;	/* torn read-set: commit will abort */
			return true;
		}
		/*
		 * Chain: old_ptr stays the COMMITTED value (what commit
		 * checks), new_ptr advances.  A load-validate (@upgrade == 0)
		 * leaves the pending write intact.
		 */
		if (upgrade)
			r->new_ptr = new_ptr;
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
	urcu_mcas_free(t);			/* pre-commit: writer-context free */
	return n;
}

static inline
void urcu_mcas_destroy(struct urcu_mcas *t)
{
	urcu_mcas_free(t);
}

/* call_rcu callback: deferred destroy. */
static inline
void urcu_mcas_free_rcu(struct rcu_head *head)
{
	urcu_mcas_free(caa_container_of(head,
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
 * through to the full descriptor path so it installs a real proxy and can hold
 * the slot latched against contenders, instead of being shut out of a slot a
 * multi-edge op keeps proxied.  (This lifts a lockout; like any transaction
 * here it remains bounded-blocking, not wait-free.)
 *
 * Call within an RCU read-side section (descriptor existence for concurrent
 * readers).
 */
static inline
bool urcu_mcas_commit(struct urcu_mcas *t,
		void (*call_rcu_fn)(struct rcu_head *,
			void (*)(struct rcu_head *)))
{
	bool committed;
	unsigned int i, planted;

	if (caa_unlikely(t->poisoned)) {
		/*
		 * A same-slot reconcile disagreed on the expected old (a torn
		 * read-set): the write-set is inconsistent.  Never parked, so
		 * free synchronously and abort -- the caller re-reads and
		 * retries.  Must precede the nr==1 fast path: a poisoned
		 * descriptor can still have a single record (two stores to one
		 * slot reconcile to one).
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
		 * escalation below makes it a real, visible transaction
		 * instead.
		 */
		struct urcu_mcas_record *r = &t->recs[0];

		committed = uatomic_cmpxchg(r->slot, r->old_ptr, r->new_ptr) == r->old_ptr;
		urcu_mcas_destroy(t);
		return committed;
	}
	if (t->nr == 1)
		URCU_MCAS_STAT(escalate);	/* single edge, retried past the threshold */
	/*
	 * Age 0 installs optimistically (see drive_install): it never waits on
	 * a foreign proxy, so its records need no slot-address sort to be
	 * deadlock-free.  Age 1+ sorts and installs under the blocking rule.
	 */
	if (t->retry != 0)
		urcu_mcas_sort(t);
	/*
	 * Engine precondition: a transaction's records must target pairwise-
	 * distinct slots (the install protocol's "own proxy"
	 * short-circuit and the read-set checks assume it).  After the
	 * slot-address sort a duplicate shows up as an adjacent equal slot, so
	 * a single linear pass catches an embedder that buffered the same slot
	 * twice.  Debug-only: no cost under NDEBUG.
	 */
	for (i = 1; i < t->nr; i++)
		urcu_assert_debug(t->recs[i].slot != t->recs[i - 1].slot);
	/*
	 * Set the record back-pointers now, deferred from add time.  The
	 * write-set may have been grown (realloc'd, hence moved) while it was
	 * being buffered; from here the descriptor is frozen and about to be
	 * parked, so every record can finally name its now-stable descriptor.
	 */
	for (i = 0; i < t->nr; i++)
		t->recs[i].mcas = t;
	/*
	 * @planted is how far the drive got (sole-driver only) so settle can
	 * convert exactly our planted prefix with a plain store; the load-test
	 * settle ignores it.  Both are static inline, so the value is threaded
	 * for free.
	 */
	/*
	 * Age 0 (sole driver): flat install -- one try-CAS per record, bail at
	 * the first conflict, decide arithmetically.  It returns its planted
	 * prefix length @i directly (the loop index), so the load-free settle
	 * works exactly as for the age-1 sorted install.  Age 1+ is the sorted,
	 * blocking install (it waits a bounded spin on a foreign proxy instead
	 * of failing fast); both plant with a bare CAS.
	 */
	if (t->retry == 0)
		planted = urcu_mcas_drive_install_age0_flat(t);
	else
		planted = urcu_mcas_drive_install(t);
	urcu_mcas_settle(t, planted);		/* owner-only: make our own slots plain */
	committed = urcu_mcas_status(t) == URCU_MCAS_SUCCEEDED;
	call_rcu_fn(&t->rcu_head, urcu_mcas_free_rcu);
	return committed;
}

#ifdef __cplusplus
}
#endif

#endif	/* _URCU_RCU_MCAS_H */
