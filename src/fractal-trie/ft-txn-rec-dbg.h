// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

#ifndef _FT_TXN_REC_DBG_H
#define _FT_TXN_REC_DBG_H

/*
 * ft-txn-rec-dbg: WHICH RECORD LOST, when a commit aborts.
 *
 * ft-txn-kind-stats.h counts what each txn RECORDS and what its commit
 * RETURNS, per creation site.  Between those two lies the question neither can
 * answer: a content txn carries a structural edge, an ordered-cell edge and a
 * §4.B guard in ONE descriptor, so its ABORT tells you nothing about which of
 * them a peer beat.  The engine knows -- `t->recs[planted]` in
 * urcu_txn_desc_commit -- and did not report it.  It does now, through the
 * default-inert URCU_TXN_STAT_ABORT hook.
 *
 * ☠ THE PROXY TAG CANNOT SUBSTITUTE FOR THIS, and the near-miss is worth
 * recording: FT_STATE_PROXY, FT_NR_KEYS_PROXY_TAG and URCU_TXN_TAG are ALL 1.
 * The tag separates a structural edge from everything else and nothing more.
 *
 * WHAT IT IS FOR -- two questions, one instrument:
 *
 *   1. G4's input.  What fraction of aborts is the ORDERED-CELL lane, against
 *      which a per-cell lock would be argued?  (The record volume said 9%;
 *      volume is not the abort rate.)
 *   2. ★ A DETECTOR.  A structural MW record whose owner the op HOLDS should
 *      never lose its CAS: the DLM lock over the slot's owner is exactly what
 *      excludes every peer writer of that word.  If one loses, the abort is not
 *      contention to be converted away -- it is a lock that did not exclude
 *      somebody, and the conversion would turn that word into an unarbitrated
 *      SW park.  FT_AB_OWN_HELD on a structural class is that event, and it is
 *      counted rather than asserted first ([[feedback_claim_without_the_arm]]):
 *      -DFT_ABORT_CLAIM turns the count into an abort with the record dumped.
 *
 * BUILD.  Needs BOTH -DFT_DEBUG_TXN_KIND (the class the FT stamps) and
 * -DURCU_TXN_REC_DBG (the field on the engine record that carries it).  The
 * second is a compilation-wide define on purpose: the descriptor is allocated
 * by urcu-txn.c's slab and filled here, so a field only one TU can see is a
 * layout split.  With only one of the two, the dump says so rather than
 * printing zeros ([[feedback_verify_the_mechanism_ran_before_believing_a_zero]]).
 */

#if defined(FT_DEBUG_TXN_KIND) && defined(URCU_TXN_REC_DBG)

#define FT_ABORT_ATTRIB	1

/*
 * The label packed into urcu_txn_record.dbg_embedder.  One CLASS, plus -- for
 * the classes that a lock could cover -- WHO witnessed the hold, because
 * "a structural record lost" and "a structural record the op OWNS lost" are
 * different findings.
 */
enum ft_ab_cls {
	FT_AB_UNSET = 0,	/* added by a path that stamps nothing */
	FT_AB_SW,		/* an SW park: it cannot lose, so seeing one is itself a finding */
	FT_AB_MW_STRUCT,	/* ★ a structural edge, MW because unarmed: THE DETECTOR'S SUBJECT */
	FT_AB_MW_LOCK,		/* the DLM lock take -- losing IS its job */
	FT_AB_VALIDATE,		/* a read-set guard */
	FT_AB_CELL_HANDLE,	/* an ordered-cell / hlist store recorded straight on the handle */
	FT_AB_MWA_BASE,		/* + enum ft_tk_mwa_class: the nine always-MW populations */
	FT_AB_CLS_NR = FT_AB_MWA_BASE + 9,
};

/* Which witness saw the hold, for the classes where a lock could cover it. */
enum ft_ab_own {
	FT_AB_OWN_NA = 0,	/* no lock could cover this word */
	FT_AB_OWN_HELD,		/* ★ the txn registry names the owner */
	FT_AB_OWN_LEDGER,	/* only this thread's hold ledger does */
	FT_AB_OWN_MISS,		/* neither */
	FT_AB_OWN_NR,
};

#define FT_AB_CLS_SHIFT		0
#define FT_AB_CLS_MASK		0xffu
#define FT_AB_OWN_SHIFT		8
#define FT_AB_OWN_MASK		0x300u
/*
 * Two DIFFERENT labels folded onto one record by read-your-own-writes chaining.
 * The record is then one word written by two FT concerns and no single class is
 * true of it, so it is counted apart instead of credited to either.
 */
#define FT_AB_MIXED		0x400u

#define ft_ab_code(cls, own)	\
	((unsigned int) (cls) | ((unsigned int) (own) << FT_AB_OWN_SHIFT))
#define ft_ab_code_cls(c)	((c) & FT_AB_CLS_MASK)
#define ft_ab_code_own(c)	(((c) & FT_AB_OWN_MASK) >> FT_AB_OWN_SHIFT)

/*
 * The label the NEXT engine record add will carry.  A thread-local hand-off
 * rather than an argument threaded through urcu_txn_store_*: the store API is
 * the engine's, shared with every other embedder, and this must not appear in
 * it.  Set immediately before the store, consumed by the stamp hook inside it.
 */
static __thread unsigned int ft_ab_pending = ft_ab_code(FT_AB_UNSET, FT_AB_OWN_NA);
/* The label of the record that lost the last aborted commit. */
static __thread unsigned int ft_ab_lost;
static __thread int ft_ab_lost_valid;
/*
 * ...and its SLOT TAG, which the class alone does not give.  A structural MW
 * record writes one of two very different words: a child POINTER slot
 * (FT_FLIP_PROXY_TAG), which only a holder of the owning node's lock may write,
 * or the node's packed STATE word (FT_STATE_PROXY), which by design also has
 * writers that hold nothing -- the re-parent sweep's guard, the §4.B validates.
 * Losing a CAS means opposite things on the two, so the alarm must not pool
 * them.
 */
static __thread uintptr_t ft_ab_lost_tag;
#ifdef FT_ABORT_CLAIM
static __thread int ft_ab_claim_armed;
/*
 * SLOT -> the OWNER the record NAMED, for the claim's dump only.  The engine
 * record has no room for a pointer and this is not worth growing it for: a ring
 * walked BACKWARDS finds the most recent naming of a slot, which is the one the
 * losing record made.  Sized far above the largest observed graft descriptor.
 */
#define FT_AB_RING_NR	256
static __thread struct {
	void **slot;
	const void *owner;
	const void *ra;		/* the record's caller, for addr2line */
} ft_ab_ring[FT_AB_RING_NR];
static __thread unsigned int ft_ab_ring_n;

#define FT_AB_NOTE_OWNER(slot_, owner_)					\
	do {								\
		unsigned int k_ = ft_ab_ring_n++ % FT_AB_RING_NR;	\
									\
		ft_ab_ring[k_].slot = (void **) (slot_);		\
		ft_ab_ring[k_].owner = (const void *) (owner_);		\
		ft_ab_ring[k_].ra = __builtin_return_address(0);		\
	} while (0)

static inline
const void *ft_ab_owner_of(void **slot)
{
	unsigned int i, n = ft_ab_ring_n < FT_AB_RING_NR ?
		ft_ab_ring_n : FT_AB_RING_NR;

	for (i = 0; i < n; i++) {
		unsigned int k = (ft_ab_ring_n - 1 - i) % FT_AB_RING_NR;

		if (ft_ab_ring[k].slot == slot)
			return ft_ab_ring[k].owner;
	}
	return NULL;
}

static inline
const void *ft_ab_ra_of(void **slot)
{
	unsigned int i, n = ft_ab_ring_n < FT_AB_RING_NR ?
		ft_ab_ring_n : FT_AB_RING_NR;

	for (i = 0; i < n; i++) {
		unsigned int k = (ft_ab_ring_n - 1 - i) % FT_AB_RING_NR;

		if (ft_ab_ring[k].slot == slot)
			return ft_ab_ring[k].ra;
	}
	return NULL;
}
#else
#define FT_AB_NOTE_OWNER(slot_, owner_)	do { } while (0)
#endif

#define FT_AB_ARM(cls, own)	\
	do { ft_ab_pending = ft_ab_code((cls), (own)); } while (0)

/*
 * ☠ The chain hook fires on a store that folds onto an EXISTING record, so the
 * disagreement it must resolve is between two labels on one word.  Resolve it
 * PESSIMISTICALLY in both fields: a differing class becomes MIXED (credited to
 * neither), and the ownership degrades to the weakest witness of the two, so the
 * detector fires only where EVERY store to that word claimed the hold.
 */
#define URCU_TXN_REC_DBG_STAMP(r)	\
	do { (r)->dbg_embedder = ft_ab_pending; } while (0)
#define URCU_TXN_REC_DBG_CHAIN(r)					\
	do {								\
		unsigned int had_ = (r)->dbg_embedder;			\
		unsigned int got_ = ft_ab_pending;			\
									\
		if (ft_ab_code_cls(had_) != ft_ab_code_cls(got_))	\
			had_ |= FT_AB_MIXED;				\
		if (ft_ab_code_own(got_) > ft_ab_code_own(had_))	\
			had_ = (had_ & ~(unsigned int) FT_AB_OWN_MASK) | \
				(got_ & FT_AB_OWN_MASK);		\
		(r)->dbg_embedder = had_;				\
	} while (0)
/*
 * @r is NULL where the abort has no losing record (a poisoned descriptor).
 * Recorded as such rather than dropped: an unattributed abort that vanishes
 * would make the attributed ones look complete.
 *
 * ☠ A FUNCTION, not a ternary in the macro: the hook is expanded with a literal
 * NULL at the poisoned exit, and `NULL ? NULL->dbg_embedder : x` is a type error
 * even in the arm that cannot run.  Declared here against an INCOMPLETE type
 * (this header is parsed before the engine's, by design) and defined in
 * ft-txn-kind-stats.h, where the record is complete and the counters exist.
 */
struct urcu_txn_record;
struct urcu_txn_desc;
static void ft_ab_note_lost(const struct urcu_txn_desc *t,
		const struct urcu_txn_record *r);

#define URCU_TXN_STAT_ABORT(t, r)	ft_ab_note_lost((t), (r))

#else	/* the instrument is not built */

#define FT_AB_ARM(cls, own)		do { } while (0)
#define FT_AB_NOTE_OWNER(slot_, owner_)	do { } while (0)

#endif	/* FT_DEBUG_TXN_KIND && URCU_TXN_REC_DBG */

#endif /* _FT_TXN_REC_DBG_H */
