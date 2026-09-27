// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

#ifndef _FT_TXN_HLIST_H
#define _FT_TXN_HLIST_H

/*
 * ft-txn-hlist: the fractal trie's duplicate chain, expressed as a small set of
 * TRANSACTIONAL link primitives on the RCU MCAS engine.  It is an FT-PRIVATE
 * counterpart to the generic concurrent <urcu/rcu-txn-hlist.h>, WITHOUT that
 * header's multi-writer arbitration: every chain mutation runs under the
 * head-holder's lock (MW LOCK_FINE Step A -- one writer per chain), so the
 * neighbour-mid-deletion load-validate and the marked-target -ENOENT/-EAGAIN
 * bails in insert_after/del/replace are dead and gone.  What remains from that
 * header is the forward slot as the sole serializer and the "next"-only mark --
 * the mark stays because it is BOTH the reader-visible logical-delete (readers
 * mask it) AND the freeze half the head ops fold into their structural flip-txn
 * (ft_hlist_freeze_prepare).  The MCAS engine (proxy on bit 0) also stays: the
 * chain edits still FOLD into the host op's flip-txn so a chain edit and the
 * trie<->head anchor edge commit atomically.  This does NOT reshape the chain
 * into the kernel hlist `**pprev' encoding -- it keeps FT's duplicate chain
 * EXACTLY as it is and only gives its link maintenance a clean abstraction.
 *
 * Representation (struct cds_ft_node, unchanged, see <urcu/fractal-trie.h>):
 *   next : cds_ft_node *   forward link; reader-visible; MARK-able
 *                          (CDS_FT_NODE_REMOVED_FLAG on bit 1); the engine proxy
 *                          rides bit 0.  Transacted under FT_HLIST_TAG.
 *   prev : cds_ft_node *   back link to the PREDECESSOR NODE (an interior
 *                          duplicate's predecessor).  Transacted too (the
 *                          coherent-both-directions edge); under the single
 *                          writer a del's re-read of the predecessor is stable.
 *
 * Why prev-as-node, not pprev (the FT-specific reason a generic reuse fails):
 * FT overloads cds_ft_node.prev.  For a chain HEAD it is the cell/parent (the
 * holder route, dereferenced as prev->parent); for a duplicate it is the
 * predecessor node -- and headness is decided by ft_node_external(prev)
 * (a duplicate's prev is an external node; a head's is not).  The kernel hlist's
 * pprev would make a duplicate's prev a raw slot address (&pred->next), which is
 * not an external node, so it would break headness detection and the holder
 * route across ~20 readers.  Keeping prev a node preserves all of that untouched;
 * the primitives just compute the forward slot as &pred->next when they need it.
 *
 * Head boundary (the "Option B" cut).  These ops own strictly the INTERIOR of a
 * chain (the head node's `next' inward, H -> D1 -> ... -> tail); the trie<->head
 * anchor slot (external_nodes) is a STRUCTURAL trie slot (FT_FLIP_PROXY_TAG, FT
 * descent resolver, relocated by holder recompaction, flipped by
 * external-promote), managed by FT structural code -- never by these ops.  A
 * duplicate is inserted after the head node H (on H->next), never "at head", and
 * the head is removed structurally, so these ops never touch external_nodes.
 * The one interop invariant the structural head-remove must uphold is
 * MARK(H->next) in its own commit, so a concurrent insert_after(H) onto a
 * sole-node chain sees the mark and aborts -- the same guard interior del/insert
 * already give one another.
 *
 * Bits: a live "next" carries the deletion MARK (bit 1) and the engine proxy TAG
 * (bit 0); a transacted "prev" carries only the proxy (never marked).
 * cds_ft_node is naturally (pointer) aligned, so (value & TAG) != TAG for every
 * live next/prev/NULL.  prev is read by FT's writer-side headness/holder logic
 * with a plain (unresolved) load: correct under the current retained caller
 * exclusion (no proxy at rest); those reads gain a resolve when concurrent
 * writers are enabled.
 *
 * Only the composable *_prepare primitives are provided (insert-after / del /
 * replace): FT folds each into the surrounding op's MCAS txn -- as the
 * ordered-cell list folds its splices via urcu_txn_list_*_prepare -- so it never
 * needs a self-contained _rcu bracket, and it never head-inserts a duplicate.
 */

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <urcu/compiler.h>
#include <urcu/uatomic.h>
#include <urcu/rcu-txn-mcas.h>
#include <urcu/rcu-txn.h>
#include <urcu-pointer.h>

#include <urcu/fractal-trie.h>	/* struct cds_ft_node, CDS_FT_NODE_REMOVED_FLAG */

/*
 * Engine proxy tag for every interior chain slot (a node next/prev).  The
 * interior chain holds only duplicate leaves (never the structural children
 * that carry FT_FLIP_PROXY_TAG), so the plain bit-0 engine tag suffices; it
 * coexists in one TU with the ordered-cell list's URCU_TXN_TAG (the slots never
 * overlap) and with the trie's FT_FLIP_PROXY_TAG structural edges.
 */
#define FT_HLIST_TAG	URCU_TXN_TAG

/* Logical-deletion mark: the public cds_ft_node removal tombstone (bit 1). */
#define FT_HLIST_MARK	CDS_FT_NODE_REMOVED_FLAG
/*
 * ☠ A prev WORD IS READ BY READERS, and a record parked on it must not be
 * bit-identical to a value it can legitimately hold.  Every up-walk from an
 * external -- the skip re-anchor, ft_skip_to_compressed, ft_get_parent_rcu --
 * loads prev through ft_dereference_prev_resolved, which resolves the FT
 * flip-proxy tag (low nibble 0xF) and nothing else; and in list mode a HEAD's
 * prev is an ordinal cell pointer tagged FT_ORD_CELL_TAG, which is BIT 0 --
 * the same bit as URCU_TXN_TAG.  A prev record with the hlist tag would
 * therefore read as a cell to a reader hopping a duplicate chain (a member's
 * prev is its predecessor, a raw external pointer; the hop stops at the first
 * non-external value and takes it for the head's word).  So prev-word records
 * carry the flip-proxy tag, which no prev value has (nodes and cells are
 * 16-byte aligned), and the reader's one resolver covers them.  next-word
 * records keep the hlist tag: next is read through ft_hlist_resolve.
 */
#define FT_HLIST_PREV_TAG	FT_FLIP_PROXY_TAG

/*
 * Worst-case MCAS edge counts, for the caller's txn reservation.  A tail append
 * (succ == NULL) records only the single pos->next edge; a mid-chain insert also
 * records the succ->next load-validate guard and the succ->prev back-edge.  del
 * and replace touch elem->next (mark), pred->next, next->prev plus the
 * next->next guard.  A freeze records only the elem->next mark (one edge), folded
 * into a host op's structural flip-txn (a chain head leaves through its
 * FT-structural anchor, not a predecessor->next store).
 */
#define FT_HLIST_INSERT_AFTER_MAX_EDGES	3
#define FT_HLIST_DEL_MAX_EDGES		4
#define FT_HLIST_REPLACE_MAX_EDGES	4
#define FT_HLIST_FREEZE_MAX_EDGES	1

/*
 * Every chain edge below goes through here, for one reason beyond the store:
 * these records carry no back-pointer to the flip-txn they fold into, so the
 * per-creation-site table in ft-txn-kind-stats.h cannot attribute them.  This
 * is where they are counted instead -- as one global class, which is all they
 * need to be: a chain edge is MW because the chain is not covered by the
 * structural node locks.
 *
 * ☞ "MW ON PURPOSE ... not part of the conservative-MW conversion" is what this
 * used to say, and it is no longer true: cds_ft_node.next/.prev are [debt] in
 * the word-kind table -- a named owner, bound for SW under the nearest ancestor
 * lock -- and ft_hlist_store_sw below is the first site to make the trip.  The
 * counter is split MW/SW for the same reason, or a conversion would read as the
 * site going quiet.
 */
/*
 * The duplicate-chain hold audit lives in ft-mutation-helpers.h (it needs the
 * per-thread hold ledger, which is defined there, and this header is included
 * well before it).  Declare the coarse arm so ft_hlist_store_chain_at can report;
 * one TU, so the later definition resolves it.
 */
static inline
int ft_hlist_store_sw_at(const char *fn, int line, const struct cds_ft *ft,
		struct urcu_txn *txn, void **slot, void *old_ptr, void *new_ptr,
		uintptr_t tag);

/*
 * Did the coarse dispatch above actually FIRE?  A green run in which it never
 * ran is indistinguishable from one where it is wrong, so count both sides.
 */
#ifdef FT_DEBUG_DUAL_SITE
unsigned long ft_hlist_coarse[2];
static void ft_hlist_coarse_report(void) __attribute__((destructor));
static void ft_hlist_coarse_report(void)
{
	fprintf(stderr, "FT HLIST KIND coarse_sw=%lu fine_mw=%lu\n",
		uatomic_read(&ft_hlist_coarse[1]),
		uatomic_read(&ft_hlist_coarse[0]));
}
# define FT_HLIST_COARSE_TALLY(c)	uatomic_inc(&ft_hlist_coarse[(c)])
#else
# define FT_HLIST_COARSE_TALLY(c)	do { (void) (c); } while (0)
#endif


#ifdef FT_DEBUG_CHAIN_HOLD
static inline void ft_ch_audit_coarse_at(const char *fn, int line);
# define FT_CH_COARSE(fn, line)	ft_ch_audit_coarse_at((fn), (line))
#else
# define FT_CH_COARSE(fn, line)	do { (void) (fn); (void) (line); } while (0)
#endif

/*
 * ☞ THE DUPLICATE-CHAIN CANARY  (-DFT_DEBUG_CHAIN_CANARY)
 *
 * The governing rule for this class is "the duplicate chain is protected by the
 * NEAREST ANCESTOR LOCK, period".  Asking whether a given op HELD that lock has
 * proved unanswerable from inside the library: the three witnesses (txn
 * registry, per-thread hold ledger, lock ctx) each see a different subset, and
 * the narrow ones produce WRONG ZEROS -- the chain hold audit measures
 * ft_detach_node at REG=2,579,201 / LED=0, so a ledger-only verdict calls every
 * one of those holds a violation.
 *
 * So stop interrogating the CONSUMER and tag the PRODUCER: record, for each
 * chain WORD, the (fn, line) of the site that last stored it.  When a reader
 * then finds a value it did not expect -- ft_hlist_freeze_chain_prepare's tail
 * finding its derived NULL replaced by an appended duplicate -- the word names
 * the site that put it there.  No witness, no inference.
 *
 * ☠ KEYED ON THE SLOT, NOT ON A FIELD INSIDE THE NODE, and that is not a
 * stylistic choice.  The first spelling put the tag in struct cds_ft_node and
 * recovered the node from the slot with caa_container_of.  It corrupted the
 * heap within seven tests ("corrupted size vs. prev_size"), because a chain
 * store's slot is not always inside a live cds_ft_node -- ft_hlist_del_prepare
 * derives @pred by LOADING @elem->prev, and a stale or head @pred makes
 * &pred->next point into something else entirely.  The existing code survives
 * that because the store is RECORDED and the expected-old check rejects it at
 * commit; an immediate debug write has no such protection.  (☞ that is worth
 * remembering on its own: the MW CAS is load-bearing for more than
 * arbitration here.)  A side table keyed by address writes only its own
 * memory and can tag any word, node or not.
 *
 * Lossy by construction: one open-addressed probe, last writer wins, no
 * locking.  A name for a human to read after the fact, never a value anything
 * branches on -- so a collision costs a misattributed row, never correctness.
 *
 * Costs 32 MiB of BSS in a build that defines the flag, and nothing at all in
 * one that does not (the stamp compiles to `do { } while (0)`).
 */
#ifdef FT_DEBUG_CHAIN_CANARY
#define FT_CANARY_SLOTS		(1u << 20)
struct ft_canary_ent {
	void **slot;
	const char *fn;
	int line;
	bool raw;		/* last writer bypassed the record layer */
	unsigned char raw_id;	/* which raw producer, when @raw */
	unsigned long tid;
};
extern struct ft_canary_ent ft_canary_tab[FT_CANARY_SLOTS];
struct ft_canary_ent ft_canary_tab[FT_CANARY_SLOTS];

static inline
unsigned int ft_canary_hash(const void *slot)
{
	uintptr_t v = (uintptr_t) slot;

	v ^= v >> 33;
	v *= 0xff51afd7ed558ccdULL;
	v ^= v >> 33;
	return (unsigned int) (v & (FT_CANARY_SLOTS - 1));
}

/*
 * ☞ THE SW-vs-RAW SLOT MIX, which is the question the flip actually turns on.
 *
 * rcu-txn.h's rule is "a slot is SW xor MW, GLOBALLY".  A RAW store -- one that
 * never reaches the engine -- is outside that rule entirely: it cannot be
 * parked, and if the same SLOT is also written through the record layer then
 * the day that layer parks SW, the raw store and the settle race on one word.
 * Reading the call graph cannot answer this, because a node's role changes: the
 * same cds_ft_node.prev is a HEAD's back edge (written raw) at one moment and a
 * chain MEMBER's prev (recorded) at another.
 *
 * The canary is keyed BY SLOT, so it can answer it directly: tag each entry
 * with whether its last writer was raw, and count the transitions.  A non-zero
 * count in either direction means one address is written both ways and the
 * flip's hazard is LIVE; zero means the two populations are disjoint.
 *
 * ☠ Counted only on a canary HIT (e->slot == slot).  The table is lossy by
 * design -- one open-addressed probe, last writer wins -- so a miss is an
 * evicted entry, not a transition, and scoring it would manufacture a mix.
 */
/*
 * ☞ SPLIT BY PRODUCER, because "is one slot written both ways" and "does that
 * MATTER" are different questions and only the second decides the flip.  Each
 * raw producer has a very different exposure, already measured by the head-word
 * audit, so attributing a transition to one of them answers Mathieu's question
 * -- are these raw stores on HIDDEN nodes? -- instead of inferring it:
 *
 *   0 MARK_REMOVED  ft_node_mark_removed_flip, &node->next on a LIVE node it is
 *                   tombstoning.  NOT hidden.  It is the only one that is not,
 *                   and it is proxy-aware (it spins out a parked FT_HLIST_TAG
 *                   rather than trampling it).
 *   1 PUB_EXT_PREV  ft_publish_external_nodes_prev, &head->prev list-off.
 *                   Audited 33,667/33,667 under the FT-wide writer lock.
 *   2 SET_PARENT    ft_set_parent's external arm.  Its callers' audit rows read
 *                   HIDDEN with LIVE=0 (ft_attach_node 1.5M, the splits, the
 *                   recompact), i.e. FRESH nodes no reader can reach.
 *   3 SET_PARENT_RAW  ft_set_parent_raw's external arm.  Audited 25,021/25,195
 *                   wlock-held, 174 NOHOLDER.
 */
#define FT_CANARY_RAW_NR	4
extern unsigned long ft_canary_mix_raw_after_rec[FT_CANARY_RAW_NR],
	ft_canary_mix_rec_after_raw[FT_CANARY_RAW_NR],
	ft_canary_raw_stores[FT_CANARY_RAW_NR], ft_canary_rec_stores;
unsigned long ft_canary_mix_raw_after_rec[FT_CANARY_RAW_NR],
	ft_canary_mix_rec_after_raw[FT_CANARY_RAW_NR],
	ft_canary_raw_stores[FT_CANARY_RAW_NR], ft_canary_rec_stores;

static inline
void ft_chain_canary_stamp_id(const char *fn, int line, void **slot, bool raw,
		unsigned char id)
{
	struct ft_canary_ent *e = &ft_canary_tab[ft_canary_hash(slot)];

	if (raw)
		uatomic_inc(&ft_canary_raw_stores[id]);
	else
		uatomic_inc(&ft_canary_rec_stores);
	if (e->slot == slot && e->raw != raw) {
		if (raw)
			uatomic_inc(&ft_canary_mix_raw_after_rec[id]);
		else
			uatomic_inc(&ft_canary_mix_rec_after_raw[e->raw_id]);
	}
	e->slot = slot;
	e->fn = fn;
	e->line = line;
	e->raw = raw;
	e->raw_id = raw ? id : e->raw_id;
	e->tid = (unsigned long) pthread_self();
}

static inline
void ft_chain_canary_stamp_at(const char *fn, int line, void **slot, bool raw)
{
	ft_chain_canary_stamp_id(fn, line, slot, raw, 0);
}

static inline
void ft_chain_canary_stamp(const char *fn, int line, void **slot)
{
	ft_chain_canary_stamp_at(fn, line, slot, false);
}

static void ft_canary_mix_report(void) __attribute__((destructor));
static void ft_canary_mix_report(void)
{
	{
		static const char * const nm[FT_CANARY_RAW_NR] = {
			"MARK_REMOVED(live node)", "PUB_EXT_PREV(wlock)",
			"SET_PARENT(hidden)", "SET_PARENT_RAW(wlock)" };
		unsigned int i;

		fprintf(stderr, "FT CHAIN SLOT-MIX by raw producer "
			"(recorded=%lu):\n",
			uatomic_read(&ft_canary_rec_stores));
		for (i = 0; i < FT_CANARY_RAW_NR; i++)
			fprintf(stderr, "  %-26s raw=%-10lu "
				"raw_after_rec=%-8lu rec_after_raw=%lu\n",
				nm[i], uatomic_read(&ft_canary_raw_stores[i]),
				uatomic_read(&ft_canary_mix_raw_after_rec[i]),
				uatomic_read(&ft_canary_mix_rec_after_raw[i]));
	}
}

/* NULL when the word has never been stored through a chain store helper. */
static inline
const struct ft_canary_ent *ft_chain_canary_of(void **slot)
{
	const struct ft_canary_ent *e = &ft_canary_tab[ft_canary_hash(slot)];

	return e->slot == slot ? e : NULL;
}

# define FT_CHAIN_CANARY_STAMP(fn, line, slot)				\
	ft_chain_canary_stamp((fn), (line), (void **) (slot))
/* FT_CHAIN_CANARY_RAW is declared in ft-helpers.h: the raw producers live in
 * files this one is included AFTER, so the macro has to exist before here. */
#else
# define FT_CHAIN_CANARY_STAMP(fn, line, slot)		do { } while (0)
#endif

static inline
int ft_hlist_store_chain_at(const char *fn, int line, const struct cds_ft *ft,
		struct urcu_txn *txn,
		void **slot, void *old_ptr, void *new_ptr, uintptr_t tag)
{
	/*
	 * ☑ THIS CLASS PARKS.  cds_ft_node.next / .prev are SW by default; the
	 * MW arm below is an ABLATION (-DNO_FEATURE_FT_CHAIN_SW), not a
	 * fallback.
	 *
	 * It is the largest conversion in the tree -- 14.5M stores per ft_inv
	 * leg, an order of magnitude above every row of the §11.6 residue and
	 * ~4.5x the ordered-cell list it used to be counted inside -- and it was
	 * ATTEMPTED AND WITHDRAWN TWICE before this.  Both attempts gated GREEN.
	 * That is the single most important thing to know before touching any of
	 * it: on this surface a green gate has twice been produced by an
	 * instrument that could not see, so the conversion rests on the six
	 * refutations below being PAID, each by a measurement, and the gate is
	 * only the last check rather than the argument.
	 *
	 *   1 KIND PER SLOT, NOT PER TXN.  The dispatch is unconditional.  The
	 *     2026-09-17 attempt asked ft_flip_txn_owns per prepare, and the
	 *     three point ops take the holder into a STACK ft_held_anchor BEFORE
	 *     the content txn exists -- empty registry, so they could only answer
	 *     MW while the structural sites answered SW.  One slot, two kinds, by
	 *     construction, against rcu-txn.h's "a slot is SW xor MW, GLOBALLY".
	 *     The engine's own kind-conflict assert is silent under
	 *     --enable-rcu-debug across six legs.
	 *
	 *   2 THE TREE'S GATE WAS A COUNTER READING ZERO, never "this is safe".
	 *     Every converted freeze site's plan-stale counter reads 0; the two
	 *     that are not converted are measured UNREACHED (0 of 0), which is a
	 *     coverage statement and is labelled as one.
	 *
	 *   3 AN SW ABORT WRITES.  urcu_txn_settle stores old_ptr back BLIND, so
	 *     an expected-old that is a caller's BELIEF becomes a published lie.
	 *     FT_CHAIN_OLD converts the five of them to loads under this same
	 *     switch, so the two can never drift apart.
	 *
	 *   4 THE SERIALIZING NULL ON THE APPEND SIDE (ft_hlist_append_run_
	 *     prepare) was missed entirely last time.  It had ZERO concurrent
	 *     coverage until a colliding-key merge test was written for it; it
	 *     now runs ~19k times per leg with 0 stale plans, every one inside a
	 *     bulk window with the FT-wide lock held.
	 *
	 *   5 A RE-READ UNDER THE LOCK MUST NOT BE READ-YOUR-OWN-WRITES.
	 *     ft_hlist_chain_plan_ok uses urcu_txn_load_committed; the previous
	 *     attempt's re-read used urcu_txn_load and in a fused commit read the
	 *     txn's own pending value, i.e. nothing.
	 *
	 *   6 A RAW STORE CANNOT BE PARKED.  The slot mix was measured by
	 *     producer: every transition comes from writes to HIDDEN nodes (a
	 *     fresh node's prev, written before publication), and the one raw
	 *     producer that touches a LIVE chain word is measured unreachable --
	 *     ☠ a coverage statement, so if it ever becomes reachable it must be
	 *     converted to a recorded store, NOT re-argued (see
	 *     ft_node_mark_removed_flip's header for the exact ordering hazard).
	 *
	 * ☞ WHAT IS STILL MW HERE, and stays: the ordinal CELL list
	 * (ft_ord_cell.lnode) is [DESIGN] MW forever and does not come through
	 * this function at all.
	 *
	 * ☞ COARSE RECORDS SW, WHATEVER THE WORD IS.  A coarse trie takes the
	 * FT-wide @writer_lock at its outermost writer scope, so every writer of
	 * every chain and cell word is serialised behind one mutex -- there is no
	 * peer CAS for an SW park to race.  That arm predates the flip and is
	 * unchanged by it.
	 */
	if (ft && !ft->lock_fine) {
		FT_HLIST_COARSE_TALLY(1);
		return ft_hlist_store_sw_at(fn, line, ft, txn, slot,
			old_ptr, new_ptr, tag);
	}
	FT_HLIST_COARSE_TALLY(0);
	/*
	 * ☑ THE COUNTER SAYS "CHAIN", BECAUSE THIS IS NOT A CELL.  Every store
	 * here writes a DUPLICATE-CHAIN word (cds_ft_node.next/.prev).  The
	 * ordinal CELL list (ft_ord_cell.lnode) is [DESIGN] MW forever and does
	 * not come through this function.
	 *
	 * ☠ AND THE TAG CANNOT DO THIS JOB: FT_HLIST_TAG *IS* URCU_TXN_TAG (1),
	 * the ordered-cell tag, so a tag test classes a chain word as a cell.
	 * The PRODUCER is what names the class.
	 *
	 * @fn/@line are the CALLER's, so every chain-word store gets its own
	 * audit row -- the whole point, since the question is per SITE.
	 */
	FT_AB_ARM(FT_AB_CELL_HANDLE, FT_AB_OWN_NA);
	FT_CH_COARSE(fn, line);
	FT_CHAIN_CANARY_STAMP(fn, line, slot);
#ifndef NO_FEATURE_FT_CHAIN_SW
	/*
	 * ★ THE FLIP, NOW THE DEFAULT.  Unconditional, and that is the point: the 2026-09-17
	 * attempt dispatched on a per-TXN predicate, which made the kind a
	 * property of the ASKING TRANSACTION rather than of the SLOT -- the
	 * three point ops take the holder into a STACK ft_held_anchor before
	 * the content txn exists, so their registry is empty and they could
	 * only ever answer MW, while the structural sites answered SW.  Same
	 * slot, two kinds, by construction, against rcu-txn.h's "a slot is SW
	 * xor MW, GLOBALLY".  There is no predicate here.
	 *
	 * What the six refutations needed, each now discharged by measurement
	 * rather than argument:
	 *   1 kind per slot ....... no predicate (this arm)
	 *   2 the tree's gate ..... plan-stale counters read 0 at every
	 *                           converted site; the two unconverted ones
	 *                           are measured UNREACHED, not argued safe
	 *   3 SW abort WRITES ..... the five belief expected-olds become loads
	 *                           with FT_CHAIN_OLD, under this same switch
	 *   4 append_run missed ... covered at last by the colliding-key merge
	 *                           test, 0 stale of ~19k concurrent events
	 *   5 the re-read was RYW .. ft_hlist_chain_plan_ok uses
	 *                           urcu_txn_load_committed
	 *   6 raw producers ....... the slot mix is ALL hidden-node writes; the
	 *                           one raw producer on a live chain word is
	 *                           measured unreachable (see
	 *                           ft_node_mark_removed_flip's header)
	 *
	 * ☠ THE MW ARM IS KEPT AS AN ABLATION, not as a fallback, and the gate
	 * runs it (config "chainmw").  Two reasons, both learned here: a
	 * conversion this size needs a control to bisect against, and an arm no
	 * config builds ROTS -- NO_FEATURE_FT_MERGE went unbuildable for
	 * fourteen commits with no gate to say so.  It is never selected at
	 * runtime: the kind is a property of the SLOT, so a build either parks
	 * this class or does not.
	 */
	FT_TK_COUNT_CHAIN_SW();
	return urcu_txn_store_sw(txn, slot, old_ptr, new_ptr, tag);
#else
	FT_TK_COUNT_CHAIN_MW();
	return urcu_txn_store_mw(txn, slot, old_ptr, new_ptr, tag);
#endif
}

#define ft_hlist_store_chain(ft, txn, slot, old_ptr, new_ptr, tag)	\
	ft_hlist_store_chain_at(__func__, __LINE__, (ft), (txn), (slot),\
		(old_ptr), (new_ptr), (tag))

/*
 * ☞ AN EXPECTED-OLD MUST DESCRIBE THE SLOT ONCE THE RECORD PARKS.
 *
 * Five chain stores pass an expected-old that is an ARGUMENT the caller
 * believes rather than a value read from the word: succ->prev <- pos,
 * pred->next <- elem and next->prev <- elem in the unlink, and replace's two.
 * Under MW that is harmless and slightly stronger than a load -- a wrong belief
 * just fails the install CAS.  ☠ Under SW it is a WRITE: urcu_txn_settle stores
 * old_ptr back BLIND on an abort, so a belief that never matched the word is
 * published into a live slot.  That was refutation #3 of the 2026-09-17
 * attempt.
 *
 * MEASURED with -DFT_DEBUG_HLIST_EXPECTED_OLD: these five disagree with the
 * word ZERO times over both suites and all three spacings, so the conversion is
 * value-neutral -- which is exactly why it must ship WITH the flip and not
 * before it.  Before the flip, replacing a belief with a load only weakens a
 * check that is currently doing real work; after it, the belief is a blind
 * write.  One macro, one switch, so the two can never drift apart.
 *
 * ☠ urcu_txn_load, not _load_committed: this IS the read-your-own-writes case
 * the engine documents.  A fused commit chains several edges on one chain, and
 * the expected-old owed here is the value the slot will hold at install time
 * FROM THIS TXN'S POINT OF VIEW -- its own pending write when it already wrote
 * that word, not the pre-commit value.  (The plan check in
 * ft_hlist_chain_plan_ok is the opposite case and uses _committed for exactly
 * the opposite reason.)
 */
#ifndef NO_FEATURE_FT_CHAIN_SW
# define FT_CHAIN_OLD(txn, slot, believed, tag)				\
	urcu_txn_load((txn), (void **) (slot), (tag))
#else
# define FT_CHAIN_OLD(txn, slot, believed, tag)		((void *) (believed))
#endif


/*
 * THE SW TWIN -- for a chain word whose exclusion is ESTABLISHED, not merely
 * arbitrated by the expected-old.
 *
 * rcu-txn.h states the rule this has to answer: "a slot is SW xor MW,
 * GLOBALLY.  If any other transaction may store_mw() the same slot, this park
 * races that CAS."  So a site may spell itself SW only where no peer writer of
 * that slot can be running -- which is a claim about the OP's exclusion, not
 * about this word.  ft_glue_record_splices is the first such site: it writes a
 * head's prev inside a BULK WINDOW, where the FT-wide writer lock is held and
 * the gate has flipped every point op onto that same lock (measured: 5401 of
 * 5401 stores wlock-held and inside a bulk body, 0 drain seams in-window).
 *
 * ☑ AND THE WHOLE-CLASS STEP HAS NOW HAPPENED.  This header used to forbid
 * copying the spelling to a point-op chain site, because converting ONE site
 * while its peers stayed MW is exactly the SW-park-races-an-MW-CAS the rule
 * above describes.  The duplicate chain migrated as a CLASS instead
 * (ft_hlist_store_chain_at), so every writer of a chain word now parks, and
 * this function remains only as the direct spelling for a site that is
 * excluded by the FT-wide writer lock rather than by a node lock.
 */
static inline
int ft_hlist_store_sw_at(const char *fn, int line, const struct cds_ft *ft,
		struct urcu_txn *txn,
		void **slot, void *old_ptr, void *new_ptr, uintptr_t tag)
{
	FT_TK_COUNT_CHAIN_SW();
	FT_AB_ARM(FT_AB_CELL_HANDLE, FT_AB_OWN_NA);
	FT_CH_COARSE(fn, line);
	FT_CHAIN_CANARY_STAMP(fn, line, slot);
	return urcu_txn_store_sw(txn, slot, old_ptr, new_ptr, tag);
}

#define ft_hlist_store_sw(ft, txn, slot, old_ptr, new_ptr, tag)		\
	ft_hlist_store_sw_at(__func__, __LINE__, (ft), (txn), (slot),	\
		(old_ptr), (new_ptr), (tag))

static inline
void *ft_hlist_set_mark(struct cds_ft_node *n)
{
	return (void *) ((uintptr_t) n | FT_HLIST_MARK);
}

static inline
struct cds_ft_node *ft_hlist_unmark(void *v)
{
	return (struct cds_ft_node *) ((uintptr_t) v & ~(uintptr_t) FT_HLIST_MARK);
}

/*
 * Resolve a raw "next" slot value: strip the engine proxy, then the mark.  Fast
 * path -- a clean value (neither a proxy under FT_HLIST_TAG nor MARK-ed) is
 * returned untouched, so a live-node traversal never runs the unmark AND and the
 * pointer stays out of the load-to-use dependency chain.  Only a tagged value
 * (an in-flight proxy, or a ghost's marked "next") takes the slow path.  prev is
 * writer-only, so it has no reader-side resolve.
 */
static inline
struct cds_ft_node *ft_hlist_resolve(void *raw)
{
	uintptr_t v = (uintptr_t) raw;

	if (caa_unlikely(v & (FT_HLIST_TAG | FT_HLIST_MARK)))
		return ft_hlist_unmark(ft_txn_resolve(raw, FT_HLIST_TAG));
	return (struct cds_ft_node *) raw;
}

/* Resolved forward step (call within an RCU read-side section). */
static inline
struct cds_ft_node *ft_hlist_next_rcu(struct cds_ft_node *node)
{
	return ft_hlist_resolve((void *) rcu_dereference(node->next));
}

#ifdef FT_DEBUG_MARK_REFUSAL
/*
 * TEMPORARY (validated-set probe): which test rows actually REACH the two
 * FT_HLIST_MARK refusals?  A row with a non-zero count here is a row that must
 * be in the validated set for this site; a row reading 0 proves nothing about
 * the refusal and does not belong in the set.
 */
static unsigned long ft_mark_refuse_ins, ft_mark_refuse_del;
# define FT_MARK_REFUSE_TALLY(which)	do {				\
		if ((which) == 0) __atomic_fetch_add(&ft_mark_refuse_ins,\
				1, __ATOMIC_RELAXED);			\
		else __atomic_fetch_add(&ft_mark_refuse_del, 1,		\
				__ATOMIC_RELAXED);			\
	} while (0)
static void ft_mark_refuse_report(void) __attribute__((destructor));
static void ft_mark_refuse_report(void)
{
	fprintf(stderr, "FT_MARK_REFUSE  insert_after=%lu  del=%lu\n",
		ft_mark_refuse_ins, ft_mark_refuse_del);
}
#else
# define FT_MARK_REFUSE_TALLY(which)	do { } while (0)
#endif

/*
 * ft_hlist_insert_after_prepare: record an insert of @newp immediately after
 * @pos, WITHOUT committing.  @pos is the predecessor NODE (the head node H for a
 * first duplicate, or an interior duplicate); the forward slot &pos->next
 * transitions its current successor @succ -> @newp.  @newp is built invisibly
 * (next = @succ, prev = @pos), and the two edges pos->next: succ -> newp and
 * succ->prev: pos -> newp are recorded.  A tail append (@pos == the walked tail,
 * @succ == NULL) is a 1-edge insert with no backward fixup -- FT's ft_chain_node
 * idiom.  OOM is sticky to the commit.
 *
 * Single-writer per chain (MW LOCK_FINE Step A: every chain mutation runs under
 * the head-holder's node lock -- or, before the FT-wide lock drops, that lock;
 * a disjoint-key optimistic writer owns its own chain), so @pos is never
 * concurrently deleted and @succ is never a neighbour mid-deletion.  The
 * multi-writer arbitration those cases needed -- bail -ENOENT on a marked @pos,
 * load-validate &succ->next and retry -EAGAIN on a marked neighbour -- is dead
 * and dropped.
 *
 * ☠ DEFECT FT-SLOT-2: THAT DROP CONTRADICTS THE INTEROP INVARIANT THIS
 * FILE'S HEADER STATES.
 * The header promises that a structural head-remove's MARK(H->next) is what
 * makes "a concurrent insert_after(H) onto a sole-node chain see the mark and
 * abort" -- and the check below was deleted, so for a while nothing looked at
 * the mark.  The remove side upholds its
 * half (ft_hlist_freeze_sole_prepare marks a derived NULL); the insert side no
 * longer checks, and a MARK(NULL) @pos->next reads back as the bare value 2,
 * which passes `succ != NULL` and makes the second store record slot
 * &((struct cds_ft_node *) 2)->prev.  Under LOCK_FINE, where insert and remove
 * are concurrent on the same key by contract, the append's head is derived
 * before the holder acquire, so this is reachable in principle -- the mirror of
 * the remove-side routing defect.  ☑ IT IS NO LONGER "UNPROVEN BY TEST": the
 * shape is reachable, inv_concurrent_insert_replace_nolist reaches it, and the
 * check is RESTORED below as a refusal.  Returns 0, or -ENOENT when @pos is a
 * retired head; the int return was retained for exactly this.
 */
static inline
int ft_hlist_insert_after_prepare(const struct cds_ft *ft, struct urcu_txn *txn,
		struct cds_ft_node *newp,
		struct cds_ft_node *pos)
{
	struct cds_ft_node *succ = (struct cds_ft_node *)
			urcu_txn_load(txn, (void **) &pos->next, FT_HLIST_TAG);

	/*
	 * ☑ THE MARK CHECK IS BACK, and it is a REFUSAL now, not an assert.
	 *
	 * @pos->next MARKED means @pos is a RETIRED head and this append is
	 * building onto a chain that no longer exists.  Left unchecked it is a
	 * LOST INSERT: MARK(NULL) reads back as the bare value 2, passes
	 * `succ != NULL` below, and makes the second store record slot
	 * &((struct cds_ft_node *) 2)->prev.  This file's header states the
	 * interop invariant in as many words -- the structural head-remove marks
	 * H->next in its own commit "so a concurrent insert_after(H) onto a
	 * sole-node chain sees the mark and ABORTS".  Seeing it is this line.
	 *
	 * ☠ IT WAS UNPROVEN AND IS NOW PROVEN.  The obituary above asked for
	 * "either restoring the check or proving the shape unreachable"; the
	 * shape is reachable, and cds_ft_insert_replace is the producer -- it
	 * displaces a whole chain, and once it TOMBSTONES what it displaces (as
	 * its sibling always did for the single node it displaces) an append
	 * that derived @pos before the displacement lands here every time.
	 * Reached by inv_concurrent_insert_replace_nolist.
	 *
	 * -ENOENT is the caller's documented code for "tail already marked"; it
	 * maps the refusal to -EAGAIN and re-descends from the root, which is
	 * the correct answer -- the chain this append aimed at is gone, so the
	 * position must be derived again.
	 */
	if (caa_unlikely((uintptr_t) succ & FT_HLIST_MARK)) {
		FT_MARK_REFUSE_TALLY(0);
		return -ENOENT;
	}

	/* Build the fresh node invisibly. */
	newp->next = succ;
	newp->prev = pos;

	/* pos->next: succ -> newp ; succ->prev: pos -> newp. */
	ft_hlist_store_chain(ft, txn, (void **) &pos->next, succ, newp, FT_HLIST_TAG);
	if (succ != NULL)
		ft_hlist_store_chain(ft, txn, (void **) &succ->prev,
			FT_CHAIN_OLD(txn, &succ->prev, pos, FT_HLIST_PREV_TAG),
			newp, FT_HLIST_PREV_TAG);
	return 0;
}

/*
 * ft_hlist_append_run_prepare: record the append of a whole null-terminated RUN
 * at a chain @tail (tail->next == NULL, as walked by the caller), WITHOUT
 * committing.  Unlike ft_hlist_insert_after_prepare this does NOT touch
 * @run_head->next, so @run_head's own chain (run_head->next -> ...) rides along
 * unmodified -- a run-splice, not a single insert (a merge concatenating a src
 * duplicate run onto a dst tail).  The forward link tail->next: NULL -> run_head
 * is the lone recorded edge and the serializing one (CAS old = NULL: a
 * concurrent freeze of the tail fails this commit), so only the tail carries a
 * proxy while the commit is in flight; the interior of neither run is disturbed.
 *
 * ★ @run_head->prev IS RECORDED BY THE CALLER, not stored here.  It used to
 * be a plain store on the claim "writer-only; readers never read prev, and
 * @run_head is unreachable to readers because a merge detaches and drains the
 * src side before appending it" -- true of ft_merge_spine_copy, the only
 * caller when it was written, FALSE for the one-decide fold, which records
 * the src detach and this splice into ONE txn with the src side still LIVE.
 * And prev IS read: every up-walk from an external starts at it, and a reader
 * on the retired copy of the src junction follows that copy's stale skip word
 * straight to @run_head.  A plain store there was the one torn state the
 * reader's two-descent witness cannot repair -- BOTH descents see the same
 * half-moved head before the flip -- and it survived an abort, which the
 * caller had to undo by hand.  Recorded (FT_HLIST_PREV_TAG, beside the
 * forward link, in ft_glue_record_splices) it flips atomically with the rest
 * of the move and is discarded with an abort; the reservation is two records
 * per splice.
 */
static inline
void ft_hlist_append_run_prepare(const struct cds_ft *ft, struct urcu_txn *txn,
		struct cds_ft_node *tail,
		struct cds_ft_node *run_head)
{
	int ret;

	ret = ft_hlist_store_chain(ft, txn, (void **) &tail->next, NULL, run_head,
			FT_HLIST_TAG);
	assert(!ret);			/* caller reserved the edge up front */
	(void) ret;
}

/*
 * ft_hlist_del_prepare: record the unlink of @elem into @txn WITHOUT committing.
 * Marks @elem (logical delete: elem->next: next -> MARK(next), the target
 * preserved so a reader parked on @elem still follows the chain to the
 * successor / end), unlinks it forward (pred->next: elem -> next) and backward
 * (next->prev: elem -> pred).  On a committed OK THIS call removed @elem; reclaim
 * it after a grace period.  OOM is sticky to the commit.
 *
 * Single-writer per chain (see ft_hlist_insert_after_prepare) covers the
 * NEIGHBOURS: @next is never a neighbour mid-deletion and @pred read this
 * attempt is stable (no peer re-links it), so that half of the multi-writer
 * arbitration (load-validate &next->next, retry -EAGAIN on a marked successor)
 * stays dropped.
 *
 * ☠ IT DOES NOT COVER @elem ITSELF.  A structural head-remove marks a head it
 * retires from OUTSIDE this chain's holder -- and since cds_ft_insert_replace
 * tombstones the chain it displaces, an already-marked @elem does arrive here.
 * The -ENOENT refusal below is therefore LIVE, not dead: this function returns
 * 0, or -ENOENT when @elem is already logically deleted.
 */
static inline
int ft_hlist_del_prepare(const struct cds_ft *ft, struct urcu_txn *txn, struct cds_ft_node *elem)
{
	struct cds_ft_node *next = (struct cds_ft_node *)
			urcu_txn_load(txn, (void **) &elem->next, FT_HLIST_TAG);
	struct cds_ft_node *pred = (struct cds_ft_node *)
			urcu_txn_load(txn, (void **) &elem->prev, FT_HLIST_PREV_TAG);

	/*
	 * ☠ ALREADY MARKED means @elem is ALREADY logically deleted, and building
	 * this delete on top of it is not merely redundant -- it FAULTS.  A
	 * MARK(NULL) @elem->next reads back as the bare value 2, which passes the
	 * `next != NULL` test below and makes the backward edge record slot
	 * &((struct cds_ft_node *) 2)->prev; the commit then installs into
	 * address 2 and SEGVs.  Byte-for-byte the failure
	 * ft_hlist_insert_after_prepare's mark check exists to stop, one function
	 * over -- and it became reachable here for the same reason: nothing used
	 * to tombstone a chain displaced by cds_ft_insert_replace, so a marked
	 * node could not arrive at a del.  Now one can.
	 *
	 * -ENOENT: the caller's comment already names this case ("@node or a
	 * NEIGHBOUR mid-deletion"), destroys the txn and retries from a fresh
	 * position derivation, where the wrapper's tombstone test answers
	 * NOT_FOUND.  Nothing is recorded, so nothing installs.
	 */
	if (caa_unlikely((uintptr_t) next & FT_HLIST_MARK)) {
		FT_MARK_REFUSE_TALLY(1);
		return -ENOENT;
	}

	/*
	 * Mark elem (logical delete), unlink forward (pred->next: elem -> next)
	 * and backward (next->prev: elem -> pred).  When next is NULL the backward
	 * edge vanishes: a 2-edge delete storing MARK(NULL).  The mark on
	 * &elem->next is retained -- readers mask it (ft_hlist_resolve) and the
	 * head ops fold it (ft_hlist_freeze_prepare) for atomicity with the
	 * structural anchor edge.
	 */
	ft_hlist_store_chain(ft, txn, (void **) &elem->next, next,
			ft_hlist_set_mark(next), FT_HLIST_TAG);
	ft_hlist_store_chain(ft, txn, (void **) &pred->next,
		FT_CHAIN_OLD(txn, &pred->next, elem, FT_HLIST_TAG),
		next, FT_HLIST_TAG);
	if (next != NULL)
		ft_hlist_store_chain(ft, txn, (void **) &next->prev,
			FT_CHAIN_OLD(txn, &next->prev, elem, FT_HLIST_PREV_TAG),
			pred, FT_HLIST_PREV_TAG);
	return 0;
}

/*
 * ft_hlist_replace_prepare: record the in-place replacement of @old by @newp
 * into @txn WITHOUT committing.  @newp takes @old's position -- pred->next and
 * next->prev swing to @newp -- while @old is logically removed (its next is
 * marked exactly as del does).  Argument order is (old, new).  On commit reclaim
 * @old after a grace period.  OOM is sticky to the commit.  Used for a non-head
 * duplicate replace (a head replace is FT-structural: it swaps the anchor slot).
 *
 * Single-writer per chain (see ft_hlist_insert_after_prepare): the multi-writer
 * arbitration (-ENOENT on a marked @old, load-validate &next->next and retry
 * -EAGAIN on a marked successor) is dead and dropped.  Always returns 0 (int
 * retained for caller-shape parity).
 */
static inline
int ft_hlist_replace_prepare(const struct cds_ft *ft, struct urcu_txn *txn,
		struct cds_ft_node *old, struct cds_ft_node *newp)
{
	struct cds_ft_node *next = (struct cds_ft_node *)
			urcu_txn_load(txn, (void **) &old->next, FT_HLIST_TAG);
	struct cds_ft_node *pred = (struct cds_ft_node *)
			urcu_txn_load(txn, (void **) &old->prev, FT_HLIST_PREV_TAG);

	/* Build @newp's links invisibly, then swing pred->next and next->prev. */
	newp->next = next;
	newp->prev = pred;

	ft_hlist_store_chain(ft, txn, (void **) &old->next, next,
			ft_hlist_set_mark(next), FT_HLIST_TAG);
	ft_hlist_store_chain(ft, txn, (void **) &pred->next,
		FT_CHAIN_OLD(txn, &pred->next, old, FT_HLIST_TAG),
		newp, FT_HLIST_TAG);
	if (next != NULL)
		ft_hlist_store_chain(ft, txn, (void **) &next->prev,
			FT_CHAIN_OLD(txn, &next->prev, old, FT_HLIST_PREV_TAG),
			newp, FT_HLIST_PREV_TAG);
	return 0;
}

/*
 * ft_hlist_freeze_prepare: record ONLY the logical-deletion mark of @node's
 * forward slot (node->next: succ -> MARK(succ)) into @txn WITHOUT committing --
 * the freeze half of a del with no chain unlink.  A chain HEAD leaves the trie
 * through its FT-structural anchor edge (the parent slot re-point / clear), not a
 * predecessor->next store, so only the mark rides the hlist; folding it into the
 * head op's structural flip-txn makes the freeze and the anchor edge commit
 * atomically -- a reader is never shown @node's head anchor promoted away while
 * @node is still unmarked.  The target is preserved (MARK(succ), or MARK(NULL)
 * for a head with no successor) so a reader parked on @node still follows the
 * chain to the promoted new head / end.  One recorded edge; the caller reserves
 * FT_HLIST_FREEZE_MAX_EDGES on top of the host op's footprint.
 */
static inline
void ft_hlist_freeze_prepare(const struct cds_ft *ft, struct urcu_txn *txn, struct cds_ft_node *node)
{
	void *en = urcu_txn_load(txn, (void **) &node->next, FT_HLIST_TAG);
	int ret;

	ret = ft_hlist_store_chain(ft, txn, (void **) &node->next, en,
			ft_hlist_set_mark((struct cds_ft_node *) en), FT_HLIST_TAG);
	assert(!ret);			/* caller reserved the edge up front */
	(void) ret;
}

/*
 * ft_hlist_freeze_sole_prepare: the freeze of a head the caller DERIVED to be
 * its key's SOLE entry -- the key-disappearing lanes (a detach, the fused
 * ft_remove_one_commit), which prune the branch around it.  The expected-old
 * is that derivation, NULL, never the slot re-loaded here: the derivation was
 * made with nothing held, and a same-key cds_ft_insert -- concurrent with a
 * remove in contract under LOCK_FINE -- can have APPENDED a duplicate to
 * @node since.  Re-loading would mark that duplicate into the tombstone
 * (MARK(N)) and the prune would orphan it behind a retired head: an OK insert
 * whose key never resolves again, and a later remove of it that never
 * terminates (measured: "key LOST after an OK concurrent insert", both list
 * modes).  Recorded against NULL, the append fails this commit's install CAS
 * instead -- the record is MW, so a mismatch is an ABORT, which every
 * key-disappearing caller already routes to a re-derivation that then finds
 * the successor and PROMOTES it.
 */
static inline
void ft_hlist_freeze_sole_prepare(const struct cds_ft *ft, struct urcu_txn *txn, struct cds_ft_node *node)
{
	int ret;

	ret = ft_hlist_store_chain(ft, txn, (void **) &node->next, NULL,
			ft_hlist_set_mark(NULL), FT_HLIST_TAG);
	assert(!ret);			/* caller reserved the edge up front */
	(void) ret;
}


/*
 * ft_hlist_chain_len: how many nodes a freeze of the chain at @head would
 * record -- ONE edge each (FT_HLIST_FREEZE_MAX_EDGES), so this is the caller's
 * reservation.
 *
 * ☠ CALL IT UNDER THE CHAIN HOLDER.  That is what makes a walked count safe to
 * size a reservation with: every producer that could APPEND takes the holder
 * first, so the chain cannot grow between this count and the record walk.
 * Counted without it the freeze would under-reserve and trip
 * ft_hlist_store_chain's assert at the extra node.  Already-marked members are
 * counted too -- they still cost the edge that re-records their mark.
 */
/*
 * ☞ WHY DOES THE TAIL DISAGREE?  (-DFT_DEBUG_HLIST_TAIL_WHY, with
 * -DFEATURE_FT_HOLD_TRACE)
 *
 * ft_hlist_freeze_chain_prepare records its LAST node against the caller's
 * derived NULL, and that derivation disagrees with the word 24,676 times per
 * ft_inv per-node leg (36,427 at exponential) -- while the other six derived
 * expected-olds in this file disagree ZERO times.
 *
 * If the governing rule holds -- every chain mutation under the nearest
 * ancestor lock -- that number should be ZERO too: no append can land while we
 * hold the holder.  It is not zero, so exactly one of two things is true, and
 * they have entirely different fixes:
 *
 *   STALE PLAN   the op DOES hold the holder, but @len was counted BEFORE the
 *                acquire, so the append landed legitimately, before our
 *                exclusion existed.  ft_hlist_chain_len's own header already
 *                forbids this ("☠ CALL IT UNDER THE CHAIN HOLDER"), so the fix
 *                is at the naming site: derive under the lock, or bail.
 *   UNHELD       the op does NOT hold the holder here, and the lock-set
 *                transition is simply not done for this path.  The fix is an
 *                acquire, and nothing in this file changes.
 *
 * So report the HOLDER witness and the CALLER that counted @len, interned by
 * (fn, line).  The chain hold audit cannot answer this: it scores the moment of
 * the STORE, and this is a question about the moment of the DERIVATION.
 *
 * ☑ ANSWERED, AND THE FIRST ANSWER WAS THE WRONG ONE.  Asked of the hold
 * ledger's stamp alone the table read US = 0 everywhere, which was recorded as
 * UNHELD.  The stamp cannot see a REGISTERED hold at all -- see the withdrawal
 * in ft_hlist_store_chain_at's parked note -- so the three witness columns are
 * now read TOGETHER, at the same event, and the answer for cds_ft_remove's
 * sole-entry path is STALE PLAN or an appender gap, never UNHELD: the registry
 * owns the holder at 100% of its disagreements.
 *
 * ☠ WHICH IS WHY @reg_own EXISTS AND WHY THE STAMP IS KEPT BESIDE IT.  A
 * single-witness verdict on this question has now been wrong in both
 * directions; the columns are only meaningful as a set.
 */
#ifdef FT_DEBUG_HLIST_TAIL_WHY
#define FT_HLIST_WHY_MAX	16
struct ft_hlist_why_site {
	const char *fn;
	int line;
	unsigned long tails, disagree;
	/*
	 * ☞ THE EXCLUSION ORACLE'S ANSWER, read at the moment of the
	 * disagreement.  cds_ft_metadata.dbg_owner_tid is CLAIMED by the hold
	 * ledger when an op takes a member and YIELDED when it drops it, so it
	 * is a POSITIVE statement of who covers the chain's holder right now --
	 * not the query that reads zero for ops whose hold only the registry
	 * can see.
	 *
	 *   own      we hold the holder => the APPENDER did not  => its gap
	 *   foreign  another op holds it => WE do not            => our gap
	 *   none     nobody claims it    => neither is covered, or the op's
	 *            hold never reached the ledger (an ENTRY-less claim)
	 */
	unsigned long excl_own, excl_foreign, excl_none;
	/*
	 * ☠☠ AND THE SAME EVENT ASKED OF THE OTHER TWO WITNESSES, because the
	 * stamp above CANNOT SEE A CONVERTED HOLD and the tree says so twice:
	 *
	 *   - ft_flip_txn_record_release_lock calls ft_hold_trace_drop() the
	 *     moment it RECORDS the release (ft-mutation-helpers.h, "the ledger
	 *     is not a hold count -- it is an OUTSTANDING-RELEASE count"), and
	 *     that drop YIELDS the stamp.  Every site that acquires a word and
	 *     immediately plants its {LOCK|s -> s} terminal -- which is what the
	 *     lock-or-guard hoists do -- therefore reads US = 0 from then on,
	 *     while the word still carries FT_STATE_LOCK and is still owned, by
	 *     the commit rather than by the frame.
	 *   - the chain canary's own header already prices it: the chain hold
	 *     audit measures ft_detach_node at REG = 2,579,201 / LED = 0.
	 *
	 * So report the REGISTRY (@reg_own) beside it, at the SAME event, from
	 * the ft_flip_txn whose handle this freeze was given.  A park is licensed
	 * by that witness and by no other (§11.9), so it is the one the flip
	 * actually needs -- and US = 0 with @reg_own large is the wrong zero,
	 * not a gap.  @led_own is the ledger asked directly, to show the stamp
	 * and the ledger agree and that the split is REGISTRY-vs-rest.
	 */
	unsigned long reg_own, led_own, reg_or_led, ftxn_none, ftxn_stale;
	/*
	 * ☠ THE CONTRADICTION COLUMN.  @excl_foreign says a PEER positively
	 * claims the chain's holder; @reg_own says THIS txn's registry holds the
	 * same word.  Both cannot be true -- ft_meta_lock_acquire refuses a word
	 * that already carries FT_STATE_LOCK -- so an overlap is either a real
	 * exclusion violation or an artefact of one of the two witnesses (a
	 * ledger entry leaked past its op, or a registry entry whose release is
	 * recorded but whose commit has not landed).  Counted rather than
	 * assumed away, because "a peer holds it" is the one column in this table
	 * that a blindness argument does NOT explain, and it only carries that
	 * weight if it is disjoint from our own hold.
	 */
	unsigned long reg_and_peer;
	/*
	 * ☠ AND THE MODE, because "the registry does not own the holder" is only
	 * a gap under FINE.  A COARSE trie serialises every writer behind the
	 * FT-wide mutex and an EXCLUSIVE trie has one writer, so neither takes a
	 * per-node lock at all -- scoring those as unprotected would manufacture
	 * a gap out of a configuration that has no locks by design.  Only
	 * @unreg_fine is owed an explanation.
	 */
	unsigned long unreg_fine, unreg_coarse, unreg_excl, unreg_wlock;
#define FT_HLIST_WHY_PROD	8
	/* WHO stored the surprise: read off the duplicate's own canary. */
	const char *prod_fn[FT_HLIST_WHY_PROD];
	int prod_line[FT_HLIST_WHY_PROD];
	unsigned long prod_tid[FT_HLIST_WHY_PROD];
	unsigned long prod_n[FT_HLIST_WHY_PROD];
	unsigned int nr_prod;
	unsigned long prod_overflow;
};
extern struct ft_hlist_why_site ft_hlist_why_sites[FT_HLIST_WHY_MAX];
extern unsigned int ft_hlist_why_n;
extern __thread const char *ft_hlist_why_fn;
extern __thread int ft_hlist_why_line;
struct ft_hlist_why_site ft_hlist_why_sites[FT_HLIST_WHY_MAX];
unsigned int ft_hlist_why_n;
__thread const char *ft_hlist_why_fn;
__thread int ft_hlist_why_line;
/*
 * THE REGISTRY WITNESS' SOURCE.  A freeze helper is handed the ENGINE handle
 * (struct urcu_txn *), and the lock registry lives one level up in the
 * ft_flip_txn that owns it -- so the wrapper cannot be recovered from the
 * handle.  Every caller of a *_prepare form reaches it through
 * ft_flip_txn_handle(), so stamp the wrapper THERE, in the one place that
 * converts one into the other, and verify it here: the classifier counts
 * @ftxn_stale whenever the stamped wrapper's handle is not the handle this
 * record is going into, so a mis-stamp reads as its own column instead of
 * silently answering for the wrong transaction.
 */
struct ft_flip_txn;
extern __thread struct ft_flip_txn *ft_hlist_why_ftxn;
__thread struct ft_flip_txn *ft_hlist_why_ftxn;
static inline bool ft_flip_txn_owns(const struct ft_flip_txn *t,
		const struct cds_ft_metadata *owner);
static inline struct urcu_txn *ft_flip_txn_handle(struct ft_flip_txn *t);
static bool ft_hold_trace_holds(const struct cds_ft_metadata *lock);
/*
 * ☞ COUNT THE BAIL, DO NOT JUST PREVENT THE EVENT.  ft_hlist_chain_plan_ok
 * turns a stale plan into a retry BEFORE the freeze records anything, so the
 * DISAGREE rows above go quiet -- and a quiet row is indistinguishable from a
 * path that stopped being taken.  This counter is what tells them apart: the
 * traffic that used to land in DISAGREE has to reappear here, one for one.
 */
extern unsigned long ft_hlist_plan_bail;
unsigned long ft_hlist_plan_bail;
/*
 * WHY the plan was refused -- SHORT (the walk hit the end before @len),
 * TAIL_MARKED (the tail is already logically deleted but still linked, so
 * ft_hlist_chain_len counted it while its word is MARK(NULL), not NULL) or
 * TAIL_LIVE (a duplicate really was appended).  Only the last is the race the
 * check exists for; the other two are disagreements between the COUNT's notion
 * of "end of chain" and the RECORD's, and a refusal on either is stable across
 * a retry -- i.e. a livelock, not an arbitration.
 */
extern unsigned long ft_hlist_plan_why[3];
unsigned long ft_hlist_plan_why[3];
/*
 * ☞ OBSERVE-ONLY ARM, for the freeze sites whose @len is their CALLER's and
 * whose bail would need its own pre-commit terminal (the fold / glue / merge
 * paths).  Before adding four more bails -- the change that already wedged this
 * tree once by leaking a lock -- ASK WHETHER THEY CAN EVER FIRE.  The tree's own
 * gate for the SW flip is a counter reading ZERO across every spacing, never an
 * argument that a path is safe, so this is the measurement that gate wants.
 * Indices: 0 = the bulk fold, 1 = the external-promote arm, 2 = the src_cn
 * retire arm, 3 = the merge collapse, 4 = ft_glue_record_splices' run append
 * (ft_hlist_append_run_prepare's literal NULL -- the same serializing word from
 * the APPEND side, and the site the 2026-09-17 flip attempt missed entirely).
 */
extern unsigned long ft_hlist_plan_obs[5], ft_hlist_plan_obs_n[5];
unsigned long ft_hlist_plan_obs[5], ft_hlist_plan_obs_n[5];

static inline
void ft_hlist_plan_snap(void)
{
	unsigned long n = uatomic_add_return(&ft_hlist_plan_bail, 1);

	if (n <= 8 || !(n % 4096))
		fprintf(stderr, "FT HLIST PLAN-BAIL %lu  SHORT=%lu "
			"TAIL_LIVE=%lu (noted: TAIL_MARKED=%lu)\n", n,
			uatomic_read(&ft_hlist_plan_why[0]),
			uatomic_read(&ft_hlist_plan_why[2]),
			uatomic_read(&ft_hlist_plan_why[1]));
}

static void ft_hlist_why_report(void) __attribute__((destructor));
static void ft_hlist_why_report(void)
{
	unsigned int i;

	fprintf(stderr, "FT HLIST PLAN-BAIL (stale plan refused UNDER THE LOCK, "
		"before any record): %lu   BAILED: SHORT=%lu TAIL_LIVE(the race "
		"this check exists for)=%lu | PASSED-but-noted: TAIL_MARKED=%lu\n",
		uatomic_read(&ft_hlist_plan_bail),
		uatomic_read(&ft_hlist_plan_why[0]),
		uatomic_read(&ft_hlist_plan_why[2]),
		uatomic_read(&ft_hlist_plan_why[1]));
	fprintf(stderr, "FT HLIST PLAN-OBSERVE (no bail; must read 0 before these "
		"sites convert): fold=%lu/%lu promote=%lu/%lu src_cn=%lu/%lu "
		"merge=%lu/%lu append_run=%lu/%lu\n",
		uatomic_read(&ft_hlist_plan_obs[0]), uatomic_read(&ft_hlist_plan_obs_n[0]),
		uatomic_read(&ft_hlist_plan_obs[1]), uatomic_read(&ft_hlist_plan_obs_n[1]),
		uatomic_read(&ft_hlist_plan_obs[2]), uatomic_read(&ft_hlist_plan_obs_n[2]),
		uatomic_read(&ft_hlist_plan_obs[3]), uatomic_read(&ft_hlist_plan_obs_n[3]),
		uatomic_read(&ft_hlist_plan_obs[4]), uatomic_read(&ft_hlist_plan_obs_n[4]));
	fprintf(stderr, "FT HLIST TAIL-WHY  (ledger %s)  (canary %s)  sites=%u/%u\n",
#ifdef FEATURE_FT_HOLD_TRACE
		"ON -- PERTURBING, see below",
#else
		"off -- registry-only, the low-perturbation rig",
#endif
#ifdef FT_DEBUG_CHAIN_CANARY
		"ON",
#else
		"OFF -- no producer can be named (rebuild with -DFT_DEBUG_CHAIN_CANARY)",
#endif
		ft_hlist_why_n, FT_HLIST_WHY_MAX);
	for (i = 0; i < ft_hlist_why_n; i++) {
		struct ft_hlist_why_site *e = &ft_hlist_why_sites[i];
		unsigned int k;

		if (!e->tails)
			continue;
		fprintf(stderr, "  len derived at %-28s:%-5d tails=%-10lu DISAGREE=%lu\n",
			e->fn, e->line, e->tails, e->disagree);
		if (e->disagree) {
			fprintf(stderr, "      REGISTRY owns=%lu  (no ftxn stamped=%lu,"
				" stale=%lu)\n",
				e->reg_own, e->ftxn_none, e->ftxn_stale);
			fprintf(stderr, "      NOT owned, by mode: FINE(a real gap)=%lu  "
				"coarse=%lu  exclusive=%lu  FT-wide-lock-held=%lu\n",
				e->unreg_fine, e->unreg_coarse, e->unreg_excl,
				e->unreg_wlock);
#ifdef FEATURE_FT_HOLD_TRACE
			fprintf(stderr, "      LEDGER (perturbing; metadata 48->88 B, "
				"2 atomics/acquire): holds=%lu either=%lu | stamp "
				"US=%lu PEER=%lu NOBODY=%lu | ☠ reg+peer (must be 0)=%lu\n",
				e->led_own, e->reg_or_led, e->excl_own,
				e->excl_foreign, e->excl_none, e->reg_and_peer);
#endif
		}
		for (k = 0; k < e->nr_prod; k++)
			fprintf(stderr, "      <- stored by %-34s:%-5d n=%-8lu tid=%lx\n",
				e->prod_fn[k], e->prod_line[k], e->prod_n[k],
				e->prod_tid[k]);
		if (e->prod_overflow)
			fprintf(stderr, "      <- (%lu more, producer table full)\n",
				e->prod_overflow);
	}
}

static inline
void ft_hlist_why_tail(struct cds_ft *ft, struct urcu_txn *txn,
		struct cds_ft_node *head)
{
	void *live = (void *) CMM_LOAD_SHARED(head->next);

	struct ft_hlist_why_site *e = NULL;
	const char *fn = ft_hlist_why_fn ? ft_hlist_why_fn : "(unstamped)";
	int line = ft_hlist_why_line;
	unsigned int i;

	(void) ft;
	for (i = 0; i < ft_hlist_why_n; i++)
		if (ft_hlist_why_sites[i].line == line &&
				ft_hlist_why_sites[i].fn == fn) {
			e = &ft_hlist_why_sites[i];
			break;
		}
	if (!e) {
		if (ft_hlist_why_n >= FT_HLIST_WHY_MAX)
			return;
		e = &ft_hlist_why_sites[ft_hlist_why_n++];
		e->fn = fn;
		e->line = line;
	}
	uatomic_inc(&e->tails);
	if (!live)
		return;
	uatomic_inc(&e->disagree);
	{
		/*
		 * ☞ THE WHOLE VERDICT RUNS ONLY HERE, ON A DISAGREEMENT -- about
		 * 1.4% of tails -- so the prev-walk in ft_chain_head_holder and
		 * the registry scan are paid on a thousandth of the traffic, not
		 * on every chain store.
		 */
		struct cds_ft_inode_flag *hf = ft_chain_head_holder(ft, head);
		struct cds_ft_metadata *hm = hf ?
			ft_flag_to_metadata(ft, hf) : NULL;
		struct ft_flip_txn *ftxn = ft_hlist_why_ftxn;
		bool reg, led;

		/*
		 * ★ THE REGISTRY WITNESS COSTS NOTHING THE WORKLOAD CAN FEEL,
		 * and that is why it is the one kept outside -DFEATURE_FT_HOLD_TRACE.
		 * It reads the txn's own lock array, which every build already
		 * maintains; it writes nothing, takes no atomic on a trie word
		 * and changes no struct.
		 *
		 * ☠ THE LEDGER IS THE OPPOSITE ON ALL THREE COUNTS, so a finding
		 * that needs it is a finding measured on a different data
		 * structure: FEATURE_FT_HOLD_TRACE adds five fields to
		 * cds_ft_metadata (48 -> 88 bytes, +83%, so slab packing and
		 * cacheline sharing change for every node in the trie), and
		 * ft_owner_stamp_claim / _yield put an atomic xchg AND an atomic
		 * cmpxchg per acquire on the node's own metadata line -- the
		 * line peers are CASing @state on.  ⇒ read this classifier
		 * WITHOUT the ledger by default, and turn it on only to compare.
		 */
		if (!ftxn)
			uatomic_inc(&e->ftxn_none);
		else if (ft_flip_txn_handle(ftxn) != txn) {
			uatomic_inc(&e->ftxn_stale);
			ftxn = NULL;
		}
		reg = ftxn && hm && ft_flip_txn_owns(ftxn, hm);
		led = hm && ft_hold_trace_holds(hm);	/* false without the ledger */
		if (reg)
			uatomic_inc(&e->reg_own);
		if (led)
			uatomic_inc(&e->led_own);
		if (reg || led)
			uatomic_inc(&e->reg_or_led);
		/*
		 * THE MODE SPLIT BELONGS TO THE REGISTRY VERDICT, not to the
		 * stamp: "nobody owns the holder" is only owed an explanation
		 * under FINE.  A COARSE trie serialises every writer behind the
		 * FT-wide mutex and an EXCLUSIVE one has a single writer, so
		 * scoring either as a gap manufactures one out of a
		 * configuration that has no per-node locks by design.
		 */
		if (!reg) {
			if (ft->exclusive)
				uatomic_inc(&e->unreg_excl);
			else if (!ft->lock_fine)
				uatomic_inc(&e->unreg_coarse);
			else if (ft_wlock_held == ft)
				uatomic_inc(&e->unreg_wlock);
			else
				uatomic_inc(&e->unreg_fine);
		}
#ifdef FEATURE_FT_HOLD_TRACE
		{
			unsigned long owner = hm ?
				CMM_LOAD_SHARED(hm->dbg_owner_tid) : 0;

			if (!owner)
				uatomic_inc(&e->excl_none);
			else if (owner == (unsigned long) pthread_self())
				uatomic_inc(&e->excl_own);
			else
				uatomic_inc(&e->excl_foreign);
			if (reg && owner &&
					owner != (unsigned long) pthread_self())
				uatomic_inc(&e->reg_and_peer);
		}
#endif
	}
#ifdef FT_DEBUG_CHAIN_CANARY
	/*
	 * ☞ ASK THE NODE WHO PUT IT THERE.  @live is the duplicate that appeared
	 * where the caller derived NULL, and &head->next is the word it landed
	 * in -- so that SLOT's canary names the site that stored it.  A fact the
	 * table carries, not an inference from a witness that can read zero.
	 */
	{
		const struct ft_canary_ent *c = ft_chain_canary_of(
			(void **) &head->next);
		const char *pfn = c ? c->fn :
			"(no canary -- evicted, or never stored through a chain helper)";
		int pline = c ? c->line : 0;
		unsigned int k;

		for (k = 0; k < e->nr_prod; k++)
			if (e->prod_fn[k] == pfn && e->prod_line[k] == pline) {
				uatomic_inc(&e->prod_n[k]);
				return;
			}
		if (e->nr_prod >= FT_HLIST_WHY_PROD) {
			uatomic_inc(&e->prod_overflow);
			return;
		}
		k = e->nr_prod++;
		e->prod_fn[k] = pfn;
		e->prod_line[k] = pline;
		e->prod_tid[k] = c ? c->tid : 0;
		uatomic_inc(&e->prod_n[k]);
	}
#endif
	/*
	 * ☠ AND CLEAR IT.  The stamp is a THREAD-LOCAL set at the derivation and
	 * read at the record, with nothing in between to bound it: left standing,
	 * the next freeze on this thread whose @len is a LITERAL (cds_ft_remove's
	 * "1 = SOLE entry", the detach's forwarded count) inherits whatever site
	 * last called ft_hlist_chain_len and is filed under ITS name.  That is a
	 * misattribution the table cannot show, so consume the stamp here and let
	 * a genuinely unstamped site read "(unstamped)".
	 */
	ft_hlist_why_fn = NULL;
	ft_hlist_why_line = 0;
}

# define FT_HLIST_WHY_STAMP()						\
	((void) (ft_hlist_why_fn = __func__),				\
	 (void) (ft_hlist_why_line = __LINE__))
# define FT_HLIST_WHY_TAIL(ft, txn, head)	ft_hlist_why_tail((ft), (txn), (head))
/*
 * ☠ atexit IS NOT ENOUGH.  A livelock is killed by a timeout or a memcg, and
 * neither runs a destructor -- so the classification of the ONE run that
 * reproduces would be lost exactly when it is needed.  Snapshot periodically:
 * the last line in the log is the reading.
 */
static inline void ft_hlist_plan_snap(void);
# define FT_HLIST_PLAN_BAIL()			ft_hlist_plan_snap()
# define FT_HLIST_PLAN_WHY(i)			uatomic_inc(&ft_hlist_plan_why[(i)])
# define FT_HLIST_PLAN_OBSERVE(ft, txn, head, len, i)	do {		\
		uatomic_inc(&ft_hlist_plan_obs_n[(i)]);			\
		if (!ft_hlist_chain_plan_ok((ft), (txn), (head), (len)))	\
			uatomic_inc(&ft_hlist_plan_obs[(i)]);		\
	} while (0)
#else
# define FT_HLIST_WHY_STAMP()			((void) 0)
# define FT_HLIST_WHY_TAIL(ft, txn, head)	do { (void) (head); } while (0)
# define FT_HLIST_PLAN_BAIL()			do { } while (0)
# define FT_HLIST_PLAN_WHY(i)			do { (void) (i); } while (0)
# define FT_HLIST_PLAN_OBSERVE(ft, txn, head, len, i)	do { } while (0)
#endif

static inline
unsigned int ft_hlist_chain_len_at(struct cds_ft_node *head)
{
	unsigned int n = 0;

	while (head) {
		n++;
		head = ft_hlist_resolve((void *) CMM_LOAD_SHARED(head->next));
	}
	return n;
}

/*
 * The naming site is what this question is about, so stamp it: the classifier
 * above reports the DERIVATION's (fn, line), not the record's.
 */
#define ft_hlist_chain_len(head)					\
	(FT_HLIST_WHY_STAMP(), ft_hlist_chain_len_at(head))

/*
 * ft_hlist_chain_plan_ok: is the caller's @len still the truth about @head's
 * chain?  Answer it HERE, where the caller holds the chain's holder, so the
 * freeze below records a CHECKED value instead of an arbitration.
 *
 * ★ MATHIEU: "if we have proper locking, a load and check should be the same as
 * an MW record."  Exactly so, and that is the whole point of this function.
 * ft_hlist_freeze_chain_prepare's tail is the one expected-old in this file
 * that is DERIVED rather than loaded, and the reason it has to be derived is
 * not a property of the record -- it is that @len is counted BEFORE the op's
 * exclusion exists (ft_hlist_chain_len's header already forbids that: "☠ CALL
 * IT UNDER THE CHAIN HOLDER").  Close that window and the derived NULL and a
 * committed load agree by construction, the MW CAS arbitrates nobody, and the
 * record becomes parkable.
 *
 * ☠ COMMITTED, NOT urcu_txn_load: the plain load is READ-YOUR-OWN-WRITES, so in
 * a FUSED commit -- which this is, the freeze rides ft_detach_node's structural
 * txn -- it returns this txn's own pending value and never reads the word at
 * all.  That was refutation #5 of the 2026-09-17 flip attempt: the "re-read
 * under the lock" it added read nothing.
 *
 * ☠☠ AND THIS CHECK IS NOT BY ITSELF A LICENCE TO PARK.  "A load and check is
 * the same as an MW record" holds ONLY where the holder lock really excludes
 * every writer of the chain; the check cannot substitute for the exclusion, it
 * can only be exact once the exclusion is.  The engine makes that concrete:
 * urcu_txn_read spins a bounded URCU_TXN_WAIT_PATIENCE on an UNDECIDED peer
 * descriptor parked on the word and then falls back to urcu_txn_resolve, which
 * yields the PRE-COMMIT value.  So a peer whose append is in flight and does
 * not decide inside that window reads here as NULL, this check answers "plan
 * ok", and the install CAS is still the arbiter.
 *
 * That residue is exactly the exclusion gap and nothing else: a peer can only
 * have a proxy parked on this chain if it did NOT take the holder.  ⇒ the SW
 * flip stays gated on completing the lock set (the registry-unowned population
 * at _cds_ft_insert_replace, the positively peer-claimed holders, and the raw
 * producers with no audit arm), NOT on this function.  What it does buy is
 * measured and large: the appended-duplicate race drops from ~12k-15k per
 * ft_inv leg to the sliver the patience window leaves.
 *
 * ☠ VALIDATE EXACTLY WHAT THE WALK WILL RECORD, which is why the tail is
 * compared against a BARE NULL and not against ft_hlist_unmark(raw): the walk
 * records {NULL -> MARK(NULL)} there, so an already-marked tail fails that CAS
 * today.  Answering false here reproduces that outcome through the caller's
 * retry instead of through an abort -- same decision, made before any
 * structural work rather than after it.
 *
 * A SHORTER chain matters too: the freeze walk stops at NULL (`i < len &&
 * head`), so a chain that lost members since the count would silently freeze
 * fewer than planned.  Both directions are the same staleness and both answer
 * false.
 *
 * Returns true when the chain is EXACTLY @len nodes and the last one's forward
 * word is still NULL.  The caller must treat false as RETRIABLE -- the plan is
 * stale, not wrong -- and must not have made any reader-visible change yet.
 */
static inline
bool ft_hlist_chain_plan_ok(const struct cds_ft *ft, struct urcu_txn *txn,
		struct cds_ft_node *head, unsigned int len)
{
	unsigned int i;

	(void) ft;
	if (!len)
		return true;		/* nothing planned, nothing to check */
	for (i = 0; i < len; i++) {
		void *raw;

		if (!head) {
			FT_HLIST_PLAN_WHY(0);
			return false;	/* shorter than @len: stale */
		}
		raw = urcu_txn_load_committed(txn, (void **) &head->next,
				FT_HLIST_TAG);
		if (i + 1 == len) {
			/*
			 * ☠ END-OF-CHAIN MEANS WHAT THE COUNT MEANT BY IT.
			 * ft_hlist_chain_len walks with ft_hlist_resolve, which
			 * UNMARKS -- so a tail that is already logically deleted
			 * but still linked (frozen without an unlink) is counted
			 * as the last node, its word holding MARK(NULL).  A
			 * check that demanded a bare NULL here refused that
			 * shape, and the refusal is STABLE across the retry
			 * because nothing about it changes: MEASURED as a
			 * livelock in inv_concurrent_insert_replace_nolist,
			 * with the classifier reading TAIL_MARKED=8,
			 * TAIL_LIVE=0 -- i.e. every refusal was this
			 * disagreement and NOT ONE was the appended duplicate
			 * the check exists for.
			 *
			 * So agree with the count: the plan is stale only when
			 * the chain really continues.
			 *
			 * ☠ AND A MARKED TAIL IS A SEPARATE, PRE-EXISTING
			 * DEFECT, recorded here because this check is what
			 * found it.  ft_hlist_freeze_chain_prepare FORCES its
			 * tail's expected-old to NULL (`succ = NULL` at
			 * i + 1 == len), which also defeats its own
			 * already-marked skip -- so on that shape it records
			 * {NULL -> MARK(NULL)} against a word already holding
			 * MARK(NULL).  Under MW the CAS just fails and the op
			 * retries.  ☠☠ Under SW that record PARKS, and its
			 * ABORT writes old_ptr back BLIND -- UNMARKING a node a
			 * peer froze.  The flip cannot happen until the tail
			 * reads its word instead of assuming it.
			 */
			if (ft_hlist_unmark(raw) == NULL) {
				if (raw != NULL)
					FT_HLIST_PLAN_WHY(1);
				return true;
			}
			FT_HLIST_PLAN_WHY(2);
			return false;
		}
		head = ft_hlist_unmark(raw);
	}
	return false;			/* unreachable */
}

/*
 * ft_hlist_freeze_chain_prepare: freeze the chain at @head into @txn -- one
 * {v -> MARK(v)} edge for each of @len nodes -- WITHOUT committing.  The
 * whole-key counterpart of ft_hlist_freeze_sole_prepare, for the lane that
 * retires a chain ENTIRE (cds_ft_remove_all) rather than its last entry.
 *
 * For @len == 1 this is byte-identical to ft_hlist_freeze_sole_prepare: the one
 * node is the tail, and its record is {NULL -> MARK(NULL)}.
 *
 * ☠ @len IS THE CALLER'S DERIVATION, AND THE WALK MUST NOT OUTRUN IT.  That is
 * the whole arbitration, and it is the same one ft_hlist_freeze_sole_prepare
 * spells out for its derived NULL: the derivation is made with NOTHING HELD, so
 * a same-key cds_ft_insert -- concurrent with a remove in contract under
 * LOCK_FINE -- can APPEND a duplicate before this commit.  Stopping at @len
 * records the derived tail against NULL, so that append fails this commit's
 * install CAS and the whole detach ABORTS, and the caller re-derives and finds
 * the longer chain.
 *
 * RE-LOADING THE TAIL INSTEAD WOULD LOSE THE KEY.  A fresh load would find the
 * appended node, mark it into the tombstone with the rest and prune the branch
 * around it -- an insert that returned OK whose key never resolves again
 * ("key LOST after an OK concurrent insert", measured in both list modes).  So
 * @len is a BOUND, never a hint: do not re-walk to the real end.
 *
 * The successor is UNMARKED before advancing: a member already logically
 * deleted carries the mark in its own next, and following the raw value would
 * walk off by FT_HLIST_MARK into nothing.  Such a member owes no second mark.
 */
static inline
void ft_hlist_freeze_chain_prepare_at(const struct cds_ft *ft,
		struct urcu_txn *txn, struct cds_ft_node *head,
		unsigned int len, bool plan_checked)
{
	unsigned int i;

	for (i = 0; i < len && head; i++) {
		struct cds_ft_node *succ;
		struct cds_ft_node *next;
		int ret;

		/*
		 * THE LAST NODE OF THE DERIVATION IS THE APPEND POINT, and its
		 * expected-old is the derived NULL -- not a load.  Every earlier
		 * node is interior, where no append can land, so its successor
		 * is read here.
		 *
		 * ☠ THE DERIVED NULL IS AN ARBITRATION, NOT A DESCRIPTION, and
		 * it is the one expected-old in this file that is.  @len is the
		 * CALLER's, counted before this op's exclusion exists, so a
		 * same-key cds_ft_insert can APPEND a duplicate in the gap.
		 * Recorded against NULL, that append makes this commit LOSE its
		 * install CAS and the caller re-derives; re-loading instead
		 * would mark the duplicate into the tombstone and the prune
		 * would orphan it ("key LOST after an OK concurrent insert",
		 * measured in both list modes).
		 *
		 * ☞ MEASURED, per ft_inv leg: the tail disagrees with the
		 * derivation 19,196 times at per-node and 18,712 at exponential,
		 * so this arbitration is live -- and it is a REASON THE CLASS
		 * CANNOT PARK YET, because an SW record always writes.  See
		 * ft_hlist_store_chain_at's parked note.
		 */
		if (i + 1 == len) {
			next = NULL;
			/*
			 * ☠☠ THE TAIL MUST READ ITS WORD, NOT ASSUME IT -- but
			 * only once the plan has been CHECKED under the holder.
			 *
			 * Forcing @succ to NULL here also defeats this walk's own
			 * already-marked skip below, so on a tail that is frozen
			 * but still LINKED (its word holding MARK(NULL), which
			 * ft_hlist_chain_len counts as the end of the chain
			 * because ft_hlist_resolve unmarks) this records
			 * {NULL -> MARK(NULL)} against a word already holding
			 * MARK(NULL).  Under MW that CAS merely fails and the op
			 * retries -- which is why it has been invisible.  ☠ Under
			 * SW the record PARKS and its abort writes @old_ptr back
			 * BLIND, UNMARKING a node a peer froze.  MEASURED as the
			 * entire residual tail-disagreement population once the
			 * appended-duplicate race was closed: 11,425 / 10,772 /
			 * 12,722 per ft_inv leg at the three spacings.
			 *
			 * ft_hlist_chain_plan_ok has just proved unmark(word) is
			 * NULL, so this load can only answer NULL or MARK(NULL):
			 * the first records as before, the second takes the skip.
			 * ☠ WITHOUT that proof the load is the "re-loading the
			 * tail LOSES THE KEY" hazard in the header above -- it
			 * would mark a duplicate appended since the count and the
			 * prune would orphan it -- so an UNCHECKED caller keeps
			 * the derived NULL and its arbitration.
			 */
			succ = NULL;
			if (plan_checked) {
				void *raw = urcu_txn_load_committed(txn,
					(void **) &head->next, FT_HLIST_TAG);

				/*
				 * ☠☠ ONLY AN ALREADY-MARKED TAIL MAY BE READ
				 * BACK.  The plan check and this load are TWO
				 * READS, and the window between them is real --
				 * it is exactly the patience window
				 * ft_hlist_chain_plan_ok's header describes, an
				 * UNDECIDED peer proxy that reads as the
				 * pre-commit value there and as a LIVE duplicate
				 * here.  Taking that value would mark the
				 * freshly appended duplicate and let the prune
				 * orphan it: "key LOST after an OK concurrent
				 * insert", and WORSE than the derived NULL,
				 * because the CAS would then SUCCEED.  MEASURED
				 * at 16 / 15 / 32 events per ft_inv leg, so the
				 * window is small and non-empty -- the two
				 * numbers that make it a defect rather than a
				 * theory.
				 *
				 * So take the read ONLY where it can lose
				 * nothing: a MARK means the node is already
				 * frozen and the skip below is right.  Anything
				 * else keeps the derived NULL and its
				 * arbitration, which is what aborts the commit
				 * and sends the caller back to re-derive.
				 */
				/*
				 * ☠ AND THE MARK ALONE IS NOT THE CONDITION --
				 * MARK(NULL) IS.  A tail word of MARK(X) with X
				 * live means the node is already frozen AND the
				 * chain continues past @len: taking it would skip
				 * the record and let the commit proceed, freezing
				 * fewer nodes than the chain has, where the derived
				 * NULL aborts instead.  So accept exactly the
				 * already-frozen genuine tail and nothing else.
				 * With that condition the read cannot lose anything
				 * whether or not the plan was checked -- the gate
				 * above limits the blast radius, it is not what
				 * makes this safe.
				 */
				if (raw != NULL && ft_hlist_unmark(raw) == NULL)
					succ = (struct cds_ft_node *) raw;
			}
		} else {
			succ = (struct cds_ft_node *) urcu_txn_load(txn,
				(void **) &head->next, FT_HLIST_TAG);
			next = ft_hlist_unmark(succ);
		}
		if (!((uintptr_t) succ & FT_HLIST_MARK)) {
			if (i + 1 == len)
				FT_HLIST_WHY_TAIL((struct cds_ft *) ft, txn, head);
			ret = ft_hlist_store_chain(ft, txn, (void **) &head->next,
					succ, ft_hlist_set_mark(succ),
					FT_HLIST_TAG);
			assert(!ret);	/* caller reserved one edge per node */
			(void) ret;
		}
		head = next;
	}
}

/*
 * The two entry points.  The plain form keeps the derived tail NULL and the
 * arbitration that goes with it; the _checked form is for a caller that has
 * just run ft_hlist_chain_plan_ok under the chain's holder, and only that form
 * may read the tail's word.
 */
#define ft_hlist_freeze_chain_prepare(ft, txn, head, len)		\
	ft_hlist_freeze_chain_prepare_at((ft), (txn), (head), (len), false)
#define ft_hlist_freeze_chain_prepare_checked(ft, txn, head, len)	\
	ft_hlist_freeze_chain_prepare_at((ft), (txn), (head), (len), true)

#endif	/* _FT_TXN_HLIST_H */
