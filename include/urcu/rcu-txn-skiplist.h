// SPDX-FileCopyrightText: 2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-or-later

#ifndef _URCU_RCU_TXN_SKIPLIST_H
#define _URCU_RCU_TXN_SKIPLIST_H

/*
 * rcu-txn-skiplist: an ordered, concurrent-writer skiplist built on the RCU
 * MCAS engine (<urcu/rcu-txn-mcas.h>).  It is the ordered sibling of the hash-bucket
 * <urcu/rcu-txn-hlist.h>: a node is a small tower of transacted forward "next"
 * pointers, and insert/delete/move commit EVERY level of the tower in ONE MCAS,
 * so a node appears or disappears at all levels atomically.  See the design note
 * design/rcu-txn-skiplist.md in the benchmark tree for the rationale.
 *
 * Why the txn skiplist is simpler than a locking / existence skiplist
 * ------------------------------------------------------------------
 * A lock-based or "existence" skiplist links a tower one level at a time, so a
 * node is transiently half-inserted; coordinating that window is what forces the
 * classic machinery -- a per-node spinlock (writer exclusion), a seqlock the
 * readers spin on to detect a half-linked node, and a separate "deleted" word.
 * MCAS commits all level pointers at once, so the half-linked window never
 * exists.  That deletes the spinlock AND the seqlock outright, and folds the
 * "deleted" flag from a word into a single MARK bit in the next pointer -- the
 * same tombstone <urcu/rcu-txn-hlist.h> carries.  What is left is: a node is an
 * array of MARK-able "next" slots, and each operation is the hlist edge-recording
 * pattern applied once per level in a single commit.
 *
 * Forward-only: no back pointer (contrast with the hlist's pprev)
 * --------------------------------------------------------------
 * A node carries only forward "next" pointers, no per-level "prev".  Deletion
 * therefore re-derives the per-level predecessors by a top-down search from the
 * head (O(log n) expected), exactly as a textbook skiplist does.  This is the
 * OPPOSITE choice from <urcu/rcu-txn-hlist.h>, which keeps a pprev so del(elem)
 * needs no search -- and the difference is deliberate: pprev costs the hlist one
 * extra pointer and one extra commit edge because the hlist is single-level,
 * whereas per-level back pointers would DOUBLE both a skiplist node's footprint
 * AND (worse, in the MCAS world) the number of transacted slots every insert and
 * delete must commit, merely to avoid a cheap read-only descent.  The scarce
 * resource here is commit width -- the slots a descriptor holds, that conflict,
 * that a peer's load must wait on -- so the skiplist inherits the hlist's MARK
 * but not its pprev; the search replaces it.
 *
 * Node / mark layout
 * ------------------
 *   Node: { unsigned int toplevel;                      (highest level index)
 *           struct urcu_txn_skiplist_node *next[toplevel + 1]; }  (flexible;
 *                                                 transacted; each MARK-able)
 * The node is embedded LAST in the caller's element (the flexible array runs
 * past it); allocate sizeof(element) + (toplevel + 1) * sizeof(void *).  Every
 * slot -- a node's next[] and the head's next[] -- is transacted under
 * URCU_TXN_SKIPLIST_TAG (the engine proxy tag, default URCU_TXN_TAG / bit 0,
 * overridable before include like the hlist tag).  A "next" value carries an
 * optional deletion MARK on bit 1 (URCU_TXN_SKIPLIST_MARK), meaning "this node
 * is logically deleted"; readers strip it per hop (urcu_txn_skiplist_resolve).
 * URCU_TXN_SKIPLIST_TAG must satisfy (value & TAG) != TAG for every live value a
 * slot holds (a possibly-MARK-ed node pointer, or NULL) -- holds for bit-0 TAG.
 *
 * Why the tombstone is required (and why it is per level)
 * ------------------------------------------------------
 * Identical in spirit to <urcu/rcu-txn-hlist.h>.  Two things detect a racing
 * deletion; only ONE is the mark:
 *
 *  (1) A NEIGHBOUR moved -- a structural slot conflict, NOT the mark.  Every
 *      adjacency at level L is named by the single slot pred[L]->next[L].  An
 *      insert between pred[L] and its successor, a delete of pred[L], and a
 *      delete of that successor all CAS THAT SAME slot with the same expected
 *      old value, so the MCAS commits at most one; the losers fail their
 *      old-value check, abort, re-search and proceed.  Unlike the hlist there is
 *      no backward "pprev" edge, so an insert touches only pred[L]->next[L] (no
 *      successor-side load-validate): every race at a level funnels through that
 *      one shared slot.
 *
 *  (2) The node used AS A PREDECESSOR was deleted -- the mark's one and only job.
 *      An insert whose pred[L] is @N, racing del(@N), would CAS @N->next[L]
 *      (its pred slot) while del(@N) CASes @N's OWN predecessor -- disjoint
 *      slots, so absent a tombstone both commit and the inserted node is
 *      orphaned (nothing points to it once @N is unlinked).  del(@N) therefore
 *      MARKs @N->next[L] as part of the same commit, turning the insert's pred
 *      slot into a shared slot: the insert's old-value check now fails and it
 *      retries against a fresh search.  Because an insert may use @N as its
 *      predecessor at ANY level up to @N->toplevel, del MARKs @N->next[L] at
 *      EVERY level @N occupies -- a level-0-only mark would leave the higher
 *      levels' inserts unserialized.
 *
 * Operations (each one MCAS commit)
 * ---------------------------------
 *   insert(new, key)  -- new->toplevel levels, each: pred[L]->next[L]: succ -> new
 *                        (and new->next[L] = succ, built invisibly).  1 visible
 *                        edge per level.
 *   del(key)          -- node's toplevel+1 levels, each: MARK node->next[L], and
 *                        pred[L]->next[L]: node -> node->next[L].  2 edges/level.
 *   move              -- del(key) in list A composed with insert(new, key) in
 *                        list B in ONE txn (see the _prepare forms).  The
 *                        commit is a single state transition, so no COMMITTED
 *                        state ever has the key in both A and B, or in
 *                        neither.  A reader is not a transaction, though: one
 *                        that walks A and then B can straddle the commit and
 *                        find the key in neither.  See the consistency model
 *                        in <urcu/rcu-txn.h>.
 *
 * Reclaim, read/write contract, escalation
 * ----------------------------------------
 * As in <urcu/rcu-txn-hlist.h>: del() reports whether THIS call removed the node
 * (so two concurrent deletes cannot double-free); reclaim it after a grace
 * period.  Mutators commit through <urcu/rcu-txn.h>, which opens an RCU read-side
 * section per attempt and defers descriptor reclaim through the flavor's
 * call_rcu; include this header AFTER an RCU flavor.  A self-contained mutator
 * loops internally until it commits or definitively fails.  The escalation
 * DOMAIN is taken explicitly (not embedded in the head), so several skiplists may
 * share one domain.  Readers descend forward within an RCU read-side section
 * through the accessors below (which resolve the proxy and strip the mark);
 * never touch the raw fields.
 *
 * Composing several prepares that touch the SAME skiplist
 * ---------------------------------------------------------
 * The _prepare forms search THROUGH the txn (urcu_txn_skiplist_search takes it),
 * so under the default read-your-own-writes the descent observes the txn's own
 * pending edits: each prepare lands on its true post-batch predecessor -- often a
 * node this txn allocated and has not published, whose slots need no record at
 * all -- and where a published slot genuinely takes two edits, the chained store
 * fuses them into one record.  Composing several prepares on ONE skiplist in ONE
 * txn is therefore supported.
 *
 * Were the buffered writes invisible instead (the retired pre-RYW mode), a
 * second prepare on the same skiplist would search the COMMITTED structure,
 * blind to the first's pending edits.  It could pick a predecessor the first
 * prepare has already displaced, and the two stores would then collide on that
 * one pred->next[L] slot: both presenting the same old, the one-record-per-slot
 * upgrade would overwrite new_ptr and silently destroy an edge -- a torn tower,
 * not merely a lost key.  Read-your-own-writes is what makes the on-one-skiplist
 * composition above sound.
 *
 * One caveat on the FIRST attempt.  A default handle's age-0 attempt runs the
 * stripped read-your-own-writes path: it maintains the Bloom filter but never
 * consults the write set.  A same-skiplist batch, whose prepares alias by
 * construction, therefore flags the coincidence and ABORTS by design,
 * re-running at age 1+ where find resolves it.  That first attempt is
 * guaranteed wasted -- urcu_txn_expect_conflict() (<urcu/rcu-txn.h>) skips it,
 * and for a same-skiplist batch it is the right default, not a tuning knob.
 *
 * The abort is a complete safety net only for what the batch PUBLISHES.  A
 * prepare's -EEXIST / -ENOENT is a verdict the caller acts on BEFORE commit, so
 * the net would never fire: at age 0 the descent reads committed values, and a
 * composed del(k)+insert(k) would report -EEXIST for the key it just deleted --
 * deterministically, and reproducibly on a fresh handle.  Both prepares
 * therefore report -EAGAIN instead whenever urcu_txn_tainted() says the attempt
 * read its own writes blind.  Callers already treat -EAGAIN as retry; nothing
 * else changes.
 *
 * Declaring the write set disjoint
 * --------------------------------
 * A one-way MOVE composed across DISTINCT skiplists -- del(key) from A, insert
 * of the same key into B -- touches disjoint slots by construction: every slot
 * it records lives in A or in B, and no slot is in both.  Such a handle may
 * declare that with urcu_txn_declare_disjoint() (see <urcu/rcu-txn.h>) and skip
 * the per-store reconcile find.
 *
 * A ROTATION IS NOT SUCH A MOVE.  Rotating k1: A->B, k2: B->C, k3: C->A hands
 * EVERY list both a del and an insert, of different keys -- and when the
 * incoming key lands adjacent to the outgoing node, the two prepares coincide
 * on one pred->next[L] slot.  That is data-dependent; the keys alone do not
 * tell you.  Declaring disjoint there suppresses the RYW consult at EVERY age,
 * so the insert's descent still sees the deleted node linked, picks it as
 * pred/succ, and records a slot the del already recorded.  At age 0 the
 * duplicate blind-appends and the commit tears the tower.  At age 1+ the
 * reconcile sees a disagreeing old (the del's pending MARK) and poisons the
 * attempt -- and a disjoint handle stays RYW-blind on every retry, so it
 * poisons again, forever.  Past URCU_TXN_FALLBACK that livelock holds the
 * domain's fair-mutex lane and stalls every writer in the domain.
 *
 * So declare disjoint only for a move that is a SINGLE op per list.  For a
 * rotation, a batch, or a range move, keep the default (RYW) handle -- and
 * since those alias by construction rather than by luck, add
 * urcu_txn_expect_conflict() so the guaranteed-doomed age-0 attempt is never
 * spent.
 */

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include <urcu/compiler.h>
#include <urcu/uatomic.h>
#include <urcu/call-rcu.h>
#include <urcu/rcu-txn-mcas.h>
#include <urcu/rcu-txn.h>
#include <urcu-pointer.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Maximum tower height (levels 0 .. URCU_TXN_SKIPLIST_MAX_LEVELS - 1). */
#ifndef URCU_TXN_SKIPLIST_MAX_LEVELS
#define URCU_TXN_SKIPLIST_MAX_LEVELS	8
#endif

/* Engine proxy tag for every skiplist slot (head and node next[]).  Override
 * before include to drive the chain under an embedder's own tag (see the hlist
 * header).  Must satisfy (value & TAG) != TAG for every live value a slot holds. */
#ifndef URCU_TXN_SKIPLIST_TAG
#define URCU_TXN_SKIPLIST_TAG		URCU_TXN_TAG
#endif

/* Logical-deletion mark: bit 1 of a node's next pointer (matches
 * <urcu/rcu-txn-hlist.h>; override before include, must be free in every live
 * "next" value). */
#ifndef URCU_TXN_SKIPLIST_MARK
#define URCU_TXN_SKIPLIST_MARK		2UL
#endif

struct urcu_txn_skiplist_node {
	unsigned int toplevel;			/* highest level index; next[0..toplevel] */
	/* transacted, MARK-able forward pointers; flexible array [toplevel + 1] */
	struct urcu_txn_skiplist_node *next[];
};

/*
 * A skiplist is a comparison callback plus a head node whose tower spans all
 * URCU_TXN_SKIPLIST_MAX_LEVELS (so the descent starts uniformly from it, with no
 * head special case -- the head's next[] slots are ordinary "next" slots).  cmp
 * returns <0 / 0 / >0 for node's key <, ==, > @key; it is never called on the
 * head (the descent only compares head->next[L], which are real keyed nodes).
 */
struct urcu_txn_skiplist {
	int (*cmp)(struct urcu_txn_skiplist_node *node, void *key);
	struct urcu_txn_skiplist_node *head;
};

static inline
void *urcu_txn_skiplist_set_mark(struct urcu_txn_skiplist_node *n)
{
	return (void *) ((uintptr_t) n | URCU_TXN_SKIPLIST_MARK);
}

static inline
int urcu_txn_skiplist_is_marked(void *v)
{
	return (int) ((uintptr_t) v & URCU_TXN_SKIPLIST_MARK);
}

/*
 * A MASK, not the subtraction the engine's proxy untag uses: no caller has
 * proven the mark set here.  See urcu_txn_list_unmark() for the full rationale.
 */
static inline
struct urcu_txn_skiplist_node *urcu_txn_skiplist_unmark(void *v)
{
	return (struct urcu_txn_skiplist_node *)
			((uintptr_t) v & ~(uintptr_t) URCU_TXN_SKIPLIST_MARK);
}

/*
 * Resolve a raw "next" slot value: strip the engine proxy, then the mark.  Fast
 * path -- a clean value (neither a proxy under URCU_TXN_SKIPLIST_TAG nor MARK-ed)
 * is returned untouched.  Mirrors urcu_txn_hlist_resolve.
 */
static inline
struct urcu_txn_skiplist_node *urcu_txn_skiplist_resolve(void *raw)
{
	uintptr_t v = (uintptr_t) raw;

	if (caa_unlikely(v & (URCU_TXN_SKIPLIST_TAG | URCU_TXN_SKIPLIST_MARK)))
		return urcu_txn_skiplist_unmark(
				urcu_txn_resolve(raw, URCU_TXN_SKIPLIST_TAG));
	return (struct urcu_txn_skiplist_node *) raw;
}

/* Resolved forward step at @level (call within an RCU read-side section). */
static inline
struct urcu_txn_skiplist_node *urcu_txn_skiplist_next_rcu(
		struct urcu_txn_skiplist_node *node, unsigned int level)
{
	return urcu_txn_skiplist_resolve(
			(void *) rcu_dereference(node->next[level]));
}

/*
 * Pick a tower height (0 .. MAX-1) from a caller-supplied random word: an
 * exponential, power-of-two decrease with level (probability 2^-(level+1)),
 * truncated to the maximum.  The primitive stays PRNG-free -- the caller feeds a
 * random() word -- so a benchmark can control the distribution.
 */
static inline
unsigned int urcu_txn_skiplist_random_level(unsigned long r)
{
	unsigned int level = 0;

	while ((r & 0x1UL) && level < URCU_TXN_SKIPLIST_MAX_LEVELS - 1) {
		level++;
		r >>= 1;
	}
	return level;
}

/*
 * Initialize @node's tower to @toplevel levels, all next NULL.  Call before
 * handing @node to an insert.  The caller must have allocated toplevel + 1
 * trailing next[] slots.
 */
static inline
void urcu_txn_skiplist_node_init(struct urcu_txn_skiplist_node *node,
		unsigned int toplevel)
{
	unsigned int i;

	node->toplevel = toplevel;
	for (i = 0; i <= toplevel; i++)
		node->next[i] = NULL;
}

/*
 * Initialize an empty skiplist: allocate a full-height head node.  Returns 0, or
 * -ENOMEM.  Pair with urcu_txn_skiplist_destroy().
 */
static inline
int urcu_txn_skiplist_init(struct urcu_txn_skiplist *sl,
		int (*cmp)(struct urcu_txn_skiplist_node *node, void *key))
{
	sl->cmp = cmp;
	sl->head = (struct urcu_txn_skiplist_node *) malloc(sizeof(*sl->head)
			+ URCU_TXN_SKIPLIST_MAX_LEVELS * sizeof(sl->head->next[0]));
	if (sl->head == NULL)
		return -ENOMEM;
	urcu_txn_skiplist_node_init(sl->head, URCU_TXN_SKIPLIST_MAX_LEVELS - 1);
	return 0;
}

/* Free the head node.  The skiplist must be empty and quiescent. */
static inline
void urcu_txn_skiplist_destroy(struct urcu_txn_skiplist *sl)
{
	free(sl->head);
	sl->head = NULL;
}

/*
 * Whether @sl holds no key.  A reader accessor like the rest: call within an
 * RCU read-side section -- the level-0 slot it reads may hold a proxy, and
 * resolving one dereferences a descriptor that a grace period would otherwise
 * reclaim.
 */
static inline
int urcu_txn_skiplist_empty(struct urcu_txn_skiplist *sl)
{
	return urcu_txn_skiplist_next_rcu(sl->head, 0) == NULL;
}

/*
 * Read policy (measured; see also rcu-txn-bitmap.h and rcu-txn-hlist.h).
 *
 * Wait iff the loaded slot belongs to this transaction's own read/write set.
 * There, a stale value dooms the install-time CAS -- drive_install aborts on
 * v != r->old_ptr -- so waiting for the UNDECIDED owner to reach a terminal
 * status buys a value the plant can actually land on.  The _prepare loads below
 * are all such slots (&pred->next[L] and &node->next[L] are stored;
 * &succ->next[L] is folded into the read set), so they use the waiting
 * urcu_txn_load/_validate.  (Waiting, never driving: the owner is the sole
 * driver of its own install -- see <urcu/rcu-txn-mcas.h>.)
 *
 * Read optimistically for NAVIGATION -- a slot this transaction will never store
 * nor validate.  The descent below is pure navigation: an UNDECIDED transaction
 * has not linearized, so the slot's logical value already IS its old_ptr, and
 * waiting would make every hop of an O(log n) descent block on a stranger's
 * install.  Switching the descent alone was worth 3.05x at 192 writers; making
 * the _prepare loads optimistic as well cost 8% back.
 */
/*
 * Resolved forward step at @level taken THROUGH a transaction: identical to
 * urcu_txn_skiplist_next_rcu() except that the hop observes the transaction's
 * own buffered stores (read-your-own-writes, the engine's default).  That is
 * what lets a later edit in a batch traverse the structure as the commit will
 * leave it rather than as it is, so it computes its write site against the
 * batch's pending edits.  On a handle that declared its write set DISJOINT --
 * which by contract never reads its own writes -- this degenerates to the plain
 * resolved read.  Call within the txn's RCU read-side section.
 */
static inline
struct urcu_txn_skiplist_node *urcu_txn_skiplist_next_txn(
		struct urcu_txn *txn,
		struct urcu_txn_skiplist_node *node, unsigned int level)
{
	return urcu_txn_skiplist_resolve(urcu_txn_load_optimistic(txn,
			(void **) &node->next[level], URCU_TXN_SKIPLIST_TAG));
}

/*
 * Descend from the head, recording in @update[L] the predecessor node at each
 * level (the last node whose key < @key, or the head) and, if @succ is non-NULL,
 * in @succ[L] that level's successor -- the first node with key >= @key, or NULL.
 * Returns the level-0 successor (the first node with key >= @key overall).
 *
 * The (pred, succ) pair search validates at each level -- pred.key < @key <=
 * succ.key -- is exactly what a mutator must transact against: the caller uses
 * @succ[L] as its store's old value, so the commit's old-value check re-proves
 * pred and succ are still consecutive (nobody spliced in between) while the
 * key ordering is inherited from this search.  Plain RCU-resolved loads: a
 * predecessor/successor made stale by a racing update just fails that check and
 * the caller retries.  Call within an RCU read-side section.
 *
 * The descent hops through @txn, so it sees the transaction's OWN pending edits
 * (read-your-own-writes, the default): a node this transaction already deleted is
 * already unlinked in that view and can never be picked as a predecessor, and a
 * node it already inserted is picked when it is the true post-batch predecessor.
 * The guards in insert_prepare/del_prepare must read through the same view --
 * they do, via urcu_txn_load -- because a MIXED view (RYW descent, committed-value
 * guard) disagrees with itself and would fail every attempt.
 */
static inline
struct urcu_txn_skiplist_node *urcu_txn_skiplist_search(
		struct urcu_txn *txn,
		struct urcu_txn_skiplist *sl, void *key,
		struct urcu_txn_skiplist_node **update,
		struct urcu_txn_skiplist_node **succ)
{
	struct urcu_txn_skiplist_node *pred = sl->head;
	struct urcu_txn_skiplist_node *cur = NULL;
	int level;

	for (level = (int) sl->head->toplevel; level >= 0; level--) {
		cur = urcu_txn_skiplist_next_txn(txn, pred, (unsigned int) level);
		while (cur != NULL && sl->cmp(cur, key) < 0) {
			pred = cur;
			cur = urcu_txn_skiplist_next_txn(txn, pred,
					(unsigned int) level);
		}
		update[level] = pred;
		if (succ != NULL)
			succ[level] = cur;
	}
	return cur;		/* == succ[0]: first node with key >= @key */
}

/*
 * Look up @key.  Returns the node with that key, or NULL.  Call within an RCU
 * read-side section.  (A node concurrently being deleted may still be returned
 * -- an RCU-legal race; the resolve strips its mark.)
 */
static inline
struct urcu_txn_skiplist_node *urcu_txn_skiplist_lookup_rcu(
		struct urcu_txn_skiplist *sl, void *key)
{
	struct urcu_txn_skiplist_node *pred = sl->head;
	struct urcu_txn_skiplist_node *cur;
	int level;

	for (level = (int) sl->head->toplevel; level >= 0; level--) {
		cur = urcu_txn_skiplist_next_rcu(pred, (unsigned int) level);
		while (cur != NULL && sl->cmp(cur, key) < 0) {
			pred = cur;
			cur = urcu_txn_skiplist_next_rcu(pred, (unsigned int) level);
		}
	}
	cur = urcu_txn_skiplist_next_rcu(pred, 0);
	if (cur != NULL && sl->cmp(cur, key) == 0)
		return cur;
	return NULL;
}

/*
 * urcu_txn_skiplist_insert_prepare: record inserting @newp (whose tower height
 * newp->toplevel and trailing next[] the caller has already sized/initialized)
 * under @key into @sl, WITHOUT committing.  Composable form.  Returns 0,
 * -EEXIST if @key is already present, or -EAGAIN if a predecessor is
 * mid-deletion (retry).  OOM is sticky to the commit.
 */
static inline
int urcu_txn_skiplist_insert_prepare(struct urcu_txn *txn,
		struct urcu_txn_skiplist *sl,
		struct urcu_txn_skiplist_node *newp, void *key)
{
	struct urcu_txn_skiplist_node *update[URCU_TXN_SKIPLIST_MAX_LEVELS];
	struct urcu_txn_skiplist_node *ssucc[URCU_TXN_SKIPLIST_MAX_LEVELS];
	struct urcu_txn_skiplist_node *cand;
	unsigned int level, top = newp->toplevel;

	/*
	 * @newp->toplevel is the CALLER's (urcu_txn_skiplist_random_level()
	 * clamps, but a hand-built tower need not have come from it).  An
	 * over-tall one indexes update[]/ssucc[] past their end -- reading
	 * whatever the stack holds and transacting the wild addresses it finds
	 * there.  Debug builds only.
	 */
	urcu_assert_debug(top < URCU_TXN_SKIPLIST_MAX_LEVELS);
	cand = urcu_txn_skiplist_search(txn, sl, key, update, ssucc);
	if (cand != NULL && sl->cmp(cand, key) == 0) {
		/*
		 * -EEXIST is a TERMINAL verdict the caller acts on, so it must
		 * not be derived from a stale view.  On an age-0 attempt the
		 * search's loads never consult the write set: a composed
		 * del(k)+insert(k) descends exactly the slots the del recorded,
		 * flags every coincidence -- and still returns the COMMITTED
		 * values, so the node this transaction just deleted reads as
		 * present.  The caller ends the bracket on -EEXIST, commit never
		 * runs, and the esc_pending net never fires; a fresh handle
		 * re-runs at age 0 and reproduces it.  Report a retry instead:
		 * at age 1+ the write set is consulted exactly.
		 */
		if (caa_unlikely(urcu_txn_tainted(txn)))
			return -EAGAIN;
		return -EEXIST;
	}
	/*
	 * Splice newp between the (pred, succ) pair search validated at each level:
	 * pred[L].key < @key < succ[L].key (succ.key is strictly > @key since @key
	 * is absent).  The store {pred[L]->next[L]: pv -> newp} carries old value
	 * @pv, which must still resolve to @succ -- so the commit re-proves pred and
	 * succ are consecutive (no node spliced in between, pred not repointed) at
	 * the install point, while the key ordering is inherited from search and
	 * needs no runtime re-check.
	 */
	for (level = 0; level <= top; level++) {
		struct urcu_txn_skiplist_node *pred = update[level];
		struct urcu_txn_skiplist_node *succ = ssucc[level];
		void *pv = urcu_txn_load(txn,
				(void **) &pred->next[level],
				URCU_TXN_SKIPLIST_TAG);

		/*
		 * @pred must be alive (its own next not tombstoned) and still adjacent
		 * to @succ.  is_marked(pv) reports pred's deletion (the mark lives on a
		 * node's own forward pointer); resolve(pv) != succ means a racing
		 * insert/delete moved the edge -- re-search either way.
		 */
		if (urcu_txn_skiplist_is_marked(pv)
				|| urcu_txn_skiplist_resolve(pv) != succ)
			return -EAGAIN;
		/*
		 * @succ must not be logically deleted: its mark lives on succ->next[L],
		 * not on @pv, so the adjacency check above cannot see it.  Fold succ's
		 * tombstone state into the commit's conflict set with a load-validate
		 * guard {snv -> snv} -- if @succ is (or becomes, before our install
		 * point) deleted, the commit aborts and we re-search.
		 */
		if (succ != NULL) {
			void *snv = urcu_txn_load_validate(txn,
					(void **) &succ->next[level],
					URCU_TXN_SKIPLIST_TAG);

			if (urcu_txn_skiplist_is_marked(snv))
				return -EAGAIN;
		}
		newp->next[level] = succ;
		urcu_txn_store_mw(txn, (void **) &pred->next[level], pv, newp,
				URCU_TXN_SKIPLIST_TAG);
	}
	return 0;
}

/*
 * urcu_txn_skiplist_del_prepare: record removing the node with @key from @sl,
 * WITHOUT committing.  On a committed OK, THIS call removed it and *removed is
 * set to the node (reclaim it after a grace period).  Composable form.  Returns
 * 0 (recorded; *removed set), -ENOENT if @key is not present or already being
 * deleted (*removed = NULL; do NOT reclaim), or -EAGAIN (retry).  OOM is sticky
 * to the commit.
 */
static inline
int urcu_txn_skiplist_del_prepare(struct urcu_txn *txn,
		struct urcu_txn_skiplist *sl, void *key,
		struct urcu_txn_skiplist_node **removed)
{
	struct urcu_txn_skiplist_node *update[URCU_TXN_SKIPLIST_MAX_LEVELS];
	struct urcu_txn_skiplist_node *node, *succ;
	unsigned int level, top;

	*removed = NULL;
	/*
	 * Delete's successor is the victim's OWN forward pointer (node->next[L],
	 * loaded below), not search's per-level successor (which for the victim is
	 * the victim itself, since victim.key == @key) -- so pass NULL for @succ.
	 */
	node = urcu_txn_skiplist_search(txn, sl, key, update, NULL);
	if (node == NULL || sl->cmp(node, key) != 0) {
		/* stale-view verdict: see insert_prepare's -EEXIST */
		if (caa_unlikely(urcu_txn_tainted(txn)))
			return -EAGAIN;
		return -ENOENT;			/* not present */
	}
	top = node->toplevel;
	/*
	 * Each level the node occupies: MARK node->next[L] (logical delete + the
	 * shared-slot conflict that catches an insert using @node as pred[L]), and
	 * unlink update[L]->next[L] : node -> node's successor.  Read node->next[L]
	 * fresh via the txn; an already-MARK-ed next[0] means a peer beat us.  The
	 * unlink's old value is @node: if a racing update changed update[L]->next[L]
	 * (neighbour moved / predecessor deleted) the commit's old-value check fails
	 * and the caller re-searches.
	 */
	for (level = 0; level <= top; level++) {
		void *nv = urcu_txn_load(txn,
				(void **) &node->next[level],
				URCU_TXN_SKIPLIST_TAG);
		void *pv = urcu_txn_load(txn,
				(void **) &update[level]->next[level],
				URCU_TXN_SKIPLIST_TAG);

		/* @node itself is already being deleted (its next is tombstoned). */
		if (urcu_txn_skiplist_is_marked(nv))
			return level == 0 ? -ENOENT : -EAGAIN;
		/*
		 * Validate the predecessor before betting on it.  search() resolves
		 * marks, so update[level] may be a logically-deleted node, or a racing
		 * insert/delete may have moved it off @node.  A stale/tombstoned
		 * predecessor whose slot happens to still read @node would splice a
		 * dead node's region (orphaning a live node); one whose slot is MARK-ed
		 * would abort on every retry against the same resolved-through search.
		 * Bet on the loaded value @pv (which must resolve to @node and be
		 * unmarked), mirroring insert_prepare's is_marked(sv) guard.
		 */
		if (urcu_txn_skiplist_is_marked(pv) ||
				urcu_txn_skiplist_resolve(pv) != node)
			return -EAGAIN;
		/*
		 * The new successor must not be logically deleted either: we are about
		 * to make update[level] point at @succ = resolve(nv).  is_marked(nv)
		 * reports @node's state, not @succ's (a node's mark lives on its OWN
		 * forward pointer), so @succ may be a tombstoned node still physically
		 * linked -- splicing update[level] -> @succ(dead) would orphan it.  Fold
		 * @succ's tombstone state into the commit's conflict set, mirroring
		 * insert_prepare's successor load-validate guard.
		 */
		succ = urcu_txn_skiplist_resolve(nv);
		if (succ != NULL) {
			void *snv = urcu_txn_load_validate(txn,
					(void **) &succ->next[level],
					URCU_TXN_SKIPLIST_TAG);

			if (urcu_txn_skiplist_is_marked(snv))
				return -EAGAIN;
		}
		urcu_txn_store_mw(txn, (void **) &node->next[level], nv,
				urcu_txn_skiplist_set_mark(
					(struct urcu_txn_skiplist_node *) nv),
				URCU_TXN_SKIPLIST_TAG);
		urcu_txn_store_mw(txn, (void **) &update[level]->next[level], pv,
				(struct urcu_txn_skiplist_node *) nv,
				URCU_TXN_SKIPLIST_TAG);
	}
	*removed = node;
	return 0;
}

/*
 * Insert @newp under @key into @sl, transacting through @domain.  Returns 0,
 * -EEXIST if @key is already present, or -ENOMEM on descriptor OOM.
 * Self-contained bracket around urcu_txn_skiplist_insert_prepare().
 */
static inline
int urcu_txn_skiplist_add_rcu(struct urcu_txn_skiplist *sl,
		struct urcu_txn_skiplist_node *newp, void *key,
		struct urcu_txn_domain *domain)
{
	struct urcu_txn txn;
	int ret, prep;

	urcu_txn_init(&txn, domain);
	urcu_txn_declare_disjoint(&txn);	/* single-op commit: distinct per-level slots, no same-slot WAW */
	for (;;) {
		urcu_txn_begin(&txn);
		prep = urcu_txn_skiplist_insert_prepare(&txn, sl, newp, key);
		if (prep == -EAGAIN) {			/* predecessor moved: retry */
			urcu_txn_conflict(&txn);
			urcu_txn_end(&txn);
			continue;
		}
		if (prep) {				/* -EEXIST */
			urcu_txn_end(&txn);
			return prep;
		}
		ret = urcu_txn_commit(&txn);
		urcu_txn_end(&txn);
		if (ret != URCU_TXN_STATUS_ABORT)
			break;
	}
	return ret < 0 ? -ENOMEM : 0;
}

/*
 * Remove the node with @key from @sl, transacting through @domain.  Returns 1 if
 * THIS call removed it (and, if @removed is non-NULL, stores the node there for
 * reclaim after a grace period), 0 if @key was not present, or -ENOMEM on
 * descriptor OOM.  Self-contained bracket around urcu_txn_skiplist_del_prepare().
 */
static inline
int urcu_txn_skiplist_del_rcu(struct urcu_txn_skiplist *sl, void *key,
		struct urcu_txn_domain *domain,
		struct urcu_txn_skiplist_node **removed)
{
	struct urcu_txn_skiplist_node *node;
	struct urcu_txn txn;
	int ret, prep;

	if (removed != NULL)
		*removed = NULL;
	urcu_txn_init(&txn, domain);
	urcu_txn_declare_disjoint(&txn);	/* single-op commit: distinct per-level slots, no same-slot WAW */
	for (;;) {
		urcu_txn_begin(&txn);
		prep = urcu_txn_skiplist_del_prepare(&txn, sl, key, &node);
		if (prep == -EAGAIN) {			/* neighbour moved: retry */
			urcu_txn_conflict(&txn);
			urcu_txn_end(&txn);
			continue;
		}
		if (prep) {				/* -ENOENT: not present */
			urcu_txn_end(&txn);
			return 0;
		}
		ret = urcu_txn_commit(&txn);
		urcu_txn_end(&txn);
		if (ret != URCU_TXN_STATUS_ABORT)
			break;
	}
	if (ret < 0)
		return -ENOMEM;
	if (removed != NULL)
		*removed = node;
	return 1;
}

#define urcu_txn_skiplist_entry(ptr, type, member) \
	caa_container_of(ptr, type, member)

/* container_of that maps a NULL terminator to a NULL entry. */
#define urcu_txn_skiplist_entry_safe(ptr, type, member) \
	__extension__ ({ __typeof__(ptr) ___ptr = (ptr); \
		___ptr ? urcu_txn_skiplist_entry(___ptr, type, member) : NULL; })

/* Iterate the level-0 chain in key order (within an RCU read-side section). */
#define urcu_txn_skiplist_for_each_rcu(pos, sl) \
	for (pos = urcu_txn_skiplist_next_rcu((sl)->head, 0); \
		(pos) != NULL; \
		pos = urcu_txn_skiplist_next_rcu(pos, 0))

#define urcu_txn_skiplist_for_each_entry_rcu(pos, sl, member) \
	for (pos = urcu_txn_skiplist_entry_safe( \
			urcu_txn_skiplist_next_rcu((sl)->head, 0), \
			__typeof__(*(pos)), member); \
		(pos) != NULL; \
		pos = urcu_txn_skiplist_entry_safe( \
			urcu_txn_skiplist_next_rcu(&(pos)->member, 0), \
			__typeof__(*(pos)), member))

#ifdef __cplusplus
}
#endif

#endif	/* _URCU_RCU_TXN_SKIPLIST_H */
