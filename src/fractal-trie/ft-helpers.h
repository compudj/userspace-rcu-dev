#if defined(FT_DEBUG_DEL_TOMB) || defined(FT_DEBUG_PAIR_STORE)
#include <execinfo.h>
#endif
// SPDX-FileCopyrightText: 2012-2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-only

/*
 * src/fractal-trie/ft-helpers.h
 *
 * Userspace RCU library - Fractal Trie: general helpers: tag/node/metadata accessors, key conversion + compare, publish, flip-latch, ordinal-cell and skip-compressed primitives, allocation glue.
 *
 * Implementation unit: #included once by fractal-trie.c, in dependency
 * order, into a single translation unit (preserves cross-module inlining).
 * Not a standalone header.
 */
#ifndef FRACTAL_TRIE_IMPL
#error "ft-helpers.h is an implementation unit; #include it from fractal-trie.c only"
#endif

/*
 * ★ WHAT MAKES THIS WRITE LEGAL?  Every writer of a shared word in this trie is
 * covered by exactly one of three regimes, and which one is a FACT ABOUT THE
 * SITE that the code has until now carried only in prose.  Naming it in the
 * call makes the answer local and checkable instead of a comment three
 * functions away:
 *
 *   HIDDEN  the target is build-invisible -- freshly allocated by THIS op and
 *           not yet published, so no reader and no peer can reach it.  A plain
 *           store is legal because there is nobody to race.  ("An interior
 *           write into an unpublished body: nothing to record.")
 *   MW_CAS  the word is arbitrated by the engine: the write is a transacted
 *           record whose expected-old makes a concurrent change lose.  No lock
 *           is held and none is needed.
 *   LOCKED  the op HOLDS the word's owner -- a per-node lock it acquired, or
 *           the FT-wide writer lock -- and peers are excluded up front.
 *
 * ☞ THE DECLARATION IS TESTABLE, which is the point.  The audit does not
 * believe it: a site that declares LOCKED is put through the witness ladder and
 * a miss is a REAL violation (the code says it holds the owner and it does
 * not), while HIDDEN is bucketed as owing nothing.  So the annotation
 * documents the design AND fails loudly when the design drifts away from it --
 * the pattern this tree already uses when it writes an invariant as a rule you
 * can run.
 *
 * ☠ THE WORD CANNOT ANSWER THIS, ONLY THE CALLER CAN.  The tempting local test
 * -- compare what the store is about to overwrite -- is wrong twice: the store
 * is a bare rcu_assign_pointer, so a read before it is not atomic with it and a
 * peer may change the word in between; and a fresh node's parent word is NOT
 * reliably NULL (a cluster wires fresh-to-fresh, and recompact's reparent loop
 * re-wires a node more than once), so "overwrote something different" does not
 * mean "live".  At the sites that matter the caller already branches on exactly
 * this fact (ft_node_recompact tests @pending_del_replace_fresh;
 * ft_split_compressed_insert stores the fresh top eagerly and PARKS the live
 * old child) -- the knowledge exists, it is simply not passed on.
 *
 * ☠ UNDECLARED IS THE ZERO VALUE ON PURPOSE.  An unconverted site keeps its own
 * bucket rather than being attributed to a regime it may not belong to, so the
 * undeclared count is the remaining work, in the open.  Never convert a site by
 * guessing: a wrong declaration turns the audit into a confident wrong zero,
 * which is worse than the honest "not yet classified" it replaced.
 *
 * Costs nothing: the value is consumed only by the debug audit and compiles out
 * with it.
 */
enum ft_word_excl {
	FT_EXCL_UNDECLARED = 0,	/* not yet classified -- counted, never scored */
	FT_EXCL_HIDDEN,		/* unpublished: no reader, no peer, nothing to race */
	FT_EXCL_MW_CAS,		/* transacted: the record's expected-old arbitrates */
	FT_EXCL_LOCKED,		/* the op holds the owner (per-node or FT-wide) */
};

/*
 * THE RAW-PRODUCER CANARY TAG.  Defined here rather than beside the table in
 * ft-txn-hlist.h because the producers it tags live in files included BEFORE
 * that one, and a macro -- unlike the audit arms above -- cannot be forward
 * declared.  The single TU resolves the function.
 */
#ifdef FT_DEBUG_CHAIN_CANARY
static void ft_chain_canary_stamp_id(const char *fn, int line, void **slot,
		bool raw, unsigned char id);
# define FT_CHAIN_CANARY_RAW(slot, id)					\
	ft_chain_canary_stamp_id(__func__, __LINE__, (void **) (slot), true, (id))
#else
# define FT_CHAIN_CANARY_RAW(slot, id)	do { (void) (slot); } while (0)
#endif

#ifdef FT_DEBUG_CHAIN_HOLD
/*
 * The duplicate-chain hold audit lives in ft-mutation-helpers.h (it needs the
 * per-thread hold ledger defined there), which is included AFTER this file.
 * Declare the head-word arm here and let the single TU resolve it -- the same
 * arrangement ft_hlist_store_mw_at already uses for the audit's coarse arm.
 */
static void ft_ch_audit_head_at(const char *fn, int line,
		const struct cds_ft *ft, struct cds_ft_node *head,
		struct cds_ft_inode_flag *owner_flag, enum ft_word_excl excl);
struct ft_flip_txn;
struct ft_lock_ctx;
/*
 * The node-body arm: a live node's own bitmap + child-slot words.
 *
 * ☞ IT TAKES ALL THREE WITNESSES.  Asking only ft_hold_trace_holds() here read
 * 3,956,849 "violations" on the refused path -- from the witness that DROPS its
 * entry the moment a release is recorded (@a5e1b0ce), so a site that acquired
 * and already recorded its release reads EMPTY.  The registry and the wide ctx
 * live one frame up in ft_node_set_nth_rec; FT_CH_TXN_PARAM carries them down
 * so the verdict is a bracket rather than one biased witness.
 */
static void ft_ch_audit_body_at(const char *fn, int line,
		const struct cds_ft *ft, struct cds_ft_metadata *owner,
		enum ft_word_excl excl, const struct ft_flip_txn *t,
		const struct ft_lock_ctx *ctx);
/*
 * Debug-only pass-through of the op's txn + lock ctx, mirroring
 * FT_BE_SITE_PARAM: release signatures are untouched, so this costs nothing
 * outside the audit build.
 */
# define FT_CH_TXN_PARAM	, const struct ft_flip_txn *dbg_ch_t,	\
				const struct ft_lock_ctx *dbg_ch_ctx
# define FT_CH_TXN_ARG(t, c)	, (t), (c)
/* Forward an already-received pair (cannot go through FT_CH_TXN_ARG: macro
 * arguments are split before expansion, so a 2-in-1 token is ONE argument). */
# define FT_CH_TXN_FWD		, dbg_ch_t, dbg_ch_ctx
/* No witness to offer: the caller builds a FRESH node (defer_parent), so the
 * audit buckets it HIDDEN and never asks. */
# define FT_CH_TXN_NONE		, NULL, NULL
# define FT_CH_TXN_USE		dbg_ch_t, dbg_ch_ctx
/* The metadata.parent_word arm (the table's FT-SLOT-3 row); same arrangement. */
static void ft_ch_audit_parent_at(const char *fn, int line,
		const struct cds_ft *ft, const struct cds_ft_metadata *child_meta,
		struct cds_ft_inode_flag *owner_flag, enum ft_word_excl excl);
# define ft_ch_audit_head(ft, head, owner)				\
	ft_ch_audit_head_at(__func__, __LINE__, (ft), (head), (owner),	\
		FT_EXCL_UNDECLARED)
#else
# define ft_ch_audit_head(ft, head, owner)	do { } while (0)
/* The _at spelling is called directly by ft_set_parent_at, which forwards its
 * caller's location -- so it needs a no-op too, or a non-debug build breaks. */
/*
 * ☠ VARIADIC ON PURPOSE.  These take a pass-through pair that expands from ONE
 * macro token (FT_CH_TXN_USE -> "NULL, NULL"): a function call expands then
 * splits, but a fixed-arity macro splits BEFORE expanding and reports "requires
 * 7 arguments, but only 6 given".  Variadic keeps the no-op arity-agnostic.
 */
# define ft_ch_audit_head_at(...)	do { } while (0)
# define ft_ch_audit_parent_at(...)	do { } while (0)
# define ft_ch_audit_body_at(...)	do { } while (0)
# define FT_CH_TXN_PARAM
# define FT_CH_TXN_ARG(t, c)
# define FT_CH_TXN_FWD
# define FT_CH_TXN_NONE
# define FT_CH_TXN_USE		NULL, NULL
#endif

static inline __attribute__((unused))
void static_array_size_check(void)
{
	CAA_BUILD_BUG_ON(CAA_ARRAY_SIZE(ft_types) < FT_TYPE_MAX_NR);
#ifdef FEATURE_FT_SKIP_COMPRESSED
	/*
	 * parent_slot_offset is 8 bits and stores byte_offset / sizeof(void *).
	 * Ensure the largest node (pigeon, 2^11 = 2048 bytes) fits:
	 * 2048 / sizeof(void *) = 256 slots, max index 255.  Only enabled
	 * on 64-bit architectures, where sizeof(void *) == 8 and the
	 * quotient is exactly 256.
	 */
	CAA_BUILD_BUG_ON((1U << 11) / sizeof(void *) > 256);
#endif
	/*
	 * Metadata packed bitfield must fit in a uint32_t.
	 * Layout: nr_child(9) + [parent_slot_offset(8)] + alloc_index
	 *         (near: FT_ALLOC_INDEX_BITS + 3; far: a separate uint32_t).
	 */
	CAA_BUILD_BUG_ON(9
#ifndef FT_FAR_METADATA
		/* far-metadata stores alloc_index as its own uint32_t. */
		+ (FT_ALLOC_INDEX_BITS + 3)
#endif
#ifdef FEATURE_FT_SKIP_COMPRESSED
		+ 8
#endif
		> 32);
}

/*
 * Reader-side helpers for the cds_ft_node.next removal tombstone (bit 1,
 * see CDS_FT_NODE_REMOVED_FLAG).  These run under the writer mutex (or RCU
 * read lock on the chain-walk side), so a plain masked load is sufficient;
 * readers use cds_ft_node_next_rcu() instead.
 *
 *   ft_node_next        masked successor (the actual chain link)
 *   ft_node_is_removed  has @node been removed from the trie?
 *
 * SETTING the tombstone is a COMMITTED flip edge -- the freed node's own next
 * is a word a concurrent duplicate-append CASes, so the mark rides the
 * descriptor protocol like every other reader-visible store; see
 * ft_node_mark_removed_flip / ft_chain_mark_removed_flip in
 * ft-mutation-helpers.h (doc/design/mcas-multiwriter-readiness.md §4
 * refinement-1 site 2).
 */
/*
 * May a mutation write a LIVE node's {child pointer, occupancy bitmap,
 * nr_child} in place, instead of routing through a whole-node recompact?
 *
 * TWO predicates, because the tree has two in-place tiers with two different
 * safety arguments, and a site must say which one it stands on:
 *
 *   ft_in_place_ok(ft)       -- the POINT-OP tier.  The build flag is the
 *                               OPT-IN; the SAFETY CONDITION is the CALLER's:
 *                               it must HOLD the node before it writes.  Under
 *                               a FINE strategy that is the node's own DLM lock
 *                               (per-node spacing) or its anchor (exponential /
 *                               root-only), taken BEFORE the reserve and
 *                               released by the same commit that publishes the
 *                               edge (ft_attach_node); under COARSE it is the
 *                               FT-wide writer lock every point op already
 *                               holds.  ☐ The delete side (ft_detach_node's
 *                               in-place arm) never stores raw at all -- its
 *                               slot NULL and nr_child-- are RECORDS in the
 *                               commit, arbitrated on the holder's state word
 *                               -- but the point removes still vouch only the
 *                               exclusive tier below: one op at a time, and
 *                               the delete's own validation step is next.
 *                               Answers true on EVERY trie type -- shared or
 *                               exclusive, coarse or fine -- so it is the
 *                               build flag alone, and the caller's vouch is
 *                               what a site passes down as @in_place.
 *
 *   ft_in_place_excl_ok(ft)  -- the legacy EXCLUSIVE-ONLY tier, for the sites
 *                               that have NOT been converted to lock-before-
 *                               write: the bulk reserves (a graft's / rekey's
 *                               dst attach parent, ft-graft.h), the build-path
 *                               wrapper ft_node_set_nth, and the bulk detaches.
 *                               cds_ft_attr_set_exclusive declares "single-
 *                               writer, no concurrent readers", which is what
 *                               makes an unlocked in-place store sound there
 *                               (mcas-multiwriter-readiness.md §5.2).  It is
 *                               ALSO what keeps the same-trie move's reader-
 *                               coherence witness honest: a rekey's dst attach
 *                               parent must RELOCATE on a shared trie so
 *                               ft_lookup_two_descents' address fold perturbs
 *                               (ft-reintroduce-in-place-mutations.md §3), and
 *                               this predicate is false there.
 *
 * WHY THE POINT-OP TIER IS SOUND AGAINST READERS: an in-place mutation NEVER
 * REMOVES A BITMAP BIT, so no existing entry's rank moves.  The insert refuses
 * (-ERANGE -> recompact) unless the new bit is strictly the highest, publishes
 * the slot release-first and the gating bitmap bit last; the delete leaves the
 * bit STICKY and only NULLs the slot, and every reader already treats a set bit
 * over a NULL slot as "not present" (the same hole a reserved byte reads as
 * between its reserve and its commit).  Concurrent readers were supported with
 * exactly this protocol before the MW-with-CAS model withdrew it; what the
 * MW model needed -- a whole-node replacement so one CAS could arbitrate every
 * word -- the DLM locks now provide by exclusion instead.
 *
 * PER TIER.  The insert tier (FEATURE_FT_INSERT_IN_PLACE) and the delete tier
 * (FEATURE_FT_DELETE_IN_PLACE) are separate build switches, each with its own
 * predicate: a site that EDITS in place asks the tier of the edit it makes.
 * ft_in_place_ok / ft_in_place_excl_ok answer "EITHER tier": they are for the
 * sites that refuse or re-route a shape because an in-place tier would edit a
 * node where it stands (the same-trie rekey's gates), which must refuse as
 * soon as either tier could.  With both tiers compiled out
 * (-DNO_FEATURE_FT_{INSERT,DELETE}_IN_PLACE) every predicate answers no and
 * every caller recompacts.
 */
/*
 * THE SAME-TRIE REKEY RUNS WITHOUT THE IN-PLACE TIERS, on the thread running it
 * (ft_rekey_dispatch holds this non-zero).  Its folds are built on the premise
 * that every edit is a RECORD in the descriptor or a build-invisible copy, and
 * its gates refuse a shape outright wherever an in-place tier could edit a node
 * where it stands (ft_in_place_excl_ok, an exclusive trie's bulk tier) -- so
 * with a tier compiled in, an EXCLUSIVE trie's rekey refused moves the same
 * build without the tier serves (measured: test_rekey_exclusive_drain_after_
 * loss on nocompress, either tier).  A bulk op already pays grace periods;
 * the in-place stores buy it nothing.  Thread-local because an exclusive trie
 * has one writer and a shared one never reaches the exclusive tier, so no
 * other op can observe the difference.
 */
static __thread unsigned int ft_tls_in_place_off;

static inline
bool ft_in_place_insert_ok(const struct cds_ft *ft)
{
	(void) ft;
#ifdef FEATURE_FT_INSERT_IN_PLACE
	return !ft_tls_in_place_off;
#else
	return false;
#endif
}

static inline
bool ft_in_place_delete_ok(const struct cds_ft *ft)
{
	(void) ft;
#ifdef FEATURE_FT_DELETE_IN_PLACE
	return !ft_tls_in_place_off;
#else
	return false;
#endif
}

static inline
bool ft_in_place_ok(const struct cds_ft *ft)
{
	return ft_in_place_insert_ok(ft) || ft_in_place_delete_ok(ft);
}

static inline
bool ft_in_place_insert_excl_ok(const struct cds_ft *ft)
{
	return ft_in_place_insert_ok(ft) && ft->exclusive;
}

static inline
bool ft_in_place_delete_excl_ok(const struct cds_ft *ft)
{
	return ft_in_place_delete_ok(ft) && ft->exclusive;
}

static inline
bool ft_in_place_excl_ok(const struct cds_ft *ft)
{
	return ft_in_place_ok(ft) && ft->exclusive;
}

static inline
struct cds_ft_node *ft_node_next(const struct cds_ft_node *node)
{
	return (struct cds_ft_node *) ((uintptr_t) node->next &
			~CDS_FT_NODE_REMOVED_FLAG);
}

static inline
bool ft_node_is_removed(const struct cds_ft_node *node)
{
	return ((uintptr_t) node->next & CDS_FT_NODE_REMOVED_FLAG) != 0;
}

/*
 * Iterate through duplicates returned by cds_ft_lookup*()
 * Receives a struct cds_ft_node * as parameter, which is used as start
 * of duplicate list and loop cursor.
 *
 * ☠ RESOLVES THE ENGINE PROXY, and must: bit 0 of cds_ft_node.next is the
 * transactional engine's in-band proxy tag, parked while a concurrent commit is
 * in flight on this chain -- exactly what CDS_FT_NODE_TXN_PROXY_TAG's own header
 * describes.  ft_node_next() masks ONLY the removal tombstone (bit 1), so a walk
 * built on it hands the raw PROXY back as if it were a successor and the next
 * hop dereferences a DESCRIPTOR as a node.  MEASURED: inv_concurrent_same_key
 * _removes SEGVs 5/5 in _cds_ft_insert's "find last duplicate" walk, faulting on
 * a value read out of a descriptor rather than a chain (1 << 57, no node there),
 * the moment the in-place delete tier makes such a commit overlap the walk.
 *
 * cds_ft_node_next_rcu() is the accessor that answers this correctly -- it
 * resolves a parked proxy to the committed successor and then strips the
 * tombstone -- and it is what the PUBLIC cds_ft_for_each_duplicate_rcu() has
 * always used.  ft_node_next() stays for the single-hop liveness tests, where
 * the question is about THIS node's own word rather than a successor to follow.
 */
#define cds_ft_for_each_duplicate(pos)				\
       for (; (pos) != NULL; (pos) = cds_ft_node_next_rcu(pos))

enum ft_recompact {
	FT_RECOMPACT_ADD_SAME,
	FT_RECOMPACT_ADD_NEXT,
	FT_RECOMPACT_DEL,
	/*
	 * Pure relocation: same type and child set, copied verbatim into a
	 * fresh allocation (new address), children reparented, republished
	 * into the parent slot (and skip slot, via the shared publish path).
	 * Used by cds_ft_compact() to defragment the node arenas.
	 */
	FT_RECOMPACT_RELOCATE,
};

enum ft_lookup_inequality {
	FT_LOOKUP_GE,
	FT_LOOKUP_LE,
	FT_LOOKUP_GT,
	FT_LOOKUP_LT,
};

enum ft_lookup_limit {
	FT_LOOKUP_LIMIT_NONE,
	FT_LOOKUP_LIMIT_FIRST,
	FT_LOOKUP_LIMIT_LAST,
};

enum ft_direction {
	FT_LEFT,
	FT_RIGHT,
	FT_LEFTMOST,
	FT_RIGHTMOST,
};

/*
 * Fractal Trie iterator object. Can be used to keep backtracking state
 * across API calls. Path use for backtracking requires to keep RCU
 * read-side lock held across calls.
 *
 * The iterator lifetime is bound to the Trie. The Trie must not be
 * destroyed while iterators to that trie exist.
 *
 * The @prefix_len is the length of the key prefix within the key for
 * traversal under a given key prefix. Iterate over the entire Trie when
 * @prefix_len=0.
 */
struct cds_ft_iter {
	struct cds_ft *ft;		/* Point to the associated Fractal Trie. */
	struct cds_ft_node *node;	/* Current external node. */
	size_t path_len;		/* Key-path length of the cached position. */
	size_t key_len;			/* Key length of the current node. */
	size_t prefix_len;		/* Key prefix length. */
	enum cds_ft_status status;	/* Iteration status. */
	enum cds_ft_iter_cache_mode cache_mode;	/* Position-reuse mode (CACHED/UNCACHED). */
	bool cache_valid;		/* Whether the cached position is valid. */
	/*
	 * Byte offset within the @data buffer at which the current-position key
	 * begins.  0 for a descent / set_key key (filled at the front); the
	 * structural up-walk fills the key at the TAIL and sets this to
	 * (max_key_len - key_len) so ft_iter_read_key returns the right pointer
	 * even after the cached position is invalidated (bind / UNCACHED), with
	 * no copy to normalize the key to the front.
	 */
	size_t key_off;

	/*
	 * Ordinal-cell walk cursor.  @ord_cell caches the cell of the current
	 * head so cds_ft_next / cds_ft_prev advance via cell->ord_next/prev
	 * without re-loading the head's leaf each step; @ord_cell_node records
	 * the node it was cached for, so the cache is honoured only while
	 * @ord_cell_node == iter->node (a point lookup or descent that re-seeded
	 * iter->node leaves a mismatch, and the first step re-enters the walk via
	 * iter->node->prev -- the one leaf touch per walk entry).  No stale cell is
	 * dereferenced: validity is a node-pointer compare, not a cell read.
	 */
	struct ft_ord_cell *ord_cell;
	struct cds_ft_node *ord_cell_node;

	/*
	 * CARRIED POSITION KEY (in-trie move coherence).  When @pos_key_node ==
	 * @node, iter_key() holds -- at offset 0, length @key_len -- the key this
	 * position had when the last coherent step CONFIRMED it, as that step's
	 * own traversal spelled it.
	 *
	 * A continuation step taken while a move is in flight is defined against
	 * that KEY and re-descends from it, instead of hopping the ordered-list
	 * cells: a move rewrites the moved run's outer cell links IN PLACE, so a
	 * walker parked on one hops into the run's new neighbourhood and skips
	 * every key in between -- and no reader can detect that, because nothing
	 * it can observe changed address.  The tree path CAN be detected (the move
	 * COWs the moved subtree's top), which is what the two-pass rests on.
	 *
	 * The node pointer makes the carried key SELF-VALIDATING (usable only
	 * while it still describes @node), so no other path has to clear it.
	 */
	struct cds_ft_node *pos_key_node;

#ifdef URCU_FRACTAL_TRIE_DEBUG_PATH
	struct urcu_gp_poll_state gp_state;	/* GP snapshot when path was populated. */
	bool gp_state_valid;			/* Whether gp_state holds a meaningful value. */
#endif

	/*
	 * Trailing buffer holding the ordinal key bytes of the current
	 * iterator position.  The going-up backtrack recovers per-level
	 * nodes from the live parent chain, so no path-node array is kept.
	 *
	 * Flexible array member, pointer-aligned.
	 */
	char data[] __attribute__((__aligned__(sizeof(struct cds_ft_inode_flag *))));
};

/* Start of the uint8_t key array. */
#define iter_key(iter) \
	((uint8_t *)((iter)->data))

/*
 * Validate the iterator-based lookup contract: @ft must be the trie the
 * iterator was created for (cds_ft_iter_create).  The descent uses @ft while
 * key handling uses iter->ft, so passing a different trie mixes their key
 * mappings and produces undefined results.  Debug-only; compiled out under
 * NDEBUG.
 */
static inline
void ft_iter_assert_bound(const struct cds_ft *ft __attribute__((unused)),
		const struct cds_ft_iter *iter __attribute__((unused)))
{
	assert(ft == iter->ft);
}

/*
 * Debug helpers for detecting stale cached iterator paths.
 *
 * Three entry-point roles mirror the rculfhash pattern:
 *
 *  iter_debug_path_snapshot() -- unconditionally captures a fresh
 *      grace-period poll state.  Called at the entry of every
 *      fresh-population operation (lookup, longest-match lookup, and
 *      the slow-path / early-exit branches of inequality lookup).
 *      Because it always overwrites the snapshot, an iterator that is
 *      reused across RCU read-side critical sections gets a current
 *      baseline, preventing false positives on the next check.
 *
 *  iter_debug_path_check() -- polls the existing snapshot.  Called at
 *      continuation entry points that consume a previously populated
 *      cached path (inequality fast-path, replace, remove).  If a full
 *      grace period has elapsed since the snapshot was taken, the RCU
 *      read-side lock must have been dropped and the cached pointers
 *      may reference freed memory -- the check aborts.
 *
 *  iter_debug_path_update() -- invalidates the snapshot when the path
 *      becomes invalid (node not found / end of traversal).  It never
 *      captures a new snapshot; the one taken at the operation's entry
 *      point persists as long as the path remains valid, giving a
 *      tighter detection window.
 *
 *  iter_debug_path_clear() -- unconditionally resets the snapshot
 *      validity.  Used by iter_auto_invalidate_cache() and by
 *      operations that structurally modify the trie (replace, remove),
 *      after which the cached path is stale regardless of RCU state.
 */
#ifdef URCU_FRACTAL_TRIE_DEBUG_PATH

/*
 * Unconditionally capture a fresh grace-period snapshot.  Called at
 * the entry of fresh-population operations so that any prior stale
 * state left by iterator reuse is replaced.
 */
static inline
void iter_debug_path_snapshot(struct cds_ft_iter *iter)
{
	const struct rcu_flavor_struct *flavor = iter->ft->group->flavor;

	iter->gp_state = flavor->update_start_poll_synchronize_rcu();
	iter->gp_state_valid = true;
}

/*
 * Validate that the RCU read-side lock has been held continuously
 * since the snapshot was captured.  Called at continuation entry
 * points before reusing a cached path.
 */
static inline
void iter_debug_path_check(const struct cds_ft_iter *iter)
{
	const struct rcu_flavor_struct *flavor = iter->ft->group->flavor;

	if (iter->cache_mode != CDS_FT_ITER_CACHED)
		return;
	if (!iter->cache_valid)
		return;
	if (!iter->gp_state_valid)
		return;
	if (caa_unlikely(flavor->update_poll_state_synchronize_rcu(
				iter->gp_state))) {
		fprintf(stderr,
			"[Fatal] Fractal Trie: cached iterator path "
			"used after a grace period elapsed (RCU "
			"read-side lock was likely dropped). "
			"%s:%d\n", __FILE__, __LINE__);
		abort();
	}
}

/*
 * Update the snapshot validity after populating the iterator.  When
 * the path is no longer valid (node not found or end of traversal),
 * clear the snapshot so that any subsequent misuse is detected by
 * iter_debug_path_check.  When the path is valid, the grace-period
 * snapshot captured by iter_debug_path_snapshot at the operation's
 * entry point remains current because the RCU read-side lock must be
 * held continuously.
 */
static inline
void iter_debug_path_update(struct cds_ft_iter *iter)
{
	if (!iter->cache_valid)
		iter->gp_state_valid = false;
}

static inline
void iter_debug_path_clear(struct cds_ft_iter *iter)
{
	iter->gp_state_valid = false;
}
#else
static inline
void iter_debug_path_snapshot(struct cds_ft_iter *iter __attribute__((unused)))
{
}

static inline
void iter_debug_path_check(const struct cds_ft_iter *iter __attribute__((unused)))
{
}

static inline
void iter_debug_path_update(struct cds_ft_iter *iter __attribute__((unused)))
{
}

static inline
void iter_debug_path_clear(struct cds_ft_iter *iter __attribute__((unused)))
{
}
#endif

/*
 * Snapshot the iterator's current result key into its own buffer when that key
 * is a live reference into the matched leaf (a lazy-ref ordinal-cell group), so
 * a later re-descent reads a stable key rather than the soon-to-be-reclaimed
 * leaf.  A no-op for groups whose key is already a value in iter_key(iter)
 * (the descent filled it / a non-ordered-list group): the next re-descent uses
 * that buffer directly.  Defined after ft_speculative_keycopy_unconditional.
 */
static inline void ft_iter_materialize_key(struct cds_ft_iter *iter);
/* Defined in ft-iter.h: is the key a reference INTO @iter->node's leaf? */
static inline bool ft_iter_key_referenced(const struct cds_ft_iter *iter);

/*
 * Drop the cached POSITION while preserving the KEY, for a site that is about
 * to re-descend from that key.
 */
static inline
void ft_iter_drop_position_keep_key(struct cds_ft_iter *iter)
{
	/*
	 * DROP THE CACHED POSITION, KEEP THE KEY.
	 *
	 * ☠ A BARE `cache_valid = false` CAN DESTROY THE KEY.  On a keycopy trie
	 * (ordered list + speculative_key_offset + identity key map) the key is
	 * not stored in the iterator at all: ft_iter_key_referenced() is true and
	 * ft_iter_read_key() returns @iter->node + speculative_key_offset.  That
	 * predicate requires @cache_valid, so clearing it makes read_key fall
	 * through to iter_key(iter) -- a buffer nothing ever wrote -- and the key
	 * reads back as ZEROS.  The same applies to the eager ordered-list
	 * up-walk arm, which also tests @cache_valid.
	 *
	 * So any site that drops the position INTENDING to re-descend from the
	 * key must materialize first.  MEASURED: cds_ft_remove_all's exponential
	 * retriable bail dropped the cache "so the next attempt re-seeds through
	 * a fresh lookup", and the next attempt re-seeded from key 0 and reported
	 * NOT_FOUND for a key that was present and reachable
	 * (inv_compact_keycopy_terminates, key 248 of 2000, exponential only).
	 *
	 * This is the same materialize-then-clear that iter_auto_invalidate_cache
	 * does for the UNCACHED contract -- the reason is identical, so the order
	 * is identical.
	 */
	/*
	 * ☠ ONLY THE LEAF-REFERENCED KEY, and the gate is not caution -- it is
	 * the difference between reading a live leaf and WALKING A DETACHED
	 * SUBTREE.  ft_iter_read_key has two lazy arms:
	 *
	 *  - key REFERENCED (this one): a single load from @iter->node at
	 *    @speculative_key_offset.  The node is alive here even after a
	 *    detach -- the caller is handed it to free -- so the read is sound,
	 *    and it is the only arm whose key the cache drop would destroy.
	 *  - the eager ordered-list arm: an O(depth) PARENT UP-WALK
	 *    (ft_rebuild_key_upwalk).  Callers reach this helper AFTER the
	 *    structure changed, and an up-walk from a node the op has just
	 *    unlinked runs off an external node -- MEASURED as ft_unit test 187
	 *    SEGV on release and `Assertion !(nf) || !ft_node_external(nf)` on
	 *    --enable-rcu-debug, at PER-NODE spacing, when this materialized
	 *    unconditionally.
	 *
	 * The second arm needs no materialize anyway: it derives the key from
	 * the trie rather than from the dropped position.
	 */
	if (ft_iter_key_referenced(iter))
		ft_iter_materialize_key(iter);
	iter->cache_valid = false;
	iter_debug_path_clear(iter);
	iter->path_len = 0;
}

/*
 * Discard the cached position if the iterator is in uncached mode.
 * Called at the end of each public iterator-based operation.
 * Preserves iter->node so the caller can read the result.
 */
static inline
void iter_auto_invalidate_cache(struct cds_ft_iter *iter)
{
	/*
	 * ☞ AND UNCACHED FOR EVERYONE WHILE A MOVE IS IN FLIGHT (MATHIEU).  In the
	 * rekey-coherent double-descent mode (ft_move_active) a cached position
	 * can straddle the move and name a place its key no longer is; the key is
	 * the absolute truth of the iterator's position, so keep the key and drop
	 * the position, and the next op re-descends by key -- coherently.  The
	 * gate's grace period drained every section older than the flag, so no
	 * cache older than the move survives into it.
	 */
	if (iter->cache_mode == CDS_FT_ITER_UNCACHED ||
			ft_move_active(iter->ft)) {
		/*
		 * Materialize a live leaf-referenced key BEFORE clearing, so the
		 * next uncached re-descent reads the saved key, not a stale leaf.
		 */
		ft_iter_materialize_key(iter);
		iter->cache_valid = false;
		iter->path_len = 0;
		iter_debug_path_clear(iter);
	}
}

static
size_t ft_key_len(const struct cds_ft *ft, size_t key_len)
{
	struct cds_ft_group *ft_group = ft->group;

	if (key_len == CDS_FT_LEN_DEFAULT) {
		if (ft_group->key_len == CDS_FT_LEN_VARIABLE)
			return CDS_FT_LEN_ERROR;
		return ft_group->key_len;
	}
	/* Validate that explicit and implicit key lengths match for fixed length Fractal Trie. */
	if (ft_group->key_len != CDS_FT_LEN_VARIABLE && key_len != ft_group->key_len)
		return CDS_FT_LEN_ERROR;
	return key_len;
}

uint64_t cds_ft_key_to_u64(const struct cds_ft *ft, const uint8_t *key,
		size_t _key_len)
{
	size_t key_len = ft_key_len(ft, _key_len);
	union {
		uint64_t v64;
		uint8_t array[8];
	} u;

	if (key_len == CDS_FT_LEN_ERROR || key_len > 8)
		return 0;
	u.v64 = 0;
	/* Copy len LSB. */
	memcpy(u.array + sizeof(u.array) - key_len , key, key_len);
	/* Big endian to host endianness. */
	return be64toh(u.v64);
}

void cds_ft_u64_to_key(const struct cds_ft *ft, uint64_t v, uint8_t *key,
		size_t _key_len)
{
	size_t key_len = ft_key_len(ft, _key_len);
	union {
		uint64_t v64;
		uint8_t array[8];
	} u;

	if (key_len == CDS_FT_LEN_ERROR || key_len > 8)
		return;
	/* Host endianness to big endian. */
	u.v64 = htobe64(v);
	/* Copy len LSB. */
	memcpy(key, u.array + sizeof(u.array) - key_len , key_len);
}

uint32_t cds_ft_key_to_u32(const struct cds_ft *ft, const uint8_t *key,
		size_t _key_len)
{
	size_t key_len = ft_key_len(ft, _key_len);
	union {
		uint32_t v32;
		uint8_t array[4];
	} u;

	if (key_len == CDS_FT_LEN_ERROR || key_len > 4)
		return 0;
	u.v32 = 0;
	/* Copy len LSB. */
	memcpy(u.array + sizeof(u.array) - key_len , key, key_len);
	/* Big endian to host endianness. */
	return be32toh(u.v32);
}

void cds_ft_u32_to_key(const struct cds_ft *ft, uint32_t v, uint8_t *key,
		size_t _key_len)
{
	size_t key_len = ft_key_len(ft, _key_len);
	union {
		uint32_t v32;
		uint8_t array[4];
	} u;

	if (key_len == CDS_FT_LEN_ERROR || key_len > 4)
		return;
	/* Host endianness to big endian. */
	u.v32 = htobe32(v);
	/* Copy len LSB. */
	memcpy(key, u.array + sizeof(u.array) - key_len , key_len);
}

/*
 * Signed integer key helpers.
 *
 * Signed integers need a sign-bit flip (XOR with the MSB of the
 * key-width value) so that the big-endian byte ordering used by the
 * Fractal Trie preserves the natural signed ordering.
 *
 * When the key is the full width of the integer type (e.g. 8 bytes
 * for int64_t), the mapping is:
 *
 *   INT64_MIN  -> 0x0000000000000000   (sorts first)
 *   -1         -> 0x7FFFFFFFFFFFFFFF
 *    0         -> 0x8000000000000000
 *   INT64_MAX  -> 0xFFFFFFFFFFFFFFFF   (sorts last)
 *
 * The same principle applies to 32-bit signed integers.
 *
 * When the key is narrower than the integer type (e.g. a 2-byte key
 * representing a signed 16-bit range within a 64-bit integer), the
 * sign bit is at position (key_len * 8 - 1), not at the MSB of the
 * full integer.  The key-to-integer direction therefore sign-extends
 * from the key's MSB to fill the integer.
 */

int64_t cds_ft_key_to_s64(const struct cds_ft *ft, const uint8_t *key,
		size_t _key_len)
{
	size_t key_len = ft_key_len(ft, _key_len);
	unsigned int shift;
	uint64_t u;

	if (key_len == 0 || key_len > 8)
		return 0;
	u = cds_ft_key_to_u64(ft, key, _key_len);
	shift = key_len * 8;
	/* Flip sign bit (MSB of key-width value) to recover signed encoding. */
	u ^= 1ULL << (shift - 1);
	/* Sign-extend from key width to 64 bits. */
	if (shift < 64) {
		uint64_t sign_bit = 1ULL << (shift - 1);

		if (u & sign_bit)
			u |= ~((1ULL << shift) - 1);
	}
	return (int64_t) u;
}

void cds_ft_s64_to_key(const struct cds_ft *ft, int64_t v, uint8_t *key,
		size_t _key_len)
{
	size_t key_len = ft_key_len(ft, _key_len);
	unsigned int shift;

	if (key_len == 0 || key_len > 8)
		return;
	shift = key_len * 8;
	/* Flip sign bit so that negative values sort before positive. */
	cds_ft_u64_to_key(ft, (uint64_t) v ^ ( 1ULL << (shift - 1)), key, _key_len);
}

int32_t cds_ft_key_to_s32(const struct cds_ft *ft, const uint8_t *key,
		size_t _key_len)
{
	size_t key_len = ft_key_len(ft, _key_len);
	unsigned int shift;
	uint32_t u;

	if (key_len == 0 || key_len > 4)
		return 0;
	u = cds_ft_key_to_u32(ft, key, _key_len);
	shift = key_len * 8;
	/* Flip sign bit (MSB of key-width value) to recover signed encoding. */
	u ^= 1U << (shift - 1);
	/* Sign-extend from key width to 32 bits. */
	if (shift < 32) {
		uint32_t sign_bit = 1U << (shift - 1);

		if (u & sign_bit)
			u |= ~((1U << shift) - 1);
	}
	return (int32_t) u;
}

void cds_ft_s32_to_key(const struct cds_ft *ft, int32_t v, uint8_t *key,
		size_t _key_len)
{
	size_t key_len = ft_key_len(ft, _key_len);
	unsigned int shift;

	if (key_len == 0 || key_len > 4)
		return;
	shift = key_len * 8;
	/* Flip sign bit so that negative values sort before positive. */
	cds_ft_u32_to_key(ft, (uint32_t) v ^ (1U << (shift - 1)), key, _key_len);
}

static inline_lookup
uint8_t key_to_ordinal(uint8_t key,
		const struct cds_ft_key_map *km)
{
	if (caa_likely(km->identity))
		return key;
	return km->key_to_ordinal[key];
}

static inline_lookup
uint8_t ordinal_to_key(const struct cds_ft *ft, uint8_t ordinal)
{
	if (caa_likely(ft->group->key_map.identity))
		return ordinal;
	return ft->group->key_map.ordinal_to_key[ordinal];
}

/*
 * Bulk key-to-ordinal conversion.  Converts @len external key bytes
 * into ordinals in @dst.  Identity maps short-circuit to memcpy.
 */
#include "ft-key.h"

static
struct cds_ft_inode_flag *ft_node_flag(struct cds_ft_inode *node,
		unsigned long type)
{
	assert(type < (1UL << FT_TYPE_BITS));
	return (struct cds_ft_inode_flag *) (((unsigned long) node) |
		(type << FT_INTERNAL_BITS) |
		FT_INTERNAL_MASK);
}

/*
 * Test whether @node has the external tag (bits 0-1 == 0b00).
 * This matches both non-NULL external leaf pointers AND NULL,
 * since NULL has tag bits 0b00.  Callers that need to distinguish
 * NULL from a valid external node should also check ft_node_ptr().
 */
static inline_lookup
bool ft_node_external(struct cds_ft_inode_flag *node)
{
	return ((unsigned long) node & FT_TAG_MASK) == 0;
}

#ifdef FEATURE_FT_COMPRESS
static inline_lookup
bool ft_node_compressed(struct cds_ft_inode_flag *node)
{
	return ((unsigned long) node & FT_TAG_MASK) == FT_COMPRESSED_MASK;
}
#else
static
bool ft_node_compressed(struct cds_ft_inode_flag *node __attribute__((unused)))
{
	return false;
}
#endif

/*
 * ft_metadata_set_external_nodes: Phase 1 -- set the cluster-internal
 * forward pointer (metadata->external_nodes) on a freshly-built node.
 * Asserts that the node is not a compressed node (compressed nodes
 * must not carry metadata->external_nodes).
 *
 * This is the cluster-init step.  The matching back-channel publish
 * (external_nodes->prev = node_flag) is intentionally NOT done here:
 * setting prev makes the cluster reachable to up-walkers via the live
 * external's back-pointer, so it must follow node_flag's own parent
 * being wired.  Use ft_publish_external_nodes_prev for that, ordered
 * after the cluster top's parent is set and immediately before (or as
 * part of) the forward publish.
 *
 * @node_flag: tagged pointer to the node (used for type check).
 * @metadata: the node's metadata.
 * @external_nodes: the external node list to set (may be NULL).
 */
static inline
void ft_metadata_set_external_nodes(struct cds_ft_inode_flag *node_flag,
		struct cds_ft_metadata *metadata,
		struct cds_ft_node *external_nodes)
{
	if (ft_node_compressed(node_flag)) {
		fprintf(stderr, "BUG: ft_metadata_set_external_nodes called on compressed node %p\n", node_flag);
		abort();
	}
	metadata->external_nodes = external_nodes;
	FT_TP(metadata_set_external_nodes, (const void *) node_flag,
		(const void *) external_nodes);
}

/*
 * Pointer unmasking via speculative mask + conditional select.
 *
 * Exploit the fact that each internal node type's allocation order
 * equals 4 + type_idx (type 0 is 16B-aligned, type 1 is 32B, etc.)
 * to compute the internal-node mask speculatively, in parallel with
 * the bit-0 test:
 *
 *   mask_internal = (~15UL) << ((v >> 1) & 7)
 *                 = ~0UL << (4 + type_idx)
 *
 * This clears all tag bits that sit below the type's alignment
 * boundary.  The shift amount is derived purely from bits 1-3 with
 * no dependency on bit 0.
 *
 * For non-internal nodes (bit 0 clear): external nodes are >= 8-byte
 * aligned (bits 0-2 zero), compressed nodes are >= 16-byte aligned
 * with tag in bit 1.  A fixed ~7UL mask suffices.
 *
 * The conditional select lets the two mask computations run in
 * parallel; the compiler emits a CMOV, keeping the critical path
 * to 4 cycles.
 */
/*
 * nr_keys is an MCAS-transacted scalar: on the rank-stats-ON path the
 * order-statistics count is folded into the op's flip-txn (so it is exact under
 * concurrent writers instead of a drifting approximate aggregate).  Like the
 * per-node state word, it therefore reserves its LOW bit for the engine's
 * in-band proxy marker -- the logical count is stored as (count << 1) and bit 0
 * = FT_NR_KEYS_PROXY_TAG carries a parked proxy for the duration of a commit.
 * Bit 0 rather than a high bit so the reservation is valid on 32-bit too.
 * Access nr_keys ONLY through these helpers; never read/write the field direct.
 */
#define FT_NR_KEYS_PROXY_TAG	1UL

/* Writer-side read of an owned / quiescent node (never a mid-commit proxy). */
static inline
unsigned long ft_nr_keys_get(const struct cds_ft_metadata *m)
{
	return m->nr_keys >> 1;
}

/*
 * Reader-side count read: acquire-load and resolve a parked proxy to its
 * committed logical value.  urcu_txn_read short-circuits to a plain acquire
 * load whenever nr_keys holds no proxy (the common case, and always so under
 * writer exclusion), so the resolve costs nothing off the commit window.
 */
static inline
unsigned long ft_nr_keys_load(const struct cds_ft_metadata *m)
{
	return (unsigned long) urcu_txn_read(
			(void **) (uintptr_t) &m->nr_keys,
			FT_NR_KEYS_PROXY_TAG) >> 1;
}

/*
 * Store a node's order-statistics key count -- a no-op unless the trie
 * maintains order statistics (cds_ft_group_attr_set_rank_stats).  Gating the
 * single write chokepoint on @ft->rank_stats means a default (rank-stats-off)
 * trie touches the nr_keys field nowhere: no per-node init, no propagation, no
 * root-ward count contention.  @ft is read-only; on cross-trie ops src/dst
 * share a group (enforced) and thus the same flag, so any in-scope trie works.
 */
static inline
void ft_nr_keys_store(const struct cds_ft *ft, struct cds_ft_metadata *m,
		unsigned long val, int mo)
{
	if (ft->rank_stats)
		uatomic_store(&m->nr_keys, val << 1, mo);
}

/*
 * Reader-side read of a node's live-child count that resolves a mid-commit proxy
 * on the state word.  Once atomic detach is wired the LIVE->DEAD tombstone rides
 * an MCAS edge on state, transiently parking a proxy (a full pointer with bit 0
 * = FT_STATE_PROXY set) that would otherwise corrupt the nr_child bits for a
 * concurrent reader.  urcu_txn_read short-circuits to a plain acquire load when
 * state holds no proxy (always so under writer exclusion), so it costs nothing
 * off the commit window.  The writer-owned ft_meta_nr_child (direct read) stays
 * for reads of a node the caller owns or that is quiescent.
 */
static inline
unsigned int ft_meta_nr_child_load(const struct cds_ft_metadata *meta)
{
	return (unsigned int) (((uintptr_t) urcu_txn_read(
			(void **) (uintptr_t) &meta->state,
			FT_STATE_PROXY) >> FT_STATE_NR_CHILD_SHIFT)
			& FT_STATE_NR_CHILD_VALMASK);
}

/*
 * Proxy-resolving read of a node's parent-slot offset -- the Phase 4.3 mirror of
 * ft_meta_nr_child_load anticipated at the ft_meta_parent_slot_offset declaration.
 * parent_slot_offset shares the state word with the flip proxy, so a RAW read of
 * a live peer-owned node mid-commit returns the proxy pointer's bits as the
 * offset (arbitrary, up to FT_STATE_PSO_VALMASK): ft_get_parent_slot would then
 * compute ptr(parent) + garbage*8 = a WILD address and fault on deref, before any
 * commit guard runs.  urcu_txn_read resolves the proxy to the real state word
 * first (and short-circuits to a plain load when no proxy is parked -- always so
 * under writer exclusion, so it is free off the commit window).  The direct
 * ft_meta_parent_slot_offset stays for a node the caller owns / that is quiescent.
 */
static inline
unsigned int ft_meta_parent_slot_offset_load(const struct cds_ft_metadata *meta)
{
	return FT_PSO_DECODE(urcu_txn_read(
			(void **) (uintptr_t) &meta->parent_slot_offset,
			FT_STATE_PROXY));
}

/*
 * Flip-proxy tag (see the full encoding note above ft_node_flip_proxy's original
 * home, further down).  Hoisted here because the tag-stripping helpers below
 * must be able to assert against it.
 */
#define FT_FLIP_PROXY_TYPE	7U
#define FT_FLIP_PROXY_TAG	(FT_INTERNAL_MASK | (FT_FLIP_PROXY_TYPE << FT_INTERNAL_BITS))

/*
 * The parent slot's tag width must cover the proxy that parks in it: a trie
 * pointer is distinguished from every other parent value by clearing exactly
 * this mask (see ft_parent_is_trie).
 */
urcu_static_assert(FT_FLIP_PROXY_TAG <= FT_PARENT_TAG_MASK,
		"the flip-proxy tag must fit within FT_PARENT_TAG_MASK",
		ft_parent_tag_covers_proxy);

static inline_lookup
bool ft_node_flip_proxy(struct cds_ft_inode_flag *node)
{
	return ((unsigned long) node & (FT_INTERNAL_MASK | FT_TYPE_MASK))
		== FT_FLIP_PROXY_TAG;
}

/*
 * ft_head_parent_word: the value to store into an external head's back-edge
 * word (cell->parent list-on, node->prev list-off) so the head names @parent_nf
 * and answers the shape question itself (FT_PARENT_PREFIX_HEAD).
 *
 * @prefix: true iff the head is, or is becoming, @parent_nf's external_nodes.
 *
 * ☠ THE BIT ONLY EXISTS ON AN INTERNAL NODE'S FLAG.  An internal flag is the
 * one form whose low nibble has bit 0 set and is not the type-7 flip proxy;
 * everything else that can sit in a parent word -- a COMPRESSED node (bit 0
 * clear, 16-byte aligned), a root-position TRIE stamp, a parked PROXY -- uses
 * bit 4 as address or tag, so the bit is DROPPED for those forms rather than
 * refused.  The drop is the right answer: a head under a compressed node is
 * never a prefix head (a compressed node carries no external_nodes), a proxy
 * is transient and the real re-parent that replaces it re-derives the answer,
 * and every reader resolves a proxy before reading the bit.
 *
 * ☠ AND THE TEST IS bit 0, NOT (tag == FT_INTERNAL_MASK).  Bits 1..3 are the
 * node's TYPE, not spare: an internal node of type 1 carries 0b011, which a
 * whole-nibble compare against FT_INTERNAL_MASK rejects.  That mistake silently
 * drops the bit on every non-type-0 holder -- caught, on the first run, by
 * cds_ft_verify's at-rest check.
 */
static inline
struct cds_ft_inode_flag *ft_head_parent_word(struct cds_ft_inode_flag *parent_nf,
		bool prefix)
{
	/*
	 * A PROXY passes VERBATIM.  "Dropped" means not SET: stripping it
	 * would clear bit 4 of a record ADDRESS (the proxy's 0xF nibble has
	 * bit 0 set, which is all ft_parent_prefix_strip tests), and half the
	 * records of a 0x30-stride descriptor have it -- a proxy naming the
	 * middle of the record before.  No caller passes one today (both
	 * ft_set_parent_raw callers hand it the node being built); the
	 * comment there that says otherwise is older than they are.
	 */
	if (ft_node_flip_proxy(parent_nf))
		return parent_nf;
	if (!prefix || !((uintptr_t) parent_nf & FT_INTERNAL_MASK))
		return ft_parent_prefix_strip(parent_nf);
	return (struct cds_ft_inode_flag *)
		((uintptr_t) parent_nf | FT_PARENT_PREFIX_HEAD);
}

/*
 * ft_head_parent_word_carry: the same value for a park or a restore -- a write
 * that does NOT decide the shape, so the head keeps whatever it was.  Takes the
 * answer from @old_word, the head's current back-edge value (RESOLVED: the
 * callers that read a transacted word read it through urcu_txn_load, and the
 * plain-store ones resolve a parked proxy first).
 */
static inline
struct cds_ft_inode_flag *ft_head_parent_word_carry(
		struct cds_ft_inode_flag *parent_nf, const void *old_word)
{
	return ft_head_parent_word(parent_nf,
		ft_parent_prefix_head((const struct cds_ft_inode_flag *) old_word));
}

/*
 * RESOLVED-POINTER CONTRACT.
 *
 * A slot under an in-flight commit does not hold a node: it holds a pointer to
 * the transaction's MCAS record, tagged FT_FLIP_PROXY_TAG (low nibble 0xF).
 * Every structural predicate MISREADS such a flag rather than rejecting it --
 *
 *	ft_node_external(proxy)   -> (0xF & 0b11) == 0b00  -> false
 *	ft_node_compressed(proxy) -> (0xF & 0b11) == 0b10  -> false
 *
 * -- so a proxy silently dispatches as "an internal node of type 7", and the
 * tag-stripping helpers below then mint a wild node pointer out of the record's
 * address.  The fault surfaces frames later, in cds_ft_item_to_metadata(), as a
 * segfault on a garbage range -- with no trace of who failed to resolve.
 *
 * The contract is therefore: RESOLVE FIRST (ft_resolve_flip_proxy), THEN
 * dispatch on kind.  ft_assert_resolved() traps a violation at its first use,
 * where the culprit is still on the stack.  Build with -DFT_DEBUG_PROXY_ASSERT;
 * it compiles to nothing otherwise.
 *
 * Deliberately NOT asserted: _ft_node_mask_ptr() and ft_flip_proxy_ptr(), which
 * exist precisely to strip a proxy's tag, and ft_node_flip_proxy() itself.
 */
#ifdef FT_DEBUG_PROXY_ASSERT
#include <stdio.h>
#include <stdlib.h>

__attribute__((noinline, cold))
static void ft_proxy_assert_fail(const char *fn, const void *p)
{
	fprintf(stderr, "FT_PROXY_ASSERT: %s() got a parked flip proxy %p\n",
		fn, p);
	fflush(stderr);
	abort();
}

# define ft_assert_resolved(node)					\
	do {								\
		if (caa_unlikely(ft_node_flip_proxy(			\
				(struct cds_ft_inode_flag *) (node))))	\
			ft_proxy_assert_fail(__func__,			\
				(const void *) (node));			\
	} while (0)
#else
# define ft_assert_resolved(node)	((void) 0)
#endif

static inline_lookup
struct cds_ft_inode *ft_node_ptr(struct cds_ft_inode_flag *node)
{
	unsigned long v = (unsigned long) node;

	ft_assert_resolved(node);

	/*
	 * Compute mask from the original pointer: the skip-compressed
	 * length bits (57-63) don't affect bits 0-3 used for type
	 * dispatch, so this runs in parallel with the ADDR_MASK AND
	 * below (full ILP).
	 */
	unsigned long mask_internal = (~15UL) << ((v >> 1) & 7);
	unsigned long mask = (v & 1) ? mask_internal : ~7UL;

#ifdef FEATURE_FT_SKIP_COMPRESSED
	/*
	 * Clear the top FT_SKIP_LEN_BITS (7 bits).  In a hot loop
	 * the compiler hoists FT_ADDR_MASK into a register, making
	 * this a single 1-cycle AND that runs in parallel with the
	 * mask chain above.
	 */
	v &= FT_ADDR_MASK;
#endif

	return (struct cds_ft_inode *) (v & mask);
}

/*
 * Identity-only variant: mask a RAW slot value to a comparable address WITHOUT
 * asserting it is resolved.  A slot under an in-flight commit legitimately holds
 * a parked flip proxy, and a conflict check that only compares the masked value
 * against a descent-captured node (proxy != captured -> retry) never
 * dereferences it.  Use this at those sites, and ft_node_ptr() -- which asserts
 * -- everywhere the result is dereferenced.  Never dereference this result.
 */
static inline_lookup
struct cds_ft_inode *ft_node_ptr_raw(struct cds_ft_inode_flag *node)
{
	unsigned long v = (unsigned long) node;
	unsigned long mask_internal = (~15UL) << ((v >> 1) & 7);
	unsigned long mask = (v & 1) ? mask_internal : ~7UL;

#ifdef FEATURE_FT_SKIP_COMPRESSED
	v &= FT_ADDR_MASK;
#endif

	return (struct cds_ft_inode *) (v & mask);
}

/*
 * Lookup-hot variant: caller has already established that the
 * internal-flag bit is set (e.g. ft_node_get_nth_skip checks
 * !(tag & FT_INTERNAL_MASK) and returns NULL before this call).
 * Skips the (v & 1) ? ... : ~7UL branch in ft_node_ptr() above,
 * shaving the cmov/branch from the per-visit dependency chain on
 * the lookup hot path.
 */
static inline_lookup
struct cds_ft_inode *ft_node_ptr_internal(struct cds_ft_inode_flag *node)
{
	unsigned long v = (unsigned long) node;
	unsigned long mask = (~15UL) << ((v >> 1) & 7);

#ifdef FEATURE_FT_SKIP_COMPRESSED
	v &= FT_ADDR_MASK;
#endif

	assert((v & FT_INTERNAL_MASK) || node == NULL);
	return (struct cds_ft_inode *) (v & mask);
}

static
struct cds_ft_inode *_ft_node_mask_ptr(struct cds_ft_inode_flag *node)
{
	unsigned long v = (unsigned long) node;

#ifdef FEATURE_FT_SKIP_COMPRESSED
	v = (v << FT_SKIP_LEN_BITS) >> FT_SKIP_LEN_BITS;
#endif
	return (struct cds_ft_inode *) (v & FT_PTR_MASK);
}

static inline_lookup
bool ft_node_internal(struct cds_ft_inode_flag *node)
{
	return (unsigned long) node & FT_INTERNAL_MASK;
}

static inline_lookup
unsigned long ft_node_type(struct cds_ft_inode_flag *node)
{
	unsigned long type;

	ft_assert_resolved(node);

	if (_ft_node_mask_ptr(node) == NULL) {
		return NODE_INDEX_NULL;
	}
	/* Compressed nodes don't have a type index. */
	assert(!ft_node_compressed(node));
	type = (unsigned int) (((unsigned long) node & FT_TYPE_MASK) >> FT_INTERNAL_BITS);
	assert(type < (1UL << FT_TYPE_BITS));
	return type;
}

/*
 * Flip-proxy encoding (see <urcu/rcu-txn-sw.h>).  cds_ft_merge_at needs
 * to switch a whole set of back-pointers (and the merge-point forward
 * slot) from their old to their new target with no mixed-regime window.
 * Each such slot transiently holds a tagged pointer to a MCAS proxy
 * latch; a single MCAS flip commit store flips them all atomically.
 *
 * A proxy is tagged as a synthetic INTERNAL node of type-index 7, the
 * maximal tag value (low nibble (FT_INTERNAL_MASK | FT_TYPE_MASK) == 0xF).
 * ft_types[7] is FT_NULL on every arch -- the canonical NULL slot on
 * 64-bit (NODE_INDEX_NULL == 7), and reserved padding on 32-bit (where
 * NODE_INDEX_NULL == 6 and real types stop at 5) -- so no real node ever
 * carries type 7 in either tier.  A flag whose low nibble is 0xF is thus a
 * proxy and nothing else (external, compressed, NULL and skip pointers
 * never set all of bits 0..3).  The
 * proxy is 16-byte aligned (low 4 bits free for the tag) and lives at a
 * userspace address with the skip-len high bits clear, so
 * _ft_node_mask_ptr recovers it exactly.  The same encoding is valid in
 * both forward child slots and parent slots, since both resolve a flag
 * through this dispatch.
 */
/* FT_FLIP_PROXY_TYPE / FT_FLIP_PROXY_TAG / ft_node_flip_proxy(): hoisted above
 * ft_node_ptr(), so the tag-stripping helpers can assert against the tag. */

static inline_lookup
struct urcu_txn_record *ft_flip_proxy_ptr(struct cds_ft_inode_flag *node)
{
	return (struct urcu_txn_record *) _ft_node_mask_ptr(node);
}

/*
 * Resolve a possibly-proxied flag to its current target.  Sits right
 * after a parent / root pointer load on the read side; the common case
 * (no mutation in flight) is a single predicted-not-taken mask-compare
 * (ft_node_flip_proxy), and the parked-record deref is reached only during a
 * commit's brief install-to-settle window.  A parked record carries FT's own
 * 0xF tag (see URCU_TXN_PROXY_* in fractal-trie-internal.h), so this masks it
 * off and resolves the record through its MCAS status word.
 */
static inline_lookup
struct cds_ft_inode_flag *ft_resolve_flip_proxy(struct cds_ft_inode_flag *node)
{
	if (caa_unlikely(ft_node_flip_proxy(node))) {
		struct urcu_txn_record *r = ft_flip_proxy_ptr(node);

		return (struct cds_ft_inode_flag *) urcu_txn_resolve_record(r);
	}
	return node;
}

/*
 * A parent word, proxy-resolved, as a NODE -- NULL at a root.
 *
 * RESOLVE FIRST, THEN LAUNDER.  ft_parent_node() maps a root's trie stamp to
 * NULL but passes a parked flip proxy through untouched, so the reverse order
 * hands back whatever the record holds -- a trie stamp when the record moves
 * a node into or out of a root position -- and a stamp passes
 * ft_node_external() (its tag bits are 0), so the caller walks a struct
 * cds_ft as a node.  A stamp is never a proxy, so resolving first is the
 * identity for it.  (No test reaches a proxied stamp today; this is the
 * order that stays right when one does.)
 */
static inline_lookup
struct cds_ft_inode_flag *ft_parent_node_resolved(struct cds_ft_inode_flag *raw)
{
	return ft_parent_node(ft_resolve_flip_proxy(raw));
}

/*
 * Explicit acquire-load for child pointer dereference.
 *
 * Count-based readers (lookup_nth, skip, count_keys) need acquire
 * ordering on child pointer loads to pair with the writer's
 * rcu_assign_pointer (release) during removal.  This ensures that
 * if a reader sees a detached pointer, it also sees the preceding
 * nr_keys decrement (undercount guarantee on weakly-ordered
 * architectures).
 *
 * Current toolchains already compile rcu_dereference (CMM_CONSUME)
 * as CMM_ACQUIRE; this macro makes the acquire unconditional,
 * removing the dependency on the URCU_DEREFERENCE_USE_VOLATILE
 * escape hatch.
 */
static inline_lookup
void ft_maybe_prefetch(const void *ptr)
{
	/*
	 * Prefetch the RAW pointer without clearing the skip-compressed
	 * length high bits.  __builtin_prefetch doesn't fault on
	 * non-canonical addresses (it's a hint that silently drops invalid
	 * loads), so:
	 *   - clean child (~97% on dns): canonical -> prefetch fires with
	 *     zero added latency on the common path;
	 *   - skip-encoded child (~3%): non-canonical -> prefetch dropped.
	 *
	 * Clearing the bits first is a NET LOSS (measured on dns ft_specv,
	 * 2026-05-23): an unconditional mask (& 57-bit imm) and an
	 * unconditional double-shift were BOTH ~2% slower because the clear
	 * sits ahead of the prefetch in the dep chain and delays the
	 * common-case prefetch issue.  A raw-prefetch-then-conditional-clear
	 * shape keeps the common case fast but only TIES no-clear --
	 * prefetching the rare 3% skip children buys nothing measurable.
	 * So: prefetch raw, accept the dropped 3%.  Do NOT re-add the clear
	 * without a skip-heavy workload that shows a real win.
	 *
	 * Prefetch INTERNAL (tagged) children only.  External (leaf) children
	 * (tag bits clear) are random and use-once, and prefetching them is at
	 * best useless and at worst harmful (measured, EPYC 9654 / dns
	 * load-names):
	 *   - under 4 KiB leaf pages it is a no-op -- the leaf-arena TLB miss
	 *     drops the prefetch before its translation resolves;
	 *   - under 2 MiB leaf pages the translation resolves, so the prefetches
	 *     flood the memory controller (~5x more software-prefetch fills
	 *     reach DRAM) and inflate demand-load latency: -27% throughput.
	 * Removing it is neutral at 4 KiB (it was dropped anyway) and removes
	 * that 2 MiB foot-gun.  Internal nodes have descent locality + reuse and
	 * do not flood, so they keep a temporal prefetch.
	 */
	if (((unsigned long) ptr & FT_TAG_MASK) != 0)
		__builtin_prefetch(ptr);
}

/*
 * Non-temporal variant for stream-once spatial prefetch (the inequality
 * adjacent-sibling: a near-future iteration target read once, not reused like a
 * descent node).  prefetchnta fills with minimal cache-level allocation so the
 * streamed siblings do not evict the hot working set -- aimed at the extra LLC
 * traffic the temporal adjacent prefetch adds.  Same internal-only FT_TAG_MASK
 * guard (leaves are the random/use-once 2 MiB-page foot-gun -- see above).
 */
static inline_lookup
void ft_maybe_prefetch_nta(const void *ptr)
{
	if (((unsigned long) ptr & FT_TAG_MASK) != 0)
		__builtin_prefetch(ptr, 0, 0);
}

/*
 * ft_dereference_prefetch: for tagged FT node pointers.  Prefetches the
 * raw pointer via ft_maybe_prefetch, which does NOT clear the
 * skip-compressed length bits (see there: a skip-encoded pointer is
 * non-canonical and its prefetch is silently dropped -- keeping the
 * common-case prefetch un-delayed beats prefetching the rare skip child).
 *
 * ft_dereference_external: for external (leaf) pointers like
 * external_nodes.  external children are deliberately NOT prefetched (see
 * ft_maybe_prefetch: random/use-once leaves drop the prefetch on a 4 KiB TLB
 * miss or flood the memory controller under 2 MiB).  It also RESOLVES a parked
 * flip proxy: a "new key at an existing internal node" publish parks a proxy
 * in external_nodes so it commits atomically with the ordinal-cell splice (one
 * MCAS flip commit), so a reader loading external_nodes must resolve it to the
 * old/new head exactly as it does for a child slot.  The common case (a real
 * head, proxy tag clear) is a single predicted-not-taken tag test.
 */
#define ft_dereference_prefetch(p)		\
	({							\
		__typeof__(p) __ft_tmp = rcu_dereference(p);	\
		ft_maybe_prefetch(__ft_tmp);			\
		__ft_tmp;					\
	})

#define ft_dereference_external(p)					\
	((__typeof__(p)) ft_resolve_flip_proxy(				\
		(struct cds_ft_inode_flag *) rcu_dereference(p)))

#define ft_dereference_acquire_prefetch(p)	\
	({							\
		__typeof__(p) __ft_tmp =			\
			(__typeof__(p)) uatomic_load(&(p),	\
						     CMM_ACQUIRE); \
		ft_maybe_prefetch(__ft_tmp);			\
		__ft_tmp;					\
	})

#define ft_dereference_acquire(p)	\
	(__typeof__(p)) uatomic_load(&(p), CMM_ACQUIRE)

/*
 * Acquire-ordered external-head dereference that ALSO resolves a flip proxy.
 * The ordered-query / rank-select readers (cds_ft_lookup_nth and friends) load
 * metadata->external_nodes with acquire ordering -- to order it against the
 * metadata key counts they consume -- but a concurrent one-commit splice
 * (ft_insert_park_external_nodes) parks a flip proxy in that slot.  Resolve it
 * exactly as ft_dereference_external does for the consume-ordered descent
 * readers, else a tagged proxy would be mistaken for the external head.  The
 * common case (no splice in flight, tag clear) is one predicted-not-taken test.
 */
#define ft_dereference_external_acquire(p)				\
	((__typeof__(p)) ft_resolve_flip_proxy(				\
		(struct cds_ft_inode_flag *) ft_dereference_acquire(p)))

/*
 * Root-slot dereference for read-side descents: like
 * ft_dereference_*(ft->root) but additionally resolves a flip-proxy that
 * a cds_ft_merge_at commit may transiently install at the merge-point
 * forward slot (here, the root).  The resolved flag then flows into the
 * cached path, the key_len==0 / longest-match metadata access, and the
 * child dispatch.  The common case (no merge in flight) is a single
 * predicted-not-taken mask-compare in ft_resolve_flip_proxy.
 */
/*
 * Root-is-internal invariant enforcement.  No mutator ever publishes a
 * compressed / skip-compressed node at the trie root: a would-be compressed
 * root is re-internalized build-invisibly BEFORE publish (ft_make_root_internal_glue
 * on the re-root/graft-swap side; the compressed -> fresh-internal replace on the
 * detach side), so a reader must never observe one.  ft_root_assert_not_compressed()
 * (defined after the skip-compressed helpers below) asserts it on EVERY reader
 * root load -- after flip-proxy resolution, so a merge/graft proxy (which resolves
 * to an internal node in both commit phases) does not trip it -- catching a stray
 * compressed root at first observation rather than as downstream descent
 * corruption.  A NULL root (untagged) passes; compiled out under NDEBUG.
 */
#define ft_root_dereference_prefetch(ft)				\
	ft_root_assert_not_compressed(					\
		ft_resolve_flip_proxy(ft_dereference_prefetch((ft)->root)))
#define ft_root_dereference_acquire_prefetch(ft)			\
	ft_root_assert_not_compressed(					\
		ft_resolve_flip_proxy(ft_dereference_acquire_prefetch((ft)->root)))
#define ft_root_dereference(ft)						\
	ft_root_assert_not_compressed(					\
		ft_resolve_flip_proxy(rcu_dereference((ft)->root)))

/*
 * cn->child dereference for read-side descents: like
 * ft_dereference_acquire_prefetch(cn->child) but additionally resolves a
 * flip-proxy that a key-disappearing remove's recompaction (or external
 * promote) commit transiently installs at cn->child -- the forward edge it
 * flips together with the ordered-cell unsplice.  EVERY reader that descends
 * through a compressed node's child must resolve it; the common case (no such
 * remove in flight) is a single predicted-not-taken mask-compare.
 */
#define ft_cn_child_dereference_acquire_prefetch(cn)			\
	ft_resolve_flip_proxy(ft_dereference_acquire_prefetch((cn)->child))

/*
 * Per-caller prefetch hint for ft_node_get_nth_skip / ft_node_get_nth
 * and the underlying scanners.  Compile-time constant at each call
 * site -- the branches inside ft_maybe_prefetch_hint fold away, leaving
 * at most a single prefetch per caller.
 *
 *   FT_PF_NONE:        no prefetch.
 *   FT_PF_DATA:        prefetch child's data (node body).  Right for
 *                      candidate lookup and non-skip exact lookup
 *                      that traverse the returned child's data next.
 */
enum ft_pf_target {
	FT_PF_NONE,
	FT_PF_DATA,
};

static inline_lookup
void ft_maybe_prefetch_hint(const void *ptr, enum ft_pf_target hint)
{
	switch (hint) {
	case FT_PF_NONE:
		break;
	case FT_PF_DATA:
		ft_maybe_prefetch(ptr);
		break;
	}
}

#define ft_dereference_acquire_prefetch_hint(p, hint)			\
	({								\
		__typeof__(p) __ft_tmp =				\
			(__typeof__(p)) uatomic_load(&(p),		\
						     CMM_ACQUIRE);	\
		ft_maybe_prefetch_hint(__ft_tmp, (hint));		\
		__ft_tmp;						\
	})

#define ft_dereference_prefetch_hint(p, hint)				\
	({								\
		__typeof__(p) __ft_tmp = rcu_dereference(p);		\
		ft_maybe_prefetch_hint(__ft_tmp, (hint));		\
		__ft_tmp;						\
	})

/*
 * Return @node's external_nodes (the dup-chain head hanging off an
 * internal or compressed node).  @node must be internal or compressed.
 * Used to recover a cached iterator position's deepest trie node without
 * re-descending: a prefix key sits at an internal/compressed node
 * whose external_nodes == iter->node.
 */
static inline_lookup
struct cds_ft_node *ft_node_external_nodes(struct cds_ft_inode_flag *node)
{
	struct cds_ft_metadata *metadata;

	assert(!ft_node_external(node));
	if (ft_node_compressed(node))
		metadata = cds_ft_item_to_metadata(ft_node_ptr(node));
	else {
		const struct cds_ft_type *type = &ft_types[ft_node_type(node)];

		metadata = cds_ft_item_to_metadata_fast(ft_node_ptr(node),
				type->order);
	}
	return ft_dereference_external(metadata->external_nodes);
}

/*
 * Speculative inequality result-key capture: when the group is configured for
 * speculative skip-compressed lookup with a leaf-key offset, the matched leaf
 * @leaf stores the full result key -- in the byte order the application passed
 * to cds_ft_insert() -- at that offset.  Transform @level bytes of it to the
 * iterator's ordinal (trie) order into @dst and return true; the caller then
 * need not rebuild the key from the descent's compressed-node bytes.
 * ft_key_to_ordinals applies the group's key map, which is a plain copy for an
 * identity map and a per-byte remap otherwise, so the fast path covers
 * non-identity maps too (no identity restriction).  Returns false (copying
 * nothing) on groups without the offset / skip-compressed encoding, so the
 * caller falls back to the descent-built ordinal_key accumulation.
 *
 * This is the result-key source on a configured speculative group: the
 * min-descent intentionally leaves dispatch-irrelevant holes in ordinal_key
 * (it follows skip pointers without filling the spanned bytes), so the leaf
 * copy -- not ordinal_key -- carries the full result key.  Correctness is
 * validated end-to-end by the ordered-iteration / relational invariant tests.
 */
static inline_lookup
bool ft_speculative_keycopy(const struct cds_ft *ft,
		const struct cds_ft_node *leaf,
		uint8_t *dst, ssize_t level)
{
	const struct cds_ft_group *group = ft->group;
	const uint8_t *leaf_key;

	if (!leaf || level < 0)
		return false;
	/*
	 * Per-trie gate: a trie that opted out of speculative keys
	 * (cds_ft_attr_set_speculative_keys false) must NOT copy from the leaf's
	 * stored key here -- its leaves may hold a key that does not match their
	 * position -- so the caller falls back to the descent-built ordinal_key.
	 */
	if (!ft->speculative_key_offset_active || !group->speculative ||
			!(group->flags & CDS_FT_FLAG_SKIP_COMPRESSED))
		return false;
	leaf_key = (const uint8_t *) leaf + group->speculative_key_offset;
	ft_key_to_ordinals(dst, leaf_key, (size_t) level, &group->key_map);
	return true;
}

/*
 * Unconditional variant of ft_speculative_keycopy for callers that have already
 * resolved use_keycopy at compile time: the leaf-copy config gate
 * (speculative_key_offset_set && speculative && SKIP_COMPRESSED) is then known
 * to hold, so the runtime re-check folds away.  Copies @level ordinal bytes of
 * @leaf's stored key into @dst; a NULL @leaf (NOT_FOUND) or @level < 0 copies
 * nothing (the result key is then unused).
 */
static inline_lookup
void ft_speculative_keycopy_unconditional(const struct cds_ft *ft,
		const struct cds_ft_node *leaf, uint8_t *dst, ssize_t level)
{
	const struct cds_ft_group *group = ft->group;
	const uint8_t *leaf_key;

	if (!leaf || level < 0)
		return;
	leaf_key = (const uint8_t *) leaf + group->speculative_key_offset;
	ft_key_to_ordinals(dst, leaf_key, (size_t) level, &group->key_map);
}

/*
 * Ordinal-cell tag + accessors (cell-always model).
 *
 * Every duplicate-chain HEAD has a library-owned ordinal cell (struct
 * ft_ord_cell), and the head's cds_ft_node.prev points to it.  The head's
 * flagged parent is relocated into ft_ord_cell.parent; the cell is the
 * external head's metadata record, peer to internal/compressed metadata.
 *
 * The cell pointer is tagged with FT_INTERNAL_MASK (bit 0) so the head-vs-dup
 * test ft_node_external(prev)==false is preserved (a non-head dup's prev is an
 * untagged external cds_ft_node, bits 0-1 == 0).  Bit 0 is the ONLY tag: in a
 * cell build a head's prev is ALWAYS a cell, so there is nothing to
 * distinguish and no per-pointer marker is needed (32-bit safe).  Cells are
 * >= 2-byte aligned, so bit 0 is free.
 *
 * The DOWNWARD child slots still point straight at the external node; the cell
 * is interposed only on the UPWARD walk (parent recovery) and ordered
 * traversal.  Every reader of a head's prev-as-parent resolves through
 * ft_resolve_head_prev (identity outside the feature).
 */
#define FT_ORD_CELL_TAG		FT_INTERNAL_MASK

#include "ft-lookup-helpers.h"

/*
 * ft_node_holder: write-side resolution of a node's holder (the slot owner
 * "above" it), independent of the cell relocation.
 *
 *   - non-head duplicate: prev is the predecessor cds_ft_node (external).
 *   - head: prev is the flagged parent directly (non-cell build) or the
 *     cell whose ->parent holds the flagged parent (cell build).
 *   - never-inserted (prev NULL): returns NULL.
 *
 * Mutex-held callers (remove / replace / locate-chain-head) that previously
 * read node->prev as the holder route through this so the cell indirection
 * is transparent.  Identity in non-cell builds.
 */
#if defined(FT_ENABLE_TRACING) || defined(FT_DEBUG_RM_SITE)
/*
 * WHICH refusal.  cds_ft_remove has six distinct "not found" exits and the
 * status code cannot tell them apart -- and a refusal is a CLAIM ABOUT THE
 * TRIE, not a return value: an idempotent miss and a derivation that lost its
 * holder to a peer report the same thing to the caller.  Each site stamps its
 * own __LINE__ here and the violation event carries it.
 */
static __thread unsigned int ft_dbg_rm_site;
#endif

static inline
struct cds_ft_inode_flag *ft_node_holder(struct cds_ft *ft,
		const struct cds_ft_node *node)
{
	/*
	 * Resolve a parked flip proxy at the load: once a head-promote folds
	 * its prev-inherit store onto the commit flip-txn (Phase 4.3), this
	 * word transiently carries FT's type-7 proxy, and a raw external/cell
	 * classification of the proxy value would mis-derive the holder.
	 */
	void *prev = (void *) ft_resolve_flip_proxy((struct cds_ft_inode_flag *)
			rcu_dereference(((struct cds_ft_node *) node)->prev));

	if (ft_node_external((struct cds_ft_inode_flag *) prev))
		return (struct cds_ft_inode_flag *) prev;
	return ft_resolve_head_prev(ft, prev);
}

/*
 * ft_chain_head_is_removed: is @node's CHAIN retired, as opposed to @node
 * itself?  Walks prev to the head (the same walk ft_chain_head_holder does) and
 * asks ft_node_is_removed of THAT.
 *
 * ★ WHY A CHAIN NEEDS A LIVENESS ANSWER ITS MEMBERS CANNOT GIVE.  A whole-chain
 * displacement -- cds_ft_insert_replace swinging the anchor slot to a fresh head
 * -- retires every node at once, but a txn is bounded and a chain is not, so
 * only the HEAD's freeze can ride the displacing commit
 * (ft_hlist_freeze_prepare).  The members are therefore RETIRED BUT UNMARKED,
 * and ft_node_is_removed -- a per-node test of node->next -- answers "live" for
 * every one of them.  An op that believes it goes on to unlink a ghost: its
 * @pred comes from a stale prev, and the `pred->next: elem -> next` edge it
 * records can never match, which is a livelock, not an error.
 *
 * ☠ MARKING THE MEMBERS INSTEAD IS THE WRONG CURE, and was tried: a post-commit
 * sweep is a BARE CAS by a non-owner on cds_ft_node.next, a word the word-kind
 * table gives to the chain HOLDER.  It lands underneath ops that already
 * validated and turns their loaded next into MARK(NULL) -- the bare value 2 --
 * so a backward edge records slot &((struct cds_ft_node *) 2)->prev.  Measured
 * 8/8 SEGV.  Asking the head costs a walk the caller is already making and
 * writes nothing.
 *
 * HOLDER-LOCK CALLERS ONLY, exactly as ft_chain_head_holder: the prev walk is
 * not stable under a concurrent relink.  Returns false for a never-inserted
 * node (prev NULL).
 */
static inline
bool ft_chain_head_is_removed(struct cds_ft_node *node)
{
	struct cds_ft_node *cur = node;

	for (;;) {
		void *prev = (void *) ft_resolve_flip_proxy(
			(struct cds_ft_inode_flag *)
			rcu_dereference(cur->prev));

		if (!prev)
			return false;
		if (!ft_node_external((struct cds_ft_inode_flag *) prev))
			break;			/* @cur is the head */
		cur = (struct cds_ft_node *) prev;
	}
	return ft_node_is_removed(cur);
}

/*
 * ft_chain_head_holder: resolve the trie HOLDER (the head's IMMEDIATE PARENT)
 * of @node's duplicate chain -- the single lockable state-word node every op on
 * the chain serialises on (MW LOCK_FINE holder lock).  @node may be the head
 * (its prev is the cell / flagged parent -> resolve directly, one iteration) or
 * an interior duplicate (its prev is a predecessor external -> walk prev up to
 * the head, whose prev is NOT external).  A flip proxy parked on prev mid-commit
 * resolves at each load, as in ft_node_holder.  HOLDER-LOCK / mutex-held callers
 * only: the prev walk is not stable under a concurrent chain relink (an interior
 * op can only find its head's holder to lock while some coarser exclusion --
 * today the FT-wide writer_lock -- still holds; that chicken-and-egg is resolved
 * with the FT-wide-lock drop).  Returns NULL for a never-inserted node (prev
 * NULL).
 */
static inline
struct cds_ft_inode_flag *ft_chain_head_holder(struct cds_ft *ft,
		struct cds_ft_node *node)
{
	struct cds_ft_node *cur = node;

	for (;;) {
		void *prev = (void *) ft_resolve_flip_proxy(
			(struct cds_ft_inode_flag *)
			rcu_dereference(cur->prev));

		if (!prev)
			return NULL;
		if (!ft_node_external((struct cds_ft_inode_flag *) prev))
			return ft_resolve_head_prev(ft, prev);
		cur = (struct cds_ft_node *) prev;
	}
}

/*
 * ft_head_parent_word_slot: the generic re-parent form -- ft_set_parent and its
 * two transacted twins, which every restructure funnels its child re-homes
 * through, and which already carry the answer in @slot.
 *
 * ★ @slot IS THE SHAPE, so no site has to be told it twice.  @slot is the
 * address of the word in @parent_nf that holds the head:
 *
 *   - &parent_meta->external_nodes: a PREFIX head, key = path(parent), the key
 *     ENDS at the parent.  The rekey S_top COW, the remove head-slot resolver
 *     and the insert replace arms all name that word explicitly.
 *   - NULL: also a prefix head.  ft-remove.h's head-slot resolver spells the
 *     convention out in as many words ("external_nodes, not a body slot"), and
 *     every re-parent that hands a head to its new holder as external_nodes
 *     passes NULL: the insert attach and its cluster-leaf branch
 *     (ic->live_slot), the merge's M_ext, the graft's displaced external.
 *   - any other word: a BODY SLOT, so a slot head, key = path(parent) + its
 *     own edge byte.
 *
 * The three NULL-slot sites with a NON-external child (a junction, a branch, a
 * compressed node's own child) never reach here: this form is consulted only on
 * the external branch of those dispatchers, and a compressed @parent_nf drops
 * the bit by construction anyway.
 */
static inline
struct cds_ft_inode_flag *ft_head_parent_word_slot(
		struct cds_ft_inode_flag *parent_nf,
		struct cds_ft_inode_flag **slot)
{
	bool prefix = true;

	if (slot && ((uintptr_t) parent_nf & FT_INTERNAL_MASK) &&
	    !ft_node_flip_proxy(parent_nf))
		prefix = ((void *) slot == (void *)
			&cds_ft_item_to_metadata(ft_node_ptr(parent_nf))->external_nodes);
	return ft_head_parent_word(parent_nf, prefix);
}

/*
 * ft_head_is_prefix: the head's OWN answer to the shape question -- is it the
 * prefix head (external_nodes) of the parent its back-edge word names?  Read
 * from that one word in both modes (cell->parent list-on, prev list-off), so
 * it stays consistent with the pointer it rides on; a parked flip proxy is
 * resolved first, its bit 4 being address.
 *
 * Write-side use: a REPLACE installs a new head in the old one's place and
 * must inherit the shape rather than re-derive it from a holder mid-op.
 */
static inline
bool ft_head_is_prefix(const struct cds_ft *ft, struct cds_ft_node *head)
{
	void *prev;

	if (!head)
		return false;
	prev = ft_dereference_prev_resolved(head);
	if (!ft->ordered_list)
		return ft_parent_prefix_head((struct cds_ft_inode_flag *) prev);
	return ft_parent_prefix_head(ft_resolve_flip_proxy(
		rcu_dereference(ft_ord_cell_ptr(prev)->parent)));
}

/*
 * Record a fresh head's flagged parent.  Non-cell builds store it directly
 * into the (pre-publish) head's prev; cell builds store it into the head's
 * pre-wired cell (node->prev already carries the cell), leaving prev intact.
 * For the fresh-head wiring sites only (the subsequent forward publish
 * orders this store); existing-head re-parents use ft_set_parent / the
 * external-nodes choke point, which resolve the cell themselves.
 *
 * @prefix says which SHAPE the head is being wired into: true when it is
 * becoming @parent's external_nodes (its key ENDS at @parent), false for a
 * body slot.  The word carries that answer from here on -- see
 * FT_PARENT_PREFIX_HEAD -- so a site that gets it wrong is caught at rest by
 * cds_ft_verify rather than as a rare wrong-node lookup.
 */
#define ft_external_head_set_parent(ft, node, parent, prefix)		\
	do {								\
		struct cds_ft_inode_flag *_hpw = ft_head_parent_word(	\
			(struct cds_ft_inode_flag *) (parent), (prefix)); \
									\
		/*						\
		 * The [debt] HEAD-WORD class, same as			\
		 * ft_publish_external_nodes_prev and ft_set_parent's	\
		 * external arm -- and this spelling had NO audit arm, so	\
		 * it could not appear in the class's zeros either way.	\
		 *						\
		 * ☞ DECLARED HIDDEN, AND THAT IS A CHECKED CLAIM here,	\
		 * not a believed one: every caller of this macro writes	\
		 * the back edge of a FRESH node it is still building --	\
		 * `node->next = NULL` follows immediately and the	\
		 * external_nodes publish is parked for a later commit --	\
		 * so no reader can reach the word.  The audit's HIDDEN	\
		 * arm scores that against ft_ch_head_reachable, whose	\
		 * positive control fires in the same run (HEAD-CONTROL	\
		 * live removed leaf, reachable 3,514,638 of 3,514,798),	\
		 * so a wrong claim here reads as hw_hidden_live.	\
		 * UNDECLARED was the wrong spelling: with a non-NULL	\
		 * owner the audit counts it and returns WITHOUT scoring,	\
		 * which left ~4M writes per leg visible but unverdicted.	\
		 */						\
		ft_ch_audit_head_at(__func__, __LINE__, (ft), (node),	\
			(struct cds_ft_inode_flag *) (parent),		\
			FT_EXCL_HIDDEN);				\
		if ((ft)->ordered_list)					\
			ft_ord_cell_set_parent((node), _hpw);		\
		else							\
			(node)->prev = _hpw;				\
	} while (0)

/*
 * ft_publish_external_nodes_prev: Phase 2 -- publish the back-channel pointer
 * up from the displaced/transferred external head @external_nodes to its
 * (re-)parent @node_flag via rcu_assign_pointer.
 *
 * Call AFTER node_flag's own parent is wired (so an up-walker arriving via
 * the new back-channel lands on a parent-wired cluster top, not a NULL
 * parent), and at-or-just-before the forward publish that makes the cluster
 * reachable through node_flag's slot.  No-op when @external_nodes is NULL
 * (callers commonly guard on metadata->external_nodes).
 *
 * Ordered list on: @external_nodes is an existing head, so its prev already
 * carries its cell; record the new parent into cell->parent (the head's
 * prev -- the cell pointer -- is unchanged).  All choke-point callers
 * re-parent an existing head (a fresh head's parent is wired by ft_set_parent
 * via ft_node_set_nth), so the cell is guaranteed present.  List off / non-cell:
 * the head's prev IS the flagged parent, so re-parent it directly.
 */
static inline
void ft_publish_external_nodes_prev(struct cds_ft *ft,
		struct cds_ft_inode_flag *node_flag,
		struct cds_ft_node *external_nodes)
{
	struct cds_ft_inode_flag *word;

	if (!external_nodes)
		return;
	/*
	 * By definition of this helper @external_nodes is @node_flag's
	 * external_nodes, so the word it publishes is a PREFIX-HEAD one: the
	 * head's key ENDS at @node_flag and no edge byte separates them
	 * (FT_PARENT_PREFIX_HEAD).  Every caller pairs this with the matching
	 * ft_metadata_set_external_nodes.
	 */
	word = ft_head_parent_word(node_flag, /*prefix=*/ true);
	/*
	 * ☞ THE SECOND [debt] WORD CLASS, and this site writes BOTH spellings
	 * of it: ft_ord_cell.parent (list on) and cds_ft_node.prev of a HEAD
	 * (list off).  Owner is "the holder P" in the word-kind table -- the
	 * node this head hangs under -- which is exactly what
	 * ft_chain_head_holder derives, so the chain audit's predicate answers
	 * for it unchanged.  Both stores are RAW rcu_assign_pointer with no txn
	 * ("MW + raw" in the table), so nothing can abort them: if the holder is
	 * not held here, the write lands regardless.
	 */
	ft_ch_audit_head(ft, external_nodes, node_flag);
	if (ft->ordered_list) {
		ft_ord_cell_set_parent(external_nodes, word);
	} else {
		FT_CHAIN_CANARY_RAW(&external_nodes->prev, 1);
		rcu_assign_pointer(external_nodes->prev, word);
	}
	FT_TP(set_parent, (const void *) external_nodes, (const void *) word);
}

static
struct cds_ft_inode_flag *ft_compressed_node_flag(
		struct cds_ft_compressed_node *node)
{
	return (struct cds_ft_inode_flag *)
		(((unsigned long) node) | FT_COMPRESSED_MASK);
}

static inline_lookup
struct cds_ft_compressed_node *ft_compressed_node_ptr(
		struct cds_ft_inode_flag *node)
{
	ft_assert_resolved(node);
	return (struct cds_ft_compressed_node *)
		(((unsigned long) node) & ~(unsigned long) FT_TAG_MASK);
}

#ifdef FT_DEBUG_PARENT_VIOLATION
#include <stdio.h>
/*
 * The same violation the ordered up-walk dumps (ft-iter.h), at the OTHER load
 * that carries the invariant.  A bare assert here names only the READER -- the
 * thread that followed the link -- while the question is about the node whose
 * @parent_word produced the illegal value, and nothing on this frame survives
 * into a core to name it at -O1.
 *
 * @node is that node, i.e. the one suspected of having been retired and freed
 * while still referenced.  Print its metadata, state and rcu_head words at the
 * point of detection instead of reconstructing them afterwards.
 */
__attribute__((noinline, cold, unused))
static void ft_parent_rcu_violation(struct cds_ft *ft,
		struct cds_ft_inode_flag *node,
		struct cds_ft_inode_flag *bad)
{
	struct cds_ft_metadata *m = NULL;

	if (!ft_node_external(node))
		m = ft_node_compressed(node) ?
			cds_ft_item_to_metadata((struct cds_ft_inode *)
				ft_compressed_node_ptr(node)) :
			cds_ft_item_to_metadata(ft_node_ptr(node));
	fprintf(stderr, "FT_PARENT_RCU_VIOLATION: illegal parent %p\n",
		(void *) bad);
	fprintf(stderr, "  ft=%p node=%p meta=%p\n",
		(void *) ft, (void *) node, (void *) m);
	if (m) {
		fprintf(stderr, "  node parent_word=%p state=0x%lx\n",
			(void *) m->parent_word, (unsigned long) m->state);
		fprintf(stderr, "  node rcu_head words: %p %p\n",
			((void **) m)[-2], ((void **) m)[-1]);
	}
	fprintf(stderr, "  bad rcu_head words: %p %p\n",
		((void **) bad)[0], ((void **) bad)[1]);
	fflush(stderr);
	abort();
}
# define ft_parent_rcu_check(ft, node, parent)				\
	do {								\
		if (caa_unlikely((parent) && ft_node_external(parent)))	\
			ft_parent_rcu_violation((ft), (node), (parent));	\
	} while (0)
#else
# define ft_parent_rcu_check(ft, node, parent)				\
	do {								\
		(void) (node);						\
		assert(!(parent) || !ft_node_external(parent));		\
	} while (0)
#endif

/*
 * ft_get_parent_rcu: read the parent pointer of @node via
 * rcu_dereference.
 *
 * For external (leaf) nodes: returns cds_ft_node.prev.  @node must
 * be the head of its duplicate chain (non-head duplicates' prev
 * points to the preceding node in the chain, not to the parent).
 * Iterators and lookups maintain this invariant by convention --
 * iter->node always refers to the chain head.
 *
 * For internal/compressed nodes: returns metadata->parent.
 *
 * Returns NULL when @node is at the root position, or when @node
 * has been orphaned by a concurrent detach / graft_swap that
 * cleared its parent link.  A read-side parent-pointer walk that
 * observes NULL terminates cleanly in either case.
 *
 * Read-side safe; the caller must be in an RCU read-side critical
 * section (or QSBR equivalent).
 *
 * An assertion verifies the returned parent is never external: an
 * external result would mean the caller passed a non-head duplicate
 * chain entry (whose prev points at the preceding duplicate, not
 * at the parent).
 *
 * Config-agnostic (only the flip-proxy resolve, a merge primitive,
 * and basic accessors): kept here, ahead of the FEATURE_FT_SKIP_COMPRESSED
 * block, so cds_ft_merge_at can use it in all build configs.
 */
/*
 * Member-to-head hops an up-walk tolerates before declaring the chain
 * pathological.  A duplicate chain is unbounded; a cycle is not a shape any
 * writer publishes, so this is a defensive bound, not a design limit.
 */
#define FT_EXT_CHAIN_HOPS_MAX	(1UL << 20)

/*
 * ft_ext_head_word: from an external @ext, the RAW parent word of its chain's
 * HEAD (the prefix-head bit kept, a parked flip proxy resolved at the load) --
 * NULL for a never-inserted node, or a chain the hop bound gave up on.
 * *@head_ret (optional) is the head itself.
 *
 * ☠ AN UP-WALK MAY START ON A MEMBER, NOT A HEAD.  A skip word names a head at
 * the instant it is encoded, and every up-walk from an external assumed it
 * still was one.  cds_ft_rekey_merge SPLICES a moved head onto the
 * destination's duplicate chain, where it is an interior member whose prev is
 * its PREDECESSOR NODE; a reader still on the retired copy of the source
 * junction follows that copy's stale skip word straight to it, and reading
 * the predecessor as a parent word decoded it as a cell (list mode) or as an
 * internal, and walked into a NULL parent: ft_skip_reanchor's assert, in a
 * reader thread, on every served merge whose source head is a skip target.
 * The discriminator is the chain's own (ft_chain_head_holder): a head's prev
 * is a cell or a flagged internal, never an external; a member's prev is an
 * external.  Hop to the head first.  A member and its head carry the SAME
 * key, so the head's position is the member's; a MOVED head's chain runs up
 * the destination path, and the descent that continues from there is a torn
 * pass the two-descent witness discards -- exactly what it already does for a
 * grafted internal node.  Every prev load resolves a parked record: a prev
 * record carries the flip-proxy tag (FT_HLIST_PREV_TAG), so the one resolver
 * covers the head's word and a member's alike, and no parked value is
 * bit-identical to a cell.
 */
/*
 * @count: bump the DEBUG_COUNTERS member-hop counter when a hop happened.  ONLY
 * the read-side skip re-anchor passes true: the counter is the same-parent
 * splice oracle's positive control ("a READER walked up from a spliced
 * member"), and the write path takes these up-walks too (ft_reanchor_flag,
 * the count walk, the builders' ft_skip_to_compressed), whose hops would make
 * a non-zero count prove nothing about readers.
 */
static inline
struct cds_ft_inode_flag *ft_ext_head_word(const struct cds_ft *ft,
		struct cds_ft_node *ext, struct cds_ft_node **head_ret, bool count)
{
	struct cds_ft_node *cur = ext;
	void *prev = ft_dereference_prev_resolved(cur);
	unsigned long hops = 0;

	while (prev && ft_node_external((struct cds_ft_inode_flag *) prev)) {
		if (++hops > FT_EXT_CHAIN_HOPS_MAX) {
			prev = NULL;
			break;
		}
		cur = (struct cds_ft_node *) prev;
		prev = ft_dereference_prev_resolved(cur);
	}
#ifdef DEBUG_COUNTERS
	if (hops && count)
		uatomic_inc(&ft->group->nr_ext_member_hops);
#else
	(void) count;
#endif
	if (head_ret)
		*head_ret = cur;
	if (!prev)
		return NULL;
	if (ft->ordered_list)
		return ft_resolve_flip_proxy(rcu_dereference(
			ft_ord_cell_ptr(prev)->parent));
	return (struct cds_ft_inode_flag *) prev;
}

static inline
struct cds_ft_inode_flag *ft_get_parent_rcu(struct cds_ft *ft,
		struct cds_ft_inode_flag *node)
{
	struct cds_ft_inode_flag *parent;

	if (ft_node_external(node))
		parent = ft_parent_prefix_strip(ft_ext_head_word(ft,
			(struct cds_ft_node *) node, NULL, false));
	else if (ft_node_compressed(node))
		/*
		 * A compressed node's metadata lives at a FT_TAG_MASK-cleared
		 * offset, not the FT_TYPE_MASK-cleared one ft_node_ptr uses; a
		 * referenced compressed child re-parented by a merge reaches
		 * here (after the caller resolves any skip form to its raw
		 * compressed flag).
		 */
		parent = ft_parent_node(rcu_dereference(cds_ft_item_to_metadata(
			(struct cds_ft_inode *) ft_compressed_node_ptr(node))->parent_word));
	else
		parent = ft_parent_node(rcu_dereference(cds_ft_item_to_metadata(
			ft_node_ptr(node))->parent_word));
	/*
	 * The parent slot may transiently hold a flip-proxy during a
	 * cds_ft_merge_at commit; resolve it to the view-appropriate
	 * (old or merged) parent before returning.
	 */
	parent = ft_resolve_flip_proxy(parent);
#ifdef FT_ENABLE_TRACING
	/*
	 * FIRE AT DETECTION, not from a signal handler: the prior rig recorded
	 * that a SIGSEGV-handler snapshot lands long after the ring has wrapped.
	 * This is the same instant the assert below would abort on, but with the
	 * window still intact.
	 *
	 * A freelist link is 8-mod-16, so it clears the EXTERNAL tag -- which is
	 * exactly why a reclaimed parent reads as an external node here.  The
	 * round trip (item -> metadata -> item) is the discriminator: equal means
	 * a valid live object with a wrong link, unequal means recycled memory.
	 */
	/*
	 * POSITIVE CONTROL for the whole emit -> freeze -> stop -> snapshot ->
	 * decode chain.  A violation site that has never been shown to LAND in a
	 * readable trace is an instrument on trust, and this one has already
	 * produced two aborts whose snapshots contained no violation event.
	 * FT_TRACE_SELFTEST=1 fires the identical sequence on the first call,
	 * from a state that is perfectly healthy, so a snapshot WITHOUT the event
	 * indicts the rig and one WITH it clears the rig.
	 */
	{
		static int selftest = -1;

		if (caa_unlikely(selftest < 0))
			selftest = getenv("FT_TRACE_SELFTEST") ? 1 : 0;
		if (caa_unlikely(selftest == 1)) {
			selftest = 0;
			FT_TP(parent_external_violation, node, parent, parent,
				0xdeadbeefUL);
			ft_trace_capture();
			fprintf(stderr, "FT_TRACE_SELFTEST: fired\n");
			abort();
		}
	}
	if (caa_unlikely(parent && ft_node_external(parent))) {
		const void *pp = ft_node_ptr(parent);
		const void *rt = NULL;

		rt = cds_ft_metadata_to_item(cds_ft_item_to_metadata(
			(struct cds_ft_inode *) pp));
		FT_TP(parent_external_violation, node, pp, rt,
			(unsigned long) rcu_dereference(cds_ft_item_to_metadata(
				ft_node_ptr(node))->parent_word));
		/*
		 * STOP FIRST, then snapshot.  system() is a fork+exec costing
		 * milliseconds, and this workload emits millions of events per
		 * second -- the 64 KiB ring wraps several times over during it,
		 * so a plain "snapshot record" here dumps a window that no
		 * longer contains the violation that triggered it.  Measured:
		 * the snapshot was written and the event was already gone.
		 * lttng stop freezes every buffer before the dump, at the cost
		 * of ending tracing for the run -- which is exactly what we
		 * want, since this process is about to abort anyway.
		 */
		ft_trace_capture();
		abort();
	}
#endif
	ft_parent_rcu_check(ft, node, parent);
	return parent;
}

/* Skip-compressed pointer helpers. */

#ifdef FEATURE_FT_SKIP_COMPRESSED
static inline
bool ft_node_skip_compressed(struct cds_ft_inode_flag *node)
{
	return ((unsigned long) node >> FT_SKIP_LEN_SHIFT) != 0;
}

static inline
unsigned int ft_skip_len(struct cds_ft_inode_flag *node)
{
	return (unsigned long) node >> FT_SKIP_LEN_SHIFT;
}

/*
 * ft_skip_child_ptr: extract the child tagged pointer from a skip
 * pointer by clearing the skip-length bits.
 */
static inline
struct cds_ft_inode_flag *ft_skip_child_ptr(struct cds_ft_inode_flag *node)
{
	ft_assert_resolved(node);
	return (struct cds_ft_inode_flag *) ((unsigned long) node & FT_ADDR_MASK);
}

/*
 * ft_skip_compressed_flag: encode a skip pointer from a child pointer
 * and the compressed path length.
 *
 * ☞ THE VALUE THIS BUILDS LIVES IN THE GRANDPARENT'S BODY, and the register in
 * fractal-trie-internal.h lists that word as GP-owned, recorded MW by every
 * producer (class DUAL) -- MW as DEBT, because the owner is DERIVED here from
 * cn's back-pointer and only the op can vouch for holding it.
 *
 * ☠ ONE WORD, TWO ROLES.  To an op that holds GP and republishes the slot as
 * an ordinary FORWARD edge, this is a structural slot of GP and PARKS SW when
 * armed; to every dual refresh it is a CAS.  SW xor MW is a per-slot,
 * cross-thread invariant, so a dual refresh that does not hold GP races that
 * park -- hold GP (ft_lock_skip_dual_gp) before recording here, and remember
 * that "the recompact took {C,P,GP}" is a PLAN-TIME claim while this owner is
 * derived at PUBLISH time.
 */
static
struct cds_ft_inode_flag *ft_skip_compressed_flag(
		struct cds_ft_inode_flag *child, unsigned int len)
{
	assert(len > 0 && len <= FT_SKIP_LEN_MAX);
	/*
	 * The encoding ORs len into the high bits of child.  If child
	 * already carries skip-length bits (i.e., is itself a skip-
	 * compressed pointer), the OR conflicts with len and produces
	 * a corrupted nested encoding from which neither len nor child
	 * can be recovered cleanly.  Chain-compress canonicalization
	 * is responsible for ensuring that cn->child is never skip-
	 * compressed at publish time (the "no two adjacent compresseds"
	 * invariant).  Assert the invariant here so any future regression
	 * fails loudly under -UNDEBUG smoke tests rather than silently
	 * corrupting the trie.
	 */
	assert(((unsigned long) child >> FT_SKIP_LEN_SHIFT) == 0);
	return (struct cds_ft_inode_flag *)
		((unsigned long) child |
		 ((unsigned long) len << FT_SKIP_LEN_SHIFT));
}

/*
 * ft_skip_to_compressed: recover the compressed node from a skip
 * pointer by following the child's parent back-pointer.
 *
 * For internal/compressed children: uses metadata->parent.
 * For external (leaf) children: uses cds_ft_node.prev (which points
 * to the parent for the head of a duplicate chain).
 *
 * No validation against the slot's skip_len: callers (including
 * writers in mid-mutation, where the back-pointer and slot value
 * are intentionally inconsistent for a brief window) get whatever
 * the back-pointer currently says.  Reader paths that must observe
 * a self-consistent slot+cn pair re-anchor via ft_skip_reanchor,
 * which walks the skip child's live parent chain to the trie
 * position the slot's skip_len encodes.
 *
 * Read-side safe (rcu_dereference on both fields).  Callers must be
 * in an RCU read-side critical section (or QSBR equivalent).
 */
static inline
struct cds_ft_compressed_node *ft_skip_to_compressed(const struct cds_ft *ft,
		struct cds_ft_inode_flag *skip_ptr)
{
	struct cds_ft_inode_flag *child = ft_skip_child_ptr(skip_ptr);
	struct cds_ft_inode_flag *parent;

	if (ft_node_external(child))
		/*
		 * @child is a head; resolve its flagged parent through
		 * ft_resolve_head_prev, which branches on the group's ordered_list
		 * mode (cell-indirect vs prev-direct).  A structural tag test is NOT
		 * usable here: a skip pointer with a STALE target (a concurrent
		 * split/merge moved the encoded position) can transiently make this
		 * head's parent an internal node -- FT_INTERNAL_MASK, bit 0, the same
		 * tag a cell carries -- so only the mode flag disambiguates safely.
		 */
		parent = ft_parent_prefix_strip(ft_ext_head_word(ft,
			(struct cds_ft_node *) child, NULL, false));
	else
		parent = ft_parent_node(rcu_dereference(cds_ft_item_to_metadata(
			ft_node_ptr(child))->parent_word));
	/*
	 * The recovered back-pointer may transiently be a flip proxy during a
	 * cds_ft_merge_at commit -- a dst-origin subtree root wrapped under a
	 * freshly-built compressed node in the merged cluster has its parent
	 * staged through the latch.  Resolve it before masking to the compressed
	 * node (otherwise the proxy's tag bits fold into a bogus pointer),
	 * mirroring ft_get_parent_rcu and the descent's resolve-then-skip order.
	 * A concurrent reader up-walking through the merge's flip window resolves
	 * via the shared selector to the consistent old-or-merged view; a no-op
	 * when no merge is in flight.
	 */
	return ft_compressed_node_ptr(ft_resolve_flip_proxy(parent));
}


/*
 * ft_upwalk_edge_bytes: the key bytes an up-walk consumes on the hop from @cur
 * to its INTERNAL parent.
 *
 * One byte for a child in a branch slot -- and ZERO for a PREFIX head: an
 * external head hanging at the parent's external_nodes, whose key ENDS at the
 * parent, so it sits AT the parent's position and no edge byte separates them.
 *
 * ★ THE HEAD ANSWERS FOR ITSELF, from @head_parent_word -- the very word the
 * caller loaded to find the parent (FT_PARENT_PREFIX_HEAD).  Parent and shape
 * therefore come from ONE load, in BOTH list modes, and cannot disagree.
 *
 * ☠ What this replaces was the holder's LIVE external_nodes, and that answer
 * is DESTROYED IN PLACE by the very removes an up-walk must survive: a
 * key-disappearing remove of a prefix head clears the holder's external_nodes
 * while a parked reader still holds the head, and a replace leaves it naming a
 * DIFFERENT head.  The dead prefix head then read as a SLOT head, was charged a
 * byte it never consumed, and the descent resumed one level off -- a present
 * key answering NOT_FOUND, and a key ABSENT at every instant answering with a
 * NEIGHBOUR'S NODE.  The ordered list could patch it with a mark on the retired
 * cell; list-off had no cell and no patch, which is why the answer had to move
 * onto the head's own word.
 *
 * A skip slot never NAMES a prefix head at rest (it names the compressed node's
 * child, whose parent is that compressed node), so a non-zero answer here is
 * reached only through a STALE slot whose target a concurrent split has
 * re-homed as a fresh internal node's prefix head -- exactly the case
 * ft_skip_reanchor exists to resolve.
 *
 * Only an EXTERNAL @cur can be a prefix head, so an internal / compressed @cur
 * takes the constant-1 arm without consulting the word at all.
 */
static inline
unsigned int ft_upwalk_edge_bytes(struct cds_ft_inode_flag *cur,
		struct cds_ft_inode_flag *head_parent_word)
{
	if (!ft_node_external(cur))
		return 1U;
	return ft_parent_prefix_head(head_parent_word) ? 0U : 1U;
}

/*
 * ft_skip_reanchor: the single concurrency-handling mechanism for skip-
 * compressed pointers.  A skip slot encodes a length (skip_len), but the live
 * compressed node recovered via the skip child's back-pointer may no longer
 * match it (a concurrent split/merge changed the path between the slot and the
 * child), so readers resolve the slot by re-anchoring rather than trusting the
 * recovered node directly.
 *
 * Spinning to re-read the slot does NOT converge when the slot lives on a node
 * that was recompacted away (frozen-stale) while the skip child was reparented
 * to a different-length compressed by a concurrent split/merge: the frozen
 * slot is never republished, so the reader would loop forever.  The skip child
 * @G, however, is reachable in the LIVE trie (a live leaf or live internal), so
 * its parent chain runs through live nodes that converge.  Walk it up,
 * accumulating consumed path length (a compressed spans its len, an internal
 * one byte -- or NONE when the child is that internal node's PREFIX head, see
 * ft_upwalk_edge_bytes), until the accumulated length reaches the slot's
 * skip_len: that locates the live tree position the failing slot encoded.
 *
 *   - split (live path lengthened into prefix+branch+suffix at the same total
 *     length): the accumulation lands exactly, @*rewind == 0; re-anchor at the
 *     live node at the same depth.
 *   - merge (live path shortened by absorbing the slot's level into a longer
 *     compressed): the first hop already exceeds skip_len; the encoded position
 *     is now interior to that compressed.  Re-anchor shallower (its parent) and
 *     have the caller rewind its descent cursor by @*rewind bytes.
 *
 * @skip_ptr: the failing skip pointer (encodes child @G + skip_len).
 * @rewind:   out -- bytes the caller must back its descent cursor/level up by.
 * @at_pos:   out (may be NULL) -- the live node spanning/at the encoded position
 *            (the merge target for rewind > 0).  Accumulator walkers (nth /
 *            iter_skip) descend INTO it on rewind > 0, because re-scanning the
 *            shallower holder would re-count its already-counted contributions.
 *            Idempotent walkers (inequality minmax/sibling) and the precise
 *            lookup ignore it and just re-scan / re-read the returned holder.
 *
 * Returns the live node holding the slot equivalent to the failing one (the
 * caller re-anchors its descent there and re-reads / re-descends).  Never
 * returns NULL on a well-formed trie: the writer wires every fresh cluster's
 * parent (including the cluster top's, into the live parent) before the
 * cluster becomes reachable, so the up-walk never observes a NULL parent.  All
 * call sites assert anchor != NULL and treat any NULL return as a bug.  The
 * defensive `return NULL` paths inside the walk (assert(0) + return NULL under
 * NDEBUG; pathological guard exhaustion) exist only so a debug build aborts at
 * the violation site instead of dereferencing NULL.
 *
 * Read-side only (rcu_dereference on every back-pointer); the caller must be in
 * an RCU read-side critical section.
 */
static
struct cds_ft_inode_flag *ft_skip_reanchor_impl(struct cds_ft *ft,
		struct cds_ft_inode_flag *skip_ptr,
		unsigned int *rewind, struct cds_ft_inode_flag **at_pos,
		bool reader)
{
	unsigned int want = ft_skip_len(skip_ptr);
	unsigned int acc = 0;
	struct cds_ft_inode_flag *cur = ft_skip_child_ptr(skip_ptr);	/* G */
	int guard;

	*rewind = 0;
	if (at_pos)
		*at_pos = NULL;
	FT_TP(reanchor_enter, (const void *) skip_ptr, (const void *) cur, want);
	for (guard = 0; guard < (int) FT_MAX_DEPTH + 2; guard++) {
		struct cds_ft_inode_flag *parent;
		struct cds_ft_inode_flag *head_word = NULL;
		void *pitem;

		if (ft_node_external(cur)) {
			/*
			 * From the chain HEAD -- @cur may be an interior member
			 * (ft_ext_head_word); the head's position is the member's.
			 * ONE load gives both halves of this hop: the parent the
			 * head names AND whether it is that parent's prefix head
			 * (ft_upwalk_edge_bytes).  Keep the raw word -- stripping
			 * is what ft_resolve_head_prev does for callers that only
			 * want the node -- so the two cannot be taken from
			 * different observations of a live re-home.
			 */
			struct cds_ft_node *head;

			head_word = ft_ext_head_word(ft,
				(struct cds_ft_node *) cur, &head, reader);
			cur = (struct cds_ft_inode_flag *) head;
			parent = ft_parent_prefix_strip(head_word);
		} else {
			parent = ft_parent_node(rcu_dereference(
				cds_ft_item_to_metadata(
				ft_node_ptr(cur))->parent_word));
		}
		/* A picked child's parent may be a flip-proxy mid-merge. */
		parent = ft_resolve_flip_proxy(parent);
		FT_TP(reanchor_walk, (const void *) cur, (const void *) parent, acc);
		if (caa_unlikely(!parent)) {
			/*
			 * A NULL parent on the up-walk is a bug.  The walk runs
			 * through LIVE nodes whose parents are wired before the node
			 * becomes reader-reachable: a build-invisible commit connects
			 * the whole fresh cluster's parents (including the top's)
			 * before any live gateway exposes it
			 * (ft_graft_glue_apply_deferred), and a detach nulls parent
			 * only after a grace period (unobservable to an in-flight
			 * reader).  The only legitimate NULL parent is the root's,
			 * and the accumulation reaches @want at or below it -- there
			 * are no root-level skip pointers -- so the walk never steps
			 * onto it.
			 *
			 * ☐ ...as long as the skip child stays at its DEPTH.  Two
			 * producers of a shallower child: a rekey to a SHORTER key
			 * moves an internal child up, and the member hop above
			 * substitutes a chain HEAD that sits on the destination path,
			 * shallower than the encoded position when the destination
			 * key is shorter.  Either way the new chain is shorter than
			 * the encoded length and this walk would step onto the root.
			 * Reasoned, not measured (the fixed-key oracles keep every
			 * head at one depth), and handing the root back here was
			 * refuted: with @rewind 0 the write-path ft_reanchor_flag,
			 * the detach source resolve and the count walk would accept
			 * it without a witness.  The variable-length oracle is owed,
			 * and the answer for those callers with it.
			 */
			assert(0);
			return NULL;		/* defensive under NDEBUG */
		}
		pitem = ft_node_compressed(parent) ?
			(void *) ft_compressed_node_ptr(parent) :
			(void *) ft_node_ptr(parent);
		acc += ft_node_compressed(parent) ?
			ft_compressed_node_ptr(parent)->len :
			ft_upwalk_edge_bytes(cur, head_word);
		if (acc >= want) {
			/*
			 * @parent is the node spanning (rewind > 0, a merge) or
			 * sitting at (rewind == 0) the failing slot's encoded
			 * position.  Return the node HOLDING the slot (its parent,
			 * = the scanned node's live version for rewind == 0); also
			 * hand back @parent itself via @at_pos so accumulator
			 * walkers can descend INTO it for rewind > 0 (where
			 * re-scanning the shallower holder would double-count).
			 */
			struct cds_ft_inode_flag *holder;

			*rewind = acc - want;
			if (at_pos)
				*at_pos = parent;
			/* Same flip-proxy resolve as the up-walk read above. */
			holder = ft_parent_node_resolved(
				rcu_dereference(cds_ft_item_to_metadata(
				(struct cds_ft_inode *) pitem)->parent_word));
			/*
			 * The holder is NULL only if @parent is the root -- the
			 * encoded position is the root itself, i.e. a root-level
			 * skip pointer, which mutators never produce.
			 */
			assert(holder != NULL);
			return holder;
		}
		cur = parent;
	}
	return NULL;	/* pathological (cycle?): caller re-descends from root */
}

/* The read-side entry: the descent, the inequality and ordered-query walkers. */
static
struct cds_ft_inode_flag *ft_skip_reanchor(struct cds_ft *ft,
		struct cds_ft_inode_flag *skip_ptr,
		unsigned int *rewind, struct cds_ft_inode_flag **at_pos)
{
	return ft_skip_reanchor_impl(ft, skip_ptr, rewind, at_pos, true);
}

static inline
bool ft_group_skip_compressed(const struct cds_ft_group *group)
{
	return group->flags & CDS_FT_FLAG_SKIP_COMPRESSED;
}
#else
static inline
bool ft_node_skip_compressed(struct cds_ft_inode_flag *node __attribute__((unused)))
{
	return false;
}

static inline
unsigned int ft_skip_len(struct cds_ft_inode_flag *node __attribute__((unused)))
{
	return 0;
}

static inline
struct cds_ft_inode_flag *ft_skip_child_ptr(struct cds_ft_inode_flag *node)
{
	return node;
}

static
struct cds_ft_inode_flag *ft_skip_compressed_flag(
		struct cds_ft_inode_flag *child,
		unsigned int len __attribute__((unused)))
{
	return child;
}

static inline
struct cds_ft_compressed_node *ft_skip_to_compressed(const struct cds_ft *ft,
		struct cds_ft_inode_flag *skip_ptr)
{
	(void) ft;
	return ft_compressed_node_ptr(skip_ptr);
}


static inline
bool ft_group_skip_compressed(const struct cds_ft_group *group __attribute__((unused)))
{
	return false;
}

#endif /* FEATURE_FT_SKIP_COMPRESSED */

/*
 * Assert the root-is-internal invariant on a reader's root load.  See the
 * ft_root_dereference_* macros above: no mutator ever publishes a compressed /
 * skip-compressed node at the trie root, so a reader must never observe one.
 * Defined here (not at the macros) because it needs ft_node_skip_compressed,
 * whose real definition and no-skip stub both live in the block above.
 * Compiled out under NDEBUG.
 */
static inline
struct cds_ft_inode_flag *ft_root_assert_not_compressed(
		struct cds_ft_inode_flag *root)
{
	assert(!ft_node_compressed(root) && !ft_node_skip_compressed(root));
	return root;
}

/*
 * ft_set_parent_slot: record a node's slot within its parent as a
 * pointer-stride offset (parent_slot_offset).  The raw byte offset is
 * divided by sizeof(void *) (always 8 on 64-bit) so that the 8-bit
 * field can cover the full pigeon node (2048 bytes / 8 = 256 slots,
 * max index 255).
 *
 * Maintained for every internal/compressed node, not just
 * skip-compressed ones: it lets the parent-pointer backtrack recover a
 * node's parent slot in O(1) without re-descending, and it backs the
 * skip-compressed dual-pointer publish / chain-merge canonicalization.
 *
 * When parent is NULL (root's child), the offset is unused --
 * ft_get_parent_slot recovers &ft->root.
 */
/* Recover the branch byte indexing @slot in internal parent @node (defined
 * after the popcount layout helpers). */
static uint8_t ft_slot_to_byte(const struct cds_ft_type *type,
		struct cds_ft_inode *node, struct cds_ft_inode_flag **slot);

static inline
void ft_set_parent_slot(struct cds_ft_metadata *meta,
		struct cds_ft_inode_flag *parent,
		struct cds_ft_inode_flag **slot)
{
	struct cds_ft_inode_flag *p;
	bool parent_compressed;

	if (!slot)
		return;	/* Slot unknown -- preserve existing offset. */
	if (!parent) {
		ft_meta_parent_slot_offset_set(meta, 0);
		return;
	}
#ifdef FT_DEBUG_DEL_TOMB
	/*
	 * PROBE: a RAW offset store onto a node whose parent word already
	 * names a DIFFERENT real parent is a re-parent that publishes its
	 * (parent, offset) pair in two separate stores -- the torn-pair
	 * producer.  Name the site.
	 */
	{
		struct cds_ft_inode_flag *oldp = ft_parent_node(
			(struct cds_ft_inode_flag *) CMM_LOAD_SHARED(meta->parent_word));
		static unsigned long ft_dt_raw_reparent;

		if (oldp && !ft_node_flip_proxy(oldp) &&
				ft_node_ptr(oldp) != ft_node_ptr(parent) &&
				uatomic_add_return(&ft_dt_raw_reparent, 1) <= 12) {
			void *bt[20];
			int nbt = backtrace(bt, 20);

			fprintf(stderr, "FT RAW-REPARENT-OFFSET meta=%p old_parent=%p new_parent=%p old_off=%u new_off=%u state=%#lx\n",
				(void *) meta, (void *) oldp, (void *) parent,
				(unsigned int) FT_PSO_DECODE(CMM_LOAD_SHARED(meta->parent_slot_offset)),
				(unsigned int) (((char *) slot - (char *) ft_node_ptr(parent)) / sizeof(void *)),
				(unsigned long) CMM_LOAD_SHARED(meta->state));
			backtrace_symbols_fd(bt, nbt, 2);
		}
	}
#endif
	ft_meta_parent_slot_offset_set(meta, (unsigned int)((char *) slot -
		(char *) ft_node_ptr(parent)) / sizeof(void *));
	/*
	 * Record this node's incoming branch byte for the up-walk key rebuild
	 * (ft_rebuild_key_upwalk).  This is THE central populate point for every
	 * slot-placed node (internal + compressed) -- it runs from ft_set_parent
	 * AND ft_publish_to_parent, so all publish paths are covered without
	 * threading the byte to each call site.  Derive it by inverting @slot
	 * against the parent's bitmap (cold path).  Only meaningful when the
	 * parent is an internal (slot-array) node: a compressed parent has no
	 * slot array -- the edge byte lives in its key_bytes -- so skip it (the
	 * up-walk likewise skips a node whose parent is compressed).
	 */
	p = parent;
	parent_compressed = ft_node_compressed(p);
#ifdef FEATURE_FT_SKIP_COMPRESSED
	parent_compressed = parent_compressed || ft_node_skip_compressed(p);
#endif
	if (!parent_compressed)
		meta->incoming_byte = FT_DT_IB_STORE(meta, ft_slot_to_byte(
			&ft_types[ft_node_type(p)], ft_node_ptr(p), slot));
}

/*
 * ft_get_parent_slot: recover a node's parent-slot address from the
 * stored pointer-stride offset.
 *
 * The node body IS the packed child-pointer array (metadata lives in a
 * sibling page), so offset 0 is a valid slot -- the node's first/lowest
 * child.  A placed non-root child therefore always has a meaningful
 * offset, including 0; the "no recorded slot" state is fully captured by
 * parent == NULL (the root's child, recovered as &ft->root below).  Do
 * NOT treat offset 0 as "unset": that aliases the lowest child of every
 * node and silently drops its skip re-encode (ft_publish_to_parent /
 * ft_node_recompact would skip it on a child-change, leaving a stale
 * skip pointer in the parent slot).
 *
 * @ft is needed for the root case (parent == NULL).
 */
#ifdef FT_DEBUG_PAIR_STORE
/*
 * PROBE, SAME-OP TORN WINDOW: ft_glue_record_back_edge stores a LIVE child's
 * offset PLAIN and only RECORDS its parent word, so until the commit this very
 * op's non-RYW ft_resolve_parent_slot would pair the OLD parent with the NEW
 * offset.  Other writers are excluded by the bulk gate; readers never read the
 * offset.  This ring names the one observer left.
 */
/* The ring itself sits in fractal-trie-internal.h: the writer scope clears it. */
static unsigned long ft_dt_glue_noted, ft_dt_glue_torn_read;

static inline
void ft_dt_glue_note(const struct cds_ft_metadata *meta,
		struct cds_ft_inode_flag *new_parent)
{
	unsigned int i = ft_dt_glue_ring_n++ % FT_DT_GLUE_RING;

	ft_dt_glue_ring[i].meta = meta;
	ft_dt_glue_ring[i].new_parent = new_parent;
	uatomic_inc(&ft_dt_glue_noted);
}

__attribute__((noinline))
static void ft_dt_glue_check(const struct cds_ft_metadata *meta,
		struct cds_ft_inode_flag *parent_seen)
{
	unsigned int i;

	for (i = 0; i < FT_DT_GLUE_RING; i++) {
		if (ft_dt_glue_ring[i].meta != meta)
			continue;
		if (ft_node_ptr(ft_parent_node(parent_seen)) ==
				ft_node_ptr(ft_dt_glue_ring[i].new_parent))
			return;		/* the window has closed */
		if (uatomic_add_return(&ft_dt_glue_torn_read, 1) <= 8) {
			void *bt[24];
			int nbt = backtrace(bt, 24);

			fprintf(stderr, "FT GLUE-TORN-READ meta=%p seen_parent=%p glue_parent=%p\n",
				(void *) meta, (void *) parent_seen,
				(void *) ft_dt_glue_ring[i].new_parent);
			backtrace_symbols_fd(bt, nbt, 2);
		}
		return;
	}
}

static void ft_dt_glue_report(void) __attribute__((destructor));
static void ft_dt_glue_report(void)
{
	fprintf(stderr, "FT GLUE-WINDOW noted=%lu torn_read=%lu\n",
		ft_dt_glue_noted, ft_dt_glue_torn_read);
}
#endif

/*
 * ft_resolve_parent_slot: recover a node's parent AND its parent-slot address as
 * a CONSISTENT snapshot, tolerating a mid-commit atomic re-home (Phase 4.3).
 *
 * A re-home commits @meta->parent (a type-7 flip-proxy) and @meta->parent_slot_
 * offset (FT_STATE_PROXY) as ONE 2-edge MCAS txn.  Resolving each field with a
 * SEPARATE status load can tear across the commit's status flip -- read parent ->
 * OLD, flip, read offset -> NEW -> a slot address computed off the wrong parent
 * body -> ft_slot_to_byte OOB.  So the two edges must be driven from a SINGLE
 * status snapshot: when @meta->parent carries the proxy, take its txn @t, read
 * urcu_txn_desc_status(t) ONCE, and resolve BOTH edges through it.  A re-home
 * that begins mid-snapshot is caught by the coherence re-read of @meta->parent
 * and retried.
 *
 * The §8.3 split SHARPENED this: the offset now has its own word, written ONLY
 * by a re-home, so a proxy parked there is unambiguously @t's.  While the offset
 * shared @state, that word was also parked by freezes and tombstones -- edges
 * that leave the offset untouched -- so the resolver had to tell a co-committed
 * offset edge apart from an unrelated state edge.  The same-descriptor check
 * below still earns its keep against a re-home racing the snapshot, but it no
 * longer has to disambiguate two different KINDS of parker.
 *
 * No re-home in flight (single writer, or between commits) => the fast path is a
 * plain parent load + a proxy-tolerant offset load, behaviour-identical to the
 * pre-4.3 raw recovery.  @parent_out (optional) receives the parent that matches
 * the returned slot.  Call from within an RCU read-side section.
 */
static inline
struct cds_ft_inode_flag **ft_resolve_parent_slot(
		const struct cds_ft_metadata *meta, struct cds_ft *ft,
		struct cds_ft_inode_flag **parent_out)
{
	struct cds_ft_inode_flag *parent;
	void *state;

	for (;;) {
		struct cds_ft_inode_flag *praw = rcu_dereference(meta->parent_word);
		void *sraw = uatomic_load(
				(void **) (uintptr_t) &meta->parent_slot_offset,
				CMM_ACQUIRE);

		if (caa_likely(!ft_node_flip_proxy(praw))) {
			/*
			 * No re-home parking @meta->parent.  The state word may
			 * still carry an unrelated proxy (a freeze that leaves the
			 * offset unchanged); resolve it independently.  A re-home
			 * that STARTS after this load is caught by the coherence
			 * re-read below.
			 */
			parent = praw;
			state = urcu_txn_resolve(sraw, FT_STATE_PROXY);
		} else {
			struct urcu_txn_record *rp = ft_flip_proxy_ptr(praw);
			struct urcu_txn_desc *t = rp->desc;
			unsigned long st = urcu_txn_desc_status(t);

			parent = (struct cds_ft_inode_flag *)
				(st == URCU_TXN_DESC_SUCCEEDED ? rp->new_ptr : rp->old_ptr);
			if (caa_likely(urcu_txn_is_proxy(sraw, FT_STATE_PROXY))) {
				struct urcu_txn_record *rs =
					urcu_txn_untag(sraw, FT_STATE_PROXY);

				if (caa_unlikely(rs->desc != t))
					continue;	/* stale offset edge: re-snapshot */
				state = st == URCU_TXN_DESC_SUCCEEDED ?
						rs->new_ptr : rs->old_ptr;
			} else {
				/* Parent parked but offset already settled: re-snapshot. */
				continue;
			}
		}
		/*
		 * Coherence guard: the snapshot is torn only if a re-home landed
		 * on @meta->parent between the two loads above.  A stable parent
		 * edge means (parent, offset) came from one consistent view.
		 */
		if (caa_likely(rcu_dereference(meta->parent_word) == praw))
			break;
	}

	/*
	 * The owner stamp is a WITNESS, never a navigation aid: the slot comes
	 * from the caller's @ft, exactly as it did when a root's parent was
	 * NULL.  A cross-trie move legitimately leaves the stamp naming the
	 * PREVIOUS owner until it commits, so deriving the slot from the stamp
	 * would answer with a slot in the wrong trie mid-move -- while @ft is
	 * the trie the caller is actually operating on.  Reading ownership is
	 * cds_ft_verify's job, and it reads the word itself.
	 *
	 * @parent_out is the parent NODE, so a root still yields NULL.
	 */
#ifdef FT_DEBUG_PAIR_STORE
	if (caa_unlikely(ft_dt_glue_ring_n))
		ft_dt_glue_check(meta, parent);
#endif
	if (parent_out)
		*parent_out = ft_parent_node(parent);
	if (ft_parent_is_root_position(parent))
		return &ft->root;
	return (struct cds_ft_inode_flag **)
		((char *) ft_node_ptr(parent) +
		 FT_PSO_DECODE(state) * sizeof(void *));
}

static inline
struct cds_ft_inode_flag **ft_get_parent_slot(const struct cds_ft_metadata *meta,
		struct cds_ft *ft)
{
	return ft_resolve_parent_slot(meta, ft, NULL);
}

/*
 * ft_txn_parent_slot: @meta's parent slot as @mtxn WILL LEAVE IT.
 *
 * ft_resolve_parent_slot above answers from the words as they stand: it
 * resolves a PEER's parked re-home through the flip proxy, but an edge this
 * op has merely RECORDED is not parked yet and lives only in the descriptor,
 * so a raw derivation cannot see it.  An op that re-parents @meta and then
 * derives @meta's parent slot in the same attempt therefore gets the PRE-OP
 * slot -- and when the re-parent came from a recompaction, that slot sits
 * inside the copy this very commit retires.
 *
 * So read both words READ-YOUR-OWN-WRITES.  The (parent, offset) pair needs no
 * coherence re-read here: a re-parent records them together, so the txn returns
 * one op's view of both, and any word without a pending edge falls through to
 * the same waiting load ft_resolve_parent_slot performs.
 *
 * @mtxn NULL answers exactly as ft_get_parent_slot does.
 */
static inline
struct cds_ft_inode_flag **ft_txn_parent_slot_at(const struct cds_ft_metadata *meta,
		struct cds_ft *ft, struct urcu_txn *mtxn,
		struct cds_ft_inode_flag **parent_out)
{
	struct cds_ft_inode_flag *parent;
	void *state;

	if (parent_out)
		*parent_out = NULL;
	if (!mtxn)
		return ft_resolve_parent_slot(meta, ft, parent_out);
	parent = urcu_txn_load(mtxn,
		(void **) (uintptr_t) &meta->parent_word, FT_FLIP_PROXY_TAG);
	state = urcu_txn_load(mtxn,
		(void **) (uintptr_t) &meta->parent_slot_offset, FT_STATE_PROXY);
	if (ft_parent_is_root_position(parent))
		return &ft->root;
	/*
	 * @parent_out is the node the returned slot LIVES IN -- the word that
	 * owns that slot (§8.2: a node's body is its own).  Handed back rather
	 * than re-derived by the caller, because only the RYW load above can see
	 * a re-parent this very commit recorded.
	 */
	if (parent_out)
		*parent_out = ft_parent_node(parent);
	return (struct cds_ft_inode_flag **)
		((char *) ft_node_ptr(parent) +
		 FT_PSO_DECODE(state) * sizeof(void *));
}

#define ft_txn_parent_slot(meta, ft, mtxn)				\
	ft_txn_parent_slot_at((meta), (ft), (mtxn), NULL)

/*
 * ft_slot_in_node: is @slot one of @node_flag's OWN child slots?
 *
 * The inverse of ft_get_parent_slot, which computes every slot as the node body
 * plus a pointer-stride offset: the node body IS the packed child-pointer array,
 * sized (1 << type->order) and naturally aligned, so containment is an address
 * range test.  A compressed node carries exactly one child slot, @cn->child.
 *
 * This is the pairing test for a plan that holds a slot address INSIDE one node
 * and a separately-resolved pointer TO that node.  Both can be individually
 * coherent while the PAIR is not: a peer that republishes the node between the
 * descent that produced the slot and the load that resolved the pointer leaves
 * the slot addressing the retired body while the pointer names the fresh copy.
 * No expected-old on the holder slot can see that -- the value there matches
 * itself; what is stale is the premise the descent established about the node.
 *
 * @node_flag must already be resolved (no flip proxy); a skip form resolves to
 * the compressed node it names.  A value that holds no child slot at all (NULL,
 * an external chain) answers false.
 */
static inline
bool ft_slot_in_node(struct cds_ft_inode_flag *node_flag,
		struct cds_ft_inode_flag **slot)
{
	const char *body;

	if (!node_flag || !slot)
		return false;
	/* Skip before external: a skip pointer's low bits read as external. */
	node_flag = ft_skip_child_ptr(node_flag);
	if (ft_node_compressed(node_flag))
		return slot == &ft_compressed_node_ptr(node_flag)->child;
	if (!ft_node_internal(node_flag))
		return false;
	body = (const char *) ft_node_ptr(node_flag);
	if (!body)
		return false;
	return (const char *) slot >= body &&
		(const char *) slot < body +
			((size_t) 1 << ft_types[ft_node_type(node_flag)].order);
}

#if defined(FT_DEBUG_DEL_TOMB) || defined(FT_DEBUG_PAIR_STORE)
/*
 * ft_dt_pso_store_probe: score the rule ft_meta_parent_slot_offset_set's own
 * header states -- the (parent_word, parent_slot_offset) PAIR may be rewritten
 * by PLAIN stores only while the child is still INVISIBLE (a fresh recompaction
 * copy nobody can reach yet).  A live, reader-reachable child re-homes through
 * ft_reparent_record_meta, which co-commits both words in one flip, precisely so
 * no reader can observe half of the pair.
 *
 * The discriminator is the child's OWN reachability, and it needs no clock: a
 * fresh copy inherits the pair of the node it REPLACES, so the slot that pair
 * names still holds the OLD node; a live child's pair names a slot holding
 * ITSELF.  "The slot already holds me" therefore means "a reader can reach me
 * through this very pair" -- and rewriting it here, one plain store at a time,
 * is the window in which that reader sees a parent from before and an offset
 * from after.
 *
 * Silent (returns) whenever the question cannot be asked cleanly: no parent yet,
 * a re-home already parked on either word, a root child (whose offset is unused),
 * a same-value republish (the in-place reserve's hot case -- it changes nothing),
 * or a pair that is ALREADY incoherent (a different probe's business).  So a hit
 * is a positive answer, never an artefact of the reading.
 */
static unsigned long ft_dt_pso_asked, ft_dt_pso_live, ft_dt_ib_asked,
	ft_dt_ib_changed, ft_dt_ib_live, ft_dt_pair_undecided, ft_dt_ib_ctl_live;

/*
 * Is @meta REACHABLE by a reader: does the slot its CURRENT (parent_word,
 * offset) pair names hold it, and so on up to the root?  One hop is not enough:
 * a fresh cluster built bottom-up has a fresh parent whose slot holds the fresh
 * child, and nobody can reach either until the top is published -- the top's
 * pair names the LIVE parent, whose slot still holds the node being replaced.
 * False (or undecided) whenever a hop cannot be read cleanly, so a true answer
 * is a positive one.
 */
static inline
bool ft_dt_live_via_pair(const struct cds_ft_metadata *meta,
		bool *first_parent_internal)
{
	const struct cds_ft_metadata *m = meta;
	int hops;

	*first_parent_internal = false;
	for (hops = 0; hops < FT_MAX_DEPTH; hops++) {
		struct cds_ft_inode_flag *praw, *parent, *val;
		struct cds_ft_inode_flag **slot;
		uintptr_t cur;

		praw = (struct cds_ft_inode_flag *) CMM_LOAD_SHARED(m->parent_word);
		if (!praw || ft_node_flip_proxy(praw))
			return false;
		if (ft_parent_is_root_position(praw)) {
			struct cds_ft *owner;
			struct cds_ft_inode_flag *rv;

			/*
			 * Every hop below held its child.  At hop 0 @meta IS
			 * a root: its offset and byte are unused, so there is
			 * no question to answer.
			 */
			if (!hops)
				return false;
			/*
			 * ☠ A root-position WORD is not a root: a fresh node is
			 * stamped with its owning trie before anything publishes
			 * it.  The trie's root slot must hold it, and the trie
			 * must be one readers can enter.
			 */
			owner = ft_parent_trie(praw);
			if (owner->exclusive)
				return false;
			rv = (struct cds_ft_inode_flag *) CMM_LOAD_SHARED(owner->root);
			return rv && !ft_node_flip_proxy(rv) &&
				ft_node_ptr(ft_skip_child_ptr(rv)) ==
				cds_ft_metadata_to_item((struct cds_ft_metadata *) m);
		}
		parent = ft_parent_node(praw);
		if (!parent || ft_node_flip_proxy(parent) ||
				ft_node_external(parent))
			return false;
		if (!hops)
			*first_parent_internal = !ft_node_compressed(parent)
#ifdef FEATURE_FT_SKIP_COMPRESSED
				&& !ft_node_skip_compressed(parent)
#endif
				;
		cur = CMM_LOAD_SHARED(m->parent_slot_offset);
		if (cur & FT_STATE_PROXY)
			return false;
		slot = (struct cds_ft_inode_flag **) ((char *) ft_node_ptr(parent) +
				FT_PSO_DECODE(cur) * sizeof(void *));
		if (!ft_slot_in_node(parent, slot))
			return false;
		val = (struct cds_ft_inode_flag *) CMM_LOAD_SHARED(*slot);
		if (!val || ft_node_flip_proxy(val))
			return false;
		if (ft_node_ptr(ft_skip_child_ptr(val)) !=
				cds_ft_metadata_to_item((struct cds_ft_metadata *) m))
			return false;	/* the slot holds another node: invisible */
		m = ft_node_compressed(parent) ?
			cds_ft_item_to_metadata((struct cds_ft_inode *)
				ft_compressed_node_ptr(parent)) :
			cds_ft_item_to_metadata(ft_node_ptr(parent));
	}
	uatomic_inc(&ft_dt_pair_undecided);
	return false;
}

/* Hits per call site: the setter's caller, symbolized offline. */
#define FT_DT_PAIR_SITES	64
static struct {
	const void *ra;
	int kind;		/* 0 offset, 1 incoming_byte */
	unsigned long n;
} ft_dt_pair_sites[FT_DT_PAIR_SITES];
static pthread_mutex_t ft_dt_pair_lock = PTHREAD_MUTEX_INITIALIZER;

static void ft_dt_pair_hit(const void *ra, int kind)
{
	int i;

	pthread_mutex_lock(&ft_dt_pair_lock);
	for (i = 0; i < FT_DT_PAIR_SITES; i++) {
		if (!ft_dt_pair_sites[i].ra) {
			ft_dt_pair_sites[i].ra = ra;
			ft_dt_pair_sites[i].kind = kind;
		}
		if (ft_dt_pair_sites[i].ra == ra &&
				ft_dt_pair_sites[i].kind == kind) {
			ft_dt_pair_sites[i].n++;
			break;
		}
	}
	pthread_mutex_unlock(&ft_dt_pair_lock);
}

__attribute__((noinline))
static void ft_dt_pso_store_probe(const struct cds_ft_metadata *meta, unsigned int off)
{
	uintptr_t cur;
	bool internal;

	uatomic_inc(&ft_dt_pso_asked);
	cur = CMM_LOAD_SHARED(meta->parent_slot_offset);
	if (!(cur & FT_STATE_PROXY) && FT_PSO_DECODE(cur) == (uintptr_t) off)
		return;			/* same-value republish: the pair does not move */
	if (!ft_dt_live_via_pair(meta, &internal))
		return;
	uatomic_inc(&ft_dt_pso_live);
	ft_dt_pair_hit(__builtin_return_address(0), 0);
}

/*
 * ft_dt_ib_store_probe: a PLAIN store of @incoming_byte into a REACHABLE node,
 * CHANGING the byte.  The up-walk loads the parent and this byte separately, and
 * nothing parks this field.  Returns @byte.
 */
__attribute__((noinline))
static unsigned int ft_dt_ib_store_probe(const struct cds_ft_metadata *meta,
		unsigned int byte)
{
	bool internal;

	uatomic_inc(&ft_dt_ib_asked);
	if (meta->incoming_byte == byte) {
		/*
		 * POSITIVE CONTROL for the climb: a same-value store is often a
		 * live node's republish, so a climb that can never answer
		 * "reachable" reads 0 here too.
		 */
		if (ft_dt_live_via_pair(meta, &internal) && internal)
			uatomic_inc(&ft_dt_ib_ctl_live);
		return byte;		/* same value: nothing a reader can see */
	}
	uatomic_inc(&ft_dt_ib_changed);
	/*
	 * Only a node reached through an INTERNAL parent: a compressed parent's
	 * child has no edge byte of its own, and readers skip it there.
	 */
	if (!ft_dt_live_via_pair(meta, &internal) || !internal)
		return byte;
	uatomic_inc(&ft_dt_ib_live);
	ft_dt_pair_hit(__builtin_return_address(0), 1);
	/* WHO: the setter's caller is out of reach of one return address. */
	if (uatomic_read(&ft_dt_ib_live) <= 8) {
		void *bt[24];
		int nbt = backtrace(bt, 24);

		fprintf(stderr, "FT PAIR-STORE-IB meta=%p byte %u -> %u\n",
			(void *) meta, (unsigned int) meta->incoming_byte, byte);
		backtrace_symbols_fd(bt, nbt, 2);
	}
	return byte;
}

static void ft_dt_pair_report(void) __attribute__((destructor));
static void ft_dt_pair_report(void)
{
	int i;

	fprintf(stderr, "FT PAIR-STORE offset: asked=%lu LIVE=%lu | incoming_byte: "
		"asked=%lu changed=%lu LIVE=%lu | undecided=%lu | control: same-value "
		"ib reachable=%lu (ra base: cds_ft_create=%p)\n", ft_dt_pso_asked,
		ft_dt_pso_live, ft_dt_ib_asked, ft_dt_ib_changed, ft_dt_ib_live,
		ft_dt_pair_undecided, ft_dt_ib_ctl_live, (void *) &cds_ft_create);
	for (i = 0; i < FT_DT_PAIR_SITES && ft_dt_pair_sites[i].ra; i++)
		fprintf(stderr, "FT PAIR-STORE-SITE kind=%s ra=%p n=%lu\n",
			ft_dt_pair_sites[i].kind ? "incoming_byte" : "offset",
			ft_dt_pair_sites[i].ra, ft_dt_pair_sites[i].n);
}
#endif


#ifdef FT_ENABLE_TRACING
#include <stdio.h>
#include <stdlib.h>
/*
 * Flight-recorder WRITE-side mis-wire validator (tracing builds only): a
 * forward-slot publish whose NEW value is a PLAIN compressed flag must name
 * a live cn whose (parent, PSO) pair derives the very slot being written --
 * every legit producer (fresh publish, recompact republish) wires the pair
 * before publishing, and the SKIP_X dual writes the skip FORM (exempt via
 * the plain-only filter).  Publishing a plain cn flag into any OTHER slot,
 * or naming a tombstoned / len==0 target, is a mis-wire caught AT CREATION
 * -- the abort core shows exactly who built the edge, and the snapshot
 * carries the window.  Wired at the _ft_publish_to_parent(_meta) entries
 * (the canonical forward-slot recorders), NOT at the txn record choke point:
 * back-pointer FIELDS legitimately hold plain cn flags (a child's
 * meta->parent naming its fresh cn parent) and would false-fire there.
 */
static __attribute__((unused))
void ft_trace_pub_check(struct cds_ft *ft,
		struct cds_ft_inode_flag **slot, struct cds_ft_inode_flag *nf,
		unsigned int site)
{
	struct cds_ft_compressed_node *cn;
	struct cds_ft_metadata *meta;
	uintptr_t state;
	struct cds_ft_inode_flag *rt_parent;
	struct cds_ft_inode_flag **rt_slotp;

	if (!nf || !ft_node_compressed(nf))
		return;
	cn = ft_compressed_node_ptr(nf);
	meta = cds_ft_item_to_metadata((struct cds_ft_inode *) cn);
	state = (uintptr_t) urcu_txn_read((void **) &meta->state,
			FT_STATE_PROXY);
	rt_parent = ft_parent_node_resolved(
			rcu_dereference(meta->parent_word));
	rt_slotp = rt_parent ? ft_get_parent_slot(meta, ft) : NULL;
	if (caa_likely(cn->len != 0 && !(state & FT_STATE_TOMBSTONE) &&
			rt_slotp == slot))
		return;
	/*
	 * ☞ FREEZE BEFORE THE SNAPSHOT, as ft_trace_miswire_check does: peers
	 * that keep tracing while `lttng snapshot record` runs wrap every
	 * per-CPU ring and leave the culprit's window overwritten.
	 */
	FT_TRACE_FREEZE();
	FT_TP(miswire, site, (const void *) nf, (const void *) cn,
		(unsigned int) cn->len, state, (const void *) rt_parent,
		(const void *) rt_slotp);
	fprintf(stderr, "FT PUB-MISWIRE site %u slot %p new %p target %p "
		"len %u state %#lx rt_parent %p rt_slot %p\n",
		site, (void *) slot, (void *) nf, (void *) cn,
		(unsigned int) cn->len, (unsigned long) state,
		(void *) rt_parent, (void *) rt_slotp);
	(void) system("lttng snapshot record 1>&2");
	abort();
}
#ifndef FT_LIGHT_TRACING	/* see FT_TRACE_MISWIRE: drop the per-publish
				 * round-trip validator, keep the tracepoints. */
#define FT_TRACE_PUB_CHECK(ft, slot, nf, site) \
	ft_trace_pub_check(ft, slot, nf, site)
#else
#define FT_TRACE_PUB_CHECK(ft, slot, nf, site) do { (void) (ft); (void) (slot); (void) (nf); (void) (site); } while (0)
#endif
#else
#define FT_TRACE_PUB_CHECK(ft, slot, nf, site) do { } while (0)
#endif	/* FT_ENABLE_TRACING */

/*
 * ft_flag_to_metadata: get the metadata for any node flag, including
 * skip-compressed pointers.  For skip pointers, returns the
 * compressed node's metadata.  For all others, returns
 * cds_ft_item_to_metadata(ft_node_ptr(nf)).
 *
 * Caller must ensure nf is not NULL and not external.
 */
static inline
struct cds_ft_metadata *ft_flag_to_metadata(const struct cds_ft *ft,
		struct cds_ft_inode_flag *nf)
{
	(void) ft;
#ifdef FEATURE_FT_SKIP_COMPRESSED
	if (ft_node_skip_compressed(nf)) {
		struct cds_ft_compressed_node *cn =
			ft_skip_to_compressed(ft, nf);
		return cds_ft_item_to_metadata(
			(struct cds_ft_inode *) cn);
	}
#endif
	return cds_ft_item_to_metadata(ft_node_ptr(nf));
}

/*
 * ft_flag_tombstoned: has this node flag been RETIRED?
 *
 * A retired node keeps its body readable (RCU) but its state word carries
 * FT_STATE_TOMBSTONE, and every fence primitive refuses it -- so an op whose
 * plan names a retired node can never make progress and must re-derive rather
 * than retry.  An EXTERNAL head has no state word and is never "tombstoned" in
 * this sense (its removal is signalled on node->next, see ft_node_is_removed).
 */
static inline
bool ft_flag_tombstoned(const struct cds_ft *ft,
		struct cds_ft_inode_flag *nf)
{
	struct cds_ft_metadata *meta;

	if (!nf || ft_node_external(nf))
		return false;
	meta = ft_flag_to_metadata(ft, nf);
	return meta != NULL &&
		(CMM_LOAD_SHARED(meta->state) & FT_STATE_TOMBSTONE) != 0;
}

/*
 * If @nf is a skip-compressed pointer, return the underlying
 * compressed node's flag pointer.  Otherwise return @nf unchanged.
 *
 * Use to "see through" the skip-compressed encoding when about to
 * inspect or recurse into the underlying compressed node.  No-op for
 * non-skip pointers; on archs without FEATURE_FT_SKIP_COMPRESSED the
 * check is constant-folded to false and the call collapses to a copy.
 */
static inline
struct cds_ft_inode_flag *ft_resolve_skip_compressed(const struct cds_ft *ft,
		struct cds_ft_inode_flag *nf)
{
	(void) ft;
	if (ft_node_skip_compressed(nf))
		return ft_compressed_node_flag(ft_skip_to_compressed(ft, nf));
	return nf;
}

/*
 * ft_reanchor_flag: resolve an (already flip-proxy-resolved) child flag that may
 * be skip-compressed to a LIVE node, via the read side's ft_skip_reanchor --
 * which walks the skip child's live parent chain to the trie position skip_len
 * encodes -- instead of the unvalidated one-hop ft_skip_to_compressed.
 *
 * This is the MW-safe replacement for ft_resolve_skip_compressed on the WRITE
 * path.  Under mutual exclusion a writer could trust the one-hop back-pointer
 * (no peer could move the structure under its own descent); under MW a peer
 * split/merge can tear that back-pointer into internal memory (type confusion:
 * cn->len reads a bitmap byte) or a stale-length node (mis-file) -- exactly what
 * the read side already tolerates by re-anchoring.  The update side converges on
 * that same mechanism here.
 *
 * @*rewind_ret is set > 0 iff a concurrent chain-merge moved the encoded
 * position shallower than the dispatched child (a caller that captured the raw
 * slot then finds it at the wrong level, and re-descends).  Non-skip @child
 * returns unchanged with rewind 0.  ft_skip_reanchor never returns NULL on a
 * well-formed trie; the result is asserted non-NULL.  RCU-read-side safe.
 */
static inline
struct cds_ft_inode_flag *ft_reanchor_flag(struct cds_ft *ft,
		struct cds_ft_inode_flag *child, unsigned int *rewind_ret)
{
	*rewind_ret = 0;
#ifdef FEATURE_FT_SKIP_COMPRESSED
	if (caa_unlikely(child && ft_node_skip_compressed(child))) {
		struct cds_ft_inode_flag *at_pos, *anchor;

		/* Write path: its hops are not the oracle's positive control. */
		anchor = ft_skip_reanchor_impl(ft, child, rewind_ret, &at_pos,
				false);
		assert(anchor != NULL);
		return at_pos;
	}
#else
	(void) ft;
#endif
	return child;
}

#ifdef FEATURE_FT_SKIP_COMPRESSED
/*
 * ft_skip_to_compressed_meta: shorthand to get the compressed node's
 * metadata from a skip pointer.
 */
static inline
struct cds_ft_metadata *ft_skip_to_compressed_meta(struct cds_ft *ft,
		struct cds_ft_inode_flag *skip_ptr)
{
	return cds_ft_item_to_metadata(
		(struct cds_ft_inode *) ft_skip_to_compressed(ft, skip_ptr));
}
#endif

/*
 * ft_publish_to_parent: atomically publish @new_child into @parent_slot.
 *
 * If the parent is a compressed node, also update the skip pointer
 * at *skip_slot (if one exists) BEFORE writing *parent_slot.  This
 * ensures candidate readers (which follow the skip pointer) see the
 * new child before exact/inequality readers (which follow cn->child).
 *
 * For compressed-form @new_child (SKIP_X or plain COMPRESSED), also
 * maintains the underlying compressed node's parent_slot_offset so it
 * records @parent_slot's offset in @parent_nf -- required by
 * ft_get_parent_slot lookups (the dual-pointer dance above, and the
 * chain-merge canonicalization in ft_detach_node that publishes a
 * replacement at the cn's same grandparent slot).  Without this,
 * compressed nodes installed via ft_publish_to_parent rather than via
 * ft_node_set_nth -> ft_set_parent leave parent_slot_offset == 0 -- a
 * latent gap that silently disabled the dual-pointer SKIP_X update
 * and tripped chain-merge.  This intentionally does NOT update
 * @new_child's parent linkage; callers manage that via their own
 * ft_set_parent (or by direct meta->parent assignment), with
 * semantics that vary across call sites.
 *
 * Centralizes the dual-pointer RCU publication pattern so every
 * write to cn->child automatically maintains the skip pointer.
 *
 * When @rec is non-NULL, the (1-2) reader-visible stores -- the forward
 * parent slot and a compressed parent's SKIP_X dual pointer -- are RECORDED
 * into @rec instead of being performed, so a key-disappearing remove can
 * commit them in one flip with the dead head cell's ordered-list unsplice
 * (ft_detach_node).  All non-reader-visible bookkeeping (parent-slot offset,
 * trace events) still runs.  The public ft_publish_to_parent wrapper passes
 * NULL (direct stores, original behaviour).
 */
#ifdef FT_DEBUG_DUAL_SITE
/*
 * WHICH PRODUCER STILL SAYS "owner NAMED, not held" (build knob;
 * -DFT_DEBUG_DUAL_SITE).
 *
 * The DUAL_NAMED column is the SKIP_X dual's whole conversion surface, and it
 * is recorded in THREE places that all key on @owner_held -- none of which can
 * name the PRODUCER that decided the answer.  Editing a producer without first
 * attributing the population is editing blind, so attribute it: @fn/@line are
 * the CALLER's, exactly as ft_hlist_store_mw_at does it for chain words.
 */
# define FT_DUAL_SITE_MAX	24
static struct {
	const char *fn;
	int line;
	unsigned long named_unheld, named_held, unnamed, root;
} ft_dual_site[FT_DUAL_SITE_MAX];
static unsigned int ft_dual_site_n;

static
void ft_dual_site_tally(const char *fn, int line, bool root,
		struct cds_ft_metadata *owner, bool owner_held)
{
	unsigned int i;

	for (i = 0; i < ft_dual_site_n; i++)
		if (ft_dual_site[i].fn == fn && ft_dual_site[i].line == line)
			break;
	if (i == ft_dual_site_n) {
		if (ft_dual_site_n == FT_DUAL_SITE_MAX)
			return;
		ft_dual_site[i].fn = fn;
		ft_dual_site[i].line = line;
		ft_dual_site_n++;
	}
	if (root)
		uatomic_inc(&ft_dual_site[i].root);
	else if (!owner)
		uatomic_inc(&ft_dual_site[i].unnamed);
	else if (owner_held)
		uatomic_inc(&ft_dual_site[i].named_held);
	else
		uatomic_inc(&ft_dual_site[i].named_unheld);
}

static void ft_dual_site_report(void) __attribute__((destructor));
static void ft_dual_site_report(void)
{
	unsigned int i;

	fprintf(stderr, "FT DUAL-SITE (per ft_pub_rec_add producer)\n");
	for (i = 0; i < ft_dual_site_n; i++)
		fprintf(stderr, "  %-44s:%-5d named_unheld=%lu named_held=%lu "
			"unnamed=%lu root=%lu\n",
			ft_dual_site[i].fn, ft_dual_site[i].line,
			uatomic_read(&ft_dual_site[i].named_unheld),
			uatomic_read(&ft_dual_site[i].named_held),
			uatomic_read(&ft_dual_site[i].unnamed),
			uatomic_read(&ft_dual_site[i].root));
}
#endif /* FT_DEBUG_DUAL_SITE */

/*
 * ☞ THE RECORD KIND ASKS ONE QUESTION: is this op EXCLUDED from every other
 * writer of @slot?  SW is legal exactly when the answer is yes, and the answer
 * has two independent sources -- a lock this op took, or a mode in which no
 * peer writer can exist at all.
 *
 * ☑ COARSE: excluded ALREADY, whatever the word is.  A coarse trie takes the
 * FT-wide @writer_lock at its outermost writer scope, so every writer of every
 * slot is serialised -- INCLUDING &ft->root, which owns no node and can never
 * be locked.  The root slot's MW is a property of FINE locking, not of the root.
 *
 * ☠ FINE / EXPONENTIAL: the FT-wide lock is dropped all-at-once, so exclusion
 * comes only from the per-node lock the op took -- and &ft->root HAS NO NODE TO
 * TAKE.  There the root slot stays MW, and every other word answers with the
 * op's own acquire (@owner_held).
 *
 * Passing this rather than a constant is what lets the producers convert
 * individually -- but ONLY because of what it ASKS.  MW-under-lock and
 * SW-under-lock do serialise, so a site spelled SW never races one still
 * spelled MW *that holds the word*.
 *
 * ☠ THAT QUALIFIER IS THE WHOLE SAFETY ARGUMENT, and dropping it turns this
 * into a licence it is not.  An MW record whose op holds NOTHING is arbitrated
 * against nothing: a peer's SW park is a plain store, so the two genuinely
 * race.  Converting per-site is safe here precisely because a site that cannot
 * vouch answers false and STAYS MW -- never because "MW and SW serialise" on
 * its own.  For a word carrying no lock in its own bits, <urcu/rcu-txn.h>
 * urcu_txn_store_sw is categorical: "a slot is SW xor MW, globally."
 */
#ifdef FT_DEBUG_DUAL_SITE
/*
 * WHY did this slot come out excluded?  The DUAL-SITE table's named_held column
 * mixes three different answers, and only ONE of them is a fine-trie SW park
 * earned by @owner_held -- the others are modes in which no peer writer exists
 * at all.  A conversion argument that reads the merged column cannot tell "this
 * producer vouches" from "this row happened to run coarse", which is the
 * one-bucket-zero trap this file has been caught by before.
 */
unsigned long ft_pub_excl_coarse, ft_pub_excl_wlock, ft_pub_excl_owner,
	ft_pub_excl_none;
static void ft_pub_excl_report(void) __attribute__((destructor));
static void ft_pub_excl_report(void)
{
	fprintf(stderr, "FT PUB-EXCL coarse=%lu wlock=%lu owner_FINE=%lu not_excluded=%lu\n",
		uatomic_read(&ft_pub_excl_coarse),
		uatomic_read(&ft_pub_excl_wlock),
		uatomic_read(&ft_pub_excl_owner),
		uatomic_read(&ft_pub_excl_none));
}
#endif

static inline
bool ft_pub_slot_excluded(const struct cds_ft *ft,
		struct cds_ft_inode_flag **slot, bool owner_held)
{
	if (!ft->lock_fine) {
#ifdef FT_DEBUG_DUAL_SITE
		uatomic_inc(&ft_pub_excl_coarse);
#endif
		return true;		/* FT-wide writer lock serialises all */
	}
	/*
	 * ☑ AND A FINE TRIE INSIDE A BULK WINDOW.  G5.25 has a fine trie
	 * RE-TAKE the FT-wide @writer_lock while a bulk op is live, and point
	 * ops take it too for the duration, so an op holding it excludes every
	 * other writer of every slot -- the same exclusion coarse mode has, for
	 * as long as the window lasts.  This is the statement the glue publish
	 * makes in prose ("its exclusion over the SKIP_X dual's grandparent is
	 * the FT-WIDE WRITER LOCK"); asking @ft_wlock_held makes it a QUERY
	 * every producer answers for itself instead of a per-site claim.
	 *
	 * ☞ It is also the exact predicate FT_OWNER_ASSERT_OWNED was taught to
	 * accept, so a true here is checked by the same test that would trap it.
	 *
	 * ☠☠ AND IT RESTS ON ft_bulk_gate_enter's GRACE PERIOD, not on the flag.
	 * @bulk_active is a PLAIN LOAD taken by the POINT OP at its writer-scope
	 * enter, so a point op that sampled the gate CLEAR skipped the FT-wide
	 * lock and is NOT excluded against the bulk op -- a mixed regime, and
	 * holding the lock would prove nothing on its own.  What closes it is
	 * the gate's own discipline: "publish, one full GP, only THEN mutate",
	 * whose comment names this exact population.  The GP cannot complete
	 * while such a point op is still inside its read section, so by the time
	 * a bulk op publishes anything, pre-flip ops have DRAINED and post-flip
	 * ops OBSERVE the gate and take the lock.
	 *
	 * ⇒ If that update_synchronize_rcu() is ever removed, made conditional,
	 * or moved after the first mutation, THIS CLAUSE SILENTLY BECOMES
	 * UNSOUND -- an SW park beside a concurrent point op's MW CAS, with no
	 * assert to catch it.  The dependency is one-way and cross-file, so it
	 * is written here rather than left to be re-derived.
	 */
	if (ft_wlock_held == (struct cds_ft *) ft) {
#ifdef FT_DEBUG_DUAL_SITE
		uatomic_inc(&ft_pub_excl_wlock);
#endif
		return true;
	}
#ifdef FT_DEBUG_DUAL_SITE
	if (slot != &ft->root && owner_held)
		uatomic_inc(&ft_pub_excl_owner);
	else
		uatomic_inc(&ft_pub_excl_none);
#endif
	return slot != &ft->root && owner_held;
}

static
void ft_pub_rec_add_at(const char *fn, int line,
		struct ft_pub_rec *rec, struct cds_ft_inode_flag **slot,
		struct cds_ft_inode_flag *expected_old,
		struct cds_ft_inode_flag *new_val, bool root,
		struct cds_ft_metadata *owner, bool owner_held)
{
	assert(rec->n < 3);
#ifdef FT_DEBUG_DUAL_SITE
	ft_dual_site_tally(fn, line, root, owner, owner_held);
#else
	(void) fn; (void) line;
#endif
	/*
	 * @owner_held: does the OP hold @owner's lock?  Separate from @owner
	 * because a NULL @owner does NOT fail closed -- the dispatching
	 * recorder branches on the txn's structural_sw alone -- so naming no
	 * owner and holding no owner are the same to it.  false records MW.
	 */
	rec->owner_held[rec->n] = owner_held;
	rec->slot[rec->n] = slot;
	/*
	 * @owner: the node whose lock excludes every other writer of @slot.
	 * Every slot here is a BODY slot -- a forward child pointer or a SKIP_X
	 * dual -- and mw-writer-lock-escalation-model.md §8.2 puts a node's body
	 * under that node's own lock, so the owner is the node the slot LIVES IN,
	 * never the child it points at.
	 *
	 * NULL where the producer cannot name one, and for a ROOT slot, which has
	 * no owning node at all and takes the always-MW route through @root
	 * instead.  A producer that leaves this unset only declines to convert
	 * (see struct ft_pub_rec), so a missing owner is safe and merely counts
	 * OWN_MISS -- which is what it did at EVERY producer before this
	 * parameter existed.
	 */
	rec->owner[rec->n] = owner;
	/*
	 * @root: a TRIE ROOT slot, which every replay must record MW (see
	 * struct ft_pub_rec).  A parameter rather than a re-derivation,
	 * because only the caller holds the trie the slot would be compared
	 * against -- and a cross-trie op holds two.
	 */
	rec->root[rec->n] = root;
	/*
	 * @expected_old is the value the slot held in the snapshot the
	 * publishing PLAN was derived from -- NOT a fresh *slot re-read at
	 * record time.  Recording the plan snapshot makes the commit-time
	 * MCAS reject (abort) a peer that published into this slot between the
	 * plan and the record, instead of ratifying the stale plan because a
	 * fresh raw capture happens to match the peer's value (CORE_682870
	 * defect #1: "stale plan ratified by fresh expected-old").
	 */
	rec->old_val[rec->n] = expected_old;
	rec->new_val[rec->n] = new_val;
	rec->n++;
}

#define ft_pub_rec_add(rec, slot, expected_old, new_val, root, owner, held) \
	ft_pub_rec_add_at(__func__, __LINE__, (rec), (slot), (expected_old), \
		(new_val), (root), (owner), (held))


/*
 * IS THE SKIP_X DUAL'S HOME A NODE THIS COMMIT BUILT?
 *
 * The dual lives in @cn's OWN parent, and the publish resolves that parent
 * READ-YOUR-OWN-WRITES -- so when the SAME commit re-parents @cn, the dual's
 * home MOVES, from the live node the descent fenced to the fresh copy this op
 * built.  A fresh copy is BUILD-INVISIBLE: no peer can reach it, no lock is
 * needed, and none is taken -- so a transacted record naming it as owner names
 * a word the commit cannot be shown to own, and no owner encoding fixes that.
 * The txn publishes REACHABILITY, not INTERIORS: a private node's body is wired
 * by PLAIN STORES, and the dual write is exactly that.
 *
 * ☠ THE STORE IS DEMOTED, NEVER DROPPED.  The recompact copy loop resolved the
 * old home's slots to their COMMITTED values, so the fresh copy is born holding
 * the skip pointer to the OLD child -- stale the instant the forward publish
 * lands.  Skipping the write would leave a candidate reader a shortcut straight
 * to a retired node.
 *
 * ☞ THE TEST IS "DID THIS TXN RE-HOME @cn", asked of the DESCRIPTOR.
 * urcu_txn_load cannot answer it: with no record on the slot it falls through
 * to a fresh read, which is indistinguishable from "not re-homed".  The
 * divergence of the two derivations is a weaker witness of the same fact.
 *
 * A ROOT dual (&ft->root) is excluded whatever the descriptor says: it lives in
 * no node, it is reader-visible at all times, and it takes the always-MW root
 * route.
 *
 * ☠ THIS LEANS ON ONE INVARIANT: a re-parent target is a FRESH cluster, never a
 * live node (ft_reparent_record_meta says so, and every producer in the tree
 * obeys it today).  A future re-homer that targets a LIVE node would turn this
 * dispatch into a plain store into a live body -- silently.  The debug arm below
 * is what would catch it: a re-home that did not MOVE the home is the shape that
 * cannot be private.
 */
static inline
bool ft_dual_home_is_private(struct cds_ft *ft, const struct ft_pub_rec *rec,
		struct cds_ft_metadata *cn_meta,
		struct cds_ft_inode_flag **skip_slot,
		struct cds_ft_inode_flag *skip_owner_nf)
{
	struct urcu_txn_desc *desc;

	if (!rec || !rec->mtxn || skip_slot == &ft->root || !skip_owner_nf)
		return false;
	desc = rec->mtxn->desc;
	if (!desc || desc == URCU_TXN_ENOMEM)
		return false;
	if (!urcu_txn_find(desc, (void **) (uintptr_t) &cn_meta->parent_word))
		return false;
#if defined(DEBUG_RCU) || defined(CONFIG_RCU_DEBUG)
	{
		/*
		 * THE INVARIANT, CHECKED WHERE IT IS CHEAP: a re-home that did
		 * not MOVE the home cannot be private, and this is the arm that
		 * would notice a future re-parent target that is a LIVE node.
		 * Guarded on the same pair urcu_assert_debug itself is, so the
		 * raw re-resolution costs a release build nothing.
		 */
		struct cds_ft_inode_flag *raw_owner = NULL;

		(void) ft_txn_parent_slot_at(cn_meta, ft, NULL, &raw_owner);
		urcu_assert_debug(raw_owner != skip_owner_nf);
	}
#endif
	return true;
}

#ifdef FT_DEBUG_DUAL_DROP
/*
 * THE SKIP_X DUAL INVARIANT DETECTOR (build knob; -DFT_DEBUG_DUAL_DROP).
 *
 * THE INVARIANT: while a compressed node @cn is reached through a SKIP_X word
 * in its own parent, that word encodes cn->child.  Every op that republishes
 * cn->child must refresh BOTH -- the forward slot and the dual -- in ONE
 * commit, and into the LIVE home.
 *
 * DUAL-STALE: the dual is about to be emitted and the word ALREADY names a
 * child other than the plan's expected old.  The invariant is broken, every
 * later attempt on this chain can only abort, and the op then retries
 * obstruction-free forever, allocating a descriptor per attempt.  ABORTS,
 * because that is the shape a regression takes and a livelock reports nothing.
 *
 * ☞ THIS PREDICATE IS SPACING-INDEPENDENT: it reads the DUAL WORD itself, not
 * a lock word.  An earlier version of this probe also counted "the dual was
 * recorded while GP's own state word carried FT_STATE_LOCK" as the exposure
 * figure -- and that IS a wrong zero above per-node spacing, where the lock
 * the op would contend for lives on GP's ANCHOR and GP's own word reads clean.
 * The arm yield for the exclusion now lives where the exclusion does:
 * ft_lock_skip_dual_gp.
 */
# include <stdio.h>
# include <stdlib.h>
# include <execinfo.h>

static
void ft_dbg_dual_probe(struct cds_ft *ft,
		struct cds_ft_compressed_node *cn,
		struct cds_ft_metadata *cn_meta,
		struct cds_ft_inode_flag **skip_slot,
		struct cds_ft_inode_flag *skip_owner_nf,
		struct cds_ft_inode_flag *expected_old,
		struct cds_ft_inode_flag *new_child)
{
	struct cds_ft_inode_flag *raw = skip_slot ? *skip_slot : NULL;

	(void) ft;
	if (!skip_slot || !ft_node_skip_compressed(raw))
		return;			/* no dual is emitted */
	if (ft_skip_child_ptr(raw) == expected_old)
		return;			/* the invariant holds */
	fprintf(stderr,
		"FT DUAL-STALE cn=%p len=%u child=%p slot=%p raw=%p owner=%p exp_old=%p new=%p pw=%p off=%u\n",
		(void *) cn, cn->len, (void *) cn->child, (void *) skip_slot,
		(void *) raw, (void *) skip_owner_nf, (void *) expected_old,
		(void *) new_child, (void *) cn_meta->parent_word,
		ft_meta_parent_slot_offset_load(cn_meta));
	{
		void *bt[24];
		int n = backtrace(bt, 24);

		backtrace_symbols_fd(bt, n, 2);
	}
	fflush(stderr);
	abort();
}
#endif /* FT_DEBUG_DUAL_DROP */

static
void _ft_publish_to_parent_meta_at(const char *pub_fn, int pub_line,
		struct cds_ft *ft,
		struct cds_ft_inode_flag *parent_nf,
		struct cds_ft_inode_flag **parent_slot,
		struct cds_ft_inode_flag *new_child,
		struct cds_ft_inode_flag *expected_old,
		struct cds_ft_metadata *new_child_meta,
		void *folded_child_prev,
		struct ft_pub_rec *rec,
		struct cds_ft_inode_flag *slot_owner_nf,
		bool dual_owner_held)
{
	/*
	 * @slot_owner_nf: the node @parent_slot LIVES IN, i.e. the word that owns
	 * the forward edge (§8.2: a node's body is its own).  Almost always
	 * @parent_nf, which is why _ft_publish_to_parent defaults it -- but NOT
	 * always, and the difference cannot be derived here: a caller may pass
	 * @parent_nf for its OTHER job, deciding whether a compressed parent's
	 * SKIP_X dual is re-emitted, while publishing into a slot that lives
	 * somewhere else entirely (ft_store_at_graft_point_commit's relocation
	 * republish).  Deriving the owner from @parent_nf there names a node the op
	 * does not hold and the record reports an exclusion gap that is not real.
	 */
	/*
	 * @expected_old: the value @parent_slot held in the snapshot the
	 * caller's publish plan was derived from (the RECORDED path only; the
	 * direct rec==NULL arms below republish a same value and ignore it).
	 * The compressed-parent SKIP_X dual mirrors the same child, so its
	 * plan-snapshot value is ft_skip_compressed_flag(expected_old, cn->len)
	 * -- both edges commit against the plan, not a record-time re-read.
	 */
	/*
	 * @new_child_meta (optional): @new_child's metadata, supplied by the
	 * caller so we DON'T recover it from the slot value.  Required when the
	 * caller defers @new_child's back-pointer into the same flip-txn as this
	 * publish: a SKIP_X @new_child is resolved to its compressed node via
	 * ft_skip_to_compressed, which reads new_child's child's parent -- which
	 * is precisely the deferred (not-yet-stored) back-edge.  Passing the
	 * metadata directly avoids that stale read.  NULL = recover as before
	 * (the back-edge was wired up front).
	 *
	 * @folded_child_prev (optional): the EXTERNAL analogue -- an external
	 * @new_child carries no metadata (its parent resolves through prev ->
	 * cell -> parent), so when the caller folds @new_child's prev into this
	 * publish's flip-txn (a head promote / swap), the forward-before-parent
	 * check below would read the not-yet-stored prev.  Passing the prev's
	 * intended value lets the check validate the folded parent instead.
	 * NULL = read new_child->prev as before.  Debug-check only.
	 *
	 * Publication-ordering invariant: a child becomes observable by
	 * downward traversal the instant it is published into a live parent
	 * slot, so its parent back-pointer MUST already be wired.  Otherwise
	 * a concurrent reader that descends to it and walks back up
	 * (ft_skip_reanchor / ordered up-walk) reads a NULL/uninitialized
	 * parent.  Applies to ALL child kinds (external -> prev,
	 * internal/compressed -> metadata->parent); the root slot is the sole
	 * exception (the root has no parent).  Catches forward-before-parent
	 * bugs at their source.
	 */
	/* Flight-recorder write-side mis-wire validator (tracing builds only). */
	FT_TRACE_PUB_CHECK(ft, parent_slot, new_child, 4);
#ifndef NDEBUG
	if (new_child && parent_slot != &ft->root) {
		struct cds_ft_inode_flag *cp;

		/*
		 * Check skip-compressed FIRST: a SKIP_X flag carries its
		 * (external) child's low tag bits, so ft_node_external() would
		 * misclassify it and dereference the tagged flag as a node.
		 * The caller-supplied metadata short-circuits the SKIP_X recovery
		 * (which would read the deferred back-edge).
		 */
		if (new_child_meta) {
			cp = ft_parent_node(new_child_meta->parent_word);
		} else
#ifdef FEATURE_FT_SKIP_COMPRESSED
		if (ft_node_skip_compressed(new_child)) {
			cp = ft_parent_node(cds_ft_item_to_metadata(
				(struct cds_ft_inode *)
				ft_skip_to_compressed(ft, new_child))->parent_word);
		} else
#endif
		if (ft_node_external(new_child)) {
			/*
			 * Cell-always: prev is the (non-NULL) cell pointer even
			 * when the parent is unset, so resolve through the cell to
			 * preserve the forward-before-parent check on cell->parent.
			 * When the caller folds the prev into this publish's txn it
			 * supplies the intended value (@folded_child_prev) so we
			 * validate the folded parent, not the not-yet-stored slot.
			 */
			cp = ft_resolve_head_prev(ft, folded_child_prev ?
				folded_child_prev :
				((struct cds_ft_node *) new_child)->prev);
		} else {
			cp = ft_parent_node(cds_ft_item_to_metadata(
				ft_node_ptr(new_child))->parent_word);
		}
		assert(cp != NULL);
	}
#endif /* !NDEBUG */
	(void) folded_child_prev;	/* debug-check only (see above) */
	/*
	 * Maintain @new_child's parent-slot offset (parent_slot_offset) so
	 * that it records the slot holding it within its parent node.  This
	 * is the value ft_get_parent_slot(child_meta) recovers later -- used by
	 * the parent-pointer backtrack to find a node's slot in O(1) without
	 * re-descending, by dual-pointer publishes from cn->child
	 * (ft_publish_to_parent itself, when called with parent_nf = cn) and
	 * by chain-merge canonicalization (ft_detach_node) that publishes a
	 * replacement at the same slot.
	 *
	 * Maintained for EVERY internal/compressed child (not just
	 * compressed): the offset field is no longer skip-specific.  On
	 * skip-on builds, without this, compressed nodes installed via
	 * ft_publish_to_parent (rather than via ft_node_set_nth, which routes
	 * through ft_set_parent) leave parent_slot_offset == 0 -- a latent gap
	 * that silently disabled the dual-pointer SKIP_X update at the
	 * grandparent slot and tripped chain-merge that *needs* the slot.
	 * On plain-internal builds, the same gap would break the
	 * parent-pointer backtrack's O(1) slot recovery.
	 *
	 * Externals carry no metadata / offset; skip them.  Test
	 * skip-compressed FIRST: a SKIP_X flag carries its external child's
	 * low tag bits, so ft_node_external() would misclassify it.
	 *
	 * We do NOT touch @new_child's parent linkage here; callers manage
	 * that via their own ft_set_parent (or by direct meta->parent
	 * assignment) before calling us.  ft_set_parent_slot computes the
	 * offset relative to child_meta->parent, which callers have already
	 * pointed at @parent_nf (the node holding @parent_slot).
	 *
	 * Skip the update at the root slot (&ft->root): root nodes have
	 * no parent, and ft_set_parent_slot's offset computation assumes
	 * the slot lives inside a node-arena chunk.
	 */
	if (new_child && parent_slot != &ft->root) {
		struct cds_ft_metadata *child_meta = new_child_meta;

		if (!child_meta) {
			if (ft_node_skip_compressed(new_child))
				child_meta = cds_ft_item_to_metadata(
					(struct cds_ft_inode *)
						ft_skip_to_compressed(ft, new_child));
			else if (!ft_node_external(new_child))
				child_meta = cds_ft_item_to_metadata(
					ft_node_ptr(new_child));
		}
		if (child_meta && ft_parent_node(child_meta->parent_word))
			ft_set_parent_slot(child_meta,
				ft_parent_node(child_meta->parent_word),
				parent_slot);
	}

	if (parent_nf && ft_node_compressed(parent_nf)) {
		struct cds_ft_compressed_node *cn =
			ft_compressed_node_ptr(parent_nf);
		struct cds_ft_metadata *cn_meta =
			cds_ft_item_to_metadata(
				(struct cds_ft_inode *) cn);

		/* Consumed via FEATURE_FT_SKIP_COMPRESSED and FT_TP only. */
		(void) cn;
		(void) cn_meta;

#ifdef FEATURE_FT_SKIP_COMPRESSED
		{
			/*
			 * READ-YOUR-OWN-WRITES: a recorded re-parent of @cn is
			 * invisible to a raw derivation, and this op may be
			 * relocating @cn's parent in the same commit that
			 * refreshes the dual.  See ft_txn_parent_slot.
			 */
			struct cds_ft_inode_flag *skip_owner_nf = NULL;
			struct cds_ft_inode_flag **skip_slot =
				ft_txn_parent_slot_at(cn_meta, ft,
					rec ? rec->mtxn : NULL,
					&skip_owner_nf);
#ifdef FT_DEBUG_DUAL_DROP
			ft_dbg_dual_probe(ft, cn, cn_meta, skip_slot,
				skip_owner_nf, expected_old, new_child);
#endif
			if (skip_slot &&
			    ft_node_skip_compressed(*skip_slot)) {
				struct cds_ft_inode_flag *skip_new =
					ft_skip_compressed_flag(new_child,
						cn->len);

				if (rec && !ft_dual_home_is_private(ft, rec,
						cn_meta, skip_slot,
						skip_owner_nf))
					/* A COMPRESSED ROOT's dual slot IS
					 * &ft->root (ft_txn_parent_slot's root
					 * arm), so ask rather than assume. */
					/*
					 * ☠ THE DUAL'S OWNER IS DERIVED HERE,
					 * NOT DECLARED BY THE CALLER: it is the
					 * GRANDPARENT, reached through
					 * @cn_meta's back-pointer, and this
					 * frame cannot know whether the op
					 * acquired it.  So the held answer is
					 * the caller's -- @dual_owner_held --
					 * and its default is false, which
					 * records MW.
					 */
					/*
					 * @pub_fn/@pub_line, not this frame's:
					 * the answer below was DECIDED by the
					 * caller, so an attribution naming
					 * _ft_publish_to_parent_meta names the
					 * messenger.
					 */
					ft_pub_rec_add_at(pub_fn, pub_line,
						rec, skip_slot,
						ft_skip_compressed_flag(
							expected_old, cn->len),
						skip_new,
						skip_slot == &ft->root,
						skip_slot == &ft->root ||
						!skip_owner_nf ? NULL :
						ft_flag_to_metadata(ft,
							skip_owner_nf),
						ft_pub_slot_excluded(ft,
							skip_slot,
							dual_owner_held));
				else if (*skip_slot != skip_new)
					rcu_assign_pointer(*skip_slot, skip_new);
			}
		}
#endif
		/*
		 * Re-emit compressed_publish so consumers tracking
		 * cn -> child relationships pick up the new subtree
		 * attached under this compressed node.  The initial
		 * creation-time compressed_publish event has
		 * parent = NULL (the compressed node is not yet
		 * attached); here we report cn_meta->parent since the
		 * compressed node is already in the trie.
		 */
		FT_TP(compressed_publish,
			(const void *) ft_compressed_node_flag(cn),
			cn->len,
			cn->key_bytes,
			(const void *) new_child,
			(const void *) ft_parent_node(CMM_LOAD_SHARED(cn_meta->parent_word)));
	}
	FT_TP(publish_to_parent, (const void *) parent_nf,
		(const void *) parent_slot,
		(const void *) *parent_slot,
		(const void *) new_child);
	/*
	 * When parent_slot points at ft->root, emit root_publish so
	 * consumers can track the top of the trie through root
	 * rewrites that have no structural parent node.
	 */
	if (parent_slot == &ft->root)
		FT_TP(root_publish, (const void *) ft,
			(const void *) new_child);
	if (rec)
		/*
		 * The FORWARD edge's owner is the caller's own @slot_owner_nf
		 * declaration -- the node it says @parent_slot lives in -- so
		 * naming it IS the held answer here, and the record-time owner
		 * assert is what checks it.  Only the DUAL above needs a
		 * separate word, because only its owner is derived.
		 */
		ft_pub_rec_add(rec, parent_slot, expected_old, new_child,
			parent_slot == &ft->root,
			parent_slot == &ft->root || !slot_owner_nf ? NULL :
				ft_flag_to_metadata(ft, slot_owner_nf),
			ft_pub_slot_excluded(ft, parent_slot,
				slot_owner_nf != NULL));
	else if (*parent_slot != new_child)
		/*
		 * Direct (rec == NULL) publish.  The only two callers -- the
		 * in-place relocation else in ft_attach_node and graft's
		 * no-recompact else -- republish the value the slot already
		 * holds (Invariant-1: no live reader-visible bare publish
		 * remains; the SKIP_X dual above is likewise a no-op in
		 * lockstep).  Eliding the redundant store and its release fence
		 * is invisible to readers; the guard keeps the store correct
		 * should a value ever differ.  The recorded (rec) path captures
		 * the edge for the flip-txn instead of storing here.
		 */
		rcu_assign_pointer(*parent_slot, new_child);
}

/*
 * Publish, recovering @new_child's metadata from the slot value (the
 * back-pointer was wired up front) -- the original behaviour.
 */
static
void _ft_publish_to_parent_at(const char *pub_fn, int pub_line,
		struct cds_ft *ft,
		struct cds_ft_inode_flag *parent_nf,
		struct cds_ft_inode_flag **parent_slot,
		struct cds_ft_inode_flag *new_child,
		struct cds_ft_inode_flag *expected_old,
		struct ft_pub_rec *rec,
		bool dual_owner_held)
{
	_ft_publish_to_parent_meta_at(pub_fn, pub_line, ft, parent_nf,
		parent_slot, new_child, expected_old, NULL, NULL, rec,
		/*slot_owner_nf=*/ parent_nf, dual_owner_held);
}

#define _ft_publish_to_parent(ft, parent_nf, parent_slot, new_child,	\
		expected_old, rec, dual_owner_held)			\
	_ft_publish_to_parent_at(__func__, __LINE__, (ft), (parent_nf),	\
		(parent_slot), (new_child), (expected_old), (rec),	\
		(dual_owner_held))

#define _ft_publish_to_parent_meta(ft, parent_nf, parent_slot, new_child, \
		expected_old, new_child_meta, folded_child_prev, rec,	\
		slot_owner_nf, dual_owner_held)				\
	_ft_publish_to_parent_meta_at(__func__, __LINE__, (ft),		\
		(parent_nf), (parent_slot), (new_child), (expected_old),	\
		(new_child_meta), (folded_child_prev), (rec),		\
		(slot_owner_nf), (dual_owner_held))

/*
 * Direct publish (original behaviour): perform the stores immediately.
 * rec == NULL, so @expected_old is unused (the direct arm republishes the
 * value already present); pass the live slot value to satisfy the interface.
 */
static
void ft_publish_to_parent(struct cds_ft *ft,
		struct cds_ft_inode_flag *parent_nf,
		struct cds_ft_inode_flag **parent_slot,
		struct cds_ft_inode_flag *new_child)
{
	_ft_publish_to_parent(ft, parent_nf, parent_slot, new_child,
		*parent_slot, NULL, false);
}

/*
 * ft_publish_compressed: convert a compressed node flag to a skip
 * pointer if skip-compressed mode is enabled, the path length fits,
 * and the child has metadata (is not external).
 *
 * Call AFTER ft_set_parent has been done with the real compressed
 * flag (@cflag).  The returned value is what should be
 * published/stored in parent child slots.
 */
static
struct cds_ft_inode_flag *ft_publish_compressed(struct cds_ft *ft,
		struct cds_ft_compressed_node *cn,
		struct cds_ft_inode_flag *cflag)
{
	/*
	 * Emit creation-time compressed_publish so trace consumers
	 * learn the cn->child binding for every newly-allocated
	 * compressed node, regardless of which creation path built
	 * it (ft_build_compressed_node, compressed-split sfx/pfx/nb,
	 * graft-split suffix/prefix).  parent is NULL here: the cn
	 * is about to be returned to the caller for attachment;
	 * cn_meta->parent is still unset.  A subsequent
	 * ft_publish_to_parent / ft_node_set_nth on the slot that
	 * holds this cn fires tree_edge_set (with the cn as child),
	 * which -- paired with this event -- gives the consumer both
	 * ends: the parent->cn edge and the cn->child edge.
	 */
	FT_TP(compressed_publish,
		(const void *) ft_compressed_node_flag(cn),
		cn->len,
		cn->key_bytes,
		(const void *) cn->child,
		(const void *) NULL);
	if (ft_group_skip_compressed(ft->group) &&
	    cn->len <= FT_SKIP_LEN_MAX) {
		return ft_skip_compressed_flag(cn->child, cn->len);
	}
	return cflag;
}

/*
 * ft_set_parent: set the parent pointer in child's metadata.
 * Skips NULL children.
 *
 * External (leaf) nodes: sets cds_ft_node.prev (head of duplicate chain).
 *
 * For skip-compressed pointers: the skip pointer represents a
 * compressed node in the trie.  Set the compressed node's parent
 * (not the compressed node's child's parent, which is the
 * compressed node itself and was set at creation time).
 *
 * Skip-compressed must be checked before external: a skip pointer
 * whose child is external has low tag bits == 0, which would match
 * ft_node_external on the raw value.
 *
 * Write-side only (mutex-held).
 */
/*
 * ft_head_stamp_incoming_byte: maintain an EXTERNAL head's up-walk edge byte at
 * a child placement -- the external counterpart of what ft_set_parent's plain
 * internal branch and ft_reparent_record_meta already do for a node.
 *
 * ☠ THE FIELD'S CONTRACT IS "MAINTAINED AT EVERY CHILD PLACEMENT"
 * (fractal-trie-internal.h, @incoming_byte), and for a head the record is its
 * CELL -- "external head: stored in the head's CELL metadata ... set to the
 * key's last byte at insert".  Both placement primitives used to set only the
 * head's PARENT word and leave the byte at whatever the INSERT stamped, on the
 * argument written at ft_reparent_record's external arm that a head has "no
 * metadata / offset".  It has no NODE metadata; the cell is its metadata, and
 * ft_rebuild_key_upwalk reads exactly this field out of it.
 *
 * MEASURED, single-threaded on a quiet trie, ordered list ON, default build,
 * release included: move a BARE EXTERNAL HEAD "q" with cds_ft_rekey_merge onto
 * an empty destination and a plain forward walk (cds_ft_lookup_first +
 * cds_ft_next) emits a key THAT IS NOT IN THE TRIE -- the destination's bytes
 * for every level but the last, and the SOURCE's last byte at the end:
 * dst "z" -> "q", dst "ax" -> "aq", dst "axy" -> "axq".  cds_ft_verify is
 * CLEAN and cds_ft_count_keys is right, so only a lookup of the emitted key can
 * see it.  Five of the six graft legs did it, the BRANCH leg included, and the
 * survivors were LATENT rather than correct: dst "axy" walks fine on the default
 * build only because the head hangs under a COMPRESSED parent, whose key_bytes
 * cover the byte -- insert "axz" to split that run and the same trie starts
 * spelling "axq".
 *
 * ☠ STAMP IT WHETHER OR NOT THE CURRENT PARENT READS IT.  Under a COMPRESSED
 * parent the walk takes the byte from key_bytes and never looks -- but the run
 * can be SPLIT later by an ordinary insert, which drops the head into a body
 * slot of a fresh internal node and re-homes nothing it would have to stamp, so
 * the walk then reads whatever was left behind.  That is not hypothetical: with
 * only the plain-parent arm stamped, dst "axy" walked clean and then
 * `insert "axz"` made the same trie spell "axq".  The value is the last byte of
 * the head's KEY in both cases -- the run's last key byte under a run, the slot
 * byte under a plain parent -- which is what the two cross-trie sites stamp
 * directly as okey_dst[dst_key_len - 1].
 *
 * The two real exclusions: @ft->ordered_list, because with the list off there is
 * no cell to hold the byte (and the walk uses the iterator's own descent buffer
 * instead); and a PREFIX head, whose key ends AT the parent so it HAS no edge
 * byte -- the same @slot-is-the-shape reading ft_head_parent_word_slot makes,
 * kept in one predicate here so the two cannot disagree about which placement
 * they are describing.
 *
 * ☞ BYTE BEFORE PARENT, which is the order ft_set_parent's internal branch
 * already argues for at its own stamp ("Publish the up-walk key byte BEFORE the
 * parent pointer ... a concurrent up-walk that follows the new parent would read
 * the still-stale byte"): a reader that reaches the head through its NEW parent
 * must not find the OLD byte.
 *
 * ★★★ WHY A PLAIN STORE IS THE RIGHT SHAPE HERE, AND WHY THE BYTE MUST NOT BE
 * PUT IN THE TXN.  THE RULE A REKEY WORKS BY IS THAT THE MOVED TOP GETS A FRESH
 * ADDRESS -- ft_rekey_cow_stop's whole purpose -- so every back edge written on
 * it, the parent word and this byte alike, is a plain store into an object NO
 * READER CAN SEE YET, published atomically by the one commit.  That is what
 * makes txn membership unnecessary, and ft_store_at_graft_point_commit states
 * it at the sibling store: "A plain store, because the payload is INVISIBLE
 * until the forward publish below -- ... the rekey fold hands over a fresh COW
 * copy."  A byte that rode the commit would be solving a problem the COW has
 * already solved for every top that gets one.
 *
 * ☠ THE ONE TOP THAT GETS NO COPY IS THE BARE EXTERNAL HEAD: it is APP-OWNED,
 * so @s_top_prime == @s_top and the writes land on a LIVE object that stays
 * reachable at the SOURCE for the whole build window.  That is exactly what
 * @payload_live exists for -- it routes the head's PARENT word through the txn
 * instead of plain-storing it (ft-graft.h) -- and the byte has no such route.
 * So for a bare head, and ONLY for it, this store is reader-visible ahead of the
 * publish and is not undone by an abort.
 *
 * ☞ AND THE CURE IS THE SAME RULE ONE LEVEL DOWN, not a transacted byte.  The
 * byte does not live in the application's cds_ft_node: it lives in the head's
 * ORDERED CELL, which is LIBRARY-allocated (ft_ord_cell_alloc) and therefore
 * CAN be copied.  Giving the moved head a FRESH CELL -- stamped while invisible,
 * published by the same commit that re-parents it, the old cell retired after a
 * grace period -- restores the fresh-address property for the one word that
 * lacks it, and rides the ordered-list splice the rekey already performs for
 * that cell.  Transacting @incoming_byte would not work anyway: it shares a
 * 32-bit word with @alloc_index, with no room for a parked descriptor pointer,
 * and the up-walk reads it raw.
 */
static inline
void ft_head_stamp_incoming_byte(const struct cds_ft *ft,
		struct cds_ft_node *en,
		struct cds_ft_inode_flag *parent_nf,
		struct cds_ft_inode_flag **slot)
{
	struct cds_ft_inode *parent;
	void *prev;

	struct cds_ft_compressed_node *cn = NULL;
	uint8_t byte;

	if (!ft->ordered_list || !slot || !parent_nf)
		return;
	if (ft_node_flip_proxy(parent_nf))
		return;
	/*
	 * KIND DISPATCH, SKIP FIRST and BEFORE the FT_INTERNAL_MASK test: a
	 * compressed flag has that bit CLEAR, so testing it up front would drop
	 * exactly the run-parent arm below.
	 */
#ifdef FEATURE_FT_SKIP_COMPRESSED
	if (ft_node_skip_compressed(parent_nf))
		cn = ft_skip_to_compressed(ft, parent_nf);
	else
#endif
	if (ft_node_compressed(parent_nf))
		cn = ft_compressed_node_ptr(parent_nf);
	else if (!((uintptr_t) parent_nf & FT_INTERNAL_MASK))
		return;
	if (cn) {
		/*
		 * A COMPRESSED parent's key_bytes already cover this byte, so the
		 * walk does not read it HERE -- but it will the moment a plain
		 * insert SPLITS the run and drops the head into a body slot of the
		 * fresh internal node, and that split re-homes nothing it would
		 * have to stamp: the value it finds is whatever was left behind.
		 * So the byte is maintained at every placement whether or not the
		 * CURRENT parent consults it.  It is the last byte of the head's
		 * key either way, which under a run is the run's last key byte.
		 */
		if (!cn->len)
			return;
		byte = cn->key_bytes[cn->len - 1];
	} else {
		parent = ft_node_ptr(parent_nf);
		if ((void *) slot == (void *)
				&cds_ft_item_to_metadata(parent)->external_nodes)
			return;		/* a PREFIX head: its key ends AT the parent */
		byte = ft_slot_to_byte(&ft_types[ft_node_type(parent_nf)],
				parent, slot);
	}
	prev = ft_dereference_prev_resolved(en);
	/*
	 * FT_ORD_CELL_TAG: only a duplicate-chain HEAD owns a cell.  A demoted
	 * duplicate's prev names its chain predecessor LEAF and must not be read
	 * as one -- see the same guard at the rekey's own head stamp.
	 */
	if (!prev || !((uintptr_t) prev & FT_ORD_CELL_TAG))
		return;
	cds_ft_item_to_metadata(ft_ord_cell_ptr(prev))->incoming_byte = byte;
}

/*
 * ft_set_parent: set the parent pointer in child's metadata,
 * and optionally set skip_slot for skip-compressed children.
 *
 * @child_nf:  child node flag (may be skip-compressed, external, etc.)
 * @parent_nf: parent node flag to record.
 * @slot:      address of the slot in the parent that holds @child_nf.
 *             When @child_nf is skip-compressed and @slot is non-NULL,
 *             the compressed node's skip_slot is set to @slot.
 *             Pass NULL when the slot is unknown or irrelevant.
 */
static
void ft_set_parent_at(const char *fn, int line, struct cds_ft *ft,
		struct cds_ft_inode_flag *child_nf,
		struct cds_ft_inode_flag *parent_nf,
		struct cds_ft_inode_flag **slot, enum ft_word_excl excl)
{
	(void) excl;	/* consumed by the word-class audits only */
	/*
	 * @fn/@line are the CALLER's, so the head parent-word audit below gets
	 * one row per CALL SITE.  ft_set_parent has 34 callers but the external
	 * arm -- the only one that writes a head's parent word -- is reached by
	 * far fewer, and which ones they are is a question to MEASURE rather
	 * than classify by inspection.
	 */
	/*
	 * A NULL @parent_nf is the root position (the publish goes into
	 * &ft->root): store the OWNING TRIE there rather than NULL, so the
	 * back-edge identifies its owner at depth 0 as it does everywhere else.
	 * @parent_nf itself stays as passed for the slot offset (a root has
	 * none) and for an external child, whose holder is always a real node.
	 */
	struct cds_ft_inode_flag *stored_parent = parent_nf ?
		parent_nf : ft_trie_parent(ft);

	if (!child_nf)
		return;
	/*
	 * A type-7 flip proxy is a transient slot VALUE (a one-commit insert
	 * or merge flip in progress), not a node: the REAL child's
	 * back-pointer is wired by the parking mutator itself.  No-op so the
	 * generic re-parent loops (recompact's child sweep, set_nth's
	 * post-store wiring) flow over a parked slot unharmed -- dispatching
	 * below would misread the proxy latch as internal-node metadata.
	 */
	if (caa_unlikely(ft_node_flip_proxy(child_nf)))
		return;
	FT_TP(set_parent, (const void *) child_nf, (const void *) parent_nf);
#ifdef FEATURE_FT_SKIP_COMPRESSED
	if (ft_node_skip_compressed(child_nf)) {
		struct cds_ft_compressed_node *cn =
			ft_skip_to_compressed(ft, child_nf);
		struct cds_ft_metadata *cn_meta =
			cds_ft_item_to_metadata(
				(struct cds_ft_inode *) cn);
		ft_ch_audit_parent_at(fn, line, ft, cn_meta, parent_nf, excl);
		rcu_assign_pointer(cn_meta->parent_word, stored_parent);
		ft_set_parent_slot(cn_meta, parent_nf, slot);
		return;
	}
	if (ft_node_compressed(child_nf)) {
		/*
		 * Plain COMPRESSED form (no SKIP_X wrap): typically
		 * arises when ft_publish_compressed gates SKIP-X off
		 * for a non-spec EXT child.  Maintain the cn's
		 * parent_slot_offset just like the SKIP_X branch above so
		 * later ft_publish_to_parent / chain-merge calls can
		 * recover the slot in cn's parent via
		 * ft_get_parent_slot.
		 */
		struct cds_ft_compressed_node *cn =
			ft_compressed_node_ptr(child_nf);
		struct cds_ft_metadata *cn_meta =
			cds_ft_item_to_metadata(
				(struct cds_ft_inode *) cn);
		ft_ch_audit_parent_at(fn, line, ft, cn_meta, parent_nf, excl);
		rcu_assign_pointer(cn_meta->parent_word, stored_parent);
		ft_set_parent_slot(cn_meta, parent_nf, slot);
		return;
	}
#endif
	if (ft_node_external(child_nf)) {
		/*
		 * Ordered list on: the head carries its cell in prev; record the
		 * parent into cell->parent (fresh head: cell pre-wired at insert;
		 * existing head re-parent: cell already present).  List off / non-cell:
		 * the parent is the head's prev directly.  rcu_assign either way:
		 * ft_set_parent re-parents live heads on the restructure path.
		 */
		struct cds_ft_node *en = (struct cds_ft_node *) child_nf;
		/*
		 * @slot settles the head's shape: a BODY SLOT in @parent_nf
		 * makes it a slot head, the external_nodes word (named, or NULL)
		 * a prefix head -- ft_head_parent_word_slot's header for why
		 * NULL means that here (FT_PARENT_PREFIX_HEAD).
		 */
		struct cds_ft_inode_flag *word =
			ft_head_parent_word_slot(parent_nf, slot);

		/* The up-walk edge byte, BEFORE the parent word: see the helper. */
		ft_head_stamp_incoming_byte(ft, en, parent_nf, slot);
		/* Same word class as the prefix-head store above. */
		ft_ch_audit_head_at(fn, line, ft, en, parent_nf, excl);
		if (ft->ordered_list) {
			ft_ord_cell_set_parent(en, word);
		} else {
			FT_CHAIN_CANARY_RAW(&en->prev, 2);
			rcu_assign_pointer(en->prev, word);
		}
		return;
	}
	{
		/*
		 * Plain internal child: record its parent AND its slot offset
		 * within the parent, so the parent-pointer backtrack can recover
		 * the slot in O(1) (ft_get_parent_slot) without re-descending.
		 * ft_set_parent_slot reads meta->parent, so set it first.
		 */
		struct cds_ft_metadata *meta =
			cds_ft_item_to_metadata(ft_node_ptr(child_nf));

		/*
		 * Publish the up-walk key byte BEFORE the parent pointer.  A node
		 * re-homed from a COMPRESSED parent (which skips incoming_byte,
		 * leaving it 0) to an INTERNAL parent gets its real branch byte
		 * here.  If we published meta->parent first (as ft_set_parent_slot
		 * needs, to compute the offset) a concurrent up-walk that follows
		 * the new parent would read the still-stale 0 byte and reconstruct
		 * a key with a hole at this level.  Pre-store it under the explicit
		 * @parent_nf and let the rcu_assign release order it; ft_set_parent_
		 * slot below recomputes the same byte (idempotent) plus the offset.
		 */
		if (slot && parent_nf && !ft_node_compressed(parent_nf)
#ifdef FEATURE_FT_SKIP_COMPRESSED
				&& !ft_node_skip_compressed(parent_nf)
#endif
		   )
			meta->incoming_byte = FT_DT_IB_STORE(meta, ft_slot_to_byte(
				&ft_types[ft_node_type(parent_nf)],
				ft_node_ptr(parent_nf), slot));
		ft_ch_audit_parent_at(fn, line, ft, meta, parent_nf, excl);
		rcu_assign_pointer(meta->parent_word, stored_parent);
		ft_set_parent_slot(meta, parent_nf, slot);
	}
}

#define ft_set_parent(ft, child_nf, parent_nf, slot)			\
	ft_set_parent_at(__func__, __LINE__, (ft), (child_nf),		\
		(parent_nf), (slot), FT_EXCL_UNDECLARED)
/*
 * The declaring spelling: the caller states WHICH REGIME makes this write legal
 * (enum ft_word_excl).  Only convert a site whose answer the code ITSELF
 * establishes.
 */
#define ft_set_parent_excl(ft, child_nf, parent_nf, slot, excl)		\
	ft_set_parent_at(__func__, __LINE__, (ft), (child_nf),		\
		(parent_nf), (slot), (excl))

/*
 * Return codes for compressed node traversal helpers.
 * Used to tell callers which loop control action to take.
 */
enum ft_descent_action {
	FT_DESCENT_CONTINUE,		/* Continue loop iteration. */
	FT_DESCENT_BREAK,		/* Break from loop. */
	FT_DESCENT_END,		/* Jump to function end (status set). */
	FT_DESCENT_GOING_UP,		/* Jump to going_up backtracking. */
	FT_DESCENT_DESCEND_CHILDREN,	/* Jump to descend_children. */
	FT_DESCENT_FOUND_MINMAX,	/* Jump to found_minmax label. */
};

/*
 * Compare @cmp key bytes starting at @key against the compressed
 * node's path.  Returns the number of matching bytes.  A return
 * value == @cmp means full match; < @cmp means divergence at that
 * position.
 */
static inline
unsigned int ft_match_compressed_key(const uint8_t *key,
		const struct cds_ft_compressed_node *cn,
		unsigned int cmp)
{
	unsigned int pos;

	if (ft_key_cmp_ordinals(key, cn->key_bytes, cmp, cmp, false, &pos) != 0)
		return pos;
	return cmp;
}

/*
 * Fill ordinal_key for every level spanned by a compressed node.  Used
 * by read-side descent loops (lookup_nth, minmax, etc.) to record the
 * ordinal key bytes through compressed nodes; the going-up backtrack
 * recovers per-level nodes from the live parent chain, not a path array.
 */
static inline
void ft_fill_compressed_path(struct cds_ft_compressed_node *cn,
		uint8_t *ordinal_key, int base)
{
	int j;

	for (j = 0; j < cn->len; j++)
		ordinal_key[base + j] = cn->key_bytes[j];
}

static
bool valid_external_node(struct cds_ft_node *node)
{
	return node != NULL && ft_node_external((struct cds_ft_inode_flag *) node);
}

/*
 * Return the metadata of the root node.
 *
 * ft->root always points to an arena-allocated internal node, even
 * when the trie is empty (nr_child == 0).  The node itself may be
 * replaced by graft or graft-swap, but the invariant on the slot
 * is maintained across all operations.  Its metadata holds:
 *   - nr_child:       number of children in the root node.
 *   - external_nodes: list of NIL-key (key_len == 0) entries.
 *
 * The root is a regular internal node whose metadata is accessed the
 * same way as any other node's.  Its metadata carries the NIL-key
 * entries, so transplanting a root node between tries is a single
 * pointer swap with no metadata relocation.
 *
 * This function is only meant to be used from update functions, _not_
 * safe for use by read-side.
 */
static inline
struct cds_ft_metadata *ft_root_metadata(const struct cds_ft *ft)
{
	/*
	 * Resolve a peer's parked flip proxy on the root slot (Phase 4.3, a
	 * root recompact mid-commit): ft_node_ptr on the raw proxy would mask
	 * the tag and hand back a RECORD address as a node.
	 */
	return cds_ft_item_to_metadata(ft_node_ptr(
		ft_resolve_flip_proxy(rcu_dereference(ft->root))));
}

static
bool valid_key_len(struct cds_ft *ft, size_t key_len)
{
	size_t max_key_len = ft->group->max_key_len;

	assert(max_key_len != CDS_FT_MAX_LEN_UNLIMITED);
	if (key_len == CDS_FT_LEN_ERROR || key_len > max_key_len)
		return false;
	return true;
}

static
struct cds_ft_inode *alloc_cds_ft_node(struct cds_ft *ft,
		const struct cds_ft_type *ft_type,
		struct cds_ft_metadata **_metadata)
{
	struct cds_ft_metadata *metadata;
	void *p;

	metadata = cds_ft_alloc_item(ft, ft_type->order, ft_type->bitmap);
	if (!metadata) {
		return NULL;
	}
	/* A NEW INCARNATION of this address begins here (see FT_LL_REUSE). */
	FT_LL_MARK_REUSE(metadata);
	p = cds_ft_metadata_to_item(metadata);
	FT_TP(item_alloc, (const void *) p, 0, ft_type->order);
#ifdef FT_DEBUG_DEL_TOMB
	{
		static unsigned long ft_dt_alloc_stale;

		if (CMM_LOAD_SHARED(metadata->state) != 0 &&
				uatomic_add_return(&ft_dt_alloc_stale, 1) <= 20)
			fprintf(stderr, "FT DT-ALLOC-STALE node=%p meta=%p state=%#lx\n",
				p, (void *) metadata,
				(unsigned long) CMM_LOAD_SHARED(metadata->state));
	}
#endif
	/*
	 * Popcount node data[] starts with a presence bitmap, followed
	 * by the pointer table.  The allocator returns zeroed memory,
	 * which is the initial "no children" state (bitmap = 0, so all
	 * lookups return NULL; nr_child derived from popcount returns 0).
	 */
	if (ft_debug_counters()) {
		uatomic_inc(&ft->group->nr_nodes_allocated);
		uatomic_inc(&ft->group->nr_internal_alloc);
	}
	*_metadata = metadata;
	return p;
}

static
void free_cds_ft_node(struct cds_ft *ft, struct cds_ft_inode *node)
{
	struct cds_ft_metadata *metadata = cds_ft_item_to_metadata(node);

	FT_LL_MARK_FREED(metadata);

	FT_TP(item_free, (const void *) node, 0);
	FT_TP(item_retire, (const void *) node, __builtin_return_address(0));

#ifdef FT_DEBUG_TOMBSTONE_AUDIT
	/*
	 * Freeze-on-free guard (doc §4.B, STEP 3): every PUBLISHED node retired
	 * through this path must carry its one-way LIVE->DEAD tombstone, set
	 * BEFORE the unlink commit that detached it.  Abandoned fresh
	 * (never-reader-visible) nodes use free_cds_ft_node_unpublished and do
	 * not reach here.  Build with -DFT_DEBUG_TOMBSTONE_AUDIT to enforce that
	 * no retire site is added without a mark (a no-op under one writer).
	 */
	assert(ft_meta_tombstone(metadata));
#endif
	cds_ft_free_item(ft, metadata);
	if (ft_debug_counters() && node) {
		uatomic_inc(&ft->group->nr_nodes_freed);
		uatomic_inc(&ft->group->nr_internal_freed);
	}
}

/*
 * Immediate-free variant for internal nodes that never escape the
 * writer's stack (e.g., nodes built by an attach/split/recompact
 * helper but freed by an -ENOMEM error path before publication).
 * See cds_ft_free_item_unpublished for the safety contract.
 */
static
void free_cds_ft_node_unpublished(struct cds_ft *ft, struct cds_ft_inode *node)
{
	struct cds_ft_metadata *metadata;

	/*
	 * ☠ FREEING NOTHING IS NOT FREEING ADDRESS ZERO.  The counter update
	 * below already guards on @node, so this function was always meant to
	 * tolerate a NULL -- but the metadata derivation ran first and
	 * unconditionally, and cds_ft_item_to_metadata(NULL) does not fault on
	 * the pointer: it indexes the metadata array of the range that WOULD
	 * contain address 0 and reads 0x200010.
	 *
	 * ft_node_recompact's @abandon_fresh unwind reaches here with
	 * @new_node == NULL whenever the attempt was sized to NODE_INDEX_NULL
	 * (a DEL whose post-lock re-read leaves the copy no children) and then
	 * bailed -- the two plan checks in the DEL block bail to it, and so does
	 * the NODE_INDEX_NULL refusal beside them.  MEASURED: 8 SIGSEGVs in 200
	 * short runs of inv_concurrent_remove_all_nolist, all at
	 * cds_ft_item_to_metadata(p=0x0) from this call.
	 */
	if (!node)
		return;
	metadata = cds_ft_item_to_metadata(node);

	FT_TP(item_free, (const void *) node, 2);
	cds_ft_free_item_unpublished(ft, metadata);
	if (ft_debug_counters() && node) {
		uatomic_inc(&ft->group->nr_nodes_freed);
		uatomic_inc(&ft->group->nr_internal_freed);
	}
}

/*
 * Compute the arena allocation order for a compressed node with
 * @path_len key bytes.  The compressed node layout is:
 *   [child pointer] [len byte] [key_bytes...]
 */
static
unsigned int ft_compressed_order(uint8_t path_len)
{
	size_t size = offsetof(struct cds_ft_compressed_node, key_bytes) + path_len;
	int order = urcu_get_count_order_ulong(size);

	if (order < 4)
		order = 4;	/* Minimum arena order. */
	return (unsigned int) order;
}

static
struct cds_ft_compressed_node *alloc_compressed_node(struct cds_ft *ft,
		uint8_t path_len,
		struct cds_ft_metadata **_metadata)
{
	struct cds_ft_metadata *metadata;
	void *p;
	unsigned int order = ft_compressed_order(path_len);

	metadata = cds_ft_alloc_compressed_item(ft, order);
	if (!metadata)
		return NULL;
	p = cds_ft_metadata_to_item(metadata);
	FT_TP(item_alloc, (const void *) p, 1, path_len);
	if (ft_debug_counters()) {
		uatomic_inc(&ft->group->nr_nodes_allocated);
		uatomic_inc(&ft->group->nr_compressed_alloc);
	}
	*_metadata = metadata;
	return p;
}

static
void free_compressed_node(struct cds_ft *ft,
		struct cds_ft_compressed_node *node)
{
	struct cds_ft_metadata *metadata =
		cds_ft_item_to_metadata((struct cds_ft_inode *) node);

	FT_TP(item_free, (const void *) node, 1);

#ifdef FT_DEBUG_TOMBSTONE_AUDIT
	/* See free_cds_ft_node: freeze-on-free guard (doc §4.B). */
	assert(ft_meta_tombstone(metadata));
#endif
	FT_TP(compressed_free, (const void *) ft_compressed_node_flag(node));
	cds_ft_free_item(ft, metadata);
	if (ft_debug_counters() && node) {
		uatomic_inc(&ft->group->nr_nodes_freed);
		uatomic_inc(&ft->group->nr_compressed_freed);
	}
}

/*
 * Immediate-free variant for compressed nodes that never escape the
 * writer's stack (e.g., -ENOMEM error paths in build/split helpers).
 * See cds_ft_free_item_unpublished for the safety contract.
 */
static
void free_compressed_node_unpublished(struct cds_ft *ft,
		struct cds_ft_compressed_node *node)
{
	FT_TP(item_free, (const void *) node, 3);
	struct cds_ft_metadata *metadata =
		cds_ft_item_to_metadata((struct cds_ft_inode *) node);

	FT_TP(compressed_free, (const void *) ft_compressed_node_flag(node));
	cds_ft_free_item_unpublished(ft, metadata);
	if (ft_debug_counters() && node) {
		uatomic_inc(&ft->group->nr_nodes_freed);
		uatomic_inc(&ft->group->nr_compressed_freed);
	}
}

#define __FT_ALIGN_MASK(v, mask)	(((v) + (mask)) & ~(mask))
#define FT_ALIGN(v, align)		__FT_ALIGN_MASK(v, (typeof(v)) (align) - 1)
#define __FT_FLOOR_MASK(v, mask)	((v) & ~(mask))
#define FT_FLOOR(v, align)		__FT_FLOOR_MASK(v, (typeof(v)) (align) - 1)

/*
 * Push a node and its depth onto the snapshot stack, maintaining the
 * parallel snapshot_depth[] array alongside snapshot[].
 */
#define ft_snapshot_push(snap, snap_depth, nr, node_flag, depth)	\
	do {								\
		(snap_depth)[(nr)] = (depth);				\
		(snap)[(nr)++] = (node_flag);				\
	} while (0)


/* Human-readable name for a cds_ft_status code (diagnostics, tests). */
const char *cds_ft_status_to_string(enum cds_ft_status status)
{
	switch (status) {
	/* Success return codes (>= 0). */
	case CDS_FT_STATUS_OK:
		return "Operation completed successfully";
	case CDS_FT_STATUS_NOT_FOUND:
		return "No node found";
	case CDS_FT_STATUS_DUPLICATE_FOUND:
		return "Duplicate node exists";
	case CDS_FT_STATUS_INTERNAL_MATCH:
		return "Match ends at an internal node";

	/* Error return codes (< 0). */
	case CDS_FT_STATUS_INVALID_ARGUMENT_ERROR:
		return "Invalid argument";
	case CDS_FT_STATUS_MEMORY_ERROR:
		return "Memory allocation failure";
	case CDS_FT_STATUS_OVERFLOW_ERROR:
		return "Buffer too small for key length";
	case CDS_FT_STATUS_BUSY_ERROR:
		return "Resource busy";
	case CDS_FT_STATUS_POPULATED_ERROR:
		return "Destination already populated";
	case CDS_FT_STATUS_INTEGRITY_ERROR:
		return "Integrity verification failure";
	case CDS_FT_STATUS_NOT_SUPPORTED:
		/*
		 * The enum documents BOTH halves ("unavailable for this trie's
		 * configuration ... or feature not compiled in"); the string used
		 * to name only the second, so a SHAPE refusal read as a build
		 * problem and sent the reader looking for a missing -D.
		 */
		return "Unsupported for this trie or shape";

	default:
		return "Unknown status value";
	}
}

/* Emit @level indentation tabs to @out (shared by the show + stats renderers). */
static
void print_indent(FILE *out, int level)
{
	int i;

	for (i = 0; i < level; i++)
		fprintf(out, "	");
}
