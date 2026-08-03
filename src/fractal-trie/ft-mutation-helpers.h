// SPDX-FileCopyrightText: 2012-2026 Mathieu Desnoyers <mathieu.desnoyers@efficios.com>
//
// SPDX-License-Identifier: LGPL-2.1-only

/*
 * src/fractal-trie/ft-mutation-helpers.h
 *
 * Userspace RCU library - Fractal Trie: shared write-path (mutation) helpers.
 *
 * The ordered-list maintenance subsystem: the flip-latch batch and the
 * ordinal-cell flip / splice / swap / run / find operations that keep the
 * key-ordered cell list consistent under the writer lock.  Used by insert,
 * remove, graft, detach and merge alike, so it lives in one module ahead of
 * them rather than parked in any single bulk-op file.
 *
 * Implementation unit: #included once by fractal-trie.c, in dependency order
 * into a single translation unit (preserves cross-module inlining).  It sits
 * after ft-inequality.h and the shared-scanner redirect, before ft-insert.h,
 * so its scanner / inequality calls route through the shared copies the rest
 * of the write path uses.  Not a standalone header.
 */
#ifndef FRACTAL_TRIE_IMPL
#error "ft-mutation-helpers.h is an implementation unit; #include it from fractal-trie.c only"
#endif

/*
 * Descent cursor -- tracks current, parent, and grandparent positions
 * during a key-guided traversal of the trie.
 *
 * Each level stores both the flagged-pointer value (nf / pnf / ppnf)
 * and the address of the slot that holds it (nfp / pnfp / ppnfp).
 * Callers that do not need every field may leave the unused ones
 * NULL; the struct carries the superset so that a single descent
 * helper can serve graft, insert, remove, and detach paths.
 */
struct ft_descent {
	unsigned int depth;			/* Levels traversed (0 .. key_len). */
	struct cds_ft_inode_flag *nf;		/* Current node-flag value. */
	struct cds_ft_inode_flag **nfp;		/* Slot that holds @nf. */
	struct cds_ft_inode_flag *pnf;		/* Parent node-flag value. */
	struct cds_ft_inode_flag **pnfp;	/* Slot that holds @pnf. */
	struct cds_ft_inode_flag *ppnf;		/* Grandparent node-flag value. */
	struct cds_ft_inode_flag **ppnfp;	/* Slot that holds @ppnf. */
	struct cds_ft_inode_flag *pppnf;	/* Great-grandparent node-flag value. */
	struct cds_ft_inode_flag **pppnfp;	/* Slot that holds @pppnf. */
	/*
	 * A reanchoring descent step (ft_descent_step) landed the live node
	 * SHALLOWER than the dispatched child (ft_skip_reanchor rewind > 0: a
	 * peer chain-merge moved the encoded position up), so the captured
	 * publish slot @nfp is at the wrong level.  A mutating caller must
	 * re-descend against the now-current tree; a caller that ignores it
	 * still navigates a LIVE node (no torn read) and its txn commit aborts
	 * any incoherent publish.  Set only, never cleared mid-descent -- the
	 * first mutator that observes it bails.
	 */
	bool skip_conflict;			/* Reanchor moved the slot's level. */
};

/*
 * Optional (parent, slot) override for a recompact's inherited edge.  A
 * cross-trie graft passes its reanchoring descent's coherent grandparent pair
 * (d->ppnf, d->pnfp) so ft_node_recompact homes the fresh copy under the LIVE
 * reanchored parent, NOT the recompacted node's lazily-updated back-pointer.
 * That back-pointer dangles once a shared-spine peer FREES the old parent: the
 * children reanchor lazily via the parked proxy, but a freed parent leaves no
 * proxy to follow, and ft_resolve_parent_slot then recovers a reclaimed node
 * (§11 cross-trie Defect C -- a wild store into a rank-N slot of a 0-child
 * recycled node).  The descent pair, captured coherently in one reanchored
 * step and RCU-pinned for the writer's read-side, names the live parent gp'
 * (or is retired-but-tombstoned -> the recompact's lock acquire rejects it ->
 * -EAGAIN re-descend).  A NULL @parent degrades to a publish into &ft->root
 * (a root-level graft point: d->ppnf == NULL, d->pnfp == &ft->root by the
 * descent shift).
 */
struct ft_parent_hint {
	struct cds_ft_inode_flag *parent;	/* d->ppnf (NULL => &ft->root). */
	struct cds_ft_inode_flag **slot;	/* d->pnfp (a slot in @parent). */
	/*
	 * SKIP_X dual coherence (§11 cross-trie Defect C, one level up).  When
	 * the recompacted node's parent @parent is a COMPRESSED node carrying a
	 * SKIP_X dual, ft_node_recompact re-encodes that dual -- a slot in
	 * @parent's OWN parent (the great-grandparent) -- and, under the drop,
	 * node locks that great-grandparent.  Deriving it from @parent's raw
	 * back-pointer (ft_get_parent_slot / ft_resolve_parent_slot on cn_meta)
	 * has the SAME staleness the hint exists to avoid, one level higher: a
	 * peer that relocates+frees the great-grandparent leaves cn's back-edge
	 * dangling -> a wild SKIP_X store into a reclaimed node.  Carry the LIVE
	 * reanchored great-grandparent (d->pppnf) and cn's coherent slot in it
	 * (d->ppnfp) so the dual re-encode and its GP-lock stay coherent.  Both
	 * NULL when @parent is not compressed / at the root; the recompact then
	 * keeps its raw derivation (build-invisible / non-SKIP_X shapes).
	 */
	struct cds_ft_inode_flag *gp;		/* d->pppnf: @parent's parent. */
	struct cds_ft_inode_flag **gp_slot;	/* d->ppnfp: @parent's slot in @gp. */
	/*
	 * FOLD (coherent rekey one-decide writer): @parent is ALREADY LOCK-held
	 * by an EARLIER step of the same op (the graft's dst-parent recompaction
	 * locked the shared spine ancestor -- in a same-trie rekey the folded graft
	 * and detach share @parent), so this recompaction must NOT re-acquire it (a
	 * second ft_dlm_lock would see it held and abort -EAGAIN) and must NOT record
	 * a second release (the holding step owns it).  It still locks the recompacted
	 * node C and republishes into @parent as an SW edge under the held lock.
	 * False for every ordinary recompaction (each acquires + releases @parent).
	 */
	bool parent_held;
	/*
	 * FOLD (coherent rekey one-decide writer, the NON-held shape): @parent is
	 * NOT held by an earlier step -- this recompaction acquires it itself --
	 * but the hinted identity must still be VALIDATED, so record the read-set
	 * guard C.parent == @parent into the acquire commit.  An ordinary hint
	 * user (the cross-trie graft) deliberately does NOT: its hint exists
	 * precisely because C's own back-pointer is LAZILY updated and may name a
	 * superseded parent copy, so guarding against it would abort valid ops.
	 * The rekey fold is the other way round -- it PARKS the republish SW (a
	 * plain store into @parent's slot), so a peer that re-homed C between the
	 * driver's descent and this acquire would have it store into a slot that
	 * no longer holds C.  Guarding closes exactly that window: the lock and
	 * the validation linearize together, and once C's parent is held no peer
	 * can re-home C (that needs @parent's LOCK).  A failure is the
	 * transient -EAGAIN the driver re-descends on.  Ignored when @parent_held
	 * is set (that arm guards unconditionally).
	 */
	bool parent_guard;
};

static
void ft_descent_init(struct ft_descent *d, struct cds_ft *ft)
{
	d->depth = 0;
	/*
	 * Resolve a transient type-7 flip proxy a peer parked on the ROOT slot
	 * (Phase 4.3: a root recompact's forward edge mid-commit) to its
	 * committed-or-old target, exactly as ft_descent_traverse_compressed
	 * does for cn->child: the descent reads d->nf as a node, so an
	 * unresolved proxy (low nibble 0xF reads as internal type 7) drives
	 * the first get_nth's type dispatch off a garbage entry -> wild jump.
	 * d->nfp still names the raw slot; only the snapshot d->nf is resolved
	 * (mirrors ft_node_get_nth).
	 */
	d->nf = ft_resolve_flip_proxy(rcu_dereference(ft->root));
	d->nfp = &ft->root;
	d->pnf = NULL;
	d->pnfp = NULL;
	d->ppnf = NULL;
	d->ppnfp = NULL;
	d->pppnf = NULL;
	d->pppnfp = NULL;
	d->skip_conflict = false;
}

/*
 * Advance descent state through a compressed node on full key match.
 * Updates parent chain, current pointer, and depth.  The caller is
 * responsible for snapshot, snapshot_n, and detach tracking before
 * calling this helper.
 */
static inline_lookup
void ft_descent_traverse_compressed(struct cds_ft *ft, struct ft_descent *d,
		struct cds_ft_compressed_node *cn,
		const uint8_t **iter_key)
{
	unsigned int rewind;

	d->pppnf  = d->ppnf;
	d->pppnfp = d->ppnfp;
	d->ppnf  = d->pnf;
	d->ppnfp = d->pnfp;
	d->pnf   = d->nf;
	d->pnfp  = d->nfp;
	d->nfp   = &cn->child;
	/*
	 * Resolve a transient type-7 flip proxy a peer parked on cn->child
	 * (Phase 4.3), then reanchor a skip-compressed child through the shared
	 * read-side primitive (ft_reanchor_flag) BEFORE it becomes the descent
	 * cursor -- the same MW convergence as ft_descent_step.  A peer split/
	 * merge below cn can leave cn->child a torn skip pointer whose one-hop
	 * recovery is internal memory (type confusion) or stale-length (mis-file);
	 * the reanchor walks cn->child's live parent chain instead.  An unresolved
	 * proxy (low nibble 0xF reads as internal type 7) would also drive the
	 * next get_nth off a garbage type -> SIGILL, hence resolve-then-reanchor.
	 * d->nfp names the raw skip slot (the publish target); on a rewind > 0
	 * (peer chain-merge moved the position shallower) that slot is at the
	 * wrong level -- flagged so a mutating caller re-descends.
	 */
	d->nf    = ft_reanchor_flag(ft, ft_resolve_flip_proxy(cn->child), &rewind);
	MRG_REANCHOR_PROBE(0, rewind);
	if (caa_unlikely(rewind != 0))
		d->skip_conflict = true;
	d->depth += cn->len;
	*iter_key += cn->len;
}

/*
 * Advance the descent cursor one level down: rotate current -> parent ->
 * grandparent, then descend into child @key_value.
 *
 * Returns the new d->nf (the child's flagged pointer, possibly NULL).
 */
static inline
struct cds_ft_inode_flag *ft_descent_step(struct cds_ft *ft, struct ft_descent *d,
		uint8_t key_value)
{
	unsigned int rewind;

	d->pppnf  = d->ppnf;
	d->pppnfp = d->ppnfp;
	d->ppnf  = d->pnf;
	d->ppnfp = d->pnfp;
	d->pnf   = d->nf;
	d->pnfp  = d->nfp;
	/*
	 * Navigate through the read side's robust reanchor primitive rather
	 * than the pre-MW unvalidated one-hop skip resolve (ft_node_get_nth):
	 * under MW a peer writer can split/merge the trie under this descent,
	 * exactly the concurrent structural change the read side already
	 * tolerates.  We additionally capture the raw publish slot (&d->nfp).
	 * A reanchor that lands shallower (rewind > 0, a concurrent
	 * chain-merge) leaves d->nfp at the wrong level -- flag it so a
	 * mutating caller re-descends (see struct ft_descent.skip_conflict).
	 */
	d->nf    = ft_node_get_nth_reanchor_slot(ft, d->pnf, &d->nfp,
			key_value, FT_PF_NONE, &rewind);
	MRG_REANCHOR_PROBE(1, rewind);
	if (caa_unlikely(rewind != 0))
		d->skip_conflict = true;
	d->depth++;
	return d->nf;
}

#ifdef FEATURE_FT_PROBE_GRAFT_SWAP
extern unsigned long cds_ft_probe_gs_pubabort;
extern unsigned long cds_ft_probe_gs_pubok;
#define FT_GS_PROBE_INC(c)	__atomic_fetch_add(&(c), 1, __ATOMIC_RELAXED)
#else
#define FT_GS_PROBE_INC(c)	do { } while (0)
#endif

/*
 * FT bridge to the concurrent MCAS transaction engine (<urcu/rcu-txn.h>).  An
 * op records its frozen edge set {slot, old, new} DIRECTLY into the engine
 * transaction (@mtxn, a `struct urcu_txn *`) during its
 * build -- through ft_flip_txn_record_reserved /
 * ft_flip_txn_record_tag and the ordered-list *_prepare helpers -- then
 * ft_flip_txn_commit commits @mtxn, so the whole set (structural index AND
 * ordered-cell list) publishes atomically (one status-word flip).  There is no
 * intermediate single-updater buffer: every slot is stored straight into the
 * concurrent engine, so no edge can be dropped between a buffer and a replay.
 *
 * Two record tag families share the ONE @mtxn (the engine carries the tag PER
 * record, urcu_txn_record.proxy_tag): STRUCTURAL trie edges carry FT's type-7 /
 * 0xF tag (FT_FLIP_PROXY_TAG), resolved on the read hot path by
 * ft_resolve_flip_proxy; ORDERED-CELL list edges carry the concurrent list's
 * engine tag (URCU_TXN_TAG, bit 0), resolved by urcu_txn_list_resolve.  Commit
 * OWNS reclaim: the committed descriptor is deferred-freed through the FT's RCU
 * flavor (a reader may hold a parked record) -- except on the exclusive build,
 * which frees it in place.  An op that aborts before committing drops its
 * uncommitted handle with ft_flip_txn_destroy (nothing was installed --
 * freeze-before-install).
 *
 * @reserved marks a bounded txn whose @mtxn was pre-reserved to its edge count
 * at create, so every later store appends without allocating and the commit is
 * infallible (the load-bearing "commit cannot fail" contract).  An unbounded
 * glue txn is reserved to its bounded cluster size before it records (see
 * ft_flip_txn_reserve); should a store still fail to grow, the OOM is sticky and
 * the commit returns a clean MEMORY_ERROR with nothing parked.
 */
/*
 * Upper bound of FT_STATE_LOCK locks one commit can hold: the chain-compress
 * fused merge fences the collapsed chain (boundary + old parent cn + old child
 * cn = 3); a LOCK_FINE recompact holds its whole {C, P} lock-set, plus {GP} when
 * P is a compressed node whose SKIP_X dual it re-encodes (§9.3) = 3.
 */
#define FT_FLIP_TXN_MAX_LOCKS	8

struct ft_flip_txn {
	struct urcu_txn *mtxn;	/* the concurrent commit engine handle:
					 * &own (standalone txn), or the op's
					 * PERSISTENT handle (create_bounded_on)
					 * whose retry aging / FIFO turn / learned
					 * size span the op's restart_attempt loop
					 * (doc/design/mcas-multiwriter-readiness.md
					 * §11) */
	struct urcu_txn own;	/* backing handle for standalone txns */
	bool reserved;			/* @mtxn pre-reserved (bounded) => infallible commit */
	/*
	 * FT_STATE_LOCK registry (MW F2, CORE_682870 fix plan): the
	 * nodes this commit's op MARKED with the reversible per-node lock.  On
	 * commit OK the lock is consumed by whichever state transition the op
	 * RECORDED on the node -- {LOCK|s -> TOMBSTONE|s} (retire) or
	 * {LOCK|s -> s} (release, the node survives) -- so the registry does
	 * not care which terminal was chosen.  On EVERY other terminal outcome
	 * of the wrapper -- commit ABORT / MEMORY_ERROR (ft_flip_txn_commit) or
	 * a pre-commit bail (ft_flip_txn_destroy) -- the lock must be CLEARED or
	 * every later peer publish into the node aborts forever.  The two
	 * terminal paths drain this registry so no caller unwind can leak a
	 * lock.
	 */
	struct cds_ft_metadata *locks[FT_FLIP_TXN_MAX_LOCKS];
	unsigned int nr_locks;
	/*
	 * Set when a per-node lock acquire MISSED (see
	 * ft_flip_txn_lock_or_guard_parent).  The op then structurally writes a
	 * slot whose owner it does not hold, so the commit must ABORT rather
	 * than publish: an all-or-none lock-set, with the miss re-descending.
	 */
	bool acquire_miss;
	/*
	 * MIXED sw/mw commit (DLM lock_fine): when true, the STRUCTURAL record
	 * helpers (every ft_flip_txn_record_tag edge) plant SW-kind records -- a
	 * plain locked park that CANNOT fail -- because the op holds the DLM
	 * node lock over each of those slots, so no peer mutates them.  The
	 * genuinely-unlocked edges (the ordered-cell interleave, the duplicate-
	 * chain splices, the rank-count propagation up unlocked ancestors) opt
	 * back out to MW via ft_flip_txn_record_tag_mw.  The mixed commit then
	 * installs the MW edges FIRST (they can conflict -> a clean abort with
	 * ZERO structural parks) and parks the SW structure last, just before the
	 * flip.  Default false keeps every non-opted-in op all-MW == byte-identical.
	 */
	bool structural_sw;
};

/*
 * The engine handle behind @t -- pass to the urcu_txn_* / *_prepare primitives
 * that record directly into the transaction.
 */
static inline
struct urcu_txn *ft_flip_txn_handle(struct ft_flip_txn *t)
{
	return t->mtxn;
}

/*
 * Initialize an op's PERSISTENT engine handle (doc §11): one handle spans the
 * op's whole restart_attempt retry loop, so contention aging (txn->retry, the
 * FIFO fair-mutex turn) and the learned descriptor size survive across
 * attempts instead of resetting with each per-attempt ft_flip_txn.  Bind the
 * trie's escalation domain and the group's RCU flavor, so urcu_txn_begin() /
 * urcu_txn_end() bracket each attempt in the FT's RUNTIME flavor -- the FT
 * owns the read-side bracket; a caller-held section merely nests.
 *
 * EXCLUSIVE trie: no concurrent writer (NULL domain -- never escalates) and
 * no concurrent reader (NULL flavor -- the bracket falls back to the
 * URCU_TXN_RCU_READ_LOCK macro, which fractal-trie-internal.h no-ops), so
 * begin()/end() only manage the per-attempt descriptor / deferred-cleanup
 * state: behavior-identical to the pre-bracket exclusive path.
 */
static inline
void ft_txn_op_init(struct cds_ft *ft, struct urcu_txn *op)
{
	if (ft->exclusive)
		urcu_txn_init_flavor(op, NULL, NULL);
	else
		urcu_txn_init_flavor(op, &ft->txn_domain, ft->group->flavor);
}

static inline
struct ft_flip_txn *ft_flip_txn_create(void)
{
	struct ft_flip_txn *t = (struct ft_flip_txn *) malloc(sizeof(*t));

	if (!t)
		return NULL;
	t->mtxn = &t->own;
	urcu_txn_init(t->mtxn, NULL);	/* flavor-agnostic: the caller brackets the
					 * RCU read side; no escalation domain
					 * under POC exclusion */
	/* Skip the age-0 optimistic install and its fixed-size RYW Bloom filter:
	 * a growable FT commit (merge/graft spine folds) can accumulate a large,
	 * dense write set that saturates the Bloom, false-positiving a same-slot
	 * coincidence into a spurious escalate-ABORT (see the note in
	 * ft_flip_txn_create_bounded).  The exact age-1+ reconcile has no Bloom. */
	urcu_txn_expect_conflict(t->mtxn);
	t->reserved = false;		/* unbounded: @mtxn grows as edges record */
	t->nr_locks = 0;
	t->acquire_miss = false;
	t->structural_sw = false;	/* all-MW until a caller opts in under lock_fine */
	return t;
}

/*
 * Bounded FT flip-txn: a pre-reserved transaction for the point-op commits
 * (insert one-commit, ordered-cell splice/unsplice/swap) and bulk-op glue folds
 * whose edge count is bounded by construction.  BOTH the edge buffer and the
 * MCAS commit descriptor are reserved up front, so every later record appends
 * without reallocating and the replay-commit is infallible.  Returns NULL on OOM
 * -> the caller degrades to a direct / sequential publish.
 */
#ifdef FEATURE_FT_FAULT_INJECT
extern long cds_ft_fault_flip_countdown;
extern long cds_ft_fault_replace_countdown;
#endif

/*
 * Arm the next commit of @t to ABORT, for the replace family's otherwise
 * unexecutable abort arms (see cds_ft_fault_replace_countdown).  Sets the
 * engine's own @acquire_miss so ft_flip_txn_commit takes its real
 * discard-unpublished path -- the caller's unwind under test is then the one
 * that would run against a live peer.  No-op unless armed, and compiled out
 * entirely without FEATURE_FT_FAULT_INJECT.
 */
static inline
void ft_replace_fault_arm_abort(struct ft_flip_txn *t)
{
#ifdef FEATURE_FT_FAULT_INJECT
	if (cds_ft_fault_replace_countdown >= 0) {
		if (cds_ft_fault_replace_countdown == 0) {
			cds_ft_fault_replace_countdown = -1;
			t->acquire_miss = true;
		} else {
			cds_ft_fault_replace_countdown--;
		}
	}
#else
	(void) t;
#endif
}

static inline
struct ft_flip_txn *ft_flip_txn_create_bounded(unsigned int cap)
{
	struct ft_flip_txn *t;

#ifdef FEATURE_FT_FAULT_INJECT
	/*
	 * Test-only flip-txn allocation fault injection (see
	 * cds_ft_fault_flip_countdown).  Drives the grow-and-abort / pre-reserve
	 * commit paths: ft_ord_cell_flip_try returns -ENOMEM (abort), and a
	 * pre-reservation (ft_chain_compress_fused / ft_detach_node) fails before
	 * its first side-effect.
	 */
	if (cds_ft_fault_flip_countdown >= 0) {
		if (cds_ft_fault_flip_countdown == 0) {
			cds_ft_fault_flip_countdown = -1;
			return NULL;
		}
		cds_ft_fault_flip_countdown--;
	}
#endif
	t = (struct ft_flip_txn *) malloc(sizeof(*t));
	if (!t)
		return NULL;
	t->mtxn = &t->own;
	urcu_txn_init(t->mtxn, NULL);	/* no escalation domain under POC exclusion */
	/*
	 * Skip the age-0 optimistic install.  Age 0 detects a same-slot read-
	 * your-own-writes coincidence with a fixed-size Bloom filter and, on a
	 * hit, sets esc_pending to force an ABORT-and-escalate to the exact
	 * age-1+ reconcile.  A pre-reserved FT commit is a DENSE bulk write set
	 * (a whole-chain detach reserves 100+ edges) that SATURATES that Bloom,
	 * so a non-coinciding slot false-positives and the commit aborts -- and
	 * these commits have no retry loop (remove_all et al.), surfacing the
	 * abort as a spurious MEMORY_ERROR.  expect_conflict runs the sorted,
	 * exact-reconcile install from attempt 0, which has no Bloom and no
	 * false positive; it is also the right lane for a contended MW commit.
	 */
	urcu_txn_expect_conflict(t->mtxn);
	if (urcu_txn_reserve(t->mtxn, cap) < 0) {
		if (t->mtxn->desc && t->mtxn->desc != URCU_TXN_ENOMEM)
			urcu_txn_destroy(t->mtxn->desc);
		free(t);
		return NULL;
	}
	t->reserved = true;
	t->nr_locks = 0;
	t->acquire_miss = false;
	t->structural_sw = false;	/* all-MW until a caller opts in under lock_fine */
	return t;
}

/*
 * UNBOUNDED FT flip-txn bound to an op's persistent engine handle -- the
 * growable sibling of ft_flip_txn_create_bounded_on, for a fold whose edge
 * count is not known up front.
 *
 * WHY IT HAS TO EXIST: ft_flip_txn_create() inits its own handle with NO
 * escalation domain ("no escalation domain under POC exclusion"), so an op
 * built on it can NEVER escalate however many times it retries -- every
 * attempt is a fresh handle, retry aging resets to zero, and
 * urcu_txn__self_qualifies is never reached.  A contended writer then
 * livelocks by construction rather than taking its FIFO turn.  Binding to
 * @op is what makes the retry loop terminate.
 *
 * Keeps create()'s expect_conflict: a dense fold write set saturates the
 * age-0 RYW Bloom and would false-positive a same-slot coincidence.
 */
static inline
struct ft_flip_txn *ft_flip_txn_create_on(struct urcu_txn *op)
{
	struct ft_flip_txn *t = (struct ft_flip_txn *) malloc(sizeof(*t));

	if (!t)
		return NULL;
	t->mtxn = op;
	urcu_txn_expect_conflict(t->mtxn);
	t->reserved = false;		/* unbounded: @mtxn grows as edges record */
	t->nr_locks = 0;
	t->acquire_miss = false;
	t->structural_sw = false;
	return t;
}

/*
 * Bounded FT flip-txn BOUND to an op's persistent engine handle (@op,
 * ft_txn_op_init'd before the op's restart_attempt loop and bracketed by
 * urcu_txn_begin()/urcu_txn_end() per attempt): the recording facade is this
 * per-attempt wrapper, but the retry aging, FIFO escalation turn, and learned
 * descriptor size live in @op and survive the wrapper (commit/destroy free
 * only the wrapper).  The reservation sizes @op's descriptor for this attempt
 * exactly as create_bounded does for a standalone txn.  Returns NULL on OOM
 * (malloc, or a failed reserve -- the sticky URCU_TXN_ENOMEM marker stays in
 * @op and the op's terminal urcu_txn_end() clears it) -> the caller bails
 * with -ENOMEM.  Shares the fault-injection countdown with create_bounded.
 */
static inline
struct ft_flip_txn *ft_flip_txn_create_bounded_on(struct urcu_txn *op,
		unsigned int cap)
{
	struct ft_flip_txn *t;

#ifdef FEATURE_FT_FAULT_INJECT
	if (cds_ft_fault_flip_countdown >= 0) {
		if (cds_ft_fault_flip_countdown == 0) {
			cds_ft_fault_flip_countdown = -1;
			return NULL;
		}
		cds_ft_fault_flip_countdown--;
	}
#endif
	t = (struct ft_flip_txn *) malloc(sizeof(*t));
	if (!t)
		return NULL;
	t->mtxn = op;
	if (urcu_txn_reserve(op, cap) < 0) {
		free(t);
		return NULL;
	}
	t->reserved = true;
	t->nr_locks = 0;
	t->acquire_miss = false;
	t->structural_sw = false;	/* all-MW until a caller opts in under lock_fine */
	return t;
}

/*
 * Reserve @cap records on an already-created (unbounded) flip-txn -- the glue
 * path pre-sizes @mtxn to its bounded cluster count before it records, so its
 * later stores append without allocating and its commit is infallible.  Returns
 * true on success, false on OOM (the caller aborts before any side-effect).
 */
static inline
bool ft_flip_txn_reserve(struct ft_flip_txn *t, unsigned int cap)
{
	if (urcu_txn_reserve(t->mtxn, cap) < 0)
		return false;
	t->reserved = true;
	return true;
}

/*
 * Widen @t's reservation by @extra records beyond its current capacity.  A
 * single-commit op (the insert recompact reparent sweep) that discovers extra
 * edges mid-build grows its txn here.  Unlike a two-commit graft/merge SECOND
 * commit -- which must pre-reserve so its post-detach publish cannot fail -- a
 * single-commit op's commit may still fail cleanly (freeze-before-install leaves
 * the structure byte-for-byte untouched), so an OOM here just aborts/retries the
 * op.  urcu_txn_reserve is grow-safe after records are recorded (records move to
 * the grown descriptor; proxies form from the final address only at install).
 * Returns false on OOM (nothing new recorded -> caller aborts).
 */
static inline
bool ft_flip_txn_reserve_extra(struct ft_flip_txn *t, unsigned int extra)
{
	unsigned int cur = (t->mtxn->desc && t->mtxn->desc != URCU_TXN_ENOMEM) ?
			t->mtxn->desc->cap : 0;

	return ft_flip_txn_reserve(t, cur + extra);
}

/*
 * Take a caller-reserved flip-txn when @pre supplies one, NULLing the caller's
 * slot to transfer ownership: from here on the consuming bulk op commits and
 * reclaims it, and the caller frees only what it still holds.  Returns NULL when
 * no txn was reserved (the standalone-op path) -- the consumer then creates and
 * reserves its own.  A same-trie rekey pre-reserves the txn before its detach so
 * the post-drain commit cannot fail, while a normal op reserves its own where
 * failure is still clean.
 */
static inline
struct ft_flip_txn *ft_flip_txn_take(struct ft_flip_txn **pre)
{
	if (pre && *pre) {
		struct ft_flip_txn *t = *pre;

		*pre = NULL;
		return t;
	}
	return NULL;
}

/*
 * Reclaim deferral passed to urcu_txn_commit_flavor on the exclusive build:
 * with no concurrent reader, a committed txn's parked group block is freed in
 * place, no grace period.  Matches the flavor call_rcu signature.
 */
static void ft_flip_txn_call_rcu_now(struct rcu_head *head,
		void (*func)(struct rcu_head *))
{
	func(head);
}

/*
 * FT_STATE_LOCK, ACQUIRE side (MW F2, Option A --
 * fractal-trie-internal.h at the bit's definition, CORE_682870 fix plan).  A
 * body copier (recompact retire / chain-compress collapse) CASes
 * {clean -> |LOCK} on the node it is about to read, BEFORE the first body
 * read: from that point every peer publish into the node fails its §4.B
 * clean-LIVE guard at commit, so the copied body cannot go stale between the
 * copy and the copier's own commit without SOMEONE aborting -- the copier's
 * state record {LOCK|s -> TOMBSTONE|s} pins the whole word from mark to
 * commit (a peer state change under the fence, e.g. a write record that
 * captured its expected old after the mark, mismatches the copier's expected
 * old instead: exactly one side survives).  A dirty word at the mark -- a
 * parked proxy, a tombstone (real retire), or LOCK (a peer holds
 * it) -- fails the acquire: -EAGAIN, the op re-descends after the peer
 * settles.  On success *@state_snapshot returns the CLEAN pre-mark word: the
 * one consistent snapshot the whole copy plan (sizing, tombstone expected-old)
 * must derive from.
 *
 * The Dekker pairing this mark forms with a peer's §4.B guard is closed by a
 * LOAD-BEARING engine contract: pure validates PARK a proxy like writes (see
 * the contract note at urcu_txn_validate, <urcu/rcu-txn.h>).  A guard planted
 * before the mark occupies the word, so the mark's dirty check sees it and
 * bails; a guard planted after expects the CLEAN image and mismatches the
 * fenced word; and a peer payload parked between the mark and the copy read
 * is caught by the copy loops' latch bail.  The once-planned engine two-phase
 * install ("F2 3/3") was DROPPED as unnecessary under that contract
 * (2026-07-06 decision, CORE_682870_FORENSICS.md).
 */
static inline
int ft_meta_lock_acquire(struct cds_ft_metadata *meta,
		uintptr_t *state_snapshot)
{
	uintptr_t s = CMM_LOAD_SHARED(meta->state);

	if (caa_unlikely(s & (FT_STATE_PROXY | FT_STATE_TOMBSTONE |
			FT_STATE_LOCK)))
		return -EAGAIN;
	if (caa_unlikely(uatomic_cmpxchg(&meta->state, s,
			s | FT_STATE_LOCK) != s))
		return -EAGAIN;
	*state_snapshot = s;
	return 0;
}

/*
 * FT_STATE_LOCK, RELEASE side: drop ONLY the reversible lock bit,
 * preserving every other field -- the copy was abandoned (pre-commit bail) or
 * its commit aborted (the engine settled the state record back to its old
 * value, LOCK included), and the node stays LIVE.  The word may transiently
 * hold a peer's parked proxy (a doomed guard mid-install -- it validates
 * clean-LIVE and the fence is still set -- or a peer write that will mismatch
 * the fence-pinned value): wait for the owner to settle, then CAS.  The wait
 * is bounded by the owner's settle, which is owner-only (helping a foreign
 * txn drives it terminal but cannot make its word plain), so a helping read
 * would not shorten it -- part of why the engine-side "F2 3/3" was dropped.
 * A writer thread dying mid-install would wedge this loop, but a writer dying
 * mid-mutation is already fatal to the trie under the library's contract.
 * The CAS loop (not a blind AND) is what keeps a parked proxy POINTER from
 * being corrupted by a bit-clear.
 */
static inline
void ft_meta_lock_release(struct cds_ft_metadata *meta)
{
	for (;;) {
		uintptr_t s = CMM_LOAD_SHARED(meta->state);

		if (caa_unlikely(s & FT_STATE_PROXY)) {
			caa_cpu_relax();
			continue;
		}
		/*
		 * Only the fence owner clears LOCK, and peers preserve
		 * foreign state bits, so the bit is still set here (NDEBUG
		 * builds degrade to a harmless same-value CAS if it is not).
		 */
		assert(s & FT_STATE_LOCK);
		if (caa_likely(uatomic_cmpxchg(&meta->state, s,
				s & ~FT_STATE_LOCK) == s))
			return;
	}
}

/*
 * CLEAR-IF-HELD: drop the reversible node lock ONLY if the word still holds
 * it, otherwise return silently.  This is the cleanup twin used when the op does
 * NOT know at the cleanup point whether its own commit already consumed the
 * fence (a fenced {LOCK|s -> TOMBSTONE|s} tombstone that committed leaves the
 * word TOMBSTONE, LOCK dropped) or whether it must still be released (any
 * abort / pre-commit bail leaves the word {LOCK|s}).  Because the fence is
 * owner-exclusive -- only WE set it (CAS clean->LOCK), and no peer clears or
 * re-sets it while we hold it -- the settled word is deterministically either
 * {LOCK|s} (release it) or {...|TOMBSTONE} without LOCK (our commit took
 * it; leave it).  A doomed peer guard may transiently park an FT_STATE_PROXY on
 * the word (Dekker note at ft_meta_lock_acquire); wait it out as the plain clear
 * does.  This lets a caller that owns MORE marks than FT_FLIP_TXN_MAX_LOCKS
 * (the orphan chain, up to FT_MAX_DEPTH) clear them from its own array with ONE
 * unconditional post-op sweep instead of registering them or tracking per-commit
 * which ones a success consumed.
 */
static inline
void ft_meta_lock_release_if_held(struct cds_ft_metadata *meta)
{
	for (;;) {
		uintptr_t s = CMM_LOAD_SHARED(meta->state);

		if (caa_unlikely(s & FT_STATE_PROXY)) {
			caa_cpu_relax();
			continue;
		}
		if (!(s & FT_STATE_LOCK))
			return;	/* our commit already consumed it (TOMBSTONE) */
		if (caa_likely(uatomic_cmpxchg(&meta->state, s,
				s & ~FT_STATE_LOCK) == s))
			return;
	}
}

/*
 * Register a marked fence with the commit wrapper that owns its outcome: the
 * two terminal paths (ft_flip_txn_commit on ABORT / MEMORY_ERROR,
 * ft_flip_txn_destroy on a pre-commit bail) clear every registered fence, and
 * a commit OK consumes it through the recorded {LOCK|s -> TOMBSTONE|s}
 * transition instead.  Register only once the mark's holder can no longer
 * clear it itself (i.e. when the op hands the outcome to the txn).
 */
static inline
void ft_flip_txn_lock_register(struct ft_flip_txn *t,
		struct cds_ft_metadata *meta)
{
	assert(t->nr_locks < FT_FLIP_TXN_MAX_LOCKS);
	t->locks[t->nr_locks++] = meta;
}

/*
 * Acquire the per-node lock of a RELEASE-terminal lock-set member -- a node the
 * op must exclude peers from but does NOT retire (recompact's {P} / {GP}, §9.3).
 * Same acquire as the retire half (ft_meta_lock_acquire: -EAGAIN on a dirty word
 * = "could not acquire, re-descend"); the halves diverge only at the commit,
 * where this one records {LOCK|s -> s} instead of the tombstone.
 *
 * The clean snapshot is stashed alongside the member so the commit can plant
 * that record and a bail can drop the lock again.  Members are held in a small
 * fixed array (the lock-set is bounded and known up front, §5: no growing a
 * lock-set in place), so an acquire failure just unwinds the ones already held.
 */
#ifdef FEATURE_FT_FAULT_INJECT
extern long cds_ft_fault_lock_countdown;
#endif
static inline
int ft_lock_member(struct cds_ft_metadata *meta,
		struct cds_ft_metadata **set, uintptr_t *snap, unsigned int *n)
{
	int ret;

#ifdef FEATURE_FT_FAULT_INJECT
	/*
	 * Test-only: fail this acquire exactly as a peer holding the lock would
	 * (see cds_ft_fault_lock_countdown).  Drives the caller's unwind --
	 * unlock the members already held, discard the build-invisible copy,
	 * re-descend -- which the FT-wide lock otherwise makes unreachable.
	 */
	if (cds_ft_fault_lock_countdown >= 0) {
		if (cds_ft_fault_lock_countdown == 0) {
			cds_ft_fault_lock_countdown = -1;
			return -EAGAIN;
		}
		cds_ft_fault_lock_countdown--;
	}
#endif
	ret = ft_meta_lock_acquire(meta, &snap[*n]);
	if (ret)
		return ret;
	set[(*n)++] = meta;
	return 0;
}

/*
 * Drop every RELEASE-terminal lock the op holds, leaving the nodes LIVE: the
 * bail path of the above, for a member set not yet handed to a txn.  Once the
 * members ARE registered (ft_flip_txn_lock_register, on the success path),
 * the txn's registry owns the unlock instead and this must not run.
 */
static inline
void ft_unlock_members(struct cds_ft_metadata **set, unsigned int n)
{
	unsigned int i;

	for (i = 0; i < n; i++)
		ft_meta_lock_release(set[i]);
}

static inline
void ft_flip_txn_lock_release_all(struct ft_flip_txn *t)
{
	unsigned int i;

	for (i = 0; i < t->nr_locks; i++)
		ft_meta_lock_release(t->locks[i]);
	t->nr_locks = 0;
}

/*
 * Drop an FT flip-txn that was NOT committed (an op aborted before publishing --
 * an OOM or a no-op path).  Nothing was stored into any slot
 * (freeze-before-install), so this frees any live (uncommitted) MCAS descriptor
 * and the handle; no grace period is owed.  Registered LOCK fences are
 * cleared (the marked nodes stay live; nothing retires them).
 */
static inline
void ft_flip_txn_destroy(struct ft_flip_txn *t)
{
	ft_flip_txn_lock_release_all(t);
	if (t->mtxn->desc && t->mtxn->desc != URCU_TXN_ENOMEM) {
		urcu_txn_destroy(t->mtxn->desc);
		/*
		 * Leave a BOUND persistent handle clean for the op's next
		 * attempt / its urcu_txn_end() (which would otherwise
		 * double-destroy).  A sticky URCU_TXN_ENOMEM marker is
		 * preserved above (only a live descriptor is destroyed), so
		 * an OOM already recorded on the handle still surfaces.
		 * Harmless for a standalone txn (@own dies with the wrapper).
		 */
		t->mtxn->desc = NULL;
	}
	free(t);
}

/*
 * Commit an FT flip-txn (ft_flip_txn_create*): commit @mtxn -- whose edge set was
 * recorded straight into it as the op built -- then free the handle.  The commit
 * publishes the whole recorded edge set atomically (one status-word flip) and
 * OWNS the descriptor reclaim, deferring it through the FT's RCU flavor (or
 * freeing it in place on the exclusive build).  A bounded (or glue-reserved) txn
 * pre-sized @mtxn, so its commit is infallible (returns OK); an un-reserved store
 * that could not grow left @mtxn sticky-ENOMEM, so the commit returns
 * MEMORY_ERROR with nothing parked (freeze-before-install -- the structure is
 * byte-for-byte untouched).  ABORT (a peer writer froze a guarded node, or won a
 * forward-slot expected-value CAS, between this op's descent and its commit) is
 * SURFACED to the caller, not retried in place: the commit consumes the
 * descriptor (urcu_txn_commit reclaims it even on abort), so re-driving the same
 * @mtxn would short-circuit to a spurious OK with nothing published.  A permanent
 * structural conflict is resolved by the FT op re-reading the tree and rebuilding
 * a fresh txn (re-descend), per doc/design/step4-concurrent-engine-plan.md; the
 * ABORT return is what tells it to.  Under the retained single-writer exclusion
 * ABORT never occurs, so this is behaviour-identical there.  @t is consumed.
 */
static inline
enum urcu_txn_status ft_flip_txn_commit(struct cds_ft *ft,
		struct ft_flip_txn *t)
{
	void (*reclaim)(struct rcu_head *, void (*)(struct rcu_head *)) =
		ft->exclusive ? ft_flip_txn_call_rcu_now :
				ft->group->flavor->update_call_rcu;
	enum urcu_txn_status st;

	if (caa_unlikely(t->acquire_miss)) {
		/*
		 * A lock-set member was not acquired, so this attempt writes a
		 * slot it does not own: discard it unpublished and report ABORT,
		 * which every caller already routes to a re-descend.  Age the
		 * handle first, exactly as a real contention abort does inside
		 * urcu_txn_commit_flavor -- without it the op never advances
		 * txn->retry, never escalates to the FIFO lane, and a contended
		 * node could starve it indefinitely.
		 */
		urcu_txn_conflict(t->mtxn);
		FT_TP(txn_commit, (const void *) t->mtxn,
			(int) URCU_TXN_STATUS_ABORT);
		ft_flip_txn_destroy(t);
		return URCU_TXN_STATUS_ABORT;
	}
	st = urcu_txn_commit_flavor(t->mtxn, reclaim);
	FT_TP(txn_commit, (const void *) t->mtxn, (int) st);
	/*
	 * node locks: a committed txn transitioned each registered node
	 * through the terminal its op recorded -- {LOCK|s -> TOMBSTONE|s}
	 * (retire) or {LOCK|s -> s} (release) -- so the lock is already
	 * consumed and the registry is not drained.  ABORT settled the state
	 * record back to its old value -- lock still set -- and MEMORY_ERROR
	 * parked nothing, so both must clear the reversible bit or the still-live
	 * nodes would fail every later peer guard forever.
	 */
	if (caa_unlikely(st != URCU_TXN_STATUS_OK))
		ft_flip_txn_lock_release_all(t);
	free(t);
	return st;
}

/*
 * Record one structural edge (FT's type-7 / 0xF proxy tag) directly into the
 * txn.  Used by the GLUE flip-txn fold and the single-commit ops, whose txn was
 * pre-reserved to its bounded edge count (create_bounded / ft_flip_txn_reserve),
 * so the store appends without reallocating and cannot fail.  A store on an
 * un-reserved txn that fails to grow is sticky-ENOMEM and surfaces at commit as
 * MEMORY_ERROR (see the glue path); the assert only guards the reserved use.
 */
static inline
void ft_flip_txn_record_tag(struct ft_flip_txn *t, void **slot,
		void *old_ptr, void *new_ptr, uintptr_t tag)
{
	int ret;

	FT_TP(edge_record, (const void *) t->mtxn, (const void *) slot,
		(const void *) old_ptr, (const void *) new_ptr, tag);
	/*
	 * MIXED sw/mw: a STRUCTURAL edge parks SW when the op holds the DLM lock
	 * over @slot (structural_sw set by the caller under lock_fine) -- a plain
	 * locked park, installed after the MW edges, that cannot fail.  Otherwise
	 * (every other op, non-lock_fine) it is MW == the all-MW behaviour.
	 */
	if (t->structural_sw)
		ret = urcu_txn_store_sw(t->mtxn, slot, old_ptr, new_ptr, tag);
	else
		ret = urcu_txn_store_mw(t->mtxn, slot, old_ptr, new_ptr, tag);
	assert(!ret);
	(void) ret;	/* reserved up front -> never fails */
}

/*
 * Record an edge that is ALWAYS MW-kind, regardless of the txn's structural_sw
 * mode: the genuinely-unlocked slots -- the ordered-cell interleave edges, the
 * duplicate-chain splices, the rank-count propagation up unlocked ancestors.
 * Under the mixed commit these install FIRST and may conflict (a clean abort);
 * the SW structure parks only after they all succeed.  Identical to
 * ft_flip_txn_record_tag whenever structural_sw is false, so switching a
 * non-lock_fine site to it is byte-neutral.
 */
static inline
void ft_flip_txn_record_tag_mw(struct ft_flip_txn *t, void **slot,
		void *old_ptr, void *new_ptr, uintptr_t tag)
{
	int ret;

	FT_TP(edge_record, (const void *) t->mtxn, (const void *) slot,
		(const void *) old_ptr, (const void *) new_ptr, tag);
	ret = urcu_txn_store_mw(t->mtxn, slot, old_ptr, new_ptr, tag);
	assert(!ret);
	(void) ret;	/* reserved up front -> never fails */
}

/*
 * MIXED sw/mw: opt @t's structural edges into SW-kind parks.  A caller holding
 * the DLM node locks over the slots it structurally rewrites calls this right
 * after creating its commit txn, so the forward publish, the re-parents, the
 * lock releases and the retires all park SW (locked, cannot fail) while the cell
 * / count edges stay MW.  A no-op (false) leaves the txn all-MW.  Must be set
 * BEFORE the first structural record.
 */
static inline
void ft_flip_txn_set_structural_sw(struct ft_flip_txn *t, bool v)
{
	t->structural_sw = v;
}

static inline
void ft_flip_txn_record_reserved(struct ft_flip_txn *t, void **slot,
		void *old_ptr, void *new_ptr)
{
	ft_flip_txn_record_tag(t, slot, old_ptr, new_ptr, FT_FLIP_PROXY_TAG);
}

/*
 * MW LOCK_FINE DLM (Step 1, see
 * doc/design/mw-writer-lock-escalation-model.md): the composable
 * one-commit lock-set acquire.  An op derives its lock-set + read-set by a
 * read-only plan (following back-edges), records both onto a DEDICATED acquire
 * flip-txn -- NOT the content lane, since acquiring on the content txn
 * circular-waits on the domain (escalation model §5) -- and commits it as ONE
 * all-or-none MCAS.  Compose:
 *
 *   struct ft_flip_txn *acq = ft_flip_txn_create_bounded(nr_lock + nr_guard);
 *   if (ft_dlm_lock(acq, C_meta, &snapC)) { ft_flip_txn_destroy(acq); goto replan; }
 *   ft_dlm_guard_parent(acq, C_meta, pf_P);   // read-set: C.parent still == P
 *   if (ft_dlm_lock(acq, P_meta, &snapP)) { ... }
 *   if (ft_flip_txn_commit(ft, acq) != URCU_TXN_STATUS_OK) goto replan;
 *
 * On commit OK every locked node holds LOCK and every guarded back-edge was
 * validated at the linearization point; the caller then registers each node in
 * its CONTENT txn's locks[] and records the terminal (release
 * {LOCK|snap -> snap} / retire {-> TOMBSTONE|snap}) from the captured @snap,
 * exactly as the incremental scheme does today.  Deadlock-free: a dirty or held
 * member, or a re-homed guarded back-edge, aborts the commit -- it never blocks,
 * and NOTHING is acquired on failure (all-or-none -> abort-and-regrow).
 */

/*
 * Record {clean -> LOCK} for @meta onto the acquire txn @t, capturing the
 * clean word in @snap.  -EAGAIN if @meta is already PROXY|TOMBSTONE|LOCK
 * (dirty): the caller destroys @t and re-plans.  The edge must be reserved
 * (create_bounded); the actual set is atomic at the commit, not here.
 */
static inline
int ft_dlm_lock(struct ft_flip_txn *t, struct cds_ft_metadata *meta,
		uintptr_t *snap)
{
	uintptr_t s = CMM_LOAD_SHARED(meta->state);

	/*
	 * The acquire MUST be a validated CAS: ft_flip_txn_record_tag dispatches
	 * SW-vs-MW on @t->structural_sw alone, so a structural_sw acquire txn
	 * would degrade the {clean -> LOCK} edge to a plain store -- a lock
	 * that cannot fail against a peer, i.e. two owners.  Every caller today
	 * builds a FRESH acquire txn for the lock-set (never the content lane,
	 * see the composition sketch above), and the fold's structural_sw lives
	 * only on content txns; assert it rather than rely on that reading.
	 */
	assert(!t->structural_sw);
	if (caa_unlikely(s & (FT_STATE_PROXY | FT_STATE_TOMBSTONE |
			FT_STATE_LOCK)))
		return -EAGAIN;
	*snap = s;
	ft_flip_txn_record_tag(t, (void **) &meta->state,
			(void *) s, (void *) (s | FT_STATE_LOCK),
			FT_STATE_PROXY);
	return 0;
}

/*
 * Guard a back-edge into the SAME acquire commit: the commit aborts unless
 * @child->parent still holds @expected_pf (the tagged parent flag the plan
 * resolved).  This is the read-set validation -- a peer re-homing @child between
 * the plan's racy read of its parent and the acquire's linearization point (or a
 * flip-proxy parked mid-re-home) fails the whole-word value-CAS and aborts the
 * acquire, so the op re-plans against the settled tree.
 */
static inline
void ft_dlm_guard_parent(struct ft_flip_txn *t, struct cds_ft_metadata *child,
		struct cds_ft_inode_flag *expected_pf)
{
	urcu_txn_validate(t->mtxn, (void **) &child->parent_word,
			(void *) expected_pf, FT_FLIP_PROXY_TAG);
}

/*
 * One member of a DLM lock-set: the node @meta to acquire (LOCK), plus an
 * OPTIONAL read-set guard that @guard_child's back-edge still resolves to
 * @guard_pf (validating the racy plan read of @meta's position).  @snap
 * receives the clean word captured at the acquire, for the caller's
 * retire/release terminal.  @meta == NULL skips the member (an absent optional
 * lock-set node -- e.g. a root with no parent, a compressed-parent that is not
 * present in a given shape); the caller then treats @snap as unused.
 */
struct ft_dlm_member {
	struct cds_ft_metadata *meta;
	struct cds_ft_metadata *guard_child;
	struct cds_ft_inode_flag *guard_pf;
	uintptr_t snap;
};

/*
 * Acquire a whole lock-set in ONE all-or-none MCAS on a DEDICATED acquire
 * flip-txn (never the content lane -- the escalation model's circular-wait
 * constraint): for each present member, record its read-set guard (if any) and
 * its {clean -> LOCK} lock onto the acquire txn, then commit it once.  On
 * commit OK every present member holds LOCK (member.snap = its clean word)
 * and every guard validated at the linearization point; the caller registers
 * each member in its CONTENT txn and records the release/retire terminal from
 * member.snap, exactly as ft_insert_dlm_acquire_split and the recompact hoist
 * do.  Returns 0 (whole set acquired), -EAGAIN (a member is held/dirty or a
 * guarded back-edge re-homed -- NOTHING acquired, abort-and-regrow), or -ENOMEM.
 * Deadlock-free: a conflict aborts the commit, never blocks.
 */
static inline
int ft_dlm_acquire_set(const struct cds_ft *ft, struct ft_dlm_member *set,
		int nr)
{
	struct ft_flip_txn *acq;
	int i, nr_present = 0;

	for (i = 0; i < nr; i++)
		if (set[i].meta)
			nr_present++;
	if (!nr_present)
		return 0;
	/* Up to one guard + one lock record per present member. */
	acq = ft_flip_txn_create_bounded(2 * nr_present);
	if (!acq)
		return -ENOMEM;
	for (i = 0; i < nr; i++) {
		if (!set[i].meta)
			continue;
		if (set[i].guard_child)
			ft_dlm_guard_parent(acq, set[i].guard_child,
				set[i].guard_pf);
		if (ft_dlm_lock(acq, set[i].meta, &set[i].snap)) {
			ft_flip_txn_destroy(acq);
			return -EAGAIN;	/* nothing acquired (all-or-none) */
		}
	}
	if (ft_flip_txn_commit((struct cds_ft *) ft, acq) != URCU_TXN_STATUS_OK)
		return -EAGAIN;		/* commit freed @acq; nothing acquired */
	return 0;
}

/*
 * FT-local order-pinned insert-between (was the engine's
 * urcu_txn_list_insert_between_prepare, dropped when the transaction engine was
 * adopted wholesale -- it only calls class-A primitives, so it lives here now).
 * insert_after_prepare derives the successor FRESH from @pos->next, so a peer
 * insert-after(@pos) that committed since the caller decided "@newp belongs
 * between @pos and @succ_expected" (by key order) would silently land @newp
 * BEFORE the peer's node.  This variant refuses (-EAGAIN) unless @pos->next
 * still equals @succ_expected, and records @succ_expected as the &pos->next
 * expected old -- so a later interposition fails the commit's value CAS instead
 * of being adopted.  -ENOENT when @pos itself was deleted.  Mirrors the new
 * insert_after_prepare edge recording, including its succ == pos self-loop guard
 * skip; compose on a DEFAULT (read-your-own-writes) handle.
 */
static inline
int ft_txn_list_insert_between_prepare(struct urcu_txn *txn,
		struct urcu_txn_list_node *newp,
		struct urcu_txn_list_node *pos,
		struct urcu_txn_list_node *succ_expected)
{
	void *pn = urcu_txn_load(txn, (void **) &pos->next, URCU_TXN_TAG);

	if (urcu_txn_list_is_marked(pn))
		return -ENOENT;				/* @pos was deleted */
	if ((struct urcu_txn_list_node *) pn != succ_expected)
		return -EAGAIN;				/* order intent stale: re-derive */
	/*
	 * Guard succ_expected->next (the slot del(succ) marks) so the &succ->prev
	 * store serializes against del(succ) exactly as &pos->next does; skip it
	 * when succ == pos (self-looping sentinel), whose next IS &pos->next, the
	 * forward store's own slot.
	 */
	if (succ_expected != pos && urcu_txn_list_is_marked(urcu_txn_load_validate(
			txn, (void **) &succ_expected->next, URCU_TXN_TAG)))
		return -EAGAIN;				/* succ (a neighbour) deleted: retry */

	/* Build the fresh node invisibly, then record the two forward edges. */
	newp->next = succ_expected;
	newp->prev = pos;
	urcu_txn_store_mw(txn, (void **) &pos->next, succ_expected, newp, URCU_TXN_TAG);
	urcu_txn_store_mw(txn, (void **) &succ_expected->prev, pos, newp, URCU_TXN_TAG);
	return 0;
}

/*
 * Resolve @src (a live child slot of the node under recompaction) to the
 * definite child it currently denotes.  The retire is fenced first
 * (ft_meta_lock_acquire set FT_STATE_LOCK in the node's state word before
 * this loop), so no peer can newly republish @src -- its §4.B clean-LIVE guard
 * fails against the LOCK bit -- and the only proxy this can meet is a peer
 * child-recompact that parked BEFORE the fence and is now doomed to abort.
 * urcu_txn_read reads @src's COMMITTED logical value, resolving a proxy to the
 * value it will settle to (helping a doomed peer to its terminal status), so the
 * copy builds against a definite child.  COMMITTED, not read-your-own-writes:
 * this reproduces the old urcu_txn_resolve_prio, which read the raw slot and
 * never consulted this attempt's own buffered records.  It is load-bearing --
 * during the BUILD phase a bulk op's pending edits live in the descriptor, not
 * in the slot, so a txn-level urcu_txn_load would return the PENDING value (e.g.
 * NULL for a source child the op has buffered a detach for) and drop a child the
 * copy must preserve; the removal a DEL recompact intends is applied separately,
 * by the by-IDENTITY skip of nullify_node_flag_ptr above, not by reading a
 * pending NULL here.  Bypassing the txn read set also avoids entering @src into
 * this attempt's read set, which would add a spurious commit-time validate.  The
 * old priority-eviction (urcu_mcas_outranks) is retired: single-driver liveness
 * comes from age-escalation, and the node-level node lock -- not a per-slot
 * freeze -- keeps @src from going stale against PEERS between here and the
 * commit.  Always resolves (returns 1); the vestigial int return keeps the call
 * sites' abandon path, now dead, as documentation.  FT proxy tag.
 */
static inline
int ft_flip_txn_resolve_prio(struct ft_flip_txn *t, void **src, void **out)
{
	*out = urcu_txn_read(src, FT_FLIP_PROXY_TAG);
	return 1;
}

#ifdef FT_ENABLE_TRACING
#include <stdio.h>
#include <stdlib.h>
/*
 * Flight-recorder mis-wire detector (tracing builds only): a PLAIN
 * compressed-tagged @edge whose target fails the identity round-trip is the
 * task-#12 corruption class (compressed edge -> live internal node: reading
 * &cn->child yields the internal node's bitmap word; the MW-oracle SIGSEGV).
 * Round-trip: the target's own (parent, offset) pair names its REAL slot --
 * if that slot holds the SAME address under an INTERNAL tag, the tree wires
 * this address as an internal node and @edge is proven mis-tagged; a set
 * tombstone bit means the target was retired (recycle/ABA); len == 0 is
 * never a valid path.  On detection: emit the enriched miswire event, dump
 * the flight-recorder ring, abort -- the last events before the abort walk
 * to whoever published the edge (skill: lttng-tracing-root-cause-analysis).
 */
static __attribute__((unused))
void ft_trace_miswire_check(struct cds_ft *ft,
		struct cds_ft_inode_flag *edge, unsigned int site)
{
	struct cds_ft_compressed_node *cn;
	struct cds_ft_metadata *meta;
	uintptr_t state;
	struct cds_ft_inode_flag *rt_parent, *rt_val = NULL;
	struct cds_ft_inode_flag **rt_slotp;
	bool bad;

	if (!ft_node_compressed(edge))
		return;
	cn = ft_compressed_node_ptr(edge);
	meta = cds_ft_item_to_metadata((struct cds_ft_inode *) cn);
	state = (uintptr_t) urcu_txn_read((void **) &meta->state,
			FT_STATE_PROXY);
	rt_parent = ft_resolve_flip_proxy(ft_parent_node(
			rcu_dereference(meta->parent)));
	rt_slotp = rt_parent ? ft_get_parent_slot(meta, ft) : NULL;
	if (rt_slotp)
		rt_val = ft_resolve_flip_proxy(rcu_dereference(*rt_slotp));
	/*
	 * NOT a criterion: a set tombstone alone.  Trace analysis (2026-07-06)
	 * proved that catch benign: a descent that read the slot just before a
	 * peer's retire flip legally holds the dead-but-RCU-live target for a
	 * few hundred ns, and its own commit then aborts on the expected-old
	 * CAS.  A LIVE legit cn must round-trip to ITSELF: its parent's slot
	 * holds either its plain flag or its SKIP form (which names the
	 * external head -- resolve it back to the cn to compare).  A live
	 * target that round-trips ANYWHERE ELSE is corruption: a mis-tagged
	 * edge to an internal node, or recycled memory whose metadata walks
	 * off into the weeds (the len byte alone cannot discriminate --
	 * uint8_t never exceeds FT_MAX_KEY_LEN).
	 */
	bool self_rt = false;

	if (rt_val) {
		if (ft_node_ptr(rt_val) == (void *) cn &&
		    ft_node_compressed(rt_val))
			self_rt = true;
#ifdef FEATURE_FT_SKIP_COMPRESSED
		else if (ft_node_skip_compressed(rt_val) &&
			 ft_skip_to_compressed(ft, rt_val) == cn)
			self_rt = true;
#endif
	}
	bad = cn->len == 0 ||
		(!(state & FT_STATE_TOMBSTONE) && !self_rt);
	if (caa_likely(!bad))
		return;
	FT_TP(miswire, site, (const void *) edge, (const void *) cn,
		(unsigned int) cn->len, state, (const void *) rt_parent,
		(const void *) rt_val);
	fprintf(stderr, "FT MISWIRE site %u edge %p target %p len %u "
		"state %#lx rt_parent %p rt_val %p\n",
		site, (void *) edge, (void *) cn, (unsigned int) cn->len,
		(unsigned long) state, (void *) rt_parent, (void *) rt_val);
	(void) system("lttng snapshot record 1>&2");
	abort();
}
#ifndef FT_LIGHT_TRACING	/* -DFT_LIGHT_TRACING: keep tracepoints, drop the
				 * per-descent round-trip detector (its overhead
				 * widens the MW race window and suppresses the
				 * mis-wire -- snapshot on the natural SIGSEGV instead). */
#define FT_TRACE_MISWIRE(ft, edge, site) ft_trace_miswire_check(ft, edge, site)
#else
#define FT_TRACE_MISWIRE(ft, edge, site) do { (void) (ft); (void) (edge); (void) (site); } while (0)
#endif
#else
#define FT_TRACE_MISWIRE(ft, edge, site) do { } while (0)
#endif	/* FT_ENABLE_TRACING */


/*
 * Ordinal-cell list maintenance.
 *
 * Mirrors the ORD_CHAIN chain maintenance, but the key-ordered doubly-linked
 * list threads the library-owned cells (one per distinct-key head) via
 * ft_ord_cell.ord_next / ord_prev instead of in-leaf fields.  Each point op
 * flips the (<=2) live neighbour edges through one flip-batch so a
 * bidirectional ordered reader sees the splice atomically; the spliced-in /
 * replacement cell pre-sets its own links with plain stores (not yet ord-
 * reachable), while an unspliced cell keeps its links for parked readers
 * until its deferred free.  Runtime-gated by group->ordered_list_set: a
 * point op consults these only when the list is enabled.
 *
 * RUNS UNDER WRITER EXCLUSION; no concurrent writer races, no proxy at rest.
 * Promotion (ft_unchain_node) and replace (cds_ft_replace) need NO list op:
 * the cell stays put and only cell->node is retargeted.  Bulk ops (merge /
 * graft / graft_swap / detach) maintain the list through the run helpers
 * below (run_detach / run_splice / run_replace / run_unlink) and the merge
 * interleave.
 */

struct ft_ord_cell_edge {
	struct ft_ord_cell **slot;	/* a neighbour's ord_next / ord_prev slot */
	struct ft_ord_cell *old_target;
	struct ft_ord_cell *new_target;
	/*
	 * Per-edge engine proxy tag: URCU_TXN_TAG (bit 0) for an ORDERED-CELL
	 * list edge (readers resolve via urcu_txn_list_resolve), or 0 / left
	 * unset for a STRUCTURAL trie edge, which ft_edge_tag() normalizes to
	 * FT_FLIP_PROXY_TAG (readers resolve via ft_resolve_flip_proxy).  Both
	 * families ride the ONE mtxn; the engine carries the tag per record.  A
	 * designated-initializer / zero-initialized edge defaults to structural.
	 */
	uintptr_t tag;
};

/* Resolve an edge's engine proxy tag: unset (0) => the structural 0xF tag. */
static inline
uintptr_t ft_edge_tag(const struct ft_ord_cell_edge *edge)
{
	return edge->tag ? edge->tag : FT_FLIP_PROXY_TAG;
}

/*
 * Deferred in-place leaf-delete publish (the remove dual of
 * ft_insert_commit).  When a leaf delete keeps the holder above min_child --
 * the common case, no recompaction -- its single reader-visible forward store
 * is the moment the key leaves the structural index.  The popcount/pigeon
 * replace_ptr RECORDS that store here (@slot transitions @old_val -> @new_val)
 * instead of doing it, so ft_detach_node can commit it in ONE flip together
 * with the dead head cell's ordered-list unsplice (ft_remove_one_commit): a
 * reader then never observes the key gone from one index but present in the
 * other.  Two store shapes share this:
 *   - leaf delete: @new_val == NULL (the child slot clears to empty).
 *   - external promote: a childless holder's external chain is promoted into
 *     the parent slot, so @new_val == the external chain head (the same value
 *     the immediate store would publish).  The promoted external's back-pointer
 *     is wired (ft_set_parent) BEFORE the deferral, parent-first.
 *
 * @armed is set only on the in-place path; the recompaction (-EFBIG) path
 * publishes its rebuilt node itself and leaves this untouched.  For a delete
 * the primitive records the node in @state_meta so its nr_child-- fuses into
 * the same commit flip (ft_remove_one_commit) -- exact and atomic with the
 * forward store (writers and canonicalize read the count; readers do not
 * navigate by it); a promote replaces a child, so nr_child is unchanged.  The
 * pigeon occupancy bitmap bit is left SET on a delete (a sticky soft-delete
 * hint, never cleared in place: the pointer load is the source of truth and a
 * later recompact rebuilds a clean bitmap), exactly as the popcount layout
 * already soft-deletes.
 */
struct ft_remove_pub {
	struct cds_ft_inode_flag **slot;
	struct cds_ft_inode_flag *old_val;
	struct cds_ft_inode_flag *new_val;	/* NULL for delete; chain head for promote */
	struct cds_ft_metadata *state_meta;	/* non-NULL (delete) => fuse its nr_child-- */
	/*
	 * External promote: the promoted head's back-channel re-parent
	 * (cell->parent list-on, node->prev list-off) captured at arm time and
	 * COMMITTED with the forward flip -- an eager arm-time store survived a
	 * commit ABORT (and the remove retry loop makes ABORT routine), leaving
	 * the still-second-in-chain head pointing at the holder.  NULL field
	 * when the armed op is not a promote.
	 */
	struct cds_ft_inode_flag **head_parent_field;
	struct cds_ft_inode_flag *head_parent_old;
	struct cds_ft_inode_flag *head_parent_new;
	bool armed;
};

static enum urcu_txn_status ft_ord_cell_flip_into(struct cds_ft *ft, struct ft_flip_txn *t,
		struct ft_ord_cell_edge *edges, unsigned int n);

/*
 * Commit a single edge as a lone publish.  A lone edge is ONE release store: it
 * parks no proxy, allocates no descriptor (hence is infallible -- cannot OOM),
 * and owes no grace period, so it is byte-identical to a bare rcu_assign_pointer.
 * The {slot, old, new} descriptor shape is retained at the call sites for
 * uniformity, but @old is not needed under one writer and no engine transaction
 * is created for a single slot -- publishing the direct new value is atomic and
 * self-resolving (a later reader loads either the old or the new pointer, never
 * a proxy).  The lone-edge publish helpers (ft_root_edge_flip, ft_chain_next_flip,
 * the point insert/remove single-slot external_nodes publishes) and
 * ft_ord_cell_flip_try's n==1 fast path all commit through here; the edge's tag
 * is irrelevant (no proxy is installed).
 */
static
void ft_ord_cell_flip_one(struct ft_ord_cell_edge *edge)
{
	FT_TP(edge_lone, (const void *) edge->slot,
		(const void *) edge->old_target,
		(const void *) edge->new_target);
	rcu_assign_pointer(*edge->slot, edge->new_target);
}

/*
 * Edge old/new TARGET for an ordinal-cell link: a real cell @c, or the trie's
 * circular-sentinel pseudo-cell when the link points "off the end" (@c NULL).
 * lnode is the cell's first field (offset 0), so this ft_ord_cell * value is
 * bit-identical to the urcu_txn_list_node * actually stored in the slot.
 *
 * In the sentinel topology the per-trie head/tail endpoint flips are no longer
 * separate edges: the first/last cell's neighbour IS the sentinel, so recording
 * its back-edge (&sentinel.node.next / .prev) through a normal 2-edge splice IS
 * the old ord_cell_head / ord_cell_tail update.  ft_ord_cell_endpoint_edge is
 * therefore gone; boundary neighbours just resolve to the sentinel pseudo-cell.
 */
static inline
struct ft_ord_cell *ft_ord_or_sentinel(const struct cds_ft *ft,
		struct ft_ord_cell *c)
{
	return c ? c : ft_ord_sentinel_cell(ft);
}

/*
 * Append @ft's ordinal-cell sentinel endpoint edges for a whole-list transfer
 * (the sentinel-model replacement for the old head/tail endpoint flips).  The
 * sentinel's next edge transitions @head_old -> @head_new and its prev edge
 * @tail_old -> @tail_new, where a NULL endpoint denotes the sentinel itself (an
 * empty boundary, ft_ord_or_sentinel) -- so a side EMPTYING flips its sentinel
 * to point at itself, a side FILLING flips it to point at the incoming run.  A
 * no-op transition (old == new) records nothing.
 *
 * When @relink_dest is non-NULL (with @relink_incoming set), the INCOMING run
 * [head_new..tail_new] also has its outer links repointed from @relink_dest's
 * sentinel to this trie's, in the SAME flip.  This is sound ONLY because the
 * run's source (@relink_dest) is an EXCLUSIVE trie with no straddlers
 * (cds_ft_merge_at's appear: @relink_dest is the fresh detach product).
 *
 * An OUTGOING run is deliberately NEVER relinked to a LIVE foreign sentinel
 * in-flip: a source straddler resolving such an outer link (global selector ->
 * new) would land on the foreign sentinel and dereference it as a cell.  The
 * cross-trie dual (ft_root_list_swap_publish_dual) instead NULL-terminates the
 * moved run (universal end) and finalizes it to the receiving sentinel AFTER a
 * drain; the single-side movers (detach / graft / merge src) leave the run at
 * the source sentinel (relink_dest NULL) and re-home it post-drain.
 */
static
unsigned int ft_ord_sentinel_edges(struct cds_ft *ft,
		struct ft_ord_cell *head_old, struct ft_ord_cell *head_new,
		struct ft_ord_cell *tail_old, struct ft_ord_cell *tail_new,
		struct cds_ft *relink_dest, bool relink_incoming,
		struct ft_ord_cell_edge *edges, unsigned int n)
{
	struct ft_ord_cell *self = ft_ord_sentinel_cell(ft);
	struct ft_ord_cell *hn_old = ft_ord_or_sentinel(ft, head_old);
	struct ft_ord_cell *hn_new = ft_ord_or_sentinel(ft, head_new);
	struct ft_ord_cell *tp_old = ft_ord_or_sentinel(ft, tail_old);
	struct ft_ord_cell *tp_new = ft_ord_or_sentinel(ft, tail_new);

	if (hn_old != hn_new) {
		edges[n].tag = URCU_TXN_TAG;	/* ordered-cell edge */
		edges[n].slot = (struct ft_ord_cell **) &ft->ord_sentinel.node.next;
		edges[n].old_target = hn_old;
		edges[n].new_target = hn_new;
		n++;
	}
	if (tp_old != tp_new) {
		edges[n].tag = URCU_TXN_TAG;	/* ordered-cell edge */
		edges[n].slot = (struct ft_ord_cell **) &ft->ord_sentinel.node.prev;
		edges[n].old_target = tp_old;
		edges[n].new_target = tp_new;
		n++;
	}
	/*
	 * Incoming-run relink, EXCLUSIVE source only (merge_at appear).  The outgoing
	 * direction (relink a moved run to a foreign sentinel) is intentionally absent
	 * -- see the function comment: live cross-trie moves NULL-terminate + finalize
	 * after a drain instead.
	 */
	if (relink_dest && relink_incoming) {
		struct ft_ord_cell *dest = ft_ord_sentinel_cell(relink_dest);

		if (head_new) {
			edges[n].tag = URCU_TXN_TAG;	/* ordered-cell edge */
			edges[n].slot = (struct ft_ord_cell **) &head_new->lnode.prev;
			edges[n].old_target = dest;
			edges[n].new_target = self;
			n++;
		}
		if (tail_new) {
			edges[n].tag = URCU_TXN_TAG;	/* ordered-cell edge */
			edges[n].slot = (struct ft_ord_cell **) &tail_new->lnode.next;
			edges[n].old_target = dest;
			edges[n].new_target = self;
			n++;
		}
	}
	return n;
}

/*
 * Fence an EMPTY destination root for a whole-trie root-level attach, and
 * decide emptiness UNDER that fence.
 *
 * WHY.  The empty-dst root swaps (cds_ft_graft key_len == 0, cds_ft_merge_at
 * dst_key_len == 0 && cnt_dst == 0) used to test emptiness, then allocate, then
 * create a txn, then -- on the merge -- run an ENTIRE ft_detach_keylen of the
 * source, and only then record the swap.  The commit validates its RYW
 * expected-olds, but both of them (@dst_ft->root and the old root's state word)
 * are READ AT RECORD TIME, so a peer that populated the destination inside that
 * window is matched by construction instead of detected.  The swap then retires
 * and frees the populated root: the peer's keys vanish while BOTH ops report
 * success, and the peer's nodes keep parent pointers into reclaimed memory.
 * That interleaving is contract-LEGAL -- fractal-trie.h grants concurrent
 * cross-trie attaches on a shared destination -- so it is the implementation
 * that has to arbitrate.
 *
 * WHAT THE FENCE BUYS.  Every path that can populate this root must first take
 * its LOCK: a republish goes through ft_node_recompact's {C,P,(GP)} acquire
 * (C == the root), and an in-place attach's nr_child CAS honours
 * FT_STATE_INPLACE_WAIT_MASK.  So once the mark is ours, both the root POINTER
 * and the root's state WORD are stable through the commit, which is what makes
 * "@dst_ft is empty" still true at the linearization point rather than merely
 * true when it was sampled.  The caller records the FENCED tombstone
 * (ft_flip_txn_record_tombstone_locked, expected-old @snap|LOCK) so any
 * state change that did slip under the fence aborts the commit instead of being
 * ratified -- and that restored exclusion is what ft_root_list_swap_publish's
 * infallible commit has always assumed.
 *
 * @external_nodes is checked but NOT covered by the fence: no retire validates
 * that word (a known, separate hole -- ft_insert_park_external_nodes publishes
 * it with no lock).  It cannot be reached here by a CONTRACT-legal peer, since
 * parking externals on the root needs either a point insert (not granted
 * against an attach) or a nil-key root attach (blocked by this very fence).
 *
 * Returns 0 with @root_out / @meta_out / @snap_out set and the fence HELD (the
 * caller owns its release: hand it to the txn via ft_flip_txn_lock_register,
 * or ft_meta_lock_release on a bail).  -EAGAIN if a peer holds the root or
 * keeps republishing it; -EEXIST if the destination is not, or no longer,
 * empty.
 */
#define FT_ROOT_FENCE_REREAD_MAX	4
static inline
int ft_root_attach_fence_empty(struct cds_ft *dst_ft,
		struct cds_ft_inode_flag **root_out,
		struct cds_ft_metadata **meta_out, uintptr_t *snap_out)
{
	unsigned int attempt;

	for (attempt = 0; attempt < FT_ROOT_FENCE_REREAD_MAX; attempt++) {
		/*
		 * Resolve a peer's parked flip proxy, exactly as
		 * ft_root_metadata does and for the same reason: a raw load of
		 * the root slot can hand back a RECORD address mid-commit, and
		 * ft_node_ptr only masks the node flags -- the tag survives and
		 * cds_ft_item_to_metadata then dereferences the record as a node.
		 * (Found the hard way: this helper segfaulted right here.)
		 */
		struct cds_ft_inode_flag *root =
			ft_resolve_flip_proxy(rcu_dereference(dst_ft->root));
		struct cds_ft_metadata *rmeta =
			cds_ft_item_to_metadata(ft_node_ptr(root));
		uintptr_t snap;

		if (ft_meta_lock_acquire(rmeta, &snap))
			return -EAGAIN;
		/*
		 * The root pointer can have moved between the load and the mark
		 * (a peer's recompact republishing it), leaving the fence on a
		 * node that is already retired while a fresh one is live.  Drop
		 * it and re-read: the peer's republish is a COMPLETED event, so
		 * this converges -- but bound the turns anyway and report -EAGAIN
		 * rather than spin against a stream of peers.
		 */
		if (ft_resolve_flip_proxy(rcu_dereference(dst_ft->root)) != root) {
			ft_meta_lock_release(rmeta);
			continue;
		}
		/*
		 * Emptiness from the MARK's clean snapshot -- the word the fence
		 * froze -- not from a fresh read that could race the mark.
		 */
		if (ft_state_nr_child(snap) != 0 || rmeta->external_nodes) {
			ft_meta_lock_release(rmeta);
			return -EEXIST;
		}
		*root_out = root;
		*meta_out = rmeta;
		*snap_out = snap;
		return 0;
	}
	return -EAGAIN;
}

/*
 * Whole-trie root + ordered-list transfer, fused in ONE flip.  The empty-dst
 * root-level graft / cds_ft_merge_at appear (dst adopts a whole list), and the
 * src-retire disappear (src empties), publish the root transition @struct_old ->
 * @struct_new together with the sentinel endpoint edges (ft_ord_sentinel_edges)
 * so a reader never sees the keys reachable in the structure but the ordered
 * list inconsistent -- it resolves the root and the sentinel proxies to ONE flip
 * phase.  @relink_dest (and @relink_incoming) cross-link the moved run's outer
 * boundary to the destination trie's sentinel in the same flip (see
 * ft_ord_sentinel_edges); pass NULL when the moved cells are re-homed separately
 * (a later run-splice / interleave, or the source is exclusive).
 */
#define FT_ROOT_LIST_SWAP_MAX_EDGES	5	/* root + 2 sentinel + 2 relink */
static
void ft_root_list_swap_publish(struct cds_ft *ft, struct ft_flip_txn *txn,
		struct cds_ft_inode_flag **struct_slot,
		struct cds_ft_inode_flag *struct_old,
		struct cds_ft_inode_flag *struct_new,
		struct ft_ord_cell *head_old, struct ft_ord_cell *head_new,
		struct ft_ord_cell *tail_old, struct ft_ord_cell *tail_new,
		struct cds_ft *relink_dest, bool relink_incoming)
{
	struct ft_ord_cell_edge edges[FT_ROOT_LIST_SWAP_MAX_EDGES] = { 0 };
	unsigned int n = 0;

	edges[n].slot = (struct ft_ord_cell **) struct_slot;
	edges[n].old_target = (struct ft_ord_cell *) struct_old;
	edges[n].new_target = (struct ft_ord_cell *) struct_new;
	n++;
	n = ft_ord_sentinel_edges(ft, head_old, head_new, tail_old, tail_new,
			relink_dest, relink_incoming, edges, n);
	/*
	 * @txn is the caller-PRE-RESERVED bounded txn (every Class-G root swap
	 * reserves in its fallible prefix), committed infallibly here -- the
	 * un-abortable post-drain root swap reaches an allocation-free commit.
	 */
	/* Bulk op, not yet MW-hardened: ABORT unreachable under its exclusion. */
	(void) ft_ord_cell_flip_into(ft, txn, edges, n);
}

/*
 * Publish a lone structural root edge as a single-edge flip descriptor.  A lone
 * edge commits as one release store (no proxy, no group flip, no grace period)
 * -- byte-identical to a bare rcu_assign_pointer -- but it is captured as a
 * {slot, old, new} descriptor edge so a future multi-writer MCAS commit covers
 * the root slot uniformly: a bare store would discard @old (the compare-and-swap
 * "expected" value) and sit outside the descriptor protocol, yet a root slot can
 * be in a concurrent writer's word-set (e.g. a near-root insert that recompacts
 * and republishes the root).  This is the ordered-list-OFF arm of every Class-G
 * root swap (detach / graft / graft_swap / merge), where there is no head/tail
 * endpoint to fuse and ft_root_list_swap_publish would reduce to this anyway.
 */
static
void ft_root_edge_flip(struct cds_ft *ft,
		struct cds_ft_inode_flag **struct_slot,
		struct cds_ft_inode_flag *struct_old,
		struct cds_ft_inode_flag *struct_new)
{
	struct ft_ord_cell_edge edge = {
		.slot = (struct ft_ord_cell **) struct_slot,
		.old_target = (struct ft_ord_cell *) struct_old,
		.new_target = (struct ft_ord_cell *) struct_new,
	};

	(void) ft;	/* a lone edge commits on an on-stack txn (no reclaim) */
	ft_ord_cell_flip_one(&edge);
}

/*
 * Publish a duplicate-chain forward link (@slot transitions @old -> @new) as a
 * single-edge flip descriptor.  @slot is a LIVE chain node's `next' pointer
 * (cds_ft_node.next), read by cds_ft_for_each_duplicate_rcu; @new is either a
 * fully-built leaf being appended (ft_chain_node: NULL -> node) or an
 * already-published successor a remove relinks past (ft_unchain_node: node ->
 * next_node).  Neither exposes any build-invisible cluster -- the appended leaf
 * is complete and the relink target is already reachable -- so a lone-edge flip
 * (one release store, byte-identical to rcu_assign_pointer) is the right and
 * sufficient MCAS-expressible form; no fusion with another edge is needed.
 */
static
void ft_chain_next_flip(struct cds_ft *ft, struct cds_ft_node **slot,
		struct cds_ft_node *old, struct cds_ft_node *new)
{
	struct ft_ord_cell_edge edge = {
		.slot = (struct ft_ord_cell **) slot,
		.old_target = (struct ft_ord_cell *) old,
		.new_target = (struct ft_ord_cell *) new,
	};

	(void) ft;	/* a lone edge commits on an on-stack txn (no reclaim) */
	ft_ord_cell_flip_one(&edge);
}

/*
 * Build a flip-latch edge for a per-node STATE WORD transition
 * (struct cds_ft_metadata.state -- nr_child / tombstone / proxy; see
 * fractal-trie-internal.h and doc/design/mcas-multiwriter-readiness.md §4.2).
 *
 * @state_slot is a scalar uintptr_t, not a pointer slot, but a uintptr_t and a
 * void * are the same width, so the word rides the SAME {slot, old, new} edge
 * machinery as a structural pointer edge (the existing flips already cast
 * cds_ft_inode_flag ** to ft_ord_cell **): under one writer the commit is a
 * plain store of @new_state; under multi-writer MCAS it becomes a
 * CAS-with-expected on the word.
 *
 * This is the "commit it" primitive: it lets a node-state change (e.g. the
 * remove-side nr_child--) ride the op's flip commit instead of mutating the
 * live node's word in place, and is shared with the future deleted-flag
 * (tombstone) commit.  Fill an edge here, then place it in the op's edge array
 * (ft_ord_cell_flip_one / _into) alongside the structural edges.  The proxy
 * bit (state bit 0) is what lets a reader/writer recognise a mid-flip word; it
 * is dormant under a single writer, where the commit just settles to @new_state.
 */
static inline
void ft_state_edge(struct ft_ord_cell_edge *edge, uintptr_t *state_slot,
		uintptr_t old_state, uintptr_t new_state)
{
	edge->slot = (struct ft_ord_cell **) state_slot;
	edge->old_target = (struct ft_ord_cell *) old_state;
	edge->new_target = (struct ft_ord_cell *) new_state;
}

/*
 * Latch-honoring standalone STATE-WORD transition (F3 lone-store hardening):
 * apply @f(s) to the state word through a proxy-tolerant CAS loop.  The prior
 * shape -- raw read + one release store of the derived value -- wrote
 * f(latch-pointer) back into the word when a peer's MCAS proxy was parked at
 * the read (e.g. proxy|TOMBSTONE = a corrupted record pointer the peer's
 * settle then CAS-misses, leaving a dangling latch forever).  The loop waits
 * out a parked proxy (owner settles in bounded steps; under one writer no
 * proxy ever appears, so the first CAS succeeds -- behaviour-identical) and
 * re-derives from the fresh value on CAS failure, so a racing peer's
 * committed state change is never overwritten with a stale image.  These
 * standalone marks remain OUTSIDE commit arbitration by design (no
 * expected-old validation of a plan -- their transitions are self-contained:
 * one-way bit sets and exact-count adjustments); paths whose retire must be
 * atomic with an unlink use the recorded ft_flip_txn_record_* forms instead.
 */
static inline
void ft_meta_state_transition(struct cds_ft_metadata *meta,
		uintptr_t (*f)(uintptr_t), uintptr_t wait_mask)
{
	for (;;) {
		uintptr_t s = CMM_LOAD_SHARED(meta->state);

		if (caa_unlikely(s & wait_mask)) {
			/*
			 * Bounded by the latch owner's settle (owner-only; a
			 * writer dying mid-install would wedge this, but
			 * writer death mid-mutation is already fatal to the
			 * trie by contract).  @wait_mask is FT_STATE_PROXY for a
			 * self-contained one-way mark (tombstone) and
			 * FT_STATE_INPLACE_WAIT_MASK for the nr_child count edge,
			 * which under a DLM build must also wait out a peer's
			 * node lock so its SW-parked state edge is not
			 * clobbered (see FT_STATE_INPLACE_WAIT_MASK).
			 */
			caa_cpu_relax();
			continue;
		}
		if (caa_likely(uatomic_cmpxchg(&meta->state, s, f(s)) == s)) {
			FT_TP(edge_lone, (const void *) &meta->state,
				(const void *) s, (const void *) f(s));
			return;
		}
	}
}

static inline
uintptr_t ft_state_f_nr_child_dec(uintptr_t s)
{
	return s - FT_STATE_NR_CHILD_ONE;
}

static inline
uintptr_t ft_state_f_tombstone(uintptr_t s)
{
	return s | FT_STATE_TOMBSTONE;
}

/*
 * The remove-side nr_child-- as a latch-honoring standalone transition,
 * replacing the bare in-place ft_meta_nr_child_dec on a LIVE node.  Same net
 * effect under one writer; under multi-writer the CAS loop re-derives from
 * the current word, so a peer's concurrent state commit is never overwritten
 * with a stale count (doc/design/mcas-multiwriter-readiness.md §4.2).  The
 * structural child-slot edge still commits separately here; fusing the two
 * into one flip (atomic {structure, count}) is a follow-up.
 */
static
void ft_meta_nr_child_dec_flip(struct cds_ft_metadata *meta)
{
	ft_meta_state_transition(meta, ft_state_f_nr_child_dec,
			FT_STATE_INPLACE_WAIT_MASK);
}

/*
 * Set a node's one-way LIVE->DEAD tombstone (state bit 1, §4.B freeze-on-free)
 * as a latch-honoring standalone transition, at the point the node is DETACHED
 * from the trie.  Under one writer this is one uncontended CAS on a node about
 * to be reclaimed -- behaviour-identical; under multi-writer MCAS the mark is
 * what a concurrent writer targeting the node validates (expected = live) so
 * its commit fails once the node is dead (doc §4.B), and the CAS loop keeps
 * the mark from trampling a parked latch or a peer's concurrent state commit.
 * Infallible; idempotent (re-marking a dead node is a same-value CAS).  The
 * mark and the structural unlink that retires the node should eventually ride
 * ONE flip (atomic detach); a standalone mark is the bridge.
 */
static
void ft_meta_tombstone_set_flip(struct cds_ft_metadata *meta)
{
	ft_meta_state_transition(meta, ft_state_f_tombstone, FT_STATE_PROXY);
}

/*
 * Record a node's one-way LIVE->DEAD tombstone (state bit 1, §4.B) as an edge in
 * the op's flip-txn @t, so the freeze mark commits ATOMICALLY with the very flip
 * that structurally unlinks the node -- the "atomic detach" that
 * ft_meta_tombstone_set_flip's lone-edge bridge stands in for (that standalone
 * helper remains for retire sites whose unlink is not yet a flip-txn commit).
 * The state word reserves bit 0 (FT_STATE_PROXY) for the engine's in-band proxy
 * marker, so the mark rides an MCAS edge like any structural slot; a concurrent
 * reader of nr_child resolves the transient proxy via ft_meta_nr_child_load.
 * Value-CAS old -> old|TOMBSTONE.  @t must be reserved for this extra edge.
 * Idempotent (re-marking a dead node is a same-value edge); under one writer it
 * is a no-op bit a reader ignores, so behaviour-identical.
 */
static inline
uintptr_t ft_flip_txn_record_tombstone(struct ft_flip_txn *t,
		struct cds_ft_metadata *meta)
{
	/*
	 * READ-YOUR-OWN-WRITES: the expected old is this txn's own PENDING view
	 * of the state word (urcu_txn_load), not a raw meta->state read.  The
	 * word is in this record's WRITE set, so per the engine's read policy it
	 * must be read through the txn -- a raw committed read is a torn read-set
	 * the moment another edge in the SAME txn already rewrote the word, which
	 * the engine POISONS (commit aborts).  This happens on a retire that
	 * fuses with a recompaction: ft_node_recompact already recorded the old
	 * node's fenced {LOCK|s -> TOMBSTONE|s} edge into @t, so a plain
	 * tombstone reading the COMMITTED old disagrees with that pending new.
	 * The RYW load returns TOMBSTONE-already-set there, chaining to a no-op
	 * upgrade; with no prior edge it returns the committed value, so every
	 * other call site is behaviour-identical.
	 */
	uintptr_t old = (uintptr_t) urcu_txn_load(t->mtxn,
			(void **) &meta->state, FT_STATE_PROXY);

	ft_flip_txn_record_tag(t, (void **) &meta->state,
			(void *) old, (void *) (old | FT_STATE_TOMBSTONE),
			FT_STATE_PROXY);
	/*
	 * Return the RYW old so a caller retiring a set of nodes (the glue
	 * free-list) can tell whether THIS txn performs the LIVE->TOMBSTONE
	 * transition (old clean) or merely no-op-upgrades a word a peer already
	 * tombstoned (old & FT_STATE_TOMBSTONE) -- the latter must not also free
	 * the node.
	 */
	return old;
}

/*
 * Record a LIVE node's nr_child++ as an edge in the op's flip-txn @t, so the
 * count goes live ATOMICALLY with the structural publish that makes the counted
 * child reachable -- the insert-side mirror of the remove path's fused
 * nr_child-- (ft_ord_cell_unsplice_commit's @state_meta edge).
 *
 * WHY THE COUNT CANNOT BE APPLIED IN PLACE.  The one-commit insert/graft
 * RESERVE occupies its byte with a bit-set + NULL slot that reads as
 * not-present, and settles the real child at commit.  Bumping nr_child in place
 * mutates a PUBLISHED node OUTSIDE @t, and nothing rolls that back: on a commit
 * ABORT the op re-descends, finds the byte still absent (the reserved slot is
 * still NULL), reserves AGAIN and increments a SECOND time -- a permanent
 * stored == counted + 1 divergence that cds_ft_verify reports as an nr_child
 * mismatch (and, on a full pigeon node, as nr_child 257 > max_child 256).
 * Recorded as an edge instead, the increment is discarded with the aborted
 * attempt and re-derived by the retry, so reserve-then-abort is idempotent.
 *
 * The edge also makes the count coherent for READERS, which the in-place bump
 * never was: between the reserve and the commit the counted child is invisible
 * (NULL slot), so an eager nr_child counts a child no reader can reach.
 *
 * THIS EDGE *IS* THE §4.B GUARD -- do not plant one beside it.  The expected
 * old is the CLEAN-LIVE value (tombstone and lock masked off), exactly
 * what ft_flip_txn_guard_parent validates, so a peer that freezes, retires or
 * locks the holder between here and the commit fails this record's CAS
 * just as it would have failed the guard; and a holder already DEAD at record
 * time cannot match a clean-live expectation, so the publish can never settle
 * into a retired copy.  It is the guard plus the increment in ONE record --
 * the same "the release record IS the guard, and it is strictly stronger, so a
 * converted site REPLACES its guard rather than adding to it" rule stated at
 * ft_flip_txn_record_release_lock.
 *
 * ★ WHY THAT MATTERS FOR SPEED, not just tidiness.  A guard recorded BESIDE
 * this edge lands on the SAME state word, and a second touch of a slot the
 * transaction has already touched is precisely what the engine's age-0 fast
 * path refuses to resolve: it keeps a Bloom filter but never calls find, so any
 * same-slot coincidence sets esc_pending and FORCES the commit to ABORT
 * unpublished, to be re-run at age 1+ where the full chaining path runs
 * (rcu-txn.h, "the trade: age 0 never runs find, at the price of a whole extra
 * attempt").  Keeping both records therefore costs EVERY such insert a second
 * commit attempt -- measured at 650460 age-0 aborts over one in-place ft_unit
 * run versus 2256 with the single fused record, a 278x difference.  Same-slot
 * chaining is CORRECT, but on this path it is never free.
 *
 * ORDERING / RESERVATION.  Recorded in the guard's slot, so it inherits
 * the arm's existing guard reservation (net-zero) and stays on the safe side of
 * the release-then-guard rule: a lock release on this word is recorded by the
 * reserve's recompact EARLIER in the same txn, so this edge chains onto it
 * ({LOCK|s -> s} then {s -> s+1} = release AND increment in one record)
 * rather than poisoning it.
 *
 * READ-YOUR-OWN-WRITES: expected-old comes from urcu_txn_load, not a raw read,
 * for the reason spelled out on ft_flip_txn_record_tombstone -- the word is in
 * this txn's own write set whenever a release/tombstone already touched it, and
 * a raw committed read would be a torn read-set the engine poisons.  A release
 * recorded earlier makes the RYW value ALREADY clean, so the mask is a no-op
 * there and the chain is exact.
 */
static inline
void ft_flip_txn_record_nr_child_inc(struct ft_flip_txn *t,
		struct cds_ft_metadata *meta)
{
	uintptr_t old = (uintptr_t) urcu_txn_load(t->mtxn,
			(void **) &meta->state, FT_STATE_PROXY);
	uintptr_t live = old & ~(uintptr_t) (FT_STATE_TOMBSTONE | FT_STATE_LOCK);

	assert(ft_state_nr_child(live) < FT_STATE_NR_CHILD_VALMASK);
	ft_flip_txn_record_tag(t, (void **) &meta->state,
			(void *) live, (void *) (live + FT_STATE_NR_CHILD_ONE),
			FT_STATE_PROXY);
}

/*
 * The FENCED variant for a node the op marked with ft_meta_lock_acquire: the
 * expected old is the mark's CLEAN snapshot with the fence bit -- NOT a fresh
 * raw read -- so the commit ratifies exactly the world the copy plan was
 * derived from (the CORE_682870 defect-1 fix: a stale plan can no longer be
 * ratified by a coincidentally-matching late capture).  The new value drops
 * the reversible fence and sets the one-way tombstone in the same atomic
 * transition {LOCK|s -> TOMBSTONE|s}: tombstone semantics (commit-atomic,
 * exactly-once retire token) are unchanged.  Any peer state change under the
 * fence -- a re-home's PSO pair, a fused count, a foreign tombstone -- makes
 * this record's expected old mismatch: the copier aborts and its fence is
 * cleared by the commit wrapper's registry.
 */
static inline
void ft_flip_txn_record_tombstone_locked(struct ft_flip_txn *t,
		struct cds_ft_metadata *meta, uintptr_t state_snapshot)
{
	ft_flip_txn_record_tag(t, (void **) &meta->state,
			(void *) (state_snapshot | FT_STATE_LOCK),
			(void *) (state_snapshot | FT_STATE_TOMBSTONE),
			FT_STATE_PROXY);
}

/*
 * The OTHER terminal of a FT_STATE_LOCK (MW lock-escalation model §9.3):
 * the holder needed EXCLUSION, not a retire -- the node is edited (or merely
 * protected) and SURVIVES the commit.  {LOCK|s -> s}: drop the lock, keep the
 * node live, atomically with the rest of the op's edge set.
 *
 * Same expected-old contract as the retire twin above -- the mark's CLEAN
 * snapshot, never a fresh read -- so it carries the same guarantee: any peer
 * state change on the locked node between the mark and the commit (a re-home's
 * PSO pair, a fused count, a foreign tombstone) mismatches this record's
 * expected old and ABORTS the holder, whose lock the registry then clears.
 * That is why a locked node needs NO separate ft_flip_txn_guard_parent: the
 * release record IS the guard (same word, same abort on a peer state change),
 * and it is strictly stronger -- the guard only makes US abort after the fact,
 * while the lock makes PEERS abort up front.  A converted site therefore
 * REPLACES its guard rather than adding to it.
 *
 * ORDERING RULE, load-bearing -- a guard and a release on ONE word are safe in
 * exactly ONE order:
 *
 *   release THEN guard  = harmless no-op.  urcu_txn_load is read-your-writes
 *       (rcu-txn.h), so the guard reads this record's PENDING value -- the clean
 *       word -- masks nothing, and validates {s -> s} against it; record_chain
 *       finds r->new_ptr == old_ptr and chains without upgrading (rcu-txn-mcas.h).
 *       Redundant, not wrong.  (This is the order the converted sites are in, so
 *       skipping their guard is an economy and a clarification, NOT a bug fix.)
 *
 *   guard THEN release  = POISON, permanently.  The guard reads the COMMITTED
 *       word (s|LOCK), masks the lock out, and adds {s -> s}.  The release
 *       then arrives with expected old s|LOCK != r->new_ptr == s, so
 *       record_chain sets t->poisoned: every commit of this txn aborts, and each
 *       retry rebuilds the same poisoned shape.  So NEVER plant a §4.B guard on
 *       a word BEFORE a recompact records its release on it in the same txn.
 *       No site does today (each op arms its txn, then recompacts, then
 *       publishes), and each conversion must keep it that way.
 *
 * Registered in the txn's locks[] registry exactly like a retire, so the two
 * non-commit terminals (ABORT / MEMORY_ERROR in ft_flip_txn_commit, a
 * pre-commit bail in ft_flip_txn_destroy) CAS-clear the lock and leave the node
 * live -- the same word this record would have committed.  The registry itself
 * therefore needs NO knowledge of which terminal an op chose.
 */
static inline
void ft_flip_txn_record_release_lock(struct ft_flip_txn *t,
		struct cds_ft_metadata *meta, uintptr_t state_snapshot)
{
	ft_flip_txn_record_tag(t, (void **) &meta->state,
			(void *) (state_snapshot | FT_STATE_LOCK),
			(void *) state_snapshot,
			FT_STATE_PROXY);
}

/*
 * Invariant-2 VALIDATE side (§4.B / doc/design/step4-concurrent-engine-plan.md
 * §3.3): guard the LIVE parent @parent_nf that a forward edge publishes INTO, by
 * recording a {live->live} freeze guard on its state word.  Once concurrent
 * per-trie writers are enabled, a remover that FROZE @parent_nf (set
 * FT_STATE_TOMBSTONE) between this op's descent and its commit makes the commit
 * ABORT -- so the op never publishes an edge into a node being retired.  The
 * forward pointer-slot CAS alone does NOT catch this: a remover retires the
 * parent by relocating it through the GRANDparent slot and tombstoning it,
 * without ever writing the body slot this op CASes, so the pointer CAS still
 * matches its expected old value.  Only the state guard sees the freeze.
 *
 * Same FT_STATE_PROXY tag the tombstone STORE uses (ft_flip_txn_record_tombstone)
 * so a later same-slot store upgrades the guard in place and an in-flight
 * nr_child proxy resolves.  On the recompact baseline nr_child is invariant per
 * node object, so a plain FULL-WORD validate is exact (no masked-validate).  A
 * NULL @parent_nf == a publish into &ft->root, auto-guarded by the root-slot CAS
 * (concurrent root relocations collide on old == current_root) -> no-op.  Under
 * the retained single-writer exclusion the guard always passes =>
 * behaviour-identical; Phase 4.3 makes it load-bearing.  @t must reserve +1.
 */
static inline
void ft_flip_txn_guard_parent(const struct cds_ft *ft, struct ft_flip_txn *t,
		struct cds_ft_inode_flag *parent_nf)
{
	/*
	 * NULL @t: the forward publish is a lone on-stack edge with no txn to
	 * attach to (a list-off pub-less / non-fused commit) -- deferred to the
	 * force-onto-txn pass.  NULL @parent_nf: a &ft->root publish, auto-guarded
	 * by the root-slot CAS.  Either way there is no live parent to guard here.
	 */
	uintptr_t v, live;

	if (!t || !parent_nf)
		return;
	/*
	 * §4.B VALIDATE as ONE record: read the holder's state (unrecorded)
	 * and expect its LIVE value at commit -- {live -> live}, a pure
	 * validate that writes nothing new.
	 *
	 *  - live holder: identical to the former {v -> v} stability guard; a
	 *    retire DURING the commit window changes the word (the tombstone
	 *    is one-way) and fails the compare.
	 *  - holder retired BEFORE this guard (the already-DEAD case a
	 *    stability-only {DEAD -> DEAD} record would PASS, committing this
	 *    publish into a retired copy = a lost update): the live
	 *    expectation cannot match -> guaranteed ABORT -> a retry-enabled
	 *    op re-derives against the current tree.
	 *
	 * Composes ORDER-FREE with an op that fuses a REAL state edge
	 * (nr_child--) on the same word in the same attempt: a guard never
	 * advances the record's new_ptr (urcu_txn_validate, upgrade-free), the
	 * engine keeps a record's original expected old on upgrade, and a
	 * mismatching expected old between the two poisons the descriptor
	 * (commit aborts) -- so a live holder's guard and decrement fuse into
	 * one record either way, and a dead one still fails.  Dead-at-guard is
	 * unreachable under a single writer.
	 */
	v = (uintptr_t) urcu_txn_load(t->mtxn,
			(void **) &ft_flag_to_metadata(ft, parent_nf)->state,
			FT_STATE_PROXY);
	/*
	 * The clean-LIVE expectation masks BOTH one-way death (tombstone) and
	 * the reversible per-node lock (FT_STATE_LOCK): a holder under a
	 * peer's body copy at validate time mismatches -> ABORT -> retry; a
	 * copier that ABORTED and cleared the fence before our validate
	 * matches -> proceed (the copy was abandoned, the holder unchanged).
	 */
	live = v & ~(FT_STATE_TOMBSTONE | FT_STATE_LOCK);
	urcu_txn_validate(t->mtxn,
			(void **) &ft_flag_to_metadata(ft, parent_nf)->state,
			(void *) live, FT_STATE_PROXY);
}

#ifdef FEATURE_FT_FAULT_INJECT
extern long cds_ft_fault_lock_countdown;
#endif
/*
 * LOCK_FINE (§9.1/§9.2): acquire @parent_nf -- a SURVIVING, VALUE-SWAP publish
 * target that the op's forward edge publishes into -- as a per-node RELEASE lock
 * instead of a §4.B guard.  This is the guard-target half of the lock-set
 * derivation rule ("guard_parent(X) becomes hold X's lock -- same node") for
 * every insert/remove publish whose target's OWN body is not copied under the
 * lock and whose OWN nr_child this op does not change (a value swap): its state
 * word carries only the lock, so the RELEASE terminal {LOCK|s -> s} is clean.
 *
 * On an acquire MISS -- a peer already holds @parent_nf -- fall back to the plain
 * guard.  For a value-swap target the guard is a correct, weaker representative:
 * it aborts at commit iff the peer still holds it, else the publish is safe (the
 * body was never copied under the lock, and the forward record_reserved's
 * expected-old catches a peer that changed the slot).  So a miss reuses the
 * existing guard/abort/re-descend teardown with NO new unwind path -- unlike
 * recompact, which copies C's body under C's lock and so must re-descend on a
 * miss.  The clean all-or-none acquire (no fallback) arrives when the op's
 * FT-wide lock drops; under that lock the miss never happens, so normal
 * operation always takes the release and FEATURE_FT_FAULT_INJECT exercises the
 * fallback.
 *
 * Reservation is net-zero: the release and the guard are each one record on
 * @parent_nf's state word, and the arm already reserved the guard slot.  NULL
 * @t / NULL @parent_nf and non-lock_fine tries route straight to the guard
 * (which no-ops on NULL) -- behaviour-identical.
 */
static inline
void ft_flip_txn_lock_or_guard_parent(const struct cds_ft *ft,
		struct ft_flip_txn *t, struct cds_ft_inode_flag *parent_nf)
{
	if (ft->lock_fine && t && parent_nf) {
		struct cds_ft_metadata *pmeta = ft_flag_to_metadata(ft, parent_nf);
		uintptr_t psnap = 0;
		bool acquired;

#ifdef FEATURE_FT_FAULT_INJECT
		/*
		 * Force the acquire to MISS (as a peer holding @parent_nf would),
		 * exercising the guard fallback the FT-wide lock otherwise makes
		 * unreachable.  Shares cds_ft_fault_lock_countdown with recompact's
		 * ft_lock_member: whichever per-node acquire the countdown
		 * lands on faults, and the op must degrade cleanly either way.
		 */
		if (cds_ft_fault_lock_countdown >= 0) {
			if (cds_ft_fault_lock_countdown == 0) {
				cds_ft_fault_lock_countdown = -1;
				acquired = false;
				goto fault_miss;
			}
			cds_ft_fault_lock_countdown--;
		}
#endif
		acquired = !ft_meta_lock_acquire(pmeta, &psnap);
#ifdef FEATURE_FT_FAULT_INJECT
fault_miss:
#endif
		if (caa_likely(acquired)) {
			ft_flip_txn_record_release_lock(t, pmeta, psnap);
			ft_flip_txn_lock_register(t, pmeta);
			return;
		}
		/*
		 * ALL-OR-NONE.  A miss used to degrade to the plain guard below
		 * and carry on.  That is sound only while every structural edge
		 * installs by CAS: the guard aborts iff the peer STILL holds the
		 * node at commit, so a peer that releases in between leaves us
		 * publishing into a slot we never locked.  Under MW the record's
		 * expected-old still arbitrates that, which is why the fallback
		 * was safe; under the SW cutover the park cannot fail, and the
		 * same window becomes a LOST UPDATE.
		 *
		 * So record the miss and let the commit ABORT.  Deliberately not
		 * a spin: waiting for the holder's release while occupying a
		 * lane turn is the circular wait that deadlocked step 2 (see the
		 * escalation-lane note at ft_writer_lock_scope_enter).  The
		 * guard is still planted -- harmless, and it keeps the record
		 * shape identical between the hit and miss paths.
		 */
		t->acquire_miss = true;
	}
	ft_flip_txn_guard_parent(ft, t, parent_nf);
}

/*
 * Holder-lock variant of ft_flip_txn_lock_or_guard_parent (MW LOCK_FINE Step A):
 * when the op ALREADY holds @parent_nf's node lock -- acquired before the
 * chain read, @held_snap the clean pre-mark word -- record the {LOCK|s -> s}
 * RELEASE + register it instead of re-marking.  The release EXPECTS the held
 * LOCK (so it fuses with any same-word nr_child-- and clears the fence at
 * commit), where the masking guard-fallback ft_flip_txn_lock_or_guard_parent
 * would take on a re-mark MISS validates the CLEAN word and thus ABORTS on the
 * op's OWN still-set fence -- a self-livelock.  Registering hands the fence
 * outcome to the txn: commit consumes it, an aborted/destroyed commit
 * auto-clears it.  @held_holder NULL routes to the ordinary acquire-or-guard
 * (unheld op / non-lock_fine).
 */
static inline
void ft_flip_txn_hold_or_lock_parent(const struct cds_ft *ft,
		struct ft_flip_txn *t, struct cds_ft_inode_flag *parent_nf,
		struct cds_ft_metadata *held_holder, uintptr_t held_snap)
{
	if (held_holder) {
		/*
		 * The held arm takes the caller's word for it: it records the
		 * release against @held_holder and NEVER derives it from
		 * @parent_nf, so a caller that pairs a stale holder with a fresh
		 * publish parent releases a fence on one node while publishing
		 * into another -- both wrong, and silently so.  Identity is a
		 * caller-construction property today (every setter assigns the
		 * holder and the parent in the same breath); assert it so it stays
		 * one.
		 */
		assert(parent_nf && held_holder == ft_flag_to_metadata(ft, parent_nf));
		ft_flip_txn_record_release_lock(t, held_holder, held_snap);
		ft_flip_txn_lock_register(t, held_holder);
		return;
	}
	ft_flip_txn_lock_or_guard_parent(ft, t, parent_nf);
}

/*
 * Set a duplicate-chain node's removal tombstone (CDS_FT_NODE_REMOVED_FLAG on
 * cds_ft_node.next) as a COMMITTED flip edge, at the point @node is unlinked
 * from the trie.  This is the chain-leaf analogue of ft_meta_tombstone_set_flip
 * (the internal-node §4.B state mark): the freed node's OWN next is the word a
 * concurrent duplicate-append CASes (a tail append is tail->next: NULL -> D), so
 * recording the tombstone as a {slot, old, new} edge is what makes that append's
 * expected-value CAS fail once the tail is dead -- instead of a bare in-place
 * bit-set sitting outside the descriptor protocol (doc/design/mcas-multiwriter-
 * readiness.md §4 DECISION FINAL, refinement-1 site 2).
 *
 * The successor pointer is preserved (only the mark bit is set), so a
 * concurrent reader positioned on @node still follows the chain (readers mask
 * the bit in cds_ft_node_next_rcu).  Under one writer the commit is one
 * uncontended CAS of a low bit readers ignore -- behaviour-identical; under
 * multi-writer the latch-honoring loop (F3, mirror of
 * ft_meta_state_transition) waits out a parked FT_HLIST_TAG proxy (engine
 * bit 0; the mark is bit 1, so the two never alias) instead of OR-ing the
 * mark into a latch POINTER -- the corrupted-record trample the raw
 * read + blind flip allowed -- and re-derives from the fresh successor on a
 * CAS miss so a peer's committed relink is never overwritten stale.
 * Infallible; idempotent (re-marking is a same-value CAS).  The wait is
 * bounded by the latch owner's settle (owner-only; a writer dying mid-install
 * would wedge it, but writer death mid-mutation is already fatal to the trie
 * by contract).  The mark and the predecessor relink that unlinks @node
 * should eventually ride ONE flip (atomic detach); a standalone mark is the
 * bridge.
 *
 * RETURNS the clean successor the mark was CASed over (mark bit stripped, no
 * proxy -- the loop validated bit 0 clear): the ONE proxy-safe way for a
 * chain sweep to advance, since a raw ft_node_next masks only the mark bit
 * and would hand a parked latch pointer to the next iteration.
 */
static
struct cds_ft_node *ft_node_mark_removed_flip(struct cds_ft *ft,
		struct cds_ft_node *node)
{
	(void) ft;
	for (;;) {
		struct cds_ft_node *old = CMM_LOAD_SHARED(node->next);

		if (caa_unlikely((uintptr_t) old & FT_HLIST_TAG)) {
			caa_cpu_relax();
			continue;
		}
		if (caa_likely(uatomic_cmpxchg(&node->next, old,
				(struct cds_ft_node *) ((uintptr_t) old |
					CDS_FT_NODE_REMOVED_FLAG)) == old)) {
			FT_TP(edge_lone, (const void *) &node->next,
				(const void *) old,
				(const void *) ((uintptr_t) old |
					CDS_FT_NODE_REMOVED_FLAG));
			return (struct cds_ft_node *) ((uintptr_t) old &
					~(uintptr_t) CDS_FT_NODE_REMOVED_FLAG);
		}
	}
}

/*
 * Tombstone every node in a duplicate chain (cds_ft_remove_all detaches a whole
 * chain at once), each via ft_node_mark_removed_flip so the marks are committed
 * edges.  Successor pointers stay intact so the caller can still traverse the
 * returned chain to reclaim it.  The sweep advances on the successor the mark
 * VALIDATED (proxy-free), not a raw ft_node_next re-read that masks only the
 * mark bit -- a peer's latch parked on an interior next would otherwise walk
 * the sweep into descriptor memory.
 */
static
void ft_chain_mark_removed_flip(struct cds_ft *ft, struct cds_ft_node *head)
{
	while (head)
		head = ft_node_mark_removed_flip(ft, head);
}

/*
 * Publish an in-place node child-slot replacement (@slot transitions @old ->
 * @new) as a single-edge flip descriptor.  This is the non-fused (pub == NULL)
 * arm of ft_node_replace_ptr -- a key-disappearing leaf delete (@new == NULL) or
 * an external promote (@new == the chain head) whose structural commit does NOT
 * fuse with an ordered-list cell unsplice (the fused, pub-armed path rides
 * ft_remove_one_commit's flip instead; this arm is reached for non-head removals
 * and every ordered-list-OFF delete).  A lone edge commits as one release store
 * -- byte-identical to a bare rcu_assign_pointer -- captured as a {slot, old,
 * new} descriptor so a future multi-writer MCAS covers the child slot uniformly
 * (a bare store would discard @old and sit outside the descriptor protocol, yet
 * the slot can be in a concurrent writer's word-set).
 */
static
void ft_node_child_edge_flip(struct cds_ft *ft,
		struct cds_ft_inode_flag **slot,
		struct cds_ft_inode_flag *old,
		struct cds_ft_inode_flag *new)
{
	struct ft_ord_cell_edge edge = {
		.slot = (struct ft_ord_cell **) slot,
		.old_target = (struct ft_ord_cell *) old,
		.new_target = (struct ft_ord_cell *) new,
	};

	(void) ft;	/* a lone edge commits on an on-stack txn (no reclaim) */
	ft_ord_cell_flip_one(&edge);
}

/*
 * One side of a two-trie root swap: a root slot transition plus (when the group
 * runs an ordered list) the trie's head/tail endpoint transfer.  The head/tail
 * fields are ignored when the group's ordered list is off.
 */
struct ft_root_swap_side {
	struct cds_ft *ft;
	struct cds_ft_inode_flag **slot;
	struct cds_ft_inode_flag *old_root, *new_root;
	struct ft_ord_cell *head_old, *head_new;
	struct ft_ord_cell *tail_old, *tail_new;
};

/*
 * Whole-trie root swap across TWO tries fused in ONE flip.  The empty-dst
 * root-level graft (dst appears / src retires to a fresh empty root) and the
 * whole-trie graft_swap (dst <-> swap exchange) both publish two root slots --
 * historically as two separate ft_root_list_swap_publish flips, leaving a
 * cross-trie window where a reader sees a key reachable in BOTH tries (appear
 * committed, disappear not yet) or in NEITHER.  Recording both sides' root (and,
 * list-on, head/tail) edges in ONE flip closes that window: the two
 * tries share a group, hence a flip selector, so a single epoch flip settles all
 * <= 10 edges atomically -- a reader resolves every root/endpoint proxy to ONE
 * phase and sees the key in exactly one trie.  All @old values must be captured
 * by the caller before the call (the two sides reference each other's pre-swap
 * roots/endpoints).
 *
 * Sentinel topology: unlike the single-side moves (detach / graft run-splice),
 * the dual has NO drain between detach and attach -- it fuses both into ONE flip
 * -- so it must NOT relink a moved run's outer links to the OTHER (foreign) trie's
 * sentinel: a source straddler resolving that flipped link (global selector ->
 * new) would land on a foreign sentinel and dereference it as a cell.  Instead
 * each side NULL-TERMINATES its OUTGOING run's outer links in the flip (NULL is
 * the universal end -- safe for a source straddler mid-iteration AND for a fresh
 * reader on the receiving side).  Only the head/tail sentinel endpoints are
 * relinked across the boundary (ft_ord_sentinel_edges, @relink_dest = NULL =
 * head/tail only).  The caller then drains (synchronize_rcu, retiring straddlers)
 * and calls ft_ord_finalize_circular() on each receiving side to repoint the run
 * at its new sentinel, restoring the circular invariant the remove folding needs.
 */
#define FT_ROOT_LIST_SWAP_DUAL_MAX_EDGES	10	/* 2 roots + 2x (2 sentinel + 2 null-term) */
static
void ft_root_list_swap_publish_dual(struct ft_flip_txn *txn,
		const struct ft_root_swap_side *a,
		const struct ft_root_swap_side *b)
{
	const struct ft_root_swap_side *sides[2] = { a, b };
	struct ft_ord_cell_edge edges[FT_ROOT_LIST_SWAP_DUAL_MAX_EDGES] = { 0 };
	unsigned int s, n = 0;

	for (s = 0; s < 2; s++) {
		const struct ft_root_swap_side *r = sides[s];

		/*
		 * Each side's new root changes trie, so it names its NEW owner.
		 * Done here, at the one helper both whole-trie swaps go through
		 * (empty-dst root graft and whole-trie graft_swap), so neither
		 * caller can forget.  Invisible to readers and mutators: they all
		 * read a parent through ft_parent_node, which answers NULL for
		 * either trie pointer, so an up-walk still just stops at the
		 * root.  Only cds_ft_verify reads the identity.
		 */
		if (r->new_root && !ft_node_flip_proxy(r->new_root))
			cds_ft_item_to_metadata(ft_node_ptr(r->new_root))
				->parent_word = ft_trie_parent(r->ft);
		edges[n].slot = (struct ft_ord_cell **) r->slot;
		edges[n].old_target = (struct ft_ord_cell *) r->old_root;
		edges[n].new_target = (struct ft_ord_cell *) r->new_root;
		n++;
		if (!r->ft->group->ordered_list_set)
			continue;
		/* Head/tail endpoints only (no foreign outer relink). */
		n = ft_ord_sentinel_edges(r->ft, r->head_old, r->head_new,
				r->tail_old, r->tail_new, NULL, false,
				edges, n);
		/*
		 * NULL-terminate this side's OUTGOING run [head_old..tail_old]
		 * (the whole list -- the dual only moves whole tries -- so its
		 * outer links currently point at r->ft's own sentinel).  The
		 * receiving side's finalize re-homes them to the new sentinel
		 * after the drain.
		 */
		if (r->head_old) {
			struct ft_ord_cell *self = ft_ord_sentinel_cell(r->ft);

			edges[n].tag = URCU_TXN_TAG;	/* ordered-cell edge */
			edges[n].slot = (struct ft_ord_cell **)
				&r->head_old->lnode.prev;
			edges[n].old_target = self;
			edges[n].new_target = NULL;
			n++;
			edges[n].tag = URCU_TXN_TAG;	/* ordered-cell edge */
			edges[n].slot = (struct ft_ord_cell **)
				&r->tail_old->lnode.next;
			edges[n].old_target = self;
			edges[n].new_target = NULL;
			n++;
		}
	}
	/*
	 * @txn is the caller-PRE-RESERVED bounded txn (both graft/graft_swap
	 * cross-trie callers reserve it), committed infallibly.  Both roots
	 * always flip, so this commit is always multi-edge (>= 2) even with the
	 * ordered list off -- it can never reduce to a lone store.
	 */
	/* Bulk op, not yet MW-hardened: ABORT unreachable under its exclusion. */
	(void) ft_ord_cell_flip_into(a->ft, txn, edges, n);
}

/*
 * Structural min/max dup-chain HEAD of the subtree rooted at @nf, under WRITER
 * EXCLUSION (no concurrent mutation -> no skip re-anchor / flip-proxy / transient
 * empty states to handle, unlike the reader-side minmax descent).  Mirrors the
 * key ordering the ordered cell list uses: a key that ends at an internal node
 * (metadata->external_nodes, a prefix key) sorts BEFORE every longer key under
 * it, so it is the subtree minimum.  Used to locate the endpoints of the
 * contiguous ordered-list run a bulk op relocates.
 */
static
struct cds_ft_node *ft_subtree_minmax_head(struct cds_ft *ft, struct cds_ft_inode_flag *nf,
		bool want_max)
{
	enum ft_direction dir = want_max ? FT_RIGHTMOST : FT_LEFTMOST;
	uint8_t scratch;

	for (;;) {
		nf = ft_resolve_skip_compressed(ft, nf);
		if (ft_node_external(nf))
			return (struct cds_ft_node *) ft_node_ptr(nf);
		if (ft_node_compressed(nf)) {
			struct cds_ft_compressed_node *cn =
				ft_compressed_node_ptr(nf);

			nf = rcu_dereference(cn->child);
			continue;
		}
		/* Internal node. */
		{
			struct cds_ft_metadata *m =
				cds_ft_item_to_metadata(ft_node_ptr(nf));
			struct cds_ft_node *ext =
				ft_dereference_external(m->external_nodes);
			struct cds_ft_inode_flag *child;

			if (!want_max && ext)
				return ext;	/* prefix key: subtree minimum */
			child = ft_node_get_minmax(ft, nf, &scratch, dir, false);
			if (!child) {
				/*
				 * No children: a NIL-key-only internal whose
				 * external_nodes is the sole key, so it is also the
				 * subtree MAXIMUM (a single-prefix-key trie root, e.g.
				 * a detached external).  want_min returned it above.
				 */
				assert(ext != NULL);
				return ext;
			}
			nf = child;
		}
	}
}

/*
 * Move the contiguous ordered-list run whose endpoints are the cells of
 * @first_head .. @last_head (heads, in key order) OUT of @ft's ordered cell
 * list and install it as the ENTIRE ordered list of @into -- the cds_ft_detach
 * shape, where @into is a fresh EXCLUSIVE trie receiving exactly that subtree.
 * The run's internal ord links are preserved; only its two boundary edges in
 * @ft are flipped (atomic for a concurrent ordered reader, per the flip-latch),
 * @ft's head/tail are repaired, and @into's head/tail are set.  @into being
 * exclusive, clearing the run's new boundary links is a plain store.  Caller
 * gates on ordered_list_set.
 */
/*
 * Append the <=4 boundary edges that excise the contiguous run @first_head ..
 * @last_head from @ft's ordered cell list (the two neighbour back-edges plus any
 * head/tail repair); the run's internal links are preserved for re-homing.
 * Stashes the run-endpoint cells in *@first_out / *@last_out for the caller's
 * post-flip @into install.  Split out so a bulk detach can fuse these edges with
 * its structural subtree unlink in ONE flip (ft_remove_one_commit / _rec with a
 * struct ft_detach_run), exactly as ft_ord_cell_unsplice_edges does for a point
 * remove; ft_ord_cell_run_detach is the standalone (two-commit) wrapper.
 */
static
unsigned int ft_ord_cell_run_detach_edges(struct cds_ft *ft,
		struct cds_ft_node *first_head, struct cds_ft_node *last_head,
		struct ft_ord_cell **first_out, struct ft_ord_cell **last_out,
		struct ft_ord_cell_edge *edges, unsigned int n)
{
	struct ft_ord_cell *first =
		ft_ord_cell_ptr(rcu_dereference(first_head->prev));
	struct ft_ord_cell *last =
		ft_ord_cell_ptr(rcu_dereference(last_head->prev));
	struct ft_ord_cell *pred = ft_ord_cell_resolve_ord(&first->lnode.prev);
	struct ft_ord_cell *succ = ft_ord_cell_resolve_ord(&last->lnode.next);

	(void) ft;
	/*
	 * Sentinel topology: @pred / @succ are the run's @ft-side neighbours, which
	 * resolve to @ft's sentinel pseudo-cell when the run sits at @ft's head /
	 * tail -- so recording &pred->lnode.next / &succ->lnode.prev IS the old
	 * head/tail repair when @pred / @succ is the sentinel (no separate endpoint
	 * edge).  The run's OUTER boundary links (first->prev, last->next) are
	 * re-homed to @into by ft_ord_cell_run_install after the flip (@into is
	 * exclusive: plain stores).
	 */
	edges[n].tag = URCU_TXN_TAG;	/* ordered-cell edge */
	edges[n].slot = (struct ft_ord_cell **) &pred->lnode.next;
	edges[n].old_target = first;
	edges[n].new_target = succ;
	n++;
	edges[n].tag = URCU_TXN_TAG;	/* ordered-cell edge */
	edges[n].slot = (struct ft_ord_cell **) &succ->lnode.prev;
	edges[n].old_target = last;
	edges[n].new_target = pred;
	n++;
	*first_out = first;
	*last_out = last;
	return n;
}

/*
 * Install the excised run (its endpoint cells, captured by
 * ft_ord_cell_run_detach_edges) as the ENTIRE ordered list of the exclusive
 * @into trie.  @into has no readers, so plain stores: point its sentinel at the
 * run.  The run's OUTER links (first->prev, last->next) are LEFT pointing at
 * @ft's former neighbours -- which a SOURCE-trie reader straddling the move (still
 * parked on a run cell, walking that outer link) recognises (an @ft cell or @ft's
 * own sentinel), so it stops / re-enters @ft instead of dereferencing @into's
 * (foreign, to that reader) sentinel as a cell.  The detach finalizes @into's
 * outer links to @into's sentinel AFTER its drain (ft_ord_finalize_circular),
 * once no straddling source reader remains, restoring the in-trie invariant the
 * remove folding / reverse walk need.  MUST run after the flip that excised the
 * run from @ft.
 */
static
void ft_ord_cell_run_install(struct cds_ft *into, struct ft_ord_cell *first,
		struct ft_ord_cell *last)
{
	into->ord_sentinel.node.next = ft_ord_cell_lnode(first);
	into->ord_sentinel.node.prev = ft_ord_cell_lnode(last);
}

/*
 * Make @ft's ordinal-cell list properly circular: point its run's outer boundary
 * links at the sentinel (first->prev and last->next).  A bulk move into @ft
 * leaves those outer links pointing at a UNIVERSAL terminator -- the SOURCE
 * trie's sentinel (detach into an exclusive @into) or NULL (the cross-trie dual
 * swaps, where the receiving side is itself live) -- straddler-safe in either
 * case (every reader treats its own sentinel or NULL as the end).  This runs
 * AFTER the move's drain, once no straddling source reader remains, to restore
 * the in-trie invariant (first->prev == sentinel) the remove folding and reverse
 * walk rely on.  A no-op on an empty list.
 *
 * @ft may be LIVE here (the empty-dst graft / whole-trie swap finalize a trie
 * that already carries concurrent readers of the just-moved run), so the stores
 * are rcu_assign_pointer, not plain: a receiving-side reader racing the finalize
 * resolves the outer link to either the old terminator (NULL / its own sentinel)
 * or the new sentinel -- both a valid end for THAT trie's reader, so the race is
 * benign, but the store must still be atomic to pair with the reader's
 * rcu_dereference.  (For detach's exclusive @into the release barrier is a
 * harmless extra.)
 */
static
void ft_ord_finalize_circular(struct cds_ft *ft)
{
	struct ft_ord_cell *first = ft_ord_first(ft);
	struct ft_ord_cell *last = ft_ord_last(ft);

	if (first) {
		rcu_assign_pointer(first->lnode.prev, &ft->ord_sentinel.node);
		rcu_assign_pointer(last->lnode.next, &ft->ord_sentinel.node);
	}
}

/* Max edges a run-detach commits: the run's two outer back-edges. */
#define FT_ORD_CELL_RUN_DETACH_MAX_EDGES	2

/*
 * Excise the run [@first_head .. @last_head] from @ft's ordered list and install
 * it as the exclusive @into trie's whole list.  Standalone (two-commit)
 * fallback for the rare detach shape that could not fuse the run into its
 * structural flip; run AFTER that structural unlink is public, so un-abortable
 * -- commit through the caller-PRE-RESERVED txn @txn (ft_ord_cell_flip_into).
 */
static
void ft_ord_cell_run_detach(struct cds_ft *ft, struct ft_flip_txn *txn,
		struct cds_ft *into, struct cds_ft_node *first_head,
		struct cds_ft_node *last_head)
{
	struct ft_ord_cell_edge edges[FT_ORD_CELL_RUN_DETACH_MAX_EDGES] = { 0 };
	struct ft_ord_cell *first, *last;
	unsigned int n = ft_ord_cell_run_detach_edges(ft, first_head, last_head,
		&first, &last, edges, 0);

	/* Bulk op, not yet MW-hardened: ABORT unreachable under its exclusion. */
	(void) ft_ord_cell_flip_into(ft, txn, edges, n);
	ft_ord_cell_run_install(into, first, last);
}

/*
 * Cell-side of a key-disappearing structural unlink, fused into ONE flip.  A
 * point remove unsplices a single dead head cell (@cell); a bulk detach excises
 * a contiguous RUN (@rfirst .. @rlast heads) out of @ft and re-homes it as the
 * exclusive @into trie's whole list.  Exactly one of @cell / @into is set (the
 * other zero); both zero means "structural edges only" (ordered list off).
 * @armed is set once the fused commit runs (so a bulk caller knows the run was
 * fused, not left for the standalone two-commit fallback).
 */
struct ft_detach_run {
	struct cds_ft *into;			/* run re-home target (exclusive), or
						 * NULL = EXCISE-ONLY: unlink the run
						 * from @ft's list without re-homing it
						 * (the merge source side, where the
						 * cells disperse into dst) */
	struct cds_ft_node *rfirst, *rlast;	/* run endpoint heads, in key order */
	struct ft_ord_cell *first, *last;	/* scratch: filled at flip time */
	bool armed;
};

/*
 * Commit @n edges through the caller-PRE-RESERVED txn @t (already sized for at
 * least @n via ft_flip_txn_create_bounded in the op's fallible build phase):
 * record every edge (freeze-before-install -- record_reserved cannot fail),
 * commit (park each proxy, flip the group, settle to the new target -- a lone
 * edge reduces to a single release store with no proxy / no grace period), then
 * reclaim the txn (deferred-freed when a proxy is owed).  OOM-infallible -- no
 * allocation, hence no bare-store fallback.  @t is consumed (do not reuse / free
 * it).  This is the "pre-reserve always" commit: an op reserves its txn where
 * failure is clean (the build prefix, before any reader-visible change) and
 * commits through it here where ALLOCATION failure is impossible.
 *
 * RETURNS the commit status: under concurrent writers the engine commit may
 * ABORT (a peer won an expected-value CAS / froze a guarded node) -- then
 * NOTHING was installed and the txn's recorded edges (freezes, counts,
 * tombstones) were all discarded together.  A retry-enabled op (cds_ft_remove,
 * cds_ft_insert one-commit) MUST propagate ABORT so its retry loop re-descends;
 * an op not yet MW-hardened void-casts the return with a comment (ABORT is
 * unreachable under its current exclusion).
 */
static
enum urcu_txn_status ft_ord_cell_flip_into(struct cds_ft *ft,
		struct ft_flip_txn *t,
		struct ft_ord_cell_edge *edges, unsigned int n)
{
	unsigned int i;

	for (i = 0; i < n; i++)
		ft_flip_txn_record_tag(t, (void **) edges[i].slot,
			(void *) edges[i].old_target,
			(void *) edges[i].new_target,
			ft_edge_tag(&edges[i]));
	return ft_flip_txn_commit(ft, t);
}

/*
 * RECORD-ONLY sibling of ft_ord_cell_flip_into: append @n heterogeneous edges to
 * the caller's SHARED mixed txn @t WITHOUT committing, so the caller runs the ONE
 * ft_flip_txn_commit that also carries the other side of a fold (the coherent
 * rekey one-decide writer records the src-unlink -- through here -- and the
 * dst-attach + S_top COW into the same @t, then commits once).
 *
 * Per-edge SW/MW by tag: a STRUCTURAL trie edge (tag unset -> ft_edge_tag ==
 * FT_FLIP_PROXY_TAG) goes through ft_flip_txn_record_tag, so it parks SW when @t
 * opted into structural_sw; a CELL / hlist edge (URCU_TXN_TAG) goes through
 * record_tag_mw (ALWAYS MW -- the ordered list stays lock-free, a cell conflict
 * aborts the mixed commit clean).  The src-unlink these edges express is the
 * default-build detach, which RECOMPACTS the src junction: the recompaction holds
 * the rebuilt node's node lock AND (via the fold's parent_held coordination)
 * the shared spine parent it republishes into, so the structural forward-publish /
 * re-parent edges are legitimately SW.  (Byte-identical to ft_ord_cell_flip_into's
 * record loop when structural_sw is false -- record_tag == record_tag_mw -- so the
 * FEATURE_FT_INSERT_IN_PLACE single-writer in-place delete, which is unlocked but
 * has no concurrent peer, is equally fine either way.)  @t must be pre-reserved
 * for >= @n edges (record cannot fail).  No commit here => the run-arm
 * (ft_ord_cell_run_install) a self-committing flip does inline must be deferred by
 * the caller to its post-commit finalize.
 */
static
void ft_ord_cell_record_into(struct ft_flip_txn *t,
		const struct ft_ord_cell_edge *edges, unsigned int n)
{
	unsigned int i;

	for (i = 0; i < n; i++) {
		uintptr_t tag = ft_edge_tag(&edges[i]);

		if (tag == FT_FLIP_PROXY_TAG)
			ft_flip_txn_record_tag(t, (void **) edges[i].slot,
				(void *) edges[i].old_target,
				(void *) edges[i].new_target, tag);
		else
			ft_flip_txn_record_tag_mw(t, (void **) edges[i].slot,
				(void *) edges[i].old_target,
				(void *) edges[i].new_target, tag);
	}
}

/*
 * Fallible self-allocating flip: returns 0, -ENOMEM with NOTHING installed, or
 * -EAGAIN with NOTHING installed (concurrent-writer commit ABORT -- the caller's
 * retry loop re-descends).  For an ABORTABLE caller -- the flip is the op's
 * commit / abort boundary, so on OOM the op returns CDS_FT_STATUS_MEMORY_ERROR
 * with the structure untouched.  A single check, no per-edge handling: the edge
 * count @n is known, so it is one bounded malloc whose failure is reported here
 * (freeze-before-install means an un-built txn installs nothing).  A lone edge
 * takes the on-stack single-store path and always returns 0 -- it cannot ABORT
 * (no engine txn), which also means it cannot DETECT a peer conflict: the
 * lone-store paths are the documented not-yet-MW residue (Group E/F force-a-txn).
 */
static
int ft_ord_cell_flip_try(struct cds_ft *ft, struct ft_ord_cell_edge *edges,
		unsigned int n)
{
	struct ft_flip_txn *t;

	if (n == 0)
		return 0;
	if (n == 1) {
		/* Lone edge: the infallible on-stack single-store commit. */
		ft_ord_cell_flip_one(&edges[0]);
		return 0;
	}
	t = ft_flip_txn_create_bounded(n);
	if (caa_unlikely(!t))
		return -ENOMEM;
	return ft_ord_cell_flip_into(ft, t, edges, n) > 0 ? -EAGAIN : 0;
}

/*
 * Per-op ON-STACK scratch iterator for the writer-side splice-position
 * searches.  These used to run on a single per-trie scratch iterator
 * (ft->ord_cell_scratch_iter, "writers are serialized") -- under Phase-4.3
 * multi-writer, concurrent inserts' pred searches clobbered each other's
 * iter_key/cursor MID-DESCENT, producing internally-consistent but key-order
 * wrong pred/succ brackets (the ord-cell mis-order family; trace-proven: a
 * search's query bytes mutated to a peer's key between its compressed-node
 * compare and its going-up walk).  An on-stack iterator is private by
 * construction.  Layout mirrors cds_ft_iter_create: header + ordinal-key
 * buffer + FT_KEY_READABLE_PAD tail so the descent's wide loads stay in
 * bounds.  Header-only zeroing: set_key fills the key bytes and lengths.
 */
struct ft_stack_iter {
	struct cds_ft_iter it;
	uint8_t key_buf[FT_MAX_KEY_LEN + FT_KEY_READABLE_PAD];
};

static inline
struct cds_ft_iter *ft_stack_iter_init(struct ft_stack_iter *si,
		struct cds_ft *ft)
{
	memset(&si->it, 0, sizeof(si->it));
	si->it.ft = ft;
	si->it.cache_mode = CDS_FT_ITER_CACHED;
	return &si->it;
}

/*
 * Find the cell of the in-order predecessor (mode LT) / successor (mode GT)
 * of @key via the eager relational descent on a private on-stack iterator.
 * Returns NULL when none exists (@key is the new minimum/maximum).
 */
static
struct ft_ord_cell *ft_ord_cell_find_rel(struct cds_ft *ft, const uint8_t *key,
		size_t key_len, enum ft_lookup_inequality mode,
		struct ft_visit_witness *wit, struct cds_ft_node **head_out)
{
	struct ft_stack_iter si;
	struct cds_ft_iter *it = ft_stack_iter_init(&si, ft);
	struct cds_ft_node *head;

	if (head_out)
		*head_out = NULL;
	if (cds_ft_iter_set_key(it, key, key_len) != CDS_FT_STATUS_OK)
		return NULL;
	it->prefix_len = 0;
	it->node = NULL;
	if (cds_ft_lookup_inequality_impl(ft, it, mode, FT_LOOKUP_LIMIT_NONE,
			false, false, wit) != CDS_FT_STATUS_OK)
		return NULL;
	head = cds_ft_iter_node(it);
	if (!head)
		return NULL;
	/*
	 * @head_out hands the descent's ANSWER NODE back alongside the cell it
	 * maps to.  Only the FT_DEBUG_SPLICE_POS_BRACKET probe wants it, and it
	 * wants it to tell two failure mechanisms apart: a descent that returned
	 * the WRONG HEAD (then cell->node == head, mapping consistent) from a
	 * RIGHT head mapped to the wrong cell through a stale head->prev (then
	 * cell->node != head).  Every production caller passes NULL.
	 */
	if (head_out)
		*head_out = head;
	return ft_ord_cell_ptr(rcu_dereference(head->prev));
}

/*
 * Find the cell of the in-order predecessor of the freshly-inserted head
 * carried by @cell, WITHOUT re-descending from the root.  The insert just
 * walked root->leaf to attach the head, so its deepest node is the going-up
 * seed: position the writer's scratch iterator AT the new head (a live cursor)
 * and re-enter the relational lookup, which recovers the deepest node from
 * iter->node via the parent chain (its cross-call fast path) and runs the SAME
 * structural backtrack the key-based descent would -- but starting at the
 * divergence point instead of the root.  @seed_from_node suppresses the
 * ordinal-cell fast path (the new head's cell is not yet spliced).
 *
 * @key / @key_len are the head's APPLICATION-form key (set_key remaps): the
 * search key is written straight into iter_key (a memcpy, no up-walk), and the
 * cursor fields are seeded on top so read_key returns that buffer.
 *
 * Returns the predecessor cell, or NULL when the head's key is the new minimum.
 * A compressed/skip-compressed holder makes the impl fall back to a root
 * re-descent internally (correctness preserved, no descent saved for that key).
 */
/*
 * Returns 0 with *@pred_ret = the predecessor cell (NULL = the key is the
 * new minimum), or -EAGAIN on an INTERNAL failure (iterator setup / lookup
 * machinery error).  The two must NOT be conflated in a NULL return: an
 * internal failure reading as "new minimum" splices the cell at the list head
 * (the sentinel-splice mis-order class).
 */
static
int ft_ord_cell_find_pred_from_head(struct cds_ft *ft,
		const uint8_t *key, size_t key_len, struct ft_ord_cell *cell,
		bool from_root, struct ft_ord_cell **pred_ret)
{
	struct ft_stack_iter si;
	struct cds_ft_iter *it = ft_stack_iter_init(&si, ft);
	struct cds_ft_node *pred_head;
	enum cds_ft_status s;

	/*
	 * Write the search key into iter_key (set_key clears cache_valid/node and
	 * sets key_len + key_off=0, path_len=0).
	 *
	 * @from_root false (the common live-structure splice): seed a live cursor
	 * AT the new head on top: cache_valid + node + path_len==key_depth drive
	 * the cross-call node-recovery fast path; prefix 0 = unscoped;
	 * ord_cell_node cleared so a fall-back cell cursor re-resolves from
	 * node->prev.
	 *
	 * @from_root true (split-compressed one-commit, the fresh cluster is
	 * parked): the new head sits in an UNPUBLISHED cluster whose live old
	 * child (cn->child) is re-parented only at the commit, so a from-head LT
	 * would reanchor down through that not-yet-wired edge and mis-navigate.
	 * Descend from the root instead -- the parked forward proxy resolves to
	 * the OLD structure, where every existing key (hence the predecessor) is
	 * reachable and consistent.  Costs one extra descent, on the split path
	 * only.
	 */
	*pred_ret = NULL;
	it->node = NULL;
	if (cds_ft_iter_set_key(it, key, key_len) != CDS_FT_STATUS_OK)
		return -EAGAIN;	/* internal failure, NOT "new minimum" */
	if (!from_root) {
		it->node = cell->node;
		it->cache_valid = true;
		it->ord_cell_node = NULL;
		it->prefix_len = 0;
		it->path_len = it->key_len + 1;
	}
	s = cds_ft_lookup_inequality_impl(ft, it, FT_LOOKUP_LT,
			FT_LOOKUP_LIMIT_NONE, false, !from_root, NULL);
	/*
	 * Genuine "no predecessor" arrives as NOT_FOUND with no landed node
	 * (every ft_ineq_descend terminal pairs status with node: OK iff a
	 * node landed).  Anything else -- a real error, or the
	 * impossible-by-code OK-with-no-node -- is a lookup-machinery
	 * inconsistency and MUST NOT read as "new minimum".
	 */
	if (s == CDS_FT_STATUS_NOT_FOUND)
		return 0;	/* new minimum: *pred_ret stays NULL */
	if (s != CDS_FT_STATUS_OK)
		return -EAGAIN;
	pred_head = cds_ft_iter_node(it);
	if (!pred_head)
		return -EAGAIN;	/* OK without a node: defensive, never a pred claim */
	*pred_ret = ft_ord_cell_ptr(rcu_dereference(pred_head->prev));
	return 0;
}

/*
 * Append @cell's ordered-list unsplice edges (its two neighbour back-edges) to
 * @edges, returning the new count.  @cell keeps its own links for parked readers
 * until its deferred free.  Split out so a key-disappearing remove can fuse these
 * edges with its structural unlink in a single flip (ft_remove_one_commit);
 * ft_ord_cell_unsplice is the standalone (two-commit) wrapper.
 *
 * The three edges this records -- pred->next: cell -> succ, succ->prev: cell ->
 * pred, and the DELETION MARK cell->next: succ -> succ|URCU_TXN_LIST_MARK --
 * are EXACTLY urcu_txn_list_del_prepare's edge set on @cell->lnode; the public
 * op is used directly where the cell records straight into a held txn (the
 * point ops), while the fused removes build into this edge array to commit the
 * cell unsplice and the structural unlink in one flip.  THE MARK IS
 * LOAD-BEARING under concurrent writers: it is the ONLY thing
 * urcu_txn_list_insert_after_prepare's deleted-pos (-ENOENT) and
 * deleted-neighbour (-EAGAIN) checks can see -- an unmarked dead cell keeps
 * bit-identical links, so a peer's splice whose position search captured this
 * cell as pred/succ before the unsplice would otherwise commit INTO the dead
 * segment whenever the neighbour values still match (arena recycling makes the
 * match likely), producing an out-of-order or resurrected cell.  Readers
 * mask the bit (urcu_txn_list_next_rcu strips URCU_TXN_TAG |
 * URCU_TXN_LIST_MARK), so a parked reader still walks off the dead cell
 * exactly as before.  Sentinel topology: @pred / @succ resolve to @ft's
 * sentinel pseudo-cell when @cell is the list first / last, so
 * &pred->lnode.next / &succ->lnode.prev IS the old head / tail endpoint flip
 * -- no separate endpoint edge.
 */
static
unsigned int ft_ord_cell_unsplice_edges(struct cds_ft *ft,
		struct ft_ord_cell *cell, struct ft_ord_cell_edge *edges,
		unsigned int n)
{
	struct ft_ord_cell *pred = ft_ord_cell_resolve_ord(&cell->lnode.prev);
	struct ft_ord_cell *succ = ft_ord_cell_resolve_ord(&cell->lnode.next);

	(void) ft;
	edges[n].tag = URCU_TXN_TAG;	/* ordered-cell edge */
	edges[n].slot = (struct ft_ord_cell **) &pred->lnode.next;
	edges[n].old_target = cell;
	edges[n].new_target = succ;
	n++;
	edges[n].tag = URCU_TXN_TAG;	/* ordered-cell edge */
	edges[n].slot = (struct ft_ord_cell **) &succ->lnode.prev;
	edges[n].old_target = cell;
	edges[n].new_target = pred;
	n++;
	edges[n].tag = URCU_TXN_TAG;	/* deletion mark: see the comment above */
	edges[n].slot = (struct ft_ord_cell **) &cell->lnode.next;
	edges[n].old_target = succ;
	edges[n].new_target = (struct ft_ord_cell *)
		urcu_txn_list_set_mark(ft_ord_cell_lnode(succ));
	n++;
	return n;
}

/* Max edges an unsplice commits: two neighbour back-edges + the deletion mark. */
#define FT_ORD_CELL_UNSPLICE_MAX_EDGES	3

/*
 * Remove @cell from the ordered cell list (its key disappeared) by committing
 * its unsplice edges through the caller-PRE-RESERVED txn @txn.  This is the
 * standalone (two-commit) path -- run AFTER the structural removal is already
 * public, so it is UN-ABORTABLE: the caller reserves @txn in its fallible prefix
 * (before the structural change, where OOM aborts the whole removal cleanly),
 * and ft_ord_cell_flip_into commits it here infallibly.
 */
static
enum urcu_txn_status ft_ord_cell_unsplice(struct cds_ft *ft,
		struct ft_flip_txn *txn,
		struct ft_ord_cell *cell)
{
	struct ft_ord_cell_edge edges[FT_ORD_CELL_UNSPLICE_MAX_EDGES] = { 0 };
	unsigned int n = ft_ord_cell_unsplice_edges(ft, cell, edges, 0);

	/*
	 * ABORT (>0) propagates: this is the two-commit fallback's SECOND
	 * commit, running after the structural commit is public, so a
	 * retry-enabled caller must DRIVE IT FORWARD (re-attempt the unsplice)
	 * rather than retry the whole op -- the key is already gone from the
	 * structural index and must not stay in the ordered list.
	 */
	return ft_ord_cell_flip_into(ft, txn, edges, n);
}

/*
 * Replace @old_cell with @new_cell at the same list position.  @new_cell
 * inherits @old_cell's neighbours; @old_cell keeps its links for parked
 * readers until its deferred free.  O(1): reuses @old_cell's neighbours, no
 * relational descent.
 *
 * The compaction cell relocation (ft-compact.h) is the sole caller, and it is
 * a best-effort relocation that ABORTS by leaving a node in place on OOM, so
 * the swap is its commit boundary: commit through a pre-reserved flip-txn
 * and propagate its status.  Returns 0 (swapped), or
 * -ENOMEM with NOTHING installed -- @old_cell stays fully in the list and the
 * caller discards the never-published @new_cell.  This was the last
 * ft_ord_cell_flip (transitional bare-store) caller; with it on a flip-txn
 * descriptor, ft_ord_cell_flip is gone and every reader-visible cell commit
 * rides the latch (readiness §6).
 */
/*
 * Worst-case records the concurrent list's replace_prepare emits into our
 * flip-txn: the load-validate guard on @next->next (only when next != prev),
 * the logical-deletion mark store on @old->next, and the two splice stores
 * prev->next and next->prev.  The single-updater list recorded only the two
 * splice stores, so migrating to the concurrent op grew the bound from 2 to 4;
 * under-reserving here would let a record-time descriptor grow fail (OOM) turn
 * a swallowed commit into a caller-visible "success" and free a still-linked
 * cell (UAF).
 */
#define FT_ORD_CELL_SWAP_REC_MAX_EDGES	4
static
int ft_ord_cell_swap(struct cds_ft *ft, struct ft_ord_cell *old_cell,
		struct ft_ord_cell *new_cell)
{
	struct ft_flip_txn *t =
		ft_flip_txn_create_bounded(FT_ORD_CELL_SWAP_REC_MAX_EDGES);

	if (caa_unlikely(!t))
		return -ENOMEM;
	/*
	 * Single-cell in-place replace via the public composable op: it records
	 * prev->next: old -> new and next->prev: old -> new (plus the concurrent
	 * list's deletion mark on old->next and, when @old is not the sole interior
	 * cell, a load-validate guard on next->next) into our FT flip-txn.
	 * FT_ORD_CELL_SWAP_REC_MAX_EDGES covers that set, so the pre-reserved commit
	 * is infallible.  The concurrent list's URCU_TXN_TAG applies, so the
	 * bidirectional ordered reader resolves the splice atomically via
	 * urcu_txn_list_resolve.  Sentinel topology: @old_cell's neighbours are its
	 * sentinel when it is the list first / last, so the same splice stores ARE
	 * the old head / tail relocation -- no endpoint edge.  The concurrent op
	 * takes (old, new), matching the single-updater list and cds_list_replace_rcu().
	 */
	(void) urcu_txn_list_replace_prepare(t->mtxn, ft_ord_cell_lnode(old_cell),
		ft_ord_cell_lnode(new_cell));
	return ft_flip_txn_commit(ft, t) < 0 ? -ENOMEM : 0;
}

/*
 * Append @old_cell -> @new_cell's in-place list-slot swap edges (its <=2
 * neighbour back-edges plus any head/tail endpoint repair) to @edges, returning
 * the new count.  The cell keeps its list POSITION; only its identity moves, so
 * @new_cell inherits @old_cell's resolved predecessor/successor.  Split out of
 * ft_ord_cell_swap_publish so a replace that touches a SECOND reader-visible
 * structural slot (a compressed head's cn->child forward edge + the grandparent
 * SKIP_X dual) can fuse both structural edges with the cell swap in one flip
 * (ft_ord_cell_swap_publish_multi).
 */
static
unsigned int ft_ord_cell_swap_edges(struct cds_ft *ft,
		struct ft_ord_cell *old_cell, struct ft_ord_cell *new_cell,
		struct ft_ord_cell_edge *edges, unsigned int n)
{
	struct ft_ord_cell *pred = ft_ord_cell_resolve_ord(&old_cell->lnode.prev);
	struct ft_ord_cell *succ = ft_ord_cell_resolve_ord(&old_cell->lnode.next);

	(void) ft;
	new_cell->lnode.prev = ft_ord_cell_lnode(pred);
	new_cell->lnode.next = ft_ord_cell_lnode(succ);
	/*
	 * Sentinel topology: @pred / @succ resolve to the sentinel when @old_cell
	 * is the list first / last, so &pred->lnode.next / &succ->lnode.prev IS the
	 * old head / tail relocation -- the same two edges as
	 * urcu_txn_list_replace_prepare, built into the fusion array so the head
	 * swap rides the SAME flip as a second reader-visible structural edge.
	 */
	edges[n].tag = URCU_TXN_TAG;	/* ordered-cell edge */
	edges[n].slot = (struct ft_ord_cell **) &pred->lnode.next;
	edges[n].old_target = old_cell;
	edges[n].new_target = new_cell;
	n++;
	edges[n].tag = URCU_TXN_TAG;	/* ordered-cell edge */
	edges[n].slot = (struct ft_ord_cell **) &succ->lnode.prev;
	edges[n].old_target = old_cell;
	edges[n].new_target = new_cell;
	n++;
	/*
	 * Deletion mark on the RETIRED cell (urcu_txn_list_del_prepare parity --
	 * see ft_ord_cell_unsplice_edges): without it, a peer's splice that
	 * captured @old_cell as its pred/succ before this swap would find its
	 * bit-identical links and commit into the retired cell.  Readers mask
	 * the bit.
	 */
	edges[n].tag = URCU_TXN_TAG;
	edges[n].slot = (struct ft_ord_cell **) &old_cell->lnode.next;
	edges[n].old_target = succ;
	edges[n].new_target = (struct ft_ord_cell *)
		urcu_txn_list_set_mark(ft_ord_cell_lnode(succ));
	n++;
	return n;
}

/*
 * Edges ft_ord_cell_swap_publish_multi commits: <=2 structural (the forward
 * publish + a compressed parent's SKIP_X dual) + <=5 cell (two neighbour
 * back-edges + the retired cell's deletion mark + the head/tail endpoint
 * repairs).  A caller that must pre-reserve its flip-txn sizes it to this.
 */
#define FT_ORD_CELL_SWAP_PUBLISH_MAX_EDGES	7

/*
 * Replace touching up to 2 reader-visible structural slots, fused with the head
 * cell's in-place swap in ONE flip: the multi-structural-edge analog of
 * ft_ord_cell_swap_publish (and the swap-dual of ft_remove_commit_rec).  Used by
 * the ordered-list-on external-leaf replace reached through a SKIP_X suffix,
 * where the new head must appear atomically at BOTH the exact-descent forward
 * slot (cn->child) AND the candidate-descent grandparent SKIP_X slot -- a reader
 * never sees the two disagree.  @sedges holds @n_sedge (1..2) structural edges;
 * @old_cell/@new_cell may be NULL (then only the structural edges flip, as the
 * list-off caller does by flipping @sedges directly).
 *
 * @txn (optional, the pre-reserve-or-grow split): when the caller performs a
 * LIVE back-pointer plain store BEFORE this flip -- the swapped-in head's prev
 * (ft_promote_head) or the chain successor's prev (cds_ft_replace) -- that store
 * must stay SETTLED for a concurrent reader and the SKIP_X resolution
 * (ft_resolve_head_prev reads a head's prev RAW), so it CANNOT ride the flip;
 * the caller instead PRE-RESERVES @txn (>= FT_ORD_CELL_SWAP_PUBLISH_MAX_EDGES)
 * in its fallible prefix, before the live store, and the commit here goes through
 * the OOM-infallible ft_ord_cell_flip_into.  @txn NULL = the fresh-head
 * case (no live store precedes the flip, so the flip is the op's sole
 * side-effect): self-allocate via ft_ord_cell_flip_try.  Returns 0, -ENOMEM
 * (nothing applied), or -EAGAIN (concurrent-writer commit ABORT, nothing
 * applied -- the caller unwinds its pre-flip side-effects and retries).
 */
static
int ft_ord_cell_swap_publish_multi(struct cds_ft *ft,
		struct ft_ord_cell *old_cell, struct ft_ord_cell *new_cell,
		const struct ft_ord_cell_edge *sedges, unsigned int n_sedge,
		struct ft_flip_txn *txn)
{
	struct ft_ord_cell_edge edges[FT_ORD_CELL_SWAP_PUBLISH_MAX_EDGES] = { 0 };
	unsigned int n = 0, i;

	for (i = 0; i < n_sedge; i++)
		edges[n++] = sedges[i];
	if (new_cell)
		n = ft_ord_cell_swap_edges(ft, old_cell, new_cell, edges, n);
	if (txn)
		return ft_ord_cell_flip_into(ft, txn, edges, n) > 0 ?
			-EAGAIN : 0;
	return ft_ord_cell_flip_try(ft, edges, n);
}

/* A pure structural publish: forward parent slot + a compressed SKIP_X dual. */
#define FT_PUB_SEDGE_MAX_EDGES	2

/*
 * Copy @rec's <=2 structural edges (the forward parent slot plus a compressed
 * parent's SKIP_X dual, populated by _ft_publish_to_parent) into @sedges for a
 * fused commit (ft_ord_cell_swap_publish_multi).  Returns the
 * edge count.
 */
static
unsigned int ft_pub_rec_sedges(struct ft_pub_rec *rec,
		struct ft_ord_cell_edge *sedges)
{
	unsigned int i;

	for (i = 0; i < rec->n; i++) {
		sedges[i].slot = (struct ft_ord_cell **) rec->slot[i];
		sedges[i].old_target = (struct ft_ord_cell *) rec->old_val[i];
		sedges[i].new_target = (struct ft_ord_cell *) rec->new_val[i];
	}
	return rec->n;
}

/*
 * Key-disappearing remove, fused (the dual of ft_ord_cell_swap_publish): commit
 * a key's single reader-visible structural unlink (@struct_slot transitions from
 * @struct_old to @struct_new -- a leaf body slot or compressed cn->child cleared
 * to NULL, or an internal holder's external_nodes cleared) ATOMICALLY with the
 * dead head cell's ordered-list unsplice, in ONE flip.  A reader thus never
 * observes the key gone from the structural index but still present in the
 * ordered list (or vice versa).  @dead_cell may be NULL (ordered list off): then
 * only the structural edge flips.
 *
 * @struct_slot must be the SOLE reader-visible slot whose change removes the
 * key; shapes that touch a second reader-visible slot (a compressed parent's
 * SKIP_X dual pointer, a recompacted node's grandparent edge) do NOT use this.
 */
static
int ft_remove_one_commit(struct cds_ft *ft,
		struct cds_ft_inode_flag **struct_slot,
		struct cds_ft_inode_flag *struct_old,
		struct cds_ft_inode_flag *struct_new,
		struct cds_ft_metadata *state_meta,
		struct ft_ord_cell *dead_cell,
		struct ft_detach_run *run,
		struct ft_flip_txn *txn,
		struct cds_ft_node *freeze_leaf,
		bool record_only)
{
	struct ft_ord_cell_edge edges[7] = { 0 };	/* 1 struct + state + <=4 cell/run + leaf freeze */
	unsigned int n = 0;

	edges[n].slot = (struct ft_ord_cell **) struct_slot;
	edges[n].old_target = (struct ft_ord_cell *) struct_old;
	edges[n].new_target = (struct ft_ord_cell *) struct_new;
	n++;
	if (run)
		n = ft_ord_cell_run_detach_edges(ft, run->rfirst, run->rlast,
			&run->first, &run->last, edges, n);
	else if (dead_cell)
		n = ft_ord_cell_unsplice_edges(ft, dead_cell, edges, n);
	/*
	 * A removed external leaf freezes atomically with this same commit that
	 * unlinks it (doc §4.B): record its one MARK edge (freeze_leaf->next ->
	 * MARK(next), a single-entry chain so next is NULL) beside the structural +
	 * cell edges.  FT_HLIST_TAG (== URCU_TXN_TAG) so a reader resolves a parked
	 * proxy via cds_ft_node_next_rcu; byte-identical to ft_hlist_freeze_prepare.
	 * On the @txn NULL path this makes n >= 2, so the infallible lone-edge
	 * on-stack store yields to a bounded flip-txn -- the single-writer force-txn
	 * cost of the atomic detach, a clean OOM abort (nothing installed, leaf stays
	 * chained).  NULL when the caller is not retiring a leaf here.
	 */
	if (freeze_leaf) {
		void *cur = freeze_leaf->next;

		edges[n].slot = (struct ft_ord_cell **) &freeze_leaf->next;
		edges[n].old_target = (struct ft_ord_cell *) cur;
		edges[n].new_target = (struct ft_ord_cell *)
			ft_hlist_set_mark((struct cds_ft_node *) cur);
		edges[n].tag = FT_HLIST_TAG;
		n++;
	}
	/*
	 * @state_meta non-NULL (a delete): fuse its nr_child-- into THIS flip so
	 * the structural unlink and the count decrement go live atomically.  Only
	 * when @txn is present, though -- a @txn NULL commit is the op's lone-edge
	 * ABORT BOUNDARY (n == 1, on-stack, infallible) and the caller ignores the
	 * return, so a second fused edge there would make it heap/fallible; the
	 * decrement rides its own lone-edge flip after instead (still a committed
	 * edge, just not atomic with the structure -- as in the pre-fusion path).
	 *
	 * @txn non-NULL: a caller-PRE-RESERVED bounded txn -- the flip commits
	 * through it (ft_ord_cell_flip_into, OOM-infallible); returns 0, or
	 * -EAGAIN on a concurrent-writer commit ABORT (nothing installed -- the
	 * fused count/freeze/tombstone edges were discarded with it; the caller
	 * retries).  @txn NULL: commit via ft_ord_cell_flip_try, which installs
	 * nothing on OOM (a lone edge is the infallible on-stack store), and
	 * return -ENOMEM so the caller aborts the removal with the structure
	 * untouched (-EAGAIN likewise propagates from a multi-edge conflict).
	 */
	if (txn) {
		if (state_meta) {
			/*
			 * WAITING load: this word enters the txn's write set on
			 * the very next line.  A raw read would take a peer's
			 * parked proxy as the expected-old AND mint the new
			 * value out of pointer bits (proxy - NR_CHILD_ONE), so
			 * a matching install would publish a corrupted word --
			 * the transacted form of the hazard
			 * ft_meta_state_transition waits out.
			 */
			uintptr_t old = (uintptr_t) urcu_txn_load(txn->mtxn,
				(void **) &state_meta->state, FT_STATE_PROXY);

			ft_state_edge(&edges[n], &state_meta->state, old,
				old - FT_STATE_NR_CHILD_ONE);
			n++;
		}
		/*
		 * FOLD (coherent rekey one-decide writer): record the SW structural
		 * slot + nr_child-- + cell unsplice into the caller's SHARED mixed txn
		 * and RETURN -- the caller runs the ONE ft_flip_txn_commit that also
		 * carries the dst-attach (a cell/count MW-conflict aborts it clean
		 * before any SW side effect, so the caller re-descends).  The run-arm
		 * (ft_ord_cell_run_install) must FOLLOW that flip, so it is deferred to
		 * the caller's post-commit finalize; the record-only detach fold today
		 * covers only the simple case (internal S_top src junction), which
		 * carries no head run -- asserted here until run-arm defer lands.
		 */
		if (record_only) {
			assert(!run);
			ft_ord_cell_record_into(txn, edges, n);
			return 0;
		}
		if (ft_ord_cell_flip_into(ft, txn, edges, n) > 0)
			return -EAGAIN;	/* peer won: nothing installed, caller retries */
	} else {
		assert(!record_only);	/* record-only requires a caller-supplied txn */
		int cret = ft_ord_cell_flip_try(ft, edges, n);

		if (cret)
			return cret;	/* nothing installed: caller aborts/retries (no dec) */
		if (state_meta)
			ft_meta_nr_child_dec_flip(state_meta);
	}
	if (run) {
		/* @into NULL = EXCISE-ONLY (the merge source side): the run is
		 * unlinked from @ft's list but not re-homed; its cells keep their
		 * internal links for the dst splice/interleave.  Reached only on a
		 * COMMITTED flip (an ABORT returned above), so the arm is never
		 * ghost-armed against an unchanged list. */
		if (run->into)
			ft_ord_cell_run_install(run->into, run->first, run->last);
		run->armed = true;
	}
	return 0;
}

/*
 * Record a LIVE child's parent back-edge into @rec so it rides the SAME flip as
 * the forward publish (instead of a bare ft_set_parent before it): the child's
 * parent field transitions old -> @new_parent atomically with the structural
 * publish, closing the re-parent-before-publish window and making the back-edge
 * an MCAS descriptor edge.  Resolves the field per child kind exactly as
 * ft_set_parent / ft_park_live_parent_edge do (metadata->parent for
 * internal/compressed, cell->parent / node->prev for an external head), and
 * sets the slot offset up front (write-side; the parked parent proxy makes a
 * concurrent up-walk reanchor, so the early offset is unobserved until settle).
 * Because the back-edge is deferred, the paired forward publish MUST use
 * _ft_publish_to_parent_meta with @new_parent's metadata: a SKIP_X forward flag
 * is otherwise resolved via ft_skip_to_compressed, which reads this very
 * (not-yet-stored) field.  @new_parent must be a COMPRESSED/internal flag whose
 * incoming_byte the child inherits via the parent's own slot, so no
 * incoming_byte write is needed here (matching ft_park_live_parent_edge).
 */
static void ft_reparent_record_meta(struct cds_ft *ft, struct ft_flip_txn *txn,
		struct cds_ft_metadata *meta,
		struct cds_ft_inode_flag *parent_nf,
		struct cds_ft_inode_flag **slot, bool child_marked);

static
void ft_pub_rec_add_back_edge(struct cds_ft *ft, struct ft_pub_rec *rec,
		struct ft_flip_txn *txn,
		struct cds_ft_inode_flag *child,
		struct cds_ft_inode_flag *new_parent,
		struct cds_ft_inode_flag **slot)
{
	struct cds_ft_metadata *meta = NULL;
	struct cds_ft_inode_flag **field;

	if (!child)
		return;
#ifdef FEATURE_FT_SKIP_COMPRESSED
	if (ft_node_skip_compressed(child))
		meta = cds_ft_item_to_metadata((struct cds_ft_inode *)
			ft_skip_to_compressed(ft, child));
	else
#endif
	if (ft_node_compressed(child))
		meta = cds_ft_item_to_metadata((struct cds_ft_inode *)
			ft_compressed_node_ptr(child));
	else if (!ft_node_external(child))
		meta = cds_ft_item_to_metadata(ft_node_ptr(child));

	if (meta) {
		/*
		 * LIVE child with metadata: its (parent, offset) must be a
		 * CO-COMMITTED pair riding @txn (ft_reparent_record_meta
		 * records parent + the state-word PSO edge together).  The
		 * former eager ft_set_parent_slot left the offset as a SETTLED
		 * store: an ABORTED flip discarded the parent edge but kept the
		 * offset -- a mismatched (old parent, new offset) pair that
		 * downstream slot derivation consumed unvalidated.  The
		 * incoming_byte settled store inside reparent_record_meta is
		 * skipped here (the new parent is always compressed), matching
		 * the "no incoming_byte write" contract above.
		 */
		ft_reparent_record_meta(ft, txn, meta, new_parent, slot,
			/*child_marked=*/ false);
		return;
	} else if (ft->ordered_list) {
		field = &ft_ord_cell_ptr(
			((struct cds_ft_node *) child)->prev)->parent;
	} else {
		field = (struct cds_ft_inode_flag **)
			&((struct cds_ft_node *) child)->prev;
	}
	/*
	 * Back-edge expected-old = the child's current back-pointer (cell->parent
	 * / prev); matches the sibling ft_reparent_record_meta's raw meta->parent.
	 */
	ft_pub_rec_add(rec, field, *field, new_parent);
}

/*
 * Key-disappearing remove via recompaction: commit the recompacted node's
 * 1-2 reader-visible structural stores -- recorded by ft_publish_to_parent
 * into @rec (the forward parent slot, plus a compressed parent's SKIP_X dual
 * pointer) -- ATOMICALLY with the dead head cell's ordered-list unsplice, in
 * ONE flip.  Fusing the forward and skip-dual edges in a single flip also
 * closes the candidate-before-exact ordering window the two-store publish
 * relied on: a reader sees the whole old-XOR-new transition at once.
 * @dead_cell may be NULL (list off): then only the recorded edges flip.
 *
 * @txn: when non-NULL, a caller-PRE-RESERVED bounded txn (capacity >=
 * FT_REMOVE_COMMIT_REC_MAX_EDGES) reserved in the op's fallible build phase --
 * the flip then commits through it (ft_ord_cell_flip_into) and cannot
 * ALLOC-fail, so a caller that has already wired a pre-flip side-effect (e.g. a
 * child's parent-slot offset via ft_pub_rec_add_back_edge) reaches an
 * allocation-free point of no return.  NULL keeps the transitional
 * self-allocating flip (bare-store fallback) for callers not yet migrated.
 *
 * RETURNS the commit status: ABORT (>0) means a peer writer won and NOTHING
 * was installed (all fused edges discarded); a retry-enabled caller unwinds
 * and re-descends.  The @txn-NULL lone-store path cannot abort (returns OK).
 */
#define FT_REMOVE_COMMIT_REC_MAX_EDGES	10	/* <=3 structural (+back-edge) + <=5 cell/run (unsplice = 2 back-edges + deletion mark) + 1 DEL-recompact tombstone + 1 promote head re-parent */
static
enum urcu_txn_status ft_remove_commit_rec(struct cds_ft *ft,
		struct ft_pub_rec *rec,
		struct ft_ord_cell *dead_cell, struct ft_detach_run *run,
		struct ft_flip_txn *txn, bool record_only)
{
	struct ft_ord_cell_edge edges[FT_REMOVE_COMMIT_REC_MAX_EDGES] = { 0 };
	unsigned int n = 0, i;

	for (i = 0; i < rec->n; i++) {
		edges[n].slot = (struct ft_ord_cell **) rec->slot[i];
		edges[n].old_target = (struct ft_ord_cell *) rec->old_val[i];
		edges[n].new_target = (struct ft_ord_cell *) rec->new_val[i];
		n++;
	}
	if (run)
		n = ft_ord_cell_run_detach_edges(ft, run->rfirst, run->rlast,
			&run->first, &run->last, edges, n);
	else if (dead_cell)
		n = ft_ord_cell_unsplice_edges(ft, dead_cell, edges, n);
	if (record_only) {
		/*
		 * FOLD (coherent rekey one-decide writer): record the recompaction's
		 * forward republish (+ any cell edges) into the caller's SHARED txn
		 * WITHOUT committing -- the caller runs the ONE commit that also carries
		 * the dst-attach + S_top COW.  Structural edges park SW (the recompacted
		 * src junction holds its own + the shared-spine parent's node lock via
		 * the fold's parent_held coordination); cells stay MW.  The run-arm is
		 * deferred to the caller's post-commit finalize; the record-only rekey is
		 * list-off (no run/cell) so far -- asserted.
		 */
		assert(!run && !dead_cell);
		ft_ord_cell_record_into(txn, edges, n);
		return URCU_TXN_STATUS_OK;
	}
	if (txn) {
		enum urcu_txn_status st = ft_ord_cell_flip_into(ft, txn,
				edges, n);

		if (st > 0)
			return st;	/* peer won: nothing installed, no run arm */
	} else {
		/*
		 * NULL @txn: a non-fused recompaction / external-promote whose
		 * forward publish is a LONE edge -- a single release store with
		 * no second reader-visible slot.  A SKIP_X dual only arises when
		 * the publish parent is compressed, and that case reserves @txn
		 * in the caller's fallible prefix (ft_detach_node's
		 * boundary-parent-compressed gate), so a NULL @txn here never
		 * carries a dual: the commit is at most one edge, infallible on
		 * the stack.  (n == 0 is a vacuous publish: nothing recorded.)
		 */
		assert(n <= 1);
		if (n)
			ft_ord_cell_flip_one(&edges[0]);
	}
	if (run) {
		/* @into NULL = EXCISE-ONLY (the merge source side): the run is
		 * unlinked from @ft's list but not re-homed; its cells keep their
		 * internal links for the dst splice/interleave.  Reached only on
		 * a COMMITTED flip (ABORT returned above). */
		if (run->into)
			ft_ord_cell_run_install(run->into, run->first, run->last);
		run->armed = true;
	}
	return URCU_TXN_STATUS_OK;
}

#ifdef FT_DEBUG_SPLICE_POS_BRACKET
/*
 * =====================================================================
 * DIAGNOSTIC PROBE (opt-in: -DFT_DEBUG_SPLICE_POS_BRACKET).  NOT a fix.
 * =====================================================================
 *
 * ft_ord_cell_find_splice_pos derives (@pred, @succ) with an unlocked relational
 * descent, and the only thing validated downstream is the ADJACENCY of the pair
 * -- which a wrong-but-adjacent pair satisfies by construction, since @succ is
 * READ OFF @pred->next.  ADJACENT + BRACKETING is the full condition and
 * bracketing is never checked, so a far-wrong @pred commits with a fully
 * "successful" CAS and parks a whole run at the wrong list position.
 *
 * This probe validates the BRACKET at the derivation site and ABORTS with a dump
 * on the first violation.  It deliberately does NOT retry: a previous attempt
 * that bailed to the graft's retry_attach on a failed bracket LIVELOCKED 38/38
 * (it rejected valid pairs, so the graft re-derived forever) and told us nothing.
 * A validation that is not yet trusted must be DIAGNOSTIC before it is
 * load-bearing.  Hence the discipline here:
 *
 *   - SKIP (silently, never abort) anything it cannot evaluate: an up-walk that
 *     returns 0 bytes, a sentinel neighbour, an over-long key.  A rebuild that
 *     legitimately yields nothing must not be read as a violation -- that was
 *     one of the three named suspects for the livelock.
 *   - Compare in ORDINAL space.  ft_rebuild_key_upwalk yields the ORDINAL key
 *     while @key is APPLICATION form, so the query is mapped through
 *     key_to_ordinal (the identity in the oracles, but not in general).
 *   - Respect the RIGHT-ALIGNED rebuild: the key occupies buf[max_len - n .. )
 *     and n is the length.  Mis-slicing that was the second named suspect.
 *
 * What the dump discriminates (the open question from the handoff, §18):
 *   cell->node == head  =>  the DESCENT returned a far-wrong head.
 *   cell->node != head  =>  a RIGHT head was mapped to a WRONG cell through a
 *                           stale head->prev (a peer promoting a duplicate-chain
 *                           head, or a graft re-parenting one).
 * Prediction under test: the bad @pred's key equals the query with byte 0
 * incremented by one (the observed {0f,02} -> max of {10,02}).
 */
static
int ft_dbg_ord_key_cmp(const uint8_t *a, size_t alen,
		const uint8_t *b, size_t blen)
{
	size_t n = alen < blen ? alen : blen;
	int c = n ? memcmp(a, b, n) : 0;

	if (c)
		return c;
	return alen < blen ? -1 : (alen > blen ? 1 : 0);
}

static
void ft_dbg_key_print(const char *tag, const uint8_t *k, size_t n)
{
	size_t i;

	fprintf(stderr, "  %s (len %zu):", tag, n);
	for (i = 0; i < n; i++)
		fprintf(stderr, " %02x", k[i]);
	fprintf(stderr, "\n");
}

static
void ft_dbg_splice_pos_check(struct cds_ft *dst, const uint8_t *key,
		size_t key_len, struct ft_ord_cell *pred,
		struct cds_ft_node *pred_head, struct ft_ord_cell *succ,
		struct cds_ft_node *succ_head)
{
	const struct cds_ft_key_map *km = &dst->group->key_map;
	struct ft_ord_cell *sentinel = ft_ord_sentinel_cell(dst);
	size_t max_len = dst->group->max_key_len;
	uint8_t qord[FT_MAX_KEY_LEN];
	uint8_t pbuf[FT_MAX_KEY_LEN], sbuf[FT_MAX_KEY_LEN];
	size_t pn = 0, sn = 0, i;
	bool pbad = false, sbad = false;

	if (!key_len || key_len > max_len || max_len > FT_MAX_KEY_LEN)
		return;				/* cannot evaluate -- SKIP */
	for (i = 0; i < key_len; i++)
		qord[i] = km->identity ? key[i] : km->key_to_ordinal[key[i]];

	/* @pred must sort STRICTLY BELOW the query. */
	if (pred && pred != sentinel) {
		pn = ft_rebuild_key_upwalk(dst, pred, pbuf, max_len);
		if (pn)
			pbad = ft_dbg_ord_key_cmp(pbuf + (max_len - pn), pn,
					qord, key_len) >= 0;
	}
	/* @succ must sort STRICTLY ABOVE it. */
	if (succ && succ != sentinel) {
		sn = ft_rebuild_key_upwalk(dst, succ, sbuf, max_len);
		if (sn)
			sbad = ft_dbg_ord_key_cmp(sbuf + (max_len - sn), sn,
					qord, key_len) <= 0;
	}
	if (!pbad && !sbad)
		return;

	fprintf(stderr, "\n=== FT SPLICE-POS BRACKET VIOLATION ===\n");
	fprintf(stderr, "ft %p  pred %s  succ %s\n", (void *) dst,
		pbad ? "BAD" : "ok", sbad ? "BAD" : "ok");
	ft_dbg_key_print("query    (app)", key, key_len);
	ft_dbg_key_print("query    (ord)", qord, key_len);
	if (pred) {
		fprintf(stderr, "  pred cell %p node %p  descent head %p  "
			"mapping %s\n", (void *) pred, (void *) pred->node,
			(void *) pred_head,
			(!pred_head || pred->node == pred_head)
				? "CONSISTENT (cell->node == head)"
				: "STALE (cell->node != head)");
		if (pn)
			ft_dbg_key_print("pred key (ord)",
				pbuf + (max_len - pn), pn);
		else
			fprintf(stderr, "  pred key: up-walk yielded nothing\n");
	} else {
		fprintf(stderr, "  pred NULL (query claimed to be new minimum)\n");
	}
	if (succ && succ != sentinel) {
		fprintf(stderr, "  succ cell %p node %p  descent head %p  "
			"mapping %s\n", (void *) succ, (void *) succ->node,
			(void *) succ_head,
			(!succ_head || succ->node == succ_head)
				? "CONSISTENT (cell->node == head)"
				: "STALE (cell->node != head)");
		if (sn)
			ft_dbg_key_print("succ key (ord)",
				sbuf + (max_len - sn), sn);
	} else {
		fprintf(stderr, "  succ is the sentinel / NULL (list tail)\n");
	}
	fprintf(stderr, "=== aborting for a core ===\n");
	fflush(stderr);
	abort();
}
#endif /* FT_DEBUG_SPLICE_POS_BRACKET */

/*
 * Locate the ordered-list neighbours (@pred, @succ) that a run grafted at @key
 * will splice between.  MUST be called while @dst is still payload-free (before
 * the structural attach publishes the grafted subtree), else the relational
 * descent would return a payload head as the boundary.  @key is APPLICATION form
 * (find_rel remaps).  Since the attach point is empty, @pred = last @dst key <
 * @key and @succ = first @dst key > @key (nothing of @dst's lies in the run's
 * range in between).
 *
 * ★ BOTH ENDPOINTS ARE DERIVED BY KEY ORDER.  @succ must NOT be read off
 * @pred->next: that makes the pair adjacent BY CONSTRUCTION and every
 * downstream expected-old CAS vacuous, since {pred->next expect succ} cannot
 * fail when @succ is defined as @pred->next.  The window is not the
 * commit's -- it is
 * between the DESCENT that produced @pred (T0) and the @pred->next read (T1): a
 * peer graft committing a run into that gap is READ BACK as our successor, so we
 * splice ahead of a run that belongs before us and the list goes out of order.
 * (Measured: FT_DEBUG_SPLICE_POS_BRACKET, 8/8 firings had @pred correct, @succ a
 * peer run's FIRST cell.  The CAS covers [T1, commit]; the tear is in [T0, T1].)
 *
 * Deriving @succ INDEPENDENTLY (its own GT descent) makes the recorded
 * {pred->next expect succ} a REAL check: it now asserts that the interval
 * between two separately key-ordered endpoints is EMPTY, so a peer that
 * interposed anywhere in that interval fails the value CAS and the caller's
 * retry re-derives.  ADJACENT + BRACKETING, which adjacency alone never was.
 *
 * This is the order-pinned discipline the POINT insert already runs
 * (ft_txn_list_insert_between_prepare: "refuses unless @pos->next still equals
 * @succ_expected ... so a later interposition fails the commit's value CAS
 * instead of being adopted"), whose own comment records that splicing anyway
 * "produced the resurrected / out-of-order cell class the ord-verify catches at
 * rest".  The BULK run splice simply never got it.  Costs one extra relational
 * descent per bulk splice, on a path that already descends and allocates.
 */
static
void ft_ord_cell_find_splice_pos(struct cds_ft *dst, const uint8_t *key,
		size_t key_len, struct ft_ord_cell **pred_out,
		struct ft_ord_cell **succ_out,
		struct ft_visit_witness *wit)
{
	struct ft_ord_cell *pred, *succ;
	size_t flen = dst->group->key_len;
#ifdef FT_DEBUG_SPLICE_POS_BRACKET
	struct cds_ft_node *pred_head = NULL, *succ_head = NULL;
	struct cds_ft_node **ph = &pred_head, **sh = &succ_head;
#else
	/* Production: no head is wanted, so find_rel skips the store. */
	struct cds_ft_node **ph = NULL, **sh = NULL;
#endif

	if (flen != CDS_FT_LEN_VARIABLE && key_len != flen) {
		/*
		 * Internal graft on a FIXED-length group (ft_graft_keylen, e.g.
		 * cds_ft_merge_at's detach+graft path): @key is the graft
		 * PREFIX, shorter than the group's key length, which the public
		 * relational lookups reject -- probing with it verbatim came
		 * back empty and the grafted run was silently never spliced
		 * (merged keys invisible to ordered iteration).  Probe with the
		 * prefix PADDED to the fixed length instead: the attach point
		 * is empty (graft returns POPULATED_ERROR otherwise), so no
		 * @dst key starts with @key, and
		 *   LT(key . min..min) = last @dst key below the prefix range,
		 *   GT(key . max..max) = first @dst key above it.
		 * Pad bytes are the ordinal-space extremes mapped back to
		 * application form (find_rel remaps app -> ordinal).
		 */
		const struct cds_ft_key_map *km = &dst->group->key_map;
		uint8_t pad_min = km->identity ? 0x00 : km->ordinal_to_key[0x00];
		uint8_t pad_max = km->identity ? 0xff : km->ordinal_to_key[0xff];
		uint8_t pad[FT_MAX_KEY_LEN];

		assert(key_len < flen && flen <= FT_MAX_KEY_LEN);
		memcpy(pad, key, key_len);
		memset(pad + key_len, pad_min, flen - key_len);
		pred = ft_ord_cell_find_rel(dst, pad, flen, FT_LOOKUP_LT, wit,
				ph);
		/*
		 * @succ from its OWN GT descent over the max-padded probe -- never
		 * off @pred->next (see the header).  LT(key . min..min) is the last
		 * @dst key below the prefix range; GT(key . max..max) is the first
		 * above it.
		 */
		memset(pad + key_len, pad_max, flen - key_len);
		succ = ft_ord_cell_find_rel(dst, pad, flen, FT_LOOKUP_GT, wit,
				sh);
		if (wit) {
			ft_witness_visit(wit, pred);
			ft_witness_visit(wit, succ);
		}
		*pred_out = pred;
		*succ_out = succ;
#ifdef FT_DEBUG_SPLICE_POS_BRACKET
		/*
		 * Checked against the BARE PREFIX, not @pad: the attach point is
		 * empty, so no @dst key starts with @key, and a prefix sorts below
		 * every extension of itself.  "pred < prefix" and "succ > prefix"
		 * is therefore the exact bracket for the whole padded range.
		 */
		ft_dbg_splice_pos_check(dst, key, key_len, pred, pred_head,
			succ, succ_head);
#endif
		return;
	}

	pred = ft_ord_cell_find_rel(dst, key, key_len, FT_LOOKUP_LT, wit, ph);
	/* @succ from its OWN GT descent -- never off @pred->next (see header). */
	succ = ft_ord_cell_find_rel(dst, key, key_len, FT_LOOKUP_GT, wit, sh);
	if (wit) {
		ft_witness_visit(wit, pred);
		ft_witness_visit(wit, succ);
	}
	*pred_out = pred;
	*succ_out = succ;
#ifdef FT_DEBUG_SPLICE_POS_BRACKET
	ft_dbg_splice_pos_check(dst, key, key_len, pred, pred_head,
		succ, succ_head);
#endif
}

/*
 * COHERENT splice-position derivation: run ft_ord_cell_find_splice_pos TWICE and
 * accept the answer only if both passes agree -- same (pred, succ) AND the same
 * visited-node witness.
 *
 * Why two passes are needed at all: find_splice_pos answers a RELATIONAL question
 * (the neighbours of a key), and a relational traversal is not coherence-hardened
 * -- while an in-trie move is in flight it can return a pair that is genuinely
 * ADJACENT in the list but sits at the WRONG key position, which every downstream
 * edge then validates happily.  Why two passes are ENOUGH: a single traversal can
 * witness such a torn view only by STRADDLING the move's commit, and two
 * SEQUENTIAL traversals cannot both straddle the same commit (the second starts
 * after the first ended), so agreement proves neither did.
 *
 * Why the WITNESS and not just the returned pair: a torn pass can land on a pair
 * that is stable across both passes (the result is a real, unmoving cell), so
 * comparing results alone accepts it.  The visited-node addresses differ, because
 * a move COWs its stitch points into fresh addresses -- which is also what makes
 * this immune to an oscillating rekey that would return an address to its old
 * value.
 *
 * Returns true with *@pred_out / *@succ_out set, or false when the two passes
 * disagreed: a move is restructuring this neighbourhood, so the caller must bail
 * and re-derive (there is no bounded amount of re-trying that makes an incoherent
 * derivation coherent).  Must run under one RCU read lock, like any traversal.
 */
static
bool ft_ord_cell_find_splice_pos_coherent(struct cds_ft *dst, const uint8_t *key,
		size_t key_len, struct ft_ord_cell **pred_out,
		struct ft_ord_cell **succ_out)
{
	struct ft_ord_cell *pred1, *succ1, *pred2, *succ2;
	struct ft_visit_witness w1, w2;

	ft_witness_init(&w1);
	ft_ord_cell_find_splice_pos(dst, key, key_len, &pred1, &succ1, &w1);
	ft_witness_init(&w2);
	ft_ord_cell_find_splice_pos(dst, key, key_len, &pred2, &succ2, &w2);
	if (pred1 != pred2 || succ1 != succ2 || !ft_witness_equal(&w1, &w2))
		return false;
	*pred_out = pred1;
	*succ_out = succ1;
	return true;
}

/*
 * Splice the contiguous ordered-list run [@run_first .. @run_last] (already
 * linked internally, in key order) into @dst's ordered cell list BETWEEN the
 * given neighbours @pred and @succ -- the cds_ft_graft shape, where the run is
 * the source trie's whole list attached at an EMPTY point in @dst (graft returns
 * POPULATED_ERROR otherwise, so no @dst key interleaves the run's range).
 *
 * @pred / @succ MUST be located BEFORE the structural attach publishes the
 * payload into @dst (see ft_ord_cell_find_splice_pos): a relational descent run
 * after the payload is live would return a PAYLOAD head (part of the run itself)
 * as the boundary.  @pred / @succ are @dst-original cells, which graft never
 * moves, so they stay valid until this splice.
 *
 * APPENDS the <=4 edges -- the run's two OUTER links plus the two @dst
 * neighbour back-edges (which double as the @dst head/tail repair under the
 * sentinel topology) -- to @edges, leaving the caller to flip them.  Split out
 * (the appear-side dual of ft_ord_cell_run_detach_edges) so a bulk graft can
 * FUSE these edges with its structural attach publish in ONE flip
 * (ft_store_at_graft_point's batch), closing the appear-side cross-view window;
 * ft_ord_cell_run_splice is the standalone (two-commit) wrapper.
 *
 * WHY THE OUTER LINKS ARE RECORDED, not plain-stored.  Plain stores would rest
 * on the run being unreachable in @dst AND this section being un-abortable.
 * The second half does not hold: cell edges are ALWAYS MW, so a peer's
 * conflicting boundary splice aborts this commit, and a plain store does NOT
 * roll back.
 * That leaves run_first->prev / run_last->next pointing into @dst's
 * neighbourhood while both neighbourhoods still point at their old targets --
 * the "PERMANENTLY broken back-edge" that broke inv_rekey_graft_shared and made
 * ft_ord_cell_run_resplice_edges record all four (see its comment).  The
 * cross-trie shape kept the plain stores and the drop made it abortable too, so
 * it now records them the same way: the whole splice is atomic and rolls back
 * clean.  Expected-old comes from the run's still-pristine outer links (the
 * SOURCE-side neighbours ft_ord_cell_run_install left in place), so this must be
 * appended BEFORE any edge of the set is flipped.
 *
 * NOTE: structurally identical to ft_ord_cell_run_resplice_edges (same four
 * edges, same order) -- they differ only in their documented preconditions
 * (cross-trie incoming run vs same-trie LIVE run).  Keep them in sync.
 */
static
unsigned int ft_ord_cell_run_splice_edges(struct cds_ft *dst,
		struct ft_ord_cell *run_first, struct ft_ord_cell *run_last,
		struct ft_ord_cell *pred, struct ft_ord_cell *succ,
		struct ft_ord_cell_edge *edges, unsigned int n)
{
	struct ft_ord_cell *src_pred =
		ft_ord_cell_resolve_ord(&run_first->lnode.prev);
	struct ft_ord_cell *src_succ =
		ft_ord_cell_resolve_ord(&run_last->lnode.next);

	/*
	 * Sentinel topology: a NULL boundary neighbour (run at @dst's head / tail)
	 * IS @dst's sentinel, so the run's outer link and the back-edge land on
	 * &dst->ord_sentinel.node -- recording the old head / tail flip without a
	 * separate endpoint edge.
	 */
	pred = ft_ord_or_sentinel(dst, pred);
	succ = ft_ord_or_sentinel(dst, succ);
	/* The run's OUTER links: src neighbours -> dst neighbours. */
	edges[n].tag = URCU_TXN_TAG;	/* ordered-cell edge */
	edges[n].slot = (struct ft_ord_cell **) &run_first->lnode.prev;
	edges[n].old_target = src_pred;
	edges[n].new_target = pred;
	n++;
	edges[n].tag = URCU_TXN_TAG;	/* ordered-cell edge */
	edges[n].slot = (struct ft_ord_cell **) &run_last->lnode.next;
	edges[n].old_target = src_succ;
	edges[n].new_target = succ;
	n++;
	edges[n].tag = URCU_TXN_TAG;	/* ordered-cell edge */
	edges[n].slot = (struct ft_ord_cell **) &pred->lnode.next;
	edges[n].old_target = succ;
	edges[n].new_target = run_first;
	n++;
	edges[n].tag = URCU_TXN_TAG;	/* ordered-cell edge */
	edges[n].slot = (struct ft_ord_cell **) &succ->lnode.prev;
	edges[n].old_target = pred;
	edges[n].new_target = run_last;
	n++;
	return n;
}

/* Max edges a run-splice commits: 2 outer links + 2 boundary back-edges. */
#define FT_ORD_CELL_RUN_SPLICE_MAX_EDGES	4

/*
 * SAME-TRIE re-splice of a LIVE run (the coherent rekey's dst half), the variant
 * ft_ord_cell_run_splice_edges cannot be: it PRE-SETS the run's two outer links
 * with PLAIN STORES, sound only for its cross-trie shape -- an incoming run that
 * is not yet reachable in @dst, spliced in the op's un-abortable failure-free
 * section.  In a same-trie rekey the run is still LIVE at its src ordered position
 * while these edges are built (the src unsplice rides the SAME commit), so a plain
 * store would
 *   (a) publish the DST neighbours to an ordered reader still walking the run at
 *       src, BEFORE the boundary flip -- recording them instead makes each of the
 *       six slots flip atomically, so no walker reads a link mid-update.  (It does
 *       NOT make the whole move atomic for a walker that STRADDLES the commit while
 *       parked on a run cell: the run's internal links never change, so it steps off
 *       the run through the NEW outer link and lands in the dst neighbourhood,
 *       skipping the keys in between.  That is inherent to moving a live run without
 *       draining readers, not something edge atomicity can fix.)  And
 *   (b) NOT roll back when the commit ABORTS -- leaving run_first->prev /
 *       run_last->next pointing into the dst neighbourhood while both
 *       neighbourhoods still point at their old targets: a PERMANENTLY broken
 *       back-edge.  With concurrent rekey writers the final commit does abort (a
 *       peer's boundary-cell splice conflicts), which is exactly how
 *       inv_rekey_graft_shared broke the list ("ord-cell back-edge broken") where
 *       the disjoint oracle -- whose serialized moves never abort the final commit
 *       -- could not.
 * So all FOUR edges (the run's two outer links + the two dst neighbour back-edges)
 * ride the txn.  They are CELL edges (URCU_TXN_TAG), hence always MW: the ordered
 * list stays lock-free and a peer's conflicting splice aborts the mixed commit
 * CLEAN, before any SW side effect.  The run's INTERNAL links are untouched.
 *
 * @pred / @succ are the DST neighbours, located BEFORE anything is published (see
 * ft_ord_cell_find_splice_pos) and ADJACENCY-CHECKED by the caller: neither may be
 * a run endpoint, else two edges would target one slot (and the run would splice
 * into itself).  The src-side expected-old values are read here from the run's
 * still-pristine outer links, so this must be appended BEFORE any edge is flipped
 * (the caller records, then commits once).
 */
static
unsigned int ft_ord_cell_run_resplice_edges(struct cds_ft *dst,
		struct ft_ord_cell *run_first, struct ft_ord_cell *run_last,
		struct ft_ord_cell *pred, struct ft_ord_cell *succ,
		struct ft_ord_cell_edge *edges, unsigned int n)
{
	struct ft_ord_cell *src_pred =
		ft_ord_cell_resolve_ord(&run_first->lnode.prev);
	struct ft_ord_cell *src_succ =
		ft_ord_cell_resolve_ord(&run_last->lnode.next);

	/* Sentinel topology as in ft_ord_cell_run_splice_edges. */
	pred = ft_ord_or_sentinel(dst, pred);
	succ = ft_ord_or_sentinel(dst, succ);
	/* The run's OUTER links: src neighbours -> dst neighbours. */
	edges[n].tag = URCU_TXN_TAG;	/* ordered-cell edge */
	edges[n].slot = (struct ft_ord_cell **) &run_first->lnode.prev;
	edges[n].old_target = src_pred;
	edges[n].new_target = pred;
	n++;
	edges[n].tag = URCU_TXN_TAG;	/* ordered-cell edge */
	edges[n].slot = (struct ft_ord_cell **) &run_last->lnode.next;
	edges[n].old_target = src_succ;
	edges[n].new_target = succ;
	n++;
	/* The dst neighbours' back-edges, as in the cross-trie form. */
	edges[n].tag = URCU_TXN_TAG;	/* ordered-cell edge */
	edges[n].slot = (struct ft_ord_cell **) &pred->lnode.next;
	edges[n].old_target = succ;
	edges[n].new_target = run_first;
	n++;
	edges[n].tag = URCU_TXN_TAG;	/* ordered-cell edge */
	edges[n].slot = (struct ft_ord_cell **) &succ->lnode.prev;
	edges[n].old_target = pred;
	edges[n].new_target = run_last;
	n++;
	return n;
}

/* Max edges a same-trie run re-splice commits: 2 outer links + 2 back-edges. */
#define FT_ORD_CELL_RUN_RESPLICE_MAX_EDGES	4

/*
 * Pre-sets the run's outer links (run not yet reachable in @dst), flips the
 * <=2 boundary edges atomically (for @dst's live readers), and repairs @dst
 * head/tail.  The run's source trie must already have released it (head/tail
 * cleared + a grace period) so no source reader is mid-run.  This standalone
 * (two-commit) splice runs in the op's failure-free section -- after the source
 * unlink + drain -- so it is UN-ABORTABLE: the caller reserves @txn (capacity >=
 * FT_ORD_CELL_RUN_SPLICE_MAX_EDGES) in its fallible prefix and the commit rides
 * it infallibly via ft_ord_cell_flip_into.
 */
static
void ft_ord_cell_run_splice(struct cds_ft *dst, struct ft_flip_txn *txn,
		struct ft_ord_cell *run_first,
		struct ft_ord_cell *run_last, struct ft_ord_cell *pred,
		struct ft_ord_cell *succ)
{
	struct ft_ord_cell_edge edges[FT_ORD_CELL_RUN_SPLICE_MAX_EDGES] = { 0 };
	unsigned int n = ft_ord_cell_run_splice_edges(dst, run_first, run_last,
		pred, succ, edges, 0);

	/* Bulk op, not yet MW-hardened: ABORT unreachable under its exclusion. */
	(void) ft_ord_cell_flip_into(dst, txn, edges, n);
}

/*
 * Appear-side run-splice fusion descriptor (the dual of struct ft_detach_run):
 * a bulk graft attaches the source trie's whole former ordered-list run
 * [@run_first .. @run_last] between @dst's @pred / @succ neighbours.  Threaded
 * through ft_store_at_graft_point so the run-splice boundary edges join the
 * SAME flip as the structural attach publish -- a reader then never observes a
 * grafted key present in the structure but absent from the ordered list (or
 * vice versa).  @armed reports that the run was fused (so cds_ft_graft skips the
 * standalone two-commit ft_ord_cell_run_splice fallback).
 */
struct ft_graft_run {
	struct ft_ord_cell *run_first, *run_last;	/* src's captured former list */
	struct ft_ord_cell *pred, *succ;		/* dst splice neighbours */
	bool armed;
};

/*
 * Replace the run [@d_first .. @d_last] currently in @dst's ordered list with
 * the run [@s_first .. @s_last] at the SAME position -- the cds_ft_graft_swap
 * shape, where @dst's subtree-at-key (run_D) is swapped out for the swap trie's
 * content (run_S).  @s_first may be NULL (empty swap -> run_D just leaves and the
 * gap closes).  The position is taken from run_D's own neighbours (no relational
 * descent: the swap exchanges two subtrees at the same key, so run_S lands
 * exactly where run_D was).  Atomic for @dst's live readers via one flip of the
 * <=2 boundary edges.  run_D keeps its links for parked readers; the caller
 * re-homes run_D into the swap trie afterwards.
 */
/*
 * Append the <=4 boundary edges that swap run_D [@d_first .. @d_last] out for
 * run_S [@s_first .. @s_last] (at run_D's position) in @dst's ordered list.
 * Split out (mirrors ft_ord_cell_run_splice_edges) so a bulk graft_swap can
 * FUSE these edges with its structural attach publish in ONE flip
 * (ft_ord_cell_flip_rec_replace), closing the sub-key cross-view window;
 * ft_ord_cell_run_replace is the standalone (two-commit) wrapper.  @s_first NULL
 * (empty swap) degrades to run_D removal.
 */
static
unsigned int ft_ord_cell_run_replace_edges(struct cds_ft *dst,
		struct ft_ord_cell *d_first, struct ft_ord_cell *d_last,
		struct ft_ord_cell *s_first, struct ft_ord_cell *s_last,
		struct ft_ord_cell_edge *edges, unsigned int n)
{
	struct ft_ord_cell *pred = ft_ord_cell_resolve_ord(&d_first->lnode.prev);
	struct ft_ord_cell *succ = ft_ord_cell_resolve_ord(&d_last->lnode.next);
	struct ft_ord_cell *new_first = s_first ? s_first : succ;
	struct ft_ord_cell *new_last = s_last ? s_last : pred;

	(void) dst;
	if (s_first) {
		/*
		 * run_S's OUTER links: RECORDED, not plain-stored -- same reason as
		 * ft_ord_cell_run_splice_edges (a plain store does not roll back when
		 * a peer's cell conflict aborts this commit, leaving run_S's back-edge
		 * pointing into @dst permanently).  Expected-old is run_S's pristine
		 * swap-trie boundary, read before anything is flipped.  run_D's own
		 * links are deliberately LEFT alone: parked readers walk off them.
		 */
		struct ft_ord_cell *s_pred =
			ft_ord_cell_resolve_ord(&s_first->lnode.prev);
		struct ft_ord_cell *s_succ =
			ft_ord_cell_resolve_ord(&s_last->lnode.next);

		edges[n].tag = URCU_TXN_TAG;	/* ordered-cell edge */
		edges[n].slot = (struct ft_ord_cell **) &s_first->lnode.prev;
		edges[n].old_target = s_pred;
		edges[n].new_target = pred;
		n++;
		edges[n].tag = URCU_TXN_TAG;	/* ordered-cell edge */
		edges[n].slot = (struct ft_ord_cell **) &s_last->lnode.next;
		edges[n].old_target = s_succ;
		edges[n].new_target = succ;
		n++;
	}
	/*
	 * Sentinel topology: @pred / @succ resolve to @dst's sentinel when run_D is
	 * at @dst's head / tail, so &pred->lnode.next / &succ->lnode.prev IS the old
	 * head / tail repair.  An empty swap (s_first NULL) closes the gap: new_first
	 * = succ, new_last = pred (the sentinel when run_D spanned the whole list).
	 */
	edges[n].tag = URCU_TXN_TAG;	/* ordered-cell edge */
	edges[n].slot = (struct ft_ord_cell **) &pred->lnode.next;
	edges[n].old_target = d_first;
	edges[n].new_target = new_first;
	n++;
	edges[n].tag = URCU_TXN_TAG;	/* ordered-cell edge */
	edges[n].slot = (struct ft_ord_cell **) &succ->lnode.prev;
	edges[n].old_target = d_last;
	edges[n].new_target = new_last;
	n++;
	return n;
}

/* Max edges a run-replace commits: run_S's 2 outer links + 2 back-edges. */
#define FT_ORD_CELL_RUN_REPLACE_MAX_EDGES	4

/*
 * Standalone (two-commit) run-replace: swap run_D out for run_S at run_D's
 * position in @dst's ordered list.  Runs in the op's failure-free section (after
 * the structural commit + drain), so UN-ABORTABLE: the caller reserves @txn
 * (capacity >= FT_ORD_CELL_RUN_REPLACE_MAX_EDGES) in its fallible prefix and the
 * commit rides it infallibly via ft_ord_cell_flip_into.
 */
static
void ft_ord_cell_run_replace(struct cds_ft *dst, struct ft_flip_txn *txn,
		struct ft_ord_cell *d_first, struct ft_ord_cell *d_last,
		struct ft_ord_cell *s_first, struct ft_ord_cell *s_last)
{
	struct ft_ord_cell_edge edges[FT_ORD_CELL_RUN_REPLACE_MAX_EDGES] = { 0 };
	unsigned int n = ft_ord_cell_run_replace_edges(dst, d_first, d_last,
		s_first, s_last, edges, 0);

	/* Bulk op, not yet MW-hardened: ABORT unreachable under its exclusion. */
	(void) ft_ord_cell_flip_into(dst, txn, edges, n);
}

/*
 * Sub-key graft_swap run-replace fusion descriptor (the swap analog of struct
 * ft_graft_run): run_D [@d_first .. @d_last] leaves @dst's ordered list and
 * run_S [@s_first .. @s_last] takes its place.  Threaded through the graft_swap
 * structural publish so the run-replace boundary edges join the SAME flip -- a
 * reader then never observes run_S's keys present in the structure but absent
 * from the ordered list (or run_D the reverse).  @s_first / @s_last NULL =
 * empty swap (run_D just leaves).  @armed reports the run was fused (so the
 * caller skips the standalone two-commit ft_ord_cell_run_replace).
 */
struct ft_graft_swap_run {
	struct ft_ord_cell *d_first, *d_last;	/* run_D (out) */
	struct ft_ord_cell *s_first, *s_last;	/* run_S (in), NULL = empty swap */
	bool armed;
};

/*
 * Max edges a glue-path publish-replace commits: the recorded structural
 * publish stores (struct ft_pub_rec, bounded at 3 by its arrays -- the forward
 * parent slot plus a compressed parent's SKIP_X dual) + the WHOLE run-replace
 * boundary set.  DERIVED from FT_ORD_CELL_RUN_REPLACE_MAX_EDGES rather than
 * spelled out, so it tracks that set automatically: it grew from 2 to 4 when
 * run_S's outer links stopped being plain stores and became recorded edges.
 */
#define FT_GLUE_PUBLISH_REPLACE_MAX_EDGES	\
	(3 + FT_ORD_CELL_RUN_REPLACE_MAX_EDGES)

/*
 * Commit a graft_swap's RECORDED structural publish edges (@rec: the forward
 * parent slot, plus a compressed parent's SKIP_X dual) ATOMICALLY with @run's
 * ordered-list run-replace, in ONE flip through the caller-PRE-RESERVED txn @txn
 * (the legacy KEY_SHORTER graft_swap commit path).  This runs in the op's
 * failure-free section (after the per-side drains), so it cannot fail for want
 * of MEMORY: the caller reserves @txn (capacity >=
 * FT_GLUE_PUBLISH_REPLACE_MAX_EDGES) in the graft_swap fallible prefix.  It can
 * still ABORT on a peer -- the returned status is the caller's cue to re-plan,
 * and @run is armed only when it committed.
 */
static
enum urcu_txn_status ft_ord_cell_flip_rec_replace(struct cds_ft *ft,
		struct ft_flip_txn *txn,
		struct ft_pub_rec *rec, struct ft_graft_swap_run *run)
{
	struct ft_ord_cell_edge edges[FT_GLUE_PUBLISH_REPLACE_MAX_EDGES] = { 0 };
	unsigned int n = 0, i;

	for (i = 0; i < rec->n; i++) {
		edges[n].slot = (struct ft_ord_cell **) rec->slot[i];
		edges[n].old_target = (struct ft_ord_cell *) rec->old_val[i];
		edges[n].new_target = (struct ft_ord_cell *) rec->new_val[i];
		n++;
	}
	n = ft_ord_cell_run_replace_edges(ft, run->d_first, run->d_last,
		run->s_first, run->s_last, edges, n);
	/*
	 * ★ ABORT IS REACHABLE HERE TOO -- see ft_glue_publish.  The "exclusion"
	 * the old comment relied on does not exist on a SHARED destination, which
	 * the public contract explicitly grants.  Arm @run only when the flip
	 * actually committed: an armed run tells the caller the ordered list was
	 * re-homed, and on an abort it was not.
	 */
	{
		enum urcu_txn_status pst =
			ft_ord_cell_flip_into(ft, txn, edges, n);

		if (pst == URCU_TXN_STATUS_OK) {
			run->armed = true;
			FT_GS_PROBE_INC(cds_ft_probe_gs_pubok);
		} else {
			FT_GS_PROBE_INC(cds_ft_probe_gs_pubabort);
		}
		return pst;
	}
}

/*
 * Remove the contiguous run [@first_head .. @last_head] from @ft's ordered list
 * WITHOUT re-homing it -- the cds_ft_merge source side, where the run's cells
 * disperse (survivors are spliced into dst, collided heads are freed).  Relink
 * the two boundary edges (atomic for @ft's live readers); the run cells keep
 * their stale links (caller no longer references them as a run).  Whole-list
 * removal leaves @ft's sentinel pointing at itself (empty).
 */
/* Max edges a run-unlink commits: the two boundary back-edges. */
#define FT_ORD_CELL_RUN_UNLINK_MAX_EDGES	2

#ifdef FEATURE_FT_MERGE
static
void ft_ord_cell_run_unlink(struct cds_ft *ft, struct ft_flip_txn *txn,
		struct cds_ft_node *first_head, struct cds_ft_node *last_head)
{
	struct ft_ord_cell *first =
		ft_ord_cell_ptr(rcu_dereference(first_head->prev));
	struct ft_ord_cell *last =
		ft_ord_cell_ptr(rcu_dereference(last_head->prev));
	struct ft_ord_cell *pred = ft_ord_cell_resolve_ord(&first->lnode.prev);
	struct ft_ord_cell *succ = ft_ord_cell_resolve_ord(&last->lnode.next);
	struct ft_ord_cell_edge edges[FT_ORD_CELL_RUN_UNLINK_MAX_EDGES] = { 0 };
	unsigned int n = 0;

	(void) ft;
	/*
	 * Sentinel topology: @pred / @succ resolve to @ft's sentinel when the run
	 * sits at @ft's head / tail (the whole-list case leaves it self-pointing =
	 * empty).  No separate endpoint edge.
	 */
	edges[n].tag = URCU_TXN_TAG;	/* ordered-cell edge */
	edges[n].slot = (struct ft_ord_cell **) &pred->lnode.next;
	edges[n].old_target = first;
	edges[n].new_target = succ;
	n++;
	edges[n].tag = URCU_TXN_TAG;	/* ordered-cell edge */
	edges[n].slot = (struct ft_ord_cell **) &succ->lnode.prev;
	edges[n].old_target = last;
	edges[n].new_target = pred;
	n++;
	/* Bulk op, not yet MW-hardened: ABORT unreachable under its exclusion. */
	(void) ft_ord_cell_flip_into(ft, txn, edges, n);
}
#endif /* FEATURE_FT_MERGE */

/*
 * Set @child's parent back-pointer to a raw flag @value (a flip-proxy)
 * verbatim, WITHOUT touching parent_slot_offset.  Mirrors ft_set_parent's
 * child-kind dispatch; the settle step later replaces @value with the
 * real parent via ft_set_parent (which does maintain skip_slot).
 */
static
void ft_set_parent_raw(struct cds_ft *ft, struct cds_ft_inode_flag *child,
		struct cds_ft_inode_flag *value)
{
	(void) ft;
	if (!child)
		return;
	/* Flip-proxy child: transient slot value, not a node (see
	 * ft_set_parent); the parking mutator wires the real child. */
	if (caa_unlikely(ft_node_flip_proxy(child)))
		return;
#ifdef FEATURE_FT_SKIP_COMPRESSED
	if (ft_node_skip_compressed(child)) {
		struct cds_ft_compressed_node *cn = ft_skip_to_compressed(ft, child);

		cds_ft_item_to_metadata(
			(struct cds_ft_inode *) cn)->parent_word = value;
		return;
	}
	if (ft_node_compressed(child)) {
		struct cds_ft_compressed_node *cn = ft_compressed_node_ptr(child);

		cds_ft_item_to_metadata(
			(struct cds_ft_inode *) cn)->parent_word = value;
		return;
	}
#endif
	if (ft_node_external(child)) {
		/*
		 * Ordered list on: store the raw flag (a flip-proxy) into the head's
		 * cell->parent, not over its prev (which is the cell pointer).  The
		 * read path resolves cell->parent THEN the flip-proxy, so a proxy
		 * parked in cell->parent settles correctly; the later ft_set_parent
		 * replaces it with the real parent.  List off / non-cell: the head's
		 * prev IS the flagged parent, so store the proxy directly there.
		 */
		if (ft->ordered_list)
			ft_ord_cell_set_parent((struct cds_ft_node *) child, value);
		else
			((struct cds_ft_node *) child)->prev = value;
		return;
	}
	cds_ft_item_to_metadata(ft_node_ptr(child))->parent_word = value;
}

/*
 * ft_flip_txn_record_count_parent: fold the order-statistics count propagation
 * into a flip-txn instead of walking it as a separate post-commit RMW loop (the
 * former ft_propagate_external_count_parent, now retired).  Records a value-CAS
 * edge cur -> cur + (delta << 1) on every node's nr_keys word from @stable_base
 * up to the root, climbing metadata->parent, tagged FT_NR_KEYS_PROXY_TAG so the
 * engine may park an in-band proxy for the duration of the commit (readers resolve
 * it via ft_nr_keys_load).  The count then flips ATOMICALLY with the op's
 * structural edges -- exact under concurrent writers, no drifting aggregate, and
 * the root-ward walk vanishes (the op already holds its descent path).
 *
 * Reader-ordering contract (preserved from the retired walk).  Two reader kinds:
 * pointer-based readers (iteration, key lookup) follow rcu_dereference pointers
 * and never read nr_keys, so they are always structurally consistent.  Count-based
 * readers (lookup_nth, skip, count_keys, count_keys_prefix) read nr_keys to guide
 * descent, using ft_dereference_acquire (CMM_ACQUIRE) for BOTH nr_keys and child
 * pointer loads.  The atomic flip settles a single op's count and pointer in ONE
 * epoch (no split-store window between them); the invariant count-based readers
 * rely on is nr_keys <= actual reachable keys (transient undercount is safe --
 * a reader may briefly miss a boundary key, but must never descend expecting a
 * key that is not there).  The acquire loads still matter for any path the reader
 * observes
 * across a flip boundary (Pattern-3 remove: nr_keys read before the pointer at
 * the same level), so a weakly-ordered CPU cannot see a detached pointer without
 * the paired count decrement.
 *
 * @stable_base MUST be the deepest STABLE existing ancestor whose subtree gains
 * @delta keys: the node that OWNS this op's committed forward slot, or -- when
 * that owner is itself relocated by the same commit -- the owner's stable parent.
 * Its metadata->parent chain is unchanged by the commit, so recording the edges
 * now (pre-commit, off the still-intact chain and current counts) and applying
 * them atomically at the flip is correct.  Fresh cluster nodes BELOW the publish
 * point carry their full post-commit count from build (a plain, build-invisible
 * store) and are never touched here -- this is the property the earlier
 * post-commit-chain attempt lacked (it climbed a re-parented node's stale chain,
 * so some deltas never reached the root: a parity undercount).
 *
 * A no-op when order statistics are off.  The caller must have reserved one txn
 * edge per node on the @stable_base -> root path (bounded by the ACTUAL descent
 * depth, never FT_MAX_DEPTH).  Under the retained writer exclusion no proxy is
 * parked on these ancestors pre-commit, so ft_nr_keys_get reads the committed
 * count for the CAS old value; the new value re-applies the (count << 1) shift.
 */
static
void ft_flip_txn_record_count_parent(struct cds_ft *ft, struct ft_flip_txn *t,
		struct cds_ft_inode_flag *stable_base, long delta)
{
	struct cds_ft_inode_flag *cur = stable_base;

	if (!ft->rank_stats)
		return;
	while (cur) {
		struct cds_ft_metadata *m =
			cds_ft_item_to_metadata(ft_node_ptr(cur));
		unsigned long old_raw = ft_nr_keys_get(m) << 1;
		unsigned long new_raw = (ft_nr_keys_get(m) + delta) << 1;

		/*
		 * The count propagates up UNLOCKED ancestors, so a peer writer
		 * may race a count on the same word: keep these MW so the CAS
		 * detects it and a mixed commit aborts cleanly (re-descend), never
		 * an SW park that would clobber the peer's delta.  Byte-identical
		 * to record_tag while no caller has opted structural_sw in.
		 */
		ft_flip_txn_record_tag_mw(t, (void **) &m->nr_keys,
			(void *) old_raw, (void *) new_raw,
			FT_NR_KEYS_PROXY_TAG);
		cur = ft_parent_node(m->parent_word);
	}
}

/*
 * Structural distinct-key count of the subtree rooted at @node_flag, used when
 * the trie does NOT maintain order statistics (rank stats off) -- there is no
 * per-node nr_keys aggregate to read, so the bulk ops (which size and short-
 * circuit on subtree key counts) and the count queries recompute it on demand.
 * Reproduces exactly the nr_keys invariant cds_ft_verify checks: an internal
 * node contributes one key for its own end-of-path entry (a non-NULL
 * external_nodes) plus the keys in every child subtree; a compressed node
 * contributes its single child subtree, counting one if that child is an
 * external leaf.  Reader-safe -- acquire loads plus the same flip-proxy /
 * skip-compressed resolution the rank readers use, so it never dereferences a
 * parked proxy or a freed node under RCU.  O(subtree size); recursion depth is
 * bounded by the trie depth (<= FT_MAX_DEPTH).  @node_flag must be internal or
 * compressed (an external leaf is one key, counted by the caller).
 */
static
unsigned long ft_subtree_key_count(struct cds_ft *ft,
		struct cds_ft_inode_flag *node_flag)
{
	unsigned long count = 0;
	struct cds_ft_inode_flag *child;
	uint8_t child_key = 0;
	int pivot;

	if (ft_node_compressed(node_flag)) {
		struct cds_ft_compressed_node *cn =
			ft_compressed_node_ptr(node_flag);

		child = ft_cn_child_dereference_acquire_prefetch(cn);
		if (!child)
			return 0;
		if (ft_node_external(child))
			return 1;
		return ft_subtree_key_count(ft, child);
	}

	/* Internal node: the end-of-path key (if present) sorts at this depth. */
	if (ft_dereference_external_acquire(
			cds_ft_item_to_metadata(ft_node_ptr(node_flag))->external_nodes))
		count += 1;

	/* Plus every child subtree, enumerated in ascending ordinal order. */
	pivot = -1;
	child = ft_node_get_direction(ft, node_flag, pivot, &child_key,
			FT_RIGHT, true);
	while (child) {
		struct cds_ft_inode_flag *rchild = child;

#ifdef FEATURE_FT_SKIP_COMPRESSED
		if (caa_unlikely(ft_node_skip_compressed(child))) {
			unsigned int rewind;
			struct cds_ft_inode_flag *at_pos;

			/*
			 * A skip slot's back-pointer and skip_len are transiently
			 * inconsistent while a concurrent split/merge restructures the
			 * path between the slot and the skip child (documented at
			 * ft_skip_to_compressed): the raw ft_resolve_skip_compressed
			 * trusts the back-pointer and would then hand back a node of the
			 * WRONG KIND -- e.g. a mid-split branch internal misread as a
			 * compressed node, its child-presence bitmap word taken for
			 * cn->child and dereferenced (the count-walk SIGSEGV).  This is a
			 * self-consistent reader, so re-anchor on the live structure like
			 * every other concurrent reader (ft_skip_reanchor) and count the
			 * live node AT the encoded position; an accumulator descends INTO
			 * @at_pos for both rewind cases.
			 */
			(void) ft_skip_reanchor(ft, child, &rewind, &at_pos);
			rchild = at_pos;	/* live self-consistent node (may be NULL on a
					   pathological reanchor -- skip that subtree,
					   an undercount is already tolerated here) */
		}
#endif
		if (rchild) {
			if (ft_node_external(rchild))
				count += 1;
			else
				count += ft_subtree_key_count(ft, rchild);
		}
		pivot = child_key;
		child = ft_node_get_direction(ft, node_flag, pivot, &child_key,
				FT_RIGHT, true);
	}
	return count;
}

/*
 * Distinct-key count of a child subtree referenced by @c (possibly skip-
 * encoded): one for an external leaf, the maintained nr_keys aggregate when the
 * trie keeps order statistics, else the structural recount above.  This is the
 * single accessor the bulk ops use wherever they previously read nr_keys for a
 * count that feeds a structural decision (no-op detection, glue / run sizing),
 * so those decisions stay correct on a rank-stats-off trie.
 */
static inline
unsigned long ft_node_key_count(struct cds_ft *ft, struct cds_ft_inode_flag *c)
{
	struct cds_ft_inode_flag *nf = ft_resolve_skip_compressed(ft, c);

	if (ft_node_external(nf))
		return 1;
	if (ft->rank_stats)
		return ft_nr_keys_get(ft_flag_to_metadata(ft, c));
	return ft_subtree_key_count(ft, nf);
}

/*
 * Fill @r with a generous SUPERSET of the nodes ONE ATTEMPT at a bulk op's
 * commit can allocate -- CDS_FT_ALLOC_RESERVE_CAP of every internal node type
 * (its own order + bitmap), the minimal order, and (speculative groups) the
 * compressed-node orders.  Drawn before the op's last fallible step, this lets
 * the commit draw and never fail on an arena allocation, so nothing after that
 * step needs a reader-observable rollback.  Used by the same-trie rekey (before
 * its detach) and by ft_graft_keylen's NOSPLIT attach (before it publishes the
 * empty source root).  A generous superset avoids predicting the exact manifest;
 * a bulk op already pays an RCU grace period, so the handful of throwaway arena
 * pops/pushes is negligible.  Returns 0, or -ENOMEM (caller drains).
 *
 * PER ATTEMPT, NOT PER OP.  Every op that draws from a reserve re-descends and
 * rebuilds on a contention bail, so what keeps a whole retry loop inside this one
 * fill is that an aborted attempt REFUNDS its items (ft_alloc_reserve_refund) --
 * not the size of the fill, which no constant could make sufficient.  Measured
 * depth of a single attempt with refunds in place: 2 of the 8, across the unit
 * suite and every concurrent-writer oracle.
 */
static
int ft_bulk_node_reserve_fill(struct cds_ft *ft, struct cds_ft_alloc_reserve *r)
{
	unsigned int ntypes = (unsigned int) (sizeof(ft_types) / sizeof(ft_types[0]));
	unsigned int i;
	int ret = 0;

	for (i = 0; i < ntypes && !ret; i++) {
		if (ft_types[i].type_class == FT_NULL)
			continue;
		ret = cds_ft_alloc_reserve_add(ft, r, CDS_FT_ALLOC_KIND_NODE,
			ft_types[i].order, ft_types[i].bitmap,
			CDS_FT_ALLOC_RESERVE_CAP);
	}
	/* Minimal order, below ft_types[0] (small / compressed nodes). */
	if (!ret && FT_ALLOC_ORDER_MIN < ft_types[0].order)
		ret = cds_ft_alloc_reserve_add(ft, r, CDS_FT_ALLOC_KIND_NODE,
			FT_ALLOC_ORDER_MIN, FT_NO_BITMAP,
			CDS_FT_ALLOC_RESERVE_CAP);
	/* Compressed-node arena (speculative groups; else compressed == NODE). */
	if (ft->group->speculative) {
		unsigned int order;
		unsigned int cmax = ft_compressed_order(FT_SKIP_LEN_MAX);

		if (cmax > FT_ALLOC_ORDER_MAX)
			cmax = FT_ALLOC_ORDER_MAX;
		for (order = FT_ALLOC_ORDER_MIN; order <= cmax && !ret; order++)
			ret = cds_ft_alloc_reserve_add(ft, r,
				CDS_FT_ALLOC_KIND_COMPRESSED, order,
				FT_NO_BITMAP, CDS_FT_ALLOC_RESERVE_CAP);
	}
	return ret;
}

/*
 * ft_glue: write-path attach-transaction subsystem.  Build a node cluster
 * entirely from fresh, unobservable nodes, then splice it into live data with a
 * single commit-time publish + deferred back-pointer wiring.  Shared by insert
 * (NOSPLIT store), remove, graft and merge -- relocated here (structs from
 * ft-insert.h, helper bodies from ft-graft.h) so the whole subsystem precedes
 * every user and needs no cross-module forward declarations.
 */
/*
 * Write-path attach-transaction glue (build-invisible / publish / reclaim --
 * see the rcu-mutation discipline).  The motivating case: a graft attaches a
 * payload subtrie at a non-root key.  The attach cluster ("glue") is built
 * entirely from fresh, unobservable nodes BEFORE the source root is unlinked,
 * so an allocation failure frees the glue with both tries pristine -- there
 * is nothing to roll back, and no past-sync abort().
 *
 * Every edge from the glue into LIVE data is a back-pointer re-parent
 * that must be deferred to the failure-free commit and applied only
 * after the source is unlinked and a grace period has drained its
 * readers.  Two flavours of live data:
 *   - the displaced dst old-child of a split compressed node, and
 *   - the live payload nodes pulled from the source (its old root, and
 *     any sub-compressed absorbed during canonicalization).
 * Forward edges into live data (a fresh node's child slot pointing at a
 * live node) ARE set during the build: they live in unobservable glue
 * nodes, so no reader follows them until the single commit-time publish.
 *
 * @built tracks every fresh glue node so the abort path can free them
 * (immediate free -- never observed).  @deferred records the live
 * back-pointers to wire at commit.  @free_list records old (replaced)
 * live nodes to reclaim deferred after the publish.
 *
 * The struct is instantiable more than once: cds_ft_graft uses a single
 * glue for the dst-side attach; cds_ft_graft_swap commits two (the
 * dst-side insert glue + the swap-side extracted-root glue) together.
 *
 * Defined up here (rather than with its helper bodies further down)
 * because ft_try_compress_chain, ft_build_branch and the build-only
 * graft split all reference the complete type.
 */
struct ft_glue_deferred_edge {
	struct cds_ft_inode_flag *child;	/* live node to re-parent */
	struct cds_ft_inode_flag *parent;	/* glue node it will point to */
	struct cds_ft_inode_flag **slot;	/* slot in parent holding child */
	/*
	 * cds_ft_merge_at references live subtrees from BOTH tries.  A
	 * src-origin child is drained by the early src unlink (applied at
	 * apply_deferred); a dst-origin child stays reachable via the old dst
	 * spine until the forward publish + dst drain, so its back-pointer flip
	 * must wait (applied at apply_deferred_dst).  graft / graft_swap only
	 * ever re-parent src-origin nodes, so this defaults to false and their
	 * single apply_deferred call still wires every edge.
	 */
	bool dst_origin;
	/*
	 * FOLD: this op holds @child's lock acquire, taken by
	 * ft_glue_acquire_reparent_marks because the commit SW-PARKS @child's
	 * state word.  Released by the re-parent's own state guard edge at the
	 * flip; swept by ft_glue_release_reparent_marks on every path that does
	 * NOT reach a successful commit.  Never set outside structural_sw.
	 */
	bool marked;
	/*
	 * FOLD: this op HOLDS @child's LOCK -- either ft_glue_acquire_
	 * reparent_marks took it (@marked) or another of the op's lock sets
	 * already had it (ft_glue_op_holds: the caller_holder case, where we must
	 * NOT take or release it but DO hold it).  Distinct from @marked, which is
	 * release attribution.  This one picks the re-parent's state-edge KIND;
	 * confusing the two turns the caller-held child's guard MW against a fence
	 * we own = a guaranteed abort on every attempt.
	 */
	bool held_lock;
};

struct ft_glue_free_item {
	void *node;		/* cds_ft_inode * or cds_ft_compressed_node * */
	bool compressed;
	/*
	 * Set false when this commit did NOT perform the node's LIVE->TOMBSTONE
	 * transition -- a peer already retired it (the fuse-list tombstone's RYW
	 * old already had FT_STATE_TOMBSTONE), so THIS commit's tombstone record
	 * was a no-op that did not conflict.  ft_glue_free_old must then leave the
	 * free to the peer that killed it, else the node is double-freed (two
	 * grafts absorbing the same shared node both reach the reclaim).  Default
	 * true: the retiring committer frees it exactly once.
	 */
	bool retired;
	/*
	 * DLM overlap-spine plan-lock (§9.4 M-2): this op holds the node's LOCK
	 * fence, acquired BEFORE its body was read into the merged cluster, and
	 * @snap is the mark's CLEAN pre-mark word.  The freeze then records the
	 * FENCED {LOCK|s -> TOMBSTONE|s} terminal, whose expected-old is exactly
	 * the world the copy plan was derived from -- so any peer state change under
	 * the fence (a fused count, a re-home's pso pair, a foreign tombstone)
	 * mismatches and ABORTS this commit, instead of the plain
	 * {s -> s|TOMBSTONE} upgrade silently retiring a node the peer just grew.
	 *
	 * False = the ordinary unlocked retire (src-side glue, graft until
	 * converted, non-lock_fine): behaviour unchanged.
	 */
	bool fenced;
	uintptr_t snap;
};

/*
 * A deferred duplicate-chain splice, used only by cds_ft_merge_at when the
 * SAME full key exists in both tries: the two LIVE external chains must be
 * concatenated under the fresh merged node @owner.  graft / graft_swap never
 * concatenate two live chains, so this is merge-only.
 *
 * @dst_head is kept as the surviving chain head: its forward owner (a fresh
 * merged node's metadata->external_nodes, or a fresh merged node's child slot)
 * and its back-pointer (@dst_head->prev = that node) are wired by the ordinary
 * Phase-1 set + deferred edge, exactly like any other re-parented external.
 * This struct carries ONLY the concatenation, which is publication-visible on
 * two live chains and is recorded into the bulk merge txn by
 * ft_glue_record_splices (so it flips atomically with the structure),
 * AFTER the source has been detached + drained: the @src_head chain is appended
 * to @dst_head's tail (prev-before-next, the ft_chain_node idiom, but preserving
 * src_head->next so the rest of the src chain rides along).
 */
struct ft_glue_splice {
	struct cds_ft_node *dst_head;		/* surviving head (kept first) */
	struct cds_ft_node *src_head;		/* appended to dst_head's tail */
	/*
	 * @src_head's back-pointer as it stood BEFORE the append demoted it from
	 * a head to a duplicate, and a flag saying the demotion actually ran.
	 *
	 * ft_hlist_append_run_prepare records the forward link tail->next into the
	 * txn but writes run_head->prev = tail as a PLAIN store, on the reasoning
	 * that prev is writer-only and the appended run is unreachable to readers.
	 * Both halves hold for ft_merge_spine_copy, which detaches and drains the
	 * src side first.  Neither holds for the one-decide FOLD, which records the
	 * src detach and the splice into ONE txn: at splice time the src side is
	 * still LIVE, and under SKIP_COMPRESSED its parent slot is skip-encoded
	 * onto this very head -- ft_skip_to_compressed recovers the compressed
	 * node THROUGH prev.  So the store is reader-visible, and on an abort it
	 * is the one mutation the txn cannot roll back: the src slot survives
	 * naming a head whose back-pointer now points into the dst chain, which
	 * cds_ft_verify reports as slen != cn->len and a reader resolves to the
	 * wrong subtree.  ft_glue_abort restores it.
	 *
	 * ARMED UNCONDITIONALLY, and consumed only by ft_glue_abort.  That is the
	 * fail-safe direction and it costs nothing: ft_merge_spine_copy's every
	 * ft_glue_abort on this glue is BEFORE its ft_glue_record_splices call
	 * (ft-merge.h, aborts at 1325..1872 vs the record at 2028, no goto), so the
	 * undo is a verified no-op there.  A per-caller opt-in was tried and
	 * dropped: it was dead code in the only caller that passed false, and it
	 * would silently skip the undo for any bail added between the record and
	 * the commit -- the arming decision must not be frozen at the call site.
	 */
	void *src_prev;
	void *src_demoted_to;
	bool src_demoted;
	/*
	 * The demoted @src_head's ordered-list cell, captured by
	 * ft_glue_record_splices.  It stays REACHABLE through its src-run
	 * neighbours' stale ord_prev/ord_next until the post-publish interleave
	 * rewires them, so it is freed only by
	 * ft_glue_free_collided_cells, called after the interleave, via
	 * the grace-period-deferred cell free.  NULL when the list is off.
	 */
	struct ft_ord_cell *src_cell;
	/*
	 * MW LOCK_FINE, the dup-chain lock-set: the node lock this op holds on
	 * @dst_head's chain HOLDER -- the head's immediate parent, the one node
	 * every chain mutation serialises on (ft_chain_head_holder).  Acquired by
	 * ft_glue_acquire_splice_holders before the merge's point of no return and
	 * released only after the commit that installs the append, so the lock
	 * spans the tail walk, the record AND the install.  @holder_snap is the
	 * mark's clean pre-mark word, kept so the fence can be handed to the
	 * flip-txn as a {LOCK|s -> s} release should the holder turn out to be
	 * the op's publish target (ft_glue_splice_holder_take).
	 *
	 * NULL on the splices that did NOT acquire: a duplicate of an earlier
	 * splice's holder (deduped -- one lock covers every chain under one
	 * holder), a NULL holder, a non-lock_fine trie, or an entry whose fence
	 * has been handed to the txn.  So exactly the non-NULL entries are the
	 * fences ft_glue_release_splice_holders still owns.
	 */
	struct cds_ft_metadata *holder;
	uintptr_t holder_snap;
};

/*
 * Inline floor sizing: a graft / graft_swap attach cluster spans at most a
 * compressed prefix + branch + suffix + a payload path of up to FT_MAX_DEPTH
 * nodes + a canonicalization wrapper, with few deferred edges and freed nodes
 * (old-child; payload top / grandchild; old cn, old src root, absorbed
 * sub-cn).  These fit the inline arrays, so graft / graft_swap never allocate
 * a backing buffer and never grow past the floor.
 *
 * cds_ft_merge_at instead builds a TREE-shaped spine (one deferred edge per
 * disjoint subtree, one free per copied node), which can far exceed the floor.
 * It calls ft_glue_reserve() to move the three arrays onto a malloc'd
 * backing sized by a read-only counting pre-pass; ft_glue_abort() and
 * ft_glue_fini() release it.  Every accessor indexes through the
 * pointers, so the growth is invisible to the helpers.
 */
#define FT_GLUE_FLOOR_BUILT	(2 * FT_MAX_DEPTH + 8)
#define FT_GLUE_FLOOR_DEFERRED	8
#define FT_GLUE_FLOOR_FREE	8
#define FT_GLUE_FLOOR_SPLICE	8

struct ft_glue {
	struct ft_glue_deferred_edge *deferred;
	int nr_deferred;
	int cap_deferred;
	struct ft_glue_free_item *free_list;
	int nr_free;
	int cap_free;
	struct cds_ft_inode_flag **built;
	int nr_built;
	int cap_built;
	struct ft_glue_splice *splices;
	int nr_splices;
	int cap_splices;
	/*
	 * The single forward store that splices the cluster into dst at
	 * commit: parent_slot is swung to top.  publish_parent is the
	 * node flag owning the slot (for compressed skip bookkeeping;
	 * NULL at the root).  The cluster top's own back-pointer into
	 * publish_parent is recorded as an ordinary deferred edge.
	 */
	struct cds_ft_inode_flag *publish_parent;
	struct cds_ft_inode_flag **publish_slot;
	struct cds_ft_inode_flag *top;
	/*
	 * PLAN-SNAPSHOT expected-old for that forward store (@publish_old_set).
	 *
	 * _ft_publish_to_parent's contract for @expected_old is "the value
	 * @parent_slot held in the SNAPSHOT THE CALLER'S PUBLISH PLAN WAS DERIVED
	 * FROM"; the glue committers used to satisfy it with `*g->publish_slot' --
	 * a re-read taken at RECORD time, one instruction before the edge is
	 * handed to the txn.  For a cluster whose plan was decided at the same
	 * moment (an insert's diverge split) those are the same value and the
	 * re-read is exact.  For a REPLACE whose plan names the displaced occupant
	 * -- cds_ft_graft_swap, which extracts that occupant into another trie --
	 * they are not: a peer that swapped the graft point between the descent
	 * and this record makes the re-read report the PEER's content, so the
	 * forward CAS ratifies a world this op never planned against.  It then
	 * publishes over the peer's attach and re-roots its OWN stale occupant
	 * elsewhere.  Setting the plan value here turns exactly that into a commit
	 * ABORT, which a retry-capable caller re-descends on.
	 *
	 * A separate @publish_old_set flag because NULL is a legal slot value (a
	 * publish into an empty slot).  Unset (ft_glue_init default) keeps the
	 * record-time re-read, so every non-graft_swap caller is byte-identical.
	 */
	struct cds_ft_inode_flag *publish_old;
	bool publish_old_set;
	/*
	 * MW LOCK_FINE drop (§11, cross-trie GLUE graft): the node lock on
	 * @publish_parent acquired BEFORE the point-of-no-return src-root swap.
	 * Under the FT-wide-lock drop @publish_parent (a live dst spine node the
	 * diverge cluster splices into, captured at descent) can be RETIRED by a
	 * concurrent peer (a sibling graft growing it, a point-remove recompacting
	 * it) between the descent and this graft's glue commit -- the commit would
	 * then publish into a tombstoned node, a wild store that corrupts the arena
	 * free-list.  Acquiring @publish_parent's node lock pre-swap (bail +
	 * re-descend on a miss, src pristine) makes it un-retirable through the
	 * commit; @publish_parent is a value-swap REPLACE target (body not copied,
	 * nr_child unchanged) so the held {LOCK|s -> s} release at commit expects
	 * an unchanged word.  @publish_parent_holder NULL = not pre-acquired (non-
	 * lock_fine, or a non-GLUE / root-splice publish): the commit takes the
	 * ordinary acquire-or-guard, behaviour-identical to before.
	 */
	struct cds_ft_metadata *publish_parent_holder;
	uintptr_t publish_parent_snap;
	/*
	 * MW LOCK_FINE drop, split-compressed graft: the node lock held on
	 * the compressed divergence node @cn (== d->nf) that this GLUE build
	 * SPLITS and REPLACES.  A graft that diverges inside a compressed node
	 * builds a replacement branch from @cn's descent-time snapshot; under the
	 * drop a concurrent peer (another graft splitting @cn, an insert growing
	 * it) can turn @cn into a wider node between this graft's descent and its
	 * commit, and the commit's forward publish re-reads the slot's CURRENT
	 * occupant as expected-old -- so the CAS succeeds and the stale branch
	 * silently drops the peer's freshly-added children.  Marking @cn LOCK
	 * pre-swap makes it un-growable through the commit (a peer's recompact of
	 * @cn bails); the held {LOCK|s -> TOMBSTONE|s} RETIRE at commit (@cn is
	 * replaced, not edited) flips atomically with the forward publish.  NULL =
	 * not pre-acquired (non-lock_fine, or a NOSPLIT / root-splice publish).
	 */
	struct cds_ft_metadata *split_cn_holder;
	/*
	 * A LOCK this op holds OUTSIDE the glue, for reconciliation ONLY: the
	 * glue reads it in ft_glue_op_holds and never clears it -- the caller that
	 * took it owns its release (one owner per fence).
	 *
	 * Needed because two correct lock sets over one node self-deadlock.  The
	 * rekey driver's FT_GRAFT_PREP_GLUE arm marks the split compressed node's
	 * DISPLACED CHILD before the build (its state word is SW-parked, and an
	 * insert below it CASes nr_child, which @cn's own fence does not exclude),
	 * and ft_split_compressed_graft_build DEFERS that same node -- so
	 * ft_glue_acquire_reparent_marks, which marks every deferred entry, would
	 * fail against our own fence.  Deterministically: the bail returns -EAGAIN,
	 * the retry rebuilds the identical shape, and the op never completes.
	 * Invisible until a fixture gives that child >=2 keys, because a lone key
	 * leaves it EXTERNAL and an external has no state word to mark.
	 */
	struct cds_ft_metadata *caller_holder;
	uintptr_t split_cn_snap;
	/*
	 * Enable the split-retire @cn fence (above) for THIS build.  Set by
	 * cds_ft_graft's ft_graft_keylen (retry_attach) AND by cds_ft_merge_at
	 * (ft-merge.h, `glue.fence_split_cn = true` right after ft_glue_init at
	 * its retry_merge label) -- both have a retry loop that handles the
	 * fence-miss re-descend (FT_GRAFT_PREP_RETRY).
	 *
	 * Both callers must keep that retry: merge_at's PREP_RETRY handling is
	 * NOT dead code, it is what makes the fence safe to enable there.
	 */
	bool fence_split_cn;
	/*
	 * Node whose nr_keys == the grafted payload's key count, and from
	 * whose parent the external-count propagation starts at commit.
	 */
	struct cds_ft_inode_flag *attached_nf;
	/*
	 * Order-statistics count fold (BULK).  When non-zero, the glue committer
	 * records a +count_delta nr_keys walk from @publish_parent up to the root
	 * into @txn, so the count flips ATOMICALLY with the attach forward publish
	 * (the cluster / branch below @publish_parent already carries its full
	 * count from build).  Zero (the ft_glue_init default) for every glue commit
	 * that is not an order-statistics attach, and always a no-op when the trie
	 * does not maintain rank stats.
	 */
	long count_delta;
	/*
	 * Multi-edge flip transaction (<urcu/rcu-txn-sw.h>).  When non-NULL
	 * (the converted graft GLUE path, list off), every live back-pointer
	 * re-parent AND the forward publish are RECORDED into @txn as the build
	 * runs, then committed atomically with one selector flip -- so the
	 * deferred-edge ordering protocol (@deferred + ft_glue_apply_deferred +
	 * fresh-before-live) is bypassed and the whole publish-ordering window
	 * is unrepresentable.  NULL keeps the legacy @deferred path (list on,
	 * NOSPLIT, graft_swap, merge).  The caller owns its lifecycle.
	 */
	struct ft_flip_txn *txn;
	/*
	 * When set, ft_glue_tombstone_free_list records each retired free_list
	 * node's freeze-on-free tombstone INTO @txn (atomic detach, §4.B) instead
	 * of a standalone lone-edge flip, so the freeze commits with the forward
	 * publish that unlinks it.  Set only by the graft-family committers that
	 * (a) route their unlink through @txn and (b) reserved @txn with
	 * FT_GLUE_FLOOR_FREE headroom for the <= cap_free tombstones.  The merge
	 * spine glues (variable cap_free, legacy deferred commit) leave it false
	 * and keep the standalone flip.
	 */
	bool fuse_free_list;
	/*
	 * FOLD (coherent rekey one-decide writer): when set, ft_glue_txn_commit_edges
	 * RECORDS the dst-attach (live back-edges + forward publish + fenced retires +
	 * cells + count) into @txn but does NOT commit it -- @txn is the caller's
	 * SHARED mixed txn and the caller runs the ONE ft_flip_txn_commit that also
	 * carries the src-unlink + S_top COW.  @txn is left intact (the caller owns and
	 * commits it); the LOCK registrations (publish_parent release, split_cn
	 * retire) stay in @txn so the caller's commit consumes them (and an abort
	 * auto-clears them).  False (ft_glue_init default) keeps the self-committing
	 * behaviour, byte-identical for every existing glue caller.
	 */
	bool record_only;
	/*
	 * Inline floor backing.  ft_glue_init points the three arrays
	 * here; graft / graft_swap never outgrow it.  ft_glue_reserve
	 * repoints to a malloc'd buffer when a count would exceed its floor.
	 */
	struct ft_glue_deferred_edge deferred_floor[FT_GLUE_FLOOR_DEFERRED];
	struct ft_glue_free_item free_floor[FT_GLUE_FLOOR_FREE];
	struct cds_ft_inode_flag *built_floor[FT_GLUE_FLOOR_BUILT];
	struct ft_glue_splice splices_floor[FT_GLUE_FLOOR_SPLICE];
};

/*
 * ft_glue helpers.  The struct and the rationale are defined just above;
 * the build-invisible builders that reference the type (ft_build_branch and
 * the graft / merge spine builders) follow in the later mutation modules.
 */
static
void ft_glue_init(struct ft_glue *g)
{
	g->deferred = g->deferred_floor;
	g->nr_deferred = 0;
	g->cap_deferred = FT_GLUE_FLOOR_DEFERRED;
	g->free_list = g->free_floor;
	g->nr_free = 0;
	g->cap_free = FT_GLUE_FLOOR_FREE;
	g->built = g->built_floor;
	g->nr_built = 0;
	g->cap_built = FT_GLUE_FLOOR_BUILT;
	g->splices = g->splices_floor;
	g->nr_splices = 0;
	g->cap_splices = FT_GLUE_FLOOR_SPLICE;
	g->publish_parent = NULL;
	g->publish_slot = NULL;
	g->top = NULL;
	g->publish_old = NULL;
	g->publish_old_set = false;
	g->publish_parent_holder = NULL;
	g->publish_parent_snap = 0;
	g->split_cn_holder = NULL;
	g->caller_holder = NULL;
	g->split_cn_snap = 0;
	g->fence_split_cn = false;
	g->attached_nf = NULL;
	g->txn = NULL;
	g->fuse_free_list = false;
	g->record_only = false;
	g->count_delta = 0;
}

/*
 * Release a glue's malloc'd backing (when it grew past the inline floor) and
 * reset the arrays to the floor, so a second call is a no-op.  Both the abort
 * path (via ft_glue_abort) and the success path call this; graft /
 * graft_swap stay on the floor, so it does nothing for them.
 */
static
void ft_glue_fini(struct ft_glue *g)
{
	if (g->deferred != g->deferred_floor) {
		free(g->deferred);
		g->deferred = g->deferred_floor;
		g->cap_deferred = FT_GLUE_FLOOR_DEFERRED;
	}
	if (g->free_list != g->free_floor) {
		free(g->free_list);
		g->free_list = g->free_floor;
		g->cap_free = FT_GLUE_FLOOR_FREE;
	}
	if (g->built != g->built_floor) {
		free(g->built);
		g->built = g->built_floor;
		g->cap_built = FT_GLUE_FLOOR_BUILT;
	}
	if (g->splices != g->splices_floor) {
		free(g->splices);
		g->splices = g->splices_floor;
		g->cap_splices = FT_GLUE_FLOOR_SPLICE;
	}
}

/*
 * Grow the three arrays onto a malloc'd backing so the build can hold a
 * cluster larger than the inline floor (cds_ft_merge_at's tree-shaped spine).
 * Sizes come from a read-only counting pre-pass; call once, right after init,
 * before any track / defer.  A request at or below a floor leaves that array
 * inline.  Returns 0, or -ENOMEM (whatever already grew is released by a
 * later abort / fini, so the caller need only surface the error).
 */
#ifdef FEATURE_FT_MERGE
static
int ft_glue_reserve(struct ft_glue *g,
		int nr_built, int nr_deferred, int nr_free, int nr_splices)
{
	if (nr_built > g->cap_built) {
		struct cds_ft_inode_flag **p =
			malloc((size_t) nr_built * sizeof(*p));

		if (!p)
			return -ENOMEM;
		memcpy(p, g->built, (size_t) g->nr_built * sizeof(*p));
		if (g->built != g->built_floor)
			free(g->built);
		g->built = p;
		g->cap_built = nr_built;
	}
	if (nr_deferred > g->cap_deferred) {
		struct ft_glue_deferred_edge *p =
			malloc((size_t) nr_deferred * sizeof(*p));

		if (!p)
			return -ENOMEM;
		memcpy(p, g->deferred, (size_t) g->nr_deferred * sizeof(*p));
		if (g->deferred != g->deferred_floor)
			free(g->deferred);
		g->deferred = p;
		g->cap_deferred = nr_deferred;
	}
	if (nr_free > g->cap_free) {
		struct ft_glue_free_item *p =
			malloc((size_t) nr_free * sizeof(*p));

		if (!p)
			return -ENOMEM;
		memcpy(p, g->free_list, (size_t) g->nr_free * sizeof(*p));
		if (g->free_list != g->free_floor)
			free(g->free_list);
		g->free_list = p;
		g->cap_free = nr_free;
	}
	if (nr_splices > g->cap_splices) {
		struct ft_glue_splice *p =
			malloc((size_t) nr_splices * sizeof(*p));

		if (!p)
			return -ENOMEM;
		memcpy(p, g->splices, (size_t) g->nr_splices * sizeof(*p));
		if (g->splices != g->splices_floor)
			free(g->splices);
		g->splices = p;
		g->cap_splices = nr_splices;
	}
	return 0;
}
#endif /* FEATURE_FT_MERGE */

/* Record a fresh glue node so the abort path can free it. */
static
void ft_glue_track(struct ft_glue *g,
		struct cds_ft_inode_flag *nf)
{
	assert(g->nr_built < g->cap_built);
	/*
	 * PLAIN forms only: a SKIP pointer encodes the CHILD's address, so
	 * the abort path's kind dispatch would free the wrong node (the 2.1
	 * corruption shape) and the identity helpers would have to chase the
	 * child's back-pointer mid-build.  Callers track the plain compressed
	 * flag and re-encode separately for the slot publish.
	 */
	assert(!ft_node_skip_compressed(nf));
	g->built[g->nr_built++] = nf;
}

/*
 * Drop a tracked glue node that has been consumed (chain-merge absorbs a
 * freshly-built compressed wrapper and frees it during the build).  Match
 * by underlying node identity so a plain-flag tracking entry is found
 * even when the absorbed reference is skip-encoded.  No-op if absent.
 */
static
void ft_glue_untrack(struct cds_ft *ft, struct ft_glue *g, void *node_ptr)
{
	int i;

	for (i = 0; i < g->nr_built; i++) {
		struct cds_ft_inode_flag *nf = g->built[i];
		void *p;

		if (ft_node_compressed(nf))
			p = ft_compressed_node_ptr(nf);
		else if (ft_node_skip_compressed(nf))
			p = ft_skip_to_compressed(ft, nf);
		else
			p = ft_node_ptr(nf);
		if (p == node_ptr) {
			g->built[i] = g->built[--g->nr_built];
			return;
		}
	}
}

/*
 * Test whether @child names a node currently in the glue's tracked
 * (fresh, unpublished) set.  Match by underlying node identity so a
 * plain-flag tracking entry is found even when @child is the skip-
 * encoded form (or vice versa).
 */
static
bool ft_glue_is_fresh(struct cds_ft *ft, struct ft_glue *g,
		struct cds_ft_inode_flag *child)
{
	void *cp;
	int i;

	if (!child)
		return false;
	if (ft_node_compressed(child))
		cp = ft_compressed_node_ptr(child);
	else if (ft_node_skip_compressed(child))
		cp = ft_skip_to_compressed(ft, child);
	else if (ft_node_external(child))
		return false;	/* externals are never glue-tracked */
	else
		cp = ft_node_ptr(child);
	for (i = 0; i < g->nr_built; i++) {
		struct cds_ft_inode_flag *nf = g->built[i];
		void *bp;

		if (ft_node_compressed(nf))
			bp = ft_compressed_node_ptr(nf);
		else if (ft_node_skip_compressed(nf))
			bp = ft_skip_to_compressed(ft, nf);
		else
			bp = ft_node_ptr(nf);
		if (bp == cp)
			return true;
	}
	return false;
}

/*
 * Wire a back-pointer (@child->parent = @parent, slot @slot) within a
 * build-invisible graft cluster.
 *
 * Two regimes by @child kind, the rule that the user pinned down:
 *
 *   - Internal-to-glue (@child is a freshly-allocated, glue-tracked node):
 *     store IMMEDIATELY.  The child is unpublished, so the rcu_assign_pointer
 *     is invisible to readers; doing it now (rather than deferring) means
 *     the entire intra-cluster back-pointer chain is fully wired by the
 *     time apply_deferred flips any LIVE-data back-pointer into the
 *     cluster.  An up-walk from re-parented live data then sees a coherent
 *     chain from the live edge through the cluster up into publish_parent
 *     -- no transient NULL parents along the way.
 *
 *   - External-to-glue (@child is a LIVE node being re-parented INTO the
 *     glue cluster, e.g. the displaced old child of a diverge split or
 *     the source-payload leaf at the bottom of the new branch): record
 *     the (child, parent, slot) tuple and defer the ft_set_parent until
 *     ft_glue_apply_deferred at commit time, after the source has
 *     been drained.  Flipping a live back-pointer during the build would
 *     be a publication-visible mutation before the cluster is observable.
 *
 *   Deferred entries are de-duplicated by @child so a canonicalization
 *   wrapper later absorbed by a chain-merge keeps only its final mapping.
 */
/*
 * ft_glue_record_back_edge: the flip-txn dual of a deferred back-pointer.
 * Instead of setting @child_nf's parent to @parent_nf now (or deferring it to a
 * post-drain ft_set_parent), RECORD the parent-field transition {old ->
 * @parent_nf} into @txn, so it flips atomically with the forward publish at
 * commit.  Mirrors ft_set_parent's child-kind dispatch exactly for the field
 * address + old value, and runs the SAME writer-only bookkeeping
 * (parent_slot_offset / incoming_byte) EARLY -- safe because it is unobservable
 * in the graft window: parent_slot_offset is read only by writers
 * (ft_get_parent_slot), and incoming_byte is skipped by the reader up-walk while
 * @child_nf's OLD parent is compressed/NULL, which holds for every graft live
 * re-parent (the displaced child's old parent is the compressed node being
 * split; every payload back-edge sits on a node unreachable until the forward
 * flip).
 *
 * @child_nf is LIVE (the caller took the fresh fast path) and never a flip
 * proxy.  List off only (the converted phase): an external head's parent is its
 * prev directly.  Records cannot fail -- the caller reserved @txn to the bounded
 * cluster size up front.
 */
static
void ft_glue_record_back_edge(struct cds_ft *ft, struct ft_flip_txn *txn,
		struct cds_ft_inode_flag *child_nf,
		struct cds_ft_inode_flag *parent_nf,
		struct cds_ft_inode_flag **slot)
{
#ifdef FEATURE_FT_SKIP_COMPRESSED
	if (ft_node_skip_compressed(child_nf)) {
		struct cds_ft_compressed_node *cn =
			ft_skip_to_compressed(ft, child_nf);
		struct cds_ft_metadata *cn_meta =
			cds_ft_item_to_metadata((struct cds_ft_inode *) cn);

		ft_set_parent_slot(cn_meta, parent_nf, slot);
		ft_flip_txn_record_reserved(txn, (void **) &cn_meta->parent_word,
			cn_meta->parent_word, ft_parent_word(ft, parent_nf));
		return;
	}
	if (ft_node_compressed(child_nf)) {
		struct cds_ft_compressed_node *cn =
			ft_compressed_node_ptr(child_nf);
		struct cds_ft_metadata *cn_meta =
			cds_ft_item_to_metadata((struct cds_ft_inode *) cn);

		ft_set_parent_slot(cn_meta, parent_nf, slot);
		ft_flip_txn_record_reserved(txn, (void **) &cn_meta->parent_word,
			cn_meta->parent_word, ft_parent_word(ft, parent_nf));
		return;
	}
#endif
	if (ft_node_external(child_nf)) {
		struct cds_ft_node *en = (struct cds_ft_node *) child_nf;

		/*
		 * Mirror ft_set_parent's external branch for the field address +
		 * old value.  Ordered list on: the head carries its cell in prev
		 * and the parent transition is on cell->parent; list off: en->prev
		 * IS the parent.  Either field flips via @txn so the reader up-walk
		 * (ft_resolve_head_prev) observes old XOR new together with the
		 * forward publish.  Every graft back-edge is src-origin (the
		 * payload, asserted by the caller), so its node is reachable ONLY
		 * through the forward edge -- the parked proxy on this field is
		 * never observed at rest, exactly as the structural back-edges.
		 */
		if (ft->ordered_list) {
			struct ft_ord_cell *cell = ft_ord_cell_ptr(en->prev);

			ft_flip_txn_record_reserved(txn,
				(void **) &cell->parent, cell->parent, parent_nf);
		} else {
			ft_flip_txn_record_reserved(txn, (void **) &en->prev,
				en->prev, parent_nf);
		}
		return;
	}
	{
		struct cds_ft_metadata *meta =
			cds_ft_item_to_metadata(ft_node_ptr(child_nf));

		/*
		 * Mirror ft_set_parent's internal branch: pre-store incoming_byte
		 * under an internal new parent (a node re-homed from a compressed
		 * parent gets its real branch byte), then the offset + idempotent
		 * byte via ft_set_parent_slot.  Both are unobservable now (see the
		 * function header); the parent pointer itself flips via @txn.
		 */
		if (slot && parent_nf && !ft_node_compressed(parent_nf)
#ifdef FEATURE_FT_SKIP_COMPRESSED
				&& !ft_node_skip_compressed(parent_nf)
#endif
		   )
			meta->incoming_byte = ft_slot_to_byte(
				&ft_types[ft_node_type(parent_nf)],
				ft_node_ptr(parent_nf), slot);
		ft_set_parent_slot(meta, parent_nf, slot);
		ft_flip_txn_record_reserved(txn, (void **) &meta->parent_word,
			meta->parent_word, ft_parent_word(ft, parent_nf));
	}
}

/*
 * ft_reparent_record_meta: record a metadata-bearing child's re-home as a
 * co-committed (parent, offset) PAIR into @txn.  The parent pointer flips via a
 * type-7 structural edge and the state-word offset via an FT_STATE_PROXY edge, so
 * a concurrent reader recovering (parent, slot) through ft_resolve_parent_slot
 * observes them atomically -- a plain two-store update tears (the dominant
 * FT_INV_MW crash: parent = new copy, offset = stale index of the old one).  Both
 * edges are ALWAYS recorded (a same-value offset when the slot index is
 * unchanged) so a reader that finds the parent proxy is guaranteed the paired
 * offset proxy -- no snapshot re-spin.  incoming_byte is key-invariant across a
 * recompact, so its plain store is a same-value write, safe before the commit.
 */
static
void ft_reparent_record_meta(struct cds_ft *ft, struct ft_flip_txn *txn,
		struct cds_ft_metadata *meta,
		struct cds_ft_inode_flag *parent_nf,
		struct cds_ft_inode_flag **slot, bool child_marked)
{
	/*
	 * WAITING load, not a raw one -- the same rule the offset word below
	 * obeys, and for the same reason: &meta->state enters THIS txn's write
	 * set a few lines down, so its last load must wait out a parked owner.
	 *
	 * A raw read bakes a peer's parked FT_STATE_PROXY -- a descriptor-record
	 * POINTER -- into @live_state, and the mask below cannot remove it (it
	 * clears TOMBSTONE and LOCK, which a proxy carries in neither).  The edge
	 * recorded from it is a VALIDATE (old == new), so a commit whose install
	 * CAS happens to match PUBLISHES that pointer back into the live word
	 * through settle.  Its owner decided and settled long before, so nothing
	 * ever clears it: the node reads as permanently latched and every later
	 * acquire of it fails forever.  ft_meta_state_transition's wait loop is
	 * the standalone counterpart of this load.
	 */
	uintptr_t old_state = (uintptr_t) urcu_txn_load(txn->mtxn,
		(void **) &meta->state, FT_STATE_PROXY);
	/*
	 * §4.B VALIDATE (Phase 4.3, MW): expect the re-homed child CLEAN-LIVE at
	 * commit.  A RAW old_state bakes a peer's already-set TOMBSTONE/LOCK
	 * into the expected-old, so a child a peer FROZE (retired) between the
	 * recompact copy loop's read (ft_node_recompact) and this sweep would
	 * MATCH at commit and re-home a DEAD child into the fresh copy = a
	 * published edge dangling to reclaimed memory (the MW concurrent-
	 * recompaction UAF).  Masking one-way death (TOMBSTONE) AND the reversible
	 * lock (LOCK) -- identical to ft_flip_txn_guard_parent -- makes
	 * such a child MISMATCH -> ABORT -> retry against the live tree.  A
	 * genuinely live child carries neither bit, so live_state == old_state and
	 * this is a no-op for it.
	 */
	uintptr_t live_state = old_state & ~(FT_STATE_TOMBSTONE | FT_STATE_LOCK);
	bool record_pso = false;
	uintptr_t old_pso = 0, new_pso = 0;

	if (slot) {
		unsigned int off = parent_nf ? (unsigned int) ((char *) slot -
			(char *) ft_node_ptr(parent_nf)) / sizeof(void *) : 0;

		/*
		 * §8.3 split: the offset is its own transacted word, so it is a
		 * SEPARATE recorded edge instead of bits folded into the state
		 * edge.  It still commits in THIS txn alongside &meta->parent, so
		 * the (parent, offset) pair a backtracker recovers remains atomic
		 * -- that pairing always came from the shared commit, never from
		 * the shared word (ft_resolve_parent_slot).  Recorded only when it
		 * actually changes: a same-value edge would be a second touch of a
		 * slot for no reason, and the engine's age-0 fast path refuses to
		 * resolve same-slot coincidence.
		 */
		/*
		 * WAITING load, not a raw one: this slot ends up in THIS txn's
		 * write set, and the read-policy rule is that such a load must
		 * wait out an undecided parker.  A raw read would bake a peer's
		 * parked FT_STATE_PROXY -- a descriptor POINTER -- into the
		 * expected-old, so the install CAS could never match and the
		 * attempt would be a guaranteed abort (or worse, mint a bogus
		 * offset from pointer bits).  urcu_txn_load settles it first.
		 */
		old_pso = (uintptr_t) urcu_txn_load(txn->mtxn,
			(void **) &meta->parent_slot_offset, FT_STATE_PROXY);
		new_pso = FT_PSO_ENCODE(off);
		record_pso = (old_pso != new_pso);
		if (parent_nf && !ft_node_compressed(parent_nf)
#ifdef FEATURE_FT_SKIP_COMPRESSED
				&& !ft_node_skip_compressed(parent_nf)
#endif
		   )
			meta->incoming_byte = ft_slot_to_byte(
				&ft_types[ft_node_type(parent_nf)],
				ft_node_ptr(parent_nf), slot);
	}
	/*
	 * @parent_nf stays the parent NODE above -- the offset and incoming-byte
	 * maths dereferences it -- so only the STORED word takes the owner at a
	 * root position (ft_parent_word).  A re-parent target is a fresh cluster
	 * today, never a root, but the invariant "no parent word is left
	 * anonymous" should not rest on that.
	 */
	ft_flip_txn_record_reserved(txn, (void **) &meta->parent_word,
		meta->parent_word, ft_parent_word(ft, parent_nf));
	/*
	 * The state edge is now a pure {live_state -> live_state} GUARD: it no
	 * longer carries the offset, so its whole job is the §4.B validate the
	 * comment above describes -- expect the re-homed child CLEAN-LIVE at
	 * commit, so a child a peer FROZE mid-recompact MISMATCHES and aborts.
	 */
	/*
	 * EDGE KIND follows who HOLDS @meta's LOCK.  The two disciplines are
	 * mirror images and neither is optional:
	 *
	 *  - HELD (@child_marked -- ft_rekey_cow_stop, ft_glue_acquire_reparent_
	 *    marks): the op owns the lock, so the SW park is safe, and it MUST
	 *    stay SW because an MW edge expecting live_state would mismatch the
	 *    op's OWN mark and abort every commit.  This edge is also what
	 *    RELEASES the mark (live_state masks LOCK out).
	 *  - NOT HELD (the recompact reparent sweep, whose DLM acquire takes
	 *    {C,P,(GP)} and never C's children): under a structural_sw txn
	 *    record_tag would degrade this guard to an UNVALIDATED plain store on
	 *    a word the op does not own -- a contradiction, since the edge's whole
	 *    job is the §4.B validate above, and a park validates nothing.  It
	 *    would also erase whatever a peer put there: a committed nr_child edge
	 *    from an insert BELOW the child (lost count) or a fresh lock acquire
	 *    (stolen lock), neither of which C's lock nor P's excludes.
	 *
	 * Byte-neutral for every non-fold caller: with structural_sw false,
	 * record_tag IS record_tag_mw.
	 *
	 * The alternative -- MARK the sweep's children instead of validating --
	 * was implemented in full and does NOT live: a contended child fails the
	 * acquire, and escalation cannot rescue it (an escalated acquirer holds
	 * its FIFO turn while spinning for a holder funnelled behind that turn).
	 * Validating aborts the COMMIT instead, which is precisely what the
	 * escalation lane arbitrates.
	 */
	if (child_marked)
		ft_flip_txn_record_tag(txn, (void **) &meta->state,
			(void *) live_state, (void *) live_state, FT_STATE_PROXY);
	else
		ft_flip_txn_record_tag_mw(txn, (void **) &meta->state,
			(void *) live_state, (void *) live_state, FT_STATE_PROXY);
	if (record_pso)
		ft_flip_txn_record_tag(txn, (void **) &meta->parent_slot_offset,
			(void *) old_pso, (void *) new_pso, FT_STATE_PROXY);
}

/*
 * Record a LIVE, reader-reachable child's re-home into @txn (Phase 4.3 atomic
 * re-home).  The recompact reparent sweep moves each child of a node from the
 * retiring old copy to the fresh copy; the child stays reachable through the old
 * copy until the forward publish, so its back-pointer update must flip ATOMICALLY
 * with that publish (all recorded into the SAME @txn = @retire_txn) and its
 * (parent, offset) must be a co-committed pair.  Mirrors ft_set_parent's
 * child-kind dispatch exactly; unlike ft_glue_record_back_edge (whose graft
 * children are build-invisible, so their offset store is unobservable) the offset
 * rides @txn beside the parent.  @child_nf is never a flip proxy at a live slot;
 * the guard mirrors ft_set_parent's for reparent sweeps that flow over one.
 */
static
void ft_reparent_record(struct cds_ft *ft, struct ft_flip_txn *txn,
		struct cds_ft_inode_flag *child_nf,
		struct cds_ft_inode_flag *parent_nf,
		struct cds_ft_inode_flag **slot, bool child_marked)
{
	if (!child_nf)
		return;
	if (caa_unlikely(ft_node_flip_proxy(child_nf)))
		return;
#ifdef FEATURE_FT_SKIP_COMPRESSED
	if (ft_node_skip_compressed(child_nf)) {
		struct cds_ft_compressed_node *cn =
			ft_skip_to_compressed(ft, child_nf);

		ft_reparent_record_meta(ft, txn,
			cds_ft_item_to_metadata((struct cds_ft_inode *) cn),
			parent_nf, slot, child_marked);
		return;
	}
	if (ft_node_compressed(child_nf)) {
		struct cds_ft_compressed_node *cn =
			ft_compressed_node_ptr(child_nf);

		ft_reparent_record_meta(ft, txn,
			cds_ft_item_to_metadata((struct cds_ft_inode *) cn),
			parent_nf, slot, child_marked);
		return;
	}
#endif
	if (ft_node_external(child_nf)) {
		struct cds_ft_node *en = (struct cds_ft_node *) child_nf;

		/*
		 * External head: no metadata / offset -- only the parent edge
		 * rides @txn (cell->parent list-on, en->prev list-off), which a
		 * reader resolves via ft_resolve_head_prev.
		 */
		if (ft->ordered_list) {
			struct ft_ord_cell *cell = ft_ord_cell_ptr(en->prev);

			ft_flip_txn_record_reserved(txn, (void **) &cell->parent,
				cell->parent, parent_nf);
		} else {
			ft_flip_txn_record_reserved(txn, (void **) &en->prev,
				en->prev, parent_nf);
		}
		return;
	}
	ft_reparent_record_meta(ft, txn,
		cds_ft_item_to_metadata(ft_node_ptr(child_nf)), parent_nf, slot, child_marked);
}

static
void ft_glue_defer_edge_origin(struct cds_ft *ft, struct ft_glue *g,
		struct cds_ft_inode_flag *child,
		struct cds_ft_inode_flag *parent,
		struct cds_ft_inode_flag **slot,
		bool dst_origin)
{
	int i;

	if (ft_glue_is_fresh(ft, g, child)) {
		ft_set_parent(ft, child, parent, slot);
		return;
	}
	for (i = 0; i < g->nr_deferred; i++) {
		if (g->deferred[i].child == child) {
			g->deferred[i].parent = parent;
			g->deferred[i].slot = slot;
			g->deferred[i].dst_origin = dst_origin;
			return;
		}
	}
	assert(g->nr_deferred < g->cap_deferred);
	g->deferred[g->nr_deferred].child = child;
	g->deferred[g->nr_deferred].parent = parent;
	g->deferred[g->nr_deferred].slot = slot;
	g->deferred[g->nr_deferred].dst_origin = dst_origin;
	/*
	 * Every field of a new entry is set HERE and nowhere else: the backing
	 * arrays are an uninitialised inline floor (or a malloc'd grow), so an
	 * unset @marked is garbage that ft_glue_release_reparent_marks would read
	 * as "we hold this" and clear a fence belonging to nobody.  That is not
	 * hypothetical -- it is what this field did before this line existed, on
	 * EVERY glue abort in every config, because the release sweep runs whether
	 * or not the fold ever acquired.
	 */
	g->deferred[g->nr_deferred].marked = false;
	g->deferred[g->nr_deferred].held_lock = false;
	g->nr_deferred++;
}

static
void ft_glue_defer_edge(struct cds_ft *ft, struct ft_glue *g,
		struct cds_ft_inode_flag *child,
		struct cds_ft_inode_flag *parent,
		struct cds_ft_inode_flag **slot)
{
	ft_glue_defer_edge_origin(ft, g, child, parent, slot,
		/*dst_origin*/ false);
}

/*
 * Record the single forward publish that splices the built cluster into
 * dst, and wire the cluster top's back-pointer into its (live)
 * publish_parent.  The builders call this once the cluster is fully
 * built.  top is fresh and unpublished, so ft_glue_defer_edge
 * stores top->parent IMMEDIATELY (its fresh-child fast path); the store
 * lands before any other commit-time mutation, so by the time
 * apply_deferred flips any live back-pointer into the cluster, the
 * up-walk path from cluster nodes through top into publish_parent is
 * already wired.
 */
static
void ft_glue_set_publish(struct cds_ft *ft, struct ft_glue *g,
		struct cds_ft_inode_flag *parent_nf,
		struct cds_ft_inode_flag **parent_slot,
		struct cds_ft_inode_flag *top)
{
	g->publish_parent = parent_nf;
	g->publish_slot = parent_slot;
	g->top = top;
	ft_glue_defer_edge(ft, g, top, parent_nf, parent_slot);
}

/*
 * The forward publish's expected-old: the PLAN-snapshot value when the builder
 * recorded one (ft_glue_set_publish_old), else the record-time re-read every
 * caller used before that hook existed.  See @publish_old at the struct.
 */
static
struct cds_ft_inode_flag *ft_glue_publish_expected_old(const struct ft_glue *g)
{
	if (g->publish_old_set)
		return g->publish_old;
	/* SETTLED, not raw: see ft_graft_swap_settle -- a parked engine proxy
	 * is a marker, never a value the slot holds. */
	return ft_resolve_flip_proxy(*g->publish_slot);
}

/*
 * ft_glue_set_publish for a REPLACE whose plan named the slot's DISPLACED
 * occupant: record @old as the forward edge's expected-old so the commit
 * ratifies the world the plan was derived from.  @old is the RAW slot value
 * read at descent (a SKIP_X form included) -- not a resolved flag, since it is
 * compared against the slot itself.
 */
static
void ft_glue_set_publish_old(struct cds_ft *ft, struct ft_glue *g,
		struct cds_ft_inode_flag *parent_nf,
		struct cds_ft_inode_flag **parent_slot,
		struct cds_ft_inode_flag *top,
		struct cds_ft_inode_flag *old)
{
	ft_glue_set_publish(ft, g, parent_nf, parent_slot, top);
	g->publish_old = old;
	g->publish_old_set = true;
}

/* Record an old (replaced) live node to reclaim deferred at commit. */
static
void ft_glue_defer_free(struct ft_glue *g,
		void *node, bool compressed)
{
	assert(g->nr_free < g->cap_free);
	g->free_list[g->nr_free].node = node;
	g->free_list[g->nr_free].compressed = compressed;
	g->free_list[g->nr_free].retired = true;
	g->free_list[g->nr_free].fenced = false;
	g->free_list[g->nr_free].snap = 0;
	g->nr_free++;
}

/*
 * DLM overlap-spine plan-lock (§9.4 M-2): record a replaced node whose LOCK
 * fence the caller ALREADY acquired -- before copying its content into the merged
 * cluster -- stashing the mark's clean @snap so the freeze can record the fenced
 * {LOCK|s -> TOMBSTONE|s} terminal.  The mark is owned by the op and released
 * by ft_glue_clear_fenced on any non-committing exit.
 *
 * WHY THIS EXISTS.  The plain retire (above) records {s -> s|TOMBSTONE} with an
 * expected-old read fresh at commit time, and takes NO lock over the window in
 * which the merge READ the node's body into the fresh cluster.  A peer that adds
 * a child in that window is not detected: the late expected-old matches, the
 * commit succeeds, and the peer's child is retired with the node -- silent key
 * loss.  This is the "unlocked retire" gap, distinct from the lock_or_guard
 * publish sites, which already abort on a peer touch.
 */
static
void ft_glue_defer_free_fenced(struct ft_glue *g,
		void *node, bool compressed, uintptr_t snap)
{
	assert(g->nr_free < g->cap_free);
	g->free_list[g->nr_free].node = node;
	g->free_list[g->nr_free].compressed = compressed;
	g->free_list[g->nr_free].retired = true;
	g->free_list[g->nr_free].fenced = true;
	g->free_list[g->nr_free].snap = snap;
	g->nr_free++;
}

/*
 * Release every node lock this glue's overlap-spine plan-lock still holds.
 * Call at the op's terminal on BOTH the committing and the aborting path:
 * ft_meta_lock_release_if_held no-ops on an entry whose fenced retire the commit
 * already turned TOMBSTONE, and releases one left {LOCK|s} by an abort -- so no
 * per-outcome bookkeeping is needed, which is the same reason the detach's orphan
 * chain sweeps unconditionally.
 *
 * The nodes are still addressable at both call points: on success they are
 * call_rcu-deferred (the writer's own read-side section holds the grace period
 * off), on abort they stay live and linked.  No-op for a glue that never fenced
 * (src side, graft until converted, non-lock_fine).
 */
/*
 * Does this glue's overlap-spine plan-lock ALREADY hold @meta's node lock?
 *
 * The dup-chain splice acquire needs this: a collided key's chain hangs off a
 * dst overlap node, which is exactly the node ft_merge_build fences before
 * copying it.  Re-marking a word the SAME op already fenced returns -EAGAIN
 * forever, and the caller's re-descend then rebuilds into the identical state --
 * a self-deadlock across two individually-correct lock sets.  The fence is the
 * stronger exclusion (it also makes the node un-retirable by a peer) and covers
 * exactly what the chain append needs, so the splice reuses it instead of taking
 * a second lock on the same word.
 */
static
bool ft_glue_fence_holds(const struct ft_glue *g,
		const struct cds_ft_metadata *meta)
{
	int i;

	for (i = 0; i < g->nr_free; i++) {
		if (!g->free_list[i].fenced)
			continue;
		if (cds_ft_item_to_metadata((struct cds_ft_inode *)
				g->free_list[i].node) == meta)
			return true;
	}
	return false;
}

/*
 * Renounce the free of every fenced overlap node: call when the commit did NOT
 * report OK.  A fenced retire's LIVE->TOMBSTONE transition is atomic with the
 * flip, so if the flip did not happen this op performed no retire and owns no
 * free -- and something else may: the dominant reason a fenced terminal aborts
 * is that a PEER retired the node under our fence (the retire primitives do not
 * honour LOCK), and that peer owns the reclaim.  Freeing here would be a
 * double free.
 *
 * Separate from ft_glue_clear_fenced, which releases the FENCE; this releases
 * the claim on the MEMORY.  Both run on the aborting path.
 */
static
void ft_glue_fenced_renounce_free(struct ft_glue *g)
{
	int i;

	for (i = 0; i < g->nr_free; i++)
		if (g->free_list[i].fenced)
			g->free_list[i].retired = false;
}

static
void ft_glue_clear_fenced(struct ft_glue *g)
{
	int i;

	for (i = 0; i < g->nr_free; i++) {
		if (!g->free_list[i].fenced)
			continue;
		ft_meta_lock_release_if_held(cds_ft_item_to_metadata(
			(struct cds_ft_inode *) g->free_list[i].node));
		g->free_list[i].fenced = false;
	}
}

/*
 * The state-word-bearing metadata a deferred re-parent of @child_nf will PARK
 * into, or NULL when there is none.  Mirrors ft_reparent_record's dispatch
 * exactly -- that is the point: this decides which children get a mark, and it
 * must agree edge-for-edge with what actually records, or a mark is taken that
 * nothing releases (leak) or an edge parks unmarked (clobber).
 *
 * NULL for an EXTERNAL head (ft_reparent_record records only its parent edge --
 * no state word, so nothing to exclude and nothing to release) and for a flip
 * proxy (which ft_reparent_record skips outright).
 */
static
struct cds_ft_metadata *ft_glue_reparent_park_meta(struct cds_ft *ft,
		struct cds_ft_inode_flag *child_nf)
{
	if (!child_nf || ft_node_flip_proxy(child_nf))
		return NULL;
#ifdef FEATURE_FT_SKIP_COMPRESSED
	/*
	 * ☠ SKIP BEFORE EXTERNAL, and the order is the whole correctness of this
	 * function.  A skip pointer whose target is an external leaf has low tag
	 * bits == 0, so ft_node_external MATCHES it on the raw value -- the trap
	 * ft_set_parent and ft_reparent_record both call out and both order around.
	 * Testing external first therefore reports "no state word" for a node that
	 * has one, its mark is never taken, and the commit SW-parks that word
	 * unheld: the clobber this whole acquire exists to prevent.
	 *
	 * Measured, because it got this wrong first: with external tested first the
	 * occupied-dst merge oracle found ZERO mark candidates over 4126 merges,
	 * while the empty-dst oracle found 1883 -- the merge's children are
	 * skip-compressed onto leaves, the graft's are plain internals, so only a
	 * COLLIDING/merge shape exposes it.
	 */
	if (ft_node_skip_compressed(child_nf))
		return cds_ft_item_to_metadata((struct cds_ft_inode *)
			ft_skip_to_compressed(ft, child_nf));
	if (ft_node_compressed(child_nf))
		return cds_ft_item_to_metadata((struct cds_ft_inode *)
			ft_compressed_node_ptr(child_nf));
#endif
	if (ft_node_external(child_nf))
		return NULL;
	(void) ft;
	return cds_ft_item_to_metadata(ft_node_ptr(child_nf));
}

/*
 * Does this op ALREADY hold @meta's LOCK through one of the glue's other
 * lock sets?  Re-marking a word we hold is -EAGAIN against our own fence, and
 * the caller's re-descend then rebuilds the identical state forever -- the
 * self-deadlock 3a hit between the overlap fence and the dup-chain splice
 * acquire, whose backtrace pointed at an innocent function.
 *
 * Every set is checked, not the ones that look reachable: the whole lesson of
 * that bug is that two individually-correct lock sets met on a node neither
 * author expected (a collided key's chain hangs off a dst overlap node, so they
 * were the SAME node).  These arrays are small and this runs once per commit.
 */
static
bool ft_glue_op_holds(const struct ft_glue *g,
		const struct cds_ft_metadata *meta)
{
	int i;

	if (g->publish_parent_holder == meta || g->split_cn_holder == meta ||
			g->caller_holder == meta)
		return true;
	if (ft_glue_fence_holds(g, meta))
		return true;
	for (i = 0; i < g->nr_splices; i++)
		if (g->splices[i].holder == meta)
			return true;
	return false;
}

/*
 * Acquire the lock acquire on every LIVE child this commit will re-parent.
 *
 * WHY.  Under structural_sw a re-parent RECORD is an SW park -- a plain store
 * that never validates -- and ft_reparent_record_meta parks the child's STATE
 * word (its {live_state -> live_state} §4.B guard).  That word is CASed by
 * ft_meta_nr_child_inc from an insert BELOW the child, a peer that neither this
 * op's locks nor the graft's exclude.  The mark is what makes that peer honour
 * FT_STATE_INPLACE_WAIT_MASK and spin until this commit settles the word,
 * instead of clobbering the park.  This is ft_rekey_cow_stop's discipline,
 * which the glue re-parent path did not have.
 *
 * ONLY under structural_sw.  Under MW the very same guard edge expects
 * live_state as its expected-old and would MISMATCH our own mark -- a
 * guaranteed abort on every commit.  Measured, and it is why this is gated
 * rather than made unconditional.
 *
 * Returns -EAGAIN on a contended child; the caller aborts and re-descends, and
 * ft_glue_abort releases whatever was taken before the miss.
 */
static
int ft_glue_acquire_reparent_marks(struct cds_ft *ft, struct ft_glue *g)
{
	int i, j;

	if (!g->txn || !g->txn->structural_sw)
		return 0;
	for (i = 0; i < g->nr_deferred; i++) {
		struct cds_ft_metadata *cm =
			ft_glue_reparent_park_meta(ft, g->deferred[i].child);
		uintptr_t snap;
		bool dup = false;

		g->deferred[i].marked = false;
		g->deferred[i].held_lock = false;
		if (!cm)
			continue;
		if (ft_glue_op_holds(g, cm)) {
			/* Held via another lock set: do not re-mark, do not
			 * release -- but DO record that we hold it. */
			g->deferred[i].held_lock = true;
			continue;
		}
		/*
		 * Two deferred entries can resolve to ONE metadata (a skip flag
		 * and the plain flag of the same compressed node), and the
		 * dedup in ft_glue_defer_edge_origin keys on the FLAG, so it
		 * does not catch that.  A second mark on our own word is the
		 * same self-deadlock as above.
		 */
		for (j = 0; j < i; j++)
			if (g->deferred[j].marked &&
					ft_glue_reparent_park_meta(ft,
						g->deferred[j].child) == cm)
				dup = true;
		if (dup) {
			/* Same word, marked via the earlier entry: we hold it. */
			g->deferred[i].held_lock = true;
			continue;
		}
#ifdef FEATURE_FT_FAULT_INJECT
		/*
		 * Test-only: miss this acquire exactly as a peer holding the child
		 * would.  Without it the bail AND the release sweep below are DEAD
		 * CODE -- measured 1883 marks taken and 0 released across the whole
		 * FT_INV_MW oracle plan, because nothing in the suite contends a
		 * re-parented child.  That dead sweep is where an uninitialised
		 * @marked hid, so it gets its own armed test rather than inherited
		 * confidence.  Needs FEATURE_FT_FAULT_INJECT *and*
		 * fault injection in one build: the gate's `fault-audit`.
		 */
		if (cds_ft_fault_lock_countdown >= 0) {
			if (cds_ft_fault_lock_countdown == 0) {
				cds_ft_fault_lock_countdown = -1;
				return -EAGAIN;
			}
			cds_ft_fault_lock_countdown--;
		}
#endif
		if (ft_meta_lock_acquire(cm, &snap))
			return -EAGAIN;
		g->deferred[i].marked = true;
		g->deferred[i].held_lock = true;
	}
	return 0;
}

/*
 * Release every re-parent mark still held.  Call ONLY on paths that did NOT
 * reach a successful commit.
 *
 * ☠ NOT the unconditional post-op sweep ft_glue_clear_fenced and the orphan
 * chain use, and the difference is load-bearing.  Those sweep RETIRED nodes: a
 * consumed fence leaves the word TOMBSTONE, which no peer will ever re-mark, so
 * "clear it if it is set" is unambiguous.  A re-parented child SURVIVES, and its
 * guard edge releases it to LIVE-and-CLEAN -- immediately re-markable by a peer.
 * Sweeping after a successful commit would therefore race a peer's fresh mark
 * and silently clear it, breaking an exclusion this op does not own.  (The
 * driver already reasons exactly this way about publish_parent_holder: "once
 * ft_glue_txn_commit_edges has run, the txn registry owns it and clearing here
 * would race a peer's re-mark.")
 *
 * On a commit that did not happen, no guard edge landed, so every mark is still
 * ours and clearing is unambiguous.  Idempotent.
 */
static
void ft_glue_release_reparent_marks(struct cds_ft *ft, struct ft_glue *g)
{
	int i;

	for (i = 0; i < g->nr_deferred; i++) {
		struct cds_ft_metadata *cm;

		if (!g->deferred[i].marked)
			continue;
		cm = ft_glue_reparent_park_meta(ft, g->deferred[i].child);
		if (cm) {
			ft_meta_lock_release(cm);
		}
		g->deferred[i].marked = false;
	}
}

/*
 * Drop every dup-chain holder lock ft_glue_acquire_splice_holders still owns
 * (the non-NULL @holder entries), leaving those nodes LIVE.  Idempotent: each
 * released entry is cleared, so the pre-commit bail (ft_glue_abort) and the
 * post-commit release cannot double-clear, and a glue that never acquired --
 * graft, graft_swap, a non-lock_fine trie, a collision-free merge -- costs one
 * loop over an empty splice array.
 *
 * The fences are NOT in the flip-txn registry (see the acquire), so nothing
 * else can clear them and no peer can have re-taken one: ft_meta_lock_acquire
 * refuses an already-locked word, and the only transition the commit records
 * on a held holder is the free-list retire's plain
 * {LOCK|s -> LOCK|s|TOMBSTONE} upgrade, which PRESERVES the fence.  So
 * the bit is still ours at every release point -- the clear is unambiguous, on
 * a node that is either about to be reclaimed (retired) or still live.
 */

static
void ft_glue_release_splice_holders(struct ft_glue *g)
{
	int i;

	for (i = 0; i < g->nr_splices; i++) {
		if (!g->splices[i].holder)
			continue;
		ft_meta_lock_release(g->splices[i].holder);
		g->splices[i].holder = NULL;
		g->splices[i].holder_snap = 0;
	}
}

/*
 * Hand a held dup-chain holder fence to the caller when @meta is one of them,
 * clearing the entry so ft_glue_release_splice_holders no longer owns it.
 *
 * The one caller is the merge's forward publish: when the dst merge point is an
 * EXTERNAL node, the chain holder of the spliced head IS the publish target, and
 * ft_flip_txn_lock_or_guard_parent would then re-mark a word we already hold --
 * a MISS against our own fence, which sets acquire_miss and ABORTS a commit that
 * has no bail path left.  Routing the held fence into
 * ft_flip_txn_hold_or_lock_parent instead records the {LOCK|s -> s} release
 * (the guard's strictly stronger twin) and hands the outcome to the txn, exactly
 * as the graft's publish_parent_holder does.
 */
static
bool ft_glue_splice_holder_take(struct ft_glue *g,
		const struct cds_ft_metadata *meta, uintptr_t *snap_ret)
{
	int i;

	for (i = 0; i < g->nr_splices; i++) {
		if (g->splices[i].holder != meta)
			continue;
		*snap_ret = g->splices[i].holder_snap;
		g->splices[i].holder = NULL;
		g->splices[i].holder_snap = 0;
		return true;
	}
	return false;
}

/*
 * Abort the build: free every freshly-built (never-observed) glue node, then
 * release the malloc'd backing.  Both tries are left pristine -- no deferred
 * edge was applied, so no live back-pointer references the glue.
 */
static
void ft_glue_abort(struct cds_ft *ft, struct ft_glue *g)
{
	int i;

	/*
	 * Demoted src heads (merge, collided keys): undo the ONE mutation the txn
	 * does not carry.  ft_hlist_append_run_prepare stores run_head->prev =
	 * tail plainly while recording only the forward tail->next edge, so an
	 * aborted commit rolls back the forward link and leaves the back-pointer
	 * re-homed into the dst chain -- with the src's skip-encoded slot still
	 * naming this head, which is how a reader (and cds_ft_verify) finds the
	 * wrong compressed node.  Restoring makes the abort byte-for-byte again.
	 *
	 * FIRST, while the dup-chain holder locks this op took are still HELD:
	 * ft_glue_release_splice_holders below drops them, and a chain slot must
	 * not be written after its holder's lock is gone.
	 *
	 * A committed merge must not reach here (same contract as the re-parent
	 * marks below), so the demotion it made is never undone.
	 */
	for (i = 0; i < g->nr_splices; i++) {
		if (!g->splices[i].src_demoted)
			continue;
		/*
		 * CAS, not a store: undo OUR write and only while it is still ours.
		 * ft_glue_acquire_splice_holders locks the DST head's chain holder
		 * only (ft_chain_head_holder(ft, dst_head)); the SRC head's holder is
		 * never acquired, so between the append and here a peer may
		 * legitimately retarget src_head->prev -- a src-side recompact, or a
		 * head promote swapping a fresh cell in.  A blind store would then
		 * install our STALE snapshot over the peer's current value, which is
		 * the same wrong-back-pointer corruption this undo exists to prevent,
		 * merely pointing the other way.  If the CAS fails the peer owns the
		 * field now and its value must stand.
		 */
		(void) uatomic_cmpxchg(&g->splices[i].src_head->prev,
			g->splices[i].src_demoted_to, g->splices[i].src_prev);
		g->splices[i].src_demoted = false;
	}

	/*
	 * Dup-chain holder locks (MW LOCK_FINE): a merge that acquired its
	 * splice lock-set and then bailed before the point of no return must
	 * leave those holders LIVE and unlocked.  Every pre-commit merge bail
	 * routes through here, so this is the single clear point, mirroring the
	 * split-retire fence below.  No-op when nothing was acquired.
	 */
	ft_glue_release_splice_holders(g);
	/*
	 * DLM overlap-spine plan-lock (§9.4 M-2): release every node lock this
	 * aborted build took on a dst overlap node.  Every pre-commit merge bail
	 * routes through here, so this is their single clear point -- the old nodes
	 * stay LIVE and unmarked, the trie byte-for-byte as before.  No-op for a
	 * glue that never fenced.
	 */
	ft_glue_clear_fenced(g);
	/*
	 * FOLD: same argument for the re-parent marks.  This is the choke point for
	 * every path that reaches here without a successful commit -- pre-commit
	 * bails AND the fold driver's abort branch, which calls ft_glue_abort before
	 * ft_glue_fini frees the array these marks are recorded in.  A commit that
	 * SUCCEEDED must not come through here: its guard edges already released
	 * every mark to LIVE-and-CLEAN, and re-clearing would race a peer's re-mark
	 * (see ft_glue_release_reparent_marks).
	 */
	ft_glue_release_reparent_marks(ft, g);

	/*
	 * Split-retire cn fence (MW LOCK_FINE drop): a GLUE graft build that
	 * marked the compressed divergence node @cn's node lock
	 * (ft_split_compressed_graft_build) and then aborts BEFORE the commit
	 * registered it with g->txn must release the fence so @cn stays LIVE.
	 * Every pre-commit graft bail routes through here, so this is the single
	 * clear point.  Idempotent / no-op when unset (NOSPLIT, non-lock_fine,
	 * non-graft callers -- ft_glue_init defaults it NULL -- or already
	 * consumed by a committed retire, which does not reach ft_glue_abort).
	 */
	if (g->split_cn_holder) {
		ft_meta_lock_release(g->split_cn_holder);
		g->split_cn_holder = NULL;
		g->split_cn_snap = 0;
	}
	/*
	 * Publish-target fence (merge: pre-acquired BEFORE the src unlink so the
	 * forward publish can never miss and abort a commit past the point of no
	 * return).  Bails AFTER that acquire exist -- the src unlink's own OOM is
	 * one -- so the release belongs at this choke point, not at the call sites
	 * that happen to be visible when the acquire is written.
	 *
	 * clear_IF_HELD, and NULL it: a caller that already released the fence
	 * itself (ft_graft_keylen clears pp_meta on its retry_attach path) or that
	 * handed ownership to its txn (which NULLs the field) must not be
	 * double-cleared -- ft_meta_lock_release asserts the bit is still set.
	 */
	if (g->publish_parent_holder) {
		ft_meta_lock_release_if_held(g->publish_parent_holder);
		g->publish_parent_holder = NULL;
		g->publish_parent_snap = 0;
	}
	for (i = 0; i < g->nr_built; i++) {
		struct cds_ft_inode_flag *nf = g->built[i];

		if (ft_node_compressed(nf))
			free_compressed_node_unpublished(ft,
				ft_compressed_node_ptr(nf));
		else if (ft_node_skip_compressed(nf))
			free_compressed_node_unpublished(ft,
				ft_skip_to_compressed(ft, nf));
		else
			free_cds_ft_node_unpublished(ft, ft_node_ptr(nf));
	}
	ft_glue_fini(g);
}

/*
 * Commit step 1: wire the deferred LIVE back-pointers.  Call after the
 * source unlink + grace period, before the forward publish of the
 * cluster top, so an up-walk from any re-parented live node enters the
 * new cluster before the old nodes are forward-detached and freed.
 *
 * Only edges whose CHILD is a live (observable) node are deferred --
 * setting a live node's parent is a publication-visible mutation that
 * must wait for sync_rcu.  Fresh-to-fresh edges inside the cluster are
 * set IMMEDIATELY during the build (the child is unpublished, the store
 * has no reader-visible effect, and by commit time the entire internal
 * chain from any live re-parent target up to publish_parent is already
 * in place).  Iteration order here therefore does not matter: each live
 * back-pointer flip lands on an already-fully-wired cluster.
 */
/*
 * Freeze-on-free (doc §4.B): set the one-way LIVE->DEAD tombstone on every old
 * (replaced) node a glue commit retires -- the g->free_list set later drained by
 * ft_glue_free_old.  Call on the committing path, BEFORE the publish/unlink that
 * detaches them (apply_deferred for the standard forward-publish glues; the merge
 * src side stamps gs before its own src-root/run unlink).  Idempotent (one-way
 * mark) and a no-op store under one writer.
 */
static
void ft_glue_tombstone_free_list(struct ft_glue *g)
{
	int i;

	for (i = 0; i < g->nr_free; i++) {
		struct cds_ft_metadata *meta = cds_ft_item_to_metadata(
			(struct cds_ft_inode *) g->free_list[i].node);

		/*
		 * MW LOCK_FINE drop (split-compressed graft): the fenced
		 * divergence node @cn is retired through its held node lock
		 * -- ft_glue_txn_commit_edges records its {LOCK|s ->
		 * TOMBSTONE|s} terminal.  A SECOND plain tombstone here would
		 * DOUBLE-record @cn's state word (guard/retire-then-release =
		 * permanent poison: every commit aborts), so skip it -- @cn stays
		 * on the free list (retired) for reclaim, its LIVE->DEAD
		 * transition owned by the fenced retire.
		 */
		if (g->split_cn_holder == meta)
			continue;
		/*
		 * DLM overlap-spine plan-lock: this node's node lock is HELD, and
		 * its snap is the clean word the copy plan was derived from.  Record
		 * the FENCED {LOCK|s -> TOMBSTONE|s} terminal, which ratifies
		 * exactly that world -- a peer state change under the fence mismatches
		 * and aborts us.  Deliberately NOT the RYW plain upgrade: that reads
		 * the word fresh at commit and so cannot tell "unchanged" from
		 * "changed and changed back into a shape that happens to match".
		 *
		 * ★ THE FENCE DOES NOT STOP A PEER RETIRE, so the
		 * peer-already-tombstoned arm below applies HERE TOO.  Neither retire
		 * primitive honours LOCK: ft_meta_tombstone_set_flip waits on
		 * FT_STATE_PROXY only (documented as deliberate at the mask's
		 * definition -- a tombstone is a self-contained one-way mark), and
		 * ft_flip_txn_record_tombstone takes its expected-old from the
		 * COMMITTED word, our LOCK bit included, so its CAS matches and
		 * lands {s|LOCK -> s|LOCK|TOMBSTONE}.  The fence excludes
		 * LOCK-respecting peers; it does not exclude these.
		 *
		 * So keep the double-free guard.  Without it the sequence is: peer
		 * retires under our fence and takes ownership of the free; our fenced
		 * terminal's expected-old mismatches, so the commit ABORTS; and we
		 * free the node anyway because @retired was left true -- a double free
		 * on top of a lost merge.  (Measured: the plain path absorbs the peer
		 * tombstone as a no-op upgrade and correctly declines the free, which
		 * is the protection project_ft_barrier_uaf_is_graft_double_free
		 * installed.  The fenced path must not lose it.)
		 *
		 * The RYW load only closes the window up to THIS point; the abort path
		 * below covers a peer that retires between here and the commit, by
		 * declining the free for every fenced entry when the commit does not
		 * report OK -- if the flip did not happen, this op did not perform the
		 * LIVE->TOMBSTONE transition and owns no free.
		 */
		if (g->free_list[i].fenced) {
			uintptr_t cur;

			assert(g->fuse_free_list);
			cur = (uintptr_t) urcu_txn_load(g->txn->mtxn,
				(void **) &meta->state, FT_STATE_PROXY);
			if (cur & FT_STATE_TOMBSTONE)
				g->free_list[i].retired = false;
			ft_flip_txn_record_tombstone_locked(g->txn, meta,
				g->free_list[i].snap);
			continue;
		}
		/*
		 * Fuse the freeze into @txn (committed with the forward publish
		 * below) when the committer reserved for it; else a standalone
		 * lone-edge flip.  urcu_txn_store upgrades a repeat slot in place,
		 * so a second apply_deferred pass costs no extra reservation.
		 */
		if (g->fuse_free_list) {
			uintptr_t old = ft_flip_txn_record_tombstone(g->txn, meta);
			/*
			 * A peer already retired this node (RYW old already has
			 * TOMBSTONE): our tombstone edge is a consistent no-op that
			 * does NOT conflict, so our commit can still succeed -- but
			 * the LIVE->TOMBSTONE transition (the retire token that owns
			 * the free) was the peer's.  Leave the free to the peer;
			 * freeing it in ft_glue_free_old would double-free the node
			 * (two grafts absorbing the same shared node both reach
			 * here).  See project_ft_barrier_uaf_is_graft_double_free.
			 */
			if (old & FT_STATE_TOMBSTONE)
				g->free_list[i].retired = false;
		} else {
			ft_meta_tombstone_set_flip(meta);
		}
	}
}

static
void ft_glue_apply_deferred(struct cds_ft *ft, struct ft_glue *g)
{
	int i;

	/*
	 * Tombstone the retired set here, the glue's commit-step-1: it runs only
	 * on the committing path (abort frees the BUILT cluster via ft_glue_abort,
	 * not this) and after the abort-impossible point, hence before the forward
	 * publish that unlinks them.
	 */
	ft_glue_tombstone_free_list(g);

	/*
	 * Apply only src-origin edges (dst_origin == false).  graft and
	 * graft_swap record every edge as src-origin (the default), so this
	 * wires all of theirs.  cds_ft_merge_at additionally records
	 * dst-origin edges, which it does NOT re-parent here: a dst child
	 * stays reachable via the old dst spine until the forward publish, so
	 * its back-pointer is switched atomically (with the merge-point
	 * forward slot) by the flip-latch, after this call.
	 */
	for (i = 0; i < g->nr_deferred; i++) {
		if (g->deferred[i].dst_origin)
			continue;
		/*
		 * FOLD (coherent rekey one-decide writer): "unreachable until the
		 * forward flip" is what licenses the plain store, and it is FALSE
		 * for the fold.  A same-trie rekey re-parents the moved subtree's
		 * own children -- which stay READER-REACHABLE through the old
		 * source spine right up to the flip -- so storing here would be a
		 * reader-visible mutation before the commit, and one that no abort
		 * rolls back.  Record them instead, exactly as the dst-origin arm
		 * does, so they flip atomically with the forward publish.
		 */
		if (g->txn && g->txn->structural_sw) {
			/*
			 * ☠ ORDER-DEPENDENCE, and why this arm is narrow.  The
			 * plain-store loop below is IN RECORDED ORDER on purpose:
			 * ft_set_parent's skip-compressed arm resolves its target
			 * through ft_skip_to_compressed, which reads a child
			 * back-pointer an EARLIER edge of this same loop may have
			 * just written.  Records do not land until the flip, so a
			 * converted edge resolves against the PRE-loop back-pointer
			 * -- a different node whenever that dependency is live.
			 *
			 * MEASURED, do not re-derive: instrumenting every src-origin
			 * edge with its target resolved before vs. after the loop,
			 * the divergence is EXACTLY the skip-compressed class and it
			 * is total -- ft_unit 12 diverged of 18 skip (125 edges), the
			 * FT_INV_MW rekey oracles 8 of 8 skip (21224 edges).  Every
			 * non-skip edge resolves to a constant.
			 *
			 * Under structural_sw the class is EMPTY -- 0 skip and 0
			 * divergent of 1879 fold re-parents across the same oracle
			 * run -- because the rekey's shape gate admits no compressed
			 * or skip-compressed node on the moved spine.  That is a
			 * property of the GATE, not of the fold, so it is asserted
			 * rather than assumed: relaxing the gate must re-measure, and
			 * will find this assert rather than a silently mis-resolved
			 * parent.
			 */
			/*
			 * THE SKIP-COMPRESSED QUESTION, SETTLED BY MEASUREMENT.
			 *
			 * The plain-store loop below is order-dependent: ft_set_parent's
			 * skip arm resolves through ft_skip_to_compressed, which reads a
			 * child back-pointer an EARLIER edge of the same loop just wrote,
			 * and the divergence is exactly that class (ft_unit 12 of 18 skip,
			 * the rekey oracles 8 of 8).  This arm first carried a blanket
			 * assert against skip children on the strength of that.
			 *
			 * That was the wrong guard, because THIS arm never stores.  Under
			 * structural_sw every src-origin edge is RECORDED, so all of them
			 * resolve against one pristine state and the pre-loop and in-order
			 * answers are identical BY CONSTRUCTION.  The order-dependence is a
			 * property of the store path, not of the resolution.
			 *
			 * What CAN still go wrong is a genuinely STALE flag -- one whose
			 * encoded child was re-homed (by an immediate store earlier in the
			 * build, or by a peer) so the recovered node no longer owns it.
			 * That is what this checks, and it is meaningful on both paths.
			 * Measured over the occupied-dst merge, the only shape reaching a
			 * skip child here: 4 of 4 consistent, 0 stale.
			 *
			 * CONCURRENTLY COVERED, and it was not until an oracle existed
			 * for it: inv_rekey_merge_occupied_dst drives 8 writers merging
			 * into permanently-occupied destinations and reaches this arm
			 * 8548 times per run -- 8548 consistent, 0 stale.  Before it,
			 * the whole FT_INV_MW plan reached this arm ZERO times (every
			 * other oracle moves into an EMPTY dst), so "a peer cannot stale
			 * the flag between this resolution and the flip" rested on an
			 * argument about holding the compressed node's LOCK.  It now
			 * rests on the assert below firing 0 times under contention.
			 */
#ifdef FEATURE_FT_SKIP_COMPRESSED
			if (g->deferred[i].child &&
					!ft_node_flip_proxy(g->deferred[i].child) &&
					ft_node_skip_compressed(g->deferred[i].child)) {
				struct cds_ft_compressed_node *scn =
					ft_skip_to_compressed(ft, g->deferred[i].child);

				assert(scn && scn->child ==
					ft_skip_child_ptr(g->deferred[i].child));
			}
#endif
			ft_reparent_record(ft, g->txn, g->deferred[i].child,
				g->deferred[i].parent, g->deferred[i].slot,
				g->deferred[i].held_lock);
			continue;
		}
		ft_set_parent(ft, g->deferred[i].child, g->deferred[i].parent,
			g->deferred[i].slot);
	}
}

/*
 * Transactional commit of a GLUE attach/replace (used when g->txn is set).
 * Follows the bulk-op rule: a pointer NOT reader-observable during the commit
 * window is set IMMEDIATELY with a plain store; only a reader-observable
 * ("live") pointer rides the txn, so its flip is atomic with the forward
 * publish.  A reader -- which descends (forward) before it walks up (back) --
 * thus observes the whole publish as old XOR new, never a half-applied mix.
 *
 *   - Hidden back-pointers (the drained payload + fresh cluster, tagged
 *     !dst_origin): ft_glue_apply_deferred sets them immediately, in recorded
 *     order.  Unreachable until the forward flip, so no atomicity is needed.
 *   - Live back-pointers (a node reachable via the OLD spine until the forward
 *     publish, tagged dst_origin) + the forward edge + the <=4 ordered-list
 *     cell edges: recorded into g->txn and committed with one selector flip.
 *
 * Called AFTER the source unlink + drain, where abort is already impossible, so
 * the live back-edge bookkeeping (ft_set_parent_slot inside
 * ft_glue_record_back_edge) runs here rather than during the build -- doing it
 * during the build would corrupt a live node's parent_slot_offset if a later
 * build step OOM'd and aborted.  The build is unchanged: edges queue in
 * g->deferred, and fresh-to-fresh edges + top->publish_parent are wired during
 * the build.
 *
 * The records cannot fail: g->txn was reserved to the bounded cluster size up
 * front (ft_flip_txn_reserve).  Reclaims the txn (deferred via the flavor, or
 * freed immediately on the exclusive fast path).
 */
static
enum urcu_txn_status ft_glue_txn_commit_edges(struct cds_ft *ft, struct ft_glue *g,
		const struct ft_ord_cell_edge *cedges, unsigned int n_cedges)
{
	struct ft_pub_rec rec = { .n = 0 };
	unsigned int j;
	int i;
	enum urcu_txn_status cst;

	/*
	 * FOLD: take the lock acquire on every child the records below will SW-park
	 * into, BEFORE the first of them is recorded and before
	 * ft_glue_tombstone_free_list runs -- nothing of this commit has landed yet,
	 * so a contended child is a clean transient.
	 *
	 * Only reachable under record_only, where @txn is the caller's and the
	 * caller's single commit is still ahead: ABORT here is a genuine bail, not a
	 * failure past the point of no return this function is otherwise specified
	 * to run at.  Asserted rather than assumed, because that is the property
	 * that makes returning ABORT from here legitimate.
	 */
	if (g->txn && g->txn->structural_sw) {
		assert(g->record_only);
		if (ft_glue_acquire_reparent_marks(ft, g))
			return URCU_TXN_STATUS_ABORT;
	}

	/*
	 * Hidden back-pointers -- re-parents of nodes NOT reader-observable
	 * during the commit window (the drained payload + the fresh cluster,
	 * tagged !dst_origin) -- are set IMMEDIATELY with plain stores, in
	 * recorded order: no reader can reach these nodes until the forward flip
	 * below, so the stores need no atomicity, and the in-order application
	 * lets a skip top resolve its compressed node (ft_skip_to_compressed
	 * reads the child back-pointer a prior edge just wired).
	 */
	ft_glue_apply_deferred(ft, g);
	/*
	 * Live back-pointers -- a node still reachable via the OLD spine until
	 * the forward publish (tagged dst_origin) -- ride @txn so their flip is
	 * atomic with the forward edge: a reader sees the re-parent old XOR new.
	 */
	for (i = 0; i < g->nr_deferred; i++) {
		if (!g->deferred[i].dst_origin)
			continue;
		/*
		 * FOLD (coherent rekey one-decide writer): a live child's re-home must
		 * be the co-committed (parent, offset) PAIR, not
		 * ft_glue_record_back_edge's parent edge beside an EARLY plain
		 * ft_set_parent_slot.  That store is unobservable in the ordinary graft
		 * window (its header says so, and it is right there), but the fold
		 * re-parents children that stay READER-REACHABLE through the old spine
		 * until the flip -- so a concurrent backtracker would read the new
		 * offset against the still-old meta->parent: the torn (parent, slot)
		 * pair ft_reparent_record_meta exists to prevent.
		 *
		 * ★ AND THE MARK.  Under structural_sw these records PARK -- plain
		 * stores that never validate.  ft_reparent_record_meta parks the
		 * child's STATE word too (its unconditional {live_state -> live_state}
		 * §4.B guard), and THAT is the word ft_meta_nr_child_inc CASes from an
		 * insert BELOW the child -- a peer neither this op's locks nor the
		 * graft's exclude.  So every metadata-bearing child re-homed here MUST
		 * carry this op's lock acquire, which is what makes that peer honor
		 * FT_STATE_INPLACE_WAIT_MASK and spin instead of clobbering the park.
		 * The same guard edge, whose new_state has LOCK masked out, is what
		 * RELEASES the mark at the flip.
		 *
		 * ☠ RELEASE ATTRIBUTION -- do not re-derive, and do not believe the
		 * pre-§8.3 story: the release is that STATE edge, recorded
		 * UNCONDITIONALLY, NOT the offset edge.  Since @118245b0 the offset is
		 * its own word and its edge is recorded only `if (record_pso)` -- so
		 * marking children on the strength of the offset edge would leak a
		 * PERMANENT LOCK on every child whose slot index happens to be
		 * unchanged across the re-home, which a merge produces routinely.
		 *
		 * An EXTERNAL head takes ft_reparent_record's own arm: no state word,
		 * hence no mark to take and none to release.
		 *
		 * Every non-fold caller keeps structural_sw false and is byte-identical.
		 */
		if (g->txn->structural_sw) {
			ft_reparent_record(ft, g->txn, g->deferred[i].child,
				g->deferred[i].parent, g->deferred[i].slot,
				g->deferred[i].held_lock);
			continue;
		}
		ft_glue_record_back_edge(ft, g->txn, g->deferred[i].child,
			g->deferred[i].parent, g->deferred[i].slot);
	}
	/*
	 * Forward publish: @g->top's parent back-pointer is already wired (a
	 * hidden top immediately above; a live top via the txn), so
	 * _ft_publish_to_parent runs its normal bookkeeping and captures its 1-2
	 * reader-visible stores (forward slot + the compressed-parent SKIP_X
	 * dance) into @rec; replay them into the txn.  *publish_slot still holds
	 * the old child here (nothing published yet post-drain).
	 */
	/*
	 * VALIDATE (§4.B) / LOCK_FINE (step 6, §9.5): acquire the LIVE dst parent
	 * this cluster publishes into as a per-node RELEASE lock ({LOCK|s -> s}).
	 * publish_parent is a value-swap REPLACE target -- it SURVIVES the commit and
	 * its body is not copied under the lock -- so a guard-fallback on an acquire
	 * miss is a correct degradation (identical to insert's ft_insert_publish_or_park
	 * parent_nf, §9.1); non-lock_fine / NULL parent falls straight to the §4.B guard.
	 */
	/*
	 * @publish_parent_holder set (LOCK_FINE cross-trie GLUE graft): this op
	 * already holds @publish_parent's node lock, acquired pre-swap so a peer
	 * could not retire it -- record the {LOCK|s -> s} RELEASE + register it
	 * (the commit consumes it, an abort auto-clears) rather than re-marking (a
	 * re-mark's masking guard would self-abort on the op's own held fence).
	 * Holder NULL routes to the ordinary acquire-or-guard (non-lock_fine / root).
	 */
	ft_flip_txn_hold_or_lock_parent(ft, g->txn, g->publish_parent,
		g->publish_parent_holder, g->publish_parent_snap);
	if (g->publish_parent_holder) {
		/*
		 * OWNERSHIP TRANSFER (the split_cn_holder block below mirrors
		 * this): the held arm just recorded the {LOCK|s -> s} RELEASE
		 * and REGISTERED the fence with @txn, so the txn owns the clear --
		 * a commit consumes it, an abort or destroy CAS-clears it back to
		 * LIVE through ft_flip_txn_lock_release_all.  NULL the holder so
		 * the caller's post-abort ft_glue_abort does NOT clear it a SECOND
		 * time: by then the survivor is LIVE and CLEAN, and under the live
		 * peers a commit-abort implies, a peer may have re-marked it in the
		 * drain->abort window -- the second clear then STEALS the peer's
		 * fence (double-free / torn publish), the exact hazard spelled out
		 * for @split_cn_holder.
		 *
		 * Two callers already DOCUMENT this NULLing as the mechanism they
		 * rely on (fractal-trie.c, the post-commit_edges bail and
		 * bail_build) and ft_merge_spine_copy open-codes it at its own
		 * hold_or_lock_parent call; commit_edges was the one path where the
		 * mechanism was missing.  Safe here because every ft_glue_op_holds
		 * read happens at the TOP of this function
		 * (ft_glue_acquire_reparent_marks), so the reconciliation set is
		 * complete before the field is disowned.
		 */
		g->publish_parent_holder = NULL;
		g->publish_parent_snap = 0;
	}
	_ft_publish_to_parent(ft, g->publish_parent, g->publish_slot, g->top,
		ft_glue_publish_expected_old(g), &rec);
	for (j = 0; j < rec.n; j++)
		ft_flip_txn_record_reserved(g->txn, (void **) rec.slot[j],
			rec.old_val[j], rec.new_val[j]);
	/*
	 * MW LOCK_FINE drop (split-compressed graft): retire the compressed
	 * divergence node @cn this GLUE build split + replaced, consuming the
	 * node lock held pre-swap (@split_cn_holder).  Records the
	 * {LOCK|snap -> TOMBSTONE|snap} terminal into the SAME txn so @cn's
	 * retire flips ATOMICALLY with the forward publish that unlinks it (the
	 * commit consumes the fence; an abort CAS-clears it via the registry).
	 * @cn is on the glue free-list, reclaimed after the commit.  Holder NULL
	 * (non-lock_fine / NOSPLIT / root splice) records nothing, byte-identical.
	 */
	if (g->split_cn_holder) {
		ft_flip_txn_record_tombstone_locked(g->txn, g->split_cn_holder,
			g->split_cn_snap);
		ft_flip_txn_lock_register(g->txn, g->split_cn_holder);
		/*
		 * OWNERSHIP TRANSFER (mirror publish_parent_holder): once
		 * registered, the txn OWNS @cn's fence clear -- a commit consumes
		 * it via the {LOCK|s -> TOMBSTONE|s} retire, and an ABORT
		 * CAS-clears it back to LIVE through ft_flip_txn_lock_release_all.
		 * NULL the holder so the caller's post-abort ft_glue_abort does NOT
		 * clear it a SECOND time -- a double clear asserts (clean word) in a
		 * debug build and, under the live peers a commit-abort implies,
		 * STEALS a peer's re-mark of @cn (fence theft -> double-free / torn
		 * publish).  ft_glue_tombstone_free_list already ran (apply_deferred,
		 * commit-step-1) so its split_cn skip saw the holder set.
		 */
		g->split_cn_holder = NULL;
		g->split_cn_snap = 0;
	}
	/*
	 * Ordered list on: also record the <=4 boundary cell edges the caller
	 * pre-computed (a run-SPLICE for a graft, a run-REPLACE for a graft_swap;
	 * a neighbour's ord_next / ord_prev plus a dst head/tail repair) into the
	 * SAME txn, so the structure becomes reachable AND the ordered list gains
	 * (and, for replace, loses) the run in one selector flip -- the cross-view
	 * atomicity the standalone ft_ord_cell_flip_rec_{run,replace} gives, now
	 * fused with the forward publish (and any live re-parents) above.  Cell
	 * slots carry the
	 * same type-7 proxy tag as structural slots, so the txn's install parks a
	 * proxy a cell reader resolves through ft_ord_cell_resolve_ord.  The
	 * _edges helper computed the run's own outer links as EDGES too (never
	 * plain stores -- they must roll back with the rest when a peer's cell
	 * conflict aborts this commit), and the wrapper arms the run descriptor
	 * on success so the caller skips the standalone splice.
	 *
	 * Recorded through the tag-dispatching helper, NOT a raw
	 * ft_flip_txn_record_tag loop: a CELL edge must be MW whatever @g->txn's
	 * structural_sw mode is -- the ordered list is lock-free and no cell
	 * carries a node lock to park an SW store under -- while record_tag
	 * keys off structural_sw alone and would demote them.
	 */
	ft_ord_cell_record_into(g->txn, cedges, n_cedges);

	/*
	 * Order-statistics fold (BULK): record the +count_delta nr_keys walk from
	 * @publish_parent (the stable node owning the forward slot) up to the root
	 * into the SAME txn, so the aggregate flips ATOMICALLY with the attach --
	 * the cluster / displaced branch below @publish_parent already carries its
	 * full count from build.  A no-op when @count_delta is 0 (every non-attach
	 * glue commit) or the trie does not maintain rank stats; @publish_parent
	 * NULL (a root splice, the whole cluster becomes the root) records nothing.
	 */
	if (g->count_delta)
		ft_flip_txn_record_count_parent(ft, g->txn, g->publish_parent,
			g->count_delta);

	/*
	 * FOLD (coherent rekey one-decide writer): the whole dst-attach is now
	 * recorded into the caller's SHARED @txn; return WITHOUT committing so the
	 * caller runs the ONE ft_flip_txn_commit that also carries the src-unlink +
	 * S_top COW.  @g->txn is left intact (the caller owns and commits it); the
	 * LOCK registrations planted above (publish_parent release, split_cn
	 * retire) live in @txn -> the caller's commit consumes them, its abort
	 * auto-clears them (ft_flip_txn_lock_release_all).  Nothing is published
	 * yet, so report OK.
	 */
	if (g->record_only)
		return URCU_TXN_STATUS_OK;

	/*
	 * Commit WITHOUT an explicit install: ft_flip_txn_commit auto-installs a
	 * multi-edge set, but a publish that reduces to a SINGLE recorded edge
	 * (e.g. a list-off attach into a plain parent with no deferred
	 * back-pointers) commits as a bare release store -- no proxy, no group
	 * flip, no grace-period reclaim -- so the common one-pointer publish pays
	 * nothing for the transaction machinery.  commit owns reclaim (deferred
	 * through the FT flavor when a proxy is owed, freed in place otherwise).
	 */
	cst = ft_flip_txn_commit(ft, g->txn);
	g->txn = NULL;
	return cst;
}

/*
 * Splice wrapper (cds_ft_graft): fuse a run-splice into the attach flip-txn.
 * Computes the <=4 run-splice boundary edges (which also pre-set @run's outer
 * links), arms @run, and commits via the edge core.  @run NULL => list off.
 */
static
enum urcu_txn_status ft_glue_txn_commit(struct cds_ft *ft, struct ft_glue *g,
		struct ft_graft_run *run)
{
	struct ft_ord_cell_edge cedges[FT_ORD_CELL_RUN_SPLICE_MAX_EDGES] = { 0 };
	unsigned int n = 0;
	enum urcu_txn_status cst;

	if (run)
		n = ft_ord_cell_run_splice_edges(ft, run->run_first,
			run->run_last, run->pred, run->succ, cedges, 0);
	cst = ft_glue_txn_commit_edges(ft, g, cedges, n);
	/*
	 * ARM ONLY when the edges took effect: on a COMMITTED flip, or under
	 * @g->record_only (nothing committed yet, but the edges are now in the
	 * CALLER's txn and its single commit carries them).  An ABORT rolled the
	 * splice back, so the run is NOT in the list and the caller's standalone
	 * fallback must still run -- arming there strands the run out of the
	 * ordered list.
	 */
	if (run && cst == URCU_TXN_STATUS_OK)
		run->armed = true;
	return cst;
}

/*
 * Replace wrapper (cds_ft_graft_swap insert side): fuse a run-replace into the
 * replace flip-txn.  run_D leaves dst's ordered list and run_S takes its place;
 * the <=4 boundary edges (pre-setting run_S's outer links) join the structural
 * replace publish.  Arms @run.  @run NULL => list off.
 */
static
enum urcu_txn_status ft_glue_txn_commit_replace(struct cds_ft *ft,
		struct ft_glue *g, struct ft_graft_swap_run *run)
{
	struct ft_ord_cell_edge cedges[FT_ORD_CELL_RUN_REPLACE_MAX_EDGES] = { 0 };
	unsigned int n = 0;
	enum urcu_txn_status cst;

	if (run)
		n = ft_ord_cell_run_replace_edges(ft, run->d_first, run->d_last,
			run->s_first, run->s_last, cedges, 0);
	/*
	 * Return the commit status: under the FT-wide lock the replace is
	 * failure-free (caller ignores it), but with the lock dropped
	 * (FT-wide-lock drop) a peer relocating the contended dst
	 * parent aborts the commit -- cds_ft_graft_swap re-descends on it.
	 */
	cst = ft_glue_txn_commit_edges(ft, g, cedges, n);
	/* Arm only when the edges took effect (see ft_glue_txn_commit). */
	if (run && cst == URCU_TXN_STATUS_OK)
		run->armed = true;
	return cst;
}

/*
 * Record a deferred duplicate-chain splice (cds_ft_merge_at, same full key in
 * both tries): the @src_head chain is to be appended to @dst_head's chain.
 * The @dst_head chain's forward owner and back-pointer are wired separately
 * (Phase-1 set + deferred edge), like any other re-parented external; this
 * records only the concatenation, applied at commit.
 */
#ifdef FEATURE_FT_MERGE
static
void ft_glue_record_splice(struct ft_glue *g,
		struct cds_ft_node *dst_head,
		struct cds_ft_node *src_head)
{
	assert(g->nr_splices < g->cap_splices);
	g->splices[g->nr_splices].dst_head = dst_head;
	g->splices[g->nr_splices].src_head = src_head;
	g->splices[g->nr_splices].src_cell = NULL;
	/*
	 * Not demoted yet: ft_glue_record_splices arms these when (and only when)
	 * it runs the append.  A bail BETWEEN this record and that call reaches
	 * ft_glue_abort with the splice already in the array, and the undo loop
	 * there must see false rather than whatever the buffer last held.
	 */
	g->splices[g->nr_splices].src_prev = NULL;
	g->splices[g->nr_splices].src_demoted_to = NULL;
	g->splices[g->nr_splices].src_demoted = false;
	g->splices[g->nr_splices].holder = NULL;
	g->splices[g->nr_splices].holder_snap = 0;
	g->nr_splices++;
}

/*
 * MW LOCK_FINE, the dup-chain lock-set: acquire the node lock of every
 * DISTINCT chain HOLDER the recorded splices are about to append to, so
 * ft_glue_record_splices' tail walk and tail append run under the same per-node
 * lock every OTHER chain mutation already takes -- insert's duplicate append,
 * remove's interior/head unchain, ft_promote_head, cds_ft_replace's non-head
 * replace.  The glue run-append was the last chain mutation running unlocked,
 * and while it did, the six urcu_txn_store_mw in ft-txn-hlist.h could not
 * become sw: a chain append that is not excluded is a LOST UPDATE the moment
 * the store stops arbitrating by expected value.
 *
 * ALL-OR-NONE, and THE CALLER MUST BAIL on -EAGAIN -- a partial lock-set is
 * precisely the race the lock exists to close.  On a miss every mark already
 * taken is dropped here, so the caller unwinds a build with no fence held.
 * Call BEFORE the merge's point of no return (both tries still pristine); the
 * release belongs AFTER the single commit that installs the appends, because
 * the lock must span the tail walk, the record and the install.
 *
 * THE HOLDER IS DERIVED, NEVER ASSUMED.  ft_chain_head_holder walks prev from
 * the head; the merge's own parent pointer is NOT the chain anchor for a
 * duplicate, and using one is a straight SIGSEGV in cds_ft_item_to_metadata.
 * It is then RE-DERIVED after the mark and compared, because holder identity is
 * not stable: ft_promote_head swaps a fresh cell in and a recompact rebuilds the
 * parent, either of which would leave the lock sitting on a node nobody uses.  A
 * mismatch bails exactly like a miss.  (The mark itself already excludes the
 * settled forms of both: a retired holder's word carries TOMBSTONE and an
 * in-flight peer's carries a parked proxy, and ft_meta_lock_acquire refuses
 * both.  The re-derive covers the change that completed between our read of
 * prev and our CAS.)
 *
 * DEDUP IS MANDATORY FOR TERMINATION, not an optimisation.  ft_meta_lock_acquire
 * returns -EAGAIN on an already-locked word, so two keys colliding under ONE
 * holder would fail against OUR OWN mark and send the caller back into the
 * identical collision forever.  The scan is quadratic in @nr_splices, which is
 * the same-key collision count -- 0 for the batch-staging workload, and bounded
 * by the smaller trie's key count otherwise.
 *
 * A NULL holder is ASSERTED against, not skipped.  It would mean a
 * never-inserted head (prev NULL), whose only producers are ft-insert's unwind
 * paths on UNPUBLISHED nodes -- and every @dst_head here was reached by the
 * merge build walking LIVE dst structure.  Skipping would silently append with
 * no exclusion, which is exactly the lost update this lock set exists to
 * prevent once the chain stores become sw.  Both of ft_chain_head_holder's NULL
 * branches measured unreachable: 0 in 491532 calls across ft_unit and ft_inv's
 * three list modes.
 *
 * THE FENCES ARE DELIBERATELY NOT REGISTERED with the flip-txn.
 * FT_FLIP_TXN_MAX_LOCKS (8) sizes the TXN-TRACKED fences of one recompact,
 * while the collision count is unbounded; and an unregistered fence is
 * unambiguously still ours at every release point (see
 * ft_glue_release_splice_holders).  The one exception is the holder that turns
 * out to BE the publish target, which ft_glue_splice_holder_take hands to the
 * txn as a release record.
 *
 * RESIDUAL, and not what this closes: @dst_head itself was captured during the
 * build, before this lock.  A peer that removes that head between the build and
 * this acquire is not excluded -- the same build-to-publish staleness the merge
 * already carries for every dst-origin edge it records.
 */
static
int ft_glue_acquire_splice_holders(struct cds_ft *ft, struct ft_glue *g)
{
	int i, j;

	if (!ft->lock_fine)
		return 0;
	for (i = 0; i < g->nr_splices; i++) {
		struct cds_ft_node *dst_head = g->splices[i].dst_head;
		struct cds_ft_inode_flag *hf = ft_chain_head_holder(ft, dst_head);
		struct cds_ft_metadata *hm;
		bool held = false;

		assert(hf);
		hm = ft_flag_to_metadata(ft, hf);
		/*
		 * Already fenced by the overlap-spine plan-lock: reuse it, take no
		 * second lock, and record no holder -- ft_glue_clear_fenced owns that
		 * fence's release.  See ft_glue_fence_holds for why re-marking it is
		 * a self-deadlock rather than a miss.
		 */
		if (ft_glue_fence_holds(g, hm))
			continue;
		for (j = 0; j < i; j++) {
			if (g->splices[j].holder == hm) {
				held = true;
				break;
			}
		}
		if (held)
			continue;
#ifdef FEATURE_FT_FAULT_INJECT
		/*
		 * Test-only: fail this acquire exactly as a peer holding the
		 * holder would (cds_ft_fault_lock_countdown, shared with
		 * ft_lock_member and ft_flip_txn_lock_or_guard_parent).
		 * Drives the caller's bail + re-descend, which the natural rate
		 * of same-key collisions under contention makes rare.
		 */
		if (cds_ft_fault_lock_countdown >= 0) {
			if (cds_ft_fault_lock_countdown == 0) {
				cds_ft_fault_lock_countdown = -1;
				goto miss;
			}
			cds_ft_fault_lock_countdown--;
		}
#endif
		if (ft_meta_lock_acquire(hm, &g->splices[i].holder_snap))
			goto miss;
		g->splices[i].holder = hm;
		if (ft_chain_head_holder(ft, dst_head) != hf)
			goto miss;	/* re-parented under us: stale lock */
	}
	return 0;
miss:
	ft_glue_release_splice_holders(g);
	return -EAGAIN;
}

/*
 * Record the deferred duplicate-chain splices into the bulk merge @txn, still in
 * PREPARE (before the commit).  Call AFTER the source has been detached + drained
 * (so the appended @src_head chain has no second owner traversing it from the
 * source tree), and AFTER the ordered-list interleave has been collected (so each
 * @src_head's cell is still captured from its intact src-run prev before the
 * append overwrites it).
 *
 * Per splice: append the whole src chain to dst's tail, prev-before-next (the
 * ft_chain_node idiom, but preserving src_head->next so the rest of the src chain
 * rides along), recorded as a single forward edge into @txn so the concatenation
 * flips ATOMICALLY with the structural publish -- a collided key's full duplicate
 * set (dst + src) becomes reachable in one instant, closing the old post-commit
 * window.  Only the dst tail carries an engine proxy while the commit is in
 * flight (readers resolve it via cds_ft_node_next_rcu).  @dst_head stays the
 * head, so in-flight dst snapshots keep their ordering.
 */
static
void ft_glue_record_splices(struct cds_ft *ft, struct ft_glue *g,
		struct ft_flip_txn *txn)
{
	int i;

	for (i = 0; i < g->nr_splices; i++) {
		struct cds_ft_node *dst_head = g->splices[i].dst_head;
		struct cds_ft_node *src_head = g->splices[i].src_head;
		struct cds_ft_node *tail = dst_head;
		/*
		 * Ordered list on: @src_head was a head in src (prev is its cell);
		 * it becomes a non-head duplicate of @dst_head, so its cell leaves
		 * the trie.  The cell is NOT unreachable yet: on the merge spine
		 * path this runs after the structural flip, and the surviving src
		 * heads' cells -- already reachable in dst -- still carry the old
		 * src-run ord_prev/ord_next, including links to THIS cell, until
		 * the post-publish interleave rewires them.  Capture it in the
		 * splice record; ft_glue_free_collided_cells frees it after
		 * the interleave through the grace-period-deferred cell free.
		 * List off: src_head->prev is the flagged parent, no cell.
		 */
		g->splices[i].src_cell = ft->ordered_list ?
			ft_ord_cell_ptr(src_head->prev) : NULL;
		/*
		 * Snapshot the back-pointer the append is about to overwrite, so an
		 * abort can put it back (see @src_prev).  Taken here, before the
		 * prepare, because the prepare is where the plain store happens.
		 */
		g->splices[i].src_prev = src_head->prev;

		while (ft_node_next(tail))
			tail = ft_node_next(tail);
		/*
		 * @src_head (an already-published, detached+drained src head)
		 * becomes a duplicate at the tail of @dst_head's chain.  Record
		 * the reader-visible forward link tail->next: NULL -> src_head
		 * into the bulk merge @txn (the run rides along via src_head->next,
		 * which is untouched; src_head->prev = tail is a writer-only plain
		 * store inside the primitive).  The tail-walk above reads unmodified
		 * slots -- recorded edges do not install until the commit -- so it
		 * always finds the true pre-merge tail.
		 */
		ft_hlist_append_run_prepare(ft_flip_txn_handle(txn), tail, src_head);
		/* The value the prepare just stored: the undo's CAS expected-old. */
		g->splices[i].src_demoted_to = tail;
		g->splices[i].src_demoted = true;
	}
}

/*
 * Free the collided (demoted) src heads' cells.  Call AFTER the ordered-list
 * interleave: only then has every surviving cell's stale src-run link been
 * rewired away from these cells, making them unreachable to NEW readers; the
 * grace-period defer inside ft_ord_cell_free then covers readers already
 * holding a stale link or parked on a demoted head.
 */
static
void ft_glue_free_collided_cells(struct cds_ft *ft,
		struct ft_glue *g)
{
	int i;

	for (i = 0; i < g->nr_splices; i++)
		if (g->splices[i].src_cell)
			ft_ord_cell_free(ft, g->splices[i].src_cell);
}
#endif /* FEATURE_FT_MERGE */

/*
 * Commit step 2: the single forward publish that makes the whole cluster
 * reachable in dst.  Call after ft_glue_apply_deferred.  The
 * cluster top's parent is wired into publish_parent at set_publish time
 * (build phase, fresh-child store) and the rest of the cluster's
 * internal back-pointers are also already set, so by the time we publish
 * every back-pointer needed for an up-walk from any re-parented live
 * node up through the cluster to publish_parent is in place.
 */
static
enum urcu_txn_status ft_glue_publish(struct cds_ft *ft, struct ft_flip_txn *txn,
		struct ft_glue *g)
{
	struct ft_pub_rec rec = { .n = 0 };
	struct ft_ord_cell_edge sedges[2] = { 0 };	/* forward slot + compressed SKIP_X dual */
	unsigned int n;

	/*
	 * Record the 1-2 reader-visible structural stores (the forward slot and,
	 * for a compressed parent, its SKIP_X dual) and commit them in ONE flip
	 * instead of a bare ft_publish_to_parent.  A lone edge still reduces to a
	 * single release store, but both stores now ride a {slot, old, new}
	 * descriptor: the compressed dual flips atomically (no torn window) and a
	 * future MCAS commit covers the publish uniformly.  Un-abortable (the
	 * op's failure-free section), so it commits through the caller-PRE-RESERVED
	 * @txn (ft_ord_cell_flip_into, infallible).
	 */
	/*
	 * VALIDATE (§4.B) / LOCK_FINE (step 6, §9.5): acquire publish_parent as a
	 * RELEASE lock (value-swap REPLACE survivor, guard-fallback on a miss); see
	 * ft_glue_txn_commit_edges for the full rationale.
	 */
	ft_flip_txn_lock_or_guard_parent(ft, txn, g->publish_parent);
	_ft_publish_to_parent(ft, g->publish_parent, g->publish_slot, g->top,
		ft_glue_publish_expected_old(g), &rec);
	n = ft_pub_rec_sedges(&rec, sedges);
	/*
	 * Order-statistics fold (BULK): record the +count_delta nr_keys walk
	 * from @publish_parent into the SAME txn before the flip, so the count
	 * goes live ATOMICALLY with the forward publish.  A no-op when
	 * @count_delta is 0 or rank stats are off.
	 */
	if (g->count_delta)
		ft_flip_txn_record_count_parent(ft, txn, g->publish_parent,
			g->count_delta);
	/*
	 * ★ ABORT IS REACHABLE HERE; RETURN THE STATUS.  Two cds_ft_graft_swap
	 * KEY_SHORTER swaps at ONE destination both take this legacy path, and
	 * the loser's flip aborts -- a SHARED destination is exactly what removes
	 * the exclusion that would otherwise make this un-abortable, and the
	 * public contract grants one.  A caller that assumes it published will
	 * re-root the displaced subtree into @swap_ft and free the dst node the
	 * publish never replaced, leaving a live dst grandchild naming a parent
	 * inside the OTHER trie ("depth 3 ... parent mismatch: expected P1, got
	 * P2", inv_graft_swap_shared_dst_deep_nolist).
	 */
	{
		enum urcu_txn_status pst =
			ft_ord_cell_flip_into(ft, txn, sedges, n);

		if (pst != URCU_TXN_STATUS_OK)
			FT_GS_PROBE_INC(cds_ft_probe_gs_pubabort);
		else
			FT_GS_PROBE_INC(cds_ft_probe_gs_pubok);
		return pst;
	}
}

/*
 * GLUE-path graft_swap publish FUSED with an ordered-list run-REPLACE (the
 * legacy KEY_SHORTER graft_swap commit path).  When @run is set, RECORD
 * the cluster's forward publish edge (plus a compressed parent's SKIP_X dual)
 * via a ft_pub_rec instead of storing it, append the run-replace boundary edges,
 * and commit them all in ONE flip, so a reader never sees run_S's keys
 * reachable in the structure but absent from the ordered list (or run_D the
 * reverse).  @run NULL (ordered list off) falls back to the plain publish.  Both
 * commit through the caller-PRE-RESERVED @txn, so neither can fail for want of
 * memory; @txn capacity must be >= FT_GLUE_PUBLISH_REPLACE_MAX_EDGES.  RETURNS
 * the commit status: on a SHARED destination a peer can still make it ABORT,
 * and the caller must re-plan rather than assume it published.
 */
static
enum urcu_txn_status ft_glue_publish_replace(struct cds_ft *ft,
		struct ft_flip_txn *txn,
		struct ft_glue *g, struct ft_graft_swap_run *run)
{
	struct ft_pub_rec rec = { .n = 0 };

	if (!run)
		return ft_glue_publish(ft, txn, g);
	/*
	 * VALIDATE (§4.B) / LOCK_FINE (step 6, §9.5): acquire publish_parent as a
	 * RELEASE lock (value-swap REPLACE survivor, guard-fallback on a miss); see
	 * ft_glue_txn_commit_edges for the full rationale.
	 */
	ft_flip_txn_lock_or_guard_parent(ft, txn, g->publish_parent);
	_ft_publish_to_parent(ft, g->publish_parent, g->publish_slot, g->top,
		ft_glue_publish_expected_old(g), &rec);
	/* Order-statistics fold (BULK): see ft_glue_publish. */
	if (g->count_delta)
		ft_flip_txn_record_count_parent(ft, txn, g->publish_parent,
			g->count_delta);
	return ft_ord_cell_flip_rec_replace(ft, txn, &rec, run);
}

/*
 * Commit step 3: reclaim the old (replaced) live nodes, deferred via the
 * normal grace-period free.  Call after the forward publish.
 */
static
void ft_glue_free_old(struct cds_ft *ft, struct ft_glue *g)
{
	int i;

	for (i = 0; i < g->nr_free; i++) {
		/*
		 * Only the committer that performed the node's LIVE->TOMBSTONE
		 * transition frees it (ft_glue_tombstone_free_list clears @retired
		 * when a peer had already tombstoned it under a concurrent drop
		 * graft).  Skipping the non-retired entries makes the reclaim
		 * exactly-once across concurrent grafts absorbing a shared node.
		 */
		if (!g->free_list[i].retired)
			continue;
		if (g->free_list[i].compressed)
			free_compressed_node(ft, g->free_list[i].node);
		else
			free_cds_ft_node(ft, g->free_list[i].node);
	}
}
