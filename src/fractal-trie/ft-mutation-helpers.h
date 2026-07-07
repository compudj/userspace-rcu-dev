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
};

static
void ft_descent_init(struct ft_descent *d, struct cds_ft *ft)
{
	d->depth = 0;
	d->nf = ft->root;
	d->nfp = &ft->root;
	d->pnf = NULL;
	d->pnfp = NULL;
	d->ppnf = NULL;
	d->ppnfp = NULL;
}

/*
 * Advance descent state through a compressed node on full key match.
 * Updates parent chain, current pointer, and depth.  The caller is
 * responsible for snapshot, snapshot_n, and detach tracking before
 * calling this helper.
 */
static inline_lookup
void ft_descent_traverse_compressed(struct ft_descent *d,
		struct cds_ft_compressed_node *cn,
		const uint8_t **iter_key)
{
	d->ppnf  = d->pnf;
	d->ppnfp = d->pnfp;
	d->pnf   = d->nf;
	d->pnfp  = d->nfp;
	/*
	 * Resolve a transient type-7 flip proxy a peer parked on cn->child
	 * (Phase 4.3) to its committed-or-old target BEFORE it becomes the
	 * descent cursor: the next ft_descent_step reads d->pnf as a node, so an
	 * unresolved proxy (low nibble 0xF reads as internal type 7) would drive
	 * a get_nth off a garbage type -> SIGILL.  d->nfp still names the raw
	 * slot; only the snapshot d->nf is resolved (mirrors ft_node_get_nth).
	 */
	d->nf    = ft_resolve_flip_proxy(cn->child);
	d->nfp   = &cn->child;
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
	d->ppnf  = d->pnf;
	d->ppnfp = d->pnfp;
	d->pnf   = d->nf;
	d->pnfp  = d->nfp;
	d->nf    = ft_node_get_nth(ft, d->pnf, &d->nfp, key_value, FT_PF_NONE);
	d->depth++;
	return d->nf;
}

/*
 * FT bridge to the concurrent MCAS transaction engine (<urcu/rcu-txn.h>).  An
 * op records its frozen edge set {slot, old, new} DIRECTLY into the urcu_mcas
 * transaction (@mtxn) during its build -- through ft_flip_txn_record_reserved /
 * ft_flip_txn_record_tag and the ordered-list *_prepare helpers -- then
 * ft_flip_txn_commit commits @mtxn, so the whole set (structural index AND
 * ordered-cell list) publishes atomically (one status-word flip).  There is no
 * intermediate single-updater buffer: every slot is stored straight into the
 * concurrent engine, so no edge can be dropped between a buffer and a replay.
 *
 * Two record tag families share the ONE @mtxn (the engine carries the tag PER
 * record, urcu_mcas_record.proxy_tag): STRUCTURAL trie edges carry FT's type-7 /
 * 0xF tag (FT_FLIP_PROXY_TAG), resolved on the read hot path by
 * ft_resolve_flip_proxy; ORDERED-CELL list edges carry the concurrent list's
 * engine tag (URCU_MCAS_TAG, bit 0), resolved by urcu_txn_list_resolve.  Commit
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
 * Upper bound of FT_STATE_COPYING fences one commit can hold: a recompact
 * retire fences ONE node; the chain-compress fused merge fences the collapsed
 * chain (boundary + old parent cn + old child cn = 3).
 */
#define FT_FLIP_TXN_MAX_COPYING	4

struct ft_flip_txn {
	struct urcu_mcas_txn *mtxn;	/* the concurrent commit engine handle:
					 * &own (standalone txn), or the op's
					 * PERSISTENT handle (create_bounded_on)
					 * whose retry aging / FIFO turn / learned
					 * size span the op's restart_attempt loop
					 * (doc/design/mcas-multiwriter-readiness.md
					 * §11) */
	struct urcu_mcas_txn own;	/* backing handle for standalone txns */
	bool reserved;			/* @mtxn pre-reserved (bounded) => infallible commit */
	/*
	 * FT_STATE_COPYING fence registry (MW F2, CORE_682870 fix plan): the
	 * nodes this commit's op MARKED with the reversible copy fence.  On
	 * commit OK the fence is consumed by the recorded state transition
	 * {COPYING|s -> TOMBSTONE|s}; on EVERY other terminal outcome of the
	 * wrapper -- commit ABORT / MEMORY_ERROR (ft_flip_txn_commit) or a
	 * pre-commit bail (ft_flip_txn_destroy) -- the fence must be CLEARED
	 * or every later peer publish into the node aborts forever.  The two
	 * terminal paths drain this registry so no caller unwind can leak a
	 * fence.
	 */
	struct cds_ft_metadata *copying[FT_FLIP_TXN_MAX_COPYING];
	unsigned int nr_copying;
};

/*
 * The engine handle behind @t -- pass to the urcu_txn_* / *_prepare primitives
 * that record directly into the transaction.
 */
static inline
struct urcu_mcas_txn *ft_flip_txn_handle(struct ft_flip_txn *t)
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
void ft_txn_op_init(struct cds_ft *ft, struct urcu_mcas_txn *op)
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
	t->reserved = false;		/* unbounded: @mtxn grows as edges record */
	t->nr_copying = 0;
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
#endif
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
	if (urcu_txn_reserve(t->mtxn, cap) < 0) {
		if (t->mtxn->mcas && t->mtxn->mcas != URCU_TXN_ENOMEM)
			urcu_mcas_destroy(t->mtxn->mcas);
		free(t);
		return NULL;
	}
	t->reserved = true;
	t->nr_copying = 0;
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
struct ft_flip_txn *ft_flip_txn_create_bounded_on(struct urcu_mcas_txn *op,
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
	t->nr_copying = 0;
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
	unsigned int cur = (t->mtxn->mcas && t->mtxn->mcas != URCU_TXN_ENOMEM) ?
			t->mtxn->mcas->cap : 0;

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
 * FT_STATE_COPYING copy fence, MARK side (MW F2, Option A --
 * fractal-trie-internal.h at the bit's definition, CORE_682870 fix plan).  A
 * body copier (recompact retire / chain-compress collapse) CASes
 * {clean -> |COPYING} on the node it is about to read, BEFORE the first body
 * read: from that point every peer publish into the node fails its §4.B
 * clean-LIVE guard at commit, so the copied body cannot go stale between the
 * copy and the copier's own commit without SOMEONE aborting -- the copier's
 * state record {COPYING|s -> TOMBSTONE|s} pins the whole word from mark to
 * commit (a peer state change under the fence, e.g. a write record that
 * captured its expected old after the mark, mismatches the copier's expected
 * old instead: exactly one side survives).  A dirty word at the mark -- a
 * parked proxy, a tombstone (real retire), or COPYING (single-copier
 * exclusion) -- fails the mark: -EAGAIN, the op re-descends after the peer
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
int ft_meta_copying_mark(struct cds_ft_metadata *meta,
		uintptr_t *state_snapshot)
{
	uintptr_t s = CMM_LOAD_SHARED(meta->state);

	if (caa_unlikely(s & (FT_STATE_PROXY | FT_STATE_TOMBSTONE |
			FT_STATE_COPYING)))
		return -EAGAIN;
	if (caa_unlikely(uatomic_cmpxchg(&meta->state, s,
			s | FT_STATE_COPYING) != s))
		return -EAGAIN;
	*state_snapshot = s;
	return 0;
}

/*
 * FT_STATE_COPYING copy fence, CLEAR side: drop ONLY the reversible fence bit,
 * preserving every other field -- the copy was abandoned (pre-commit bail) or
 * its commit aborted (the engine settled the state record back to its old
 * value, COPYING included), and the node stays LIVE.  The word may transiently
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
void ft_meta_copying_clear(struct cds_ft_metadata *meta)
{
	for (;;) {
		uintptr_t s = CMM_LOAD_SHARED(meta->state);

		if (caa_unlikely(s & FT_STATE_PROXY)) {
			caa_cpu_relax();
			continue;
		}
		/*
		 * Only the fence owner clears COPYING, and peers preserve
		 * foreign state bits, so the bit is still set here (NDEBUG
		 * builds degrade to a harmless same-value CAS if it is not).
		 */
		assert(s & FT_STATE_COPYING);
		if (caa_likely(uatomic_cmpxchg(&meta->state, s,
				s & ~FT_STATE_COPYING) == s))
			return;
	}
}

/*
 * Register a marked fence with the commit wrapper that owns its outcome: the
 * two terminal paths (ft_flip_txn_commit on ABORT / MEMORY_ERROR,
 * ft_flip_txn_destroy on a pre-commit bail) clear every registered fence, and
 * a commit OK consumes it through the recorded {COPYING|s -> TOMBSTONE|s}
 * transition instead.  Register only once the mark's holder can no longer
 * clear it itself (i.e. when the op hands the outcome to the txn).
 */
static inline
void ft_flip_txn_copying_register(struct ft_flip_txn *t,
		struct cds_ft_metadata *meta)
{
	assert(t->nr_copying < FT_FLIP_TXN_MAX_COPYING);
	t->copying[t->nr_copying++] = meta;
}

static inline
void ft_flip_txn_copying_clear_all(struct ft_flip_txn *t)
{
	unsigned int i;

	for (i = 0; i < t->nr_copying; i++)
		ft_meta_copying_clear(t->copying[i]);
	t->nr_copying = 0;
}

/*
 * Drop an FT flip-txn that was NOT committed (an op aborted before publishing --
 * an OOM or a no-op path).  Nothing was stored into any slot
 * (freeze-before-install), so this frees any live (uncommitted) MCAS descriptor
 * and the handle; no grace period is owed.  Registered COPYING fences are
 * cleared (the marked nodes stay live; nothing retires them).
 */
static inline
void ft_flip_txn_destroy(struct ft_flip_txn *t)
{
	ft_flip_txn_copying_clear_all(t);
	if (t->mtxn->mcas && t->mtxn->mcas != URCU_TXN_ENOMEM) {
		urcu_mcas_destroy(t->mtxn->mcas);
		/*
		 * Leave a BOUND persistent handle clean for the op's next
		 * attempt / its urcu_txn_end() (which would otherwise
		 * double-destroy).  A sticky URCU_TXN_ENOMEM marker is
		 * preserved above (only a live descriptor is destroyed), so
		 * an OOM already recorded on the handle still surfaces.
		 * Harmless for a standalone txn (@own dies with the wrapper).
		 */
		t->mtxn->mcas = NULL;
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
 * descriptor (urcu_mcas_commit reclaims it even on abort), so re-driving the same
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

	st = urcu_txn_commit_flavor(t->mtxn, reclaim);
	FT_TP(txn_commit, (const void *) t->mtxn, (int) st);
	/*
	 * COPYING fences: a committed txn transitioned each registered node
	 * {COPYING|s -> TOMBSTONE|s} (the fence is consumed by the retire);
	 * ABORT settled the state record back to its old value -- fence still
	 * set -- and MEMORY_ERROR parked nothing, so both must clear the
	 * reversible bit or the still-live nodes would fail every later peer
	 * guard forever.
	 */
	if (caa_unlikely(st != URCU_TXN_STATUS_OK))
		ft_flip_txn_copying_clear_all(t);
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
	ret = urcu_txn_store(t->mtxn, slot, old_ptr, new_ptr, tag);
	assert(!ret);
	(void) ret;	/* reserved up front -> never fails */
}

static inline
void ft_flip_txn_record_reserved(struct ft_flip_txn *t, void **slot,
		void *old_ptr, void *new_ptr)
{
	ft_flip_txn_record_tag(t, slot, old_ptr, new_ptr, FT_FLIP_PROXY_TAG);
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
static
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
	state = (uintptr_t) urcu_mcas_read((void **) &meta->state,
			FT_STATE_PROXY);
	rt_parent = ft_resolve_flip_proxy(rcu_dereference(meta->parent));
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
#define FT_TRACE_MISWIRE(ft, edge, site) ft_trace_miswire_check(ft, edge, site)
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
	 * Per-edge engine proxy tag: URCU_MCAS_TAG (bit 0) for an ORDERED-CELL
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
		edges[n].tag = URCU_MCAS_TAG;	/* ordered-cell edge */
		edges[n].slot = (struct ft_ord_cell **) &ft->ord_sentinel.node.next;
		edges[n].old_target = hn_old;
		edges[n].new_target = hn_new;
		n++;
	}
	if (tp_old != tp_new) {
		edges[n].tag = URCU_MCAS_TAG;	/* ordered-cell edge */
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
			edges[n].tag = URCU_MCAS_TAG;	/* ordered-cell edge */
			edges[n].slot = (struct ft_ord_cell **) &head_new->lnode.prev;
			edges[n].old_target = dest;
			edges[n].new_target = self;
			n++;
		}
		if (tail_new) {
			edges[n].tag = URCU_MCAS_TAG;	/* ordered-cell edge */
			edges[n].slot = (struct ft_ord_cell **) &tail_new->lnode.next;
			edges[n].old_target = dest;
			edges[n].new_target = self;
			n++;
		}
	}
	return n;
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
		uintptr_t (*f)(uintptr_t))
{
	for (;;) {
		uintptr_t s = CMM_LOAD_SHARED(meta->state);

		if (caa_unlikely(s & FT_STATE_PROXY)) {
			/*
			 * Bounded by the latch owner's settle (owner-only; a
			 * writer dying mid-install would wedge this, but
			 * writer death mid-mutation is already fatal to the
			 * trie by contract).
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
	ft_meta_state_transition(meta, ft_state_f_nr_child_dec);
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
	ft_meta_state_transition(meta, ft_state_f_tombstone);
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
void ft_flip_txn_record_tombstone(struct ft_flip_txn *t,
		struct cds_ft_metadata *meta)
{
	uintptr_t old = meta->state;

	ft_flip_txn_record_tag(t, (void **) &meta->state,
			(void *) old, (void *) (old | FT_STATE_TOMBSTONE),
			FT_STATE_PROXY);
}

/*
 * The FENCED variant for a node the op marked with ft_meta_copying_mark: the
 * expected old is the mark's CLEAN snapshot with the fence bit -- NOT a fresh
 * raw read -- so the commit ratifies exactly the world the copy plan was
 * derived from (the CORE_682870 defect-1 fix: a stale plan can no longer be
 * ratified by a coincidentally-matching late capture).  The new value drops
 * the reversible fence and sets the one-way tombstone in the same atomic
 * transition {COPYING|s -> TOMBSTONE|s}: tombstone semantics (commit-atomic,
 * exactly-once retire token) are unchanged.  Any peer state change under the
 * fence -- a re-home's PSO pair, a fused count, a foreign tombstone -- makes
 * this record's expected old mismatch: the copier aborts and its fence is
 * cleared by the commit wrapper's registry.
 */
static inline
void ft_flip_txn_record_tombstone_copying(struct ft_flip_txn *t,
		struct cds_ft_metadata *meta, uintptr_t state_snapshot)
{
	ft_flip_txn_record_tag(t, (void **) &meta->state,
			(void *) (state_snapshot | FT_STATE_COPYING),
			(void *) (state_snapshot | FT_STATE_TOMBSTONE),
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
	 * the reversible copy fence (FT_STATE_COPYING): a holder under a
	 * peer's body copy at validate time mismatches -> ABORT -> retry; a
	 * copier that ABORTED and cleared the fence before our validate
	 * matches -> proceed (the copy was abandoned, the holder unchanged).
	 */
	live = v & ~(FT_STATE_TOMBSTONE | FT_STATE_COPYING);
	urcu_txn_validate(t->mtxn,
			(void **) &ft_flag_to_metadata(ft, parent_nf)->state,
			(void *) live, FT_STATE_PROXY);
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

			edges[n].tag = URCU_MCAS_TAG;	/* ordered-cell edge */
			edges[n].slot = (struct ft_ord_cell **)
				&r->head_old->lnode.prev;
			edges[n].old_target = self;
			edges[n].new_target = NULL;
			n++;
			edges[n].tag = URCU_MCAS_TAG;	/* ordered-cell edge */
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
	edges[n].tag = URCU_MCAS_TAG;	/* ordered-cell edge */
	edges[n].slot = (struct ft_ord_cell **) &pred->lnode.next;
	edges[n].old_target = first;
	edges[n].new_target = succ;
	n++;
	edges[n].tag = URCU_MCAS_TAG;	/* ordered-cell edge */
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
 * Find the cell of the in-order predecessor (mode LT) / successor (mode GT)
 * of @key via the eager relational descent on the writer's cell scratch
 * iterator.  Returns NULL when none exists (@key is the new minimum/maximum).
 */
static
struct ft_ord_cell *ft_ord_cell_find_rel(struct cds_ft *ft, const uint8_t *key,
		size_t key_len, enum ft_lookup_inequality mode)
{
	struct cds_ft_iter *it = ft->ord_cell_scratch_iter;
	struct cds_ft_node *head;

	if (cds_ft_iter_set_key(it, key, key_len) != CDS_FT_STATUS_OK)
		return NULL;
	it->prefix_len = 0;
	it->node = NULL;
	if (cds_ft_lookup_inequality_impl(ft, it, mode, FT_LOOKUP_LIMIT_NONE,
			false, false) != CDS_FT_STATUS_OK)
		return NULL;
	head = cds_ft_iter_node(it);
	if (!head)
		return NULL;
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
static
struct ft_ord_cell *ft_ord_cell_find_pred_from_head(struct cds_ft *ft,
		const uint8_t *key, size_t key_len, struct ft_ord_cell *cell,
		bool from_root)
{
	struct cds_ft_iter *it = ft->ord_cell_scratch_iter;
	struct cds_ft_node *pred_head;

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
	it->node = NULL;
	if (cds_ft_iter_set_key(it, key, key_len) != CDS_FT_STATUS_OK)
		return NULL;
	if (!from_root) {
		it->node = cell->node;
		it->cache_valid = true;
		it->ord_cell_node = NULL;
		it->prefix_len = 0;
		it->path_len = it->key_len + 1;
	}
	if (cds_ft_lookup_inequality_impl(ft, it, FT_LOOKUP_LT,
			FT_LOOKUP_LIMIT_NONE, false, !from_root) != CDS_FT_STATUS_OK)
		return NULL;
	pred_head = cds_ft_iter_node(it);
	if (!pred_head)
		return NULL;
	return ft_ord_cell_ptr(rcu_dereference(pred_head->prev));
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
 * mask the bit (urcu_txn_list_next_rcu strips URCU_MCAS_TAG |
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
	edges[n].tag = URCU_MCAS_TAG;	/* ordered-cell edge */
	edges[n].slot = (struct ft_ord_cell **) &pred->lnode.next;
	edges[n].old_target = cell;
	edges[n].new_target = succ;
	n++;
	edges[n].tag = URCU_MCAS_TAG;	/* ordered-cell edge */
	edges[n].slot = (struct ft_ord_cell **) &succ->lnode.prev;
	edges[n].old_target = cell;
	edges[n].new_target = pred;
	n++;
	edges[n].tag = URCU_MCAS_TAG;	/* deletion mark: see the comment above */
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
	 * is infallible.  The concurrent list's URCU_MCAS_TAG applies, so the
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
	edges[n].tag = URCU_MCAS_TAG;	/* ordered-cell edge */
	edges[n].slot = (struct ft_ord_cell **) &pred->lnode.next;
	edges[n].old_target = old_cell;
	edges[n].new_target = new_cell;
	n++;
	edges[n].tag = URCU_MCAS_TAG;	/* ordered-cell edge */
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
	edges[n].tag = URCU_MCAS_TAG;
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
		struct cds_ft_node *freeze_leaf)
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
	 * cell edges.  FT_HLIST_TAG (== URCU_MCAS_TAG) so a reader resolves a parked
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
			uintptr_t old = state_meta->state;

			ft_state_edge(&edges[n], &state_meta->state, old,
				old - FT_STATE_NR_CHILD_ONE);
			n++;
		}
		if (ft_ord_cell_flip_into(ft, txn, edges, n) > 0)
			return -EAGAIN;	/* peer won: nothing installed, caller retries */
	} else {
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
static void ft_reparent_record_meta(struct ft_flip_txn *txn,
		struct cds_ft_metadata *meta,
		struct cds_ft_inode_flag *parent_nf,
		struct cds_ft_inode_flag **slot);

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
		ft_reparent_record_meta(txn, meta, new_parent, slot);
		return;
	} else if (ft->ordered_list) {
		field = &ft_ord_cell_ptr(
			((struct cds_ft_node *) child)->prev)->parent;
	} else {
		field = (struct cds_ft_inode_flag **)
			&((struct cds_ft_node *) child)->prev;
	}
	ft_pub_rec_add(rec, field, new_parent);
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
#define FT_REMOVE_COMMIT_REC_MAX_EDGES	9	/* <=3 structural (+back-edge) + <=5 cell/run (unsplice = 2 back-edges + deletion mark) + 1 DEL-recompact tombstone */
static
enum urcu_txn_status ft_remove_commit_rec(struct cds_ft *ft,
		struct ft_pub_rec *rec,
		struct ft_ord_cell *dead_cell, struct ft_detach_run *run,
		struct ft_flip_txn *txn)
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

/*
 * Locate the ordered-list neighbours (@pred, @succ) that a run grafted at @key
 * will splice between.  MUST be called while @dst is still payload-free (before
 * the structural attach publishes the grafted subtree), else the relational
 * descent would return a payload head as the boundary.  @key is APPLICATION form
 * (find_rel remaps).  Since the attach point is empty, @pred = last @dst key <
 * @key and @succ = first @dst key > @key (nothing of @dst's lies in the run's
 * range in between).
 */
static
void ft_ord_cell_find_splice_pos(struct cds_ft *dst, const uint8_t *key,
		size_t key_len, struct ft_ord_cell **pred_out,
		struct ft_ord_cell **succ_out)
{
	struct ft_ord_cell *pred, *succ;
	size_t flen = dst->group->key_len;

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
		pred = ft_ord_cell_find_rel(dst, pad, flen, FT_LOOKUP_LT);
		if (pred) {
			succ = ft_ord_cell_resolve_ord(&pred->lnode.next);
		} else {
			memset(pad + key_len, pad_max, flen - key_len);
			succ = ft_ord_cell_find_rel(dst, pad, flen,
					FT_LOOKUP_GT);
		}
		*pred_out = pred;
		*succ_out = succ;
		return;
	}

	pred = ft_ord_cell_find_rel(dst, key, key_len, FT_LOOKUP_LT);
	if (pred)
		succ = ft_ord_cell_resolve_ord(&pred->lnode.next);
	else
		succ = ft_ord_cell_find_rel(dst, key, key_len, FT_LOOKUP_GT);
	*pred_out = pred;
	*succ_out = succ;
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
 * Pre-sets the run's outer links (run not yet reachable in @dst) and APPENDS
 * the <=2 neighbour edges plus any @dst head/tail repair to @edges, leaving the
 * caller to flip them.  Split out (the appear-side dual of
 * ft_ord_cell_run_detach_edges) so a bulk graft can FUSE these edges with its
 * structural attach publish in ONE flip (ft_store_at_graft_point's batch),
 * closing the appear-side cross-view window; ft_ord_cell_run_splice is the
 * standalone (two-commit) wrapper.
 */
static
unsigned int ft_ord_cell_run_splice_edges(struct cds_ft *dst,
		struct ft_ord_cell *run_first, struct ft_ord_cell *run_last,
		struct ft_ord_cell *pred, struct ft_ord_cell *succ,
		struct ft_ord_cell_edge *edges, unsigned int n)
{
	/*
	 * Sentinel topology: a NULL boundary neighbour (run at @dst's head / tail)
	 * IS @dst's sentinel, so the run's outer link and the back-edge land on
	 * &dst->ord_sentinel.node -- recording the old head / tail flip without a
	 * separate endpoint edge.
	 */
	pred = ft_ord_or_sentinel(dst, pred);
	succ = ft_ord_or_sentinel(dst, succ);
	/* Pre-set the run's outer links; not yet reachable via @dst's list. */
	run_first->lnode.prev = ft_ord_cell_lnode(pred);
	run_last->lnode.next = ft_ord_cell_lnode(succ);
	edges[n].tag = URCU_MCAS_TAG;	/* ordered-cell edge */
	edges[n].slot = (struct ft_ord_cell **) &pred->lnode.next;
	edges[n].old_target = succ;
	edges[n].new_target = run_first;
	n++;
	edges[n].tag = URCU_MCAS_TAG;	/* ordered-cell edge */
	edges[n].slot = (struct ft_ord_cell **) &succ->lnode.prev;
	edges[n].old_target = pred;
	edges[n].new_target = run_last;
	n++;
	return n;
}

/* Max edges a run-splice commits: the two boundary back-edges. */
#define FT_ORD_CELL_RUN_SPLICE_MAX_EDGES	2

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
		/* Pre-set run_S's outer links; not yet reachable via @dst. */
		s_first->lnode.prev = ft_ord_cell_lnode(pred);
		s_last->lnode.next = ft_ord_cell_lnode(succ);
	}
	/*
	 * Sentinel topology: @pred / @succ resolve to @dst's sentinel when run_D is
	 * at @dst's head / tail, so &pred->lnode.next / &succ->lnode.prev IS the old
	 * head / tail repair.  An empty swap (s_first NULL) closes the gap: new_first
	 * = succ, new_last = pred (the sentinel when run_D spanned the whole list).
	 */
	edges[n].tag = URCU_MCAS_TAG;	/* ordered-cell edge */
	edges[n].slot = (struct ft_ord_cell **) &pred->lnode.next;
	edges[n].old_target = d_first;
	edges[n].new_target = new_first;
	n++;
	edges[n].tag = URCU_MCAS_TAG;	/* ordered-cell edge */
	edges[n].slot = (struct ft_ord_cell **) &succ->lnode.prev;
	edges[n].old_target = d_last;
	edges[n].new_target = new_last;
	n++;
	return n;
}

/* Max edges a run-replace commits: the two boundary back-edges. */
#define FT_ORD_CELL_RUN_REPLACE_MAX_EDGES	2

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

/* Max edges a glue-path publish-replace commits: <=2 structural + <=4 replace. */
#define FT_GLUE_PUBLISH_REPLACE_MAX_EDGES	6

/*
 * Commit a graft_swap's RECORDED structural publish edges (@rec: the forward
 * parent slot, plus a compressed parent's SKIP_X dual) ATOMICALLY with @run's
 * ordered-list run-replace, in ONE flip through the caller-PRE-RESERVED txn @txn
 * (the legacy KEY_SHORTER graft_swap commit path).  This runs in the op's
 * failure-free section (after the per-side drains), so it is UN-ABORTABLE: the
 * caller reserves @txn (capacity >= FT_GLUE_PUBLISH_REPLACE_MAX_EDGES) in the
 * graft_swap fallible prefix and ft_ord_cell_flip_into commits it here
 * infallibly.  Arms @run.
 */
static
void ft_ord_cell_flip_rec_replace(struct cds_ft *ft, struct ft_flip_txn *txn,
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
	/* Bulk op, not yet MW-hardened: ABORT unreachable under its exclusion. */
	(void) ft_ord_cell_flip_into(ft, txn, edges, n);
	run->armed = true;
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
	edges[n].tag = URCU_MCAS_TAG;	/* ordered-cell edge */
	edges[n].slot = (struct ft_ord_cell **) &pred->lnode.next;
	edges[n].old_target = first;
	edges[n].new_target = succ;
	n++;
	edges[n].tag = URCU_MCAS_TAG;	/* ordered-cell edge */
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
			(struct cds_ft_inode *) cn)->parent = value;
		return;
	}
	if (ft_node_compressed(child)) {
		struct cds_ft_compressed_node *cn = ft_compressed_node_ptr(child);

		cds_ft_item_to_metadata(
			(struct cds_ft_inode *) cn)->parent = value;
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
	cds_ft_item_to_metadata(ft_node_ptr(child))->parent = value;
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

		ft_flip_txn_record_tag(t, (void **) &m->nr_keys,
			(void *) old_raw, (void *) new_raw,
			FT_NR_KEYS_PROXY_TAG);
		cur = m->parent;
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
 * Fill @r with a generous SUPERSET of the nodes a bulk op's commit can allocate
 * -- CDS_FT_ALLOC_RESERVE_CAP of every internal node type (its own order +
 * bitmap), the minimal order, and (speculative groups) the compressed-node
 * orders.  Drawn before the op's last fallible step, this lets the commit draw
 * and never fail on an arena allocation, so nothing after that step needs a
 * reader-observable rollback.  Used by the same-trie rekey (before its detach)
 * and by ft_graft_keylen's NOSPLIT attach (before it publishes the empty source
 * root).  A generous superset avoids predicting the exact manifest; a bulk op
 * already pays an RCU grace period, so the handful of throwaway arena
 * pops/pushes is negligible.  Returns 0, or -ENOMEM (caller drains).
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
};

struct ft_glue_free_item {
	void *node;		/* cds_ft_inode * or cds_ft_compressed_node * */
	bool compressed;
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
	 * The demoted @src_head's ordered-list cell, captured by
	 * ft_glue_record_splices.  It stays REACHABLE through its src-run
	 * neighbours' stale ord_prev/ord_next until the post-publish interleave
	 * rewires them, so it is freed only by
	 * ft_glue_free_collided_cells, called after the interleave, via
	 * the grace-period-deferred cell free.  NULL when the list is off.
	 */
	struct ft_ord_cell *src_cell;
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
	g->attached_nf = NULL;
	g->txn = NULL;
	g->fuse_free_list = false;
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
		ft_flip_txn_record_reserved(txn, (void **) &cn_meta->parent,
			cn_meta->parent, parent_nf);
		return;
	}
	if (ft_node_compressed(child_nf)) {
		struct cds_ft_compressed_node *cn =
			ft_compressed_node_ptr(child_nf);
		struct cds_ft_metadata *cn_meta =
			cds_ft_item_to_metadata((struct cds_ft_inode *) cn);

		ft_set_parent_slot(cn_meta, parent_nf, slot);
		ft_flip_txn_record_reserved(txn, (void **) &cn_meta->parent,
			cn_meta->parent, parent_nf);
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
		ft_flip_txn_record_reserved(txn, (void **) &meta->parent,
			meta->parent, parent_nf);
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
void ft_reparent_record_meta(struct ft_flip_txn *txn,
		struct cds_ft_metadata *meta,
		struct cds_ft_inode_flag *parent_nf,
		struct cds_ft_inode_flag **slot)
{
	uintptr_t old_state = meta->state, new_state = old_state;

	if (slot) {
		unsigned int off = parent_nf ? (unsigned int) ((char *) slot -
			(char *) ft_node_ptr(parent_nf)) / sizeof(void *) : 0;

		new_state = (old_state & ~FT_STATE_PSO_MASK)
			| (((uintptr_t) off & FT_STATE_PSO_VALMASK)
				<< FT_STATE_PSO_SHIFT);
		if (parent_nf && !ft_node_compressed(parent_nf)
#ifdef FEATURE_FT_SKIP_COMPRESSED
				&& !ft_node_skip_compressed(parent_nf)
#endif
		   )
			meta->incoming_byte = ft_slot_to_byte(
				&ft_types[ft_node_type(parent_nf)],
				ft_node_ptr(parent_nf), slot);
	}
	ft_flip_txn_record_reserved(txn, (void **) &meta->parent,
		meta->parent, parent_nf);
	ft_flip_txn_record_tag(txn, (void **) &meta->state,
		(void *) old_state, (void *) new_state, FT_STATE_PROXY);
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
		struct cds_ft_inode_flag **slot)
{
	if (!child_nf)
		return;
	if (caa_unlikely(ft_node_flip_proxy(child_nf)))
		return;
#ifdef FEATURE_FT_SKIP_COMPRESSED
	if (ft_node_skip_compressed(child_nf)) {
		struct cds_ft_compressed_node *cn =
			ft_skip_to_compressed(ft, child_nf);

		ft_reparent_record_meta(txn,
			cds_ft_item_to_metadata((struct cds_ft_inode *) cn),
			parent_nf, slot);
		return;
	}
	if (ft_node_compressed(child_nf)) {
		struct cds_ft_compressed_node *cn =
			ft_compressed_node_ptr(child_nf);

		ft_reparent_record_meta(txn,
			cds_ft_item_to_metadata((struct cds_ft_inode *) cn),
			parent_nf, slot);
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
	ft_reparent_record_meta(txn,
		cds_ft_item_to_metadata(ft_node_ptr(child_nf)), parent_nf, slot);
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

/* Record an old (replaced) live node to reclaim deferred at commit. */
static
void ft_glue_defer_free(struct ft_glue *g,
		void *node, bool compressed)
{
	assert(g->nr_free < g->cap_free);
	g->free_list[g->nr_free].node = node;
	g->free_list[g->nr_free].compressed = compressed;
	g->nr_free++;
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
		 * Fuse the freeze into @txn (committed with the forward publish
		 * below) when the committer reserved for it; else a standalone
		 * lone-edge flip.  urcu_txn_store upgrades a repeat slot in place,
		 * so a second apply_deferred pass costs no extra reservation.
		 */
		if (g->fuse_free_list)
			ft_flip_txn_record_tombstone(g->txn, meta);
		else
			ft_meta_tombstone_set_flip(meta);
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
	for (i = 0; i < g->nr_deferred; i++)
		if (!g->deferred[i].dst_origin)
			ft_set_parent(ft, g->deferred[i].child, g->deferred[i].parent,
				g->deferred[i].slot);
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
void ft_glue_txn_commit_edges(struct cds_ft *ft, struct ft_glue *g,
		const struct ft_ord_cell_edge *cedges, unsigned int n_cedges)
{
	struct ft_pub_rec rec = { .n = 0 };
	unsigned int j, k;
	int i;

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
	/* VALIDATE (§4.B): guard the LIVE dst parent this cluster publishes into. */
	ft_flip_txn_guard_parent(ft, g->txn, g->publish_parent);
	_ft_publish_to_parent(ft, g->publish_parent, g->publish_slot, g->top,
		&rec);
	for (j = 0; j < rec.n; j++)
		ft_flip_txn_record_reserved(g->txn, (void **) rec.slot[j],
			rec.old_val[j], rec.new_val[j]);
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
	 * _edges helper already pre-set the run's own outer links (plain stores;
	 * the run is not ord-reachable in dst until the flip) and the wrapper
	 * armed the run descriptor so the caller skips the standalone splice.
	 */
	for (k = 0; k < n_cedges; k++)
		ft_flip_txn_record_tag(g->txn, (void **) cedges[k].slot,
			cedges[k].old_target, cedges[k].new_target,
			ft_edge_tag(&cedges[k]));

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
	 * Commit WITHOUT an explicit install: ft_flip_txn_commit auto-installs a
	 * multi-edge set, but a publish that reduces to a SINGLE recorded edge
	 * (e.g. a list-off attach into a plain parent with no deferred
	 * back-pointers) commits as a bare release store -- no proxy, no group
	 * flip, no grace-period reclaim -- so the common one-pointer publish pays
	 * nothing for the transaction machinery.  commit owns reclaim (deferred
	 * through the FT flavor when a proxy is owed, freed in place otherwise).
	 */
	ft_flip_txn_commit(ft, g->txn);
	g->txn = NULL;
}

/*
 * Splice wrapper (cds_ft_graft): fuse a run-splice into the attach flip-txn.
 * Computes the <=4 run-splice boundary edges (which also pre-set @run's outer
 * links), arms @run, and commits via the edge core.  @run NULL => list off.
 */
static
void ft_glue_txn_commit(struct cds_ft *ft, struct ft_glue *g,
		struct ft_graft_run *run)
{
	struct ft_ord_cell_edge cedges[4] = { 0 };
	unsigned int n = 0;

	if (run) {
		n = ft_ord_cell_run_splice_edges(ft, run->run_first,
			run->run_last, run->pred, run->succ, cedges, 0);
		run->armed = true;
	}
	ft_glue_txn_commit_edges(ft, g, cedges, n);
}

/*
 * Replace wrapper (cds_ft_graft_swap insert side): fuse a run-replace into the
 * replace flip-txn.  run_D leaves dst's ordered list and run_S takes its place;
 * the <=4 boundary edges (pre-setting run_S's outer links) join the structural
 * replace publish.  Arms @run.  @run NULL => list off.
 */
static
void ft_glue_txn_commit_replace(struct cds_ft *ft, struct ft_glue *g,
		struct ft_graft_swap_run *run)
{
	struct ft_ord_cell_edge cedges[4] = { 0 };
	unsigned int n = 0;

	if (run) {
		n = ft_ord_cell_run_replace_edges(ft, run->d_first, run->d_last,
			run->s_first, run->s_last, cedges, 0);
		run->armed = true;
	}
	ft_glue_txn_commit_edges(ft, g, cedges, n);
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
	g->nr_splices++;
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
void ft_glue_publish(struct cds_ft *ft, struct ft_flip_txn *txn,
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
	/* VALIDATE (§4.B): guard the LIVE dst parent this cluster publishes into. */
	ft_flip_txn_guard_parent(ft, txn, g->publish_parent);
	_ft_publish_to_parent(ft, g->publish_parent, g->publish_slot, g->top,
		&rec);
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
	/* Bulk op, not yet MW-hardened: ABORT unreachable under its exclusion. */
	(void) ft_ord_cell_flip_into(ft, txn, sedges, n);
}

/*
 * GLUE-path graft_swap publish FUSED with an ordered-list run-REPLACE (the
 * legacy KEY_SHORTER graft_swap commit path).  When @run is set, RECORD
 * the cluster's forward publish edge (plus a compressed parent's SKIP_X dual)
 * via a ft_pub_rec instead of storing it, append the run-replace boundary edges,
 * and commit them all in ONE flip, so a reader never sees run_S's keys
 * reachable in the structure but absent from the ordered list (or run_D the
 * reverse).  @run NULL (ordered list off) falls back to the plain publish.  Both
 * commit through the caller-PRE-RESERVED @txn (un-abortable failure-free
 * section); @txn capacity must be >= FT_GLUE_PUBLISH_REPLACE_MAX_EDGES.
 */
static
void ft_glue_publish_replace(struct cds_ft *ft, struct ft_flip_txn *txn,
		struct ft_glue *g, struct ft_graft_swap_run *run)
{
	struct ft_pub_rec rec = { .n = 0 };

	if (!run) {
		ft_glue_publish(ft, txn, g);
		return;
	}
	/* VALIDATE (§4.B): guard the LIVE dst parent this cluster publishes into. */
	ft_flip_txn_guard_parent(ft, txn, g->publish_parent);
	_ft_publish_to_parent(ft, g->publish_parent, g->publish_slot, g->top,
		&rec);
	/* Order-statistics fold (BULK): see ft_glue_publish. */
	if (g->count_delta)
		ft_flip_txn_record_count_parent(ft, txn, g->publish_parent,
			g->count_delta);
	ft_ord_cell_flip_rec_replace(ft, txn, &rec, run);
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
		if (g->free_list[i].compressed)
			free_compressed_node(ft, g->free_list[i].node);
		else
			free_cds_ft_node(ft, g->free_list[i].node);
	}
}
