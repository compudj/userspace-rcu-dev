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
 * One lock level's anchor, captured as the descent crosses it
 * (doc/design/ft-dlm-lock-coarseness.md §4).  @cover is the node whose
 * half-open byte span contains the level; @bound is the node starting at the
 * first node boundary at or after it, and @bound_start that boundary's byte.
 *
 * The two differ only inside a COMPRESSED node: a level strictly inside a span
 * has no node starting at it, so @cover is that compressed node and @bound is
 * its successor on the path (NULL until the descent reaches it).  Where a node
 * starts exactly at the level -- every level in a bushy trie -- @cover ==
 * @bound and @bound_start is the level itself.
 */
struct ft_lock_anchor {
	struct cds_ft_inode_flag *cover;
	struct cds_ft_inode_flag *bound;
	unsigned int bound_start;
};

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
	/*
	 * Lock-level anchor table, one slot per ft_lock_level_index(), filled by
	 * ft_descent_enter_node as the descent passes each level.  A lock-set
	 * member's anchor is then an O(1) lookup (ft_descent_anchor) rather than
	 * an up-walk: absolute byte-depth is unknown to a climb until it reaches
	 * the root, so a climb cannot stop early and ends without the node it
	 * walked past (doc/design/ft-dlm-lock-coarseness.md §5.3).
	 *
	 * Only slots the descent has CROSSED are readable, and a query at depth
	 * @d touches ft_lock_level_index(@d) <= the deepest crossed slot, so
	 * every reachable read is written first -- @anchor needs no init sweep on
	 * the mutation hot path.  @anchor_crossed records the written set so a
	 * debug build asserts that rather than trusting it.
	 */
	struct ft_lock_anchor anchor[FT_LOCK_LEVEL_MAX];
	uint16_t anchor_pending;		/* Levels awaiting their boundary node. */
	uint16_t anchor_crossed;		/* Levels written (debug validation). */
	enum cds_ft_lock_spacing lock_spacing;	/* The trie's lock granularity. */
	struct cds_ft_inode_flag *nf;		/* Current node-flag value. */
	struct cds_ft_inode_flag **nfp;		/* Slot that holds @nf. */
	struct cds_ft_inode_flag *pnf;		/* Parent node-flag value. */
	struct cds_ft_inode_flag **pnfp;	/* Slot that holds @pnf. */
	struct cds_ft_inode_flag *ppnf;		/* Grandparent node-flag value. */
	struct cds_ft_inode_flag **ppnfp;	/* Slot that holds @ppnf. */
	struct cds_ft_inode_flag *pppnf;	/* Great-grandparent node-flag value. */
	struct cds_ft_inode_flag **pppnfp;	/* Slot that holds @pppnf. */
	/*
	 * Byte-depth each window slot STARTS at, rotated with the slot itself.
	 * @depth alone cannot recover them: a compressed ancestor starts cn->len
	 * bytes back, not one, and the span is not derivable from the flag.  An
	 * acquire site anchors a lock-set member by its OWN depth
	 * (ft_descent_anchor), so a member taken from the window needs the depth
	 * that came with it.  Meaningful only where the matching slot is non-NULL.
	 */
	unsigned int pdepth;			/* Byte-depth of @pnf. */
	unsigned int ppdepth;			/* Byte-depth of @ppnf. */
	unsigned int pppdepth;			/* Byte-depth of @pppnf. */
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

/*
 * Record @nf, spanning key bytes [@start, @start + @len), into @d's anchor
 * table.  A node starts exactly where its predecessor ended, so entering one
 * resolves every level left pending by that predecessor; the node then covers
 * each lock level inside its own span.
 */
static inline
void ft_descent_enter_node(struct ft_descent *d, struct cds_ft_inode_flag *nf,
		unsigned int start, unsigned int len)
{
	unsigned int lvl;

	/*
	 * Per-node granularity anchors every member on itself, so it reads no
	 * table and builds none -- the zero-cost path the default rests on.
	 * Root-only needs the FIRST node and nothing after it.
	 */
	if (d->lock_spacing == CDS_FT_LOCK_SPACING_PER_NODE)
		return;
	if (d->lock_spacing == CDS_FT_LOCK_SPACING_ROOT_ONLY) {
		if (!d->anchor_crossed) {
			d->anchor[0].cover = nf;
			d->anchor[0].bound = nf;
			d->anchor[0].bound_start = 0;
			d->anchor_crossed = 1;
		}
		return;
	}
	/* This node IS the boundary the pending levels were waiting for. */
	while (caa_unlikely(d->anchor_pending != 0)) {
		unsigned int i = (unsigned int) __builtin_ctz(d->anchor_pending);

		d->anchor[i].bound = nf;
		d->anchor_pending &= (uint16_t) ~(1U << i);
	}
	/* Levels inside [@start, @start + @len) are covered by this node. */
	lvl = ft_lock_level(start);
	if (lvl < start)
		lvl = lvl ? lvl << 1 : 1;
	for (; lvl < start + len; lvl = lvl ? lvl << 1 : 1) {
		unsigned int i = ft_lock_level_index(lvl);

		d->anchor[i].cover = nf;
		d->anchor_crossed |= (uint16_t) (1U << i);
		if (lvl == start) {
			d->anchor[i].bound = nf;
			d->anchor[i].bound_start = start;
		} else {
			/* No node starts at @lvl: its boundary is this span's end. */
			d->anchor[i].bound = NULL;
			d->anchor[i].bound_start = start + len;
			d->anchor_pending |= (uint16_t) (1U << i);
		}
	}
}

/*
 * The anchor for a node at byte-depth @depth: the node starting at the first
 * node boundary at or after ft_lock_level(@depth), or -- with no boundary in
 * [level, @depth] -- the node whose span contains that level
 * (doc/design/ft-dlm-lock-coarseness.md §2).  The clamp keeps the result an
 * ancestor-or-self of the queried node.
 *
 * @depth must be at most @d->depth: the table holds the levels the descent has
 * PASSED, and a member BELOW the cursor is resolved by its caller from the two
 * candidate boundaries it already holds (§7.1), not here.
 *
 * The cursor's own node is what the table cannot carry: it spans [@d->depth, ...)
 * and has not been entered.  It is nonetheless a boundary, in two ways -- the
 * level can fall exactly on it (@depth a power of two equal to @d->depth), and
 * it is the boundary a still-pending level is waiting for, since a level goes
 * pending only from the LAST entered node and that node ends at @d->depth.
 */
/*
 * The node starting at the first node boundary at or after lock level @lvl,
 * taking no boundary later than @clamp; the coverer of @lvl if none qualifies.
 * @lvl must be a lock level the descent has crossed (or the cursor's own).
 *
 * @clamp is a separate argument because it is NOT always the depth that chose
 * the level: an immediate child of the cursor takes ITS level but clamps at its
 * OWN depth, which lies past the cursor.
 */
static inline
struct cds_ft_inode_flag *ft_descent_anchor_at_level(const struct ft_descent *d,
		unsigned int lvl, unsigned int clamp)
{
	const struct ft_lock_anchor *a;
	unsigned int i;

	if (lvl == d->depth)
		return d->nf;
	i = ft_lock_level_index(lvl);
	a = &d->anchor[i];
	assert(d->anchor_crossed & (1U << i));
	if (a->bound_start <= clamp)
		return a->bound ? a->bound : d->nf;
	return a->cover;
}

static inline
struct cds_ft_inode_flag *ft_descent_anchor(const struct ft_descent *d,
		unsigned int depth)
{
	assert(depth <= d->depth);
	return ft_descent_anchor_at_level(d, ft_lock_level(depth), depth);
}

/*
 * The anchor for @child_nf, the cursor's IMMEDIATE child, sitting at byte-depth
 * @child_depth -- the below-cursor case ft_descent_anchor refuses (§7.1).
 *
 * Only two boundaries lie in (@d->depth, @child_depth]: the cursor's own start
 * and the child's, because the cursor spans the whole gap -- one slot hop, or a
 * compressed run of cn->len bytes.  So the child anchors on ITSELF when its
 * level falls past the cursor, and otherwise on whatever the table already
 * holds for that level.  Callers with a deeper member must extend the descent
 * rather than reach further with this.
 */
static inline
struct cds_ft_inode_flag *ft_descent_anchor_child(const struct ft_descent *d,
		struct cds_ft_inode_flag *child_nf, unsigned int child_depth)
{
	unsigned int lvl = ft_lock_level(child_depth);

	assert(child_depth > d->depth);
	if (lvl > d->depth)
		return child_nf;
	return ft_descent_anchor_at_level(d, lvl, child_depth);
}

/*
 * The byte-depth the descent recorded for @nf, matched against the four window
 * slots.  An acquire site resolves several of its lock-set members by a one-hop
 * back-pointer (ft_resolve_parent_slot), which yields a node carrying NO depth
 * at all, while a depth SCALAR describes only the member the caller was handed:
 * a site locking {C, P, GP} needs the WINDOW depths for P and GP
 * (doc/design/ft-dlm-lock-coarseness.md §9).
 *
 * FALSE when the descent does not describe @nf.  That is not an error but a
 * RE-PLAN: a back-pointer resolving to a node this descent never passed is a
 * node whose depth is unknown here, and anchoring it with another node's depth
 * puts two ops on different anchors for one node -- the exact disagreement §1
 * forbids.  ft_insert_dlm_acquire_split already follows this rule for P.
 */
static inline
bool ft_descent_depth_of(const struct ft_descent *d,
		const struct cds_ft_inode_flag *nf, unsigned int *depth)
{
	if (!d || !nf)
		return false;
	if (nf == d->nf)
		*depth = d->depth;
	else if (nf == d->pnf)
		*depth = d->pdepth;
	else if (nf == d->ppnf)
		*depth = d->ppdepth;
	else if (nf == d->pppnf)
		*depth = d->pppdepth;
	else
		return false;
	return true;
}


/*
 * Exercise the anchor LOOKUP from the descent itself, at exactly the depths an
 * acquire site queries -- the cursor and the three ancestors the window carries.
 * Until an acquire site consumes the table, ft_descent_enter_node is the only
 * part the suites reach; this gate makes the lookup reachable too, so a build
 * under CDS_FT_LOCK_SPACING=exponential covers both halves.
 */
#ifdef FEATURE_FT_ANCHOR_VALIDATE
static inline
void ft_descent_anchor_validate(const struct ft_descent *d)
{
	unsigned int back;

	/*
	 * Window depths are ordered and strictly shallower than the cursor: every
	 * node spans at least one key byte, so a slot can never start where the
	 * one below it does.  This holds under every granularity.
	 */
	if (d->pnf)
		assert(d->pdepth < d->depth);
	if (d->ppnf)
		assert(d->ppdepth < d->pdepth);
	if (d->pppnf)
		assert(d->pppdepth < d->ppdepth);
	if (d->lock_spacing == CDS_FT_LOCK_SPACING_PER_NODE)
		return;
	if (d->lock_spacing == CDS_FT_LOCK_SPACING_ROOT_ONLY) {
		assert(d->anchor_crossed & 1U);
		assert(d->anchor[0].cover != NULL);
		return;
	}
	/*
	 * A NULL cursor means the descent walked off the trie (an absent
	 * child): there is no node at @d->depth to anchor, and the two arms
	 * that resolve to the cursor would report its absence, not a gap in
	 * the table.
	 */
	if (!d->nf)
		return;
	for (back = 0; back < 4; back++) {
		if (back > d->depth)
			break;
		assert(ft_descent_anchor(d, d->depth - back) != NULL);
	}
}
#else
static inline
void ft_descent_anchor_validate(const struct ft_descent *d __attribute__((unused)))
{
}
#endif

/*
 * The lock-set anchor for @nf, a node at byte-depth @depth that the acquire
 * would otherwise lock directly.  This is the call-site-facing form: it applies
 * the trie's granularity, so an acquire site asks for an anchor uniformly and
 * per-node granularity hands back the node itself.
 *
 * Members mapping to ONE anchor must be acquired ONCE -- a second ft_dlm_lock on
 * a held node aborts -EAGAIN -- so a caller with several members dedupes on the
 * returned pointer (doc/design/ft-dlm-lock-coarseness.md §7.3).
 *
 * A member BELOW the cursor -- the chain-compress set's surviving child is the
 * canonical one (§7.1) -- resolves through ft_descent_anchor_child, which is
 * exact for ONE hop and no further: past that the descent skipped boundaries the
 * table never saw, and such a caller must extend the descent rather than reach
 * deeper from here.
 */
static inline
struct cds_ft_inode_flag *ft_descent_anchor_of(const struct ft_descent *d,
		struct cds_ft_inode_flag *nf, unsigned int depth)
{
	switch (d->lock_spacing) {
	case CDS_FT_LOCK_SPACING_PER_NODE:
		return nf;
	case CDS_FT_LOCK_SPACING_ROOT_ONLY:
		/*
		 * The first node ANY descent enters is the trie's root, so
		 * @anchor[0] normally holds it.  A descent that never ADVANCED
		 * entered nothing -- both advance paths enter the node they
		 * LEAVE, and a merge whose point IS the root descends an empty
		 * key, so its loop never steps -- and then the cursor is still
		 * that root.  Either way the anchor is the root.
		 */
		if (caa_unlikely(!(d->anchor_crossed & 1U))) {
			assert(!d->depth);
			return d->nf;
		}
		return d->anchor[0].cover;
	case CDS_FT_LOCK_SPACING_EXPONENTIAL:
	default:
		/*
		 * A node starting ON a lock level never reaches here --
		 * ft_anchor_meta settles it from the depth alone.  What is left
		 * is the depths that genuinely need the table, and the table
		 * describes the path the DESCENT took: for a node it never
		 * passed (the detach's orphan walk leaves the key path, a
		 * sibling reached by back-pointer was never on it) the arms
		 * below answer from the wrong path -- the CURSOR's node, or
		 * NULL where the descent walked off the trie.  Such a caller
		 * must extend the descent rather than reach further from here.
		 */
		if (depth > d->depth)
			return ft_descent_anchor_child(d, nf, depth);
		return ft_descent_anchor(d, depth);
	}
}

/*
 * THE ACQUIRE CHOKE POINT.  Every lock-set member resolves through here to the
 * metadata its acquire must actually take: the node's own under per-node
 * granularity, its anchor's under a coarser one.  Sites call this instead of
 * deriving metadata from the member flag directly, so the mapping lives in ONE
 * place -- agreement is a property of every site computing the SAME anchor for
 * a node, which is not something 40 independent derivations can be trusted to
 * preserve (doc/design/ft-dlm-lock-coarseness.md §1, §9).
 *
 * @d may be NULL where no descent ran; that is legal ONLY under per-node
 * granularity, where no depth is needed, and is asserted as such.
 */
static inline
struct cds_ft_metadata *ft_anchor_meta(const struct cds_ft *ft,
		const struct ft_descent *d, struct cds_ft_inode_flag *nf,
		struct cds_ft_metadata *node, unsigned int depth)
{
	struct cds_ft_inode_flag *anchor;

	if (ft->lock_spacing == CDS_FT_LOCK_SPACING_PER_NODE)
		return node;
	/*
	 * Byte-depth 0 is the ROOT, and the root is its own anchor under every
	 * spacing: it covers lock level 0, and no boundary lies above it.  So a
	 * root-level acquire needs no descent, which is what lets the
	 * descent-less root fences anchor at all.
	 *
	 * ★ 0 IS A POSITION, NOT AN "UNKNOWN".  A caller that could not date its
	 * member and leaves the depth at its initializer arrives here, and this
	 * arm answers -- anchoring that node on ITSELF while every op that dates
	 * it anchors on an ancestor, so the two exclude nothing (§1).  An undated
	 * member is FT_DEPTH_FROM_DESCENT, which the acquire sites refuse.
	 *
	 * So CHECK the claim rather than trust it: depth 0 must mean this node
	 * really has no parent.  The failure it catches is silent, coarse-only,
	 * and reads as a lost update three layers away, so nothing downstream
	 * will report it for you.
	 *
	 * STANDING as of the merge spine's absolute dating (1c288c07).  It was
	 * opt-in behind FEATURE_FT_ANCHOR_VALIDATE for one reason -- "a member
	 * dated RELATIVE to its op's own origin lands here too, and the merge
	 * spine still does that below its first hop" -- and that reason died with
	 * ft_merge_build's @dst_base_depth / @src_base_depth: every fence now
	 * arrives dated from the trie root, not from the recursion.  An assert
	 * with ONE opt-in config is an assert almost nobody runs, and this class
	 * has already produced two hard defects (@becb4528, @1c288c07).
	 */
	if (!depth) {
		assert(ft_node_flip_proxy(node->parent_word) ||
			!ft_parent_node(node->parent_word));
		return node;
	}
	/*
	 * A node starting ON a lock level is the first boundary at that level,
	 * so §2 settles its anchor from @depth alone -- the same rule the root
	 * case above is, at level 0.  No descent is read, so a DESCENT-LESS
	 * site is legal for it: the assert below guards only the depths that
	 * genuinely need a table.
	 */
	if (ft->lock_spacing == CDS_FT_LOCK_SPACING_EXPONENTIAL &&
			ft_lock_level(depth) == depth)
		return node;
	/*
	 * Root-only anchors every member on the trie's ROOT, which a
	 * descent-less site can name directly -- ft_descent_init reads this
	 * same slot the same way, so both routes answer with one node, which is
	 * what §1's agreement asks.
	 */
	if (ft->lock_spacing == CDS_FT_LOCK_SPACING_ROOT_ONLY && !d)
		return ft_flag_to_metadata(ft, ft_resolve_flip_proxy(
			rcu_dereference(ft->root)));
	assert(d);
	anchor = ft_descent_anchor_of(d, nf, depth);
	/*
	 * @node, never a re-derivation, whenever the anchor IS the member: a
	 * flag reconstructed from a node pointer and a type index is wrong for
	 * every node kind whose metadata is not at the internal-node offset (a
	 * compressed node reached through a skip pointer, above all), and the
	 * acquire would then fence a DIFFERENT word than the one the caller
	 * retires.  Only a genuine ancestor -- a flag the descent stored, and so
	 * well-formed -- is resolved here.
	 */
	return anchor == nf ? node : ft_flag_to_metadata(ft, anchor);
}

/*
 * A HELD lock-set member, once coarsening has split a node's two words apart.
 *
 * @lock is the word the acquire actually CAS'd -- the member's ANCHOR -- with
 * its clean snapshot @lock_snap.  EVERY lock-lifecycle operation names that
 * pair: ft_meta_lock_release, ft_flip_txn_lock_register, and the
 * {LOCK|s -> s} release record.  Naming the node instead releases a word this
 * op never locked and leaks the one it did.
 *
 * @node_snap is the clean word of the PROTECTED node itself, which is what a
 * RETIRE tombstones.  It is sampled before the acquire and ratified by it
 * (ft_held_anchor_sample_node / ft_held_anchor_guard_node), because the anchor
 * excludes the node's mutators only from the linearization point on.
 *
 * Per-node granularity makes @lock the node's own metadata and the two
 * snapshots equal, which is why the two terminals fuse there into the single
 * {LOCK|s -> TOMBSTONE|s} transition.
 */
struct ft_held_anchor {
	struct cds_ft_metadata *lock;
	uintptr_t lock_snap;
	uintptr_t node_snap;
	/*
	 * The op ALREADY held @lock when this member asked for it: coarsening
	 * collapsed two of its members onto one word.  The member is protected --
	 * by the op's own earlier acquire -- but it owes no release and no
	 * terminal, both of which that earlier acquire recorded.  A second
	 * release would drop the word while the op still writes under it, and a
	 * second terminal would record the single word twice.
	 */
	bool shared;
	/*
	 * The op already holds @node's OWN word -- an EARLIER member of this op
	 * anchored ON @node, so a coarsened member arrives at a node its own op
	 * has marked.  Distinct from @shared, which is about the ANCHOR: this one
	 * says the RETIRE must take the fused shape (drop LOCK, set TOMBSTONE)
	 * rather than expect a clean word, and that the acquire's node guard is
	 * redundant -- holding the word already excludes @node's mutators.
	 */
	bool node_held;
	/*
	 * A txn now OWNS this mark's outcome: it carries the {LOCK|s -> s}
	 * release in its edge set AND the word in its locks[] registry, so a
	 * commit OK consumes the mark and every other terminal drains it.  The
	 * holder that took the mark must stop sweeping it -- one owner per fence.
	 *
	 * ★ Set only where the anchor SURVIVES the commit, and there it is
	 * MANDATORY, because such a word is clean and re-lockable the instant the
	 * release settles: a sweep that reaches it afterwards and asks "is it
	 * still locked?" gets YES from a PEER'S fresh mark and clears that.  The
	 * question has no answer at the word -- an owner-less LOCK bit cannot say
	 * whose it is -- so the outcome must be tracked, not sampled.  A mark
	 * whose terminal is the node's own TOMBSTONE needs none of this: the
	 * acquire refuses a TOMBSTONE, so nobody re-locks that word and a
	 * still-set LOCK there can only be ours.
	 */
	bool txn_owned;
};

/*
 * Sample @node's own word for a member whose lock is going to @anchor, a
 * DIFFERENT word.  -EAGAIN if it is dirty (PROXY | TOMBSTONE | LOCK), which is
 * the same refusal ft_dlm_lock gives for the word it locks: per-node
 * granularity gets that check for free because the two words are one, and
 * coarsening must not lose it -- the retire tombstones @node against this
 * snapshot, so a dirty value here is a retire that could only ever abort.
 *
 * Call BEFORE the acquire and validate the result into the acquire's own commit
 * (ft_held_anchor_guard_node): the anchor excludes every mutator of @node only
 * from the linearization point on, so the window between this sample and the
 * commit needs the engine's read-set arbitration, exactly as the plan's racy
 * parent resolution does.
 *
 * ★ THE OP'S OWN MARK IS NOT A PEER'S -- see ft_member_node_snap.  A raw sample
 * cannot tell them apart, and refusing one's own mark is a refusal no RETRY can
 * clear (measured: 200M descents and 9M -EAGAIN from ONE detach site, under a
 * coarse spacing that had anchored an earlier member on this very node).
 */
static inline
int ft_held_anchor_sample_node(const struct cds_ft_metadata *node,
		uintptr_t *node_snap)
{
	uintptr_t s = CMM_LOAD_SHARED(node->state);

	if (caa_unlikely(s & (FT_STATE_PROXY | FT_STATE_TOMBSTONE |
			FT_STATE_LOCK)))
		return -EAGAIN;
	*node_snap = s;
	return 0;
}

/*
 * Record that this op holds @node's lock-set member: @lock is the word the
 * acquire CAS'd and @lock_snap the clean value it captured there, @node_snap
 * the value ft_held_anchor_sample_node ratified for @node itself (ignored, and
 * equal to @lock_snap, when the two words coincide).
 */
static inline
void ft_held_anchor_set(struct ft_held_anchor *h, struct cds_ft_metadata *lock,
		uintptr_t lock_snap, const struct cds_ft_metadata *node,
		uintptr_t node_snap)
{
	h->lock = lock;
	h->lock_snap = lock_snap;
	h->node_snap = lock == node ? lock_snap : node_snap;
	h->shared = false;
	h->node_held = false;
	h->txn_owned = false;
}

static
void ft_descent_init(struct ft_descent *d, struct cds_ft *ft)
{
	d->depth = 0;
	d->pdepth = 0;
	d->ppdepth = 0;
	d->pppdepth = 0;
	d->anchor_pending = 0;
	d->anchor_crossed = 0;
	d->lock_spacing = ft->lock_spacing;
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
	d->pppdepth = d->ppdepth;
	d->ppnf  = d->pnf;
	d->ppnfp = d->pnfp;
	d->ppdepth = d->pdepth;
	d->pnf   = d->nf;
	d->pnfp  = d->nfp;
	d->pdepth = d->depth;
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
	/* @cn spans [d->depth, d->depth + cn->len) -- one lock, many levels. */
	ft_descent_enter_node(d, d->pnf, d->depth, cn->len);
	d->depth += cn->len;
	ft_descent_anchor_validate(d);
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
	d->pppdepth = d->ppdepth;
	d->ppnf  = d->pnf;
	d->ppnfp = d->pnfp;
	d->ppdepth = d->pdepth;
	d->pnf   = d->nf;
	d->pnfp  = d->nfp;
	d->pdepth = d->depth;
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
	/* The node just traversed spans one key byte, [d->depth, d->depth + 1). */
	ft_descent_enter_node(d, d->pnf, d->depth, 1);
	d->depth++;
	ft_descent_anchor_validate(d);
	return d->nf;
}

#ifdef FEATURE_FT_PROBE_GRAFT_SWAP
extern unsigned long cds_ft_probe_gs_pubabort;
extern unsigned long cds_ft_probe_gs_pubok;
#define FT_GS_PROBE_INC(c)	__atomic_fetch_add(&(c), 1, __ATOMIC_RELAXED)
#else
#define FT_GS_PROBE_INC(c)	do { } while (0)
#endif

#ifdef FEATURE_FT_PROBE_PROMOTE
extern unsigned long cds_ft_probe_promote_deferred;
extern unsigned long cds_ft_probe_promote_immediate;
extern unsigned long cds_ft_probe_promote_guarded;
#define FT_PROMOTE_PROBE_INC(c)	__atomic_fetch_add(&(c), 1, __ATOMIC_RELAXED)
#else
#define FT_PROMOTE_PROBE_INC(c)	do { } while (0)
#endif

/*
 * A TRIE ROOT LIVES IN NO NODE, so no lock-set can own it and no structural
 * record may park it SW.  ft_flip_txn_record_root is the mechanism that
 * enforces this (see it for the argument); these three macros are the
 * --enable-rcu-debug DETECTOR that no other record helper reaches a root slot
 * behind the mechanism's back.
 *
 * Armed regardless of @structural_sw, so the check has coverage NOW -- before
 * ft_txn_content_sw_ok arms anything -- rather than only once a wrongly-routed
 * root has become a silent erasure.  A release build carries no field, no
 * store and no compare.
 */
#if defined(DEBUG_RCU) || defined(CONFIG_RCU_DEBUG)
# define FT_ROOT_ASSERT_TXN_FIELD	void **dbg_root_slot;
# define FT_ROOT_ASSERT_INIT(t, ft)					\
	do {								\
		(t)->dbg_root_slot = (ft) ?				\
			(void **) &(ft)->root : NULL;			\
	} while (0)
# define FT_ROOT_ASSERT_NOT_ROOT(t, slot)				\
	urcu_assert_debug((void **) (slot) != (t)->dbg_root_slot)
#else
# define FT_ROOT_ASSERT_TXN_FIELD
# define FT_ROOT_ASSERT_INIT(t, ft)	do { (void) (ft); } while (0)
# define FT_ROOT_ASSERT_NOT_ROOT(t, slot)	do { } while (0)
#endif

/*
 * THE RECORD-TIME OWNER CHECK: under FINE the SW promise is PER-OP, so a
 * structural edge may park only if the op's held lock-set OWNS that word
 * (doc/design/mw-writer-lock-escalation-model.md §8: a parent->child edge's
 * four fields are owned by the PARENT's lock, a node's state word by its own).
 * ft_txn_content_sw_ok cannot answer that -- it is a property of the TRIE --
 * so the answer travels with the RECORD, as the @owner every SW-capable
 * record helper takes.
 *
 * ★ THE CHECK IS A MACHINE CHECK, NOT A TABLE.  A per-site conversion table
 * goes stale the first time a slot's shape changes; this assert travels with
 * the code and fires on the record that broke the rule
 * ([[feedback_a_site_inventory_cannot_cover_a_dynamic_slot]]).
 *
 * ☠ ARMED ONLY FOR A PER-OP ARM, and that is not a weakening.  A COARSE or
 * exclusive trie arms because a TRIE-WIDE exclusion holds (Phase A), and its
 * ops register no per-node lock for most of what they park -- so asserting
 * held-ness there would report the wide mutex's own soundness as a violation.
 * @dbg_arm_per_op is set only by the Phase B arm (ft_flip_txn_arm_per_op) and
 * by the dry-run claim beside it (ft_flip_txn_claim_per_op), so the assert
 * covers exactly the txns whose promise is per-word.
 *
 * ⇒ IT THEREFORE HAS NO COVERAGE UNTIL THE FIRST SITE ARMS.  What gives it
 * coverage NOW is the counter beside it (FT_TK_OWN_HELD / FT_TK_OWN_MISS,
 * ft-txn-kind-stats.h), which runs the SAME predicate on every SW-capable
 * record whatever the mode: a site whose records are 100% owner-held under
 * FINE is a site the arm can convert, and one below that has named the
 * exclusion gap it must close first
 * ([[feedback_an_assert_config_that_never_fires_is_not_coverage]]).
 *
 * @owner NULL means "no node owns this word".  For a TRIE ROOT that is the
 * true answer and the always-MW ft_flip_txn_record_root is the route (G2); on
 * an SW-capable record it means the site has not named an owner, which is the
 * surface Phase B closes -- so it is a MISS, never a pass.
 */
#if defined(DEBUG_RCU) || defined(CONFIG_RCU_DEBUG)
# define FT_OWNER_ASSERT_TXN_FIELD	bool dbg_arm_per_op;
# define FT_OWNER_ASSERT_INIT(t)					\
	do { (t)->dbg_arm_per_op = false; } while (0)
# define FT_OWNER_ASSERT_SET_PER_OP(t)					\
	do { (t)->dbg_arm_per_op = true; } while (0)
/*
 * ☠ @nr_locks IS PART OF THE PREDICATE, not a shortcut.  ft_flip_txn_arm_per_op
 * REFUSES an empty registry -- a commit holding nothing owns nothing -- so a txn
 * that has registered no lock is one the arm never touches, and asserting on it
 * reports a gap that no conversion could ever close.  For the real arm the term
 * is inert (it arms only when @nr_locks is already non-zero, and the registry
 * never shrinks).  What it buys is the DRY RUN: a claim must be set before the
 * records it wants to check, which is necessarily before the op's acquires have
 * finished, and this is what stops that from reporting every pre-acquire record
 * as a miss.
 */
# define FT_OWNER_ASSERT_OWNED(t, owner)				\
	urcu_assert_debug(!(t)->dbg_arm_per_op || !(t)->nr_locks ||	\
			ft_flip_txn_owns((t), (owner)))
/*
 * THE SAME QUESTION, ASKED OF A RECORD THAT CARRIES ITS OWN WITNESS.
 *
 * A miss splits two ways (see ft_flip_txn_owns): a FINDING where the txn owns
 * the mark's CLEARING, a VISIBILITY gap where a caller's SWEEP does.  Only the
 * second may widen the predicate, and only one record SHAPE qualifies -- the
 * argument lives on ft_owner_retire_witnessed.
 *
 * The witness is a PARAMETER, never txn state: it dies with the call, so no
 * window exists in which another record could be covered by it and there is no
 * `clear` to forget.  A caller that supplies @ctx on a record of any other
 * shape gets the NARROW predicate back -- misuse fails CLOSED.
 */
# define FT_OWNER_ASSERT_OWNED_CTX(t, ctx, owner, slot, new_ptr)	\
	urcu_assert_debug(!(t)->dbg_arm_per_op || !(t)->nr_locks ||	\
			ft_flip_txn_owns((t), (owner)) ||		\
			ft_owner_retire_witnessed((ctx), (owner),	\
					(slot), (new_ptr)))
#else
# define FT_OWNER_ASSERT_TXN_FIELD
# define FT_OWNER_ASSERT_INIT(t)	do { } while (0)
# define FT_OWNER_ASSERT_SET_PER_OP(t)	do { } while (0)
# define FT_OWNER_ASSERT_OWNED(t, owner)				\
	do { (void) (t); (void) (owner); } while (0)
# define FT_OWNER_ASSERT_OWNED_CTX(t, ctx, owner, slot, new_ptr)	\
	do {								\
		(void) (t); (void) (ctx); (void) (owner);		\
		(void) (slot); (void) (new_ptr);			\
	} while (0)
#endif

/*
 * THE TWO WAYS A SITE HAS NO OWNER TO NAME.  Both are NULL -- both therefore
 * count OWN_MISS and both refuse a per-op SW park -- but they are different
 * findings, and spelling them apart makes each one a GREPPABLE INVENTORY
 * instead of a bare NULL that reads as an oversight.
 *
 * FT_OWNER_NONE_EXTERNAL_HEAD: the word belongs to an EXTERNAL HEAD or its
 *   ordered cell -- cell->parent, en->prev, next_node->prev -- and no lock word
 *   owns it, because neither a cell nor an external node carries a state word.
 *
 *   ☠ THE REMAINING SITES ARE THE BACK-EDGE RE-PARENTS, and for them this is a
 *   DESIGN question, not plumbing.  The code keys the edge kind on holding the
 *   CHILD (ft_flip_txn_record_parent_word's @child_held; ft_reparent_record_meta
 *   names owner = meta), so an external child has no owner to name.  ☞ But
 *   mw-writer-lock-escalation-model.md §8.2 "Field-by-field ownership" already
 *   assigns the `parent` pointer to the PARENT (P) -- so the model and the code
 *   DISAGREE, and closing this class means deciding which moves, not inventing
 *   a convention.  The alternative is giving externals a state word (§8.1).
 *
 *   ☞ THE KIND, HOWEVER, IS SETTLED AHEAD OF THAT ANSWER: whoever owns the
 *   word, no op can hold it today, so every DIRECT record of it goes through
 *   the always-MW ft_flip_txn_record_head_back_edge and leaves the MW_STRUCT
 *   conversion surface.  This marker therefore survives only where the word
 *   travels through a RECORDER THAT CANNOT SAY MW -- ft_pub_rec_add, whose
 *   per-edge answers are @root and @owner and neither means "always MW".
 *
 *   ☞ It is NOT the class every ownerless external-head word belongs to.  The
 *   head-promote sites had a holder all along -- ft_promote_head takes it as a
 *   parameter and registers it on the same txn -- and only the ORDER of the
 *   acquire hid it.  Check for a holder in scope before reaching for this.
 *
 * FT_OWNER_UNPLUMBED: the owner EXISTS and is unambiguous -- it is simply not
 *   in scope at the record, because the caller passed a bare slot pointer.
 *   Purely a plumbing job, and until it is done the site cannot arm.
 *
 * ☠ Neither is a licence.  A MISS is a site the Phase B arm must skip; the
 * counter is what says how much traffic each class carries.
 */
#define FT_OWNER_NONE_EXTERNAL_HEAD	NULL
#define FT_OWNER_UNPLUMBED		NULL

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
 * Upper bound of FT_STATE_LOCK locks one commit can hold.
 *
 * The small consumers are unchanged: the chain-compress fused merge fences the
 * collapsed chain (boundary + old parent cn + old child cn = 3), and a
 * LOCK_FINE recompact holds its whole {C, P} lock-set, plus {GP} when P is a
 * compressed node whose SKIP_X dual it re-encodes (§9.3) = 3.
 *
 * The bound is the FAN-OUT, though, not those.  A lock-set is the unit the MCAS
 * install can SORT by slot address, and that order is what makes concurrent
 * acquisition deadlock-free (rcu-txn-mcas.h) -- so a site that wants ordering
 * must present its whole set at once, and the widest such set is a node's
 * children plus the node itself.  At 8 that was impossible: ft_rekey_cow_stop
 * alone reaches 17 distinct anchors on the unit fixture (3 of 18 calls exceed
 * 8), which is why its marks live in fn-scope arrays outside the registry
 * rather than in a set.
 *
 * ☠ COST, paid by every op: this array is embedded in struct ft_flip_txn, which
 * is malloc'd per attempt, and ft_dlm_acquire_set mirrors it TWICE on the stack.
 * At 16 bytes an entry that is 4 KB in each place.  If that shows up in a
 * profile, the shape to move to is a small embedded array with a heap overflow
 * for the rare wide set -- not a lower cap, which merely re-hides the assert.
 */
#define FT_FLIP_TXN_MAX_LOCKS	(FT_ENTRY_PER_NODE + 1)

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
	 *
	 * Each entry carries the CLEAN word the acquire captured, because the
	 * registry is half of the op's HELD SET: a later member that coarsens
	 * onto a registered word deduplicates against it and has no snapshot of
	 * its own, while its plan re-validation and its retire both need the
	 * value the FIRST acquire ratified.
	 */
	struct ft_flip_txn_lock {
		struct cds_ft_metadata *meta;
		uintptr_t snap;
	} locks[FT_FLIP_TXN_MAX_LOCKS];
	unsigned int nr_locks;
	/*
	 * Set when a per-node lock acquire MISSED (see
	 * ft_flip_txn_lock_or_guard_parent).  The op then structurally writes a
	 * slot whose owner it does not hold, so the commit must ABORT rather
	 * than publish: an all-or-none lock-set, with the miss re-descending.
	 */
	bool acquire_miss;
	/*
	 * The miss above was an ALLOCATION failure, not a peer.  The acquire
	 * choke point builds a small txn of its own (ft_dlm_acquire_set_at), so
	 * it can fail -ENOMEM as well as -EAGAIN -- and its caller here returns
	 * void, with @acquire_miss its only channel.  Without this bit the
	 * commit reports an OOM as ABORT and ages the handle for it, so every
	 * downstream errno test reads memory pressure as contention: measured as
	 * the one -EAGAIN at cds_ft_remove_all's tail that no peer produced.
	 */
	bool acquire_enomem;
	/*
	 * THE OP'S PENDING FORWARD PUBLISH, carried so a recompaction of the
	 * publish PARENT can fold it into the copy it makes.
	 *
	 * A same-trie rekey's src branch point IS its dst publish parent, so the
	 * src detach DEL-recompacts the very node the glue captured in
	 * @publish_slot.  The copy is built from that parent's COMMITTED slots
	 * (ft_flip_txn_resolve_prio is deliberately not read-your-own-writes), so
	 * it inherits the PRE-publish child; the forward publish then lands in a
	 * node nothing reads, the merged top is stranded, and the coherent
	 * reader's two-descent address witness never sees the move.
	 *
	 * Applied BY IDENTITY in the copy loop, exactly as @nullify_node_flag_ptr
	 * applies a pending detach there.  The copy must not read pending values
	 * wholesale -- that would drop children a buffered detach has NULLed --
	 * so every pending edit it must honour is named explicitly.  Both edits
	 * then ride ONE flip, which is what keeps the move atomic to readers.
	 */
	struct cds_ft_inode_flag **pending_pub_slot;
	struct cds_ft_inode_flag *pending_pub_val;
	/*
	 * Set by the recompaction that FOLDED the publish above into its copy.
	 * The forward publish is then already live in the surviving node, and
	 * recording it a second time would aim an edge at the superseded copy --
	 * a slot no reader reaches, on a node this same commit retires.
	 */
	bool pending_pub_folded;
	/*
	 * THE OP'S PENDING SLOT DROP, carried so a recompaction of the node the
	 * drop targets folds it into the copy it makes.  The mirror of
	 * @pending_pub_slot above, reachable on the same geometry.
	 *
	 * A same-trie rekey whose src branch point IS the graft child both
	 * ATTACHES to that node and DETACHES from it inside one decide.  The
	 * attach ADD-recompacts it first, so a detach that then edits the node
	 * addresses the copy the attach retired -- and DECIDES ITS SHAPE from
	 * that copy's committed child count, which is short by the attach's
	 * pending child.  A boundary that ends up keeping two children reads as
	 * keeping one, and the chain-compress fuse retires a child the attach
	 * has just re-parented onto its fresh copy.
	 *
	 * Folded, the surviving node is born with the new child present and the
	 * dropped one absent: its child count is never transiently wrong, and
	 * there is no second recompaction left to decide anything from.
	 *
	 * @pending_del_expected is the PLAN's expected-old, checked by identity
	 * in the copy loop for the reason @nullify_expected states -- a peer that
	 * republished the slot between the plan and the copy would otherwise have
	 * its whole subtree dropped by a "copy every child except this slot".  A
	 * peer's parked flip proxy cannot match it either, so it bails the same
	 * way.
	 */
	struct cds_ft_inode_flag **pending_del_slot;
	struct cds_ft_inode_flag *pending_del_expected;
	/*
	 * Set by the recompaction that FOLDED the drop above into its copy.  The
	 * slot is then already absent from the surviving node, and clearing it a
	 * second time would aim an edge at the superseded copy -- a slot no
	 * reader reaches, on a node this same commit retires.
	 */
	bool pending_del_folded;
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
	/*
	 * --enable-rcu-debug only: the NAMED trie's root slot, for the
	 * assertion in ft_flip_txn_record_tag that no generic structural
	 * record ever aims at it (ft_flip_txn_record_root is the only legal
	 * way to write a root).  A DETECTOR, not a dispatcher: the kind is
	 * decided by which helper the site calls, so a release build carries
	 * neither the field nor the compare.
	 *
	 * ☠ BLIND TO A CROSS-TRIE TXN'S SECOND ROOT.  A dual names one trie
	 * and writes two roots, and this word can hold only one of them --
	 * which is precisely why the helper, not this assert, is the
	 * mechanism.  The dual's two roots are marked at the one helper both
	 * whole-trie swaps go through (ft_root_list_swap_publish_dual), so
	 * they are covered by construction rather than by detection.
	 */
	FT_ROOT_ASSERT_TXN_FIELD
	/*
	 * --enable-rcu-debug only: this txn armed @structural_sw from its own
	 * HELD SET (ft_flip_txn_arm_per_op) rather than from a trie-wide
	 * exclusion, so every SW record it plants must name an owner the
	 * registry above holds.  See FT_OWNER_ASSERT_OWNED for why a
	 * trie-wide arm is exempt.
	 */
	FT_OWNER_ASSERT_TXN_FIELD
	/*
	 * -DFT_DEBUG_TXN_KIND only (ft-txn-kind-stats.h): where this txn was
	 * created, and whether the record in flight is the DLM lock TAKE.  The
	 * take reaches the same dispatch as every other structural edge, so
	 * without the flag it would be counted as a conservative-MW edge --
	 * i.e. as convertible, which it is precisely not.
	 */
	FT_TK_TXN_FIELDS
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

/*
 * Close one attempt of a retry loop bracketed by the handle above, on a path
 * that is NOT re-attempting: success, or a terminal error.
 *
 * @open is the loop's own record of whether it opened the bracket, and it is a
 * VARIABLE rather than a re-test of the opening condition on purpose.  The
 * bulk-op loops bracket CONDITIONALLY -- only a body that provably takes no
 * grace period may hold a read section or an escalation turn -- and a condition
 * re-evaluated at the exit is a second chance to disagree with the entry.  One
 * flag set beside begin() cannot.
 */
static inline
void ft_txn_attempt_end(struct urcu_txn *op, bool open)
{
	if (open)
		urcu_txn_end(op);
}

/*
 * Close one attempt that BAILED BEFORE ITS OWN COMMIT and will re-attempt.
 *
 * Two things, in this order.  urcu_txn_conflict() AGES the handle, which is the
 * entire point of giving the loop one: without it every attempt restarts at
 * retry 0, the domain never escalates the writer into its per-trie FIFO
 * fair-mutex lane, and the loop has no termination argument at all.  Then
 * end(), which FORFEITS the turn -- a pre-commit bail re-descends and asks a
 * peer for the very thing it just lost, so keeping the lane across that ask
 * queues the holder behind the waiter (the insert livelock).
 *
 * Use ft_txn_attempt_end() instead where the ABORT came from a commit made
 * THROUGH this handle: that commit already aged it and legitimately keeps the
 * turn.  These loops commit through a separate per-attempt ft_flip_txn, so
 * every one of their retry edges is a bail by this definition.
 */
/*
 * Lock refusals this attempt has taken and not yet aged for.
 *
 * ft_dlm_acquire_set records them and ages nothing: an acquire knows a WORD was
 * contended, but only the op's retry loop knows an ATTEMPT ended, and aging is
 * a statement about attempts.  Splitting it that way keeps ONE ager -- see the
 * bail below -- so every op ages by the same rule no matter which level noticed
 * the conflict.
 */
static __thread unsigned int ft_acq_contended;

static inline
void ft_txn_attempt_bail(struct urcu_txn *op, bool open)
{
	if (open) {
		urcu_txn_conflict(op);
		/*
		 * Then once per refused lock-set.  An attempt that lost three
		 * acquires waited on three peers, and folding them into the one
		 * conflict above under-ages exactly the op the FIFO lane exists
		 * to rescue.
		 */
		while (ft_acq_contended) {
			urcu_txn_conflict(op);
			ft_acq_contended--;
		}
		urcu_txn_end(op);
	}
	/*
	 * Refusals from an attempt that went on to SUCCEED belong to no retry.
	 * Drop them, or the next op on this thread is aged for them.
	 */
	ft_acq_contended = 0;
}

/* Defined below; the constructors arm through it. */
static inline
void ft_flip_txn_set_structural_sw(struct ft_flip_txn *t, bool v);

/*
 * MAY a CONTENT txn on @ft park its structural edges SW instead of recording
 * them MW?  This is the conversion's single policy switch, and it is asked of
 * the TRIE rather than of the call site because the answer is a property of the
 * trie's writer mode: an SW park cannot fail, so it is legal exactly where the
 * op excludes every peer writer of the slots it rewrites.
 *
 *   COARSE non-exclusive   the FT-wide writer mutex is that exclusion
 *   exclusive              single writer by contract
 *   FINE non-exclusive     only the per-node DLM locks, so the promise holds
 *                          for an op whose lock set provably covers every slot
 *                          it rewrites -- a per-op property, not a trie-wide one
 *
 * ARMED FOR COARSE NON-EXCLUSIVE.  ft_writer_lock_scope_enter takes the FT-wide
 * fair mutex at the OUTERMOST writer scope of every mutation on such a trie and
 * holds it for the whole op body -- the per-domain DROP is FINE-only, and the
 * exclusive early-out is the other mode.  So a content txn here has no peer
 * writer at all: the word its park lands on is one no other mutator can be
 * inside.  That mutex is a stronger exclusion than the per-node lock-set FINE
 * will have to argue slot by slot.
 *
 * ARMED FOR EXCLUSIVE, at either granularity, and on a DIFFERENT argument --
 * do not read the two as one rule.  An exclusive trie takes NO mutex at all
 * (ft_writer_lock_scope_enter returns early on it, deliberately, so a
 * cross-trie op can hold one live side's lock while an exclusive consumed
 * source rides through the fused body).  The exclusion is the caller's
 * single-writer contract, which cds_ft_make_exclusive also drains readers
 * behind (ft_writer_lock_gp_wait before the flag is set).
 *
 * ☠ WHICH MAKES THE CROSS-TRIE DIRECTION THE THING TO CHECK, because a txn
 * arms off the trie it NAMES while it may record a second trie's slots, and
 * "exclusive" is a claim about one trie only.  The dst-named direction was
 * already argued (an exclusive consumed src, guaranteed by the BUSY gate,
 * writer-excludes the src slots).  The src-named direction is the new one, and
 * it holds because those txns record NOTHING BUT their own trie's slots:
 * ft_glue_apply_deferred's src pass stores rather than records for them (their
 * edges are @live false), their root edges carry @root and are forced MW, and
 * their ordered-cell edges carry URCU_TXN_TAG and are likewise forced MW.  The
 * exclusivity itself spans the whole op -- the BUSY gates refuse a live
 * swap_ft / src_ft up front, and the one place that MUTATES the flag
 * (graft_swap handing swap_ft dst's discipline) runs after every commit.
 *
 * Answering false leaves every content txn all-MW: stricter than necessary and
 * always sound.  The two callers that arm explicitly (the rekey writer, the root
 * COW) still do so on their own reasoning; this switch is what will retire that
 * hand-arming.
 *
 * @ft NULL is the acquire lane, which never arms.
 *
 * ☠ THE PARK IS UNFAILABLE, so a WRONGLY armed txn does not abort -- it silently
 * erases whatever a peer left in the slot.  Every widening of this predicate
 * therefore owes a positive control that the mode it opened actually RAN under
 * the -DFT_DEBUG_TXN_KIND counters (armSW > 0 with MW_STRUCT moving to SW at the
 * sites that mode drives), because a green suite proves nothing about a mode the
 * suite never entered.  The default strategy is FINE.
 *
 * The answer is SHAPE-INDEPENDENT: a CROSS-TRIE txn names one trie and may
 * write two roots (the graft / graft_swap duals flip &dst_ft->root together
 * with &src_ft->root, resp. &swap_ft->root), and arming off the named trie is
 * sound anyway because roots record MW by construction -- every root edge goes
 * through ft_flip_txn_record_root, whichever trie it belongs to.  What remains
 * to argue per arm is only the ordinary one: that the op excludes every peer
 * writer of the NON-root slots it rewrites.
 */
static inline
bool ft_txn_content_sw_ok(const struct cds_ft *ft)
{
	if (!ft)
		return false;		/* the acquire lane names no trie */
	return !ft->lock_fine || ft->exclusive;
}

/*
 * A CONTENT flip-txn: the lane that rewrites the STRUCTURE -- forward publishes,
 * re-parents, retires, lock releases.  It takes @ft because arming is decided
 * from the trie (ft_txn_content_sw_ok); that argument is also what makes the
 * CONTENT / ACQUIRE split compiler-enforced rather than a naming convention.
 *
 * ☠ NOT for a DLM lock-set acquire.  Use ft_flip_txn_acquire_bounded(): the
 * lock take {clean -> LOCK|s} is the arbitration point and must record MW even
 * on a trie whose content may park SW.
 */
static inline
struct ft_flip_txn *ft_flip_txn_create_at(FT_TK_SITE_PARAM struct cds_ft *ft)
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
	t->acquire_enomem = false;
	t->pending_pub_slot = NULL;
	t->pending_pub_val = NULL;
	t->pending_pub_folded = false;
	t->pending_del_slot = NULL;
	t->pending_del_expected = NULL;
	t->pending_del_folded = false;
	t->structural_sw = false;	/* all-MW until a caller opts in under lock_fine */
	FT_ROOT_ASSERT_INIT(t, ft);
	FT_OWNER_ASSERT_INIT(t);
	FT_TK_TXN_INIT(t, dbg_site);	/* names @t before anything counts against it */
	if (ft_txn_content_sw_ok(ft))
		ft_flip_txn_set_structural_sw(t, true);
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
extern long cds_ft_fault_compact_countdown;
extern long cds_ft_fault_removeall_countdown;
#endif

/* Defined below; the bounded constructor is a composition over it. */
static inline
bool ft_flip_txn_reserve(struct ft_flip_txn *t, unsigned int cap);

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

/*
 * Refuse the next RELOCATE lock-set acquire, for cds_ft_compact_step's
 * otherwise unexecutable contention bail (cds_ft_fault_compact_countdown).
 * Scoped by the mode because FT_RECOMPACT_RELOCATE has exactly one caller --
 * ft_compact_relocate_at -- so no other recompact user is perturbed.
 *
 * Must be consulted BEFORE the acquire: forcing the code after a successful
 * one would return -EAGAIN holding the set it just took.
 */
/*
 * cds_ft_remove_all's refused-acquire fault.  Armed for the dynamic extent of
 * its detach only -- the acquire choke point below is shared by every op, so
 * the scope flag is what keeps the fault from perturbing them.
 */
#ifdef FEATURE_FT_FAULT_INJECT
static __thread bool ft_removeall_fault_scope;
#endif

static inline
void ft_removeall_fault_scope_enter(void)
{
#ifdef FEATURE_FT_FAULT_INJECT
	ft_removeall_fault_scope = true;
#endif
}

static inline
void ft_removeall_fault_scope_exit(void)
{
#ifdef FEATURE_FT_FAULT_INJECT
	ft_removeall_fault_scope = false;
#endif
}

static inline
bool ft_removeall_fault_refuse_acquire(void)
{
#ifdef FEATURE_FT_FAULT_INJECT
	if (!ft_removeall_fault_scope)
		return false;
	if (cds_ft_fault_removeall_countdown < 0)
		return false;
	if (cds_ft_fault_removeall_countdown == 0) {
		cds_ft_fault_removeall_countdown = -1;
		return true;
	}
	cds_ft_fault_removeall_countdown--;
	return false;
#else
	return false;
#endif
}

static inline
bool ft_recompact_fault_refuse_acquire(enum ft_recompact mode)
{
#ifdef FEATURE_FT_FAULT_INJECT
	if (mode != FT_RECOMPACT_RELOCATE)
		return false;
	if (cds_ft_fault_compact_countdown < 0)
		return false;
	if (cds_ft_fault_compact_countdown == 0) {
		cds_ft_fault_compact_countdown = -1;
		return true;
	}
	cds_ft_fault_compact_countdown--;
	return false;
#else
	(void) mode;
	return false;
#endif
}

static inline
struct ft_flip_txn *ft_flip_txn_create_bounded_at(FT_TK_SITE_PARAM
		struct cds_ft *ft, unsigned int cap)
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
	/*
	 * COMPOSITION, not a second implementation.  Reservation is an ENGINE
	 * capability (urcu_txn_reserve: "optional; call after begin, before the
	 * first store") and the FT already exposes it as a METHOD --
	 * ft_flip_txn_reserve, used by the glue folds at five sites, which sets
	 * the very same @reserved flag.  Spelling it a second time as a
	 * CONSTRUCTOR duplicated the logic and, worse, made boundedness a birth
	 * attribute that then multiplied against the _on axis into four
	 * constructors for one constructor plus one method.
	 *
	 * Behaviour-identical to the open-coded form it replaces: create() sets
	 * @reserved false and performs the same urcu_txn_init +
	 * expect_conflict (whose reasoning -- a dense pre-reserved write set
	 * saturates the age-0 RYW Bloom and false-positives into a spurious
	 * abort -- applies to exactly these commits), and reserve() then makes
	 * the same urcu_txn_reserve call and sets @reserved true.
	 *
	 * ☠ ft_flip_txn_create_bounded_on is deliberately NOT collapsed the same
	 * way: it omits expect_conflict where create_on performs it, so routing
	 * it through create_on would silently change its install lane.  That
	 * asymmetry is defensible (its op handle carries a retry loop that
	 * absorbs an age-0 false positive, which the standalone commits here --
	 * remove_all et al. -- do not have) but it is a behaviour difference,
	 * not a spelling one.
	 */
	t = ft_flip_txn_create_at(FT_TK_SITE_FWD ft);
	if (!t)
		return NULL;
	if (!ft_flip_txn_reserve(t, cap)) {
		if (t->mtxn->desc && t->mtxn->desc != URCU_TXN_ENOMEM)
			urcu_txn_destroy(t->mtxn->desc);
		free(t);
		return NULL;
	}
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
struct ft_flip_txn *ft_flip_txn_create_on_at(FT_TK_SITE_PARAM
		struct cds_ft *ft, struct urcu_txn *op)
{
	struct ft_flip_txn *t = (struct ft_flip_txn *) malloc(sizeof(*t));

	if (!t)
		return NULL;
	t->mtxn = op;
	urcu_txn_expect_conflict(t->mtxn);
	t->reserved = false;		/* unbounded: @mtxn grows as edges record */
	t->nr_locks = 0;
	t->acquire_miss = false;
	t->acquire_enomem = false;
	t->pending_pub_slot = NULL;
	t->pending_pub_val = NULL;
	t->pending_pub_folded = false;
	t->pending_del_slot = NULL;
	t->pending_del_expected = NULL;
	t->pending_del_folded = false;
	t->structural_sw = false;
	FT_ROOT_ASSERT_INIT(t, ft);
	FT_OWNER_ASSERT_INIT(t);
	FT_TK_TXN_INIT(t, dbg_site);	/* names @t before anything counts against it */
	if (ft_txn_content_sw_ok(ft))
		ft_flip_txn_set_structural_sw(t, true);
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
struct ft_flip_txn *ft_flip_txn_create_bounded_on_at(FT_TK_SITE_PARAM
		struct cds_ft *ft, struct urcu_txn *op, unsigned int cap)
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
	t->acquire_enomem = false;
	t->pending_pub_slot = NULL;
	t->pending_pub_val = NULL;
	t->pending_pub_folded = false;
	t->pending_del_slot = NULL;
	t->pending_del_expected = NULL;
	t->pending_del_folded = false;
	t->structural_sw = false;	/* all-MW until a caller opts in under lock_fine */
	FT_ROOT_ASSERT_INIT(t, ft);
	FT_OWNER_ASSERT_INIT(t);
	FT_TK_TXN_INIT(t, dbg_site);	/* names @t before anything counts against it */
	if (ft_txn_content_sw_ok(ft))
		ft_flip_txn_set_structural_sw(t, true);
	return t;
}

/*
 * THE ACQUIRE LANE, and why it is a SEPARATE CONSTRUCTOR rather than a content
 * txn used differently: the lock take {clean -> LOCK|s} IS the point at which
 * two ops racing for a node are decided.  An MW record installs with a CAS-old,
 * so the loser's commit aborts; an SW park cannot fail, so an SW take would
 * hand BOTH ops the node.  This txn therefore names no trie and can never be
 * armed -- and ft_dlm_lock's assert(!t->structural_sw) then catches a
 * miscategorised site loudly instead of silently producing two lock owners.
 *
 * Always bounded: an acquire's edge count is its lock-set size plus its guards,
 * both known before the first record.
 */
static inline
struct ft_flip_txn *ft_flip_txn_acquire_bounded_at(FT_TK_SITE_PARAM
		unsigned int cap)
{
	return ft_flip_txn_create_bounded_at(FT_TK_SITE_FWD NULL, cap);
}

/*
 * THE CREATION SITE IS THE INSTRUMENT'S IDENTITY (ft-txn-kind-stats.h).  Each
 * expansion below plants a function-static naming its own __FILE__:__LINE__ and
 * hands it to the constructor, so the per-site record-kind / commit-outcome
 * table needs nothing from the 70 call sites themselves.  Without the knob the
 * site argument is absent from the SIGNATURE too (FT_TK_SITE_PARAM), so the
 * uninstrumented build is not merely cheaper but identical -- an always-NULL
 * argument still moves the compiler's inlining decisions, and an instrument
 * used to compare latencies must not perturb the paths it compares.
 */
#ifdef FT_DEBUG_TXN_KIND
# define ft_flip_txn_create(ft)						\
	ft_flip_txn_create_at(FT_TK_SITE_HERE("create"), (ft))
# define ft_flip_txn_create_bounded(ft, cap)				\
	ft_flip_txn_create_bounded_at(FT_TK_SITE_HERE("bounded"), (ft), (cap))
# define ft_flip_txn_create_on(ft, op)					\
	ft_flip_txn_create_on_at(FT_TK_SITE_HERE("on"), (ft), (op))
# define ft_flip_txn_create_bounded_on(ft, op, cap)			\
	ft_flip_txn_create_bounded_on_at(FT_TK_SITE_HERE("bounded_on"),	\
			(ft), (op), (cap))
# define ft_flip_txn_acquire_bounded(cap)				\
	ft_flip_txn_acquire_bounded_at(FT_TK_SITE_HERE("acquire"), (cap))
#else
# define ft_flip_txn_create(ft)		ft_flip_txn_create_at(ft)
# define ft_flip_txn_create_bounded(ft, cap)				\
	ft_flip_txn_create_bounded_at((ft), (cap))
# define ft_flip_txn_create_on(ft, op)					\
	ft_flip_txn_create_on_at((ft), (op))
# define ft_flip_txn_create_bounded_on(ft, op, cap)			\
	ft_flip_txn_create_bounded_on_at((ft), (op), (cap))
# define ft_flip_txn_acquire_bounded(cap)				\
	ft_flip_txn_acquire_bounded_at(cap)
#endif

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

#ifdef FEATURE_FT_HOLD_TRACE
/*
 * TEST-ONLY LEDGER of the words this thread currently holds, with the SITE that
 * took each one.  It answers the one question the -EAGAIN cannot: a refused
 * acquire says the word is LOCKed, never by WHOM, and under a coarse spacing the
 * whom is usually the refusing op itself.
 *
 * Maintained at the primitives rather than at the choke point so it sees every
 * hold regardless of which registry (if any) the op filed it in -- that
 * mismatch is the defect being measured.
 */
#define FT_HOLD_TRACE_MAX	1024
struct ft_hold_trace_ent {
	const struct cds_ft_metadata *lock;
	const char *fn;
	int line;
};
static __thread struct ft_hold_trace_ent ft_hold_trace[FT_HOLD_TRACE_MAX];
static __thread unsigned int ft_hold_trace_n;

/*
 * A refusal a RETRY LOOP re-derives fires once per attempt, so an unbounded
 * report is not a report -- it is a disk filling at the speed of the loop
 * (measured: 25 GB in one suite).  Cap the whole feature's output; the first
 * few lines are the diagnosis and the rest is the same line again.
 */
#define FT_HOLD_TRACE_REPORT_MAX	200
static __thread unsigned int ft_hold_trace_reports;

static inline
bool ft_hold_trace_report_ok(void)
{
	if (ft_hold_trace_reports >= FT_HOLD_TRACE_REPORT_MAX)
		return false;
	if (++ft_hold_trace_reports == FT_HOLD_TRACE_REPORT_MAX)
		fprintf(stderr, "FT HOLD TRACE: report cap reached, silencing\n");
	return true;
}

static inline
void ft_hold_trace_note(const struct cds_ft_metadata *lock, const char *fn,
		int line)
{
	if (ft_hold_trace_n >= FT_HOLD_TRACE_MAX)
		return;
	ft_hold_trace[ft_hold_trace_n].lock = lock;
	ft_hold_trace[ft_hold_trace_n].fn = fn;
	ft_hold_trace[ft_hold_trace_n].line = line;
	ft_hold_trace_n++;
}

/*
 * Does THIS THREAD hold @lock, by the ledger rather than by any registry?
 *
 * The ledger's whole point is that it is maintained at the lock PRIMITIVES, so
 * it sees a hold whichever ft_lock_ctx frame filed it -- extras, glue, an outer
 * frame -- or none at all.  That is exactly the reach ft_flip_txn_owns lacks,
 * and pairing the two is what separates a registry gap from an exclusion gap.
 *
 * ☠ NEITHER SIDE SUBSUMES THE OTHER, so only the UNION is the held set.  The
 * ledger drops its entry the moment a release is RECORDED, while the word keeps
 * LOCK until that commit lands -- ft_hold_trace_refused spells this out, and in
 * that window the registry is the only witness.  The ledger in turn can carry a
 * LEAKED entry (taken, dropped by a terminal whose commit never applied), so it
 * can over-report.  Test-only either way.
 */
static inline
bool ft_hold_trace_holds(const struct cds_ft_metadata *lock)
{
	unsigned int i = ft_hold_trace_n;

	while (i--)
		if (ft_hold_trace[i].lock == lock)
			return true;
	return false;
}

static inline
void ft_hold_trace_drop(const struct cds_ft_metadata *lock)
{
	unsigned int i = ft_hold_trace_n;

	while (i--) {
		if (ft_hold_trace[i].lock == lock) {
			ft_hold_trace[i] = ft_hold_trace[--ft_hold_trace_n];
			return;
		}
	}
}

/*
 * An acquire was refused.  Report it as a SELF-collision only when the word
 * really carries LOCK and this thread's ledger names it: any other dirty bit is
 * an ordinary peer refusal.  The tail of the ledger is printed too, because a
 * commit that CONSUMED a fence (a fenced tombstone leaves TOMBSTONE, no LOCK)
 * never calls a release and so leaves its entry behind.
 */
static inline
void ft_hold_trace_refused(const struct cds_ft_metadata *lock, const char *fn,
		int line)
{
	unsigned int i = ft_hold_trace_n;

	if (!(CMM_LOAD_SHARED(lock->state) & FT_STATE_LOCK)) {
		if (ft_hold_trace_report_ok())
			fprintf(stderr,
				"FT REFUSED (not ours): %s:%d word %p state=%lx\n",
				fn, line, (const void *) lock,
				(unsigned long) CMM_LOAD_SHARED(lock->state));
		return;
	}
	while (i--) {
		if (ft_hold_trace[i].lock != lock)
			continue;
		fprintf(stderr,
			"FT SELF-COLLISION: %s:%d refused word %p, taken at %s:%d (ledger %u deep)\n",
			fn, line, (const void *) lock, ft_hold_trace[i].fn,
			ft_hold_trace[i].line, ft_hold_trace_n);
		for (i = ft_hold_trace_n; i-- > 0 && i + 8 >= ft_hold_trace_n;)
			fprintf(stderr, "  held[%u] %p %s:%d\n", i,
				(const void *) ft_hold_trace[i].lock,
				ft_hold_trace[i].fn, ft_hold_trace[i].line);
		abort();
	}
	/*
	 * LOCK is set and the ledger does not name the word.  Under a single
	 * writer that is not contention -- there is no peer -- so it is this
	 * thread's own mark, and there are TWO ways to get here.  Print the
	 * ledger, because which one it is shows in whether the op still holds
	 * anything at all.
	 *
	 *  - LEAKED: taken, dropped from the ledger by a terminal its commit did
	 *    not apply, and never released.
	 *  - STILL HELD, RELEASE ONLY RECORDED: ft_flip_txn_record_release_lock
	 *    and friends drop the ledger entry the moment they record the
	 *    {LOCK|s -> s} edge ("the commit owns this release now"), while the
	 *    word keeps its LOCK bit until that commit lands.  Between those two
	 *    points the txn's locks[] registry is the ONLY witness of the hold --
	 *    so a site that re-acquires the word here is really missing the
	 *    registry from its ft_lock_ctx, not meeting a leak.  A fold makes that
	 *    window the whole op: nothing commits until the single decide.
	 *
	 * Either way the next attempt refuses against it forever, which is a
	 * retry storm rather than a failure.
	 */
	if (ft_hold_trace_report_ok()) {
		fprintf(stderr,
			"FT REFUSED (LOCK, unknown holder): %s:%d word %p state=%lx "
			"(ledger %u deep)\n",
			fn, line, (const void *) lock,
			(unsigned long) CMM_LOAD_SHARED(lock->state),
			ft_hold_trace_n);
		for (i = ft_hold_trace_n; i-- > 0 && i + 8 >= ft_hold_trace_n;)
			fprintf(stderr, "  held[%u] %p %s:%d\n", i,
				(const void *) ft_hold_trace[i].lock,
				ft_hold_trace[i].fn, ft_hold_trace[i].line);
	}
}

/*
 * A RELEASE naming a word this thread never took -- the MIRROR of a
 * self-collision, and the shape coarsening produces when a site re-derives the
 * release target from the NODE instead of naming the word the acquire actually
 * CAS'd (its ANCHOR).  The bare assert below says only that some word was not
 * locked; this says WHICH word and what this thread does hold instead, which is
 * the whole diagnosis.
 */
static inline
void ft_hold_trace_bad_release(const struct cds_ft_metadata *lock,
		uintptr_t s)
{
	unsigned int i = ft_hold_trace_n;

	/*
	 * Capped like every other report in this feature: the assert below is
	 * compiled out under NDEBUG, and the release then RETRIES -- an
	 * uncapped line here is a disk filling at the speed of that loop.
	 */
	if (!ft_hold_trace_report_ok())
		return;
	fprintf(stderr,
		"FT BAD RELEASE: word %p state=%lx not held (ledger %u deep)\n",
		(const void *) lock, (unsigned long) s, ft_hold_trace_n);
	while (i-- > 0 && i + 12 >= ft_hold_trace_n)
		fprintf(stderr, "  held[%u] %p %s:%d\n", i,
			(const void *) ft_hold_trace[i].lock,
			ft_hold_trace[i].fn, ft_hold_trace[i].line);
}
#else
static inline
void ft_hold_trace_note(const struct cds_ft_metadata *lock, const char *fn,
		int line)
{
	(void) lock; (void) fn; (void) line;
}

static inline
bool ft_hold_trace_holds(const struct cds_ft_metadata *lock)
{
	(void) lock;
	return false;
}

static inline
void ft_hold_trace_drop(const struct cds_ft_metadata *lock)
{
	(void) lock;
}


static inline
void ft_hold_trace_refused(const struct cds_ft_metadata *lock, const char *fn,
		int line)
{
	(void) lock; (void) fn; (void) line;
}

static inline
void ft_hold_trace_bad_release(const struct cds_ft_metadata *lock, uintptr_t s)
{
	(void) lock; (void) s;
}
#endif	/* FEATURE_FT_HOLD_TRACE */

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

#ifdef FEATURE_FT_AGREEMENT_RED
	/*
	 * RED CONTROL for the agreement oracle (NOT a shipping configuration).
	 * Report the lock as taken WITHOUT excluding anyone: peers see the word
	 * clean and acquire it too.  This is exactly what a lock-set that does
	 * not agree on its mapping produces -- two writers mutating one node,
	 * each believing it holds it.  An oracle that stays GREEN under this
	 * cannot certify the anchored conversion.
	 */
	if (!(s & (FT_STATE_PROXY | FT_STATE_TOMBSTONE))) {
		*state_snapshot = s;
		return 0;
	}
#endif
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
	ft_hold_trace_drop(meta);
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
#ifdef FEATURE_FT_AGREEMENT_RED
		/*
		 * The RED control never SET the bit, so its release must not
		 * demand one: the point is to remove EXCLUSION while leaving the
		 * protocol's bookkeeping self-consistent, so any corruption the
		 * oracle reports comes from two writers sharing a node -- not
		 * from a half-broken lock discipline.
		 */
		if (!(s & FT_STATE_LOCK))
			return;
#endif
		if (caa_unlikely(!(s & FT_STATE_LOCK)))
			ft_hold_trace_bad_release(meta, s);
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
	ft_hold_trace_drop(meta);
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
 * THE CLAIM WITHOUT THE ARM: assert this commit's records against its held set
 * while leaving every one of them MW.
 *
 * ★ THIS IS THE DRY RUN FOR B1-B5, and the reason the record-time check is not
 * nested under @structural_sw.  Converting a site is two questions -- "does the
 * op own what it writes?" and "does parking it pay?" -- and only the first can
 * make the structure wrong.  Claiming answers it with an ABORT AT THE
 * OFFENDING RECORD, naming the slot, on a build whose behaviour is otherwise
 * byte-identical to the unconverted one.  Arming first and debugging the
 * fallout answers the same question far more expensively.
 *
 * ☠ AND IT IS WHY THE RED CONTROL IS SOUND.  A control that armed would plant
 * SW parks a mid-txn arm never reserved for, and the engine's own kind /
 * duplicate-slot self-checks fire on those FIRST -- so the abort would prove
 * the ENGINE detects a malformed descriptor, not that this check detects an
 * unowned park ([[feedback_a_fix_you_cannot_revert_into_red]] wants the
 * detector under test to be the one that speaks).  Claiming changes no record
 * kind, so nothing else can answer first.
 */
static inline
void ft_flip_txn_claim_per_op(struct ft_flip_txn *t)
{
	FT_OWNER_ASSERT_SET_PER_OP(t);
}

/*
 * Register a marked fence with the commit wrapper that owns its outcome: the
 * two terminal paths (ft_flip_txn_commit on ABORT / MEMORY_ERROR,
 * ft_flip_txn_destroy on a pre-commit bail) clear every registered fence, and
 * a commit OK consumes it through the recorded {LOCK|s -> TOMBSTONE|s}
 * transition instead.  Register only once the mark's holder can no longer
 * clear it itself (i.e. when the op hands the outcome to the txn).
 *
 * @snap is the CLEAN word the acquire captured: the registry doubles as half
 * the op's held set, and a later member that coarsens onto this word carries no
 * snapshot of its own.
 */
static inline
void ft_flip_txn_lock_register(struct ft_flip_txn *t,
		struct cds_ft_metadata *meta, uintptr_t snap)
{
	assert(t->nr_locks < FT_FLIP_TXN_MAX_LOCKS);
#ifdef FT_LOCKS_HIGHWATER
	{
		static unsigned int hw;

		if (t->nr_locks + 1 > hw) {
			hw = t->nr_locks + 1;
			fprintf(stderr, "FT_LOCKS_HIGHWATER %u / %u\n", hw,
				(unsigned int) FT_FLIP_TXN_MAX_LOCKS);
		}
	}
#endif
	t->locks[t->nr_locks].meta = meta;
	t->locks[t->nr_locks].snap = snap;
	t->nr_locks++;
#ifdef FT_RED_OWNER_CLAIM_ON_LOCK
	/*
	 * RED CONTROL for the record-time owner check, never a shipped
	 * configuration: the moment a commit holds ONE word, make it claim it
	 * owns EVERY word it writes.  That is the canonical form of the defect
	 * the check exists for -- an op parking on the strength of a lock that
	 * covers something else -- rather than a synthetic wrong owner, and
	 * ft_flip_txn_owns is the only thing that can answer it.
	 *
	 * Behaviour-neutral BY CONSTRUCTION: claiming sets no record kind (see
	 * ft_flip_txn_claim_per_op), so an --enable-rcu-debug run under this
	 * knob differs from one without it in exactly one way -- whether the
	 * owner assert fires.  A build WITHOUT --enable-rcu-debug and with
	 * this knob is a no-op, which is the control for the control.
	 */
	ft_flip_txn_claim_per_op(t);
#endif
}

/* Is @m in this op's held set (the DLM lock registry)? */
static inline
bool ft_flip_txn_holds(const struct ft_flip_txn *t,
		const struct cds_ft_metadata *m)
{
	unsigned int i;

	for (i = 0; i < t->nr_locks; i++)
		if (t->locks[i].meta == m)
			return true;
	return false;
}

/*
 * Does this commit OWN the word a record is about to write -- i.e. is @owner,
 * the node whose lock excludes every other writer of that word (§8), one this
 * txn holds?
 *
 * The registry is the honest question to ask HERE, for a mechanical reason: a
 * record helper has only the txn.  ft_held_set_snap reaches wider (extras,
 * glue, outer frames), but the op's ft_held_set lives on a stack frame the
 * record cannot see.
 *
 * ☠ THE STRONGER JUSTIFICATION THAT USED TO STAND HERE IS FALSE, and it was
 * load-bearing for reading every miss as a finding: "a lock whose TERMINAL this
 * commit records must be registered on this commit anyway, or the two terminal
 * paths cannot clear it".  The orphan freeze is a deliberate counterexample --
 * ft_detach_freeze_one records the fused {LOCK|s -> TOMBSTONE|s} terminal on
 * this commit and registers NOTHING when the anchor IS the retired node,
 * because that terminal leaves the word TOMBSTONE, ft_meta_lock_acquire refuses
 * a tombstone forever, and the caller's unconditional release_if_held sweep is
 * therefore correct on BOTH outcomes with no per-commit bookkeeping.  It also
 * cannot register: an orphan chain is FT_MAX_DEPTH long and the registry is
 * FT_ENTRY_PER_NODE + 1, the same number.
 *
 * So a miss here is a finding at the sites whose clearing the txn owns, and a
 * VISIBILITY gap at the sites whose clearing stays with a sweep.  Read it as
 * "the registry cannot see this hold", never as "the op does not hold it", and
 * check which of the two before acting.
 *
 * NULL @owner is a MISS: see FT_OWNER_ASSERT_OWNED.
 *
 * ☠ EXACT AT PER-NODE SPACING, CONSERVATIVE ABOVE IT.  Coarser lock spacing
 * (CDS_FT_LOCK_SPACING=exponential / root-only) puts the word's lock on an
 * ANCHOR ANCESTOR, which the registry holds while @owner itself is absent --
 * so this reports a MISS for a word that IS excluded.  Per-node is the
 * default, which is the mode the readiness numbers are taken in; at any other
 * spacing read the counter as a LOWER BOUND on ownership, and note that the
 * assert stays SAFE either way (it only ever refuses a park, never permits
 * one).  Resolving the anchor here would need the op's descent, which a
 * record helper does not have.
 */
static inline
bool ft_flip_txn_owns(const struct ft_flip_txn *t,
		const struct cds_ft_metadata *owner)
{
	return owner && ft_flip_txn_holds(t, owner);
}

#ifdef FEATURE_FT_FAULT_INJECT
extern long cds_ft_fault_lock_countdown;
#endif

/*
 * The anchors an OP currently holds -- the set every acquire must consult
 * before taking another one.
 *
 * Coarsening maps several lock-set members onto ONE word, so an op reaches its
 * second, third and fourth acquire already holding the word they resolve to.
 * A second acquire on a held word returns -EAGAIN (ft_meta_lock_acquire refuses
 * FT_STATE_LOCK, and it cannot tell the op's own mark from a peer's), the site
 * reads that as contention and re-plans, and the retry rebuilds the identical
 * shape: the op waits on ITSELF, forever.  That failure is a LIVELOCK, not an
 * assertion -- no test reports it, the suite simply stops advancing.
 *
 * @txn is the ordinary registry: every word handed to a commit's outcome is
 * recorded there with its terminal.  @extra covers the marks an op deliberately
 * keeps OUTSIDE it -- ft_detach_node's orphan plan-lock chain reaches
 * FT_MAX_DEPTH, past FT_FLIP_TXN_MAX_LOCKS, and owns its own release sweep --
 * because a held word is a held word wherever the op chose to remember it.
 */
struct ft_glue;
/*
 * A glue keeps its marks in NAMED FIELDS rather than an array -- its publish
 * parent, the compressed node its build splits, the overlap fences, the
 * dup-chain splice holders -- so it is a third source the held set must consult.
 * Defined with the glue itself; declared here because the choke point is above
 * it.  @ratified says whether THIS op has the word's clean value: a word the
 * CALLER acquired is held (dedupe still mandatory) but was never sampled here.
 */
static bool ft_glue_held_snap(const struct ft_glue *g,
		const struct cds_ft_metadata *meta, uintptr_t *snap,
		bool *ratified);

struct ft_held_set {
	struct ft_flip_txn *txn;		/* the commit's lock registry */
	const struct ft_held_anchor *extra;	/* marks held outside it */
	unsigned int nr_extra;
	const struct ft_glue *glue;		/* marks the glue names by field */
	/*
	 * The CALLER's held set, when this one belongs to a nested step of the
	 * same op.  One op has ONE held set, and it is a CHAIN of frames because
	 * a frame carries only ONE out-of-registry array: a step that keeps marks
	 * of its own (ft_detach_node's orphan set) would otherwise DROP its
	 * caller's (the rekey fold's ft_rekey_cow_stop marks), and a dropped
	 * frame is not a missed optimisation -- it is the op refusing its own
	 * fence, deterministically, on every retry.
	 *
	 * Points at a frame that OUTLIVES this one (a caller's, further down the
	 * stack), so it never dangles.  Followed to the end; the chain is
	 * bounded by the call depth.
	 */
	const struct ft_held_set *outer;
};

/*
 * An op's LOCK CONTEXT: the two things every acquire needs that belong to the
 * OP rather than to the member being acquired -- the descent that supplies
 * anchors, and the words the op already holds.  Threaded as one pointer because
 * the acquires sit deep under the entry points that own both, and a conversion
 * that added two parameters at each level would be abandoned halfway.
 *
 * A NULL context means "no descent ran and nothing is held".  That is legal
 * ONLY under per-node granularity, where a member anchors on itself and no two
 * members can collide; ft_anchor_meta asserts it.
 */
struct ft_lock_ctx {
	const struct ft_descent *d;
	struct ft_held_set held;
	/*
	 * The op's PERSISTENT engine handle, when it has one.  A lock-set
	 * acquire binds to it so the commit AGES and takes its FIFO turn: a
	 * per-attempt handle is domain-less and never begin/end-bracketed, which
	 * makes the acquire a participant the escalation lane cannot order --
	 * and then a peer can hold a member while a lane-holding writer spins
	 * for it.  NULL builds a standalone acquire txn, as before.
	 */
	struct urcu_txn *op;
};

/*
 * Does the op hold @meta, and with WHAT snapshot?
 *
 * Coarsening maps several members onto ONE anchor, and a second acquire on a
 * word the op already holds aborts -EAGAIN -- so a site with more than one
 * member must test before acquiring, and record the terminal ONCE (§7.3: dedupe
 * the LOCKS, keep ALL the guards).  Linear scans, because a lock-set is small:
 * the path members number <= 5, and a fan-out either collapses to ONE anchor
 * for every child or gives each child its own, so neither shape wants a hash.
 *
 * ★ The word alone is not enough.  A member that deduped onto a word the op
 * already holds has no snapshot of its own -- the acquire never ran -- yet it
 * still needs the CLEAN pre-mark value: to re-validate its plan against the
 * word (a boundary's nr_child), and, where the word IS its own node, to retire
 * against it.  Only the FIRST acquire has that value, so the held set must
 * carry it.  @snap is left untouched when the word is not held.
 *
 * Entries in @extra that are themselves deduped (`shared`) carry no snapshot
 * and are skipped: the one non-shared entry for a word is the acquire.
 */
static inline
bool ft_held_set_snap(const struct ft_held_set *h,
		const struct cds_ft_metadata *meta, uintptr_t *snap,
		bool *ratified)
{
	unsigned int i;

	*ratified = true;
	if (!h)
		return false;
	if (h->txn)
		for (i = 0; i < h->txn->nr_locks; i++)
			if (h->txn->locks[i].meta == meta) {
				*snap = h->txn->locks[i].snap;
				return true;
			}
	for (i = 0; i < h->nr_extra; i++)
		if (h->extra[i].lock == meta && !h->extra[i].shared) {
			*snap = h->extra[i].lock_snap;
			return true;
		}
	if (h->glue && ft_glue_held_snap(h->glue, meta, snap, ratified))
		return true;
	return ft_held_set_snap(h->outer, meta, snap, ratified);
}

static inline
const struct ft_descent *ft_lock_ctx_descent(const struct ft_lock_ctx *ctx)
{
	return ctx ? ctx->d : NULL;
}

/*
 * The op holds @meta: @snap receives the acquire's clean word (see
 * ft_held_set_snap).  Pass a scratch uintptr_t where the value is not wanted.
 */
static inline
bool ft_lock_ctx_holds(const struct ft_lock_ctx *ctx,
		const struct cds_ft_metadata *meta, uintptr_t *snap,
		bool *ratified)
{
	*ratified = true;
	return ctx && ft_held_set_snap(&ctx->held, meta, snap, ratified);
}

/*
 * MAY THIS RECORD BE JUDGED ON THE OP'S WHOLE HELD SET RATHER THAN THE TXN
 * REGISTRY ALONE?  Debug-only; the answer is NO unless all four hold.
 *
 * ★ THE SHAPE DECIDES, NOT THE CALL PATH.  Registration buys exactly one
 * service -- the transfer of the mark's CLEARING -- and only a record whose new
 * value sets FT_STATE_TOMBSTONE can do without it: that terminal leaves the word
 * tombstoned, ft_meta_lock_acquire refuses a tombstone forever, so no peer can
 * re-mark it and the caller's unconditional release_if_held sweep is
 * deterministic on BOTH outcomes.  For every other record the word SURVIVES the
 * commit clean and live, a peer takes it immediately, and a late caller-side
 * release would strip that peer's mark -- which is the bug the narrow predicate
 * is the only detector of, and the one this must never bless.
 *
 * ☠ SO THE TOMBSTONE TERM IS NOT A CONVENIENCE.  Drop it and a caller passing a
 * @ctx on a release or an edge silently widens the check that catches the
 * double-clearing race.  With it, misuse fails CLOSED: the wide term simply does
 * not apply and the narrow one answers as before.
 *
 * ☠ AND THIS IS A WITNESS, NOT A VERDICT.  @ctx is consulted here, by the choke
 * point, through the SAME ft_held_set_snap the exclusion logic already trusts
 * for dedupe -- where a false positive would break real exclusion, not merely an
 * assert.  A caller may not pass "I already checked": a verdict token is
 * mintable and unfalsifiable at the point that consumes it, which is how a red
 * control goes blind.  Ownership is TAKEN, never OBSERVED; here it is merely
 * LOOKED UP in a maintained held set.
 *
 * ☞ @slot must be @owner's OWN state word: a retire tombstones the node whose
 * lock it is, and any other slot with the tombstone bit set in its value would
 * be a coincidence of encoding rather than this shape.
 *
 * The hold-trace ledger is deliberately NOT consulted: it is
 * FEATURE_FT_HOLD_TRACE-gated, and an assert whose reach depends on a second
 * unrelated knob quietly loses coverage in the config nobody checks (the rule
 * ft_flip_txn_record_tag states beside FT_OWNER_ASSERT_OWNED).  The ledger stays
 * in the counter's OWN_LEDGER lane.
 */
static inline
bool ft_owner_retire_witnessed(const struct ft_lock_ctx *ctx,
		const struct cds_ft_metadata *owner, void **slot,
		const void *new_ptr)
{
	uintptr_t snap;
	bool ratified;

	if (!ctx || !owner)
		return false;
	if (slot != (void **) (uintptr_t) &owner->state)
		return false;
	if (!((uintptr_t) new_ptr & FT_STATE_TOMBSTONE))
		return false;
	/*
	 * @ratified is ignored on purpose -- the question is OWNERSHIP, not
	 * whether this frame sampled the word, and the anchored retire's own
	 * held-set check does the same.
	 */
	return ft_lock_ctx_holds(ctx, owner, &snap, &ratified);
}

/*
 * The CLEAN word of a lock-set member whose ANCHOR is a DIFFERENT word.
 *
 * ft_held_anchor_sample_node reads the word and refuses it dirty, which is right
 * for a PEER's mark and wrong for the op's OWN: coarsening routinely anchors an
 * earlier member ON this very node, so the LOCK a raw sample sees is one this op
 * set.  Refusing it is a refusal no retry can clear -- the op re-descends and
 * re-derives the identical plan -- so the held set answers first, with the clean
 * value that acquire captured.  @held_out says which arm answered: a member
 * whose node the op holds retires it in the FUSED shape and needs no acquire-time
 * guard, both of which that flag carries.
 *
 * A word held but NOT ratified (the CALLER took it, so this op never sampled it)
 * falls through to the raw sample, which refuses exactly as before: no snapshot
 * exists to hand out, and inventing one would retire against a value nothing
 * vouched for.
 */
static inline
int ft_member_node_snap(const struct ft_lock_ctx *ctx,
		const struct cds_ft_metadata *node, uintptr_t *node_snap,
		bool *held_out)
{
	bool ratified;

	if (ft_lock_ctx_holds(ctx, node, node_snap, &ratified) && ratified) {
		*held_out = true;
		return 0;
	}
	*held_out = false;
	return ft_held_anchor_sample_node(node, node_snap);
}

/*
 * Build an op's lock context from its descent and its commit txn -- the shape
 * every op has.  An op that also keeps marks outside the registry fills
 * @held.extra itself afterwards.
 */
/*
 * @op is the enclosing operation's PERSISTENT txn handle, or NULL where the op
 * genuinely has none (an exclusive trie, or a standalone internal txn with no
 * retry loop).
 *
 * IT IS A PARAMETER, not a field left for the caller to patch afterwards.  It
 * used to be initialised to NULL unconditionally, and three sites out of
 * thirty-two remembered to assign it: measured 6 enrolled acquires against
 * 5,824,454 domain-less ones.  A refused acquire on a domain-less handle cannot
 * age its op (ft_dlm_acquire_set's eagain path), so the op never reaches the
 * escalation domain's FIFO lane and a contended anchor starves it.  Making it an
 * argument is what stops a new site from re-opening that hole silently.
 */
static inline
void ft_lock_ctx_init(struct ft_lock_ctx *ctx, const struct ft_descent *d,
		struct ft_flip_txn *txn, struct urcu_txn *op)
{
	ctx->d = d;
	ctx->held.txn = txn;
	ctx->held.extra = NULL;
	ctx->held.nr_extra = 0;
	ctx->held.glue = NULL;
	ctx->held.outer = NULL;
	ctx->op = op;
}

/*
 * The byte-depth to anchor @nf by, for a member the site reached through a
 * BACK-POINTER (ft_resolve_parent_slot and friends) rather than by descending
 * to it -- the shape most lock-sets take for their P and GP members.
 *
 * FALSE means the descent does not describe @nf, so this op has no depth for it
 * and must RE-PLAN.  Per-node granularity always succeeds with an unused depth:
 * a member anchors on itself there, so no descent is required and none of these
 * sites pay for the lookup.
 */
static inline
bool ft_lock_ctx_depth_of_at(const char *fn, int line,
		const struct cds_ft *ft,
		const struct ft_lock_ctx *ctx,
		const struct cds_ft_inode_flag *nf, unsigned int *depth)
{
	if (ft->lock_spacing == CDS_FT_LOCK_SPACING_PER_NODE) {
		*depth = 0;
		return true;
	}
	/*
	 * Root-only anchors EVERY member on the root, so like per-node it never
	 * reads the depth -- only the exponential schedule selects a level from
	 * it.  Answering here is what keeps a member the descent never passed
	 * (a chain head's holder below an empty-key merge point) from bailing to
	 * a re-descend that must fail the same way forever.  Non-zero, so
	 * ft_anchor_meta's "depth 0 IS the root" early-out does not mistake a
	 * deep member for the root; both roads lead to the root regardless.
	 */
	if (ft->lock_spacing == CDS_FT_LOCK_SPACING_ROOT_ONLY) {
		*depth = 1;
		return true;
	}
	if (caa_likely(ft_descent_depth_of(ft_lock_ctx_descent(ctx), nf, depth)))
		return true;
#ifdef FEATURE_FT_HOLD_TRACE
	if (ft_hold_trace_report_ok()) {
		const struct ft_descent *d = ft_lock_ctx_descent(ctx);

		fprintf(stderr,
			"FT UNDATABLE MEMBER: %s:%d nf=%p descent=%p\n",
			fn, line, (const void *) nf, (const void *) d);
		if (d)
			fprintf(stderr,
				"  window nf=%p@%u pnf=%p@%u ppnf=%p@%u pppnf=%p@%u\n",
				(void *) d->nf, d->depth,
				(void *) d->pnf, d->pdepth,
				(void *) d->ppnf, d->ppdepth,
				(void *) d->pppnf, d->pppdepth);
	}
#endif
	(void) fn; (void) line;
	return false;
}

#define ft_lock_ctx_depth_of(ft, ctx, nf, depth)			\
	ft_lock_ctx_depth_of_at(__func__, __LINE__, (ft), (ctx), (nf), (depth))

/*
 * The key bytes @nf consumes to reach its child: one for a bitmap node, the
 * whole run for a compressed one -- kept in the TARGET for the skip-encoded
 * form, whose tag is therefore tested FIRST, as everywhere else that dispatches
 * on node kind.
 */
static inline
unsigned int ft_node_span(const struct cds_ft *ft,
		const struct cds_ft_inode_flag *nf)
{
	if (ft_node_skip_compressed(nf))
		return ft_skip_to_compressed(ft, nf)->len;
	if (ft_node_compressed(nf))
		return ft_compressed_node_ptr(nf)->len;
	return 1;
}

/*
 * Date @parent_nf -- a member reached ONE HOP UP from a node whose byte-depth
 * @child_depth is already known -- for the sets the descent's window cannot
 * cover.
 *
 * A {C, P, GP} lock-set names three CONSECUTIVE ancestors, and the window holds
 * the last four nodes the descent passed, not the last four a set names: with C
 * already at the third slot, GP falls off the end.  Stepping one hop up from a
 * DATED node is legal where a climb is not (§5.3 -- a climb starts undated, and
 * byte-depth is absolute): a node's span is a property of the node itself, so
 * the parent of a node at @child_depth sits at @child_depth - span(parent).
 *
 * FALSE when the span exceeds @child_depth: @parent_nf is then not the parent of
 * anything at that depth, so the plan is stale and the op re-descends.
 */
static inline
bool ft_parent_depth_of(const struct cds_ft *ft,
		const struct cds_ft_inode_flag *parent_nf,
		unsigned int child_depth, unsigned int *depth)
{
	unsigned int span = ft_node_span(ft, parent_nf);

	if (span > child_depth)
		return false;
	*depth = child_depth - span;
	return true;
}

/*
 * ft_lock_ctx_depth_of for a member the site reached as the PARENT of a node it
 * has already dated: the descent's window answers when it describes @parent_nf,
 * and the one-hop derivation covers the rest.
 */
static inline
bool ft_lock_ctx_depth_of_parent(const struct cds_ft *ft,
		const struct ft_lock_ctx *ctx,
		const struct cds_ft_inode_flag *parent_nf,
		unsigned int child_depth, unsigned int *depth)
{
	if (ft->lock_spacing == CDS_FT_LOCK_SPACING_PER_NODE) {
		*depth = 0;
		return true;
	}
	if (ft_descent_depth_of(ft_lock_ctx_descent(ctx), parent_nf, depth))
		return true;
	if (caa_likely(ft_parent_depth_of(ft, parent_nf, child_depth, depth)))
		return true;
#ifdef FEATURE_FT_HOLD_TRACE
	if (ft_hold_trace_report_ok())
		fprintf(stderr,
			"FT UNDATABLE PARENT: nf=%p span=%u child_depth=%u\n",
			(const void *) parent_nf, ft_node_span(ft, parent_nf),
			child_depth);
#endif
	return false;
}

/*
 * ft_lock_ctx_depth_of for a member the site reached as a CHILD of the node the
 * descent stopped on -- the shape a BUILD's re-parent targets take, since a
 * build works below the descent's cursor and the window names only nodes the
 * descent ENTERED.
 *
 * @child_parent is @child_nf's LIVE parent, resolved from its back-pointer: the
 * anchor must describe where the node is NOW, which for a node about to be
 * MOVED is its old path, not the one it is being built into (§3).
 *
 * Exact for ONE hop and refused beyond it, which is the same bound
 * ft_descent_anchor_child carries and for the same reason: the cursor spans the
 * whole gap [d->depth, child_depth), so the child's own start is the only node
 * boundary inside it.  A member two hops down has a boundary between it and the
 * cursor that the table never saw.
 *
 * FALSE is a RE-PLAN, not an error -- the op has no depth for the node, and
 * anchoring it with another node's depth is the disagreement §1 forbids.
 */
static inline
bool ft_lock_ctx_depth_of_cursor_child(const struct cds_ft *ft,
		const struct ft_lock_ctx *ctx,
		const struct cds_ft_inode_flag *child_parent,
		unsigned int *depth)
{
	const struct ft_descent *d = ft_lock_ctx_descent(ctx);

	if (ft->lock_spacing == CDS_FT_LOCK_SPACING_PER_NODE) {
		*depth = 0;
		return true;
	}
	/*
	 * NO parent at all: the node is not reachable from the trie, either
	 * because it sits at a ROOT position -- depth 0 by definition -- or
	 * because this op BUILT it and has not published it yet.  The second is
	 * the fold's COW copy: ft_rekey_cow_stop's @stop_prime is not in the
	 * glue's @built array, because a DIFFERENT step of the op built it, so
	 * the glue's own fresh test cannot see it and it arrives here looking
	 * live.
	 *
	 * Depth 0 is right for both.  The root IS its own anchor under every
	 * spacing (§2), and an unpublished node has no peer to agree WITH --
	 * §1's agreement binds only nodes two ops can both reach.
	 */
	if (!child_parent) {
		*depth = 0;
		return true;
	}
	if (!d || !d->nf || child_parent != d->nf)
		return false;
	*depth = d->depth + ft_node_span(ft, d->nf);
	return true;
}

/*
 * Key-guided walk to @key_len, stopping at an external or a short path.
 *
 * The handle-derived entry points -- node-handle remove, remove-all, replace --
 * reach their holder through a back-pointer and never walk, so this is their
 * only source of per-level BYTE-DEPTHS, which is what selects each lock-set
 * member's anchor.  (Remove's stale-holder recovery arm uses the same walk to
 * re-derive a tombstoned holder from the authoritative forward path.)
 * @ik_ret receives the key cursor the walk consumed.
 */
static
void ft_anchor_descend(struct cds_ft *ft, struct ft_descent *d,
		const uint8_t *iter_key, size_t key_len, const uint8_t **ik_ret)
{
	const uint8_t *ik = iter_key;

	ft_descent_init(d, ft);
	while (d->depth < key_len) {
		if (!d->nf || ft_node_external(d->nf))
			break;
		if (ft_node_compressed(d->nf)) {
			ft_descent_traverse_compressed(ft, d,
				ft_compressed_node_ptr(d->nf), &ik);
			continue;
		}
		ft_descent_step(ft, d, *(ik++));
	}
	*ik_ret = ik;
}

/*
 * Reserved edges a FREEZE of @n anchored nodes costs.
 *
 * Per-node granularity fuses each retire into the single
 * {LOCK|s -> TOMBSTONE|s}, so one edge per node.  Coarsening splits that in two
 * -- a release on the surviving ancestor plus a plain tombstone on the node --
 * and dedupe can only remove releases, never add them, so twice is an upper
 * bound (§7.3: the reservation stays safe, merely loose).
 */
static inline
unsigned int ft_freeze_reserve(const struct cds_ft *ft, unsigned int n)
{
	return ft->lock_spacing == CDS_FT_LOCK_SPACING_PER_NODE ? n : 2 * n;
}

/*
 * Extra edges a GLUE build's split-retire terminal costs beyond its per-node
 * form.  Per-node fuses the terminal into one {LOCK|s -> TOMBSTONE|s} on the one
 * word @cn's lock and @cn's body share; coarsening splits it into a tombstone on
 * @cn plus a release on its anchor.  A glue that sets @fence_split_cn budgets
 * this so its post-detach publish stays the infallible step it is documented to
 * be.
 */
static inline
unsigned int ft_glue_split_cn_reserve(const struct cds_ft *ft)
{
	return ft->lock_spacing == CDS_FT_LOCK_SPACING_PER_NODE ? 0 : 1;
}

/*
 * "@ctx's descent knows the depth; look it up."
 *
 * A site that reached its member through a BACK-POINTER usually cannot name a
 * byte-depth, and the descent's window is what dates it -- but that lookup can
 * FAIL (a node this descent never passed), and most publish sites sit past the
 * last point where a clean bail is available.  Passing this makes the failure
 * the same one a CONTENDED acquire already produces there: the commit aborts and
 * the caller re-descends.  A site that still has a bail should look the depth up
 * itself and take it.
 */
#define FT_DEPTH_FROM_DESCENT	UINT_MAX

/*
 * THE ACQUIRE CHOKE POINT.  Take the lock-set member @nf -- a node at byte-depth
 * @depth -- for this op: resolve it to the word its acquire must actually CAS
 * (its own under per-node granularity, its ANCHOR's under a coarser one), refuse
 * to take that word TWICE, mark it, and ratify @nf's own state word.
 *
 * Every acquire in the library goes through here or through its transacted
 * sibling ft_dlm_acquire_set.  That is not tidiness: agreement is the
 * property that every op computes the SAME anchor for a node, which 40
 * independent derivations cannot be trusted to preserve, and the dedupe above is
 * only sound when the op's held set sees EVERY word the op took.  One site left
 * outside collides with the converted ones and livelocks
 * (doc/design/ft-dlm-lock-coarseness.md §1, §9).
 *
 * On 0, @held describes the member: @held->lock is the word to release /
 * register / record the {LOCK|s -> s} release against, and @held->node_snap the
 * clean word of @nf ITSELF, which is what a RETIRE tombstones.  @held->shared
 * says the op already held the word, so this member owes NO release and NO
 * terminal -- the acquire that first took it recorded both.
 *
 * -EAGAIN when a peer holds the word, or when @nf's own word is dirty
 * (PROXY | TOMBSTONE | LOCK).  Per-node granularity gets that second check for
 * free -- the two words are one -- and coarsening must not lose it: the op
 * retires @nf against @node_snap, so a dirty value there is a retire that could
 * only ever abort.  Nothing is held on failure.
 *
 * @nf's word is sampled AFTER the mark lands, which is where it is stable: from
 * that point every mutator of @nf must first take the word we now hold.  The
 * transacted sibling cannot do that -- its mark lands only at the commit -- and
 * pays for it with a sample-then-guard instead.
 */
struct ft_dlm_member {
	struct cds_ft_inode_flag *nf;
	struct cds_ft_metadata *node;
	unsigned int depth;
	struct cds_ft_metadata *guard_child;
	struct cds_ft_inode_flag *guard_pf;
	struct ft_held_anchor held;
};

/* Defined below; the single-member acquire is a one-element set. */
static inline
int ft_dlm_acquire_set_at(const char *fn, int line,
		const struct cds_ft *ft, const struct ft_lock_ctx *ctx,
		struct ft_dlm_member *set, int nr);

static inline
int ft_acquire_member_at(const char *fn, int line,
		const struct cds_ft *ft, const struct ft_lock_ctx *ctx,
		struct cds_ft_inode_flag *nf, struct cds_ft_metadata *node,
		unsigned int depth, struct ft_held_anchor *held)
{
	struct ft_dlm_member m = { .nf = nf, .node = node, .depth = depth,
		.guard_child = NULL, .guard_pf = NULL };
	int ret;

#ifdef FEATURE_FT_FAULT_INJECT
	/*
	 * Test-only: fail this acquire exactly as a peer holding the word would
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
	ret = ft_dlm_acquire_set_at(fn, line, ft, ctx, &m, 1);
	if (ret)
		return ret;		/* nothing acquired */
	*held = m.held;
	return 0;
}

#define ft_acquire_member(ft, ctx, nf, node, depth, held)		\
	ft_acquire_member_at(__func__, __LINE__, (ft), (ctx), (nf),	\
		(node), (depth), (held))

/*
 * Drop every RELEASE-terminal lock the op holds, leaving the nodes LIVE: the
 * bail path of the above, for a member set not yet handed to a txn.  Once the
 * members ARE registered (ft_flip_txn_lock_register, on the success path),
 * the txn's registry owns the unlock instead and this must not run.
 */
static inline
void ft_unlock_held(const struct ft_held_anchor *set, unsigned int n)
{
	unsigned int i;

	for (i = 0; i < n; i++)
		if (!set[i].shared)
			ft_meta_lock_release(set[i].lock);
}

static inline
void ft_flip_txn_lock_release_all(struct ft_flip_txn *t)
{
	unsigned int i;

	for (i = 0; i < t->nr_locks; i++)
		ft_meta_lock_release(t->locks[i].meta);
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
	FT_TK_COUNT_END(t, FT_TK_BAILED);
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

	/*
	 * THE FT'S OWNERSHIP WORDS ARE ITS STATE WORDS.  A node's lock lives in
	 * meta->state (FT_STATE_PROXY), and this commit's release of that lock
	 * must become visible only after every structural word the lock protects
	 * is plain -- otherwise a peer acquires on the strength of the release
	 * while our SW parks are still parked, and our own settle overwrites
	 * what it then publishes.
	 */
	urcu_txn_desc_set_late_tag(t->mtxn->desc, FT_STATE_PROXY);
	if (caa_unlikely(t->acquire_miss)) {
		/*
		 * A lock-set member was not acquired, so this attempt writes a
		 * slot it does not own: discard it unpublished and report the
		 * reason, which every caller routes to a re-descend (ABORT) or
		 * out to the app (MEMORY_ERROR).  Nothing was parked either way.
		 *
		 * Age the handle on ABORT only, exactly as a real contention
		 * abort does inside urcu_txn_commit_flavor -- without it the op
		 * never advances txn->retry, never escalates to the FIFO lane,
		 * and a contended node could starve it indefinitely.  An
		 * allocation failure is NOT contention: aging for it spends the
		 * escalation budget on a conflict that never happened, and
		 * re-descending cannot make memory appear.
		 */
		enum urcu_txn_status miss_st = t->acquire_enomem ?
			URCU_TXN_STATUS_MEMORY_ERROR : URCU_TXN_STATUS_ABORT;

		if (miss_st == URCU_TXN_STATUS_ABORT)
			urcu_txn_conflict(t->mtxn);
		FT_TP(txn_commit, (const void *) t->mtxn, (int) miss_st);
		FT_TK_COUNT_END(t, FT_TK_MISS);
		ft_flip_txn_destroy(t);
		return miss_st;
	}
	st = urcu_txn_commit_flavor(t->mtxn, reclaim);
	FT_TP(txn_commit, (const void *) t->mtxn, (int) st);
	FT_TK_COUNT_END(t, st == URCU_TXN_STATUS_OK ? FT_TK_OK :
			(st == URCU_TXN_STATUS_MEMORY_ERROR ? FT_TK_MEMERR :
				FT_TK_ABORT));
	/*
	 * node locks: a committed txn transitioned each registered node
	 * through the terminal its op recorded -- {LOCK|s -> TOMBSTONE|s}
	 * (retire) or {LOCK|s -> s} (release) -- so the lock is already
	 * consumed and the registry is not drained.  ABORT settled the state
	 * record back to its old value -- lock still set -- and MEMORY_ERROR
	 * parked nothing, so both must clear the reversible bit or the still-live
	 * nodes would fail every later peer guard forever.
	 */
	if (caa_unlikely(st != URCU_TXN_STATUS_OK)) {
		ft_flip_txn_lock_release_all(t);
	} else {
		unsigned int i;

		/*
		 * The terminals above are how a COMMITTED registered lock stops
		 * being held: no ft_meta_lock_release runs, so the trace ledger
		 * would otherwise keep the word forever.
		 */
		for (i = 0; i < t->nr_locks; i++)
			ft_hold_trace_drop(t->locks[i].meta);
	}
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
/*
 * THE RECORDER CORE, taking the op's held-set witness as a PARAMETER.
 *
 * @dbg_ctx is consumed by FT_OWNER_ASSERT_OWNED_CTX and nothing else: it is the
 * debug-only witness for the ONE record shape whose mark is swept rather than
 * registered (ft_owner_retire_witnessed has the argument).  NULL -- what every
 * wrapper below passes -- is the ordinary registry-only predicate, so no
 * existing caller's check changes by a bit.
 */
static inline
void __ft_flip_txn_record_tag_ctx(struct ft_flip_txn *t,
		const struct ft_lock_ctx *dbg_ctx,
		struct cds_ft_metadata *owner, void **slot,
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
	 *
	 * @owner is the node whose lock makes that park legal -- see
	 * FT_OWNER_ASSERT_OWNED for the rule and for why the counter beside
	 * the assert runs in EVERY mode while the assert itself waits for a
	 * per-op arm.  It is named by the SITE because a bare `void **slot`
	 * cannot yield it: the owner differs by FIELD KIND (an edge's four
	 * fields are the PARENT's, a state word is its own node's, §8), and no
	 * arithmetic on the address recovers that.
	 */
	FT_ROOT_ASSERT_NOT_ROOT(t, slot);
	/*
	 * ☠ NOT the DLM lock TAKE.  It reaches this same helper, but it is the
	 * ARBITRATION POINT and must stay MW forever -- so it is not part of
	 * the conversion surface, and counting it would put the whole acquire
	 * lane in OWN_MISS and read as a gap that can never close.  A take is
	 * also the one record that CANNOT be owner-held by construction: it is
	 * what makes the op the owner.
	 */
	if (!FT_TK_TXN_IS_TAKE(t)) {
		/*
		 * COUNTED ONLY WHERE THE QUESTION IS OPEN, so that OWN_HELD +
		 * OWN_MISS == MW_STRUCT exactly: the surface, split by whether
		 * arming it would be legal.
		 *
		 * An ALREADY-ARMED txn is excluded because its arm was decided
		 * on a wider argument -- a COARSE or exclusive trie excludes
		 * every peer, and its ops register no per-node lock for most of
		 * what they park -- so its records would land in OWN_MISS and
		 * read as a gap in ops that have no gap.  The mode with that
		 * arm is Phase A's, already done; what these two count is the
		 * mode Phase B has yet to convert.
		 */
		if (!t->structural_sw)
			FT_TK_COUNT_OWN(t, owner);
	}
	/*
	 * ASKED OF EVERY SW-CAPABLE RECORD, not only of the ones that actually
	 * park, and NOT behind the take gate above -- that gate is spelled in
	 * a -DFT_DEBUG_TXN_KIND macro while this assert answers to
	 * --enable-rcu-debug, and an assert whose reach depends on a SECOND,
	 * unrelated knob is one that quietly loses coverage in the config
	 * nobody thought to check.  A take needs no exemption anyway: it rides
	 * the acquire lane, which never claims.
	 *
	 * A txn that CLAIMS per-op ownership is making the claim about its
	 * whole record set, so the check belongs where the set is -- and
	 * asking it outside the park branch lets the claim be made WITHOUT
	 * arming (ft_flip_txn_claim_per_op), which turns "would arming this
	 * site be legal?" into an abort at the offending record instead of a
	 * number to interpret.
	 */
	FT_OWNER_ASSERT_OWNED_CTX(t, dbg_ctx, owner, slot, new_ptr);
	if (t->structural_sw) {
		FT_TK_COUNT_REC(t, FT_TK_SW);
		ret = urcu_txn_store_sw(t->mtxn, slot, old_ptr, new_ptr, tag);
	} else {
		/*
		 * The DLM lock TAKE reaches this same branch and is the one MW
		 * record that must never become SW, so it is counted apart from
		 * the conservative ones (ft-txn-kind-stats.h).
		 */
		FT_TK_COUNT_REC(t, FT_TK_TXN_IS_TAKE(t) ?
				FT_TK_MW_LOCK : FT_TK_MW_STRUCT);
		ret = urcu_txn_store_mw(t->mtxn, slot, old_ptr, new_ptr, tag);
	}
	assert(!ret);
	(void) ret;	/* reserved up front -> never fails */
}

static inline
void ft_flip_txn_record_tag(struct ft_flip_txn *t,
		struct cds_ft_metadata *owner, void **slot,
		void *old_ptr, void *new_ptr, uintptr_t tag)
{
	__ft_flip_txn_record_tag_ctx(t, /*dbg_ctx=*/ NULL, owner, slot,
		old_ptr, new_ptr, tag);
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
	FT_TK_COUNT_REC(t, FT_TK_MW_ALWAYS);
	ret = urcu_txn_store_mw(t->mtxn, slot, old_ptr, new_ptr, tag);
	assert(!ret);
	(void) ret;	/* reserved up front -> never fails */
}

/*
 * Record a TRIE ROOT edge.  ALWAYS MW, whatever @t's structural_sw mode and
 * whichever trie the slot belongs to.
 *
 * A ROOT LIVES IN NO NODE.  Every other structural slot a structural_sw op
 * writes sits inside a node whose state word the op holds, and that hold is
 * what makes an unarbitrated park legal.  The root POINTER has no such node --
 * ft_node_recompact says so at its own NULL-parent arm ("no node to lock,
 * auto-guarded by the root-slot CAS") -- so what arbitrates it is the CAS, and
 * an SW park is not a CAS: it neither arbitrates nor is VISIBLE to one,
 * because the engine's kind rule is SW xor MW per slot GLOBALLY.  A park here
 * would be a plain store racing every other writer of the trie root.
 *
 * ☠ NOT because the root pointer COULD NOT be locked.  A lock word on struct
 * cds_ft itself would span the empty<->non-empty transition fine; only putting
 * the word in the root NODE is impossible.  The reasons a lock is the wrong
 * instrument are design reasons rather than impossibilities:
 *
 *   - EVERY OTHER WRITER OF THIS SLOT IS ALREADY MW.  insert, remove, graft,
 *     graft_swap, merge, detach and the bulk root swaps all CAS it, so an SW
 *     op is the outlier and this helper is what makes it conform.  A per-trie
 *     lock word is a legitimate alternative, but it is a tree-wide protocol
 *     change across all seven ops, and it buys uniformity, not correctness.
 *   - A node lock buys a LONG UNFAILABLE WINDOW: reserve the set, build
 *     invisibly, park at the end.  The root pointer takes ONE transition per
 *     op and its collision window is a POINT, which is what an optimistic CAS
 *     is for; a lock would be pessimistic over the whole build of the hottest
 *     word in the structure.
 *   - A fence's lifetime here is bounded by NODE DEATH ("the acquire refuses a
 *     TOMBSTONE, so nobody re-locks that word").  A trie-level word never
 *     dies, so a mark leaked on it would have no natural end.
 *
 * The rule is a property of the SLOT, not of the txn's named trie, which is
 * what lets a CROSS-TRIE dual -- one txn, two roots (ft_graft / graft_swap) --
 * record both of them MW while its own trie's content parks SW.  Counted
 * MW_ALWAYS: a root can never convert, so it is not part of the MW_STRUCT
 * conversion surface.
 */
static inline
void ft_flip_txn_record_root(struct ft_flip_txn *t, void **slot,
		void *old_ptr, void *new_ptr)
{
	ft_flip_txn_record_tag_mw(t, slot, old_ptr, new_ptr,
		FT_FLIP_PROXY_TAG);
}

/*
 * An EXTERNAL HEAD's BACK CHANNEL re-pointed at a new parent: cell->parent with
 * the ordered list on, en->prev with it off.  ALWAYS MW, for the reason
 * ft_flip_txn_record_parent_word decides per record on @child_held -- a
 * structural edge may park SW only where the op holds the DLM lock over the
 * slot, and here it never can, because neither an external node nor its cell
 * carries a state word to hold.  This is that predicate's PERMANENT FALSE ARM,
 * not a site awaiting conversion, so it takes the ft_flip_txn_record_root
 * treatment: a word that can never convert leaves the MW_STRUCT surface.
 *
 * Parking it would claim an exclusion the op does not have -- two ops re-homing
 * one head both park, neither fails, and the last install wins.  MW makes the
 * second writer's expected-old mismatch and abort, which the retry lane exists
 * to absorb.  Byte-identical on an unarmed txn (there record_tag IS
 * record_tag_mw); what it changes is that an ARMED op cannot park this word.
 *
 * ☞ This settles the KIND, and only the kind.  WHO owns the back edge --
 * FT_OWNER_NONE_EXTERNAL_HEAD's design question, the model's §8.2 assigning
 * `parent` to P against the code keying it on holding the child -- stays open
 * and is Phase C's ledger to re-examine.
 *
 * ☠ Reach for this only after checking for a HOLDER in scope: the head-promote
 * sites looked ownerless and had their holder as a parameter all along.
 */
static inline
void ft_flip_txn_record_head_back_edge(struct ft_flip_txn *t, void **slot,
		void *old_ptr, void *new_ptr)
{
	ft_flip_txn_record_tag_mw(t, slot, old_ptr, new_ptr,
		FT_FLIP_PROXY_TAG);
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
	if (v)
		FT_TK_COUNT_ARMED(t);
}

/*
 * PHASE B ARM (§4 of doc/design/mw-to-fine-locking-remainder.md): opt this
 * commit's structural edges into SW parks on the strength of THE LOCK-SET IT
 * HOLDS, rather than of a trie-wide exclusion.
 *
 * ★ THE ORDER IS THE POINT.  Phase A's arm is decided inside the constructor,
 * from the trie (ft_txn_content_sw_ok), because a COARSE or exclusive trie
 * excludes every peer writer before the op does anything.  Under FINE nothing
 * is excluded until the op's ACQUIRE COMMITS, so the arm cannot precede it --
 * and taking the answer from @t->locks makes "acquire, then arm from what you
 * hold" the shape of the call rather than a rule in a document.
 *
 * Refuses on a trie the constructor already decided (COARSE / exclusive): that
 * txn is armed on a wider argument, and marking it per-op would point the
 * record-time assert at words the wide mutex covers without registering.
 *
 * Refuses an EMPTY registry.  A commit holding nothing owns nothing, so every
 * park it made would be unarbitrated -- the exact defect the assert exists to
 * catch.  Refusing leaves it all-MW, which is stricter and always sound.
 *
 * ☠ THE REGISTRY MUST BE COMPLETE AT THE CALL.  A lock registered AFTER this
 * point still protects its word, but a record planted in between is checked
 * against a registry that does not yet name its owner -- so the arm belongs
 * after the last ft_flip_txn_lock_register of the op, not after the first.
 */
/*
 * THE DRY RUN'S GATE: claim exactly where ft_flip_txn_arm_per_op would ARM, and
 * nowhere else.
 *
 * ft_flip_txn_claim_per_op on its own claims unconditionally, which is wrong for
 * a readiness measurement in the same way an ungated arm would be wrong for a
 * conversion: it points the record-time assert at tries the arm REFUSES -- a
 * COARSE or exclusive trie armed on the constructor's wider argument, or a
 * non-FINE one -- and every record there reports a gap the site does not have.
 * Measured: the ungated form aborts inside test_lifecycle_lock_spacing, a
 * single-threaded trie no per-op arm would ever touch.
 *
 * ☠ IT CANNOT REPRODUCE THE @nr_locks REFUSAL, and that is a real limit rather
 * than an oversight.  The arm belongs after the op's LAST lock_register, where
 * an empty registry means "this commit owns nothing"; a dry run has to claim
 * BEFORE the records it wants checked, which for most sites is before the
 * acquires finish.  So a site whose records precede its own acquires still
 * reports misses here -- and those are ORDERING findings, the class already
 * fixed three times (92e27199, e9268e13, 8e8d0232), not false alarms.  Read a
 * miss as "this record is planted before its owner is registered" first.
 */
static inline
void ft_flip_txn_claim_per_op_armable(const struct cds_ft *ft,
		struct ft_flip_txn *t)
{
	if (ft_txn_content_sw_ok(ft))
		return;		/* the constructor armed it trie-wide */
	if (!ft || !ft->lock_fine)
		return;
	ft_flip_txn_claim_per_op(t);
}

static inline
void ft_flip_txn_arm_per_op(const struct cds_ft *ft, struct ft_flip_txn *t)
{
	if (ft_txn_content_sw_ok(ft))
		return;		/* the constructor armed it trie-wide */
	if (!ft || !ft->lock_fine || !t->nr_locks)
		return;
	ft_flip_txn_claim_per_op(t);
	ft_flip_txn_set_structural_sw(t, true);
}


static inline
void ft_flip_txn_record_reserved(struct ft_flip_txn *t,
		struct cds_ft_metadata *owner, void **slot,
		void *old_ptr, void *new_ptr)
{
	ft_flip_txn_record_tag(t, owner, slot, old_ptr, new_ptr,
		FT_FLIP_PROXY_TAG);
}

/*
 * Record a FORWARD PUBLISH whose slot a DESCENT resolved, so its shape is not
 * known statically: it is @ft's root exactly when the descent stood at depth 0
 * (ft_descent_init seeds d->nfp = &ft->root) and an in-node child slot at every
 * other depth.  Dispatches to the always-MW root record for the first case.
 *
 * @ft is the trie whose root the slot could be -- the one the descent ran in --
 * which is what keeps this correct for a CROSS-TRIE op: the caller names the
 * side it descended, not the txn's named trie.
 */
static inline
void ft_flip_txn_record_publish(struct ft_flip_txn *t, struct cds_ft *ft,
		struct cds_ft_metadata *owner,
		struct cds_ft_inode_flag **slot,
		void *old_ptr, void *new_ptr)
{
	if (slot == &ft->root)
		ft_flip_txn_record_root(t, (void **) slot, old_ptr, new_ptr);
	else
		ft_flip_txn_record_reserved(t, owner, (void **) slot, old_ptr,
			new_ptr);
}

/*
 * Replay a recorded publish (@rec, from _ft_publish_to_parent) into @t, each
 * edge with the KIND its slot demands: a TRIE ROOT records MW whatever the
 * txn's mode (ft_flip_txn_record_root), every in-node slot takes the ordinary
 * structural_sw dispatch.
 *
 * The rule lives here once, for the callers that record a rec STRAIGHT into a
 * txn.  The callers that first convert a rec into ft_ord_cell_edges
 * (ft_remove_commit_rec, ft_pub_rec_sedges, ft_ord_cell_flip_rec_replace) carry
 * BOTH per-edge answers -- @root and @owner -- across the conversion and
 * dispatch them in ft_ord_cell_flip_into / ft_ord_cell_record_into_ft.  The two
 * travel together because neither is re-derivable at the replay: @root because
 * only the publishing descent saw the NULL parent, @owner because a bare slot
 * address cannot yield the node it lives in.
 */
static inline
void ft_flip_txn_record_pub_rec(struct ft_flip_txn *t,
		const struct ft_pub_rec *rec)
{
	unsigned int i;

	for (i = 0; i < rec->n; i++) {
		if (rec->root[i])
			ft_flip_txn_record_root(t, (void **) rec->slot[i],
				(void *) rec->old_val[i],
				(void *) rec->new_val[i]);
		else
			ft_flip_txn_record_reserved(t, rec->owner[i],
				(void **) rec->slot[i],
				(void *) rec->old_val[i],
				(void *) rec->new_val[i]);
	}
}

/*
 * THE STATE-WORD PROTOCOL, and the record KIND each of its steps requires.
 *
 * A node's state word is written by three different steps, and they do NOT all
 * take the same kind, because only one of them arbitrates between contending
 * ops:
 *
 *   1. TAKE the lock protecting the node  {clean -> LOCK|s}          -- MW.
 *      THE ARBITRATION POINT.  Two ops racing for the node are decided here and
 *      nowhere else: an MW record installs with a CAS-old, so the loser's commit
 *      aborts.  An SW park cannot fail, so an SW take would hand BOTH ops the
 *      node.  ft_dlm_lock asserts !structural_sw for exactly this reason.
 *   2. SET THE TOMBSTONE  {LOCK|s -> TOMBSTONE|s}                    -- SW.
 *      Legitimate BECAUSE step 1 already won the word.  Re-validating here would
 *      arbitrate a race that was settled one step earlier, at the cost of an
 *      abort the caller has no way to retry (the copy is already built).
 *   3. RELEASE the lock   {LOCK|s -> s}                              -- SW, for
 *      the same reason.  The lock released need not sit on the node that was
 *      tombstoned: under a coarse acquire the mark is on a surviving ANCESTOR,
 *      so the retire's two halves land on two words.
 *
 * ⇒ AN SW TOMBSTONE IS NOT A DEFECT, IT IS THE PROTOCOL.  What is a defect is an
 * SW take, or a step-2/3 record made without the lock step 1 was supposed to
 * take -- and, on the far side, an op that writes or validates this word having
 * never performed step 1 at all.  A peer cannot substitute a {live -> live}
 * validate for the take: the take is what excludes it, and the engine's
 * "SW xor MW, globally" rule means an MW validate is not arbitrated against the
 * SW parks of steps 2 and 3 anyway.  Ownership is taken, never observed.
 *
 * CONDITIONAL ON THE MODE.  Steps 2 and 3 are SW only where the op runs under
 * lock_fine with structural_sw set; every other op records all-MW, which is
 * stricter and always sound.  That is what the @sw_ok argument selects: it says
 * "SW is PERMITTED for this step", not "SW is used".
 */
#define ft_flip_txn_record_state(t, meta, o, n)				\
	ft_flip_txn_record_state_kind((t), (meta), (o), (n), 1)
/*
 * The same, carrying the op's held-set witness for the ANCHORED RETIRE -- the
 * one record shape whose mark is swept rather than registered.  Every other
 * caller uses the plain spelling and keeps the registry-only predicate.
 */
#define ft_flip_txn_record_state_ctx(t, ctx, meta, o, n)			\
	ft_flip_txn_record_state_kind_ctx((t), (ctx), (meta), (o), (n), 1)
#define ft_flip_txn_record_state_mw(t, meta, o, n)			\
	ft_flip_txn_record_state_kind((t), (meta), (o), (n), 0)

static inline
void ft_flip_txn_record_state_kind_ctx(struct ft_flip_txn *t,
		const struct ft_lock_ctx *dbg_ctx,
		struct cds_ft_metadata *meta, void *old_ptr, void *new_ptr,
		int sw_ok)
{
	if (sw_ok)
		__ft_flip_txn_record_tag_ctx(t, dbg_ctx, /*owner=*/ meta,
			(void **) &meta->state,
			old_ptr, new_ptr, FT_STATE_PROXY);
	else
		ft_flip_txn_record_tag_mw(t, (void **) &meta->state,
			old_ptr, new_ptr, FT_STATE_PROXY);
}

static inline
void ft_flip_txn_record_state_kind(struct ft_flip_txn *t,
		struct cds_ft_metadata *meta, void *old_ptr, void *new_ptr,
		int sw_ok)
{
	ft_flip_txn_record_state_kind_ctx(t, /*dbg_ctx=*/ NULL, meta,
		old_ptr, new_ptr, sw_ok);
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
	FT_TK_TXN_SET_TAKE(t, true);
	ft_flip_txn_record_state(t, meta,
			(void *) s, (void *) (s | FT_STATE_LOCK));
	FT_TK_TXN_SET_TAKE(t, false);
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
	FT_TK_COUNT_REC(t, FT_TK_VALIDATE);
	urcu_txn_validate(t->mtxn, (void **) &child->parent_word,
			(void *) expected_pf, FT_FLIP_PROXY_TAG);
}

/*
 * Guard a coarsened member's OWN state word into the acquire commit, from the
 * value ft_held_anchor_sample_node ratified: the anchor excludes every mutator
 * of the node only from the linearization point on, so a peer that changes the
 * word between the sample and that point must abort the whole acquire -- else
 * the op retires the node against a stale expected old.  The per-node acquire
 * needs none of this: ft_dlm_lock's own CAS is the guard, the word being the
 * same one it locks.
 */
static inline
void ft_held_anchor_guard_node(struct ft_flip_txn *t,
		struct cds_ft_metadata *node, uintptr_t node_snap)
{
	FT_TK_COUNT_REC(t, FT_TK_VALIDATE);
	urcu_txn_validate(t->mtxn, (void **) &node->state,
			(void *) node_snap, FT_STATE_PROXY);
}

/*
 * One member of a DLM lock-set: the node @nf to protect, sitting at byte-depth
 * @depth, plus an OPTIONAL read-set guard that @guard_child's back-edge still
 * resolves to @guard_pf (validating the racy plan read of @nf's position).
 *
 * @nf is the node, NOT the word to lock: coarsening sends the acquire to @nf's
 * ANCHOR, and only ft_dlm_acquire_set may make that mapping (§1 -- two sites
 * deriving it independently is how agreement is lost).  @held returns the
 * result, naming both words: the one the acquire CAS'd, for the release, and
 * @nf's own, for a retire.
 *
 * @nf == NULL skips the member -- an absent optional lock-set node, e.g. a root
 * with no parent, or a compressed parent a given shape does not have -- and
 * leaves @held untouched.
 */

/*
 * Is @node where its own back-edge says it is -- i.e. does the parent slot it
 * names actually hold it?  A node that answers false is not (yet, or no longer)
 * part of the tree: a copy still being built, or one a peer has already
 * replaced.  Locking such a node arbitrates nothing, because the writer that
 * matters -- its builder -- is not playing on that word.
 *
 * @node NULL (an external member carrying no metadata) answers true: the
 * question does not apply, and the acquire's other members still gate it.
 */
static inline
bool ft_dlm_member_linked(const struct cds_ft *ft,
		struct cds_ft_metadata *node)
{
	struct cds_ft_inode_flag *parent = NULL;
	struct cds_ft_inode_flag **slot;
	struct cds_ft_inode_flag *v;

	if (!node)
		return true;
	slot = ft_resolve_parent_slot(node, (struct cds_ft *) ft, &parent);
	if (!slot)
		return false;
	v = ft_resolve_flip_proxy((struct cds_ft_inode_flag *)
			CMM_LOAD_SHARED(*slot));
	if (!v)
		return false;
	/*
	 * COMPARE NODES, NOT WORDS.  A slot value is an ENCODING -- a type tag
	 * in the low bits, and for SKIP_X a run length in the high ones -- so
	 * masking the low tag off and comparing addresses answers "no" for
	 * every skip-compressed child that is perfectly well linked.  Measured
	 * as a permanent refusal loop (the same node refused over and over,
	 * ft_insert_dlm_acquire_split, unit test 2) before this resolved the
	 * value the way every other reader of a slot does.  Test skip FIRST: a
	 * SKIP_X flag carries its child's low tag bits, so ft_node_external()
	 * would misclassify it.
	 */
	if (ft_node_external(v))
		return false;		/* an external head carries no metadata */
	return (ft_node_compressed(v) ?
			cds_ft_item_to_metadata((struct cds_ft_inode *)
				ft_compressed_node_ptr(v)) :
			cds_ft_item_to_metadata(ft_node_ptr(
				ft_resolve_skip_compressed(ft, v)))) == node;
}


/*
 * THE ACQUIRE CHOKE POINT, transacted flavour: take a whole lock-set in ONE
 * all-or-none MCAS on a DEDICATED acquire flip-txn (never the content lane --
 * the escalation model's circular-wait constraint).  Each present member is
 * resolved to its anchor, deduped, guarded and locked; then the set commits
 * once.
 *
 * On commit OK every DISTINCT anchor in the set holds LOCK and every guard
 * validated at the linearization point; the caller registers each held word in
 * its CONTENT txn and records the release/retire terminal from
 * @set[i].held, exactly as ft_insert_dlm_acquire_split does.  Members that
 * deduped onto an anchor the set (or the op) already took come back
 * @held.shared: protected, but owing no release and no terminal.
 *
 * Returns 0 (whole set acquired), -EAGAIN (a member is held by a PEER or dirty,
 * a coarsened member's own word is dirty, or a guarded back-edge re-homed --
 * NOTHING acquired, abort-and-regrow), or -ENOMEM.  Deadlock-free: a conflict
 * aborts the commit, never blocks.
 *
 * @ctx supplies the anchors and the op's already-held words.  Passing NULL
 * while the op does hold something is the self-collision livelock
 * ft_held_set documents, not an optimisation.
 */
static inline
int ft_dlm_acquire_set_at(const char *fn, int line,
		const struct cds_ft *ft, const struct ft_lock_ctx *ctx,
		struct ft_dlm_member *set, int nr)
{
	struct cds_ft_metadata *taken[FT_FLIP_TXN_MAX_LOCKS];
	uintptr_t taken_snap[FT_FLIP_TXN_MAX_LOCKS];
	unsigned int nr_taken = 0;
	struct ft_flip_txn *acq;
	int i, nr_present = 0;

	for (i = 0; i < nr; i++)
		if (set[i].nf)
			nr_present++;
	if (!nr_present)
		return 0;
	if (ft_removeall_fault_refuse_acquire())
		return -EAGAIN;		/* test-only; nothing acquired */
	assert(nr_present <= FT_FLIP_TXN_MAX_LOCKS);
	/*
	 * Up to one back-edge guard + one lock + one coarsened-node guard per
	 * present member.  Dedupe only ever removes records, so this bound holds
	 * whatever the granularity (§7.3: the reservation stays safe, merely
	 * loose).
	 */
	/*
	 * The acquire txn is STANDALONE.  What a retry loop needs from a refused
	 * acquire is the CONTENTION AGE (see the eagain path), and that is
	 * separable from sharing the op's handle: binding to it would also share
	 * its descriptor and its install lane, which measured WORSE (median 9.5
	 * starving removes against 4 for aging alone, complete separation).
	 */
	acq = ft_flip_txn_acquire_bounded(3 * nr_present);
	if (!acq)
		return -ENOMEM;
	for (i = 0; i < nr; i++) {
		struct cds_ft_metadata *node, *lock;
		uintptr_t node_snap = 0, lock_snap, held_snap = 0;
		bool coarsened, deduped = false, held_ratified = true;
		bool node_held = false;

		if (!set[i].nf)
			continue;
		node = set[i].node;
#ifdef FT_ACQUIRE_LINK_GATE
		/*
		 * EXPERIMENT, OFF BY DEFAULT -- the predicate is not yet exact.
		 *
		 * STEP 1 IS ONLY AN ARBITRATION OVER NODES THAT ARE IN THE TREE.
		 *
		 * A node under construction has a CLEAN state word, so nothing
		 * below refuses a take on it -- and its builder holds no lock on
		 * it either (a copy is private by convention, not by exclusion).
		 * A plan that resolves a member to such a copy therefore acquires
		 * it, retires it, and the builder publishes it afterwards: the
		 * live parent slot then names a node whose grace period has
		 * already run (project_ft_retire_still_linked_forward_edge).
		 * Measured: EVERY dangling link came through a take on a member
		 * whose own back-edge did not name it, all at ft_node_recompact's
		 * {C,P,GP} acquire, against 56.7M takes on linked members.
		 *
		 * So ask the member's OWN back-edge whether it is where it claims
		 * to be, and re-plan when it is not.  The value is resolved
		 * through a parked proxy first: a member whose slot a peer is
		 * mid-flip on IS in the tree, and refusing that would trade a
		 * correctness gap for a contention one.
		 *
		 * MEASURED, 900-run batches of inv_rekey_contended_mixed_writers:
		 * the FT_RETIRE_STILL_LINKED oracle drops from 25/3600 runs to
		 * 3/1800.  ☠ BUT ft_dlm_member_linked answers NO for shapes that
		 * are perfectly well linked -- a member whose resolved parent slot
		 * holds an EXTERNAL head, seen at ft_insert_dlm_acquire_split --
		 * and a false refusal is permanent: the plan re-derives the same
		 * member and is refused again.  test_urcu_ft_unit stalls at test 2
		 * with the gate on and passes 308/308 with it off.  So the
		 * DIRECTION is confirmed by the oracle and the PREDICATE is not
		 * finished: it needs to be exact for every node kind (external
		 * heads, duplicate chains, compressed runs) before it can gate a
		 * real acquire.
		 */
		if (!ft_dlm_member_linked(ft, node)) {
			goto eagain;
		}
#endif
		lock = ft_anchor_meta(ft, ft_lock_ctx_descent(ctx), set[i].nf,
			node, set[i].depth);
		coarsened = lock != node;
		/*
		 * A coarsened member's OWN word is not the one being CAS'd, so
		 * the acquire does not ratify it.  Sample it and validate the
		 * value into THIS commit: the anchor excludes @nf's mutators only
		 * from the linearization point on, and the caller retires @nf
		 * against this snapshot.  A dirty word is the same -EAGAIN the
		 * per-node acquire gives for free.
		 */
		if (coarsened) {
			if (ft_member_node_snap(ctx, node, &node_snap,
					&node_held))
				goto eagain;
			/*
			 * A word this op already HOLDS needs no such guard: the
			 * mark is the exclusion the guard approximates, it is
			 * already in force, and validating the CLEAN value against
			 * a word carrying the op's own LOCK aborts every attempt.
			 */
			if (!node_held)
				ft_held_anchor_guard_node(acq, node, node_snap);
		}
		/*
		 * The guard is a read-set validation of the MEMBER's own
		 * back-edge, so it names the node even where the lock went to an
		 * ancestor: anchoring moves the exclusion, not the edge being
		 * validated.  It rides the commit for EVERY member, deduped or
		 * not -- dedupe merges LOCKS, never guards (§7.3).
		 */
		if (set[i].guard_child)
			ft_dlm_guard_parent(acq, set[i].guard_child,
				set[i].guard_pf);
		/*
		 * Dedupe against the op's held set AND against this set's own
		 * earlier members, taking the CLEAN word from whichever holds it:
		 * an uncoarsened member's word IS @node's, and a deduped member
		 * has no acquire of its own to sample it (a fresh read would
		 * return the op's own LOCK).
		 */
		if (!ft_lock_ctx_holds(ctx, lock, &held_snap, &held_ratified)) {
			unsigned int k;

			for (k = 0; k < nr_taken; k++) {
				if (taken[k] != lock)
					continue;
				held_snap = taken_snap[k];
				deduped = true;
				break;
			}
		} else {
			deduped = true;
		}
		if (deduped) {
			if (!coarsened) {
				assert(held_ratified);	/* see ft_acquire_member */
				node_snap = held_snap;
				node_held = true;
			}
			set[i].held.lock = lock;
			set[i].held.lock_snap = 0;
			set[i].held.node_snap = node_snap;
			set[i].held.shared = true;
			set[i].held.node_held = node_held;
			set[i].held.txn_owned = false;
			continue;
		}
		if (ft_dlm_lock(acq, lock, &lock_snap)) {
			ft_hold_trace_refused(lock, fn, line);
			goto eagain;
		}
		taken_snap[nr_taken] = lock_snap;
		taken[nr_taken++] = lock;
		ft_held_anchor_set(&set[i].held, lock, lock_snap, node,
			node_snap);
		set[i].held.node_held = node_held;
	}
	if (ft_flip_txn_commit((struct cds_ft *) ft, acq) != URCU_TXN_STATUS_OK)
		return -EAGAIN;		/* commit freed @acq; nothing acquired */
	for (i = 0; i < (int) nr_taken; i++)
		ft_hold_trace_note(taken[i], fn, line);
	return 0;
eagain:
	ft_flip_txn_destroy(acq);
	/*
	 * RECORD the refusal for the op's retry loop; do not age here.  Aging at
	 * this level too would count one contention event twice, and unevenly:
	 * ft-insert.h reaches -EAGAIN through
	 *
	 *	if (!ft_lock_ctx_depth_of(...) || ft_acquire_member(...))
	 *
	 * whose first disjunct never enters an acquire and whose second does, so
	 * one statement would age by one or by two with nothing at the caller
	 * able to tell which.  Escalation rate must not depend on WHERE the
	 * conflict was detected.
	 */
	if (ctx && ctx->op)
		ft_acq_contended++;
	return -EAGAIN;			/* nothing acquired (all-or-none) */
}

#define ft_dlm_acquire_set(ft, ctx, set, nr)				\
	ft_dlm_acquire_set_at(__func__, __LINE__, (ft), (ctx), (set), (nr))

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
	FT_TK_COUNT_CELL_MW();		/* a cell edge: see ft_hlist_store_mw */
	urcu_txn_store_mw(txn, (void **) &pos->next, succ_expected, newp, URCU_TXN_TAG);
	FT_TK_COUNT_CELL_MW();
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
			rcu_dereference(meta->parent_word)));
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
	/*
	 * This edge's slot is a TRIE ROOT (&ft->root), so it records MW
	 * whatever the txn's structural_sw mode -- see ft_flip_txn_record_root.
	 * Set by the two whole-trie swap helpers (which know the shape
	 * statically) and carried in from ft_pub_rec, whose forward edge is a
	 * root exactly when the publishing descent had a NULL parent.  A
	 * zero-initialized edge is an ordinary in-node slot.
	 */
	bool root;
	/*
	 * The node whose DLM lock OWNS @slot (§8), for the record-time owner
	 * check (FT_OWNER_ASSERT_OWNED).  NULL -- the zero-initialized default
	 * -- means the producer names no owner, so the edge stays ineligible
	 * for a per-op SW park.  A CELL edge legitimately has none: no cell
	 * carries a node lock, which is why the ordered list is MW by design.
	 */
	struct cds_ft_metadata *owner;
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
 *
 * READERS are safe on the store alone, whatever the writer count: a single
 * pointer-width release store is atomic and self-resolving, so a later reader
 * loads either the old or the new pointer and never a proxy.
 *
 * WRITERS are safe because the slot is EXCLUDED, not because @old is checked.
 * The store discards @old, so it cannot detect a peer that changed the slot --
 * it would silently overwrite one.  What makes that sound is the per-node lock
 * every caller holds over this slot: multi-writer IS the DLM lock-sets, so
 * exclusion is what a peer is stopped by here, and the engine's expected-old is
 * the mechanism the MULTI-slot commit needs and this one does not.  A caller
 * that reaches here without the covering lock is therefore a lost update with
 * nothing to catch it -- the lock-set, not this function, is where that is
 * checked.
 *
 * The {slot, old, new} descriptor shape is retained at the call sites for
 * uniformity, so a slot can move to a multi-edge commit without reshaping its
 * producer.  The lone-edge publish helpers (ft_root_edge_flip,
 * ft_chain_next_flip, the point insert/remove single-slot external_nodes
 * publishes) and ft_ord_cell_flip_try's n==1 fast path all commit through here;
 * the edge's tag is irrelevant (no proxy is installed).
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
		struct cds_ft_metadata **meta_out, uintptr_t *snap_out,
		struct urcu_txn *op)
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

		struct ft_held_anchor held;

		/*
		 * The root is its own anchor under every spacing, so this fence
		 * needs no descent -- but it still goes through the choke point,
		 * for the dedupe and so the machine check stays total.
		 */
		/*
		 * @op reaches the acquire through a ctx built here: the fence
		 * takes ONE member and needs no descent, so it had been passing
		 * a literal NULL ctx -- which also meant the dst ROOT metadata,
		 * the most contended word under root-only spacing, could never
		 * age its op on a refusal.
		 */
		struct ft_lock_ctx fctx;

		ft_lock_ctx_init(&fctx, NULL, NULL, op);
		if (ft_acquire_member(dst_ft, &fctx, root, rmeta, 0, &held))
			return -EAGAIN;
		snap = held.lock_snap;
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

	/*
	 * Every Class-G root swap passes its own trie's root slot, which is
	 * what makes the edge below a root edge unconditionally -- asserted
	 * rather than assumed, since the flag is what keeps it MW.
	 */
	assert(struct_slot == &ft->root);
	edges[n].slot = (struct ft_ord_cell **) struct_slot;
	edges[n].old_target = (struct ft_ord_cell *) struct_old;
	edges[n].new_target = (struct ft_ord_cell *) struct_new;
	edges[n].root = true;		/* a root records MW: no node owns it */
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
 * {slot, old, new} descriptor edge so the root slot keeps the same producer
 * shape as every other publish: a root slot can be in a concurrent writer's
 * word-set (e.g. a near-root insert that recompacts and republishes the root),
 * and the caller's lock-set is what excludes that peer.  @old rides along
 * unconsumed here (see ft_ord_cell_flip_one) and becomes the expected-old
 * without reshaping this producer if the slot ever joins a multi-edge commit.
 * This is the ordered-list-OFF arm of every Class-G root swap (detach / graft /
 * graft_swap / merge), where there is no head/tail endpoint to fuse and
 * ft_root_list_swap_publish would reduce to this anyway.
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

	ft_flip_txn_record_state(t, meta,
			(void *) old, (void *) (old | FT_STATE_TOMBSTONE));
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
	ft_flip_txn_record_state(t, meta,
			(void *) live, (void *) (live + FT_STATE_NR_CHILD_ONE));
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
void ft_flip_txn_record_tombstone_locked_ctx(struct ft_flip_txn *t,
		const struct ft_lock_ctx *dbg_ctx,
		struct cds_ft_metadata *meta, uintptr_t state_snapshot)
{
	ft_flip_txn_record_state_kind_ctx(t, dbg_ctx, meta,
			(void *) (state_snapshot | FT_STATE_LOCK),
			(void *) (state_snapshot | FT_STATE_TOMBSTONE), 1);
}

static inline
void ft_flip_txn_record_tombstone_locked(struct ft_flip_txn *t,
		struct cds_ft_metadata *meta, uintptr_t state_snapshot)
{
	ft_flip_txn_record_tombstone_locked_ctx(t, /*dbg_ctx=*/ NULL, meta,
		state_snapshot);
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
	/* The commit owns this release now; the op no longer owes one. */
	ft_hold_trace_drop(meta);
	ft_flip_txn_record_state(t, meta,
			(void *) (state_snapshot | FT_STATE_LOCK),
			(void *) state_snapshot);
}

/*
 * Drop LOCK from a word the op HOLDS, chaining onto whatever THIS txn has
 * already written to it -- the release twin of
 * ft_flip_txn_record_retire_anchored's fused arm, and for the same reason.
 *
 * ft_flip_txn_record_anchor_release carries the mark's acquire-time CLEAN
 * snapshot as its expected old, which makes the record a read-set guard on the
 * anchor.  That is right when the anchor is a word the op does not otherwise
 * touch (a graft's publish parent, acquired late).  It is UNUSABLE for an
 * anchor the op keeps WRITING: under a coarse spacing the anchor is an
 * ancestor's state word -- under root-only, the trie root's -- and the op
 * records its own counts, re-parents and terminals there.  A fixed expected old
 * then matches at NO point in the chain: recorded first, the op's later edges
 * expect the committed value while pending has already moved; recorded last,
 * the earlier edges have moved pending off the snapshot.  Either way every
 * commit aborts and the retry rebuilds the same shape -- a livelock with no
 * diagnostic (measured: single-writer, so not contention).
 *
 * The RYW load is what the retire twin already does, and its justification
 * carries over exactly: no peer can have moved this word, BECAUSE WE HOLD IT
 * LOCKED.  So the fresh read gives up nothing -- the LOCK is the exclusion the
 * snapshot form was approximating, and it is strictly stronger.
 *
 * Self-guarding, which is what makes it safe to call for EVERY non-shared mark:
 * a TOMBSTONE in pending means the op retires the anchor itself, and a pending
 * value with LOCK already clear means a node terminal (a retire's fused
 * transition, a re-parent's {live_state -> live_state}) has settled it.  Both
 * return without recording, so an UNcoarsened mark -- whose node terminal IS
 * its release -- costs nothing here.
 *
 * ORDER-INDEPENDENT, which is the point of the RYW load: recorded before the
 * op's other edges on the word it drops LOCK and they chain onto that; recorded
 * after them it sees their pending value and chains onto it.  Callers place it
 * by COVERAGE (past the last acquire), not by order.
 */
static inline
void ft_flip_txn_record_anchor_release_held(struct ft_flip_txn *t,
		struct cds_ft_metadata *lock)
{
	uintptr_t pending = (uintptr_t) urcu_txn_load(t->mtxn,
			(void **) &lock->state, FT_STATE_PROXY);

	/*
	 * The ledger tracks who still OWES a release, and past this point the op
	 * owes none: all three arms below hand the release to the COMMIT (a
	 * recorded {LOCK|s -> s}, a fused retire terminal, or a node terminal
	 * that already settled it), and none of them goes through
	 * ft_meta_lock_release, which is where the ledger is otherwise dropped.
	 *
	 * ☠ WITHOUT THIS THE LEDGER POISONS ITSELF.  A successful commit sets
	 * marks_consumed, which suppresses the caller's sweep -- the only other
	 * dropper -- so the entry survives the op that is done with it.  The next
	 * op to refuse that word for ORDINARY PEER CONTENTION then finds the
	 * stale entry and reports a SELF-COLLISION, fatally, against a lock the
	 * peer legitimately holds.  That is a detector fault reported as a code
	 * fault, and it needs a peer to show up at all: measured, every spacing
	 * aborts under FT_INV_MW with the ledger armed and passes 111/111 with it
	 * compiled out.
	 *
	 * Dropping here cannot lose a real release: if the commit aborts, the
	 * caller's sweep still runs ft_meta_lock_release_if_held over the marks,
	 * which clears the word regardless of the ledger.
	 */
	ft_hold_trace_drop(lock);
	if (caa_unlikely(pending & FT_STATE_TOMBSTONE))
		return;			/* the op retires the anchor itself */
	if (!(pending & FT_STATE_LOCK))
		return;			/* a node terminal already settled it */
	ft_flip_txn_record_state(t, lock,
			(void *) pending,
			(void *) (pending & ~FT_STATE_LOCK));
}

/*
 * A retire terminal whose lock-set member was ANCHORED has TWO halves on TWO
 * words: the lock sits on an ancestor that SURVIVES the commit, while the
 * retired node's own word was never locked.  Per-node granularity puts both on
 * one word, which is what the fused {LOCK|s -> TOMBSTONE|s} transition above
 * expresses in a single record.  Recording that fused form across two words
 * instead would tombstone a LIVE ancestor, and would carry the ancestor's word
 * as the node's expected old -- unrelated values, so the commit could only ever
 * abort.  The two halves are recorded at DIFFERENT times, hence two functions.
 *
 * THE RELEASE half, recorded when the op REGISTERS the lock rather than at the
 * commit.  The surviving ancestor can also be the publish target's own word,
 * and a §4.B guard planted on a word BEFORE its release POISONS the txn
 * permanently (the ordering rule on ft_flip_txn_record_release_lock).
 * Recording the release first makes the later guard the harmless no-op that
 * rule describes.  A no-op when the lock IS the retired node's word: the fused
 * terminal covers it, unchanged.
 *
 * ★ ONE WORD TAKES ONE TERMINAL, AND A RETIRE OUTRANKS A RELEASE.  Coarsening
 * lets a member anchor on a node the SAME op retires: this member wants the
 * anchor to survive, that one kills it, and a node's fate belongs to the op's
 * PLAN, not to the order its members were acquired.  Recording both poisons the
 * txn either way round -- {LOCK|s -> s} and {LOCK|s -> TOMBSTONE|s} carry the
 * same expected old, so whichever lands second mismatches the first's pending
 * new and record_chain sets t->poisoned, permanently (the ordering rule on
 * ft_flip_txn_record_release_lock).  So the release YIELDS: if this txn's own
 * pending view of the word already shows TOMBSTONE, the op has retired the
 * anchor and that record is the terminal.  The other order needs nothing here
 * -- the later retire chains onto the release's clean pending value.
 *
 * The read is READ-YOUR-OWN-WRITES for the reason spelled out on
 * ft_flip_txn_record_tombstone: the word is in this record's own write set, so
 * the engine's read policy requires it be read through the txn.  A TOMBSTONE
 * seen here can only be OURS -- a peer's would have failed the acquire that
 * gave us @h.
 *
 * Costs ONE MORE reserved edge than the fused form; @t must have budgeted it.
 */
static inline
void ft_flip_txn_record_anchor_release(struct ft_flip_txn *t,
		const struct ft_held_anchor *h, const struct cds_ft_metadata *node)
{
	uintptr_t pending;

	/*
	 * A member that deduped onto a word the op already holds owes no
	 * release: the acquire that first took the word recorded one, and a
	 * second record settles the single word twice.  Callers gate on
	 * @h->shared; asserting keeps that from being forgotten silently.
	 */
	assert(!h->shared);
	if (h->lock == node)
		return;
	pending = (uintptr_t) urcu_txn_load(t->mtxn, (void **) &h->lock->state,
			FT_STATE_PROXY);
	if (caa_unlikely(pending & FT_STATE_TOMBSTONE))
		return;			/* the op retires the anchor itself */
	ft_flip_txn_record_release_lock(t, h->lock, h->lock_snap);
}

/*
 * THE RETIRE half: tombstone @node.  Fused with the release while they share a
 * word, plain once ft_flip_txn_record_anchor_release has lifted the lock off a
 * surviving ancestor.
 *
 * The expected old is a snapshot taken at the ACQUIRE, not a fresh raw read of
 * the word, which is what keeps the fenced contract: a peer state change on the
 * retired node between the acquire and the commit aborts this op instead of
 * being ratified by a coincidentally-matching late capture.  The one value the
 * snapshot is allowed to miss is a LOCK THIS OP took on @node's own word after
 * the member was acquired -- see the arm that reads it back through the txn.
 *
 * @ctx is the op's LEDGER, and that arm is the only thing it is read for:
 * whether the op holds @node's own word is a fact about the OP, and the word
 * cannot be asked (an owner-less LOCK bit does not say whose it is).  NULL is
 * the conservative answer -- "not known to be held" -- which keeps the
 * snapshot, so a caller with no ledger loses at most an abort it could have
 * avoided.
 */
static inline
void ft_flip_txn_record_retire_anchored_arms(struct ft_flip_txn *t,
		const struct ft_lock_ctx *ctx,
		const struct ft_held_anchor *h, struct cds_ft_metadata *node)
{
	/*
	 * Every arm below hands @node's LOCK to the commit -- fused into the
	 * tombstone where the op holds @node's own word, and lifted by
	 * ft_flip_txn_record_anchor_release beforehand where it does not.  Either
	 * way the op stops owing a release on it, so the ledger must forget it;
	 * see ft_flip_txn_record_anchor_release_held for what a surviving entry
	 * does to the next op that refuses this word.  A no-op when @node was
	 * never in the ledger (the anchored case, where its own word is unlocked).
	 */
	if (h->lock == node || h->node_held) {
		/*
		 * The op holds @node's OWN word, whether because the member
		 * anchored on itself or because an EARLIER member of this op did.
		 * Either way the retire is the FUSED transition -- drop LOCK, set
		 * TOMBSTONE -- never a clean-word expected old, which would name a
		 * value the word has not carried since that mark landed.
		 */
		if (caa_unlikely(h->shared || h->node_held)) {
			/*
			 * The op already held the very node it now retires: an
			 * earlier member ANCHORED on @node, and that acquire kept
			 * the clean snapshot this retire would want.  There is no
			 * snapshot in @h to use -- a deduped member carries none --
			 * so take the word's value from THIS txn (read-your-own-
			 * writes), which is exact for both shapes the collapse
			 * produces: an earlier release on @node returns its clean
			 * pending value and the retire chains onto it, and no
			 * earlier record returns the committed word, which no peer
			 * can have moved because we hold it LOCKED.
			 *
			 * The transition is the fused one either way -- drop LOCK,
			 * set TOMBSTONE -- so the mask is what expresses it, and it
			 * is a no-op where a release already cleared the bit.
			 */
			uintptr_t pending = (uintptr_t) urcu_txn_load(t->mtxn,
					(void **) &node->state, FT_STATE_PROXY);

			ft_flip_txn_record_state_ctx(t, ctx, node,
				(void *) pending,
				(void *) ((pending & ~(uintptr_t) FT_STATE_LOCK)
					| FT_STATE_TOMBSTONE));
			return;
		}
		ft_flip_txn_record_tombstone_locked_ctx(t, ctx, node,
			h->lock_snap);
		return;
	}
	/*
	 * @node's own word is not one this member locked, so the snapshot is the
	 * expected old -- UNLESS the op has marked that word since, which under a
	 * coarse spacing is ordinary: a LATER member anchoring ON @node locks it,
	 * and the two acquires are separate commits, so nothing updated @h.  The
	 * snapshot then names a value the word has not carried since that mark
	 * landed, the install CAS can only fail, and every retry rebuilds the same
	 * shape -- a deterministic abort with no diagnostic (measured
	 * single-writer, so not contention).
	 *
	 * So take the FUSED transition when the txn's own view differs from the
	 * snapshot BY THE LOCK BIT ALONE -- but ONLY once @ctx has confirmed the
	 * op holds @node's own word.
	 *
	 * ☠ The bit alone does NOT identify its owner.  Holding the ANCHOR does
	 * not exclude every mutator of @node either: a peer whose descent dates
	 * @node differently -- across a graft, a merge, or any move that puts the
	 * node on a second path -- anchors it elsewhere and locks it legitimately.
	 * Swallowing that bit into the expected old turns the retire into a
	 * COMMITTING write over a peer's fence: the tombstone lands, the peer's
	 * lock silently vanishes with it, and the peer's own release then asserts
	 * on a word no writer is accountable for.
	 *
	 * Any OTHER difference stays on the snapshot, which is what makes the
	 * commit abort rather than ratify a world that moved -- the fenced
	 * contract this arm exists for.  Per-node granularity never reaches here
	 * (@h->lock is always @node), so the default stays byte-identical.
	 */
	{
		uintptr_t pending = (uintptr_t) urcu_txn_load(t->mtxn,
				(void **) &node->state, FT_STATE_PROXY);
		uintptr_t held_snap;
		bool ratified;

		if (caa_unlikely(pending == (h->node_snap | FT_STATE_LOCK) &&
				ft_lock_ctx_holds(ctx, node, &held_snap,
					&ratified))) {
			ft_flip_txn_record_state_ctx(t, ctx, node,
				(void *) pending,
				(void *) ((pending & ~(uintptr_t) FT_STATE_LOCK)
					| FT_STATE_TOMBSTONE));
			return;
		}
	}
	ft_flip_txn_record_state_ctx(t, ctx, node,
			(void *) h->node_snap,
			(void *) (h->node_snap | FT_STATE_TOMBSTONE));
}

/*
 * ☠ THE LEDGER DROP HAPPENS AFTER THE ARMS, NOT BEFORE THEM.
 *
 * Every arm above hands @node's LOCK to the commit, so the ledger must forget
 * it -- but only once the records that arm plants have been made.  Those
 * records ask, at the moment they are planted, whether the op OWNS the word
 * (ft_flip_txn_owns, and the OWN_LEDGER counter beside it); dropping first
 * answers "no" to a question whose true answer this branch has already
 * established, and the whole class then reads as an exclusion gap instead of
 * the registry gap it is.  Measured: 2,200,222 of 14,907,298 anchored retires
 * under the MW oracle hold @node's own word with the registry silent, and the
 * ledger is the only witness left.
 *
 * A no-op when @node was never in the ledger (the anchored case, where its own
 * word is unlocked), and compiled out entirely without FEATURE_FT_HOLD_TRACE.
 */
static inline
void ft_flip_txn_record_retire_anchored(struct ft_flip_txn *t,
		const struct ft_lock_ctx *ctx,
		const struct ft_held_anchor *h, struct cds_ft_metadata *node)
{
	ft_flip_txn_record_retire_anchored_arms(t, ctx, h, node);
	ft_hold_trace_drop(node);
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
 *
 * COARSENING keeps that net-zero, and needs no guard beside the release.  The
 * release then lands on an ANCESTOR rather than on @parent_nf's own word, so it
 * stops validating that word.  What stands in for it is §1's AGREEMENT
 * property: an op that could retire @parent_nf resolves it to the SAME anchor
 * and so contends for the word this op now holds, making the anchor the
 * strictly stronger representative the per-node release record was.
 *
 * ☠ That is a property of the DATING, not of this site.  A member whose
 * byte-depth the caller could not derive, passed as the 0 an initializer left
 * behind, reads as THE ROOT (ft_anchor_meta) and anchors on ITSELF -- and then
 * the retiring op and this one hold two different words and exclude nothing.
 * Such a member is FT_DEPTH_FROM_DESCENT, refused below.
 *
 * What coarsening does lose is the acquire's refusal of an ALREADY-dirty target
 * (a node a previous holder of the anchor retired), and ft_acquire_member
 * restores exactly that by sampling @parent_nf's own word.
 *
 * ALREADY HELD: a coarser spacing maps several lock-set members onto ONE word,
 * so an op can arrive here holding this publish target's lock already, taken for
 * a DIFFERENT member of its own set.  Re-acquiring then misses against the op's
 * OWN hold, which sets @acquire_miss and aborts the commit, and the caller
 * retries into the identical shape: the op waits on itself, forever.  The choke
 * point dedupes against @ctx's held set instead and leaves only the guard owed.
 */
static inline
void ft_flip_txn_lock_or_guard_parent_at(const char *fn, int line,
		const struct cds_ft *ft,
		struct ft_flip_txn *t, const struct ft_lock_ctx *ctx,
		struct cds_ft_inode_flag *parent_nf, unsigned int parent_depth)
{
	(void) fn; (void) line;
	if (ft->lock_fine && t && parent_nf) {
		/*
		 * @t is the registry this record joins, so it is authoritative
		 * for the dedupe; @ctx contributes the anchor source and any
		 * marks the op keeps outside a txn.
		 */
		struct ft_lock_ctx lctx = {
			.d = ft_lock_ctx_descent(ctx),
			/*
			 * CARRY THE OP.  Omitting it here zero-initialises it,
			 * so a @ctx that arrived ENROLLED in the escalation
			 * domain was laundered into an unenrolled one and the
			 * acquire below could no longer age it -- measured
			 * 1,346,610 domain-less acquires per ft_inv run at this
			 * one site, all from callers that had a handle.
			 */
			.op = ctx ? ctx->op : NULL,
			.held = { .txn = t,
				.extra = ctx ? ctx->held.extra : NULL,
				.nr_extra = ctx ? ctx->held.nr_extra : 0,
				.glue = ctx ? ctx->held.glue : NULL },
		};
		struct ft_held_anchor held;

		if (parent_depth == FT_DEPTH_FROM_DESCENT &&
				!ft_lock_ctx_depth_of(ft, ctx, parent_nf,
					&parent_depth)) {
#ifdef FEATURE_FT_HOLD_TRACE
			if (ft_hold_trace_report_ok())
				fprintf(stderr, "  ...from %s:%d\n", fn, line);
#endif
			t->acquire_miss = true;
			goto guard;
		}
		int aret = ft_acquire_member(ft, &lctx, parent_nf,
				ft_flag_to_metadata(ft, parent_nf),
				parent_depth, &held);

		if (caa_likely(!aret)) {
			/*
			 * The terminal for a word the op already held was
			 * recorded when it was acquired, so all that is owed
			 * here is the guard -- and a guard AFTER a release on
			 * one word is the ordering rule's harmless no-op (it
			 * reads the record's own pending clean value and
			 * validates {s -> s}), never the poison order.
			 *
			 * Except on the word the op HOLDS.  The guard's
			 * expectation is clean-LIVE, which masks out the very
			 * FT_STATE_LOCK this op set, so it names a value the
			 * word has not carried since the mark landed -- and
			 * the member that took the word records its terminal
			 * LATER in the commit, which is the poison order the
			 * ordering rule above forbids.  Per-node granularity
			 * cannot reach it (the holder is @parent_nf's own
			 * metadata and its release is already recorded, so the
			 * RYW value really is clean); coarsening splits the
			 * two apart, and then the op's own mark is what the
			 * guard would be validating against.  The mark is the
			 * stronger statement anyway -- it is the exclusion the
			 * guard approximates, already in force -- so the word
			 * it protects owes nothing here.
			 */
			if (held.shared) {
				if (held.lock != ft_flag_to_metadata(ft, parent_nf)
						&& !held.node_held)
					ft_flip_txn_guard_parent(ft, t,
						parent_nf);
				return;
			}
			/*
			 * REGISTER BEFORE RECORDING.  The op holds @held.lock --
			 * that is why it is releasing it -- so the registry must
			 * say so at the moment the release edge is planted, not
			 * one line later: the record asks ft_flip_txn_owns who
			 * owns the word it writes, and the answer has to be
			 * already true.  Recording first made the whole acquire
			 * lane report an exclusion gap it does not have.
			 * ft_flip_txn_record_release_lock reads no registry, so
			 * the order is free.
			 */
			ft_flip_txn_lock_register(t, held.lock, held.lock_snap);
			ft_flip_txn_record_release_lock(t, held.lock,
				held.lock_snap);
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
		/*
		 * -ENOMEM is not a peer.  Distinguish it so the commit reports
		 * MEMORY_ERROR instead of ABORT and does NOT age the handle:
		 * aging for an allocation failure spends the op's escalation
		 * budget on a conflict that never happened.
		 */
		if (aret == -ENOMEM)
			t->acquire_enomem = true;
	}
guard:
	ft_flip_txn_guard_parent(ft, t, parent_nf);
}

#define ft_flip_txn_lock_or_guard_parent(ft, t, ctx, parent_nf, parent_depth)	\
	ft_flip_txn_lock_or_guard_parent_at(__func__, __LINE__, (ft), (t),	\
		(ctx), (parent_nf), (parent_depth))

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
void ft_flip_txn_hold_or_lock_parent_at(const char *fn, int line,
		const struct cds_ft *ft,
		struct ft_flip_txn *t, const struct ft_lock_ctx *ctx,
		struct cds_ft_inode_flag *parent_nf, unsigned int parent_depth,
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
		 *
		 * Coarsening makes the held word @parent_nf's ANCHOR rather than
		 * its own metadata, so the identity is exact only at per-node
		 * granularity.  Otherwise it is checked at the acquire site, which
		 * is the only place holding both the member and its byte-depth.
		 */
		assert(parent_nf);
		assert(ft->lock_spacing != CDS_FT_LOCK_SPACING_PER_NODE ||
			held_holder == ft_flag_to_metadata(ft, parent_nf));
		/*
		 * ★ ONE WORD TAKES ONE TERMINAL, AND A RETIRE OUTRANKS A RELEASE
		 * -- the rule ft_flip_txn_record_anchor_release states, and the
		 * reason this arm uses the SELF-GUARDING form rather than the
		 * fixed-snapshot ft_flip_txn_record_release_lock.
		 *
		 * The publish parent's ANCHOR can be a word the SAME op retires:
		 * coarsening puts it on an ancestor, and root-only puts every
		 * member on one word.  Then {LOCK|s -> s} and
		 * {LOCK|s -> TOMBSTONE|s} carry the same expected old, whichever
		 * lands second mismatches the first's pending new, and
		 * record_chain POISONS the descriptor -- permanently, so every
		 * commit aborts and the retry rebuilds the identical shape.  A
		 * livelock with no contention, which is exactly how it presents
		 * (measured single-threaded at the rekey's retry cap).
		 *
		 * The RYW form yields: a pending TOMBSTONE means the op retires
		 * this anchor itself and that record is the terminal; a pending
		 * value with LOCK already clear means a node terminal settled
		 * it.  What it gives up is the acquire-time read-set guard the
		 * snapshot form doubled as -- and that guard was approximating
		 * an exclusion this arm already has, since we HOLD the word.
		 */
		ft_flip_txn_lock_register(t, held_holder, held_snap);
		ft_flip_txn_record_anchor_release_held(t, held_holder);
		return;
	}
	ft_flip_txn_lock_or_guard_parent_at(fn, line, ft, t, ctx, parent_nf,
			parent_depth);
}

#define ft_flip_txn_hold_or_lock_parent(ft, t, ctx, pnf, pd, hh, hs)	\
	ft_flip_txn_hold_or_lock_parent_at(__func__, __LINE__, (ft), (t),	\
		(ctx), (pnf), (pd), (hh), (hs))

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
		assert(r->slot == &r->ft->root);
		edges[n].slot = (struct ft_ord_cell **) r->slot;
		edges[n].old_target = (struct ft_ord_cell *) r->old_root;
		edges[n].new_target = (struct ft_ord_cell *) r->new_root;
		/*
		 * BOTH sides' roots, though @txn names only ONE trie: the rule
		 * is a property of the slot, not of the named trie, so a dual
		 * arming off its own trie still records the foreign root MW.
		 * This is what makes the arming decision shape-independent.
		 */
		edges[n].root = true;
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
/*
 * The commit status as an errno, for the ops that speak errno.  The engine's
 * three-way status is SIGNED on purpose (MEMORY_ERROR < OK < ABORT), and a bare
 * "> 0 ? -EAGAIN : 0" therefore reads MEMORY_ERROR as SUCCESS -- publishing
 * nothing and reporting that it did.  That is unreachable while every commit
 * here runs on a pre-reserved txn, but an acquire that could not allocate now
 * surfaces exactly here (ft_flip_txn_commit's @acquire_enomem arm), so the
 * distinction has to be carried rather than assumed away.
 */
static inline
int ft_flip_status_to_errno(enum urcu_txn_status st)
{
	if (st > 0)
		return -EAGAIN;		/* ABORT: a peer won; nothing installed */
	if (st < 0)
		return -ENOMEM;		/* nothing installed either */
	return 0;
}

static
enum urcu_txn_status ft_ord_cell_flip_into(struct cds_ft *ft,
		struct ft_flip_txn *t,
		struct ft_ord_cell_edge *edges, unsigned int n)
{
	unsigned int i;

	for (i = 0; i < n; i++) {
		uintptr_t tag = ft_edge_tag(&edges[i]);

		/*
		 * PER-EDGE KIND, exactly as in the record-only sibling
		 * ft_ord_cell_record_into_ft: a trie ROOT and an ORDERED-CELL /
		 * hlist edge (URCU_TXN_TAG) are both slots no lock-set owns, so
		 * they record MW whatever @t's structural_sw mode; only a
		 * STRUCTURAL trie edge takes the dispatch.  Recording a cell
		 * through the dispatching helper would park the ordered list
		 * under an armed txn -- the lane the conversion deliberately
		 * leaves MW, and the lane whose clean MW abort is what lets the
		 * mixed commit back out before any SW side effect.
		 *
		 * Unlike the sibling this path plants no §4.B installed-child
		 * guard; that difference is pre-existing and untouched here.
		 */
		if (edges[i].root)
			ft_flip_txn_record_root(t, (void **) edges[i].slot,
				(void *) edges[i].old_target,
				(void *) edges[i].new_target);
		else if (tag == FT_FLIP_PROXY_TAG)
			ft_flip_txn_record_tag(t, edges[i].owner,
				(void **) edges[i].slot,
				(void *) edges[i].old_target,
				(void *) edges[i].new_target, tag);
		else
			ft_flip_txn_record_tag_mw(t, (void **) edges[i].slot,
				(void *) edges[i].old_target,
				(void *) edges[i].new_target, tag);
	}
	return ft_flip_txn_commit(ft, t);
}

/*
 * §4.B VALIDATE FOR A FORWARD PUBLISH WHOSE VALUE IS AN EXISTING NODE.
 *
 * A structural edge's own arbitration covers the SLOT, never the VALUE, and the
 * DLM acquire takes {C,P,(GP)} and never C's children -- so a lock can NEVER
 * cover the node a forward edge installs.  Without a guard, a peer that retires
 * that node between this op building its write set and its commit landing loses
 * the race silently: the commit republishes a node whose grace period is already
 * running, and once that period ends the slot names arena memory.  Measured as
 * the dangling parent slot in [[project_ft_retire_still_linked_forward_edge]] --
 * every occurrence of it came through here.
 *
 * The guard is the same {live_state -> live_state} edge the re-parent sweep
 * records (ft_reparent_record_meta), for the same reason and with the same three
 * rules:
 *
 *  - WAITING load, not a raw one: a raw read bakes a peer's parked
 *    FT_STATE_PROXY into the expected-old, and a validate that matches it
 *    publishes that pointer back into the live word, latching the node forever.
 *  - MASK TOMBSTONE and LOCK out of the expected value, so a node a peer FROZE
 *    or LOCKED MISMATCHES and aborts this commit.
 *  - MW kind, unconditionally: an SW park validates nothing (it cannot fail), so
 *    recording this through the structural_sw-dispatching helper would be a
 *    guard in name only.
 *
 * SKIPPED when the op HOLDS the node: its own terminal already governs that
 * word, and an MW edge expecting live_state would mismatch the op's OWN lock and
 * abort every commit -- the mirror of ft_reparent_record_meta's @child_marked
 * arm.
 */
static inline
void ft_flip_txn_guard_installed_child(struct cds_ft *ft, struct ft_flip_txn *t,
		struct cds_ft_inode_flag *nf)
{
	struct cds_ft_metadata *meta;
	uintptr_t old_state, live_state;

	/*
	 * NULL clears a slot, a parked proxy is a value in flight rather than a
	 * node, and an external head carries no state word to validate.
	 */
	if (!nf || ft_node_flip_proxy(nf) || ft_node_external(nf))
		return;
	meta = ft_node_compressed(nf) ?
		cds_ft_item_to_metadata((struct cds_ft_inode *)
			ft_compressed_node_ptr(nf)) :
		cds_ft_item_to_metadata(ft_node_ptr(
			ft_resolve_skip_compressed(ft, nf)));
	if (ft_flip_txn_holds(t, meta)) {
		return;
	}
	if (!ft_flip_txn_reserve_extra(t, 1)) {
		return;
	}
	old_state = (uintptr_t) urcu_txn_load(t->mtxn,
		(void **) &meta->state, FT_STATE_PROXY);
	live_state = old_state & ~(FT_STATE_TOMBSTONE | FT_STATE_LOCK);
	ft_flip_txn_record_state_mw(t, meta,
		(void *) live_state, (void *) live_state);
}

static inline
void ft_ord_cell_record_into_ft(struct cds_ft *ft, struct ft_flip_txn *t,
		const struct ft_ord_cell_edge *edges, unsigned int n)
{
	unsigned int i;

	for (i = 0; i < n; i++) {
		uintptr_t tag = ft_edge_tag(&edges[i]);

		if (tag == FT_FLIP_PROXY_TAG) {
			if (ft)
				ft_flip_txn_guard_installed_child(ft, t,
					(struct cds_ft_inode_flag *)
						edges[i].new_target);
			/*
			 * A root edge takes the SAME §4.B guard -- what the
			 * guard covers is the VALUE (a node a peer may have
			 * retired), which a root publish installs like any
			 * other.  Only the record KIND differs.
			 */
			if (edges[i].root)
				ft_flip_txn_record_root(t,
					(void **) edges[i].slot,
					(void *) edges[i].old_target,
					(void *) edges[i].new_target);
			else
				ft_flip_txn_record_tag(t, edges[i].owner,
					(void **) edges[i].slot,
					(void *) edges[i].old_target,
					(void *) edges[i].new_target, tag);
		} else
			ft_flip_txn_record_tag_mw(t, (void **) edges[i].slot,
				(void *) edges[i].old_target,
				(void *) edges[i].new_target, tag);
	}
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
	ft_ord_cell_record_into_ft(NULL, t, edges, n);
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
	t = ft_flip_txn_create_bounded(ft, n);
	if (caa_unlikely(!t))
		return -ENOMEM;
	return ft_flip_status_to_errno(ft_ord_cell_flip_into(ft, t, edges, n));
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
		ft_flip_txn_create_bounded(ft, FT_ORD_CELL_SWAP_REC_MAX_EDGES);

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
		return ft_flip_status_to_errno(
			ft_ord_cell_flip_into(ft, txn, edges, n));
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
		sedges[i].root = rec->root[i];
		sedges[i].owner = rec->owner[i];
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
			ft_ord_cell_record_into_ft(ft, txn, edges, n);
			return 0;
		}
		int cret = ft_flip_status_to_errno(
			ft_ord_cell_flip_into(ft, txn, edges, n));

		if (cret)
			return cret;	/* nothing installed: -EAGAIN peer won and the
					 * caller retries; -ENOMEM an acquire could
					 * not allocate and retrying cannot help */
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
 * Record a LIVE child's parent back-edge into @txn so it rides the SAME flip as
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
		struct cds_ft_inode_flag **slot, bool child_marked,
		const struct ft_lock_ctx *hold_ctx);

static
void ft_record_child_back_edge(struct cds_ft *ft, struct ft_flip_txn *txn,
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
			/*child_marked=*/ false, /*hold_ctx=*/ NULL);
		return;
	} else if (ft->ordered_list) {
		field = &ft_ord_cell_ptr(
			((struct cds_ft_node *) child)->prev)->parent;
	} else {
		field = (struct cds_ft_inode_flag **)
			&((struct cds_ft_node *) child)->prev;
	}
	/*
	 * Back-edge expected-old = the child's current back-pointer
	 * (cell->parent / prev), read as a WAITING load rather than raw: this
	 * slot enters @txn's write set on the very next line, so its last load
	 * must wait out an undecided parker.  A raw read bakes a peer's parked
	 * flip proxy -- a descriptor-record POINTER -- into the expected-old,
	 * which is the engine's !urcu_txn_is_proxy self-check.
	 */
	/*
	 * An EXTERNAL head's back channel goes STRAIGHT into @txn, always-MW,
	 * exactly as the metadata arm above goes straight into it: routing it
	 * through @rec instead put an ownerless word on the rec's SW-capable
	 * replay, where an armed op would park a word it cannot hold.  Same
	 * commit, same abort, same reservation -- the rec never added anything
	 * to this edge, having no per-edge answer that means "always MW".
	 */
	ft_flip_txn_record_head_back_edge(txn, (void **) field,
		urcu_txn_load(txn->mtxn, (void **) field, FT_FLIP_PROXY_TAG),
		new_parent);
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
 * child's parent-slot offset via ft_record_child_back_edge) reaches an
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
		edges[n].root = rec->root[i];
		edges[n].owner = rec->owner[i];
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
		ft_ord_cell_record_into_ft(ft, txn, edges, n);
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
		edges[n].root = rec->root[i];
		edges[n].owner = rec->owner[i];
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

/*
 * ★ NOT PART OF THE MERGE FEATURE: a generic ordered-run unlink, called by the
 * rekey paths that survive -DNO_FEATURE_FT_MERGE.
 */
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

#ifdef FEATURE_FT_MERGE
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
	 * must wait (applied at apply_deferred_dst).
	 *
	 * ☠ THE NAME IS NARROWER THAN THE FLAG.  It reads "still reachable when
	 * this commit runs, so RIDE the txn", and the graft paths set it for their
	 * own live children too -- a displaced suffix child (ft-graft.h) and the
	 * displaced external of a store-at-graft-point, which is dst-side but is
	 * reached through a graft.  Only a child whose source is already unlinked
	 * and drained leaves it false and takes the plain store.  @live is the
	 * property actually consumed; this is one of its two sources.
	 */
	bool dst_origin;
	/*
	 * @child is READER-REACHABLE until the forward flip, so its back-pointer
	 * must RIDE the commit: ft_glue_apply_deferred RECORDS this edge instead
	 * of storing it, and the flip is what publishes it.  Two shapes carry it
	 * -- a @dst_origin child, still on the old dst spine; and a FOLD child,
	 * whose src spine is unlinked by this very commit and so stays reachable
	 * for the whole build window.  Every other src-origin child is HIDDEN --
	 * a drained payload or a fresh cluster -- and takes the plain store, in
	 * recorded order.
	 *
	 * ☠ REACHABILITY IS A PROPERTY OF THE EDGE, NOT OF THE TXN.  Deriving the
	 * store-vs-record choice from @g->txn->structural_sw instead reads the
	 * txn's record KIND as if it answered reader visibility: every armed
	 * committer then records its HIDDEN edges too, and the built cluster's
	 * back-pointers are still unapplied when the forward publish runs --
	 * _ft_publish_to_parent_meta reads parent_word RAW and cannot see a
	 * merely recorded re-parent, and a SKIP_X top resolves its
	 * ft_set_parent_slot dual against the pre-loop parent.
	 *
	 * ☠ NOT A COMPLETE REACHABILITY ORACLE, and its one gap is where it
	 * already was: the graft_swap KEY_SHORTER wrap re-parents a LIVE dst
	 * child on a glue with no @txn to record into, so it stays @live false
	 * and keeps its immediate store.  ft-graft.h states that classification
	 * at the site that defers it.
	 */
	bool live;
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
	/*
	 * The word the acquire actually CAS'd for this child -- its ANCHOR
	 * under a coarse spacing, its own metadata under per-node.  Stored
	 * rather than re-derived, because the release sweep must lift the word
	 * this op TOOK, and a re-derivation names the node instead.  Doubles as
	 * the dedupe key: two deferred entries mapping to one anchor must be
	 * acquired ONCE.
	 */
	struct cds_ft_metadata *lock_word;
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
	/*
	 * The word the acquire actually LOCKED -- @node's ANCHOR -- with its own
	 * clean snapshot.  Under per-node granularity it IS @node's metadata and
	 * the terminal fuses to the single {LOCK|s -> TOMBSTONE|s}; coarsening
	 * splits the two, and then @node takes the TOMBSTONE while the anchor
	 * takes a RELEASE.  Recording the fenced terminal against the NODE alone
	 * names a LOCK the node does not carry: the install mismatches, the
	 * commit aborts, and the merge reports OK on top of it.
	 *
	 * @holder_shared: the op ALREADY held that word, so this entry owes no
	 * release -- the acquire that first took it recorded one.
	 */
	struct cds_ft_metadata *holder;
	uintptr_t holder_snap;
	bool holder_shared;
	/*
	 * The txn now OWNS @holder's outcome -- its {LOCK|s -> s} release is in
	 * the edge set AND the word is in locks[] -- so ft_glue_clear_fenced must
	 * stop sweeping it.  ONE OWNER PER FENCE (ft_held_anchor's @txn_owned,
	 * and the same argument): a SURVIVING anchor is clean and re-lockable the
	 * instant the release settles, and under a coarse spacing every op wants
	 * that same ancestor, so a later "release it if it is still held" reads
	 * back a PEER's fresh mark and strips it.
	 *
	 * ☞ An anchor that IS the retired node cannot be stolen that way -- the
	 * acquire refuses a TOMBSTONE, so a LOCK still set on that word can only
	 * be ours -- which makes the sweep SUFFICIENT there, not required.  It
	 * goes to the txn all the same, because the retire RECORD names that node
	 * as its owner and a record-time owner check has only locks[] to read.
	 */
	bool holder_txn_owned;
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
	/*
	 * The byte-depth of the node that HOLDS @dst_head, captured where the
	 * splice is recorded -- the merge frame that owns that node knows it.
	 * The descent cannot supply it: the holder is reached by walking a
	 * chain head's prev, and for a merge point at the root the descent has
	 * passed nothing at all, so ft_lock_ctx_depth_of fails and the acquire
	 * bails to a re-descend that must fail identically forever.
	 */
	unsigned int holder_depth;
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
	/*
	 * The enclosing op's persistent txn handle, so the lock contexts this
	 * glue builds can age it on a refused acquire (ft_dlm_acquire_set).
	 * NULL where the op has none.
	 */
	struct urcu_txn *op;
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
	/*
	 * Anchor source for this op's acquires: @publish_parent and the split
	 * CN are lock-set members, and a coarse spacing sends their acquires to
	 * an ancestor selected by BYTE-DEPTH.  Set by the builder that owns the
	 * descent; NULL where none ran, which is legal only under per-node
	 * granularity (doc/design/ft-dlm-lock-coarseness.md §2).
	 */
	const struct ft_descent *lock_d;
	/*
	 * Anchor source for the op's SRC-ORIGIN members, where "src" is a
	 * SECOND key path the op reads: the rekey fold moves a subtree from one
	 * prefix to another, so its lock set spans the src path and the dst
	 * path, and @lock_d -- one descent, one path -- can date only half of it.
	 *
	 * A node under a move has an OLD path and a NEW one, and its anchor is a
	 * function of the path it is on NOW: that is the only one a concurrent
	 * peer can compute, since a peer reaches the node by descending to it,
	 * and the move is a single commit, so no instant exists at which the two
	 * disagree (doc/design/ft-dlm-lock-coarseness.md §3).  Anchoring a
	 * src-origin member from @lock_d would name a node on the DST path and
	 * exclude nobody.
	 *
	 * @deferred[].dst_origin, which already separates the two sides for the
	 * apply order, is the selector.  NULL for every op whose members all lie
	 * on @lock_d's path (graft, merge, insert), and then the selector is
	 * inert.
	 */
	const struct ft_descent *lock_d_src;
	/*
	 * The op's OTHER glue, when it commits two together (the rekey fold's
	 * dst glue and the src glue it fuses into one txn).  One OP has one HELD
	 * SET, and a dedupe that reads only the glue it was handed answers "not
	 * held" for a word the op is holding through the other -- which is not a
	 * refusal a retry can clear, since the next attempt takes the same two
	 * marks in the same order and refuses again
	 * ([[feedback_self_refusal_is_not_contention]]).
	 *
	 * Coarsening is what makes the two meet: a src child's anchor is the src
	 * node the merge build already fenced as an overlap.  Symmetric, set once
	 * on both glues; followed exactly ONE level, never a chain.
	 *
	 * Torn down from BOTH ends by ft_glue_fini, which is why it is not const:
	 * fini frees the very arrays a dedupe scans, and the two halves do NOT
	 * die together -- the fold aborts its src side on paths its dst side
	 * survives, so a surviving @peer would scan a freed list against a count
	 * fini leaves standing.
	 */
	struct ft_glue *peer;
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
	/*
	 * @publish_parent_holder deduped onto a word the op ALREADY held (a
	 * caller's out-of-registry marks chained in as @outer).  The fence is in
	 * force -- so the forward publish may still PARK its SW store, which is
	 * legal exactly because the op holds the slot's word -- but the acquire
	 * that FIRST took it owns both the release and the registry entry, so
	 * this commit records neither.
	 */
	bool publish_parent_shared;
	uintptr_t publish_parent_snap;
	/*
	 * The TXN owns this fence's outcome, handed over at the TAKE rather than
	 * at ft_glue_txn_commit_edges -- @publish_gp_txn_owned's argument and the
	 * same mechanism.  See ft_glue_take_publish_parent.
	 */
	bool publish_parent_txn_owned;
	/*
	 * THE SKIP_X DUAL'S OWNER (§9.3, one level up).  A publish into a
	 * COMPRESSED @publish_parent is not one store: _ft_publish_to_parent also
	 * re-encodes the SKIP_X pointer that lets a candidate reader bypass the
	 * compressed node, and that pointer lives in a slot of the compressed
	 * node's OWN parent -- a THIRD node.  Under MW the record's expected-old
	 * arbitrates that second slot, so every ordinary glue caller leaves this
	 * NULL and is byte-identical.  A STRUCTURAL_SW caller cannot: its records
	 * PARK, a park does not arbitrate, and the fold's rule is "SW iff the op
	 * holds the slot's word" -- so it must present that grandparent's node
	 * lock here, exactly as ft_node_recompact takes {GP} as the third member
	 * of its lock-set whenever its parent P is compressed.
	 *
	 * @publish_gp_shared says the acquire deduped onto a word the op already
	 * held (coarsening collapses GP onto the same word as @publish_parent
	 * routinely): the fence is in force, but the FIRST acquire owns both the
	 * release and the registry entry, so this commit records neither.
	 */
	struct cds_ft_metadata *publish_gp_holder;
	bool publish_gp_shared;
	uintptr_t publish_gp_snap;
	/*
	 * The TXN owns this fence's outcome -- its {LOCK|s -> s} release is in
	 * the edge set AND the word is in locks[] -- because the TAKE handed it
	 * over immediately (@holder_txn_owned's argument, and the same
	 * mechanism).  The FIELD stays set: it is still the witness every narrow
	 * glue predicate reads (ft_glue_held_snap_one, ft_glue_op_holds), and
	 * dropping it would answer "not held" for a word the op holds.  What the
	 * flag changes is who CLEARS -- ft_glue_abort must not -- and that
	 * ft_glue_txn_commit_edges must not hand the same fence over twice.
	 *
	 * ☠ THE TAKE IS THE ONLY CORRECT MOMENT.  Records into the fenced node's
	 * BODY happen between the take and commit_edges -- the fold's detach
	 * republishes into it -- and a record-time owner check reads the txn
	 * registry alone, so a fence that arrives later reads as UNOWNED at every
	 * one of them.
	 */
	bool publish_gp_txn_owned;
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
	 *
	 * @split_cn_holder is the word the acquire LOCKED -- @cn's ANCHOR -- and
	 * @split_cn_node is @cn's OWN word, the one the retire tombstones.  Under
	 * per-node granularity they are the same word and the terminal is the
	 * single fused {LOCK|s -> TOMBSTONE|s}; coarsening splits them, and then
	 * the anchor takes a RELEASE while the tombstone lands on @cn.  Retiring
	 * the anchor instead would tombstone a LIVE ancestor -- the very node this
	 * graft publishes into.
	 */
	struct cds_ft_metadata *split_cn_holder;
	/*
	 * @split_cn_holder deduped onto a word the op ALREADY held (its caller's
	 * out-of-registry marks, chained in as @outer).  The fence is in force,
	 * but this member owes NO release and NO anchor terminal -- the acquire
	 * that first took the word recorded both, and settling it twice drops a
	 * word the op still writes under.  @cn's OWN retire is unaffected: the
	 * tombstone lands on @split_cn_node, never on the shared anchor.
	 */
	bool split_cn_shared;
	struct cds_ft_metadata *split_cn_node;
	uintptr_t split_cn_node_snap;
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
	 * IN -- DROP THE OLD DIRECTION of the split.  Names the compressed node
	 * whose ONE child is a subtree the caller is MOVING away in this same
	 * decide: an in-trie rekey whose src junction hangs off the very run the
	 * dst key diverges inside.  When the build splits exactly this node it
	 * omits the old-direction half outright -- no suffix node, no branch, no
	 * edge to the displaced child -- and lays a plain fresh path for the new
	 * key over the span the run covered.  The old child is then unreachable
	 * the instant the forward publish lands, which is what makes the move ONE
	 * flip instead of a publish plus a detach.
	 *
	 * ☠ WHY IT CANNOT BE A DETACH INSTEAD.  Splitting a run RETIRES it, so a
	 * detach that then clears a slot in that same node edits the copy this
	 * flip already superseded -- its lock-set acquire aborts against the
	 * pending tombstone and the op's retry loop re-derives the identical plan
	 * forever.  Dropping the direction AT BUILD TIME is what removes the
	 * second edit rather than trying to order it.
	 *
	 * The caller owes the identity: a compressed node has exactly one child,
	 * so naming the node names the edge, and it must be the subtree whose
	 * COPY this build's @payload is.
	 */
	struct cds_ft_inode_flag *drop_old_dir_of;
	/* OUT: the build took that path, so the caller owes NO detach. */
	bool old_dir_dropped;
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
 * The op's lock context, as a glue op carries it: the descent that dates its
 * lock-set members, and @txn as the registry naming what it already holds.
 * Built into the caller's own storage at each use rather than cached on the
 * glue: the glue is shared across the op's steps, and @txn changes under it.
 */
static inline
void ft_glue_lock_ctx(const struct ft_glue *g, struct ft_lock_ctx *ctx)
{
	ft_lock_ctx_init(ctx, g->lock_d, g->txn, g->op);
	/*
	 * The glue's OWN marks are part of the op's held set -- its build fences
	 * the compressed node it splits, and under a coarse spacing the publish
	 * parent acquired later anchors onto that very word.
	 */
	ctx->held.glue = g;
}

/*
 * The same context, for a member whose CURRENT path is the op's src rather than
 * its dst (@lock_d_src, above).  @dst_origin is the deferred edge's own flag;
 * an op with no second path answers identically for both values.
 */
static inline
void ft_glue_lock_ctx_origin(const struct ft_glue *g, struct ft_lock_ctx *ctx,
		bool dst_origin)
{
	ft_glue_lock_ctx(g, ctx);
	if (!dst_origin && g->lock_d_src)
		ctx->d = g->lock_d_src;
}


/*
 * ft_glue helpers.  The struct and the rationale are defined just above;
 * the build-invisible builders that reference the type (ft_build_branch and
 * the graft / merge spine builders) follow in the later mutation modules.
 */
static
void ft_glue_init(struct ft_glue *g)
{
	g->op = NULL;			/* the op sets it beside g->txn */
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
	g->lock_d = NULL;
	g->lock_d_src = NULL;
	g->peer = NULL;
	g->publish_slot = NULL;
	g->top = NULL;
	g->publish_old = NULL;
	g->publish_old_set = false;
	g->publish_parent_holder = NULL;
	g->publish_parent_shared = false;
	g->publish_parent_snap = 0;
	g->publish_parent_txn_owned = false;
	g->publish_gp_holder = NULL;
	g->publish_gp_shared = false;
	g->publish_gp_snap = 0;
	g->publish_gp_txn_owned = false;
	g->split_cn_holder = NULL;
	g->split_cn_shared = false;
	g->split_cn_node = NULL;
	g->split_cn_node_snap = 0;
	g->caller_holder = NULL;
	g->split_cn_snap = 0;
	g->fence_split_cn = false;
	g->drop_old_dir_of = NULL;
	g->old_dir_dropped = false;
	g->attached_nf = NULL;
	g->txn = NULL;
	g->fuse_free_list = false;
	g->record_only = false;
	g->count_delta = 0;
}

/*
 * Take the glue's PUBLISH PARENT fence AND hand it to @g->txn in the same
 * breath: the two lines are the ones ft_flip_txn_hold_or_lock_parent's held arm
 * runs, and the take is the moment they become true.
 *
 * ☠ THE TRANSFER CANNOT WAIT FOR ft_glue_txn_commit_edges.  Records into the
 * fenced node's BODY happen in between -- the fold's detach recompacts under
 * this word and republishes into it -- and a record-time owner check reads the
 * txn's locks[] and nothing else, so a fence that arrives afterwards reads as
 * UNOWNED at every record in the window.  The acquires those records sit behind
 * DEDUPE onto this fence and therefore register nothing themselves (rightly:
 * this acquire owes the release), so the registry never learns it by any other
 * route.
 *
 * The FIELDS stay set.  They are still the witness every NARROW glue predicate
 * reads (ft_glue_held_snap_one, ft_glue_op_holds), and clearing them would
 * answer "not held" for a word the op holds; @publish_parent_txn_owned moves
 * only who CLEARS -- ft_glue_abort must not, or a bail double-clears against
 * ft_flip_txn_lock_release_all's strict release, and commit_edges must not hand
 * the same fence over twice.
 *
 * A SHARED acquire transfers nothing: the fence is in force, but the acquire
 * that FIRST took the word owns both the release and the registry entry.
 */
static inline
void ft_glue_take_publish_parent(struct ft_glue *g,
		struct cds_ft_metadata *holder, uintptr_t snap, bool shared)
{
	g->publish_parent_holder = holder;
	g->publish_parent_snap = snap;
	g->publish_parent_shared = shared;
	if (shared || !holder || !g->txn)
		return;
	ft_flip_txn_lock_register(g->txn, holder, snap);
	ft_flip_txn_record_anchor_release_held(g->txn, holder);
	g->publish_parent_txn_owned = true;
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
	/*
	 * Break the two-glue held set from BOTH ends, FIRST: everything below
	 * frees an array a peer's dedupe would scan, and the counts naming those
	 * arrays are left standing.
	 */
	if (g->peer) {
		g->peer->peer = NULL;
		g->peer = NULL;
	}
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
 * Record a child's PARENT WORD re-parent edge, expected old read THROUGH the
 * txn.
 *
 * The slot enters this txn's WRITE set here, so the engine's read policy
 * demands a WAITING load.  A raw read bakes a peer's parked FT_FLIP_PROXY_TAG
 * -- a descriptor POINTER -- into the expected old: the install CAS can then
 * never match, so the attempt is a guaranteed abort and the retry re-reads the
 * same parked word.  A release build turns that into a silent poison-and-abort
 * the retry loops absorb, which is why it stayed invisible; --enable-rcu-debug
 * traps it at urcu_txn_add's !urcu_txn_is_proxy assert.
 *
 * The tag is the one the RECORD carries (FT_FLIP_PROXY_TAG via
 * ft_flip_txn_record_reserved), NOT the state word's FT_STATE_PROXY.
 */
#ifdef FT_RED_PARENT_WORD_SW
static unsigned long ft_red_pw_sw_unheld, ft_red_pw_sw_held;

static __attribute__((destructor))
void ft_red_pw_sw_report(void)
{
	fprintf(stderr, "# FT_RED_PARENT_WORD_SW: %lu parent_word parks on an "
		"UNHELD child (the injected defect), %lu on a held one\n",
		uatomic_load(&ft_red_pw_sw_unheld, CMM_RELAXED),
		uatomic_load(&ft_red_pw_sw_held, CMM_RELAXED));
}
#endif

static inline
void ft_flip_txn_record_parent_word(const struct cds_ft *ft,
		struct ft_flip_txn *txn, struct cds_ft_metadata *meta,
		struct cds_ft_inode_flag *parent_nf, bool child_held)
{
	void *old_pw = urcu_txn_load(txn->mtxn, (void **) &meta->parent_word,
			FT_FLIP_PROXY_TAG);
	void *new_pw = ft_parent_word(ft, parent_nf);

	/*
	 * EDGE KIND follows who HOLDS the child, exactly as the state edge in
	 * ft_reparent_record_meta does -- same node, same reason.
	 *
	 * A structural edge may park SW only "when the op holds the DLM lock
	 * over @slot" (ft_flip_txn_record_tag), and an SW park CANNOT FAIL: it
	 * does not arbitrate against a peer.  @slot here is the CHILD's
	 * parent_word, while the DLM set is {C,(P),(GP)} whose acquire states
	 * that "C's CHILDREN are never in it".  Parking unconditionally
	 * therefore claims an exclusion the op does not have whenever
	 * @child_held is false, and two ops re-parenting one unheld child both
	 * park, neither fails, and the last install wins -- a lost update that
	 * leaves the loser's child naming a node the winner then retires.
	 *
	 * The {live_state -> live_state} validates recorded beside this cannot
	 * arbitrate that pair either: both expect the same value and neither
	 * moves it.
	 *
	 * MW makes the second writer's expected-old mismatch and abort, which is
	 * what the retry lane exists to absorb.
	 */
#ifdef FT_RED_PARENT_WORD_SW
	/*
	 * RED CONTROL, never a shipped configuration: park unconditionally,
	 * which is this function as it stood before @9ef2f648 and is a REAL
	 * defect rather than a synthetic one -- the canonical unarbitrated SW
	 * park.  It exists so inv_rekey_fine_mixed_writers can be shown to
	 * DETECT the class it is named for instead of merely running in it.
	 * Keep it out of --enable-rcu-debug builds: there the engine's own
	 * kind/duplicate-slot asserts fire first, and then the assert is the
	 * detector, not the oracle.
	 *
	 * ★ AND IT COUNTS ITS OWN ARM.  A red control that is never TAKEN is
	 * indistinguishable from a green one, and that mistake has already been
	 * made twice on this arm -- so the unheld parks (the ones this
	 * deliberately gets wrong) are tallied and reported at exit.  A zero
	 * there means the control proved nothing, whatever the arm reported.
	 */
	if (!child_held)
		uatomic_inc(&ft_red_pw_sw_unheld);
	else
		uatomic_inc(&ft_red_pw_sw_held);
	ft_flip_txn_record_reserved(txn, /*owner=*/ meta,
		(void **) &meta->parent_word, old_pw, new_pw);
	return;
#endif
	/*
	 * @child_held IS the ownership predicate this word needs, and @meta --
	 * the CHILD -- is the node it tests: the DLM set here is {C,(P),(GP)}
	 * and what makes the park legal is holding the child whose parent word
	 * moves.  So the owner named here is the same node the branch above
	 * already decides on; the record-time check re-asks it against the
	 * registry, which is where a hold taken on a frame the txn never
	 * registered shows up.
	 */
	if (child_held)
		ft_flip_txn_record_reserved(txn, /*owner=*/ meta,
			(void **) &meta->parent_word, old_pw, new_pw);
	else
		ft_flip_txn_record_tag_mw(txn, (void **) &meta->parent_word,
			old_pw, new_pw, FT_FLIP_PROXY_TAG);
}

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
 * proxy.  That says nothing about the child's PARENT WORD, which a peer can
 * have parked: a graft's displaced-child re-parent reaches here on a published
 * node (measured, from ft_merge_at_inner), so every parent-word edge goes
 * through ft_flip_txn_record_parent_word's read-your-own-writes load.  List off
 * only (the converted phase): an external head's parent is its prev directly.
 * Records cannot fail -- the caller reserved @txn to the bounded cluster size
 * up front.
 *
 * ★ WHICH ARM the published-node case reaches matters, and this comment used to
 * leave it open: it is the METADATA arms, and those are exactly the ones that
 * take the RYW-safe ft_flip_txn_record_parent_word.  The EXTERNAL arm below
 * reads its back-edge RAW, which is only sound because the external children
 * reaching it are build-invisible -- reachable solely through the forward flip,
 * so no peer can park on that field.  Read as a single claim about the whole
 * function the two look contradictory, and the raw read reads like the
 * expected-old defects fixed at c3400b27 / f8b1640e.
 *
 * MEASURED rather than argued, since the distinction is the whole safety
 * argument: a probe counting how often either external field already carries a
 * proxy sees ZERO, over 5541 reaches of the two arms across unit + inv, all
 * three lock spacings, with the MW oracles enabled.  (The same probe over the
 * other nine raw expected-old reads in the tree: zero of 16.4M.)
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
		ft_flip_txn_record_parent_word(ft, txn, cn_meta, parent_nf,
		/*child_held=*/ true);
		return;
	}
	if (ft_node_compressed(child_nf)) {
		struct cds_ft_compressed_node *cn =
			ft_compressed_node_ptr(child_nf);
		struct cds_ft_metadata *cn_meta =
			cds_ft_item_to_metadata((struct cds_ft_inode *) cn);

		ft_set_parent_slot(cn_meta, parent_nf, slot);
		ft_flip_txn_record_parent_word(ft, txn, cn_meta, parent_nf,
		/*child_held=*/ true);
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

			ft_flip_txn_record_head_back_edge(txn,
				(void **) &cell->parent, cell->parent, parent_nf);
		} else {
			ft_flip_txn_record_head_back_edge(txn,
				(void **) &en->prev, en->prev, parent_nf);
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
		ft_flip_txn_record_parent_word(ft, txn, meta, parent_nf,
		/*child_held=*/ true);
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
		struct cds_ft_inode_flag **slot, bool child_marked,
		const struct ft_lock_ctx *hold_ctx)
{
	/*
	 * THE THIRD STATE @child_marked cannot express: the op HOLDS this
	 * child's word, but a DIFFERENT STEP took it and therefore owns its
	 * release.
	 *
	 * @child_marked is two-valued and both of its values are wrong here.
	 * FALSE records the §4.B MW validate below, whose expected-old is the
	 * word CLEAN -- and the word carries the op's OWN LOCK, so the install
	 * CAS can never match: a deterministic abort on every attempt, which
	 * presents as a livelock with no contention.  TRUE records the SW
	 * {live_state -> live_state} form, which IS a release -- and releasing
	 * a mark this step never took hands the word a second terminal.
	 *
	 * So a caller that can see the op's whole held set passes it, and a
	 * child found in it takes neither: no state edge at all.  The
	 * justification is already written at ft_flip_txn_hold_or_lock_parent's
	 * SHARED arm -- "the mark is the stronger statement anyway, it is the
	 * exclusion the guard approximates, already in force, so the word it
	 * protects owes nothing here."  The validate exists to catch a child a
	 * PEER froze mid-recompact; a word this op holds cannot be frozen by a
	 * peer at all.
	 *
	 * NULL @hold_ctx keeps every caller that cannot answer the question on
	 * the two-valued dispatch, byte-identical.  Only a caller that is NEVER
	 * THE ACQUIRER may pass one -- for it, "held" always means "held by an
	 * earlier step" -- which is exactly ft_node_recompact's reparent sweep.
	 */
	bool held_earlier = false;

	if (hold_ctx) {
		uintptr_t held_snap;
		bool ratified;

		held_earlier = ft_lock_ctx_holds(hold_ctx, meta, &held_snap,
				&ratified);
	}
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
	/*
	 * READ IT THROUGH THE TXN, for the reason the parent_slot_offset load
	 * above spells out -- the two fields are siblings and the rule is the
	 * same.  This slot enters THIS txn's write set on the very next line, so
	 * the engine's read policy requires a WAITING load: a raw read bakes a
	 * peer's parked FT_FLIP_PROXY_TAG -- a descriptor POINTER -- into the
	 * expected old, which the install CAS can never match, so the attempt is
	 * a guaranteed abort and the retry re-reads the same parked word.
	 *
	 * @meta is LIVE here (this is the re-parent of a published child), which
	 * is exactly the case that CAN meet a peer's park.  Measured:
	 * --enable-rcu-debug traps it at urcu_txn_add's !urcu_txn_is_proxy
	 * assert, on the DEFAULT per-node granularity, from
	 * ft_chain_compress_fused's back-edge fold.
	 */
	ft_flip_txn_record_parent_word(ft, txn, meta, parent_nf,
		child_marked || held_earlier);
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
	if (held_earlier) {
		/*
		 * NEITHER, and the ledger is NOT dropped: the step that took
		 * this word still owes its release and is still the only one
		 * that may record it.  See the @hold_ctx note at the top.
		 */
	} else if (child_marked) {
		/*
		 * This edge IS the mark's release (live_state masks LOCK out), so
		 * the op stops owing one the moment it is recorded and the ledger
		 * must forget the word -- nothing else will, because the caller
		 * skips its release sweep once the commit consumed the marks.
		 */
		ft_hold_trace_drop(meta);
		ft_flip_txn_record_state(txn, meta,
			(void *) live_state, (void *) live_state);
	} else
		ft_flip_txn_record_state_mw(txn, meta,
			(void *) live_state, (void *) live_state);
	/*
	 * THE OFFSET IS THE THIRD WORD OF THE SAME CHILD, and it takes the same
	 * kind dispatch as the two above it -- @meta->parent_word
	 * (ft_flip_txn_record_parent_word) and @meta->state.  It is the one word
	 * of the parentage triple that was left on the unconditional recorder,
	 * which dispatches on @txn->structural_sw ALONE: in a fold commit that
	 * parks an UNVALIDATED plain store on a word the op does not own, since
	 * the DLM set here is {C,(P),(GP)} and C's CHILDREN are never in it.
	 *
	 * An SW park cannot fail, so two ops re-homing one unheld child both
	 * park, neither aborts, and the later SETTLE stores its offset over the
	 * winner's -- and the settle is a plain store, so it also erases
	 * whatever the peer left in the word after that.
	 *
	 * MW makes the second writer's expected-old mismatch and abort, which is
	 * what the retry lane absorbs.  Byte-neutral for every non-fold caller:
	 * with structural_sw false, record_tag IS record_tag_mw.
	 */
	if (record_pso) {
		if (child_marked || held_earlier)
			ft_flip_txn_record_tag(txn, /*owner=*/ meta,
				(void **) &meta->parent_slot_offset,
				(void *) old_pso, (void *) new_pso,
				FT_STATE_PROXY);
		else
			ft_flip_txn_record_tag_mw(txn,
				(void **) &meta->parent_slot_offset,
				(void *) old_pso, (void *) new_pso,
				FT_STATE_PROXY);
	}
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
		struct cds_ft_inode_flag **slot, bool child_marked,
		const struct ft_lock_ctx *hold_ctx)
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
			parent_nf, slot, child_marked, hold_ctx);
		return;
	}
	if (ft_node_compressed(child_nf)) {
		struct cds_ft_compressed_node *cn =
			ft_compressed_node_ptr(child_nf);

		ft_reparent_record_meta(ft, txn,
			cds_ft_item_to_metadata((struct cds_ft_inode *) cn),
			parent_nf, slot, child_marked, hold_ctx);
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
		/*
		 * WAITING loads, not raw ones, for the reason
		 * ft_reparent_record_meta's &meta->state load spells out: the
		 * slot read here is the slot recorded on the very next line, so
		 * it enters THIS txn's write set and its last load must wait out
		 * a parked owner.  A raw read hands a peer's parked flip proxy --
		 * a descriptor-record POINTER -- to urcu_txn_add as the
		 * expected-old, which is the engine's !urcu_txn_is_proxy(old_ptr)
		 * self-check (an --enable-rcu-debug abort; a release build
		 * POISONS the descriptor instead and the retry loop absorbs it).
		 */
		if (ft->ordered_list) {
			struct ft_ord_cell *cell = ft_ord_cell_ptr(en->prev);

			ft_flip_txn_record_head_back_edge(txn,
				(void **) &cell->parent,
				urcu_txn_load(txn->mtxn, (void **) &cell->parent,
					FT_FLIP_PROXY_TAG),
				parent_nf);
		} else {
			ft_flip_txn_record_head_back_edge(txn,
				(void **) &en->prev,
				urcu_txn_load(txn->mtxn, (void **) &en->prev,
					FT_FLIP_PROXY_TAG),
				parent_nf);
		}
		return;
	}
	ft_reparent_record_meta(ft, txn,
		cds_ft_item_to_metadata(ft_node_ptr(child_nf)), parent_nf, slot,
		child_marked, hold_ctx);
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
			g->deferred[i].live = dst_origin || g->record_only;
			return;
		}
	}
	assert(g->nr_deferred < g->cap_deferred);
	g->deferred[g->nr_deferred].child = child;
	g->deferred[g->nr_deferred].parent = parent;
	g->deferred[g->nr_deferred].slot = slot;
	g->deferred[g->nr_deferred].dst_origin = dst_origin;
	/*
	 * WHO IS STILL REACHABLE (see @live at the struct).  @record_only is the
	 * FOLD, and the fold's src spine is unlinked by the very commit this edge
	 * rides -- so a src-origin child under it is reader-reachable for the
	 * whole build window.  Every other committer arrives here with its source
	 * already unlinked and DRAINED (graft, the merge src side) or EXCLUSIVE
	 * (graft_swap), which is what makes its src-origin children invisible and
	 * their stores unobservable.
	 *
	 * Read at DEFER time, not at apply time, because it is the BUILD that
	 * knows where a child came from -- and @record_only is set on the glue
	 * before its build defers anything.
	 */
	g->deferred[g->nr_deferred].live = dst_origin || g->record_only;
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
	g->deferred[g->nr_deferred].lock_word = NULL;
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
	/*
	 * Announce the pending publish to any recompaction of @parent_nf that
	 * runs before the commit -- the same-trie rekey's src detach does
	 * exactly that -- so the copy carries @top, not the child it replaces.
	 */
	if (g->txn) {
		g->txn->pending_pub_slot = parent_slot;
		g->txn->pending_pub_val = top;
	}
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
	g->free_list[g->nr_free].holder = NULL;
	g->free_list[g->nr_free].holder_snap = 0;
	g->free_list[g->nr_free].holder_shared = false;
	g->free_list[g->nr_free].holder_txn_owned = false;
	g->free_list[g->nr_free].snap = 0;
	g->nr_free++;
}

/*
 * DLM overlap-spine plan-lock (§9.4 M-2): record a replaced node whose LOCK
 * fence the caller ALREADY acquired -- before copying its content into the merged
 * cluster -- stashing the mark's clean @snap so the freeze can record the fenced
 * {LOCK|s -> TOMBSTONE|s} terminal.  The mark is owned by the op and released
 * by ft_glue_clear_fenced on any non-committing exit -- unless the freeze hands
 * a SURVIVING anchor to the txn instead (@holder_txn_owned).
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
		void *node, bool compressed, const struct ft_held_anchor *h)
{
	assert(g->nr_free < g->cap_free);
	g->free_list[g->nr_free].node = node;
	g->free_list[g->nr_free].compressed = compressed;
	g->free_list[g->nr_free].retired = true;
	g->free_list[g->nr_free].fenced = true;
	g->free_list[g->nr_free].snap = h->node_snap;
	g->free_list[g->nr_free].holder = h->lock;
	g->free_list[g->nr_free].holder_snap = h->lock_snap;
	g->free_list[g->nr_free].holder_shared = h->shared;
	g->free_list[g->nr_free].holder_txn_owned = false;
	g->nr_free++;
}

/*
 * Release every node lock this glue's overlap-spine plan-lock still holds AND
 * still owns.  Call at the op's terminal on BOTH the committing and the aborting
 * path: ft_meta_lock_release_if_held no-ops on an entry whose fenced retire the
 * commit already turned TOMBSTONE, and releases one left {LOCK|s} by an abort --
 * so no per-outcome bookkeeping is needed, which is the same reason the detach's
 * orphan chain sweeps unconditionally.
 *
 * ★ THAT ARGUMENT IS THE RETIRED NODE'S OWN WORD SPEAKING, and it holds only
 * while the mark sits on it: a TOMBSTONE is un-lockable, so a LOCK still set
 * there is ours.  A COARSENED mark sits on an ancestor that SURVIVES, whose
 * terminal leaves it clean, LIVE and immediately re-lockable -- asking that word
 * "are you still locked?" gets YES from a PEER and strips its mark.  Such a mark
 * is handed to the txn at the freeze (@holder_txn_owned) and skipped here.
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
		/*
		 * The caller asks about the word it would ACQUIRE, so compare
		 * the word this entry HOLDS.  Comparing the node instead makes
		 * a coarsened entry answer for a word it never took.
		 */
		if (g->free_list[i].holder == meta)
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
		/*
		 * Release the word the acquire TOOK, which coarsening makes an
		 * ancestor rather than the node itself.  A shared entry took
		 * none, so it releases none -- the entry that first held the
		 * word owns that release; a txn-owned one was handed away.
		 */
		if (!g->free_list[i].holder_shared &&
				!g->free_list[i].holder_txn_owned &&
				g->free_list[i].holder)
			ft_meta_lock_release_if_held(g->free_list[i].holder);
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
bool ft_glue_held_snap_one(const struct ft_glue *g,
		const struct cds_ft_metadata *meta, uintptr_t *snap,
		bool *ratified)
{
	int i;

	*ratified = true;
	if (g->publish_parent_holder == meta) {
		*snap = g->publish_parent_snap;
		return true;
	}
	if (g->publish_gp_holder == meta) {
		*snap = g->publish_gp_snap;
		return true;
	}
	if (g->split_cn_holder == meta) {
		*snap = g->split_cn_snap;
		return true;
	}
	if (g->caller_holder == meta) {
		/*
		 * The CALLER acquired this one and owns its release, so this op
		 * never sampled the word.  Held all the same: dedupe is what
		 * makes the op terminate, and it must not be skipped for want of
		 * a value.
		 */
		*ratified = false;
		return true;
	}
	for (i = 0; i < g->nr_free; i++) {
		if (!g->free_list[i].fenced)
			continue;
		if (cds_ft_item_to_metadata((struct cds_ft_inode *)
				g->free_list[i].node) == meta) {
			*snap = g->free_list[i].snap;
			return true;
		}
		/*
		 * ★ And the word the acquire actually LOCKED, which coarsening
		 * makes an ANCESTOR of @node.  Asking only about @node answers
		 * "not held" for the very word this op holds; at per-node the two
		 * ARE one word and this arm never fires.  A @holder_shared entry
		 * carries no snapshot of its own -- the acquire that first took
		 * the word owns its value -- exactly as @extra's shared entries.
		 */
		if (g->free_list[i].holder == meta &&
				!g->free_list[i].holder_shared) {
			*snap = g->free_list[i].holder_snap;
			return true;
		}
	}
	for (i = 0; i < g->nr_splices; i++)
		if (g->splices[i].holder == meta) {
			*snap = g->splices[i].holder_snap;
			return true;
		}
	return false;
}

/*
 * The same question against the OP's held set rather than one glue's: @peer is
 * the other half when a single commit fuses two builds.  One level, so the
 * answer terminates whatever the callers wire.
 */
static
bool ft_glue_held_snap(const struct ft_glue *g,
		const struct cds_ft_metadata *meta, uintptr_t *snap,
		bool *ratified)
{
	if (ft_glue_held_snap_one(g, meta, snap, ratified))
		return true;
	return g->peer &&
		ft_glue_held_snap_one(g->peer, meta, snap, ratified);
}

static
bool ft_glue_op_holds(const struct ft_glue *g,
		const struct cds_ft_metadata *meta)
{
	uintptr_t snap;
	bool ratified;

	return ft_glue_held_snap(g, meta, &snap, &ratified);
}

/*
 * ☠ THE EXCLUSIVE SKIP'S ONE OBLIGATION.
 *
 * Not taking the mark also means not DETECTING one: @held_lock stays false for
 * every deferred entry, so a live re-parent records the §4.B MW guard, whose
 * expected-old is the child's CLEAN live_state.  That is the right record iff
 * the word really is clean -- and it is not if this op ALREADY holds the
 * child's own word through one of its other lock sets.  The guard would then
 * validate against a fence we planted ourselves and mismatch on every attempt:
 * the deterministic self-abort ft_reparent_record_meta documents.
 *
 * No exclusive shape reaches here holding a re-parented child's word, so this
 * STATES that instead of paying for a detection walk to discover it.  A claim
 * with no behaviour attached is what makes it an assert and not a branch
 * (FT_OWNER_ASSERT_OWNED is the same shape), and it is armed exactly where a
 * violation would otherwise be absorbed by a retry loop.
 *
 * Both sets are asked because a word can be held from either: the glue's own
 * lock sets (and its peer's, cross-trie) and the flip-txn registry.
 */
static
void ft_glue_assert_reparent_unheld(struct cds_ft *ft, struct ft_glue *g)
{
#if defined(DEBUG_RCU) || defined(CONFIG_RCU_DEBUG)
	int i;

	for (i = 0; i < g->nr_deferred; i++) {
		struct cds_ft_metadata *cm;

		if (!g->deferred[i].live)
			continue;
		cm = ft_glue_reparent_park_meta(ft, g->deferred[i].child);
		if (!cm)
			continue;
		urcu_assert_debug(!ft_glue_op_holds(g, cm));
		urcu_assert_debug(!ft_flip_txn_owns(g->txn, cm));
	}
#else
	(void) ft;
	(void) g;
#endif
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
 * AND ONLY UNDER LOCK_FINE, exactly as its sibling
 * ft_glue_acquire_splice_holders.  The mark is a lock-set EXTENSION over a word
 * the op's own DLM set does not cover, and it defends against ONE peer: the
 * ft_meta_nr_child_inc of an insert BELOW @child.  It is FINE mode that leaves
 * that peer unexcluded, because FINE is where the FT-wide writer mutex is
 * dropped (§11) and the per-node lock-sets are the whole exclusion.  On a COARSE
 * non-exclusive trie that peer holds the FT-wide mutex for its entire op -- the
 * same mutex this op holds, and the very exclusion that licenses the SW park
 * there -- so the mark has nobody to arbitrate against and the guard edge it
 * would displace validates against a word no peer can move.
 *
 * AND NOT ON AN EXCLUSIVE TRIE, for that same reason carried one step further.
 * The peer the mark arbitrates against is a concurrent WRITER, and an exclusive
 * trie has none at all -- a stronger exclusion than the COARSE mutex, which at
 * least admits one writer at a time.  @lock_fine is a GROUP property and
 * @exclusive a per-trie one, so the two are independent and a FINE trie can be
 * exclusive; taking the mark there costs an acquire that can only refuse, from
 * a caller whose bail is a genuine abort.  ft_glue_txn_commit_edges mirrors
 * this gate EXACTLY and asserts on the arming state, so the two must move
 * together.
 *
 * Returns -EAGAIN on a contended child; the caller aborts and re-descends, and
 * ft_glue_abort releases whatever was taken before the miss.
 */
static
int ft_glue_acquire_reparent_marks(struct cds_ft *ft, struct ft_glue *g)
{
	int i, j;

	if (!ft->lock_fine || ft->exclusive)
		return 0;
	if (!g->txn || !g->txn->structural_sw)
		return 0;
	for (i = 0; i < g->nr_deferred; i++) {
		struct cds_ft_metadata *cm =
			ft_glue_reparent_park_meta(ft, g->deferred[i].child);
		struct cds_ft_metadata *anchor;
		struct ft_lock_ctx gctx;
		struct ft_held_anchor h;
		unsigned int cd;
		uintptr_t cm_snap;
		bool cm_rat, dup = false;

		g->deferred[i].marked = false;
		g->deferred[i].held_lock = false;
		g->deferred[i].lock_word = NULL;
		/*
		 * ONLY THE EDGES THIS COMMIT RECORDS (@live at the struct).  The
		 * mark exists to make an SW park safe, and only a recorded edge
		 * parks: a HIDDEN edge is a plain store into a node no reader can
		 * reach, whose only peer would be one that cannot reach it either.
		 * Marking it takes a lock on a build-invisible child -- pure cost,
		 * and an acquire that can only refuse, from a caller that is past
		 * the source unlink and has no bail left.
		 */
		if (!g->deferred[i].live)
			continue;
		if (!cm)
			continue;
		/*
		 * §7.2 fan-out: every child of one node shares a byte-depth, so
		 * a coarse spacing either collapses them all onto one anchor --
		 * frequently one this op already holds, making the collapse
		 * 256 -> 0 -- or gives each its own, and none can collide.  The
		 * dedupe below is what decides it either way.  A child the
		 * descent cannot date has no anchor here: re-plan.
		 *
		 * A re-parent target sits on the path it is being moved OFF, so
		 * the descent that dates it is the one that WALKED that path:
		 * @dst_origin picks it (@lock_d_src).  And it sits BELOW that
		 * descent's cursor -- it is a child of the node the build took it
		 * from -- which the window cannot name, so the one-hop derivation
		 * from its live parent is what answers.
		 */
		ft_glue_lock_ctx_origin(g, &gctx, g->deferred[i].dst_origin);
		if (!ft_lock_ctx_depth_of(ft, &gctx, g->deferred[i].child, &cd)) {
			struct cds_ft_inode_flag *live_parent = NULL;

			(void) ft_resolve_parent_slot(cm, ft, &live_parent);
			if (!ft_lock_ctx_depth_of_cursor_child(ft, &gctx,
					live_parent, &cd))
				return -EAGAIN;
		}
		anchor = ft_anchor_meta(ft, ft_lock_ctx_descent(&gctx),
			g->deferred[i].child, cm, cd);
		if (ft_glue_op_holds(g, anchor)) {
			/*
			 * Held via another lock set: do not re-mark, do not
			 * release -- but DO record that we hold it.  @held_lock
			 * still asks about the CHILD'S OWN word (see the acquire
			 * below): holding an ancestor is not holding the child.
			 */
			g->deferred[i].held_lock = anchor == cm ||
				ft_lock_ctx_holds(&gctx, cm, &cm_snap, &cm_rat);
			g->deferred[i].lock_word = anchor;
			continue;
		}
		/*
		 * Two deferred entries can resolve to ONE word (a skip flag and
		 * the plain flag of the same compressed node; or, under
		 * coarsening, two distinct children sharing an anchor), and the
		 * dedup in ft_glue_defer_edge_origin keys on the FLAG, so it
		 * does not catch either.  A second mark on our own word is the
		 * same self-deadlock as above.
		 */
		for (j = 0; j < i; j++)
			if (g->deferred[j].lock_word == anchor)
				dup = true;
		if (dup) {
			/* Same word, marked via the earlier entry: we hold it. */
			g->deferred[i].held_lock = anchor == cm ||
				ft_lock_ctx_holds(&gctx, cm, &cm_snap, &cm_rat);
			g->deferred[i].lock_word = anchor;
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
		if (ft_acquire_member(ft, &gctx, g->deferred[i].child, cm, cd,
				&h))
			return -EAGAIN;
		if (h.shared) {
			/*
			 * NOT a failure: the acquire found the word in the op's
			 * held set and DEDUPED.  ft_glue_op_holds above reads the
			 * GLUE only, while this reads the whole context -- the txn
			 * registry, the caller's frames, an out-of-registry array
			 * -- so a word held anywhere else arrives here instead of
			 * there.  Refusing it refuses this op's own fence, which no
			 * retry can clear.  Record it held, owing no release: the
			 * acquire that first took it owns that.
			 */
			g->deferred[i].lock_word = h.lock;
			g->deferred[i].held_lock = h.lock == cm || h.node_held;
			continue;
		}
		g->deferred[i].lock_word = h.lock;
		/*
		 * ☠ @held_lock names the CHILD'S OWN word, never the anchor, and
		 * coarsening is what splits them.  It picks the re-parent's
		 * state-edge KIND: the SW park is a plain store that validates
		 * nothing, and it is legitimate only because the op holds the very
		 * word it parks.  Take an ANCESTOR's word instead and the child's
		 * own word is unheld -- a peer's ft_meta_nr_child_inc from an
		 * insert BELOW it sees a CLEAN word, does not honour
		 * FT_STATE_INPLACE_WAIT_MASK, and the park CLOBBERS its count.
		 * That is the exact clobber this acquire exists to prevent.
		 *
		 * So a coarsened member keeps the MW guard, whose expected-old is
		 * the clean live_state the child really carries: the same peer then
		 * ABORTS this commit instead, which is the outcome the escalation
		 * lane arbitrates.  Per-node makes anchor and node one word and the
		 * SW park returns, byte-identical.
		 */
		g->deferred[i].held_lock = h.lock == cm || h.node_held;
		if (h.lock == cm) {
			/*
			 * The child's own guard edge RELEASES this mark at the
			 * flip (live_state masks LOCK out), so the terminal is on
			 * this txn either way.  REGISTER IT ANYWAY: the re-parent
			 * record ft_glue_apply_deferred plants names @cm as its
			 * owner, and a record-time owner check reads locks[] and
			 * nothing else -- the mark being sweep-owned is an answer
			 * to who CLEARS, not to whether the commit owns the word.
			 * @marked then stays false, exactly as the coarsened arm
			 * below leaves it, so the abort sweep does not
			 * double-release against ft_flip_txn_lock_release_all.
			 */
			ft_flip_txn_lock_register(g->txn, h.lock, h.lock_snap);
			continue;
		}
		/*
		 * A coarsened mark has no such edge -- the guard lands on the
		 * CHILD and the LOCK sits on its ANCESTOR -- so record the
		 * anchor's own {LOCK|s -> s} release and hand it to the txn,
		 * exactly as the publish parent's held fence is handed over.  The
		 * registry then releases it on either outcome, and @marked stays
		 * false so the abort sweep does not double-release.
		 *
		 * Bounded by TWO, well inside FT_FLIP_TXN_MAX_LOCKS: a dated
		 * child is the CURSOR's own child (nothing else dates here), all
		 * children of one node share a byte-depth and therefore ONE
		 * anchor (§7.2), and an op has at most two cursors -- @lock_d and
		 * @lock_d_src.
		 */
		ft_flip_txn_lock_register(g->txn, h.lock, h.lock_snap);
		ft_flip_txn_record_release_lock(g->txn, h.lock, h.lock_snap);
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
		cm = g->deferred[i].lock_word;
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
		/*
		 * A SHARED fence belongs to the caller's earlier acquire, which
		 * owns its release: dropping it here would unlock a word the op
		 * still writes under (one owner per fence).
		 */
		if (!g->split_cn_shared)
			ft_meta_lock_release(g->split_cn_holder);
		g->split_cn_holder = NULL;
		g->split_cn_shared = false;
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
		/*
		 * SHARED: the caller's earlier acquire owns the release.
		 * TXN-OWNED: the TAKE already handed it over, and
		 * ft_flip_txn_lock_release_all drains it -- clearing here as well
		 * is the fence theft @split_cn_holder spells out.
		 */
		if (!g->publish_parent_shared && !g->publish_parent_txn_owned)
			ft_meta_lock_release_if_held(g->publish_parent_holder);
		g->publish_parent_holder = NULL;
		g->publish_parent_shared = false;
		g->publish_parent_snap = 0;
		g->publish_parent_txn_owned = false;
	}
	/*
	 * The SKIP_X dual's owner, on the same terms -- plus one: a fence the
	 * TAKE already handed to the txn (@publish_gp_txn_owned) is drained by
	 * ft_flip_txn_lock_release_all, and clearing it here as well is the
	 * fence theft @split_cn_holder spells out.
	 */
	if (g->publish_gp_holder) {
		if (!g->publish_gp_shared && !g->publish_gp_txn_owned)
			ft_meta_lock_release_if_held(g->publish_gp_holder);
		g->publish_gp_holder = NULL;
		g->publish_gp_shared = false;
		g->publish_gp_snap = 0;
		g->publish_gp_txn_owned = false;
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
		 *
		 * Matched on @cn's OWN word, which is what both this sweep and
		 * that terminal write; the anchor the acquire locked is a
		 * different word under any coarser granularity.
		 */
		if (g->split_cn_node == meta)
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
			struct ft_held_anchor h = {
				.lock = g->free_list[i].holder,
				.lock_snap = g->free_list[i].holder_snap,
				.node_snap = g->free_list[i].snap,
				.shared = g->free_list[i].holder_shared,
				.node_held = false,
			};
			uintptr_t cur;

			assert(g->fuse_free_list);
			cur = (uintptr_t) urcu_txn_load(g->txn->mtxn,
				(void **) &meta->state, FT_STATE_PROXY);
			if (cur & FT_STATE_TOMBSTONE)
				g->free_list[i].retired = false;
			/*
			 * The retire is fused back into the one
			 * {LOCK|s -> TOMBSTONE|s} record whenever the anchor IS
			 * @meta, which is always under per-node.  A member that
			 * deduped onto a word this op already holds owes no
			 * release and no registry entry: the acquire that first
			 * took it recorded both, and a second settles one word
			 * twice.
			 */
			/*
			 * THE MARK GOES TO THE TXN BEFORE THE RECORD IT COVERS,
			 * and for BOTH anchor shapes.
			 *
			 * The retire below is a record whose owner is @meta, and
			 * a record-time owner check reads the txn's locks[] and
			 * nothing else -- so a registry entry that arrives after
			 * it (or, where the anchor IS @meta, never) reads as a
			 * word the commit does not own.  Registering the anchor
			 * at the acquire with its terminal recorded afterwards is
			 * the shape ft_chain_compress_register_retire already
			 * uses for exactly these retires, and it gates on nothing
			 * but @shared.
			 *
			 * ★ THE FORMER `h.lock != meta` GATE WAS ABOUT SAFETY, NOT
			 * ABOUT THE REGISTRY.  A SURVIVING anchor MUST leave the
			 * sweep -- it is clean and re-lockable the instant the
			 * release settles, so a later "is it still locked?" reads
			 * a PEER's fresh mark and strips it -- while a mark whose
			 * terminal is @meta's own TOMBSTONE cannot be stolen that
			 * way, the acquire refusing a TOMBSTONE.  That makes the
			 * sweep merely SUFFICIENT there, not required, and the
			 * registry the answer to a question the sweep cannot be
			 * asked.  Under per-node the anchor IS @meta always, so
			 * the gate skipped the common case entirely.
			 *
			 * @holder_txn_owned follows the registration, both shapes:
			 * one owner per fence, and it is now the txn -- commit OK
			 * consumes the mark through its terminal, every other
			 * outcome drains locks[] (ft_flip_txn_lock_release_all),
			 * and ft_glue_clear_fenced must not clear it a second time.
			 * The registry is bounded by the FAN-OUT (257), which no
			 * glue free list approaches.
			 */
			if (!h.shared) {
				ft_flip_txn_lock_register(g->txn, h.lock,
					h.lock_snap);
				g->free_list[i].holder_txn_owned = true;
			}
			/*
			 * @meta takes the TOMBSTONE, its anchor the RELEASE.  The
			 * @h.lock == @meta arm consults no ctx, so the
			 * registration above cannot have moved which arm this
			 * takes.
			 */
			{
				struct ft_lock_ctx fctx;

				ft_glue_lock_ctx(g, &fctx);
				ft_flip_txn_record_retire_anchored(g->txn,
					&fctx, &h, meta);
			}
			/*
			 * A no-op where the anchor IS @meta (the fused terminal
			 * above is the whole story) -- ft_flip_txn_record_anchor_release
			 * returns early on that shape.
			 */
			if (!h.shared)
				ft_flip_txn_record_anchor_release(g->txn, &h,
					meta);
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
		 * PER EDGE (@live at the struct): "unreachable until the forward
		 * flip" is what licenses the plain store, and a src-origin edge is
		 * the one place that premise can be false.  A same-trie rekey FOLD
		 * re-parents the moved subtree's own children -- which stay
		 * READER-REACHABLE through the old source spine right up to the flip
		 * -- so storing one here would be a reader-visible mutation before
		 * the commit, and one that no abort rolls back.  Record those,
		 * exactly as the dst-origin arm does, so they flip atomically with
		 * the forward publish; store the hidden ones, in recorded order.
		 *
		 * ☠ THE TXN'S ARMING IS NOT THE QUESTION.  structural_sw says how a
		 * record is KINDED, never whether a reader can see the slot, so
		 * asking it here makes every armed committer record its HIDDEN edges
		 * -- and then _ft_publish_to_parent's parent-first check reads a
		 * back-pointer this commit has only recorded.
		 */
		if (g->deferred[i].live) {
			/*
			 * A live edge has nowhere to land but the commit, and only
			 * the fold reaches this arm -- @dst_origin edges are skipped
			 * above and every fold glue carries the caller's shared txn.
			 */
			assert(g->txn);
			/*
			 * THE SKIP-COMPRESSED QUESTION, SETTLED BY MEASUREMENT.
			 *
			 * The plain-store arm below is ORDER-DEPENDENT, and applies in
			 * recorded order on purpose: ft_set_parent's skip-compressed arm
			 * resolves its target through ft_skip_to_compressed, which reads
			 * a child back-pointer an EARLIER edge of this same loop just
			 * wrote.  MEASURED, do not re-derive: resolving every src-origin
			 * edge before vs. in-order, the divergence is EXACTLY the
			 * skip-compressed class and it is total -- ft_unit 12 diverged of
			 * 18 skip (125 edges), the FT_INV_MW rekey oracles 8 of 8 skip
			 * (21224 edges).  Every non-skip edge resolves to a constant.
			 *
			 * THIS arm inherits none of that, because it never stores: a
			 * record does not land until the flip, so every edge taking this
			 * arm resolves against one pristine state and the pre-loop and
			 * in-order answers are identical BY CONSTRUCTION.  The
			 * order-dependence belongs to the store path, not to the
			 * resolution.
			 *
			 * ☠ AND THE TWO ARMS DO NOT INTERLEAVE within one src pass, which
			 * is what keeps that split clean: @live on a src-origin edge means
			 * the FOLD, and the fold is a property of the GLUE, so a src pass
			 * is all-record or all-store.  A future builder that mixes them
			 * owes a re-measure -- a STORED edge whose skip resolution depends
			 * on a back-pointer a RECORDED edge carries resolves against the
			 * pre-loop value, which is the divergence above with the arms
			 * swapped.
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
				g->deferred[i].held_lock, /*hold_ctx=*/ NULL);
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
 *   - Hidden back-pointers (the drained payload + fresh cluster): tagged @live
 *     false, ft_glue_apply_deferred sets them immediately, in recorded order.
 *     Unreachable until the forward flip, so no atomicity is needed.
 *   - Live back-pointers (a node reachable until the forward publish -- via the
 *     OLD DST spine, tagged @dst_origin, or via the fold's not-yet-unlinked SRC
 *     spine) + the forward edge + the <=4 ordered-list cell edges: recorded into
 *     g->txn and committed with one selector flip.  The dst-origin half is
 *     recorded by the loop below, the src-origin half by apply_deferred.
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
	struct ft_pub_rec rec = { .n = 0, .mtxn = g->txn ? g->txn->mtxn : NULL };
	int i;
	enum urcu_txn_status cst;

	/*
	 * FOLD: take the lock acquire on every child the records below will SW-park
	 * into, BEFORE the first of them is recorded and before
	 * ft_glue_tombstone_free_list runs -- nothing of this commit has landed yet,
	 * so a contended child is a clean transient.
	 *
	 * The gate MIRRORS the acquire's own (@ft->lock_fine + armed): this one
	 * carries an assert on the ARMED STATE, not on the abort-returning path, so
	 * a condition the acquire no-ops on must no-op here too or the assert names
	 * states the acquire never sees.
	 *
	 * Only reachable under record_only, where @txn is the caller's and the
	 * caller's single commit is still ahead: ABORT here is a genuine bail, not a
	 * failure past the point of no return this function is otherwise specified
	 * to run at.  Asserted rather than assumed, because that is the property
	 * that makes returning ABORT from here legitimate.
	 *
	 * ☠ AN EXCLUSIVE TRIE IS NOT record_only AND MUST NOT ASSERT.  It arms
	 * @structural_sw through the constructor's `!lock_fine || exclusive` arm, so
	 * a FINE exclusive trie satisfies this gate on every glue txn -- graft
	 * attach, graft_swap and cds_ft_merge_at into an exclusive destination all
	 * arrive here with @record_only false.  The acquire has nothing to do for
	 * them (the mark arbitrates against a concurrent writer, which an exclusive
	 * trie does not have), so the gate excludes them rather than the assert
	 * admitting them: the assert keeps naming exactly the state it was written
	 * for, an ARMED committer whose bail must be clean.
	 */
	if (ft->lock_fine && g->txn && g->txn->structural_sw) {
		if (ft->exclusive) {
			ft_glue_assert_reparent_unheld(ft, g);
		} else {
			assert(g->record_only);
			if (ft_glue_acquire_reparent_marks(ft, g))
				return URCU_TXN_STATUS_ABORT;
		}
	}

	/*
	 * The src-origin half, dispatched PER EDGE on @live.  Hidden re-parents --
	 * nodes NOT reader-observable during the commit window (the drained
	 * payload + the fresh cluster) -- are set IMMEDIATELY with plain stores,
	 * in recorded order: no reader can reach these nodes until the forward
	 * flip below, so the stores need no atomicity, and the in-order
	 * application lets a skip top resolve its compressed node
	 * (ft_skip_to_compressed reads the child back-pointer a prior edge just
	 * wired).  The fold's src-origin edges are LIVE and are recorded there.
	 */
	ft_glue_apply_deferred(ft, g);
	/*
	 * The dst-origin half, live by construction -- a node still reachable via
	 * the OLD dst spine until the forward publish -- rides @txn so its flip is
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
				g->deferred[i].held_lock, /*hold_ctx=*/ NULL);
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
	{
		struct ft_lock_ctx gctx;

		ft_glue_lock_ctx(g, &gctx);
		/*
		 * A SHARED fence records NOTHING here: the earlier acquire owns
		 * both the {LOCK|s -> s} release and the registry entry, and the
		 * held arm below would settle the single word a second time.
		 * Routing it to the NULL arm instead would be worse -- that arm
		 * re-acquires or guards a word this op already holds.
		 */
		if (!g->publish_parent_shared && !g->publish_parent_txn_owned)
			ft_flip_txn_hold_or_lock_parent(ft, g->txn, &gctx,
				g->publish_parent, FT_DEPTH_FROM_DESCENT,
				g->publish_parent_holder,
				g->publish_parent_snap);
	}
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
		g->publish_parent_shared = false;
		g->publish_parent_snap = 0;
		g->publish_parent_txn_owned = false;
	}
	/*
	 * THE SECOND SLOT THE PUBLISH BELOW WRITES.  A compressed
	 * @publish_parent makes _ft_publish_to_parent re-encode the SKIP_X dual
	 * -- a slot in the compressed node's own parent -- and a caller that
	 * parks its records SW had to acquire that grandparent for the park to
	 * be legal.  Record the {LOCK|s -> s} RELEASE and register it, the same
	 * two lines the held arm of ft_flip_txn_hold_or_lock_parent runs, then
	 * DISOWN the field: the txn now carries the clear, so a commit consumes
	 * it and an abort CAS-clears it through ft_flip_txn_lock_release_all --
	 * and the caller's ft_glue_abort must not clear it a SECOND time (fence
	 * theft, the hazard @split_cn_holder spells out).
	 *
	 * A SHARED dedupe records neither: the acquire that FIRST took the word
	 * owns both, and settling it twice drops a word the op still writes
	 * under.  NULL holder (every caller but the fold, and the fold whenever
	 * the parent is plain or carries no dual) is byte-identical to before.
	 */
	if (g->publish_gp_holder) {
		if (!g->publish_gp_shared && !g->publish_gp_txn_owned) {
			/*
			 * SELF-GUARDING, for the reason the publish parent's own
			 * release above states: this anchor can be a word the SAME
			 * op RETIRES.  The grandparent of a compressed publish
			 * parent is the node ABOVE it, and when the op's other end
			 * is a root-level source junction that node is the ROOT --
			 * which the detach recompacts and therefore retires.  A
			 * release recorded beside that retire poisons the
			 * descriptor permanently.  One word, one terminal; the
			 * retire outranks.
			 */
			ft_flip_txn_lock_register(g->txn, g->publish_gp_holder,
				g->publish_gp_snap);
			ft_flip_txn_record_anchor_release_held(g->txn,
				g->publish_gp_holder);
		}
		g->publish_gp_holder = NULL;
		g->publish_gp_shared = false;
		g->publish_gp_snap = 0;
		g->publish_gp_txn_owned = false;
	}
	/*
	 * A recompaction already folded this publish into the node that
	 * replaced @publish_parent, so @publish_slot addresses the superseded
	 * copy: recording it here would install into a node nothing reads.
	 */
	if (g->txn && g->txn->pending_pub_folded &&
			g->publish_slot == g->txn->pending_pub_slot)
		goto publish_done;
	_ft_publish_to_parent(ft, g->publish_parent, g->publish_slot, g->top,
		ft_glue_publish_expected_old(g), &rec);
	ft_flip_txn_record_pub_rec(g->txn, &rec);
publish_done:
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
		/*
		 * The two words the acquire distinguished: @cn takes the
		 * TOMBSTONE, its anchor the RELEASE.  ft_flip_txn_record_retire_
		 * anchored fuses them into the single {LOCK|s -> TOMBSTONE|s}
		 * whenever the anchor IS @cn (always, under per-node), and
		 * ft_flip_txn_record_anchor_release is then a no-op -- so the
		 * default granularity records byte-identically to before.
		 */
		struct ft_held_anchor sh = {
			.lock = g->split_cn_holder,
			.lock_snap = g->split_cn_snap,
			.node_snap = g->split_cn_node_snap,
			.shared = g->split_cn_shared,
			.node_held = false,
		};

		/*
		 * THE REGISTRY OWNERSHIP belongs to the acquire that FIRST took the
		 * word, so a deduped member registers nothing -- an earlier member
		 * already did.  ★ REGISTER BEFORE THE RETIRE, not after it: the
		 * retire records @cn's own word and asks ft_flip_txn_owns who owns
		 * what it writes, so the registry has to name the holder ALREADY.
		 * Registering below the retire answered "not held" for a fence this
		 * op demonstrably holds -- it is retiring under it.
		 */
		if (!g->split_cn_shared)
			ft_flip_txn_lock_register(g->txn, g->split_cn_holder,
				g->split_cn_snap);
		{
			struct ft_lock_ctx dctx;

			ft_glue_lock_ctx(g, &dctx);
			ft_flip_txn_record_retire_anchored(g->txn, &dctx, &sh,
				g->split_cn_node);
		}
		/*
		 * The RELEASE half belongs to that same first acquire: a deduped
		 * member records none (ft_flip_txn_record_anchor_release asserts
		 * it).  It stays BELOW the retire, which is where it has always
		 * been -- at per-node granularity it is a no-op because the retire
		 * already settled @cn's own word, and under coarsening it lands on
		 * the ANCHOR, a different word entirely.
		 */
		if (!g->split_cn_shared)
			ft_flip_txn_record_anchor_release(g->txn, &sh,
				g->split_cn_node);
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
		g->split_cn_shared = false;
		g->split_cn_snap = 0;
		g->split_cn_node = NULL;
		g->split_cn_node_snap = 0;
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
	ft_ord_cell_record_into_ft(ft, g->txn, cedges, n_cedges);

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
		struct cds_ft_node *src_head,
		unsigned int holder_depth)
{
	assert(g->nr_splices < g->cap_splices);
	g->splices[g->nr_splices].holder_depth = holder_depth;
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
		struct cds_ft_metadata *hm, *anchor;
		struct ft_lock_ctx sctx;
		struct ft_held_anchor sh;
		unsigned int hd;
		bool held = false;

		assert(hf);
		hm = ft_flag_to_metadata(ft, hf);
		/*
		 * The holder was reached by walking a chain head's prev, so the
		 * descent's window is what dates it (§5.2: a head's holder IS
		 * the descent's parent).  One it never passed cannot be anchored
		 * here -- bail to the caller's re-descend, exactly as a
		 * contended holder does.
		 */
		ft_glue_lock_ctx(g, &sctx);
		hd = g->splices[i].holder_depth;
		anchor = ft_anchor_meta(ft, ft_lock_ctx_descent(&sctx), hf, hm,
			hd);
		/*
		 * Already fenced by the overlap-spine plan-lock: reuse it, take no
		 * second lock, and record no holder -- ft_glue_clear_fenced owns that
		 * fence's release.  See ft_glue_fence_holds for why re-marking it is
		 * a self-deadlock rather than a miss.
		 */
		if (ft_glue_fence_holds(g, anchor))
			continue;
		for (j = 0; j < i; j++) {
			if (g->splices[j].holder == anchor) {
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
		if (ft_acquire_member(ft, &sctx, hf, hm, hd, &sh))
			goto miss;
		/*
		 * DEDUPED ONTO A MARK THIS OP ALREADY HOLDS.  @shared is the
		 * acquire's "protected, owing no release and no terminal" answer,
		 * so the chain is excluded exactly as the ft_glue_fence_holds arm
		 * above is: take no second lock, record no holder, and leave the
		 * release to whichever registry owns the mark.
		 *
		 * Treating it as a MISS is a SELF-REFUSAL, not contention: the
		 * caller re-descends onto the identical shape and asks again
		 * forever.  ft_glue_fence_holds only scans the glue's FENCED free
		 * list, while the acquire dedupes against the whole held set (the
		 * txn's locks[], the frame chain, the glue), so a mark filed in any
		 * other registry reaches here as @shared.
		 */
		if (sh.shared)
			continue;
		g->splices[i].holder = sh.lock;
		g->splices[i].holder_snap = sh.lock_snap;
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
	struct ft_pub_rec rec = { .n = 0, .mtxn = txn ? txn->mtxn : NULL };
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
	{
		struct ft_lock_ctx gctx;

		ft_glue_lock_ctx(g, &gctx);
		ft_flip_txn_lock_or_guard_parent(ft, txn, &gctx,
			g->publish_parent, FT_DEPTH_FROM_DESCENT);
	}
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
	struct ft_pub_rec rec = { .n = 0, .mtxn = txn ? txn->mtxn : NULL };

	if (!run)
		return ft_glue_publish(ft, txn, g);
	/*
	 * VALIDATE (§4.B) / LOCK_FINE (step 6, §9.5): acquire publish_parent as a
	 * RELEASE lock (value-swap REPLACE survivor, guard-fallback on a miss); see
	 * ft_glue_txn_commit_edges for the full rationale.
	 */
	{
		struct ft_lock_ctx gctx;

		ft_glue_lock_ctx(g, &gctx);
		ft_flip_txn_lock_or_guard_parent(ft, txn, &gctx,
			g->publish_parent, FT_DEPTH_FROM_DESCENT);
	}
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

/*
 * COMPLETENESS IS A MACHINE CHECK, NOT AN AUDIT
 * (doc/design/ft-dlm-lock-coarseness.md §9).
 *
 * Anchoring is all-or-nothing: the moment ONE site locks a node directly while
 * the rest resolve to its anchor, a coarser spacing excludes nothing -- and a
 * mostly single-writer suite still reports green, which is a false green on the
 * one invariant the design rests on.  Worse, the two collide inside a SINGLE op:
 * the raw site takes a word the op already holds as some other member's anchor,
 * reads its own hold as contention, and retries into the identical shape.
 *
 * "Did we convert them all" is therefore answered by the BUILD.  Past this
 * point the raw primitives do not exist under the validate build; a new acquire
 * must come through ft_acquire_member or ft_dlm_acquire_set, which is where the
 * mapping and the dedupe live.  Both choke points are defined ABOVE, so they
 * keep their access.
 */
#ifdef FEATURE_FT_ANCHOR_VALIDATE
#define ft_meta_lock_acquire(...)	\
	FT_A_RAW_ACQUIRE_MUST_GO_THROUGH_ft_acquire_member
#define ft_dlm_lock(...)		\
	FT_A_RAW_ACQUIRE_MUST_GO_THROUGH_ft_dlm_acquire_set
#endif
