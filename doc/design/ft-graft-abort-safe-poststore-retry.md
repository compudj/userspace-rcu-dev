# Abort-safe cross-trie graft: post-swap store retry (DESIGN, ready to implement)

Status: DESIGN 2026-07-16. Closes the LOST-GRAFT (rc=1) that the abort-safety
commit (1633579e) left behind: under FEATURE_FT_MW_LOCK_FINE_DROP the graft
store commit CAN abort (MCAS conflict with a peer), and the abort-safety fix
correctly stops freeing the old nodes on abort (no double-free) but the payload
is then orphaned -- detached from src by the swap, never published into dst.
This doc specifies the retry that re-attaches the payload to dst until the
commit succeeds.  All line numbers are against HEAD = 04b18ca5 (ft-txn-integ).

## Why a DST-side retry (not src rollback, not a fused txn)

After the src-root swap, src points at a fresh empty root and the payload
(graft_payload == old_src_root's subtree) is detached and OWNED by this writer.
src is EXCLUSIVE for a lock_fine cross-trie graft (BUSY_ERROR at ft-graft.h:964),
so it carries no readers and stays empty across retries with no reader-visible
flicker.  Re-publishing the payload into dst at `key` is therefore a
self-contained, retryable DST-side operation.  On a commit abort we re-descend
dst, re-prepare, re-commit -- src is never touched again.

Rejected alternatives: (a) undo the src swap + `goto retry_attach` -- the
list-on swap (ft_root_list_swap_publish) is hard to invert cleanly; (b) fuse
src-retire + dst-store into ONE cross-trie flip-txn (fully atomic, no orphan
window) -- the right long-term architecture but a much larger change; revisit
under the whole-trie mcas->sw migration.

## Mechanism: reuse the existing retry_attach loop, guard the src-side steps

ft_graft_keylen already has a `retry_attach:` loop (ft-graft.h:1219) that runs
descend -> prepare -> [src reserve] -> swap -> commit and, on a PRE-swap
failure, frees the invisible dst build + fresh_node and `goto retry_attach`.
Extend it: add `bool already_swapped = false;`, guard the SRC-side steps with
`!already_swapped`, and on a POST-swap commit abort free the failed dst products
and `goto retry_attach`.  The loop then re-runs ONLY the dst-side descend ->
prepare -> commit; src stays swapped/empty.

### Src-side steps to guard with `!already_swapped`

  1. fresh_node alloc (1225-1227): only pre-swap (src already has its fresh
     root on a retry).
  2. The 10 `free_cds_ft_node_unpublished(src_ft, fresh_node)` error-path frees
     at 1262, 1284, 1290, 1308, 1320, 1346, 1353, 1390, 1475, 1534.  On a retry
     fresh_node is PUBLISHED (src's live root) -- freeing it is a UAF.
     SIMPLEST FIX: set `fresh_node = NULL;` immediately after the swap and
     verify free_cds_ft_node_unpublished(ft, NULL) is a no-op (grep its body in
     ft-helpers.h; if it derefs, add `if (fresh_node)` at each site or gate the
     block on `!already_swapped`).  With a NULL-safe free, all 10 sites become
     no-ops post-swap with ONE change.
  3. src_retire_txn pre-reserve (~1320-1360): src-side; the retire already
     committed at the first swap.  Guard so a retry does not re-reserve/leak it.
  4. old_src_root capture (1561), graft_run capture (ft_ord_first/last ~1583),
     and the SWAP itself (ft_root_list_swap_publish 1595 / ft_root_edge_flip
     1615): guard the whole block with `!already_swapped`; set
     `already_swapped = true; fresh_node = NULL;` at its end.

### NOT guarded (re-run every attempt, dst-side)

  ft_glue_init (1234) + glue.txn create/reserve (1252-1265; on a retry
  ft_flip_txn_take(pre_txn) returns NULL since pre_txn was consumed, so a fresh
  txn is created), ft_graft_build (1278), the self-secure reserve (1365-1393),
  ft_store_at_graft_point_prepare (1447), the Fix-A publish fence (1480-1560),
  run_arg setup, and the commit (1672 GLUE / 1710 store).  The RCU read-side pin
  (whole-op bracket in cds_ft_graft, commit 79f59ded) already spans the whole
  loop, so re-descents stay pinned.

## Commit-abort cleanup (the delicate part -- shape-dependent)

On `store_cst != URCU_TXN_STATUS_OK`, free the failed attempt's UNPUBLISHED new
products before `goto retry_attach` (the aborted flip-txn rolled back its edges
but did NOT free the nodes the build/prepare allocated):

  * GLUE arm (prep == FT_GRAFT_PREP_GLUE): `ft_glue_abort(dst_ft, &glue)` frees
    the whole diverge cluster -- every fresh node was ft_glue_track'd
    (ft-mutation-helpers.h:3702).  glue.txn is already NULL (the commit consumed
    it).  DO NOT ft_glue_free_old here (those are the OLD live nodes; the
    abort-safety gate already skipped them).
  * NOSPLIT store arm: `ft_glue_abort(dst_ft, st.glue)` frees the displaced
    branch (ft_build_branch tracks it) and any glue cluster.  THEN, if the
    recompact RELOCATED {p} (st.old_recompacted_node != NULL), st.dest is the
    unpublished relocated {p}' -- NOT tracked in glue->built (ft_node_recompact
    allocates it directly) -- so free it explicitly:
        if (st.old_recompacted_node)
            free_cds_ft_node_unpublished(dst_ft, ft_node_ptr(st.dest));
    IN-PLACE recompact (st.old_recompacted_node == NULL) leaves st.dest == the
    LIVE d->pnf -- do NOT free it.
  * Re-zero per-attempt state before looping: st = {0} (or re-init), reset
    nosplit_prepared = false, self_secured = false, and destroy any per-attempt
    txns not consumed by the commit (run_splice_txn).  Do NOT touch
    src_retire_txn (consumed) or fresh_node (NULLed).

VERIFY each of the above against ASAN (heap-use-after-free catches a wrong free;
`detect_leaks=1` catches a missed free) -- FT arena nodes are ASAN-invisible, so
also run the FT_DEBUG_DOUBLE_FREE detector
(project_ft_barrier_uaf_is_graft_double_free) to catch a double-free of an arena
node, and add a per-range nr_live watch if a leak is suspected.

## Edge cases

  * POST-swap POPULATED (a peer inserted `key` between our swap and a retry's
    re-descend): impossible with the oracle's disjoint keys.  For general
    correctness the payload is detached and `key` now exists -- there is no
    clean rc; treat as a hard error (assert in debug) or, better, fold into the
    fused-txn architecture later.  Document the limitation.
  * POST-swap OOM (glue.txn create or reserve fill fails on a retry): the
    self-secure reserve pre-covers the recompact copy; a small-alloc OOM should
    `goto retry_attach` (transient), NOT return (src is consumed).  ~~Bound the
    spin with a retry counter -> best-effort.~~

    > ★ **THE "bound the spin -> best-effort" HALF IS REFUTED (2026-07-31) AND
    > WAS NEVER IMPLEMENTED.  DO NOT IMPLEMENT IT.**  It fails the CLAUDE.md
    > abort-boundary gate on both conditions, and the sentence refutes itself:
    > it says "NOT return (src is consumed)", and a bounded spin's only
    > terminal action *is* that forbidden return.
    >
    > At the give-up point the source root has ALREADY been reader-visibly
    > replaced -- `ft_root_edge_flip(src_ft, &src_ft->root, old_src_root,
    > fresh_node)` (ft-graft.h:1999) is an `rcu_assign_pointer`
    > (ft-mutation-helpers.h:1377), and `already_swapped` is set right after.
    > The whole subtree is then referenced ONLY by the local `graft_payload`,
    > so giving up drops it with no reference anywhere -- not leaked-reachable,
    > lost.  The status returned would be `CDS_FT_STATUS_MEMORY_ERROR`, which
    > is byte-identical to the PRE-swap OOM returns that mean "both tries
    > pristine": `enum cds_ft_status` has no enumerator able to express
    > "source consumed, destination not updated".
    >
    > This is not hypothetical for an exclusive source only.  The exclusive-src
    > gate (ft-graft.h:1140) deliberately EXEMPTS `src_ft == dst_ft`, which
    > `cds_ft_merge_at`'s whole-source rekey reaches with a LIVE trie -- so the
    > victim can be a live user trie left permanently empty.
    >
    > **The code today is UNBOUNDED and that is the safer of the two**: no
    > retry counter exists anywhere in `src/fractal-trie/` (verified by grep),
    > and the three post-swap sites (ft-graft.h:1514, :1543, :1714) say
    > `/* src consumed: OOM is transient */`.  An unbounded spin can hang; the
    > bounded give-up silently destroys data.  Note the spin sits inside
    > `cds_ft_graft`'s whole-op read section (ft-graft.h:2294-2296), so it also
    > blocks the grace periods that would replenish the arena -- i.e. the "OOM
    > is transient" premise is itself questionable there.  Fixing this properly
    > needs either an explicit rollback of the source root or a distinct
    > status; adding the counter alone would be a strict regression.
  * skip_conflict / prepare -EAGAIN on a retry: already `goto retry_attach`;
    with the guards this simply re-descends (the normal retry path).

## Termination / progress

Each commit abort means a peer committed a conflicting flip (made progress);
each retry re-descends the now-current tree and re-competes for {p}'s COPYING
lock, whose try-or-bail guarantees a winner per round.  The retry loop is inside
the whole-op RCU read section, so it delays grace periods for its duration --
bounded because it terminates; if a livelock is ever observed, that is a
separate bug (skip_conflict + COPYING-winner should prevent it), not a reason to
release the pin mid-loop (which would reopen the ABA the pin closes).

## Validation plan

  1. Build drop-ASAN + FT_DEBUG_DOUBLE_FREE; run inv_concurrent_crosstrie_fine_lock
     (FT_INV_MW=1) x30: expect rc=1 -> rc=0 (lost graft closed), UAF=0,
     double-free=0, no leak (detect_leaks=1).
  2. Non-drop ft_inv full suite: 0 fails (the retry is drop-only behaviourally;
     under the FT-wide lock the commit never aborts so the retry never fires).
  3. Point-op gate 1600/1600 @16w unchanged.
  4. Adversarial skeptic on the cleanup: refute that any abort path frees a
     LIVE/published node or leaks a new node, for each shape.

## Open follow-ups this does NOT address

  * The RCU-pin campaign residual: merge detach fall-through + retro-pin Fix A's
    C-fence/P-lock (project_ft_barrier_uaf_is_graft_double_free).
  * The fused src-retire + dst-store single cross-trie flip-txn (removes the
    orphan window entirely) -- the eventual clean architecture.
