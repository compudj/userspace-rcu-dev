# Defect C fix design: recompact must inherit the graft descent's parent, not the node's stale back-pointer

Status: PROPOSED (for review). Root-caused via LTTng 2026-07-16.
Scope: `FEATURE_FT_MW_LOCK_FINE_DROP` cross-trie graft only. Point ops unaffected.

## 1. Root cause (recap from the trace)

Under the FT-wide-lock drop, `inv_concurrent_crosstrie_fine_lock` aborts in
`ft_popcount_node_get_ith_pos` (`i < nr_child`) inside the NOSPLIT graft
commit's relocation arm (`ft-graft.h:661`). LTTng flight-recorder proof:

- `gp` = `0x7F9DDB802809` is the **shared spine node** all 16 writers graft into.
- Writer A: `node_recompact gp -> gp'`, republishes `gp'` into gp's grandparent,
  then `item_free(gp)` (call_rcu-deferred; the dst is non-exclusive).
- ~11 ms + a grace period later, grafting writer B recompacts its graft-point
  `{p}` (a **child of gp**) and publishes `st->dest` into `{p}`'s resolved parent
  = the **reclaimed gp** -> rank-45 index into a 0-child reclaimed node -> OOB /
  wild store (arena corruption in a release build).

The stale parent originates in the recompact of `{p}`:

    // ft-mutation-node.h:1143 (ft_node_recompact)
    struct cds_ft_inode_flag **inh_slot =
        ft_resolve_parent_slot(metadata /* = {p} */, ft, &inh_parent);
    new_metadata->parent = inh_parent;            // = freed gp
    ft_meta_parent_slot_offset_set(new_metadata, offset(inh_slot, inh_parent));
    if (ft->lock_fine && fenced && inh_parent &&
        ft_copying_lock_member(ft_flag_to_metadata(ft, inh_parent), ...))  // "locks" freed gp
        return -EAGAIN;

`ft_resolve_parent_slot({p})` reads `{p}->meta->parent`, a **back-pointer that
was never updated** when gp was recompacted->gp' (child back-pointers reanchor
lazily via the parked proxy; readers follow the proxy). Once gp is *freed*,
there is no proxy to follow — the read returns the dangling gp, and
`ft_copying_lock_member` on gp's reclaimed/zeroed word false-succeeds (no
TOMBSTONE bit survives reuse), so the "lock" does not reject it. Then FIX 2 in
the commit resolves `st->dest->parent` (= inherited gp) and publishes into it.

The FT-wide lock serialized spine-recompact vs. graft, so gp was never reclaimed
while `{p}` still referenced it; the drop removes that serialization.

## 2. The correct parent is already in the descent

`ft_store_at_graft_point_prepare` receives `struct ft_descent *d`. The recompacted
node is `dest = d->pnf = {p}` (both graft arms, `ft-graft.h:450` and `:516`).
The descent is produced by the **reanchoring** writer descent
(`project_ft_writer_reanchor_shared_primitive`, landed @d48ed267), so its captured
nodes are live. By the pointer-shift in `ft_descent_traverse_compressed`
(`ppnf<-pnf`, `pnf<-nf`, `pnfp<-nfp`):

- `d->ppnf` = `{p}`'s **live** parent = **gp'** (the reanchored replacement).
- `d->pnfp` = the slot in `d->ppnf` from which `{p}` was read.

So `(d->ppnf, d->pnfp)` is exactly the coherent (parent, slot) pair that
`ft_resolve_parent_slot({p})` *should* have returned — coherent by construction
(captured in one descent step), and live (reanchored). `d->skip_conflict` flags
the one case where a mid-descent reanchor moved the level, making the pair
unreliable.

## 3. The fix

Thread the descent's `(parent, slot)` as an **optional override** from the graft
into the recompact, used in place of `ft_resolve_parent_slot()` for `inh_parent`
/ `inh_slot`. `NULL` override = today's behaviour (insert/remove/merge unchanged).

### 3.1 New optional hint

    struct ft_parent_hint {
        struct cds_ft_inode_flag *parent;   /* d->ppnf; NULL => publish into &ft->root */
        struct cds_ft_inode_flag **slot;    /* d->pnfp (a slot inside @parent, or &ft->root) */
    };

### 3.2 `ft_node_recompact` (ft-mutation-node.h:958, use site :1143)

Add a trailing param `const struct ft_parent_hint *inh_hint`:

    struct cds_ft_inode_flag *inh_parent;
    struct cds_ft_inode_flag **inh_slot;
    if (inh_hint) {                          /* graft: coherent, reanchored pair */
        inh_parent = inh_hint->parent;
        inh_slot   = inh_hint->slot;
    } else {                                 /* insert/remove/merge: unchanged */
        inh_slot = ft_resolve_parent_slot(metadata, ft, &inh_parent);
    }
    new_metadata->parent = inh_parent;
    ft_meta_parent_slot_offset_set(new_metadata, inh_parent ?
        (unsigned int)((char *)inh_slot - (char *)ft_node_ptr(inh_parent))/sizeof(void*) : 0);
    /* lock inh_parent exactly as today (NULL => root, no lock) */

The `inh_parent == NULL` (root child) path is already handled (no lock, root-slot
CAS auto-guards). The hint degrades to `{NULL, &ft->root}` for a root-child `{p}`.

### 3.3 `ft_node_set_nth_rec` (ft-mutation-node.h:1904)

Add the same trailing `const struct ft_parent_hint *inh_hint` and forward it to
its three `ft_node_recompact(...)` calls (ADD_NEXT/ADD_SAME/DEL). The in-place
(`_ft_node_set_nth` ret == 0) arm never recompacts, so the hint is only consulted
on the recompact path.

### 3.4 Callers

- `ft-graft.h:461` and `:532` (NOSPLIT prepare, both arms): pass
  `&(struct ft_parent_hint){ .parent = d->ppnf, .slot = d->pnfp }`.
- `ft-insert.h:1605`, `ft-mutation-node.h:2008` (`ft_node_set_nth` wrapper),
  any merge caller: pass `NULL`.

### 3.5 skip_conflict guard (graft)

The graft does **not** currently check `d->skip_conflict` (only insert does,
`ft-insert.h:2519/2566`). Add, in `cds_ft_graft` right after the descent (before
`ft_store_at_graft_point_prepare`):

    if (caa_unlikely(d.skip_conflict)) { ...abort build, free fresh... goto retry_attach; }

so the override is only used when `(d->ppnf, d->pnfp)` is level-coherent. (This
also hardens the existing `st->pnf = d->pnf` / `d->pnfp` uses.)

## 4. Correctness

- **No UAF:** `inh_parent = gp'` is the live reanchored node. `ft_copying_lock_member(gp')`
  succeeds if gp' is live; if gp' was itself retired since the descent (TOMBSTONE
  set by freeze-on-free *before* free), the mark rejects it -> `-EAGAIN` ->
  re-descend — the same try-or-bail contract the point-op gate validated. The old
  code locked a *freed* gp whose reclaimed word no longer carries TOMBSTONE, so
  the reject never fired.
- **Correct offset/publish:** offset = `(d->pnfp - ft_node_ptr(gp'))/8` = `{p}`'s
  index in gp'. FIX 2's commit then resolves `st->dest->parent` (now gp') and
  publishes `st->dest` into gp' at `{p}`'s slot — a same-slot value swap
  (`{p} -> st->dest`), gp' `nr_child` unchanged.
- **Self-healing:** the retired `{p}` (with the stale back-pointer) is replaced by
  `st->dest` whose `parent = gp'` is correct, so the structure converges.
- **No livelock:** a "validate `{p}->parent == d->ppnf` else bail" alternative
  would spin forever (the stale back-pointer never changes without a recompact);
  the override *performs* the recompact with the correct parent, healing it.
- **No new lock contention:** the recompact already locks `inh_parent`; the change
  only corrects *which* node (live gp' vs. freed gp), not whether a lock is taken.

## 5. Scope & risk

- Behaviour-identical for `inh_hint == NULL` (insert/remove/merge): the point-op
  §11.4 gate (1600/1600) and non-fine builds are untouched.
- Signature churn: `ft_node_recompact` (+3 internal calls) and `ft_node_set_nth_rec`
  (+its callers) gain one trailing pointer param; all non-graft callers pass NULL.
- Latent siblings (audit, not in this fix): (a) insert/remove recompact use the
  same `ft_resolve_parent_slot(node)` and could hit the same stale-back-pointer
  under enough shared-parent contention — the point-op gate has not exercised it;
  (b) the graft **displaced-external** arm (`ft-graft.h:492`) and the **merge**
  spine builders resolve parents independently and may need the same treatment;
  (c) Defect D (GLUE commit not failure-free under drop) is orthogonal.

## 6. Open questions for review

1. Scope the hint to the graft only (proposed), or make the recompact *always*
   prefer a caller-supplied coherent (parent, slot) and migrate insert/remove to
   pass their descent pair too (larger, closes the latent sibling but re-touches
   the 1600/1600-validated point-op path)?
2. Is a small `struct ft_parent_hint` acceptable, or prefer two trailing params
   `(inh_parent_hint, inh_slot_hint)` to avoid a new type?
3. Should the `skip_conflict` graft guard land as its own bisectable commit first
   (it is arguably a pre-existing graft gap independent of the drop)?
