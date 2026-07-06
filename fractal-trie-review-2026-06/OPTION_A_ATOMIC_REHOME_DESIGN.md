# Option A — atomic `(parent, offset)` re-home + consistent-snapshot reads

Branch `ft-txn-integ`, Phase 4.3. Fixes the dominant `FT_INV_MW` crash:
`ft_attach_node`'s relocation publish faults in `ft_slot_to_byte` because the
`(parent, offset)` back-pointer of a live shared node is read **torn** while a
peer re-homes it (recompacts its grandparent).

## Why the crash is a torn pair, not a stale read

`metadata->parent` is published via `rcu_assign_pointer`; `parent_slot_offset`
lives in the MCAS `state` word. A re-home (`ft_set_parent`, `ft-helpers.h:2227`)
writes them as **two separate plain stores** — parent then offset. A concurrent
reader can observe (new parent, old offset) or (old parent, new offset). With
`ft_slot_to_byte(parent, base(parent)+offset)` the mismatched index runs off the
node → `i < nr_child` assert (`ft-lookup-node.h:182`), 12/16 oracle runs.

The split-style early guard (`ft_get_parent_slot(meta) != captured_slot`) does
**not** catch it: `ft_get_parent_slot` computes `ptr(parent) + offset*8`, and the
torn pair *coincidentally* reproduces the captured slot address (the offset was
minted for the OTHER parent's layout), so the guard passes and the fault lands
downstream.

## The engine already supports the fix (facts)

- A parent-field flip-proxy is a **type-7** tagged pointer (`FT_FLIP_PROXY_TAG`,
  low nibble `0xF`), distinct from real node types 0-6; `ft_resolve_flip_proxy`
  (ft-helpers.h:863) untags → `urcu_mcas_resolve_record(r)`.
- The `state`-word proxy is tagged `FT_STATE_PROXY` (bit 0); `ft_meta_*_load`
  resolve it via `urcu_mcas_read(&state, FT_STATE_PROXY)`.
- `urcu_mcas_resolve_record(r) = urcu_mcas_status(r->mcas)==SUCCEEDED ?
  r->new_ptr : r->old_ptr`. **Two co-committed edges share `r->mcas`.**
- `ft_flip_txn_record_reserved(txn, &meta->parent, old, new)` already parks a
  parent-field edge (used by graft; `ft-mutation-helpers.h:2815`).
- The up-walk reader `ft_get_parent_rcu` (ft-helpers.h:1313) and the iterator
  (`ft-iter.h:99/178`) **already** resolve the parent flip-proxy. Read-side
  up-walks are covered; the gaps are on the **write/build path**.

## The crux the map missed: a consistent PAIR read

A 2-edge MCAS commits `(parent, offset)` atomically, but resolving them in **two
separate** `urcu_mcas_read` calls still tears if the commit's status flip lands
*between* the two reads (read parent→old, flip, read offset→new). "Every reader
resolves consistently" therefore means a **single status snapshot** feeding both
edges, not a per-field wrap:

```
p_raw = rcu_dereference(meta->parent); s_raw = load(meta->state)
if p_raw is flip-proxy(0xF):
    r_p = untag(p_raw, 0xF);  t = r_p->mcas;  s = urcu_mcas_status(t)   // ONE read
    parent = s? r_p->new : r_p->old
    if s_raw is proxy(bit0) && untag(s_raw,bit0)->mcas == t:
        state = s? r_s->new : r_s->old        // SAME s
    else state = s_raw
    offset = pso(state)
else: parent = p_raw; offset = pso_load(meta)   // no re-home in flight
// ABA guard: re-read meta->parent; if changed, retry (bounded)
```

Because parent and offset are **always** co-committed in one txn, `p_raw` being a
proxy ⟺ `s_raw` is that txn's proxy; the `r_s->mcas==t` check + re-read closes the
txn-settle/new-txn ABA window.

## Three interlocking parts

1. **A.1 — consistent pair-resolver (read).** New helper
   `ft_resolve_parent_slot(meta, ft) -> {parent, slot}` (single snapshot, above).
   Route `ft_get_parent_slot` through it (today it reads `meta->parent` raw at
   1723/1732, offset resolved). Also the recompact-inherit
   (`ft-mutation-node.h:1008/1186` `new_meta->parent = meta->parent`) must inherit
   the resolved parent + matching offset from ONE snapshot. No-op when no proxy →
   single-writer behaviour-identical, individually gate-testable.

2. **A.2 — atomic re-home (write).** The live-shared re-home — the recompact
   reparent loop (`ft-mutation-node.h:1265-1294` → `ft_set_parent`) — must commit
   `{&meta->parent→new, &meta->state→new_pso}` as ONE 2-edge MCAS instead of the
   two plain stores. `ft_glue_record_back_edge` is the template but plain-stores
   the offset (safe only for build-invisible grafts); the recompact needs the
   offset edge IN the txn. Build-invisible writes (fresh nodes, graft clusters)
   stay plain — they have no concurrent reader.

3. **A.3 — use the snapshot (op).** `ft_attach_node` (and `ft_split_compressed_*`)
   must take the publish slot from `ft_get_parent_slot(node)` (the consistent
   current slot) AND inherit the matching parent from the **same** snapshot —
   instead of pairing a descent-captured `d.pnfp` with an inherit-time
   `meta->parent` read at a different instant. Then `ft_slot_to_byte(parent, slot)`
   cannot tear by construction; a genuine re-home makes the forward-CAS old-value
   stale → commit ABORT → retry.

## Sequencing & gate

A.1 first (no-op foundation, gate-safe) → A.3 (attach/split use the snapshot;
still inert until fields are consistent) → A.2 (atomic write activates the
resolver). Each increment: single-writer `ft_unit` + `ft_inv` behaviour-identical,
then the 6-config gate, then the `FT_INV_MW` oracle to watch the crash spectrum
drop. Best-effort skeptic gate + DCO sign-off per CLAUDE.md.

Scope note: this hardens the INSERT/recompact re-home. Remove/graft/merge re-homes
need the same treatment (their `_ft_publish_to_parent(meta->parent, ...)` reads),
but insert is the proving ground.
