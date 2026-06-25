# MCAS OOM handling — grow-and-abort, never a bare-store fallback (2026-06-25)

Status: DESIGN OF RECORD. Decided in discussion with Mathieu. Captures the
allocation-failure model for the flip-latch commit under the multi-writer
(MCAS) goal, and the conversion plan for the two sites that currently violate
it. No implementation yet.

Related: `doc/design/mcas-multiwriter-readiness.md` (Invariant 1 / Invariant 2),
`doc/design/transactional-flip-latch.md` (the single-writer flip-latch).

---

## 1. The principle — no `rcu_assign_pointer`, OOM included

Under the MCAS endgame a reader-visible commit edge is a `{slot, old, new}`
descriptor: a multi-writer sorts the slots it touches and CASes each
`old -> descriptor` in address order. A slot published by a bare
`rcu_assign_pointer` is **outside that set** — a racing writer never sees it as
contended, its compare never fails, and you get a lost update plus the
disjoint-word / freeze-on-free hazard Invariant 2 exists to close.

Therefore a bare store **cannot be "mitigated later" by the MCAS transactions**
— it is a hole *in* the very mechanism that mitigates everything else. An OOM
path that degrades to a bare store is the one place every other guarantee
silently lapses, and memory pressure is exactly when a correctness cliff is
least affordable.

> **Rule: in no case is `rcu_assign_pointer` the right tool in a mutator — not
> even on an OOM fallback.** Post-campaign every reader-visible store is either a
> plain store (target still build-invisible) or a flip-latch descriptor edge
> (target live). `rcu_assign_pointer` fits neither.

## 2. The model — the boundary is the *first side-effect*

A commit is structured as:

```
[  grow / record  |  allocate everything commit needs  ]   <-- prefix (invisible)
---------------------------------------------------------   <-- POINT OF NO RETURN
[  park proxies  ->  flip selector  ->  settle          ]   <-- side-effect tail
```

- The **prefix** — building fresh structure, recording `{slot, old, new}` edges
  into the txn, *and any allocation commit itself performs* (proxies, group,
  selector) — is entirely reader-invisible. Recording an edge appends to a list;
  it touches no live slot. So an allocation failure anywhere in the prefix is
  handled by **abort**: free the partial txn, return a memory-error status, leave
  the structure byte-for-byte untouched. The "clean-failure point" is not an
  instant before the build — it is the whole prefix.
- The **side-effect tail** — from the first live-slot mutation (first proxy
  parked / first store) through flip and settle — must be **allocation-free**, so
  it cannot fail. Every byte it touches was acquired in the prefix.

So the invariant is precisely:

> **All allocation precedes the first side-effect; the side-effect tail
> (park -> flip -> settle) allocates nothing.**

Note this is *weaker* than "commit allocates nothing" — commit may allocate, and
may OOM, **as long as it does so before its first side-effect** (Mathieu's
refinement). It also means the implementation requirement is only an *ordering*
one: commit must be `allocate-all, then apply-all`, never an alloc interleaved
into the park/flip/settle tail.

A welcome consequence: for single-commit ops there is **no need to compute a
worst-case edge bound up front**. The txn just grows until commit or OOM.

## 3. The two carve-outs

1. **Inherently two-commit cross-trie ops** (graft src-disappear, `cds_ft_merge_at`;
   see mcas-multiwriter-readiness.md §7). They require a `synchronize_rcu`
   between an src-unlink commit and a dst-publish commit. Once the *first* commit
   is public there is no clean abort for the op as a whole, so the *second*
   commit's allocation must complete **before the first commit's side-effects go
   live**. This is the sole place genuine pre-reservation (not grow-and-abort)
   survives. (It is an op-level atomicity constraint, not an allocation-
   infallibility one.)

   **MCAS resolution (mcas-multiwriter-readiness.md §7.2):** commit1 is irrevocable
   and commit2 is a *restartable* boundary MCAS over the now-exclusive payload --
   **reserve capacity from the payload (invariant after commit1), re-record edge
   targets from dst per retry**. Pre-reservation supplies the *capacity* (no OOM,
   ever); the restart supplies *progress under contention* (re-descend + re-record,
   never undo). The capacity bound is dst-independent (`commit2_bound(payload)`); the
   one missing piece is a reset/re-record txn mode (today: record-once-freeze-install).

2. **Ops contractually forbidden from failing.** If an op may not return a
   memory error at all, even its prefix cannot be allowed to OOM, forcing
   inline/stack storage so the prefix cannot fail. **There are none in the
   remove/insert family** — see §4.

## 4. The contract already says this (remove family)

`include/urcu/fractal-trie.h` documents `cds_ft_remove_all` (and `cds_ft_remove`
returns "a negative cds_ft_status on error"):

> `CDS_FT_STATUS_MEMORY_ERROR` reports an allocation failure while restructuring
> the trie around the removed key: **the key was NOT removed** (the chain is
> still reachable; `*result_node` is NULL) and **the call may be retried**. An
> allocation failure while pruning an already-emptied internal holder is **NOT an
> error**: the key's removal is published before the holder is pruned, so the
> removal has already succeeded (`CDS_FT_STATUS_OK`). Completing the prune is
> deferred to a later mutation through that slot.

This paragraph is the §2 model, specialized to remove:

- **before** the key-removal commit (build / restructure): OOM => abort =>
  `MEMORY_ERROR`, key not removed, retriable — the prefix;
- the removal commit: the linearization point — the first side-effect;
- **after** it (pruning the emptied holder): OOM => *not* an error, removal
  already `OK`, prune **deferred** (it does nothing — it does not bare-store a
  degraded result) — a benign best-effort tail.

A remove already allocates (recompaction needs a fresh node; head promotion needs
a fresh write-once cell), so it is *already* allowed to ENOMEM. The carve-out #2
concern dissolves: the right model for the remove family is grow-and-abort, not
inline storage.

The precedent is in the code: `ft-remove.h` `cds_ft_remove` calls
`ft_detach_node`, which **returns** a status, and the caller rolls back:

```c
ret = ft_detach_node(ft, head_slot, ..., pubp, NULL);
if (ret)
    ft_propagate_external_count_parent(ft, holder_flag, 1);  /* undo the -1 */
else
    ft_node_mark_removed(node);
```

The `-1` count propagation is a rollbackable write-side pre-side-effect, not a
published edge. This is exactly grow-and-abort, already working.

## 5. The two current violators and their conversion

Found by grepping `rcu_assign_pointer` across the mutation modules (the cheap
completeness probe: every survivor must be plain-store-on-hidden or txn-on-live;
a bare store on a live commit is the smell).

### 5a. `ft-mutation-helpers.h:589` — `ft_ord_cell_flip` degraded fallback

```c
t = ft_flip_txn_create_bounded(n);
if (caa_unlikely(!t)) {
    for (i = 0; i < n; i++)
        rcu_assign_pointer(*edges[i].slot, edges[i].new_target);  /* <-- bare */
    return;
}
```

`create_bounded` fails **before any record is applied**, so no side-effect has
occurred. The comment already notes only point-op splices (<= 3 edges) can reach
the fallback (the merge interleave pre-reserves its txn in the fallible build
phase and never lands here), and for a point op the flip is the op's sole commit
(verified e.g. `ft-remove.h:1087`, where `_ft_publish_to_parent(..., &rec)` only
*records* and the flip is the first apply). So abort is clean.

Two conversion options (`ft_ord_cell_flip` is a shared primitive with ~24
callers, so the surface matters):

- **(a) Make it fallible + propagate.** Return a status; on `create_bounded`
  failure return the error instead of bare-storing; thread the failure up through
  the splice helpers to the op, which aborts. Uniform with §4's `ft_detach_node`
  pattern; larger propagation surface.
- **(b) Make the bounded txn allocation-free (inline/stack).** For the bounded
  point-op case (<= N edges) the `urcu_flip_txn` (already a "bounded inline-chunk,
  `sizeof % 16 == 0`, aligned(16)" structure) lives in caller-provided stack
  storage, so `create_bounded` *cannot* fail and the fallback ceases to exist —
  no propagation, the function stays `void`. Requires confirming what
  `create_bounded` allocates beyond the edge array (proxy nodes, group/selector)
  and whether those fit inline for the bounded count. The unbounded paths (merge
  interleave) keep their heap txn pre-reserved in the build phase.

Recommendation: **(b) where the edge count is statically bounded, (a) where it is
not** — which mirrors §2 (inline where bounded, grow-fallibly where not).

### 5b. `ft-remove.h:1033` — `ft_promote_head` in-place degrade

```c
new_cell_flag = old_cell ? ft_ord_cell_alloc(ft, next_node, old_cell->parent) : NULL;
...
} else {  /* list off, or cell-alloc OOM: in-place publish (degraded) */
    next_node->prev = node->prev;
    if (old_cell)                                   /* OOM: retarget in place */
        rcu_assign_pointer(old_cell->node, next_node);  /* <-- bare, breaks write-once */
    ...
}
```

The cell alloc (1008-9) is at the very top, before any side-effect. The
head-promotion callers (`ft-remove.h:1270, 1314`) are the *duplicates-remain*
cases: the key count is unchanged (no `-1` to roll back) and nothing is published
before the call. So on cell-alloc OOM the promotion aborts with zero rollback.

Conversion: make `ft_promote_head` and `ft_unchain_node` return a status
(currently `void`); on cell-alloc OOM return `-ENOMEM`; `cds_ft_remove` maps it
to `CDS_FT_STATUS_MEMORY_ERROR` (rolling back any count propagation, mirroring the
`ft_detach_node` path). The list-off branch is unaffected (`old_cell == NULL`, no
alloc, no bare retarget — a genuine build-invisible inherit).

## 6. Related findings from the same grep pass (separate from OOM)

- **Two genuine Invariant-1 live-publish gaps** — `ft-remove.h:1532` and `:1620`,
  the list-OFF remove-direction `external_nodes -> NULL` stores. They are the
  remove-direction twins of `dbc6858d` (which converted the insert-direction
  NULL -> node); the list-ON sibling already fuses via `ft_remove_one_commit`.
  Fix: a 1-edge `ft_ord_cell_flip {slot, old = chain_head, new = NULL}`.
- **Dead code** — `ft-helpers.h:1731` `ft_update_skip_pointer` has no callers
  (only a comment in `ft-remove.h:1267` names it). Remove it.
- Everything else the grep surfaced is accounted for: flip-latch internals (the
  settle store / proxy machinery — that *is* the descriptor commit), build-
  invisible / NULL-reserve writes (which by the §1 rule should arguably be plain
  stores rather than `rcu_assign_pointer`, a separate over-fencing cleanup), and
  out-of-scope compaction relocation (`ft-compact.h`, a separate MCAS-scoping
  decision).

## 7. Order of work

1. Land the two live gaps (`1532`/`1620`) and drop the dead function — mechanical,
   behavior-identical under one writer.
2. Convert `ft_promote_head` / `ft_unchain_node` to fallible (§5b) — small, mirrors
   the existing `ft_detach_node` pattern.
3. Convert `ft_ord_cell_flip` (§5a) via (b)/(a) — the larger piece; settles the
   inline-vs-propagate question per call site.
4. Re-grep `rcu_assign_pointer` over the mutation modules => only flip-latch
   internals and (optionally demoted) build-invisible plain stores remain.
