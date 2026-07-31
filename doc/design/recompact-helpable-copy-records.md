# Helpable recompaction via txn copy-records

Status: ENGINE BUILT (2026-07-08), recompact wiring in progress. Companion to
`mcas-multiwriter-readiness.md`. Author of idea: Mathieu Desnoyers. Captured
during the MW-hardening campaign (session 9e075fe0) as the intended fix for the
AUDIT #1 recompact copy-loop parked-latch defect.

NOTE (2026-07-08): the two-"action-record" design below (§2 memcpy +
copy_ptr_array) was SUPERSEDED. The engine building blocks actually built are a
single **COPY_SLOT** CAS-family record + a **priority-resolve** helper; see
"§2bis Final design" immediately after §2. §1 (problem) and §3/§4
(claim/reclamation) still hold. The action-record text is kept for provenance.

## 1. Problem (AUDIT #1, CERTAIN, permanent)

`ft_node_recompact` (ft-mutation-node.h ~1115-1186) rebuilds an internal node
`N` by copying its child slots into a fresh node `N'`, then re-homes the
surviving children and publishes `N'` at `N`'s grandparent slot, retiring `N`.

The copy loop reads each child slot **raw** (`get_ith_pos`, `if (!iter)
continue`) and copies `iter` verbatim. If a concurrent peer has a **parked
flip-proxy latch** (a type-7 proxy value) on one of `N`'s child slots, that
non-NULL latch is `memcpy`'d verbatim into `N'`. The reparent sweep skips
proxy-valued slots, so the copied latch is never repaired; `settle` only CASes
the **original** slot (rcu-mcas.h:737), never `N'`'s copy. After the peer's
descriptor completes and is GP-freed, `N'`'s slot still holds the latch, now
pointing into freed descriptor memory. Resolving it = wild type dispatch =>
SIGSEGV. This is **permanent**, not a window.

The audit's stopgap ("resolve-or-abort in the copy loop") forces a bad
tradeoff: *resolve* an undecided proxy gives old-or-new and can produce a torn
snapshot; *abort* on any parked proxy risks livelock under a steady stream of
peer latches.

## 2. Core idea — make the body copy first-class, helpable txn work

Express the recompaction body copy as **transaction records** so a contending
peer can **help drive it forward** (not only abort it), preserving the engine's
lock-free/wait-free progress guarantee. The copy work must therefore be
deterministic and replayable by any helper.

Two new **record kinds** (both "action records", not CAS records):

- **`memcpy` record** `{dst, src, len}` — copies an **immutable** memory region
  (the node header / key bytes). Idempotent: `src` never mutates, so every
  helper writes identical bytes.

- **`copy_ptr_array` record** `{dst[], src[], n}` — copies an array of child
  pointers; each element resolves **`proxy -> old_ptr`** when the source slot
  bears a flip-proxy tag, else copies the raw value. Idempotent *because the
  source is frozen for the duration of the copy* (see §4).

### Action-record semantics (new to the engine)

- **No live slot.** They are not CAS'd into any reader-reachable location; they
  are positional entries in the record sequence.
- **Scratch destination.** They write only into the fresh, still-unpublished
  node `N'`.
- **Not undone on abort.** Their side effect into `N'` is explicitly *outside*
  rollback — on abort `N'` is discarded, so there is nothing to undo. (Contrast
  a CAS record, whose live-slot install must be rolled back.)
- **Idempotent / replayable.** Any number of helpers may execute them
  concurrently; deterministic input (immutable / frozen src, deterministic
  proxy resolution) => identical output bytes.
- **Positionally anchored.** They must run *after* the status-claim record and
  *before* the forward-publish record. They carry no slot, so the engine's
  commit-time record sort (rcu-mcas.h ~1021, sorts by slot) must keep them in
  sequence rather than address-order.

## 2bis. Final design (BUILT 2026-07-08) — COPY_SLOT record + priority-resolve

The action-record idea above was dropped: an action record has no live slot, so
it cannot itself catch a peer that mutates a source child slot in the copy
window, and the positional-sort special-case complicates the engine. The built
design is smaller and needs no new record *class* — a COPY_SLOT is a CAS record.

### The AUDIT #1 latch is a concurrent CHILD recompact

The parked flip-proxy the copy loop meets on one of `N`'s child slots is a peer
**child recompact's forward-publish** (it CASes `N.child[b]`, the child's own
grandparent slot, from `child -> child'`). It does NOT touch `N.status`, so the
F2 node lock — which pins `N.status` — cannot see it. That is exactly the
gap: `N.status` is fenced, but a per-child-slot republish is not.

### COPY_SLOT record `{src, V, dst}` (rcu-mcas.h, committed 25e18335)

A CAS record with `old == new == V`: an identity edge that FREEZES `src`
(readers resolve to V; settle restores V on BOTH commit and abort, so `N` stays
traversable) PLUS a side effect — at install it publishes V into the fresh,
still-unpublished word `dst` (`N'`'s slot). It sorts by `src` address and
settles like the CAS it is; only the dst publish and the nr==1 fast-path
exclusion are specific to it. `urcu_txn_copy_slot` / `ft_flip_txn_record_copy_slot`
wrap it.

- **Window-catch = the read-set check.** V is the child the prep resolved `src`
  to; if a peer changes `src` in the prep->install window, install's
  `resolved != V` aborts, and the op re-resolves on retry. So the copy is never
  committed stale (contrast the reverted "read src at commit" idea, which would
  let the copy disagree with the separately-built reparent record for V).
- **dst must be RCU-safe.** The dst publish runs on ANY driver incl. a lagging
  helper, so it can land after the owner returned from commit — `N'` must be
  `call_rcu`-deferred PER ATTEMPT (commit AND abort), never reused. Same
  slot-lifetime precondition as `src` (rcu-txn.h). This IS the §4 reclamation
  rule; validated by test_rcu_mcas_copy_slot (stack dst -> ~40% torn snapshots;
  per-attempt GP-deferred -> 0).

### Priority-resolve (rcu-mcas.h, committed 4107a0fa)

Prep resolves each proxied child slot to a definite child V. `urcu_mcas_read`
only HELPS a foreign proxy; a stream of peer child-recompacts could keep it
busy. `urcu_mcas_resolve_prio` arbitrates by AGING PRIORITY (shared
`urcu_mcas_engage_foreign`: help a higher-priority owner, evict a lower one),
returning V, or CAP (help budget spent) -> the op aborts with `-EAGAIN` and
retries; the existing `urcu_txn_conflict` path ages the persistent handle, so
the next attempt outranks and evicts. `urcu_txn_resolve_prio` /
`ft_flip_txn_resolve_prio` wrap it (self = the reserved descriptor).

Note: prep-resolution correctness does NOT require the priority variant —
during the copy loop the recompact holds no proxies yet, and the *commit*-time
COPY_SLOT install already does the aging help/evict + window-catch, so plain
`urcu_mcas_read` would be correct too. `resolve_prio` is used because its
prep-time evict lets an aged recompact FREEZE the source before building `N'`,
avoiding the build-then-abort `N'` rebuild churn (which feeds the call_rcu
drain balloon) under sustained child-recompact contention.

### Reparent stays separate; the proxy-skip disappears

The child reparent stays per-child (`ft_reparent_record`, unchanged). Today it
SKIPS a proxy-valued child (leaving an orphaned back-pointer on the retired
node) — but once the copy loop resolves every slot to a definite V, `N'` holds
no proxies, so the sweep reparents ALL children. Resolving the copy closes the
orphaned-back-pointer half of AUDIT #1 for free.

### Recompact wiring (retire_txn arm only)

The build-invisible / cluster-leaf / no-txn arms copy nodes no peer publishes
into, so they keep the raw-read + proxy-bail (defensive; unreachable). For the
live-retire (`retire_txn`) arm, per surviving child slot at byte b:

1. `ft_flip_txn_resolve_prio(&N.child[b])` -> V (CAP -> `-EAGAIN`, abandon_fresh);
2. `set_nth(N', b, V)` builds the compact layout + plain-writes V;
3. `ft_flip_txn_record_copy_slot(&N.child[b], V, &N'.slot[b])` freezes src +
   records the helpable dst publish.

Reserve widens by `nr_child` (one COPY_SLOT per child) on top of the existing
`2*(nr_child+1)+1` reparent/back-channel budget. The claim stays the F2
`{LOCK|s -> TOMBSTONE|s}` record on `N.status`; commit = claim + N COPY_SLOTs
+ reparents + forward-publish, one helpable txn.

### Reclamation under helping — `N'` must be GP-deferred, not immediate-freed

Once helpers can execute our action records, the fresh destination `N'` is **no
longer thread-private**: a peer helper may still be writing into `N'` (running a
`memcpy` / `copy_ptr_array` record inside its RCU read-side section) at the
instant the owner — or an *evictor* — decides to abort. The current abort path's
**immediate** free of the fresh cluster (`free_cds_ft_node_unpublished` in
`ft_attach_node`'s `check_error` / the abort cb) would then reclaim memory a
helper is still touching => UAF. So on abort, `N'` (and every fresh node that is
an action-record destination) must be **`call_rcu`-deferred**, so reclamation
waits out any helper's read section.

This is a small, well-layered change — do NOT couple FT node ownership to the
RCU-txn descriptor's lifecycle. Two points make it precise:

1. **Ownership stays with the FT owner thread; only the timing changes.** The
   owner thread that allocated `N'` still owns it and still performs the free.
   The single change: on the abort path it **defers the free through `call_rcu`
   after settle** instead of freeing immediately — i.e. it calls the FT's
   existing GP-deferred reclaim (`free_cds_ft_node`, which `update_call_rcu`s)
   rather than the immediate `free_cds_ft_node_unpublished`. The grace period —
   not any descriptor coupling — is what waits out a helper still mid-record in
   its read section. "After settle" falls out naturally: the owner's abort
   cleanup (`check_error` / abort cb) already runs after `ft_flip_txn_commit`
   returns terminal, i.e. after the descriptor's drive-forward + settle have
   completed and its proxies are rolled back (so no *new* helper can engage `N'`;
   only in-flight ones remain, and `call_rcu` covers them). The engine keeps
   owning descriptors; the FT keeps owning nodes; the two reclamation paths stay
   independent.

2. **The immediate-free cutover is "before the claim is planted."** Immediate
   free stays valid only for aborts that occur **before** the status-claim proxy
   is planted (before the descriptor becomes helpable). This mirrors the current
   justification (`ft_attach_node` `check_error`: "all paths are before
   `ft_publish_to_parent` ... immediate-free is safe"). After the claim proxy is
   planted, the owner's abort free of any helpable-destination node must switch
   to the `call_rcu`-deferred path. Audit item when building: every immediate
   `free_cds_ft_node_unpublished` (+ abort cb) reachable *after* the first
   helpable record is planted converts to the deferred free.

Cost note: deferring `N'` on abort adds one object to the `call_rcu` backlog per
aborted recompact, which under heavy MW contention (frequent evict/abort) feeds
the drain-starvation balloon already seen in the oracle. Correctness is
unaffected; the retire *volume* under contention is worth watching (and is an
argument for keeping recompact aborts rare — priority/aging, not spin).

## 3. The status-word claim — one record, TOMBSTONE-first (no COPY flag)

Plant a **TOMBSTONE-valued flip proxy on `N.status` as the first record**, ahead
of the action records. A separate `FT_STATE_LOCK` flag is **not needed**: a
parked proxy already provides all three roles.

1. **Conflict signal.** A parked proxy on `N.status` makes any txn that touches
   that word help/evict per the normal MCAS priority protocol. The *presence*
   of the proxy is the announcement; no distinct COPY value is required.

2. **Reversible-until-commit fence, for free.** The proxy resolves to `old`
   (LIVE) for readers and rolls back byte-for-byte to the old word on abort —
   exactly the reversible fence `FT_STATE_LOCK` hand-rolled (set-then-clear),
   without an explicit clear path. Because the proxy claims the **whole** word,
   a concurrent `nr_child+-` on `N` is forced to contend too (correct: a child
   insert/remove during the copy is a true conflict), whereas LOCK-as-a-bit
   let `nr_child` mutate underneath.

3. **One-way death on commit.** Resolves to TOMBSTONE => `N` retired.

The **second status record** (`COPY -> TOMBSTONE`) that an earlier two-record
sketch used as the "copy complete, safe to publish" fence is unnecessary: that
fence is now **commit itself**. Every record — including the action records —
must reach DONE before the descriptor commits, so `N'`'s forward-publish cannot
become visible until the copy is finished. One commit point *is* the fence.
This also removes the same-word double-record hazard (the engine poisons a
descriptor that records one slot twice, rcu-mcas.h ~948-973).

### Record order in the descriptor

```
[0] N.status : LIVE -> TOMBSTONE     (proxy planted first = the claim)
[1..k] action records:               (positional, run after the claim)
        memcpy(N'.header, N.header)
        copy_ptr_array(N'.slots, N.slots, nr)   proxy -> old_ptr
    ... plus the reparent / count / freeze edges as today ...
[last] grandparent slot : N -> N'    (forward publish; visible only at commit)
```

### Bonus — retire `FT_STATE_LOCK`

On a fully txn'd recompact path the guard mask collapses from
`~(TOMBSTONE | LOCK)` to `~TOMBSTONE`; proxy contention handles the in-flight
case. Caveat: only where recompact is *fully* txn'd — any single-writer / bulk /
compact path still using LOCK must convert first, or the two schemes coexist
during the transition.

## 4. The load-bearing precondition (why determinism holds)

The TOMBSTONE claim makes `N`'s child slots **immutable for the duration of the
copy** — the same footing the `memcpy` src has by construction. Both record
kinds then reduce to "copy an immutable source", which is what makes them
replayable. **Determinism and lost-update-safety collapse into ONE precondition**
(both hold for the identical reason): every mutation of an `N` child slot
serializes through `N.status`, so once our proxy is planted nothing can change a
child pointer under the copy.

This is true **iff every child mutation actually routes through `N.status`**.
Two ways the premise breaks, both currently live:

1. **Blind raw store to `N.slot[j]`** — the lone-store family (`ft_ord_cell_flip_one`
   and callers, `mark_removed`, the direct-publish rec==NULL arms, `ft_set_parent`
   bare back-pointer stores, in-place body writes). Parks no proxy, touches no
   status word, slips *under* the claim: the copy reads it non-deterministically
   AND it is a lost update. For this scheme these are not a nit — they falsify
   the idempotency premise.

2. **An engine child-flip that omits `N.status` from its word-set** — parks a
   proxy on `N.slot[j]` but is free to settle it because nothing serializes it
   against the claim.

So the copy is idempotent **exactly to the extent that (1) is eliminated and (2)
is enforced** — a stronger requirement than "guard the parent on publish": *no*
mutation of `N`'s children may exist that is not a proxy'd, `N.status`-serialized
txn.

RCU-lifetime rider (free): resolving a peer's parked proxy dereferences the
peer's descriptor, so the whole recompact must run inside one read-side section
(it does), so the peer's `call_rcu` free cannot land mid-resolution.

## 5. Engine changes required

- Two new record kinds (`memcpy`, `copy_ptr_array`) with action-record
  semantics: no live slot, scratch dst, excluded from rollback, idempotent.
- Preserve positional order of action records through the commit-time record
  sort (they have no slot key).
- Drive-forward executes action records after the status-claim proxy is planted
  and before the forward-publish record installs.
- (FT side) route `ft_node_recompact` body copy through these records; plant the
  TOMBSTONE claim first; keep reparent/count/freeze/publish edges as today.

## 6. Enumeration result (2026-07-07) — precondition NOT met, small worklist

Full sweep of live internal-node child-slot writers (agent, HEAD @ea8cef0f).
Verdict: the point-op publish paths are already **class A** (txn edge + holder
`guard_parent` in the same commit) — split / attach in-place+relocation / remove
promote+detach-recompact / unchain / chain-compress / graft store-point / merge
all guard the holder. The recent commits are confirmed (ea8cef0f promoted the
attach grandparent republish C->A; 77c3d1c9 promote; b99639e6 attach in-place).
Requirement (2) holds for **all** insert/remove point-op paths.

Residual **live blockers** (must be emptied before the scheme is sound for
concurrent insert/remove):

- **C-1 (blind, LIVE, reachable in a plain trie today):** `ft_node_replace_ptr`
  non-`pub` arm — ft-mutation-node.h:769 (popcount) / :844 (pigeon) ->
  `ft_node_child_edge_flip` -> `ft_ord_cell_flip_one` (bare rcu_assign into a live
  child slot). Reached for external-promote when `ordered_list` OFF && `rank_stats`
  OFF (ft-remove.h:1525/3170/825). FIX: route through a guarded txn, or force
  `pub` non-NULL unconditionally at ft_detach_node (extend the rank_stats force at
  ft-remove.h:825 to also cover `!ordered_list`).
- **B-1 / C-2 (insert-replace external-chain-at-end, LIVE, unguarded):**
  ft-insert.h:3083-3180 — list-on `ft_ord_cell_swap_publish_multi` (:3157, txn'd
  but NO `guard_parent(d.pnf)`) and list-off `flip_try` n==1 (:3174, blind lone
  edge). The replace op also lacks a retry loop (ft-insert.h:3550). FIX: mirror
  the already-hardened head-replace (guard :3506/:3544): add
  `guard_parent(d.pnf)` + reserved txn on both commits.
- **C-3 (graft :655) + B-2 (compactor :208):** bulk paths — exclusion status
  RESOLVED (2026-07-07). Both stores are already under `CDS_FT_SCOPED_WRITER` at
  their entry (`ft_store_at_graft_point_commit` reached via `cds_ft_graft` ->
  `ft_graft_keylen` whose scope is at ft-graft.h:849; `ft_compact_relocate_compressed`
  reached via `cds_ft_compact` scope at ft-compact.h:613). `ft_excl_writer_enter`
  (fractal-trie-internal.h:1522) CASes `excl_owner` and **aborts on ANY concurrent
  writer from another thread** under `FEATURE_FT_EXCL_VALIDATE` — a blunt
  single-writer validator (a no-op when the feature is off). So:
  - In a `FEATURE_FT_EXCL_VALIDATE` build, a graft/compact that overlaps *any*
    other writer already aborts — this **is** the runtime exclusion assertion the
    caveat asked for. Run that build in CI to catch a contract violation.
  - It is all-or-nothing: it cannot express "point-ops concurrent among themselves,
    bulk exclusive." In the MW build (validator necessarily off, else concurrent
    point ops would abort) the bulk-vs-point exclusion is therefore an **unchecked
    app contract**: the app must quiesce point-op writers around a graft/compact.
  - A *targeted* MW-mode assertion (bulk excludes point, point-among-point allowed)
    would need a shared/exclusive writer lock (point = shared, bulk = exclusive).
    That is a design decision (open item §7), not required to build the scheme:
    the scheme only needs point-op insert/remove to satisfy the precondition, and
    bulk ops are excluded by contract + the single-writer validator.
- **C-4:** `ft_remove_commit_rec` lone `txn==NULL` arm (ft-mutation-helpers.h:2338)
  — currently unreachable (all callers pass a txn); transitional fallback, note only.
- **C-5:** `FEATURE_FT_INSERT_IN_PLACE` in-place `set_nth` stores are live blind
  child-slot writes in that (single-writer) build — the scheme must EXCLUDE that
  config or convert them.

Minimal prerequisite worklist for concurrent insert/remove: **close C-1 and
B-1/C-2** (both are independently real MW lost-update bugs), then **assert
exclusion for C-3/B-2**, then **scope out FEATURE_FT_INSERT_IN_PLACE**.

**Also (outside `copy_ptr_array` scope but on the recompacted node):**
`metadata->external_nodes` is a *distinct* live downward edge that recompact also
copies (ft-remove.h:2957/2597, ft-insert.h:532/2993/3039). The scheme must give
it the same claim-serialized treatment as the child slots (mostly guarded today,
but confirm) — a third record target or a rule folding it into the ptr-array.

## 7. Open items / verification

- Positional-record support vs. the slot-sort — engine surgery scope.
- **Shared/exclusive writer lock (optional MW-mode assertion).** To *runtime-assert*
  the bulk-vs-point exclusion in the MW build (point-op writers concurrent among
  themselves; bulk graft/compact/merge exclusive against all writers), the
  `FEATURE_FT_EXCL_VALIDATE` validator would need to become a shared/exclusive
  lock rather than a single-owner CAS: point ops take SHARED, bulk ops take
  EXCLUSIVE, and entry aborts on shared-while-exclusive / exclusive-while-shared.
  Not required to build the helpable-recompact scheme (bulk excluded by contract);
  decide if we want the CI-catchable assertion.
- Raw-vs-resolved sweep: every write-side reader of `N.status` must use
  `urcu_txn_load` (a raw read during the parked window sees a descriptor
  pointer). Same class as the raw `ft_meta_nr_child` reads on the audit ledger.
- Interaction with the existing `ft_reparent_record_meta` state-tag edge (it
  already records `N`'s children's `meta->state`; the child re-home and the
  parent claim are different words, but confirm no same-word collision).
