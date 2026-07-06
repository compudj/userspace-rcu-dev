# Step 4 — concurrent (multi-writer) engine swap: implementation plan (2026-06-30)

The FT↔urcu-txn integration's final step: swap each op's commit body from the
single-writer flip-latch (`urcu_txn_sw`) to the concurrent MCAS engine
(`urcu_txn` over `urcu_mcas` / `urcu_txn_list`), wire Invariant-2's freeze
*validate*, and lift the writer-exclusion contract so per-trie writers run
concurrently. Grounded in `mcas-multiwriter-readiness.md` (§1, §3–§5, §7) and the
concurrent-engine headers; campaign-disciplined (each phase gate-green).

## 0. Foundation already in place (do not redo)

- **Invariant 1 (edge-expressibility): certified.** Every reader-visible
  structural / ordered edge commits through a `{slot, old, new}` flip descriptor
  edge — no bare `rcu_assign_pointer`. The commit *body* is therefore swappable.
- **Invariant 2 (freeze-on-free) MARK side: done + audited.** `FT_STATE_TOMBSTONE`
  (bit 1 of `cds_ft_metadata.state`, ~19 retire sites) and `CDS_FT_NODE_REMOVED_FLAG`
  (bit 0 of `cds_ft_node.next`, ~15 sites) are committed flip edges; the
  `FT_DEBUG_TOMBSTONE_AUDIT` build (asserts the mark on every published-node free)
  is green (ft_unit 254 / ft_inv 56). Refinement-1 precondition met (no
  reader-visible in-place node-word mutation survives).
- **Commit body today:** flip-latch — park tagged proxies, flip one selector word,
  settle, defer-free — safe *only* because the caller guarantees per-trie writer
  mutual exclusion. There is **no internal writer mutex**; exclusion is the
  caller's contract. `FEATURE_FT_EXCL_VALIDATE` is a runtime *overlap detector*
  (per-trie owner CAS at the API boundary; aborts the process on writer/writer
  overlap) — it ASSERTS the contract, it does not provide it.
- **Ordered-cell list** on the circular `urcu_txn_sw_list` sentinel; single-cell
  splice/unsplice/replace ride the public `_prepare` ops folded into FT's flip-txn.

## 1. End state

- Commit body = real MCAS: each slot CASed `old -> descriptor` in sorted address
  order, status word flipped, retried on contention (`urcu_txn` retry loop).
- Invariant-2 validate live: a writer publishing into node P `load_validate`s P's
  tombstone == LIVE; a duplicate append is `insert_after_guarded` on the tail's
  `next`; the freeze mark rides the SAME MCAS as the unlink (atomic detach).
- Contract: **concurrent per-trie writers.** Tier-1/2 lock-free; Tier-3 coarser
  (below). Caller no longer provides writer mutual exclusion.
- Readers nearly unchanged (RCU + descriptor resolution); the resolver gains MCAS
  proxy decode and a tag-space move.

## 2. The per-op shape (the retry loop)

Every mutator becomes (engine idiom, confirmed in `test_rcu_txn_list.c`):

```
urcu_txn_init(&txn, &ft->group->domain);        /* once */
do {
    urcu_txn_begin(&txn);                        /* opens RCU read section */
    /* re-read inputs each attempt (they may have moved) */
    ... = urcu_txn_load(&txn, slot)  /  urcu_txn_load_validate(&txn, guard);
    /* build fresh cluster INVISIBLY (allocs pre-secured, see §3.1) */
    urcu_txn_store(&txn, slot, old, new);        /* == today's flip edges, 1:1 */
    st = urcu_txn_commit_flavor(&txn, ft->group->flavor->update_call_rcu);
    urcu_txn_end(&txn);
} while (st == URCU_TXN_STATUS_ABORT);
/* OK(0)=done; ABORT(>0)=retried; MEMORY_ERROR(<0)=propagate */
```

What is INVARIANT across the swap (the leverage): the recorded edge set is the
SAME frozen `{slot, old, new}` set FT already builds; the engine's frozen-set +
distinct-slot contract matches `urcu_txn_sw`. So **per-op edge construction does
not change** — only the commit call, the surrounding loop, and the read calls.

Concrete wrapper change: `ft_flip_txn_commit` returns **void** today (the sw
engine never aborts) — it must become **status-returning** (`urcu_txn_status`) so
the loop can branch OK / ABORT / MEMORY_ERROR. The lone-edge helpers
(`ft_ord_cell_flip_one` etc.) likewise gain an abort outcome under the concurrent
engine (a single-record commit is a bare CAS that can fail → retry).

What CHANGES: the descend+build must become **idempotent / re-runnable** (no
side-effect outside the transacted slots before commit), because ABORT replays
the whole bracket. FT's build-invisible pattern (`PREP` then `===== COMMIT
(failure-free) =====`) is already this shape; the retry wraps descend+build+commit.

## 3. Hard problems and resolutions

### 3.1 Idempotent build + allocation lifetime
ABORT replays the bracket, so a fresh cluster allocated mid-bracket would leak or
be re-allocated each spin. Resolution: pre-secure nodes via the existing
`cds_ft_alloc_reserve` BEFORE `begin`, reuse the reservation across retries, and
free-unpublished only on terminal exit. The OOM model (`mcas-oom-grow-and-abort.md`):
prefix (alloc + record) abortable, tail (park→flip→settle) alloc-free — already
the contract. NOTE: the concurrent front-end already supplies the "reset/re-record
txn mode" the OOM doc flagged as missing (begin clears the write-set; `reserve` /
`min_alloc` avoid re-growing on retry).

### 3.2 Tag reconciliation (reader side)
Today FT parks its flip proxy as a **type-7 / `0xF`** tag (`FT_FLIP_PROXY_TAG`) in
node-pointer slots and resolves it with its OWN resolver (`ft_node_flip_proxy` →
`ft_flip_proxy_ptr` → `urcu_txn_sw_proxy_get`), supplied to the engine via the tag
callback (`ft_flip_txn_tag`). The engine stores that tagged value opaquely. So FT's
tag and the engine's internal bit-0 (`URCU_MCAS_TAG`) are NOT a flat collision —
FT resolves its own slots. The migration is primarily: **swap the resolution call**
`urcu_txn_sw_proxy_get` → `urcu_mcas_resolve` (read the MCAS status word instead of
the sw selector) while keeping the type-7 detection.

The open detail to settle in the POC (§7): does FT keep `0xF` (which has bit 0 set,
so the engine's own `urcu_mcas_resolve`/`is_proxy` would ALSO fire on it — must
verify `urcu_mcas_untag` recovers the record under the embedder's wider tag), or
move FT's type field to **bits 2–3** (engine owns bit 0, list owns bit 1, FT owns
2–3) for a clean separation. Either way this is the only substantive reader-side
change (§1 "readers nearly unchanged"); pick the variant that keeps the resolver a
single mask-compare on the hot path.

### 3.3 Invariant-2 validate (the §3.1 lost-key/UAF hazard)
- **Internal/compressed node:** a writer about to publish an edge into node P adds
  `urcu_txn_load_validate(&txn, &P->state)` (expected = LIVE, tombstone clear) so a
  concurrent remover that froze P aborts this commit. The inverse (insert lands
  first → remover's freeze-CAS fails → re-reads, abandons collapse) is automatic.
- **Duplicate-chain leaf:** the append uses the guarded form on `tail->next`
  (the word the appender CASes is the freed node's own `next`) so appending onto a
  dying tail aborts. `urcu_txn_conflict()` on a fired guard advances retry + keeps
  the FIFO turn (livelock-free).
- **Atomic detach:** fold each freeze mark into the SAME `urcu_txn` as the unlink
  (today they are separate lone-edge flips — "the bridge"). The node goes
  LIVE→DEAD atomically with removal, so a concurrent writer sees live-or-dead, never
  the torn window.
- **Live up-walk back-edges** (`meta->parent` / external `->prev` / `cell->parent`
  re-pointed on a *kept* node during a restructure): NO separate validate needed
  (SAFE-BY-FUSION). Each back-edge is fused as an edge into the same flip-txn as the
  forward publish, and its target (the new parent) is always a fresh build-invisible
  cluster node — unreachable to a concurrent remover, so there is no live target to
  freeze. The only live node in the commit is the forward-publish target, already
  covered by the internal-node guard above; the reanchor readers resolve the parked
  proxy (old-XOR-new atomically) and rely on fresh-cluster parents being wired before
  the cluster is reachable. Full argument + per-site inventory:
  `fractal-trie-review-2026-06/PHASE4.2_ATOMIC_DETACH_SCOPE.md` "BACK-EDGE(live)".

### 3.4 Two-commit cross-trie ops (§7)
Graft src-disappear, `cds_ft_merge_at`, the dual root swaps (and the phase-B
fix's drain) need a `synchronize_rcu` between an irrevocable commit1 and a
restartable commit2. Resolution (§7.2): commit1 irrevocable; commit2 a restartable
boundary MCAS over the now-exclusive payload — **reserve capacity from the payload
(invariant after commit1), re-record edge targets from dst per retry.** The
concurrent front-end's begin/end re-record bracket supplies the restart.

### 3.5 Cost tiers (§5)
- Tier 1 (point mutators) and Tier 2 (collapse/prune, depth-bounded freeze set):
  lock-free.
- Tier 3 (bulk detach / merge-unlink of a populated subtree, O(size) freeze set):
  the irreducible cost is **quiescing writers already inside** the subtree being
  made exclusive — needs a per-subtree "sealed" epoch in-flight writers check and
  back off on, or initial domain-escalated serialization. Producing structure ONTO
  an exclusive side is Tier-1 regardless of size; only the shared boundary publish
  is contended.

### 3.6 Exclusivity gating (§5.2) — preserve
On `ft->exclusive` BOTH invariants are vacuous: edges aren't reader-visible (plain
stores, no descriptor/MCAS) and no concurrent writer can race (no freeze, sync
free). Keep the exclusive fast paths; the MCAS + validate apply only on the shared
side. This is the build-invisible pattern extended to writers.

### 3.7 OOM vs ABORT, and the overlap detector
`MEMORY_ERROR` (<0) propagates as `CDS_FT_STATUS_MEMORY_ERROR`; `ABORT` (>0)
retries — distinct control flow. `FEATURE_FT_EXCL_VALIDATE`'s overlap-abort must be
disabled (or inverted to *expect* overlap) for the concurrent build, since
concurrent per-trie writers become legal.

### 3.8 `nr_keys` (order-statistics) — the count-atomicity resolution (DECIDED)
`nr_keys` (the subtree-total aggregate behind `count_keys`/`lookup_nth`/`skip`) is
the one count that doesn't fit the structural-MCAS model: an exact aggregate is a
serialization point, so exact-and-concurrent is impossible without spatial sharding.
Sharding (depth-modulo checkpoints / split counters) was explored and **shelved** —
the bulk ops change a moved subtree's depth, which would force O(size) checkpoint
re-alignment, breaking their boundary-only cheapness. Settled design instead:

- **Optional, per-group, default OFF** (mirrors `cds_ft_group_attr_set_ordered_list`;
  an immutable flag read on the hot path). ⇒ **the default config has ZERO count
  surface, so `nr_keys` is not a step-4 blocker at all.**
- **OFF (default):** `skip_forward`/`skip_reverse` → `n × next/prev` (O(n) with
  `ordered_list` on; O(n·depth) off — the two flags interact for skip's cost class,
  document it); `lookup_nth`/`_last` → iterate n from first/last; `count_keys`/
  `count_keys_prefix` → **full enumeration** (O(size)). No field, no propagation, no
  contention, no multi-writer hazard.
- **ON — undercount, concurrency preserved (NOT folded into the txn):** `nr_keys`
  stays a separate word; propagation becomes an atomic `uatomic_add` (today's plain
  `store(get + delta)` at `ft-mutation-helpers.h:2098` + the direct-store sites is the
  lost-update bug under concurrent writers; fresh/build-invisible inits stay plain,
  uncontended). Keep the undercount ordering (insert: commit-publish **then** `+`;
  remove: `−` **then** commit-detach), propagated **exactly once** per op, positioned
  around the retry loop (after-commit for insert; once-before-loop for remove —
  remove's commit is OOM-free so the single decrement never doubles or strands). The
  existing acquire/release undercount machinery is KEPT. Concurrency preserved: a
  fetch-add bounces a cache line, never aborts.
- **Rejected: fold-into-commit.** Exact/linearized, and it would dissolve the
  positioning rule + retire the undercount machinery — but the root `nr_keys` is in
  *every* op's word-set, so every writer conflicts/aborts at the root → `nr_keys`-on
  goes single-threaded, defeating step 4's concurrency. Exactness buys little anyway:
  rank/skip are inherently fuzzy under concurrent mutation.

## 4. Phased campaign (each gate-green)

> STATUS (2026-07-03): Phase 4.1 is **DONE** and the atomic-detach half of Phase 4.2
> is **DONE**. The validate half of Phase 4.2 (`load_validate` / guarded-append) is the
> current front — see §7 (updated). Do not re-plan 4.1 from this section as if pending.

**Phase 4.1 — engine swap under RETAINED caller exclusion (behavior-identical). ✅ DONE @ dd57e6b4 (2026-06-30).**
`ft_flip_txn` now embeds `struct urcu_mcas_txn` and `ft_flip_txn_commit` drives
`urcu_txn_commit_flavor` inside the `do {} while (st == URCU_TXN_STATUS_ABORT)` retry
loop (dormant: the retained caller exclusion means 0 contention → the loop runs once).
Zero `urcu_txn_sw_*` symbols remain under `src/fractal-trie/`. The tag reconciliation
(§3.2) landed as the per-record proxy tag (`33a5813d`) keeping FT's `FT_FLIP_PROXY_TAG`
(type-7 / `0xF`) resolved by FT's own resolver; the "type-7 → bits 2–3 move" variant was
NOT taken (FT resolves its own slots, no flat collision). Suite + ASAN green under exclusion.

**Phase 4.2 — Invariant-2 validate + atomic detach (still under exclusion).**
Two halves, tracked separately:
- **Atomic detach (fuse mark-into-unlink): ✅ DONE @ 4a069957.** Because `ft_flip_txn` IS
  the MCAS txn now, every fused freeze mark already commits as an atomic MCAS detach.
  All safe tombstone-relocation fusions landed; two residuals deferred (chain-leaf reader
  proxy; list-off force-txn) — see `fractal-trie-review-2026-06/PHASE4.2_ATOMIC_DETACH_SCOPE.md`.
- **Validate side (`load_validate` on edge-publish-into-live-P + guarded chain-append): ❌ NOT
  wired** (0 uses of `urcu_txn_load_validate` / `insert_after_guarded` in FT). This is the
  current work. Under exclusion the guards always pass → behavior-identical, existing gate green.
  (This is step 3's validate side, which could not land on the sw engine.)

**Phase 4.3 — enable concurrent writers (Tier-1/2).**
Relax `FEATURE_FT_EXCL_VALIDATE`; document the new concurrent-writer contract; wire
the `urcu_txn_domain` for fair escalation. NOW the guards/validates are
load-bearing. NEW validation: writer-vs-writer stress + the §3.1 race oracle (race
a remover-prune R against an inserter I on disjoint words; assert no lost key, no
UAF) + TSAN + concurrent ASAN.

**Phase 4.4 — Tier-3 bulk ops.**
Concurrent bulk detach/merge: the per-subtree seal / "becoming exclusive"
quiescing, or keep Tier-3 domain-serialized initially and lift later.

**Phase 4.5 — hardening.**
Bounded-progress / livelock test (mirror `test_rcu_txn_fallback`); LTTng
flight-recorder for residual races (per the project's RCA method); multi-writer
throughput benchmark vs the single-writer baseline.

## 5. Validation strategy

- **4.1 / 4.2:** existing reader-vs-writer suite (ft_unit 254, ft_inv 56) + ASAN +
  the config gate — proves behavior-identity under retained exclusion. (As with
  Invariant-1, green here cannot detect a *missed* concurrency hook, only a broken
  edge — so coverage is argued structurally, not by the suite.)
- **4.3+:** a NEW writer-vs-writer harness (N threads mutating one shared trie, key
  partitions overlapping); the §3.1 race oracle as a standing ft_inv test; TSAN
  (`--enable-compiler-atomic-builtins`); LTTng snapshot+abort for the hard cases.
- **Progress:** the fair-escalation bounded-retry test.

## 6. Risks / open questions

1. **Idempotent-build refactor scope** — every op's descend/build must be
   retry-safe; ops that mutate reserve/scratch state before commit need care.
2. **Tag-space move (type-7 → bits 2–3)** touches the resolver + every tag
   producer/consumer; high blast radius, must be a clean behavior-identical commit.
3. **Tier-3 seal primitive** — the irreducible "becoming exclusive" cost; may need
   a new per-subtree epoch the engine does not provide.
4. **Restartable commit2** for the two-commit cross-trie ops (reserve-from-payload
   + re-record) — needs `commit2_bound(payload)` and exercises the re-record mode.
5. **Reserve ↔ retry composition** — does `cds_ft_alloc_reserve` survive/reuse
   across ABORT replays cleanly?

## 7. Recommended first move

> OBSOLETE (2026-07-03): the Phase 4.1 engine swap already landed wholesale across
> ALL ops @ dd57e6b4 — not one-op-at-a-time, and the retry loop + tag reconciliation
> came with it. What follows is kept for history; the CURRENT first move is below.

~~**Phase 4.1 on ONE op — `cds_ft_insert`**~~ as a POC: retarget its commit to
`urcu_txn` under retained exclusion, wrap in the retry loop, do the tag-space move +
resolver retarget, validate gate-green. — SUPERSEDED: done for every op at once.

### CURRENT first move (2026-07-03): wire the Invariant-2 VALIDATE side (§3.3)
Under RETAINED exclusion (behavior-identical, guards always pass), add:
1. `urcu_txn_load_validate(&P->state, LIVE)` on each edge-publish INTO a pre-existing
   reader-visible node P (a concurrent remover that froze P then aborts this commit).
2. Guarded chain-append (`urcu_txn_list_insert_after_guarded_rcu`-analog) on `tail->next`
   for the duplicate-chain append, so appending onto a dying tail aborts.
See the scoping in `PHASE4.2_ATOMIC_DETACH_SCOPE.md` (validate-side inventory). This is the
last piece landable under exclusion; Phase 4.3 (relax `FEATURE_FT_EXCL_VALIDATE`) makes it
load-bearing.
