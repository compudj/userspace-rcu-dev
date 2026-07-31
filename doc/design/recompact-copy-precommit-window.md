# Recompaction body-copy in the pre-commit window — design note (2026-07-13)

Status: **ATTEMPTED AND REVERTED (2026-07-13).** The `FT_STATE_COPYING` fence is
load-bearing and cannot be retired; see "## Outcome" below. The rest of this note
is retained as the record of *why* the pre-commit-callback direction does not
work, so it is not re-attempted.
Supersedes the COPY_SLOT ("copy flag") mechanism of
`recompact-helpable-copy-records.md` (§2bis). Author of direction: Mathieu
Desnoyers.

## Outcome (2026-07-13): REVERTED — the tombstone claim cannot fence the plan window

Option 1 (defer the recompact body-copy into the pre-commit callback and retire
`FT_STATE_COPYING`) was implemented on `ft-mutation-node.h` / `ft-mutation-helpers.h`,
passed single-writer (ft_unit 275/275, ft_inv 58/58), then **regressed the clean
`d48ed267` MW baseline**: an 8×200 = 1600-run oracle soak produced 5/1600 failures
(2 teardown SIGSEGV on a wild back-pointer in `ft_detach_node`; 3 lost-key oracle
failures) versus 0/1600 at baseline. Both symptoms are one root cause: a corrupted
trie (a dropped child + a dangling back-edge) committed during the concurrent phase.

**Root cause — the plan→install window is unprotected.** The old `FT_STATE_COPYING`
mark is a *persistent* CAS on `old_node.state`, held from **before the build loop
through commit**. Any concurrent insertion that tries to `guard_parent(old_node)`
during that whole window computes clean-LIVE, mismatches the COPYING-set word, and
**aborts** — so the build/reparent phase reads a frozen source.

The tombstone claim cannot replace it, for a structural reason:

- It is a *record* — it only plants at **install**, never during the plan.
- Its expected-old is a *value* compare on `old_node.state`. A peer that
  re-anchors / splits / republishes `old_node.slot[b]` does so with a `{live→live}`
  **guard** on `old_node.state` (which does NOT change the state word's value) plus a
  CAS on the *slot*. If that peer commits and RELEASES in our plan→install gap, then
  when our claim finally plants `{build_state → TOMBSTONE}`, `old_node.state` still
  equals `build_state` (the released guard left it unchanged) → our claim **succeeds**
  even though `old_node.slot[b]` changed under us.

So the §4 precondition ("child-slot mutations serialize through the state claim")
gives mutual exclusion only *while both are planted*; a guard is transient (released
at the peer's commit). The reparent records guard each child's *back-pointer* and
*state*, and the claim guards the state *word* — but **nothing guards the forward
slot `old_node.slot[b]` itself** against a `{live→live}`-guarded republish that lands
in the gap. Empirically ~24% of oracle runs showed at least one `old_node` slot whose
build-time value differed from its commit-time value; most are benign skip
re-encodings (same resolved child), but the rare identity-changing ones corrupt
(lost key + wild edge). Confirmed by Mathieu's own observation: "the initial phase
where we iterate on the children nodes to reparent them is [not] protected
adequately; a concurrent insertion won't be caught before we begin the install phase."

**Why the callback then has no use.** With `FT_STATE_COPYING` kept, the fence
transitively freezes every child slot (incl. its skip encoding) from build through
commit, so the imperative build-phase copy already reads exactly the committed
values; a pre-commit re-read is a redundant no-op. The stale-encoding gap the
callback closed only opens *without* the fence. And the records/error-handling
simplification the callback was pitched to buy was already realized at `d48ed267`
(no COPY_SLOT records remain; `new_node` is thread-private until RELEASE, so the abort
path already frees immediately). Nothing was left for the callback to earn.

**Disposition.** Reverted `ft-mutation-node.h` / `ft-mutation-helpers.h` to
`d48ed267` (restoring `ft_meta_copying_mark` / `ft_flip_txn_record_tombstone_copying`
/ `ft_flip_txn_copying_register` and the abort-path `copying_clear`s; removing the
callback, `copy_ctx[]`, and the `_snapshot` helpers). The COPYING fence stays.

---

Below: the original (now-rejected) design rationale, retained for the record.

## Decision

Deprecate the **copy flag** (per-slot `COPY_SLOT` freeze records, and the
hand-rolled `FT_STATE_COPYING` reversible fence) in favor of performing the
`ft_node_recompact` body copy in the engine's **pre-commit callback**
(`urcu_txn_on_precommit` / `urcu_mcas_commit_precommit`). The node-level
**tombstone claim** on the retiring node's state word is the only fence
required; no per-source-slot proxy is installed.

## Why the copy flag existed, and why it no longer earns its keep

The per-slot `COPY_SLOT` record served two purposes under the *helping* MCAS
scheme:

1. **Freeze the source child slot** so the copy reads a coherent value, and
2. **Make the copy replayable by a helper** — a contending peer could drive the
   copy forward deterministically (identical input → identical bytes), so the
   transaction stayed lock-free even if the owner stalled.

The adopted engine (47a1a612) is **single-driver with no helping**
(rcu-mcas.h:14-18, "no helping, no stealing, no deadlock"; liveness comes from
age-escalation into a per-domain FIFO fair mutex, not from peers completing an
op). Purpose (2) therefore evaporates: **no peer ever replays the copy**, so the
copy need not be expressed as records at all — it can be plain imperative owner
code. Purpose (1) is subsumed by the node tombstone claim (below). With both
purposes gone, the per-slot proxy is pure cost (one record per child, plus the
`call_rcu`-defer-`N'`-on-abort obligation helping forced).

## Error handling: the copy flag's real cost

The copy flag was not just an extra record kind — it complicated the abort/error
path, and that complication was the helping model leaking into reclamation.

A COPY_SLOT is a CAS record `add`'ed before commit, whose side effect
`*dst = V` (publish into `N'`'s slot) fires **at install**, and — per its own
contract — "runs on EVERY driver that reaches an installed COPY_SLOT record
(planted by us or found installed)." So the moment the first COPY_SLOT installs,
a **lagging peer driver may still be storing into `N'`**. The fresh node is no
longer thread-private, so:

- the abort/error path can no longer `free_cds_ft_node_unpublished` immediately —
  it must `call_rcu`-**defer** the free of `N'` (and every fresh
  action-record destination) so reclamation waits out any driver still mid-store
  (`recompact-helpable-copy-records.md` §2bis "Reclamation under helping");
- with a **cutover rule**: immediate-free stays valid only for aborts *before*
  the first helpable record is planted; after that, every reachable immediate
  free must convert to the deferred path — an audit obligation per abort site;
- at a runtime cost: one extra `call_rcu` object **per aborted recompact**, which
  under heavy contention (frequent evict/abort) feeds the drain-starvation
  balloon already seen in the oracle.

The pre-commit callback removes every line of that. The fill runs **owner-only,
once, inside the frozen window** — there is no lagging driver, so `N'` stays
thread-private and the abort path keeps its immediate `free_cds_ft_node_unpublished`
unchanged. No `dst` install-time side effect, no positional-ordering special case
in the commit-time record sort, no immediate→deferred cutover, no per-abort
`call_rcu` object. The simpler error handling falls straight out of single-driver.

## What the pre-commit window provides

`urcu_txn_on_precommit` runs a callback in the one phase that is both
**owner-exclusive and unpublished**: every record planted, outcome
SUCCEEDED-bound, terminal status not yet released (rcu-txn.h:136-144). In that
window **every slot the transaction stores or guards is frozen** — a peer that
touched one has already lost its plant CAS — and the callback's plain stores
publish on the same RELEASE as the write set. This is exactly the documented use
case: "fill in a fresh node that one of its own edges is about to make
reachable, as ordinary owner code inside the commit."

So the recompact plants its edges (tombstone claim first), and in the pre-commit
callback fills `N'` from `N`. Because the claim proxy is planted, `N`'s state
word — and, by the precondition below, every one of `N`'s child slots — is
frozen for the duration of the fill.

## The load-bearing precondition (unchanged from the copy-flag design)

"The tombstone is sufficient" holds **iff every mutation of an `N` child slot
serializes through `N`'s state-word claim.** Once our claim proxy is planted, no
peer may change a child pointer under the copy. This is the same precondition the
COPY_SLOT design carried (`recompact-helpable-copy-records.md` §4), and its
enumeration is §6 of that doc:

- Point-op publish paths are class A (txn edge + holder `guard_parent` in the
  same commit) — a peer child-recompact that republishes `N.child[b]` guards `N`,
  so it contends the claim and aborts. **This is the AUDIT #1 boundary: it must
  be true that no `N`-child mutation exists that is not a claim-serialized txn.**
- The residual blockers were the lone-store family (C-1, B-1/C-2) and the bulk
  paths (excluded by contract). **Status to confirm before flipping the switch:**
  that the lone-store blockers are closed (C-1 landed @2189cce7 per the campaign
  ledger) so the precondition is met for concurrent insert/remove.

The pre-commit transition does **not** change this boundary — it changes *where
the fill runs and what fences it*, not *what serializes the source*. If a
child-slot writer that bypasses the claim still exists, neither the copy flag nor
the pre-commit callback is safe; that writer must be routed through the claim
either way.

## Target flow (before → after)

Before (`ft_node_recompact`, live-retire arm):
```
ft_meta_copying_mark(N)          # standalone CAS: set FT_STATE_COPYING
for each src child slot:         # BUILD PHASE copy
    resolve_prio(&N.child[b]) -> V
    set_nth(N', b, V)
record reparents / count / freeze
ft_flip_txn_copying_register(N)  # {COPYING|s -> TOMBSTONE|s} rides the commit
record forward-publish (grandparent: N -> N')
commit
  ... on any bail: ft_meta_copying_clear(N)   # reversible-fence unwind
```

After:
```
alloc N' empty
record N.state claim FIRST       # {LIVE -> TOMBSTONE} proxy = the fence
record reparents / count / freeze
record forward-publish (grandparent: N -> N')
urcu_txn_on_precommit(fill_Nprime, {N, N'})
commit_precommit
  # in the frozen pre-commit window, fill_Nprime copies N's slots into N'
```

Retired by the switch: the standalone `ft_meta_copying_mark` / `_clear` pair,
the `FT_STATE_COPYING` state bit and its guard-mask term (`~(TOMBSTONE|COPYING)`
collapses to `~TOMBSTONE`), and the ft-verify.h "leaked COPYING fence" check.
(Caveat: only where recompact is *fully* txn'd — any single-writer / bulk /
compact path still using `COPYING` must convert first, or the two schemes coexist
during the transition.)

## Status

- **Engine — done.** COPY_SLOT record kind dropped at 47a1a612; `resolve_prio`
  degraded to a committed read; pre-commit callback is the standard commit
  entry (`urcu_mcas_commit` == `commit_precommit(…, NULL, NULL)`).
- **FT — pending.** `ft_node_recompact` still marks `FT_STATE_COPYING` and copies
  in the build phase; it calls `on_precommit` nowhere.
- **Cleanup owed.** Stale `COPY_SLOT` references in ft-mutation-node.h comments
  (e.g. :1293) and the orphan unit tests `test_rcu_mcas_copy_slot.c` /
  `test_rcu_mcas_resolve_prio.c` (still in tests/unit/Makefile.am) test engine
  primitives that no longer exist — remove or retarget.

## Open items

1. Confirm the §4 precondition is met for concurrent insert/remove (C-1/B-1/C-2
   closed) so "the tombstone is sufficient" is sound at flip time.
2. `metadata->external_nodes` is a distinct downward edge recompact also copies —
   it needs the same claim-serialized treatment as the child slots (fold into the
   pre-commit fill).
3. Fill ordering vs. reparent records: the pre-commit fill writes `N'`'s slots;
   the reparent records re-home children's back-pointers. Confirm the fill runs
   before the forward-publish install (it does — pre-commit precedes RELEASE) and
   that reparents reference `N'` slots the fill has populated.
4. Bulk / compact / `FEATURE_FT_INSERT_IN_PLACE` paths still on `FT_STATE_COPYING`
   — scope them out or convert before removing the bit.
