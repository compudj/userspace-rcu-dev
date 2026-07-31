# FT-wide-lock DROP mechanics — under MCAS, incremental

Status: **DROP LANDED + SOAKED, point-ops AND cross-trie (2026-07-18).** Extends
`mw-writer-lock-escalation-model.md` §11; the immediate prerequisite the
whole-trie sw migration is gated on (`whole-trie-mcas-to-sw-migration.md` §2).
Working note (untracked).

> **⚡ CURRENT STATE (2026-07-18), read this first.** The drop is **all-at-once
> for FINE and it holds for cross-trie too.** §§0–7 (the point-op story) are
> current and were gate-passed (§4: 1600/1600 @16w + ASAN). §8 records the
> *first* cross-trie finding and recommended **Option 1 (keep the FT-wide lock
> for cross-trie)** — **that recommendation is SUPERSEDED:** Mathieu chose
> **Option 2 (rearchitect the cross-trie attach to be contention-tolerant)**,
> and it is **done and validated** — see the new **§9**. graft, merge
> (whole-source + sub-position), nil-key, and graft_swap are all drop-safe under
> concurrent shared-spine contention; a rank-stats trie is coerced to COARSE (it
> always serialises on the root count anyway). Remaining to certify the default
> flip: re-run the §4 point-op soak on current HEAD, then flip FINE→drop.

**Decision (Mathieu, 2026-07-15):** drop the FT-wide `writer_lock` on a FINE
trie **under MCAS first**, as its own bisectable step — *not* coupled with the
sw cutover. Residual §11.1 under-count sites stay MCAS-abort-safe here; full
lock-set completeness + §8.3 become mandatory only at the later sw cutover,
where the MCAS safety net is removed (`whole-trie-mcas-to-sw-migration.md`
§8.3 KEY FINDING). This resolves the escalation-model §8.3 vs scope-doc §8.3
tension: both are true; sequencing is the question, and it is MCAS-first.

Memory: `[[project_ft_mw_lock_escalation_pivot]]`,
`[[project_ft_mcas_to_sw_migration_scope]]`,
`[[project_ft_step6_crosstrie_under_locks]]`.

---

## 0. What "the drop" is

Through step 6 (HEAD `fcd93a5c`) a FINE trie takes **two** things per op: the
per-node lock-sets (COPYING try-locks, steps 3–6) **and** the FT-wide
`cds_fair_mutex` `writer_lock` (`ft_writer_lock_scope_enter`, gated on
`ft->lock_mode`). The FT-wide lock serialises every writer, so the per-node
locks are *exercised but never contended* (the DEAD-PATH note: their bail
unwinds are reachable only by fault injection).

The drop makes a FINE trie **skip the FT-wide mutex**, leaving the per-node
lock-sets as the sole writer↔writer exclusion. COARSE keeps the mutex (it
derives no lock-set — §10.5).

## 1. The mechanism — one branch

The entire runtime change is a skip in `ft_writer_lock_scope_enter`, gated
behind `FEATURE_FT_MW_LOCK_FINE_DROP` (default off ⇒ shipping behaviour
byte-identical, reversible per §11.2):

```c
#ifdef FEATURE_FT_MW_LOCK_FINE_DROP
    if (ft->lock_fine)
        return;   /* rely on per-node lock-sets; no FT-wide mutex */
#endif
```

Placed after the reentrancy and exclusive tests. Everything else falls out:

- **`ft_writer_lock_scope_exit` no-ops.** It keys its release off the TLS
  `ft_wlock_held` *identity*, which the skipping enter never sets, so
  `ft_wlock_held != ft` ⇒ nothing to release. (Same reason the exclusive-skip
  and mid-scope `make_exclusive`/`make_concurrent` flips can't unbalance it.)
- **`ft_writer_lock_gp_wait` degrades to a plain `synchronize_rcu`.** Its
  `held = ft_wlock_held` is NULL (nothing held) ⇒ the drop/retake bracket is
  skipped and it is just the GP wait. **Bonus:** after the drop, a mid-op GP no
  longer needs the lock drop/retake at all — a COPYING bit is non-blocking, so
  the "no blocking lock across a GP" constraint (pivot constraint 2) is vacuous
  for FINE. The bracket stays only for COARSE.

The drop is **all-at-once for FINE**, not "domain by domain": the FT-wide lock
sits at the *writer scope* (one per op), not per op-domain, and a recompact
runs *inside* an insert/remove under that one scope — there is no per-domain
sub-scope to drop independently. The per-domain structure was the *conversion*
(steps 3–6, done); the *drop* is a single step. Net-(A) below is why one step
is enough. (Per-domain drop would need an op-type-gated scope enter and buys
only bisection granularity net-(A) already makes unnecessary; offered as an
option in §6 if the caution is wanted.)

## 2. Why it is correct under MCAS — two safety nets

### Net (A): the drop can only ADD exclusion, and MCAS arbitrates the rest

The pre-pivot lock-free-MW tree (`d48ed267`) reached **0/1600 @16w** on **pure
MCAS**, no locks, with the reanchor / skip-resolver / PSO fixes that are **still
in the tree** (deleted only at §11.6, after the default flips). So concurrent
writers on MCAS-alone were *already correct*. The per-node COPYING try-locks sit
**on top**. Dropping the FT-wide lock returns FINE to "MCAS + extra try-locks",
which can only differ from the proven MCAS baseline in three ways — all closed:

1. **Deadlock?** No. The per-node COPYING locks are **try-or-bail**
   (acquire, or `-EAGAIN` + re-descend — they never block). No hold-and-wait ⇒
   no cycle. The *only* blocking primitive was the FT-wide mutex, now gone ⇒
   FINE post-drop has **no blocking lock at all** ⇒ deadlock-free by
   construction. (This is also why the GP drop/retake becomes unnecessary.)

2. **Lost update at an under-count site?** No — under MCAS. The three
   §11.1-flagged residuals (below the lock-set) all write via **CAS or an MCAS
   record with expected-old**, never a raw store, so a concurrent write is a
   *spurious abort/retry*, never a lost update:
   - **I-1 in-place `nr_child` / §8.3 shared state word** —
     `ft_meta_nr_child_inc` (`fractal-trie-internal.h:995`) is a **CAS loop**
     whose comment is on-point: a spine node "two writers share, so a plain +=
     both LOSES increments… The CAS loop re-derives from the current word and
     waits out a parked proxy." It even spins on a peer's parked
     `FT_STATE_PROXY` — i.e. it already handles a concurrent re-home on the
     **same** state word §8.3 warns about. Re-home itself CASes
     (`ft_meta_state_transition`). Word-sharing is a spurious retry here, a lost
     update only under sw plain-stores.
   - **I-4b skip-dual `P`** — the under-count is an *unguarded*
     `record_reserved`, but `record_reserved` **is** an MCAS record; two writers
     on that slot ⇒ one commits, the other's expected-old mismatches ⇒ abort +
     retry.

   These sites were written contention-safe by the lock-free-MW campaign; the
   FT-wide lock only made them *uncontended*. The drop contends them again —
   which is precisely what 0/1600@16w certified.

3. **A bail path with a broken abort boundary?** This is the **real risk
   surface.** The per-node lock bail/unwind paths (`ft_copying_lock_member`
   miss → re-descend; publish guard-fallback) were only ever *fault-injected*,
   never *naturally raced* (DEAD-PATH note). The drop makes them **live**. A
   byte-for-byte-abort violation (a node left COPYING-marked, an orphaned child,
   a half-published edge, a leaked reservation) would surface only now. Net (A)
   cannot argue this away — it is **what the soak validates** (`cds_ft_verify`
   is a leaked-COPYING-fence detector; ASAN catches the UAF/leak). See §4.

### Net (B): per-node lock-set completeness — DEFERRED to the sw cutover

If the lock-sets were *complete* (every written slot owned by a held lock), the
drop would be correct by mutual exclusion alone, needing no net (A). They are
not (the I-1/I-4b/§8.3 residuals). Under **sw** net (A) vanishes (plain stores,
no expected-old) ⇒ those residuals become lost updates ⇒ completeness + §8.3
become **mandatory**. That is the sw-cutover workstream
(`whole-trie-mcas-to-sw-migration.md` §4.A/§8.3), not this step.

**Thesis:** the drop is correct under MCAS via net (A) — it adds exclusion and
exposes fault-injected bail paths, both bounded — with net (B) deferred to the
cutover where net (A) no longer holds.

## 3. Per-op-domain audit (post-drop, under MCAS)

Each op-domain relies on: its per-node lock-set for exclusion where complete,
and net (A) (MCAS CAS/record) for the residual. None introduces a blocking wait.

- **recompact `{C,P}(+GP)`** — COPYING fence *is* C's lock; miss ⇒ re-descend
  (no fallback, body was copied). P/GP resolved via the RELEASE terminal.
  Complete; contention now real, arbitrated by the try-lock.
- **insert `{P}`/`{P,CN}`** — publish-into node converted to a RELEASE lock with
  **guard-fallback** on a miss (value-swap target ⇒ guard suffices). Residual:
  I-1 in-place `nr_child` (CAS, net A), I-4b skip-dual P (record, net A).
- **remove** — plan → all-or-none acquire → edit → commit climb; `{L}`
  dup-chain serialises same-key. Detach recompact carried by recompact's lock.
- **cross-trie graft/merge/graft_swap** — **exclusive-src contract** (step 6):
  src is private ⇒ skips its scope anyway; only the *live dst* took the FT-wide
  lock, and now skips it too. §8 found this **NOT drop-safe as first written**
  (the FT-wide lock was masking several single-writer assumptions in the
  attach). **★ NOW DROP-SAFE via the §9 rearchitecture** (Option 2): a
  thread-local reserve, a coherent descent-sourced recompact parent, a fenced
  compressed-split, and a plan→commit retry on every contention abort. Rank-stats
  is the one exception — coerced to COARSE (§9.5). Net (A) still does not cover
  the reserve/spine-staleness class; §9 closes it structurally, not via MCAS.

## 4. Validation

Prototype = the §1 branch, `FEATURE_FT_MW_LOCK_FINE_DROP`. Positive liveness
confirmed (one-shot marker fired from the FINE branch — not a dead path).

- **ft_unit, drop on:** 281/281 (plain) and 281/281 under **ASAN**
  (`detect_leaks=1`), no UAF/leak. The FINE lock tests
  (`test_writer_lock_mode_fine{,_split,_graft,_crosstrie_busy}`) run
  single-threaded on per-node locks alone — green.
- **FINE MW disjoint oracle (`inv_concurrent_writers_fine_lock`, 16 writers),
  drop on:** **formal §11.4 gate PASSED — 1600/1600 @16w, 0 fail** (~119 min
  wall), plus **ASAN 40/40 clean** (no UAF/leak). The oracle checks every
  writer's shadow set, `count_keys`, and `cds_ft_verify` (a leaked-COPYING-fence
  detector — the exact failure a broken bail path produces). This is the same
  `0/1600` acceptance number every prior pivot step (2–6) had to hold.

## 5. What the soak covers vs what remains argued (honest gaps)

- **Covered:** concurrent disjoint-range insert/remove at 16 writers — the hot
  path, with recompact churn contending `{C,P}` spine locks and the bail paths
  now live. Plain + ASAN.
- **Now soaked — FAILED then RESOLVED (§9):**
  1. **Concurrent cross-trie** (graft/merge/graft_swap into a live dst) — the
     `inv_concurrent_crosstrie_*_fine_lock` oracles. Found real drop-exposed
     defects (§8); all fixed via the §9 rearchitecture. All four cross-trie
     oracles now pass under the drop + ASAN (§9.6).
- **Argued only (net A), not soaked:**
  2. **Same-key / shared** contention (`inv_concurrent_writers_shared`). Its
     status as an MW *target* is itself open (`[[project_ft_mw_transition_status]]`
     — ft_promote_head crashes). Not a drop regression if it was never a clean
     baseline; to be confirmed against the pre-drop FINE baseline before reading
     any shared-oracle result as a drop signal.
- **The one genuinely-new-under-drop non-CAS store (benign, adversarial-skeptic
  finding):** `max_used_key_len` (`ft-insert.h:2930-2931`, and the merge/graft
  propagations) is a `CMM_RELAXED` read-then-store, *not* a CAS. Under the drop
  two concurrent inserts of different-length keys can lose the larger
  (W1,W2 both read old max; the larger store is overwritten). **Benign, and not
  a regression:** it is a documented *conservative* per-trie hint; key buffers
  size off the **immutable** `group->max_key_len` (`internal.h:2454` — "bounds
  key buffers"), never this field; and the identical non-CAS race already exists
  on an **OPTIMISTIC** trie (which also skips the FT-wide lock) under the
  certified `0/1600@16w` baseline. The drop just makes FINE match OPTIMISTIC
  here. **sw-cutover note:** if `max_used_key_len` ever becomes
  correctness-load-bearing (e.g. buffer sizing), the cutover must CAS it — under
  sw plain stores it is an unconditional lost update.

## 5a. Adversarial-skeptic result (CLAUDE.md gate)

An adversarial skeptic ran the full four-class refutation search (raw-store
lost update / blocking wait / bail-path abort-boundary / FT-wide-lock doing more
than serialising) against net (A). **Verdict: refuted = FALSE.** Confirmed on
inspection: the in-place reserve slot is `record_reserved` + an unconditional
`guard_parent` on the same state word (not a raw store); `nr_child`/PSO are CAS
loops; `{L}` dup-chain splices are expected-old flip-txns; `lock_or_guard_parent`
always records *either* the RELEASE terminal *or* the fallback guard on the
state word (never neither) so the try-locks are provably additive; bail paths
are register-then-destroy byte-for-byte; retrying writers `urcu_txn_end` before
`goto restart` and `SCOPED_WRITER` is not an RCU read-side, so no peer GP is
wedged; no mid-op GP fence is held on the point-op path. The only new store it
surfaced is the benign `max_used_key_len` above; the only residual uncertainty
is the un-soaked concurrent-cross-trie path (§5.1), where it found no defect.

## 6. Rollout options

- **A (recommended, matches the prototype): all-at-once for FINE.** One flag,
  net (A) covers every domain uniformly, one soak gate. Simplest; the FT-wide
  lock is per-scope not per-domain, so this is the natural unit. **Confirmed for
  cross-trie too** (§9): after the rearchitecture the earlier §8 "carve out
  cross-trie" option is unnecessary — one flag, all domains drop.
- **B (extra caution): per-op-domain drop.** Gate `ft_writer_lock_scope_enter`
  on the op-type so e.g. insert drops while remove still locks, to bisect a
  regression to a domain. Costs an op-type argument threaded to the scope enter
  and buys bisection granularity net (A) already makes unnecessary. Offer only
  if a soak regression actually appears and needs isolating.

## 7. Handoff to the sw cutover (what net B must then complete)

Once the drop soaks clean and (optionally) the default flips to FINE, the sw
cutover (`whole-trie-mcas-to-sw-migration.md`) removes net (A), so it must:
1. **§8.3 layout split** — move `parent_slot_offset` + `incoming_byte` out of
   `state` so C's word is not RMW'd under two locks (lost update under
   plain-store sw).
2. **§4.A state-word fold** — pre-compute the RYW-chained state edits into one
   `{raw→final}` sw record (sw forbids same-slot records).
3. **Close I-1 / I-4b** — bring the in-place `nr_child` and skip-dual `P` slots
   under a held lock (no CAS retry to lean on under sw).

These are the same "coupled end-state" the scope-doc names; MCAS-first simply
lands the *drop* before them, buying a bisectable checkpoint and an early
disjoint-perf datapoint (FINE-post-drop vs OPTIMISTIC — scope-doc P5).

## 8. FINDING — cross-trie `active_reserve` collision (drop-exposed defect)

> **HISTORICAL (2026-07-15) — the recommendation below (Option 1, keep the
> FT-wide lock for cross-trie) is SUPERSEDED.** Mathieu chose **Option 2
> (rearchitect)**, which is done and validated (§9). This section is kept as the
> root-cause record: `active_reserve` collision (Fix 1) → the deeper
> plan-window/spine-staleness class it exposed. The rest of that class was then
> LTTng-root-caused and fixed as described in §9.

The new `inv_concurrent_crosstrie_fine_lock` oracle (16 writers grafting
exclusive sources into one shared live dst, prefixes laid out `{p, w}` so every
writer contends the shared `{p}` spine) **reproduces a real defect under the
drop, deterministically:**

```
cds_ft_alloc_reserve_add:      Assertion `!ft->active_reserve' failed   (alloc.c:1442)
cds_ft_alloc_reserve_activate: Assertion `!ft->active_reserve' failed   (alloc.c:1461)
ft alloc reserve underflow: kind=0 order=11
cds_ft_alloc_item_from:        Assertion `0' failed                     (alloc.c:1240)
ft_graft_keylen:               Assertion `status == CDS_FT_STATUS_OK'    (graft.h:1427)
```

**Root cause.** Graft's NOSPLIT path (and merge, and graft_swap's `gs_reserve`)
pre-fills a stack-local allocation reserve and *activates* it by stashing its
address in the **per-dst** field `dst_ft->active_reserve`
(`cds_ft_alloc_reserve_activate`, ft-graft.h:1414), allocates the bulk op's
nodes from it, then deactivates (1424). This is a **whole-trie singleton**: the
`assert(!ft->active_reserve)` enforces "one active bulk reserve per dst." The
FT-wide lock serialized bulk ops on a dst, so only one was ever active. **Under
the drop, two concurrent grafts into one dst both try to activate → the assert
fires**, or the second drains the first's pool → underflow.

**Why net (A) does not cover it.** `active_reserve` is not a reader-visible slot
written by CAS/MCAS — it is an *allocation resource* (a pointer singleton + a
counted pool). The per-node attach locks (6A) cover the structural *edges*, not
this whole-trie resource. So it is a genuine **net-(B) incompleteness** on the
cross-trie domain that MCAS cannot arbitrate. It is the class-4 case ("FT-wide
lock doing more than serialising writers") the earlier skeptic searched for and
did not find — because it lives on the un-soaked cross-trie path.

**Blast radius is bounded.** `active_reserve` is used *only* by graft / merge /
graft_swap (grep: `ft-graft.h`, `ft-merge.h`, the bulk infra in
`ft-mutation-helpers.h`) — **never insert / remove**. So the point-op drop
(§4, the `1600/1600` gate) is unaffected. Confirmed drop-exposed, not a
pre-existing limitation or a test bug: the *same oracle* passes 3/3 with the flag
OFF (FT-wide lock held serializes the grafts).

**Fix 1 done: thread-local reserve (LANDED in the working tree).** `active_reserve`
moved from the `cds_ft` field to a per-thread `{trie,reserve}` set
(`fractal-trie-alloc.c` `ft_tls_reserves[]`); `cds_ft_alloc_reserve_activate/
deactivate/covers` operate on it; the draw path and graft's `!…active_reserve`
guard consult it. Behaviour-preserving with the flag OFF: ft_unit 281/281 and
the cross-trie oracle both pass on `build-ts-clean`. Under the drop, this
**removes the reserve collision** — the `active_reserve` asserts are gone.

**★ But it exposed a SECOND, deeper drop-exposure — the graft plan-window.**
With the reserve collision gone, the oracle now aborts at
`assert(status == CDS_FT_STATUS_OK)` (ft-graft.h:1428) with
`status = MEMORY_ERROR`, from **ft-graft.h:537** (`ft_node_set_nth_rec`
recompacting `dest`, the graft-point's parent = the shared `{p}` spine). The
graft is designed **unfailable**: it pre-fills a reserve *manifest* sized to what
the store will allocate, then asserts the store cannot fail. That manifest is
computed at **fill** time on `{p}`'s snapshot; under the drop a concurrent writer
grafting at `{p, w'}` **grows `{p}` between this writer's fill and store**, so the
store recompacts a now-larger `{p}` needing nodes the manifest never stocked →
allocation fails → MEMORY_ERROR → the unfailable assert fires. **The reserve is
now thread-local (collision fixed) but the plan (fill) and commit (store) are
still not atomic w.r.t. concurrent changes to the shared spine.**

**Consequence for rollout.** The all-at-once decision holds for the point-op
domains (`1600/1600` stands) but **NOT for cross-trie**. The cross-trie *attach*
carries multiple single-writer assumptions the FT-wide lock was satisfying: the
reserve singleton (now fixed) AND an **unfailable-store guarantee that requires a
stable spine across fill→store**. Making it drop-safe needs the graft/merge/
graft_swap attach to be **contention-tolerant** — hold the graft-point-parent
lock across fill+store so the manifest can't go stale, or retry the whole
plan→commit on failure (like insert/remove's `-EAGAIN` re-descend). That is a
real rearchitecture, disproportionate to the benefit (cross-trie are rare bulk
ops). Options (pending decision):
1. **Keep the FT-wide lock for cross-trie only** (recommended) — point ops
   (insert/remove/recompact) drop; graft/merge/graft_swap retain the lock. Keep
   the thread-local reserve fix (a correctness cleanup + prerequisite for any
   later cross-trie drop). Narrow, principled per-domain carve-out.
2. **Rearchitect the cross-trie attach to be contention-tolerant** — fill+store
   under the graft-point-parent lock, or a plan→commit retry loop. Substantial;
   unblocks a true cross-trie drop.
3. **Defer cross-trie drop to the sw cutover** — same as (1) but framed as
   sequencing; the attach path is reworked at the cutover anyway.

## 9. RESOLUTION — cross-trie is drop-safe (Option 2, rearchitect) — 2026-07-16..18

Mathieu chose **Option 2**: make the cross-trie attach contention-tolerant so it
can drop the FT-wide lock like the point ops, rather than carve it out (Option 1)
or defer it (Option 3). This section supersedes §8's recommendation and records
the defect classes, the fixes, and the unifying principle.

### 9.0 The one root cause behind all cross-trie drop defects

The FT-wide lock was masking a single family of bugs: **the cross-trie attach
plans against a snapshot of the shared `{p}` spine taken at descent, then commits
that plan later — and under the drop a peer relocates/grows/retires that spine in
the descent→commit window.** Every §8/§9 defect is an instance:

- the pre-fill reserve **manifest** goes stale when a peer grows `{p}` (§8);
- the recompact inherits a **stale back-pointer** to a since-reclaimed grandparent
  (Defect C);
- a build-invisible **compressed-split** copies a since-grown node's old child
  set, silently dropping a sibling (the stale-split);
- an **unfailable store/detach** `assert`s success that a contention `-EAGAIN`
  now breaks (graft NOSPLIT, merge sub-position, graft_swap detach);
- a **swallowed abort** proceeds as if it published (graft_swap non-empty).

The fix family mirrors the point-op discipline: **source the plan from the live
descent (reanchored to the current spine), fence the live node being
replaced/split, and retry the whole plan→commit on any contention abort — with
the src kept pristine (or rolled back) so the retry re-plans cleanly.**

### 9.1 The graft plan-window (Defect C) and the stale-split sibling-drop

Two distinct crashes after the thread-local reserve fix (§8, Fix 1):

- **Defect C (commit-side gp-UAF), LTTng-root-caused.** The graft's recompact
  derived its inherited parent from `{p}`'s stale `meta->parent` **back-pointer**,
  which dangled to a grandparent a peer had recompacted→reclaimed. Fix: **source
  the recompact's inherited parent from the graft's own DESCENT** (reanchored to
  the live grandparent), not the stale back-pointer — same family as
  `[[project_ft_chain_compress_stale_publish_slot]]` and the writer-reanchor
  shared primitive. Landed with an RCU-pin of the whole-op descent so a captured
  spine node cannot be reclaimed+recycled (arena re-zero → false-success wild
  store) mid-plan.

- **Stale compressed-split (silent sibling-drop), LTTng-root-caused.** A GLUE
  graft descended a **compressed** divergence node `cn`, built a replacement
  branch from that snapshot **unfenced** (`ft_split_compressed_graft_build`), while
  peers grew `cn` into a multi-child internal; the branch-publish replaced the
  grown node via a refreshed expected-old but carried only the old child →
  siblings lost, **no crash, verify passes** (structure consistent, subtree
  unreachable). Fix: **fence `cn` (its COPYING mark) BEFORE the split reads it**
  (mirroring `ft_split_compressed_insert`), record its retire on the glue txn, and
  bail→retry on a mark miss so a concurrent grow conflicts and one side re-descends.

### 9.2 The abort-orphan and the unfailable-op asserts (retry loops)

Under the drop the attach commit can MCAS-abort (a peer changed the spine); the
fix is a **plan→commit retry** in each cross-trie entry, with one-shot src-side
steps guarded so a retry re-drives only the dst side:

- **graft** (`ft_graft_keylen`): `retry_attach` + `already_swapped`; on a
  store-abort, free the unpublished products and re-descend; the fences
  auto-clear on abort so the retry re-locks cleanly.
- **merge sub-position** (`ft_merge_graft_subpos_inplace`): `retry_merge` +
  `already_unlinked` (the payload is owned after `ft_merge_unlink_src_subtree`);
  the combined `ft_store_at_graft_point` now OWNS its failure cleanup and returns
  non-OK (was an unfailable `assert(st==OK)`), discriminating a permanent
  POPULATED from a transient MEMORY/-EAGAIN.
- **graft_swap** (`cds_ft_graft_swap`): `retry_swap` — the empty-swap prune's
  `ft_detach_node` `-EAGAIN` re-descends (build-invisible, both tries pristine),
  and the non-empty exchange **fuses the swap-root retire into the insert-replace
  txn** so an abort rolls both sides back; `ft_glue_txn_commit_replace` now
  returns the commit status for the check (was `void`, silently swallowed). See
  `[[project_ft_graft_poststore_retry_and_residual]]`.

### 9.3 Cross-trie fusion (single atomic src-retire + dst-attach)

Where the src is exclusive and the list is off, the src-root retire is recorded
INTO the dst-attach flip-txn (`src_swap_fused` / graft_swap's fused retire) so the
unlink and attach commit as ONE MCAS: no orphan window, and an abort rolls the src
back → the retry re-descends with the src still full (no `already_swapped`
bookkeeping). Applied to graft (non-nil + nil-key) and graft_swap; the merge
sub-position keeps the retry-based owned-payload recovery (a comment-only parity
note, full fusion deferred). List-on / non-exclusive-src keep the separate-retire
path (a tracked follow-up, §9.7).

### 9.4 Defect D — rank-stats count-parent walk (NOT drop-safe) → coerce COARSE

A separate class the fusion did not touch: with `rank_stats` on, every
count-changing writer walks `metadata->parent` back-pointers from its stable base
**to the root**, MCAS-recording a `+delta` on each ancestor's `nr_keys`. That walk
(`ft_flip_txn_record_count_parent`) **explicitly assumes the FT-wide writer lock**
(its own comment): under the drop a concurrent recompact relocates an ancestor
between the plain proxy-unaware `nr_keys` read and the commit, so the count edge
lands on the RETIRED ancestor (expected-old still matches → commit succeeds → edge
LOST). The oracle caught it as an over-count (lost decrement) / under-count (lost
increment) — structure fine, only the maintained aggregate wrong.

### 9.5 rank-stats → COARSE coercion (the principled fix)

Mathieu's resolution: **a rank-stats trie always updates the ROOT `nr_keys`, so
there are no disjoint writers — every count-changing op serialises on the root
regardless of lock granularity. Fine buys no measured concurrency, so coerce it to
COARSE.** `group_create` maps `rank_stats_set && writer_strategy != LOCK_COARSE →
LOCK_COARSE` (universal: FINE **and** OPTIMISTIC, since OPTIMISTIC+rank has the
same walk-without-exclusion hazard). A rank-stats trie thus keeps the FT-wide lock
and the walk runs under exclusion = correct. Cost: coercion sets `lock_mode`, which
imposes the step-6 exclusive-src requirement on cross-trie ops — 5 single-threaded
rank-stats unit tests were updated to `cds_ft_make_exclusive` their sources.
**Framing (Mathieu):** re-enabling rank+fine requires a *benchmark* proving a real
win first (root serialises the count either way); until then the deep coherent-walk
fix stays abandoned.

### 9.6 Validation (all under the drop, `FEATURE_FT_MW_LOCK_FINE_DROP`)

- **Cross-trie oracles:** `inv_concurrent_crosstrie_{fine_lock, merge_fine_lock,
  graft_nilkey_fine_lock, merge_nilkey_fine_lock, graft_ord_fine_lock,
  graft_rank_coarse_lock, graft_swap_fine_lock}` all pass under the drop + ASAN,
  and pass non-drop (fences/retries inert under the FT-wide lock). The
  `rank_coarse` oracle was 2–4/12 over-count before §9.5, clean after.
- **graft_swap** (the last, 2026-07-18): drop 51/51 + ASAN/LSAN 7/7 (both retry
  paths proven to fire), full plan 68/68 with it un-gated, non-drop inert 7/7.
- **Point ops:** unchanged by all cross-trie work — the §4 `1600/1600` gate
  stands; a fresh re-run on current HEAD is the last certification step (§9.7).
- **Skeptics:** each fix cleared an async adversarial skeptic (refuted=FALSE);
  several rare abort-path double-free / arena-leak defects were caught by the
  skeptic that the oracle passed over (arena nodes are ASAN-invisible).

### 9.7 Remaining before the default flip, and deferred follow-ups

**To certify:** (1) re-run the §4 point-op `1600/1600 @16w` soak on current HEAD
(~2h; point-ops untouched); (2) this doc refresh (done). **Then** flip the default
FINE→drop → sw cutover P1 (§7 / net B). **Deferred, tracked (not blockers):**
list-on cross-trie under the drop (the separate-retire path is not yet
drop-hardened — the ordered-list dual for the fusion); merge sub-position full
fusion (parity, not a supported-use bug); the `max_used_key_len` non-CAS store
(§5, benign hint) and the rank+fine coherent-walk fix (abandoned pending a
benchmark, §9.5). Commit trail on `ft-txn-integ`: `edb70217`/`79f59ded`/`1633579e`/
`04b18ca5` (Defect C + reserve/reclaim), `c322c42c`/`77fc101c` (graft stale-split
+ abort-orphan), `6789bb64`/`3b65b317` (merge sub-position), `7062609b`/`bc0bb06c`/
`851b3bde`/`07ca464d` (fusion + nil-key), `b8711feb` (rank→coarse), `a3334766`/
`3213fcc7` (graft_swap oracle + retry).
