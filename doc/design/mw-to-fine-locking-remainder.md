# MW → fine locking: the remaining half — transition plan (2026-08-23)

Status: **IN EXECUTION**. Branch `ft/unpub-free-audit` (Phase A is COMPLETE —
G2, A1 and A2 landed; Phase B's mechanism — the owner parameter, the
record-time assert and its readiness counters — is LANDED as step B0, and its
first measurement says NO site is owner-complete yet, so each B1-B5 step now
opens by closing that site's ownership gap: §4).

Companion docs: `mw-writer-lock-escalation-model.md` (the pivot's master note,
§9 lock-sets / §11 migration posture), `ft-dlm-lock-coarseness.md` (the anchor
schedule + acquire-site inventory), `whole-trie-mcas-to-sw-migration.md`
(the pure-sw engine scope — partially superseded, see gate G3).

---

## Execution guide (implementer: START HERE)

Decisions already taken (2026-08-23, Mathieu): **G1 = YES** (writer exclusion
is sufficient for readers — arming unblocked), **G2 = APPROVED** (always-MW
root helper, retire `sw_exempt_slot`). G3–G5 do not block the steps below.

### The rig

    build-g7   ../configure --enable-rcu-debug          # knob OFF
    build-tk   CFLAGS="-g -O2 -DFT_DEBUG_TXN_KIND" ../configure --enable-rcu-debug
    ulimit -v 40000000 ; always run from the BUILD dir, never the source tree
    smoke = test_urcu_ft_unit + test_urcu_ft_inv (FT_INV_MW=1), in parallel
    the kind-counter table prints to STDERR at process exit (destructor)

Before trusting any `--enable-rcu-debug` green, verify the engine self-checks
are ARMED: a two-line TU probing
`#if defined(DEBUG_RCU) || defined(CONFIG_RCU_DEBUG)` must compile the armed
branch — a release build silently poisons the descriptor and retry loops
absorb it (§10). Expected baseline: ft_unit 313 ok / 3 deliberate rekey
failures (109/111/122); ft_inv 119/119; ☠ 4 concurrent ft_inv copies FAIL
`inv_remove_cross_view` — PRE-EXISTING (§9.2), record a control run before
your first change and compare after each step; it must not get worse.

### Step 1 — G2: always-MW root records — ☑ LANDED `a961c6e6`

`ft_flip_txn_record_root` is the mechanism; `sw_exempt_slot`, the record-time
compare, the constructor plumbing and the slot parameter of
`ft_flip_txn_set_structural_sw` are deleted; the two ☠ dual caveats are
rewritten. Behaviour-identical by construction (armed or not, a root edge
reached `urcu_txn_store_mw` before and after), so no abort measurement applies.

☠ **THE §2/G2 SITE MAP BELOW WAS INCOMPLETE — do not trust a site list for
this class.** Two site classes it does not name were found by the detector,
and both would have parked a root the moment A1 arms:

* `cds_ft_graft_swap` (`ft-graft.h:3998`) builds its OWN edge array with
  `&swap_ft->root` and commits it through neither choke point.
* `ft_merge_spine_copy` / its rekey twin publish into `pub_slot ==
  d_dst->nfp`, which **is** `&dst_ft->root` at depth 0.

The lesson generalizes to every later step: root-ness is carried per EDGE
(`ft_pub_rec.root`, `ft_ord_cell_edge.root`), set at the publish branch that
already emits `root_publish`, and dispatched in all five replays. A new
publish path must set it, and the `--enable-rcu-debug` detector in
`ft_flip_txn_record_tag` (armed regardless of `structural_sw`) is what will
say so — it was proven to FIRE, by reverting one `edges[].root` into a red
control, not merely proven armed.

Measured: the reclassification lands at exactly ten root-swap sites; the armed
rekey site keeps `MW_STRUCT == 0`, and its expected small MW_STRUCT →
MW_ALWAYS shift is **zero** — its baseline MW_STRUCT was already zero, so that
writer records no roots in this workload.

### Step 2 — A1: arm COARSE non-exclusive — ☑ LANDED, FOUR commits

Not one commit, and not a predicate flip. The arm itself is
`ft_txn_content_sw_ok` → `!ft->lock_fine && !ft->exclusive` (`543e7c53`), but
three prerequisites had to land first, and each is the same lesson the §2/G2
site map already taught:

* `dc2cfde0` — **the glue's re-parent marks are a FINE-mode acquire.**
  `ft_glue_acquire_reparent_marks` gated on arming while its sibling
  `ft_glue_acquire_splice_holders` gates on `ft->lock_fine`; the mark defends
  against ONE peer (`ft_meta_nr_child_inc` from an insert below the re-homed
  child), and it is FINE that leaves that peer unexcluded. Behaviour-neutral
  at the time it landed BY MEASUREMENT: 1,272,204 glue commits over the
  FT_INV_MW plan, 26,787 armed, ZERO of them COARSE.
* `a34db457` — **a deferred edge carries its own reader-reachability.**
  `ft_glue_apply_deferred` chose store-vs-record from
  `g->txn->structural_sw`, which answers how a record is KINDED, not whether
  a reader can see the slot. The new `@live` bit is set at defer time and
  selected the identical edge set under an equivalence probe (0 divergences
  over 1,595,734 src-origin edges), so it too landed neutral.
* `9ba72990` — `FT_TK_TXN_INIT` ran AFTER the constructor's arm decision, so
  arming counted against an uninitialised `dbg_site`. Latent exactly as long
  as nothing armed; the counter build faults inside the trie's insert path
  the moment A1 lands, which reads as a trie defect rather than an instrument
  that has not been told its own name.

**The positive control** (`inv_concurrent_writers_coarse_lock`, build-tk):
MW_STRUCT 369,694 → **0** across `ft-insert.h:769`, `ft-remove.h:2875` and
`ft-remove.h:880`; armSW 0 → 8,898; SW 0 → 415,790; ABORT and MEMERR stay 0.
`test_writer_lock_mode_*` (5) and both `*_coarse_lock` oracles run, not skip.

**Protocol results.** rcu-debug gate 313 ok / 3 deliberate + 119/119; ASAN
same, no AddressSanitizer report; fault-inject 367 ok / 3 + 119/119; the
concurrent-copy control run like-for-like against a `5051c223` worktree built
the same way — 4/4 and 8/8 clean on BOTH arms, so §9.2's pre-existing
`inv_remove_cross_view` did not fire this time and the control bounds only
NEW breakage, not the pre-existing rate.
Reservations re-checked under `-DURCU_TXN_DEBUG_RESERVE` (proven to fire by
under-reserving the graft glue txn into a red control): no overflow.

☠ **THE ABORT COLUMN DOES NOT MOVE, and that is the correct outcome — do not
go looking for A1's share of the 29%.** On a COARSE trie every writer holds
the FT-wide mutex, so the content lanes had NO aborts to relieve: the coarse
oracle's ABORT column is 0 before and 0 after. Over the whole FT_INV_MW plan
the total abort RATE is 1.95–2.37% before and 1.82–2.39% after (n=3 each,
alternated) — overlapping ranges, i.e. no effect distinguishable from
run-to-run spread, while armSW rises 390k–475k → 544k–586k and SW 2.9M–3.6M →
4.5M–4.8M. A single before/after pair reads as a 29% abort drop and is an
artifact; the acquire lane, which A1 does not touch, moves further than the
content lanes do. **The 29% content-lane bound is Phase B's to collect.**

☞ What A1 buys is the record-kind conversion and the path it clears, not
abort relief. Two red controls stand behind it: reverting `a34db457` under
the arm aborts ft_unit at test 61 on `_ft_publish_to_parent_meta`'s
`assert(cp != NULL)` and silently records 137 HIDDEN back-edges over the
FT_INV_MW plan with every test still passing; reverting `dc2cfde0` alone is
green. The 08-23e residual `not ok 70 inv_graft_swap_shared_dst_ksfix_solo`
does NOT reproduce in any of the three configurations.

### Step 3 — A2: + exclusive — ☑ LANDED `f6093f8b` (one commit)

`ft_txn_content_sw_ok` → `!ft->lock_fine || ft->exclusive`. ☠ A SECOND RULE,
not a widening of A1's: an exclusive trie takes **no mutex at all**
(`ft_writer_lock_scope_enter` returns early on it, deliberately, so a
cross-trie op can hold one live side's lock while an exclusive consumed source
rides through the fused body). The exclusion is the caller's single-writer
contract, which `cds_ft_make_exclusive` drains readers behind
(`ft_writer_lock_gp_wait` before the flag is set).

**The audit that mattered was the CROSS-TRIE direction**, because a txn arms
off the trie it NAMES while it may record a second trie's slots, and
"exclusive" is a claim about one trie. §3's existing note covers only the
dst-named direction. The src-named one — graft's `src_retire` / `swap_retire`
/ `extract`, merge's `src_side` / `run_unlink` — holds because those record
NOTHING BUT their own trie's slots: `ft_glue_apply_deferred`'s src pass stores
rather than records for them (`@live` false), root edges carry `@root` and are
forced MW, ordered-cell edges carry `URCU_TXN_TAG` and are likewise forced MW.
Exclusivity spans the whole op — the BUSY gates refuse a live `swap_ft`
(ft-graft.h) / `src_ft` (ft-merge.h) up front, and the one place that MUTATES
the flag (graft_swap handing `swap_ft` dst's discipline) runs after every
commit.

☞ **The site list came from the COUNTERS, not from reading.** Arming into a
throwaway build and diffing armSW per site named the seven sites A2 actually
opens, which is a far smaller and more honest audit surface than the ~40
creation sites — and it is the same lesson as §2/G2's incomplete site map,
applied before the fact instead of after.

**Positive control.** `ft_unit` under `-DFT_DEBUG_TXN_KIND` is deterministic
and both runs agree on `created` to the txn (3,695,680), so this is exact
rather than a sample: armSW 2,310 → 4,625, SW 27,774 → 41,552, ABORT 0 both
ways, and MW_STRUCT goes to exactly **0** at every newly-armed site. On the
FT_INV_MW plan the extract-side txn (`ft-graft.h:3545`) moves MW_STRUCT
898,064–941,821 → **0**, with 926,651 SW records and 1,088,219 armed.

Gates: rcu-debug 313 ok / 3 deliberate + 119/119; `-DURCU_TXN_DEBUG_RESERVE`
clean; ASAN same, no report; fault-inject 367 ok / 3 + 119/119; 8-copy control
8/8 clean on BOTH arms against a `570f0a3c` worktree. No abort claim, per the
step protocol above.

☞ The hand-arming is NOT retired by this step: the rekey writer arms under
FINE (a per-op argument, Phase B) and the root-COW driver names an arbitrary
trie. Both still arm on their own reasoning.

### Step 4 — B0: the record-time owner mechanism — ☑ LANDED

The owner parameter, the record-time assert, the dry-run claim and its red
control, plus the `OWN_HELD`/`OWN_MISS` readiness counters — all in §4, with
the measurement they produced. Behaviour-neutral by construction: the owner is
read only under `--enable-rcu-debug`, and no site arms per-op yet.

Gates: rcu-debug 313 ok / 3 deliberate + 119/119 with `FT_INV_MW=1`, no
assertion; `FT_RED_OWNER_CLAIM_ON_LOCK` aborts on ft_unit's second test, by
this assert's name. No abort claim applies (nothing converted).

★ Behaviour-neutrality is MEASURED, not asserted: the full 13-config
`ft_parallel_gate.sh` was run at this commit and at `4887aaab`, and **all 115
legs are identical** — same ok / notok / abrt per leg. That is the strongest
form of "changes nothing" available here, and it is what a step whose whole
product is a DETECTOR should be held to. ☠ It also means the gate was already
RED before this step: see §9.5.

☞ The owner is an UNCONDITIONAL parameter — unlike `FT_TK_SITE_PARAM`, which
is knob-gated precisely so an instrument cannot perturb what it measures — so
some sites now compute a metadata pointer a release build never reads.
Measured rather than argued, like-for-like `-g -O2`: `liburcu-cds` text
603,177 B before, 602,853 B after (**−324 B**). The optimizer drops the
derivations; the parameter is free where it is not read.

### Then

Phase B's mechanism is done; what it revealed is that **no site is
owner-complete**, so each site step now starts with closing its ownership gap
(§4). Read §2/G5 for the bulk-op freeze gate that Phase B's bulk sites will
sit behind. §11 has the full landing order.

---

## 0. Where we stand (the done half)

The model, one breath: a structural op acquires its per-node DLM lock-set in
ONE all-or-none MCAS (the MW arbitration point), then edits under the held
locks; readers stay wait-free (proxy resolve, never see the lock). The
transition's first half converted the EXCLUSION machinery; the second half
converts the CONTENT records from MW (validated CAS per slot) to SW (plain
parks under the held locks) and then collects the payoffs that exclusion
enables.

Landed (exclusion, complete):

* DLM one-commit acquire on every op family (recompact, insert, remove/detach,
  merge, graft/graft_swap, rekey); cross-trie ops require an EXCLUSIVE
  consumed source (BUSY gate), dst-side-only lock-sets.
* `CDS_FT_WRITER_OPTIMISTIC` is **deleted** — the enum is
  `{LOCK_COARSE = 1, LOCK_FINE = 2}` and **FINE is the default**
  (`include/urcu/fractal-trie.h:2867`). The per-slot-CAS MW mode Mathieu
  marked for deprecation no longer exists.
* The FT-wide writer lock is dropped per-domain under FINE; escalation-domain
  enrolment is complete at every measured acquire site (0 domain-less, was
  2.3M), with aging-on-refusal (which beat the `_on` binding: 4 vs 9.5
  starving removes).
* The mixed SW/MW txn engine is the vendored substrate (`rcu-txn-mcas.h`):
  MW records commit-ordered FIRST (can fail, back out clean), SW records park
  just before the flip. This is what makes a per-record-kind conversion
  possible at all.
* Lock-spacing (anchor coarseness): capture, knob, and the FULL acquire-site
  conversion are in — every acquire resolves through the two choke points and
  the raw primitives do not compile under `FEATURE_FT_ANCHOR_VALIDATE`
  (`ft-dlm-lock-coarseness.md` §9). The coarse settings are still refused at
  the API (`ft-lifecycle.h:369`) pending certification (Phase E below).

Landed (content, staged but UNARMED):

* `6815f178` — per-creation-site record-kind counters
  (`-DFT_DEBUG_TXN_KIND`, `ft-txn-kind-stats.h`).
* `a45869d2` — the CONTENT / ACQUIRE constructor split: content constructors
  take `struct cds_ft *ft` (56 sites, compiler-enforced); the one acquire
  site uses `ft_flip_txn_acquire_bounded()` and names no trie.
  `ft_txn_content_sw_ok(ft)` (`ft-mutation-helpers.h:1154`) is the single
  policy switch and returns **false** — nothing armed.
* Two sites still HAND-arm `structural_sw` on their own reasoning: the rekey
  writer (`ft-rekey.h:1754`) and the root-COW debug driver
  (`fractal-trie.c:357`). Retiring that hand-arming is part of the done
  definition.

### The baseline (ft_inv FT_INV_MW=1, 507 threads, 94.9M txns)

    MW_STRUCT   136,857,021   <- THE CONVERSION SURFACE, 27 content sites
    MW_ALWAYS    85,087,969      cells / counts / conditional re-parents: stays MW
    MW_LOCK      52,667,070      the lock take: stays MW forever
    VALIDATE     23,383,128      §4.B guards
    cell/hlist   17,551,703      ordered cells: stays MW (deliberate)
    ABORT         2,415,932   <- the acceptance number

* 71% of all aborts are the ACQUIRE lane (`ft_dlm_acquire_set_at`,
  1,718,411). Content lanes total 697,521 — **the conversion's upper bound on
  abort relief is ~29% of aborts**, not the headline count. Say so in any
  claimed win.
* MW_STRUCT is concentrated: `ft-insert.h:774` (50.2M) + `ft-remove.h:2875`
  (46.8M) + `ft-remove.h:3716` (15.7M) + `ft-remove.h:880` (9.6M) +
  `ft-remove.h:3942` (4.9M) = **127M of 137M. Five creation sites, all
  single-trie.** Convert those and the rest is noise.
* Positive control: the one already-armed site (the rekey writer) reports
  armSW 457,757 / SW 3,472,296 / MW_STRUCT **0**. A conversion that leaves
  MW_STRUCT in place at its site did not run.

Baseline test state: `--enable-rcu-debug` ft_unit 313 ok / 3 deliberate rekey
failures (109/111/122); ft_inv 119/119 with FT_INV_MW=1; ☠ ft_inv ABORTS with
4 concurrent process copies (`inv_remove_cross_view`) — pre-existing, and the
single-process gate cannot see it (§9.2).

---

## 1. End state (definition of done)

Two writer modes, one granularity axis, one mixed-commit engine:

1. **Single-writer** — the caller guarantees one writer (readers stay
   concurrent and wait-free). Today this mode is only approximated
   (`ft->exclusive` excludes readers too; COARSE pays a mutex the caller may
   not need). A validation-only strategy is owed (§8.4).
2. **DLM / LOCK_FINE** — per-node lock-sets, with the lock-spacing axis
   (per-node … exponential … root-only) as the coarseness knob. At root-only
   spacing this degenerates into today's LOCK_COARSE, which then becomes a
   redundant strategy to fold away (Phase E).

Record kinds at end state:

| kind        | fate                                                          |
|-------------|---------------------------------------------------------------|
| MW_LOCK     | stays MW forever — the lock take IS the arbitration point     |
| MW_STRUCT   | **→ SW** wherever the op's lock-set owns the written word     |
| MW_ALWAYS   | stays MW unless the Phase-C decision converts a class         |
| cell/hlist  | stays MW (deliberate: no per-cell serialization)              |
| VALIDATE    | stays — read-set validation is orthogonal to record kind      |

Done means: `ft_txn_content_sw_ok` answers true for COARSE, exclusive, and
the armed FINE ops; the two hand-armed sites route through the switch;
MW_STRUCT ≈ 0 at every converted site under the counter build; the engine's
`--enable-rcu-debug` kind-conflict self-checks are green under load; the
abort delta is measured and reported against the 29% bound.

---

## 2. Decision gates (before Phase A arms anything)

### G1 — is writer exclusion SUFFICIENT FOR READERS? — **ANSWERED YES (Mathieu, 2026-08-23)**

An SW park is legal only where the op excludes every peer WRITER of the slot.
Established: resolve is identical for both kinds (a parked SW record is
reader-visible exactly like an MW one), with ONE asymmetry — a LONE SW record
commits as a plain release store with **no proxy and no grace period**
(`urcu_txn_desc_commit`, `t->nr == 1`), where a lone MW record below the
escalation age commits as a bare CAS. Neither publishes through the flip.
**Decision: writer exclusion is sufficient for readers; the lone-record
asymmetry is acceptable. Arming is unblocked.**

### G2 — retire `sw_exempt_slot`: route root republishes through an always-MW helper — **APPROVED (Mathieu, 2026-08-23)**

A cross-trie dual txn names ONE trie and may write TWO roots
(`ft-graft.h:1362`, `:2853`), while the exemption holds one slot — arming
such a txn would park the second root unarbitrated. Two ways out were named;
**decision: the always-MW record helper** (a `ft_root_record_*` that
records any `&ft->root` edge MW regardless of `structural_sw`), because the
rule it encodes is "a trie root lives in no node" — a property of the slot,
not of the txn's named trie — and it deletes the exemption plumbing
(`sw_exempt_slot`, the record-time pointer compare at
`ft-mutation-helpers.h:2675`) instead of generalizing it. Land this FIRST; it
makes every later arming step shape-independent and turns the two ☠ call-site
caveats into dead text.

#### G2 implementation map (surveyed 2026-08-23 at `a45869d2`)

★ **The exemption is a record-time compare BECAUSE root-slot records reach
the generic record path through DYNAMICALLY RESOLVED slots** — a NULL parent
degrades to a publish into `&ft->root` (`ft-mutation-helpers.h:118`, `:4347`,
`:4359`; `ft-mutation-node.h:1479`, `:1526`). So G2 is not a purely static
site conversion: those arms already KNOW the root shape (they branch on the
NULL parent and emit the `root_publish` tracepoint, `ft-helpers.h:2628`), so
each routes to the always-MW helper from its existing branch — the compare
moves from record time into branches the sites already take.

Conversion surface:

* **Two choke points cover the root/list swaps**: `ft_root_list_swap_publish`
  (7 call sites: ft-detach.h:187, ft-graft.h:2251/3787, ft-merge.h:2104/3555,
  ft-rekey.h:4200/5579) and `ft_root_list_swap_publish_dual` (the 2 dual
  sites, ft-graft.h:1462/2903). Convert INSIDE the helpers once.
* Direct `(void **) &…_ft->root` records beside those calls:
  ft-merge.h:2120/3572, ft-graft.h:2222/2264, ft-rekey.h:4216/5596.
* The root COW driver's forward publish: `fractal-trie.c:381`
  (`ft_flip_txn_record_reserved(txn, (void **) &ft->root, …)` — its comment
  at :367 already states the MW intent).
* The dynamic NULL-parent publish arms listed above.

Deletions once routed: the `sw_exempt_slot` field
(ft-mutation-helpers.h:1003), the compare (:2675), the constructor plumbing
(:1199/:1419/:1472 with their `&ft->root` arguments), the slot parameter of
`ft_flip_txn_set_structural_sw` (:1119/:2724), the ☠ block in
`ft_txn_content_sw_ok`'s header, and the two ☠ call-site caveats
(ft-graft.h:1362/:2853) — rewrite those as "roots record MW by
construction".

Counter classification: root records under an armed txn currently count
MW_STRUCT (they take the non-exempt branch's else). The helper should count
them **MW_ALWAYS** — a root can never convert, so moving it out of MW_STRUCT
cleans the conversion-surface metric. Consequently the armed rekey site's
table row shifts a small count from MW_STRUCT to MW_ALWAYS: expected, not a
regression. Optionally keep a debug-only assert that an SW park's slot is
never the NAMED trie's root — cheap belt while the plumbing is deleted; it
cannot cover the second trie's root, which is exactly why the helper, not
the assert, is the mechanism.

### G3 — confirm the engine end state is the MIXED commit, not pure-sw

`whole-trie-mcas-to-sw-migration.md` (2026-07-15) scoped REPLACING the MCAS
engine with `rcu-txn-sw`. The mixed SW/MW engine adopted a week later
(2026-07-22) changed the ground: cells, locks, and rank counts stay MW, so a
pure-sw engine can never carry a whole FT commit. Proposed doc-status update:
the pure-sw migration is RETIRED as a whole-engine goal; its §4 content
survives as exactly this plan's per-record conversion. (Its §4.G — the layout
split — re-enters at Phase F.)

### G4 — the residual MW_ALWAYS lanes (decide AFTER Phase B measures)

Mathieu's open granularity question: do ordered cells (and dup-chain splices,
rank propagation) keep a narrow MW lane, or grow a state word and join the
lock-sets? Deferred deliberately — see Phase C for the measurement that
decides it. Do not decide it on argument now.

### G5 — bulk vs point ops: who carries the exclusion, and by which mechanism

Goal (Mathieu, 2026-08-23): point ops must not contend on near-root nodes, so
the bulk/point mutual exclusion is the BULK op's to carry. The §9 point-op
lock-sets are already leaf-local; the residual near-root cases are (a) a
point op mutating a direct child of a junction — a genuine conflict, small
and correct; (b) recompact cascade climbs — rare by construction; (c)
rank-stats — coerced serial anyway; and (d) ☠ COARSE LOCK SPACING, which
anchors deep ops near the root BY DESIGN — a bulk-heavy trie should stay
per-node spacing, and Phase E's bench must measure the spacing axis against a
bulk MIX, not the point mix alone.

Three candidate mechanisms, with a proposed split:

**(A) The bulk op locks its WRITTEN FAN — never the subtree.** A moved
subtree's INTERIOR moves wholesale: no interior word is written, so a deep
point op shares no word with the bulk op and COMMUTES with the move (its
edit lands in the subtree wherever the subtree now hangs). Exclusion is owed
only on words both sides write: {junction / prune path} ∪ {the fan of
children whose edge words the op rewrites}. Cost O(fan) ≤
`FT_ENTRY_PER_NODE`, not O(subtree) — the §9.6 wholesale-move result
survives. Precedent: `ft_rekey_cow_stop` / `ft_glue_acquire_reparent_marks`
(`child_marked`). ☠ Bound from the REVERTED lock-the-sweep experiment:
fan-locking the HOT recompact path livelocks (a contended child fails the
ACQUIRE; escalation rescues only a COMMIT) — fan locks belong on RARE ops
only. Note the fan lock exists because pre-§8.3 the child's
`parent_slot_offset` shares the child's STATE word; **after the §8.3 layout
split the junction lock alone owns the whole written frontier and the fan
locks retire** (§8.1 carries this as a payoff). What (A) does NOT give:
semantic freeze — deep point ops proceed and commute. No lock frontier can
exclude them without point ops READING an ancestor word, which is exactly
the near-root traffic the goal forbids.

**(B) Swap the trie's concurrency mode and DRAIN MUTATIONS — the move-gate
pattern applied to writers (Mathieu's alternative).** The rekey reader gate
(`move_active` + piggybacked GP; one stable load per reader) generalizes: the
bulk op flips a per-trie writer-mode word, then drains in-flight point
mutations, runs writer-exclusive, flips back. Because every point mutation
already executes inside the caller's RCU read-side bracket, the drain is ONE
grace period: a mutation that loaded the old mode word finishes inside its
bracket; new arrivals see DRAINING and park. The point-op cost is a single
LOAD of a trie-level word that changes only at bulk boundaries — the line
stays shared in every cache, no stores, no refcount: strictly better for the
near-root goal than any lock. What (B) buys over (A): TRUE semantic freeze
(deep mutations excluded, not merely commuting), and the bulk window is
writer-exclusive so the bulk op's content records become SW-able by the
EXCLUSIVE argument — no fan discipline, no `child_marked` subtleties, the
hardest Phase-B cases collapse into the easy arm. What it costs: every point
mutation on the trie stalls for the window (tail latency), and the window
pays a GP even when no reader coherence needed one — though for the rekey
family the gate + GP are ALREADY on the path for reader coherence, so the
writer-side drain adds no new GP there. Constraints proven elsewhere that
bind (B): parked mutators must go `thread_offline` around their wait (the
QSBR owner-waits-for-waiter deadlock, fixed once already for the mover's
cond_wait); gate entry needs FIFO fairness both ways; ☠ and the drain's
latency bound is the longest point-op retry storm — under QSBR a spinning
retry loop has NO quiescent window, so Phase D's liveness work is a
PREREQUISITE for (B)'s drain to have a bound at all.

**(C) The exclusivity contract** stays what it is: cross-trie ops require an
EXCLUSIVE consumed source; detach-and-hand-off is a mode swap by definition.

**(D) The hybrid (Mathieu): point mutations LOAD a locking-mode state on
every node of their descent, root to leaf — bulk ops flip it only on the
JUNCTION they affect.** The mutation descent already loads each entered
node's state word for dispatch (kind, proxy, skip), so a per-level mode
check is a BRANCH on a word already in hand: zero extra loads, zero stores,
and loads of a rarely-written line do not contend — the strongest possible
fit for the near-root goal. Bulk protocol: set a FREEZE state on the
junction (taken UNDER the junction's already-held DLM lock-set, so
bulk-vs-bulk composition rides the existing acquire/validate machinery),
then ONE grace period drains the concerned in-flight mutations — every point
mutation runs inside the caller's RCU bracket, so after the GP each has
either finished or will see the freeze on its descent and park. The bulk op
then edits writer-excluded over the subtree — SW content by the exclusive
argument, scoped — clears the state and wakes. No commit-time validation of
the freeze is needed: load-and-branch + GP is the move-gate publication
pattern. Stall scope is exactly the ops whose descent crosses the frozen
junction; disjoint bulk ops run in parallel. **Placement generalizes: the
freeze on the ROOT is (B)** — one mechanism, placement chooses scope, the
same shape as the lock-spacing axis. Obligations: (1) handle-based
mutations do not descend (the §5.3 node-handle hole) — they RE-DESCEND when
the feature can be armed, the precedent `ft_anchor_descend` already
established for coarse spacing; (2) parked mutators go `thread_offline`
around the wait (the QSBR mover fix, again); (3) the drain bound is still
the longest in-flight point op, so Phase D remains the prerequisite —
though only retries INTO the frozen subtree matter; (4) readers never check
the state: wait-freedom untouched.

**Proposed split:** (D) is the default for the bulk family — it gives (B)'s
semantic freeze and free SW-content arming at (A)'s scope, for a point-op
cost of a branch on an already-loaded word, and (B) falls out of it as the
root placement. (A) remains for bulk shapes that need no semantic freeze
(deep ops commute and are never stalled; its fan half retires at §8.3
regardless). (C) unchanged for cross-trie. To settle with Mathieu before
Phase B reaches the bulk sites: the freeze encoding in the state word, the
park/wake mechanism, FIFO fairness both ways, and the handle-path
re-descend trigger.

---

## 3. Phase A — arm COARSE, then exclusive

Order: **COARSE non-exclusive → exclusive**. COARSE first because the FT-wide
mutex is the simplest exclusion argument (every structural writer serializes;
cells are already forced MW via `ft_hlist_store_mw` /
`ft_ord_cell_record_into_ft`; with G2 landed the root is always-MW), and
because it is cheaply measurable end to end. Exclusive second (single writer
by contract; also covers teardown paths — note `ft->exclusive` switches
reclaim to `ft_flip_txn_call_rcu_now`, so the lone-SW no-GP asymmetry from G1
is most exposed here — G1's answer covers it, Mathieu 2026-08-23).

Arm predicates (fields verified at `a45869d2`: `bool exclusive`
fractal-trie-internal.h:1592, `bool lock_fine` :1747):

    A1  COARSE non-exclusive:   !ft->lock_fine && !ft->exclusive
    A2  + exclusive:            !ft->lock_fine || ft->exclusive
    B   FINE non-exclusive stays false at THIS switch — it arms per-op,
        from the held set, never from the trie (§4)

Note a cross-trie commit consults the switch for its NAMED trie while
recording a second trie's slots: with an exclusive consumed src (the BUSY
gate guarantees it), src-side slots are writer-excluded by exclusivity, so
the named-trie answer remains sound in both A1 and A2.

Per-step protocol (same for every arming step in this plan):

1. Flip the corresponding arm of `ft_txn_content_sw_ok` only.
2. Counter build (`build-tk`): MW_STRUCT must MOVE to SW at the armed sites —
   the positive-control pattern; a zero that appears without the armSW
   counter moving means the mechanism did not run, not that it is clean.
3. `--enable-rcu-debug` gate: the engine's nine `urcu_assert_debug`
   self-checks are the SW/MW kind-conflict detector; verify the detector is
   ARMED (the two-line TU probe) before trusting a green — a release build
   silently poisons the descriptor and the retry loop absorbs it.
4. ft_unit + ft_inv (FT_INV_MW=1) + the 4-concurrent-copies control run
   (§9.2) + ASAN + the fault-inject builds that reach the abort arms.
5. ABORT column, and ☠ **NOT against the 29% bound — that is an END-STATE
   number and reporting it per step manufactures a false win.** While the
   transition is incomplete the plan-wide total is dominated by the lanes
   this step did NOT convert: the FINE tries still recording all-MW, plus the
   acquire lane that is 71% of aborts and is never converted at all. So the
   step-local signal is buried under run-to-run spread. Measured on A1: one
   before/after pair reads as a 29% abort drop, and three alternated runs each
   way dissolve it into overlapping ranges (1.95–2.37% vs 1.82–2.39%). What
   the column IS good for per step:
   * a **wrong-direction alarm** — an SW park cannot fail, so mis-arming does
     not show up as an abort, but it often shows up as aborts RISING (an MW
     guard edge whose expected-old now mismatches the op's own mark). Cheap,
     keep it.
   * a **per-mode baseline**, read on the oracles that actually DRIVE the
     armed mode rather than on the plan total.
   Never report a plan-wide abort delta from a single before/after pair; n>=3
   alternated runs, or say nothing.

Risk to keep in front: **an SW park cannot fail** — a wrongly-armed park does
not abort, it silently erases a peer's committed edge. That is why each arm
is gated on its exclusion argument, and why step 3's detector must be proven
armed rather than assumed.

## 4. Phase B — FINE non-exclusive, per-op

Under FINE the promise is per-op, not trie-wide: **an op may park a word SW
only if its held lock-set OWNS that word.** Ownership per the §8 edge
principle: a parent→child edge's four fields are owned by the PARENT's lock;
a node's state word is owned by its own COPYING. The invariant proven at the
shared-junction oracle binds here: *a state word may be SW-parked only by the
txn holding that node's COPYING* — a recompaction's child edges stay MW
unless the op marks each child (the A3 lesson; the reverted lock-the-sweep
experiment must not be silently re-run).

### Step B0 — the mechanism — ☑ LANDED

`ft_txn_content_sw_ok` alone cannot answer a per-op question, so the answer
travels with the RECORD. Three pieces, all in place:

* **An `owner` on every SW-capable record helper**, compiler-enforced exactly
  as the CONTENT/ACQUIRE split was — `ft_flip_txn_record_tag`,
  `_record_reserved`, `_record_publish`, and per EDGE in `ft_pub_rec.owner[]`
  / `ft_ord_cell_edge.owner` (the `root[]` shape G2 landed, and for the same
  reason: the producer is the only place that knows). ☠ A bare `void **slot`
  cannot yield its owner — the owner differs by FIELD KIND (§8) and no
  arithmetic on the address recovers it.
* **The record-time assert** (`FT_OWNER_ASSERT_OWNED`, `--enable-rcu-debug`
  only), asking `ft_flip_txn_owns` of every SW-capable record on a txn that
  claims per-op ownership. Zero cost in a release build.
* **`ft_flip_txn_claim_per_op` — the CLAIM WITHOUT THE ARM**, and it is the
  tool the site steps below are run with. Converting a site asks two
  questions — "does the op own what it writes?" and "does parking it pay?" —
  and only the first can make the structure wrong. Claiming answers it with
  an abort AT the offending record on a build that is otherwise
  byte-identical to the unconverted one. It is also why the red control is
  sound: a control that ARMED would plant SW parks its txn never reserved
  for, and the engine's own kind / duplicate-slot self-checks would answer
  first — proving the engine detects a malformed descriptor, not that this
  check detects an unowned park.

Proven RED by `FT_RED_OWNER_CLAIM_ON_LOCK` (claim on the first
`ft_flip_txn_lock_register`): `ft_unit` aborts on its second test, and the
abort is this assert by name, not the engine's.

☠ `ft_flip_txn_owns` reads THE TXN REGISTRY, not `ft_held_set_snap` — a
record helper cannot see the op's `ft_held_set`, which lives on a stack frame
above it. A lock whose terminal this commit records must be registered on it
anyway, so a word owned by an unregistered lock is a finding; but it does
mean the numbers below are a LOWER BOUND on ownership. Exact at the default
per-node lock spacing; conservative above it (a coarse anchor is held while
the owner itself is absent).

### The readiness measurement — ☠ NO SITE CAN ARM YET

`OWN_HELD` / `OWN_LEDGER` / `OWN_MISS` (ft-txn-kind-stats.h) run the same
predicate on every record in EVERY mode, so a FINE **unarmed** run says whether
an arm would be legal BEFORE the arm is written. They split `MW_STRUCT` exactly
— the surface, by whether the op holds the word's owner — and their sum being
`MW_STRUCT` is an invariant of the table worth checking.

The three columns are the two WITNESSES of a hold, plus neither. `OWN_HELD` is
the txn's `locks[]` registry. `OWN_LEDGER` is `FEATURE_FT_HOLD_TRACE`'s
per-thread ledger, maintained at the lock PRIMITIVES so it sees a hold
whichever `ft_lock_ctx` frame filed it — a REGISTRY gap, not an exclusion gap.
Neither subsumes the other, so the held set is the UNION: the ledger drops its
entry the moment a release is *recorded* while the word keeps LOCK until that
commit lands, and there the registry is the only witness. ☠ `OWN_LEDGER` reads
a constant 0 without `-DFEATURE_FT_HOLD_TRACE`, and that zero is not evidence.

ft_inv, `FT_INV_MW=1`, 507 threads, per-node spacing, `build-ownwide`
(`--enable-rcu-debug CPPFLAGS="-DFT_DEBUG_TXN_KIND -DFEATURE_FT_HOLD_TRACE"`):

| creation site | MW_STRUCT | registry | ledger | owner-held |
|---|---|---|---|---|
| `ft-insert.h:776` — insert one-commit | 41,124,651 | 5,991,155 | 0 | **14.6%** |
| `ft-remove.h:2877` — remove commit_rec | 39,893,261 | 4,101,447 | 1,427,533 | **13.9%** |
| `ft-remove.h:3735` — head promote | 12,977,124 | 4,325,708 | 0 | **33.3%** |
| `ft-remove.h:882` — detach-side creator | 8,083,737 | 3,447,004 | 218,259 | **45.3%** |
| `ft-remove.h:3958` — unchain publish | 5,010,706 | 0 | 0 | **0.0%** |
| `ft-graft.h:3463` | 2,250,327 | 0 | 0 | **0.0%** |
| `ft-graft.h:1705` | 1,080,992 | 198,055 | 136,632 | 31.0% |
| **TOTAL** | **111,531,468** | **18,163,222** | **1,953,955** | **18.0%** |

☠ ft_inv is concurrent and its totals move a few percent run to run, so compare
COLUMNS WITHIN ONE RUN, never a percentage against an older run's.

**NOT ONE creation site is owner-complete, so not one of the five sites below
can arm as things stand** — the assert would fire at every one of them. That
is the finding, and it reorders the phase: the step per site is no longer
"arm it", it is *make it owner-complete, prove it with the claim dry-run,
then arm it*. The arm is the cheap half.

### ☑ THE PREDICATE WAS PART-BLIND — measured, and it is worth 1.7 points

`ft_flip_txn_owns` asked the txn registry only, and the claim dry-run's first
abort said that was too narrow: it landed in `ft_flip_txn_record_retire_anchored`
(reached from `ft_detach_node`) inside `if (h->lock == node || h->node_held)`
**and** `if (h->shared || h->node_held)` — both of which require the op to hold
`node`'s own word. Structurally proven, no debugging needed: read the branch the
frame is in.

Widening it to registry ∪ hold-ledger (`92e27199`) puts a number on it:
**1,953,955 of 111,531,468 records, 1.7 points**, all of it at
`ft-remove.h:2877` and `:882` and none at the insert site. The readiness
conclusion does not move — no site is owner-complete — so the remainder can now
be read as a REAL exclusion gap, which is what the site work needed.

☠☠ **AND ITS FIRST ANSWER WAS A ZERO THAT WAS AN INSTRUMENT BUG.**
`OWN_LEDGER` read 0 at every site over 110M records — not because the widening
buys nothing, but because `ft_flip_txn_record_retire_anchored` called
`ft_hold_trace_drop(node)` at the TOP, before planting the records that ask
whether the op owns `node`. The one class where out-of-registry holds actually
occur was erasing its own evidence. Two checks were needed and neither alone
sufficed: a RED CONTROL (force the query true → every MISS moves to LEDGER,
proving the wiring) and a LIVENESS PROBE (`return ft_hold_trace_n > 0` → 84.2M
of 85.5M records are planted with a NON-EMPTY ledger, proving the zero was about
*which* words it named). The red control alone would have "proved" the wiring
and left the bug standing.

### The external-head class — ☠ NOT one design question, and §8.2 is the OTHER doc

The 0.0% rows were recorded as ONE class, `FT_OWNER_NONE_EXTERNAL_HEAD` (11
sites) — an external head's back-channel word (`cell->parent`, `en->prev`,
`next_node->prev`) has no owning lock, because neither a cell nor an external
node carries a state word — and as "the largest single item in Phase B, a
design question, not plumbing", on the strength of *"§8.2 puts the entry list
under the HOLDER's lock"*.

☠ **THE CITATION IS TO `mw-writer-lock-escalation-model.md` §8.2
("Field-by-field ownership"), NOT to §8.2 of THIS document** (which is In-place
mutation, Phase F). That table is real and it answers more than the marker
claimed: `external_nodes` → **self (C)**, and — the line that matters for the
open fork — the `parent` pointer → **parent (P)**. ☞ Section numbers are reused
across the two design docs; say WHICH doc.

★★★★ **AND THE FORK IS NARROWER THAN "UNDECIDED" BECAUSE OF IT.** The
escalation model already assigns the parent pointer to the PARENT, while the
landed code keys the edge kind on holding the **child**
(`ft_flip_txn_record_parent_word`'s `@child_held`). So the design question is
not "pick a convention from scratch" — it is *the model and the code disagree,
and one of them must move*. That is a much cheaper question to put to Mathieu.

Read the three rows apart and they are three different things:

* **Head promote** (`ft-remove.h`, the `ft_promote_head` row). The holder was
  never missing: it is a PARAMETER (`held_holder`), `@head_slot` is the holder's
  own slot, and `ft_flip_txn_hold_or_lock_parent` REGISTERS it into this very
  txn. The only obstacle was ORDERING — the `&next_node->prev` record was
  planted before that call. Hoisting the acquire and naming
  `ft_flag_to_metadata(ft, parent_nf)` took the row from **0.0% to 33.3%**, and
  exactly so: 4,325,708 × 3 = 12,977,124, one owned back edge per two unowned
  publish edges. Landed; it was plumbing, not design.
* **Unchain publish** (`ft-remove.h:3958`) is NOT the external-head class at
  all. Its acquire is already ahead of the publish; its edges arrive through
  `struct ft_pub_rec`, whose `owner[3]` array **no producer ever fills**, so
  every one of them is NULL by default. `ft_pub_rec_add` has no `owner`
  parameter to fill it with. That is a PLUMBING item on the publish lane — add
  the parameter, name the owner at its four call sites — and it is the same
  two-thirds remainder the promote row still carries, so it is worth
  substantially more than this one row.
* **Back-edge re-parent of an external child** (`ft-graft.h:3463`, and the
  `ft-insert.h` / glue sites) is the REAL design question, and it is the
  SMALLEST of the three. `ft_reparent_record_meta` sets the convention —
  `owner = meta`, the CHILD's own word — and an external has none. Closing it
  is a CHOICE: give externals a state word (§8.1's layout split, Phase F), or
  change the convention so a back edge is owned by the HOLDER — which is what
  the escalation model's §8.2 already says for the `parent` pointer, against
  what the code does.

  ☑ **THE KIND IS SETTLED AHEAD OF THAT ANSWER, AT EVERY SITE** — `5fa631c7`
  ruled it and converted one; `4da182f2` converted the other seven records,
  behind one named helper (`ft_flip_txn_record_head_back_edge`). Whoever ends
  up owning the word, NO op can hold it today, so it is
  `ft_flip_txn_record_parent_word`'s permanent false arm and takes the
  always-MW `ft_flip_txn_record_root` treatment. The last one needed the
  `ft_pub_rec` detour removed rather than a kind swap — a rec's per-edge
  answers are `@root` and `@owner`, and neither can say "always MW" — which
  also retired `ft_pub_rec_add_back_edge` into `ft_record_child_back_edge`.
  What stays open is only OWNERSHIP, and it is Phase C's ledger to re-examine
  now that the traffic has left the MW_STRUCT surface.

☐ **Still unproven for all three, and it must not be skipped**: owner-AVAILABLE
is not owner-SUFFICIENT. Nobody has yet shown the holder's lock EXCLUDES every
writer of a chain member's `->prev` — an insert adding a duplicate, a
rekey/graft moving the head, `cds_ft_compact` relocating cells
(`ft-compact.h:403` writes `&head->prev`). The `ft-txn-hlist.h` `->prev` stores
are a SEPARATE lane, MW_ALWAYS by design and out of scope (the G4 decision).
Naming an owner only makes a site *eligible*; the exclusion argument is what
the arm needs, and it gets its own adversarial skeptic.

`FT_OWNER_UNPLUMBED` (1 site, `ft-compact.h`) is the other marker: the owner
exists and is simply not in scope. Both are greppable.

### Site order

Unchanged as an ordering, but each step is now
*owner-complete → claim → arm*, one site per step, one adversarial skeptic per
claimed exclusion argument:

1. `ft-insert.h` insert one-commit — ☑ **ARMED** `e2ba43f8`. (Was owner-complete
   at 14.3% → 40.1% held.)
   Its claim dry-run passes ft_unit AND ft_inv `FT_INV_MW=1` at 507 threads with
   no abort. NOT armed: the arm waits on 9.1, and its placement is an open API
   question (`ft_flip_txn_arm_per_op` has zero call sites and refuses an empty
   registry, yet kind dispatch happens at RECORD time). ☞ Read the DRY RUN as
   the readiness signal, not the percentage — the counter also prices records on
   txns the arm refuses outright.
2. `ft-remove.h` remove commit_rec — ☑ **OWNER-COMPLETE** `a9ff9549`, after ONE
   fix. Dry run (`-DFT_REMOVE_CLAIM`, `a3c75659`) started at ft_unit test **239**
   — the rekey writer's started at 112, and the difference is the classes closed
   there, which were shared machinery. The single item was an UNNAMED owner on
   `ft_remove_one_commit`'s structural edge: `b7334aa4`'s shape at a producer
   that builds its edge inline rather than through a rec. Fixed at the PRODUCER
   (`struct ft_remove_pub` gains `@slot_owner`, filled where both
   `ft_*_node_replace_ptr` arms already hold the node) rather than re-derived at
   the replay — this session paid twice for that shape.
   ☞ The dry run is now clean on ft_unit AND ft_inv `FT_INV_MW=1`, and it claims
   EARLIER than an arm would, so it covers strictly more records than an arm
   converts — the readiness is established for every arm point on this txn, not
   just the first.
   ☑ **IN-PLACE PUBLISH PATH ARMED** `6024f167`, after
   `ft_remove_one_commit`'s last register — correct, and it converts almost
   nothing.
   ☠☠ **AND THE "0.7%" WAS NOT A MEASUREMENT OF THAT ARM.** It was read off
   `armSW`, and that column cannot answer the question: `ft-txn-kind-stats`
   prices a site by TXN CREATION, so an arm that never RUNS and an arm that runs
   and is REFUSED report the same number, and a *different* site hand-arming the
   same txn (the rekey fold) reports as this site's `armSW`.
   `-DFT_B2_ARM_PROBE` (`19befc1f`) splits REACH from REFUSAL, and the real
   number is **10 reaches in a whole ft_inv run** — every
   one of that 34,685 was the fold's. The mechanism is not a defect in the arm:
   an in-place delete needs `ft_in_place_ok()`, which needs an EXCLUSIVE trie,
   which is a trie the per-op arm refuses outright — so on a shared trie EVERY
   delete recompacts (`ft_popcount_node_replace_ptr` returns `-EFBIG`) and only
   the external PROMOTE sub-case reaches that path at all.
   ★ **THE LESSON GENERALISES TO EVERY REMAINING B STEP**: `armSW` is not an
   arm's yield. Pair each arm with a REACH counter, or the next site's zero will
   read the same way.
   ☑ **RECOMPACTION REPUBLISH ARMED** `21b6b559` (both paths) — that is where
   the site commits. They do not share a last register:
   the FUSED path's is `ft_node_recompact`'s RELEASE half ({P}, +{GP} when P is
   compressed), planted inside `ft_node_replace_ptr`; the NON-FUSED path's is
   `ft_flip_txn_lock_or_guard_parent` on its own `else` arm. Refused on the FOLD
   path (`record_only`): there the txn is the CALLER's and the registry's
   completeness is the caller's judgement.
   Measured, WITHIN ONE RUN (ft_inv `FT_INV_MW=1`, 507 threads, per-node):
   `created 5,103,780 / armSW 4,000,657 / SW 3,252,905 / MW_STRUCT 9,827,650 /
   OK 3,999,992 / ABORT 665`, with the probe reading `in-place 10 armed 0 |
   republish A 1,318,159 all armed | republish B 2,801,578 all armed`.
   **`armSW` now tracks `OK` almost exactly**: essentially every committing txn
   at this site is armed.
   ☐ The REMAINDER is ~2.4 records per commit planted BEFORE the arm — the
   reparent sweep's held-child edges, the tombstones, the anchor releases, the
   orphan freezes. Converting those needs the op's ACQUIRES to finish earlier,
   which no arm can do from a publish site; that is Phase C/E, not B2's.
   ☠ A cross-run delta against the unarmed measurement is NOT sound — ft_inv's
   totals move run to run, so columns compare only within one run.
   ☞ `OWN_MISS` barely moves (1,377,786 → 1,405,328) and that is the expected
   shape: the counter prices records on txns the arm refuses outright, which the
   assert exempts via `!nr_locks`. Read the DRY RUN as the readiness signal.
3. The PUBLISH LANE — ☑ **LANDED** `6f54e698`: `ft_pub_rec_add` takes an `owner`
   and all four producers name it. Every such slot is a BODY slot, so the owner
   is the node the slot LIVES IN — which is not always `@parent_nf`, and
   `695d23c1` fixed the one caller that passes it for a different job.
   ☑ REMAINDER LANDED `b7334aa4`: the rec → `ft_ord_cell_edge` conversions
   copied `.root` and DROPPED `.owner` at all three (`ft_pub_rec_sedges`
   assigned `.root` TWICE, and the second of those was the owner), so every
   converted edge reached the engine owner-NULL however well the producer had
   named it. Byte-neutral in a release build — `ft_flip_txn_record_tag` reads
   `@owner` only for the assert and the counters, and parks on
   `@t->structural_sw` alone.
4. `ft-remove.h:882` `ft_chain_compress_fused` — the detach-side creator, and
   the largest CONTENT-LANE abort source. ☑ **ARMED** `ec00e234`, after
   `ft_detach_freeze_orphans` (this op's last register; under `lock_fine` the
   rest was taken in the ONE all-or-none acquire, and the incremental
   `lock_or_guard` beside the publish is the `!lock_fine` arm the helper
   refuses anyway). Refused on the FOLD path.
   ☑ Its dry run `-DFT_COLLAPSE_CLAIM` (`cec68299`) was **clean at first
   reading on BOTH suites** — no fixing pass at all, because the classes B2 paid
   for were shared machinery. `-DFT_ARM_REACH`: 1,645,902 reaches, 1,642,370
   armed (99.8%), only trie-wide refusals.
   ☠ **AND THE ABORT COLUMN NEEDED n=3, ALTERNATED.** One armed run read as a
   wrong-direction alarm (site aborts 5.69 → 6.74 per 100 created) and it did
   not survive. Alternated ctl/arm ×3, per 1k txns created at the site:
   `control 62.57 aborts / 2291.5 MW_STRUCT / 3.1 SW` vs
   `armed 55.44 / 1763.2 / 565.4`. ☞ The control's own spread (51.25 / 72.81 /
   63.64) is WIDER than the gap while the armed leg is tight (56.69 / 55.81 /
   53.82), so what this establishes is the ABSENCE of a wrong-direction signal,
   not a certified 11% ([[feedback_abort_column_is_not_a_per_step_number]]).
5. The EXTERNAL-HEAD lane — split by what the evidence shows, after an
   adversarial skeptic took the first framing apart in BOTH directions.
   ☑ **`ft_unchain_node`'s HEAD CLEAR — ARMED** `8cd52c64`. It never emits a SKIP_X
   dual, by ROUTING: `_ft_publish_to_parent`'s dual arm is gated on
   `ft_node_compressed(@parent_nf)`, and `_cds_ft_remove_locked` sends a
   COMPRESSED holder to `ft_detach_node` (no successor) or to the PROMOTE arm
   (one or more), so this branch only ever sees a NULL or INTERNAL parent. The
   branch comment naming "a compressed holder's SKIP_X dual" is STALE.
   ☞ The arm rests on a DETECTOR (`urcu_assert_debug(n_s < 2)`), not on that
   paragraph — a routing invariant is a code fact and code moves. Red control:
   inverted it does NOT fire on ft_unit (that suite never reaches the lane) and
   aborts 6× on ft_inv `FT_INV_MW=1`. Reach 2,346,297 / armed 2,346,297, ZERO
   refusals; the site goes 2.00 → 1.00 MW_STRUCT per txn (the residue is
   `hold_or_lock_parent`'s own release terminal, which cannot move above its
   own register). `ABORT` 0 before and after — fairness only, as expected.
   ☑ **`ft_promote_head`'s TWO ARMS — ARMED** `1c59dc9c`, after the per-edge
   `@owner_held` fix `af22756b` (option (a) of the fork).
   ☠☠ **THE GAP IT CLOSED, and the reason it was worse than a missing arm:**
   `_ft_publish_to_parent_meta` emits a **SKIP_X DUAL into the GRANDPARENT's
   body**, a node neither arm acquires — and `struct ft_pub_rec`'s claim that a
   NULL `@owner` leaves a record "never eligible for a per-op SW park" was a
   **STALE MECHANISM**: `ft_flip_txn_record_tag` dispatches on `structural_sw`
   ALONE, so an armed txn parks an unnamed slot SW and, without
   `--enable-rcu-debug`, silently.
   ☞ **THE FIX.** `@owner_held` is now a real per-edge word on `ft_pub_rec` and
   `ft_ord_cell_edge`, false by default, false ⇒ MW; carried across all three
   rec→edge conversions and honoured at all three replays. The FORWARD edge
   inherits the caller's `@slot_owner_nf` declaration (so every existing
   conversion is preserved); the DUAL's owner is DERIVED from a back-pointer, so
   `_ft_publish_to_parent{,_meta}` take `@dual_owner_held` and all **19** call
   sites state it, compiler-enforced.
   ☠☠ **AND EVERY ONE OF THEM NOW SAYS FALSE** (`46f33b7c`). The two
   `ft_detach_node` republishes briefly said `old_recompacted_node != NULL`, on
   the ground that "the recompact takes {C,P,GP} exactly when P is compressed".
   That is a **PLAN-TIME** fact: `pf_gp` is resolved BEFORE the acquire and GP
   enters the set only `if (pf_gp)`; an empty member is skipped by
   `if (!set[i].nf) continue` — its GUARD with it — while the dual's slot AND
   owner are derived FRESH at publish. A compressed P that was root-attached at
   plan time yields no GP and no guard, so a peer re-home in that window makes
   the publish derive a grandparent the op never acquired: an SW park on an
   unowned word, at an ARMED site, silent without `--enable-rcu-debug`.
   Unproven-reachable (no current op live-re-homes a root-attached compressed
   node) — **not a defence**. The parameter stays: it is where a publish-time
   answer belongs — the ACQUIRED GP compared against the DERIVED dual owner.
   ☞ Verified on the construction that found it: the public-API repro
   (`fractal-trie-review-2026-06/repro_listoff_promote_skipx_dual.c`) now
   SURVIVES both arms, and both dry runs are clean. Reach 4,847,315 + 115,511,
   ALL armed; the site goes 0% → 33.3% converted.
   ☠ **THE "CONVERSION FRACTION" EVIDENCE IS RETRACTED.** `!owner_held` routes
   through `ft_flip_txn_record_tag_mw`, which counts **MW_ALWAYS** — outside the
   conversion surface by design — so the dual population leaves BOTH numerator
   and denominator of `SW/(SW+MW_STRUCT)`, and the fraction can hold still while
   conversions are lost. The honest figure, per 1k txns created, ONE RUN per
   column (an estimate, not a certified delta): B2's `SW` goes 668 → 600 → 534
   across the mechanism and then the fail-close — a tenth to a fifth of its SW
   records, which is what closing the window above costs. `ABORT` at these sites
   is unchanged.
6. The remaining content sites in descending count.

★ **EVERY ARM FROM HERE CARRIES A REACH COUNTER.** `-DFT_ARM_REACH` lives in
`ft_flip_txn_arm_per_op` itself and tallies reach against each refusal term per
CALL SITE, because `armSW` cannot tell an arm that never RAN from one that ran
and was refused, and credits another frame's hand-arm to the site whose txn it
armed ([[feedback_armsw_is_not_an_arms_yield]]).

☠ A raw `ft_flip_txn_record_tag` loop over ordered-cell edges survives at
`ft-merge.h` and `ft-rekey.h` (the sibling graft path routes through the
tag-dispatching recorder precisely to avoid it). Sound today only because the
modes that arm exclude trie-wide; the per-edge `owner` stops it at the Phase B
arm, and routing the loop through the dispatching recorder is the real fix,
owed with those sites' arm.

Then retire the hand-arming at the rekey writer and root-COW driver onto the
same helper, so the switch has no bypass.

☠☠ **B6 IS NOT BLOCKED — the entry that said so was WRONG, and this is its
retraction.** It claimed the rekey writer's fences "reach the txn only as
RECORDS ... no `ft_flip_txn_lock_register` at all", and that registering them was
closed by the `FT_ENTRY_PER_NODE + 1` registry bound. Both false:

* `ft_rekey_marks_to_txn` (`ft-rekey.h:995`) loops `ft_flip_txn_lock_register`
  over every non-shared mark and is called at **:1836** (right after
  `ft_rekey_cow_stop`) and **:2684**. It landed at **`a91ebafd`** — an ANCESTOR
  of the commit that claimed it did not exist.
* Measured over a full ft_inv `FT_INV_MW=1` run: at the hand-arm point
  `nr_locks == 0` in 383,140/383,140, but immediately after the stop it is
  NONZERO in **285,294/285,294** (max 5) and at the commit in
  **123,689/123,689** (max 15) — against a bound of **257**. The overflow
  argument belongs to `ft_detach_freeze_one`'s orphan chain, a different op.

⇒ **B6 is a PLACEMENT problem, exactly like B1–B5**: the fences reach the
registry, just LATER than the first records — the register-before-record
ordering class already closed once as a 17-site sweep (`55c0350c`). The
"second witness" framing was invented to explain a refusal whose real cause is
that the hand-arm sits at txn CREATION, where every witness is empty.

**The shape of the fix** (small, and half of it is already built):
  1. `ft_rekey_cow_stop`'s `assert(txn->structural_sw)` (`ft-rekey.h:192`) is a
     DECLARATION, not an algorithmic dependency — `structural_sw` is consulted
     only per record at plant time — so it can move below the stop-fence take.
  2. Hoist the idempotent `ft_rekey_marks_to_txn` to the take, and put the one
     sanctioned `ft_flip_txn_arm_per_op(ft, txn)` there, inside `cow_stop`,
     which has both `ft` and `txn`.
  3. Port the same handover to `_cds_ft_debug_cow_replace_root`, which — unlike
     the writer — never adopted `ft_rekey_marks_to_txn`. **The two sites are NOT
     the same shape**, contrary to the retracted entry.
☞ COARSE / exclusive legs need nothing: `ft_txn_content_sw_ok` means the
constructor already armed them, and 39,420 of the measured arm reaches were
`lock_fine == 0` — there the hand-arm is redundant and the swap is a no-op. The
whole B6 residue is the FINE lane.
☑ **THE HOIST WAS IMPLEMENTED AND IT WORKS AT PER-NODE.** `ft_rekey_marks_to_txn`
at every mark-growth point in `ft_rekey_cow_stop`, `ft_flip_txn_arm_per_op` at
the stop take, the assert moved below it, the hand-arms retired at BOTH sites,
and the driver's sweep gaining the `!marks[i].txn_owned` clause registration
requires: ft_unit 315 ok / 3 deliberate, ft_inv `FT_INV_MW=1` 119/119, no
assertion — and the only `set_structural_sw` callers left were the constructor's
trie-wide arm and the helper itself. The predicted ordering findings DID appear
(a marked child's re-parent record ahead of its registration) and registering at
every growth point closed them.

☠☠ **THEN THE GATE KILLED IT AT THE OTHER TWO SPACINGS.** `txndbg`, `anchorval`
and `proxyassert` went RED at **exponential and root-only**, ft_unit dying after
109 tests. The mechanism is the arm's SPACING REFUSAL, not its placement:
`ft_flip_txn_arm_per_op` declines any spacing but per-node, so at a coarser FINE
spacing `structural_sw` stays false — and this writer **cannot run all-MW**.
`ft_reparent_record_meta` (`ft-mutation-helpers.h:9324-9326`): a MARKED child's
edge MUST stay SW, because "an MW edge expecting live_state would mismatch the
op's OWN mark and **abort every commit**". A LIVELOCK, not a slowdown — and
exactly what `assert(txn->structural_sw)` was guarding.

☠☠ **AND "BLOCKED ON PHASE E" IS ITSELF REFUTED — B6 IS LANDABLE NOW.** The
"two doors" argument conflated CALL SITES with POLICIES. The two doors B6 exists
to remove are the two HAND-ARM CALL SITES; a spacing dispatch *inside* the one
sanctioned arm point is ONE door, retires both hand-arms today, and is
byte-identical at every spacing. It is also exactly as sound as the status quo,
because it IS the status quo's policy behind one entry point.
☞ And the coarse spacings it covers are not shippable anyway:
`cds_ft_group_attr_set_lock_spacing` refuses exponential / root-only without
`FEATURE_FT_ANCHOR_VALIDATE` — "not a shippable configuration ... development
sweep" (`ft-lifecycle.h:355-375`). **Phase E is the prerequisite for CERTIFYING
the coarse SW fallback, not for consolidating the arm.**

☑☑ **B6 IS LANDED** `f8b6317f`, on the fourth attempt.  `ft_flip_txn_arm_structural`
is the ONE DOOR: it takes the sanctioned per-op arm and, when that refuses, says
SW explicitly.  Both hand-arms call it **at TXN CREATION** — the one point every
branch of the op passes — and no site outside the two arm helpers touches
`ft_flip_txn_set_structural_sw`.
☞ The per-op arm STAYS at `cow_stop`'s stop fence, additive: only a per-op arm
sets `@dbg_arm_per_op`, and at creation `nr_locks == 0` so it cannot.  Its marks
are registered first at every growth point (register-BEFORE-record), and the
driver's sweep gained `!marks[i].txn_owned` because registration TRANSFERS the
clear.
☞ `assert(txn->structural_sw)` now also sits before the writer's commit — the
choke point every branch crosses.
☠ The fallback line is **Phase E's debt**: a policy branch behind one entry
point, one line to certify or replace.
Verified: per-node ft_unit 315/3 deliberate, ft_inv `FT_INV_MW=1` 119/119,
per-op reach 269,738 / armed 228,109 (all 41,629 refusals trie-wide); the
**nocompress canary** back to 316/2, matching control; all 65 gate legs
identical.

★★★★★ **WHAT THE THREE FAILED ATTEMPTS TAUGHT**, and it generalises past B6:
  1. A CLAIM belongs at a choke point EVERY path crosses, even where the ARM
     cannot live.  Moving the assert with the arm made the un-armed branch the
     un-asserted branch — a deterministic livelock presented as a silent hang.
  2. Ask the LAST CONFIRMED mechanism whether it has another ROUTE before
     inventing a new one.  The livelock was already documented; only its REACH
     was open.
  3. `gdb -p` on a hang is the FIRST move.  A leaked fence stalls the NEXT op at
     an ACQUIRE; a livelock inside ONE op at COMMIT is a different signature.
  4. Gate legs are CONFIGS, not branches.  When exactly one config diverges, ask
     which BRANCH only that config drives (`nocompress` was the only leg driving
     the merge_dst fold).

☠☠ **ROOT CAUSE — the arm was moved onto a branch one caller never visits.**
Retiring the hand-arm at txn creation and putting the helper arm inside
`ft_rekey_cow_stop` leaves the **merge_dst** branch of
`ft_rekey_graft_simple_attempt` with NO arm point at all: `cow_stop` is called
only on `!merge_dst`.  That branch then runs all-MW, and its MW edges on the
state words the op ITSELF fenced expect `live_state`, read the op's OWN LOCK
marks, and abort the commit **deterministically — no peer needed**.  The retry
wrapper re-plans the identical shape forever.  It is the SAME livelock this
entry already documented, reached by a BRANCH-shaped route rather than a
spacing-shaped one.
Proof, from the hung process: `txn->structural_sw = false`, `merge_dst = true`,
`nr_marks = 0` (cow_stop never entered), commit returning `ABORT` repeatedly with
`nr_locks = 4` climbing fresh every round.  `nocompress` is the only gate leg
whose ft_unit drives that fold — with compression on, the shape gates route this
rekey elsewhere.  A COVERAGE ARTIFACT, the §9.5 class again.

☠ **THE `@txn_owned` SUSPECT IS REFUTED.**  No fence is leaked: `acquire_miss`
is false and `nr_locks` climbs fresh each retry (a leaked LOCK would fail the
next round's ACQUIRE, not its commit), and the release audit HOLDS — every
`goto sweep` in the writer is destroy-preceded, `bail_build` routes through
`ft_flip_txn_destroy`, commit-OK consumes registered marks via state edges,
commit-ABORT clears them via `ft_flip_txn_lock_release_all`, and both sweeps skip
`txn_owned`.  The registration half of the patch was always sound.

☑ **THE SMALLEST CORRECT LANDING** (verified: restores `nocompress` to
316 ok / 2 deliberate, byte-identical to control):
  1. Put the arm+fallback pair **at txn CREATION** in
     `ft_rekey_graft_simple_attempt` and in `_cds_ft_debug_cow_replace_root` —
     where the hand-arm was.  At creation `nr_locks == 0`, so the helper always
     declines and the FALLBACK carries correctness; this covers EVERY branch,
     merge_dst included.
  2. KEEP the arm-at-take inside `cow_stop` (idempotent).  At creation the helper
     can never CLAIM (the per-op owner assert needs `nr_locks > 0`), so the
     take-site arm is what buys B6's audit value on that path.
  3. Keep the registration at every growth point and both `txn_owned` sweep
     clauses — they closed the ordering findings and audit clean.
  4. RESTORE `assert(txn->structural_sw)` at `cow_stop`'s top; valid again at
     every spacing and every caller.
  5. **ADD `assert(txn->structural_sw)` immediately before the writer's
     `ft_flip_txn_commit`** — the commit is the one choke point EVERY branch
     crosses, and that assert would have turned this hang into a named abort.
  6. Optional: fold the pair into one `ft_flip_txn_arm_structural` helper so
     neither site has a naked `set_structural_sw`.
★ **THE LESSON THIS COST**: a CLAIM belongs at a choke point every path crosses,
even where the ARM cannot live.  Moving the assert WITH the arm made the
un-armed branch the un-asserted branch, which is why a livelock presented as a
silent hang.

☠ **THE RED CELLS WERE ALREADY RED AT HEAD, and this entry hid it.** On a clean
tree with the gate's anchorval flags, ft_unit dies at **294** (exponential) and
**238** (root-only), and ft_inv MW root-only after **73**, all on the engine's
`urcu_txn_record_chain: r->kind == kind` assert (`rcu-txn-mcas.h:879`) — the
§9.5 class-2 failures. The hoist did not turn those cells red; it died EARLIER
in them (110 vs 294/238). Those pre-existing reds gate any future coarse-cell
green and are NOT B6's.

☞ **THE all-MW LIVELOCK IS REAL BUT NARROWER than stated**: measured, all-MW is
outcome-identical to control at root-only and across the whole ft_unit surface
(every child acquire DEDUPES there), and livelocks only on ft_inv `FT_INV_MW=1`
at **exponential** — 7/119 tests in 480 s, with 112,271,789 child acquires taking
the child's OWN word against 527,664 dedupes. The assert is load-bearing at
exactly one measured spacing.

☞ Per-node measurements, corrected: the writer gives up **~0.6-0.8 records/txn,
~8-12%** of its structural records. Two identical runs gave 0.64/8.4% and
0.81/11.6%, so the single figure this entry first quoted was one draw from a
distribution as wide as its own precision. Reach/armed totals likewise vary ±25%
run to run; what is stable is the refusal SPLIT (all trie-wide, three zeros).

## 5. Phase C — the residual MW_ALWAYS lanes (the G4 decision)

### C.1 — THE RE-MEASURE (Phase B complete, B1-B6 all landed)

ft_inv `FT_INV_MW=1`, 507 threads, per-node, `--enable-rcu-debug`
`-DFT_DEBUG_TXN_KIND`, against §4's pre-Phase-B baseline. Normalised per 1k txns
created, because the two runs differ in size (94.9M vs 88.6M):

    lane          baseline      post-B6 |  base/1k  post/1k   delta
    SW           3,472,296   21,752,370 |     36.6    245.7   +571%
    MW_STRUCT  136,857,021   44,580,604 |   1442.1    503.4    -65%
    MW_ALWAYS   85,087,969  149,372,031 |    896.6   1686.9    +88%
    MW_LOCK     52,667,070   46,413,021 |    555.0    524.1     -6%
    VALIDATE    23,383,128   18,173,415 |    246.4    205.2    -17%
    cell/hlist  17,551,703   14,565,802 |    184.9    164.5    -11%
    ABORT        2,415,932    1,906,296 |     25.5     21.5    -15%

☑ **THE CONVERSION SURFACE IS DOWN 65%** and SW is up 6.7×. That is Phase B's
result, and it is large enough not to be run noise.

☠ **THE ABORT NUMBER IS NOT.** −15% is inside the ±40% run-to-run spread
measured on this column ([[feedback_abort_column_is_not_a_per_step_number]]), and
§4's own ceiling for the whole conversion was ~29% of aborts (the content lanes;
71% are the ACQUIRE lane, still 46.4M MW_LOCK records here). Read the direction,
not the figure. A defensible number needs n≥3 alternated against a control
build, which C should run before anything is claimed. ⇒ **RUN in C.1(c) below.
The −15% is superseded: the plan-wide column does not survive n=3, and the lane
the conversion actually touches does — by −34.5%.**

☠☠ **MW_ALWAYS NEARLY DOUBLED, and part of that is BOOKKEEPING, not traffic.**
`af22756b`'s per-edge `@owner_held` routes every unheld SKIP_X dual through
`ft_flip_txn_record_tag_mw`, which counts MW_ALWAYS — so edges that used to sit
in MW_STRUCT (or in SW, unsoundly) now land here. **The split between real
MW_ALWAYS traffic and reclassified duals is not measured, and G4 must not be
decided on this number until it is.** That is C's first task, and it is cheap:
count the dual edges at their producer. ⇒ **DONE in C.1(b) below — and the
suspect named in this paragraph turned out to be 0.4% of the column.**

### C.1(b) — MW_ALWAYS DECOMPOSED, and this plan's own suspect was WRONG

`ft_flip_txn_record_tag_mw` now takes a CLASS naming the population its caller
joins — required in the instrumented build, absent from every other one, so the
default `.text` is byte-identical (proved: same size, the only differing
instructions are 30 `mov $imm,%edx` assert `__LINE__` constants). Eleven record
branches, nine classes, and the classes SUM to the MW_ALWAYS column exactly —
that equality is printed as the table's self-check, so a future branch added
without a class announces itself instead of vanishing into a residue.

Two runs, same rig as C.1 (ft_inv `FT_INV_MW=1`, 507 threads, per-node,
`--enable-rcu-debug`), 119/119 both times, sum == column both times:

    class          run 2 records   share   run 1 share   fate
    HEAD_BACK         53,670,264   36.0%      36.2%      NEVER converts (no state word)
    PARENT_WORD       39,082,973   26.2%      25.7%      lock-set REACH  ⎫ the recompaction
    STATE             39,082,973   26.2%      25.8%*     a VALIDATE      ⎭ re-parent sweep
    CELL               8,435,084    5.7%       6.2%      ☞ THE G4 LANE
    ROOT               4,185,351    2.8%       3.0%      NEVER converts (no node to lock)
    PSO                4,033,026    2.7%       2.7%      §8.3 retires it
    DUAL                 526,699    0.4%       0.4%      ☠ THE SUSPECT ABOVE
    GUARD                127,955    0.1%        —*       a VALIDATE
    RANK                  19,582    0.0%       0.0%      Phase E (root-only spacing)
    sum              149,163,907             (* run 1 counted GUARD inside STATE)

**READ THE SHARES, NOT THE PER-1k.** Between these two runs the same build gave
1527/1k and 1776/1k for the same column while every share moved ≤0.5pp — so the
NORMALISER is the noisy part here, not the populations. Compare columns within
one run, exactly as the per-1k tables above already warn.

☠☠ **THE RECLASSIFICATION IS REAL AND IT IS THE WRONG COMMIT.** `af22756b`'s
duals are 0.4%. The reclassification that moved ~36% of this column out of
MW_STRUCT is **`f79438e7`** — it converted eight external-head back-edge sites
from `ft_flip_txn_record_reserved` (which counts MW_STRUCT on an unarmed txn) to
`ft_flip_txn_record_head_back_edge` (always MW). All three suspects
(`5fa631c7`, `f79438e7`, `af22756b`) landed AFTER §4's baseline was recorded, so
the whole HEAD_BACK population was MW_STRUCT in that baseline — 553-639 records
per 1k txns, against a total rise of ~630/1k. ⇒ The doubling is bookkeeping, as
this plan suspected, of a lane this plan had already ruled on.

☑☑ **THE LARGEST POPULATION IS THE RECOMPACTION RE-PARENT SWEEP** — PARENT_WORD
and STATE are the same records to the unit (39,082,973 each: two words per
re-parented child), 52.4% of MW_ALWAYS and **1.8× the whole remaining MW_STRUCT
conversion surface** (43.4M). This is the "B2 remainder" Phase B scoped out
because no arm at a publish site can reach it, now with a size. Its two halves
have DIFFERENT answers and that is why they are counted apart:

* STATE is a `{live -> live}` VALIDATE on a child the sweep does not hold, and
  it must stay MW — a park validates nothing. Marking those children instead
  was implemented in full and does not live (a contended child fails the
  acquire; escalation cannot rescue it — it holds its FIFO turn while spinning
  for a holder funnelled behind that turn).
* PARENT_WORD is a real STORE, and it is MW only because the sweep's acquire
  takes {C,P,(GP)} and never C's children. With PSO (2.7%) it is 28.9% of the
  column and it is §8.3's prize, quantified for the first time: parent-owned
  edge WORDS make both plain stores under the junction lock.

### C.1(c) — THE n≥3 ALTERNATED ABORT MEASUREMENT

Control `c8ce38c2` (the commit that recorded §4's baseline) against HEAD, both
`--enable-rcu-debug -DFT_DEBUG_TXN_KIND -O1`, ft_inv `FT_INV_MW=1`, 507 threads,
per-node, alternated ctl/arm ×3, SEQUENTIAL, 119/119 on all six.
☑ `test_urcu_ft_inv.c` is byte-identical between the two commits, so both legs
run the same workload — checked, not assumed.

Aborts per 1k txns created, split by lane at the txn CREATION SITE (an acquire
txn is one whose site records MW_LOCK and VALIDATE and nothing else):

    lane      control runs          armed runs         paired Δ       verdict
    TOTAL     24.86 22.58 18.22     19.81 17.72 14.43  −20.3/−21.5/−20.8  ☠ OVERLAP
    ACQUIRE   18.09 14.20 11.71     15.19 12.41 10.21  −16.0/−12.6/−12.8  ☠ OVERLAP
    CONTENT    6.77  8.38  6.51      4.62  5.31  4.22  −31.7/−36.6/−35.2  ☑ SEPARATED

☑☑ **THE CONTENT LANE IS −34.5%, AND IT SEPARATES COMPLETELY.** Every control
run's content-abort rate (min 6.51) is above every armed run's (max 5.31) — all
nine cross-comparisons agree, which at n=3 v 3 is the strongest rank result
available (p = 1/20 one-sided). The three paired deltas span 4.9pp. **This is
the conversion's own lane and this is its number.**

☠ **THE PLAN-WIDE COLUMN STILL DOES NOT SURVIVE, AND NOW THE REASON IS NAMED:
IT DRIFTS.** Both legs fall monotonically across the session — control 24.86 →
22.58 → 18.22, armed 19.81 → 17.72 → 14.43 — and the ACQUIRE lane carries the
whole drift (18.09 → 14.20 → 11.71). The CONTENT lane does NOT drift (6.77 →
8.38 → 6.51, non-monotone), which is exactly why it separates and the other two
do not. A paired Δ on a drifting column credits the arm with the drift, so
−20.9% plan-wide is NOT a result; −34.5% on the content lane is.
⇒ **Any future abort claim must be made PER LANE and paired within a session.**

☞ **AN UNEXPLAINED SECOND-ORDER EFFECT, AND IT IS NOT CLAIMED HERE.** The
ACQUIRE lane fell too (−13.8% paired, direction consistent in 3/3), and the
conversion does not touch it. The mechanism that would explain it is real: an
MW_STRUCT record on `&meta->state` (a fused count, a tombstone) is a CAS
contender on the very word the acquire's take arbitrates, so converting it to
an SW park removes a competitor from the acquire's own arbitration point — which
would mean §4's "~29% ceiling" UNDERSTATED the conversion's reach. It is a
hypothesis. It shares its column with the drift above, so it cannot be read off
this table; the test is a state-word-only conversion measured against the
acquire lane alone, and Phase D owns it.

☑ §4's ceiling holds as a share: CONTENT is 27/37/36% of all aborts on the
control and 23/30/29% armed.

### C.2 — G4's actual input

With MW_STRUCT at 503/1k, the residue is dominated by MW_ALWAYS (1687/1k) and
MW_LOCK (524/1k). MW_LOCK is the lock take and stays MW forever. So the G4
question — does the ordered-cell / hlist lane stay MW? — is now the largest open
lever, and `cell/hlist` (164/1k, unattributed by site) is only the part recorded
straight on the engine handle; the rest is inside MW_ALWAYS above.
☑ **MW_ALWAYS IS DECOMPOSED (C.1(b)), and G4's lane is now a number.** The
ordered-cell / duplicate-chain lane is CELL (8.4M inside MW_ALWAYS) plus the
`cell/hlist` line recorded straight on the engine handle (14.7M) =
**23.1M of 252.8M MW records, 9.1%** (run 1: 27.2M of 274.0M, 9.9%). Against
this section's own decision rule — "if the cell lane is minor, keep the narrow
MW lane" — 9% on the same order as §4's expectation (17.5M of 315M) is the
MINOR reading, and the mixed commit already backs a cell conflict out clean
before any SW side effect. ☞ **G4's remaining input is therefore the ABORT
attribution, not the record volume**: what fraction of aborts is the cell lane.
C.1(c) has now run the alternated measurement and split aborts ACQUIRE vs
CONTENT — but not CELL vs the rest of the content lane, and it cannot: the split
is done at the txn CREATION SITE, and a content txn carries cell and structural
records together. ☠ The obvious proxy — "aborts at sites that plant cell
records" — is a loose bound, not attribution
([[feedback_a_correlated_flag_is_not_attribution]]): the top content-abort sites
(ft-remove.h:881, ft-insert.h:866, ft-graft.h:3496) all plant both.
⇒ **The exact answer needs the LOSING RECORD's class**, which the engine has at
`t->recs[planted]` in `urcu_txn_desc_commit` and did not report. ☠ And the
proxy TAG cannot stand in for it: `FT_STATE_PROXY`, `FT_NR_KEYS_PROXY_TAG` and
`URCU_TXN_TAG` are all `1`, so the tag separates structural from everything
else and nothing more — checked. ⇒ **BUILT (Mathieu approved the engine touch):
C.1(d) below.**

### C.1(d) — ABORT ATTRIBUTED TO THE RECORD THAT LOST IT

`eaf883af` (engine, default-inert) + `9e263841` (the FT label). Same rig, two
runs, 119/119 both, and **the classes sum to the ABORT column exactly with ZERO
unattributed** — which is also the proof that the three engine exits cover every
abort, the lone-MW-edge fast path included.

    losing record's class    run 2 aborts   share   run 1 share
    MW_LOCK                     2,178,560   86.3%      78.1%     the lock take: losing IS its job
    mwa:STATE                     261,276   10.4%      15.2%     the re-parent sweep's §4.B validate
    MW_STRUCT                      61,347    2.4%       4.8%     ☠ see the alarm
    mwa:CELL                       22,463    0.9%       1.9%     ☞ THE G4 LANE
    VALIDATE                            —      —        0.0%
    mwa:PARENT_WORD                     —      —        0.0%

☑☑ **G4's LANE IS 0.9-1.9% OF ABORTS.** The ordered-cell lane is 6% of the
always-MW record volume and **~1% of what actually aborts**. Against C.2's own
decision rule — "if the cell lane is minor, keep the narrow MW lane" — this is
as minor as the rule can be given, and it closes the volume-vs-abort gap C.1(c)
left open. A per-cell lock would buy ~1% of aborts and cost a state word per
cell plus neighbour serialization. ⇒ **KEEP THE NARROW MW LANE.**

☠☠ **AND THE DETECTOR FIRED, WHICH IS THE REAL RESULT.** 60,727 of the 61,347
MW_STRUCT losses are records whose owner the op **HOLDS** (79,445 of 80,179 in
run 1) — and **every one of them is on a child POINTER slot**, zero on the
packed state word. That distinction is the whole finding: a state word has
lock-free writers by design, a child pointer slot may only be written by a
holder of the owning node's lock. Two runs agree, and it is concentrated at
**exactly two sites, both in the GRAFT lane**:

    ft-graft.h:3496 create    ptr 56,461 / state 0      (glue_insert.txn)
    ft-graft.h:3520 bounded   ptr  4,266 / state 0      (glue_publish_txn)

Every Phase B armed site reports **zero**. `ft_flip_txn_owns` is registry-only
with no trivially-true arm, so the hold is real
([[feedback_the_first_question_at_a_claim_abort_is_lock_fine]] cleared).

☞ **WHAT IT MEANS TODAY, AND WHAT IT DOES NOT.** Unarmed, the record is MW, the
CAS loses, the op retries, and nothing is published wrong — this is not a live
correctness bug. What it says is that **the graft lane must not be armed**: an
armed txn parks that word SW, and an SW park neither arbitrates nor is visible
to the peer that just won. The candidate causes are not yet separated —
a peer writing that slot without the lock, an `owner` naming the wrong node
([[feedback_a_raw_rederivation_at_the_write_site_is_a_class]], three known
instances), an expected-old captured before the acquire, or the cross-trie shape
(`glue_insert.txn` is created on `dst_ft`). 

### C.1(e) — THE CLAIM DUMP: the owner is CORRECT, and the question moves

`f2eaa3e1`. `-DFT_ABORT_CLAIM` prints, at the abort, every record in the
descriptor — class, kind, witness, {slot, old, new}, the slot's LIVE value, the
tag, the owner the record NAMED, its call site — with the loser marked, then the
op's LOCK REGISTRY, which the engine cannot see. Three independent samples,
identical shape:

    rec[0] MW_STRUCT MW/held slot=..0148 old=..1a1 new=..0e1 now=..081 tag=0xf  <== LOST
    rec[1] MW_STRUCT MW/held slot=..2f8  old=0x80004 new=0x4    now=0x80004 tag=0x1
    registry: nr_locks=1   held[0] meta=..2d8

`rec[1]` is the held node's own state word (`meta+0x20`, `offsetof(state)`
confirmed) — the `{LOCK|s -> s}` release. `rec[0]` is a child POINTER slot whose
value cycles among exactly **three** tagged pointers across samples: several
peers are writing that one word. addr2line through the inlined chain names it
exactly: `ft_glue_txn_commit_edges` → `ft_flip_txn_record_pub_rec` →
`ft_flip_txn_record_reserved` — the glue's FORWARD PUBLISH, whose owner is
`@publish_parent` by construction (`_ft_publish_to_parent` passes
`slot_owner_nf = parent_nf`).

☠☠ **AND THE "MISNAMED OWNER" EXPLANATION IS REFUTED.** The raw-re-derivation
class was the obvious suspect and the two addresses are 2 MB apart — but that is
an inference from an ADDRESS, and metadata and node bodies live in separate
arenas, so the gap says nothing
([[feedback_print_dont_debug_a_value_the_code_has]]). `ft_slot_in_node` is the
pairing test for exactly this question — its own header was written for it —
and counted at the publish it reads **in 1,231,998 / NOT-IN 0**. The slot IS one
of the named parent's own child slots, every time.

☞ **SO THE FINDING NARROWS RATHER THAN DISSOLVES**: the op holds P's DLM lock,
writes a child slot inside P, and a peer still wins that word. The remaining
question is about the **WINNER, not the loser** — who writes a child slot of P
without holding P — and it needs a different instrument (stamp the writer's
identity into the value, or trace the word). ☠ The two surviving candidates are
a peer path that writes that slot without taking the lock, and the CROSS-TRIE
shape (`cds_ft_graft_swap`'s glue txn is created on `dst_ft` while the oracle
`gs_shared_writer` swaps subtrees between two tries). Not separated.

★ METHOD NOTE, paid for twice in this section: a counter that reads 0 was quoted
here as refuting the forward-publish reading, and the call site had never been
added — two patch scripts aborted before reaching it. `in + NOT-IN` is a REACH
counter and it is what caught the mistake
([[feedback_verify_the_mechanism_ran_before_believing_a_zero]]).

☠ COVERAGE: ft_unit reports ZERO aborts, so the detector has none there. It
lives on ft_inv MW.

☠ **AND THE ABORT COLUMN'S SPREAD IS NOW MEASURED ON THIS BUILD**: the two runs
above are the same binary on the same rig and reported ABORT 2,981,880 and
1,514,561 — a factor of 2.0. Any abort claim smaller than that is noise.

### C.1(f) — THE WINNER NAMED: the alarm is the graft swap's own conflict
### detection, and the exclusion is INTACT

The winner-side instrument (`-DFT_WINNER_DBG`, on two new default-inert engine
hooks). The PRIMARY witness is the value the losing CAS **observed**, captured
by the engine at the loss itself (`URCU_TXN_REC_LOST` — the compare-exchange
returns that word and it had always been thrown away; any later re-read races
every subsequent writer). A proxy observed there IS the winning record,
decoded directly. A plain value is matched against a WINNER LEDGER — a global
slot-keyed table of the last engine write {label, the owner the writer named
(its own ring, its own thread), record call site, tid}, fed by
`URCU_TXN_REC_WROTE` after every engine slot store, try-claim only — and a
match is **corroboration, never proof** (ABA can name an older write of the
same value). The non-engine release-store lane (`ft_ord_cell_flip_one`, the
remove head-promote store) files raw notes so a raw winner is provable rather
than inferred from a miss; two loser-side probes check the re-home candidate
at the alarm itself (registered owner TOMBSTONED? `parent_word` NULL?).

**THE ANSWER** (ft_inv MW, 507 threads, per-node, 119/119; claim sample
agreeing, inside `inv_graft_swap_shared_dst_*`):

    alarms 81,859 (create 77,364 / bounded 4,495), ledger reach 505M stamps
    engine winner 81,859 = 100%, label MW_STRUCT witness=HELD 100%
    winner's named owner == the loser's 100%; self-tid 0; proxy 0
    raw-lane 0 / ledger-mismatch 0 / no-entry 0 / torn 0 / DRIFT 0
    loser's owner at the alarm: TOMBSTONED 0 / MID-RE-HOME 0
    claim sample ra: ft_glue_txn_commit_edges → ft_flip_txn_record_pub_rec
      — the glue FORWARD PUBLISH, the same site the LOSER records through

⇒ **The winner is a racing glue commit that held P's lock, published the same
graft-point slot, and released before the loser acquired.** Two ops cannot
hold one DLM lock at once, so the loser's expected-old predates its own
acquire — and it does, BY DESIGN: `@publish_old` is the PLAN-SNAPSHOT
expected-old `cds_ft_graft_swap` records precisely so a peer that swaps the
graft point between the descent and the commit turns into an abort +
re-descend instead of a publish over the peer's attach. DRIFT 0 over 81,859
alarms is the exclusion's own witness: once the loser holds P, nobody writes
that slot. Both surviving C.1(e) candidates are DEAD: no peer writes without
the lock, and the cross-trie / re-home shape never fired its probes.

**SO THE DETECTOR'S PREMISE GAINS ITS ONE EXCEPTION**: "an owner-held
structural record cannot lose" is true of a record whose expected-old was
read UNDER the held lock; the graft forward publish deliberately carries an
OLDER one. Not a bug — and the lane is still **NOT ARMABLE**, now with the
precise reason: an SW park does not CAS, so it cannot implement the
plan-snapshot conflict check; arming would ratify the stale plan, which is
the lost update `@publish_old` exists to prevent.

☞ FOLLOW-UP (small, optional): stamp records whose expected-old is a declared
plan snapshot (`publish_old_set`) with a label bit and EXEMPT them from the
alarm — the alarm then returns to being a true invariant (today it is 100%
this expected lane, and a real exclusion violation would drown in it).

☠ METHOD, paid twice in this section: (1) a value-match ledger WITHOUT the
raw lane hooked and WITHOUT the captured observed value gave the same headline
number — but only coincidentally, and it could not have defended it; (2) the
second attempt misfiled every winner as "raw" because
`snap.code >= FT_AB_CLS_NR` compared the FULL label, which carries the
ownership witness at bit 8 (`MW_STRUCT/held` = 0x102). ONE `-DFT_ABORT_CLAIM`
sample printing the fields the counter had already binned caught it — a
witness-bearing code must be compared by its CLASS FIELD, and a counter's
first reading should always be checked against a sample that prints what was
binned.


After Phase B, re-run the counter baseline. The decision input Mathieu asked
for is the granularity comparison: what fraction of remaining aborts and
retries is the CELL lane (vs the acquire lane), and what would a per-cell
lock serialize (adjacent-key ops share neighbors by construction)?

* If the cell lane is minor (expected: cells were 17.5M of 315M records):
  **keep the narrow MW lane** — the mixed commit already backs a cell
  conflict out clean before any SW side effect, which is the property a cell
  lock would buy, without a state word per cell or neighbor serialization.
* If it dominates: cells grow a state word and join lock-sets per the §9
  decision (cells are ordinary lock-set members in the same commit) — a
  larger, separately-planned workstream.

Dup-chain splices follow the same decision; rank propagation does not — its
unlocked-ancestor stores are only SW-able where the whole path is covered,
which is the root-only-spacing / rank-stats shape (Phase E folds that in).

## 6. Phase D — acquire-side liveness (parallel track)

The conversion does not touch the dominant abort source (71% acquire lane),
and the measured starvation lives there: a `cds_ft_remove` can lose 100+
attempts to a DLM acquire miss (159/179 never ran a commit); root cause is
lock-holder preemption on the hottest word (6,611 takes/100ms, median hold
160ns, the four longest holds each one involuntary context switch). Two
constraints are already proven and MUST bound any fix: a lock acquire cannot
ride the escalation lane (circular wait — structural), and plain backoff on
the acquire is REFUTED (median 40→61, worse tail). Aging-on-refusal is
landed and is the current best (4 starving removes vs 9.5).

Direction to design, with Mathieu: a FAIR HANDOFF on the contended word
itself (queue the acquire beside the engine, the way the COARSE lock is a
`cds_fair_mutex` beside the lane), or accept coarser spacing as the
mitigation on hot tries once Phase E certifies it. On QSBR the retry loop has
no quiescent window at all — any queued design must park offline, not spin.

### D.0/D.1 — the tail re-measured AND attributed at HEAD (2026-08-27)

Instrument: `-DFT_DEBUG_REMOVE_RETRY_CAP` (landed, byte-neutral by default;
the rekey cap detector's twin), with three per-op refusal classifiers.
Removes reaching 100 attempts, 8 sequential reps each, idle 192-core box:
`inv_prefix_key_park_vs_holder_churn` 7 8 5 9 11 8 9 5 (median 8);
`inv_writer_progress_chainmerge` 2 3 5 11 19 3 3 6 (median 4.5); the
1000-attempt milestone 0 of 17 runs.  ⇒ Phase B's −65% MW_STRUCT did NOT
dissolve the tail, and the enrolment fix's gain held (no 374-class runs).

☑☑ **THE ATTRIBUTION INVERTS THE DESIGN PREMISE.**  Of 61 victims at the
100-attempt milestone, 52 report `dirtyLOCK=100 dirtyOTHER=0 cabort=0` and 7
more `dirtyLOCK=99 cabort=1`: the victim's refusals are ~100% `ft_dlm_lock`
pre-checks OBSERVING THE WORD HELD, with zero-to-one set-commit losses.
Every victim is at the lane head (`in_fallback=1, active=1`).  So the
residual tail is NOT an arbitration loss among runnable contenders — with
the lane already silencing the crowd at begin(), the victim burns a full
re-descend+replan per attempt only to re-sample a word that is STILL HELD
across its whole episode (100 µs-scale attempts fit a single 119–380µs
preemption-stretched hold; on an idle box, chained/long RUNNING holds — the
08-20 p99 was 2.2µs, max 33µs — produce the same observation and the
hold-length distribution needs a re-capture to split the two).

**Consequences for the direction sentence above, all skeptic-verified
(adversarial review, 2026-08-27):**

* A RESERVATION / priority-token scheme (peers defer at begin) is REFUTED
  three ways: the defer point sits under held locks on COARSE/nested paths
  (inflating the very holds under attack); the claimant either honors
  `domain->active` and parks at the lane's TAIL exactly when starving, or is
  exempted and voids the lane's landed termination argument; and the
  arithmetic — a begin()-time defer is a ~2% attempt-rate shift placed
  µs upstream of the take, where flipping a streak would need a 10–30×
  cut.  D.1's data then moots the whole family: there is no crowd left to
  thin.
* A FAIR-HANDOFF QUEUE on the word survives the wait-shape objections only
  as GRANT-AT-RELEASE (nobody parks, nobody spins; the release terminal
  grants {LOCK|snap → LOCK|next}), but that holds the hottest word for an
  ABSENTEE winner across its re-descent (~100× duty-cycle inflation) and
  needs lease machinery for a winner that never returns.  D.1 moots this
  family too: the victim is not losing races at the release instant.
* What the data actually prices: the cost of ONE MORE SAMPLE of a held word
  is a full re-descent.  The un-refuted directions are (i) a BOUNDED linger
  at the refused word (µs-scale, online, at `ft_dlm_lock`'s pre-check — the
  inverse of the refuted backoff: the victim keeps its temporal position
  instead of unwinding; must be refused/capped where earlier frames are
  held), (ii) release-side notification (futex-class wake on release; a
  protocol change on the release terminal), (iii) rseq time-slice extension
  for holders (the preemption half only; the fair-mutex header already
  anticipates it), (iv) accept-and-bound (the cap as a loud detector; the
  tail is heavy but bounded on idle boxes).  Sizing (i)'s bound needs the
  CURRENT hold-length distribution: re-run the 08-20 LTTng dlm_take/drop
  capture at HEAD first (D.2).

### D.2 — the LTTng capture: the holds are LEGITIMATE BULK-OP TAILS (2026-08-27)

Mathieu named three candidate scenarios — (1) an acquire refusal forfeits the
FIFO turn, (2) descent/acquire under a NULL domain, (3) a bulk mutation
legitimately holds the lock long — and suspected (3).  The rig (committed
this time: `cds_ft:dlm_take/dlm_drop/dlm_long_hold/remove_anchor_starved`,
`442ff66f`) confirms (3) with the mechanism named:

    one snapshot window, full ft_inv, traced -O2 -DNDEBUG:
    546,877 takes; 19,025 holds ≥ 50µs — median 81µs, p90 122, p99 238
    86%  ft_node_recompact:1357   (then detach_node:2727,
         insert_dlm_acquire_split:771, detach_orphan_planlock:693,
         chain_compress_fused:1009)
    98.3% of long holds: nivcsw = 0 — the holder RAN the whole time
    99.8% of takes carry a domain-bound op (scenario 2 retired: 0.2%,
         all recompact:1357 / insert_split:771 — worth a look, not a cause)

Untraced churn victims agree (the 200µs word-sampler): the refused word is
held with `trans=0` across the whole window, take ages 160–535µs, takers
`ft_unchain_node:4147` / `_cds_ft_insert:3329` — i.e. a DLM hold lasts from
the acquire-set commit to the release terminal, the compound op's WHOLE
build+publish tail.  Scenario (1) is true by construction (the bail must
forfeit — the circular-wait rule) but secondary: with the lane parking every
enrolled peer, the victim's collisions are with these running holds, not
with a crowd.  ⇒ The 08-20 "lock-holder PREEMPTION" conclusion was the
under-load special case; the idle-box steady state is scenario (3).

### D.3 — the bounded-linger palliative, MEASURED (2026-08-27, @d3fe955a)

The retry-edge linger (spin on the refused word after the bail, nothing
held, 200µs cap) was skeptic-amended THREE ways before measuring — TLS word
cleared at every begin (a stale word is the refuted blind backoff), gated on
the ESCALATED state (aging is attempt-denominated), and the A/B run with the
attempt-triggered milestone compiled out (`-DFT_REMOVE_TAIL_QUIET`) since it
biases the control arm.  Clean rig (-O2 -DNDEBUG, wall-clock >1ms metric,
interleaved, sequential):

    churn      slow/run 136.5 → 87.5 (−36%); starved-class 71.5 → 38 (−47%)
    chainmerge NO separation (311 vs 258, overlapping, lin outliers above)
    max wall   ~10–20ms in BOTH arms — untouched
    ops        within spread (~−3%)

⇒ A CLASS-SPECIFIC palliative: it deflates attempts/CPU for the
stable-single-word victim class (churn's unchain/insert holds) and does
nothing for the rotating {C,P,GP} set class (chainmerge's recompact) —
exactly the skeptic's prediction.  No latency-tail win anywhere (sub-µs
inter-hold gaps cannot be captured by a µs wake-to-acquire path), and no
tail growth from the de-escalation window.  Landed DEFAULT-OFF; whether it
ever turns on is a separate decision from the curative work below.  The
~10ms stall class visible in both arms is unattributed — a candidate next
question.

### D.4 — the ~10ms stall class ROOT-CAUSED: the lane's 10ms poll quanta (@d41219c9)

The class D.3 left unattributed (removes >8ms, ~20% of the >1ms
population and its whole extreme tail, including attempts=0 ops, both
arms).  Decomposition put the wall in `urcu_txn_begin` (the domain lane)
or the bail's `end()`, clustered just above exactly 10ms.  The constant
is the adaptative-wait fallbacks: the fair mutex's granted-waiter
TEARDOWN wait (1000 cpu_relax then `poll(NULL,0,10)` — unwakeable BY
DESIGN, the granter must never touch the node after TEARDOWN) and
wfcqueue's `sync_next` busy-wait on the unlock side (same quantum).
Verified by per-thread poll counters: 6 victims ate the teardown poll,
17 the sync_next poll, each in its predicted column — and the ~446
others counted ZERO polls of their own, because the lane is FIFO: **one
thread sleeping its 10ms quantum mid-handshake convoys the whole
enrolled queue** (23 pollers × 10–13 queued writers ≈ the downstream
victims).  A µs-scale handshake race lost costs 10ms × queue depth.

☑ **FIXED @2d857e80 (Mathieu's design): the graduated ladder** —
busy-wait, then nanosleep 10→640µs doubling, then poll 1/2/4/8ms, then
16ms per rung.  A lost race now costs ~10–60µs typical (timer slack is
the floor), ≤2× overshoot, instead of a flat 10ms.  Measured: removes
>8ms 96→0, max wall 17–20ms→4.9ms on churn; test_fair_mutex green;
ft_unit/ft_inv smoke green; the full 64-leg gate's reds are exactly the
documented set, and a pre-fix CONTROL run of the txndbg config
reproduces its 15 legs identically, abort for abort.  Two recordings
from the verification: §9.5 class 2's signature is the engine's
`r->kind == kind` assert (rcu-txn-mcas.h:1000) via
`ft_flip_txn_guard_parent`, and the class covers the ft_inv root-only
legs too, not only ft_unit.  ☐ REMAINS OPEN: a small ~10ms residue on
chainmerge that is NOT this mechanism — body-located (inside
`_cds_ft_remove_locked`), zero rung-counter hits, tightly clustered
just above 10ms, convoying the lane secondarily (~4/run).  The next
unattributed constant.

Note the two classes compose: D.1–D.3's starvation is 1–8ms of
legitimate bulk holds; D.4 was the lane's own handoff machinery
quantizing rare µs races into 10ms convoys.

### D.5 — the ladder is a PUBLIC API, and the residue is closed (@027eeb9d)

Mathieu's call: the graduated delay lifted into `urcu/wait-ladder.h`
(header-only; schedule constants overridable; `rung_us`/`sleep_us`/
`clamp` primitives) and EVERY poll-for-delay site converted — wfcqueue
(hooks preserved, ms-only overriders keep their every-sleep contract),
fair-mutex, urcu-bp's GP re-check (clamped 8ms), workqueue pause
(clamped at the old 1ms — the loop spans fork) and RT idle (8ms),
compat-futex (POLL RUNGS ONLY: the function is documented
async-signal-safe and nanosleep is not on the POSIX list), and the four
DISTRUST trylock loops.  The adversarial review's amendments are all
folded in.  Verified: 64-leg gate IDENTICAL PER LEG to the ladder
baseline, fair-mutex/bp-torture/fork/FT suites green.

☑ **The chainmerge/churn in-body ~10ms residue is CLOSED as a
machine-load artifact** (@da868037): it fires only while concurrent
builds saturate the box, every code-owned wait column reads zero (GP
bracket: gp_calls=0 on 269+ slow removes — the remove body runs NO
grace period on these workloads; ladder rungs 0; arena waits 0 idle),
and its one discriminated instance is a single ~10ms deschedule
(nvcsw=1 nivcsw=1) convoying the lane.  The instrument's new
begin/body/bail + gp/arena/rusage columns make any future occurrence
self-attributing.  Under real oversubscription the mitigation is
scheduler-side (rseq slice extension for lane holders), not wait
policy — noted for Phase F.

**What this does to the fork above:** the starvation is priced by
(bulk-op hold time × recompaction rate), so the CURATIVE lever is
SHORTENING THE HOLD — which is §8.2 in-place mutation's exact target
(~130 structural records per recompaction commit, re-parenting every
child), now carrying a LIVENESS justification on top of its throughput
one; a build-outside-the-lock / flip-inside recompaction shape is the same
lever.  The victim-side BOUNDED LINGER (~1–2× median hold) remains the
cheap PALLIATIVE for the retry-storm cost (100 re-descents per episode),
and rseq slice extension addresses only the 1.7% preemption sliver.
G5(D)'s freeze-drain bound inherits the hold tail: p99 ~240µs + one GP.

## 7. Phase E — lift the lock-spacing gate

The acquire-site conversion is complete and build-enforced; what remains is
certification, then the API gate lift (`ft-lifecycle.h:369`):

1. `ft_compact_relocate_at` still passes a NULL lock context — plumb the
   depth `ft_compact_descend` already tracks.
2. An EXCLUSION oracle per spacing that fails by VIOLATION, not by livelock —
   strengthen `FEATURE_FT_AGREEMENT_RED` (today it reports only by hanging
   >36x, a weak signal to certify 40 sites with).
3. `FEATURE_FT_HOLD_TRACE` (self-collision ledger) clean across the full
   suite at exponential and root-only.
4. Bench the spacings (`-O2 -DNDEBUG`, sequential runs); pick the default
   from data.
5. Lift the `FEATURE_FT_ANCHOR_VALIDATE`-only refusal; then fold the
   redundant strategy: rank-stats coercion retargets ROOT_ONLY spacing, and
   LOCK_COARSE becomes an alias (or is retired) once root-only spacing
   matches it on the bench.

## 8. Phase F — the payoffs (strictly last)

### 8.1 §8.3 layout split

Move the parent-owned fields (`parent_slot_offset`, `incoming_byte`) out of
the state/alloc words into their own parent-owned word. Under MW the sharing
costs only spurious aborts; under SW content it is what turns the remaining
CAS loops (`nr_child`, pso) into plain stores under the held lock, and it
retires the G5(A) fan locks: once the edge fields are parent-owned WORDS, the
junction lock alone covers a bulk op's whole written frontier. Approved
long ago but SEQUENCED: it lands only with the re-home rewrite that stops
relying on the one-MCAS-state-edge packing. It regressed the soak when tried
standalone — do not land it early.

### 8.2 In-place mutation

The DLM+SW engine's step 2: rank changes as plain stores + one selector
commit under the held lock (the SW variant of the txn bitmap — under the DLM
lock the MW bitmap's per-word MCAS is redundant CAS). Attacks recompaction
copy churn (~130 structural records per insert/remove commit is real: a
recompaction re-parents every child, two records each). Only after this
lands is the FINE-vs-COARSE throughput sweep meaningful — before it, FINE
still COWs and the bench measures the wrong thing.

### 8.3 Terminology + doc sweep

"lock-free" → "optimistic" is purged from new code but pervasive in older
comments and the design notes; sweep once the machinery stops moving.

### 8.4 The missing single-writer strategy

Add the validation-only mode (caller guarantees one writer, library takes no
lock, `CDS_FT_SCOPED_WRITER` machinery already validates it). Under it,
content SW is trivially sound — it is the cheapest full consumer of this
whole conversion.

---

## 9. Cross-cutting blockers folded into the transition

### 9.1 The rekey writer's fine-lock completion

The three deliberate ft_unit failures (109/111/122) are the rekey decide's
missing fine-lock conversions (109's root cause and single-recompaction fold
are closed; 111 is measured at six gates with the arity floor now first; 122's
collision unlink is implemented but banked, the abut duplicate-slot fix
next). This thread OWNS the only armed SW content site — it stays the
conversion's live proving ground and should land ahead of Phase B's remove
sites, which share its machinery.

#### ☑ THE GLUE-FENCE TIMING ITEM — LANDED `6e77bab7`

Eight items closed (`695d23c1`, `d67851c6`, `bec0c726`, `280fdac1`,
`c6ac8c42`, `b7334aa4`, `f79438e7`, `6e77bab7`); the claim reaches ft_unit
**test 119** (was 113 at the start of the session).

The abort was `ft_chain_compress_fused`'s republish into `publish_parent`'s
body, reached through `ft_detach_node` from the rekey fold's detach. The record
named its owner correctly and the op HELD the word — `owner->state` carried
`FT_STATE_LOCK`, `glue.publish_gp_holder == owner`, `publish_gp_shared ==
false`, single-threaded. Only the WITNESS failed: the fence lived in the glue's
named field and reached `t->locks[]` at `ft_glue_txn_commit_edges`, after the
detach. The detach's own acquire of the word DEDUPED onto that glue witness and
so registered nothing (`ft-remove.h:1008` registers only `!held.shared`) —
rightly, since the earlier acquire owes the release.

Nothing was missing except WHEN the fence changes hands, so it changes hands at
the TAKE, with `@publish_gp_txn_owned` moving only the CLEAR
(`ft_glue_free_entry`'s `@holder_txn_owned`, applied to this fence). The FIELD
stays set — it is still the witness every narrow glue predicate reads.

☠ **TWO OBJECTIONS TO THAT HOIST WERE RAISED AND BOTH REFUTED** against the
code. Recorded because each is a plausible trap:

* *"Registering transfers the unlock, but `ft_glue_abort` still clears the fence
  on every bail, before `ft_flip_txn_destroy`'s strict release."* The ordering
  is real; the objection describes a register-WITHOUT-disown the idiom never
  writes. Every transfer in the tree pairs the register with disowning the clear
  (`ft-mutation-helpers.h:10440` / `:10484`, `ft-rekey.h:3226`'s
  `pp_meta = NULL`, `@holder_txn_owned` at `:7819`).
* *"A registration is sound only where the commit is guaranteed to record that
  word's terminal."* Contradicted two functions from the abort:
  `ft_chain_compress_register_retire` (`ft-remove.h:783`) registers at the
  ACQUIRE, with *"The tombstone itself is recorded later, at the commit."* And
  the companion worry — *"recording the release early is worse, a retire
  outranks"* — is backwards:
  `ft_flip_txn_record_anchor_release_held`'s header puts the release half AT the
  register, *"rather than at the commit"*, and says *"the later retire chains
  onto the release's clean pending value"*. Only the OTHER order needs the yield,
  which is what its RYW self-guard is for.

☠ **AND WIDENING `ft_flip_txn_owns` TO CONSULT THE GLUE IS REJECTED**, not
merely bigger. It already refused the wider witness in writing
(`ft-mutation-helpers.h:2192`), and it has a hole the registry does not:
`ft_glue_held_snap_one`'s `caller_holder` arm (`:9424`) returns held with
`ratified = false` for a fence whose RELEASE belongs to an OUTER frame — an SW
park on a word this commit does not own the outcome of. B0 already carries the
wide answer as a deliberately separate category: `FT_TK_COUNT_OWN`
(ft-txn-kind-stats.h:466) is three-way, registry-first, `ft_hold_trace_holds` =
`OWN_LEDGER`. Widening would erase exactly that distinction.

#### ☑ TWO MORE CLOSED — the same class, three instances now

    d4b4d2d7  the glue free-list retire's anchor: register, and BEFORE the record
    05356918  the glue's PUBLISH PARENT fence, handed over at the take

`d4b4d2d7` was the ordering class again, twice over. `ft_glue_tombstone_free_list`
recorded a fused `{LOCK|s -> TOMBSTONE|s}` whose owner is the retired node, then
registered the anchor BELOW that record — and only when the anchor was a
different word. Under per-node the anchor IS the node always, so the common case
registered nothing. The `h.lock != meta` gate was about SAFETY (a SURVIVING
anchor must leave `ft_glue_clear_fenced`, a tombstoned one cannot be stolen), not
about the registry, and `ft_chain_compress_register_retire` already registers the
same retires with no such gate.

`05356918` is `6e77bab7`'s sibling: a fold whose publish parent is PLAIN takes no
SKIP_X dual and therefore no GP fence, so the word the detach republishes into is
the publish parent itself. The rule now lives once, in
`ft_glue_take_publish_parent` — this holder has FIVE producers across rekey /
merge / graft, and a rule spread over five assignment triples is one a sixth
producer will not know about. The two rekey-fold takes adopt it; merge and graft
keep today's behaviour until their own sites are converted.

☠ **THE CLAIM'S ABORT NUMBER CAN GO DOWN ON A CORRECT FIX, AND IT DID** — 120 →
112 at `05356918`. `FT_OWNER_ASSERT_OWNED` refuses on `!t->nr_locks` ("this
commit owns nothing, so do not check"), so registering a lock SOONER puts records
that previously escaped the check under it. Read a backwards jump as coverage
gained, not as a regression, and confirm it by identifying the newly-exposed
record — never by reverting on the number alone.

#### ☑ THE WITNESS CLASS IS CLOSED — four instances

    6e77bab7  the glue's SKIP_X GRANDPARENT fence      (take -> txn)
    d4b4d2d7  the glue FREE-LIST retire's anchor       (register, and BEFORE the record)
    05356918  the glue's PUBLISH PARENT fence          (take -> txn, one shared helper)
    bf9c490c  the glue's per-node RE-PARENT MARK       (register; @marked stays false)

All four were one shape: *the op HOLDS the word, a witness that is not the txn
registry knows it, and the record-time owner check reads `t->locks[]` alone.*
Recognise it in gdb on a single-threaded run — `owner->state & FT_STATE_LOCK`
set, and `owner` findable in some other witness. Three of the four had a
carve-out saying the registry was unnecessary, and each of those carve-outs was
about who CLEARS the mark, never about whether the commit owns the word.

☠ **THE CLAIM'S ABORT NUMBER CAN GO DOWN ON A CORRECT FIX**, and it did: 120 →
112 at `05356918`. `FT_OWNER_ASSERT_OWNED` refuses on `!t->nr_locks` ("this
commit owns nothing, so do not check"), so registering a lock SOONER puts
records that previously escaped the check under it. Read a backwards jump as
coverage gained; confirm it by identifying the newly-exposed record, never by
reverting on the number.

☐ **ONE BOUND IS OWED.** `bf9c490c` registers one entry per deferred child, and
`FT_FLIP_TXN_MAX_LOCKS` (`FT_ENTRY_PER_NODE + 1` = 257) was sized on exactly
that set — *"the widest such set is a node's children plus the node itself"*.
Measured with `-DFT_LOCKS_HIGHWATER` (new; prints at each new maximum,
immediately, because `atexit` does not run on `abort`): **15 / 257** on ft_inv
`FT_INV_MW=1` at all three spacings, 13 on ft_unit. Seventeen-fold headroom on
the suite as it stands; the worst case — a glue whose deferred list is one
node's whole fan-out, plus its publish parent — is NOT proven, and ☠ the guard
is a plain `assert`, so an overflow is silent under `NDEBUG`.

#### ☑ THE SKIP_X DUAL — LANDED `ebf24686`, and the claim jumped 125 → 315

The dual lives in the compressed parent's OWN parent, and the publish resolves
that parent READ-YOUR-OWN-WRITES. So when the SAME commit re-parents the
compressed node, the dual's home MOVES — off the live node the descent fenced,
onto the fresh copy this op built. Verified rather than inferred:
`skip_owner_nf == detach_rc.new_flag` exactly, with the gate's dual slot inside
`detach_rc.old_node`.

A fresh copy is BUILD-INVISIBLE, so no owner encoding can make a record into it
legal — `bec0c726`'s rule applies: the txn publishes REACHABILITY, not INTERIORS.
The sink now dispatches per edge; a re-homed home takes the plain-store arm that
already existed for the `rec == NULL` case. ☠ DEMOTED, never DROPPED: the copy is
born holding the skip pointer to the OLD child, stale the instant the forward
publish lands. The test asks the DESCRIPTOR (`urcu_txn_find`), because
`urcu_txn_load` falls through to a fresh read and cannot tell "not re-homed" from
"no record".

☠☠ **THREE READINGS, TWO WRONG, AND BOTH WRONG ONES CAME FROM AN ADDRESS.**
First "the SPLIT arm never holds GP" (no — `merge_dst == true`, the MERGE arm,
which does reach the gate). Then "the dual lands in `g->top`", from *"absent from
`g->built[]`, therefore live"* — and that premise is false: `ft_glue_track`
inventories only what the GLUE builds, while `struct ft_detach_recompact_out`
(ft-remove.h:1407), the collapse reclaim and the orphan chain carry the rest.
**Three abort-free inventories, not one.** Printing `detach_rc.new_flag` settled
in one command what three rounds of address arithmetic did not.

#### ☑ THE FIFTH WITNESS — LANDED `a91ebafd`, and **ft_unit's claim is CLEAN**

`ft_rekey_cow_stop`'s fences live in a fn-scope array rather than a lock-SET, and
rightly: it reaches 17 anchors on the unit fixture, while a SET is the unit the
MCAS install sorts for deadlock-free acquisition. But *"not a set"* was read as
*"not in the registry either"*, and the registry answers a different question —
it is what a RECORD-time owner check can see. Every later acquire of those words
dedupes against the array through `@held.extra` and correctly registers nothing
of its own, so the registry never learned them by any route, while the re-parent
records those marks exist to license named them as owner.

`ft_rekey_marks_to_txn` registers at the TAKE and sets `@txn_owned` — the field
`ft_held_anchor` has carried all along for exactly this handover — and the
post-abort sweep skips those entries. The release RECORDS stay put; their own
comment already called that placement order-independent. What was never
order-independent is the REGISTRATION.

★★★★★ **`-DFT_REKEY_CLAIM` NOW PASSES ft_unit ENTIRELY: 315 ok, 3 deliberate, NO
ABORT.** The walk started this session at test 112.

Registry high-water (`-DFT_LOCKS_HIGHWATER`): unchanged at **15/257** (ft_inv MW)
and 13/257 (ft_unit) — the marks dedupe or peak below the existing maximum, so
the bound is no more stressed than before.

#### ☠ THE "REAL EXCLUSION GAP" ABOVE WAS AN INSTRUMENT ARTIFACT — `67f72278`

The previous revision of this row reported `ft_store_at_graft_point_commit`'s
relocation republish as an op writing a live node it never fenced: unlocked
owner, `acquire_miss == false`, absent from a populated registry, every fresh
inventory empty. All true. **`ft->lock_fine` was `false`.**

On a COARSE (or exclusive) trie the exclusion is the FT-wide mutex —
`ft_rekey_graft_simple_locked` takes `CDS_FT_SCOPED_WRITER` — and the recompact's
DLM acquire block is gated on `lock_fine`, so P is correctly never locked and
never registered. The registry is still non-empty (retire anchors, cow_stop
marks), so the assert's `!nr_locks` escape does not fire, and the first record
whose owner coarse mode never marks aborts.

`658989ef` reached for the raw `ft_flip_txn_claim_per_op`;
`ft_flip_txn_claim_per_op_armable` exists for exactly this and says so in its own
doc block, and `6f55710b` had already taught B1's dry run to use it. `67f72278`
gates the rekey claim the same way. Diagnostic-only — the hunk is inside
`#ifdef FT_REKEY_CLAIM`, which no shipped or gate configuration defines.

☠☠ **THE FIRST QUESTION AT ANY CLAIM ABORT IS `p ft->lock_fine`** (and
`ft->exclusive`, and the spacing). It costs one gdb line. Skipping it cost a full
misdiagnosis and a doc revision that had to be retracted. ☞ And read a
suite split — "ft_unit clean, only ft_inv aborts" — as a COVERAGE question
first: ft_inv has dedicated COARSE rekey arms, ft_unit's rekeys run on FINE
tries.

#### ☑ §9.1(A) IS COMPLETE — `64085ea3`. The claim is CLEAN on BOTH suites.

    -DFT_REKEY_CLAIM, --enable-rcu-debug:
      ft_unit                      315 ok / 3 deliberate / NO ABORT
      ft_inv FT_INV_MW=1 per-node  119 / 119            / NO ASSERTION

That is the same pair `bbb8e795` used to declare B1 owner-complete, and it is
§9.1(A)'s completion criterion: **the rekey writer — the one site in the tree
that already parks SW under FINE — now names an owner it holds at every record it
plants.** The walk began this session at ft_unit test 112.

The last witness turned on a distinction worth keeping: **an owner-check miss
splits two ways, and only one of them is a defect.** Where the TXN owns the
mark's clearing, a missing registry entry IS the double-clearing race — the five
fixes before this one closed exactly that. Where a caller's SWEEP owns it, the
entry is absent by design: the anchored retire's fused `{LOCK|s -> TOMBSTONE|s}`
leaves the word tombstoned, `ft_meta_lock_acquire` refuses a tombstone forever,
so no peer can re-mark it and the sweep is deterministic on both outcomes. That
site also *cannot* register — `FT_MAX_DEPTH == FT_ENTRY_PER_NODE + 1 == 257`.

**The mechanism**: the witness travels with the record, the judgment stays at the
choke point. `__ft_flip_txn_record_tag_ctx` takes a debug-only `@dbg_ctx`;
`record_tag` / `record_state_kind` / `record_tombstone_locked` become wrappers
passing NULL, so every existing caller's predicate is unchanged to the bit. Only
`ft_flip_txn_record_retire_anchored_arms` passes it, at its four record sites.

★ **The SHAPE decides, not the call path.** `ft_owner_retire_witnessed` widens
only for a record whose slot is `@owner`'s own state word and whose NEW value
sets `FT_STATE_TOMBSTONE`. A `ctx` supplied on a release, an edge or a count gets
the narrow predicate back — misuse fails CLOSED. And it is a WITNESS, never a
VERDICT: the ctx is looked up HERE, through the same `ft_held_set_snap` the
exclusion logic already trusts for dedupe. A caller may not pass "I already
checked" — a verdict token is mintable and unfalsifiable at the point that
consumes it, which is how a red control goes blind.

☑ **DETECTOR VERIFIED, NOT ASSUMED**: `-DFT_RED_OWNER_CLAIM_ON_LOCK` still aborts
ft_unit on its third test, on this assert by name, with the wide term present and
correctly declining to save it. All 118 gate legs identical to the control.

☞ It closed THREE witnesses, not one — the detach's orphan freeze plus
`ft_rekey_cow_stop`'s two stop retires.

#### ☞ WHAT §9.1(A) BEING DONE UNLOCKS, AND WHAT IT DOES NOT

☑ **B1's arm is no longer blocked by 9.1(A)** — that was the whole dependency:
arming removes the last detection of the rekey writer's unowned parks, and there
are now none to detect.

☐ **9.1(B) is untouched** — the atomic writers for the deleted staged rekey
writer's shapes, which is what greens ft_unit 110/112/123. Large, and never a B1
blocker.

☐ **Arm PLACEMENT remains the open API question**: `ft_flip_txn_arm_per_op` has
zero call sites, refuses `!t->nr_locks`, and its doc says "after the last
lock_register" — yet kind dispatch happens at RECORD time, so a doc-legal arm
point converts only records planted after it.

### 9.2 The load-sensitive concurrency class

`ft_inv` is clean alone and was recorded aborting with 4 concurrent process
copies (`inv_remove_cross_view`, plus aborts around the graft_swap solo
family) at the pre-split control commit `6815f178`, at roughly a 50% per-round
rate. The single-process gate cannot see the class. Two obligations: add a
multi-process arm to the gate, and root-cause it BEFORE Phase B certification
— it is the one known FINE-mode concurrency failure, and arming SW content on
top of an undiagnosed exclusion failure would convert its aborts into silent
erasure (an SW park cannot fail).

**Obligation 1 — ☑ DONE `8adf179e`.** `ft_parallel_gate.sh` grew an `imwx`
leg: `FT_GATE_COPIES` (default 4) concurrent copies on the default config,
reported per copy, each keeping its own completeness checks and core dir.
Proven to go red by a 2-second-timeout red control, not merely to run.

**Obligation 2 — ☠ BLOCKED: IT NO LONGER REPRODUCES, and that is not the same
as fixed.** Re-run 2026-08-24 at the very commit the failure was measured at
(`6815f178`, worktree, `--enable-rcu-debug`, `FT_INV_MW=1`):

    4 concurrent copies x 3 rounds     12/12 clean
    8 concurrent copies x 2 rounds     16/16 clean
    12 concurrent copies x 2 rounds    24/24 clean
    16 concurrent copies x 2 rounds    32/32 clean
    4 copies x 3 rounds under a concurrent `make -j96` churn loop   12/12 clean

**96 clean runs where ~50% were expected to fail.** So the recorded repro
conditions are INCOMPLETE — the variable that mattered was not captured. The
leading hypothesis is that the original "control" tree was not clean
`6815f178`: it is described as that commit *with the constructor split
stashed*, and this working tree routinely carries other uncommitted WIP, so
the control may have carried changes of its own. Machine load is the other
candidate and is the one ruled out above.

☞ **What this means for Phase B.** The obligation cannot be discharged by
root-causing a failure that will not reproduce, and it must not be waved
through either. Two things stand in for it, and BOTH are prerequisites:

1. `imwx` above, so the class is visible from now on rather than only when
   someone happens to run copies by hand.
2. §4's **record-time owner assert**, which is the direct machine check for
   precisely the hazard the gate exists to prevent — an SW park on a word the
   op does not own. It converts "silent erasure" into an abort, which is what
   made the undiagnosed failure dangerous in the first place. Build it BEFORE
   arming any site (B1–B5), and run it under `imwx`.

If the class resurfaces under `imwx`, catch a core
(`tests/regression/ft_corecatch.sh`) and root-cause it then — the evidence
will exist, which today it does not.

### 9.5 The gate is RED, in three PRE-EXISTING classes

Found by running the full matrix at B0 and, as the control, at `4887aaab`:
all 115 legs identical, so none of this is B0's and none of it is new. It is
recorded here because it was not previously written down, and because the
first class is a Phase B prerequisite that has come due EARLY.

1. **`nocompress` / per-node — `assert(g->record_only)`.** `ft_unit` aborts
   at `test_exclusive_graft_swap_inherit_non_root` (243 ok / 2 notok / 245
   tests), in `ft_glue_txn_commit_edges` under
   `ft->lock_fine && g->txn && g->txn->structural_sw`.

   **The trigger is not what was predicted.** The prediction (a code comment
   at the assert, `ft-mutation-helpers.h`, introduced with `dc2cfde0`) was
   that FINE arming would trip it at Phase B, and that the fix was to hoist
   `ft_glue_acquire_reparent_marks` above the source unlink + drain. It is
   EXCLUSIVE arming that trips it, so **A2 (`f6093f8b`) is already red in a
   config it was never run against.**

   ☠ **THE FIX IS OPEN — do not treat the hoist as decided.** The prescription
   lives in ONE code comment, not in a signed-off plan step; an earlier
   revision of this section claimed it discharged "§4's prerequisite 1", and
   no such item exists in §4 — that cross-reference was wrong. What §4's A1
   step actually records (`dc2cfde0`) is a GATING change to that acquire, not
   a hoist. The candidates, and what is now established about each, are in
   the handoff note `project_ft_glue_reparent_hoist_open_question`. In brief:

   * All SIX paths into `ft_glue_txn_commit_edges` honour an ABORT — verified
     per caller — so `record_only` is a stale PROXY for the property the
     assert wants ("this caller absorbs an ABORT here"). ☞ But "honour"
     differs: `ft-rekey.h:3001` and `ft-graft.h:3625` have a genuinely clean
     bail, while `ft-merge.h:2849` / `ft-rekey.h:4836` / the non-fused graft
     arms are PAST a point of no return and honour it only by a retry that
     must eventually succeed.
   * A full hoist is impossible for ATTACH-TIME-discovered edges: the acquire
     needs `g->deferred[]` complete, and the displaced child's identity is
     attach-time knowledge a retry's re-descend replaces. It is achievable
     only for build-time edges. `ft_glue_acquire_splice_holders` — the model
     the hoist appeals to — is itself SKIPPED on the `unfailable` path for
     the same reason (`ft-merge.h:1978-1993`).
   * ★ **A fourth candidate, and it addresses the cause:** on an EXCLUSIVE
     trie the mark defends against nobody. The acquire's own header already
     makes exactly this argument for COARSE — the mark arbitrates against one
     peer (`ft_meta_nr_child_inc` from an insert below the child), and where
     that peer is excluded "the mark has nobody to arbitrate against" — and
     exclusivity excludes it more strongly than the COARSE mutex does. So the
     gate may simply be wrong: `lock_fine && armed` should perhaps read
     `lock_fine && armed && !exclusive`, which keeps the assert un-relaxed.
     ☠ NOT free: skipping the acquire also skips its `ft_glue_op_holds`
     detection, so `held_lock` stays false and the live re-parent records the
     §4.B MW guard — a deterministic self-abort if any exclusive shape
     self-holds a re-parented child's word. Unproven either way by reading.

   ☞ Reachability is SETTLED BY READING, and it does not narrow the problem:
   `structural_sw` has exactly two sources (the constructor arm
   `!lock_fine || exclusive`, and the rekey one-decide writer);
   `ft_flip_txn_arm_per_op` has zero call sites. Combined with `lock_fine`
   the branch is reachable iff the glue txn's trie is FINE **and** exclusive
   — and no graft/merge/rekey path gates an exclusive DST out (only src /
   swap exclusivity is checked, `ft-graft.h:1283`, `:2795`). So graft-attach
   can reach it. ☠ A runtime call counter would answer the WEAKER question of
   which callers the SUITE drives there — see
   `feedback_reachability_is_a_code_fact`.

   ☞ The lesson generalizes past this bug: A2's gate list (§4 step 3) was
   ft_unit + ft_inv + ASAN + fault-inject + reserve + an 8-copy control, and
   every one of them passed. None of them is the config that fails. An arming
   step must be gated on the FEATURE-FLAG MATRIX, not on the default build.

2. **`txndbg` and `anchorval` at `exponential` and `root-only` spacing.**
   `ft_unit` aborts (291 and 236 tests in), and at root-only `ft_inv` aborts
   after 73 tests on all three arms. `proxyassert` runs the same three
   spacings and passes all of them, so this is not the coarse-spacing axis
   alone — it is those two detectors ON that axis. Unattributed; no root
   cause yet.

Per-node — the default, and the spacing every other config runs at — is clean
everywhere except class 1.

### 9.3 The deleted staged rekey writer

8 tests fail on purpose until the staged writer is reimplemented atomically
(a tmp-trie hides live keys for a GP per move; an FT-wide lock does not
redeem it). Independent of the arming steps, but it holds test-suite green
hostage; schedule it with the 9.1 thread.

### 9.4 Monitors

`skip_conflict`'s slot arm rewinds 50,316/run (the "never fires" note is
stale) — watch it across Phase B, it shares words with the converted sites.

---

## 10. Method rules that bind this work

* Prove the mechanism ran before believing any zero (counter must MOVE; the
  positive control is the pattern). FT code lives in `liburcu-cds.so`, not
  the test binary.
* The kind-conflict detector must be verified ARMED per build
  (`DEBUG_RCU`/`CONFIG_RCU_DEBUG` probe TU) before a green certifies a step.
* One adversarial skeptic per exclusion claim, async; default refuted=true.
  The best-effort claim gate from CLAUDE.md applies to every abort path this
  plan touches.
* Byte-neutral claims are verified at the OBJECT (normalized `objdump -d` +
  `nm -S`), a lesson the constructor split already paid for.
* Benchmarks: `-O2 -DNDEBUG`, sequential, before/after on the same build
  pair; abort-relief claims quote the 29% bound.
* A status table is a dated claim — `git log -S` the identifier before
  acting on any ☐ row above.

## 11. Landing order (the one list)

    G2  always-MW root helper, retire sw_exempt_slot        ☑ LANDED a961c6e6 (site map in §2 was INCOMPLETE)
    G1  reader-sufficiency answer                           ☑ ANSWERED YES 2026-08-23 — cleared
    A1  arm COARSE non-exclusive  + protocol §3             ☑ LANDED (four commits, §4 step 2)
    A2  arm exclusive             + protocol §3             ☑ LANDED f6093f8b
    9.2 multi-process gate arm                              ☑ LANDED 8adf179e
    9.2 root-cause cross_view                               ☠ BLOCKED — 96 clean runs at
                                                              its own control commit; §9.2
                                                              names the two stand-ins
    9.1 rekey fine-lock completion                         (IN FLIGHT) — the reds are
                                                              110/112/123 now (two
                                                              regression tests shifted the
                                                              indices).  ☞ TWO HALVES: the
                                                              fine-lock conversions (what
                                                              blocks B1) and the atomic
                                                              writers for the deleted staged
                                                              writer's shapes (what greens
                                                              the three).  -DFT_REKEY_CLAIM
                                                              (658989ef) enumerates the
                                                              first, abort by abort;
                                                              695d23c1 / d67851c6 / bec0c726
                                                              / 280fdac1 / c6ac8c42 /
                                                              b7334aa4 / f79438e7 / 6e77bab7
                                                              / d4b4d2d7 / 05356918 / bf9c490c
                                                              landed — the WITNESS class is
                                                              CLOSED (4 instances); next is a
                                                              / ebf24686 / a91ebafd / 67f72278
                                                              / 64085ea3 — ☑ §9.1(A) COMPLETE:
                                                              the claim is CLEAN on ft_unit
                                                              AND on ft_inv FT_INV_MW=1.
                                                              B1's arm is no longer blocked by
                                                              it; 9.1(B) and the arm-PLACEMENT
                                                              question remain (§9.1)
    B0  per-op arm helper + record-time owner assert        ☑ LANDED — and its first
                                                              measurement says NO site is
                                                              owner-complete (11.9% of the
                                                              surface); §4 has the table
    B0b the owner predicate: registry ∪ hold-ledger        ☑ LANDED @92e27199 — worth
                                                              1.7 pts; its first zero was an
                                                              INSTRUMENT BUG (§4)
    Bx  the external-head class                            ☠ SPLIT IN THREE (§4).  head
                                                              promote ☑ LANDED e9268e13
                                                              (0%→33.3%); publish-lane owner[]
                                                              ☑ LANDED 6f54e698; external
                                                              back-edge ☐ the real DESIGN call,
                                                              and the smallest of the three.
                                                              ☞ The "§8.2" reference is NOT
                                                              phantom — it is the ESCALATION
                                                              MODEL's §8.2 (b44d08cc); the two
                                                              design docs reuse section
                                                              numbers, so always name the doc
    B1  insert one-commit                                   ☑ ARMED e2ba43f8 -- 2.64M txns
                                                              armed, 6,998,341 records SW.
                                                              ☠ NEEDED A PER-NODE SPACING
                                                              GATE: ungated, per-node stayed
                                                              green and txndbg/anchorval went
                                                              RED at exponential + root-only
    B2,B4,B5 the other hot sites, one at a time            (each: owner-complete -> claim
                                                              dry-run -> arm; NOT mechanical).
                                                              B2 ☑ ARMED: owner-complete
                                                              a9ff9549, in-place path 6024f167
                                                              (reaches 10 times -- an EXCLUSIVE
                                                              trie shape), RECOMPACTION
                                                              REPUBLISH 21b6b559 -- armSW now
                                                              tracks OK (4.00M of 4.00M
                                                              commits).  ☠ "0.7%" was armSW
                                                              mis-read as yield; 19befc1f is
                                                              the REACH counter that separates
                                                              them.  Remainder = the ~2.4
                                                              records/commit planted BEFORE
                                                              the acquires finish (Phase C/E)
    B3  ft_chain_compress_fused (the collapse)              ☑ ARMED ec00e234 -- dry run
                                                              cec68299 CLEAN at FIRST reading,
                                                              no fixing pass; 99.8% of reaches
                                                              armed.  ☠ its abort column needed
                                                              n=3 ALTERNATED: one run read as a
                                                              wrong-direction alarm and did not
                                                              survive (ctl 62.6 vs arm 55.4
                                                              aborts/1k, control spread 51-73)
    B5  ft_unchain_node's head clear                        ☑ ARMED 8cd52c64 -- no SKIP_X dual
                                                              by ROUTING, and a DETECTOR says
                                                              so (red control: silent on
                                                              ft_unit, 6 aborts on ft_inv)
    B4  ft_promote_head (both arms)                         ☑ ARMED 1c59dc9c -- unblocked by
                                                              the per-edge @owner_held fix af22756b
                                                              (option (a)).  ☠☠ a NULL @owner
                                                              NEVER failed closed: record_tag
                                                              dispatches on structural_sw ALONE
    B6  retire hand-arming (rekey writer, root COW)         (small)
    C   re-measure; G4 cell-lane decision                   (gate + data)
    G5  subtree freeze-state gate design (hybrid D)         (design, w/ D)
    D   acquire fair-handoff design w/ Mathieu              (design, parallel)
    E   spacing certification + gate lift + strategy fold   (medium)
    F   §8.3 layout split + in-place + single-writer mode   (large, last)

Each step is bisectable, each gated by §3's protocol; nothing below the C
line starts before the C measurement exists, because C is what prices D–F.
