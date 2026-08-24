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

`OWN_HELD` / `OWN_MISS` (ft-txn-kind-stats.h) run the same predicate on every
record in EVERY mode, so a FINE **unarmed** run says whether an arm would be
legal BEFORE the arm is written. They split `MW_STRUCT` exactly — the surface,
by whether the op holds the word's owner — and `OWN_HELD + OWN_MISS ==
MW_STRUCT` is an invariant of the table worth checking.

ft_inv, `FT_INV_MW=1`, 507 threads, per-node spacing, `build-tk`:

| creation site | MW_STRUCT | OWN_HELD | owner-held |
|---|---|---|---|
| `ft-insert.h:776` — insert one-commit | 39,300,256 | 5,607,525 | **14.3%** |
| `ft-remove.h:2877` — remove commit_rec | 39,201,793 | 4,033,801 | **10.3%** |
| `ft-remove.h:3720` — publish-sedge | 14,924,241 | 0 | **0.0%** |
| `ft-remove.h:882` — detach-side creator | 7,783,981 | 3,263,065 | **41.9%** |
| `ft-remove.h:3946` — publish-sedge | 4,482,056 | 0 | **0.0%** |
| `ft-graft.h:3463` | 2,115,340 | 0 | **0.0%** |
| `ft-graft.h:1705` | 994,282 | 209,406 | 21.1% |
| **TOTAL** | **109,929,466** | **13,114,407** | **11.9%** |

**NOT ONE creation site is owner-complete, so not one of the five sites below
can arm as things stand** — the assert would fire at every one of them. That
is the finding, and it reorders the phase: the step per site is no longer
"arm it", it is *make it owner-complete, prove it with the claim dry-run,
then arm it*. The arm is the cheap half.

### ☠ READ THE 11.9% AS A LOWER BOUND — the first task is the PREDICATE, not the locking

The claim dry-run's very first abort proves it. Under
`FT_RED_OWNER_CLAIM_ON_LOCK`, ft_unit aborts in
`ft_flip_txn_record_retire_anchored` (ft-mutation-helpers.h:4679, reached from
`ft_detach_node`), writing a node's own state word — and that line sits inside
`if (h->lock == node || h->node_held)` **and** inside `if (h->shared ||
h->node_held)`. Both conditions require the op to hold `node`'s own word. So
the op demonstrably OWNS what it writes, and `ft_flip_txn_owns` says otherwise
— because the hold came from an earlier member's acquire recorded in the
`ft_held_anchor` / `ft_lock_ctx` ledger, not in `t->locks[]`.

That is a FALSE NEGATIVE on the hottest retire path, structurally proven
(no debugging needed — read the branch the frame is in). It does not weaken
the assert, which only ever refuses a park; it means the gap between 11.9%
and 100% is *some mixture* of real exclusion gaps and predicate blindness,
and nobody should act on the split until the two are separated.

⇒ **The next step is to widen the predicate before widening the locking.**
`ft_held_set_snap` already walks registry + extras + glue + outer; what a
record helper lacks is the `ft_lock_ctx` to walk it from. Stash it on the txn
under `--enable-rcu-debug` at the op entry points that own one, re-run the
table, and only then read a site's remainder as a real exclusion gap.

The 0.0% rows are ONE class and it is already inventoried in the source:
`FT_OWNER_NONE_EXTERNAL_HEAD` (11 sites) — an external head's back-channel
word (`cell->parent`, `en->prev`, `next_node->prev`) has no owning lock at
all, because neither a cell nor an external node carries a state word. §8.2
puts the entry list under the HOLDER's lock, so closing this class means the
holder's lock actually covering the chain, and it is worth **21.9M records**
of the surface — a fifth of it, concentrated in three sites. It is the
largest single item in Phase B and it is a design question, not plumbing.
`FT_OWNER_UNPLUMBED` (1 site, `ft-compact.h`) is the other marker: the owner
exists and is simply not in scope. Both are greppable.

### Site order

Unchanged as an ordering, but each step is now
*owner-complete → claim → arm*, one site per step, one adversarial skeptic per
claimed exclusion argument:

1. `ft-insert.h:776` — the insert one-commit (39.3M; 14.3% held).
2. `ft-remove.h:2877` — remove commit_rec (39.2M; 10.3% held).
3. `ft-remove.h:3720` and `:3946` — the publish-sedge pair (19.4M; 0% held,
   and 0% because it IS the external-head class — so this step is that
   class's design question, not a per-site conversion).
4. `ft-remove.h:882` — the detach-side creator (7.8M; 41.9% held, the closest
   to ready, and also the largest content-lane abort source).
5. The remaining content sites in descending count.

☠ A raw `ft_flip_txn_record_tag` loop over ordered-cell edges survives at
`ft-merge.h` and `ft-rekey.h` (the sibling graft path routes through the
tag-dispatching recorder precisely to avoid it). Sound today only because the
modes that arm exclude trie-wide; the per-edge `owner` stops it at the Phase B
arm, and routing the loop through the dispatching recorder is the real fix,
owed with those sites' arm.

Then retire the hand-arming at the rekey writer and root-COW driver onto the
same helper, so the switch has no bypass.

## 5. Phase C — the residual MW_ALWAYS lanes (the G4 decision)

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
    9.1 rekey 109/111/122 fine-lock completion              (in flight)
    B0  per-op arm helper + record-time owner assert        ☑ LANDED — and its first
                                                              measurement says NO site is
                                                              owner-complete (11.9% of the
                                                              surface); §4 has the table
    Bx  the external-head class (FT_OWNER_NONE_EXTERNAL_HEAD) ☠ NEW, and it is the biggest
                                                              single item in B: 21.9M records,
                                                              3 sites at 0% owner-held, and a
                                                              DESIGN question (§8.2 puts the
                                                              entry list under the holder's
                                                              lock; nothing enforces that today)
    B1-5 five hot sites, one at a time                      (each: owner-complete -> claim
                                                              dry-run -> arm; NOT mechanical)
    B6  retire hand-arming (rekey writer, root COW)         (small)
    C   re-measure; G4 cell-lane decision                   (gate + data)
    G5  subtree freeze-state gate design (hybrid D)         (design, w/ D)
    D   acquire fair-handoff design w/ Mathieu              (design, parallel)
    E   spacing certification + gate lift + strategy fold   (medium)
    F   §8.3 layout split + in-place + single-writer mode   (large, last)

Each step is bisectable, each gated by §3's protocol; nothing below the C
line starts before the C measurement exists, because C is what prices D–F.
