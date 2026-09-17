# Fractal Trie — lock-set inventory at FINE (per-node) and EXPONENTIAL (2026-09-16)

Mathieu's shape for the work: *"we need to complete the missing lock-set
transition for fine and exponential. only when it is COMPLETE can we turn MW
into SW."* This is the inventory that transition starts from: **which mutation
primitives do not hold their lock set, at per-node and at exponential
spacing**, measured and attributed to the primitive rather than argued.

It is an inventory, not a plan. No primitive is converted here, and nothing
below is a proposal to convert one class on its own.

---

## 0. The question, and how it was asked

**"Holds its lock set"** means: at the moment a word is written, the op holds
the lock that `ft-dlm-lock-coarseness.md` §1/§2 assign to that word — the
**anchor of the word's owner** (the owner itself at per-node). Two ways to fail
it, and they need different instruments:

* **Coverage** — the op writes a word and holds no lock covering it.
* **Agreement** — the op holds *a* lock, but not the one every other op
  computes for that node, so it excludes nobody (§1).

`-DFT_DEBUG_STRUCT_ANCHOR` (needs `-DFT_DEBUG_TXN_KIND`; build with
`-fno-omit-frame-pointer`) answers both against an **independent** anchor: it
climbs the owner's parent words to the root, sums the spans to get the byte
depth, and applies §2 over that path. This is §5.3's rejected up-walk, used as
an oracle only. A parked proxy, a NULL parent word (transient re-home) or an
external on the path reads UNDATED rather than guessed.

| probe | where | question |
|---|---|---|
| A | `ft_dlm_acquire_set_at`, after `ft_anchor_meta` | does the anchor the op takes equal the trie's anchor for that member? |
| B | `__ft_flip_txn_record_tag_ctx` / `ft_flip_txn_record_tag_mw`, re-asked in `ft_flip_txn_commit` | at record time, is the owner's anchor held? If not, is it held by commit (LATE) or never (NEVER)? Keyed by (txn creation site, call site, one frame up), symbolized offline with `addr2line -i` |
| C | the chain audit arms (`-DFT_DEBUG_CHAIN_HOLD`) | the same exact-anchor verdict for the words that never reach a record helper: hlist stores, raw parent/head words, in-place body writes. Writes the caller declares `FT_EXCL_HIDDEN` are skipped **except chain words**, which carry no declaration and are always scored |

The witness is the union the owner assert uses: the txn registry, the ctx
frames, the per-thread hold ledger (`-DFEATURE_FT_HOLD_TRACE`), and this trie's
FT-wide writer lock. COARSE and EXCLUSIVE tries are bucketed out (no peer to
exclude), as are commits already carrying `@acquire_miss` (discarded before the
engine).

**Red control, and it fires.** `FT_SA_RED=root` asks every record about the
ROOT instead of its real anchor, without touching the ops. ft_unit at
exponential goes from 0 to **2,700,761 SELF_ONLY + 41,617 UNHELD**. The 41,617
matches the normal run's 41,619 records held through a coarser anchor to within
2. So the witness can say *no* while the op holds other locks, and the zeros
below are readings, not a blind instrument.

**Rig.** `-O2 -g -fno-omit-frame-pointer -DFEATURE_FT_INSERT_IN_PLACE
-DFEATURE_FT_LOCK_SPACING_ENV -DFT_DEBUG_STRUCT_ANCHOR -DFT_DEBUG_TXN_KIND
-DFEATURE_FT_HOLD_TRACE -DFT_EXCL_REPORT_ONLY -DFT_DEBUG_CHAIN_HOLD`,
`CDS_FT_LOCK_SPACING=per-node|exponential`, ft_unit 357/357 and ft_inv 152/152
green in every leg. Numbers are per single leg (one ft_inv run covers its
list-on, list-off and MW rows).

**Limits.** Only paths the two suites reach (§4). The always-MW lanes are
lockless by design, so B counts them and does not score them (§3). HIDDEN declarations are
believed on the raw lanes. Record time asks the SW question on purpose: an SW
park is legal only if the lock was held when the overwritten value was read.

---

## 1. Coverage: primitives that write while holding NOTHING — both spacings

Probe B puts these in NOLOCKS (registry and ledger both empty), and the commit
re-check puts them in NEVER: the lock is still not held when the descriptor goes
to the engine. None of them depends on spacing, because nothing is held.

| # | primitive | ft_inv per-node | ft_inv exponential | ft_unit | what is missing |
|---|---|---|---|---|---|
| 1 | **delete tier, pure leaf delete** — `ft_detach_node` → `ft_remove_one_commit` (holder's child slot + the fused `nr_child--`), and its chain word at `ft_detach_node:4975` (freeze_leaf) | 2,745,486 records, **2,585,871 committed never held** (159,615 aborted); chain lane 2,384,879 | 2,609,398, **2,443,766 committed** (165,632 aborted); chain lane 2,247,316 | 655,092 / chain 646,700 | the holder (its anchor). `ft_flip_txn_lock_or_guard_parent` runs only on the promote arm (`commit_txn && !pub->state_meta`); the leaf arm leans on the fused `nr_child--` CAS, the "arbitrated on the holder's state word" of `ft-reintroduce-in-place-mutations.md` §6.5 |
| 2 | **`cds_ft_remove_all` of the EMPTY key** — root's `external_nodes` (`_cds_ft_remove_all_locked`, both `key_len == 0` commits) | 57,918, all committed | 55,126, all committed | 4 | the root node's lock (the root is its own anchor at every spacing) |
| 3 | **insert over an OCCUPIED slot** — `_cds_ft_insert`'s "NULL or external node before end of key" arm → `ft_attach_node`'s reserved-slot record, when the slot already holds a displaced external head | 13,359 (13,326 committed) | 13,942 (13,917 committed) | 102 (exponential: 55 LATE, 47 NEVER) | the attach node. The hoist is gated on `!old_node_flag`; the other `ft_attach_node` call site (`_cds_ft_insert:3476`) is fully held |
| 4 | **compaction** — `ft_compact_relocate_compressed`'s raw parent and head words | — | — | 1,030 | any lock: its exclusion is the "caller's responsibility" TODO, which is not a contract |
| 5 | bulk glue writes outside the FT-wide lock — `ft_glue_apply_deferred`, `ft_glue_defer_edge_origin`, `ft_store_at_graft_point_commit` | ≤ 45 | ≤ 40 | ≤ 11 | bulk ops are excluded by design by the FT-wide lock; these few rows are writes the lock did not cover at that instant — not yet attributed |

**Everything else that holds a lock holds the right one for the words it
records.** Across the structural record lane, ft_inv at exponential: HELD
29,243,068, held through a coarser anchor 1,207,645, **SELF_ONLY 0, UNHELD 0**
(UNDATED 1,925, 1,923 of them in row 1's commit, racing re-homes). Per-node likewise has
no UNHELD. On the raw lanes the chain words of `ft_chain_compress_fused:2326`
and `ft_detach_node:3902`, and the in-place body writes
(`ft_popcount_node_set_nth` / `ft_pigeon_node_set_nth`), are covered at
exponential, a large share of them through an ancestor anchor.
(`ft_popcount_node_set_nth:516` reads UNHELD / UNDATED on an unpublished node
and is the refused-arm false positive `ft-reintroduce-in-place-mutations.md`
§6.3 already names.)

---

## 2. Agreement: acquires that take the WRONG word — exponential only

Probe A, ft_inv at exponential (per-node is exact by construction). The
mismatch is independent of load for rows 6–9: each one also shows up in
single-threaded ft_unit, or has a mechanism read directly off the anchor table
in the sample.

| # | acquire site | mismatches | mechanism (from the samples) |
|---|---|---|---|
| 6 | `ft_detach_node`'s compressed-parent free walk (`ft_detach_orphan_acquire` in the `free_detached_subtree \|\| nr_elevated` loop) | **88,059 of 199,918**; ft_unit 2 | `walk_depth` is computed once and **never advanced**, so every orphan past the first is dated at the first one's depth. Sample: depth(op) 2, depth(trie) 3, L=2 → `ft_anchor_meta`'s on-a-level early return anchors the node on ITSELF, while its anchor is the parent at byte 2 |
| 7 | `ft_detach_orphan_planlock` (both orphan walks, via `ft_walk_extend`) | 3 per run, plus ~2,190 stale (below) | `ft_walk_extend` leaves `d->nf` on the node it ENTERED and moves `d->depth` to that node's end, so the cursor is the member's PARENT. Sample: `pending=1 bound=(nil) bound_start=7 == clamp` → `bound ? bound : d->nf` returns the compressed parent instead of the member (SELF). This is the design doc's still-open *"`a->bound ? a->bound : d->nf` substitutes the CURSOR"* |
| 8 | `ft_node_recompact:1761` | 11 (1 non-stale); ft_unit 6 | the cursor is the member's **SKIP-encoded** flag (`0xa007…f00b` over a member `0x7ff…f00b`). The pending-level fallback returns it, `anchor == nf` fails on the encoding, and `ft_flag_to_metadata` resolves the skip word to the elided compressed node (ABOVE) |
| 9 | `ft_merge_lock_overlap:523` | 22 | the cursor is at depth 0 and level 2 was never crossed (`crossed=0`); a member three bytes down goes through `ft_descent_anchor_child`, whose rule is exact for ONE hop only, and anchors on itself. Bulk: the FT-wide lock excludes today |

**Live evidence that 7 breaks exclusion.** With the E.2 exclusion oracle armed
(`FEATURE_FT_HOLD_TRACE` without `FT_EXCL_REPORT_ONLY`), ft_inv at exponential
aborts in `inv_concurrent_insert_unique_nolist`: **2 of 80** filtered runs, plus
1 full leg. The shape is identical each time:

    FT EXCLUSION VIOLATION: node X claimed at ft_flip_txn_lock_or_guard_parent_ex
      by tid B (anchor X=SELF) while owned by tid A (from ft_detach_orphan_planlock:984,
      anchor <ancestor>) ... ANCHORS DIFFER

Two threads cover one node through two words. Probe B cannot see this: each op
holds *a* covering lock for its own records. Only A and the oracle see an
agreement failure.

**Stale plans, not defects.** The other ~2,330 mismatches (2,192 at the
plan-lock, 85 at `ft_chain_compress_fused:1614`, 25 at
`ft_flip_txn_lock_or_guard_parent_ex`, 13 at `_cds_ft_insert:3812`, 10 at
`ft_node_recompact`, 2 at `ft_unchain_node`) are OFF PATH, and **every one**
has its anchor or its member TOMBSTONED at the check. That is the §2.1
"planned against the pre-change node hits TOMBSTONE and replans" path working.

---

## 3. The always-MW lanes — lockless by design, counted not scored

`ft_flip_txn_record_tag_mw` names no owner because these writes take no lock.
They are LOCKLESS BY DESIGN (Mathieu): the MCAS arbitrates them on their
expected values, and the lock-set question does not apply. They are counted
here only to size the surface. ft_inv, per-node:

| class | records | producers (call site) |
|---|---|---|
| HEAD_BACK | 7,404,358 | `ft_node_recompact` re-homing heads into its copy (insert reserve relocation ~3.3M, detach `ft_node_replace_ptr` ~1.66M), `ft_park_live_parent_edge` 1.25M, `ft_chain_compress_fused`'s child back edge 1.02M, `ft_detach_node_replace_compressed_parent` ~85k, `cds_ft_remove_all` 36k, rekey COW, compaction, merge |
| PARENT_WORD / STATE | 990,002 each | `ft_reparent_record_meta` from `ft_node_recompact`, `ft_chain_compress_fused`, `ft_park_live_parent_edge`, graft, merge |
| PSO | 417,404 | the same re-parent sweeps |
| DUAL_UNNAMED | 410,491 | `_cds_ft_insert_replace`'s ordered-cell lanes (the register already says these are cell edges filed under the dual's name) |
| CELL / ROOT / RANK | 12.6M / 2.0M / 1.5k | [DESIGN] per the register |

---

## 4. Not reached by either suite

`_cds_ft_remove_locked`'s and `_cds_ft_remove_all_locked`'s ordered-cell
unsplice RETRY txns, `ft_compact_relocate_compressed`'s own txn, and every
rekey spine / subpos site. `cds_ft_replace` commits 3 times in ft_unit and not
enough in ft_inv to score. A primitive missing from §1 is only as clean as its
coverage.

---

## 5. SETTLED BY TEST, then CLOSED: holding the old anchor did not exclude a late acquirer

§3 of the anchor doc argues that *"every change to which-node-covers-byte-L is
itself a locked mutation of those very nodes"*. Holding the OLD anchor during
the change excludes ops that hold it *concurrently*. The question was whether it
also excludes an op that PLANNED before the change and ACQUIRES the old anchor
after the restructurer released it. It does not.

**The rig** (`doc/design/ft-anchor-move-rig.c`, against a library built with
`-DFT_DEBUG_INTERLEAVE`, whose test-only hook parks the delete tier's leaf hoist
before and after its acquire). Exponential spacing:

    root [0,1) -'a'-> cn [1,9) "bcdefghi" -> A @9 -'J'-> X @10, children '1'..'8'
    anchor(X): depth 10, L = 8, first boundary >= 8 is A          ==> A
    split: insert "abcdefghQzz" (diverges at byte 8), boundary B @8 ==> anchor(X) = B

The VICTIM removes `abcdefghiJ8` (a leaf directly in X's slot, so it holds
nothing before its hoist) and is parked at PRE, with its plan -- anchor A --
already taken. The split commits. The victim then acquires A and is parked at
POST, holding it. A PEER runs with a 2 s budget.

| mode | peer | peer completed while the victim held its anchor | reading |
|---|---|---|---|
| no split | delete `J7` | **no** | CONTROL: both anchor X on A, the peer waits |
| split | delete `J7` | **YES** | **HOLE**: the victim holds A, the peer holds B, both write X |
| split | insert `J0` | no | a false negative, see below |

Probe A records the victim's acquire as a LIVE mismatch (neither node
tombstoned): `op anchor A (start 9), trie anchor B (start 8)`. Both commits
landed and both keys are gone -- today's MW records arbitrated the two writes to
X's slot and state word. Under SW parks they would not.

☠ **The insert peer is not a counter-example.** Growing X recompacts it with lock
set {X, A, ...}: A is X's PARENT and therefore a coarsened MEMBER, whose own word
the acquire samples -- and A's own word carries the victim's lock. It waits on A
as a member, not on the anchor it computed. A peer whose lock set is {X} alone
(the leaf delete) shows the gap.

**What the split does do**, observed on the way: its lock set includes the child
it re-homes (A), so a victim already HOLDING A blocks the split. The gap is only
the late acquire: the acquire validates the MEMBER's back edge, and X's edge is
unchanged by a split above A; nothing re-checks that A is still the first
boundary at or after L once it is held.

### 5.1 The cure: a missing tombstone validation (Mathieu)

At the victim's stale acquire, the node the plan derived A from -- cn, which
covers level 8 -- was already `TOMBSTONE` (probe A: `cover state 0x6`), retired
by the split's fenced tombstone. The anchor answer depends on that node:

- A boundary can appear in `[L, anchor)` only by splitting the node that covers
  L strictly inside its span.
- A boundary can disappear at the anchor only by fusing the anchor into that
  node.
- A compressed node's span never changes in place, so both changes RETIRE it.

`ft_anchor_coverer` names that node for a member (NULL when the answer
depended on none). `ft_dlm_acquire_set_at` loads its state after the acquire
commit. A tombstone releases what was taken and re-plans with `-EAGAIN`.
TOMBSTONE never clears, so a load after the commit answers for the commit --
**read logically**.

☠ **The first version read it raw, and the skeptic REFUTED it by running.** A
split's commit settles its guard on the old anchor A (a plain late record) BEFORE
the fenced tombstone on cn (a registered lock word settles last, `8c517957`).
Between the two, A is takeable while cn still holds the split's SUCCEEDED proxy.
That raw word carries `FT_STATE_PROXY` (bit 0), not `FT_STATE_TOMBSTONE` (bit 1),
so the check passed and the hole was back. The check now resolves the word
(`urcu_txn_resolve`: SUCCEEDED answers new, anything else old). An UNDECIDED
restructure cannot succeed past the guard on A that the lock just taken fails.
`-DFT_DEBUG_INTERLEAVE` gained `FT_IL_SETTLE_LOCKS_LAST`, fired from
`ft_flip_txn_late_last` at the start of the last settle pass, so the rig's
`splitlate` mode parks the split in exactly that window.

A load after the commit is enough only if no split can land while the lock is
held. That holds because moving the boundary re-homes (or retires) the old
anchor, and the re-home records a guard on that node's CLEAN state word
(`ft_reparent_record_meta`), which the victim's lock fails. `splitheld` measures
that, with a red control:

| mode (peer = delete `J7`) | coverer check | split while held | peer while held | reading |
|---|---|---|---|---|
| nosplit | 18 checked, 0 retired | -- | waited | CONTROL OK |
| split | 17 checked, **1 retired** | -- | **waited** | the stale acquire re-planned onto B: **no hole** |
| splitheld | 17 checked, 0 retired | waited | waited | the held order was already closed |
| splitheld, RED: child state guard removed | 17 checked, 0 retired | **completed** | **completed** | the guard is what closes it |
| splitlate, RED: coverer read raw | 17 checked, 0 retired | -- | **completed** (3/3) | a committed tombstone read as a proxy: **HOLE** |
| splitlate | 17 checked, **1 retired** | -- | waited (3/3) | resolved: **no hole** |

A skip-encoded coverer names its node only through the child's back pointer --
which the very split being guarded against re-homes -- so the check re-plans on
one rather than resolve it. The descents enter resolved flags, so it is not
expected to occur. `-DFT_DEBUG_STRUCT_ANCHOR` counts it (`FT_SA_COVER ... skip=`).

---

## 6. Progress on §1 (ordered by Mathieu: 1, 2, 3, then 4)

| row | commit | after |
|---|---|---|
| 1 delete-tier leaf delete | `7e4e9d15` | NOLOCKS 2.7M -> 0 (per-node), 2.6M -> 0 (exponential) |
| 2 `remove_all` of the empty key | `c4089e33` | both commit rows absent |
| 3 insert over an occupied slot | `79f0b245` | row absent |
| 4 compaction (compressed + cell relocation) | `06662abe` | raw writes 1,030 -> 1 (the exclusive-trie test) |

**Found on the way, and a precondition of the flip:** the engine's late settle
pass selected the words it hands ownership over by PROXY TAG, and the trie's
duplicate-chain links share tag 1 with its state words. A registered lock's
release was settled before the chain links it protects, so a peer could take the
lock while a link was still a proxy. It surfaced as a pre-existing SIGSEGV in
`inv_concurrent_insert_replace_nolist` (28/1500 filtered runs before row 1,
40/1200 after -- row 1 put a release next to a chain mark in one more commit),
and it is 0/1200 once the registered lock words settle last (`8c517957`).

---

## 7. Audit: does every fine / exponential lock holder check TOMBSTONE with the lock held? (Mathieu)

### 7.1 Where the check lives

Every DLM take in the tree goes through `ft_dlm_acquire_set_at`:
- `ft_dlm_lock` has exactly one caller, the acquire loop.
- `ft_acquire_member` wraps a one-member set.
- `ft_flip_txn_lock_or_guard_parent_ex` locks through `ft_acquire_member`.

So the check is structural, and it is in the SAME commit that takes the lock:

| what the site's write depends on | where it is checked |
|---|---|
| the lock word (per-node: the member itself) | `ft_dlm_lock` refuses `PROXY \| TOMBSTONE \| LOCK` and records `{s -> LOCK\|s}` against that exact `s` |
| a coarsened member's own word | `ft_held_anchor_sample_node` refuses `PROXY \| TOMBSTONE \| LOCK`; `ft_held_anchor_guard_node` validates the snapshot in the acquire commit |
| the coverer the anchor was derived from (exponential) | after the commit, resolved (§5.1) |
| a member deduped onto a word the op already holds | no peer can retire a held word; the op's OWN consumed fence is the designed `TOMBSTONE` dedupe arm |

What the choke point cannot see is a node a site WRITES without naming it as a
member. That is §1's coverage question, and rows 1-4 closed the measured cases
(§6).

### 7.2 The measurement

`-DFT_DEBUG_STRUCT_ANCHOR` gains two probes.
- **DEAD MEMBER.** After `ft_dlm_acquire_set_at` returns 0, a member whose node
  reads logically TOMBSTONE is counted by arm (taken / shared / shared-coarsened /
  coarsened-taken) and sampled with the acquire's `fn:line`.
- **DEAD-OWNER WRITE.** Every owner-bearing record whose verdict was reached
  through the owner has that owner's state resolved at commit ENTRY, while the
  op's locks are held. An owner already TOMBSTONE before a commit that lands is
  a lock holder writing into a retired node. It is counted per (txn site, pc0,
  pc1) as `T_DEAD_HELD_OK` / `T_DEAD_UNCOV_OK` / `t_dead_ab`, and sampled.

RED CONTROL, `FT_SA_RED_DEAD=1`: the acquire takes and samples TOMBSTONED words.

| leg (probe tree, `bil`) | owner words asked at a landed commit | dead members | dead-owner writes | suite |
|---|---|---|---|---|
| ft_inv exponential | 39,899,794 | 0 | 0 | 152/152 |
| ft_inv per-node | 44,612,485 | 0 | 0 | 152/152 |
| ft_unit exponential | 5,061,857 | 0 | 0 | 358/358 |
| ft_unit per-node | 5,105,301 | 0 | 0 | 358/358 |
| ft_inv exponential, RED | -- | 5 sampled (3 taken, 1 shared, 1 coarsened-taken) | 4 sampled | key loss (`an OK insert is NOT READABLE`), hang in test 27 |
| ft_inv per-node, RED | -- | 4 sampled (taken) | 5 sampled | key loss (2 reports), hang in test 27 |

The same legs' record verdicts: NOLOCKS 0, UNHELD 0, SELF_ONLY 0 at both
spacings; HELD 38.3M + held-coarse 1.7M (exponential), HELD 44.7M (per-node).

### 7.3 What the measurement does NOT cover

- **The always-MW lanes are out of scope, not a gap**: they are lockless by
  design (§3), so they have no lock holder to audit. That is 21.4M
  (exponential) / 19.8M (per-node) records per ft_inv leg.
- **Raw stores outside the record layer** (`ft_hlist_store_*` chain words,
  `ft_set_parent_excl` HIDDEN) never reach a record.
- **Bulk ops (WLOCK)** are excluded by design: the FT-wide lock with the point
  mode flip. The skeptic of §5.1 notes their re-homes carry no state guard, so
  §5's held-order argument rests on the drain, not the guard.
- A zero from the suites is not a proof over interleavings the suites never
  produce.

### 7.4 Raw tombstone reads (the §5.1 lesson)

A raw `state & FT_STATE_TOMBSTONE` can only UNDER-report: a parked record is
`ptr | FT_STATE_PROXY`, and bit 1 of an aligned pointer is 0. So a raw read is
sound exactly when its "not tombstoned" branch is followed by a take or a sample
that refuses PROXY. The coverer check broke that rule: it proceeded on a node it
never locks. Every other decision read found:

| site | use | verdict |
|---|---|---|
| `ft-remove.h` `ft_rm_holder_rehomed`, `_cds_ft_remove_locked` (`ft_flag_tombstoned` x5) | routes the plan (re-descend / retry / miss) before the acquire | conservative: a missed tombstone routes as live, and the acquire refuses the PROXY |
| `ft-remove.h` `ft_detach_node` climb (`ft_meta_tombstone`) | pre-lock bail of the in-place tier | pre-lock filter; the hoist's acquire re-checks the node it writes |
| `fractal-trie.c` `_cds_ft_debug_cow_replace_root`, `ft-rekey.h` x3, `ft-remove.h` `ft_detach_node` | scrubbing released marks ("consumed fence keeps answering holds()") | after `ft_meta_lock_release_if_held` waits out PROXY; an under-report scrubs, so the op re-takes and is refused |
| `ft_glue_held_snap_one` x2 | does a consumed free-list entry answer holds() | under-report answers "not held", so the op takes and is refused |
| `ft_dlm_acquire_set_at` dedupe | covering hold is LOCK, PROXY or TOMBSTONE | PROXY-inclusive |
| `ft_flip_txn_record_anchor_release*`, `ft_remove_one_commit`, `ft_glue_tombstone_free_list` | `urcu_txn_load` | resolved through the txn |
| trace, stats, verify, and the probes themselves | diagnostics | not a decision |
