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

**Limits.** Only paths the two suites reach (§4). The always-MW lanes carry
no owner, so B counts them but cannot score them. §3 says which are MW on
purpose and which are unfinished. HIDDEN declarations are
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

### 2.1 Rows 6-8 fixed; row 9 open (2026-09-17)

**Rows 7 and 8 were one defect in the derivation.** `ft_descent_anchor_at_level`'s
pending-level fallback, `a->bound ? a->bound : d->nf`, names the node starting at
`bound_start` through the cursor. It assumes the cursor is the not-yet-entered node
at `d->depth`. `ft_walk_extend` leaves the cursor on the node it ENTERED (row 7), and
`ft_node_recompact` left it skip-encoded (row 8). The member passed to the query
starts at `bound_start`, so it IS that node: the query now takes the member
(`self`) and answers with it when `bound_start == clamp`.

**Row 6** gives `ft_detach_node`'s first orphan walk its own descent copy, extended
per orphan with its depth advanced, as the phase-2 walk already did. Re-entering
key-path nodes in order restores each pending level before a deeper member is
queried.

Probe A and the anchor-property probe (§7), ft_inv at exponential, before -> after:

| site | before | after |
|---|---|---|
| `ft_detach_node` orphan walk (row 6) | 65,290 mismatches of 150,365 | **0** of 171,553 (ft_unit 2 -> 0) |
| `ft_detach_orphan_planlock` (row 7) | 3,025: 1 above, 1 cursor-is-parent, rest off-path | 2,300, **all off-path** (stale plans: the op anchored on an older copy of the parent) |
| `ft_node_recompact` (row 8) | 0 (11 at 09-16f) | 0 |
| `ft_merge_lock_overlap` (row 9) | 29 | 27 -- **not fixed** |
| non-locked-word writes: op holds A and the parent shares A | 1,090,320 of 1,090,340; 2 SELF_ANCHORED | **1,160,550 of 1,160,552; 0** |

The live E.2 violation above no longer reproduces at HEAD before this change
(0 of 400 filtered runs; the rows 1-4 conversions changed that test's
interleavings), so it cannot confirm the fix. The probe rows do.

One on-path mismatch remains at `ft_chain_compress_fused:1614` (1 in 2.46M). Its
descent dates the compressed parent's end at byte 18 while the member starts at
byte 12. That is not the pending-boundary case, so it is unaffected by this change.
Not yet explained.

**Row 9 is not an agreement defect to fix; it is a bulk op that should take no
DLM lock at all.** Its 27-33 mismatches come from the cross-trie merge
(`inv_ordered_bulk_consistency`, `inv_merge_root_src_cross_view`): `ft_merge_build`
fences each dst overlap node through the glue's descent, which ends at the merge
point. But `cds_ft_merge_at` holds the dst's FT-wide writer lock for its whole
body (its exclusive source skips every grace-period wait), and while a bulk op is
live a fine trie's point ops re-take that lock (G5.25). So the fence excludes
nothing that is not already excluded, whatever word it takes.

Mathieu: the bulk-op transformation is INCOMPLETE -- bulk ops "should not have to
take _any_ node or exp locks, because they exclude all other ops with per-ft
locking". Two facts support it:
- the FT-wide lock is dropped only at `ft_writer_lock_gp_wait` (reader drains in
  detach, graft, graft_swap and rekey on a live trie);
- the seam rule forbids holding a node lock across one.
The one duty a take still performs incidentally is refusing a TOMBSTONE/PROXY
word for a plan read before a drain seam and used after it. The next step is to
measure which bulk-op plans cross a seam.

**Stale plans, not defects.** The other ~2,330 mismatches (2,192 at the
plan-lock, 85 at `ft_chain_compress_fused:1614`, 25 at
`ft_flip_txn_lock_or_guard_parent_ex`, 13 at `_cds_ft_insert:3812`, 10 at
`ft_node_recompact`, 2 at `ft_unchain_node`) are OFF PATH, and **every one**
has its anchor or its member TOMBSTONED at the check. That is the §2.1
"planned against the pre-change node hits TOMBSTONE and replans" path working.

---

## 3. The always-MW lanes — two by design, the rest unfinished

`ft_flip_txn_record_tag_mw` names no owner, so these records carry no coverage
verdict. Mathieu's classification (2026-09-17):
- **MW on purpose:** the root pointer, and the cell sibling list.
- **Everything else should be SW under lock.** Anything still MW here is an
  unfinished transition, not a design choice.
- **`nr_keys`:** a rank-stats group is coerced to `CDS_FT_WRITER_LOCK_COARSE`
  (`ft-lifecycle.h`), so every writer holds the FT-wide writer lock. It may be
  SW under that condition only (register note 9).

Records per ft_inv leg, by class (the `-DFT_DEBUG_STRUCT_ANCHOR` legs of §7.2):

| class | exponential | per-node | what it is | classification |
|---|---|---|---|---|
| ROOT | 1,911,262 | 1,892,283 | `&ft->root` (graft swap, remove, root list swaps, graft/merge publishes) | **MW on purpose** |
| CELL | 9,405,804 | 8,837,644 | every non-structural-tag edge through `ft_ord_cell_flip_into` / `ft_ord_cell_record_into_ft` (remove 4.6M, swap publish 1.8M, remove rec 1.3M, root list swaps, graft): the key-ordered CELL LIST links and deletion marks, plus ONE state-word edge -- `ft_remove_one_commit`'s not-held `nr_child--` (`ft_state_edge`, tag `FT_STATE_PROXY`), counted here because it shares bit 0 with `URCU_TXN_TAG` | the cell list is **MW on purpose**; the `nr_child--` edge is not a cell edge (not split yet). ☠ NOT the duplicate chain: `cds_ft_node.next` / `prev` are a different list, stored through `ft_hlist_store_*` and not counted in this class |
| HEAD_BACK | 6,449,049 | 6,288,104 | head back edges: `ft_node_recompact`'s re-home sweep 4.4M, `ft_chain_compress_fused` 0.88M, `ft_park_live_parent_edge` 1.08M, detach, remove_all | **unfinished** (owner: the chain holder) |
| STATE | 1,673,695 | 984,569 | `{live -> live}` validations of re-homed children (= the PARENT_WORD count); at exponential also **599,063 MW anchored retires** (`ft_chain_compress_fused`, `ft_detach_freeze_one`, insert, recompact, glue) | the retires are **unfinished** writes; the validation is a read check, which the register keeps MW ("a park validates nothing") -- ☐ to confirm |
| PARENT_WORD | 1,074,632 | 984,569 | `ft_reparent_record_meta`, the child not held | **unfinished** (owner: the parent) |
| DUAL_UNNAMED | 463,642 | 451,030 | `_cds_ft_insert_replace`'s leaf replace: the list-off arm 437k (`ft_ord_cell_flip_into`), the list-on arm 27k (`ft_ord_cell_swap_publish_multi`). ☠ NOT cell edges: STRUCTURAL edges built by hand with no owner -- the forward edge, or a SKIP_X dual plus `cn->child` | ☑ asked at every arm (§10.3); the dual's grandparent was NOT held, now taken |
| PSO | 411,687 | 408,417 | the same re-parent sweeps | **unfinished** |
| RANK | 1,515 | 1,515 | `nr_keys` via `ft_flip_txn_record_count_parent` (coarse tries only) | **unfinished**: SW under the FT-wide lock, COARSE only |

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

- **The always-MW lanes carry no owner**, so nothing asks about them: 21.4M
  (exponential) / 19.8M (per-node) records per ft_inv leg.
  - The root pointer and the cell sibling list are MW on purpose and have no
    lock holder to audit.
  - The rest should be SW under lock (§3), so they ARE a gap in this audit.
    That is HEAD_BACK 6.4M, PARENT_WORD 1.1M, PSO 0.4M, the 0.6M exponential MW
    retires, and the non-sibling part of CELL and DUAL_UNNAMED (not yet split).
    Asking about them needs the owner named on each record, which is the same
    step their conversion to SW needs.
  - `nr_keys` writers all hold the FT-wide writer lock (COARSE only).
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

---

## 8. The live validation of a re-homed child, measured (Mathieu: "I worry about the live validation")

When an op re-homes a child whose state word it does not hold,
`ft_reparent_record_meta` records that word `{live -> live}` MW. This is a read
check: the commit aborts if the child was locked, retired or changed meanwhile.
The code gives its purpose as catching "a child a PEER froze mid-recompact".
`-DFT_DEBUG_LIVE_VALIDATE` (on top of `-DFT_DEBUG_TXN_KIND -DURCU_TXN_REC_DBG
-DFT_WINNER_DBG`) records, per re-home call site:
- how many checks were recorded;
- whether the child is its own anchor, and whether the op holds the child's anchor
  (with `-DFT_DEBUG_STRUCT_ANCHOR`);
- for each check that LOSES, what beat it, read from the value the losing CAS
  observed.

ft_inv, one leg each:

| re-home site | per-node: checks / lost (cause) | exponential: checks / lost (cause) |
|---|---|---|
| `ft_reparent_record`, skip-compressed child (its callers: the `ft_node_recompact` sweep, compaction's compressed relocation, the rekey COW, the bulk glue) | 187,662 / 1,910 (LOCK held by a member claim 1,792, all `ft_detach_orphan_planlock`; LOCK unclaimed 118) | 192,304 / **0** |
| ... internal child | 533,293 / 0 | 518,790 / 0 |
| ... compressed child | 183,158 / 0 | 197,348 / 0 |
| chain-compress fuse, the surviving child (`ft_record_child_back_edge`) | 92,772 / 4,839 (LOCK unclaimed 2,545, nr_child 2,141, member claim 153) | 87,110 / 235 (LOCK unclaimed 227) |
| split's live child (`ft_park_live_parent_edge`) | 20,398 / 77 | 23,714 / 86 (LOCK unclaimed 85) |

At exponential, **every** check on a child that is not its own anchor was made
by an op that already holds that anchor. That is 220,007 of 220,007 across the
five sites, where property 2 predicts it (§7). For those children the check is
redundant.

**RED CONTROL, `FT_LV_RED_DROP=1`:** the check is dropped inside the
span-preserving re-homes only (`ft_node_recompact`'s sweep and compaction's
compressed relocation, bracketed by `FT_LV_SPAN`). No anchor moves there, and a
peer that retires or recompacts a child must rewrite the child's slot, so it holds
the parent the re-homer holds.

| leg | checks dropped | suite | exclusion violations | dead members | dead-owner writes |
|---|---|---|---|---|---|
| ft_inv per-node | 994,183 | 152/152 | 0 | 0 | 0 |
| ft_inv exponential | 1,056,364 | 152/152 | 0 | 0 | 0 |
| ft_unit (both spacings) | 18,909 | 358/358 | 0 | 0 | 0 |

The baseline legs without the drop read the same zeros.

The two boundary-moving re-homes (the split's live child, the chain-compress
fuse) keep the check. The split's is what closes §5's held order (§5.1 red
control). There the re-homed node is a compressed node's SINGLE child and the
old anchor of everything below it. Taking its lock instead of validating it
avoids the fan the reverted lock-the-sweep experiment choked on.

---

## 9. Bulk ops take no node lock; compaction follows the mode flip (Mathieu, 2026-09-17)

**The rule.** A bulk op excludes every other op through per-FT locking: the bulk
lock, and point ops flipped onto the FT-wide writer lock while the gate is open
(G5.25). So it should take no node or anchor lock at all. Point ops and
compaction take BOTH the FT-wide lock and their nearest-ancestor locks in bulk
mode, because the regime is mixed at the gate's two edges:
- while the gate opens, ops that saw it clear hold node locks only until its grace
  period ends;
- after it closes, fresh ops are back on node locks while flipped ones may still run.

Only the bulk BODY, strictly inside the gate, may drop them.

**Compaction now takes part in the flip, per step.** `cds_ft_compact_step` enters
the writer scope inside its own read-side section, as a point op does, so the
gate's grace period covers a step in flight and a bulk op that starts between two
steps makes the next one queue. Sampling once per `cds_ft_compact` pass did not
work: the pass drops the read lock between steps, and the gate's grace period
completed in that gap. The step keeps its relocations' node locks (row 4).

**The inventory** (`-DFT_DEBUG_BULK_ACQ`): every DLM acquire made inside a bulk op.
- ft_inv makes 1,201,459 (exponential) / 1,064,667 (per-node) of them, at 12 sites:
  - `ft_flip_txn_lock_or_guard_parent_ex` 740k;
  - `ft_node_recompact` 178k;
  - `ft_root_attach_fence_empty` 89k;
  - `ft_merge_lock_overlap` 59k;
  - `ft_graft_keylen` 21k;
  - `ft_split_compressed_graft_build` 21k;
  - `ft_rekey_cow_stop` 2.7k;
  - five sites below 120.
- ft_unit reaches 26 sites, including the rekey graft attempts and the glue marks.
- **Every one** is made under the FT-wide lock of its trie, or on an exclusive trie.
- **None** follows a drain seam (`ft_writer_lock_gp_wait`) earlier in the same op.
  The arm yield confirms seams occur inside bulk ops: 825,704 / 708,654 in ft_inv,
  45 in ft_unit.
- So no bulk-op take provides exclusion or stale-plan detection that per-FT locking
  does not already give.

**Removing those takes is QUEUED AS AN OPTIMIZATION (Mathieu: "having bulk ops
take fine-grained locking is not a correctness issue").** A first prototype
answered every bulk-op member at the acquire choke point as already held. It did
not survive, because bulk bodies are wired to a real take at about 124
lock-lifecycle call sites:
- `ft_meta_lock_release` 52, `ft_flip_txn_lock_register` 24,
  `ft_meta_lock_release_if_held` 16, anchor/held release records 16,
  `ft_flip_txn_record_release_lock` 9, fenced tombstones that expect `LOCK` 7;
- `inv_graft_root_swap_cross_view` registered the untaken lock and the commit's
  release sweep asserted;
- the rekey fold tests chained an MW record onto an SW one;
- the rig slowed from 250+ to 1-5 bulk cycles in 20 s.

Two shapes for later:
- **A (central):** a sentinel `lock_snap` for "held by per-FT exclusion, never
  taken", honoured by register, the release records, the fenced tombstone, and a
  bulk-scoped `ft_meta_lock_release`.
- **B (per site):** each bulk site stops acquiring and drops its lifecycle code.

`doc/design/ft-compact-bulk-rig.c` is the harness for it.

### 8.1 The live check removed where it observed an exclusion the op already holds, and TAKEN where it did not (Mathieu: "let's try" it)

**A. Span-preserving re-homes, no check.** `ft_node_recompact`'s sweep and
compaction's compressed relocation now pass `check_child = false`. No anchor
moves, and a peer that could freeze or retire the child must rewrite its slot in
the node being copied, whose lock the re-homer holds.

**B1. The split TAKES its live child.** A compressed split ADDS a boundary above
cn's only child without retiring that child, so everything anchored on the child
re-anchors under the fresh junction. `ft_insert_dlm_acquire_split` now takes the
child (when internal) in the same lock set as {cn, P}, and re-reads `cn->child`
under cn's lock. The lock is carried on the insert commit like P's, released on a
pre-publish bail, and registered with its release record in
`ft_insert_publish_or_park` before the arm. `ft_park_live_parent_edge` asks the
txn registry, so a held child records no state edge.

Rig `splitheld` (the §5 held order):

| tree | split while the victim holds A | peer while held |
|---|---|---|
| child locked | waited | waited |
| RED `-DFT_RED_SPLIT_NO_CHILD_LOCK` (child neither locked nor checked) | COMPLETED | COMPLETED -- **HOLE, 3/3** |

**B2. The fuse, no check.** The chain-compress fuse moves boundaries only by
REMOVING them: the boundary node, and a fused compressed child. It retires and
holds both. A node anchored on either one is excluded by the fuse, and a plan
made before it hits the tombstone. A node anchored on the surviving child or
below keeps its anchor. Its per-node losses (4,839 of 92,772) were aborts for
nothing.

Measured, `-DFT_DEBUG_LIVE_VALIDATE`, ft_inv, per-node / exponential:

| | before | after |
|---|---|---|
| live checks recorded (all sites) | ~1.02M / ~1.02M | 150 / 307 (the split's untaken children, 0 lost) |
| checks lost | 6,826 / 321 | 0 / 0 |
| always-MW STATE records | 984,569 / 1,673,695 | 150 / 646,693 (the rest: MW anchored retires) |
| exclusion violations, dead members, dead-owner writes | 0 | 0 |

**Skeptic: refuted=false.** Two hardening items, both applied:
- the re-read of `cn->child` under cn's lock is unconditional, so a child that
  became internal after the plan is re-planned, never re-homed unlocked;
- the child's release is recorded through the txn
  (`ft_flip_txn_record_anchor_release_held`, as P's is), not as a fixed
  `{LOCK|snap -> snap}`.

The skeptic's open question was LIVENESS: the split now also conflicts with ops
that hold the child but not cn, and escalation rescues a refused commit, not a
refused acquire. Measured with `-DFT_DEBUG_LIVE_VALIDATE` counters in
`ft_insert_dlm_acquire_split`, ft_inv, against the same tree with the child lock
compiled out:

| | split acquires | with a child member | refused | longest refusal streak (one thread) |
|---|---|---|---|---|
| per-node, child locked | 1,035,083 | 24,367 | 87,592 | 132 |
| per-node, no child lock | 1,044,292 | 0 | 101,638 | 545 |
| exponential, child locked | 923,969 | 27,903 | 123,205 | 134 |
| exponential, no child lock | 931,228 | 0 | 100,605 | 150 |

No starvation signal. Refusals move by run-to-run amounts in both directions, and
the longest streak is not worse. Only about 2.5% of splits have an internal child
to lock.

## 10. The unfinished lanes' owners, and the (parent, offset) pair stored plain (2026-09-17)

### 10.1 Who should lock each unfinished lane, and is it held

`-DFT_DEBUG_STRUCT_ANCHOR` now asks each unfinished always-MW lane (`ft_sa_lane_ask`)
against the owner the register names for it:
- a head back edge: the chain holder that its old value names (compaction's cell
  relocation: `cell->parent`);
- `parent_word` and `parent_slot_offset`: the parent being replaced;
- `_cds_ft_insert_replace`'s structural edges: the node that contains the slot.

It gives the same verdicts as an owner-bearing record, plus `O_CHILD_ONLY`: not
covered by the owner, but the op holds the child's anchor. A head back edge is
asked with the site's own lock context (`-DFT_DEBUG_BACK_EDGE_OWNER`). Some holds
live only in the context's `extra[]` frames, and without it the probe reads
93k-310k uncovered records that are in fact held.

| ft_inv | asked | uncovered | child-only |
|---|---|---|---|
| per-node | 8,666,129 | **0** | 0 |
| exponential | 8,381,861 | **4** (1 recompact head back edge, 2 fuse `parent_word`, 1 insert_replace) | 1 |

ft_unit: 0 uncovered at both spacings. The 4 were re-asked at commit, and one arm of
`_cds_ft_insert_replace` was never asked: see §10.3.

### 10.2 The pair stored plain on a reachable node

`ft_resolve_parent_slot` treats `parent_word` as a sequence word for the offset.
So the precondition is: every offset change goes through a parked `parent_word`,
or the node is invisible. `incoming_byte` is read by the key up-walk beside the
parent, with nothing parking it.

`-DFT_DEBUG_PAIR_STORE` asks at every store that CHANGES the offset or the byte:
is the node reachable through its current pair, all the way up to the trie's
root slot? Hits are counted per call site.

☠ Two earlier versions of the climb read false positives:
- one hop: a fresh cluster built bottom-up holds its own children;
- a root-position parent word taken as the root: a fresh node is stamped with its
  owning trie before it is published. The climb now requires `owner->root` to hold
  the top, and the owner not to be exclusive.

That second hole alone made up the whole incoming_byte "producer": 295 per ft_inv
leg, from the merge build and `ft_node_recompact`'s `FT_EXCL_HIDDEN` arm.

| ft_inv, per-node / exponential | result |
|---|---|
| incoming_byte changed on a reachable node | **0** / **0** |
| control: same-value byte stores on reachable nodes | 1,050,066 (per-node) -- the climb answers yes |
| offset changed on a reachable node | 36,335-56,246 / 42,491-50,947, **one site**: `ft_glue_record_back_edge` → `ft_set_parent_slot` |

ft_unit: 0 and 0.

The glue site stores a live child's offset plain before the commit parks its
`parent_word`. It is unobservable, for three reasons:
- **readers** never read the offset: the up-walk and `ft_skip_to_compressed` read
  `parent_word` only. The one iterator use is an `FT_DEBUG_PARENT_VIOLATION` dump;
- **other writers** are excluded: every caller is a bulk op, whose gate is
  published and followed by a grace period, and point ops and compaction steps
  then take the FT-wide lock (§9). ☠ So this is a DEPENDENCY on the FT-wide lock,
  the same one `ft_glue_record_back_edge`'s FT-SLOT-3 note names;
- **the op itself**: a per-thread ring notes each glue store and checks it in
  `ft_resolve_parent_slot`, cleared when the outermost FT-wide scope releases.
  0 torn reads of 59,433 (per-node) / 73,167 (exponential) noted.
  RED `-DFT_DT_GLUE_RED` resolves right after the store: 67,880 of 67,880.
  `ft_txn_parent_slot_at` reads both words through the txn, so it is coherent by
  construction.

### 10.3 Every DUAL_UNNAMED record asked, and the uncovered re-asked at commit

**Commit-time re-ask.** An uncovered lane record is kept on the txn and asked
again when its commit goes to the engine, this time without the site's lock
context:
- `c_LATE`: covered by then;
- `c_NEVER_OK`: still uncovered, and the commit landed;
- `c_NEVER_AB`: still uncovered, and the commit aborted;
- `c_BAILED`: the txn never reached the engine (an acquire miss, an op bail).

A LATE report prints the registry and ledger counts at the record and at the
commit. If they are unchanged, no lock arrived in between, so the owner was
covered at the record too. RED `FT_SA_RED=root` (ft_inv exponential): late
2,697,912, NEVER_OK 24,941,870, never_ab 57,301, bailed 59,932, overflow 337,714.

**The arm that was never asked.** §10.1 said the hot DUAL_UNNAMED path was
unasked. That was a misread: the ask with 2 hits per leg is the list-off leaf
arm's SKIP_X variant. Its single-edge ask read 493,072 (exponential) / 582,596
(per-node) asked, 0 uncovered. The arm that had no ask was the LIST-ON leaf arm
(`ft_ord_cell_swap_publish_multi`). DUAL_UNNAMED is filed by `_cds_ft_insert_replace`
alone, every arm of it. The per-site split (`mwa5`) names no other producer. Its
edges are built by hand and never pass `ft_pub_rec_add_at`, so the recorder that
read `unnamed = 0` never saw them.

Asked at the list-on leaf arm, per leg:

| | ft_inv per-node | ft_inv exponential | ft_unit |
|---|---|---|---|
| single forward edge (owner P) | 10,620 held | 10,313 held (4 exact, 10,309 via an ancestor) | 1,713 held |
| SKIP_X variant, `cn->child` (owner cn = P) | 15,729 held | 10,279 held | 130 held |
| SKIP_X variant, the dual (owner GP) | **15,729 UNCOVERED, all NEVER_OK** | **10,279 UNCOVERED, all NEVER_OK** | **130 UNCOVERED** |

The leaf replace under a compressed parent wrote the grandparent's dual slot
holding only {P}. §8.1 A drops the live check on the premise that a writer of a
child's slot holds the node it lives in, and this writer broke that premise. The
list-off arm has the same shape, reached about once per leg. `cds_ft_replace`
already takes GP (`ft_lock_skip_dual_gp`, §9.3's third member).

**Fix.** Both leaf arms take GP with `ft_lock_skip_dual_gp` after {P}. They
derive the edges after that acquire (`ft_insert_replace_leaf_sedges`), from the
same read-your-own-writes derivation the acquire used. Each reserves one more
record. The records stay MW.

After, per leg (the `gp`/`gl` legs):

| | uncovered | re-ask at commit |
|---|---|---|
| ft_inv per-node x2 | 0 | -- |
| ft_inv root-only | 0 | -- |
| ft_inv exponential x7 | 0-4 per leg, all `UNDATED` at the record: 14 total, 1 covered through its child | the other **13: all LATE, 0 NEVER_OK**; the 8 printed with counts all read `locks 1->1 ledger 0->0` |
| ft_unit per-node, exponential | 0 | -- |

The dual asks now read held: 7,948 of 7,948 (per-node), 12,331 and 9,156
(exponential), and held_coarse at root-only. The exponential residue is head
back edges (txn sites ft-insert.h:1163 and ft-remove.h:4785) and the fuse's
`parent_word`. The climb that dates the owner reads the words without locks, and
could not date it past a peer's park. At commit the owner's anchor is held, and no
lock was taken in between. **No lane record commits uncovered.**


## 11. Readiness for the MW→SW flip, measured (2026-09-17)

The flip happens once, after the lock-set transition is complete at FINE and
EXPONENTIAL (Mathieu). This section records what the instruments say about
each class that is to become SW, and what is still open.

### 11.1 What the flip changes

Per the register (fractal-trie-internal.h), a structural slot at exponential
and root-only is MW *by construction*: `ft_flip_txn_arm_per_op` refuses any
spacing but per-node (`ft_txn_per_op_spacing_ok`, "door 2"). The refusal
exists because `ft_flip_txn_owns` is exact at per-node and anchor-blind above
it. So the flip is two changes that must land together:
- lift door 2, with an owner witness that resolves the anchor (the debug
  owner assert would otherwise fire on every coarse leg, as it did when
  measured before);
- convert the unfinished always-MW lanes (§3, §10) to owner-dispatched records,
  the exponential anchored retires, and the not-held `nr_child--`.

`-DFT_DEBUG_STRUCT_ANCHOR`'s record-time inventory is the anchor-aware
witness that door 2 lacks.

### 11.2 Measured, per class (ft_inv, probe trees `blane4` / `bchain3`)

| class | per-node | exponential | root-only |
|---|---|---|---|
| owner-bearing records (MW_STRUCT surface) | 0 unheld | 0 unheld; UNDATED re-asked, see 11.3 | 0 unheld; 6 UNDATED, all LATE |
| unfinished lanes | 0 uncovered | 0 NEVER_OK (§10.3) | 0 |
| duplicate chain (`-DFT_DEBUG_CHAIN_HOLD`) | 4 unheld | 19 unheld | 1 unheld |
| anchored retires, P1/P2 (§7, 09-17) | -- | 1,160,550 of 1,160,552, 0 exceptions after rows 6-8 | -- |

ft_unit: 0 at both spacings on every row.

**The chain audit had no exclusive-trie bucket.** It scored an exclusive trie
(a bulk product or source, no peer) as UNHELD. At the in-place delete's fused
freeze_leaf that was the entire residue: 57,823 per ft_inv per-node leg and 24
in ft_unit, confirmed one for one by a probe splitting the same population on
`ft->exclusive`. Bucketed now (`exclTrie`). The site then reads 0 at every
spacing (149,152 / 119,208 exclusive records per leg).

☠ A first probe at that site counted "registry and ledger both empty" and
pointed at the external promote. It was blind to the orphan plan-lock marks
in `lctx`'s extras, which the audit counts as HELD(ctx). A lock hoist written
from it closed nothing and was reverted.

### 11.3 Closed (2026-09-17)

The five open items, in order. Probe trees `blane4` (struct anchor) and
`bchain3` (chain audit, `-DFEATURE_FT_HOLD_TRACE`). All legs green.

1. **Chain, `ft_detach_node:3941`: fixed.** The freeze sat in the helper's
   retiring arm (`topmost=0`, no orphans): the leaf hangs straight under `cn`,
   which only the helper's {src_cn, pub_parent} set takes. Recording the freeze
   in the caller first was a record made holding nothing. It now happens inside
   `ft_detach_node_replace_compressed_parent`, after each arm's acquire (and on
   the bulk split re-home return). Chain audit: 0 violations at every spacing.
   The moved row reads held: 47,618 per-node (own lock), 69,923 exponential
   (17,022 own + 52,758 ancestor), 41,800 root-only (ancestor).
2. **UNDATED owner-bearing records: settled.** The record keeps the owner's held
   ancestors (`anc_rec`). At commit the report says whether the owner's anchor
   was among them, or, for an owner still undatable, whether every ancestor held
   at commit was already held at the record. 8 exponential legs: every LATE
   record reads `anchor_held_at_record 1`, and the 7 still undatable read
   `ancestor_held 1`, subset 1. No hold arrived between record and commit on
   any of them.
3. **The fuse's `parent_word` lane: settled.** Its `locks 1->2` is an orphan
   anchor registered after the record, not the owner's. The owner's anchor was
   already held at the record in every sample (`anchor_held_at_record 1`).
4. **Unscored rows: scored.**
   - Every chain-store call site now has a FULL audit row. The three remove_all
     freezes that had none (the NIL key's two, the prefix clear) read held:
     registry, or ancestor at root-only.
   - HIDDEN declarations are now tested. Is the target reachable, through its
     current (parent, offset) pairs up to a non-exclusive trie's root slot, or
     for a head through its holder's slot? Per ft_inv leg: 5,616,097 /
     4,848,829 / 5,042,418 asked (per-node / exponential / root-only), **0
     reachable**; ft_unit 1,331,448, 0.
   - Controls: LOCKED in-place body writes read reachable at ~100% (counters
     are unlocked, so within a few dozen). Live removed leaves (heads) read
     reachable at 99.95%; the rest are heads moving mid-scan.
   - ☠ The control first read 3% at two sites. The climb did not resolve a
     SKIP_X dual, which names cn's child and not cn, so every path through a
     skip-compressed node read unreachable. The head test also asked a
     compressed holder by byte. Both are fixed; without the control, the
     HIDDEN zeros were blind there.
   - `ft_popcount_node_set_nth:516` is the REFUSED arm (`-ERANGE`, no store).
     Its `lockVIOL` counts callers that declined the in-place tier, on nodes
     with no parent word yet. Not a write.
5. **The not-held `nr_child--`: split and asked.** `ft_remove_one_commit`
   records it directly, with `FT_STATE_PROXY` spelled out, under its own
   always-MW class `NR_CHILD_DEC`. It is the same MW record as before, no
   longer counted in CELL. Per ft_inv leg:

   | | per-node | exponential |
   |---|---|---|
   | fine, shared trie | 0 | 536,499, anchor held (`ft_sa_anchor_props`) |
   | coarse trie, FT-wide lock | 286,007 | 273,252 |
   | exclusive trie | 104,643 | 95,269 |
   | fine, bulk (FT-wide lock) | 0 | 25 |

   Root-only: 2,084,317 of 2,084,321 anchor held, 4 undatable.

Gate after the batch, clean rebuild: 18/18 GREEN. `bf-nsk`
(`NO_FEATURE_FT_SKIP_COMPRESSED`): ft_unit and ft_inv at per-node and
exponential, GREEN.

### 11.4 Queued for the flip (Mathieu, 2026-09-17)

**`ft_remove_one_commit`'s not-held `nr_child--` keeps its MW kind until the
flip.** Its kind is chosen by `ft_flip_txn_owns` -- the per-node lock registry --
so it takes the hardcoded-MW branch wherever the registry is empty, which
includes the tries that are armed SW trie-wide by door 1:

| per ft_inv leg | per-node | exponential |
|---|---|---|
| coarse trie (FT-wide writer lock) | 286,007 | 273,252 |
| exclusive trie (no peer) | 104,643 | 95,269 |
| fine, shared (op holds the ANCHOR, an ancestor) | 0 | 536,499 |
| fine, bulk (FT-wide lock) | 0 | 25 |

Nothing chose MW for the first two rows: the txn around them parks every other
state-word record SW (`ft_txn_content_sw_ok`), and the other producers of this
very word (`nr_child++`, the tombstone) park SW there too, so the MW record is
the odd one out -- harmless only because those tries exclude every peer. MW
stays correct meanwhile (a CAS that arbitrates against nobody).

The third row is door 2's anchor-blindness: the op holds the holder's anchor and
`owns()` looks for the holder itself. ⇒ at the flip, pick the kind from the
trie's mode AND an anchor-aware ownership answer, and correct the register row
(`metadata.state nr_child--`), whose "MW [DESIGN]" is not backed by its own
note (2) -- that note is about the TAG (FT-SLOT-1), not the kind, and the code's
"self-guarded by this very CAS" comment predates the leaf-delete hoist.

### 11.5 The flip, spacing axis: door 2 opened for exponential and root-only

Mathieu chose both coarse spacings, and the chain class in scope (see §11.6 for
the rest). What door 2 refused was never the spacing -- it was the PREDICATE.
`ft_flip_txn_owns` compared the registry's words against the node a record
names, while a coarse spacing registers that node's ANCHOR, so it answered
"miss" for a word that IS excluded, and under an arm a false miss is fatal.

Four changes make the answer exact, and one makes a wrong answer harmless:

1. **The acquire's answer travels with the registration.** `ft_held_anchor`
   carries the MEMBER its word was taken for, `ft_flip_txn_lock_register_held`
   files both, and `ft_flip_txn_holds` matches either. Only the acquire knows
   this (the anchor comes from the op's descent).
2. **Members whose word deduped onto another member's** file no entry at all --
   they owe no release. The acquire records them in the txn's `@covered` list,
   and the three sites that hand a shared member on (the split's P and child,
   the fuse's retire and publish parent) say so explicitly.
3. **ROOT-ONLY needs no descent:** every node's anchor IS the trie's root, so
   `ft_flip_txn_owns` resolves it directly from the txn's trie (`@ft`, a
   production field; the debug copy it replaces was read NULL in a core).
4. **The kind is decided PER RECORD** (the former `-DFT_SW_REQUIRES_OWNER`): a
   per-op armed txn parks SW only where it owns the record's owner, and records
   MW otherwise. So an arm can no longer convert a word the op does not own --
   the structural weakness that made the arm's "registry is non-empty" gate too
   coarse -- and the park and the assert share ONE predicate.
   ☠ A whole-body SW writer (the rekey / root-COW driver, `@sw_body`) is EXEMPT:
   its edges sit on words it fenced itself, so an MW fallback aborts its own
   commit forever. Measured as a livelock at ft_unit test 111, then as a 16G
   leak at test 117, until the flag existed.

**Debug gate, `--enable-rcu-debug` with the owner assert armed, 6 legs GREEN:**
ft_unit 358/358 and ft_inv 152/152 at per-node, exponential and root-only.
Each gap was named by that assert, one at a time; the last two needed a
self-diagnosing dump and a core, because they did not reproduce under gdb.

**Positive control** (same tree, door 2 shut vs open, per ft_inv leg):

| | armSW | SW records | MW_STRUCT | OWN_MISS | ABORT |
|---|---|---|---|---|---|
| exponential | 3,531,046 → **9,624,073** | 5,714,002 → **11,662,220** | 39,598,068 → 35,674,503 | 1,620,759 → 1,121,390 | 654,580 → 744,820 |
| root-only | 3,224,762 → **7,323,652** | 4,435,496 → **8,060,515** | 25,881,500 → 23,443,122 | 5,242,617 → **1,928,817** | 1,782,700 → 1,581,854 |
| per-node | 10,879,803 → 10,281,676 | 13,579,840 → 12,452,716 | 39,359,409 → 38,564,634 | 1,847,956 → 1,691,751 | 859,505 → 699,931 |

Per-node loses about 8% of its SW parks: that is change 4 refusing to park
where the registry cannot confirm the owner. MW is correct there, just a CAS
that arbitrates against nobody, and its aborts fell too.

☞ USER-VISIBLE VALUE STILL WAITS ON THE API: both coarse spacings remain
refused by `cds_ft_group_attr_set_lock_spacing` unless
`FEATURE_FT_ANCHOR_VALIDATE` (§12.3). The flip makes them correct; graduating
exponential is a separate decision.

### 11.6 Still owed for the flip

The always-MW LANES are unchanged by §11.5 -- they call
`ft_flip_txn_record_tag_mw` directly, so they are MW at every spacing:
HEAD_BACK, PARENT_WORD, PSO, `_cds_ft_insert_replace`'s structural edges
(DUAL_UNNAMED), the exponential anchored retires, RANK, NR_CHILD_DEC (§11.4),
and the duplicate CHAIN class (`cds_ft_node.next`/`.prev`,
`ft_ord_cell.parent`), which Mathieu put in scope. Each needs its owner named
at the producer and then the ordinary dispatch; §11.2-§11.3 is the measurement
that says their owners are held.

## 12. API / design questions queued by Mathieu (2026-09-17)

Not implemented; recorded so the flip does not silently decide them.

### 12.1 An application-provided writer exclusion mode

Planned, removed from the API, to be restored: the app guarantees writer
exclusion (single-threaded, or its own mutex around every caller). It should
behave "pretty much as COARSE", except the contract comes from the caller and
the library takes no writer lock of its own. Debug builds keep the validation
macros that check the contract.

For the record kinds this costs nothing: `ft_txn_content_sw_ok` is
`!lock_fine || exclusive`, so such a trie is armed SW trie-wide by door 1, like
COARSE. The requirement is that the mode be DECLARED, so the library can read
it; nothing needs to be inferred.

Two things to pin when it returns:
- it is NOT `cds_ft_attr_set_exclusive`, which additionally promises NO
  concurrent RCU readers (and so licenses skipping reader-visible publication
  discipline -- the reachability climb of §11.3 treats an exclusive trie as
  unreachable by readers). This mode keeps readers, so proxies, tags and grace
  periods all stay. It is a third writer strategy;
- the existing hold-ledger oracle polices LOCK collisions, which this mode has
  none of. The contract to check is "no two writers inside the trie at once",
  i.e. a writer-scope entry counter that must never exceed one.

☞ AND IT RETIRES THE REMAINING MW (Mathieu): under app exclusion the always-MW
lanes can go SW too, except the ones that are genuinely MW by design. They are
MW today on a COARSE trie as well, and not because anything decided it: the
lanes call `ft_flip_txn_record_tag_mw` UNCONDITIONALLY (the root pointer, the
cell list, head back edges, parent_word, PSO, state, RANK, NR_CHILD_DEC), so
door 1 arms the rest of the txn SW and these stay MW. Nothing can race them
there and no validate can fail, so one predicate (`ft_txn_content_sw_ok`)
consulted by those recorders converts the lot. The only record that must stay
MW is the DLM lock TAKE -- the arbitration point -- and such a trie takes no
DLM locks at all. Independent of the fine-spacing flip; do it right after.

### 12.2 COARSE vs FINE + ROOT_ONLY -- redundant?

As a CONCURRENCY setting, yes: both serialize every structural writer. As
machinery, no:

| | COARSE | FINE + root-only |
|---|---|---|
| `lock_fine` | false | true |
| what serializes writers | the FT-wide writer lock (a fair mutex) | the ROOT node's state-word LOCK bit, taken by an MW CAS through the engine |
| lock sets | none derived | derived per op, every member's anchor collapsing onto one word (dedupe everywhere) |
| on contention | waits | refuses (-EAGAIN), the op re-descends; escalation for fairness |
| per-op cost | one mutex | an acquire record, a release/retire terminal, dedupe bookkeeping, guards |
| record kind | door 1, trie-wide | door 2, per record, keyed on ownership |
| bulk ops | FT-wide lock | FT-wide lock -- common |

Root-only earns its keep as the MAXIMAL-COLLAPSE TEST AXIS for the anchor
machinery, not as a configuration: every gap this flip had to close (members
deduping onto one word with nothing filing them, and a record whose owner is
covered only by the root) was invisible at per-node and exponential.

### 12.3 Gate ROOT_ONLY as non-public (Mathieu)

`cds_ft_group_attr_set_lock_spacing` already refuses EXPONENTIAL and ROOT_ONLY
unless `FEATURE_FT_ANCHOR_VALIDATE`, but the ENUMERATOR sits in the public
header, so the option reads as public API. Once the flip makes EXPONENTIAL a
real setting, the two part ways: exponential graduates, root-only stays a dev
axis behind its own config gate (its own macro name, not the anchor-validation
one it shares today).
