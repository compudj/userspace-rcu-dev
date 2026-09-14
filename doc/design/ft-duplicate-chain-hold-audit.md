# The duplicate-chain hold audit — every MW writer, and whether it holds the lock

**Status 2026-09-14.** Measured on `ft/unpub-free-audit`, working tree, with
`-DFT_DEBUG_CHAIN_HOLD -DFEATURE_FT_HOLD_TRACE`.

## Why this replaces the per-op conversion

Converting `cds_ft_node.next/.prev` from MW to SW one operation at a time does
not work. An SW record is sound only once **every** writer of those words is
excluded: a single unheld producer anywhere makes the word MW again, and it
does so silently — the engine poisons the descriptor and the op's retry loop
absorbs it. So the unit of work is THE WORD, not the op.

`ft-txn-hlist.h`'s own header already states the invariant:

> Single-writer per chain (MW LOCK_FINE Step A: **every chain mutation runs
> under the head-holder's node lock**).

Nothing checked it. `-DFT_DEBUG_CHAIN_HOLD` checks it, at every site.

## The predicate, and its three witnesses

At each write of a duplicate-chain word, ask: does this thread hold
`ft_chain_head_holder(node)`?

    WLOCK      inside a writer scope on this trie -- the FT-wide mutex
               excludes everyone, no per-node holder is taken   -> not a violation
    HELD(reg)  ft_flip_txn_owns(): the op's txn owns the holder
    HELD(led)  ft_hold_trace_holds(): the per-thread hold ledger sees it
    UNHELD     FINE trie, no writer scope, and NEITHER witness  -> ☠ VIOLATION
    coarseFT   a COARSE trie outside a writer scope -- a different question,
               bucketed and never scored

☠ **ALL THREE ARE REQUIRED.** Scoring on the ledger alone reported 423,154
violations in one row that the registry then showed were all held: the ledger
drops its entry the moment a release is RECORDED, while the word keeps
`FT_STATE_LOCK` until that commit lands. The witnesses split by MECHANISM —
ops that take the holder directly report via the ledger, ops that fold the
release into their commit report via the registry. Because both witnesses can
only ever OVER-report a hold, **UNHELD is a floor, not a ceiling.**

## ☠ THREE false-positive classes the probe had to grow past

Recorded because each one produced a confident, wrong number first:

1. **Ledger-only scoring** — 423,154 "violations" in one row that the registry
   showed were all held. The ledger drops on a RECORDED release while the word
   keeps `FT_STATE_LOCK` until commit.
2. **No WLOCK / coarse bucket** — every COARSE arm reads as a violation by
   construction, since exclusion there is the FT-wide mutex and no per-node
   holder is ever taken.
3. **Prepare is not commit** — `ft_flip_txn_lock_or_guard_parent` is ALL-OR-NONE:
   an acquire miss sets `acquire_miss` and `ft_flip_txn_commit` DISCARDS the
   descriptor before the engine sees it. A record made without the holder under
   that bit never installs. This alone accounted for 294,754 false violations at
   `_cds_ft_insert_replace`, which is in fact CLEAN (365,779 records: 128,588
   held, 248,315 aborting, 0 unheld). ☞ the stale header on
   `ft_flip_txn_lock_or_guard_parent_at` still describes the OLD degrade-to-guard
   behaviour that the ALL-OR-NONE block replaced; do not reason from it.

## Result — the violations that survive

Measured per row (the whole-suite run cannot be used: test 152
`inv_concurrent_insert_replace_coarse` grows without bound and the memcg SIGKILL
takes the destructor with it — a separate, pre-existing defect).

**A. Definitive — a bare CAS with no txn, so nothing can abort it.**

| site | rows | UNHELD |
|---|---|---|
| `_cds_ft_remove_all_locked:8108` | remove_all nolist/list/prefix | 50,517 of 50,517 = **100%** |
| `_cds_ft_remove_all_locked:8073` | (full suite) | **100%** |
| `_cds_ft_remove_all_locked:7783` | NIL-key arm | **100%** |

All three are `ft_chain_mark_removed_flip` called with **no txn at all** — the
"lone-store residual ... to be closed once the lone edge is forced through a
txn" their own comment names. They install unconditionally. **This is the debt.**

**B. ☑ NOT VIOLATIONS — the FOURTH false-positive class (settled 2026-09-14).**

These three read UNHELD only because the audit asked the NARROW witness. They
hold the chain holder in the op's ORPHAN PLAN-LOCK ARRAY (`@extra`), which
`ft_flip_txn_owns` cannot see **by design**.

| site | narrow witness | four witnesses | source |
|---|---|---|---|
| `ft_detach_node:3459` | 100% UNHELD | **0 UNHELD** | extra=3,114 |
| `ft_detach_node:4463` | 49.5% UNHELD | **0 UNHELD** | extra=4,823 |
| `ft_chain_compress_fused:2165` | 50% UNHELD | **0 UNHELD** | extra=75 |

Every ctx hit is `extra=` — zero via txn / glue / outer. `FT EXTRAS STALE` (the
tree's own over-report detector) fired 0 times in 10 runs. Rows:
`inv_remove_cross_view_compressed_parent`, `inv_concurrent_same_key_removes_nolist`,
`inv_remove_cross_view`.

The 50/50 split flagged above as "two code paths rather than a race" was exactly
that: one path registers in the txn, the other keeps its mark in `@extra`.

☠ **AND "FIXING" THEM WOULD HAVE BEEN THE DEFECT.** A second acquire on a word
the op already holds returns `-EAGAIN`, `ft_meta_lock_acquire` cannot tell the
op's own mark from a peer's, the site reads it as contention and re-plans, and
the op waits on ITSELF forever (`struct ft_held_set`'s header). That is exactly
how `FT_RM_ACQUIRE_FIRST` hung test 44 deterministically.

The witness is now the fourth column of the audit (`HELD(ctx)`), with a
breakdown naming which of the four sources answered.

**C. Clean.** `_cds_ft_insert_replace` (all four displacing arms),
`ft_chain_node`, `ft_unchain_node`, `ft_promote_head`, `_cds_ft_replace_locked`,
`ft_glue_record_splices`.

## Clean — 0 violations, and high volume, so this is coverage not silence

    ft_chain_node:2447            2,958,769   the APPEND (ledger; holder taken directly)
    ft_promote_head:5539          2,222,725   (registry)
    ft_unchain_node:6015          1,684,402   (registry)
    ft_unchain_node:5863            280,868   the interior unlink (ledger)
    _cds_ft_replace_locked:5243     260,826   (ledger)
    ft_promote_head:5444            139,628   (registry)
    ft_glue_record_splices:16175      4,442   (WLOCK)
    _cds_ft_insert_replace:4405       3,924   (registry)

Both appends derive the holder, acquire it, and RE-VALIDATE
(`ft_node_is_removed(head) || ft_chain_head_holder(head) != holder_flag`)
before walking the chain. That is the pattern the seven above are missing.

## Reached by no test — unproven, not clean

`ft-remove.h:5199` (the standalone `ft_node_mark_removed_flip` fallback in
`ft_detach_node`, "a no-op under one writer"), `ft-remove.h:7994`,
`ft-insert.h:4087`, `ft-insert.h:4182`, `ft-insert.h:5344`.

## Instrumented by no probe yet — static reading only

`ft-helpers.h:1622` and `ft-helpers.h:3566` (raw `rcu_assign_pointer(en->prev,
word)` back-edge stores, list-off); the four `new_node->next->prev = …` plain
stores to a LIVE neighbour in `ft-insert.h`; `ft-compact.h:400` (a cell edge on
`&head->prev`); `ft-mutation-helpers.h:9880` (a freeze edge on
`&freeze_leaf->next`).

## Why the per-op predicates could not have worked

`ft_member_pred_lost` and the `-ESTALE` chain-retired check were both attempts
to let a reader of the chain DETECT a displacement after the fact. The audit
says why neither can: at `_cds_ft_insert_replace:4492` the displacing writer
tombstones the chain **without holding it** 70.5% of the time, so there is no
serialization point at which the two loads a predicate needs are consistent.
The fix is upstream of every predicate — make the seven sites hold the holder.

## Locks added (phase 1, per-node spacing)

`ft_ra_sweep_held()` takes the chain holder across `remove_all`'s tombstone
sweep and releases it immediately, **never publishing into `@lctx`** -- that
scope is what got `FT_RM_REVALIDATE` refuted. Measured over the full ft_inv
suite:

| arm | writes | verdict |
|---|---|---|
| prefix (`:8209`) | 139,741 | **all HELD** (was 100% unheld) |
| NIL key (`:7901`) | 63,525 | **all HELD** (was 100% unheld) |
| leaf (`:8258`) | 3,774,910 | still unheld -- see below |

`FT_RA_SWEEP ok=203266 refused=0`: every acquire attempted SUCCEEDED, and
203266 = 139741 + 63525 exactly. ft_unit 355/355, ft_inv 152/152.

### ☠ The leaf and compress arms CANNOT be locked, and the reason is not contention

MEASURED on `inv_concurrent_remove_all_prefix` before the short-circuit went in:
`ok=0, refused=22007`, of which **22005 are -EAGAIN on a word whose state reads
`FT_STATE_TOMBSTONE`** (state=6 = TOMBSTONE | nr_child 1). Those two arms retire
the holder IN the very commit that unlinks the chain, so by sweep time there is
no live word left to take -- **the lock is not lost, it is GONE.**

Nor can the acquire be hoisted above that commit:

- unpublished, `ft_detach_node` climbs from `holder_meta` and its own acquire
  misses **against the op's own hold** -- "the op waits on itself, forever";
- published into `@lctx`, it becomes the whole-op scope that `FT_RM_REVALIDATE`
  was measured and refuted for (20/41 hangs, reader-visible key loss), and
  violates the seam rule (no node lock across a grace period).

⇒ **These two arms are a STRUCTURAL item, not a lock item.**

## ☑ THE LEAF ARM IS CONVERTED (2026-09-14)

`ft_detach_node`'s `freeze_leaf` now carries a LENGTH (`freeze_len`), and
`ft_hlist_freeze_chain_prepare` records one `{v -> MARK(v)}` edge per chain node
into the detach's own commit. `_cds_ft_remove_all_locked`'s leaf arm passes
`chain_head` + `ft_hlist_chain_len(chain_head)` and the post-detach
`ft_ra_sweep_held` call is GONE.

For `freeze_len == 1` the new primitive is byte-identical to
`ft_hlist_freeze_sole_prepare`, which is what keeps `cds_ft_remove` unchanged.

☠ **@freeze_len IS A BOUND, NOT A HINT.** The walk stops at the caller's
derivation so the derived tail is recorded against NULL — a duplicate appended
since tears that expected-old and ABORTS the whole detach, and the caller
re-derives. Re-walking to the real end instead would mark the fresh duplicate
into the tombstone and prune the branch around it: "key LOST after an OK
concurrent insert", the defect `ft_hlist_freeze_sole_prepare`'s header was
written for.

Measured, 9 rows (`inv_detach_cross_view`, `inv_remove_cross_view*`,
`inv_concurrent_remove_all_{nolist,list,prefix}`):

| row | UNHELD before | UNHELD after |
|---|---|---|
| `inv_remove_cross_view` | 92,668 | **0** |
| `inv_remove_cross_view_compressed` | 74,671 | **0** |
| `inv_remove_cross_view_prefix_siblings_all` | 20,000 | **0** |
| `inv_detach_cross_view` | 18,000 | **0** |
| `inv_remove_cross_view_compressed_parent` | 16,874 | **0** |
| `inv_concurrent_remove_all_nolist` | 14,151 | **0** |

☞ **COUNTED ON BOTH SIDES**, because a site that merely stops reporting proves
nothing: the writes MOVED. `inv_detach_cross_view` went from 18,000 unheld
sweeps to `ft_detach_node:4467` total 18,000 / 0 UNHELD with
`ft_hlist_freeze_chain_prepare` showing 18,000 — the same volume, now inside the
commit.

## Order of work

1. Force the three `_cds_ft_remove_all_locked` lone chain marks through a txn
   that owns the holder. This is class A -- the only part proven to install.
2. Settle class B: instrument the COMMIT, not the prepare, so "recorded unheld"
   and "installed unheld" stop being the same column. Then fix what remains.
3. `_cds_ft_insert_replace` needs NOTHING here -- it was a probe artifact.
4. Instrument the five never-reached sites and the six static-only shapes;
   add rows that reach them.
5. Only then convert the words to SW. The audit reading 0 is the gate.
