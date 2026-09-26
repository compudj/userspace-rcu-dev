# FT: remaining MW, validate and load-validate records (audit, 2026-09-26 @ `76801d6d`)

Mathieu (2026-09-26): *"A validate guard is a remain of the MW CAS era. The new
pattern is to validate or revalidate under lock. We should audit FT for
remaining MW or validate or load validate records. Those are unconverted
sites."*

This is that audit: every record that still installs by CAS (MW) or validates
`{v -> v}`, with how often it fires on a FINE shared trie under concurrent
writers, and what converting it means. Companion to the plan of record,
`mw-to-fine-locking-remainder.md` (its G4 is the MW_ALWAYS lane question).

## Method

- **Static:** every call of `ft_flip_txn_record_tag_mw[_pinned]`,
  `ft_flip_txn_record_state_mw`, `ft_flip_txn_lock_or_guard_parent*`,
  `ft_flip_txn_guard_parent*`, `ft_flip_txn_guard_installed_child`,
  `ft_flip_txn_rebase_state_guard`, `urcu_txn_validate`,
  `urcu_txn_load_validate*`, `urcu_txn_read`, `ft_acq_guard_add`.
- **Dynamic:** two census trees, both `--enable-rcu-debug`,
  `-fno-omit-frame-pointer`, all three lock spacings compiled in:
  - `-DFT_DEBUG_TXN_KIND -DFT_DEBUG_STRUCT_ANCHOR`: per-class MW split, MW_STRUCT
    by record pc, and the owner question for every always-MW lane record.
    Per-node and root-only only. At exponential, `ft_sa_check` SEGVs climbing
    the unpublished S_top' copy that the build's absorb acquires (instrument
    gap, not a product crash).
  - `-DFT_DEBUG_TXN_KIND -DFT_DEBUG_GUARD_AUDIT`: every §4.B parent-guard plant,
    whether it is on the miss path, and whether the op's frame covers the word.
    All three spacings.
- **Workload:** `test_urcu_ft_inv` with `FT_INV_MW=1` (concurrent writers, fine
  shared tries), one full run per spacing. The numbers are records per run.
  ft_unit is mostly exclusive or coarse tries, where MW parks through door 1,
  so it is not the conversion signal.

Counts: **pn** = per-node, **ro** = root-only, **ex** = exponential.

## A. Validate records `{v -> v}`

| Record | Sites | pn | ro | ex | What it is today | Conversion |
|---|---|---:|---:|---:|---|---|
| §4.B parent guard (`urcu_txn_validate` in `ft_flip_txn_guard_parent_ctx`), total planted | ~25 `lock_or_guard_parent*` expansions; direct: ft-graft.h:1426, ft-insert.h:904 / 2796 / 2940 / 7048 / 7181, ft-remove.h:6241 | 19.68M | 27.90M | 20.52M | | |
| ↳ planted on the **miss path** (commit discards, never installs) | `lock_or_guard_parent_ex` after a failed acquire | 18.59M | 23.38M | 18.51M | dead weight: "keeps the record shape identical" | **Drop the plant.** Cost only, no semantics. |
| ↳ installed on a fine trie, word **held through the op's frame** (not the registry) | coarsened shared dedupe, ctx-covered | 0 | 0.33M | 0.03M | redundant: the held lock is the exclusion | **Drop** once the "hold cancels the guard" question consults the frame / anchor, not just the registry. |
| ↳ installed, **not fine** (coarse or exclusive trie, `t == NULL`) | the `guard:` label's NOT_FINE exit | 0.88M | 1.04M | 0.94M | redundant under the FT-wide lock or the exclusive contract | **Drop.** |
| ↳ installed on a fine trie, truly unheld | — | 0 | 0 | 0 | none left | — |
| Re-parent sweep's child `{live -> live}` (class STATE) | `ft_reparent_record_meta` (`check_child` arm) | 214 | 20k | n/a¹ | 3e8c7d52 skips it when `ft_flip_txn_holds()`, but that answers exact word / member / covered only. At coarse spacings the child is excluded by its **anchor**, which it never asks. | Ask anchor coverage (root-only: root lock held ⇒ covered). Otherwise **take the child's lock** and drop the validate. |
| GUARD: installed value node `{live -> live}` | `ft_flip_txn_guard_installed_child` ← `ft_ord_cell_record_into_ft` | 0 | 52k | 0 | root-only only | Same question as STATE: covered by the anchor ⇒ drop. |
| Acquire-time back-edge guards | `ft_acq_guard_add` ×2 in `ft_dlm_acquire_set_at` | — | — | — | validated by the acquire commit itself, atomically with the lock take | **Conforming** (validate at the lock instant). |
| `rebase_state_guard` | `record_tombstone_locked_ctx`, `record_release_lock` | — | — | — | re-bases a guard our own acquire staled onto the held value (SW when armed) | **Conforming** (this is the conversion). |
| `urcu_txn_load_validate*` | — | 0 | 0 | 0 | no FT caller | — |
| `urcu_txn_read` | 7 (`ft_nr_keys_load`, `ft_meta_nr_child_load`, `ft_meta_parent_slot_offset_load`, `ft_acq_guards_ok`, `ft_node_recompact`, 2 trace) | — | — | — | a stable **load** (bounded wait on an undecided owner), not a record | Out of scope. |

¹ The per-producer split comes from the STRUCT_ANCHOR census, which does not
run at exponential. The class STATE total there is 2.92M, mixing all three
STATE producers.

## B. MW stores that stay MW by class (MW_ALWAYS)

The class STATE is **not** mostly the validate above. Split by record pc, it is
three producers: the validate (A), and the two real writes below.

| Class | Producer | pn | ro | ex | Why still MW | Conversion |
|---|---|---:|---:|---:|---|---|
| STATE: anchored retire, MW arm | `ft_flip_txn_record_retire_anchored_arms` (`sw_ok=0` when anchor ≠ node and `!node_held`), `{s -> s\|TOMBSTONE}` | 0 | **4.91M** | in 2.92M¹ | Deliberate: holding the anchor does not exclude a peer that dates the node on a second path (graft / merge move), and the MW expected-old is that defence. Unreachable at per-node. | At root-only every date resolves to the root, so the defence is moot and the arm can park SW. At exponential it needs the node's own word, or anchors that do not move with the node. |
| STATE: `nr_child++` fallback | `ft_flip_txn_record_nr_child_inc` → `record_state_mw` when the exact word is not owned (6a2c2ae2's rule), from `ft_attach_node` (ft-insert.h:2799) | 239k | 251k | in 2.92M¹ | A real write on a word the op does not hold exactly. It follows `lock_or_guard_parent`, whose miss path is 18.6M at per-node, so part of it may sit in discarded txns (no commit verdict is collected for always-MW classes). | Take the exact word (per-node: it is the parent the insert locks), or count under the lock. |
| CELL | ordered-cell edges with no cell owner: `ft_ord_cell_flip_into`, `ft_ord_cell_record_into_ft` | 667k | 702k | 783k | the G4 lane | List BEGIN/END locks (`ft-cell-list-locking-scheme-PROPOSAL.md`, @11a9956e). |
| ROOT | `ft_flip_txn_record_root` (`record_tag_mw_pinned`) | 548k | 619k | 651k | `&ft->root` CAS | Park SW under `ft->root_lock` (the dedicated root lock of the 100%-SW endgame). |
| NR_CHILD_DEC | `ft_remove_one_commit` (ft-mutation-helpers.h:~19904), in-place delete's `nr_child--` | 366k | 373k | 828k | holder not in the registry | Take the holder's word. 6a2c2ae2 did the `inc` side. |
| HEAD_BACK | `ft_flip_txn_record_head_back_edge_owned`, owner not held | 453k | 39k | 242k | holder-owned back edge, lock-set reach | Uncovered at per-node: `ft_detach_node` (ft-remove.h:5964) 177k, of which 129k commit without the owner; `ft_detach_node_replace_compressed_parent` (:367 / :373) 88k + 37k, nearly all committed without it. Lock the holder before recording. |
| DUAL_UNNAMED | `ft_flip_txn_record_pub_rec`, `ft_ord_cell_record_into_ft` (`!owner_held`, owner NULL) | 267k | 145k | 221k | SKIP_X dual's owner not plumbed | Plumbing: name the grandparent. |
| DUAL_NAMED | same, owner named but not held | 4.1k | 1 | 24k | the conversion surface proper | Acquire the grandparent (the SKIP_X dual lock). |
| RANK | `ft_flip_txn_record_count_parent` | 24k | 25k | 25k | `nr_keys` up unlocked ancestors | Phase E. |
| PARENT_WORD | `ft_flip_txn_record_parent_word` (child not held) | 129 | 8.3k | 129 | re-parent sweep, lock-set reach | Per-node: 65 uncovered, 18 taken late. |
| PSO | `parent_slot_offset` | 64 | 725 | 64 | | §8.3 layout split retires it. |

## C. SW-eligible records the per-record gate refused (MW_STRUCT)

Totals: **pn 2.14M / ro 431k / ex 1.51M**, OWN_HELD = 0 everywhere. By record pc (per-node):

| Record site | pn | Why MW | Conversion |
|---|---:|---|---|
| `ft_flip_txn_record_retire_anchored_arms` → `record_tombstone_locked_ctx` (txn sites ft-remove.h:5430 / 1661 / 4439) | 1.61M | The op holds the fence, but in its frame (fenced retire), not the registry. The gate asks the registry only. STRUCT_ANCHOR witness at this pc: 5.53M records, 4.50M HELD + 0.86M WLOCK + 0.15M coarse, **0 uncovered**. | Register the fence, or let the gate ask the frame. The record is `{LOCK\|s -> TOMBSTONE\|s}` on a word the op holds. |
| `ft_ord_cell_flip_into` (ft-mutation-helpers.h:18610; txn sites ft-insert.h:6998 / 7136) | 384k | Cell owner not in the registry. Witness at this pc: 9.70M records, 8.62M HELD, 4k on the miss path, **0 uncovered**. | With the CELL lane (B). |
| OWN_RETIRE (a node tombstoning itself) | 77k / 96k / 72k | unregisterable by construction | Not an obligation. |

## Status (2026-09-26, after the first two conversions)

- **1 ☑ the §4.B parent guard.** No plant on a missed txn; none under
  trie-wide exclusion (`ft_flip_txn_excludes_all`: door 1, or the FT-wide
  lock this thread holds); and `cds_ft_replace`'s `{L}` counts as covering at
  root-only when it was taken for the parent (`ft_replace_hold_covers`).
  Installed parent validates per ft_inv MW run: **per-node 0, root-only 0**,
  exponential ~18-30k (the replace arm, kept: see below).
- **2 ☑ re-parent and installed-child validates** at root-only
  (`ft_flip_txn_owns`' root arm) and under trie-wide exclusion: 20k and 52k to
  0. **Per-node keeps both**, deliberately: no lock in `{C,P,(GP)}` reaches a
  child or an installed node, and these validates are the defence against a
  node retired while still linked (measured through the installed-child
  guard). The alternative, locking the sweep's children, was implemented and
  rejected (escalation cannot rescue a contended child). Converting them needs
  that residue proven gone first.
- **Exponential keeps its validates.** A plan dated before a move can lock the
  old path's anchor while a peer locks the new one. The validate is that gap's
  only defence until a revalidation under the lock replaces it.
- **Evidence the dropped validates never decided a commit:** with knobs
  restoring every one of them (`FT_DEBUG_GUARD_ON_MISS`,
  `FT_DEBUG_GUARD_WHEN_EXCLUSIVE`, `FT_DEBUG_REPLACE_GUARD_ROOT_ONLY`,
  `FT_DEBUG_VALIDATE_WHEN_COVERED`) and the engine's abort attribution
  (`-DFT_DEBUG_TXN_KIND -DURCU_TXN_REC_DBG`), 24M plants and 62k
  installed-child guards reached the engine at root-only, and every aborted
  commit lost on an ordered-cell record. No validate loss, and the "record that
  cannot lose, lost" alarm read 0.
- Landed as `05bc8aa4` (parent guard) and `e6f893e0` (re-parent and
  installed-child validates). Gates: gate18 18/18, gate-mw 9/9, full parallel
  gate 117/119. Both reds are pre-existing, attributed by a matched A/B against
  `76801d6d` (see the last two side findings).

## Suggested order (as audited)

1. **The §4.B parent guard.** Drop the miss-path plant; make "a hold cancels the
   guard" ask the frame / anchor. That removes every validate this family still
   installs on a fine trie. The not-fine plants go with it. *(Done: frame holds
   were NOT trusted in general, since a frame's hold can close before this
   commit. Only `{L}`, which provably outlives it, was.)*
2. **The remaining validates** (re-parent `{live -> live}`, installed-child
   GUARD): an anchor-coverage question in `ft_reparent_record_meta` and
   `ft_flip_txn_guard_installed_child` (20k + 52k at root-only). Where the child
   is not covered, take its lock (lock-before-write) instead of validating.
   *(Done for root-only and trie-wide exclusion; per-node kept, see Status.)*
3. **The anchored retire's MW arm** at root-only (4.9M): SW under the root
   anchor. Exponential stays MW until anchors stop moving with the node.
4. **Registry gaps** (C): the anchored-retire fence and the in-place delete's
   `nr_child--` holder (B, NR_CHILD_DEC). Same shape: a word the op holds or
   should hold, answered from the registry only.
5. **HEAD_BACK** in the detach paths: lock the holder before recording.
6. **Design items:** ROOT under `root_lock`, CELL under the list BEGIN/END locks,
   DUAL plumbing, RANK (Phase E).

## Side findings (not part of the conversion)

- Every rcu-debug ft_unit leg prints 5-8 `FT LOCK UNDERFLOW` reports. They also
  appear in gates before today's commits, and in **none** of 424 isolated
  per-test runs, so they need the suite's sequence. Suspect the address-keyed
  take/release detector across heap `struct cds_ft` reuse (`root_lock` is not
  reset on free; `FT_NR_KEYS_PROXY_TAG == FT_STATE_PROXY` also passes its
  state-word filter). Unconfirmed.
- At exponential, the STRUCT_ANCHOR census reports op-vs-trie anchor mismatches
  at `ft_merge_lock_overlap:523` and `ft_glue_acquire_reparent_marks:24909`
  (exponential is experimental).
- Pre-existing `_cds_ft_remove_all_locked: Assertion
  ft_meta_nr_child(holder_meta) > 0` (ft-remove.h:~10664), about 0.2-0.4% per
  run of `inv_prefix_shape_zoo` / `inv_owned_prefix_dense_remove_all`. A/B
  against 3f668cdc: same rate. It fired again in the gate after `e6f893e0`
  (proxyassert, exponential, list on).
- The `rmcap` config's remove retry cap (`FT_REMOVE_RETRY_CAP` 50000)
  aborts `inv_prefix_dup_promote_vs_extension` (list off, per-node) under
  preemption: each run pinned to 2 CPUs, `76801d6d` 220/240 and `e6f893e0`
  227/240; unpinned, 0/480 each. Pre-existing remove starvation, not a
  conversion regression.
