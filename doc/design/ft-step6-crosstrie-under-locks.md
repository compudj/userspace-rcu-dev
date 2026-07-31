# Step 6: cross-trie ops under locks — exclusive-source contract

Status: **6A DONE (@71741029); 6B DONE (@fcd93a5c, 2026-07-15) — graft + merge_at
+ graft_swap all landed.** Implements `mw-writer-lock-escalation-model.md`
§11.3 step 6. Extends the lock pivot ([[project_ft_mw_lock_escalation_pivot]]) and
unblocks the whole-trie MCAS→sw migration
([[project_ft_mcas_to_sw_migration_scope]]).

> **Pivot note (2026-07-15).** An earlier form of 6B (@b7d01b60) implemented a
> "decision-C" route: on a both-LIVE cross-trie op, detach src into an exclusive
> `B`, reinvoke `op(dst, B)`, and on a phase-2 reject **return** `B` to the caller
> through a new `cds_ft_graft **residual_ft` out-param (reattach was abandoned as
> racy — src's lock is dropped between phases, so a concurrent writer can
> repopulate it). Mathieu then called the **simplification** that supersedes all
> of it: *require the consumed source to be exclusive.* That removes the whole
> two-phase machinery — the fused body just runs directly on the exclusive source
> and is build-invisible, so a failure leaves the source pristine. The decision-C
> route, the `residual_ft` API, and the reattach were all removed; only the
> enabling primitive (exclusive-skip bracketing) and the guard narrowing were
> kept. This note describes the landed exclusive-source design.

## The contract

A cross-trie `cds_ft_graft` / `cds_ft_merge_at` / `cds_ft_graft_swap` **consumes
its source** (graft/merge `src_ft`, graft_swap `swap_ft`). Under
`CDS_FT_WRITER_LOCK_FINE` the source **must be EXCLUSIVE** — no concurrent readers
or writers:

- an exclusive source is private, so its writer scope **skips** its FT-wide lock
  (see mechanism below) — only `dst`'s lock is ever held: one lock, no cross-trie
  deadlock;
- a **LIVE** (`lock_mode && !exclusive`) source is rejected with
  **`CDS_FT_STATUS_BUSY_ERROR`** at the op entry, **before any lock, allocation or
  mutation** — both tries are byte-for-byte unchanged. The caller makes the source
  exclusive first with `cds_ft_make_exclusive()` (the zone-graft "build private,
  graft" pattern already does this);
- `dst` may be a **live concurrent** trie — that is the supported case;
- the gate is `src != dst && src->lock_mode && !src->exclusive`; a **same-trie**
  merge rekey (`src == dst`) is excluded (it takes one FT-wide lock reentrantly);
- **inert outside lock-mode** — optimistic groups have `lock_mode == false` and
  never take an FT-wide lock for a cross-trie op, so the gate never fires.

Why no residual / undo is needed: with an exclusive source the **existing fused
body runs directly** and is **build-invisible** — every fallible step (node/txn
allocation, POPULATED/OVERFLOW check) happens *before* the point of no return (the
source-root unlink / dst-root swap), and each such exit frees what it reserved. So
a failed graft/merge/graft_swap (`POPULATED_ERROR` / `OVERFLOW_ERROR` /
`MEMORY_ERROR`) leaves the source **pristine** (full, unchanged). Nothing is lost
or stranded; there is nothing to hand back.

## The mechanism: exclusive tries skip the FT-wide lock

Two facts compose (fractal-trie-internal.h):

1. **An exclusive trie is private / single-writer by contract** — no concurrent
   writers, no concurrent readers until `cds_ft_make_concurrent`. So it needs no
   FT-wide writer lock at all.
2. `ft_writer_lock_scope_enter/_exit` **skip** the FT-wide lock when
   `ft->exclusive`. Then a cross-trie op with a live `dst` and an exclusive source
   holds `SCOPED_WRITER(dst)` [live → locks] + `SCOPED_WRITER(src)` [exclusive →
   **skips**] = **one FT-wide lock held**, no deadlock, and the existing fused body
   runs unchanged (it already skips the src drain / `synchronize_rcu` for an
   exclusive source).

`gp_wait` needs no change: it keys the lock drop off the TLS `ft_wlock_held` (the
live lock actually held), never the exclusive trie.

**HAZARD — the `exclusive` flag must not unbalance the lock if it flips inside a
scope.** `graft_swap` sets `swap_ft->exclusive` to `dst`'s mode mid-op (mode
inheritance), and `cds_ft_make_exclusive`/`_make_concurrent` flip the flag inside
their own writer scope. Resolved structurally: the exclusive-skip is checked
**after** the reentrancy test, and `_exit` releases off `ft_wlock_held`
**identity**, never a re-read of `ft->exclusive`. So a flag flip between enter and
exit cannot unbalance the TLS depth — `make_exclusive` took the lock at enter
(flag was false) and releases it at exit (`held == ft`); `make_concurrent` /
graft_swap's inheritance skipped it at enter (flag was true) and skips it at exit
(`held != ft`).

## The guard — a defense-in-depth assert

`ft_crosstrie_lock_mode_guard(a, b)` fires (aborts) only when **both** tries are
LIVE (non-exclusive) lock-mode and `a != b`:

```
if (a != b && (a->lock_mode && !a->exclusive) && (b->lock_mode && !b->exclusive))
        abort();
```

With the BUSY gate in front of every op entry, the source is always exclusive (or
not-lock-mode) by the time the guard runs, so it **cannot fire on a valid op**. It
stays as an assert that a both-live pair reaching the fused body means a BUSY gate
was missed — a bug — and aborts rather than deadlocking.

## Landed pieces (@fcd93a5c, over 6A @71741029)

- **6A** (@71741029): the five attach-side guard→RELEASE-lock conversions.
- Exclusive-skip bracketing primitive (`ft_writer_lock_scope_enter/_exit`).
- `ft_crosstrie_lock_mode_guard` narrowed to fire only on a both-LIVE pair.
- BUSY gate at all three entries: `ft_graft_keylen`, `cds_ft_graft_swap`,
  `ft_merge_at_inner`. `cds_ft_graft` reverts to its 4-argument form (no
  `residual_ft`).
- Internal callers need no gate of their own: `cds_ft_graft`→`ft_graft_keylen`,
  merge's whole-source fallback→`ft_graft_keylen`, and the graft_swap
  DELEGATE→`cds_ft_graft` all re-enter the same BUSY gate; the merge spine-copy
  sits behind merge's entry gate.

**★ Same-trie safety (skeptic-confirmed).** merge's same-trie rekey recursion
passes a **detach product** as its source, and `ft_detach_keylen` forces
`detached->exclusive = true` (ft-detach.h:79/312). So the new BUSY gate passes it
through and the recursion's `assert(status == OK)` is not tripped. That was the
one existing path the gate could have broken; it does not.

**Validated:** unit default/audit/noskip/nocompress 281, fault 325, ASAN full 281
(0 leak/UAF — pins the no-leak on BUSY/reject paths); inv 61 including
`FT_INV_MW=1` (`inv_concurrent_writers_fine_lock` + `inv_ordered_no_escape_graft`
/ `inv_merge_no_escape` / `inv_rekey_no_escape` — exclusive-detach-product grafts
succeed under LOCK_FINE); gate 6/7 (in-place = pre-existing compaction flake, not
a regression). Adversarial skeptic `refuted=false` on all six axes (gate
pre-mutation/pre-alloc, same-trie safety, build-invisible on both root-swap and
non-root shapes, graft_swap mode-flip lock balance, guard unreachable on valid
ops, leak-free). Tests: `test_writer_lock_mode_fine_graft` (exclusive happy path +
POPULATED reject leaves source pristine), `test_writer_lock_mode_fine_crosstrie_busy`
(live src → BUSY for all three ops, then OK after `make_exclusive`),
`test_fine_lock_acquire_fault` graft phase (exclusive source, survives an
acquire miss).

## Next

Step 6 (cross-trie ops under LOCK_FINE) is complete. Next milestone: the
**per-domain FT-wide-lock drop** — LOCK_FINE still takes the FT-wide lock in
addition to the per-node lock-set; dropping it per op-domain so LOCK_FINE relies
solely on the per-node lock-set is the gate for the whole-trie MCAS→sw REPLACE
migration ([[project_ft_mcas_to_sw_migration_scope]]).
