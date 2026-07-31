# FT configuration simplification

Motivation: the sibling split/compress orphan defect
(`project_ft_sibling_split_compress_orphan`) lives in a configuration quadrant
that is already slated for deprecation. Hardening that quadrant is wasted work;
making it **unrepresentable** removes the defect structurally and cuts the
matrix at the same time.

## The two axes today

**RUNTIME — writer strategy** (`enum cds_ft_writer_strategy`, 2 values):
* `CDS_FT_WRITER_LOCK_COARSE` — one FT-wide writer mutex.
* `CDS_FT_WRITER_LOCK_FINE` — **the documented default**; per-node lock sets.
* (order statistics coerces FINE -> COARSE, an effective third behaviour.)

`ft->lock_fine` is set in exactly one place, `ft-lifecycle.h:819`, from the
strategy alone. **Nothing about DLM feeds it.**

**COMPILE TIME — MW implementation**: `FEATURE_FT_MW_DLM_ACQUIRE` (87
references) is **opt-in and does NOT self-enable**, unlike
`FEATURE_FT_MW_LOCK_FINE_DROP` (21 refs, self-enables at
fractal-trie-internal.h:137).

## The problem: a 2x2 where one quadrant is both default and doomed

| | no DLM (default build) | DLM |
|---|---|---|
| **COARSE** | FT-wide mutex | DLM code inert (`lock_fine` false) |
| **FINE** (default strategy) | ★ **per-slot-CAS MW — deprecated, and where the defect lives** | the intended end state |

The **default build with the default strategy** lands in the starred quadrant.
Seven of nine gate configs are in it; only `dlm` and `dlm-fault` are not.

Concretely, that quadrant is missing the §9.2 orphan-chain plan-lock: the
collection-time `ft_meta_copying_mark`, the fenced
`{COPYING|s -> TOMBSTONE|s}` retire terminal, and the release sweep are all
bracketed in `#ifdef FEATURE_FT_MW_DLM_ACQUIRE` *around code that is already
runtime-gated on `ft->lock_fine`* — and they use only base F2 fence primitives,
no DLM acquire sets. `ft_detach_freeze_orphans` (ft-remove.h:497/509) therefore
falls back to the plain RYW `ft_flip_txn_record_tombstone`, whose expected-old
matches by construction and so **cannot** detect a peer that grew the orphan.
That is the measured orphaning.

## The plan

**Step 0 (prereq, already scoped).** Pay down the DLM-as-default test debt:
55 unit failures + 1 abort, all cross-trie-live-src `BUSY` — tests asserting the
PRE-DLM contract, which DLM legitimately refuses. See
`project_ft_dlm_default_phase1a`. This is test rewriting, not library work.

**Step 1. Make DLM unconditional.** Delete `FEATURE_FT_MW_DLM_ACQUIRE` as a
switch: keep the DLM arms, delete the `#else` per-slot-CAS arms. MW semantics
become ONE implementation, selected at runtime by `ft->lock_fine`. This deletes
the broken path rather than fixing it.

**Step 2. Delete the lock-fine drop/keep pair.** `FEATURE_FT_MW_LOCK_FINE_DROP`
self-enables and has been the shipping behaviour since @939b36da;
`FEATURE_FT_MW_LOCK_FINE_KEEP` has ONE reference. Neither has a gate config, so
the opt-out is untested by construction.

**Step 3. Collapse the gate 9 -> 7.** `dlm` folds into `default`, `dlm-fault`
into `fault-audit`. Every remaining config then exercises the one MW
implementation, so a failure no longer has to be attributed to a mode first.

**Step 4. Decide the uncovered flags.** No gate config sets these, so their
non-default state is untested by construction — each is either deletable or
needs coverage:
`NO_FEATURE_FT_MERGE`, `NO_FEATURE_FT_KEY_MAP`,
`NO_FEATURE_FT_INSERT_IN_PLACE`, `FEATURE_FT_EXCL_VALIDATE` (12 refs),
`FEATURE_FT_ORD_CELL`.

Keep, with real coverage: `FEATURE_FT_COMPRESS`, `FEATURE_FT_SKIP_COMPRESSED`
(perf features, both have `noskip`/`nocompress` gate configs),
`FEATURE_FT_FAULT_INJECT`, `FEATURE_FT_VERIFY_AT_MUTATION`,
`FT_DEBUG_TOMBSTONE_AUDIT` (test-only).

## What this does and does NOT fix

DOES: removes the quadrant in which the orphan-chain coherence is absent, and
removes mode-attribution ambiguity from every future failure.

DOES NOT: the sibling oracle `inv_sibling_split_compress` **also fails in the
`dlm` build**, so DLM carries a residual hole of its own. Simplification closes
one of two holes. Chase the residual AFTER the cutover, when there is only one
mode for it to be in.

## Validation

`inv_sibling_split_compress` (opt-in `FT_INV_SIBP=1`, hangs today) is the
acceptance test for the residual. It must go into the gate unconditionally once
it neither hangs nor loses keys.
