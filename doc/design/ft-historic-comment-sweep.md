# Sweep: remove historic narration from FT comments

Status: **PLANNED** (2026-07-31). Standalone commit, comment-only, no behaviour.

## The rule

A comment states what the code **is**. It does not narrate what the code, or a
previous comment, **used to be** — that is what `git log` and `git blame` are
for. Archaeology in a comment costs a reader attention on every future read and
ages badly: the "old" thing it contrasts against eventually means nothing to
anyone, while the sentence stays.

Where the history is genuinely load-bearing, keep the *conclusion* as a
present-tense invariant plus a rationale, not as a story:

```
BAD   ★ THEY USED TO COME FROM TWO.  ft_descent_step sets @d->nf from
      ft_node_get_nth_reanchor_slot's load; the descent then did a bare
      `*raw_ret = *d->nfp`, a SECOND, independent load of that same slot ...

GOOD  The pair MUST come from ONE load: @d->nf (the occupant the extract side
      re-roots) and @raw_ret (the forward publish's expected-old) both describe
      the graft-point slot, so two loads let a peer swap it in between and make
      them name different objects -- the commit then ratifies displacing one
      node while the extract side re-roots another.
```

The rationale survives; the archaeology goes.

## Scope

Two groups. Do them in ONE commit; both are comment-only.

### A. Narration added 2026-07-31 (13 sites)

Introduced while fixing the defects/doc-debt of that day. Each pairs a real
invariant with a "this used to say ..." preamble; keep the invariant, drop the
preamble.

| file | marker |
|---|---|
| `fractal-trie-internal.h` | `LOCK IS IN THE MASK UNCONDITIONALLY.  This used to say ...` |
| `fractal-trie-internal.h` | `Bit 19, unchanged by the §8.3 offset split ...` (check) |
| `ft-graft.h` | `★ THEY USED TO COME FROM TWO` |
| `ft-graft.h` | `★ THIS STATUS USED TO BE DROPPED` |
| `ft-graft.h` | `(the OPTIMISTIC build this also listed is gone)` |
| `ft-lifecycle.h` | `(This once also named the OPTIMISTIC strategy and a lock_mode field ...)` |
| `ft-mutation-helpers.h` | `★ This used to say merge_at "has NO retry loop ..."` |
| `ft-mutation-helpers.h` | `★ THIS STATUS USED TO BE DROPPED, on a comment that read ...` |
| `ft-mutation-helpers.h` | `@mtxn -- ... the field name predates the rcu-mcas -> rcu-txn rename` |
| `ft-remove.h` (x2) | `... ignoring it as this comment used to claim` |
| `include/urcu/fractal-trie.h` | `(The optimistic engine this sentence also weighed no longer exists.)` |
| `test_urcu_ft_inv.c` (x3) | `this used to say "the concurrent-writer path is not yet correct"`, `(This was once gated on FEATURE_FT_MW_LOCK_FINE_DROP ...)`, `(This used to say pinning existed for ...)` |
| `test_urcu_ft_unit.c` | the `NR_TESTS_DLM` "no longer a build mode" block |

Find them with:

```sh
grep -rniE 'used to (say|be|come|claim|read)|this once (also )?(named|described|listed|weighed)|it began as|was once gated|predates the' src/ include/ tests/
```

### B. Pre-existing historic comments

Not introduced on 2026-07-31; same treatment. Known ones:

* `fractal-trie-internal.h` — `WRITER LOCK.  It began as the copy fence (MW
  campaign ...)`. The origin story of `FT_STATE_LOCK`; after the
  COPYING -> LOCK rename it is the last trace of the old name, and that is
  precisely why it should go rather than be preserved.
* `fractal-trie-internal.h` — `LOCK is merely a copy fence here ...`
* `ft-lifecycle.h` — `->ordered_list gate here used to be for`
* `fractal-trie.c` — `That argument used to be carried by the d_src ...`
* `ft-merge.h` — `that carried the double-OOM leak -- and its rollback are gone`
* `ft-mutation-helpers.h` — `The two used to be conflated ...`
* `test_urcu_ft_unit.c` — `coherence used to be gated on ->ordered_list`

Widen with the same grep; treat each on its merits (a few may be describing a
current fallback rather than history — read before deleting).

## Method

1. Rewrite, do not delete blindly. For each site ask: *is there an invariant
   here that only exists in the historical clause?* If yes, restate it in the
   present tense first, then drop the clause.
2. Do NOT delete a live warning because it is phrased historically. Example
   from the rename: `parent_slot_offset shares the state word with the flip
   proxy` was factually wrong about the location but described a REAL hazard —
   the fix was to correct the location, not remove the guard.
3. Verify comment-only mechanically before committing:

   ```sh
   git diff -U0 src/ include/ tests/ | grep -E '^[+-]' | grep -vE '^[+-]{3}' \
     | grep -vE '^\s*[+-]\s*\*|^[+-]\s*/\*|^[+-]\s*$'
   ```
   Empty output means every changed line is a comment line.
4. `make all` + `ft_unit` / `ft_inv` smoke. A full gate is disproportionate for
   a provably comment-only change.

## Why not now

Folded into the COPYING -> LOCK rename it would have made a 599-line pure
substitution unreviewable. Same reason the rename itself was kept standalone.
