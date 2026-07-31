# Stage 4b Exact-Mode Descent — Design Options

**Status**: design proposal, not implemented. Blocks re-enabling the
chain-merge canonicalization site in `ft_detach_node` (~line 11162 in
`src/fractal-trie.c`).

## 1. Problem statement

Stage 4b migrates an internal child X of a compressed node `cn` into a
freshly-allocated skip-target node S whose body uses a different
layout:

```
                              skip-target body S
  byte 0    : type-class header
  ...
  byte B-1  : end of bitmap header
  byte B    : subkey_len           (== cn->len)
  byte B+1  : cached_subkey_len    (≤ subkey_len)
  byte B+2..: cached subkey (cn->key_bytes prefix)
  byte ...  : free space
  end       : pointer table at FT_SKIP_SLOT(s_node, alloc_size, i)
              indexed from the END of the allocation
```

After migration:
- `cn->child = s_internal_flag` (bare internal flag, no `FT_SKIP_MASK`).
- Cn's parent slot is overwritten by the outer mutator to
  `s_internal_flag | FT_SKIP_MASK`, encoding the skip-target for
  cand-mode descent (which bypasses cn entirely).

Cand-mode lookup works: the outer loop sees the skip-mask in the parent
slot, dispatches `ft_node_get_nth_skip` directly on S, and the
skip-aware scanner reads the pointer table at the correct offset.

**Exact-mode lookup is broken** for the migrated case. Path:

1. Outer loop receives `node_flag = parent_slot = (FT_SKIP_MASK |
   s_internal_flag)`.
2. `ft_node_skip_compressed(node_flag) → true`. Non-cand branch:
   `node_flag = ft_compressed_node_flag(ft_skip_to_compressed(node_flag))`
   — recovers `cn` via S's `meta->parent`.
3. `ft_lookup_compressed(cn)` runs: verifies `cn->key_bytes` against
   the current key window, advances `i += cn->len`, then loads
   `node_flag = ft_dereference_acquire_prefetch(cn->child)` at line
   7040. Returns `cn->child = s_internal_flag` (bare).
4. Outer loop next iteration: `ft_node_skip_compressed(s_internal_flag)`
   is `false` (FT_SKIP_MASK not set on bare cn->child). Falls through
   to `ft_node_get_nth(s_internal_flag, ...)` — the **non-skip** scanner.
5. Non-skip scanner reads pointers from `node + B`, but S's pointers
   are at `alloc_size - (i+1)*8`. Wrong offset → `NOT_FOUND` or worse.

Setting `FT_SKIP_MASK` on `cn->child` to route the dispatcher to the
skip-aware scanner produces a different failure:

3'. `node_flag = (FT_SKIP_MASK | s_internal_flag)` from cn->child.
4'. Outer loop next iteration: `ft_node_skip_compressed(node_flag) →
    true`. Non-cand branch: `ft_skip_to_compressed(node_flag)` reads
    S's `meta->parent = cn` → returns **cn again**.
5'. Loop re-enters `ft_lookup_compressed(cn)`, but `i` has already
    advanced past cn's path. Either re-compares wrong bytes
    (silent NOT_FOUND) or detects `cn->len > remaining_key` and
    bails. Infinite-loop avoided by index, but exact-mode lookup
    is wrong.

## 2. Why this is hard

The non-cand skip-handling branch (`ft_lookup_compressed` at
`fractal-trie.c:7251-7278`) was designed for the **parent-slot**
skip pointer — where the skip pointer's job is to skip past `cn`
entirely. The branch recovers `cn`, hands control to
`ft_lookup_compressed` for full key matching.

That branch fires identically when `node_flag` came from
`cn->child` after migration — the bit pattern is the same
(`FT_SKIP_MASK | s_internal_flag`), the back-pointer chain
recovers the same `cn`. The loop has no way to distinguish "I'm
descending into cn from cn's parent (must verify cn->key_bytes)"
from "I've already done cn's key match, this is the post-cn
child" without external context.

## 3. Design options

### Option A — Special-case in `ft_lookup_compressed` (smallest)

Keep `cn->child = s_internal_flag` (no skip-mask). In
`ft_lookup_compressed`, after the line-7040 load, detect that
cn->child is a skip-target body and dispatch directly into S's
pointer table for the next key byte. Return the next-level node
flag so the outer loop continues normally.

```c
node_flag = ft_dereference_acquire_prefetch(cn->child);
if (skip_compressed && ft_node_skip_internal_body(node_flag)) {
    /* cn's key_bytes are already verified; cached subkey in S
     * is redundant in exact mode. Consume one key byte and
     * step through S's skip-aware pointer table.
     */
    iter_key = *(key + 1);
    advance_to_skip_target_child(node_flag, iter_key,
        &node_flag, ...);
    /* Optional: also handle FT_DESCENT_END for terminal cases. */
}
```

**Pros**:
- No outer-loop changes. No new return codes.
- Touches one function plus a new helper.

**Cons**:
- Duplicates the dispatch logic that the outer loop also runs
  (one extra key-byte step + scanner call). Easy to drift out of
  sync as the loop body evolves.
- Need `ft_node_skip_internal_body()` predicate — a way to tell
  "the pointed-at node has skip-target body layout" from the
  bare pointer alone. Not currently available; would need a
  type-table bit or a per-node header peek.
- `cn->child` carries layout information *not encoded in its
  bits* (skip-target-ness lives in `meta` or has to be
  reconstructed via cn's parent slot). Violates the slot-tag
  invariant from the QP-skip-phase2 design doc ("variant
  recovered exclusively from the SLOT TAG").

### Option B — Skip-mask on `cn->child` + unify the skip branch

Set `FT_SKIP_MASK` on `cn->child` so its layout is communicated
through the pointer's low bits, matching the parent-slot
encoding (slot-tag-driven, per QP-skip-phase2 §2). Then teach the
outer loop's non-cand skip-handling branch to **not** re-enter
`ft_lookup_compressed` on a skip pointer it just emerged from.

Mechanics: pass a `just_emerged_from_compressed` flag to the
outer loop (or thread it through the return value of
`ft_lookup_compressed`). When set, the next-iter skip-handling
branch unifies with cand-mode behavior: strip the skip-mask via
`ft_skip_child_ptr` (preserves the bit for internal targets per
existing semantics), advance, dispatch via the skip-aware
scanner. No cn re-entry.

```c
enum ft_descent_action ft_lookup_compressed(..., bool *child_is_skip) {
    ...
    node_flag = ft_dereference_acquire_prefetch(cn->child);
    if (skip_compressed && ft_node_internal(node_flag) &&
        ft_node_skip_target(node_flag)) {
        *child_is_skip = true;
    }
    ...
}

/* In the outer loop */
if (skip_compressed && ft_node_skip_compressed(node_flag)) {
    if (!descend_cand && !just_emerged_from_compressed) {
        node_flag = ft_compressed_node_flag(
            ft_skip_to_compressed(node_flag));
    } else {
        /* cand-mode-like stripping: advance past skip_len and
         * keep the bit for internal-target dispatch. */
        ...
    }
}
```

**Pros**:
- Layout info stays in the pointer's low bits (slot-tag
  invariant preserved).
- Outer loop has one unified skip-handling code path; the
  `just_emerged` flag is a small local addition.
- Migration helper sets `cn->child = (FT_SKIP_MASK |
  s_internal_flag)` — no separate "bare" and "tagged" forms to
  keep straight.

**Cons**:
- Outer loop API grows by one bit of state.
- Touches both `ft_lookup_compressed` (sets flag) and the outer
  loop (consumes flag). Reviewers need to check the flag isn't
  stale across iterations.
- Need to audit every reader-side site that loads `cn->child`
  and dispatches it directly (e.g., `ft_traverse_compressed` at
  line 7100). Each such site needs the same `just_emerged`
  treatment.

### Option C — Slot-tag-driven variant dispatch (largest)

Adopt the full QP-skip-phase2 model for POPCOUNT: each
internal-class type gets two type-table entries (direct +
skip), keyed by a wider slot tag. The dispatcher decides
skip-vs-direct from the **tag in the pointer**, not from
FT_SKIP_MASK overlaid on a single type-tag.

`cn->child` for a migrated S carries `FT_KIND_SKIP_POPCOUNT_N`
(say type-tag 0x10 | FT_INTERNAL_MASK). The outer loop's
`ft_node_get_nth` reads the tag, routes to the skip-aware
scanner. No `FT_SKIP_MASK` bit involved on `cn->child`.

**Pros**:
- Clean, no overload of a single bit between "skip-encoded
  pointer" and "skip-target body layout".
- Matches the QP-skip-phase2 direction (consistent across QP
  and POPCOUNT).
- No outer-loop flag plumbing.

**Cons**:
- Needs spare bits in the type-tag encoding. Per existing
  memory `project_ft_5bit_encoding_ambiguity.md`, the current
  encoding is already crowded. Adding skip variants of every
  POPCOUNT type pushes us back into the ambiguity issue.
- Touches every dispatch site: bitmap scanners,
  `ft_node_get_nth`, `ft_node_set_nth`, recompact reparenting,
  parent-walk consumers, verify, show_stats. Multi-week effort,
  not a single commit.
- Need to coordinate with the planned 5-bit encoding refactor
  rather than fighting it.

## 4. Recommendation

**Option B**, as the smallest design-clean change that
preserves the slot-tag invariant and unblocks Stage 4b without
waiting for the 5-bit encoding refactor. The
`just_emerged_from_compressed` flag is local to the lookup loop
and doesn't leak into write-side code paths.

`ft_traverse_compressed` (used by replace / count_keys_prefix at
`fractal-trie.c:7100`) needs the same treatment — verify it
isn't called in a context where it would observe a migrated
`cn->child`. Audit step before implementation.

Option A is tempting for size but introduces a "layout known
only via predicate, not pointer tag" mode that complicates
every future reader. Option C is the right long-term shape but
needs the encoding refactor first.

## 5. Validation strategy

1. Re-enable site 11162 (revert the `NULL` passed to
   `ft_publish_compressed` back to `&migrated_x` plumbing). Run
   `test_skip_target_migration` (#151) under DEBUG_COUNTERS,
   verify forward-migration count > 0 and lookup still passes.
2. Smoke: 191/191 unit + 12/12 inv across build-bench,
   build-asserts, build-debug-counters.
3. Promote the `diag()` lines in
   `test_skip_target_migration` to `ok()` assertions on
   migration count > 0.
4. Add an inv-level invariant test that mixes
   parent-slot-skip and migrated-cn-child cases under
   concurrent reads (extends an existing skip-mode inv test).
5. Bench: `load-names` + `u32s` at T=1/48/96/192, compare
   against the dormant baseline. Expect:
   - Cand-mode: unchanged (already used migration via parent
     slot, which works today via legacy high-bit encoding).
   - Exact-mode: slight increase per dispatch (one extra
     pointer-table indirection through S for migrated nodes),
     offset by cache-line savings from cn->key_bytes already
     being warm.
