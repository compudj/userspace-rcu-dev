# Skip-bit-budget refactor

## Status

Abandoned 2026-05-18.

The refactor was implemented end-to-end on `fractal-trie-dev-skip-bit`
(preserved at commit dbcc68ca) through Stages 1-7 plus follow-up
optimizations (external-check merge, branchful skip-apply, fused
descent step, class-gated cascade dispatcher).  Best post-optimization
state on dns ft_specv (taskset -c 0, -O2 -DNDEBUG, single-thread)
measured 270.8 ns/op median vs the pre-refactor baseline (this
commit, 97a6cc0c) of 192.5 ns/op — a +78 ns / +41% regression.

Critical-path analysis (disassembly of the fused descent step's
type-0 popcount_2l max_lc=3 hot path) identified the root cause:

  Pre-refactor (skip_len in pointer high bits):
    node_flag (in reg from prev iter dispatch)
      → shr node_flag, 56            ; 1c, no memory load
      → lea key+skip                 ; 1c
      → movzx [key+skip] → iter_key  ; 4-5c memory load
    (in parallel: bms_full load — 4-5c — for bitmap dispatch)

  Refactor (skip_len in child body byte):
    node_flag (in reg)
      → load bms_full [node-1]       ; 4-5c memory load
      → shr bms_full, 56 → skip      ; 1c
      → lea key+skip                 ; 1c
      → movzx [key+skip] → iter_key  ; 4-5c memory load (serialized)

The body-byte encoding adds ~5 cycles to the per-iter critical path
because the iter_key load must wait for the body load + skip
extraction.  Over a ~6-iter average dns descent this is ~30 cycles
/ ~9 ns, accounting for ~1/3 of the +78 ns regression.  The remainder
lives in the surrounding skip-target machinery (lazy validation
infrastructure, dispatch fan-out across many cases, producer-side
housekeeping touching cold lines), each of which costs a smaller
amount that compounds.

**Decision: keep the high-bit skip_len encoding (the design active at
this commit) as the going-forward layout for skip-compressed pointers.**
This document is preserved as a record of the explored alternative;
the implementation lives on `fractal-trie-dev-skip-bit` for inspection
but is not slated for merge.

The 2026-05-13 SUB-vs-AND tag-clear win (5-18% lookup gain) and the
type-0 occupancy data (30-64% of internal nodes) were the original
motivation; both remain valid observations, but recovering them inside
the high-bit-encoding constraints is the open question — orthogonal
to the body-byte / cache-page direction proposed below.

---

Design draft below preserved verbatim for reference.

## Original status

Design draft.  Motivated by the 2026-05-13 SUB-vs-AND tag-clear
experiment (5-18% lookup gain on the hot path) and the type-0
occupancy probe (type-0 dominates 30-64% of internal nodes).
Implementation pending.

## Goals

1. Replace the high-bit `FT_SKIP_LEN_SHIFT` encoding of skip-compressed
   pointers with a low-bit flag on internal-node pointers.
2. Move the skip-length and an inline subkey cache from pointer bits
   into the target node's header, so candidate descent can validate
   the cached prefix without touching the compressed node.
3. Enable the SUB-by-immediate tag-clear pattern on the dispatch hot
   path, replacing the current variable-shift AND mask.

## Non-goals

- Does not change the user-visible API.
- Does not change how compressed nodes are allocated or traversed
  in exact / inequality lookups (the compressed node remains as the
  authoritative source of the path bytes).
- Does not deprecate skip-compressed on architectures that already
  benefit from the high-bit encoding — this redesign supersedes
  the high-bit scheme cleanly, not in parallel.

## Context

### Today's tag layout (post collapse-removal, HEAD ec2526c0)

```
(ptr & 0b011) == 0b00   →  external node (leaf) or NULL
(ptr & 0b001) == 0b001  →  internal node (bit 0 set), bits 1-3 = type index
(ptr & 0b011) == 0b010  →  compressed path node
```

- Internal nodes are order-5 aligned (32B), so bits 0-4 of a raw
  internal pointer are zero.  Tag uses bits 0-3 (1 bit class + 3 bit
  type-index).  **Bit 4 is currently always zero in internal pointers
  and is unallocated tag space.**
- Compressed nodes are order-4 aligned (16B), so only bits 0-3 are
  zero in a raw compressed pointer.  Bit 4 carries address content.
- External nodes are order-3 aligned (8B), so only bits 0-2 are zero.

### Today's skip-compressed encoding

When `CDS_FT_FLAG_SKIP_COMPRESSED` is set on the group, the compressed
pointer in a parent's child slot is replaced by a *skip pointer* that
points directly to the compressed node's child.  The skipped path
length is encoded in the **high** bits of the pointer at
`FT_SKIP_LEN_SHIFT` (57 on x86-64, 56 on most other 64-bit archs).

Candidate descent ignores compressed-path bytes and uses the skip
pointer to jump over them in one indirection.  Exact and inequality
descent still need the path bytes; they fetch the compressed node via
`metadata->parent` (or `cds_ft_node.prev` for external chain heads).

### Why change it

1. **Dispatcher hot path arithmetic is expensive.**  The current
   `ft_node_ptr_internal` AND-clears the tag via
   `mask = (~15UL) << ((v >> 1) & 7); v & mask`.  That's a 5-insn
   variable-shift dep chain on every descent step.  Per-case
   `SUB(ptr, (T << 1) | 1)` collapses to a single LEA but requires
   compile-time-constant tag bits — i.e., a unique tag pattern per
   type.  Adding more tag patterns (skip-variant types) needs more
   tag-bit budget.

2. **High pointer bits are precious.**  Today's skip-len encoding
   consumes the top 7-8 bits.  These bits are also used for ASLR
   randomization, kernel mappings (kernel/user split), Intel CET
   shadow stack tags, and future CPU features.  Relocating skip
   metadata into the node body frees those bits for other uses.

3. **The compressed-node load is the last cache miss on the
   candidate hot path.**  Today's skip-compressed skips dereferencing
   the compressed node only when the candidate descent succeeds.
   Speculative-validated lookup (the dominant configuration per
   recent benches) revalidates with the inline SIMD comparator at the
   leaf, which sees the user's stored key.  But cand-mode without
   spec-validation still relies on the compressed node's bytes when
   the parent-pointer walk is invoked (e.g., for split/recompact).
   Moving the cache inline removes one indirection and improves cand
   latency on cold-cache traces.

4. **Type-0 is the right host for the cache.**  Probe data shows
   type-0 holds 30-64% of all internal nodes and 80%+ of internals
   on string workloads (dns, dict).  Putting the inline subkey cache
   in type-0 (and the skip-variant of other types) amortizes the
   cache over the most-visited nodes.

## Proposed encoding

### Tag scheme

| bit 0 | bit 1 | bits 2-4 / bit 2 | class | notes |
| --- | --- | --- | --- | --- |
| 0 | 0 | bit 2 = 0 | external, non-skip | bits 3..N are address |
| 0 | **1** | bit 2 = 0 | external, **skip-target** | bits 3..N are address |
| 0 | 0 | bit 2 = 1 | compressed, non-skip | bits 4..N are address |
| 1 | 0 | bits 2-4 = type index | internal, non-skip | bits 5..N are address |
| 1 | **1** | bits 2-4 = type index | internal, **skip-target** | bits 5..N are address |

Notes:

- **Bit 1 is the skip flag** and is orthogonal to class.  Because
  external (8B → bits 0-2 zero), compressed (16B → bits 0-3 zero),
  and internal (32B → bits 0-4 zero) are all aligned so that bit 1
  is zero in the raw address, the same bit serves all three.
- The compressed-flag (formerly bit 1) shifts one position left to
  bit 2.  Internal type-index (formerly bits 1-3) likewise shifts to
  bits 2-4.  All shifts fit within the alignment budget.
- A "compressed, skip-target" combination is not produced by the
  insert path (chain-compress never points one compressed node
  directly at another; skip-compressed today already targets the
  compressed node's *child*).  The row is reserved as undefined.
- `FT_SKIP_LEN_SHIFT` high-bit encoding is removed.  Skip pointers
  no longer exist as a distinct pointer class; instead, the parent
  slot holds a normal-tagged pointer with bit 1 set, pointing to the
  skip-target node.

### Skip-target node layout

#### Internal skip-target nodes: flexible per-instance cache width

Each existing internal type (P2L_32, P2L_64, P2L_128, P1L_*, PIGEON)
gains a *skip-target variant* with the **same allocation size** as
its non-skip counterpart (so the type-index 3-bit budget stays
unchanged).  Within that fixed alloc size, the **subkey cache width
varies per instance**: the compressed path bytes get inlined to
whatever fits, trading child capacity for cache bytes on a
per-node basis.

The mechanism keeps the **bitmap at offset 0** (matching the existing
popcount layout), grows `subkey_len + subkey[]` rightward immediately
after the bitmap, and grows the **pointer array leftward from the
end** of the node.  No `nr_child` field: it's recovered by
`popcount(root_bm)` exactly as today.

Layout sketch for a 32B type-0 skip-target (same bitmap shape as
non-skip type-0: `root_bm:u16 + sub_bm[0..2]:u16` = 8 bytes):

```
offset   content                                  size
  0      root_bm (u16)                            2
  2      sub_bm[0..max_lc-1] (u16 each)           2*max_lc
  B      subkey_len (u8)            descent advance length    1
  B+1    cached_subkey_len (u8)     inlined-bytes length      1
  B+2    subkey[0..cached_subkey_len-1]                       cached_subkey_len    ──→ grows right
                       . . . padding . . .
  ALLOC_SIZE - nr_child*8     ─────────────────────────────  pointer boundary
                       child_ptr[nr_child-1]                      ←── grows left
                       . . .
  ALLOC_SIZE - 8       child_ptr[0]
```

**Why two length fields:**

- `subkey_len` is the total length of the skipped compressed path —
  the amount by which descent advances the key cursor.  This is
  fixed for the life of the node (the compressed path is immutable).
- `cached_subkey_len` is how many of those bytes are actually
  stored inline.  When the path is short enough to fit, `cached_subkey_len
  == subkey_len`.  When the path is longer than the node has room
  for, `cached_subkey_len < subkey_len` and only a prefix is
  cached.  Setting `cached_subkey_len = 0` is valid: the node is
  still a skip-target (cand-mode advances by `subkey_len` without
  reading the cache); exact / inequality mode falls back to the
  compressed node `C` (reachable via `metadata->parent`) to read
  the un-cached tail.

This separation means **any compressed path length can be a
skip-target**, regardless of inline-cache capacity — the previous
"fall back to compressed-tagged parent slot for paths > cap" rule
is no longer needed.  The trade-off becomes per-instance: pick how
many bytes to cache (0..max), trade against pointer capacity.

Here `B = sizeof(bitmap)` is compile-time constant per type
(`B = 8` for type-0 in this example).  `max_lc` per type refers to
the bitmap's encoding capacity (unchanged from non-skip), not the
physical pointer slot count — physical capacity is bounded
separately by the per-instance `subkey_len`.

Properties:

- **No `nr_child` field.**  Recovered via `popcount(root_bm)` on
  every visit, matching the existing scan helpers.

- **Pointer indexing is compile-time constant per alloc size.**
  `ptr[i] = node_base + ALLOC_SIZE - (i+1)*8`.  The lookup hot path
  doesn't need to know `subkey_len` to fetch a child — only the
  skip-aware descent path consults it.  This preserves the
  SUB-by-imm-friendly dispatch.

- **Same scan algorithm as non-skip.**  Because the bitmap is at
  offset 0 with the same shape and the pointer offsets are anchored
  at `ALLOC_SIZE - 8*(i+1)` (which, in the existing scan helpers,
  is just where the pointer table happens to live for the
  flat-packed variants), the scan helpers can be shared.  The
  insert-side capacity check changes (see below) but the read-side
  popcount → index → pointer load does not.

- **Per-instance physical capacity** is
  `max_phys = (ALLOC_SIZE - B - 2 - cached_subkey_len) / 8`,
  rounded down.  The two-byte overhead is the
  `subkey_len + cached_subkey_len` header pair.
  `cached_subkey_len = 0` recovers the full non-skip capacity
  minus 2 bytes for the length fields.  The skip-flag bit on the
  pointer is what tells the dispatcher "consult `subkey_len` /
  `cached_subkey_len` before descending"; non-skip pointers
  short-circuit past the cache field entirely.

- **DNS-friendly choices**: for a 32B type-0 skip-target (B=8, so
  22 bytes for cache + pointers):
  - `cached_subkey_len=6 / max_phys=2`  (6-byte cache, 2 pointers)
  - `cached_subkey_len=14 / max_phys=1` (covers full path up to 14 bytes, 1 pointer)
  - `cached_subkey_len=0 / max_phys=2`  (long path, no inline cache, exact-mode falls back to C)

  Type-1 (64B, P2L_64 with B=12) has 50 bytes for cache + pointers,
  enabling e.g. `cached_subkey_len=10 / max_phys=5`,
  `cached_subkey_len=18 / max_phys=4`, etc.

- **Cap on `cached_subkey_len` per node**: `cached_subkey_len ≤
  min(subkey_len, max_inline_for_this_alloc_class)`.  The insert
  path picks the largest cached length that still leaves at least
  one pointer slot (`max_phys ≥ 1`), preferring full coverage when
  the path fits and degrading gracefully when it doesn't.

- **Grow / recompact** rule: when `nr_child + 1` would exceed the
  current per-instance `max_phys`, recompact to a wider layout
  in two possible ways:
  (a) **Same alloc class with reduced `cached_subkey_len`** — drop
      inline bytes to free pointer space.  Always valid (the dropped
      bytes are still findable via the compressed node), but exact
      / inequality cost goes up.
  (b) **Next alloc class** — same path used by non-skip overflow.

  The insert path picks (a) when `cached_subkey_len > 0` and the
  exact-mode hit rate on this descent path is dominated by
  spec-validated cand (which doesn't read the cache); picks (b)
  otherwise.  Heuristic TBD; provisionally prefer (b) so the cache
  width stays workload-stable.

**Why keep non-skip and skip-target as two distinct types rather
than unify with `subkey_len = 0`:** unification would force every
non-skip node to carry the 2-byte `subkey_len + cached_subkey_len`
header, which costs a pointer slot on the alloc classes that fit
their pointer array tightly:

| type | alloc | B (bitmap) | non-skip `max_lc` | unified (`B + 2 + 8*max_lc ≤ ALLOC`) | capacity loss |
| --- | --- | --- | --- | --- | --- |
| 0 | 32 | 8 | 3 | 2 | **−1 pointer** |
| 1 | 64 | 12 | 6 | 6 (4B slack absorbs the 2B) | 0 |
| 2 | 128 | 16 | 14 | 13 | **−1 pointer** |
| 3 | 256 | 32 | 28 | 27 | **−1 pointer** |

Type-0 is 30-64% of internal nodes per the probe; degrading its
`max_lc` from 3 to 2 globally would force significantly more
type-1 promotions on every workload, regardless of whether
skip-target is even in use.  The cost dominates any code-share
win, so non-skip and skip-target stay as separate types in the
`ft_types[]` table.

#### External skip-target nodes

External nodes carry no FT metadata.  When a compressed path
terminates at an external leaf and the skip flag is set on the
parent slot:

- The bit-1 tag on the external pointer marks "this leaf was the
  skip-target of a compressed path."
- Candidate descent terminates at the leaf without needing the
  skip length explicitly: in **spec-validated** mode the leaf
  comparator revalidates against the user's stored key, so the
  cand path can advance `depth = leaf.stored_key_len`.  In
  **non-spec-validated** cand mode, the depth-on-success is
  irrelevant to the public API (cand returns the leaf or NOT_FOUND;
  the caller revalidates).
- Exact and inequality descent follow `external->prev` (the head's
  `prev` field, which the duplicate-chain invariants set to the
  slot owner — the compressed node `C` when skip-compressed is in
  play) to load `C->key_bytes` and verify the path byte-by-byte.
  This matches today's behavior.

**External nodes need no new field** for skip-target support.  The
flag alone is the marker; storage stays in the compressed node `C`,
reachable from the external via `prev` exactly as today.

### Cand-mode descent change

Today's cand descent on a skip-compressed pointer:

```
if (ft_node_skip_compressed(child)) {
    skip = ft_skip_len(child);         // shift+mask of high bits
    key += skip;
    child = ft_skip_child_ptr(child);  // strip high bits
}
```

New cand descent on a skip-variant pointer:

```
if (ft_node_skip_variant(child)) {     // test bit 4
    struct skip_meta *m = ft_skip_meta(child);  // node header offset
    skip = m->skip_len;
    // (cand-mode: trust the cache, just advance)
    key += skip;
}
// child is already a normal internal pointer — no further unmasking
```

Exact and inequality descent paths additionally consult `m->subkey_cache`
to validate the skipped bytes against the user's key, avoiding the
compressed-node load when the cache holds the full path.

### SUB-by-immediate tag clear

With the skip flag at bit 1 and the type-index at bits 2-4, the
per-case tag-clear becomes:

```
FT_NODE_PTR_SUB(node_flag, T)       = node_flag - ((T << 2) | 1)
FT_NODE_PTR_SUB_SKIP(node_flag, T)  = node_flag - ((T << 2) | (1 << 1) | 1)
                                   = node_flag - ((T << 2) | 0b011)
```

Each branch in the type-index dispatch knows its concrete tag at
compile time (type-index and whether skip-target), so the SUB
immediate is a literal in every call site.  No variable-shift mask.

## Migration plan

The refactor lands as a sequence of small, individually-testable
commits.  Each commit keeps the smoke tests (`ft_unit`, `ft_inv`)
green.

### Stage 1: tag-layout reshuffling (no behavior change)

This stage is the riskiest because it touches the encoding of every
pointer-tag macro and every dispatch site.

- Shift the compressed-flag from bit 1 to bit 2:
  `FT_COMPRESSED_MASK = (1U << 2)`.
- Shift the type-index from bits 1-3 to bits 2-4:
  `FT_TYPE_MASK = (FT_TYPE_MAX_NR - 1) << 2`.
- Reserve bit 1 as `FT_SKIP_MASK = (1U << 1)`.
- Add `ft_node_skip_target(ptr)` helper that always returns `false`
  for now (no skip-target nodes exist yet — the bit is never set).
- Update all tag arithmetic in fractal-trie.c (mask constants,
  dispatch table indices, JSON kind table) to the new positions.
- Static assert that `FT_SKIP_MASK` is alignment-clear for all three
  classes (8B/16B/32B).
- Smoke tests pass; library size unchanged.

### Stage 2: skip-target type infrastructure

- Add `ft_type_skip[]` parallel array (or a flag in the existing
  `ft_types[]` entry) describing each non-skip type's skip-target
  slot count and offset of the inline-metadata field.
- Allocator helpers learn to allocate skip-target internal nodes
  (same arena class as non-skip, since size is identical).
- External skip-targets need no allocator change — same external
  layout, just the parent's tag bit differs.
- No skip-target node is yet produced — purely introduces the
  type-table plumbing.

### Stage 3: skip-target builders and accessors

- Implement per-type `ft_<type>_skip_set_nth`, `ft_<type>_skip_get_nth`,
  `ft_<type>_skip_get_direction`, etc., mirroring the non-skip
  helpers but reading the inline-metadata as the "missing" child
  slot for internal skip-targets.
- Wire `ft_node_skip_target(ptr)` to actually test bit 1.
- Update the dispatcher to route skip-target pointers (internal and
  external) through their handlers.  External skip-target descent
  is essentially a no-op vs non-skip external (the flag only
  influences the parent's slot interpretation, not the leaf itself).
- No insert path yet produces a skip-target pointer.

### Stage 4: insert-path migration

- The `ft_publish_skip_compressed` site (which today writes a
  `FT_SKIP_LEN_SHIFT`-tagged pointer) is split by child class:
  - **Internal child**: allocate a skip-target internal node, copy
    the compressed-path bytes into the inline subkey cache, set
    `skip_len`, and write a normal-tagged pointer to the new node
    with bit 1 set.  Migrate non-internal children to internal
    skip-target nodes by recompacting via the existing
    reparent-recompact path.
  - **External child**: write a normal external pointer with bit 1
    set.  No new node allocation; the existing external's `prev`
    field continues to point to the compressed node `C` for exact
    / inequality fallback.
- For compressed paths longer than `FT_INLINE_SKIP_LEN_MAX = 7`
  AND child is internal, the parent slot retains a compressed-tagged
  pointer (no skip-target — falls back to traditional compressed-node
  load).
- For external children, all skip-target paths are supported
  regardless of length (the external itself carries no cache).
- Dual-pointer RCU publication (skip pointer + cn->child) collapses
  to single-pointer publication of the skip-target pointer.
  `cn->child` is no longer load-bearing for cand-mode readers.

### Stage 5: remove `FT_SKIP_LEN_SHIFT` encoding

- Delete the high-bit skip-len encoding, all its arch-specific
  defines, and `ft_skip_to_compressed` / `ft_skip_child_ptr`.
- The compressed node remains allocated (still used for exact /
  inequality bytes), but no parent slot points to it as a skip
  pointer.  Parent slots are either compressed-tagged (for long
  compressed paths) or skip-variant-internal-tagged.

### Stage 6: SUB-by-immediate tag clear on hot path

- Replace `ft_node_ptr_internal`'s variable-shift mask with per-case
  `FT_NODE_PTR_SUB(node_flag, T)` macros in the dispatch arms.
- Add `FT_NODE_PTR_SUB_SKIP(node_flag, T)` for skip-target arms.
- The dispatcher becomes:
  ```
  if (likely(tag_with_skip == FT_TYPE_0_NOSKIP))
      return ft_popcount_2l_scan_16_16_max_3(...);
  if (likely(tag_with_skip == FT_TYPE_0_SKIP))
      return ft_popcount_2l_scan_16_16_max_2_skip(...);
  ...
  ```
  Each branch knows its concrete 5-bit tag (bit 0 + bit 1 + bits 2-4)
  at compile time.

### Stage 7: validation bench

- Run the same A/B harness as the post-cleanup validation (HEAD vs
  pre-refactor baseline) on u32s, dns, dict, plus paths and a
  multi-thread point (T=48 or T=96).
- Target: ≥5% lookup gain on dns/dict (where type-0 dominates), no
  regression on u32s.  Library size should stay within ±5% of
  current.

## Risks

| risk | mitigation |
| --- | --- |
| Per-instance `subkey_cache_len` wastes capacity when the trade-off was wrong (e.g., cache reserved but child count grows past `max_lc`) | Per-instance choice is made at insert time when the compressed path length is known; recompact path handles overflow.  Probe data shows type-0 modal `nr_child = 2` — sacrificing 1 slot is essentially free on average. |
| Insert path complexity: deciding `subkey_cache_len` per node | The choice is local: equal to the compressed path length being inlined, capped by the alloc class's largest cache size (which leaves `max_lc >= 1`).  No global heuristic needed. |
| Inline subkey cache truncates long compressed paths | `cached_subkey_len < subkey_len` is permitted; descent advances by `subkey_len` regardless, and exact / inequality fall back to the compressed node `C` for the un-cached tail.  Cand-mode and spec-validated paths are unaffected.  Cap depends on alloc class (32B → 14B cache, 64B → 30B+ cache). |
| Stage 5 deletion breaks an unforeseen call site that depends on high-bit encoding | Stage 4 is fully functional with both encodings coexisting; Stage 5 only removes after the full bench in Stage 7 passes. |
| Bit 1 was the compressed-flag; shifting it to bit 2 ripples through every tag-arithmetic site | Stage 1 is one atomic commit that does the whole shift; smoke tests gate the change.  Subsequent stages build on the new layout without revisiting it. |
| 32-bit architectures have tighter alignment budgets (compressed at 16B → bits 0-3 free; external at 8B → bits 0-2 free).  Bit 1 still works on 32-bit but no spare bits remain above bit 4 | Verified: skip flag at bit 1 is alignment-clear on every supported architecture.  Static assert in Stage 1 catches any regression. |
| Architecture portability — the SUB-by-imm pattern is x86-specific in the experiment | The arithmetic itself is portable; the win on non-x86 is less clear.  Keep the AND-mask fallback under a `FT_USE_SUB_CLEAR_TAG` build gate for non-x86 (mirroring the experiment's gate). |

## Open questions

1. **Inline cache width is now per-instance,** chosen at insert
   time based on the measured compressed-path length.  Open
   sub-question: what *cap* should we set across the type lattice?
   The natural bound is the minimum `max_lc` we're willing to host
   (e.g., `max_lc >= 1` always, which gives the largest possible
   cache per alloc class).  Profile Stage 4 to see how often the
   path length exceeds the largest cache the smallest type-0 node
   can host (16B for `max_lc=1` in a 32B node).  Falling back to
   non-skip + compressed-node load for paths that exceed that cap
   is acceptable; the cap should be set so that the fallback is
   rare on realistic workloads (DNS load-names: rare for >= 16-byte
   paths).

2. **Non-spec-validated variable-length cand on external skip-target.**
   For fixed-length keys the descent terminates at the leaf with
   `depth = max_key_len`; for spec-validated cand the leaf's stored
   key length is authoritative.  For the corner case of
   non-spec-validated cand on a variable-length group, descent
   would need either to read `skip_len` from the compressed node
   `C` (one cache line via `external->prev`) or accept that
   non-spec-validated cand on variable-length keys cannot determine
   the candidate's depth precisely.  Resolve during Stage 4 by
   benchmarking the cache-line-load fallback; if cheap, take it.

3. **Pigeon-256 skip variant.**  Pigeon already has 256 direct
   slots; sacrificing one is < 0.4% capacity.  But pigeon is rare
   (root-level only in the probe data).  Decide whether to skip the
   pigeon skip-variant in Stage 3 to reduce scope.

## References

- `project-ft-sub-vs-and-tagclear` (the experiment numbers)
- `project-ft-type0-occupancy-probe-2026-05-13` (population justification)
- `project-ft-qp-hi-skip-specialization` (earlier Phase-2 plan that
  proposed dropping one child to extend the inline cache; this
  refactor generalizes that idea)
- `project-ft-compressed-simd-audit` (SWAR/SSE compare audit on
  compressed nodes — informs cache-width decision)
