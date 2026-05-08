# Fractal Trie — 5-bit Tag Encoding (Chosen Layout: Candidate D)

**Status**: Decided 2026-05-08, branch `fractal-trie-dev-qp-wip`. Step 3
of the QP-wip plan. Read `qp-tag-bit-layout.md` (the 4-bit refactor that
shipped) and `qp-nibble-refactor.md` first.

The 4-bit kind encoding currently in production has 7 used kinds; the
QP-wip plan adds 4 more (POPCOUNT_32 / POPCOUNT_64 / SKIP_POPCOUNT_32 /
SKIP_POPCOUNT_64). One more bit gives us room while keeping the
external-node alignment at 16 bytes (only internal nodes need 32-byte
alignment).

This doc captures the decided layout and the staged migration plan.
Earlier candidates (A: pure one-hot; B: nibble-class with PIGEON ↔
COMPRESSED clash; C: compact-3-bit-kind) were considered and rejected;
their analysis is preserved in the git history of this file
(commit immediately preceding the chosen-layout rewrite).

## 1. Bit layout (Candidate D, chosen)

| Bit | Meaning |
|---|---|
| 0 | external (0) / internal (1) |
| 1 | skip (slot-context, FEATURE_FT_SKIP_COMPRESSED on) / compressed (node-context; or slot-context with FEATURE_FT_SKIP_COMPRESSED off) |
| 2 | one-hot QP |
| 3 | one-hot POPCOUNT_32 |
| 4 | one-hot POPCOUNT_64 |

Bits 2-4 are one-hot for the resolved internal class.  When all three
are zero, the kind is **PIGEON** (or its SKIP variant).  PIGEON occupies
the "default" pattern because the QP-nibble-refactor expects PIGEON to
be the rare kind on the read hot path; the explicit `bits_2_4 == 0`
check costs one extra branch which is acceptable for the rare case.

Encoding table:

| Kind                       | b4 | b3 | b2 | b1 | b0 | Hex  |
|----------------------------|----|----|----|----|----|------|
| `EXT`                      |  0 |  0 |  0 |  0 |  0 | `0x00` |
| `SKIP_EXT`                 |  0 |  0 |  0 |  1 |  0 | `0x02` |
| `PIGEON`                   |  0 |  0 |  0 |  0 |  1 | `0x01` |
| `COMPRESSED` (node-ctx)    |  0 |  0 |  0 |  1 |  1 | `0x03` |
| `SKIP_PIGEON` (slot-ctx)   |  0 |  0 |  0 |  1 |  1 | `0x03` |
| `QP`                       |  0 |  0 |  1 |  0 |  1 | `0x05` |
| `SKIP_QP` (slot-ctx)       |  0 |  0 |  1 |  1 |  1 | `0x07` |
| `POPCOUNT_32`              |  0 |  1 |  0 |  0 |  1 | `0x09` |
| `SKIP_POPCOUNT_32` (slot)  |  0 |  1 |  0 |  1 |  1 | `0x0B` |
| `POPCOUNT_64`              |  1 |  0 |  0 |  0 |  1 | `0x11` |
| `SKIP_POPCOUNT_64` (slot)  |  1 |  0 |  0 |  1 |  1 | `0x13` |

`COMPRESSED` (node-context) and `SKIP_PIGEON` (slot-context) share the
bit pattern `0x03`.  This is the only bit-pattern collision in the
encoding, and it is intentional: post-Step-2, every call site already
uses direction-aware predicates (`_in_node` vs `_in_slot`) which
disambiguate by the caller's context, not by the bit pattern.

## 2. Predicates (strict form)

```c
static inline bool ft_node_external(struct cds_ft_inode_flag *x) {
    return ((unsigned long) x & 0x01) == 0;
}

static inline bool ft_node_internal(struct cds_ft_inode_flag *x) {
    /* QP / PIGEON / POPCOUNT_X — *not* COMPRESSED.  Bit 0 set, bit 1 clear. */
    return ((unsigned long) x & 0x03) == 0x01;
}

static inline bool ft_node_compressed_in_node(struct cds_ft_inode_flag *x) {
    /* Strict: bits 2-4 must all be zero so SKIP_POPCOUNT_X (which set
     * bit 3 or 4) does not falsely match.  Single AND + CMP. */
    return ((unsigned long) x & 0x1F) == 0x03;
}

static inline bool ft_node_compressed_in_slot(struct cds_ft_inode_flag *x) {
#ifdef FEATURE_FT_SKIP_COMPRESSED
    /* Slots never carry raw COMPRESSED tag in skip-compressed builds —
     * publish_compressed always wraps in SKIP_X.  Constant-fold to false. */
    (void) x;
    return false;
#else
    return ((unsigned long) x & 0x1F) == 0x03;
#endif
}

static inline bool ft_node_skip_compressed_in_slot(struct cds_ft_inode_flag *x) {
    /* Universal SKIP predicate, matches all SKIP_X (including SKIP_EXT). */
    return ((unsigned long) x & 0x02) != 0;
}
```

Bit 0 (external/internal) and bit 1 (skip/compressed) are the same
positions as today; predicate forms for `ft_node_external` and
`ft_node_skip_compressed_in_slot` are unchanged.

The single behavioural change is `ft_node_compressed_in_node`: its
mask grows from 4 bits (`0x0F`) to 5 (`0x1F`) and the expected value
flips from `0x01` to `0x03`.  The strict mask (`0x1F`) is the only
correct form — the lenient `(x & 0x07) == 0x03` would falsely match
SKIP_POPCOUNT_32 (`0x0B`) and SKIP_POPCOUNT_64 (`0x13`) in node
context, which would route a skip-encoded popcount slot value through
`ft_compressed_node_ptr` and corrupt downstream behaviour.

## 3. Read-side hot-path dispatch

```c
unsigned long t = (unsigned long) nf & 0x1F;
if (caa_unlikely((t & 0x01) == 0))    goto external_or_skip_ext;
if (caa_unlikely((t & 0x02) != 0))    goto skip_resolve_or_compressed;

/* Direct internal kind: bits 2-4 are one-hot for the class. */
unsigned long class_bits = t & 0x1C;   /* bits 2-4 isolated, in their high positions */
if (caa_likely(class_bits == 0x04))    return ft_qp_byte_get(...);     /* QP */
if (class_bits == 0x00)                return ft_pigeon_node_get_nth(...); /* PIGEON */
if (class_bits == 0x08)                return ft_popcount32_node_get_nth(...);
if (class_bits == 0x10)                return ft_popcount64_node_get_nth(...);
__builtin_unreachable();
```

QP is the expected-most-frequent direct internal kind (per the QP-wip
design intent), so it's tested first with `caa_likely`.  PIGEON is the
"default-pattern" kind (bits 2-4 = 0) and gets the explicit `class_bits
== 0` arm.

The compiler should fold the four-arm if-chain into a small jump
table on `t & 0x1C`.

## 4. Pointer mask (alignment-aware)

`ft_node_ptr` strips the tag bits to recover the underlying pointer.
The mask depends on the kind's underlying alignment:

| Kind | Underlying alignment | Mask |
|---|---|---|
| EXT, SKIP_EXT | 16 B | `~15UL` |
| COMPRESSED | 16 B (alloc order ≥ 4) | `~15UL` |
| QP, PIGEON, POPCOUNT_X | 32 B (alloc order ≥ 5) | `~31UL` |

In *node-context* (the only context where `ft_node_ptr` is called),
SKIP_X tags are absent (callers must resolve SKIP first via
`ft_resolve_skip_compressed`). The remaining classification is:

- Bit 0 = 0: external (16 B aligned), mask `~15UL`.
- Bit 0 = 1, Bit 1 = 0: direct internal (32 B aligned), mask `~31UL`.
- Bit 0 = 1, Bit 1 = 1: COMPRESSED (16 B aligned), mask `~15UL`.

The dispatch reduces to a single test on `(v & 0x03) == 0x01`:

```c
static inline struct cds_ft_inode *ft_node_ptr(struct cds_ft_inode_flag *nf) {
    unsigned long v = (unsigned long) nf;
    /* (v & 0x03) == 0x01 -> direct internal (PIGEON/QP/POPCOUNT_X), 32 B align.
     * Else (external or COMPRESSED) -> 16 B align. */
    unsigned long mask = ((v & 0x03) == 0x01) ? ~31UL : ~15UL;
    return (struct cds_ft_inode *) (v & mask);
}
```

The fast variant `ft_node_ptr_internal` (caller has already
established that the value is a direct internal kind, i.e. not
COMPRESSED) drops the conditional:

```c
static inline_lookup
struct cds_ft_inode *ft_node_ptr_internal(struct cds_ft_inode_flag *nf) {
    return (struct cds_ft_inode *) ((unsigned long) nf & ~31UL);
}
```

`ft_compressed_node_ptr` is unchanged in form — still a SUB by the
COMPRESSED tag value (now `0x03` instead of `0x01`). The
prefetcher's stride detector continues to see the SUB as a linear
offset.

## 5. Alignment invariants

Audit results (Stage 1, performed before Stage 2 lands):

| Allocator | Min alignment | Tag bits used | Notes |
|---|---|---|---|
| External nodes (`struct cds_ft_node`) | 16 B | bits 0-3 (tag = 0x00 / 0x02) | `__aligned__(16)` per `include/urcu/fractal-trie.h:458`. Bit 4 of the address remains unmasked. |
| QP nodes (T0..T3) | 32 B | bits 0-4 (full 5-bit tag) | `FT_QP16_T0_ALLOC_ORDER = 5` per `src/fractal-trie-internal.h:503`. |
| PIGEON nodes | 2 KB | bits 0-4 | `FT_PIGEON_ORDER = 11` (`src/fractal-trie.c:183`); huge headroom. |
| Compressed nodes | 16 B | bits 0-3 (tag = 0x03) | `ft_compressed_order(path_len)` clamps to ≥ 4 (`src/fractal-trie.c:2262`). 1-7 byte compresseds stay at 16 B; 8+ byte compresseds are 32 B. COMPRESSED's tag (`0x03`) only sets bits 0-1, so bit 4 stays an address bit. |
| POPCOUNT_32 / POPCOUNT_64 (Stage 3) | 32 B (planned) | bits 0-4 | New kinds; allocator orders set to 5+ when wired up. |

Compressed nodes stay 16-byte aligned to avoid bumping every 1-7 byte
compressed by 16 B (significant memory cost given chain-compress
emits many 1-byte canonical compresseds). COMPRESSED's tag value
(`0x03`) only sets bits 0-1, leaving bit 4 free as part of the
address — so `ft_node_ptr` masking with `~15UL` correctly recovers
the compressed-node address.

The cost of this asymmetry is paid in `ft_node_ptr`: one extra CMP +
CMOV to dispatch mask width. The fast variant
`ft_node_ptr_internal` is unchanged — it's used on hot lookup paths
where the kind is already known to be direct internal.

## 6. Migration stages

Each stage compiles, smoke-tests cleanly (`test_urcu_ft_unit` 185 +
1 pre-existing fail; `test_urcu_ft_inv` 12), and is committed
independently.

### Stage 1 — design + alignment audit (this commit)

Audit the alignment of every node class and document the encoding.
No code change.  Audit findings:

- External nodes: `__aligned__(16)` per `include/urcu/fractal-trie.h`.
  Stays at 16 B; SKIP_EXT and EXT both use only bits 0-3 of the tag,
  so bit 4 of the address remains an address bit.
- QP nodes: `FT_QP16_T0_ALLOC_ORDER = 5` (32 B).  Already 32-byte
  aligned for all tiers.
- PIGEON nodes: `FT_PIGEON_ORDER = 11` (2 KB).  Already 32-byte
  aligned (and then some).
- Compressed nodes: `ft_compressed_order` clamps to ≥ 4 (16 B); 1-7
  byte path lengths fit in a single 16 B node.  We keep this 16-byte
  alignment and accept the asymmetric `ft_node_ptr` mask described
  in §4.

No allocator changes are required.  The Stage 2 flip can land
without bumping any allocation order.

### Stage 2 — flip predicates and constructors atomically

This is the atomic encoding change.  In a single commit:

1. Update `enum ft_kind` values:
   - `FT_KIND_PIGEON` 0x9 → 0x1
   - `FT_KIND_COMPRESSED` 0x1 → 0x3
   - `FT_KIND_SKIP_PIGEON` 0xB → 0x3
   - `FT_KIND_QP` keeps 0x5
   - `FT_KIND_SKIP_QP` keeps 0x7
   - `FT_KIND_EXT` keeps 0x0
   - `FT_KIND_SKIP_EXT` keeps 0x2

2. Update `FT_KIND_MASK` 0xF → 0x1F.

3. Update `ft_node_ptr` to alignment-aware mask
   (`((v & 0x03) == 0x01) ? ~31UL : ~15UL`).  The "lookup-hot"
   variant `ft_node_ptr_internal` flips to `~31UL` unconditionally.

4. Update `ft_compressed_node_flag` to OR `0x3` (was `0x1`).
   Update `ft_compressed_node_ptr` to subtract `0x3` (was `0x1`).
   Both still satisfy the SUB-friendly low-bit isolation that the
   prefetcher's stride detector depends on.

5. Update `ft_node_compressed_in_node` and `_in_slot` to the new
   strict form (`(x & 0x1F) == 0x03`).

6. Update `ft_node_skip_compressed_in_slot` to match all SKIP_X via
   bit 1.  The encoding doesn't change here; this is just a doc
   update in the predicate body.

7. Update PIGEON-family bit identification: `FT_KIND_PIGEON_FAMILY_BIT`
   (currently `0x8`, matching the high bit of PIGEON 0x9 / SKIP_PIGEON
   0xB) is no longer meaningful — PIGEON has bits 2-4 = 0, which is
   the *absence* of any class bit.  Drop the macro; rewrite call
   sites to test `(x & 0x1C) == 0` (no class bits set, indicating
   PIGEON or SKIP_PIGEON).

8. Update `FT_KIND_INTERNAL_BITS` (currently `0xC`, "is QP or
   PIGEON" via bits 2-3): no longer holds — QP has bit 2, PIGEON has
   bits 2-4 = 0, POPCOUNT_X have bits 3 or 4.  The new "is internal
   non-compressed" predicate is `(x & 0x03) == 0x01` (bit 0 set, bit 1
   clear).  Inline this at every call site; drop the macro.

This commit is the largest individual change in the migration; expect
~200 LOC.  The atomic flip is required because the predicates and the
constants are intertwined.

### Stage 3 — wire up POPCOUNT_32 / POPCOUNT_64 (Step 4 in the parent plan)

Once the encoding is in place, add the new kinds:

1. Add `FT_KIND_POPCOUNT_32 = 0x09`, `FT_KIND_POPCOUNT_64 = 0x11`,
   `FT_KIND_SKIP_POPCOUNT_32 = 0x0B`, `FT_KIND_SKIP_POPCOUNT_64 =
   0x13` to `enum ft_kind`.

2. Add `ft_types[]` entries and helpers (`ft_popcount32_*`,
   `ft_popcount64_*`).  This is the bulk of Step 4 in the parent
   plan, but the encoding piece is just adding the enum values
   (already valid under Stage 2's `FT_KIND_MASK = 0x1F`).

3. Add descent-dispatch arms.

4. Add SKIP_X publish/resolve handling for the new kinds.

This is properly Step 4 of the parent plan; it lands as its own
commit(s) after Stage 2.

### Stage 4 — drop legacy 4-bit constants and audit

After Stages 2-3 settle, look for any remaining direct comparisons
against literal tag values (e.g. `tag == 0x9` for PIGEON) that the
mass migration may have missed.  Also audit:

- `FT_KIND_PIGEON_FAMILY_BIT` — should be removed in Stage 2; if any
  call site still uses it, fix.
- `FT_KIND_INTERNAL_BITS` — same.
- Any place that tests `(tag & 0xF)` instead of `(tag & 0x1F)` —
  these would silently mis-classify POPCOUNT_64 (`0x11`) as PIGEON
  (`0x01`) since the high bit is masked off.

## 6.5. Stage 2b discovery (2026-05-08): SKIP_PIGEON / COMPRESSED ambiguity

A first attempt at the atomic encoding flip surfaced a deeper issue
than the original plan accounted for.  The plan assumed call sites
that consult bit 1 (`FT_KIND_SKIP_BIT`) could be left alone — under
the old encoding, COMPRESSED's tag (`0x1`) had bit 1 clear, so
`(node & 0x02) != 0` cleanly matched only SKIP_X kinds.  Under the
new encoding, COMPRESSED (`0x03`) and SKIP_PIGEON (`0x03`) share both
bits 0 and 1, and *every* SKIP_X variant has bit 1 set.  Plain `bit
1` no longer distinguishes "SKIP-encoded slot value" from
"COMPRESSED resolved value".

The direction-aware predicate aliases landed in 60bdc30b / 9a34a949
already encode the right intent:
`ft_node_skip_compressed_in_slot` is for slot-context callers
(where COMPRESSED can't appear, so bit 1 = SKIP), and
`ft_node_compressed_in_node` is for node-context callers
(where SKIP_X can't appear, so bit 1 = COMPRESSED).  But there is a
class of call sites that operates on *mixed* contexts — values that
might be slot-encoded (just read from a parent slot) or
node-context (returned from `ft_resolve_skip_compressed` earlier in
the same path) — and those sites cannot decide from the bits alone.

The most prominent example is `ft_resolve_skip_compressed` itself.
Several callers (e.g. `cds_ft_lookup_inequality:6917`,
`_cds_ft_insert:8500`, `cds_ft_remove:9719`) pass `node_flag` whose
context is mixed: it might be a freshly-read slot value (SKIP_X
candidate) or a value left from a prior resolve (COMPRESSED
candidate).  Under the old encoding the resolve was a no-op for
COMPRESSED (bit 1 clear), so callers could call it freely.  Under
the new encoding, calling resolve on COMPRESSED erroneously fires
`ft_skip_to_compressed`, which then tries to recover the cn from
the compressed pointer's `prev` (a parent which is *not* itself
compressed) and trips the `ft_compressed_node_ptr` assertion.

### What it took to discover

A single-key insert (`test_insert_basic`) crashed during
`cds_ft_count_entries`.  The descent built a SKIP_EXT pointer
(legitimate — chain-compress wrapped the 4-byte path), the
inequality-lookup back-walk resolved that into a COMPRESSED-tagged
cn flag (also legitimate), and the *next* iteration of the descent
loop called `ft_resolve_skip_compressed` again on the now-COMPRESSED
value (legitimate under the old encoding's bit 1 = 0 invariant for
COMPRESSED, broken under the new encoding's bit 1 = 1).

### Implications for the migration plan

Stage 2b cannot be a single atomic commit covering encoding +
predicate updates.  It needs to be split:

1. **Stage 2b.1 — audit `ft_resolve_skip_compressed` callers.**
   Each call site is one of:
   - **Pure slot-context** (raw slot read, never resolved): keep
     the call.  Bit-1 test correctly catches SKIP_X.
   - **Pure node-context** (post-resolve, parent ptr from
     metadata): drop the call.  The value is already resolved.
   - **Mixed-context** (could be either): rewrite to test
     unambiguously.  Options:
     a. Test for SKIP_X variants that *don't* collide with
        COMPRESSED — i.e. SKIP_EXT (`(x & 0x03) == 0x02`) and
        SKIP_X-with-class-bit (`(x & 0x02) && (x & 0x1C)`).
        Skip this branch for the 0x03 pattern entirely
        (treat as already-resolved).
     b. Restructure the surrounding code so the value's context
        is determined unambiguously before reaching the resolve.

2. **Stage 2b.2 — audit `ft_skip_compressed_skip_len` and other
   functions that switch on the kind tag.**  Same issue: a tag
   read of `0x03` could be COMPRESSED (node) or SKIP_PIGEON (slot).
   Functions that operate on slot values should be safe; functions
   called in mixed-context need rewriting.

3. **Stage 2b.3 — kind extraction.**  `& FT_KIND_MASK` returns a
   different value depending on whether the underlying alignment is
   16-byte (EXT, SKIP_EXT, COMPRESSED) or 32-byte (QP, PIGEON,
   POPCOUNT_X, SKIP variants thereof).  An EXT pointer at an
   address with bit 4 set yields kind `0x10` instead of `0x00`
   under the 5-bit mask.  Kind-extraction sites need to dispatch
   on alignment first.  This was caught early in the first attempt:
   the fix is to dispatch on `(v & 0x01)` (external/internal) and
   choose `& 0x0F` or `& 0x1F` accordingly.

4. **Stage 2b.4 — the actual encoding flip.**  Once 2b.1-2b.3 are
   each landed and smoke-tested under the *old* encoding (each
   change preserves semantics), the final encoding flip becomes
   small and contained: just the enum values, `FT_KIND_MASK`,
   `ft_node_ptr` alignment dispatch, and the predicates' bit
   patterns.

The first attempt skipped 2b.1-2b.3 and tried to do everything in
one commit.  The result compiled cleanly but crashed on the first
insert.  The smoke tests caught it; the production CI (which runs
under `-DNDEBUG` and so silences the assertion) would have
mis-classified slot values as compressed and corrupted state
silently.  Lesson: each pre-encoding-flip stage must preserve
semantics under the *current* encoding, not just under the future
one.

A reverted snapshot of the broken atomic flip lives in the local
working-directory history; rebuilding it from the design table is
straightforward once 2b.1-2b.3 land.

## 7. Risks and rollback

The Stage 2 flip is the high-risk commit.  Rollback strategy:

- Each stage is a clean revert if needed.  Stage 2 is a single commit
  by design; `git revert` recovers the 4-bit encoding.
- Smoke tests (`test_urcu_ft_unit` 186, `test_urcu_ft_inv` 12) must
  pass after Stage 2.  Any failure means either a missed call site
  or a predicate bug.
- The longer-duration `test_urcu_ft_inv` runs (60 s, 12-readers /
  4-writers) should be re-run after Stage 2 to surface any
  concurrent-path regression in the SKIP-resolve code.
- A 32-bit build was never enabled for skip-compressed historically
  (memory `project_ft_skip_compressed_32bit.md`); the 5-bit encoding
  is fine on 64-bit but tag bits compete with address bits even more
  on 32-bit pointers.  Confirm 32-bit build still compiles after
  Stage 2 (it may, since the tag bits are in the low 5 bits which
  remain free under 32-byte alignment).

## 8. Open questions

1. **`ft_node_internal` re-definition.**  Today the predicate matches
   QP / PIGEON / SKIP variants thereof — basically "anything with
   bit 0 set and not COMPRESSED".  Under Candidate D, the strict
   "non-compressed internal" test is `(x & 0x03) == 0x01`.  But the
   old "anything with bit 0 set" test (`(x & 0x01) != 0`) is wider —
   it matches COMPRESSED and SKIP_X too.  Audit each call site of
   `ft_node_internal` to determine which semantic the caller wants.

2. **`ft_node_type_index`.**  Today this returns the `ft_types[]`
   slot index for an internal node (QP T0..T3 = 0..3; PIGEON = 4).
   Under the new layout, the slot index is recoverable from
   `cds_ft_item_order(node) - FT_QP16_T0_ALLOC_ORDER` for QP and a
   constant `4` for PIGEON; POPCOUNT_X get new indices.  The function
   stays but its body gets a switch on `ft_kind_of(x)` instead of
   bit math.

3. **Reserved bit patterns.**  `0x04`, `0x06`, `0x08`, `0x0A`, `0x0C`,
   `0x0D`, `0x0E`, `0x0F`, `0x10`, `0x12`, `0x14`-`0x1F` (most of
   them) are unused in the new encoding.  Some are reachable only via
   bit-manipulation bugs and should `__builtin_unreachable` /
   `assert(0)` in dispatch defaults.  Others (e.g. `0x10`, "external
   side with bit 4") are address bits for external pointers and would
   trip up a tag-aware test if not masked properly.  Document the
   reserved set in `fractal-trie-internal.h`.
