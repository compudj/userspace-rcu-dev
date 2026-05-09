# Fractal Trie — 5-bit Tag Encoding (Chosen Layout: Candidate E)

**Status**: Decided 2026-05-09, branch `fractal-trie-dev-qp-wip`. Step 3
of the QP-wip plan. Read `qp-tag-bit-layout.md` (the 4-bit refactor that
shipped) and `qp-nibble-refactor.md` first.

The 4-bit kind encoding currently in production has 7 used kinds; the
QP-wip plan adds 4 more (POPCOUNT_32 / POPCOUNT_64 / SKIP_POPCOUNT_32 /
SKIP_POPCOUNT_64). One more bit gives us room while keeping the
external-node alignment at 16 bytes (only internal nodes need 32-byte
alignment).

This doc captures the decided layout and the staged migration plan.
Earlier candidates were considered and rejected:
- **A** (pure one-hot)
- **B** (nibble-class with PIGEON ↔ COMPRESSED clash)
- **C** (compact-3-bit-kind)
- **D** (one-hot with intentional COMPRESSED ↔ SKIP_PIGEON collision at
  `0x03`, disambiguated by call-site `_in_node` / `_in_slot` context)

Candidate D was the chosen layout briefly (decided 2026-05-08) but
caused trouble: the COMPRESSED ↔ SKIP_PIGEON pattern collision meant
mixed-context callers could not safely test bit patterns alone, and a
parent-kind-boundary race in compressed-split (the SKIP_X demote bug,
fixed 2026-05-09) used the same call sites that the collision made
fragile.  The fix that landed (commit 9dfff634) is independent of the
encoding choice, but the analysis raised the question: is the
collision worth the saved tag value?  Candidate E says no — moving
COMPRESSED to a free bit pattern in the *external-aligned* half of the
encoding is essentially free and dissolves the collision.

Earlier candidates (and the brief Candidate D) are preserved in the
git history of this file.

## 1. Bit layout (Candidate E, chosen)

| Bit | Meaning |
|---|---|
| 0 | external-aligned (0) / internal-aligned (1) |
| 1 | skip variant (1) — applies in both halves |
| 2 | external-half: COMPRESSED indicator. internal-half: QP one-hot |
| 3 | one-hot POPCOUNT_32 (internal-half) |
| 4 | one-hot POPCOUNT_64 (internal-half) |

Bit 0 partitions the encoding by underlying alignment: the external-
aligned half (16-byte aligned: EXT, SKIP_EXT, COMPRESSED) has bit 0
clear; the internal-aligned half (32-byte aligned: PIGEON, QP,
POPCOUNT_*, and their SKIP variants) has bit 0 set.

Bit 1 is the universal "skip" indicator across both halves.

Bits 2-4 are one-hot for the resolved internal class within the
internal-aligned half.  When all three are zero, the kind is
**PIGEON** (or its SKIP variant) — the "default" pattern, on the
assumption that PIGEON is the rare kind on the read hot path.

In the external-aligned half, bit 2 is repurposed as the COMPRESSED
indicator: bit-0=0 + bit-2=1 = COMPRESSED.

Encoding table:

| Kind                       | b4 | b3 | b2 | b1 | b0 | Hex   |
|----------------------------|----|----|----|----|----|-------|
| `EXT`                      |  0 |  0 |  0 |  0 |  0 | `0x00`|
| `PIGEON`                   |  0 |  0 |  0 |  0 |  1 | `0x01`|
| `SKIP_EXT`                 |  0 |  0 |  0 |  1 |  0 | `0x02`|
| `SKIP_PIGEON`              |  0 |  0 |  0 |  1 |  1 | `0x03`|
| `COMPRESSED`               |  0 |  0 |  1 |  0 |  0 | `0x04`|
| `QP`                       |  0 |  0 |  1 |  0 |  1 | `0x05`|
| `SKIP_QP`                  |  0 |  0 |  1 |  1 |  1 | `0x07`|
| `POPCOUNT_32`              |  0 |  1 |  0 |  0 |  1 | `0x09`|
| `SKIP_POPCOUNT_32`         |  0 |  1 |  0 |  1 |  1 | `0x0B`|
| `POPCOUNT_64`              |  1 |  0 |  0 |  0 |  1 | `0x11`|
| `SKIP_POPCOUNT_64`         |  1 |  0 |  0 |  1 |  1 | `0x13`|

**No bit-pattern collisions.**  Every used pattern is uniquely
identified.  COMPRESSED and SKIP_PIGEON occupy different patterns
(`0x04` vs `0x03`) — the direction-aware predicate split (`_in_node`
vs `_in_slot`) added in Step 2 is no longer required for
disambiguation; it remains useful only as a documentation aid for
caller intent.

Reserved (assert-on-encode): `0x06`, `0x08`, `0x0A`, `0x0C`-`0x0F`,
`0x10`, `0x12`, `0x14`-`0x1F`.  Bit pattern `0x06` (bit 0=0, bit 1=1,
bit 2=1) is reserved: it would mean "skip-compressed of COMPRESSED",
forbidden by the chain-compress invariant (no two adjacent
compresseds).

## 2. Predicates (strict form)

```c
static inline bool ft_node_external(struct cds_ft_inode_flag *x) {
    /* EXT or SKIP_EXT: bit 0 clear, bit 2 clear (excludes COMPRESSED). */
    return ((unsigned long) x & 0x05) == 0;
}

static inline bool ft_node_external_direct(struct cds_ft_inode_flag *x) {
    /* EXT only (post-resolve): bit 0 clear, bits 1-2 clear. */
    return ((unsigned long) x & 0x07) == 0;
}

static inline bool ft_node_internal(struct cds_ft_inode_flag *x) {
    /* QP / PIGEON / POPCOUNT_X (direct, post-resolve).
     * Bit 0 set, bit 1 clear. */
    return ((unsigned long) x & 0x03) == 0x01;
}

static inline bool ft_node_compressed(struct cds_ft_inode_flag *x) {
    /* COMPRESSED in any context.  Bit 2 set, bits 0-1 clear.
     * Lenient on bits 3-4 — they are address bits for 16-B-aligned
     * compressed nodes.  Strict on bits 0-2 disambiguates from EXT
     * (0x00), SKIP_EXT (0x02), and all internal-half kinds. */
    return ((unsigned long) x & 0x07) == 0x04;
}

static inline bool ft_node_skip_compressed_in_slot(struct cds_ft_inode_flag *x) {
    /* Universal SKIP predicate: matches all SKIP_X (SKIP_EXT,
     * SKIP_PIGEON, SKIP_QP, SKIP_POPCOUNT_*).  Bit 1 set. */
    return ((unsigned long) x & 0x02) != 0;
}
```

The single-bit `ft_node_external` test from Step 2 (`bit 0 == 0`)
becomes `(x & 0x05) == 0` because COMPRESSED also has bit 0 = 0; we
need bit 2 clear too to distinguish.  All other predicates remain
single-AND-and-CMP.

`ft_node_compressed_in_node` and `ft_node_compressed_in_slot` collapse
into a single `ft_node_compressed` predicate — both contexts use the
same bit-pattern test and there is no ambiguity to resolve.  The
direction-aware aliases from Step 2 stay as forwarding wrappers for
documentation, but the bodies converge.

## 3. Read-side hot-path dispatch

```c
unsigned long t = (unsigned long) nf & 0x1F;

if (caa_unlikely((t & 0x01) == 0)) {
    /* External-aligned half: bits 0-2 are clean (16-B alignment
     * means address bits 0-3 are zero; only address bit 4 can leak
     * into the 5-bit mask, but it does not affect bits 0-2). */
    if (t & 0x04)        goto compressed;       /* 0x04 */
    if (t & 0x02)        goto skip_ext;         /* 0x02 */
    goto ext_or_null;                           /* 0x00 */
}

/* Internal-aligned half: bits 0-4 are all clean kind bits
 * (32-B alignment forces address bits 0-4 to zero). */
if (caa_unlikely((t & 0x02) != 0))
    goto skip_resolve;                          /* 0x03 / 0x07 / 0x0B / 0x13 */

/* Direct internal kind: bits 2-4 are one-hot for the class. */
unsigned long class_bits = t & 0x1C;
if (caa_likely(class_bits == 0x04))    goto qp;            /* QP, 0x05 */
if (class_bits == 0x00)                goto pigeon;        /* PIGEON, 0x01 */
if (class_bits == 0x08)                goto popcount_32;   /* 0x09 */
if (class_bits == 0x10)                goto popcount_64;   /* 0x11 */
__builtin_unreachable();
```

QP is the expected-most-frequent direct internal kind (per the QP-wip
design intent), so it's tested first with `caa_likely`.  PIGEON is the
"default-pattern" kind (bits 2-4 = 0) and gets the explicit
`class_bits == 0` arm.

The compiler should fold the four-arm if-chain into a small jump
table on `t & 0x1C`.

**Dispatch cost vs Candidate D**: same.  Bit 0 test partitions into
external-aligned vs internal-aligned; in the external-aligned arm we
test bit 2 (COMPRESSED) before bit 1 (SKIP_EXT) — a single extra test
on the cold external arm, which is dominated by the leaf compare
anyway.  The hot internal arm is identical.

## 4. Pointer mask (alignment-aware)

`ft_node_ptr` strips the tag bits to recover the underlying pointer.
The mask depends on the kind's underlying alignment, which is now
encoded *exactly* by bit 0:

| Bit 0 | Half | Alignment | Mask |
|---|---|---|---|
| 0 | external-aligned (EXT, SKIP_EXT, COMPRESSED) | 16 B | `~15UL` |
| 1 | internal-aligned (PIGEON, SKIP_PIGEON, QP, SKIP_QP, POPCOUNT_*, SKIP_POPCOUNT_*) | 32 B | `~31UL` |

In *node-context* (the only context where `ft_node_ptr` is called),
SKIP_X tags are absent (callers must resolve SKIP first via
`ft_resolve_skip_compressed`).  The dispatch reduces to a single test
on bit 0:

```c
static inline struct cds_ft_inode *ft_node_ptr(struct cds_ft_inode_flag *nf) {
    unsigned long v = (unsigned long) nf;
    /* bit 0 == 0 → 16-B-aligned (EXT or COMPRESSED post-resolve);
     *           1 → 32-B-aligned (direct internal). */
    unsigned long mask = ((v & 0x01) == 0) ? ~15UL : ~31UL;
    return (struct cds_ft_inode *) (v & mask);
}
```

Compared to Candidate D's `((v & 0x03) == 0x01) ? ~31UL : ~15UL` test,
this is a single-bit test on bit 0 — strictly simpler.

The fast variant `ft_node_ptr_internal` (caller has already
established that the value is a direct internal kind) drops the
conditional:

```c
static inline_lookup
struct cds_ft_inode *ft_node_ptr_internal(struct cds_ft_inode_flag *nf) {
    return (struct cds_ft_inode *) ((unsigned long) nf & ~31UL);
}
```

`ft_compressed_node_ptr` is unchanged in form — still a SUB by the
COMPRESSED tag value (now `0x04` instead of `0x01`).  The
prefetcher's stride detector continues to see the SUB as a linear
offset.

## 5. Alignment invariants

Audit results (Stage 1, performed before Stage 2 lands):

| Allocator | Min alignment | Tag bits used | Notes |
|---|---|---|---|
| External nodes (`struct cds_ft_node`) | 16 B | bits 0-3 (tag = `0x00` / `0x02`) | `__aligned__(16)` per `include/urcu/fractal-trie.h:458`.  Bit 4 of the address remains unmasked. |
| QP nodes (T0..T3) | 32 B | bits 0-4 (full 5-bit tag) | `FT_QP16_T0_ALLOC_ORDER = 5` per `src/fractal-trie-internal.h`. |
| PIGEON nodes | 1-2 KB | bits 0-4 | `FT_PIGEON_ORDER = 10` (32-bit) / `11` (64-bit); huge headroom. |
| Compressed nodes | 16 B | bits 0-3 (tag = `0x04`) | `ft_compressed_order(path_len)` clamps to ≥ 4.  1-12 byte compresseds fit in a 16-B node; longer paths get 32-B+ nodes.  COMPRESSED's tag (`0x04`) only sets bit 2, so bit 4 stays an address bit for 16-B compressed nodes. |
| POPCOUNT_32 / POPCOUNT_64 (Stage 3) | 32 B (planned) | bits 0-4 | New kinds; allocator orders set to 5+ when wired up. |

Compressed nodes stay 16-byte aligned to avoid bumping every 1-12 byte
compressed by 16 B (significant memory cost given chain-compress
emits many 1-byte canonical compresseds).  COMPRESSED's tag value
(`0x04`) only sets bit 2 — bits 3-4 of a 16-B-aligned compressed
pointer remain part of the address.  The `& 0x07` mask in
`ft_node_compressed` extracts only bits 0-2, which are clean kind
bits.  The `~15UL` mask in `ft_node_ptr` correctly recovers the
compressed-node address.

The cost of the alignment asymmetry is paid in `ft_node_ptr`: one CMP
+ CMOV on bit 0.  The fast variant `ft_node_ptr_internal` is
unchanged — used on hot lookup paths where the kind is already known
to be direct internal.

## 6. Migration stages

Each stage compiles, smoke-tests cleanly (`test_urcu_ft_unit` 185 +
1 pre-existing fail; `test_urcu_ft_inv` 12), and is committed
independently.

### Stage 1 — design + alignment audit (commit `4140980b`)

Audit the alignment of every node class and document the encoding.
No code change.  Audit findings: see §5.

No allocator changes are required.  The Stage 2 flip can land
without bumping any allocation order.

### Stage 2a — direction-aware predicate aliases (commit `60bdc30b`, `9a34a949`)

Add `_in_node` / `_in_slot` aliases for the existing predicates.
Migrate call sites to the alias matching their context.  No behaviour
change under the *current* (4-bit) encoding; sets up the call sites
for the encoding flip.

### Stage 2b — the encoding flip, split into safe sub-stages

The first attempt at the atomic flip (Candidate D) surfaced two
issues:

1. **`ft_resolve_skip_compressed` mixed-context callers**: under the
   new encoding's bit-1=skip semantics, calling resolve on an
   already-resolved COMPRESSED value would erroneously treat
   COMPRESSED as a SKIP_X and trip the `ft_compressed_node_ptr`
   assertion.  Fixed by sub-stages 2b.1 / 2b.2 / 2b.3 below.
2. **Pre-existing parent-kind-boundary race** in compressed-split
   (the SKIP_X demote bug) which the encoding-flip experiment exposed
   but did not cause.  Fixed by commit `9dfff634` (independent of
   the encoding choice).

Sub-stages:

#### Stage 2b.1 (commit `84970de2`) — descent helpers normalise `d.nf`

`ft_descent_init` and `ft_descent_traverse_compressed` end with a
post-step `ft_resolve_skip_compressed` so callers see a normalised
node-context value.  Outer descent loops drop their own resolves.

#### Stage 2b.2 (commit `2437c573`) — non-descent helpers normalise at `cn->child` reads

`ft_inequality_compressed`, `ft_inequality_minmax_compressed`,
`ft_lookup_nth_compressed`, `ft_lookup_nth_last_compressed` resolve
SKIP_X at every `cn->child` read.

#### Stage 2b.3 (commit `c9019676`) — alignment-aware kind extraction

`FT_KIND_MASK_INTERNAL = 0x1F` is added.  Sites that have already
established the value is a direct internal kind use the 5-bit mask;
slot-context sites continue to use `FT_KIND_MASK = 0x0F`.

#### Stage 2b.4 — encoding flip to Candidate E (this stage, redo)

This is the atomic encoding change.  In a single commit:

1. Update `enum ft_kind` values:
   - `FT_KIND_PIGEON` 0x9 → 0x01
   - `FT_KIND_COMPRESSED` 0x1 → 0x04
   - `FT_KIND_SKIP_PIGEON` 0xB → 0x03
   - `FT_KIND_QP` keeps 0x05
   - `FT_KIND_SKIP_QP` keeps 0x07
   - `FT_KIND_EXT` keeps 0x00
   - `FT_KIND_SKIP_EXT` keeps 0x02

2. Update `FT_KIND_MASK` from 0x0F to 0x1F (slot-context).
   `FT_KIND_MASK_INTERNAL` remains 0x1F (now equal — safe to fold).

3. Update `ft_node_ptr` to alignment-aware mask
   (`((v & 0x01) == 0) ? ~15UL : ~31UL`).  `ft_node_ptr_internal`
   stays at `~31UL` unconditionally.

4. Update `ft_compressed_node_flag` to OR `0x04` (was `0x01`).
   Update `ft_compressed_node_ptr` to subtract `0x04` (was `0x01`).
   Both still SUB-friendly for the prefetcher's stride detector.

5. Update `ft_node_compressed_in_node` and `_in_slot` to match
   `(x & 0x07) == 0x04`.  The `_in_node` / `_in_slot` split is no
   longer needed for disambiguation; both predicates now share an
   identical body.  Keep the names for caller-intent documentation.

6. Update `ft_node_skip_compressed_in_slot` to match all SKIP_X via
   bit 1.  The encoding doesn't change here; this is just a doc
   update in the predicate body.

7. Update `ft_node_external` from `(x & 0x01) == 0` to
   `(x & 0x05) == 0`.  COMPRESSED also has bit 0 = 0; bit 2 must be
   clear too to distinguish.

8. `FT_KIND_PIGEON_FAMILY_BIT` (currently `0x8`, matching the high
   bit of PIGEON 0x9 / SKIP_PIGEON 0xB) is no longer meaningful —
   PIGEON has bits 2-4 = 0, which is the *absence* of any class bit.
   Drop the macro; rewrite call sites to test `(x & 0x1C) == 0` (no
   class bits set, indicating PIGEON or SKIP_PIGEON).

9. `FT_KIND_INTERNAL_BITS` (currently `0xC`, "is QP or PIGEON" via
   bits 2-3) is no longer meaningful: QP has bit 2, PIGEON has bits
   2-4 = 0, POPCOUNT_X have bits 3 or 4.  The new "is internal
   non-compressed" predicate is `(x & 0x03) == 0x01` (bit 0 set, bit
   1 clear).  Inline this at every call site; drop the macro.

10. Audit `(tag & 0xF)` extractions.  Slot-context sites that read
    a 16-B-aligned value (could be EXT, SKIP_EXT, COMPRESSED) need
    to dispatch on bit 0 first to avoid the bit-4 address leak;
    direct-internal sites use `(tag & 0x1F)` cleanly.  This was done
    proactively in 2b.3.

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
   plan; the encoding piece is just adding the enum values (already
   valid under Stage 2's `FT_KIND_MASK = 0x1F`).

3. Add descent-dispatch arms.

4. Add SKIP_X publish/resolve handling for the new kinds.

This is properly Step 4 of the parent plan; it lands as its own
commit(s) after Stage 2.

### Stage 4 — drop legacy 4-bit constants and audit

After Stages 2-3 settle, look for any remaining direct comparisons
against literal tag values (e.g. `tag == 0x9` for PIGEON) that the
mass migration may have missed.  Also audit:

- `FT_KIND_PIGEON_FAMILY_BIT` — should be removed in Stage 2b.4; if
  any call site still uses it, fix.
- `FT_KIND_INTERNAL_BITS` — same.
- Any place that tests `(tag & 0xF)` instead of `(tag & 0x1F)` or
  `(tag & 0x07)` — these would silently mis-classify POPCOUNT_64
  (`0x11`) as PIGEON (`0x01`) since the high bit is masked off.

## 6.5. Stage 2b discovery (2026-05-08): SKIP_PIGEON / COMPRESSED ambiguity (resolved by Candidate E)

A first attempt at the atomic encoding flip (under Candidate D)
surfaced a deeper issue than the original plan accounted for.  The
plan assumed call sites that consult bit 1 (`FT_KIND_SKIP_BIT`) could
be left alone — under the old encoding, COMPRESSED's tag (`0x1`) had
bit 1 clear, so `(node & 0x02) != 0` cleanly matched only SKIP_X
kinds.  Under Candidate D's new encoding, COMPRESSED (`0x03`) and
SKIP_PIGEON (`0x03`) share both bits 0 and 1, and *every* SKIP_X
variant has bit 1 set.  Plain `bit 1` no longer distinguishes
"SKIP-encoded slot value" from "COMPRESSED resolved value".

The direction-aware predicate aliases landed in `60bdc30b` /
`9a34a949` already encode the right intent:
`ft_node_skip_compressed_in_slot` is for slot-context callers (where
COMPRESSED can't appear, so bit 1 = SKIP), and
`ft_node_compressed_in_node` is for node-context callers (where
SKIP_X can't appear, so bit 1 = COMPRESSED).  But there was a class
of call sites that operated on *mixed* contexts — values that might
be slot-encoded (just read from a parent slot) or node-context
(returned from `ft_resolve_skip_compressed` earlier in the same
path) — and those sites could not decide from the bits alone.

The most prominent example was `ft_resolve_skip_compressed` itself.
Several callers passed `node_flag` whose context was mixed: it might
be a freshly-read slot value (SKIP_X candidate) or a value left from
a prior resolve (COMPRESSED candidate).  Under the old encoding the
resolve was a no-op for COMPRESSED (bit 1 clear), so callers could
call it freely.  Under Candidate D, calling resolve on COMPRESSED
erroneously fired `ft_skip_to_compressed`, which then tried to
recover the cn from the compressed pointer's `prev` (a parent which
is *not* itself compressed) and tripped the `ft_compressed_node_ptr`
assertion.

### Why Candidate E resolves this

Candidate E moves COMPRESSED to `0x04` (bit 2 set, bit 1 clear).  It
no longer collides with SKIP_PIGEON.  `(x & 0x02) != 0` now correctly
identifies "SKIP_X" in *any* context (mixed, slot, or node-context)
because COMPRESSED has bit 1 clear.  `ft_resolve_skip_compressed`
becomes safe to call on mixed-context values again — calling it on a
COMPRESSED value is correctly a no-op.

The sub-stage 2b.1 / 2b.2 / 2b.3 work landed regardless: the
normalisation at descent-helper boundaries and the alignment-aware
kind extraction are improvements independent of the encoding choice.

## 7. Risks and rollback

The Stage 2b.4 flip is the high-risk commit.  Rollback strategy:

- Each stage is a clean revert if needed.  Stage 2b.4 is a single
  commit by design; `git revert` recovers the 4-bit encoding.
- Smoke tests (`test_urcu_ft_unit` 186, `test_urcu_ft_inv` 12) must
  pass after Stage 2b.4.  Any failure means either a missed call site
  or a predicate bug.
- The longer-duration `test_urcu_ft_inv` runs (60 s, 12-readers /
  4-writers) should be re-run after Stage 2b.4 to surface any
  concurrent-path regression in the SKIP-resolve code.
- A 32-bit build was never enabled for skip-compressed historically
  (memory `project_ft_skip_compressed_32bit.md`); the 5-bit encoding
  is fine on 64-bit but tag bits compete with address bits even more
  on 32-bit pointers.  Confirm 32-bit build still compiles after
  Stage 2b.4 (it should, since the tag bits remain in the low 5 bits
  which stay free under 32-byte alignment).

## 8. Open questions

1. **`ft_node_internal` re-definition.**  Today the predicate matches
   QP / PIGEON / SKIP variants thereof — basically "anything with
   bit 0 set and not COMPRESSED".  Under Candidate E, the strict
   "non-compressed internal" test is `(x & 0x03) == 0x01`.  But the
   old "anything with bit 0 set" test (`(x & 0x01) != 0`) is wider —
   it matches COMPRESSED too (since COMPRESSED also has bit 0 set
   under the old 4-bit encoding).  Under Candidate E this widening
   *ceases* to apply: COMPRESSED has bit 0 = 0.  Audit each call
   site of `ft_node_internal` to determine which semantic the caller
   wants — this gets simpler under Candidate E.

2. **`ft_node_type_index`.**  Today this returns the `ft_types[]`
   slot index for an internal node (QP T0..T3 = 0..3; PIGEON = 4).
   Under the new layout, the slot index is recoverable from
   `cds_ft_item_order(node) - FT_QP16_T0_ALLOC_ORDER` for QP and a
   constant `4` for PIGEON; POPCOUNT_X get new indices.  The
   function stays but its body gets a switch on `ft_kind_of(x)`
   instead of bit math.

3. **Reserved bit patterns.**  `0x06`, `0x08`, `0x0A`, `0x0C`-`0x0F`,
   `0x10`, `0x12`, `0x14`-`0x1F` are unused in Candidate E.  Some
   are reachable only via bit-manipulation bugs and should
   `__builtin_unreachable` / `assert(0)` in dispatch defaults.
   Others (e.g. `0x10`, "EXT pointer with address bit 4 set") are
   address bits for external pointers and must be handled by the
   alignment-aware mask in `ft_node_ptr`.  Document the reserved
   set in `fractal-trie-internal.h`.

4. **Symmetric `_in_node` / `_in_slot` predicates for COMPRESSED.**
   Now that the bodies are identical, should the aliases be merged
   into a single `ft_node_compressed`?  Candidate E removes the
   technical reason for the split; keeping them aids documentation
   of caller intent but adds maintenance.  Recommend: keep both as
   forwarding wrappers in the header, with a comment noting the
   bodies converged with Candidate E.
