# Fractal Trie — Tag-Bit Layout Refactor

**Status**: WIP plan, branch `fractal-trie-dev-qp-wip`. Follows
the FT_POOL / FT_POPCOUNT / FT_LINEAR removals (commits
`7e63175a`, `330d0c56`, `dc6208d5`) and the `FEATURE_FT_QP` gate
drop (`75a5897a`).

This document captures the plan for replacing the current 4-bit
tag layout (1-bit internal, 1-bit compressed, 3-bit type-index)
with a kind-encoded 4-bit nibble. Read
`src/fractal-trie-internal.h` (current macros) and
`src/fractal-trie.c` `ft_node_*` predicate / constructor family
first.

## 1. Goal

Replace the existing tag layout:

```
bit 0 = FT_INTERNAL_MASK       (1 = internal, 0 = ext / compressed)
bit 1 = FT_COMPRESSED_MASK     (when internal=0: 1 = compressed cn ptr)
bits 1..3 = FT_TYPE_MASK       (when internal=1: 3-bit type-index)
```

with a flat **kind-encoded** 4-bit nibble. The new layout has 8
defined kinds; the remaining 8 nibble values are reserved.

| Tag (low nibble) | Kind | Pointer points at |
|---|---|---|
| `0x0` | `EXT`           | `struct cds_ft_node`  (raw external chain head) |
| `0x1` | `COMPRESSED`    | `struct cds_ft_compressed_node`  (the cn itself) |
| `0x2` | `SKIP_EXT`      | (skip-compressed; resolved target is EXT) |
| `0x3` | `SKIP_QP`       | (skip-compressed; resolved target is QP_HI) |
| `0x5` | `QP_HI`         | `struct cds_ft_qp16_node`  (hi-nibble, all tiers) |
| `0x9` | `PIGEON`        | `struct cds_ft_inode`     (pigeon node) |
| `0xB` | `SKIP_PIGEON`   | (skip-compressed; resolved target is PIGEON) |
| `0xD` | `QP_LO`         | `struct cds_ft_qp16_node`  (lo-nibble, all tiers) |

Reserved (will assert / `__builtin_unreachable`): `0x4`, `0x6`,
`0x7`, `0x8`, `0xA`, `0xC`, `0xE`, `0xF`.

The skip-compressed *length* still lives in the high pointer
bits at `[FT_SKIP_LEN_SHIFT, 63]` (unchanged).

### 1.1 Why this layout

1. **Drop the 3-bit type-index entirely.** Today bits 1..3 hold
   a type-index used to load `&ft_types[idx]` for tier sizing.
   Under QP-only, the four hi-tiers (T0..T3) all share one
   descent helper (`ft_qp_byte_get`) and one set_nth helper
   (`ft_qp_byte_set`) — the only place that consumes the tier
   number is allocator pricing, which already has
   `cds_ft_item_order()`. The type-index is dead weight on the
   hot path.

2. **Encode skip-target class in the tag.** Today
   `ft_resolve_skip_compressed` reads the cn's metadata to learn
   what kind of node the skip leads to (the load is
   parallelizable but not free). Putting the target class in the
   tag bits lets the descent dispatch fold the resolve and the
   class-check into one branch.

3. **Symmetric kind dispatch.** Every distinct read-side
   branch (external chain walk, compressed key compare,
   skip-resolve, qp byte-step, pigeon byte-step) is one tag
   value. No more bit-0 / bit-1 / type-index combo predicates
   stacked across the descent loop.

### 1.2 Bit-pattern rationale

Bit 0 distinguishes the **two NULL-equivalent kinds**: `0x0`
(EXT) and `0x2` (SKIP_EXT) both have bit 0 clear. Combined with
the existing convention that `node == NULL` reads as `0x0`
(empty slot), `(tag & 1) == 0` cleanly separates "external or
no-tag" from everything else. This is the single fast-path
fall-through in descent: not bit-0 → done, fall through to
external / NULL handling.

Bits 2..3 encode the resolved node class (independent of
"is skip"):

| bits 3..2 | Class |
|---|---|
| `00` | EXT or COMPRESSED (the byte-boundary classes) |
| `01` | QP_HI |
| `10` | PIGEON |
| `11` | QP_LO |

Bit 1 is the **skip-compressed indicator** when bit 0 is set:

- `(tag & 0x3) == 0x1` → direct (COMPRESSED, QP_HI, PIGEON, QP_LO)
- `(tag & 0x3) == 0x3` → skip-compressed (SKIP_QP, SKIP_PIGEON,
  and SKIP_EXT — note SKIP_EXT has bit 0 clear; see below)

`SKIP_EXT = 0x2` is the single asymmetry: bit 0 clear because
the resolved target is external. `(tag & 0x2) != 0` is the
universal "is skip-compressed" predicate.

Net result: descent decides

```c
if (caa_unlikely((tag & 0x2)))           goto skip_resolve;  /* 0x2/0x3/0xB */
if (caa_unlikely((tag & 0x1) == 0))      goto external_or_null; /* 0x0 */
switch ((tag >> 2) & 0x3) {
    case 0x0: goto compressed;           /* 0x1 */
    case 0x1: goto qp_hi;                /* 0x5 */
    case 0x2: goto pigeon;               /* 0x9 */
    case 0x3: goto qp_lo;                /* 0xD */
}
```

`tag & 0x2` covers all three skip-compressed kinds; the
post-resolve dispatch then re-reads bits 2..3 of the resolved
pointer.

## 2. Scope of change

### 2.1 Macros in `src/fractal-trie-internal.h`

Removed:

- `FT_INTERNAL_BITS` (was `1`)
- `FT_INTERNAL_MASK` (was `(1U << 0)`)
- `FT_COMPRESSED_MASK` (was `(1U << 1)`)
- `FT_TAG_MASK` (was `0b011`)
- `FT_TYPE_BITS` (was `3`)
- `FT_TYPE_MAX_NR` (was `1UL << 3`)
- `FT_TYPE_MASK` (was `((8 - 1) << 1) = 0b1110`)
- `FT_PTR_MASK` (was `~(FT_TYPE_MASK | FT_INTERNAL_MASK) = ~0xF`)
- `FT_QP_LO_TYPE_INDEX` (was `5U`)
- `FT_QP16_NR_TIERS` (was `4U`)  — keep but unused at descent;
  still consulted by allocator. Re-evaluate after Section 4.

Added:

```c
#define FT_TAG_BITS         4U
#define FT_TAG_MASK         0xFUL  /* low nibble of the pointer */
#define FT_TAG_PTR_MASK     (~FT_TAG_MASK)

/* 4-bit kinds, low-nibble values. */
enum ft_kind {
    FT_KIND_EXT         = 0x0,
    FT_KIND_COMPRESSED  = 0x1,
    FT_KIND_SKIP_EXT    = 0x2,
    FT_KIND_SKIP_QP     = 0x3,
    FT_KIND_QP_HI       = 0x5,
    FT_KIND_PIGEON      = 0x9,
    FT_KIND_SKIP_PIGEON = 0xB,
    FT_KIND_QP_LO       = 0xD,
};

/* Compound predicates (single-AND, single-CMP). */
#define FT_TAG_IS_SKIP_BIT      0x2UL  /* set on 0x2/0x3/0xB */
#define FT_TAG_INTERNAL_NODE_BIT 0x1UL /* set on 0x1/0x3/0x5/0x9/0xB/0xD; clear on 0x0/0x2 */
```

`ft_types[]` keeps its 5-entry layout (T0..T3 + PIGEON) — its
*storage* purpose (per-tier `order`, `min_child`, `max_child`,
`bitmap`) is unchanged. It is *not* indexed off tag bits anymore;
descent never loads `&ft_types[type_index]` on the hot path.
Writers / verify code use the alloc order to recover the tier:

```c
static inline unsigned int ft_qp_tier_from_node(struct cds_ft_inode *node)
{
    return (unsigned int) cds_ft_item_order(node) - FT_QP16_T0_ALLOC_ORDER;
}
```

### 2.2 Predicates and constructors in `src/fractal-trie.c`

Renamed / rewritten:

| Today | New |
|---|---|
| `ft_node_external(nf)` (`tag == 0x0`) | unchanged predicate, but matches `0x0` only — re-derive with `(tag & FT_TAG_MASK) == 0` |
| `ft_node_internal(nf)` (`tag & 0x1`) | replaced by `ft_kind_of(nf)` switch |
| `ft_node_compressed(nf)` (`tag == 0x2`) | now `ft_kind_of(nf) == FT_KIND_COMPRESSED` (`tag == 0x1`) |
| `ft_node_skip_compressed(nf)` (high-bits != 0) | unchanged; high-bit length still encodes "is skip" |
| `ft_node_type(nf)` (extract 3-bit type-index) | **removed**; tier from `cds_ft_item_order(node)` |
| `ft_node_flag(node, type)` (constructor) | replaced by per-kind constructors: `ft_qp_hi_flag`, `ft_pigeon_flag` (and `ft_qp16_lo_flag` already exists, retagged) |
| `ft_compressed_node_flag(cn)` (`tag = 0x2`) | new tag `0x1`; mask change |
| `ft_skip_compressed_flag(child, len)` | now consults child's kind to pick `SKIP_EXT` / `SKIP_QP` / `SKIP_PIGEON`; tag bits become part of the flag (today they came from `child`'s tag verbatim) |

New helper:

```c
static inline enum ft_kind ft_kind_of(struct cds_ft_inode_flag *nf)
{
    return (enum ft_kind) ((unsigned long) nf & FT_TAG_MASK);
}
```

`ft_node_ptr` and `ft_node_ptr_internal` simplify to a single
mask:

```c
static inline_lookup
struct cds_ft_inode *ft_node_ptr(struct cds_ft_inode_flag *nf)
{
    unsigned long v = (unsigned long) nf;
#ifdef FEATURE_FT_SKIP_COMPRESSED
    v &= FT_ADDR_MASK;
#endif
    return (struct cds_ft_inode *) (v & FT_TAG_PTR_MASK);
}
```

The `(v & 1) ? ~15UL : ~7UL` branch goes away — every kind now
clears the same low 4 bits.

### 2.3 Skip-compressed encoding

Today: `ft_skip_compressed_flag(child, len)` ORs `len` into
`child`'s high bits and copies `child`'s low tag bits verbatim.

New: the skip pointer's low tag is one of `0x2 / 0x3 / 0xB`,
*derived from the child's kind*:

```c
static inline
struct cds_ft_inode_flag *ft_skip_compressed_flag(
        struct cds_ft_inode_flag *child, unsigned int len)
{
    enum ft_kind ck = ft_kind_of(child);
    enum ft_kind sk;

    switch (ck) {
    case FT_KIND_EXT:    sk = FT_KIND_SKIP_EXT; break;
    case FT_KIND_QP_HI:  sk = FT_KIND_SKIP_QP; break;
    case FT_KIND_PIGEON: sk = FT_KIND_SKIP_PIGEON; break;
    default: assert(0); __builtin_unreachable();
    }
    return (struct cds_ft_inode_flag *)
        (((unsigned long) child & FT_TAG_PTR_MASK) | sk |
         ((unsigned long) len << FT_SKIP_LEN_SHIFT));
}
```

Constraint: a skip target must be one of EXT / QP_HI / PIGEON.
The "compressed → compressed → ..." chain canonicalization
(already an invariant — see `ft_skip_compressed_flag` assert)
keeps cn->child outside skip-compressed encoding, but does not
guarantee cn->child is non-COMPRESSED. The current code allows
`cn->child` to be a tagged compressed-node pointer in transient
states; under the new layout we'd need either:

1. A post-canonicalization invariant `cn->child ∈ {EXT, QP_HI,
   PIGEON, QP_LO}` (no `COMPRESSED`), enforced at every
   `cn_install` site. **This is the proposed direction** — it
   matches the existing chain-compress design intent (no two
   adjacent compressed nodes).
2. Or, keep one more skip variant `SKIP_COMPRESSED = 0x6` (uses a
   reserved tag) — at the cost of one extra dispatch arm.

Option 1 is cheaper and aligns with the chain-compress
canonicalization already in place. The migration commit will
land an `assert(ft_kind_of(cn->child) != FT_KIND_COMPRESSED)`
inside `ft_skip_compressed_flag` to surface any latent
violation.

### 2.4 QP_LO retagging

`ft_qp16_lo_flag` today builds:

```c
return (lo | (FT_QP_LO_TYPE_INDEX << 1) | FT_INTERNAL_MASK);
       /* FT_QP_LO_TYPE_INDEX = 5; tag bits = (5 << 1) | 1 = 0xB */
```

That is, currently the lo-flag's low 4 bits are `0xB`. Under
the new layout, `0xB` is `SKIP_PIGEON` — a different kind.
The lo-flag must move to `0xD` (QP_LO):

```c
static inline
struct cds_ft_inode_flag *ft_qp16_lo_flag(struct cds_ft_qp16_node *lo)
{
    return (struct cds_ft_inode_flag *) (((unsigned long) lo) | FT_KIND_QP_LO);
}
```

This is a small textual change but every call site that
*compares* the lo-flag's tag bits (e.g.
`ft_parent_depth_span`'s `ft_node_type(parent_nf) ==
FT_QP_LO_TYPE_INDEX`) needs to compare against
`FT_KIND_QP_LO` instead. The
`ft_set_parent` lo-resolver (`src/fractal-trie.c:1738..1752`)
also needs to swap from `p_type < FT_QP16_NR_TIERS` to
`ft_kind_of(parent_nf) == FT_KIND_QP_HI`.

### 2.5 QP_HI retagging — and the tier picker

Today hi-nodes carry tag bits `0x1..0x7` per-tier (3-bit
type-index encoded in bits 1..3, T0=0..T3=3, plus
`FT_INTERNAL_MASK`). After the refactor every hi-node carries
the same tag `0x5`. The tier still has to be recovered
*somewhere*:

- **Read side**: never. Descent passes the qp16_node pointer to
  `ft_qp_byte_get`, which only reads `bitmap` and `ptrs[]` — no
  tier-dependent dispatch.
- **Write side**: at recompact, where the tier-up / tier-down
  policy lives. Today the tier is read from `type_index` in the
  flag; new code reads it from `cds_ft_item_order(node) -
  FT_QP16_T0_ALLOC_ORDER`. The order is already a free byte in
  the metadata header (set at allocation), so no new load.
- **Verify side** (`ft_verify_node_recursive`): same — recover
  via `cds_ft_item_order`.

`ft_types[]` is consulted at recompact for `min_child` /
`max_child` / `order` / `bitmap`. Indexing remains 0..4 (T0..T3
+ PIGEON), but the index now comes from
`ft_qp_tier_from_node(node)` for QP nodes and a fixed `4` for
PIGEON. This is a one-line change in
`find_nearest_type_index` and friends.

## 3. Migration strategy

This is one logical refactor but it touches every tag check in
the file (~80 sites by current grep), so we land it in stages.
Each stage compiles + smoke-tests cleanly.

### 3.1 Stage A — add new macros in parallel (non-breaking)

Define `enum ft_kind`, `FT_TAG_MASK = 0xF`, `FT_TAG_PTR_MASK`,
and `ft_kind_of()` alongside the existing `FT_INTERNAL_MASK` /
`FT_COMPRESSED_MASK` / `FT_TYPE_MASK`. Implement `ft_kind_of`
to decode today's encoding by inspection (it's a 4-bit value
already; the macro is a pure mask). No call-site change yet.

This commit lands the new vocabulary so subsequent commits can
introduce switch statements that already use it. Smoke tests
verify nothing changed.

### 3.2 Stage B — flip tag value of QP_LO from `0xB` → `0xD`

Single-site change: `ft_qp16_lo_flag` constructor +
`ft_parent_depth_span`'s `FT_QP_LO_TYPE_INDEX` compare + the
`p_type < FT_QP16_NR_TIERS` guard in `ft_set_parent`. All other
sites use `ft_qp16_lo_flag` opaquely.

`FT_QP_LO_TYPE_INDEX` macro stays defined as `5U` (used by
verify code that walks `ft_types[]`); the *tag bits* on the
flag move independently. The decoupling lets QP_LO advance to
its new tag without renaming the type-table slot.

Smoke: tests must still pass; this is a tag-bit rename only.

### 3.3 Stage C — flip QP_HI tag from `0x1..0x7` (per-tier) → `0x5` (uniform)

(Done before the COMPRESSED move because today's QP T0 = `0x1`,
which COMPRESSED needs to occupy.)

This collapses the 3-bit type-index for QP-hi nodes. Sites:

- `ft_node_flag(node, type)` constructor — replace with
  per-kind constructors (`ft_qp_hi_flag`, `ft_pigeon_flag`).
- `ft_node_type(nf)` — replace with `ft_kind_of(nf)` at every
  call site; the tier (when actually needed) comes from
  `cds_ft_item_order(node)`.
- The `ft_node_get_nth_skip` dispatch
  (`src/fractal-trie.c:3669..3690`): `type_index < 4` →
  `ft_kind_of == FT_KIND_QP_HI`; the PIGEON arm becomes
  `FT_KIND_PIGEON`.
- The recompact `find_nearest_type_index` / tier-up logic
  (~`src/fractal-trie.c:4090..4170`): tier is now derived from
  `cds_ft_item_order(old_node) - FT_QP16_T0_ALLOC_ORDER` rather
  than from `old_type_index` directly.
- `ft_verify_node_recursive`'s class check.

`ft_types[]` indexing at recompact uses
`ft_qp_tier_from_node(node)` for QP nodes, `4` for PIGEON.

Smoke: full unit + inv. Likely smallest risk surface — the
read-side dispatch is just substituting one switch arm for
another; the descent helper is unchanged.

### 3.4 Stage D — flip tag value of COMPRESSED from `0x2` → `0x1`

Now that QP_HI no longer occupies `0x1` (Stage C), COMPRESSED
can move there.  Sites:

- `ft_compressed_node_flag` constructor (low-bit OR change).
- `ft_compressed_node_ptr` (mask change: `~0x3` → `~0xF` —
  effectively becomes `& FT_KIND_PTR_MASK`).
- `ft_node_compressed` predicate (`tag == 0x2` → `tag == 0x1`).
- `ft_node_external` predicate (currently `(tag & 0x3) == 0` —
  still matches `0x0`, no change in semantics).
- `ft_node_internal` predicate — needs update: today it's
  `tag & 1`, which under the new layout is *also* set on
  COMPRESSED (`0x1`).  New formula: `(tag & 1) && (tag != 0x1)`,
  or equivalently `ft_kind_of(nf) >= FT_KIND_QP_HI`.

This is the **invasive stage**.  Bit 1 stops being
`FT_COMPRESSED_MASK`; it is now reserved as the future
skip-compressed indicator (used by Stage E).  Every direct test
against `FT_COMPRESSED_MASK` or `FT_TAG_MASK` must be audited.
Plan to grep:

```sh
grep -n 'FT_COMPRESSED_MASK\|FT_TAG_MASK\|FT_INTERNAL_MASK' src/fractal-trie.c
```

and rewrite each one to use `ft_kind_of` + an enum compare.

Smoke: must pass.  Run `test_urcu_ft_inv` with longer durations
(60 s) to surface any concurrent-path regression — the
skip-resolve path is the highest-risk surface.

### 3.5 Stage E — add SKIP_EXT / SKIP_QP / SKIP_PIGEON encoding

Sites:

- `ft_skip_compressed_flag(child, len)` — branches on
  `ft_kind_of(child)` to pick the skip-tag (per Section 2.3).
- `ft_node_skip_compressed(nf)` — unchanged (still tests high
  bits).
- `ft_resolve_skip_compressed(nf)` — today calls
  `ft_compressed_node_flag(ft_skip_to_compressed(nf))`. New
  code can short-circuit using the skip-tag bits when only the
  child class is needed (e.g., the descent dispatch can route
  `SKIP_EXT` straight to external-chain handling without the cn
  metadata load). The simple path keeps the cn-metadata load
  for skip→cn lookups; the optimization is a follow-up.
- The `ft_skip_compressed_flag` `assert(((unsigned long) child >>
  FT_SKIP_LEN_SHIFT) == 0)` stays — no nested skip.
- New invariant assert: `assert(ft_kind_of(cn->child) !=
  FT_KIND_COMPRESSED)` at every cn-publish site to keep skip
  tags well-formed.

This is the stage that *unlocks* the skip-target dispatch
optimization, but the optimization itself is a follow-up — this
stage just makes the encoding faithful.

Smoke: full unit + inv + extended duration. The skip path is
the primary user of these tags; any encoding bug surfaces here.

### 3.6 Stage F — drop `FT_INTERNAL_MASK` / `FT_COMPRESSED_MASK` / `FT_TYPE_MASK`

After Stages A..E every call site uses `ft_kind_of` and the
`enum ft_kind` constants. The legacy macros become dead. Delete
them in one final cleanup commit (along with `FT_TYPE_BITS`,
`FT_TYPE_MAX_NR`, `FT_PTR_MASK`).

This commit also drops `ft_node_type` and removes the
`FT_QP_LO_TYPE_INDEX` macro (the kind is now `FT_KIND_QP_LO`
directly; the type-table slot index is a write-side detail).

Smoke: full unit + inv.

## 4. Risk and rollout

### 4.1 Hot-path bit fiddling

The current descent loop
(`src/fractal-trie.c:3641..3702`, `ft_node_get_nth_skip`) does

```c
unsigned long tag = (unsigned long) node_flag & 0xF;
if (caa_unlikely(!(tag & FT_INTERNAL_MASK))) return NULL;  /* ext */
type_index = (tag >> FT_INTERNAL_BITS) & 0x7;
if (caa_likely(type_index < 4)) return ft_qp_byte_get(...);
return ft_pigeon_node_get_nth(...);
```

Under the new layout the same fast path is

```c
unsigned long tag = (unsigned long) node_flag & FT_TAG_MASK;
switch (tag) {
case FT_KIND_QP_HI:  return ft_qp_byte_get(...);
case FT_KIND_PIGEON: return ft_pigeon_node_get_nth(...);
case FT_KIND_QP_LO:  /* unreachable in this dispatch */ break;
default: return NULL;  /* ext / compressed / skip / nil */
}
```

The compiler should emit a jump-table — same shape as today's
`scan_*` dispatch. The fast-path branch count is unchanged.
`type_index < 4` becomes `tag == FT_KIND_QP_HI`, single CMP.

### 4.2 Concurrent skip-resolve correctness

The skip-resolve path (`ft_resolve_skip_compressed` →
`ft_skip_to_compressed` → `cn` deref) reads `meta->parent` to
identify the cn. Under the new layout the *kind of skip* is in
the tag bits, so the meta load is no longer load-bearing for
class dispatch — but it is still used to find the cn pointer
itself. No memory-ordering change; the existing acquire on the
slot suffices.

Risk: a writer that updates a skip pointer (e.g., chain-compress
re-canonicalization) must publish the new flag bits coherently
with the new high-bit length. `rcu_assign_pointer` already
provides release ordering on the whole pointer — no special
care needed.

### 4.3 Invariant-test coverage

`test_urcu_ft_inv` exercises the skip-compressed path via
`inv_relational_lookup` and `inv_skip_*_stability`. Any tag
mismatch surfaces as a `going_up` assert or a missed key. The
8-reader / 2-writer / 2 s configuration we run after each stage
is sufficient for compile-correctness; for the SKIP_EXT /
SKIP_QP / SKIP_PIGEON encoding (Stage E) bump to
`FT_INV_DURATION_MS=10000` and 12-readers / 4-writers.

### 4.4 Build configurations

The existing `build-bench-noskipcompress` / `build-noqp` /
`build-bench-O3` / `build-bench-noclgate` etc. directories still
exist as artifacts of previous configurations. After stage F,
`build-noqp` and `build-bench-nopopcount` become aliases of the
default — they can be removed in a separate cleanup commit.

The `-DFEATURE_FT_QP` flag still works (no-op after `75a5897a`)
and is dropped from the smoke configs as part of stage F.

## 5. Stage-by-stage commit shape

| Stage | Subject (commit title) | Approx LOC change |
|---|---|---|
| A | Tag-bit refactor: add `enum ft_kind` + `ft_kind_of` (parallel) | +60 / -0 |
| B | Tag-bit refactor: move QP_LO tag from `0xB` to `0xD` | +20 / -10 |
| C | Tag-bit refactor: collapse QP T0..T3 to uniform `QP_HI = 0x5` | +80 / -100 |
| D | Tag-bit refactor: move COMPRESSED tag from `0x2` to `0x1` | +120 / -100 |
| E | Tag-bit refactor: encode skip-target class in tag bits | +60 / -30 |
| F | Tag-bit refactor: drop legacy `FT_INTERNAL_MASK` / `FT_TYPE_MASK` macros | +0 / -100 |

Each stage runs `test_urcu_ft_unit` (187/187) + `test_urcu_ft_inv`
(12/12). Stage E additionally runs the long-duration
inv config. No benchmarks run between stages (per
`feedback_no_bench_for_dead_code_cleanup.md` — the cleanup is
in dead-code territory until Stage E flips skip encoding;
Stage E is the bench-relevant stage and runs the FT bench
suite).

## 6. Open questions

1. **Should COMPRESSED be permitted as `cn->child`?** Stage E
   asserts no. Today's chain-compress canonicalization already
   forbids two adjacent compresseds; verify by audit before
   landing the assert as enforcement.

2. **Reserved tag values for future kinds.** `0x4`, `0x6`,
   `0x7`, `0x8`, `0xA`, `0xC`, `0xE`, `0xF` are unused. Reserve
   `0x6` for a hypothetical `SKIP_QP_LO` if descent ever needs
   to skip across a hi/lo pair (today: not needed —
   skip-compressed only crosses byte boundaries, never lands on
   QP_LO). Document but don't define.

3. **`FT_TAG_MASK = 0xF`** makes the low nibble exclusive to
   tag bits. The current pool encoding (gone with FT_POOL
   removal) used bits 4..order-1 for sub-class indices. Under
   QP-only those bits are address bits again, so there is no
   conflict — but worth re-checking arena alignment to confirm
   the smallest allocation order is `>= 4` (16-byte alignment),
   so bits 0..3 are guaranteed zero in raw addresses. Today
   `FT_ALLOC_ORDER_MIN = 4` (`src/fractal-trie-internal.h:231`)
   — invariant holds.

## 7. References

- `src/fractal-trie-internal.h:53..65` — current tag-bit macro
  stack.
- `src/fractal-trie.c:1071..1078` — `ft_node_flag` constructor.
- `src/fractal-trie.c:1086..1104` — external / compressed
  predicates.
- `src/fractal-trie.c:1162..1185` — `ft_node_ptr` mask logic.
- `src/fractal-trie.c:1245..1311` — compressed / skip
  constructors.
- `src/fractal-trie.c:2609..2615` — `ft_qp16_lo_flag`
  constructor.
- `src/fractal-trie.c:3641..3702` — read-side dispatch hub
  (`ft_node_get_nth_skip`).
- `doc/design/qp-nibble-refactor.md` — parent design (encoding
  contracts and overall layout).
