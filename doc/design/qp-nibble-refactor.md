# Fractal Trie — QP-nibble Refactor Design Doc

**Status**: WIP, branch `fractal-trie-dev-qp-wip`.

This doc captures the QP-nibble refactor design so future sessions can pick up the work without re-litigating decisions. It is *not* an introduction to the FT — read `src/fractal-trie-internal.h`, `src/fractal-trie.c`, and the `FT_PIGEON` / `FT_POPCOUNT` paths first.

## 1. Goal

Replace the byte-keyed `FT_LINEAR` / `FT_POPCOUNT` / `FT_POOL` internal node types with a uniform pair: **QP-nibble** (16-bit popcount + popcount-indexed `ptrs[]`) for sparse byte levels, and **PIGEON** (existing direct `ptrs[256]`) for dense byte levels. External and compressed node types are kept.

Final taxonomy:
- **External** — leaves (unchanged).
- **Compressed** — skip-edge with stored bytes (unchanged).
- **Skip-compressed pointers** — bypass compressed nodes at descent on supporting archs (unchanged; checked only at byte boundaries).
- **QP-nibble** — sparse byte levels: a *byte step* uses two chained QP-16 nodes (hi nibble, lo nibble), each independently FT-allocated with its own metadata.
- **PIGEON** — dense byte levels: single 256-slot direct-indexed node (one hop per byte).

## 2. Scaffolding done (commits on `fractal-trie-dev-qp-wip`)

| Commit | Topic |
|---|---|
| `a2b88635` | `struct cds_ft_qp16_node` + tier macros, `ft_qp16_node_get_nth` |
| `ec25abc9` | Read-side helpers: `get_direction`, `get_ith_pos`, `get_extremum` |
| `e19c30ec` | Writer helpers: `init`, `set_nth_safe`, `clear_nth`, `cow_insert`, `recompact` |
| `72d752c9` | `FT_QP` enum value + parallel `ft_qp16_tiers[]` table |
| `a4922dc0` | Bitmap loads switched ACQUIRE → RELAXED (pre-filter, not sync) |
| `190e2397` | `ft_qp16_node_get_nth` reshaped → `ft_qp16_node_descend(..., is_lo_nibble)` with hi/lo prefetch split |

All under `#ifdef FEATURE_FT_QP` (not auto-enabled). Both `build-nocollapse` (no `FT_QP`) and `urcu-build-qp` (`-DFEATURE_FT_QP`) compile clean and pass `test_urcu_ft_unit` (195) + `test_urcu_ft_inv` (12). Nothing in the FEATURE_FT_QP code is reachable from runtime descent yet.

## 3. Encoding contracts (decided)

These are the load-bearing decisions. Don't re-debate them.

### 3.1 Pointer encoding

- **Hi-nibble slots** store the lo-nibble `cds_ft_qp16_node *` **raw and untagged**. Reader casts directly; no tag-bit work in descent.
- **Lo-nibble slots** store a **tagged** `cds_ft_inode_flag *` (external / compressed / internal / skip-compressed). Reader inspects tag bits at the byte boundary to dispatch.
- **Compressed and skip-compressed pointers only appear at byte boundaries** — never between hi and lo of the same byte step.
- **External node pointers only at byte boundaries** — same reason.

### 3.2 Memory ordering

- Bitmap is a **pre-filter**: relaxed-loaded by readers, relaxed-stored by writers (single-writer or under lock).
- Slot pointer is the **source of truth**: acquire-loaded by readers (via `ft_dereference_acquire`), released by writers (via `rcu_assign_pointer`).
- **Acquire on the slot is required on both hi and lo descent** — pairs with the count-based readers' undercount guarantee. Writer does the `nr_keys--` decrement BEFORE `rcu_assign_pointer` release at removal, so a reader that observes a detached pointer also observes the preceding `nr_keys` update on weakly-ordered architectures.
- The hi/lo flag selects the **prefetch path only**:
  - lo: `ft_dereference_acquire_prefetch_hint(*slot, pf_hint)` — acquire + `ft_maybe_prefetch_hint` (tag-bit dispatch may skip prefetch for compressed children).
  - hi: `ft_dereference_acquire(*slot)` + unconditional `__builtin_prefetch` (slot known untagged, no tag work).

### 3.3 Mutation model

- **Bitmap is monotonic**: bits are set on insert, never cleared on delete.
- **Delete = tombstone**: `clear_nth` NULLs the slot via `rcu_assign_pointer`; bit stays set. Bitmap therefore acts as a "may be present" hint; slot value is ground truth.
- **Re-insert revives a tombstone**: if bit set and slot NULL, `set_nth_safe` just `rcu_assign_pointer`s the slot — no bitmap change.
- **In-place insert is safe-append only**: bit not currently set + every set bit is < this nibble. `bm |= bit` (relaxed) FIRST, then `rcu_assign_pointer` slot (release). The slot at `popcount(bm)` was zero from allocator or prior tombstone, so concurrent readers see either NULL (not-found) or new child.
- **Non-safe-append insert returns -ERANGE** → caller CoWs at same tier (preserves tombstones).
- **Capacity exhaustion (popcount == capacity, bit not set) returns -ENOSPC** → caller does **recompact-first-then-tier-up**: try CoW recompact at same tier (drops tombstones); if live count still ≥ tier capacity, CoW recompact at next tier. The CoW also inserts the new (nibble, child) in the same walk.

### 3.4 Memory layout

- Each `cds_ft_qp16_node` is independently FT-allocated and has its own `cds_ft_metadata` (parent pointer, `nr_keys`, etc.).
- A "byte level" of the trie is therefore physically (1 hi-node) + (up to 16 lo-nodes) — `nr_keys` of distinct allocations. Hi-node's parent slot points to it tagged-internal (byte boundary); hi-node's own slots point untagged at lo-nodes; lo-nodes' slots point tagged (byte boundary again).

### 3.5 Tier sizing (current draft, tunable)

| Tier | Capacity | Order | Size | min_child (hysteresis) |
|---|---|---|---|---|
| T0 | 3 | 5 | 32 B | 1 |
| T1 | 7 | 6 | 64 B | 2 |
| T2 | 15 | 7 | 128 B | 5 |
| T3 | 16 | 8 | 256 B | 11 |

8-byte header (16-bit bitmap + 6-byte pad), then `ptrs[]`. T0/T1 fit bitmap + ptrs in a single 64 B cache line.

## 4. Remaining work

### 4.1 Step 1b — `ft_types[]` replacement (one commit)

Under `FEATURE_FT_QP`, replace the 8-entry table with:
```
[0..3] = QP T0..T3 (type_class = FT_QP)
[4]    = PIGEON (unchanged)
[5]    = NULL sentinel
```

LINEAR / POPCOUNT / POOL entries disappear. Their `case` arms in dispatch switches become unreachable but are kept for the no-FT_QP build.

This must land **atomically with the `case FT_QP:` arms** (step 2), otherwise the allocator picks an FT_QP entry and the dispatch hits `default: assert(0)` at first allocation.

### 4.2 Step 2 — dispatch arms

Sites (line numbers from current HEAD):

| Site | Function | Notes |
|---|---|---|
| ~5511 | `_ft_node_get_nth` (or whatever the dispatch hub is) | byte→hi+lo chain |
| ~5398 | `_ft_node_set_nth` | byte insert |
| ~6193 | `_ft_node_replace_ptr` | graft replace |
| ~6377 | `ft_node_get_direction` | directional lookup |
| ~6580 | `ft_node_get_first` (extremum) | iter start |
| ~6985 | recompact macro | needs FT_QP arm |
| ~7006 | (review when wiring) | |
| ~7316 | iter / verify | |
| ~17552 | debug walk | |

Each arm chains hi→lo via byte-step wrappers (see 4.3). Reduce the count by introducing a single `ft_node_qp_byte_*` helper that does the chain and is called from each dispatch site.

### 4.3 Byte-step wrappers (new helpers)

```c
struct cds_ft_qp16_node *ft_qp_byte_descend_to_lo(
    struct cds_ft_qp16_node *hi, uint8_t byte);
struct cds_ft_inode_flag *ft_qp_byte_get(
    struct cds_ft_qp16_node *hi, uint8_t byte,
    enum ft_pf_target pf_hint);
int ft_qp_byte_set(...);              /* lazy lo-node alloc on first hi-bit set */
int ft_qp_byte_replace(...);
int ft_qp_byte_clear(...);
struct cds_ft_inode_flag *ft_qp_byte_get_direction(...);
struct cds_ft_inode_flag *ft_qp_byte_get_ith_pos(...);
struct cds_ft_inode_flag *ft_qp_byte_get_extremum(...);
```

The `byte_set` insert path:
1. `descend` hi-nibble. If lo-node exists → reuse. Else allocate fresh lo-node, init, set lo-bit + assign child. Then link lo-node from hi-node via `set_nth_safe(hi, hi_nibble, lo_node, hi_capacity)` — passing the **untagged** `lo_node` directly. May return -ERANGE/-ENOSPC; recurse to CoW/recompact at hi level.
2. lo-bit insert in the lo-node uses `set_nth_safe` again (with tagged child).

`byte_clear` is asymmetric — clearing a lo-bit might leave the lo-node empty (live-count 0). Decision needed (see 4.6).

### 4.4 Hi/lo flag for the other read helpers

`get_direction`, `get_ith_pos`, `get_extremum` currently return `cds_ft_inode_flag *` without distinguishing hi vs lo. When they're called from byte-step wrappers, add the `is_lo_nibble` parameter and apply the same prefetch split as `descend`.

### 4.5 Allocator integration

- Tier picker for QP nodes consults `ft_qp16_tiers[]` (not `ft_types[]`'s entries — those are still indexed by the existing 3-bit type_index for write paths until we land step 1b).
- Tier-up CoW: allocate at `ft_qp16_tiers[tier+1].order`, build via `recompact_and_insert`, swap parent pointer, RCU-free old.
- New helper `ft_qp16_node_recompact_and_insert(new, src, nibble, child, capacity)`: combines `recompact` (drops tombstones) + insert in a single walk. Saves an alloc on the -ENOSPC path.

### 4.6 Decisions

These were deliberated and settled during the scaffolding phase. Don't relitigate.

1. **Lo-node reclamation on full-tombstone**: when `byte_clear` empties a lo-node (live-child count drops to 0), the writer **NULLs the hi-node's slot pointing at that lo-node** and **RCU-frees the lo-node**. Concretely:
   1. `byte_clear` does the lo-side `clear_nth` (NULL the lo-slot for the dying child).
   2. If `nr_keys` of the lo-node is now 0 (live count, not bitmap popcount), proceed to detach the lo-node:
      - `clear_nth(hi_node, hi_nibble)` — tombstones the hi-slot (NULL the slot, bit stays set per monotonic-bitmap rule).
      - `call_rcu` to free the lo-node after the grace period.
   3. Concurrent readers either resolve the old hi-slot value (old lo-node pointer) and descend into the now-tombstoned lo-node (all lo-slots NULL → returns not-found), or see the new NULL hi-slot value and return not-found immediately. Both correct. The grace period defers the actual `free()` until all readers using the old hi-slot value have exited.

   The lo-node's `nr_keys` (stored in its `cds_ft_metadata`) is the live-child counter to test against zero — bitmap popcount can be > 0 with all slots NULL'd (tombstones-only state).

2. **Skip-compressed pointer placement**: skip-compress encodes byte-aligned hop count in the high pointer bits, so a skip-compressed pointer can only live in a **lo-nibble slot** (byte-boundary). The descent flow is unchanged from the pre-QP design:
   1. hi descend → lo-node (untagged).
   2. lo descend → tagged ptr. Tag = skip-compressed → resolve via existing `ft_resolve_skip_compressed`.

   The QP hi/lo split adds an unconditional hi hop in front of each byte boundary but does not affect skip-compress encoding or resolution. No new handling needed; `ft_resolve_skip_compressed` lands on the next byte's hi-nibble node by construction.

3. **Compressed node child placement**: same rule as skip-compress — compressed children are byte-boundary objects, so they live exclusively in **lo-nibble slots**. No hi-side handling needed.

### 4.7 Remaining design questions

1. **PIGEON ↔ QP transitions**: when a sparse byte level grows past T3 capacity (16 distinct nibbles, hi-node fully populated), the writer must transition from QP to PIGEON. Likely path:
   - Detect at hi-node `set_nth_safe` returning -ENOSPC with `popcount == 16` (T3 maxed and bit not set is impossible since bm == 0xFFFF). Real trigger: hi-node at T3 + each lo-node at T3 = 16×16 = 256 distinct bytes. At that point, allocate a PIGEON node, walk all 256 byte values, set each in PIGEON, swap parent pointer, RCU-free hi+all 16 lo-nodes.
   - Reverse direction (PIGEON shrink to QP) on byte-level delete below threshold. Less critical; could defer.

2. **Recompact policy**: tunable trigger (live ≤ X% of capacity → recompact even before -ENOSPC) is a future perf optimization. Initial implementation: only on -ENOSPC.

## 5. Smoke / bench plan (Phase 3)

- `urcu-build-qp`: `test_urcu_ft_unit`, `test_urcu_ft_inv` must pass post-step-2.
- `build-nocollapse`: must continue passing throughout (no FT_QP build is the regression-safe path).
- Bench (post step 2): compare against the post-Phase-1 `build-nocollapse` baseline on the existing FT bench corpus (`u32s`, `u64s`, `dict`, `paths`, `dns`, `load-names`).
- Specific to watch: the byte-step 2-hop cost (QP) vs single-hop PIGEON for sparse vs dense workloads. Doubled internal node count means ~2× metadata loads in the worst case.

## 6. References

- `tests/regression/test_urcu_ft_inv.c` — invariant smoke (12 tests).
- `tests/unit/test_urcu_ft_unit.c` — full unit smoke (195 tests).
- Existing `FT_PIGEON` (`ft_pigeon_node_*` family in `fractal-trie.c`) — closest existing analog: bitmap as pre-filter, slot as ground truth, in-place mutations with `rcu_assign_pointer`.
- Existing `FT_POPCOUNT.nibble_popcount_2l` (`ft_popcount_node_set_nth`) — the in-place safe-append vs `-ERANGE → recompact` model is borrowed from there.
- Memory: `density_bug_investigation.md`, `project_ft_collapse_*` series for prior structural-change incidents.
