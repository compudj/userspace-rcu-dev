# Retiring reserve_slot/bind_slot: the reserved-byte publish model (2026-06-24)

Status: IMPLEMENTED on `fractal-trie-dev` (insert one-commit + both NOSPLIT
graft points converted; `reserve_slot`/`bind_slot`/`placed` deleted from the
flip-latch primitive). See §7 for what actually shipped and why the node-layer
parameter in §3 turned out unnecessary. Motivated by the discomfort that
`urcu_flip_txn_reserve_slot` + `urcu_flip_txn_bind_slot` is a single-writer
work-around that hides real problems — borne out by a recompact overflow it
exposed (later shown to be memory corruption in the proxy-into-slot path, which
this model removes outright).

Related: `doc/design/mcas-multiwriter-readiness.md`,
`doc/design/transactional-flip-latch.md`,
`project_ft_invariant1_campaign` memory.

---

## 1. What reserve+bind is, and the three problems with it

A `urcu_flip_txn` commits a set of `{slot, old, new}` edges. Publishing a fresh
child into a popcount node via `ft_node_set_nth` is the awkward edge: the slot's
final address isn't known until `set_nth` runs (it may **recompact and relocate**
the node), and the new child must stay **invisible** until the atomic commit.

`reserve_slot` bridges that by handing `set_nth` a **tagged proxy** (resolving to
old) which `set_nth` stores *into the live node's child array*; `bind_slot`
records the final address afterward; commit settles it. Three problems:

1. **Recompact entanglement (the bug).** The proxy is a foreign, occupied slot in
   the popcount machinery. `metadata->nr_child` tracks *live pointers* and does
   NOT count the proxy (it resolves to old), while recompact **sizes** the new
   node from `nr_child + 1` but **re-inserts** from the bitmap (which DOES include
   the proxy). When the proxy lag exceeds the `+1` budget, the recompact
   under-sizes and overflows (`ft_popcount_2l_node_set_nth: nr_child < max_lc`).
2. **Realloc fragility.** The node slot stores `tag(&latch->proxy)` — a pointer
   *into the txn's chunk array*. A later `record()` that grows the head chunk
   reallocs it, moving the latch and dangling the slot. So `reserve_slot` must
   assert a pre-`reserve`d, non-growing txn — you can't grow the edge set after a
   placement.
3. **MCAS-incompatibility.** It installs a proxy mid-build, in arbitrary order. A
   lock-free MCAS commits by acquiring every slot in **sorted address order**,
   from a **frozen** set, at commit. Eager mid-build install is the opposite — and
   it's the *last* violation of the freeze-before-install invariant that
   `mcas-multiwriter-readiness.md` sets up as the MCAS prerequisite.

The COW alternative (build a fresh node copy + flip the parent edge) fixes 2 and 3
but **kills the in-place fast path** — every append/remove would copy the node.
Rejected: we want to *minimize* recompaction, not force it.

---

## 2. The key observations

- **`nr_child` must count reserved slots.** It is writer-side accounting (readers
  iterate via the bitmap, `get_ith_pos` asserts against `ft_popcount_node_get_nr_
  child`, not `metadata->nr_child`), so it can safely count a reserved-but-invisible
  slot. Counting it makes SIZE (`nr_child`) agree with RE-INSERT (the occupied
  bitmap), which removes the overflow at the source.
- **A bit-set + NULL-pointer slot already reads as "not present."** Documented and
  verified: the per-type read accessors return `pointers[idx]` directly, so a NULL
  pointer passes through as NULL, and descent treats NULL as not-found — the
  "append-in-flight returns NULL (legitimate)" contract (ft-lookup-node.h:86-89,
  scanner e.g. :350). So a slot can be **reserved (bit set) yet hold NULL** for its
  whole lifetime and stay invisible-and-safe, with no marker or proxy occupying it.
- **There is exactly ONE reserved byte per op.** Each reserve+bind site adds a
  single new child: insert one-commit (one byte), both NOSPLIT graft points (one
  byte). The merge interleave doesn't reserve popcount slots (it flips existing
  cell links / a parent edge). So the reservation state is just `{has_reserved,
  reserved_byte}` — no list.

---

## 3. The reserved-byte model

Replace "install a proxy in the slot" with "set the bit, leave the slot NULL, and
tell recompact about the one reserved byte."

- **Reserve** = set the bitmap bit for `n` + `metadata->nr_child++`; leave
  `pointers[idx] == NULL`. Nothing is installed in the slot.
- **`ft_node_set_nth` / `ft_node_recompact` gain `bool has_reserved, uint8_t
  reserved_byte`.** The recompact re-insert keeps `if (!iter) continue` for
  genuinely-empty NULL slots, but when `has_reserved && byte == reserved_byte` it
  **carries the bit anyway** (sets it in the new node, NULL pointer, counts it in
  the new `nr_child`). So the one reserved slot survives relocation with nothing
  occupying it — recompact no longer garbage-collects it.
- **The txn edge is a plain `urcu_flip_txn_record(&final_node->pointers[idx], NULL,
  child)`**, taken *after* `set_nth` (so the node/`idx` are final, post-recompact),
  and **installed at commit** like every other edge — freeze → install all in
  sorted address order → flip → settle to `child`.

Slot lifecycle: `empty → reserved (bit set, NULL, invisible, recompact-survivable)
→ proxy (installed at commit, sorted) → child (settle)`. No proxy or marker in the
slot during the build; no `bind`.

### Why this resolves all three problems

- **Recompact entanglement:** `nr_child` counts the reserved byte, so SIZE matches
  RE-INSERT; recompact carries the reserved byte explicitly. No overflow.
- **Realloc:** the slot holds NULL, not `&latch->proxy`. Nothing in the tree points
  into the txn array during the build, so `record()` can realloc freely. The
  `reserve`/`placed` constraints disappear.
- **MCAS:** nothing installs eagerly; the txn commits the whole frozen edge set in
  sorted address order. Freeze-before-install is restored *everywhere* — this was
  the last hole.

And it keeps **in-place** (no COW), needs **no follow-the-proxy** (the recompact
happens *inside* `set_nth`, the reserved byte is conveyed by parameter, and the
edge is recorded against the final slot *after* — so there is no
record-then-relocate to chase).

---

## 4. Blast radius

- **Add:** a "reserve a byte" capability (set bit + `nr_child++`, NULL slot); the
  `has_reserved, reserved_byte` parameters threaded into `ft_node_set_nth` /
  `ft_node_recompact` so recompact carries the one reservation; the `nr_child++` at
  reserve.
- **Convert:** the three reserve+bind sites — insert one-commit (ft-insert.h),
  both NOSPLIT graft points (ft-graft.h) — to: reserve the byte, then
  `record(final_slot, NULL, child)`. List-off (single edge, no cell) needs no
  deferral at all: reserve + the single store/CAS.
- **Delete:** `urcu_flip_txn_reserve_slot`, `urcu_flip_txn_bind_slot`, the `placed`
  flag and its commit/abort special-cases in `src/urcu-flip-latch.h`. The txn
  returns to pure `{slot, old, new}` edges with stable addresses installed at
  commit.

Independent of: the merge spine-copy interleave (already plain records) and the
ordered-cell point ops (`ft_ord_cell_flip`, stable-address edges).

---

## 5. Plan

1. **`nr_child` at reserve** (standalone, hardening): count the reserved/occupied
   slot. This alone makes the recompact SIZE==RE-INSERT and removes the overflow,
   independent of the rest.
2. **Reserved-byte recompact carry**: thread `has_reserved/reserved_byte`; recompact
   preserves the one NULL reservation.
3. **Convert the three sites** to reserve + record-final-slot; list-off first
   (single edge, smallest, also the path that exposed the bug).
4. **Delete reserve_slot/bind_slot/placed** once no caller remains.
5. Gate each step with the full suite (incl. list-off `test_list_off_ops` +
   `inv_no_ordered_list_consistency`, the merge/graft cross-view oracles, VAM,
   ASAN). MCAS-edge completeness is measured by re-running the edge audit
   (functional tests can't detect a missed conversion).

## 6. Caveats / confirmed

- **Confirmed:** a bit-set + NULL slot reads as not-present in the descent/lookup
  read paths (ft-lookup-node.h:86-89, :350), so the reserved slot is invisible and
  safe for its whole (now longer) lifetime.
- **Relied upon:** edges are recorded against the *post-recompact* final slot, so
  no op records an edge into a node and *then* recompacts that same node (no
  record-then-relocate). True for the single-child reserve+bind sites today; a
  future multi-reserve op would need to widen `{has_reserved, reserved_byte}` and
  reconsider this.
- The recompact must count the carried reserved byte in the new node's `nr_child`
  (so the invariant holds across relocation).

---

## 7. Implementation outcome (2026-06-24)

Shipped on `fractal-trie-dev`. The model works, and it came out **simpler than
§3 designed** — no node-layer signature change was needed.

### The `has_reserved/reserved_byte` parameter was unnecessary

§3 proposed threading `bool has_reserved, uint8_t reserved_byte` into
`ft_node_set_nth` / `ft_node_recompact` so a recompact would carry the one
NULL-reserved byte instead of NULL-skipping it. In practice **`ft_node_set_nth(
node, n, /*child=*/NULL)` already reserves a byte correctly**, with no new
parameter:

- *In place* (safe-append): the popcount setter writes the (NULL) child, sets
  the bitmap bit, and bumps `metadata->nr_child` — i.e. a bit-set+NULL slot that
  reads as not-present. So `nr_child` counts the reserved slot automatically;
  §5 step 1 ("nr_child at reserve") falls out for free.
- *On overflow* (recompact): the reserve `set_nth(n, NULL)` triggers
  `FT_RECOMPACT_ADD_*`, whose **ADD-tail** adds byte `n` with `child=NULL` — so
  the reserved byte is carried by the existing add path, not the re-insert loop.

The re-insert loop's `if (!iter) continue` only ever sees a bit-set+NULL slot
that is a **pre-existing soft-deleted slot** (which it must GC) — never the byte
this op is reserving, because that byte arrives via the ADD-tail (insert/graft)
or is already a live pointer (the displaced-external case). And no converted
site reserves a byte in a node and *then* recompacts that node via a *different*
child, so a recompact never has to carry a previously-reserved NULL slot. Hence
the `has_reserved` carry has no live caller and was not added.

### What shipped

- **Insert one-commit** (`ft_attach_node`, ordered-list ON / armed path):
  - new byte (`old_node_flag == NULL`): `set_nth(dest, key_value, NULL)` to
    reserve, then `ft_flip_txn_record_reserved(slot, NULL, top)`.
  - displaced external (`old_node_flag != NULL`): **no** `set_nth` (the slot
    already holds the external; a NULL store would drop it before the commit);
    record `(slot, old_node_flag, top)`.
  - Ordered-list OFF stays a direct `set_nth(real child)` — never used
    reserve+bind. Routing it through the txn is the separate Invariant-1
    campaign step, not part of retiring reserve+bind.
- **Graft** (both NOSPLIT points, `ft_store_at_graft_point`): both are the fresh
  (`old == NULL`) shape — `set_nth(dest, byte, NULL)` in prepare, then
  `ft_flip_txn_record_reserved(slot, NULL, slot_value)` against the final slot in
  the commit, replacing the `bind_slot`.
- **Deleted** from `src/urcu-flip-latch.h`: `urcu_flip_txn_reserve_slot`,
  `urcu_flip_txn_bind_slot`, the `placed` field, and its commit/abort special
  cases (the PREPARE shortcuts now apply unconditionally; abort in PREPARE always
  returns false — nothing is ever installed before commit). `test_flip_latch.c`
  lost its 4 reserve_slot cases (NR_TESTS 22 → 13).

### Validation

Behavior-identical under one writer, so the suite catches regressions only: ft
default/NO_SKIP/NO_COMPRESS/both — `ft_unit`, `flip_latch`, and all 51 `ft_inv`
invariants green on every config. MCAS-edge **completeness** (the slot publish is
now a frozen-before-install txn edge in every armed/graft shape) is verified by
re-running the edge audit, not by the functional suite.
