# Fractal Trie — re-introducing in-place node mutations (2026-09-15)

Status: DESIGN OF RECORD. No implementation yet. Records why in-place mutation
was withdrawn, why the reasons no longer apply, the one blocker that survives,
and the agreed cure.

Related: `doc/design/mcas-multiwriter-readiness.md` §4 (the disjoint-word
hazard) and §5.2 ("an exclusive trie keeps the in-place store"),
`doc/design/mw-to-fine-locking-remainder.md`, `doc/design/in-trie-move-seqcount.md`,
`doc/design/mw-writer-lock-escalation-model.md`.

---

## 1. What was removed, and why

In-place mutation of a live node's words was withdrawn when the trie moved to
the MW-with-CAS model. An MW CAS arbitrates a **whole slot**, so every node
change had to become a whole-node replacement published through one parent edge:
a fresh node is built build-invisibly, carries the new entry *and* its bitmap,
and the parent edge is flipped. That is the multi-writer-safe shape, and it cost
the O(1) in-place insert, turning it into an O(node) alloc-and-copy recompact.

The machinery was kept behind a gate rather than deleted:

- **`FEATURE_FT_INSERT_IN_PLACE`** — compile-time opt-in. Recompact-on-insert is
  the default. (`fractal-trie-internal.h`; the CI config is named `in-place`.)
- **`ft_in_place_ok(ft)`** in `ft-helpers.h` — the runtime safety condition,
  today `return ft && ft->exclusive;`. Its header states the split: *"The build
  flag is the OPT-IN, this is the SAFETY CONDITION."* The flag alone used to
  decide, which applied in-place to shared tries and asserted —
  *"ft_attach_node's slot_ptr, 303 failures in 480 saturated runs"* — or tore a
  publish quietly.

`fractal-trie-internal.h` already anticipated the reversal: *"Re-enabling the
in-place fast path for a single-writer trie (a runtime gate) is a future perf
knob."*

## 2. Why the reasons no longer apply

`ft_in_place_ok`'s header names two hazards and declares both away via
`cds_ft_attr_set_exclusive` ("single-writer, no concurrent readers"):

> *"Those three words are separate stores, so a concurrent READER can sample them
> torn, and a concurrent WRITER can rebuild the node from its occupied slots
> while the mutation is mid-flight — which drops a reserved (bit set, NULL child)
> hole from under the writer that reserved it."*

### 2.1 The writer hazard — subsumed by the locking regimes

There is always a lock on the relevant node. Four cases, each providing
exclusion at least locally:

| # | case | trie state | what the op holds |
|---|---|---|---|
| 1 | coarse | `!ft->lock_fine` | the FT-wide writer lock (`ft_wlock_held == ft`) |
| 2 | fine | `lock_fine && lock_spacing == CDS_FT_LOCK_SPACING_PER_NODE` | the node's own metadata lock |
| 3 | exponential | `lock_fine && lock_spacing == EXPONENTIAL` (`ROOT_ONLY` is the degenerate coarsest of the same family) | the nearest-parent anchor (`ft_anchor_meta`) |
| 4 | user-provided | `ft->exclusive` | nothing — the app declares single-writer/mutex |

Removing MW-CAS updates is what makes this sufficient: an MW CAS needs the node
replaced atomically, SW-under-lock does not. The only MW survivors are the
**root pointer** and the **cell list**, neither of which is a node-body
bitmap/slot mutation, so neither blocks in-place.

### 2.2 The reader hazard — already handled: THE BITMAP IS MONOTONE IN PLACE

Concurrent readers were supported with in-place mutation *before* the MW work,
and the publish protocol is still in the code. The invariant that makes it sound:
**an in-place mutation never removes a bitmap bit, so no existing rank moves.**

- **Insert (safe-append)** — `ft_popcount_node_set_nth` refuses (`-ERANGE` →
  recompact) unless the new bit is strictly the highest
  (`in_byte_above || above_slot || root_above`), which is exactly the condition
  under which no existing entry's rank shifts. The slot is published with
  `rcu_assign_pointer` first; in the new-hi case the **root bitmap bit is the
  gate and is written last**; the metadata count is added last.
- **Delete** — the bit is left **sticky**: *"occupancy bitmap bit SET — a sticky
  soft-delete hint, never cleared in place"*. Only the slot goes NULL and the
  metadata count decrements, so `popcount(bitmap)` never shrinks.
- **The two bounds are different words.** `ft_popcount_node_get_ith_pos` asserts
  on `popcount(bitmap)`; the metadata `nr_child` is *"the soft-delete pointer
  accounting"*, independent of it. Because the bitmap only grows in place, that
  assert cannot be over-run from either direction.
- Readers already tolerate the transient by design: *"if a value is there, but
  its associated pointer is still NULL, we return NULL too."*

### 2.3 The gates that remain are STRUCTURAL, not modal

Room (`-ENOSPC`), append-at-end (`-ERANGE`), and for delete the shrink threshold
— which already exists:

```c
if (!ft_in_place_ok(ft) || ft_meta_nr_child_load(metadata) <= type->min_child)
        return -EFBIG;
```

Setting the child pointer to NULL and decrementing `nr_child` is always safe to
*attempt*; recompaction is the fallback when the node would drop below
`min_child`.

## 3. The one surviving blocker: the rekey's two-descent coherence witness

This is **not** a locking question and **not** an MW/SW question. It is a
reader-side mechanism that free-rides on COW-induced address churn.

`ft_lookup_two_descents` (`ft-lookup.h`) runs two descents and compares a
`struct ft_visit_witness` — an order-dependent 64-bit fold of the **addresses**
of the nodes visited. Its header states the dependency:

> *"IDENTITY, not value: a move COWs its stitch points, so a moved subtree's
> junction and top get FRESH addresses, and RCU cannot recycle a node inside a
> read section — which makes the witness immune to the away-and-back
> (oscillating rekey) case that fools any comparison of keys or result nodes."*

`ft-mutation-node.h` names this as the prerequisite for this exact change:

> *"☠☠ AND IT WOULD BREAK THE IN-TRIE MOVE'S READER COHERENCE … what guarantees
> the perturbation is THIS arm: the destination attach parent gains an occupancy,
> so it relocates. Make the bitmap sticky and a move whose dst parent is a pigeon
> node can publish no fresh address anywhere on the graft arm, and a torn descent
> becomes indistinguishable from a clean one. … in-place mutation without a COW
> is meant to be RE-ALLOWED more widely (… `ft_in_place_ok`'s exclusive-only gate
> is another). The move's coherence must therefore stop being a free ride on this
> arm BEFORE that happens."*

Failure mode: allow in-place, and a same-trie move whose dst attach parent merely
gains an occupancy publishes no fresh address on the graft arm. Two sequential
descents fold identical addresses, agree on the result node, and the reader
accepts a view that straddled the move's commit — "somewhere that was never
correct at any instant".

Not a niche configuration: `ft-lifecycle.h` sets
`ft->rekey_coherence = !ft->speculative_key_offset_active;` and installs the
coherent lookup on `ft->lookup_key_fn`. It only bites while a move is active
(`ft_move_active`), which is precisely the window the witness exists for.

Locking does not fix it — readers hold no locks, so no lock-set change can make
a reader's address fold perturb.

## 4. The agreed cure, and the order of operations

1. **Introduce a `force_recompact` flag for rekey.** Chosen over the older note's
   "special-case the rekey so it COWs the parent itself": same effect, carried as
   a per-mutation flag rather than bespoke logic inside the rekey.
   - Threading precedent exists: `defer_parent` / `deferred_count` already pass
     down `ft_node_set_nth_rec` → `ft_popcount_node_set_nth` to the very test
     that gates this. `force_recompact` lands beside them.
   - It must override only the **live** case: `defer_parent` (build-invisible)
     has no published address to churn, so forcing there buys nothing.
   - Scope: the **dst attach parent** is load-bearing — the src side relocates
     too but is not what the witness needs.
2. **Then `ft_in_place_ok()` becomes `return true;`** The flag carries the
   rekey's requirement, so the predicate never enumerates locking modes. The four
   cases of §2.1 become documentation of *why* this is correct, not a per-op
   runtime check in the hot path.

## 5. Open items to validate (do not assume)

- Forcing recompact on the dst attach parent perturbs the fold only for readers
  whose descent actually **visits** that node. Confirm this covers every reader
  the move can tear — the move's own junction/top COW should cover paths that
  never reach the dst parent. `FEATURE_FT_FAULT_INJECT`'s
  `cds_ft_fault_rekey_countdown` forces a coherence miss and is the cheap way to
  exercise it.
- The in-place reserve sets a bitmap bit in the LIVE attach node and only the
  *count* is made idempotent; nothing rolls the bit back on the `acquire_miss`
  discard path. Sticky semantics suggest this is harmless, but the tree does not
  say so. Settle it.
- The pigeon sticky-hint arm (`ft-mutation-node.h`) is the sibling change: make
  the occupancy bitmap a sticky hint set with an atomic OR, with a cleanup
  recompact once stale bits (`popcount(bitmap) - nr_child`) get high, keeping
  pigeon's O(1) insert *and* delete. It has the same §3 prerequisite.
