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

## 4b. MEASURED BLOCKER: the in-place store PRECEDES the acquire (@90b44097)

Step 1 landed as the `move_active` conjunct in `ft_in_place_ok` (@ee12ab22):
in-place is refused while a move is in flight, using the same gate the reader
consults, with the mover's existing set-gate-then-GP-then-mutate drain making it
sound. Observably a no-op until `ft->exclusive` is dropped.

Before dropping it, the refused `-ERANGE` arms were probed — they are exactly
the population the widening would newly admit — declaring `FT_EXCL_LOCKED` and
asking the audit whether the op holds the node. With **all three** witnesses
(registry, hold ledger, wide ctx) plumbed down from `ft_node_set_nth_rec`:

```
nodebody  total=67699790  WLOCK=1093244  hidden=62565702
          lockOK=0  lockVIOL=4040923  anchored=0
```

Nothing holds the node, and no ancestor is held either. **The reason is
ORDERING, not an absent lock**: in `ft_attach_node` the reserve
(`ft_node_set_nth_rec(ft, &iter_dest_node_flag, ...)`) is at :2187 and the
acquire of that same node (`ft_flip_txn_lock_or_guard_parent(..., ctx,
iter_dest_node_flag, ...)`) is at :2266 — 79 lines later. The op holds the node
by the time it COMMITS; it does not hold it at the moment the in-place store
would happen. Harmless today because the reserve is refused; with
`ft_in_place_ok` returning true it would mutate a LIVE node before locking it.

⇒ **The widening therefore needs the ACQUIRE HOISTED above the reserve, not just
the predicate relaxed.** That hoist is the known-dangerous move here — a prior
attempt HUNG 2 of 8 producers when an op re-took a word it already held and read
its own mark as contention (`struct ft_held_set`'s header: the second acquire
returns -EAGAIN and the op waits on itself forever). So it is its own step, with
its own validation, and the dedupe question (does the op already hold this word
via a lock-set member or an anchor?) must be answered BEFORE adding the acquire.

## 4c. THE HOIST IS A MOVE, NOT AN ADDITION — feasibility settled

§4b's "the op holds the node by NO witness" is true *at the reserve* and must not
be read as "the op never holds it". At `:2266` the op ALREADY acquires
`iter_dest_node_flag` — the same word, the same op, guarded by
`if (iter_dest_node_flag == attach_node_flag)`, i.e. exactly the in-place case.
So hoisting acquires `attach_node_flag` at `:2187` **instead of** at `:2266`; it
relocates an existing acquire rather than introducing one. The window in which
the op holds that lock grows by the work between the two points — a CONTENTION
cost, not a correctness one.

**The self-collision livelock is already prevented, provided registration is
visible.** `ft_flip_txn_lock_or_guard_parent` acquires through
`ft_acquire_member` with `lctx.held = { .txn = t, .extra, .glue }`, and
`ft_dlm_acquire_set`'s contract is explicit: when the op already holds the word
the member comes back SHARED and "owes NO release and NO terminal — the acquire
that first took it recorded both". So the later call at `:2266` dedupes instead
of self-colliding **iff** the hoisted acquire is registered where that lookup
looks: `ic->txn` (via `ft_flip_txn_lock_register`) or the ctx's extras. The
historic hang (2 of 8 producers) is the failure mode of registering it
somewhere else, not of taking it earlier.

Implementation sketch, in order:
1. Acquire `attach_node_flag` before the reserve and register it in `ic->txn`.
   Registering before the ARM is legal — the arm's contract is "after the op's
   LAST ft_flip_txn_lock_register", so earlier is fine.
2. `:2266`'s `lock_or_guard` then takes the SHARED exit and only plants its
   guard, which is the no-op ordering (guard after a release on one word reads
   that record's pending clean value and validates `{s -> s}`).
3. On the RELOCATION path the hoisted lock sits on a node the op then retires —
   the existing F2 retire-fence shape (`@free_old_cn_held`), not a new one.
4. `-EAGAIN` (a peer holds it) must unwind the partially built cluster via the
   existing `goto check_error`.

☐ Still to validate: the contention cost of the widened hold window, and that
every path between `:2187` and `:2266` tolerates the lock being held (nothing
there may take it again outside the dedupe, and nothing may release it early).

## 4d. ☠ THE :2266 ACQUIRE IS UNREACHABLE TODAY — §4c's "move" reading is wrong

Measured the split the hoist's shape depends on (`-DFT_DEBUG_INPLACE_HOIST`,
counting `iter_dest_node_flag == attach_node_flag` right after the reserve):

```
FT_INPLACE_HOIST  reserve IN PLACE=0  RELOCATED=3518118  (reloc 100.00%)
```

☠ **READ THE CONTROL FIRST: this number is CIRCULAR.** With `ft_in_place_ok`
false, every reserve returns `-ERANGE` and routes through
`ft_node_recompact(ADD_SAME)`, which builds a fresh node — so relocation is
forced *by construction*, not observed. It says nothing about the ratio that
would obtain with in-place enabled, and the honest measurement needs a config
where the predicate can be true (an EXCLUSIVE trie under
`-DFEATURE_FT_INSERT_IN_PLACE`), which today's oracles do not drive.

☑ **What it DOES establish, and it matters:** `iter_dest_node_flag` never equals
`attach_node_flag`, so the guard at `:2265` is never true and **the acquire at
`:2266` never executes in the default build.** It exists FOR the in-place case
and is currently dead code.

⇒ So §4c's "the hoist relocates an existing acquire" is **wrong**: today there is
no acquire on that path at all. The hoist would INTRODUCE a lock — and on the
relocation arm it would sit on a node the reserve RETIRES, which the comment at
`:2261` says needs nothing today ("the reserve's recompact already locked the
grandparent it republishes into"). Introducing one there means the retire must
become ANCHORED against the held word rather than plain — the exact pairing this
tree has already been bitten by ("the GUARD and the ANCHORED RETIRE shared a
predicate and disagreed on the KIND").

**Revised shape**: acquire `attach_node_flag` before the reserve *only when
`ft_in_place_ok`* (so the relocation-only default is byte-for-byte unchanged),
and hand the held anchor to the retire on the arm where the reserve relocates
anyway. The dedupe reasoning in §4c still holds for the later `:2266` call once
it becomes reachable.

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
