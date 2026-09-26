# Fractal Trie — re-introducing in-place node mutations (2026-09-15)

Status: **§6 LANDED for the point INSERT (2026-09-15)** — the rest is the design
of record it was implemented from. §1–§5 record why in-place mutation was
withdrawn, why the reasons no longer apply, the one blocker that survived, and
the agreed cure; §4b–§4e are the measured history of the first attempt (reverted
at @1bbc8570) and are kept because §6 is built on their findings.

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

## 4e. ☠ THE HOIST WAS TRIED AND IT LIVELOCKS — the dedupe is DIRECTIONAL

The §4d shape was implemented (`if (ft_in_place_ok(ft))
ft_flip_txn_lock_or_guard_parent(ft, ic->txn, ctx, attach_node_flag,
FT_DEPTH_FROM_DESCENT);` before the reserve), reviewed by an adversarial
skeptic, **refuted**, and the refutation reproduced. It is REVERTED.

**The mechanism — a permanent self-refusal `-EAGAIN` spin:**

1. `ft_lock_ctx_init(&actx, &d, ic.txn, ic.op)` (ft-insert.h:3369, and the three
   sibling callers) runs **before** `ft_attach_node` arms the txn
   (`ft_insert_commit_arm`, :2154). So `actx.held.txn == NULL`, with no extras,
   glue or outer frame either.
2. The hoist acquires `attach_node_flag` and registers it in `ic->txn`.
3. The reserve relocates → `ft_node_recompact(ADD_SAME)`, which under
   `lock_fine` acquires its own lock-set — the OLD node (= `attach_node_flag`),
   P and GP — via `ft_dlm_acquire_set(ft, ctx, set, 3)` (ft-mutation-node.h:1445,
   :1549), passing **the caller's const `ctx`**.
4. Dedupe consults `ft_lock_ctx_holds(ctx, ...)` → `ft_held_set_snap` walks a
   NULL registry → **false**. `ft_dlm_lock` then sees `FT_STATE_LOCK` on the
   word and refuses → `-EAGAIN`.
5. `-EAGAIN` → `goto check_error` → `ft_flip_txn_destroy(ic->txn)` CAS-clears the
   hoisted lock → `ic->txn = NULL` → caller re-descends → identical state →
   forever. On an exclusive trie no peer can change the outcome and the op
   handle never escalates.

☞ **THE GENERAL LESSON: DEDUPE IS DIRECTIONAL.** It works for
*lock_or_guard AFTER recompact* because `ft_flip_txn_lock_or_guard_parent` builds
its OWN `lctx` with `.held.txn = t`, so it sees the recompact's registrations. It
does NOT work for *recompact AFTER lock_or_guard*: the recompact reaches
`ft_dlm_acquire_set` through the caller's const ctx, whose registry is NULL at
every `ft_attach_node` call site. §4c's "ft_acquire_member already dedupes" was
right about the primitive and wrong about which ctx reaches it.

**Reproduced, not merely argued**: `binplace` (`-DFEATURE_FT_INSERT_IN_PLACE`)
hung at `test_urcu_ft_unit` row 160 and `test_urcu_ft_inv` row 80, both spinning
(101% / 297% CPU, no output for 14 minutes). The default build stayed green
because the predicate is compile-time false there — i.e. the green half of the
evidence was the half that could not fail.

⇒ **THE REAL PRECONDITION**: the recompact's acquire must be able to SEE the op's
held set. That means `actx` must carry `ic->txn` — either by creating the txn
before `ft_lock_ctx_init`, or by re-initialising/refreshing the ctx after the
arm, or by passing a ctx that chains to the commit's registry. Until one of
those lands, ANY lock taken before the reserve is invisible to the reserve and
self-refuses. That is the next step for in-place, and it is a
plumbing/lifetime change, not a locking one.

Also surfaced by the review, unfixed and worth its own look:
- Under a COARSE spacing the later site's SHARED exit would plant a §4.B guard on
  `attach_node_flag`'s own word where the REGISTERED exit never did.
- On a coarse trie the hoist takes the `guard:` label and plants a guard BEFORE
  the recompact — the direction `ft_flip_txn_record_release_lock`'s doc forbids
  and whose comment asserts "No site does today". Benign only because :1445
  requires `lock_fine`; the stated invariant is nonetheless false.

## 5. Open items to validate (do not assume)

- Forcing recompact on the dst attach parent perturbs the fold only for readers
  whose descent actually **visits** that node. Confirm this covers every reader
  the move can tear — the move's own junction/top COW should cover paths that
  never reach the dst parent. `FEATURE_FT_FAULT_INJECT`'s
  `cds_ft_fault_rekey_countdown` forces a coherence miss and is the cheap way to
  exercise it.
- ☑ SETTLED (2026-09-23): **a discarded in-place reserve is harmless.** The
  reserve sets the bitmap bit over an EMPTY slot (the child arrives through the
  txn) and defers `nr_child++` into the txn, so a later missed take or a commit
  abort leaves "bit set, slot NULL, not counted" -- exactly the soft-deleted
  HOLE the node format already carries: the setter's refill arm fills it and
  counts it; `cds_ft_verify` checks `nr_child` against NON-NULL slots; readers
  and iteration skip a NULL slot; and popcount capacity is decided by RANK
  (`qp_ptr_idx = popcount(bms) >= max_child` -> `-ENOSPC` -> recompact, which
  drops holes), so a stale bit cannot push an append past the slot array.
  MEASURED with `-DFT_DEBUG_INPLACE_DISCARD` (1 in 64 in-place reserves
  discarded AFTER their raw stores), rcu-debug + in-place, THP disabled per
  process: ft_inv 155/155 at all three spacings (21,941-24,697 holes left per
  leg out of 1.40-1.58M in-place reserves), ft_unit 363/363 (10,346 per leg).
- The pigeon sticky-hint arm (`ft-mutation-node.h`) is the sibling change: make
  the occupancy bitmap a sticky hint set with an atomic OR, with a cleanup
  recompact once stale bits (`popcount(bitmap) - nr_child`) get high, keeping
  pigeon's O(1) insert *and* delete. It has the same §3 prerequisite.


## 6. LANDED — the point-op insert tier, lock-before-write (2026-09-15)

Mathieu's framing, verbatim: *"we need to make sure the fine and exponential
locks suffice to protect the node state updated by the in-place mutation ops,
once we have this, then we can wire up those in place mutation operations
again."* One op at a time: this section is the INSERT. The delete tier keeps the
exclusive-only predicate until its own step (§6.5).

### 6.1 The shape

- **Two predicates, not one** (`ft-helpers.h`). `ft_in_place_ok(ft)` is the
  build flag alone: the safety condition is the CALLER's, and a site passes its
  own vouch down as `@in_place`. `ft_in_place_excl_ok(ft)` is the legacy
  exclusive-only tier, kept for every site not yet converted to
  lock-before-write: the bulk reserves (graft / rekey dst attach parent), the
  build-path wrapper `ft_node_set_nth`, the bulk detaches, and — for now — the
  point removes. §3's witness needs exactly that: a same-trie move's dst attach
  parent still relocates on a shared trie, because its reserve vouches only the
  exclusive tier. The eight `ft-rekey.h` gates that spelled `ft_in_place_ok`
  now spell `ft_in_place_excl_ok`, unchanged in meaning.
- **`@in_place` threaded** through `ft_node_set_nth_rec` → `_ft_node_set_nth` →
  the popcount / pigeon setters, and `ft_node_replace_ptr` →
  `_ft_node_replace_ptr` → the two class `replace_ptr`s, and `ft_detach_node`.
  Build-invisible copies (recompact / COW loops) pass `true`; nothing they write
  is reader-visible.
- **The hoist** (`ft_attach_node`). Under the FINE strategy the attach node —
  its own DLM lock at per-node spacing, its ANCHOR at exponential / root-only —
  is acquired through `ft_flip_txn_lock_or_guard_parent_ex` BEFORE the reserve,
  REGISTERED in `ic->txn` with its `{LOCK|s -> s}` release; a MISS bails
  `-EAGAIN` before any raw store. Under COARSE the FT-wide writer lock is the
  exclusion and nothing per-node is taken. The reserve then writes the bitmap
  bit and the slot on a node the op holds.
- **The nested held-set frame** (`rctx`, `held.outer = &ctx->held`,
  `held.txn = ic->txn`) is what lets the RELOCATION arm dedupe: when the reserve
  cannot stay in place, `ft_node_recompact`'s `{C, P, GP}` acquire finds C
  already held (SHARED) instead of refusing its own mark (§4e's livelock), and
  the fused retire chains C's tombstone onto the hoist's release —
  `{LOCK|s -> TOMBSTONE|s}` at per-node, tombstone-beside-anchor-release above.
- **ONE acquire, not two.** The post-reserve `lock_or_guard` is gone for the
  fine strategy (its SHARED exit at a coarse spacing planted the MW guard that
  collided with the SW count, §4e / @898c08ce's control); COARSE keeps its §4.B
  guard, recorded before the count edge as the ordering rule requires.
- **The in-place arm resolves (grandparent, slot) ONCE**, through
  `ft_resolve_parent_slot`, and no longer republishes: holding the attach node's
  state word does not freeze its PARENT word, and the old same-value republish
  re-read that word raw inside `_ft_publish_to_parent_meta_at` — measured
  landing on a peer's re-home PROXY (`ft_popcount_node_get_ith_pos`'s type
  assert in `inv_writer_progress_chainmerge`, the parent word settled to the
  peer's fresh copy by the time the core was read). The grandparent ACQUIRE is
  the validation the republish only approximated.

### 6.2 The engine's kind check, and where a kind mix is fixed

`urcu_txn_record_chain`'s `r->kind == kind` debug police fired on the
COARSE-strategy insert (an MW §4.B validate on the attach node's state word,
then the SW `nr_child++`). Mathieu: *"kind conflict between taking the lock and
updating bits on the same word as the lock with the lock held is benign
(over-strict check)"* — the release path already resolves it MW-dominant, which
is exactly right under the exclusion. Decided: relax the check IN THE ENGINE,
accepting the ordered shape (first record MW — a validate or the lock take on
that word — then SW updates of it) and still trapping SW-then-MW. Not in the FT:
an FT-side "inherit the slot's kind" cost a linear `urcu_txn_find` per state
record and turned the dense-node unit test quadratic (a 2-minute suite ran 15+
minutes on every build; read as a hang until the timing data said otherwise).

The one SW-then-MW instance was pre-existing (red at HEAD, debug + in-place +
ROOT-ONLY: an exclusive trie's in-place delete recorded the anchor's release SW
on the ROOT's word, then `ft_remove_one_commit`'s fused `nr_child--` on the same
word through the cell-edge loop's always-MW branch). Fixed at the site: when
`ft_flip_txn_owns(txn, state_meta)` the count records through
`ft_flip_txn_record_state` (SW under `structural_sw`, chained onto the release);
otherwise it rides `edges[]` MW as before. ☠ NOT by dispatching `edges[]` on its
tag: `URCU_TXN_TAG` and `FT_STATE_PROXY` are both bit 0, and the cell-edge
producers leave `owner_held` uninitialised — that dispatch shipped an SW park of
a lock-free cell edge (`owner_held == 7`, `inv_rekey_fine_mixed_writers`) before
the debug owner assert caught it.

### 6.3 Measured: the locks suffice at the store

`-DFT_DEBUG_CHAIN_HOLD -DFEATURE_FT_HOLD_TRACE -DFEATURE_FT_INSERT_IN_PLACE`,
`test_urcu_ft_inv` (list on, per-node), the node-body class declared LOCKED at
every in-place store, put through the registry / ledger / wide-ctx ladder:

| site (taken arm) | total | WLOCK | hidden | lockOK | lockVIOL |
|---|---|---|---|---|---|
| `ft_popcount_node_set_nth:521` (3-level) | 3977298 | 189701 | 1131980 | 2655927 | **0** |
| `ft_popcount_node_set_nth:154` (5+3) | 171188 | 108383 | 0 | 62805 | **0** |
| `ft_popcount_node_set_nth:335` (6+2) | 379739 | 168884 | 0 | 210855 | **0** |
| `ft_popcount_node_set_nth:710` (1-level) | 176582 | 18514 | 0 | 158068 | **0** |
| `ft_pigeon_node_set_nth:893` | 34348418 | 391424 | 33796626 | 160368 | **0** |

Before the hoist the same population read `lockOK=0 lockVIOL=4040923` (§4b).
The one row with violations (`:516`, 19465) is a REFUSED arm: callers that
declined the tier (the exclusive-only wrapper on a fresh build node) and
recompact; no store happens there, and the LOCKED declaration on an unpublished
node is the audit's known false positive.

### 6.4 Validation (all with THP disabled per process — see §6.6)

Release `-O2 -DNDEBUG -DFEATURE_FT_INSERT_IN_PLACE`, debug
`--enable-rcu-debug` with the same flag (detector verified armed), the audit
build above, and the default build; each at per-node / exponential / root-only
(the debug and release in-place trees carry `-DFEATURE_FT_LOCK_SPACING_ENV`):
ft_unit 357/357 and ft_inv 152/152 in the list-on, list-off and MW modes —
see the commit message for the leg table.

### 6.5 Open

- ☑ **The DELETE tier LANDED on 2026-09-16** (the text that stood here said it
  was still `ft_in_place_excl_ok`; it was not updated when the tier landed).
  The three point-remove `ft_detach_node` calls pass `ft_in_place_ok(ft)`.  The
  first widening attempt's two failures -- a duplicate-chain walk into freed
  memory in `inv_concurrent_same_key_removes` at exponential spacing, and a
  lost key in `inv_sibling_split_compress_unpinned` -- were root-caused before
  it landed: a NULL plan expected-old the equality guard could not see and an
  in-place arm that re-read the contended slot; a TORN (parent word, slot
  offset) pair that made the delete clear a slot OUTSIDE the node it
  decremented (refused per climb level); three NULL dereferences on stale
  plans (`36354d84`); and the duplicate walk ignoring engine proxies.  The
  lock-set inventory then made the pure leaf delete lock its holder before
  reading the plan (`7e4e9d15`).
  RE-VALIDATED 2026-09-23 on the in-place build (+ the SW stale-old audit and
  lock detectors, THP off), with each test repeated in-process and pinned to 2
  cpus with 2 competing spinners: 16 fine-locking concurrent tests x 550
  repetitions at per-node AND at root-only -- 0 red, 0 detector signals; and
  `inv_sibling_split_compress_unpinned`, unpinned, 275 repetitions per spacing
  -- 0 lost, 0 transient at all three (pinned it trips its own per-writer
  liveness assertion, also 0 lost).
- The bulk reserves and the build-path wrapper stay exclusive-only; each needs
  its own lock-before-write before `ft_in_place_excl_ok` can retire.
- The default build still compiles the tiers out.  Since 2026-09-26 they are
  two switches, `FEATURE_FT_INSERT_IN_PLACE` (insert) and
  `FEATURE_FT_DELETE_IN_PLACE` (delete); each edit site asks its own tier
  (`ft_in_place_insert_ok` / `ft_in_place_delete_ok` and their `_excl_`
  forms), and the same-trie rekey's refusal gates keep `ft_in_place_excl_ok`,
  which answers "either tier".  ☑ ON BY DEFAULT since 2026-09-26
  (-DNO_FEATURE_FT_{INSERT,DELETE}_IN_PLACE opt out; the gate's no-in-place,
  no-in-place-insert and no-in-place-delete configs keep the recompact paths
  covered), after fixing the three defects the first default-on gate found:
  a sibling key LOST and a chain DOUBLE-OWNED under concurrent remove_all with
  skip-compression off (delete tier: the orphan walk now must arrive at its
  own target), a peer's lock bit erased by an SW count park at exponential
  spacing (insert tier: SW needs the exact state word), and an exclusive-trie
  rekey refused under either tier (the same-trie rekey now runs with both
  tiers off).

### 6.6 Operational: the stall that was not a livelock

Two whole rounds of this validation "hung" box-wide, the unchanged default
build included. It was transparent-hugepage direct compaction on this host
(`enabled=always`, a THP-backed arena), seen from inside the stall: 100% system
time, zero user time, no syscall, zero voluntary context switches, `SIGKILL`
ignored, and `/proc/vmstat`'s compaction counters FLAT — a fault stuck inside
one compaction increments nothing until it returns. Both frozen PCs were the
first touch of a fresh 2 MB-aligned arena region. Every leg here runs with an
`LD_PRELOAD` constructor calling `prctl(PR_SET_THP_DISABLE, 1)`; the same
mechanism was root-caused earlier for ft_unit's `same_path` row.
