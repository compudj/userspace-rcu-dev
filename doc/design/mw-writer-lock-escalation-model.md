# MW writer model: optimistic per-node locks + FIFO escalation — design note (2026-07-14)

Status: **IMPLEMENTED (was PROPOSED 2026-07-14; status corrected 2026-07-31).**
A significant pivot for the Fractal Trie multi-writer (MW) transition, worked
out in discussion with Mathieu Desnoyers.  The per-node lock (DLM) model
described here is now the ONLY multi-writer implementation -- it is selected at
runtime by `ft->lock_fine`, not by a build flag, and the optimistic per-slot-CAS
strategy it replaced has been removed.  Four sites in the FT sources cite this
note (ft-insert.h, ft-mutation-node.h, ft-mutation-helpers.h), and the public
header cites it too, so "not yet implemented" was actively misleading.  This
note captures the model, its progress proof, the
load-bearing disciplines, and what it buys vs. costs, so it is the reference we
implement against. The next design corner *below* this note — per-operation
lock-set definition — is deliberately left open (see "## Open: per-operation
lock-sets").

Supersedes the *direction* of the lock-free-MW campaign at `d48ed267` (skip-resolver
reanchor, detach coherence, PSO/graft/stale-slot fixes). Those fixes stay correct
and stay in the tree; the pivot changes the *strategy* they were serving. Related:
`recompact-copy-precommit-window.md` (why `FT_STATE_LOCK` is load-bearing — the
seed observation for this pivot), `transactional-flip-latch.md`,
`mcas-multiwriter-readiness.md`.

---

## 0. The seed observation

`FT_STATE_LOCK` is a lock.

- Acquired by CAS on the node's state word → a try-lock.
- Held across a multi-step critical section (build the recompacted node, reparent)
  → the critical region.
- Contending writers observe it and back off with EAGAIN → contention + retry.
- Released at commit → unlock.

That is mutual exclusion on a node's mutation, full stop. The reverted
pre-commit-callback experiment (`recompact-copy-precommit-window.md`) is the proof:
the tombstone *claim* is a commit-time record and could **not** fence the
build/reparent plan window, whereas the persistent LOCK CAS could. LOCK was
load-bearing precisely *because* it is a lock and the txn commit primitive is not.

The pivot: stop half-implementing exclusion inside the txn state word and fighting
the engine that is trying to be lock-free. Make the lock explicit, use the MW engine
*as a deadlock-free lock manager*, and run the actual structural edit single-writer
under the held locks.

---

## 1. The layering

Every structural mutation splits into two engines with disjoint jobs:

- **MW-txn = lock acquisition only.** Its entire job shrinks to: atomically CAS a
  known set of `{lock, node-reclaim}` state words from free→held, with **read-set
  validation** on the slots assumed during descent. Success ⇒ the writer owns that
  subgraph. Failure ⇒ owns *nothing* and re-descends. It never touches structural
  slots. In the contended path it is backed by a FIFO lane (§3).

- **SW-txn = the actual edit.** Once the locks are held the writer is — by
  construction — the single writer over that subgraph, so the build / reparent /
  publish / reclaim runs under the *already-proven* single-writer discipline. Its
  only job is writer↔reader coherence via RCU publish. It runs **off the lane**,
  under the held locks.

Release `{lock, node-reclaim}` at commit.

**Why this is the point, not just tidy:** it moves the concurrency proof burden from
a *structure-shaped* surface to a *word-shaped* one. The MW residual-crash taxonomy —
PSO stale-slot, skip-resolver type confusion, unpaired slot-record, parked-proxy
clone — is *entirely* "two writers racing on structure." If structure is only ever
mutated single-writer-under-lock, that whole family is not patched, it is **designed
out**. The only thing left to verify hard is one fixed-format primitive: "acquire
this word-set atomically with read validation." MW correctness is audited *once*, on
words.

---

## 2. The lock's two terminals (REVISED at implementation — the tombstone does NOT split)

**This section originally proposed splitting the tombstone into `node-reclaim` (MW,
reader-invisible) and `node-deleted` (SW, reader-visible), and reserved bit 20 for it.
Implementing step 3 showed the split buys nothing, and its stated justification does
not hold in this tree. It is WITHDRAWN (Mathieu, 2026-07-14); bit 20 is un-reserved.**

Why it was withdrawn — three independent reasons, each sufficient:

- **The premise is false: `FT_STATE_TOMBSTONE` is not reader-visible.** No reader
  reads it — zero references across every lookup / descent / iterator /
  ordered-query / inequality header. Reader-visible deletion is expressed by the
  structural unlink plus `CDS_FT_NODE_REMOVED_FLAG` on `cds_ft_node.next`: a
  different mark, on a different word. The bit is purely writer-side, so
  "reader-invisible reclaim vs reader-visible delete" is a distinction with no
  reader to draw it.
- **No writer consumer distinguishes the two meanings either.** All four treat any
  tombstone identically: the lock's own dirty check (`ft_meta_lock_acquire`) bails on
  it, `ft_flip_txn_guard_parent` masks it into an abort, the recompact reparent sweep
  masks it (the UAF guard), and the freeze-on-free audit accepts it as free-eligible.
  A writer already knows at the call site whether it is retiring keys or deleting
  them; it never needs to *read back* which kind of death a node died.
- **It would not have dissolved the double-tombstone poison.** The poison was two
  records on the SAME WORD with a stale expected-old; splitting the bits leaves both
  records on that same word (`{s|LOCK → s|RECLAIM}` then `{s → s|DELETED}` has
  exactly the same stale expected-old). What actually fixed it is the read-your-writes
  load in `ft_flip_txn_record_tombstone` (`47a1a612`,
  `project_ft_txn_double_tombstone_poison`).

**What the lock model actually needed was the other thing this section was reaching
for: a lock that can UNLOCK.** Today `FT_STATE_LOCK`'s only commit-OK terminal is
`→ TOMBSTONE`, because the only thing that ever takes it is a copier that retires the
node. The moment a lock-set contains a member that is *edited but survives* —
recompact's parent `P` (§9.3) — the lock needs a second terminal. So the lock has
exactly two, both recorded as an MCAS edge on the state word whose expected old is the
mark's clean snapshot:

| terminal | transition | who |
|---|---|---|
| **retire** | `{LOCK\|s → TOMBSTONE\|s}` | the node is copied away and dies (`C`) |
| **release** | `{LOCK\|s → s}` | the node is edited/protected and lives (`P`, `GP`) |

Plus the pre-existing non-commit terminal: ABORT / MEMORY_ERROR / a pre-commit bail
CAS-clear the bit through the txn's `locks[]` registry, leaving the node live — the
same resulting word as *release*, differing only in who writes it (a bare CAS vs the
atomic commit). **The registry therefore needs no knowledge of which terminal an op
chose**, which is why adding *release* touched neither the drain nor the CORE_682870
expected-old contract.

**Corollary — the release record IS the guard.** A locked member needs no
`ft_flip_txn_guard_parent`: both are one record on the same word, and the lock is
strictly stronger (the guard only makes *us* abort after the fact; the lock makes
*peers* abort up front). Planting both is a bug, not belt-and-braces — the guard
expects the clean word and the release expects it fenced, and two records with
different expected olds poison the descriptor. Every §9 conversion therefore
*replaces* a guard site rather than adding to it, which is the mechanical form of
"the guard sites ARE the lock-set" (§9).

**Reader contract still shrinks**, just for a simpler reason than the split: readers
resolve proxies and **never observe the lock**.

---

## 3. The engine: optimistic acquire + FIFO fallback (no helping)

A well-understood, provably-correct shape: **optimistic MCAS lock-acquisition + a
fair FIFO fallback on contention + RCU wait-free readers.** Optimistic concurrency
with a serialization fallback for progress. The updated MW/MCAS engine has **no
helping**; livelock-freedom comes from the FIFO lane.

- **The lane is per-FT (per txn domain).** One fair queue per trie. It is *not* the
  steady-state path — it is an **escalation** path. "Global per trie" is the lane's
  *identity*; *engagement* is conflict-gated.

- **Fast path (no collision):** optimistic MCAS acquire of the lock-set; no lane;
  fully parallel. Disjoint writers on disjoint subtrees never touch the lane.

- **Escalation (sustained collision):** past a **threshold of genuine collisions**
  the writer elevates to the per-trie FIFO lane, which serializes contending writers
  fairly. Threshold (not first-collision) so that *transient* collisions — a peer
  holding a lock for a few ns — resolve cheaply on the fast path, and only *sustained*
  contention pays the lane's enqueue cost. The bounded retry window is the price;
  the threshold bounds it.

### 3.1 The (a)/(b) seam — the load-bearing detail of the threshold

The optimistic acquire fails two very different ways, and **only (b) counts toward
the escalation threshold**:

- **(a) Read-set stale, no lock contention** — a slot assumed during descent changed,
  but no node the writer wants to lock is actually held by a peer. Benign: a peer
  committed elsewhere; just re-read. Re-descend and retry *on the fast path*. Common
  in a busy trie. **Must NOT increment the escalation counter.**

- **(b) Genuine lock contention** — a node the writer needs to lock is already held by
  a peer. *This* is the collision. Counts toward the threshold; at threshold, escalate.

If any acquire failure bumped the counter, a writer in a hot-but-*disjoint* region
(lots of (a) churn, zero real contention) would spuriously cross the threshold and
escalate — silently destroying the disjoint-parallel property, which is the whole
thing the pivot buys. **Threshold counts (b); (a) does not feed it.** Put an
assertion around this.

---

## 4. Progress proof (bounded per-operation)

End-to-end liveness, worst case per operation:

1. **Fast path:** up to *threshold* wasted optimistic attempts on (b) collisions.
   (a) failures re-descend on the fast path and do not bound-limit.
2. **Escalate:** take a per-trie FIFO ticket.
3. **Keep your position across abort.** Abort-and-regrow (§5) releases the locks but
   **retains the FIFO ticket** — it does not send the writer to the back. This is the
   crux: because position is retained, once every op ahead has committed the writer is
   at the **head**, and nothing behind it can invalidate its read-set (all later ops
   are ordered after it). So its *final* acquisition attempt is **guaranteed** to
   succeed, in bounded time.
4. **Edit + commit:** SW-txn build/publish under held locks, off-lane; release.

FIFO livelock-freedom at the engine level + retained-position ⇒ bounded progress at
the **FT-operation** level. This is the guarantee the earlier helping-based framing
could not give for free.

---

## 5. Load-bearing disciplines

The safety of the whole model rests on all-or-none acquisition plus these rules.
Stricter ordering (global-per-trie lane) can only cost throughput, never correctness;
safety comes from *all-or-none acquire*, not from the lane's scope.

- **Lock-set conservatively complete up front.** The acquisition MCAS must cover every
  node whose state OR slots the SW edit will touch — including nodes discovered
  reachable during descent (parent + child for a reparent; the recompacted node + its
  parent; a spine for merge/graft; a sibling pulled in by boundary compression).

- **Abort-and-regrow, NEVER grow in place.** The moment the SW edit discovers it needs
  a node it did not lock, it may **not** grab that lock incrementally — that
  reintroduces hold-and-wait and destroys deadlock-freedom. It must release the full
  set and re-descend with a bigger set. All-or-none acquire + abort-and-regrow is what
  removes any lock-ordering obligation.

- **Recompute the lock-set on every re-descend.** Whether the re-descend came from a
  benign (a) or from abort-and-regrow, the lock-set is recomputed against *current*
  structure. A stale lock-set is never carried into a retry. (Confirmed.)

- **Abort leaves the structure byte-for-byte unchanged.** Every abort path releases
  the full lock-set, un-stamps any speculative `node-reclaim`, and frees every
  reserved-but-unused node. This is the CLAUDE.md best-effort / abort-boundary gate,
  now doing real load-bearing work: at the abort boundary the structure must be
  identical to before the op and no resource may leak.

- **Exactly one lane transit per operation (intended).** Descent, invisible build, and
  reclaim run outside the lane. The writer transits the MW lane *once* to acquire
  `{lock, node-reclaim}`. The publish (parent-slot flip + `node-deleted`) is the
  **SW-txn**, done under the held locks and therefore *off* the lane. Release is a
  store to owned words. Keeps the serialized region to a single fixed-size acquisition
  MCAS.

- **The acquire must NOT ride the escalation lane. (PROVEN at implementation, step 2 —
  this corrects the sketch above.)** Acquiring a lock *through* a transaction on the
  trie's `urcu_txn_domain` deadlocks, structurally. The engine's escalation proof
  assumes a txn that reaches the head of the lane can **complete**; a lock acquire
  cannot — completing requires the *current holder* to release. So an escalated
  acquirer holds its FIFO turn while it spins, and once `domain->active` is published
  the lane funnels **every** txn in the domain — including the holder's own commit and
  its release — in behind the waiter that is waiting on it. Circular wait. Measured:
  16-writer soak wedged (383 threads futex-blocked in `urcu_txn_reserve`), RSS to
  3.8 GB in 8 s (a fresh MCAS descriptor per acquire retry), `call_rcu` never draining
  (spinners never quiesce). **A lock acquire sits BESIDE the engine, not inside it**:
  coarse mode uses a plain `cds_fair_mutex` (FIFO-fair, futex-parking,
  zero-allocation). The lane keeps carrying *transactions*, which is what its proof
  covers. Fine-grained per-node locks (steps 3-6) inherit this constraint: the
  all-or-none lock-set acquisition may be an MCAS on the state words, but it may not be
  a txn that *waits* for a peer's release while occupying the lane.

- **No blocking writer lock may be held across a grace-period wait. (PROVEN at
  implementation, step 2.)** Under QSBR the waiters parked on the lock are RCU-online
  and non-quiescent — they are precisely what keeps the grace period from completing —
  so a holder that calls `synchronize_rcu()` while holding the lock deadlocks against
  its own waiters. (Same invariant the engine states for its lane: *the lane owner
  never blocks on a GP while holding the mutex*.) Every mid-operation GP wait on a
  writer-scope path — detach, `make_exclusive`, graft/graft_swap, and the same-trie
  `merge_at` — therefore **drops the lock, waits, and retakes it**
  (`ft_writer_lock_gp_wait()`). This is sound, not a hole: the GP sits at the *seam
  between two distinct commits*, so the structure is coherent and
  reader-visible-consistent there, and another writer may legitimately run in the
  window. It does mean an op that spans a GP is **not** one atomic writer critical
  section, and never was — the pre-existing MW cross-view invariants (not the lock) are
  what make that window safe.

---

## 6. What it buys

- **Whole MW structural-race bug class designed out** (§1): PSO, skip-resolver,
  unpaired-slot, parked-proxy — all "two writers on structure," all gone by
  construction.
- **Double-tombstone poison dissolved** (§2), not patched.
- **Reader contract shrinks** (§2): readers never observe the lock; wait-freedom
  tightens.
- **Same-key update pulled INTO contract for free.** Today concurrent same-key update
  crashes in `ft_promote_head` (two writers on one duplicate chain) — "outside the
  mutual-exclusion contract." Under the pivot both same-key writers compute
  overlapping lock-sets (same leaf), so they collide, serialize, and the loser
  re-descends onto the updated structure. No concurrent chain manipulation. The model
  *provides* the mutual exclusion the contract was hand-waving.
- **In-place mutation returns for the common case** (re-enabled by writer-writer
  exclusion). In-place append/remove was abandoned under lock-free MW because two
  writers racing a node's tail slot + count had no mutual exclusion; the lock supplies
  exactly that, and readers stay safe by publish ordering for the cases that do not
  move an existing child's reader-visible position:
    - **append-at-tail** — the new slot lands beyond the live range (append order, or
      the new branch byte is the current max in a bitmap-sorted node): release-store
      the slot → bitmap bit → `nr_child`, all under `C`'s lock. A reader observes the
      published bit and the formed slot behind it, or neither. No alloc / copy / free.
    - **remove-at-tail / tombstone-remove** — clearing the *tail* bit shifts no live
      child's popcount index; a *middle* physical remove would, so it stays a tombstone
      (`node-deleted`, reclaim later).

  This attacks the dominant MW cost directly — recompaction full-node copies + the
  alloc/free storm (`project_ft_recompaction_livelock`) — and *reduces how often* the
  LOCK-fenced recompaction path runs at all. Scope today: amortized-O(1) append into
  capacity; a *full* node still grows via copy; middle-insert / middle physical
  compaction still copy (they move other children's reader-visible popcount indices).

  **Future work — txn-mitigated bitmap → in-place middle-insert too.** The lock already
  removes the writer↔writer obstacle for a middle-insert; the only thing left is
  writer↔reader coherence during the array shift (a reader mid-lookup could read a
  half-shifted slot at a moved index). Make the bitmap a *transacted / reader-validated*
  word so readers observe a coherent pre- or post-insert index mapping, and middle-insert
  becomes in-place without a full-node copy. This is the txn engine's **second** role,
  orthogonal to lock acquisition: the lock handles writer↔writer, the txn-mitigated
  bitmap handles writer↔reader for the shift. **The engine is repurposed by the pivot,
  not discarded.**

  Recurring theme across §6 and §8.3: the lock keeps *removing* lock-free-era machinery —
  CAS'd `nr_child`, MCAS-packed re-home, copy-on-write append/remove — replacing each
  with a cheaper operation under mutual exclusion.
- **MW proof burden collapses** to one auditable primitive.

---

## 7. Costs and trades

- **Structural collisions serialize; disjoint stays parallel.** Disjoint-key writers
  on genuinely disjoint subtrees run fully parallel (fast path, never touch the lane).
  Writers that structurally need the same internal nodes escalate and serialize
  through the one per-trie lane — even across unrelated collisions, because the lane
  is global-per-trie. Expected profile: leaf-local edits parallelize; split / merge /
  recompaction on a shared upper spine serialize. Most steady-state inserts after
  warmup are leaf-local, so the 16w-disjoint number (0/1600 @16w @`d48ed267`) should
  largely survive; heavy structural churn on a shared spine is where the lane bites.
  **Measure this**, do not assume it.

- **Offsetting per-op gain (not only a cost).** The serialization above is partially
  repaid: under the lock, append-at-tail and tail/tombstone-remove become in-place O(1)
  instead of whole-node copy-on-write (§6), and — with the future txn-mitigated bitmap —
  middle-insert too. Churn-heavy workloads may net *positive* despite the lane. Price the
  lane and the per-op saving **together**; do not measure the lane in isolation.

- **Blocking under preemption (chosen).** No helping ⇒ a lane-head / lock-holder that
  is descheduled blocks its successors, exactly as a preempted lock-holder does. This
  is the accepted price of *choosing locking*; it matches the standing guidance not to
  oversubscribe writers on the FT tests. FIFO buys starvation-freedom on top of the
  lock, which a naive lock would not give.

- **Lane scope is an engine knob, decoupled from FT correctness.** Global-per-trie is
  the strictest, safest ordering. If per-trie throughput among *escalated* writers
  later becomes the bottleneck, shard the escalated lane by region (order only
  overlapping word-sets) — with **zero change to the FT-side proof**, because §4's
  bounded-progress argument holds within a conflict class just as it does globally.
  Ship correct-and-serial; widen the lane later only if a real workload demands it.

---

## 8. Field ownership model (the lock-to-field map)

The foundation §9 is derived from. **Every mutable field is owned by exactly one
lock.** Two writers that both write a field must both hold that field's owning lock,
so they serialize and the field stays coherent. Crucially — and counter-intuitively —
**the owning lock is not necessarily in the same node as the field:** node `C`'s
back-edge bits live physically in `C`'s metadata but are owned by `C`'s **parent**'s
lock. A node's own lock bit lives in its own state word and protects its self-owned
fields; a field owned by another node's lock is reached only while holding *that*
node's lock.

### 8.1 The edge principle

A parent→child edge has **four** physical fields, and all four are owned by the
**parent's** lock, so the lock holder flips the whole edge coherently:

- `P.slot[i]` — forward pointer (parent side)
- `C.meta.parent` — back pointer (child side)
- `C.parent_slot_offset` — C's slot position within P (child side)
- `C.incoming_byte` — the branch byte on the P→C edge (child side)

This is what kills the PSO / stale-`(parent,slot)` family
(`project_ft_chain_compress_stale_publish_slot`): a peer growing `P`'s node type and a
writer publishing into `P.slot[i]` now both require `P`'s lock, so the incoherent
pairing cannot arise.

### 8.2 Field-by-field ownership

| Field | Physical home | Owner lock |
|---|---|---|
| forward child slots (body) | node body | **self (C)** |
| node type / class / bitmap (body) | node body | **self (C)** |
| skip_len / compressed key_bytes (body) | node body | **self (C)** — boundary move needs C **and** child |
| `nr_child` (state bits 2–10) | state word | **self (C)** |
| `proxy` (state bit 0), lock bit, `node-reclaim` (MW) | state word | **self (C)**, lock-domain |
| `node-deleted` (SW, reader-visible) | state word | **self (C)** |
| `external_nodes` (entry list; same-key chain) | metadata | **self (C)** |
| `nr_keys` (subtree rollup, **opt-in**) | metadata (own word) | **self (C)** — SW txn under C's lock; the spine climbs into the lock-set (§10) |
| `parent` pointer | metadata (own 8 B word) | **parent (P)** — already isolated ✓ |
| `parent_slot_offset` (state bits 11–18) | **state word** | **parent (P)** — ✗ *straddles* |
| `incoming_byte` | **u32 with `alloc_index`** | **parent (P)** — ✗ *straddles* |
| `alloc_index` | packed u32 | self / immutable (set once at alloc) |

### 8.3 The layout violation this surfaces (needs sign-off before implementing)

The current 32 B `cds_ft_metadata` layout was optimized for the *lock-free* strategy;
the pivot inverts that pressure and two parent-owned fields end up welded to
self-owned words, so a plain masked RMW under one lock silently loses a field written
under the other lock:

- **`parent_slot_offset` is packed into `state` bits 11–18, adjacent to `nr_child`.**
  The struct comment states the reason outright — "so a re-home commits the parent
  edge and the slot offset as ONE atomic MCAS state edge." Under locks that welds
  `P`'s lock (re-home) and `C`'s lock (`nr_child` bump) into one RMW unit:
  `ft_meta_nr_child_set` is a masked read-modify-write that only "preserves
  parent_slot_offset" if nobody else is writing it — which is exactly the cross-lock
  case. Lost update.
- **`incoming_byte` shares a `uint32_t` with `alloc_index`** — safe *only* because
  `alloc_index` is immutable post-allocation. A landmine, not a bug today.

**The fix the model forces:** group the parent-edge fields `{parent_slot_offset,
incoming_byte}` into their own word adjacent to `parent`, all under `P`'s lock;
`state` retains `{proxy, lock, node-reclaim, node-deleted, nr_child}`, all cleanly
`C`-owned.

**Payoff — it removes lock-free-era machinery rather than adding to it:**
- `nr_child` inc/dec drops from the Phase-4.3 latch-honoring CAS-retry back to a
  **plain store under `C`'s lock**.
- Re-home drops the "one atomic MCAS state edge" packing rationale — `parent`,
  `parent_slot_offset`, `incoming_byte` become coherent stores under `P`'s lock.

STATUS of this split: **APPROVED (Mathieu, 2026-07-14), and DEFERRED PAST STEP 3
(2026-07-14, at implementation).** Sequencing constraint: it must land *with* the
pivot's lock + re-home rewrite, **not** standalone on the current optimistic tree.
Moving `parent_slot_offset` out of `state` removes the atomic MCAS-state-edge property
that today's optimistic re-home depends on (the state-word MCAS carries the new offset
coherently with the flip); the replacement — coherent stores under `P`'s lock — exists
only once `P`'s lock does. Landing it early would regress the `0/1600 @16w` baseline.

**Step 3 clarified WHEN "with the lock rewrite" actually is: not at step 3 — at the
END, when OPTIMISTIC is retired.** The lost update this section warns about does not
occur while the writers remain CAS loops, and they are:
`ft_meta_parent_slot_offset_set` CASes `state` (`7045681c`) and `nr_child` uses the
Phase-4.3 latch-honoring CAS-retry, so a cross-lock write to the shared word costs a
**spurious abort, never a lost update** (the MCAS record validates its expected old and
one side loses). Word-sharing is therefore a *contention* defect under locks, not a
*correctness* one. The split's payoff — dropping those CAS loops to plain stores — only
materializes once the optimistic re-home that needs them is gone. Landing it at step 3
would take the stated `0/1600` risk to buy nothing yet.

So: approved, still welded to the re-home rewrite, but that rewrite is the last step of
the transition, not this one.

---

## 9. Open: per-operation lock-sets (the next design corner)

Once §8's table exists, per-operation lock-set derivation is mechanical:

> **lock-set(op) = { owner-lock(f) : f ∈ fields the op writes }**

The §8 table *is* this function. A re-home writes `{P.slot[i], C.parent,
C.parent_slot_offset, C.incoming_byte}` → all owned by `P` → lock-set `{P}`; add a
change to `C`'s own shape → `{P, C}`.

**Scope: these lock-sets are for the `rank_stats`-OFF (default) build only.** The
`rank_stats`-ON build uses one FT-wide mutation lock (§10.5), so it needs no per-op
lock-set, and the spine count-fold never enters any set below (`arm(0)` always).

The correctness work lives one level below this note: for each structural operation,
what is the **conservatively-complete up-front lock-set**, and how is
abort-and-regrow (§5) enforced when the SW edit discovers it needs more?

Operations to pin, each its own analysis:

- **insert** — leaf node; parent iff node-type change (split/grow).
- **remove** — leaf node; parent iff merge/shrink; duplicate-chain nodes for same-key.
- **recompact** — the node + its parent (the LOCK window, restated as an explicit
  lock).
- **merge_at** — the spine.
- **graft / graft_swap** — the spine + graft point.
- **bulk** (detach / graft / merge) — cross-view atomicity under the lock model.

For each: enumerate the nodes whose state OR slots are touched; prove the set is
knowable at descent time or that abort-and-regrow converges; state the read-set that
the acquisition MCAS validates.

### 9.1 insert (default / `rank_stats`-off build) — PINNED (2026-07-14); LANDED in part (step 4)

**Landed (LOCK_FINE):** the compressed-split family (I-4a diverge / key-shorter, I-4b
past-child) is converted at its one shared choke point, `ft_insert_publish_or_park`:
its `parent_nf` — the publish-into node, which SURVIVES the commit and whose slot is a
same-slot value swap (I-4a's `P` = CN's parent; I-4b's `CN` itself) — is acquired as a
per-node RELEASE lock ({LOCK|s → s}) instead of §4.B-guarded. The body-read node
(`CN`, split/retired) was already LOCK-locked at build entry (ins:653/2188) and
RETIRE-terminated, so insert has no *new* body-locked node — `parent_nf` is the only
guard→lock flip, and it is uniform across all three shapes.

On an acquire miss the publish FALLS BACK to the plain guard, which is a correct
degradation *specifically because* `parent_nf` is a value-swap target whose body is not
copied under the lock: the guard aborts at commit iff the peer still holds it, else the
publish is safe. (Contrast recompact, which copies `C`'s body under `C`'s lock and so
must re-descend on a miss.) The clean all-or-none acquire, with no fallback, arrives
when insert's FT-wide lock drops; under that lock the miss never happens, so the
fallback is `FEATURE_FT_FAULT_INJECT`-only.

**Deferred, both principled:**
- **I-1 in-place reserve `{P}`** — the attach node whose *own* `nr_child` this op
  increments (a direct latch-CAS, `ft_meta_nr_child_inc`, that does not honor the lock).
  Locking that node only becomes meaningful once §8.3 moves `nr_child` under the lock
  (a plain store), so I-1 lands with §8.3 at the end of the transition. Its guards
  (ins:1632/1827) stay.
- **I-4b's skip-dual `P`** — the `record_reserved`-owner half of the lock-set (the SKIP_X
  dual writes a slot in CN's parent), which is unguarded *today* (§9.1.4's under-count).
  Load-bearing only at the FT-wide-lock drop; deferred with it.
- **I-3 duplicate append `{L}`** — `L` is a bare `cds_ft_node` hlist with no `state`
  word, so it cannot take an FT_STATE_LOCK; `{L}` is the `last->next` value-CAS
  in its own private txn. Nothing to convert.

The original derivation follows.



The lock-set is read off the conflict-guard sites the current MW code already places,
with one correction. The precise rule is:

> **lock-set(op) = owners of every written slot = { `ft_flip_txn_guard_parent`
> targets } ∪ { owners of `ft_flip_txn_record_reserved` slots }.**

`guard_parent(X)` ("validate `X`'s state word at commit, abort if a peer froze it")
becomes "hold `X`'s lock" — same node. But guard sites *alone* under-count: a slot
written via a recorded CAS without a state guard (the past-child skip-dual, I-4b)
still has an owner that must be held. Hence the union with `record_reserved` owners.

Cases (all `arm(0)` — no spine fold; `P` = node whose slot gains the edge, `GP` =
`P`'s parent, `L` = duplicate-chain owner, `CN` = compressed node split/extended):

| Case | Shape | Lock-set | Retire | Guard / record site |
|---|---|---|---|---|
| **I-1** in-place reserve | `P` gains a child, has capacity | **{P}** | — | `guard_parent(P)` (ins:1632) |
| **I-2** reserve recompacts `P` | `P` full → grow; fresh `P'` published at `GP` | **{GP, P}** | `P` | `guard_parent(GP)` (ins:1718) |
| **I-3** duplicate append | append to dup-chain tail | **{L}** | — | recorded CAS `last->next` (ins:1934) |
| **I-4a** compressed diverge / key-shorter | replace `CN` with prefix→branch | **{P, CN}** | `CN` | `guard_parent(P)` (ins:474); `CN` LOCK @build-entry |
| **I-4b** compressed past-child | grow a branch under `CN` | **{CN, P}** skip / **{CN}** no-skip | — | `guard_parent(CN)`; `P` skip-dual recorded, unguarded |

Four things this pins down:

1. **`GP` enters only on grow (I-2).** Every compressed split (I-4) re-publishes
   `CN`'s slot as a *value-swap in an existing slot* — the parent is never given a new
   child, so it never recompacts, so `GP` stays out. Only I-2 (attach into a *full*
   node) copies `P` and re-publishes at `GP`'s slot. So the §9 sketch is now exact:
   insert = `{P}`, plus `{GP}` iff the target node must grow.

2. **A re-parented child is covered by its parent's lock, not locked separately.** In
   I-4, `leaf = cn->child` is re-homed from `CN` onto the fresh cluster. By §8.1
   `leaf`'s back-edge is owned by `leaf`'s *parent*'s lock = `CN` (until commit), which
   is already held — so `leaf` is not a separate lock-set member. The back-edge write
   is *parked* and flips atomically with `P`'s forward slot at commit, never before
   (else a reader up-walking from the still-reachable `leaf` lands in the unpublished
   cluster; ins:1644-1657). Corollary: a re-parent of `leaf` (holder `CN`) and a
   child-insert *into* `leaf` (holder `leaf`) touch disjoint owners and legitimately
   run **concurrently** — field-granular ownership buys concurrency a whole-node lock
   would forfeit.

3. **Abort-and-re-descend is already coded.** A peer re-homing `CN`/`P` between descent
   and build is caught by `ft_get_parent_slot(cn_meta) != parent_slot` → `-EAGAIN` →
   re-descend (ins:962, 1691, 2261) and the concurrent-writer bail (ins:1429) — exactly
   "read-set stale → recompute the lock-set, re-descend" (§5). The `CN` lock acquire
   (ins:653/2176), taken *before reading CN's body*, is `CN`'s lock acquired at build
   entry — confirming the set is held up front, not merely at commit.

4. **The skip-dual (I-4b) is the sole guard-site under-count.** Under skip-compressed
   (default) build, extending `CN` re-encodes `CN`'s slot *inside* `P` (skip target
   old-external→branch): a write to `P`'s forward slot → owned by `P` → `P` must be
   held, though today it is value-CASed rather than `guard_parent`-ed. This is why the
   rule unions in the `record_reserved` owners. A no-skip build drops this → `{CN}`.

**Open sub-point — ordered-list cells (list-ON only).** With the ordered list on, every
insert also splices the new cell between two existing ordinal cells, writing
`pred->next` and `succ0->prev` (ins:276-334). These neighbors are **discovered late** —
by a from-root/from-head predecessor search at *commit* time, not at descent — so they
fall outside the descent-knowable `{P, CN, leaf}` set. **Resolved in §9.2:** the in-order
predecessor/successor are structurally local (adjacent sort-order keys share a long
prefix), so the pred/succ cells become ordinary lock-set members discovered in the plan
phase and flipped in the *same* commit as the structural edge — required by cross-view
atomicity. The current commit-time predecessor search folds into the plan.

### 9.2 remove (default / `rank_stats`-off build) — PINNED (2026-07-14)

Remove is where the lock-set stops being leaf-local. Two facts from the code frame it:

- **Every pure leaf delete recompacts its boundary.** In the default build
  (`FEATURE_FT_INSERT_IN_PLACE` off), deleting the last entry at a popcount/pigeon
  boundary returns `-EFBIG` and is serviced by `ft_node_recompact(FT_RECOMPACT_DEL)` —
  a full copy-and-republish of the boundary, never an in-place `nr_child--`. So
  remove's common case is copy-on-write *today*. This is exactly the churn §6's
  in-place tail/tombstone-remove eliminates once the lock makes in-place safe — turning
  `FEATURE_FT_INSERT_IN_PLACE` on is that payoff, not a separate feature.
- **No remove publish ever recompacts its target.** Every forward flip is a same-slot
  pointer swap (`_ft_publish_to_parent` / `ft_ord_cell_flip_into`), never an
  `ft_node_set_nth`. So the publish-into node `GP` is never relocated — locking `GP`
  does not cascade to *its* parent for the flip. (Skip-on exception: if `GP` is
  compressed the publish re-encodes the SKIP_X dual in `GP`'s own parent slot, pulling
  that one node in a level higher.)

Same derivation rule as §9.1 (`guard_parent` targets ∪ `record_reserved` owners), plus
the §9.1 refinement that a re-parented child is covered by its parent's lock — which
does heavy lifting here.

`L` = chain holder; `BP` = surviving prune boundary (= the detach target's parent when
no elevation); `GP` = `BP`'s parent = publish-into node; `B`/`parent_CN`/`child_CN` =
chain-merge trio; `orphan` = the pruned single-child ancestor chain.

| Case | Shape | Lock-set | Guard | Skip-only? |
|---|---|---|---|---|
| **R-1a** unchain interior dup | hlist splice `prev->next`/`next->prev` | **{L}** | — (engine CAS) | no |
| **R-1c** unchain head, key gone | `L.head_slot→NULL` (+ skip-dual) | **{L}** (+ skip-dual holder above `L`) | `L` @2397 | no |
| **R-2** `ft_promote_head` | flip `head_slot`, fold `next->prev`, cell swap, freeze `node` | **{L}** (+ skip-dual holder) | `L` @2232/2294 | no |
| **R-3** detach → recompact `BP` | copy `BP`, re-parent survivors, publish copy at `GP` | **{BP, GP}** + `orphan[≤MAX_DEPTH]` | `BP` @1781, `GP` @1881/1963 | no |
| **R-3A** detach, compressed parent | ext-promote into `L`, or replace at `GP` | **{L}** or **{GP}** + orphan | `L` @171/194, `pub_parent` @261 | partial |
| **R-4** `ft_chain_compress_fused` | merge `B+parent_CN+child_CN` → fresh `new_cn` at `publish_parent` | **{B, parent_CN, child_CN, publish_parent}** | `publish_parent` @684 | **yes** |
| **R-5** `ft_canonicalize_chain_compress` | R-4 as a standalone 2nd flip, one level up | = R-4 | `publish_parent` | **yes** |

Four load-bearing findings:

1. **The lock-set is discovered by a read-only planning climb, not known at the leaf.**
   Only R-1a is determined by `@node` alone. Every spine path (R-3/4/5) finds `BP`,
   `GP`, the orphan set, and the merge neighbours *during* the `metadata->parent`
   up-climb or under the node lock — where the prune stops depends on live
   `nr_child`/`external_nodes` read mid-climb. So remove's acquisition is inherently
   two-phase, and **this is the template for recompact / merge_at / graft**:

   > **plan** (read-only descent + up-climb computes `{BP, GP, orphan…}` and the
   > read-set = each climbed ancestor's `nr_child`/`external_nodes`) → **acquire** the
   > whole set in one all-or-none MCAS that validates that read-set → **edit** →
   > **commit**; a read-set mismatch (a peer grew a single-child ancestor, or re-homed
   > `BP`) → **abort and re-plan**.

   Not new machinery: the existing `-EAGAIN` bails (2035/1169/1875/1956) and
   node lock bails (§4 @496/511/535) *are* the re-plan hooks. The lock model renames
   "guard + validate at commit" to "acquire + validate the read-set," and fixes that the
   set is computed by the plan, never grown in place mid-climb.

2. **Recompaction fans the write-set sideways, but the fan is covered.** A `DEL`
   recompaction of `BP` re-parents *every surviving child of `BP`*. By the edge
   principle those back-edges are owned by `BP`'s lock (their parent), already held — so
   the survivors are **not** separate lock-set members. A peer inserting *into* a
   survivor runs concurrently with our re-parent (disjoint owners): the field-granular
   win again.

3. **The orphan chain is genuinely multi-node and each member must be held.** The prune
   retires up to `FT_MAX_DEPTH` single-child ancestors; each must be in the lock-set
   (node-reclaim), *not* covered by a parent — because "this ancestor has exactly one
   child, collapse it" races a peer adding a second child to it, and holding its lock is
   what freezes its child-set for the decision. This is the widest part of the set:
   `{BP, GP} ∪ orphan[≤depth]`, and skip-on's R-4 adds `{B, parent_CN, child_CN,
   publish_parent}` on top.

4. **Skip-on is where the width lives.** In a *no-skip* build the whole chain-compress
   family (R-4/R-5), shape-D fusion, the SKIP_X duals and the post-commit canonicalize
   climb **do not exist**: remove is strictly `{BP, GP} ∪ orphan ∪ {leaf}`, single-edge
   publishes, no upward climb after commit. Skip-compressed (default) adds the merge trio
   and a *second* lock-set acquisition for the post-commit canonicalize (R-5, its own
   mini-op one level up).

**Ordered-list cells — in the single commit, as structurally-local lock-set members
(decision 2026-07-14).** The cell splice edges (`pred->lnode.next`, `succ->lnode.prev`)
recur in `ft_promote_head` (R-2) and detach (R-3), and they are **not** a separate
concern to be carved out:

- **They must ride the same commit.** Cross-view consistency is a hard FT invariant, not
  a preference — a reader must never see a key present in the trie but absent from the
  ordered list, or vice versa (`project_ft_bulk_xview`, `project_ft_remove_fusion_xview`:
  "unlink + cell unsplice in ONE flip"). So the cell edges must be recorded into the
  *same* SW-txn as the structural edge and flip with it atomically. Splitting the list
  onto its own txn would break cross-view atomicity outright.
- **They are structurally local, not arbitrary.** Keys adjacent in sort order share a
  long common prefix, so the in-order predecessor/successor sit in the same neighborhood
  as the structural mutation — reachable by an in-order neighbor walk from the insertion
  point, bounded by trie depth, the same O(depth) band as `{BP, GP, orphan}`. (Worst
  case — inserting the first key of a top-level branch — the in-order predecessor climbs
  to a near-root common ancestor and descends a sibling; but that is the same O(depth)
  worst case the prune climb already has.)

Decision: **the pred/succ cells are ordinary lock-set members**, discovered in the
**plan** phase by the in-order neighbor walk (folding the current commit-time from-root
predecessor search into the plan, which locality makes affordable) and acquired in the
same all-or-none MCAS as the structural set. One mechanism (locks), one atomicity domain
(one commit). This supersedes the value-CAS carve-out floated in §9.1's open sub-point.

Contention consequence: two mutations whose keys are in-order-adjacent share a cell and
serialize — but within the same O(depth) locality band as trie contention, and largely
subsumed by it (adjacent keys usually share structure too). The one extra sliver — in-order
neighbors that are structural siblings — is bounded and local, never the arbitrary
coupling a global list lock would impose. Tie-in: `project_ft_bidir_list_integration`.

### 9.3 recompact (default / `rank_stats`-off build) — PINNED (2026-07-14)

Recompact is not a user operation — it is the shared boundary-copy **primitive** that
insert I-2 (grow), remove R-3 (shrink), and the per-trie compactor (relocate) all
invoke. Pinning it once fixes the atom the climbing ops reuse per level. Frame: `C` =
the node being recompacted, `P` = `C`'s parent.

**The node lock *is* `C`'s lock — §0's seed observation, made concrete.**
`ft_node_recompact` opens (mut-node:1043) with `ft_meta_lock_acquire(C)` *before any read
of `C`'s body* — the sizing loads, the `(parent, offset)` inherit, and the copy loops all
read under it — and the commit converts `{LOCK|s → TOMBSTONE|s}`. That is exactly
acquire-lock-up-front + node-reclaim-at-commit. A dirty mark (peer proxy / concurrent
copier / real retire) **bails before any allocation** (mut-node:1044-46) — "couldn't
acquire → re-descend." The whole pivot is this one function generalized.

**Write-set (live retire, the `retire_txn` arm):**
- `C` — state word `LOCK`→`TOMBSTONE`, retired → `C`'s lock (the fence).
- every **surviving child of `C`** — back-edge `(parent, PSO, incoming_byte)` re-parented
  to the fresh copy (`ft_reparent_record_meta`), recorded into `retire_txn`. Owned by
  `C`'s lock (their parent), already held → **covered, not a separate member** (the
  §9.1/§9.2 refinement).
- `P`'s slot — fresh copy published old→copy, a **same-slot swap** (never `set_nth`), so
  `P` is not itself recompacted and there is no cascade to `P`'s parent → `P`'s lock.
- skip-dual — if `P` is compressed, the publish re-encodes `C`'s SKIP_X in `P`'s own
  parent slot → grandparent `GP` enters one level higher (recorded by the caller's
  `_ft_publish_to_parent` for DEL; into `rec` otherwise).

**Lock-set: `{C, P}`** (+ `{GP}` iff `P` compressed). Known immediately from `C` — `P` is
resolved once via `ft_resolve_parent_slot(C)` and validated. **No planning climb**:
recompact is the near-leaf atom, in contrast to remove, which invokes it at a boundary
its climb has already located.

**Build-invisible (`cluster_leaf`) recompact is unfenced → no `C` lock.** When `C` is the
lower boundary of an as-yet-unpublished cluster it is thread-private: the fence is skipped
(mut-node:1039-41), the re-parent loop is skipped (the mutator wires the live back-pointers
itself), and the lock-set collapses to `{P}` — or nothing while the whole cluster is
private. So "recompact needs `C`'s lock" holds only for a **live, published** `C`.

**Modes → callers → naming (the consolidation):**

| Mode | Caller | `C` / `P` in caller's names | Caller lock-set |
|---|---|---|---|
| `ADD_SAME`/`ADD_NEXT` (grow) | insert I-2 | `C` = grown attach node, `P` = its parent | `{C, P}` = I-2's `{P, GP}` |
| `DEL` (shrink) | remove R-3 | `C` = `BP`, `P` = `GP` | `{C, P}` = R-3's `{BP, GP}` (+ orphan) |
| `RELOCATE` | compactor `cds_ft_compact` | `C` = relocated node, `P` = its live parent slot | `{C, P}` |

So `{C, P}` is the atom: insert I-2 adds nothing, remove R-3 adds the orphan chain, and
the **background compactor is not special** — its relocate acquires `{C, P}` like any
other mutator (`project_ft_compactor_landed`).

### 9.4 merge_at (default / `rank_stats`-off build) — write-sets PINNED; cross-domain acquisition OPEN

**Merge is a cross-trie operation, decomposed into two single-domain commits.** `src_ft`
is *not* exclusive by default: merge takes `CDS_FT_SCOPED_WRITER` on **both** tries
(mrg:2190-98), `src_ft` may have concurrent readers (`exclusive` is a default-false
per-trie attribute), and `src_ft` is genuinely mutated. `exclusive` only elides the src
grace period (`!src_ft->exclusive` gates the *drain*, mrg:1643/2010) — not the writes.
(The "EXCLUSIVE" comment at mrg:2581 is about the transient `@subtree` a prior
`ft_detach_keylen` produced, not the caller's `src_ft`.) Rather than acquire across two
per-FT lanes at once, merge is structured **detach-then-attach** (decision, §9.4.1):
first `ft_detach_keylen` the src subtree into an exclusive standalone subtree — a
**src-domain** commit — then attach that now-exclusive subtree into dst — a **dst-domain**
commit. Each commit is single-domain; the cross-domain problem never arises. So the shape
table below is the **attach** step's dst-side lock-set (src is exclusive by then), and the
src side is a §9.2 detach in `src_ft`'s own domain.

**Shape taxonomy** (`ft_merge_at_inner`):

| Shape | dst-side lock-set | src-side | Known@descent |
|---|---|---|---|
| **M-1** empty-dst root swap (2522) / whole-source →graft (2502) | `{dst root}` + 1 tombstone | pre-detached → **exclusive** subtree | yes |
| **M-2** spine-copy `ft_merge_spine_copy` (1086) | `{pub_parent}` ∪ **overlap-spine dst nodes** (retired); dst-only children covered by them | src unlink slot + recompacted single-child ancestor chain + tombstones (src domain) | **no — recursed** |
| **M-3** graft-in-place `ft_merge_graft_subpos_inplace` (1821) | GLUE: `{GP_dst}` + displaced child (covered); NOSPLIT: `{graft-pt}` + recompact may pull graft-pt's parent (§9.3) | src unlink + drain (src domain) | mostly |

Guards confirm the dst forward-publish points: `pub_parent` (mrg:1677, M-2),
`glue.publish_parent` (mrg:3977/4195, M-3); M-1's root slot is auto-guarded by the root CAS.

Two PINNED findings:

1. **M-2's dst lock-set is the overlap-spine, discovered by the recursive build** —
   `{pub_parent} ∪ {dst nodes ft_merge_build copies}`, the re-parented dst-only children
   (`nr_dst`) covered by their overlap-spine parents (already in the set). Its size is
   bounded by the **overlap** — the read-only `ft_merge_count` pre-pass (mrg:1142) already
   computes an upper bound — **not** `FT_MAX_DEPTH`, not the whole subtree. This is the
   §9.2 plan/acquire template with the plan = that pre-pass. The forward publish is a
   same-slot REPLACE (nr_child invariant) → `pub_parent` is not recompacted → no further
   parent pull.

2. **M-3 is graft** — whole-source and diverged cases delegate to `ft_graft_keylen`, so
   §9.5 is largely M-3: dst lock-set `{GP_dst}`, plus the NOSPLIT sub-case where attaching
   a high byte forces a *range recompact* of the graft-point node (mrg:1901-06),
   re-publishing it at its parent via the §9.3 atom — the one place graft pulls a level up.

#### 9.4.1 Cross-domain acquisition — DECIDED: two single-domain commits (2026-07-14)

Merge/graft/bulk-cross-trie are structured as **two sequential single-domain commits —
detach, then attach** — never a simultaneous two-domain acquire. Commit 1 detaches the src
subtree into an exclusive standalone subtree (a `src_ft`-domain op, lock-set the §9.2
detach family `{BP_src, GP_src, orphan_src}`); commit 2 attaches that now-exclusive subtree
into dst (a `dst_ft`-domain op, dst lock-set per the shape table). This is what M-1 and the
same-trie-rekey path already do; it is now the model for **all** cross-trie ops.

Consequences:
- **No cross-domain acquisition, no cross-domain deadlock, no domain-ordering rule** — each
  commit lives entirely in one per-FT lane. The all-or-none MCAS (§3) never spans two domains.
- **The attach step's src side is exclusive**, so its lock-set is dst-side-only — the shape
  table above.
- **merge_at needs no merge-specific lock-set** — it *composes* from a §9.2 detach and a §9.5
  graft/spine-copy. The M-2 spine-copy's dst overlap-spine set (discovered by the recursive
  build, bounded by the overlap) is the only piece unique to merge's attach; M-3 is graft.
- **Cross-trie atomicity is not required and not provided.** Two commits means the moved keys
  are transiently in *neither* trie (they sit in the exclusive detached subtree, not
  reader-reachable). That is fine: the cross-view invariant (§9.2) is *within* one trie
  (its structure vs its own ordered list), never across two independent tries with separate
  readers. Each trie stays independently consistent — src loses the keys atomically at commit
  1, dst gains them atomically at commit 2.

Reconciliation note: the current in-place fused M-2/M-3 unlink (the "leak-free reorder",
mrg:2639-43) fuses src-unlink into the dst attach commit — a lock-free-era optimization. Under
the lock model those shapes conform to detach-then-attach like M-1; re-examine the
leak-freedom under locks + pre-reserved allocation during implementation.

### 9.5 graft / graft_swap (default / `rank_stats`-off build) — PINNED; one exception flagged

Under §9.4.1 every cross-trie op is a sequence of single-domain commits, and graft is the
**attach half** merge composes with — so §9.4 + §9.5 cover all cross-trie ops.

**graft (`cds_ft_graft`, non-swap)** = M-3, confirmed: the src side empties
(`ft_root_list_swap_publish(src_ft…)` + drain — a `src`-domain commit) making the moved content
exclusive, then a `dst`-domain attach. Attach outcomes (`ft_graft_build`):

| Outcome | Shape | dst lock-set |
|---|---|---|
| `GLUE` (diverge inside a compressed node) | build split cluster, publish at grandparent | **{GP_dst}** + displaced child (covered) |
| `NOSPLIT` (graft point located) | `ft_store_at_graft_point` writes graft-pt's child slot (ADD) | **{graft-pt}**; a high-byte add range-recompacts it → pulls its parent via §9.3 |
| `POPULATED` | occupied → error, tries pristine | — (no writes) |
| root / empty-dst | dual root swap | **{dst root}** (auto-guarded) |

Guard: `glue.publish_parent` (grft:3977/4195). Identical to merge's M-3 — merge and graft share
one attach lock-set.

**graft_swap subtree (`cds_ft_graft_swap`, `key_len > 0`)** — a symmetric exchange of dst's
subtree-at-key with swap_ft's content — already **decomposes into single-domain commits**
(grft:2096+), consistent with §9.4.1: (1) retire swap's root to empty + drain [`swap` domain];
(2) publish-replace swap's content into dst at `key`, detaching dst's old content [`dst` domain,
`{GP_dst}` via `glue_insert`]; (3) install dst's extracted old subtree as swap_ft's new root
[`swap` domain]. Single-domain each, drain between; lock-sets are the same `{GP_dst}` / `{root}`
shapes.

**graft_swap whole-trie (`key_len == 0`) — the one exception to §9.4.1.** A **symmetric
dual-root exchange**: `dst->root` and `swap->root` flip in **one** cross-trie flip
(`ft_root_list_swap_publish_dual`, grft:1655), explicitly to avoid the window where a key is
"reachable in both tries or neither" (grft:1622-24). It **cannot** decompose into
detach-then-attach — both sides are live and must swap atomically. So it is the single cross-trie
op that needs a genuine cross-domain step. But its write-set is minimal and fixed: exactly
**`{dst->root, swap->root}`** (+ the four list endpoints), and both tries must **share a group**
(grft:1550), the dual swap being "one epoch flip" *because* they share a group (grft:1619).

**DECIDED (2026-07-14): the lane stays per-FT, and the whole-trie swap decomposes like every
other cross-trie op — no exception at all.** Per-*group* was rejected: its only benefit is fusing
this one op into a single atomic txn, while its cost is serializing *every independent trie in a
group* through one lane — a broad concurrency tax for a single rare op. (The whole-trie swap's real
need is a group-wide *epoch* — one flip settling both roots' proxies — not a group-wide *lane*;
separable, and neither worth a per-group construct.)

Rather than the bounded 2-root acquire, the whole-trie swap is kept **regular** — three
single-domain commits (option (b)), structured so the **live dst never blinks empty**:
1. **[swap domain]** detach swap's content → exclusive `B`; `swap->root → empty`.
2. **[dst domain]** **atomic replace** `dst->root: old_dst → B` (a same-slot root flip, so dst is
   never NULL); `old_dst` falls out as exclusive `A`.
3. **[swap domain]** install `A` at `swap->root`.

Only the swap/staging side transiently empties, which matches the API roles (`dst` = live
destination, `swap` = the trie exchanged in), so the atomic "publish new version, retrieve old"
contract holds. **The only guarantee surrendered vs. a 2-root acquire is *symmetric* atomicity —
both tries live and neither ever observed empty — a rare case; if it is ever a required contract,
reinstate the bounded address-ordered `{dst->root, swap->root}` acquire (a) for `key_len == 0`
only.** With (b), **every cross-trie op is (C) with zero exceptions.**

### 9.6 bulk (detach / graft / merge) — PINNED: no new primitive; §9 closed

"Bulk" here is **subtree-granular**, not many-individual-keys: bulk-load = graft a pre-built
subtree (§9.5); bulk-removal = `cds_ft_detach` a whole subtree at a prefix into a standalone
trie; bulk-update = merge (§9.4). So §9.6 pins only `cds_ft_detach`; graft and merge are done.

**`cds_ft_detach` (`ft_detach_keylen`) is the §9.2 R-3 detach family applied at a *prefix*, not
a leaf** — it excises the whole subtree under `prefix` into an exclusive result trie:
- **Root detach** (`prefix` empty): move the source root into the result trie (root swap); source
  becomes empty. Lock-set `{src root}`.
- **Non-root detach**: unlink the subtree at `prefix` from its parent, prune the now-empty
  single-child ancestor chain, recompact a shrinking parent (`ft_detach_node`,
  `free_detached_subtree=false` preserves the moved subtree). Lock-set = **R-3**:
  `{unlink point = prefix's parent, orphan-chain prune ancestors, recompaction {BP,GP}}` —
  **source-side only** (result trie exclusive).

Two properties make bulk detach cheap and confirm it adds nothing new:

1. **The detached subtree — of any size — contributes only its root.** Its internal nodes move
   *wholesale by pointer*; only the subtree root's back-edge is re-homed (to the result trie), and
   that back-edge is owned by the unlink point's lock, already held (§8.1). So the structural
   lock-set is **O(prune-depth), not O(subtree-size)** — detaching a million-key subtree locks the
   same handful of nodes as a ten-key one.
2. **The ordered-list run is contiguous, so its splice is O(1).** A subtree = a key-prefix = a
   *contiguous* sort-order range, so excising it is two boundary edges — `run_first`'s predecessor
   and `run_last`'s successor — regardless of subtree size. Those two boundary cells are the
   in-order neighbors of the subtree's min/max leaves: structurally local, in the lock-set, fused
   into the unlink commit (§9.2). This is `project_ft_bulk_xview`'s "excise contiguous run in one
   flip," now: **O(1) list boundary cells on an O(prune-depth) structural set, one commit.**

**§9 result — the whole mutation surface reduces to a fixed set of lock-set atoms:**

| Atom | Lock-set | Where |
|---|---|---|
| in-place child add/remove | `{node}` | §9.1 I-1, §9.2 R-1 |
| recompact (grow / shrink / relocate) | `{C, P}` (+ `GP` skip-dual) | §9.3 |
| edge publish / graft attach | `{GP_dst}` (+ recompact-pull) | §9.1 I-4, §9.5 |
| detach + prune | `{unlink, orphan-chain, recompact}` | §9.2 R-3, §9.6 |
| dup-chain splice | `{L}` | §9.1 I-3, §9.2 R-1/2 |
| ordered-list cell splice | pred/succ cells, same commit | §9.2 |

Every operation is a composition of these, each acquired in one domain, sequenced per (C). Bulk
introduces **no new atom** — it is detach (§9.6) + graft (§9.5) at subtree granularity. **§9 is
closed.**

---

## 10. `nr_keys` and subtree aggregates (opt-in)

**Decision (2026-07-14):** `nr_keys` is protected by the **per-node lock** (self-owned,
§8.2) and updated by the **SW txn alongside the structural flip** — *not* by lock-free
atomic fetch-add. It stays **opt-in** (off by default), as it already is, precisely
because it is a known root-contention feature.

### 10.1 Why the lock, not a fetch-add

`nr_keys` is a subtree rollup: every mutation bumps the count of every ancestor up to
the root. The maintenance mechanism is chosen for **causal linearization**, not
throughput:

- A lock-free fetch-add spine-climb has *no single linearization point for the
  aggregate*, and — worse than merely lagging — it can transiently expose
  `parent.nr_keys < child.nr_keys` (the child's `+1` lands before the parent's), a
  structurally impossible state. That inversion is exactly the pathology behind the
  `inv_nr_keys_undercount` history (`reference_ft_inv_nr_keys_exact_listoff_overcount`).
- Placing the spine's `nr_keys` words in the SW txn's committed write-set makes the
  count update **atomic with the structural flip**: a reader that observes the new
  structure observes the matching counts. Snapshot-consistent; the inversion is never
  visible. This is a *stronger* contract than the fetch-add would give, bought with
  throughput.
- **The existing code stays correct.** `nr_keys`'s current `uatomic_store` release /
  acquire-load was only wrong under lock-free concurrent writers (racing
  read-compute-store loses an increment). Under per-node-lock serialization a release
  store — for reader visibility — inside the lock-held SW txn is exactly right. No
  fetch-add churn; the store simply now happens under the owning lock.

### 10.2 Consequence: a deliberate opt-in serialization mode

- **`nr_keys` ON:** a mutation's lock-set is its **full root path** (each ancestor's
  lock, to bump its `nr_keys` in the SW txn). The root is therefore a member of every
  such op's set → they all collide at the root → escalate → serialize. Acquisition
  also grows to O(depth) locks + O(depth) read-set validation.
- **`nr_keys` OFF (default):** lock-sets stay local (leaf + maybe parent); full
  disjoint-key parallelism is preserved. This is the throughput reason it is off by
  default.

### 10.3 Future scalability: shard near the root

Stripe the near-root counts into K per-lock shards (per-CPU or hashed). A mutation
bumps only its shard → root contention ÷ K; reads sum the shards. **Caveat to resolve
when built:** summing K shards is snapshot-consistent only if the reader reads all
shards within one RCU snapshot; otherwise the aggregate read reverts to
eventually-consistent. Sharding thus trades some causal exactness back for
scalability — acceptable for a size *hint*, to be specified per consumer. Deferred.

### 10.4 Bulk ops

A graft / detach of a subtree with cardinality `K` applies `±K` at the boundary and
climbs both the old and new ancestor spines within the SW txn — same discipline,
larger delta.

### 10.5 `rank_stats`-ON uses a single FT-wide mutation lock (decision 2026-07-14)

Because §10.2 already serializes every `rank_stats`-ON mutation at the root, the
fine-grained per-node lock model buys nothing there. So **for the `rank_stats`-ON
build, all mutation ops take one FT-wide writer lock** — no lock-set derivation, no
edge-principle bookkeeping, no abort-and-regrow, and the spine count-fold is atomic by
construction (single writer). Readers stay wait-free (they never take the lock); this
is just classic RCU single-writer for that build. Consequence for §9: **per-operation
lock-sets are derived for the `rank_stats`-OFF (default, fine-grained) build only.** In
that build `ft_insert_commit_arm` is always `arm(0)` — the I6/I7 spine count-fold edges
are never armed — so the spine never enters a fine-grained lock-set.

---

## 11. Migration posture (PROPOSED 2026-07-14 — for sign-off)

> **STATUS UPDATE (2026-07-18):** the FT-wide-lock DROP this section proposes is
> LANDED and soaked — point-ops (§11.4 gate 1600/1600 @16w) AND cross-trie
> (graft/merge/graft_swap, drop-safe under shared-spine contention; rank-stats
> coerced to COARSE). The mechanics + the full cross-trie resolution are in the
> companion note `ft-wide-lock-drop-mechanics.md` (§9). Remaining to flip the
> default FINE→drop: re-run the §11.4 point-op soak on current HEAD. Then the sw
> cutover (net B / §11.6).

The tree on `ft-txn-integ` carries the lock-free-MW machinery this pivot replaces
(the reanchor / skip-resolver / PSO campaigns, baseline `0/1600 @16w`).

RECOMMENDATION: **no runtime coexistence — a trie is entirely `lock`-mode or entirely
`lock-free`-mode, chosen by a flag — and `lock`-mode is built and soaked op-domain by
op-domain, never by racing converted and unconverted ops on one live trie.** That is
*big-bang on the runtime axis, incremental on the development axis*: it keeps the
bisectable per-step regression gate the `0/1600 @16w` baseline makes valuable, without
the transient hybrid §11.1 shows to be unsafe. This synthesis is the answer to the
"big-bang vs incremental" question — it is not one or the other.

### 11.1 Why not naive op-by-op (the coexistence hazard)

The tempting reading of §9 — "the guard sites ARE the lock-set, so convert one op at a
time and let converted and unconverted ops race" — is UNSAFE, concretely. The lock is a
bit in `state` (§8.2). A converted op acquires by stamping that bit; an unconverted peer
backs off *only* where it already value-compares `state` (`ft_flip_txn_guard_parent`).
But §9's own derivation flags that **guard sites UNDER-count**: some edits are a
`record_reserved` CAS on a *slot* with no state guard. During a converted op's
build-invisible window it has stamped `state.lock` on node `C` but not yet changed `C`'s
slot; an unconverted peer's `record_reserved` CAS on that slot still sees the *unchanged*
slot value, succeeds, and sneaks an edit under the held lock. Slot-value-compare does not
see the state lock.

This is the same class of gap the reverted pre-commit experiment hit — a claim that
fences the *commit* but not the *build/reparent plan window* (see
`project_ft_recompact_precommit_reverted`): the **persistent held lock** fences the plan
window, a slot-CAS does not. Closing the gap generically would mean adding a `state`
guard at every `record_reserved`-only site *in the unconverted code* — editing the code
we are trying to leave alone, on every op, before converting any. So: **do not mix at
runtime.**

### 11.2 The flag and the converted-only soak

Select the writer strategy per **group** (or per build, if simpler): `lock` vs
`lock-free`. A `lock`-mode trie runs *only* converted ops; invoking an as-yet-unconverted
op on a `lock`-mode trie is a hard assert, never a silent fallback. Confidence is built
*within* `lock`-mode by restricting the **workload**, not by mixing strategies:

- Convert op-domain `D`. Soak a `lock`-mode trie whose oracle exercises only the
  already-converted domains `{…, D}`. Require `0/1600 @16w`.
- Widen the oracle as each domain converts. The final soak — all domains, the full
  disjoint + shared (`inv_concurrent_writers_shared`, `FT_INV_MW=1`) oracles — is the
  acceptance gate for flipping the default.

Every intermediate commit stays soakable in isolation (bisectable) while the hybrid
§11.1 rejects is never stood up. `lock-free`-mode remains the default and untouched, so
its `0/1600 @16w` is preserved trivially until the very end.

### 11.3 Landing order

Foundational bricks first — the substrate every op conversion consumes ("first
regardless of op order"):

1. **Engine + words.** — *LANDED, and smaller than planned.* Optimistic-acquire / per-FT
   FIFO-lane engine with the **(a)/(b) seam assertion** (§3.1); the state-word terminals
   (§2) per §8.2; the **abort-boundary gate** (§5: byte-for-byte, free reserved) wired as
   the acquire/edit failure path. *The `node-reclaim` bit reserved in `4e943653` was
   UN-reserved at step 3: the tombstone split is withdrawn (§2). What the words actually
   needed was one added terminal — RELEASE `{LOCK|s → s}` — and because a terminal is
   just the MCAS record an op plants, the `locks[]` registry and its drain in
   `ft_flip_txn_commit` / `ft_flip_txn_destroy` were NOT touched at all. The abort-boundary
   gate was already built for the fence and carries the lock unchanged.*
2. **`rank_stats`-ON = one FT-wide lock (§10.5).** — *LANDED (`14545ebb`), as
   `CDS_FT_WRITER_LOCK_COARSE`.* The *simplest* `lock`-mode: single lock, no lock-set
   derivation, classic RCU single-writer. Land it first to exercise the acquire / hold /
   release + reader-wait-free plumbing end to end with a trivial lock-set — a working
   `lock`-mode immediately, and a de-risked substrate for the fine-grained work.
   **Two corrections this step forced, both PROVEN, both in §5:** the lock is a plain
   `cds_fair_mutex` and **not** a txn on the escalation lane (a lock acquire cannot ride
   the lane — circular wait), and it is **dropped and retaken across every
   grace-period wait** (a blocking lock held across a GP deadlocks against its own
   non-quiescent waiters). Scope: single-trie ops; cross-trie ops on two lock-mode tries
   hard-abort until step 6.

Then the fine-grained (`rank_stats`-OFF) op-domains, smallest / most-local lock-set first
(the §9 order):

3. **recompact `{C, P}` (§9.3).** — *LANDED.* Convert first — `FT_STATE_LOCK`
   *already is* this lock (§0), so it is the smallest conceptual delta and it validates
   the seed directly. It is also the shared boundary-copy atom behind insert-grow,
   remove-shrink and the compactor, so converting it has leverage. *As landed: under
   `LOCK_FINE`, `ft_node_recompact` acquires `{C, P}` (+ `{GP}` when `P` is a compressed
   node whose SKIP_X dual it re-encodes) and resolves the surviving members through the
   new RELEASE terminal (§2); the three §4.B guard sites that guarded `P` are REPLACED
   by that lock (insert's relocation republish, remove's two detach republish arms —
   the second only when a recompact actually ran, since its external-promote sub-case
   reaches the same publish with no recompact and no lock).*
   **Two things this step was expected to carry, and does not:**
   *(a) the tombstone split is WITHDRAWN — see §2; what was needed was the release
   terminal, which touches neither the `locks[]` drain nor the CORE_682870
   expected-old contract. (b) the §8.3 layout split is DEFERRED to the end of the
   transition — see §8.3: with the writers still CAS loops, word-sharing costs a
   spurious abort, not a lost update, so the split buys nothing until OPTIMISTIC's
   re-home is retired.*
4. **insert (§9.1)** — leaf-local `{P}` / `{P, CN}`; the grow case now calls the converted
   recompact. — *LANDED in part.* The compressed-split publish-into node (`P` for I-4a,
   `CN` for I-4b) is converted at the shared `ft_insert_publish_or_park` choke point to a
   RELEASE lock, guard-fallback on a miss (a value-swap target, so the guard suffices —
   §9.1). The grow case (I-2) was already carried by step 3's recompact. Deferred with a
   reason (§9.1): I-1 in-place reserve (its own `nr_child` mutation → §8.3) and I-4b's
   skip-dual `P` (unguarded under-count → FT-wide-lock drop); I-3 dup-append has no state
   word to lock.
5. **remove (§9.2)** — introduces the plan → all-or-none acquire → edit → commit **climb
   template** (§9.2); the shared same-key case serializes on `{L}` (the `ft_promote_head`
   crash pulled into contract).
6. **merge_at / graft / graft_swap / bulk (§9.4–9.6)** — cross-domain via decision **(C)**
   (two sequential single-domain commits); add no new atom, composing from the detach +
   attach lock-sets already converted at step 5.

### 11.4 Per-step gate

Each conversion commit: `test_urcu_ft_unit` + `test_urcu_ft_inv` green, then the
16-writer oracle soak at `0/1600` on the converted-only workload (§11.2). A step that
cannot hold `0/1600` does not land — the baseline is the contract, and incremental is
chosen precisely so a regression bisects to a single op-domain. The abort-boundary
skeptic (CLAUDE.md) runs on every commit that adds or moves an abort path.

### 11.5 Reader side is a parallel track

Writer conversion is orthogonal to the **reader** changes. Same-trie `merge_at` under
`lock`-mode additionally needs the per-FT **move sequence counter**
(`doc/design/in-trie-move-seqcount.md`): the single-flip move that step 6 enables is only
reader-coherent once the seqcount lands. That note's first brick — the speculative-key
rejection gate — has already landed; the seqcount itself sequences *with* step 6, not
before it.

### 11.6 End state

When the full-workload soak passes `0/1600 @16w`, flip the default group mode to `lock`,
then delete the `lock-free`-mode machinery (the reanchor / skip-resolver / PSO fixes of
the campaigns this pivot supersedes) in a final commit. Until then the two strategies
coexist **in the source** (behind the flag) but never **at runtime** on one trie.
