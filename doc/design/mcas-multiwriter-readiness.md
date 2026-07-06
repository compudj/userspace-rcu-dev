# MCAS multi-writer readiness — design note (2026-06-24)

Status: ANALYSIS / design of record. No implementation yet. This note reframes
what "MCAS-ready" means for the Fractal Trie and records the two invariants it
requires, the protocol options for the second, and the cost structure — as the
conceptual gate before the edge-conversion campaign.

Related: `doc/design/transactional-flip-latch.md` (the single-writer flip-latch),
`fractal-trie-review-2026-06/MCAS_EDGE_AUDIT.md` (the 2026-06-24 edge audit).

---

## 1. Goal and the single-writer foundation

The `urcu_flip_txn` flip-latch is, by construction, a **single-writer MCAS**: a
mutation builds fresh structure invisibly, records its reader-visible transitions
as `{slot, old, new}` descriptor edges, freezes the edge set, installs the tagged
proxies, flips one selector word (every proxy resolves new atomically), settles,
and defer-frees the old nodes. Readers are passive: RCU keeps the old nodes live
and a type-7 proxy resolves each proxied slot to a consistent old-XOR-new target.

The end goal is **lock-free multi-producer writers**: swap the commit *body*
(plain stores of a tagged proxy + one selector flip, safe only because the app
holds the writer mutex) for a real MCAS that CASes each slot `old -> descriptor`
in sorted address order, flips a status word, and retries on contention. The
reader side (RCU + descriptor resolution) is nearly unchanged.

The key realization of this note: making the commit body an MCAS is **necessary
but not sufficient**. The flip-latch was only ever required to defend against
passive *readers*; under the single writer mutex, `descend -> build -> flip ->
defer-free` is atomic against other *writers*. Remove the mutex and a second
hazard appears that the per-edge model does not address.

There are therefore **two** invariants. Invariant 1 is what the flip-latch
already gives (and what the edge campaign finishes). Invariant 2 is the real
multi-writer gate and is not started.

---

## 2. Invariant 1 — edge-expressibility (necessary)

> Every reader-visible structural / ordered pointer edge must commit through a
> flip-latch descriptor edge, never a bare `rcu_assign_pointer` / release store —
> so the commit body is swappable for an MCAS.

**Status: MET — campaign closed, re-certified gap → 0 (2026-06-26).** The
2026-06-24 audit classified 274 in-scope store sites and found **~62
reader-reachable edges still publishing via bare stores** (plus 7 compaction
sites); the conversion campaign closed them all, and the re-certification
(`fractal-trie-review-2026-06/MCAS_EDGE_AUDIT_2026-06-26f.md`, 39 audited sites)
counts the remaining gap at **zero**. The root causes were:

- the systemic **list-OFF bypass** (every list-ON txn path has an `else` that does
  the same edge bare);
- the forward-publish primitive `_ft_publish_to_parent(..., rec==NULL)`
  (ft-helpers.h:1935, + the SKIP_X dual at :1900);
- the parent-back-pointer primitive `ft_set_parent` (ft-helpers.h:2042/2061/2078/2112);
- `ft_unchain_node` head-promotion; dual-pointer (cn->child + SKIP_X) torn windows;
- whole-trie root/endpoint swaps (detach, merge root-src, graft_swap);
- the `ft_node_recompact` republish sweep (also reached by ordinary insert/remove
  tier changes, not just `cds_ft_compact`).

Nearly all were mechanically closable by routing through the existing
`urcu_flip_txn` / `ft_ord_cell_flip` / `ft_glue_record_back_edge` machinery, whose
`nr==1` bare-store fast path makes the conversion **zero runtime cost**. The
campaign that closed them is phased in the audit file.

**Completeness oracle.** Because every conversion is behavior-identical under one
writer (the `nr==1` path *is* a bare release store), the functional suite **cannot
detect a missed conversion** — green tells you nothing about coverage. The
completeness check is the **edge audit re-run** (gap count -> 0), performed
2026-06-26 with gap 0; a lightweight standing static guard (grep mutation
modules for bare reader-visible stores against an allowlist of proven
build-invisible sites) remains recommended-but-unbuilt (decision item 4) so the
invariant cannot silently regress.

---

## 3. Invariant 2 — freeze-on-free (the real multi-writer gate)

> Every node an op frees must, at commit, be made to fail a concurrent writer's
> CAS on it, while still resolving for in-flight RCU readers; and a writer's
> commit must re-validate that its target was not frozen out from under it.

### 3.1 The hazard

`descend -> commit` is not atomic with respect to a concurrent op that removes the
target's reachability. Concretely:

- Remover **R** decides to prune / collapse / detach a branch because, *as it read
  it*, the holder went empty or single-child.
- Inserter **I** has already descended to a node **P inside that branch** and built
  a descriptor for `&P->slot`.

They conflict — but on **disjoint words**: I CASes `&P->slot`; R CASes
`&grandparent->slot_to_P` (and defer-frees P's chain). Per-slot MCAS only
serializes contention on the *same* word, so **both CASes succeed**. I's node now
hangs off a detached, about-to-be-freed P: a **lost key** and a **use-after-free**
when R's grace period reclaims the chain.

This cannot happen today (single writer mutex serializes R and I). It is purely a
multi-writer hazard — and it is the textbook lock-free-tree obstacle (Ellen,
Fatourou, Ruppert; Natarajan–Mittal edge marking; Fraser's MCAS trees): an
operation's *validity* (its target still reachable) must be welded to the words it
CASes, or a disjoint-word structural change silently invalidates it.

### 3.2 The fix and the inverse it also catches

Make every freed node, at commit, fail a concurrent writer's CAS on it. The
elegant case (slot tombstone, below) makes the formerly-disjoint words **shared**:
R freezes `&P->slot`, so I's `CAS(&P->slot, expected=old, …)` fails its compare and
I re-descends from a still-live ancestor and retries. The **inverse** is handled
for free: if I's insert lands first (P gains a child), R's freeze-CAS on that slot
fails -> R re-reads P, sees it is no longer single-child, and **abandons the
collapse**. The collapse precondition is re-validated by the very same contention.

---

## 4. The freeze mechanism — a fork

### A. Slot tombstone (tag the internal cluster pointers)

At/after detach, each freed node's child-slots become a **forwarding** tombstone:

```
tombstone(slot) = TAG | old_child
```

- **Readers resolve through it** to `old_child` and keep traversing the
  frozen-but-RCU-live subtree. This is mandatory: a reader that entered before the
  detach must, by linearizability, still see the *pre-detach* subtree in full. A
  bare "dead" sentinel would dead-end its traversal mid-subtree (a tree that never
  existed) — **the tombstone must forward, not poison.**
- **Writers** see their `CAS(&P->slot, expected=old_child)` fail (the slot holds
  the tombstone) -> re-descend and retry. Same-word, **automatic, no writer
  cooperation** beyond the rule "my CAS failed and the slot is reserved-tagged ⇒
  retry."

This is structurally the flip-proxy pattern (resolve-to-a-target for readers,
tag-visible for the controller).

**Tag budget.** The low nibble is nearly full — bit 0 = internal/cell, bits 1–3 =
type index, compressed = bit 1 / bit 0 clear, external = low bits clear, skip =
high bits — and **type-7 (`0xF`) is the one clean spare, already the flip-proxy.**
So a forwarding tombstone most cheaply **reuses type-7 as a proxy variant** (a
"detached proxy" with `ptr[0]` = the frozen child): readers already resolve type-7,
so the reader path is unchanged; the genuinely new thing is purely **writer-side**
("a type-7 proxy in my CAS target ⇒ abort", which a writer needs anyway for any
in-flight commit). No new pointer encoding is burned. Cost: one proxy per frozen
slot.

### B. Node mark (mark the node, not the slots)

A `detached` bit in node metadata. Child pointers stay byte-intact, so readers are
**untouched (zero added cost)**. The writer's MCAS must **include the target node's
mark word** (expected = live); a concurrent detach that set it fails the writer's
commit. Requires the writer to deliberately read-and-include the mark and
re-validate.

### Trade

| | A. slot tombstone | B. node mark |
|---|---|---|
| writer | automatic (same-word CAS) | must read + include the mark word |
| reader | resolves tombstone on internal child reads **during a detach** | zero added cost |
| pointers | child slots carry a forwarding proxy transiently | clean |

The decision hinges on the reader column. This structure is read-biased and its
whole proxy design minimizes reader cost; (A) extends proxy resolution onto the
**hot internal-descent path** (every child fetch must tolerate the tag) for the
duration of a detach, whereas (B) keeps readers free and pays on the rare writer.
(A) wins on writer simplicity and "no cooperation"; (B) wins on reader cost.

### DECISION (2026-06-24): (A), encoded as an `old == new` flip-proxy

A tombstone is simply **a type-7 flip-proxy whose `ptr[0] == ptr[1]` == the frozen
child.** This reuses the existing proxy machinery with no new tag and no new node
metadata word, and (A)'s "no writer cooperation" (the freeze word *is* the
inserter's CAS target) is the deciding property — the reader cost is bounded to
the detach window and only on the specific subtree a reader was already inside.

Why the encoding satisfies every requirement:

- **readers forward** — `ptr[selector]` is the frozen child for *either* selector,
  so an in-flight reader resolves through it and keeps traversing the
  detached-but-live subtree (before and after the flip);
- **writers fail** — a writer's `CAS(slot, expected=raw_child)` sees the tagged
  proxy and fails -> re-descend + retry;
- **persists ("never untag")** — see the commit rule below;
- **composes** — one detach txn carries the boundary unlink edge (`old != new`,
  flips + settles normally) plus the subtree's tombstone edges (`old == new`); the
  single selector flip atomically activates the unlink *and* the tombstones.

**Commit rule.** Settle is the "untag" step (`*slot = ptr[1]`, dropping the proxy).
A tombstone must keep its proxy, so **commit flips but skips settle for any latch
with `ptr[0] == ptr[1]`** (the flip is a harmless no-op for it). This guard must
apply in **both** the multi-edge settle loop **and** the `nr == 1` bare-store fast
path (a lone tombstone must not be untagged by the shortcut store).

**Safeguard — `old == new` must be intentional only.** If an *ordinary* edge ever
recorded `old == new` (a coincidental no-op publish), commit would silently leave a
permanent tombstone on a **live** slot (that subtree becomes unmutatable; the proxy
leaks / UAFs). So: `assert(old != new)` on the normal record path (a no-op edge is
already a latent bug), and a dedicated `urcu_flip_txn_record_tombstone(slot)` as the
*only* producer of an `old == new` latch. (A 1-bit `tombstone` flag in the latch is
the more explicit alternative if the assert proves noisy.)

**Lifetime.** A tombstone proxy lives inside a slot of a dead node, so it is only
reachable while that node is. The txn (proxies + group) and the dead nodes are
defer-freed from the **same commit grace period**, so after that GP no reader can
reach a tombstoned slot — proxy and node die together. Constraint: *tombstone
proxies reclaim no earlier than the nodes whose slots hold them* — automatically
satisfied by the existing same-GP reclaim.

### DECISION REVISED (2026-06-24): lean to (B) — a metadata **seqcount** word folded into the flip-latch record

> **SUPERSEDED (2026-06-26) — the seqcount is RETIRED.** See "DECISION FINAL"
> below: once the two in-place node mutations are eliminated, a one-way `deleted`
> flag replaces the seqcount entirely. This subsection is kept for the reasoning
> trail (why a disjoint freeze word *would* need a count if any in-place mutator
> survived).

Reconsidering (A): a forwarding tombstone *per freed slot* means a wide popcount
node (up to 256 child slots) contributes up to 256 proxy edges to a single detach
txn — inelegant, a large freeze set, and (A) extends proxy resolution onto the
**hot internal-descent path** for the detach window. (B)'s metadata word keeps
child pointers byte-intact (**zero reader cost**) and bounds the freeze to **one
word per freed node**. The objection to (B) was "writers must cooperate (read +
include the mark)" — but under the campaign every writer already commits through
the flip-latch record, so including one more word is the *existing* mechanism, not
new cooperation. So (B) is now the leading direction for the general (wide-node)
case; (A) may still win locally where a freed node is narrow and the txn already
tiny.

**The mark must be a seqcount, not a dead bit.** In (A) the inverse race
(insert-lands-first ⇒ remover abandons the collapse, §3.2) was free because the
freeze word *is* the inserter's CAS target — same-word contention. In (B) the
freeze word (metadata) is **disjoint** from the slot the inserter CASes, so that
automatic catch is **lost**: a remover that set a bare `dead` bit would still
succeed even though an insert just added a child to a free slot — losing it (the
I-then-R hazard, the dual of §3.2). Fix: make the word a **monotonic sequence
count** that *every op mutating the node bumps*; the remover **validates the seq it
read** at prune-decision time (`expected = seq_at_decision`). An intervening insert
bumped the seq ⇒ the remover's commit fails its compare ⇒ it re-reads the node and
abandons the collapse. (The seq is the writer/writer serialization; the dead state
is freeze-on-free. Both live in the one word.)

**Encoding.**

```
state = (seq << 2) | phase      phase ∈ { LIVE = 0, FLIP = 1, DEAD = 2 }
```

- insert / child-add: `expected (seq<<2 | LIVE)` → `((seq+1)<<2 | LIVE)`
- remove / collapse:  `expected (seq<<2 | LIVE)` → `(seq<<2 | DEAD)`  (validate seq, freeze)
- recompact (= fused remove-old + install-new): deads old N's word; the fresh N′
  starts at `0 | LIVE`.

**Scalar in the flip-latch.** A metadata word is a scalar; it cannot hold a
tagged-pointer proxy, so the in-band `FLIP = 1` phase is its transient ("a commit
is in flight on this word") — the scalar analogue of the type-7 proxy. Because the
word is **writer-side only** — lookups descend via the child/external pointer
slots, which carry their own flip proxies, and never read the state word — `FLIP`
only has to be interpreted by *another writer* mid-commit (help/retry, standard
MCAS); it need not carry old/new for readers, which is exactly what lets a bare
in-band sentinel suffice (a pointer proxy would have to carry both).

**seq width / ABA.** The seq need only be unique across the in-flight window. RCU
already forbids freeing+reusing a node until a grace period elapses, so a modest
high-bit field is ABA-safe in practice.

**The campaign bridge (why it can land before MCAS).** Recording the state word as
one more flip-latch edge is **behavior-identical under the single writer** (a
redundant scalar store / no-op validate) and becomes the real CAS-with-expected in
the MCAS word-set later. So the field + the every-mutating-op-records-it rule can be
added and validated under the *current* suite (the gate stays green) well before any
MCAS commit body exists — the same "express now, swap the commit later" discipline
Invariant 1 uses.

### RECONSIDER (2026-06-26, Mathieu): a per-node `deleted` flag in place of the seqcount — *if* in-place node mutation is first eliminated

Two coupled refinements to weigh against the seqcount decision above.

1. **Eliminate the last in-place node mutation: the insert-without-rerank
   occupancy-bitmap update.** A pigeon / popcount insert that finds spare capacity
   at the correct rank publishes its child-slot through a flip edge (Invariant 1),
   but *also sets the node's occupancy bitmap bit in place* — a bare store on a
   **live node's metadata word**, outside any descriptor. Because it is **not a
   pointer edge**, Invariant 1's edge-expressibility does not cover it, and the
   Invariant-1 edge audit (§2) does not count it. It is exactly the Invariant-2
   disjoint-word hazard at metadata granularity: a concurrent writer that validates
   the node to collapse / recompact it CASes a *different* word and never observes
   the bitmap change ⇒ a lost insert (and, if the bitmap is one word that two
   inserters RMW, a lost update outright). The fix is to bring this site into the
   build-invisible + flip model like every other tier change: **recompact the
   node** — build a fresh node carrying the new entry *and* its bitmap, flip the
   parent edge — rather than mutating the bitmap in place. Cost: the cheap O(1)
   in-place insert tier becomes an O(node) alloc-and-copy recompact under
   multi-writer (a §5 cost-tier consequence, paid only on a shared trie; an
   exclusive trie keeps the in-place store, §5.2).

2. **Then the §4.B coherency word can degrade from a seqcount to a plain `deleted`
   flag.** The "must be a seqcount, not a dead bit" argument above rests on
   *in-place child-adds existing*: the freeze word (metadata) is disjoint from the
   slot the inserter CASes, so a bare `dead` bit would miss an insert-lands-first
   and lose it (the I-then-R dual of §3.2), forcing a monotonic count that every
   in-place mutator bumps and the remover validates. **Remove in-place mutation
   (refinement 1) and that premise is gone:** every mutation becomes a whole-node
   replacement via the parent edge, so two ops touching the same node now contend
   on the **same** word (the node's state word / its parent edge), not disjoint
   words — the MCAS serializes them directly and the loser re-plans. The version
   counter was only ever needed to detect the in-place child-adds that no longer
   exist; a one-way `deleted` latch (`LIVE → DEAD`, set at retire, validated by any
   op that targets the node) then suffices for **both** freeze-on-free and
   writer/writer serialization. The `FLIP` transient and the same-GP
   proxy/node reclaim from the seqcount encoding carry over unchanged to the flag.

*(Refinement 2 is a synthesis of the two notes — its validity hinges on refinement
1 being total, i.e. that NO reader-visible node mutation remains in place once the
bitmap site is recompacted. Confirm that completeness before adopting the flag over
the seqcount; the seqcount remains the safe default while any in-place mutator
survives.)*

### DECISION FINAL (2026-06-26, Mathieu): the one-way `deleted` flag — the seqcount is retired; the dup-chain is refinement-1's second site

**Decision: adopt the one-way `deleted` latch (`LIVE → DEAD`, §4.B above); retire
the seqcount / version counter entirely.** The version was only ever the patch for
a *disjoint* freeze word (metadata word ≠ the inserter's CAS target). Once no
in-place node mutation survives, every op that touches a node contends on the
**same** word as a concurrent remover, the disjoint-word catch is no longer needed,
and the count collapses to a bit.

Refinement 1 ("eliminate every in-place node mutation") therefore has **two** sites,
not one — and it is total only when **both** are converted:

1. **The occupancy-bitmap insert-without-rerank** (refinement 1 above): recompact
   the node instead of setting the bitmap bit in place.

2. **The duplicate-node chain** — the second in-place node mutation, missed by the
   §2 edge audit because (like the bitmap) the hazard is not where Invariant 1 looks.
   *The dup-chain is part of the trie, not a side structure: its mutations must fold
   into the op's transaction commit.* The concrete hazard is the §3.1 disjoint-word
   race re-instanced in the chain:

   - A duplicate **append** is an in-place mutation of a *live* node:
     `tail->next : NULL → D`.
   - A **remover of that tail** unlinks it by CASing the **predecessor's** word
     (`pred->next : tail → tail->next`) and defer-frees `tail`.
   - `tail->next` and `pred->next` are **disjoint words** ⇒ per-slot MCAS serializes
     neither ⇒ both CASes succeed ⇒ **D is appended onto a freed `tail`: a lost key
     and a use-after-free** — the exact §3.1 obstacle, at the chain.

   **Fix — fold the chain into the op transaction, same-word (§4.A, applied to the
   chain):** the remover **tombstones the freed node's *own* `next`** (the word the
   appender CASes), *recorded as an edge in the remove txn* — not a bare bit-set.
   Then an append-after-a-dying-node finds its expected-value CAS on `tail->next`
   fail against the tombstone and re-descends; the inverse (append-lands-first ⇒ the
   tail is no longer the chain end ⇒ remover re-reads) is caught by the same
   contention. The append's `next` store is already an expected-value flip
   descriptor (Invariant-1 class-E, `ft_chain_next_flip`); head/promotion changes
   already fuse the chain relink with the trie slot + cell swap (class-D,
   `ft_promote_head`). What remains is to make the **remove-side tombstone a txn
   edge on the freed node's own `next`**, so the freeze contends in the MCAS
   word-set rather than sitting outside it as a plain store.

**Why this needs no seqcount and no separate chain mechanism.** The chain leaf's
freeze and the internal node's freeze become the **same one-way `DEAD` mark in two
different words**:

| Node kind | freeze word | reader cost |
|---|---|---|
| internal / compressed | a `deleted` bit in node metadata (§4.B) | zero — lookups never read it |
| duplicate-chain leaf | the `CDS_FT_NODE_REMOVED_FLAG` tombstone in `next` (§4.A) | already paid — readers already mask it in `cds_ft_node_next_rcu` |

For the chain leaf the §4.A "same-word, no cooperation" property is the *natural*
fit (the inserter's CAS target **is** `next`, and the reader cost §4.A worried about
is already a single mask that exists today), exactly where §4.A's wide-node
objection (256 proxies) does not apply. Neither word is a count: both are a
monotonic `LIVE → DEAD` latch with the in-band `FLIP` transient, validated by any op
that targets the node, freeing the node and its txn from the same commit GP.

**Precondition stands.** This is sound **only when refinement 1 is total** — both
the bitmap recompact **and** the chain fold landed, with a re-audit confirming no
other reader-visible in-place node-word mutation survives (the §2 edge audit covers
pointer edges, not metadata/word mutations, so it must be re-run with that lens).
Until then the seqcount remains the safe fallback for any surviving in-place mutator.

**Campaign bridge (unchanged).** The `deleted` flag and the chain's remove-side
tombstone-edge both record into the flip-latch: behavior-identical under the single
writer (a redundant store / no-op validate), real CAS-with-expected in the MCAS
word-set later — so both land and validate under the current suite before any MCAS
commit body exists.

### Why a seqcount is not a general solution (committed summary vs out-of-commit validation)

The DECISION-REVISED step ("must be a *seqcount*, not a dead bit") was wrong for a
reason worth recording, so the idea does not resurface. There are **two categorically
different** uses of a seqcount, and only one of them works:

- **A seqcount as a *committed summary* of committed words — works.** The count is
  bumped *inside the same MCAS* as the word set it summarizes, so it is
  consistent-by-construction with those words. A reader/writer checks the count
  instead of re-scanning the whole set; it carries no information the commit did not
  already carry. This is a pure optimization over the scan (e.g. "did this node's
  child set change since I snapshotted it").

- **A seqcount to retroactively validate an update that is *not* in the commit —
  does not work.** If a mutation `X` is an in-place store outside the txn, bumping a
  count next to `X` and having another op "validate the count it read" just relocates
  the §3.1 disjoint-word race onto the count: `X`, the bump, and the validating read
  are three separate steps with no atomicity binding them. You cannot validate your
  way to atomicity for something the commit does not contain. The DECISION-REVISED
  justification — a monotonic count to detect *disjoint, in-place* child-adds — was
  exactly this second kind, which is why the DECISION FINAL retires it.

**The rule (no third bucket).** Every reader-visible word a mutation touches is
**either** part of the atomic commit (truth, exact) **or** a provably-benign racy
hint that nothing relies on for correctness. A seqcount does not create a safe
"out-of-commit but validated" middle category. So an out-of-commit update has exactly
three sound dispositions — **commit it** (bring it into the txn word-set; a scalar
word rides the flip-latch via the reserved low-bit phase tag, §B), **eliminate it**
(don't store it — derive the value on demand from the committed words), or **prove it
benign** (a pure hint, with correctness depending on it *nowhere*) — and a seqcount is
on none of them.

Mapping the node-word mutations to that rule:

- **insert occupancy-bitmap set** — was out-of-commit; recompact-on-insert (§4.1)
  **eliminates** it by making the change a whole-node replacement committed via the
  parent edge. The correct move under this rule.
- **remove-side `nr_child--`** — out-of-commit today. Fix is one of: *commit it*
  (low-bit-tagged scalar edge → exact), *eliminate it* (the committed bitmap/pointers
  are the truth; derive the count when needed), or *prove it benign* (a pure recompact
  *trigger* hint, which first requires decoupling recompact sizing from it — §4.2).
- **pigeon bitmap clear** — same: the sticky-hint design (§4.2) is the *prove-benign*
  path (pointers are truth, readers already tolerate a stale bit), **not** a seqcount.

### 4.1 Implementation gate: `FEATURE_FT_INSERT_IN_PLACE` (recompact-on-insert)

The occupancy-bitmap insert (site 1) is retired behind a build gate:
`FEATURE_FT_INSERT_IN_PLACE` is **opt-in**, and **recompact-on-insert is the
default** (the default was flipped once the MW campaign landed;
`-DNO_FEATURE_FT_INSERT_IN_PLACE` still force-disables an explicit opt-in). By
default, a new-occupancy `ft_*_node_set_nth` on a *live* node returns `-ERANGE`
instead of setting the bitmap bit in place, so the wrapper routes the insert
through `ft_node_recompact(ADD_SAME)` — the same whole-node rebuild a non-tail
insert already takes. Both popcount and pigeon recompact uniformly. Behavior-
identical under one writer, so the default build validates the MCAS-ready shape
(no in-place bitmap mutation on the insert path) on the current suite. Cost:
the O(1) in-place insert becomes an O(node) alloc-and-copy; re-enabling the
in-place fast path for a single-writer / exclusive trie is a future runtime
perf knob (§5.2).

### 4.2 Hint vs truth — the pigeon bitmap and `nr_child` (deferred refinements)

Two metadata words are mutated in place outside the structural pointer edges:
the **occupancy bitmap** and the per-node **`nr_child`** count. Both are
candidates for the same insight — *demote them from truth to hint, with the
pointers as the sole truth* — which avoids the whole-node recompact for the
cases that are not a genuine re-rank.

- **Pigeon bitmap → sticky hint.** A pigeon slot is direct-indexed, so the
  pointer store is already a clean flip edge and the bitmap is only an
  occupancy *hint*: `ft_pigeon_node_get_nth` reads `data[n]` directly, and the
  directional scan already rescans past a set bit whose slot is NULL ("source
  of truth is the pointer load"). So the pigeon recompact-on-insert is
  avoidable: make the bit **sticky** — set with an atomic OR (concurrent
  setters of other bits in the word don't lose updates), never cleared in
  place; a delete leaves the bit set and only a recompact rebuilds a clean
  bitmap. (For popcount the bitmap *is* the rank index = truth, so it must be
  rebuilt — no hint option.) Needs: relax the verify cross-check to
  `slot_set ⇒ bit_set` only, drop the delete-side `cds_clear_bit_relaxed` /
  deferred `pub->pigeon_bitmap` clear, and — to bound the stale-bit scan cost
  — optionally trigger a cleanup recompact once
  `popcount(bitmap) − nr_child` (stale bits) gets high.

- **`nr_child` → commit it (the unified per-node state word).** `nr_child` is
  `++`/`--` in place and drives the recompact tier decision. **Recompact-on-insert
  already removes the `++`:** a new-occupancy insert rebuilds the node, so its
  count lands in the fresh node's metadata — no in-place bump on the live node.
  The surviving in-place mutation is therefore **remove-side only**: a non-shrink
  delete stores `NULL` into the slot (a flip edge — already MCAS-fine) but does
  `metadata->nr_child--` in place (`ft_popcount_node_replace_ptr` /
  `ft_pigeon_node_replace_ptr`); the pigeon delete also clears its hint bit in
  place (popcount instead *soft-deletes*: it leaves the bit set and only
  decrements `nr_child`, so popcount's bitmap is already non-mutated on delete).
  So remove-in-place needs no "recompact-on-remove" of the structure — only the
  metadata words must be tamed.

  **DECISION (2026-06-27, Mathieu): commit `nr_child`, do not recalculate it.**
  Per the "commit it / eliminate it / prove it benign" rule, take the *commit it*
  disposition: fold the `nr_child` update into the op's flip-latch commit so it is
  exact and atomic with the structural edges. Eliminating it (re-deriving the
  count from the bitmap/pointer scan on every recompact-trigger decision) was
  rejected — the scan is the cost we are avoiding, and an exact committed count is
  cheaper to consult.

  **Encoding — one `uintptr_t` per-node *state word* holding three things:**
  - **bit 0 — proxy:** the in-band flip marker. A scalar word cannot hold a
    tagged-pointer proxy during the FLIP window, so bit 0 set means "this word is
    mid-flip — resolve via the latch"; this is what lets the scalar ride the
    flip-latch as a real committed edge (not a side-band store).
  - **bit 1 — tombstone:** the one-way `LIVE → DEAD` `deleted` latch of §4.B
    (freeze-on-free), validated by any op targeting the node.
  - **bits 2+ — `nr_child`:** the live-child count (9 bits suffice, max 256).

  This *unifies* the §4.B node mark and the committed count into a **single**
  word: every node-state change (count ± and/or mark-dead) CASes this one word via
  the latch — exactly the DECISION-FINAL "every op on a node contends on the same
  word," now made concrete. Because the count is exact-and-committed, the
  recompact may size directly from it (no separate true-occupancy scan, no
  sizing-decouple needed); the cost is that two independent same-node deletes
  serialize on this word (acceptable — they already must agree on the node's
  liveness). Replaces the retired seqcount: the count rides the SAME word as the
  freeze mark, but it is a *committed* value, not an out-of-commit-validated one.

The fully-uniform alternative (the §4 baseline) is to **recompact on every
structural change** including delete, so the node is *rebuilt, never mutated* and
all of a node's mutations contend on its parent edge. That is the simplest
correctness story but the most expensive (every delete is O(node) too); the state
word above buys back the O(1) remove for the non-re-rank case. Exclusivity (§5.2)
gates all of it: an exclusive trie keeps the cheap in-place mutate-count path.

**Status:** **implemented** — the recompact-on-insert default (§4.1), the unified
`nr_child`+tombstone+proxy state word, and the pigeon sticky-hint bitmap. The
freeze MARK is now recorded at every node retire (the internal-node state bit
and the chain-leaf `next` tombstone, §4 DECISION FINAL). With recompact-on-insert
(§4.1) as the default, no reader-visible in-place node-word mutation survives —
refinement 1 is total. The **validate** side is well underway: the MCAS commit
body exists and runs concurrent (`urcu_txn` / `urcu_mcas`), and the Phase-4.2
atomic-detach campaign folded the marks into their unlink commits across the
whole dup-chain surface (interior/head × remove/replace), the single-leaf
detach/remove paths, and the graft/merge/detach family, with state-word
validate edges guarding the `external_nodes` publishes/clears. Remaining: the
standalone whole-chain / freeze-leaf marks on the detach and remove-all paths
(5 lone-edge `ft_*_mark_removed_flip` call sites in ft-remove.h) plus the
list-off pub-less lone store — to be folded into their unlink commits, or
proven safe as-is and allowlisted (mark-BEFORE-unlink is the conservative
order: a dead-but-still-linked window only costs racing writers a spurious
abort/retry; the reverse order would be unsafe).

### 4.3 Commit granularity — single-word edges vs node identity

An MCAS edge is one word, so the per-node mutations decompose into exactly two
commit units, and nothing falls outside:

| unit | width | how it commits |
|---|---|---|
| state word (`nr_child` + tombstone + proxy) | 1 `uintptr_t` | its **own** flip-latch edge |
| node body (occupancy bitmap + pointer array) | many words | the **parent-pointer** flip (whole-node replacement / recompact) |
| pigeon hint bitmap | many words | not committed — per-word atomic-OR, pointers are truth |

A structure **wider than a `uintptr_t` cannot be its own committed edge**; it
commits by riding a node's *identity* instead — the recompact builds the whole
new node (its multi-word bitmap included) build-invisibly, then flips the single
parent-pointer word, so a reader sees the old node or the new node, never a torn
bitmap. *This is why a popcount node must recompact:* its bitmap is wide truth,
and whole-node replacement is the only way to commit a wide update atomically.
The two units compose per op — an insert (recompact) carries both the new body
and the new count in the fresh node (one parent flip); an in-place delete
changes the body by one `pointer→NULL` edge and the count by the state-word edge
(two single-word edges, same commit). The wide bitmap never needs its own edge.

**Possibility (design note, not chosen): committing the bitmap word-by-word.**
If the occupancy bitmap ever *had* to be part of the commit directly (rather than
via node identity), it could be: reserve the in-band proxy/FLIP tag bit in
**every** `uintptr_t` of the bitmap — `{ 63-bit bitmap fragment, 1 tag }` per
word — so each word rides the flip-latch as its own committed edge. A 256-bit
pigeon bitmap then occupies **5 words instead of 4** (⌈256/63⌉). The cost is that
**every bitscan op gets much more complex**: `find_next/prev_bit`, popcount,
`test_bit`, set/clear all have to mask the per-word tag and index in non-power-of-2
63-bit strides instead of clean 64-bit words. So it is feasible but unattractive;
the chosen path keeps wide truth committing via node identity (recompact) and the
pigeon bitmap as a per-word hint.

---

## 5. Cost tiers and exclusivity

### 5.1 Tiers (the freeze set = the nodes the op frees)

- **Tier 1 — point mutators.** Freeze set ≈ the directly-replaced node(s). Cheap;
  fully lock-free.
- **Tier 2 — collapses / prunes.** Freeze set = the collapsing **spine**,
  depth-bounded (≈ key length). Cheap.
- **Tier 3 — bulk detach / merge-unlink of a populated subtree.** Freeze set =
  the **whole subtree**, O(size). Likely retains a coarser serialization (a writer
  epoch / per-subtree lock) or a heavier marking protocol even in an otherwise
  lock-free world.

So "MCAS-ready" is really three tiers, not one uniform lock-free surface — and the
tiers are **conditioned on exclusivity** (below).

### 5.2 Exclusivity gates both invariants

`ft->exclusive` is already a first-class property: an exclusive trie carries no
concurrent RCU readers by construction, which is why detach/graft/merge skip the
drain on the exclusive side (`if (!ft->exclusive) ... update_synchronize_rcu()`)
and free synchronously. The natural extension under multi-writer: an exclusive
trie has no concurrent **writers** either — it is thread-private (a freshly
allocated staging trie, or a subtree already quiesced and handed to one thread).

On an exclusive trie **both invariants are vacuous**:

- Invariant 1 (edge-expressibility) — the edges are not reader-visible, so plain
  stores suffice; no descriptor, no MCAS.
- Invariant 2 (freeze-on-free) — no concurrent writer can race, so no freeze; and
  no reader, so frees are synchronous (no grace period).

So **the requirements for bulk ops differ by exclusivity**, and a bulk op is
two-sided:

- **Exclusive / transient side** (build a staging cluster; restructure a
  detached-and-quiesced subtree): plain stores, free, *regardless of size*. This is
  exactly what the build-invisible pattern already exploits.
- **Shared side**: only the boundary edge(s) crossing into or out of the live trie
  — the publish that makes the staging reachable, the unlink that removes a subtree
  from shared — are reader-visible and writer-contended, and need the full
  Invariant-1 + Invariant-2 treatment.

### 5.3 Consequence: the expensive direction is *becoming* exclusive

The Tier-3 "O(subtree)" cost is borne **only when the subtree being restructured
is shared**. Producing structure onto an exclusive side is Tier-1 no matter how
large it is — only the O(1) boundary publish is contended. The genuinely expensive
direction is the reverse: **transitioning a shared subtree to exclusive** —
quiescing every concurrent writer that may be *inside* it. A writer deep in the
subtree does not touch the boundary edge, so freezing only the boundary does not
evict it; quiescing requires either subtree-wide freezing or a per-subtree
"sealed" epoch that in-flight writers check and back off on. That — not the
restructure — is the irreducible Tier-3 cost.

Today detach makes a subtree exclusive by draining **readers** (`sync_rcu`). Under
multi-writer it must additionally quiesce **writers**; freeze-on-free is precisely
the writer-quiescing half (freeze -> racing writers' CAS fail -> they retire and
retry elsewhere). So **exclusive = reader-drained AND writer-quiesced**; once a
subtree is exclusive, all further work on it reverts to free plain stores.

---

## 6. Compaction (relocation) — same shape, in scope

`cds_ft_compact` / `ft_node_recompact` relocation = copy node N (at A) to a fresh
A′, re-parent its live children to A′, flip the inbound references (parent slot +
SKIP_X dual) to A′, defer-free A. This is the **same build-copy / flip-references /
defer-free shape as merge/graft**, so its edges are MCAS-expressible — and the
decision is to migrate them onto the txn (Phase 6 of the campaign), one small txn
per relocated node (forward edge + child back-pointers).

The extra hazard vs. merge/graft: relocation changes a node's **address**, and the
node's own child-slots are live MCAS targets, so a writer holding the old address
CASes the dead copy. This is exactly **Invariant 2** (the slots being moved away
are "freed") and is closed by the **same freeze**: tombstone/forward A's slots so a
racing writer's CAS fails and it re-resolves to A′. Note this means the relocation
must CAS-freeze every internal slot of the moved node — strictly more work than
merge/graft, which never touch the moved nodes' internals.

Therefore compaction is **not fundamentally exclusive** under MCAS; whether to run
it concurrently with mutators (paying the freeze + reader proxy-resolution cost) or
as a brief writer-exclusive maintenance pass is a **performance** choice, not a
correctness barrier.

And this is itself gated by §5.2: compacting an **exclusive** trie needs neither
freeze nor MCAS edges — plain relocation as today. The freeze + forwarding cost is
incurred *only* when compacting a **shared** trie concurrently with live mutators.
So a practical split is: relocate freely on exclusive/transient tries; on shared
tries either accept the freeze cost or take a brief writer-exclusive window.

---

## 7. The structural exception — inherently two-commit ops

Cross-trie operations — graft's **src-disappear** side and `cds_ft_merge_at` — are
*inherently* two-commit: the jump-out / parent-backtrack invariant requires a
`synchronize_rcu` **between** the src-unlink and the dst-publish, so a reader can
only ever progress old -> merged, never observe both or neither. These cannot
become a single atomic edge-set; the MCAS-ready shape is a documented **two-txn
protocol**, not an Invariant-1 gap.

### 7.1 commit1 is irrevocable; commit2 is restartable, never undone

The structure is `commit1 (src-unlink) -> synchronize_rcu (drain) -> commit2
(dst-publish)`. The drain is a **reader** temporal invariant (jump-out), so once it
runs, src-empty has been globally observed and **commit1 can never be rolled back** —
linearizability forbids un-emptying src after a reader saw it empty. Therefore, past
commit1 the op can only be **driven forward**, never aborted.

Under multi-writer MCAS this is the decisive constraint: commit2's MCAS can lose its
CAS to a concurrent writer that reshaped the dst graft point. "Abort commit2" must
therefore mean **re-plan and retry**, not undo and not give up — commit2 must always
*eventually* complete. (The existing "no rollback after the src retire" property is
not a limitation to remove; it is exactly correct for MCAS too.)

What makes unconditional forward progress tractable: **after commit1 + drain the
payload is an exclusive, op-owned, RCU-live cluster** (the same shape `cds_ft_detach`
produces). So commit2 is "graft an *exclusive* subtree into dst," whose only
contended surface is the **single dst boundary edge**. On a conflict (CAS fails, or
the target was frozen / relocated by a concurrent op), commit2 **re-descends dst
fresh and retries the boundary MCAS**. Progress holds because the payload can wait
indefinitely (live, owned, untouchable by others) and dst always admits *some* valid
landing for the keys.

### 7.2 The crux: reserve capacity from the payload, re-record targets from dst

A restartable commit2 collides with pre-reservation (§3/§6, the
`urcu_flip_txn` built before commit1 so commit2 cannot OOM): pre-reservation assumes
the commit2 **edge set is fixed and known before commit1**, but a retry may re-plan
against a *changed* dst, so the edge *targets* differ per attempt. The resolution is
to **split reserve from record**:

- **Reserve *capacity* — payload-derived and invariant.** commit2's edge *count* is a
  function of the payload's **boundary**, not of dst's size or shape: the root-attach
  edge(s) plus the ordered-run **endpoint** splice edges. The detached cluster is
  frozen at commit1 (exclusive), so its boundary — hence the worst-case edge count —
  is fixed the instant commit1 happens. Reserve that worst case before commit1 and
  commit2 never allocates, across any number of retries.
- **Re-record *targets* — dst-derived, per attempt.** Each commit2 try re-descends
  dst and records *which* slot / *which* pred·succ cells into the pre-reserved
  capacity; a conflict resets and records against the new dst. Targets move; the
  count does not.

**Keep the bound dst-independent.** A few edges are conditioned on dst (e.g. the
structural boundary needs the extra SKIP_X dual only when the dst attach parent is
compressed — a dst property that can flip between retries). So reserve the
**dst-independent upper bound** (always budget the potential dual and the full
endpoint splice set); each attempt records `<=` that. Concretely:

```
commit2_bound(payload) = root_attach (<=2: slot + maybe SKIP_X dual)
                       + run_splice_endpoints (<=4: pred·next, succ·prev,
                                                     run_first·prev, run_last·next)
```

a fixed small cap, computed and reserved at the src retire.

### 7.3 What it needs (both small)

1. **`commit2_bound(payload)`** — the worst-case derivation above, used to size the
   reserved txn before commit1.
2. **A reset/re-record txn mode.** Today a `urcu_flip_txn` is *record-once -> freeze
   -> install* (the freeze before install is deliberate: the sorted-address MCAS
   needs the edge set frozen once any proxy is live). A restartable commit2 needs
   *abort-to-PREPARE -> re-record (reusing capacity) -> re-freeze -> re-install*.
   `urcu_flip_txn_abort` already returns the txn to a clean state; the new piece is
   "reset-and-reuse-capacity" rather than "abort-and-destroy," and the
   freeze-before-install invariant still holds **within** each attempt.

### 7.4 Progress level and the optional helping upgrade

As described, commit2 is **obstruction-free across the grace period**: it retries
until it lands, and RCU writers already block on grace periods, so a bounded wait is
in-model. For full **lock-freedom** at the dst boundary, publish the pending move as a
descriptor so a writer colliding there **helps** complete it (idempotently) instead
of merely losing its CAS. Obstruction-free-with-retry is the recommended initial bar;
helping is a later refinement, not a correctness prerequisite.

---

## 8. Summary

| | meaning | status | validated by |
|---|---|---|---|
| **Invariant 1** | every reader-visible edge is a flip-latch descriptor edge | **MET** — campaign closed; re-certified gap → 0, 2026-06-26 (§2) | the edge-audit re-run (functional tests cannot detect a miss); standing static guard still open (decision item 4) |
| **Invariant 2** | freeze-on-free: a freed node fails a concurrent writer's CAS, still resolves for readers; writer re-validates | mechanism **SETTLED** (§4 DECISION FINAL): one-way `deleted` latch — state-word bit (internal) + `next` tombstone (chain leaf); MARK side implemented at every retire; VALIDATE side in progress (§4.2 status) | `FT_DEBUG_TOMBSTONE_AUDIT` builds + the `FT_INV_MW` writer oracle in concurrent mode |

Invariant 1 is the prerequisite the campaign closed. **Invariant 2 is the actual
lock-free enabler** — without it, a fully edge-expressible structure is still
unsafe for multi-writer because an insert can commit into a concurrently-detached
branch.

Both invariants are **vacuous on an exclusive trie** (§5.2): plain stores, no
freeze, synchronous frees. A bulk op pays only at its **shared-side boundary**; its
exclusive/transient side is free at any size. The one genuinely expensive
multi-writer cost is *transitioning a shared subtree to exclusive* — quiescing the
writers inside it (§5.3).

### Decisions / open questions
1. ~~Freeze mechanism: (A) slot tombstone vs (B) node mark~~ — **SETTLED (§4
   DECISION FINAL): the hybrid one-way `deleted` latch** — a state-word bit for
   internal/compressed nodes (§4.B) and the `next`-word tombstone for
   duplicate-chain leaves (§4.A); the seqcount is retired. (The earlier
   "(A), `old == new` flip-proxy" settlement was superseded twice; see the §4
   decision trail.)
2. **Tier-3 bulk ops** (shared subtree → exclusive): subtree-wide freezing vs a
   per-subtree "sealed" epoch the writers check (§5.3). Exclusive/transient sides
   need neither (§5.2).
3. **Compaction**: concurrent (pay the freeze) vs brief-exclusive (§6).
4. **Standing completeness oracle** for Invariant 1 (§2).
5. ~~Confirm the **two-txn protocol** for the inherently two-commit cross-trie ops
   (§7) is acceptable for the MCAS model.~~ — **RESOLVED (§7, 2026-06-25):** commit1
   irrevocable; commit2 a *restartable* boundary MCAS over the exclusive payload —
   **reserve capacity from the payload (invariant), re-record targets from dst (per
   retry)**. Needs `commit2_bound(payload)` + a reset/re-record txn mode. Obstruction-
   free across the GP; descriptor-helping is an optional later lock-freedom upgrade.
6. ~~Read-side consistency under concurrent writers.~~ — **SETTLED (§9,
   2026-07-06): fine-grained** — resolve the proxy at every slot read; no coarse
   read-side re-descent (RCU covers lifetime, immutability covers content).
7. ~~Reclaim ownership / the `nr_live>0` double-free.~~ — **SETTLED (§10,
   2026-07-06): exactly-once retire, the tombstone flip is the single-owner
   token.** A double-free is a bug (a free outside a won tombstone), never
   tolerated by atomic accounting.
8. ~~Who owns the txn `begin`/`end` (read-side bracket + retry identity).~~ —
   **SETTLED (§11, 2026-07-06, refined): the FT API owns it unconditionally**
   (read-side sections nest, so the internal bracket composes with any
   caller-held section). The caller is never *required* to bracket; a caller
   section is needed only to extend returned-reference lifetime (reads —
   unchanged contract) and is FORBIDDEN across a two-commit op (§11).
9. **Two-commit terminal-failure disposition (§7.1/§10):** if commit2 fails
   terminally (`MEMORY_ERR` past what §7.2 pre-reservation excludes), who owns
   the drained, exclusive, RCU-live payload? It holds live keys — a synchronous
   free silently loses data; hand-back to the caller as a detached trie (the
   `cds_ft_detach` shape) preserves it. **Undecided** — must be written down,
   or the §10 audit cannot classify that free path.

---

## 9. Read-side consistency under concurrent writers (addendum 2026-07-06)

Sections 1–8 specify how a writer *commits*. They do not specify what a
concurrent *reader* — or a writer's own descent, which reads exactly like a
reader — may observe, and that omission is the source of the descent/scan
crashes (a garbage-type dereference `ft_type_is_popcount`, a SIGSEGV in
`ft_descent_step`). **Decision (2026-07-06): fine-grained resolution** — resolve
at every slot read; no coarse read-side retry.

A reader is protected by three orthogonal mechanisms, one per hazard:

| hazard | mechanism |
|---|---|
| the node it points at is being **freed** | RCU: the op runs in a read-side section (§11) and concurrent-mode reclaim is deferred via `call_rcu`, so nothing it can reach is freed before its grace period |
| the node's **content** changes under it | immutability: a published node body (bitmap, pointer array, `len`, `key_bytes`, `incoming_byte`, `alloc_index`) is never mutated in place — a change builds a fresh node (§4.3), so the reader sees a stable snapshot |
| a **slot it reads is mid-flip** | resolution: a slot may transiently hold a proxy during a peer's commit window (a type-7 structural proxy, a state-word `FT_STATE_PROXY`, or a list `URCU_MCAS_TAG`); the reader must resolve it to the committed-or-old value before using it |

The first two are global properties requiring nothing at each site. **The third
is a per-site obligation**, and it is the one the code violates: a raw slot read
whose value is then dereferenced or type-classified will, in the flip window,
read the proxy latch — low nibble `0xF` = internal type-7 — and either jump
through a bogus type (`ft_type_is_popcount` / SIGILL) or dereference the latch as
node memory (SIGSEGV).

**The obligation:** every slot read that is subsequently dereferenced,
type-classified, or compared as a node identity resolves first, through the
matching resolver:

| slot class | resolver |
|---|---|
| child slot (descent, scanners, `get_nth`) | `ft_resolve_flip_proxy` |
| compressed `cn->child` | `ft_resolve_flip_proxy` |
| parent / back-pointer (up-walk, reanchor) | `ft_resolve_parent_slot` / `ft_resolve_head_prev` |
| state word (`nr_child`, PSO, tombstone) | `ft_meta_nr_child_load` / `ft_meta_parent_slot_offset_load` / `ft_meta_tombstone` |
| ordered-list link | `urcu_txn_list_resolve` |

**Consequence — no read-side re-descent.** Because RCU covers lifetime and
immutability covers content, a reader never restarts because a writer recompacted
under it: it reads the old node consistently and only resolves the mid-flip slot.
This is why fine-grained resolution suffices and a coarse read-side retry is
unnecessary.

**Completeness is an audit, not a guess.** As with Invariant 1, functional tests
cannot prove the absence of a miss; resolution coverage is enumerated
site-by-site. The write-descent sites (`ft_descent_step`,
`ft_descent_traverse_compressed`, `ft_park_live_parent_edge`) were closed
2026-07-06; the read/scan path is the remaining sweep (Phase B).

## 10. Reclaim — exactly-once retire, via the tombstone token (addendum 2026-07-06)

**Invariant: a node is `call_rcu`'d exactly once.** A `range->nr_live > 0`
failure at free time is a double-free bug, to be root-caused and eliminated —
never masked by making the arena counter tolerate it.

Exactly-once is not separate bookkeeping; it *is* freeze-on-free (§3, §4.B). The
**tombstone flip is the single-owner retire token**:

- A node is retired by exactly the writer whose commit sets that node's tombstone
  **atomically with unlinking its last reader edge** (atomic detach).
- The tombstone is one-way `LIVE→DEAD` and commits through the MCAS, so **only one
  writer can win it**. A concurrent writer that would also retire the node finds
  it already tombstoned — its state-word load-validate fails, its commit aborts,
  and it frees nothing.
- An **aborted** commit frees only its own **fresh, never-published** nodes
  (freeze-before-install): those never reached a reader, so no peer can also own
  them.
- The **retry** rebuilds fresh nodes; it never re-hands a previous attempt's node
  to `call_rcu`.

So the free owner is unambiguous: a published node is freed by the commit that
tombstones+unlinks it (once); an unpublished node by its sole builder on abort
(once). A double-free means a path **frees outside a won tombstone**:

1. an abort/error path frees a node a racing peer actually **published** (it must
   free only its own unpublished fresh nodes);
2. a node is retired by **two commits** because one retire path emits **no
   tombstone edge** (a residual lone-flip / non-atomic retire the freeze-guard
   cannot veto);
3. the **retry** frees a node the previous attempt already gave to `call_rcu`.

One disposition is currently *unspecified* and must be decided before the audit
can classify it: the drained, exclusive payload of a two-commit op whose commit2
fails terminally (decision item 9).

**Audit obligation:** enumerate every `call_rcu`-of-a-node site (`free_cds_ft_node`
/ `free_compressed_node` and the deferred-free lists) and confirm each is reached
only by winning that node's tombstone flip in the same committing txn. Any free
not gated by a won tombstone is the bug. Finite and mechanical, the same shape as
the Invariant-1 and §9 audits.

## 11. Transaction scoping, retry, and progress — FT-owned (addendum 2026-07-06)

The retry loop (`restart_attempt`) and the FIFO livelock-escape need a real
**transaction identity** spanning the loop; the caller's `rcu_read_lock` is only
reclaim protection, not that identity. **Decision (2026-07-06, refined): the FT
API owns the txn `begin`/`end`, unconditionally.** Read-side sections nest, so
the FT-internal bracket composes with any section the caller may hold — no op
needs a carve-out for "the caller might already be a reader."
`urcu_txn_init_flavor` exists precisely for a flavor-agnostic embedder, so
`begin`/`end` open the read-side section in the FT's runtime flavor.

The caller contract that falls out is uniform for op *safety*, with exactly two
caller-side obligations — both about things the internal bracket cannot provide:

- **Single-commit op** (insert, remove, point lookup): one internal
  `begin`/`end` bracket around the whole op. The caller is never *required* to
  hold `rcu_read_lock`; a caller-held outer section is harmless (it nests).
- **Reference-returning reads** (lookup, iteration): the internal bracket ends
  at return, so it cannot keep the *result* alive. A caller that dereferences a
  returned node reference must hold its own enclosing read-side section, exactly
  as the public API documents today — nesting makes the internal bracket
  compatible with that contract, not a substitute for it.
- **Two-commit op** (graft, merge, detach between live tries): two internal
  brackets around the internal drain — `begin`/detach/`end`, `synchronize_rcu`,
  `begin`/attach/`end`. Here the caller **MUST NOT** hold a read-side section:
  the inner `end` closes only the nested section, so a caller-held outer section
  would still be open across the internal `synchronize_rcu` — self-deadlock.
  This is the one asymmetry in the contract, and nesting cannot remove it.

**The retry, concretely:**

- A **persistent** `urcu_mcas_txn` is `urcu_txn_init_flavor`'d **once, before
  `restart_attempt`**, with the trie's escalation **domain** (today the FT passes
  `NULL` — the dormancy) and the FT flavor.
- Each attempt: `urcu_txn_begin` (claims the FIFO turn if the domain escalated) →
  descend → `reserve`/record edges → `commit_flavor`. On `ABORT` the engine ages
  `txn->retry`++ and sets `txn->retrying` (keep the turn); the FT re-descends from
  root.
- After `URCU_TXN_FALLBACK` retries the domain escalates the thread to a **FIFO
  fair-mutex turn**, so a contended writer eventually commits — **no livelock**.
  `urcu_txn_end` closes the section on success or terminal error.
- The FT's per-attempt **bounded** allocation reconciles with the persistent
  handle through the engine's `min_alloc`/`reserve`/grow. Capacity is
  **grow-monotone**: an attempt grows the handle when it needs more edges than
  any prior attempt (a retry re-plans against a reshaped tree, so it may
  legitimately need more), and the handle never shrinks — steady-state retries
  allocate nothing.
- **Reclaim rides the txn:** `defer_on_commit` schedules the retired nodes'
  `call_rcu`; the on-abort action frees the fresh cluster — the single reclaim
  place §10 audits against.

**Migration:** the FT currently drives `commit_flavor` directly with a fresh
per-attempt txn, `NULL` domain, and no `begin`/`end`; the read-side is the
caller's `rcu_read_lock` (e.g. `mw_writer`). Phase A moves to the persistent-handle
+ domain + FT-owned bracket, after which the caller's manual `rcu_read_lock` is
removed. This is the foundation the implementation plan starts from.
