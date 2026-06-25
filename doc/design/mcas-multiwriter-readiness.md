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

**Status: NOT met.** The 2026-06-24 audit classified 274 in-scope store sites and
confirmed **~62 reader-reachable edges still publish via bare stores** (plus 7
compaction sites), concentrated in a few root causes:

- the systemic **list-OFF bypass** (every list-ON txn path has an `else` that does
  the same edge bare);
- the forward-publish primitive `_ft_publish_to_parent(..., rec==NULL)`
  (ft-helpers.h:1935, + the SKIP_X dual at :1900);
- the parent-back-pointer primitive `ft_set_parent` (ft-helpers.h:2042/2061/2078/2112);
- `ft_unchain_node` head-promotion; dual-pointer (cn->child + SKIP_X) torn windows;
- whole-trie root/endpoint swaps (detach, merge root-src, graft_swap);
- the `ft_node_recompact` republish sweep (also reached by ordinary insert/remove
  tier changes, not just `cds_ft_compact`).

Nearly all are mechanically closable by routing through the existing
`urcu_flip_txn` / `ft_ord_cell_flip` / `ft_glue_record_back_edge` machinery, whose
`nr==1` bare-store fast path makes the conversion **zero runtime cost**. The
campaign that closes them is phased in the audit file.

**Completeness oracle.** Because every conversion is behavior-identical under one
writer (the `nr==1` path *is* a bare release store), the functional suite **cannot
detect a missed conversion** — green tells you nothing about coverage. The
completeness check is the **edge audit re-run** (gap count -> 0); a lightweight
standing static guard (grep mutation modules for bare reader-visible stores
against an allowlist of proven build-invisible sites) is recommended so the
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
| **Invariant 1** | every reader-visible edge is a flip-latch descriptor edge | NOT met (~62 gaps) | re-run the edge audit (functional tests cannot detect a miss) |
| **Invariant 2** | freeze-on-free: a freed node fails a concurrent writer's CAS, still resolves for readers; writer re-validates | mechanism leaning to (B) metadata **seqcount** word folded into the flip-latch record (§4 revised); (A) `old==new` proxy kept for narrow-node local use; not implemented | (TBD — needs a multi-writer stress harness) |

Invariant 1 is the prerequisite the campaign closes. **Invariant 2 is the actual
lock-free enabler** — without it, a fully edge-expressible structure is still
unsafe for multi-writer because an insert can commit into a concurrently-detached
branch.

Both invariants are **vacuous on an exclusive trie** (§5.2): plain stores, no
freeze, synchronous frees. A bulk op pays only at its **shared-side boundary**; its
exclusive/transient side is free at any size. The one genuinely expensive
multi-writer cost is *transitioning a shared subtree to exclusive* — quiescing the
writers inside it (§5.3).

### Decisions / open questions
1. ~~Freeze mechanism: (A) slot tombstone vs (B) node mark~~ — **SETTLED (§4):
   (A), encoded as an `old == new` flip-proxy, commit-skips-settle, intentional-only.**
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
