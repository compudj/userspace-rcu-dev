# In-trie subtree move — reader coherence via a per-FT move sequence counter — design note (2026-07-14)

Status: **PROPOSED.** Worked out with Mathieu across one session. Nothing of the
seqcount mechanism is implemented; the one concrete artifact that landed is the
production gate rejecting a same-trie `cds_ft_merge_at` rekey on a
speculative-key trie (see §8). Author of direction: Mathieu Desnoyers.

This note is **orthogonal to** the multi-writer lock-escalation pivot
(`mw-writer-lock-escalation-model.md`): the jump / grace / absence tradeoff below
is a writer↔**reader** re-home concern that exists identically under today's
lock-free design and under the pivot. The pivot only supplies the *writer* half
(a single-domain lock that lets the move commit in one flip); this note supplies
the *reader* half (a sequence counter that makes that flip reader-atomic).

---

## 0. The question

`cds_ft_merge_at` can move a subtree from one key position to another **within a
single trie** (`src_ft == dst_ft`, `src_key` → `dst_key`, the keys disjoint). How
should a concurrent wait-free reader observe that move?

The contract we want, and why it differs from cross-trie:

- **Cross-trie move** (`src_ft != dst_ft`): two namespaces. A move is inherently
  "disappear from src / appear in dst." A reader of either trie already tolerates
  the key not being there — that is what a move across namespaces *means*. The
  existing `detach + grace period + attach` is faithful to it, and a transient
  ABSENT window is acceptable.
- **Same-trie move**: one namespace. The key never leaves the namespace, so a
  reader querying it should resolve it at the **old full-key XOR the new
  full-key at every instant — never neither.** Absence here is not "a move in
  progress"; it is "the key blinked out of existence," which no reader of a
  single trie should witness. Same-trie move is a **rename**, and a rename should
  behave like hard-link-then-unlink (never absent), not unlink-then-relink
  (today's staged behavior, which *is* briefly absent).

Today's same-trie rekey is implemented as `detach (fully commits + drains) → merge
back`, and the header documents the consequence honestly: the moved keys may be
"briefly ABSENT (neither at `src_key` nor yet at `dst_key`)." This note is about
closing that window.

---

## 1. Why the cheap fix (drop the grace period) is unsafe — the "jump"

The moved subtree's root carries a **back-edge triplet**
`{meta->parent, parent_slot_offset, incoming_byte}`. A move *must* rewrite it:
the subtree now hangs under a different parent at a different position, and the
triplet is what changes the *key*.

- A **forward** descent survives a re-home fine (it reads parent→child slots,
  which are proxied and atomic, plus forward node content, which the move leaves
  structurally intact).
- But a reader already **inside** the subtree that uses **back-edges** — an
  ordered iterator's successor/predecessor climb, up-walk key rematerialization,
  or parent-pointer backtrack (the iterator default) — climbs to the subtree
  root, reads the *re-homed* parent pointer, and is carried into the **dst**
  spine. It entered under `src_key` and climbs out under `dst_key`. That is the
  **jump**, and making the edge-flip atomic does not prevent it: the corruption
  is in the subtree's own internal back-references, not in the parent edge.

So a grace period is doing load-bearing work beyond reclamation: it **drains
every reader out of the subtree before re-homing its back-edges.** And that is
exactly what forces the absence window — to pointer-move cheaply you must drain,
and to drain you must first make the subtree unreachable → absent.

The current cross-trie merge proves this split concretely
(`src/fractal-trie/ft-merge.h`): `src`-origin subtrees are pointer-moved, so the
code drains (`update_synchronize_rcu()`, ft-merge.h:1644) and then re-homes their
back-edges by direct write ("the src drain above made them unreachable to
readers... so this is invisible"); `dst`-origin subtrees are re-homed through a
**flip-proxy over a fresh spine copy**, needing no grace ("No dst
synchronize_rcu — the flip subsumed the dst drain," ft-merge.h:1770), because a
proxy boundary routes each reader *wholly* into old or into new physical nodes,
so it never crosses old→new mid-walk.

## 2. Why the clean fix (copy the subtree) is impossible — pinned externals

The `dst`-origin trick is a *copy*. Copying the moved subtree would let a proxy
boundary route readers old-XOR-new with no jump. **But external nodes cannot be
copied** — `cds_ft_alloc_external` was removed; the application embeds
`cds_ft_node` in its own payload and owns the leaf's storage and identity. The
merge's "copy" is only ever the internal **spine**; leaves are always
pointer-moved. So the jump on the moved leaves is unavoidable by construction —
you cannot copy your way out of it.

That leaves exactly two doors, and neither is free:

| approach | reachability during move | cost | verdict |
|---|---|---|---|
| pointer-move + drain (grace) | ABSENT during drain | O(depth) | today's staged behavior; violates never-missing |
| copy the subtree | never absent (double-live) | impossible — pinned externals | ruled out |

## 3. The chosen mechanism — a per-FT move sequence counter (seqcount)

Make the jump **harmless** instead of preventing it. The straggler still climbs
the re-homed back-edge, but it never *returns* that result — it detects it
straddled a move and retries into a clean epoch. This converts the move into a
reader-atomic operation via bounded retry, which is what a single atomic flip
alone cannot give (a wait-free reader straddling the flip has no snapshot).

Mechanism:

- A **per-FT sequence counter**, owned by the **FT root-node lock** (writer side),
  updated **inside the move transaction** so its bump is one of the words that
  flip at the commit's linearization instant τ.
- A reader **samples the counter before fetching the root** and **again after the
  last result-producing dereference**; on a change it **re-descends**. It brackets
  `[root-fetch … result]`.
- **Only key-changing re-homes bump.** Recompaction, split, in-place grow all
  re-home children too, but to the *same* key position (same incoming byte, same
  prefix), so a reader that jumps old-node→fresh-copy reconstructs the identical
  key — the jump is unobservable, no bump needed. An in-trie move is the *only* op
  that re-homes a live node to a *different* key. So the counter fires only on the
  rare move; a move-free workload pays only two loads of a clean, shared
  cacheline — no retries, no writer contention on the line.

Memory-safety holds throughout: the jumped-into dst nodes are live, and the
vacated src spine is not freed until a grace period, so the straggler only ever
reads valid memory — it just discards a stale/torn *key*.

### 3.1 No odd/even seqlock is required

Classic odd/even exists to mask a **non-atomic** write window (the writer mutates
protected data over several stores; the odd state says "torn, don't look, spin").
Here the bump rides the move txn, so it flips at the **same τ** as the structural
selector: before τ the reader sees old-structure + old-seq, after τ new-structure
+ new-seq, with no observable in-between. The reader's question therefore
collapses from "is a write in progress *right now*?" (needs odd/even) to "did a
move *complete* between my two samples?" (a plain monotonic counter answers
completely). `s0 == s1` ⟹ zero moves committed in the window ⟹ the whole
traversal fell on one side of every τ ⟹ consistent. No spin-on-odd; just
re-descend on mismatch, and the mismatch is rare because it requires an actual
straddle, not mere concurrency.

### 3.2 Requirements (load-bearing, not incidental)

1. **The bump MUST be a recorded slot in the commit's write-set, not a store
   sequenced after `ft_flip_txn_commit` returns.** A post-commit bump — even one
   instruction later, even under the root lock — opens a window where structure is
   *new* but seq is still *old*; a reader whose `s0` and `s1` both read the old
   seq while its pointer reads land after the flip sees `s0==s1` and silently
   accepts a jumped view. That is precisely the torn case a plain counter cannot
   detect and the only thing that would drag odd/even back in. Fold it into the
   txn (the existing `ft_flip_txn_record_count_parent` order-statistics fold is
   the template — it already lands a scalar delta atomically with the forward
   publish at τ).
2. **Reader fences are standard:** `s0` acquire-loaded before the root fetch, `s1`
   loaded after the last result-producing dereference, with the structure reads
   ordered before `s1`. The only twist versus a textbook seqcount is that the
   closing sample sits past the up-walk, not at leaf-discovery.
3. **Counter width 64-bit** so wraparound-within-a-traversal is not even
   theoretically reachable.

### 3.3 Consequences of root-lock ownership

- The counter is **single-writer** (every move takes the root lock), so the
  in-txn bump is a contention-free `n→n+1` — no lost update.
- The flip side: **all in-trie moves serialize at the root lock** — two disjoint
  same-trie moves cannot run concurrently. Moves are the rare op, so this is an
  easy trade, but it is a real narrowing versus the fully-disjoint parallelism
  the pivot gives ordinary inserts/removes. It belongs in the cost column.

### 3.4 Pairing with the pivot — single-flip move

The seqcount pairs with a **single-flip** move. If the move stays staged
(`detach … grace … attach`), a straggler that retries just lands back in the
absence window and spins for the *entire* detach→attach duration — you have
turned absence into unbounded reader spin. It only works if the move is one
atomic commit, which needs the pivot's single-domain lock to hold the shared
common-prefix ancestor and fold its recompaction into the same flip — dissolving
the historical obstruction (ft-merge.h:2261: the source unlink's ancestor
recompaction would otherwise yank the destination build's publish slot out from
under it, a stale-slot publish). So: **writer half = the pivot's lock makes the
move one flip; reader half = the seqcount makes that flip reader-atomic.**

---

## 4. Why EVERY reader must bracket — the rekey-then-insert phantom

The seqcount cannot be skipped even for a plain forward exact-key point lookup.
Earlier reasoning that "a forward descent only reads proxied atomic edges, so it
linearizes cleanly" is **wrong**, and the counter-example (Mathieu) shows why —
it fabricates a key that existed at no instant:

- Prefix `A` = src, `B` = dst. Reader R does a point lookup for `AX`. At t0, `AX`
  does **not** exist — the `A`-subtree has no `X` child yet.
- R descends, matches the `A` prefix at the ancestors, enters the subtree, and
  **straggles** at the branch node where `X` would hang.
- t1: **move** `A`→`B`. R is now physically inside the subtree, which lives at `B`.
- t2: **insert** `BX` at the new location (into the moved subtree). The `X` child
  now exists.
- t3: R resumes, sees the freshly-inserted `X` child, descends, finds the leaf,
  and returns it as a match for **`AX`** — a key that existed at **no instant**
  (t0: absent; t1+: subtree is at `B`; t2+: the leaf is `BX`, never `AX`).

This is **missed causality between rekey and insert.** R stitches a fact true only
*before* t1 (the `A` prefix) onto a fact true only *after* t2 (the `X` leaf). The
insert is causally *after* the move, so any coherent observer that sees the
inserted leaf must also see the post-move prefix `B` — R sees `A`. No linearization
point has both, so `AX` is a **phantom**, not a stale-but-linearizable read.
(Without the insert it would be linearizable-before-move — the leaf genuinely was
`AX` at t0; the insert is what converts a benign stale read into a fabricated key.)

Crucially this is a **pure forward point lookup** — no back-edge climb, no
in-leaf-key read, no skip-reanchor; every byte-check R does individually
*succeeds*. The violation is purely **temporal**: the descent's prefix and suffix
matches are sampled on opposite sides of a move+insert and the structure R walked
was never coherent at any single instant. So it kills the forward-fast-path
exemption at a level no edge-atomicity argument can reach — regardless of EAGER
vs speculative, precise vs skip, `NO_FEATURE_FT_SKIP_COMPRESSED` or not.
**Bracket-all, no exemptions.**

The seqcount catches it: R samples `s0` before the root fetch; the **move bumps
the counter at t1** (the insert does not — only moves bump), so R's closing sample
`s1` differs from `s0` → R re-descends into a coherent post-move structure and
correctly reports `AX` absent. The phantom *requires* the move to fall inside R's
descent window (that is what relocated the prefix), which is exactly `[s0, s1]`.
Sampling on moves alone is therefore sufficient.

This also re-explains what the cross-trie grace period was silently buying: not
only safe back-edge re-home, but **draining stragglers so none can be mid-descent
when the vacated region is re-homed and re-populated.** Dropping it for same-trie
(to avoid the absence window) is what lets the straggler stitch, and the seqcount
is the **wait-free substitute for that drain** — it does not drain the straggler,
it makes the straggler *detect* the straddle and retry.

---

## 5. Scope — EAGER tries only (speculative is out of contract)

A speculative trie stores each leaf's full key **in-leaf**
(`speculative_key_offset`), and an exact-key lookup confirms a candidate by
memcmp against that stored key. The library treats that field as **app-owned and
never rewrites it** across a re-keying move (`ft-verify.h:203-215`): a move that
re-parents a leaf under `dst_key` leaves its stored key with the **old prefix**,
so a speculative lookup returns the wrong key. Cross-trie rekeys have a staging
seam for this — the detached source is returned EAGER (`speculative_keys_disabled
= true`, `ft-detach.h:294`) so the app re-stamps each leaf with its destination
key before the data is looked up speculatively. A **same-trie** move has **no such
seam** — one trie carries one speculative mode, and the moved leaves stay
reader-visible throughout — so it cannot be made coherent.

Therefore: **concurrent same-trie move is scoped to EAGER tries.** On EAGER the
key is reconstructed from structure (`ft_rebuild_key_upwalk`) and the in-leaf key
is never read, so there is nothing to re-stamp and the move stays an O(depth)
pointer re-home; the only reader hazard is the back-edge jump, which §3–§4 cover.

## 6. Cost model (EAGER, default rank_stats-off)

- Move: single-flip pointer re-home, **O(depth)** — the subtree relocates by
  pointer; only the spine to `dst_key` and the src unlink / shared-ancestor
  recompaction are touched.
- Writer: all in-trie moves serialize at the root lock (rare op).
- Reader: two counter loads + a compare, on the bracketed paths; a retry only on
  a genuine straddle. Move-free workloads never retry.

## 7. Open decisions

- **Counter granularity.** Per-FT means *any* in-trie move retries *all*
  concurrent readers of that trie, even in an unrelated corner. Fine for move-rare
  workloads; shardable (per-region counter) later if moves ever get hot, with zero
  contract change.
- **Forward-fast-path exemption: REJECTED** by §4's phantom. Recorded here so it is
  not re-proposed.

## 8. First concrete step (LANDED 2026-07-14) — the same-trie speculative gate

Independently of the seqcount work, same-trie `cds_ft_merge_at` on a speculative
trie is **already unsafe today** (it silently mis-stamps moved leaves; only
`ft_verify_speculative_key` under `FEATURE_FT_VERIFY_AT_MUTATION` catches it). A
production gate now rejects it:

- `src/fractal-trie/ft-merge.h`, `ft_merge_at_inner`: early
  `if (src_ft == dst_ft && dst_ft->speculative_key_offset_active) return
  CDS_FT_STATUS_INVALID_ARGUMENT_ERROR;`
- `include/urcu/fractal-trie.h`: `cds_ft_merge_at` contract updated — same-trie
  rekey is INVALID_ARGUMENT when the trie has speculative leaf keys active; move
  within an EAGER trie instead.

- `tests/unit/test_urcu_ft_unit.c`: positive-case test
  `test_merge_rekey_same_trie_speculative_rejected` — creates a
  speculative-key-offset trie (`cds_ft_group_attr_set_speculative_key_offset`,
  which alone sets `speculative_key_offset_active`) and asserts same-trie
  `ft_rekey` → INVALID_ARGUMENT. An *empty* trie suffices because the gate fires
  before any descent, and the same call is a no-op (OK) on a non-speculative trie,
  so the rejection is attributable solely to the gate. `NR_TESTS` bumped
  (275→276, 316→317 fault-inject).

Verified: ft_unit passes (`ok 105 - test_merge_rekey_same_trie_speculative_rejected`,
`1..276`, exit 0); the existing `test_merge_rekey_same_trie` uses a non-speculative
trie (`create_varlen_ft` = NULL attrs) so the gate is inert there and its coverage
is unchanged; ft_inv invariants all pass (its `exit=1` is a pre-existing TAP
plan-count off-by-one in the uncommitted `MW_NR_WRITERS=16` test file, and its
occasional segfault is the known intermittent 16-writer MW residual — ft_inv does
not call `merge_at`).

**Not yet done:** the seqcount mechanism itself; the single-flip same-trie move
(depends on the pivot lock).

---

## Cross-references

- `doc/design/mw-writer-lock-escalation-model.md` — the writer half (single-domain
  lock enabling the one-flip move). This note is the reader half.
- `ft-merge.h:1644,1770,2261` — the grace/proxy split and the same-trie staging
  obstruction.
- `ft-detach.h:294`, `ft-verify.h:203-215` — pinned-external key ownership and the
  speculative re-stamp contract.
