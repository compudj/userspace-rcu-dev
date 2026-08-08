# DLM lock coarseness — the anchor rule

Status: **anchor rule settled; the descent-side CAPTURE and the GRANULARITY KNOB
are IMPLEMENTED and INERT** (no acquire site consumes the anchor yet). Extends
`mw-writer-lock-escalation-model.md` (the DLM whose granularity this
parameterises) and the mode axis in `[[project_ft_two_writer_modes_plan]]`.

> **Landed:** `ft_lock_level` / `ft_lock_level_index` / `FT_LOCK_LEVEL_MAX`
> (`fractal-trie-internal.h`), `struct ft_lock_anchor` + the anchor table in
> `struct ft_descent`, `ft_descent_enter_node`, `ft_descent_anchor`
> (`ft-mutation-helpers.h`), wired into `ft_descent_step` and
> `ft_descent_traverse_compressed`. Verified: the schedule exhaustively over
> depths 0-256; the capture against a brute-force §2 reference over **23.8M
> anchor queries across 400k random span shapes**, compressed-heavy shapes
> included. ft_unit 306/306 and ft_inv 111/111 green (the code is inert, so this
> is a no-regression check, not coverage of the rule).
>
> **Cost:** `struct ft_lock_anchor` is 24 B, the table 240 B, so `struct
> ft_descent` grows ~248 B. It sits on the MUTATION descent only (lookups use a
> different descent), and needs **no init sweep** — see §4.

Memory: `[[project_ft_three_excl_modes_and_dlm_coarseness]]` (the agreed design
this specifies), `[[project_ft_lockset_completeness_audit]]`,
`[[project_ft_point_op_concurrency_contract]]`.

**Decisions (Mathieu, 2026-08-06):**
1. Depth is tracked on the descent; the descent's depth selects the nearest
   ancestor lock. No up-walk (§5.3).
2. The spacing is a **runtime knob on the group attr**, not a fixed schedule
   and not a build flag (§6).

---

## 0. What the knob is

Today a `CDS_FT_WRITER_LOCK_FINE` trie locks **the nodes it mutates**: a point
op acquires `{C, P, GP}` and a lock-set op up to four members. The knob makes an
op instead lock an **anchor** — an ancestor at a designated *lock level* — so
that several distinct nodes map to one lock word.

Lock levels are spaced on an exponential schedule (`0, 1, 2, 4, 8, …`): dense
near the root, sparse deeper. A lock near the root covers a subtree that may be
half the trie, so coarsening there would block enormous amounts of unrelated
work; deep down a subtree is tiny, so one lock spanning eight sparse levels
blocks almost nothing. Subtree size falls off roughly as `256^-d`, so spacing
may RISE exponentially while WORK COVERED PER LOCK stays roughly constant.

The measurable win is **lock-set cardinality collapsing with depth**: `C`, `P`
and `GP` at byte-depths 20/19/18 all anchor at byte 16, so a three-member
acquire becomes a one-member acquire and the MCAS commit narrows. Near the root
nothing collapses — the anchor *is* the node — which is where precision is
worth paying for.

At maximum coarseness (only the root is a lock level) the DLM degenerates
**exactly** into the internal per-trie lock, so `CDS_FT_WRITER_LOCK_COARSE` and
`CDS_FT_WRITER_LOCK_FINE` (`include/urcu/fractal-trie.h:2866`) become two points
on one granularity axis rather than two strategies.

---

## 1. The agreement invariant, and why partial coarsening is unsafe

> For any node `X` and any two ops that will mutate `X`, both must acquire the
> **same lock word**.

So there must exist a function `anchor: node → node` that is **total** and that
every acquire route computes **identically**.

This rules out the tempting hybrid. "Coarsen when the whole lock-set came from
one descent, otherwise lock per-node" is **unsafe, not conservative**: op A
anchors `X` to an ancestor, op B locks `X` directly, and they exclude nothing.
Making an anchoring op *also* lock `X` restores safety by *adding* locks —
strictly more cost than today, with none of the collapse. `anchor()` is global
and total or the knob cannot ship.

The surface that must agree is wide:

| acquire primitive | sites |
|---|---|
| `ft_meta_lock_acquire` | 30 |
| `ft_dlm_lock` (`ft-mutation-helpers.h:1012`) | 5 |
| `ft_dlm_acquire_set` (`ft-mutation-helpers.h:1083`) | 2 |

---

## 2. The rule

Let `depth(X)` be `X`'s **byte-depth**: the length of the key prefix from the
root to `X`. `struct ft_descent` already maintains it
(`ft-mutation-helpers.h:37`), advancing by one per slot hop
(`ft-mutation-helpers.h:226`) and by `cn->len` across a compressed node
(`ft-mutation-helpers.h:189`).

Let `L(d)` be the deepest lock level at or above `d`. For the default doubling
schedule:

```c
/* Clear all but the top set bit: 0,1,2,4,8,...  No table. */
static inline unsigned int ft_lock_level(unsigned int d)
{
        return d ? 1u << (31 - __builtin_clz(d)) : 0;
}
```

> **`anchor(X)` = the node starting at the first node boundary at or after byte
> `L(depth(X))`; if no boundary falls in `[L(depth(X)), depth(X)]`, the node
> whose half-open byte span `[start, start+len)` contains byte `L(depth(X))`.**

Half-open makes a boundary unambiguous: where parent `P` spans `[a,b)` and child
`C` spans `[b,c)`, byte `b` is covered by `C`, never by both.

The round-up to the next boundary is what keeps compressed nodes from swallowing
the schedule — see §2.1. In a bushy trie (one byte per node) the first boundary
at or after `L(d)` *is* `L(d)`, so the round-up is the identity and the rule
reads as the plain covering-node form.

Units are **key bytes, not node hops** — a compressed node spanning 8 bytes
advances depth by 8 while costing one lock. Byte-depth bounds the KEY SPACE a
lock covers, which is what the `256^-d` rationale is about; node-depth would
bound only how many locks an op takes.

`FT_MAX_KEY_LEN` is 256 (`fractal-trie-internal.h:293`), so the doubling
schedule has 9 levels: `0,1,2,4,…,128`.

### 2.1 Compressed nodes crossing lock levels

`cn->len` is a `uint8_t`, **1-255** (`fractal-trie-internal.h:1193`), and
`FT_SKIP_LEN_MAX` is 255 (8 bits). **One compressed node can span the entire key
space.**

Three sub-cases need no special handling:

* **Several lock levels inside one span** — they collapse to that node. One
  lock covering many byte-levels is what a compressed node *is*.
* **The descent stops inside a span** (a split point) — `L()` is evaluated at the
  actual stop depth, so the anchor may sit above the compressed node.
* **Span identity churn** (a split or fuse changing which node covers byte `L`) —
  the restructurer holds both pieces, and an op that planned against the
  pre-change node hits `TOMBSTONE` and replans (§3).

The case that does need handling is the **`256^-d` premise itself**. That
estimate assumes every byte level branches; a compressed node is by definition a
run of levels where branching did **not** happen. Byte-depth therefore
over-counts subtree reduction exactly where compression exists.

Worked case: a 200-byte common prefix (URLs, paths, timestamps, sequential IDs)
yields a root compressed node spanning `[0,200)`. Ops in the bushy subtree below
sit at depths 200-255, so `L(d) = 128`, which lands *inside* that span. Under a
plain covering-node rule every op anchors on the root compressed node — the
subtree at byte 128 is not tiny, it is the whole trie. That is
`CDS_FT_WRITER_LOCK_COARSE` by accident, on a common workload shape.

**The round-up in §2 closes it.** X at depth 250 with `L=128` inside `[0,200)`
takes the first boundary at or after 128 — byte 200, i.e. `cn->child` — so ops
below the prefix stop serialising against ops inside it. The clamp keeps the
anchor an ancestor-or-self: X at depth 150 has no boundary in `[128,150]`, so it
falls back to `cn`, which is the node it sits inside anyway.

Collapse survives: C/P/GP at 250/249/248 all have `L=128` and all round to the
same boundary, so the three-member acquire still becomes one. Stability
survives: splitting `[0,200)` into `[0,50)+[50,200)` leaves the span containing
128 still ending at 200 — same boundary, same anchor — because the rule stays in
byte space and inherits §3.

---

## 3. Why the rule lives in key-byte space

Split, fuse and recompaction change **which node covers a byte**; they never
move **a key's byte position**. Defining the anchor on byte positions therefore
survives every restructuring the trie does to itself.

The residual — two ops disagreeing because the covering node changed between
them — needs no new machinery, because **every change to which-node-covers-byte-L
is itself a locked mutation of those very nodes.** A split or fuse acquires both
pieces, so an op holding the pre-change node conflicts with the restructurer on
the existing protocol. Node-identity churn is already serialised; the anchor
rule inherits that for free.

Absolute depth changes only when a subtree is **re-homed**. Cross-trie
graft / graft_swap / merge_at already require an **EXCLUSIVE source** under
`LOCK_FINE`, so no concurrent writer is inside the moved subtree and no peer can
observe the depth change mid-op. This closes the re-homing hazard that
`[[project_ft_three_excl_modes_and_dlm_coarseness]]` flagged as the correctness
constraint.

---

## 4. The anchor is captured on the way DOWN

`struct ft_descent` (`ft-mutation-helpers.h:31-57`) is a **4-deep sliding
window** — `nf` / `pnf` / `ppnf` / `pppnf`. It carries `depth`, but only three
ancestors. The anchor for byte-depth 20 sits at byte 16 (up to 4 hops — just
fits); for byte-depth 40 it sits at byte 32 (up to 8 hops — does not). **Depth
is free; the ancestor at that depth is not.**

`L(d)` is monotone non-decreasing, so a descent crosses each lock level exactly
once. Capture the anchor as the descent passes it:

* `struct ft_descent` gains a small ring of the last K crossed lock levels, each
  entry `{flag, slot, start, len}` — a **span, not a point**, since the lookup
  asks which entry's span contains `L(d)` (§2.1). A skip-compressed flag must
  resolve through `ft_skip_to_compressed` to reach the lockable metadata word,
  tested before external per the usual dispatch order
  (`[[feedback_skip_before_external_tag_order]]`).
* `anchor(X)` for any `X` in the descent window is an O(1) index into the ring.
* **K ≥ 5.** The bound comes from the *window depth* (4 slots, so at most 4
  distinct anchors) and not from the schedule, so K is independent of the knob.
* Updated inside `ft_descent_step` / `ft_descent_traverse_compressed`, so it
  inherits the existing flip-proxy resolution and `skip_conflict` discipline
  rather than adding a second coherence story.

A descent may stop *inside* a compressed node (a split point). The anchor is
then evaluated at the actual stop depth, which may leave the anchor above that
node — handled by evaluating `L()` at the stop depth rather than at the node
boundary.

**No init sweep.** Only levels the descent has CROSSED are readable, and a query
at depth `d` touches `ft_lock_level_index(d)`, which is at or below the deepest
crossed slot — so every reachable read is written first and `ft_descent_init`
zeroes only the two bitmasks. `anchor_crossed` records the written set so a
debug build asserts that rather than trusting it.

**The cursor node is the table's blind spot, in two ways.** It spans
`[d->depth, …)` and has not been entered, yet it is a legitimate boundary:

1. the level can fall exactly on it (`L(depth) == d->depth`), and
2. it is the boundary a still-**pending** level is waiting for — a level goes
   pending only from the last entered node, and that node ends at `d->depth`.

Case 2 was a live defect caught by the §2 reference check, not by inspection:
resolving `bound` only on the *next* `ft_descent_enter_node` left the last node's
interior levels pointing at their coverer instead of the cursor. Both cases are
handled in `ft_descent_anchor`, and `depth <= d->depth` is asserted — a member
BELOW the cursor is resolved by its caller from the two candidate boundaries it
already holds (§7.1), never here.

---

## 5. Route inventory

### 5.1 Routes with a real descent

insert, graft, merge, detach (`struct ft_descent`), and the compaction walk
`ft_compact_descend` (`ft-compact.h:368`), which is hand-rolled but already
tracks `size_t depth` and the holder at each step. These need the ring and
nothing else.

### 5.2 Routes that have a descent but do not carry it — plumbing

| route | derives from | fix |
|---|---|---|
| `ft_node_recompact` `{C,P,GP}` (`ft-mutation-node.h:1255-1332`) | `ft_resolve_parent_slot` / `ft_parent_hint` | widen the signature to carry the caller's anchor state |
| `ft_compact_relocate_at` (`ft-compact.h:51`) | `meta->parent_word` | take the depth its caller already tracks |
| `ft_chain_head_holder` under insert (`ft-insert.h:4068`) | `prev` walk | the head's holder *is* the descent's parent |

`ft-mutation-node.h` is the plumbing frontier: **10 acquire sites, zero
`ft_descent_init`.** Every acquire there inherits depth from a caller that has
it.

### 5.3 The one genuinely descent-less route: node-handle remove

`_cds_ft_remove_locked` (`ft-remove.h:3289`) takes a `struct cds_ft_node *` and
derives its holder from the back-pointer:

```c
3351:  holder_flag = ft_node_holder(ft, node);   /* node->prev */
```

There is no key-guided descent on the fast path, so there is no depth to track.
It descends only in the unlikely stale-holder recovery
(`ft-remove.h:3378-3399`), and that descent exists to repair a tombstoned
holder, not to locate anything. `ft-remove.h` carries 12 acquire sites behind
this path, including the interior-duplicate unchain
(`ft-remove.h:3453` → `ft_chain_head_holder`).

**Resolution: remove re-descends when the group's spacing is coarser than
per-node.** At per-node granularity `anchor(X) = X`, no depth is required, and
remove keeps its handle fast path untouched. The key is already in scope
(`iter_key` / `key_len`) and the recovery arm proves the descent is available.

**An up-walk was considered and rejected.** It cannot stop early: selecting the
anchor needs `L(depth(X))`, and `depth(X)` is *absolute*, so a climb from `X`
does not know it until it reaches the root and counts. The walk is therefore
always full-height — strictly longer than the descent it would replace — and it
ends holding a count but not the anchor node, which it has already walked past.
Cost compounds it: each hop is a cold metadata cache line, `ft_resolve_parent_slot`
does a same-mcas read plus a stability re-read per hop, and its unbounded
`for(;;)` has a recorded 1-core MW livelock arm (`ft-mutation-node.h:1902`).

---

## 6. The knob — LANDED

`enum cds_ft_lock_spacing` + `cds_ft_group_attr_set_lock_spacing()`, resolved at
group create and copied to each trie and then to each descent. The enum is the
granularity axis:

| setting | anchor | note |
|---|---|---|
| `CDS_FT_LOCK_SPACING_PER_NODE` (default) | `X` itself | today's behaviour |
| `CDS_FT_LOCK_SPACING_EXPONENTIAL` | §2 rule over levels `0,1,2,4,8,…` | |
| `CDS_FT_LOCK_SPACING_ROOT_ONLY` | the root node | meets `CDS_FT_WRITER_LOCK_COARSE` from the other side |

Per-node is a genuine zero-cost path, not a schedule with spacing 1:
`ft_descent_enter_node` returns immediately, so it builds no table and reads
none. Root-only fills slot 0 and returns on every later node. This matters
because the setting also decides whether remove pays a descent (§5.3).

The default stays PER_NODE — current semantics — until the bench picks
otherwise; the win is workload-shaped (trie depth, key distribution, writer
disjointness). Follow `[[project_bench_methodology_checklist]]`.

**Coverage.** With PER_NODE as the default the table would be dead in every test
config, so the resolution of the DEFAULT (never an explicit
`cds_ft_group_attr_set_lock_spacing`, which always wins) honours a
`CDS_FT_LOCK_SPACING` env override — `per-node` / `exponential` / `root-only` —
letting a whole suite sweep the axis. `FEATURE_FT_ANCHOR_VALIDATE` additionally
exercises the anchor LOOKUP from the descent at the depths an acquire site
queries, since until an acquire site consumes the table
`ft_descent_enter_node` is the only half the suites otherwise reach.
`test_lifecycle_lock_spacing` covers the API contract and all three settings
without the env var.

---

## 7. Lock-set shape (audited 2026-08-06)

A lock-set is **not** a path segment within the ancestor window. Two findings,
both of which the rule absorbs, but which change the acquire code.

### 7.1 Sets extend BELOW the pivot

The chain-compress set (`ft-remove.h:685-722`) is
`{B, parent_CN, child_CN, pp}`: `child_CN` comes from `surviving_child` — one
node **below** B — while `pp` is parent_CN's parent, two **above**. The ring
holds ancestors only, so a below-pivot member is not in it.

Closable without extending the ring: a member one hop below the pivot has only
two candidate boundaries in `[L(depth(member)), depth(member)]` — the pivot's
span and the member's own — and both are in hand at the acquire site.
**`ft_descent_anchor_child()` implements exactly that**, and
`ft_descent_anchor_at_level()` is the shared core: an immediate child takes ITS
level but clamps at its OWN depth, which lies past the cursor, so the level and
the clamp had to stop being the same argument. A member deeper than one hop has
intermediate boundaries the table never saw — such a caller must extend the
descent, not reach further with this.

Note this set is derived entirely from back-pointers
(`rcu_dereference(iter_meta->parent_word)`, `ft_resolve_parent_slot`), consistent
with §5.3: it sits under the descent-less node-handle remove.

### 7.2 Sets FAN OUT — but only two of them lock the fan

`ft_rekey_cow_stop` (`ft-mutation-node.h:2374`, sweep at 2626-2673) and
`ft_glue_acquire_reparent_marks` (`ft-mutation-helpers.h:5710`) call
`ft_meta_lock_acquire` **per child, over `FT_ENTRY_PER_NODE`** — up to **256
acquires in one op** — and pass `child_marked=true`.

**The GENERAL recompact reparent sweep does not.** It takes `{C,P,(GP)}` and
"never C's children" (`ft-mutation-helpers.h:5294`), recording each child as an
MW *validated* edge (`ft_flip_txn_record_tag_mw`, `child_marked=false`) instead.

**★ Locking the sweep's children was implemented in full and REVERTED**
(`ft-mutation-helpers.h:5305-5310`): *"a contended child fails the acquire, and
escalation cannot rescue it (an escalated acquirer holds its FIFO turn while
spinning for a holder funnelled behind that turn). Validating aborts the COMMIT
instead, which is precisely what the escalation lane arbitrates."* Anchoring
must not silently re-run that experiment.

For the two sweeps that *do* lock the fan, all children sit at depth `d+1`, so
unless `d+1` is a power of two they share one anchor at or above the parent —
which the op frequently **already holds** as C/P/GP, making the collapse 256→0
rather than 256→1. That cuts directly at the failure mode above (fewer acquires,
fewer chances a contended child kills the op).

**It may also cut the other way, and the bench must settle it:** a coarse anchor
high in the tree is *more* likely to be contended per acquire even though there
are far fewer acquires. Claiming the win before measuring would be repeating the
reverted experiment's mistake.

### 7.3 What dedupe actually needs

**The fan-out needs a hoist, not a set.** Every child of one node shares depth
`d+1`, so the case splits once, before the loop:

* `L(d+1) <= d` — all children share ONE anchor on the path above; compute it
  once, acquire once (or find it already held), and drop the per-child acquire.
* `L(d+1) == d+1` (`d+1` a power of two) — each child is its own anchor, all
  distinct, so the loop stands as today and no two children can collide.

Either way **no dedupe structure is required for the fan-out.** It is a branch.

**The path members do need dedupe, and `n <= 5`.** C, P, GP, pp and `child_CN`
sit at *different* depths, so they can collide on one anchor. A linear scan over
the existing `struct ft_dlm_member set[]` is ≤10 comparisons. No hash set, no new
allocation.

**★ Dedupe the LOCKS; keep ALL the GUARDS.** `ft_dlm_guard_parent` is a
per-MEMBER read-set validation, so if X and Y collapse onto anchor A, both
guards still ride the commit and only the `{clean -> LOCK}` record merges.
Reservation becomes `nr_guards + nr_distinct_anchors`; since that is
`<= 2 * nr_present`, `ft_dlm_acquire_set`'s existing bound
(`ft-mutation-helpers.h:1093`) stays **safe, merely loose** — the transition
carries no reservation risk.

**Terminals are per-ANCHOR.** A member that deduped away was never locked and has
no LOCK bit to release, so `marks[]` / `snaps[]` hold anchors. The mapping
therefore belongs at the **caller** — build anchors, dedupe, hand
`ft_dlm_acquire_set` the distinct set; `struct ft_dlm_member` keeps its shape.

**Self-collision generalises `parent_held`.** `ft_parent_hint.parent_held`
(`ft-mutation-node.h:1277`) already exists because *"a second `ft_dlm_lock` would
abort -EAGAIN"*. Under anchoring the dedupe must also test the op's
**already-held** anchors, so that bool becomes a small array.

**★ `child_marked` must stay FALSE for an anchor-protected child.** It selects
SW-park (`record_tag`, which RELEASES the child's own mark) versus MW-validated
(`record_tag_mw`), and the discriminator is *who holds `@meta`'s LOCK*. An
anchor-protected child holds no mark of its own, and an SW park on its state word
would "erase whatever a peer put there: a committed `nr_child` edge from an
insert BELOW the child (lost count) or a fresh lock acquire (stolen lock)"
(`ft-mutation-helpers.h:5300`). The anchor does **not** exclude an insert below
the child — that op anchors deeper. So the collapse merges LOCKS only; it never
converts a validated edge into a park.

## 7bis. Open

1. **Rank-stats coercion.** An order-statistics trie is forced to COARSE today.
   Under a granularity axis it should coerce to the root-only *setting* rather
   than to a separate strategy.
2. **Bench.** Anchor collapse should show as a narrower MCAS commit and fewer
   `-EAGAIN` regrow rounds at depth; measure against the per-node setting as the
   control. Lead with the §7.2 recompact fan-out, not the point-op case.
3. **Dedupe structure.** §7.2 needs an anchor-keyed dedupe set on the acquire
   path. Sizing it is the first implementation question.

---

## 9. Acquire-site inventory

### The choke point

Every acquire resolves its member through ONE of two entry points, which anchor
and dedupe together:

| | |
|---|---|
| `ft_acquire_member(ft, ctx, nf, node, depth, &held)` | immediate: marks the anchor now |
| `ft_dlm_acquire_set(ft, ctx, set, nr)` | transacted: one all-or-none MCAS over a whole lock-set |

`struct ft_lock_ctx` carries what belongs to the OP rather than to the member:
the descent that supplies anchors, and the words already held. Its held set
spans the txn registry **and** the marks an op keeps outside it
(`ft_detach_node`'s orphan chain reaches `FT_MAX_DEPTH`, past
`FT_FLIP_TXN_MAX_LOCKS`). A NULL context means "no descent, nothing held" and
is legal only under per-node granularity, which `ft_anchor_meta` asserts.

`struct ft_held_anchor` is the result: the word the acquire CAS'd (release,
register) and the node's own clean word (retire), plus `shared` — the op
already held the word, so this member owes neither terminal.

★ **The anchor comes from the caller's METADATA, never from a flag rebuilt out
of a node pointer and a type index.** That reconstruction takes the
internal-node path in `ft_flag_to_metadata`, so for a compressed node reached
through a skip pointer it names a DIFFERENT node — the acquire then fences one
word while the retire tombstones another. `ft_anchor_meta` resolves a flag only
when the anchor is a genuine ancestor, i.e. a flag the descent stored.

★ **`ft_descent_depth_of`** answers what a depth SCALAR cannot: a site locking
`{C, P, GP}` reaches P and GP through back-pointers, which yield nodes with no
depth at all. It returns false where the descent does not describe the node, so
the caller RE-PLANS rather than anchoring one node by another's depth.

★ **The window is four NODES, not four SET MEMBERS**, and a `{C, P, GP}` set
whose C already sits at the third slot runs off it — measured, not feared:
`test_rekey_coherence_lookup` reaches `ft_node_recompact` with C at byte-depth
6, P at 1 (a five-byte compressed run) and GP nowhere in the window.
`ft_lock_ctx_depth_of_parent` dates such a member from the node BELOW it: a
node's span is a property of the node itself, so the parent of a node at a
KNOWN depth sits at that depth minus the parent's span (`ft_node_span`, shared
with `ft_child_depth_of` so the two directions cannot drift). This is NOT the
up-walk §5.3 forbids — a climb starts *undated*, and byte-depth is absolute;
this steps one hop up from a member already dated.

★ **Spacing is a FINE-mode property and is made inert elsewhere**
(`cds_ft_create` resolves it beside `lock_fine`). COARSE derives no lock-set:
its remaining acquires are the F2 body-copy fences, taken one at a time with no
set to dedupe against, so coarsening them only collapses an op's OWN marks onto
one word — `ft_chain_compress_fused` takes three in a row and the second
refuses against the first. The FT-wide writer lock already excludes every peer
the anchor would.

### Conversion status

**Every acquire site is converted**, and that is a BUILD property rather than a
table: past the choke points the raw `ft_meta_lock_acquire` / `ft_dlm_lock` are
`#define`d to an undeclared identifier under `FEATURE_FT_ANCHOR_VALIDATE`, so a
new raw acquire cannot compile. Verified against a red control (a raw acquire
reintroduced into `ft_split_compressed_graft_build` fails the build, naming the
choke point it must use).

Two supporting pieces landed with it:

* **`ft_walk_extend` — a writer walk EXTENDS the descent.** `ft_detach_node`'s
  orphan walks move DOWN a chain past the cursor, so their depths are neither
  `detach_depth` nor anything the window carries. Each step's span is known (1
  internal, `cn->len` compressed), so feeding it through
  `ft_descent_enter_node` puts the walked region in the SAME anchor table and
  plain `ft_descent_anchor` answers for every node on it. One depth mechanism
  and one table per op, which is what agreement wants.
* **`ft_anchor_descend`** (§5.3) is now used by remove, remove-all AND replace —
  every handle-derived entry point pays a descent exactly when the spacing is
  coarser than per-node.

`ft_compact_relocate_at` still passes a NULL context; it needs the depth
`ft_compact_descend` already tracks.

### The self-collision class, and the detector that names it

A refused acquire says the word carries `FT_STATE_LOCK`, never by WHOM — and
under a coarse spacing the whom is usually the refusing op itself. Fatal rather
than merely slow, because `_cds_ft_remove_all_locked` has **no retry loop**: its
-EAGAIN surfaces as a hard `MEMORY_ERROR`.
★ **A refusal is only a fallback where a RETRY exists.** Same reason the orphan
walk had to extend the descent instead of refusing an undatable member.

`FEATURE_FT_HOLD_TRACE` is the oracle for the class: a per-thread ledger of the
words held and the SITE that took each, kept at the lock PRIMITIVES rather than
at the choke point so it sees every hold whatever registry the op filed it in —
that mismatch being exactly what is measured. A refusal whose word the ledger
names is reported with both sites and aborted. `ft_flip_txn_commit` drops its
registry from the ledger on OK, because a committed lock stops being held
through its recorded terminal with no release to observe; without that the
ledger saturates and every report is a stale entry.

Two instances found and closed with it, neither by inspection:

* `ft_detach_node` kept the **trailing skip-target's** plan-lock in a variable
  of its own, outside `@orphan_held` and so outside the held set entirely. It is
  now an entry of that array like every other orphan. ★ 08-07b's *"the held set
  needs no new plumbing: the txn's `locks[]` registry"* was wrong in the useful
  direction — the word was in NEITHER registry, so filing is what was missing,
  not a second view.
* The **spacing was not inert under COARSE** (above).

### CLOSED: an anchor that is the node the op RETIRES

The probe armed in `ft_flip_txn_record_retire_anchored` fired: `ft_detach_node`
marks an orphan whose anchor climbs to the node `ft_node_recompact` then
replaces, so the recompact's own member comes back `shared` on the node it is
about to retire. Two things were missing.

**The snapshot.** A deduped member carries none — the acquire never ran — and
re-reading the word samples the op's own LOCK. Two consumers needed it and both
were reading zero: the fenced retire's expected old, and the PLAN
RE-VALIDATION (`ft_chain_compress_fused` reads its boundary's `nr_child` out of
`node_snap`, so a deduped member failed the check and the merge re-planned into
the same shape). The txn's `locks[]` registry therefore stores the snapshot
beside the word, `ft_held_set_snap` returns the FIRST acquire's, and both choke
points fill `node_snap` from it wherever the word IS the member's own node.

★ **ONE WORD TAKES ONE TERMINAL, AND A RETIRE OUTRANKS A RELEASE.** One member
wants the anchor to survive, the other kills it, and a node's fate belongs to
the op's PLAN, not to the order its members were acquired. Recording both
poisons the txn either way round — the two carry the same expected old, so
whichever lands second mismatches the first's pending new and `record_chain`
sets `t->poisoned`, permanently. So the release YIELDS to a pending TOMBSTONE,
and a retire that follows a release chains onto its clean pending value instead
of re-asserting the mark.

### The glue's anchor source, and how a livelock announces itself

A glue's own acquires fire from commit helpers that never see a descent, so
`ft_glue::lock_d` is how they get one — and only the graft's store-prepare set
it. With it NULL, a coarse acquire has no depth, MISSES, aborts the commit, and
the caller retries into the identical shape. That is a LIVELOCK, and the suite
does not report it: it announced itself as **25 GB of one repeated diagnostic**.
Hence the report cap on `FEATURE_FT_HOLD_TRACE` — an unbounded report of a
refusal a retry loop re-derives is not a report.

`lock_d` is now set where the descent is CREATED (`ft_graft_build`), because
`ft_merge_graft_subpos_inplace` reaches the commit without passing through the
consumer that used to set it.

### The self-refusal class, closed on both words

Every coarse-arm defect so far has one shape: **the op refuses its own mark**.
That is not contention and no retry can clear it — the op re-descends and
re-derives a byte-identical plan — so it does not fail, it SPINS. A suite does
not report a spin; it reports a timeout, hours later, on a test that names
nothing.

Three sources of marks were invisible to the choke point and are now in the held
set:

* **`ft_rekey_cow_stop`'s `@marks`** — @stop's fence plus one per child, none of
  which reaches a txn registry until the caller's sweep. Every child of @stop
  anchors on the path above, frequently @stop itself.
* **A glue's NAMED FIELDS** — publish parent, the compressed node its build
  splits, the overlap fences, the splice holders. `ft_glue_op_holds` already
  enumerated them for sites that ask directly; `ft_held_set` could not see them.
  Written once now, in `ft_glue_held_snap`, which also yields each field's
  SNAPSHOT.
* **A glue with no anchor source at all** (`lock_d` NULL) — three of them.

★ **And the dedupe has to cover BOTH WORDS.** Coarsening splits a node's lock
word from its own word, so an acquire deduped the ANCHOR and then sampled the
member's OWN word with a raw read that refuses anything dirty. Right for a
peer's mark, wrong for the op's: an earlier member routinely anchored ON this
node. Measured: **200 million descents in 60 s** against under 16 million for
the whole test at per-node granularity. `ft_member_node_snap` asks the held set
first; `@node_held` then carries the two consequences — the acquire-time node
guard is dropped (the mark IS that exclusion, already in force, and validating a
clean value against the op's own LOCK aborts every attempt), and the retire
takes the fused shape.

### The residual was TWO defects, and the control could not fail

Two tests did not finish, each fine in the OTHER coarse arm — read at the time
as one spacing-dependent mechanism. It was **two unrelated defects in two
different ops**; the symmetry was a coincidence. Both tests are
SINGLE-THREADED, which is the fact that mattered: with no peer, every
"contention" retry is self-inflicted.

★ **The knob was compiled in only under the probe macros**, so the obvious
control — build it plain, is it still slow? — had never run a coarse arm at
all. It answered with the per-node timing, and the number agreeing with the
baseline is what made it convincing. `FEATURE_FT_LOCK_SPACING_ENV` now carries
the knob and each probe implies it, so a build exists that runs the coarse arm
with no instrumentation. With that control both hangs reproduce uninstrumented:
the probes were innocent.

Consequently the earlier "what it is NOT" list is void. Two of its entries were
wrong outright: `test_density_stress` IS an `-EAGAIN` storm (28,799,994 retries
in ONE remove op — the tagging sweep had missed the firing site, whose `return
-EAGAIN;` carries a trailing comment), and the cost is not per-descent — the
record set averages 3.5 entries, so it is the sheer NUMBER of tiny
transactions. A retry loop that re-runs a whole op profiles as FLAT, because it
re-does everything.

### CLOSED: a split-retire that named the anchor instead of the node

`ft_split_compressed_graft_build` stored the anchor its acquire locked into
`glue->split_cn_holder`, and the commit recorded the fenced
`{LOCK|s -> TOMBSTONE|s}` against that field. Per-node the anchor IS the
compressed node. Coarsening splits them, and the commit **tombstoned the
anchor** — measured as the dst root, `nr_child` 2 at depth 0, the node the
graft publishes into. It never landed only because the publish parent's release
was already recorded on that word, so the two contradicted and the engine
poisoned the txn: **the livelock was the structure defending itself.** Where
the anchor is an ancestor no other record touches, the tombstone lands.

The glue now carries `@split_cn_node` / `@split_cn_node_snap` beside the
holder, and `ft_glue_tombstone_free_list`'s skip matches the node's own word
(matching the holder, it did not fire under coarsening and the node took a
second plain tombstone).

### CLOSED: a §4.B guard on a word the op holds

`ft_flip_txn_lock_or_guard_parent`'s shared arm guarded the member it had just
deduped. The guard's expected old is clean-LIVE — it masks out `FT_STATE_LOCK`,
**including the op's own** — so on a held word it names a value that word has
not carried since the mark landed, and the member that took it records its
terminal later in the same commit. That is exactly the "guard THEN release =
POISON, permanently" order §7.3's ordering rule calls unreachable; coarsening
made a site reach it.

★ **A fix you cannot show FIRING is not demonstrated.** An intermediate attempt
made `ft_flip_txn_record_anchor_release` yield when the pending word showed
LOCK already clear. The suite went green — and the rule was unsound: a guard's
masked value and a terminal's cleared value are indistinguishable in the
pending word, so it was suppressing the only release. Probing whether it ever
fired returned 0/0 on both arms. It was dropped; the guard fix carries the
result alone.

### ★ OPEN: a retire snapshot a LATER acquire invalidates

`exponential` / `test_density_stress` remains. `@node_snap` is sampled when a
member is acquired; a **later** acquire in the same op then takes that node's
OWN word (the detach's orphan chain walks up, so a coarsened member becomes a
direct anchor a step later). `ft_flip_txn_record_retire_anchored` then records
`{clean -> clean|TOMBSTONE}` against a word carrying the op's own `LOCK`:

```
STALESNAP node=0x..598 snap=4 cur=80004 lock=0x..658 lock==node=0 node_held=0
```

`ft_member_node_snap` answers "did an EARLIER member anchor here?" — it cannot
see a member that has not run yet. Per-node never exposes it: `lock == node`,
so the fused terminal is taken and no separate snapshot exists to go stale.

The expected old must NOT be re-read raw (a peer change between acquire and
commit must abort rather than be ratified by a late capture). The distinction
needed is *our own mark vs a peer's change*, which the txn's own lock registry
can answer at record time.

Status after these two: per-node `unit 307/307`, `inv 111/111`; root-only
advances 102 → 109 and inv to 3 — same class, more sites.

### Why it cannot land site by site

**Anchoring is all-or-nothing (§1), so the conversion cannot be incremental.**
The moment one site anchors while another still locks the node, a coarser
spacing excludes nothing — and a mostly single-writer suite still reports green,
which is a false green on the one invariant the design rests on. Non-per-node
spacings are therefore REFUSED by
`cds_ft_group_attr_set_lock_spacing` (and the env override ignored) unless
`FEATURE_FT_ANCHOR_VALIDATE` is defined; the gate lifts when this table is
fully converted.

### Completeness is a machine check, not an audit

Route every acquire through the anchored wrapper, and under the validate build
assert the raw `ft_meta_lock_acquire` / `ft_dlm_lock` are unreachable directly.
"Did we convert them all" is then answered by the build and the suite rather
than by this table staying accurate.

The suites are largely single-writer, so they check the MAPPING (same node →
same anchor), not exclusion. Proving exclusion holds under a coarser spacing
needs a multi-writer oracle per setting.

## 8. Rejected

* **Partial coarsening** (anchor only descent-reached lock-sets) — unsafe, §1.
* **Up-walk to the anchor** — cannot stop early, §5.3.
* **Stored absolute depth in metadata** — a graft re-homing a subtree would need
  an O(subtree) depth fixup.
* **The schedule on NODE INDEX rather than byte depth** — immune to span length,
  and arguably the honest proxy for branching (§2.1), but **not stable**: a split
  inserts a node and shifts every descendant's index by one, changing their
  anchors. Unlike re-homing (§3, closed by the EXCLUSIVE-source requirement),
  splits run concurrently as a matter of course. Byte positions are immutable
  because keys are; anything counted off tree shape is not.
* **Capping the climb at H node hops** — breaks the collapse: `anchor(C)` is C's
  H-th ancestor and `anchor(P)` is P's H-th ancestor, i.e. different nodes. The
  collapse needs an ABSOLUTE position, not a relative offset.
* **Capped spacing** (`0,1,2,4,8` then fixed) — proposed to bound an up-walk
  climb; unnecessary once the anchor is captured on the descent.
* **Immutable creation-time anchor bit** — *not rejected, held as fallback.* It
  needs no absolute depth, so a climb can stop at the first marked ancestor
  (bounded by the spacing, not by depth), and immutability means a graft needs no
  fixup. Costs a metadata bit and makes the exponential approximate for re-homed
  subtrees. Revisit only if the §5.3 descent measures too expensive.
