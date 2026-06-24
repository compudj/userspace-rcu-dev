# Transactional flip-latch — design notes (2026-06-22, implemented through 2026-06-24)

Status: IMPLEMENTED. The `urcu_flip_txn` primitive (state machine below) is built,
and EVERY FT mutation commit now rides it — `ft_flip_batch` is retired. The
sections from "All bulk ops unified" onward record what landed; the final
sections record the lock-free (MCAS) direction this serves and the merge
spine-copy interleave rework (the last append-after-install), now DONE — the txn
is freeze-before-install everywhere and its INSTALLED-state append is deleted.
The original design body below (§§1–12) is kept as the rationale of record.

Related: the generic latch core in `src/urcu-flip-latch.h`; the current per-op
machinery in `src/fractal-trie/ft-mutation-helpers.h` (`ft_flip_batch`,
`ft_ord_cell_edge`, `ft_ord_cell_flip`), `ft-insert.h` (`ft_insert_commit`),
`ft-graft.h` (`struct ft_graft_glue`, `cds_ft_graft`), `ft-merge.h` (the spine-copy
fold). Memory: `project_ft_transactional_flip_latch`, `project_ft_bulk_xview`.

---

## 1. Why — what it replaces in `struct ft_glue`

Glue exists to **fake an atomic multi-pointer commit when only one pointer can be
flipped atomically**. Its four jobs:

1. Build-invisible + single forward publish — readers see old XOR new on the
   **forward** path.
2. Deferred back-pointer re-parenting (`ft_glue_defer_edge` /
   `ft_glue_apply_deferred`, the fresh-before-live ordering + per-re-parent
   `synchronize_rcu`) — up-walkers see old XOR new on the **back** path.
3. Fresh-node tracking for OOM rollback.
4. Old-node reclaim.

Jobs 1 + 2 are glue's reason for existing, and job 2's ordering protocol is the
*sole* source of this codebase's publish-ordering bug class (`holder!=NULL`, the
split back-channel, "publish fresh before live", glue-parent immediate-vs-deferred).
A genuinely atomic multi-edge commit makes back-pointers *transacted edges too*,
flipping in the same epoch as the forward edge — so the interleaving becomes
**unrepresentable** and the whole class dissolves. Jobs 3 + 4 don't go away; they
relocate into the transaction's abort path (fresh nodes) and post-commit reclaim
(old nodes).

The empirical case: the entire 2026-05 bug arc was publish-ordering, not allocation.
That is the bug surface this design attacks.

---

## 2. The core idea — a latch is an edge in two states

Today there are **two representations of the same fact** ("slot transitions
old→new") stacked on each other:

- the *edge* layer: `struct ft_ord_cell_edge {slot, old, new}`, the stack
  `edges[3..4]` arrays, and the bespoke slot-trackers `ft_remove_pub` /
  `ft_insert_commit`;
- the *latch* layer: `ft_flip_batch` / `ft_flip_proxy` / `urcu_flip_proxy{ptr[0],
  ptr[1]}`.

`ft_ord_cell_flip_prealloc` is the seam, with three manual loops over the same
edges (install proxies, `urcu_flip_commit`, settle), and the slot tracked *twice*
(in the edge for the settle loop; implicitly by the caller of `ft_flip_batch_add`,
which returns only the tagged flag — which is *why* `ft_remove_pub` /
`ft_insert_commit` exist).

The unification: **fold the slot into the record.** A latch is then a single object
that is

- a **descriptor** `{slot, old, new}` before it is installed, and
- a live **proxy** (its tagged address sitting in `*slot`) after.

There is no edge-vs-proxy duality — there is one latch in one of two states. The
transaction owns install, settle, restore and reclaim, so `ft_ord_cell_edge`, the
manual loops, and the slot-tracking fields of the bespoke structs all disappear.
The conditional-edge idiom (`ft_ord_cell_endpoint_edge`'s "add only if
`*slot==match`") becomes a plain `if (*slot==match) record(...)`.

This works uniformly because structural slots and cell slots **already share one
tag**: `ft_ord_cell_flip_prealloc` casts `ft_flip_batch_add`'s
`cds_ft_inode_flag*` proxy straight into a `ft_ord_cell**` slot, and cell readers
resolve it through the same proxy machinery (`ft_resolve_flip_proxy` /
`ft_ord_cell_resolve_ord`). So a `void **slot` + cast covers both kinds.

---

## 3. State machine

A flip-latch **group** is a transaction. Each **latch** is one transacted edge.

```
                record            install            commit
   ( init ) ──▶ PREPARE ──▶ … ──▶ PREPARE ──▶ INSTALLED ──▶ committed
                  │                              │  ▲
                  │ abort                        │  │ record (append + install now)
                  ▼                              │  │
                freed (plain free,               │  └── … ──▶ INSTALLED
                 no GP, no undo)                 │ abort
                                                 ▼
                                              aborted
                                       (restore *slot=old each;
                                        free fresh; call_rcu group)
```

- **PREPARE** — the group is created here. `record(slot, old, new)` appends a latch.
  Nothing is installed yet, so **no proxy address has been handed out**, so the
  backing can grow by `realloc` freely (see §4). This is the abortable phase: every
  allocation that can fail (fresh nodes *and* latch records) happens here.
- **`install`** — the single PREPARE→INSTALLED transition. It populates each
  recorded latch into its slot (`rcu_assign_pointer(*slot, tag(proxy))`, selector 0
  ⇒ readers still resolve to old). Latch addresses now go live; the head chunk can
  no longer realloc.
- **INSTALLED** — proxies are parked, readers transparently resolve to old.
  `record` here is still allowed for use-cases that must discover edges *after*
  install (the merge fold, §9): it appends to a new chunk and installs immediately.
- **`commit`** — `urcu_flip_commit` flips the selector 0→1 (every proxy resolves to
  new atomically), then settle rewrites each `*slot` to its direct new value, then
  reclaim.
- **`abort`** — two cases, by state (see §6).

Invariants: `record` only in PREPARE or INSTALLED; `install` only from PREPARE;
`commit` only from INSTALLED. The point of no return is `commit` (the selector
flip) — everything before it is recoverable by `abort`.

---

## 4. Backing store — head chunk reallocs in PREPARE, append-only after install

The two growth modes map exactly onto the two states, so the backing is a **list of
chunks where only the head chunk ever reallocs, and only while no address is live**:

- **chunk 0** is the PREPARE-phase **realloc-growable array**. It grows to an exact
  fit as records are appended; safe because nothing references a record's address
  yet.
- `install` freezes chunk 0 in place and takes the addresses (installs proxies).
  Chunk 0 never reallocs again.
- INSTALLED-state `record`s **append new fixed-size chunks** (linked). A new chunk
  never moves chunk 0's or another chunk's live proxies, so every parked address
  stays put.

The degenerate common case (point ops, single-commit builds) is a one-element list:
a realloc'd array plus `next == NULL`, zero overhead. Each latch element is

```c
struct ft_flip_latch {
        struct urcu_flip_proxy proxy;   /* ptr[0]=old, ptr[1]=new, group=&txn->group */
        void                 **slot;    /* settle / restore target */
} __attribute__((aligned(16)));         /* the type-7 proxy tag needs 16B alignment */
```

The **group/selector word lives in the txn header**, not in the chunks, so it is
stable across both head-chunk realloc and chunk append; `proxy->group` always points
at `&txn->group`.

Post-install chunk size: a small fixed constant (e.g. 8), grown by append. Callers
that must add records after install (merge) do **not** pre-size — they just record.

---

## 5. API

The transaction is **generic** — it lives in `src/urcu-flip-latch.h` and knows
nothing about FT. The reclaim domain (`cds_ft.exclusive` / flavor) is *not* a txn
field: `commit`/`abort` return whether a grace period is owed and the embedder drives
the actual free, so the generic layer stays flavor-agnostic.

```c
struct urcu_flip_txn {
        struct urcu_flip_group  group;          /* the selector */
        struct rcu_head         rcu_head;       /* one deferred free for the whole txn */
        void                 *(*tag)(struct urcu_flip_proxy *p); /* proxy -> tagged slot value */
        enum { URCU_FLIP_PREPARE, URCU_FLIP_INSTALLED } state;
        struct urcu_flip_chunk *head;           /* chunk 0 (realloc) ... appended chunks */
        struct urcu_flip_chunk *tail;
        unsigned int            nr;
};

void  urcu_flip_txn_init   (struct urcu_flip_txn *t, void *(*tag)(struct urcu_flip_proxy *));
bool  urcu_flip_txn_record (struct urcu_flip_txn *t, void **slot, void *old, void *new);
void  urcu_flip_txn_install(struct urcu_flip_txn *t);   /* PREPARE -> INSTALLED */
bool  urcu_flip_txn_commit (struct urcu_flip_txn *t);   /* INSTALLED -> committed; true => owe a GP */
bool  urcu_flip_txn_abort  (struct urcu_flip_txn *t);   /* by state; true => owe a GP */
void  urcu_flip_txn_free_rcu(struct rcu_head *h);       /* generic chunk-walking free callback */
```

`record` returns false on OOM (the only failure). `install`/`commit`/`abort` cannot
fail. Per-state `record`:

```
record(slot, old, new):
  PREPARE   → append to chunk 0 (realloc-grow); no install
  INSTALLED → append to tail chunk (new chunk if full) AND
              install tag(proxy) into *slot now
```

INSTALLED `record` **always installs immediately** — there is no deferred-batch
install mode. This keeps the INSTALLED state uniform (every recorded latch is
parked, so `commit`/`abort` restore/settle all records identically) and keeps
`install` a single one-shot transition rather than a repeatable verb. The price is
a constraint on any caller that records in INSTALLED state (only the merge fold,
§9): recording an edge **parks a proxy in that slot at once**, so from that instant
the slot may be read only through resolution. In particular a **raw identity guard**
— `ft_ord_cell_endpoint_edge`'s `if (*slot == match)` — is valid only *before* that
slot is recorded. PREPARE makes this automatic (nothing parked); the merge collect
is the one place that must respect it.

Only `record`'s install needs structure-specific knowledge — the **type-7 tag**
(FT passes `ft_flip_proxy_flag` as the `tag` hook). `commit`/`abort`/settle/restore
write *untagged* `proxy->ptr[1]` / `proxy->ptr[0]` and are tag-free. So the **one and
only structural hook is `tag`**; everything else is generic. Reclaim is embedder-
driven: `commit` and `abort` return "owe a GP", and FT does either
`flavor->update_call_rcu(&t->rcu_head, urcu_flip_txn_free_rcu)` or — on its
`exclusive` fast path / a PREPARE-abort — a plain immediate free. The txn does **not**
track the embedder's fresh nodes; the caller frees those itself (§6).

Element alignment: each latch is `aligned(16)` so a proxy's address can carry FT's
type-7 tag. 16-byte alignment is a safe universal default (malloc already returns it
on LP64), so the generic struct over-aligns harmlessly for embedders with looser tag
schemes.

---

## 6. commit / abort / reclaim semantics

**commit** (INSTALLED): `urcu_flip_commit(&t->group)` (one release store, 0→1);
walk every chunk and `rcu_assign_pointer(*slot, proxy->ptr[1])` (settle — idempotent
for readers, who already resolve the proxy to new); return "GP owed" — the embedder
`call_rcu`s `t->rcu_head` and queues the displaced **old** nodes' deferred frees.

**abort**, by state (the txn restores/frees only its own proxies; the **caller**
frees its fresh nodes either way):

- **PREPARE** — nothing is installed in any live slot, so the txn just **plain-frees**
  its chunks (no grace period, no undo) and returns "no GP owed". This is exactly
  today's `ft_flip_batch_free_unpublished`.
- **INSTALLED** — walk every chunk and `rcu_assign_pointer(*slot, proxy->ptr[0])`
  (restore to old) and return "GP owed" (the embedder `call_rcu`s `t->rcu_head`).

Two nuances that make abort cheap:

- **Fresh nodes free immediately in both cases.** They were only ever reachable via
  `proxy->ptr[1]`, never by a reader, so no grace period is owed. This is sound iff
  the build maintains *every live→fresh edge is a recorded latch, never a direct
  store* — the same invariant that makes commit atomic.
- **The restore store itself needs no grace period.** A reader sees either the proxy
  (→old) or old-direct; both are old, consistent. The `call_rcu` on INSTALLED-abort
  frees only the proxy *memory* (a reader may hold a transiently-loaded proxy
  pointer) and is off the critical path. Unlike commit, abort never flips the epoch
  and never needs an in-line `synchronize_rcu`.

**Reclaim** is one `call_rcu` on `t->rcu_head` whose callback frees chunk 0 and walks
the appended chunks — independent of chunk count, for both commit and INSTALLED-abort.

---

## 7. OOM — abort replaces reserve-up-front

Today `ft_flip_batch_add` is unfailable *only because* `cap` is computed by a
read-only **count pass** up front (`ms_cap = 2·merged_keys+2`, `nr_dst+1`), threaded
via `pre_flip` / `ft_flip_batch_take`. That whole discipline exists to guarantee no
fallible step between the first install and commit.

With a real abort, the discipline collapses to one rule:

> **All allocation lives in PREPARE (and in INSTALLED `record`s, which are also
> abortable). The commit is the only point of no return.**

The PREPARE backing grows by realloc as edges are *planned*, so the **count pass is
deleted** — the plan *is* the count, gathered during the build already being done.
A mid-PREPARE OOM aborts via the plain free. The reader cost is unchanged: abort
parks/restores the same proxies a reader already tolerates.

Net trade for OOM: a count pass on every op ⇒ a deferred proxy GP on the rare OOM
path. OOM is cold; the count pass is hot.

---

## 8. Multi-commit ops and the drain — detach + graft

A flip **group is trie-agnostic**: `urcu_flip_group` is one selector word, and
`urcu_flip_commit` flips an arbitrary proxy set atomically regardless of which trie
each slot lives in (the only trie coupling, `ft_flip_batch.ft`, is just the reclaim
domain). So a *cross-trie move* could in principle be one group, one commit.

**But a move that re-parents a deep subtree cannot be a single flip**, because it
needs a **drain** — a grace period that must sit *between* two reader-visible epochs:

1. commit-1 unlinks R from src (and points R's back-pointer at the standalone root);
2. **drain** (`update_synchronize_rcu`) — wait until no reader still holds a from-src
   path up through old `Sp`;
3. commit-2 publishes R's new parent → `Dp` (forward attach + back-pointer, one flip).

A single flip flips every proxy at one instant; there is no "unlinked-from-src but
not-yet-reparented-into-dst" epoch for the grace period to live in. The order is
also forced (detach then graft, never the reverse): R has exactly one back-pointer,
so it cannot transiently have two parents. **Two commits are mandatory.**

Therefore the **reservation boundary is hard at commit-1**: there is no abort after
commit-1, so everything fallible for *both* commits — the fresh dst spine and every
latch G2 will install — must be allocated before commit-1. Abort cannot relax this
boundary; it only operates inside PREPARE. The shape:

```
PREPARE:   build fresh dst spine; record G1 (detach) and G2 (attach) latches   [abortable]
install G1; commit G1                                                          [unfailable tail]
drain (synchronize_rcu)                                                        [unfailable]
install G2; commit G2                                                          [unfailable]
```

The drain is `synchronize_rcu`: failure-free, costs a grace period, cannot OOM. So
"do not fail after the first commit" is satisfied structurally — there is **no
allocation after commit-1**, not because the detach can be undone (it cannot). The
two groups can share one txn (two selector words) or be two txns; either way the
fallible work is all in PREPARE.

This is the irreducible ceiling from the cross-view analysis: the drain (and
`ft_skip_reanchor`) are *not* publish-ordering problems the latch removes — they are
"reader holds a stale deep reference", and they survive the latch.

---

## 9. The merge fold — record after install

One current pattern genuinely needs `record` in INSTALLED state: the occupied-dst
spine-copy merge fold. It installs the **structural** latches, then the
collect-walk reads the **merged** view *through* those installed proxies
(`ft_tls_resolve_merged`, plus the GT up-walk resolving dst-origin parents via
`ft_get_parent_rcu`) to discover the **cell** latches, and only then records them —
all under one group, committed in one flip. Cell discovery *depends on* the
structural install (an up-walk through a not-yet-installed parent would read old and
walk into the old structure), so the latches cannot all be recorded before a single
`install`.

The append-chunk design handles this directly: after `install`, the collect's
`record`s append cell latches into new chunks and install them immediately, all
resolving to old until the single `commit`. Because those post-install `record`s are
**fallible → INSTALLED-abort**, merge can **shed its exact `ms_cap` pre-size**: it
records cell latches as the collect finds them and aborts on OOM; the commit is
reached only if every append succeeded.

Because INSTALLED `record` installs immediately (§5, no deferred-batch mode), the
collect must obey the **raw-read constraint**: once it has recorded a cell edge, that
slot holds a proxy, so any later read of it must resolve (`ft_ord_cell_resolve_ord`),
never a raw `*slot ==` compare. Verification before coding (§11) is therefore an
audit, not a design fork — confirm the collect reads the merged order through the
*structural* proxies and never raw-reads a cell slot it has already recorded (it links
consecutive cells via disjoint `ord_next`/`ord_prev` fields and the only raw guards
are the head/tail endpoints, read once, so this is expected to hold).

---

## 10. Reader cost — largely sunk

Build-invisible glue needs zero reader changes (the live slot holds old directly
until the single publish). A proxy transaction parks a proxy in every transacted
slot, so every reader of a transacted slot must resolve it. That universal
resolution cost is **already paid**: the cross-view fusion made proxy resolution
universal on every read hot path — child fetch (`ft_node_get_nth_reanchor`), root
(`ft_root_dereference`), parent (`ft_get_parent_rcu`), skip-decode
(`ft_skip_to_compressed`), cell reads (`ft_ord_cell_resolve_ord`). Extending the
latch into more publish sites adds no new resolve sites; it only makes a proxy
*present* more often. This tilts the design from "measure first" toward "worth
prototyping" — but the iterate/lookup benchmarks should still confirm the
proxy-present hot path before structure-wide rollout.

---

## 11. Open questions / verification points

- **Placement — DECIDED: generic** `urcu_flip_txn` in `src/urcu-flip-latch.h`, one
  structural hook (`tag`), embedder-driven reclaim (§5). FT passes
  `ft_flip_proxy_flag` and drives `call_rcu` via its flavor / `exclusive` fast path.
- **Tag coverage — AUDITED 2026-06-22.** Type-7 tag collision is *provably
  impossible* for every slot kind (nibble `0xF` is the `FT_NULL` type-index — never a
  real node; the skip-length lives in disjoint high bits; cells are 32B-aligned,
  nibble 0). Readers resolve correctly for **root, internal child, skip slots** (the
  riskiest — every reader resolves the proxy *before* the skip-length decode: descent
  ft-descent.h:356, `ft_node_get_nth_reanchor` :1771, `ft_skip_to_compressed` :1383),
  **cell links, parent back-pointers** → all SAFE to transact. **`external_nodes` is
  the one gap:** six ordered-query/select readers in `ft-ordered-query.h`
  (:256/:537/:996/:1186/:1361/:1538) load it via the non-resolving
  `ft_dereference_acquire` and consume the raw value (e.g. store a tagged proxy into
  `iter->node` at :268). Route them through `ft_resolve_flip_proxy` /
  `ft_dereference_external` before transacting external_nodes. **This is already a
  live latent bug** — insert's one-commit splice publish parks a proxy into
  external_nodes today (`ft_insert_park_external_nodes`, ft-insert.h:311, gated on
  ordered_list), so a concurrent nth/select query can already misread it. Track + fix
  independently of this design.
- **Merge collect raw-read audit — AUDITED 2026-06-22: SAFE.**
  `ft_merge_ord_interleave_collect` (ft-merge.h:713) reads every cell ord-slot through
  `ft_ord_cell_resolve_ord`, never calls the raw `ft_ord_cell_endpoint_edge` idiom (it
  inlines resolved head/tail reads at ft-merge.h:825/:857), and its record-set vs
  read-set never alias on a re-read (disjoint `ord_next`/`ord_prev` fields). So
  immediate-install (§5) is safe for the merge fold.
- **Post-commit side effects stay caller-side (first cut).** `ft_remove_pub`
  (pigeon bitmap clear) and `ft_insert_commit` (`free_old_cn`, `count_from` nr_keys
  bump, `incoming_byte`/`skip_slot` finalize, `publish_to_parent`) carry post-commit
  work that is *not* a slot transition. Keep it as caller code after `commit()`
  returns; a `txn_defer(hook)` that absorbs it is a separate, later ambition.
- **Degraded fallback** — `ft_ord_cell_flip` falls back to non-atomic sequential
  stores when the batch alloc fails (point-op splices, transient/self-healing).
  Install-as-you-go can't degrade after the first install, so this becomes "reserve
  the tiny fixed chunk up front; if it fails, degrade *all* edges to direct stores;
  else record all." Only the small point-ops need it.
- **Long-lived parked proxies** — abort-on-OOM parks proxies across several fallible
  allocs. Single-writer exclusion (`CDS_FT_SCOPED_WRITER`) means only one txn is open
  at a time; confirm `urcu_flip_commit` epoch semantics tolerate a long-open
  transaction (they should — the selector is per-group).

---

## 12. Proving ground

Smallest first target that exercises the state machine end to end: convert
**`cds_ft_graft`** (the cleanest attach — single live boundary) from the
deferred-ordered-store glue protocol to a PREPARE/install/commit transaction.
Measure two numbers:

1. lines of ordering-protocol **deleted** (deferred back-edges, fresh-before-live,
   per-re-parent `synchronize_rcu`, the `pre_flip` reserve plumbing + count pass) —
   the simplification, the number that matters;
2. settle/reclaim delta — the only real cost.

Existing graft + cross-view oracles + VAM guard it. If it lands well, the next
consumers in order are `graft_swap`, then the merge spine (extend its existing flip
to the overlap spine), then the pure back-pointer-ordering publishes (the
external-nodes two-phase publish and the split back-channel become no-ops by
construction).

### Implementation status (2026-06-22)

- **Primitive DONE + validated (committed `f2ff0d07`).** `urcu_flip_txn` is
  implemented in `src/urcu-flip-latch.h` (create / record / install / commit /
  abort / destroy / free_rcu, chunk-list backing — head chunk reallocs in PREPARE,
  fixed chunks append after install, one `tag` hook + embedder-driven reclaim;
  install + commit-settle are RELEASE, the abort restore is RELAXED — it moves the
  slot back to an already-published value).
  Validated standalone (basic commit, PREPARE-abort, INSTALLED-abort,
  post-install record, growth-by-realloc); compiles clean in the FT build,
  unused so far.

- **Graft-conversion reconnaissance — a sharper picture than §12 assumed.** The
  glue map (ft-graft.h / ft-mutation-helpers.h:1455-2108) shows:
  - The **simplest graft (NOSPLIT in-place)** already commits **one**
    reader-visible edge atomically (the forward slot via `ft_flip_batch_commit`,
    fused with ≤4 cell-splice edges when the list is on) plus one back-pointer
    that is safe because it lands on a **drained-exclusive** payload. Little to
    win there.
  - The **~118-line deferred-edge ordering protocol** that the latch makes
    unrepresentable (`struct ft_glue_deferred_edge`, `ft_glue_defer_edge`,
    `ft_glue_defer_edge_origin`, `ft_glue_apply_deferred`, `ft_glue_is_fresh`,
    `ft_glue_set_publish` + the glue deferred fields) lives in the **diverge/glue
    shape**, and `struct ft_glue` is **shared with `graft_swap` and `merge`**. So
    converting plain graft **alone does not delete it** — the deletion lands only
    once all three consumers convert and the glue can retire.
  - The one `update_synchronize_rcu` drain and the pre-commit reserve are
    **irreducible** (§8): the cross-trie deep-subtree move needs the grace period,
    and there is no abort after commit-1, so the reserve relocates, not deletes.
  - `ft_glue_track`/`abort`/`free_old` (fresh-node + post-commit reclaim, ~96
    lines) **relocate** into the txn's reclaim hooks rather than disappear.

  Net: the graft pilot's honest near-term value is **validating the primitive +
  the conversion pattern + measuring the per-op settle/reclaim cost** — the
  ~118-line LOC deletion is a *cross-consumer* payoff realized across the
  graft → graft_swap → merge migration, not from graft in isolation.

### First conversion landed — graft GLUE diverge, list off (2026-06-22)

The diverge-split graft (`FT_GRAFT_PREP_GLUE`) now commits through `urcu_flip_txn`
when the ordered list is off (gated `!ordered_list_set && !pre_flip`; list-on keeps
the run-splice fusion, the merge rekey keeps the proven path). `ft_glue` gained a
`txn` field; `ft_glue_txn_commit` (the dual of `ft_glue_apply_deferred` +
`ft_glue_publish`) records every live back-pointer re-parent **and** the forward
publish into one group and flips them atomically — the deferred-edge ordering
window is gone for this shape.

Two design points settled differently than §12's first sketch:

- **Bookkeeping runs at commit, not during the build.** The §12 plan recorded
  edges + ran `ft_set_parent_slot` *early* (build time). That is unsafe on the
  **abort** path: early bookkeeping mutates a LIVE node's `parent_slot_offset`
  (writer-read by `ft_get_parent_slot`), and a later build-step OOM would leave it
  corrupt with `meta->parent` still old. So the build is left **unchanged** (live
  edges still queue in `g->deferred`); `ft_glue_txn_commit` replays them through
  the txn **after the drain**, where abort is already impossible. `g->deferred`
  survives as a parameter carrier for this first cut (its *ordering role* is what
  the txn retires); it disappears only when the build records straight into the
  txn — deferred to the later phases.
- **Bounded reserve, not fallible-record.** The cluster is floor-bounded, so the
  txn is reserved to that bound up front (`urcu_flip_txn_reserve`, added to the
  primitive) and records can't fail mid-replay — matching the existing glue floor
  arrays. The abort-replaces-reserve model (§7) stays the plan for the unbounded
  **merge** spine.

Forward edge captured via the existing `_ft_publish_to_parent(&rec)` recorder
(forward slot + compressed-parent SKIP_X dual), replayed into the txn — byte-for-byte
the stores the legacy publish performs, now atomic with the back edges.

Validated: unit `test_graft_diverge_no_list` + concurrent inv
`inv_graft_no_list_diverge` (graft-split / detach-heal oscillation, EAGER readers —
SPECULATIVE oscillation hits a *pre-existing* churn livelock, reproducible on the
legacy path). 4 feature configs (default / no-skip / no-compress / both) unit
252 / inv 48, VAM-targeted (period 1) on both new tests, ASAN clean (no leaks),
20× stress. Next: list-on run-splice edges into the txn, then NOSPLIT, then
graft_swap, then merge, then delete the dead deferred-edge machinery.

### Governing rule refined — hidden immediate, live via txn (2026-06-23)

Converting `graft_swap` surfaced the rule that actually governs every bulk op,
and it is simpler than "fold every back-pointer into the txn":

> A pointer that is **not reader-observable** during the commit window is set
> **immediately** with a plain store.  Only a **live** (reader-observable)
> pointer rides the txn, so its flip is atomic with the forward publish.

The discriminator already exists as `dst_origin`: `ft_glue_apply_deferred` sets
the `!dst_origin` (hidden) edges immediately; the `dst_origin` (live, reachable
via the OLD spine until the forward publish) edges are switched atomically by
the flip.  So `ft_glue_txn_commit_edges` is: apply the hidden back-pointers
immediately (in recorded order), then record only the live back-pointers + the
forward edge + the `<=4` ordered-list cell edges into the txn and commit.

Why this matters (the bug it fixes): the earlier "fold every back-pointer into
the txn" sketch *deferred* the hidden back-pointers past the commit's own skip
resolution.  `ft_skip_to_compressed(skip(cn))` recovers `cn` by reading the
skip child's parent, and for a `graft_swap` replace that child is re-parented by
a *sibling* edge in the same commit.  With the back-pointers deferred, the
recorder read the child's **stale** parent and re-parented the wrong compressed
node, leaving the published `cn` with a NULL parent — a dangling skip slot.
Setting the hidden back-pointers immediately (in recorded order, the set-publish
edge last) makes the skip resolve correctly, exactly as the legacy
`apply_deferred` did.

`graft_swap` replace (EXACT + diverge; KEY_SHORTER stays legacy) now commits
this way: the whole inserted cluster is the drained swap content, so *all* its
back-pointers are hidden — only the forward replace edge + the run-replace cell
edges flip.  Validated: 4 feature configs unit 252 / inv 50, VAM period-1 on the
five graft_swap unit shapes + the four `inv_graft_swap_*` oracles, 120× stress,
ASAN clean (library; a pre-existing `populate_at_prefix` stack over-read in the
test harness was fixed alongside).  The now-unnecessary `atomic_parent` plumbing
on `_ft_publish_to_parent` (added by the folded-back-pointer sketch) is removed.

Next: realign **graft** (re-tag the diverge-split's displaced `cn->child` as
`dst_origin` — it is reachable via the old `cn` spine, hence live — so it rides
the txn in every graft path, list-on included) and **merge** to the same rule.

### Graft realigned (2026-06-23)

The diverge-split graft now follows the same rule.  Its one live re-parent --
the displaced `cn->child` (reachable through `cn` until the forward publish
replaces it) -- is tagged `dst_origin` so it rides the flip-txn instead of
`ft_glue_apply_deferred` setting it immediately ahead of the forward edge
(`ft_split_compressed_graft_build`, all three suffix_len cases; gated on
`glue->txn` so the merge-rekey no-txn path keeps fresh-before-live).

This settled the external-parent question: a flip proxy parked on an **external
node's parent** IS resolved by the up-walk readers (`ft_get_parent_rcu`,
`ft_skip_to_compressed`, `ft_skip_reanchor`, each via the trailing
`ft_resolve_flip_proxy`).  So an external `cn->child` -- the leaf of a
fully-compressed path -- flips atomically like any other child; external
parents are NOT an exception to "live → txn".  A stale comment on the NOSPLIT
displaced-external store claimed those readers do not resolve such a proxy; it
was corrected.  (That displaced-external back-channel still uses a direct
fresh-before-live store, which is simplest there; it could equally ride the
flip-latch.)

Validated: 4 configs unit 252 / inv 50, 40× stress + VAM period-1 + ASAN on
`inv_graft_no_list_diverge` (a single fully-compressed key split by a graft, so
its external leaf is the displaced child, under concurrent point-lookup readers
that descend through the skip path and resolve the parked proxy).

Remaining: merge already uses `dst_origin` for its live dst children; confirm it
matches the unified `ft_glue_txn_commit_edges` shape.  The `g->deferred` array
and `ft_glue_apply_deferred` stay -- they ARE the immediate-store mechanism for
the hidden bucket under the rule, not legacy to delete.

### Merge audited — already conforms (2026-06-23)

`cds_ft_merge_at`'s spine-copy commit was effectively the template for the rule
and already obeys it: step 2 `ft_glue_apply_deferred` wires the src-origin
(hidden, drained) subtrees immediately ("dst-origin edges are NOT applied here
-- they go through the flip"); step 3 stages every dst-origin (live) child
re-parent + the forward publish slot into one `ft_flip_batch`; step 3b adds the
ordered-list interleave cell edges to the SAME batch; step 4 `urcu_flip_commit`
flips them together.  It uses the lower-level flip-batch rather than the generic
`urcu_flip_txn`, but that is a primitive choice, not a rule deviation, and its
merged-view interleave collect (`ft_tls_resolve_merged`) reads through the
staged proxies -- a delicate path with no reason to churn.

The only places any bulk op still applies a LIVE re-parent immediately are the
no-flip fallbacks -- the merge same-trie **rekey** diverge (no txn; line ~1732)
and the NOSPLIT **displaced-external** store -- both fresh-before-live by
construction.  They are correct (the new parent is fresh / the cluster is
drained) and could be flip-converted for uniformity, but are not required by the
rule.  Optional follow-ups, in rough order: (1) flip-convert those two
fresh-before-live fallbacks; (2) migrate merge from `ft_flip_batch` to
`urcu_flip_txn` so all bulk ops share one primitive.

Status: graft_swap, graft, and merge all follow "hidden immediate, live via the
flip" on their primary paths.  `g->deferred` + `ft_glue_apply_deferred` are the
hidden-bucket immediate-store mechanism and stay.

### Fresh-before-live fallbacks flip-converted (2026-06-23)

The two remaining spots that applied a LIVE re-parent immediately now ride the
flip, so every reader-observable pointer in a bulk op flips atomically:

- **NOSPLIT displaced-external** (`ft_store_at_graft_point_commit`): the
  displaced leaf's back-channel (`displaced->prev` / `cell->parent = branch`) is
  recorded as a `dst_origin` edge on the txn path, flipping with the forward
  publish + run-splice instead of a fresh-before-live direct store.  A flip
  proxy parked on an external's parent is resolved by the up-walk readers.  New
  oracle `inv_graft_displaced_external` (confirmed to drive the path 75× in 2 s).
- **Merge same-trie rekey diverge** (`ft_merge_graft_subpos_inplace`): mirrors
  `cds_ft_graft` -- a glue flip-txn is created before the build (tagging the
  displaced old child `dst_origin`) and the GLUE path commits via
  `ft_glue_txn_commit`; NOSPLIT keeps its existing path.  Exercised 278k+ times
  under concurrent readers by the existing merge inv suite.

The legacy (no-txn) paths -- merge rekey NOSPLIT, and any future no-txn caller --
keep fresh-before-live, which stays correct.

### All bulk ops unified on `urcu_flip_txn` (2026-06-23)

The last unification is done: the **merge spine-copy commit** and the **graft
in-place NOSPLIT point-store** -- the only bulk-op commits still on the bespoke
`ft_flip_batch` -- now ride `urcu_flip_txn`, so every structural bulk commit
(graft / graft_swap-diverge / merge spine-copy / merge rekey) shares one
primitive.  `ft_flip_batch` survives only for the non-bulk users (point insert,
the ordered-cell point ops, graft_swap's own batches).

The one obstacle was the in-place store: it hands a flip proxy to
`ft_node_set_nth` *before* the store runs, and a recompact may relocate the slot
-- so the txn's record-then-install model (slot known at record time) did not
fit.  Resolved with two additions to the primitive
(`src/urcu-flip-latch.h`), the txn analogue of an embedder-managed
`ft_flip_batch_add`:

- **`urcu_flip_txn_reserve_slot(t, old, new, &latch)`** -- on a *reserved* txn
  (stable latch addresses), append a latch and return its tagged proxy for the
  embedder to place itself; the slot is not yet known.
- **`urcu_flip_txn_bind_slot(latch, slot)`** -- bind the slot once the store has
  placed the proxy (post-recompact address), so commit can settle it.

A placed proxy is immediately reader-visible, so a new `placed` flag makes
`commit()` flip the group rather than take the single-edge bare-store fast path,
and `abort()` restore it even from PREPARE.  `ft_node_set_nth` is unchanged.

Consequences: the shared `pre_flip` flip-batch reservation became a pre-reserved
txn (`pre_txn`) threaded through `ft_merge_at_inner` and the same-trie rekey
(sized per shape: `(m+1)+(2n+2)` for spine-copy, `FT_GLUE_FLOOR_DEFERRED+6` for
graft); `ft_graft_keylen`'s `!pre_flip` gate is gone (`glue.txn` is always
taken-or-created, never retired for NOSPLIT); and the now-dead
`ft_flip_batch_take` / `ft_flip_batch_commit` / `ft_glue_publish_run` /
`ft_ord_cell_flip_rec_run` were removed.  The merge spine-copy's
interleave-collect-over-merged-view is unchanged: txn proxies and flip-batch
proxies are the same type-7 tagged `urcu_flip_proxy`, so `ft_resolve_flip_proxy`
(incl. the `ft_tls_resolve_merged` path) resolves both identically.

Validated: 4 feature configs (default / no-skip / no-compress / both) unit
252 / flip_latch 18 / inv 51 0-violations; VAM-targeted (period 1) on the
merge/graft unit shapes + graft oracles; ASAN clean (no leaks); 20x cross-view
stress on the spine-copy / graft-store / rekey oracles.

### Every mutation on `urcu_flip_txn`; `ft_flip_batch` retired (2026-06-24)

The two remaining mutation commits on the bespoke `ft_flip_batch` -- the
ordered-cell point ops and the insert one-commit -- moved to `urcu_flip_txn`,
and `ft_flip_batch` (struct + `ft_flip_proxy` + alloc / add / reclaim /
free_unpublished) was deleted.  Every FT mutation commit now expresses its edge
set through the one `urcu_flip_txn` descriptor.

To keep the point paths' single allocation, the txn gained a single-allocation
bounded mode (`urcu_flip_txn_create_bounded`: header + an inline head chunk in
one malloc; a `head_inline` flag so destroy frees it with the header).  The
ordered-cell point-op flip (`ft_ord_cell_flip`) records its `<=4` edges and
commits; a lone edge takes the bare-store fast path.  The insert one-commit
(`ft_insert_one_commit`) maps its three forward-publish shapes onto the txn: the
set_nth branch via `urcu_flip_txn_reserve_slot`/`bind_slot` (the slot the
recompact-capable `ft_node_set_nth` relocates), the `external_nodes` prefix-key
publish via a plain record, and the publish-to-parent shape via
`_ft_publish_to_parent(&rec)` capture-then-record -- so the compressed-parent
skip-slot dual now flips ATOMICALLY with the forward edge instead of just after
it.  The live re-parent edge and the `<=4` ordered-list neighbour edges record
into the same txn.  Insert/remove perf A/B (list-on, 1 writer, -O2 -DNDEBUG) was
neutral (+0.3%).

### Lock-free (MCAS) direction and the one remaining rework (2026-06-24)

The motivation for unifying every mutation onto `urcu_flip_txn` is that the
flip-latch is a *single-writer* MCAS: the same `{slot, old, new}` descriptor; the
install is plain stores of a tagged proxy plus one selector flip (safe only
because the app holds the writer mutex), where a true MCAS would CAS each slot
`old -> descriptor` in sorted address order, flip a status word, and retry on
contention.  The reader side (RCU + descriptor resolution) is nearly identical.
So once every mutation is a descriptor, swapping the commit *body* for an MCAS
gives **multi-producer lock-free writers** as one localized change.

MCAS adds a hard constraint: **install in sorted address order** (deadlock-free
acquisition) -> the edge set must be **frozen before install**; *no append after
install*.  Every FT mutation commit now obeys this.  The last exception -- the
`cds_ft_merge_at` spine-copy ordered-list interleave -- was reworked on 2026-06-24
(below), and the `urcu_flip_txn` INSTALLED-state append, which existed only to
serve it, was deleted.

### The merge spine-copy interleave: two-pointer suffix merge (2026-06-24)

The interleave (`ft_merge_ord_interleave_collect`) re-homes the surviving source
cells into dst's ordered list.  It used to *append after install*: it installed
the structural proxies, set `ft_tls_resolve_merged` so an up-walk over the staged
structure resolved to the merged view, walked that merged structure with the
inequality oracle to enumerate the heads in key order, then recorded the cell
edges while INSTALLED.  Its iterator could only advance via the merged
back-edges, which were not set pre-commit -- hence the install-first dance.

It now reconstructs the merged ORDER WITHOUT any installed proxy, by a two-pointer
key-order MERGE of the two already-sorted LIVE cell runs:
  - the dst region (`ms_cursor` .. D's max head, walked via `ord_next`), up-walked
    LIVE over the still-intact D (`ft_rebuild_key_upwalk`), suffix =
    `full_key[dst_key_len..]`;
  - the surviving src run (`ms_s_first .. ms_s_last`), each carrying its key
    suffix `full_key[src_key_len..]`.

Both share the merge-point prefix, so they merge by comparing the suffixes (an
equal-suffix step is a collision: the dst head wins, its chain already absorbs
the src head via `ft_glue_apply_splices`, so the src head is dropped -- a floating
duplicate, never a distinct reachable head).  The cursor-block / survivor-block /
trailing-edge splice logic is reused verbatim, driven by the suffix compare
instead of the structural walk.  The cell edges are recorded into the txn in
PREPARE; commit auto-installs.  `ft_tls_resolve_merged` is gone.

The one wrinkle: the collect runs AFTER `ft_merge_unlink_src_subtree` detaches S,
and that detach does not reliably NULL S-root's parent, so a post-unlink src
up-walk could run past S into stale src structure.  The src-head suffixes are
therefore CAPTURED BEFORE the unlink (a fallible pre-pass while S is fully
attached -- the run is bounded by `cnt_src`; the variable-length suffix bytes
pack into a realloc-growable pool addressed by offset, since a byte pointer would
dangle across the realloc), then merged against the LIVE dst suffixes (D stays
intact until the flip) at the post-unlink collect.  Identity key_map only (the
existing collect constraint).

With the append gone, `urcu_flip_txn` is a single-chunk transaction: `record()`
is valid only in PREPARE (asserts it), the chunk realloc-grows while nothing is
installed, and `install` / `commit` / `abort` / `destroy` operate on the one head
chunk -- the `next` / `tail` / fixed-`CHUNKN` append machinery was removed.  One
subtlety surfaced by that removal: the single-allocation bounded txn embeds its
chunk at `t + 1`, and the chunk's `latches[]` are `aligned(16)`, so the header's
size must be a multiple of 16 (else the compiler's aligned vector moves on a
misaligned latch fault).  This was holding only because the field count happened
to sum to 64; it is now made explicit with `aligned(16)` on `struct
urcu_flip_txn`.

A merge-interleave defect is an intermittent cross-view race the suite may not
catch deterministically, so the rework was gated hard (all GREEN): 4 feature
configs (default / NO_SKIP / NO_COMPRESS / both) ft_unit + flip_latch + ft_inv
(0 violations); VAM (period 1) on the ordered-list merge shapes + the merge
cross-view oracles (`inv_merge_spinecopy_cross_view`,
`inv_merge_src_spinecopy_cross_view`, `inv_merge_cross_view`, `_src_cross_view`,
`_root_src_cross_view`) at 8x; those oracles + the atomicity / no-escape oracles
at 20x; ASAN clean (no leak / overflow / use-after-free).
