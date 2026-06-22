# Transactional flip-latch — design notes (DRAFT, 2026-06-22)

Status: DESIGN / not implemented. Successor to the cross-view fusion + graft_swap
work. This revision replaces the original sketch with a concrete **state-machine**
design for a growable, abortable multi-edge transaction, worked out to the point a
prototype can build to it.

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
