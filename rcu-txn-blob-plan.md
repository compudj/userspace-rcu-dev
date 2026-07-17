# Plan: rcu-txn binary blobs — atomically-mutated in-place, modular, three variants

## Context

The txn family lets a mutation compose several word-granular edits into one
atomic commit (the MCAS engine `rcu-mcas.h`, or the single-writer flip engine
`rcu-txn-sw.h`). `rcu-txn-bitmap.h` layers a *logical* thing — a bitmap — over an
array of transacted `uintptr_t` words so a bit flip folds into the same commit as
the mutation it accompanies. This plan does the same for an **opaque binary
blob**: an arbitrary C structure encoded across an array of transacted words, so
that **all of its fields switch to new values at a single linearization point**,
updated **in place** on a **fixed address**, with **no reallocation**.

The natural RCU way to atomically replace a struct is copy-on-write: allocate a
new copy, `rcu_assign_pointer`, `call_rcu` the old one. One atomic store, no
per-word machinery. The blob exists for the cases COW cannot serve:

- **The address must not move.** Other structures hold a raw pointer *into* the
  blob's storage (a pool slot, an embedded record). COW relocates the object and
  invalidates those pointers; in-place mutation keeps them valid.
- **No allocation on the write path.** Pinned / fixed region, bounded-memory or
  real-time contexts where realloc is not available.
- **Composition.** A blob field update folds into one commit with *other* txn
  edits — update a blob field AND splice a list node AND flip a bitmap bit, all
  atomic — touching the interior without republishing a container.

The cost is `ceil(N/7)` records per commit, so the blob is the right tool for
**small, fixed-address, composable** structs (a few words, ≤ ~64 bytes), not for
transactionally rewriting large buffers — there COW is cheaper.

### Memory model (decided): intra-process, fixed slot

A parked word holds `(proxy_pointer | TAG)`, a **process-local** pointer into the
slab. The blob therefore targets a **stable address within one process** (a pool
entry, a node other nodes point into, a single-process `mmap`). It does **not**
serve cross-process shared memory: a proxy pointer is meaningless in another
address space, and the whole park/resolve mechanism breaks there. A shared-memory
blob would need offsets-not-pointers and a versioned region — a different header,
explicitly out of scope.

### Design stance: pay for what you use

The blob is **not** one monolithic mechanism. It is a lean core — an array of
transacted words plus an encoding — with three **independent, optional** layers
bolted on through a control word (lock, tombstone, seqcount). A blob turns on only
what its callers need; a blob that needs none of them is a **bare `uintptr_t
words[]`**, embedded anywhere, exactly as lean as the bitmap. The layers and the
three engine variants are the two axes of the design and are treated as such
below.

## Encoding (identical across all variants)

Split the blob into **7-byte sections, one per transacted word**, payload in byte
lanes 1..7, the low byte (lane 0) spent as the engine tag:

```
#define URCU_TXN_BLOB_BYTES_PER_WORD    7
#define URCU_TXN_BLOB_NR_WORDS(nbytes)  (((size_t)(nbytes) + 6) / 7)

  word w, byte j (0..6)  <->  physical bits (8 + 8*j) .. (8 + 8*j + 7)
  encode:  word |= (uintptr_t) src[7*w + j] << (8 + 8*j);   /* lane 0 stays 0 */
  decode:  dst[7*w + j] = (uint8_t) (word >> (8 + 8*j));
```

Rationale:

- **Bytewise, not bitwise.** A byte lane never crosses a word boundary, so each
  word is an independent, self-contained record. (A bit-granular 63-data-bit
  split — the bitmap's scheme — needs cross-word shifting; here we spend 8 bits,
  not 1, to buy clean byte lanes and a plain `>> 8` payload.)
- **Endianness-independent by construction.** The packing is arithmetic shifts on
  the integer value, not `memcpy` into a byte offset, so lane 0 is always the
  byte containing bit 0 (the LSB) on any endianness — exactly where the engine
  tag must live.
- **Engine tag contract.** Only bit 0 is *required* clear for a settled literal
  (`(value & URCU_MCAS_TAG) != URCU_MCAS_TAG`); we reserve the whole low byte for
  byte alignment. A parked word has bit 0 set (`proxy | TAG`) and its other bits
  are the proxy pointer, not payload — readers must resolve, never read directly.
- **Zero-init is a valid empty blob** (all words zero, lane 0 clear), so
  demand-zero / `calloc` storage needs no init, same as the bitmap.
- **Last word zero-padded** when `nbytes % 7 != 0`; load extracts only `nbytes`.

### Fields vs words, and the straddling wrinkle

An opaque blob packs *raw struct bytes* with **no field alignment**, so a single
logical field can straddle a word boundary (a 4-byte field at byte offset 5 lands
in words 0 and 1). This matters for reads (below): a field contained in one word
is read atomically with a single resolve; a field that straddles needs a
multi-word snapshot. Two levers:

- A **≤ 7-byte blob** is one word — every read of it is single-word atomic.
- **Field-aligned mode** (optional): pad each field to a word boundary so every
  single-field read is one word and thus seqcount-free, at the cost of padding.
  Opaque-packed mode is the default; field-aligned is for callers that do many
  single-field reads and want them snapshot-free.

## The control layers (optional) — independent presence AND placement

The optional machinery is three independent layers, each an embedder-placed word
(or bit within one), **not** a single packed struct. Presence is opt-in (a blob
using none is a bare `uintptr_t words[]`); *placement* is opt-in too, because the
layers have **opposite cacheline needs**:

| layer | needed when | who touches it | placement |
|---|---|---|---|
| **lock** (writer excl.) | hybrid engine | writers only (acquire/release) | **cold metadata**, away from data |
| **tombstone** (liveness) | slot has a live/dead lifecycle | writers set; readers test | **cold metadata** (with lock) |
| **seqcount** (multi-word snapshot) | readers snapshot > 1 word | writers bracket; readers sample twice | **hot, within the data cachelines** |

The placement split is not cosmetic:

- **seqcount → with the data.** A consistent read samples it before and after the
  data reads; on a separate line every snapshot read pays an extra cache miss,
  which defeats the whole "passive cheap read" point. The writer bumps it only
  while mutating the data (which already owns that line exclusively), so
  co-location adds no cross-core traffic on the write side.
- **lock → away from the data.** Acquirers spin-CAS the lock word under
  contention; on the data line, every *failed* CAS from a spinning writer would
  invalidate readers' copy of data they never needed to reload — false sharing
  that scales with writer contention and directly costs reader scalability. Off
  the data line, lock churn is invisible to readers.
- **tombstone** rides with the lock in metadata (liveness checks are colder than
  data reads, and tombstone-set folds into the same MCAS commit as an unlink).

So the primitive API takes **bare pointers**, bitmap-style: the encoding operates
on `uintptr_t *words`, and each enabled layer is a *separate* caller-provided
location — a `seqcount` word the embedder puts on the hot data line, a `lockword`
(lock + tombstone bits) it puts in cold metadata. The packed

```c
struct urcu_txn_blob { unsigned long ctrl; uintptr_t words[]; };
```

is just *one* convenience layout for callers who don't care about placement; an
embedder like the fractal trie ignores it and places each layer by hand.

Bit layout, whether packed into one `ctrl` or split across words:

```
bit 0     : MCAS tag   — reserved on any word that is MCAS-set (the lock/tombstone
                         metadata word); a data/seqcount-only word need not reserve it
bit 1     : LOCK       — writer-only
bit 2     : TOMBSTONE  — reader liveness
bits 3..  : GENERATION — reader seqcount (parity, or split enter/exit)
```

**Common case: all three on one word.** Splitting is the *optimization*, not the
default — in most use cases lock + tombstone + seqcount share a **single** word
(smallest footprint, and operations *fuse*: a reader's one load yields both
liveness and the generation sample; the hybrid holder's release-and-advance clears
the lock and bumps the generation in one store). The API addresses each layer by
pointer and **detects when the pointers coincide**, operating on the one physical
word instead of several. FT's split (lock + tombstone in cold metadata, seqcount
on the hot data line) is the contended-lock / hot-read optimization.

Sharing a word is safe for exactly the reason the discipline below requires:
readers **mask to the generation field**, so a lock (bit 1) or tombstone (bit 2)
transition leaves the gen bits untouched and never triggers a spurious retry. This
is physical co-location, **not** the lock≡parity *semantic* conflation we rejected
— the bits stay separate and the lock stays invisible to readers. One wrinkle when
the shared word is the **MCAS-set** lock word: while a lock acquire is escalated (a
real proxy parked), a seqcount sample of that word must resolve the proxy to
recover the generation, so it is no longer a plain load. The common single-blob
acquire is a bare CAS (never parked), so this only bites under lock contention —
precisely when FT splits the seqcount off onto a plain data word.

Key discipline (a correction we locked in): **the lock is writer-only and must
never enter a reader's decision.** A reader's consistency comes from exactly two
reader-facing things — the **txn state** (per-word proxy resolution) and the
**seqcount** — and never from the lock. Conflating the lock with the seqcount
parity is a bug, because:

- a **lock release with no content change must not perturb readers** — so a reader
  masks to the *generation field only* and ignores lock/tombstone bit transitions;
- a writer may **hold the lock across several gen-bracketed sub-updates**, letting
  readers snapshot consistently *between* them while exclusion is still held;
- the lock may be held for **non-content reasons** where no gen bump is warranted.

The lock *enables* a cheap parity seqcount in the hybrid (by providing the
exclusion parity needs) but is **not** the parity.

## When is the seqcount actually needed?

**The seqcount buys exactly one capability: reading a value that spans more than
one transacted word as a single atomic unit.** The txn state already linearizes
each *individual* word, so anything whose reads are single-word resolves needs no
seqcount:

- **pointer-linked structures** (list, hlist, skiplist) — a traversal is a chain
  of single-word resolves, linked by the data itself; the reader never needs two
  slots from one instant. Seqcount would be dead weight.
- **a single-bit bitmap op, a ≤ 7-byte blob, a word-contained blob field** — one
  word, inherently atomic.

Only a **flat multi-word value read as a whole** (or a straddling field) needs it.
That is the blob's whole-struct read, and essentially nothing else in the family —
which is why the seqcount is opt-in, **off by default**. Off: writers skip the
bracket, readers get single-word-atomic field reads plus the tearing whole-struct
hint, and reads stay **wait-free**. On: writers bracket, `load_consistent` becomes
available — but that path is no longer wait-free (see the progress-class note), so
turning it on costs more than a hot word and a writer bracket.

## Three variants (writer model drives everything)

| variant | header | writer model | writer exclusion | torn-free multi-word read |
|---|---|---|---|---|
| **pure-sw** | `rcu-txn-sw-blob.h` | single writer | external (caller) | **parity** seqcount |
| **pure-mw** | `rcu-txn-blob.h` | lock-free concurrent | none (lock-free) | **validating-txn** (composes), or **enter/exit** seqcount (passive) |
| **hybrid** | `rcu-txn-blob-hybrid.h` *(name tentative)* | serialized by the lock | **MCAS lock bit** | **parity** seqcount |

Progress note: pure-mw has **lock-free writes** (bounded-blocking via helping);
pure-sw and hybrid have **blocking writes** (a stalled writer stalls others). That
is not a regression from the family — the sw engine already requires external
writer serialization; the hybrid merely makes that mandatory serialization
*fine-grained, composable and deadlock-free* instead of one global lock. Readers
stay lock-free in every variant.

### The hybrid: MCAS as a composable, deadlock-free multi-lock

The point of the hybrid is that using **MCAS to set the lock bit** gives what a
plain CAS cannot: **acquire N locks atomically and deadlock-free** (address-sorted
install, bail-at-first-conflict, no hold-and-wait). So sw's cheap writes become
usable under multiple writers — writers on disjoint lock-sets run in parallel;
only writers contending on the same blob serialize.

Write flow:

1. **Acquire** lock(s) via one MCAS commit (`ctrl: old -> old | LOCK`, checking
   `LOCK` and `TOMBSTONE` clear). A *single-blob* acquire is a one-record
   un-escalated commit — a **bare CAS, no proxy, no grace period**; proxies appear
   only under lock contention (escalation). **All locks needed must be acquired in
   ONE commit** — acquiring incrementally reintroduces hold-and-wait deadlock.
2. **Mutate** under the lock (sole writer) via an **sw txn** (flip), folding
   several locked structures into one sw flip commit; bump the parity generation
   odd→even around the content update.
3. **Release** — a **plain atomic store** clearing `LOCK` by the holder. Safe with
   no CAS: while `LOCK` is held, every acquirer's CAS fails, so the holder is the
   sole writer of `ctrl`.

Cross-*structure* atomicity comes from step 2's single sw flip (the lock is only
exclusion, invisible to readers); cross-structure *read* snapshots use a seqcount
spanning each involved blob's generation, or (pure-mw) the validating txn.

### Fractal trie: the converging use (DLM now, seqcount deferred)

The fractal trie is converging on this **hybrid / DLM** scheme (MCAS composable
multi-lock + sw content), and — deliberately — **without the seqcount yet**, for
two reasons: a generation word on the node's hot data cacheline is a footprint it
does not want to pay, **and** the seqcount would forfeit FT's **wait-free reads**
(a seqcount snapshot is only obstruction-free — see the progress-class note). Lock
and tombstone go in the node's cold metadata area; the data cachelines stay lean.

The consequence is an accepted price: **recompaction.** Without a seqcount on the
hot line, readers cannot get a cheap consistent `{occupancy-bitmap,
popcount-compressed-array}` snapshot, so a **rank-changing** update (one that sets
or clears an occupancy bit and thus shifts every higher entry's rank) is done by
**COW recompaction** — build a new node cluster, publish with one store, reclaim
the old — inherently torn-free via the single publish, and — crucially — keeping
reads **wait-free** (one pointer resolve yields a whole consistent node), which a
seqcount would give up. The price is allocate + copy per rank change. What keeps this affordable is the split:
**rank-preserving** in-place updates (an existing entry's value, ≤ one word) stay
seqcount-free single-word stores, so only rank *changes* recompact. The seqcount
would only ever be needed for a multi-word *in-place* snapshot, which this split
sidesteps. Placement flexibility keeps the door open: if recompaction cost ever
dominates, add the seqcount on the data line later, without disturbing the
lock/tombstone in metadata.

### Tombstone (just a live/dead flag)

A fixed-slot blob that cannot be freed (pinned) needs a logical "dead": readers
honor `TOMBSTONE` as absent, and acquire refuses a tombstoned blob (so you cannot
write a corpse). Because `ctrl` is an MCAS slot in the hybrid, tombstone-set folds
into a commit that also **unlinks the blob from a list** — atomic logical removal.
No reuse protocol in scope: the caller handles recycling (a tombstone → grace
period → reinit is left to the caller for now).

**Readers do not validate the tombstone.** It is a single bit, read atomically as
a one-shot liveness gate — not folded into the seqcount window or a validating
read-set. RCU makes both linearizations valid: a reader reading `live` then the
data legitimately precedes a concurrent removal; one reading the data then `dead`
may treat the entry as absent. The only obligations are the usual ones — read it
inside the RCU section — plus that *revival* (dead → live reuse) be
grace-period-gated so the bit cannot flip under a reader within one section (which
the deferred reuse lifecycle already requires). An application wanting a *jointly*
consistent {liveness, data} snapshot may still fold the tombstone word into its
read-set / seqcount, but that is opt-in, not required.

## Consistency model (summary)

**Write atomicity and read atomicity are separate problems.**

- **Writes** switch the whole struct at one linearization point (mw status CAS, or
  sw selector flip), and compose via the `_prepare` forms.
- **Reads are not atomic automatically.** A per-word `load_rcu` resolves words at
  different times and can tear across a commit. Torn-free needs either:
  - a **validating read-only transaction** (pure-mw only) — `load_validate` each
    word into a read-set linearizing on its own status CAS (the bitmap T4
    pattern). The **only** path that composes a snapshot across *other* txn
    structures. Cost: each validated word plants a proxy, so snapshot-readers
    contend (reader-reader aborts) and perturb writers; does not scale; and it is
    **not wait-free** (abort + retry).
  - a **seqcount** — passive pure-load reads bracketed by the generation; no proxy
    planting, no reader-reader aborts, never perturbs writers. Cost: snapshots the
    blob **alone** (does not compose), requires the writer to bracket (rule 3),
    and — the significant one — it is **not wait-free** (see below).
- **A ≤ 7-byte blob is inherently torn-free** — one resolve to read, one record to
  write, no seqcount or validating txn, and reads stay wait-free.

**Progress class — the significant catch.** The family's baseline reads are
**wait-free**: a single-word resolve, `load_rcu`, a word-contained field read, and
a pointer-linked traversal all complete in a bounded number of their own steps
with no retry — RCU's headline guarantee. **Both torn-free multi-word reads give
that up.** The seqcount reader is only **obstruction-free** — it retries until it
catches a quiescent window, so a sustained stream of writers can **starve it
indefinitely**; the validating-txn reader likewise aborts and retries (and
contends). For a flat multi-word value you can have wait-free *or* torn-free, not
both — the wait-free way to snapshot many words at once is to make them reachable
through a **single pointer (COW)**, which is exactly what FT's recompaction buys. A
read path that must be wait-free (RT, signal/constrained context) therefore cannot
use `load_consistent`; it stays on the single-word / `load_rcu` reads and is
designed not to need a multi-word snapshot.

Seqcount flavor by writer model: **parity** where writers are serialized (pure-sw,
hybrid); **enter/exit** in-flight-writer count where they are concurrent
(pure-mw), the reader accepting a window where `enter == exit` throughout (a
fully-settled generation). Parity is the single-writer collapse of enter/exit, so
one code path serves both.

## Rules that fall out (document prominently)

1. **`store` may declare its write set disjoint** — the *opposite* of the bitmap
   rule. A store writes each word once from the source copy, so the words are
   pairwise-distinct slots; `urcu_txn_declare_disjoint()` is correct and takes the
   fast install path. Composing a blob store with a list/bitmap edit still touches
   distinct slots. Caveat: do **not** store the same blob twice, or store
   overlapping dirty-ranges, on a disjoint handle in one txn — use the default
   (read-your-own-writes) handle then.

2. **Single-word (≤ 7B) blobs, and word-contained fields, are torn-free for free**
   on both read and write; no validating txn or seqcount needed.

3. **A seqcount read requires the writer to bracket its commit.** Only the
   self-contained `store_rcu` (and the hybrid critical section) brackets. A
   *composed* `store_prepare` cannot (it appends edges to the caller's txn and does
   not own the install/settle lifecycle), so a blob updated through composition is
   not seqcount-readable. Mixing seqcount reads with unbracketed composed writers
   on one blob is unsafe and the header forbids it. For torn-free reads of composed
   updates, use pure-mw's validating-txn path.

4. **The lock never enters a read.** Readers mask to the generation field and
   ignore lock/tombstone transitions; a lock acquire/release must not force a
   snapshot retry. (See the control-word section.)

5. **Hybrid: acquire all locks in one MCAS commit** — incremental acquisition
   deadlocks.

## API surface

Shared (all variants) — encoding + single-word access:

| purpose | function |
|---|---|
| size a `words[]` | `URCU_TXN_BLOB_NR_WORDS(nbytes)` |
| resolve one word (RCU rd) | `urcu_txn[_sw]_blob_word_rcu(blob, w)` |
| single-field read (word-contained → atomic) | `urcu_txn[_sw]_blob_load_field_rcu(blob, dst, off, len)` |
| cheap whole read, may tear | `urcu_txn[_sw]_blob_load_rcu(blob, dst, nbytes)` |

pure-mw — `rcu-txn-blob.h` (`urcu_txn_blob_*`):

| purpose | function |
|---|---|
| torn-free snapshot, composing | `..._load_validate_prepare(txn, blob, dst, nbytes)` |
| torn-free snapshot, passive (opt-in seqcount) | `..._load_consistent(blob, dst, nbytes)` |
| composable write | `..._store_prepare(txn, blob, src, nbytes)` (disjoint-OK) |
| self-contained write | `..._store_rcu(domain, blob, src, nbytes)` (brackets seqcount if enabled) |

pure-sw — `rcu-txn-sw-blob.h` (`urcu_txn_sw_blob_*`):

| purpose | function |
|---|---|
| torn-free snapshot (opt-in seqcount) | `..._load_consistent(blob, dst, nbytes)` |
| composable write | `..._store_prepare(txn, blob, src, nbytes)` (flip; not bracketed) |
| self-contained write | `..._store_rcu(blob, src, nbytes)` (brackets seqcount if enabled) |

hybrid — `rcu-txn-blob-hybrid.h` (`urcu_txn_blob_hy_*`, name tentative):

| purpose | function |
|---|---|
| acquire lock(s), one commit | `..._lock_prepare(txn, blob)` + commit; or `..._trylock(blob)` single-blob bare CAS |
| release lock | `..._unlock(blob)` (plain store) |
| write under lock | sw `..._store_prepare(txn, blob, src, nbytes)` folded into the sw flip |
| torn-free snapshot | `..._load_consistent(blob, dst, nbytes)` (parity seqcount) |

Optional control-word ops (where the layer is enabled), all variants:

| purpose | function |
|---|---|
| set / test tombstone | `..._tombstone_prepare(txn, blob)` / `..._is_tombstoned_rcu(blob)` |

### Store internals

`store_prepare` (mw): for each word `w`, `old = urcu_txn_load(...)`,
`new = encode(src, w)` (depends only on `src`), `urcu_txn_store(txn, &words[w],
old, new, URCU_MCAS_TAG)`. `new` is a blind overwrite; `old` is needed only for
the commit's value-CAS, and a mid-flight change aborts and retries via the
caller's loop. sw uses `urcu_txn_sw_load` / `urcu_txn_sw_record`.

`store_rcu` brackets the seqcount (when enabled): parity generation odd before the
commit, even after settle, so `load_consistent` observes only settled generations.
The composable `store_prepare` does not bracket — hence rule 3. `load_consistent`
samples the generation, resolves all words, `smp_rmb`, re-samples, and retries
while the *generation field* differs (masking lock/tombstone).

## Implementation notes

- Headers are inline-only (like the bitmap pair); no new `.c`. Include an RCU
  flavor header before use (commit reclaims proxies via `call_rcu`).
- Shared encoding + control-word helpers factor into a common include
  (`rcu-txn-blob-common.h` or similar); the three front-ends build on it.
- `words[]` (and `ctrl`) must be naturally `uintptr_t`-aligned; the struct
  guarantees it. A bare-`words[]` blob is caller-aligned.
- Read accessors run inside an RCU read-side critical section of the flavor the
  transactions use — same rule as the bitmap.
- `word_rcu`: mw uses `urcu_mcas_read_optimistic` (a pure reader never helps); sw
  does an acquire load + `urcu_txn_sw_proxy_get` on a tagged word.

## Testing (mirror `tests/unit/test_rcu_txn_bitmap.c`)

- **T1** encode/decode round-trip across sizes 1, 7, 8, 63, 64 bytes (word
  boundaries), non-multiple-of-7 tails, all-zero blob; opaque vs field-aligned.
- **T2** self-contained store then `load_rcu`; ≤7-byte single-word atomicity;
  word-contained `load_field_rcu` needs no seqcount.
- **T3** compose a blob store with a bitmap flip / list splice in one commit;
  assert both visible together (no partial).
- **T4** torn-free snapshot under a concurrent writer, every read path: seqcount
  `load_consistent` (sw + hybrid parity, pure-mw enter/exit) and pure-mw
  `load_validate_prepare` (retry on ABORT); each observes only whole generations.
  Composing case: snapshot blob + another structure via `load_validate_prepare`.
  Negative controls: `load_rcu` CAN tear; a seqcount read of a *composed*
  (unbracketed) update is not protected (rule 3).
- **T5** disjoint-declared store (fast path) vs default handle.
- **T6** hybrid: multi-blob lock acquire is atomic and deadlock-free (two threads
  acquiring {A,B} and {B,A} both make progress); single-blob `trylock` is a bare
  CAS; release is a plain store; a reader never observes the lock bit (masking).
  Tombstone: reader sees a tombstoned blob as absent; acquire refuses it.
- Build against the local tree (`-L` to the in-tree lib, per CLAUDE.md); a
  separate `-O2 -DNDEBUG` benchmark: blob store cost vs COW by size; hybrid write
  (lock + sw flip) vs pure-mw write; `load_consistent` reader scaling.

## Build order

1. **Encoding + bare `words[]` core**, plus pure-mw `rcu-txn-blob.h` (fully
   engine-native: writes and torn-free reads both compose). Validates the encoding
   and the store / load-validate shape end to end.
2. **pure-sw `rcu-txn-sw-blob.h`** — reuse the encoding, add the flip
   `store_prepare` and the parity-seqcount `store_rcu` / `load_consistent`.
3. **Optional layers** — the control word: tombstone, then the seqcount opt-in
   wired through both variants above.
4. **hybrid `rcu-txn-blob-hybrid.h`** — MCAS lock (acquire/trylock/release) +
   sw-under-lock content + parity seqcount; the deadlock-free multi-lock property.
5. Wire into the build (`include/Makefile.am` nobase_include_HEADERS), add tests.

## Future work

- **Dirty-range store** (`store_range_prepare`): record only the words overlapping
  changed bytes, shrinking the commit and conflict footprint for single-field
  updates. Start with full-copy for clarity.
- **Tombstone reuse lifecycle**: helpers for tombstone → grace → reinit → revive
  of a pinned slot (currently caller's responsibility).
- **Cross-process variant**: offsets + versioned region, out of scope here.
