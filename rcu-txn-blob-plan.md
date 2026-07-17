# Plan: `rcu-txn-blob.h` / `rcu-txn-sw-blob.h` — atomically-mutated in-place binary blobs

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

## Encoding (identical across both variants)

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
  word is an independent, self-contained record and no logical field's bytes ever
  straddle two words. (A bit-granular 63-data-bit split — the bitmap's scheme —
  would put a field's bytes in two records and needs cross-word shifting; here we
  spend 8 bits, not 1, to buy clean byte lanes and a plain `>> 8` payload.)
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

### Shared layout with a version word

```c
struct urcu_txn_blob {
    unsigned long version;   /* seqlock: sw standalone reads use it; concurrent ignores it */
    uintptr_t     words[];   /* URCU_TXN_BLOB_NR_WORDS(nbytes) transacted words */
};
```

Both variants use the `version` word as an **enter/exit seqcount** (see the
consistency model): the sw case is the single-writer collapse of the same scheme,
so the pair keeps the bitmap's layout+mechanism parity — one `words[]` (with its
header) migrates mechanically between engines. The concurrent variant *also*
offers a composing read path (validating txn) that ignores `version`; the sw
variant has the seqcount read only.

## Consistency model

**Write atomicity and read atomicity are separate problems.** This is the central
point and must be loud in the header intro.

- **Writes are atomic and composable.** A store records `ceil(N/7)` edges; the
  whole struct switches at the single status-word CAS (concurrent) or selector
  flip (sw). Folded into a larger commit via the `_prepare` forms, the blob's
  fields become visible together with the other folded edits — one linearization
  point across all of them.

- **Reads are NOT atomic automatically.** A per-word `load_rcu` resolves word 0,
  then word 3, at two different times; a commit landing in between yields new
  bytes in word 0 and old in word 3 — **torn** — even though each word resolved
  correctly. The tear is the reader straddling the writer's linearization point;
  per-word proxy resolution cannot fix reads taken at two times. Two mechanisms
  give a torn-free read, with different strengths, and **both variants can offer
  both** (see the seqcount note below):
  - a **validating read-only transaction** (concurrent engine only) — `load_validate`
    every word into a read-set that linearizes on its own status CAS (the bitmap
    T4 pattern). This is the ONLY mechanism that **composes**: it folds the blob
    snapshot into a read-set spanning *other* txn structures, so the blob and a
    list and a bitmap snapshot atomically together. Cost: each validated word
    *plants a proxy* (`rcu-mcas.h` install is a try-CAS), so two concurrent
    snapshot-readers of one blob collide and one aborts — reads cost ~like writes
    and do not scale, and readers perturb writers.
  - a **seqcount** on `version` — reader decodes optimistically between two counter
    samples and retries if a writer's window overlapped. Reads are **passive pure
    loads**: no proxy planting, no reader-reader aborts, readers never perturb
    writers. Cost: the reader may spin until it catches a quiescent window (under
    sustained writes), writers pay counter contention, and it snapshots the blob
    **alone — it does not compose** across structures.

  #### The seqcount works for BOTH engines (corrected)

  A *parity* (odd/even single-counter) seqcount needs writer mutual exclusion —
  hence it fits the single-writer engine directly. But an **enter/exit
  in-flight-writer count** needs no mutual exclusion and works with any number of
  concurrent writers, so the **concurrent (mw) variant can use a seqcount too**:

  ```
  writer:   atomic_inc(&enter, RELEASE)   /* before mutations become visible */
            ... install / status-CAS / settle ...
            atomic_inc(&exit,  RELEASE)   /* after every word is settled */
  reader:   do { e = load(exit, ACQUIRE);
                 <resolve all words into dst>; smp_rmb();
                 n = load(enter, ACQUIRE);
            } while (n != e);             /* no writer in flight during the read */
  ```

  A window where `enter == exit` throughout is exactly a fully-settled single
  generation (between install and settle every word is a proxy). The sw parity
  seqcount is just the degenerate single-writer case of this, so **both variants
  share the `version` field and the same read/write bracketing** — restoring the
  bitmap pair's layout+mechanism parity. Tempering the mw "many writers" concern:
  a full-struct store writes every word, so two concurrent stores to one blob
  already collide at the MCAS install and serialize (one wins, the other retries);
  truly-parallel writers to one blob only arise with dirty-range stores on
  disjoint words.

  So the mw variant offers **both read paths**: a `load_consistent` seqcount read
  (fast, passive, non-composing) for "read this one struct atomically", and
  `load_validate_prepare` for "snapshot this blob together with other structures".
  The sw variant offers the seqcount read only (its engine has no read set).

- **A ≤ 7-byte blob is one word, hence inherently torn-free** with no txn and no
  seqlock — a single atomic resolve for the read, a single record for the write.
  This is the sweet spot and gets a first-class fast path.

## Three rules that fall out (document prominently)

1. **`store` may declare its write set disjoint** — the *opposite* of the bitmap
   rule. A store writes each word exactly once from the source copy, so the words
   are pairwise-distinct slots; `urcu_txn_declare_disjoint()` is correct and takes
   the fast install path. A blob's words are its private storage, so composing a
   blob store with a list/bitmap edit still touches distinct slots — disjoint
   holds even when composed. Caveat: do **not** store the same blob twice, or
   store overlapping dirty-ranges, on a disjoint handle in one txn — use the
   default (read-your-own-writes) handle then.

2. **Single-word (≤ 7B) blobs are torn-free for free** on both read and write; no
   validating txn or seqlock needed.

3. **A seqcount read requires the writer to bracket its commit** with the
   enter/exit increments — which only the self-contained `store_rcu` does. A
   *composed* `store_prepare` cannot bracket (it appends edges to the caller's txn
   and does not control the caller's install/settle lifecycle), so a blob updated
   through composition is not seqcount-readable. Seqcount reads are therefore valid
   only for blobs whose writers all go through the bracketing `store_rcu` (or a
   caller who manually brackets enter/exit around its own composed commit); mixing
   seqcount reads with unbracketed composed writers on one blob is unsafe and the
   header forbids it. For torn-free reads of *composed* updates, use the concurrent
   variant's validating-txn path — the sw engine has none, so this mirrors
   `rcu-txn-sw-bitmap.h`'s "use the concurrent header for snapshots" guidance.

## API surface (bitmap-parallel)

Concurrent — `include/urcu/rcu-txn-blob.h` (`urcu_txn_blob_*`):

| purpose | function |
|---|---|
| resolve one word (RCU rd) | `urcu_txn_blob_word_rcu(blob, w)` |
| cheap read, may tear | `urcu_txn_blob_load_rcu(blob, dst, nbytes)` |
| torn-free snapshot, passive | `urcu_txn_blob_load_consistent(blob, dst, nbytes)` (seqcount) |
| torn-free snapshot, composing | `urcu_txn_blob_load_validate_prepare(txn, blob, dst, nbytes)` (validating txn) |
| composable write | `urcu_txn_blob_store_prepare(txn, blob, src, nbytes)` (disjoint-OK) |
| self-contained write | `urcu_txn_blob_store_rcu(domain, blob, src, nbytes)` (brackets seqcount) |
| later: partial update | `urcu_txn_blob_store_range_prepare(txn, blob, src, off, len)` |

Single-writer — `include/urcu/rcu-txn-sw-blob.h` (`urcu_txn_sw_blob_*`):

| purpose | function |
|---|---|
| resolve one word (RCU rd) | `urcu_txn_sw_blob_word_rcu(blob, w)` (acquire + `proxy_get`) |
| cheap read, may tear | `urcu_txn_sw_blob_load_rcu(blob, dst, nbytes)` |
| torn-free snapshot (bracketed writers only) | `urcu_txn_sw_blob_load_consistent(blob, dst, nbytes)` (seqcount) |
| composable write | `urcu_txn_sw_blob_store_prepare(txn, blob, src, nbytes)` (flip; not bracketed) |
| self-contained write | `urcu_txn_sw_blob_store_rcu(blob, src, nbytes)` (brackets seqcount) |
| later: partial update | `urcu_txn_sw_blob_store_range_prepare(txn, blob, src, off, len)` |

Asymmetry to flag in both headers: only the concurrent `load_consistent` /
`load_validate_prepare` composes into a larger read-set (snapshot blob + other
structures atomically) — only the concurrent engine has a read set. The sw seqlock
protects the blob alone.

### Store internals

`store_prepare` (concurrent): for each word `w`, `old = urcu_txn_load(...)`,
`new = encode(src, w)` (depends only on `src`), `urcu_txn_store(txn, &words[w],
old, new, URCU_MCAS_TAG)`. `new` is a blind overwrite; `old` is needed only for
the commit's value-CAS, and a mid-flight change to the word aborts and retries via
the caller's loop. sw uses `urcu_txn_sw_load` / `urcu_txn_sw_record`.

`store_rcu` (both variants): the self-contained form brackets its commit with the
enter/exit seqcount — `enter++` (RELEASE) before install, commit, settle,
`exit++` (RELEASE) after — so a `load_consistent` reader observes only
fully-settled generations. The composable `store_prepare` does **not** bracket (it
appends edges to the caller's txn and does not own the install/settle lifecycle) —
hence rule 3. For a single writer, `enter`/`exit` can share one word as the parity
low bit (the classic odd/even seqcount); the enter/exit form is the general case
that also covers concurrent writers, and keeping it identical across both variants
is what preserves layout parity. `load_consistent` is the reader loop from the
consistency model (sample `exit`, resolve all words, `smp_rmb`, sample `enter`,
retry while they differ).

## Implementation notes

- Headers are inline-only (like the bitmap pair); no new `.c`. Include an RCU
  flavor header before use (commit reclaims proxies via `call_rcu`).
- `words[]` must be naturally `uintptr_t`-aligned; the `struct urcu_txn_blob`
  header guarantees it.
- Read accessors must run inside an RCU read-side critical section of the flavor
  the transactions use (they resolve descriptors whose lifetime is the grace
  period) — same rule as the bitmap.
- `word_rcu` mirrors the bitmap's: concurrent uses `urcu_mcas_read_optimistic`
  (a pure reader never helps); sw does an acquire load + `urcu_txn_sw_proxy_get`
  on a tagged word.

## Testing (mirror `tests/unit/test_rcu_txn_bitmap.c`)

- **T1** encode/decode round-trip across sizes 1, 7, 8, 63, 64 bytes (word
  boundaries), incl. non-multiple-of-7 tails and the all-zero blob.
- **T2** single self-contained store then `load_rcu` (both variants); ≤7-byte
  single-word atomicity.
- **T3** compose a blob store with a bitmap flip / list splice in one commit;
  assert both visible together (no partial).
- **T4** torn-free snapshot under a concurrent writer, exercising every read path:
  seqcount `load_consistent` (both variants), and the concurrent variant's
  validating-txn `load_validate_prepare` (retrying on ABORT). Each must observe
  only whole generations — never a mixed old/new struct. Also cover the composing
  case: snapshot the blob together with another txn structure via
  `load_validate_prepare` and assert cross-structure consistency. This is the
  property the plain `load_rcu` deliberately does not provide; add a stress test
  showing `load_rcu` CAN tear (documents the contract), and one showing a seqcount
  read of a *composed* (unbracketed) update is not protected (rule 3).
- **T5** disjoint-declared store correctness (fast path) vs default handle.
- Build against the local tree (`-L` to the in-tree lib, per CLAUDE.md), and a
  separate `-O2 -DNDEBUG` benchmark run: blob store cost vs COW as a function of
  size, and `load_consistent` reader scaling (concurrent contention vs sw seqlock).

## Build order

1. **`rcu-txn-blob.h` (concurrent) first** — fully engine-native: writes and
   torn-free reads both compose, no seqlock subtlety. Validates the encoding and
   the store/load-validate shape end to end.
2. **`rcu-txn-sw-blob.h` (sw twin)** — reuse the encoding verbatim, add the flip
   `store_prepare`, the seqlock `store_rcu` / `load_consistent`, and the
   standalone-only guidance.
3. Wire into the build (`include/Makefile.am` nobase_include_HEADERS), add tests.

## Future work

- **Dirty-range store** (`store_range_prepare`): record only the words overlapping
  changed bytes, shrinking the commit and the conflict footprint for a
  single-field update. Start with full-copy for clarity.
- **Cross-process variant**: offsets + versioned region, out of scope here.
