# The stale disposal predicate — design brief (2026-09-04)

Status: **BRIEF. No design chosen, no implementation proposed.**
Measured at `c6c06d12` on branch `ft/unpub-free-audit`.
Decides: the **shipping two-writer key loss** — two concurrent
`cds_ft_insert` / `cds_ft_remove` writers on one trie, which the header grants
for `CDS_FT_WRITER_LOCK_FINE` — and it **removes G4 from the critical path**
for it.

☠ **It does NOT close the plan's open box.**
`mw-to-fine-locking-remainder.md:2718` asks whether the holder's lock excludes
every writer of a chain member's **`->prev`**.  That box stays open: the word
that loses the key here is the holder's `external_nodes` or one of its body
slots, not `->prev`.  What this brief does contribute to it is the *shape* of
the answer — for these words the lock **is** held by every writer and the
acquires **are** ordered by it, and the loss happens anyway, so "an owner is
available and the op holds it" is measured to be an insufficient exclusion
argument on its own.

---

## 0. The one-paragraph statement

Two concurrent `cds_ft_insert` / `cds_ft_remove` writers lose keys, in contract,
on the default `CDS_FT_WRITER_LOCK_FINE`.  It is **not** a missing lock and
**not** the external-head back channel.  Both writers take the holder's node
lock and they serialize correctly — the second acquires only after the first's
commit has released, measured at 110 ns in one instance and 4.0 ms in the
other.  The defect is that a **disposal predicate** — "this node has one child and no
external head", "this ancestor is an orphan" — is evaluated from reads taken at
**plan** time and is never converted into a **read-set entry** of the commit
that disposes of the node.  The peer's write lands on a *different word of the
same node*; the disposal CASes the node's **parent slot** and its **state
word**, neither of which the peer moved; the MCAS therefore has nothing to
arbitrate, ratifies a stale predicate, and the peer's key is freed away inside
the disposed node.

> **owner-AVAILABLE ✓ · owner-HELD ✓ · owner-SUFFICIENT ✗**

---

## 1. Why the lock does not cover it — the asymmetry

A node's `nr_child` lives **inside the state word** (`ft_state_nr_child`), which
is the same word the lock bit lives in.  So the acquire's snapshot
(`ft_held_anchor_sample_node` → `iter_held.node_snap`) carries `nr_child` **for
free**, and `ft_chain_compress_fused` re-validates it under the fence
(`ft-remove.h:1314`).

`external_nodes` is a **separate word** (`struct cds_ft_metadata`: `parent_word`
@0, `external_nodes` @+0x8, … `state` @+0x20).  Nothing carries it:

* it is not in the lock word, so the acquire snapshot does not cover it;
* it is not in the disposal's write set, so the expected-old does not cover it;
* it is not re-read under the fence, so `ft-remove.h:1314` does not cover it.

The same holds for a **body slot** under a *same-slot value swap*: replacing a
bare external head with a junction node in one slot moves neither `nr_child` nor
the parent slot.  `ft-remove.h:1314` re-reads exactly **one** body slot (the
survivor's, `ft_node_get_nth(... surviving_byte ...) != surviving_child`); every
other slot of the boundary is uncovered.

☞ The tree already books the first half of this, at
`ft-mutation-helpers.h:6784`:

> "@external_nodes is checked but NOT covered by the fence: **no retire
> validates that word (a known, separate hole** — ft_insert_park_external_nodes
> publishes it with no lock)."

☠ Its parenthetical is **stale**: the park *does* take the holder's lock
(`ft_flip_txn_lock_or_guard_parent`, `ft-insert.h:927`, recorded at
`ft-insert.h:929`), and the trace shows both the acquire and the release
terminal.  The main clause is exactly right.

---

## 2. The evidence (LTTng flight recorder, `c6c06d12`)

Harness `stab5.c` (`stab2.c` + `NOFREE=1` + the violation hook), `CHK=1
WRITERS=2 ALPHA=2 MAXLEN=8 NSTABLE=100 NCHURN=100 SECS=5 READERS=4`,
`taskset -c 0-1`, 8 G memcg, 1 MiB × 8 overwrite ring.  Reproduces ~4/4 in 5 s.
Control `build-pfxbit`: **8/8 red** (5× `RM-LOOKUP-MISS`, `RM-FAIL`,
`STALE-FOUND`, 1 SEGV).

### Instance A — `ft_chain_compress_fused`, the boundary free at `ft-remove.h:1795`

Holder metadata `0x7F8DB0BDB318` (`external_nodes` @`…320`, `state` @`…338`):

```
writer0  .033244764  ACQUIRE   &meta->state           0x8 -> 0x80008
         .033245044  RELEASE   &meta->state           0x80008 -> 0x8   (terminal)
         .033245164  PARK      &meta->external_nodes  0x0 -> the key's node
         .033245685  txn_commit status = 0                    INSTALLED
writer1  .037260997  ACQUIRE   &meta->state           0x8 -> 0x80008   (+4.015 ms)
         .037263821  TOMBSTONE &meta->state           0x80008 -> 0xA
         .037264262  UNLINK    parent slot: holder -> its promoted child
         .037265894  item_retire  <-- ft_chain_compress_fused, ft-remove.h:1795
                     ☠ no record anywhere on &meta->external_nodes
 .045476069  ext_violation kind=0  hext == the node, hstate == 0xA (TOMBSTONE)
```

The key's node is left as the `external_nodes` of a **tombstoned** holder that
still names it — reachable from the node, unreachable from the root.

### Instance B — `ft_detach_node`'s deferred orphan free at `ft-remove.h:4239`

```
B  .996033923  publish junction 0x7FE416C12621 (holds the key's node)
               into node 0x7FE416C12600's BODY slot 0x7FE416C12608
   .996035084  txn_commit status = 0                          INSTALLED
A  .996035194  ACQUIRE 0x7FE416C12600's state        (+110 ns after B's commit)
   .996036747  UNLINK  parent slot 0x7FE416C004B0
                       old = 0x7FE416C12601 -> 0x55F5B1BAA770
   .996037488  txn_commit status = 0
   .996037638  item_retire 0x7FE416C12600  <-- ft_detach_node, ft-remove.h:4239
```

A's expected-old is the node's **parent slot**, which B did not touch.  B's
publish is a **same-slot value swap**, so it moved neither `nr_child` nor the
state word.

---

## 3. The affected sites

### 3.1 `ft_chain_compress_fused` — the boundary's `external_nodes`

The header (`ft-remove.h:599`) lists `@iter_meta->external_nodes == NULL` as a
**precondition the caller asserts**.  Four callers, and they do **not** agree on
what the word should be:

| caller | plan assumes `external_nodes` is | gate |
|---|---|---|
| `ft-remove.h:1827` `ft_canonicalize_chain_compress` | **NULL** | caller gate at `ft-remove.h:4206` |
| `ft-remove.h:3356` shape-D detach fold (`plan_nr_child == 2`) | **NULL** | caller gate at `ft-remove.h:3239` |
| `ft-remove.h:5367` | **the head this op removes** (non-NULL) | *"sole body child; the removed entry is external"* |
| `ft-remove.h:6176` | **the chain head this op removes** (non-NULL) | same |

### 3.2 `ft_detach_node`'s upward prune — the orphan set

The climb reads `metadata->external_nodes` three times to decide the orphan set
and where to stop: `ft-remove.h:2332` (collect), `:2352` (stop the climb),
`:2460` (promote the chain).  The orphans are then locked into `orphan_held[]`,
the commit runs, and `ft-remove.h:4234`–`:4239` frees them.  The uncovered word
here is a **body slot** as well as `external_nodes`.

### 3.3 Already correct — the model to copy

`ft_node_recompact`'s DEL arm carries `@nullify_expected`
(`ft-mutation-node.h:1707`): it re-reads the slot
its plan named and bails `-EAGAIN` when the two differ, with a comment that
states the class exactly (*"A peer that republished this slot between the two …
makes the two differ, and copying 'every child except this slot' then drops the
peer's subtree whole"*).  That is the shape a fix has to generalise.

---

## 4. What has already been refuted (do not re-try these)

1. **The remove's stale `node->prev` holder derivation.**  Forcing the
   coarse-spacing forward-holder cross-check under `PER_NODE`
   (`ft-remove.h:5103`, `-DFT_DEBUG_FWD_HOLDER_ALWAYS`): **4/4 red both sides.**
2. **A blanket `external_nodes != NULL` test in the `ft-remove.h:1314`
   re-validation.**  **8/8 `rc=137`** — the memcg SIGKILLing a livelock.  Two of
   the four callers (§3.1 rows 3 and 4) legitimately hold a non-NULL head, so a
   NULL test is a *permanent* `-EAGAIN` and the retry lane spins.  ★ A 137 is a
   result, not an obstacle.
3. **The same test gated to the no-external-removal shape** (`!freeze_leaf &&
   !dead_cell` — the callers that are not themselves removing an external
   entry): no livelock, **still 8/8 red**.

★ (2) and (3) together say the cure is **not** a NULL test at one site.  It has
to be an **expected-old** — the value the *caller's* plan assumed — and it has
to cover §3.2 as well, where the word is a body slot.

---

## 5. The questions for Mathieu

**Q1 — Where does the predicate become a read-set entry?**  Three shapes, and
they are not equivalent:

* **(a) re-read under the fence** — extend `ft-remove.h:1314` with the plan's
  expected value, threaded from each caller.  Cheapest; correct only while the
  fence really is exclusive against *every* writer of that word, which is the
  claim this whole brief says was never proved.
* **(b) a genuine `urcu_txn_validate` on the disposing commit** — the word joins
  the read set and the engine arbitrates it, which is what `@nullify_expected`
  and the §4.B guards already do elsewhere.  Costs a reservation slot per
  validated word and needs the SW/MW kind dispatch (`ft_flip_txn_record_tag`,
  not raw `urcu_txn_validate`, or the head's SW park and this MW guard conflict
  on one slot).
* **(c) fold the predicate into the lock word** — give `external_nodes`
  emptiness a bit in `state`, so the acquire snapshot covers it the way
  `nr_child` already is.  Cheapest at commit, but it is a layout change and a
  new invariant to maintain at every head park/clear.

**Q1b — and what does a FAILED validation DO?**  This is the harder half, and
§6 shows why: **skipping the collapse is refuted** — the canonical form is
enforced by `cds_ft_verify` (`ft-verify.h:1078`) and by an `abort()` at the
writer-scope exit — while plain `-EAGAIN` is what livelocked in refutation (2),
because the peer's head is *permanently* there.  That leaves two: **re-plan to
the shape the trie now has** (the boundary keeps a head, so it is no longer a
collapse candidate — does the caller have a legal non-collapsing path from
there?), or **refuse the whole op terminally**, the way the fold arm's `-EDOM`
already does (`ft-remove.h:3292`).  A terminal refusal from a *remove* is a
contract question of its own.

**Q2 — What is the expected value at each of the four `ft_chain_compress_fused`
call sites?**  §3.1 says two assume NULL and two assume "the head I am
removing".  Is that a *parameter* the caller passes, or is it derivable inside
the callee from the arguments it already has (`freeze_leaf`, `dead_cell`)?  The
second is tempting and (3) above shows it is **not sufficient on its own**.

**Q3 — Is §3.2 the same fix or a different one?**  The orphan set's uncovered
word is a body slot, and the climb collects up to `FT_MAX_DEPTH` of them.
Validating every body slot of every orphan is a different cost class from
validating one word per boundary.  Is there a cheaper statement — e.g. the
orphan's `nr_child` snapshot plus "no slot was value-swapped", which needs a
per-node version, not a per-slot compare?

**Q4 — Does this retire G4?**  G4 asks whether ordered cells and dup-chain
splices keep a narrow MW lane or grow a state word, and the plan defers it to a
Phase C measurement.  This defect is **not** in that lane: the contended word is
a holder's `external_nodes` or a body slot, both of which already have an owner
and a lock.  So G4 need not be decided to fix the shipping key loss — but the
answer to **Q1(c)** is adjacent to it and the two should not be decided in
opposite directions.

**Q5 — Scope.**  Is the deliverable the two measured instances, or a sweep for
the class?  The class is: *"an op that DISPOSES of a node on the strength of a
predicate over words it does not carry into its commit"*.  A sweep would start
at every `item_retire` / `free_*_node` reachable from a mutator and ask, for
each, which words its decision read.

---

## 6. What the fix must not break

* **Liveness — and ☠ SKIPPING IS NOT AN OPTION.**  Refutation (2) is the
  warning: a predicate that can be *permanently* false turns a re-validation
  into a livelock, because the retry lane re-plans the same disposal.  The
  obvious escape — "just skip the compress, it is only a canonicalisation" — is
  **REFUTED** (adversarial review, 2026-09-04, all four call sites).  The
  canonical form is **ENFORCED, not preferred**: `cds_ft_verify` returns -1 on
  exactly the shape a skip leaves — a non-root 1-child internal with no
  `external_nodes`, in skip mode (`ft-verify.h:1078`) — and
  `ft_writer_scope_verify` `abort()`s on it under
  `-DFEATURE_FT_VERIFY_AT_MUTATION` (`ft-verify.h:1442`).  Four more sites state
  the same invariant: the fold arm returns `-EDOM` rather than publish one
  (`ft-remove.h:3292`), `ft_compress_single_child_if_needed` says the caller
  "must NOT publish @child non-canonically -- it must fail the mutation"
  (`ft-cluster-build.h:261`), and `ft-graft.h:580` / `ft-rekey.h:1541` restate
  it.  The one residue the tree does tolerate is labelled a **defect**, not a
  licence: *"keeps its 1-child internal past the op, which skip mode's verifier
  flags at the writer-scope exit.  KNOWN INCOMPLETE"* (`ft-remove.h:4200`).

  ★ So a validated collapse must **re-plan to a different shape**, or **refuse
  terminally** the way the fold arm's `-EDOM` does.  Never skip.  Solving the
  livelock is therefore part of Q1, not a fallback under it.

* **☠ The fold arm is stricter still.**  On `record_only` (`ft-remove.h:3356`)
  `ft_chain_compress_register_retire` has already registered the boundary and
  both compressed neighbours into the **caller's shared txn** *before* the
  `:1314` re-validation (`ft-remove.h:1217`).  `-EAGAIN` is safe there only
  because the caller aborts and destroys that txn (`ft-remove.h:3401`).  A path
  that lets the caller commit anyway — which is what `cret > 0` does today
  (`ft-remove.h:3404`) — would commit those registrations *terminal-less* and
  strand their LOCK bits on live nodes.  ☞ **INFERRED from the cited contracts,
  NOT measured** — it needs its own check before anything relies on it.

* **☐ OPEN, unverified**: `ft_canonicalize_chain_compress` discards its return
  with `(void)` (`ft-remove.h:1827`), so the verifier-flagged residue looks
  reachable on ANY peer conflict, not only in the rare out-of-bound geometry the
  KNOWN-INCOMPLETE note describes.  Both suites are green today, so either it
  does not happen or a later op canonicalises before any verify point.  Worth
  one arm before Q1 is answered.

* **The abort boundary.**  Every bail added must be *before* any reader-visible
  mutation, or it inherits the leave-in-place audit this branch exists for.
  Measured for the standalone arm: every pre-publish bail destroys its own txn,
  which drains the registered locks and frees `new_cn` (`ft-remove.h:1312`,
  `:1337`, `:1370`; the drain is `ft-mutation-helpers.h:4050`) — byte-for-byte
  clean.
* **Reservation budget.**  Shape (b) adds records to commits that are already
  sized by `FT_REMOVE_COMMIT_REC_MAX_EDGES + …`; the reserve-overflow detector
  is `-DURCU_TXN_DEBUG_RESERVE`.

---

## 7. Reproducing and re-measuring

```sh
# tracing tree
CPPFLAGS="-DFT_ENABLE_TRACING -DFT_LIGHT_TRACING -I<tree>/src/fractal-trie -I/usr/local/include" \
LDFLAGS="-L/usr/local/lib -Wl,-rpath=/usr/local/lib" CFLAGS="-O2 -g -DNDEBUG" ../configure
make -C src -j48

# session: 1 MiB x 8 overwrite, mutator events only, vpid/vtid context
lttng create $S --snapshot --output $OUT
lttng enable-channel -u --overwrite --subbuf-size 1M --num-subbuf 8 mutator
lttng enable-event -u -c mutator \
  cds_ft:ext_violation,cds_ft:recompact_head_snap,cds_ft:edge_record,\
cds_ft:txn_commit,cds_ft:metadata_set_external_nodes,cds_ft:set_parent,\
cds_ft:item_retire,cds_ft:node_recompact,cds_ft:unchain_node
lttng add-context -u -t vpid -t vtid && lttng start
FT_TRACE_SESSION=$S CHK=1 WRITERS=2 ALPHA=2 MAXLEN=8 NSTABLE=100 NCHURN=100 \
  SECS=5 READERS=4 NOFREE=1 ./stab5_tr
```

☠ Four traps, all paid for once:

1. `cds_ft_debug_ext_violation` runs `lttng stop` — **re-create the session
   before every run**, or the next run traces nothing.
2. Per-UID rings retain a dead process's events — **filter by `vpid`**, taken
   from the trace, not from the shell's job pid.
3. `NOFREE=1` — without it a recycled malloc chunk makes a traced pointer
   ambiguous and the whole analysis is "what happened to *this* node".
4. Resolve `item_retire`'s caller as `caller - (self - nm_offset(cds_ft_debug_ext_violation))`;
   `@self` on `ext_violation` is the ASLR anchor.
