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

★ **MEASURED, not inferred** (`cds_ft:chain_compress_enter/exit`, 2026-09-04):
the losing caller is the **shape-D detach fold** (`ft-remove.h:3356`,
`plan_nr_child == 2`), and the losing op **enters the callee with
`external_nodes` ALREADY non-NULL** — equal to the lost key's own node — and
carries it **unchanged** through the commit.  So the window is not
plan-vs-commit inside the callee at all: the peer's park lands between the
caller's **pre-fence gate** (`ft-remove.h:3258`, a plain read of
`!bmeta->external_nodes`) and the call ~120 lines later, and **nothing between
there and the retire ever looks at the word again** — not the callee's entry,
not the `:1314` re-validation under the fence, not the commit.

☠ This corrects the framing in §0: the predicate does not go stale *during* the
disposal, it is **already false when the disposal begins**.  A probe that
watched for the word CHANGING between entry and commit found **zero** such
pairs across three runs.

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

### 3.3 THE TAXONOMY IS COMPLETE — two defects, discriminated exactly

☠☠ **THREE SUCCESSIVE VERSIONS OF THIS TABLE WERE THE INSTRUMENT, NOT THE
TRIE**, and every correction moved samples INTO a known defect:

1. a row labelled "`hext != node`" that **never compared them**;
2. `headis=null` invented whole — a **COMPRESSED** holder keeps its single
   external child in `cn->child`, and `meta->external_nodes` is legitimately
   NULL there;
3. the record described the **HOLDER**, then the holder **and its parent** —
   while the disposal that loses a key can be at **any** ancestor.  A classifier
   that inspects a fixed number of levels invents a residual shape for every
   defect that happens one level higher than it looks.

The fix is to stop counting levels: walk **up to the trie root** and report
`anc_tomb` (hops to the nearest TOMBSTONED ancestor, 0 == the holder, -1 == none)
and `anc_root` (hops to reach the trie, -1 == the up-walk never does).

**42 samples** (30 + a 12-sample `RETRY=1` control):

| oracle shape | n | `anc_tomb` | reading |
|---|---|---|---|
| **RM-LOOKUP-MISS** (the lost key) | **31** | **≥ 0 in EVERY ONE** | a RETIRED ANCESTOR — §3.1 / §3.2 / §3.3.2, ONE defect |
| **RM-FAIL** | **6** | **-1 in EVERY ONE**, and every one carries an `rmsite` | the bogus refusal — §3.3.1, a SECOND defect |
| STALE-FOUND | 2 | -1 | ☐ a removed key still found; not investigated here |
| `rc=124` / `rc=137` | 3 | — | ☐ liveness: two timeouts and one memcg-SIGKILLed leak |

★★ `anc_tomb` **discriminates the two defects exactly**: not one lost key
without a retired ancestor, not one refusal with one.  `anc_root` was ≥ 4 in
every sample, so no cluster was detached from the trie — the losses are
ancestors retired *under* a correctly wired subtree, not broken links.

★ **The losses are HARD, not transient.**  A key can be momentarily unreachable
to the exact path while a peer splits or re-merges the run it sits under
(measured previously at ~1400 per 3000 cycles, with an immediate retry always
succeeding).  `RETRY=1` asks a second time **in the same read section** and only
a second failure aborts: **10 of 12 still aborted on RM-LOOKUP-MISS.**
☠ The `wtransient` counter itself printed nothing — the run aborts before its
summary, the atexit-detector trap — so the evidence here is the abort surviving
the retry, not the counter.

#### 3.3.1 ★ ROOT-CAUSED: the bogus NOT_FOUND from an identity-compare arm

`cds_ft_remove` picks between four arms on how `@node` hangs off its holder
(`ft-remove.h:5267`): a non-head duplicate, a **compressed** holder's single
child, an internal holder whose **head word is `@node`** (the PREFIX-key arm,
`ft_node_external_nodes(holder_flag) == node`), else *"internal holder, @node is
a body child"*.  The last two each end in an identity compare that answers
**`CDS_FT_STATUS_NOT_FOUND`** on a mismatch — `ft-remove.h:5308` (compressed,
`*head_slot != node`) and `:5520` (body child,
`holder_body[key[key_len-1]] != node`).  Both fire in the histogram.

MEASURED at those returns: `htomb=0 headis=node`.  A **fresh**
`ft_node_holder(@node)` says the node IS its holder's head, and the holder is
**alive and untombstoned**.  So the branch tested a **STALE `holder_flag`** —
derived from `@node->prev` at `:5054` and never re-validated — took the wrong
arm, and returned a **permanent, key-losing answer to a transient race**.

☞ Not the flip proxy: the prefix arm reads through `ft_node_external_nodes` →
`ft_dereference_external`, which resolves them (`ft-helpers.h:1147`).  The
staleness is in **which holder**.  ☞ And `:5308`'s own comment reassures that
*"a peer republished the holder … is caught by the commit's expected-value CAS
… as ABORT -> retry"* — but this return happens **before any commit**, so the
reassurance does not cover the exit it is written on.

☞ The op ALREADY has the escape (`*need_retry`, `:5165`) and the tombstoned
(`:5081`) and coarse-spacing (`:5103`) cases are ALREADY re-derived.  Holder
replaced, **not yet tombstoned**, per-node spacing is the hole between them.

**A/B, 24 seeds each:** re-derive at the `:5520` return and `*need_retry` when
the holder moved → **4/24 → 0/24, eliminated.**  ☠ But one run returned
`rc=137` (a memcg-SIGKILLed livelock) and two moved to a lookup miss that never
reaches the remove.  Right diagnosis, **not yet the right disposition** — Q1b.

#### 3.3.2 ★ THE `headis=other` SHAPE IS §3.1's DEFECT, ONE LEVEL UP

Caught in the act (`trace-o-s5`), on the parent `P = 0x7FF93EC39341`:

```
.989065371  A  chain_compress_enter  boundary=P  plan_nr_child=2  ext=0x0
.989066052  B  edge_record  slot = a BODY SLOT of P:  head -> B's fresh junction
.989069387  B  txn_commit status = 0                     ← B's publish INSTALLED
.989069307  A  chain_compress_enter  boundary=P          ← re-attempt, AFTER B
.989071630  A  chain_compress_exit   status = 0          ← A's collapse COMMITTED
.989071750  A  item_retire item = P                      ← the parent B published into
```

B's fresh junction is **alive, correctly wired, and holds the key** — and hangs
off a node A freed.  ★ `ext = 0x0` throughout, so §3.1's `external_nodes`
re-validation would never have fired here.  What went stale is **`plan_nr_child`
plus the identity of the slot A is DETACHING**, and `ft-remove.h:1314` re-reads
**only the SURVIVOR's slot** — B wrote the *other* one, as a **same-slot value
swap**, so `nr_child` did not move either.

☞ So §3.1 and this are ONE defect with two uncovered words, and
`ft_node_recompact`'s DEL arm already carries the fix shape for the second one:
`@nullify_expected`, a plan expected-old on the slot being dropped.

### 3.4 Already correct — the model to copy

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

4. **The re-validation, corrected to cover the caller that actually loses the
   key** — `plan_nr_child == 2 || (!freeze_leaf && !dead_cell)`, i.e. shape-D
   plus canonicalize.  **No livelock this time** (the caller's own gate
   re-evaluates on the retry and skips the shape-D branch), but **8/8 still
   red** — because the compress is only 2/5 of the routes (§3.3).

★ (2), (3) and (4) together say the cure is **not** a NULL test at one site.  It has
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

**Q5b — the BOGUS REFUSAL (§3.3.1) is root-caused: what disposition?**  It is
NOT the disposal class — the holder is alive, nothing was retired, and the op
simply took the wrong arm on a stale `holder_flag` and answered NOT_FOUND.  The
diagnosis is settled (4/24 → 0/24 on a targeted probe).  What is not settled is
the same question as Q1b: `*need_retry` alone produced **one `rc=137` livelock**
in 24, so a bounded re-derivation, a re-descent, or a different arm-selection
that cannot go stale all need weighing.  ☞ It may be cheaper to make the
FOUR-WAY BRANCH ITSELF robust — re-derive the holder immediately before it,
under the same mark that protects the arms — than to patch each arm's refusal.

**Q5c — ☑ CLOSED.**  There is no residue: with the up-walk in the record,
**every** RM-LOOKUP-MISS in 42 samples has a tombstoned ancestor and **every**
RM-FAIL has none.  What is left outside the two defects is 2 STALE-FOUND (a
removed key still found — a different oracle check, uninvestigated) and 3
liveness events (two `rc=124` timeouts, one `rc=137`).  ☞ So a candidate fix
CAN now be measured: the metric is "RM-LOOKUP-MISS count with `anc_tomb >= 0`",
and it does not need the reproducer to go green.

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
cds_ft:item_retire,cds_ft:node_recompact,cds_ft:unchain_node,\
cds_ft:chain_compress_enter,cds_ft:chain_compress_exit
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
