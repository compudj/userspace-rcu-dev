# The disposal plan read before its own lock — design brief (2026-09-04)

*(filename kept as `ft-stale-disposal-predicate.md`: the plan of record links
it, and "stale disposal predicate" is still the name of the defect.)*

Status: **BRIEF. No design chosen, no implementation proposed.**
Measured at `c6c06d12` on branch `ft/unpub-free-audit`.

☠ **FRAMING CORRECTION (Mathieu, 2026-09-04).**  The first draft stated the
mechanism as "the word never becomes a read-set entry, so the MCAS has nothing
to arbitrate", and offered three cures of which two entrenched MW on a lockable
word.  That is the wrong axis: the FT is migrating **TO fine locking and AWAY
from MW CAS**, so "the MCAS would arbitrate it" describes the safety net being
removed, not a design.  Re-stated below on the lock axis — where the same
measurements say something simpler and stronger: **every word involved already
has an owner whose lock its writers already take, and the only defect is that
the disposal reads those words before it acquires.**
Decides: the **shipping two-writer key loss** — two concurrent
`cds_ft_insert` / `cds_ft_remove` writers on one trie, which the header grants
for `CDS_FT_WRITER_LOCK_FINE` — and it **removes G4 from the critical path**
for it.

☠ **It does NOT close the plan's open box.**
`mw-to-fine-locking-remainder.md:2718` asks whether the holder's lock excludes
every writer of a chain member's **`->prev`**.  That box stays open: the word
that loses the key here is the holder's `external_nodes` or one of its body
slots, not `->prev`.  What this brief does contribute to it is the *shape* of
the answer, in the form that survives the transition: for these words **every
writer already holds the node's lock**, and the acquires **are** ordered by it —
so the lock argument is available and it is the right one.  What is missing is
that the disposal's PLAN is read **outside** the window the lock covers.

---

## 0. The one-paragraph statement

Two concurrent `cds_ft_insert` / `cds_ft_remove` writers lose keys, in contract,
on the default `CDS_FT_WRITER_LOCK_FINE`.  It is **not** a missing lock and
**not** the external-head back channel.  Both writers take the holder's node
lock and they serialize correctly — the second acquires only after the first's
commit has released, measured at 110 ns in one instance and 4.0 ms in the
other.  The defect is an **ORDER INVERSION between the plan and the acquire**.  A
**disposal predicate** — "this node has one child and no external head", "this
ancestor is an orphan" — is read from the node **before** the op takes that
node's lock, and is never read again **after**.  The peer then writes a
*different word of the same node*, legally, holding the same lock; the disposal
wakes up on the far side of its own acquire still believing what it read
before it, and frees the peer's key inside the disposed node.

★ Nothing here needs arbitrating.  Under fine locking **the mark IS the
exclusion**, and every writer of every word involved already takes it.  The
repair is to read the predicate under the lock — not to add a validator, and
emphatically not to route a lockable word into the always-MW lane.

> **owner-AVAILABLE ✓ · owner-HELD ✓ · HELD-WHEN-THE-PLAN-WAS-READ ✗**

---

## 1. The lock covers the words.  It does not cover the READ.

**Every word involved has a nameable, lockable owner, and every writer of it
already takes that owner's lock** — measured, both sides, in the traces of §2:

| word | who writes it | do they hold the node's lock? |
|---|---|---|
| `&meta->external_nodes` | `ft_insert_park_external_nodes` (`ft-insert.h:927` acquires, `:929` records) | **yes** — the trace shows the acquire and the release terminal |
| a body slot | the publishing insert | **yes** — the disposing op acquired the same word 110 ns *after* the peer's commit released it |

That is the durable form of the argument: not "MW arbitrates it today", but
"every writer of this word already holds node X".  It survives the migration,
and it means **no new arbitration is needed anywhere in this brief**.

What the lock does not cover is **when the disposal read the word**.  The op
forms its plan from the node, *then* acquires, *then* commits — and only
`nr_child` and one body slot are read again on the far side of the acquire:

* `nr_child` lives **inside the state word** (`ft_state_nr_child`), the same
  word the lock bit lives in, so the acquire's own snapshot
  (`ft_held_anchor_sample_node` → `iter_held.node_snap`) carries it for free and
  `ft-remove.h:1314` checks it;
* `ft-remove.h:1314` also re-reads exactly **one** body slot — the survivor's
  (`ft_node_get_nth(... surviving_byte ...) != surviving_child`);
* **`external_nodes` is never re-read**, and it is a separate word
  (`struct cds_ft_metadata`: `parent_word` @0, `external_nodes` @+0x8, …
  `state` @+0x20), so the acquire snapshot cannot carry it either;
* **every body slot but the survivor's is never re-read**, and a peer's publish
  into one is a *same-slot value swap*, which moves neither `nr_child` nor the
  parent slot.

☞ So the file has already chosen the right pattern and applied it to two of the
four things the plan reads.  `ft-remove.h:1314` says so itself:

> *"Re-validate the caller's PRE-fence plan under the fence … a peer commit
> between the caller's derivation and the mark is exactly what the mark
> snapshot cannot vouch for."*

☞ The tree already books half of this at `ft-mutation-helpers.h:6784`:

> "@external_nodes is checked but NOT covered by the fence: **no retire
> validates that word (a known, separate hole** — ft_insert_park_external_nodes
> publishes it with no lock)."

★ *"not covered by the fence"* is exactly right, and is the lock statement.
☠ Its parenthetical is **stale and points the wrong way**: the park *does* take
the holder's lock (`ft-insert.h:927`), which is what makes the fence the right
place to look in the first place.

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
                     ☠ writer1 never re-reads &meta->external_nodes after its
                        own ACQUIRE -- its "no external head" was read before it
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

★ Read this as a LOCK trace, not a CAS trace.  A acquires the node's state word
**110 ns after B's commit released it** — the exclusion works perfectly.  What A
never does is look at the node again once it holds it: B's publish is a
**same-slot value swap**, so it moved neither `nr_child` nor the state word, and
A's plan — formed before the acquire — still describes the node as B found it.

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

☠ Sharpens §0: the predicate does not go stale *during* the disposal, it is
**already false when the disposal begins**.  A probe that watched for the word
CHANGING between entry and commit found **zero** such pairs across three runs.
★ Which is the good news for the fix: the word does not have to be *watched*,
only **read once, on the far side of the acquire**.

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

☞ So §3.1 and this are ONE defect with two words read too early, and
`ft_node_recompact`'s DEL arm already carries the shape for the second one:
`@nullify_expected`, the plan's value for the slot being dropped, re-read under
the fence.

### 3.4 Already correct — the model to copy

`ft_node_recompact`'s DEL arm carries `@nullify_expected`
(`ft-mutation-node.h:1707`): it re-reads, under the fence, the slot its plan
named, and bails `-EAGAIN` when the two differ — with a comment that states the
class exactly (*"A peer that republished this slot between the two … makes the
two differ, and copying 'every child except this slot' then drops the peer's
subtree whole"*).  ★ It is a **plain load under the mark**, not a validator and
not an MW record.  That is the shape a fix has to generalise.

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

★ (2), (3) and (4) together say the cure is **not** a NULL test at one site: the
re-read has to compare against **the value the caller's plan assumed** (which
differs per caller — Q2), and it has to cover §3.2 as well, where the word read
too early is a body slot.
★ And (4) is the load-bearing liveness datum: with the *correct* predicate there
was **no livelock**.  The 8/8 `rc=137` in (2) came from testing the wrong thing,
not from bailing.

---

## 5. The questions for Mathieu

**Q1 — where is the plan formed, relative to the mark?**  In the lock frame
this is a **code-motion** question, not a choice of mechanism.  §1 measured that
every writer of both words already holds the node's lock, so the mark is the
exclusion and nothing needs arbitrating; the only defect is that the disposal
reads the words **before** it acquires.  Two shapes:

* **(a) COMPLETE the under-fence re-read** at `ft-remove.h:1314` — the pattern
  the file already chose and already applies to `nr_child` and the survivor
  slot.  Add `external_nodes` and the **detached** slot, each compared against
  the value the caller's plan assumed (Q2).  Smallest diff; leaves the plan
  where it is, so every future field the plan grows must remember to join the
  list.
* **(b) HOIST the plan INSIDE the fence** — acquire first, then derive
  `plan_nr_child`, the survivor, the head and the detached slot from the node
  the op now holds.  Then there is nothing to re-validate, and the class cannot
  come back.  Fine-locking-native, and the only shape that is robust by
  construction; costs a restructure of the caller/callee split, since today the
  caller derives and the callee acquires.

★ **The real question under both**: for each disposing op, does its lock-set
ALREADY contain every node whose words its plan reads?

* `ft_chain_compress_fused` — **yes**: the boundary is a lock-set member, so
  this is pure code motion.
* `ft_detach_node`'s orphan climb — **not established**: the climb reads
  `metadata->external_nodes` at `ft-remove.h:2332` / `:2352` / `:2460` to choose
  the orphan set, and the orphan acquires happen later.  This is the one place
  the answer might be "grow the lock-set" rather than "move the read", and that
  is a liveness cost on the hottest op, so it wants its own measurement.

★ **Not adding an acquire.**  The locks are already taken; the reads move to
after them.  So (a) and (b) should both be liveness-neutral — unlike the
`ft_detach_node` case above, if it turns out to need a new member.

☠ **What is OFF the table, and was wrongly on it in the first draft of this
brief**: routing either word through `ft_flip_txn_record_tag_mw`, or giving
`external_nodes` emptiness a bit in `state` so the acquire snapshot carries it.
Both entrench MW on a word that has a nameable, lockable owner, and "a slot is
SW xor MW, globally" means one such decision pins every other writer of that
word to MW too.  The direction is TO fine locking and AWAY from MW CAS; "the
MCAS would arbitrate it" describes the safety net being removed, not a design.
(The state-word bit is also only half a fix — a bit meaning "has an external
head" says nothing about which body slot holds what.)

**Q1b — and what does a failed re-read DO?**  Much smaller than it looked:

* Under **(b)** it largely cannot arise — the plan is derived under the mark, so
  it is simply correct.  What remains is the **acquire MISS**, which already has
  a settled answer: `t->acquire_miss` is recorded and the commit ABORTS,
  all-or-none, and the op re-plans (`ft-mutation-helpers.h`, the
  `lock_or_guard_parent` miss path — *"Deliberately not a spin"*).
* Under **(a)** the bail is `-EAGAIN`, and **refutation (4) measured no
  livelock**: the caller's own gate re-evaluates on the retry and stops
  selecting the collapse, because the boundary genuinely is no longer a collapse
  candidate.  The 8/8 `rc=137` in refutation (2) came from a predicate that
  could be permanently false, not from bailing as such.
* ☠ **Skipping is refuted** (§6): the canonical form is enforced by
  `cds_ft_verify` and by an `abort()` at the writer-scope exit.
* ☐ Two residuals, both flagged in §6: the `record_only` fold arm, where
  `-EAGAIN` is safe only because the caller destroys the shared txn (INFERRED,
  not measured); and the SECOND defect (§3.3.1), where `*need_retry` eliminated
  the shape but produced **one `rc=137` in 24** — there a bounded re-derivation
  or a re-descent is probably needed instead of a plain retry.

★ For the second defect the same lock reading applies and points somewhere
specific: `cds_ft_remove`'s four-way branch reads a `holder_flag` derived from
`node->prev` far above any mark, so **derive the holder under the mark and the
whole arm-selection stops being able to go stale** — which is cheaper than
patching each arm's refusal, and closes `ft-remove.h:5308` and `:5520` together.

**Q2 — (only under Q1(a)) what value does the re-read compare against, per
caller?**  §3.1 says two callers assume NULL and two assume "the head I am
removing".  Is that a *parameter* the caller passes, or is it derivable inside
the callee from the arguments it already has (`freeze_leaf`, `dead_cell`)?  The
second is tempting and refutation (3) shows it is **not sufficient on its own**.
★ Q1(b) deletes this question entirely: a plan derived under the mark has no
"value the caller assumed" to thread.

**Q3 — is `ft_detach_node`'s climb the same fix, or a lock-set change?**  This
is the one place the answer may not be code motion.  The climb reads
`metadata->external_nodes` at `ft-remove.h:2332` / `:2352` / `:2460` to choose
its orphan set and acquires the orphans later, and it collects up to
`FT_MAX_DEPTH` of them.  Re-reading every body slot of every orphan under its
mark is a different cost class from re-reading one word per boundary.  Is there
a cheaper *lock* statement — e.g. climb-and-acquire in one pass, so each level
is read only after its own mark is taken?  ★ Note that anything that grows the
lock-set, rather than moving a read, costs liveness on the hottest op.

**Q4 — Does this retire G4?**  G4 asks whether ordered cells and dup-chain
splices keep a narrow MW lane or grow a state word, and the plan defers it to a
Phase C measurement.  This defect is **not** in that lane: the contended word is
a holder's `external_nodes` or a body slot, both of which already have an owner
and a lock.  So G4 need not be decided to fix the shipping key loss.  ★ And in the lock
frame the two do not even touch: G4 is about words that have **no state word to
lock** (cells, externals), while every word in this brief has a lockable owner
that its writers already take.  Nothing here should be routed to the MW lane,
so nothing here prejudges G4.

**Q5b — the BOGUS REFUSAL (§3.3.1) is root-caused: what disposition?**  It is
NOT the disposal class — the holder is alive, nothing was retired, and the op
simply took the wrong arm on a stale `holder_flag` and answered NOT_FOUND.  The
diagnosis is settled (4/24 → 0/24 on a targeted probe).  What is not settled is
the same question as Q1b: `*need_retry` alone produced **one `rc=137` livelock**
in 24, so a bounded re-derivation, a re-descent, or a different arm-selection
that cannot go stale all need weighing.  ☞ ★ **The same lock reading applies and is the likely answer**: the branch
chooses its arm from a `holder_flag` derived far above any mark, so DERIVE THE
HOLDER UNDER THE MARK and the arm selection stops being able to go stale.  That
is Q1(b) at a second site, it closes `ft-remove.h:5308` and `:5520` together,
and it is cheaper than patching each arm's refusal.

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

  ★ So a collapse whose under-mark re-read fails must **re-plan** or **refuse
  terminally** the way the fold arm's `-EDOM` does.  Never skip.  ☞ Under Q1(b)
  this mostly stops arising: a plan derived under the mark has nothing to fail.

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
* **Reservation budget — ★ NOT SPENT.**  A plain load under the mark records
  nothing, so neither Q1 shape touches the commits' `FT_REMOVE_COMMIT_REC_MAX_EDGES
  + …` sizing.  That is one of the reasons the lock reading is cheaper than the
  validator reading it replaced.  (If anything ever does add records here, the
  reserve-overflow detector is `-DURCU_TXN_DEBUG_RESERVE`.)

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
