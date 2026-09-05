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

### 3.2 `ft_detach_node` — ☠ AN EARLIER VERSION OF THIS SECTION BLAMED THE WRONG WALK

It said "the climb reads `external_nodes` to decide the orphan set and the
orphan acquires happen later".  That conflates **two different walks**, and the
one it named is not the one that picks the orphans:

* the **ORPHAN DOWN-WALK** (`ft-remove.h:2790`+) picks the set to free, and it
  is **ALREADY CORRECT**: it acquires each orphan (`ft_detach_orphan_acquire`,
  `:2826`) *before* reading its `nr_child` (`:2836`) and `ext_nodes` (`:2838`),
  and its own comment states the discipline — *"§9.2 plan-lock: lock acquire the
  orphan BEFORE reading its nr_child for the collapse decision, so the 'retire
  it' verdict is derived from a FROZEN word"*.  This is Q1(b) done natively.
* the **UP-CLIMB** (`:2400`–`:2660`) has **no acquires at all**.  It reads
  `ft_meta_nr_child_load` (`:2467`) and `metadata->external_nodes` at `:2481`
  (the `nr_clear` tally), `:2501` (the BOUNDARY STOP test) and `:2609` (the
  promote into `topmost_external_nodes`) — every one of them unlocked, feeding
  a plan whose nodes are locked later or not at all.

☞ So the open question here is narrower than the section claimed, and it sits
at the seam between the two walks: the down-walk's stop test is
`if (!phase2_first && (nr_child > 1 || ext_nodes))` (`:2853`), which **exempts
the FIRST orphan** because *"the target itself may carry residual content
(external_nodes) that was promoted as topmost_external_nodes"* (`:2766`).  That
exemption is keyed on POSITION, while the thing that makes it safe is
IDENTITY — the head there being the one the climb promoted, read unlocked at
`:2609`.  ☐ Whether a peer can park a different head in that window, and
whether `topmost_external_nodes` is the right comparand, is Q3.

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

**Q1 — ☑ ANSWERED AND IMPLEMENTED for `ft_chain_compress_fused`** (@`b0995d2c`
+ @`6792267b`), by shape (b): `struct ft_chain_compress_intent` carries what the
OP intends, `ft_chain_compress_plan_stale()` reads the trie under the boundary's
own mark, and all four callers name their target.  Both uncovered words are now
checked there — the external head, and the child the commit detaches (the pair
the climb condemned, `@elevated_old_child` at byte `@n`, compared **raw** via
`ft_node_get_nth_skip`).  ★ Arm yield over 20 seeds of the two-writer
reproducer: **7 detach refusals across 17 runs, 5 head refusals** — races that
previously proceeded and now abort.  ☠ It does NOT claim a measurable loss-rate
drop: the compress is one route, §3.2 is untouched, and n=30 cannot resolve it.

☠ **Two wrong targets on the way, both caught by measurement, not review**: the
raw slot word compared against `ft_node_get_nth` wedged ft_unit at test 2
(`ft_node_get_nth` RESOLVES a skip-compressed word to a different address, so
the compare was shape-determined, not racy — the 347-vs-441 split); and
re-reading the boundary at `@n` at the call site named the right node too late
to be a plan value and was **inert**, 0 refusals in ~100k armings.

☐ **Still open for `ft_detach_node`'s orphan climb** (§3.2), which is the
question below.  The original three-option Q1 is kept for the record:

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

**☑ CONTRACT RULING (Mathieu, 2026-09-04) — the rule the disposition must obey:**

> *"a remove may refuse if the key is not found, but should not refuse due to a
> transient condition"*

★ That settles Q1b(3) and it **rules out the fallback this brief proposed**:
"concede `NOT_FOUND` after a bound" is refusing for a transient reason.  The
terminal answer must be a fact about the TRIE — "the key is genuinely absent" —
never a fact about CONTENTION.  So the disposition must keep working until it
can make a true statement, which means it must **converge**, not give up.

**Q1b — ☑ MOSTLY ANSWERED, by building it.**  The headline is settled and
shipped; what is left is four named residuals, not a design choice.

**Settled by implementation + measurement:**

* the **chain compress** returns `-EAGAIN` (@`b0995d2c`, @`6792267b`).  No
  livelock — the caller's own gate re-evaluates and stops selecting the
  collapse.  Yield over 20 seeds: 22 head + 9 detach refusals.
* the **orphan walk** returns `-EAGAIN; goto end` instead of `break`
  (@`f1888262`, @`9b5d2c60`); `ft_unit` 43 s, yield 4 refusals across 18 runs.
* **skipping stays dead** (§6): `cds_ft_verify` rejects the residue and the
  writer-scope verifier `abort()`s on it.
* the **acquire miss** was never in question — `t->acquire_miss` aborts the
  commit all-or-none and the op re-plans, *"deliberately not a spin"*.
* **the gate agrees**: 60/62 at @`e36c5a3e`, the two reds byte-identical to the
  baseline.

**☐ The four residuals, most load-bearing first:**

1. **The `record_only` fold arm — VERIFIED IN CODE, ZERO MEASURED COVERAGE.**
   Its sole caller is `ft-rekey.h:3724`; the `detach_bail:` block handles
   `if (ret)` generically and calls `ft_flip_txn_destroy(txn)`, whose sweep
   releases the registrations *through each node's own metadata*, ordered before
   the unpublished frees.  The path is ALREADY reachable for `-EAGAIN` from two
   pre-existing sources (the up-front lock-set acquire; the `parent_guard`
   read-set validation), so the new refusal is a third source of an
   already-handled return — not new handler code.  ☠ But `record_only` is set
   only by the REKEY lane, so the two-writer reproducer cannot reach it:
   **measured `fold=0` in 8 of 8 runs**, every refusal on the standalone arm.
   The gate exercises rekey but single-writer, where the check never fires.  ☞
   Real coverage needs a CONCURRENT REKEY workload.
2. **The second defect's disposition — ☑ DIAGNOSED AND MEASURED (@`14de2ecf`),
   ☐ THE CONVERGENT FORM IS DESIGNED AND UNBUILT.**  See §5.1 below.
   *(historical framing kept:)*  The bogus
   `NOT_FOUND` at `ft-remove.h:5308` / `:5520` is exactly as it was.  The one
   probe (`*need_retry` on holder movement) eliminated the shape 4/24 → 0/24
   **and produced one `rc=137` in 24**, so a plain retry there can spin — unlike
   the two dispositions that landed.  ☞ The lock reading points at deriving the
   holder UNDER THE MARK so the four-way arm selection cannot go stale, closing
   both sites at once.
3. **Is a terminal refusal from `cds_ft_remove` contractually acceptable?**
   Only bites if (2) lands that way, and no measurement decides it.
4. **Liveness under sustained contention — nobody has measured it.**  Every
   `-EAGAIN` added here is "one retry per peer event" BY ARGUMENT (the mismatch
   is a committed peer write already in the op's locked snapshot, so the
   re-descend sees a settled tree).  The evidence is 5-second two-writer runs
   and a single-config gate.  This trie has a starvation history — a deep rekey
   starved ~12,000x inside one call, the remove retry lane not draining — and
   these refusals sit on the hottest op.  ★ The argument is sound and
   unmeasured, and those are different things.


### 5.1 ☐ THE CONVERGENT DISPOSITION FOR `cds_ft_remove` — designed, not built

The contract ruling makes "retry then concede" illegal, so the disposition must
converge.  The measured spin is not bad luck, it is structural:

> the recheck's escape is a **pre-commit bail**, which **forfeits the op's lane
> turn** (`rcu-txn.h:1225`), so every restart re-derives against a moving tree
> with **no exclusion ever held across derive → select → compare**.

★ So the answer is not to retry more cleverly.  It is to **hold the exclusion
across those three steps**: acquire the holder BEFORE the four-way branch, and
on a moved holder re-derive IN PLACE under a local loop rather than restarting
the op.  This is the same shape already landed for `ft_chain_compress_fused`.

**Why it converges.**  The peer that re-homes `@node` is a holder copier: it
holds H's `FT_STATE_LOCK` from mark to commit and retires H in that commit
(`ft-mutation-helpers.h:2925`).  Once our `ft_acquire_member` on H succeeds
(untombstoned) and `ft_node_holder(node) == H` is re-read AFTER the CAS, nothing
can move: a publish into H fails the §4.B guard, and a copy of H needs the lock
we hold.  The branch and both compares then read FROZEN words, so a mismatch
under that lock is a TRIE FACT — `ft_node_is_removed(node)`, or corruption —
which is a legal refusal.  Peers are throttled at their OP boundary once we take
the lane: op handles are domain-bound (`ft_txn_op_init` → `&ft->txn_domain`),
so a peer mid-op finishes at most one commit and then queues.

**✓ VERIFIED HERE, not taken on trust:**

* **cost is zero extra CAS** — every arm ALREADY acquires the holder
  (`ft_unchain_node` asserts it never falls through unlocked; the other arms go
  through `ft_flip_txn_lock_or_guard_parent` / `ft_remove_one_commit` /
  `ft_detach_node`).  The hoist MOVES that acquire; the choke point dedupes
  against the ctx's held set and returns `held.shared`.
* **no count pre-decrement precedes the branch**, so a re-derive has nothing to
  undo.
* **the convergence premise holds**: op handles are domain-bound, while all 13
  per-arm txns are standalone (`_on`: zero) — the deliberate choice that stops
  them manufacturing the conflicts their retry loops absorb.

**What must move**: the block that builds `cell_was_head` / `dead_cell` /
`cell_succ` / `fuse_remove` / `pub` / `unsplice_txn` goes BELOW the new acquire
(`cell_succ` must be read under the lock, and the txn then never needs
destroying on a re-derive); the loop label goes above the two existing
correction arms so `holder_depth` and the descent re-derive; the EXTERNAL-holder
arm is skipped (a non-head duplicate has no state word — `ft_unchain_node` locks
`ft_chain_head_holder` itself), and the two NOT_FOUND compares live only in the
compressed and body-child arms anyway.

**☠ How to measure it — NOT attempt count.**  `-DFT_DEBUG_REMOVE_RETRY_CAP` is
blind (it reports at 100/1000/10000 and this arm never passes 100 while 5/20
runs died).  Use: the per-op MAX of in-place iterations paired with
`optxn.in_fallback` (prediction: ≤ #writers − 1 once in the lane — a violation
refutes the bound); `ft_dbg_rm_site` at the two compares must be **0** on
oracle-present keys; and the outcome-level signals `rc=124` / `rc=137`, which
are what actually caught the spin.

☞ **This is a lock HOIST on the hottest op.**  It adds no acquire, but it moves
one, and the plan's economics say that class of change wants explicit agreement
before it lands.  Recommended, and not started.


**Q2 — (only under Q1(a)) what value does the re-read compare against, per
caller?**  §3.1 says two callers assume NULL and two assume "the head I am
removing".  Is that a *parameter* the caller passes, or is it derivable inside
the callee from the arguments it already has (`freeze_leaf`, `dead_cell`)?  The
second is tempting and refutation (3) shows it is **not sufficient on its own**.
★ Q1(b) deletes this question entirely: a plan derived under the mark has no
"value the caller assumed" to thread.

**Q3 — ☑ HALF ANSWERED @`f1888262`, ☠ HALF REFUTED.**

☑ **The bail was not a refusal, and now is.**  The down-walk's `break` stopped
collecting but let everything after it run: the replace unlinks the whole chain
while `to_free` holds only `[0, k)`, so orphan `k` was unlinked and **never
freed** — leaked with whatever a peer put on it, and invisible to
`cds_ft_verify` because the node is unlinked rather than live-and-linked.
Single-writer the walk always reaches the leaf, so the arm fires only on a plan
a peer invalidated, and that must abort the op.  Both walks now release the mark
and `-EAGAIN; goto end` — byte-for-byte clean (the walk is read-only,
`orphan_txn` does not exist yet, the sweep at `end` releases every held mark),
and it cannot livelock because the mismatch is a **committed** peer write in the
op's own locked snapshot, so the re-descend sees a settled tree.

☑ **And the exemption IS now keyed on identity — the RIGHT one** (@`9b5d2c60`).
The first attempt required `ext_nodes == topmost_external_nodes` and wedged
`ft_unit` entering `test_density_stress` (252) **single-writer**, RSS flat, a
spin not a leak.  A probe that RECORDS instead of refusing found the shape in
one run: in **every one of 24 samples** the first orphan's own head is
`ext_nodes == NULL` while `topmost_external_nodes` is **non-NULL**, all in
branch 2, `nr_child == 1`, `prev_external_nodes_found` already set.  `topmost`
was promoted from an ancestor **above** this node and is simply not this node's
head — so the design's premise ("`elevated_old_child` is always the promoted-from
node") is false for branch 2, and the old predicate refused the NORMAL shape.

★ What the climb actually believes about the first orphan is weaker: it carries
**no head of its own, OR the one head the climb lifted off it**.  Anything else
arrived after the plan.  So the test is
`phase2_first && ext_nodes && ext_nodes != topmost_external_nodes`.

★ Arm yield, 20 seeds: the orphan refusal fires **4 times across 18 classifiable
runs** (3 runs with ≥1, one seed showing 5 in a single run) — up from 1 across
16 with the position-keyed version, which is the point of tightening it.  Head
and detach refusals from the compress run alongside at 22 and 9.

☞ One run returned `rc=137`.  Re-running that seed five times gives the ordinary
oracle abort every time, and the same rate appears in runs of the UNMODIFIED
library, so it is the known background livelock — not attributable here, and not
claimed clean either.

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

### 5.2 ☠ BOTH DISPOSITIONS FOR DEFECT 2 ARE REFUTED BY MEASUREMENT

The DIAGNOSIS is confirmed: an acquire-first hoist takes RM-FAIL from **11 of 20
seeds to 1 of 20**.  The order inversion is the cause.  What has failed twice is
the DISPOSITION.

**A — plain retry (`*need_retry = true`).**  REFUTED: 5 of 20 seeds now time out
or are killed by the 8G memcg.  The wrapper re-derives the same anchor and spins.
It is also refused on contract: Mathieu's ruling is that *a remove may refuse if
the key is not found, but must not refuse due to a transient condition*, and
"retry N times, then concede NOT_FOUND" is exactly a transient refusal.

**B — acquire the holder before planning** (`-DFT_RM_ACQUIRE_FIRST`, default OFF,
`ft-remove.h` ~:5545).  REFUTED: `ft_inv` HANGS deterministically at test 44
(`inv_insert_replace_splice_window`).  gdb on the hung process puts the spinning
thread at `ft-remove.h:5608`, inside the hoist's own `ft_anchor_descend`.  The
loop ALTERNATES: descend finds the LIVE holder, the acquire takes it, the
validation asks `ft_node_holder(ft, node)` -- the BACK-POINTER -- which still
names the RETIRED holder, so it mismatches, releases, re-aims at the dead word,
is refused as a TOMBSTONE, descends again.

★ **The lesson is one this file already carried.**  The STALE BACK-EDGE note at
`ft-remove.h` ~:5081 documents this same livelock at 2,000,000+ consecutive
attempts with 11 of 12 writers parked, and states the rule: *a tombstone is
permanent, not contention*, and *the FORWARD path is authoritative*.  Disposition
B obeys that rule in its REFUSAL arm and then breaks it in its VALIDATION arm.
Any third disposition must not consult `node->prev` as an authority anywhere.

☠ A contributing cause worth naming separately: `ft_acquire_member` folds three
facts into one `-EAGAIN` -- a peer LOCK and a mid-flip PROXY (both transient) and
a TOMBSTONE (permanent).  A caller that cannot tell them apart cannot write a
terminating loop.

---

### 5.3 ☠☠☠ THE THIRD DISPOSITION IS REFUTED TOO — and on a premise the whole file shares

Three adversarial skeptics were run against the forward-slot design (§5.2's
successor: acquire a candidate holder, then validate by reading the
key-selected slot INSIDE it under its own lock, with a one-way ratchet that
forbids the back-pointer as a candidate source after lap 0).  **All three
returned REFUTED.**  Two of them, working independently on different questions,
converged on the same root cause.

★★★★★ **THE FORWARD PATH IS NOT FORWARD.**  `ft_anchor_descend` resolves every
SKIP-COMPRESSED hop through `ft_descent_step` -> `ft_reanchor_flag` ->
`ft_skip_reanchor` (`ft-helpers.h` ~:1970-2060), **which walks UP BACK-EDGES**:
it reads `ft_head_parent_word_raw(...)` (== `G->prev`) and
`metadata->parent_word`, accumulates upward, and returns `*at_pos = parent` as
the descent's own result.  VERIFIED by reading.  The stronger form: under
default config a compressed holder `cn` is **not forward-reachable at all** --
the grandparent's slot holds `SKIP(node, len)` aimed straight at the external,
and `cn` is recoverable ONLY through `node->prev`.

☞ So *"The FORWARD path is authoritative, so re-derive the holder by a
key-guided descent"* -- the rule this file states at `ft-remove.h` ~:5081, and
the rule BOTH refuted dispositions were built to obey -- is false exactly where
the defect lives.  A "re-descend" lap re-derives the same dead holder with zero
peer commits, which is the §5.2 hang with the roles swapped.
`ft_skip_reanchor`'s own comment (~:1935) cures its non-convergent case on the
premise *"@G's parent chain runs through live nodes"*; that premise is what
fails.

☠ **The suites cannot see it.**  `tests/regression/test_urcu_ft_inv.c:21182` --
*"only `*_spec` keeps `CDS_FT_FLAG_SKIP_COMPRESSED` set"*.  ft_inv 128/128 is
largely blind to the shape that breaks the design.

**Further defects found, each verified in the tree:**

* ☠ **Class (a) would DETACH THE WRONG WORD.**  The predicate
  `ft_node_external(v) && ft_node_ptr(v) == node` is satisfied by a SKIP_X
  dual: `ft_node_external` is only `(v & FT_TAG_MASK) == 0`, a skip word is
  `child | (len << FT_SKIP_LEN_SHIFT)` with the child's tag bits asserted zero,
  and `_ft_node_mask_ptr` strips the length.  The tree already names this trap
  in `ft_slot_in_node` (`ft-helpers.h` ~:2388): *"Skip before external: a skip
  pointer's low bits read as external."*  Test `ft_node_skip_compressed` FIRST.
  ★ A wrong (a) is KEY LOSS, strictly worse than the bogus refusal being cured.
* ☠ **A RECLAIMED holder passes the lock test.**  If the stale `prev` names a
  freed-and-recycled node, the reanchor reads its `len`/`parent_word`, the lap
  LOCKS it -- its word is clean, it is live, merely not ours -- finds
  `->child != node`, and answers NOT_FOUND **at a genuine linearization point
  about the wrong node**.  ☞ `NOFREE=1` HIDES THIS, and every seed of the
  corpus below ran with it.
* ☐ **A root-attached holder has nothing to lock.**  `d.pnf == NULL` names no
  state word (`ft-mutation-node.h`:1579, *"no node to lock, auto-guarded by the
  root-slot CAS"*), so the validation has no meaning there.
* ☠ **Dropping the PER_NODE gate is an EXCLUSION LOSS**, in the tree's own
  words: the re-aim assigns `holder_flag` without re-deriving `holder_depth`,
  and `ft-remove.h`:5655 says *"it becomes a MIS-ANCHOR"* when that gate widens.
* ☠ **HOLD DURATION.**  The mark would be held across the unsplice reservation
  malloc, the branch txn malloc, the arm, its commit, AND the post-commit
  UN-ABORTABLE unsplice loop (~:6040), which spins on OOM.  Pre-hoist the
  holder's release rode the structural commit and nothing was held there.
* ☠ **`-ENOMEM` is folded into `*need_retry`**, though the arms deliberately
  distinguish it (*"-ENOMEM is not a peer ... does NOT age the handle"*).  Under
  memory pressure the remove retries forever instead of returning MEMORY_ERROR.
* ☠ **"No extra CAS" is false**: the op runs TWO acquire commits where it ran
  one -- a standalone acquire txn plus the arm's own set, the dedupe only
  removing the holder from the latter.  Cost, not safety.
* ★ **The laps run inside ONE read-side bracket**, so each lap blocks peers'
  deferred frees: obstruction-free with unbounded RSS.  ☞ **This finally
  accounts for §5.2's Disposition A**, whose 5-of-20 memcg kills had no
  explanation.

**What SURVIVED scrutiny** (so it need not be re-litigated): no deadlock (the
acquire is a try-lock and the set acquire is all-or-none, so a cycle degrades
to ping-pong plus aging, not a wait); release is total (all 10 post-hoist exits
carry it, no `goto`, no double release); no `MAX_LOCKS` overflow (extras never
enter `t->locks[]`); the lock DOES exclude peer writers of the slot; and the
GP-starvation line is not made worse.  ☞ The `shared`/`txn_owned` guard terms
on the anchor are DEAD code.

### 5.4 ☑ THE TAXONOMY RE-MEASURED AT 3.2x — and a THIRD population

**Workload** (without it the table cannot be re-derived): the in-tree rig
against a `-DFT_ENABLE_TRACING` build (`-O2 -g -DNDEBUG`), `CHK=1 WRITERS=2
ALPHA=2 MAXLEN=8 NSTABLE=100 NCHURN=100 SECS=5 READERS=4 NOFREE=1`, seeds
1..160, 8 concurrent, each in its own 8G memcg, default feature flags.
160 seeds -> **134 classified violations** (129 rc=134, 20 rc=139, 5 rc=0,
4 rc=124, 2 rc=137).

| oracle | n | `anc_tomb >= 0` | dominant `headis` |
|---|---|---|---|
| RM-FAIL (defect 2) | 81 | **0/81 — 0.0%** | `node` (72) |
| RM-LOOKUP-MISS (defect 1) | 28 | **28/28 — 100%** | `null` (14) |
| STALE-FOUND | 25 | **19/25 — 76%** | `other` (21) |

★ **The two-defect discriminator is 109/109 PERFECT** at 3.2x the 42 samples
§3.3 was written on.  The taxonomy holds.

☐ **STALE-FOUND IS A THIRD POPULATION.**  It straddles the discriminator and is
marked by `headis=other` (21/25) -- the holder's external head names a DIFFERENT
node than the lookup found.  Every sample carries `rmsite=0`, because the oracle
fires in a LOOKUP, not a remove; and `stale_ext`/`stale_det`/`alone` are
non-zero, i.e. the defect-1 fixes are firing and this happens anyway.  It owes
its own root cause.  ☠ Also unaccounted: **20 of 160 runs SEGV**.

---

### 5.5 ☠ THE `NOFREE=1` ARM WAS HIDING A KIND — RM-WRONG-NODE

§5.3's soundness skeptic marked its worst shape SUSPECTED rather than PROVED
because *the reproducer's own `NOFREE=1` hides it*: with retired nodes leaked,
an address is never reused, so a holder recovered through a stale `prev` can
never be a RECLAIMED node.  Every seed of §5.4 ran that way.  So the arm was
re-run with the ONLY change being `NOFREE=0` (retired nodes go through
`call_rcu`), same 160 seeds, same everything else.

| oracle | NOFREE=1 (leak) | NOFREE=0 (recycle) |
|---|---|---|
| RM-FAIL, `anc_tomb >= 0` | 0/81 | **0/68** |
| RM-LOOKUP-MISS, `anc_tomb >= 0` | 28/28 | **37/37** |
| STALE-FOUND, `anc_tomb >= 0` | 19/25 (76%) | 14/23 (61%) |
| **RM-WRONG-NODE (kind 3)** | **0** | **2** |

★ **The two-defect discriminator now stands at 214/214 across both arms**
(RM-FAIL 0/149 with a tombstoned ancestor, RM-LOOKUP-MISS 65/65 with one).
§3.3's taxonomy is as solid as this rig can make it.

☠ **A KIND THAT NEVER APPEARED WITH LEAKING ON.**  `RM-WRONG-NODE` fires when a
writer looks up its OWN key and is handed a node it never inserted -- and the
churn set is PARTITIONED per writer, so `k->cur` has exactly one owner and
cannot race:

```
RM-WRONG-NODE writer1 key=aaabab nd=0x7f61042f2260 cur=0x7f6104301f80
RM-WRONG-NODE writer0 key=aaaaba nd=0x7ff6fc1811a0 cur=0x7ff6fc194700
```

Both carry `anctomb=-1`, `tomb=0`, `hstate=0x8`, `stale_ext=1`, `alone=1` -- so
neither is defect 1's retired-ancestor shape.  ☐ **UNDIAGNOSED, and only 2
samples**: it needs its own reproduction before it is called a fourth defect.
☞ The methodological point is the durable one: `NOFREE=1` is required for
POINTER-LEVEL analysis and is a BLIND SPOT for anything whose mechanism needs
an address to be reused.  Run both arms.

☞ One more difference, consistent with (not proof of) §5.3's read-side finding:
memcg kills rose 2 -> 8 between the arms.  Freeing is what a blocked grace
period defers, so a lane that spins inside a read-side bracket only shows its
memory cost once `call_rcu` is actually in play.

---

---

### 5.6 ☠☠☠ THE FULL DESCENT CONTRACT — DESIGNED, REFUTED 3/3, AND THE DILEMMA IT EXPOSED

Mathieu's scope call (2026-09-04) on the §5.3 conclusion was **FULL**: make
`ft_skip_reanchor` / `ft_reanchor_flag` never hand back an unvalidated up-walk
result, so every one of the ~17 call sites gets a CLASSIFIED result.  A Fable
agent designed it; three adversarial skeptics, one per load-bearing claim, were
run against it.  **All three refuted.**  Do not build it.

**The design in one paragraph.**  Four outcome classes returned in place of a
bare pointer: `LIVE` (landing node untombstoned, claims the caller's forward
slot, `*fwd_slot` unchanged, `len`/`child` match), `MERGED` (today's
`rewind > 0`), `WITNESS` (all `LIVE` shape checks pass but the landing node is
TOMBSTONED — "a dead node consistent with the live slot"), `TORN` (anything
else).  Termination by a LAP RULE: a lap is permitted only if the witness set
`{*fwd_slot, A, A->state, A->(parent,pso), G's back-edge}` changed; since those
words are written only by committed transactions, a changed set IS a distinct
peer commit and an unchanged one is terminal.  Plus a second, remove-side
proposal (below).

#### The three verdicts

* **The LAP RULE: REFUTED.**  Three independent breaks.  (1) The BOUND
  `laps <= FALLBACK + #writers` has no mechanism: `ft_txn_attempt_bail`
  (`ft-mutation-helpers.h:1584`) is `urcu_txn_conflict` + `urcu_txn_end`, and
  `urcu_txn_end` EXITS the fallback lane (`rcu-txn.h:1272`, `exit_fallback`
  clears `domain->active` at `:784`) — so every lap that ages also forfeits the
  turn and re-queues behind every peer it just released.  (2) "Every listed word
  is written only by a committed transaction" is FALSE: `ft_set_parent`
  (`ft-helpers.h:3114`) is a plain `rcu_assign_pointer` writer of `parent_word`
  reached with `retire_txn == NULL` from the recompact child sweep
  (`ft-mutation-node.h:2359`), and `ft-detach.h:546` plain-stores `parent_word`
  outright.  (3) Cross-trie ops init on `dst_ft`'s domain (`ft-graft.h:1465`),
  so peers mutating the same nodes never queue in this trie's lane at all.
* **The WITNESS class: REFUTED, both halves.**  Soundness: at rest there is no
  bound on staleness but a hard one on the body — retire tombstones then
  `cds_ft_free_item_deferred`, and one grace period later the up-walk's
  `ft_compressed_node_ptr(parent)->len` (`ft-helpers.h:2029`) reads freed
  memory.  The state word that would carry TOMBSTONE is in that same freed body,
  so **there is no pre-check that is not itself the UAF**.  Per-site: the
  precise-lookup terminal is `NOT_FOUND` for a present key — the defect renamed;
  externals carry no key (`fractal-trie.h:509`) so the skipped bytes are
  unrecoverable.  The five ORDERED-QUERY accumulators are worse than the design
  claims: "an undercount is tolerated" is a comment on `ft_subtree_key_count`
  (`ft-mutation-helpers.h:10689`), which backs COUNT queries; the SELECT path
  picks a subtree by `remaining < child_keys` (`ft-ordered-query.h:369`), so an
  undercount returns the WRONG KEY, or `NOT_FOUND` for a valid rank.
* **The remove-side proposal: REFUTED**, and it reproduces disposition C's
  failure modes (a), (b) and (d).  Its SKIP branch defers to
  `ft_skip_reanchor(SKIP word, slot)` — which starts at `cur = G` and takes
  `ft_head_parent_word_raw(G)` (`ft-helpers.h:1997`), i.e. **`node->prev`, the
  very back-edge whose use was the stated reason to delete
  `FT_RM_ACQUIRE_FIRST` and `FT_RM_HOLDER_RECHECK`**.  It also returns the node
  HOLDING the slot (P, the dual's owner), not G's holder, so the body arm
  detaches through P's dual slot with `holder_meta` = P's — C's wrong-word
  detach, one hop later.  And `holder_depth` is dated only by the coarse arm at
  `:5434-5509`; the proposal fires after it and specifies no replacement, which
  `:5653` already names as "a MIS-ANCHOR the moment that gate widens".

#### ★★ WHAT SURVIVED, AND IT IS THE IMPORTANT PART

**F2 SURVIVED** (verified independently by reading every path from `:5388`):
under the default PER_NODE spacing the remove NEVER calls the up-walk before its
arms.  The pre-arm descents are `:5418` (tombstoned holder) and `:5434`
(non-PER_NODE) only; `:5537` is compiled out by default.  In the arms, `:5727`
`ft_skip_to_compressed` reads `child->prev` and `:5972` `ft_node_get_nth_skip`
is a slot scan — neither is the up-walk.  **So a contract on `ft_skip_reanchor`
alone can never reach the RM-FAIL return path.**  That is why the full scope had
to grow a remove-side half, and the remove-side half is what got refuted for the
fourth time.

**☠ AND A MEASUREMENT CORRECTION.**  `htomb` / `headis` / `anc_tomb` are
computed AT ABORT from a FRESH `ft_node_holder(node)` (`fractal-trie.c:871-975`)
— i.e. AFTER the `NOT_FOUND` return.  So "the holder is ALIVE **at the
compare**" was never measured, only inferred; the record cannot order the peer
commit against the compare.  The `anc_tomb` discriminator between defect 1 and
defect 2 is unaffected (it is a property of the trie at abort either way), but
every timing reading built on `htomb=0 headis=node` must be restated as an
inference.

#### ☞ THE DILEMMA — two skeptics converged on it independently

> If `G->prev` can be stale AT REST for a SKIP_X child, then `cn'` is reachable
> by NO path and the trie is not "actually fine" — the bug is a lost node, not a
> misread one.  If it cannot, the WITNESS class never arises on a SKIP_X hop and
> the design is curing a shape that does not exist.

**VERIFIED BY READING, and it is the second horn.**  The one same-shape producer
in the tree, `ft_compact_relocate_compressed`, re-homes the child's back-edge to
the NEW node — `ft_set_parent(ft, cn2->child, cn2_flag, &cn2->child)`,
`ft-compact.h:283` — and only THEN tombstones the old one
(`ft_meta_tombstone_set_flip`, `:299`) and defers its free (`:311`).  Its own
comment says the result plainly: the retired node "is already detached
(unreachable via its child's back-pointer / grandparent slot) and merely
awaiting its grace period, **exactly the transient `ft_skip_reanchor` already
tolerates**."  The other producers record the child edge IN the retire txn —
`ft_reparent_record` (`ft-mutation-node.h:2355`), `ft_record_child_back_edge`
(`ft-remove.h:1783`).

**So the SKIP_X shape §5.3 built its refutations on — a `G` reachable from a
LIVE slot whose `prev` names a retired node AT REST — has no producer that
three readings could name.**  What the corpus does measure, per the WITNESS
skeptic, are stale edges on **orphans not named by any live slot, reachable only
from an application-held node handle across a grace period** — which is exactly
`cds_ft_remove(iter, node)`'s own entry condition.  That is a different defect
in a different place, and it would explain why four dispositions aimed at the
DESCENT have now all failed.

☠ **This does not retire §5.3.**  §5.3's verified half stands: `ft_skip_reanchor`
DOES resolve skip hops by walking up back-edges, so "the forward path is
authoritative" is false as a general rule.  What is now in doubt is the SECOND,
load-bearing half — that the shape is a REST state for a slot-reachable node.
Three readings failing to find a producer is not a proof that none exists.

#### ☞ THE NEXT STEP, and it needs no scope call

Decide the dilemma by MEASUREMENT, not by more reading.  Extend `cds_ft_verify`'s
head back-edge check (`ft-verify.h:342-370`, which already compares
`cell->parent` to the owner) with one predicate: **for every external head and
every compressed node behind a SKIP_X dual, the node its back-edge names is
UNTOMBSTONED and its forward slot resolves back to it.**  Run it under
`-DFEATURE_FT_VERIFY_AT_MUTATION` on the `*_spec` ft_inv tests (`:21182` — the
only place skip mode survives) and on the rig at `WRITERS=1 CHK=1 NOFREE=1`.
Single-writer, so a hit is attributable to the op that just committed, in one
run, with no memcg.  Both outcomes are decisive:

* **a hit** — the at-rest shape is real, the offending producer is NAMED, and
  the cure is at that producer's retire, not in the descent;
* **no hit, ever** — the at-rest shape for slot-reachable nodes is a myth, and
  the whole investigation moves to the application-handle path, where the
  orphan edges actually live.

It is a claim without an arm (no behaviour change), it is single-threaded, and
it is the cheapest thing in this file that can move the question.

##### ☠ AMENDED IMMEDIATELY: THAT FALSIFIER AS WRITTEN IS CIRCULAR

`cds_ft_verify` already carries most of the predicate — `:484` and `:779` check
`parent_word == expected_parent` against a TOP-DOWN walk, `:626` round-trips
`parent_slot_offset`, `:908-926` checks that a slot holding `skip(cn)` is the
very slot `cn` records — **but its skip path cannot see a stale skip edge,
because it REACHES the skipped node through that edge**:
`ft_resolve_skip_compressed` (`ft-helpers.h:2527`) is `ft_skip_to_compressed`,
which reads the child's `prev` / `parent_word` (`:1850-1863`). Every fact the
verifier then asserts about that compressed node is derived from the word it
would have to validate. A walk cannot validate the edge it walks.

★ **And the reason is structural, not an oversight.** `ft-compact.h:278-283`
states it for the skip form: *"the skip pointer addresses the TARGET, so the
child's back-reference redirect IS the lone structural edge publishing the
relocation — a single atomic store, no two-step window."* In a SKIP_X dual
**no forward word names the compressed node at all**; the child's back-edge is
its publish edge, updated atomically by the producer. So for this shape
"the back-edge is stale, trust the forward path instead" is not a repair —
**there is no forward path to be authoritative**, and §5.3's framing needs
restating in those terms.

☞ A non-circular check therefore cannot be a walk. It has to compare each
compressed node's own recorded `skip_slot` against that slot's CURRENT content,
over an enumeration of compressed nodes obtained INDEPENDENTLY of the trie's
edges (an allocator/arena inventory). Whether such an enumeration is available
is the open question, and it is a design question, not a coding one.

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

★ **THE RIG IS IN THE TREE**: `doc/design/ft-stale-disposal-rig.c`.  It is not
built by anything -- build it by hand, and `ldd` it to PROVE it resolved into
your build tree rather than an installed `liburcu`.  Its header documents the
six `CHK=1` writer oracles and every env knob.  ☞ The counts in §3 and §5 were
taken with it; without it in the tree they could not be re-derived.

☠ **`CHK=1` NEEDS `-DFT_ENABLE_TRACING` IN THE LIBRARY.**  The hook, the
`ft_dbg_*` counters and the rm-site stamp are one subsystem behind that flag,
and the rig reaches the hook by a WEAK reference -- so against a default build
it resolves to NULL and every violation is classified as nothing, silently.
MEASURED: a 120-seed sweep against `build-pfxbit` fired 42 writer oracles and
produced ZERO discriminator lines.  The rig now refuses to start rather than
report that zero.  An LTTng session is NOT needed for the stderr classifier --
only the build flag is.

```sh
# the rig itself (the stderr classifier needs no LTTng session, only the flag)
gcc -O0 -g -I<top>/include -I<build>/include -I<top>/src -I<top> \
    doc/design/ft-stale-disposal-rig.c -o /tmp/ftrig \
    -L<build>/src/.libs -lurcu-qsbr -lurcu-cds -lurcu-common \
    -Wl,-rpath,<build>/src/.libs
ldd /tmp/ftrig | grep urcu          # MUST name <build>, not /usr

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
  SECS=5 READERS=4 NOFREE=1 ./ftrig_traced
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
