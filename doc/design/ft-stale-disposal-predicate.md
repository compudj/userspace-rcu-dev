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

### 5.7 ☠☠☠ IT REPRODUCES WITH SKIP-COMPRESSION COMPILED OUT — MORE, NOT LESS

**Mathieu asked the question `CLAUDE.md` opens with and this file had never
answered: does it still reproduce with the feature disabled?**  Nothing in §1-§5.6
records a skip-off arm.  Run 2026-09-05, both arms at the same commit, on the
same box, same 160 seeds, `CHK=1 WRITERS=2 ALPHA=2 MAXLEN=8 NSTABLE=100
NCHURN=100 SECS=5 READERS=4 NOFREE=1`, 8 concurrent, each seed in its own 8G
memcg.  Control `build-sfdisc`; arm `build-sfnoskip`, identical but for
`-DNO_FEATURE_FT_SKIP_COMPRESSED`.  Proven disabled, not assumed:
`nm liburcu-cds.so.8.2.0 | grep -c skip_reanchor` is 1 in the control and **0**
in the arm.

| oracle | skip ON | skip OFF |
|---|---|---|
| **RM-FAIL (defect 2)** | 47 | **74** |
| RM-LOOKUP-MISS (defect 1) | 26 | 28 |
| STALE-FOUND | 41 | 27 |
| RM-WRONG-NODE | 0 | 0 |

rc: control 103x134 / 41x139 / 9x124 / 7x137; arm 124x134 / 29x139 / 4x124 /
3x137.

★ **And it is the SAME population, not a lookalike.**  The `anctomb`
discriminator holds in both arms: RM-FAIL carries `anctomb=-1` in **47/47** and
**74/74**; RM-LOOKUP-MISS carries a tombstoned ancestor in 24/26 and 25/28.

#### What this retires

* ☠ **`ft_skip_reanchor`'s back-edge up-walk is NOT the mechanism of RM-FAIL.**
  The function is not compiled into the arm that produces 74 of them.
* ☠ **The SKIP_X-dual argument does not apply.**  With skip off, the parent's
  slot names the compressed node DIRECTLY — the forward path to the holder
  EXISTS — and the defect fires MORE.  "The holder is unnameable forward" cannot
  be the cause of a failure that is worse when the holder is nameable forward.
* ☠ **The retire-time namer detector of §5.6 is aimed at nothing**, and was
  independently REFUTED on its own terms the same day: its singleton premise is
  false (ordered-list cells, parked proxies and six writer-side ledgers also
  name a `cn`); it false-positives on correct code at `ft-mutation-helpers.h:13851`,
  which tombstones BEFORE the re-home by design (`:13861-13867`), and in the
  top-down orphan loop at `ft-remove.h:692-704`; and `ft-remove.h:4484-4486`
  frees a retired trailing `cn` by resolving it THROUGH the child's back-edge
  after the tombstone, so the proposed invariant is the negation of a
  load-bearing contract.
* ☞ **§5.3 is not wrong, it is IRRELEVANT here.**  The up-walk does read
  back-edges; that remains true and remains a hazard for other lanes.  It is not
  what makes `cds_ft_remove` answer `NOT_FOUND` for a present key.

#### Where the evidence now points

The one thing common to both arms is that the holder is derived from
`node->prev` (`ft-remove.h:5388`, `ft_node_holder`) and never re-validated
before the identity compares.  What survives as a mechanism is a back-edge
naming **a LIVE but WRONG node** — not a retired one: a non-retired sibling
copy, an internal node, a node re-parented into a detached trie
(`ft-detach.h:150`, `:546`), a dst-spine node that "stays reachable via the old
dst spine until the forward publish" (`ft-mutation-helpers.h:13870-13877`), or a
recycled address.  Every one of those mis-anchors identically with skip on or
off, and none of them involves a compressed-node retire.  That is the next
hypothesis, and it is testable the same way this was.

☠ **A caveat on the counts.**  Run-to-run variance is large: the control arm
here reads RM-FAIL 47 / STALE-FOUND 41 where §5.4's corpus, same commit and same
workload, read 81 / 25.  Treat the ABSOLUTE numbers as noisy and the
BETWEEN-ARM comparison and the discriminator ratios as the result.

---

### 5.8 ☑ THE FULL FEATURE MATRIX — RM-FAIL IS INVARIANT

§5.7 answered one feature.  This answers the rest.  Same rig, same workload
(`CHK=1 WRITERS=2 ALPHA=2 MAXLEN=8 NSTABLE=100 NCHURN=100 SECS=5 READERS=4
NOFREE=1`), 160 seeds per arm, 8 concurrent, each seed in its own 8G memcg,
2026-09-05, all arms at this commit on one box.

**Every feature was proven off at the PREPROCESSOR**, not assumed -- a two-line
TU including `fractal-trie-internal.h` with the build's own `CPPFLAGS` and
`config.h`, emitting `#warning PROBE COMPRESS/SKIP/MERGE=ON|OFF`.  Recorded so
the arms can be re-derived:

    build-sfdisc        COMPRESS=ON   SKIP=ON   MERGE=ON    (control)
    build-sfnoskip      COMPRESS=ON   SKIP=OFF  MERGE=ON
    build-sfnocompress  COMPRESS=OFF  SKIP=OFF  MERGE=ON
    build-sfnomerge     COMPRESS=ON   SKIP=ON   MERGE=OFF

The ordered-cell index has no build knob -- it is a per-group runtime flag -- so
that arm is the rig's `LIST=0`.

| arm | what is off | RM-FAIL | RM-LOOKUP-MISS | STALE-FOUND |
|---|---|---|---|---|
| ctrlA | nothing | 60 | 29 | 24 |
| noskip (§5.7) | skip-compression | 74 | 28 | 27 |
| **nocompress** | **path compression entirely** | **64** | 42 | 23 |
| **list0** | **the ordered-list cell indirection** | **61** | 38 | 46 |
| nomerge | the merge subsystem | 71 | 31 | 28 |
| ctrlB ☠ | nothing (50 seeds only) | 25 | 3 | 12 |

★ `anctomb=-1` on RM-FAIL: **281/281 across every arm.**  One population
throughout; no arm is producing a lookalike.

☠ **The trailing bracket control is INCOMPLETE** -- 50 of 160 seeds -- because
the run was killed under system memory pressure (see below).  So the intended
"control at both ends" bound is weaker than planned: ctrlA gives 60 per 160 and
ctrlB's partial rate scales to roughly 80, i.e. a control band of about 60-80.
**Every arm falls inside or beside that band.**  The honest reading is therefore
NOT that some feature slightly raises RM-FAIL, but that **no feature moves it
outside control variance at all.**

#### What this leaves

`nocompress` is the decisive arm: with path compression compiled out there are
**no compressed nodes in the trie at all**, so every holder is a plain internal
node or an external chain -- and the defect fires 64 times.  `list0` removes the
cell indirection from `ft_node_holder` (a head's `prev` is then the owner
directly) -- 61.

**So the defect lives in the PLAINEST configuration the structure has**: an
external head whose `prev` names an internal owner directly, no compression, no
skip encoding, no cell, and the identity compare at the body arm fails anyway.
Every mechanism this document has proposed -- the up-walk, the SKIP_X dual, the
compressed-holder arm, the retire-time namer -- needs a feature that can be
switched off without touching the defect.

☞ What survives is exactly one sentence: **`ft_node_holder(node)` at
`ft-remove.h:5388` returns a LIVE but WRONG internal node, and nothing
re-validates it before the compare.**  That is now a small enough claim to hunt
directly, and -- because it needs none of the optional structure -- a MINIMAL,
possibly single-threaded reproducer should exist.  That is the next step.

#### ☠ An operational note, because it cost the bracket control

`/tmp` on this box is **tmpfs, i.e. RAM**.  It was holding ~90 GB: the gate's
per-config source trees (~40 GB, ~16 configs at ~2.4-2.7 GB each, accumulated
across sessions) plus assorted session scratch.  Add 8 concurrent `NOFREE=1`
rigs -- which LEAK retired nodes by design -- at an 8 GB cap each, and an
unrelated 24 GB VM, and the box shed background jobs mid-arm.  The memcg cage
did its job per seed; what was unbounded was the SUM.  Two follow-ups for any
future sweep: bound `concurrency * cap` against free RAM *minus the tmpfs*, and
`systemctl --user reset-failed` afterwards -- 405 failed transient scopes had
accumulated, one per aborting seed.

---

### 5.9 ★★★ COARSE WRITER STRATEGY IS TOTALLY CLEAN — EVERY ORACLE IS A WRITER-WRITER RACE

§5.7 and §5.8 switched off the structural FEATURES and nothing moved.  This
switches off writer CONCURRENCY, and everything moves at once.

`CDS_FT_WRITER_LOCK_COARSE` serialises every structural writer on the one
FT-wide lock; readers stay fully concurrent.  Selected through the SUPPORTED
attribute `cds_ft_group_attr_set_writer_strategy` (the rig gained a `WRITER=`
knob and now echoes an `ARM writer=... spacing_env=...` line so an arm cannot be
mislabelled).  160 seeds per arm, same rig binary, same seeds, same workload,
same concurrency (4) and same 6G cap -- the ONLY difference is the strategy.

| arm | seeds | RM-FAIL | RM-LOOKUP-MISS | STALE-FOUND | exit codes |
|---|---|---|---|---|---|
| fine (per-node, default) | 160 | 48 | 27 | 28 | 96 abort, **42 SEGV**, 10 memcg, 10 timeout, 2 wrongid |
| **coarse** | 160 | **0** | **0** | **0** | **160 x rc=0** |

★ **NOT A WRONG ZERO.**  The coarse arm ran the oracles armed (`CHK=1`; the rig
refuses to start when the classifier hook is absent) and completed
**156,657,174 churn operations and 6.33 billion reads** across its 160 runs,
every one reaching its final line.  The fine arm reports no `churn_ops` at all
because its runs abort before the summary -- which is itself the contrast.

#### What this establishes

**Every oracle in this rig is a WRITER-WRITER race.**  Serialising writers
removes not only RM-FAIL but the 42 SEGVs, the 10 memcg kills and the 10
timeouts -- the whole rc=139 lane §5.4 left unaccounted, and the liveness lane
with it.  Readers remained concurrent throughout (6.33 billion of them), so
reader/writer concurrency alone does NOT produce any of it.  **Two concurrent
STRUCTURAL WRITERS are necessary.**

☠ **This REFUTES the closing suggestion of §5.8.**  That section ended by
proposing a "minimal, possibly SINGLE-THREADED reproducer", on the reasoning
that the defect needs no optional structure.  It needs no optional structure
AND it needs two writers: those are independent axes, and I conflated
"structurally simple" with "reachable with one writer".  A single-threaded
`ft_unit` case cannot reach this.  The cheap reproducer to build is a
TWO-WRITER one, and the rig already is it.

☞ So the target narrows to the intersection: `ft_node_holder(node)`
(`ft-remove.h:5388`) reads `node->prev` and the body arm compares against it,
in a trie with NO optional structure, while a PEER STRUCTURAL WRITER is running
-- and nothing in the fine-grained path gives that read the exclusion the coarse
lock supplies for free.

#### ☠ The coarser lock SPACINGS could not be measured at all

`CDS_FT_LOCK_SPACING_EXPONENTIAL` and `_ROOT_ONLY` are REFUSED by
`cds_ft_group_attr_set_lock_spacing` by design (`fractal-trie.h:2952-2956`:
"Anchoring is ALL-OR-NOTHING ... rather than offered as a setting that silently
excludes nothing").  Reached anyway through the `CDS_FT_LOCK_SPACING` env back
door on a `-DFEATURE_FT_LOCK_SPACING_ENV` build, BOTH **SEGV within two seconds**
on a 20-key workload where per-node completes 567k ops.  So no RM-FAIL
statistics exist for those spacings; the configuration dies first.

★ The crash is worth recording because of WHERE it lands:

    #0 cds_ft_item_to_metadata   fractal-trie-internal.h:3985
    #1 ft_detach_node            ft-remove.h:3437
    #2 _cds_ft_remove_locked     ft-remove.h:6003   <-- the BODY ARM
    #3 cds_ft_remove             ft-remove.h:6205

`:6003` is the detach in the same body arm whose identity compare at `:5976`
produces RM-FAIL.  Under root-only that arm dereferences garbage instead of
answering NOT_FOUND.  ☠ Treat this as a LEAD, not evidence: the configuration is
one the library itself refuses as incomplete, so a crash there is expected and
says nothing directly about the supported per-node path.

---

### 5.10 ★★★★★ ROOT CAUSE — A COARSE-LOCK PREMISE LEFT BEHIND BY THE FINE-LOCKING MIGRATION

**In one sentence:** `_cds_ft_remove_locked` selects its arm from TWO UNLOCKED
READS TAKEN IN DIFFERENT EPOCHS -- `node->prev` at `ft-remove.h:5388` and the
holder's key-selected body slot at `:5972` -- under a comment that still asserts
the FT-wide writer mutex is held; under fine locking a peer's IN-PLACE
leaf-to-prefix-head conversion flips BOTH words between those two reads while
retiring NOTHING, so the identity compare at `:5977` fails and the op answers
`NOT_FOUND` for a key that is present throughout.

#### The premise, still in the source

`ft-remove.h:5354-5359`:

> *"No top-down descent.  @node is application-owned and, with the RCU
> read-side lock held continuously since it was obtained, stays alive;
> **the writer mutex held here freezes the structure**, so node->prev is a
> settled live pointer to the node's holder and the slot that holds @node can
> be derived directly"*

That was TRUE under `CDS_FT_WRITER_LOCK_COARSE`.  It is FALSE under the
fine-grained DLM default, and the whole no-descent derivation rests on it.
`ft_excl_writer_enter` (`fractal-trie-internal.h:3623-3660`) only bumps a
counter when `lock_fine`; `ft_writer_lock_scope_enter` returns early at
`:2735-2741` for `lock_fine` and reaches `cds_fair_mutex_lock(&ft->writer_lock)`
only otherwise.  Between `:5388` and `:5977` the op holds **no word at all** --
the per-node acquire happens inside `ft_detach_node`, AFTER the compare.

#### The interleaving

Victim key K owned by writer A; peer writer B owns K' = K + one more byte;
H is K's holder with K in `H.body[b]`, b = K's last byte.

1. B inserts K'.  Its descent breaks on the external leaf K with key bytes
   remaining (`ft-insert.h:3064-3065`) and takes the "transform this external
   node into an internal node with associated external node" branch (`:3366-3386`)
   -> `ft_attach_node(..., &H.body[b], K, ..., external_nodes = K)`.
2. B builds the new internal N invisibly with `N.external_nodes = K`
   (`ft-insert.h:1943-1944`).  The publish is IN PLACE on the LIVE H: for a
   displaced external no set_nth/reserve runs (`:2020-2022`); the slot edge
   `H.body[b]: K -> N` is recorded under H's state word alone (`:2099-2113`),
   and K's back-edge `cell->parent`/`prev`: `H -> N` is parked into the SAME
   one-commit (`:2136-2141`, "must flip ATOMICALLY with the forward publish").
   **Nothing is retired.  H keeps its identity and its tier.**
3. A, still in the read section of its successful lookup, reads
   `ft_node_holder(node)` at `:5388` BEFORE B's flip -> `holder_flag = H`.
   `:5418` H is live, so no tombstone re-descent.  `:5434` per-node, so no
   depth descent.  `ft_lock_ctx_init` at `:5535` acquires NOTHING.
   `:5671-5705` does a cell capture and a `ft_flip_txn_create_bounded` malloc,
   widening the window.
4. B's MCAS commits: `H.body[b] = N` and `K.prev -> N`, atomically.
5. A reaches the body arm, reads `ft_node_get_nth_skip(H, key[len-1])` at
   `:5972` -- now N -- resolves the proxy, and `ft_node_ptr(N) != node` at
   `:5977` -> `rmsite=5993`, `CDS_FT_STATUS_NOT_FOUND`.

#### FOUR independent lines of evidence

1. **The key-length signature.**  Across **450** RM-FAIL samples the victim key
   is NEVER of length MAXLEN (8):

   | victim key length | RM-FAIL | RM-LOOKUP-MISS (sibling defect, SAME runs, same key pool) |
   |---|---|---|
   | 5 | 49 | 2 |
   | 6 | 235 | 33 |
   | 7 | 166 | 40 |
   | **8 = MAXLEN** | **0** | **95 (its LARGEST bucket)** |

   A MAXLEN key cannot be EXTENDED, so it can never be converted into a prefix
   head.  Length-8 keys are not rare -- they dominate the sibling defect in the
   very same runs.  This is a PREDICTION of the mechanism, not a fit.
2. **The `PFX=1` falsifier.**  With every churn key a proper prefix of a
   permanent stable key, the path node always exists and the conversion cannot
   occur, while two writers still churn under shared ancestors:
   **0 / 48 seeds, all rc=0, 87.6 M churn ops** (1.83 M per run -- MORE work
   than the control, so not a wrong zero) versus **20 / 48** in the control
   arm at the same seeds.
3. **The coarse arm** (§5.9): 0/160 over 156 M churn ops -- the exclusion the
   premise assumes, restored.
4. **The site stamp**: `rmsite=5993` (the body arm) in 324/329 classified
   samples, `rmsite=5757` (its compressed twin) in the other 5.

#### Why `anctomb = -1` in 281/281

The conversion is an in-place slot replacement on the LIVE holder plus one NEW
node.  Nothing on K's path is retired, so no ancestor is ever tombstoned -- and
every tombstone-guarded recovery arm (`:5418`, `:5486-5508`) is bypassed by
construction.  ☞ This also corrects `ft-remove.h:5552-5555`, whose
`FT_RM_ACQUIRE_FIRST` rationale assumes the re-homer is "a holder copier" that
"retires H in that same commit".  It is not, and `anctomb=-1` always said so.

☠ **My own prime suspect was WRONG.**  I briefed tier promotion / holder COW as
the likely edit.  It is impossible in this workload: the holder tag nibble is 1
(smallest tier) in 329/329 samples, and with `ALPHA=2` an internal node never
exceeds two body children.  The evidence eliminated my hypothesis, not the
hypothesis the evidence.

#### Confidence

**High for the forward mechanism** (leaf -> prefix-head conversion), which the
length signature and the `PFX=1` arm both single out.  **Moderate** for the
mirror image -- a peer REMOVING K', collapsing N and hoisting K back into
`H.body[b]` via `pub->head_parent_field` (`ft-mutation-node.h:940-968`) -- as
the account of the 46/329 samples where `headis` is `other`/`null` rather than
`node`.  Cheapest settling probe: stamp `holder_flag` and `*head_slot` beside
`ft_dbg_rm_site` at `:5977`.

☞ **THE SHAPE OF ANY FIX.**  The defect is not a missing validation to bolt on;
it is a derivation whose stated precondition no longer holds.  Either the two
reads must be made in ONE epoch (hold the holder's word across derive -> select
-> compare), or the derivation must stop claiming a frozen structure.  Note
that the previously-refuted dispositions A-E were all attempts to patch the
CONSEQUENCE; this is the first statement of the CAUSE.

---

### 5.11 ☑ THE MINIMAL OPERATION SET — TWO WRITERS, INSERT + REMOVE, NOTHING ELSE

Which concurrent operations are actually required?  80 seeds per arm, same
build and workload as §5.9 (`CHK=1 ALPHA=2 MAXLEN=8 NSTABLE=100 NCHURN=100
SECS=3 NOFREE=1`), one variable changed at a time.

★ **No bulk operation is involved at all**: the rig issues NONE (`grep -c bulk`
= 0).  Its entire API surface is `cds_ft_insert`, `cds_ft_lookup` /
`cds_ft_iter_*`, `cds_ft_remove` on the writers, and `cds_ft_eager_lookup_key`
plus a 1-in-16 ordered walk (`cds_ft_lookup_first` / `cds_ft_next`) on the
readers.

| arm | configuration | RM-FAIL | RM-LOOKUP-MISS | STALE-FOUND | rc |
|---|---|---|---|---|---|
| A | 2 writers, 4 readers, walk | 23 | 9 | 20 | 48 abort, 25 SEGV, 4 memcg, 3 timeout |
| **B** | **2 writers, NO readers** | **35** | 15 | 13 | 58 abort, 15 SEGV, 7 timeout |
| C | 2 writers, 4 readers, NO walk | 34 | 14 | 11 | 53 abort, 23 SEGV |
| **D** | **1 writer**, 4 readers, walk | **0** | **0** | **0** | **80 x rc=0** |

**THE MINIMAL SET IS TWO CONCURRENT WRITERS DOING ONLY INSERT AND REMOVE, ON
DISJOINT KEYS.**

* **Readers are not needed.**  Arm B removes them entirely and reproduces MORE
  (35 vs 23) -- readers were merely slowing the writers down.  This also
  re-confirms §5.9 from the other side: the defect is not reader/writer.
* **The ordered walk is not needed** (arm C).
* **A bulk op is not needed**, and cannot be: none is issued.
* **One writer is clean**, 80/80 rc=0, with readers and walks still running.

☞ **WHY THIS MATTERS FOR THE FIX.**  A targeted regression test needs no
reader threads, no ordered list traffic, no bulk lane and no third operation:
two threads, `insert` then `remove` of their own partitioned keys, is the whole
reproducer.  Combined with §5.10's key-length signature -- the victim key must
be EXTENDABLE, i.e. shorter than MAXLEN -- and the `PFX=1` result, a
deterministic two-thread unit case is now specifiable: writer B inserts a
proper EXTENSION of the key writer A is removing.

---

### 5.12 ★★★★★ THE INTERLEAVING, CAPTURED — LTTng FLIGHT RECORDER

§5.10 derived the mechanism from code plus statistics.  This is the mechanism
OBSERVED, in one window, with nanosecond timestamps and per-CPU attribution.

**Setup** (the standing methodology): `lttng create ftrd --snapshot`, one
userspace channel at `--subbuf-size=64K --num-subbuf=4`, and events enabled BY
HYPOTHESIS rather than `cds_ft:*` -- `remove_enter`, `attach_node_enter`,
`tree_edge_set`, `set_parent`, `metadata_set_external_nodes`, `ext_violation`
(all six confirmed enabled; a typo enables nothing silently).  The violation
path already freezes, stops and snapshots before aborting (`ft_trace_capture`,
`fractal-trie-trace.h:90`), so the window survives.  Reproducer: the §5.11 split
arm -- `CHK=1 ROLES=1 WRITERS=2 READERS=0 ORD=0 ALPHA=2 MAXLEN=8 NSTABLE=100
NCHURN=100 SECS=3 NOFREE=1 SEED=13`.  8,170 events captured.

**The window.**  Victim `K = 0x7FA41C00D5C0` (key `"ababab"`, 6 bytes);
old holder `H = 0x7FA42660BB21`; the fresh internal node `N = 0x7FA42660BBA1`;
the peer's extending key `K' = 0x7FA41C00D620`.  Times are `14:27:45.618...`.

| time | cpu | event |
|---|---|---|
| `083135` | 251 | `set_parent` child=**K** parent=**H** |
| `083345` | 251 | `tree_edge_set` parent=**H** key_byte=`98 'b'` -- K sits in `H.body['b']` |
| `090716` | 251 | **`attach_node_enter`** attach=**H** old_node=**K** level=6 -- the leaf->internal conversion |
| `091177` | 251 | `set_parent` child=**K'** parent=**N** |
| `091267` | 251 | `tree_edge_set` parent=**N** key_byte=`97 'a'` -- K' installed in N |
| `091387` | 251 | **`metadata_set_external_nodes`** node=**N** external_nodes=**K** -- **K becomes N's PREFIX HEAD** |
| `091547` | 251 | `set_parent` child=**N** parent=**H** |
| **`092118`** | **71** | **`remove_enter`** key_len=6 key=`61 62 61 62 61 62` -- THE REMOVER ENTERS |
| **`092128`** | **251** | **`tree_edge_set` parent=H key_byte=`98 'b'`** -> **`H.body['b'] = N`** |
| `105999` | 71 | `ext_violation` node=**K** holder=**N** hext=**K** rmsite=6090 **anctomb=-1** |

★★ **THE WINDOW IS TEN NANOSECONDS.**  The remover enters
`_cds_ft_remove_locked` at `092118`; the peer's forward publish into the very
slot it is about to read lands at `092128`.  The two-epoch gap of §5.10 is no
longer an inference.

**Every prediction confirmed, independently of the statistics:**

* the peer's edit is `ft_attach_node` on the victim's own leaf
  (`attach_node_enter old_node = K`) -- the conversion §5.10 named;
* the inserted key is a ONE-BYTE EXTENSION of the victim (`K'` under N at byte
  `'a'`, i.e. `"abababa"` extending `"ababab"`), which is why a MAXLEN victim
  can never be hit (§5.10's key-length signature, 0 of 450);
* `metadata_set_external_nodes(N, K)` is the in-place conversion of the victim
  into a prefix head;
* H keeps its identity and NOTHING is retired -- hence `anctomb=-1`, and hence
  every tombstone-guarded recovery arm is bypassed;
* at the abort a fresh holder read returns N, whose `external_nodes` IS the
  victim: `hext == node`.

☞ The trace was read the way the methodology prescribes: the violation is the
last event; the events immediately before it are on a DIFFERENT cpu (251 vs
71), which is the cross-thread interleaving no static reading could show; and
grepping by the victim's and the holder's addresses across all CPUs in time
order reconstructs the whole conversion.

---

### 5.13 ☠☠☠ THE SELECTION-WINDOW DESIGN — REFUTED 3/3, AND WHAT THE THREE AGREE ON

The seventh disposition.  Designed after §5.12, put to three adversarial
skeptics (exclusion / termination+peer-progress / arm-side+blast-radius).
**All three refuted.**  Do not build it.

**The design.**  A bounded SELECTION WINDOW in `_cds_ft_remove_locked`: derive
candidate holder H from `node->prev` (or by descent when that holder is
tombstoned), take H's own state word via `ft_acquire_member`, do ONLY LOADS
under it (re-check removed, select the arm, read H's key-selected word, compare
to `@node`), release with a bare CAS BEFORE any allocation / txn / arm /
commit, never publish into `@lctx`.  Then thread the decided value into the
arm's slot EXPECTED-OLD so a later peer move is an ABORT, not silent adoption.

#### ★★★ THE THREE CONVERGENT FINDINGS — these outlive the design

**1. `node->prev` CANNOT be a derivation source in any loop that also
descends.**  The file's own measured fact, `ft-remove.h:5403-5407`: a
back-pointer is updated LAZILY, so a peer that replaced the holder leaves
`node->prev` naming the RETIRED one while the forward path resolves `@node`
correctly -- **78%** of tombstone-arm entries.  Any design that derives from
`prev` and validates/re-aims by DESCENT alternates between the two answers.
Disposition B died on exactly this; F plausibly did; and this design does too:
its lap T fires on "the fresh prev is dead -> re-descend", so descent yields
live H', the hold reads `prev` = dead H, T fires again -- **forever, every lap
charged to the SAME retire commit.**  ☞ THE RULE: derive by ONE authority, and
never consult the other inside the same loop.

**2. ☠☠ REMOVE CANNOT USE `need_retry` AS A CONVERGENCE MECHANISM AT ALL.**
The FIFO-lane argument that every retry-based disposition here has leaned on is
CIRCULAR for this op.  `urcu_txn_conflict` only bumps `retry`
(`rcu-txn.h:1239-1243`); the lane turn is KEPT only while `retrying == 1`
(`:1225-1236`, `urcu_txn_end:1272-1273`), which is cleared at every `begin`
(`:822`) and set only by commit paths of the txn's OWN handle.  **Remove's
commits are all on standalone flip txns** -- there is no `create_on` /
`create_bounded_on` anywhere in `ft-remove.h` -- so `optxn->retrying` is NEVER
1, and every lap runs `exit_fallback` (`:784-791`), clears `domain->active`,
unlocks the fair mutex and RE-QUEUES AT THE TAIL.  That is disposition A's
measured failure (`:5255-5259`, "the failure is wall-clock and RSS, not attempt
count") explained structurally.

☞ ☠ **CORRECTED, by reading `rcu-txn.h` directly rather than trusting the
skeptic's phrasing.**  The forfeit is DELIBERATE, and `rcu-txn.h:1225-1235`
states the intent: *"Why a pre-commit retry must not keep the turn.  @retrying
exists so that [a commit abort keeps the turn] ... begin() clears @retrying, and
only the [commit path] keeps the turn ... while an attempt that never reached a
commit [forfeits]."*  So the rule is NOT "remove can never keep its turn":

* **A PRE-COMMIT BAIL FORFEITS THE TURN BY DESIGN; A COMMIT ABORT KEEPS IT.**
* Remove's retries are ALL pre-commit bails, which is why A, F and the window
  died the same way.
* ☠☠ **AND THAT IS STILL WRONG -- corrected a SECOND time, verified from the
  code.**  `retrying` is PER-HANDLE and set ONLY inside that handle's own
  commit (`urcu_txn_commit_flavor` / `_commit` / `_commit_sw_flavor`,
  `rcu-txn.h:1147,1156,1201,1210`).  **Remove's `optxn` is NEVER COMMITTED** --
  it receives only `begin`, `urcu_txn_conflict` (which sets `retry++`, NOT
  `retrying`, `:1239-1243`) and `end`; every actual commit runs on a private
  flip-txn handle.  So a COMMIT ABORT does not keep remove's turn either:
  **every lap forfeits, wherever the failure is detected.**
* ☞ The escape therefore needs an EXPLICIT mechanism -- something that sets
  `op->retrying = 1` on the coordination handle -- not a change of where the
  failure is noticed.  ★ I got this wrong twice: first by repeating a skeptic's
  phrasing unverified, then by over-correcting it.  The rule is simply: **remove
  cannot keep its FIFO turn today by any route.**
* ☠ But BINDING is not that route: `ft-mutation-helpers.h:5831-5836` records it
  MEASURED WORSE -- "median 9.5 starving removes against 4 for aging alone" --
  because binding shares the op's descriptor and install lane.

This retires the termination story of A, F and this design.  It does NOT point
the eighth at the commit -- it says any eighth must ADD a keep-turn mechanism,
or terminate without needing the turn.

**3. THE WINDOW NARROWS, IT DOES NOT CLOSE.**  The peer's conversion needs H's
word only AT COMMIT (`ft-insert.h:2099-2102`, all-or-none
`ft-mutation-helpers.h:7967-7984`), so it is refused during the window and
succeeds THE INSTANT THE RELEASE LANDS -- and the arm acts after that release.
The design's answer was the expected-old threading, and that is refuted
separately (below).  A decision made under a lock that is dropped before the
act is only as good as the act's own validation.

#### The per-skeptic refutations

* **Exclusion (P1/P2).**  P1 (every writer of H's slots takes or guards H's
  word) SURVIVED for H's body slots under enumeration.  **P2 is FALSE as a code
  invariant** -- the recompact UNFENCED arm (`ft-mutation-node.h:1417-1423`)
  fences only when `retire_txn && !cluster_leaf` and plain-stores live
  children's back-edges (`ft-helpers.h:3191/3193`, `:1574/1576`); "copies only
  build-invisible nodes" is asserted in comments (`:2098-2100`), never enforced.
  ☠ And the gate `!ft_node_external(cand)` **leaves a second copy of the defect
  unfixed**: the chain arm at `ft-remove.h:4974-5008` derives
  `ft_chain_head_holder` (an unlocked prev walk), acquires it at `:5005`, and
  commits at `:5050-5062` with NO re-derivation -- while
  `ft-mutation-helpers.h:14748` already does re-check
  `ft_chain_head_holder(...) != hf` after acquiring.  The site's own comment
  (`:4983-4985`) calls the result a LOST UPDATE.
  ☠ The BARE RELEASE is unsafe: `ft_dlm_acquire_set_at` DEDUPES when the ctx
  already holds H (`:5933-5945`, `held.shared = true`, nothing taken), so a bare
  release clears a bit owned by an outer frame.  `ft_unchain_node` guards this
  (`ft-remove.h:5016`, `if (!h.shared)`); the design does not.
* **Termination / peer progress.**  Beyond finding 1: lap T's premise ("unlink
  and tombstone are one commit") is FALSE for at least four retire paths, and
  `ft-compact.h:299` places the tombstone **explicitly AFTER the unlink** -- so
  there is a window where a node is unlinked but its state word is CLEAN, the
  acquire SUCCEEDS, `fresh == cand`, the dead node's slot still names `@node`,
  and the compare PASSES.  **The window then makes a confident decision against
  an unreachable holder** -- worse than a lap.  Peer progress is degraded:
  clean->dirty->clean transitions per remove go 2 -> 4, window-vs-window
  refusals between two removes are new in kind, and the peer site F measured
  being refused (`ft_insert_dlm_acquire_split`) is the TURN-FORFEITING one.
  Cost is understated: a descriptor alloc + full single-record MCAS + deferred
  free + CAS per remove, where `ft-mutation-helpers.h:5831-5836` records that
  binding "measured WORSE".  ☠ And the hold does not freeze what it claims:
  `ft-mutation-helpers.h:13700-13708` states "THE FENCE DOES NOT STOP A PEER
  RETIRE".
* **Arm-side / blast radius.**  The expected-old threading reaches only
  `_ft_node_replace_ptr` (`ft-mutation-node.h:963`, `:1051`), which **the
  COMPRESSED-HOLDER ARM NEVER CALLS**: that arm commits via
  `ft_detach_node_replace_compressed_parent`, which validates the GRANDPARENT's
  word and never `cn->child` (`:102`).  So a conversion between decision and
  commit is SILENTLY ADOPTED there -- `cn` is dropped with the peer's new
  prefix head inside it, which is KEY LOSS, worse than the NOT_FOUND being
  cured.  The "no-op for other callers" claim is also false: `pub->old_val` at
  `:963` is a LATER load than the one `:2746` pins, and the new `-EAGAIN`
  becomes `CDS_FT_STATUS_MEMORY_ERROR` in `ft-detach.h:445-457` and `-ENOMEM`
  in `ft-merge.h:189-196`.

#### ★ WHAT SURVIVED — do not re-litigate

* **P1 for H's body slots**: every enumerated writer of `cn->child`,
  `external_nodes` and an internal node's body slot routes through a recorded
  edge owned by that node, or is a same-value republish.
* **The anchor**: under PER_NODE, H's own word IS the word a peer must take to
  write `H.body[b]` (`ft-mutation-helpers.h:644-645`).
* **The seam rule**: `ft-remove.h` contains no `ft_writer_lock_gp_wait` call at
  all, and the window holds no word across a GP.
* **No ledger residue**: `ft_meta_lock_release` calls `ft_hold_trace_drop`, so
  the op-init LEAK CANARY has nothing to catch.
* **C3's holder pick does NOT cause a wrong-word detach**: `d.depth` means the
  same at both sites, `d.nf` is never skip-encoded there, and every arm
  identity-checks `@node` before touching a word (`:5870`, `:5919`, `:6106`).

#### ☞ THREE INDEPENDENT DEFECTS SURFACED, worth their own work

1. **The chain arm** (`ft-remove.h:4974-5062`) has the SAME unlocked derivation
   with no re-validation, and the correct pattern already exists at
   `ft-mutation-helpers.h:14748`.
2. **`ft-compact.h:299` unlinks before tombstoning**, so a node can be
   unreachable with a CLEAN state word -- which defeats any acquire-based
   validation, not just this one.
3. **`ft-remove.h:5429` refuses on a TRANSIENT** (the forward holder retired
   between the descent's read and the test) -- a contract violation that exists
   TODAY, independent of any fix.  And `d.skip_conflict` still has ZERO
   consumers in `ft-remove.h` while the descent sets it to mean "a mutating
   caller must re-descend".

---

### 5.14 ☑ THE FAST REPRODUCER — TWO KEYS, TWO THREADS, 253 ms

Seven dispositions have died partly for want of a cheap falsifier: the only
reproducer was statistical (80 seeds x 5 s, 53 hits), and its failures were
entangled with SEGVs and memcg kills.  §5.10 and §5.12 specify a much smaller
one, so the rig gained `DETERM=1`:

    churn[0] = "ab"    the VICTIM,    owned by writer 1 (insert / lookup / remove)
    churn[1] = "aba"   the EXTENSION, owned by writer 0 (insert / remove)

Two writers, ONE key each, no readers, no ordered walk, no bulk lane, 10 stable
keys.  Ownership is intact (one key per writer), so unlike `ROLES=1` **every
oracle stays armed**.

| reproducer | hit rate | time per attempt | trie |
|---|---|---|---|
| the §5.4 corpus | 53 / 80 seeds | ~5 s | 200 keys |
| **`DETERM=1`** | **13 / 20** | **253 ms mean** | **2 churn keys** |

Same hit rate (66% vs 65%), **20x faster**, on a trie small enough to reason
about.  Five runs give >99% detection in ~1.3 s.  The signature is the expected
one: `RM-FAIL writer1 key=ab`, `rmsite=6146` (the body arm), `anctomb=-1`,
`headis=node`, `hext == node`.

☞ It is FAST-PROBABILISTIC, not deterministic.  Do not put it in `ft_unit` /
`ft_inv` as-is; it would be flaky.  It is a FALSIFIER for fix work, and it is
the shape the eventual regression test should take once a fix makes it green.

★ A second oracle fires here too: `RM-FAIL writer0 key=aba`, i.e. the EXTENSION
key is also a victim -- inserting `"ab"` while `"aba"` exists creates the
junction the same way.  The defect is symmetric in the pair.

#### ☠ AND A NEGATIVE RESULT: DELAY INJECTION MAKES IT WORSE

A `ft_delay_writer()` hook was added at the epoch boundary in
`_cds_ft_remove_locked` (byte-neutral without `-DFT_DELAY_INJECT`), on the
theory that widening a 10 ns window would make the case deterministic.  It does
the opposite:

    no delay        13/20      FT_DELAY_US=5     5/10
    FT_DELAY_US=1    6/10      FT_DELAY_US=20    5/10
                              FT_DELAY_US=200    3/5

**The race is SELF-HEALING under a long enough delay**: the peer's insert of the
extension AND its own removal both fit inside the sleep, so the holder's slot is
back to `@node` by the time the compare runs.  The hook is kept with these
numbers at the site so the next reader does not repeat the experiment.

---

### 5.15 ☠☠☠ THE EIGHTH DISPOSITION — "INTENT-CARRIED REMOVE", REFUTED 3/3

Designed after §5.14, three adversarial skeptics (intent equivalence /
keep-turn / refusal predicate + fold steady state).  **All three refuted.**

**The design.**  The derivation stops producing a VERDICT and produces an
INTENT `{holder H, slot S, expected raw value V}`; nothing is held while
deciding; each arm RE-ASSERTS `S == V` where it already holds S's owner; the
only refusal is `@node`'s own removed mark; every other mismatch is a lap
re-derived BY DESCENT.  Plus a new engine primitive `urcu_txn_keep_turn()` to
retain the FIFO turn on value laps.

#### ★★★ TWO SKEPTICS INDEPENDENTLY WALKED IT INTO THE SAME RECORDED WEDGE

Its new terminal after `ft-remove.h:3368` --
`if (freeze_leaf && walk_nf != flag-of(freeze_leaf)) -EAGAIN` -- is a
**PERMANENT `-EAGAIN` on every skip-compressed leaf**.  `walk_nf` carries the
skip LENGTH in the high bits (`ft_node_skip_compressed`, `ft-helpers.h:1774`)
while `flag-of(freeze_leaf)` is the bare external pointer, and
`ft_node_external` tests only the LOW tag bits (`:615-618`) -- the very
distinction `ft-remove.h:2295-2298` spells out.  Deterministic shape, so
re-derivation reproduces it forever.

☞ **AND THE FILE ALREADY RECORDS THIS EXACT CLASS**, `ft-remove.h:3584-3592`:
*"A raw slot value and a get_nth result are not the same encoding, so one never
compares equal ... measured as a PERMANENT -EAGAIN, ft_unit wedged at test 2."*
★ THE DURABLE RULE: **any new comparison in this file must state which
ENCODING each side is in.**  Raw slot word, accessor result and bare pointer
are three different things here, and mixing them is a measured wedge, not a
theoretical one.

#### The three refutations

* **Intent equivalence — REFUTED.**  The claim is only as good as its
  placements, and (a) two of them are under NO hold at all: `:2291-2321` and
  `:2404-2406` sit where `:2397` says *"Nothing is built, locked or reserved
  yet"*, so they cannot "speak for the commit" by the design's own logic; (b)
  it enumerated three `ft_flip_txn_arm_per_op` sites and there are SEVEN
  (`ft-remove.h:1855, 4021, 4204, 4352, 4835, 4920, 5188`); (c) a WHOLE LANE
  has no placement -- the compressed-boundary lane `:2763-3089` with its own
  orphan walk (`:2794-2979`, not the one at `:3133`); (d) the support bullet
  "only hlist/cell/root edges stay MW" is incomplete --
  `ft-mutation-helpers.h:8574-8592` names **the unheld SKIP_X DUAL**, the
  reader-facing copy of `cn->child` owned by the GRANDPARENT, which the promote
  arms never acquire (`ft-remove.h:4753-4760`).
  ☠ And the precedent it leaned on is weaker than claimed:
  `ft_chain_compress_plan_stale` compares the op's NAMED node -- `:872-875`
  *"Never a value read speculatively from the trie"* -- whereas the design's V
  IS a speculative raw slot word.  That precedent's own header (`:904-912`)
  records the raw-vs-accessor compare as a permanent `-EAGAIN`.
  ☠ The `ft-mutation-helpers.h:14748` precedent is also misread: it sits AFTER
  `if (sh.shared) continue;`, so it SKIPS the re-check on a shared hold.
  Copied literally, the re-assert is silently skipped exactly when the word is
  deduped against a caller's txn.
* **The keep-turn primitive — REFUTED.**  It contradicts the field's contract
  by construction (`rcu-txn.h:319`, *"a COMMIT aborted and asks to re-attempt"*,
  justified at `:1226-1228` because the op "re-runs a plan it already carried
  all the way to install"; remove's `optxn` never reaches install, so every
  kept lap is the re-descent `:1228-1231` forbids).  Its gate is INCOMPLETE:
  three zero-refusal lap classes mean "a peer holds", not "a value moved" --
  the acquire set's own lost commit returns `-EAGAIN` directly, BYPASSING the
  `eagain:` label (`ft-mutation-helpers.h:6149-6155` vs `:6236`); a
  `ft_lock_ctx_depth_of` miss sets `acquire_miss` with no acquire attempted
  (`:7905-7913`) and is shape-determined for remove, which passes
  `have_descent ? &d : NULL` = NULL (`ft-remove.h:5539`); and guard-validation
  failures abort on the flip handle.  All collapse into one arm at `:6257-6267`,
  and `lap_reason` does not exist -- `need_retry` is a bare bool.  Peer
  starvation is NOT bounded by W: with `fb_published == 0` the kept turn holds
  the fair mutex while `active == 0`, so fresh peers never park and the one
  self-qualified peer waits up to `fallback_at` (64..4096) attempts.  And it is
  unmeasured, where the closest measurement (the superset) was negative.
* **The refusal predicate — REFUTED.**  "Marked ⇔ absent, one linearization
  point" is false.  `ft_node_mark_removed_flip`
  (`ft-mutation-helpers.h:8110-8132`) is a bare `uatomic_cmpxchg` whose own
  header says *"a standalone mark is the bridge"* (`:8100-8102`), and
  `_cds_ft_remove_all_locked` marks AFTER a separately committed unlink at
  `ft-remove.h:6705, :6882, :6960, :6994`, as does `ft_detach_node`'s fallback
  at `:4617-4618`.  So a leaf can be unreachable AND unmarked.  Also `-EDOM`
  reaches `default: abort()` at `:6248-6271`, so an "everything else is a lap"
  catch-all would turn a deliberately terminal refusal into a spin.

#### ★★ WHAT THIS ROUND ADDED THAT OUTLIVES THE DESIGN

1. **`cn->child` AS A RECORDED EDGE WAS DECLINED ON A FALSE BASIS** -- so it
   is still OPEN, and may be the right closure.  Two of the three grounds are
   wrong: the `ft-remove.h:138` "@cn IS A NODE THIS VERY COMMIT DESTROYS"
   hazard is about a REKEY's graft SPLIT (`split_g = record_only ? ... : NULL`,
   NULL for a plain remove) and `:145-146` names the write into `cn->child` as
   **the harmless one**; and "an armed txn parks it blind" is false because
   records planted BEFORE arming stay MW -- `ft-remove.h:4004-4009`, *"stay MW,
   which is stricter and always sound"*.  Only the COST ground stands: +1 in an
   11-edge budget (`ft-mutation-helpers.h:9483`) and +1 CAS per compressed
   remove.
2. ☠ **AN INDEPENDENT LATENT KEY LOSS, with ZERO test coverage.**  The
   compressed-boundary lane's sub-case 2 (compressed ROOT retire,
   `ft-remove.h:380-382`) locks `{src_cn}` (`:443-453`), never re-reads
   `cn->child` under that hold, and its root edge's expected-old is a FRESH
   `*pub_slot` (`:519-522`) -- **a value compared against itself**, the
   anti-pattern the file names at `:831-832`.  A peer converting the sole key's
   leaf into a prefix head before the remover's DLM acquire passes the root CAS,
   `cn` is tombstoned, and the peer's key is dropped SILENTLY.  `:529-531`
   records that this sub-case has ZERO test hits.
3. **The fold arm cannot distinguish a value refusal from an acquire refusal**:
   both return `-EAGAIN` (`ft-remove.h:1318` acquire, `:1451` re-validation),
   folded at `:6034-6036`.  Any future design keying on that distinction is
   already refuted.

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
