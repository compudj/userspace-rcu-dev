# Ordered-cell sibling list: a locking scheme for 100% SW at BOTH spacings

Status: **PROPOSAL, 2026-09-21.  Design only -- no source file changed.**
Tree: `ft/unpub-free-audit` @c7f695ed.  Companion:
`doc/design/ft-cell-list-sw-transition.md` (the problem statement, the census,
the two livelocking widenings).  Every `file:line` below is against @c7f695ed.

The one-paragraph version: **lock the cell, not the neighbour's holder.**  A
cell is an allocated item with its own `cds_ft_metadata.state` word
(`fractal-trie-internal.h:1714`, allocated by `cds_ft_alloc_cell_item`,
`fractal-trie-alloc.c:1618`), and *nothing in the tree takes that word today*.
Each list word `&c->lnode.next` / `&c->lnode.prev` is owned by the cell whose
body it lives in, so the cell's own lock IS its nearest lock-bearing ancestor
(register §8.2, "a node's body is its own").  A cell lock needs no descent, no
byte-depth and no `@ctx`, so it converts identically at `fine` and at
`exponential`; it contends only with splices adjacent to that cell, not with
the neighbouring KEYS' subtrees; and it composes with the existing DLM by
address order.  The two words nobody owns -- the sentinel's `next` and `prev`
-- and `&ft->root` get dedicated lock words embedded in `struct cds_ft`, as
Mathieu directed.  The release-ownership defect that killed both widenings is
a missing terminal record, not a design limit, and is closed by one rule:
**a cell lock is handed to the txn with BOTH a registration and a terminal
record at the instant it is taken, and no hand ever releases it afterwards.**

---

## 0. Corrections to the brief (read these first)

I was asked to say where the stated facts are wrong.  Four are, and one of
them is the centre of the design.

### 0.1 ☠ The promote's commit DOES release its holder -- the widening leaked for a different reason

The brief says: *"a promote's commit 'leaves holder->state untouched' by
design, so its holder is released by the CALLER, not the commit"*, citing the
comment in `ft_unchain_node` (`ft-remove.h:6650-6667`, the sentence at `:6656`).  **That comment is
stale.**  The promote takes the held arm of
`ft_flip_txn_hold_or_lock_parent_at` (`ft-mutation-helpers.h:13757-13790`),
which does two things for `@held_holder`:

    ft_flip_txn_lock_register_member(t, held_holder, held_snap, ...);
    ft_flip_txn_record_anchor_release_held(t, held_holder);

i.e. it REGISTERS the word *and* RECORDS a `{LOCK|s -> s}` terminal.  Its own
header (`:13740-13755`) says so: *"record the {LOCK|s -> s} RELEASE + register
it ... commit consumes it, an aborted/destroyed commit auto-clears it."*  That
is why `ft_promote_head` has no post-commit release on its success path and
`ft_unchain_node`'s promote arm says "No post-commit clear here"
(`ft-remove.h:6925-6927`).

Now the commit's registry handling, `ft_flip_txn_commit`
(`ft-mutation-helpers.h:7214-7248`):

* `st != OK` (ABORT / MEMORY_ERROR): `ft_flip_txn_lock_release_all(t)` --
  every registered word is CAS-cleared.
* `st == OK`: **the registry is NOT drained** -- *"a committed txn transitioned
  each registered node through the terminal its op recorded ... so the lock is
  already consumed"*.  A word that was registered but has NO terminal record
  stays LOCKED forever.

The v3 patch
(`fractal-trie-review-2026-06/widen-remove-v3-sorted-deduped-STILL-LIVELOCKS-2026-09-21.patch`,
hunk at `ft-remove.h:6258-6272`) called `ft_flip_txn_lock_register_held(txn,
pred_held)` and `..._held(txn, succ_held)` -- **registration only, no
`ft_flip_txn_record_release_lock`**.  So the first promote that took its
neighbours and COMMITTED left both neighbour holders locked permanently, and
every later attempt met a held word: that is precisely `widened ok=1
miss=50001`, "previous site :0".  The compaction lane shows the correct pair
three lines apart (`ft-compact.h:603-606`):

    ft_flip_txn_lock_register_held(t, &h);
    ft_flip_txn_record_release_lock(t, h.lock, h.lock_snap);

**So the "central problem" is a one-line ownership rule, not a lifetime
paradox** -- see §5.  (The SEGV of the release-after-return variant is
addressed in §5.4; I cannot name the faulting pointer either, and the design
removes that path rather than explaining it.)

### 0.2 Fact 4 is incomplete: the census is blind to THREE producers, not one

`ft_txn_list_insert_between_prepare` (`ft-mutation-helpers.h:10958`, from
`ft-insert.h:459`) records MW unconditionally, as stated.  Two more lanes
bypass `ft_cell_edge_owner` by recording through the ENGINE's list op, whose
stores are `urcu_txn_store_mw` (`include/urcu/rcu-txn-list.h:720-723`):

| lane | site | exclusion today |
|---|---|---|
| `ft_ord_cell_swap` | `ft-mutation-helpers.h:14950` `urcu_txn_list_replace_prepare` | compaction: caller's writer exclusion (`ft-compact.h:28`, `:494`) |
| `ft_compact_relocate_cell` | `ft-compact.h:614` `urcu_txn_list_replace_prepare` | same |

Both are under whole-trie exclusion today, so they are uniformly MW and
sound; but they are *records on the same words*, and they must be routed
through the FT recorder before the flip (§8 stage 2) or they become the mixed
writer.  There are also RAW stores into list words on bulk paths --
`ft_ord_finalize_circular` (`ft-mutation-helpers.h:14219`),
`ft_ord_survivor_link` (`ft-merge.h:1115`, `:1131`), `ft-graft.h:4742-4743` --
all under the bulk gate; they are listed in §7 because "every writer holds the
lock" has to include them or exclude them provably.

### 0.3 Fact 1's "needs a descent to date the holder" is an artefact of the wrong anchor

`ft_cell_edge_owner` (`ft-mutation-helpers.h:11493-11614`) refuses at coarse
spacing with *"no descent to date the holder"* (`:11545-11548`) because it
anchors the cell word on `owner_cell->parent` -- a TRIE node, which at
exponential maps to an ancestor via `ft_anchor_meta` (`:691-760`) and so needs
a byte-depth.  Under a per-cell lock the anchor of a cell word is the cell:
`anchor(cell) = cell` is total and every route computes it identically, which
is all §1 of `ft-dlm-lock-coarseness.md` asks.  `ft_lock_ctx_depth_of_climb`
(`:5895`) is therefore NOT needed for the cell words at all; it stays the tool
for the trie members (the holder) exactly as today.  Fact 8 is right about the
tool; the design just does not need it for this class.

### 0.4 Fact 5: the ownerless population is the sentinel's two words -- and the cell's own word was never counted as an owner

The sentinel is a bare `urcu_txn_list_node` inside `struct cds_ft`
(`fractal-trie-internal.h:2696`; `include/urcu/rcu-txn-list.h:158-160`) and
has no metadata, so two dedicated lock words are required exactly as the brief
says (§2.2).  But the sentence *"pred/succ are the NEIGHBOURING KEYS in key
order, whose holders are off this op's descent"* (fact 3) is where the failed
design was chosen: it names the neighbour's HOLDER as the lock.  The
memory note `project_ft_chain_protection_design_matrix` records that Mathieu's
own matrix for cell-ON + fine is *"THE CELL's OWN LOCK (it already exists and
NOTHING takes it)"*.  This proposal applies that row to the LIST words, at
both spacings (reasons in §1.2).

### 0.5 A recorded reversal to confirm with Mathieu

`project_ft_mw_by_design_is_root_and_sibling_list` (2026-09-17): *"the root
pointer is MW on purpose, and the cell sibling list is MW on purpose too."*
The brief (and step 3 of the companion doc, 2026-09-20) says the direction is
now dedicated locks for `&ft->root`, BEGIN and END and 100% SW.  I have
designed to the later statement.  If the earlier one still stands, §7's
"stays MW" set is `&ft->root` plus the sentinel's two words -- and everything
else in this document is unchanged, because the interior cell words are the
30.9% + insert lane that the earlier statement did not exempt.

---

## 1. Which lock protects which word (answer to (a))

### 1.1 The table

| word | lives in | lock word | at `fine` | at `exponential` |
|---|---|---|---|---|
| `&c->lnode.next`, `c` a cell | `c`'s body | `cds_ft_item_to_metadata(c)->state`, `FT_STATE_LOCK` | same | **same** |
| `&c->lnode.prev`, `c` a cell | `c`'s body | same word as above | same | **same** |
| `&ft->ord_sentinel.node.next` (list BEGIN) | `struct cds_ft` | **`ft->ord_begin_lock.state`** (new, embedded) | same | same |
| `&ft->ord_sentinel.node.prev` (list END) | `struct cds_ft` | **`ft->ord_end_lock.state`** (new, embedded) | same | same |
| `&ft->root` | `struct cds_ft` | **`ft->root_lock.state`** (new, embedded) | same | same |
| structural edges of the same commit (head slot, SKIP_X dual, nr_child) | trie nodes | UNCHANGED: `ft_anchor_meta` of the member | node itself | anchor ancestor, dated as today |

A regular cell has ONE lock covering BOTH its words; the sentinel has TWO,
one per word.  That is not an asymmetry in the rule -- the rule is "the word's
owner's lock" -- it is that the sentinel is the one owner Mathieu asked to
split so a head splice and a tail splice do not serialise
(`ft-cell-list-sw-transition.md` step 3).  For a cell the same argument does
not apply: both words of one cell are written by splices *adjacent to that
cell*, and serialising those is the intent.

The three new lock words are `struct cds_ft_metadata` instances embedded in
`struct cds_ft` beside `ord_sentinel` so that `ft_meta_lock_acquire`
(`ft-mutation-helpers.h:4782`), `ft_dlm_lock_now` (`:9106`),
`ft_meta_lock_release` (`:4845`) and the registry (`t->locks[i].meta`) work on
them unchanged and they carry a sortable address.  Only `state` is used; the
rest of the struct is dead weight (three of them, once per trie -- acceptable;
a slimmer `struct ft_lock_word { uintptr_t state; }` plus a `container_of`-free
lock API is the alternative if the embedding offends).

### 1.2 Why the cell's own lock is the nearest-ancestor rule, and why it is right at exponential

* **It is literally the nearest lock-bearing ancestor.**  The word
  `&c->lnode.next` is a field of `c`; `c` has a state word.  The holder
  `c->parent` is one hop FURTHER up.  The register's §8.2 already reads a
  body word as owned by the object it sits in; the failed design skipped the
  cell and went to the holder because the cell's state word had never been
  used as a lock.
* **Agreement (§1 of `ft-dlm-lock-coarseness.md`) holds trivially and at
  every spacing.**  The spacing schedule maps a TRIE NODE at byte-depth `d`
  onto an anchor at `L(d)`.  A cell is not on any key path and has no
  byte-depth; `ft_anchor_meta` is never asked about it.  So `anchor(cell) =
  cell` is a second, independent total function -- there is nothing for it to
  disagree with, and no descent, `@ctx` or climb is needed to compute it.
  This is what makes exponential convert at 100% instead of 0%: the
  `noctx`-at-coarse refusal (`ft-mutation-helpers.h:11545-11548`) disappears
  because the predicate stops consulting `@ctx` for cell words.
* **Contention footprint is the smallest possible.**  Lock(c) is contended
  only by ops that splice adjacent to `c`.  The failed widenings took the
  neighbour KEYS' holders -- which, on the `skr_writer` shape, are the next
  remover's PRIMARY acquire (`ft-cell-list-sw-transition.md` "Mechanism").
  With cell locks, same-key removers still serialise on the holder (class 0,
  §3) exactly as today, and only the winner reaches the cell take.
* **It does not collide with the spacing regime**, contrary to the note's
  worry ("a per-cell lock would be inconsistent with the spacing regime").
  The regime's purpose is that two ops mutating one TRIE NODE take one word;
  cells are never lock-set MEMBERS of a trie descent.  The one place the two
  classes meet is the ORDER of takes, handled in §3.
* **`&ft->root`**: the memo `project_ft_lock_bearing_word_protocol` quotes
  Mathieu: *"root node pointer is the exception where we keep MW CAS rather
  than locking"* (`&ft->root` lives in no node).  A dedicated `root_lock`
  gives it a node-equivalent; every writer of `&ft->root` (the `.root = true`
  edges: `ft-mutation-helpers.h:11998`, `:14011`, and the root publishes that
  route through `ft_flip_txn_record_root`) takes it in class 0.  Until every
  such writer does, the ROOT class stays uniformly MW (§7).

### 1.3 The cell's state word is free today, and the retire needs a TOMBSTONE in it

No code reads or writes a cell's `state` (grep: the only cell-metadata
accesses are `incoming_byte`, e.g. `ft-remove.h:6291`, and the alloc/free
routes `ft-lookup-helpers.h:247-291`).  Two consequences:

* `FT_STATE_LOCK` on a cell is a fresh bit with no reader; the acquire and
  release primitives' refusal set (`PROXY | FT_SA_DEAD_REFUSE | LOCK`,
  `ft_dlm_lock_now` `:9110-9112`) applies as-is.
* The op that RETIRES a cell (swap: `ft_ord_cell_swap_edges`
  `:14976-15019`; unsplice: `ft_ord_cell_unsplice_edges` `:14832-14858`) must
  leave `TOMBSTONE` in the retired cell's state word, fused with the LOCK drop
  -- the RETIRE terminal `{LOCK|s -> TOMBSTONE|s}` nodes already use.  A peer
  whose stale plan still names that cell as `pred`/`succ` is then REFUSED AT
  THE TAKE by `FT_SA_DEAD_REFUSE`, before it records anything.  The free is
  already RCU-deferred (`ft_ord_cell_free` `:273-279` routes through
  `cds_ft_free_item`), so the word stays readable for the grace period.  The
  deletion MARK on the retired cell's `next` is kept (it is what
  `urcu_txn_list_is_marked` readers and the engine prepares see); it is simply
  recorded SW under the retired cell's own lock like any other word of it.

---

## 2. Deriving the complete lock set before the first acquire (answer to (b))

### 2.1 Two lock CLASSES, and the rule that makes two takes one order

Every acquire in a point op is one of:

* **class 0 -- trie anchors**: the lock-set members the op already takes
  (holder, P, GP, the SKIP_X dual's grandparent) resolved by `ft_anchor_meta`,
  plus `ft->root_lock`.
* **class 1 -- list locks**: cell state words, `ft->ord_begin_lock`,
  `ft->ord_end_lock`.

**Rule C: an op never takes a class-0 word while it holds a class-1 word.**
Within one class the take is one sorted all-or-none `ft_dlm_acquire_set`
(`ft-mutation-helpers.h:10142`).  The global order is therefore
`(class, address)`, a total order, and §3's argument composes across the two
takes exactly as it does within one.

Why two takes instead of the single take the companion doc and v3 tried:

* the class-1 set is derivable only from the cell (`pred`/`succ` are the
  cell's links), and the cell is reached AFTER the holder is known -- at the
  promote it is `ft_ord_cell_ptr(node->prev)` (`ft-remove.h:6190`), at the
  insert it is the result of `ft_ord_cell_find_pred_from_head`
  (`ft-insert.h:404-431`), which runs after the split cluster is built;
* the class-0 take of `ft_unchain_node` (`ft-remove.h:6717-6719`) and the
  in-promote class-0 takes (`ft_flip_txn_hold_or_lock_parent`, the dual GP
  via `ft_lock_skip_dual_gp`, `ft-remove.h:6337`) then stay EXACTLY as
  they are -- the v3 restructuring of the holder take is not needed;
* Rule C is satisfiable at every site by one placement constraint: **the
  class-1 take is the LAST take before the commit**, immediately before the
  edge array is built.

Single-take variant (v3's shape, one `ft_dlm_acquire_set` of both classes,
sorted by address regardless of class): equally correct under §3, and it is
the fallback for any site that cannot place its class-1 take last.  It costs
the pre-acquire resolution and the re-validate.  Do not mix the two variants
in one op.

### 2.2 The set, per producer

Notation: `L(c)` = lock(c) for a cell; `L(sentinel as pred)` =
`ord_begin_lock` (the word written is `sentinel.next`); `L(sentinel as succ)`
= `ord_end_lock` (the word written is `sentinel.prev`).  `dedupe` = the DLM's
existing same-anchor dedupe (`ft-mutation-helpers.h:10603-10612`, the
`deduped` arm sets `held.shared`); it is needed because a two-cell list has
`pred == succ`, and an empty-list insert has pred = succ = sentinel (two
DIFFERENT locks, begin and end -- no dedupe there).

| producer | class-1 set | words written (owner) |
|---|---|---|
| head promote, `ft_promote_head` (`ft-remove.h:6185`, census :6228, 14,642) | `{L(old_cell), L(pred), L(succ)}` | `pred.next`(pred), `succ.prev`(succ), `old_cell.next` mark (old_cell) -- `ft_ord_cell_swap_edges` `:14976` |
| in-place delete / detach, `ft_detach_node` (`ft-remove.h:2562`, census :4845, 9,488) via `ft_remove_one_commit` `:15252` / `ft_remove_commit_rec` `:15659` | `{L(dead_cell), L(pred), L(succ)}` | `ft_ord_cell_unsplice_edges` `:14832`: `pred.next`, `succ.prev`, `dead.next` mark |
| chain-compress collapse, `ft_chain_compress_fused` (`ft-remove.h:1375`, census :1519, 75) | same as detach | same |
| insert splice, `ft-insert.h:459` -> `ft_txn_list_insert_between_prepare` `:10958` (19,628) | `{L(pred or begin), L(succ0 or end)}` | `pred.next`(pred), `succ0.prev`(succ0); the new cell is unpublished |
| insert-replace cell swap, `ft-insert.h:4858`, `:5299`, `:6361` (`ft_ord_cell_swap_publish_multi`) | `{L(old_cell), L(pred), L(succ)}` | as promote |
| compaction relocate, `ft-compact.h:614` and `ft_ord_cell_swap` `:14950` (engine lane, §0.2) | `{L(old), L(pred), L(succ)}` | as promote (`:14990`: "the same two edges as `urcu_txn_list_replace_prepare`") |
| root/list swap publishes `ft_ord_sentinel_edges` `:11793` (head_new/tail_new edges `:11833`, `:11841`) | `{L(head_new)`, `L(tail_new)`, begin, end}` as touched | sentinel endpoint repairs |
| bulk run ops: `run_splice` `:16049`, `run_resplice` `:16135`, `run_replace` `:16241`, `run_detach` `:14130`, `run_unlink` `:16419`, `ft_ord_survivor_link` `ft-merge.h:1110`, `ft_ord_finalize_circular` `:14211` | `{L(run_first), L(run_last), L(pred), L(succ)}` -- four locks per run, NOT one per cell | the run's two outer back-edges + the two neighbours |

Bulk ops run under the FT-wide bulk gate; §7 explains why they take the four
cell locks anyway (uniformity is cheaper to prove than exclusion).

### 2.3 Validate under the lock, then never re-resolve (this is also (e))

After a successful class-1 take, and BEFORE any record:

    old_cell == ft_ord_cell_ptr(node->prev)                (promote/detach only)
    ft_ord_cell_resolve_ord(&old_cell->lnode.prev) == pred
    ft_ord_cell_resolve_ord(&old_cell->lnode.next) == succ  (unmarked)
    insert:  resolve(&pred->lnode.next) == succ0, unmarked; succ0->next unmarked

If any term fails: `ft_flip_txn_destroy(txn)` (which releases every
registered lock, §5), return `-EAGAIN`, lane back-off.  If all pass, they
CANNOT change until the commit: every writer of `old_cell.prev` /
`old_cell.next` / `pred.next` / `succ.prev` needs one of the words this op
now holds.  A peer that retired `pred` or `succ` between the unlocked resolve
and the take is refused at the take itself (TOMBSTONE, §1.3), so the
validation never sees a dead neighbour.

This is the same "★ RE-DERIVE AFTER THE ACQUIRE" idiom `ft_unchain_node`
already carries for its holder (`ft-remove.h:6728-6760`; the routing re-validation at `:6766`) -- established, not
invented.

---

## 3. Lock order and the deadlock argument (answer to (c))

**Order key = `(class, &meta->state)` ascending**, i.e. all trie anchors and
`root_lock` before any list lock, and by address within a class.  Within a
class the take is `ft_dlm_acquire_set` with `FEATURE_FT_LOCK_TAKE_ORDERED`
made **unconditional** (Mathieu's rule; today it is opt-in at
`ft-mutation-helpers.h:10146`, and the shipping build compiles the sort out --
the v3 patch had to re-implement the sort at the caller for that reason,
`ft_dlm_set_sort_by_anchor`).  `ft_lock_set_order_by_anchor` (`:9141`)
learns the lock-only member form (§9.1) and sorts it by its given anchor.

The argument, in three parts:

1. **No deadlock is possible at all**, independent of order: every take is a
   try (`uatomic_cmpxchg` in `ft_dlm_lock_now`, `:9114`) and a miss unwinds
   everything taken in that call (`:10908-10910`) and returns `-EAGAIN`
   holding nothing new.  Nobody blocks on a lock.  The memo
   `project_ft_semantic_retry_on_held_lock` shows why this must stay so: a
   BLOCKING wait from a lock into the FIFO lane closes a cycle and deadlocks
   (backtrace-proven).  So the design keeps try + release-all + back-off.
2. **The total order makes mutual obstruction impossible**, which is what
   turns "no deadlock" into "no ordering livelock": if op A holds `x` and
   misses `y > x`, the holder B of `y` took every word it needs below `y`
   before `y` -- so B does not need `x` (else it would have missed at `x`
   while A held it and hold nothing).  B therefore completes and releases;
   A's retry finds `y` free.  Rule C extends this across the two takes,
   because a class-1 holder never goes back for a class-0 word.
3. **Starvation is handled by the lane**: `ft_acq_lane_backoff` (`:9325`),
   aged (`FT_ACQ_LANE_AGE` = 4 refusals) into the domain's FIFO lane,
   queued holding nothing.  This is the arbitration the engine acquire used
   to provide and the ordered take put back; it runs only under
   `FEATURE_FT_LOCK_TAKE_ORDERED` today (`:10929-10935`) -- another reason
   ordered-always is a precondition, not an option.

Honest caveat: class-0 takes across FRAMES are already not one sorted take
today (`ft_unchain_node`'s holder is taken while the remove wrapper may hold
descent members -- "an op's HELD SET is a CHAIN of frames",
`ft-dlm-lock-coarseness.md` §9).  Their freedom from livelock rests on (1)+(3)
plus dedupe-on-shared, not on (2).  This proposal does not worsen that: the
class-1 take is strictly after every class-0 frame.

**Why this does not re-create the two measured livelocks.**  Both were the
§0.1 leak (`widened ok=1`, every later miss at `ft_dlm_lock`
`:10673` with "previous site :0" -- a *permanently* held word, not a
contended one).  The companion doc's contention theory ("one remover's
neighbour hold is the next remover's PRIMARY acquire") described the
neighbour-HOLDER design; under cell locks the primary acquire of a same-key
remover is still the holder (class 0), and adjacent-key removers overlap only
on ONE cell each, in a sorted all-or-none take.

---

## 4. The lock-only member and the acquire path (mechanics for (b)/(c))

`struct ft_dlm_member` (`ft-mutation-helpers.h:6047-6054`) grows an explicit
lock-only form.  A NULL `.nf` is a HOLE today (`:10165-10168` counts
`nr_present` by `.nf`; `ft_lock_set_order_by_anchor` `:9160-9176` skips
`.nf == NULL`), so the discriminator must be a new flag, not NULL:

    struct ft_dlm_member {
        ...existing...
        struct cds_ft_metadata *anchor;   /* lock-only: the word to take */
        bool lock_only;                   /* no nf, no depth, no guard */
    };

* `ft_lock_set_order_by_anchor`: `key[i] = set[i].lock_only ? set[i].anchor :
  (set[i].nf ? ft_anchor_meta(...) : NULL)`.
* `ft_dlm_acquire_set_at`: a lock-only member counts as present, contributes
  NO `ft_dlm_guard_parent` / node guard (there is no parent edge to validate,
  and no `node_snap` -- `held.member = anchor`), takes `ft_dlm_lock_now` on
  `anchor`, dedupes by anchor address like any other member.
* `ft_cell_lock_member` (`:11628-11655`) is rewritten to build this form from
  a cell / the sentinel side; its per-node-only refusal (`:11637`) goes away.

A helper `ft_cell_lockset_take(ft, ctx, txn, plan)` wraps: build the class-1
set from a `struct ft_cell_plan { cell, pred, succ, pred_is_begin,
succ_is_end }`, sort, `ft_dlm_acquire_set`, on success **immediately** hand
every non-shared hold to `txn` (§5), then run §2.3's validation and on
failure `ft_flip_txn_destroy(txn)`.  Every producer in §2.2 calls this one
helper so the set and the hand-off cannot diverge site by site (the lesson of
"an op's 13 sites audited": `project_ft_acquire_miss_carries_on`).

---

## 5. Lifetime and release ownership (answer to (d) -- the central problem)

### 5.1 The rule

> **A class-1 lock has exactly ONE owner from the instant it is taken: the
> txn.**  The take happens only when the txn already exists; the hold is
> registered AND given its terminal record in the same helper call; after
> that, no site releases it by hand, on any path.

Concretely, per hold `h` returned by the take, `!h.shared`:

| the cell's fate in this commit | terminal record | primitive |
|---|---|---|
| survives (pred, succ, sentinel words) | `{LOCK|s -> s}` RELEASE | `ft_flip_txn_lock_register_held(t, &h)` + `ft_flip_txn_record_release_lock(t, h.lock, h.lock_snap)` (`:12508`) -- the pair `ft-compact.h:603-606` already uses |
| retired by this op (old_cell / dead_cell) | `{LOCK|s -> TOMBSTONE|s}` RETIRE | register + the retire-shaped record (the node retire's fused arm, `ft_flip_txn_record_retire_anchored`'s twin), and `ft_flip_txn_lock_mark_retiring` (`:5147-5155`) so the OK-path scrub keeps it as a retiring terminal |

`h.shared` (dedupe onto a word this op already holds: `pred == succ`) owes
nothing -- the first take recorded both (`ft_held_anchor.shared`'s header,
`:837-845`).

### 5.2 What each terminal path then does -- all existing machinery, no new sweep

| path | who clears the LOCK | where |
|---|---|---|
| commit OK | the terminal record itself, settled in the LATE pass with the other lock words (`urcu_txn_desc_set_late_tag(FT_STATE_PROXY)` `:6963`; "lock words settle last", `project_ft_late_settle_tag_collision`) | `ft_flip_txn_commit` `:7214-7248`: registry NOT drained, by design |
| commit ABORT / MEMORY_ERROR | `ft_flip_txn_lock_release_all(t)` -- the RELEASE record settled back to `LOCK|s`, so the sweep CAS-clears | `:7226-7228` |
| any bail after the take | `ft_flip_txn_destroy(t)` -> `ft_flip_txn_lock_release_all` | `:6143-6160`, `:6114-6127` |
| a bail BEFORE the txn exists | impossible by construction: the take requires `txn` | -- |

`ft_flip_txn_record_release_lock`'s header (`:12462-12506`) states this
contract verbatim: *"Registered in the txn's locks[] registry exactly like a
retire, so the two non-commit terminals ... CAS-clear the lock and leave the
node live ... The registry itself therefore needs NO knowledge of which
terminal an op chose."*  The design adds nothing to it; it stops the class-1
locks from being the one population that skipped it.

### 5.3 The placement this forces at each site

* **`ft_promote_head`**: the txn is created at `ft-remove.h:6228-6230` -- which is exactly the census's "creation site :6228", since `FT_TK_TXN_SITE` keys on `ft_flip_txn_create_bounded`
  (`ft_flip_txn_create_bounded`).  The class-1 take goes AFTER
  `ft_lock_skip_dual_gp` (the last class-0 take, `:6337`) and BEFORE
  `ft_ord_cell_swap_publish_multi` (`:6432`), with the reservation grown by 3
  state-word records (`+3` at `:6230` becomes `+6`, or
  `ft_flip_txn_reserve_locks` `:4990`).  The two `-ENOMEM` exits above the
  take keep releasing `held_holder` by hand exactly as today (`:6221-6237`);
  no exit below the take touches a lock.
* **`ft_detach_node`**: `commit_txn` is created at `:4845` *before*
  `ft_node_replace_ptr`'s recompaction, which takes class-0 words through
  the glue.  The class-1 take must be the LAST take, i.e. immediately before
  `ft_remove_one_commit` (`:15190`) / `ft_remove_commit_rec` (`:15636`)
  build the unsplice edges -- after the recompaction's acquires.  **I have
  not verified that no class-0 acquire happens inside those two commit
  helpers**; §10 lists the debug assert that makes this a machine check.
* **`ft_chain_compress_fused`**: its txn is already created FIRST precisely
  so that *"every bail from here on is a ft_flip_txn_destroy or the commit
  itself, and both terminal paths drain the fence registry -- no unwind can
  leak a fence"* (`ft-remove.h:1499-1512`, "Created FIRST" at `:1505`).  That comment is the design
  rule of this section, already in the tree; the class-1 take slots in
  before its cell edges are built.
* **insert** (`ft-insert.h:459`): `ic->txn` exists; the structural set is
  held; take `{L(pred|begin), L(succ0|end)}` after the second
  `ft_ord_cell_find_pred_from_head` and before the prepare; the existing
  `splice_conflict:` path already ends in `ft_flip_txn_destroy(ic->txn)`
  (`:466-470`), which now also releases the cell locks.
* **compaction** (`ft-compact.h:590-640`): already the model site.

### 5.4 Why the two failures cannot recur, and what I cannot prove

* **The leak**: impossible, because the helper that takes also records the
  terminal (§5.1); a hold with a registration and no terminal is not
  constructible through the helper.  Debug check: at commit, every
  `t->locks[i]` whose `meta` is a class-1 word must have a state record on
  `&meta->state` in the descriptor (`urcu_txn_find`), else abort -- a
  positive control is one line (skip the record) and must fire.
* **The SEGV of release-after-return**: the path no longer exists (no hand
  release after the take).  Mechanism, uncertain: with the v3 shape the
  released words were the neighbour keys' HOLDERS, and a promote whose
  `next_node` publish recompacts or re-homes can retire a holder it never
  named -- but I could not identify it from the patch alone and the brief
  says it was not isolated.  Under cell locks the released words are cells,
  whose free is RCU-deferred and whose terminal is in the commit.
* **A retired cell's lock word**: the RETIRE terminal leaves `TOMBSTONE|s`;
  the `ft_flip_txn_lock_release_all` sweep on ABORT sees `LOCK|s` (the record
  settled back) and clears it -- same as a node retire that aborts.

---

## 6. The insert lane's interposition detector (answer to (f))

Today (`ft-mutation-helpers.h:10958-10995`): the MW record on `&pos->next`
with expected-old `succ_expected` makes the commit's CAS fail if a peer
interposed after the caller decided the order; `ft-insert.h:414-431` adds the
capture-and-confirm re-search (`succ0`, `pred2 == pred`).

Replacement: **exclusion plus a validate at a point after which interposition
is impossible.**  An interposition between `pred` and `succ0` must write
`pred->next` (and `succ0->prev`), and both words are under locks this op
holds from the take until the commit.  So:

1. take `{L(pred|begin), L(succ0|end)}` (class 1, after the structural set);
2. under the lock: `resolve(&pred->lnode.next) == succ0`, neither marked --
   this observes a state no earlier than the CAS would have, and unlike the
   CAS it is followed by an interval in which the observed fact cannot
   change (the lock), so it is strictly stronger than a commit-time CAS
   (the same argument `ft_acq_guards_ok`'s header makes for the acquire's
   read set, `:9204-9230`; `ft_acq_guards_ok` at `:9375`);
3. record both edges through the owner-aware recorder; they park SW.

The `pred2` re-search stays through the migration (it costs nothing in
soundness) and becomes removable perf after the flip; the recycled-`pred`
coincidence the header worries about is covered by the TOMBSTONE refusal at
the take (§1.3) -- a freed-and-rebound cell cannot be locked by a stale plan
inside the grace period, and after it the plan's `pred` pointer is a live
different cell whose `next` fails step 2.

`-ENOENT` (`pos` deleted) and `-EAGAIN` (successor mid-deletion) keep their
meaning: both are now detected at the take (dead cell refused) or at step 2
(mark seen), before any record.

---

## 7. Words that stay MW, and the writers of each (answer to (g))

**End state: none of the list words stays MW.**  During migration, whole
CLASSES stay uniformly MW until their stage flips; the invariant at every
stage is "one discipline per word", proven per class:

| class of word | writers (complete list from `grep owner_cell =` + engine/raw stores) | discipline per stage |
|---|---|---|
| interior cell `next`/`prev` | `ft_ord_cell_swap_edges` `:14976`, `ft_ord_cell_unsplice_edges` `:14832`, `ft_txn_list_insert_between_prepare` `:10958`, engine `replace_prepare` at `:14950` and `ft-compact.h:614`, run ops `:14130`/`:16049`/`:16135`/`:16241`/`:16419`, `ft_ord_sentinel_edges` `:11793` (head_new/tail_new), raw: `:14219`, `ft-merge.h:1115`/`:1131`, `ft-graft.h:4742` | MW until stage 4; SW after |
| sentinel `next` / `prev` | the same producers when a neighbour is the sentinel (`ft_ord_is_end`), `ft_ord_sentinel_edges` | MW until stage 4 (sound today: keyed on identity, per-word) |
| retired cell's `next` (mark) | swap `:15010-15018`, unsplice `:14850-14857`, engine `del_prepare`/`replace_prepare` | same as interior |
| `&ft->root` | `.root = true` edges (`:11998`, `:14011`) via `ft_flip_txn_record_root` | MW until every writer takes `root_lock` (stage 5); note §0.5 |
| `cell->parent` (back-channel, NOT a list word) | `ft_ord_cell_set_parent` raw (`ft-lookup-helpers.h:296`), the parent-word records | out of scope; the register calls it "MW only by debt", owner = holder P |

**Bulk producers take the four cell locks** (§2.2) rather than relying on the
bulk gate's exclusion.  Reason: "every point writer is excluded during the
bulk op" is a property of the bulk↔point mode flip (memory:
`project_ft_fine_locking_transition_incomplete`, "bulk↔point is G5.25's
FT-wide re-take = a DEPENDENCY"), which I cannot certify from this tree; four
CAS per run under a gate nobody contends is free, and it keeps ONE rule the
census can check per record.  The raw stores (`:14219`, `ft-merge.h:1115`,
`ft-graft.h:4742`) are on tries the bulk op is finalising; the finalize
comment (`:14200-14210`) says readers may be live, so they must become
recorded edges under the cell lock at stage 3 -- or be proven to run only on
an unpublished list (then they are unpublished-node writes, outside the
discipline).  I have not proven either.

---

## 8. Staged migration, every stage sound (answer to (h))

Meter throughout: `-DFT_DEBUG_CELL_OWNER -DFT_DEBUG_TXN_KIND` census
(`ft_cow_report`, `:11418-11485`), extended at stage 0 with two buckets --
`engine_lane` (a cell-word record that never asked the owner) and, after the
flip, `owned` (records whose owner lock `ft_flip_txn_holds(t, owner)`
`:5201-5203` answers for).  Gate: the debug (`--enable-rcu-debug`) kind
detector stays armed; it is per-descriptor and cannot see the cross-txn mix,
so the per-record ownership assert below is the instrument that matters.

| stage | change | discipline on list words | meter target | how it is tested |
|---|---:|---|---|---|
| **0 infrastructure** | lock-only `ft_dlm_member`; `ft_lock_set_order_by_anchor` takes `.anchor`; ordered-always; `ord_begin_lock`/`ord_end_lock`/`root_lock` in `struct cds_ft`; `ft_cell_edge_owner` rewritten to answer by OWNER IDENTITY (slot == `&sentinel.next` -> begin, `.prev` -> end, else `cds_ft_item_to_metadata(owner_cell)`), no `@ctx`, no depth; park still OFF | all MW (unchanged) | census now reports `would-hold` per producer at BOTH spacings | ft_unit + ft_inv at fine/exp/root-only; unchanged behaviour |
| **1 point ops take cell locks, still record MW** | `ft_cell_lockset_take` at promote, detach, chain-compress, insert, insert-replace swap; §5 hand-off | MW + lock = "MW after the take is CORRECT, just a wasted CAS" (`project_ft_lock_bearing_word_protocol` §1-3) | `notheld` -> 0, `sentinel` -> 0 at fine AND exp | the same-key remove reproducer (the livelock shape); `FT_DEBUG_OP_RETRY_CAP`; take miss counters `ok/miss` per site; leak canary = the §5.4 commit assert |
| **2 route the engine lanes through the FT recorder** | `ft_ord_cell_swap` and `ft-compact.h:614` build `ft_ord_cell_swap_edges` into `ft_ord_cell_flip_into` instead of `urcu_txn_list_replace_prepare`; `ft_txn_list_insert_between_prepare` records through the owner-aware recorder | still MW | `engine_lane` -> 0 | ft_inv compaction + insert legs |
| **3 bulk ops take the four cell locks; raw stores recorded** | run ops + finalize + survivor link | still MW | every cell-word record has a registered owner: `owned == total` | rekey/merge/graft legs |
| **4 THE FLIP** | park SW by default; delete `FT_CELL_SW_ENABLED`; per-record assert: a cell-word record whose owner lock this txn does not hold ABORTS (`FT_OWNER_ASSERT_OWNED`, the structural edges' existing check) | **SW, uniformly, all spacings** | `ok == total`, refusals 0; red control: drop ONE take at one site -> the assert must fire on the first record | full gate incl. rcu-debug; matched A/B against stage 3 (the memo's 120-run table shape) |
| **5 root** | every `&ft->root` writer takes `root_lock`; `.root = true` records via the owner-aware recorder | ROOT class SW | `MW_ALWAYS ROOT` -> 0 | root-only spacing legs |
| **6 perf (optional)** | drop the insert's `pred2` re-search; drop the §4.B-style guards that a held cell lock subsumes | -- | -- | -- |

Soundness of each intermediate state: stages 0-3 change no record kind, so
every word stays uniformly MW -- the protocol's "MW after the take" case.
Stage 4 flips every producer at once (one predicate, one recorder), which is
the "no per-class conversion, no partial flip" rule; the ownership assert is
what proves the flip is not partial, per record, at run time.

---

## 9. Concrete code shape (for the implementer; still not implemented)

### 9.1 Data

    struct cds_ft {
        ...
        struct urcu_txn_list_head ord_sentinel;
        struct cds_ft_metadata ord_begin_lock;   /* &ord_sentinel.node.next */
        struct cds_ft_metadata ord_end_lock;     /* &ord_sentinel.node.prev */
        struct cds_ft_metadata root_lock;        /* &root */
    };

    struct ft_cell_plan {
        struct ft_ord_cell *cell;      /* NULL for an insert (unpublished new cell) */
        struct ft_ord_cell *pred, *succ;   /* sentinel pseudo-cell allowed */
    };

### 9.2 Owner predicate (replaces `ft_cell_edge_owner`'s body)

    static inline struct cds_ft_metadata *
    ft_cell_word_lock(struct cds_ft *ft, const struct ft_ord_cell_edge *e)
    {
        if (!e->owner_cell) return NULL;                  /* not a cell link */
        if ((void *) e->slot == (void *) &ft->ord_sentinel.node.next) return &ft->ord_begin_lock;
        if ((void *) e->slot == (void *) &ft->ord_sentinel.node.prev) return &ft->ord_end_lock;
        return cds_ft_item_to_metadata(e->owner_cell);
    }
    /* park SW iff ft_flip_txn_holds(t, ft_cell_word_lock(ft, e)) */

`ft_ord_is_end`'s fold of `!c` with "is the sentinel" (`ft-lookup-helpers.h:190-193`)
must split as the companion doc notes: a NULL owner is "not a cell link", the
sentinel is a lockable owner.

### 9.3 The take-and-hand-off helper (the only place class-1 locks are taken)

    int ft_cell_lockset_take(struct cds_ft *ft, const struct ft_lock_ctx *ctx,
                             struct ft_flip_txn *txn, const struct ft_cell_plan *p)
    {
        struct ft_dlm_member set[4] = { 0 };   /* cell, pred, succ (+ begin/end) */
        int n = 0, ret;

        /* build lock-only members; sentinel -> begin/end by SIDE */
        ...
        ret = ft_dlm_acquire_set(ft, ctx, set, n);   /* sorted, all-or-none */
        if (ret) return ret;                         /* nothing held */
        for (i = 0; i < n; i++) {
            if (set[i].held.shared) continue;
            ft_flip_txn_lock_register_held(txn, &set[i].held);
            if (set[i].anchor == cell_meta(p->cell))
                ft_flip_txn_record_retire_cell(txn, ...);  /* {LOCK|s -> TOMBSTONE|s} */
            else
                ft_flip_txn_record_release_lock(txn, set[i].held.lock, set[i].held.lock_snap);
        }
        if (!ft_cell_plan_still_valid(ft, p)) {      /* §2.3 */
            ft_flip_txn_destroy(txn);                /* releases all registered */
            return -EAGAIN;
        }
        return 0;
    }

Callers pass `p` (not a re-resolution) to the edge builders:
`ft_ord_cell_swap_edges(ft, p, new_cell, edges, n)`,
`ft_ord_cell_unsplice_edges(ft, p, edges, n)`; a debug assert in each compares
`p->pred/succ` with a fresh resolve.  That is the whole of (e).

---

## 10. Open items and what I could not verify

1. **Rule C inside `ft_detach_node`**: whether any class-0 acquire runs
   between the last recompaction acquire and `ft_remove_one_commit` /
   `ft_remove_commit_rec`.  Make it a machine check: in
   `ft_dlm_acquire_set_at`, refuse (debug abort) a non-lock-only take while
   `txn->locks[]` holds any class-1 word.  Telling a cell's metadata from a
   node's needs a discriminator; the state word has a free static bit (cells
   never carry PROXY/TOMBSTONE today) that `ft_ord_cell_alloc` can set, or
   the arena kind (`CDS_FT_ALLOC_KIND_CELL`, `fractal-trie-alloc.c:1622`) if
   it is recoverable from the metadata.  Unverified.
2. **Bulk raw stores** (`:14219`, `ft-merge.h:1115`, `ft-graft.h:4742`):
   recorded-under-lock or proven-unpublished.  Unverified which.
3. **`FT_DLM_ACQUIRE_MAX_SET` / reservation**: three extra state-word records
   per point commit, four per run; the bounded reservations at each site
   (`ft-remove.h:6228-6230`, `:4845-4853`, `:1519-1527`) grow by that count.
   Mechanical, but every site.
4. **The SEGV** of the v3 release-after-return variant: unexplained (§5.4).
5. **§0.5**: the 09-17 "MW on purpose" statement versus the 09-20 direction.
   Confirm with Mathieu before stage 5 (root) -- stages 0-4 are unaffected.
6. **Cost**: 2-3 CAS on lines the op already writes (the cells), plus one
   more `-EAGAIN` class (a cell-lock miss) that re-descends.  The
   `ok/miss` counters per site at stage 1 are the measurement; if a cell-lock
   miss rate approaches the holder's, revisit Rule C's placement (a class-1
   miss after a class-0 success re-descends the whole op).

---

## Mathieu's direction on this proposal (2026-09-21)

**Accepted, with the order fixed and one item reclassified.**

1. **Ordered-always lands FIRST**, before stage 0. Done: `ft_dlm_acquire_set_at`
   now sorts every lock set by anchor address unconditionally,
   `FEATURE_FT_LOCK_TAKE_ORDERED` is gone, and there is no unordered fallback to
   select between. *"Lock take should always be ordered, else it's deadlocks."*

2. **Then stage 0.**

3. **☞ AT EXPONENTIAL, LOCKING THE CELLS IS THE CORRECTNESS ANSWER — locking the
   ANCHORS instead is a later PERFORMANCE OPTIMISATION, not a correctness
   requirement.** Mathieu: *"for exponential locking, I agree that locking the
   cells is a good first step. Then we can investigate locking the anchors
   instead in exponential as a performance optimisation rather than for
   correctness (typically already locked, so fewer locks to take)."*

   The reasoning to carry forward: at a coarse spacing many cells share one
   anchor, and the op **typically already holds it**, so anchoring a cell word on
   its holder's anchor would collapse several class-1 takes into zero extra
   takes. That is a real win — but it is a win to measure against a design that
   is already CORRECT, not a reason to keep the correctness work waiting on the
   depth/`@ctx` plumbing that anchoring-on-the-holder needs. Cell locks first;
   anchor-folding afterwards, gated on measurement.

   ☠ Note the asymmetry when that optimisation is attempted: "typically already
   locked" is not "always locked", and a cell whose anchor this op does NOT hold
   must still fall back to taking the cell's own lock — otherwise the word goes
   back to carrying two disciplines, which is the defect this whole document
   exists to close.
