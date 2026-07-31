# Sibling split/compress orphan: fix scope

Branch `ft-dlm-acquire`, defect present at **@18b307b4**. Reproducer:
`inv_sibling_split_compress` in `tests/regression/test_urcu_ft_inv.c`
(opt-in, `FT_INV_SIBP=1`). Root cause note: the memory entry
`project_ft_sibling_split_compress_orphan`.

Two writers per junction, each owning ONE child of a shared three-byte prefix,
both cycling insert -> read-back -> remove. Every insert must SPLIT the prefix
node; every remove must PATH-COMPRESS it back. No merge, no rekey, no graft.

## What is MEASURED (do not re-derive)

1. **The losing insert is ALWAYS the split path.** Tagging all six
   `ft_flip_txn_record_reserved` arms in `ft-insert.h` and accumulating a mask
   per op: every captured loss has `arms = 0x3` = `ft_insert_publish_or_park`
   (the forward `{P.m: expected_old -> new_top}` edge) + the external head's
   parent re-parent. The three `ft_attach_node` in-place arms are NEVER set.
   So I-1 (closed at HEAD -- the in-place attach does acquire) is not the gap.
2. **The split's own freshly published top is TOMBSTONE the instant its commit
   reports success**: `holder == newtop`, `holder_state = 0xa`. Checked by
   re-descending from the root inside `cds_ft_insert` immediately after
   `_cds_ft_insert` returns 0.
3. **The orphaned leaf is unreachable and unmarked**: root descent
   `NOT_FOUND`, `ft_node_is_removed(node) == 0`, `node->prev` names the
   tombstoned top.
4. **The livelock is that same orphan met by a later remove.**
   `_cds_ft_remove_locked` returns -EAGAIN 2,000,000+ consecutive times;
   `ft_meta_copying_mark` correctly refuses a TOMBSTONE word; the wrapper loop
   (`cds_ft_remove`, ft-remove.h ~:3620) re-derives from `node->prev` with **no
   top-down descent** and **no tombstone check on the holder** (its only guard,
   `ft_node_is_removed`, is on the LEAF and is false here), so it re-derives the
   same dead holder forever -- while holding the per-trie FIFO fair mutex.
   Measured 11 of 12 writer threads parked in `cds_fair_mutex_lock` behind it.
5. **Feature-independent.** fine-drop / dlm / noskip / nocompress all reproduce
   with `state=0xa`. Turning features off only moves WHICH fence site refuses
   the dead node (ft-remove.h:749 `ft_chain_compress_fused` -> ft-mutation-node.h
   `ft_node_recompact` / `ft_meta_copying_mark` directly).

## What is still HYPOTHESIS

Which remove-side commit tombstones the fresh top, and on what plan. The shape
that fits every measurement: the remover planned against the **pre-split** child
`C` at `P.m`, the splitter swapped `P.m: C -> N` (N containing both children),
and the remover's commit then retired **N** -- a node it never read -- instead
of aborting. That is the "stale plan slot paired with a freshly resolved
target" family already seen at @f6e7ea18
(`project_ft_chain_compress_stale_publish_slot`).

**Next probe to settle it (one run):** tag every freeze/tombstone record site
with its caller, and at the site log `{planned slot, planned expected_old, live
slot value at commit, node being tombstoned}`. If `planned expected_old != live
value` on the losing commit, the hypothesis is confirmed and (b) reduces to
"pin the pair".

---

# (b) UPSTREAM: stop the orphaning

This is the real defect. (a) without (b) converts a hang into silent data loss.

## The fault line

A structural remove must not retire the node currently at a slot unless that
node is the one its plan was derived from. Concretely: the freeze/tombstone
target and the publish slot's expected-old must be **one frozen pair captured
at plan time**, and the commit must reject a slot whose value changed.

## Candidate closures

**B1 — Pin the (slot, expected_old, retire-target) triple (preferred).**
Capture the retire target at the same instant as the slot's expected-old, and
record the forward edge with that expected-old so the commit CAS rejects a peer
swap. No new lock, no new fence; it makes an existing commit stricter.
*Risk:* raises abort rates on the remove path; each new abort must route to an
existing re-descend (they do -- `need_retry`). Low structural risk, needs an
abort-rate measurement.

**B2 — Re-read the plan under the fence.** `ft_chain_compress_fused` already
claims to "re-validate the caller's PRE-fence plan under the fence". Extend that
re-validation to the identity of the node at the publish slot, not just its
child population. *Risk:* only fixes the sites that have such a fence; the
noskip/nocompress runs show at least two other refusal sites, so this may be
per-site whack-a-mole rather than one closure.

**B3 — Have the split acquire the node it replaces.** Today
`ft_insert_publish_or_park` acquires/guards `parent_nf` (the node whose slot it
writes) but not `expected_old` (the subtree it subsumes). Acquiring the latter
would exclude a remover planning on it. *Risk:* this is the shape that
self-deadlocked at @b20c471e (I-1) before the §8.3 word split; it adds a second
lock-set member to the insert and must be reconciled with the remove's lock set
(`feedback_reconcile_lock_sets_across_subsystems` -- two correct lock sets over
one node self-deadlock, and a retry loop turns that into an infinite spin,
which is exactly the failure mode we are already in).

**Recommendation: B1**, with the settling probe above run first so the fix is
made against a confirmed mechanism rather than a plausible one. B3 is the
option to avoid unless B1 proves insufficient.

## Work items

1. Run the settling probe; confirm or refute the stale-pair mechanism.
2. Implement B1 at the confirmed site(s).
3. Extend `inv_sibling_split_compress` from a hang-detector into an assertion:
   entries == live, `cds_ft_verify`, leak check (all present, none reachable
   today because the run wedges first).
4. Fault-injection coverage: the new abort must be *taken*, not merely
   reachable -- count it, per `feedback_oracle_covers_only_shared_nodes`.
5. Full gate 9/9 + ASAN; re-run `harnesses/skeptic_sweep.c` (it sweeps the
   fault countdown over every acquire of a merge, and B1 touches commit
   validation).

## Validation gap to close alongside

The existing disjoint-key insert/remove oracles cannot reach this: their keys
diverge high enough that no node is ever both split and compressed under
contention. `inv_sibling_split_compress` is the first oracle that drives it,
and it must go into the gate once it can pass.

---

# (a) TERMINAL-BAIL: stop the wedge

Strictly secondary. It converts an unbounded wedge into a bounded, reportable
outcome, and is worth landing on its own because a livelock holding the trie's
FIFO mutex denies service to every writer.

## The change

`-EAGAIN` currently means two different things at
`_cds_ft_remove_locked`'s switch: "a peer won this attempt" (transient,
retryable) and "the node my plan names is dead" (terminal -- excluding peers
cannot clear a tombstone). The wrapper's own no-livelock argument is that FIFO
escalation drains contention; that argument does not cover the terminal case,
and the comment should say so.

## Candidate closures

**A1 — Re-derive top-down on a terminal bail.** Keep `node->prev` as the fast
path; on a tombstoned holder, fall back to a descent from the key. Correct in
the presence of (b)'s residue; costs a descent on a rare path.

**A2 — Return NOT_FOUND on a tombstoned holder**, symmetric with the existing
`ft_node_is_removed(node)` idempotent-miss at the top of
`_cds_ft_remove_locked`. Cheapest, and it is the honest answer for a node that
is genuinely unlinked. *But it is only honest once (b) lands* -- today it would
report NOT_FOUND for a key the caller successfully inserted.

**A3 — Bounded retry then a loud failure** (debug builds abort, release returns
BUSY). A diagnostic, not a fix; useful as a permanent tripwire regardless of
which of A1/A2 lands, since it turns any future terminal-retry bug into a
report instead of a hang.

**Recommendation: A1 + A3.** A1 keeps the operation correct rather than merely
non-hanging; A3 is cheap insurance and would have caught this defect the first
time it ran.

## Work items

1. Split the terminal case out of the `-EAGAIN`/`-ENOENT` arm; correct the
   wrapper's no-livelock comment (it is a true claim with a stale mechanism --
   `feedback_stale_mechanism_worse_than_wrong_claim`).
2. Implement A1; add A3's counter with a debug-build abort.
3. Drop the `FT_INV_SIBP` env gate on `inv_sibling_split_compress` once it can
   no longer hang, so the gate runs it by default.
4. Gate 9/9 + ASAN.

## Sequencing note

(a) is independently landable and low-risk, but landing it FIRST would make the
oracle stop hanging and start failing quietly on entries/verify -- which is a
better failure mode, but also removes the loudest symptom while the data loss
remains. If both are landed in one series, land (b) first and (a) second so no
intermediate commit is "green but lossy".
