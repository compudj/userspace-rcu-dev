# Finding B — mark-vs-anchor dual coverage (E.2 oracle catch #2)

## The observation (root-only MW, test 30, deterministic)

    FT EXCLUSION VIOLATION: node N claimed at ft_chain_compress_fused:1009
      by T1 while owned by T2 (from ft_node_recompact:1357)
    ...preceded by: FT REFUSED (LOCK, unknown holder):
      ft_node_recompact:1357 word N state=0x80100 (ledger 0 deep)

state=0x80100 on N's OWN word while the DLM anchors at root-only all live
on the ROOT word: the lock on N is a per-node FENCE/MARK, not a DLM anchor.

## The mechanism (read-only analysis, 2026-08-27)

Two exclusion systems partition the trie DIFFERENTLY at coarse spacing:

  * DLM anchors: coarsen with the spacing -- at root-only, every acquire
    set collapses onto the root word.  A member node is covered WITHOUT
    any bit on its own word.
  * Fences/marks (copy fences, cow_stop marks, orphan freezes): always on
    the NODE's own word, at every spacing -- a fence is per-node by
    nature.

At per-node the two systems collide on the SAME word, so each refuses the
other (ft_meta_lock_acquire and ft_dlm_lock both refuse a LOCKed word)
and mutual exclusion composes for free.  At coarse spacings they are
BLIND to each other:

  * a fence taker never looks at the root anchor word -> it can fence a
    node that a root-anchored acquire set is covering (the observed
    violation: recompact's set covers N via the root; a peer fences N);
  * an anchored acquire never looks at member nodes' own words (the
    anchor IS the exclusion) -> it proceeds over a node a peer has
    fenced... UNLESS the coarsened-member guard catches it:
    ft_dlm_acquire_set_at DOES sample coarsened members' own words
    (ft_member_node_snap refuses a DIRTY member word), which closes HALF
    the window -- the acquire refuses a pre-existing fence, but a fence
    LANDING AFTER the sample (before the set's commit) is caught only by
    the guard's commit validation, and a fence landing after the COMMIT
    (during the op's post-acquire work) is caught by NOTHING.

So finding B = the second half: a fence taken on a member DURING the
anchored op's covered window.  The fence taker sees a clean node word
(the anchor holds no bit there) and succeeds; both then mutate.

## Fix directions (for Mathieu -- E-phase design decision)

  D1. FENCE-UNDER-ANCHOR: at coarse spacing, every fence take first
      acquires (or verifies holding) the node's covering ANCHOR, making
      the anchor the single arbiter; fences become node-local markers
      subordinate to it.  Cost: fence paths grow an anchor resolution +
      possible acquire; at per-node it degenerates to today (anchor ==
      node word).  The clean composition rule; matches the direction
      "one exclusion protocol" and dissolves B by construction.
  D2. ANCHOR-RESPECTS-FENCES: the acquire-set samples EVERY member's own
      word (not just coarsened ones -- it already does) AND the op
      re-validates member words at commit (guards exist) AND fences are
      forbidden between a peer's sample and release -- unenforceable
      without D1's ordering; rejected on analysis.
  D3. Spacing-scoped: declare fences legal only at per-node, refuse
      coarse spacings while any fence-using op class is enabled --
      effectively keeps coarse spacings dev-only forever; contradicts
      E.5's lift goal.

D1 is the recommendation.  Note test 30 sits in the FT_INV_MW suite:
re-adding the holdtrace gate's imw legs (the E.2 completion criterion)
requires B fixed; B fixed requires D1-class work; E.5's coarse-spacing
lift sits behind both.

## Status

Read-only analysis; no code touched.  The oracle's member-keyed stamp is
what made this class VISIBLE at all -- neither suite ever failed on it.


---

# ☠☠ SUPERSEDED 2026-08-27 (late) — THE MECHANISM ABOVE IS REFUTED

The analysis above inferred "the lock on N is a per-node FENCE/MARK, not
a DLM anchor" from N's own word carrying LOCK at root-only.  Measured
directly (oracle extended to carry each owner's DERIVED ANCHOR, the
claim's @shared, and the dedupe's LANE -- @69b09863), every step of that
inference is wrong:

  * BOTH sites are DLM ACQUIRE-SETS through the choke, not fences:
    ft-mutation-node.h:1357 and ft-remove.h:1009 are literally the
    `ft_dlm_acquire_set(...)` calls in ft_node_recompact and
    ft_chain_compress_fused.  A raw fence would stamp
    `ft_meta_lock_acquire:0`, and under FEATURE_FT_ANCHOR_VALIDATE the
    raw primitive is poisoned anyway.
  * BOTH derive the SAME anchor, so it is not an anchor disagreement
    either (the other hypothesis, and the one the splice root cause
    made plausible).  The anchor is the MEMBER ITSELF (=SELF).
  * THE CLAIMANT DEDUPED.  It did not take the word: it asked whether it
    already held it, was told yes, and proceeded -- while a peer
    genuinely held it.

    FT EXCLUSION VIOLATION: node N claimed at ft_chain_compress_fused:1009
      (anchor N=SELF) while owned by tid T2 (from ft_node_recompact:1357,
      anchor N=SELF)  claimant DEDUPED via lane: extra

## The real mechanism: finding A's class, through the fast path

A frame's EXTRAS entry answered holds() for a word whose exclusion had
already ended, and the acquire chain's OWNED-MASK FAST PATH accepted it:

    } else if (CMM_LOAD_SHARED(lock->state) &
                    (FT_STATE_LOCK | FT_STATE_PROXY | FT_STATE_TOMBSTONE))
            deduped = true;         /* "the covering hold is LIVE" */

The LOCK bit does NOT prove the hold is OURS.  When the frame's answer is
stale and a PEER has since taken the word, this arm reads the peer's
exclusion as confirmation of our own -- the ownership-from-word-state
inference the tree forbids elsewhere ("a word-check inside the
caller_holder arm would be UNSOUND -- a peer can hold that word
post-release").  Because the fast path is consulted BEFORE the
record-scan, finding A's designed REFUSE never runs: the stale answer is
laundered into a silent dedupe.

That also explains the spacing dependence without any two-system story:
at root-only every member collapses onto one anchor word, so "a peer
holds the very word my stale entry names" is the common case rather than
a coincidence.  It is intermittent (~1 run in 2) because it needs the
peer to take the word inside the stale window.

## Consequence for the fix directions

D1 (fence-under-anchor) addresses a mechanism that is not the one
measured; it should NOT be built on this evidence.  Finding B is the
same defect as finding A -- a hold answer outliving its exclusion -- so
it takes finding A's remedy, applied to the lane that is still
un-scrubbed:

  E1. SCRUB THE EXTRAS AT EVERY CONSUMPTION POINT, not only the rekey
      fold's.  The fold already scrubs at `marks_consumed = true`
      (txn_owned marks -> shared); the class fix is the same discipline
      wherever a txn consumes marks it was handed.  ☞ NEXT PROBE: print
      the stale entry's filing site and @txn_owned at the violation, the
      way the refusal arm already does (it reports lane=extra
      txn_owned=1 on the inv leg) -- that names the consumption point
      missing its scrub.
  E2. AND/OR make the fast path stop laundering: a stale answer should
      reach the refuse logic instead of being confirmed by a bit that
      may be a peer's.  Distinguishing our LOCK from a peer's is not
      possible from the word alone -- only a release RECORD in a live
      descriptor (the closing lane) or the ledger can do it -- so this
      is a protocol question for Mathieu, and it is exactly R1 of the
      truthful-ownership brief.

The brief's premise ("finding A's glue lane and finding B are the same
defect seen from two sides") is CONFIRMED -- but the shared defect is
truthful holds(), not fence-vs-anchor composition.  R2
(fence-under-anchor) loses its evidence; R1 and R3 carry the whole
redesign.


---

# ☠ CORRECTION (same evening, one probe later): THE ABOVE OVERREACHED

The section above concluded "the claimant's answer is the stale one, and
the owned-mask fast path laundered it".  That was inferred, not
measured, and the next probe REFUTES it.  Asking the claimant's OWN
thread-local ledger at the violation:

    FT EXCLUSION VIOLATION: node N claimed at ft_node_recompact:1357
      (anchor N=SELF) while owned by tid T2 (from
      ft_chain_compress_fused:1009, anchor N=SELF)
      ledger CONFIRMS the claimant holds it (so the OWNER's stamp is
      the stale one)
      claimant DEDUPED via lane: extra

The claimant's dedupe is CORROBORATED by a second, independent hold
system: it acquired the word earlier in the same op and never dropped
it.  So the stale entry is the OWNER's stamp, not the claimant's answer,
and the fast path laundered nothing.

## What is actually established, and what is not

ESTABLISHED (measured, not inferred):
  1. Both sites are DLM acquire-sets, not fences (source-verified:
     ft-mutation-node.h:1357 and ft-remove.h:1009 ARE the
     ft_dlm_acquire_set calls).  The original fence-vs-anchor mechanism
     is refuted on this evidence and D1 should not be built on it.
  2. Both ops derive the SAME anchor, and it is the MEMBER ITSELF
     (=SELF) -- at ROOT-ONLY, where every anchor is expected to be the
     root word.  That anomaly is unexplained and is the most suspicious
     fact on the table.
  3. The claimant deduped via the EXTRA lane, and its ledger agrees.

NOT ESTABLISHED -- and the trap to avoid: "the ledger confirms it" is
only as strong as the DROP discipline, which is exactly what a stale
stamp calls into question.  If a release can leave a stamp behind, the
same release can leave a LEDGER ENTRY behind, and then both of the
claimant's witnesses are stale together and T2 is the true holder.  The
word itself cannot arbitrate: a LOCK bit names no owner.  So the honest
statement is:

    ONE of the two hold-tracking systems holds an entry that outlived
    its hold, and no static reading of either can say which.

## The next instrument (do not guess again)

The remaining question is a HISTORY question -- who set the word's
current LOCK bit last, and whether the loser's release ran -- and the
tree already has the tool for exactly this wall: LTTng in flight-recorder
(snapshot) mode, small per-CPU buffers, a violation event emitted from
ft_owner_stamp_claim with the abort, then read the last events before the
violation (CLAUDE.md's own root-cause recipe; rig notes in
reference_ft_lttng_dlm_anchor_rig_2026_08_20).  Trace the acquire, the
drop and the release of ONE word: the last writer of the LOCK bit and
the presence or absence of the loser's drop settle it in one trace.

Two candidate mechanisms to test with it, in order:
  M1. A release that DROPS BY A DIFFERENT KEY THAN IT FILED BY.  The
      entry is filed under the derived anchor (here =SELF); if the
      release re-derives the anchor from a different context (or drops
      the node while the entry is under an ancestor, or vice versa), the
      entry and its stamp both survive the release.  This is the
      acquire-vs-release form of the SAME derivation-disagreement class
      the splice bug turned out to be (@ec9e68f8) -- one op, two
      derivations of one node's word.
  M2. The =SELF anchoring itself at root-only: ft_anchor_meta returns
      the node for depth 0, and ft_descent_anchor_of returns the
      descent's own cursor when the descent never advanced.  A member
      dated 0 that is NOT the root would anchor on itself while a
      correctly-dated peer anchors on the root -- a real exclusion gap,
      and the one the spacing refusal exists to prevent.  M2 would show
      as an ANCHOR DISAGREEMENT in the stamp, which has NOT been
      observed -- so it is the weaker candidate, but it must be
      explained rather than left as an anomaly.
