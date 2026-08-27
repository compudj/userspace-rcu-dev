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
