# Phase 4.2 — atomic detach: scope + site inventory (2026-07-02)

> ENGINE STATUS (verified 2026-07-03): FT already commits through the **concurrent MCAS
> engine** (`urcu_txn` / `urcu_mcas`), NOT the single-writer `urcu_txn_sw`. The swap landed
> @ **dd57e6b4** (2026-06-30) — `struct ft_flip_txn` embeds `struct urcu_mcas_txn`;
> `ft_flip_txn_commit` runs `urcu_txn_commit_flavor` in a `do{}while(ABORT)` retry loop; 0
> `urcu_txn_sw_*` symbols remain under src/fractal-trie/. So plan-Phase-4.1 is DONE, and every
> atomic-detach fusion below already commits as an atomic MCAS detach (the fused txn IS the MCAS
> txn). It runs under RETAINED caller exclusion (`FEATURE_FT_EXCL_VALIDATE`), so the retry loop is
> dormant (0 contention). **NOT yet wired: the Invariant-2 VALIDATE side** (`urcu_txn_load_validate`
> on edge-publish-into-live-P + guarded chain-append) — 0 uses in FT; that is the current front
> (see the VALIDATE-SIDE INVENTORY appended at the end of this doc).

## PROGRESS (2026-07-02, branch ft-txn-integ) — single-node category COMPLETE
All single-node "easy" retires are fused (or verified no-op / gap-closed):
- **1,2** insert-split → d71fb2c7 (ic->txn)
- **3** insert recompact-relocate → 83af4cc9 (ic->txn; dedups site-2)
- **8** remove DEL-recompact → e5d7b5d0 (commit_txn; residual: list-off lone shrink standalone)
- **19** graft attach recompact-relocate → 872cf96e (glue->txn)
- **22** graft nil-key src-root → f3e47f2e ; **23** graft_swap dst-root → 286f610f
- **24** graft_swap extract transient root → 872cf96e ; **27** merge_at empty-dst → f3e47f2e
- **31** compact recompact-relocate → 6bde2478 (+ the `retire_txn` param threaded through
  ft_node_recompact/ft_node_set_nth_rec/ft_node_replace_ptr; reader state-word resolve bdfa3645)
- **9** compressed->fresh-internal detach retire → da653f6f — was a genuinely MISSING mark
  (verified UNEXERCISED: 0 probe hits across ft_unit + ft_inv both modes); added standalone,
  folds into the detach-branch category fusion later.
- **32** destroy root → already a deliberate teardown no-op mark (ft-lifecycle.h:790), no unlink
  to fuse — NO ACTION.

MECHANISM: `retire_txn`/op-txn NULL → standalone lone-edge flip; non-NULL → tombstone recorded
INTO the commit txn (dedups same state slot via urcu_txn_store upgrade=1). Reader resolves the
state-word proxy via `ft_meta_nr_child_load`; the count-walk `ft_subtree_key_count` reads child
slots/external_nodes not meta->state so it is transparent (the inv_nr_keys_undercount flake is
pre-existing, unrelated). Each commit validated default + audit (0 tombstone asserts) + fault.

MULTI-NODE/GLUE — GRAFT-FAMILY DONE @d3b5baa2: added `ft_glue.fuse_free_list` flag; when set,
ft_glue_tombstone_free_list records each free_list retire INTO g->txn (dedup via urcu_txn_store
upgrade=1) instead of a standalone flip. Set on the 3 committers whose free_list is floor-bounded
(cap_free==FT_GLUE_FLOOR_FREE==8, graft never calls ft_glue_reserve): cds_ft_graft create path,
graft_swap insert-side, merge_at-graft — each +FLOOR_FREE reservation. Audit+fault both modes 0 asserts.

MERGE DST-SIDE gd DONE @3b0857fb: same flag, first VARIABLE reservation. cds_ft_merge grows gd's
free capacity via ft_glue_reserve (cnt.nf_dst+8), so the structural-txn reservation bumps by
`gd.cap_free` (exact upper bound; ft_glue_defer_free asserts nr_free<cap_free) and `gd.txn = txn`
aliases the merge commit txn so ft_glue_apply_deferred (ft-merge.h:~1580, past the last fallible
step) records the retires into it. Flag set ONLY on the created+reserved txn (the rekey take() path
keeps standalone; gd.txn then a dead store). Validated: audit ft_unit 256 + ft_inv 56/56 on+off 0
asserts; audit+fault ft_unit 298/298; adversarial skeptic SURVIVES 5 targets (lost-mark, reservation
sizing, txn double-free, rekey, ordering — the fused path strictly REMOVES the benign
tombstoned-but-reachable window vs the standalone flip).

GRAFT REKEY take() DONE @098e325c: the last graft-family glue free-list committer left standalone.
ft_merge_at_inner's same-trie rekey pre-sizes pf_txn before its detach; its m==0 (graft) sizing branch
is the ONLY non-NULL pre_txn source reaching ft_graft_keylen's take() (m>0 → ft_merge_spine_copy; detach
reshapes only the disjoint src path ⇒ m==0 iff graft reached). Bumped that branch by +FT_GLUE_FLOOR_FREE
(exact bound: graft never ft_glue_reserve ⇒ cap_free stays FLOOR_FREE) + set glue.fuse_free_list
UNCONDITIONALLY in ft_graft_keylen (both txn sources now carry the headroom). Symmetric completion of
d3b5baa2 ⇒ every graft-family committer's free-list now freezes atomically with its unlink. Validated:
audit ft_unit 256 + ft_inv 56/56 on+off 0 asserts; audit+fault ft_unit 298; skeptic SURVIVES all 5 targets
(under-reservation / lost-mark / freeze-then-strand / double-free / reader-ordering).

GRAFT_SWAP EXTRACT-SIDE DONE @053e529e: glue_extract (swap_ft) free_list = AT MOST 1 node (the peeled
compressed old_child that ft_make_root_internal_glue defers @ft-cluster-build.h:714, only when old_child
is compressed ⇒ top_B non-NULL). Pointed glue_extract.txn at extract_txn (post-drain root-install txn) +
set fuse_free_list, so the retire freezes atomically w/ the root install. KEY invariant: free_list
non-empty ⟹ compressed old_child ⟹ top_B ⟹ extract_txn created+reserved ⟹ never NULL-txn deref (the
!gs_ord && !top_B path keeps empty free_list + standalone default). Bumped extract_txn reservation +1/arm
(FT_ROOT_LIST_SWAP_MAX_EDGES+2 list-on; 3 list-off — list-off is EXACTLY tight at 3, the fused free-list
tomb is a DISTINCT node from the transient-root tomb ⇒ own slot). n≥1 whenever free_list non-empty ⇒
extract_txn always committed not destroyed. Validated: audit ft_unit 256 + ft_inv 56/56 on+off 0 asserts;
audit+fault ft_unit 298; skeptic SURVIVES all 6 targets.

MERGE SRC-SIDE gs ROOT-SRC DONE @6c21a81c (Option B: root-src fused + non-root moved past abort):
- P1 root_src+ms_ord: gs.txn=src_side_txn + fuse, tombstones recorded before ft_root_list_swap_publish;
  bumped reservation to FT_ROOT_LIST_SWAP_MAX_EDGES + gs.cap_free.
- P2 root_src+list-off+gs.nr_free>0: NEW txn (1 + gs.cap_free), record_reserved root edge (normalizes to
  same FT_FLIP_PROXY_TAG) + tombstones + commit; EMPTY gs keeps bare ft_root_edge_flip (no grace period).
- P3 non-root: MOVED the standalone stamp to AFTER ft_merge_unlink_src_subtree returns 0 (past its abort)
  ⇒ removes the freeze-then-strand (the actual defect); freeze-after-unlink SAFE (nodes already unlinked
  by ft_detach_node's flip). gs stamped exactly once/path (root block vs `if(!root_src)` mutually excl).
Validated: audit ft_unit 256 + ft_inv 56/56 on 0 asserts; fault ft_unit 298; skeptic SURVIVES all 6.
inv_nr_keys_undercount list-off flake = PRE-EXISTING (repro'd on base ~2/6, count-walk rank_stats-OFF
overcount, [[project_ft_countwalk_split_uaf]]).
MERGE SRC-SIDE gs NON-ROOT (P3) DONE @cc6dd0c4 (full fusion into ft_detach_node's commit_txn):
- ft_detach_node gained a `struct ft_glue *retire_glue` LAST param; branch-2 commit_txn reservation
  bumped by retire_glue->cap_free; the post-replace_ptr freeze block records the glue free-list into
  commit_txn (gs.txn=commit_txn, fuse_free_list) beside the site-7 orphan freeze when pub && commit_txn.
- PROBE (MSPROBE over 138725 LIST-ON merge-stress detaches): move-style src detach is 100% branch-2 +
  pub!=NULL + commit_txn present (0 branch-1, 0 shape-D) ⇒ LIST-ON always fuses. LIST-OFF (pub==NULL,
  run==NULL) ⇒ freeze block's `pub` gate fails ⇒ standalone FALLBACK after end: (gated !ret && !fused),
  byte-identical relocation of the old caller standalone. gs freed CALLER-side (ft_glue_free_old ~1734)
  so its free always follows every detach commit (no hoist needed, unlike site-7's own orphans).
- ft_merge_unlink_src_subtree threads retire_glue → ft_detach_node; full-merge caller passes &gs, merge_at
  passes NULL (src-excise preserves payload, no overlap-spine copy). Deleted the `if(!root_src)` standalone.
  5 non-merge ft_detach_node callers pass NULL (strict no-op). gs.txn nulled on return (defensive).
Validated: audit ft_unit 256 + ft_inv 56/56 on+off 0 asserts; default 256; fault 298; ASAN ft_inv 56/56
both modes 0 leak/double-free/UAF; TWO adversarial skeptics BOTH SOUND (8 hazards each; the 2nd added a
negative-control: neutering the mark ⇒ audit aborts, proving the fuse path is exercised + load-bearing).
RESIDUAL: LIST-OFF merge-src (pub-less) still standalone via the fallback — same "force-txn for the
lone-edge shape" dependency as site-7's list-off pub-less lone store.

===== PHASE 4.2 PAUSED @4a069957 (Mathieu 2026-07-03) — all SAFE tombstone-relocation fusions DONE =====
The 2 sites below need NEW infra whose cost is a single-writer-PRESENT loss for a multi-writer-only gain,
so deferred until the concurrent engine is closer:
  (a) chain-leaf family (sites 5,11-18): needs a chain-next READER PROXY in cds_ft_node_next_rcu.
  (b) list-off force-txn residual (site-7 in-place pub-less + merge-src-non-root list-off fallback):
      needs forcing a txn onto the hot list-off remove path (benchmark-gated).

GLUE RESIDUAL (variable reservation) — ALL DONE: gd (merge dst) @3b0857fb; graft rekey take() @098e325c;
graft_swap extract-side @053e529e; merge src root-src @6c21a81c; merge src NON-ROOT (P3) LIST-ON @cc6dd0c4
(LIST-OFF pub-less → force-txn residual (b)); **graft_swap KEY_SHORTER @4a069957** = closes graft_swap. KEY:
the "LIVE dst compressed-wrap re-parent not txn-classified" blocker was ORTHOGONAL to the freeze — apply_deferred
tombstones the free-list (into g->txn iff fuse_free_list) SEPARATELY from applying the immediate back-edges, so
route ONLY the free-list freeze into the existing glue_publish_txn (bump +FT_GLUE_FLOOR_FREE=14), set
glue_insert.txn INSIDE the else branch (after the dispatch) and PAST the last prep_oom (else double-free), leave
the re-parent immediate. Neg-control (FT_KS_NEUTER) audit-aborts ⇒ path exercised; skeptic SOUND (8 hazards).
- **Detach-branch — 3/4 DONE**: site 10 @e2bc9e70 (ft_chain_compress_fused's own ≤3 merge retires
  → its txn, +3), site 6 @3dc1cce5 (variable orphan chain up to FT_MAX_DEPTH → caller-created txn
  sized 8+nr_to_free+trailing, threaded to replace_compressed_parent which now takes a caller txn;
  create_bounded mallocs the exact runtime cap so no inline limit), site 9 @3dc1cce5 (compressed→
  fresh-internal retire folded into the same txn). KEY: ft_remove_commit_rec always takes the
  ft_ord_cell_flip_into(txn) branch when txn!=NULL, merging the ≤8 structural edges into the txn that
  already holds the orphan tombstones → atomic. All audit+fault both modes 0 tombstone asserts.
  **site 7 DONE (partial fusion; recompaction + list-off residual)**: dropped the shared standalone
  mark loop; new helper `ft_detach_freeze_orphans(ft, txn, orphans, nr, trailing)` (txn!=NULL → record
  into txn; NULL → standalone flip). Per commit path: (a) shape-D ft_chain_compress_fused — orphans
  threaded as 3 new params, recorded into its own merge flip, reservation +nr_orphans+trailing; 3 other
  callers pass NULL/0/NULL. FUSED both pub modes. (b) in-place delete (pub->armed) — recorded into
  commit_txn (reserved +nr_to_free+trailing; gate gained `|| (pub && orphans)`), committed by
  ft_remove_one_commit BEFORE the free. FUSED. KEY DISCOVERY (via FT_SITE7_PROBE joint-distribution
  run): the freeze must be applied AFTER ft_node_replace_ptr, because the RECOMPACTION path defers its
  forward republish to the later "Update address of parent ptr" block (~1243+) which runs AFTER the
  orphan free (~1218) → recording into commit_txn commits the tombstone AFTER the free (audit assert).
  So (c) recompaction (pub unarmed) + list-off pub-less direct-lone-store initially kept a STANDALONE
  freeze before the free = RESIDUAL (~350k/550k branch-2 retires in ft_inv list-off stress). Probe also
  CONFIRMED the trap is real (2× pub==NULL + orphan-chain + direct-lone-store in list-off stress) so (c)
  is load-bearing, not just theoretical. Validated: audit ft_unit 256 + ft_inv 56/56 on+off (0 asserts)
  + audit+fault 298 + skeptic. Commit: fb6119d0.

  **FREE-HOIST follow-up @bff17072**: RECOMPACTION now FUSED too. The recompaction residual was because
  its forward republish is deferred to the "Update parent ptr" block (~1265) which ran AFTER the orphan
  free (~1218). Fix = HOIST the branch-2 orphan free past the republish: copy to_free→function-scope
  orphan_free[]/orphan_trailing on success (gated by free_orphans_pending), reclaim just before end:. Now
  the free follows WHICHEVER commit unlinked the chain, so the freeze gate simplifies to `(pub &&
  commit_txn)` (fuses in-place AND recompaction; pub!=NULL always reserves+consumes commit_txn). Bonus:
  the free now follows the structural unlink (recompaction previously freed BEFORE unlink, RCU-grace-
  covered). ONLY RESIDUAL now = list-off pub-less DIRECT LONE STORE (~256k; no commit_txn → needs
  "force-txn for the lone-edge shape", same dep as merge-src-non-root P3). Validated: audit ft_unit 256 +
  ft_inv 56/56 on+off (0 asserts) + fault 298 + ASAN (leak+double-free) ft_unit 256 + ft_inv 56/56 both
  modes clean + skeptic SOUND (5 hazards; strictly-safer-than-HEAD free ordering).
- **Chain-leaf family 5, 11-15, 16-18** — CDS_FT_NODE_REMOVED_FLAG on node->next, lone-edge flips
  POST-unlink. Needs a NEW capability: chain-next reader proxy resolution in cds_ft_node_next_rcu
  (analog of ft_meta_nr_child_load), since a fused multi-edge commit plants a transient proxy on next.

---


Branch `ft-txn-integ`. Phase 4.2 = the **validate side** of Invariant-2, still under
retained caller exclusion (behavior-identical, gate-green). Three workstreams per
`doc/design/step4-concurrent-engine-plan.md` §3.3:

1. **Atomic detach** — fold each freeze MARK into the SAME `urcu_txn` as the structural unlink.
2. **Insert/publish load-validate** — `urcu_txn_load_validate(&P->state, LIVE)` when publishing an
   edge into node P (a concurrent remover that froze P aborts this commit; inverse automatic).
3. **Chain guarded-append** — remove-side tombstone as a txn edge on the freed node's own `next`;
   the append (already `ft_chain_next_flip`, class-E) validates against it.

## Engine readiness (Agent, 2026-07-02): NO engine change needed
- `urcu_txn_load_validate(txn, slot, tag)` EXISTS — `include/urcu/rcu-txn.h:538-546`; records a
  `{v→v}` guard edge, fails whole commit if slot moved at install (value-CAS at linearization pt).
- `urcu_txn_list_insert_after_guarded_rcu` — `rcu-txn-list.h:319-374` = ready template for
  "guard on an external per-node live word".
- `urcu_txn_store` value-CAS (`*slot==old` at install; mismatch → whole-txn ABORT) → folding mark
  into unlink gives atomic detach directly.
- `urcu_txn_reserve(edges+1)` budgets the extra guard/mark record; a guard on a slot a store also
  touches UPGRADES IN PLACE (one record/slot) → 0 extra edge in the common case.
- Monotonic LIVE→DEAD flag ⇒ no A-B-A ⇒ no generation counter needed (value-CAS caveat n/a).
- Status: OK=0 terminal, ABORT=+1 retry (encapsulated in `ft_flip_txn_commit` do/while),
  MEMORY_ERROR=-1 terminal.

## KEY FACT: today 100% of retire sites are 2 SEPARATE commits
Every mark helper (`ft_meta_tombstone_set_flip`, `ft_node_mark_removed_flip`,
`ft_chain_mark_removed_flip`) bottoms out in `ft_ord_cell_flip_one` / `ft_chain_next_flip`
(ft-mutation-helpers.h:658-712) — infallible on-stack single-edge flips NOT recorded into any txn.
So 4.2 is pure fusion. Docstrings say so (ft-mutation-helpers.h:656, 686: "should eventually ride
ONE flip (atomic detach); a lone-edge mark is the bridge").

**Ordering asymmetry to normalize when fusing:**
- Internal §4.B state tombstone (`ft_meta_tombstone_set_flip`) is applied **BEFORE** the unlink commit.
- Chain-leaf REMOVED flag is applied **AFTER** the successful unlink (sits in the `ret==0`/`else` arm)
  — folding it into the unlink txn changes its ordering relative to the parent-slot flip.

## Site table (op | mark file:line | unlink commit file:line | today | freed node)
| # | op-group | mark | unlink commit | today | node |
|---|---|---|---|---|---|
| 1 | insert / split old compressed | ft-insert.h:216 (pre) | ft-insert.h:230 commit(ic->txn); free 239 | 2 | PUB |
| 2 | insert / split old internal | ft-insert.h:219 (pre) | ft-insert.h:230; free 241 | 2 | PUB |
| 3 | insert / recompact-relocate attach node | ft-mutation-node.h:1250 | ft-insert.h:230 (RELOCATE fwd in ic->txn); free 1279 | 2 | PUB |
| 4 | insert / recompact during invisible split-build | mutation-node.h:1250 | none (never published); free ft-insert.h:537 | mark=no-op | FRESH |
| 5 | replace (cds_ft_replace) old chain leaf | ft-insert.h:2760 (post) | ft-insert.h:2723/2751 swap_publish_multi/flip_into | 2 | PUB leaf |
| 6 | remove / detach-branch orphan chain br.1 | ft-remove.h:751,771 (HOISTED, pre) | ft-remove.h:777 replace_compressed_parent→commit_rec; free 785/788/792 | 2 | PUB MULTI |
| 7 | remove / detach-branch orphan chain br.2 | ft-remove.h:950-958 (HOISTED, pre) | 1003 chain_compress_fused OR 1065 replace_ptr / 1089 remove_one_commit; free 1104/1109/1113 | 2 | PUB MULTI |
| 8 | remove / detach recompaction old node | mutation-node.h:1250 | ft-remove.h:1163 or 1195 commit_rec; free 1236 | 2 | PUB |
| 9 | remove / replace_compressed_parent compressed→fresh-internal | **NONE FOUND** | ft-remove.h:185 commit_rec; free 187 | 2 | PUB — **MARK GAP?** |
| 10 | remove / chain_compress_fused merged-node replace | ft-remove.h:383,385,387 (pre) | ft-remove.h:389 commit_rec; free 392/394/396 | 2 | PUB MULTI≤3 |
| 11 | remove / dup-chain leaf, compressed-holder last | ft-remove.h:1624 (post) | ft-remove.h:1618 detach_node | 2 | PUB leaf |
| 12 | remove / dup-chain leaf, prefix last (shape-D) | ft-remove.h:1684 (post) | ft-remove.h:1678 chain_compress_fused | 2 | leaf |
| 13 | remove / dup-chain leaf, prefix last (non-fused) | ft-remove.h:1723 (post) | ft-remove.h:1715 remove_one_commit | 2 | leaf |
| 14 | remove / dup-chain leaf, body-child last | ft-remove.h:1789 (post) | ft-remove.h:1783 detach_node | 2 | leaf |
| 15 | remove / ft_unchain_node head/non-head unlink | ft-remove.h:1439 (post) | ft-remove.h:1427-1431 flip_try / prev→next relink | 2 | leaf |
| 16 | remove_all / bulk chain shape-D | ft-remove.h:2106 chain_mark_removed_flip (post) | ft-remove.h:2098 chain_compress_fused | 2 | MULTI |
| 17 | remove_all / bulk chain non-fused clear | ft-remove.h:2161 (post) | ft-remove.h:2139 remove_one_commit / 2157 flip_one | 2 | MULTI |
| 18 | remove_all / bulk chain leaf detach | ft-remove.h:2194 (post) | ft-remove.h:2188 detach_node | 2 | MULTI |
| 19 | graft / attach recompact-relocate dst node | ft-graft.h:631 (pre) | ft-graft.h:634 commit(glue->txn); free 640 | 2 | PUB |
| 20 | graft / attach glue free-list | mutation-helpers.h:2988 (via glue_apply_deferred 570/3003/3064) | 3118 glue_txn_commit_edges; free ft-graft.h:642/1231 | 2 | PUB MULTI |
| 21 | graft (internal FT_GRAFT_PREP_GLUE) glue free-list | mutation-helpers.h:2988 | 3118 (via ft-merge.h:1921); free ft-merge.h:1923 | 2 | MULTI |
| 22 | graft / nil-key src-root wrapper | ft-graft.h:1148-1150 (pre) | ft-graft.h:1171 root_list_swap_publish(src_retire_txn); free 1314 | 2 | PUB |
| 23 | graft_swap / root-swap dst old root | ft-graft.h:902 (pre) | ft-graft.h:904 root_list_swap_publish_dual; free 925 | 2 | PUB |
| 24 | graft_swap / extract transient root | ft-graft.h:2230-2232 (pre) | ft-graft.h:2242 flip_into / 2248 flip_one; free 2254/2257 | 2 | PUB |
| 25 | graft_swap / insert-side glue free-list | mutation-helpers.h:2988 (via ft-graft.h:2010/2013) | 3118 / ft-graft.h:2014 publish_replace; free 2262 | 2 | MULTI |
| 26 | graft_swap / extract-side glue free-list | mutation-helpers.h:2988 (via ft-graft.h:2137) | extract publish 2242/2248; free 2263 | 2 | MULTI |
| 27 | merge_at / empty-dst root swap | ft-merge.h:2441 (pre) | ft-merge.h:2450 root_list_swap_publish / 2464 root_edge_flip; free 2470 | 2 | PUB |
| 28 | merge_at / dst-side glue free-list (interleave) | ft-merge.h:1562→2988 (pre) | ft-merge.h:1631 commit; free 1658 | 2 | MULTI |
| 29 | merge_at / src-side glue free-list | ft-merge.h:1485 tombstone_free_list (pre) | 1502/1515 root swap OR 1519 unlink_src_subtree→detach_node; free 1657 | 2 | MULTI |
| 30 | detach primitive (ft_detach_subtree) glue free-list | ft-detach.h:492→2988 | extract/root publish same fn; free ft-detach.h:519 glue_free_old | 2 | MULTI |
| 31 | compact (cds_ft_compact) recompact-relocate node | mutation-node.h:1250 (RELOCATE) | rec committed by caller commit_rec; RCU-deferred free | 2 | PUB |
| 32 | destroy (cds_ft_destroy) root | ft-lifecycle.h:790 (pre) | none — teardown free 791, no unlink | 1 flip+free | PUB no-op |

Published free loop for glue sites (20/21/25/26/28/29/30): `ft_glue_free_old`
mutation-helpers.h:3311-3320 (asserts mark via free_cds_ft_node/free_compressed_node
ft-helpers.h:2249/2322).

## Fusion difficulty
- **Single-node fuse (easy: mark + parent-slot in ONE flip):** 3, 5, 8, 9, 19, 22, 23, 24, 27, 31, 32.
  Rows 1+2 retire ≤2 nodes into one ic->txn forward commit — fuse cleanly. Chain-leaf 11-15 are a
  clean 2-word fuse (leaf next REMOVED bit + head-slot/prev/external_nodes relink) but the mark
  currently applied AFTER unlink must move into the txn.
- **Multi-node/bulk retire (harder: N nodes freed per unlink; edge-budget pressure):**
  6, 7, 10 (detach-branch + chain-compress, up to FT_MAX_DEPTH orphans / ≤3 merged), the whole
  glue free-list family 20/21/25/26/28/29/30 (a replaced cluster), remove_all 16-18 (whole chain).
  Fusing = folding N per-node tombstone edges into the unlink txn. Budgets:
  `FT_REMOVE_COMMIT_REC_MAX_EDGES`=7, insert txn bounded to 9 (ft-insert.h:305), glue txns reserved
  to cluster size.

## Flags to resolve during 4.2
1. **Possible MISSING mark — site 9, ft-remove.h:187:** `ft_detach_node_replace_compressed_parent`
   sub-case 2 (compressed→fresh-internal recompaction) publishes via commit_rec (185) then
   `free_compressed_node(...)` (187) with NO co-located `ft_meta_tombstone_set_flip` in lines
   156-190. Under `-DFT_DEBUG_TOMBSTONE_AUDIT` the assert at free_compressed_node (ft-helpers.h:2322)
   would fire unless the path is unexercised or the node is marked upstream. VERIFY — may be a genuine
   missing mark, not a fuse candidate.
2. **Fresh nodes freed via PUBLISHED free API (audit edge cases):** ft-cluster-build.h:151/159 (glue==NULL
   error path frees never-published branch), ft-graft.h:2307 (prep_oom frees transient fresh swap root).
   Should use the `*_unpublished` variants (ft-helpers.h:2265/2338) or the audit assert trips. Row 4
   (ft-insert.h:537) survives only because ft_recompact_node unconditionally marks at 1250.
3. **Ordering normalization:** internal-state marks are pre-unlink; chain-leaf REMOVED marks are
   post-unlink (in ret==0/else arms). A single fused MCAS must pick one side; chain-leaf marks
   currently rely on the unlink having already succeeded.

---

# VALIDATE-SIDE INVENTORY (2026-07-03) — the OTHER half of Phase 4.2

The atomic-detach half (fuse the freeze MARK into the unlink txn) is DONE. This is the scope of the
**validate side**: guards so that, once concurrent per-trie writers are enabled, a writer publishing an
edge INTO a live node P aborts if a concurrent remover FROZE P (set `FT_STATE_TOMBSTONE` in
`cds_ft_metadata.state`). Built from a 3-agent per-site classification of all 29 structural record/publish
sites + independent read of the insert forward-publish and the `nr_child` machinery.

## THE HEADLINE FINDING — automatic vs explicit hinges on the count word, and insert has an asymmetry

Insert's in-place safe-append (`FEATURE_FT_INSERT_IN_PLACE`, default on) mutates THREE live words on the
target node P: the child pointer at `qp_pointers[qp_ptr_idx]` where `qp_ptr_idx = popcount(qp_bms)`
(mutation-node.h:153/172), the occupancy **bitmap word** (`data[4]`/`data[8]`/`data[0]`,
:157/180/291/295), and `nr_child` (`ft_meta_nr_child_inc`, :128/160/186…). Two of these can NEVER be
MCAS-transacted:
- **The occupancy bitmap word has no free bit for the engine's at-rest proxy tag** (bit 0 /
  `URCU_MCAS_TAG`). It is fully packed with occupancy bits; a word that cannot be tagged cannot carry a
  transactional descriptor, so it cannot ride an MCAS commit. (Mathieu, 2026-07-03.)
- **The pointer-array index `qp_ptr_idx` is derived from that same (racing) bitmap** — two concurrent
  appends read the same `qp_bms`, compute the same index, and clobber the same slot.

### RESOLUTION: in-place safe-append is SINGLE-WRITER-ONLY; multi-writer REQUIRES recompact-on-insert
Because the bitmap word can't be transacted and its derived index races, **no per-word edge conversion can
make in-place safe-append multi-writer-safe** — in particular the earlier idea "make insert's `nr_child++`
a committed edge" (`ft_meta_nr_child_inc_flip`) is **DROPPED**: it is unneeded for single-writer (in-place
is correct and reader-tolerant there) and insufficient for multi-writer (the bitmap + index still race,
and the bitmap can't hold a descriptor at all). The existing `NO_FEATURE_FT_INSERT_IN_PLACE` routing
(mutation-node.h:103-115: a new-occupancy insert on a LIVE node returns `-ERANGE` →
`ft_node_recompact(ADD_SAME)`) is therefore **not an option but the ONLY multi-writer-correct insert
path**: it builds a FRESH copy of P with the new child and publishes it by CASing the **grandparent's
child slot** (`old_P → fresh`). Classic copy-update — concurrent inserts into P both copy from P, one wins
the grandparent CAS, the loser retries and rebuilds from the winner; all three in-place hazards vanish
because the bitmap/index/count land on the build-invisible fresh copy, and the only reader-visible publish
is the single (taggable, transactable) grandparent-slot pointer edge.
- **In-place safe-append is RETAINED as the exclusive / single-writer fast path** (plan §3.6). It is
  selected today only because the POC keeps caller exclusion. Making the concurrent path recompact needs
  the currently *compile-time* `NO_FEATURE_FT_INSERT_IN_PLACE` to become a **runtime gate keyed on the
  concurrent-writer mode** (NOT `ft->exclusive`, which is a reader-side axis — a has-readers single-writer
  trie legitimately uses in-place today). This gate is a Phase-4.3 change: under the current retained
  exclusion every trie is single-writer, so in-place stays correct and there is nothing to change yet.

### Consequence for the A-GUARD buckets (insert)
On the recompact path insert NEVER publishes into a live P's child slot in place; it always relocates P via
the grandparent slot. So the "A-GUARD-AUTO via count-edge collision" idea is **moot for insert**:
concurrent inserts into P serialize on the **grandparent-slot CAS** (copy-update), not on `P->state`; an
insert vs. a concurrent removal *of P* is likewise auto-guarded (both CAS the grandparent slot,
`old == P`). The one residual freeze hazard is an insert relocating P while a remover freezes the
**grandparent** — guarded by the grandparent-relocation A-GUARD-EXPLICIT tombstone validate on
`grandparent->state`.

### RECOMPACT-UNIFORM BASELINE ⇒ nr_child is invariant ⇒ NO masked-validate needed (Mathieu, 2026-07-03)
Take the symmetric decision for REMOVE: make recompact the multi-writer baseline for remove too (not just
insert). Then a live node is **immutable except for a one-way LIVE→DEAD tombstone**: every count change
produces a fresh copy and retires the old node, so `nr_child` (state bits 2+) is INVARIANT for a node
object's life, and the ONLY `P->state` transition a live P ever undergoes is the freeze at retire (the
relocation's proxy rides the grandparent slot, not P's own state). ⇒ a plain FULL-WORD
`urcu_txn_load_validate(&P->state, v)` aborts iff P is being retired = EXACTLY the freeze guard, with zero
benign churn. **So the masked-validate primitive is NOT needed on the recompact baseline** — this reaffirms
the plan's "no engine change needed". Masked-validate (and the count-edge churn that motivates it) is the
COST OF THE IN-PLACE FAST PATHS, which are an opt-in optimization layer, coordinated as a pair:
  - remove soft-delete in-place: clear the child pointer (transactable) + `dec_flip` count edge, bitmap
    left sticky-set — multi-writer-safe (no untaggable-bitmap problem, unlike insert of a NEW byte);
  - fill-hole insert in-place: the sticky-bit FUTURE OPTIMIZATION below (`inc_flip` count edge).
  Enabling either reintroduces live-node `nr_child` churn ⇒ a pure-pointer-edge guard's full-word validate
  over-aborts (the root, count-oscillating and collapse-exempt, is the worst case) ⇒ THEN add masked-
  validate. Baseline first (uniform recompact, full-word validate); in-place + masked-validate as one
  later opt-in layer. The general count-edge collision principle below still describes what those in-place
  ops do when enabled; on the pure recompact baseline only remove's retire touches `P->state`.

## INVARIANT — liveness is the tombstone BIT, never the count (the empty-live-root proof)
The "automatic guard" is a SLOT-level MCAS collision on `&P->state` (any two txns storing that word
conflict; one aborts) — NOT a `nr_child == 0` deadness test. `FT_STATE_TOMBSTONE` (bit 1) and `nr_child`
(bits 2+) share the word but are semantically independent; the count is merely the vehicle that lands a
store on the same word, never the freeze signal. **The ROOT is the living proof they must never be
conflated:** the root (`metadata->parent == NULL`) is collapse-EXEMPT (ft-remove.h:615/618/639-640
`!is_root`), stays LIVE at `nr_child == 0` (empty trie), and is tombstoned ONLY at destroy/teardown
(ft-lifecycle.h:790, no concurrent writers) — NEVER during remove. So an empty live root has the tombstone
bit clear and passes every freeze validate. Two consequences baked into the buckets below:
  (i) ON THE RECOMPACT-UNIFORM BASELINE this is a non-issue: nr_child is invariant per node object (a count
      change recompacts the node, including the collapse-exempt root → a fresh empty-live root via
      `&ft->root`), so a full-word validate never sees benign churn and no masked primitive is needed (see
      "RECOMPACT-UNIFORM BASELINE" above). The masking concern applies ONLY IF the in-place fast paths are
      enabled: then the root's count oscillates 0↔1 in place on a hot word, a full-word validate over-aborts
      on that benign churn, and A-GUARD-EXPLICIT wants a MASKED bit-1 validate (the root never sets bit 1 in
      normal operation). So: masked-validate = a co-requisite of the in-place optimization layer, not the
      baseline.
  (ii) Root-slot publishes (insert 1264 / graft 592 when the grandparent IS the root ⇒ the flip targets
      `&ft->root`, no owner-node state) are ROOT-bucket, auto-guarded by the `&ft->root` slot CAS (two
      concurrent root relocations collide on `old == current_root`); the empty-live-root never
      participates (not being retired). No count, no node-state guard.

## GUARD BUCKETS (the actionable scope)

**CHILD-ADDING publishes (insert / graft-attach / merge-attach) — NO count-collision auto-guard; they take
the RECOMPACT-RELOCATE path under multi-writer.** Any op that adds a child to a LIVE node via in-place
safe-append hits the same bitmap-not-taggable + derived-index race as insert (above), so under concurrent
writers it MUST recompact the target node and publish the fresh copy via the **grandparent slot** — where
the guard is the grandparent-relocation A-GUARD-EXPLICIT (below), not a count-edge collision on the target.
The in-place variants stay on the single-writer/exclusive fast path. Sites that are child-adds in-place
today (insert 273/1209, graft 612 in-place arm, merge 1663 in-place arm, glue ft-mh:3135 when
`publish_parent` gains a child) therefore carry NO node-state guard themselves; their multi-writer form is
the relocation edge on the grandparent. (Displaced-external / already-occupied-slot variants change no
count and never recompact — they publish in place and fall to A-GUARD-EXPLICIT.)

**A-GUARD-EXPLICIT** — forward publish into a LIVE P with NO count change on P ⇒ needs an explicit
tombstone check `urcu_txn_load_validate(&P->state, live)`. On the RECOMPACT-UNIFORM BASELINE a plain
FULL-WORD `load_validate` is exact — nr_child is invariant, so the only `P->state` change is the freeze
(see "RECOMPACT-UNIFORM BASELINE"). A masked-validate is needed ONLY once the in-place fast paths (in-place
remove / fill-hole insert) are enabled and churn nr_child in place; treat it as a co-requisite of that
optimization layer, not the baseline. Sites:
- insert 332 + list-off twins 1879/2279/2315 (park into `&P->state`'s sibling field `external_nodes`; P=`d.nf`)
- insert 1264 / graft 592 (recompact-relocation: same-count pointer swap into the LIVE **grandparent** slot;
  P = `d.ppnf` / `st->publish_pmeta->parent`; ROOT sub-case when the grandparent slot is `&ft->root`)
- remove 2180 / 2349 (list-off `external_nodes` clear on the LIVE holder; P = `metadata`/`holder_meta`)
- remove ~1556 promote-head forward (P = `parent_nf`)
- glue-path replaces graft 1273 / 2088 / 2105, merge 2001 (publish into LIVE dst `publish_parent`;
  guard where the glue helper records its forward `pub_slot` edge = ft-mh:3135)
- **CAVEAT:** list-off `external_nodes` sites (332/1879/2279/2315/2180/2349) are currently ON-STACK
  lone-edge `ft_ord_cell_flip_one` (plain rcu_assign) — they must FIRST be routed onto an MCAS txn before
  any guard can attach.

**B-CHAIN** — duplicate-chain publishes; the fix is to keep them as REAL recorded CAS edges (not the
degraded lone-edge store), so the recorded `old` value auto-guards: a concurrent freeze sets
`CDS_FT_NODE_REMOVED_FLAG` on the SAME `next` word ⇒ `old` mismatches ⇒ abort. No separate primitive.
- insert 1334 (append, old==NULL onto live tail), ft-mh:3272 (run-splice append, old==NULL)
- insert 2694 (replace-relink, old==old_node into live predecessor `next`)
- remove 1586 (chain unlink/relink, old==node) — CHAIN-LEAF class (next-word REMOVED flag, not `->state`);
  part of the DEFERRED chain-leaf reader-proxy residual, not this pass.

**BACK-EDGE(live)** — writer-only up-walk fields (`meta->parent` / external `prev` / `cell->parent`) on a
LIVE re-parented node. NOT a child-array publish; the reanchor up-walk is the only reader. Likely already
safe (each is FUSED into its one commit, closing the back-channel window), but needs the reanchor-reader
hazard write-up before declaring done. Sites: insert 132, ft-mh 2831/2842/2864/2867/2892, merge 1658 loop.

**OUT OF THIS PASS (no internal-node `->state` guard applies):**
- **ROOT / cross-trie** (the trie-root pointer has no owner-node state word; hazard is straddler/ABA,
  handled by the §3.4 `synchronize_rcu` drain between commit1/commit2; source/swap trie is consumed-
  exclusive, dst-appear swaps from an empty root): graft 1201/1213/1221/2066/2073, graft dual 908/1592,
  merge 1552/1567/1583/2539/2555, detach 198.
- **LIST-EDGE / ORD-CELL** (ordered-cell doubly-linked list neighbours carry NO `metadata->state`; their
  freeze is the chain-leaf/list next-mark = the DEFERRED reader-proxy mechanism): graft 618, merge 1691,
  ft-mh 3153, ft-mh 1092 (passthrough loop).

## FRESH (no guard — owner is a build-invisible node)
insert 1209 (recompact sub-case; guard shifts to 1264), graft 612 (recompact sub-case; guard shifts to
592), detach 507 (`m->parent=NULL` on the fresh detached root).

## ENGINE QUESTION — RESOLVED: no engine change on the recompact baseline
Earlier this section floated a **masked `load_validate`** (`(*slot & mask) == expected`) as a candidate
engine addition to avoid full-word over-abort on `nr_child` churn. Under the RECOMPACT-UNIFORM BASELINE
that concern DISAPPEARS: nr_child is invariant per node object (only the freeze mutates a live `P->state`),
so plain full-word `load_validate` IS exact. ⇒ the plan's "no engine change needed" holds. Masked-validate
becomes necessary ONLY if/when the in-place fast paths (in-place remove / fill-hole insert) are enabled —
it is a co-requisite of THAT optimization layer, decided together with it, not a baseline blocker.

## FUTURE OPTIMIZATION (Mathieu, 2026-07-03) — sticky occupancy bit lets some inserts skip recompact
The recompact-on-insert requirement is driven by the bitmap word being untaggable. But if REMOVE leaves the
occupancy bit SET (the sticky soft-delete already done for pigeon, and popcount already soft-deletes), then
a later insert into that SAME slot finds the bit already set ⇒ it is a fill-the-hole in-place REPLACE of a
NULL pointer (set_nth Case 1, mutation-node.h:118-133): the **bitmap word is not modified** and the index
`qp_ptr_idx` is **stable** (bitmap unchanged, no index race). Such an insert can commit multi-writer
IN PLACE — the pointer slot IS taggable/transactable — WITHOUT recompaction. The only remaining live-word
change is `nr_child` (NULL→child ⇒ +1), which THEN needs to be a committed edge on `P->state`: this is
exactly where the dropped `ft_meta_nr_child_inc_flip` would resurface (fused into the fill-hole commit,
mirror of remove's `dec_flip`). NOT for now — recompact-on-insert is the baseline; this is a perf follow-up
that trades a node copy for a sticky bit + a count edge on the hot fill-hole path. Prereq: make the sticky
bit consistent across layouts + confirm readers/writers tolerate the wider soft-delete holes.

## SUGGESTED LANDING ORDER (all behavior-identical under retained exclusion, gate-green)
1. **B-CHAIN** — restore the 3 chain publishes (1334/3272/2694) to recorded CAS edges (drop the lone-edge
   degrade). The recorded `old` auto-guards under multi-writer; behavior-identical under one writer.
2. **A-GUARD-EXPLICIT plumbing** — route the list-off `external_nodes` lone-edges (332/1879/2279/2315/
   2180/2349) onto their MCAS txn, then add the FULL-WORD tombstone `load_validate` on the LIVE holder/
   parent (no-op under exclusion; exact on the recompact baseline — no masked primitive).
3. **Grandparent-relocation guard** — full-word tombstone validate on the grandparent for the child-adding
   recompact-relocate publishes (insert 1264 / graft 592 / merge). No-op under exclusion.
4. **BACK-EDGE reanchor write-up** — argue or guard the 7 up-walk re-parents.
5. **(Phase 4.3)** runtime-gate recompact (insert AND remove) on the concurrent-writer mode (in-place stays
   the exclusive/single-writer fast path); ROOT/cross-trie via the §3.4 drain; chain-leaf/list next-mark
   validate (remove 1586 + LIST-EDGE) rides the deferred chain-leaf reader-proxy residual.
6. **(Opt-in optimization layer, LATER)** in-place remove soft-delete + fill-hole insert (sticky bit) +
   masked-validate primitive — a coordinated triple that trades a node copy for count edges on the hot
   path and reintroduces `nr_child` churn, so it takes masked-validate with it.
NOTE: the earlier "P0 = insert `nr_child` committed edge" is DROPPED for the baseline (see the headline
finding) — it can't make in-place safe-append multi-writer-safe (untaggable bitmap) and is unneeded for
single-writer; it only returns inside the FUTURE OPTIMIZATION above.

Validation bar (per commit, same as atomic-detach class): default ft_unit 256 + audit
`-DFT_DEBUG_TOMBSTONE_AUDIT` ft_unit 256 + ft_inv 56/56 on+off + audit+fault 298 + one adversarial skeptic.
Under retained exclusion the guards ALWAYS pass, so the suite proves behavior-identity, NOT the guard's
concurrent correctness — that is argued structurally now and exercised by the Phase 4.3 writer-vs-writer
race oracle later.
