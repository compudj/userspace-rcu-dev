# FT configuration

This began as a plan to cut a 2x2 build matrix whose default quadrant was both
the shipping one and the one carrying the sibling split/compress orphan defect:
that quadrant lacked the §9.2 orphan-chain plan-lock, so a removal could not
detect a peer that grew the orphan. The cut is done, and it was made by deleting
the per-slot-CAS arms rather than hardening them, so the quadrant is
unrepresentable rather than merely fixed. What follows is the configuration as
it stands.

## The one axis

**RUNTIME — writer strategy** (`enum cds_ft_writer_strategy`, 2 values):
* `CDS_FT_WRITER_LOCK_COARSE` — one FT-wide writer mutex.
* `CDS_FT_WRITER_LOCK_FINE` — **the documented default**; per-node lock sets.
* (order statistics coerces FINE -> COARSE, an effective third behaviour.)

`ft->lock_fine` is set in exactly one place, `ft-lifecycle.h:834`, from the
strategy alone.

**There is no compile-time MW axis.** DLM per-node lock-sets are the only
multi-writer implementation; a FINE trie drops the FT-wide writer mutex and a
COARSE trie keeps it, decided at runtime by the strategy above.
`FEATURE_FT_MW_DLM_ACQUIRE`, `FEATURE_FT_MW_LOCK_FINE_DROP` and
`FEATURE_FT_MW_LOCK_FINE_KEEP` no longer exist — each carried an opt-out state
no gate config built, which is how a state rots unseen. Nothing outside a
comment recording their removal references them, so a build recipe or harness
still passing `-DFEATURE_FT_MW_*` selects **nothing**: the flags are inert, not
merely deprecated, and a measurement that believes it varied them did not.

## The gate

`tests/regression/ft_parallel_gate.sh` builds and runs 12 configurations
concurrently, one configured source tree each:

`default`, `fault-audit`, `audit`, `vam`, `txndbg`, `proxyassert`, `noskip`,
`nocompress`, `in-place`, `nokeymap`, `excl`, `nomerge`.

Every one exercises the single MW implementation, so a failure never has to be
attributed to a build mode before it can be diagnosed.

## Flag coverage

Each compile-time flag whose non-default state changes behaviour has a config
that builds it — a flag with no config is untested by construction:

| flag | config |
|---|---|
| `FEATURE_FT_COMPRESS` / `FEATURE_FT_SKIP_COMPRESSED` | `nocompress` / `noskip` |
| `FEATURE_FT_FAULT_INJECT`, `FT_DEBUG_TOMBSTONE_AUDIT` | `fault-audit`, `audit` |
| `FEATURE_FT_VERIFY_AT_MUTATION` | `vam` |
| `FEATURE_FT_INSERT_IN_PLACE` | `in-place` |
| `NO_FEATURE_FT_KEY_MAP` | `nokeymap` |
| `NO_FEATURE_FT_MERGE` | `nomerge` |
| `FEATURE_FT_EXCL_VALIDATE` | `excl` |
| `DEBUG_RCU`, `URCU_TXN_DEBUG_READ_POLICY` (engine) | `txndbg` |
| `FT_DEBUG_PROXY_ASSERT` (resolved-pointer contract) | `proxyassert` |

`FEATURE_FT_ORD_CELL` is **not a build flag** and needs no config: it names the
library-owned ordered sibling list in prose only, and the feature is a runtime
group attribute (`cds_ft_group_attr_set_ordered_list` -> `ft->ordered_list`).

Uncovered by design: `FEATURE_FT_PROBE_*` and `FT_DEBUG_SPLICE_POS_BRACKET` are
instrumentation, not behaviour. `URCU_TXN_DEBUG_DISJOINT` guards
`urcu_txn_declare_disjoint()`, which has no caller in the tree — a config for it
would assert over a workload that cannot reach the contract it checks. The SW
engine's analogue IS reached (`rcu-txn-sw-list.h`'s `_rcu` wrappers declare
disjoint) and its duplicate scan is trapped under `DEBUG_RCU`, which `txndbg`
builds.

## The residual this did not fix

Removing the quadrant closed one of two holes: the sibling key loss also
occurred under DLM. That residual was chased separately and is fixed — a
prune-climb holder-identity defect, `ft_detach_node` resolving `cur` from a
second load of the slot holding the holder, so a peer republish left the climb
walking a node the descent never reached.

Its regression arm is `inv_sibling_split_compress_unpinned`, and it needed its
own: the pinned `inv_sibling_split_compress` passes with and without the fix,
because the static `(p,3,guard)` seed it uses to confine the churn also forbids
the multi-level collapse the defect needs.
