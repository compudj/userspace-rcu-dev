# Blocking DLM locks taken as a sorted set -- future-work design

Status: **future work, not started** (2026-09-28).  Nothing below is
implemented.  Where a statement is a measurement it says so; where it is a
precondition still to verify it says that too.

## 1. Today

* A DLM lock is bit 19 (`FT_STATE_LOCK`) of the node's state word, the word
  that also carries `nr_child`, the tombstone and the proxy tag.  It is taken
  by a CAS (`ft_dlm_lock_now`, `ft_meta_lock_acquire`) and never waited for:
  a word that is locked, tombstoned or parked is **refused**.
* One `ft_dlm_acquire_set_at` call takes its members in ascending anchor
  order, all-or-none: a refusal releases what that call took and returns
  `-EAGAIN`.  The op then bails its attempt, and `ft_txn_attempt_bail` ages it
  (`retry++`, plus once per contended acquire).  Past its budget (11/4 x cost,
  within [64, 4096]) the op enters the escalation lane, a per-trie FIFO fair
  mutex that runs escalated attempts one at a time.  A lane holder refused by
  a word waits a bounded time for that word before retrying (8abc7e79).
* Lock order (05b58ed1, `ft_dlm_acquire_set_at`'s comment): address within
  one take, class across takes -- Rule C, the list locks (class 1) last.
  Class 0 is taken in STAGES (a remove's orphan plan-lock, then the parent
  guard, the recompact and the collapse; a replace's holder, then its parent;
  the root lock after a merge's or graft's node takes).  MEASURED, `ft_inv`
  per-node: about 1.8M of 93M takes land below a class-0 word an earlier stage
  of the same op holds.
* That is deadlock-free only BECAUSE nothing blocks: every take is a try, a
  refusal releases, and the waits that exist (the lane, the refused-word wait)
  happen with nothing held.

## 2. What is wrong with it

A retry today has two unrelated causes, and both age the op toward the lane:

1. **contention** -- a take was refused because a peer holds the word, while
   the plan may be perfectly valid;
2. **a stale plan** -- the op took its locks and its re-validation under them
   failed, because a structural update changed what the plan was derived from.

Only (2) needs a re-plan.  (1) is waiting, expressed as spinning: it burns
attempts (measured on `inv_prefix_pair_compressed_holder` before 8abc7e79: a
remove took 43k attempts in 21 ms against a lock held ~0.5 ms), inflates the
retry count, and sends ops into the lane, which then serializes ops that only
needed to wait for one word.

## 3. Goal (Mathieu, 2026-09-28)

Take each lock set **sorted** and **blocking**.  Contention becomes waiting on
the word; the escalation lane is reached only when structural updates make the
plan fail its re-validation under the locks, and retry.

## 4. Design

### 4.1 A global order, and what blocks

Blocking is deadlock-free only if every op takes its words in one global
order: `(class, address)`.  The staged class-0 takes break it, so:

* a member **above every word the op holds** (class-aware) may **block**;
* a member **below a held word** (a later stage reaching back) stays a
  **try**.  If that try is refused, release everything the attempt holds and
  retake the union sorted and blocking, then re-validate -- or simply bail as
  today; which is cheaper is a measurement (section 5).

Uncontended out-of-order takes still succeed at once, so only a contended one
pays the release-and-retake.  A debug check in the spirit of `FT RULE C` must
assert that no blocking take is out of order: a violation here is a deadlock,
not a livelock.

### 4.2 How to wait

* Waiters must not need a bit in the state word.  The state word is written by
  txn settles (SW parks settle blind, `urcu_txn_settle`), and a settle racing a
  waiter that just set a bit would erase it: a lost wake-up.  Prefer a **side
  wait table**: buckets hashed by the word's address, each with a sleeper count
  and a futex word.  A waiter increments the bucket's count, re-checks the
  lock bit, and sleeps; every unlock of a DLM word (`ft_meta_lock_release`, and
  the staged releases a commit runs after its settle) loads the bucket's count
  and wakes it when non-zero.
* Start with wake-all plus a CAS race among the woken; add per-word hand-off
  only if a hot word starves someone.  The lane stays for livelock of kind (2).

### 4.3 Retirement and parked words

* A holder often retires the node it locked (tombstone).  Its release must
  wake the waiters, who then see `FT_STATE_TOMBSTONE`: a stale plan, re-plan.
* A word parked by a commit (`FT_STATE_PROXY`) is waited out until the settle,
  where today it is refused.

### 4.4 Aging

A take that waited and succeeded does not age the op.  Only a failed
re-validation (and a commit abort) does, so the lane is reached by structural
churn alone.

### 4.5 Preconditions to verify before building

* **No grace-period wait under a held DLM lock.**  Point ops run inside the
  caller's read-side critical section (the tests wrap them in
  `rcu_read_lock`), so a waiter blocks with its read side open.  That only
  delays grace periods -- unless a lock holder waits for one
  (`synchronize_rcu`, `rcu_barrier`, a flavor's synchronize) while holding a
  DLM word, which would deadlock.  Audit every path that holds a word,
  bulk ops and reclaim included.  If one cannot be removed, the wait must be
  bounded and fall back to today's bail.
* **Every DLM release is a wake point.**  Enumerate the release paths: the
  explicit `ft_meta_lock_release`, the commit's staged releases (`rel_after`),
  the txn terminals that release registered holds (`ft_flip_txn_destroy`,
  abort), and any recorded `{LOCK|s -> s}` release a settle performs.  A
  release that does not wake is a hang.
* **Nothing blocks while holding the escalation lane** in a way that the lane
  holder's own peers must release first, or the lane becomes a cycle member.

## 5. Measure first

Split the retries by cause, per op kind and per site, before building:

1. refused take (`ft_acq_refused_word` set, or `acquire_miss`);
2. failed re-validation (the `-EAGAIN` / `-ENOENT` producers after the takes;
   `FT_DBG_EAGAIN` names them);
3. commit abort.

Also split lane entries by the cause that pushed the op over its budget.
Blocking removes (1) and nothing else, so its share bounds the benefit.  The
out-of-order refusal rate (4.1) decides between retake-sorted and bail.

## 6. Steps

1. The measurement of section 5.
2. The precondition audits of 4.5.
3. The side wait table and the wake at every release point.
4. Blocking in-order takes in `ft_dlm_acquire_set_at`, try for out-of-order
   members, and the out-of-order debug check.
5. A/B against today: retries, lane entries, `ft_inv` MW and test 147 times,
   the DNS multi-writer bench; then the rig and the parallel gate.

## 7. Relation to the engine swap

A word held under a blocking lock has one updater until its release, which is
the single-updater premise the SW engine's stores make.  Blocking does not
change what the txns record; it changes when an op gets to record.
